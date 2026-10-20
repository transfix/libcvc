#include <algorithm>
#include <cvc/core/app.h>
#include <cvc/gl/SceneEventSink.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneNode.h>
#include <vtkProp.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>

namespace cvc {
namespace gl {

void SceneNode::setSceneGraph(SceneGraph *sceneGraph) {
  {
    std::lock_guard<std::mutex> lock(m_sceneMutex);
    m_sceneGraph = sceneGraph;
    // Take a weak handle on the scene alongside the raw pointer. The scene resets
    // its token in ~SceneGraph, which is how a node that outlives its scene finds
    // out — see getSceneGraph().
    m_sceneAlive = sceneGraph ? sceneGraph->aliveToken() : std::weak_ptr<void>();
    m_sceneEvents = sceneGraph ? sceneGraph->eventSink() : std::weak_ptr<SceneEventSink>();
    // Whoever attaches or detaches the node is who it belongs to while it has no
    // scene: the scene's owner, which is the only thread that may do either.
    m_homeThread = std::this_thread::get_id();
  }
  // What other threads sent while the node had no scene was held for this: run
  // it now, before the subclass and the children hear of the change. Attaching,
  // this is the owner thread, so it runs inline -- ahead of whatever the caller
  // does to the node next, which is what keeps a stale value from landing after
  // a fresh one.
  runHeld();
  onSceneGraphChanged();

  // Propagate to all children so they marshal through the same pump.
  for (auto &child : m_children) {
    child->setSceneGraph(sceneGraph);
  }
}

SceneGraph *SceneNode::getSceneGraph() const {
  // The back-pointer is only good while the scene that installed it is alive.
  // The lock is uncontended except while the node is being attached, so this
  // stays cheap enough for the pose hot path (GraphicsNode::setPosition consults
  // it per call); it makes the pair read consistent from producer threads.
  std::lock_guard<std::mutex> lock(m_sceneMutex);
  return m_sceneAlive.expired() ? nullptr : m_sceneGraph;
}

std::shared_ptr<SceneEventSink> SceneNode::sceneEvents() const {
  std::lock_guard<std::mutex> lock(m_sceneMutex);
  return m_sceneAlive.expired() ? nullptr : m_sceneEvents.lock();
}

void SceneNode::runOnMainThread(std::function<void()> func) {
  // Through the LOCKED event sink, never a raw SceneGraph*: the scene may be
  // destroyed by its owner while this thread is between the check and the post.
  std::shared_ptr<SceneEventSink> events;
  bool home = false;
  {
    std::lock_guard<std::mutex> lock(m_sceneMutex);
    if (!m_sceneAlive.expired())
      events = m_sceneEvents.lock();
    home = std::this_thread::get_id() == m_homeThread;
  }

  // No scene, so no queue. On the node's home thread -- the one building it, or
  // the one that removed it -- run inline, after anything held for it. From any
  // other thread, hold it: the home thread may be using the node right now.
  // The publisher's worker is such a thread: addGraphicsChild() constructs a
  // node, connecting it to its state path, before it attaches it, and a pose
  // still queued for that path (a removed node of the same name) flushed in
  // between used to run the new node's handlers on the worker, inside VTK,
  // while the owner was attaching it.
  if (!events) {
    if (home) {
      runHeld();
      func();
    } else {
      hold(std::move(func));
    }
    return;
  }

  // On the owner thread: run inline. The node is alive and we are on the
  // render/owner thread, so VTK work happens immediately and in order. This
  // matters for teardown: removing a node runs its removeFromRenderer()
  // synchronously, pulling its prop from the renderer *before* the node is
  // destroyed — never leaving a dangling prop behind.
  if (events->onOwnerThread()) {
    func();
    return;
  }

  // Off the owner thread: marshal to it via the event queue (drained by
  // processEvents()). Guard the callback with a weak_ptr to this node so that if
  // the node is destroyed before the callback runs, it is skipped rather than
  // dereferencing a freed `this`. This keeps cross-thread marshalling safe
  // across teardown: a destroyed node's still-queued callbacks become no-ops.
  std::weak_ptr<SceneNode> weak = weak_from_this();
  std::function<void()> job = [weak, func = std::move(func)]() {
    if (auto self = weak.lock()) {
      func();
    }
  };
  // Refused only once ~SceneGraph has closed the sink -- which it does after
  // dropping its alive token, so this node already has no scene
  // (getSceneGraph() == nullptr) and its work takes the no-scene path, as above:
  // inline on the home thread, held from any other. tryPost leaves `job` with us
  // when it refuses: no copy of `func`.
  if (!events->tryPost(job)) {
    if (home) {
      runHeld();
      job();
    } else {
      hold(std::move(job));
    }
  }
}

void SceneNode::hold(std::function<void()> func) {
  // Bounded, in case nothing ever drains it: a node removed from its scene and
  // kept alive by a script, say, whose path another scene's publisher keeps
  // writing. What is held is almost all state handlers, which read the tree when
  // they run, so the newest entries carry the current values; the oldest go.
  constexpr std::size_t kMaxHeld = 4096;
  std::lock_guard<std::mutex> lock(m_heldMutex);
  if (m_held.size() >= kMaxHeld)
    m_held.pop_front();
  m_held.push_back(std::move(func));
}

void SceneNode::runHeld() {
  std::deque<std::function<void()>> held;
  {
    std::lock_guard<std::mutex> lock(m_heldMutex);
    if (m_held.empty())
      return;
    held.swap(m_held);
  }
  // Each one as if it had just arrived from this thread: inline on the owner or
  // home thread, which is where the callers run this from.
  for (auto &job : held)
    runOnMainThread(std::move(job));
}

SceneNode::SceneNode(cvc::app &ctx, const std::string &statePath)
    : state_object<SceneNode>(ctx, statePath), m_visible(true), m_renderer(nullptr),
      m_sceneGraph(nullptr) {
  // Scene nodes run their state handlers synchronously on the caller's thread —
  // never on a background thread spawned by state_object. This is permanent (it
  // is never re-enabled) and is half of the teardown-safety contract: no library
  // thread can be mid-handleStateChanged() on a node while it is being destroyed.
  // See the class comment in SceneNode.h.
  setInstanceThreading(false);
  // Initialize visible state
  if (!statePath.empty()) {
    getState("visible").value(1); // Default to visible
  }
}

SceneNode::~SceneNode() {
  // Disconnect from state tree before derived class destructor completes
  // to prevent pure virtual method calls during destruction
  disconnectState();
}

void SceneNode::addToRenderer(vtkRenderer *renderer) {
  m_renderer = renderer;
  // Capture the prop pointer before queuing the lambda
  vtkProp *prop = m_visible ? getProp() : nullptr;
  if (prop) {
    // Wrap VTK operation in runOnMainThread
    runOnMainThread([prop, renderer]() { renderer->AddViewProp(prop); });
  }

  for (auto &child : m_children) {
    child->addToRenderer(renderer);
  }
}

void SceneNode::removeFromRenderer(vtkRenderer *renderer) {
  // Capture the prop pointer before queuing the lambda to avoid accessing 'this'
  // after the node might be deleted
  vtkProp *prop = getProp();
  if (prop) {
    // Wrap VTK operation in runOnMainThread
    runOnMainThread([prop, renderer]() { renderer->RemoveViewProp(prop); });
  }

  for (auto &child : m_children) {
    child->removeFromRenderer(renderer);
  }

  m_renderer = nullptr;
}

void SceneNode::update() {
  for (auto &child : m_children) {
    child->update();
  }
}

void SceneNode::setVisible(bool visible) {
  if (m_visible == visible)
    return;

  m_visible = visible;
  // What is drawn changed. A plain node's prop leaves or joins the renderer
  // below, but nodes that toggle VTK visibility in place (GridNode's planes,
  // AxisNode, an LOD node's rungs) change no prop list: either way, tell the
  // scene, so a camera holding still re-fits its clipping range.
  if (SceneGraph *sg = getSceneGraph())
    sg->markContentChanged();

  if (m_renderer && getProp()) {
    // Wrap VTK operations in runOnMainThread for thread safety
    runOnMainThread([this, visible]() {
      vtkProp *prop = getProp();
      if (m_renderer && prop) {
        if (visible) {
          m_renderer->AddViewProp(prop);
        } else {
          m_renderer->RemoveViewProp(prop);
        }
      }
    });
  }

  propagateVisible(visible);
}

void SceneNode::propagateVisible(bool visible) {
  for (auto &child : m_children) {
    pushVisible(*child, visible);
  }
}

void SceneNode::pushVisible(SceneNode &child, bool visible) {
  // A child whose flag already matches keeps it, exactly as a plain setVisible()
  // would; it and its subtree are only told that an ancestor changed.
  if (child.m_visible != visible) {
    child.setVisible(visible);
  } else {
    child.ancestorVisibilityChanged();
  }
}

void SceneNode::ancestorVisibilityChanged() {
  for (auto &child : m_children) {
    child->ancestorVisibilityChanged();
  }
}

void SceneNode::addChild(std::shared_ptr<SceneNode> child) {
  m_children.push_back(child);
  if (m_renderer) {
    child->addToRenderer(m_renderer);
  }
}

void SceneNode::removeChild(std::shared_ptr<SceneNode> child) {
  auto it = std::find(m_children.begin(), m_children.end(), child);
  if (it != m_children.end()) {
    if (m_renderer) {
      (*it)->removeFromRenderer(m_renderer);
    }
    m_children.erase(it);
  }
}

void SceneNode::handleStateChanged(const std::string &childState) {
  // Marshal to main thread via event queue
  runOnMainThread([this, childState]() {
    // Handle visible state changes
    if (childState == "visible") {
      int visible = getState("visible").value<int>();
      setVisible(visible != 0);
    }
  });
}

} // namespace gl
} // namespace cvc
