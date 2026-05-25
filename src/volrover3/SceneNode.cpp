#include <algorithm>
#include <cvc/core/app.h>
#include <volrover3/SceneGraph.h>
#include <volrover3/SceneNode.h>
#include <vtkProp.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>

// Static member for main thread callback (DEPRECATED - use SceneGraph event queue)
SceneNode::MainThreadCallback SceneNode::s_mainThreadCallback;

void SceneNode::setMainThreadCallback(MainThreadCallback callback) {
  s_mainThreadCallback = callback;
}

void SceneNode::setSceneGraph(SceneGraph *sceneGraph) {
  m_sceneGraph = sceneGraph;

  // Enable threading now that we have a SceneGraph for event posting
  // If sceneGraph is null (cleanup), use static threading setting
  if (sceneGraph) {
    setInstanceThreading(state_object<SceneNode>::getUseThreading());
  } else {
    clearInstanceThreading();
  }

  // Propagate to all children
  for (auto &child : m_children) {
    child->setSceneGraph(sceneGraph);
  }
}

void SceneNode::runOnMainThread(std::function<void()> func) {
  // If threading is disabled (tests or during construction), execute immediately
  if (!getInstanceThreading()) {
    func();
    return;
  }

  // Try node's SceneGraph event queue first (production with threading)
  if (m_sceneGraph) {
    m_sceneGraph->postEvent(std::move(func));
    return;
  }

  // Fallback to old callback system (for Qt-based scenarios without SceneGraph)
  if (s_mainThreadCallback) {
    s_mainThreadCallback(func);
    return;
  }

  // No queue or callback available - execute immediately as last resort
  // This should rarely happen in production
  func();
}

SceneNode::SceneNode(cvc::app &ctx, const std::string &statePath)
    : state_object<SceneNode>(ctx, statePath), m_visible(true), m_renderer(nullptr),
      m_sceneGraph(nullptr) {
  // Disable threading for this instance during construction
  // Will be enabled when SceneGraph reference is set
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

  for (auto &child : m_children) {
    child->setVisible(visible);
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
