#ifndef SCENENODE_H
#define SCENENODE_H

#include <cvc/core/state_object.h>
#include <functional>
#include <memory>
#include <vector>
#include <vtkSmartPointer.h>

class vtkProp;
class vtkRenderer;

namespace cvc {
namespace gl {

class SceneGraph;

// Lifetime / threading contract
// ------------------------------
// A SceneGraph and its nodes are owned by a single "owner" thread — the thread
// that drives SceneGraph::processEvents() (the main/GUI thread, or the sole
// thread in a headless / SWIG-scripted context). Two rules make node teardown
// race-free in any order, with no joins:
//
//   1. Scene nodes never run their state handlers on background threads.
//      state changes are dispatched synchronously (see SceneNode's ctor), so no
//      library thread can be mid-handleStateChanged() on a node being destroyed.
//   2. Work initiated on the owner thread runs inline (so a removed node's VTK
//      prop is pulled from the renderer before the node dies). Work from any
//      other thread is marshalled through SceneGraph's event queue via
//      runOnMainThread(), and every queued callback is guarded by a weak_ptr to
//      the node — if the node is destroyed before the callback runs it is a
//      no-op instead of a dangling `this`.
//
// SceneNode therefore derives from enable_shared_from_this so runOnMainThread()
// can weak-reference the node. All nodes are created via std::make_shared (see
// GraphicsNode::addGraphicsChild), which wires that up.
class SceneNode : public cvc::state_object<SceneNode>,
                  public std::enable_shared_from_this<SceneNode> {
public:
  SceneNode(cvc::app &ctx, const std::string &statePath);
  virtual ~SceneNode();

  // Access the app context this node is bound to. Subclasses use this when
  // creating child nodes so that the singleton is not consulted.
  cvc::app &app() const { return _ctx; }

  virtual void addToRenderer(vtkRenderer *renderer);
  virtual void removeFromRenderer(vtkRenderer *renderer);
  virtual void update();

  void setVisible(bool visible);
  bool isVisible() const { return m_visible; }

  void addChild(std::shared_ptr<SceneNode> child);
  void removeChild(std::shared_ptr<SceneNode> child);

  // SceneGraph association (set when node is added to a scene graph).
  void setSceneGraph(SceneGraph *sceneGraph);
  // The scene this node belongs to, or nullptr if it has none — never attached,
  // or attached to a scene that has since been destroyed. A node can outlive its
  // scene (removeGraphics() hands ownership back to the caller; a script may
  // keep the node past the scene itself), so the back-pointer alone is not
  // enough: it is paired with a weak handle on the scene's lifetime and only
  // handed out while that handle is live. ALWAYS reach the scene through here.
  SceneGraph *getSceneGraph() const;

  // Public accessor for the node's primary VTK prop. Needed by callers that
  // manage multiple vtkRenderers (e.g. a picture-in-picture overlay) and need
  // to add or remove the node's actor from a second renderer directly. The
  // protected getProp() below stays as the polymorphic implementation hook.
  vtkProp *prop() { return getProp(); }

protected:
  virtual vtkProp *getProp() = 0;
  virtual void handleStateChanged(const std::string &childState) override;

  // Run work on the SceneGraph's owner thread. On the owner thread (or with no
  // SceneGraph attached) it runs inline. From any other thread it is marshalled
  // through the event queue (drained by processEvents()), guarded by a weak_ptr
  // to this node so it is skipped if the node is destroyed before it runs.
  void runOnMainThread(std::function<void()> func);

  bool m_visible;
  std::vector<std::shared_ptr<SceneNode>> m_children;
  vtkRenderer *m_renderer;

private:
  // Non-owning pointer to the parent SceneGraph, meaningful only while
  // m_sceneAlive is unexpired. Private on purpose: getSceneGraph() is the only
  // read that can tell "detached" from "dangling", so subclasses go through it.
  SceneGraph *m_sceneGraph;
  std::weak_ptr<void> m_sceneAlive; // SceneGraph::aliveToken() of m_sceneGraph
};

} // namespace gl
} // namespace cvc

#endif // SCENENODE_H
