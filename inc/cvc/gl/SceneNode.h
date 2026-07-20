#ifndef SCENENODE_H
#define SCENENODE_H

#include <cvc/core/state_object.h>
#include <functional>
#include <memory>
#include <vector>
#include <vtkSmartPointer.h>

class vtkProp;
class vtkRenderer;
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

  // SceneGraph association (set when node is added to a scene graph)
  void setSceneGraph(SceneGraph *sceneGraph);
  SceneGraph *getSceneGraph() const { return m_sceneGraph; }

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
  SceneGraph *m_sceneGraph; // Non-owning pointer to parent SceneGraph
};

#endif // SCENENODE_H
