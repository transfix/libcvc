#ifndef SCENENODE_H
#define SCENENODE_H

#include <cvc/state_object.h>
#include <functional>
#include <memory>
#include <vector>
#include <vtkSmartPointer.h>

class vtkProp;
class vtkRenderer;
class SceneGraph;

class SceneNode : public cvc::state_object<SceneNode> {
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

  // DEPRECATED: Old callback system - kept for compatibility during transition
  // Use node's SceneGraph::postEvent() instead
  using MainThreadCallback = std::function<void(std::function<void()>)>;
  static void setMainThreadCallback(MainThreadCallback callback);

protected:
  virtual vtkProp *getProp() = 0;
  virtual void handleStateChanged(const std::string &childState) override;

  // Execute a function on the main thread (if callback is set)
  void runOnMainThread(std::function<void()> func);

private:
  static MainThreadCallback s_mainThreadCallback;

protected:
  bool m_visible;
  std::vector<std::shared_ptr<SceneNode>> m_children;
  vtkRenderer *m_renderer;
  SceneGraph *m_sceneGraph; // Non-owning pointer to parent SceneGraph
};

#endif // SCENENODE_H
