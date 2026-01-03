#ifndef SCENENODE_H
#define SCENENODE_H

#include <cvc/state_object.h>
#include <vtkSmartPointer.h>
#include <memory>
#include <vector>
#include <functional>

class vtkProp;
class vtkRenderer;

class SceneNode : public CVC_NAMESPACE::state_object<SceneNode>
{
public:
    SceneNode(const std::string& statePath);
    virtual ~SceneNode();

    virtual void addToRenderer(vtkRenderer *renderer);
    virtual void removeFromRenderer(vtkRenderer *renderer);
    virtual void update();

    void setVisible(bool visible);
    bool isVisible() const { return m_visible; }

    void addChild(std::shared_ptr<SceneNode> child);
    void removeChild(std::shared_ptr<SceneNode> child);

    // Set a callback to execute state changes on the main thread
    // The callback receives a function to execute on the main thread
    using MainThreadCallback = std::function<void(std::function<void()>)>;
    static void setMainThreadCallback(MainThreadCallback callback);

protected:
    virtual vtkProp* getProp() = 0;
    virtual void handleStateChanged(const std::string& childState) override;
    
    // Execute a function on the main thread (if callback is set)
    void runOnMainThread(std::function<void()> func);

private:
    static MainThreadCallback s_mainThreadCallback;

protected:
    bool m_visible;
    std::vector<std::shared_ptr<SceneNode>> m_children;
    vtkRenderer *m_renderer;
};

#endif // SCENENODE_H
