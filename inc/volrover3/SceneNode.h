#ifndef SCENENODE_H
#define SCENENODE_H

#include <vtkSmartPointer.h>
#include <memory>
#include <vector>

class vtkProp;
class vtkRenderer;

class SceneNode
{
public:
    SceneNode();
    virtual ~SceneNode();

    virtual void addToRenderer(vtkRenderer *renderer);
    virtual void removeFromRenderer(vtkRenderer *renderer);
    virtual void update();

    void setVisible(bool visible);
    bool isVisible() const { return m_visible; }

    void addChild(std::shared_ptr<SceneNode> child);
    void removeChild(std::shared_ptr<SceneNode> child);

protected:
    virtual vtkProp* getProp() = 0;

    bool m_visible;
    std::vector<std::shared_ptr<SceneNode>> m_children;
    vtkRenderer *m_renderer;
};

#endif // SCENENODE_H
