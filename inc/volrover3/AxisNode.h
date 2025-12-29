#ifndef AXISNODE_H
#define AXISNODE_H

#include <volrover3/SceneNode.h>
#include <vtkSmartPointer.h>

class vtkAxesActor;

class AxisNode : public SceneNode
{
public:
    AxisNode();
    ~AxisNode() override;

    void setAxisLength(double length);

protected:
    vtkProp* getProp() override;

private:
    vtkSmartPointer<vtkAxesActor> m_axesActor;
};

#endif // AXISNODE_H
