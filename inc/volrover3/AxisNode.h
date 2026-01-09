#ifndef AXISNODE_H
#define AXISNODE_H

#include <volrover3/GraphicsNode.h>
#include <vtkSmartPointer.h>
#include <cvc/bounding_box.h>

class vtkAxesActor;

class AxisNode : public GraphicsNode
{
public:
    AxisNode(const std::string& statePath, const std::string& name = "axis");
    ~AxisNode() override;

    void setAxisLength(double length);

    cvc::bounding_box getBoundingBox() const override;

protected:
    vtkProp* getProp() override;
    void handleStateChanged(const std::string& childState) override;
    void applyTransformToVTK() override;  // Apply transform to axes actor

private:
    vtkSmartPointer<vtkAxesActor> m_axesActor;
};

#endif // AXISNODE_H
