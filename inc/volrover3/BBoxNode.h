#ifndef BBOXNODE_H
#define BBOXNODE_H

#include <volrover3/SceneNode.h>
#include <cvc/bounding_box.h>
#include <vtkSmartPointer.h>

class vtkActor;
class vtkPolyDataMapper;

class BBoxNode : public SceneNode
{
public:
    BBoxNode();
    ~BBoxNode() override;

    vtkProp* getProp() override;
    
    void setBoundingBox(const cvc::bounding_box& bbox);
    void setColor(double r, double g, double b);
    void setLineWidth(double width);

private:
    void createBBox();

    vtkSmartPointer<vtkActor> m_actor;
    vtkSmartPointer<vtkPolyDataMapper> m_mapper;
    cvc::bounding_box m_bbox;
};

#endif // BBOXNODE_H
