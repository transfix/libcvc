#ifndef GRIDNODE_H
#define GRIDNODE_H

#include <volrover3/SceneNode.h>
#include <vtkSmartPointer.h>
#include <cvc/bounding_box.h>

class vtkActor;
class vtkPolyDataMapper;

class GridNode : public SceneNode
{
public:
    GridNode();
    ~GridNode() override;

    void setGridSize(double size);
    void setGridSpacing(double spacing);
    void setBounds(const cvc::bounding_box& bounds);

protected:
    vtkProp* getProp() override;

private:
    void createGrid();

    vtkSmartPointer<vtkActor> m_actor;
    vtkSmartPointer<vtkPolyDataMapper> m_mapper;
    
    cvc::bounding_box m_bounds;
    double m_size;
    double m_spacing;
};

#endif // GRIDNODE_H
