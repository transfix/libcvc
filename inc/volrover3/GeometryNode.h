#ifndef GEOMETRYNODE_H
#define GEOMETRYNODE_H

#include <volrover3/SceneNode.h>
#include <vtkSmartPointer.h>

class vtkActor;
class vtkPolyDataMapper;
class vtkPolyData;

namespace cvc {
    class geometry;
}

class GeometryNode : public SceneNode
{
public:
    GeometryNode();
    ~GeometryNode() override;

    void setGeometry(const cvc::geometry &geom);

protected:
    vtkProp* getProp() override;

private:
    void updatePolyData(const cvc::geometry &geom);

    vtkSmartPointer<vtkActor> m_actor;
    vtkSmartPointer<vtkPolyDataMapper> m_mapper;
    vtkSmartPointer<vtkPolyData> m_polyData;
};

#endif // GEOMETRYNODE_H
