#include <volrover3/GeometryNode.h>
#include <cvc/geometry.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkProperty.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>

GeometryNode::GeometryNode()
    : m_actor(vtkSmartPointer<vtkActor>::New())
    , m_mapper(vtkSmartPointer<vtkPolyDataMapper>::New())
    , m_polyData(vtkSmartPointer<vtkPolyData>::New())
{
    m_mapper->SetInputData(m_polyData);
    m_actor->SetMapper(m_mapper);
    
    // Set default material properties
    m_actor->GetProperty()->SetColor(0.8, 0.8, 0.9);
    m_actor->GetProperty()->SetSpecular(0.3);
    m_actor->GetProperty()->SetSpecularPower(20);
}

GeometryNode::~GeometryNode()
{
}

vtkProp* GeometryNode::getProp()
{
    return m_actor;
}

void GeometryNode::setGeometry(const CVC_NAMESPACE::geometry &geom)
{
    updatePolyData(geom);
}

void GeometryNode::updatePolyData(const CVC_NAMESPACE::geometry &geom)
{
    // Create VTK points from geometry
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    points->SetNumberOfPoints(geom.num_points());

    for (size_t i = 0; i < geom.num_points(); ++i) {
        const auto &pt = geom.points()[i];
        points->SetPoint(i, pt[0], pt[1], pt[2]);
    }

    // Create VTK cells (triangles)
    vtkSmartPointer<vtkCellArray> triangles = vtkSmartPointer<vtkCellArray>::New();
    
    for (size_t i = 0; i < geom.num_tris(); ++i) {
        const auto &tri = geom.tris()[i];
        triangles->InsertNextCell(3);
        triangles->InsertCellPoint(tri[0]);
        triangles->InsertCellPoint(tri[1]);
        triangles->InsertCellPoint(tri[2]);
    }

    // Update polydata
    m_polyData->SetPoints(points);
    m_polyData->SetPolys(triangles);

    // Add normals if available
    if (geom.normals().size() == geom.num_points()) {
        vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New();
        normals->SetNumberOfComponents(3);
        normals->SetNumberOfTuples(geom.num_points());
        normals->SetName("Normals");

        for (size_t i = 0; i < geom.num_points(); ++i) {
            const auto &n = geom.normals()[i];
            normals->SetTuple3(i, n[0], n[1], n[2]);
        }

        m_polyData->GetPointData()->SetNormals(normals);
    }

    // Add colors if available
    if (geom.colors().size() == geom.num_points()) {
        vtkSmartPointer<vtkFloatArray> colors = vtkSmartPointer<vtkFloatArray>::New();
        colors->SetNumberOfComponents(3);
        colors->SetNumberOfTuples(geom.num_points());
        colors->SetName("Colors");

        for (size_t i = 0; i < geom.num_points(); ++i) {
            const auto &c = geom.colors()[i];
            colors->SetTuple3(i, c[0], c[1], c[2]);
        }

        m_polyData->GetPointData()->SetScalars(colors);
    }

    m_polyData->Modified();
}
