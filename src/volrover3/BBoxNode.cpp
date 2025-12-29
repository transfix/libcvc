#include <volrover3/BBoxNode.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkProperty.h>

BBoxNode::BBoxNode()
    : m_actor(vtkSmartPointer<vtkActor>::New())
    , m_mapper(vtkSmartPointer<vtkPolyDataMapper>::New())
    , m_bbox(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0)
{
    m_actor->SetMapper(m_mapper);
    
    // Set default appearance
    m_actor->GetProperty()->SetColor(1.0, 1.0, 0.0); // Yellow
    m_actor->GetProperty()->SetLineWidth(2.0);
    m_actor->GetProperty()->SetOpacity(1.0);

    createBBox();
}

BBoxNode::~BBoxNode()
{
}

vtkProp* BBoxNode::getProp()
{
    return m_actor;
}

void BBoxNode::setBoundingBox(const cvc::bounding_box& bbox)
{
    m_bbox = bbox;
    createBBox();
}

void BBoxNode::setColor(double r, double g, double b)
{
    m_actor->GetProperty()->SetColor(r, g, b);
}

void BBoxNode::setLineWidth(double width)
{
    m_actor->GetProperty()->SetLineWidth(width);
}

void BBoxNode::createBBox()
{
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

    double minX = m_bbox[0];
    double minY = m_bbox[1];
    double minZ = m_bbox[2];
    double maxX = m_bbox[3];
    double maxY = m_bbox[4];
    double maxZ = m_bbox[5];

    // Create 8 corner points
    vtkIdType p0 = points->InsertNextPoint(minX, minY, minZ);
    vtkIdType p1 = points->InsertNextPoint(maxX, minY, minZ);
    vtkIdType p2 = points->InsertNextPoint(maxX, maxY, minZ);
    vtkIdType p3 = points->InsertNextPoint(minX, maxY, minZ);
    vtkIdType p4 = points->InsertNextPoint(minX, minY, maxZ);
    vtkIdType p5 = points->InsertNextPoint(maxX, minY, maxZ);
    vtkIdType p6 = points->InsertNextPoint(maxX, maxY, maxZ);
    vtkIdType p7 = points->InsertNextPoint(minX, maxY, maxZ);

    // Create 12 edges
    vtkIdType edges[12][2] = {
        {p0, p1}, {p1, p2}, {p2, p3}, {p3, p0}, // Bottom face
        {p4, p5}, {p5, p6}, {p6, p7}, {p7, p4}, // Top face
        {p0, p4}, {p1, p5}, {p2, p6}, {p3, p7}  // Vertical edges
    };

    for (int i = 0; i < 12; ++i) {
        lines->InsertNextCell(2);
        lines->InsertCellPoint(edges[i][0]);
        lines->InsertCellPoint(edges[i][1]);
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetLines(lines);

    m_mapper->SetInputData(polyData);
}
