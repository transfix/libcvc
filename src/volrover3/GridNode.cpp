#include <volrover3/GridNode.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkProperty.h>
#include <algorithm>
#include <cmath>

GridNode::GridNode()
    : m_actor(vtkSmartPointer<vtkActor>::New())
    , m_mapper(vtkSmartPointer<vtkPolyDataMapper>::New())
    , m_bounds(-10.0, -10.0, -10.0, 10.0, 10.0, 10.0)
    , m_size(10.0)
    , m_spacing(1.0)
{
    m_actor->SetMapper(m_mapper);
    
    // Set grid appearance
    m_actor->GetProperty()->SetColor(0.3, 0.3, 0.3);
    m_actor->GetProperty()->SetLineWidth(1.0);
    m_actor->GetProperty()->SetOpacity(0.5);

    createGrid();
}

GridNode::~GridNode()
{
}

vtkProp* GridNode::getProp()
{
    return m_actor;
}

void GridNode::setGridSize(double size)
{
    m_size = size;
    createGrid();
}

void GridNode::setGridSpacing(double spacing)
{
    m_spacing = spacing;
    createGrid();
}

void GridNode::setBounds(const cvc::bounding_box& bounds)
{
    m_bounds = bounds;
    createGrid();
}

void GridNode::createGrid()
{
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

    // Get bounding box dimensions
    double minX = m_bounds[0];
    double minY = m_bounds[1];
    double minZ = m_bounds[2];
    double maxX = m_bounds[3];
    double maxY = m_bounds[4];
    double maxZ = m_bounds[5];
    
    double spanX = maxX - minX;
    double spanY = maxY - minY;
    double spanZ = maxZ - minZ;
    
    // Auto-calculate spacing based on largest dimension (aim for ~20 divisions)
    double maxSpan = std::max({spanX, spanY, spanZ});
    double spacing = maxSpan / 20.0;
    if (spacing <= 0.0) spacing = 1.0;
    
    // Round spacing to nearest nice value
    double magnitude = std::pow(10.0, std::floor(std::log10(spacing)));
    double normalized = spacing / magnitude;
    if (normalized < 1.5) spacing = magnitude;
    else if (normalized < 3.5) spacing = 2.0 * magnitude;
    else if (normalized < 7.5) spacing = 5.0 * magnitude;
    else spacing = 10.0 * magnitude;

    // Create grid lines on XY plane (at minZ)
    for (double x = std::floor(minX / spacing) * spacing; x <= maxX; x += spacing)
    {
        vtkIdType id1 = points->InsertNextPoint(x, minY, minZ);
        vtkIdType id2 = points->InsertNextPoint(x, maxY, minZ);
        lines->InsertNextCell(2);
        lines->InsertCellPoint(id1);
        lines->InsertCellPoint(id2);
    }
    for (double y = std::floor(minY / spacing) * spacing; y <= maxY; y += spacing)
    {
        vtkIdType id1 = points->InsertNextPoint(minX, y, minZ);
        vtkIdType id2 = points->InsertNextPoint(maxX, y, minZ);
        lines->InsertNextCell(2);
        lines->InsertCellPoint(id1);
        lines->InsertCellPoint(id2);
    }

    // Create grid lines on XZ plane (at minY)
    for (double x = std::floor(minX / spacing) * spacing; x <= maxX; x += spacing)
    {
        vtkIdType id1 = points->InsertNextPoint(x, minY, minZ);
        vtkIdType id2 = points->InsertNextPoint(x, minY, maxZ);
        lines->InsertNextCell(2);
        lines->InsertCellPoint(id1);
        lines->InsertCellPoint(id2);
    }
    for (double z = std::floor(minZ / spacing) * spacing; z <= maxZ; z += spacing)
    {
        vtkIdType id1 = points->InsertNextPoint(minX, minY, z);
        vtkIdType id2 = points->InsertNextPoint(maxX, minY, z);
        lines->InsertNextCell(2);
        lines->InsertCellPoint(id1);
        lines->InsertCellPoint(id2);
    }

    // Create grid lines on YZ plane (at minX)
    for (double y = std::floor(minY / spacing) * spacing; y <= maxY; y += spacing)
    {
        vtkIdType id1 = points->InsertNextPoint(minX, y, minZ);
        vtkIdType id2 = points->InsertNextPoint(minX, y, maxZ);
        lines->InsertNextCell(2);
        lines->InsertCellPoint(id1);
        lines->InsertCellPoint(id2);
    }
    for (double z = std::floor(minZ / spacing) * spacing; z <= maxZ; z += spacing)
    {
        vtkIdType id1 = points->InsertNextPoint(minX, minY, z);
        vtkIdType id2 = points->InsertNextPoint(minX, maxY, z);
        lines->InsertNextCell(2);
        lines->InsertCellPoint(id1);
        lines->InsertCellPoint(id2);
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetLines(lines);

    m_mapper->SetInputData(polyData);
}
