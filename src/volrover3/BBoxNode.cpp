#include <volrover3/BBoxNode.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkActor2D.h>
#include <vtkTextMapper.h>
#include <vtkTextProperty.h>
#include <vtkCoordinate.h>
#include <sstream>
#include <iomanip>
#include <cmath>

BBoxNode::BBoxNode()
    : m_actor(vtkSmartPointer<vtkActor>::New())
    , m_mapper(vtkSmartPointer<vtkPolyDataMapper>::New())
    , m_bbox(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0)
    , m_ticksVisible(false)
    , m_tickInterval(1.0)
    , m_tickLabelFontSize(12)
{
    m_actor->SetMapper(m_mapper);
    
    // Set default appearance
    m_actor->GetProperty()->SetColor(1.0, 1.0, 0.0); // Yellow
    m_actor->GetProperty()->SetLineWidth(2.0);
    m_actor->GetProperty()->SetOpacity(1.0);
    
    // Default tick label color (white)
    m_tickLabelColor[0] = m_tickLabelColor[1] = m_tickLabelColor[2] = 1.0;

    createBBox();
}

BBoxNode::~BBoxNode()
{
}

vtkProp* BBoxNode::getProp()
{
    return m_actor;
}

void BBoxNode::addToRenderer(vtkRenderer* renderer)
{
    if (renderer && isVisible()) {
        m_renderer = renderer;  // Store renderer reference
        renderer->AddActor(m_actor);
        if (m_ticksVisible) {
            for (auto& actor : m_tickLabelActors) {
                renderer->AddActor2D(actor);
            }
        }
    }
}

void BBoxNode::removeFromRenderer(vtkRenderer* renderer)
{
    if (renderer) {
        renderer->RemoveActor(m_actor);
        for (auto& actor : m_tickLabelActors) {
            renderer->RemoveActor2D(actor);
        }
        if (renderer == m_renderer) {
            m_renderer = nullptr;
        }
    }
}

void BBoxNode::setBoundingBox(const cvc::bounding_box& bbox)
{
    m_bbox = bbox;
    createBBox();
    createTickLabels();
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

void BBoxNode::setTicksVisible(bool visible)
{
    m_ticksVisible = visible;
    for (auto& actor : m_tickLabelActors) {
        actor->SetVisibility(visible);
    }
}

void BBoxNode::setTickInterval(double interval)
{
    m_tickInterval = interval;
    createTickLabels();
}

void BBoxNode::setTickLabelColor(double r, double g, double b)
{
    m_tickLabelColor[0] = r;
    m_tickLabelColor[1] = g;
    m_tickLabelColor[2] = b;
    
    for (auto& actor : m_tickLabelActors) {
        vtkTextMapper* mapper = vtkTextMapper::SafeDownCast(actor->GetMapper());
        if (mapper) {
            mapper->GetTextProperty()->SetColor(r, g, b);
        }
    }
}

void BBoxNode::getTickLabelColor(double& r, double& g, double& b) const
{
    r = m_tickLabelColor[0];
    g = m_tickLabelColor[1];
    b = m_tickLabelColor[2];
}

void BBoxNode::setTickLabelFontSize(int size)
{
    m_tickLabelFontSize = std::max(1, size);
    
    for (auto& actor : m_tickLabelActors) {
        vtkTextMapper* mapper = vtkTextMapper::SafeDownCast(actor->GetMapper());
        if (mapper) {
            mapper->GetTextProperty()->SetFontSize(m_tickLabelFontSize);
        }
    }
}

void BBoxNode::createTickLabels()
{
    // Remove old labels from renderer first
    if (m_renderer) {
        for (auto& actor : m_tickLabelActors) {
            // Only remove if actor was actually added to a renderer
            if (actor->GetReferenceCount() > 1) {
                m_renderer->RemoveActor2D(actor);
            }
        }
    }
    
    // Clear existing labels
    m_tickLabelActors.clear();
    
    if (!m_ticksVisible) return;
    
    double minX = m_bbox[0];
    double minY = m_bbox[1];
    double minZ = m_bbox[2];
    double maxX = m_bbox[3];
    double maxY = m_bbox[4];
    double maxZ = m_bbox[5];
    
    double spanX = maxX - minX;
    double spanY = maxY - minY;
    double spanZ = maxZ - minZ;
    
    if (spanX <= 0.0 || spanY <= 0.0 || spanZ <= 0.0) return;
    
    // Helper lambda to create a label
    auto createLabel = [&](double x, double y, double z, const std::string& text) {
        vtkSmartPointer<vtkTextMapper> textMapper = vtkSmartPointer<vtkTextMapper>::New();
        textMapper->SetInput(text.c_str());
        textMapper->GetTextProperty()->SetFontSize(m_tickLabelFontSize);
        textMapper->GetTextProperty()->SetColor(m_tickLabelColor);
        textMapper->GetTextProperty()->SetJustificationToCentered();
        textMapper->GetTextProperty()->SetVerticalJustificationToCentered();
        
        vtkSmartPointer<vtkActor2D> textActor = vtkSmartPointer<vtkActor2D>::New();
        textActor->SetMapper(textMapper);
        textActor->GetPositionCoordinate()->SetCoordinateSystemToWorld();
        textActor->GetPositionCoordinate()->SetValue(x, y, z);
        textActor->SetVisibility(m_ticksVisible);
        
        m_tickLabelActors.push_back(textActor);
    };
    
    // If interval is 0 or negative, only show coordinates at the 8 vertices
    if (m_tickInterval <= 0.0) {
        std::ostringstream oss;
        // 8 vertices of the bounding box
        std::vector<std::tuple<double, double, double>> vertices = {
            {minX, minY, minZ}, {maxX, minY, minZ},
            {minX, maxY, minZ}, {maxX, maxY, minZ},
            {minX, minY, maxZ}, {maxX, minY, maxZ},
            {minX, maxY, maxZ}, {maxX, maxY, maxZ}
        };
        for (const auto& [x, y, z] : vertices) {
            oss.str("");
            oss << "(" << std::fixed << std::setprecision(2) << x << ", " << y << ", " << z << ")";
            createLabel(x, y, z, oss.str());
        }
        return;
    }
    
    // Otherwise use interval-based ticks along edges
    // X-axis labels along bottom-front edge (minY, minZ)
    for (double x = minX; x <= maxX; x += m_tickInterval) {
        if (x > maxX + 1e-6) break; // Avoid floating point overshoot
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << x;
        createLabel(x, minY, minZ, oss.str());
    }
    
    // Y-axis labels along left-front edge (minX, minZ)
    for (double y = minY + m_tickInterval; y <= maxY; y += m_tickInterval) {
        if (y > maxY + 1e-6) break;
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << y;
        createLabel(minX, y, minZ, oss.str());
    }
    
    // Z-axis labels along left-bottom edge (minX, minY)
    for (double z = minZ + m_tickInterval; z <= maxZ; z += m_tickInterval) {
        if (z > maxZ + 1e-6) break;
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << z;
        createLabel(minX, minY, z, oss.str());
    }
    
    // Add new labels to renderer if we have one and ticks are visible
    if (m_renderer && m_ticksVisible && isVisible()) {
        for (auto& actor : m_tickLabelActors) {
            m_renderer->AddActor2D(actor);
        }
    }
}

