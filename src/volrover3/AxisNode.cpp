#include <volrover3/AxisNode.h>
#include <vtkAxesActor.h>
#include <vtkCaptionActor2D.h>
#include <vtkTextProperty.h>
#include <vtkTextActor.h>

AxisNode::AxisNode(const std::string& statePath, const std::string& name)
    : GraphicsNode(statePath, name)
    , m_axesActor(vtkSmartPointer<vtkAxesActor>::New())
{
    // Set axis length
    m_axesActor->SetTotalLength(2.0, 2.0, 2.0);
    m_axesActor->SetShaftTypeToLine();
    m_axesActor->SetAxisLabels(1);

    // Configure X axis label
    m_axesActor->GetXAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
    m_axesActor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(20);
    m_axesActor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(1.0, 0.0, 0.0);

    // Configure Y axis label
    m_axesActor->GetYAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
    m_axesActor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(20);
    m_axesActor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(0.0, 1.0, 0.0);

    // Configure Z axis label
    m_axesActor->GetZAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
    m_axesActor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(20);
    m_axesActor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(0.0, 0.0, 1.0);
    
    // Initialize state tree
    if (!statePath.empty()) {
        getState("visible").value(1);  // Visible by default
    }
}

AxisNode::~AxisNode()
{
}

vtkProp* AxisNode::getProp()
{
    return m_axesActor;
}

void AxisNode::setAxisLength(double length)
{
    m_axesActor->SetTotalLength(length, length, length);
}

cvc::bounding_box AxisNode::getBoundingBox() const
{
    // Axis doesn't contribute to scene bounds - it's just a visualization helper
    return cvc::bounding_box(0, 0, 0, 0, 0, 0);
}

void AxisNode::handleStateChanged(const std::string& childState)
{
    // Delegate to parent GraphicsNode for common handling (visible, etc.)
    GraphicsNode::handleStateChanged(childState);
}
