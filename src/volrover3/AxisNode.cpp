#include <volrover3/AxisNode.h>
#include <vtkAxesActor.h>
#include <vtkCaptionActor2D.h>
#include <vtkTextProperty.h>
#include <vtkTextActor.h>

AxisNode::AxisNode()
    : m_axesActor(vtkSmartPointer<vtkAxesActor>::New())
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
