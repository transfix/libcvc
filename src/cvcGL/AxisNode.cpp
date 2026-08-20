#include <cvc/gl/AxisNode.h>
#include <vtkAxesActor.h>
#include <vtkCaptionActor2D.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkTransform.h>

AxisNode::AxisNode(cvc::app &ctx, const std::string &statePath, const std::string &name)
    : GraphicsNode(ctx, statePath, name), m_axesActor(vtkSmartPointer<vtkAxesActor>::New()) {
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

  // Initialize state tree with all rendering attributes
  // Use batch scope to prevent callbacks from firing until all values are set
  if (!statePath.empty()) {
    cvc::state_change_batch_scope<SceneNode> batch(*this);

    getState("visible").value(1); // Visible by default

    // Axis length (for all axes)
    getState("axis_length").value(2.0);

    // Shaft type (0=cylinder, 1=line)
    getState("shaft_type_line").value(1);

    // Show axis labels
    getState("show_labels").value(1);

    // Label font size
    getState("label_font_size").value(20);

    // X axis label color (red by default)
    getState("x_label_color_r").value(1.0);
    getState("x_label_color_g").value(0.0);
    getState("x_label_color_b").value(0.0);

    // Y axis label color (green by default)
    getState("y_label_color_r").value(0.0);
    getState("y_label_color_g").value(1.0);
    getState("y_label_color_b").value(0.0);

    // Z axis label color (blue by default)
    getState("z_label_color_r").value(0.0);
    getState("z_label_color_g").value(0.0);
    getState("z_label_color_b").value(1.0);
  } // batch ends here, callbacks fire with all values initialized
}

AxisNode::~AxisNode() {}

void AxisNode::applyTransformToVTK() {
  // Use generic helper to apply world transform
  applyWorldTransformToProps({m_axesActor});
}

vtkProp *AxisNode::getProp() { return m_axesActor; }

void AxisNode::setAxisLength(double length) { getState("axis_length").value(length); }

void AxisNode::setVisible(bool visible) {
  GraphicsNode::setVisible(visible);
  m_axesActor->SetVisibility(visible);
  m_axesActor->GetXAxisCaptionActor2D()->SetVisibility(visible);
  m_axesActor->GetYAxisCaptionActor2D()->SetVisibility(visible);
  m_axesActor->GetZAxisCaptionActor2D()->SetVisibility(visible);
}

cvc::bounding_box AxisNode::getBoundingBox() const {
  // Axis doesn't contribute to scene bounds - it's just a visualization helper
  return cvc::bounding_box(0, 0, 0, 0, 0, 0);
}

void AxisNode::handleStateChanged(const std::string &childState) {
  // Synchronize rendering attributes from state tree
  // All VTK operations MUST be wrapped in runOnMainThread() for thread safety
  if (childState == "axis_length") {
    runOnMainThread([this]() {
      double length = getState("axis_length").value<double>();
      m_axesActor->SetTotalLength(length, length, length);
    });
  } else if (childState == "shaft_type_line") {
    runOnMainThread([this]() {
      bool useLine = getState("shaft_type_line").value<bool>();
      if (useLine) {
        m_axesActor->SetShaftTypeToLine();
      } else {
        m_axesActor->SetShaftTypeToCylinder();
      }
    });
  } else if (childState == "show_labels") {
    runOnMainThread([this]() {
      bool showLabels = getState("show_labels").value<bool>();
      m_axesActor->SetAxisLabels(showLabels ? 1 : 0);
    });
  } else if (childState == "label_font_size") {
    runOnMainThread([this]() {
      int fontSize = getState("label_font_size").value<int>();
      m_axesActor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(fontSize);
      m_axesActor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(fontSize);
      m_axesActor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(fontSize);
    });
  } else if (childState == "x_label_color_r" || childState == "x_label_color_g" ||
             childState == "x_label_color_b") {
    runOnMainThread([this]() {
      try {
        double r = getState("x_label_color_r").value<double>();
        double g = getState("x_label_color_g").value<double>();
        double b = getState("x_label_color_b").value<double>();
        m_axesActor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(r, g, b);
      } catch (const boost::bad_lexical_cast &) {
        // Ignore - values not fully initialized yet
      }
    });
  } else if (childState == "y_label_color_r" || childState == "y_label_color_g" ||
             childState == "y_label_color_b") {
    runOnMainThread([this]() {
      try {
        double r = getState("y_label_color_r").value<double>();
        double g = getState("y_label_color_g").value<double>();
        double b = getState("y_label_color_b").value<double>();
        m_axesActor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(r, g, b);
      } catch (const boost::bad_lexical_cast &) {
        // Ignore - values not fully initialized yet
      }
    });
  } else if (childState == "z_label_color_r" || childState == "z_label_color_g" ||
             childState == "z_label_color_b") {
    runOnMainThread([this]() {
      try {
        double r = getState("z_label_color_r").value<double>();
        double g = getState("z_label_color_g").value<double>();
        double b = getState("z_label_color_b").value<double>();
        m_axesActor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(r, g, b);
      } catch (const boost::bad_lexical_cast &) {
        // Ignore - values not fully initialized yet
      }
    });
  } else {
    // Delegate to parent for common fields (visible, show_bbox, label, etc.)
    GraphicsNode::handleStateChanged(childState);
  }
}
