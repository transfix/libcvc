#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <volrover3/AppState.h>
#include <volrover3/GridNode.h>
#include <vtkActor.h>
#include <vtkActor2D.h>
#include <vtkCellArray.h>
#include <vtkCoordinate.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkTextMapper.h>
#include <vtkTextProperty.h>
#include <vtkTransform.h>

GridNode::GridNode(cvc::app &ctx, const std::string &statePath, const std::string &name)
    : GraphicsNode(ctx, statePath, name), m_yzActor(vtkSmartPointer<vtkActor>::New()),
      m_xzActor(vtkSmartPointer<vtkActor>::New()), m_xyActor(vtkSmartPointer<vtkActor>::New()),
      m_yzMapper(vtkSmartPointer<vtkPolyDataMapper>::New()),
      m_xzMapper(vtkSmartPointer<vtkPolyDataMapper>::New()),
      m_xyMapper(vtkSmartPointer<vtkPolyDataMapper>::New()),
      m_bounds(-10.0, -10.0, -10.0, 10.0, 10.0, 10.0), m_divisionsX(64), m_divisionsY(64),
      m_divisionsZ(64), m_tickIntervalX(8), m_tickIntervalY(8), m_tickIntervalZ(8),
      m_tickLabelFontSize(12), m_yzPlaneVisible(true), m_xzPlaneVisible(true),
      m_xyPlaneVisible(true), m_renderer(nullptr) {
  // Initialize default colors
  m_yzPlaneColor[0] = m_yzPlaneColor[1] = m_yzPlaneColor[2] = 0.5;
  m_xzPlaneColor[0] = m_xzPlaneColor[1] = m_xzPlaneColor[2] = 0.5;
  m_xyPlaneColor[0] = m_xyPlaneColor[1] = m_xyPlaneColor[2] = 0.5;
  m_tickLabelColor[0] = m_tickLabelColor[1] = m_tickLabelColor[2] = 1.0; // White

  // Setup YZ plane actor (at X=0)
  m_yzActor->SetMapper(m_yzMapper);
  m_yzActor->GetProperty()->SetColor(m_yzPlaneColor);
  m_yzActor->GetProperty()->SetLineWidth(1.0);
  m_yzActor->GetProperty()->SetOpacity(0.5);

  // Setup XZ plane actor (at Y=0)
  m_xzActor->SetMapper(m_xzMapper);
  m_xzActor->GetProperty()->SetColor(m_xzPlaneColor);
  m_xzActor->GetProperty()->SetLineWidth(1.0);
  m_xzActor->GetProperty()->SetOpacity(0.5);

  // Setup XY plane actor (at Z=0)
  m_xyActor->SetMapper(m_xyMapper);
  m_xyActor->GetProperty()->SetColor(m_xyPlaneColor);
  m_xyActor->GetProperty()->SetLineWidth(1.0);
  m_xyActor->GetProperty()->SetOpacity(0.5);

  // Initialize state tree with all rendering attributes
  // Use batch scope to prevent callbacks from firing until all values are set
  if (!statePath.empty()) {
    cvc::state_change_batch_scope<SceneNode> batch(*this);

    getState("visible").value(1); // Visible by default

    // YZ Plane (organized under yz_plane. hierarchy)
    getState("yz_plane.visible").value(1);
    getState("yz_plane.color_r").value(0.5);
    getState("yz_plane.color_g").value(0.5);
    getState("yz_plane.color_b").value(0.5);
    getState("yz_plane.line_width").value(1.0);
    getState("yz_plane.opacity").value(0.5);

    // XZ Plane (organized under xz_plane. hierarchy)
    getState("xz_plane.visible").value(1);
    getState("xz_plane.color_r").value(0.5);
    getState("xz_plane.color_g").value(0.5);
    getState("xz_plane.color_b").value(0.5);
    getState("xz_plane.line_width").value(1.0);
    getState("xz_plane.opacity").value(0.5);

    // XY Plane (organized under xy_plane. hierarchy)
    getState("xy_plane.visible").value(1);
    getState("xy_plane.color_r").value(0.5);
    getState("xy_plane.color_g").value(0.5);
    getState("xy_plane.color_b").value(0.5);
    getState("xy_plane.line_width").value(1.0);
    getState("xy_plane.opacity").value(0.5);

    // Grid divisions
    getState("divisions_x").value(64);
    getState("divisions_y").value(64);
    getState("divisions_z").value(64);

    // Tick properties (organized under tics. hierarchy)
    getState("tics.interval_x").value(8);
    getState("tics.interval_y").value(8);
    getState("tics.interval_z").value(8);

    getState("tics.label_color_r").value(1.0);
    getState("tics.label_color_g").value(1.0);
    getState("tics.label_color_b").value(1.0);
    getState("tics.label_font_size").value(12);

    // Tick visibility (hidden by default)
    getState("tics.visible").value(0);
  } // batch ends here, callbacks fire with all values initialized

  createGridPlanes();
}

GridNode::~GridNode() {}

void GridNode::applyTransformToVTK() {
  // Use generic helper to apply world transform to all three grid actors
  applyWorldTransformToProps({m_yzActor, m_xzActor, m_xyActor});
}

void GridNode::applyClipPlanes(vtkPlaneCollection *planes) {
  // Apply clip planes to all three grid mappers
  if (planes && planes->GetNumberOfItems() > 0) {
    if (m_yzMapper)
      m_yzMapper->SetClippingPlanes(planes);
    if (m_xzMapper)
      m_xzMapper->SetClippingPlanes(planes);
    if (m_xyMapper)
      m_xyMapper->SetClippingPlanes(planes);
  } else {
    if (m_yzMapper)
      m_yzMapper->RemoveAllClippingPlanes();
    if (m_xzMapper)
      m_xzMapper->RemoveAllClippingPlanes();
    if (m_xyMapper)
      m_xyMapper->RemoveAllClippingPlanes();
  }
}

vtkProp *GridNode::getProp() {
  // Return first actor for compatibility with base class
  return m_yzActor;
}

void GridNode::addToRenderer(vtkRenderer *renderer) {
  if (renderer && isVisible()) {
    m_renderer = renderer; // Store renderer reference
    if (m_yzPlaneVisible) {
      renderer->AddActor(m_yzActor);
      for (auto &actor : m_yzTickLabelActors) {
        renderer->AddViewProp(actor);
      }
    }
    if (m_xzPlaneVisible) {
      renderer->AddActor(m_xzActor);
      for (auto &actor : m_xzTickLabelActors) {
        renderer->AddViewProp(actor);
      }
    }
    if (m_xyPlaneVisible) {
      renderer->AddActor(m_xyActor);
      for (auto &actor : m_xyTickLabelActors) {
        renderer->AddViewProp(actor);
      }
    }
  }
}

void GridNode::removeFromRenderer(vtkRenderer *renderer) {
  if (renderer) {
    renderer->RemoveActor(m_yzActor);
    renderer->RemoveActor(m_xzActor);
    renderer->RemoveActor(m_xyActor);

    for (auto &actor : m_yzTickLabelActors) {
      renderer->RemoveViewProp(actor);
    }
    for (auto &actor : m_xzTickLabelActors) {
      renderer->RemoveViewProp(actor);
    }
    for (auto &actor : m_xyTickLabelActors) {
      renderer->RemoveViewProp(actor);
    }

    m_renderer = nullptr; // Clear renderer reference
  }
}

void GridNode::setBounds(const cvc::bounding_box &bounds) {
  std::cout << "[DEBUG] GridNode::setBounds called with bounds: [" << bounds[0] << "," << bounds[1]
            << "," << bounds[2] << "] to [" << bounds[3] << "," << bounds[4] << "," << bounds[5]
            << "]" << std::endl;
  m_bounds = bounds;
  createGridPlanes();
  updateTickLabelsInRenderer();
}

void GridNode::setColor(double r, double g, double b) {
  setYZPlaneColor(r, g, b);
  setXZPlaneColor(r, g, b);
  setXYPlaneColor(r, g, b);
}

cvc::bounding_box GridNode::getBoundingBox() const {
  // Grid doesn't contribute to scene bounds - it's just a visualization
  // helper
  return cvc::bounding_box(0, 0, 0, 0, 0, 0);
}

void GridNode::handleStateChanged(const std::string &childState) {
  // Synchronize rendering attributes from state tree
  // All VTK operations MUST be wrapped in runOnMainThread() for thread safety
  if (childState == "yz_plane.visible") {
    runOnMainThread([this]() {
      m_yzPlaneVisible = getState("yz_plane.visible").value<bool>();
      m_yzActor->SetVisibility(m_yzPlaneVisible);
      for (auto &actor : m_yzTickLabelActors) {
        actor->SetVisibility(m_yzPlaneVisible);
      }
    });
  } else if (childState == "xz_plane.visible") {
    runOnMainThread([this]() {
      m_xzPlaneVisible = getState("xz_plane.visible").value<bool>();
      m_xzActor->SetVisibility(m_xzPlaneVisible);
      for (auto &actor : m_xzTickLabelActors) {
        actor->SetVisibility(m_xzPlaneVisible);
      }
    });
  } else if (childState == "xy_plane.visible") {
    runOnMainThread([this]() {
      m_xyPlaneVisible = getState("xy_plane.visible").value<bool>();
      m_xyActor->SetVisibility(m_xyPlaneVisible);
      for (auto &actor : m_xyTickLabelActors) {
        actor->SetVisibility(m_xyPlaneVisible);
      }
    });
  } else if (childState == "yz_plane.color_r" || childState == "yz_plane.color_g" ||
             childState == "yz_plane.color_b") {
    runOnMainThread([this]() {
      // Only update if all color components can be read
      try {
        m_yzPlaneColor[0] = getState("yz_plane.color_r").value<double>();
        m_yzPlaneColor[1] = getState("yz_plane.color_g").value<double>();
        m_yzPlaneColor[2] = getState("yz_plane.color_b").value<double>();
        m_yzActor->GetProperty()->SetColor(m_yzPlaneColor);
      } catch (const boost::bad_lexical_cast &) {
        // Ignore - values not fully initialized yet
      }
    });
  } else if (childState == "yz_plane.line_width") {
    runOnMainThread([this]() {
      double lineWidth = getState("yz_plane.line_width").value<double>();
      m_yzActor->GetProperty()->SetLineWidth(lineWidth);
    });
  } else if (childState == "yz_plane.opacity") {
    runOnMainThread([this]() {
      double opacity = getState("yz_plane.opacity").value<double>();
      m_yzActor->GetProperty()->SetOpacity(opacity);
    });
  } else if (childState == "xz_plane.color_r" || childState == "xz_plane.color_g" ||
             childState == "xz_plane.color_b") {
    runOnMainThread([this]() {
      try {
        m_xzPlaneColor[0] = getState("xz_plane.color_r").value<double>();
        m_xzPlaneColor[1] = getState("xz_plane.color_g").value<double>();
        m_xzPlaneColor[2] = getState("xz_plane.color_b").value<double>();
        m_xzActor->GetProperty()->SetColor(m_xzPlaneColor);
      } catch (const boost::bad_lexical_cast &) {
        // Ignore - values not fully initialized yet
      }
    });
  } else if (childState == "xz_plane.line_width") {
    runOnMainThread([this]() {
      double lineWidth = getState("xz_plane.line_width").value<double>();
      m_xzActor->GetProperty()->SetLineWidth(lineWidth);
    });
  } else if (childState == "xz_plane.opacity") {
    runOnMainThread([this]() {
      double opacity = getState("xz_plane.opacity").value<double>();
      m_xzActor->GetProperty()->SetOpacity(opacity);
    });
  } else if (childState == "xy_plane.color_r" || childState == "xy_plane.color_g" ||
             childState == "xy_plane.color_b") {
    runOnMainThread([this]() {
      try {
        m_xyPlaneColor[0] = getState("xy_plane.color_r").value<double>();
        m_xyPlaneColor[1] = getState("xy_plane.color_g").value<double>();
        m_xyPlaneColor[2] = getState("xy_plane.color_b").value<double>();
        m_xyActor->GetProperty()->SetColor(m_xyPlaneColor);
      } catch (const boost::bad_lexical_cast &) {
        // Ignore - values not fully initialized yet
      }
    });
  } else if (childState == "xy_plane.line_width") {
    runOnMainThread([this]() {
      double lineWidth = getState("xy_plane.line_width").value<double>();
      m_xyActor->GetProperty()->SetLineWidth(lineWidth);
    });
  } else if (childState == "xy_plane.opacity") {
    runOnMainThread([this]() {
      double opacity = getState("xy_plane.opacity").value<double>();
      m_xyActor->GetProperty()->SetOpacity(opacity);
    });
  } else if (childState == "divisions_x" || childState == "divisions_y" ||
             childState == "divisions_z") {
    runOnMainThread([this]() {
      try {
        m_divisionsX = std::max(1, getState("divisions_x").value<int>());
        m_divisionsY = std::max(1, getState("divisions_y").value<int>());
        m_divisionsZ = std::max(1, getState("divisions_z").value<int>());
        createGridPlanes();
        updateTickLabelsInRenderer();
      } catch (const boost::bad_lexical_cast &) {
        // Ignore - values not fully initialized yet
      }
    });
  } else if (childState == "tics.interval_x" || childState == "tics.interval_y" ||
             childState == "tics.interval_z") {
    runOnMainThread([this]() {
      try {
        m_tickIntervalX = std::max(1, getState("tics.interval_x").value<int>());
        m_tickIntervalY = std::max(1, getState("tics.interval_y").value<int>());
        m_tickIntervalZ = std::max(1, getState("tics.interval_z").value<int>());
        updateTickLabelsInRenderer();
      } catch (const boost::bad_lexical_cast &) {
        // Ignore - values not fully initialized yet
      }
    });
  } else if (childState == "tics.label_color_r" || childState == "tics.label_color_g" ||
             childState == "tics.label_color_b") {
    runOnMainThread([this]() {
      try {
        m_tickLabelColor[0] = getState("tics.label_color_r").value<double>();
        m_tickLabelColor[1] = getState("tics.label_color_g").value<double>();
        m_tickLabelColor[2] = getState("tics.label_color_b").value<double>();

        // Update all existing labels
        auto updateLabels = [&](std::vector<vtkSmartPointer<vtkActor2D>> &actors) {
          for (auto &actor : actors) {
            vtkTextMapper *mapper = vtkTextMapper::SafeDownCast(actor->GetMapper());
            if (mapper) {
              mapper->GetTextProperty()->SetColor(m_tickLabelColor);
            }
          }
        };

        updateLabels(m_yzTickLabelActors);
        updateLabels(m_xzTickLabelActors);
        updateLabels(m_xyTickLabelActors);
      } catch (const boost::bad_lexical_cast &) {
        // Ignore - values not fully initialized yet
      }
    });
  } else if (childState == "tics.label_font_size") {
    runOnMainThread([this]() {
      m_tickLabelFontSize = std::max(1, getState("tics.label_font_size").value<int>());

      // Update all existing labels
      auto updateLabels = [&](std::vector<vtkSmartPointer<vtkActor2D>> &actors) {
        for (auto &actor : actors) {
          vtkTextMapper *mapper = vtkTextMapper::SafeDownCast(actor->GetMapper());
          if (mapper) {
            mapper->GetTextProperty()->SetFontSize(m_tickLabelFontSize);
          }
        }
      };

      updateLabels(m_yzTickLabelActors);
      updateLabels(m_xzTickLabelActors);
      updateLabels(m_xyTickLabelActors);
    });
  } else if (childState == "tics.visible") {
    runOnMainThread([this]() {
      // Update tick label visibility in renderer
      updateTickLabelsInRenderer();
    });
  } else {
    // Delegate to parent for common fields (visible, show_bbox, label, etc.)
    GraphicsNode::handleStateChanged(childState);
  }
}

void GridNode::setYZPlaneColor(double r, double g, double b) {
  getState("yz_plane.color_r").value(r);
  getState("yz_plane.color_g").value(g);
  getState("yz_plane.color_b").value(b);
}

void GridNode::setXZPlaneColor(double r, double g, double b) {
  getState("xz_plane.color_r").value(r);
  getState("xz_plane.color_g").value(g);
  getState("xz_plane.color_b").value(b);
}

void GridNode::setXYPlaneColor(double r, double g, double b) {
  getState("xy_plane.color_r").value(r);
  getState("xy_plane.color_g").value(g);
  getState("xy_plane.color_b").value(b);
}

void GridNode::getYZPlaneColor(double &r, double &g, double &b) const {
  r = m_yzPlaneColor[0];
  g = m_yzPlaneColor[1];
  b = m_yzPlaneColor[2];
}

void GridNode::getXZPlaneColor(double &r, double &g, double &b) const {
  r = m_xzPlaneColor[0];
  g = m_xzPlaneColor[1];
  b = m_xzPlaneColor[2];
}

void GridNode::getXYPlaneColor(double &r, double &g, double &b) const {
  r = m_xyPlaneColor[0];
  g = m_xyPlaneColor[1];
  b = m_xyPlaneColor[2];
}

void GridNode::setYZPlaneVisible(bool visible) {
  getState("yz_plane.visible").value(visible ? 1 : 0);
}

void GridNode::setXZPlaneVisible(bool visible) {
  getState("xz_plane.visible").value(visible ? 1 : 0);
}

void GridNode::setXYPlaneVisible(bool visible) {
  getState("xy_plane.visible").value(visible ? 1 : 0);
}

void GridNode::setGridDivisions(int x, int y, int z) {
  getState("divisions_x").value(std::max(1, x));
  getState("divisions_y").value(std::max(1, y));
  getState("divisions_z").value(std::max(1, z));
}

void GridNode::getGridDivisions(int &x, int &y, int &z) const {
  x = m_divisionsX;
  y = m_divisionsY;
  z = m_divisionsZ;
}

void GridNode::setTickIntervals(int x, int y, int z) {
  getState("tics.interval_x").value(std::max(1, x));
  getState("tics.interval_y").value(std::max(1, y));
  getState("tics.interval_z").value(std::max(1, z));
}

void GridNode::getTickIntervals(int &x, int &y, int &z) const {
  x = m_tickIntervalX;
  y = m_tickIntervalY;
  z = m_tickIntervalZ;
}

void GridNode::setTickLabelColor(double r, double g, double b) {
  getState("tics.label_color_r").value(r);
  getState("tics.label_color_g").value(g);
  getState("tics.label_color_b").value(b);
}

void GridNode::getTickLabelColor(double &r, double &g, double &b) const {
  r = m_tickLabelColor[0];
  g = m_tickLabelColor[1];
  b = m_tickLabelColor[2];
}

void GridNode::setTickLabelFontSize(int size) {
  getState("tics.label_font_size").value(std::max(1, size));
}

int GridNode::getTickLabelFontSize() const { return m_tickLabelFontSize; }

void GridNode::createGridPlanes() {
  createYZPlane();
  createXZPlane();
  createXYPlane();
}

void GridNode::createYZPlane() {
  // Create grid at X=minX (YZ plane at minimum corner)
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

  double minX = m_bounds[0];
  double minY = m_bounds[1];
  double minZ = m_bounds[2];
  double maxY = m_bounds[4];
  double maxZ = m_bounds[5];

  double spanY = maxY - minY;
  double spanZ = maxZ - minZ;

  double spacingY = spanY / m_divisionsY;
  double spacingZ = spanZ / m_divisionsZ;

  if (spacingY > 0.0 && spacingZ > 0.0) {
    // Vertical lines (along Z axis)
    for (int i = 0; i <= m_divisionsY; ++i) {
      double y = minY + i * spacingY;
      vtkIdType id1 = points->InsertNextPoint(minX, y, minZ);
      vtkIdType id2 = points->InsertNextPoint(minX, y, maxZ);
      lines->InsertNextCell(2);
      lines->InsertCellPoint(id1);
      lines->InsertCellPoint(id2);
    }

    // Horizontal lines (along Y axis)
    for (int i = 0; i <= m_divisionsZ; ++i) {
      double z = minZ + i * spacingZ;
      vtkIdType id1 = points->InsertNextPoint(minX, minY, z);
      vtkIdType id2 = points->InsertNextPoint(minX, maxY, z);
      lines->InsertNextCell(2);
      lines->InsertCellPoint(id1);
      lines->InsertCellPoint(id2);
    }
  }

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->SetPoints(points);
  polyData->SetLines(lines);
  m_yzMapper->SetInputData(polyData);
}

void GridNode::createXZPlane() {
  // Create grid at Y=minY (XZ plane at minimum corner)
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

  double minX = m_bounds[0];
  double minY = m_bounds[1];
  double minZ = m_bounds[2];
  double maxX = m_bounds[3];
  double maxZ = m_bounds[5];

  double spanX = maxX - minX;
  double spanZ = maxZ - minZ;

  double spacingX = spanX / m_divisionsX;
  double spacingZ = spanZ / m_divisionsZ;

  if (spacingX > 0.0 && spacingZ > 0.0) {
    // Lines along Z axis
    for (int i = 0; i <= m_divisionsX; ++i) {
      double x = minX + i * spacingX;
      vtkIdType id1 = points->InsertNextPoint(x, minY, minZ);
      vtkIdType id2 = points->InsertNextPoint(x, minY, maxZ);
      lines->InsertNextCell(2);
      lines->InsertCellPoint(id1);
      lines->InsertCellPoint(id2);
    }

    // Lines along X axis
    for (int i = 0; i <= m_divisionsZ; ++i) {
      double z = minZ + i * spacingZ;
      vtkIdType id1 = points->InsertNextPoint(minX, minY, z);
      vtkIdType id2 = points->InsertNextPoint(maxX, minY, z);
      lines->InsertNextCell(2);
      lines->InsertCellPoint(id1);
      lines->InsertCellPoint(id2);
    }
  }

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->SetPoints(points);
  polyData->SetLines(lines);
  m_xzMapper->SetInputData(polyData);
}

void GridNode::createXYPlane() {
  // Create grid at Z=minZ (XY plane at minimum corner)
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

  double minX = m_bounds[0];
  double minY = m_bounds[1];
  double minZ = m_bounds[2];
  double maxX = m_bounds[3];
  double maxY = m_bounds[4];

  double spanX = maxX - minX;
  double spanY = maxY - minY;

  double spacingX = spanX / m_divisionsX;
  double spacingY = spanY / m_divisionsY;

  if (spacingX > 0.0 && spacingY > 0.0) {
    // Lines along Y axis
    for (int i = 0; i <= m_divisionsX; ++i) {
      double x = minX + i * spacingX;
      vtkIdType id1 = points->InsertNextPoint(x, minY, minZ);
      vtkIdType id2 = points->InsertNextPoint(x, maxY, minZ);
      lines->InsertNextCell(2);
      lines->InsertCellPoint(id1);
      lines->InsertCellPoint(id2);
    }

    // Lines along X axis
    for (int i = 0; i <= m_divisionsY; ++i) {
      double y = minY + i * spacingY;
      vtkIdType id1 = points->InsertNextPoint(minX, y, minZ);
      vtkIdType id2 = points->InsertNextPoint(maxX, y, minZ);
      lines->InsertNextCell(2);
      lines->InsertCellPoint(id1);
      lines->InsertCellPoint(id2);
    }
  }

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->SetPoints(points);
  polyData->SetLines(lines);
  m_xyMapper->SetInputData(polyData);
}

void GridNode::createTickLabels() {
  createYZTickLabels();
  createXZTickLabels();
  createXYTickLabels();
}

void GridNode::createYZTickLabels() {
  // Clear existing labels
  m_yzTickLabelActors.clear();

  if (!m_yzPlaneVisible || m_tickIntervalY <= 0 || m_tickIntervalZ <= 0)
    return;

  double minY = m_bounds[1];
  double minZ = m_bounds[2];
  double maxY = m_bounds[4];
  double maxZ = m_bounds[5];

  double spanY = maxY - minY;
  double spanZ = maxZ - minZ;

  double spacingY = spanY / m_divisionsY;
  double spacingZ = spanZ / m_divisionsZ;

  if (spacingY <= 0.0 || spacingZ <= 0.0)
    return;

  double minX = m_bounds[0];

  // Create labels only along bottom edge (minZ) at Y intervals
  for (int j = 0; j <= m_divisionsY; j += m_tickIntervalY) {
    double y = minY + j * spacingY;
    double z = minZ; // Bottom edge only

    std::ostringstream oss;
    oss << j;

    vtkSmartPointer<vtkTextMapper> textMapper = vtkSmartPointer<vtkTextMapper>::New();
    textMapper->SetInput(oss.str().c_str());
    textMapper->GetTextProperty()->SetFontSize(m_tickLabelFontSize);
    textMapper->GetTextProperty()->SetColor(m_tickLabelColor);
    textMapper->GetTextProperty()->SetJustificationToCentered();
    textMapper->GetTextProperty()->SetVerticalJustificationToCentered();

    vtkSmartPointer<vtkActor2D> textActor = vtkSmartPointer<vtkActor2D>::New();
    textActor->SetMapper(textMapper);

    textActor->GetPositionCoordinate()->SetCoordinateSystemToWorld();
    textActor->GetPositionCoordinate()->SetValue(minX, y, z);

    textActor->SetVisibility(m_yzPlaneVisible);
    m_yzTickLabelActors.push_back(textActor);
  }

  // Create labels only along left edge (minY) at Z intervals (skip corner)
  for (int k = m_tickIntervalZ; k <= m_divisionsZ; k += m_tickIntervalZ) {
    double y = minY; // Left edge only
    double z = minZ + k * spacingZ;

    std::ostringstream oss;
    oss << k;

    vtkSmartPointer<vtkTextMapper> textMapper = vtkSmartPointer<vtkTextMapper>::New();
    textMapper->SetInput(oss.str().c_str());
    textMapper->GetTextProperty()->SetFontSize(m_tickLabelFontSize);
    textMapper->GetTextProperty()->SetColor(m_tickLabelColor);
    textMapper->GetTextProperty()->SetJustificationToCentered();
    textMapper->GetTextProperty()->SetVerticalJustificationToCentered();

    vtkSmartPointer<vtkActor2D> textActor = vtkSmartPointer<vtkActor2D>::New();
    textActor->SetMapper(textMapper);

    textActor->GetPositionCoordinate()->SetCoordinateSystemToWorld();
    textActor->GetPositionCoordinate()->SetValue(minX, y, z);

    textActor->SetVisibility(m_yzPlaneVisible);
    m_yzTickLabelActors.push_back(textActor);
  }
}

void GridNode::createXZTickLabels() {
  // Clear existing labels
  m_xzTickLabelActors.clear();

  if (!m_xzPlaneVisible || m_tickIntervalX <= 0 || m_tickIntervalZ <= 0)
    return;

  double minX = m_bounds[0];
  double minZ = m_bounds[2];
  double maxX = m_bounds[3];
  double maxZ = m_bounds[5];

  double spanX = maxX - minX;
  double spanZ = maxZ - minZ;

  double spacingX = spanX / m_divisionsX;
  double spacingZ = spanZ / m_divisionsZ;

  if (spacingX <= 0.0 || spacingZ <= 0.0)
    return;

  double minY = m_bounds[1];

  // Create labels only along bottom edge (minZ) at X intervals
  for (int i = 0; i <= m_divisionsX; i += m_tickIntervalX) {
    double x = minX + i * spacingX;
    double z = minZ; // Bottom edge only

    std::ostringstream oss;
    oss << i;

    vtkSmartPointer<vtkTextMapper> textMapper = vtkSmartPointer<vtkTextMapper>::New();
    textMapper->SetInput(oss.str().c_str());
    textMapper->GetTextProperty()->SetFontSize(m_tickLabelFontSize);
    textMapper->GetTextProperty()->SetColor(m_tickLabelColor);
    textMapper->GetTextProperty()->SetJustificationToCentered();
    textMapper->GetTextProperty()->SetVerticalJustificationToCentered();

    vtkSmartPointer<vtkActor2D> textActor = vtkSmartPointer<vtkActor2D>::New();
    textActor->SetMapper(textMapper);

    textActor->GetPositionCoordinate()->SetCoordinateSystemToWorld();
    textActor->GetPositionCoordinate()->SetValue(x, minY, z);

    textActor->SetVisibility(m_xzPlaneVisible);
    m_xzTickLabelActors.push_back(textActor);
  }

  // Create labels only along left edge (minX) at Z intervals (skip corner)
  for (int k = m_tickIntervalZ; k <= m_divisionsZ; k += m_tickIntervalZ) {
    double x = minX; // Left edge only
    double z = minZ + k * spacingZ;

    std::ostringstream oss;
    oss << k;

    vtkSmartPointer<vtkTextMapper> textMapper = vtkSmartPointer<vtkTextMapper>::New();
    textMapper->SetInput(oss.str().c_str());
    textMapper->GetTextProperty()->SetFontSize(m_tickLabelFontSize);
    textMapper->GetTextProperty()->SetColor(m_tickLabelColor);
    textMapper->GetTextProperty()->SetJustificationToCentered();
    textMapper->GetTextProperty()->SetVerticalJustificationToCentered();

    vtkSmartPointer<vtkActor2D> textActor = vtkSmartPointer<vtkActor2D>::New();
    textActor->SetMapper(textMapper);

    textActor->GetPositionCoordinate()->SetCoordinateSystemToWorld();
    textActor->GetPositionCoordinate()->SetValue(x, minY, z);

    textActor->SetVisibility(m_xzPlaneVisible);
    m_xzTickLabelActors.push_back(textActor);
  }
}

void GridNode::createXYTickLabels() {
  // Clear existing labels
  m_xyTickLabelActors.clear();

  if (!m_xyPlaneVisible || m_tickIntervalX <= 0 || m_tickIntervalY <= 0)
    return;

  double minX = m_bounds[0];
  double minY = m_bounds[1];
  double maxX = m_bounds[3];
  double maxY = m_bounds[4];

  double spanX = maxX - minX;
  double spanY = maxY - minY;

  double spacingX = spanX / m_divisionsX;
  double spacingY = spanY / m_divisionsY;

  if (spacingX <= 0.0 || spacingY <= 0.0)
    return;

  double minZ = m_bounds[2];

  // Create labels only along bottom edge (minY) at X intervals
  for (int i = 0; i <= m_divisionsX; i += m_tickIntervalX) {
    double x = minX + i * spacingX;
    double y = minY; // Bottom edge only

    std::ostringstream oss;
    oss << i;

    vtkSmartPointer<vtkTextMapper> textMapper = vtkSmartPointer<vtkTextMapper>::New();
    textMapper->SetInput(oss.str().c_str());
    textMapper->GetTextProperty()->SetFontSize(m_tickLabelFontSize);
    textMapper->GetTextProperty()->SetColor(m_tickLabelColor);
    textMapper->GetTextProperty()->SetJustificationToCentered();
    textMapper->GetTextProperty()->SetVerticalJustificationToCentered();

    vtkSmartPointer<vtkActor2D> textActor = vtkSmartPointer<vtkActor2D>::New();
    textActor->SetMapper(textMapper);

    textActor->GetPositionCoordinate()->SetCoordinateSystemToWorld();
    textActor->GetPositionCoordinate()->SetValue(x, y, minZ);

    textActor->SetVisibility(m_xyPlaneVisible);
    m_xyTickLabelActors.push_back(textActor);
  }

  // Create labels only along left edge (minX) at Y intervals (skip corner)
  for (int j = m_tickIntervalY; j <= m_divisionsY; j += m_tickIntervalY) {
    double x = minX; // Left edge only
    double y = minY + j * spacingY;

    std::ostringstream oss;
    oss << j;

    vtkSmartPointer<vtkTextMapper> textMapper = vtkSmartPointer<vtkTextMapper>::New();
    textMapper->SetInput(oss.str().c_str());
    textMapper->GetTextProperty()->SetFontSize(m_tickLabelFontSize);
    textMapper->GetTextProperty()->SetColor(m_tickLabelColor);
    textMapper->GetTextProperty()->SetJustificationToCentered();
    textMapper->GetTextProperty()->SetVerticalJustificationToCentered();

    vtkSmartPointer<vtkActor2D> textActor = vtkSmartPointer<vtkActor2D>::New();
    textActor->SetMapper(textMapper);

    textActor->GetPositionCoordinate()->SetCoordinateSystemToWorld();
    textActor->GetPositionCoordinate()->SetValue(x, y, minZ);

    textActor->SetVisibility(m_xyPlaneVisible);
    m_xyTickLabelActors.push_back(textActor);
  }
}

void GridNode::updateTickLabelsInRenderer() {
  // Remove old tick labels from renderer if present
  if (m_renderer) {
    for (auto &actor : m_yzTickLabelActors) {
      m_renderer->RemoveViewProp(actor);
    }
    for (auto &actor : m_xzTickLabelActors) {
      m_renderer->RemoveViewProp(actor);
    }
    for (auto &actor : m_xyTickLabelActors) {
      m_renderer->RemoveViewProp(actor);
    }
  }

  // Create new tick labels
  createTickLabels();

  // Add new tick labels to renderer if present and ticks are visible
  bool ticksVisible = getState("tics.visible").value<bool>();
  if (m_renderer && isVisible() && ticksVisible) {
    if (m_yzPlaneVisible) {
      for (auto &actor : m_yzTickLabelActors) {
        m_renderer->AddViewProp(actor);
      }
    }
    if (m_xzPlaneVisible) {
      for (auto &actor : m_xzTickLabelActors) {
        m_renderer->AddViewProp(actor);
      }
    }
    if (m_xyPlaneVisible) {
      for (auto &actor : m_xyTickLabelActors) {
        m_renderer->AddViewProp(actor);
      }
    }
  }
}
