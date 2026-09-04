#include <algorithm>
#include <cmath>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/gl/BBoxNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/NullGraphicNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/state_publisher.h>
#include <vtkActor2D.h>
#include <vtkMapper.h>
#include <vtkMatrix4x4.h>
#include <vtkPlane.h>
#include <vtkPlaneCollection.h>
#include <vtkProp3D.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkTextMapper.h>
#include <vtkTextProperty.h>
#include <vtkTransform.h>

namespace cvc {
namespace gl {

GraphicsNode::GraphicsNode(cvc::app &ctx, const std::string &statePath, const std::string &name)
    : SceneNode(ctx, statePath), m_name(name), m_transform(vtkSmartPointer<vtkMatrix4x4>::New()),
      m_worldMatrix(vtkSmartPointer<vtkMatrix4x4>::New()),
      m_worldXf(vtkSmartPointer<vtkTransform>::New()),
      m_vtkTransform(vtkSmartPointer<vtkTransform>::New()), m_parent(nullptr), m_showBBox(false),
      m_bboxNode(std::make_shared<BBoxNode>()), m_showLabel(false), m_labelText(name),
      m_labelSize(14), m_labelActor(vtkSmartPointer<vtkActor2D>::New()), m_clipChildren(false),
      m_clipPlanes(vtkSmartPointer<vtkPlaneCollection>::New()) {
  // Initialize transform to identity
  m_transform->Identity();
  m_worldMatrix->Identity();
  m_worldXf->SetMatrix(m_worldMatrix);

  // Resolve the transform state paths once (getState is ~8 us a call).
  if (!statePath.empty()) {
    m_pathPosition = getState("position").fullName();
    m_pathRotation = getState("rotation").fullName();
    m_pathScale = getState("scale").fullName();
    m_pathMatrix = getState("matrix").fullName();
  }
  m_vtkTransform->SetMatrix(m_transform);

  // Initialize label color to white
  m_labelColor[0] = m_labelColor[1] = m_labelColor[2] = 1.0;

  // Setup label actor
  vtkSmartPointer<vtkTextMapper> textMapper = vtkSmartPointer<vtkTextMapper>::New();
  textMapper->SetInput(m_labelText.c_str());
  textMapper->GetTextProperty()->SetFontSize(m_labelSize);
  textMapper->GetTextProperty()->SetColor(m_labelColor);
  textMapper->GetTextProperty()->SetJustificationToCentered();
  textMapper->GetTextProperty()->SetVerticalJustificationToCentered();
  m_labelActor->SetMapper(textMapper);
  m_labelActor->GetPositionCoordinate()->SetCoordinateSystemToWorld();
  m_labelActor->SetVisibility(m_showLabel);

  // Initialize clip planes (6 planes for bounding box faces)
  for (int i = 0; i < 6; ++i) {
    m_clipPlaneArray[i] = vtkSmartPointer<vtkPlane>::New();
    m_clipPlanes->AddItem(m_clipPlaneArray[i]);
  }

  // Initialize state tree values if we have a valid state path
  // Don't batch during construction - initial values should be set silently
  // Handlers will fire when values change AFTER construction completes
  if (!statePath.empty()) {
    getState("show_bbox").value(0);
    getState("show_label").value(0);
    getState("label_text").value(name);
    getState("label_size").value(14);
    getState("label_color").value(std::string("1.0,1.0,1.0"));

    // Transform state attributes
    getState("position").value(std::string("0.0,0.0,0.0"));
    getState("rotation").value(std::string("0.0,0.0,0.0"));
    getState("scale").value(std::string("1.0,1.0,1.0"));

    // Full matrix (16 values, row-major)
    getState("matrix").value(std::string("1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1"));

    // Clip planes
    getState("clip_children").value(0);
  }
}

GraphicsNode::~GraphicsNode() {}

void GraphicsNode::setTransform(vtkMatrix4x4 *matrix) {
  if (matrix) {
    m_transform->DeepCopy(matrix);

    // Update state tree (matrix in row-major format)
    std::ostringstream oss;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        if (i > 0 || j > 0)
          oss << ",";
        oss << m_transform->GetElement(i, j);
      }
    }
    getState("matrix").value(oss.str());

    updateTransform();
  }
}

void GraphicsNode::setTransform(const double matrix[16]) {
  // Input is row-major, VTK uses row-major storage
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      m_transform->SetElement(i, j, matrix[i * 4 + j]);
    }
  }

  // Update state tree (matrix in row-major format)
  std::ostringstream oss;
  for (int i = 0; i < 16; ++i) {
    if (i > 0)
      oss << ",";
    oss << matrix[i];
  }
  getState("matrix").value(oss.str());

  updateTransform();
}

void GraphicsNode::setPosition(double x, double y, double z) {
  m_transform->SetElement(0, 3, x);
  m_transform->SetElement(1, 3, y);
  m_transform->SetElement(2, 3, z);

  // Publish rather than write. The value still reaches the state tree, just on
  // the publisher's cadence and coalesced, so posing a node no longer pays for a
  // path lookup, a signal, and the echo-driven second cascade.
  if (!m_pathPosition.empty()) {
    const std::string value = positionString();
    // Publish through THIS scene's publisher (no process-wide singleton). A node
    // not yet attached to a scene has no publisher, so it writes state directly —
    // the slow path, but such a node is not on the animation hot loop.
    if (SceneGraph *sg = getSceneGraph())
      sg->publisher().publish(m_pathPosition, value);
    else
      cvc::state::instance(app())(m_pathPosition).value(value);
  }

  updateTransform();
}

std::string GraphicsNode::positionString() const {
  // The translation column, formatted exactly as setPosition publishes it, so
  // the two can be compared as strings and a value that survived the round trip
  // through the state tree still compares equal.
  std::ostringstream oss;
  oss << m_transform->GetElement(0, 3) << "," << m_transform->GetElement(1, 3) << ","
      << m_transform->GetElement(2, 3);
  return oss.str();
}

void GraphicsNode::setRotation(double x, double y, double z) {
  // Create transform with rotation
  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  transform->Identity();
  transform->RotateZ(z);
  transform->RotateY(y);
  transform->RotateX(x);

  // Preserve current translation
  double tx = m_transform->GetElement(0, 3);
  double ty = m_transform->GetElement(1, 3);
  double tz = m_transform->GetElement(2, 3);

  m_transform->DeepCopy(transform->GetMatrix());
  m_transform->SetElement(0, 3, tx);
  m_transform->SetElement(1, 3, ty);
  m_transform->SetElement(2, 3, tz);

  // Update state tree
  std::ostringstream oss;
  oss << x << "," << y << "," << z;
  getState("rotation").value(oss.str());

  updateTransform();
}

void GraphicsNode::setScale(double x, double y, double z) {
  // Get current translation
  double tx = m_transform->GetElement(0, 3);
  double ty = m_transform->GetElement(1, 3);
  double tz = m_transform->GetElement(2, 3);

  // Extract rotation part (normalize the 3x3 upper-left)
  vtkSmartPointer<vtkMatrix4x4> rotation = vtkSmartPointer<vtkMatrix4x4>::New();
  for (int i = 0; i < 3; ++i) {
    double len = 0.0;
    for (int j = 0; j < 3; ++j) {
      double val = m_transform->GetElement(i, j);
      len += val * val;
    }
    len = std::sqrt(len);
    if (len > 0.0) {
      for (int j = 0; j < 3; ++j) {
        rotation->SetElement(i, j, m_transform->GetElement(i, j) / len);
      }
    }
  }

  // Apply new scale to rotation
  for (int i = 0; i < 3; ++i) {
    double scale = (i == 0) ? x : (i == 1) ? y : z;
    for (int j = 0; j < 3; ++j) {
      m_transform->SetElement(i, j, rotation->GetElement(i, j) * scale);
    }
  }

  // Restore translation
  m_transform->SetElement(0, 3, tx);
  m_transform->SetElement(1, 3, ty);
  m_transform->SetElement(2, 3, tz);

  // Update state tree
  std::ostringstream oss;
  oss << x << "," << y << "," << z;
  getState("scale").value(oss.str());

  updateTransform();
}

void GraphicsNode::resetTransform() {
  m_transform->Identity();

  // Update state tree to identity
  getState("position").value(std::string("0.0,0.0,0.0"));
  getState("rotation").value(std::string("0.0,0.0,0.0"));
  getState("scale").value(std::string("1.0,1.0,1.0"));
  getState("matrix").value(std::string("1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1"));

  updateTransform();
}

vtkSmartPointer<vtkMatrix4x4> GraphicsNode::getWorldTransform() const {
  // A COPY of the cache, which updateTransform() keeps current top-down. Callers
  // used to receive a freshly recursed product; the value is identical, and the
  // copy preserves the contract that the result may be held and mutated.
  auto out = vtkSmartPointer<vtkMatrix4x4>::New();
  out->DeepCopy(m_worldMatrix);
  return out;
}

void GraphicsNode::updateTransform(bool isRoot) {
  // Refresh the cached world matrix from the parent's, which is already current
  // because a parent is always updated before its children. One multiply, no
  // walk back up to the root and no allocation.
  if (m_parent)
    vtkMatrix4x4::Multiply4x4(m_parent->m_worldMatrix, m_transform, m_worldMatrix);
  else
    m_worldMatrix->DeepCopy(m_transform);
  m_worldXf->SetMatrix(m_worldMatrix); // reused object; SetMatrix marks it Modified

  // Update VTK transform wrapper
  m_vtkTransform->SetMatrix(m_transform);
  m_vtkTransform->Modified();

  // Apply to VTK prop (subclasses override this)
  applyTransformToVTK();

  // Update all children (their world transform changed too) — not a root move.
  for (auto &child : m_graphicsChildren) {
    child->updateTransform(false);
  }

  // Update bbox if visible
  if (m_showBBox) {
    updateBoundingBoxNode();
  }

  // Update clip planes if clipping is enabled
  if (m_clipChildren) {
    updateClipPlanes();
  }

  // Notify once, at the top of the moved subtree, so the SceneGraph can recompute
  // the world bounds/grid to follow this node.
  if (isRoot) {
    transformChanged(this);
  }
}

void GraphicsNode::applyWorldTransformToProps(const std::vector<vtkProp *> &props) {
  if (props.empty())
    return;

  // m_worldXf already carries this node's current world transform (refreshed in
  // updateTransform). Hand the SAME object to the prop and only re-set it if the
  // prop is not already holding it: after the first frame this loop is a pointer
  // compare, and the per-frame work is the one SetMatrix above.
  for (vtkProp *prop : props) {
    if (!prop)
      continue;
    if (vtkProp3D *prop3D = vtkProp3D::SafeDownCast(prop)) {
      if (prop3D->GetUserTransform() != m_worldXf.Get())
        prop3D->SetUserTransform(m_worldXf.Get());
    }
  }
}

void GraphicsNode::applyTransformToVTK() {
  // Base class does nothing - subclasses override to apply transform to their
  // specific VTK prop E.g., GeometryNode calls
  // applyWorldTransformToProps({m_actor})
}

void GraphicsNode::updateBoundingBoxNode() {
  if (!m_bboxNode)
    return;

  // Get COMBINED bounding box (this node + all children) in LOCAL space
  // This ensures the bbox shows the full extent including children
  cvc::bounding_box bbox = getCombinedBoundingBox();

  // Get world transform
  vtkSmartPointer<vtkMatrix4x4> worldTransform = getWorldTransform();

  // Update bbox on main thread (VTK operations must be on main thread)
  runOnMainThread([this, bbox, worldTransform]() {
    if (m_bboxNode) {
      // Set the bounding box geometry in local space
      m_bboxNode->setBoundingBox(bbox);

      // Apply the world transform to the bbox actor so it renders correctly
      m_bboxNode->setTransform(worldTransform);
    }
  });
}

void GraphicsNode::handleStateChanged(const std::string &childState) {
  // Marshal to main thread via event queue
  runOnMainThread([this, childState]() {
    // Handle state changes for graphics-specific fields
    if (childState == "show_bbox") {
      int showBBox = getState("show_bbox").value<int>();
      setShowBBox(showBBox != 0);
    } else if (childState == "show_label") {
      int showLabel = getState("show_label").value<int>();
      setShowLabel(showLabel != 0);
    } else if (childState == "label_text") {
      std::string labelText = getState("label_text").value<std::string>();
      setLabelText(labelText);
    } else if (childState == "label_size") {
      int labelSize = getState("label_size").value<int>();
      setLabelSize(labelSize);
    } else if (childState == "label_color") {
      try {
        std::string colorStr = getState("label_color").value<std::string>();
        std::istringstream iss(colorStr);
        double r, g, b;
        char comma;
        if (iss >> r >> comma >> g >> comma >> b) {
          setLabelColor(r, g, b);
        }
      } catch (const boost::bad_lexical_cast &) {
        // Ignore - state initialization may trigger before all components are
        // set
      }
    } else if (childState == "position") {
      try {
        std::string posStr = getState("position").value<std::string>();
        // Skip only what is genuinely nothing to do: a value the node ALREADY
        // HOLDS. Comparing against the last value published instead looked the
        // same whenever the echo arrived promptly, and inverted the meaning
        // whenever it did not — a late echo of an older pose failed the test and
        // was applied, moving the node backwards, and the echo of the current
        // pose then passed it and was dropped, so the node never came back. The
        // node is the source of truth for its transform, so anything it is not
        // already showing is a real change and gets applied; an external write
        // (the dashboard, a script) still lands, exactly as before.
        if (posStr == positionString())
          return;
        std::istringstream iss(posStr);
        double x, y, z;
        char comma;
        if (iss >> x >> comma >> y >> comma >> z) {
          // Directly update matrix without triggering state update (avoid
          // loop)
          m_transform->SetElement(0, 3, x);
          m_transform->SetElement(1, 3, y);
          m_transform->SetElement(2, 3, z);
          updateTransform();
        }
      } catch (const boost::bad_lexical_cast &) {
      }
    } else if (childState == "rotation") {
      try {
        std::string rotStr = getState("rotation").value<std::string>();
        std::istringstream iss(rotStr);
        double rx, ry, rz;
        char comma;
        if (iss >> rx >> comma >> ry >> comma >> rz) {
          // Create transform with rotation
          vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
          transform->Identity();
          transform->RotateZ(rz);
          transform->RotateY(ry);
          transform->RotateX(rx);

          // Preserve current translation
          double tx = m_transform->GetElement(0, 3);
          double ty = m_transform->GetElement(1, 3);
          double tz = m_transform->GetElement(2, 3);

          m_transform->DeepCopy(transform->GetMatrix());
          m_transform->SetElement(0, 3, tx);
          m_transform->SetElement(1, 3, ty);
          m_transform->SetElement(2, 3, tz);

          updateTransform();
        }
      } catch (const boost::bad_lexical_cast &) {
      }
    } else if (childState == "scale") {
      try {
        std::string scaleStr = getState("scale").value<std::string>();
        std::istringstream iss(scaleStr);
        double sx, sy, sz;
        char comma;
        if (iss >> sx >> comma >> sy >> comma >> sz) {
          // Get current translation
          double tx = m_transform->GetElement(0, 3);
          double ty = m_transform->GetElement(1, 3);
          double tz = m_transform->GetElement(2, 3);

          // Extract rotation part (normalize the 3x3 upper-left)
          vtkSmartPointer<vtkMatrix4x4> rotation = vtkSmartPointer<vtkMatrix4x4>::New();
          for (int i = 0; i < 3; ++i) {
            double len = 0.0;
            for (int j = 0; j < 3; ++j) {
              double val = m_transform->GetElement(i, j);
              len += val * val;
            }
            len = std::sqrt(len);
            if (len > 0.0) {
              for (int j = 0; j < 3; ++j) {
                rotation->SetElement(i, j, m_transform->GetElement(i, j) / len);
              }
            }
          }

          // Apply new scale to rotation
          for (int i = 0; i < 3; ++i) {
            double scale = (i == 0) ? sx : (i == 1) ? sy : sz;
            for (int j = 0; j < 3; ++j) {
              m_transform->SetElement(i, j, rotation->GetElement(i, j) * scale);
            }
          }

          // Restore translation
          m_transform->SetElement(0, 3, tx);
          m_transform->SetElement(1, 3, ty);
          m_transform->SetElement(2, 3, tz);

          updateTransform();
        }
      } catch (const boost::bad_lexical_cast &) {
      }
    } else if (childState == "matrix") {
      try {
        std::string matrixStr = getState("matrix").value<std::string>();
        std::istringstream iss(matrixStr);
        double values[16];
        char comma;

        // Read 16 comma-separated values
        for (int i = 0; i < 16; ++i) {
          if (i > 0)
            iss >> comma;
          if (!(iss >> values[i]))
            break;
        }

        // Update matrix (row-major input)
        for (int i = 0; i < 4; ++i) {
          for (int j = 0; j < 4; ++j) {
            m_transform->SetElement(i, j, values[i * 4 + j]);
          }
        }
        updateTransform();
      } catch (const boost::bad_lexical_cast &) {
      }
    } else if (childState == "clip_children") {
      int clip = getState("clip_children").value<int>();
      setClipChildren(clip != 0);
    } else {
      // Delegate to parent for common fields like visible
      // Parent will NOT wrap again - we're already on main thread
      SceneNode::handleStateChanged(childState);
    }

    // Request render after any state change. NEVER render synchronously here:
    // this handler runs once per changed state key, and per-frame animation
    // (setTransform/setColor on many nodes) writes many keys per frame — a
    // synchronous Render() per key turns each displayed frame into dozens of
    // full scene renders (measured: 63 animated nodes at ~5 ms per render
    // dragged VolRover3 from 60 FPS to under 3). The host's frame loop renders
    // once per frame via checkAndResetRenderNeeded(). The synchronous fallback
    // remains only for a node used without a SceneGraph, where nothing drains
    // the flag.
    if (m_sceneGraph) {
      m_sceneGraph->requestRender();
    } else if (m_renderer && m_renderer->GetRenderWindow()) {
      m_renderer->GetRenderWindow()->Render();
    }
  });
}

void GraphicsNode::addGraphicsChild(std::shared_ptr<GraphicsNode> child) {
  if (!child)
    return;

  // Add to graphics children list
  m_graphicsChildren.push_back(child);

  // Set parent pointer
  child->m_parent = this;

  // Propagate SceneGraph reference to child
  child->setSceneGraph(m_sceneGraph);

  // Also add as SceneNode child so it gets rendered
  addChild(child);

  // Update child's transform to reflect new parent
  child->updateTransform();

  // Update this node's bounding box to include the new child
  if (m_showBBox) {
    updateBoundingBoxNode();
  }
}

std::shared_ptr<GraphicsNode> GraphicsNode::createChild(const std::string &name) {
  // Create NullGraphicNode child for placeholder/hierarchy purposes
  return addGraphicsChild<NullGraphicNode>(name);
}

void GraphicsNode::removeGraphicsChild(std::shared_ptr<GraphicsNode> child) {
  if (!child)
    return;

  // Remove from graphics children
  auto it = std::find(m_graphicsChildren.begin(), m_graphicsChildren.end(), child);
  if (it != m_graphicsChildren.end()) {
    m_graphicsChildren.erase(it);
    child->m_parent = nullptr;
    child->updateTransform();
  }

  // Also remove as SceneNode child
  removeChild(child);

  // Update this node's bounding box after removing child
  if (m_showBBox) {
    updateBoundingBoxNode();
  }
}

std::shared_ptr<GraphicsNode> GraphicsNode::findChildByName(const std::string &name) {
  for (auto &child : m_graphicsChildren) {
    if (child->getName() == name) {
      return child;
    }
    // Recursively search in child's children
    auto found = child->findChildByName(name);
    if (found) {
      return found;
    }
  }
  return nullptr;
}

cvc::bounding_box GraphicsNode::getCombinedBoundingBox() const {
  // Check if this is a NullGraphicNode and if it should include own bounds
  const NullGraphicNode *nullNode = dynamic_cast<const NullGraphicNode *>(this);
  bool includeOwnBounds = true;
  if (nullNode) {
    includeOwnBounds = nullNode->getIncludeOwnBounds();
  }

  // Accumulate extents without creating invalid bbox
  double acc_minx = std::numeric_limits<double>::max();
  double acc_miny = std::numeric_limits<double>::max();
  double acc_minz = std::numeric_limits<double>::max();
  double acc_maxx = std::numeric_limits<double>::lowest();
  double acc_maxy = std::numeric_limits<double>::lowest();
  double acc_maxz = std::numeric_limits<double>::lowest();

  // Include own bounds if requested
  if (includeOwnBounds) {
    cvc::bounding_box ownBBox = getBoundingBox();
    acc_minx = ownBBox[0];
    acc_miny = ownBBox[1];
    acc_minz = ownBBox[2];
    acc_maxx = ownBBox[3];
    acc_maxy = ownBBox[4];
    acc_maxz = ownBBox[5];
  }

  // Expand to include all children (transformed to this node's local space)
  for (const auto &child : m_graphicsChildren) {
    if (!child)
      continue;

    // Get child's combined bbox (includes child's descendants in child's
    // local space)
    cvc::bounding_box childBBox = child->getCombinedBoundingBox();

    // Skip invalid bounding boxes
    if (childBBox[0] > childBBox[3] || childBBox[1] > childBBox[4] || childBBox[2] > childBBox[5]) {
      continue;
    }

    // Transform child's bbox by child's local transform to get it in this
    // node's space
    vtkMatrix4x4 *childTransform = child->getTransform();

    // Transform all 8 corners of child's bbox
    double corners[8][3] = {
        {childBBox[0], childBBox[1], childBBox[2]}, // min, min, min
        {childBBox[3], childBBox[1], childBBox[2]}, // max, min, min
        {childBBox[0], childBBox[4], childBBox[2]}, // min, max, min
        {childBBox[3], childBBox[4], childBBox[2]}, // max, max, min
        {childBBox[0], childBBox[1], childBBox[5]}, // min, min, max
        {childBBox[3], childBBox[1], childBBox[5]}, // max, min, max
        {childBBox[0], childBBox[4], childBBox[5]}, // min, max, max
        {childBBox[3], childBBox[4], childBBox[5]}  // max, max, max
    };

    double minx = std::numeric_limits<double>::max();
    double miny = std::numeric_limits<double>::max();
    double minz = std::numeric_limits<double>::max();
    double maxx = std::numeric_limits<double>::lowest();
    double maxy = std::numeric_limits<double>::lowest();
    double maxz = std::numeric_limits<double>::lowest();

    for (int i = 0; i < 8; ++i) {
      double in[4] = {corners[i][0], corners[i][1], corners[i][2], 1.0};
      double out[4];
      childTransform->MultiplyPoint(in, out);

      minx = std::min(minx, out[0]);
      miny = std::min(miny, out[1]);
      minz = std::min(minz, out[2]);
      maxx = std::max(maxx, out[0]);
      maxy = std::max(maxy, out[1]);
      maxz = std::max(maxz, out[2]);
    }

    // Expand accumulated extents to include transformed child
    acc_minx = std::min(acc_minx, minx);
    acc_miny = std::min(acc_miny, miny);
    acc_minz = std::min(acc_minz, minz);
    acc_maxx = std::max(acc_maxx, maxx);
    acc_maxy = std::max(acc_maxy, maxy);
    acc_maxz = std::max(acc_maxz, maxz);
  }

  // Create final bounding box from accumulated extents
  // If no valid extents were accumulated, return a default small box
  if (acc_minx > acc_maxx || acc_miny > acc_maxy || acc_minz > acc_maxz) {
    return cvc::bounding_box(-0.5, -0.5, -0.5, 0.5, 0.5, 0.5);
  }

  return cvc::bounding_box(acc_minx, acc_miny, acc_minz, acc_maxx, acc_maxy, acc_maxz);
}

void GraphicsNode::setMetadata(const std::string &key, const std::any &value) {
  m_metadata[key] = value;

  // Also sync to state tree for persistence and visibility
  // Create metadata substate if needed
  try {
    std::string metadataPath = "metadata." + key;

    // Convert std::any to appropriate type and store in state
    if (value.type() == typeid(int)) {
      getState(metadataPath).value(std::any_cast<int>(value));
      getState(metadataPath).readOnly(true);
    } else if (value.type() == typeid(double)) {
      getState(metadataPath).value(std::any_cast<double>(value));
      getState(metadataPath).readOnly(true);
    } else if (value.type() == typeid(std::string)) {
      getState(metadataPath).value(std::any_cast<std::string>(value));
      getState(metadataPath).readOnly(true);
    } else if (value.type() == typeid(const char *)) {
      getState(metadataPath).value(std::string(std::any_cast<const char *>(value)));
      getState(metadataPath).readOnly(true);
    } else if (value.type() == typeid(bool)) {
      getState(metadataPath).value(std::any_cast<bool>(value));
      getState(metadataPath).readOnly(true);
    }
    // Add more types as needed
  } catch (...) {
    // Ignore metadata sync errors
  }
}

std::any GraphicsNode::getMetadata(const std::string &key) const {
  auto it = m_metadata.find(key);
  if (it != m_metadata.end()) {
    return it->second;
  }
  return std::any();
}

bool GraphicsNode::hasMetadata(const std::string &key) const {
  return m_metadata.find(key) != m_metadata.end();
}

void GraphicsNode::update() {
  // With state_object, we don't need manual syncing
  // The state tree automatically synchronizes via handleStateChanged()
  // Just propagate to children
  SceneNode::update();
}

void GraphicsNode::setVisible(bool visible) {
  SceneNode::setVisible(visible);

  // Update label visibility (wrap VTK operation)
  if (m_labelActor) {
    runOnMainThread([this, visible]() {
      if (m_labelActor) {
        m_labelActor->SetVisibility(m_showLabel && visible);
      }
    });
  }
}

void GraphicsNode::setShowBBox(bool show) {
  if (m_showBBox == show)
    return;

  m_showBBox = show;

  // Update state tree value
  getState("show_bbox").value(show ? 1 : 0);

  if (m_bboxNode && m_renderer) {
    // Wrap VTK operations in runOnMainThread
    runOnMainThread([this, show]() {
      if (m_bboxNode && m_renderer) {
        if (show) {
          updateBoundingBoxNode();
          m_bboxNode->addToRenderer(m_renderer);
        } else {
          m_bboxNode->removeFromRenderer(m_renderer);
        }
      }
    });
  }
}

void GraphicsNode::setBBoxColor(double r, double g, double b) {
  if (m_bboxNode) {
    m_bboxNode->setColor(r, g, b);
  }
}

void GraphicsNode::getBBoxColor(double &r, double &g, double &b) const {
  if (m_bboxNode) {
    m_bboxNode->getColor(r, g, b);
  } else {
    r = g = b = 1.0;
  }
}

void GraphicsNode::setShowExtentLabels(bool show) {
  // Update state tree value
  getState("show_extent_labels").value(show ? 1 : 0);

  if (m_bboxNode) {
    m_bboxNode->setCoordinatesVisible(show);
  }
}

bool GraphicsNode::getShowExtentLabels() const {
  if (m_bboxNode) {
    return m_bboxNode->getCoordinatesVisible();
  }
  return false;
}

void GraphicsNode::setExtentLabelColor(double r, double g, double b) {
  // Update state tree values
  getState("extent_label_color_r").value(r);
  getState("extent_label_color_g").value(g);
  getState("extent_label_color_b").value(b);

  if (m_bboxNode) {
    m_bboxNode->setCoordinateLabelColor(r, g, b);
  }
}

void GraphicsNode::getExtentLabelColor(double &r, double &g, double &b) const {
  if (m_bboxNode) {
    m_bboxNode->getCoordinateLabelColor(r, g, b);
  } else {
    r = g = b = 1.0;
  }
}

void GraphicsNode::setExtentLabelFontSize(int size) {
  // Update state tree value
  getState("extent_label_font_size").value(size);

  if (m_bboxNode) {
    m_bboxNode->setCoordinateLabelFontSize(size);
  }
}

int GraphicsNode::getExtentLabelFontSize() const {
  if (m_bboxNode) {
    return m_bboxNode->getCoordinateLabelFontSize();
  }
  return 12; // Default font size
}

void GraphicsNode::setShowLabel(bool show) {
  if (m_showLabel == show)
    return;

  m_showLabel = show;
  m_labelActor->SetVisibility(m_showLabel && isVisible());

  // Add or remove from renderer if needed
  if (m_renderer) {
    if (m_showLabel && isVisible()) {
      updateLabel();
      m_renderer->AddViewProp(m_labelActor);
    } else {
      m_renderer->RemoveViewProp(m_labelActor);
    }
  }
}

void GraphicsNode::setLabelText(const std::string &text) {
  m_labelText = text;
  vtkTextMapper *mapper = vtkTextMapper::SafeDownCast(m_labelActor->GetMapper());
  if (mapper) {
    mapper->SetInput(m_labelText.c_str());
  }
}

void GraphicsNode::setLabelSize(int size) {
  m_labelSize = std::max(1, size);
  vtkTextMapper *mapper = vtkTextMapper::SafeDownCast(m_labelActor->GetMapper());
  if (mapper) {
    mapper->GetTextProperty()->SetFontSize(m_labelSize);
  }
}

void GraphicsNode::setLabelColor(double r, double g, double b) {
  m_labelColor[0] = r;
  m_labelColor[1] = g;
  m_labelColor[2] = b;
  vtkTextMapper *mapper = vtkTextMapper::SafeDownCast(m_labelActor->GetMapper());
  if (mapper) {
    mapper->GetTextProperty()->SetColor(r, g, b);
  }
}

void GraphicsNode::getLabelColor(double &r, double &g, double &b) const {
  r = m_labelColor[0];
  g = m_labelColor[1];
  b = m_labelColor[2];
}

void GraphicsNode::updateLabel() {
  // Position label at center of bounding box in LOCAL space
  cvc::bounding_box bbox = getBoundingBox();
  double centerX = (bbox[0] + bbox[3]) / 2.0;
  double centerY = (bbox[1] + bbox[4]) / 2.0;
  double centerZ = (bbox[2] + bbox[5]) / 2.0;

  // Transform center to world space
  vtkSmartPointer<vtkMatrix4x4> worldTransform = getWorldTransform();
  double localCenter[4] = {centerX, centerY, centerZ, 1.0};
  double worldCenter[4];
  worldTransform->MultiplyPoint(localCenter, worldCenter);

  m_labelActor->GetPositionCoordinate()->SetValue(worldCenter[0], worldCenter[1], worldCenter[2]);
}

void GraphicsNode::addToRenderer(vtkRenderer *renderer) {
  // Call base implementation to add the main prop
  SceneNode::addToRenderer(renderer);

  // Add bbox if it should be visible
  if (m_showBBox && m_bboxNode) {
    // Update and capture references before queuing
    updateBoundingBoxNode();
    auto bboxNode = m_bboxNode;
    runOnMainThread([bboxNode, renderer]() { bboxNode->addToRenderer(renderer); });
  }

  // Add label if it should be visible
  if (m_showLabel && m_labelActor) {
    // Update and capture the actor before queuing
    updateLabel();
    vtkActor2D *labelActor = m_labelActor;
    runOnMainThread([labelActor, renderer]() { renderer->AddViewProp(labelActor); });
  }
}

void GraphicsNode::removeFromRenderer(vtkRenderer *renderer) {
  // Remove label - capture the actor pointer to avoid accessing 'this' after
  // deletion
  vtkActor2D *labelActor = m_labelActor;
  if (labelActor) {
    runOnMainThread([labelActor, renderer]() { renderer->RemoveViewProp(labelActor); });
  }

  // Remove bbox
  if (m_bboxNode) {
    m_bboxNode->removeFromRenderer(renderer);
  }

  // Call base implementation to remove the main prop
  SceneNode::removeFromRenderer(renderer);
}

void GraphicsNode::setClipChildren(bool clip) {
  if (m_clipChildren == clip)
    return;

  m_clipChildren = clip;

  // Update state tree
  getState("clip_children").value(clip ? 1 : 0);

  if (m_clipChildren) {
    // Enable clipping - update and apply planes
    updateClipPlanes();
    applyClipPlanesToChildren();
  } else {
    // Disable clipping - remove planes from children
    for (auto &child : m_graphicsChildren) {
      child->runOnMainThread([child]() { child->applyClipPlanes(nullptr); });
    }
  }
}

void GraphicsNode::updateClipPlanes() {
  // Get this node's OWN bounding box (not combined)
  cvc::bounding_box bbox = getBoundingBox();
  double bounds[6] = {bbox.minx, bbox.maxx, bbox.miny, bbox.maxy, bbox.minz, bbox.maxz};

  // Get world transform to transform the planes
  vtkSmartPointer<vtkMatrix4x4> worldTransform = getWorldTransform();

  // Define 6 plane normals in local space
  // Order: +X, -X, +Y, -Y, +Z, -Z
  double normals[6][3] = {
      {1.0, 0.0, 0.0},  // +X face (points inward: -X)
      {-1.0, 0.0, 0.0}, // -X face (points inward: +X)
      {0.0, 1.0, 0.0},  // +Y face (points inward: -Y)
      {0.0, -1.0, 0.0}, // -Y face (points inward: +Y)
      {0.0, 0.0, 1.0},  // +Z face (points inward: -Z)
      {0.0, 0.0, -1.0}  // -Z face (points inward: +Z)
  };

  // Plane origins in local space (centers of each face)
  double origins[6][3] = {
      {bounds[1], (bounds[2] + bounds[3]) / 2.0, (bounds[4] + bounds[5]) / 2.0}, // +X
      {bounds[0], (bounds[2] + bounds[3]) / 2.0, (bounds[4] + bounds[5]) / 2.0}, // -X
      {(bounds[0] + bounds[1]) / 2.0, bounds[3], (bounds[4] + bounds[5]) / 2.0}, // +Y
      {(bounds[0] + bounds[1]) / 2.0, bounds[2], (bounds[4] + bounds[5]) / 2.0}, // -Y
      {(bounds[0] + bounds[1]) / 2.0, (bounds[2] + bounds[3]) / 2.0, bounds[5]}, // +Z
      {(bounds[0] + bounds[1]) / 2.0, (bounds[2] + bounds[3]) / 2.0, bounds[4]}  // -Z
  };

  // Transform and set each plane
  for (int i = 0; i < 6; ++i) {
    // Transform origin to world space
    double worldOrigin[4] = {origins[i][0], origins[i][1], origins[i][2], 1.0};
    double transformedOrigin[4];
    worldTransform->MultiplyPoint(worldOrigin, transformedOrigin);

    // Transform normal to world space (using transpose of inverse for
    // normals) For orthogonal transforms (rotation + uniform scale), we can
    // use the matrix directly
    vtkSmartPointer<vtkMatrix4x4> normalMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
    normalMatrix->DeepCopy(worldTransform);
    normalMatrix->Invert();
    normalMatrix->Transpose();

    double worldNormal[4] = {normals[i][0], normals[i][1], normals[i][2], 0.0};
    double transformedNormal[4];
    normalMatrix->MultiplyPoint(worldNormal, transformedNormal);

    // Normalize the transformed normal
    double len = std::sqrt(transformedNormal[0] * transformedNormal[0] +
                           transformedNormal[1] * transformedNormal[1] +
                           transformedNormal[2] * transformedNormal[2]);
    if (len > 0.0) {
      transformedNormal[0] /= len;
      transformedNormal[1] /= len;
      transformedNormal[2] /= len;
    }

    // Set plane
    m_clipPlaneArray[i]->SetOrigin(transformedOrigin[0], transformedOrigin[1],
                                   transformedOrigin[2]);
    m_clipPlaneArray[i]->SetNormal(transformedNormal[0], transformedNormal[1],
                                   transformedNormal[2]);
  }

  // Apply updated planes to children
  if (m_clipChildren) {
    applyClipPlanesToChildren();
  }
}

void GraphicsNode::applyClipPlanesToChildren() {
  for (auto &child : m_graphicsChildren) {
    child->runOnMainThread([this, child]() { child->applyClipPlanes(m_clipPlanes); });
  }
}

void GraphicsNode::applyClipPlanes(vtkPlaneCollection *planes) {
  // Base implementation does nothing
  // Subclasses that support clipping (GeometryNode, VolumeNode, GridNode)
  // override this
}

} // namespace gl
} // namespace cvc
