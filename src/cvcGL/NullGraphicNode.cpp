#include <algorithm>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/gl/AxisNode.h>
#include <cvc/gl/GridNode.h>
#include <cvc/gl/NullGraphicNode.h>
#include <limits>
#include <sstream>
#include <vtkActor.h>

NullGraphicNode::NullGraphicNode(cvc::app &ctx, const std::string &statePath,
                                 const std::string &name)
    : GraphicsNode(ctx, statePath, name),
      m_bounds(-0.5, -0.5, -0.5, 0.5, 0.5, 0.5) // Default 1x1x1 box centered at origin
      ,
      m_dummyActor(vtkSmartPointer<vtkActor>::New()),
      m_includeOwnBounds(false) // Don't include own bounds by default (typical for root nodes)
      ,
      m_syncBoundsWithChildren(true) // By default, sync bounds to encompass children
{
  // Dummy actor has no mapper, won't render anything
  // This node exists only to provide bounding box extents

  // Initialize bounds in state tree
  if (!statePath.empty()) {
    std::ostringstream oss;
    oss << m_bounds.minx << "," << m_bounds.miny << "," << m_bounds.minz << "," << m_bounds.maxx
        << "," << m_bounds.maxy << "," << m_bounds.maxz;
    getState("bounds").value(oss.str());
    getState("include_own_bounds").value(m_includeOwnBounds ? 1 : 0);
    getState("sync_bounds_with_children").value(m_syncBoundsWithChildren ? 1 : 0);
  }
}

NullGraphicNode::~NullGraphicNode() {}

vtkProp *NullGraphicNode::getProp() {
  // Return dummy actor that won't render anything
  return m_dummyActor;
}

void NullGraphicNode::setBounds(const cvc::bounding_box &bbox) {
  m_bounds = bbox;

  // Update state tree
  std::ostringstream oss;
  oss << bbox[0] << "," << bbox[1] << "," << bbox[2] << "," << bbox[3] << "," << bbox[4] << ","
      << bbox[5];
  getState("bounds").value(oss.str());

  updateBoundingBoxNode();
}

void NullGraphicNode::setBounds(double minX, double minY, double minZ, double maxX, double maxY,
                                double maxZ) {
  m_bounds = cvc::bounding_box(minX, minY, minZ, maxX, maxY, maxZ);

  // Update state tree
  std::ostringstream oss;
  oss << minX << "," << minY << "," << minZ << "," << maxX << "," << maxY << "," << maxZ;
  getState("bounds").value(oss.str());

  updateBoundingBoxNode();
}

cvc::bounding_box NullGraphicNode::getBoundingBox() const {
  // Return this node's own bounds
  // Note: If we have children, getCombinedBoundingBox() (inherited from GraphicsNode)
  // will handle merging children's transformed bboxes with our bounds
  return m_bounds;
}

void NullGraphicNode::setIncludeOwnBounds(bool include) {
  if (m_includeOwnBounds == include)
    return;

  m_includeOwnBounds = include;

  // Update state tree
  getState("include_own_bounds").value(include ? 1 : 0);

  // Update bbox visualization since combined bounds may have changed
  updateBoundingBoxNode();
}

void NullGraphicNode::setSyncBoundsWithChildren(bool sync) {
  if (m_syncBoundsWithChildren == sync)
    return;

  m_syncBoundsWithChildren = sync;

  // Update state tree
  getState("sync_bounds_with_children").value(sync ? 1 : 0);

  // If enabling sync, update bounds immediately to match children
  if (sync) {
    syncBoundsToChildren();
  }
}

void NullGraphicNode::addGraphicsChild(std::shared_ptr<GraphicsNode> child) {
  // Call parent implementation first
  GraphicsNode::addGraphicsChild(child);

  // Auto-sync bounds to encompass new child
  if (m_syncBoundsWithChildren) {
    syncBoundsToChildren();
  }
}

void NullGraphicNode::removeGraphicsChild(std::shared_ptr<GraphicsNode> child) {
  // Call parent implementation first
  GraphicsNode::removeGraphicsChild(child);

  // Auto-sync bounds after removing child
  if (m_syncBoundsWithChildren) {
    syncBoundsToChildren();
  }
}

void NullGraphicNode::syncBoundsToChildren() {
  if (!m_syncBoundsWithChildren)
    return;

  std::cout << "[DEBUG] NullGraphicNode::syncBoundsToChildren - Syncing bounds for node '"
            << getName() << "', children count: " << m_graphicsChildren.size() << std::endl;

  // Calculate combined bounds of all children (without including our own bounds)
  double acc_minx = std::numeric_limits<double>::max();
  double acc_miny = std::numeric_limits<double>::max();
  double acc_minz = std::numeric_limits<double>::max();
  double acc_maxx = std::numeric_limits<double>::lowest();
  double acc_maxy = std::numeric_limits<double>::lowest();
  double acc_maxz = std::numeric_limits<double>::lowest();

  bool hasChildren = false;

  for (const auto &child : m_graphicsChildren) {
    if (!child)
      continue;

    // Skip grid and axis nodes - they don't contribute to scene bounds
    if (dynamic_cast<GridNode *>(child.get()) || dynamic_cast<AxisNode *>(child.get())) {
      continue;
    }

    cvc::bounding_box childBBox = child->getCombinedBoundingBox();

    // Skip invalid bounding boxes
    if (childBBox[0] > childBBox[3] || childBBox[1] > childBBox[4] || childBBox[2] > childBBox[5]) {
      continue;
    }

    // Transform child's bbox by child's local transform
    vtkMatrix4x4 *childTransform = child->getTransform();

    double corners[8][3] = {
        {childBBox[0], childBBox[1], childBBox[2]}, {childBBox[3], childBBox[1], childBBox[2]},
        {childBBox[0], childBBox[4], childBBox[2]}, {childBBox[3], childBBox[4], childBBox[2]},
        {childBBox[0], childBBox[1], childBBox[5]}, {childBBox[3], childBBox[1], childBBox[5]},
        {childBBox[0], childBBox[4], childBBox[5]}, {childBBox[3], childBBox[4], childBBox[5]}};

    for (int i = 0; i < 8; ++i) {
      double in[4] = {corners[i][0], corners[i][1], corners[i][2], 1.0};
      double out[4];
      childTransform->MultiplyPoint(in, out);

      acc_minx = std::min(acc_minx, out[0]);
      acc_miny = std::min(acc_miny, out[1]);
      acc_minz = std::min(acc_minz, out[2]);
      acc_maxx = std::max(acc_maxx, out[0]);
      acc_maxy = std::max(acc_maxy, out[1]);
      acc_maxz = std::max(acc_maxz, out[2]);
    }

    hasChildren = true;
  }

  // Update bounds to match children (if we have any valid children)
  if (hasChildren && acc_minx <= acc_maxx && acc_miny <= acc_maxy && acc_minz <= acc_maxz) {
    m_bounds = cvc::bounding_box(acc_minx, acc_miny, acc_minz, acc_maxx, acc_maxy, acc_maxz);

    std::cout << "[DEBUG] NullGraphicNode::syncBoundsToChildren - Updated bounds to [" << acc_minx
              << "," << acc_miny << "," << acc_minz << "] to [" << acc_maxx << "," << acc_maxy
              << "," << acc_maxz << "]" << std::endl;

    // Update state tree
    std::ostringstream oss;
    oss << m_bounds.minx << "," << m_bounds.miny << "," << m_bounds.minz << "," << m_bounds.maxx
        << "," << m_bounds.maxy << "," << m_bounds.maxz;
    getState("bounds").value(oss.str());

    // Update visualization
    updateBoundingBoxNode();
  }
}

void NullGraphicNode::handleStateChanged(const std::string &childState) {
  // Handle bounds state changes (no VTK calls, so no runOnMainThread needed)
  if (childState == "bounds") {
    std::string boundsStr = getState("bounds").value<std::string>();
    std::istringstream iss(boundsStr);
    double minX, minY, minZ, maxX, maxY, maxZ;
    char comma;
    if (iss >> minX >> comma >> minY >> comma >> minZ >> comma >> maxX >> comma >> maxY >> comma >>
        maxZ) {
      setBounds(minX, minY, minZ, maxX, maxY, maxZ);
    }
  } else if (childState == "include_own_bounds") {
    int includeOwn = getState("include_own_bounds").value<int>();
    m_includeOwnBounds = (includeOwn != 0);
    // Update bbox visualization since combined bounds may have changed
    updateBoundingBoxNode();
  } else if (childState == "sync_bounds_with_children") {
    int syncBounds = getState("sync_bounds_with_children").value<int>();
    m_syncBoundsWithChildren = (syncBounds != 0);
    // If enabling sync, update bounds immediately
    if (m_syncBoundsWithChildren) {
      syncBoundsToChildren();
    }
  } else {
    // Delegate to parent for common graphics fields
    // Parent will handle its own runOnMainThread wrapping
    GraphicsNode::handleStateChanged(childState);
  }
}
