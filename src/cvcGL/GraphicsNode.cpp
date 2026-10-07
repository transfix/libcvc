#include <algorithm>
#include <cmath>
#include <cvc/core/app.h>
#include <cvc/gl/BBoxNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/NullGraphicNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/state_publisher.h>
#include <cvc/state/state.h>
#include <iomanip>
#include <sstream>
#include <vtkActor2D.h>
#include <vtkInformation.h>
#include <vtkInformationIntegerKey.h>
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
      m_worldInverse(vtkSmartPointer<vtkMatrix4x4>::New()),
      m_worldXf(vtkSmartPointer<vtkTransform>::New()),
      m_vtkTransform(vtkSmartPointer<vtkTransform>::New()), m_parent(nullptr), m_showBBox(false),
      m_bboxNode(std::make_shared<BBoxNode>()), m_showLabel(false), m_labelText(name),
      m_labelSize(14), m_labelActor(vtkSmartPointer<vtkActor2D>::New()), m_clipChildren(false),
      m_clipPlanes(vtkSmartPointer<vtkPlaneCollection>::New()) {
  // Initialize transform to identity
  m_transform->Identity();
  m_worldMatrix->Identity();
  m_worldInverse->Identity();
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
    getState("clip_planes").value(std::string());
  }
}

GraphicsNode::~GraphicsNode() {
  // Orphan the graphics children before they are released. m_parent is a raw
  // back-pointer used by updateTransform() to compose the world matrix, and a
  // child can outlive its parent: the scene's own destruction drops the graphics
  // root while a host still holds one of its nodes. Without this, that node's
  // next setPosition() multiplies through a freed parent — the same dangling
  // back-pointer the SceneGraph handle closes, one level down. Children that
  // nothing else holds die immediately after; the rest carry on as roots, which
  // is what they now are. No updateTransform() here: the cached world matrix
  // stays put until the next pose write recomputes it parentless.
  for (auto &child : m_graphicsChildren) {
    if (child)
      child->m_parent = nullptr;
  }
}

void GraphicsNode::setTransform(vtkMatrix4x4 *matrix) {
  if (matrix) {
    m_transform->DeepCopy(matrix);

    // Update state tree (matrix in row-major format)
    // Full double precision so the transform round-trips exactly through the
    // string-backed state tree -- world coordinates and dimensions taken through
    // the chain must not lose precision to a 6-significant-digit default.
    std::ostringstream oss;
    oss << std::setprecision(17);
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        if (i > 0 || j > 0)
          oss << ",";
        oss << m_transform->GetElement(i, j);
      }
    }
    getState("matrix").value(oss.str());
    supersedePublishedMatrix();

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
  oss << std::setprecision(17); // exact round-trip through the state tree
  for (int i = 0; i < 16; ++i) {
    if (i > 0)
      oss << ",";
    oss << matrix[i];
  }
  getState("matrix").value(oss.str());
  supersedePublishedMatrix();

  updateTransform();
}

void GraphicsNode::setPoseMatrix(const double matrix[16]) {
  SceneGraph *sg = getSceneGraph();
  if (!sg || sg->onOwnerThread()) {
    // Applied here and now. A pose another thread parked earlier is older than
    // this one, so drop it rather than let its queued apply land on top at the
    // next pump. The atomic read keeps the common case (nothing parked) free of
    // the lock.
    if (m_posePending.load(std::memory_order_acquire)) {
      std::lock_guard<std::mutex> lock(m_poseMutex);
      m_posePending.store(false, std::memory_order_relaxed);
    }
    applyPoseMatrix(matrix);
    return;
  }

  // Off the owner thread: touch nothing VTK-side here. Park the matrix in the
  // one-slot mailbox and queue ONE apply for it; later calls before that apply
  // runs just overwrite the slot (latest wins), so a simulation posing faster
  // than the frame rate cannot grow the event queue.
  bool queueApply = false;
  {
    std::lock_guard<std::mutex> lock(m_poseMutex);
    std::copy(matrix, matrix + 16, m_parkedPose.begin());
    queueApply = !m_posePending.exchange(true, std::memory_order_acq_rel);
  }
  if (queueApply)
    runOnMainThread([this]() { applyParkedPose(); }); // weak-guarded against node teardown
}

void GraphicsNode::applyParkedPose() {
  double m[16];
  {
    std::lock_guard<std::mutex> lock(m_poseMutex);
    if (!m_posePending.load(std::memory_order_relaxed))
      return; // an owner-thread call superseded it
    std::copy(m_parkedPose.begin(), m_parkedPose.end(), m);
    m_posePending.store(false, std::memory_order_relaxed);
  }
  applyPoseMatrix(m);
}

void GraphicsNode::applyPoseMatrix(const double matrix[16]) {
  // Change detection. A parked vehicle re-sent every frame must cost nothing:
  // no cascade, no actor Modified() (the shadow baker re-bakes on that), no
  // publish, no bounds walk. vtkMatrix4x4 stores row-major, as the input is.
  const double *cur = m_transform->GetData();
  if (std::equal(matrix, matrix + 16, cur)) {
    // The skipped publish would have relied on the previous one reaching the
    // tree. The publisher's eventual consistency assumes a node keeps
    // republishing, and a parked node does not. So if the publisher has shed
    // anything since our last publish (its 8192-path cap), our pose may be what
    // it shed: send it again, to state only.
    SceneGraph *sg = getSceneGraph();
    if (sg && m_matrixPublished.load(std::memory_order_relaxed) &&
        sg->publisher().dropped() != m_poseDropMark)
      publishPoseMatrix(matrix);
    return;
  }
  m_transform->DeepCopy(matrix);
  publishPoseMatrix(matrix);
  updateTransform(); // one cascade, one transformChanged
}

void GraphicsNode::publishPoseMatrix(const double matrix[16]) {
  if (m_pathMatrix.empty())
    return;
  // Through THIS scene's publisher, which formats on the flushing thread; the
  // echo is recognised in handleStateChanged (state_publisher::flushing()).
  if (SceneGraph *sg = getSceneGraph()) {
    state_publisher &pub = sg->publisher();
    // Read BEFORE the enqueue, so a shed of this very offer moves dropped()
    // past the mark and the next unchanged pose is sent again.
    m_poseDropMark = pub.dropped();
    m_matrixPublished.store(true, std::memory_order_relaxed);
    pub.publish_matrix(m_pathMatrix, matrix);
    return;
  }
  // No scene, so no publisher: write directly (the slow path, but a scene-less
  // node is not on an animation loop). The handler this fires runs inline on
  // this thread; m_writingPose tells it the value is our own.
  struct writing_scope {
    bool &flag;
    explicit writing_scope(bool &f) : flag(f) { flag = true; }
    ~writing_scope() { flag = false; }
  } writing(m_writingPose);
  cvc::state::instance(app())(m_pathMatrix).value(state_publisher::format_matrix(matrix));
}

void GraphicsNode::supersedePublishedMatrix() {
  // Only a node that has published a pose can have one queued. The flag is
  // never cleared, which costs at most a redundant enqueue. The flush that
  // follows then writes the text the tree already holds, which fires nothing;
  // if the text differs (an external write in another format), the change it
  // fires is this scene's publisher's, so the echo guard drops it.
  if (!m_matrixPublished.load(std::memory_order_relaxed))
    return;
  // The caller has already written the new matrix to state directly. Queueing
  // it as well makes it the publisher's last value for the path. A pose still
  // in the queue is replaced by it. A pose a flush is writing at this moment is
  // followed by it at the next flush, because flushes are serialised. Either
  // way the tree ends on the node's current matrix. Removing the queued entry
  // instead could not handle the second case: the flush already holds it.
  if (SceneGraph *sg = getSceneGraph())
    sg->publisher().publish_matrix(m_pathMatrix, m_transform->GetData());
}

void GraphicsNode::setPosition(double x, double y, double z) {
  m_transform->SetElement(0, 3, x);
  m_transform->SetElement(1, 3, y);
  m_transform->SetElement(2, 3, z);

  // Publish rather than write. The value still reaches the state tree, just on
  // the publisher's cadence and coalesced, so posing a node no longer pays for a
  // path lookup, a signal, and the echo-driven second cascade.
  if (!m_pathPosition.empty()) {
    std::ostringstream oss;
    oss << std::setprecision(17); // exact round-trip through the state tree
    oss << x << "," << y << "," << z;
    m_echoPosition = oss.str();
    // Publish through THIS scene's publisher (no process-wide singleton). A node
    // not yet attached to a scene has no publisher, so it writes state directly —
    // the slow path, but such a node is not on the animation hot loop.
    if (SceneGraph *sg = getSceneGraph())
      sg->publisher().publish(m_pathPosition, m_echoPosition);
    else
      cvc::state::instance(app())(m_pathPosition).value(m_echoPosition);
  }

  updateTransform();
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
  oss << std::setprecision(17); // exact round-trip through the state tree
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
  oss << std::setprecision(17); // exact round-trip through the state tree
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
  supersedePublishedMatrix();

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

namespace {
// Apply a 4x4 to a 3-point, normalising the homogeneous w so a projective
// matrix is handled; for an affine scene transform w stays 1 and this is a plain
// matrix-times-point. in/out may alias. Takes a non-const matrix because
// vtkMatrix4x4::MultiplyPoint is a non-const member (it mutates nothing); the
// smart-pointer members yield a non-const raw pointer even from a const method.
void transform_point(vtkMatrix4x4 *m, const double in[3], double out[3]) {
  const double h[4] = {in[0], in[1], in[2], 1.0};
  double r[4];
  m->MultiplyPoint(h, r);
  const double w = r[3];
  const double inv = (w != 0.0) ? 1.0 / w : 1.0;
  out[0] = r[0] * inv;
  out[1] = r[1] * inv;
  out[2] = r[2] * inv;
}

// Transform an axis-aligned bbox by a 4x4 and re-fit an AABB around the eight
// transformed corners. A degenerate/empty input (any min > max) is returned
// unchanged so it stays recognisably empty. (min/max by ternary to avoid a
// dependency on <algorithm> here.)
cvc::bounding_box transform_bbox(vtkMatrix4x4 *m, const cvc::bounding_box &b) {
  if (b[0] > b[3] || b[1] > b[4] || b[2] > b[5])
    return b;
  const double xs[2] = {b[0], b[3]}, ys[2] = {b[1], b[4]}, zs[2] = {b[2], b[5]};
  double mn[3] = {0, 0, 0}, mx[3] = {0, 0, 0};
  for (int ci = 0; ci < 8; ++ci) {
    const double in[3] = {xs[ci & 1], ys[(ci >> 1) & 1], zs[(ci >> 2) & 1]};
    double out[3];
    transform_point(m, in, out);
    for (int a = 0; a < 3; ++a) {
      if (ci == 0 || out[a] < mn[a])
        mn[a] = out[a];
      if (ci == 0 || out[a] > mx[a])
        mx[a] = out[a];
    }
  }
  return cvc::bounding_box(mn[0], mn[1], mn[2], mx[0], mx[1], mx[2]);
}
} // namespace

void GraphicsNode::localToWorld(const double local[3], double world[3]) const {
  transform_point(m_worldMatrix, local, world);
}

void GraphicsNode::worldToLocal(const double world[3], double local[3]) const {
  if (m_worldInverseDirty) {
    m_worldInverse->DeepCopy(m_worldMatrix);
    m_worldInverse->Invert();
    m_worldInverseDirty = false;
  }
  transform_point(m_worldInverse, world, local);
}

cvc::world_units::coordinate GraphicsNode::localPointToReal(const double local[3],
                                                            const cvc::world_units &units) const {
  double world[3];
  localToWorld(local, world);
  return units.world_point_to_real(world[0], world[1], world[2]);
}

cvc::bounding_box GraphicsNode::getWorldBoundingBox() const {
  // getBoundingBox() is this node's untransformed local box; m_worldMatrix is the
  // whole chain of local transforms up to the root, kept current top-down.
  return transform_bbox(m_worldMatrix, getBoundingBox());
}

cvc::bounding_box GraphicsNode::getCombinedWorldBoundingBox() const {
  // getCombinedBoundingBox() is already this-node-plus-descendants in THIS node's
  // local space, so one push through the world matrix puts the whole subtree in
  // world space.
  return transform_bbox(m_worldMatrix, getCombinedBoundingBox());
}

cvc::world_units::coordinate GraphicsNode::realDimensions(const cvc::world_units &units,
                                                          bool includeChildren) const {
  const cvc::bounding_box b =
      includeChildren ? getCombinedWorldBoundingBox() : getWorldBoundingBox();
  // Degenerate/empty box -> zero size in the regime's base unit.
  if (b[0] > b[3] || b[1] > b[4] || b[2] > b[5])
    return units.world_point_to_real(0.0, 0.0, 0.0);
  // World-space AABB extents (already reflect the full chain's scale), converted
  // to the regime with one shared unit across the three axes.
  return units.world_point_to_real(b[3] - b[0], b[4] - b[1], b[5] - b[2]);
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
  // The cached inverse is now stale; recompute it lazily on the next query
  // rather than paying an Invert() here on every node of every pose cascade.
  m_worldInverseDirty = true;

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

  // Planes this node owns move with it, in place (every renderer clipped by them
  // holds these same objects).
  if (!m_ownClip.empty())
    updateOwnClipWorld();
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
  // Recognise our own pose coming back BEFORE marshalling anything.
  //
  // The echo check further down (posStr == m_echoPosition) cannot do it. That
  // lambda runs whenever the owner thread next pumps, which may be several
  // poses later, and it compares the value the tree holds NOW against the
  // node's LATEST published value. Pose a node twice with a publisher flush
  // landing in between and those are two different poses: the guard reads a
  // mismatch, concludes a script moved the node, and rewinds it onto the older
  // pose — dragging the subtree with it. That is one-run-in-six flakiness in
  // anything that moves a group twice between pumps.
  //
  // Here we are still inside the write, so the question is answerable exactly:
  // a position write arriving during a flush of THIS scene's publisher is this
  // node's own, stale or not. That publisher's queue only holds what this
  // scene's nodes published, and a node publishes only its own paths. Dropping
  // it loses nothing — setPosition() already applied the pose to m_transform.
  //
  // The same holds for "matrix", which setPoseMatrix publishes: the pose is
  // already applied, so its echo -- current or stale -- is dropped here before
  // it can re-parse 16 numbers and run the cascade a second time. (setTransform
  // writes "matrix" directly rather than through the publisher, so its echo
  // still takes the marshalled path below exactly as before.)
  //
  // It must be this scene's publisher, not just any. A node in another scene
  // bound to the same path (a mirror view built with the same state prefix)
  // sees the write during a flush too. For that node it is a foreign pose to
  // follow, exactly as it follows a setTransform.
  //
  // Everything else keeps going through the check below: only position and
  // the pose matrix publish, and a node with no scene writes state directly
  // and echoes back inline on this same thread, where the values do line up.
  if (childState == "position" || childState == "matrix") {
    if (const state_publisher *flushing = state_publisher::flushing()) {
      SceneGraph *sg = getSceneGraph();
      if (sg && flushing == &sg->publisher())
        return;
    }
  }
  // A scene-less node's setPoseMatrix writing its own value (see
  // publishPoseMatrix): the echo arrives here, inline, during that write.
  if (childState == "matrix" && m_writingPose)
    return;
  // The metadata mirror is written by setMetadata and read by nobody here: it
  // changes nothing drawn, so there is nothing to marshal or redraw -- for this
  // node's own ("metadata.k") or, as every ancestor hears it too, a
  // descendant's ("children.n.metadata.k").
  if (childState.compare(0, 9, "metadata.") == 0 ||
      childState.find(".metadata.") != std::string::npos)
    return;

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
        if (posStr == m_echoPosition)
          return; // our own published value coming back; the node already has it
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
        // A pose this node published before the write must not land on top
        // of it at the next flush.
        supersedePublishedMatrix();
        updateTransform();
      } catch (const boost::bad_lexical_cast &) {
      }
    } else if (childState == "clip_children") {
      int clip = getState("clip_children").value<int>();
      setClipChildren(clip != 0);
    } else if (childState == "clip_planes") {
      // px,py,pz,nx,ny,nz per plane; anything malformed (a count not a multiple
      // of six, a non-number) is ignored and the planes stay as they are.
      std::vector<double> v;
      std::istringstream iss(getState("clip_planes").value<std::string>());
      std::string tok;
      bool ok = true;
      while (ok && std::getline(iss, tok, ',')) {
        try {
          std::size_t used = 0;
          v.push_back(std::stod(tok, &used));
          ok = tok.find_first_not_of(" \t", used) == std::string::npos;
        } catch (const std::exception &) {
          ok = false;
        }
      }
      if (ok && v.size() % 6 == 0) {
        std::vector<ClipPlane> planes(v.size() / 6);
        for (std::size_t i = 0; i < planes.size(); ++i)
          for (int k = 0; k < 3; ++k) {
            planes[i].origin[k] = v[6 * i + k];
            planes[i].normal[k] = v[6 * i + 3 + k];
          }
        applyOwnClip(planes);
      }
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
    if (SceneGraph *sg = getSceneGraph()) {
      sg->requestRender();
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
  child->setSceneGraph(getSceneGraph());

  // A non-casting subtree stays non-casting as it grows (LOD rungs, parts).
  if (!castsShadow())
    child->setCastsShadow(false);

  // Also add as SceneNode child so it gets rendered
  addChild(child);

  // Update child's transform to reflect new parent
  child->updateTransform();

  // Clipped by whatever clips everything below this node (if anything does).
  if (!m_ownClip.empty() || m_clipChildren || !m_inheritedClip.empty())
    child->runOnMainThread([this, child]() {
      child->m_inheritedClip = clipPassdown();
      child->propagateClip();
    });

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
    // Leaving: no longer clipped by this node or its ancestors.
    child->runOnMainThread([child]() {
      if (child->m_inheritedClip.empty())
        return;
      child->m_inheritedClip.clear();
      child->propagateClip();
    });
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
  // Every node includes its own box, except a NullGraphicNode told not to.
  const bool includeOwnBounds = m_combinedIncludesOwnBounds;

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

namespace {
// The metadata value types mirrored into the state tree. Each is written TYPED
// (state::value<T>), so the tree records the C++ type, as it always has. long
// and the unsigned/float types are what a Python int or a size_t arrive as.
template <typename T> bool any_is(const std::any &a) { return a.type() == typeid(T); }

template <typename T> bool any_equal_as(const std::any &a, const std::any &b) {
  return any_is<T>(a) && any_is<T>(b) && std::any_cast<T>(a) == std::any_cast<T>(b);
}

// Same type and same value, for the types above; anything else counts as
// changed (std::any has no equality of its own).
bool metadata_equal(const std::any &a, const std::any &b) {
  if (a.type() != b.type())
    return false;
  if (any_is<const char *>(a)) {
    const char *x = std::any_cast<const char *>(a), *y = std::any_cast<const char *>(b);
    return x && y && std::string(x) == y;
  }
  return any_equal_as<int>(a, b) || any_equal_as<long>(a, b) || any_equal_as<long long>(a, b) ||
         any_equal_as<unsigned int>(a, b) || any_equal_as<unsigned long>(a, b) ||
         any_equal_as<unsigned long long>(a, b) || any_equal_as<double>(a, b) ||
         any_equal_as<float>(a, b) || any_equal_as<bool>(a, b) || any_equal_as<std::string>(a, b);
}

template <typename T> bool write_as(cvc::state &s, const std::any &v) {
  if (!any_is<T>(v))
    return false;
  s.value(std::any_cast<T>(v));
  return true;
}

// Write `v` to `s` if it is one of the mirrored types; false for any other type.
bool write_metadata(cvc::state &s, const std::any &v) {
  if (any_is<const char *>(v)) {
    const char *c = std::any_cast<const char *>(v);
    s.value(std::string(c ? c : ""));
    return true;
  }
  return write_as<int>(s, v) || write_as<long>(s, v) || write_as<long long>(s, v) ||
         write_as<unsigned int>(s, v) || write_as<unsigned long>(s, v) ||
         write_as<unsigned long long>(s, v) || write_as<double>(s, v) || write_as<float>(s, v) ||
         write_as<bool>(s, v) || write_as<std::string>(s, v);
}

bool is_mirrored_metadata(const std::any &v) {
  return any_is<int>(v) || any_is<long>(v) || any_is<long long>(v) || any_is<unsigned int>(v) ||
         any_is<unsigned long>(v) || any_is<unsigned long long>(v) || any_is<double>(v) ||
         any_is<float>(v) || any_is<bool>(v) || any_is<std::string>(v) || any_is<const char *>(v);
}
} // namespace

void GraphicsNode::setMetadata(const std::string &key, const std::any &value) {
  // Unchanged: nothing to do. GeometryNode::updateMetadata re-sends ~17 keys on
  // every setGeometry, and a mesh rebuilt in place (a wall layer that grows by a
  // few cells) keeps most of them. Each skipped key saves a state path lookup and
  // a write.
  auto it = m_metadata.find(key);
  if (it != m_metadata.end() && metadata_equal(it->second, value))
    return;
  m_metadata[key] = value;
  if (!is_mirrored_metadata(value))
    return; // kept on the node only, as before

  // Mirror into the state tree, where a state browser shows it. The mirror is
  // READ-ONLY to everyone else: the node never reads it back (handleStateChanged
  // has no metadata branch), so an edit made there would only make the tree lie
  // about the node -- the lock keeps a dashboard or script from doing that, and
  // makes VolRover3's state tree show the key as locked. The node itself is the
  // owner, so it lifts the lock for its own write and puts it back, as
  // VolRover3's AppState does for world_bounds. Writing through the lock used to
  // throw read_only_error on every write after the first; the throw was
  // swallowed, so the mirror kept the FIRST value forever and each re-mesh paid
  // for ~17 exceptions.
  try {
    cvc::state &s = getState("metadata." + key); // one path lookup per write
    struct relock {
      cvc::state &s;
      ~relock() {
        try {
          s.readOnly(true);
        } catch (...) {
        }
      }
    } guard{s};
    s.readOnly(false); // no-op (no signal) for a key written for the first time
    write_metadata(s, value);
  } catch (...) {
    // The mirror is best effort; the node's own copy above is authoritative.
  }
}

namespace {
// setCastsShadow's flag lives ON the vtkProp, in its PropertyKeys -- where VTK's
// render passes keep their own per-prop keys -- so the shadow baker can read it
// from the prop alone, for a node's actor and a host's raw prop alike.
vtkInformationIntegerKey *nonCasterKey() {
  static vtkInformationIntegerKey *key =
      new vtkInformationIntegerKey("NON_SHADOW_CASTER", "cvc::gl::GraphicsNode");
  return key;
}
} // namespace

void GraphicsNode::setPropCastsShadow(vtkProp *prop, bool casts) {
  if (!prop)
    return;
  vtkInformation *info = prop->GetPropertyKeys();
  if (casts) {
    if (info)
      info->Remove(nonCasterKey());
    return;
  }
  if (!info) {
    auto fresh = vtkSmartPointer<vtkInformation>::New();
    prop->SetPropertyKeys(fresh);
    info = fresh;
  }
  info->Set(nonCasterKey(), 1);
}

bool GraphicsNode::propCastsShadow(vtkProp *prop) {
  vtkInformation *info = prop ? prop->GetPropertyKeys() : nullptr;
  return !(info && info->Has(nonCasterKey()));
}

void GraphicsNode::setCastsShadow(bool casts) {
  // All of it on the owner thread: the flag and the child list are what
  // addGraphicsChild reads and grows there, and the key is VTK state.
  runOnMainThread([this, casts]() {
    m_castsShadow.store(casts, std::memory_order_relaxed);
    setPropCastsShadow(getProp(), casts);
    for (auto &child : m_graphicsChildren)
      if (child)
        child->setCastsShadow(casts); // owner thread: inline
  });
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

bool GraphicsNode::isVisibleInHierarchy() const {
  for (const GraphicsNode *n = this; n; n = n->m_parent)
    if (!n->isVisible())
      return false;
  return true;
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

void GraphicsNode::ownBoundsChanged() {
  // Nothing to re-fit (a hidden outline is re-fitted when shown, by setShowBBox):
  // the common case, and some callers report a new box every frame.
  if (!m_showBBox && !m_clipChildren)
    return;
  // On the owner thread: some callers are loader threads (VolSliceNode::setVolume,
  // VolRenNode's state-settings apply), and both updates walk this node's children.
  runOnMainThread([this]() {
    updateBoundingBoxNode();
    // The planes this node clips its children to are its own box's faces.
    if (m_clipChildren)
      updateClipPlanes();
  });
}

void GraphicsNode::setClipChildren(bool clip) {
  if (m_clipChildren == clip)
    return;

  m_clipChildren = clip;

  // Update state tree
  getState("clip_children").value(clip ? 1 : 0);

  runOnMainThread([this]() {
    if (m_clipChildren)
      updateClipPlanes();
    propagateClip(); // everything below gains (or loses) the box planes
  });
}

std::vector<GraphicsNode::ClipPlane> GraphicsNode::boxClipPlanes(const cvc::bounding_box &b) {
  const double cx = (b.minx + b.maxx) / 2, cy = (b.miny + b.maxy) / 2, cz = (b.minz + b.maxz) / 2;
  // Order: +X, -X, +Y, -Y, +Z, -Z faces, each normal pointing into the box.
  return {{{b.maxx, cy, cz}, {-1, 0, 0}}, {{b.minx, cy, cz}, {1, 0, 0}},
          {{cx, b.maxy, cz}, {0, -1, 0}}, {{cx, b.miny, cz}, {0, 1, 0}},
          {{cx, cy, b.maxz}, {0, 0, -1}}, {{cx, cy, b.minz}, {0, 0, 1}}};
}

void GraphicsNode::setClipPlanes(const std::vector<ClipPlane> &planes) {
  // Mirror into the state tree (exactly: 17 significant digits round-trip a
  // double), then apply. The handler that the write fires parses the same
  // planes back and finds nothing to do.
  std::ostringstream oss;
  oss << std::setprecision(17);
  for (std::size_t i = 0; i < planes.size(); ++i)
    for (int k = 0; k < 6; ++k)
      oss << (i || k ? "," : "") << (k < 3 ? planes[i].origin[k] : planes[i].normal[k - 3]);
  runOnMainThread([this, planes]() { applyOwnClip(planes); });
  getState("clip_planes").value(oss.str());
}

void GraphicsNode::applyOwnClip(const std::vector<ClipPlane> &planes) {
  if (planes == m_ownClip)
    return;
  const bool sameCount = planes.size() == m_ownClip.size();
  m_ownClip = planes;
  if (!sameCount) {
    // A different SET: new plane objects, re-handed to this node and below.
    m_ownClipWorld.clear();
    for (std::size_t i = 0; i < planes.size(); ++i)
      m_ownClipWorld.push_back(vtkSmartPointer<vtkPlane>::New());
  }
  updateOwnClipWorld(); // same count: just moved, in place
  if (!sameCount)
    propagateClip();
  if (SceneGraph *sg = getSceneGraph())
    sg->requestRender();
}

void GraphicsNode::updateOwnClipWorld() {
  // Origins by the world matrix; normals by its inverse transpose (correct under
  // non-uniform scale), normalized.
  vtkSmartPointer<vtkMatrix4x4> normalMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
  normalMatrix->DeepCopy(m_worldMatrix);
  normalMatrix->Invert();
  normalMatrix->Transpose();
  for (std::size_t i = 0; i < m_ownClip.size() && i < m_ownClipWorld.size(); ++i) {
    const ClipPlane &p = m_ownClip[i];
    const double o[4] = {p.origin[0], p.origin[1], p.origin[2], 1.0};
    const double n[4] = {p.normal[0], p.normal[1], p.normal[2], 0.0};
    double wo[4], wn[4];
    m_worldMatrix->MultiplyPoint(o, wo);
    normalMatrix->MultiplyPoint(n, wn);
    const double len = std::sqrt(wn[0] * wn[0] + wn[1] * wn[1] + wn[2] * wn[2]);
    if (len > 0.0)
      for (int k = 0; k < 3; ++k)
        wn[k] /= len;
    m_ownClipWorld[i]->SetOrigin(wo[0], wo[1], wo[2]);
    m_ownClipWorld[i]->SetNormal(wn[0], wn[1], wn[2]);
  }
}

std::vector<vtkSmartPointer<vtkPlane>> GraphicsNode::clipPassdown() const {
  std::vector<vtkSmartPointer<vtkPlane>> down(m_ownClipWorld.begin(), m_ownClipWorld.end());
  if (m_clipChildren)
    down.insert(down.end(), m_clipPlaneArray.begin(), m_clipPlaneArray.end());
  down.insert(down.end(), m_inheritedClip.begin(), m_inheritedClip.end());
  return down;
}

void GraphicsNode::propagateClip() {
  // What this node is clipped by, nearest first, capped by what its renderer
  // honours.
  std::vector<vtkPlane *> mine;
  for (const auto &p : m_ownClipWorld)
    mine.push_back(p);
  for (const auto &p : m_inheritedClip)
    mine.push_back(p);
  const std::size_t cap = static_cast<std::size_t>(std::max(0, maxClipPlanes()));
  if (mine.size() > cap) {
    if (cap > 0 && !m_clipCapWarned) {
      m_clipCapWarned = true;
      app().log(1, "GraphicsNode[" + getName() + "]: clipped by " + std::to_string(mine.size()) +
                       " planes but its renderer honours " + std::to_string(cap) +
                       "; the nearest " + std::to_string(cap) + " apply");
    }
    mine.resize(cap);
  }
  if (mine != m_appliedClipList) {
    m_appliedClipList = mine;
    if (mine.empty()) {
      applyClipPlanes(nullptr);
    } else {
      // A fresh collection whenever the set changes, so a renderer that
      // compares the pointer it holds (VTK's setters do) sees the change.
      m_appliedClip = vtkSmartPointer<vtkPlaneCollection>::New();
      for (vtkPlane *p : mine)
        m_appliedClip->AddItem(p);
      applyClipPlanes(m_appliedClip);
    }
  }

  // Everything below: re-handed only where what it inherits changed.
  const std::vector<vtkSmartPointer<vtkPlane>> down = clipPassdown();
  for (auto &child : m_graphicsChildren) {
    if (!child || child->m_inheritedClip == down)
      continue;
    child->m_inheritedClip = down;
    child->propagateClip();
  }
}

int GraphicsNode::clipPlaneCount() const {
  return static_cast<int>(m_ownClipWorld.size() + m_inheritedClip.size());
}

vtkPlaneCollection *GraphicsNode::getAppliedClipPlanes() const {
  return m_appliedClipList.empty() ? nullptr : m_appliedClip.Get();
}

void GraphicsNode::updateClipPlanes() {
  // Get this node's OWN bounding box (not combined)
  cvc::bounding_box bbox = getBoundingBox();
  double bounds[6] = {bbox.minx, bbox.maxx, bbox.miny, bbox.maxy, bbox.minz, bbox.maxz};

  // Get world transform to transform the planes
  vtkSmartPointer<vtkMatrix4x4> worldTransform = getWorldTransform();

  // Define 6 plane normals in local space, each pointing INTO the box. VTK keeps
  // the half-space a clipping plane's normal points into: a point x survives
  // where n . (x - origin) >= 0 (vtkOpenGLPolyDataMapper discards a fragment
  // whose clip distance is negative; the GPU volume ray caster clips the same
  // side). Six inward planes keep the inside of the box; six outward ones would
  // keep nothing.
  // Order: +X, -X, +Y, -Y, +Z, -Z
  double normals[6][3] = {
      {-1.0, 0.0, 0.0}, // +X face (points inward: -X)
      {1.0, 0.0, 0.0},  // -X face (points inward: +X)
      {0.0, -1.0, 0.0}, // +Y face (points inward: -Y)
      {0.0, 1.0, 0.0},  // -Y face (points inward: +Y)
      {0.0, 0.0, -1.0}, // +Z face (points inward: -Z)
      {0.0, 0.0, 1.0}   // -Z face (points inward: +Z)
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

  // The children hold these same plane objects: moved in place, nothing to re-hand.
}

void GraphicsNode::applyClipPlanes(vtkPlaneCollection *planes) {
  // Base implementation does nothing
  // Subclasses that support clipping (GeometryNode, VolumeNode, GridNode)
  // override this
}

} // namespace gl
} // namespace cvc
