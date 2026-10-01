/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <algorithm>
#include <cstring>
#include <cvc/gl/SceneEventSink.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/StreamingGeometryNode.h>
#include <cvc/gl/StreamingMappers.h>
#include <stdexcept>
#include <vtkActor.h>
#include <vtkCallbackCommand.h>
#include <vtkCellArray.h>
#include <vtkCommand.h>
#include <vtkFloatArray.h>
#include <vtkIdTypeArray.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkShaderProgram.h>

namespace cvc {
namespace gl {

namespace {
void validateLayout(const StreamingLayout &layout) {
  for (const auto &t : layout.triangles)
    for (std::uint32_t i : t)
      if (static_cast<std::size_t>(i) >= layout.capacity_points)
        throw std::invalid_argument("StreamingGeometryNode: triangle index " + std::to_string(i) +
                                    " is outside the layout's " +
                                    std::to_string(layout.capacity_points) + "-point capacity");
}

void toArray(const cvc::bounding_box &b, double out[6]) {
  out[0] = b.minx;
  out[1] = b.maxx;
  out[2] = b.miny;
  out[3] = b.maxy;
  out[4] = b.minz;
  out[5] = b.maxz;
}

bool sameBox(const cvc::bounding_box &a, const cvc::bounding_box &b) {
  return a.minx == b.minx && a.miny == b.miny && a.minz == b.minz && a.maxx == b.maxx &&
         a.maxy == b.maxy && a.maxz == b.maxz;
}
} // namespace

StreamingGeometryNode::StreamingGeometryNode(cvc::app &ctx, const std::string &statePath,
                                             const std::string &name, const StreamingLayout &layout)
    : GeometryNode(ctx, statePath, name, newStreamingMapper(layout.mapper, nullptr)),
      m_kind(StreamingLowMemoryPolyDataMapper::SafeDownCast(mapper())
                 ? StreamingMapperKind::LowMemory
                 : StreamingMapperKind::Classic),
      m_core(streamingCore(mapper())) {
  validateLayout(layout);
  m_staging.assign(3 * layout.capacity_points, 0.0f);
  m_stagingCapacity = layout.capacity_points;
  m_bounds = layout.reserved_bounds;
  buildPolyData(layout, m_staging.data(), layout.capacity_points);
  double b[6];
  toArray(m_bounds, b);
  m_core->setReservedBounds(b);
  m_appliedBounds = m_bounds;

  // Draw-time uniforms + the subclass shader hook, in every pass.
  m_uniformCb = vtkSmartPointer<vtkCallbackCommand>::New();
  m_uniformCb->SetClientData(this);
  m_uniformCb->SetCallback(&StreamingGeometryNode::onUpdateShader);
  mapper()->AddObserver(vtkCommand::UpdateShaderEvent, m_uniformCb);
  m_core->beforeDraw = [this](vtkRenderer *ren) { this->beforeDraw(ren); };
  m_core->beforeBounds = [this]() { this->beforeComputeBounds(); };

  // CPU pickers would hit the polydata, not what is drawn (see the header).
  actor()->PickableOff();
}

StreamingGeometryNode::~StreamingGeometryNode() {
  // The mapper is ref-counted and may outlive this node inside a renderer; a
  // later draw must not call back into freed state (cf. ~GeometryNode).
  if (mapper() && m_uniformCb)
    mapper()->RemoveObserver(m_uniformCb);
  if (m_core) {
    m_core->beforeDraw = nullptr;
    m_core->beforeBounds = nullptr;
  }
}

void StreamingGeometryNode::buildPolyData(const StreamingLayout &layout, const float *points,
                                          std::size_t nPoints) {
  const vtkIdType cap = static_cast<vtkIdType>(layout.capacity_points);
  auto xyz = vtkSmartPointer<vtkFloatArray>::New();
  xyz->SetNumberOfComponents(3);
  xyz->SetNumberOfTuples(cap);
  float *dst = xyz->GetPointer(0);
  const std::size_t keep = std::min<std::size_t>(nPoints, layout.capacity_points);
  if (keep)
    std::memcpy(dst, points, keep * 3 * sizeof(float));
  std::fill(dst + 3 * keep, dst + 3 * cap, 0.0f);
  auto pts = vtkSmartPointer<vtkPoints>::New();
  pts->SetData(xyz);

  // Raw offsets/connectivity: one allocation each, no per-cell insert calls.
  const vtkIdType nTris = static_cast<vtkIdType>(layout.triangles.size());
  auto offsets = vtkSmartPointer<vtkIdTypeArray>::New();
  offsets->SetNumberOfValues(nTris + 1);
  auto conn = vtkSmartPointer<vtkIdTypeArray>::New();
  conn->SetNumberOfValues(3 * nTris);
  for (vtkIdType t = 0; t < nTris; ++t) {
    offsets->SetValue(t, 3 * t);
    for (int k = 0; k < 3; ++k)
      conn->SetValue(3 * t + k, static_cast<vtkIdType>(layout.triangles[t][k]));
  }
  offsets->SetValue(nTris, 3 * nTris);
  auto polys = vtkSmartPointer<vtkCellArray>::New();
  polys->SetData(offsets, conn);

  // One constant normal: overlays are flat, and a mesh without normals takes
  // VTK's unlit path, where the shadow snippet does not compile (GeometryNode).
  auto nrm = vtkSmartPointer<vtkFloatArray>::New();
  nrm->SetName("Normals");
  nrm->SetNumberOfComponents(3);
  nrm->SetNumberOfTuples(cap);
  float *n = nrm->GetPointer(0);
  for (vtkIdType i = 0; i < cap; ++i) {
    n[3 * i + 0] = layout.normal[0];
    n[3 * i + 1] = layout.normal[1];
    n[3 * i + 2] = layout.normal[2];
  }

  vtkPolyData *pd = polyData();
  pd->Initialize();
  pd->SetPoints(pts);
  pd->SetPolys(polys);
  pd->GetPointData()->SetNormals(nrm);
  pd->Modified();
  m_triangleCount = layout.triangles.size();
}

std::size_t StreamingGeometryNode::capacityPoints() const {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_stagingCapacity;
}

std::size_t StreamingGeometryNode::triangleCount() const {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_pendingLayout ? m_pendingLayout->triangles.size() : m_triangleCount;
}

void StreamingGeometryNode::writePoints(std::size_t firstPoint, const float *xyz, std::size_t n) {
  if (n == 0)
    return;
  if (!xyz)
    throw std::invalid_argument("StreamingGeometryNode::writePoints: null xyz");
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (firstPoint > m_stagingCapacity || n > m_stagingCapacity - firstPoint)
      throw std::out_of_range("StreamingGeometryNode::writePoints[" + getName() + "]: points [" +
                              std::to_string(firstPoint) + ", " + std::to_string(firstPoint + n) +
                              ") exceed the " + std::to_string(m_stagingCapacity) +
                              "-point capacity");
    std::memcpy(m_staging.data() + 3 * firstPoint, xyz, n * 3 * sizeof(float));
    if (m_dirtyHi <= m_dirtyLo) {
      m_dirtyLo = firstPoint;
      m_dirtyHi = firstPoint + n;
    } else {
      m_dirtyLo = std::min(m_dirtyLo, firstPoint);
      m_dirtyHi = std::max(m_dirtyHi, firstPoint + n);
    }
  }
  ++m_writes;
  m_pointsWritten += n;
  scheduleApply();
}

void StreamingGeometryNode::setDrawRange(std::size_t firstTri, std::size_t triCount) {
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_drawRangePending = true;
    m_drawFirst = firstTri;
    m_drawCount = triCount;
  }
  scheduleApply();
}

void StreamingGeometryNode::setReservedBounds(const cvc::bounding_box &bounds) {
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_bounds = bounds;
    m_boundsPending = true;
    m_boundsPinned = true;
  }
  scheduleApply();
}

void StreamingGeometryNode::growReservedBounds(const cvc::bounding_box &b) {
  bool grew = false;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    // Field by field: cvc::bounding_box's own union drops zero-volume boxes, and
    // a flat ribbon's box is exactly that.
    cvc::bounding_box u = m_bounds;
    u.minx = std::min(u.minx, b.minx);
    u.miny = std::min(u.miny, b.miny);
    u.minz = std::min(u.minz, b.minz);
    u.maxx = std::max(u.maxx, b.maxx);
    u.maxy = std::max(u.maxy, b.maxy);
    u.maxz = std::max(u.maxz, b.maxz);
    grew = u.minx != m_bounds.minx || u.miny != m_bounds.miny || u.minz != m_bounds.minz ||
           u.maxx != m_bounds.maxx || u.maxy != m_bounds.maxy || u.maxz != m_bounds.maxz;
    if (grew) {
      m_bounds = u;
      m_boundsPending = true;
    }
  }
  if (grew)
    scheduleApply();
}

bool StreamingGeometryNode::stageDerivedBounds(const cvc::bounding_box &bounds) {
  if (!commitDerivedBounds(bounds))
    return false;
  scheduleApply();
  return true;
}

bool StreamingGeometryNode::commitDerivedBounds(const cvc::bounding_box &bounds) {
  std::lock_guard<std::mutex> lock(m_mutex);
  if (m_boundsPinned)
    return false;
  m_bounds = bounds;
  m_boundsPending = true;
  return true;
}

void StreamingGeometryNode::unpinReservedBounds() {
  std::lock_guard<std::mutex> lock(m_mutex);
  m_boundsPinned = false;
}

bool StreamingGeometryNode::reservedBoundsPinned() const {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_boundsPinned;
}

void StreamingGeometryNode::setDerivedBoundsNow(const cvc::bounding_box &bounds) {
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_boundsPinned)
      return;
    m_bounds = bounds; // a pending apply re-sends this same (latest) box
  }
  if (sameBox(bounds, m_appliedBounds))
    return; // what the mapper already reports
  if (m_core) {
    double b[6];
    toArray(bounds, b);
    m_core->setReservedBounds(b);
  }
  m_appliedBounds = bounds;
  // The box VTK culls and clips by moved: a camera holding still re-fits its
  // clipping range to it (CameraController::update watches contentVersion).
  if (SceneGraph *sg = getSceneGraph())
    sg->markContentChanged();
}

void StreamingGeometryNode::setPickable(bool pickable) {
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_pickable = pickable;
    m_pickablePending = true;
  }
  scheduleApply();
}

bool StreamingGeometryNode::pickable() const { return m_pickable.load(); }

cvc::bounding_box StreamingGeometryNode::reservedBounds() const {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_bounds;
}

cvc::bounding_box StreamingGeometryNode::getBoundingBox() const { return reservedBounds(); }

void StreamingGeometryNode::relayout(const StreamingLayout &layout) {
  validateLayout(layout);
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_staging.resize(3 * layout.capacity_points, 0.0f); // keeps the staged prefix
    m_stagingCapacity = layout.capacity_points;
    m_pendingLayout = std::make_unique<StreamingLayout>(layout);
    m_bounds = layout.reserved_bounds;
    m_boundsPending = true;
    // The relayout re-sends every point; a pending range is part of it.
    m_dirtyLo = m_dirtyHi = 0;
  }
  scheduleApply();
}

void StreamingGeometryNode::setUniform(const std::string &name, float v) {
  {
    std::lock_guard<std::mutex> lock(m_uniformMutex);
    UniformValue &u = m_uniforms[name];
    u.n = 1;
    u.v[0] = v;
  }
  requestRenderIfAttached();
}

void StreamingGeometryNode::setUniform(const std::string &name, float x, float y, float z) {
  {
    std::lock_guard<std::mutex> lock(m_uniformMutex);
    UniformValue &u = m_uniforms[name];
    u.n = 3;
    u.v[0] = x;
    u.v[1] = y;
    u.v[2] = z;
  }
  requestRenderIfAttached();
}

StreamStats StreamingGeometryNode::streamStats() const {
  StreamStats s;
  s.writes = m_writes.load();
  s.pointsWritten = m_pointsWritten.load();
  s.applies = m_applies.load();
  if (m_core) {
    s.uploads = m_core->partialUploads.load();
    s.uploadBytes = m_core->partialBytes.load();
    s.uploadCalls = m_core->partialGLCalls.load();
    s.fullUploads = m_core->fullUploads.load();
  }
  return s;
}

const char *StreamingGeometryNode::pointIdGLSL() const {
  return m_kind == StreamingMapperKind::LowMemory ? "pointId" : "gl_VertexID";
}

void StreamingGeometryNode::beforeDraw(vtkRenderer *) {}

void StreamingGeometryNode::beforeComputeBounds() {}

void StreamingGeometryNode::updateShaderProgram(vtkShaderProgram *) {}

void StreamingGeometryNode::requestRenderIfAttached() {
  // The locked sink, not a raw SceneGraph*: callable from producer threads
  // (setUniform) while the owner may be destroying the scene.
  if (std::shared_ptr<SceneEventSink> events = sceneEvents())
    events->requestRender();
}

void StreamingGeometryNode::onSceneGraphChanged() {
  if (!sceneEvents())
    return;
  bool pending = false;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    pending = hasPendingLocked();
  }
  if (pending)
    scheduleApply(); // what was staged while unattached (or posted to a dead scene)
}

bool StreamingGeometryNode::hasPendingLocked() const {
  return m_pendingLayout || m_dirtyHi > m_dirtyLo || m_boundsPending || m_drawRangePending ||
         m_pickablePending;
}

void StreamingGeometryNode::requestApply() { scheduleApply(); }

void StreamingGeometryNode::scheduleApply() {
  // Hold the scene's event sink, not a raw SceneGraph*: the owner thread may
  // destroy the scene while this (producer) thread is on its way to post. The
  // sink outlives it while held, and refuses the post once closed -- the staged
  // state then simply stays pending for the next attach.
  std::shared_ptr<SceneEventSink> events = sceneEvents();
  if (!events)
    return; // unattached: staged only, applied on the owner thread at attach
  if (events->onOwnerThread()) {
    applyPending(); // owner thread: nothing to marshal
    return;
  }
  // Every off-thread change posts; the scene keeps ONE slot per key until it
  // drains, and the apply reads whatever is staged when it runs. No "already
  // posted" flag: a slot that never runs (its scene destroyed first) must not
  // stop the next change from posting again.
  std::weak_ptr<SceneNode> weak = weak_from_this();
  const SceneEventSink *drainedBy = events.get(); // identity only, never dereferenced
  events->postCoalesced(&m_applyKey, [weak, drainedBy]() {
    auto self = weak.lock();
    if (!self)
      return;
    auto *node = static_cast<StreamingGeometryNode *>(self.get());
    // Only the scene the node is in NOW may apply, on its own owner thread.
    // A slot left behind in a scene the node has since moved out of (to one
    // that another thread may own) leaves the staging alone: the new scene got
    // its own apply when the node arrived (onSceneGraphChanged).
    if (node->sceneEvents().get() != drainedBy)
      return;
    node->applyPending();
  });
}

void StreamingGeometryNode::applyPending() {
  bool layoutChanged = false, boundsChanged = false, rangeChanged = false, pickChanged = false;
  bool pick = false;
  std::size_t lo = 0, hi = 0, drawFirst = 0, drawCount = kAllTriangles;
  cvc::bounding_box bounds;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!hasPendingLocked())
      return; // an earlier apply (inline, or another slot) already took it all
    if (m_pendingLayout) {
      // Topology change: rebuild the polydata (full upload at the next draw)
      // from every staged point.
      buildPolyData(*m_pendingLayout, m_staging.data(), m_stagingCapacity);
      m_pendingLayout.reset();
      layoutChanged = true;
    } else if (m_dirtyHi > m_dirtyLo) {
      vtkPoints *pts = polyData()->GetPoints();
      auto *fa = vtkFloatArray::SafeDownCast(pts ? pts->GetData() : nullptr);
      lo = m_dirtyLo;
      hi = std::min(m_dirtyHi, fa ? static_cast<std::size_t>(fa->GetNumberOfTuples()) : 0);
      if (fa && hi > lo)
        std::memcpy(fa->GetPointer(static_cast<vtkIdType>(3 * lo)), m_staging.data() + 3 * lo,
                    (hi - lo) * 3 * sizeof(float));
    }
    m_dirtyLo = m_dirtyHi = 0;
    if (m_boundsPending) {
      bounds = m_bounds;
      m_boundsPending = false;
      // Staged is not changed: setReservedBounds and a restyle's re-derived box
      // stage whatever they are given, often the very box already applied.
      boundsChanged = !sameBox(bounds, m_appliedBounds);
    }
    if (m_drawRangePending) {
      drawFirst = m_drawFirst;
      drawCount = m_drawCount;
      m_drawRangePending = false;
      rangeChanged = true;
    }
    if (m_pickablePending) {
      pick = m_pickable.load();
      m_pickablePending = false;
      pickChanged = true;
    }
  }
  ++m_applies;
  // A new box (grown over appended points, re-derived, re-laid out) is what VTK
  // culls and clips by, and it changes no MTime: tell the scene, so a camera
  // holding still re-fits its clipping range to it.
  if (boundsChanged || layoutChanged)
    if (SceneGraph *sg = getSceneGraph())
      sg->markContentChanged();
  if (pickChanged && actor())
    actor()->SetPickable(pick ? 1 : 0);
  // Streamed writes bump no MTime by design (that is what keeps the upload
  // partial), so a CASTING node says itself that what it draws changed: an
  // actor Modified() re-bakes the shadow maps at the scene's update interval
  // and re-uploads nothing.
  if ((hi > lo || layoutChanged || rangeChanged) && castsShadow() && actor())
    actor()->Modified();
  if (!m_core)
    return;
  if (hi > lo)
    m_core->markPoints(static_cast<vtkIdType>(lo), static_cast<vtkIdType>(hi - lo));
  if (boundsChanged) {
    double b[6];
    toArray(bounds, b);
    m_core->setReservedBounds(b);
    m_appliedBounds = bounds;
    updateBoundingBoxNode();
  }
  if (rangeChanged)
    m_core->setDrawRange(static_cast<vtkIdType>(std::min<std::size_t>(
                             drawFirst, static_cast<std::size_t>(VTK_ID_MAX))),
                         drawCount == kAllTriangles
                             ? -1
                             : static_cast<vtkIdType>(std::min<std::size_t>(
                                   drawCount, static_cast<std::size_t>(VTK_ID_MAX))));
  (void)layoutChanged;
  requestRenderIfAttached();
}

void StreamingGeometryNode::onUpdateShader(vtkObject *, unsigned long, void *clientData,
                                           void *callData) {
  auto *self = static_cast<StreamingGeometryNode *>(clientData);
  auto *program = static_cast<vtkShaderProgram *>(callData);
  if (!self || !program)
    return;
  {
    std::lock_guard<std::mutex> lock(self->m_uniformMutex);
    for (const auto &kv : self->m_uniforms) {
      if (!program->IsUniformUsed(kv.first.c_str()))
        continue;
      if (kv.second.n == 3)
        program->SetUniform3f(kv.first.c_str(), kv.second.v);
      else
        program->SetUniformf(kv.first.c_str(), kv.second.v[0]);
    }
  }
  self->updateShaderProgram(program);
}

} // namespace gl
} // namespace cvc
