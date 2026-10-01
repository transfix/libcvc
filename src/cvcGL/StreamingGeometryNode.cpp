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

  // Draw-time uniforms + the subclass shader hook, in every pass.
  m_uniformCb = vtkSmartPointer<vtkCallbackCommand>::New();
  m_uniformCb->SetClientData(this);
  m_uniformCb->SetCallback(&StreamingGeometryNode::onUpdateShader);
  mapper()->AddObserver(vtkCommand::UpdateShaderEvent, m_uniformCb);
  m_core->beforeDraw = [this](vtkRenderer *ren) { this->beforeDraw(ren); };
}

StreamingGeometryNode::~StreamingGeometryNode() {
  // The mapper is ref-counted and may outlive this node inside a renderer; a
  // later draw must not call back into freed state (cf. ~GeometryNode).
  if (mapper() && m_uniformCb)
    mapper()->RemoveObserver(m_uniformCb);
  if (m_core)
    m_core->beforeDraw = nullptr;
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
  }
  scheduleApply();
}

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

void StreamingGeometryNode::updateShaderProgram(vtkShaderProgram *) {}

void StreamingGeometryNode::requestRenderIfAttached() {
  if (SceneGraph *sg = getSceneGraph())
    sg->requestRender();
}

void StreamingGeometryNode::scheduleApply() {
  SceneGraph *sg = getSceneGraph();
  if (!sg || sg->onOwnerThread()) {
    applyPending(); // owner thread (or no scene yet): nothing to marshal
    return;
  }
  // One apply in flight per node. A write that races the apply's start either
  // lands in it (it reads the staging under the lock) or posts the next one.
  if (m_applyPosted.exchange(true))
    return;
  std::weak_ptr<SceneNode> weak = weak_from_this();
  sg->postEventCoalesced(this, [weak]() {
    if (auto self = weak.lock())
      static_cast<StreamingGeometryNode *>(self.get())->applyPending();
  });
}

void StreamingGeometryNode::applyPending() {
  m_applyPosted.store(false);
  bool layoutChanged = false, boundsChanged = false, rangeChanged = false;
  std::size_t lo = 0, hi = 0, drawFirst = 0, drawCount = kAllTriangles;
  cvc::bounding_box bounds;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
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
      boundsChanged = true;
    }
    if (m_drawRangePending) {
      drawFirst = m_drawFirst;
      drawCount = m_drawCount;
      m_drawRangePending = false;
      rangeChanged = true;
    }
  }
  ++m_applies;
  if (!m_core)
    return;
  if (hi > lo)
    m_core->markPoints(static_cast<vtkIdType>(lo), static_cast<vtkIdType>(hi - lo));
  if (boundsChanged) {
    double b[6];
    toArray(bounds, b);
    m_core->setReservedBounds(b);
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
