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

// No vtkOpenGL*ErrorMacro anywhere in this file: those macros are inline in the
// including translation unit, so a cvcGL build without NDEBUG would issue a
// glGetError per draw -- a synchronous round trip on WebGL.
#include <atomic>
#include <cstdlib>
#include <cstring>
#include <cvc/gl/LowMemoryPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkCellType.h>
#include <vtkMath.h>
#include <vtkObjectFactory.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkShaderProgram.h>
#include <vtkVersionMacros.h>

#if VTK_MAJOR_VERSION == 9 && VTK_MINOR_VERSION == 5 && VTK_BUILD_VERSION == 0
#define CVC_GL_LOWMEM_REPLICA 1
#else
#define CVC_GL_LOWMEM_REPLICA 0
#endif

namespace cvc {
namespace gl {

namespace {

int drawPathFromEnvironment() {
  const char *v = std::getenv("CVCGL_LOWMEM_DRAW");
  if (v && std::strcmp(v, "stock") == 0)
    return static_cast<int>(LowMemoryPolyDataMapper::DrawPath::Stock);
  if (v && std::strcmp(v, "all") == 0)
    return static_cast<int>(LowMemoryPolyDataMapper::DrawPath::AllCellTypes);
  return static_cast<int>(LowMemoryPolyDataMapper::DrawPath::Fast);
}

std::atomic<int> &drawPathStore() {
  static std::atomic<int> path{drawPathFromEnvironment()};
  return path;
}

int policyFromEnvironment() {
  const char *v = std::getenv("CVCGL_LOWMEM_MAPPER");
  if (v && std::strcmp(v, "force") == 0)
    return static_cast<int>(LowMemoryMapperPolicy::Force);
  if (v && std::strcmp(v, "off") == 0)
    return static_cast<int>(LowMemoryMapperPolicy::Off);
  return static_cast<int>(LowMemoryMapperPolicy::Auto);
}

std::atomic<int> &policyStore() {
  static std::atomic<int> policy{policyFromEnvironment()};
  return policy;
}

// Does VTK's object factory hand out the low-memory mapper on this build? Asked
// once, on the first node construction (the OpenGL2 factory is registered by
// then: cvcGL's sources carry the module auto-init).
bool factoryIsLowMemory() {
  static const bool lowMemory =
      vtkSmartPointer<vtkPolyDataMapper>::New()->IsA("vtkOpenGLLowMemoryPolyDataMapper") != 0;
  return lowMemory;
}

#if CVC_GL_LOWMEM_REPLICA
// The depth offset vtkGLSLModCoincidentTopology::GetCoincidentParameters (VTK
// 9.5.0) computes for one primitive class, as the floats it would upload.
// slot: 0 points, 1 lines, 2 polygons. `set` false: the mod uploads nothing.
struct CoincidentValue {
  bool set = false;
  float factor = 0.0f, offset = 0.0f;
  bool operator==(const CoincidentValue &o) const {
    return set == o.set && (!set || (factor == o.factor && offset == o.offset));
  }
  bool operator!=(const CoincidentValue &o) const { return !(*this == o); }
};

CoincidentValue coincidentValue(vtkMapper *mapper, vtkProperty *prop, int slot) {
  float factor = 0.0f, offset = 0.0f;
  if (vtkMapper::GetResolveCoincidentTopology() == VTK_RESOLVE_SHIFT_ZBUFFER)
    offset = static_cast<float>(vtkMapper::GetResolveCoincidentTopologyZShift() * 4.0);
  if (vtkMapper::GetResolveCoincidentTopology() == VTK_RESOLVE_POLYGON_OFFSET ||
      (prop->GetEdgeVisibility() && prop->GetRepresentation() == VTK_SURFACE)) {
    double f = 0.0, u = 0.0;
    if (slot == 0)
      mapper->GetCoincidentTopologyPointOffsetParameter(u);
    else if (slot == 1)
      mapper->GetCoincidentTopologyLineOffsetParameters(f, u);
    else
      mapper->GetCoincidentTopologyPolygonOffsetParameters(f, u);
    factor = static_cast<float>(f);
    offset = static_cast<float>(u);
  }
  CoincidentValue v;
  v.set = factor != 0.0f || offset != 0.0f;
  v.factor = factor;
  v.offset = offset;
  return v;
}
#endif

} // namespace

vtkStandardNewMacro(LowMemoryPolyDataMapper);

LowMemoryPolyDataMapper::LowMemoryPolyDataMapper() {
#if VTK_MAJOR_VERSION == 9
  // VTK 9.5.0 declares these without initialisers (9.5.1 initialises the bool
  // only). With DISABLE_SHIFT_SCALE nothing ever assigns them, so whether the
  // mapper uploaded a (garbage) shift/scale copy of the positions depended on a
  // heap byte.
  this->CoordinateShiftAndScaleInUse = false;
  this->ShiftValues.fill(0.0);
  this->ScaleValues.fill(1.0);
#endif
}

void LowMemoryPolyDataMapper::setDrawPath(DrawPath path) {
  drawPathStore().store(static_cast<int>(path), std::memory_order_relaxed);
}

LowMemoryPolyDataMapper::DrawPath LowMemoryPolyDataMapper::drawPath() {
  return static_cast<DrawPath>(drawPathStore().load(std::memory_order_relaxed));
}

bool LowMemoryPolyDataMapper::replicaActive() { return CVC_GL_LOWMEM_REPLICA != 0; }

void LowMemoryPolyDataMapper::ReleaseGraphicsResources(vtkWindow *win) {
  // The cached program belongs to that window's context; look it up afresh.
  m_cache = nullptr;
  m_builtStamp = 0;
  Superclass::ReleaseGraphicsResources(win);
}

#if !CVC_GL_LOWMEM_REPLICA

void LowMemoryPolyDataMapper::RenderPieceDraw(vtkRenderer *ren, vtkActor *act) {
  ++m_stats.draws;
  ++m_stats.stockDraws;
  Superclass::RenderPieceDraw(ren, act);
}
void LowMemoryPolyDataMapper::readyProgram(vtkRenderer *ren, bool) {
  this->vtkDrawTexturedElements::ReadyShaderProgram(ren);
}
bool LowMemoryPolyDataMapper::renderable(int) const { return true; }
bool LowMemoryPolyDataMapper::coincidentSkipSafe(vtkActor *) const { return false; }
void LowMemoryPolyDataMapper::agentPreDraw(int, vtkRenderer *, vtkActor *) {}
void LowMemoryPolyDataMapper::agentDraw(int, vtkRenderer *, vtkActor *) {}

#else

void LowMemoryPolyDataMapper::RenderPieceDraw(vtkRenderer *ren, vtkActor *act) {
  ++m_stats.draws;
  const DrawPath path = drawPath();
  // Selection (picking state, point-picking sizes) and the vertex-visibility
  // pass stay on VTK's own path; the replica covers the plain draw only.
  if (path == DrawPath::Stock || ren->GetSelector() || act->GetProperty()->GetVertexVisibility()) {
    ++m_stats.stockDraws;
    Superclass::RenderPieceDraw(ren, act);
    return;
  }
  const bool fast = path == DrawPath::Fast;
  readyProgram(ren, fast);
  // Uniforms common to every cell type; also fires UpdateShaderEvent, which
  // GeometryNode's custom shader textures hang off.
  this->SetShaderParameters(ren, act);
  if (!this->ShaderProgram)
    return; // compile/link failure: VTK's agents would dereference null here
  bool skip = fast;
  if (skip && !coincidentSkipSafe(act)) {
    skip = false;
    ++m_stats.coincidentFallbacks;
  }
  for (int t = 0; t < 4; ++t) { // verts, lines, polys, strips: VTK's order
    if (skip && !renderable(t)) {
      ++m_stats.cellTypesSkipped;
      continue;
    }
    agentPreDraw(t, ren, act);
    agentDraw(t, ren, act);
    // vtkOpenGLLowMemoryCellTypeAgent::PostDraw; every 9.5.0 PostDrawInternal is empty.
    this->vtkDrawTexturedElements::PostDraw(ren, act, this);
  }
}

void LowMemoryPolyDataMapper::readyProgram(vtkRenderer *ren, bool allowSkip) {
  auto *win = vtkOpenGLRenderWindow::SafeDownCast(ren->GetRenderWindow());
  vtkOpenGLShaderCache *cache = win ? win->GetShaderCache() : nullptr;
  const vtkMTimeType stamp = this->ShaderBuildTimeStamp.GetMTime();
  // this->Shaders only changes in UpdateShaders, which RenderPieceStart follows
  // with ShaderBuildTimeStamp.Modified(); while the stamp and the cache are the
  // ones the last lookup saw, that lookup would find the same program again.
  // ReadyShaderProgram(program) still compiles it if its context was released
  // and binds it exactly as the lookup's tail would.
  if (allowSkip && cache && this->ShaderProgram && cache == m_cache.GetPointer() &&
      stamp == m_builtStamp && this->ElementType != AbstractPatches) {
    this->ShaderProgram = cache->ReadyShaderProgram(this->ShaderProgram.GetPointer());
    ++m_stats.programLookupsSkipped;
    return;
  }
  this->vtkDrawTexturedElements::ReadyShaderProgram(ren); // copy, substitute, MD5, find, bind
  ++m_stats.programLookups;
  m_cache = cache;
  m_builtStamp = stamp;
}

bool LowMemoryPolyDataMapper::renderable(int t) const {
  // vtkOpenGLLowMemoryCellTypeAgent::Draw draws cell group 0 iff it CanRender.
  const auto &groups = this->Primitives[t].CellGroups;
  return !groups.empty() && groups[0].CanRender;
}

bool LowMemoryPolyDataMapper::coincidentSkipSafe(vtkActor *act) const {
  // vtkGLSLModCoincidentTopology sets cOffset/cFactor in every PreDraw, per
  // primitive class, and only when the value is non-zero; otherwise the program
  // keeps whatever was set last. So a cell type whose own offset is zero sees
  // what an earlier cell type -- possibly one with nothing to draw -- left
  // behind, and the last one leaves a value for the next draw of that program.
  // Skipping is safe when every drawn type, and the program afterwards, ends up
  // with the same value either way.
  vtkProperty *prop = act->GetProperty();
  const int mode = vtkMapper::GetResolveCoincidentTopology();
  if (mode != VTK_RESOLVE_POLYGON_OFFSET && mode != VTK_RESOLVE_SHIFT_ZBUFFER &&
      !(prop->GetEdgeVisibility() && prop->GetRepresentation() == VTK_SURFACE))
    return true; // no offset uniform is ever set (VTK's default)
  auto *self = const_cast<LowMemoryPolyDataMapper *>(this);
  const bool points = prop->GetRepresentation() == VTK_POINTS;
  CoincidentValue stock, skipped; // unset: unchanged since before this draw
  for (int t = 0; t < 4; ++t) {
    const int slot = points ? 0 : (t == 0 ? 0 : (t == 1 ? 1 : 2));
    const CoincidentValue v = coincidentValue(self, prop, slot);
    if (v.set)
      stock = v;
    if (renderable(t)) {
      if (v.set)
        skipped = v;
      if (stock != skipped)
        return false;
    }
  }
  return stock == skipped;
}

void LowMemoryPolyDataMapper::agentPreDraw(int t, vtkRenderer *ren, vtkActor *act) {
  // vtkOpenGLLowMemory{Vertices,Lines,Polygons}Agent::PreDrawInternal (the
  // strips agent is the polygons agent) ...
  vtkProperty *prop = act->GetProperty();
  int pointsPerPrimitive = 3;
  if (t == 0) {
    pointsPerPrimitive = 1;
    this->ElementType = vtkDrawTexturedElements::ElementShape::Point;
    this->NumberOfInstances = 1;
    this->ShaderProgram->SetUniformi("cellType", VTK_VERTEX);
  } else if (t == 1) {
    pointsPerPrimitive = 2;
    this->ElementType = vtkDrawTexturedElements::ElementShape::Line;
    if (prop->GetLineWidth() > 1)
      this->NumberOfInstances = 2 * vtkMath::Ceil(prop->GetLineWidth());
    else
      this->NumberOfInstances = 1;
    this->ShaderProgram->SetUniformi("cellType", VTK_LINE);
  } else {
    this->ElementType = vtkDrawTexturedElements::ElementShape::Triangle;
    this->NumberOfInstances = 1;
    this->ShaderProgram->SetUniformi("cellType", VTK_TRIANGLE);
  }
  // ... then vtkOpenGLLowMemoryCellTypeAgent::PreDraw (never in the vertex
  // visibility pass, which takes the stock draw).
  if (prop->GetRepresentation() == VTK_POINTS)
    this->ElementType = vtkDrawTexturedElements::ElementShape::Point;
  bool needLighting = false;
  if (prop->GetRepresentation() == VTK_POINTS) {
    needLighting = prop->GetInterpolation() != VTK_FLAT && this->HasPointNormals;
  } else {
    const bool isTrisOrStrips = pointsPerPrimitive >= 3;
    needLighting = isTrisOrStrips || (!isTrisOrStrips && prop->GetInterpolation() != VTK_FLAT &&
                                      this->HasPointNormals);
  }
  this->ShaderProgram->SetUniformi("enable_lights", needLighting);
  this->ShaderProgram->SetUniformi("vertex_pass", false);
  switch (this->ElementType) {
  case vtkDrawTexturedElements::ElementShape::Point:
    this->ShaderProgram->SetUniformi("primitiveSize", 1);
    break;
  case vtkDrawTexturedElements::ElementShape::Line:
    this->ShaderProgram->SetUniformi("primitiveSize", 2);
    break;
  case vtkDrawTexturedElements::ElementShape::Triangle:
  default:
    this->ShaderProgram->SetUniformi("primitiveSize", 3);
    break;
  }
  // PointPicking is only ever set with a selector, which takes the stock draw.
  this->ShaderProgram->SetUniformf("pointSize", prop->GetPointSize());
  this->vtkDrawTexturedElements::PreDraw(ren, act, this);
}

void LowMemoryPolyDataMapper::agentDraw(int t, vtkRenderer *ren, vtkActor *act) {
  // vtkOpenGLLowMemoryCellTypeAgent::Draw, cell group 0.
  const auto &groups = this->Primitives[t].CellGroups;
  if (groups.empty() || !groups[0].CanRender)
    return;
  const auto &group = groups[0];
  const auto &offsets = group.Offsets;
  this->FirstVertexId = offsets.VertexIdOffset;
  this->NumberOfElements = group.NumberOfElements;
  // when rendering vertices, increase number of elements and draw 1 instance.
  if (act->GetProperty()->GetRepresentation() == VTK_POINTS) {
    this->NumberOfElements *= (t == 0 ? 1 : (t == 1 ? 2 : 3));
    this->NumberOfInstances = 1;
  }
  vtkShaderProgram *program = this->ShaderProgram;
  program->SetUniformi("cellIdOffset", offsets.CellIdOffset);
  program->SetUniformi("vertexIdOffset", offsets.VertexIdOffset);
  program->SetUniformi("edgeValueBufferOffset", offsets.EdgeValueBufferOffset);
  program->SetUniformi("pointIdOffset", offsets.PointIdOffset);
  program->SetUniformi("primitiveIdOffset", offsets.PrimitiveIdOffset);
  program->SetUniformi("usesCellMap", group.UsesCellMapBuffer);
  program->SetUniformi("usesEdgeValues", group.UsesEdgeValueBuffer);
  this->vtkDrawTexturedElements::DrawInstancedElementsImpl(ren, act, this);
}

#endif // CVC_GL_LOWMEM_REPLICA

void setLowMemoryMapperPolicy(LowMemoryMapperPolicy policy) {
  policyStore().store(static_cast<int>(policy), std::memory_order_relaxed);
}

LowMemoryMapperPolicy lowMemoryMapperPolicy() {
  return static_cast<LowMemoryMapperPolicy>(policyStore().load(std::memory_order_relaxed));
}

vtkSmartPointer<vtkPolyDataMapper> newPolyDataMapper() {
  const LowMemoryMapperPolicy policy = lowMemoryMapperPolicy();
  if (policy == LowMemoryMapperPolicy::Force ||
      (policy == LowMemoryMapperPolicy::Auto && factoryIsLowMemory()))
    return vtkSmartPointer<LowMemoryPolyDataMapper>::New();
  return vtkSmartPointer<vtkPolyDataMapper>::New();
}

} // namespace gl
} // namespace cvc
