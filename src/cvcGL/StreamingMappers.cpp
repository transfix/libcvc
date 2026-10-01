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
#include <cvc/gl/StreamingMappers.h>
#include <vector>
#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkDataArray.h>
#include <vtkObjectFactory.h>
#include <vtkOpenGLBufferObject.h>
#include <vtkOpenGLIndexBufferObject.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkOpenGLState.h>
#include <vtkOpenGLVertexBufferObject.h>
#include <vtkOpenGLVertexBufferObjectGroup.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkShaderProgram.h>
#include <vtkShaderProperty.h>
#include <vtkStringToken.h>
#include <vtkTextureObject.h>
#include <vtk_glad.h>

namespace cvc {
namespace gl {

// ─────────────────────────────── shared core ────────────────────────────────
void StreamingMapperCore::markPoints(vtkIdType first, vtkIdType count) {
  if (count <= 0)
    return;
  if (!dirty()) {
    dirtyLo = first;
    dirtyHi = first + count;
  } else {
    dirtyLo = std::min(dirtyLo, first);
    dirtyHi = std::max(dirtyHi, first + count);
  }
}

void StreamingMapperCore::setReservedBounds(const double b[6]) {
  // No Modified() on the mapper: a mapper-MTime bump makes the classic mapper
  // re-run BuildBufferObjects (its IBO state string includes the mapper MTime,
  // so the index buffer would be re-uploaded). Bounds are re-queried per frame.
  std::memcpy(bounds, b, sizeof(bounds));
  reservedBounds = true;
}

void StreamingMapperCore::clampedRange(vtkIdType nTris, vtkIdType &first, vtkIdType &count) const {
  first = std::min(nTris, std::max<vtkIdType>(0, triFirst));
  count = nTris - first;
  if (triLimit >= 0)
    count = std::min(count, triLimit);
}

// ─────────────────────────────── classic mapper ─────────────────────────────
vtkStandardNewMacro(StreamingPolyDataMapper);

void StreamingPolyDataMapper::ComputeBounds() {
  if (Core.beforeBounds)
    Core.beforeBounds();
  if (Core.reservedBounds) {
    std::memcpy(this->Bounds, Core.bounds, sizeof(Core.bounds));
    return;
  }
  Superclass::ComputeBounds();
}

void StreamingPolyDataMapper::BuildIBO(vtkRenderer *ren, vtkActor *act, vtkPolyData *poly) {
  Superclass::BuildIBO(ren, act, poly);
  vtkProperty *prop = act->GetProperty();
  vtkCellArray *polys = poly->GetPolys();
  if (prop->GetRepresentation() != VTK_SURFACE || prop->GetEdgeVisibility() ||
      polys->GetNumberOfCells() == 0 ||
      polys->GetNumberOfConnectivityIds() != 3 * polys->GetNumberOfCells())
    return;
  vtkOpenGLIndexBufferObject *ibo = this->Primitives[PrimitiveTris].IBO;
  if (ibo->IndexCount == static_cast<size_t>(polys->GetNumberOfConnectivityIds()))
    return; // nothing dropped: the stock IBO already is the connectivity, in order
  std::vector<unsigned int> idx;
  idx.reserve(static_cast<size_t>(polys->GetNumberOfConnectivityIds()));
  vtkIdType npts = 0;
  const vtkIdType *ids = nullptr;
  for (polys->InitTraversal(); polys->GetNextCell(npts, ids);)
    for (vtkIdType i = 0; i < npts; ++i)
      idx.push_back(static_cast<unsigned int>(ids[i]));
  ibo->Upload(idx, vtkOpenGLIndexBufferObject::ElementArrayBuffer);
  ibo->IndexCount = idx.size();
}

void StreamingPolyDataMapper::RenderPieceStart(vtkRenderer *ren, vtkActor *act) {
  if (Core.beforeDraw)
    Core.beforeDraw(ren);
  vtkOpenGLVertexBufferObject *vbo = this->VBOs->GetVBO("vertexMC");
  const vtkMTimeType before = vbo ? vbo->GetUploadTime().GetMTime() : 0;

  // The base rebuilds the VBOs/IBO only when the input, property or texture
  // state changed. Streaming writes never call Modified() on the points, so on
  // a steady frame this uploads nothing.
  Superclass::RenderPieceStart(ren, act);

  vbo = this->VBOs->GetVBO("vertexMC");
  if (!vbo)
    return;
  if (before == 0 || vbo->GetUploadTime().GetMTime() != before) {
    // A full upload just happened (first draw, topology change, context loss):
    // it already carried every pending point.
    ++Core.fullUploads;
    Core.clearDirty();
    return;
  }
  if (!Core.dirty())
    return;

  vtkDataArray *pts = this->CurrentInput->GetPoints()->GetData();
  const vtkIdType n = pts->GetNumberOfTuples();
  const vtkIdType lo = std::max<vtkIdType>(0, Core.dirtyLo);
  const vtkIdType hi = std::min<vtkIdType>(n, Core.dirtyHi);
  Core.clearDirty();
  if (hi <= lo)
    return;
  if (vbo->GetCoordShiftAndScaleEnabled() || pts->GetDataType() != VTK_FLOAT ||
      pts->GetNumberOfComponents() != 3 || vbo->GetStride() != 12) {
    // Not the raw-float layout (shift/scale on, or doubles): re-upload the whole
    // array. Correct, just not streaming; streaming nodes never get here.
    vbo->UploadDataArray(pts);
    ++Core.fullUploads;
    return;
  }
  const float *p = static_cast<const float *>(pts->GetVoidPointer(0));
  vbo->UploadRange(p + 3 * lo, static_cast<ptrdiff_t>(lo * 12), static_cast<size_t>(3 * (hi - lo)),
                   vtkOpenGLBufferObject::ArrayBuffer);
  ++Core.partialUploads;
  ++Core.partialGLCalls;
  Core.partialBytes += static_cast<std::uint64_t>(hi - lo) * 12;
}

void StreamingPolyDataMapper::RenderPieceDraw(vtkRenderer *ren, vtkActor *act) {
  vtkOpenGLIndexBufferObject *ibo = this->Primitives[PrimitiveTris].IBO;
  const size_t saved = ibo->IndexCount;
  vtkIdType first = 0, count = 0;
  Core.clampedRange(static_cast<vtkIdType>(saved / 3), first, count);
  if (first == 0) {
    // A prefix: the base draw takes IndexCount as its element count.
    ibo->IndexCount = static_cast<size_t>(3 * count);
    Superclass::RenderPieceDraw(ren, act); // glDrawRangeElements(..., IndexCount, ..., 0)
    ibo->IndexCount = saved;
    return;
  }
  // A non-zero first triangle needs an index-buffer byte offset, which the base
  // hard-codes to 0. Let the base draw every other primitive type (IndexCount 0
  // makes it skip triangles), then draw the triangle sub-range here with the
  // same shader setup. Cell-data colouring through PrimitiveIDOffset and the
  // selection pass are not offset; streaming overlays use neither.
  ibo->IndexCount = 0;
  Superclass::RenderPieceDraw(ren, act);
  ibo->IndexCount = saved;
  if (count == 0 || this->PointPicking)
    return;
  vtkOpenGLHelper &prim = this->Primitives[PrimitiveTris];
  this->DrawingVertices = false;
  this->DrawingSelection = false;
  this->UpdateShaders(prim, ren, act);
  prim.IBO->Bind();
  const int numVerts = this->VBOs->GetNumberOfTuples("vertexMC");
  glDrawRangeElements(
      this->GetOpenGLMode(act->GetProperty()->GetRepresentation(), PrimitiveTris), 0,
      static_cast<GLuint>(numVerts - 1), static_cast<GLsizei>(3 * count), GL_UNSIGNED_INT,
      reinterpret_cast<const void *>(static_cast<size_t>(3 * first) * sizeof(GLuint)));
  prim.IBO->Release();
}

// ─────────────────────────────── low-memory mapper ──────────────────────────
// The base, LowMemoryPolyDataMapper, initialises the shift/scale members VTK
// 9.5.0 leaves uninitialised: with garbage there the mapper uploaded a
// shift/scale COPY of the positions, which also defeats streaming (the uploaded
// array must be the input's own).
vtkStandardNewMacro(StreamingLowMemoryPolyDataMapper);

void StreamingLowMemoryPolyDataMapper::ComputeBounds() {
  if (Core.beforeBounds)
    Core.beforeBounds();
  if (Core.reservedBounds) {
    std::memcpy(this->Bounds, Core.bounds, sizeof(Core.bounds));
    return;
  }
  Superclass::ComputeBounds();
}

void StreamingLowMemoryPolyDataMapper::RenderPieceStart(vtkRenderer *ren, vtkActor *act) {
  if (Core.beforeDraw)
    Core.beforeDraw(ren);
  // VTK 9.5's low-memory mapper never looks at the actor's shader property once
  // its program is built: IsShaderUpToDate ignores it, where the classic mapper
  // compares GetShaderMTime(). A replacement added or cleared after the first
  // draw (GeometryNode::add*ShaderReplacement / clearShaderReplacements) would
  // never reach the GPU. Dropping the program makes the base rebuild it.
  if (vtkShaderProperty *sp = act->GetShaderProperty())
    if (sp->GetShaderMTime() > this->ShaderBuildTimeStamp.GetMTime())
      this->ShaderProgram = nullptr;
  // IsUpToDate() false => the base deletes EVERY array texture and re-binds them
  // all (the per-frame churn this mapper exists to avoid). That only happens on
  // a real input/topology/shader change; streaming writes never touch the
  // input's MTime.
  const bool rebuild = !this->IsUpToDate(ren, act);
  Superclass::RenderPieceStart(ren, act);
  if (rebuild) {
    ++Core.fullUploads;
    Core.clearDirty();
    return;
  }
  if (!Core.dirty())
    return;

  auto it = this->Arrays.find(vtkStringToken("positions"));
  vtkDataArray *pts = this->CurrentInput->GetPoints()->GetData();
  const vtkIdType n = pts->GetNumberOfTuples();
  const vtkIdType lo = std::max<vtkIdType>(0, Core.dirtyLo);
  const vtkIdType hi = std::min<vtkIdType>(n, Core.dirtyHi);
  Core.clearDirty();
  if (hi <= lo)
    return;
  if (it == this->Arrays.end() || !it->second.Texture || it->second.Arrays.size() != 1 ||
      it->second.Arrays[0] != pts || pts->GetDataType() != VTK_FLOAT ||
      pts->GetNumberOfComponents() != 3) {
    // A shift/scale copy, doubles, or not uploaded yet: let the base rebuild
    // everything on the next draw. Correct, just not streaming.
    vtkWarningMacro(<< "positions are not streamable here; falling back to a full upload");
    this->CurrentInput->GetPoints()->Modified();
    return;
  }
  const float *p = static_cast<const float *>(pts->GetVoidPointer(0));
  auto &adapter = it->second;
  int calls = 0;
  if (adapter.Texture->GetTarget() == GL_TEXTURE_2D) {
    // GLES3/WebGL2: VTK emulated the texture buffer with a 2-D texture and
    // deleted the source buffer; update the texel rows in place.
    calls = uploadTexelRange(
        adapter.Texture, vtkOpenGLRenderWindow::SafeDownCast(ren->GetRenderWindow()), p, lo, hi);
  } else if (adapter.Buffer) {
    // Desktop GL: a real texture buffer sourced from adapter.Buffer, which VTK
    // keeps alive -- update it in place.
    adapter.Buffer->UploadRange(p + 3 * lo, static_cast<ptrdiff_t>(lo * 12),
                                static_cast<size_t>(3 * (hi - lo)),
                                vtkOpenGLBufferObject::TextureBuffer);
    calls = 1;
  } else {
    this->CurrentInput->GetPoints()->Modified();
    return;
  }
  ++Core.partialUploads;
  Core.partialGLCalls += static_cast<std::uint64_t>(calls);
  Core.partialBytes += static_cast<std::uint64_t>(hi - lo) * 12;
}

void StreamingLowMemoryPolyDataMapper::RenderPieceDraw(vtkRenderer *ren, vtkActor *act) {
  // Primitives[2] is the polygon agent, one cell group per input mesh. It draws
  // glDrawArraysInstanced(Offsets.VertexIdOffset, 3 * NumberOfElements) and the
  // vertex shader pulls pointId = vertexIdBuffer[gl_VertexID] (absolute),
  // primitive = (gl_VertexID - vertexIdOffset) / 3, cellId = primitive +
  // cellIdOffset. Shifting the vertex AND cell offsets by the first triangle
  // draws exactly [first, first + count) with every id still correct.
  auto &groups = this->Primitives[2].CellGroups;
  if (groups.empty()) {
    Superclass::RenderPieceDraw(ren, act);
    return;
  }
  const CellGroupInformation saved = groups[0];
  vtkIdType first = 0, count = 0;
  Core.clampedRange(saved.NumberOfElements, first, count);
  CellGroupInformation &g = groups[0];
  g.NumberOfElements = count;
  g.Offsets.VertexIdOffset += 3 * first;
  g.Offsets.CellIdOffset += first;
  if (g.UsesCellMapBuffer)
    g.Offsets.PrimitiveIdOffset += first;
  if (g.UsesEdgeValueBuffer)
    g.Offsets.EdgeValueBufferOffset += first;
  Superclass::RenderPieceDraw(ren, act);
  groups[0] = saved;
}

// ─────────────────────────────── factory ────────────────────────────────────
StreamingMapperKind resolveStreamingMapperKind(StreamingMapperKind requested) {
  if (requested != StreamingMapperKind::Auto)
    return requested;
  // Exactly the question GeometryNode's vtkPolyDataMapper::New() answers: VTK's
  // object factory overrides it with the low-memory mapper on GLES builds
  // (VTK_OPENGL_USE_GLES, i.e. WebGL2) and the classic mapper elsewhere.
  vtkSmartPointer<vtkPolyDataMapper> probe = vtkSmartPointer<vtkPolyDataMapper>::New();
  return probe->IsA("vtkOpenGLLowMemoryPolyDataMapper") ? StreamingMapperKind::LowMemory
                                                        : StreamingMapperKind::Classic;
}

vtkSmartPointer<vtkPolyDataMapper> newStreamingMapper(StreamingMapperKind kind,
                                                      StreamingMapperCore **core) {
  vtkSmartPointer<vtkPolyDataMapper> m;
  StreamingMapperCore *c = nullptr;
  if (resolveStreamingMapperKind(kind) == StreamingMapperKind::LowMemory) {
    auto lm = vtkSmartPointer<StreamingLowMemoryPolyDataMapper>::New();
    c = &lm->core();
    m = lm;
  } else {
    auto cm = vtkSmartPointer<StreamingPolyDataMapper>::New();
    c = &cm->core();
    m = cm;
  }
  // Raw float positions with stride 12 are what makes a sub-range upload a
  // plain byte-offset copy (and the shader's vertexMC the true coordinate).
  m->SetVBOShiftScaleMethod(vtkOpenGLVertexBufferObject::DISABLE_SHIFT_SCALE);
  m->ScalarVisibilityOff();
  if (core)
    *core = c;
  return m;
}

StreamingMapperCore *streamingCore(vtkPolyDataMapper *mapper) {
  if (auto *cm = StreamingPolyDataMapper::SafeDownCast(mapper))
    return &cm->core();
  if (auto *lm = StreamingLowMemoryPolyDataMapper::SafeDownCast(mapper))
    return &lm->core();
  return nullptr;
}

// ─────────────────────────────── GLES emulation ─────────────────────────────
int computeTexelRowSpans(vtkIdType texWidth, vtkIdType lo, vtkIdType hi, TexelRowSpan spans[3]) {
  if (texWidth <= 0 || hi <= lo || lo < 0)
    return 0;
  const vtkIdType w = texWidth;
  int n = 0;
  vtkIdType i = lo;
  if (i % w) { // head: the rest of a partial first row
    const vtkIdType cnt = std::min(hi - i, w - i % w);
    spans[n++] = TexelRowSpan{i % w, i / w, cnt, 1, i};
    i += cnt;
  }
  if (hi - i >= w) { // body: whole rows in one call
    const vtkIdType rows = (hi - i) / w;
    spans[n++] = TexelRowSpan{0, i / w, w, rows, i};
    i += rows * w;
  }
  if (i < hi) // tail: the start of a partial last row
    spans[n++] = TexelRowSpan{0, i / w, hi - i, 1, i};
  return n;
}

int uploadTexelRange(vtkTextureObject *tex, vtkOpenGLRenderWindow *window, const float *xyz,
                     vtkIdType lo, vtkIdType hi) {
  if (!tex || !xyz || hi <= lo)
    return 0;
  TexelRowSpan spans[3];
  const int n = computeTexelRowSpans(static_cast<vtkIdType>(tex->GetWidth()), lo, hi, spans);
  if (n == 0)
    return 0;
  tex->Activate();
  // A bound pixel-unpack buffer would turn the pointer below into an offset.
  glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
  if (window)
    window->GetState()->vtkglPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  for (int s = 0; s < n; ++s)
    glTexSubImage2D(GL_TEXTURE_2D, 0, static_cast<GLint>(spans[s].x),
                    static_cast<GLint>(spans[s].y), static_cast<GLsizei>(spans[s].width),
                    static_cast<GLsizei>(spans[s].rows), GL_RGB, GL_FLOAT,
                    xyz + 3 * spans[s].first);
  tex->Deactivate();
  return n;
}

} // namespace gl
} // namespace cvc
