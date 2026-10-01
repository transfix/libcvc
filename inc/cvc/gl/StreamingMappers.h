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

// Streaming poly-data mappers: the VTK half of cvc::gl::StreamingGeometryNode.
//
// VTK 9.5 offers no way to change part of a mesh without re-uploading all of
// it. Modified() on the points re-sends the whole array (and on GLES/WebGL2 the
// low-memory mapper deletes and re-creates every array texture), and the index
// buffer builder drops triangles whose vertices coincide, which is exactly what
// reserved, not-yet-written capacity looks like. These two thin subclasses, one
// per VTK mapper family, add the three things a streaming overlay needs:
//
//   1. RESERVED BOUNDS -- the mapper reports a caller-fixed box instead of
//      walking a capacity-sized, partly unused point array. Streaming writes
//      never call Modified(), so without it the frustum culler and the
//      clipping-range reset would use stale bounds (a template-space link would
//      be culled outright).
//   2. PARTIAL UPLOAD -- markPoints(first, count) uploads ONLY that point range
//      of the position array at the next draw, into the GPU storage VTK already
//      allocated for the full capacity:
//        classic  (vtkOpenGLPolyDataMapper):           glBufferSubData on vertexMC
//        low-mem  (vtkOpenGLLowMemoryPolyDataMapper), texture buffer (desktop GL):
//                                                       glBufferSubData on the TBO
//        low-mem, 2-D texture (GLES3/WebGL2 emulation): at most 3 glTexSubImage2D
//      Under a multi-pass pipeline (vtkShadowMapPass) the first pass uploads and
//      the rest see nothing pending: one upload per frame, not per pass.
//   3. DRAW RANGE -- only triangles [first, first + count) of the polys are
//      drawn, with no upload: an append-only track draws what has been
//      appended, a route hides its consumed front and a stale tail.
//
// The CPU copy of the positions is the input's own vtkPoints float array. The
// owner thread writes it in place and calls markPoints instead of Modified(). A
// full rebuild (input Modified(), topology change, context loss) still works
// and simply absorbs the pending range. Positions must be float32 xyz with
// shift/scale DISABLED (DISABLE_SHIFT_SCALE); anything else falls back to a
// full re-upload, correct but not streaming.
//
// These rely on protected VTK 9.5 internals (the low-memory "positions" array
// entry and CellGroups, the classic triangle IBO IndexCount, GetVBO("vertexMC"),
// UpdateShaders, GetOpenGLMode). The cvcgl_streaming_* tests pin them, so a VTK
// bump that moves them fails there first.
#ifndef CVC_GL_STREAMING_MAPPERS_H
#define CVC_GL_STREAMING_MAPPERS_H

#include <atomic>
#include <cstdint>
#include <cvc/gl/StreamingGeometryNode.h> // StreamingMapperKind
#include <functional>
#include <vtkOpenGLLowMemoryPolyDataMapper.h>
#include <vtkOpenGLPolyDataMapper.h>
#include <vtkSmartPointer.h>

class vtkOpenGLRenderWindow;
class vtkRenderer;
class vtkTextureObject;

namespace cvc {
namespace gl {

// Book-keeping shared by both streaming mapper families. The dirty range, the
// draw range and the bounds are owner-thread state (the thread that renders);
// the counters are atomics so anything may read them.
struct StreamingMapperCore {
  bool reservedBounds = false;
  double bounds[6] = {0, 0, 0, 0, 0, 0};
  vtkIdType dirtyLo = 0, dirtyHi = 0; // pending point range [lo, hi)
  vtkIdType triFirst = 0;             // first triangle drawn
  vtkIdType triLimit = -1;            // max triangles drawn from triFirst; -1 = all

  // Sub-range uploads issued, their bytes and the GL calls they took (1 buffer
  // sub-data, or up to 3 texture sub-images on the GLES emulation).
  std::atomic<std::uint64_t> partialUploads{0}, partialBytes{0}, partialGLCalls{0};
  // Full uploads VTK did itself (first draw, topology change, context loss).
  std::atomic<std::uint64_t> fullUploads{0};

  // Called at the start of every draw of this mapper, on the render thread with
  // the GL context current -- the place to create or refresh GL resources the
  // shader needs (a height-field texture). Cleared by the owning node's dtor.
  std::function<void(vtkRenderer *)> beforeDraw;

  void markPoints(vtkIdType first, vtkIdType count);
  bool dirty() const { return dirtyHi > dirtyLo; }
  void clearDirty() { dirtyLo = dirtyHi = 0; }
  void setDrawRange(vtkIdType first, vtkIdType count) {
    triFirst = first < 0 ? 0 : first;
    triLimit = count;
  }
  void setReservedBounds(const double b[6]);
  // [first, first + count) clamped into a mesh of nTris triangles.
  void clampedRange(vtkIdType nTris, vtkIdType &first, vtkIdType &count) const;
};

class StreamingPolyDataMapper : public vtkOpenGLPolyDataMapper {
public:
  static StreamingPolyDataMapper *New();
  vtkTypeMacro(StreamingPolyDataMapper, vtkOpenGLPolyDataMapper);

  StreamingMapperCore &core() { return Core; }
  const StreamingMapperCore &core() const { return Core; }

  void RenderPieceStart(vtkRenderer *ren, vtkActor *act) override;
  void RenderPieceDraw(vtkRenderer *ren, vtkActor *act) override;

protected:
  StreamingPolyDataMapper() = default;
  ~StreamingPolyDataMapper() override = default;
  void ComputeBounds() override;
  // vtkOpenGLIndexBufferObject DROPS triangles whose vertices coincide when it
  // builds the IBO. Reserved capacity is all-coincident placeholders at build
  // time, so the stock IBO would be empty, and a partially collapsed build would
  // shift the triangle order the draw range relies on. A triangle-only surface
  // gets the raw connectivity instead, so triangle i of the input is triangle i
  // of the draw.
  void BuildIBO(vtkRenderer *ren, vtkActor *act, vtkPolyData *poly) override;

private:
  StreamingMapperCore Core;
  StreamingPolyDataMapper(const StreamingPolyDataMapper &) = delete;
  void operator=(const StreamingPolyDataMapper &) = delete;
};

class StreamingLowMemoryPolyDataMapper : public vtkOpenGLLowMemoryPolyDataMapper {
public:
  static StreamingLowMemoryPolyDataMapper *New();
  vtkTypeMacro(StreamingLowMemoryPolyDataMapper, vtkOpenGLLowMemoryPolyDataMapper);

  StreamingMapperCore &core() { return Core; }
  const StreamingMapperCore &core() const { return Core; }

  void RenderPieceStart(vtkRenderer *ren, vtkActor *act) override;
  void RenderPieceDraw(vtkRenderer *ren, vtkActor *act) override;

protected:
  StreamingLowMemoryPolyDataMapper();
  ~StreamingLowMemoryPolyDataMapper() override = default;
  void ComputeBounds() override;

private:
  StreamingMapperCore Core;
  StreamingLowMemoryPolyDataMapper(const StreamingLowMemoryPolyDataMapper &) = delete;
  void operator=(const StreamingLowMemoryPolyDataMapper &) = delete;
};

// Auto -> the family vtkPolyDataMapper::New() yields on this build; Classic and
// LowMemory pass through.
StreamingMapperKind resolveStreamingMapperKind(StreamingMapperKind requested);

// A new streaming mapper of the (resolved) kind, shift/scale disabled and scalar
// colouring off. `core` receives its book-keeping.
vtkSmartPointer<vtkPolyDataMapper> newStreamingMapper(StreamingMapperKind kind,
                                                      StreamingMapperCore **core);

// The book-keeping of a streaming mapper, or nullptr for any other mapper.
StreamingMapperCore *streamingCore(vtkPolyDataMapper *mapper);

// ── GLES3/WebGL2 texture-buffer emulation ──────────────────────────────────
// GLES has no texture buffers, so VTK 9.5 copies each array into a 2-D RGB32F
// texture of width w = min(N, GL_MAX_TEXTURE_SIZE) and reads element i at texel
// (i % w, i / w) (vtkTextureObject::EmulateTextureBufferWith2DTextures). A point
// range [lo, hi) is then at most three row spans: a partial head row, a block
// of whole rows, a partial tail row.
struct TexelRowSpan {
  vtkIdType x = 0, y = 0;        // first texel
  vtkIdType width = 0, rows = 0; // extent in texels
  vtkIdType first = 0;           // element index of texel (x, y)
};
// Fills up to 3 spans covering [lo, hi) in a texture `texWidth` texels wide;
// returns how many.
int computeTexelRowSpans(vtkIdType texWidth, vtkIdType lo, vtkIdType hi, TexelRowSpan spans[3]);
// Upload elements [lo, hi) of a tightly packed float xyz array into such a 2-D
// RGB32F texture with glTexSubImage2D (render thread, context current). Returns
// the number of GL upload calls (0..3). Compiled on every platform -- the
// low-memory mapper takes it whenever VTK handed it a 2-D texture -- so the
// native tests cover it against a real texture.
int uploadTexelRange(vtkTextureObject *tex, vtkOpenGLRenderWindow *window, const float *xyz,
                     vtkIdType lo, vtkIdType hi);

} // namespace gl
} // namespace cvc

#endif // CVC_GL_STREAMING_MAPPERS_H
