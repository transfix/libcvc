// The slicing engine: view-aligned proxy polygons for one volume box.
//
// This is the pure-geometry half of the legacy renderer (RendererBase's
// plane sweep + ClipCube's 256-case table), lifted out of GL entirely so it
// is unit-testable and backend-neutral: the caller hands in the local->clip
// matrix and gets back triangle-fan polygons in the volume's LOCAL space with
// 3D texture coordinates, ordered back to front.  What the legacy code read
// from GL state (glGetFloatv of the matrix stack) is here an explicit input.
//
// Space conventions
// -----------------
// The legacy renderer sliced an origin-centered cube of extent +-0.5*ratio
// (aspect ratios normalized so max(ratio)==1) and left placement to the
// caller's translate/scale.  A real volume's local bounding box is exactly
// that cube under a uniform scale by its longest side plus a translation to
// its center, and uniform scale + translation preserve plane parallelism and
// relative spacing -- so compute_slices() runs the legacy algorithm verbatim
// in ratio space and maps the result into the caller's box on output.  The
// slice COUNT for a given (quality, max_planes) is identical to legacy for
// any box shape.
#ifndef CVC_VOLSLICE_SLICER_H
#define CVC_VOLSLICE_SLICER_H

#include <cstdint>
#include <cvc/volslice/types.h>
#include <vector>

namespace cvc {
namespace volslice {

// An axis-aligned box in the volume's local space.
struct box3d {
  vec3d min{-0.5, -0.5, -0.5};
  vec3d max{0.5, 0.5, 0.5};
};

// The geometry-affecting settings subset (the renderer-side settings --
// blend mode, filter, TF -- do not change the polygons).
struct slice_params {
  double quality = defaults::quality;       // clamped to [0,1]
  int max_planes = defaults::max_planes;    // clamped to limits::
  double near_plane = defaults::near_plane; // clamped to [0,1]
  // Texture sub-cube: renders only this texcoord range of the 3D texture
  // (legacy setTextureSubCube; the pow2-padding crop).  Full texture default.
  vec3d tex_min{0.0, 0.0, 0.0};
  vec3d tex_max{1.0, 1.0, 1.0};
};

// Flattened triangle-fan slices, ordered back to front.
struct slice_geometry {
  std::vector<float> positions;          // 3 floats per vertex, local space
  std::vector<float> texcoords;          // 3 floats per vertex
  std::vector<std::uint32_t> fan_offset; // per polygon: first vertex index
  std::vector<std::uint32_t> fan_count;  // per polygon: vertex count (3..6)
  // Distance between adjacent slice planes, in LOCAL units -- what the
  // opacity-correction shader term needs (defaults::opacity_correction).
  double plane_spacing = 0.0;
  std::size_t planes() const { return fan_offset.size(); }
  std::size_t vertices() const { return positions.size() / 3; }
  bool empty() const { return fan_offset.empty(); }
};

// Extract the object-space view plane from a local->clip matrix (row-major,
// column-vector convention: clip = M * local).  Returns the UNIT normal of a
// plane parallel to the near/view plane, in local space -- the legacy
// RendererBase::getViewPlane() viewcull trick, minus the GL state reads.
// Throws cvc::volslice_error if the matrix yields a zero normal (degenerate
// projection).
vec3d view_plane_normal(const mat4 &local_to_clip);

// Compute the back-to-front view-aligned slice polygons for `box` as seen
// through `local_to_clip`.  Deterministic; no GL; safe on any thread.
slice_geometry compute_slices(const mat4 &local_to_clip, const box3d &box,
                              const slice_params &params);

} // namespace volslice
} // namespace cvc

#endif // CVC_VOLSLICE_SLICER_H
