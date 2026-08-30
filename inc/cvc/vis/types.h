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

// types.h -- the geometric primitives every cvc::vis culler operates on.
//
// Pure math: no VTK, no GL, no I/O. This is the foundation of the visibility
// subsystem (docs/roadmap/VISIBILITY-AND-LOD-ROADMAP.md section 4), and, like
// cvc::lod, it is deliberately dependency-free so it can be exercised headless
// under the coverage gate and reused by the renderer through a thin adapter.
//
// Bounds and planes are float on purpose: they are the SIMD-friendly, GPU-
// adjacent quantities a per-frame culler sweeps in bulk (the SoA/SIMD kernel is
// a later PR). World-space distances that need precision -- the eye position,
// the near-distance clamp -- stay double and are carried in cvc::lod::view_params
// (see view_params.h), which owns the screen-projection math this module reuses
// rather than re-deriving.

#ifndef __CVC_VIS_TYPES_H__
#define __CVC_VIS_TYPES_H__

#include <cstdint>

namespace cvc {
namespace vis {

// A dense index into a scene_view's parallel columns. Not a pointer: the culler
// never dereferences a proxy, it reads columns by id.
using proxy_id = std::uint32_t;
inline constexpr proxy_id invalid_proxy = ~0u;

// Axis-aligned bounding box, min/max corner.
struct aabb {
  float mn[3];
  float mx[3];
};

// Bounding sphere.
struct sphere {
  float c[3];
  float r;
};

// A plane, inside half-space at n.x*x + n.y*y + n.z*z + d >= 0. `n` is unit
// length after construction.
struct plane {
  float n[3];
  float d;
};

// The result of testing a box against the frustum. INSIDE is load-bearing: a
// fully-inside box lets a hierarchical culler emit its whole subtree with zero
// further plane tests (section 15.3), so it is reported distinctly rather than
// folded into "not outside".
enum class frustum_test : std::uint8_t { outside = 0, intersect = 1, inside = 2 };

// Why a proxy was culled -- the diagnostic column an overlay reads. `visible`
// is 0 so a zeroed scratch buffer reads as "not yet decided == visible", which
// is the conservative default. The later stages (cell/portal/terrain/volume)
// are enumerated now so the value is stable across the PRs that add them.
enum class cull_result : std::uint8_t {
  visible = 0,
  layer_masked,
  cell_masked,
  distance_culled,
  frustum_culled,
  small_feature,
  terrain_occluded,
  volume_occluded,
  portal_unreached
};

// Six frustum planes in OUR canonical order: L, R, B, T, NEAR, FAR.
//
// This order is not arbitrary and getting it wrong is silent: vtkCamera emits
// L, R, B, T, FAR, NEAR (its own header warns "NOT near,far"), and a raw copy
// swaps the near and far planes -- which inverts depth clipping and passes most
// tests, so it is not caught by a smoke test. from_vtk_planes() does the reorder
// in one named place; nothing else should touch the raw 24-double array.
struct frustum {
  plane p[6];

  // From vtkCamera::GetFrustumPlanes(aspect, planes24). Reorders vtk's
  // L,R,B,T,FAR,NEAR into L,R,B,T,NEAR,FAR and normalizes each plane.
  static frustum from_vtk_planes(const double planes24[24]) noexcept;

  // From a row-major world->clip matrix M = P*V (Gribb-Hartmann), for the
  // headless tests where no vtkCamera exists. Rows m0..m3:
  //   L = m3+m0  R = m3-m0  B = m3+m1  T = m3-m1  N = m3+m2  F = m3-m2
  // each normalized by |(x,y,z)|.
  static frustum from_view_proj(const double m[16]) noexcept;
};

// Signed distance from a plane to a point (positive inside).
inline float plane_distance(const plane &p, const float c[3]) noexcept {
  return p.n[0] * c[0] + p.n[1] * c[1] + p.n[2] * c[2] + p.d;
}

// Sphere vs frustum (section 15.2). OUTSIDE iff the centre is farther than r
// behind any plane; INSIDE iff it clears every plane by at least r. Conservative
// at the frustum corners (a corner false-positive is reported not-outside),
// which is the standard, harmless conservatism.
frustum_test sphere_vs_frustum(const sphere &s, const frustum &f) noexcept;

// AABB vs frustum, p/n-vertex with plane masking (section 15.3).
//
// `mask` names the planes still worth testing: a hierarchical caller passes the
// parent's residual mask and, on INSIDE against a plane's n-vertex, this clears
// that bit so the whole subtree skips it. A flat caller passes 0b111111 and
// ignores the write-back. One dot product per live plane, no per-corner loop.
frustum_test aabb_vs_frustum(const aabb &b, const frustum &f, std::uint8_t &mask) noexcept;

// Flat convenience: test all six planes, no mask threading.
inline frustum_test aabb_vs_frustum(const aabb &b, const frustum &f) noexcept {
  std::uint8_t mask = 0x3f;
  return aabb_vs_frustum(b, f, mask);
}

// The tightest sphere around a box (centre + half-diagonal). Used to seed the
// distance/small-feature test, which is a sphere test.
sphere bounding_sphere(const aabb &b) noexcept;

} // namespace vis
} // namespace cvc

#endif // __CVC_VIS_TYPES_H__
