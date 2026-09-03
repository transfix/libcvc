// Light-view camera fitting and the shadow lookup, shared by raycaster.cpp,
// the CUDA host side and volren_kernels_test.  Header-only so the three have
// exactly one source of truth for the geometry; the device transcription in
// raycast.cu mirrors shadow_bias/shadow_visibility line for line.
#ifndef CVC_VOLREN_DETAIL_SHADOW_MAP_H
#define CVC_VOLREN_DETAIL_SHADOW_MAP_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cvc/volren/camera.h>
#include <cvc/volren/settings.h>
#include <cvc/volren/shadow.h>
#include <cvc/volren/types.h>
#include <cvc/volume/bounding_box.h>

namespace cvc {
namespace volren {
namespace detail {

// Fit an orthographic light-view camera covering `bounds` from a directional
// light.  Returns false (leaving `out_cam`/`out_view` untouched) for a
// degenerate light direction or degenerate bounds -- such a light simply casts
// no shadow, which is consistent rather than a special case: a zero direction
// already contributes nothing in blinn_phong (ndotl == 0).
inline bool fit_light_camera(const light &l, const cvc::bounding_box &bounds, int resolution,
                             camera &out_cam, shadow_view &out_view) {
  const vec3d d = normalized(vec3d(l.direction)); // TOWARD the light
  if (dot(d, d) == 0.0)
    return false;
  if (!std::isfinite(bounds.minx) || !std::isfinite(bounds.miny) || !std::isfinite(bounds.minz) ||
      !std::isfinite(bounds.maxx) || !std::isfinite(bounds.maxy) || !std::isfinite(bounds.maxz))
    return false;

  const int res =
      std::max(limits::min_shadow_resolution, std::min(resolution, limits::max_raster_dim));
  const vec3d c(0.5 * (bounds.minx + bounds.maxx), 0.5 * (bounds.miny + bounds.maxy),
                0.5 * (bounds.minz + bounds.maxz));

  // Up-vector choice.  camera::basis() throws when `up` is parallel to
  // back == d, and cvc scenes are Z-up, so prefer world +Z and fall back to +Y
  // only when d is within ~26 degrees of vertical.  The threshold is 0.9, NOT
  // 1 - eps: a near-parallel up produces a numerically garbage `right` long
  // before cross() actually underflows.
  const vec3d world_up = std::fabs(d.z) < 0.9 ? vec3d(0, 0, 1) : vec3d(0, 1, 0);

  const vec3d back = d; // camera::basis(): back = normalize(eye - focal)
  const vec3d right = normalized(cross(world_up, back));
  if (dot(right, right) == 0.0)
    return false;
  const vec3d true_up = cross(back, right);
  const vec3d forward = -back;

  // scene_bounds is an AXIS-ALIGNED box centered on `c`, so its 8 corners come
  // in +/- pairs about the center and max|.| IS the exact half-extent -- no
  // separate min/max pass and no re-centering needed.
  double half_s = 0.0, half_t = 0.0, half_w = 0.0;
  for (int corner = 0; corner < 8; ++corner) {
    const vec3d p(corner & 1 ? bounds.maxx : bounds.minx, corner & 2 ? bounds.maxy : bounds.miny,
                  corner & 4 ? bounds.maxz : bounds.minz);
    const vec3d rel = p - c;
    half_s = std::max(half_s, std::fabs(dot(rel, right)));
    half_t = std::max(half_t, std::fabs(dot(rel, true_up)));
    half_w = std::max(half_w, std::fabs(dot(rel, forward)));
  }

  // 2% pad so a silhouette ray never lands in the outermost half-texel and
  // reads the +inf of a texel whose center missed the scene.
  const double parallel_scale = std::max(half_s, half_t) * 1.02;
  if (!(parallel_scale > 0.0) || !std::isfinite(parallel_scale))
    return false;

  // Any clearance beyond half_w works; this one is scale-free and always
  // positive, and it puts every scene point at light depth in
  // [parallel_scale, parallel_scale + 2*half_w] > 0.
  const double distance = half_w + parallel_scale;

  camera lc;
  lc.eye = (c + d * distance).to_array();
  lc.focal = c.to_array();
  lc.up = world_up.to_array();
  lc.projection = camera::projection_type::orthographic;
  lc.parallel_scale = parallel_scale;
  lc.width = res;
  lc.height = res; // SQUARE: aspect == 1, so one parallel_scale drives both axes

  // Fill the view from lc.basis(), never from the locals above: basis()
  // recomputes right/true_up from eye/focal/up and IS what the light pass will
  // march with, so there must be exactly one source of truth.
  view_basis b;
  try {
    b = lc.basis();
  } catch (const cvc::volren_error &) {
    return false;
  }

  out_cam = lc;
  out_view.eye = lc.eye;
  out_view.right = b.right.to_array();
  out_view.up = b.true_up.to_array();
  out_view.forward = (-b.back).to_array();
  out_view.parallel_scale = parallel_scale;
  out_view.width = res;
  out_view.height = res;
  out_view.texel_world = 2.0 * parallel_scale / double(res);
  return true;
}

// bias = bias_scale * latch_quantum + slope_scale * texel_world * tan(theta),
// where cos(theta) = |n_dot_l| floored at 0.1 (so tan caps at ~9.95).
//
// Two independent error sources, one term each:
//
//  - the ALONG-RAY quantization of the map, `latch_quantum`.  This is NOT
//    simply the march step, which is what it looks like it should be: it
//    depends on how the light pass latched.  An exact ray/MC isosurface
//    intersection is not quantized at all, so one march step already covers
//    it; a transfer-function latch fires on the first march SAMPLE that
//    crossed the alpha threshold and contributions are one per CELL, so it
//    walks in cells -- and so does the receiver sample, along its own ray.
//    raycaster::render() sizes it accordingly and passes the result here; its
//    comment carries the measurements.
//
//  - the LATERAL footprint.  The map stores the depth along the texel-center
//    ray; a receiver offset by up to half a texel across a surface tilted
//    theta from the light sits texel_world * tan(theta) deeper.
//
// The cos floor rather than an epsilon: a flat-gradient sample normalizes to
// {0,0,0} (normalized()'s <= 1e-12 contract), which would divide by zero.
// Flooring makes such a sample maximally biased, i.e. LIT -- and it gets
// neither diffuse nor specular anyway, so the value is inert.  Failing lit is
// the right direction: an erroneous bright pixel reads as unshadowed, an
// erroneous dark one reads as acne.
inline double shadow_bias(const shadow_view &v, double latch_quantum, double bias_scale,
                          double slope_scale, double n_dot_l) {
  double cos_t = std::fabs(n_dot_l);
  if (!(cos_t > 0.1)) // inverted: NaN floors too
    cos_t = 0.1;
  else if (cos_t > 1.0)
    cos_t = 1.0;
  const double tan_t = std::sqrt(1.0 - cos_t * cos_t) / cos_t;
  return bias_scale * latch_quantum + slope_scale * v.texel_world * tan_t;
}

// 1.0 when lit, (1 - strength) when the map says the light is blocked.
// `depth_px` is the map's raw f32 buffer, row-major, width*height.
inline float shadow_visibility(const shadow_view &v, const float *depth_px, const vec3d &p,
                               double bias, float strength) {
  int ix = 0, iy = 0;
  double depth = 0.0;
  if (depth_px == nullptr || !v.project(p.to_array(), ix, iy, depth))
    return 1.f; // outside the map: fail LIT, the non-destructive direction
  const double map_depth = double(depth_px[std::size_t(iy) * std::size_t(v.width) + ix]);
  // Inverted-NaN safe: +inf (the light ray hit nothing) never shadows, and a
  // NaN receiver depth fails the test rather than shadowing at random.
  const bool shadowed = depth > map_depth + bias;
  return shadowed ? (1.f - strength) : 1.f;
}

} // namespace detail
} // namespace volren
} // namespace cvc

#endif // CVC_VOLREN_DETAIL_SHADOW_MAP_H
