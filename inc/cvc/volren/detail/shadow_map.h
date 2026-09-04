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
// `slices` > 0 additionally fits the DEEP profile's knot grid to the scene's
// own light-depth extent (out_view::depth_min / slice_dz); 0 leaves the view a
// hard map.
inline bool fit_light_camera(const light &l, const cvc::bounding_box &bounds, int resolution,
                             camera &out_cam, shadow_view &out_view, int slices = 0) {
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

  // Deep profile knot grid.  The light camera is orthographic and `distance`
  // was chosen so every scene point sits at light depth in
  // [distance - half_w, distance + half_w] -- an EXACT, tight bracket, so the
  // grid needs no slack and spends no knot outside the marched region.  A
  // non-degenerate bounding box has positive extent along every direction, so
  // half_w > 0; the guard is for a caller that hands in something else.
  out_view.slices = 0;
  out_view.depth_min = distance - half_w;
  out_view.slice_dz = 0.0;
  if (slices > 0 && half_w > 0.0) {
    out_view.slices = slices;
    out_view.slice_dz = 2.0 * half_w / double(slices);
  }
  return true;
}

// bias = bias_scale * latch_quantum
//        + slope_scale * texel_world * tan(theta) * (1 + 2 * pcf_radius),
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
//    theta from the light sits texel_world * tan(theta) deeper.  PERCENTAGE-
//    CLOSER FILTERING is exactly the operation that grows that footprint: with
//    a filter of half-width R texels the receiver is compared against texels up
//    to R + 0.5 away, so the term scales by (R + 0.5) / 0.5 == 1 + 2R.  At
//    R == 0 that factor is exactly 1.0 and the expression is unchanged bit for
//    bit, which is what keeps an unfiltered render byte-identical.
//
// The cos floor rather than an epsilon: a flat-gradient sample normalizes to
// {0,0,0} (normalized()'s <= 1e-12 contract), which would divide by zero.
// Flooring makes such a sample maximally biased, i.e. LIT -- and it gets
// neither diffuse nor specular anyway, so the value is inert.  Failing lit is
// the right direction: an erroneous bright pixel reads as unshadowed, an
// erroneous dark one reads as acne.
inline double shadow_bias(const shadow_view &v, double latch_quantum, double bias_scale,
                          double slope_scale, double n_dot_l, double pcf_radius = 0.0) {
  double cos_t = std::fabs(n_dot_l);
  if (!(cos_t > 0.1)) // inverted: NaN floors too
    cos_t = 0.1;
  else if (cos_t > 1.0)
    cos_t = 1.0;
  const double tan_t = std::sqrt(1.0 - cos_t * cos_t) / cos_t;
  return bias_scale * latch_quantum +
         slope_scale * v.texel_world * tan_t * (1.0 + 2.0 * pcf_radius);
}

// ---------------------------------------------------------------------------
// Percentage-closer filtering (shadow_settings::pcf_radius / pcf_taps)
// ---------------------------------------------------------------------------
// The filter is a plain box average of (2k+1)^2 comparisons on a regular grid
// spread over [-radius, +radius] texels.  Two structural notes:
//
//  * ONE projection, many texels.  The light camera is ORTHOGRAPHIC, so a
//    receiver's light-space depth does not depend on which texel it is compared
//    against -- the taps offset the MAP index, never the query point.  A
//    perspective light would have to re-project per tap; this one does 3 dot
//    products for the whole filter and then reads texels.
//
//  * The taps are FIXED, unjittered and unrotated.  A rotated Poisson disk
//    resolves a wide penumbra into more levels for the same tap count and is
//    the standard answer, but it needs a per-pixel random rotation, and
//    determinism (byte-identical across runs and thread counts) is a documented
//    contract here.  The regular grid is also what makes the CPU and CUDA paths
//    comparable tap for tap.

// Half-width k of the tap grid: (2k+1)^2 taps.  0 means the filter is inert,
// so the callers below branch to the historical single-tap expression -- which
// is what makes an unfiltered lookup byte-identical rather than merely equal.
// `taps` is clamped into range, and an EVEN value rounds DOWN to the odd grid
// below it: a grid with no center tap would displace the shadow by half a tap
// spacing, which reads as the shadow sliding when the quality knob is nudged.
// The radius the BIAS must use.  pcf_half_width() collapses taps == 1 (or a
// non-positive radius) to a single-tap lookup, but the slope bias is widened by
// (1 + 2*radius) to cover the filter footprint -- so feeding it the raw radius
// while the lookup is single-tap widens the bias for a filter that is not
// there.  That silently broke the documented contract that the filter "degrades
// to the historical single-tap behavior from either end (taps 1 or radius 0)":
// the radius-0 end was byte-identical, the taps-1 end was not.  Both ends are
// now, because the footprint and the bias are derived from the same predicate.
inline float pcf_effective_radius(float radius, int taps);

inline int pcf_half_width(float radius, int taps) {
  if (!(radius > 0.f)) // inverted: NaN is inert too
    return 0;
  const int t = taps < limits::min_pcf_taps
                    ? limits::min_pcf_taps
                    : (taps > limits::max_pcf_taps ? limits::max_pcf_taps : taps);
  return (t - 1) / 2;
}

inline float pcf_effective_radius(float radius, int taps) {
  return pcf_half_width(radius, taps) > 0 ? radius : 0.f;
}

// Tap i of a k-half-width grid over `radius` texels, as an integer texel offset.
// Rounded to nearest, so a radius under k texels collapses taps onto the same
// texel (a filter narrower than a texel cannot blur, and pretending otherwise
// would need a bilinear read per tap for no visible gain).
//
// Rounded AWAY FROM ZERO at a half, not up.  The distinction is invisible
// almost everywhere and load-bearing where it is not: the filter is a CENTERED
// box average, so `pcf_offset(-i) == -pcf_offset(i)` has to hold exactly, and
// floor(x + 0.5) breaks it at every tap whose exact position is a half texel --
// radius 5 with 5 taps lands on {-5, -2, 0, 3, 5} rather than {-5, -3, 0, 3, 5},
// a footprint whose centroid sits a fifth of a texel off the receiver.  A filter
// that is supposed to be centered on the sample must not slide when `radius`
// crosses a half-integer, so the sign is taken out and put back.
inline int pcf_offset(int i, int k, double radius) {
  const double x = double(i) * radius / double(k);
  const double m = std::floor(std::fabs(x) + 0.5);
  return x < 0.0 ? -int(m) : int(m);
}

// One texel's HARD comparison.  Inverted-NaN safe: +inf (the light ray hit
// nothing) never shadows, and a NaN receiver depth fails the test rather than
// shadowing at random.
inline float shadow_tap(const shadow_view &v, const float *depth_px, int tx, int ty, double depth,
                        double bias, float strength) {
  if (tx < 0 || tx >= v.width || ty < 0 || ty >= v.height)
    return 1.f; // the filter reached off the map: that tap is LIT
  const double map_depth = double(depth_px[std::size_t(ty) * std::size_t(v.width) + tx]);
  return depth > map_depth + bias ? (1.f - strength) : 1.f;
}

// 1.0 when lit, (1 - strength) when the map says the light is blocked -- or the
// box average of pcf_taps^2 such comparisons when the filter is on.
// `depth_px` is the map's raw f32 buffer, row-major, width*height.
inline float shadow_visibility(const shadow_view &v, const float *depth_px, const vec3d &p,
                               double bias, float strength, float pcf_radius = 0.f,
                               int pcf_taps = 1) {
  int ix = 0, iy = 0;
  double depth = 0.0;
  if (depth_px == nullptr || !v.project(p.to_array(), ix, iy, depth))
    return 1.f; // outside the map: fail LIT, the non-destructive direction
  const int k = pcf_half_width(pcf_radius, pcf_taps);
  if (k == 0)
    return shadow_tap(v, depth_px, ix, iy, depth, bias, strength);
  float sum = 0.f;
  for (int j = -k; j <= k; ++j) {
    const int ty = iy + pcf_offset(j, k, double(pcf_radius));
    for (int i = -k; i <= k; ++i)
      sum += shadow_tap(v, depth_px, ix + pcf_offset(i, k, double(pcf_radius)), ty, depth, bias,
                        strength);
  }
  // Fixed summation order over a fixed grid: the average is reproducible to the
  // bit, which the determinism contract needs.
  return sum / float((2 * k + 1) * (2 * k + 1));
}

// The DEEP lookup.  `profile` is the map's raw f32 payload as (slices + 1)
// PLANES of width*height, plane-major -- texel (x, y) of plane j lives at
// j * width * height + y * width + x:
//
//   plane 0        the exact light-space depth at which this ray's accumulated
//                  alpha first reached opacity_cutoff (+inf if it never did)
//   plane 1 + k    accumulated alpha at knot k+1, i.e. at light-space depth
//                  depth_min + (k+1) * slice_dz, counting only contributions
//                  STRICTLY IN FRONT of the terminal depth above
//
// PLANE-major rather than texel-major, and the honest reason is STRUCTURE, not
// speed: both layouts were built and measured, and on this box they are within
// noise of each other on both backends, for both the lookup and the light
// pass's writes (the rebuild is dominated by the host round trip, and the
// lookup by the terminal channel, which either layout reaches in one load).
// What plane-major buys is that every plane is a plain width x height raster --
// plane 0 in particular has exactly the shape and the meaning of a HARD map's
// depth raster, so a consumer can feed it to code written for that -- and that
// the profile image stays a normal-width raster instead of one
// width * (slices + 1) wide.  It is also the coalescing-friendly layout on the
// device (neighbouring threads hold neighbouring texels), which is why it is
// the one to keep now that the two measure the same.
//
// Two channels, one comparison each, because the renderer's two occluder kinds
// need different things (see shadow_mode in shadow.h):
//
//   * Beyond the terminal depth the answer is `1 - strength`, tested with
//     exactly the expression shadow_visibility() uses -- `depth > stored +
//     bias`, not the algebraically equal `depth - bias > stored`.  That is what
//     makes an opaque occluder reproduce the hard map BIT FOR BIT rather than
//     merely closely.
//   * Otherwise the profile is reconstructed piecewise-linearly.
//
// INTERPOLATION SPACE: linear in ACCUMULATED ALPHA (equivalently in
// transmittance, its affine image), against light-space DEPTH.  Not in optical
// depth / log-transmittance, which is the reflex when a quantity is called
// "transmittance": this renderer composites with the discrete front-to-back
// alpha recurrence, not Beer-Lambert, so there is no tau for a reconstruction
// to be linear in -- and log space cannot represent the fully-blocked
// transmittance 0 that opaque media reach constantly.  Linear in alpha also
// makes the value at a knot EXACT (it is the accumulation a co-located sample
// would have produced) and keeps the reconstruction monotone for free.
//
// Returns 1 - strength * alpha, which collapses to shadow_visibility()'s
// `shadowed ? 1 - strength : 1` whenever alpha is 0 or 1.
// One texel's DEEP lookup, the two-channel body described above.
inline float shadow_tap_deep(const shadow_view &v, const float *profile, int tx, int ty,
                             double depth, double bias, float strength) {
  if (tx < 0 || tx >= v.width || ty < 0 || ty >= v.height)
    return 1.f; // the filter reached off the map: that tap is LIT
  const std::size_t plane = std::size_t(v.width) * std::size_t(v.height);
  const float *cell = profile + std::size_t(ty) * std::size_t(v.width) + std::size_t(tx);
  // Inverted-NaN safe exactly like the hard path: +inf never blocks, and a NaN
  // receiver depth fails the test rather than shadowing at random.
  if (depth > double(cell[0]) + bias)
    return 1.f - strength;

  // The bias shifts the QUERY toward the light so a receiver is not attenuated
  // by its own contribution -- the same job it does in the hard path, where it
  // is added to the stored depth instead.
  const double u = (depth - bias - v.depth_min) / v.slice_dz;
  if (!(u > 0.0)) // in front of the first knot; NaN lands here too
    return 1.f;
  float alpha;
  if (u >= double(v.slices)) {
    alpha = cell[std::size_t(v.slices) * plane];
  } else {
    const int i = int(u); // u > 0 and u < slices, so i is in [0, slices)
    const double f = u - double(i);
    // Knot 0 is implicit: nothing accumulates before the scene box.
    const double a0 = i == 0 ? 0.0 : double(cell[std::size_t(i) * plane]);
    const double a1 = double(cell[std::size_t(i + 1) * plane]);
    alpha = float(a0 + (a1 - a0) * f);
  }
  return 1.f - strength * alpha;
}

inline float shadow_visibility_deep(const shadow_view &v, const float *profile, const vec3d &p,
                                    double bias, float strength, float pcf_radius = 0.f,
                                    int pcf_taps = 1) {
  int ix = 0, iy = 0;
  double depth = 0.0;
  if (profile == nullptr || v.slices <= 0 || !v.project(p.to_array(), ix, iy, depth))
    return 1.f; // outside the map: fail LIT, the non-destructive direction
  const int k = pcf_half_width(pcf_radius, pcf_taps);
  if (k == 0)
    return shadow_tap_deep(v, profile, ix, iy, depth, bias, strength);
  // PCF over a deep map filters the RECONSTRUCTED visibility, not the stored
  // alpha: each tap answers "how much light reaches this depth through THAT
  // column", and averaging those answers is what a partially-occluded footprint
  // means.  Averaging the profiles first and reconstructing once would be
  // cheaper by a hair and wrong at a terminal-depth step, where one column is
  // opaque past a plane the neighbour never reaches.
  float sum = 0.f;
  for (int j = -k; j <= k; ++j) {
    const int ty = iy + pcf_offset(j, k, double(pcf_radius));
    for (int i = -k; i <= k; ++i)
      sum += shadow_tap_deep(v, profile, ix + pcf_offset(i, k, double(pcf_radius)), ty, depth, bias,
                             strength);
  }
  return sum / float((2 * k + 1) * (2 * k + 1));
}

} // namespace detail
} // namespace volren
} // namespace cvc

#endif // CVC_VOLREN_DETAIL_SHADOW_MAP_H
