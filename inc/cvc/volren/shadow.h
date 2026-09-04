// Volumetric shadows for cvc::volren.
//
// One ORTHOGRAPHIC light-view pass per shadow-casting directional light,
// rendered by the SAME marcher: its frame::depth latches at the first
// isosurface hit or the first depth_alpha_threshold crossing, which is exactly
// "where the light stops".  The main march then projects each shaded sample
// into that map and darkens the light's diffuse+specular terms when the sample
// lies behind the recorded depth.
//
// The comparison needs no projection matrix and no near/far round-trip: for an
// orthographic camera every ray shares direction == forward, so frame::depth
// stores exactly dot(p - eye, forward) -- the same quantity project() computes
// for an arbitrary world point.  Misses are +inf, which fails open (lit).
//
// shadow_mode::deep replaces that single scalar with the light ray's
// accumulated opacity AS A FUNCTION OF DEPTH, so a translucent occluder casts a
// partial shadow and a sample deep inside a medium is attenuated by everything
// in front of it.  Both representations share this file's geometry, the same
// bias and the same `strength`; they differ only in the per-texel payload and
// the lookup.  See the shadow_mode comment below and docs/VOLREN_API.md.
//
// Both modes can be SOFTENED: pcf_radius/pcf_taps below box-average the lookup
// over a neighbourhood of the light map, which turns a hard-edged shadow into a
// band of partial visibility.  It is off by default and byte-identical there.
//
// Known contract limits of shadow_mode::hard, all deliberate:
//  - The occluder test is binary, scaled by `strength` (percentage-closer
//    filtering softens the RESULT of that test, but the penumbra it produces is
//    a constant number of texels everywhere rather than one that grows with the
//    occluder-to-receiver distance).
//  - ONE occluder layer per light ray (the latch is a single scalar), so a
//    point behind two thin sheets is exactly as dark as behind one.
//  - Translucency is ignored on the CASTER side; min_occluder_opacity is the
//    knob that keeps a decorative low-opacity shell from eclipsing the body it
//    wraps.
// Both modes share one limit:
//  - volume_settings::unshaded samples are NOT shadowed -- that mode is
//    defined as "TF readout with no lighting model", and there is no light
//    term to attenuate.
#ifndef CVC_VOLREN_SHADOW_H
#define CVC_VOLREN_SHADOW_H

#include <array>
#include <cmath>
#include <cstddef>
#include <cvc/volren/types.h>
#include <vector>

namespace cvc {
namespace volren {

// What one light-view texel stores, and therefore what the lookup can answer.
//
//   hard -- ONE scalar: the depth at which the light stopped (the first
//           isosurface hit, or the first sample past depth_alpha_threshold).
//           Every occluder is fully opaque and every shadow is binary.  This
//           is the DEFAULT, and it is the cheaper of the two in both memory
//           (4 bytes per texel) and lookup (one load, one compare).
//
//   deep  -- a TRANSMITTANCE PROFILE: `depth_slices` accumulated-alpha samples
//           at uniformly spaced light-space depths across the scene's
//           light-depth extent, PLUS one exact terminal depth.  A translucent
//           occluder casts a partial shadow, and a receiver inside a medium is
//           attenuated by exactly what lies between it and the light.
//
// The two-part payload is the whole design, and the split is not arbitrary:
// the renderer's occluders come in exactly two kinds and each gets the
// representation it needs.
//
//   * An ISOSURFACE is a STEP in accumulated alpha at an exactly known depth.
//     Uniform slices cannot represent a step -- they smear it over one slice
//     width, which for the flagship content (an opaque SDF isosurface) would
//     turn every contact shadow into a soft onset several cells long.  So the
//     terminal-depth scalar records the exact ray parameter at which the light
//     ray's accumulated alpha first reached opacity_cutoff, and everything
//     beyond it is blocked with the SAME comparison the hard map uses.  An
//     opaque isosurface therefore reproduces the hard map bit for bit.
//   * A TRANSFER-FUNCTION medium accumulates GRADUALLY, one contribution per
//     cell entered, so a piecewise-linear reconstruction over a handful of
//     knots is genuinely accurate for it -- and it is the case the hard map
//     cannot represent at all.
//
// Contributions at or beyond the terminal depth are deliberately excluded from
// the slices, so a receiver IN FRONT of an opaque occluder is never dimmed by
// the interpolation ramping up early.
enum class shadow_mode {
  hard = 0,
  deep = 1,
};

struct shadow_settings {
  // Off by default: every existing scene renders byte-identically.
  bool enabled = false;

  // Indices into render_settings::lights that cast.  EMPTY means every light
  // casts.  Each entry costs one extra render pass, so the count is capped by
  // limits::max_shadow_maps; render() throws cvc::volren_error above that
  // rather than silently dropping a light.
  std::vector<int> lights;

  // Light-view raster edge (square).  Clamped to [64, limits::max_raster_dim].
  int resolution = defaults::shadow_resolution;

  // 0 = no darkening (visually a no-op, though the light-view pass still
  // runs), 1 = the light contributes nothing where it is blocked.  Ambient is
  // never attenuated.  render() throws cvc::volren_error outside [0, 1].
  float strength = 1.0f;

  // Constant depth bias, in units of the light-view depth latch's own quantum.
  // That quantum is NOT simply the march step: an isosurface latch is an exact
  // ray/MC intersection while a transfer-function latch walks in CELLS, so
  // render() sizes it per scene (detail::shadow_bias carries the measurements).
  // 1.0 is the measured bound; raise it if a thick translucent medium
  // self-shadows at grazing incidence, which is the one case no bias fixes for
  // free.
  float bias_scale = defaults::shadow_bias_scale;

  // Slope-scaled bias, in units of one light-map texel's world width times
  // tan(angle between the surface normal and the light direction).
  float slope_scale = defaults::shadow_slope_scale;

  // An isosurface casts only when its opacity reaches this.  The light pass's
  // depth latch fires on the FIRST isosurface hit regardless of that surface's
  // opacity, so without this a 0.16-opacity decorative shell (volren_bunny
  // --shell) would become the occluder and drop the whole body it wraps into
  // shadow.  Transfer-function volumes still cast through the alpha latch and
  // are not filtered by this.
  //
  // IGNORED in shadow_mode::deep, which needs no such workaround: there a
  // 0.16-opacity shell contributes a 0.16 step to the transmittance profile and
  // dims what it wraps by 16%, which is the right answer rather than a
  // thresholded guess.  Filtering in deep mode would DELETE that occluder
  // instead of representing it.
  float min_occluder_opacity = defaults::shadow_min_occluder_opacity;

  // Which representation the light pass builds and the main march reads.
  // hard is the default, so every existing scene stays byte-identical.
  shadow_mode mode = shadow_mode::hard;

  // Depth slices in a deep map's per-texel profile.  Clamped to
  // [limits::min_shadow_depth_slices, limits::max_shadow_depth_slices].
  // IGNORED in shadow_mode::hard, which stores no profile.
  //
  // Memory, per casting light, on the host AND on the device:
  //   resolution^2 * (depth_slices + 1) * 4 bytes
  // -- 17.8 MB at the defaults (512^2, 16 slices).  Halving `resolution`
  // quarters it; the slice count scales it linearly.
  int depth_slices = defaults::shadow_depth_slices;

  // ---- soft shadows: percentage-closer filtering ---------------------------
  // Half-width of the filter footprint, in light-map TEXELS.  0 -- the default
  // -- takes the single-tap lookup this renderer has always taken, evaluating
  // the same expression bit for bit; anything above it averages a pcf_taps x
  // pcf_taps grid of comparisons spread over [-pcf_radius, +pcf_radius] texels
  // about the receiver's own texel.  Clamped to [0, limits::max_pcf_radius].
  //
  // TEXELS rather than world units, and the trade is real: a texel radius is
  // what bounds the work (the taps are texel reads, and a world radius would
  // have to be converted to one anyway), but it makes the penumbra WIDTH depend
  // on `resolution` -- doubling the map halves the blur in world terms.  Both
  // are quality knobs a caller sets together, and the conversion is one
  // multiply by shadow_view::texel_world, which is public.
  //
  // The filter also widens the slope bias, by (1 + 2 * pcf_radius): the bias
  // exists to cover the depth a receiver can be off by across the LATERAL
  // footprint of what it is compared against, and PCF is exactly the operation
  // that grows that footprint.  Without it the outer taps self-shadow and the
  // soft edge arrives with a ring of acne inside it.  At radius 0 the factor is
  // exactly 1, which is what keeps the default byte-identical.
  float pcf_radius = defaults::shadow_pcf_radius;

  // Taps per EDGE of the filter grid, so the lookup costs pcf_taps^2 texel
  // reads (each of them TWO loads in shadow_mode::deep).  Clamped to
  // [limits::min_pcf_taps, limits::max_pcf_taps]; ignored when pcf_radius is 0.
  //
  // The taps sit on a fixed regular grid, offsets (i - k) * pcf_radius / k for
  // k = (pcf_taps - 1) / 2, with no jitter and no rotated/Poisson disk.  A
  // jittered disk resolves a wide penumbra into more levels for the same tap
  // count, and it is the standard answer -- but determinism is a documented
  // contract of this renderer (byte-identical across runs and thread counts),
  // and a per-pixel rotation is exactly the thing that breaks it.  A fixed grid
  // also keeps the CPU and CUDA paths comparable tap for tap.
  int pcf_taps = defaults::shadow_pcf_taps;
};

// The light-view orthographic frame of one built shadow map.  Public so a
// consumer can test its OWN points against the map -- a ground-plane decal, a
// debug overlay -- without reaching into detail/.
struct shadow_view {
  std::array<double, 3> eye{};     // on the light's eye PLANE
  std::array<double, 3> right{};   // orthonormal basis, matching view_basis
  std::array<double, 3> up{};      // == view_basis::true_up
  std::array<double, 3> forward{}; // -normalize(light.direction); depth grows along it
  double parallel_scale = 1.0;     // half-height AND half-width (aspect is 1)
  int width = 0, height = 0;
  double texel_world = 0.0; // 2 * parallel_scale / height
  int light_index = -1;     // index into render_settings::lights

  // ---- deep maps only; `slices` is 0 for a hard map -------------------------
  // The profile's knot grid, in LIGHT-SPACE DEPTH (the same quantity project()
  // returns).  Knot j (j in [0, slices]) sits at depth_min + j * slice_dz;
  // knot 0 is implicitly zero accumulated alpha (nothing is in front of the
  // scene box), so only the `slices` knots above it are stored.  The grid is
  // fitted to the scene's own light-depth extent, so no knot is spent outside
  // the marched region.
  int slices = 0;
  double depth_min = 0.0; // light-space depth where the scene box starts
  double slice_dz = 0.0;  // (scene light-depth extent) / slices

  double depth_max() const { return depth_min + slice_dz * double(slices); }

  // World point -> integer map texel + light-space depth
  // (depth == dot(p - eye, forward)).  Returns false when p projects outside
  // the map, which callers must treat as LIT: the map is fitted to
  // scene_bounds with a pad, so that only happens on FP edge cases.
  bool project(const std::array<double, 3> &p, int &ix, int &iy, double &depth) const {
    const vec3d rel = vec3d(p) - vec3d(eye);
    const double s = dot(rel, vec3d(right));
    const double t = dot(rel, vec3d(up));
    depth = dot(rel, vec3d(forward));
    if (!(parallel_scale > 0.0) || width <= 0 || height <= 0)
      return false;
    const double u = s / parallel_scale;
    const double v = t / parallel_scale;
    // Inverts ray_generator::at exactly (aspect == 1):
    //   u = (px + 0.5) / W * 2 - 1  ->  px = ((u + 1) * 0.5) * W - 0.5
    //   v = 1 - (py + 0.5) / H * 2  ->  py = ((1 - v) * 0.5) * H - 0.5
    // and the nearest texel is floor(px + 0.5) == floor(((u + 1) * 0.5) * W).
    const double fx = (u + 1.0) * 0.5 * double(width);
    const double fy = (1.0 - v) * 0.5 * double(height);
    // Range-checked in the DOUBLE domain before the cast, so NaN fails the
    // test instead of hitting an undefined double->int conversion (the
    // grid_sampler::cell_index discipline).
    if (!(fx >= 0.0 && fx < double(width)) || !(fy >= 0.0 && fy < double(height)))
      return false;
    ix = int(fx); // fx >= 0, so truncation == floor
    iy = int(fy);
    return true;
  }
};

} // namespace volren
} // namespace cvc

#endif // CVC_VOLREN_SHADOW_H
