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
// Known contract limits, all deliberate (see docs/VOLREN_API.md):
//  - HARD shadows only: the test is binary, scaled by `strength`.
//  - ONE occluder layer per light ray (the latch is a single scalar), so a
//    point behind two thin sheets is exactly as dark as behind one.
//  - volume_settings::unshaded samples are NOT shadowed -- that mode is
//    defined as "TF readout with no lighting model", and there is no light
//    term to attenuate.
//  - Translucency is ignored on the CASTER side; min_occluder_opacity is the
//    knob that keeps a decorative low-opacity shell from eclipsing the body it
//    wraps.
#ifndef CVC_VOLREN_SHADOW_H
#define CVC_VOLREN_SHADOW_H

#include <array>
#include <cmath>
#include <cstddef>
#include <cvc/volren/types.h>
#include <vector>

namespace cvc {
namespace volren {

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
  float min_occluder_opacity = defaults::shadow_min_occluder_opacity;
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
