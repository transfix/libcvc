// Settings surface for cvc::volren.
//
// This is the typed replacement for the legacy `.cnf` config + vrSet* API:
// every knob the tracer actually consumed survives here (materials/opacity
// trapezoids and ColorMode were parsed-but-dead and are dropped; cut planes
// were declared-but-ignored and are now actually implemented).  The
// state-tree binding in state_settings.h mirrors this surface 1:1.
#ifndef CVC_VOLREN_SETTINGS_H
#define CVC_VOLREN_SETTINGS_H

#include <cvc/volren/shadow.h>
#include <cvc/volren/transfer_function.h>
#include <cvc/volren/types.h>
#include <vector>

namespace cvc {
namespace volren {

// Directional light.  `direction` points TOWARD the light (the legacy Light
// commented "position" but shaded with the normalized vector directly).
// Multiple lights ACCUMULATE -- the legacy overwrite bug is fixed.
struct light {
  std::array<float, 3> color{1.f, 1.f, 1.f};      // [0,1]
  std::array<double, 3> direction{0.0, 0.0, 1.0}; // normalized on use
};

// One isosurface rendered by per-cell marching-cubes ray intersection.
struct isosurface {
  double value = 0.0;   // raw value domain
  float opacity = 1.0f; // constant per surface (legacy contract)
  std::array<float, 3> color{1.f, 1.f, 1.f};
  float shininess = defaults::shininess; // real exponent; legacy ignored its own
};

// Half-space clip: sample points with dot(p - point, normal) < 0 are culled.
struct cut_plane {
  std::array<double, 3> point{0.0, 0.0, 0.0};
  std::array<double, 3> normal{0.0, 0.0, 1.0};
};

// Per-volume settings.  The legacy RenderMode bitmask maps to:
//   RAY_CASTING -> shaded,  COL_DENSITY -> unshaded,
//   ISO_SURFACE -> !isosurfaces.empty().
struct volume_settings {
  bool shaded = true;    // TF sample x gradient ramp, Blinn-Phong shaded
  bool unshaded = false; // TF sample composited without shading

  // Model matrix placing this volume in the world -- row-major, column-vector
  // points, i.e. a cvcGL scene-graph node's composed world transform
  // (GraphicsNode "matrix" state key) verbatim.  Rays are marched in world
  // space and sampled in volume-local space through the affine inverse;
  // gradients/normals come back out through the inverse-transpose.
  mat4 model_transform;

  transfer_function tf;
  // Bake the TF over the volume's [min,max] (legacy behavior after load);
  // false bakes over the TF's own control-point extent.
  bool tf_auto_domain = true;

  gradient_opacity_ramp gradient_ramp;

  // Density window: samples outside [window_min, window_max] are skipped
  // (the legacy dual-purpose min_den/max_den, now a single-purpose knob).
  bool window_enabled = false;
  double window_min = 0.0;
  double window_max = 0.0;

  std::vector<isosurface> isosurfaces;
};

// Scene-level settings.
struct render_settings {
  std::array<float, 3> background{0.f, 0.f, 0.f}; // [0,1]
  std::vector<light> lights;
  std::vector<cut_plane> cut_planes;
  // Volumetric shadows (shadow.h).  OFF by default: with `enabled` false the
  // renderer takes exactly the code path it took before shadows existed, on
  // both backends.
  shadow_settings shadows;
  bool two_sided_lighting = false; // legacy light_both
  float ambient = 0.0f;            // legacy zeroed ambient; now a real knob
  int steps = defaults::steps;     // samples along the scene bbox diagonal
  float opacity_cutoff = defaults::opacity_cutoff;
  float depth_alpha_threshold = defaults::depth_alpha_threshold;
  unsigned threads = 0; // 0 => thread pool default; 1 => serial

  // Supersampled anti-aliasing: sub-samples per pixel EDGE.  n casts an n x n
  // REGULAR grid of rays at sub-pixel offsets ((i+0.5)/n, (j+0.5)/n) and
  // box-filters them into the one output pixel, so the cost is exactly n^2
  // rays.  Named for the grid edge rather than "samples_per_pixel" because a
  // pixel gets n^2 of those, not n.  Must be in [1, limits::max_supersample];
  // 1 is a single ray through the pixel center -- bit-identical to the
  // renderer before supersampling existed, on both backends.
  //
  // This is the EDGE-QUALITY knob and it is orthogonal to the raster size (the
  // latency knob, cvc::gl::VolRenNode::setResolutionScale on the cvcGL side):
  // supersampling sharpens silhouettes at a fixed output resolution, raster
  // size trades output resolution for time.  See docs/VOLREN_API.md.
  int supersample = defaults::supersample;
};

} // namespace volren
} // namespace cvc

#endif // CVC_VOLREN_SETTINGS_H
