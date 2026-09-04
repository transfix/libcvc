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

// Hemispheric ambient: a sky colour overhead, a ground colour underfoot, mixed
// by the shaded sample's own normal.
//
// `render_settings::ambient` alone is a FLAT constant added to every sample
// whatever way it faces, which is the one term in this shading model that
// carries no information about the surface -- an upward-facing plane and a
// downward-facing one get exactly the same fill.  A two-colour hemisphere is
// the cheapest thing that fixes that (one dot product and three lerps), and it
// is what makes an unlit side read as "in shade under a sky" rather than as a
// uniformly dimmed copy of the lit side.
//
// It SHAPES the ambient constant rather than replacing it: the result is
// `ambient * mix(ground, sky, 0.5 + 0.5 * dot(N, up)) * base`, so `ambient`
// stays the single intensity knob and the two colours are pure tint.  With
// both colours white the mix is exactly {1,1,1} for every normal -- a0 == a1
// makes `a0 + (a1 - a0) * f` exact -- which is why `enabled` costs nothing to
// leave false and changes nothing when a caller turns it on with the defaults.
struct hemisphere_ambient {
  bool enabled = false;
  std::array<float, 3> sky{1.f, 1.f, 1.f};    // toward +up
  std::array<float, 3> ground{1.f, 1.f, 1.f}; // toward -up
  // Normalized on use; cvc scenes are Z-up (the same convention
  // detail::fit_light_camera prefers).  A degenerate vector normalizes to zero
  // (the vrNormalize contract), which puts every normal at the equator and
  // makes the hemisphere a flat 50/50 blend -- consistent, not a special case.
  std::array<double, 3> up{0.0, 0.0, 1.0};
};

// Ambient occlusion for SIGNED-DISTANCE volumes (volume_settings::distance_field).
//
// It attenuates the AMBIENT term only, which is both the physically-motivated
// place -- AO approximates how much of the ambient hemisphere a point can see --
// and the only place it can go without double-counting: direct light already
// has an exact visibility term (the shadow maps), and multiplying that by a
// local occlusion estimate would darken the same occluder twice.  A consequence
// worth stating plainly: with `render_settings::ambient` at 0 -- the default --
// AO has nothing to attenuate and changes no pixel.
//
// SCOPE: isosurface hits, on volumes whose settings declare `distance_field`.
// Transfer-function media are deliberately excluded; the reasoning is in
// docs/VOLREN_API.md, and it is a cost argument, not an aesthetic one (a shaded
// TF sample fires once per CELL ENTERED, hundreds of times along a ray, where
// an isosurface hit fires once or twice).
struct ao_settings {
  // 0 -- the default -- skips the cone entirely: no taps, and the shading
  // expression is the one evaluated before AO existed, bit for bit.  1 lets a
  // fully enclosed point lose all of its ambient.
  float strength = defaults::ao_strength;

  // How far the cone reaches, in the volume's OWN local units (which are world
  // units for the identity and translation transforms every scene here uses;
  // under a scaled model_transform they are the volume's, not the world's).
  // 0 is off, like strength 0.
  //
  // This is the knob that decides what AO is ABOUT: at a fraction of a cell it
  // sees only the surface's own curvature, at several cells it darkens the
  // creases between folds, and at a large fraction of the object it starts to
  // read as a contact term.  It is not resolution-relative on purpose -- a
  // crevice is a size in the world, not a count of voxels.
  double radius = defaults::ao_radius;

  // Taps along the cone, uniformly spaced in distance.  This is the
  // QUALITY/COST knob, and the cost is exact: `samples` trilinear fetches per
  // shaded isosurface hit per volume.  Clamped to
  // [limits::min_ao_samples, limits::max_ao_samples].
  int samples = defaults::ao_samples;
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

  // This volume's scalars are a SIGNED DISTANCE field in local units, positive
  // OUTSIDE the surface -- the cvc::sdf convention.  It is a claim about the
  // DATA, not a rendering mode: nothing else in the renderer reads it, and the
  // only consumer is ao_settings, whose cone trace is sound exactly because
  // `f(q) - isovalue` is then the distance from q to that isosurface.  False
  // (the default) makes the volume contribute no occlusion, which is the honest
  // answer for a field where that subtraction means nothing.
  bool distance_field = false;

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
  // Shape the flat `ambient` by a sky/ground hemisphere.  Off by default.
  hemisphere_ambient ambient_hemisphere;
  // Attenuate `ambient` by local occlusion on distance-field volumes.  Off by
  // default (strength and radius both 0).
  ao_settings ao;
  // Output gain applied to the whole shaded colour, then clamped per channel.
  //
  // It is the legacy vrPhongShading damping, kept at its 0.9 so every existing
  // image is unchanged, but it is now a KNOB because 0.9 is not a neutral
  // choice and it is applied to the ambient term too: a surface with
  // `ambient` 1 and no lights renders at 0.9 * base, i.e. the renderer cannot
  // reproduce its own material colour without setting this to 1.  Stacking
  // shadows and AO on top of a fixed 0.9 is what makes that visible, which is
  // why it stops being a constant here.
  float shading_gain = defaults::shading_gain;
  // Scene-level specular reflectance, multiplying every light's specular lobe.
  //
  // The legacy model has NO specular material term at all: the highlight is
  // added at the light's full colour, on top of a diffuse term that already
  // reaches the material colour, so a white key can push a fully-lit sample to
  // 1.85 before the clamp -- and it does.  Measured on the flagship bunny at
  // its shipped settings (ambient 0.25, one white key), 15.8% of the object's
  // pixels lose energy to the per-channel clamp, worst overshoot 223/255 -- and
  // at specular 0 the same scene clamps NOTHING, so this term is the whole
  // cause rather than a contributor.  1.0 is that behaviour exactly, kept
  // as the default; it is a scene knob rather than a per-surface one because
  // the per-surface list has a fixed state encoding (state_settings.h) and this
  // does not need to break it.
  float specular = defaults::specular;
  int steps = defaults::steps; // samples along the scene bbox diagonal
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
