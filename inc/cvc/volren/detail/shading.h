// Blinn-Phong shading for volume samples and isosurface hits -- the port of
// vrPhongShading, in linear [0,1] color space.
//
// Deliberate fixes over the legacy code (see docs/VOLREN_API.md "Fidelity"):
//  - multiple lights ACCUMULATE (the legacy `=` made the last light win);
//  - the blue diffuse term uses the blue light channel (was green);
//  - the specular exponent is a real per-material parameter evaluated with
//    std::pow (the legacy ignored Shading::shining and always shaded through
//    a baked x^10 table);
//  - ambient is a real knob (the legacy zeroed it unconditionally), and it is
//    now a per-channel SCALE rather than a scalar, so a sky/ground hemisphere
//    and an occlusion factor can shape it without touching this expression
//    (ambient_scale below builds it; {a,a,a} is the flat legacy term exactly).
//  - the output gain is a parameter rather than a constant.
// Kept: the two-sided option (|N.L|, |N.H|), the one-sided clamp semantics
// (N.L < 0 kills both terms), and the 0.9 gain as that parameter's default.
// Derived from volrover's volren/libiso (C) 2000-2005 University of Texas at
// Austin (Park/Zhang/Rivera, advisor Bajaj), LGPL 2.1 -- the same license as
// libcvc.
#ifndef CVC_VOLREN_DETAIL_SHADING_H
#define CVC_VOLREN_DETAIL_SHADING_H

#include <algorithm>
#include <cmath>
#include <cvc/volren/settings.h>
#include <cvc/volren/types.h>
#include <vector>

namespace cvc {
namespace volren {
namespace detail {

// The per-channel ambient SCALE for one sample: the flat `ambient` constant,
// optionally tinted by a sky/ground hemisphere and attenuated by an ambient
// occlusion factor (`occlusion` 1 == fully open).
//
// Both shaping steps land on AMBIENT and nowhere else, and that is a deliberate
// division of labour rather than a convenience: direct light already carries an
// exact per-light visibility term (the shadow maps), so folding a local
// occlusion estimate into it would darken the same occluder twice, while the
// ambient term is precisely the one that pretends every direction is equally
// visible and therefore the one an occlusion estimate is ABOUT.
//
// The hemisphere mix is not clamped: dot() of two unit vectors is inside
// [-1, 1] to within an ulp, so f is inside [0, 1] to within an ulp, and a clamp
// would buy nothing except a NaN-routing decision that the flat path below does
// not make either.
inline std::array<float, 3> ambient_scale(float ambient, const hemisphere_ambient &hemi,
                                          const vec3d &normal, float occlusion) {
  float r = ambient, g = ambient, b = ambient;
  if (hemi.enabled) {
    // A degenerate `up` normalizes to {0,0,0} (the vrNormalize contract), which
    // puts every normal at the equator: a flat 50/50 blend of the two colours,
    // consistent rather than a special case.
    const vec3d up = normalized(vec3d(hemi.up));
    const float f = float(0.5 + 0.5 * dot(normal, up));
    r *= hemi.ground[0] + (hemi.sky[0] - hemi.ground[0]) * f;
    g *= hemi.ground[1] + (hemi.sky[1] - hemi.ground[1]) * f;
    b *= hemi.ground[2] + (hemi.sky[2] - hemi.ground[2]) * f;
  }
  return {r * occlusion, g * occlusion, b * occlusion};
}

// `normal` must be unit length (callers normalize the spline gradient);
// `view` is the unit vector from the sample TOWARD the viewer;
// `base` is the material color (TF entry or isosurface color);
// `ambient` is the per-channel scale from ambient_scale() -- {a,a,a} is the
// flat constant, and it is what every caller passes when neither the
// hemisphere nor AO is on, so the expression is unchanged bit for bit.
// `light_visibility`, when non-null, is one factor in [0,1] per entry of
// `lights` (volumetric shadows -- shadow.h).  It scales DIFFUSE and SPECULAR
// only: ambient must survive, or a shadowed region crushes to black instead of
// falling into shade.  Null means every light is fully visible, which is the
// pre-shadow expression bit for bit.
// Returns the shaded color, channels clamped to [0,1].
inline std::array<float, 3> blinn_phong(const std::array<float, 3> &base, const vec3d &normal,
                                        const vec3d &view, const std::vector<light> &lights,
                                        bool two_sided, const std::array<float, 3> &ambient,
                                        float shininess, float gain, float specular,
                                        const float *light_visibility = nullptr) {
  float r = ambient[0] * base[0];
  float g = ambient[1] * base[1];
  float b = ambient[2] * base[2];

  for (std::size_t i = 0; i < lights.size(); ++i) {
    const light &l = lights[i];
    const vec3d ldir = normalized(vec3d(l.direction));
    const vec3d half = normalized(ldir + view);
    float ndotl = float(dot(normal, ldir));
    float ndoth = float(dot(normal, half));

    if (two_sided) {
      ndotl = std::fabs(ndotl);
      ndoth = std::fabs(ndoth);
    } else if (ndotl >= 0.f) {
      ndoth = std::max(ndoth, 0.f);
    } else {
      ndotl = 0.f;
      ndoth = 0.f;
    }

    // `specular` folds into the lobe rather than into each channel's sum, so at
    // its 1.0 default the multiply is exact and the expression is unchanged.
    const float spec = ndoth > 0.f ? std::pow(ndoth, shininess) * specular : 0.f;
    const float vis = light_visibility ? light_visibility[i] : 1.f;
    // Scale note: the legacy diffuse term was diffuse*xl*light/256 with
    // 0-255 channels; base*ndotl*light in 0-1 floats is a uniform 256/255
    // (+0.39%) of that -- visually identical.
    r += (base[0] * ndotl * l.color[0] + l.color[0] * spec) * vis;
    g += (base[1] * ndotl * l.color[1] + l.color[1] * spec) * vis;
    b += (base[2] * ndotl * l.color[2] + l.color[2] * spec) * vis;
  }

  return {std::min(gain * r, 1.f), std::min(gain * g, 1.f), std::min(gain * b, 1.f)};
}

} // namespace detail
} // namespace volren
} // namespace cvc

#endif // CVC_VOLREN_DETAIL_SHADING_H
