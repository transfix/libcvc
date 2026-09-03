// Blinn-Phong shading for volume samples and isosurface hits -- the port of
// vrPhongShading, in linear [0,1] color space.
//
// Deliberate fixes over the legacy code (see docs/VOLREN_API.md "Fidelity"):
//  - multiple lights ACCUMULATE (the legacy `=` made the last light win);
//  - the blue diffuse term uses the blue light channel (was green);
//  - the specular exponent is a real per-material parameter evaluated with
//    std::pow (the legacy ignored Shading::shining and always shaded through
//    a baked x^10 table);
//  - ambient is a real knob (the legacy zeroed it unconditionally).
// Kept: the two-sided option (|N.L|, |N.H|), the one-sided clamp semantics
// (N.L < 0 kills both terms), and the 0.9 output gain.
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

// `normal` must be unit length (callers normalize the spline gradient);
// `view` is the unit vector from the sample TOWARD the viewer;
// `base` is the material color (TF entry or isosurface color).
// Returns the shaded color, channels clamped to [0,1].
inline std::array<float, 3> blinn_phong(const std::array<float, 3> &base, const vec3d &normal,
                                        const vec3d &view, const std::vector<light> &lights,
                                        bool two_sided, float ambient, float shininess) {
  float r = ambient * base[0];
  float g = ambient * base[1];
  float b = ambient * base[2];

  for (const light &l : lights) {
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

    const float spec = ndoth > 0.f ? std::pow(ndoth, shininess) : 0.f;
    // Scale note: the legacy diffuse term was diffuse*xl*light/256 with
    // 0-255 channels; base*ndotl*light in 0-1 floats is a uniform 256/255
    // (+0.39%) of that -- visually identical.
    r += base[0] * ndotl * l.color[0] + l.color[0] * spec;
    g += base[1] * ndotl * l.color[1] + l.color[1] * spec;
    b += base[2] * ndotl * l.color[2] + l.color[2] * spec;
  }

  const float gain = defaults::shading_gain;
  return {std::min(gain * r, 1.f), std::min(gain * g, 1.f), std::min(gain * b, 1.f)};
}

} // namespace detail
} // namespace volren
} // namespace cvc

#endif // CVC_VOLREN_DETAIL_SHADING_H
