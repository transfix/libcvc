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

// WindGLSL.h — the scene's gusts in a shader: cvc::algo::wind_gust (cvc/algo/wind_field.h) in GLSL
// ES 3.00, so whatever a shader draws in the wind (the water's ripples and roughness; later the
// clouds and the trees, O14) agrees with what the CPU and physics sample at the same (x, y, t).
//
//   shader:    WindGLSL::glsl() spliced into a stage, then cvc_wind_gust(p, fine)
//   per frame: WindGLSL::frame u = WindGLSL::at(wind, t);
//              its three vec3s to uWindGust, uWindDriftL and uWindDriftF
//
// No texture: the noise is hashed from integers in uint arithmetic, as the CPU does, so it costs a
// handful of ALU operations and no sampler (WebGL2's units are a hard budget). The drift is
// computed once a frame in double precision, reduced modulo the lattice's period, so a float
// shader reads the same pattern after hours of wind. `fine` (0..1) fades the cat's paws out with
// distance: evaluate it per vertex on a coarse grid and pass it on, rather than per fragment.

#ifndef __CVC_GL_WIND_GLSL_H__
#define __CVC_GL_WIND_GLSL_H__

#include <array>
#include <cvc/algo/wind_field.h>

namespace cvc {
namespace gl {

struct WindGLSL {
  // Uniforms (vec3s): uWindGust (gustiness, pulse, 0); uWindDriftL and uWindDriftF, wind_drift's
  // large and fine (x, y, 0).
  // Functions: cvc_lattice_hash, cvc_noise (value, d/dx, d/dy) and cvc_wind_gust(p, fine).
  static const char *glsl() {
    return "uniform highp vec3 uWindGust; uniform highp vec3 uWindDriftL; uniform highp vec3 "
           "uWindDriftF;\n"
           "uint cvc_lattice_hash(ivec2 c, uint seed){\n"
           "  uvec2 u = uvec2(c) & uvec2(4095u);\n"
           "  uint h = u.x * 0x8da6b343u;\n"
           "  h ^= u.y * 0xd8163841u;\n"
           "  h ^= seed * 0xcb1ab31fu;\n"
           "  h ^= h >> 16u; h *= 0x7feb352du; h ^= h >> 15u; h *= 0x846ca68bu; h ^= h >> 16u;\n"
           "  return h;\n"
           "}\n"
           "float cvc_lattice_corner(ivec2 c, uint seed){\n"
           "  return float(cvc_lattice_hash(c, seed) >> 8u) * (1.0/16777216.0);\n"
           "}\n"
           // value noise, quintic: (value, d/dx, d/dy) per lattice unit
           "vec3 cvc_noise(vec2 p, uint seed){\n"
           "  vec2 fl = floor(p), f = p - fl;\n"
           "  ivec2 i = ivec2(fl);\n"
           "  float a = cvc_lattice_corner(i, seed), b = cvc_lattice_corner(i + ivec2(1, 0), "
           "seed);\n"
           "  float c = cvc_lattice_corner(i + ivec2(0, 1), seed);\n"
           "  float d = cvc_lattice_corner(i + ivec2(1, 1), seed);\n"
           "  vec2 s = f*f*f*(f*(f*6.0 - 15.0) + 10.0);\n"
           "  vec2 ds = 30.0*f*f*(f - 1.0)*(f - 1.0);\n"
           "  float ab = a + (b - a)*s.x, cd = c + (d - c)*s.x;\n"
           "  return vec3(ab + (cd - ab)*s.y, ds.x*((b - a) + ((d - c) - (b - a))*s.y), "
           "ds.y*(cd - ab));\n"
           "}\n"
           // the factor on the mean wind at world p (cvc::algo::wind_gust)
           "float cvc_wind_gust(vec2 p, float fine){\n"
           "  if (uWindGust.x <= 0.0) return 1.0;\n"
           "  float n1 = cvc_noise(p*(1.0/120.0) - uWindDriftL.xy, 0x6057u).x;\n"
           "  float f = clamp(fine, 0.0, 1.0);\n"
           "  float n2 = f > 0.0 ? cvc_noise(p*(1.0/37.0) - uWindDriftF.xy, 0x6058u).x : 0.5;\n"
           "  float n = 0.65*n1 + 0.35*(0.5 + f*(n2 - 0.5));\n"
           "  return max(0.0, 1.0 + uWindGust.x*(2.4*(n - 0.5) + 0.5*uWindGust.y));\n"
           "}\n";
  }

  // This frame's uniforms for wind w at clock time t.
  struct frame {
    std::array<float, 3> gust{}, driftL{}, driftF{};
  };
  static frame at(const cvc::algo::wind_state &w, double t) {
    const cvc::algo::gust_frame g = cvc::algo::gust_frame_at(w, t);
    frame f;
    f.gust = {static_cast<float>(g.gustiness), static_cast<float>(g.pulse), 0.0f};
    f.driftL = {static_cast<float>(g.drift[0]), static_cast<float>(g.drift[1]), 0.0f};
    f.driftF = {static_cast<float>(g.drift[2]), static_cast<float>(g.drift[3]), 0.0f};
    return f;
  }
};

} // namespace gl
} // namespace cvc

#endif // __CVC_GL_WIND_GLSL_H__
