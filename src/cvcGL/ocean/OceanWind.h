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

// OceanWind — the wind on the GPU ocean (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md E2-E4, E6,
// O13): the GLSL and the per-frame uniforms that make the drawn cascades what
// cvc::ocean::sea_surface's forcing makes the physics sea.
//
//   - Fetch and lee: the cascades' weights times owWindW(p, gust): each cascade's
//     clamp(sqrt(F / F_full), 0, 1), F read from one RG32F texture holding the old and the new
//     keyframe's fetch maps (coarse: at most 256 a side, uploaded only when a keyframe or the coast
//     changes), crossfaded by uSeaMix; times the gust factor for the cascades whose uGustW is 1.
//   - Gusts: cvc_wind_gust (cvc/gl/WindGLSL.h), no texture.
//   - Currents: owCurrent(p), the curl noise with no texture, and owDisp, a cascade read through
//     the two-phase flow map with the current owFlow (set per vertex before the reads).
//
// One sampler in all (uWindFetch); the gusts and the current are integer-hashed noise in the
// shader, evaluated per vertex (the near grid's 2 m, the far grid's coarse cells) and handed to the
// fragments. The h0 crossfade itself is the CPU's (mix_h0), only while a crossfade runs.

#ifndef __CVC_GL_OCEAN_OCEANWIND_H__
#define __CVC_GL_OCEAN_OCEANWIND_H__

#include <array>
#include <cvc/gl/WindGLSL.h>
#include <string>
#include <vector>

namespace cvc {
namespace ocean {
struct fetch_field;
struct sea_forcing;
struct h0_field;
} // namespace ocean
namespace gl {

class GeometryNode;

struct OceanWind {
  // The GLSL for both stages: WindGLSL's, then the fetch weights, the current and the flow map.
  // Needs nothing declared before it.
  static std::string glsl();

  // Each cascade's part in the wind: its fully grown fetch (new and old keyframe), its gust weight
  // and whether the current carries it.
  struct cascades {
    std::array<double, 3> full{}, full_from{}, gust{};
    std::array<bool, 3> advect{};
  };

  // This frame's uniforms.
  struct frame {
    float seaMix = 1.0f;
    std::array<float, 3> full{}, fullFrom{}, gustW{}, advect{};
    WindGLSL::frame wind;
    std::array<float, 3> current{}, curDriftL{}, curDriftS{}, flowTau{}, flowW{}, flowOff0{},
        flowOff1{};
  };
  static frame at(const cvc::ocean::sea_forcing &f, const cascades &c, double t);
  static void apply(GeometryNode &g, const frame &f);

  // The fetch maps as one RG texture's texels (the old keyframe's in R, the new's in G), its size
  // and its grid (origin x, origin y, cell). Without a map, one open texel (no lee).
  struct fetch_texels {
    int cols = 1, rows = 1;
    std::array<float, 3> grid{0.0f, 0.0f, 1.0f};
    std::vector<float> rg;
  };
  static fetch_texels fetch(const cvc::ocean::fetch_field *from, const cvc::ocean::fetch_field *to);
  static const char *fetchName() { return "uWindFetch"; }
  static const char *fetchGridName() { return "uWindFetchGrid"; }

  // (1 - w) from + w to, texel by texel: the h0 a crossfade draws at weight w.
  static void mix_h0(const cvc::ocean::h0_field &from, const cvc::ocean::h0_field &to, double w,
                     cvc::ocean::h0_field &out);
};

} // namespace gl
} // namespace cvc

#endif // __CVC_GL_OCEAN_OCEANWIND_H__
