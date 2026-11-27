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

// StormFrontGLSL.h — the storms' gust fronts in a shader (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md
// F4): cvc::algo::add_gust_front (cvc/algo/gust_front.h) in GLSL ES 3.00, so the water's ripples
// and roughness and the trees' lean take the front where each wave and tree is, as the physics sea
// does on the CPU (cvc::ocean::sea_forcing::fronts) at the same (x, y, t).
//
//   shader:    StormFrontGLSL::glsl() spliced into a stage, then cvc_storm_front(p) (m/s, x y)
//              or cvc_front_ratio(p) (the wind there over the reference wind: front_wind_ratio)
//   per frame: StormFrontGLSL::frame f = StormFrontGLSL::at(cells, ux, uy, ref);
//              f.apply(set) or its vec4s by name (uniforms())
//
// Uniforms only: up to four cells (the strongest), two vec4s each, their widths in one vec4 and the
// mean wind with the reference in one more (ten vec4s), no texture (WebGL2's units are a hard
// budget). uFrontU.w is 0 with no front, and both functions then cost one branch.

#ifndef __CVC_GL_STORM_FRONT_GLSL_H__
#define __CVC_GL_STORM_FRONT_GLSL_H__

#include <array>
#include <cmath>
#include <cvc/algo/gust_front.h>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace gl {

struct StormFrontGLSL {
  static constexpr int kCells = 4;
  // Uniforms (vec4s): uFrontC0..3 (x, y, radius, outflow at the edge; 0 outflow: no cell),
  // uFrontM0..3 (the motion's unit direction x y, ahead, behind), uFrontW (each cell's width),
  // uFrontU (the mean wind x y, the reference wind's speed, 1 when any cell blows).
  static const char *glsl() {
    return "uniform highp vec4 uFrontC0; uniform highp vec4 uFrontC1; uniform highp vec4 uFrontC2; "
           "uniform highp vec4 uFrontC3;\n"
           "uniform highp vec4 uFrontM0; uniform highp vec4 uFrontM1; uniform highp vec4 uFrontM2; "
           "uniform highp vec4 uFrontM3;\n"
           "uniform highp vec4 uFrontW; uniform highp vec4 uFrontU;\n"
           // one cell's outflow at p (cvc::algo::add_gust_front)
           "vec2 cvc_front_cell(vec2 p, vec4 c, vec4 m, float width){\n"
           "  if (c.w <= 0.0 || c.z <= 0.0) return vec2(0.0);\n"
           "  float most = max(m.z, m.w);\n"
           "  vec2 d = p - c.xy;\n"
           "  float d2 = dot(d, d);\n"
           "  float reach = c.z * most * (1.0 + width);\n"
           "  if (d2 <= 0.0 || d2 >= reach * reach) return vec2(0.0);\n"
           "  float dist = sqrt(d2);\n"
           "  float along = dot(d, m.xy) / dist;\n"
           "  float share = 0.5 * (m.z + m.w) + 0.5 * (m.z - m.w) * along;\n"
           "  float q = dist / (c.z * share);\n"
           "  float shape = q * (2.0 - q);\n"
           "  if (q > 1.0) {\n"
           "    float u = (q - 1.0) / width;\n"
           "    if (u >= 1.0) return vec2(0.0);\n"
           "    shape = (1.0 - u * u) * (1.0 - u * u);\n"
           "  }\n"
           "  return (c.w * shape * (share / most) / dist) * d;\n"
           "}\n"
           // every front at p (m/s)
           "vec2 cvc_storm_front(vec2 p){\n"
           "  if (uFrontU.w <= 0.0) return vec2(0.0);\n"
           "  return cvc_front_cell(p, uFrontC0, uFrontM0, uFrontW.x) +\n"
           "         cvc_front_cell(p, uFrontC1, uFrontM1, uFrontW.y) +\n"
           "         cvc_front_cell(p, uFrontC2, uFrontM2, uFrontW.z) +\n"
           "         cvc_front_cell(p, uFrontC3, uFrontM3, uFrontW.w);\n"
           "}\n"
           // the wind at p over the reference wind (cvc::algo::front_wind_ratio)
           "float cvc_front_ratio(vec2 p){\n"
           "  if (uFrontU.w <= 0.0) return 1.0;\n"
           "  return clamp(length(uFrontU.xy + cvc_storm_front(p)) / max(uFrontU.z, 0.5), 0.0, "
           "4.0);\n"
           "}\n";
  }

  // This frame's uniforms: the first kCells cells (the caller's strongest), the mean wind (ux, uy)
  // and the reference wind's speed.
  struct frame {
    std::array<std::array<float, 4>, kCells> c{}, m{};
    std::array<float, 4> w{}, u{};
    // name, value: what a node sets (setSharedShaderInputs' f4, or setShaderUniform4f)
    std::vector<std::pair<std::string, std::array<float, 4>>> uniforms() const {
      std::vector<std::pair<std::string, std::array<float, 4>>> out;
      for (int i = 0; i < kCells; ++i) {
        out.emplace_back("uFrontC" + std::to_string(i), c[i]);
        out.emplace_back("uFrontM" + std::to_string(i), m[i]);
      }
      out.emplace_back("uFrontW", w);
      out.emplace_back("uFrontU", u);
      return out;
    }
  };
  static frame at(const std::vector<cvc::algo::front_cell> &cells, double ux, double uy,
                  double ref) {
    frame f;
    int n = 0;
    for (const cvc::algo::front_cell &cell : cells) {
      if (n >= kCells)
        break;
      const double mv = std::sqrt(cell.vx * cell.vx + cell.vy * cell.vy);
      f.c[n] = {float(cell.x), float(cell.y), float(cell.r), float(cell.out)};
      f.m[n] = {float(mv > 0.0 ? cell.vx / mv : 0.0), float(mv > 0.0 ? cell.vy / mv : 0.0),
                float(cell.ahead), float(cell.behind)};
      f.w[n] = float(cell.width);
      ++n;
    }
    for (int i = n; i < kCells; ++i) {
      f.m[i] = {0.0f, 0.0f, 1.0f, 1.0f}; // inert: no outflow, and no division by 0
      f.w[i] = 1.0f;
    }
    f.u = {float(ux), float(uy), float(ref), n > 0 ? 1.0f : 0.0f};
    return f;
  }
};

} // namespace gl
} // namespace cvc

#endif // __CVC_GL_STORM_FRONT_GLSL_H__
