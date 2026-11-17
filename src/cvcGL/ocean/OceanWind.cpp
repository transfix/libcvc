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

#include "OceanWind.h"

#include <algorithm>
#include <cvc/gl/GeometryNode.h>
#include <cvc/ocean/current.h>
#include <cvc/ocean/fetch.h>
#include <cvc/ocean/h0.h>
#include <cvc/ocean/sea_lag.h>
#include <cvc/ocean/sea_surface.h>

namespace cvc {
namespace gl {

namespace {

// The fetch maps' texture and their read, left out when the caller reads its own (glsl(true)).
const char *const kFetchDec =
    "uniform highp sampler2D uWindFetch; uniform highp vec3 uWindFetchGrid;\n";
const char *const kOceanWindUniforms =
    "uniform highp vec3 uFetchFull; uniform highp vec3 uFetchFullFrom; uniform highp float "
    "uSeaMix;\n"
    "uniform highp vec3 uGustW; uniform highp vec3 uAdvect;\n"
    // the current: (gain, 1/L, 1/Ls) and each scale's drift; the flow map: each phase's time,
    // weight and offset (a share of the tile)
    "uniform highp vec3 uCurrent; uniform highp vec3 uCurDriftL; uniform highp vec3 uCurDriftS;\n"
    "uniform highp vec3 uFlowTau; uniform highp vec3 uFlowW;\n"
    "uniform highp vec3 uFlowOff0; uniform highp vec3 uFlowOff1;\n"
    "vec2 owFlow = vec2(0.0);\n"; // the current where the cascades are read (owDisp)
// the old and the new keyframe's fetch at p (m), bilinear by hand, clamped to the map
const char *const kFetchRead =
    "vec2 owFetch(vec2 p){\n"
    "  ivec2 sz = textureSize(uWindFetch, 0);\n"
    "  vec2 f = (p - uWindFetchGrid.xy) / uWindFetchGrid.z;\n"
    "  vec2 fl = floor(f), fr = f - fl;\n"
    "  ivec2 i0 = clamp(ivec2(fl), ivec2(0), sz - ivec2(1));\n"
    "  ivec2 i1 = clamp(ivec2(fl) + ivec2(1), ivec2(0), sz - ivec2(1));\n"
    "  vec2 a = mix(texelFetch(uWindFetch, i0, 0).rg, texelFetch(uWindFetch, ivec2(i1.x, i0.y), "
    "0).rg, fr.x);\n"
    "  vec2 b = mix(texelFetch(uWindFetch, ivec2(i0.x, i1.y), 0).rg, texelFetch(uWindFetch, i1, "
    "0).rg, fr.x);\n"
    "  return mix(a, b, fr.y);\n"
    "}\n";
const char *const kOceanWindFuncs =
    "float owScale(float F, float full){\n"
    "  return full > 0.0 ? clamp(sqrt(max(F, 0.0)/full), 0.0, 1.0) : 1.0;\n"
    "}\n"
    // each cascade's weight from the wind at p: the fetch (crossfading between the keyframes'),
    // times the gust for the cascades it moves
    "vec3 owWindW(vec2 p, float gust){\n"
    "  vec2 F = owFetch(p);\n"
    "  vec3 s1 = vec3(owScale(F.y, uFetchFull.x), owScale(F.y, uFetchFull.y), owScale(F.y, "
    "uFetchFull.z));\n"
    "  vec3 s0 = vec3(owScale(F.x, uFetchFullFrom.x), owScale(F.x, uFetchFullFrom.y), owScale(F.x, "
    "uFetchFullFrom.z));\n"
    "  return mix(s0, s1, uSeaMix) * max(vec3(0.0), vec3(1.0) + uGustW*(gust - 1.0));\n"
    "}\n"
    // the current at p (m/s): the curl of two scales of noise (cvc::ocean::current_velocity)
    "vec2 owCurrent(vec2 p){\n"
    "  if (uCurrent.x <= 0.0) return vec2(0.0);\n"
    "  vec3 a = cvc_noise(p*uCurrent.y - uCurDriftL.xy, 0xC022u);\n"
    "  vec3 b = cvc_noise(p*uCurrent.z - uCurDriftS.xy, 0xC022u + 17u);\n"
    "  return uCurrent.x * vec2(a.z + 0.45*b.z, -(a.y + 0.45*b.y));\n"
    "}\n"
    // a cascade at world w, read nearest (halfTexel), through the flow map when the current carries
    // it: two reads half a period apart, blended so the variance holds
    "vec4 owDisp(highp sampler2D t, vec2 w, float tile, float halfTexel, float adv){\n"
    "  if (adv < 0.5 || uCurrent.x <= 0.0) return textureLod(t, w/tile + halfTexel, 0.0);\n"
    "  vec2 q0 = w - owFlow*uFlowTau.x + uFlowOff0.xy*tile;\n"
    "  vec2 q1 = w - owFlow*uFlowTau.y + uFlowOff1.xy*tile;\n"
    "  const vec4 one = vec4(0.0, 0.0, 0.0, 1.0);\n" // the Jacobian blends about 1
    "  vec4 a = textureLod(t, q0/tile + halfTexel, 0.0) - one;\n"
    "  vec4 b = textureLod(t, q1/tile + halfTexel, 0.0) - one;\n"
    "  return uFlowW.x*a + uFlowW.y*b + one;\n"
    "}\n";

} // namespace

std::string OceanWind::glsl(bool ownFetch) {
  return std::string(WindGLSL::glsl()) + (ownFetch ? "" : kFetchDec) + kOceanWindUniforms +
         (ownFetch ? "" : kFetchRead) + kOceanWindFuncs;
}

OceanWind::frame OceanWind::at(const cvc::ocean::sea_forcing &f, const cascades &c, double t) {
  frame o;
  o.seaMix = static_cast<float>(cvc::ocean::crossfade_weight(t, f.t0, f.t1));
  for (int i = 0; i < 3; ++i) {
    o.full[i] = f.fetch ? static_cast<float>(c.full[i]) : 0.0f;
    o.fullFrom[i] =
        f.fetch ? static_cast<float>(c.full_from[i] > 0.0 ? c.full_from[i] : c.full[i]) : 0.0f;
    o.gustW[i] = static_cast<float>(c.gust[i]);
    o.advect[i] = c.advect[i] ? 1.0f : 0.0f;
  }
  o.wind = WindGLSL::at(f.wind, t);
  const cvc::ocean::current_frame cf = cvc::ocean::current_frame_at(f.current, t);
  const double L = std::max(f.current.scale_m, 1e-3);
  o.current = {static_cast<float>(cf.gain), static_cast<float>(1.0 / L),
               static_cast<float>(1.0 / (L * cvc::ocean::kCurrentSmall))};
  o.curDriftL = {static_cast<float>(cf.drift[0]), static_cast<float>(cf.drift[1]), 0.0f};
  o.curDriftS = {static_cast<float>(cf.drift[2]), static_cast<float>(cf.drift[3]), 0.0f};
  o.flowTau = {static_cast<float>(cf.tau[0]), static_cast<float>(cf.tau[1]), 0.0f};
  o.flowW = {static_cast<float>(cf.weight[0]), static_cast<float>(cf.weight[1]), 0.0f};
  o.flowOff0 = {static_cast<float>(cf.offset[0][0]), static_cast<float>(cf.offset[0][1]), 0.0f};
  o.flowOff1 = {static_cast<float>(cf.offset[1][0]), static_cast<float>(cf.offset[1][1]), 0.0f};
  return o;
}

void OceanWind::apply(GeometryNode &g, const frame &f) {
  g.setShaderUniformf("uSeaMix", f.seaMix);
  g.setShaderUniform3f("uFetchFull", f.full[0], f.full[1], f.full[2]);
  g.setShaderUniform3f("uFetchFullFrom", f.fullFrom[0], f.fullFrom[1], f.fullFrom[2]);
  g.setShaderUniform3f("uGustW", f.gustW[0], f.gustW[1], f.gustW[2]);
  g.setShaderUniform3f("uAdvect", f.advect[0], f.advect[1], f.advect[2]);
  auto v3 = [&g](const char *name, const std::array<float, 3> &v) {
    g.setShaderUniform3f(name, v[0], v[1], v[2]);
  };
  v3("uWindGust", f.wind.gust);
  v3("uWindDriftL", f.wind.driftL);
  v3("uWindDriftF", f.wind.driftF);
  v3("uCurrent", f.current);
  v3("uCurDriftL", f.curDriftL);
  v3("uCurDriftS", f.curDriftS);
  v3("uFlowTau", f.flowTau);
  v3("uFlowW", f.flowW);
  v3("uFlowOff0", f.flowOff0);
  v3("uFlowOff1", f.flowOff1);
}

OceanWind::fetch_texels OceanWind::fetch(const cvc::ocean::fetch_field *from,
                                         const cvc::ocean::fetch_field *to) {
  fetch_texels out;
  if (!to || to->empty()) {
    out.rg = {1.0e9f, 1.0e9f}; // open water: every cascade at full height
    return out;
  }
  const bool same = from && !from->empty() && from->rows == to->rows && from->cols == to->cols &&
                    from->min_x == to->min_x && from->min_y == to->min_y && from->cell == to->cell;
  out.cols = to->cols;
  out.rows = to->rows;
  out.grid = {static_cast<float>(to->min_x), static_cast<float>(to->min_y),
              static_cast<float>(to->cell)};
  out.rg.resize(to->count() * 2);
  for (std::size_t i = 0; i < to->count(); ++i) {
    out.rg[i * 2] = same ? from->fetch_m[i] : to->fetch_m[i];
    out.rg[i * 2 + 1] = to->fetch_m[i];
  }
  return out;
}

void OceanWind::mix_h0(const cvc::ocean::h0_field &from, const cvc::ocean::h0_field &to, double w,
                       cvc::ocean::h0_field &out) {
  out.n = to.n;
  out.tile_m = to.tile_m;
  out.texels.resize(to.texels.size());
  const float a = static_cast<float>(w);
  if (from.texels.size() != to.texels.size()) {
    out.texels = to.texels;
    return;
  }
  for (std::size_t i = 0; i < to.texels.size(); ++i)
    out.texels[i] = from.texels[i] + (to.texels[i] - from.texels[i]) * a;
}

} // namespace gl
} // namespace cvc
