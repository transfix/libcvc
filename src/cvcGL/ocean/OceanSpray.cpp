/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  ---------------------------------------------------------------------------
  The spray's flight and look follow ABYSSAL's spray particles
  (github.com/Token-Gremlin/natural-disasters, src/weather/Precipitation.js at 849ff7f4), MIT
  licensed, Copyright (c) 2026 Davi (Token-Gremlin). MIT notice retained; see
  THIRD_PARTY_NOTICES.md.

  sprayHash is Dave Hoskins' "Hash without Sine" (hash31; shadertoy.com/view/4djSRW), MIT
  licensed, Copyright (c) 2014 David Hoskins; see THIRD_PARTY_NOTICES.md.
*/

// OceanSpray.cpp — the wind-blown spray's mesh, shaders and inputs (see OceanSpray.h).

#include "OceanSpray.h"

#include <algorithm>
#include <cmath>
#include <cvc/gl/WindGLSL.h>
#include <cvc/ocean/fetch.h>
#include <cvc/ocean/shore.h>

namespace {

const char *const kVsDecHead = R"(//VTK::PositionVC::Dec
uniform highp sampler2D sprayDisp;
uniform vec3 sprayCam; uniform float sprayTime; uniform vec4 sprayWind; uniform vec4 sprayK;
uniform vec4 sprayTiles; uniform vec4 sprayFold; uniform vec4 sprayGrid;
uniform vec4 sprayWeight[64]; uniform vec3 sprayPulse;
uniform float sprayPixel; uniform vec3 spraySun; uniform float sprayCrest;
out vec2 vSprayUv; out float vSprayFade; out float vSprayThin; out float vSprayMu;
// Dave Hoskins' Hash without Sine (hash31), MIT.
vec3 sprayHash(float n) {
  vec3 p = fract(vec3(n) * vec3(0.1031, 0.1030, 0.0973));
  p += dot(p, p.yzx + 33.33);
  return fract((p.xxy + p.yzz) * p.zyx);
}
// The whole wind's pulse (cvc::algo::gust_pulse) `age` seconds ago: its three sines' phases now
// in sprayPulse (oceanSprayPulse).
float sprayPulseAt(float age) {
  return dot(vec3(0.5, 0.3, 0.2), sin(sprayPulse - age * vec3(0.21, 0.21 * 2.37, 0.21 * 5.1)));
}
float sprayWeightAt(int k) {
  vec4 v = sprayWeight[k / 4];
  int c = k - (k / 4) * 4;
  return dot(v, vec4(equal(ivec4(c), ivec4(0, 1, 2, 3))));
}
// How much the waves there may break (the shallows and the fetch; 0 on land), bilinear over the
// grid round the camera.
float sprayWeightAt(vec2 p) {
  vec2 f = clamp((p - sprayGrid.xy) / sprayGrid.z, vec2(0.0), vec2(14.999));
  ivec2 i = ivec2(floor(f));
  vec2 r = f - vec2(i);
  int k = i.y * 16 + i.x;
  return mix(mix(sprayWeightAt(k), sprayWeightAt(k + 1), r.x),
             mix(sprayWeightAt(k + 16), sprayWeightAt(k + 17), r.x), r.y);
}
// Cascade c's Jacobian at world p: the texel nearest p in its tile (the water's wTexel).
float sprayJac(int c, vec2 p, float tileM) {
  float fn = sprayTiles.w;
  vec2 f = floor(p * (fn / tileM) + 0.5);
  f -= fn * floor(f / fn);
  return texelFetch(sprayDisp, ivec2(int(f.x) + c * int(fn), int(f.y)), 0).w;
}
// How hard the sea breaks at p for the water's whitecaps (their bias and ramp, sprayFold.xy): the
// cascades' least Jacobian, each fold faded by `fade` as the water fades it, 0 to 1.
float sprayBreak(vec2 p, float fade) {
  vec3 j = vec3(sprayJac(0, p, sprayTiles.x), sprayJac(1, p, sprayTiles.y),
                sprayTiles.z > 0.0 ? sprayJac(2, p, sprayTiles.z) : 1.0);
  j = vec3(1.0) + fade * (j - vec3(1.0));
  return clamp((sprayFold.x - min(min(j.x, j.y), j.z)) / sprayFold.y, 0.0, 1.0);
}
)";

const char *const kVsImpl = R"(  float id = vertexMC.z;
  vSprayUv = vertexMC.xy;
  vSprayFade = 0.0; vSprayThin = 1.0; vSprayMu = 0.0;
  vec4 pm = vec4(sprayCam, 1.0);
  // past the particles drawn (adaptive quality's share; none while it waits for a wind): off screen
  if (id < sprayK.y - 0.5) {
    vec3 h = sprayHash(id + 0.5);
    // its tile: a quarter, a half or all of the reach (unscaled: adaptive quality's scale moves the
    // fade, not the tiles), world-fixed and wrapped round the camera
    float ring = floor(h.x * 3.0);
    float sR = sprayK.z * (ring < 0.5 ? 0.25 : (ring < 1.5 ? 0.5 : 1.0));
    // its cycles (0.8 to 1.8 s): a whole number of them in the hour sprayTime wraps at, counted
    // modulo that number, so the wrap is seamless
    float cycles = floor(3600.0 / mix(0.8, 1.8, h.y));
    float life = 3600.0 / cycles;
    float su = sprayTime * (cycles / 3600.0) + h.z;
    float cyc = mod(floor(su), cycles);
    float age = fract(su) * life;
    vec3 sg = sprayHash(id * 1.618 + cyc * 7.31 + 11.0);
    vec3 sg2 = sprayHash(id * 2.718 + cyc * 3.17 + 29.0);
    vec3 sg3 = sprayHash(id * 3.141 + cyc * 5.03 + 47.0);
    vec2 p0 = sg.xy * (2.0 * sR);
    p0 -= floor((p0 - sprayCam.xy) / (2.0 * sR) + 0.5) * (2.0 * sR);
    // its wind: the mean wind times the gust where and when it rose (the gusts' pattern then is
    // now's carried back upwind, and the whole wind's pulse then), so one wind for its whole
    // flight; and how much spray that wind tears off (9 to 20 m/s)
    float gust = cvc_wind_gust_at(p0 + sprayWind.xy * age, 0.0, sprayPulseAt(age));
    vec3 W = vec3(sprayWind.xy * gust, 0.0);
    float share = smoothstep(9.0, 20.0, length(sprayWind.xy) * gust);
    // what breaks: the whitecaps' fold at its spawn point now, or at the crest it left, carried on
    // along the waves (sprayWind.zw) at the crests' speed (sprayFold.z), the harder; each faded by
    // the weight there (the shallows and the fetch, 0 on land)
    float fade = sprayWeightAt(p0);
    float brk = max(sprayBreak(p0, fade),
                    sprayBreak(p0 + sprayWind.zw * (sprayFold.z * age), fade));
    float live = smoothstep(sg.z, sg.z + 0.12, brk * share * sprayK.x * 2.0);
    // its flight: out of the crest (the water's level plus the wind sea's crest, as faded, times a
    // draw) with part of the wind's speed and a kick up of its own draw, then dragged toward the
    // wind as it falls, dV/dt = k (W - V) - g z, in closed form
    float size = sg2.x;
    float sk = mix(3.5, 1.2, size);
    vec3 v0 = vec3(W.xy * mix(0.3, 0.7, sg2.y), (2.5 + 6.5 * sg3.x) * mix(0.6, 1.0, sg2.z));
    vec3 sa = W - vec3(0.0, 0.0, 9.80665 / sk);
    float ek = exp(-sk * age);
    vec3 base = vec3(p0, sprayFold.w + sprayCrest * fade * mix(0.4, 1.0, sg3.y));
    vec3 wp = base + sa * age + (v0 - sa) * ((1.0 - ek) / sk);
    vec3 vel = sa + (v0 - sa) * ek;
    // the sea takes it back where it falls to the height it left, or its cycle ends
    float above = smoothstep(-0.5, 0.0, wp.z - base.z);
    float env = smoothstep(0.0, 0.06, age) * (1.0 - smoothstep(0.7 * life, life, age));
    float d = length(p0 - sprayCam.xy);
    float edge = (1.0 - smoothstep(0.7 * sR, sR, d)) *
                 (1.0 - smoothstep(0.7 * sprayK.w, sprayK.w, d));
    // a clump of drops, stretched along its flight as a shutter records it; at least a pixel wide,
    // its opacity scaled down by the widening, so far spray settles into a haze
    float speed = length(vel);
    vec3 dir = vel / max(speed, 1e-4);
    vec3 toEye = sprayCam - wp;
    float dist = length(toEye);
    toEye /= max(dist, 1e-4);
    vec3 side = cross(dir, toEye);
    side = dot(side, side) > 1e-8 ? normalize(side) : vec3(1.0, 0.0, 0.0);
    float trueWide = mix(0.03, 0.15, size);
    float len = max(2.0 * trueWide, speed * 0.04);
    float wide = max(trueWide, sprayPixel * dist * 1.2);
    vSprayThin = clamp(trueWide / wide, 0.08, 1.0);
    vSprayMu = dot(-toEye, spraySun);
    pm = vec4(wp + dir * (vertexMC.y - 0.5) * len + side * vertexMC.x * wide, 1.0);
    vSprayFade = live * env * above * edge;
  }
  vertexVCVSOutput = MCVCMatrix * pm;
  gl_Position = MCDCMatrix * pm;
  if (vSprayFade <= 0.002) gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
)";

const char *const kFsDec = R"(//VTK::Normal::Dec
in vec2 vSprayUv; in float vSprayFade; in float vSprayThin; in float vSprayMu;
uniform vec3 sprayColor;
)";

// A clump's colour: white, soft-edged, brighter looking toward the sun through it (forward
// scattering, after ABYSSAL's 0.55 + 1.9 mu^6 but 0.9 + 0.6 mu^6 here: ABYSSAL tone-maps an HDR
// frame, this draws straight to the screen, where its 0.55 reads as grey), lit by what of the sun
// the deck lets through (SPRAY_SUN) and fogged as the ground is (SPRAY_FOG) once a cloud_sky lists
// it among its receivers.
const char *const kFsLight = R"(  vec2 q = vec2(vSprayUv.x * 2.0, vSprayUv.y * 2.0 - 1.0);
  float body = max(1.0 - dot(q, q), 0.0);
  float scat = 0.9 + 0.6 * pow(max(vSprayMu, 0.0), 6.0);
  vec3 scol = min(sprayColor * (0.6 + 0.4 * SPRAY_SUN) * scat, vec3(1.0));
  gl_FragData[0] = vec4(SPRAY_FOG, vSprayFade * body * vSprayThin * 0.9);
)";

double smooth01(double x) {
  x = std::clamp(x, 0.0, 1.0);
  return x * x * (3.0 - 2.0 * x);
}

} // namespace

cvc::geometry oceanSprayMesh(cvc::app &app, int count) {
  cvc::geometry g(app);
  auto &pts = g.points();
  auto &tris = g.tris();
  count = std::max(count, 1);
  pts.reserve(std::size_t(count) * 4);
  tris.reserve(std::size_t(count) * 2);
  using idx = cvc::geometry::index_t;
  for (int i = 0; i < count; ++i) {
    const idx v = idx(pts.size());
    pts.push_back({-0.5, 0.0, double(i)});
    pts.push_back({0.5, 0.0, double(i)});
    pts.push_back({0.5, 1.0, double(i)});
    pts.push_back({-0.5, 1.0, double(i)});
    tris.push_back({v, idx(v + 1), idx(v + 2)});
    tris.push_back({v, idx(v + 2), idx(v + 3)});
  }
  return g;
}

std::string oceanSprayVsDec() { return std::string(kVsDecHead) + cvc::gl::WindGLSL::glsl(); }
const char *oceanSprayVsImpl() { return kVsImpl; }
const char *oceanSprayFsDec() { return kFsDec; }

std::string oceanSprayLight(bool shaded) {
  std::string s = kFsLight;
  auto swap = [&s](const std::string &from, const std::string &to) {
    s.replace(s.find(from), from.size(), to);
  };
  swap("SPRAY_SUN", shaded ? "cloudShade(vertexVC.xyz)" : "1.0");
  swap("SPRAY_FOG", shaded ? "cloudFog(scol, vertexVC.xyz)" : "scol");
  return s;
}

double oceanSprayAmount(double wind_mps) { return smooth01((wind_mps - 9.0) / (20.0 - 9.0)); }

double oceanSprayGustMax(double gustiness) {
  // cvc::algo::wind_gust: 1 + g (2.4 (n - 0.5) + 0.5 pulse), n in [0, 1), |pulse| <= 1
  return 1.0 + std::max(gustiness, 0.0) * (1.2 + 0.5);
}

std::array<float, 3> oceanSprayPulse(double t) {
  // cvc::algo::gust_pulse: 0.5 sin(phi) + 0.3 sin(2.37 phi + 1.1) + 0.2 sin(5.1 phi), phi = 0.21 t
  constexpr double kTwoPi = 6.283185307179586;
  const double phi = 0.21 * t;
  const auto wrap = [](double a) { return float(a - kTwoPi * std::floor(a / kTwoPi)); };
  return {wrap(phi), wrap(2.37 * phi + 1.1), wrap(5.1 * phi)};
}

void oceanSprayWeights(const cvc::ocean::shore_grid &g, const cvc::ocean::fetch_field *fetch,
                       double full_fetch_m, double fade_m, double x0, double y0, double cell,
                       std::array<float, kSprayGrid * kSprayGrid> &out) {
  const auto depth = [&g](double x, double y) -> double {
    if (g.rows < 1 || g.cols < 1 || g.depth.size() < g.count())
      return -1.0;
    const double fc = std::clamp((x - g.min_x) / g.cell, 0.0, double(g.cols - 1));
    const double fr = std::clamp((y - g.min_y) / g.cell, 0.0, double(g.rows - 1));
    const int c0 = std::min(int(fc), std::max(g.cols - 2, 0)),
              r0 = std::min(int(fr), std::max(g.rows - 2, 0));
    const int c1 = std::min(c0 + 1, g.cols - 1), r1 = std::min(r0 + 1, g.rows - 1);
    const double tc = fc - c0, tr = fr - r0;
    const auto d = [&g](int r, int c) {
      const float v = g.depth[std::size_t(r) * std::size_t(g.cols) + std::size_t(c)];
      return std::isfinite(v) ? double(v) : -1.0; // NaN: dry
    };
    return (d(r0, c0) * (1.0 - tc) + d(r0, c1) * tc) * (1.0 - tr) +
           (d(r1, c0) * (1.0 - tc) + d(r1, c1) * tc) * tr;
  };
  for (int j = 0; j < kSprayGrid; ++j)
    for (int i = 0; i < kSprayGrid; ++i) {
      const double x = x0 + i * cell, y = y0 + j * cell;
      // the shallows (as the water fades its waves toward the shore) and the land (0)
      double w = smooth01((depth(x, y) / std::max(fade_m, 0.1) - 0.2) / 0.8);
      // the fetch: the cascade's height grows as sqrt(F / full) until it is fully grown (fetch.h)
      if (w > 0.0 && fetch && !fetch->empty() && full_fetch_m > 0.0)
        w *= std::clamp(std::sqrt(std::max(fetch->sample(x, y), 0.0) / full_fetch_m), 0.0, 1.0);
      out[std::size_t(j) * kSprayGrid + std::size_t(i)] = float(w);
    }
}
