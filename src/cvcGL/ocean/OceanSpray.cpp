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
*/

// OceanSpray.cpp — the wind-blown spray's mesh, shaders and inputs (see OceanSpray.h).

#include "OceanSpray.h"

#include <algorithm>
#include <cmath>
#include <cvc/ocean/shore.h>

namespace {

const char *const kVsDec = R"(//VTK::PositionVC::Dec
uniform highp sampler2D sprayDisp;
uniform vec3 sprayCam; uniform float sprayTime; uniform vec4 sprayWind; uniform vec4 sprayK;
uniform vec4 sprayTiles; uniform vec4 sprayFold; uniform vec4 sprayGrid; uniform vec4 sprayDepth[64];
uniform float sprayPixel; uniform vec3 spraySun;
out vec2 vSprayUv; out float vSprayFade; out float vSprayThin; out float vSprayMu;
vec3 sprayHash(float n) {
  vec3 p = fract(vec3(n) * vec3(0.1031, 0.1030, 0.0973));
  p += dot(p, p.yzx + 33.33);
  return fract((p.xxy + p.yzz) * p.zyx);
}
float sprayDepthAt(int k) {
  vec4 v = sprayDepth[k / 4];
  int c = k - (k / 4) * 4;
  return dot(v, vec4(equal(ivec4(c), ivec4(0, 1, 2, 3))));
}
// The still water's depth at p, bilinear over the grid round the camera.
float sprayDepthAt(vec2 p) {
  vec2 f = clamp((p - sprayGrid.xy) / sprayGrid.z, vec2(0.0), vec2(14.999));
  ivec2 i = ivec2(floor(f));
  vec2 r = f - vec2(i);
  int k = i.y * 16 + i.x;
  return mix(mix(sprayDepthAt(k), sprayDepthAt(k + 1), r.x),
             mix(sprayDepthAt(k + 16), sprayDepthAt(k + 17), r.x), r.y);
}
// Cascade c of the displacement atlas at world p: the texel nearest p in its tile (the water's
// wTexel): .y the height, .w the Jacobian.
vec4 sprayTexel(int c, vec2 p, float tileM) {
  float fn = sprayTiles.w;
  vec2 f = floor(p * (fn / tileM) + 0.5);
  f -= fn * floor(f / fn);
  return texelFetch(sprayDisp, ivec2(int(f.x) + c * int(fn), int(f.y)), 0);
}
// The cascades at p, each fold faded by `fade` as the water fades it in the shallows: (the height,
// the least Jacobian) as the water's wSum gives them.
vec2 sprayWaves(vec2 p, float fade) {
  vec4 a = sprayTexel(0, p, sprayTiles.x), b = sprayTexel(1, p, sprayTiles.y);
  vec4 c = sprayTiles.z > 0.0 ? sprayTexel(2, p, sprayTiles.z) : vec4(0.0, 0.0, 0.0, 1.0);
  vec3 j = vec3(1.0) + fade * (vec3(a.w, b.w, c.w) - vec3(1.0));
  return vec2(fade * (a.y + b.y + c.y), min(min(j.x, j.y), j.z));
}
// How hard the sea breaks for a Jacobian j: the water's whitecaps (the bias and ramp it draws them
// with, sprayFold.xy), 0 to 1.
float sprayBreak(float j) { return clamp((sprayFold.x - j) / sprayFold.y, 0.0, 1.0); }
)";

const char *const kVsImpl = R"(  float id = vertexMC.z;
  vec3 h = sprayHash(id + 0.5);
  // its tile: a quarter, a half or all of the reach, world-fixed and wrapped round the camera
  float ring = floor(h.x * 3.0);
  float sR = sprayK.z * (ring < 0.5 ? 0.25 : (ring < 1.5 ? 0.5 : 1.0));
  float life = mix(0.8, 1.8, h.y);
  float su = sprayTime / life + h.z;
  float cyc = floor(su);
  float age = (su - cyc) * life;
  vec3 sg = sprayHash(id * 1.618 + cyc * 7.31 + 11.0);
  vec3 sg2 = sprayHash(id * 2.718 + cyc * 3.17 + 29.0);
  vec2 p0 = sg.xy * (2.0 * sR);
  p0 -= floor((p0 - sprayCam.xy) / (2.0 * sR) + 0.5) * (2.0 * sR);
  // what breaks there: the whitecaps' fold at its spawn point now, or, carried on with the crests
  // since it left (along the waves at sprayFold.z m/s), at the crest it left; faded where the still
  // water is shallow, as the water fades its waves
  float fade = smoothstep(0.2, 1.0, sprayDepthAt(p0) / max(sprayGrid.w, 0.1));
  vec2 here = sprayWaves(p0, fade);
  vec2 crest = sprayWaves(p0 + sprayWind.zw * (sprayFold.z * age), fade);
  float brk = max(sprayBreak(here.y), sprayBreak(crest.y));
  float live = smoothstep(sg.z, sg.z + 0.12, brk * sprayK.x * 2.0) * step(id, sprayK.y - 0.5);
  // its flight: out of the crest with part of the wind's speed and a kick up, then dragged toward
  // the wind as it falls, dV/dt = k (W - V) - g z, in closed form
  float size = sg2.x;
  float sk = mix(3.5, 1.2, size);
  vec3 W = vec3(sprayWind.xy, 0.0);
  vec3 v0 = vec3(W.xy * mix(0.3, 0.7, sg2.y), (2.5 + 6.5 * brk) * mix(0.6, 1.0, sg2.z));
  vec3 sa = W - vec3(0.0, 0.0, 9.80665 / sk);
  float ek = exp(-sk * age);
  vec3 base = vec3(p0, sprayFold.w + sprayK.w * here.x);
  vec3 wp = base + sa * age + (v0 - sa) * ((1.0 - ek) / sk);
  vec3 vel = sa + (v0 - sa) * ek;
  // the sea takes it back where it falls to the height it left, or its cycle ends
  float above = smoothstep(-0.5, 0.0, wp.z - base.z);
  float env = smoothstep(0.0, 0.06, age) * (1.0 - smoothstep(0.7 * life, life, age));
  float edge = 1.0 - smoothstep(0.7 * sR, sR, length(p0 - sprayCam.xy));
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
  vec4 pm = vec4(wp + dir * (vertexMC.y - 0.5) * len + side * vertexMC.x * wide, 1.0);
  vSprayFade = live * env * above * edge;
  vSprayUv = vertexMC.xy;
  vertexVCVSOutput = MCVCMatrix * pm;
  gl_Position = MCDCMatrix * pm;
  if (vSprayFade <= 0.002) gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
)";

const char *const kFsDec = R"(//VTK::Normal::Dec
in vec2 vSprayUv; in float vSprayFade; in float vSprayThin; in float vSprayMu;
uniform vec3 sprayColor;
)";

// A clump's colour: white, soft-edged, brighter looking toward the sun through it (forward
// scattering after ABYSSAL's 0.55 + 1.9 mu^6, raised here: ABYSSAL tone-maps an HDR frame, this
// draws straight to the screen, where its 0.55 reads as grey), lit by what of the sun the deck
// lets through (SPRAY_SUN) and fogged as the ground is (SPRAY_FOG) once a cloud_sky lists it among
// its receivers.
const char *const kFsLight = R"(  vec2 q = vec2(vSprayUv.x * 2.0, vSprayUv.y * 2.0 - 1.0);
  float body = max(1.0 - dot(q, q), 0.0);
  float scat = 0.9 + 0.6 * pow(max(vSprayMu, 0.0), 6.0);
  vec3 scol = min(sprayColor * (0.6 + 0.4 * SPRAY_SUN) * scat, vec3(1.0));
  gl_FragData[0] = vec4(SPRAY_FOG, vSprayFade * body * vSprayThin * 0.9);
)";

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

const char *oceanSprayVsDec() { return kVsDec; }
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

double oceanSprayAmount(double wind_mps) {
  const double x = std::clamp((wind_mps - 9.0) / (20.0 - 9.0), 0.0, 1.0);
  return x * x * (3.0 - 2.0 * x);
}

void oceanSprayDepths(const cvc::ocean::shore_grid &g, double x0, double y0, double cell,
                      std::array<float, kSprayGrid * kSprayGrid> &out) {
  const auto at = [&g](double x, double y) -> float {
    if (g.rows < 1 || g.cols < 1 || g.depth.size() < g.count())
      return 0.0f;
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
    return float((d(r0, c0) * (1.0 - tc) + d(r0, c1) * tc) * (1.0 - tr) +
                 (d(r1, c0) * (1.0 - tc) + d(r1, c1) * tc) * tr);
  };
  for (int j = 0; j < kSprayGrid; ++j)
    for (int i = 0; i < kSprayGrid; ++i)
      out[std::size_t(j) * kSprayGrid + std::size_t(i)] = at(x0 + i * cell, y0 + j * cell);
}
