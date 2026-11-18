/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  ---------------------------------------------------------------------------
  The spray's flight and look follow ABYSSAL's spray particles
  (github.com/Token-Gremlin/natural-disasters, src/weather/Precipitation.js at 849ff7f4: drag
  toward the wind under gravity, an upward kick out of the breaking crest, billboards stretched by
  speed, forward scattering after its 0.55 + 1.9 mu^6, 0.9 + 0.6 mu^6 here), MIT licensed,
  Copyright (c) 2026 Davi (Token-Gremlin). MIT notice retained; see THIRD_PARTY_NOTICES.md. ABYSSAL
  keeps its particles in ping-pong float targets; these are a closed form of time, with no state at
  all. The shader's hash is Dave Hoskins' "Hash without Sine" (MIT, Copyright (c) 2014 David
  Hoskins).
*/

#ifndef __CVC_GL_OCEAN_OCEANSPRAY_H__
#define __CVC_GL_OCEAN_OCEANSPRAY_H__

#include <array>
#include <cvc/geometry/geometry.h>
#include <string>

namespace cvc {
class app;
namespace ocean {
struct shore_grid;
struct fetch_field;
} // namespace ocean
} // namespace cvc

// OceanSpray — the wind-blown spray off the breaking crests (OCEAN-AND-WEATHER-ROADMAP.md
// section 10.5): drops torn off whitecaps in a strong wind, flying downwind and falling back.
//
// Drawing only, and stateless: each quad of the mesh is a particle (its corner in vertexMC.xy, its
// index in vertexMC.z) whose whole life is a function of the time, worked out in the vertex
// shader. A particle lives cycle after cycle (0.8 to 1.8 s each, its own: a whole number of cycles
// in the hour the time wraps at, so the wrap is seamless); each cycle it takes a new spawn point, a
// hash of its index and the cycle, on a tile 2 R wide that is fixed in the world and wrapped round
// the camera (so the camera moving does not drag the spray with it). A third of the particles tile
// a quarter of the reach R, a third half of it and a third all of it, so the spray is densest near
// the camera. The tiles use the reach unscaled; adaptive quality's scale moves only where the spray
// fades out, so a new scale reshuffles nothing.
//
// It rises where the sea breaks as the water draws its whitecaps: the cascades' fold under its
// spawn point (the least Jacobian of the long, middle and short cascades, against the whitecaps'
// bias and ramp), or at the crest it left, carried on along the waves at their crests' speed since
// (the fold moves with its crest while the drop flies), the harder; each fold faded by the weight
// grid below (the shallows and the fetch: calm water in the lee throws none); times how much its
// wind raises (oceanSprayAmount). Its wind is the mean wind times the gust where and when it rose
// (cvc_wind_gust_at, WindGLSL.h, its fine scale left out: the gusts' pattern of now carried back
// upwind by its age, and the whole wind's pulse of then, from sprayPulse), so one wind for its
// whole flight. A particle shows when that beats its cycle's draw, so the spray thickens with the
// breaking and the wind. It leaves the crest (the water's level plus the wind sea's crest, half
// its significant height, faded by the weight below, times a draw of its own) with part of its
// wind's speed and an upward kick of its own draw (2.5 to 9 m/s), and flies under gravity with
// drag toward the wind, dV/dt = k (W - V) - g z, in closed form; it ends when it falls back to the
// height it left or its cycle ends. Nothing in its flight changes while it flies. Particles past
// the count drawn (sprayK.y) are put off screen first thing, so a mesh kept in the renderer with
// a count of 0 costs a few instructions a vertex.
//
// Not the persistent foam's spray channel (OceanFoam's .a), which section 10.2 kept for this: the
// persistent foam holds only the long and middle cascades' folds, and in a 24 m/s wind on
// terrain_lab's lake it read 0 under almost every particle while the whitecaps (mostly the short
// cascade's) covered the water.
//
// Its inputs are uniforms and one texture it shares, allocating none: the cascades' displacement
// atlas the water already reads (OceanCascades::displacement()). Where the waves may break (the
// land kept out, the shallows and the fetch) comes as a weight over a 16 x 16 grid round the
// camera, in a uniform array (oceanSprayWeights): coarse, a cell of 2 R / 13 (24.6 m at R = 160 m),
// bilinear, so near a shore the spray's edge is out by up to a cell. The mesh is drawn as a
// GeometryNode in the volume layer like the rain's streaks, blended, depth-tested against the
// water and the ground.
//
// Uniforms (vertex): sprayDisp (the atlas), sprayCam, sprayTime (s, wrapping each hour), sprayWind
// (the mean wind's x, y in m/s, and the waves' unit direction), sprayK (strength, count, the tiles'
// reach m, the fade's reach m), sprayTiles (the three tiles' sizes m, 0 for a cascade not drawn,
// and the atlas's n), sprayFold (the whitecaps' bias and ramp, the crests' speed m/s, the water's
// level m), sprayCrest (m over the level), sprayGrid (x0, y0, cell m), sprayWeight[64] (16 x 16
// weights, four a vec4, row by row), sprayPulse (oceanSprayPulse), sprayPixel (a pixel's angle at
// distance 1), spraySun (the way to the sun, world), and WindGLSL's uWindGust, uWindDriftL and
// uWindDriftF. Fragment: sprayColor.

constexpr int kSprayGrid = 16;                             // the weight grid's samples a side
constexpr int kSprayWeights = kSprayGrid * kSprayGrid / 4; // its vec4s

// The mesh: `count` quads, each a particle (its corner in xy, its index in z).
cvc::geometry oceanSprayMesh(cvc::app &app, int count);

// The GLSL. The light writes the final colour; `shaded` uses a cloud_sky's cloudShade and cloudFog
// (the sky declares them once it lists the mesh among its receivers).
std::string oceanSprayVsDec();
const char *oceanSprayVsImpl();
const char *oceanSprayFsDec();
std::string oceanSprayLight(bool shaded);

// How much spray a wind (U10, m/s) tears off the crests: none below 9 m/s, all of it from 20
// (Beaufort 5, where spray begins, to 8, where spindrift streaks the sea), smoothstep between.
double oceanSprayAmount(double wind_mps);
// The most a gust of cvc::algo::wind_gust can raise the mean wind by, at this gustiness (a factor).
double oceanSprayGustMax(double gustiness);
// The three sines' phases of the whole wind's pulse (cvc::algo::gust_pulse) at clock time t,
// reduced modulo 2 pi in double precision: sprayPulse, from which the shader works out the pulse
// when each particle rose.
std::array<float, 3> oceanSprayPulse(double t);

// Where the waves may break over the grid of kSprayGrid x kSprayGrid points `cell` metres apart
// from (x0, y0), four a vec4 in `out` (point (i, j) at index j kSprayGrid + i): 0 on land, rising
// over the shallows as the water fades its waves (the still water's depth from the shore grid,
// against `fade_m`), times the fetch's share sqrt(F / full_fetch_m), at most 1, where the fetch map
// is given (the lee on).
void oceanSprayWeights(const cvc::ocean::shore_grid &g, const cvc::ocean::fetch_field *fetch,
                       double full_fetch_m, double fade_m, double x0, double y0, double cell,
                       std::array<float, kSprayGrid * kSprayGrid> &out);

#endif // __CVC_GL_OCEAN_OCEANSPRAY_H__
