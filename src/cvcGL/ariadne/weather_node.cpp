/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

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

  ---------------------------------------------------------------------------
  The rain's streaks (RAIN_VS_IMPL, RAIN_FS_LIGHT) port ABYSSAL's Rain
  (github.com/Token-Gremlin/natural-disasters, src/weather/Precipitation.js at 849ff7f4),
  MIT licensed, Copyright (c) 2026 Davi (Token-Gremlin). MIT notice retained; see
  THIRD_PARTY_NOTICES.md.

  The lightning bolts (growBolt, boltOf, boltMesh, BOLT_FS_LIGHT) port ABYSSAL's Lightning
  (the same repository, src/weather/Lightning.js at 849ff7f4), under the same MIT licence and
  copyright; see THIRD_PARTY_NOTICES.md.
*/

// weather_node.cpp — the Ariadne `weather` node (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md §9 F2,
// F3, F5, F6, §11.2, O18, O19, O20): the scene's weather model, the storms it puts in the sky,
// their rain and lightning, and the wet ground's friction.
//
//   - { id: weather, type: weather, ground: world, sky: sky }
//
// It holds a cvc::world::weather (storm cells on a hashed schedule, and the storms people start)
// over the ground's extent, and every frame evaluates it at the world clock's time: a pure function
// of its props, the time and the events in `weather.events`, so every client holding those keys
// sees the same storms (decision D5). What it does with them:
//   - the cloud_sky named by `sky:` (its key root; `sky: ""` drives none) gets the storms as discs
//     (sky.storms): the cells under a storm close up, thicken and darken; its deck rises to
//     `storm_top` while a storm is within `reach` of the ground (sky.top, in DECK_STEPS steps, so
//     the cumulonimbus maps are built a few times a storm, off the frame, and only while one is
//     about), and its darkness to `darkness` (sky.darkness);
//   - it publishes the weather: weather.state (clear, building, storm, clearing), .storminess,
//     .cells (how many), .storms (the cells as "x y r intensity" discs, world metres), .rain (the
//     rain rate at the ground's centre, mm/h) and .deck (how much of a storm the deck shows).
// The rain (O19): it hands the sky the recent storms' tracks (sky.rain) and the rain's constants
// (sky.rain_k), from which the sky's shaders draw the wet ground, the rain shafts, the rings on the
// water and the fog; and it draws the streaks near the camera itself (its mesh `<id>_rain`, which
// the sky lists among its receivers). weather.streaks / .rings / .fog / .wet scale each, and
// weather.rain_quality the streaks' count.
// The lightning (O20): the strikes the weather schedules (cvc::world::weather::strikes), each drawn
// as a bolt while it shows (its mesh `<id>_bolt`, listed among the sky's receivers like the rain's)
// and handed to the sky as a flash (sky.flash) that lights the receivers, the water and the far
// clouds; weather.lightning scales it and weather.flash publishes it. The wet ground's friction
// (O20) is physics, so the scene's world sets it, inside the physics step on the world clock's
// ticks, with or without a renderer (cvc::ariadne::WeatherFriction, scene_weather.h:
// weather.friction, weather.wetness); this node reads its props the same way
// (scene_weather_params), so the friction follows the storms it draws. The storm wind (F4): each
// cell's gust front (cvc::world::weather::gust_front) is in the scene's wind, which the scene's
// world owns too (StormWindFlow in its physics, wind.front for the water and the trees;
// scene_wind.h); this node slants its rain by the wind at the camera, the front there in it.
//
// The steering wind the cells ride is the scene's one wind (R5), read through steeringWind(): with
// a scene wind (O14's wind.speed / wind.dir) its heading at its speed at the deck's base, else the
// cloud deck's own, sky.wind_az at the deck's drift speed, so the storms move with the clouds
// drawn. A change in it is logged as a
// steer event at the time it settles (it has held for HOLD_FRAMES frames), so the cells' tracks
// stay a function of the events; so is a change of weather.time_scale (a rate event: weather
// seconds per world second). weather.request starts a storm ("upwind": born upwind, timed to
// cross the ground in its prime; "overhead": in its prime over the ground's centre now) or clears
// the sky ("calm"), each an event at the time asked, and is emptied once read.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_weather.h> // scene_weather_params: the storms' props, as the world reads them
#include <cvc/ariadne/scene_wind.h>    // the scene's wind: the steering, and the rain's slant
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h>
#include <cvc/core/world_clock.h>
#include <cvc/geometry/height_field.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/lsystem_nodes.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/world_nodes.h>
#include <cvc/world/weather.h>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <vtkCamera.h>
#include <vtkRenderer.h>

namespace cvc {
namespace gl {
namespace ariadne {
namespace {

namespace ari = cvc::ariadne;

constexpr int HOLD_FRAMES = 10;     // a wind or rate change is logged once it has held this long
constexpr int DECK_STEPS = 4;       // the deck's top moves in this many steps to the storm's
constexpr std::size_t COMPACT = 24; // the events are compacted past this many
constexpr double STORM_TOP = 3200.0, STORM_DARK = 0.6, REACH = 4000.0;
// The rain (O19): a downpour is RAIN_FULL mm/h (full streaks); the ground is soaked by SOAK weather
// seconds of rain and dries by 1/e in DRY; a track counts while it is younger than 4 DRY; the
// streaks are at most RAIN_COUNT, in a box RAIN_BOX metres around the camera and RAIN_HIGH high.
constexpr double RAIN_FULL = 25.0, RAIN_BOX = 80.0, RAIN_HIGH = 52.0;
constexpr double SOAK = ari::kWeatherSoak,
                 DRY = ari::kWeatherDry; // the world's friction reads them too
constexpr int RAIN_COUNT = 12000, MAX_TRACKS = 4;
const char *const RAIN_INPUTS = "weather_rain"; // the streaks' own shader inputs

// The rain near the camera: streaks, entirely in the vertex shader, a port of ABYSSAL's Rain
// (Token-Gremlin/natural-disasters, src/weather/Precipitation.js at 849ff7f4, MIT; z up here).
// Each quad of the mesh is a drop: its corner in vertexMC.xy, its index in vertexMC.z. A drop owns
// a slot in a box that travels with the camera (pushed half its reach ahead along the view), its
// place a hash of its index; it falls at its own terminal velocity (4 to 9 m/s), the wind carries
// it sideways, and it wraps the box's height, so the field is a closed form of time: no state, no
// texture, nothing on the CPU. A drop is drawn as the streak a shutter records (rainStreak seconds
// of its fall), at least a pixel wide with its opacity scaled down by the widening, so far rain
// settles into a veil. Only the share rainAmount of the slots holds a drop, and none above the
// cloud base (rainTop) or below the ground (rainGround).
const char *const RAIN_VS_DEC = R"(//VTK::PositionVC::Dec
uniform vec3 rainCam; uniform vec3 rainFwd; uniform float rainTime; uniform float rainAmount;
uniform vec4 rainWind; uniform vec4 rainBox; uniform float rainCount; uniform float rainStreak;
uniform float rainPixel; uniform float rainGround; uniform float rainTop;
out vec2 vRainUv; out float vRainFade; out float vRainThin;
vec3 rainHash(float n) {
  vec3 p = fract(vec3(n) * vec3(0.1031, 0.1030, 0.0973));
  p += dot(p, p.yzx + 33.33);
  return fract((p.xxy + p.yzz) * p.zyx);
}
)";
const char *const RAIN_VS_IMPL = R"(  float id = vertexMC.z;
  vec3 h = rainHash(id + 0.5);
  float live = step(h.x, clamp(rainAmount * 1.15, 0.0, 1.0)) * step(id, rainCount - 0.5);
  vec3 anchor = rainCam + vec3(rainFwd.xy, 0.0) * rainBox.x * 0.45;
  anchor.z = rainCam.z;
  float size = mix(0.35, 1.0, h.y);
  float vz = mix(4.2, 9.4, size) * mix(0.85, 1.15, h.z);
  float fall = mod(h.z * rainBox.y + rainTime * vz, rainBox.y);
  vec3 wp;
  wp.z = anchor.z + rainBox.y * 0.55 - fall;
  float ang = h.x * 6.2831853, rad = rainBox.x * pow(h.y, 1.7);
  vec2 drift = rainWind.xy * (fall / max(vz, 0.1));
  wp.xy = anchor.xy + vec2(cos(ang), sin(ang)) * rad + drift;
  wp.xy -= floor((wp.xy - anchor.xy) / (2.0 * rainBox.x) + 0.5) * (2.0 * rainBox.x);
  vec3 vel = vec3(rainWind.xy, -vz);
  float speed = length(vel);
  vec3 dir = vel / max(speed, 1e-4);
  vec3 toEye = rainCam - wp;
  float dist = length(toEye);
  toEye /= max(dist, 1e-4);
  vec3 side = normalize(cross(dir, toEye));
  float len = clamp(speed * rainStreak, 0.30, 3.2) * mix(0.7, 1.3, size);
  float trueWide = mix(0.006, 0.016, size);
  float wide = max(trueWide, rainPixel * dist * 1.15);
  vRainThin = clamp(trueWide / wide, 0.10, 1.0);
  vec4 pm = vec4(wp + dir * (vertexMC.y - 0.5) * len + side * vertexMC.x * wide, 1.0);
  float edge = 1.0 - smoothstep(rainBox.x * 0.72, rainBox.x, length(wp.xy - anchor.xy));
  float above = smoothstep(-0.5, 2.0, wp.z - rainGround) * (1.0 - smoothstep(rainTop - 30.0, rainTop, wp.z));
  vRainFade = live * edge * above * clamp(rainAmount * 1.6, 0.0, 1.0);
  vRainUv = vertexMC.xy;
  vertexVCVSOutput = MCVCMatrix * pm;
  gl_Position = MCDCMatrix * pm;
  if (vRainFade <= 0.001) gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
)";
const char *const RAIN_FS_DEC = R"(//VTK::Normal::Dec
in vec2 vRainUv; in float vRainFade; in float vRainThin;
uniform vec3 rainColor;
)";
// A streak's colour: a bright core falling off to its edges (a water cylinder), lit by what of the
// sun the deck lets through (RAIN_SUN: cloudShade() once the sky lists the streaks among its
// receivers) and fogged as the ground is (RAIN_FOG).
const char *const RAIN_FS_LIGHT = R"(  float rr = vRainUv.x * 2.0;
  float body = sqrt(max(1.0 - rr * rr, 0.0));
  float ends = smoothstep(0.0, 0.12, vRainUv.y) * (1.0 - smoothstep(0.88, 1.0, vRainUv.y));
  vec3 rcol = rainColor * (0.55 + 0.45 * RAIN_SUN) * (0.9 + 0.35 * body);
  gl_FragData[0] = vec4(RAIN_FOG, vRainFade * body * ends * vRainThin * 0.7);
)";

std::string rain_light(bool shaded) {
  std::string s = RAIN_FS_LIGHT;
  auto swap = [&s](const std::string &from, const std::string &to) {
    s.replace(s.find(from), from.size(), to);
  };
  swap("RAIN_SUN", shaded ? "cloudShade(vertexVC.xyz)" : "1.0");
  swap("RAIN_FOG", shaded ? "cloudFog(rcol, vertexVC.xyz)" : "rcol");
  return s;
}

// ── Lightning (O20, F5) ──────────────────────────────────────────────────────────────────────
// A strike's bolt, a port of ABYSSAL's Lightning (src/weather/Lightning.js at 849ff7f4, MIT): a
// polyline from the cloud base to the ground, its midpoints displaced across the channel (by 0.42
// of its height at first, 0.55 less each level, to depth 5 or 40 m), forking into shorter dead
// ends that fade (0.42 a level, to depth 3), with two to four crawlers spreading in the cloud. An
// in-cloud strike is its crawlers alone. Seeded by the strike's id (a splitmix64 stream, not
// Math.random), so every client grows the same bolt. Drawn as camera-facing ribbons, a hot core
// and a wide glow, three vertices across so the colour's red runs 0 at an edge to 1 on the line.
struct BoltSeg {
  double a[3], b[3], fade;
};

struct BoltRng {
  std::uint64_t s;
  double next() {
    s += 0x9E3779B97F4A7C15ull;
    std::uint64_t z = s;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    z ^= z >> 31;
    return double(z >> 11) * (1.0 / 9007199254740992.0);
  }
};

constexpr std::size_t BOLT_SEGMENTS = 2048;

void growBolt(std::vector<BoltSeg> &out, BoltRng &g, const double a[3], const double b[3],
              double jitter, int depth, double fade) {
  if (out.size() > BOLT_SEGMENTS - 8)
    return;
  const double d[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
  const double len = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
  if (depth > 5 || len < 40.0) {
    out.push_back({{a[0], a[1], a[2]}, {b[0], b[1], b[2]}, fade});
    return;
  }
  const double dir[3] = {d[0] / len, d[1] / len, d[2] / len};
  // across the channel: horizontal and perpendicular to it, and the third axis
  double perp[3] = {-dir[1], dir[0], 0.0};
  double pl = std::sqrt(perp[0] * perp[0] + perp[1] * perp[1]);
  if (pl < 1e-6) {
    perp[0] = 1.0;
    perp[1] = 0.0;
    pl = 1.0;
  }
  for (double &v : perp)
    v /= pl;
  const double up[3] = {dir[1] * perp[2] - dir[2] * perp[1], dir[2] * perp[0] - dir[0] * perp[2],
                        dir[0] * perp[1] - dir[1] * perp[0]};
  const double jp = (g.next() - 0.5) * jitter, ju = (g.next() - 0.5) * jitter;
  double mid[3];
  for (int k = 0; k < 3; ++k)
    mid[k] = 0.5 * (a[k] + b[k]) + perp[k] * jp + up[k] * ju;
  growBolt(out, g, a, mid, jitter * 0.55, depth + 1, fade);
  growBolt(out, g, mid, b, jitter * 0.55, depth + 1, fade);
  if (depth < 3 && g.next() < 0.42) { // a fork: a shorter dead end that fades out
    const double fl = len * (0.35 + 0.45 * g.next());
    const double fp = (g.next() - 0.5) * 1.5, fu = (g.next() - 0.5) * 1.5;
    double bd[3], bn = 0.0;
    for (int k = 0; k < 3; ++k) {
      bd[k] = dir[k] + perp[k] * fp + up[k] * fu;
      bn += bd[k] * bd[k];
    }
    bn = std::sqrt(std::max(bn, 1e-12));
    double tip[3];
    for (int k = 0; k < 3; ++k)
      tip[k] = mid[k] + bd[k] / bn * fl;
    growBolt(out, g, mid, tip, jitter * 0.5, depth + 2, fade * 0.5);
  }
}

// A strike's segments: from `top` (the cloud base) down to the ground under it (ground: its height
// there), and the crawlers in the cloud.
std::vector<BoltSeg> boltOf(const cvc::world::lightning_strike &k, double top, double ground) {
  std::vector<BoltSeg> out;
  BoltRng g{k.id};
  const double height = std::max(50.0, top - ground);
  if (k.ground) {
    const double a[3] = {k.x, k.y, top};
    const double b[3] = {k.x + (g.next() - 0.5) * height * 0.5,
                         k.y + (g.next() - 0.5) * height * 0.5, ground};
    growBolt(out, g, a, b, height * 0.42, 0, 1.0);
  }
  const int crawl = 2 + static_cast<int>(g.next() * 3.0);
  for (int i = 0; i < crawl; ++i) {
    const double ang = 2.0 * 3.14159265358979323846 * g.next();
    const double len = height * (0.5 + g.next());
    const double a[3] = {k.x, k.y, top + height * 0.25 * g.next()};
    const double b[3] = {k.x + std::cos(ang) * len, k.y + std::sin(ang) * len,
                         top + height * (0.05 + 0.3 * g.next())};
    growBolt(out, g, a, b, len * 0.3, 2, 0.45);
  }
  return out;
}

// The bolt's mesh, its ribbons facing `eye`: per segment a core and a glow, each three vertices
// across at either end and tapering to nothing a half-width beyond them, so the segments' ends
// blend where they meet (colour red: 0 at the edges and the tips, 1 on the line; green: 0 core,
// 1 glow; blue: the segment's fade). Widths are ABYSSAL's (2.2 m core, 26 m glow across, wider
// with distance).
cvc::geometry boltMesh(cvc::app &app, const std::vector<BoltSeg> &segs, const double eye[3]) {
  cvc::geometry g(app);
  auto &pts = g.points();
  auto &cols = g.colors();
  auto &tris = g.tris();
  using idx = cvc::geometry::index_t;
  for (const BoltSeg &s : segs) {
    double d[3], m[3], e[3];
    for (int k = 0; k < 3; ++k) {
      d[k] = s.b[k] - s.a[k];
      m[k] = 0.5 * (s.a[k] + s.b[k]);
      e[k] = eye[k] - m[k];
    }
    double side[3] = {d[1] * e[2] - d[2] * e[1], d[2] * e[0] - d[0] * e[2],
                      d[0] * e[1] - d[1] * e[0]};
    const double sl = std::sqrt(side[0] * side[0] + side[1] * side[1] + side[2] * side[2]);
    const double dl = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    if (sl < 1e-9 || dl < 1e-9)
      continue;
    const double dist = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
    const double widen = 1.0 + dist * 0.002;
    for (int layer = 0; layer < 2; ++layer) {
      const double w = (layer == 0 ? 1.1 : 13.0) * s.fade * widen; // half-widths
      const idx v = idx(pts.size());
      // Four rows along: the tip before a, a, b, the tip past b.
      for (int row = 0; row < 4; ++row) {
        const double *end = row < 2 ? s.a : s.b;
        const double out = row == 0 ? -w / dl : row == 3 ? w / dl : 0.0;
        const bool tip = row == 0 || row == 3;
        for (int c = -1; c <= 1; ++c) {
          pts.push_back({end[0] + d[0] * out + side[0] / sl * w * c,
                         end[1] + d[1] * out + side[1] / sl * w * c,
                         end[2] + d[2] * out + side[2] / sl * w * c});
          cols.push_back({c == 0 && !tip ? 1.0 : 0.0, double(layer), s.fade});
        }
      }
      for (int row = 0; row < 3; ++row)
        for (int c = 0; c < 2; ++c) {
          const idx q = idx(v + 3 * row + c);
          tris.push_back({q, idx(q + 1), idx(q + 3)});
          tris.push_back({idx(q + 1), idx(q + 4), idx(q + 3)});
        }
    }
  }
  if (pts.empty()) { // no bolt yet: one degenerate triangle, so the mesh has something to hold
    for (int k = 0; k < 3; ++k) {
      pts.push_back({0.0, 0.0, 0.0});
      cols.push_back({0.0, 0.0, 0.0});
    }
    tris.push_back({0, 1, 2});
  }
  return g;
}

// The bolt's light: emissive, from the vertex colour VTK has put in diffuseColor (red: across, 1 on
// the line; green: the glow layer; blue: the fade) and boltIntensity; a hot near-white core and a
// blue-white halo, as ABYSSAL's; a little fogged once the sky shades the mesh (BOLT_FOG: a
// bolt's light reaches through the rain far better than the ground's).
const char *const BOLT_FS_DEC = R"(//VTK::Normal::Dec
uniform float boltIntensity;
)";
const char *const BOLT_FS_LIGHT = R"(  float bx = 1.0 - clamp(diffuseColor.r, 0.0, 1.0);
  float core = exp(-bx * bx * 2.5), halo = exp(-bx * bx * 4.5);
  float glow = clamp(diffuseColor.g, 0.0, 1.0), fade = clamp(diffuseColor.b, 0.0, 1.0);
  vec3 bcol = mix(vec3(1.0, 0.98, 0.95), vec3(0.70, 0.78, 1.0), glow);
  float ba = clamp(boltIntensity * fade * mix(core, 0.45 * halo, glow), 0.0, 1.0);
  gl_FragData[0] = vec4(BOLT_FOG, ba);
)";

std::string bolt_light(bool shaded) {
  std::string s = BOLT_FS_LIGHT;
  const std::string mark = "BOLT_FOG";
  s.replace(s.find(mark), mark.size(),
            shaded ? "mix(bcol, cloudFog(bcol, vertexVC.xyz), 0.2)" : "bcol");
  return s;
}

// The streaks' mesh: `count` quads, each a drop (its corner in xy, its index in z).
cvc::geometry rain_mesh(cvc::app &app, int count) {
  cvc::geometry g(app);
  auto &pts = g.points();
  auto &tris = g.tris();
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

struct WeatherRT {
  cvc::app *app = nullptr;
  std::string prefix; // the scene's state prefix (its wind's keys)
  cvc::world::weather model;
  // Its keys (weather.* or the node's keys: root).
  std::string eventsKey, scaleKey, requestKey;
  std::string stateKey, storminessKey, cellsKey, stormsKey, rainKey, deckKey;
  // The cloud_sky it drives (its key root's keys), and the scene wind's (steeringWind).
  std::string skyTop, skyBase, skyDark, skyStorms, skyWindAz, skyDrift;
  bool drivesSky = false;
  double cx = 0.0, cy = 0.0;
  // The deck: as authored (read on the first tick), and in a full storm.
  bool fairRead = false;
  double fairTop = 0.0, fairBase = 0.0, fairDark = 0.0;
  double stormTop = STORM_TOP, stormBase = std::numeric_limits<double>::quiet_NaN();
  double stormDark = STORM_DARK, reach = REACH;
  int lastStep = -1;      // the deck step last written
  double lastDark = -1.0; // the darkness last written
  std::string lastStorms; // the discs last written
  std::string lastState;  // ...the state, and the numbers (storminess, cells, rain, deck)
  double lastNums[4] = {-1.0, -1.0, -1.0, -1.0};
  std::string eventsText; // the events last parsed (or written)
  // A change waiting to settle: the steering wind and the time scale.
  double steerSpeed = 0.0, steerAz = 0.0, scale = 1.0;
  double pendSpeed = 0.0, pendAz = 0.0, pendScale = 1.0;
  int steerHeld = 0, scaleHeld = 0;
  bool first = true;
  // O19, the rain: its keys (how strong the streaks, rings, fog and wet ground are; the streaks'
  // share of their most; the rain at the camera, published), the sky's (its tracks and constants),
  // the ground under the camera, and the streaks.
  std::string streaksKey, ringsKey, fogKey, wetKey, qualityKey, rainHereKey;
  std::string skyRain, skyRainK;
  double soak = SOAK, dry = DRY;
  std::string lastRain, lastRainK;
  std::shared_ptr<const cvc::height_field> ground;
  std::weak_ptr<cvc::gl::GeometryNode> streaks;
  std::shared_ptr<cvc::gl::GeometryNode::ShaderInputs> rainIn;
  int rainCount = RAIN_COUNT;
  // O20: the lightning (its strength key; the flash, published; the bolt's mesh, the strike it
  // shows) and the rain in the scene's friction (weather.friction; the wetness it set).
  std::string lightningKey, flashKey, skyFlash;
  std::weak_ptr<cvc::gl::GeometryNode> bolt;
  std::shared_ptr<cvc::gl::GeometryNode::ShaderInputs> boltIn;
  std::uint64_t boltId = 0;
  bool boltShown = false, boltShaded = false;
  double lastFlash = -1.0, lastSkyFlash = -1.0, flashT = -1.0;
  bool rainShaded = false;
  double lastRainHere = -1.0;
};

std::string key(const std::string &pref, const ari::SceneNode &n, const char *k) {
  return ari::resolve_bind(pref, ari::node_key(n, k));
}

void put(cvc::app &app, const std::string &path, const std::string &v) {
  ari::write_node(app, path).value(v);
}
void put(cvc::app &app, const std::string &path, double v) { ari::write_node(app, path).value(v); }

// The scene's one wind as the storms' steering: metres per second and the heading it blows toward
// (degrees, 90 is +x, as cloud_layer's wind_az). With a scene wind (O14's wind.speed, wind.dir),
// ari::scene_weather_steering: its heading at its speed at the deck's base as authored times
// sky.drift, how the cloud deck drifts without its gusts' pulse, so a storm keeps to the clouds
// drawn around it, its own gust front does not carry it, and a steer is logged only when the wind
// is changed (the weather starts from it, here and in the scene's world). Without one, the cloud
// deck's own: sky.wind_az at CLOUD_SKY_DRIFT m/s times sky.drift.
double deckBase(const WeatherRT &r) {
  return r.fairRead ? r.fairBase : ari::read_or<double>(*r.app, r.skyBase, ari::kWeatherDeckBase);
}
void steeringWind(const WeatherRT &r, double &speed, double &az) {
  const double drift = ari::read_or<double>(*r.app, r.skyDrift, 1.0);
  if (ari::scene_weather_steering(*r.app, r.prefix, deckBase(r), drift, speed, az))
    return;
  az = ari::read_or<double>(*r.app, r.skyWindAz, r.model.params().steer_az);
  speed = CLOUD_SKY_DRIFT * drift;
}

// The ground's height under (x, y), at its nearest edge beyond it (a strike can land past the
// terrain, the camera can stand off it) and 0 over a hole or with no ground.
double groundAt(const WeatherRT &r, double x, double y) {
  if (!r.ground || r.ground->rows() < 2 || r.ground->cols() < 2)
    return 0.0;
  const cvc::height_field &g = *r.ground;
  const double h = g.height(std::clamp(x, g.x0(), g.max_x()), std::clamp(y, g.y0(), g.max_y()));
  return std::isfinite(h) ? h : 0.0;
}

// Log an event at world time t and publish the list (compacted when it has grown long).
void logEvent(WeatherRT &r, const cvc::world::weather_event &e, double t) {
  r.model.add_event(e);
  if (r.model.events().size() > COMPACT)
    r.model.compact(t);
  r.eventsText = cvc::world::format_weather_events(r.model.events());
  put(*r.app, r.eventsKey, r.eventsText);
}

// The discs as text: "x y r intensity" entries split by ';' (cloud_sky's sky.storms).
std::string discs(const cvc::world::weather_now &w) {
  std::string s;
  char buf[96];
  for (const cvc::world::storm_cell &c : w.cells) {
    std::snprintf(buf, sizeof buf, "%s%.1f %.1f %.1f %.4f", s.empty() ? "" : "; ", c.x, c.y, c.r,
                  c.intensity);
    s += buf;
  }
  return s;
}

void rain(WeatherRT &r, const cvc::world::weather_now &w, vtkRenderer *ren);
void lightning(WeatherRT &r, vtkRenderer *ren);

void tick(WeatherRT &r, vtkRenderer *ren) {
  cvc::app &app = *r.app;
  const double t = app.world_clock().t();
  // The events, when someone else wrote them (a server, a script).
  {
    const std::string text = ari::read_or<std::string>(app, r.eventsKey, std::string());
    if (text != r.eventsText) {
      std::vector<cvc::world::weather_event> ev;
      std::string err;
      if (cvc::world::parse_weather_events(text, ev, &err)) {
        r.model.set_events(std::move(ev));
      } else {
        app.log(1, "ari: weather: weather.events: " + err + "\n");
      }
      r.eventsText = text;
    }
  }
  // The steering wind and the time scale: logged once a change has settled.
  double speed, az;
  steeringWind(r, speed, az);
  const double scale = std::max(0.0, ari::read_or<double>(app, r.scaleKey, r.scale));
  if (r.first) {
    // The weather starts from its props' steering and rate; what the scene has now is an event.
    // With a scene wind it starts steered by it at the sky's base and drift, read now that the sky
    // has seeded its keys, as the scene's world starts it (WeatherFriction,
    // scene_weather_deck_base: the same cloud_sky's base).
    r.first = false;
    {
      cvc::world::weather_params p = r.model.params();
      if (ari::scene_weather_steering(app, r.prefix, deckBase(r),
                                      ari::read_or<double>(app, r.skyDrift, 1.0), p.steer_speed,
                                      p.steer_az))
        r.model.set_params(p);
    }
    r.steerSpeed = r.model.params().steer_speed;
    r.steerAz = r.model.params().steer_az;
    r.scale = r.model.params().rate;
    for (const cvc::world::weather_event &e : r.model.events())
      if (e.type == cvc::world::weather_event::kind::steer) {
        r.steerSpeed = e.speed;
        r.steerAz = e.az;
      } else if (e.type == cvc::world::weather_event::kind::rate) {
        r.scale = e.scale;
      }
    r.pendSpeed = r.steerSpeed;
    r.pendAz = r.steerAz;
    r.pendScale = r.scale;
  }
  if (speed != r.pendSpeed || az != r.pendAz) {
    r.pendSpeed = speed;
    r.pendAz = az;
    r.steerHeld = 0;
  }
  if (scale != r.pendScale) {
    r.pendScale = scale;
    r.scaleHeld = 0;
  }
  const bool windHeld = ++r.steerHeld >= HOLD_FRAMES, scaleHeld = ++r.scaleHeld >= HOLD_FRAMES;
  if (windHeld && (std::fabs(speed - r.steerSpeed) > 1e-6 || std::fabs(az - r.steerAz) > 1e-6)) {
    cvc::world::weather_event e;
    e.type = cvc::world::weather_event::kind::steer;
    e.t = t;
    e.speed = speed;
    e.az = az;
    logEvent(r, e, t);
    r.steerSpeed = speed;
    r.steerAz = az;
  }
  if (scaleHeld && std::fabs(scale - r.scale) > 1e-9) {
    cvc::world::weather_event e;
    e.type = cvc::world::weather_event::kind::rate;
    e.t = t;
    e.scale = scale;
    logEvent(r, e, t);
    r.scale = scale;
  }
  // A storm started, or the sky cleared, from the panel (or a script).
  {
    const std::string req = ari::read_or<std::string>(app, r.requestKey, std::string());
    if (!req.empty()) {
      cvc::world::weather_event e;
      e.t = t;
      bool ok = true;
      if (req == "calm" || req == "clear") {
        e.type = cvc::world::weather_event::kind::calm;
      } else if (req == "overhead") {
        // In its prime over the ground's centre now: halfway through its maturity, born that long
        // ago (its weather time anchored, whatever the rate was) upwind by the steering now.
        const cvc::world::weather_params &p = r.model.params();
        e.life = 0.5 * (p.life_min + p.life_max);
        const double mid = e.life * (p.growth + 0.5 * (1.0 - p.growth - p.decay));
        const double tau = r.model.weather_time(t);
        double vx, vy;
        r.model.steering(tau, vx, vy);
        e.x = r.cx - vx * mid;
        e.y = r.cy - vy * mid;
        e.tau = tau - mid;
      } else if (req == "upwind" || req == "storm" || req == "start") {
        // born now upwind of the ground, timed to cross it in its prime (the model places it)
      } else {
        ok = false;
        app.log(1, "ari: weather: unknown request '" + req + "'\n");
      }
      if (ok)
        logEvent(r, e, t);
      put(app, r.requestKey, std::string());
    }
  }

  // The weather now, published as it changes.
  const cvc::world::weather_now w = r.model.at(t);
  const std::string state = cvc::world::weather_state_name(w.state);
  if (state != r.lastState) {
    r.lastState = state;
    put(app, r.stateKey, state);
  }
  const double deck = cvc::world::weather::nearby(w, r.cx, r.cy, r.reach);
  const double nums[4] = {w.storminess, double(w.cells.size()), r.model.rain_rate(w, r.cx, r.cy),
                          deck};
  const std::string *numKeys[4] = {&r.storminessKey, &r.cellsKey, &r.rainKey, &r.deckKey};
  for (int k = 0; k < 4; ++k)
    if (nums[k] != r.lastNums[k]) {
      r.lastNums[k] = nums[k];
      put(app, *numKeys[k], nums[k]);
    }
  const std::string d = discs(w);
  if (d != r.lastStorms) {
    put(app, r.stormsKey, d);
    if (r.drivesSky)
      put(app, r.skyStorms, d);
    r.lastStorms = d;
  }
  rain(r, w, ren);
  lightning(r, ren);
  if (!r.drivesSky)
    return;
  // The deck: its heights and darkness as authored, read once; then a storm's.
  if (!r.fairRead) {
    r.fairRead = true;
    r.fairTop = ari::read_or<double>(app, r.skyTop, 520.0);
    r.fairBase = ari::read_or<double>(app, r.skyBase, 300.0);
    r.fairDark = ari::read_or<double>(app, r.skyDark, 0.0);
  }
  const int step = static_cast<int>(std::lround(deck * DECK_STEPS));
  if (step != r.lastStep) {
    r.lastStep = step;
    const double u = double(step) / DECK_STEPS;
    put(app, r.skyTop, r.fairTop + (std::max(r.stormTop, r.fairTop) - r.fairTop) * u);
    if (std::isfinite(r.stormBase))
      put(app, r.skyBase, r.fairBase + (r.stormBase - r.fairBase) * u);
  }
  const double dark = r.fairDark + (std::max(r.stormDark, r.fairDark) - r.fairDark) * deck;
  if (std::fabs(dark - r.lastDark) > 0.005 || (deck == 0.0 && r.lastDark != dark)) {
    r.lastDark = dark;
    put(app, r.skyDark, dark);
  }
}

// The rain (O19): the recent storms' tracks and the rain's constants for the sky (its wet ground,
// rings, shafts and fog), and the streaks near the camera.
void rain(WeatherRT &r, const cvc::world::weather_now &w, vtkRenderer *ren) {
  cvc::app &app = *r.app;
  const double t = app.world_clock().t();
  const double strength[4] = {std::max(0.0, ari::read_or<double>(app, r.streaksKey, 1.0)),
                              std::max(0.0, ari::read_or<double>(app, r.ringsKey, 1.0)),
                              std::max(0.0, ari::read_or<double>(app, r.fogKey, 1.0)),
                              std::max(0.0, ari::read_or<double>(app, r.wetKey, 1.0))};
  if (r.drivesSky) {
    const cvc::world::weather_params &p = r.model.params();
    char buf[160];
    std::snprintf(buf, sizeof buf, "%.4f %.4f %.1f %.1f %.4f %.4f %.4f", p.rain_onset, p.rain_max,
                  r.soak, r.dry, strength[1], strength[2], strength[3]);
    if (buf != r.lastRainK) {
      r.lastRainK = buf;
      put(app, r.skyRainK, std::string(buf));
    }
    // The tracks: the storms of the last 4 dry times, the latest first, at most MAX_TRACKS.
    std::vector<cvc::world::storm_cell> cells = r.model.recent(t, 4.0 * r.dry);
    std::sort(cells.begin(), cells.end(),
              [](const cvc::world::storm_cell &a, const cvc::world::storm_cell &b) {
                return a.end > b.end;
              });
    std::string text;
    const double tau = r.model.weather_time(t);
    for (std::size_t k = 0; k < cells.size() && k < std::size_t(MAX_TRACKS); ++k) {
      const cvc::world::storm_cell &c = cells[k];
      std::snprintf(buf, sizeof buf, "%s%.1f %.1f %.1f %.4f %.4f %.1f %.1f",
                    text.empty() ? "" : "; ", c.x, c.y, r.model.rain_radius(c.radius, c.peak), c.vx,
                    c.vy, c.age, c.end - tau);
      text += buf;
    }
    if (text != r.lastRain) {
      r.lastRain = text;
      put(app, r.skyRain, text);
    }
  }
  // The streaks, near the camera.
  auto st = r.streaks.lock();
  if (!st || !ren || !ren->IsActiveCameraCreated())
    return;
  vtkCamera *cam = ren->GetActiveCamera();
  double cp[3], fp[3];
  cam->GetPosition(cp);
  cam->GetFocalPoint(fp);
  const double here = r.model.rain_rate(w, cp[0], cp[1]);
  if (here != r.lastRainHere) {
    r.lastRainHere = here;
    put(app, r.rainHereKey, here);
  }
  const double amount = strength[0] * std::min(here / RAIN_FULL, 1.5);
  const double base = ari::read_or<double>(app, r.skyBase, 300.0);
  const bool show = amount > 0.0 && cp[2] < base + 0.5 * RAIN_HIGH;
  if (st->isVisible() != show)
    st->setVisible(show);
  // Lit and fogged by the sky once it lists the streaks among its receivers.
  if (const bool shaded = st->hasSharedShaderInputs("cloud_sky"); shaded != r.rainShaded) {
    r.rainShaded = shaded;
    st->addFragmentShaderReplacement("//VTK::Light::Impl", rain_light(shaded));
  }
  if (!show)
    return;
  cvc::gl::GeometryNode::ShaderInputs &in = *r.rainIn;
  const double quality = std::clamp(ari::read_or<double>(app, r.qualityKey, 1.0), 0.0, 1.0);
  double fwd[3] = {fp[0] - cp[0], fp[1] - cp[1], fp[2] - cp[2]};
  const double fl = std::max(1e-9, std::sqrt(fwd[0] * fwd[0] + fwd[1] * fwd[1] + fwd[2] * fwd[2]));
  // The slant: the scene's wind where the camera is, its gust front there in it (F4); without a
  // scene wind, the steering wind (the clouds' drift).
  constexpr double kPi = 3.14159265358979323846;
  double windX, windY;
  if (const std::optional<ari::SceneWind> sw = ari::scene_wind_set(app, r.prefix)) {
    double fx, fy;
    r.model.gust_front(w, cp[0], cp[1], fx, fy);
    windX = sw->speed * std::cos(sw->dir * kPi / 180.0) + fx;
    windY = sw->speed * std::sin(sw->dir * kPi / 180.0) + fy;
  } else {
    double speed, az;
    steeringWind(r, speed, az);
    windX = speed * std::sin(az * kPi / 180.0);
    windY = -speed * std::cos(az * kPi / 180.0);
  }
  int *sz = ren->GetSize();
  const double h = sz && sz[1] > 0 ? double(sz[1]) : 720.0;
  const double ground = groundAt(r, cp[0], cp[1]);
  in.f3["rainCam"] = {float(cp[0]), float(cp[1]), float(cp[2])};
  in.f3["rainFwd"] = {float(fwd[0] / fl), float(fwd[1] / fl), float(fwd[2] / fl)};
  in.f["rainTime"] = float(std::fmod(t, 3600.0));
  in.f["rainAmount"] = float(amount);
  in.f4["rainWind"] = {float(windX), float(windY), 0.0f, 0.0f};
  in.f["rainCount"] = float(r.rainCount * quality * std::min(amount * 1.2, 1.0));
  in.f4["rainBox"] = {float(RAIN_BOX), float(RAIN_HIGH), 0.0f, 0.0f};
  in.f["rainStreak"] = 0.042f;
  in.f["rainPixel"] = float(2.0 * std::tan(0.5 * cam->GetViewAngle() * kPi / 180.0) / h);
  in.f["rainGround"] = float(ground);
  in.f["rainTop"] = float(base);
  in.f3["rainColor"] = {0.74f, 0.77f, 0.82f};
}

// The lightning (O20): the latest strike of the last moment, its bolt drawn while it shows, and its
// flash handed to the sky (sky.flash, "x y z intensity": the receivers', the water's and the far
// clouds' transient light). A strike's strokes are in world seconds, whatever the weather's rate.
// The sky reads sky.flash on its own tick, earlier in the frame than this one (it comes first in a
// document), so it is handed the flash a frame step ahead: the light lands on the bolt's frame.
constexpr double FLASH_LOOK = 0.8; // world seconds back the strikes are looked for
void lightning(WeatherRT &r, vtkRenderer *ren) {
  cvc::app &app = *r.app;
  const double t = app.world_clock().t();
  const double ahead = r.flashT >= 0.0 ? std::clamp(t - r.flashT, 0.0, 0.1) : 0.0;
  r.flashT = t;
  const double strength = std::max(0.0, ari::read_or<double>(app, r.lightningKey, 1.0));
  const double rate = r.model.rate_at(t);
  double flash = 0.0, next = 0.0; // now, and a frame step on (the sky's)
  const cvc::world::lightning_strike *shown = nullptr, *lit = nullptr;
  std::vector<cvc::world::lightning_strike> ks;
  if (strength > 0.0 && rate > 0.0) {
    ks = r.model.strikes(t + ahead, (FLASH_LOOK + ahead) * rate);
    const double tau = r.model.weather_time(t), tauNext = r.model.weather_time(t + ahead);
    for (const cvc::world::lightning_strike &k : ks) {
      if (const double a = k.intensity((tau - k.tau) / rate) * strength; a > flash) {
        flash = a;
        shown = &k;
      }
      if (const double a = k.intensity((tauNext - k.tau) / rate) * strength; a > next) {
        next = a;
        lit = &k;
      }
    }
  }
  const double base = ari::read_or<double>(app, r.skyBase, 300.0);
  if (r.drivesSky && (next > 0.0 || r.lastSkyFlash != 0.0)) {
    r.lastSkyFlash = next;
    char buf[128];
    if (lit)
      std::snprintf(buf, sizeof buf, "%.1f %.1f %.1f %.4f", lit->x, lit->y, base, next);
    else
      std::snprintf(buf, sizeof buf, "0 0 0 0");
    put(app, r.skyFlash, std::string(buf));
  }
  // Published in steps of 0.02, so a strike is a few writes and not one a frame.
  if (const double fq = std::round(flash * 50.0) / 50.0; fq != r.lastFlash) {
    r.lastFlash = fq;
    put(app, r.flashKey, fq);
  }
  auto bolt = r.bolt.lock();
  if (!bolt)
    return;
  if (const bool shaded = bolt->hasSharedShaderInputs("cloud_sky"); shaded != r.boltShaded) {
    r.boltShaded = shaded;
    bolt->addFragmentShaderReplacement("//VTK::Light::Impl", bolt_light(shaded));
  }
  const bool show = shown != nullptr && flash > 0.002;
  if (show && shown->id != r.boltId && ren && ren->IsActiveCameraCreated()) {
    // A new strike: its bolt, facing the camera now (it shows for a fraction of a second).
    double eye[3];
    ren->GetActiveCamera()->GetPosition(eye);
    const double ground = groundAt(r, shown->x, shown->y);
    bolt->setGeometry(boltMesh(app, boltOf(*shown, base, ground), eye));
    r.boltId = shown->id;
  }
  if (show)
    r.boltIn->f["boltIntensity"] = float(flash);
  if (show != r.boltShown) {
    r.boltShown = show;
    bolt->setVisible(show);
  }
}

// type: weather — the scene's weather (see above).
// props: ground (the node whose extent the storms serve; default world, else `area` around
// (`cx`, `cy`)), sky (the cloud_sky's key root to drive: default sky; "" drives none), seed,
// interval / chance / quiet (the schedule: a slot's length, the share of slots with a storm, and
// the clear opening, weather seconds), life_min / life_max, growth / decay (shares of a life),
// radius_min / radius_max, peak_min / peak_max, miss (how far to the side of the centre a storm
// passes, in half-widths), rain_onset / rain_max (mm/h), calm (a calm's fade, s), time_scale
// (weather seconds per world second; live: weather.time_scale), events (authored events, as
// weather.events text), storm_top / storm_base (the deck in a full storm; the base stays put
// unless given), darkness (the deck's darkness with a storm in view) and reach (how far past a
// storm's edge the deck feels it, metres); O20: flash_rate / flash_onset / ground_share (the
// lightning's schedule, cvc::world::weather_params), lightning (its strength; live:
// weather.lightning) and friction (0: the rain leaves the physics world's friction alone; live:
// weather.friction); F4: gust_front (a full cell's outflow at its front's edge, m/s; 0: none),
// front_ahead / front_behind (how far the front's edge is ahead of and behind the cell, in its
// radii) and front_width (how far past the edge it dies away, in that distance).
std::shared_ptr<GraphicsNode> realize_weather(SceneGraph &sg, const ari::SceneNode &n,
                                              GraphicsNode *parent, RealizedScene &out,
                                              std::vector<std::string> *warnings) {
  auto rt = std::make_shared<WeatherRT>();
  cvc::app &app = sg.appContext();
  rt->app = &app;
  // The storms' props, read as the scene's world reads them (its friction follows these storms).
  const std::string ground = ari::scene_weather_ground(n);
  const auto groundIt = out.height_fields.find(ground);
  const cvc::world::weather_params p = ari::scene_weather_params(
      n, groundIt != out.height_fields.end() ? groundIt->second.get() : nullptr);
  rt->model.set_params(p);
  rt->cx = p.cx;
  rt->cy = p.cy;
  rt->stormTop = n.props.num("storm_top", STORM_TOP);
  if (n.props.has("storm_base"))
    rt->stormBase = n.props.num("storm_base", 0.0);
  rt->stormDark = std::clamp(n.props.num("darkness", STORM_DARK), 0.0, 1.0);
  rt->reach = std::max(0.0, n.props.num("reach", REACH));

  const std::string pref = sg.getStatePrefix();
  rt->prefix = pref;
  rt->eventsKey = key(pref, n, "weather.events");
  rt->scaleKey = key(pref, n, "weather.time_scale");
  rt->requestKey = key(pref, n, "weather.request");
  rt->stateKey = key(pref, n, "weather.state");
  rt->storminessKey = key(pref, n, "weather.storminess");
  rt->cellsKey = key(pref, n, "weather.cells");
  rt->stormsKey = key(pref, n, "weather.storms");
  rt->rainKey = key(pref, n, "weather.rain");
  rt->deckKey = key(pref, n, "weather.deck");
  ari::read_or_seed<std::string>(app, rt->eventsKey, n.props.str("events"));
  ari::read_or_seed<double>(app, rt->scaleKey, p.rate);
  ari::read_or_seed<std::string>(app, rt->requestKey, std::string());
  // The cloud_sky it drives: its key root (sky, or its keys:).
  const std::string sky = n.props.has("sky") ? n.props.str("sky") : std::string("sky");
  rt->drivesSky = !sky.empty();
  const std::string root = sky.empty() ? std::string("sky") : sky;
  rt->skyTop = ari::resolve_bind(pref, root + ".top");
  rt->skyBase = ari::resolve_bind(pref, root + ".base");
  rt->skyDark = ari::resolve_bind(pref, root + ".darkness");
  rt->skyStorms = ari::resolve_bind(pref, root + ".storms");
  rt->skyWindAz = ari::resolve_bind(pref, root + ".wind_az");
  rt->skyDrift = ari::resolve_bind(pref, root + ".drift");
  // The events as authored, parsed now (the first tick sees them unchanged).
  {
    rt->eventsText = ari::read_or<std::string>(app, rt->eventsKey, std::string());
    std::vector<cvc::world::weather_event> ev;
    std::string err;
    if (cvc::world::parse_weather_events(rt->eventsText, ev, &err))
      rt->model.set_events(std::move(ev));
    else if (warnings)
      warnings->push_back("ari: weather '" + n.id + "': events: " + err);
  }

  // O19, the rain's keys, and the streaks: a mesh of drops in the volume layer (after the cloud
  // deck: list it among the sky's receivers, after the sky in the document), out of the bounds.
  rt->streaksKey = key(pref, n, "weather.streaks");
  rt->ringsKey = key(pref, n, "weather.rings");
  rt->fogKey = key(pref, n, "weather.fog");
  rt->wetKey = key(pref, n, "weather.wet");
  rt->qualityKey = key(pref, n, "weather.rain_quality");
  rt->rainHereKey = key(pref, n, "weather.rain_here");
  for (const std::string *k : {&rt->streaksKey, &rt->ringsKey, &rt->fogKey, &rt->wetKey})
    ari::read_or_seed<double>(app, *k, 1.0);
  ari::read_or_seed<double>(app, rt->qualityKey,
                            std::clamp(n.props.num("rain_quality", 1.0), 0.0, 1.0));
  rt->skyRain = ari::resolve_bind(pref, root + ".rain");
  rt->skyRainK = ari::resolve_bind(pref, root + ".rain_k");
  rt->soak = ari::scene_weather_soak(n);
  rt->dry = ari::scene_weather_dry(n);
  if (groundIt != out.height_fields.end())
    rt->ground = groundIt->second;
  rt->rainCount = std::clamp(static_cast<int>(n.props.num("rain_count", RAIN_COUNT)), 0, 65536);
  std::shared_ptr<GraphicsNode> node = parent ? parent->createChild(n.id) : sg.addGraphics(n.id);
  if (rt->rainCount > 0 && node) {
    auto mesh = std::dynamic_pointer_cast<GeometryNode>(
        node->createChild<GeometryNode>(n.id + "_rain", rain_mesh(app, rt->rainCount)));
    if (mesh) {
      mesh->setUseBounds(false);
      mesh->setContributesToSceneBounds(false);
      mesh->setCastsShadow(false);
      mesh->setDrawWithVolumes(true);
      mesh->disableCoordinateShiftScale();
      mesh->addVertexShaderReplacement("//VTK::PositionVC::Dec", RAIN_VS_DEC);
      mesh->addVertexShaderReplacement("//VTK::PositionVC::Impl", RAIN_VS_IMPL);
      mesh->addFragmentShaderReplacement("//VTK::Normal::Dec", RAIN_FS_DEC);
      mesh->addFragmentShaderReplacement("//VTK::Light::Impl", rain_light(false));
      rt->rainIn = std::make_shared<GeometryNode::ShaderInputs>();
      mesh->setSharedShaderInputs(RAIN_INPUTS, rt->rainIn);
      mesh->setVisible(false);
      rt->streaks = mesh;
    }
  }
  // O20: the lightning's keys and its bolt (a mesh of ribbons in the volume layer, after the deck:
  // list it among the sky's receivers too), and the rain's friction.
  rt->lightningKey = key(pref, n, "weather.lightning");
  rt->flashKey = key(pref, n, "weather.flash");
  ari::read_or_seed<double>(app, rt->lightningKey, std::max(0.0, n.props.num("lightning", 1.0)));
  // weather.friction (the Weather panel's): the scene's world reads it (WeatherFriction).
  ari::read_or_seed<double>(app, key(pref, n, "weather.friction"),
                            n.props.num("friction", 1.0) != 0.0 ? 1.0 : 0.0);
  rt->skyFlash = ari::resolve_bind(pref, root + ".flash");
  if (node) {
    std::vector<BoltSeg> none;
    const double eye[3] = {0.0, 0.0, 0.0};
    auto mesh = std::dynamic_pointer_cast<GeometryNode>(
        node->createChild<GeometryNode>(n.id + "_bolt", boltMesh(app, none, eye)));
    if (mesh) {
      mesh->setUseBounds(false);
      mesh->setContributesToSceneBounds(false);
      mesh->setCastsShadow(false);
      mesh->setDrawWithVolumes(true);
      mesh->disableCoordinateShiftScale();
      mesh->addFragmentShaderReplacement("//VTK::Normal::Dec", BOLT_FS_DEC);
      mesh->addFragmentShaderReplacement("//VTK::Light::Impl", bolt_light(false));
      mesh->setAmbient(0.0); // diffuseColor is the vertex colour as given (bolt_light reads it)
      mesh->setDiffuse(1.0);
      mesh->setSpecular(0.0);
      rt->boltIn = std::make_shared<GeometryNode::ShaderInputs>();
      mesh->setSharedShaderInputs("weather_bolt", rt->boltIn);
      mesh->setVisible(false);
      rt->bolt = mesh;
    }
  }

  out.custom_ticks.push_back([rt](vtkRenderer *ren) { tick(*rt, ren); });
  // A node of its own (the rain's mesh under it), so the scene lists it and a host can find it.
  return node;
}

} // namespace

void register_weather_node_types(cvc::app &app) {
  register_scene_node_type(app, "weather", realize_weather);
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
