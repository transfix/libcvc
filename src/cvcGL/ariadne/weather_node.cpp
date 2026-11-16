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
*/

// weather_node.cpp — the Ariadne `weather` node (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md §9 F2,
// F3, §11.2, O18, O19): the scene's weather model, the storms it puts in the sky, and their rain.
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
// weather.rain_quality the streaks' count. O20's storm wind, lightning and friction read the cells,
// the rain rate and cvc::world::weather::wetness.
//
// The steering wind the cells ride is the scene's one wind (R5), read through steeringWind(): until
// one scene wind exists (O14's wind.speed / wind.dir) it is the cloud deck's, sky.wind_az at the
// deck's drift speed, so the storms move with the clouds drawn. A change in it is logged as a
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
constexpr double RAIN_FULL = 25.0, SOAK = 300.0, DRY = 900.0, RAIN_BOX = 80.0, RAIN_HIGH = 52.0;
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
// (degrees, 90 is +x, as cloud_layer's wind_az). Until O14 adds one scene wind (wind.speed,
// wind.dir), the cloud deck's: sky.wind_az at the deck's drift speed (CLOUD_SKY_DRIFT m/s times
// sky.drift), so a storm keeps to the clouds drawn around it. O14 points this at wind.*.
void steeringWind(const WeatherRT &r, double &speed, double &az) {
  az = ari::read_or<double>(*r.app, r.skyWindAz, r.model.params().steer_az);
  speed = CLOUD_SKY_DRIFT * ari::read_or<double>(*r.app, r.skyDrift, 1.0);
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
    r.first = false;
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
  double speed, az;
  steeringWind(r, speed, az);
  constexpr double kPi = 3.14159265358979323846;
  int *sz = ren->GetSize();
  const double h = sz && sz[1] > 0 ? double(sz[1]) : 720.0;
  double ground = 0.0;
  if (r.ground)
    ground = r.ground->height(cp[0], cp[1]);
  in.f3["rainCam"] = {float(cp[0]), float(cp[1]), float(cp[2])};
  in.f3["rainFwd"] = {float(fwd[0] / fl), float(fwd[1] / fl), float(fwd[2] / fl)};
  in.f["rainTime"] = float(std::fmod(t, 3600.0));
  in.f["rainAmount"] = float(amount);
  in.f4["rainWind"] = {float(speed * std::sin(az * kPi / 180.0)),
                       float(-speed * std::cos(az * kPi / 180.0)), 0.0f, 0.0f};
  in.f["rainCount"] = float(r.rainCount * quality * std::min(amount * 1.2, 1.0));
  in.f4["rainBox"] = {float(RAIN_BOX), float(RAIN_HIGH), 0.0f, 0.0f};
  in.f["rainStreak"] = 0.042f;
  in.f["rainPixel"] = float(2.0 * std::tan(0.5 * cam->GetViewAngle() * kPi / 180.0) / h);
  in.f["rainGround"] = float(ground);
  in.f["rainTop"] = float(base);
  in.f3["rainColor"] = {0.74f, 0.77f, 0.82f};
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
// storm's edge the deck feels it, metres).
std::shared_ptr<GraphicsNode> realize_weather(SceneGraph &sg, const ari::SceneNode &n,
                                              GraphicsNode *parent, RealizedScene &out,
                                              std::vector<std::string> *warnings) {
  auto rt = std::make_shared<WeatherRT>();
  cvc::app &app = sg.appContext();
  rt->app = &app;
  cvc::world::weather_params p;
  p.seed = static_cast<std::uint64_t>(n.props.num("seed", 1.0));
  // The area: the ground's box, or the props'.
  const std::string ground = n.props.str("ground").empty() ? "world" : n.props.str("ground");
  if (const auto it = out.height_fields.find(ground);
      !n.props.has("area") && it != out.height_fields.end() && it->second) {
    const cvc::height_field &g = *it->second;
    p.cx = 0.5 * (g.x0() + g.max_x());
    p.cy = 0.5 * (g.y0() + g.max_y());
    p.area = 0.5 * std::max(g.max_x() - g.x0(), g.max_y() - g.y0());
  } else {
    p.cx = n.props.num("cx", 0.0);
    p.cy = n.props.num("cy", 0.0);
    p.area = std::max(1.0, n.props.num("area", p.area));
  }
  p.interval = std::max(1.0, n.props.num("interval", p.interval));
  p.chance = std::clamp(n.props.num("chance", p.chance), 0.0, 1.0);
  p.quiet = n.props.num("quiet", p.quiet);
  p.life_min = std::max(1.0, n.props.num("life_min", p.life_min));
  p.life_max = std::max(p.life_min, n.props.num("life_max", p.life_max));
  p.growth = std::clamp(n.props.num("growth", p.growth), 0.0, 1.0);
  p.decay = std::clamp(n.props.num("decay", p.decay), 0.0, 1.0 - p.growth);
  p.radius_min = std::max(1.0, n.props.num("radius_min", p.radius_min));
  p.radius_max = std::max(p.radius_min, n.props.num("radius_max", p.radius_max));
  p.peak_min = std::clamp(n.props.num("peak_min", p.peak_min), 0.0, 1.0);
  p.peak_max = std::clamp(n.props.num("peak_max", p.peak_max), p.peak_min, 1.0);
  p.miss = std::max(0.0, n.props.num("miss", p.miss));
  p.rain_onset = std::clamp(n.props.num("rain_onset", p.rain_onset), 0.0, 0.999);
  p.rain_max = std::max(0.0, n.props.num("rain_max", p.rain_max));
  p.calm_s = std::max(1.0, n.props.num("calm", p.calm_s));
  p.rate = std::max(0.0, n.props.num("time_scale", 1.0));
  rt->model.set_params(p);
  rt->cx = p.cx;
  rt->cy = p.cy;
  rt->stormTop = n.props.num("storm_top", STORM_TOP);
  if (n.props.has("storm_base"))
    rt->stormBase = n.props.num("storm_base", 0.0);
  rt->stormDark = std::clamp(n.props.num("darkness", STORM_DARK), 0.0, 1.0);
  rt->reach = std::max(0.0, n.props.num("reach", REACH));

  const std::string pref = sg.getStatePrefix();
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
  rt->soak = std::max(1.0, n.props.num("soak", SOAK));
  rt->dry = std::max(1.0, n.props.num("dry", DRY));
  if (const auto it = out.height_fields.find(ground); it != out.height_fields.end())
    rt->ground = it->second;
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
