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
*/

// weather_node.cpp — the Ariadne `weather` node (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md §9 F2,
// §11.2, O18): the scene's weather model, and the storms it puts in the sky.
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
// Hooks for what follows: O19's rain reads the cells (weather.storms) and the rain rate
// (cvc::world::weather::rain_rate, mm/h at any point); O20's storm wind, lightning and friction
// read the same cells, their phases and motion.
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
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/lsystem_nodes.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/world_nodes.h>
#include <cvc/world/weather.h>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
namespace gl {
namespace ariadne {
namespace {

namespace ari = cvc::ariadne;

constexpr int HOLD_FRAMES = 10;     // a wind or rate change is logged once it has held this long
constexpr int DECK_STEPS = 4;       // the deck's top moves in this many steps to the storm's
constexpr std::size_t COMPACT = 24; // the events are compacted past this many
constexpr double STORM_TOP = 3200.0, STORM_DARK = 0.6, REACH = 4000.0;

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

void tick(WeatherRT &r) {
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

  out.custom_ticks.push_back([rt](vtkRenderer *) { tick(*rt); });
  // A node of its own (nothing drawn), so the scene lists it and a host can find it.
  return parent ? parent->createChild(n.id) : sg.addGraphics(n.id);
}

} // namespace

void register_weather_node_types(cvc::app &app) {
  register_scene_node_type(app, "weather", realize_weather);
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
