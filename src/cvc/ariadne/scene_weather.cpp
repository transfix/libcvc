/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// scene_weather.cpp — a weather node's model, and the rain's friction run by the scene's world (see
// scene_weather.h).

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cvc/algo/det_math.h>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_weather.h>
#include <cvc/ariadne/scene_wind.h>
#include <cvc/core/app.h>
#include <cvc/core/world_clock.h>
#include <cvc/geometry/height_field.h>
#include <cvc/ocean/sea_surface.h> // sea_fronts: the fronts where each wave is
#include <cvc/physics/world.h>
#include <cvc/physics/world_geometry.h> // apply_wetness
#include <functional>
#include <limits>
#include <stdexcept>
#include <vector>

namespace cvc {
namespace ariadne {

cvc::world::weather_params scene_weather_params(const SceneNode &n,
                                                const cvc::height_field *ground_field) {
  cvc::world::weather_params p;
  p.seed = static_cast<std::uint64_t>(n.props.num("seed", 1.0));
  // The area: the ground's box, or the props'.
  if (!n.props.has("area") && ground_field) {
    const cvc::height_field &g = *ground_field;
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
  p.flash_rate = std::max(0.0, n.props.num("flash_rate", p.flash_rate));
  p.flash_onset = std::clamp(n.props.num("flash_onset", p.flash_onset), 0.0, 0.999);
  p.ground_share = std::clamp(n.props.num("ground_share", p.ground_share), 0.0, 1.0);
  p.front_speed = std::max(0.0, n.props.num("gust_front", p.front_speed));
  p.front_ahead = std::max(0.01, n.props.num("front_ahead", p.front_ahead));
  p.front_behind = std::max(0.01, n.props.num("front_behind", p.front_behind));
  p.front_width = std::max(0.01, n.props.num("front_width", p.front_width));
  p.rate = std::max(0.0, n.props.num("time_scale", 1.0));
  return p;
}

std::string scene_weather_ground(const SceneNode &n) {
  return n.props.str("ground").empty() ? std::string("world") : n.props.str("ground");
}

double scene_weather_soak(const SceneNode &n) {
  return std::max(1.0, n.props.num("soak", kWeatherSoak));
}

double scene_weather_dry(const SceneNode &n) {
  return std::max(1.0, n.props.num("dry", kWeatherDry));
}

bool scene_weather_steering(cvc::app &app, const std::string &bind_prefix, double deck_base,
                            double drift, double &speed, double &az) {
  const std::optional<SceneWind> w = scene_wind_set(app, bind_prefix);
  if (!w)
    return false;
  az = w->azimuth();
  speed = w->speed * cvc::algo::wind_profile(deck_base) * drift;
  return true;
}

std::string scene_weather_sky(const SceneNode &weather) {
  const std::string sky = weather.props.str("sky");
  return sky.empty() ? std::string("sky") : sky;
}

double scene_weather_deck_base(const Scene &scene, const SceneNode &weather) {
  const std::string root = scene_weather_sky(weather);
  const SceneNode *found = nullptr;
  const std::function<void(const SceneNode &)> visit = [&](const SceneNode &n) {
    if (found)
      return;
    if (n.type == "cloud_sky") {
      const std::string keys = n.props.str("keys");
      if ((keys.empty() ? std::string("sky") : keys) == root) {
        found = &n;
        return;
      }
    }
    for (const SceneNode &c : n.children)
      visit(c);
  };
  for (const SceneNode &n : scene.nodes)
    visit(n);
  return found ? found->props.num("base", kCloudSkyBase) : kWeatherDeckBase;
}

namespace {
constexpr std::uint64_t kNever = std::numeric_limits<std::uint64_t>::max();
} // namespace

struct WeatherFriction::impl {
  cvc::app *app = nullptr;
  SceneNode node;
  std::string ground;
  std::shared_ptr<const cvc::height_field> field;
  cvc::world::weather model;
  double soak = kWeatherSoak, dry = kWeatherDry;
  std::string eventsKey, frictionKey, wetnessKey;
  std::string eventsText; // the events last parsed
  bool on = true;         // weather.friction
  bool flipped = false;   // weather.friction changed since the last step: act at the next one
  std::uint64_t period = kNever; // the kWetPeriod last applied in (its index in clock ticks)
  double applied = -1.0;         // the wetness last applied (-1: none yet)
  double published = std::numeric_limits<double>::quiet_NaN();
  std::uint64_t revision = 0; // model()'s changes
  bool steered = false;       // the scene's wind steers the storms from the start
  double steer_speed = 0.0, steer_az = 0.0;
  // The storm wind's peak (weather.gust_front, m/s): the one the model has, the one the key last
  // read, and the clock tick past which that goes in (the start of the clock period after the one
  // it was read in).
  std::string gustKey;
  double gust = 0.0, gustPending = 0.0;
  std::uint64_t gustFrom = 0;
  // The steering wind and the time scale (F4): the sky's keys the steering is read from (its drift
  // and heading) and the deck's base; the steering and rate in effect (the props', then the last
  // steer and rate events); and a change read, waiting for the tick it goes in after.
  std::string driftKey, windAzKey, scaleKey, prefix;
  double base = kWeatherDeckBase;
  double steerSpeed = 0.0, steerAz = 0.0, rate = 1.0;
  double pendSpeed = 0.0, pendAz = 0.0, pendRate = 1.0;
  std::uint64_t steerFrom = kNever, rateFrom = kNever;
  double dt = 0.0; // the clock's quantum when they were read
  // It follows another host's weather (follow, OW5q): that host logs the steering; this one
  // takes its events as they come and logs none of its own.
  bool following = false;

  // The steering and rate in effect: the props', then the events' last.
  void in_effect() {
    steerSpeed = model.params().steer_speed;
    steerAz = model.params().steer_az;
    rate = model.params().rate;
    for (const cvc::world::weather_event &e : model.events())
      if (e.type == cvc::world::weather_event::kind::steer) {
        steerSpeed = e.speed;
        steerAz = e.az;
      } else if (e.type == cvc::world::weather_event::kind::rate) {
        rate = e.scale;
      }
  }
  // Log event e (at world time e.t), compacted past kCompact events, into weather.events.
  void log(const cvc::world::weather_event &e) {
    model.add_event(e);
    if (model.events().size() > kCompact)
      model.compact(e.t);
    eventsText = cvc::world::format_weather_events(model.events());
    if (app)
      write<std::string>(*app, eventsKey, eventsText);
    period = kNever; // the storms' history changed: the wetness again at the next step
    ++revision;
  }
  static constexpr std::size_t kCompact = 24; // the drawn node's COMPACT

  void params() {
    cvc::world::weather_params p = scene_weather_params(node, field.get());
    if (steered) {
      p.steer_speed = steer_speed;
      p.steer_az = steer_az;
    }
    p.front_speed = gust;
    model.set_params(p);
  }
  std::string key(const std::string &prefix, const char *k) const {
    return resolve_bind(prefix, node_key(node, k));
  }
};

WeatherFriction::WeatherFriction(cvc::app *app, const SceneNode &n, const std::string &bind_prefix,
                                 double deck_base)
    : _impl(std::make_unique<impl>()) {
  impl &d = *_impl;
  d.app = app;
  d.node = n;
  d.node.children.clear();
  d.ground = scene_weather_ground(n);
  d.soak = scene_weather_soak(n);
  d.dry = scene_weather_dry(n);
  d.gust = d.gustPending = scene_weather_params(n, nullptr).front_speed;
  if (app) {
    // The sky it drives (its key root, as the drawn node reads it): its drift, and its base, the
    // sky's key first (a --set or a kept key), as the drawn node reads it, else the document's.
    const std::string sky = scene_weather_sky(n);
    const double drift = read_or<double>(*app, resolve_bind(bind_prefix, sky + ".drift"), 1.0);
    const double base = read_or<double>(*app, resolve_bind(bind_prefix, sky + ".base"), deck_base);
    d.steered = scene_weather_steering(*app, bind_prefix, base, drift, d.steer_speed, d.steer_az);
    d.base = base;
    d.prefix = bind_prefix;
    d.driftKey = resolve_bind(bind_prefix, sky + ".drift");
    d.windAzKey = resolve_bind(bind_prefix, sky + ".wind_az");
    d.scaleKey = d.key(bind_prefix, "weather.time_scale");
    d.gustKey = d.key(bind_prefix, "weather.gust_front");
    d.gust = d.gustPending = std::max(0.0, read_or_seed<double>(*app, d.gustKey, d.gust));
  }
  d.params();
  d.eventsKey = d.key(bind_prefix, "weather.events");
  d.frictionKey = d.key(bind_prefix, "weather.friction");
  d.wetnessKey = d.key(bind_prefix, "weather.wetness");
  d.eventsText = n.props.str("events");
  if (app) {
    d.eventsText = read_or_seed<std::string>(*app, d.eventsKey, d.eventsText);
    d.on = read_or_seed<double>(*app, d.frictionKey,
                                n.props.num("friction", 1.0) != 0.0 ? 1.0 : 0.0) != 0.0;
  } else {
    d.on = n.props.num("friction", 1.0) != 0.0;
  }
  std::vector<cvc::world::weather_event> ev;
  if (cvc::world::parse_weather_events(d.eventsText, ev))
    d.model.set_events(std::move(ev));
  d.in_effect();
  d.pendSpeed = d.steerSpeed;
  d.pendAz = d.steerAz;
  if (app) // as the drawn node seeds it
    read_or_seed<double>(*app, d.scaleKey, d.model.params().rate);
  d.pendRate = d.rate;
}

WeatherFriction::~WeatherFriction() = default;

const std::string &WeatherFriction::node() const { return _impl->node.id; }
const std::string &WeatherFriction::ground() const { return _impl->ground; }

void WeatherFriction::set_ground(std::shared_ptr<const cvc::height_field> field) {
  impl &d = *_impl;
  d.field = std::move(field);
  d.params();
  d.period = kNever; // the area moved: the wetness again at the next step
  ++d.revision;
}

bool WeatherFriction::has_ground() const { return _impl->field != nullptr; }

void WeatherFriction::read_keys(const cvc::world_clock *clock_on) {
  impl &d = *_impl;
  if (!d.app)
    return;
  const std::string text = read_or<std::string>(*d.app, d.eventsKey, std::string());
  if (text != d.eventsText) {
    std::vector<cvc::world::weather_event> ev;
    if (cvc::world::parse_weather_events(text, ev)) {
      d.model.set_events(std::move(ev));
      d.period = kNever; // the storms' history changed: the wetness again at the next step
      ++d.revision;
      d.in_effect();
    }
    d.eventsText = text;
  }
  // The steering wind and the time scale, read as the drawn node reads them: a change goes in at
  // the start of the next clock period (switch_steering), the last value read before it.
  if (!d.following) {
    const cvc::world_clock &clock = clock_on ? *clock_on : d.app->world_clock();
    const std::uint64_t ticks = scene_clock_period_ticks(kSceneWindFrontPeriod, clock.fixed_dt());
    const std::uint64_t from = (clock.tick() / ticks + 1) * ticks;
    d.dt = clock.fixed_dt();
    // No step took a change in (the physics is off): it goes in here, a period after its tick.
    if ((d.steerFrom != kNever && clock.tick() > d.steerFrom + ticks) ||
        (d.rateFrom != kNever && clock.tick() > d.rateFrom + ticks))
      switch_steering(clock.tick());
    const double drift = read_or<double>(*d.app, d.driftKey, 1.0);
    double speed = 0.0, az = 0.0;
    if (!scene_weather_steering(*d.app, d.prefix, d.base, drift, speed, az)) {
      az = read_or<double>(*d.app, d.windAzKey, d.model.params().steer_az);
      speed = kCloudSkyDrift * drift;
    }
    if (std::fabs(speed - d.steerSpeed) <= 1e-6 && std::fabs(az - d.steerAz) <= 1e-6) {
      d.steerFrom = kNever; // back where it is: nothing to log
    } else if (speed != d.pendSpeed || az != d.pendAz || d.steerFrom == kNever) {
      if (d.steerFrom == kNever || from > d.steerFrom)
        d.steerFrom = from;
    }
    d.pendSpeed = speed;
    d.pendAz = az;
    const double scale = std::max(0.0, read_or<double>(*d.app, d.scaleKey, d.rate));
    if (std::fabs(scale - d.rate) <= 1e-9)
      d.rateFrom = kNever;
    else if (scale != d.pendRate || d.rateFrom == kNever)
      if (d.rateFrom == kNever || from > d.rateFrom)
        d.rateFrom = from;
    d.pendRate = scale;
  }
  const bool on = read_or<double>(*d.app, d.frictionKey, 1.0) != 0.0;
  if (on != d.on) {
    d.on = on;
    d.flipped = true;
  }
  // The storm wind's peak: it goes in at the start of the next clock period (before_step), so at
  // the same step whatever the frames.
  const double gust = std::max(0.0, read_or<double>(*d.app, d.gustKey, d.gustPending));
  if (gust != d.gustPending) {
    d.gustPending = gust;
    const cvc::world_clock &clock = clock_on ? *clock_on : d.app->world_clock();
    const std::uint64_t ticks = scene_clock_period_ticks(kSceneWindFrontPeriod, clock.fixed_dt());
    d.gustFrom = (clock.tick() / ticks + 1) * ticks;
  }
}

bool WeatherFriction::switch_gust(std::uint64_t tick) {
  impl &d = *_impl;
  if (d.gustPending == d.gust || tick <= d.gustFrom)
    return false;
  d.gust = d.gustPending; // the storm wind's new peak
  d.params();
  ++d.revision;
  return true;
}

bool WeatherFriction::switch_steering(std::uint64_t tick) {
  impl &d = *_impl;
  bool changed = false;
  if (d.following)
    return false; // the host it follows logs them
  if (d.steerFrom != kNever && tick > d.steerFrom) {
    cvc::world::weather_event e;
    e.type = cvc::world::weather_event::kind::steer;
    e.t = static_cast<double>(d.steerFrom) * d.dt; // the period's start, world seconds
    e.speed = d.pendSpeed;
    e.az = d.pendAz;
    d.steerFrom = kNever;
    d.log(e);
    d.steerSpeed = e.speed;
    d.steerAz = e.az;
    changed = true;
  }
  if (d.rateFrom != kNever && tick > d.rateFrom) {
    cvc::world::weather_event e;
    e.type = cvc::world::weather_event::kind::rate;
    e.t = static_cast<double>(d.rateFrom) * d.dt;
    e.scale = d.pendRate;
    d.rateFrom = kNever;
    d.log(e);
    d.rate = e.scale;
    changed = true;
  }
  return changed;
}

void WeatherFriction::before_step(cvc::physics::world &w, std::uint64_t tick, double dt) {
  impl &d = *_impl;
  if (!d.on) {
    // Off: the surfaces dry at once, and stay as the world's materials have them.
    if (d.applied > 0.0)
      cvc::physics::apply_wetness(w, 0.0);
    d.applied = 0.0;
    d.flipped = false;
    d.period = kNever;
    return;
  }
  // The kWetPeriod this step ends in, in whole clock ticks, so where the friction changes depends
  // on the clock alone: never on how many steps a frame took.
  const std::uint64_t ticks = std::max<std::uint64_t>(
      1, static_cast<std::uint64_t>(std::llround(kWetPeriod / std::max(dt, 1e-9))));
  const std::uint64_t period = tick / ticks;
  if (period == d.period && !d.flipped)
    return;
  d.period = period;
  d.flipped = false;
  const double t = static_cast<double>(period * ticks) * dt; // the period's start, world seconds
  const cvc::physics::world_snapshot snap = w.snapshot();
  double wet = 0.0;
  if (snap.vehicles.empty()) {
    wet = d.model.wetness(t, d.model.params().cx, d.model.params().cy, d.soak, d.dry);
  } else {
    for (const cvc::physics::vehicle_snapshot &v : snap.vehicles)
      wet += d.model.wetness(t, v.state.body.position.x, v.state.body.position.y, d.soak, d.dry);
    wet /= static_cast<double>(snap.vehicles.size());
  }
  wet = std::round(wet * 64.0) / 64.0;
  if (wet == d.applied)
    return;
  d.applied = wet;
  cvc::physics::apply_wetness(w, wet);
}

void WeatherFriction::world_gone() {
  impl &d = *_impl;
  d.period = kNever;
  d.applied = -1.0;
}

void WeatherFriction::publish() {
  impl &d = *_impl;
  if (!d.app || d.applied < 0.0 || d.applied == d.published)
    return;
  d.published = d.applied;
  write<double>(*d.app, d.wetnessKey, d.applied);
}

double WeatherFriction::applied() const { return _impl->applied; }
double WeatherFriction::gust_front() const { return _impl->gust; }
const cvc::world::weather &WeatherFriction::model() const { return _impl->model; }
std::uint64_t WeatherFriction::revision() const { return _impl->revision; }
const std::string &WeatherFriction::events_text() const { return _impl->eventsText; }
std::uint64_t WeatherFriction::gust_from() const { return _impl->gustFrom; }

bool WeatherFriction::follow(const std::string &events, double gust, std::uint64_t gust_from) {
  impl &d = *_impl;
  bool changed = false;
  d.following = true; // the steering is the followed host's to log (F4)
  d.steerFrom = d.rateFrom = kNever;
  if (!events.empty() && events != d.eventsText) {
    std::vector<cvc::world::weather_event> ev;
    if (cvc::world::parse_weather_events(events, ev)) {
      d.model.set_events(std::move(ev));
      d.eventsText = events;
      d.period = kNever; // the storms' history changed: the wetness again at the next step
      if (d.app)
        write<std::string>(*d.app, d.eventsKey, events);
      changed = true;
    }
  }
  gust = std::max(0.0, gust);
  if (gust != d.gust || gust != d.gustPending) {
    d.gust = d.gustPending = gust;
    d.gustFrom = gust_from;
    d.params();
    if (d.app && !d.gustKey.empty())
      write<double>(*d.app, d.gustKey, gust);
    changed = true;
  }
  if (changed)
    ++d.revision;
  return changed;
}

// ── the fronts where each thing is (F4) ──────────────────────────────────────────────────────────

void scene_fronts(const cvc::physics::flow_velocity *air, double clock_t,
                  cvc::ocean::sea_fronts &out) {
  out.cells.clear();
  out.ux = out.uy = out.ref = 0.0;
  const auto *storm = dynamic_cast<const StormWindFlow *>(air);
  if (!storm || storm->weathers().empty())
    return;
  const cvc::algo::wind_state &w = storm->wind();
  out.ux = w.speed_mps * cvc::algo::det::cos(w.dir_rad);
  out.uy = w.speed_mps * cvc::algo::det::sin(w.dir_rad);
  cvc::algo::front_cell f;
  for (const cvc::world::weather &m : storm->weathers())
    for (const cvc::world::storm_cell &c : m.at(clock_t).cells)
      if (m.front_cell_of(c, f))
        out.cells.push_back(f);
  // The strongest, in the order the weathers list them among equals.
  std::stable_sort(
      out.cells.begin(), out.cells.end(),
      [](const cvc::algo::front_cell &a, const cvc::algo::front_cell &b) { return a.out > b.out; });
  if (out.cells.size() > kSceneFrontCells)
    out.cells.resize(kSceneFrontCells);
  const cvc::world::weather_params &p = storm->weathers().front().params();
  double vx = out.ux, vy = out.uy;
  for (const cvc::algo::front_cell &c : out.cells)
    cvc::algo::add_gust_front(c, p.cx, p.cy, vx, vy);
  out.ref = std::sqrt(vx * vx + vy * vy);
}

namespace {
using air_slot = std::shared_ptr<std::shared_ptr<const cvc::physics::flow_velocity>>;

class SceneSeaFronts final : public cvc::ocean::sea_front_source {
public:
  explicit SceneSeaFronts(air_slot slot) : _slot(std::move(slot)) {}
  std::shared_ptr<const cvc::ocean::sea_fronts> at(double t) const override {
    const std::shared_ptr<const cvc::physics::flow_velocity> air = _slot ? *_slot : nullptr;
    const auto *storm = dynamic_cast<const StormWindFlow *>(air.get());
    if (!storm)
      return nullptr; // no storms: no fronts
    std::lock_guard<std::mutex> lock(_mutex);
    if (_last && _air == air && _t == t)
      return _last;
    auto f = std::make_shared<cvc::ocean::sea_fronts>();
    scene_fronts(storm, t + storm->clock_offset(), *f);
    _air = air;
    _t = t;
    _last = std::move(f);
    return _last;
  }

private:
  air_slot _slot;
  mutable std::mutex _mutex;
  mutable std::shared_ptr<const cvc::physics::flow_velocity> _air;
  mutable double _t = 0.0;
  mutable std::shared_ptr<const cvc::ocean::sea_fronts> _last;
};
} // namespace

std::shared_ptr<const cvc::ocean::sea_front_source> scene_sea_fronts(air_slot slot) {
  return std::make_shared<const SceneSeaFronts>(std::move(slot));
}

// ── StormWindFlow ────────────────────────────────────────────────────────────────────────────────

struct StormWindFlow::moment {
  double t = 0.0;
  cvc::algo::gust_frame gusts;
  std::vector<cvc::world::weather_now> now; // per weather
};

StormWindFlow::StormWindFlow(const cvc::algo::wind_state &w, double base_z,
                             std::vector<cvc::world::weather> weathers, double clock_offset)
    : _w(w), _base(base_z), _offset(clock_offset), _weathers(std::move(weathers)) {
  if (!std::isfinite(w.speed_mps) || !std::isfinite(w.dir_rad) || !std::isfinite(w.gustiness) ||
      !std::isfinite(base_z) || !std::isfinite(clock_offset) || w.speed_mps < 0.0 ||
      w.gustiness < 0.0)
    throw std::invalid_argument(
        "StormWindFlow: the wind must be finite, its speed and gustiness >= 0");
  // As wind_flow has them, so the wind without a front is wind_flow's bit for bit.
  _cx = cvc::algo::det::cos(w.dir_rad);
  _cy = cvc::algo::det::sin(w.dir_rad);
  std::uint64_t h = 1469598103934665603ull;
  auto add = [&h](const void *p, std::size_t n) {
    const auto *b = static_cast<const unsigned char *>(p);
    for (std::size_t i = 0; i < n; ++i)
      h = (h ^ b[i]) * 1099511628211ull;
  };
  const char *tag = "cvc::ariadne::StormWindFlow v1";
  add(tag, std::strlen(tag));
  const double v[5] = {w.speed_mps, w.dir_rad, w.gustiness, base_z, clock_offset};
  add(v, sizeof v);
  for (const cvc::world::weather &m : _weathers) {
    const cvc::world::weather_params &p = m.params();
    add(&p.seed, sizeof p.seed);
    const double q[] = {p.cx,          p.cy,           p.area,        p.interval,     p.chance,
                        p.quiet,       p.life_min,     p.life_max,    p.growth,       p.decay,
                        p.radius_min,  p.radius_max,   p.peak_min,    p.peak_max,     p.miss,
                        p.steer_speed, p.steer_az,     p.rate,        p.rain_onset,   p.rain_max,
                        p.calm_s,      p.flash_rate,   p.flash_onset, p.ground_share, p.front_speed,
                        p.front_ahead, p.front_behind, p.front_width};
    add(q, sizeof q);
    const std::string ev = cvc::world::format_weather_events(m.events());
    const std::uint64_t n = ev.size();
    add(&n, sizeof n);
    add(ev.data(), ev.size());
  }
  _digest = h;
}

std::shared_ptr<const StormWindFlow::moment> StormWindFlow::at(double t) const {
  {
    std::lock_guard<std::mutex> lock(_mutex);
    if (_last && std::memcmp(&_last->t, &t, sizeof t) == 0)
      return _last;
  }
  auto m = std::make_shared<moment>();
  m->t = t;
  m->gusts = cvc::algo::gust_frame_at(_w, t);
  m->now.reserve(_weathers.size());
  for (const cvc::world::weather &w : _weathers)
    m->now.push_back(w.at(t + _offset)); // the clock's time: the storms drawn
  std::lock_guard<std::mutex> lock(_mutex);
  _last = m;
  return m;
}

void StormWindFlow::front(double x, double y, double t, double &vx, double &vy) const {
  vx = vy = 0.0;
  if (_weathers.empty())
    return;
  const std::shared_ptr<const moment> m = at(t);
  for (std::size_t k = 0; k < _weathers.size(); ++k) {
    double fx, fy;
    _weathers[k].gust_front(m->now[k], x, y, fx, fy);
    vx += fx;
    vy += fy;
  }
}

cvc::physics::vec3 StormWindFlow::reference(const cvc::physics::vec3 &p, double t) const {
  const std::shared_ptr<const moment> m = at(t);
  double fx = 0.0, fy = 0.0;
  for (std::size_t k = 0; k < _weathers.size(); ++k) {
    double x, y;
    _weathers[k].gust_front(m->now[k], p.x, p.y, x, y);
    fx += x;
    fy += y;
  }
  const double g = cvc::algo::wind_gust(m->gusts, p.x, p.y);
  // wind_flow's arithmetic, then the front: with no front the sum adds nothing.
  cvc::physics::vec3 r;
  if (_w.speed_mps > 0.0) {
    const double s = _w.speed_mps * g;
    r = {s * _cx, s * _cy, 0.0};
  }
  if (fx != 0.0 || fy != 0.0) {
    r.x += g * fx;
    r.y += g * fy;
  }
  return r;
}

cvc::physics::vec3 StormWindFlow::velocity(const cvc::physics::vec3 &p, double t) const {
  const double prof = profile(p.z);
  if (!(prof > 0.0))
    return {};
  const cvc::physics::vec3 r = reference(p, t);
  return {r.x * prof, r.y * prof, 0.0};
}

double StormWindFlow::profile(double z) const { return cvc::algo::wind_profile(z - _base); }

std::uint64_t StormWindFlow::digest() const { return _digest; }

} // namespace ariadne
} // namespace cvc
