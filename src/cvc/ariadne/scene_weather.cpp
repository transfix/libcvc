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
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_weather.h>
#include <cvc/core/app.h>
#include <cvc/geometry/height_field.h>
#include <cvc/physics/world.h>
#include <cvc/physics/world_geometry.h> // apply_wetness
#include <limits>
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

  void params() { model.set_params(scene_weather_params(node, field.get())); }
  std::string key(const std::string &prefix, const char *k) const {
    return resolve_bind(prefix, node_key(node, k));
  }
};

WeatherFriction::WeatherFriction(cvc::app *app, const SceneNode &n, const std::string &bind_prefix)
    : _impl(std::make_unique<impl>()) {
  impl &d = *_impl;
  d.app = app;
  d.node = n;
  d.node.children.clear();
  d.ground = scene_weather_ground(n);
  d.soak = scene_weather_soak(n);
  d.dry = scene_weather_dry(n);
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
}

WeatherFriction::~WeatherFriction() = default;

const std::string &WeatherFriction::node() const { return _impl->node.id; }
const std::string &WeatherFriction::ground() const { return _impl->ground; }

void WeatherFriction::set_ground(std::shared_ptr<const cvc::height_field> field) {
  impl &d = *_impl;
  d.field = std::move(field);
  d.params();
  d.period = kNever; // the area moved: the wetness again at the next step
}

bool WeatherFriction::has_ground() const { return _impl->field != nullptr; }

void WeatherFriction::read_keys() {
  impl &d = *_impl;
  if (!d.app)
    return;
  const std::string text = read_or<std::string>(*d.app, d.eventsKey, std::string());
  if (text != d.eventsText) {
    std::vector<cvc::world::weather_event> ev;
    if (cvc::world::parse_weather_events(text, ev)) {
      d.model.set_events(std::move(ev));
      d.period = kNever; // the storms' history changed: the wetness again at the next step
    }
    d.eventsText = text;
  }
  const bool on = read_or<double>(*d.app, d.frictionKey, 1.0) != 0.0;
  if (on != d.on) {
    d.on = on;
    d.flipped = true;
  }
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
const cvc::world::weather &WeatherFriction::model() const { return _impl->model; }

} // namespace ariadne
} // namespace cvc
