/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// scene_weather.h — a `weather` scene node's model as the scene's world needs it
// (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md O18-O20): the weather its props describe, read the
// same way by the drawn node (cvcGL's weather node, which draws the storms, the rain and the
// lightning) and by the scene's world, and the rain's friction (O20, F6), which the world applies.
//
// The friction is physics, so it is the world's: SceneWorld runs a WeatherFriction for each weather
// node, with or without a renderer, and sets it inside the physics step on the world clock's ticks.
// Every host (arigl at any frame rate, an offscreen capture, ari, a headless test) so applies the
// same friction at the same physics step.

#ifndef __CVC_ARIADNE_SCENE_WEATHER_H__
#define __CVC_ARIADNE_SCENE_WEATHER_H__

#include <cstdint>
#include <cvc/world/weather.h>
#include <memory>
#include <string>

namespace cvc {
class app;
class height_field;
namespace physics {
class world;
}
namespace ariadne {

struct SceneNode;

// How long the rain takes to soak the ground and how long it takes to dry by 1/e (weather seconds;
// the node's `soak` and `dry`).
constexpr double kWeatherSoak = 300.0;
constexpr double kWeatherDry = 900.0;

// The weather a `type: weather` node's props describe: its schedule, its time scale, and its area,
// the ground's box when `ground` has a height field (`ground_field`) and the props give no `area`,
// else `area` around (`cx`, `cy`). The props are listed at cvcGL's weather node.
cvc::world::weather_params scene_weather_params(const SceneNode &n,
                                                const cvc::height_field *ground_field);
// The node whose extent the storms serve: its `ground`, default "world".
std::string scene_weather_ground(const SceneNode &n);
// Its soak and dry times (at least 1 s).
double scene_weather_soak(const SceneNode &n);
double scene_weather_dry(const SceneNode &n);

// The rain into the scene's friction (O20, F6). Every half second of world time (kWetPeriod,
// counted in the world clock's ticks), the wetness where the world's vehicles are (their mean; the
// weather's centre with none) runs every surface material's friction toward its wet value
// (cvc::physics::apply_wetness), when it has moved by 1/64. The wetness is the weather's
// (cvc::world::weather::wetness): a function of the node's props, the time and the events in its
// `weather.events`, which the drawn node writes when someone starts a storm or the steering wind
// turns. Its keys, under the node's id: `weather.friction` (1; 0 dries the surfaces at the next
// step, and they are wetted again at the next step after it is set back) and `weather.wetness`,
// the wetness last applied.
class WeatherFriction {
public:
  static constexpr double kWetPeriod = 0.5; // s of world time between two settings

  // `app` may be null (no keys: the props' events and the friction on).
  WeatherFriction(cvc::app *app, const SceneNode &n, const std::string &bind_prefix);
  ~WeatherFriction();
  WeatherFriction(const WeatherFriction &) = delete;
  WeatherFriction &operator=(const WeatherFriction &) = delete;

  const std::string &node() const;
  const std::string &ground() const;
  // The ground's height field, which sets the weather's area when the props give none. Until it is
  // given, the props' area is used.
  void set_ground(std::shared_ptr<const cvc::height_field> field);
  bool has_ground() const;

  // The keys (the frame's thread, between steps): weather.events and weather.friction.
  void read_keys();
  // Before the physics step that ends on the world clock's tick `tick` (whose quantum is `dt`):
  // the friction on the first step of each kWetPeriod, or at once when weather.friction changed.
  void before_step(cvc::physics::world &w, std::uint64_t tick, double dt);
  // The world was made again (sim.restart): its materials are dry, so set them at the next step.
  void world_gone();
  // weather.wetness, when it changed.
  void publish();

  // The wetness last applied (0 dry; -1 before any).
  double applied() const;
  const cvc::world::weather &model() const;

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_SCENE_WEATHER_H__
