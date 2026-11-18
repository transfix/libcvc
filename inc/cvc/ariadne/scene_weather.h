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
//
// The storms' gust fronts (O20, F4; F6's gusts) are physics too: in a scene with a wind and a
// weather node, the world's air is a StormWindFlow, the scene's wind with every cell's outflow in
// it, a function of the point and the time that each body reads at the step's own time. So a front
// sweeps over the bodies as its storm walks, the same at every frame rate and headless.

#ifndef __CVC_ARIADNE_SCENE_WEATHER_H__
#define __CVC_ARIADNE_SCENE_WEATHER_H__

#include <cstdint>
#include <cvc/algo/wind_field.h>
#include <cvc/physics/flow_velocity.h>
#include <cvc/world/weather.h>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace cvc {
class app;
class height_field;
namespace physics {
class world;
}
namespace ariadne {

struct Scene;
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

// The cloud deck's base the storms' steering is read at. The drawn weather node reads its sky's
// `<sky>.base` key (kWeatherDeckBase, 300 m, without one), which the cloud_sky whose keys those
// are seeds from its `base` prop (cvcGL's cloud_sky defaults it to kCloudSkyBase).
constexpr double kCloudSkyBase = 74.0;
constexpr double kWeatherDeckBase = 300.0;
// The same headless, from the document: the `base` of the scene's cloud_sky (at any depth) whose
// key root is the one the weather node's `sky:` names (default `sky`; a cloud_sky's root is its
// `keys:`, default `sky`), else kWeatherDeckBase. So a drawn and a headless run steer alike.
// The storms' steering in a scene with a wind (O14, F4): the scene's wind as authored
// (scene_wind_set, no gust front: a storm is not carried by its own outflow), its heading (degrees,
// as weather_params::steer_az: dir + 90) at its speed at the deck's base `deck_base` (m, the
// profile the physics reads) times `drift`, which is how the cloud deck drifts without its gusts'
// pulse, so the storms keep to the clouds drawn. The drawn weather node steers by it, and the
// weather starts from it, drawn or headless (WeatherFriction), so both steer alike. False, with
// speed and az untouched, when the scene has no wind.
bool scene_weather_steering(cvc::app &app, const std::string &bind_prefix, double deck_base,
                            double drift, double &speed, double &az);
double scene_weather_deck_base(const Scene &scene, const SceneNode &weather);
// The key root of the sky a weather node drives (its `sky:`, default and when empty `sky`).
std::string scene_weather_sky(const SceneNode &weather);

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

  // `app` may be null (no keys: the props' events and the friction on). With a scene wind, the
  // storms start steered by it at the deck's base `deck_base` (scene_weather_steering).
  WeatherFriction(cvc::app *app, const SceneNode &n, const std::string &bind_prefix,
                  double deck_base = kWeatherDeckBase);
  ~WeatherFriction();
  WeatherFriction(const WeatherFriction &) = delete;
  WeatherFriction &operator=(const WeatherFriction &) = delete;

  const std::string &node() const;
  const std::string &ground() const;
  // The ground's height field, which sets the weather's area when the props give none. Until it is
  // given, the props' area is used.
  void set_ground(std::shared_ptr<const cvc::height_field> field);
  bool has_ground() const;

  // The keys (the frame's thread, between steps): weather.events, weather.friction and
  // weather.gust_front (the storm wind's peak, seeded from the `gust_front` prop).
  void read_keys();
  // Before the physics step that ends on the world clock's tick `tick` (whose quantum is `dt`):
  // the friction on the first step of each kWetPeriod, or at once when weather.friction changed.
  void before_step(cvc::physics::world &w, std::uint64_t tick, double dt);
  // Before that step too: a new storm-wind peak (weather.gust_front) goes into model() on the
  // first step past the start of the clock period after the one read_keys saw it in, so at the same
  // step whatever the frames. True when it did (the scene's world makes its air again).
  bool switch_gust(std::uint64_t tick);
  // The world was made again (sim.restart): its materials are dry, so set them at the next step.
  void world_gone();
  // weather.wetness, when it changed.
  void publish();

  // The wetness last applied (0 dry; -1 before any).
  double applied() const;
  // The storm wind's peak model() has (m/s): weather.gust_front as last switched in, else the
  // node's `gust_front` prop (12; 0: no gust fronts).
  double gust_front() const;
  const cvc::world::weather &model() const;
  // Counts the changes to model() (its events, or its area once the ground came): what the scene's
  // air (StormWindFlow) is made again on.
  std::uint64_t revision() const;

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

// The scene's air with the storms in it (O20, F4; F6's gusts push light bodies): the scene's wind
// (a cvc::physics::wind_flow's: the mean wind times the gusts, times the height profile over
// base_z) with each weather's gust fronts (cvc::world::weather::gust_front) added before the gusts
// and the profile, so a front gusts and thins toward the ground as the wind does:
//   v(p, t) = g(x, y, t) (U d + front(x, y, t)) wind_profile(z - base_z).
// A pure function of the point and the time, the weathers being pure functions of the time, so it
// is immutable: a new one when the wind or a weather's events change. Each body reads it at its
// centre of mass at the step's own time (the world's field time), so the fronts sweep over the
// bodies as the storms walk, at any frame rate. The weathers are read at that time plus
// `clock_offset`, the world clock's time less the world's field time (the scene's world sets it
// where the world anchored on the clock), so they are the clock's storms, the ones drawn; the
// gusts are read as wind_flow reads them. Where no front reaches, it is the wind_flow's velocity
// bit for bit. Its digest names the wind, the base, the offset and each weather's parameters and
// events.
class StormWindFlow : public cvc::physics::flow_velocity {
public:
  // Throws std::invalid_argument for a non-finite value or a negative speed or gustiness.
  StormWindFlow(const cvc::algo::wind_state &w, double base_z,
                std::vector<cvc::world::weather> weathers, double clock_offset = 0.0);
  cvc::physics::vec3 velocity(const cvc::physics::vec3 &p, double t) const override;
  std::uint64_t digest() const override;
  cvc::physics::vec3 reference(const cvc::physics::vec3 &p, double t) const override;
  double profile(double z) const override;
  double base() const override { return _base; }

  // The fronts alone at (x, y) at field time t, m/s along x and y (the sum over the weathers).
  void front(double x, double y, double t, double &vx, double &vy) const;
  double clock_offset() const { return _offset; }
  const std::vector<cvc::world::weather> &weathers() const { return _weathers; }
  const cvc::algo::wind_state &wind() const { return _w; }

private:
  struct moment; // the gusts and each weather's cells at one time
  std::shared_ptr<const moment> at(double t) const;
  cvc::algo::wind_state _w;
  double _base, _offset, _cx, _cy;
  std::vector<cvc::world::weather> _weathers;
  std::uint64_t _digest;
  // The last time read (a cache: a pure function of t, so it changes no value).
  mutable std::mutex _mutex;
  mutable std::shared_ptr<const moment> _last;
};

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_SCENE_WEATHER_H__
