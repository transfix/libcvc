/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/host_cli.h>
#include <cvc/ariadne/scene_world.h>
#include <cvc/ariadne/world_gym.h>
#include <cvc/core/app.h>
#include <cvc/core/world_clock.h>
#include <cvc/nav/world_grid.h>
#include <cvc/physics/world.h>
#include <cvc/state/state.h>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

namespace cvc {
namespace ariadne {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kFallen = 5.0; // m below the grid's ground that is out of the world

std::string first_lines(const std::vector<std::string> &lines) {
  std::string out;
  for (std::size_t i = 0; i < lines.size() && i < 3; ++i)
    out += (out.empty() ? "" : "; ") + lines[i];
  return out.empty() ? out : " (" + out + ")";
}

// The distance from (x, y) along `angle` to the first blocked sample of the grid (off the grid
// counts as blocked), marched at half a cell, up to `range`.
float ray_range(const nav::world_grid &g, double x, double y, double angle, double range) {
  const double step = std::max(0.05, 0.5 * g.cell());
  const double dx = std::cos(angle), dy = std::sin(angle);
  for (double s = step; s < range; s += step)
    if (g.occupied(x + s * dx, y + s * dy))
      return static_cast<float>(s);
  return static_cast<float>(range);
}

std::string num(double v) {
  if (!std::isfinite(v))
    return "nan";
  char buf[32];
  std::snprintf(buf, sizeof buf, "%.9g", v);
  return buf;
}

} // namespace

void gym_log::clear() { *this = gym_log(); }

std::vector<double> gym_progress_reward(const gym_observation &before, const std::vector<double> &,
                                        const std::vector<double> &, const gym_observation &after) {
  std::vector<double> r(after.n, 0.0);
  for (std::size_t i = 0; i < after.n && i < before.n; ++i) {
    r[i] = before.goal_dist[i] - after.goal_dist[i];
    if (after.collided[i])
      r[i] -= 1.0;
    if (after.reached[i] && !before.reached[i])
      r[i] += 10.0;
  }
  return r;
}

struct WorldGym::impl {
  gym_options opts;
  std::unique_ptr<cvc::app> app;
  std::unique_ptr<AppRuntime> rt;
  std::vector<std::string> warnings;
  gym_reward reward;
  gym_log log;
  gym_observation obs;
  bool have_episode = false;
  double return_sum = 0.0;
  // Each agent's body and envelope, fixed for the episode (spawn order).
  struct body {
    double length = 0.0, width = 0.0, wheelbase = 0.0, max_steer = 0.0, max_accel = 0.0,
           max_decel = 0.0, max_speed = 0.0;
    bool has_goal = false; // nav@1 drives it to a goal; none parks it at its start
  };
  std::vector<body> bodies;

  SceneWorld &world() const { return *rt->scene_world(); }
  WorldVehicles &vehicles() const { return *rt->scene_world()->vehicles(); }

  void put(const std::string &leaf, const std::string &value) {
    cvc::state::instance (*app)(resolve_bind(opts.prefix, "gym." + leaf)).value(value);
  }

  void take(std::vector<std::string> w) {
    for (std::string &s : w)
      warnings.push_back(std::move(s));
  }

  gym_observation read() const {
    gym_observation o;
    o.episode = obs.episode;
    o.step = obs.step;
    o.range_m = opts.range_m;
    const int rays = std::max(0, opts.rays);
    for (int k = 0; k < rays; ++k)
      o.ray_angles.push_back(-kPi + 2.0 * kPi * k / rays);
    const std::vector<entity_info> ents = vehicles().entities();
    const std::shared_ptr<const nav::world_grid> grid = vehicles().grid();
    o.n = ents.size();
    for (std::size_t i = 0; i < ents.size(); ++i) {
      const entity_info &e = ents[i];
      o.id.push_back(e.id);
      o.plant.push_back(e.spec.plant);
      o.x.push_back(e.x);
      o.y.push_back(e.y);
      o.z.push_back(e.z);
      o.yaw.push_back(e.yaw);
      o.speed.push_back(e.speed);
      o.vx.push_back(e.vx);
      o.vy.push_back(e.vy);
      o.steer.push_back(e.steer);
      o.goal_x.push_back(e.goal_x);
      o.goal_y.push_back(e.goal_y);
      const double dist = std::hypot(e.goal_x - e.x, e.goal_y - e.y);
      o.goal_dist.push_back(dist);
      o.reach.push_back(e.reach);
      o.reached.push_back(e.placed && dist < e.reach ? 1 : 0);
      o.hit_obstacle.push_back(e.obstacle_contacts > 0 ? 1 : 0);
      o.hit_vehicle.push_back(e.vehicle_contact ? 1 : 0);
      o.collided.push_back(o.hit_obstacle.back() || o.hit_vehicle.back() ? 1 : 0);
      const bool off = grid && e.placed &&
                       (e.x < grid->min_x || e.x > grid->max_x || e.y < grid->min_y ||
                        e.y > grid->max_y || e.z < e.ground_z - kFallen);
      o.out.push_back(off ? 1 : 0);
      for (int k = 0; k < rays; ++k)
        o.ranges.push_back(grid && e.placed
                               ? ray_range(*grid, e.x, e.y, e.yaw + o.ray_angles[k], opts.range_m)
                               : 0.0f);
      const body b = i < bodies.size() ? bodies[i] : body{};
      o.length.push_back(b.length);
      o.width.push_back(b.width);
      o.wheelbase.push_back(b.wheelbase);
      o.max_steer.push_back(b.max_steer);
      o.max_accel.push_back(b.max_accel);
      o.max_decel.push_back(b.max_decel);
      o.max_speed.push_back(b.max_speed);
    }
    return o;
  }

  void record(const gym_observation &o, const std::vector<double> *steer,
              const std::vector<double> *accel, const std::vector<double> *reward) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (std::size_t i = 0; i < o.n; ++i) {
      log.episode.push_back(o.episode);
      log.step.push_back(o.step);
      log.id.push_back(o.id[i]);
      log.time.push_back(o.time);
      log.x.push_back(o.x[i]);
      log.y.push_back(o.y[i]);
      log.yaw.push_back(o.yaw[i]);
      log.speed.push_back(o.speed[i]);
      log.steer_cmd.push_back(steer ? (*steer)[i] : nan);
      log.accel_cmd.push_back(accel ? (*accel)[i] : nan);
      log.goal_dist.push_back(o.goal_dist[i]);
      log.reward.push_back(reward ? (*reward)[i] : 0.0);
      log.reached.push_back(o.reached[i]);
      log.collided.push_back(o.collided[i]);
    }
  }

  void publish(bool done) {
    put("episode", std::to_string(obs.episode));
    put("step", std::to_string(obs.step));
    put("time", num(obs.time));
    put("return", num(return_sum));
    put("done", done ? "1" : "0");
    put("agents", std::to_string(obs.n));
  }
};

WorldGym::WorldGym(const std::string &document, gym_options options)
    : _impl(std::make_unique<impl>()) {
  impl &d = *_impl;
  d.opts = std::move(options);
  d.app = std::make_unique<cvc::app>();
  // The app's lazily made singletons, on this thread before a pool thread needs them.
  d.app->exec_scheduler();
  d.app->exec_async_host();
  d.rt = std::make_unique<AppRuntime>(*d.app, d.opts.prefix);
  if (!AppRuntime::have_yaml())
    throw std::runtime_error("WorldGym: this libcvc reads no YAML documents");
  apply_state_sets(
      *d.app, d.opts.prefix,
      std::vector<std::pair<std::string, std::string>>(d.opts.set.begin(), d.opts.set.end()));
  d.take(d.rt->load(document));
  if (!d.rt->scene_world())
    throw std::runtime_error("WorldGym: '" + document +
                             "' runs no world (a scene with physics: and bodies)" +
                             first_lines(d.warnings));
  if (!d.rt->scene_world()->vehicles() || d.vehicles().archetypes().empty())
    throw std::runtime_error("WorldGym: '" + document +
                             "' offers no vehicles (its world: vehicles: archetypes: is empty)");
  d.rt->pause_world(true); // the world steps only when step() asks
  if (!d.opts.world.empty()) {
    std::vector<std::string> w;
    if (!d.world().set_world(d.opts.world, &w))
      throw std::runtime_error("WorldGym: '" + document + "' has no world '" + d.opts.world + "'" +
                               first_lines(w));
    d.take(std::move(w));
  }
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(
                            static_cast<long long>(std::max(0.0, d.opts.ready_timeout) * 1000.0));
  for (;;) {
    d.rt->drain();
    if (d.vehicles().grid())
      break;
    if (std::chrono::steady_clock::now() > deadline)
      throw std::runtime_error("WorldGym: '" + document + "': its world was not ready in " +
                               num(d.opts.ready_timeout) + " s (" + d.vehicles().status() + ")");
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  d.take(d.rt->take_warnings());
  d.put("status", "loaded");
}

WorldGym::~WorldGym() = default;

const gym_observation &WorldGym::reset(std::uint64_t seed) {
  impl &d = *_impl;
  WorldVehicles &v = d.vehicles();
  v.clear();
  if (d.opts.regenerate) { // a generated world of the episode's own, made from its seed
    const std::shared_ptr<const nav::world_grid> before = v.grid();
    if (d.world().regenerate_world(static_cast<double>(seed))) {
      const auto deadline =
          std::chrono::steady_clock::now() + std::chrono::milliseconds(static_cast<long long>(
                                                 std::max(0.0, d.opts.ready_timeout) * 1000.0));
      for (;;) {
        d.rt->drain();
        if (d.vehicles().grid() && d.vehicles().grid() != before)
          break;
        if (std::chrono::steady_clock::now() > deadline)
          throw std::runtime_error("WorldGym::reset: the world was not made again in " +
                                   num(d.opts.ready_timeout) + " s (" + v.status() + ")");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }
  }
  d.rt->restart_world(); // the clock at tick 0, the world as the document built it
  v.reseed(seed);
  v.set_wander(d.opts.wander);
  std::vector<entity_spec> agents = d.opts.agents;
  if (agents.empty()) {
    const vehicle_archetype &a = v.archetypes().front();
    const bool nav =
        std::find(a.controllers.begin(), a.controllers.end(), "nav@1") != a.controllers.end();
    for (int i = 0; i < std::max(1, d.opts.count); ++i) {
      entity_spec s;
      s.archetype = a.id;
      s.name = "agent " + std::to_string(i + 1);
      s.controller = nav ? "nav@1" : std::string();
      agents.push_back(s);
    }
  }
  d.bodies.clear();
  for (entity_spec &s : agents) {
    if (s.archetype.empty()) // the document's first vehicle
      s.archetype = v.archetypes().front().id;
    std::string why;
    if (v.spawn(s, &why) == 0)
      throw std::runtime_error("WorldGym::reset: agent '" + s.name + "' (" + s.archetype +
                               "): " + why);
  }
  for (const entity_info &e : v.entities()) {
    impl::body b;
    b.has_goal = e.spec.controller == "nav@1";
    for (const vehicle_archetype &a : v.archetypes())
      if (a.id == e.spec.archetype) {
        const physics::vehicle_desc desc = vehicle_desc_of(a, e.spec.mass, e.spec.max_speed);
        b.length = 2.0 * desc.half_extents.x;
        b.width = 2.0 * desc.half_extents.y;
        b.wheelbase = desc.wheelbase();
        b.max_steer = desc.max_steer;
        b.max_accel = desc.max_accel;
        b.max_decel = desc.max_decel;
        b.max_speed = desc.max_speed;
      }
    d.bodies.push_back(b);
  }
  d.rt->drain(); // the world anchors on the clock (no step runs: it is paused)
  d.take(d.rt->take_warnings());
  const std::uint64_t episode = d.obs.episode + 1;
  d.obs.episode = episode;
  d.obs.step = 0;
  d.obs = d.read();
  d.obs.time = 0.0;
  d.have_episode = true;
  d.return_sum = 0.0;
  d.record(d.obs, nullptr, nullptr, nullptr);
  d.publish(false);
  d.put("status", "episode " + std::to_string(episode) + ", seed " + std::to_string(seed));
  return d.obs;
}

gym_step WorldGym::step(const std::vector<double> &steer, const std::vector<double> &accel) {
  impl &d = *_impl;
  if (!d.have_episode)
    throw std::logic_error("WorldGym::step: reset() first");
  if (steer.size() != d.obs.n || accel.size() != d.obs.n)
    throw std::invalid_argument("WorldGym::step: " + std::to_string(d.obs.n) + " agents, but " +
                                std::to_string(steer.size()) + " steers and " +
                                std::to_string(accel.size()) + " accelerations");
  WorldVehicles &v = d.vehicles();
  for (std::size_t i = 0; i < d.obs.n; ++i)
    v.command(d.obs.id[i], steer[i], accel[i]);
  // One navigator tick: its physics steps, as quanta of the clock.
  const physics::world &w = d.world().world();
  const std::uint64_t target =
      w.tick() + static_cast<std::uint64_t>(std::max(1, v.steps_per_tick()));
  const double quantum = d.app->world_clock().fixed_dt();
  const std::uint64_t quanta =
      quantum > 0.0 ? static_cast<std::uint64_t>(std::max(1.0, std::round(dt() / quantum))) : 1;
  d.rt->step_world(quanta);
  for (int tries = 0; tries < 100 && d.world().world().tick() < target; ++tries)
    d.rt->drain();
  if (d.world().world().tick() < target)
    throw std::runtime_error("WorldGym::step: the world did not step (" + v.status() + ")");
  d.take(d.rt->take_warnings());

  gym_step out;
  const gym_observation before = d.obs;
  gym_observation after = d.read();
  after.episode = before.episode;
  after.step = before.step + 1;
  after.time = static_cast<double>(after.step) * dt();
  // The world has stepped: a reward that throws (or gives the wrong count) leaves the gym at the
  // new tick, its log row without a reward, and the error goes on to the caller.
  try {
    out.reward = d.reward ? d.reward(before, steer, accel, after)
                          : gym_progress_reward(before, steer, accel, after);
    if (out.reward.size() != after.n)
      throw std::runtime_error("WorldGym::step: the reward gave " +
                               std::to_string(out.reward.size()) + " values for " +
                               std::to_string(after.n) + " agents");
  } catch (...) {
    d.obs = after;
    d.record(d.obs, &steer, &accel, nullptr);
    d.publish(false);
    throw;
  }
  for (const double r : out.reward)
    d.return_sum += r;
  bool goals = false, arrived = true, gone = false;
  for (std::size_t i = 0; i < after.n; ++i) {
    if (i < d.bodies.size() && d.bodies[i].has_goal) {
      goals = true;
      arrived = arrived && after.reached[i] != 0;
    }
    gone = gone || after.out[i] != 0;
  }
  out.terminated = (!d.opts.wander && goals && arrived) || gone;
  out.truncated = d.opts.max_steps > 0 && after.step >= d.opts.max_steps;
  d.obs = after;
  d.record(d.obs, &steer, &accel, &out.reward);
  d.publish(out.terminated || out.truncated);
  out.observation = d.obs;
  return out;
}

const gym_observation &WorldGym::observe() const { return _impl->obs; }

bool WorldGym::set_goal(std::uint64_t id, double x, double y) {
  impl &d = *_impl;
  if (!d.vehicles().set_goal(id, x, y))
    return false;
  gym_observation &o = d.obs;
  for (std::size_t i = 0; i < o.n; ++i)
    if (o.id[i] == id) {
      o.goal_x[i] = x;
      o.goal_y[i] = y;
      o.goal_dist[i] = std::hypot(x - o.x[i], y - o.y[i]);
      o.reached[i] = o.goal_dist[i] < o.reach[i] ? 1 : 0;
    }
  return true;
}

void WorldGym::set_reward(gym_reward reward) { _impl->reward = std::move(reward); }

const gym_log &WorldGym::log() const { return _impl->log; }
void WorldGym::clear_log() { _impl->log.clear(); }

bool WorldGym::save_log(const std::string &path) const {
  std::ofstream f(path, std::ios::binary);
  if (!f)
    return false;
  const gym_log &l = _impl->log;
  f << "episode,step,id,time,x,y,yaw,speed,steer_cmd,accel_cmd,goal_dist,reward,reached,"
       "collided\n";
  for (std::size_t i = 0; i < l.size(); ++i)
    f << l.episode[i] << ',' << l.step[i] << ',' << l.id[i] << ',' << num(l.time[i]) << ','
      << num(l.x[i]) << ',' << num(l.y[i]) << ',' << num(l.yaw[i]) << ',' << num(l.speed[i]) << ','
      << num(l.steer_cmd[i]) << ',' << num(l.accel_cmd[i]) << ',' << num(l.goal_dist[i]) << ','
      << num(l.reward[i]) << ',' << int(l.reached[i]) << ',' << int(l.collided[i]) << '\n';
  return static_cast<bool>(f);
}

double WorldGym::dt() const {
  const double t = _impl->vehicles().tick_seconds();
  return t > 0.0 ? t : 0.05;
}

std::size_t WorldGym::agents() const { return _impl->obs.n; }
double WorldGym::episode_return() const { return _impl->return_sum; }

std::vector<std::string> WorldGym::take_warnings() {
  std::vector<std::string> out;
  out.swap(_impl->warnings);
  for (std::string &w : _impl->rt->take_warnings())
    out.push_back(std::move(w));
  return out;
}

cvc::app &WorldGym::app() { return *_impl->app; }
AppRuntime &WorldGym::runtime() { return *_impl->rt; }
SceneWorld &WorldGym::world() { return _impl->world(); }
WorldVehicles &WorldGym::vehicles() { return _impl->vehicles(); }

} // namespace ariadne
} // namespace cvc
