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

#ifndef __CVC_ARIADNE_WORLD_GYM_H__
#define __CVC_ARIADNE_WORLD_GYM_H__

#include <cstdint>
#include <cvc/ariadne/world_vehicles.h> // entity_spec
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
class app;
namespace ariadne {

class AppRuntime;
class SceneWorld;

// ----------------
// The gym: a world document driven by reset, step and observe (ARIADNE_WORLDS.md §14, L4)
// ----------------
// A world document (its scene's worlds, physics and bodies, its world: vehicles: and spawns:)
// run headless, with no GL and no window, and driven one control tick at a time:
//
//   cvc::ariadne::WorldGym gym("world_lab.ari", opts);   // loads it on an app of its own
//   gym_observation o = gym.reset(7);                    // the agents spawned, seeded
//   while (true) {
//     gym_step s = gym.step(steer, accel);               // a tick of every agent's command
//     if (s.terminated || s.truncated) break;
//   }
//   gym.save_log("episode.csv");
//
// The agents are the world's vehicles (WorldVehicles), each a Jolt vehicle in the scene's own
// physics world. A step gives each its command for one navigator tick (0.05 s, a whole number of
// physics steps): the steering's bicycle angle (rad, + left) and the acceleration along its
// heading (m/s^2). A command that is not finite leaves that agent to its controller for the tick
// (nav@1, the navigator, drives it to its goal; none holds it still), so NaN everywhere is the
// navigator's baseline.
//
// The world steps on its clock, paused, only by step(): N steps from a reset with one seed and
// one list of commands give the same world on every run. Each gym has its own app (state tree,
// clock, physics world), so one process runs many side by side; the same document opens in
// arigl to watch one.
//
// Keys, under `<prefix>.gym`: episode, step, time, return (the reward summed over the agents and
// the episode), done (1 once terminated or truncated), agents, status.

// Every agent at one tick, as flat arrays of `n` (SI: m, rad, m/s), in spawn order.
struct gym_observation {
  std::uint64_t episode = 0; // resets so far
  std::uint64_t step = 0;    // steps since the reset
  double time = 0.0;         // s since the reset
  std::size_t n = 0;
  std::vector<std::uint64_t> id; // the entity ids (WorldVehicles)
  std::vector<std::string> plant; // what drives each: jolt, kinematic, dynamic or dynamic4w
  // The chassis's centre and heading (0 along +x, counter-clockwise), its speed along the heading
  // (negative backing up), its velocity in the world, the steering's angle now.
  std::vector<double> x, y, z, yaw, speed, vx, vy, steer;
  // Its goal, the distance to it, and within `reach` of it.
  std::vector<double> goal_x, goal_y, goal_dist, reach;
  std::vector<std::uint8_t> reached;
  // Over the last tick: touching the world's bodies other than its ground (hit_obstacle), touching
  // another vehicle (hit_vehicle), and either (collided).
  std::vector<std::uint8_t> collided, hit_obstacle, hit_vehicle;
  // Out of the world: off its grid, or more than 5 m below the grid's ground under it (fallen off
  // an edge, or through the ground). The episode ends.
  std::vector<std::uint8_t> out;
  // The distance to anything blocked in the world's grid along each ray from the chassis's centre
  // (n x rays, row-major), at `ray_angles` off the heading, up to `range_m`.
  std::vector<float> ranges;
  std::vector<double> ray_angles;
  double range_m = 0.0;
  // Each agent's body and envelope: the chassis's length and width, its wheelbase, the steering's
  // lock (rad), its acceleration and braking (m/s^2), its top speed (m/s; 0 unlimited).
  std::vector<double> length, width, wheelbase, max_steer, max_accel, max_decel, max_speed;
};

// What step() returns.
struct gym_step {
  gym_observation observation;
  std::vector<double> reward; // one per agent
  // Every agent with a goal at it (when goals are held), or one out. An agent whose controller is
  // none has no goal of its own: it parks at its start, which is its goal, and is not counted.
  bool terminated = false;
  bool truncated = false; // the episode reached max_steps
};

// A reward: each agent's, from the observation before a step, the commands it took (as given,
// NaN included) and the observation after.
using gym_reward = std::function<std::vector<double>(
    const gym_observation &before, const std::vector<double> &steer,
    const std::vector<double> &accel, const gym_observation &after)>;

// The default reward: each agent's progress toward its goal over the step (m), less 1 for a tick
// it collided, plus 10 on the step it first reaches its goal.
std::vector<double> gym_progress_reward(const gym_observation &before,
                                        const std::vector<double> &steer,
                                        const std::vector<double> &accel,
                                        const gym_observation &after);

struct gym_options {
  // The document's bind prefix (its keys, <prefix>.vehicles and the rest; "" none).
  std::string prefix;
  // State keys written after the document loads and before its world is first run, as `--set`
  // gives them (a key relative to the prefix: world.seed, scene.world.source, ...).
  std::map<std::string, std::string> set;
  // The scene's world to run ("" the document's).
  std::string world;
  // Each reset makes a generated world on again from the reset's seed (its world.seed), so every
  // episode has a world of its own; the same seed, the same world. Off: one world for all.
  bool regenerate = false;
  // The agents each reset spawns, in this order (a spec's empty archetype: the document's first;
  // its plant: jolt, kinematic, dynamic or dynamic4w, "" the archetype's). Empty: `count` of the
  // document's first vehicle archetype, each with nav@1 when the archetype allows it. A spec
  // without a place starts where the document's spawn points and their policy put it, else at a
  // free point; every draw follows the reset's seed.
  std::vector<entity_spec> agents;
  int count = 1;
  // Whether an agent at its goal gets another (nav@1's wandering). Off: goals are held for the
  // episode, and it ends when every agent is at its own.
  bool wander = false;
  std::uint64_t max_steps = 1200; // steps per episode (60 s at 0.05 s); 0 unlimited
  int rays = 16;
  double range_m = 60.0;
  // How long the constructor waits for the world (a generated world being made), s.
  double ready_timeout = 120.0;
};

// One row per agent per step (and one per agent at each reset, step 0), every episode since the
// gym was made or the log was cleared.
struct gym_log {
  std::vector<std::uint64_t> episode, step, id;
  std::vector<double> time, x, y, yaw, speed, steer_cmd, accel_cmd, goal_dist, reward;
  std::vector<std::uint8_t> reached, collided;
  std::size_t size() const { return step.size(); }
  void clear();
};

class WorldGym {
public:
  // Load `document` headless on an app of its own and wait for its world. Throws
  // std::runtime_error when it cannot load, runs no world, offers no vehicles, or its world is
  // not ready in time; the message says which.
  explicit WorldGym(const std::string &document, gym_options options = {});
  ~WorldGym();
  WorldGym(const WorldGym &) = delete;
  WorldGym &operator=(const WorldGym &) = delete;

  // A new episode: every agent gone, the world rebuilt as the document built it (clock at tick
  // 0; with options.regenerate a generated world made from `seed`), the draws seeded with `seed`,
  // and the options' agents spawned. Throws std::runtime_error when an agent cannot be spawned or
  // the world is not made again in options.ready_timeout.
  const gym_observation &reset(std::uint64_t seed = 0);
  // One control tick: each agent's command (n values each; not finite: its controller drives).
  // Throws std::invalid_argument when the sizes are not the agents', std::logic_error before a
  // reset. A reward that throws, or gives other than n values, throws on from here after the step
  // (the gym is at the new tick).
  gym_step step(const std::vector<double> &steer, const std::vector<double> &accel);
  // The agents now (as the last reset or step left them, with any goal set since).
  const gym_observation &observe() const;
  // Drive agent `id` to (x, y) world metres from the next step (its goal in observe() at once).
  // False when it is not an agent in the world.
  bool set_goal(std::uint64_t id, double x, double y);

  // The reward step() pays (gym_progress_reward by default; null puts it back).
  void set_reward(gym_reward reward);

  const gym_log &log() const;
  void clear_log();
  // The log as CSV, a header and a row per entry. False when the file cannot be written.
  bool save_log(const std::string &path) const;

  // The seconds one step covers (the navigator's tick).
  double dt() const;
  std::size_t agents() const;
  // The reward summed over the agents and the episode's steps so far.
  double episode_return() const;
  // The load's warnings and the runs' since (the world, the document's programs).
  std::vector<std::string> take_warnings();

  cvc::app &app();
  AppRuntime &runtime();
  SceneWorld &world();
  WorldVehicles &vehicles();

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_WORLD_GYM_H__
