/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The world half of a document's scene: its physics world (`scene.physics:`) and every node's
// `body:`, with no renderer. A headless host (ari, ari test, pycvc's AriApp, arix, a gym worker)
// runs a scene's world through this; cvcGL builds the same world through it and binds its nodes to
// the bodies, so one document runs the same world drawn or not (ARIADNE_WORLDS.md section 14,
// "Headless first").
//
//   auto w = SceneWorld::from_scene(app, scene, "physics", &warnings); // the bodies, built
//   SceneClockDriver clock(app, scene.clock);                          // the scene's clock
//   ... per frame: clock.drive(wall_dt); w->follow(app.world_clock());
//
// The world steps on the app's world clock, never on frames, so N quanta give one state whether a
// window draws them or not.
//
// Each step publishes its telemetry under `<stats>.stats` (stats_key, "physics" by default,
// resolved against the host's bind prefix):
//   tick, time            the world's tick and its time in seconds
//   step_ms               the wall time of the last step
//   bodies                the bodies in the world
//   active_bodies         the ones awake (moving)
//   contact_events        the last step's contact events
//   saturated             an engine buffer filled and contacts were dropped

#ifndef __CVC_ARIADNE_SCENE_WORLD_H__
#define __CVC_ARIADNE_SCENE_WORLD_H__

#include <array>
#include <chrono>
#include <cstdint>
#include <cvc/core/world_clock.h>
#include <cvc/physics/types.h>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace cvc {

class app;
class geometry;
class height_field;
class world_clock_binding;
struct world_clock_keys;

namespace physics {
class world;
}

namespace ariadne {

struct Scene;
struct SceneClock;
struct SceneNode;
struct ScenePhysics;

// Whether a scene has a world to run: a `physics:` block, or a node (at any depth) with a `body:`
// (a data node's bodies come with its dataset, so it does not count).
bool scene_has_world(const Scene &scene);

// Every node's world matrix (row-major 4x4) from the scene alone: its local transform composed with
// its ancestors', by node. The pointers are into `scene`.
void scene_world_matrices(const Scene &scene,
                          std::map<const SceneNode *, std::array<double, 16>> &out);

// The clock keys a scene's `clock:` block names (sim.speed, sim.paused, ...), as given.
cvc::world_clock_keys scene_clock_keys(const SceneClock &clock);

// Drives the app's world clock for a scene that runs on it: a `clock:` block, or a physics world
// (which runs as if it declared `clock: {}`). The clock's keys must already be resolved against the
// host's bind prefix. The declared rate and pause go on the clock when the first drive() binds it.
class SceneClockDriver {
public:
  SceneClockDriver(cvc::app &app, const SceneClock &resolved);
  ~SceneClockDriver();
  SceneClockDriver(const SceneClockDriver &) = delete;
  SceneClockDriver &operator=(const SceneClockDriver &) = delete;

  // Pull the clock's keys (speed, pause, mode, step requests), advance by `wall_dt` seconds (< 0:
  // the real time since the last call, 0 on the first), push its time and tick.
  cvc::world_clock::step_result drive(double wall_dt);

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

// A body the world builds from a node: where the node is and the mesh it has.
struct SceneBodyInput {
  const SceneNode *spec = nullptr;
  std::array<double, 16> world{};                // the node's world matrix, row-major
  std::shared_ptr<const cvc::geometry> geometry; // the node's mesh (its source:, fitted)
};

// A body the world made, and the node it came from.
struct SceneWorldBody {
  std::string node;
  physics::body_id id;
  bool moves = false; // dynamic or kinematic; a static body never moves
  // Where the node is drawn relative to the body: body pose x body_from_visual (row-major).
  std::array<double, 16> body_from_visual{};
};

class SceneWorld {
public:
  // The world of `scene` from its spec alone: the node transforms composed, each body node's mesh
  // read from its source: (a primitive, a heightfield, a file or an asset), its heightfield's grid
  // for a terrain. `stats_key` is the absolute state path the telemetry goes under ("" = none).
  // Null when the scene has no world or its world could not be made (it warns).
  static std::unique_ptr<SceneWorld> from_scene(cvc::app &app, const Scene &scene,
                                                const std::string &stats_key,
                                                std::vector<std::string> *warnings);

  // The world from bodies a renderer already holds: their nodes, world matrices and meshes, and
  // the heightfields by node id for terrain bodies. Problems warn, naming the node, and leave that
  // body out. Null only when the physics world itself could not be made.
  static std::unique_ptr<SceneWorld>
  build(cvc::app &app, const ScenePhysics &spec, const std::vector<SceneBodyInput> &bodies,
        const std::map<std::string, std::shared_ptr<const cvc::height_field>> &fields,
        const std::string &stats_key, std::vector<std::string> *warnings);

  ~SceneWorld();
  SceneWorld(const SceneWorld &) = delete;
  SceneWorld &operator=(const SceneWorld &) = delete;

  physics::world &world();
  const physics::world &world() const;
  const std::vector<SceneWorldBody> &bodies() const;
  // The body node `id` declared (an invalid id when it has none).
  physics::body_id body(const std::string &id) const;

  // Run the steps due on `clock` since the last call and publish the telemetry when any ran. The
  // first call anchors at the clock's tick and runs none. `after_step` is called after the last two
  // steps of a batch (and once on the first call), where a view captures the poses it draws
  // between (pose_tracker::capture). Returns the steps run.
  std::uint64_t follow(const cvc::world_clock &clock,
                       const std::function<void()> &after_step = nullptr);

  // The telemetry, written now.
  void publish_stats();

private:
  SceneWorld();
  struct impl;
  std::unique_ptr<impl> _impl;
};

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_SCENE_WORLD_H__
