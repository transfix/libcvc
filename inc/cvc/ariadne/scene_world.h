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
//   auto w = SceneWorld::from_scene(app, scene, prefix, "physics", &warnings); // the bodies
//   SceneClockDriver clock(app, scene.clock);                                  // the scene's clock
//   ... per frame: clock.drive(wall_dt); w->follow(app.world_clock());
//
// Some nodes make what their body: collides with at run time, and the world swaps those bodies
// when it changes (sync(), which follow() runs first):
//   * a `generated_world` node: its terrain and buildings (scene_generated_world.h). Headless, the
//     world runs the node itself (GeneratedWorldRuntime: its world.* keys and Regenerate);
//     drawn, cvcGL's node does and hands over its colliders.
//   * a `source: { data }` node: its dataset's world, terrain or geometry, once the dataset is
//     ready and again each time it changes (its rev). It needs the scene's physics: block.
// Those colliders are static and placed as they were made: the node's transform does not move
// them.
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
namespace nav {
struct world_grid;
}
namespace world {
struct generated_world;
class world_geometry;
}

namespace ariadne {

struct Scene;
struct SceneClock;
struct SceneNode;
struct ScenePhysics;
struct SceneColliders;

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

// A body the world builds from a node: where the node is and the mesh it has. A node that makes
// its colliders at run time gives `colliders` (a generated world) or `data_path` (a data node's
// dataset, an absolute state path) instead of a mesh.
struct SceneBodyInput {
  const SceneNode *spec = nullptr;
  std::array<double, 16> world{};                // the node's world matrix, row-major
  std::shared_ptr<const cvc::geometry> geometry; // the node's mesh (its source:, fitted)
  std::shared_ptr<SceneColliders> colliders;     // what the node makes, swapped on each revision
  std::string data_path;                         // <prefix>.data.<name>
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
  // for a terrain; each generated_world node run here (its world made now, its world.* keys under
  // `bind_prefix`), its colliders a body: on it; each data node's dataset watched (under
  // `bind_prefix`). `stats_key` is the absolute state path the telemetry goes under ("" = none).
  // Null when the scene has no world or its world could not be made (it warns).
  static std::unique_ptr<SceneWorld> from_scene(cvc::app &app, const Scene &scene,
                                                const std::string &bind_prefix,
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
  // The body node `id` declared (an invalid id when it has none); a node with colliders has a
  // body per collider, and this is the first.
  physics::body_id body(const std::string &id) const;
  // The bodies node `id` has now.
  std::vector<physics::body_id> bodies_of(const std::string &id) const;

  // The world a generated_world node made, when this world runs the node (from_scene); null when a
  // renderer runs it, or there is no such node.
  std::shared_ptr<const cvc::world::generated_world> generated_world(const std::string &id) const;
  // The world node `id` makes at run time (a generated world's, a dataset's) as it is now, and its
  // revision, which each new world bumps (0: none yet). Null for any other node, or before it has
  // one. The bodies take a new world at the next sync().
  std::shared_ptr<const cvc::world::world_geometry>
  world_geometry(const std::string &id, std::uint64_t *revision = nullptr) const;
  // That world as the navigator sees it (cvc::nav::world_grid): `cell` m samples over its terrain,
  // or a square `half` m either side of the terrain's centre. Made once per world revision, cell
  // and half, and kept until the next. Null when the node has no world yet, or it has no terrain
  // (it warns).
  std::shared_ptr<const cvc::nav::world_grid> grid(const std::string &id, double cell,
                                                   double half = 0.0,
                                                   std::vector<std::string> *warnings = nullptr);

  // Run the generated_world nodes this world runs (a request or an edit makes a new world), take a
  // ready or changed dataset's colliders, and swap the bodies of each node whose colliders changed.
  // follow() runs it first. Problems warn, naming the node.
  void sync(std::vector<std::string> *warnings = nullptr);

  // Run the steps due on `clock` since the last call and publish the telemetry when any ran. The
  // first call anchors at the clock's tick and runs none. `after_step` is called after the last two
  // steps of a batch (and once on the first call), where a view captures the poses it draws
  // between (pose_tracker::capture). Returns the steps run.
  std::uint64_t follow(const cvc::world_clock &clock,
                       const std::function<void()> &after_step = nullptr);

  // Rebuild the world as build() made it: the same object, a fresh physics world with the same
  // config and materials, each node's body where the document put it (at rest unless it declared
  // a velocity), and each tracked node's colliders as they are now. Its bodies are made in the
  // order build() made them, so their ids are the same, and stepping it runs as a world built
  // now. The next follow() anchors on the clock again (sim.restart resets the clock to tick 0).
  // Problems warn, naming the node.
  void restart(std::vector<std::string> *warnings = nullptr);

  // The telemetry, written now.
  void publish_stats();

  // The warnings follow()'s syncs gathered since the last call.
  std::vector<std::string> take_warnings();

private:
  SceneWorld();
  struct impl;
  std::unique_ptr<impl> _impl;
};

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_SCENE_WORLD_H__
