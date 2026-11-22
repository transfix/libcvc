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
//   * a `source: { data }` node: its dataset's world, terrain or geometry, once the dataset is
//     ready (at once when it already is) and again each time it changes (its rev). It needs the
//     scene's physics: block.
//   * a `generated_world` node: the same, its dataset the world it made (scene_generated_world.h,
//     publish_generated_world: the world as a version 2 bundle, read back). Headless, the world
//     runs the node itself (GeneratedWorldRuntime: its world.* keys and Regenerate); drawn,
//     cvcGL's node does. Either way each new world replaces the dataset, and the bodies follow.
// Those colliders are static and placed as they were made: the node's transform does not move
// them. A water node's field is swapped the same way when its sea changes, except one with a wake
// (`wake_push`, cvc::physics::field_wake), which is set in place, so it keeps its id and the wake
// its waves.
//
// The world steps on the app's world clock, never on frames, so N quanta give one state whether a
// window draws them or not.
//
// Each step publishes its telemetry under `<stats>.stats` (stats_key, "physics" by default,
// resolved against the host's bind prefix):
//   tick, time            the world's tick and its time in seconds
//   step_ms               the wall time of the last step
//   step_ms_history       the last 120 steps' step_ms, oldest first, comma-separated (a plot:)
//   bodies                the bodies in the world
//   active_bodies         the ones awake (moving)
//   contact_events        the last step's contact events
//   saturated             an engine buffer filled and contacts were dropped
//   nodes                 a line per node with bodies: "<node>  <bodies>  <awake>"
// and `<stats>.enabled` (seeded 1) turns the stepping off and on: off, the steps due are let go,
// so the world holds still while the clock runs on and carries on from there when it is back on.
//
// A scene's worlds (bind_worlds; SceneWorldSpec): the nodes with `world: <name>` are the worlds a
// scene can be (a generated world, a data bundle's city, a terrain, a flat ground), and one is on
// at a time. The others' bodies are out of the physics world (and cvcGL does not draw them). Under
// the world key (<prefix>.scene.world):
//   source                the world on: write another world's name to switch (world_source.ari)
//   sources               the worlds' names, one per line, in the document's order
//   node, kind            the world's nodes (comma-separated) and what it is: generated, data,
//                         terrain or flat
//   ready                 1 once every node of it has its world (a generated world made, a dataset
//                         loaded); the bodies come with it
//   rev                   goes up each time the world on changes: a switch, a new world from it
//   status                a line saying so
//   on.<node>             1 for each node of the world on, 0 for the others' (their visible:)
//   grid.{rows,cols,cell,occupied,summary}   with the block's cell: the world on's grid (grid())

#ifndef __CVC_ARIADNE_SCENE_WORLD_H__
#define __CVC_ARIADNE_SCENE_WORLD_H__

#include <array>
#include <chrono>
#include <cstdint>
#include <cvc/core/world_clock.h>
#include <cvc/physics/field.h>
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
struct body_desc;
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
struct SceneWorldSpec;
struct SceneColliders;
struct vehicle_archetype;
struct spawn_rules;
class WorldVehicles;
class BoatRuntime;
class WaterRuntime;
class WeatherFriction;

// Whether a scene has a world to run: a `physics:` block, a `world:` block, or a node (at any
// depth) with a `body:` or a `world:` (a data node's bodies come with its dataset, so its body
// alone does not count).
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
  // Ground a vehicle drives on, when its node is one of the scene's worlds: a node's own body,
  // and of the colliders a node makes (a generated or data world's) its terrain, a height field.
  // A world's buildings are not: a vehicle touching one has hit something.
  bool ground = true;
  // Where the node is drawn relative to the body: body pose x body_from_visual (row-major).
  std::array<double, 16> body_from_visual{};
  // What the body was made from (its shape, mass, motion) when a node declared it; null for the
  // bodies of a node's colliders (a generated world's, a dataset's). A water node's wake reads it.
  std::shared_ptr<const physics::body_desc> desc;
};

class SceneWorld {
public:
  // A scene with vehicles (world: vehicles:) runs on the simulation hosts' clock (cvc/core/
  // sim_tick.h): app.world_clock() goes to the 1/120 s quantum that the vehicles' tick divides
  // into, with real time's 0.25 s catch-up, unless it has already ticked (it warns then). Call it
  // before the scene's physics world is made: from_scene does, and a host that builds the world
  // itself (cvcGL's realize_scene) does too.
  static void use_vehicle_clock(cvc::app &app, const Scene &scene,
                                std::vector<std::string> *warnings = nullptr);
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

  // The world a generated_world node made: the one this world runs (from_scene), else the one its
  // dataset holds (a renderer's node, or a data node on a made dataset: its `generated`). Null for
  // any other node.
  std::shared_ptr<const cvc::world::generated_world> generated_world(const std::string &id) const;
  // The world node `id` makes at run time (a generated world's, a dataset's) as it is now, and its
  // revision, which each new world bumps (0: none yet). Null for any other node, or before it has
  // one. The bodies take a new world at the next sync().
  std::shared_ptr<const cvc::world::world_geometry>
  world_geometry(const std::string &id, std::uint64_t *revision = nullptr) const;
  // That world as the navigator sees it (cvc::nav::world_grid): `cell` m samples over its terrain,
  // or a square `half` m either side of the terrain's centre. A terrain node's grid is its height
  // field's; a node with a body of its own is a flat ground: an open walled grid over its
  // footprint. Made once per world revision, cell and half, and kept until the next. Null when the
  // node has no world yet, no body, or its world has no terrain (it warns).
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
  // The same, with `before_each` called before each step (after the vehicles, boats and weather
  // have put their forces in, just before the world steps) and `after_each` after it (after the
  // vehicles have read theirs), each with the clock tick the step ends on. A holon that runs this
  // world puts its avatars' drives in the first and reads them in the second (holon::attach_world).
  using step_hook = std::function<void(std::uint64_t tick)>;
  std::uint64_t follow(const cvc::world_clock &clock, const std::function<void()> &after_step,
                       const step_hook &before_each, const step_hook &after_each);

  // Rebuild the world as build() made it: the same object, a fresh physics world with the same
  // config and materials, each node's body where the document put it (at rest unless it declared
  // a velocity), and each tracked node's colliders as they are now. Its bodies are made in the
  // order build() made them, so their ids are the same, and stepping it runs as a world built
  // now. The next follow() anchors on the clock again (sim.restart resets the clock to tick 0).
  // Problems warn, naming the node.
  void restart(std::vector<std::string> *warnings = nullptr);

  // The telemetry, written now.
  void publish_stats();
  // Take on the scene's worlds: each node with `world: <name>` (at any depth, with a body or not),
  // and the scene's `world:` block. The world on starts as `<world_key>.source` says if it names
  // one of them, else as the block does, else the first; the others' bodies leave the physics
  // world. Then sync() follows the key, and the keys above are written under `world_key` (an
  // absolute state path, <prefix>.scene.world). from_scene does this itself; a renderer that
  // build()s the world calls it. A scene without world nodes writes nothing.
  void bind_worlds(const Scene &scene, const std::string &world_key,
                   std::vector<std::string> *warnings = nullptr);
  // The world on ("" without worlds), and switch to another: false (and a warning) when the scene
  // has no world of that name. The key follows.
  const std::string &world_on() const;
  bool set_world(const std::string &name, std::vector<std::string> *warnings = nullptr);
  // The world on made again with `seed`, when it is a generated world this world runs (from_scene)
  // and that is not its seed now: each of its generated_world nodes' seed key is set and its
  // request key asks for a new world, which the next sync() makes. False when there is nothing to
  // make (no such node, or the seed is already its).
  bool regenerate_world(double seed);

  // Take on the scene's vehicles (Scene::vehicles, the document's world: vehicles:): a
  // WorldVehicles under `vehicles_key` and `entities_key` (absolute state paths, <prefix>.vehicles
  // and <prefix>.entities), driving on the world that is on (its grid: the world block's cell, 2 m
  // without one). from_scene does this itself; a renderer that build()s the world calls it. No
  // archetypes: nothing.
  // `spawns` are the document's spawn points and policy, under `spawns_key` (<prefix>.spawns).
  void bind_vehicles(const std::vector<vehicle_archetype> &archetypes,
                     const std::string &vehicles_key, const std::string &entities_key,
                     const spawn_rules &spawns, const std::string &spawns_key);
  // The vehicles (null before bind_vehicles, or without archetypes).
  WorldVehicles *vehicles();
  const WorldVehicles *vehicles() const;

  // Take on the scene's boat nodes (type: boat, at any depth; scene_boat.h): a BoatRuntime each,
  // its keys under `bind_prefix`. Each boat goes in at a sync() once its water's grid (the water
  // node's ground's, with the water's depth) is ready and has water where it starts; it steps with
  // the world. from_scene does this itself; a renderer that build()s the world calls it.
  void bind_boats(const Scene &scene, const std::string &bind_prefix,
                  std::vector<std::string> *warnings = nullptr);
  // The boat nodes' runtimes, in the document's order, and one by its node's id (null for none).
  const std::vector<std::unique_ptr<BoatRuntime>> &boats() const;
  BoatRuntime *boat(const std::string &node);
  // A water node's runtime this world runs (from_scene's; a renderer runs its own), or null.
  const WaterRuntime *water(const std::string &node) const;
  // Take on the scene's wind (scene_wind.h, O14): seed its keys from `scene: wind:` and, while
  // wind.speed is set under bind_prefix, keep a flow field of air (1.225 kg/m^3, C_d 1) whose
  // velocity is that wind (cvc::physics::wind_flow, its profile over the document's wind base)
  // in the world, so every dynamic body in the air is pushed by it and a vehicle's aero drag
  // reads it; a kinematic or model-driven vehicle feels it through WorldVehicles::set_wind.
  // sync() follows the keys (a change swaps the field). from_scene does this itself; a renderer
  // that build()s the world calls it.
  void bind_wind(const Scene &scene, const std::string &bind_prefix);
  // The wind's flow field now (invalid without a scene wind), and its air (null without one): a
  // wind_flow, or a StormWindFlow with the weathers' gust fronts. What a body outside the world
  // reads the same wind from: a MuJoCo robot's set_wind(wind(), world().field_time() at its
  // start). The air is read at the world's field time (it adds its own offset to the clock for its
  // storms), so pass that, not the clock's time.
  cvc::physics::field_id wind_field() const;
  std::shared_ptr<const cvc::physics::flow_velocity> wind() const;
  // Take on the scene's weather nodes (type: weather, at any depth; scene_weather.h): a
  // WeatherFriction each, its keys under `bind_prefix`, which sets the surface materials'
  // friction from the rain inside the physics step, on the world clock's ticks, so every host
  // applies the same friction at the same step. from_scene does this itself; a renderer that
  // build()s the world calls it.
  // With a scene wind, the weather's storms are in it too (F4): the wind's field is then a
  // StormWindFlow (the gust fronts in space and time, read at each step's own time), and each
  // sync() writes the front at the first weather's centre to wind.front (scene_wind.h), before
  // the water reads its wind.
  void bind_weather(const Scene &scene, const std::string &bind_prefix);
  // The weather nodes' friction runtimes, in the document's order.
  const std::vector<std::unique_ptr<WeatherFriction>> &weathers() const;

  // The warnings follow()'s syncs gathered since the last call.
  std::vector<std::string> take_warnings();

private:
  SceneWorld();
  // The scene.world keys, when the world on changed since the last time (or `force`).
  void publish_worlds(bool force);
  // The wind's field after the scene wind's keys (bind_wind) and the weathers' storms.
  void sync_wind();
  // wind.front: the storms' gust front at the first weather's centre at the start of the clock's
  // kSceneWindFrontPeriod.
  void publish_front();
  struct impl;
  std::unique_ptr<impl> _impl;
};

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_SCENE_WORLD_H__
