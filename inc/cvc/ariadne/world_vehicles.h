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

#ifndef __CVC_ARIADNE_WORLD_VEHICLES_H__
#define __CVC_ARIADNE_WORLD_VEHICLES_H__

#include <array>
#include <cstdint>
#include <cvc/ariadne/world_rules.h> // vehicle_archetype
#include <cvc/ariadne/world_spawns.h>
#include <cvc/physics/types.h>
#include <cvc/physics/vehicle.h> // vehicle_desc
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
class app;
namespace physics {
class world;
}
namespace nav {
struct world_grid;
}
namespace world {
class world_geometry;
}

namespace ariadne {

// ----------------
// A world's vehicles, run (ARIADNE_WORLDS.md §14, L2)
// ----------------
// The vehicles a world document offers (world: vehicles: archetypes:) become entities: each one
// spawned from an archetype, with an identity (a name, a team, two colours, metadata), a
// controller (nav@1, the navigator with a policy of its own, or none: it holds still) and its
// settings within the archetype's ranges. Its body is in the world's own physics world, so it
// collides with the scene's bodies (their buildings too), and it steps on the world clock with
// them, on its archetype's plant:
//
//   jolt       a Jolt vehicle (nav_plant, which commands it; the world steps it)
//   dynamic    the dynamic single-track model (dyn_bike_plant), its chassis box a body
//   dynamic4w  the four-wheel model with suspension (dyn_4w_plant), its chassis box a body
//   kinematic  nav's bicycle, its chassis box a kinematic body that pushes what it meets (nav
//              keeps it off the buildings; nothing stops it)
//
// The dynamic models take one substep of each world step, so they step on the world's clock with
// everything else. The entities on each plant have a navigator of their own (a navigator drives
// one plant), all on the world's grid and the same tick.
//
// One WorldVehicles per world runtime (SceneWorld), with no process-wide state: a host that runs
// several worlds (a relay's rooms) runs one each, under its own keys.
//
// From outside (S1), any entity takes an intent: a wanted steer and acceleration that replaces or
// blends with its controller's command, an external planar force on its drive, or both. A
// document, a script or a server writes it as state (`entities.<id>.intent.*`, `.force.*`); a C++
// or Python host calls set_intent(). The `external` controller is one that only its intent
// drives.

// An archetype's vehicle: its preset with the archetype's overrides on top, then a spawn's mass
// and top speed (0: the archetype's), each held to the archetype's range. Throws
// std::invalid_argument for a preset that is not one.
physics::vehicle_desc vehicle_desc_of(const vehicle_archetype &a, double mass = 0.0,
                                      double max_speed = 0.0);

// A vehicle model's file by name (`humvee`: Humvee.glb; any other name is taken as the file's
// name), where the vehicle-humvee cvcpkg package installs it: beside $CVC_NAV_BUNDLE (in it, or
// in its ../shared), then in share/cvc-scenes/shared and share/cvc-scenes/*/ under $CVC_PREFIX,
// $CVCPKG_PREFIX, $CONDA_PREFIX and the running program's prefix. "" when there is none.
std::string find_vehicle_model(const std::string &name);

struct SceneAsset;
// Each archetype's look as a view can draw it wherever it runs: a model by name found (a fitted
// file), a document asset's file, a file beside the document (`doc_dir`), and the default look
// made explicit (the Humvee model for a humvee when it is installed, else the box). A look that
// cannot be found is drawn as its box, with a warning.
void resolve_vehicle_looks(std::vector<vehicle_archetype> &archetypes, const std::string &doc_dir,
                           const std::vector<SceneAsset> &assets,
                           std::vector<std::string> *warnings = nullptr);

// What a vehicle_selector shows of a vehicle.
struct vehicle_stats {
  double length = 0.0, width = 0.0, height = 0.0; // m, the chassis box
  double mass = 0.0;                              // kg
  double power_kw = 0.0;  // the engine's peak, over its torque curve and rev range
  double top_speed = 0.0; // m/s (0: unlimited)
  double wheelbase = 0.0; // m
};
vehicle_stats vehicle_stats_of(const physics::vehicle_desc &d);

// What a spawn asks for. Empty and zero fields take the archetype's.
struct entity_spec {
  std::string archetype;
  std::string name;
  int team = 0;
  std::array<float, 3> color{0.85f, 0.70f, 0.20f};  // primary, rgb in [0, 1]
  std::array<float, 3> color2{0.12f, 0.12f, 0.14f}; // secondary
  std::map<std::string, std::string> metadata;      // the user's own; nothing reads it
  // nav@1, none or external (its intent alone drives it); "" the archetype's first
  std::string controller;
  std::string policy, policy_sha256; // nav@1's policy (a .cvcnav); "" the default policy
  double mass = 0.0, max_speed = 0.0;
  // Where it starts: (x, y) in world metres facing `yaw` (rad; not finite: toward its goal), when
  // `placed`; else a free point of the world's grid away from the others.
  bool placed = false;
  double x = 0.0, y = 0.0, yaw = std::numeric_limits<double>::quiet_NaN();
};

// What an entity is told from outside: a command, a force, or both, kept until changed.
//
//   command  off: the controller's command alone; replace: steer and accel in its place; blend:
//            `weight` of them and 1 - weight of the controller's. An `external` entity has no
//            command of its own: replace and blend both drive it, and off holds it still.
//   steer    the bicycle angle (rad, + left), held to the vehicle's lock
//   accel    m/s^2 along its heading; from rest a negative one backs it up. Held to the speeds
//            the navigator's command is held to (its top speed).
//   force    m/s^2 in world x and y, added to the drive's force (the goal's pull and the walls'
//            push) as cvc::nav::ext_force adds one: it speeds, slows and (with force_steers)
//            steers the drive, under any command but replace, which takes the drive's place.
//            The drive has one steering switch for every vehicle: it is on while any entity's
//            force asks for it.
struct entity_intent {
  enum class mode { off, replace, blend };
  mode command = mode::off;
  double steer = 0.0, accel = 0.0;
  double weight = 0.5; // blend: the intent's share, in [0, 1]
  double force_x = 0.0, force_y = 0.0;
  bool force_steers = true;
  bool operator==(const entity_intent &) const = default;
};
// "off", "replace", "blend"; and back (false for anything else).
const char *intent_mode_name(entity_intent::mode m);
bool parse_intent_mode(const std::string &s, entity_intent::mode &out);

// A wheel as it is drawn, in its vehicle's chassis box's frame (m from the box's centre: x
// forward, y left, z up): its centre, its steering angle (rad, + left), and whether it is on the
// ground. Front to back by axle, each axle's left wheel first.
struct entity_wheel {
  double x = 0.0, y = 0.0, z = 0.0;
  double steer = 0.0;
  bool down = true;
};

// An entity as it is now.
struct entity_info {
  std::uint64_t id = 0;
  entity_spec spec;    // with its controller, mass and max_speed resolved
  bool placed = false; // in the physics world (false while the world has no grid)
  physics::body_id chassis;
  double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0; // the chassis's centre, world metres
  double ground_z = 0.0;                       // the grid's ground under it
  double speed = 0.0;                          // m/s along its heading
  double power_kw = 0.0, energy_wh = 0.0, wh_per_km = 0.0, distance = 0.0;
  double wheel_load = 0.0, aero_force = 0.0; // N: the wheels' push on the ground, the drag
  int gear = 0;
  double rpm = 0.0, throttle = 0.0, brake = 0.0;
  bool reached = false; // at its goal (nav@1 then draws another)
  std::string plant;    // its archetype's: jolt, dynamic, dynamic4w or kinematic
  // The command its plant took on the last navigator tick: rad and m/s^2, as entity_intent's.
  double command_steer = 0.0, command_accel = 0.0;
  entity_intent intent; // as of the last navigator tick
};

class WorldVehicles {
public:
  // The vehicles of `world`, under `vehicles_key` (the catalog, the spawn keys), `entities_key`
  // (the table) and `spawns_key` (the spawn points, WorldSpawns), absolute state paths ("" = no
  // keys). `app` may be null (no keys).
  WorldVehicles(cvc::app *app, physics::world &world, std::vector<vehicle_archetype> archetypes,
                std::string vehicles_key, std::string entities_key, spawn_rules spawns = {},
                std::string spawns_key = std::string());
  ~WorldVehicles();
  WorldVehicles(const WorldVehicles &) = delete;
  WorldVehicles &operator=(const WorldVehicles &) = delete;

  const std::vector<vehicle_archetype> &archetypes() const;

  // The ground the vehicles drive on: the navigator's grid of the world that is on, and the bodies
  // that are the ground (a chassis touching one has not hit anything). A new grid places every
  // entity again: at its spawn point if that is free in it, else at a free point. Null: the
  // vehicles leave the physics world, and the entities wait for a grid.
  // `world_name` and `world` (its roads, null for a terrain or a flat ground) are the world on's,
  // for the spawn points.
  void set_ground(std::shared_ptr<const nav::world_grid> grid,
                  std::vector<physics::body_id> ground_bodies,
                  const std::string &world_name = std::string(),
                  std::shared_ptr<const cvc::world::world_geometry> world = nullptr);
  // The physics world was made afresh (SceneWorld::restart): every vehicle is gone from it. Each
  // entity goes back to its spawn point, at rest.
  void world_reset();

  // The spawn points: a spawn without a place of its own starts at the one their policy picks
  // (a free point of the grid when none applies).
  WorldSpawns &spawns();
  const WorldSpawns &spawns() const;

  // Spawn one: its id, or 0 and why (an archetype that is not one, a controller it does not
  // allow, a policy that cannot be loaded). Without a grid it is kept and placed once there is
  // one.
  std::uint64_t spawn(const entity_spec &spec, std::string *why = nullptr);
  bool despawn(std::uint64_t id);
  void clear();

  // An entity's intent (entity_intent), from now until it is changed: any thread may call these.
  // It takes effect at the next navigator tick. False for an id that is not an entity, or an
  // intent that is not finite or a weight outside [0, 1]. intent() is the one that will be used
  // (off for none).
  bool set_intent(std::uint64_t id, const entity_intent &intent);
  bool clear_intent(std::uint64_t id);
  entity_intent intent(std::uint64_t id) const;
  // A second table of intents beside the entities' own keys (world: vehicles: intents:): an
  // absolute state path whose `<path>.<id>` (or `<path>.<name>`, for a name that is a state name)
  // holds `intent.*` and `force.*` as an entity's keys do. A relay's published namespace, so a
  // server document's program steers the entities of the worlds its clients run. "" none.
  void set_intents_key(std::string key);
  const std::string &intents_key() const;

  // Read the request keys: the selected archetype and settings (its keys follow), a spawn, a
  // removal, a clear.
  void poll();
  // Around each physics step. Before it: every k-th (k = nav's tick / the step) runs the
  // navigators' tick, which commands every vehicle for the steps until the next; that tick reads
  // the intent keys first (an intent key's change replaces that entity's intent, as set_intent()
  // does). Then the dynamic models' substep, and the kinematic boxes a step along the bicycle's
  // path. After it: the dynamic models take back what the step's contacts did.
  void before_step();
  void after_step();
  // The entities' keys and the selection's, now.
  void publish();

  std::vector<entity_info> entities() const;
  // Entity `id`'s wheels as drawn (entity_wheel), as of the last step: a view poses each one.
  // Empty for an id that is not a placed entity.
  std::vector<entity_wheel> wheels(std::uint64_t id) const;
  std::size_t size() const;
  // Bumps whenever a chassis comes or goes (a spawn, a removal, a new grid, a restart): a view
  // binds its drawings to the chassis again.
  std::uint64_t revision() const;
  // A line per entity for physics.stats.vehicles: id, name, plant, speed, power, energy, wheel
  // load.
  std::string stats_lines() const;
  const std::string &status() const;

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_WORLD_VEHICLES_H__
