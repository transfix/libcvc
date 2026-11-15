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
#include <cvc/nav/detail/dyn_4w.h> // the models a twin steps: dyn4_params, dyn_bike_params, bike_veh
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
// settings within the archetype's ranges. Its body is a Jolt vehicle in the world's own physics
// world, so it collides with the scene's bodies, and it steps on the world clock with them.
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
  // What drives its body: jolt (a Jolt vehicle in the world's physics world), kinematic (nav's
  // bicycle, no body), dynamic or dynamic4w (the dynamic single-track or four-wheel model, its
  // chassis in a world of its own holding the world's colliders); "" the archetype's.
  std::string plant;
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
//            Each plant's drive has one steering switch for its vehicles: it is on while any
//            of their forces asks for it.
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
  double vx = 0.0, vy = 0.0;         // m/s, the chassis's velocity in the world
  double steer = 0.0;                // rad, + left: the steering's bicycle angle now
  double goal_x = 0.0, goal_y = 0.0; // world metres: where it drives (none: where it parks)
  double reach = 0.0;                // m from its goal that counts as there
  bool reached = false;              // at its goal (nav@1 then draws another, unless held)
  // Its chassis's contacts over the physics steps since the navigator's last tick (each step's
  // contact events, begun or, when the scene's physics reports them, persisting): with the
  // world's bodies other than the ground (buildings, walls, props), and with another vehicle.
  int obstacle_contacts = 0;
  bool vehicle_contact = false;
  // The command its plant took on the last navigator tick: rad and m/s^2, as entity_intent's.
  double command_steer = 0.0, command_accel = 0.0;
  entity_intent intent; // as of the last navigator tick
};

// What drives an entity, as a model that a differentiable twin of the world steps (WorldGym::
// plants, L4's twin; ARIADNE_WORLDS.md §14). In SI: world metres, seconds, kg, rad. The models are
// cvc::nav::diff's, and on the dynamic plants they are the plants' own: their parameters, step
// limits and state as the plant holds them.
struct entity_plant {
  std::uint64_t id = 0;
  std::string plant;          // jolt, kinematic, dynamic or dynamic4w
  physics::vehicle_desc desc; // its body
  double rear_x = 0.0;        // the rear axle along the body from the chassis's centre (m)
  bool placed = false;        // in the world; nothing below is set otherwise
  // The model that stands for the plant:
  //   dyn4      dynamic4w: the plant's own (physics::dyn_4w_plant), `exact`. jolt: the four-wheel
  //             model with the vehicle's own numbers (nav_plant::dyn4_si_of), its pose and twist
  //             read from the chassis, its heave 0 and its drive state 0 (not `exact`).
  //   dyn_bike  dynamic: the plant's own (physics::dyn_bike_plant), `exact`.
  //   bike      kinematic: nav's bicycle; `bike_state` holds the rear axle, the heading and the
  //             speed (as vx), `limits` its limits.
  std::string model;
  bool exact = false;
  nav::diff::dyn4_params dyn4{};
  nav::diff::dyn4_state<double> dyn4_state{};
  nav::diff::dyn_bike_params bike{};
  nav::diff::dyn_state<double> bike_state{};
  nav::diff::bike_veh limits{}; // the model step's limits (hdt: one model step)
  int substeps = 1;             // model steps per navigator tick
  // dyn4: the model's heights are the ground's less this (m), as dyn_4w_plant measures them.
  double ground_reference = 0.0;
  double mu = 0.9; // the ground's grip the model drives on
  // The last tick: nav's command (the steering's angle, rad; the acceleration along the heading,
  // m/s^2) and, on the dynamic plants, what the model took of it in each model step after the
  // driver (m/s^2), with the grade (the sine along the heading) for dyn_bike. Away from a
  // collision, replaying the model from the tick's first state with these gives its last state.
  double steer_command = 0.0, accel_command = 0.0;
  std::vector<double> applied_accel, applied_grade;
  int collisions_tick = 0; // model steps of the last tick a collision corrected (exact models)
};

// The navigator the entities drive with, as a twin of the drive takes it: nav's normalized units
// per metre, its tick (s), the arrival radius (m), and its vehicle in normalized units (the
// largest and fastest any archetype may be; each vehicle's own envelope is in entity_plant).
struct navigator_params {
  bool ready = false; // a grid, and the navigator built for it
  double scale = 0.0, tick = 0.0, reach = 0.0;
  float rr = 0.0f, d_hat = 0.0f, dt = 0.0f, vmax = 0.0f, L = 0.0f, delta_max = 0.0f, a_max = 0.0f,
        a_lat_max = 0.0f, k_steer = 0.0f;
  int nsub = 1;
  bool allow_reverse = true;
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
  // The physics world was made afresh (SceneWorld::restart): every vehicle is gone from it, and
  // the ground is `ground_bodies` (a node whose colliders are made at run time, a generated
  // world's, has new body ids). Each entity goes back to its spawn point, at rest.
  void world_reset(std::vector<physics::body_id> ground_bodies);

  // The spawn points: a spawn without a place of its own starts at the one their policy picks
  // (a free point of the grid when none applies).
  WorldSpawns &spawns();
  const WorldSpawns &spawns() const;

  // Spawn one: its id, or 0 and why (an archetype that is not one, a controller it does not
  // allow, a plant a scene world does not run yet, a policy that cannot be loaded). Without a
  // grid it is kept and placed once there is one.
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
  // Before each physics step: every k-th (k = nav's tick / the step) runs the navigator's tick,
  // which commands every vehicle for the steps until the next. That tick reads the intent keys
  // first: an intent key's change replaces that entity's intent, as set_intent() does.
  void before_step();
  // After each physics step: the step's contacts, counted on the vehicles they touch.
  void after_step();
  // The entities' keys and the selection's, now.
  void publish();

  // ---- what a gym drives (WorldGym, L4) ----
  // The draws from here on: the free points spawns and goals are drawn at, and the spawn
  // policy's (random's draw, round_robin's turn), as a runtime made now with `seed` would draw
  // them. With no entities, ids start again at 1.
  void reseed(std::uint64_t seed);
  // Whether nav@1 draws another goal when it reaches one (on by default). Off, it parks there.
  void set_wander(bool wander);
  // Entity `id`'s command for the navigator's next tick only, in place of its controller's: the
  // steering's bicycle angle (rad, + left) and the acceleration along its heading (m/s^2), held to
  // its envelope as the navigator's own command is. Either not finite: its controller drives that
  // tick. False when it is not in the world.
  bool command(std::uint64_t id, double steer, double accel);
  // Drive entity `id` to (x, y) world metres. False when it is not in the world.
  bool set_goal(std::uint64_t id, double x, double y);
  // The navigator's tick, in seconds and in physics steps (a whole number of them); 0 without a
  // grid.
  double tick_seconds() const;
  int steps_per_tick() const;
  // The navigator's grid of the world on (null without one).
  std::shared_ptr<const nav::world_grid> grid() const;

  // ---- what a twin of the world reads (WorldGym::fields, WorldGym::plants) ----
  // Each entity's plant as a model (entity_plant), in the order entities() gives them.
  std::vector<entity_plant> plants() const;
  // The navigator's own settings (not ready without a grid).
  navigator_params navigator() const;
  // The navigator's signed distance (m) on its grid (cvc::nav::signed_distance of grid()), rows *
  // cols; empty without a grid.
  const std::vector<float> &sdf() const;

  std::vector<entity_info> entities() const;
  std::size_t size() const;
  // Bumps whenever a chassis comes or goes (a spawn, a removal, a new grid, a restart): a view
  // binds its drawings to the chassis again.
  std::uint64_t revision() const;
  // A line per entity for physics.stats.vehicles: id, name, speed, power, energy.
  std::string stats_lines() const;
  const std::string &status() const;

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_WORLD_VEHICLES_H__
