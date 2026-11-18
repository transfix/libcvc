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
#include <cvc/physics/dyn_boat_plant.h> // dyn_boat_desc
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
class flow_velocity;
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
// them, on its archetype's plant (or its spawn's own):
//
//   jolt       a Jolt vehicle (nav_plant, which commands it; the world steps it)
//   dynamic    the dynamic single-track model (dyn_bike_plant), its chassis box a body
//   dynamic4w  the four-wheel model with suspension (dyn_4w_plant), its chassis box a body
//   kinematic  nav's bicycle, its chassis box a kinematic body that pushes what it meets (nav
//              keeps it off the buildings; nothing stops it)
//   dynamicboat  a boat (dyn_boat_plant), its hull a body afloat on the world's water: the field
//              a scene's water node puts in the world, or, where the world has none over the
//              grid's water, a still one at the water's level that WorldVehicles adds. Its
//              navigator plans on the water deep enough for the deepest boat (land blocked).
//
// Each archetype starts and aims on its medium (its plant's vehicle_plant::medium(); WorldSpawns'
// needs): a car out of water deeper than a ford, a boat in water deep and open enough for it.
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
// and top speed (0: the archetype's), each held to the archetype's range. A boat's (dynamicboat)
// is a stand-in for what reads a vehicle: its hull's box, mass, top speed, the thrusts per unit
// mass as its acceleration and braking, the lock of its plan, and no wheels. Throws
// std::invalid_argument for a preset that is not one.
physics::vehicle_desc vehicle_desc_of(const vehicle_archetype &a, double mass = 0.0,
                                      double max_speed = 0.0);
// A boat archetype's boat (dynamicboat): its preset (launch, jonboat) with the archetype's mass and
// max_speed overrides, then a spawn's, held to its ranges. Throws std::invalid_argument for a
// preset that is not a boat's.
physics::dyn_boat_desc boat_desc_of(const vehicle_archetype &a, double mass = 0.0,
                                    double max_speed = 0.0);

// A vehicle model's file by name (`humvee`: Humvee.glb; any other name is the file's name, a
// name without an extension its .glb: TacticalBoat, TacticalBoat.glb), where the vehicle-*
// cvcpkg packages install them: beside $CVC_NAV_BUNDLE (in it, or
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
  // Its plant (jolt, dynamic, dynamic4w or kinematic, as WorldVehicles runs them); "" its
  // archetype's.
  std::string plant;
  std::string policy, policy_sha256; // nav@1's policy (a .cvcnav); "" the default policy
  double mass = 0.0, max_speed = 0.0;
  // Where it starts: (x, y) in world metres facing `yaw` (rad; not finite: toward its goal), when
  // `placed`; else a free point of the world's grid away from the others. (x, y) is its rear axle,
  // the navigator's reference point on every plant (a spawn point's is too); entity_info's (x, y)
  // is its chassis's centre, ahead of it along the heading.
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
  // N, world x and y: the scene wind's push on it, its share of the air's drag (the drag in the
  // moving air less the drag in still air at the same speed). A jolt vehicle's aero drag reads the
  // wind in physics; a kinematic or model-driven one's drive is pushed by it (set_wind). 0 without
  // a wind.
  double wind_fx = 0.0, wind_fy = 0.0;
  int gear = 0;
  double rpm = 0.0, throttle = 0.0, brake = 0.0;
  double vx = 0.0, vy = 0.0;         // m/s, the chassis's velocity in the world
  double steer = 0.0;                // rad, + left: the steering's bicycle angle now
  double goal_x = 0.0, goal_y = 0.0; // world metres: where it drives (none: where it parks)
  double reach = 0.0;                // m from its goal that counts as there
  bool reached = false;              // at its goal (nav@1 then draws another, unless held)
  std::string plant;                 // jolt, dynamic, dynamic4w, kinematic or dynamicboat (as
                                     // spec.plant)
  // Its chassis's contacts over the physics steps since the navigator's last tick (each step's
  // contact events, begun or, when the scene's physics reports them, persisting): with the
  // world's bodies other than the ground (buildings, walls, props), and with another vehicle. A
  // kinematic box's obstacles are its footprint on the grid's blocked cells.
  int obstacle_contacts = 0;
  bool vehicle_contact = false;
  // The command its plant took on the last navigator tick: rad and m/s^2, as entity_intent's.
  double command_steer = 0.0, command_accel = 0.0;
  entity_intent intent; // as of the last navigator tick
};

// The navigator's drive on an entity's last tick, as a twin replays it: the coefficients its
// policy gave (alpha, the walls' barrier; beta, the goal's pull; gamma, the damping), the point it
// steered for (its carrot, world metres) and the external force it summed (m/s^2, world x and y:
// an intent's force). Known only while the navigators capture their drive
// (WorldVehicles::set_drive_telemetry) and after a tick has run since.
struct entity_drive {
  bool known = false;
  double alpha = 0.0, beta = 0.0, gamma = 0.0;
  double goal_x = 0.0, goal_y = 0.0;
  double force_x = 0.0, force_y = 0.0;
};

// The limits the navigator's drive plans an entity's command with on its plant (its plant arm,
// cvc::nav::plant_drive; sim_world::plant_plan), in SI, for a twin to give the command the plant
// took from the drive's: the vehicle's envelope (a_max, vmax, the cornering a_lat) and its
// wheelbase; the steering lock (rad, the fleet's tightest; 0: nav's own alone), the actuator's
// slew limit (rad/s; 0: none) and its angle now (rad); the latency (s) and the braking the
// vehicle achieves (a_brake; 0: a_max); its body as footprint discs (radius, and offsets along
// the heading from the rear axle, m), when the plant gives its size; whether the next tick
// parks it (it brakes to a stop at a_brake); and whether the plant's measured state modulates
// the drive (plant_modulation), which a twin then cannot replay from these. Known with a plant
// (none on nav's bicycle), from the first tick's start.
struct entity_plan {
  bool known = false;
  double a_max = 0.0, vmax = 0.0, a_lat = 0.0, wheelbase = 0.0;
  double max_steer = 0.0, steer_rate = 0.0, steer_now = 0.0;
  double latency = 0.0, a_brake = 0.0;
  bool footprint = false;
  double body_rr = 0.0;
  std::array<double, 4> body_offsets{};
  bool parked = false, modulated = false;
};

// What drives an entity, as a model that a differentiable twin of the world steps (WorldGym::
// plants, L4's twin; ARIADNE_WORLDS.md §14). In SI: world metres, seconds, kg, rad. The models are
// cvc::nav::diff's, and on the dynamic plants they are the plants' own: their parameters, step
// limits and state as the plant holds them.
struct entity_plant {
  std::uint64_t id = 0;
  std::string plant;          // jolt, kinematic, dynamic, dynamic4w or dynamicboat
  physics::vehicle_desc desc; // its body
  double rear_x = 0.0;        // the rear axle along the body from the chassis's centre (m)
  bool placed = false;        // in the world; nothing below is set otherwise
  // The model that stands for the plant:
  //   dyn4      dynamic4w: the plant's own (physics::dyn_4w_plant), `exact`. jolt: the four-wheel
  //             model with the vehicle's own numbers (nav_plant::dyn4_si_of), its pose and twist
  //             read from the chassis, its heave 0 and its drive state 0 (not `exact`).
  //   dyn_bike  dynamic: the plant's own (physics::dyn_bike_plant), `exact`.
  //   bike      kinematic: nav's bicycle, `exact`; `bike_state` holds the rear axle, the heading
  //             and the speed (as vx), `limits` its limits. dynamicboat: the bicycle nav plans the
  //             boat with (physics::dyn_boat_plant drives it; not `exact`).
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
  entity_drive drive;      // the drive that gave that command
  entity_plan plan;        // the limits it planned with on the plant
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
  // How it arrives (sim_world::config): each agent comes to rest at its goal (stop_at_goal: the
  // drive's stop point is the goal), and a parked one on nav's bicycle brakes to a stop and holds
  // (park_hold).
  bool stop_at_goal = false, park_hold = false;
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
  // allow, a policy that cannot be loaded). Without a grid it is kept and placed once there is
  // one.
  std::uint64_t spawn(const entity_spec &spec, std::string *why = nullptr);
  // The vehicles a world starts with (world: vehicles: start:), spawned in order, each as spawn()
  // would; a refusal is a warning and stops that entry. Returns how many it spawned.
  std::size_t spawn_starts(const std::vector<vehicle_start> &starts,
                           std::vector<std::string> *warnings = nullptr);
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

  // The scene's wind (scene_wind.h, O14), or null. A jolt vehicle feels it in physics already
  // (its aero drag reads the air's velocity, the world's wind field). The kinematic plant (nav's
  // bicycle and a kinematic box) and the dynamic models (their own integrators, the box following
  // them) feel no field, so each navigator tick adds the wind's drag on each of their vehicles to
  // its drive, as an intent's force is added (cvc::nav::ext_force, steering too):
  // 1/2 rho C_d A |u| u over its mass, u the wind at its chassis minus its velocity, with its
  // vehicle_desc's air density, drag coefficient and frontal area. So a crosswind pushes it
  // aside and its navigator steers back against it. Costs a wind read per vehicle per tick.
  void set_wind(std::shared_ptr<const physics::flow_velocity> wind);

  // Read the request keys: the selected archetype and settings (its keys follow), a spawn, a
  // removal, a clear.
  void poll();
  // Around each physics step. Before it: every k-th (k = nav's tick / the step) runs the
  // navigators' tick, which commands every vehicle for the steps until the next; that tick reads
  // the intent keys first (an intent key's change replaces that entity's intent, as set_intent()
  // does). Then the dynamic models' substep, and the kinematic boxes a step along the bicycle's
  // path. After it: the dynamic models take back what the step's contacts did.
  void before_step();
  // After it, too: the step's contacts, counted on the vehicles they touch.
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
  // Whether vehicles steer around each other (on by default): each navigator's separation pushes
  // its carrot away from every vehicle near it, its own fleet's and the other plants' alike
  // (cvc::nav::sim_world::set_foreign_agents), at the nav demos' radius and strength for the
  // world's largest vehicle. The vehicles.avoid key ("0" or "1") sets it too, read each poll; a
  // document's init: may set it before the world is made.
  void set_avoid(bool avoid);
  bool avoid() const;
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
  // The navigator's tick to use (s), cvc::kSimTick by default: the one simulation tick every host
  // shares (cvc/core/sim_tick.h). It takes effect when the navigator is next built (a new grid, a
  // restart: WorldGym::reset), held to the nearest whole number of physics steps (the status says
  // when it is not one). Throws std::invalid_argument for a tick that is not finite and > 0.
  void set_tick(double seconds);
  double tick() const;
  // The navigator's grid of the world on (null without one).
  std::shared_ptr<const nav::world_grid> grid() const;

  // ---- what a twin of the world reads (WorldGym::fields, WorldGym::plants) ----
  // Each entity's plant as a model (entity_plant), in the order entities() gives them.
  std::vector<entity_plant> plants() const;
  // Whether the navigators keep their drive's telemetry each tick (cvc::nav::sim_world's
  // set_capture_drive_telemetry), so plants() reports each entity's drive (entity_drive). Off by
  // default; on or off, the drive steers the same.
  void set_drive_telemetry(bool on);
  bool drive_telemetry() const;
  // The navigator's own settings (not ready without a grid).
  navigator_params navigator() const;
  // The navigator's signed distance (m) on its grid (cvc::nav::signed_distance of grid()), rows *
  // cols; empty without a grid.
  const std::vector<float> &sdf() const;

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
