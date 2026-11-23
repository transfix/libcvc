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
#include <cvc/ariadne/vehicle_kinematics.h> // kinematic_drive
#include <cvc/ariadne/world_rules.h>        // vehicle_archetype
#include <cvc/ariadne/world_spawns.h>
#include <cvc/nav/detail/dyn_4w.h> // the models a twin steps: dyn4_params, dyn_bike_params, bike_veh
#include <cvc/physics/dyn_boat_plant.h> // dyn_boat_desc
#include <cvc/physics/types.h>
#include <cvc/physics/vehicle.h> // vehicle_desc
#include <limits>
#include <map>
#include <memory>
#include <span>
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
//
// An entity with the `pose` controller is not driven at all: an outside simulation says where it
// is (entity_pose), and no plant integrates it. Its chassis is a kinematic box in the physics
// world, moved to each pose at the next step, so it is drawn with its archetype's look and label,
// casts shadows, is picked, pushes what it meets and is a vehicle the others steer around. The
// poses come from its keys (`entities.<id>.pose.*`), a packed value of many (`entities.poses`), a
// table a world block names (world: vehicles: poses:, a relay's published namespace, whose
// entries may spawn the entities too), or set_pose() / set_poses().

// An archetype's vehicle: its preset with the archetype's overrides on top, then a spawn's mass
// and top speed (0: the archetype's), each held to the archetype's range. A boat's (dynamicboat)
// is a stand-in for what reads a vehicle: its hull's box, mass, top speed, the thrusts per unit
// mass as its acceleration and braking, the lock of its plan, and no wheels. Throws
// std::invalid_argument for a preset that is not one.
physics::vehicle_desc vehicle_desc_of(const vehicle_archetype &a, double mass = 0.0,
                                      double max_speed = 0.0);
// How high a vehicle's chassis's centre stands over the ground under it (m): half its box's height
// plus its front wheels' radius (0.4 m with no axles). Where a kinematic box and a `pose` entity
// whose pose gives no z stand, and how far below a `pose` entity's chassis its drawn model's base
// hangs.
double stand_height(const physics::vehicle_desc &d);
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
struct SceneData;
// Each archetype's look as a view can draw it wherever it runs: a model by name found (a fitted
// file), a document asset's file, a file beside the document (`doc_dir`), a data: entry's package
// (`data`: its uri and path: copied into the look, so it is whole without the document; a view
// loads it, cvcGL's vehicle_model_state), and the default look made explicit (the Humvee model for
// a humvee when it is installed, else the box). A look that cannot be found is drawn as its box,
// with a warning.
void resolve_vehicle_looks(std::vector<vehicle_archetype> &archetypes, const std::string &doc_dir,
                           const std::vector<SceneAsset> &assets,
                           std::vector<std::string> *warnings, const std::vector<SceneData> &data);
// The same with no data: entries (a data look then has nowhere to come from unless it arrived
// resolved).
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
  // nav@1, none, external (its intent alone drives it) or pose (an outside simulation places it:
  // entity_pose); "" the archetype's first
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
  // The participant driving it (its pid; ARIADNE_WORLDS.md section 18.4), or "" for its controller.
  // Set by WorldVehicles::set_driver (a holon's set_driver), not at spawn.
  std::string driver;
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

// Where an outside simulation puts a `pose` entity, kept until the next:
//
//   x, y       world metres: its chassis's centre (entity_info's x, y), or its rear axle (the
//              navigator's reference point, as entity_spec's) when `rear_axle`
//   z          world metres, its chassis's centre; not finite: standing on the ground under it,
//              as a kinematic box stands (the ground, its wheels' radius, half its height)
//   heading    rad from +x, counter-clockwise (entity_info's yaw)
//   pitch      rad about its own y (+ nose down), then `roll` about its own x (+ right side down):
//              its rotation is Rz(heading) Ry(pitch) Rx(roll)
//   speed      m/s along its heading: its speed in entity_info, and its wheels roll at it
//   steer      rad, + left: its steered wheels' angle
//   time       s on the outside simulation's clock; not finite: none (below)
//
// A pose without a time is where the entity is at the end of the next physics step. Timed poses
// are drawn between the last two: from the step a pose timed t1 is taken at, the entity moves
// from the one before it (t0) toward it, reaching it t1 - t0 world seconds later (its angles along
// the shorter arc, its speed and steer between the two), and holds there if the next is late. So
// poses that arrive one tick of the outside simulation apart are drawn smoothly, a tick behind.
// A pose farther than WorldVehicles::pose_jump() from the last (a respawn), the first timed pose
// and one whose time is earlier than the last's are taken at once; one with the last's time again
// is the same pose, and changes nothing.
struct entity_pose {
  double x = 0.0, y = 0.0;
  double z = std::numeric_limits<double>::quiet_NaN();
  double heading = 0.0, pitch = 0.0, roll = 0.0;
  double speed = 0.0, steer = 0.0;
  double time = std::numeric_limits<double>::quiet_NaN();
  bool rear_axle = false;
};

// One entity's pose in a batch (WorldVehicles::set_poses).
struct entity_pose_update {
  std::uint64_t id = 0;
  entity_pose pose;
};

// A packed pose value's records (`entities.poses`, `<poses table>.poses`), one a line (or
// separated by ';'): the entity's key (its id in `entities.poses`; in a table its entry's key, its
// id or its name; no spaces, commas or ';'), then x y z heading pitch roll speed steer time,
// numbers as text ("nan" where z or the time is not given), and `rear` last for a rear axle's
// (x, y). Text, so a relay carries it.
// pack_poses writes them (17 significant digits, a pose round-trips exactly); parse_poses reads
// them, skipping a record that is not one (false when any was skipped).
std::string pack_poses(const std::vector<std::pair<std::string, entity_pose>> &records);
bool parse_poses(const std::string &text, std::vector<std::pair<std::string, entity_pose>> &out);

// A wheel as it is drawn, in its vehicle's chassis box's frame (m from the box's centre: x
// forward, y left, z up): its centre, its steering angle (rad, + left), whether it is on the
// ground, and how far it has rolled about its axle (rad in (-pi, pi], + forward; a `pose`
// entity's, at its pose's speed; 0 on the plants). Front to back by axle, each axle's left wheel
// first.
struct entity_wheel {
  double x = 0.0, y = 0.0, z = 0.0;
  double steer = 0.0;
  bool down = true;
  double spin = 0.0;
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
                                     // spec.plant; a `pose` entity's is its look's alone)
  std::string fleet;                 // the fleet it was laid out in (start: fleet:), "" none
  int rally = -1;                    // its team's rally point in its fleet, -1 none
  // Its chassis's contacts over the physics steps since the navigator's last tick (each step's
  // contact events, begun or, when the scene's physics reports them, persisting): with the
  // world's bodies other than the ground (buildings, walls, props), and with another vehicle. A
  // kinematic box's obstacles are its footprint on the grid's blocked cells.
  int obstacle_contacts = 0;
  bool vehicle_contact = false;
  // The command its plant took on the last navigator tick: rad and m/s^2, as entity_intent's.
  double command_steer = 0.0, command_accel = 0.0;
  entity_intent intent; // as of the last navigator tick
  // A `pose` entity: the pose its chassis is at after the last step (between two timed poses,
  // where it is drawn), with x, y, z and the angles its chassis's centre's; and whether it has had
  // one (it is not placed before its first).
  entity_pose pose;
  bool posed = false;
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

// A fleet's vehicles at one world tick (WorldVehicles::poses, FL1): each one's entity id, its
// chassis's centre (world metres) and heading (rad, 0 along +x, counter-clockwise), its speed
// along that heading (m/s, negative backing up) and its team (its rally point, -1 none), in the
// fleet's order, as flat arrays of the same length; and the box they all lie in.
struct fleet_poses {
  std::uint64_t tick = 0; // the physics world's step these are of
  std::vector<std::uint64_t> id;
  std::vector<float> x, y, z, yaw, speed;
  std::vector<std::int16_t> team;
  double min_x = 0.0, min_y = 0.0, max_x = 0.0, max_y = 0.0; // all 0 with none placed
  std::size_t size() const { return id.size(); }
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
  // Who drives entity `id`: a participant's pid, or "" for its controller (section 18.4). The
  // holon that runs the world sets it (holon::set_driver) and feeds the participant's drive axes
  // in as the entity's intent. A change is a new revision (room_entities sends it). False for an
  // unknown id.
  bool set_driver(std::uint64_t id, const std::string &pid);
  std::string driver(std::uint64_t id) const;
  bool clear_intent(std::uint64_t id);
  entity_intent intent(std::uint64_t id) const;
  // A second table of intents beside the entities' own keys (world: vehicles: intents:): an
  // absolute state path whose `<path>.<id>` (or `<path>.<name>`, for a name that is a state name)
  // holds `intent.*` and `force.*` as an entity's keys do. A relay's published namespace, so a
  // server document's program steers the entities of the worlds its clients run. "" none.
  void set_intents_key(std::string key);
  const std::string &intents_key() const;

  // A `pose` entity's pose (entity_pose), from now until the next: any thread may call these. It
  // is taken at the next physics step, after the pose keys. False for an id that is not a `pose`
  // entity, or a pose whose x, y, angles, speed or steer are not finite. set_poses() sets many
  // under one lock and returns how many it took; pose() is the last one given (false: none yet).
  bool set_pose(std::uint64_t id, const entity_pose &pose);
  std::size_t set_poses(std::span<const entity_pose_update> poses);
  bool pose(std::uint64_t id, entity_pose &out) const;
  // A table of poses beside the entities' own keys (world: vehicles: poses:): an absolute state
  // path whose `<path>.<key>.pose.*` places the entity `<key>` names (its id, its name when that is
  // a state name, or the entry it was spawned from), as `entities.<id>.pose.*` does, and whose
  // `<path>.poses` is a packed value of many (pack_poses). An entry that names an archetype
  // (`<path>.<key>.archetype`, with `name`, `team`, `color` and `color2` beside it) spawns a `pose`
  // entity of it, and its removal (or an empty archetype) removes that entity: so a server
  // document that publishes the table puts the entities it simulates in every client's world.
  // "" none.
  void set_poses_key(std::string key);
  const std::string &poses_key() const;
  // How far (m) a pose may be from the last one for the entity to move there along the way (over
  // a step, or between two timed poses); a farther one is taken at once, as a respawn. 20 m by
  // default; not finite or <= 0 throws std::invalid_argument.
  void set_pose_jump(double metres);
  double pose_jump() const;

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
  // removal, a clear; and the poses table's entries, spawning and removing the entities they name.
  void poll();
  // Around each physics step. Before it: the `pose` entities' poses (their keys, the packed
  // values, then set_pose's), each chassis moved to where it is at the step's end; then every k-th
  // (k = nav's tick / the step) runs the navigators' tick, which commands every vehicle for the
  // steps until the next; that tick reads the intent keys first (an intent key's change replaces
  // that entity's intent, as set_intent() does). Then the dynamic models' substep, and the
  // kinematic boxes a step along the bicycle's path. After it: the dynamic models take back what
  // the step's contacts did. The `pose` entities need no grid: they are placed at their first
  // pose, on the world's ground when it has one.
  void before_step();
  // The same, for a step ending on CLOCK_TICK of the clock the world steps on (SceneWorld::follow
  // passes it): the navigator's tick then also notes the clock tick it ran on, so kinematics()
  // can say which clock ticks it runs on.
  void before_step(std::uint64_t clock_tick);
  // After it, too: the step's contacts, counted on the vehicles they touch.
  void after_step();
  // The entities' keys and the selection's, now.
  void publish();
  // The same, but what changes every step (each entity's pose, speed, books and command, the
  // summary, the fleets' keys) only when `telemetry` is true: the world writes those at its
  // telemetry rate (SceneWorld::follow, `physics.telemetry_hz`). A newly listed entity's keys,
  // and all of them after a change of the entities (a spawn, a placing, a removal: rev), are
  // written either way. Only what changed is written. True when the listed entities (`ids`)
  // changed.
  bool publish(bool telemetry);

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
  // Other things that move in the world, which the navigators steer around as they do around each
  // other's vehicles while avoid() is on (ARIADNE_WORLDS.md section 18.4): a holon's avatars
  // (holon::attach_world sets them before each step), at their positions in the world's metres.
  // Nothing moves them; kept until set again, empty for none (the default).
  void set_movers(std::vector<std::array<double, 2>> xy);
  const std::vector<std::array<double, 2>> &movers() const;
  // What the navigators know of the world (the vehicles.sense key, "known" or "fog", sets it
  // too, read each poll; a document's init: may set it before the world is made):
  //   known  the world's grid, whole (the default): nothing to sense
  //   fog    nothing at first: each plant's navigator builds a belief of the grid from its
  //          vehicles' rays (60 m, 120 of them), as libcvc's fog demos do, and plans on that
  // Changing it makes the navigators again: every vehicle goes back to its start. False for a
  // mode it does not know.
  bool set_sense(const std::string &mode);
  const std::string &sense() const;
  // The occupancy plant `plant`'s navigator plans on now (row-major, the grid's rows and cols;
  // 1 = blocked): the grid under known, its belief (its first plane) under fog. A fleet's own
  // navigator is "<plant>/<fleet>". False when that plant has no navigator yet.
  bool belief(const std::string &plant, std::vector<std::uint8_t> &out, int &rows, int &cols) const;
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
  // Entity `id` on the kinematic plant as a client runs it (ARIADNE_WORLDS.md section 18.5, OW5b;
  // vehicle_kinematics.h): its bicycle's numbers, its envelope and the clock ticks the navigator
  // ticks on. False for one on another plant, not in the world, or before the navigator has
  // ticked on a step with a clock tick (before_step(clock_tick)). The phase it reports changes
  // only when the navigator is built again; revision() moves when it does.
  bool kinematics(std::uint64_t id, kinematic_drive &out) const;
  // The navigator's tick to use (s). By default the world's: cvc::sim_tick of its physics step
  // (cvc/core/sim_tick.h), kSimTick (7/120 s) at the default 120 Hz, 4/60 s at 60 Hz. It takes
  // effect when the navigator is next built (a new grid, a restart: WorldGym::reset), held to the
  // nearest whole number of physics steps (the status says when it is not one). Throws
  // std::invalid_argument for a tick that is not finite and > 0.
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

  // Fleets (world: vehicles: start: entries with fleet:, spawn_starts): their names in the
  // document's order. A fleet is laid out once the world's grid exists; its keys are
  // `vehicles.fleets.<name>.*` (rally points, restart and reseed requests, its counts).
  // Each fleet has a navigator of its own per plant ("<plant>/<fleet>"), made with its sense and
  // belief: writing `.sense` (known, fog) or `.belief` (shared, grouped, private) makes it again,
  // its vehicles at rest where they are; `.planes` counts its belief planes. `.policy` (trained,
  // builtin, data:<entry> or a uri) switches what drives it live, and `.drives` says what does:
  // "trained (<where>, sha256:...)", "weights (<uri>, sha256:...)", "builtin", "builtin: loading
  // <uri>" while weights are on their way, or "builtin: no trained weights (<why>)", which the
  // host's log also gets once.
  // `.plant` (fleet_plants::text: "jolt", "jolt, kinematic", "0: jolt, kinematic") switches its
  // plants, its vehicles at rest where they are on the new ones (a bad one is said in
  // vehicles.status and changes nothing). `.metric` (trained, vehicle, policy) is what it plans
  // with (D9), and `.plans` what it does now: "trained" (the trainer's vehicle, while weights drive
  // or are on their way) or "vehicle" (each plant's own); a change makes its navigators again.
  // `.tick` is the physics world's step its keys are of, and `.bounds` the box its placed
  // vehicles lie in ("min_x,min_y,max_x,max_y", world metres; "" with none placed).
  std::vector<std::string> fleets() const;
  // Fleet `fleet`'s rally point `team` (0 to rally.count - 1): where it is, false when there is
  // none (no such fleet or team, or not laid out yet).
  bool rally(const std::string &fleet, int team, double &x, double &y) const;
  // Moves it (held to the grid): its team's vehicles drive there. As writing
  // vehicles.fleets.<fleet>.rally.<team>.x and .y.
  bool move_rally(const std::string &fleet, int team, double x, double y);
  // Lays fleet `fleet` out again, every vehicle back to a start: from its seed (`reseed` false),
  // or from the next one. As changing vehicles.fleets.<fleet>.restart.request or
  // reseed.request.
  bool restart_fleet(const std::string &fleet, bool reseed);
  // Fleet `fleet`'s vehicles as the last publish() left them (fleet_poses), for a view that draws
  // hundreds without reading state: lock-free, from any thread, the snapshot the caller holds
  // unchanged while later steps make new ones. Null for a name that is not a fleet's, and before
  // its first publish.
  std::shared_ptr<const fleet_poses> poses(const std::string &fleet) const;
  // The start: entries spawn_starts has been given since the last clear(), in order: after a load,
  // the document's (a gym keeps them to lay its fleets out again at each reset,
  // gym_options::fleets).
  const std::vector<vehicle_start> &starts() const;
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
