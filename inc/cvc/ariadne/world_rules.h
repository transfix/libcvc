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

#ifndef __CVC_ARIADNE_WORLD_RULES_H__
#define __CVC_ARIADNE_WORLD_RULES_H__

#include <array>
#include <cvc/ariadne/avatar_physics.h> // avatar_body, avatar_energy_policy
#include <cvc/physics/types.h>
#include <map>
#include <string>
#include <vector>

namespace cvc {
class app;
namespace ariadne {

struct LoadResult;
struct Scene;
struct Value;

// ----------------
// The world's rules (ARIADNE_WORLDS.md §3, WD2)
// ----------------
// What a room's avatars move under: the world document says it, the relay publishes it as four
// revisioned sections (room_rules.h), and every holon -- the relay's and each client's -- runs
// under the same one. Each section takes effect at its activation tick (holon.h).
//
//   physics      the world's gravity (from the document's scene: physics:)
//   archetypes   the bodies avatars may have, and the controllers each allows
//   policy       the movement veto (max_speed, allow_vertical) and the energy budget
//   controllers  what turns an intent into forces: a kernel by name and version, and its params
//
// Every avatar has the first archetype and moves with the first controller it allows (WD2; a
// participant's own pick comes with WD4 and WD6). default_world_rules() is what the conference
// had compiled in before: a 70 kg, 0.5 x 0.5 x 1.7 m person, a 4 m/s cap, and the walker, now
// walker@2 (walker@1 on land; wading and swimming in water, OW9c).

struct world_physics {
  cvc::physics::vec3 gravity{0.0, 0.0, -9.80665}; // m/s^2
  // The room's wind (the document's scene: wind:, scene_wind.h, O14): its speed (m/s; 0, the
  // default, is none), the way it blows (degrees, from +x toward +y), its gustiness and the level
  // its height profile is over. Every holon pushes its avatars with it alike.
  double wind_speed = 0.0, wind_dir = 0.0, wind_gustiness = 0.0, wind_base = 0.0;
  bool operator==(const world_physics &o) const {
    return gravity.x == o.gravity.x && gravity.y == o.gravity.y && gravity.z == o.gravity.z &&
           wind_speed == o.wind_speed && wind_dir == o.wind_dir &&
           wind_gustiness == o.wind_gustiness && wind_base == o.wind_base;
  }
};

struct avatar_archetype {
  std::string id = "person";
  avatar_body body;
  std::vector<std::string> controllers{"walker@2"}; // controller ids; the first is the avatar's
  // An avatar that is a vehicle (ARIADNE_WORLDS.md section 18.6, OW6c): the id of one of the
  // world's vehicle archetypes (world: vehicles: archetypes:). A participant with this archetype
  // has no body of its own: the server spawns that vehicle for it and it drives it for good. ""
  // is a person, with `body` and `controllers`.
  std::string vehicle;
  bool operator==(const avatar_archetype &) const = default;
};

struct avatar_controller {
  std::string id = "walker@2";
  std::string kernel = "walker";
  int version = 2;
  // The kernel's knobs. The walker's walk_speed (m/s) is what a client's walk keys ask for; the
  // run a client may add is capped by the policy's max_speed. walker@2's swimming knobs
  // (avatar_swim, avatar_physics.h; ARIADNE_WORLDS.md section 18.10.2) are walker@1's walker in
  // the water; walker@1 has only walk_speed (and jump_speed, 4 when left out).
  std::map<std::string, double> params{
      {"walk_speed", 2.0},   {"wade_depth", 0.3},  {"swim_depth", 1.2},   {"wade_slow", 0.6},
      {"swim_speed", 1.0},   {"swim_sprint", 1.8}, {"swim_force", 300.0}, {"dive_speed", 0.8},
      {"climb_height", 0.6}, {"swim_cost", 0.8}};
  double param(const std::string &name, double def) const;
  bool operator==(const avatar_controller &) const = default;
};

struct avatar_policy {
  avatar_energy_policy energy;
  // The veto (holon::sanitize_intent): the fastest an intent may ask an avatar to move horizontally
  // (m/s), and whether it may ask to move vertically at all (avatars walk; gravity owns z).
  double max_speed = 4.0;
  bool allow_vertical = false;
  bool operator==(const avatar_policy &) const = default;
};

// How a vehicle archetype is drawn (ARIADNE_WORLDS.md §14): its chassis box in the entity's two
// colours (`box`, the default), or a mesh from a file or a document asset, placed on the chassis
// by `scale`, `yaw` (degrees, about +z) and `z` (m, up from the chassis box's centre). The physics
// body is the archetype's either way.
//
// `model: humvee` is a vehicle model by name: the vehicle-humvee package's Humvee.glb, found where
// cvcpkg installs it. `fit: true` (a model's default) orients the mesh (its smallest extent up,
// its longest forward, the nose the end whose top sits lower) and scales it to the chassis box's
// length, standing on the ground under the chassis. With no look:, a humvee is drawn as the
// Humvee in parts when the package's humvee/chassis.glb, humvee/wheel.glb and humvee/rig.json are
// installed (a parted look, below: its wheels turn), else as the one-piece Humvee model when that
// is installed, and as its box otherwise. `model: humvee` is the parted Humvee too, unless it has
// scale:, yaw: or z:, which only the one-piece model takes. A look that names a file (`file:`, or
// `data:` with `file:`) is that file.
//
// A parted look (`chassis:`, `wheel:`, `rig:`) is a model in parts: the chassis's mesh without its
// wheels, one wheel's mesh, and the rig that says where the wheels go (vehicle_rig). Each wheel is
// drawn on its own at its hub, turned by its steer and rolled by its spin, so its wheels turn as
// the vehicle drives. With `data: NAME` the three are files in that data: entry's package.
//
// A parted look's rig (`rig:`, a map inline or a file of one: YAML, or JSON, which is YAML too):
// where its model's wheels are, in the frame its chassis and wheel files share (metres; x forward,
// y left, z up; with `up: y`, glTF's: +y up, +z forward, +x left).
//
//   radius   the wheel mesh's radius (or `wheel: { radius: }`); none: half the wheel mesh's larger
//            extent across its axle
//   hubs     each wheel's hub, [x, y, z]; or a map whose `xyz:` lists them and whose `order:`
//            names them (for steer:)
//   steer    the hubs that steer, by index or by name; none given: the front axle's (the hubs
//            whose x is within 5% of the hubs' length of the largest; with one axle, none)
//   mirror   how the wheel mesh (a left wheel, its outside toward +y) is drawn on the right (hubs
//            with y < 0): turn (half a turn about its upright, the default), reflect (mirrored
//            across its xz plane) or none (as it is)
//   up       z (the default) or y
//
// Any other key is ignored (a rig file's provenance: source:, frame:, size:, ...).
struct vehicle_rig {
  double radius = 0.0; // 0: from the wheel mesh
  std::vector<std::array<double, 3>> hubs;
  std::vector<bool> steers; // per hub (the front axle's when steer: is not given)
  std::string mirror = "turn";
  bool y_up = false;
  bool operator==(const vehicle_rig &) const = default;
};

// A rig from its map: false, and why, when it is not one (no hubs, a hub that is not three
// numbers, a steer that names no hub, a mirror or up it does not know, a radius not > 0).
bool parse_vehicle_rig(const Value &v, vehicle_rig &out, std::string *why = nullptr);
// A rig as a map's text, on one line in braces (YAML's flow style: parse_value and
// parse_vehicle_rig read it back, numbers to 17 digits): a catalog's look.rig key carries an
// inline rig so.
std::string vehicle_rig_text(const vehicle_rig &rig);

struct vehicle_look {
  std::string kind; // box, file, asset, model or data; "" the default (above)
  std::string path; // the file, the asset's id, the model's name, or (data) the file in the package
  // data: the document's data: entry it names (`look: { data: NAME, file: FILE }`); then, once
  // resolve_vehicle_looks has run, the entry's source and its path: inside the package, so the
  // look is whole wherever it travels (a client has no document): a view loads `uri`
  // (cvcpkg://ORG/NAME, https://..., a file or directory), keeps `subpath`, and reads `path` there.
  std::string data, uri, subpath;
  double scale = 1.0, yaw = 0.0, z = 0.0;
  bool fit = false;
  bool texture = true; // false: a model drawn in its entity's colours, not its texture (`texture:`)
  // A parted look (kind file, or data with `data:`): the chassis's file and the wheel's, and its
  // rig, given inline (`rig`) or read from `rig_file` (in the package, for a data look) when the
  // view draws it. Fitted to the vehicle always: scale, yaw and z do not apply. Its `path` is
  // unused, but for the default Humvee in parts, whose path is the one-piece model (when it is
  // installed) for a reader that draws one model only (a relay's clients).
  std::string chassis, wheel, rig_file;
  vehicle_rig rig;
  bool parted() const { return !chassis.empty(); }
  bool operator==(const vehicle_look &) const = default;
};

// A vehicle the world offers (ARIADNE_WORLDS.md §14, L2), beside the avatars' archetypes:
//
//   plant        what drives it, in the world's own physics: jolt (the Jolt vehicle), kinematic
//                (nav's bicycle), dynamic (the single-track model), dynamic4w (the four-wheel
//                model with suspension), dynamicboat (a boat, cvc::physics::dyn_boat_plant, on the
//                world's water); dynamic:<preset>, dynamic4w:<preset> and dynamicboat:<preset>
//                name the vehicle as well, as `vehicle: <preset>` would
//   vehicle      its body: a preset (humvee, roboracer; a boat's launch or jonboat), with any of
//                `overrides` on top (a boat takes mass and max_speed)
//   look         how it is drawn
//   ranges       what a spawn may set, by setting: mass (kg) and max_speed (m/s), each [lo, hi]
//   controllers  what may drive it: nav@1 (the navigator), none (it holds still), external (its
//                intent alone: WorldVehicles::set_intent, the entity's intent keys) or pose (an
//                outside simulation places it, and no plant integrates it: WorldVehicles::set_pose,
//                the entity's pose keys); the first is a spawn's default
//   drivable     whether a participant may get in and drive it (ARIADNE_WORLDS.md section 18.6;
//                holon::set_driver), from within `reach` metres of its chassis box (3 m)
//   controls     the control scheme a driver drives it with (controls.h): car or boat; "" by its
//                plant (a dynamicboat's boat, every other car)
struct vehicle_archetype {
  std::string id;
  std::string plant = "jolt";
  std::string preset = "humvee";
  std::map<std::string, double> overrides; // mass, max_speed, max_accel, max_decel, max_steer
  vehicle_look look;
  std::map<std::string, std::pair<double, double>> ranges;
  std::vector<std::string> controllers{"nav@1", "none"};
  bool drivable = false;
  double reach = 3.0;
  std::string controls;
  bool operator==(const vehicle_archetype &) const = default;
};
// The control scheme a driver drives A with: its controls, or by its plant (boat or car).
std::string vehicle_controls(const vehicle_archetype &a);

// The vehicle presets a `vehicle:` may name, and the controllers and plants a vehicle may have.
// A dynamicboat's presets are the boats' (boat_presets()), every other plant's the vehicles'.
std::vector<std::string> vehicle_presets();
std::vector<std::string> boat_presets();
bool vehicle_controller_known(const std::string &id);
bool vehicle_plant_known(const std::string &plant);
// Whether `plant` is a boat's (dynamicboat): its vehicles move on water, not land.
bool vehicle_plant_floats(const std::string &plant);

// Where vehicles start (ARIADNE_WORLDS.md §14, L3, the `spawns` section): points with a heading
// (degrees from +x, counter-clockwise), and tags a spawn is matched on: a team (-1: any) and an
// archetype ("": any). A point belongs to one of the scene's worlds (`world`, "" every world).
struct spawn_point {
  double x = 0.0, y = 0.0, heading = 0.0;
  int team = -1;
  std::string archetype;
  std::string world;
  bool operator==(const spawn_point &) const = default;
};

// The points and the policy a spawn without a place of its own follows: `fixed` (the first point
// that matches), `random`, `round_robin` or `nearest_free` (the first matching point no vehicle
// is near), with the random draw and the round-robin cursor kept per `scope`: team, archetype or
// all.
//
// Each vehicle starts on its medium (its plant's, cvc::nav::plant_medium), by the depth of the
// still water there (nav::world_grid's water layer):
//   land   no water deeper than `ford_depth` (m): a car does not start in a lake;
//   water  at least its draft and `keel_clearance` (m) of water, and open water round it for
//          half its hull's length and `shore_clearance` (m) more: a boat does not start aground.
// A point tagged with an archetype serves it where its medium allows; an untagged one serves only
// the archetypes whose medium it is on.
//
//   world:
//     spawns:
//       policy: round_robin
//       scope: team
//       ford_depth: 0.3
//       keel_clearance: 0.3
//       shore_clearance: 0
//       points:
//         - { at: [10, 5], heading: 90, team: 1, archetype: humvee, world: flat }
struct spawn_rules {
  std::vector<spawn_point> points;
  std::string policy = "random";
  std::string scope = "team";
  double ford_depth = 0.3;
  double keel_clearance = 0.3;
  double shore_clearance = 0.0;
  bool operator==(const spawn_rules &) const = default;
};
bool spawn_policy_known(const std::string &policy);
bool spawn_scope_known(const std::string &scope);

// Vehicles a world starts with (world: vehicles: start:): `count` (1 to 256) of archetype
// `archetype`, spawned when the world's vehicles are made, as a spawn request would make them:
// `name` (with " 1", " 2", ... when count is more than one), `team`, `controller`, `plant` and
// `policy`, empty for the archetype's; at (`x`, `y`) facing `heading` (degrees from +x; without one
// toward its goal) when `placed`, else at a free point (or a spawn point, world: spawns:). As for
// every start, (x, y) is the vehicle's rear axle, the navigator's reference point.
//
//   world:
//     vehicles:
//       archetypes: { rover: { plant: kinematic, vehicle: roboracer } }
//       start:
//         - { archetype: rover, count: 12, team: 1 }
//         - { archetype: rover, name: lead, at: [0, 0], heading: 90 }
//         - { fleet: city, archetype: rover, count: 800, spawn: opposed, rally: { count: 16 } }
//
// An entry with `fleet:` (a state name) is a fleet: up to 4096 vehicles laid out together once the
// world's grid exists, from `seed`. `spawn:` opposed starts them in the grid's west 30 % and gives
// them goals in its east 30 %, scattered (the default) draws both anywhere free, and points starts
// them at the world's spawn points with scattered goals. `rally: { count: T, assign: latitude }`
// makes T teams by their goals' latitude, each team's goal held at its rally point (the goal
// nearest its band's centre) and its vehicles coloured hsv(t / T, 0.74, 0.98) (`colors: hsv`, the
// default; `colors: spawn` keeps the spawn's). A fleet's goals are held (they do not wander).
// `keys: summary` (a fleet's default) publishes the fleet's keys (vehicles.fleets.<fleet>.*), and
// per-entity keys only for the vehicles vehicles.watch names or with an intent; `keys: all`
// publishes every vehicle's, as an unnamed entry's are.
//
// A fleet has a navigator of its own (FLEETS_AND_NIGHT_ROADMAP.md FL1b):
//   sense:   known | fog: what it knows of the world ("" the world's vehicles.sense)
//   belief:  shared (one map) | grouped (max(2, n / 64) maps, teammates together) | private
//            (one each): how its vehicles share what they sense
//   policy:  trained (the default: the published CoefMLP, $CVC_NAV_WEIGHTS, else an installed
//            copy, else downloaded from cvcpkg://cvc/grl-snam-weights), builtin (the built-in net),
//            a .cvcnav's path, or { uri:, path:, sha256: } (a file, an https URL or a cvcpkg://
//            package, the .cvcnav inside it and its hash) or { data: <entry> } (the document's
//            data: entry). Without the weights it drives on the built-in net and says why.
//   metric:  trained (the trainer's vehicle, as the weights learned) | vehicle (each plant's own
//            size and limits) | policy (the default: trained when weights drive or are on their
//            way, vehicle on the built-in net)
//   plant:   one plant, a list its vehicles cycle over ([jolt, kinematic]), or each team's
//            ({ 0: jolt, 2: jolt }; a team not named on the archetype's plant). A fleet's plants
//            all float (dynamicboat) or none do.
struct fleet_plants {
  std::vector<std::string> cycle;   // the vehicles' plants in turn (none: the archetype's)
  std::map<int, std::string> teams; // a team's own (its rally point's index)
  bool empty() const { return cycle.empty() && teams.empty(); }
  // Member `member`'s plant on team `team` (-1 none): its team's, else the cycle's, else "".
  std::string of(int member, int team) const;
  // As parse_fleet_plants reads it: "jolt", "jolt, kinematic", "0: jolt, 2: jolt",
  // "kinematic, 0: jolt" (team 0 on jolt, the rest kinematic).
  std::string text() const;
  bool operator==(const fleet_plants &) const = default;
};
// `text` (fleet_plants::text's form) into `out`: "" or what is wrong. A team must be below
// `teams` (its fleet's rally points; -1 unchecked), and the plants all float or none do.
std::string parse_fleet_plants(const std::string &text, fleet_plants &out, int teams = -1);
bool fleet_metric_known(const std::string &metric); // trained, vehicle or policy

struct vehicle_start {
  std::string archetype, name, controller, plant, policy;
  int team = 0;
  int count = 1;
  bool placed = false;
  double x = 0.0, y = 0.0;
  bool has_heading = false;
  double heading = 0.0;
  // A fleet's (all empty or 0 for an entry of single vehicles).
  std::string fleet;
  std::string layout; // spawn: opposed | scattered | points ("" scattered)
  unsigned long long seed = 0;
  int rally = 0;      // rally: { count: } (0: no teams; each its own goal)
  std::string assign; // rally: { assign: } latitude ("" latitude)
  std::string colors; // hsv | spawn ("" hsv with rally points, spawn without)
  std::string keys;   // summary | all ("" summary)
  std::string sense;  // known | fog ("" the world's)
  std::string belief; // shared | grouped | private ("" shared)
  // policy: { uri:, path:, sha256: } (`policy` the uri) or { data: } (`policy_data`).
  std::string policy_path, policy_sha256, policy_data;
  std::string metric; // trained | vehicle | policy ("" policy)
  // collide: true: a fleet's vehicles kept from passing through each other (nav's hard de-overlap
  // on its own plants; a Jolt plant's bodies collide anyway).
  bool collide = false;
  // righting: whether a fleet's Jolt vehicle that has lain on its side or roof, slow, for
  // `righting_after` s is set back on its wheels where it lies (nav_plant_options::right_after).
  // Off by default: a vehicle that rolls stays down until the fleet is laid out again (a restart,
  // a reseed, another world). `righting: true` is on after 2 s, `righting: <s>` on after that
  // long (0: off).
  static constexpr double kRighting = 2.0;
  bool righting = false;
  double righting_after = kRighting;
  // A fleet's plant: as a list or each team's (`plant` is then the list's first, or "").
  fleet_plants plants;
  bool operator==(const vehicle_start &) const = default;
};
constexpr int kMaxVehicleStartCount = 256;
constexpr int kMaxFleetCount = 4096;
constexpr int kMaxFleetRally = 64;

struct world_rules {
  world_physics physics;
  std::vector<avatar_archetype> archetypes{avatar_archetype{}};
  avatar_policy policy;
  // walker@1 stays defined beside walker@2 (OW9c), first as it always was, so a document whose
  // archetype names walker@1 still runs; the default person allows walker@2.
  std::vector<avatar_controller> controllers{
      avatar_controller{"walker@1", "walker", 1, {{"walk_speed", 2.0}}}, avatar_controller{}};
  // The vehicles a world offers (world: vehicles: archetypes:), none by default. The relay does
  // not read them yet; a local world runtime (SceneWorld) spawns them.
  std::vector<vehicle_archetype> vehicles;
  // Where else their intents may come from (world: vehicles: intents:): a state path, resolved
  // as a bind is (a leading / outside the document's prefix), "" none. WorldVehicles reads
  // `<path>.<id or name>.intent.*` beside each entity's own keys.
  std::string vehicle_intents;
  // And their poses (world: vehicles: poses:), the same way: `<path>.<key>.pose.*` and the packed
  // `<path>.poses` place the `pose` entities, and an entry naming an archetype spawns one.
  std::string vehicle_poses;
  // The vehicles the world starts with (world: vehicles: start:), in order; none by default.
  std::vector<vehicle_start> vehicle_starts;
  // Where they start (world: spawns:).
  spawn_rules spawns;
  // What avatar() and controller() give when the lists above are empty: the default person and
  // walker@2. Left as they are.
  avatar_archetype fallback_avatar{};
  avatar_controller fallback_controller{};
  bool operator==(const world_rules &) const = default;
  const vehicle_archetype *find_vehicle(const std::string &id) const;

  // The person every avatar's body is: the first archetype that is not a vehicle (the default
  // person when there is none). A participant whose archetype is a vehicle drives that instead
  // (holon::set_archetype), and is this person when it gets out of it.
  const avatar_archetype &avatar() const;
  // An archetype by id; null for none.
  const avatar_archetype *find_avatar(const std::string &id) const;
  // The avatar's controller: the first its archetype allows, as `controllers` defines it (the
  // default walker@2 when that is missing).
  const avatar_controller &controller() const;
  const avatar_controller *find_controller(const std::string &id) const;
};

world_rules default_world_rules();

// Whether this build runs KERNEL@VERSION: walker@1 and walker@2 so far (WD4 brings a catalog). A
// client never checks: an older build predicts walker@2 as walker@1 (the same on land) and is
// corrected in water.
bool controller_kernel_known(const std::string &kernel, int version);

// "" or what is wrong with RULES: an archetype's body that is not finite and positive, an id
// repeated or not a name, an archetype allowing no controller or one `controllers` does not define,
// a kernel this build does not run, walker@2's knobs out of range (each >= 0, wade_slow at most 1,
// wade_depth below swim_depth), a policy limit that is negative or not finite.
std::string check_world_rules(const world_rules &rules);

// The world: block (ARIADNE_WORLDS.md §2), on top of RULES:
//
//   world:
//     avatars:
//       archetypes:                 # replaces the default list; the first is every avatar's
//         person: { body: { mass: 70, width: 0.5, height: 1.7, density: 985 },
//                   controllers: [walker@2] }
//       policy:                     # each key given replaces the default
//         max_speed: 4.0
//         allow_vertical: false
//         energy: { enabled: true, force_max: 840, power_max: 1200, power_rest: 300,
//                   reserve_max: 30000, transport_cost: 0.2, empty: rest, recover_fraction: 0.25 }
//     controllers:                  # added to (or replacing, by id) the defaults walker@1, @2
//       walker@2: { kernel: walker, version: 2, params: { walk_speed: 2.0, wade_depth: 0.3,
//                   swim_depth: 1.2, wade_slow: 0.6, swim_speed: 1.0, swim_sprint: 1.8,
//                   swim_force: 300, dive_speed: 0.8, climb_height: 0.6, swim_cost: 0.8 } }
//     vehicles:
//       archetypes:                 # the vehicles a spawn may pick, in this order
//         humvee:
//           plant: jolt
//           vehicle: humvee         # a preset, or { preset: humvee, mass: 3000, max_speed: 20 }
//           look: { file: humvee.glb, scale: 1, yaw: 90, z: -0.5 }   # default: the chassis box
//           ranges: { mass: [2000, 3500], max_speed: [5, 24.6] }
//           controllers: [nav@1, none]
//       intents: /drive             # optional: a second table of intents (a relay's namespace)
//       poses: /fleet               # optional: a table of the pose entities' poses (the same)
//
// "" or what is wrong (check_world_rules included). A key it does not know is a warning, pushed
// onto WARNINGS when given, and so is avatars: catalog:, which the relay's seeded catalog still
// stands in for.
std::string parse_world_block(const Value &content, world_rules &rules,
                              std::vector<std::string> *warnings = nullptr);

// The world: block's parser (register_ari_block) on `app`; idempotent. Without it the loader
// skips a world: block. arisrv, AppRuntime and ServerDocument register it.
void register_world_block(cvc::app &app);
// The world: block a load kept (register_world_block), or null.
const Value *find_world_block(const LoadResult &lr);

// A world document's rules: the defaults, its scene's physics: gravity, then its world: block
// (WORLD, null for none). A physics: fixed_dt other than the room's quantum (1/120 s, or 1/hz for
// a scene: clock: hz), a backend or materials are warnings: the avatars step one quantum at a
// time, and bodies other than the avatars' wait for WD3. "" or what is wrong.
std::string world_rules_from(const Scene &scene, const Value *world, world_rules &out,
                             std::vector<std::string> *warnings = nullptr);

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_WORLD_RULES_H__
