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

#ifndef __CVC_ARIADNE_AVATAR_PHYSICS_H__
#define __CVC_ARIADNE_AVATAR_PHYSICS_H__

#include <cstdint>
#include <cvc/algo/wind_field.h>
#include <cvc/ariadne/avatar_sim.h> // avatar_intent
#include <cvc/physics/body.h>
#include <cvc/physics/field.h> // field_id
#include <cvc/physics/types.h>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace world {
class world_geometry;
}
} // namespace cvc

namespace cvc {
namespace physics {
class world;
struct world_config;
class flow_velocity;
} // namespace physics
namespace ariadne {

// ----------------
// cvc::ariadne::avatar_world (H1 physics)
// ----------------
// The room's avatars as rigid bodies in a Jolt cvc::physics::world: each avatar is a dynamic box
// with a mass, under gravity, colliding with the other avatars and with the room's static objects
// (the ground it stands on, the reference cube, ...). The holon steps it once per quantum: the
// server with every avatar dynamic (the authority), a client with its OWN avatar dynamic and each
// peer a kinematic body held at the pose it is shown at (so the client predicts bumping into it;
// what the peer does in return is the server's call).
//
// An intent DRIVES its avatar (drive()) through its CONTROLLER (ARIADNE_CONFERENCE_ROADMAP.md §5k):
// what turns a participant's intent into forces on its body. There is one so far, the walker: the
// horizontal velocity eases toward the intent's, pushed with at most the energy policy's
// force_max (so a heavier body accelerates more slowly), and the body is held upright facing the
// intent's yaw. Everything else -- gravity, a push from a heavier avatar, stopping at the cube --
// is the solver's. The avatar's material is frictionless (the drive is its grip on the floor), so
// a still avatar shoved by another slides and the drive brings it back to rest.
//
// The controller pays for what it does from the avatar's reserve (avatar_energy_policy, the same
// for every avatar): the positive work it does on the body, plus carrying the body along. The
// server sets the policy, and a client predicts under the same one, so both spend alike.
//
// Poses in and out are the avatar's FEET (body_state.position at floor level, as holon_sync
// replicates and room_view draws); the box's centre is half its height above them.
//
// ONE WORLD (ARIADNE_WORLDS.md section 18.2): an avatar_world made over a world it does not own
// (the shared-world constructor: a server document's SceneWorld, with its statics, its vehicles and
// its other bodies) puts the avatars in that world. Its owner steps it, and its gravity and air are
// the owner's: step(), set_statics(), set_wind() and set_time() are then not this class's to do
// (step and set_statics throw std::logic_error; the wind and the time are the owner's, no-ops), and
// set_gravity() only prices the walk. Avatars there meet everything else in the world: a car
// shoves them, and they stop what they are heavy enough to stop.
//
// The world runs in cvc::physics' cross-platform determinism mode, so the server (Linux) and a
// client (Windows, wasm) stepping the same inputs from the same state agree -- what client
// prediction relies on. Creation order is part of a Jolt run, so statics are always created first,
// in the order given.

// An avatar's physical body: its archetype's (world_rules.h), the same for every avatar so far.
struct avatar_body {
  double mass = 70.0;  // kg
  double width = 0.5;  // m, the box's x and y edge
  double height = 1.7; // m
  // kg/m^3, a person's (OW9b, ARIADNE_WORLDS.md section 18.10): in water the box displaces
  // mass / density, not its own volume, so a person floats head-out (98.5% of the box under)
  // rather than like a cork. The box stays the shape for contact and prediction.
  double density = 985.0;
  bool operator==(const avatar_body &) const = default;
};

// The share of an avatar's box that displaces water (physics::body_desc::displacement): its mass
// over the body's density, over the box's volume. 0.167 for the default person.
double avatar_displacement(const avatar_body &body, double mass);

// What an avatar's controller may spend (§5k). The SERVER sets it, the same for every avatar, and
// publishes it in the room (room_avatar.h) so a client predicts under the same rule.
struct avatar_energy_policy {
  // false: no budget, only the force limit (the reserve stays full).
  bool enabled = true;
  // The most the controller pushes the body with, whatever its mass, in N: 840 N moves the
  // default 70 kg at 12 m/s^2, a 140 kg body at 6.
  double force_max = 840.0;
  double power_max = 1200.0;    // W: the most it spends in a step
  double power_rest = 300.0;    // W: what refills the reserve, and all an empty one leaves
  double reserve_max = 30000.0; // J
  // Carrying the body along costs transport_cost x m g |v| (horizontal), a walking human's cost
  // of transport, so a steady walk on the frictionless floor is not free: a 70 kg walker
  // sustains about 2.2 m/s on resting power.
  double transport_cost = 0.2;
  // When the reserve runs out: `rest` leaves resting power (it slows to what that sustains);
  // `stop` stops it speeding up at all until recover_fraction of the reserve has refilled.
  enum class when_empty { rest, stop };
  when_empty empty = when_empty::rest;
  double recover_fraction = 0.25;
  bool operator==(const avatar_energy_policy &) const = default;
};

// Where an avatar's reserve went since it was added, in J (the energy books). They balance:
//   reserve now = start + refilled - work - carried - overflow + unpaid + corrections
struct avatar_ledger {
  double start = 0.0;       // the reserve it was added with
  double refilled = 0.0;    // power_rest over every drive
  double work = 0.0;        // positive work on the body: speeding up, pushing
  double carried = 0.0;     // carrying the body along (transport_cost x m g |v|)
  double overflow = 0.0;    // refill a full reserve could not hold
  double unpaid = 0.0;      // spending an empty reserve could not cover (a `stop` coming to rest)
  double corrections = 0.0; // set_reserve's changes: the server correcting a client's prediction
  double distance = 0.0;    // m moved horizontally
  double time = 0.0;        // s of drives booked
  double exhausted_time = 0.0;
  double peak_power = 0.0; // W, the most one drive spent
  // Swimming's share of the lines above (walker@2, OW9c): the work and carrying it paid for while
  // swimming, in J, and the metres it swam. In `work` and `carried` already, so not in residual().
  double swam = 0.0;
  double swam_distance = 0.0;
  // The books' own error: what the lines do not account for (0 up to rounding).
  double residual(double reserve) const {
    return reserve - (start + refilled - work - carried - overflow + unpaid + corrections);
  }
};

// An avatar's energy now.
struct avatar_energy {
  double reserve = 0.0;   // J
  double power = 0.0;     // W spent over the last drive
  bool exhausted = false; // `stop`: ran out, waiting for recover_fraction
  avatar_ledger ledger;
};

// How an avatar is in the water (walker@2, ARIADNE_WORLDS.md section 18.10): its controller's
// state, which the relay publishes beside the pose (room_avatar.h) for drawing.
enum class avatar_motion : std::uint8_t {
  walking,  // dry, or the water below wade_depth: walker@1
  wading,   // feet on the bed, the water from wade_depth to swim_depth: slower by the depth
  swimming, // the water over swim_depth, or the feet off the bed with the water over half of it
};
// "walking", "wading", "swimming".
const char *avatar_motion_name(avatar_motion m);
// The motion NAME names; false (OUT untouched) for any other text.
bool avatar_motion_of(const std::string &name, avatar_motion &out);

// walker@2's knobs (section 18.10.2), from its controller's params; off for walker@1, which
// ignores the water. Every speed is m/s, every depth metres of water above the feet.
struct avatar_swim {
  bool on = false;
  double walk_speed = 2.0;   // what a walk asks: a swim's pace is the intent's share of it
  double wade_depth = 0.3;   // wading from here...
  double swim_depth = 1.2;   // ...swimming from here (chest-high)
  double wade_slow = 0.6;    // the pace lost wading, at swim_depth
  double swim_speed = 1.0;   // a swim asked at walk_speed
  double swim_sprint = 1.8;  // the fastest a run swims
  double swim_force = 300.0; // N, the most a stroke pushes with, each way (across, up and down)
  double dive_speed = 0.8;   // diving and surfacing (move.z, -1 to 1, a share of it)
  double climb_height = 0.6; // the highest edge above the water a stroke up climbs onto
  double swim_cost = 0.8;    // carrying the body while swimming: swim_cost m g |v|, v the water's
  bool operator==(const avatar_swim &) const = default;
};
// Its knobs from a controller's params (walk_speed, wade_depth, ...: each one left out keeps its
// default), on.
avatar_swim avatar_swim_of(const std::map<std::string, double> &params);

// A static object avatars collide with: the room scene's ground, boxes and spheres; or, from the
// world document itself (world_replica, ARIADNE_WORLDS.md section 18.5, OW5c), a static body as
// the document's world made it (`body`: its shape and pose, a mesh's or a terrain's included) or
// the colliders a node makes at run time (`geometry`: a generated world's terrain and buildings).
struct avatar_static {
  enum class kind { box, sphere, body, geometry };
  kind type = kind::box;
  cvc::physics::vec3 center;
  cvc::physics::quat rotation;
  cvc::physics::vec3 half_extents{0.5, 0.5, 0.5};             // box
  double radius = 0.5;                                        // sphere
  std::shared_ptr<const cvc::physics::body_desc> body;        // body: made static
  std::shared_ptr<const cvc::world::world_geometry> geometry; // geometry: its collide items
  // body: its surface (friction, restitution) as the world it came from has it (body->material is
  // that world's id, not this one's); null: this world's default (ARIADNE_WORLDS.md section 18.5,
  // OW5h). A geometry's items carry their own surface classes.
  std::shared_ptr<const cvc::physics::material> material;
  // Ground a vehicle drives on (OW5p), as the relay's world has it (SceneWorldBody::ground): a
  // vehicle's chassis touching it has not hit anything. A geometry's: its height fields (its
  // terrain), not its other items (its buildings) or its edge walls.
  bool ground = false;
};

class avatar_world {
public:
  // Throws cvc::unsupported_exception when this build has no Jolt backend.
  avatar_world(double fixed_dt, const avatar_body &body,
               const avatar_energy_policy &energy = avatar_energy_policy{});
  // ... in a world of its own made with CFG (its backend Jolt, its step and gravity CFG's), for a
  // world that must step as another does (vehicle_dynamics.h's jolt plant: the relay's config).
  avatar_world(const cvc::physics::world_config &cfg, const avatar_body &body,
               const avatar_energy_policy &energy = avatar_energy_policy{});
  // The avatars in `shared`, a world the caller owns and steps (see ONE WORLD above): its fixed_dt
  // is the walker's step and its gravity prices the walk. `shared` must outlive this, or this must
  // be released (release()) first.
  avatar_world(cvc::physics::world &shared, const avatar_body &body,
               const avatar_energy_policy &energy = avatar_energy_policy{});
  // A shared world's avatars stay in it (the world may be gone already); release() takes them out.
  ~avatar_world();
  avatar_world(const avatar_world &) = delete;
  avatar_world &operator=(const avatar_world &) = delete;

  // Whether the world is someone else's (the shared-world constructor).
  bool shares_world() const { return _own == nullptr; }
  // A shared world: take every avatar's body out of it, and forget the avatars.
  void release();
  // A shared world that its owner replaced (a restart): the avatars again, in `shared` now, each at
  // `feet` (by pid; one not there keeps where it was last read, at rest), with its mass and its
  // reserve. The old world is not touched (it may be gone).
  void move_to(cvc::physics::world &shared,
               const std::map<std::string, cvc::physics::body_state> &feet);
  cvc::physics::world &world() { return *_world; }

  // Replace the static objects (a body or a geometry without one is skipped). Avatars are
  // re-created after them (same pids, same states), so the world's creation order stays
  // statics-then-avatars. Not in a shared world (its owner's bodies are the statics):
  // std::logic_error.
  void set_statics(const std::vector<avatar_static> &statics);
  // Every avatar's body from now on: each is re-created at its feet, keeping its velocity and its
  // reserve, with the new mass unless set_mass gave it its own.
  void set_body(const avatar_body &body);
  // The world's gravity (m/s^2) from the next step; carrying a body along (transport_cost) costs
  // its weight under it.
  void set_gravity(const cvc::physics::vec3 &g);
  const cvc::physics::vec3 &gravity() const { return _gravity; }
  // The room's wind (world_physics' wind, O14): a flow field of air (1.225 kg/m^3, C_d 1) whose
  // velocity is the wind (cvc::physics::wind_flow, its height profile over `base_z`), so it pushes
  // every dynamic avatar with drag against its box (0.5 x 1.7 m across it: 50 N at 9 m/s), and
  // the walker leans into it within force_max. A speed of 0 takes the field out.
  void set_wind(const cvc::algo::wind_state &w, double base_z = 0.0);
  // The water the avatars are in (OW9a, ARIADNE_WORLDS.md section 18.10): fluid fields as the
  // relay's world has them (a sea, its free surface read at set_time's time), so a dynamic avatar
  // floats and is dragged as it is there. Replaces the water set before; an empty list dries the
  // world. A shared world's water is its owner's: nothing.
  void set_water(const std::vector<cvc::physics::field> &water);
  // The time the wind's gusts are read at (world::set_field_time): the holon sets it to its tick's,
  // so the server and every client read the same gust at the same tick.
  void set_time(double t);

  // Add an avatar standing at `feet` (dynamic: simulated; kinematic: placed by the host).
  void add_avatar(const std::string &pid, const cvc::physics::body_state &feet, bool dynamic);
  void remove_avatar(const std::string &pid);
  bool has_avatar(const std::string &pid) const;
  // Its body in the world (invalid for no such avatar).
  cvc::physics::body_id body_of(const std::string &pid) const;

  // Before a step: drive a dynamic avatar from its intent through its controller, paying from its
  // reserve (see above).
  void drive(const std::string &pid, const avatar_intent &in);
  // The walker's jump (avatar_intent::jump): the upward speed it leaves the ground with (m/s; 4
  // clears about 0.8 m under Earth's gravity, 0 no jumping). It jumps while the intent holds jump
  // and something is under its feet (within 8 cm), and not while already rising; with a budget
  // (avatar_energy_policy) the reserve pays the jump's kinetic energy, and one it cannot pay for
  // is no jump.
  void set_jump_speed(double v) { _jump_speed = v; }
  double jump_speed() const { return _jump_speed; }
  // WALKER@2 (OW9c, ARIADNE_WORLDS.md section 18.10.2): with SWIM on, each drive reads the water
  // at the avatar's feet (physics::world::water_at, less its own wake) and the ground under them,
  // and is in one of three motions:
  //   walking   walker@1, jump and all.
  //   wading    walker@1 with the intent's move cut by the depth, to 1 - wade_slow at swim_depth;
  //             no jump above half swim_depth.
  //   swimming  across, toward the intent's move through the water (its share of walk_speed times
  //             swim_speed, at most swim_sprint) with at most swim_force, paying swim_cost m g |v|
  //             for carrying it; no move, no push: the water's drag carries it with the water.
  //             Up and down, move.z (-1 to 1) dives or surfaces at that share of dive_speed with
  //             swim_force; jump is a stroke up, surfacing, or onto an edge in front within
  //             climb_height of the surface, paying m g for the height it climbs.
  // Off (the default), the walker of walker@1, which ignores the water.
  void set_swim(const avatar_swim &swim) { _swim = swim; }
  const avatar_swim &swim() const { return _swim; }
  // The motion of an avatar's last drive (walking before its first, and always without swim).
  // Throws std::out_of_range for an unknown pid.
  avatar_motion motion(const std::string &pid) const;
  // The energy policy every avatar spends under. An avatar added later starts with a full reserve;
  // one already here keeps its reserve, cut to the new reserve_max.
  void set_energy_policy(const avatar_energy_policy &p);
  const avatar_energy_policy &energy_policy() const { return _energy; }
  // An avatar's energy. Throws std::out_of_range for an unknown pid.
  avatar_energy energy(const std::string &pid) const;
  // Set a dynamic avatar's reserve (J, clamped to [0, reserve_max]): a client correcting its
  // prediction to the server's, a replay restoring where it began.
  void set_reserve(const std::string &pid, double joules);
  // Teleport an avatar (feet pose + velocities): a reconcile snap, a respawn.
  void place(const std::string &pid, const cvc::physics::body_state &feet);
  // A kinematic avatar: arrive at this feet pose by the end of the next step.
  void move_kinematic(const std::string &pid, const cvc::physics::pose &feet);
  // A dynamic avatar's mass (kg, > 0) from the next step on, and the water it displaces with it
  // (avatar_displacement); it survives set_statics. A kinematic avatar has no mass (a no-op).
  void set_mass(const std::string &pid, double kg);

  // OBSTACLES (ARIADNE_WORLDS.md section 18.5, OW5): a box the host holds where it shows something
  // this world does not simulate -- a client's view of the room's vehicles. Each is a kinematic
  // body, HALF_EXTENTS about its centre: it moves only as the host moves it, and pushes what it
  // meets as it goes. The first hold of KEY (or one with other extents) creates it at CENTRE; a
  // later one moves it there by the end of the next step. Kept through set_statics (re-created
  // after the avatars), set_body and move_to; release() takes them out with the avatars.
  void hold_obstacle(std::uint64_t key, const cvc::physics::vec3 &half_extents,
                     const cvc::physics::pose &centre);
  void remove_obstacle(std::uint64_t key);
  bool has_obstacle(std::uint64_t key) const;
  // Its centre now. Throws std::out_of_range for an unknown key.
  cvc::physics::body_state obstacle_state(std::uint64_t key) const;

  // The statics' bodies that are ground (avatar_static::ground), as set_statics made them.
  const std::vector<cvc::physics::body_id> &ground_bodies() const { return _ground; }

  // One step of the world. Not in a shared world (its owner steps it): std::logic_error.
  void step();

  // An avatar's state, at its feet. Throws std::out_of_range for an unknown pid.
  cvc::physics::body_state state(const std::string &pid) const;

  const avatar_body &body() const { return _body; }

private:
  struct avatar {
    std::uint32_t id = 0; // cvc::physics::body_id value
    bool dynamic = true;
    double mass = 0.0;     // kg; avatar_body::mass until set_mass
    bool own_mass = false; // set_mass gave it one: a new body keeps it
    avatar_energy energy;
    avatar_motion motion = avatar_motion::walking; // its last drive's
  };
  std::uint32_t create_avatar(const cvc::physics::body_state &feet, bool dynamic, double mass);
  void add_avatar_material();
  // Remove every avatar's body and create it again (statics untouched, so the order stays
  // statics-then-avatars), each from its feet state.
  void recreate_avatars(const std::map<std::string, cvc::physics::body_state> &feet);
  // The walker controller: ease the horizontal velocity of S toward (TX, TY) within FORCE (N) and
  // what A can pay for, carrying the body at COST m g |v|, and charge A for it.
  void drive_walker(avatar &a, cvc::physics::body_state &s, double tx, double ty, double force,
                    double cost) const;
  // walker@2 (set_swim): the motion the water gives it, then that motion's drive.
  void drive_walker2(avatar &a, cvc::physics::body_state &s, const avatar_intent &in) const;
  // A swimmer's push up or down toward VZ (m/s) within swim_force, paying its work; false when A
  // cannot pay.
  void swim_vertical(avatar &a, cvc::physics::body_state &s, double vz) const;
  // A stroke up onto an edge in front of S within climb_height of SURFACE (the water's height):
  // placed on it, paying m g for the height. False when there is none, or A cannot pay.
  bool climb_out(avatar &a, cvc::physics::body_state &s, const avatar_intent &in,
                 double surface) const;
  // Pay JOULES of work from A's reserve as a jump does: false (nothing paid) when it cannot.
  bool pay_work(avatar &a, double joules) const;
  // Whether something is under the feet of S (within 8 cm), as a jump needs.
  bool standing(const cvc::physics::body_state &s) const;
  // The walker's jump: S (its centre's state) leaves the ground if the intent asks, it stands on
  // something and A can pay.
  void jump(avatar &a, cvc::physics::body_state &s, const avatar_intent &in) const;
  double _jump_speed = 4.0;
  avatar_swim _swim;
  avatar_body _body;
  avatar_energy_policy _energy;
  cvc::physics::vec3 _gravity{0.0, 0.0, -9.80665};
  double _dt;
  std::unique_ptr<cvc::physics::world> _own; // the world, when this owns it
  cvc::physics::world *_world = nullptr;     // the world the avatars are in (_own's or shared)
  std::uint32_t _avatar_material = 0;
  std::vector<std::uint32_t> _statics;
  std::vector<cvc::physics::body_id> _ground; // the statics that are ground (OW5p)
  // The statics' materials added to the world, each once (set_statics): the material and its id.
  std::vector<std::pair<cvc::physics::material, cvc::physics::material_id>> _static_materials;
  std::vector<cvc::physics::field_id> _static_fields; // a static geometry's volumes
  std::map<std::string, avatar> _avatars;
  struct obstacle {
    std::uint32_t id = 0; // cvc::physics::body_id value
    cvc::physics::vec3 half_extents;
  };
  std::map<std::uint64_t, obstacle> _obstacles;
  std::uint32_t create_obstacle(const cvc::physics::vec3 &half_extents,
                                const cvc::physics::body_state &centre);
  std::uint32_t _wind_field = 0xffffffffu; // the wind's flow field (none)
  std::vector<cvc::physics::field_id> _water_fields; // set_water's fluid fields
};

// A kinematic avatar's drift in a wind (O14): what a body the host places, not the solver, does in
// it. It holds its ground while the wind's drag on it (1/2 rho C_d A |w|^2, rho 1.225, C_d 1, A its
// width times its height, as the dynamic avatar's) is under what its controller can push with
// (force_max), as a dynamic walker does; past that it slides downwind at the speed where the drag
// at the relative wind equals force_max. 0 below about 40 m/s for the default 70 kg person and
// 840 N. `feet` is where it stands, t the wind's time.
cvc::physics::vec3 avatar_wind_drift(const cvc::physics::flow_velocity &wind,
                                     const avatar_body &body, double force_max,
                                     const cvc::physics::vec3 &feet, double t);

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_AVATAR_PHYSICS_H__
