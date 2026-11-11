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
#include <cvc/ariadne/avatar_sim.h> // avatar_intent
#include <cvc/physics/body.h>
#include <cvc/physics/types.h>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
namespace physics {
class world;
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
// The world runs in cvc::physics' cross-platform determinism mode, so the server (Linux) and a
// client (Windows, wasm) stepping the same inputs from the same state agree -- what client
// prediction relies on. Creation order is part of a Jolt run, so statics are always created first,
// in the order given.

// An avatar's physical body: its archetype's (world_rules.h), the same for every avatar so far.
struct avatar_body {
  double mass = 70.0;  // kg
  double width = 0.5;  // m, the box's x and y edge
  double height = 1.7; // m
  bool operator==(const avatar_body &) const = default;
};

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

// A static object avatars collide with (the room scene's ground, boxes, spheres).
struct avatar_static {
  enum class kind { box, sphere };
  kind type = kind::box;
  cvc::physics::vec3 center;
  cvc::physics::quat rotation;
  cvc::physics::vec3 half_extents{0.5, 0.5, 0.5}; // box
  double radius = 0.5;                            // sphere
};

class avatar_world {
public:
  // Throws cvc::unsupported_exception when this build has no Jolt backend.
  avatar_world(double fixed_dt, const avatar_body &body,
               const avatar_energy_policy &energy = avatar_energy_policy{});
  ~avatar_world();
  avatar_world(const avatar_world &) = delete;
  avatar_world &operator=(const avatar_world &) = delete;

  // Replace the static objects. Avatars are re-created after them (same pids, same states), so the
  // world's creation order stays statics-then-avatars.
  void set_statics(const std::vector<avatar_static> &statics);
  // Every avatar's body from now on: each is re-created at its feet, keeping its velocity and its
  // reserve, with the new mass unless set_mass gave it its own.
  void set_body(const avatar_body &body);
  // The world's gravity (m/s^2) from the next step; carrying a body along (transport_cost) costs
  // its weight under it.
  void set_gravity(const cvc::physics::vec3 &g);
  const cvc::physics::vec3 &gravity() const { return _gravity; }

  // Add an avatar standing at `feet` (dynamic: simulated; kinematic: placed by the host).
  void add_avatar(const std::string &pid, const cvc::physics::body_state &feet, bool dynamic);
  void remove_avatar(const std::string &pid);
  bool has_avatar(const std::string &pid) const;

  // Before a step: drive a dynamic avatar from its intent through its controller, paying from its
  // reserve (see above).
  void drive(const std::string &pid, const avatar_intent &in);
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
  // A dynamic avatar's mass (kg, > 0) from the next step on; it survives set_statics. A kinematic
  // avatar has no mass (a no-op).
  void set_mass(const std::string &pid, double kg);

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
  };
  std::uint32_t create_avatar(const cvc::physics::body_state &feet, bool dynamic, double mass);
  // Remove every avatar's body and create it again (statics untouched, so the order stays
  // statics-then-avatars), each from its feet state.
  void recreate_avatars(const std::map<std::string, cvc::physics::body_state> &feet);
  // The walker controller: ease the horizontal velocity of S toward the intent's within the
  // force limit and what A can pay for, and charge A for it.
  void drive_walker(avatar &a, cvc::physics::body_state &s, const avatar_intent &in) const;
  avatar_body _body;
  avatar_energy_policy _energy;
  cvc::physics::vec3 _gravity{0.0, 0.0, -9.80665};
  double _dt;
  std::unique_ptr<cvc::physics::world> _world;
  std::uint32_t _avatar_material = 0;
  std::vector<std::uint32_t> _statics;
  std::map<std::string, avatar> _avatars;
};

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_AVATAR_PHYSICS_H__
