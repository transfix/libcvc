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
// An intent DRIVES its avatar (drive()): the horizontal velocity eases toward the intent's at up to
// avatar_body::max_accel, and the body is held upright facing the intent's yaw. Everything else --
// gravity, a push from a heavier avatar, stopping at the cube -- is the solver's. The avatar's
// material is frictionless (the drive is its grip on the floor), so a still avatar shoved by
// another slides and the drive brings it back to rest.
//
// Poses in and out are the avatar's FEET (body_state.position at floor level, as holon_sync
// replicates and room_view draws); the box's centre is half its height above them.
//
// The world runs in cvc::physics' cross-platform determinism mode, so the server (Linux) and a
// client (Windows, wasm) stepping the same inputs from the same state agree -- what client
// prediction relies on. Creation order is part of a Jolt run, so statics are always created first,
// in the order given.

// An avatar's physical body.
struct avatar_body {
  double mass = 70.0;      // kg
  double width = 0.5;      // m, the box's x and y edge
  double height = 1.7;     // m
  double max_accel = 12.0; // m/s^2: how fast the drive changes the horizontal velocity
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
  avatar_world(double fixed_dt, const avatar_body &body);
  ~avatar_world();
  avatar_world(const avatar_world &) = delete;
  avatar_world &operator=(const avatar_world &) = delete;

  // Replace the static objects. Avatars are re-created after them (same pids, same states), so the
  // world's creation order stays statics-then-avatars.
  void set_statics(const std::vector<avatar_static> &statics);

  // Add an avatar standing at `feet` (dynamic: simulated; kinematic: placed by the host).
  void add_avatar(const std::string &pid, const cvc::physics::body_state &feet, bool dynamic);
  void remove_avatar(const std::string &pid);
  bool has_avatar(const std::string &pid) const;

  // Before a step: drive a dynamic avatar from its intent (see above).
  void drive(const std::string &pid, const avatar_intent &in);
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
    double mass = 0.0; // kg; avatar_body::mass until set_mass
  };
  std::uint32_t create_avatar(const cvc::physics::body_state &feet, bool dynamic, double mass);

  avatar_body _body;
  double _dt;
  std::unique_ptr<cvc::physics::world> _world;
  std::uint32_t _avatar_material = 0;
  std::vector<std::uint32_t> _statics;
  std::map<std::string, avatar> _avatars;
};

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_AVATAR_PHYSICS_H__
