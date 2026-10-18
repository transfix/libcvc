/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics::to_json — a world_snapshot as JSON, for readers outside C++: Python
// (json.loads(pycvc.state_get(app, "physics.snapshot_json"))), a remote peer of the state
// tree, a log.
//
//   const std::string all = cvc::physics::to_json(w.snapshot());
//
// The document (format "cvc.physics.snapshot/1"):
//   {"format", "tick", "time", "static_version",
//    "config":    {backend, fixed_dt, gravity, origin, collision_steps, velocity_steps,
//                  position_steps, penetration_slop, threads, determinism,
//                  friction_combine, restitution_combine, contacts},
//    "report":    {step_ms, contacts, active_bodies, saturated},
//    "materials": [{id, name, friction, restitution, rolling_resistance, nav_material}],
//    "bodies":    [{id, motion, material, mass, linear_damping, angular_damping,
//                   allow_sleeping, user_data, position, rotation, velocity,
//                   angular_velocity, active, field_force, field_torque, immersed_volume}],
//    "fields":    [{id, kind, lo, hi, moving, drag_coefficient, damping, traction, ruts,
//                   rut_cell, rut_cells, rut_deepest}],
//    "vehicles":  [{id, desc: {... every vehicle_desc value, axles: [...]},
//                   state: {chassis, position, rotation, velocity, angular_velocity, active,
//                           speed, steer, steer_command, throttle, brake, hand_brake, gear,
//                           engine_rpm, aero_force, air_velocity, wheels: [...]}}],
//    "contacts":  [{type, body_a, body_b, sub_shape_a, sub_shape_b, material_a, material_b,
//                   friction, restitution, point, normal, depth}]}
// Units are the facade's (SI, world metres). A vector is [x, y, z] and a rotation is
// [w, x, y, z]. Enums are their names. A double is written in the shortest form that reads
// back exactly (17 significant digits where the C++ library lacks floating-point
// std::to_chars), always with a '.' or an exponent; NaN and infinities are null, and so is
// an invalid id. Lists are in ascending id order.

#ifndef __CVC_PHYSICS_SNAPSHOT_JSON_H__
#define __CVC_PHYSICS_SNAPSHOT_JSON_H__

#include <cstdint>
#include <cvc/physics/world.h>
#include <string>

namespace cvc {
namespace physics {

// Which bodies a document lists.
enum class json_bodies : std::uint8_t {
  all,         // every body
  moving,      // the dynamic and kinematic ones (world_binding's snapshot_json)
  static_only, // only the static ones, in a document of just format, tick, time,
               // static_version and bodies (world_binding's snapshot_json_static)
};

std::string to_json(const world_snapshot &s, json_bodies bodies = json_bodies::all);

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_SNAPSHOT_JSON_H__
