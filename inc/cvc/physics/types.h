/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics — the engine-agnostic rigid-body facade. This header holds the
// value types every other physics header uses.
//
// Units and frame: SI throughout (metres, kilograms, seconds, newtons), z up,
// gravity along -z. Positions are doubles: the world keeps a double-precision
// floating origin and hands the engine float offsets from it, which resolves
// about 0.25 mm across a 3 km city (world::origin, world::rebase).
//
// The physics headers include only the standard library, so a backend can be
// compiled without libcvc's other dependencies. No public header names an
// engine: which backends a build carries is a run-time question
// (available_backends(), backend.h).

#ifndef __CVC_PHYSICS_TYPES_H__
#define __CVC_PHYSICS_TYPES_H__

#include <cstdint>
#include <string>

namespace cvc {
namespace physics {

// A point, direction or vector quantity in world space.
struct vec3 {
  double x = 0.0, y = 0.0, z = 0.0;
};

inline vec3 operator+(const vec3 &a, const vec3 &b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline vec3 operator-(const vec3 &a, const vec3 &b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline vec3 operator*(const vec3 &a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline vec3 operator*(double s, const vec3 &a) { return a * s; }
inline double dot(const vec3 &a, const vec3 &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

// A rotation as a unit quaternion.
struct quat {
  double w = 1.0, x = 0.0, y = 0.0, z = 0.0;
};

// How a body moves.
enum class motion_type : std::uint8_t {
  static_,   // never moves: terrain, buildings
  kinematic, // moved by the application (velocity or move_kinematic); pushes dynamic bodies and is
             // never pushed back
  dynamic,   // integrated by the solver
};

// A body handle. Generation-checked: a handle to a removed body stays invalid
// even after its slot is reused, so a stale handle is refused, never aliased.
struct body_id {
  std::uint32_t value = 0xffffffffu;
  bool valid() const { return value != 0xffffffffu; }
  friend bool operator==(body_id a, body_id b) { return a.value == b.value; }
  friend bool operator!=(body_id a, body_id b) { return a.value != b.value; }
  friend bool operator<(body_id a, body_id b) { return a.value < b.value; }
};

// Index into a world's material table (world::add_material). Material 0 always
// exists; see world.h.
using material_id = std::uint16_t;

// A surface material. One material per body, per triangle of a mesh, or per cell
// of a height field. The engine's contact friction is these values combined
// (world_config::friction); the tire model (the vehicle) reads the same table, so
// every plant sees the μ that cvc::nav's friction field holds.
struct material {
  std::string name;
  float friction = 1.0f;           // Coulomb μ; Jolt uses one value for static and dynamic
  float restitution = 0.0f;        // 0 = no bounce, 1 = elastic
  float rolling_resistance = 0.0f; // C_rr, for the vehicle and the energy ledger
  int nav_material = -1;           // the cvc::nav material id this surface carries; -1 = none
};

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_TYPES_H__
