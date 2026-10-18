/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The field step: the forces cvc::physics fields put on bodies, which world.cpp
// applies before each engine step (field.h has the model). Private: not
// installed. It runs in the facade, in double precision and in ascending field and
// body id order, so a run stays a function of its inputs; the backend sees one
// force and one torque per pushed body.

#ifndef __CVC_PHYSICS_FIELD_STEP_H__
#define __CVC_PHYSICS_FIELD_STEP_H__

#include "backend_impl.h"

#include <cstdint>
#include <map>
#include <vector>

namespace cvc {
namespace physics {
namespace detail {

// A copy of a cvc::volume's values, sampled trilinearly in world metres.
struct field_grid {
  std::vector<double> values; // x fastest, as cvc::voxels stores them
  std::uint64_t n[3] = {0, 0, 0};
  double lo[3] = {0.0, 0.0, 0.0}, hi[3] = {0.0, 0.0, 0.0};
  double span[3] = {0.0, 0.0, 0.0}; // 0 along an axis with one sample

  bool contains(const double p[3]) const;
  // The value at p, which the caller has checked lies in the box.
  double at(const double p[3]) const;
};

struct compiled_field {
  field desc; // as given: it holds the volumes
  field_grid scalar;
  bool moving = false;
  field_grid velocity[3];
};

// A dynamic body as the fields see it: its sample cells and what its drag area
// needs, built when the body is added.
struct field_body {
  enum class outline : std::uint8_t { sphere, box, other };
  std::vector<vec3> cells; // centres, body-local about the body's origin
  vec3 cell;               // a cell's size along the body's axes
  vec3 lo, hi;             // the cells' body-local bounds
  double cell_volume = 0.0, volume = 0.0, mass = 0.0;
  vec3 center_of_mass; // body-local
  outline shape = outline::other;
  double radius = 0.0; // sphere
  vec3 half;           // box
  bool vehicle = false; // a vehicle's chassis: no flow drag (vehicle.h's aero is its drag)
  field_load load;      // the last step's
};

class field_set {
public:
  // Throw cvc::physics_error for a field world::add_field refuses.
  field_id add(world_backend &be, const field &f);
  void set(world_backend &be, field_id id, const field &f);
  void remove(field_id id);
  std::size_t size() const { return _fields.size(); }

  // A body world::add_body has checked and the backend has added. Only dynamic
  // spheres, boxes and convex hulls are recorded.
  void body_added(const world_backend &be, body_id id, const body_desc &d);
  void body_removed(body_id id) { _bodies.erase(id.value); }

  // A vehicle's chassis (body_added first): its centre of mass, and no flow drag.
  void mark_vehicle(body_id id, const vec3 &center_of_mass);

  // Push every awake body the fields reach, for the step about to run.
  void apply(world_backend &be, const world_config &cfg);
  field_load load(body_id id) const;

  // The air at `p` for a vehicle's aero: a flow field's density and wind where one covers
  // `p` (the first, in id order), true; else false, `rho` and `wind` untouched.
  bool air_at(const vec3 &p, double &rho, vec3 &wind) const;
  // The medium over a wheel's ground contact `p`: the deepest medium field above it along
  // +z, its bearing modulus at mid-depth and its traction. False if none lies over `p`.
  // Read-only: safe from the engine's worker threads during a step.
  bool medium_under(const vec3 &p, wheel_medium &out) const;
  // The height of the tallest medium field (0 with none).
  double medium_reach() const;

private:
  void wake_in(world_backend &be, const field_grid &box);

  std::map<std::uint32_t, compiled_field> _fields;
  std::map<std::uint32_t, field_body> _bodies;
  std::uint32_t _next = 0;
};

} // namespace detail
} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_FIELD_STEP_H__
