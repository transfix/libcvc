/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The boats' step: the facade's side of cvc::physics boats (boat.h). Private: not installed.
// Before each engine step it moves every boat's actuators toward its controls and pushes its
// hull with the propeller's thrust and the rudder's lift and drag, each at its point, from the
// water the fields hold there. Double precision, det:: math, ascending boat ids, so a run stays a
// function of its inputs.

#ifndef __CVC_PHYSICS_BOAT_STEP_H__
#define __CVC_PHYSICS_BOAT_STEP_H__

#include "backend_impl.h"
#include "field_step.h"

#include <cstdint>
#include <cvc/physics/boat.h>
#include <map>
#include <string>

namespace cvc {
namespace physics {
namespace detail {

class boat_set {
public:
  // Throws cvc::physics_error for a description world::add_boat refuses (the hull's own checks
  // are add_body's).
  static void validate(const boat_desc &d);
  // A boat on `hull`, a dynamic body world::add_boat has just added from d.hull.
  boat_id add(const boat_desc &d, body_id hull);
  // Forgets the boat and returns its hull, which the caller removes.
  body_id remove(boat_id id);
  std::size_t size() const { return _b.size(); }
  bool contains(boat_id id) const { return _b.count(id.value) != 0; }
  // Whether `id` is a boat's hull (world::remove_body refuses it).
  bool is_hull(body_id id) const;
  const boat_desc &desc(boat_id id) const { return at(id).desc; }

  // Throws cvc::physics_error for a control that is not finite or outside [-1, 1].
  void set_controls(boat_id id, const boat_controls &c);

  // Before the engine step: the actuators, then each boat's thrust and rudder force through
  // be.add_force_at. `t` is the world's field time.
  void pre_step(world_backend &be, const field_set &fields, const world_config &cfg, double t);
  boat_state state(const world_backend &be, const field_set &fields, boat_id id, double t) const;

  // Each boat's id, hull, description digest, controls, actuators and last forces, in id order,
  // for world::save. restore() refuses a world whose boats differ and then leaves the boats as
  // they were (world::restore has restored the engine's bodies before it gets here).
  void save(std::string &out) const;
  void restore(const std::string &in, std::size_t &pos);

private:
  struct rt {
    boat_desc desc;
    body_id hull;
    std::uint64_t digest = 0; // of the propulsion, the rudder and the actuators
    vec3 direction;           // the propeller's, normalised
    boat_controls command;
    double throttle = 0.0, rudder = 0.0; // the actuators
    // The last step's.
    bool propeller_wet = false, rudder_wet = false;
    double inflow = 0.0, thrust = 0.0, outboard_angle = 0.0;
    double rudder_angle = 0.0, angle_of_attack = 0.0, rudder_lift = 0.0, rudder_drag = 0.0;
    vec3 thrust_force, rudder_force;
  };
  rt &at(boat_id id);
  const rt &at(boat_id id) const;

  std::map<std::uint32_t, rt> _b;
  std::uint32_t _next = 0;
};

} // namespace detail
} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_BOAT_STEP_H__
