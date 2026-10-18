/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The vehicles' step: the facade's side of cvc::physics vehicles (vehicle.h). Private: not
// installed. Before each engine step it slews the steering actuators, gives every steered
// wheel its Ackermann angle, runs the driver model, hands the backend its inputs, and loads
// each chassis with its aero, rolling and medium resistance from the last step's wheels;
// after the step it reads the vehicles back. Double precision, ascending vehicle ids, so a
// run stays a function of its inputs.

#ifndef __CVC_PHYSICS_VEHICLE_STEP_H__
#define __CVC_PHYSICS_VEHICLE_STEP_H__

#include "backend_impl.h"
#include "field_step.h"

#include <cstdint>
#include <cvc/physics/vehicle.h>
#include <map>
#include <string>
#include <vector>

namespace cvc {
namespace physics {
namespace detail {

class vehicle_set {
public:
  // Throw cvc::physics_error for a description world::add_vehicle refuses, and
  // cvc::unsupported_exception for a backend without vehicles.
  vehicle_id add(world_backend &be, field_set &fields, const vehicle_desc &d, const vec3 &position,
                 const quat &rotation);
  void remove(world_backend &be, field_set &fields, vehicle_id id);
  std::size_t size() const { return _v.size(); }
  // Whether `id` is a vehicle's chassis (world::remove_body refuses it).
  bool is_chassis(body_id id) const;
  body_id chassis(vehicle_id id) const { return at(id).chassis; }
  const vehicle_desc &desc(vehicle_id id) const { return at(id).desc; }
  // world::set_vehicle: throws cvc::physics_error for a value add_vehicle refuses or a
  // change to one fixed when the vehicle was added, and changes nothing then.
  void tune(world_backend &be, vehicle_id id, const vehicle_desc &d);

  void drive(vehicle_id id, const drive_command &c);
  void set_controls(vehicle_id id, const vehicle_controls &c);

  void pre_step(world_backend &be, const field_set &fields, const world_config &cfg,
                const std::vector<material> &materials);
  // After it: read the vehicles back, and press their ruts into rutted media.
  void post_step(world_backend &be, field_set &fields, const world_config &cfg);
  vehicle_state state(const world_backend &be, vehicle_id id) const;
  // Every vehicle, for world::snapshot.
  void snapshot(const world_backend &be, std::vector<vehicle_snapshot> &out) const;

  // The facade's vehicle state (actuators, driver memory, the last readback), for
  // world::save / restore. restore() needs the same vehicles, added in the same order.
  void save(std::string &out) const;
  void restore(const std::string &in, std::size_t &pos);

private:
  struct rt {
    vehicle_desc desc;
    body_id chassis;
    std::uint32_t handle = 0;
    // The steer at the centre of the front axle, and what it slews toward.
    double steer = 0.0;
    double steer_command = 0.0;
    bool raw = false; // set_controls rather than drive
    drive_command command;
    vehicle_controls controls;
    double last_speed = 0.0;
    bool have_last_speed = false;
    // Applied this step.
    double throttle = 0.0, brake = 0.0, hand_brake = 0.0;
    vec3 aero_force, air_velocity;
    std::vector<double> rolling, medium; // per wheel, N
    std::vector<double> medium_load;     // per wheel, N: the load the medium has felt
    vehicle_readback readback;           // the last step's wheels
  };
  rt &at(vehicle_id id);
  const rt &at(vehicle_id id) const;

  std::map<std::uint32_t, rt> _v;
  std::uint32_t _next = 0;
};

} // namespace detail
} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_VEHICLE_STEP_H__
