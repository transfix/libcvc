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
  bool contains(vehicle_id id) const { return _v.count(id.value) != 0; }
  const vehicle_desc &desc(vehicle_id id) const { return at(id).desc; }
  // world::set_vehicle: throws cvc::physics_error for a value add_vehicle refuses or a
  // change to one fixed when the vehicle was added, and changes nothing then.
  void tune(world_backend &be, vehicle_id id, const vehicle_desc &d);

  void drive(vehicle_id id, const drive_command &c);
  void set_controls(vehicle_id id, const vehicle_controls &c);
  // world::set_vehicle_steer: the actuator's angle, clamped to max_steer.
  void set_steer(vehicle_id id, double steer);

  // `t` is the world's field time, the air's wind is read at (field_set::air_at).
  void pre_step(world_backend &be, const field_set &fields, const world_config &cfg,
                const std::vector<material> &materials, double t);
  // After it: read the vehicles back, and press their ruts into rutted media.
  void post_step(world_backend &be, field_set &fields, const world_config &cfg);
  vehicle_state state(const world_backend &be, vehicle_id id) const;
  // Every vehicle, for world::snapshot.
  void snapshot(const world_backend &be, std::vector<vehicle_snapshot> &out) const;

  // The facade's vehicle state (actuators, driver memory, the last readback), for
  // world::save / restore. restore() needs the same vehicles, added in the same order.
  void save(const world_backend &be, std::string &out) const;
  void restore(world_backend &be, const std::string &in, std::size_t &pos);

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
    double accel_trim = 0.0; // the driver's integral correction (m/s^2)
    double trim_sign = 1.0;  // the sign of the command the trim learnt
    // Applied this step.
    double throttle = 0.0, brake = 0.0, hand_brake = 0.0;
    vec3 aero_force, air_velocity;
    std::vector<double> rolling, medium; // per wheel, N
    std::vector<double> medium_load;     // per wheel, N: the load the medium has felt
    // Wading (vehicle_desc::wading): per wheel the water over its contact (m) and its drag (N),
    // the push on wheels and underbody, and the flooded engine (state: saved, with wading on).
    std::vector<double> water_depth, water_drag;
    vec3 water_force;
    bool flooded = false;
    // With a flood_area: the water in the cabin and engine bay, m^3 and kg (state: saved while
    // there is any).
    double inside_volume = 0.0, inside_mass = 0.0;
    vehicle_readback readback;           // the last step's wheels
    vehicle_energy energy;               // the ledger since added
    double potential0 = 0.0;             // the potential energy at the first sample
    bool have_potential0 = false;
  };
  static void account(rt &v, double dt);
  // Wading: the water's push on the wheels and underbody for the step about to run, at the
  // chassis state `s` (centre of mass `com`) with each wheel's `steer`, into `force` / `torque`
  // (about com); sets each wheel's grip and the flooded engine, and records the readbacks. `t` is
  // the field time. With a flood_area it also floods or drains the cabin and engine bay over the
  // step and adds the water inside's weight. Returns whether any part was in water or there is
  // water inside (with neither, `force` and `torque` are left alone), and sets `moving` when that
  // water flows (a current, waves) or the water inside rises or falls. It reads only what save()
  // keeps, so a restored world wades the same.
  static bool wade(rt &v, const body_state &s, const vec3 &com, const std::vector<double> &steer,
                   const field_set &fields, const world_config &cfg,
                   const std::vector<material> &materials, double t, vec3 &force, vec3 &torque,
                   std::vector<double> &grip, bool &moving);
  rt &at(vehicle_id id);
  const rt &at(vehicle_id id) const;

  std::map<std::uint32_t, rt> _v;
  std::uint32_t _next = 0;
};

} // namespace detail
} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_VEHICLE_STEP_H__
