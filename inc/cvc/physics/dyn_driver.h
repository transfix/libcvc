/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// How the dynamic plants (dyn_bike_plant, dyn_4w_plant) take nav's acceleration: as the Jolt
// plant's driver does (vehicle.cpp), so nav drives every plant the same way. Nav's acceleration is
// what the vehicle is to achieve along its heading, not a force. Every substep:
//   - within 0.05 m/s^2 of none, below 0.3 m/s, the vehicle holds on its brakes: its speed eased to
//     zero against the grade too, damped critically behind the drive's lag;
//   - else its drive force per unit mass is the acceleration wanted, plus gravity along the slope
//     (the feed-forward), plus half of what the last substep missed by and an integral trim that
//     learns the rest at 2 per second of it, within the vehicle's max_accel and max_decel.
// So a slope, a wall dragging along the vehicle's side or the drive's lag do not cost nav what it
// asked for until the vehicle is flat out. Without it a dynamic model creeps on the smallest
// command, rolls down any slope (its rolling resistance fades to nothing at rest, gravity along
// the slope does not) and crawls along a wall whose friction matches nav's acceleration.

#ifndef __CVC_PHYSICS_DYN_DRIVER_H__
#define __CVC_PHYSICS_DYN_DRIVER_H__

namespace cvc {
namespace physics {

// One vehicle's driver: what it has learnt and measured.
struct dyn_driver {
  double trim = 0.0;      // the integral trim (m/s^2)
  double trim_sign = 0.0; // the sign of the command it learnt on: a new sign restarts it
  double got = 0.0;       // the last substep's acceleration along the heading (m/s^2)
};

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_DYN_DRIVER_H__
