/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The Ackermann allocation (docs/roadmap/NAV-COPH-ROADMAP.md §3.3.2): CoPH's planar force as a
// car's steer and acceleration. The twin of GRL-SNAM's grl_snam.coph.allocation, which the cars
// controller (T4) uses; the fleet driver (C5) uses this one. With h the heading and n its left
// normal:
//
//   a      = clamp(u . h, -a_max, a_max)
//   kappa* = (u . n) / max(v^2, v_floor^2)
//   delta  = clamp(atan(L kappa*), -delta_max, delta_max)
//   u_used = a h + (v^2 tan(delta) / L) n
//   e      = u - u_used                      (the force the car did not take)
//
// The steer is the bicycle angle (rad, + left) and the acceleration is along the heading, as
// sim_world::override_command takes them. v is the signed speed along h (v < 0 backing up). At
// rest the car takes no lateral force, and e holds all of u . n; v_floor keeps kappa* finite there.
// §3.3.2 left v_min open; coph_speed_floor is the default, as GRL-SNAM's SPEED_FLOOR.
//
// GRL-SNAM's golden pins it (G11, FLOAT64). The trigonometry is cvc::algo::det's, so the same bits
// on every platform.

#ifndef CVC_NAV_COPH_ALLOCATION_H
#define CVC_NAV_COPH_ALLOCATION_H

#include <cvc/nav/coph_ledger.h>

namespace cvc {
namespace nav {

inline constexpr double coph_speed_floor = 0.5; // m/s

struct coph_allocation {
  double accel = 0; // along the heading
  double steer = 0; // the bicycle angle, rad, + left
  double kappa = 0; // the curvature asked for, before the lock: (u . n) / max(v^2, v_floor^2)
  coph_vec2 used{}; // the force the car took
  coph_vec2 unallocated{}; // u - used
};

// Allocates u to a car heading `heading` (rad) at signed `speed`, with wheelbase L > 0, steer lock
// steer_max >= 0, acceleration limit accel_max >= 0 and speed_floor > 0.
coph_allocation coph_allocate(const coph_vec2 &u, double heading, double speed, double wheelbase,
                              double steer_max, double accel_max,
                              double speed_floor = coph_speed_floor);

} // namespace nav
} // namespace cvc

#endif // CVC_NAV_COPH_ALLOCATION_H
