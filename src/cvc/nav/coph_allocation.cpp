/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The Ackermann allocation (coph_allocation.h), in allocation.allocate's order. Built with
// -ffp-contract=off (/fp:precise).

#include <algorithm>
#include <cvc/algo/det_math.h>
#include <cvc/nav/coph_allocation.h>

namespace cvc {
namespace nav {

namespace {

// allocation._clamp: min(max(x, lo), hi).
double clamp_to(double x, double lo, double hi) { return std::min(std::max(x, lo), hi); }

} // namespace

coph_allocation coph_allocate(const coph_vec2 &u, double heading, double speed, double wheelbase,
                              double steer_max, double accel_max, double speed_floor) {
  const double hx = algo::det::cos(heading), hy = algo::det::sin(heading);
  const double nx = -hy, ny = hx;
  const double along = u[0] * hx + u[1] * hy;
  const double across = u[0] * nx + u[1] * ny;
  coph_allocation a;
  a.accel = clamp_to(along, -accel_max, accel_max);
  a.kappa = across / std::max(speed * speed, speed_floor * speed_floor);
  a.steer = clamp_to(algo::det::atan(wheelbase * a.kappa), -steer_max, steer_max);
  const double lateral = speed * speed * algo::det::tan(a.steer) / wheelbase;
  a.used = {a.accel * hx + lateral * nx, a.accel * hy + lateral * ny};
  a.unallocated = {u[0] - a.used[0], u[1] - a.used[1]};
  return a;
}

} // namespace nav
} // namespace cvc
