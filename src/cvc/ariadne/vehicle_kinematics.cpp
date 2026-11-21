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

#include <algorithm>
#include <cmath>
#include <cvc/ariadne/vehicle_kinematics.h>

namespace cvc {
namespace ariadne {

namespace {
constexpr double kPi = 3.14159265358979323846;

// The steps of the navigator's tick taken by the end of the step ending on TICK, less one: 0 on
// the step the navigator ticks on.
int into_tick(const kinematic_drive &k, std::uint64_t tick) {
  const std::uint64_t n = static_cast<std::uint64_t>(k.tick_steps);
  return static_cast<int>((tick % n + n - k.phase % n) % n);
}

// The chassis box of the bicycle (WorldVehicles' bicycle_box): its centre ahead of the rear axle.
std::array<double, 4> box_of(const kinematic_vehicle &v, const kinematic_drive &k) {
  return {v.rx - k.rear_x * std::cos(v.heading), v.ry - k.rear_x * std::sin(v.heading), v.z,
          v.heading};
}
} // namespace

kinematic_vehicle kinematic_vehicle_at(const kinematic_drive &k,
                                       const cvc::physics::body_state &chassis, std::uint64_t tick,
                                       double fixed_dt) {
  kinematic_vehicle v;
  const cvc::physics::quat &q = chassis.rotation;
  const double yaw =
      std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  v.z = chassis.position.z;
  v.vx = chassis.linear_velocity.x;
  v.vy = chassis.linear_velocity.y;
  v.box = {chassis.position.x, chassis.position.y, v.z, yaw};
  if (!k.valid()) {
    v.from = v.to = v.box;
    v.heading = yaw;
    return v;
  }
  // Through its tick so far, and to its end, along the velocity.
  const int n = k.tick_steps;
  const int j = into_tick(k, tick);
  v.sub = j + 1;
  const double back = fixed_dt * static_cast<double>(j + 1);
  const double ahead = fixed_dt * static_cast<double>(n - j - 1);
  v.from = {v.box[0] - v.vx * back, v.box[1] - v.vy * back, v.z, yaw};
  v.to = {v.box[0] + v.vx * ahead, v.box[1] + v.vy * ahead, v.z, yaw};
  v.heading = yaw;
  v.speed = v.vx * std::cos(yaw) + v.vy * std::sin(yaw);
  v.rx = v.to[0] + k.rear_x * std::cos(yaw);
  v.ry = v.to[1] + k.rear_x * std::sin(yaw);
  return v;
}

void kinematic_vehicle_step(kinematic_vehicle &v, const kinematic_drive &k, std::uint64_t tick,
                            double fixed_dt) {
  if (!k.valid())
    return;
  const int n = k.tick_steps;
  if (into_tick(k, tick) == 0) {
    // The navigator's tick (sim_world's override_command on the kinematic arm): the steering
    // within the lock, the acceleration held to the top speed, the bicycle over the tick.
    v.from = box_of(v, k);
    const double dt = static_cast<double>(n) * fixed_dt;
    const double steer = std::clamp(v.steer_cmd, -k.lock, k.lock);
    double a = v.accel_cmd;
    if (v.speed >= 0.0)
      a = std::min(a, std::max((k.vmax - v.speed) / dt, -k.a_max));
    else
      a = std::max(a, std::min((-k.vmax - v.speed) / dt, k.a_max));
    const double sp_min = k.allow_reverse ? -0.25 * k.vmax : 0.0;
    const double h = dt / static_cast<double>(k.nsub);
    const double t = std::tan(steer);
    for (int s = 0; s < k.nsub; ++s) {
      v.speed = std::min(std::max(v.speed + h * a, sp_min), k.vmax);
      v.heading = v.heading + h * (v.speed / k.wheelbase) * t;
      v.rx = v.rx + h * v.speed * std::cos(v.heading);
      v.ry = v.ry + h * v.speed * std::sin(v.heading);
    }
    v.to = box_of(v, k);
    v.sub = 0;
  }
  // The box a step along the line between them (WorldVehicles' before_world_step).
  const double t = std::min(1.0, (static_cast<double>(v.sub) + 1.0) / static_cast<double>(n));
  const std::array<double, 4> &a = v.from, &b = v.to;
  const std::array<double, 4> next{a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]),
                                   a[2] + t * (b[2] - a[2]),
                                   a[3] + t * std::remainder(b[3] - a[3], 2.0 * kPi)};
  if (fixed_dt > 0.0) {
    v.vx = (next[0] - v.box[0]) / fixed_dt;
    v.vy = (next[1] - v.box[1]) / fixed_dt;
  }
  v.box = next;
  ++v.sub;
}

cvc::physics::body_state kinematic_vehicle_state(const kinematic_vehicle &v) {
  cvc::physics::body_state s;
  s.position = {v.box[0], v.box[1], v.box[2]};
  s.rotation = {std::cos(0.5 * v.box[3]), 0.0, 0.0, std::sin(0.5 * v.box[3])};
  s.linear_velocity = {v.vx, v.vy, 0.0};
  return s;
}

} // namespace ariadne
} // namespace cvc
