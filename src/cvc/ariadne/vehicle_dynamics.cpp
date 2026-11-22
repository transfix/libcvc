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
#include <cvc/ariadne/vehicle_dynamics.h>
#include <cvc/ariadne/world_rules.h>
#include <cvc/ariadne/world_vehicles.h>
#include <stdexcept>

namespace cvc {
namespace ariadne {

namespace ph = cvc::physics;

namespace {
// The steps of the navigator's tick taken by the end of the step ending on TICK, less one: 0 on
// the step the navigator ticks on (as vehicle_kinematics.cpp's).
int into_tick(const dynamic_drive &d, std::uint64_t tick) {
  const std::uint64_t n = static_cast<std::uint64_t>(d.tick_steps);
  return static_cast<int>((tick % n + n - d.phase % n) % n);
}

double yaw_of(const ph::quat &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
} // namespace

dynamic_vehicle_model::dynamic_vehicle_model(const dynamic_drive &drive, double fixed_dt)
    : _drive(drive), _dt(fixed_dt) {
  if (!drive.valid())
    throw std::invalid_argument("dynamic_vehicle_model: not a dynamic drive to run");
  if (!(fixed_dt > 0.0) || !std::isfinite(fixed_dt))
    throw std::invalid_argument("dynamic_vehicle_model: the step must be finite and > 0");
  // The world's vehicle: its preset with the numbers it was given (vehicle_desc_of).
  vehicle_archetype a;
  a.id = "driven";
  a.plant = "dynamic";
  a.preset = drive.preset;
  a.overrides = {{"mass", drive.mass},
                 {"max_speed", drive.max_speed},
                 {"max_accel", drive.max_accel},
                 {"max_decel", drive.max_decel},
                 {"max_steer", drive.max_steer}};
  const ph::vehicle_desc desc = vehicle_desc_of(a);
  // WorldVehicles' plant options but the world: one model substep a physics step.
  ph::dyn_bike_plant_options o;
  o.substep = fixed_dt;
  _plant = std::make_unique<ph::dyn_bike_plant>(std::vector<ph::vehicle_desc>{desc}, o);
  cvc::world_units::normalized_frame si; // metres: the commands go in SI
  si.scale = 1.0;
  const cvc::nav::plant_state start{};
  _plant->attach(1, &start, si, fixed_dt);
}

dynamic_vehicle_model::~dynamic_vehicle_model() = default;

dynamic_vehicle dynamic_vehicle_model::at(const ph::body_state &chassis, std::uint64_t tick,
                                          const dynamic_vehicle *like) const {
  (void)tick;
  dynamic_vehicle v;
  if (like) {
    v.model = like->model;
    v.steer_cmd = like->steer_cmd;
    v.accel_cmd = like->accel_cmd;
  } else {
    v.model = _plant->save(0);
    v.model.body = {};
    v.model.wheels[0] = v.model.wheels[1] = 0.0;
    v.model.driver = {};
    v.model.steer_cmd = v.model.accel_cmd = 0.0;
  }
  // The box's pose and velocity back into the model, as the plant takes a world's (its settle):
  // the rear axle behind the box's centre, the twist at the centre of gravity.
  const double psi = yaw_of(chassis.rotation);
  const double c = std::cos(psi), s = std::sin(psi);
  const double rear_x = _plant->rear_axle(0);
  const double off = -rear_x - _plant->params(0).lr;
  ph::dyn_bike_saved &m = v.model;
  m.body.x = chassis.position.x + rear_x * c;
  m.body.y = chassis.position.y + rear_x * s;
  m.body.psi = psi;
  const double r = like ? like->model.body.r : 0.0;
  m.body.r = r;
  const double vgx = chassis.linear_velocity.x + r * off * s,
               vgy = chassis.linear_velocity.y - r * off * c;
  m.body.vx = c * vgx + s * vgy;
  m.body.vy = -s * vgx + c * vgy;
  v.box = {chassis.position.x, chassis.position.y, chassis.position.z, psi};
  v.vx = chassis.linear_velocity.x;
  v.vy = chassis.linear_velocity.y;
  v.wz = r;
  return v;
}

void dynamic_vehicle_model::step(dynamic_vehicle &v, std::uint64_t tick) {
  _plant->restore(0, v.model);
  if (into_tick(_drive, tick) == 0) {
    // The navigator's tick (sim_world's override_command on a plant, in its floats): the steering
    // within the lock, the acceleration held to the top speed within a tick, given to the plant.
    const float dt = static_cast<float>(static_cast<double>(_drive.tick_steps) * _dt);
    const float lock = static_cast<float>(_drive.lock);
    const float steer = std::min(std::max(static_cast<float>(v.steer_cmd), -lock), lock);
    const float speed = static_cast<float>(v.model.body.vx);
    const float cap = static_cast<float>(_drive.vmax), a_b = static_cast<float>(_drive.a_hold);
    float accel = static_cast<float>(v.accel_cmd);
    if (speed >= 0.0f)
      accel = std::min(accel, std::max((cap - speed) / dt, -a_b));
    else
      accel = std::max(accel, std::min((-cap - speed) / dt, a_b));
    _plant->command(0, steer, accel);
  }
  const ph::dyn_bike_saved before = _plant->save(0);
  _plant->advance(); // one substep: one physics step
  v.model = _plant->save(0);
  // The box: put where the substep starts, moving as the model moved, for the world's step
  // (dyn_bike_plant::step_model, then the world).
  const double rear_x = _plant->rear_axle(0);
  const double off = -rear_x - _plant->params(0).lr;
  const cvc::nav::diff::dyn_state<double> &s0 = before.body, &o = v.model.body;
  const double c0 = std::cos(s0.psi), sn0 = std::sin(s0.psi);
  const double c1 = std::cos(o.psi), s1 = std::sin(o.psi);
  v.vx = c1 * o.vx - s1 * o.vy - o.r * off * s1;
  v.vy = s1 * o.vx + c1 * o.vy + o.r * off * c1;
  v.wz = o.r;
  v.box = {s0.x - rear_x * c0 + v.vx * _dt, s0.y - rear_x * sn0 + v.vy * _dt, v.box[2],
           s0.psi + o.r * _dt};
}

ph::body_state dynamic_vehicle_state(const dynamic_vehicle &v) {
  ph::body_state s;
  s.position = {v.box[0], v.box[1], v.box[2]};
  s.rotation = {std::cos(0.5 * v.box[3]), 0.0, 0.0, std::sin(0.5 * v.box[3])};
  s.linear_velocity = {v.vx, v.vy, 0.0};
  s.angular_velocity = {0.0, 0.0, v.wz};
  return s;
}

} // namespace ariadne
} // namespace cvc
