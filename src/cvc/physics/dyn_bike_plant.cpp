/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cmath>
#include <cvc/core/exception.h>
#include <cvc/nav/dyn_bike.h>
#include <cvc/physics/dyn_bike_plant.h>

namespace cvc {
namespace physics {

namespace {

using dim = cvc::world_units::dimension;
namespace diff = cvc::nav::diff;
constexpr double kPi = 3.14159265358979323846;

// The rearmost unsteered axle's x, as nav_plant's (vehicle_desc::wheelbase()'s rear axle).
double rear_axle_x(const vehicle_desc &d) {
  double rear = 0.0;
  bool have = false;
  for (const axle_desc &a : d.axles)
    if (!a.steered && (!have || a.x < rear)) {
      rear = a.x;
      have = true;
    }
  return rear;
}

double wrap(double a) {
  while (a > kPi)
    a -= 2.0 * kPi;
  while (a <= -kPi)
    a += 2.0 * kPi;
  return a;
}

void add(dyn_bike_books &to, const dyn_bike_books &d) {
  to.drive += d.drive;
  to.brake += d.brake;
  to.dissipated += d.dissipated;
  to.floor += d.floor;
  to.grade += d.grade;
  to.limiter += d.limiter;
  to.residual += d.residual;
}

} // namespace

double dyn_bike_books::flow() const {
  return std::fabs(drive) + std::fabs(brake) + std::fabs(dissipated) + std::fabs(floor) +
         std::fabs(grade) + std::fabs(limiter);
}

dyn_bike_plant::dyn_bike_plant(std::vector<vehicle_desc> descs, dyn_bike_plant_options options)
    : _descs(std::move(descs)), _o(std::move(options)),
      _units(
          cvc::world_units::config{cvc::world_units::system::si, _o.plant.metres_per_world_unit}) {
  if (_descs.empty())
    throw cvc::physics_error("dyn_bike_plant: needs at least one vehicle_desc");
  if (!(_o.plant.metres_per_world_unit > 0.0) || !std::isfinite(_o.plant.metres_per_world_unit))
    throw cvc::physics_error("dyn_bike_plant: metres_per_world_unit must be finite and > 0");
  if (!(_o.substep > 0.0) || !std::isfinite(_o.substep))
    throw cvc::physics_error("dyn_bike_plant: the substep must be finite and > 0");
  if (!(_o.mu > 0.0))
    throw cvc::physics_error("dyn_bike_plant: mu must be > 0");
}

void dyn_bike_plant::attach(int n, const cvc::nav::plant_state *start,
                            const cvc::world_units::normalized_frame &frame, double dt) {
  frame.validate();
  if (n < 0 || (n > 0 && !start))
    throw cvc::physics_error("dyn_bike_plant::attach: needs n >= 0 starts");
  if (!(dt > 0.0))
    throw cvc::physics_error("dyn_bike_plant::attach: the tick must be > 0");
  _frame = frame;
  _dt = dt;
  _nsub = std::max(1, static_cast<int>(std::ceil(dt / _o.substep - 1e-9)));
  const double h = dt / _nsub;
  cvc::world_units::normalized_frame si;
  si.scale = 1.0;
  _agents.clear();
  _agents.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    agent a;
    a.desc = _descs[static_cast<std::size_t>(i) % _descs.size()];
    a.rear_x = rear_axle_x(a.desc);
    a.p = cvc::nav::dyn_bike_params_of(
        nav_plant::dyn_bike_si_of(a.desc, _o.mu, 1.0, _o.drive_lag, _o.gravity), si);
    a.lim.L = a.p.lf + a.p.lr;
    a.lim.delta_max = static_cast<float>(a.desc.max_steer);
    a.lim.hdt = static_cast<float>(h);
    a.lim.vmax = static_cast<float>(a.desc.max_speed > 0.0 ? a.desc.max_speed : 1e3);
    a.lim.allow_reverse = 1;
    const cvc::world_units::xyz p =
        _units.normalized_point_to_si(start[i].x, start[i].y, 0.0, frame);
    a.s.x = p.x;
    a.s.y = p.y;
    a.s.psi = start[i].heading;
    a.kinetic0 = 0.0;
    a.total = a.tick = dyn_bike_books{};
    measure(a, 0.0, 0.0, a.s.psi, false);
    _agents.push_back(a);
  }
}

cvc::nav::plant_state dyn_bike_plant::state(int i) const {
  const agent &a = _agents.at(static_cast<std::size_t>(i));
  cvc::nav::plant_state s;
  const cvc::world_units::xyz n = _units.si_point_to_normalized(a.s.x, a.s.y, 0.0, _frame);
  s.x = static_cast<float>(n.x);
  s.y = static_cast<float>(n.y);
  s.heading = static_cast<float>(wrap(a.s.psi));
  s.speed = static_cast<float>(_units.to_normalized(a.s.vx, dim::velocity, _frame));
  s.steer = static_cast<float>(a.s.delta);
  // The most used tire: its slip angle over the Pacejka peak's, tan(pi / 2C) / B.
  if (std::fabs(a.s.vx) >= 2.0) {
    double Dt[2][2], kf, kr, af, ar;
    diff::dyn_ph_tangent_D(a.p, a.s, Dt, kf, kr, af, ar);
    const double C = a.p.C, peak_f = std::tan(kPi / (2.0 * C)) / a.p.B_f,
                 peak_r = std::tan(kPi / (2.0 * C)) / a.p.B_r;
    s.tire_use = static_cast<float>(std::max(std::fabs(af) / peak_f, std::fabs(ar) / peak_r));
  }
  return s;
}

cvc::nav::plant_limits dyn_bike_plant::limits(int i) const {
  const agent &a = _agents.at(static_cast<std::size_t>(i));
  const vehicle_desc &d = a.desc;
  const nav_plant_options &o = _o.plant;
  cvc::nav::plant_limits l;
  l.wheelbase = static_cast<float>(_units.to_normalized(d.wheelbase(), dim::length, _frame));
  l.max_steer = static_cast<float>(d.max_steer);
  l.steer_rate = static_cast<float>(d.max_steer_rate);
  l.a_brake = static_cast<float>(
      _units.to_normalized(o.brake_margin * d.max_decel, dim::acceleration, _frame));
  l.latency = static_cast<float>(o.latency);
  if (!o.plan_with_vehicle)
    return l;
  l.a_max = static_cast<float>(
      _units.to_normalized(o.accel_margin * d.max_accel, dim::acceleration, _frame));
  if (d.max_speed > 0.0)
    l.vmax = static_cast<float>(_units.to_normalized(d.max_speed, dim::velocity, _frame));
  double a_lat = o.lateral_g * _o.gravity;
  const double ssf = nav_plant::terrain_config_of(d, _o.mu, _o.gravity).tip_ratio;
  if (ssf > 0.0)
    a_lat = std::min(a_lat, o.tip_margin * ssf * _o.gravity);
  l.a_lat = static_cast<float>(_units.to_normalized(a_lat, dim::acceleration, _frame));
  // The body: the chassis box about the body origin, and the wheels where they stand out of it.
  double ahead = d.half_extents.x, behind = -d.half_extents.x, half_width = d.half_extents.y;
  for (const axle_desc &ax : d.axles) {
    ahead = std::max(ahead, ax.x + ax.wheel_radius);
    behind = std::min(behind, ax.x - ax.wheel_radius);
    half_width = std::max(half_width, ax.half_track + 0.5 * ax.wheel_width);
  }
  l.half_width = static_cast<float>(_units.to_normalized(half_width, dim::length, _frame));
  l.front = static_cast<float>(_units.to_normalized(ahead - a.rear_x, dim::length, _frame));
  l.rear = static_cast<float>(_units.to_normalized(a.rear_x - behind, dim::length, _frame));
  return l;
}

void dyn_bike_plant::command(int i, float steer, float accel) {
  agent &a = _agents.at(static_cast<std::size_t>(i));
  const double lock = a.desc.max_steer;
  a.steer_cmd = std::max(-lock, std::min(lock, static_cast<double>(steer)));
  a.accel_cmd = _units.from_normalized(accel, dim::acceleration, _frame);
}

void dyn_bike_plant::advance() {
  for (agent &a : _agents) {
    const double vx0 = a.s.vx, vy0 = a.s.vy, psi0 = a.s.psi;
    a.tick = dyn_bike_books{};
    for (int k = 0; k < _nsub; ++k) {
      diff::dyn_state<double> o;
      diff::dyn_ph_books<double> b;
      diff::dyn_plant_books<double>(a.p, a.lim, a.s, a.steer_cmd, a.accel_cmd, 0.0, o, b);
      dyn_bike_books d;
      (b.supplied >= 0.0 ? d.drive : d.brake) = std::fabs(b.supplied);
      d.dissipated = b.dissipated;
      d.floor = b.floor;
      d.grade = b.grade;
      d.limiter = b.limiter;
      d.residual = b.residual;
      add(a.tick, d);
      a.s = o;
    }
    a.tick.kinetic = diff::dyn_ph_hamiltonian(a.p, a.s);
    add(a.total, a.tick);
    a.total.kinetic = a.tick.kinetic;
    measure(a, vx0, vy0, psi0, true);
  }
}

double dyn_bike_plant::energy_step(int i) const {
  return _agents.at(static_cast<std::size_t>(i)).tick.drive;
}

cvc::nav::plant_sample dyn_bike_plant::sample(int i) const {
  return _agents.at(static_cast<std::size_t>(i)).sample;
}

void dyn_bike_plant::measure(agent &a, double vx0, double vy0, double psi0, bool tick) const {
  cvc::nav::plant_sample s;
  s.vx = a.s.vx;
  s.vy = a.s.vy;
  s.r = a.s.r;
  s.steer = a.s.delta;
  s.max_steer = a.desc.max_steer;
  s.steer_cmd = a.steer_cmd;
  s.accel_cmd = a.accel_cmd;
  if (tick && _dt > 0.0) {
    // The centre of mass's horizontal velocity's change over the tick, across the mean heading
    // (plant_log's and nav_plant's).
    const double c0 = std::cos(psi0), s0 = std::sin(psi0), c1 = std::cos(a.s.psi),
                 s1 = std::sin(a.s.psi);
    const double dvx = (c1 * a.s.vx - s1 * a.s.vy) - (c0 * vx0 - s0 * vy0);
    const double dvy = (s1 * a.s.vx + c1 * a.s.vy) - (s0 * vx0 + c0 * vy0);
    const double pm = psi0 + 0.5 * wrap(a.s.psi - psi0);
    const double cm = std::cos(pm), sm = std::sin(pm);
    s.a_lat = (-dvx * sm + dvy * cm) / _dt;
    s.a_lon = (dvx * cm + dvy * sm) / _dt;
  }
  s.wheels = s.wheels_down = 2 * static_cast<int>(a.desc.axles.size());
  a.sample = s;
}

} // namespace physics
} // namespace cvc
