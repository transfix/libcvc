/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "dyn_world.h"

#include <algorithm>
#include <cmath>
#include <cvc/core/exception.h>
#include <cvc/nav/dyn_4w.h>
#include <cvc/physics/dyn_4w_plant.h>

namespace cvc {
namespace physics {

namespace {

using dim = cvc::world_units::dimension;
namespace diff = cvc::nav::diff;
constexpr double kPi = 3.14159265358979323846;

// The rearmost unsteered axle's x, as nav_plant's.
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

void add(dyn_4w_books &to, const dyn_4w_books &d) {
  to.drive += d.drive;
  to.brake += d.brake;
  to.dissipated += d.dissipated;
  to.lever += d.lever;
  to.grade += d.grade;
  to.ground += d.ground;
  to.limiter += d.limiter;
  to.contact += d.contact;
  to.residual += d.residual;
}

} // namespace

double dyn_4w_books::flow() const {
  return std::fabs(drive) + std::fabs(brake) + std::fabs(dissipated) + std::fabs(lever) +
         std::fabs(grade) + std::fabs(ground) + std::fabs(limiter) + std::fabs(contact);
}

dyn_4w_plant::dyn_4w_plant(std::vector<vehicle_desc> descs, dyn_4w_plant_options options)
    : _descs(std::move(descs)), _o(std::move(options)),
      _units(
          cvc::world_units::config{cvc::world_units::system::si, _o.plant.metres_per_world_unit}),
      _chassis(std::make_unique<dyn::dyn_chassis>()) {
  if (_descs.empty())
    throw cvc::physics_error("dyn_4w_plant: needs at least one vehicle_desc");
  if (!(_o.plant.metres_per_world_unit > 0.0) || !std::isfinite(_o.plant.metres_per_world_unit))
    throw cvc::physics_error("dyn_4w_plant: metres_per_world_unit must be finite and > 0");
  if (!(_o.substep > 0.0) || !std::isfinite(_o.substep))
    throw cvc::physics_error("dyn_4w_plant: the substep must be finite and > 0");
  if (!(_o.mu > 0.0))
    throw cvc::physics_error("dyn_4w_plant: mu must be > 0");
}

dyn_4w_plant::~dyn_4w_plant() = default;

bool dyn_4w_plant::collides() const { return _chassis->on(); }

body_id dyn_4w_plant::chassis(int i) const { return _chassis->on() ? _chassis->box(i) : body_id{}; }

double dyn_4w_plant::ground_height(double x, double y) const {
  return dyn::ground_at(_o.plant, x, y);
}

void dyn_4w_plant::attach(int n, const cvc::nav::plant_state *start,
                          const cvc::world_units::normalized_frame &frame, double dt) {
  frame.validate();
  if (n < 0 || (n > 0 && !start))
    throw cvc::physics_error("dyn_4w_plant::attach: needs n >= 0 starts");
  if (!(dt > 0.0))
    throw cvc::physics_error("dyn_4w_plant::attach: the tick must be > 0");
  if (_o.physics) {
    const double k = dt / _o.physics->config().fixed_dt;
    _nsub = static_cast<int>(std::lround(k));
    if (_nsub < 1 || std::fabs(k - _nsub) > 1e-6)
      throw cvc::physics_error("dyn_4w_plant::attach: nav's tick must be a whole number of the "
                               "world's steps");
  } else {
    _nsub = std::max(1, static_cast<int>(std::ceil(dt / _o.substep - 1e-9)));
  }
  _frame = frame;
  _dt = dt;
  const double h = dt / _nsub;
  cvc::world_units::normalized_frame si;
  si.scale = 1.0;
  _chassis->reset(_o.physics, _o.ground_bodies, _o.chassis_friction);
  _agents.clear();
  _agents.reserve(static_cast<std::size_t>(n));
  _z0 = 0.0;
  if (n > 0) {
    const cvc::world_units::xyz p0 =
        _units.normalized_point_to_si(start[0].x, start[0].y, 0.0, frame);
    _z0 = dyn::ground_at(_o.plant, p0.x, p0.y);
    if (!std::isfinite(_z0))
      _z0 = 0.0;
  }
  const ground_sampler ground{this};
  for (int i = 0; i < n; ++i) {
    agent a;
    a.desc = _descs[static_cast<std::size_t>(i) % _descs.size()];
    a.rear_x = rear_axle_x(a.desc);
    cvc::nav::dyn4_si s4 = nav_plant::dyn4_si_of(a.desc, 1.0, _o.drive_lag, _o.gravity);
    s4.grade_force = _o.grade_force;
    a.p = cvc::nav::dyn4_params_of(s4, si);
    a.lim.L = a.p.lf + a.p.lr;
    a.lim.delta_max = static_cast<float>(a.desc.max_steer);
    a.lim.hdt = static_cast<float>(h);
    a.lim.vmax = static_cast<float>(a.desc.max_speed > 0.0 ? a.desc.max_speed : 1e3);
    a.lim.allow_reverse = 1;
    const cvc::world_units::xyz p =
        _units.normalized_point_to_si(start[i].x, start[i].y, 0.0, frame);
    a.s = cvc::nav::diff::dyn4_state<double>{};
    a.s.x = p.x;
    a.s.y = p.y;
    a.s.psi = start[i].heading;
    // At rest on the ground: each corner at the ground under its wheel, heave, roll and pitch
    // fitted to the four.
    const double c = std::cos(a.s.psi), sn = std::sin(a.s.psi);
    const double lf = a.p.lf, lr = a.p.lr, tf = a.p.tf > 0.0f ? a.p.tf : 0.5,
                 tr = a.p.tr > 0.0f ? a.p.tr : 0.5;
    const double cgx = a.s.x + lr * c, cgy = a.s.y + lr * sn;
    const auto under = [&](double bx, double by) {
      double mu = 0.0, z = 0.0;
      ground.at(cgx + bx * c - by * sn, cgy + bx * sn + by * c, mu, z);
      return z;
    };
    const double fl = under(lf, tf), fr = under(lf, -tf), rl = under(-lr, tr), rr = under(-lr, -tr);
    a.s.phi = 0.5 * ((fl - fr) / (2.0 * tf) + (rl - rr) / (2.0 * tr));
    a.s.th = (0.5 * (rl + rr) - 0.5 * (fl + fr)) / (lf + lr);
    a.s.z = 0.5 * (fl + fr) + lf * a.s.th;
    a.stored0 = diff::dyn4_ph_hamiltonian(a.p, a.lim, ground, a.s);
    a.total = a.tick = dyn_4w_books{};
    a.total.stored = a.tick.stored = a.stored0;
    measure(a, 0.0, 0.0, a.s.psi, false);
    if (_chassis->on()) {
      vec3 pos;
      quat rot;
      chassis_pose(a, a.s, pos, rot);
      _chassis->add(a.desc, pos, rot);
    }
    _agents.push_back(a);
  }
}

void dyn_4w_plant::chassis_pose(const agent &a, const diff::dyn4_state<double> &s, vec3 &position,
                                quat &rotation) const {
  const double c = std::cos(s.psi), sn = std::sin(s.psi);
  rotation = dyn::attitude(s.psi, s.th, s.phi);
  const vec3 cg{s.x + a.p.lr * c, s.y + a.p.lr * sn, _z0 + a.p.h + s.z};
  // The box's centre is the body origin: the centre of gravity less its body-local offset.
  position = cg - dyn::rotate(rotation, {a.rear_x + a.p.lr, 0.0, a.desc.center_of_mass.z});
}

cvc::nav::plant_state dyn_4w_plant::state(int i) const {
  const agent &a = _agents.at(static_cast<std::size_t>(i));
  cvc::nav::plant_state s;
  const cvc::world_units::xyz n = _units.si_point_to_normalized(a.s.x, a.s.y, 0.0, _frame);
  s.x = static_cast<float>(n.x);
  s.y = static_cast<float>(n.y);
  s.heading = static_cast<float>(dyn::wrap(a.s.psi));
  s.speed = static_cast<float>(_units.to_normalized(a.s.vx, dim::velocity, _frame));
  s.steer = static_cast<float>(a.s.delta);
  // The most used tire: its slip angle over the Pacejka peak's, tan(pi / 2C) / B.
  if (std::fabs(a.s.vx) >= 2.0) {
    diff::dyn4_ph_parts<double> q;
    diff::dyn4_ph_parts_at(a.p, a.lim, ground_sampler{this}, a.s, q);
    const double C = a.p.C, t = std::tan(kPi / (2.0 * C));
    double use = 0.0;
    for (int w = 0; w < 4; ++w) {
      const double alpha = std::atan2(std::fabs(q.w[w]), q.ui[w]);
      use = std::max(use, alpha / (t / (w < 2 ? a.p.B_f : a.p.B_r)));
    }
    s.tire_use = static_cast<float>(use);
  }
  return s;
}

cvc::nav::plant_limits dyn_4w_plant::limits(int i) const {
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

void dyn_4w_plant::command(int i, float steer, float accel) {
  agent &a = _agents.at(static_cast<std::size_t>(i));
  const double lock = a.desc.max_steer;
  a.steer_cmd = std::max(-lock, std::min(lock, static_cast<double>(steer)));
  a.accel_cmd = _units.from_normalized(accel, dim::acceleration, _frame);
}

void dyn_4w_plant::advance() {
  const std::size_t n = _agents.size();
  const ground_sampler ground{this};
  std::vector<double> vx0(n), vy0(n), psi0(n);
  std::vector<diff::dyn4_state<double>> next(n);
  std::vector<diff::dyn4_ph_books<double>> books(n);
  for (std::size_t i = 0; i < n; ++i) {
    agent &a = _agents[i];
    vx0[i] = a.s.vx;
    vy0[i] = a.s.vy;
    psi0[i] = a.s.psi;
    a.tick = dyn_4w_books{};
    a.hits_tick = 0;
  }
  const bool world = _chassis->on();
  const double h = _dt / _nsub;
  for (int k = 0; k < _nsub; ++k) {
    for (std::size_t i = 0; i < n; ++i) {
      agent &a = _agents[i];
      // Nav's command as the Jolt plant's driver takes it (dyn_driver), against the slope along
      // the heading under the centre of gravity when gravity acts on it.
      const double c = std::cos(a.s.psi), sn = std::sin(a.s.psi);
      const double grade = _o.grade_force
                               ? dyn::grade_at(_o.plant, a.s.x + a.p.lr * c, a.s.y + a.p.lr * sn,
                                               a.s.psi, static_cast<double>(a.p.lf + a.p.lr))
                               : 0.0;
      const double accel = _o.driver ? dyn::drive(a.driver, a.desc, a.accel_cmd, a.s.vx, grade,
                                                  _o.gravity, a.p.tau_drive, h)
                                     : a.accel_cmd;
      diff::dyn4_plant_books<double>(a.p, a.lim, ground, a.s, a.steer_cmd, accel, next[i],
                                     books[i]);
      if (world) {
        // The box where this substep starts, moving as the model moved.
        vec3 pos;
        quat rot;
        chassis_pose(a, a.s, pos, rot);
        const diff::dyn4_state<double> &o = next[i];
        const double c1 = std::cos(o.psi), s1 = std::sin(o.psi);
        const double off = -a.rear_x - a.p.lr;
        const vec3 v{c1 * o.vx - s1 * o.vy - o.r * off * s1, s1 * o.vx + c1 * o.vy + o.r * off * c1,
                     o.zd};
        const vec3 w{c1 * o.phid - s1 * o.thd, s1 * o.phid + c1 * o.thd, o.r};
        _chassis->put(static_cast<int>(i), pos, rot, v, w);
      }
    }
    if (world)
      _chassis->step();
    for (std::size_t i = 0; i < n; ++i) {
      agent &a = _agents[i];
      const diff::dyn4_ph_books<double> &b = books[i];
      dyn_4w_books d;
      (b.supplied >= 0.0 ? d.drive : d.brake) = std::fabs(b.supplied);
      d.dissipated = b.dissipated;
      d.lever = b.lever;
      d.grade = b.grade;
      d.ground = b.ground;
      d.limiter = b.limiter;
      d.residual = b.residual;
      diff::dyn4_state<double> o = next[i];
      if (world && _chassis->touched(static_cast<int>(i))) {
        // The world's horizontal pose and twist, back into the model (heave, roll, pitch and the
        // actuators stay the model's); the energy they changed is the contact's.
        const body_state m = _chassis->moved(static_cast<int>(i));
        const double before = diff::dyn4_ph_hamiltonian(a.p, a.lim, ground, o);
        o.psi += dyn::wrap(dyn::yaw_of(m.rotation) - o.psi);
        const double c1 = std::cos(o.psi), s1 = std::sin(o.psi);
        const double off = -a.rear_x - a.p.lr;
        const vec3 cg =
            m.position + dyn::rotate(m.rotation, {a.rear_x + a.p.lr, 0.0, a.desc.center_of_mass.z});
        o.x = cg.x - a.p.lr * c1;
        o.y = cg.y - a.p.lr * s1;
        o.r = m.angular_velocity.z;
        const double vgx = m.linear_velocity.x + o.r * off * s1,
                     vgy = m.linear_velocity.y - o.r * off * c1;
        o.vx = c1 * vgx + s1 * vgy;
        o.vy = -s1 * vgx + c1 * vgy;
        d.contact = diff::dyn4_ph_hamiltonian(a.p, a.lim, ground, o) - before;
        ++a.hits_tick;
      }
      add(a.tick, d);
      dyn::measured(a.driver, a.s.vx, o.vx, h);
      a.s = o;
    }
  }
  for (std::size_t i = 0; i < n; ++i) {
    agent &a = _agents[i];
    a.hits += a.hits_tick;
    a.tick.stored = diff::dyn4_ph_hamiltonian(a.p, a.lim, ground, a.s);
    add(a.total, a.tick);
    a.total.stored = a.tick.stored;
    measure(a, vx0[i], vy0[i], psi0[i], true);
  }
}

double dyn_4w_plant::energy_step(int i) const {
  return _agents.at(static_cast<std::size_t>(i)).tick.drive;
}

cvc::nav::plant_sample dyn_4w_plant::sample(int i) const {
  return _agents.at(static_cast<std::size_t>(i)).sample;
}

void dyn_4w_plant::measure(agent &a, double vx0, double vy0, double psi0, bool tick) const {
  cvc::nav::plant_sample s;
  s.vx = a.s.vx;
  s.vy = a.s.vy;
  s.r = a.s.r;
  s.steer = a.s.delta;
  s.max_steer = a.desc.max_steer;
  s.steer_cmd = a.steer_cmd;
  s.accel_cmd = a.accel_cmd;
  if (tick && _dt > 0.0) {
    const double c0 = std::cos(psi0), s0 = std::sin(psi0), c1 = std::cos(a.s.psi),
                 s1 = std::sin(a.s.psi);
    const double dvx = (c1 * a.s.vx - s1 * a.s.vy) - (c0 * vx0 - s0 * vy0);
    const double dvy = (s1 * a.s.vx + c1 * a.s.vy) - (s0 * vx0 + c0 * vy0);
    const double pm = psi0 + 0.5 * dyn::wrap(a.s.psi - psi0);
    const double cm = std::cos(pm), sm = std::sin(pm);
    s.a_lat = (-dvx * sm + dvy * cm) / _dt;
    s.a_lon = (dvx * cm + dvy * sm) / _dt;
  }
  s.wheels = s.wheels_down = 4;
  a.sample = s;
}

} // namespace physics
} // namespace cvc
