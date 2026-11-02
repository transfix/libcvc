/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "dyn_world.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cvc/core/exception.h>
#include <cvc/nav/drive.h>
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

void add(dyn_bike_books &to, const dyn_bike_books &d) {
  to.drive += d.drive;
  to.brake += d.brake;
  to.dissipated += d.dissipated;
  to.slip += d.slip;
  to.floor += d.floor;
  to.grade += d.grade;
  to.limiter += d.limiter;
  to.contact += d.contact;
  to.contact_stored += d.contact_stored;
  to.contact_dissipated += d.contact_dissipated;
  to.contact_exchange += d.contact_exchange;
  to.residual += d.residual;
}

} // namespace

double dyn_bike_books::flow() const {
  return std::fabs(drive) + std::fabs(brake) + std::fabs(dissipated) + std::fabs(slip) +
         std::fabs(floor) + std::fabs(grade) + std::fabs(limiter) + std::fabs(contact) +
         std::fabs(contact_stored) + std::fabs(contact_dissipated) + std::fabs(contact_exchange);
}

dyn_bike_plant::dyn_bike_plant(std::vector<vehicle_desc> descs, dyn_bike_plant_options options)
    : _descs(std::move(descs)), _o(std::move(options)),
      _units(
          cvc::world_units::config{cvc::world_units::system::si, _o.plant.metres_per_world_unit}),
      _chassis(std::make_unique<dyn::dyn_chassis>()) {
  if (_descs.empty())
    throw cvc::physics_error("dyn_bike_plant: needs at least one vehicle_desc");
  if (!(_o.plant.metres_per_world_unit > 0.0) || !std::isfinite(_o.plant.metres_per_world_unit))
    throw cvc::physics_error("dyn_bike_plant: metres_per_world_unit must be finite and > 0");
  if (!(_o.substep > 0.0) || !std::isfinite(_o.substep))
    throw cvc::physics_error("dyn_bike_plant: the substep must be finite and > 0");
  if (!(_o.mu > 0.0))
    throw cvc::physics_error("dyn_bike_plant: mu must be > 0");
}

dyn_bike_plant::~dyn_bike_plant() = default;

bool dyn_bike_plant::collides() const { return _chassis->on(); }

std::function<void(double, double, double &, double &, double &)>
dyn_bike_plant::sdf_field(const cvc::nav::field_stack &fs, int plane,
                          double metres_per_world_unit) {
  if (!fs.data || fs.H < 2 || fs.W < 2 || plane < 0 || plane >= fs.M ||
      !(fs.mxx > fs.mnx && fs.mxy > fs.mny) || !(fs.S > 0.0) || !(metres_per_world_unit > 0.0))
    throw cvc::physics_error("dyn_bike_plant::sdf_field: not a signed-distance grid");
  const float *ph = fs.data + static_cast<long>(plane) * 3 * fs.H * fs.W;
  const int H = fs.H, W = fs.W;
  const double mpw = metres_per_world_unit, mnx = fs.mnx, mny = fs.mny;
  const double sx = (W - 1) / (fs.mxx - fs.mnx), sy = (H - 1) / (fs.mxy - fs.mny);
  const double to_m = mpw / fs.S; // phi is in nav's normalized units
  return [=](double x, double y, double &phi, double &gx, double &gy) {
    const double fx = (x / mpw - mnx) * sx, fy = (y / mpw - mny) * sy;
    const int c = std::max(0, std::min(W - 2, static_cast<int>(std::floor(fx))));
    const int r = std::max(0, std::min(H - 2, static_cast<int>(std::floor(fy))));
    const double ax = std::min(1.0, std::max(0.0, fx - c)),
                 ay = std::min(1.0, std::max(0.0, fy - r));
    const double p00 = ph[r * W + c], p01 = ph[r * W + c + 1];
    const double p10 = ph[(r + 1) * W + c], p11 = ph[(r + 1) * W + c + 1];
    phi = to_m * ((1.0 - ay) * ((1.0 - ax) * p00 + ax * p01) + ay * ((1.0 - ax) * p10 + ax * p11));
    // The bilinear's own gradient (zero across the clamp, off the grid).
    gx = fx > 0.0 && fx < W - 1 ? to_m * sx / mpw * ((1.0 - ay) * (p01 - p00) + ay * (p11 - p10))
                                : 0.0;
    gy = fy > 0.0 && fy < H - 1 ? to_m * sy / mpw * ((1.0 - ax) * (p10 - p00) + ax * (p11 - p01))
                                : 0.0;
  };
}

void dyn_bike_plant::contacts(const std::vector<diff::dyn_state<double>> &s,
                              std::vector<diff::dyn_contact> &field,
                              std::vector<diff::dyn_contact> &pair) const {
  const std::size_t n = _agents.size();
  field.assign(n, diff::dyn_contact{});
  pair.assign(n, diff::dyn_contact{});
  // Each agent's centre of gravity, heading and twist.
  std::vector<diff::dyn_body> b(n);
  for (std::size_t i = 0; i < n; ++i) {
    const double lr = _agents[i].p.lr, c = std::cos(s[i].psi), sn = std::sin(s[i].psi);
    b[i] = {s[i].x + lr * c, s[i].y + lr * sn, s[i].psi, s[i].vx, s[i].vy, s[i].r};
  }
  const diff::dyn_contact_params &cp = _o.contact_params;
  if (_o.contact_field) {
    struct sampler {
      const std::function<void(double, double, double &, double &, double &)> *f;
      void at(double x, double y, double &phi, double &gx, double &gy) const {
        (*f)(x, y, phi, gx, gy);
      }
    } smp{&_o.contact_field};
    for (std::size_t i = 0; i < n; ++i)
      diff::dyn_contact_field(cp, _agents[i].points, smp, b[i], field[i]);
  }
  if (!_o.contact_vehicles)
    return;
  const auto reach = [&](std::size_t i) {
    const diff::dyn_footprint &f = _agents[i].fp;
    return std::hypot(std::max(f.front, f.rear), f.half_width);
  };
  for (std::size_t i = 0; i < n; ++i)
    for (std::size_t j = i + 1; j < n; ++j)
      if (std::hypot(b[i].x - b[j].x, b[i].y - b[j].y) < reach(i) + reach(j) + cp.skin)
        diff::dyn_contact_pair(cp, _agents[i].points, _agents[i].fp, b[i], _agents[j].points,
                               _agents[j].fp, b[j], pair[i], pair[j]);
}

double dyn_bike_plant::wheel_speed(int i, int axle) const {
  const agent &a = _agents.at(static_cast<std::size_t>(i));
  if (axle < 0 || axle > 1)
    throw cvc::physics_error("dyn_bike_plant::wheel_speed: the axle is 0 (front) or 1 (rear)");
  return _o.wheel_spin ? a.om[axle] : a.s.vx / a.p.r_w;
}

body_id dyn_bike_plant::chassis(int i) const {
  return _chassis->on() ? _chassis->box(i) : body_id{};
}

void dyn_bike_plant::attach(int n, const cvc::nav::plant_state *start,
                            const cvc::world_units::normalized_frame &frame, double dt) {
  frame.validate();
  if (n < 0 || (n > 0 && !start))
    throw cvc::physics_error("dyn_bike_plant::attach: needs n >= 0 starts");
  if (!(dt > 0.0))
    throw cvc::physics_error("dyn_bike_plant::attach: the tick must be > 0");
  if (_o.physics) {
    // One world step per substep: nav's tick must be a whole number of them.
    const double k = dt / _o.physics->config().fixed_dt;
    _nsub = static_cast<int>(std::lround(k));
    if (_nsub < 1 || std::fabs(k - _nsub) > 1e-6)
      throw cvc::physics_error("dyn_bike_plant::attach: nav's tick must be a whole number of the "
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
  for (int i = 0; i < n; ++i) {
    agent a;
    a.desc = _descs[static_cast<std::size_t>(i) % _descs.size()];
    a.rear_x = rear_axle_x(a.desc);
    for (const axle_desc &ax : a.desc.axles)
      a.half_track = std::max(a.half_track, ax.half_track);
    a.p = cvc::nav::dyn_bike_params_of(
        nav_plant::dyn_bike_si_of(a.desc, _o.mu, 1.0, _o.drive_lag, _o.gravity), si);
    a.lim.L = a.p.lf + a.p.lr;
    a.lim.delta_max = static_cast<float>(a.desc.max_steer);
    a.lim.hdt = static_cast<float>(h);
    a.lim.vmax = static_cast<float>(a.desc.max_speed > 0.0 ? a.desc.max_speed : 1e3);
    a.lim.allow_reverse = 1;
    // The outline about the model's centre of gravity (body x rear_x + lr): the chassis box and
    // the wheels where they stand out of it, as limits() reads them.
    {
      double ahead = a.desc.half_extents.x, behind = -a.desc.half_extents.x;
      double half_width = a.desc.half_extents.y;
      for (const axle_desc &ax : a.desc.axles) {
        ahead = std::max(ahead, ax.x + ax.wheel_radius);
        behind = std::min(behind, ax.x - ax.wheel_radius);
        half_width = std::max(half_width, ax.half_track + 0.5 * ax.wheel_width);
      }
      const double cg = a.rear_x + a.p.lr;
      a.fp.front = ahead - cg;
      a.fp.rear = cg - behind;
      a.fp.half_width = half_width;
      a.points = diff::dyn_footprint_points(a.fp, _o.contact_params);
    }
    const cvc::world_units::xyz p =
        _units.normalized_point_to_si(start[i].x, start[i].y, 0.0, frame);
    a.s.x = p.x;
    a.s.y = p.y;
    a.s.psi = start[i].heading;
    a.om[0] = a.om[1] = 0.0;
    a.kinetic0 = 0.0;
    a.total = a.tick = dyn_bike_books{};
    measure(a, 0.0, 0.0, a.s.psi, false);
    if (_chassis->on()) {
      vec3 pos;
      quat rot;
      chassis_pose(a, a.s, pos, rot);
      _chassis->add(a.desc, pos, rot);
    }
    _agents.push_back(a);
  }
  if (_o.contact) {
    // The barrier's energy at attach (an agent started overlapping something holds some).
    std::vector<diff::dyn_state<double>> now(_agents.size());
    for (std::size_t i = 0; i < _agents.size(); ++i)
      now[i] = _agents[i].s;
    std::vector<diff::dyn_contact> f, p;
    contacts(now, f, p);
    for (std::size_t i = 0; i < _agents.size(); ++i)
      _agents[i].tick.contact_energy = _agents[i].total.contact_energy = f[i].energy + p[i].energy;
  }
}

void dyn_bike_plant::chassis_pose(const agent &a, const diff::dyn_state<double> &s, vec3 &position,
                                  quat &rotation) const {
  const double c = std::cos(s.psi), sn = std::sin(s.psi);
  const double L = a.p.lf + a.p.lr;
  const double cgx = s.x + a.p.lr * c, cgy = s.y + a.p.lr * sn;
  const double hr = dyn::ground_at(_o.plant, s.x, s.y),
               hf = dyn::ground_at(_o.plant, s.x + L * c, s.y + L * sn);
  const double t = a.half_track > 0.0 ? a.half_track : 0.5;
  const double hl = dyn::ground_at(_o.plant, cgx - t * sn, cgy + t * c),
               hrt = dyn::ground_at(_o.plant, cgx + t * sn, cgy - t * c);
  const double th = std::atan2(hr - hf, L), phi = std::atan2(hl - hrt, 2.0 * t);
  rotation = dyn::attitude(s.psi, th, phi);
  const double cgz = dyn::ground_at(_o.plant, cgx, cgy) + a.p.h;
  position = {s.x - a.rear_x * c, s.y - a.rear_x * sn, cgz - a.desc.center_of_mass.z};
}

cvc::nav::plant_state dyn_bike_plant::state(int i) const {
  const agent &a = _agents.at(static_cast<std::size_t>(i));
  cvc::nav::plant_state s;
  const cvc::world_units::xyz n = _units.si_point_to_normalized(a.s.x, a.s.y, 0.0, _frame);
  s.x = static_cast<float>(n.x);
  s.y = static_cast<float>(n.y);
  s.heading = static_cast<float>(dyn::wrap(a.s.psi));
  s.speed = static_cast<float>(_units.to_normalized(a.s.vx, dim::velocity, _frame));
  s.steer = static_cast<float>(a.s.delta);
  // The most used tire: its slip angle over the Pacejka peak's, tan(pi / 2C) / B; with wheel spin
  // as nav_plant reads Jolt's, the slip ratio and angle over theirs together.
  if (std::fabs(a.s.vx) >= 2.0) {
    double Dt[2][2], kf, kr, af, ar;
    diff::dyn_ph_tangent_D(a.p, a.s, Dt, kf, kr, af, ar);
    const double C = a.p.C, peak_f = std::tan(kPi / (2.0 * C)) / a.p.B_f,
                 peak_r = std::tan(kPi / (2.0 * C)) / a.p.B_r;
    double use_f = std::fabs(af) / peak_f, use_r = std::fabs(ar) / peak_r;
    if (_o.wheel_spin) {
      diff::dyn_spin_state<double> ss;
      ss.b = a.s;
      ss.om_f = a.om[0];
      ss.om_r = a.om[1];
      diff::dyn_spin_ph_parts<double> q;
      diff::dyn_spin_ph_parts_at(a.p, ss, q);
      const double peak_x = std::tan(kPi / (2.0 * a.p.C_x)) / a.p.B_x, e2 = a.p.eps * a.p.eps;
      const double kx_f = q.sf / std::sqrt(q.gf * q.gf + e2) / peak_x;
      const double kx_r = q.sr / std::sqrt(q.gr * q.gr + e2) / peak_x;
      use_f = std::hypot(use_f, kx_f);
      use_r = std::hypot(use_r, kx_r);
    }
    s.tire_use = static_cast<float>(std::max(use_f, use_r));
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
  const std::size_t n = _agents.size();
  std::vector<double> vx0(n), vy0(n), psi0(n);
  std::vector<diff::dyn_state<double>> next(n);
  std::vector<diff::dyn_ph_books<double>> books(n);
  std::vector<diff::dyn_spin_ph_books<double>> spin_books(n);
  std::vector<std::array<double, 2>> om_next(n);
  for (std::size_t i = 0; i < n; ++i) {
    agent &a = _agents[i];
    vx0[i] = a.s.vx;
    vy0[i] = a.s.vy;
    psi0[i] = a.s.psi;
    a.tick = dyn_bike_books{};
    a.hits_tick = 0;
    a.statics_tick = 0;
    a.touched = -1;
    a.touch_depth = 0.0;
  }
  const bool world = _chassis->on();
  const double h = _dt / _nsub;
  std::vector<double> port(n, 0.0);
  std::vector<diff::dyn_step_free<double>> free(n);
  std::vector<diff::dyn_contact> cf0, cp0, cf1, cp1; // the contact at a substep's start and end
  for (int k = 0; k < _nsub; ++k) {
    if (_o.contact) {
      std::vector<diff::dyn_state<double>> now(n);
      for (std::size_t i = 0; i < n; ++i)
        now[i] = _agents[i].s;
      contacts(now, cf0, cp0);
    }
    for (std::size_t i = 0; i < n; ++i) {
      agent &a = _agents[i];
      // The contact's wrench, held over the step through the external port.
      double W[3] = {0.0, 0.0, 0.0};
      const double *wrench = nullptr;
      if (_o.contact) {
        diff::dyn_contact &c = a.cnow;
        c = cf0[i];
        for (int j = 0; j < 3; ++j) {
          c.Wc[j] += cp0[i].Wc[j];
          c.Wd[j] += cp0[i].Wd[j];
        }
        c.energy += cp0[i].energy;
        c.damping += cp0[i].damping;
        c.touching += cp0[i].touching;
        c.wrench(W);
        wrench = W;
      }
      // The slope along the heading under the centre of gravity, over a wheelbase.
      const double c = std::cos(a.s.psi), sn = std::sin(a.s.psi);
      const double grade = dyn::grade_at(_o.plant, a.s.x + a.p.lr * c, a.s.y + a.p.lr * sn, a.s.psi,
                                         static_cast<double>(a.p.lf + a.p.lr));
      // Nav's command as the Jolt plant's driver takes it (dyn_driver).
      const double accel = _o.driver ? dyn::drive(a.driver, a.desc, a.accel_cmd, a.s.vx, grade,
                                                  _o.gravity, a.p.tau_drive, h)
                                     : a.accel_cmd;
      if (_o.wheel_spin) {
        diff::dyn_spin_state<double> ss, so;
        ss.b = a.s;
        ss.om_f = a.om[0];
        ss.om_r = a.om[1];
        diff::dyn_spin_plant_books<double>(a.p, a.lim, ss, a.steer_cmd, accel, grade, so,
                                           spin_books[i], wrench, &port[i], &free[i]);
        next[i] = so.b;
        om_next[i] = {so.om_f, so.om_r};
      } else {
        diff::dyn_plant_books<double>(a.p, a.lim, a.s, a.steer_cmd, accel, grade, next[i], books[i],
                                      wrench, &port[i], &free[i]);
      }
      if (world) {
        // The box where this substep starts, moving as the model moved: its centre's velocity is
        // the centre of gravity's plus the yaw rate's sweep of the offset along the heading.
        vec3 pos;
        quat rot;
        chassis_pose(a, a.s, pos, rot);
        const diff::dyn_state<double> &o = next[i];
        const double c1 = std::cos(o.psi), s1 = std::sin(o.psi);
        const double off = -a.rear_x - a.p.lr; // from the centre of gravity to the box's centre
        const vec3 v{c1 * o.vx - s1 * o.vy - o.r * off * s1, s1 * o.vx + c1 * o.vy + o.r * off * c1,
                     0.0};
        _chassis->put(static_cast<int>(i), pos, rot, v, {0.0, 0.0, o.r});
      }
    }
    if (world)
      _chassis->step();
    // The barrier's energy where the step left the agents.
    if (_o.contact)
      contacts(next, cf1, cp1);
    for (std::size_t i = 0; i < n; ++i) {
      agent &a = _agents[i];
      dyn_bike_books d;
      if (_o.wheel_spin) {
        const diff::dyn_spin_ph_books<double> &b = spin_books[i];
        (b.supplied >= 0.0 ? d.drive : d.brake) = std::fabs(b.supplied);
        d.brake += b.braked;
        d.dissipated = b.dissipated;
        d.slip = b.slip;
        d.floor = b.floor;
        d.grade = b.grade;
        d.limiter = b.limiter;
        d.residual = b.residual;
        a.om[0] = om_next[i][0];
        a.om[1] = om_next[i][1];
      } else {
        const diff::dyn_ph_books<double> &b = books[i];
        (b.supplied >= 0.0 ? d.drive : d.brake) = std::fabs(b.supplied);
        d.dissipated = b.dissipated;
        d.floor = b.floor;
        d.grade = b.grade;
        d.limiter = b.limiter;
        d.residual = b.residual;
      }
      if (_o.contact) {
        // The port's energy, split: the field barrier's work, the vehicles' barrier's work, the
        // dampers'. Over the step the barriers' work is minus their energy's change, to second
        // order: the field's difference joins the residual, the pairs' is what the vehicles
        // passed each other (contact_exchange; over a pair, second order too).
        const diff::dyn_step_free<double> &e = free[i];
        const auto work = [&](const double Wv[3]) {
          return diff::dyn_ph_port_work(h, a.s.vx, a.s.vy, a.s.r, e.vx, e.vy, e.r, Wv);
        };
        double Wd[3];
        for (int j = 0; j < 3; ++j)
          Wd[j] = cf0[i].Wd[j] + cp0[i].Wd[j];
        const double work_field = work(cf0[i].Wc), work_pair = work(cp0[i].Wc), work_d = work(Wd);
        const double d_field = cf1[i].energy - cf0[i].energy;
        const double d_pair = cp1[i].energy - cp0[i].energy;
        d.contact_stored = d_field + d_pair;
        d.contact_dissipated = -work_d;
        d.contact_exchange = work_pair + d_pair;
        d.residual += work_field + d_field;
        a.tick.contact_energy = cf1[i].energy + cp1[i].energy;
      }
      diff::dyn_state<double> o = next[i];
      if (world && _chassis->touched(static_cast<int>(i))) {
        // The world's pose and twist, back into the model; the energy they changed is the
        // contact's. The actuators keep the model's.
        const body_state m = _chassis->moved(static_cast<int>(i));
        const double before = diff::dyn_ph_hamiltonian(a.p, o);
        o.psi += dyn::wrap(dyn::yaw_of(m.rotation) - o.psi);
        const double c1 = std::cos(o.psi), s1 = std::sin(o.psi);
        const double off = -a.rear_x - a.p.lr;
        o.x = m.position.x + a.rear_x * c1;
        o.y = m.position.y + a.rear_x * s1;
        o.r = m.angular_velocity.z;
        const double vgx = m.linear_velocity.x + o.r * off * s1,
                     vgy = m.linear_velocity.y - o.r * off * c1;
        o.vx = c1 * vgx + s1 * vgy;
        o.vy = -s1 * vgx + c1 * vgy;
        d.contact = diff::dyn_ph_hamiltonian(a.p, o) - before;
        ++a.hits_tick;
        dyn::count_contacts(*_chassis, static_cast<int>(i), a.statics_tick, a.touched,
                            a.touch_depth);
      }
      add(a.tick, d);
      dyn::measured(a.driver, a.s.vx, o.vx, h);
      a.s = o;
    }
  }
  for (std::size_t i = 0; i < n; ++i) {
    agent &a = _agents[i];
    a.hits += a.hits_tick;
    a.tick.kinetic = diff::dyn_ph_hamiltonian(a.p, a.s);
    if (_o.wheel_spin)
      a.tick.kinetic += 0.5 * (a.p.J_f * a.om[0] * a.om[0] + a.p.J_r * a.om[1] * a.om[1]);
    add(a.total, a.tick);
    a.total.kinetic = a.tick.kinetic;
    if (_o.contact)
      a.total.contact_energy = a.tick.contact_energy;
    measure(a, vx0[i], vy0[i], psi0[i], true);
  }
}

double dyn_bike_plant::energy_step(int i) const {
  return _agents.at(static_cast<std::size_t>(i)).tick.drive;
}

int dyn_bike_plant::static_contacts_step(int i) const {
  return _agents.at(static_cast<std::size_t>(i)).statics_tick;
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
    const double pm = psi0 + 0.5 * dyn::wrap(a.s.psi - psi0);
    const double cm = std::cos(pm), sm = std::sin(pm);
    s.a_lat = (-dvx * sm + dvy * cm) / _dt;
    s.a_lon = (dvx * cm + dvy * sm) / _dt;
  }
  // The body's attitude, the ground's under it (the chassis box's in the world).
  vec3 position;
  quat rotation;
  chassis_pose(a, a.s, position, rotation);
  dyn::attitude_of(rotation, s);
  s.wheels = s.wheels_down = 2 * static_cast<int>(a.desc.axles.size());
  s.contacts = tick ? a.statics_tick : 0;
  s.vehicle_contact = tick ? a.touched : -1;
  // A single track on the ground: no left and right loads, no roll rate, no wheel off it; and
  // contacts only in a world.
  s.measured = cvc::nav::plant_sample::has_attitude;
  if (_chassis->on())
    s.measured |=
        cvc::nav::plant_sample::has_contacts | cvc::nav::plant_sample::has_vehicle_contact;
  a.sample = s;
}

} // namespace physics
} // namespace cvc
