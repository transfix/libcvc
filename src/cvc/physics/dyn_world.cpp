/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "dyn_world.h"

#include "det_math.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <map>
#include <tuple>

namespace cvc {
namespace physics {
namespace dyn {

namespace {
constexpr double kPi = 3.14159265358979323846;

quat mul(const quat &a, const quat &b) {
  return {
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
} // namespace

double ground_at(const nav_plant_options &o, double x, double y) {
  return o.ground_height ? o.ground_height(x, y) : o.ground_z;
}

double grade_at(const nav_plant_options &o, double x, double y, double psi, double span) {
  if (!o.ground_height || !(span > 0.0))
    return 0.0;
  const double c = det::cos(psi), s = det::sin(psi), d = 0.5 * span;
  const double rise = o.ground_height(x + c * d, y + s * d) - o.ground_height(x - c * d, y - s * d);
  if (!std::isfinite(rise))
    return 0.0;
  return rise / std::sqrt(span * span + rise * rise);
}

quat attitude(double psi, double th, double phi) {
  const quat z{det::cos(0.5 * psi), 0.0, 0.0, det::sin(0.5 * psi)};
  const quat y{det::cos(0.5 * th), 0.0, det::sin(0.5 * th), 0.0};
  const quat x{det::cos(0.5 * phi), det::sin(0.5 * phi), 0.0, 0.0};
  return mul(mul(z, y), x);
}

vec3 rotate(const quat &q, const vec3 &v) {
  // v + 2 w (u x v) + 2 u x (u x v), u the vector part.
  const vec3 u{q.x, q.y, q.z};
  const vec3 t{2.0 * (u.y * v.z - u.z * v.y), 2.0 * (u.z * v.x - u.x * v.z),
               2.0 * (u.x * v.y - u.y * v.x)};
  return {v.x + q.w * t.x + (u.y * t.z - u.z * t.y), v.y + q.w * t.y + (u.z * t.x - u.x * t.z),
          v.z + q.w * t.z + (u.x * t.y - u.y * t.x)};
}

double yaw_of(const quat &q) {
  return det::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double wrap(double a) {
  while (a > kPi)
    a -= 2.0 * kPi;
  while (a <= -kPi)
    a += 2.0 * kPi;
  return a;
}

double drive(dyn_driver &driver, const vehicle_desc &d, double want, double vx, double grade,
             double gravity, double drive_lag, double h) {
  // vehicle.cpp's: the deadband and hold speed, the correction's gain and the trim's rate (1/s).
  constexpr double kDeadband = 0.05, kHoldSpeed = 0.3, kGain = 0.5, kTrim = 2.0;
  constexpr double kHoldTau = 0.1; // s: the shortest the hold eases the speed to zero over
  if (std::fabs(want) <= kDeadband && std::fabs(vx) < kHoldSpeed) {
    // Slow, with nothing asked: on the brakes, the grade included, damped critically behind the
    // drive's lag.
    driver.trim = 0.0;
    const double a = -vx / std::max(kHoldTau, 4.0 * drive_lag) + gravity * grade;
    return std::max(-d.max_decel, std::min(d.max_decel, a));
  }
  if ((want > 0.0) != (driver.trim_sign > 0.0)) {
    driver.trim = 0.0;
    driver.trim_sign = want > 0.0 ? 1.0 : -1.0;
  }
  const double e = want - driver.got;
  const double u = want + gravity * grade + kGain * e + driver.trim;
  // Forward the engine's, against the motion the brakes', backward the engine's in reverse.
  const double hi = vx < -kHoldSpeed ? d.max_decel : d.max_accel;
  const double lo = vx > kHoldSpeed ? -d.max_decel : -d.max_accel;
  const double a = std::max(lo, std::min(hi, u));
  if (!((a >= hi && e > 0.0) || (a <= lo && e < 0.0)))
    driver.trim = std::max(-d.max_decel, std::min(d.max_accel, driver.trim + kTrim * e * h));
  return a;
}

void measured(dyn_driver &driver, double vx0, double vx1, double h) {
  driver.got = (vx1 - vx0) / h;
}

dyn_chassis::~dyn_chassis() { clear(); }

void dyn_chassis::clear() {
  if (_w)
    for (const body_id b : _boxes) {
      try {
        _w->remove_body(b);
      } catch (const std::exception &) {
      }
    }
  _boxes.clear();
  _touched.clear();
  _other.clear();
  _statics.clear();
  _other_depth.clear();
}

void dyn_chassis::reset(world *w, std::vector<body_id> ground, double friction) {
  clear();
  _w = w;
  _ground.clear();
  for (const body_id b : ground)
    _ground.insert(b.value);
  // The boxes' surface: the world's own of this friction if it has one (materials are never
  // removed), else a new one.
  _material = 0;
  if (_w) {
    bool found = false;
    for (std::size_t i = 0; i < _w->material_count() && !found; ++i) {
      const material m = _w->get_material(static_cast<material_id>(i));
      if (m.name == "dyn chassis" && m.friction == static_cast<float>(friction)) {
        _material = static_cast<material_id>(i);
        found = true;
      }
    }
    if (!found)
      _material = _w->add_material({"dyn chassis", static_cast<float>(friction)});
  }
  // A chassis pressed against a wall stays in contact: every step's contacts are needed.
  if (_w && _w->config().contacts != contact_reporting::all) {
    world_config c = _w->config();
    c.contacts = contact_reporting::all;
    _w->reconfigure(c);
  }
}

int dyn_chassis::add(const vehicle_desc &d, const vec3 &position, const quat &rotation) {
  if (!_w)
    return -1;
  body_desc b;
  physics::box chassis_box;
  chassis_box.half_extents = d.half_extents;
  b.shape = chassis_box;
  b.motion = motion_type::dynamic;
  b.position = position;
  b.rotation = rotation;
  b.mass = d.mass;
  b.linear_damping = 0.0;
  b.angular_damping = 0.0;
  b.allow_sleeping = false;
  b.material = _material;
  _boxes.push_back(_w->add_body(b));
  _touched.push_back(0);
  _other.push_back(-1);
  _statics.push_back(0);
  _other_depth.push_back(0.0);
  return static_cast<int>(_boxes.size()) - 1;
}

void dyn_chassis::put(int i, const vec3 &position, const quat &rotation, const vec3 &v,
                      const vec3 &w) {
  body_state s;
  s.position = position;
  s.rotation = rotation;
  s.linear_velocity = v;
  s.angular_velocity = w;
  _w->set_state(_boxes.at(static_cast<std::size_t>(i)), s);
}

void dyn_chassis::step() {
  if (!_w)
    return;
  _w->step();
  std::map<std::uint32_t, std::size_t> of_box;
  for (std::size_t i = 0; i < _boxes.size(); ++i) {
    of_box[_boxes[i].value] = i;
    _touched[i] = 0;
    _other[i] = -1;
    _statics[i] = 0;
    _other_depth[i] = 0.0;
  }
  std::map<std::uint32_t, bool> fixed; // a body without mass, as nav_plant counts static contacts
  const auto is_fixed = [&](body_id b) {
    const auto it = fixed.find(b.value);
    if (it != fixed.end())
      return it->second;
    bool f = false;
    try {
      f = !(_w->get_body(b).mass > 0.0);
    } catch (const std::exception &) {
    }
    fixed[b.value] = f;
    return f;
  };
  for (const contact_event &e : _w->contacts()) {
    if (e.type == contact_event::kind::removed)
      continue;
    const auto a = of_box.find(e.body_a.value), b = of_box.find(e.body_b.value);
    for (const auto &[self, other, them] :
         {std::tuple{a, b, e.body_b}, std::tuple{b, a, e.body_a}}) {
      if (self == of_box.end() || _ground.count(them.value))
        continue;
      const std::size_t i = self->second;
      _touched[i] = 1;
      if (other != of_box.end()) { // another vehicle's box: the deepest
        if (_other[i] < 0 || e.depth > _other_depth[i]) {
          _other[i] = static_cast<int>(other->second);
          _other_depth[i] = e.depth;
        }
      } else if (e.type == contact_event::kind::added && is_fixed(them)) {
        ++_statics[i];
      }
    }
  }
}

body_state dyn_chassis::moved(int i) const {
  return _w->state(_boxes.at(static_cast<std::size_t>(i)));
}

void count_contacts(const dyn_chassis &c, int i, int &statics, int &touched, double &depth) {
  statics += c.static_contacts(i);
  const int o = c.touched_box(i);
  if (o >= 0 && (touched < 0 || c.touched_box_depth(i) > depth)) {
    touched = o;
    depth = c.touched_box_depth(i);
  }
}

void attitude_of(const quat &rotation, cvc::nav::plant_sample &s) {
  const auto clamp1 = [](double v) { return std::max(-1.0, std::min(1.0, v)); };
  s.grade = clamp1(rotate(rotation, {1.0, 0.0, 0.0}).z);
  s.roll = clamp1(rotate(rotation, {0.0, 1.0, 0.0}).z);
  s.up_z = clamp1(rotate(rotation, {0.0, 0.0, 1.0}).z);
}

} // namespace dyn
} // namespace physics
} // namespace cvc
