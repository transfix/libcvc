/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "dyn_world.h"

#include "det_math.h"

#include <cmath>
#include <exception>
#include <map>

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
}

void dyn_chassis::reset(world *w, std::vector<body_id> ground) {
  clear();
  _w = w;
  _ground.clear();
  for (const body_id b : ground)
    _ground.insert(b.value);
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
  _boxes.push_back(_w->add_body(b));
  _touched.push_back(0);
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
  }
  for (const contact_event &e : _w->contacts()) {
    if (e.type == contact_event::kind::removed)
      continue;
    const auto a = of_box.find(e.body_a.value), b = of_box.find(e.body_b.value);
    if (a != of_box.end() && !_ground.count(e.body_b.value))
      _touched[a->second] = 1;
    if (b != of_box.end() && !_ground.count(e.body_a.value))
      _touched[b->second] = 1;
  }
}

body_state dyn_chassis::moved(int i) const {
  return _w->state(_boxes.at(static_cast<std::size_t>(i)));
}

} // namespace dyn
} // namespace physics
} // namespace cvc
