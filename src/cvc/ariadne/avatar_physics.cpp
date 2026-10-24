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

#include <cmath>
#include <cvc/ariadne/avatar_physics.h>
#include <cvc/physics/world.h>
#include <stdexcept>

namespace cvc {
namespace ariadne {

namespace {

namespace ph = cvc::physics;

ph::body_id bid(std::uint32_t v) {
  ph::body_id b;
  b.value = v;
  return b;
}

ph::quat yaw_quat(double yaw) {
  return ph::quat{std::cos(0.5 * yaw), 0.0, 0.0, std::sin(0.5 * yaw)};
}

// The yaw of a rotation about +Z (the only rotation an avatar is ever given).
double quat_yaw(const ph::quat &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

} // namespace

avatar_world::avatar_world(double fixed_dt, const avatar_body &body) : _body(body), _dt(fixed_dt) {
  ph::world_config cfg;
  cfg.backend = ph::backend::jolt;
  cfg.fixed_dt = fixed_dt;
  cfg.determinism = ph::determinism::cross_platform;
  cfg.contacts = ph::contact_reporting::none;
  cfg.collision_steps = 1;
  cfg.max_bodies = 4096;
  cfg.max_body_pairs = 8192;
  cfg.max_contact_constraints = 8192;
  cfg.temp_allocator_bytes = 4u << 20;
  _world = std::make_unique<ph::world>(cfg); // throws without a Jolt backend
  // Frictionless: the drive is the avatar's grip on the floor (see the header), and a box dragged
  // across the floor with friction would tip and stutter.
  ph::material m;
  m.name = "avatar";
  m.friction = 0.0f;
  m.restitution = 0.0f;
  _avatar_material = _world->add_material(m);
}

avatar_world::~avatar_world() = default;

std::uint32_t avatar_world::create_avatar(const ph::body_state &feet, bool dynamic) {
  ph::body_desc d;
  d.shape = ph::box{ph::vec3{0.5 * _body.width, 0.5 * _body.width, 0.5 * _body.height}};
  d.motion = dynamic ? ph::motion_type::dynamic : ph::motion_type::kinematic;
  d.position = feet.position + ph::vec3{0.0, 0.0, 0.5 * _body.height};
  d.rotation = feet.rotation;
  d.linear_velocity = feet.linear_velocity;
  d.material = static_cast<ph::material_id>(_avatar_material);
  d.mass = dynamic ? _body.mass : 0.0;
  d.linear_damping = 0.0;
  d.angular_damping = 0.0;
  d.allow_sleeping = false; // an idle avatar must still be pushable and respond to its intent
  return _world->add_body(d).value;
}

void avatar_world::set_statics(const std::vector<avatar_static> &statics) {
  // Keep the avatars' states, clear the world, rebuild statics-then-avatars (creation order is
  // part of a deterministic Jolt run).
  std::map<std::string, ph::body_state> kept;
  for (const auto &kv : _avatars) {
    kept[kv.first] = state(kv.first);
    _world->remove_body(bid(kv.second.id));
  }
  for (std::uint32_t id : _statics)
    _world->remove_body(bid(id));
  _statics.clear();
  for (const avatar_static &s : statics) {
    ph::body_desc d;
    if (s.type == avatar_static::kind::sphere)
      d.shape = ph::sphere{s.radius};
    else
      d.shape = ph::box{s.half_extents};
    d.motion = ph::motion_type::static_;
    d.position = s.center;
    d.rotation = s.rotation;
    _statics.push_back(_world->add_body(d).value);
  }
  for (auto &kv : _avatars)
    kv.second.id = create_avatar(kept[kv.first], kv.second.dynamic);
}

void avatar_world::add_avatar(const std::string &pid, const ph::body_state &feet, bool dynamic) {
  remove_avatar(pid);
  avatar a;
  a.dynamic = dynamic;
  a.id = create_avatar(feet, dynamic);
  _avatars[pid] = a;
}

void avatar_world::remove_avatar(const std::string &pid) {
  auto it = _avatars.find(pid);
  if (it == _avatars.end())
    return;
  _world->remove_body(bid(it->second.id));
  _avatars.erase(it);
}

bool avatar_world::has_avatar(const std::string &pid) const { return _avatars.count(pid) != 0; }

void avatar_world::drive(const std::string &pid, const avatar_intent &in) {
  const avatar &a = _avatars.at(pid);
  if (!a.dynamic)
    return;
  ph::body_state s = _world->state(bid(a.id));
  // Ease the horizontal velocity toward the intent's, at most max_accel * dt per quantum; gravity
  // (and whatever the solver does) keeps the vertical.
  const double dvx = in.move.x - s.linear_velocity.x, dvy = in.move.y - s.linear_velocity.y;
  const double dv = std::sqrt(dvx * dvx + dvy * dvy);
  const double cap = _body.max_accel * _dt;
  const double k = (dv > cap && dv > 0.0) ? cap / dv : 1.0;
  s.linear_velocity.x += dvx * k;
  s.linear_velocity.y += dvy * k;
  // Upright, facing the intent: a pushed box must not tip over.
  s.rotation = yaw_quat(in.yaw);
  s.angular_velocity = ph::vec3{0.0, 0.0, 0.0};
  _world->set_state(bid(a.id), s);
}

void avatar_world::place(const std::string &pid, const ph::body_state &feet) {
  const avatar &a = _avatars.at(pid);
  ph::body_state s = feet;
  s.position = feet.position + ph::vec3{0.0, 0.0, 0.5 * _body.height};
  // Only a facing about +Z survives (a replicated pose is never tilted, but a tilted one must not
  // stand the box on its edge).
  s.rotation = yaw_quat(quat_yaw(feet.rotation));
  s.angular_velocity = ph::vec3{0.0, 0.0, 0.0};
  _world->set_state(bid(a.id), s);
}

void avatar_world::move_kinematic(const std::string &pid, const ph::pose &feet) {
  const avatar &a = _avatars.at(pid);
  if (a.dynamic)
    return;
  _world->move_kinematic(bid(a.id), feet.position + ph::vec3{0.0, 0.0, 0.5 * _body.height},
                         feet.rotation);
}

void avatar_world::step() { _world->step(); }

ph::body_state avatar_world::state(const std::string &pid) const {
  ph::body_state s = _world->state(bid(_avatars.at(pid).id));
  s.position = s.position - ph::vec3{0.0, 0.0, 0.5 * _body.height};
  return s;
}

} // namespace ariadne
} // namespace cvc
