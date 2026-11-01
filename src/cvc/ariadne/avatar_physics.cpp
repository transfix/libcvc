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

avatar_world::avatar_world(double fixed_dt, const avatar_body &body,
                           const avatar_energy_policy &energy)
    : _body(body), _energy(energy), _dt(fixed_dt) {
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

std::uint32_t avatar_world::create_avatar(const ph::body_state &feet, bool dynamic, double mass) {
  ph::body_desc d;
  d.shape = ph::box{ph::vec3{0.5 * _body.width, 0.5 * _body.width, 0.5 * _body.height}};
  d.motion = dynamic ? ph::motion_type::dynamic : ph::motion_type::kinematic;
  d.position = feet.position + ph::vec3{0.0, 0.0, 0.5 * _body.height};
  d.rotation = feet.rotation;
  d.linear_velocity = feet.linear_velocity;
  d.material = static_cast<ph::material_id>(_avatar_material);
  d.mass = dynamic ? mass : 0.0;
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
    kv.second.id = create_avatar(kept[kv.first], kv.second.dynamic, kv.second.mass);
}

void avatar_world::add_avatar(const std::string &pid, const ph::body_state &feet, bool dynamic) {
  remove_avatar(pid);
  avatar a;
  a.dynamic = dynamic;
  a.mass = _body.mass;
  a.energy.reserve = _energy.reserve_max; // rested
  a.id = create_avatar(feet, dynamic, a.mass);
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
  avatar &a = _avatars.at(pid);
  if (!a.dynamic)
    return;
  ph::body_state s = _world->state(bid(a.id));
  drive_walker(a, s, in); // the only controller so far (§5k)
  // Upright, facing the intent: a pushed box must not tip over.
  s.rotation = yaw_quat(in.yaw);
  s.angular_velocity = ph::vec3{0.0, 0.0, 0.0};
  _world->set_state(bid(a.id), s);
}

void avatar_world::drive_walker(avatar &a, ph::body_state &s, const avatar_intent &in) const {
  // Horizontal only: gravity (and whatever the solver does) keeps the vertical.
  const double m = a.mass, dt = _dt;
  const double v0x = s.linear_velocity.x, v0y = s.linear_velocity.y;
  const auto capped_change = [&](double tx, double ty, double &dx, double &dy) {
    // From v0 toward (TX, TY), pushed with at most force_max: the same push for every mass.
    dx = tx - v0x, dy = ty - v0y;
    const double dv = std::sqrt(dx * dx + dy * dy), cap = _energy.force_max / m * dt;
    if (dv > cap && dv > 0.0)
      dx *= cap / dv, dy *= cap / dv;
  };
  double dvx = 0.0, dvy = 0.0;
  avatar_energy &e = a.energy;
  if (!_energy.enabled) {
    capped_change(in.move.x, in.move.y, dvx, dvy);
    s.linear_velocity.x += dvx, s.linear_velocity.y += dvy;
    e.reserve = _energy.reserve_max, e.power = 0.0, e.exhausted = false;
    return;
  }

  constexpr double kG = 9.81;
  const double refill = _energy.power_rest * dt;
  // What this step may spend: the reserve plus its refill, at most power_max's worth; an
  // exhausted `stop` avatar only its refill, and none of it on speeding up.
  const bool stopped = _energy.empty == avatar_energy_policy::when_empty::stop && e.exhausted;
  const double spend = stopped ? refill : std::min(_energy.power_max * dt, e.reserve + refill);
  // The fastest it can afford to carry itself along (no limit without a transport cost).
  const double carry = _energy.transport_cost * m * kG;
  double tx = in.move.x, ty = in.move.y;
  if (carry > 0.0) {
    const double v_afford = stopped ? 0.0 : spend / (carry * dt);
    const double t = std::sqrt(tx * tx + ty * ty);
    if (t > v_afford && t > 0.0)
      tx *= v_afford / t, ty *= v_afford / t;
  }
  capped_change(tx, ty, dvx, dvy);
  // The work this change does on the body, W(k) = a k^2 + b k for the share k of it applied: what
  // speeding up costs (slowing down is free). Pay for carrying the body first, then apply as much
  // of the change as the rest buys.
  const double a2 = 0.5 * m * (dvx * dvx + dvy * dvy);
  const double b = m * (v0x * dvx + v0y * dvy);
  double v1x = v0x + dvx, v1y = v0y + dvy;
  const double for_work =
      stopped ? 0.0 : std::max(0.0, spend - carry * std::sqrt(v1x * v1x + v1y * v1y) * dt);
  double k = 1.0;
  if (a2 > 0.0 && a2 + b > for_work) {
    const double disc = b * b + 4.0 * a2 * for_work;
    k = std::clamp((-b + std::sqrt(std::max(0.0, disc))) / (2.0 * a2), 0.0, 1.0);
    v1x = v0x + k * dvx, v1y = v0y + k * dvy;
  }
  s.linear_velocity.x = v1x, s.linear_velocity.y = v1y;
  const double work = std::max(0.0, a2 * k * k + b * k);
  const double spent = work + carry * std::sqrt(v1x * v1x + v1y * v1y) * dt;
  e.reserve = std::clamp(e.reserve + refill - spent, 0.0, _energy.reserve_max);
  e.power = spent / dt;
  if (e.reserve <= 0.0)
    e.exhausted = true;
  else if (e.exhausted && e.reserve >= _energy.recover_fraction * _energy.reserve_max)
    e.exhausted = false;
}

void avatar_world::set_energy_policy(const avatar_energy_policy &p) {
  _energy = p;
  for (auto &kv : _avatars) {
    avatar_energy &e = kv.second.energy;
    e.reserve = p.enabled ? std::min(e.reserve, p.reserve_max) : p.reserve_max;
    if (!p.enabled || p.empty != avatar_energy_policy::when_empty::stop)
      e.exhausted = false;
  }
}

avatar_energy avatar_world::energy(const std::string &pid) const { return _avatars.at(pid).energy; }

void avatar_world::set_reserve(const std::string &pid, double joules) {
  avatar &a = _avatars.at(pid);
  if (!a.dynamic || !std::isfinite(joules))
    return;
  a.energy.reserve = std::clamp(joules, 0.0, _energy.reserve_max);
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

void avatar_world::set_mass(const std::string &pid, double kg) {
  avatar &a = _avatars.at(pid);
  if (!a.dynamic)
    return;
  ph::body_settings s = _world->get_body(bid(a.id));
  s.mass = kg;
  _world->set_body(bid(a.id), s);
  a.mass = kg;
}

void avatar_world::step() { _world->step(); }

ph::body_state avatar_world::state(const std::string &pid) const {
  ph::body_state s = _world->state(bid(_avatars.at(pid).id));
  s.position = s.position - ph::vec3{0.0, 0.0, 0.5 * _body.height};
  return s;
}

} // namespace ariadne
} // namespace cvc
