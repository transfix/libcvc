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
#include <cvc/physics/flow_velocity.h>
#include <cvc/physics/world.h>
#include <cvc/physics/world_geometry.h> // add_world_geometry
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
  _own = std::make_unique<ph::world>(cfg); // throws without a Jolt backend
  _world = _own.get();
  add_avatar_material();
}

avatar_world::avatar_world(const ph::world_config &cfg, const avatar_body &body,
                           const avatar_energy_policy &energy)
    : _body(body), _energy(energy), _dt(cfg.fixed_dt) {
  ph::world_config c = cfg;
  c.backend = ph::backend::jolt;
  _gravity = c.gravity;
  _own = std::make_unique<ph::world>(c); // throws without a Jolt backend
  _world = _own.get();
  add_avatar_material();
}

avatar_world::avatar_world(ph::world &shared, const avatar_body &body,
                           const avatar_energy_policy &energy)
    : _body(body), _energy(energy), _dt(shared.config().fixed_dt), _world(&shared) {
  _gravity = shared.config().gravity;
  add_avatar_material();
}

avatar_world::~avatar_world() = default;

void avatar_world::add_avatar_material() {
  // Frictionless: the drive is the avatar's grip on the floor (see the header), and a box dragged
  // across the floor with friction would tip and stutter.
  ph::material m;
  m.name = "avatar";
  m.friction = 0.0f;
  m.restitution = 0.0f;
  _avatar_material = _world->add_material(m);
}

void avatar_world::release() {
  if (_own)
    return;
  for (const auto &kv : _avatars)
    _world->remove_body(bid(kv.second.id));
  _avatars.clear();
  for (const auto &kv : _obstacles)
    _world->remove_body(bid(kv.second.id));
  _obstacles.clear();
}

void avatar_world::move_to(ph::world &shared, const std::map<std::string, ph::body_state> &feet) {
  if (_own)
    throw std::logic_error("avatar_world::move_to: this world is its own, not a shared one");
  _world = &shared;
  _dt = shared.config().fixed_dt;
  _gravity = shared.config().gravity;
  add_avatar_material();
  for (auto &kv : _avatars) {
    const auto it = feet.find(kv.first);
    kv.second.id = create_avatar(it != feet.end() ? it->second : ph::body_state{},
                                 kv.second.dynamic, kv.second.mass);
  }
  // The old world's bodies may be gone: an obstacle comes back at the origin until held again.
  for (auto &kv : _obstacles)
    kv.second.id = create_obstacle(kv.second.half_extents, ph::body_state{});
}

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
  if (!_own)
    throw std::logic_error("avatar_world::set_statics: a shared world's statics are its owner's");
  // Keep the avatars' states, clear the world, rebuild statics-then-avatars (creation order is
  // part of a deterministic Jolt run).
  std::map<std::string, ph::body_state> kept;
  for (const auto &kv : _avatars) {
    kept[kv.first] = state(kv.first);
    _world->remove_body(bid(kv.second.id));
  }
  std::map<std::uint64_t, ph::body_state> held;
  for (const auto &kv : _obstacles) {
    held[kv.first] = _world->state(bid(kv.second.id));
    _world->remove_body(bid(kv.second.id));
  }
  for (std::uint32_t id : _statics)
    _world->remove_body(bid(id));
  _statics.clear();
  _ground.clear();
  for (const ph::field_id f : _static_fields)
    _world->remove_field(f);
  _static_fields.clear();
  for (const avatar_static &s : statics) {
    if (s.type == avatar_static::kind::geometry) {
      if (!s.geometry)
        continue;
      const ph::world_bodies made = ph::add_world_geometry(*_world, *s.geometry);
      for (const auto &b : made.bodies) {
        _statics.push_back(b.second.value);
        if (s.ground) // its terrain, as SceneWorld marks a node's colliders
          for (const cvc::world::geometry_item &it : s.geometry->items())
            if (it.name == b.first && it.height_field()) {
              _ground.push_back(b.second);
              break;
            }
      }
      for (const auto &f : made.fields)
        _static_fields.push_back(f.second);
      continue;
    }
    ph::body_desc d;
    if (s.type == avatar_static::kind::body) {
      if (!s.body)
        continue;
      d = *s.body; // its shape and pose; its material is another world's id
      d.material = ph::body_desc{}.material;
      if (s.material) {
        // Its surface as its world has it, added to this one once (OW5h).
        const ph::material &m = *s.material;
        const auto same = [&m](const std::pair<ph::material, ph::material_id> &x) {
          return x.first.name == m.name && x.first.friction == m.friction &&
                 x.first.restitution == m.restitution &&
                 x.first.rolling_resistance == m.rolling_resistance &&
                 x.first.nav_material == m.nav_material && x.first.wet_friction == m.wet_friction;
        };
        const auto it = std::find_if(_static_materials.begin(), _static_materials.end(), same);
        if (it != _static_materials.end()) {
          d.material = it->second;
        } else {
          d.material = _world->add_material(m);
          _static_materials.emplace_back(m, d.material);
        }
      }
      d.linear_velocity = d.angular_velocity = ph::vec3{0.0, 0.0, 0.0};
    } else {
      if (s.type == avatar_static::kind::sphere)
        d.shape = ph::sphere{s.radius};
      else
        d.shape = ph::box{s.half_extents};
      d.position = s.center;
      d.rotation = s.rotation;
    }
    d.motion = ph::motion_type::static_;
    const ph::body_id made = _world->add_body(d);
    _statics.push_back(made.value);
    if (s.ground)
      _ground.push_back(made);
  }
  for (auto &kv : _avatars)
    kv.second.id = create_avatar(kept[kv.first], kv.second.dynamic, kv.second.mass);
  for (auto &kv : _obstacles)
    kv.second.id = create_obstacle(kv.second.half_extents, held[kv.first]);
}

void avatar_world::recreate_avatars(const std::map<std::string, ph::body_state> &feet) {
  for (const auto &kv : _avatars)
    _world->remove_body(bid(kv.second.id));
  for (auto &kv : _avatars)
    kv.second.id = create_avatar(feet.at(kv.first), kv.second.dynamic, kv.second.mass);
}

void avatar_world::set_body(const avatar_body &body) {
  if (body == _body)
    return;
  // The feet under the old body (state() measures from its centre), then the new one there.
  std::map<std::string, ph::body_state> feet;
  for (const auto &kv : _avatars)
    feet[kv.first] = state(kv.first);
  _body = body;
  for (auto &kv : _avatars)
    if (!kv.second.own_mass)
      kv.second.mass = body.mass;
  recreate_avatars(feet);
}

void avatar_world::set_gravity(const ph::vec3 &g) {
  if (!_own) {
    _gravity = _world->config().gravity; // the owner's: it prices the walk
    return;
  }
  if (g.x == _gravity.x && g.y == _gravity.y && g.z == _gravity.z)
    return;
  ph::world_config cfg = _world->config();
  cfg.gravity = g;
  _world->reconfigure(cfg);
  _gravity = g;
}

void avatar_world::set_wind(const cvc::algo::wind_state &w, double base_z) {
  if (!_own)
    return; // a shared world's air is its owner's
  ph::field_id id;
  id.value = _wind_field;
  if (!(w.speed_mps > 0.0)) {
    if (id.valid())
      _world->remove_field(id);
    _wind_field = ph::field_id{}.value;
    return;
  }
  // Air everywhere, moving as the wind (a flow with a wind and no volume, field.h).
  ph::field f;
  f.kind = ph::field_kind::flow;
  f.drag_coefficient = 1.0;
  f.wind = std::make_shared<const ph::wind_flow>(w, base_z);
  if (id.valid())
    _world->set_field(id, f);
  else
    _wind_field = _world->add_field(f).value;
}

void avatar_world::set_time(double t) {
  if (_own)
    _world->set_field_time(t);
}

ph::vec3 avatar_wind_drift(const ph::flow_velocity &wind, const avatar_body &body, double force_max,
                           const ph::vec3 &feet, double t) {
  constexpr double kRho = 1.225, kCd = 1.0;
  const ph::vec3 w = wind.velocity(feet + ph::vec3{0.0, 0.0, 0.5 * body.height}, t);
  const double speed = std::sqrt(w.x * w.x + w.y * w.y);
  const double area = body.width * body.height;
  if (!(speed > 0.0) || !(area > 0.0))
    return {};
  // The relative wind at which the drag is force_max: it holds below it, slides to it above.
  const double hold = std::sqrt(2.0 * std::max(force_max, 0.0) / (kRho * kCd * area));
  if (!(speed > hold))
    return {};
  const double k = (speed - hold) / speed;
  return {w.x * k, w.y * k, 0.0};
}

void avatar_world::add_avatar(const std::string &pid, const ph::body_state &feet, bool dynamic) {
  remove_avatar(pid);
  avatar a;
  a.dynamic = dynamic;
  a.mass = _body.mass;
  a.energy.reserve = _energy.reserve_max; // rested
  a.energy.ledger.start = a.energy.reserve;
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
  jump(a, s, in);
  // Upright, facing the intent: a pushed box must not tip over.
  s.rotation = yaw_quat(in.yaw);
  s.angular_velocity = ph::vec3{0.0, 0.0, 0.0};
  _world->set_state(bid(a.id), s);
}

void avatar_world::jump(avatar &a, ph::body_state &s, const avatar_intent &in) const {
  const double vj = _jump_speed;
  // Not while rising, nor while still falling onto the floor (it lands first).
  if (!in.jump || !(vj > 0.0) || s.linear_velocity.z > 0.5 * vj || s.linear_velocity.z < -1.0)
    return;
  // Something under its feet: a ray from a hair below the box (so it misses the avatar's own body)
  // to kJumpReach under it, enough for the solver's resting gap.
  constexpr double kJumpReach = 0.08;
  const double feet = s.position.z - 0.5 * _body.height;
  const ph::vec3 from{s.position.x, s.position.y, feet - 0.002};
  const ph::vec3 to{s.position.x, s.position.y, feet - kJumpReach};
  if (!_world->raycast(from, to))
    return;
  const double vz = std::max(0.0, s.linear_velocity.z);
  if (_energy.enabled) {
    const bool stopped =
        _energy.empty == avatar_energy_policy::when_empty::stop && a.energy.exhausted;
    const double cost = 0.5 * a.mass * (vj * vj - vz * vz);
    if (stopped || cost > a.energy.reserve)
      return;
    a.energy.reserve -= cost;
    a.energy.ledger.work += cost;
    a.energy.power += cost / _dt;
    a.energy.ledger.peak_power = std::max(a.energy.ledger.peak_power, a.energy.power);
    if (a.energy.reserve <= 0.0)
      a.energy.exhausted = true;
  }
  s.linear_velocity.z = vj;
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
  avatar_ledger &book = e.ledger;
  if (!_energy.enabled) {
    capped_change(in.move.x, in.move.y, dvx, dvy);
    s.linear_velocity.x += dvx, s.linear_velocity.y += dvy;
    // No budget: nothing is spent, and the reserve stays full (any change booked as a correction).
    book.corrections += _energy.reserve_max - e.reserve;
    e.reserve = _energy.reserve_max, e.power = 0.0, e.exhausted = false;
    book.distance += std::sqrt(s.linear_velocity.x * s.linear_velocity.x +
                               s.linear_velocity.y * s.linear_velocity.y) *
                     dt;
    book.time += dt;
    return;
  }

  // The body's weight under the world's gravity (what carrying it along is priced by).
  const double g = std::sqrt(ph::dot(_gravity, _gravity));
  const double refill = _energy.power_rest * dt;
  // What this step may spend: the reserve plus its refill, at most power_max's worth; an
  // exhausted `stop` avatar only its refill, and none of it on speeding up.
  const bool stopped = _energy.empty == avatar_energy_policy::when_empty::stop && e.exhausted;
  const double spend = stopped ? refill : std::min(_energy.power_max * dt, e.reserve + refill);
  // The fastest it can afford to carry itself along (no limit without a transport cost).
  const double carry = _energy.transport_cost * m * g;
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
  const double speed = std::sqrt(v1x * v1x + v1y * v1y);
  const double work = std::max(0.0, a2 * k * k + b * k);
  const double carried = carry * speed * dt;
  const double spent = work + carried;
  const double unclamped = e.reserve + refill - spent;
  e.reserve = std::clamp(unclamped, 0.0, _energy.reserve_max);
  e.power = spent / dt;
  book.refilled += refill;
  book.work += work;
  book.carried += carried;
  book.overflow += std::max(0.0, unclamped - _energy.reserve_max);
  book.unpaid += std::max(0.0, -unclamped);
  book.distance += speed * dt;
  book.time += dt;
  book.peak_power = std::max(book.peak_power, e.power);
  if (e.exhausted)
    book.exhausted_time += dt;
  if (e.reserve <= 0.0)
    e.exhausted = true;
  else if (e.exhausted && e.reserve >= _energy.recover_fraction * _energy.reserve_max)
    e.exhausted = false;
}

void avatar_world::set_energy_policy(const avatar_energy_policy &p) {
  _energy = p;
  for (auto &kv : _avatars) {
    avatar_energy &e = kv.second.energy;
    const double before = e.reserve;
    e.reserve = p.enabled ? std::min(e.reserve, p.reserve_max) : p.reserve_max;
    e.ledger.corrections += e.reserve - before; // a new policy's cut or fill
    if (!p.enabled || p.empty != avatar_energy_policy::when_empty::stop)
      e.exhausted = false;
  }
}

avatar_energy avatar_world::energy(const std::string &pid) const { return _avatars.at(pid).energy; }

void avatar_world::set_reserve(const std::string &pid, double joules) {
  avatar &a = _avatars.at(pid);
  if (!a.dynamic || !std::isfinite(joules))
    return;
  const double before = a.energy.reserve;
  a.energy.reserve = std::clamp(joules, 0.0, _energy.reserve_max);
  a.energy.ledger.corrections += a.energy.reserve - before;
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
  a.own_mass = true;
}

void avatar_world::step() {
  if (!_own)
    throw std::logic_error("avatar_world::step: a shared world is stepped by its owner");
  _world->step();
}

std::uint32_t avatar_world::create_obstacle(const ph::vec3 &half_extents,
                                            const ph::body_state &centre) {
  ph::body_desc d;
  d.shape = ph::box{half_extents};
  d.motion = ph::motion_type::kinematic;
  d.position = centre.position;
  d.rotation = centre.rotation;
  d.linear_velocity = centre.linear_velocity;
  d.allow_sleeping = false; // a held obstacle moves every frame
  return _world->add_body(d).value;
}

void avatar_world::hold_obstacle(std::uint64_t key, const ph::vec3 &half_extents,
                                 const ph::pose &centre) {
  auto it = _obstacles.find(key);
  if (it != _obstacles.end() && it->second.half_extents.x == half_extents.x &&
      it->second.half_extents.y == half_extents.y && it->second.half_extents.z == half_extents.z) {
    _world->move_kinematic(bid(it->second.id), centre.position, centre.rotation);
    return;
  }
  if (it != _obstacles.end())
    _world->remove_body(bid(it->second.id));
  ph::body_state at;
  at.position = centre.position;
  at.rotation = centre.rotation;
  obstacle o;
  o.half_extents = half_extents;
  o.id = create_obstacle(half_extents, at);
  _obstacles[key] = o;
}

void avatar_world::remove_obstacle(std::uint64_t key) {
  auto it = _obstacles.find(key);
  if (it == _obstacles.end())
    return;
  _world->remove_body(bid(it->second.id));
  _obstacles.erase(it);
}

bool avatar_world::has_obstacle(std::uint64_t key) const { return _obstacles.count(key) != 0; }

ph::body_state avatar_world::obstacle_state(std::uint64_t key) const {
  return _world->state(bid(_obstacles.at(key).id));
}

ph::body_state avatar_world::state(const std::string &pid) const {
  ph::body_state s = _world->state(bid(_avatars.at(pid).id));
  s.position = s.position - ph::vec3{0.0, 0.0, 0.5 * _body.height};
  return s;
}

} // namespace ariadne
} // namespace cvc
