/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The null backend: bodies, ticks and save/restore with no physics. Static and
// dynamic bodies stay where they are put; kinematic bodies move with their
// velocity (and move_kinematic), so a scripted scene still animates. It exists so
// the facade, and code built on it, runs in a build without an engine.

#include "backend_impl.h"

#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace cvc {
namespace physics {
namespace detail {

namespace {

constexpr std::uint32_t kIndexBits = 24;
constexpr std::uint32_t kIndexMask = (1u << kIndexBits) - 1;
constexpr std::uint32_t kMaxIndex = kIndexMask - 1; // keep 0xffffffff (invalid) unreachable

struct slot {
  bool used = false;
  std::uint8_t generation = 0;
  motion_type motion = motion_type::static_;
  body_state s;
  bool has_target = false;
  vec3 target_p;
  quat target_q;
};

quat normalized(quat q) {
  const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (!(n > 0.0))
    return quat{};
  return {q.w / n, q.x / n, q.y / n, q.z / n};
}

// q' = q + dt/2 * (0, w) * q, renormalized: first-order integration of a rotation
// at angular velocity w.
quat integrate(const quat &q, const vec3 &w, double dt) {
  const double h = 0.5 * dt;
  quat d;
  d.w = -h * (w.x * q.x + w.y * q.y + w.z * q.z);
  d.x = h * (w.x * q.w + w.y * q.z - w.z * q.y);
  d.y = h * (w.y * q.w + w.z * q.x - w.x * q.z);
  d.z = h * (w.z * q.w + w.x * q.y - w.y * q.x);
  return normalized({q.w + d.w, q.x + d.x, q.y + d.y, q.z + d.z});
}

// The angular velocity that turns `from` into `to` in dt seconds.
vec3 angular_velocity_between(const quat &from, const quat &to, double dt) {
  // r = to * conj(from), taken the short way round.
  quat r;
  r.w = to.w * from.w + to.x * from.x + to.y * from.y + to.z * from.z;
  r.x = -to.w * from.x + to.x * from.w - to.y * from.z + to.z * from.y;
  r.y = -to.w * from.y + to.x * from.z + to.y * from.w - to.z * from.x;
  r.z = -to.w * from.z - to.x * from.y + to.y * from.x + to.z * from.w;
  if (r.w < 0.0)
    r = {-r.w, -r.x, -r.y, -r.z};
  const double s = std::sqrt(r.x * r.x + r.y * r.y + r.z * r.z);
  if (s < 1e-15)
    return {};
  const double angle = 2.0 * std::atan2(s, r.w);
  const double k = angle / (s * dt);
  return {r.x * k, r.y * k, r.z * k};
}

template <class T> void put(std::string &out, const T &v) {
  out.append(reinterpret_cast<const char *>(&v), sizeof(T));
}

template <class T> void get(const std::string &in, size_t &pos, T &v) {
  if (pos + sizeof(T) > in.size())
    throw std::invalid_argument("null backend: truncated state");
  std::memcpy(&v, in.data() + pos, sizeof(T));
  pos += sizeof(T);
}

class null_backend final : public world_backend {
public:
  explicit null_backend(const world_config &cfg) : _dt(cfg.fixed_dt) {}

  void add_material(material_id, const material &) override {}
  void set_material(material_id, const material &) override {}
  void reconfigure(const world_config &) override {}

  body_id add_body(const body_desc &d, const engine_shape &) override {
    size_t index = _slots.size();
    for (size_t i = 0; i < _slots.size(); ++i)
      if (!_slots[i].used) {
        index = i;
        break;
      }
    if (index > kMaxIndex)
      throw std::length_error("null backend: too many bodies");
    if (index == _slots.size())
      _slots.emplace_back();
    slot &s = _slots[index];
    s.used = true;
    s.motion = d.motion;
    s.s = body_state{};
    s.s.position = d.position;
    s.s.rotation = normalized(d.rotation);
    if (d.motion != motion_type::static_) {
      s.s.linear_velocity = d.linear_velocity;
      s.s.angular_velocity = d.angular_velocity;
    }
    s.has_target = false;
    ++_count;
    body_id id;
    id.value = (static_cast<std::uint32_t>(s.generation) << kIndexBits) |
               static_cast<std::uint32_t>(index);
    return id;
  }

  void remove_body(body_id id) override {
    slot &s = at(id);
    s.used = false;
    s.generation = static_cast<std::uint8_t>(s.generation + 1);
    --_count;
  }

  bool contains(body_id id) const override { return find(id) != nullptr; }
  std::size_t body_count() const override { return _count; }

  body_state state(body_id id) const override {
    const slot &s = at(id);
    body_state out = s.s;
    out.active = active(s);
    return out;
  }

  void set_state(body_id id, const body_state &st) override {
    slot &s = at(id);
    s.s = st;
    s.s.rotation = normalized(st.rotation);
    if (s.motion == motion_type::static_) {
      s.s.linear_velocity = vec3{};
      s.s.angular_velocity = vec3{};
    }
    s.has_target = false;
  }

  void add_force(body_id id, const vec3 &) override { at(id); }
  void add_impulse(body_id id, const vec3 &) override { at(id); }

  void move_kinematic(body_id id, const vec3 &p, const quat &q) override {
    slot &s = at(id);
    if (s.motion != motion_type::kinematic)
      throw std::invalid_argument("move_kinematic: the body is not kinematic");
    s.has_target = true;
    s.target_p = p;
    s.target_q = normalized(q);
  }

  void step(std::vector<contact_event> &, step_report &report) override {
    std::uint32_t n_active = 0;
    for (slot &s : _slots) {
      if (!s.used || s.motion != motion_type::kinematic)
        continue;
      if (s.has_target) {
        s.s.linear_velocity = (s.target_p - s.s.position) * (1.0 / _dt);
        s.s.angular_velocity = angular_velocity_between(s.s.rotation, s.target_q, _dt);
        s.s.position = s.target_p;
        s.s.rotation = s.target_q;
        s.has_target = false;
      } else {
        s.s.position = s.s.position + s.s.linear_velocity * _dt;
        s.s.rotation = integrate(s.s.rotation, s.s.angular_velocity, _dt);
      }
      if (active(s))
        ++n_active;
    }
    report.active_bodies = n_active;
  }

  std::optional<ray_hit> raycast(const vec3 &, const vec3 &) const override { return std::nullopt; }

  std::string save_engine() const override {
    std::string out;
    put(out, static_cast<std::uint64_t>(_slots.size()));
    for (const slot &s : _slots) {
      put(out, static_cast<std::uint8_t>(s.used));
      put(out, s.generation);
      put(out, static_cast<std::uint8_t>(s.motion));
      put(out, s.s.position);
      put(out, s.s.rotation);
      put(out, s.s.linear_velocity);
      put(out, s.s.angular_velocity);
      put(out, static_cast<std::uint8_t>(s.has_target));
      put(out, s.target_p);
      put(out, s.target_q);
    }
    return out;
  }

  void restore_engine(const std::string &bytes, const vec3 &) override {
    size_t pos = 0;
    std::uint64_t n = 0;
    get(bytes, pos, n);
    if (n > kMaxIndex + 1u)
      throw std::invalid_argument("null backend: corrupt state");
    std::vector<slot> slots(static_cast<size_t>(n));
    size_t count = 0;
    for (slot &s : slots) {
      std::uint8_t used = 0, motion = 0, has_target = 0;
      get(bytes, pos, used);
      get(bytes, pos, s.generation);
      get(bytes, pos, motion);
      get(bytes, pos, s.s.position);
      get(bytes, pos, s.s.rotation);
      get(bytes, pos, s.s.linear_velocity);
      get(bytes, pos, s.s.angular_velocity);
      get(bytes, pos, has_target);
      get(bytes, pos, s.target_p);
      get(bytes, pos, s.target_q);
      if (motion > static_cast<std::uint8_t>(motion_type::dynamic))
        throw std::invalid_argument("null backend: corrupt state");
      s.used = used != 0;
      s.motion = static_cast<motion_type>(motion);
      s.has_target = has_target != 0;
      count += s.used ? 1 : 0;
    }
    if (pos != bytes.size())
      throw std::invalid_argument("null backend: trailing bytes in state");
    _slots = std::move(slots);
    _count = count;
  }

  void rebase(const vec3 &) override {} // positions are kept in world coordinates

private:
  static bool active(const slot &s) {
    if (s.motion != motion_type::kinematic)
      return false;
    return s.has_target || dot(s.s.linear_velocity, s.s.linear_velocity) > 0.0 ||
           dot(s.s.angular_velocity, s.s.angular_velocity) > 0.0;
  }

  const slot *find(body_id id) const {
    if (!id.valid())
      return nullptr;
    const std::uint32_t index = id.value & kIndexMask;
    const std::uint32_t gen = id.value >> kIndexBits;
    if (index >= _slots.size())
      return nullptr;
    const slot &s = _slots[index];
    return (s.used && s.generation == gen) ? &s : nullptr;
  }
  const slot &at(body_id id) const {
    const slot *s = find(id);
    if (!s)
      throw std::invalid_argument("unknown or removed body id " + std::to_string(id.value));
    return *s;
  }
  slot &at(body_id id) {
    return const_cast<slot &>(static_cast<const null_backend *>(this)->at(id));
  }

  double _dt;
  std::vector<slot> _slots;
  size_t _count = 0;
};

} // namespace

std::unique_ptr<world_backend> make_null_backend(const world_config &cfg) {
  return std::make_unique<null_backend>(cfg);
}

std::uint32_t null_capabilities() {
  return cap_determinism_same_binary | cap_determinism_cross_platform | cap_exact_save_restore;
}

} // namespace detail
} // namespace physics
} // namespace cvc
