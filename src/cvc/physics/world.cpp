/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "backend_impl.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cvc/core/clock_follower.h>
#include <cvc/core/exception.h>
#include <cvc/core/world_clock.h>
#include <cvc/physics/world.h>
#include <mutex>
#include <string>
#include <tuple>
#include <utility>

namespace cvc {
namespace physics {

// --- backends ---------------------------------------------------------------

detail::world_backend::~world_backend() = default;

const char *backend_name(backend b) {
  switch (b) {
  case backend::null:
    return "null";
  case backend::jolt:
    return "jolt";
  }
  return "unknown";
}

std::vector<backend> available_backends() {
  std::vector<backend> out{backend::null};
#ifdef CVC_WITH_JOLT
  out.push_back(backend::jolt);
#endif
  return out;
}

bool backend_available(backend b) {
  const std::vector<backend> all = available_backends();
  return std::find(all.begin(), all.end(), b) != all.end();
}

std::uint32_t backend_capabilities(backend b) {
  switch (b) {
  case backend::null:
    return detail::null_capabilities();
  case backend::jolt:
#ifdef CVC_WITH_JOLT
    return detail::jolt_capabilities();
#else
    return 0;
#endif
  }
  return 0;
}

std::string backend_version(backend b) {
  switch (b) {
  case backend::null:
    return "null";
  case backend::jolt:
#ifdef CVC_WITH_JOLT
    return detail::jolt_version();
#else
    return "";
#endif
  }
  return "";
}

namespace {

// Backends throw std exceptions; the facade's contract is cvc ones.
template <class F> auto guarded(F &&f) -> decltype(f()) {
  try {
    return f();
  } catch (const cvc::exception &) {
    throw;
  } catch (const std::exception &e) {
    throw cvc::physics_error(e.what());
  }
}

bool finite(const vec3 &v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
bool finite(const quat &q) {
  return std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z);
}
bool unit_length(const quat &q) {
  const double n2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  return std::fabs(n2 - 1.0) < 1e-6;
}

void validate(const world_config &c) {
  if (!(c.fixed_dt > 0.0) || !std::isfinite(c.fixed_dt))
    throw cvc::physics_error("world_config: fixed_dt must be finite and > 0");
  if (c.collision_steps < 1 || c.velocity_steps < 1 || c.position_steps < 0)
    throw cvc::physics_error(
        "world_config: collision_steps and velocity_steps must be >= 1, position_steps >= 0");
  if (!(c.penetration_slop >= 0.0) || !std::isfinite(c.penetration_slop))
    throw cvc::physics_error("world_config: penetration_slop must be finite and >= 0");
  if (c.threads < 0)
    throw cvc::physics_error("world_config: threads must be >= 0");
  if (!finite(c.gravity) || !finite(c.origin))
    throw cvc::physics_error("world_config: gravity and origin must be finite");
  if (c.max_bodies == 0 || c.max_body_pairs == 0 || c.max_contact_constraints == 0 ||
      c.temp_allocator_bytes == 0)
    throw cvc::physics_error("world_config: the engine buffer sizes must be > 0");
}

void validate(const material &m) {
  if (!(m.friction >= 0.0f) || !std::isfinite(m.friction))
    throw cvc::physics_error("material '" + m.name + "': friction must be finite and >= 0");
  if (!(m.restitution >= 0.0f && m.restitution <= 1.0f))
    throw cvc::physics_error("material '" + m.name + "': restitution must be in [0, 1]");
  if (!(m.rolling_resistance >= 0.0f) || !std::isfinite(m.rolling_resistance))
    throw cvc::physics_error("material '" + m.name +
                             "': rolling_resistance must be finite and >= 0");
}

// 64-bit FNV-1a, the digest the Jolt smoke test prints too.
std::uint64_t fnv1a(const void *data, size_t n, std::uint64_t h = 1469598103934665603ull) {
  const unsigned char *p = static_cast<const unsigned char *>(data);
  for (size_t i = 0; i < n; ++i) {
    h ^= p[i];
    h *= 1099511628211ull;
  }
  return h;
}

// save() layout: a fixed header, then the engine's bytes.
constexpr char kMagic[8] = {'C', 'V', 'C', 'P', 'H', 'Y', 'S', '1'};
constexpr std::uint32_t kFormat = 1;

struct state_header {
  char magic[8];
  std::uint32_t format;
  std::uint32_t backend;
  std::uint64_t tick;
  double origin[3];
  std::uint64_t bodies;
  std::uint64_t materials;
  std::uint64_t engine_bytes;
};

} // namespace

// --- world ------------------------------------------------------------------
//
// Thread safety: one mutex guards the whole world. Every public call takes it,
// and step() holds it for the whole engine step, so the backend (and Jolt, whose
// body interface must not be used during an update) only ever sees one caller.
// The impl's own methods expect the lock to be held.

struct world::impl {
  mutable std::mutex mutex;
  world_config cfg;
  std::unique_ptr<detail::world_backend> be;
  std::vector<material> materials;
  std::uint64_t tick = 0;
  std::vector<contact_event> contacts;
  step_report last;
  std::optional<cvc::clock_follower> follower;

  material_id add_material(const material &m) {
    validate(m);
    if (materials.size() >= 65535)
      throw cvc::physics_error("add_material: too many materials");
    const material_id id = static_cast<material_id>(materials.size());
    guarded([&] { be->add_material(id, m); });
    materials.push_back(m);
    return id;
  }

  step_report step() {
    step_report r;
    contacts.clear();
    const auto t0 = std::chrono::steady_clock::now();
    guarded([&] { be->step(contacts, r); });
    const auto t1 = std::chrono::steady_clock::now();
    std::sort(contacts.begin(), contacts.end(), [](const contact_event &a, const contact_event &b) {
      return std::make_tuple(a.body_a.value, a.sub_shape_a, a.body_b.value, a.sub_shape_b,
                             static_cast<int>(a.type)) <
             std::make_tuple(b.body_a.value, b.sub_shape_a, b.body_b.value, b.sub_shape_b,
                             static_cast<int>(b.type));
    });
    ++tick;
    r.tick = tick;
    r.step_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.contacts = contacts.size();
    last = r;
    return r;
  }

  step_report step(std::uint64_t n) {
    step_report r = last;
    for (std::uint64_t i = 0; i < n; ++i)
      r = step();
    return r;
  }

  std::vector<std::uint8_t> save() const {
    const std::string engine = guarded([&] { return be->save_engine(); });
    state_header h{};
    std::memcpy(h.magic, kMagic, sizeof(kMagic));
    h.format = kFormat;
    h.backend = static_cast<std::uint32_t>(cfg.backend);
    h.tick = tick;
    h.origin[0] = cfg.origin.x;
    h.origin[1] = cfg.origin.y;
    h.origin[2] = cfg.origin.z;
    h.bodies = be->body_count();
    h.materials = materials.size();
    h.engine_bytes = engine.size();
    std::vector<std::uint8_t> out(sizeof(h) + engine.size());
    std::memcpy(out.data(), &h, sizeof(h));
    if (!engine.empty())
      std::memcpy(out.data() + sizeof(h), engine.data(), engine.size());
    return out;
  }
};

world::world(const world_config &cfg) : _impl(std::make_unique<impl>()) {
  validate(cfg);
  _impl->cfg = cfg;
  switch (cfg.backend) {
  case physics::backend::null:
    _impl->be = detail::make_null_backend(cfg);
    break;
  case physics::backend::jolt:
#ifdef CVC_WITH_JOLT
    _impl->be = guarded([&] { return detail::make_jolt_backend(cfg); });
    break;
#else
    throw cvc::unsupported_exception("cvc::physics: this libcvc was built without Jolt "
                                     "(CVC_WITH_JOLT); available backends: null");
#endif
  }
  const std::uint32_t caps = backend_capabilities(cfg.backend);
  const std::uint32_t need = cfg.determinism == determinism::cross_platform
                                 ? cap_determinism_cross_platform
                                 : cap_determinism_same_binary;
  if (!(caps & need))
    throw cvc::unsupported_exception(std::string("cvc::physics: the ") + backend_name(cfg.backend) +
                                     " backend in this build cannot deliver the requested "
                                     "determinism mode; it is refused, not downgraded");
  if (cfg.threads > 0 && !(caps & cap_threads))
    throw cvc::unsupported_exception(std::string("cvc::physics: the ") + backend_name(cfg.backend) +
                                     " backend in this build cannot use worker threads "
                                     "(world_config::threads must be 0)");
  _impl->add_material(material{"default"});
}

world::~world() = default;
world::world(world &&) noexcept = default;
world &world::operator=(world &&) noexcept = default;

world_config world::config() const {
  std::lock_guard<std::mutex> lock(_impl->mutex);
  return _impl->cfg;
}

backend world::backend() const {
  std::lock_guard<std::mutex> lock(_impl->mutex);
  return _impl->cfg.backend;
}

std::uint32_t world::capabilities() const { return backend_capabilities(backend()); }

material_id world::add_material(const material &m) {
  std::lock_guard<std::mutex> lock(_impl->mutex);
  return _impl->add_material(m);
}

void world::set_material(material_id id, const material &m) {
  std::lock_guard<std::mutex> lock(_impl->mutex);
  if (id >= _impl->materials.size())
    throw cvc::physics_error("set_material: unknown material id " + std::to_string(id));
  validate(m);
  guarded([&] { _impl->be->set_material(id, m); });
  _impl->materials[id] = m;
}

void world::reconfigure(const world_config &cfg) {
  validate(cfg);
  std::lock_guard<std::mutex> lock(_impl->mutex);
  const world_config &now = _impl->cfg;
  const auto fixed = [](const char *name, bool same) {
    if (!same)
      throw cvc::physics_error(std::string("reconfigure: ") + name +
                               " is fixed when the world is built");
  };
  fixed("backend", cfg.backend == now.backend);
  fixed("fixed_dt", cfg.fixed_dt == now.fixed_dt);
  fixed("threads", cfg.threads == now.threads);
  fixed("determinism", cfg.determinism == now.determinism);
  fixed("max_bodies", cfg.max_bodies == now.max_bodies);
  fixed("max_body_pairs", cfg.max_body_pairs == now.max_body_pairs);
  fixed("max_contact_constraints", cfg.max_contact_constraints == now.max_contact_constraints);
  fixed("temp_allocator_bytes", cfg.temp_allocator_bytes == now.temp_allocator_bytes);
  fixed("origin (use rebase)", cfg.origin.x == now.origin.x && cfg.origin.y == now.origin.y &&
                                   cfg.origin.z == now.origin.z);
  guarded([&] { _impl->be->reconfigure(cfg); });
  _impl->cfg = cfg;
}

material world::get_material(material_id id) const {
  std::lock_guard<std::mutex> lock(_impl->mutex);
  if (id >= _impl->materials.size())
    throw cvc::physics_error("get_material: unknown material id " + std::to_string(id));
  return _impl->materials[id];
}

std::size_t world::material_count() const {
  std::lock_guard<std::mutex> lock(_impl->mutex);
  return _impl->materials.size();
}

namespace {

struct shape_check {
  const std::vector<material> &materials;
  const body_desc &d;

  void need_material(material_id id, const char *what) const {
    if (id >= materials.size())
      throw cvc::physics_error(std::string(what) + ": unknown material id " + std::to_string(id));
  }
  void positive(double v, const char *what) const {
    if (!(v > 0.0) || !std::isfinite(v))
      throw cvc::physics_error(std::string(what) + " must be finite and > 0");
  }

  void operator()(const sphere &s) const { positive(s.radius, "sphere radius"); }
  void operator()(const box &b) const {
    positive(b.half_extents.x, "box half extent");
    positive(b.half_extents.y, "box half extent");
    positive(b.half_extents.z, "box half extent");
  }
  void operator()(const convex_hull &h) const {
    if (h.points.size() < 4)
      throw cvc::physics_error("convex_hull needs at least 4 points");
    for (const vec3 &p : h.points)
      if (!finite(p))
        throw cvc::physics_error("convex_hull: a point is not finite");
  }
  void operator()(const triangle_mesh &m) const {
    if (d.motion != motion_type::static_)
      throw cvc::physics_error("triangle_mesh bodies must be static");
    if (m.indices.empty() || m.indices.size() % 3 != 0)
      throw cvc::physics_error("triangle_mesh: indices must be a non-empty multiple of 3");
    for (std::uint32_t i : m.indices)
      if (i >= m.vertices.size())
        throw cvc::physics_error("triangle_mesh: an index is out of range");
    for (const vec3 &v : m.vertices)
      if (!finite(v))
        throw cvc::physics_error("triangle_mesh: a vertex is not finite");
    if (!m.triangle_materials.empty() && m.triangle_materials.size() != m.indices.size() / 3)
      throw cvc::physics_error(
          "triangle_mesh: triangle_materials must have one entry per triangle");
    for (material_id id : m.triangle_materials)
      need_material(id, "triangle_mesh");
  }
  void operator()(const heightfield &h) const {
    if (d.motion != motion_type::static_)
      throw cvc::physics_error("heightfield bodies must be static");
    if (h.rows < 2 || h.cols < 2)
      throw cvc::physics_error("heightfield: needs at least 2 rows and 2 columns");
    positive(h.dx, "heightfield dx");
    positive(h.dy, "heightfield dy");
    positive(h.max_error, "heightfield max_error");
    if (!finite(h.origin))
      throw cvc::physics_error("heightfield: origin must be finite");
    const size_t n = static_cast<size_t>(h.rows) * h.cols;
    if (h.heights.size() != n)
      throw cvc::physics_error("heightfield: heights must hold rows * cols samples");
    for (float z : h.heights)
      if (std::isinf(z))
        throw cvc::physics_error("heightfield: a height is infinite (use NaN for a hole)");
    const size_t cells = static_cast<size_t>(h.rows - 1) * (h.cols - 1);
    if (!h.cell_materials.empty() && h.cell_materials.size() != cells)
      throw cvc::physics_error("heightfield: cell_materials must hold (rows-1) * (cols-1) entries");
    if (h.materials.size() > 256)
      throw cvc::physics_error("heightfield: at most 256 palette materials");
    const size_t palette = h.materials.empty() ? 1 : h.materials.size();
    for (std::uint8_t c : h.cell_materials)
      if (c >= palette)
        throw cvc::physics_error("heightfield: a cell material is outside the palette");
    for (material_id id : h.materials)
      need_material(id, "heightfield");
  }
};

} // namespace

body_id world::add_body(const body_desc &d) {
  if (d.motion > motion_type::dynamic)
    throw cvc::physics_error("add_body: bad motion type");
  if (!finite(d.position) || !finite(d.rotation) || !finite(d.linear_velocity) ||
      !finite(d.angular_velocity))
    throw cvc::physics_error("add_body: position, rotation and velocities must be finite");
  if (!unit_length(d.rotation))
    throw cvc::physics_error("add_body: rotation must be a unit quaternion");
  if (!(d.mass >= 0.0) || !std::isfinite(d.mass) || !(d.density > 0.0) || !std::isfinite(d.density))
    throw cvc::physics_error("add_body: mass must be >= 0 and density > 0");
  if (!(d.linear_damping >= 0.0) || !(d.angular_damping >= 0.0))
    throw cvc::physics_error("add_body: damping must be >= 0");
  std::lock_guard<std::mutex> lock(_impl->mutex);
  if (d.material >= _impl->materials.size())
    throw cvc::physics_error("add_body: unknown material id " + std::to_string(d.material));
  std::visit(shape_check{_impl->materials, d}, d.shape);
  return guarded([&] { return _impl->be->add_body(d); });
}

void world::remove_body(body_id id) {
  std::lock_guard<std::mutex> lock(_impl->mutex);
  guarded([&] { _impl->be->remove_body(id); });
}

bool world::contains(body_id id) const {
  std::lock_guard<std::mutex> lock(_impl->mutex);
  return _impl->be->contains(id);
}

std::size_t world::body_count() const {
  std::lock_guard<std::mutex> lock(_impl->mutex);
  return _impl->be->body_count();
}

body_state world::state(body_id id) const {
  std::lock_guard<std::mutex> lock(_impl->mutex);
  return guarded([&] { return _impl->be->state(id); });
}

void world::set_state(body_id id, const body_state &s) {
  if (!finite(s.position) || !finite(s.rotation) || !finite(s.linear_velocity) ||
      !finite(s.angular_velocity) || !unit_length(s.rotation))
    throw cvc::physics_error("set_state: finite values and a unit rotation are required");
  std::lock_guard<std::mutex> lock(_impl->mutex);
  guarded([&] { _impl->be->set_state(id, s); });
}

void world::add_force(body_id id, const vec3 &f) {
  if (!finite(f))
    throw cvc::physics_error("add_force: the force must be finite");
  std::lock_guard<std::mutex> lock(_impl->mutex);
  guarded([&] { _impl->be->add_force(id, f); });
}

void world::add_impulse(body_id id, const vec3 &j) {
  if (!finite(j))
    throw cvc::physics_error("add_impulse: the impulse must be finite");
  std::lock_guard<std::mutex> lock(_impl->mutex);
  guarded([&] { _impl->be->add_impulse(id, j); });
}

void world::move_kinematic(body_id id, const vec3 &p, const quat &q) {
  if (!finite(p) || !finite(q) || !unit_length(q))
    throw cvc::physics_error("move_kinematic: a finite position and a unit rotation are required");
  std::lock_guard<std::mutex> lock(_impl->mutex);
  guarded([&] { _impl->be->move_kinematic(id, p, q); });
}

step_report world::step() {
  std::lock_guard<std::mutex> lock(_impl->mutex);
  return _impl->step();
}

step_report world::step(std::uint64_t n) {
  std::lock_guard<std::mutex> lock(_impl->mutex);
  return _impl->step(n);
}

std::uint64_t world::follow(const cvc::world_clock &clock) {
  std::lock_guard<std::mutex> lock(_impl->mutex);
  if (!_impl->follower) {
    _impl->follower = cvc::clock_follower::for_step(clock, _impl->cfg.fixed_dt);
    return 0;
  }
  const std::uint64_t n = _impl->follower->take(clock);
  _impl->step(n);
  return n;
}

std::uint64_t world::tick() const {
  std::lock_guard<std::mutex> lock(_impl->mutex);
  return _impl->tick;
}

double world::time() const {
  std::lock_guard<std::mutex> lock(_impl->mutex);
  return static_cast<double>(_impl->tick) * _impl->cfg.fixed_dt;
}

std::vector<contact_event> world::contacts() const {
  std::lock_guard<std::mutex> lock(_impl->mutex);
  return _impl->contacts;
}

step_report world::last_report() const {
  std::lock_guard<std::mutex> lock(_impl->mutex);
  return _impl->last;
}

std::optional<ray_hit> world::raycast(const vec3 &from, const vec3 &to) const {
  if (!finite(from) || !finite(to))
    throw cvc::physics_error("raycast: the segment must be finite");
  std::lock_guard<std::mutex> lock(_impl->mutex);
  return guarded([&] { return _impl->be->raycast(from, to); });
}

std::vector<std::uint8_t> world::save() const {
  std::lock_guard<std::mutex> lock(_impl->mutex);
  return _impl->save();
}

void world::restore(const std::vector<std::uint8_t> &state) {
  state_header h{};
  if (state.size() < sizeof(h))
    throw cvc::physics_error("restore: the state is truncated");
  std::memcpy(&h, state.data(), sizeof(h));
  if (std::memcmp(h.magic, kMagic, sizeof(kMagic)) != 0 || h.format != kFormat)
    throw cvc::physics_error("restore: not a cvc::physics state, or an unknown format");
  if (h.engine_bytes != state.size() - sizeof(h))
    throw cvc::physics_error("restore: the state's length does not match its header");
  std::lock_guard<std::mutex> lock(_impl->mutex);
  if (h.backend != static_cast<std::uint32_t>(_impl->cfg.backend))
    throw cvc::physics_error("restore: the state was saved by a different backend");
  if (h.bodies != _impl->be->body_count() || h.materials != _impl->materials.size())
    throw cvc::physics_error("restore: this world does not hold the bodies and materials the "
                             "state was saved with");
  const std::string engine(reinterpret_cast<const char *>(state.data()) + sizeof(h),
                           static_cast<size_t>(h.engine_bytes));
  const vec3 origin{h.origin[0], h.origin[1], h.origin[2]};
  guarded([&] { _impl->be->restore_engine(engine, origin); });
  _impl->cfg.origin = origin;
  _impl->tick = h.tick;
  _impl->contacts.clear();
}

std::uint64_t world::state_hash() const {
  std::lock_guard<std::mutex> lock(_impl->mutex);
  const std::vector<std::uint8_t> s = _impl->save();
  return fnv1a(s.data(), s.size());
}

vec3 world::origin() const {
  std::lock_guard<std::mutex> lock(_impl->mutex);
  return _impl->cfg.origin;
}

void world::rebase(const vec3 &new_origin) {
  if (!finite(new_origin))
    throw cvc::physics_error("rebase: the origin must be finite");
  std::lock_guard<std::mutex> lock(_impl->mutex);
  guarded([&] { _impl->be->rebase(new_origin); });
  _impl->cfg.origin = new_origin;
}

} // namespace physics
} // namespace cvc
