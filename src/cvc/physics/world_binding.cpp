/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <boost/lexical_cast.hpp>
#include <cctype>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/exception.h>
#include <cvc/core/world_clock.h>
#include <cvc/physics/world_binding.h>
#include <cvc/state/state.h>
#include <utility>
#include <vector>

namespace cvc {
namespace physics {

namespace {

std::string trim_lower(const std::string &s) {
  std::string t;
  for (char c : s)
    if (!std::isspace(static_cast<unsigned char>(c)))
      t += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return t;
}

std::string at(const std::string &prefix, const std::string &name) {
  return prefix.empty() ? name : prefix + "." + name;
}

// Read through a transparent link to its target, as world_clock_binding does, so
// a key mounted from another scope reads its source.
std::string read_raw(cvc::app &ctx, const std::string &path) {
  try {
    cvc::state &s = cvc::state::instance(ctx)(path);
    if (s.isLink() && s.linkMode() == cvc::state::link_mode::transparent) {
      const cvc::state::link_resolution lr = s.resolveLink();
      if (lr.kind == cvc::state::link_resolution_kind::resolved && lr.target)
        return lr.target->value();
    }
    return s.value();
  } catch (const std::exception &) {
    return std::string();
  }
}

// Never throws into a frame: a read-only key keeps its value.
template <class T> void write_val(cvc::app &ctx, const std::string &path, const T &v) {
  try {
    cvc::state::instance(ctx)(path).value(v);
  } catch (const std::exception &) {
  }
}

template <class T> bool parse_num(const std::string &s, T &out) {
  const std::string t = trim_lower(s);
  if (t.empty())
    return false;
  try {
    out = boost::lexical_cast<T>(t);
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

// Seed an unset key, then read it back. The value stays `v` when the key holds
// something unreadable.
template <class T> void knob(cvc::app &ctx, const std::string &path, T &v) {
  const std::string raw = read_raw(ctx, path);
  if (raw.empty()) {
    write_val(ctx, path, v);
    return;
  }
  T parsed;
  if (parse_num(raw, parsed))
    v = parsed;
}

// The same for a named choice: `names` lists (name, value) pairs.
template <class E, size_t N>
void knob_choice(cvc::app &ctx, const std::string &path, E &v,
                 const std::pair<const char *, E> (&names)[N]) {
  const std::string raw = trim_lower(read_raw(ctx, path));
  if (raw.empty()) {
    for (const auto &n : names)
      if (n.second == v) {
        write_val(ctx, path, std::string(n.first));
        return;
      }
    return;
  }
  for (const auto &n : names)
    if (raw == n.first) {
      v = n.second;
      return;
    }
}

const std::pair<const char *, backend> kBackends[] = {{"null", backend::null},
                                                      {"jolt", backend::jolt}};
const std::pair<const char *, determinism> kDeterminism[] = {
    {"same_binary", determinism::same_binary}, {"cross_platform", determinism::cross_platform}};
const std::pair<const char *, combine_rule> kCombine[] = {
    {"product", combine_rule::product},
    {"min", combine_rule::min},
    {"max", combine_rule::max},
    {"geometric_mean", combine_rule::geometric_mean},
    {"average", combine_rule::average}};
const std::pair<const char *, contact_reporting> kContacts[] = {
    {"none", contact_reporting::none},
    {"added_removed", contact_reporting::added_removed},
    {"all", contact_reporting::all}};

// The keys a world is built with.
void read_build_keys(cvc::app &ctx, const std::string &p, world_config &c) {
  knob_choice(ctx, at(p, "backend"), c.backend, kBackends);
  knob(ctx, at(p, "fixed_dt"), c.fixed_dt);
  knob(ctx, at(p, "threads"), c.threads);
  knob_choice(ctx, at(p, "determinism"), c.determinism, kDeterminism);
  knob(ctx, at(p, "max_bodies"), c.max_bodies);
  knob(ctx, at(p, "max_body_pairs"), c.max_body_pairs);
  knob(ctx, at(p, "max_contact_constraints"), c.max_contact_constraints);
  knob(ctx, at(p, "origin.x"), c.origin.x);
  knob(ctx, at(p, "origin.y"), c.origin.y);
  knob(ctx, at(p, "origin.z"), c.origin.z);
}

// The keys every pull() applies.
void read_runtime_keys(cvc::app &ctx, const std::string &p, world_config &c) {
  knob(ctx, at(p, "gravity.x"), c.gravity.x);
  knob(ctx, at(p, "gravity.y"), c.gravity.y);
  knob(ctx, at(p, "gravity.z"), c.gravity.z);
  knob(ctx, at(p, "collision_steps"), c.collision_steps);
  knob(ctx, at(p, "velocity_steps"), c.velocity_steps);
  knob(ctx, at(p, "position_steps"), c.position_steps);
  knob(ctx, at(p, "penetration_slop"), c.penetration_slop);
  knob_choice(ctx, at(p, "friction_combine"), c.friction, kCombine);
  knob_choice(ctx, at(p, "restitution_combine"), c.restitution, kCombine);
  knob_choice(ctx, at(p, "contacts"), c.contacts, kContacts);
}

bool same_build(const world_config &a, const world_config &b) {
  return a.backend == b.backend && a.fixed_dt == b.fixed_dt && a.threads == b.threads &&
         a.determinism == b.determinism && a.max_bodies == b.max_bodies &&
         a.max_body_pairs == b.max_body_pairs &&
         a.max_contact_constraints == b.max_contact_constraints && a.origin.x == b.origin.x &&
         a.origin.y == b.origin.y && a.origin.z == b.origin.z;
}

bool same_material(const material &a, const material &b) {
  return a.friction == b.friction && a.restitution == b.restitution &&
         a.rolling_resistance == b.rolling_resistance && a.nav_material == b.nav_material;
}

} // namespace

world_config world_binding::config_from_state(cvc::app &ctx, const std::string &prefix) {
  world_config c;
  read_build_keys(ctx, prefix, c);
  read_runtime_keys(ctx, prefix, c);
  return c;
}

world_binding::world_binding(cvc::app &ctx, physics::world &w, std::string prefix)
    : _ctx(ctx), _w(w), _prefix(std::move(prefix)) {}

std::string world_binding::key(const std::string &name) const { return at(_prefix, name); }

void world_binding::seed() {
  // Material 0 is the world's own; publish it as "default" so it can be tuned.
  material_slot def;
  def.id = 0;
  def.applied = _w.get_material(0);
  _materials["default"] = def;
  const std::string m = key("materials.default");
  write_val(_ctx, at(m, "friction"), static_cast<double>(def.applied.friction));
  write_val(_ctx, at(m, "restitution"), static_cast<double>(def.applied.restitution));
  write_val(_ctx, at(m, "rolling_resistance"), static_cast<double>(def.applied.rolling_resistance));
  write_val(_ctx, at(m, "nav_material"), def.applied.nav_material);
  _seeded = true;
}

void world_binding::pull_materials() {
  std::vector<std::string> children;
  try {
    children = cvc::state::instance(_ctx)(key("materials")).children();
  } catch (const std::exception &) {
    return;
  }
  for (const std::string &full : children) {
    const size_t dot = full.find_last_of('.');
    const std::string name = dot == std::string::npos ? full : full.substr(dot + 1);
    if (name.empty())
      continue;
    auto it = _materials.find(name);
    physics::material m = it != _materials.end() ? it->second.applied : physics::material{};
    m.name = name;
    double friction = m.friction, restitution = m.restitution, rolling = m.rolling_resistance;
    int nav = m.nav_material;
    const std::string base = key("materials." + name);
    knob(_ctx, at(base, "friction"), friction);
    knob(_ctx, at(base, "restitution"), restitution);
    knob(_ctx, at(base, "rolling_resistance"), rolling);
    knob(_ctx, at(base, "nav_material"), nav);
    m.friction = static_cast<float>(friction);
    m.restitution = static_cast<float>(restitution);
    m.rolling_resistance = static_cast<float>(rolling);
    m.nav_material = nav;
    try {
      if (it == _materials.end()) {
        material_slot slot;
        slot.id = _w.add_material(m);
        slot.applied = m;
        _materials.emplace(name, slot);
      } else if (!same_material(m, it->second.applied)) {
        _w.set_material(it->second.id, m);
        it->second.applied = m;
      }
    } catch (const cvc::exception &) {
      // An invalid material (say, negative friction) is ignored: the world keeps
      // the last good values.
    }
  }
}

void world_binding::pull() {
  std::lock_guard<std::mutex> lock(_mutex);
  pull_locked();
}

void world_binding::pull_locked() {
  if (!_seeded)
    seed();
  world_config build = _w.config();
  read_build_keys(_ctx, _prefix, build);
  _restart_required = !same_build(build, _w.config());

  world_config cfg = _w.config();
  read_runtime_keys(_ctx, _prefix, cfg);
  try {
    _w.reconfigure(cfg);
  } catch (const cvc::exception &) {
    // An invalid runtime value is ignored, never thrown into a frame.
  }
  knob(_ctx, key("publish_every"), _publish_every);
  knob(_ctx, key("hash_every"), _hash_every);
  pull_materials();
}

void world_binding::push() {
  std::lock_guard<std::mutex> lock(_mutex);
  push_locked();
}

void world_binding::push_locked() {
  const step_report r = _w.last_report();
  write_val(_ctx, key("tick"), _w.tick());
  write_val(_ctx, key("time"), _w.time());
  write_val(_ctx, key("step_ms"), r.step_ms);
  write_val(_ctx, key("bodies"), static_cast<std::uint64_t>(_w.body_count()));
  write_val(_ctx, key("active_bodies"), static_cast<std::uint64_t>(r.active_bodies));
  write_val(_ctx, key("contact_events"), static_cast<std::uint64_t>(r.contacts));
  write_val(_ctx, key("saturated"), r.saturated);
  write_val(_ctx, key("version"), backend_version(_w.backend()));
  write_val(_ctx, key("restart_required"), _restart_required);
  if (_hash_every > 0 && _w.tick() % static_cast<std::uint64_t>(_hash_every) == 0) {
    char hex[17];
    std::snprintf(hex, sizeof(hex), "%016llx", static_cast<unsigned long long>(_w.state_hash()));
    write_val(_ctx, key("state_hash"), std::string(hex));
  }
}

step_report world_binding::step() {
  std::lock_guard<std::mutex> lock(_mutex);
  pull_locked();
  const step_report r = _w.step();
  if (_publish_every <= 1 || r.tick % static_cast<std::uint64_t>(_publish_every) == 0)
    push_locked();
  return r;
}

std::uint64_t world_binding::follow(const cvc::world_clock &clock) {
  std::lock_guard<std::mutex> lock(_mutex);
  pull_locked();
  const std::uint64_t n = _w.follow(clock);
  if (n > 0) {
    const std::uint64_t every =
        _publish_every <= 1 ? 1 : static_cast<std::uint64_t>(_publish_every);
    const std::uint64_t t = _w.tick();
    if (t / every > (t - n) / every) // a multiple of publish_every was crossed
      push_locked();
  }
  return n;
}

material_id world_binding::material(const std::string &name) const {
  std::lock_guard<std::mutex> lock(_mutex);
  const auto it = _materials.find(name);
  if (it == _materials.end())
    throw cvc::physics_error("world_binding: no material '" + name + "' under " + key("materials"));
  return it->second.id;
}

} // namespace physics
} // namespace cvc
