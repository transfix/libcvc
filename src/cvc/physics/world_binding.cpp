/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <boost/any.hpp>
#include <boost/lexical_cast.hpp>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/exception.h>
#include <cvc/core/world_clock.h>
#include <cvc/physics/world_binding.h>
#include <cvc/physics/world_geometry.h>
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

bool parse_flag(const std::string &s, bool &out) {
  const std::string t = trim_lower(s);
  if (t == "1" || t == "true" || t == "yes" || t == "on") {
    out = true;
    return true;
  }
  if (t == "0" || t == "false" || t == "no" || t == "off") {
    out = false;
    return true;
  }
  return false;
}

void put_vec(cvc::app &ctx, const std::string &base, const vec3 &v) {
  write_val(ctx, at(base, "x"), v.x);
  write_val(ctx, at(base, "y"), v.y);
  write_val(ctx, at(base, "z"), v.z);
}

void put_quat(cvc::app &ctx, const std::string &base, const quat &q) {
  write_val(ctx, at(base, "w"), q.w);
  write_val(ctx, at(base, "x"), q.x);
  write_val(ctx, at(base, "y"), q.y);
  write_val(ctx, at(base, "z"), q.z);
}

const char *motion_name(motion_type m) {
  switch (m) {
  case motion_type::static_:
    return "static";
  case motion_type::kinematic:
    return "kinematic";
  default:
    return "dynamic";
  }
}

const char *kind_name(field_kind k) {
  switch (k) {
  case field_kind::fluid:
    return "fluid";
  case field_kind::medium:
    return "medium";
  default:
    return "flow";
  }
}

void need_name(const std::string &name) {
  if (name.empty() || name.find('.') != std::string::npos)
    throw cvc::physics_error("world_binding: '" + name + "' is not a name (empty, or with a '.')");
}

// The tuning world::set_vehicle accepts, as knobs.
const std::pair<const char *, double vehicle_desc::*> kVehicleKnobs[] = {
    {"max_steer", &vehicle_desc::max_steer},
    {"max_steer_rate", &vehicle_desc::max_steer_rate},
    {"max_accel", &vehicle_desc::max_accel},
    {"max_decel", &vehicle_desc::max_decel},
    {"drag_coefficient", &vehicle_desc::drag_coefficient},
    {"frontal_area", &vehicle_desc::frontal_area},
    {"air_density", &vehicle_desc::air_density},
    {"rolling_resistance", &vehicle_desc::rolling_resistance},
    {"engine_max_torque", &vehicle_desc::engine_max_torque},
    {"engine_min_rpm", &vehicle_desc::engine_min_rpm},
    {"engine_max_rpm", &vehicle_desc::engine_max_rpm},
    {"shift_up_rpm", &vehicle_desc::shift_up_rpm},
    {"shift_down_rpm", &vehicle_desc::shift_down_rpm}};
const std::pair<const char *, double axle_desc::*> kAxleKnobs[] = {
    {"max_brake_torque", &axle_desc::max_brake_torque},
    {"max_hand_brake_torque", &axle_desc::max_hand_brake_torque},
    {"suspension_frequency", &axle_desc::suspension_frequency},
    {"suspension_damping", &axle_desc::suspension_damping},
    {"anti_roll_stiffness", &axle_desc::anti_roll_stiffness}};
const std::pair<const char *, double field::*> kFieldKnobs[] = {
    {"drag_coefficient", &field::drag_coefficient},
    {"damping", &field::damping},
    {"traction", &field::traction}};

const char *const kCommandKeys[] = {"steer", "accel", "reverse"};
const char *const kControlKeys[] = {"steer",      "throttle", "brake",
                                    "hand_brake", "reverse",  "neutral"};

// Equal, or both NaN: a knob that holds "nan" is not a change on every pull.
bool same_value(double a, double b) { return a == b || (std::isnan(a) && std::isnan(b)); }

bool same_tuning(const vehicle_desc &a, const vehicle_desc &b) {
  for (const auto &k : kVehicleKnobs)
    if (!same_value(a.*k.second, b.*k.second))
      return false;
  for (std::size_t i = 0; i < a.axles.size() && i < b.axles.size(); ++i)
    for (const auto &k : kAxleKnobs)
      if (!same_value(a.axles[i].*k.second, b.axles[i].*k.second))
        return false;
  return true;
}

void copy_tuning(const vehicle_desc &from, vehicle_desc &to) {
  for (const auto &k : kVehicleKnobs)
    to.*k.second = from.*k.second;
  for (std::size_t i = 0; i < from.axles.size() && i < to.axles.size(); ++i)
    for (const auto &k : kAxleKnobs)
      to.axles[i].*k.second = from.axles[i].*k.second;
}

// Command keys as a slot first sees them: unset ones are seeded with 0. If the tree held any
// before, the slot has seen none of them, so the first pull applies what the tree holds.
template <size_t N>
std::vector<std::string> first_seen(cvc::app &ctx, const std::string &base,
                                    const char *const (&names)[N]) {
  bool preset = false;
  std::vector<std::string> seen;
  for (const char *n : names) {
    const std::string path = at(base, n);
    std::string raw = read_raw(ctx, path);
    if (raw.empty()) {
      write_val(ctx, path, 0);
      raw = read_raw(ctx, path);
    } else {
      preset = true;
    }
    seen.push_back(raw);
  }
  return preset ? std::vector<std::string>(N) : seen;
}

template <size_t N>
std::vector<std::string> read_keys(cvc::app &ctx, const std::string &base,
                                   const char *const (&names)[N]) {
  std::vector<std::string> now;
  for (const char *n : names)
    now.push_back(read_raw(ctx, at(base, n)));
  return now;
}

template <class T, class Id> const T *find_id(const std::vector<T> &v, Id id) {
  const auto it = std::lower_bound(v.begin(), v.end(), id,
                                   [](const T &x, Id i) { return x.id.value < i.value; });
  return it != v.end() && it->id.value == id.value ? &*it : nullptr;
}

std::string dotless(std::string s) {
  for (char &c : s)
    if (c == '.')
      c = '_';
  return s;
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
  // children() lists every node below, the materials' own keys too: a material is a direct
  // child, <prefix>.materials.<name>.
  const std::string under = key("materials") + ".";
  for (const std::string &full : children) {
    if (full.compare(0, under.size(), under) != 0)
      continue;
    const std::string name = full.substr(under.size());
    if (name.empty() || name.find('.') != std::string::npos)
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
  knob(_ctx, key("snapshot_every"), _snapshot_every);
  pull_materials();
  for (auto &v : _vehicles)
    pull_vehicle(v.first, v.second);
  for (auto &f : _fields)
    pull_field(f.first, f.second);
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
  // The snapshot every snapshot_every steps (a multiple crossed since the last one), and
  // the named actors' keys from the same snapshot, so they agree on a tick.
  const std::uint64_t t = _w.tick();
  const std::uint64_t every = _snapshot_every > 0 ? static_cast<std::uint64_t>(_snapshot_every) : 0;
  const bool due = every > 0 && (!_snapshot || t / every != _snapshot->tick / every);
  if (!due && _bodies.empty() && _vehicles.empty() && _fields.empty())
    return;
  std::shared_ptr<const world_snapshot> snap;
  try {
    snap = std::make_shared<const world_snapshot>(_w.snapshot());
  } catch (const cvc::exception &) {
    return;
  }
  if (due) {
    _snapshot = snap;
    try {
      cvc::state::instance(_ctx)(key("snapshot")).data(boost::any(snap));
    } catch (const std::exception &) {
    }
  }
  push_named(*snap);
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

void world_binding::publish_body(const std::string &name, body_id id) {
  need_name(name);
  if (!_w.contains(id))
    throw cvc::physics_error("world_binding: no body " + std::to_string(id.value) +
                             " to publish as '" + name + "'");
  std::lock_guard<std::mutex> lock(_mutex);
  _bodies[name] = id;
}

void world_binding::publish_vehicle(const std::string &name, vehicle_id id) {
  need_name(name);
  vehicle_slot v;
  v.id = id;
  v.tuned = _w.get_vehicle(id); // throws for an id the world does not hold
  std::lock_guard<std::mutex> lock(_mutex);
  const std::string base = key("vehicles." + name);
  v.command = first_seen(_ctx, at(base, "command"), kCommandKeys);
  v.controls = first_seen(_ctx, at(base, "controls"), kControlKeys);
  pull_vehicle(name, v); // seeds the knobs, and applies what the tree already holds
  _vehicles[name] = std::move(v);
}

void world_binding::publish_field(const std::string &name, field_id id) {
  need_name(name);
  field_slot f;
  f.id = id;
  const field d = _w.get_field(id); // throws for an id the world does not hold
  for (int i = 0; i < 3; ++i)
    f.tuned[i] = d.*kFieldKnobs[i].second;
  std::lock_guard<std::mutex> lock(_mutex);
  pull_field(name, f);
  _fields[name] = f;
}

void world_binding::publish(const world_bodies &added) {
  for (const auto &b : added.bodies)
    if (!b.first.empty())
      publish_body(dotless(b.first), b.second);
  for (const auto &f : added.fields)
    if (!f.first.empty())
      publish_field(dotless(f.first), f.second);
}

void world_binding::unpublish_body(const std::string &name) {
  std::lock_guard<std::mutex> lock(_mutex);
  _bodies.erase(name);
}

void world_binding::unpublish_vehicle(const std::string &name) {
  std::lock_guard<std::mutex> lock(_mutex);
  _vehicles.erase(name);
}

void world_binding::unpublish_field(const std::string &name) {
  std::lock_guard<std::mutex> lock(_mutex);
  _fields.erase(name);
}

std::shared_ptr<const world_snapshot>
world_binding::snapshot_from_state(cvc::app &ctx, const std::string &prefix) {
  try {
    const boost::any d = cvc::state::instance(ctx)(at(prefix, "snapshot")).resolvedData();
    if (const auto *p = boost::any_cast<std::shared_ptr<const world_snapshot>>(&d))
      return *p;
  } catch (const std::exception &) {
  }
  return nullptr;
}

std::shared_ptr<const world_snapshot> world_binding::last_snapshot() const {
  std::lock_guard<std::mutex> lock(_mutex);
  return _snapshot;
}

void world_binding::pull_vehicle(const std::string &name, vehicle_slot &v) {
  const std::string base = key("vehicles." + name);
  // Commands, on a change: the navigator's, then pedals (which win if both changed).
  const std::vector<std::string> command = read_keys(_ctx, at(base, "command"), kCommandKeys);
  if (command != v.command) {
    drive_command c;
    parse_num(command[0], c.steer);
    parse_num(command[1], c.accel);
    parse_flag(command[2], c.reverse);
    try {
      _w.drive(v.id, c);
    } catch (const cvc::exception &) {
    }
    v.command = command;
  }
  const std::vector<std::string> controls = read_keys(_ctx, at(base, "controls"), kControlKeys);
  if (controls != v.controls) {
    vehicle_controls c;
    parse_num(controls[0], c.steer);
    parse_num(controls[1], c.throttle);
    parse_num(controls[2], c.brake);
    parse_num(controls[3], c.hand_brake);
    parse_flag(controls[4], c.reverse);
    parse_flag(controls[5], c.neutral);
    try {
      _w.set_controls(v.id, c);
    } catch (const cvc::exception &) {
    }
    v.controls = controls;
  }
  // Knobs.
  vehicle_desc t = v.tuned;
  const std::string tuning = at(base, "tuning");
  for (const auto &k : kVehicleKnobs)
    knob(_ctx, at(tuning, k.first), t.*k.second);
  for (std::size_t i = 0; i < t.axles.size(); ++i)
    for (const auto &k : kAxleKnobs)
      knob(_ctx, at(tuning, "axles." + std::to_string(i) + "." + k.first), t.axles[i].*k.second);
  if (!same_tuning(t, v.tuned)) {
    try {
      vehicle_desc d = _w.get_vehicle(v.id);
      copy_tuning(t, d);
      _w.set_vehicle(v.id, d);
    } catch (const cvc::exception &) {
      // An invalid value is ignored: the vehicle keeps its last good tuning.
    }
    v.tuned = t;
  }
}

void world_binding::pull_field(const std::string &name, field_slot &f) {
  const std::string base = key("fields." + name);
  double t[3];
  bool changed = false;
  for (int i = 0; i < 3; ++i) {
    t[i] = f.tuned[i];
    knob(_ctx, at(base, kFieldKnobs[i].first), t[i]);
    changed = changed || !same_value(t[i], f.tuned[i]);
  }
  if (!changed)
    return;
  try {
    field d = _w.get_field(f.id);
    for (int i = 0; i < 3; ++i)
      d.*kFieldKnobs[i].second = t[i];
    _w.set_field(f.id, d);
  } catch (const cvc::exception &) {
    // An invalid value is ignored: the field keeps its last good values.
  }
  for (int i = 0; i < 3; ++i)
    f.tuned[i] = t[i];
}

void world_binding::push_named(const world_snapshot &s) {
  for (auto it = _bodies.begin(); it != _bodies.end();) {
    const body_snapshot *b = find_id(s.bodies, it->second);
    if (!b) {
      it = _bodies.erase(it);
      continue;
    }
    const std::string base = key("bodies." + it->first);
    write_val(_ctx, at(base, "id"), b->id.value);
    write_val(_ctx, at(base, "motion"), std::string(motion_name(b->motion)));
    put_vec(_ctx, at(base, "position"), b->state.position);
    put_quat(_ctx, at(base, "rotation"), b->state.rotation);
    put_vec(_ctx, at(base, "velocity"), b->state.linear_velocity);
    put_vec(_ctx, at(base, "angular_velocity"), b->state.angular_velocity);
    write_val(_ctx, at(base, "active"), b->state.active);
    put_vec(_ctx, at(base, "field_force"), b->fields.force);
    write_val(_ctx, at(base, "immersed_volume"), b->fields.immersed_volume);
    ++it;
  }
  for (auto it = _vehicles.begin(); it != _vehicles.end();) {
    const vehicle_snapshot *v = find_id(s.vehicles, it->second.id);
    if (!v) {
      it = _vehicles.erase(it);
      continue;
    }
    const std::string base = key("vehicles." + it->first);
    const vehicle_state &vs = v->state;
    write_val(_ctx, at(base, "chassis"), vs.chassis.value);
    write_val(_ctx, at(base, "speed"), vs.speed);
    write_val(_ctx, at(base, "steer"), vs.steer);
    write_val(_ctx, at(base, "steer_command"), vs.steer_command);
    write_val(_ctx, at(base, "throttle"), vs.throttle);
    write_val(_ctx, at(base, "brake"), vs.brake);
    write_val(_ctx, at(base, "hand_brake"), vs.hand_brake);
    write_val(_ctx, at(base, "gear"), vs.gear);
    write_val(_ctx, at(base, "engine_rpm"), vs.engine_rpm);
    put_vec(_ctx, at(base, "position"), vs.body.position);
    put_quat(_ctx, at(base, "rotation"), vs.body.rotation);
    put_vec(_ctx, at(base, "velocity"), vs.body.linear_velocity);
    put_vec(_ctx, at(base, "angular_velocity"), vs.body.angular_velocity);
    put_vec(_ctx, at(base, "aero_force"), vs.aero_force);
    put_vec(_ctx, at(base, "air_velocity"), vs.air_velocity);
    for (std::size_t i = 0; i < vs.wheels.size(); ++i) {
      const wheel_state &w = vs.wheels[i];
      const std::string wb = at(base, "wheels." + std::to_string(i));
      write_val(_ctx, at(wb, "steer"), w.steer);
      write_val(_ctx, at(wb, "angular_velocity"), w.angular_velocity);
      write_val(_ctx, at(wb, "suspension_length"), w.suspension_length);
      write_val(_ctx, at(wb, "contact"), w.contact);
      put_vec(_ctx, at(wb, "contact_point"), w.contact_point);
      write_val(_ctx, at(wb, "load"), w.load);
      write_val(_ctx, at(wb, "longitudinal_slip"), w.longitudinal_slip);
      write_val(_ctx, at(wb, "lateral_slip"), w.lateral_slip);
      write_val(_ctx, at(wb, "friction"), w.friction);
      write_val(_ctx, at(wb, "rolling_resistance"), w.rolling_resistance);
      write_val(_ctx, at(wb, "medium_depth"), w.medium_depth);
      write_val(_ctx, at(wb, "sinkage"), w.sinkage);
      write_val(_ctx, at(wb, "rut"), w.rut);
      write_val(_ctx, at(wb, "medium_resistance"), w.medium_resistance);
    }
    ++it;
  }
  for (auto it = _fields.begin(); it != _fields.end();) {
    const field_snapshot *f = find_id(s.fields, it->second.id);
    if (!f) {
      it = _fields.erase(it);
      continue;
    }
    const std::string base = key("fields." + it->first);
    write_val(_ctx, at(base, "kind"), std::string(kind_name(f->desc.kind)));
    write_val(_ctx, at(base, "ruts.cells"), static_cast<std::uint64_t>(f->rut_cells));
    write_val(_ctx, at(base, "ruts.deepest"), f->rut_deepest);
    ++it;
  }
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
