/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "binding_meta.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cvc/core/app.h>
#include <cvc/core/exception.h>
#include <cvc/physics/nav_plant.h>
#include <cvc/state/state.h>
#include <limits>
#include <map>
#include <utility>

namespace cvc {
namespace physics {

namespace {

using dim = cvc::world_units::dimension;
constexpr double kG = 9.80665;

vec3 cross(const vec3 &a, const vec3 &b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

vec3 rotate(const quat &q, const vec3 &v) {
  const vec3 u{q.x, q.y, q.z};
  const vec3 t = cross(u, v) * 2.0;
  return v + t * q.w + cross(u, t);
}

// The rearmost unsteered axle's x (vehicle_desc::wheelbase()'s rear axle); 0 without one.
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

// Just above where the wheels reach with their suspension extended.
double resting_height(const vehicle_desc &d) {
  double h = 0.0;
  for (const axle_desc &a : d.axles)
    h = std::max(h, -a.z + a.suspension_max + a.wheel_radius);
  return h + 0.03;
}

constexpr double kNone = std::numeric_limits<double>::quiet_NaN();

// What nav_plant's keys are (binding_meta.h's form), relative to <prefix>.nav_plant.
const detail::key_meta kPlantKeys[] = {
    {"latency", "knob", "s", 0, 0.5,
     "The vehicle's response lag beyond the tick it holds a command (driver, pedals, engine). The "
     "stopping governor takes the distance it covers off the distance to a wall."},
    {"brake_margin", "knob", "1", 0.3, 1,
     "The share of the vehicle's max_decel the stopping governor plans with."},
    {"drift_nsub", "knob", "", 0, 64,
     "Substeps of the kinematic bicycle the drift is measured against; 0 = the physics steps a "
     "tick (doc 31 D14's nsub = k)."},
    {"settings.step_world", "build", "", kNone, kNone,
     "1: advance() steps the world; 0: the world steps itself on a shared clock."},
    {"settings.ground_z", "build", "m", kNone, kNone,
     "The ground's height under the agents' starts."},
    {"settings.spawn_height", "build", "m", kNone, kNone,
     "How high above the ground a vehicle is added; < 0 = just above its wheels' full reach."},
    {"settings.metres_per_world_unit", "build", "m", kNone, kNone, "Nav's world frame in metres."},
    {"settings.physics_steps", "build", "", kNone, kNone, "Physics steps per nav tick."},
    {"settings.drift_bins_g", "build", "g", kNone, kNone,
     "The drift summary's lateral-acceleration bins: their upper edges."},
    {"vehicles", "telemetry", "", kNone, kNone, "Vehicles the plant drives."},
    {"advance_us", "telemetry", "us", kNone, kNone,
     "advance()'s mean wall time a tick, all vehicles."},
    {"advance_us_per_vehicle", "telemetry", "us", kNone, kNone,
     "advance()'s mean wall time a tick, per vehicle."},
};

// A drift bin's keys, relative to <prefix>.nav_plant.drift.<b>.
const detail::key_meta kBinKeys[] = {
    {"lo_g", "telemetry", "g", kNone, kNone, "The bin's lower edge of lateral acceleration."},
    {"hi_g", "telemetry", "g", kNone, kNone, "The bin's upper edge; -1 for the open last bin."},
    {"ticks", "telemetry", "", kNone, kNone, "Agent ticks in the bin."},
    {"position_rms_m", "telemetry", "m", kNone, kNone,
     "RMS of the tick's position drift: the plant against nav's kinematic bicycle."},
    {"position_max_m", "telemetry", "m", kNone, kNone, "The largest position drift."},
    {"heading_rms_rad", "telemetry", "rad", kNone, kNone, "RMS of the tick's heading drift."},
    {"heading_max_rad", "telemetry", "rad", kNone, kNone, "The largest heading drift."},
    {"speed_rms_mps", "telemetry", "m/s", kNone, kNone, "RMS of the tick's speed drift."},
    {"speed_max_mps", "telemetry", "m/s", kNone, kNone, "The largest speed drift."},
};

// A vehicle's nav keys, relative to <prefix>.vehicles.<name>.nav.
const detail::key_meta kAgentKeys[] = {
    {"steer_command", "telemetry", "rad", kNone, kNone,
     "Nav's steering command this tick (+ left)."},
    {"accel_command", "telemetry", "m/s^2", kNone, kNone,
     "Nav's acceleration command this tick, along the heading."},
    {"drift.position_m", "telemetry", "m", kNone, kNone,
     "This tick: where the vehicle is against where nav's kinematic bicycle puts it."},
    {"drift.heading_rad", "telemetry", "rad", kNone, kNone, "This tick's heading drift."},
    {"drift.speed_mps", "telemetry", "m/s", kNone, kNone, "This tick's speed drift."},
    {"drift.lateral_g", "telemetry", "g", kNone, kNone,
     "The bicycle's lateral acceleration this tick."},
    {"drift.stop_speed_mps", "telemetry", "m/s", kNone, kNone,
     "The stopping speed the tick's command respected (latency-aware v_stop)."},
};

template <std::size_t K>
void describe_keys(cvc::app &ctx, const std::string &base, const detail::key_meta (&t)[K]) {
  for (const detail::key_meta &m : t)
    detail::describe(ctx, base + m.key, m);
}

template <class T> void write_val(cvc::app &ctx, const std::string &path, const T &v) {
  try {
    cvc::state::instance(ctx)(path).value(v);
  } catch (const std::exception &) {
  }
}

} // namespace

nav_plant::nav_plant(world &w, std::vector<vehicle_desc> descs, nav_plant_options options)
    : _w(w), _descs(std::move(descs)), _o(std::move(options)),
      _units(cvc::world_units::config{cvc::world_units::system::si, _o.metres_per_world_unit}) {
  if (_descs.empty())
    throw cvc::physics_error("nav_plant: needs at least one vehicle_desc");
  if (!(_o.metres_per_world_unit > 0.0) || !std::isfinite(_o.metres_per_world_unit))
    throw cvc::physics_error("nav_plant: metres_per_world_unit must be finite and > 0");
  for (std::size_t b = 1; b < _o.drift_bins_g.size(); ++b)
    if (!(_o.drift_bins_g[b] > _o.drift_bins_g[b - 1]))
      throw cvc::physics_error("nav_plant: drift_bins_g must ascend");
  _bins.assign(_o.drift_bins_g.size() + 1, bin_sums{});
}

void nav_plant::attach(int n, const cvc::nav::plant_state *start,
                       const cvc::world_units::normalized_frame &frame, double dt) {
  frame.validate();
  if (n < 0 || (n > 0 && !start))
    throw cvc::physics_error("nav_plant::attach: needs n >= 0 starts");
  if (!(dt > 0.0))
    throw cvc::physics_error("nav_plant::attach: the tick must be > 0");
  const double fixed = _w.config().fixed_dt;
  const double k = dt / fixed;
  _steps = static_cast<int>(std::lround(k));
  if (_o.step_world && (_steps < 1 || std::fabs(k - _steps) > 1e-6))
    throw cvc::physics_error("nav_plant::attach: nav's tick must be a whole number of physics "
                             "steps when the plant steps the world");
  for (const agent &a : _agents)
    _w.remove_vehicle(a.vehicle);
  _agents.clear();
  _frame = frame;
  _dt = dt;
  _agents.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    agent a;
    a.desc = _descs[static_cast<std::size_t>(i) % _descs.size()];
    a.rear_x = rear_axle_x(a.desc);
    // The rear axle on the agent's start, the body along its heading.
    const cvc::world_units::xyz p =
        _units.normalized_point_to_si(start[i].x, start[i].y, 0.0, frame);
    const double yaw = start[i].heading;
    const double c = std::cos(yaw), s = std::sin(yaw);
    const double h = _o.spawn_height >= 0.0 ? _o.spawn_height : resting_height(a.desc);
    const vec3 origin{p.x - c * a.rear_x, p.y - s * a.rear_x, _o.ground_z + h};
    quat q;
    q.w = std::cos(0.5 * yaw);
    q.z = std::sin(0.5 * yaw);
    a.vehicle = _w.add_vehicle(a.desc, origin, q);
    const physics::vehicle_state vs = _w.vehicle_state(a.vehicle);
    a.chassis = vs.chassis;
    a.drawn = drawn(a);
    _agents.push_back(a);
  }
  _ticks = 0;
  _advance_us = 0.0;
  clear_drift();
}

cvc::nav::plant_state nav_plant::state(int i) const {
  const agent &a = _agents.at(static_cast<std::size_t>(i));
  const physics::vehicle_state vs = _w.vehicle_state(a.vehicle);
  const body_state &b = vs.body;
  const vec3 fwd = rotate(b.rotation, {1.0, 0.0, 0.0});
  const vec3 rear = b.position + rotate(b.rotation, {a.rear_x, 0.0, 0.0});
  const vec3 com = b.position + rotate(b.rotation, a.desc.center_of_mass);
  const vec3 v_rear = b.linear_velocity + cross(b.angular_velocity, rear - com);
  cvc::nav::plant_state s;
  const cvc::world_units::xyz n = _units.si_point_to_normalized(rear.x, rear.y, 0.0, _frame);
  s.x = static_cast<float>(n.x);
  s.y = static_cast<float>(n.y);
  s.heading = static_cast<float>(std::atan2(fwd.y, fwd.x));
  s.speed = static_cast<float>(_units.to_normalized(dot(v_rear, fwd), dim::velocity, _frame));
  s.steer = static_cast<float>(vs.steer);
  return s;
}

cvc::nav::plant_limits nav_plant::limits(int i) const {
  const agent &a = _agents.at(static_cast<std::size_t>(i));
  cvc::nav::plant_limits l;
  l.wheelbase = static_cast<float>(_units.to_normalized(a.desc.wheelbase(), dim::length, _frame));
  l.max_steer = static_cast<float>(a.desc.max_steer);
  l.steer_rate = static_cast<float>(a.desc.max_steer_rate);
  l.a_brake = static_cast<float>(
      _units.to_normalized(_o.brake_margin * a.desc.max_decel, dim::acceleration, _frame));
  l.latency = static_cast<float>(_o.latency);
  return l;
}

void nav_plant::command(int i, float steer, float accel) {
  agent &a = _agents.at(static_cast<std::size_t>(i));
  const double a_si = _units.from_normalized(accel, dim::acceleration, _frame);
  const double speed = _units.from_normalized(state(i).speed, dim::velocity, _frame);
  // Forward or backward by where the agent is to be moving at the tick's end; the driver
  // model takes the acceleration along that direction.
  const bool reverse = speed + _dt * a_si < 0.0;
  drive_command c;
  c.steer = steer;
  c.accel = reverse ? -a_si : a_si;
  c.reverse = reverse;
  _w.drive(a.vehicle, c);
  a.steer_command = steer;
  a.accel_command = a_si;
}

void nav_plant::advance() {
  const auto t0 = std::chrono::steady_clock::now();
  for (agent &a : _agents)
    a.contacts_step = 0;
  if (_o.step_world) {
    for (int k = 0; k < _steps; ++k) {
      _w.step();
      count_contacts();
    }
  } else {
    count_contacts(); // the world's last step
  }
  for (agent &a : _agents) {
    const double now = drawn(a);
    a.energy_step = now - a.drawn;
    a.drawn = now;
  }
  _advance_us +=
      std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
  ++_ticks;
}

void nav_plant::count_contacts() {
  if (_agents.empty())
    return;
  std::map<std::uint32_t, std::size_t> of_chassis;
  for (std::size_t i = 0; i < _agents.size(); ++i)
    of_chassis[_agents[i].chassis.value] = i;
  std::map<std::uint32_t, bool> fixed; // a body without mass: static or kinematic geometry
  const auto is_fixed = [&](body_id b) {
    const auto it = fixed.find(b.value);
    if (it != fixed.end())
      return it->second;
    bool f = false;
    try {
      f = !(_w.get_body(b).mass > 0.0);
    } catch (const std::exception &) {
    }
    fixed[b.value] = f;
    return f;
  };
  for (const contact_event &e : _w.contacts()) {
    if (e.type != contact_event::kind::added)
      continue;
    const auto ia = of_chassis.find(e.body_a.value), ib = of_chassis.find(e.body_b.value);
    if (ia != of_chassis.end() && ib == of_chassis.end() && is_fixed(e.body_b))
      ++_agents[ia->second].contacts_step;
    else if (ib != of_chassis.end() && ia == of_chassis.end() && is_fixed(e.body_a))
      ++_agents[ib->second].contacts_step;
  }
}

double nav_plant::drawn(const agent &a) const {
  const vehicle_energy e = _w.vehicle_state(a.vehicle).energy;
  // What the source gave, net of what regeneration returned; the shaft work without a source.
  if (a.desc.energy.type != energy_source::kind::none)
    return e.source - e.regen;
  return e.engine;
}

int nav_plant::reference_substeps() const {
  return _o.drift_nsub > 0 ? _o.drift_nsub : std::max(1, _steps);
}

void nav_plant::set_latency(double s) {
  if (!(std::isfinite(s) && s >= 0.0))
    throw cvc::physics_error("nav_plant: latency must be finite and >= 0");
  _o.latency = s;
}

void nav_plant::set_brake_margin(double share) {
  if (!(share > 0.0 && share <= 1.0))
    throw cvc::physics_error("nav_plant: brake_margin must be in (0, 1]");
  _o.brake_margin = share;
}

void nav_plant::set_drift_nsub(int n) {
  if (n < 0)
    throw cvc::physics_error("nav_plant: drift_nsub must be >= 0");
  _o.drift_nsub = n;
}

void nav_plant::pull(cvc::app &ctx, const std::string &prefix) {
  const std::string k = (prefix.empty() ? std::string() : prefix + ".") + "nav_plant.";
  if (_described.insert(k).second)
    describe_keys(ctx, k, kPlantKeys);
  const auto read = [&](const std::string &name, double now, auto apply) {
    std::string raw;
    try {
      raw = cvc::state::instance(ctx)(k + name).value();
    } catch (const std::exception &) {
    }
    if (raw.empty()) {
      write_val(ctx, k + name, now);
      return;
    }
    try {
      std::size_t used = 0;
      const double v = std::stod(raw, &used);
      apply(v);
    } catch (const std::exception &) {
      // an invalid value is ignored: the plant keeps its last good one
    }
  };
  read("latency", _o.latency, [&](double v) { set_latency(v); });
  read("brake_margin", _o.brake_margin, [&](double v) { set_brake_margin(v); });
  read("drift_nsub", static_cast<double>(_o.drift_nsub), [&](double v) {
    if (v == std::floor(v))
      set_drift_nsub(static_cast<int>(v));
  });
}

double nav_plant::energy_step(int i) const {
  return _agents.at(static_cast<std::size_t>(i)).energy_step;
}

int nav_plant::static_contacts_step(int i) const {
  return _agents.at(static_cast<std::size_t>(i)).contacts_step;
}

void nav_plant::report_drift(int i, const cvc::nav::drift_sample &s) {
  agent &a = _agents.at(static_cast<std::size_t>(i));
  drift_si d;
  d.position_m = _units.from_normalized(s.position, dim::length, _frame);
  d.heading_rad = s.heading;
  d.speed_mps = _units.from_normalized(s.speed, dim::velocity, _frame);
  d.lateral_g = _units.from_normalized(s.a_lateral, dim::acceleration, _frame) / kG;
  d.stop_speed_mps = _units.from_normalized(s.stop_speed, dim::velocity, _frame);
  a.drift = d;
  std::size_t b = 0;
  while (b < _o.drift_bins_g.size() && d.lateral_g >= _o.drift_bins_g[b])
    ++b;
  bin_sums &bs = _bins[b];
  ++bs.ticks;
  bs.pos2 += d.position_m * d.position_m;
  bs.pos_max = std::max(bs.pos_max, d.position_m);
  bs.head2 += d.heading_rad * d.heading_rad;
  bs.head_max = std::max(bs.head_max, std::fabs(d.heading_rad));
  bs.speed2 += d.speed_mps * d.speed_mps;
  bs.speed_max = std::max(bs.speed_max, std::fabs(d.speed_mps));
}

std::vector<nav_plant::drift_bin> nav_plant::drift_summary() const {
  std::vector<drift_bin> out(_bins.size());
  for (std::size_t b = 0; b < _bins.size(); ++b) {
    drift_bin &o = out[b];
    o.lo_g = b == 0 ? 0.0 : _o.drift_bins_g[b - 1];
    o.hi_g =
        b < _o.drift_bins_g.size() ? _o.drift_bins_g[b] : std::numeric_limits<double>::infinity();
    const bin_sums &s = _bins[b];
    o.ticks = s.ticks;
    if (s.ticks > 0) {
      const double n = static_cast<double>(s.ticks);
      o.position_rms_m = std::sqrt(s.pos2 / n);
      o.heading_rms_rad = std::sqrt(s.head2 / n);
      o.speed_rms_mps = std::sqrt(s.speed2 / n);
    }
    o.position_max_m = s.pos_max;
    o.heading_max_rad = s.head_max;
    o.speed_max_mps = s.speed_max;
  }
  return out;
}

void nav_plant::clear_drift() { _bins.assign(_o.drift_bins_g.size() + 1, bin_sums{}); }

double nav_plant::advance_us() const {
  return _ticks > 0 ? _advance_us / static_cast<double>(_ticks) : 0.0;
}

double nav_plant::advance_us_per_vehicle() const {
  return _agents.empty() ? 0.0 : advance_us() / static_cast<double>(_agents.size());
}

void nav_plant::publish(cvc::app &ctx, const std::string &prefix,
                        const std::vector<std::string> &names) const {
  const std::string root = prefix.empty() ? std::string() : prefix + ".";
  for (std::size_t i = 0; i < names.size() && i < _agents.size(); ++i) {
    if (names[i].empty())
      continue;
    const agent &a = _agents[i];
    const std::string k = root + "vehicles." + names[i] + ".nav.";
    if (_described.insert(k).second)
      describe_keys(ctx, k, kAgentKeys);
    write_val(ctx, k + "steer_command", a.steer_command);
    write_val(ctx, k + "accel_command", a.accel_command);
    write_val(ctx, k + "drift.position_m", a.drift.position_m);
    write_val(ctx, k + "drift.heading_rad", a.drift.heading_rad);
    write_val(ctx, k + "drift.speed_mps", a.drift.speed_mps);
    write_val(ctx, k + "drift.lateral_g", a.drift.lateral_g);
    write_val(ctx, k + "drift.stop_speed_mps", a.drift.stop_speed_mps);
  }
  const std::string k = root + "nav_plant.";
  if (_described.insert(k).second)
    describe_keys(ctx, k, kPlantKeys);
  const std::vector<drift_bin> bins = drift_summary();
  for (std::size_t b = 0; b < bins.size(); ++b) {
    const drift_bin &d = bins[b];
    const std::string kb = k + "drift." + std::to_string(b) + ".";
    if (_described.insert(kb).second)
      describe_keys(ctx, kb, kBinKeys);
    write_val(ctx, kb + "lo_g", d.lo_g);
    write_val(ctx, kb + "hi_g", std::isfinite(d.hi_g) ? d.hi_g : -1.0);
    write_val(ctx, kb + "ticks", static_cast<std::int64_t>(d.ticks));
    write_val(ctx, kb + "position_rms_m", d.position_rms_m);
    write_val(ctx, kb + "position_max_m", d.position_max_m);
    write_val(ctx, kb + "heading_rms_rad", d.heading_rms_rad);
    write_val(ctx, kb + "heading_max_rad", d.heading_max_rad);
    write_val(ctx, kb + "speed_rms_mps", d.speed_rms_mps);
    write_val(ctx, kb + "speed_max_mps", d.speed_max_mps);
  }
  const std::string ks = k + "settings.";
  write_val(ctx, ks + "step_world", _o.step_world ? 1 : 0);
  write_val(ctx, ks + "ground_z", _o.ground_z);
  write_val(ctx, ks + "spawn_height", _o.spawn_height);
  write_val(ctx, ks + "metres_per_world_unit", _o.metres_per_world_unit);
  write_val(ctx, ks + "physics_steps", _steps);
  {
    std::string bins;
    for (std::size_t b = 0; b < _o.drift_bins_g.size(); ++b)
      bins += (b ? ", " : "") + std::to_string(_o.drift_bins_g[b]);
    write_val(ctx, ks + "drift_bins_g", bins);
  }
  write_val(ctx, k + "vehicles", static_cast<std::int64_t>(_agents.size()));
  write_val(ctx, k + "advance_us", advance_us());
  write_val(ctx, k + "advance_us_per_vehicle", advance_us_per_vehicle());
}

} // namespace physics
} // namespace cvc
