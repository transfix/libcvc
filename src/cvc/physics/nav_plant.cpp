/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "binding_meta.h"
#include "det_math.h"

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
    {"plan.with_vehicle", "knob", "", 0, 1,
     "1: the navigator plans with each vehicle's own acceleration, top speed, cornering and size; "
     "0: with its own (veh_params)."},
    {"plan.accel_margin", "knob", "1", 0.3, 1,
     "The share of the vehicle's max_accel the navigator plans with."},
    {"plan.lateral_g", "knob", "g", 0.1, 1.2,
     "The lateral acceleration the navigator corners at, in g; at most tip_margin of the "
     "vehicle's static stability factor."},
    {"plan.tip_margin", "knob", "1", 0.3, 1,
     "The share of the vehicle's static stability factor (its rollover threshold, in g) the "
     "navigator may corner at."},
    {"drift_nsub", "knob", "", 0, 64,
     "Substeps of the kinematic bicycle the drift is measured against; 0 = the physics steps a "
     "tick (doc 31 D14's nsub = k)."},
    {"modulation.tire_onset", "knob", "1", 0.3, 1.2,
     "The tire use (combined slip over its peak) past which the drive slows (H4)."},
    {"modulation.tire_gain", "knob", "1", 0, 10,
     "How fast the drive's speed damping grows per unit of tire use past its onset; 0 = off."},
    {"modulation.tip_onset", "knob", "1", 0.1, 0.9,
     "The load transfer (|left - right| over the total wheel load) past which the drive slows and "
     "its barriers stiffen (H4)."},
    {"modulation.tip_gain", "knob", "1", 0, 10,
     "How fast the speed damping and the barriers' weight grow per unit of load transfer past "
     "its onset; 0 = off."},
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
    {"tire_use", "telemetry", "1", kNone, kNone,
     "The most used tire's combined slip over its peak: 1 = at the grip limit (0 below 2 m/s)."},
    {"load_transfer", "telemetry", "1", kNone, kNone,
     "|left - right| wheel load over the total: 1 = one side's wheels unloaded."},
    {"pitch_rad", "telemetry", "rad", kNone, kNone, "The chassis' pitch, nose up."},
    {"roll_rad", "telemetry", "rad", kNone, kNone, "The chassis' roll, left side up."},
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
  set_accel_margin(_o.accel_margin);
  set_lateral_g(_o.lateral_g);
  set_tip_margin(_o.tip_margin);
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
    const double c = det::cos(yaw), s = det::sin(yaw);
    const double h = _o.spawn_height >= 0.0 ? _o.spawn_height : resting_height(a.desc);
    const vec3 origin{p.x - c * a.rear_x, p.y - s * a.rear_x, _o.ground_z + h};
    quat q;
    q.w = det::cos(0.5 * yaw);
    q.z = det::sin(0.5 * yaw);
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
  s.heading = static_cast<float>(det::atan2(fwd.y, fwd.x));
  s.speed = static_cast<float>(_units.to_normalized(dot(v_rear, fwd), dim::velocity, _frame));
  s.steer = static_cast<float>(vs.steer);
  // Measured, for the drive's modulators (H4).
  const vec3 left = rotate(b.rotation, {0.0, 1.0, 0.0});
  s.pitch = static_cast<float>(det::asin(std::max(-1.0, std::min(1.0, fwd.z))));
  s.roll = static_cast<float>(det::asin(std::max(-1.0, std::min(1.0, left.z))));
  double use = 0.0, l_load = 0.0, r_load = 0.0;
  const double ks = std::max(1e-6, a.desc.tire_peak_slip);
  const double as = std::max(1e-6, a.desc.tire_peak_slip_angle);
  for (std::size_t w = 0; w < vs.wheels.size(); ++w) {
    const wheel_state &ws = vs.wheels[w];
    if (!ws.contact)
      continue;
    const double k = ws.longitudinal_slip / ks, al = ws.lateral_slip / as;
    use = std::max(use, std::sqrt(k * k + al * al));
    (w % 2 == 0 ? l_load : r_load) += std::max(0.0, ws.load); // wheel 2a is the left one
  }
  // A slip ratio at a crawl (a launch, settling) is no cornering hazard: below 2 m/s it reads 0.
  s.tire_use = std::fabs(dot(v_rear, fwd)) < 2.0 ? 0.0f : static_cast<float>(use);
  const double total = l_load + r_load;
  s.load_transfer = static_cast<float>(total > 0.0 ? std::fabs(l_load - r_load) / total : 0.0);
  return s;
}

cvc::nav::terrain_config nav_plant::terrain_config_of(const vehicle_desc &d, double mu,
                                                      double gravity) {
  cvc::nav::terrain_config c;
  c.gravity = gravity;
  // First gear's pull: the peak torque through the driveline, a converter's stall
  // multiplication, the gearing and the wheels.
  double ratio = d.gear_ratios.empty() ? 1.0 : d.gear_ratios.front();
  double radius = 0.0, z = 0.0, travel = 0.0, half_track = 0.0;
  for (const axle_desc &a : d.axles) {
    radius = std::max(radius, a.wheel_radius);
    z = std::min(z, a.z);
    travel = std::max(travel, 0.5 * (a.suspension_min + a.suspension_max));
    half_track = std::max(half_track, a.half_track);
  }
  const double stall = d.converter_k > 0.0 ? d.converter_stall : 1.0;
  const double pull = d.engine_max_torque * d.driveline_efficiency * stall * ratio *
                      d.differential_ratio / std::max(radius, 1e-6);
  const double power_limit = pull / (d.mass * gravity);
  c.max_grade = std::max(0.05, std::min(mu, power_limit));
  // The centre of gravity's height: the axle's attachment below the chassis origin, the
  // suspension's mid travel and the wheel, plus the centre of mass's offset.
  const double cg = -z + travel + radius + d.center_of_mass.z;
  c.tip_ratio = cg > 0.0 ? half_track / cg : 0.0;
  return c;
}

cvc::nav::dyn_bike_si nav_plant::dyn_bike_si_of(const vehicle_desc &d, double mu,
                                                double front_stiffness, double drive_lag,
                                                double gravity) {
  cvc::nav::dyn_bike_si si;
  si.mass = d.mass;
  si.yaw_inertia =
      d.mass * (d.half_extents.x * d.half_extents.x + d.half_extents.y * d.half_extents.y) / 3.0;
  // The axles: the frontmost steered one and the rearmost unsteered one; drive and brakes.
  double front = 0.0, radius = 0.0, z = 0.0, travel = 0.0;
  double brake_front = 0.0, brake_rear = 0.0;
  bool have_front = false, front_driven = false, rear_driven = false;
  const double rear = rear_axle_x(d);
  for (const axle_desc &a : d.axles) {
    radius = std::max(radius, a.wheel_radius);
    z = std::min(z, a.z);
    travel = std::max(travel, 0.5 * (a.suspension_min + a.suspension_max));
    if (a.steered && (!have_front || a.x > front)) {
      front = a.x;
      have_front = true;
    }
    (a.steered ? brake_front : brake_rear) += a.max_brake_torque;
    if (a.driven)
      (a.steered ? front_driven : rear_driven) = true;
  }
  if (!have_front)
    front = rear + d.wheelbase();
  si.cg_to_front = std::max(1e-3, front - d.center_of_mass.x);
  si.cg_to_rear = std::max(1e-3, d.center_of_mass.x - rear);
  si.cg_height = std::max(0.0, -z + travel + radius + d.center_of_mass.z);
  si.mu = mu;
  // Pacejka: sin(C pi / 2) = tire_sliding, C in (1, 2). B makes the cornering stiffness per unit
  // load, B C, kJoltTireStiffness times Jolt's: its lateral friction ramps linearly to 1 at
  // tire_peak_slip_angle (jolt_backend.cpp).
  const double kPi = 3.14159265358979323846;
  const double sliding = std::min(std::max(d.tire_sliding, 0.05), 0.999);
  si.tire_C = 2.0 - 2.0 * std::asin(sliding) / kPi;
  const double B = kJoltTireStiffness / (si.tire_C * std::max(d.tire_peak_slip_angle, 1e-3));
  si.tire_B_front = B * front_stiffness;
  si.tire_B_rear = B;
  si.front_drive = front_driven && rear_driven ? 0.5 : (front_driven ? 1.0 : 0.0);
  si.front_brake = brake_front + brake_rear > 0.0 ? brake_front / (brake_front + brake_rear) : 0.5;
  // A direct drive: one gear, no torque converter. The driver model's trim makes its commanded
  // acceleration the achieved one within the tick, drag and rolling resistance included.
  const bool direct = d.gear_ratios.size() <= 1 && !(d.converter_k > 0.0);
  si.drag = direct ? 0.0 : 0.5 * d.air_density * d.drag_coefficient * d.frontal_area;
  si.rolling_resistance = direct ? 0.0 : d.rolling_resistance;
  si.steer_rate = d.max_steer_rate;
  si.drive_lag = drive_lag >= 0.0 ? drive_lag : (direct ? 0.0 : 0.1);
  si.gravity = gravity;
  return si;
}

cvc::nav::diff::dyn_bike_params nav_plant::dyn_bike_params_at(const vehicle_desc &d,
                                                              double wheelbase, double mu) {
  const cvc::nav::dyn_bike_si si = dyn_bike_si_of(d, mu);
  cvc::world_units::normalized_frame f;
  f.scale = wheelbase / (si.cg_to_front + si.cg_to_rear);
  cvc::nav::diff::dyn_bike_params p = cvc::nav::dyn_bike_params_of(si, f);
  // Exactly the controller's wheelbase: the float conversion of each half need not sum to it.
  p.lr = static_cast<float>(wheelbase) - p.lf;
  return p;
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
  if (!_o.plan_with_vehicle)
    return l;
  const vehicle_desc &d = a.desc;
  l.a_max = static_cast<float>(
      _units.to_normalized(_o.accel_margin * d.max_accel, dim::acceleration, _frame));
  if (d.max_speed > 0.0)
    l.vmax = static_cast<float>(_units.to_normalized(d.max_speed, dim::velocity, _frame));
  const vec3 gv = _w.config().gravity;
  const double g = std::sqrt(gv.x * gv.x + gv.y * gv.y + gv.z * gv.z);
  double a_lat = _o.lateral_g * g;
  const double ssf = terrain_config_of(d, 0.9, g).tip_ratio;
  if (ssf > 0.0)
    a_lat = std::min(a_lat, _o.tip_margin * ssf * g);
  l.a_lat = static_cast<float>(_units.to_normalized(a_lat, dim::acceleration, _frame));
  // The body: the chassis box about the body origin, and the wheels where they stand out of it.
  double ahead = d.half_extents.x, behind = -d.half_extents.x, half_width = d.half_extents.y;
  for (const axle_desc &ax : d.axles) {
    ahead = std::max(ahead, ax.x + ax.wheel_radius);
    behind = std::min(behind, ax.x - ax.wheel_radius);
    half_width = std::max(half_width, ax.half_track + 0.5 * ax.wheel_width);
  }
  l.half_width = static_cast<float>(_units.to_normalized(half_width, dim::length, _frame));
  l.front = static_cast<float>(_units.to_normalized(ahead - a.rear_x, dim::length, _frame));
  l.rear = static_cast<float>(_units.to_normalized(a.rear_x - behind, dim::length, _frame));
  return l;
}

void nav_plant::command(int i, float steer, float accel) {
  agent &a = _agents.at(static_cast<std::size_t>(i));
  const double a_si = _units.from_normalized(accel, dim::acceleration, _frame);
  const double speed = _units.from_normalized(state(i).speed, dim::velocity, _frame);
  // Forward or backward by where the agent is to be moving at the tick's end; the driver
  // model takes the acceleration along that direction. A vehicle at rest reads a few cm/s on its
  // springs; that is rest, so a command to back up from it backs up (as nav's bicycle does from
  // zero) rather than braking against the jitter forever.
  constexpr double kRestSpeed = 0.1; // m/s
  const double v0 = std::fabs(speed) < kRestSpeed ? 0.0 : speed;
  const bool reverse = v0 + _dt * a_si < 0.0;
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

void nav_plant::set_plan_with_vehicle(bool on) { _o.plan_with_vehicle = on; }

void nav_plant::set_accel_margin(double share) {
  if (!(share > 0.0 && share <= 1.0))
    throw cvc::physics_error("nav_plant: accel_margin must be in (0, 1]");
  _o.accel_margin = share;
}

void nav_plant::set_lateral_g(double g) {
  if (!(g > 0.0 && g <= 2.0))
    throw cvc::physics_error("nav_plant: lateral_g must be in (0, 2]");
  _o.lateral_g = g;
}

void nav_plant::set_tip_margin(double share) {
  if (!(share > 0.0 && share <= 1.0))
    throw cvc::physics_error("nav_plant: tip_margin must be in (0, 1]");
  _o.tip_margin = share;
}

void nav_plant::set_drift_nsub(int n) {
  if (n < 0)
    throw cvc::physics_error("nav_plant: drift_nsub must be >= 0");
  _o.drift_nsub = n;
}

void nav_plant::set_modulation(const cvc::nav::plant_modulation &m) {
  const auto ok = [](float v) { return std::isfinite(v) && v >= 0.0f; };
  if (!(ok(m.tire_onset) && ok(m.tire_gain) && ok(m.tip_onset) && ok(m.tip_gain)))
    throw cvc::physics_error("nav_plant: modulation onsets and gains must be finite and >= 0");
  _o.modulation = m;
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
  read("plan.with_vehicle", _o.plan_with_vehicle ? 1.0 : 0.0, [&](double v) {
    if (v == 0.0 || v == 1.0)
      set_plan_with_vehicle(v == 1.0);
  });
  read("plan.accel_margin", _o.accel_margin, [&](double v) { set_accel_margin(v); });
  read("plan.lateral_g", _o.lateral_g, [&](double v) { set_lateral_g(v); });
  read("plan.tip_margin", _o.tip_margin, [&](double v) { set_tip_margin(v); });
  read("drift_nsub", static_cast<double>(_o.drift_nsub), [&](double v) {
    if (v == std::floor(v))
      set_drift_nsub(static_cast<int>(v));
  });
  const auto mod = [&](const char *name, float cvc::nav::plant_modulation::*f) {
    read(std::string("modulation.") + name, static_cast<double>(_o.modulation.*f), [&](double v) {
      cvc::nav::plant_modulation m = _o.modulation;
      m.*f = static_cast<float>(v);
      set_modulation(m);
    });
  };
  mod("tire_onset", &cvc::nav::plant_modulation::tire_onset);
  mod("tire_gain", &cvc::nav::plant_modulation::tire_gain);
  mod("tip_onset", &cvc::nav::plant_modulation::tip_onset);
  mod("tip_gain", &cvc::nav::plant_modulation::tip_gain);
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
    const cvc::nav::plant_state s = state(static_cast<int>(i));
    write_val(ctx, k + "tire_use", static_cast<double>(s.tire_use));
    write_val(ctx, k + "load_transfer", static_cast<double>(s.load_transfer));
    write_val(ctx, k + "pitch_rad", static_cast<double>(s.pitch));
    write_val(ctx, k + "roll_rad", static_cast<double>(s.roll));
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
