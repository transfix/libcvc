/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "vehicle_step.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cvc/core/exception.h>
#include <string>

namespace cvc {
namespace physics {

// --- presets --------------------------------------------------------------------

double vehicle_desc::wheelbase() const {
  double rear = 0.0, front = 0.0;
  bool have_rear = false, have_front = false;
  for (const axle_desc &a : axles) {
    if (!a.steered && (!have_rear || a.x < rear)) {
      rear = a.x;
      have_rear = true;
    }
    if (a.steered && (!have_front || a.x > front)) {
      front = a.x;
      have_front = true;
    }
  }
  return have_rear && have_front ? front - rear : 0.0;
}

double vehicle_desc::track() const {
  for (const axle_desc &a : axles)
    if (a.steered)
      return 2.0 * a.half_track;
  return 0.0;
}

vehicle_desc vehicle_desc::humvee() {
  vehicle_desc d;
  d.half_extents = {2.285, 1.08, 0.45};
  d.center_of_mass = {0.0, 0.0, -0.25};
  d.mass = 2340.0;
  axle_desc a;
  a.half_track = 0.91;
  a.z = -0.2;
  a.wheel_radius = 0.47;
  a.wheel_width = 0.31;
  a.wheel_inertia = 6.0;
  a.driven = true;
  a.max_brake_torque = 4000.0;
  a.suspension_min = 0.2;
  a.suspension_max = 0.5;
  a.suspension_frequency = 1.3;
  a.suspension_damping = 0.5;
  a.anti_roll_stiffness = 15000.0;
  axle_desc front = a, rear = a;
  front.x = 1.65;
  front.steered = true;
  rear.x = -1.65;
  rear.max_hand_brake_torque = 3000.0;
  d.axles = {front, rear};
  d.max_steer = 0.45;          // a 15 m turning circle
  d.max_steer_rate = 0.35;     // rad/s
  d.engine_max_torque = 515.0; // 6.5 L diesel
  d.engine_min_rpm = 700.0;
  d.engine_max_rpm = 3400.0;
  d.engine_inertia = 0.6;
  d.gear_ratios = {2.48, 1.48, 1.0, 0.75}; // 4L80-E
  d.reverse_ratio = -2.08;
  d.differential_ratio = 4.92; // 2.56 axle x 1.92 geared hubs
  d.shift_up_rpm = 2800.0;
  d.shift_down_rpm = 1400.0;
  d.max_accel = 2.5;
  d.max_decel = 7.5;
  d.drag_coefficient = 0.7;
  d.frontal_area = 3.8;
  d.rolling_resistance = 0.015;
  return d;
}

vehicle_desc vehicle_desc::roboracer() {
  vehicle_desc d;
  d.half_extents = {0.27, 0.10, 0.05};
  d.center_of_mass = {0.0, 0.0, -0.02};
  d.mass = 3.5;
  axle_desc a;
  a.half_track = 0.14;
  a.z = 0.0;
  a.wheel_radius = 0.055;
  a.wheel_width = 0.045;
  a.wheel_inertia = 2.0e-4;
  a.driven = true;
  a.max_brake_torque = 1.0;
  a.suspension_min = 0.01;
  a.suspension_max = 0.04;
  a.suspension_frequency = 4.0;
  a.suspension_damping = 0.6;
  a.anti_roll_stiffness = 50.0;
  axle_desc front = a, rear = a;
  front.x = 0.165;
  front.steered = true;
  rear.x = -0.165;
  d.axles = {front, rear};
  d.max_steer = 0.4189; // 24 degrees
  d.max_steer_rate = 3.2;
  d.engine_max_torque = 0.2; // brushless motor, at the motor
  d.engine_min_rpm = 300.0;
  d.engine_max_rpm = 20000.0;
  d.engine_inertia = 1.0e-4;
  d.gear_ratios = {1.0};
  d.reverse_ratio = -1.0;
  d.differential_ratio = 12.0;
  d.shift_up_rpm = 19000.0;
  d.shift_down_rpm = 200.0;
  d.max_accel = 7.0;
  d.max_decel = 8.0;
  d.drag_coefficient = 0.8;
  d.frontal_area = 0.03;
  d.rolling_resistance = 0.02;
  return d;
}

namespace detail {

namespace {

vec3 cross(const vec3 &a, const vec3 &b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
double norm(const vec3 &v) { return std::sqrt(dot(v, v)); }

vec3 rotate(const quat &q, const vec3 &v) {
  const double tx = 2 * (q.y * v.z - q.z * v.y), ty = 2 * (q.z * v.x - q.x * v.z),
               tz = 2 * (q.x * v.y - q.y * v.x);
  return {v.x + q.w * tx + (q.y * tz - q.z * ty), v.y + q.w * ty + (q.z * tx - q.x * tz),
          v.z + q.w * tz + (q.x * ty - q.y * tx)};
}

double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

void need(bool ok, const std::string &what) {
  if (!ok)
    throw cvc::physics_error("add_vehicle: " + what);
}
bool positive(double v) { return v > 0.0 && std::isfinite(v); }
bool finite(const vec3 &v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

void validate(const vehicle_desc &d) {
  need(positive(d.half_extents.x) && positive(d.half_extents.y) && positive(d.half_extents.z),
       "the chassis's half extents must be finite and > 0");
  need(finite(d.center_of_mass), "the centre of mass must be finite");
  need(positive(d.mass), "mass must be finite and > 0");
  need(!d.axles.empty(), "a vehicle needs at least one axle");
  bool steered = false, driven = false;
  for (const axle_desc &a : d.axles) {
    need(std::isfinite(a.x) && std::isfinite(a.z) && positive(a.half_track),
         "an axle's position must be finite and its half track > 0");
    need(positive(a.wheel_radius) && positive(a.wheel_width) && positive(a.wheel_inertia),
         "a wheel's radius, width and inertia must be finite and > 0");
    need(a.wheel_angular_damping >= 0.0 && a.max_brake_torque >= 0.0 &&
             a.max_hand_brake_torque >= 0.0 && a.anti_roll_stiffness >= 0.0,
         "damping, brake torques and anti-roll stiffness must be >= 0");
    need(a.suspension_min >= 0.0 && a.suspension_max > a.suspension_min,
         "the suspension needs 0 <= min < max");
    need(positive(a.suspension_frequency) && a.suspension_damping >= 0.0,
         "the suspension's frequency must be > 0 and its damping >= 0");
    steered = steered || a.steered;
    driven = driven || a.driven;
  }
  need(driven, "no axle is driven");
  need(!steered || d.wheelbase() > 0.0,
       "a steered axle needs an unsteered axle behind it (the rear axle Ackermann turns about)");
  need(d.max_steer > 0.0 && d.max_steer < 1.5, "max_steer must be in (0, 1.5) rad");
  need(positive(d.max_steer_rate), "max_steer_rate must be finite and > 0");
  need(positive(d.engine_max_torque) && positive(d.engine_max_rpm) && d.engine_min_rpm >= 0.0 &&
           d.engine_min_rpm < d.engine_max_rpm && positive(d.engine_inertia),
       "the engine needs a torque, an inertia and 0 <= min rpm < max rpm");
  need(!d.gear_ratios.empty(), "the transmission needs at least one gear");
  for (const double g : d.gear_ratios)
    need(positive(g), "gear ratios must be > 0");
  need(d.reverse_ratio < 0.0 && std::isfinite(d.reverse_ratio), "reverse_ratio must be < 0");
  need(positive(d.differential_ratio), "differential_ratio must be > 0");
  need(positive(d.max_accel) && positive(d.max_decel), "max_accel and max_decel must be > 0");
  need(d.drag_coefficient >= 0.0 && d.frontal_area >= 0.0 && d.air_density >= 0.0 &&
           d.rolling_resistance >= 0.0,
       "drag, area, air density and rolling resistance must be >= 0");
  need(positive(d.tire_peak_slip) && positive(d.tire_peak_slip_angle) && d.tire_sliding > 0.0 &&
           d.tire_sliding <= 1.0,
       "the tire needs peak slips > 0 and 0 < tire_sliding <= 1");
}

// How fast a medium answers a wheel's load (s).
constexpr double kMediumTime = 0.1;

// A force smoothly signed by the velocity it resists: full at |v| >> 0.1 m/s, zero at rest.
double rolling_sign(double v) { return std::tanh(v / 0.1); }

template <class T> void put(std::string &out, const T &v) {
  out.append(reinterpret_cast<const char *>(&v), sizeof(T));
}
template <class T> void get(const std::string &in, std::size_t &pos, T &v) {
  if (pos + sizeof(T) > in.size())
    throw cvc::physics_error("restore: the vehicle state is truncated");
  std::memcpy(&v, in.data() + pos, sizeof(T));
  pos += sizeof(T);
}

// Field by field, never a struct with padding: the bytes feed state_hash.
void put_vec(std::string &out, const vec3 &v) {
  put(out, v.x);
  put(out, v.y);
  put(out, v.z);
}
void get_vec(const std::string &in, std::size_t &pos, vec3 &v) {
  get(in, pos, v.x);
  get(in, pos, v.y);
  get(in, pos, v.z);
}
void put_flag(std::string &out, bool b) { put(out, static_cast<std::uint8_t>(b ? 1 : 0)); }
bool get_flag(const std::string &in, std::size_t &pos) {
  std::uint8_t b = 0;
  get(in, pos, b);
  return b != 0;
}

} // namespace

vehicle_set::rt &vehicle_set::at(vehicle_id id) {
  auto it = _v.find(id.value);
  if (it == _v.end())
    throw cvc::physics_error("unknown or removed vehicle id " + std::to_string(id.value));
  return it->second;
}
const vehicle_set::rt &vehicle_set::at(vehicle_id id) const {
  auto it = _v.find(id.value);
  if (it == _v.end())
    throw cvc::physics_error("unknown or removed vehicle id " + std::to_string(id.value));
  return it->second;
}

vehicle_id vehicle_set::add(world_backend &be, field_set &fields, const vehicle_desc &d,
                            const vec3 &position, const quat &rotation) {
  validate(d);
  need(_next < 0xffffffffu, "too many vehicles");
  vehicle_setup s;
  s.desc = &d;
  s.position = position;
  s.rotation = rotation;
  body_id chassis;
  std::uint32_t handle = 0;
  if (!be.add_vehicle(s, chassis, handle))
    throw cvc::unsupported_exception("cvc::physics: this backend has no vehicles (cap_vehicles); "
                                     "use the Jolt backend");
  // The chassis as the fields see it: a box about the body origin, its own centre of mass,
  // and no flow drag (the vehicle's aero is its drag).
  body_desc box_desc;
  box_desc.shape = box{d.half_extents};
  box_desc.mass = d.mass;
  fields.body_added(be, chassis, box_desc);
  fields.mark_vehicle(chassis, d.center_of_mass);

  rt v;
  v.desc = d;
  v.chassis = chassis;
  v.handle = handle;
  const std::size_t n = 2 * d.axles.size();
  v.rolling.assign(n, 0.0);
  v.medium.assign(n, 0.0);
  v.medium_load.assign(n, 0.0);
  v.readback.wheels.resize(n);
  vehicle_id id;
  id.value = _next++;
  _v.emplace(id.value, std::move(v));
  return id;
}

void vehicle_set::remove(world_backend &be, field_set &fields, vehicle_id id) {
  rt &v = at(id);
  be.remove_vehicle(v.handle);
  fields.body_removed(v.chassis);
  _v.erase(id.value);
}

bool vehicle_set::is_chassis(body_id id) const {
  for (const auto &e : _v)
    if (e.second.chassis == id)
      return true;
  return false;
}

void vehicle_set::drive(vehicle_id id, const drive_command &c) {
  if (!std::isfinite(c.steer) || !std::isfinite(c.accel))
    throw cvc::physics_error("drive: the command must be finite");
  rt &v = at(id);
  v.raw = false;
  v.command = c;
}

void vehicle_set::set_controls(vehicle_id id, const vehicle_controls &c) {
  if (!std::isfinite(c.steer) || !std::isfinite(c.throttle) || !std::isfinite(c.brake) ||
      !std::isfinite(c.hand_brake))
    throw cvc::physics_error("set_controls: the controls must be finite");
  rt &v = at(id);
  v.raw = true;
  v.controls = c;
}

void vehicle_set::pre_step(world_backend &be, const field_set &fields, const world_config &cfg,
                           const std::vector<material> &materials) {
  const double dt = cfg.fixed_dt;
  if (!_v.empty())
    be.set_medium_reach(fields.medium_reach());
  for (auto &entry : _v) {
    rt &v = entry.second;
    const vehicle_desc &d = v.desc;
    const body_state s = be.state(v.chassis);
    const vec3 fwd = rotate(s.rotation, {1.0, 0.0, 0.0});
    const double speed = dot(s.linear_velocity, fwd);
    const double a_meas = v.have_last_speed ? (speed - v.last_speed) / dt : 0.0;
    v.last_speed = speed;
    v.have_last_speed = true;

    // The steering actuator: slew toward the command, at most max_steer_rate.
    v.steer_command = clamp(v.raw ? v.controls.steer : v.command.steer, -d.max_steer, d.max_steer);
    const double slew = d.max_steer_rate * dt;
    v.steer += clamp(v.steer_command - v.steer, -slew, slew);

    // Ackermann: each steered wheel about the rear axle's line.
    const double L = d.wheelbase();
    double rear = 0.0;
    bool have_rear = false;
    for (const axle_desc &a : d.axles)
      if (!a.steered && (!have_rear || a.x < rear)) {
        rear = a.x;
        have_rear = true;
      }
    const double t = std::tan(v.steer);
    const double relax = std::min(1.0, dt / kMediumTime);
    std::vector<double> steer(2 * d.axles.size(), 0.0), load(2 * d.axles.size(), 0.0);
    for (std::size_t ai = 0; ai < d.axles.size(); ++ai) {
      const axle_desc &a = d.axles[ai];
      for (int side = 0; side < 2; ++side) {
        const std::size_t w = 2 * ai + side;
        const double y = side == 0 ? a.half_track : -a.half_track;
        if (a.steered && L > 0.0)
          steer[w] = std::atan((a.x - rear) * t / (L - y * t));
        // The medium answers the load over kMediumTime: a wheel touching down settles into
        // its rut as the load builds, rather than drumming on it.
        const wheel_state &ws = v.readback.wheels[w].state;
        v.medium_load[w] += ((ws.contact ? ws.load : 0.0) - v.medium_load[w]) * relax;
        load[w] = v.medium_load[w];
      }
    }

    // Pedals: the driver model, or the caller's own.
    double throttle = 0.0, brake = 0.0, hand_brake = 0.0;
    bool reverse = false, neutral = false;
    if (v.raw) {
      throttle = clamp(v.controls.throttle, 0.0, 1.0);
      brake = clamp(v.controls.brake, 0.0, 1.0);
      hand_brake = clamp(v.controls.hand_brake, 0.0, 1.0);
      reverse = v.controls.reverse;
      neutral = v.controls.neutral;
    } else {
      reverse = v.command.reverse;
      const double dir = reverse ? -1.0 : 1.0;
      const double along = dir * speed;    // speed along the commanded direction
      const double want = v.command.accel; // a_cmd is already along it
      const double got = dir * a_meas;
      // One controller: the wanted acceleration, corrected by how far the measured one
      // misses it, is a throttle when positive and a brake when negative. So a_cmd = 0 keeps
      // the speed on a grade (braking against gravity downhill, pulling against it uphill).
      constexpr double kDeadband = 0.05, kGain = 0.5, kHoldSpeed = 0.3;
      const double u = want + kGain * (want - got);
      if (along < -kHoldSpeed) {
        brake = 1.0; // moving the wrong way: stop first
      } else if (std::fabs(want) <= kDeadband && std::fabs(along) < kHoldSpeed) {
        brake = 1.0; // slow, with nothing asked: hold on the brakes (a grade included)
      } else if (u > kDeadband) {
        throttle = clamp(u / d.max_accel, 0.0, 1.0);
      } else if (u < -kDeadband) {
        brake = clamp(-u / d.max_decel, 0.0, 1.0);
      }
    }
    v.throttle = throttle;
    v.brake = brake;
    v.hand_brake = hand_brake;
    be.set_vehicle_inputs(v.handle, steer, load, (reverse ? -1.0 : 1.0) * throttle, brake,
                          hand_brake, neutral);

    // The chassis's resistance, from the last step's wheels.
    const vec3 com = s.position + rotate(s.rotation, d.center_of_mass);
    double rho = d.air_density;
    vec3 wind;
    fields.air_at(com, rho, wind);
    const vec3 u = s.linear_velocity - wind;
    const double un = norm(u);
    const vec3 aero = u * (-0.5 * rho * d.drag_coefficient * d.frontal_area * un);
    vec3 force = aero, torque;
    v.aero_force = aero;
    v.air_velocity = wind;
    for (std::size_t w = 0; w < v.readback.wheels.size(); ++w) {
      const vehicle_readback::wheel &rw = v.readback.wheels[w];
      v.rolling[w] = v.medium[w] = 0.0;
      if (!rw.state.contact || !(rw.state.load > 0.0))
        continue;
      const vec3 arm = rw.state.contact_point - com;
      const vec3 vp = s.linear_velocity + cross(s.angular_velocity, arm);
      const double v_long = dot(vp, rw.longitudinal);
      const double sign = rolling_sign(v_long);
      const material_id mid = rw.state.contact_material;
      const double crr = mid < materials.size() && materials[mid].rolling_resistance > 0.0f
                             ? materials[mid].rolling_resistance
                             : d.rolling_resistance;
      v.rolling[w] = crr * rw.state.load;
      const axle_desc &a = d.axles[w / 2];
      const double z = rw.state.sinkage;
      v.medium[w] = 0.5 * a.wheel_width * rw.medium_modulus * z * z;
      const vec3 f = rw.longitudinal * (-(v.rolling[w] + v.medium[w]) * sign);
      force = force + f;
      torque = torque + cross(arm, f);
    }
    if (s.active)
      be.add_field_load(v.chassis, force, torque);
  }
}

void vehicle_set::post_step(world_backend &be, const world_config &) {
  for (auto &entry : _v)
    be.read_vehicle(entry.second.handle, entry.second.readback);
}

vehicle_state vehicle_set::state(const world_backend &be, vehicle_id id) const {
  const rt &v = at(id);
  vehicle_state out;
  out.chassis = v.chassis;
  out.body = be.state(v.chassis);
  out.speed = dot(out.body.linear_velocity, rotate(out.body.rotation, {1.0, 0.0, 0.0}));
  out.steer = v.steer;
  out.steer_command = v.steer_command;
  out.throttle = v.throttle;
  out.brake = v.brake;
  out.hand_brake = v.hand_brake;
  out.gear = v.readback.gear;
  out.engine_rpm = v.readback.engine_rpm;
  out.aero_force = v.aero_force;
  out.air_velocity = v.air_velocity;
  out.wheels.reserve(v.readback.wheels.size());
  for (std::size_t w = 0; w < v.readback.wheels.size(); ++w) {
    wheel_state ws = v.readback.wheels[w].state;
    ws.rolling_resistance = v.rolling[w];
    ws.medium_resistance = v.medium[w];
    out.wheels.push_back(ws);
  }
  return out;
}

void vehicle_set::save(std::string &out) const {
  put(out, static_cast<std::uint64_t>(_v.size()));
  for (const auto &e : _v) {
    const rt &v = e.second;
    put(out, e.first);
    put(out, v.steer);
    put(out, v.steer_command);
    put_flag(out, v.raw);
    put(out, v.command.steer);
    put(out, v.command.accel);
    put_flag(out, v.command.reverse);
    put(out, v.controls.steer);
    put(out, v.controls.throttle);
    put(out, v.controls.brake);
    put(out, v.controls.hand_brake);
    put_flag(out, v.controls.reverse);
    put_flag(out, v.controls.neutral);
    put(out, v.last_speed);
    put_flag(out, v.have_last_speed);
    put(out, static_cast<std::uint64_t>(v.readback.wheels.size()));
    for (const double l : v.medium_load)
      put(out, l);
    for (const vehicle_readback::wheel &w : v.readback.wheels) {
      const wheel_state &s = w.state;
      put(out, s.load);
      put_flag(out, s.contact);
      put_vec(out, s.contact_point);
      put_vec(out, w.longitudinal);
      put(out, s.contact_material);
      put(out, s.sinkage);
      put(out, s.medium_depth);
      put(out, w.medium_modulus);
    }
  }
}

void vehicle_set::restore(const std::string &in, std::size_t &pos) {
  std::uint64_t n = 0;
  get(in, pos, n);
  if (n != _v.size())
    throw cvc::physics_error("restore: this world does not hold the vehicles the state was "
                             "saved with");
  for (auto &e : _v) {
    rt &v = e.second;
    std::uint32_t key = 0;
    get(in, pos, key);
    if (key != e.first)
      throw cvc::physics_error("restore: the vehicles were added in a different order");
    get(in, pos, v.steer);
    get(in, pos, v.steer_command);
    v.raw = get_flag(in, pos);
    get(in, pos, v.command.steer);
    get(in, pos, v.command.accel);
    v.command.reverse = get_flag(in, pos);
    get(in, pos, v.controls.steer);
    get(in, pos, v.controls.throttle);
    get(in, pos, v.controls.brake);
    get(in, pos, v.controls.hand_brake);
    v.controls.reverse = get_flag(in, pos);
    v.controls.neutral = get_flag(in, pos);
    get(in, pos, v.last_speed);
    v.have_last_speed = get_flag(in, pos);
    std::uint64_t wheels = 0;
    get(in, pos, wheels);
    if (wheels != v.readback.wheels.size())
      throw cvc::physics_error("restore: a vehicle has a different number of wheels");
    for (double &l : v.medium_load)
      get(in, pos, l);
    for (vehicle_readback::wheel &w : v.readback.wheels) {
      wheel_state &s = w.state;
      get(in, pos, s.load);
      s.contact = get_flag(in, pos);
      get_vec(in, pos, s.contact_point);
      get_vec(in, pos, w.longitudinal);
      get(in, pos, s.contact_material);
      get(in, pos, s.sinkage);
      get(in, pos, s.medium_depth);
      get(in, pos, w.medium_modulus);
    }
  }
}

} // namespace detail
} // namespace physics
} // namespace cvc
