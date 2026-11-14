/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "boat_step.h"
#include "det_math.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cvc/core/exception.h>
#include <memory>
#include <string>

namespace cvc {
namespace physics {
namespace detail {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfPi = 1.57079632679489661923;

void need(bool ok, const std::string &what) {
  if (!ok)
    throw cvc::physics_error("add_boat: " + what);
}

bool finite(const vec3 &v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
bool at_least(double v, double lo) { return std::isfinite(v) && v >= lo; }

vec3 cross(const vec3 &a, const vec3 &b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

// v rotated by the unit quaternion q, and back.
vec3 rotate(const quat &q, const vec3 &v) {
  const vec3 u{q.x, q.y, q.z};
  const vec3 t = cross(u, v) * 2.0;
  return v + t * q.w + cross(u, t);
}
vec3 unrotate(const quat &q, const vec3 &v) { return rotate(quat{q.w, -q.x, -q.y, -q.z}, v); }

// The rudder's lift coefficient at an angle of attack in [-pi/2, pi/2]: linear to the stall, then
// a flat plate's, the lift part of a normal force cross_flow_drag sin(alpha), whose drag part is
// the drag's cross_flow_drag sin^2(alpha).
double lift_coefficient(const rudder_desc &r, double alpha) {
  const double a = std::fabs(alpha);
  const double cl =
      a <= r.stall_angle ? r.lift_slope * a : 0.5 * r.cross_flow_drag * det::sin(2.0 * a);
  return alpha < 0.0 ? -cl : cl;
}

std::uint64_t fnv1a(const void *data, std::size_t n, std::uint64_t h = 1469598103934665603ull) {
  const unsigned char *p = static_cast<const unsigned char *>(data);
  for (std::size_t i = 0; i < n; ++i) {
    h ^= p[i];
    h *= 1099511628211ull;
  }
  return h;
}

// What the boat adds to its hull body: the propulsion, the rudder and the actuators.
std::uint64_t digest_of(const boat_desc &d) {
  const propeller_desc &p = d.propeller;
  const rudder_desc &r = d.rudder;
  const double v[] = {p.position.x,       p.position.y,
                      p.position.z,       p.direction.x,
                      p.direction.y,      p.direction.z,
                      p.max_thrust,       p.max_reverse_thrust,
                      p.max_inflow_speed, p.outboard_max_angle,
                      r.position.x,       r.position.y,
                      r.position.z,       r.area,
                      r.lift_slope,       r.stall_angle,
                      r.drag_coefficient, r.cross_flow_drag,
                      r.max_angle,        d.actuator_time_constant};
  return fnv1a(v, sizeof(v));
}

template <class T> void put(std::string &out, const T &v) {
  out.append(reinterpret_cast<const char *>(&v), sizeof(T));
}
template <class T> void get(const std::string &in, std::size_t &pos, T &v) {
  if (pos + sizeof(T) > in.size())
    throw cvc::physics_error("restore: the boat state is truncated");
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

void boat_set::validate(const boat_desc &d) {
  need(d.hull.motion == motion_type::dynamic, "the hull must be a dynamic body");
  const propeller_desc &p = d.propeller;
  need(finite(p.position) && finite(p.direction), "the propeller's position and direction must "
                                                  "be finite");
  need(dot(p.direction, p.direction) > 0.0, "the propeller's direction must not be zero");
  need(at_least(p.max_thrust, 0.0) && at_least(p.max_reverse_thrust, 0.0) &&
           at_least(p.max_inflow_speed, 0.0),
       "the propeller's thrusts and max_inflow_speed must be finite and >= 0");
  need(at_least(p.outboard_max_angle, 0.0) && p.outboard_max_angle <= kHalfPi,
       "the propeller's outboard_max_angle must be in [0, pi/2]");
  const rudder_desc &r = d.rudder;
  need(finite(r.position), "the rudder's position must be finite");
  need(at_least(r.area, 0.0) && at_least(r.lift_slope, 0.0) && at_least(r.drag_coefficient, 0.0) &&
           at_least(r.cross_flow_drag, 0.0),
       "the rudder's area, lift_slope and drag coefficients must be finite and >= 0");
  need(std::isfinite(r.stall_angle) && r.stall_angle > 0.0 && r.stall_angle < kHalfPi,
       "the rudder's stall_angle must be in (0, pi/2)");
  need(at_least(r.max_angle, 0.0) && r.max_angle <= kHalfPi,
       "the rudder's max_angle must be in [0, pi/2]");
  need(at_least(d.actuator_time_constant, 0.0), "actuator_time_constant must be finite and >= 0");
}

boat_id boat_set::add(const boat_desc &d, body_id hull) {
  need(_next < 0xffffffffu, "too many boats");
  rt b;
  b.desc = d;
  // A copy of the mesh, as add_body keeps one: the caller may still change the one it gave.
  if (d.hull.hull)
    b.desc.hull.hull = std::make_shared<const hull_mesh>(*d.hull.hull);
  b.hull = hull;
  b.digest = digest_of(d);
  const double n = std::sqrt(dot(d.propeller.direction, d.propeller.direction));
  b.direction = d.propeller.direction * (1.0 / n);
  boat_id id;
  id.value = _next++;
  _b.emplace(id.value, std::move(b));
  return id;
}

body_id boat_set::remove(boat_id id) {
  const body_id hull = at(id).hull;
  _b.erase(id.value);
  return hull;
}

bool boat_set::is_hull(body_id id) const {
  for (const auto &e : _b)
    if (e.second.hull == id)
      return true;
  return false;
}

boat_set::rt &boat_set::at(boat_id id) {
  auto it = _b.find(id.value);
  if (it == _b.end())
    throw cvc::physics_error("unknown or removed boat id " + std::to_string(id.value));
  return it->second;
}

const boat_set::rt &boat_set::at(boat_id id) const {
  auto it = _b.find(id.value);
  if (it == _b.end())
    throw cvc::physics_error("unknown or removed boat id " + std::to_string(id.value));
  return it->second;
}

void boat_set::set_controls(boat_id id, const boat_controls &c) {
  if (!std::isfinite(c.throttle) || !std::isfinite(c.rudder) || c.throttle < -1.0 ||
      c.throttle > 1.0 || c.rudder < -1.0 || c.rudder > 1.0)
    throw cvc::physics_error("set_boat_controls: the throttle and rudder must be in [-1, 1]");
  at(id).command = c;
}

void boat_set::pre_step(world_backend &be, const field_set &fields, const world_config &cfg,
                        double t) {
  const double dt = cfg.fixed_dt;
  for (auto &e : _b) {
    rt &b = e.second;
    // The actuators: a first-order lag toward the controls.
    const double tau = b.desc.actuator_time_constant;
    const double k = tau > 0.0 ? -det::expm1(-dt / tau) : 1.0;
    b.throttle += (b.command.throttle - b.throttle) * k;
    b.rudder += (b.command.rudder - b.rudder) * k;

    const body_state s = be.state(b.hull);
    const vec3 com = s.position + rotate(s.rotation, fields.center_of_mass(b.hull));
    const auto point_velocity = [&](const vec3 &p) {
      return s.linear_velocity + cross(s.angular_velocity, p - com);
    };
    water_sample ws;

    // The propeller: an outboard swings its direction about the body's z axis, pushing the
    // stern right (-y) to turn left.
    const propeller_desc &pd = b.desc.propeller;
    b.outboard_angle = b.rudder * pd.outboard_max_angle;
    const double c = det::cos(b.outboard_angle), sn = det::sin(b.outboard_angle);
    const vec3 local{b.direction.x * c + b.direction.y * sn, b.direction.y * c - b.direction.x * sn,
                     b.direction.z};
    const vec3 dir = rotate(s.rotation, local);
    const vec3 prop = s.position + rotate(s.rotation, pd.position);
    fields.water_at(prop, t, ws);
    b.propeller_wet = ws.wet;
    b.inflow = 0.0;
    b.thrust = 0.0;
    b.thrust_force = vec3{};
    if (ws.wet) {
      b.inflow = dot(point_velocity(prop) - ws.velocity, dir);
      if (b.throttle != 0.0) {
        const double sign = b.throttle > 0.0 ? 1.0 : -1.0;
        const double rated = sign > 0.0 ? pd.max_thrust : pd.max_reverse_thrust;
        double falloff = 1.0;
        if (pd.max_inflow_speed > 0.0)
          falloff = std::min(1.0, std::max(0.0, 1.0 - sign * b.inflow / pd.max_inflow_speed));
        b.thrust = b.throttle * rated * falloff;
        b.thrust_force = dir * b.thrust;
      }
    }

    // The rudder: lift and drag from the water flowing past it, in the body's horizontal plane.
    const rudder_desc &rd = b.desc.rudder;
    b.rudder_angle = b.rudder * rd.max_angle;
    b.rudder_wet = false;
    b.angle_of_attack = b.rudder_lift = b.rudder_drag = 0.0;
    b.rudder_force = vec3{};
    vec3 at;
    if (rd.area > 0.0) {
      at = s.position + rotate(s.rotation, rd.position);
      fields.water_at(at, t, ws);
      b.rudder_wet = ws.wet;
      const vec3 flow = unrotate(s.rotation, ws.velocity - point_velocity(at));
      const double v = det::hypot(flow.x, flow.y);
      if (ws.wet && v > 0.0) {
        const double fx = flow.x / v, fy = flow.y / v;
        // The trailing edge's direction from the stock: aft, swung to port by the angle. Water
        // from astern meets the trailing edge first, and alpha is taken from that edge.
        const double tx = -det::cos(b.rudder_angle), ty = det::sin(b.rudder_angle);
        double alpha = det::atan2(tx * fy - ty * fx, tx * fx + ty * fy);
        if (alpha > kHalfPi)
          alpha -= kPi;
        else if (alpha < -kHalfPi)
          alpha += kPi;
        const double q = 0.5 * ws.density * rd.area * v * v;
        const double sa = det::sin(alpha);
        b.angle_of_attack = alpha;
        b.rudder_lift = q * lift_coefficient(rd, alpha);
        b.rudder_drag = q * (rd.drag_coefficient + rd.cross_flow_drag * sa * sa);
        // Lift normal to the flow, to its left (z x flow); drag along it.
        const vec3 f{-fy * b.rudder_lift + fx * b.rudder_drag,
                     fx * b.rudder_lift + fy * b.rudder_drag, 0.0};
        b.rudder_force = rotate(s.rotation, f);
      }
    }

    if (b.thrust != 0.0)
      be.add_force_at(b.hull, b.thrust_force, prop);
    if (b.rudder_lift != 0.0 || b.rudder_drag != 0.0)
      be.add_force_at(b.hull, b.rudder_force, at);
  }
}

boat_state boat_set::state(const world_backend &be, const field_set &fields, boat_id id,
                           double t) const {
  const rt &b = at(id);
  boat_state out;
  out.hull = b.hull;
  out.body = be.state(b.hull);
  const body_state &s = out.body;
  const vec3 com = s.position + rotate(s.rotation, fields.center_of_mass(b.hull));
  water_sample ws;
  fields.water_at(com, t, ws);
  out.water_velocity = ws.velocity;
  const vec3 forward = rotate(s.rotation, {1.0, 0.0, 0.0});
  const vec3 left = rotate(s.rotation, {0.0, 1.0, 0.0});
  const vec3 through = s.linear_velocity - ws.velocity;
  out.speed = dot(through, forward);
  out.sway = dot(through, left);
  out.heading = det::atan2(forward.y, forward.x);
  out.yaw_rate = s.angular_velocity.z;
  out.command = b.command;
  out.throttle = b.throttle;
  out.rudder = b.rudder;
  out.propeller_wet = b.propeller_wet;
  out.inflow = b.inflow;
  out.thrust = b.thrust;
  out.thrust_force = b.thrust_force;
  out.outboard_angle = b.outboard_angle;
  out.rudder_wet = b.rudder_wet;
  out.rudder_angle = b.rudder_angle;
  out.angle_of_attack = b.angle_of_attack;
  out.rudder_lift = b.rudder_lift;
  out.rudder_drag = b.rudder_drag;
  out.rudder_force = b.rudder_force;
  return out;
}

void boat_set::save(std::string &out) const {
  put(out, static_cast<std::uint64_t>(_b.size()));
  for (const auto &e : _b) {
    const rt &b = e.second;
    put(out, e.first);
    put(out, b.hull.value);
    put(out, b.digest);
    put(out, b.command.throttle);
    put(out, b.command.rudder);
    put(out, b.throttle);
    put(out, b.rudder);
    put_flag(out, b.propeller_wet);
    put_flag(out, b.rudder_wet);
    put(out, b.inflow);
    put(out, b.thrust);
    put(out, b.outboard_angle);
    put(out, b.rudder_angle);
    put(out, b.angle_of_attack);
    put(out, b.rudder_lift);
    put(out, b.rudder_drag);
    put_vec(out, b.thrust_force);
    put_vec(out, b.rudder_force);
  }
}

void boat_set::restore(const std::string &in, std::size_t &pos) {
  // Read into a copy, so a refused state leaves the boats as they were.
  std::map<std::uint32_t, rt> next = _b;
  std::size_t at = pos;
  std::uint64_t n = 0;
  get(in, at, n);
  if (n != next.size())
    throw cvc::physics_error("restore: this world's boats differ from the state's");
  for (auto &e : next) {
    rt &b = e.second;
    std::uint32_t id = 0, hull = 0;
    std::uint64_t digest = 0;
    get(in, at, id);
    get(in, at, hull);
    get(in, at, digest);
    if (id != e.first || hull != b.hull.value || digest != b.digest)
      throw cvc::physics_error("restore: this world's boats differ from the state's");
    get(in, at, b.command.throttle);
    get(in, at, b.command.rudder);
    get(in, at, b.throttle);
    get(in, at, b.rudder);
    b.propeller_wet = get_flag(in, at);
    b.rudder_wet = get_flag(in, at);
    get(in, at, b.inflow);
    get(in, at, b.thrust);
    get(in, at, b.outboard_angle);
    get(in, at, b.rudder_angle);
    get(in, at, b.angle_of_attack);
    get(in, at, b.rudder_lift);
    get(in, at, b.rudder_drag);
    get_vec(in, at, b.thrust_force);
    get_vec(in, at, b.rudder_force);
  }
  _b.swap(next);
  pos = at;
}

} // namespace detail
} // namespace physics
} // namespace cvc
