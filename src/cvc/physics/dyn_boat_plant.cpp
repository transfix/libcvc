/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "boat_step.h"
#include "det_math.h"
#include "dyn_world.h"

#include <algorithm>
#include <cmath>
#include <cvc/core/exception.h>
#include <cvc/physics/dyn_boat_plant.h>
#include <cvc/physics/hull.h>
#include <map>
#include <stdexcept>
#include <string>
#include <variant>

namespace cvc {
namespace physics {

namespace {

using dim = cvc::world_units::dimension;

// The speed loop: dyn_driver's deadband, hold speed, correction gain and trim rate (1/s); the
// shortest the hold eases the speed to zero over (s); the least share of the rated thrust the
// throttle is divided by (the falloff near max_inflow_speed).
constexpr double kDeadband = 0.05, kHoldSpeed = 0.3, kGain = 0.5, kTrim = 2.0;
constexpr double kHoldTau = 1.0;
constexpr double kFalloffFloor = 0.1;
// The yaw loop: the rudder per unit of curvature error times turn_radius, the trim's rate (1/s),
// and the least speed (m/s) a yaw-rate error is divided by.
constexpr double kYawGain = 1.0, kYawTrim = 1.0, kSpeedFloor = 0.5;
// Where the plant asks for the water's top: well above any sea, inside every fluid's box.
constexpr double kHigh = 1.0e4;

double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

vec3 cross(const vec3 &a, const vec3 &b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

vec3 unrotate(const quat &q, const vec3 &v) { return dyn::rotate(quat{q.w, -q.x, -q.y, -q.z}, v); }

vec3 mat_mul(const std::array<double, 9> &m, const vec3 &v) {
  return {m[0] * v.x + m[1] * v.y + m[2] * v.z, m[3] * v.x + m[4] * v.y + m[5] * v.z,
          m[6] * v.x + m[7] * v.y + m[8] * v.z};
}

quat about_z(double a) { return {det::cos(0.5 * a), 0.0, 0.0, det::sin(0.5 * a)}; }

// A box's hull with each face split into n x n squares, two triangles each (12 n^2), so the
// water's push on the sides spreads along them as the boat yaws (physics_boat_test's).
std::shared_ptr<const hull_mesh> split_box_hull(const vec3 &h, int n) {
  auto out = std::make_shared<hull_mesh>();
  const double e[3] = {h.x, h.y, h.z};
  const int faces[6][4] = {{0, +1, 1, 2}, {0, -1, 2, 1}, {1, +1, 2, 0},
                           {1, -1, 0, 2}, {2, +1, 0, 1}, {2, -1, 1, 0}};
  for (const auto &f : faces) {
    const int a = f[0], u = f[2], v = f[3];
    const std::uint32_t base = static_cast<std::uint32_t>(out->vertices.size());
    for (int j = 0; j <= n; ++j)
      for (int i = 0; i <= n; ++i) {
        double p[3];
        p[a] = f[1] * e[a];
        p[u] = e[u] * (-1.0 + 2.0 * i / n);
        p[v] = e[v] * (-1.0 + 2.0 * j / n);
        out->vertices.push_back({p[0], p[1], p[2]});
      }
    const auto at = [&](int i, int j) {
      return base + static_cast<std::uint32_t>(j * (n + 1) + i);
    };
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i) {
        out->triangles.push_back({at(i, j), at(i + 1, j), at(i + 1, j + 1)});
        out->triangles.push_back({at(i, j), at(i + 1, j + 1), at(i, j + 1)});
      }
  }
  return out;
}

// The volume of a closed mesh below the plane z = d, upright: minus the integral of (d - z) n_z
// over its faces below the plane (the divergence theorem; the cap at the plane adds nothing).
double volume_below(const hull_mesh &h, double d) {
  double v = 0.0;
  for (const auto &t : h.triangles) {
    const vec3 p[3] = {h.vertices[t[0]], h.vertices[t[1]], h.vertices[t[2]]};
    // The triangle clipped to z < d: up to four corners.
    vec3 q[4];
    int n = 0;
    for (int k = 0; k < 3; ++k) {
      const vec3 &a = p[k], &b = p[(k + 1) % 3];
      const bool ina = a.z < d, inb = b.z < d;
      if (ina)
        q[n++] = a;
      if (ina != inb) {
        const double s = (d - a.z) / (b.z - a.z);
        q[n++] = a + (b - a) * s;
      }
    }
    for (int k = 1; k + 1 < n; ++k) {
      const vec3 area = cross(q[k] - q[0], q[k + 1] - q[0]) * 0.5;
      const double depth = d - (q[0].z + q[k].z + q[k + 1].z) / 3.0;
      v -= depth * area.z;
    }
  }
  return v;
}

// A closed mesh as a solid of unit density: its volume, centroid and second moment about the
// origin (the integral of x x^T over it, row-major), summed over the tetrahedra the faces make
// with the origin.
void solid_of(const hull_mesh &h, double &volume, vec3 &centroid, std::array<double, 9> &second) {
  volume = 0.0;
  vec3 first;
  second.fill(0.0);
  for (const auto &t : h.triangles) {
    const vec3 &a = h.vertices[t[0]], &b = h.vertices[t[1]], &c = h.vertices[t[2]];
    const double det6 = dot(a, cross(b, c));
    volume += det6 / 6.0;
    first = first + (a + b + c) * (det6 / 24.0);
    const vec3 s = a + b + c;
    const double k = det6 / 120.0;
    const vec3 r[4] = {a, b, c, s};
    for (const vec3 &x : r) {
      const double xs[3] = {x.x, x.y, x.z};
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          second[static_cast<std::size_t>(3 * i + j)] += k * xs[i] * xs[j];
    }
  }
  centroid = volume > 0.0 ? first * (1.0 / volume) : vec3{};
}

void add(dyn_boat_books &to, const dyn_boat_books &d) {
  to.drive += d.drive;
  to.brake += d.brake;
  to.rudder += d.rudder;
  to.damping += d.damping;
  to.water += d.water;
  to.gravity += d.gravity;
  to.residual += d.residual;
}

void need(bool ok, const std::string &what) {
  if (!ok)
    throw cvc::physics_error("dyn_boat_plant: " + what);
}

// Nav's frame in metres, checked before world_units sees it (it throws its own kind).
cvc::world_units units_of(double metres_per_world_unit) {
  need(metres_per_world_unit > 0.0 && std::isfinite(metres_per_world_unit),
       "metres_per_world_unit must be finite and > 0");
  return cvc::world_units(
      cvc::world_units::config{cvc::world_units::system::si, metres_per_world_unit});
}

} // namespace

double dyn_boat_books::flow() const {
  return std::fabs(drive) + std::fabs(brake) + std::fabs(rudder) + std::fabs(damping) +
         std::fabs(water) + std::fabs(gravity);
}

// ── the presets ─────────────────────────────────────────────────────────────────────────────

dyn_boat_desc dyn_boat_desc::launch() {
  dyn_boat_desc d;
  boat_desc &b = d.boat;
  const vec3 half{2.0, 0.75, 0.3};
  b.hull.shape = box{half};
  b.hull.hull = split_box_hull(half, 4);
  b.hull.mass = 900.0;
  b.hull.linear_damping = 0.0; // the water is its resistance
  b.hull.angular_damping = 0.0;
  b.hull.allow_sleeping = false;
  b.propeller.position = {-2.1, 0.0, -0.35};
  b.propeller.max_thrust = 2000.0;
  b.propeller.max_reverse_thrust = 1000.0;
  b.propeller.max_inflow_speed = 8.0;
  b.rudder.position = {-2.3, 0.0, -0.35};
  b.rudder.area = 0.12;
  b.actuator_time_constant = 0.5;
  d.max_speed = 3.19;
  d.turn_radius = 15.4;
  return d;
}

dyn_boat_desc dyn_boat_desc::jonboat() {
  dyn_boat_desc d;
  boat_desc &b = d.boat;
  const vec3 half{1.8, 0.7, 0.25};
  b.hull.shape = box{half};
  b.hull.hull = split_box_hull(half, 4);
  b.hull.mass = 350.0;
  b.hull.linear_damping = 0.0;
  b.hull.angular_damping = 0.0;
  b.hull.allow_sleeping = false;
  b.propeller.position = {-1.9, 0.0, -0.35};
  b.propeller.max_thrust = 1500.0;
  b.propeller.max_reverse_thrust = 700.0;
  b.propeller.max_inflow_speed = 9.0;
  b.propeller.outboard_max_angle = 0.5;
  b.rudder.position = {-1.95, 0.0, -0.3};
  b.rudder.area = 0.03;
  b.actuator_time_constant = 0.4;
  d.max_speed = 4.0;
  d.turn_radius = 4.1;
  return d;
}

dyn_boat_desc dyn_boat_desc::preset(const std::string &name) {
  if (name == "launch")
    return launch();
  if (name == "jonboat")
    return jonboat();
  throw cvc::physics_error("dyn_boat_desc: no boat preset named '" + name + "' (launch, jonboat)");
}

std::vector<std::string> dyn_boat_desc::presets() { return {"launch", "jonboat"}; }

// ── hydrostatics ────────────────────────────────────────────────────────────────────────────

double dyn_boat_plant::float_height(const boat_desc &d, double mass, double density) {
  need(d.hull.hull && !d.hull.hull->triangles.empty(), "the boat's hull needs a hull_mesh");
  need(std::isfinite(mass) && mass > 0.0, "the hull's mass must be > 0");
  need(std::isfinite(density) && density > 0.0, "the water's density must be > 0");
  const hull_mesh &h = *d.hull.hull;
  double lo = h.vertices.front().z, hi = lo;
  for (const vec3 &v : h.vertices) {
    lo = std::min(lo, v.z);
    hi = std::max(hi, v.z);
  }
  const double want = mass / density;
  need(volume_below(h, hi) >= want, "the boat is too heavy for its hull to float");
  // Bisect for the waterline (body z) at which the hull displaces its mass.
  for (int k = 0; k < 200 && hi - lo > 1e-12; ++k) {
    const double mid = 0.5 * (lo + hi);
    (volume_below(h, mid) < want ? lo : hi) = mid;
  }
  return -0.5 * (lo + hi);
}

// ── the plant ───────────────────────────────────────────────────────────────────────────────

void dyn_boat_plant::check(const dyn_boat_desc &d) const {
  detail::boat_set::validate(d.boat); // add_boat's own refusals (it names them "add_boat")
  need(d.boat.hull.hull && !d.boat.hull.hull->triangles.empty() &&
           !d.boat.hull.hull->vertices.empty(),
       "the boat's hull needs a hull_mesh (to float it at its draft)");
  need(std::isfinite(d.max_speed) && d.max_speed > 0.0, "max_speed must be finite and > 0");
  need(std::isfinite(d.turn_radius) && d.turn_radius > 0.0, "turn_radius must be finite and > 0");
  need(std::holds_alternative<box>(d.boat.hull.shape) ||
           std::holds_alternative<sphere>(d.boat.hull.shape) ||
           std::holds_alternative<convex_hull>(d.boat.hull.shape),
       "the hull's shape must be a box, a sphere or a convex hull");
  double lo = d.boat.hull.hull->vertices.front().x, hi = lo;
  for (const vec3 &v : d.boat.hull.hull->vertices) {
    lo = std::min(lo, v.x);
    hi = std::max(hi, v.x);
  }
  need(hi - lo > 0.0, "the hull_mesh has no length");
}

dyn_boat_plant::dyn_boat_plant(std::vector<dyn_boat_desc> descs, dyn_boat_plant_options options)
    : _descs(std::move(descs)), _o(std::move(options)),
      _units(units_of(_o.plant.metres_per_world_unit)) {
  need(!_descs.empty(), "needs at least one dyn_boat_desc");
  need(_o.physics != nullptr, "needs a world to float the boats in (options.physics)");
  need(std::isfinite(_o.plant.latency) && _o.plant.latency >= 0.0, "the latency must be >= 0");
  need(_o.plant.brake_margin > 0.0 && _o.plant.brake_margin <= 1.0,
       "brake_margin must be in (0, 1]");
  for (const dyn_boat_desc &d : _descs)
    check(d);
}

int dyn_boat_plant::add_desc(dyn_boat_desc d) {
  check(d);
  _descs.push_back(std::move(d));
  return static_cast<int>(_descs.size()) - 1;
}

double dyn_boat_plant::kinetic_of(const agent &a, const body_state &s) const {
  const vec3 &v = s.linear_velocity;
  const vec3 wb = unrotate(s.rotation, s.angular_velocity);
  return 0.5 * a.props.mass * dot(v, v) + 0.5 * dot(wb, mat_mul(a.props.I, wb));
}

vec3 dyn_boat_plant::com_of(const agent &a, const body_state &s) const {
  return s.position + dyn::rotate(s.rotation, a.props.com);
}

void dyn_boat_plant::heading_speed(const agent &a, const body_state &s, double &psi,
                                   double &u) const {
  const vec3 f = dyn::rotate(s.rotation, {1.0, 0.0, 0.0});
  psi = det::atan2(f.y, f.x);
  const double h = std::sqrt(f.x * f.x + f.y * f.y);
  // The origin's velocity: the centre of mass's, and the spin's about it.
  const vec3 v = s.linear_velocity + cross(s.angular_velocity, s.position - com_of(a, s));
  u = h > 1e-9 ? (v.x * f.x + v.y * f.y) / h : 0.0;
}

void dyn_boat_plant::attach(int n, const cvc::nav::plant_state *start,
                            const cvc::world_units::normalized_frame &frame, double dt) {
  frame.validate();
  need(n >= 0 && (n == 0 || start), "attach needs n >= 0 starts");
  need(dt > 0.0, "the tick must be > 0");
  need(_o.physics != nullptr, "the world is gone (world_gone)");
  world &w = *_o.physics;
  const double k = dt / w.config().fixed_dt;
  const int nsub = static_cast<int>(std::lround(k));
  need(nsub >= 1 && std::fabs(k - nsub) <= 1e-6,
       "nav's tick must be a whole number of the world's steps");
  // The boats of an earlier attach leave the world.
  for (const agent &a : _agents) {
    try {
      w.remove_boat(a.boat);
    } catch (const std::exception &) {
    }
  }
  _agents.clear();
  _nsub = nsub;
  _frame = frame;
  _dt = dt;
  _ticking = false;
  // A hull pressed against a pier or another boat stays in contact: every step's contacts.
  if (w.config().contacts != contact_reporting::all) {
    world_config c = w.config();
    c.contacts = contact_reporting::all;
    w.reconfigure(c);
  }
  _agents.reserve(static_cast<std::size_t>(n));
  try {
    for (int i = 0; i < n; ++i)
      place(start[i], _descs[static_cast<std::size_t>(i) % _descs.size()]);
  } catch (...) {
    // Refused: none of this attach's boats stay.
    for (const agent &a : _agents) {
      try {
        w.remove_boat(a.boat);
      } catch (const std::exception &) {
      }
    }
    _agents.clear();
    throw;
  }
}

void dyn_boat_plant::place(const cvc::nav::plant_state &start, const dyn_boat_desc &desc) {
  world &w = *_o.physics;
  const cvc::world_units::xyz p = _units.normalized_point_to_si(start.x, start.y, 0.0, _frame);
  // The water's top over the start, and its density there.
  const water_sample top = w.water_at({p.x, p.y, kHigh});
  if (!(top.density > 0.0))
    throw cvc::physics_error("dyn_boat_plant: no water under the start (" + std::to_string(p.x) +
                             ", " + std::to_string(p.y) + "): the world has no fluid field there");
  const double level = kHigh + top.depth;
  agent a;
  a.desc = desc;
  const hull_mesh &mesh = *desc.boat.hull.hull;
  double x0 = mesh.vertices.front().x, x1 = x0, keel = mesh.vertices.front().z;
  for (const vec3 &v : mesh.vertices) {
    x0 = std::min(x0, v.x);
    x1 = std::max(x1, v.x);
    keel = std::min(keel, v.z);
    a.half_width = std::max(a.half_width, std::fabs(v.y));
  }
  a.L = x1 - x0;
  a.front = x1;
  a.rear = -x0;
  a.tan_max = a.L / desc.turn_radius;
  a.max_steer = det::atan(a.tan_max);
  // The mass: as given, or the engine's from the density (the boat added first, then floated).
  double mass = desc.boat.hull.mass;
  const quat rot = about_z(start.heading);
  if (mass > 0.0) {
    const double fh = float_height(desc.boat, mass, top.density);
    const water_sample under = w.water_at({p.x, p.y, level + fh + keel + 1e-9});
    need(under.wet, "the water at the start is shallower than the boat's draft");
    a.boat = w.add_boat(desc.boat, {p.x, p.y, level + fh}, rot);
    a.hull = w.boat_state(a.boat).hull;
  } else {
    a.boat = w.add_boat(desc.boat, {p.x, p.y, level}, rot);
    a.hull = w.boat_state(a.boat).hull;
    mass = w.get_body(a.hull).mass;
    double fh = 0.0;
    try {
      fh = float_height(desc.boat, mass, top.density);
      need(w.water_at({p.x, p.y, level + fh + keel + 1e-9}).wet,
           "the water at the start is shallower than the boat's draft");
    } catch (...) {
      w.remove_boat(a.boat);
      throw;
    }
    body_state s;
    s.position = {p.x, p.y, level + fh};
    s.rotation = rot;
    w.set_state(a.hull, s);
  }
  // The hull's mass properties, as the engine takes them from its shape.
  hull_props &hp = a.props;
  const body_settings bs = w.get_body(a.hull);
  hp.mass = bs.mass > 0.0 ? bs.mass : mass;
  hp.linear_damping = bs.linear_damping;
  hp.angular_damping = bs.angular_damping;
  hp.I.fill(0.0);
  const shape &sh = desc.boat.hull.shape;
  if (const box *b = std::get_if<box>(&sh)) {
    const vec3 &e = b->half_extents;
    hp.I[0] = hp.mass / 3.0 * (e.y * e.y + e.z * e.z);
    hp.I[4] = hp.mass / 3.0 * (e.x * e.x + e.z * e.z);
    hp.I[8] = hp.mass / 3.0 * (e.x * e.x + e.y * e.y);
  } else if (const sphere *sp = std::get_if<sphere>(&sh)) {
    hp.I[0] = hp.I[4] = hp.I[8] = 0.4 * hp.mass * sp->radius * sp->radius;
  } else {
    // A convex hull: the hull_mesh as a solid of uniform density, scaled to the mass.
    double v = 0.0;
    std::array<double, 9> m2{};
    solid_of(mesh, v, hp.com, m2);
    if (v > 0.0) {
      const double c[3] = {hp.com.x, hp.com.y, hp.com.z};
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          m2[static_cast<std::size_t>(3 * i + j)] -= v * c[i] * c[j]; // about the centroid
      const double tr = m2[0] + m2[4] + m2[8];
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          hp.I[static_cast<std::size_t>(3 * i + j)] =
              hp.mass / v * ((i == j ? tr : 0.0) - m2[static_cast<std::size_t>(3 * i + j)]);
    }
  }
  a.now = w.boat_state(a.boat);
  a.kinetic0 = kinetic_of(a, a.now.body);
  a.tick = a.total = dyn_boat_books{};
  a.tick.kinetic = a.total.kinetic = a.kinetic0;
  double psi = 0.0, u = 0.0;
  heading_speed(a, a.now.body, psi, u);
  a.vwx0 = a.now.body.linear_velocity.x;
  a.vwy0 = a.now.body.linear_velocity.y;
  a.psi0 = psi;
  measure(a, false);
  _agents.push_back(std::move(a));
}

int dyn_boat_plant::add_vehicle(const cvc::nav::plant_state &start, int vehicle) {
  need(_dt > 0.0, "add_vehicle: attach() first");
  need(_o.physics != nullptr, "add_vehicle: the world is gone (world_gone)");
  if (vehicle >= static_cast<int>(_descs.size()) || vehicle < -1)
    throw std::invalid_argument("dyn_boat_plant::add_vehicle: vehicle " + std::to_string(vehicle) +
                                " is not one of the plant's " + std::to_string(_descs.size()));
  const int i = size();
  const std::size_t k = vehicle >= 0 ? static_cast<std::size_t>(vehicle)
                                     : static_cast<std::size_t>(i) % _descs.size();
  place(start, _descs[k]);
  return i;
}

void dyn_boat_plant::remove_vehicle(int i) {
  if (i < 0 || i >= size())
    throw std::out_of_range("dyn_boat_plant::remove_vehicle: no vehicle " + std::to_string(i));
  if (_o.physics) {
    try {
      _o.physics->remove_boat(_agents[static_cast<std::size_t>(i)].boat);
    } catch (const std::exception &) {
    }
  }
  _agents.erase(_agents.begin() + i);
  for (agent &a : _agents) // the boat each touched, by index
    if (a.touched == i) {
      a.touched = -1;
      a.touch_depth = 0.0;
    } else if (a.touched > i) {
      --a.touched;
    }
}

void dyn_boat_plant::world_gone() { _o.physics = nullptr; }

physics::boat_state dyn_boat_plant::boat_state(int i) const {
  const agent &a = _agents.at(static_cast<std::size_t>(i));
  need(_o.physics != nullptr, "boat_state: the world is gone (world_gone)");
  return _o.physics->boat_state(a.boat);
}

cvc::nav::plant_state dyn_boat_plant::state(int i) const {
  const agent &a = _agents.at(static_cast<std::size_t>(i));
  const body_state &s = a.now.body;
  cvc::nav::plant_state out;
  const cvc::world_units::xyz n =
      _units.si_point_to_normalized(s.position.x, s.position.y, 0.0, _frame);
  out.x = static_cast<float>(n.x);
  out.y = static_cast<float>(n.y);
  double psi = 0.0, u = 0.0;
  heading_speed(a, s, psi, u);
  out.heading = static_cast<float>(psi);
  out.speed = static_cast<float>(_units.to_normalized(u, dim::velocity, _frame));
  out.steer = static_cast<float>(a.steer_cmd);
  const vec3 f = dyn::rotate(s.rotation, {1.0, 0.0, 0.0});
  const vec3 l = dyn::rotate(s.rotation, {0.0, 1.0, 0.0});
  out.pitch = static_cast<float>(det::asin(clamp(f.z, -1.0, 1.0)));
  out.roll = static_cast<float>(det::asin(clamp(l.z, -1.0, 1.0)));
  return out;
}

cvc::nav::plant_limits dyn_boat_plant::limits(int i) const {
  const agent &a = _agents.at(static_cast<std::size_t>(i));
  const boat_desc &b = a.desc.boat;
  const nav_plant_options &o = _o.plant;
  const double m = a.props.mass, g = 9.80665;
  cvc::nav::plant_limits l;
  l.wheelbase = static_cast<float>(_units.to_normalized(a.L, dim::length, _frame));
  l.max_steer = static_cast<float>(a.max_steer);
  l.steer_rate = static_cast<float>(a.max_steer / std::max(0.05, b.actuator_time_constant));
  l.a_brake = static_cast<float>(_units.to_normalized(
      o.brake_margin * b.propeller.max_reverse_thrust / m, dim::acceleration, _frame));
  l.latency = static_cast<float>(o.latency + b.actuator_time_constant);
  if (!o.plan_with_vehicle)
    return l;
  l.a_max = static_cast<float>(
      _units.to_normalized(o.accel_margin * b.propeller.max_thrust / m, dim::acceleration, _frame));
  l.vmax = static_cast<float>(_units.to_normalized(a.desc.max_speed, dim::velocity, _frame));
  const double a_lat =
      std::min(o.lateral_g * g, a.desc.max_speed * a.desc.max_speed / a.desc.turn_radius);
  l.a_lat = static_cast<float>(_units.to_normalized(a_lat, dim::acceleration, _frame));
  l.half_width = static_cast<float>(_units.to_normalized(a.half_width, dim::length, _frame));
  l.front = static_cast<float>(_units.to_normalized(a.front, dim::length, _frame));
  l.rear = static_cast<float>(_units.to_normalized(a.rear, dim::length, _frame));
  return l;
}

void dyn_boat_plant::command(int i, float steer, float accel) {
  agent &a = _agents.at(static_cast<std::size_t>(i));
  a.steer_cmd = clamp(static_cast<double>(steer), -a.max_steer, a.max_steer);
  a.accel_cmd = _units.from_normalized(accel, dim::acceleration, _frame);
}

void dyn_boat_plant::drive(agent &a, double h) {
  world &w = *_o.physics;
  const physics::boat_state bs = w.boat_state(a.boat);
  const body_state &s = bs.body;
  const boat_desc &b = a.desc.boat;
  // The step's start, for the books: the hull, its energy and the water at its propeller and
  // rudder, where the boat's step will read them.
  step_start &st = a.start;
  st.s = s;
  st.com = com_of(a, s);
  st.kinetic = kinetic_of(a, s);
  double psi = 0.0;
  heading_speed(a, s, psi, st.u);
  st.prop = s.position + dyn::rotate(s.rotation, b.propeller.position);
  st.rudder = s.position + dyn::rotate(s.rotation, b.rudder.position);
  st.water_prop = w.water_at(st.prop).velocity;
  st.water_rudder = w.water_at(st.rudder).velocity;

  const double m = a.props.mass;
  double throttle = 0.0, rudder = 0.0;
  if (!_o.driver) {
    const double rated =
        a.accel_cmd >= 0.0 ? b.propeller.max_thrust : b.propeller.max_reverse_thrust;
    throttle = rated > 0.0 ? clamp(a.accel_cmd * m / rated, -1.0, 1.0) : 0.0;
    rudder = clamp(det::tan(a.steer_cmd) / a.tan_max, -1.0, 1.0);
  } else {
    // The throttle: nav's acceleration as a thrust, over what the propeller gives at this speed.
    driver_state &d = a.driver;
    const double want = a.accel_cmd, u = st.u;
    double accel = 0.0, e = 0.0;
    const bool hold = std::fabs(want) <= kDeadband && std::fabs(u) < kHoldSpeed;
    if (hold) {
      d.trim = 0.0;
      accel = -u / std::max(kHoldTau, 2.0 * b.actuator_time_constant);
    } else {
      // Unlike a car's brakes and drive, the water holds the boat back whichever way nav asks:
      // the trim carries on when the command changes sign.
      e = want - d.got;
      accel = want + kGain * e + d.trim;
    }
    const double thrust = m * accel;
    const double sign = thrust >= 0.0 ? 1.0 : -1.0;
    const double rated = sign > 0.0 ? b.propeller.max_thrust : b.propeller.max_reverse_thrust;
    double falloff = 1.0;
    if (b.propeller.max_inflow_speed > 0.0)
      falloff = clamp(1.0 - sign * bs.speed / b.propeller.max_inflow_speed, kFalloffFloor, 1.0);
    const double raw = rated > 0.0 ? thrust / (rated * falloff) : 0.0;
    throttle = clamp(raw, -1.0, 1.0);
    if (!hold && !((raw >= 1.0 && e > 0.0) || (raw <= -1.0 && e < 0.0))) {
      const double most = std::max(b.propeller.max_thrust, b.propeller.max_reverse_thrust) / m;
      d.trim = clamp(d.trim + kTrim * e * h, -most, most);
    }
    // The rudder: the bicycle's yaw rate, held as a curvature.
    const double kappa = det::tan(a.steer_cmd) / a.L;
    const double r_cmd = u * kappa;
    const double uf = u >= 0.0 ? std::max(u, kSpeedFloor) : std::min(u, -kSpeedFloor);
    const double ek = (r_cmd - bs.yaw_rate) / uf;
    const double R = a.desc.turn_radius;
    const double rraw = kappa * R + kYawGain * ek * R + d.yaw_trim;
    rudder = clamp(rraw, -1.0, 1.0);
    if (!((rraw >= 1.0 && ek > 0.0) || (rraw <= -1.0 && ek < 0.0)))
      d.yaw_trim = clamp(d.yaw_trim + kYawTrim * ek * R * h, -1.0, 1.0);
  }
  w.set_boat_controls(a.boat, {throttle, rudder});
  st.on = true;
}

void dyn_boat_plant::book(agent &a, double h) {
  world &w = *_o.physics;
  const step_start &st = a.start;
  a.now = w.boat_state(a.boat);
  const body_state &s0 = st.s, &s1 = a.now.body;
  const double m = a.props.mass;
  // Each force's work at the step's mean velocity, its torque's about the centre of mass where
  // the step began.
  const vec3 vm = (s0.linear_velocity + s1.linear_velocity) * 0.5;
  const vec3 wm = (s0.angular_velocity + s1.angular_velocity) * 0.5;
  const auto at = [&](const vec3 &p) { return vm + cross(wm, p - st.com); };
  dyn_boat_books d;
  // The thrust: T u_a through the water, and the water's own motion's share.
  const vec3 &ft = a.now.thrust_force;
  const double through = dot(ft, at(st.prop) - st.water_prop) * h;
  (through >= 0.0 ? d.drive : d.brake) = std::fabs(through);
  d.water += dot(ft, st.water_prop) * h;
  // The rudder: its force against the water past it, and the water's share.
  const vec3 &fr = a.now.rudder_force;
  d.rudder = dot(fr, st.water_rudder - at(st.rudder)) * h;
  d.water += dot(fr, st.water_rudder) * h;
  // The hull: buoyancy, resistance, damping, slamming.
  const field_load fl = w.field_load_on(a.hull);
  d.water += (dot(fl.force, vm) + dot(fl.torque, wm)) * h;
  // Gravity along the centre of mass's fall.
  const vec3 com1 = com_of(a, s1);
  d.gravity = m * dot(w.config().gravity, com1 - st.com);
  // The body's own damping, at the mean velocity.
  if (a.props.linear_damping > 0.0 || a.props.angular_damping > 0.0) {
    const vec3 wb = unrotate(s1.rotation, wm);
    d.damping = (a.props.linear_damping * m * dot(vm, vm) +
                 a.props.angular_damping * dot(wb, mat_mul(a.props.I, wb))) *
                h;
  }
  const double kinetic = kinetic_of(a, s1);
  d.residual =
      (kinetic - st.kinetic) - (d.drive - d.brake - d.rudder - d.damping + d.water + d.gravity);
  add(a.tick, d);
  a.tick.kinetic = kinetic;
  // What the driver measures: the acceleration along the heading the step gave.
  double psi = 0.0, u1 = 0.0;
  heading_speed(a, s1, psi, u1);
  a.driver.got = (u1 - st.u) / h;
}

void dyn_boat_plant::count_contacts() {
  if (!_o.physics || _agents.empty())
    return;
  world &w = *_o.physics;
  std::map<std::uint32_t, std::size_t> of_hull;
  for (std::size_t i = 0; i < _agents.size(); ++i)
    of_hull[_agents[i].hull.value] = i;
  std::map<std::uint32_t, bool> fixed; // a body without mass: static geometry
  const auto is_fixed = [&](body_id b) {
    const auto it = fixed.find(b.value);
    if (it != fixed.end())
      return it->second;
    bool f = false;
    try {
      f = !(w.get_body(b).mass > 0.0);
    } catch (const std::exception &) {
    }
    fixed[b.value] = f;
    return f;
  };
  for (const contact_event &e : w.contacts()) {
    if (e.type == contact_event::kind::removed)
      continue;
    const auto ha = of_hull.find(e.body_a.value), hb = of_hull.find(e.body_b.value);
    const auto one = [&](decltype(ha) self, decltype(ha) other, body_id them) {
      if (self == of_hull.end())
        return;
      agent &a = _agents[self->second];
      if (other != of_hull.end()) {
        if (a.touched < 0 || e.depth > a.touch_depth) {
          a.touched = static_cast<int>(other->second);
          a.touch_depth = e.depth;
        }
      } else if (e.type == contact_event::kind::added && is_fixed(them)) {
        ++a.statics_tick;
      }
    };
    one(ha, hb, e.body_b);
    one(hb, ha, e.body_a);
  }
}

void dyn_boat_plant::begin_tick() {
  for (agent &a : _agents) {
    const body_state &s = a.now.body;
    double psi = 0.0, u = 0.0;
    heading_speed(a, s, psi, u);
    a.vwx0 = s.linear_velocity.x;
    a.vwy0 = s.linear_velocity.y;
    a.psi0 = psi;
    a.tick = dyn_boat_books{};
    a.tick.kinetic = a.total.kinetic;
    a.statics_tick = 0;
    a.touched = -1;
    a.touch_depth = 0.0;
  }
}

void dyn_boat_plant::end_tick() {
  for (agent &a : _agents) {
    add(a.total, a.tick);
    a.total.kinetic = a.tick.kinetic;
    measure(a, true);
  }
}

void dyn_boat_plant::before_world_step() {
  need(!_o.step_world && _o.physics != nullptr,
       "before_world_step: the plant steps its own world (options.step_world), or it has none");
  const double h = _dt / _nsub;
  for (agent &a : _agents)
    drive(a, h);
}

void dyn_boat_plant::after_world_step() {
  need(!_o.step_world && _o.physics != nullptr,
       "after_world_step: the plant steps its own world (options.step_world), or it has none");
  const double h = _dt / _nsub;
  count_contacts();
  for (agent &a : _agents) {
    if (!a.start.on) // added since the step began
      continue;
    a.start.on = false;
    book(a, h);
  }
}

void dyn_boat_plant::advance() {
  if (!_o.physics)
    return; // the world is gone: nothing moves
  if (!_o.step_world) {
    // The host steps the world: end the tick its steps ran, begin the next.
    if (_ticking)
      end_tick();
    begin_tick();
    _ticking = true;
    return;
  }
  world &w = *_o.physics;
  const double h = _dt / _nsub;
  begin_tick();
  for (int k = 0; k < _nsub; ++k) {
    for (agent &a : _agents)
      drive(a, h);
    w.step();
    count_contacts();
    for (agent &a : _agents) {
      a.start.on = false;
      book(a, h);
    }
  }
  end_tick();
}

double dyn_boat_plant::energy_step(int i) const {
  return _agents.at(static_cast<std::size_t>(i)).tick.drive;
}

int dyn_boat_plant::static_contacts_step(int i) const {
  return _agents.at(static_cast<std::size_t>(i)).statics_tick;
}

cvc::nav::plant_sample dyn_boat_plant::sample(int i) const {
  return _agents.at(static_cast<std::size_t>(i)).sample;
}

void dyn_boat_plant::measure(agent &a, bool tick) const {
  const body_state &b = a.now.body;
  cvc::nav::plant_sample s;
  double psi = 0.0, u = 0.0;
  heading_speed(a, b, psi, u);
  const double c = det::cos(psi), sn = det::sin(psi);
  const vec3 &v = b.linear_velocity;
  s.vx = c * v.x + sn * v.y;
  s.vy = -sn * v.x + c * v.y;
  s.r = b.angular_velocity.z;
  s.steer = a.steer_cmd;
  s.max_steer = a.max_steer;
  s.steer_cmd = a.steer_cmd;
  s.accel_cmd = a.accel_cmd;
  if (tick && _dt > 0.0) {
    // The centre of mass's horizontal velocity's change over the tick, across the mean heading
    // (plant_log's and nav_plant's).
    const double dvx = v.x - a.vwx0, dvy = v.y - a.vwy0;
    const double pm = a.psi0 + 0.5 * dyn::wrap(psi - a.psi0);
    const double cm = det::cos(pm), sm = det::sin(pm);
    s.a_lat = (-dvx * sm + dvy * cm) / _dt;
    s.a_lon = (dvx * cm + dvy * sm) / _dt;
  }
  dyn::attitude_of(b.rotation, s);
  s.roll_rate = dot(b.angular_velocity, dyn::rotate(b.rotation, {1.0, 0.0, 0.0}));
  s.wheels = s.wheels_down = 0;
  s.contacts = tick ? a.statics_tick : 0;
  s.vehicle_contact = tick ? a.touched : -1;
  // A hull: no wheels and no load transfer.
  s.measured = cvc::nav::plant_sample::has_attitude | cvc::nav::plant_sample::has_roll_rate |
               cvc::nav::plant_sample::has_contacts | cvc::nav::plant_sample::has_vehicle_contact;
  a.sample = s;
}

} // namespace physics
} // namespace cvc
