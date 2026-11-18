/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "mujoco_engine.h"

#include <cmath>
#include <cvc/core/exception.h>
#include <cvc/geometry/height_field.h>
#include <cvc/physics/flow_velocity.h>
#include <cvc/physics/mujoco_robot.h>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace cvc {
namespace physics {

namespace {

// The engine throws std exceptions; the facade's contract is cvc ones.
template <class F> auto guarded(F &&f) -> decltype(f()) {
  try {
    return f();
  } catch (const cvc::exception &) {
    throw;
  } catch (const std::exception &e) {
    throw cvc::physics_error(std::string("mujoco_robot: ") + e.what());
  }
}

bool finite(const vec3 &v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

void validate(const mujoco_robot_options &o) {
  if (o.model.empty())
    throw cvc::physics_error("mujoco_robot: no model given");
  if (!std::isfinite(o.timestep) || o.timestep < 0.0)
    throw cvc::physics_error("mujoco_robot: timestep must be finite and >= 0");
  if (!std::isfinite(o.x) || !std::isfinite(o.y) || !std::isfinite(o.yaw))
    throw cvc::physics_error("mujoco_robot: the start must be finite");
  if (!std::isfinite(o.kp) || !std::isfinite(o.kd) || o.kp < 0.0 || o.kd < 0.0)
    throw cvc::physics_error("mujoco_robot: kp and kd must be finite and >= 0");
  if (o.terrain && o.floor.empty())
    throw cvc::physics_error("mujoco_robot: a terrain needs the floor geom it replaces");
}

#ifdef CVC_WITH_MUJOCO
// The height field as the engine takes it; and the ground under the start.
detail::mujoco_grid grid_of(const cvc::height_field &f) {
  if (f.rows() < 2 || f.cols() < 2)
    throw cvc::physics_error("mujoco_robot: the terrain needs at least 2 x 2 samples");
  detail::mujoco_grid g;
  g.rows = f.rows();
  g.cols = f.cols();
  g.x0 = f.x0();
  g.y0 = f.y0();
  g.dx = f.dx();
  g.dy = f.dy();
  g.heights = f.heights();
  for (float h : g.heights)
    if (!std::isfinite(h))
      throw cvc::physics_error("mujoco_robot: the terrain has holes (NaN), which MuJoCo's height "
                               "fields cannot have");
  return g;
}
#endif

} // namespace

bool mujoco_robot::available() {
#ifdef CVC_WITH_MUJOCO
  return true;
#else
  return false;
#endif
}

std::string mujoco_robot::version() {
#ifdef CVC_WITH_MUJOCO
  return detail::mujoco_version();
#else
  return std::string();
#endif
}

mujoco_robot::mujoco_robot(const mujoco_robot_options &o) {
#ifdef CVC_WITH_MUJOCO
  validate(o);
  detail::mujoco_ground ground;
  ground.point = {o.x, o.y, 0.0};
  detail::mujoco_grid grid;
  if (o.terrain) {
    grid = grid_of(*o.terrain);
    const double z = o.terrain->height(o.x, o.y);
    if (!std::isfinite(z))
      throw cvc::physics_error("mujoco_robot: the start (x, y) is off the terrain");
    const cvc::vector_t n = o.terrain->normal(o.x, o.y);
    ground.point.z = z;
    ground.normal = {n[0], n[1], n[2]};
  }
  _e = guarded([&] { return detail::make_mujoco_engine(o, o.terrain ? &grid : nullptr, ground); });
#else
  validate(o);
  throw cvc::unsupported_exception("cvc::physics::mujoco_robot: this libcvc was built without "
                                   "MuJoCo (CVC_WITH_MUJOCO)");
#endif
}

mujoco_robot::~mujoco_robot() = default;
mujoco_robot::mujoco_robot(mujoco_robot &&) noexcept = default;
mujoco_robot &mujoco_robot::operator=(mujoco_robot &&) noexcept = default;

namespace {
void check_index(int i, int n, const char *what) {
  if (i < 0 || i >= n)
    throw cvc::physics_error(std::string("mujoco_robot: no ") + what + " " + std::to_string(i));
}
} // namespace

int mujoco_robot::nq() const { return _e->nq(); }
int mujoco_robot::nv() const { return _e->nv(); }
int mujoco_robot::actuators() const { return _e->actuators(); }
int mujoco_robot::bodies() const { return _e->bodies(); }
int mujoco_robot::geoms() const { return _e->geoms(); }
int mujoco_robot::actuator(const std::string &name) const { return _e->actuator(name); }
int mujoco_robot::body(const std::string &name) const { return _e->body(name); }
int mujoco_robot::geom(const std::string &name) const { return _e->geom(name); }
std::string mujoco_robot::actuator_name(int i) const {
  check_index(i, actuators(), "actuator");
  return _e->actuator_name(i);
}
std::string mujoco_robot::body_name(int b) const {
  check_index(b, bodies(), "body");
  return _e->body_name(b);
}
std::string mujoco_robot::geom_name(int g) const {
  check_index(g, geoms(), "geom");
  return _e->geom_name(g);
}
int mujoco_robot::geom_body(int g) const {
  check_index(g, geoms(), "geom");
  return _e->geom_body(g);
}
int mujoco_robot::terrain_geom() const { return _e->terrain_geom(); }
int mujoco_robot::actuator_joint(int i) const {
  check_index(i, actuators(), "actuator");
  return _e->actuator_joint(i);
}
int mujoco_robot::actuator_qpos(int i) const {
  check_index(i, actuators(), "actuator");
  return _e->actuator_qpos(i);
}
int mujoco_robot::actuator_qvel(int i) const {
  check_index(i, actuators(), "actuator");
  return _e->actuator_qvel(i);
}
double mujoco_robot::force_min(int i) const {
  check_index(i, actuators(), "actuator");
  return _e->force_min(i);
}
double mujoco_robot::force_max(int i) const {
  check_index(i, actuators(), "actuator");
  return _e->force_max(i);
}
double mujoco_robot::mass() const { return _e->mass(); }
double mujoco_robot::timestep() const { return _e->timestep(); }
double mujoco_robot::time() const { return _e->time(); }

void mujoco_robot::set_mode(int i, actuator_mode m) {
  check_index(i, actuators(), "actuator");
  _e->set_mode(i, m);
}
void mujoco_robot::set_mode(actuator_mode m) {
  for (int i = 0; i < actuators(); ++i)
    _e->set_mode(i, m);
}
actuator_mode mujoco_robot::mode(int i) const {
  check_index(i, actuators(), "actuator");
  return _e->mode(i);
}
void mujoco_robot::set_gains(int i, double kp, double kd) {
  check_index(i, actuators(), "actuator");
  if (!std::isfinite(kp) || !std::isfinite(kd) || kp < 0.0 || kd < 0.0)
    throw cvc::physics_error("mujoco_robot: kp and kd must be finite and >= 0");
  _e->set_gains(i, kp, kd);
}
double mujoco_robot::kp(int i) const {
  check_index(i, actuators(), "actuator");
  return _e->kp(i);
}
double mujoco_robot::kd(int i) const {
  check_index(i, actuators(), "actuator");
  return _e->kd(i);
}
void mujoco_robot::command(int i, double u) {
  check_index(i, actuators(), "actuator");
  if (!std::isfinite(u))
    throw cvc::physics_error("mujoco_robot: a command must be finite");
  _e->command(i, u);
}
void mujoco_robot::command(const std::vector<double> &u) {
  if (static_cast<int>(u.size()) != actuators())
    throw cvc::physics_error("mujoco_robot: " + std::to_string(u.size()) + " commands for " +
                             std::to_string(actuators()) + " actuators");
  for (double v : u)
    if (!std::isfinite(v))
      throw cvc::physics_error("mujoco_robot: a command must be finite");
  for (int i = 0; i < actuators(); ++i)
    _e->command(i, u[static_cast<std::size_t>(i)]);
}
std::vector<double> mujoco_robot::commands() const { return _e->commands(); }
void mujoco_robot::hold() { _e->hold(); }

void mujoco_robot::apply_wrench(int b, const vec3 &force, const vec3 &torque) {
  check_index(b, bodies(), "body");
  if (b == 0)
    throw cvc::physics_error("mujoco_robot: the world body cannot be pushed");
  if (!finite(force) || !finite(torque))
    throw cvc::physics_error("mujoco_robot: a wrench must be finite");
  _e->apply_wrench(b, force, torque);
}
void mujoco_robot::apply_force_at(int b, const vec3 &force, const vec3 &point) {
  check_index(b, bodies(), "body");
  if (!finite(point))
    throw cvc::physics_error("mujoco_robot: the point must be finite");
  const vec3 r = point - _e->body_com(b);
  apply_wrench(b, force,
               {r.y * force.z - r.z * force.y, r.z * force.x - r.x * force.z,
                r.x * force.y - r.y * force.x});
}
void mujoco_robot::clear_wrenches() { _e->clear_wrenches(); }

void mujoco_robot::set_wind(std::shared_ptr<const flow_velocity> air, double time_offset,
                            double drag_coefficient, double density) {
  if (!std::isfinite(time_offset) || !std::isfinite(drag_coefficient) || drag_coefficient < 0.0 ||
      !std::isfinite(density) || !(density > 0.0))
    throw cvc::physics_error("mujoco_robot: the wind's time offset and C_d must be finite, C_d >= "
                             "0, and its density > 0");
  if (!air) {
    _e->set_air({});
    return;
  }
  const double half_rho_cd = 0.5 * density * drag_coefficient;
  _e->set_air(
      [air, time_offset, half_rho_cd](double t, const std::vector<detail::mujoco_air_geom> &geoms,
                                      const std::vector<vec3> &coms, std::vector<double> &wrench) {
        const double at = t + time_offset;
        // The flow's reference velocity once per body, at its centre of mass (the field step's way:
        // O(bodies)), and its profile at each geom's height.
        std::vector<vec3> ref(coms.size());
        std::vector<char> have(coms.size(), 0);
        const double floor = air->base();
        for (const detail::mujoco_air_geom &g : geoms) {
          const std::size_t b = static_cast<std::size_t>(g.body);
          if (b >= coms.size() || !(g.centre.z > floor))
            continue;
          const double prof = air->profile(g.centre.z);
          if (!(prof > 0.0))
            continue;
          if (!have[b]) {
            ref[b] = air->reference(coms[b], at);
            have[b] = 1;
          }
          const vec3 u = ref[b] * prof - g.velocity;
          const double s = std::sqrt(dot(u, u));
          if (!(s > 0.0))
            continue;
          const vec3 n = u * (1.0 / s);
          // The box's area across u: each pair of faces, its area times the cosine to u.
          const double area = 4.0 * (g.half.y * g.half.z * std::fabs(dot(n, g.axis[0])) +
                                     g.half.x * g.half.z * std::fabs(dot(n, g.axis[1])) +
                                     g.half.x * g.half.y * std::fabs(dot(n, g.axis[2])));
          const vec3 f = u * (half_rho_cd * area * s);
          const vec3 r = g.centre - coms[b];
          double *w = wrench.data() + 6 * b;
          w[0] += f.x;
          w[1] += f.y;
          w[2] += f.z;
          w[3] += r.y * f.z - r.z * f.y;
          w[4] += r.z * f.x - r.x * f.z;
          w[5] += r.x * f.y - r.y * f.x;
        }
      });
}

vec3 mujoco_robot::wind_force() const { return _e->air_force(); }
vec3 mujoco_robot::wind_torque() const { return _e->air_torque(); }

void mujoco_robot::step() {
  guarded([&] { _e->step(); });
}
void mujoco_robot::step(int n) {
  for (int k = 0; k < n; ++k)
    step();
}

std::vector<double> mujoco_robot::qpos() const { return _e->qpos(); }
std::vector<double> mujoco_robot::qvel() const { return _e->qvel(); }
vec3 mujoco_robot::base_position() const { return _e->base_position(); }
quat mujoco_robot::base_rotation() const { return _e->base_rotation(); }
vec3 mujoco_robot::base_velocity() const { return _e->base_velocity(); }
vec3 mujoco_robot::base_spin() const { return _e->base_spin(); }
double mujoco_robot::base_up_z() const {
  const quat q = base_rotation();
  return 1.0 - 2.0 * (q.x * q.x + q.y * q.y); // R(q)'s (2, 2) entry
}
vec3 mujoco_robot::body_position(int b) const {
  check_index(b, bodies(), "body");
  return _e->body_position(b);
}
quat mujoco_robot::body_rotation(int b) const {
  check_index(b, bodies(), "body");
  return _e->body_rotation(b);
}

std::vector<double> mujoco_robot::actuator_forces() const { return _e->actuator_forces(); }
std::vector<mujoco_contact> mujoco_robot::contacts() const { return _e->contacts(); }

mujoco_energy mujoco_robot::energy() const {
  if (!_e->energy_on())
    throw std::logic_error("mujoco_robot: built with energy off (mujoco_robot_options::energy)");
  return _e->energy();
}
mujoco_books mujoco_robot::books() const {
  if (!_e->energy_on())
    throw std::logic_error("mujoco_robot: built with energy off (mujoco_robot_options::energy)");
  return _e->books();
}

mujoco_snapshot mujoco_robot::save() const { return _e->save(); }
void mujoco_robot::restore(const mujoco_snapshot &s) {
  guarded([&] { _e->restore(s); });
}
void mujoco_robot::reset() {
  guarded([&] { _e->reset(); });
}

mujoco_trial_result run_mujoco_trial(const mujoco_trial_options &t) {
  if (!(t.duration > 0.0) || !(t.settle >= 0.0) || !(t.push_duration >= 0.0) ||
      !std::isfinite(t.duration) || !std::isfinite(t.settle) || !std::isfinite(t.push_duration))
    throw cvc::physics_error("run_mujoco_trial: duration > 0, settle and push_duration >= 0");
  if (!(t.cell > 0.0) || !(t.half_extent > t.cell) || !std::isfinite(t.half_extent))
    throw cvc::physics_error("run_mujoco_trial: half_extent must exceed cell, and cell be > 0");
  if (!(std::fabs(t.slope_deg) < 80.0))
    throw cvc::physics_error("run_mujoco_trial: |slope_deg| must be under 80");
  if (!finite(t.push))
    throw cvc::physics_error("run_mujoco_trial: the push must be finite");

  const double pi = std::acos(-1.0);
  const int n = static_cast<int>(std::lround(2.0 * t.half_extent / t.cell)) + 1;
  const double rise = std::tan(t.slope_deg * pi / 180.0);
  std::vector<float> heights(static_cast<std::size_t>(n) * static_cast<std::size_t>(n));
  for (int r = 0; r < n; ++r)
    for (int c = 0; c < n; ++c)
      heights[static_cast<std::size_t>(r) * n + c] =
          static_cast<float>(rise * (-t.half_extent + c * t.cell));
  auto field = std::make_shared<cvc::height_field>(n, n, -t.half_extent, -t.half_extent, t.cell,
                                                   t.cell, std::move(heights));

  mujoco_robot_options o;
  o.model = t.model;
  o.terrain = field;
  o.timestep = t.timestep;
  o.yaw = t.yaw;
  o.kp = t.kp;
  o.kd = t.kd;
  mujoco_robot robot(o);
  const int base = robot.body(t.base);
  if (base < 0)
    throw cvc::physics_error("run_mujoco_trial: no body named '" + t.base + "'");
  std::set<int> feet;
  for (const std::string &f : t.feet) {
    const int g = robot.geom(f);
    if (g < 0)
      throw cvc::physics_error("run_mujoco_trial: no geom named '" + f + "'");
    feet.insert(g);
  }
  const int terrain = robot.terrain_geom();

  const double h = robot.timestep();
  const int settle = static_cast<int>(std::lround(t.settle / h));
  const int push = static_cast<int>(std::lround(t.push_duration / h));
  const int total = std::max(static_cast<int>(std::lround(t.duration / h)), settle + push);
  const bool pushes = t.push.x != 0.0 || t.push.y != 0.0 || t.push.z != 0.0;

  auto height = [&] {
    const vec3 p = robot.base_position();
    return p.z - field->height(p.x, p.y);
  };
  auto speed = [&] {
    const vec3 v = robot.base_velocity();
    return std::sqrt(dot(v, v));
  };

  mujoco_trial_result out;
  vec3 at_push;
  for (int k = 0; k <= total; ++k) {
    if (k == settle) {
      out.settled_height = out.min_height = height();
      out.settled_up_z = out.min_up_z = robot.base_up_z();
      out.settled_speed = speed();
      at_push = robot.base_position();
      if (pushes && push > 0)
        robot.apply_wrench(base, t.push);
    }
    if (k == settle + push)
      robot.clear_wrenches();
    if (k >= settle) {
      const vec3 p = robot.base_position();
      const double moved = std::hypot(p.x - at_push.x, p.y - at_push.y);
      out.min_height = std::min(out.min_height, height());
      out.min_up_z = std::min(out.min_up_z, robot.base_up_z());
      out.max_displacement = std::max(out.max_displacement, moved);
      out.end_displacement = moved;
    }
    if (k == total)
      break;
    robot.step();
    ++out.steps;
    for (const mujoco_contact &c : robot.contacts()) {
      const bool t0 = c.geom[0] == terrain, t1 = c.geom[1] == terrain;
      if (t0 != t1 && !feet.count(t0 ? c.geom[1] : c.geom[0])) {
        ++out.body_contact_steps;
        break;
      }
    }
  }
  out.end_height = height();
  out.end_up_z = robot.base_up_z();
  out.end_speed = speed();
  out.fell = out.min_up_z < 0.5 || out.body_contact_steps > 0;
  out.recovered = !out.fell && std::fabs(out.end_height - out.settled_height) < 0.02 &&
                  std::fabs(out.end_up_z - out.settled_up_z) < 0.02 && out.end_speed < 0.05;
  out.books = robot.books();
  return out;
}

std::string describe(const mujoco_robot &r) {
  std::ostringstream s;
  s << mujoco_robot::version() << "\n"
    << "nq " << r.nq() << ", nv " << r.nv() << ", " << r.actuators() << " actuators, " << r.bodies()
    << " bodies, " << r.geoms() << " geoms\n"
    << "mass " << r.mass() << " kg, timestep " << r.timestep() << " s\n"
    << "actuators (joint angle now, force range N m):\n";
  const std::vector<double> q = r.qpos();
  for (int i = 0; i < r.actuators(); ++i)
    s << "  " << i << " " << r.actuator_name(i) << ": q "
      << q[static_cast<std::size_t>(r.actuator_qpos(i))] << ", force [" << r.force_min(i) << ", "
      << r.force_max(i) << "]\n";
  s << "bodies:";
  for (int b = 0; b < r.bodies(); ++b)
    s << " " << r.body_name(b);
  s << "\n";
  return s.str();
}

std::string to_json(const mujoco_trial_result &t) {
  std::ostringstream s;
  s << std::setprecision(9) << std::boolalpha;
  s << "{\"steps\": " << t.steps << ", \"fell\": " << t.fell << ", \"recovered\": " << t.recovered
    << ", \"settled\": {\"height\": " << t.settled_height << ", \"up_z\": " << t.settled_up_z
    << ", \"speed\": " << t.settled_speed << "}, \"end\": {\"height\": " << t.end_height
    << ", \"up_z\": " << t.end_up_z << ", \"speed\": " << t.end_speed
    << ", \"displacement\": " << t.end_displacement << "}, \"min_height\": " << t.min_height
    << ", \"min_up_z\": " << t.min_up_z << ", \"max_displacement\": " << t.max_displacement
    << ", \"body_contact_steps\": " << t.body_contact_steps;
  const mujoco_books &b = t.books;
  s << ", \"books\": {\"energy_start\": " << b.energy_start << ", \"energy_now\": " << b.energy_now
    << ", \"actuators\": " << b.actuators << ", \"external\": " << b.external
    << ", \"contacts\": " << b.contacts << ", \"joint_friction\": " << b.joint_friction
    << ", \"joint_limits\": " << b.joint_limits << ", \"joint_damping\": " << b.joint_damping
    << ", \"other\": " << b.other << ", \"residual\": " << b.residual() << "}}\n";
  return s.str();
}

} // namespace physics
} // namespace cvc
