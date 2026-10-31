/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "det_math.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cvc/core/exception.h>
#include <cvc/nav/coef_train.h>
#include <cvc/physics/jolt_rollout.h>
#include <type_traits>

namespace cvc {
namespace physics {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr char kMagic[8] = {'C', 'V', 'C', 'J', 'R', 'O', 'L', '1'};

vec3 cross(const vec3 &a, const vec3 &b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

vec3 rotate(const quat &q, const vec3 &v) {
  const vec3 u{q.x, q.y, q.z};
  const vec3 t = cross(u, v) * 2.0;
  return v + t * q.w + cross(u, t);
}

// The rearmost unsteered axle's x (nav_plant's rear axle).
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

double clamp1(double v) { return std::max(-1.0, std::min(1.0, v)); }

world_config rollout_config(int threads) {
  world_config c;
  c.backend = backend::jolt;
  c.threads = threads;
  // Every step a contact persists, so a vehicle against a wall counts on each tick it is.
  c.contacts = contact_reporting::all;
  return c;
}

// The scene's obstacles as rectangles in its normalized frame: each row's runs of occupied cells,
// merged with the same run in the rows below. Cells are the raster's nodes (training_scene's
// convention), so a run spans half a cell beyond its first and last node.
struct rect {
  double x0, y0, x1, y1;
};
std::vector<rect> scene_rects(const cvc::nav::training_scene &sc) {
  std::vector<rect> out;
  if (sc.rows < 2 || sc.cols < 2)
    return out;
  const double dx = (sc.max_x - sc.min_x) / (sc.cols - 1),
               dy = (sc.max_y - sc.min_y) / (sc.rows - 1);
  struct open_run {
    int c0, c1, r0;
  };
  std::vector<open_run> open, next;
  const auto close = [&](const open_run &o, int r1) {
    rect q;
    q.x0 = (sc.min_x + (o.c0 - 0.5) * dx - sc.cx) * sc.scale;
    q.x1 = (sc.min_x + (o.c1 + 0.5) * dx - sc.cx) * sc.scale;
    q.y0 = (sc.min_y + (o.r0 - 0.5) * dy - sc.cy) * sc.scale;
    q.y1 = (sc.min_y + (r1 + 0.5) * dy - sc.cy) * sc.scale;
    out.push_back(q);
  };
  for (int r = 0; r <= sc.rows; ++r) {
    next.clear();
    if (r < sc.rows) {
      const std::uint8_t *row = sc.occ.data() + static_cast<std::size_t>(r) * sc.cols;
      for (int c = 0; c < sc.cols;) {
        if (!row[c]) {
          ++c;
          continue;
        }
        int e = c;
        while (e + 1 < sc.cols && row[e + 1])
          ++e;
        const auto it = std::find_if(open.begin(), open.end(),
                                     [&](const open_run &o) { return o.c0 == c && o.c1 == e; });
        next.push_back({c, e, it != open.end() ? it->r0 : r});
        c = e + 1;
      }
    }
    for (const open_run &o : open)
      if (std::none_of(next.begin(), next.end(),
                       [&](const open_run &q) { return q.c0 == o.c0 && q.c1 == o.c1; }))
        close(o, r - 1);
    open.swap(next);
  }
  return out;
}

// The vehicle's body about its rear axle (body x forward): how far it reaches ahead and behind,
// and its half width, the chassis box and the wheels where they stand out of it (as
// nav_plant::limits).
struct body_reach {
  double ahead = 0.0, behind = 0.0, half_width = 0.0;
};
body_reach reach_of(const vehicle_desc &d) {
  double ahead = d.half_extents.x, behind = -d.half_extents.x, half_width = d.half_extents.y;
  for (const axle_desc &ax : d.axles) {
    ahead = std::max(ahead, ax.x + ax.wheel_radius);
    behind = std::min(behind, ax.x - ax.wheel_radius);
    half_width = std::max(half_width, ax.half_track + 0.5 * ax.wheel_width);
  }
  const double rear = rear_axle_x(d);
  return {ahead - rear, rear - behind, half_width};
}

// Whether the body, its rear axle at (x, y) (normalized) heading psi, clears every wall of the
// scene by `clear` (normalized): the SDF at the raster node nearest each of 7 x 3 points over
// the body. Off the raster is clear.
bool body_fits(const cvc::nav::training_scene &sc, const body_reach &b, double k, double x,
               double y, double psi, double clear) {
  const double c = det::cos(psi), s = det::sin(psi);
  const double dx = (sc.max_x - sc.min_x) / (sc.cols - 1),
               dy = (sc.max_y - sc.min_y) / (sc.rows - 1);
  for (int i = 0; i < 7; ++i)
    for (int j = 0; j < 3; ++j) {
      const double bx = (-b.behind + (b.ahead + b.behind) * i / 6.0) * k;
      const double by = (j - 1) * b.half_width * k;
      const double wx = (x + c * bx - s * by) / sc.scale + sc.cx;
      const double wy = (y + s * bx + c * by) / sc.scale + sc.cy;
      const long col = std::lround((wx - sc.min_x) / dx), row = std::lround((wy - sc.min_y) / dy);
      if (col < 0 || row < 0 || col >= sc.cols || row >= sc.rows)
        continue;
      const std::size_t cell = static_cast<std::size_t>(row) * sc.cols + col;
      if (sc.occ[cell] || sc.field_data[cell] < clear)
        return false;
    }
  return true;
}

template <class T> void put(std::vector<std::uint8_t> &out, const T &v) {
  static_assert(std::is_trivially_copyable<T>::value, "plain values");
  const auto *b = reinterpret_cast<const std::uint8_t *>(&v);
  out.insert(out.end(), b, b + sizeof(T));
}
template <class T> T get(const std::vector<std::uint8_t> &in, std::size_t &pos) {
  if (in.size() - pos < sizeof(T))
    throw cvc::physics_error("jolt_rollout::restore: the state is truncated");
  T v;
  std::memcpy(&v, in.data() + pos, sizeof(T));
  pos += sizeof(T);
  return v;
}

} // namespace

jolt_rollout::jolt_rollout(const vehicle_desc &desc, double wheelbase, jolt_rollout_options options)
    : _desc(desc), _o(std::move(options)) {
  const double wb = desc.wheelbase();
  if (!(wb > 0.0))
    throw cvc::physics_error("jolt_rollout: the vehicle needs a wheelbase (a steered axle ahead "
                             "of an unsteered one)");
  if (!(wheelbase > 0.0) || !std::isfinite(wheelbase))
    throw cvc::physics_error("jolt_rollout: the normalized wheelbase must be finite and > 0");
  if (_o.settle_ticks < 0 || !(_o.margin >= 0.0) || !(_o.wall_height > 0.0))
    throw cvc::physics_error("jolt_rollout: settle_ticks and margin must be >= 0, wall_height > 0");
  _k = wheelbase / wb;
  // The height the chassis rests at on flat ground: one vehicle, settled.
  world w(rollout_config(0));
  body_desc g;
  g.shape = box{{50.0, 50.0, 0.5}};
  g.motion = motion_type::static_;
  g.position = {0.0, 0.0, -0.5};
  w.add_body(g);
  nav_plant_options po = _o.plant;
  po.metres_per_world_unit = 1.0;
  po.spawn_height = -1.0;
  nav_plant p(w, {_desc}, po);
  cvc::world_units::normalized_frame f;
  f.scale = 1.0;
  cvc::nav::plant_state start;
  p.attach(1, &start, f, 0.06);
  for (int t = 0; t < 60; ++t)
    p.advance();
  _spawn_height = w.vehicle_state(p.vehicle(0)).body.position.z;
}

jolt_rollout::~jolt_rollout() {
  _plant.reset(); // before its world
  _w.reset();
}

world &jolt_rollout::physics_world() {
  if (!_w)
    throw cvc::physics_error("jolt_rollout: no batch yet (reset() it)");
  return *_w;
}

nav_plant &jolt_rollout::plant() {
  if (!_plant)
    throw cvc::physics_error("jolt_rollout: no batch yet (reset() it)");
  return *_plant;
}

void jolt_rollout::reset(const cvc::nav::rollout_batch &b) {
  if (!b.scene)
    throw cvc::physics_error("jolt_rollout::reset: the batch needs its scene");
  if (b.n < 0 || (b.n > 0 && (!b.o || !b.psi)))
    throw cvc::physics_error("jolt_rollout::reset: needs n >= 0 starts");
  const cvc::nav::training_scene &sc = *b.scene;
  _veh = b.veh;
  _model = b.model;
  _plant.reset();
  _w = std::make_unique<world>(rollout_config(_o.threads));

  // The copies of the scene on a grid, `spacing` apart, centred on the origin.
  const double ex = (sc.max_x - sc.min_x) * sc.scale, ey = (sc.max_y - sc.min_y) * sc.scale;
  const double spacing = std::max(ex, ey) * (1.0 + _o.margin);
  const int cols = std::max(1, static_cast<int>(std::ceil(std::sqrt(static_cast<double>(b.n)))));
  const int rows = std::max(1, (b.n + cols - 1) / cols);
  const material_id ground =
      _w->add_material({"ground", b.mu, 0.0f, static_cast<float>(_desc.rolling_resistance)});
  body_desc g;
  g.shape = box{{0.5 * (cols + 1) * spacing / _k, 0.5 * (rows + 1) * spacing / _k, 0.5}};
  g.motion = motion_type::static_;
  g.position = {0.0, 0.0, -0.5};
  g.material = ground;
  _w->add_body(g);
  _agents.assign(static_cast<std::size_t>(b.n), agent{});
  for (int i = 0; i < b.n; ++i) {
    agent &a = _agents[static_cast<std::size_t>(i)];
    a.ox = ((i % cols) - 0.5 * (cols - 1)) * spacing;
    a.oy = ((i / cols) - 0.5 * (rows - 1)) * spacing;
  }
  const std::vector<rect> rects = _o.walls ? scene_rects(sc) : std::vector<rect>{};
  _wall_boxes = static_cast<int>(rects.size());
  for (const agent &a : _agents)
    for (const rect &r : rects) {
      body_desc w;
      w.shape = box{{0.5 * (r.x1 - r.x0) / _k, 0.5 * (r.y1 - r.y0) / _k, 0.5 * _o.wall_height}};
      w.motion = motion_type::static_;
      w.position = {(0.5 * (r.x0 + r.x1) + a.ox) / _k, (0.5 * (r.y0 + r.y1) + a.oy) / _k,
                    0.5 * _o.wall_height};
      w.material = ground;
      _w->add_body(w);
    }

  nav_plant_options po = _o.plant;
  po.metres_per_world_unit = 1.0;
  po.spawn_height = _spawn_height;
  po.ground_z = 0.0;
  po.ground_height = nullptr;
  po.step_world = true;
  _plant = std::make_unique<nav_plant>(*_w, std::vector<vehicle_desc>{_desc}, po);
  cvc::world_units::normalized_frame f;
  f.scale = _k;
  std::vector<cvc::nav::plant_state> starts(static_cast<std::size_t>(b.n));
  const body_reach reach = reach_of(_desc);
  for (int i = 0; i < b.n; ++i) {
    // The nearest heading, 15 degrees at a time either way, at which the body clears the walls.
    agent &a = _agents[static_cast<std::size_t>(i)];
    a.turn = 0.0;
    if (_o.walls && sc.rows > 1 && sc.cols > 1 && !sc.field_data.empty())
      for (int step = 0; step <= 24; ++step) {
        const double turn = (step % 2 == 1 ? 1.0 : -1.0) * ((step + 1) / 2) * kPi / 12.0;
        if (body_fits(sc, reach, _k, b.o[2 * i], b.o[2 * i + 1], b.psi[i] + turn,
                      _o.clearance_m * _k)) {
          a.turn = turn;
          break;
        }
      }
    starts[i].x = static_cast<float>(b.o[2 * i] + a.ox);
    starts[i].y = static_cast<float>(b.o[2 * i + 1] + a.oy);
    starts[i].heading = static_cast<float>(b.psi[i] + a.turn);
  }
  _plant->attach(b.n, starts.data(), f, b.veh.hdt);
  for (int t = 0; t < _o.settle_ticks; ++t)
    _plant->advance();
  for (int i = 0; i < b.n; ++i) {
    agent &a = _agents[static_cast<std::size_t>(i)];
    const body_state bs = _w->vehicle_state(_plant->vehicle(i)).body;
    a.z_rest = (bs.position + rotate(bs.rotation, _desc.center_of_mass)).z;
    a.psi = b.psi[i] + a.turn;
    a.a = 0.0f;
  }
  _steps = 0;
  _step_us = 0.0;
}

cvc::nav::diff::dyn4_state<float> jolt_rollout::read(int i, double psi_ref) const {
  const agent &ag = _agents.at(static_cast<std::size_t>(i));
  const vehicle_state vs = _w->vehicle_state(_plant->vehicle(i));
  const body_state &b = vs.body;
  const vec3 fwd = rotate(b.rotation, {1.0, 0.0, 0.0});
  const vec3 left = rotate(b.rotation, {0.0, 1.0, 0.0});
  const vec3 rear = b.position + rotate(b.rotation, {rear_axle_x(_desc), 0.0, 0.0});
  const vec3 com = b.position + rotate(b.rotation, _desc.center_of_mass);
  double h = det::atan2(fwd.y, fwd.x) - psi_ref;
  h -= 2.0 * kPi * std::floor((h + kPi) / (2.0 * kPi));
  const double psi = psi_ref + h;
  const double c = det::cos(psi), s = det::sin(psi);
  const vec3 &v = b.linear_velocity, &w = b.angular_velocity;
  cvc::nav::diff::dyn4_state<float> o;
  o.x = static_cast<float>(rear.x * _k - ag.ox);
  o.y = static_cast<float>(rear.y * _k - ag.oy);
  o.psi = static_cast<float>(psi);
  o.vx = static_cast<float>((v.x * c + v.y * s) * _k);
  o.vy = static_cast<float>((-v.x * s + v.y * c) * _k);
  o.r = static_cast<float>(w.z);
  o.z = static_cast<float>((com.z - ag.z_rest) * _k);
  o.zd = static_cast<float>(v.z * _k);
  o.phi = static_cast<float>(det::asin(clamp1(left.z)));
  o.phid = static_cast<float>(dot(w, fwd));
  o.th = static_cast<float>(det::asin(clamp1(-fwd.z)));
  o.thd = static_cast<float>(dot(w, left));
  o.delta = static_cast<float>(vs.steer);
  o.a = ag.a;
  return o;
}

cvc::nav::diff::dyn4_state<float> jolt_rollout::state(int i) const {
  if (!_plant)
    throw cvc::physics_error("jolt_rollout: no batch yet (reset() it)");
  return read(i, _agents.at(static_cast<std::size_t>(i)).psi);
}

void jolt_rollout::step(const float *delta_cmd, const float *a_cmd) {
  if (!_plant)
    throw cvc::physics_error("jolt_rollout: no batch yet (reset() it)");
  const auto t0 = std::chrono::steady_clock::now();
  const int n = size();
  std::vector<float> a1(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    a1[i] = cvc::nav::diff::dyn4_drive_actuator(_model, _veh, state(i), a_cmd[i]);
    _plant->command(i, delta_cmd[i], a_cmd[i]);
  }
  _plant->advance();
  for (int i = 0; i < n; ++i) {
    agent &a = _agents[static_cast<std::size_t>(i)];
    a.a = a1[i];
    a.psi = read(i, a.psi).psi;
  }
  ++_steps;
  _step_us +=
      std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
}

cvc::nav::plant_energy_lines jolt_rollout::lines(int i) const {
  cvc::nav::plant_energy_lines l;
  if (!_plant || _steps == 0)
    return l;
  const vehicle_energy &e = _plant->ledger_step(i);
  const double k2 = _k * _k;
  l.drawn = _plant->energy_step(i) * k2;
  l.port = (e.supplied + e.idle + e.drivetrain_split - e.drivetrain - e.brake) * k2;
  l.tire = e.tire * k2;
  l.drag = e.aero * k2;
  l.rolling = (e.rolling + e.medium) * k2;
  l.damper = (e.suspension + e.chassis_damping) * k2;
  l.stored = e.stored * k2;
  l.residual = e.residual * k2;
  l.contact = _plant->static_contacts_step(i) > 0 || _plant->vehicle_contact_step(i) >= 0;
  return l;
}

double jolt_rollout::step_us_per_vehicle() const {
  return _steps == 0 || _agents.empty()
             ? 0.0
             : _step_us / static_cast<double>(_steps) / static_cast<double>(_agents.size());
}

std::vector<std::uint8_t> jolt_rollout::save() const {
  if (!_plant)
    throw cvc::physics_error("jolt_rollout: no batch yet (reset() it)");
  std::vector<std::uint8_t> out(kMagic, kMagic + sizeof(kMagic));
  put(out, static_cast<std::uint64_t>(_agents.size()));
  put(out, _steps);
  for (const agent &a : _agents) {
    put(out, a.psi);
    put(out, a.a);
  }
  const std::vector<std::uint8_t> p = _plant->save();
  out.insert(out.end(), p.begin(), p.end());
  return out;
}

void jolt_rollout::restore(const std::vector<std::uint8_t> &state) {
  if (!_plant)
    throw cvc::physics_error("jolt_rollout: no batch yet (reset() it)");
  if (state.size() < sizeof(kMagic) || std::memcmp(state.data(), kMagic, sizeof(kMagic)) != 0)
    throw cvc::physics_error("jolt_rollout::restore: not a jolt_rollout state");
  std::size_t pos = sizeof(kMagic);
  if (get<std::uint64_t>(state, pos) != _agents.size())
    throw cvc::physics_error("jolt_rollout::restore: the state is of another batch");
  const auto steps = get<std::uint64_t>(state, pos);
  std::vector<agent> agents = _agents;
  for (agent &a : agents) {
    a.psi = get<double>(state, pos);
    a.a = get<float>(state, pos);
  }
  _plant->restore(
      std::vector<std::uint8_t>(state.begin() + static_cast<std::ptrdiff_t>(pos), state.end()));
  _agents = std::move(agents);
  _steps = steps;
}

} // namespace physics
} // namespace cvc
