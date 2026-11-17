/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_boat.h>
#include <cvc/core/app.h>
#include <cvc/core/exception.h>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/sim_world.h>
#include <cvc/nav/world_grid.h>
#include <cvc/physics/world.h>
#include <exception>
#include <sstream>

namespace cvc {
namespace ariadne {

namespace ph = cvc::physics;

namespace {

constexpr double kScale = 0.05;        // nav's normalized units per metre, as WorldVehicles'
constexpr double kNavDt = 0.05;        // nav's tick (s), rounded to whole physics steps
constexpr double kReach = 3.0;         // m from its goal that counts as there
constexpr double kKeelClearance = 0.3; // m of water under its draft its navigator keeps to
constexpr double kPi = 3.14159265358979323846;

std::string num(double v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.6g", v);
  return buf;
}

// "x,y" (world metres); false for anything else.
bool parse_xy(const std::string &s, double &x, double &y) {
  std::stringstream in(s);
  std::string a, b;
  if (!std::getline(in, a, ',') || !std::getline(in, b, ','))
    return false;
  try {
    x = std::stod(a);
    y = std::stod(b);
  } catch (const std::exception &) {
    return false;
  }
  return std::isfinite(x) && std::isfinite(y);
}

} // namespace

SceneBoatSpec scene_boat_spec(const SceneNode &n, const std::array<double, 16> &world,
                              const std::string &bind_prefix, std::vector<std::string> *warnings) {
  SceneBoatSpec s;
  s.node = n.id;
  const std::string root = n.props.str("keys").empty() ? n.id : n.props.str("keys");
  s.keys = resolve_bind(bind_prefix, root);
  s.water = n.props.str("water");
  s.preset = n.props.str("preset", "launch");
  try {
    s.desc = ph::dyn_boat_desc::preset(s.preset);
  } catch (const std::exception &) {
    if (warnings)
      warnings->push_back("ari: scene node '" + n.id + "' (boat): no boat preset '" + s.preset +
                          "' (launch, jonboat); a launch");
    s.preset = "launch";
    s.desc = ph::dyn_boat_desc::launch();
  }
  if (const double m = n.props.num("mass", 0.0); m > 0.0)
    s.desc.boat.hull.mass = m;
  if (const double v = n.props.num("max_speed", 0.0); v > 0.0)
    s.desc.max_speed = v;
  s.x = world[3];
  s.y = world[7];
  s.heading = std::atan2(world[4], world[0]);
  s.drive = n.props.str("drive", "keys") == "nav" ? "nav" : "keys";
  if (const Value *g = n.props.find("goal"); g && g->is_seq() && g->items.size() >= 2) {
    s.goal_x = g->items[0].as_double(s.x);
    s.goal_y = g->items[1].as_double(s.y);
    s.has_goal = true;
  }
  return s;
}

struct BoatRuntime::impl {
  cvc::app *app = nullptr;
  SceneBoatSpec spec;
  ph::world *world = nullptr;
  std::shared_ptr<const nav::world_grid> grid;
  std::unique_ptr<ph::dyn_boat_plant> plant;
  std::unique_ptr<cvc::nav::sim_world> sim;
  int every = 1;
  std::uint64_t steps = 0;
  bool stepping = false; // before_step() began a step that after_step() ends
  // What place() last failed with, so it waits for a new world or grid before trying again.
  const ph::world *tried_world = nullptr;
  const nav::world_grid *tried_grid = nullptr;
  std::size_t tried_fields = 0;
  std::string status = "waiting for its water";
  std::string drive = "keys";
  double gx = 0.0, gy = 0.0;
  std::string seen_goal;
  double half_length = 0.0, half_beam = 0.0, sheer = 0.0, bottom = 0.0, draft = 0.0;
  std::shared_ptr<const ph::body_desc> hull_desc;

  float nx(double x) const { return static_cast<float>((x - grid->center_x()) * kScale); }
  float ny(double y) const { return static_cast<float>((y - grid->center_y()) * kScale); }
  std::string key(const char *leaf) const { return spec.keys + "." + leaf; }
};

BoatRuntime::BoatRuntime(cvc::app *app, SceneBoatSpec spec) : _impl(std::make_unique<impl>()) {
  impl &d = *_impl;
  d.app = app;
  d.spec = std::move(spec);
  d.drive = d.spec.drive;
  d.gx = d.spec.has_goal ? d.spec.goal_x : d.spec.x;
  d.gy = d.spec.has_goal ? d.spec.goal_y : d.spec.y;
  const ph::hull_mesh &mesh = *d.spec.desc.boat.hull.hull;
  double lo[3] = {1e300, 1e300, 1e300}, hi[3] = {-1e300, -1e300, -1e300};
  for (const ph::vec3 &v : mesh.vertices) {
    const double p[3] = {v.x, v.y, v.z};
    for (int k = 0; k < 3; ++k) {
      lo[k] = std::min(lo[k], p[k]);
      hi[k] = std::max(hi[k], p[k]);
    }
  }
  d.half_length = 0.5 * (hi[0] - lo[0]);
  d.half_beam = std::max(std::fabs(lo[1]), std::fabs(hi[1]));
  d.sheer = hi[2];
  d.bottom = lo[2];
  d.draft = ph::dyn_boat_plant::draft(d.spec.desc.boat, d.spec.desc.boat.hull.mass, 1000.0);
  d.hull_desc = std::make_shared<const ph::body_desc>(d.spec.desc.boat.hull);
  if (d.app) {
    read_or_seed<std::string>(*d.app, d.key("drive"), d.spec.drive);
    read_or_seed<double>(*d.app, d.key("throttle"), 0.0);
    read_or_seed<double>(*d.app, d.key("rudder"), 0.0);
    read_or_seed<std::string>(*d.app, d.key("goal"),
                              d.spec.has_goal ? num(d.spec.goal_x) + "," + num(d.spec.goal_y)
                                              : std::string(""));
  }
  publish();
}

BoatRuntime::~BoatRuntime() {
  impl &d = *_impl;
  if (d.sim)
    d.sim->set_plant(nullptr);
}

const SceneBoatSpec &BoatRuntime::spec() const { return _impl->spec; }
bool BoatRuntime::placed() const { return _impl->plant && _impl->plant->size() > 0; }

bool BoatRuntime::place(ph::world &world, std::shared_ptr<const nav::world_grid> grid,
                        std::string *why) {
  impl &d = *_impl;
  if (placed() && d.world == &world)
    return true;
  const auto refuse = [&](const std::string &w) {
    d.status = w;
    if (why)
      *why = w;
    return false;
  };
  if (!grid || !grid->has_water()) {
    d.status = "waiting for its water's grid"; // not an attempt: nothing to warn about
    return false;
  }
  if (d.tried_world == &world && d.tried_grid == grid.get() &&
      d.tried_fields == world.field_count())
    return false; // the same world and grid it failed on
  d.tried_world = &world;
  d.tried_grid = grid.get();
  d.tried_fields = world.field_count();
  d.world = &world;
  d.grid = std::move(grid);
  const double fixed_dt = world.config().fixed_dt;
  d.every = std::max(1, static_cast<int>(std::lround(kNavDt / fixed_dt)));
  try {
    // Its navigator plans on the water deep enough for it, the land blocked.
    const nav::world_grid water =
        nav::medium_grid(*d.grid, cvc::nav::plant_medium::water, d.draft + kKeelClearance);
    const ph::dyn_boat_desc &b = d.spec.desc;
    const double length = 2.0 * d.half_length, k = std::max(0.1, length / 4.84);
    cvc::nav::sim_world::config cfg;
    nav::configure(cfg, water, kScale);
    cfg.range_m = 60.0 * k;
    cfg.n_rays = 120;
    cfg.veh.rr = static_cast<float>(1.2 * k * kScale);
    cfg.veh.d_hat = static_cast<float>(3.0 * k * kScale);
    cfg.veh.dt = static_cast<float>(d.every * fixed_dt);
    cfg.veh.vmax = static_cast<float>(b.max_speed * kScale);
    cfg.veh.L = static_cast<float>(length * kScale);
    cfg.veh.delta_max = static_cast<float>(std::atan(length / b.turn_radius));
    cfg.veh.a_max =
        static_cast<float>(0.8 * b.boat.propeller.max_thrust / b.boat.hull.mass * kScale);
    cfg.veh.a_lat_max = static_cast<float>(0.5 * 9.80665 * kScale);
    cfg.veh.k_steer = 0.8f;
    cfg.veh.nsub = 5;
    cfg.veh.allow_reverse = true;
    cfg.reach_tol = static_cast<float>(kReach * kScale);
    cfg.freeze_sense = true;
    const float o[2] = {d.nx(d.spec.x), d.ny(d.spec.y)};
    const float g[2] = {d.nx(d.gx), d.ny(d.gy)};
    const float col[3] = {1.0f, 1.0f, 1.0f};
    auto sim =
        std::make_unique<cvc::nav::sim_world>(cfg, water.occupancy.data(), water.occupancy.data(),
                                              cvc::nav::coef_mlp::default_biased(), o, g, col, 1);
    sim->set_pose(0, o[0], o[1], static_cast<float>(d.spec.heading));
    ph::dyn_boat_plant_options po;
    po.physics = &world;
    po.step_world = false;
    auto plant = std::make_unique<ph::dyn_boat_plant>(std::vector<ph::dyn_boat_desc>{b}, po);
    sim->set_plant(plant.get()); // attach: the boat goes in, or this throws
    d.sim = std::move(sim);
    d.plant = std::move(plant);
  } catch (const std::exception &ex) {
    d.sim.reset();
    d.plant.reset();
    return refuse(std::string("not in the world: ") + ex.what());
  }
  d.steps = 0;
  d.stepping = false;
  d.seen_goal.clear();
  d.status = "afloat";
  read_keys();
  return true;
}

void BoatRuntime::world_gone() {
  impl &d = *_impl;
  if (d.plant)
    d.plant->world_gone();
  if (d.sim)
    d.sim->set_plant(nullptr);
  d.sim.reset();
  d.plant.reset();
  d.world = nullptr;
  d.tried_world = nullptr;
  d.tried_grid = nullptr;
  d.status = "waiting for its water";
}

void BoatRuntime::read_keys() {
  impl &d = *_impl;
  if (d.app) {
    d.drive = read_or<std::string>(*d.app, d.key("drive"), d.spec.drive) == "nav" ? "nav" : "keys";
    if (const std::string goal = read_or<std::string>(*d.app, d.key("goal"), std::string());
        goal != d.seen_goal) {
      d.seen_goal = goal;
      double x = 0.0, y = 0.0;
      if (parse_xy(goal, x, y)) {
        d.gx = x;
        d.gy = y;
        if (d.sim)
          d.sim->retarget(0, d.nx(x), d.ny(y));
      }
    }
  }
  if (!placed())
    return;
  if (d.drive == "nav") {
    d.plant->release_controls(0);
  } else {
    const double t = d.app ? read_or<double>(*d.app, d.key("throttle"), 0.0) : 0.0;
    const double r = d.app ? read_or<double>(*d.app, d.key("rudder"), 0.0) : 0.0;
    d.plant->hold_controls(0, {std::isfinite(t) ? std::clamp(t, -1.0, 1.0) : 0.0,
                               std::isfinite(r) ? std::clamp(r, -1.0, 1.0) : 0.0});
  }
}

void BoatRuntime::before_step() {
  impl &d = *_impl;
  if (!placed())
    return;
  if (d.steps % static_cast<std::uint64_t>(d.every) == 0)
    d.sim->step(1); // nav's tick: its command for the steps to the next, and the plant's books
  d.plant->before_world_step();
  d.stepping = true;
  ++d.steps;
}

void BoatRuntime::after_step() {
  impl &d = *_impl;
  if (!placed() || !d.stepping)
    return;
  d.stepping = false;
  d.plant->after_world_step();
}

void BoatRuntime::publish() {
  impl &d = *_impl;
  if (!d.app)
    return;
  const auto put = [&](const char *leaf, const std::string &v) {
    write<std::string>(*d.app, d.key(leaf), v);
  };
  put("placed", placed() ? "1" : "0");
  put("status", d.status);
  if (!placed())
    return;
  const ph::boat_state s = d.plant->boat_state(0);
  put("x", num(s.body.position.x));
  put("y", num(s.body.position.y));
  put("z", num(s.body.position.z));
  put("heading", num(s.heading * 180.0 / kPi));
  put("speed", num(s.speed));
  put("thrust", num(s.thrust));
  put("reached", d.drive == "nav" && std::hypot(d.gx - s.body.position.x,
                                                d.gy - s.body.position.y) < kReach + d.half_length
                     ? "1"
                     : "0");
}

ph::boat_id BoatRuntime::boat() const { return placed() ? _impl->plant->boat(0) : ph::boat_id{}; }
ph::body_id BoatRuntime::hull() const { return placed() ? _impl->plant->hull(0) : ph::body_id{}; }
ph::boat_state BoatRuntime::state() const {
  if (!placed())
    throw cvc::physics_error("boat '" + _impl->spec.node + "' is not in the world");
  return _impl->plant->boat_state(0);
}
double BoatRuntime::draft() const { return _impl->draft; }
std::shared_ptr<const ph::body_desc> BoatRuntime::hull_desc() const { return _impl->hull_desc; }
double BoatRuntime::half_length() const { return _impl->half_length; }
double BoatRuntime::half_beam() const { return _impl->half_beam; }
double BoatRuntime::sheer() const { return _impl->sheer; }
double BoatRuntime::bottom() const { return _impl->bottom; }
const std::string &BoatRuntime::status() const { return _impl->status; }

} // namespace ariadne
} // namespace cvc
