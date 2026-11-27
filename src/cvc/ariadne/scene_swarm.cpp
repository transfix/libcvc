/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/color_value.h>
#include <cvc/ariadne/scene_swarm.h>
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h>
#include <cvc/core/clock_follower.h>
#include <cvc/core/sim_tick.h>
#include <cvc/core/world_clock.h>
#include <cvc/geometry/height_field.h>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/coef_train.h>
#include <cvc/nav/grid_nav.h> // astar, inflate, simplify: the belief routes
#include <cvc/nav/sim_thread.h>
#include <cvc/nav/sim_world.h>
#include <cvc/nav/world_planes.h>
#include <cvc/physics/plant_host.h>
#include <cvc/world/world_geometry.h>
#include <exception>
#include <future>
#include <limits>
#include <map>
#include <mutex>
#include <thread>
#include <utility>

// A worker thread for the navigator wherever there are threads: natively, and in a browser built
// with them (a cross-origin-isolated page). A browser without them steps it on the frame's thread.
#if !defined(__EMSCRIPTEN__) || defined(__EMSCRIPTEN_PTHREADS__)
#define CVC_SWARM_WORKER 1
#else
#define CVC_SWARM_WORKER 0
#endif

namespace cvc {
namespace ariadne {

namespace {

constexpr double kMetre = 0.05;     // navigator units a metre on a real city: the trained metric
constexpr double kHumveeLen = 4.72; // m, an M1097's length: the vehicle drawn on a real city
constexpr int kMaxTargets = 64;
constexpr int kMaxSeeing = 128; // planes kept per discovered wall cell

// HSV (h in [0, 1)) to RGB: the targets' colours, as the demos made them.
void hsv2rgb(double h, double s, double v, float out[3]) {
  const double i = std::floor(h * 6.0);
  const double f = h * 6.0 - i;
  const double p = v * (1.0 - s), q = v * (1.0 - f * s), t = v * (1.0 - (1.0 - f) * s);
  double r = v, g = v, b = v;
  switch (static_cast<int>(i) % 6) {
  case 0:
    r = v;
    g = t;
    b = p;
    break;
  case 1:
    r = q;
    g = v;
    b = p;
    break;
  case 2:
    r = p;
    g = v;
    b = t;
    break;
  case 3:
    r = p;
    g = q;
    b = v;
    break;
  case 4:
    r = t;
    g = p;
    b = v;
    break;
  default:
    r = v;
    g = p;
    b = q;
    break;
  }
  out[0] = static_cast<float>(r);
  out[1] = static_cast<float>(g);
  out[2] = static_cast<float>(b);
}

// A one-cell blocked border, so a reactive agent cannot drive off the open edge.
void add_border(std::vector<std::uint8_t> &occ, int rows, int cols) {
  if (rows < 1 || cols < 1 || occ.size() < static_cast<std::size_t>(rows) * cols)
    return;
  for (int c = 0; c < cols; ++c) {
    occ[static_cast<std::size_t>(c)] = 1;
    occ[static_cast<std::size_t>(rows - 1) * cols + c] = 1;
  }
  for (int r = 0; r < rows; ++r) {
    occ[static_cast<std::size_t>(r) * cols] = 1;
    occ[static_cast<std::size_t>(r) * cols + cols - 1] = 1;
  }
}

std::string lower(std::string s) {
  for (char &c : s)
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

// What one build makes, as the keys said when it was asked for.
struct build_request {
  int agents = 800;
  std::string belief = "grouped";
  bool fog = true;
  unsigned seed = 7;
  std::string plant = "kinematic";
  bool phantoms = true; // set out with the city's stale map, where it has one
  double range = 60.0;  // the sensor's range, world units
};

// The fog as a worker hands it over.
struct coverage_box {
  std::mutex m;
  std::uint64_t rev = 0;
  std::vector<std::uint8_t> coverage;
  std::vector<SceneSwarm::sensed_wall> walls;
  std::vector<std::uint8_t> belief;
  int ghosts = 0;
};

// The belief routes as a worker plans them (route: belief).
struct route_box {
  std::mutex m;
  std::uint64_t rev = 0;
  std::vector<std::vector<float>> routes;
  long replans = 0;
  int field_ver = std::numeric_limits<int>::min(); // the world's field when they were planned
  std::atomic<bool> busy{false};                   // a plan is posted to the worker
};

// What a route is planned on: the grid, and how far its walls are grown (the vehicle's radius).
struct route_grid {
  int rows = 0, cols = 0;
  double min_x = 0.0, min_y = 0.0, max_x = 0.0, max_y = 0.0;
  int inflate = 1;
};

// Each agent's route over its plane's belief, from where it is to goals[i] (world): the 8-connected
// A* path over the belief grown by the vehicle's radius, string-pulled, as world points from its
// second cell on (the first is its own), then the goal itself. An agent with no path drives
// straight at its goal. On the thread that steps `w`.
std::vector<std::vector<float>> plan_routes(const cvc::nav::sim_world &w, const route_grid &g,
                                            const std::vector<float> &goals) {
  const int n = w.size();
  std::vector<std::vector<float>> out(static_cast<std::size_t>(n));
  if (g.rows < 2 || g.cols < 2 || goals.size() < static_cast<std::size_t>(2) * n)
    return out;
  std::vector<float> pos(static_cast<std::size_t>(2) * n);
  w.snapshot(pos.data(), nullptr, nullptr, nullptr, nullptr);
  const double sx = (g.max_x - g.min_x) / (g.cols - 1), sy = (g.max_y - g.min_y) / (g.rows - 1);
  const auto cell = [&](double x, double y, int &r, int &c) {
    c = std::clamp(static_cast<int>(std::lround((x - g.min_x) / sx)), 0, g.cols - 1);
    r = std::clamp(static_cast<int>(std::lround((y - g.min_y) / sy)), 0, g.rows - 1);
  };
  const int *planes = w.agent_planes();
  std::map<int, std::vector<std::uint8_t>> grown; // a plane's belief, grown, once a plan
  for (int i = 0; i < n; ++i) {
    const int m = planes ? planes[i] : 0;
    auto it = grown.find(m);
    if (it == grown.end())
      it = grown.emplace(m, cvc::nav::inflate(w.belief_occ(m), g.rows, g.cols, g.inflate)).first;
    const std::uint8_t *occ = it->second.data();
    int sr, sc, gr, gc;
    cell(pos[2 * i], pos[2 * i + 1], sr, sc);
    cell(goals[2 * i], goals[2 * i + 1], gr, gc);
    const std::vector<int> path = cvc::nav::astar(occ, g.rows, g.cols, sr, sc, gr, gc);
    std::vector<float> &r = out[static_cast<std::size_t>(i)];
    if (path.size() >= 4) {
      const std::vector<int> sp =
          cvc::nav::simplify(occ, g.rows, g.cols, path.data(), static_cast<int>(path.size() / 2));
      for (std::size_t k = 2; k + 1 < sp.size(); k += 2) {
        r.push_back(static_cast<float>(g.min_x + sp[k + 1] * sx));
        r.push_back(static_cast<float>(g.min_y + sp[k] * sy));
      }
    }
    r.push_back(goals[2 * i]); // always the goal last, so a route that fails still drives at it
    r.push_back(goals[2 * i + 1]);
  }
  return out;
}

// The cells a box (x0 y0 x1 y1) covers in a grid whose cell (r, c) is centred at
// (min_x + c * cell, min_y + r * cell): their centres, else the row or column nearest its middle.
void stamp_boxes(const std::vector<std::array<double, 4>> &boxes, const swarm_city &c, double cell,
                 std::vector<std::uint8_t> &occ) {
  const auto span = [&](double a, double b, double lo, int n, int &i0, int &i1) {
    if (a > b)
      std::swap(a, b);
    i0 = static_cast<int>(std::ceil((a - lo) / cell - 1e-9));
    i1 = static_cast<int>(std::floor((b - lo) / cell + 1e-9));
    if (i0 > i1)
      i0 = i1 = static_cast<int>(std::lround((0.5 * (a + b) - lo) / cell));
    if (i1 < 0 || i0 > n - 1)
      return false;
    i0 = std::max(i0, 0);
    i1 = std::min(i1, n - 1);
    return true;
  };
  for (const auto &b : boxes) {
    int k0, k1, r0, r1;
    if (!span(b[0], b[2], c.min_x, c.cols, k0, k1) || !span(b[1], b[3], c.min_y, c.rows, r0, r1))
      continue;
    for (int r = r0; r <= r1; ++r)
      for (int k = k0; k <= k1; ++k)
        occ[static_cast<std::size_t>(r) * c.cols + k] = 1;
  }
}

// x, y from a [x, y] sequence; false (out untouched) otherwise.
bool read_xy(const Value *v, double out[2]) {
  if (!v || !v->is_seq() || v->items.size() < 2)
    return false;
  const double x = v->items[0].as_double(std::nan("")), y = v->items[1].as_double(std::nan(""));
  if (!std::isfinite(x) || !std::isfinite(y))
    return false;
  out[0] = x;
  out[1] = y;
  return true;
}

// Boxes from a list of [x0, y0, x1, y1]; others are skipped.
void read_boxes(const Value *v, std::vector<std::array<double, 4>> &out) {
  if (!v || !v->is_seq())
    return;
  for (const Value &b : v->items) {
    if (!b.is_seq() || b.items.size() < 4)
      continue;
    std::array<double, 4> a{};
    bool ok = true;
    for (int k = 0; k < 4; ++k) {
      a[k] = b.items[k].as_double(std::nan(""));
      ok = ok && std::isfinite(a[k]);
    }
    if (ok)
      out.push_back(a);
  }
}

// A colour from [r, g, b] (0..1) or a colour's text ("#rrggbb", a name, c0..c7).
bool read_rgb(const Value *v, float out[3]) {
  if (!v)
    return false;
  if (v->is_seq()) {
    if (v->items.size() < 3)
      return false;
    for (int k = 0; k < 3; ++k)
      out[k] = static_cast<float>(std::clamp(v->items[k].as_double(0.8), 0.0, 1.0));
    return true;
  }
  return parse_color_text(v->as_string(), out);
}

// The fog of `w` over `occ`: per cell 0 never seen, 1 seen, 2 in view; each seen wall's planes;
// and what the planes believe (scene_swarm.h's belief()), with how many ghosts.
void read_coverage(const cvc::nav::sim_world &w, const std::vector<std::uint8_t> &occ,
                   std::vector<std::uint8_t> &coverage, std::vector<SceneSwarm::sensed_wall> &walls,
                   std::vector<std::uint8_t> &belief, int &ghosts) {
  const std::size_t cells = occ.size();
  coverage.assign(cells, 0);
  walls.clear();
  const int M = w.planes();
  for (int m = 0; m < M; ++m) {
    const std::uint8_t *seen = w.ever_seen(m);
    const std::uint8_t *vis = w.last_visible(m);
    if (!seen || !vis)
      continue;
    for (std::size_t i = 0; i < cells; ++i) {
      if (vis[i])
        coverage[i] = 2;
      else if (seen[i] && coverage[i] == 0)
        coverage[i] = 1;
    }
  }
  for (std::size_t i = 0; i < cells; ++i) {
    if (!occ[i])
      continue;
    SceneSwarm::sensed_wall sw;
    for (int m = 0; m < M && static_cast<int>(sw.planes.size()) < kMaxSeeing; ++m) {
      const std::uint8_t *seen = w.ever_seen(m);
      if (seen && seen[i])
        sw.planes.push_back(m);
    }
    if (!sw.planes.empty()) {
      sw.cell = static_cast<int>(i);
      walls.push_back(std::move(sw));
    }
  }
  belief.assign(cells, 0);
  for (int m = 0; m < M; ++m) {
    const std::uint8_t *b = w.belief_occ(m);
    for (std::size_t i = 0; i < cells; ++i)
      if (b[i])
        belief[i] = occ[i] ? 1 : 2;
  }
  ghosts = static_cast<int>(std::count(belief.begin(), belief.end(), std::uint8_t{2}));
}

} // namespace

swarm_spec swarm_spec::from_props(const Value &p) {
  swarm_spec s;
  s.agents = std::max(1, static_cast<int>(p.num("agents", s.agents)));
  s.grid = std::clamp(static_cast<int>(p.num("grid", s.grid)), 8, 1024);
  s.belief = lower(p.str("belief", s.belief));
  s.fog = p.flag("fog", s.fog);
  s.seed = static_cast<unsigned>(std::max(0.0, p.num("seed", s.seed)));
  s.spawn = lower(p.str("spawn", s.spawn));
  s.targets = std::clamp(static_cast<int>(p.num("targets", s.targets)), 1, kMaxTargets);
  s.reach = std::max(0.01, p.num("reach", s.reach));
  s.separate = p.flag("separate", s.separate);
  s.sep_radius = std::max(0.0, p.num("sep_radius", s.sep_radius));
  s.sep_gain = std::max(0.0, p.num("sep_gain", s.sep_gain));
  s.plant = p.str("plant", s.plant);
  s.policy = p.str("policy", s.policy);
  s.sense_every = std::max(1, static_cast<int>(p.num("sense_every", s.sense_every)));
  s.sense_ticks = std::max(1, static_cast<int>(p.num("sense_ticks", s.sense_ticks)));
  s.noun = p.str("noun", s.noun);
  s.groups = p.str("groups", s.groups);
  if (const Value *vs = p.find("vehicles"); vs && vs->is_seq()) {
    for (const Value &v : vs->items) {
      swarm_vehicle sv;
      if (!v.is_map() || !read_xy(v.find("start"), sv.start) || !read_xy(v.find("goal"), sv.goal))
        continue;
      sv.has_color = read_rgb(v.find("color"), sv.color);
      sv.name = v.str("name");
      s.vehicles.push_back(sv);
    }
    if (!s.vehicles.empty() && !p.find("agents"))
      s.agents = static_cast<int>(s.vehicles.size());
  }
  if (const Value *a = p.find("arena"); a && a->is_map()) {
    s.has_arena = true;
    swarm_arena &ar = s.arena;
    const Value *b = a->find("bounds");
    const Value *sz = a->find("size");
    if (b && b->is_seq() && b->items.size() >= 4) {
      ar.min_x = b->items[0].as_double(ar.min_x);
      ar.min_y = b->items[1].as_double(ar.min_y);
      ar.max_x = b->items[2].as_double(ar.max_x);
      ar.max_y = b->items[3].as_double(ar.max_y);
    } else if (sz) {
      double wh[2] = {200.0, 200.0};
      if (!read_xy(sz, wh))
        wh[0] = wh[1] = sz->as_double(200.0);
      ar.min_x = -0.5 * wh[0];
      ar.max_x = 0.5 * wh[0];
      ar.min_y = -0.5 * wh[1];
      ar.max_y = 0.5 * wh[1];
    }
    read_boxes(a->find("walls"), ar.walls);
    read_boxes(a->find("phantoms"), ar.phantoms);
    ar.wall_height = std::max(0.0, a->num("wall_height", ar.wall_height));
    ar.name = a->str("name", ar.name);
  }
  s.route = lower(p.str("route", s.route));
  if (s.route != "belief")
    s.route = "none";
  s.range = std::max(1.0, p.num("range", s.range));
  return s;
}

double swarm_city::ground(double x, double y) const {
  if (!terrain)
    return 0.0;
  const double h = terrain->height(std::clamp(x, min_x, max_x), std::clamp(y, min_y, max_y));
  return std::isnan(h) ? 0.0 : h; // a hole reads as sea level
}

swarm_city synthetic_swarm_city(int grid) {
  const cvc::nav::training_scene ts = cvc::nav::city_scene(grid);
  swarm_city c;
  c.occ = ts.occ;
  c.rows = ts.rows;
  c.cols = ts.cols;
  c.min_x = ts.min_x;
  c.min_y = ts.min_y;
  c.max_x = ts.max_x;
  c.max_y = ts.max_y;
  c.scale = ts.scale;
  c.min_z = 0.0;
  c.max_z = 0.06 * (c.max_x - c.min_x) * 1.45; // its tallest block (swarm_layer city's)
  c.real = false;
  c.name = "the synthetic city";
  add_border(c.occ, c.rows, c.cols);
  return c;
}

swarm_city swarm_city_from_world(const cvc::world::world_geometry &w, int grid,
                                 const std::string &name) {
  swarm_city c;
  c.terrain = w.terrain();
  // The extent: the terrain's (a city bundle's terrain.json), else the world's footprint.
  if (c.terrain) {
    c.min_x = c.terrain->x0();
    c.min_y = c.terrain->y0();
    c.max_x = c.terrain->max_x();
    c.max_y = c.terrain->max_y();
  } else {
    const cvc::world::world_bounds b = w.bounds();
    if (b.empty())
      return c;
    c.min_x = b.min[0];
    c.min_y = b.min[1];
    c.max_x = b.max[0];
    c.max_y = b.max[1];
  }
  if (!(c.max_x > c.min_x) || !(c.max_y > c.min_y))
    return swarm_city{};
  const cvc::world::world_bounds wb = w.bounds();
  if (!wb.empty()) {
    c.min_z = wb.min[2];
    c.max_z = wb.max[2];
  }
  c.rows = c.cols = grid;
  c.scale = kMetre;
  c.real = true;
  c.name = name.empty() ? std::string("the city") : name;
  const cvc::world::geometry_item *b = w.find("buildings");
  if (b && b->geometry && b->geometry->num_tris() > 0) {
    // The navigator's occupancy from the world the scene draws: the buildings' footprints and any
    // hard ground on the grid.
    c.occ = cvc::nav::rasterize(w, grid, grid, c.min_x, c.min_y, c.max_x, c.max_y).occupancy;
  } else {
    // No buildings (a bundle whose mesh did not load): the synthetic city's blocks over the real
    // ground, so the swarm still has walls to keep off, as the demos did.
    const cvc::nav::training_scene ts = cvc::nav::city_scene(grid);
    c.occ = ts.occ;
    c.rows = ts.rows;
    c.cols = ts.cols;
  }
  add_border(c.occ, c.rows, c.cols);
  return c;
}

swarm_city arena_swarm_city(const swarm_arena &a, int grid) {
  swarm_city c;
  const double w = a.max_x - a.min_x, h = a.max_y - a.min_y;
  if (!(w > 0.0) || !(h > 0.0) || grid < 2)
    return c;
  const double cell = std::max(w, h) / (grid - 1);
  c.cols = std::max(2, static_cast<int>(std::lround(w / cell)) + 1);
  c.rows = std::max(2, static_cast<int>(std::lround(h / cell)) + 1);
  c.min_x = a.min_x;
  c.min_y = a.min_y;
  c.max_x = a.min_x + (c.cols - 1) * cell;
  c.max_y = a.min_y + (c.rows - 1) * cell;
  c.min_z = 0.0;
  c.max_z = a.wall_height;
  c.scale = kMetre;
  c.real = false;
  c.arena = true;
  c.name = a.name;
  c.wall_h = a.wall_height;
  c.occ.assign(static_cast<std::size_t>(c.rows) * c.cols, 0);
  stamp_boxes(a.walls, c, cell, c.occ);
  add_border(c.occ, c.rows, c.cols);
  if (!a.phantoms.empty()) {
    c.prior = c.occ;
    stamp_boxes(a.phantoms, c, cell, c.prior);
  }
  return c;
}

struct SceneSwarm::impl {
  cvc::app &app;
  std::string keys;
  swarm_spec spec;
  cvc::nav::training_scene ts; // the trainer's vehicle (rr, d_hat, dt, vmax), and its city

  swarm_city city;
  bool has_city = false;

  cvc::nav::coef_mlp model;
  std::string policy; // what drives: "trained (<path>)" or "built-in"

  cvc::physics::plant_host plant;
  std::unique_ptr<cvc::nav::sim_world> world; // one object for the run: rebuilds move into it
  std::unique_ptr<cvc::nav::sim_thread> thread;
  cvc::clock_follower follower; // without a worker
  std::uint64_t gen = 0;
  bool built = false;
  bool fog_on = true;
  std::string belief;
  double range = 60.0; // the built world's sensor range

#if CVC_SWARM_WORKER
  std::future<std::unique_ptr<cvc::nav::sim_world>> pending;
#endif
  build_request building_req; // the build in flight (what install() makes)
  build_request queued_req;   // one asked for while it builds: started once it is in
  bool queued = false;

  SceneSwarm::frame now;
  long last_tick = -1;

  // This generation's colours and targets.
  std::vector<float> colors, goals, target_colors, targets, plane_colors;
  std::vector<int> agent_targets;
  std::uint64_t targets_rev = 0;
  std::vector<float> targets_keyed; // the target positions the keys last held

  std::shared_ptr<coverage_box> box = std::make_shared<coverage_box>();
  std::uint64_t box_seen = 0;
  std::vector<std::uint8_t> coverage;
  std::vector<SceneSwarm::sensed_wall> walls;
  std::vector<std::uint8_t> belief_cells;
  int ghosts = 0, ghosts0 = 0;
  std::uint64_t cov_rev = 0;
  long last_sense = std::numeric_limits<long>::min() / 2;

  // The belief routes (route: belief), a new box each build so a plan for the old world is dropped.
  bool routing = false;
  std::shared_ptr<route_box> rbox = std::make_shared<route_box>();
  std::uint64_t rbox_seen = 0;
  std::vector<std::vector<float>> routes;
  std::vector<int> cursor;
  std::vector<float> aimed; // [2n] world: the waypoint each agent was last retargeted to
  std::uint64_t routes_rev = 0;
  bool replan_asked = false; // a goal moved: plan again whatever the belief
  long replans = 0;

  // The live keys as last applied.
  int separate = -1;
  double sep_gain = -1.0;
  long frames = 0;

  impl(cvc::app &a, std::string k, swarm_spec s)
      : app(a), keys(std::move(k)), spec(std::move(s)), ts(cvc::nav::city_scene(spec.grid)),
        plant(plant_options()) {
    routing = spec.route == "belief";
    load_policy();
    // The navigator's step is 7 quanta of the suite quantum on the scene's clock (cvc/core/
    // sim_tick.h), as the world's vehicles have it; a clock that has already ticked keeps its own,
    // and sim_thread refuses a step that is not a whole number of its quanta.
    cvc::adopt_sim_clock(app.world_clock());
    seed_keys();
  }

  ~impl() {
#if CVC_SWARM_WORKER
    if (pending.valid())
      pending.wait();
#endif
    if (thread)
      thread->stop();
    thread.reset();
  }

  static cvc::physics::plant_host_options plant_options() {
    cvc::physics::plant_host_options o;
    o.world.fixed_dt = cvc::kSimQuantum; // the navigator's tick (cvc::kSimTick) is whole steps
#if !defined(__EMSCRIPTEN__)
    const unsigned hw = std::thread::hardware_concurrency();
    o.world.threads = hw > 2 ? static_cast<int>(std::min(3u, hw - 2)) : 0;
#endif
    return o;
  }

  std::string key(const std::string &leaf) const { return keys + "." + leaf; }

  void load_policy() {
    if (lower(spec.policy) == "default" || lower(spec.policy) == "builtin") {
      model = cvc::nav::coef_mlp::default_biased();
      policy = "built-in CoefMLP";
      return;
    }
    const std::string path =
        spec.policy.empty() ? cvc::nav::coef_mlp::default_weights_path() : spec.policy;
    try {
      model = cvc::nav::coef_mlp::load(path);
      policy = "learned CoefMLP";
    } catch (const std::exception &e) {
      model = cvc::nav::coef_mlp::default_biased();
      policy = "built-in CoefMLP";
      std::fprintf(stderr,
                   "nav_swarm: no trained weights (%s: %s); the built-in biased net drives\n",
                   path.c_str(), e.what());
    }
  }

  void seed_keys() {
    read_or_seed<int>(app, key("agents"), spec.agents);
    read_or_seed<std::string>(app, key("belief"), spec.belief);
    read_bool_or_seed(app, key("fog"), spec.fog ? 1 : 0);
    read_or_seed<std::string>(app, key("plant"), spec.plant);
    read_or_seed<int>(app, key("seed"), static_cast<int>(spec.seed));
    read_bool_or_seed(app, key("separate"), spec.separate ? 1 : 0);
    read_or_seed<double>(app, key("sep_gain"), spec.sep_gain);
    read_or_seed<int>(app, key("follow"), -1);
    read_or_seed<int>(app, key("drag"), -1); // the target a minimap is dragging (-1: none)
    read_bool_or_seed(app, key("phantoms"), 1); // the city's stale map, where it has one
    read_or_seed<double>(app, key("range"), spec.range);
    write(app, key("stats.policy"), policy);
    write(app, key("stats.ready"), 0);
    write(app, key("stats.building"), 0);
  }

  build_request request_from_keys() const {
    build_request r;
    r.agents = std::clamp(read_or<int>(app, key("agents"), spec.agents), 1, 100000);
    r.belief = lower(read_or<std::string>(app, key("belief"), spec.belief));
    if (r.belief != "shared" && r.belief != "grouped" && r.belief != "private")
      r.belief = "grouped";
    r.fog = read_bool_or(app, key("fog"), spec.fog ? 1 : 0) != 0;
    r.seed = static_cast<unsigned>(std::max(0, read_or<int>(app, key("seed"), int(spec.seed))));
    r.plant = read_or<std::string>(app, key("plant"), spec.plant);
    if (r.plant.empty())
      r.plant = "kinematic";
    r.phantoms = read_bool_or(app, key("phantoms"), 1) != 0;
    r.range = std::max(1.0, read_or<double>(app, key("range"), spec.range));
    return r;
  }

  float rr() const { return ts.rr; }

  cvc::nav::sim_world::config config_for(const build_request &r) const {
    cvc::nav::sim_world::config cfg;
    cfg.rows = city.rows;
    cfg.cols = city.cols;
    cfg.min_x = city.min_x;
    cfg.min_y = city.min_y;
    cfg.max_x = city.max_x;
    cfg.max_y = city.max_y;
    cfg.cx = 0.5 * (city.min_x + city.max_x);
    cfg.cy = 0.5 * (city.min_y + city.max_y);
    cfg.scale = city.scale;
    // The trainer's vehicle: the coef_mlp's normalized constants stay physical at its metric.
    cfg.veh.rr = ts.rr;
    cfg.veh.d_hat = ts.d_hat;
    cfg.veh.dt = static_cast<float>(cvc::kSimTick); // the hosts' one tick, not the trainer's 0.06 s
    cfg.veh.vmax = ts.vmax;
    cfg.freeze_sense = !r.fog;
    cfg.reach_tol = static_cast<float>(spec.reach);
    cfg.range_m = r.range;
    cfg.sense_every = spec.sense_ticks;
    const bool sep = read_bool_or(app, key("separate"), spec.separate ? 1 : 0) != 0;
    const double gain = read_or<double>(app, key("sep_gain"), spec.sep_gain);
    cfg.sep_radius = static_cast<float>(spec.sep_radius) * ts.rr;
    cfg.sep_gain = sep ? static_cast<float>(gain) * ts.rr : 0.0f;
    cfg.spawn_layout = spec.spawn == "scattered" ? cvc::nav::sim_world::config::spawn::scattered
                                                 : cvc::nav::sim_world::config::spawn::opposed;
    return cfg;
  }

  // Listed vehicle i's colour (of n): its own, else the targets' hsv colour for its place.
  static void listed_color(const swarm_vehicle &v, int i, int n, float out[3]) {
    if (v.has_color) {
      std::copy(v.color, v.color + 3, out);
      return;
    }
    hsv2rgb(static_cast<double>(i) / std::max(1, n), 0.74, 0.98, out);
  }

  // A world as `r` asks, on the city's occupancy `occ` (the truth) with `prior` its agents' map
  // (empty: occ), its agents scattered or `listed`: callable on any thread (it reads only its
  // arguments).
  static std::unique_ptr<cvc::nav::sim_world>
  make_world(const cvc::nav::sim_world::config &cfg, const std::vector<std::uint8_t> &occ,
             const std::vector<std::uint8_t> &prior, const cvc::nav::coef_mlp &model,
             const build_request &r, const std::vector<swarm_vehicle> &listed) {
    const bool stale = !prior.empty() && prior.size() == occ.size();
    if (!listed.empty()) {
      const int n = std::clamp(r.agents, 1, static_cast<int>(listed.size()));
      std::vector<float> o(static_cast<std::size_t>(2) * n), g(o.size()),
          col(static_cast<std::size_t>(3) * n);
      for (int i = 0; i < n; ++i) {
        const swarm_vehicle &v = listed[static_cast<std::size_t>(i)];
        o[2 * i] = static_cast<float>((v.start[0] - cfg.cx) * cfg.scale);
        o[2 * i + 1] = static_cast<float>((v.start[1] - cfg.cy) * cfg.scale);
        g[2 * i] = static_cast<float>((v.goal[0] - cfg.cx) * cfg.scale);
        g[2 * i + 1] = static_cast<float>((v.goal[1] - cfg.cy) * cfg.scale);
        listed_color(v, i, n, &col[3 * static_cast<std::size_t>(i)]);
      }
      // shared: one plane; private: one each; grouped: max(2, n / 64) of them, in turn.
      std::vector<int> ids;
      int planes = 1;
      if (r.belief == "private" || r.belief == "grouped") {
        planes = r.belief == "private" ? n : std::min(n, std::max(2, n / 64));
        ids.resize(n);
        for (int i = 0; i < n; ++i)
          ids[i] = i % planes;
      }
      auto w = std::make_unique<cvc::nav::sim_world>(
          cfg, occ.data(), stale ? prior.data() : occ.data(), model, o.data(), g.data(), col.data(),
          n, ids.empty() ? nullptr : ids.data(), planes);
      for (int i = 0; i < n; ++i)
        if (!listed[static_cast<std::size_t>(i)].name.empty())
          w->set_name(i, listed[static_cast<std::size_t>(i)].name);
      return w;
    }
    auto m = cvc::nav::sim_world::belief_mode::shared;
    int k = 1;
    if (r.belief == "private") {
      m = cvc::nav::sim_world::belief_mode::private_belief;
    } else if (r.belief == "grouped") {
      m = cvc::nav::sim_world::belief_mode::clustered;
      k = std::max(2, r.agents / 64);
    }
    // A stale map is the city and its phantoms: the agents scatter on its free cells (none in a
    // phantom), their planes are seeded from it, and the world as it is replaces it as the truth.
    auto w = std::make_unique<cvc::nav::sim_world>(cvc::nav::sim_world::from_occupancy(
        cfg, stale ? prior.data() : occ.data(), model, r.agents, r.seed, m, k));
    if (stale)
      w->set_truth(occ.data());
    return w;
  }

  // The prior a build sets out with: the city's stale map when it has one and `r` asks for it.
  std::vector<std::uint8_t> prior_for(const build_request &r) const {
    return r.phantoms ? city.prior : std::vector<std::uint8_t>{};
  }

  void start_build(const build_request &r) {
    if (!has_city)
      return;
#if CVC_SWARM_WORKER
    if (pending.valid()) { // one at a time: this one follows the one in flight
      queued_req = r;
      queued = true;
      return;
    }
    building_req = r;
    const cvc::nav::sim_world::config cfg = config_for(r);
    pending = std::async(std::launch::async, [cfg, occ = city.occ, prior = prior_for(r), m = model,
                                              r, listed = spec.vehicles]() {
      return make_world(cfg, occ, prior, m, r, listed);
    });
    write(app, key("stats.building"), 1);
#else
    building_req = r;
    try {
      install(make_world(config_for(r), city.occ, prior_for(r), model, r, spec.vehicles));
    } catch (const std::exception &e) {
      std::fprintf(stderr, "nav_swarm: build: %s\n", e.what());
    }
#endif
  }

  // A built world in, in place of the one running.
  void install(std::unique_ptr<cvc::nav::sim_world> nw) {
    if (!nw)
      return;
    if (thread) {
      thread->stop();
      thread.reset(); // joined before the world it borrows changes under it
    }
    if (world) {
      try {
        plant.use(*world, "kinematic"); // off the old plant before its world goes
      } catch (const std::exception &) {
      }
      *world = std::move(*nw);
    } else {
      world = std::move(nw);
    }
    world->set_thread_pool(&app.computePool()); // a move clears it
    try {
      plant.use(*world, building_req.plant);
    } catch (const std::exception &e) {
      std::fprintf(stderr, "nav_swarm: plant %s: %s; on the kinematic bicycle\n",
                   building_req.plant.c_str(), e.what());
      try {
        plant.use(*world, "kinematic");
      } catch (const std::exception &) {
      }
    }
    fog_on = building_req.fog;
    belief = building_req.belief;
    range = building_req.range;
    separate = -1; // the live keys apply again to the new world
    sep_gain = -1.0;
    assign_targets();
    ++gen;
    built = true;
    now = SceneSwarm::frame{};
    last_tick = -1;
    last_sense = std::numeric_limits<long>::min() / 2;
    // The fog and the belief as built (a stale map's ghosts all standing), read here before a
    // worker owns the world.
    read_coverage(*world, city.occ, coverage, walls, belief_cells, ghosts);
    ghosts0 = ghosts;
    ++cov_rev;
    start_routes();
    // The world's first frame, read here before a worker owns it: a view has every agent at its
    // start from the frame the world is in, not only once the worker has published.
    read_frame(true);
    cvc::world_clock &clock = app.world_clock();
#if CVC_SWARM_WORKER
    if (spec.worker) {
      try {
        thread = std::make_unique<cvc::nav::sim_thread>(*world, clock);
        thread->start();
      } catch (const std::exception &e) {
        std::fprintf(stderr, "nav_swarm: no worker (%s); stepping on the frame's thread\n",
                     e.what());
        thread.reset();
      }
    }
#endif
    if (!thread) {
      // The world's own step, the hosts' tick (config_for), as a sim_thread follows it; not the
      // training scene's 0.06 s, which is no whole number of the suite's quanta.
      try {
        follower = cvc::clock_follower::for_step(
            clock, static_cast<double>(world->current_config().veh.dt));
      } catch (const std::exception &e) {
        // The clock's quantum does not divide the navigator's step: it cannot follow this clock.
        std::fprintf(stderr, "nav_swarm: %s\n", e.what());
      }
    }
    publish_build();
  }

  // The targets, their colours and each agent's: listed vehicles are each their own target, in
  // their own colours; scattered ones go by their goals' latitudes.
  void assign_targets() {
    if (!spec.vehicles.empty()) {
      const int n = world->size();
      targets.assign(static_cast<std::size_t>(2) * n, 0.0f);
      world->goals_world(targets.data());
      target_colors.assign(static_cast<std::size_t>(3) * n, 0.0f);
      for (int i = 0; i < n; ++i)
        listed_color(spec.vehicles[static_cast<std::size_t>(i)], i, n,
                     &target_colors[3 * static_cast<std::size_t>(i)]);
      colors = target_colors;
      agent_targets.resize(n);
      for (int i = 0; i < n; ++i)
        agent_targets[i] = i;
      goals = targets;
      color_planes();
      return;
    }
    const int n = world->size();
    const int T = spec.targets;
    std::vector<float> gw(static_cast<std::size_t>(2) * n);
    world->goals_world(gw.data()); // the scattered goals: reachable free ground
    double ymin = 1e300, ymax = -1e300, xsum = 0.0;
    for (int i = 0; i < n; ++i) {
      ymin = std::min(ymin, static_cast<double>(gw[2 * i + 1]));
      ymax = std::max(ymax, static_cast<double>(gw[2 * i + 1]));
      xsum += gw[2 * i];
    }
    const double cx = 0.5 * (city.min_x + city.max_x);
    const double xmean = n ? xsum / n : cx;
    target_colors.assign(static_cast<std::size_t>(3) * T, 0.0f);
    targets.assign(static_cast<std::size_t>(2) * T, 0.0f);
    for (int t = 0; t < T; ++t) {
      hsv2rgb(static_cast<double>(t) / T, 0.74, 0.98,
              &target_colors[3 * static_cast<std::size_t>(t)]);
      // One target per latitude band, on the scattered goal nearest its centre.
      const double ty = ymin + (t + 0.5) / T * std::max(1e-6, ymax - ymin);
      double best = 1e300;
      float bx = static_cast<float>(xmean), by = static_cast<float>(ty);
      for (int i = 0; i < n; ++i) {
        const double d = std::hypot(gw[2 * i] - xmean, gw[2 * i + 1] - ty);
        if (d < best) {
          best = d;
          bx = gw[2 * i];
          by = gw[2 * i + 1];
        }
      }
      targets[2 * static_cast<std::size_t>(t)] = bx;
      targets[2 * static_cast<std::size_t>(t) + 1] = by;
    }
    colors.assign(static_cast<std::size_t>(3) * n, 0.0f);
    agent_targets.assign(n, 0);
    for (int i = 0; i < n; ++i) {
      const double f =
          ymax > ymin ? (static_cast<double>(gw[2 * i + 1]) - ymin) / (ymax - ymin) : 0.0;
      const int t = std::clamp(static_cast<int>(f * T), 0, T - 1);
      agent_targets[i] = t;
      for (int k = 0; k < 3; ++k)
        colors[3 * static_cast<std::size_t>(i) + k] =
            target_colors[3 * static_cast<std::size_t>(t) + k];
      float nx, ny;
      to_nav(targets[2 * t], targets[2 * t + 1], nx, ny);
      world->retarget(i, nx, ny); // the band's agents to their shared target (no worker yet)
    }
    goals.assign(static_cast<std::size_t>(2) * n, 0.0f);
    world->goals_world(goals.data());
    color_planes();
  }

  // Each belief plane's colour (the mean of its agents'), and the targets out.
  void color_planes() {
    const int n = world->size();
    const int M = std::max(1, world->planes());
    const int *pl = world->agent_planes();
    plane_colors.assign(static_cast<std::size_t>(3) * M, 0.0f);
    std::vector<int> cnt(M, 0);
    for (int i = 0; i < n; ++i) {
      const int m = std::clamp(pl[i], 0, M - 1);
      for (int k = 0; k < 3; ++k)
        plane_colors[3 * static_cast<std::size_t>(m) + k] +=
            colors[3 * static_cast<std::size_t>(i) + k];
      ++cnt[m];
    }
    for (int m = 0; m < M; ++m)
      if (cnt[m])
        for (int k = 0; k < 3; ++k)
          plane_colors[3 * static_cast<std::size_t>(m) + k] /= static_cast<float>(cnt[m]);
    ++targets_rev;
    targets_keyed = targets;
    publish_targets();
  }

  void to_nav(double wx, double wy, float &nx, float &ny) const {
    const double cx = 0.5 * (city.min_x + city.max_x), cy = 0.5 * (city.min_y + city.max_y);
    nx = static_cast<float>((wx - cx) * city.scale);
    ny = static_cast<float>((wy - cy) * city.scale);
  }

  void retarget(int i, float nx, float ny) {
    if (thread)
      thread->retarget(i, nx, ny);
    else
      world->retarget(i, nx, ny);
  }

  void move_target(int t, double x, double y) {
    if (!built || t < 0 || t >= static_cast<int>(targets.size() / 2))
      return;
    x = std::clamp(x, city.min_x, city.max_x);
    y = std::clamp(y, city.min_y, city.max_y);
    targets[2 * static_cast<std::size_t>(t)] = static_cast<float>(x);
    targets[2 * static_cast<std::size_t>(t) + 1] = static_cast<float>(y);
    float nx, ny;
    to_nav(x, y, nx, ny);
    for (int i = 0; i < static_cast<int>(agent_targets.size()); ++i)
      if (agent_targets[i] == t) {
        retarget(i, nx, ny);
        goals[2 * static_cast<std::size_t>(i)] = static_cast<float>(x);
        goals[2 * static_cast<std::size_t>(i) + 1] = static_cast<float>(y);
      }
    ++targets_rev;
    replan_asked = routing; // the routes go to the goals: plan them again
    aimed.clear();          // and the agents were sent straight at them
    targets_keyed[2 * static_cast<std::size_t>(t)] = static_cast<float>(x);
    targets_keyed[2 * static_cast<std::size_t>(t) + 1] = static_cast<float>(y);
    write(app, key("targets." + std::to_string(t) + ".x"), x);
    write(app, key("targets." + std::to_string(t) + ".y"), y);
  }

  void publish_targets() {
    const int T = static_cast<int>(targets.size() / 2);
    write(app, key("targets.count"), T);
    for (int t = 0; t < T; ++t) {
      const std::string b = "targets." + std::to_string(t) + ".";
      write(app, key(b + "x"), static_cast<double>(targets[2 * t]));
      write(app, key(b + "y"), static_cast<double>(targets[2 * t + 1]));
      char rgb[64];
      std::snprintf(rgb, sizeof rgb, "%.3f,%.3f,%.3f", target_colors[3 * t],
                    target_colors[3 * t + 1], target_colors[3 * t + 2]);
      write(app, key(b + "color"), std::string(rgb));
    }
  }

  double vehicle_length() const {
    return city.real || city.arena ? kHumveeLen : 2.0 * static_cast<double>(ts.rr);
  }
  double wall_height() const { return city.wall_h > 0.0 ? city.wall_h : 0.06 * city.span(); }

  void publish_build() {
    write(app, key("stats.agents"), world ? world->size() : 0);
    write(app, key("stats.planes"), world ? world->planes() : 0);
    write(app, key("stats.driving"), plant.driving());
    write(app, key("stats.city"), city.name);
    write(app, key("stats.ready"), built ? 1 : 0);
    write(app, key("stats.building"), 0);
    write(app, key("stats.vehicle_length"), vehicle_length());
    write(app, key("stats.vehicle_radius"),
          city.real || city.arena ? 0.5 * kHumveeLen : static_cast<double>(ts.rr));
    write(app, key("stats.range"), range);
    write(app, key("stats.ghosts0"), ghosts0);
    write(app, key("stats.ghosts"), ghosts);
    write(app, key("stats.span"), city.span());
    write(app, key("stats.wall_height"), wall_height());
    char b[192];
    std::snprintf(b, sizeof b, "%g %g %g %g %g %g", city.min_x, city.min_y, city.min_z, city.max_x,
                  city.max_y, std::max(city.max_z, city.min_z + wall_height()));
    write(app, key("stats.bounds"), std::string(b));
    write(app, key("stats.center.x"), 0.5 * (city.min_x + city.max_x));
    write(app, key("stats.center.y"), 0.5 * (city.min_y + city.max_y));
    write(app, key("stats.center.z"), city.min_z + 0.5 * wall_height());
    publish_status();
  }

  void publish_status() {
    const int n = world ? world->size() : 0;
    const std::string driving = plant.driving();
    char st[256];
    // "800 agents", or "60 vehicles · 6 convoys" with groups: (the drive's line).
    std::string noun = spec.noun;
    if (n == 1 && noun.size() > 1 && noun.back() == 's') // "1 vehicle", not "1 vehicles"
      noun.pop_back();
    std::string who = std::to_string(n) + " " + noun;
    if (!spec.groups.empty())
      who += " \xC2\xB7 " + std::to_string(spec.targets) + " " + spec.groups;
    std::snprintf(st, sizeof st,
                  "%s \xC2\xB7 belief %s (M=%d) \xC2\xB7 %s \xC2\xB7 fog %s \xC2\xB7 %s "
                  "\xC2\xB7 arrivals %ld/%d",
                  who.c_str(), belief.c_str(), world ? world->planes() : 0, policy.c_str(),
                  fog_on ? "on" : "off",
                  driving == "kinematic" ? "kinematic" : (driving + " on Jolt").c_str(),
                  now.arrived, n);
    write(app, key("stats.status"), std::string(st));
    write(app, key("stats.arrived"), static_cast<int>(now.arrived));
    write(app, key("stats.tick"), static_cast<int>(now.tick));
    char tw[32]; // to the hundredth: a readout, not a clock
    std::snprintf(tw, sizeof tw, "%.2f",
                  static_cast<double>(now.tick) *
                      static_cast<double>(world ? world->current_config().veh.dt : 0.0f));
    write(app, key("stats.time"), std::string(tw));
    write(app, key("stats.replans"), static_cast<int>(replans));
  }

  // The latest navigator frame into `now`; true when it is newer.
  bool read_frame(bool force) {
    if (!world)
      return false;
#if CVC_SWARM_WORKER
    if (thread) {
      std::shared_ptr<const cvc::nav::sim_thread::snapshot> s = thread->read();
      if (!s || (!force && s->tick == last_tick))
        return false;
      now.tick = s->tick;
      now.n = s->n;
      now.pos = s->pos;
      now.heading = s->heading;
      now.mode = s->mode;
      now.reached = s->reached;
      now.carrot = s->carrot;
      last_tick = s->tick;
    } else
#endif
    {
      const long t = world->tick();
      if (!force && t == last_tick)
        return false;
      const int n = world->size();
      now.tick = t;
      now.n = n;
      now.pos.resize(static_cast<std::size_t>(2) * n);
      now.heading.resize(n);
      now.mode.resize(n);
      now.reached.resize(n);
      world->snapshot(now.pos.data(), now.heading.data(), nullptr, now.mode.data(),
                      now.reached.data());
      now.carrot.resize(static_cast<std::size_t>(2) * n);
      world->carrots_world(now.carrot.data());
      last_tick = t;
    }
    now.ground.resize(now.n);
    long arrived = 0;
    for (int i = 0; i < now.n; ++i) {
      now.ground[i] = city.ground(now.pos[2 * i], now.pos[2 * i + 1]);
      // On a route, only its goal is the agent's: a waypoint it parks at on the way is not.
      if (routing && i < static_cast<int>(routes.size()) && i < static_cast<int>(cursor.size()) &&
          static_cast<std::size_t>(cursor[i] + 1) < routes[i].size() / 2)
        now.reached[i] = 0;
      arrived += now.reached[i] ? 1 : 0;
    }
    now.arrived = arrived;
    return true;
  }

  // The live keys: avoidance, a target moved, a follow request, a restart.
  void take_keys() {
    const std::string req = read_or<std::string>(app, key("request"), std::string());
    if (!req.empty()) {
      write(app, key("request"), std::string());
      if (req == "reseed")
        write(app, key("seed"), read_or<int>(app, key("seed"), int(spec.seed)) + 1);
      if (req == "restart" || req == "reseed")
        start_build(request_from_keys());
    }
    if (!built)
      return;
    const int sep = read_bool_or(app, key("separate"), spec.separate ? 1 : 0);
    const double gain = read_or<double>(app, key("sep_gain"), spec.sep_gain);
    if (sep != separate || gain != sep_gain) {
      separate = sep;
      sep_gain = gain;
      // As the demos: off is no radius and no gain, and the vehicles pass through one another.
      const float r = sep ? static_cast<float>(spec.sep_radius) * ts.rr : 0.0f;
      const float g = sep ? static_cast<float>(gain) * ts.rr : 0.0f;
      if (thread)
        thread->post([r, g](cvc::nav::sim_world &w) { w.set_separation(r, g); });
      else
        world->set_separation(r, g);
    }
    // A target a program moved.
    const int T = static_cast<int>(targets.size() / 2);
    for (int t = 0; t < T; ++t) {
      const std::string b = "targets." + std::to_string(t) + ".";
      const double x = read_or<double>(app, key(b + "x"), targets_keyed[2 * t]);
      const double y = read_or<double>(app, key(b + "y"), targets_keyed[2 * t + 1]);
      if (std::fabs(x - targets_keyed[2 * t]) > 1e-6 ||
          std::fabs(y - targets_keyed[2 * t + 1]) > 1e-6)
        move_target(t, x, y);
    }
    // Follow the agent nearest a point.
    const std::string at = read_or<std::string>(app, key("follow_at"), std::string());
    if (!at.empty()) {
      write(app, key("follow_at"), std::string());
      double x = 0.0, y = 0.0;
      if (std::sscanf(at.c_str(), "%lf%*[ ,]%lf", &x, &y) == 2) {
        const int i = nearest_agent(x, y);
        if (i >= 0)
          write(app, key("follow"), i);
      }
    }
  }

  int nearest_agent(double x, double y) const {
    int best = -1;
    double bd = 1e300;
    for (int i = 0; i < now.n; ++i) {
      const double dx = now.pos[2 * i] - x, dy = now.pos[2 * i + 1] - y;
      const double d = dx * dx + dy * dy;
      if (d < bd) {
        bd = d;
        best = i;
      }
    }
    return best;
  }

  // The fog on the sense cadence: read on the worker (only it may touch the planes while it
  // runs), picked up here a frame later.
  void sense() {
    if (now.tick < last_sense || now.tick - last_sense >= spec.sense_every) {
      last_sense = now.tick;
      if (thread) {
        std::shared_ptr<coverage_box> b = box;
        std::vector<std::uint8_t> occ = city.occ;
        thread->post([b, occ = std::move(occ)](cvc::nav::sim_world &w) {
          std::vector<std::uint8_t> c, bel;
          std::vector<SceneSwarm::sensed_wall> s;
          int g = 0;
          read_coverage(w, occ, c, s, bel, g);
          std::lock_guard<std::mutex> lk(b->m);
          b->coverage = std::move(c);
          b->walls = std::move(s);
          b->belief = std::move(bel);
          b->ghosts = g;
          ++b->rev;
        });
      } else {
        const int was = ghosts;
        read_coverage(*world, city.occ, coverage, walls, belief_cells, ghosts);
        ++cov_rev;
        if (ghosts != was)
          write(app, key("stats.ghosts"), ghosts);
      }
    }
    std::lock_guard<std::mutex> lk(box->m);
    if (box->rev != box_seen) {
      box_seen = box->rev;
      if (box->coverage.size() == coverage.size()) {
        coverage.swap(box->coverage);
        walls.swap(box->walls);
        belief_cells.swap(box->belief);
        const int was = ghosts;
        ghosts = box->ghosts;
        ++cov_rev;
        if (ghosts != was)
          write(app, key("stats.ghosts"), ghosts);
      }
    }
  }

  route_grid grid_for_routes() const {
    route_grid g;
    g.rows = city.rows;
    g.cols = city.cols;
    g.min_x = city.min_x;
    g.min_y = city.min_y;
    g.max_x = city.max_x;
    g.max_y = city.max_y;
    // The walls grown by the vehicle's radius, at least a cell: the route keeps the drive's
    // clearance where it can.
    const double cell = city.span() / std::max(1, city.cols - 1);
    g.inflate = std::max(1, static_cast<int>(std::lround((ts.rr / city.scale) / cell)));
    return g;
  }

  // A new world's routes, planned here before a worker owns it, each agent sent to its first
  // waypoint.
  void start_routes() {
    routes.clear();
    cursor.clear();
    aimed.clear();
    replans = 0;
    replan_asked = false;
    rbox = std::make_shared<route_box>(); // a plan posted to the old world is dropped
    rbox_seen = 0;
    if (!routing)
      return;
    routes = plan_routes(*world, grid_for_routes(), goals);
    rbox->field_ver = world->field_version();
    replans = 1;
    cursor.assign(routes.size(), 0);
    ++routes_rev;
    follow_routes(true);
  }

  // Each agent on to its route's next waypoint once it is near its current one, or, `all`, every
  // agent to its current one (new routes). Retargeted only when the point it drives at changes: a
  // retarget sets the drive's carrot going again, and unparks an agent at its goal.
  void follow_routes(bool all) {
    // A waypoint is passed within 3% of the city's span, or its reach and a quarter, so the cursor
    // moves on before the drive would park at it.
    const double arrive = std::max(0.03 * city.span(), 1.25 * spec.reach / city.scale);
    const int n = std::min(static_cast<int>(routes.size()), static_cast<int>(cursor.size()));
    for (int i = 0; i < n; ++i) {
      const std::vector<float> &r = routes[static_cast<std::size_t>(i)];
      const int nw = static_cast<int>(r.size() / 2);
      if (nw == 0)
        continue;
      int &k = cursor[static_cast<std::size_t>(i)];
      bool moved = all;
      float px = 0.0f, py = 0.0f;
      bool have = false;
      if (i < now.n && now.pos.size() >= static_cast<std::size_t>(2) * (i + 1)) {
        px = now.pos[2 * i];
        py = now.pos[2 * i + 1];
        have = true;
      } else if (world && !thread) {
        std::vector<float> p(static_cast<std::size_t>(2) * world->size());
        world->snapshot(p.data(), nullptr, nullptr, nullptr, nullptr);
        px = p[2 * i];
        py = p[2 * i + 1];
        have = true;
      }
      while (have && k + 1 < nw && std::hypot(px - r[2 * k], py - r[2 * k + 1]) <= arrive) {
        ++k;
        moved = true;
      }
      if (aimed.size() != routes.size() * 2)
        aimed.assign(routes.size() * 2, std::numeric_limits<float>::quiet_NaN());
      const float ax = r[2 * k], ay = r[2 * k + 1];
      if (moved &&
          !(std::fabs(ax - aimed[2 * i]) < 1e-3f && std::fabs(ay - aimed[2 * i + 1]) < 1e-3f)) {
        aimed[2 * i] = ax;
        aimed[2 * i + 1] = ay;
        float nx, ny;
        to_nav(ax, ay, nx, ny);
        retarget(i, nx, ny);
      }
    }
  }

  // The routes each frame: planned again (on the worker, which alone may read the planes while
  // it runs) once a belief flip has changed some plane's map or a goal has moved, picked up a
  // frame later, and followed.
  void tick_routes(bool fresh) {
    if (!routing || !world)
      return;
    const route_grid g = grid_for_routes();
    if (thread) {
      if (!rbox->busy.exchange(true)) {
        const bool force = replan_asked;
        replan_asked = false;
        std::shared_ptr<route_box> b = rbox;
        thread->post([b, g, goals = goals, force](cvc::nav::sim_world &w) {
          if (force || w.field_version() != b->field_ver) {
            std::vector<std::vector<float>> r = plan_routes(w, g, goals);
            std::lock_guard<std::mutex> lk(b->m);
            b->routes = std::move(r);
            b->field_ver = w.field_version();
            ++b->replans;
            ++b->rev;
          }
          b->busy.store(false);
        });
      }
    } else if (replan_asked || world->field_version() != rbox->field_ver) {
      replan_asked = false;
      std::vector<std::vector<float>> r = plan_routes(*world, g, goals);
      std::lock_guard<std::mutex> lk(rbox->m);
      rbox->routes = std::move(r);
      rbox->field_ver = world->field_version();
      ++rbox->replans;
      ++rbox->rev;
    }
    bool fresh_routes = false;
    {
      std::lock_guard<std::mutex> lk(rbox->m);
      if (rbox->rev != rbox_seen) {
        rbox_seen = rbox->rev;
        routes = rbox->routes;
        replans = 1 + rbox->replans;
        fresh_routes = true;
      }
    }
    if (fresh_routes) {
      cursor.assign(routes.size(), 0);
      ++routes_rev;
      follow_routes(true);
    } else if (fresh) {
      follow_routes(false);
    }
  }

  bool update(bool wait) {
#if CVC_SWARM_WORKER
    if (pending.valid() && pending.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      std::unique_ptr<cvc::nav::sim_world> nw;
      try {
        nw = pending.get();
      } catch (const std::exception &e) {
        std::fprintf(stderr, "nav_swarm: build: %s\n", e.what());
        write(app, key("stats.building"), 0);
      }
      if (nw)
        install(std::move(nw));
      if (queued) {
        queued = false;
        start_build(queued_req);
      }
    }
#endif
    take_keys();
    if (!built)
      return false;
    if (!thread) {
      for (std::uint64_t s = follower.take(app.world_clock()); s > 0; --s)
        world->step(0);
    } else if (wait) {
      thread->wait_caught_up(10.0);
    }
    const bool fresh = read_frame(false);
    sense();
    tick_routes(fresh);
    // The followed agent's pose, for a camera or a program.
    const int follow = read_or<int>(app, key("follow"), -1);
    if (fresh && follow >= 0 && follow < now.n) {
      write(app, key("follow_pose.x"), static_cast<double>(now.pos[2 * follow]));
      write(app, key("follow_pose.y"), static_cast<double>(now.pos[2 * follow + 1]));
      write(app, key("follow_pose.z"), now.ground[follow]);
      write(app, key("follow_pose.heading"), static_cast<double>(now.heading[follow]));
    }
    if (++frames % 15 == 1)
      publish_status();
    return fresh;
  }
};

SceneSwarm::SceneSwarm(cvc::app &app, std::string keys, swarm_spec spec)
    : _impl(std::make_unique<impl>(app, std::move(keys), std::move(spec))) {}

SceneSwarm::~SceneSwarm() = default;

void SceneSwarm::set_city(swarm_city city) {
  if (city.rows < 2 || city.cols < 2 ||
      city.occ.size() != static_cast<std::size_t>(city.rows) * city.cols)
    return;
  _impl->city = std::move(city);
  _impl->has_city = true;
  _impl->start_build(_impl->request_from_keys());
}

bool SceneSwarm::has_city() const { return _impl->has_city; }
const swarm_city &SceneSwarm::city() const { return _impl->city; }
bool SceneSwarm::update(bool wait) { return _impl->update(wait); }
const SceneSwarm::frame &SceneSwarm::now() const { return _impl->now; }
std::uint64_t SceneSwarm::generation() const { return _impl->gen; }
bool SceneSwarm::running() const { return _impl->built; }
bool SceneSwarm::building() const {
#if CVC_SWARM_WORKER
  return _impl->pending.valid();
#else
  return false;
#endif
}
int SceneSwarm::size() const { return _impl->world ? _impl->world->size() : 0; }
int SceneSwarm::planes() const { return _impl->world ? _impl->world->planes() : 0; }
bool SceneSwarm::fog() const { return _impl->fog_on; }
double SceneSwarm::vehicle_radius() const { return _impl->ts.rr; }
double SceneSwarm::vehicle_length() const { return _impl->vehicle_length(); }
double SceneSwarm::wall_height() const { return _impl->wall_height(); }
double SceneSwarm::sensor_range() const { return _impl->range; }
const std::vector<float> &SceneSwarm::colors() const { return _impl->colors; }
const std::vector<float> &SceneSwarm::goals() const { return _impl->goals; }
const std::vector<int> &SceneSwarm::agent_targets() const { return _impl->agent_targets; }
const std::vector<float> &SceneSwarm::target_colors() const { return _impl->target_colors; }
const std::vector<float> &SceneSwarm::targets() const { return _impl->targets; }
std::uint64_t SceneSwarm::targets_revision() const { return _impl->targets_rev; }
const std::vector<float> &SceneSwarm::plane_colors() const { return _impl->plane_colors; }
std::uint64_t SceneSwarm::coverage_revision() const { return _impl->cov_rev; }
const std::vector<std::uint8_t> &SceneSwarm::coverage() const { return _impl->coverage; }
const std::vector<SceneSwarm::sensed_wall> &SceneSwarm::sensed_walls() const {
  return _impl->walls;
}
const std::vector<std::uint8_t> &SceneSwarm::belief() const { return _impl->belief_cells; }
int SceneSwarm::ghosts() const { return _impl->ghosts; }
int SceneSwarm::ghosts_at_build() const { return _impl->ghosts0; }
const std::vector<std::vector<float>> &SceneSwarm::routes() const { return _impl->routes; }
const std::vector<int> &SceneSwarm::route_cursor() const { return _impl->cursor; }
std::uint64_t SceneSwarm::routes_revision() const { return _impl->routes_rev; }
int SceneSwarm::nearest_agent(double x, double y) const { return _impl->nearest_agent(x, y); }
void SceneSwarm::move_target(int t, double x, double y) { _impl->move_target(t, x, y); }
void SceneSwarm::restart() { _impl->start_build(_impl->request_from_keys()); }
const std::string &SceneSwarm::keys() const { return _impl->keys; }

} // namespace ariadne
} // namespace cvc
