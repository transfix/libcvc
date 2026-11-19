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
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/scene_swarm.h>
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h>
#include <cvc/core/clock_follower.h>
#include <cvc/core/sim_tick.h>
#include <cvc/core/world_clock.h>
#include <cvc/geometry/height_field.h>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/coef_train.h>
#include <cvc/nav/sim_thread.h>
#include <cvc/nav/sim_world.h>
#include <cvc/nav/world_planes.h>
#include <cvc/physics/plant_host.h>
#include <cvc/world/world_geometry.h>
#include <exception>
#include <future>
#include <limits>
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
};

// The fog as a worker hands it over.
struct coverage_box {
  std::mutex m;
  std::uint64_t rev = 0;
  std::vector<std::uint8_t> coverage;
  std::vector<SceneSwarm::sensed_wall> walls;
};

// The fog of `w` over `occ`: per cell 0 never seen, 1 seen, 2 in view; each seen wall's planes.
void read_coverage(const cvc::nav::sim_world &w, const std::vector<std::uint8_t> &occ,
                   std::vector<std::uint8_t> &coverage,
                   std::vector<SceneSwarm::sensed_wall> &walls) {
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
  s.noun = p.str("noun", s.noun);
  s.groups = p.str("groups", s.groups);
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
  std::uint64_t cov_rev = 0;
  long last_sense = std::numeric_limits<long>::min() / 2;

  // The live keys as last applied.
  int separate = -1;
  double sep_gain = -1.0;
  long frames = 0;

  impl(cvc::app &a, std::string k, swarm_spec s)
      : app(a), keys(std::move(k)), spec(std::move(s)), ts(cvc::nav::city_scene(spec.grid)),
        plant(plant_options()) {
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
    const bool sep = read_bool_or(app, key("separate"), spec.separate ? 1 : 0) != 0;
    const double gain = read_or<double>(app, key("sep_gain"), spec.sep_gain);
    cfg.sep_radius = static_cast<float>(spec.sep_radius) * ts.rr;
    cfg.sep_gain = sep ? static_cast<float>(gain) * ts.rr : 0.0f;
    cfg.spawn_layout = spec.spawn == "scattered" ? cvc::nav::sim_world::config::spawn::scattered
                                                 : cvc::nav::sim_world::config::spawn::opposed;
    return cfg;
  }

  // A world as `r` asks, on `c`: callable on any thread (it reads only its arguments).
  static std::unique_ptr<cvc::nav::sim_world> make_world(const cvc::nav::sim_world::config &cfg,
                                                         const std::vector<std::uint8_t> &occ,
                                                         const cvc::nav::coef_mlp &model,
                                                         const build_request &r) {
    auto m = cvc::nav::sim_world::belief_mode::shared;
    int k = 1;
    if (r.belief == "private") {
      m = cvc::nav::sim_world::belief_mode::private_belief;
    } else if (r.belief == "grouped") {
      m = cvc::nav::sim_world::belief_mode::clustered;
      k = std::max(2, r.agents / 64);
    }
    return std::make_unique<cvc::nav::sim_world>(
        cvc::nav::sim_world::from_occupancy(cfg, occ.data(), model, r.agents, r.seed, m, k));
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
    pending = std::async(std::launch::async, [cfg, occ = city.occ, m = model, r]() {
      return make_world(cfg, occ, m, r);
    });
    write(app, key("stats.building"), 1);
#else
    building_req = r;
    try {
      install(make_world(config_for(r), city.occ, model, r));
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
    separate = -1; // the live keys apply again to the new world
    sep_gain = -1.0;
    assign_targets();
    ++gen;
    built = true;
    now = SceneSwarm::frame{};
    last_tick = -1;
    last_sense = std::numeric_limits<long>::min() / 2;
    coverage.assign(static_cast<std::size_t>(city.rows) * city.cols, 0);
    walls.clear();
    ++cov_rev;
    // The world's first frame, read here before a worker owns it: a view has every agent at its
    // start from the frame the world is in, not only once the worker has published.
    read_frame(true);
    cvc::world_clock &clock = app.world_clock();
#if CVC_SWARM_WORKER
    try {
      thread = std::make_unique<cvc::nav::sim_thread>(*world, clock);
      thread->start();
    } catch (const std::exception &e) {
      std::fprintf(stderr, "nav_swarm: no worker (%s); stepping on the frame's thread\n", e.what());
      thread.reset();
    }
#endif
    if (!thread) {
      try {
        follower = cvc::clock_follower::for_step(clock, ts.dt);
      } catch (const std::exception &e) {
        // The clock's quantum does not divide the navigator's step: it cannot follow this clock.
        std::fprintf(stderr, "nav_swarm: %s\n", e.what());
      }
    }
    publish_build();
  }

  // The targets, their colours and each agent's, from the scattered goals' latitudes.
  void assign_targets() {
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
    // Each belief plane's colour: the mean of its agents'.
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
    return city.real ? kHumveeLen : 2.0 * static_cast<double>(ts.rr);
  }
  double wall_height() const { return 0.06 * city.span(); }

  void publish_build() {
    write(app, key("stats.agents"), world ? world->size() : 0);
    write(app, key("stats.planes"), world ? world->planes() : 0);
    write(app, key("stats.driving"), plant.driving());
    write(app, key("stats.city"), city.name);
    write(app, key("stats.ready"), built ? 1 : 0);
    write(app, key("stats.building"), 0);
    write(app, key("stats.vehicle_length"), vehicle_length());
    write(app, key("stats.vehicle_radius"),
          city.real ? 0.5 * kHumveeLen : static_cast<double>(ts.rr));
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
    std::string who = std::to_string(n) + " " + spec.noun;
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
      last_tick = t;
    }
    now.ground.resize(now.n);
    long arrived = 0;
    for (int i = 0; i < now.n; ++i) {
      now.ground[i] = city.ground(now.pos[2 * i], now.pos[2 * i + 1]);
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
          std::vector<std::uint8_t> c;
          std::vector<SceneSwarm::sensed_wall> s;
          read_coverage(w, occ, c, s);
          std::lock_guard<std::mutex> lk(b->m);
          b->coverage = std::move(c);
          b->walls = std::move(s);
          ++b->rev;
        });
      } else {
        read_coverage(*world, city.occ, coverage, walls);
        ++cov_rev;
      }
    }
    std::lock_guard<std::mutex> lk(box->m);
    if (box->rev != box_seen) {
      box_seen = box->rev;
      if (box->coverage.size() == coverage.size()) {
        coverage.swap(box->coverage);
        walls.swap(box->walls);
        ++cov_rev;
      }
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
int SceneSwarm::nearest_agent(double x, double y) const { return _impl->nearest_agent(x, y); }
void SceneSwarm::move_target(int t, double x, double y) { _impl->move_target(t, x, y); }
void SceneSwarm::restart() { _impl->start_build(_impl->request_from_keys()); }
const std::string &SceneSwarm::keys() const { return _impl->keys; }

} // namespace ariadne
} // namespace cvc
