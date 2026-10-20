/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc-plant-log: nav driving physics vehicles (cvc::physics::nav_plant on Jolt), logged as planar
// rigid bodies for fitting a plant model, and the dynamic single-track model's one-tick residual
// against them (cvc/physics/plant_log.h).
//
//   cvc-plant-log --vehicle humvee --scene swerve --seeds 100,101,102 --out runs/humvee-swerve
//   cvc-plant-log --vehicle roboracer --scene city --agents 8 --out runs/roboracer-city
//   cvc-plant-log --vehicle humvee --scene bundle --bundle <city bundle dir> --out runs/humvee-city
//
// Scenes:
//   swerve  open flat ground; each agent slaloms, its goal jumping from side to side, at a
//           seed-varied speed with a corner cap of --lateral-g (the swerves past 0.3 g).
//   city    the coef trainer's city (city_scene(96)) scaled to the vehicle, as the plant reach
//           gate drives it.
//   bundle  a cvc::world city bundle (load_city_bundle): its buildings and terrain in the
//           physics world, its rasterized occupancy for nav (50 m in from the terrain's edge),
//           vehicles spawned on the terrain.
//   In city each agent gets a new random goal when it arrives (the trained drive alone does well
//   on so small a map). In bundle each drives vehicle routes (plan_vehicle_route with the plant's
//   footprint and turning circle, route_follower) to random goals 5-15% of the map away, a new
//   one at each arrival: the reactive drive alone stalls on long blocks.
// Logging starts after 0.5 s, once the vehicles have settled on their springs.
//
// Out (--out DIR):
//   flights.npy   float64 (F, T, 8): equal-length flights cut from the runs, rows
//                 [x, y, psi, vx, vy, r, drive, steer] (cvc::physics::kPlanarWidth): the centre
//                 of mass on SE(2), its body twist, nav's acceleration command and the steering
//                 (--steer actual|command). Row k's inputs acted from t_k-1 to t_k.
//   meta.json     the rate, the layout, the run's settings and each flight's seed and agent.
//   residual.json the one-tick residuals of diff::dyn_plant and the kinematic bicycle against
//                 the plant, binned by lateral g (also printed).
//   samples.csv   with --samples: every tick of every run.
// --fit-dyn fits dyn_bike's lags, tire stiffness and yaw inertia to the runs of every seed but
// the last (cvc::physics::fit_dyn_bike) and reports the last seed's residual with the spec-sheet
// and the fitted vehicle (residual.json's "fit").
// tools/liesphgp_pack.py turns one or more of these directories into a LieSPHGP dataset pickle.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/coef_train.h>
#include <cvc/nav/route_follower.h>
#include <cvc/nav/sim_world.h>
#include <cvc/nav/vehicle_route.h>
#include <cvc/nav/world_planes.h>
#include <cvc/physics/nav_plant.h>
#include <cvc/physics/plant_host.h>
#include <cvc/physics/plant_log.h>
#include <cvc/physics/world_geometry.h>
#include <cvc/world/city_bundle.h>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

namespace phys = cvc::physics;
namespace nav = cvc::nav;

struct options {
  std::string vehicle = "humvee";
  std::string scene = "swerve";
  std::string bundle;
  std::string policy;
  std::string out;
  std::vector<unsigned> seeds = {100, 101, 102, 103};
  int agents = -1; // per scene default
  double seconds = 60.0;
  double dt = 0.02;
  double mu = 0.9;
  double lateral_g = -1.0; // per scene default
  double flight_seconds = 5.0;
  double stride_seconds = 5.0;
  int nsub = 0; // residual substeps; 0 = the physics steps a tick
  int grid = 600;
  int inflate = 0;
  bool verbose = false;
  bool samples = false;
  bool fit_dyn = false;
  double accel_margin = -1.0, brake_margin = -1.0; // the plant's; < 0 keeps its defaults
  phys::steer_input steer = phys::steer_input::actual;
};

void usage() {
  std::printf(
      "cvc-plant-log --out DIR [--vehicle humvee|roboracer] [--scene swerve|city|bundle]\n"
      "              [--bundle DIR] [--seeds 100,101,...] [--agents N] [--seconds S] [--dt S]\n"
      "              [--mu MU] [--lateral-g G] [--flight-seconds S] [--stride-seconds S]\n"
      "              [--steer actual|command] [--nsub N] [--grid N] [--inflate CELLS]\n"
      "              [--accel-margin SHARE] [--brake-margin SHARE] [--policy FILE.cvcnav]\n"
      "              [--verbose] [--samples] [--fit-dyn]\n");
}

options parse(int argc, char **argv) {
  options o;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    const auto val = [&]() -> std::string {
      if (i + 1 >= argc)
        throw std::invalid_argument(a + " needs a value");
      return argv[++i];
    };
    if (a == "--vehicle")
      o.vehicle = val();
    else if (a == "--scene")
      o.scene = val();
    else if (a == "--bundle")
      o.bundle = val();
    else if (a == "--policy")
      o.policy = val();
    else if (a == "--out")
      o.out = val();
    else if (a == "--seeds") {
      o.seeds.clear();
      std::stringstream ss(val());
      std::string t;
      while (std::getline(ss, t, ','))
        o.seeds.push_back(static_cast<unsigned>(std::stoul(t)));
    } else if (a == "--agents")
      o.agents = std::stoi(val());
    else if (a == "--seconds")
      o.seconds = std::stod(val());
    else if (a == "--dt")
      o.dt = std::stod(val());
    else if (a == "--mu")
      o.mu = std::stod(val());
    else if (a == "--lateral-g")
      o.lateral_g = std::stod(val());
    else if (a == "--flight-seconds")
      o.flight_seconds = std::stod(val());
    else if (a == "--stride-seconds")
      o.stride_seconds = std::stod(val());
    else if (a == "--nsub")
      o.nsub = std::stoi(val());
    else if (a == "--grid")
      o.grid = std::stoi(val());
    else if (a == "--inflate")
      o.inflate = std::stoi(val());
    else if (a == "--verbose")
      o.verbose = true;
    else if (a == "--samples")
      o.samples = true;
    else if (a == "--fit-dyn")
      o.fit_dyn = true;
    else if (a == "--accel-margin")
      o.accel_margin = std::stod(val());
    else if (a == "--brake-margin")
      o.brake_margin = std::stod(val());
    else if (a == "--steer") {
      const std::string s = val();
      if (s == "actual")
        o.steer = phys::steer_input::actual;
      else if (s == "command")
        o.steer = phys::steer_input::command;
      else
        throw std::invalid_argument("--steer is actual or command");
    } else if (a == "-h" || a == "--help") {
      usage();
      std::exit(0);
    } else
      throw std::invalid_argument("unknown option " + a);
  }
  if (o.out.empty())
    throw std::invalid_argument("--out is required");
  if (o.scene != "swerve" && o.scene != "city" && o.scene != "bundle")
    throw std::invalid_argument("--scene is swerve, city or bundle");
  if (o.scene == "bundle" && o.bundle.empty())
    throw std::invalid_argument("--scene bundle needs --bundle DIR");
  if (o.agents < 0)
    o.agents = o.scene == "swerve" ? 4 : 8;
  if (o.lateral_g < 0.0)
    o.lateral_g = o.scene == "swerve" ? 0.9 : 0.5;
  return o;
}

// A grid's free cells in its largest 8-connected free component (start and goal candidates).
std::vector<int> largest_free_component(const std::vector<std::uint8_t> &occ, int rows, int cols) {
  std::vector<int> label(occ.size(), -1), best;
  int next = 0;
  for (int s = 0; s < rows * cols; ++s) {
    if (occ[s] || label[s] >= 0)
      continue;
    std::vector<int> comp;
    std::deque<int> q{s};
    label[s] = next;
    while (!q.empty()) {
      const int c = q.front();
      q.pop_front();
      comp.push_back(c);
      const int r = c / cols, k = c % cols;
      for (int dr = -1; dr <= 1; ++dr)
        for (int dk = -1; dk <= 1; ++dk) {
          const int rr = r + dr, kk = k + dk;
          if (rr < 0 || kk < 0 || rr >= rows || kk >= cols)
            continue;
          const int n = rr * cols + kk;
          if (!occ[n] && label[n] < 0) {
            label[n] = next;
            q.push_back(n);
          }
        }
    }
    if (comp.size() > best.size())
      best.swap(comp);
    ++next;
  }
  return best;
}

// The sim and the plant for one seed.
struct run_setup {
  nav::sim_world::config cfg;
  std::vector<std::uint8_t> occ;
  std::vector<int> free_cells;     // goal candidates (city, bundle)
  std::vector<float> o, goal, col; // normalized starts and goals
  phys::plant_host_options host;   // the physics world and the plant's options
  const cvc::world::world_geometry *geometry = nullptr;
  std::vector<float> height; // the terrain's heights on the grid (bundle)
  double vmax_mps = 0.0;
};

// The cell -> normalized position of a grid.
void cell_to_normalized(const nav::sim_world::config &c, int cell, float &x, float &y) {
  const int r = cell / c.cols, k = cell % c.cols;
  const double wx = c.min_x + k * (c.max_x - c.min_x) / (c.cols - 1);
  const double wy = c.min_y + r * (c.max_y - c.min_y) / (c.rows - 1);
  x = static_cast<float>((wx - c.cx) * c.scale);
  y = static_cast<float>((wy - c.cy) * c.scale);
}

void base_vehicle(nav::sim_world::config &c, double dt) {
  c.veh.rr = 0.15f;
  c.veh.d_hat = 0.35f;
  c.veh.dt = static_cast<float>(dt);
  c.veh.nsub = 1;
  c.freeze_sense = true;
}

void scatter(run_setup &s, int n, std::mt19937 &rng) {
  std::uniform_int_distribution<std::size_t> pick(0, s.free_cells.size() - 1);
  s.o.resize(2 * n);
  s.goal.resize(2 * n);
  s.col.assign(3 * n, 1.0f);
  for (int i = 0; i < n; ++i) {
    cell_to_normalized(s.cfg, s.free_cells[pick(rng)], s.o[2 * i], s.o[2 * i + 1]);
    cell_to_normalized(s.cfg, s.free_cells[pick(rng)], s.goal[2 * i], s.goal[2 * i + 1]);
  }
}

} // namespace

int main(int argc, char **argv) {
  options opt;
  try {
    opt = parse(argc, argv);
  } catch (const std::exception &e) {
    std::fprintf(stderr, "cvc-plant-log: %s\n", e.what());
    usage();
    return 2;
  }
  if (!phys::plant_host::available()) {
    std::fprintf(stderr, "cvc-plant-log: this libcvc was built without Jolt\n");
    return 1;
  }
  try {
    const phys::vehicle_desc desc = phys::plant_host::preset(opt.vehicle);
    const nav::coef_mlp policy =
        opt.policy.empty() ? nav::coef_mlp::default_biased() : nav::coef_mlp::load(opt.policy);
    const int steps_per_tick = static_cast<int>(std::lround(opt.dt / 0.01));
    if (steps_per_tick < 1 || std::fabs(steps_per_tick * 0.01 - opt.dt) > 1e-9)
      throw std::invalid_argument("--dt must be a whole number of 0.01 s physics steps");
    const int nsub = opt.nsub > 0 ? opt.nsub : steps_per_tick;
    const int ticks = static_cast<int>(std::lround(opt.seconds / opt.dt));

    // The city bundle is loaded once and shared by every seed.
    std::unique_ptr<cvc::world::city_bundle> city;
    nav::world_planes planes;
    if (opt.scene == "bundle") {
      city.reset(new cvc::world::city_bundle(cvc::world::load_city_bundle(opt.bundle)));
      if (!city->buildings_loaded)
        std::fprintf(stderr, "cvc-plant-log: no buildings (%s); terrain only\n",
                     city->buildings_error.c_str());
      planes = nav::rasterize(city->geometry, opt.grid, opt.grid, opt.inflate);
    }

    std::vector<std::vector<phys::planar_sample>> runs;
    double eff_accel_margin = 0.0, eff_brake_margin = 0.0; // what the plant planned with
    std::vector<unsigned> run_seed;
    std::vector<int> run_agent;
    for (const unsigned seed : opt.seeds) {
      std::mt19937 rng(seed);
      run_setup s;
      s.host.plant.plan_with_vehicle = true;
      s.host.ground_mu = opt.mu;
      const int n = opt.agents;
      if (opt.scene == "swerve") {
        // Open ground, 4 m cells over +-800 m; the trained metric (1 normalized unit = 20 m).
        s.cfg.rows = s.cfg.cols = 401;
        s.cfg.min_x = s.cfg.min_y = -800.0;
        s.cfg.max_x = s.cfg.max_y = 800.0;
        s.cfg.scale = 0.05;
        base_vehicle(s.cfg, opt.dt);
        std::uniform_real_distribution<double> frac(0.45, 0.85);
        s.vmax_mps = frac(rng) * desc.max_speed;
        s.cfg.veh.vmax = static_cast<float>(s.vmax_mps * s.cfg.scale);
        s.cfg.reach_tol = static_cast<float>(4.0 * s.cfg.scale);
        s.occ.assign(static_cast<std::size_t>(s.cfg.rows) * s.cfg.cols, 0);
        s.o.resize(2 * n);
        s.goal.resize(2 * n);
        s.col.assign(3 * n, 1.0f);
        for (int i = 0; i < n; ++i) {
          const double y0 = -300.0 + 600.0 * (i + 0.5) / n;
          s.o[2 * i] = static_cast<float>(-700.0 * s.cfg.scale);
          s.o[2 * i + 1] = static_cast<float>(y0 * s.cfg.scale);
          s.goal[2 * i] = static_cast<float>(-620.0 * s.cfg.scale);
          s.goal[2 * i + 1] = static_cast<float>((y0 + 30.0) * s.cfg.scale);
        }
      } else if (opt.scene == "city") {
        const nav::training_scene sc = nav::city_scene(96);
        s.cfg.rows = sc.rows;
        s.cfg.cols = sc.cols;
        s.cfg.min_x = sc.min_x;
        s.cfg.min_y = sc.min_y;
        s.cfg.max_x = sc.max_x;
        s.cfg.max_y = sc.max_y;
        s.cfg.scale = sc.scale;
        base_vehicle(s.cfg, opt.dt);
        s.cfg.veh.vmax = sc.vmax;
        s.cfg.reach_tol = 0.8f * 0.15f;
        // The city scaled to the vehicle, as the plant reach gate does: its wheelbase is
        // coef_train's veh_L (0.035 normalized).
        const double mpw = sc.scale / (0.035 / desc.wheelbase());
        s.host.plant.metres_per_world_unit = mpw;
        // The vehicle plans with its own top speed when it is below nav's.
        s.vmax_mps = std::min(sc.vmax / sc.scale * mpw, desc.max_speed);
        s.occ = sc.occ;
        s.free_cells = sc.free_cells;
        scatter(s, n, rng);
      } else {
        s.cfg.rows = planes.rows;
        s.cfg.cols = planes.cols;
        s.cfg.min_x = planes.min_x;
        s.cfg.min_y = planes.min_y;
        s.cfg.max_x = planes.max_x;
        s.cfg.max_y = planes.max_y;
        s.cfg.scale = 0.05;
        base_vehicle(s.cfg, opt.dt);
        std::uniform_real_distribution<double> frac(0.45, 0.8);
        s.vmax_mps = frac(rng) * desc.max_speed;
        s.cfg.veh.vmax = static_cast<float>(s.vmax_mps * s.cfg.scale);
        s.cfg.reach_tol = static_cast<float>(6.0 * s.cfg.scale);
        s.occ = planes.occupancy;
        // The physics terrain ends at the grid's edge: keep the agents 50 m inside it, where a
        // vehicle can neither spawn on the rim nor drive off it.
        const double cell = (planes.max_x - planes.min_x) / (planes.cols - 1);
        const int band = std::max(1, static_cast<int>(std::ceil(50.0 / cell)));
        for (int r = 0; r < s.cfg.rows; ++r)
          for (int k = 0; k < s.cfg.cols; ++k)
            if (r < band || k < band || r >= s.cfg.rows - band || k >= s.cfg.cols - band)
              s.occ[static_cast<std::size_t>(r) * s.cfg.cols + k] = 1;
        s.free_cells = largest_free_component(s.occ, s.cfg.rows, s.cfg.cols);
        if (s.free_cells.empty())
          throw std::runtime_error("the bundle's grid has no free cells");
        scatter(s, n, rng);
        s.geometry = &city->geometry;
        // Spawn on the terrain: its height under each start, bilinear on the grid.
        const nav::sim_world::config c = s.cfg;
        const std::vector<float> *hgt = &planes.height;
        s.host.plant.ground_height = [c, hgt](double x, double y) {
          const double fx = (x - c.min_x) / (c.max_x - c.min_x) * (c.cols - 1);
          const double fy = (y - c.min_y) / (c.max_y - c.min_y) * (c.rows - 1);
          const int k = std::max(0, std::min(c.cols - 2, static_cast<int>(std::floor(fx))));
          const int r = std::max(0, std::min(c.rows - 2, static_cast<int>(std::floor(fy))));
          const double ax = std::min(1.0, std::max(0.0, fx - k));
          const double ay = std::min(1.0, std::max(0.0, fy - r));
          const auto h = [&](int rr, int kk) { return (*hgt)[rr * c.cols + kk]; };
          return (1 - ay) * ((1 - ax) * h(r, k) + ax * h(r, k + 1)) +
                 ay * ((1 - ax) * h(r + 1, k) + ax * h(r + 1, k + 1));
        };
      }

      nav::sim_world sim(s.cfg, s.occ.data(), s.occ.data(), policy, s.o.data(), s.goal.data(),
                         s.col.data(), n);
      phys::plant_host host(s.host);
      if (s.geometry)
        host.set_geometry(s.geometry);
      host.use(sim, opt.vehicle);
      phys::nav_plant &plant = *host.plant();
      plant.set_lateral_g(opt.lateral_g);
      // The share of the vehicle's acceleration and braking nav plans with (nav_plant's knobs):
      // below 1, the commands stay gentler, e.g. where hard braking pitches a low chassis onto
      // the ground.
      if (opt.accel_margin > 0.0)
        plant.set_accel_margin(opt.accel_margin);
      if (opt.brake_margin > 0.0)
        plant.set_brake_margin(opt.brake_margin);
      eff_accel_margin = plant.options().accel_margin;
      eff_brake_margin = plant.options().brake_margin;
      phys::plant_log log(sim, plant);

      std::vector<float> pos(2 * n), gw(2 * n), hd(n);
      std::vector<std::uint8_t> reached(n);
      // Vehicle routes (city, bundle): the plant's footprint and turning circle, in world units.
      const bool routed = opt.scene == "bundle";
      nav::follower_options fo;
      nav::route_vehicle rv;
      std::vector<int> route_fail(n, 0);
      std::size_t planned = 0, unplanned = 0;
      if (routed) {
        const nav::plant_limits l = plant.limits(0);
        nav::sim_world::plant_plan pp;
        sim.plant_plan_of(0, pp);
        // The planner's step is at most 0.6 x the turning radius; a radius below 2.5 cells would
        // make a small vehicle's route cross the map in steps a fraction of a cell long. A wider
        // turn than the vehicle's own is always drivable.
        const double cell = (s.cfg.max_x - s.cfg.min_x) / (s.cfg.cols - 1);
        rv.min_radius = std::max(l.wheelbase / s.cfg.scale / std::tan(l.max_steer), 2.5 * cell);
        rv.disc_radius = pp.body_rr / s.cfg.scale;
        for (const float off : pp.body_offsets)
          rv.disc_offsets.push_back(off / s.cfg.scale);
        fo.wheelbase = l.wheelbase / s.cfg.scale;
        fo.max_steer = l.max_steer;
      }
      nav::route_follower follow(sim, fo);
      const double span = s.cfg.max_x - s.cfg.min_x;
      // A route from agent i's pose to a free cell 5-15% of the map away: the goal is drawn from
      // the cells in that ring (a draw from the whole map lands in it ~6% of the time), and the
      // planner gets a few of them.
      std::size_t no_goal = 0, planner_failed = 0;
      std::vector<int> ring;
      const auto plan_for = [&](int i) {
        ring.clear();
        const double dx_cell = span / (s.cfg.cols - 1),
                     dy_cell = (s.cfg.max_y - s.cfg.min_y) / (s.cfg.rows - 1);
        for (const int cell : s.free_cells) {
          const double gx = s.cfg.min_x + (cell % s.cfg.cols) * dx_cell;
          const double gy = s.cfg.min_y + (cell / s.cfg.cols) * dy_cell;
          const double dist = std::hypot(gx - pos[2 * i], gy - pos[2 * i + 1]);
          if (dist >= 0.05 * span && dist <= 0.15 * span)
            ring.push_back(cell);
        }
        if (ring.empty()) {
          ++no_goal;
          ++unplanned;
          ++route_fail[i];
          return;
        }
        std::uniform_int_distribution<std::size_t> pk(0, ring.size() - 1);
        for (int attempt = 0; attempt < 4; ++attempt) {
          const int cell = ring[pk(rng)];
          const double gx = s.cfg.min_x + (cell % s.cfg.cols) * dx_cell;
          const double gy = s.cfg.min_y + (cell / s.cfg.cols) * dy_cell;
          nav::route_options ro;
          ro.margin = 2.0 / s.host.plant.metres_per_world_unit; // 2 m off building meshes
          const nav::vehicle_route r = nav::plan_vehicle_route(
              s.occ.data(), s.cfg.rows, s.cfg.cols, s.cfg.min_x, s.cfg.min_y, s.cfg.max_x,
              s.cfg.max_y, rv, {pos[2 * i], pos[2 * i + 1], hd[i], false}, gx, gy, ro);
          if (r.reached && r.poses.size() > 2) {
            follow.set(i, r);
            ++planned;
            return;
          }
          ++planner_failed;
        }
        ++unplanned;
        ++route_fail[i];
      };
      std::uniform_real_distribution<double> amp(15.0, 45.0), adv(50.0, 90.0);
      std::vector<int> side(n, 1), dir(n, 1);
      // A routed agent stopped for 5 s (wedged against a building, or a leg it cannot drive) gets
      // a new route from where it stands; the planner leads a wedged start out.
      std::vector<int> still(n, 0);
      std::size_t replans = 0;
      const int stuck_ticks = static_cast<int>(std::lround(5.0 / opt.dt));
      const int settle = static_cast<int>(std::lround(0.5 / opt.dt));
      std::uniform_int_distribution<std::size_t> pick(
          0, s.free_cells.empty() ? 0 : s.free_cells.size() - 1);
      for (int t = -settle; t < ticks; ++t) {
        if (t == 0)
          log.start();
        sim.snapshot(pos.data(), hd.data(), nullptr, nullptr, reached.data());
        sim.goals_world(gw.data());
        for (int i = 0; i < n; ++i) {
          if (opt.scene == "swerve") {
            // The next gate once within 25 m of this one: ahead, on the other side.
            const double dx = pos[2 * i] - gw[2 * i], dy = pos[2 * i + 1] - gw[2 * i + 1];
            if (std::hypot(dx, dy) < 25.0) {
              if (gw[2 * i] > 650.0)
                dir[i] = -1;
              else if (gw[2 * i] < -650.0)
                dir[i] = 1;
              side[i] = -side[i];
              const double lane = -300.0 + 600.0 * (i + 0.5) / n;
              const double nx = gw[2 * i] + dir[i] * adv(rng), ny = lane + side[i] * amp(rng);
              sim.retarget(i, static_cast<float>(nx * s.cfg.scale),
                           static_cast<float>(ny * s.cfg.scale));
            }
          } else if (routed) {
            const bool stopped = t > 0 && std::fabs(log.runs()[i].back().vx) < 0.3;
            still[i] = stopped ? still[i] + 1 : 0;
            if (still[i] > stuck_ticks && follow.following(i) && !follow.done(i)) {
              follow.clear(i);
              still[i] = 0;
              ++replans;
            }
            if ((!follow.following(i) || follow.done(i)) && route_fail[i] < 20)
              plan_for(i);
          } else if (reached[i]) {
            float gx, gy;
            cell_to_normalized(s.cfg, s.free_cells[pick(rng)], gx, gy);
            sim.retarget(i, gx, gy);
          }
        }
        if (routed)
          follow.apply();
        sim.step();
        if (t >= 0)
          log.record();
        // --verbose: every agent's state every 5 s.
        if (opt.verbose && t > 0 && t % static_cast<int>(std::lround(5.0 / opt.dt)) == 0)
          for (int i = 0; i < n; ++i) {
            const phys::planar_sample &q = log.runs()[i].back();
            const phys::vehicle_state vs = plant.physics_world().vehicle_state(plant.vehicle(i));
            std::printf("  t %5.1f agent %d: (%7.1f, %7.1f, z %6.1f) vx %5.2f, wheels %d/%d, "
                        "contacts %d, grade %+.2f, cmd a %+6.2f steer %+.2f (has %+.2f), gear %d "
                        "throttle %.2f brake %.2f, route %s leg %d/%d%s\n",
                        t * opt.dt, i, q.x, q.y, vs.body.position.z, q.vx, q.wheels_down, q.wheels,
                        q.contacts, q.grade, q.accel_cmd, q.steer_cmd, q.steer, vs.gear,
                        vs.throttle, vs.brake, routed && follow.following(i) ? "on" : "off",
                        follow.leg(i), follow.legs(i), routed && follow.done(i) ? " done" : "");
          }
      }
      for (int i = 0; i < n; ++i) {
        runs.push_back(log.runs()[i]);
        run_seed.push_back(seed);
        run_agent.push_back(i);
      }
      std::size_t moving = 0;
      for (int i = 0; i < n; ++i)
        for (const phys::planar_sample &q : log.runs()[i])
          moving += std::fabs(q.vx) > 0.5 ? 1 : 0;
      std::printf("seed %u: %d agents, %d ticks, vmax %.1f m/s, moving %.0f%% of the time, %.0f us "
                  "a tick\n",
                  seed, n, ticks, s.vmax_mps,
                  100.0 * moving / (static_cast<double>(n) * (ticks + 1)), plant.advance_us());
      if (routed)
        std::printf("  routes: %zu planned, %zu requests unplanned (%zu with no goal in range), "
                    "%zu planner attempts failed, %zu replans of a stuck agent\n",
                    planned, unplanned, no_goal, planner_failed, replans);
      host.use(sim, "kinematic"); // detach before the host and sim go
    }

    // How often the wheels left the ground (by wheels down).
    {
      std::map<int, std::size_t> down;
      std::size_t total = 0;
      for (const auto &run : runs)
        for (const phys::planar_sample &q : run) {
          ++down[q.wheels - q.wheels_down];
          ++total;
        }
      std::printf("wheels off the ground:");
      for (const auto &kv : down)
        std::printf(" %d: %.1f%%", kv.first, 100.0 * kv.second / total);
      std::printf("\n");
    }

    // The flights.
    const std::size_t rows = static_cast<std::size_t>(std::lround(opt.flight_seconds / opt.dt)) + 1;
    const std::size_t stride = std::max<std::size_t>(
        1, static_cast<std::size_t>(std::lround(opt.stride_seconds / opt.dt)));
    const phys::flight_set f = phys::cut_flights(runs, rows, stride, 1.0, opt.steer);
    std::filesystem::create_directories(opt.out);
    phys::write_npy((std::filesystem::path(opt.out) / "flights.npy").string(), f.data,
                    {f.flights, f.rows, static_cast<std::size_t>(phys::kPlanarWidth)});

    // --samples: every sample of every run, for analysis (samples.csv).
    if (opt.samples) {
      std::ofstream cs(std::filesystem::path(opt.out) / "samples.csv");
      cs << "seed,agent,t,x,y,psi,vx,vy,r,rear_x,rear_y,steer,grade,roll,steer_cmd,accel_cmd,"
            "drive,vmax,a_lat,a_lon,contacts,wheels,wheels_down\n";
      char line[512];
      for (std::size_t k = 0; k < runs.size(); ++k)
        for (const phys::planar_sample &q : runs[k]) {
          std::snprintf(line, sizeof line,
                        "%u,%d,%.3f,%.4f,%.4f,%.5f,%.4f,%.4f,%.5f,%.4f,%.4f,%.5f,%.5f,%.5f,%.5f,"
                        "%.4f,%.4f,%.3f,%.4f,%.4f,%d,%d,%d\n",
                        run_seed[k], run_agent[k], q.t, q.x, q.y, q.psi, q.vx, q.vy, q.r, q.rear_x,
                        q.rear_y, q.steer, q.grade, q.roll, q.steer_cmd, q.accel_cmd, q.drive,
                        q.vmax, q.a_lat, q.a_lon, q.contacts, q.wheels, q.wheels_down);
          cs << line;
        }
    }

    // The residuals.
    const phys::plant_residuals res = phys::one_tick_residuals(runs, opt.dt, desc, opt.mu, nsub);
    const auto print_bins = [](const char *name, const std::vector<phys::residual_bin> &bins,
                               bool dyn) {
      std::printf("  %s\n    %-9s %7s %12s %12s %12s %12s %12s %10s %10s\n", name, "g", "ticks",
                  "pos rms m", "pos max m", "head rms", "head max", "speed rms", "vy rms", "r rms");
      for (const phys::residual_bin &b : bins) {
        if (!b.ticks)
          continue;
        char lab[32];
        if (std::isfinite(b.hi_g))
          std::snprintf(lab, sizeof lab, "%.1f-%.1f", b.lo_g, b.hi_g);
        else
          std::snprintf(lab, sizeof lab, ">= %.1f", b.lo_g);
        std::printf("    %-9s %7llu %12.4f %12.4f %12.5f %12.5f %12.4f", lab,
                    static_cast<unsigned long long>(b.ticks), b.position_rms_m, b.position_max_m,
                    b.heading_rms_rad, b.heading_max_rad, b.speed_rms_mps);
        if (dyn)
          std::printf(" %10.4f %10.4f", b.lateral_rms_mps, b.yaw_rate_rms);
        std::printf("\n");
      }
    };
    std::printf("one-tick residual against %s on Jolt (dt %.3f s, %d substeps, mu %.2f); ticks "
                "skipped: %llu static contact, %llu a wheel off the ground, %llu impact, %llu at "
                "rest\n",
                opt.vehicle.c_str(), opt.dt, nsub, opt.mu,
                static_cast<unsigned long long>(res.skipped_contact),
                static_cast<unsigned long long>(res.skipped_airborne),
                static_cast<unsigned long long>(res.skipped_impact),
                static_cast<unsigned long long>(res.skipped_rest));
    print_bins("diff::dyn_plant (dynamic single track)", res.dynamic, true);
    print_bins("kinematic bicycle", res.kinematic, false);

    // --fit-dyn: fit on every seed but the last, judge on the last.
    phys::dyn_bike_fit fit;
    phys::plant_residuals res_spec, res_fit;
    double held_default = 0.0, held_fit = 0.0;
    const unsigned held = opt.seeds.back();
    if (opt.fit_dyn) {
      std::vector<std::vector<phys::planar_sample>> fit_runs, test_runs;
      for (std::size_t k = 0; k < runs.size(); ++k)
        (opt.seeds.size() > 1 && run_seed[k] == held ? test_runs : fit_runs).push_back(runs[k]);
      if (test_runs.empty())
        test_runs = fit_runs; // one seed: in-sample
      fit = phys::fit_dyn_bike(fit_runs, opt.dt, desc, opt.mu, nsub);
      const cvc::nav::dyn_bike_si spec = phys::nav_plant::dyn_bike_si_of(desc, opt.mu);
      held_default = phys::dyn_bike_cost(test_runs, opt.dt, spec, desc.max_steer, nsub);
      held_fit = phys::dyn_bike_cost(test_runs, opt.dt, fit.si, desc.max_steer, nsub);
      std::printf("dyn_bike fitted to %s on %zu runs (%llu ticks): drive lag %.3f s (spec %.3f), "
                  "steer lag %.3f s, tire B x%.2f front / x%.2f rear, yaw inertia x%.2f, drag and "
                  "rolling resistance x%.2f\n"
                  "  mean squared twist residual (m/s)^2: fit set %.5f -> %.5f; held-out seed %u "
                  "%.5f -> %.5f\n",
                  opt.vehicle.c_str(), fit_runs.size(), static_cast<unsigned long long>(fit.ticks),
                  fit.drive_lag, spec.drive_lag, fit.steer_lag, fit.tire_b_front_factor,
                  fit.tire_b_rear_factor, fit.yaw_inertia_factor, fit.resistance_factor,
                  fit.cost_default, fit.cost_fit, held, held_default, held_fit);
      res_spec =
          phys::one_tick_residuals(test_runs, opt.dt, spec, desc.wheelbase(), desc.max_steer, nsub);
      res_fit = phys::one_tick_residuals(test_runs, opt.dt, fit.si, desc.wheelbase(),
                                         desc.max_steer, nsub);
      std::printf("held-out seed %u:\n", held);
      print_bins("diff::dyn_plant, spec sheet", res_spec.dynamic, true);
      print_bins("diff::dyn_plant, fitted", res_fit.dynamic, true);
      print_bins("kinematic bicycle", res_spec.kinematic, false);
    }
    std::printf("flights: %zu x %zu rows (%.1f s at %.0f Hz), %zu windows dropped, peak %.2f g\n",
                f.flights, f.rows, (f.rows - 1) * opt.dt, 1.0 / opt.dt, f.dropped,
                f.peak_lateral_g);

    // residual.json and meta.json.
    const auto num = [](double x) {
      if (!std::isfinite(x))
        return std::string("null");
      char b[40];
      std::snprintf(b, sizeof b, "%.9g", x);
      return std::string(b);
    };
    const auto bins_json = [&](const std::vector<phys::residual_bin> &bins) {
      std::string s = "[";
      for (std::size_t k = 0; k < bins.size(); ++k) {
        const phys::residual_bin &b = bins[k];
        s += std::string(k ? ",\n   " : "") + "{\"lo_g\": " + num(b.lo_g) +
             ", \"hi_g\": " + num(b.hi_g) + ", \"ticks\": " + std::to_string(b.ticks) +
             ", \"position_rms_m\": " + num(b.position_rms_m) +
             ", \"position_max_m\": " + num(b.position_max_m) +
             ", \"heading_rms_rad\": " + num(b.heading_rms_rad) +
             ", \"heading_max_rad\": " + num(b.heading_max_rad) +
             ", \"speed_rms_mps\": " + num(b.speed_rms_mps) +
             ", \"speed_max_mps\": " + num(b.speed_max_mps) +
             ", \"lateral_rms_mps\": " + num(b.lateral_rms_mps) +
             ", \"yaw_rate_rms\": " + num(b.yaw_rate_rms) + "}";
      }
      return s + "]";
    };
    {
      std::ofstream js(std::filesystem::path(opt.out) / "residual.json");
      js << "{\n \"format\": \"cvc-plant-residual-1\",\n \"vehicle\": \"" << opt.vehicle
         << "\", \"scene\": \"" << opt.scene << "\", \"dt\": " << num(opt.dt)
         << ", \"nsub\": " << nsub << ", \"mu\": " << num(opt.mu)
         << ",\n \"skipped\": {\"contact\": " << res.skipped_contact
         << ", \"airborne\": " << res.skipped_airborne << ", \"impact\": " << res.skipped_impact
         << ", \"rest\": " << res.skipped_rest << "},\n \"dynamic\": " << bins_json(res.dynamic)
         << ",\n \"kinematic\": " << bins_json(res.kinematic);
      if (opt.fit_dyn)
        js << ",\n \"fit\": {\"held_out_seed\": " << held
           << ", \"drive_lag\": " << num(fit.drive_lag) << ", \"steer_lag\": " << num(fit.steer_lag)
           << ", \"tire_b_front_factor\": " << num(fit.tire_b_front_factor)
           << ", \"tire_b_rear_factor\": " << num(fit.tire_b_rear_factor)
           << ", \"yaw_inertia_factor\": " << num(fit.yaw_inertia_factor)
           << ", \"resistance_factor\": " << num(fit.resistance_factor) << ", \"cost_fit_set\": ["
           << num(fit.cost_default) << ", " << num(fit.cost_fit) << "], \"cost_held_out\": ["
           << num(held_default) << ", " << num(held_fit)
           << "],\n  \"held_out_dynamic_spec\": " << bins_json(res_spec.dynamic)
           << ",\n  \"held_out_dynamic_fit\": " << bins_json(res_fit.dynamic)
           << ",\n  \"held_out_kinematic\": " << bins_json(res_spec.kinematic) << "}";
      js << "\n}\n";
    }
    {
      std::ofstream js(std::filesystem::path(opt.out) / "meta.json");
      js << "{\n \"format\": \"cvc-plant-log-1\",\n \"vehicle\": \"" << opt.vehicle
         << "\", \"scene\": \"" << opt.scene << "\", \"bundle\": \"" << opt.bundle
         << "\",\n \"dt\": " << num(opt.dt) << ", \"rate_hz\": " << num(1.0 / opt.dt)
         << ", \"physics_dt\": 0.01, \"mu\": " << num(opt.mu)
         << ", \"lateral_g\": " << num(opt.lateral_g) << ", \"seconds\": " << num(opt.seconds)
         << ", \"accel_margin\": " << num(eff_accel_margin)
         << ", \"brake_margin\": " << num(eff_brake_margin)
         << ",\n \"layout\": [\"x\", \"y\", \"psi\", \"vx\", \"vy\", \"r\", \"drive\", \"steer\"],"
         << "\n \"units\": [\"m\", \"m\", \"rad\", \"m/s\", \"m/s\", \"rad/s\", \"m/s^2\", "
            "\"rad\"],"
         << "\n \"frame\": \"physics world; (x, y) the centre of mass, (vx, vy) its velocity in "
            "the body frame, psi continuous\","
         << "\n \"inputs\": \"row k's acted from t_k-1 to t_k; drive = nav's acceleration "
            "command (zero-order hold over the tick) within the vehicle's -max_decel..max_accel, "
            "steer = "
         << (opt.steer == phys::steer_input::actual ? "the road-wheel angle's mean over the tick"
                                                    : "nav's steering command (held)")
         << "\",\n \"shape\": [" << f.flights << ", " << f.rows << ", " << phys::kPlanarWidth
         << "], \"dropped\": " << f.dropped << ", \"peak_lateral_g\": " << num(f.peak_lateral_g)
         << ",\n \"seeds\": [";
      for (std::size_t k = 0; k < opt.seeds.size(); ++k)
        js << (k ? ", " : "") << opt.seeds[k];
      js << "],\n \"flight_seed\": [";
      for (std::size_t k = 0; k < f.flights; ++k)
        js << (k ? ", " : "") << run_seed[f.agent[k]];
      js << "],\n \"flight_agent\": [";
      for (std::size_t k = 0; k < f.flights; ++k)
        js << (k ? ", " : "") << run_agent[f.agent[k]];
      js << "],\n \"flight_first\": [";
      for (std::size_t k = 0; k < f.flights; ++k)
        js << (k ? ", " : "") << f.first[k];
      js << "]\n}\n";
    }
    std::printf("wrote %s/{flights.npy, meta.json, residual.json}\n", opt.out.c_str());
  } catch (const std::exception &e) {
    std::fprintf(stderr, "cvc-plant-log: %s\n", e.what());
    return 1;
  }
  return 0;
}
