/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cvc/nav/coef_train.h>
#include <cvc/nav/route_follower.h>
#include <cvc/nav/sim_world.h>
#include <cvc/nav/vehicle_route.h>
#include <cvc/nav/world_planes.h>
#include <cvc/physics/nav_plant.h>
#include <cvc/physics/plant_host.h>
#include <cvc/physics/plant_log_scene.h>
#include <cvc/physics/world_geometry.h>
#include <cvc/world/city_bundle.h>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace cvc {
namespace physics {

namespace {

namespace nav = cvc::nav;

void say(const plant_log_options &o, const char *fmt, ...) {
  if (!o.report)
    return;
  va_list ap;
  va_start(ap, fmt);
  std::vfprintf(o.report, fmt, ap);
  va_end(ap);
}

int steps_per_tick(double dt) { return static_cast<int>(std::lround(dt / 0.01)); }

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
  plant_host_options host;         // the physics world and the plant's options
  const cvc::world::world_geometry *geometry = nullptr;
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

void plant_log_options::resolve() {
  if (scene != "swerve" && scene != "city" && scene != "bundle")
    throw std::invalid_argument("the scene is swerve, city or bundle");
  if (scene == "bundle" && bundle.empty())
    throw std::invalid_argument("the bundle scene needs a city bundle directory");
  if (seeds.empty())
    throw std::invalid_argument("no seeds");
  const int steps = steps_per_tick(dt);
  if (steps < 1 || std::fabs(steps * 0.01 - dt) > 1e-9)
    throw std::invalid_argument("dt must be a whole number of 0.01 s physics steps");
  if (agents < 0)
    agents = scene == "swerve" ? 4 : 8;
  if (lateral_g < 0.0)
    lateral_g = scene == "swerve" ? 0.9 : 0.5;
  if (nsub <= 0)
    nsub = steps;
}

plant_log_drive drive_plant_log_scene(const plant_log_options &o, const nav::coef_mlp &policy) {
  if (!plant_host::available())
    throw std::runtime_error("this libcvc was built without Jolt");
  const vehicle_desc desc = plant_host::preset(o.vehicle);
  const int ticks = static_cast<int>(std::lround(o.seconds / o.dt));

  // The city bundle is loaded once and shared by every seed.
  std::unique_ptr<cvc::world::city_bundle> city;
  nav::world_planes planes;
  if (o.scene == "bundle") {
    city.reset(new cvc::world::city_bundle(cvc::world::load_city_bundle(o.bundle)));
    if (!city->buildings_loaded)
      say(o, "no buildings (%s); terrain only\n", city->buildings_error.c_str());
    planes = nav::rasterize(city->geometry, o.grid, o.grid, o.inflate);
  }

  plant_log_drive d;
  for (const unsigned seed : o.seeds) {
    std::mt19937 rng(seed);
    run_setup s;
    s.host.plant.plan_with_vehicle = true;
    s.host.ground_mu = o.mu;
    const int n = o.agents;
    if (o.scene == "swerve") {
      // Open ground, 4 m cells over +-800 m; the trained metric (1 normalized unit = 20 m).
      s.cfg.rows = s.cfg.cols = 401;
      s.cfg.min_x = s.cfg.min_y = -800.0;
      s.cfg.max_x = s.cfg.max_y = 800.0;
      s.cfg.scale = 0.05;
      base_vehicle(s.cfg, o.dt);
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
    } else if (o.scene == "city") {
      const nav::training_scene sc = nav::city_scene(96);
      s.cfg.rows = sc.rows;
      s.cfg.cols = sc.cols;
      s.cfg.min_x = sc.min_x;
      s.cfg.min_y = sc.min_y;
      s.cfg.max_x = sc.max_x;
      s.cfg.max_y = sc.max_y;
      s.cfg.scale = sc.scale;
      base_vehicle(s.cfg, o.dt);
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
      base_vehicle(s.cfg, o.dt);
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
    plant_host host(s.host);
    if (s.geometry)
      host.set_geometry(s.geometry);
    host.use(sim, o.vehicle);
    nav_plant &plant = *host.plant();
    plant.set_lateral_g(o.lateral_g);
    // The share of the vehicle's acceleration and braking nav plans with (nav_plant's knobs):
    // below 1, the commands stay gentler, e.g. where hard braking pitches a low chassis onto
    // the ground.
    if (o.accel_margin > 0.0)
      plant.set_accel_margin(o.accel_margin);
    if (o.brake_margin > 0.0)
      plant.set_brake_margin(o.brake_margin);
    d.accel_margin = plant.options().accel_margin;
    d.brake_margin = plant.options().brake_margin;
    plant_log log(sim, plant);

    std::vector<float> pos(2 * n), gw(2 * n), hd(n);
    std::vector<std::uint8_t> reached(n);
    // Vehicle routes (bundle): the plant's footprint and turning circle, in world units.
    const bool routed = o.scene == "bundle";
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
    const int stuck_ticks = static_cast<int>(std::lround(5.0 / o.dt));
    const int settle = static_cast<int>(std::lround(0.5 / o.dt));
    std::uniform_int_distribution<std::size_t> pick(
        0, s.free_cells.empty() ? 0 : s.free_cells.size() - 1);
    for (int t = -settle; t < ticks; ++t) {
      if (t == 0)
        log.start();
      sim.snapshot(pos.data(), hd.data(), nullptr, nullptr, reached.data());
      sim.goals_world(gw.data());
      for (int i = 0; i < n; ++i) {
        if (o.scene == "swerve") {
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
      // verbose: every agent's state every 5 s.
      if (o.verbose && t > 0 && t % static_cast<int>(std::lround(5.0 / o.dt)) == 0)
        for (int i = 0; i < n; ++i) {
          const planar_sample &q = log.runs()[i].back();
          const vehicle_state vs = plant.physics_world().vehicle_state(plant.vehicle(i));
          say(o,
              "  t %5.1f agent %d: (%7.1f, %7.1f, z %6.1f) vx %5.2f, wheels %d/%d, "
              "contacts %d, grade %+.2f, cmd a %+6.2f steer %+.2f (has %+.2f), gear %d "
              "throttle %.2f brake %.2f, route %s leg %d/%d%s\n",
              t * o.dt, i, q.x, q.y, vs.body.position.z, q.vx, q.wheels_down, q.wheels, q.contacts,
              q.grade, q.accel_cmd, q.steer_cmd, q.steer, vs.gear, vs.throttle, vs.brake,
              routed && follow.following(i) ? "on" : "off", follow.leg(i), follow.legs(i),
              routed && follow.done(i) ? " done" : "");
        }
    }
    for (int i = 0; i < n; ++i) {
      d.runs.push_back(log.runs()[i]);
      d.run_seed.push_back(seed);
      d.run_agent.push_back(i);
    }
    std::size_t moving = 0;
    for (int i = 0; i < n; ++i)
      for (const planar_sample &q : log.runs()[i])
        moving += std::fabs(q.vx) > 0.5 ? 1 : 0;
    say(o,
        "seed %u: %d agents, %d ticks, vmax %.1f m/s, moving %.0f%% of the time, %.0f us "
        "a tick\n",
        seed, n, ticks, s.vmax_mps, 100.0 * moving / (static_cast<double>(n) * (ticks + 1)),
        plant.advance_us());
    if (routed)
      say(o,
          "  routes: %zu planned, %zu requests unplanned (%zu with no goal in range), "
          "%zu planner attempts failed, %zu replans of a stuck agent\n",
          planned, unplanned, no_goal, planner_failed, replans);
    host.use(sim, "kinematic"); // detach before the host and sim go
  }
  return d;
}

void write_plant_log(const plant_log_options &o, const plant_log_drive &d) {
  if (o.out.empty())
    throw std::invalid_argument("no output directory");
  const vehicle_desc desc = plant_host::preset(o.vehicle);
  const std::vector<std::vector<planar_sample>> &runs = d.runs;

  // How often the wheels left the ground (by wheels down).
  {
    std::map<int, std::size_t> down;
    std::size_t total = 0;
    for (const auto &run : runs)
      for (const planar_sample &q : run) {
        ++down[q.wheels - q.wheels_down];
        ++total;
      }
    say(o, "wheels off the ground:");
    for (const auto &kv : down)
      say(o, " %d: %.1f%%", kv.first, 100.0 * kv.second / total);
    say(o, "\n");
  }

  // The flights.
  const std::size_t rows = static_cast<std::size_t>(std::lround(o.flight_seconds / o.dt)) + 1;
  const std::size_t stride =
      std::max<std::size_t>(1, static_cast<std::size_t>(std::lround(o.stride_seconds / o.dt)));
  const flight_set f = cut_flights(runs, rows, stride, 1.0, o.steer);
  std::filesystem::create_directories(o.out);
  write_npy((std::filesystem::path(o.out) / "flights.npy").string(), f.data,
            {f.flights, f.rows, static_cast<std::size_t>(kPlanarWidth)});

  // samples: every sample of every run, for analysis (samples.csv).
  if (o.samples) {
    std::ofstream cs(std::filesystem::path(o.out) / "samples.csv");
    cs << "seed,agent,t,x,y,psi,vx,vy,r,rear_x,rear_y,steer,grade,roll,steer_cmd,accel_cmd,"
          "drive,vmax,a_lat,a_lon,contacts,wheels,wheels_down,energy\n";
    char line[512];
    for (std::size_t k = 0; k < runs.size(); ++k)
      for (const planar_sample &q : runs[k]) {
        std::snprintf(line, sizeof line,
                      "%u,%d,%.3f,%.4f,%.4f,%.5f,%.4f,%.4f,%.5f,%.4f,%.4f,%.5f,%.5f,%.5f,%.5f,"
                      "%.4f,%.4f,%.3f,%.4f,%.4f,%d,%d,%d,%.6g\n",
                      d.run_seed[k], d.run_agent[k], q.t, q.x, q.y, q.psi, q.vx, q.vy, q.r,
                      q.rear_x, q.rear_y, q.steer, q.grade, q.roll, q.steer_cmd, q.accel_cmd,
                      q.drive, q.vmax, q.a_lat, q.a_lon, q.contacts, q.wheels, q.wheels_down,
                      q.energy);
        cs << line;
      }
  }

  // The residuals.
  const plant_residuals res = one_tick_residuals(runs, o.dt, desc, o.mu, o.nsub);
  const auto print_bins = [&o](const char *name, const std::vector<residual_bin> &bins, bool dyn) {
    say(o, "  %s\n    %-9s %7s %12s %12s %12s %12s %12s %10s %10s\n", name, "g", "ticks",
        "pos rms m", "pos max m", "head rms", "head max", "speed rms", "vy rms", "r rms");
    for (const residual_bin &b : bins) {
      if (!b.ticks)
        continue;
      char lab[32];
      if (std::isfinite(b.hi_g))
        std::snprintf(lab, sizeof lab, "%.1f-%.1f", b.lo_g, b.hi_g);
      else
        std::snprintf(lab, sizeof lab, ">= %.1f", b.lo_g);
      say(o, "    %-9s %7llu %12.4f %12.4f %12.5f %12.5f %12.4f", lab,
          static_cast<unsigned long long>(b.ticks), b.position_rms_m, b.position_max_m,
          b.heading_rms_rad, b.heading_max_rad, b.speed_rms_mps);
      if (dyn)
        say(o, " %10.4f %10.4f", b.lateral_rms_mps, b.yaw_rate_rms);
      say(o, "\n");
    }
  };
  say(o,
      "one-tick residual against %s on Jolt (dt %.3f s, %d substeps, mu %.2f); ticks "
      "skipped: %llu static contact, %llu a wheel off the ground, %llu impact, %llu at "
      "rest\n",
      o.vehicle.c_str(), o.dt, o.nsub, o.mu, static_cast<unsigned long long>(res.skipped_contact),
      static_cast<unsigned long long>(res.skipped_airborne),
      static_cast<unsigned long long>(res.skipped_impact),
      static_cast<unsigned long long>(res.skipped_rest));
  print_bins("diff::dyn_plant (dynamic single track)", res.dynamic, true);
  print_bins("kinematic bicycle", res.kinematic, false);

  // fit_dyn: fit on every seed but the last, judge on the last.
  dyn_bike_fit fit;
  plant_residuals res_spec, res_fit;
  double held_default = 0.0, held_fit = 0.0;
  const unsigned held = o.seeds.back();
  if (o.fit_dyn) {
    std::vector<std::vector<planar_sample>> fit_runs, test_runs;
    for (std::size_t k = 0; k < runs.size(); ++k)
      (o.seeds.size() > 1 && d.run_seed[k] == held ? test_runs : fit_runs).push_back(runs[k]);
    if (test_runs.empty())
      test_runs = fit_runs; // one seed: in-sample
    fit = fit_dyn_bike(fit_runs, o.dt, desc, o.mu, o.nsub);
    const cvc::nav::dyn_bike_si spec = nav_plant::dyn_bike_si_of(desc, o.mu);
    held_default = dyn_bike_cost(test_runs, o.dt, spec, desc.max_steer, o.nsub);
    held_fit = dyn_bike_cost(test_runs, o.dt, fit.si, desc.max_steer, o.nsub);
    say(o,
        "dyn_bike fitted to %s on %zu runs (%llu ticks): drive lag %.3f s (spec %.3f), "
        "steer lag %.3f s, tire B x%.2f front / x%.2f rear, yaw inertia x%.2f, drag and "
        "rolling resistance x%.2f\n"
        "  mean squared twist residual (m/s)^2: fit set %.5f -> %.5f; held-out seed %u "
        "%.5f -> %.5f\n",
        o.vehicle.c_str(), fit_runs.size(), static_cast<unsigned long long>(fit.ticks),
        fit.drive_lag, spec.drive_lag, fit.steer_lag, fit.tire_b_front_factor,
        fit.tire_b_rear_factor, fit.yaw_inertia_factor, fit.resistance_factor, fit.cost_default,
        fit.cost_fit, held, held_default, held_fit);
    res_spec = one_tick_residuals(test_runs, o.dt, spec, desc.wheelbase(), desc.max_steer, o.nsub);
    res_fit = one_tick_residuals(test_runs, o.dt, fit.si, desc.wheelbase(), desc.max_steer, o.nsub);
    say(o, "held-out seed %u:\n", held);
    print_bins("diff::dyn_plant, spec sheet", res_spec.dynamic, true);
    print_bins("diff::dyn_plant, fitted", res_fit.dynamic, true);
    print_bins("kinematic bicycle", res_spec.kinematic, false);
  }
  // fit_power: the power model fitted to the ledger of every seed but the last, judged on the
  // last.
  power_fit pfit, pheld;
  if (o.fit_power) {
    std::vector<std::vector<planar_sample>> fit_runs, test_runs;
    for (std::size_t k = 0; k < runs.size(); ++k)
      (o.seeds.size() > 1 && d.run_seed[k] == held ? test_runs : fit_runs).push_back(runs[k]);
    if (test_runs.empty())
      test_runs = fit_runs; // one seed: in-sample
    pfit = fit_power(fit_runs, o.dt, desc);
    pheld = evaluate_power(test_runs, o.dt, desc, pfit.si);
    say(o,
        "power model fitted to %s's ledger on %zu runs (%llu ticks): k_pos %.3f, k_neg %.3f, "
        "p_idle %.1f W, c_v %.3f W/(m/s), c_v3 %.5f W/(m/s)^3, c_lat %.5f s^2/m\n"
        "  tick r2 %.3f; held-out seed %u: r2 %.3f, total %+.2f%%, worst run %.1f%%\n",
        o.vehicle.c_str(), fit_runs.size(), static_cast<unsigned long long>(pfit.ticks),
        pfit.si.k_pos, pfit.si.k_neg, pfit.si.p_idle, pfit.si.c_v, pfit.si.c_v3, pfit.si.c_lat,
        pfit.r2, held, pheld.r2, 100.0 * pheld.total_error, 100.0 * pheld.worst_run_error);
  }
  say(o, "flights: %zu x %zu rows (%.1f s at %.0f Hz), %zu windows dropped, peak %.2f g\n",
      f.flights, f.rows, (f.rows - 1) * o.dt, 1.0 / o.dt, f.dropped, f.peak_lateral_g);

  // residual.json and meta.json.
  const auto num = [](double x) {
    if (!std::isfinite(x))
      return std::string("null");
    char b[40];
    std::snprintf(b, sizeof b, "%.9g", x);
    return std::string(b);
  };
  const auto bins_json = [&](const std::vector<residual_bin> &bins) {
    std::string s = "[";
    for (std::size_t k = 0; k < bins.size(); ++k) {
      const residual_bin &b = bins[k];
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
    std::ofstream js(std::filesystem::path(o.out) / "residual.json");
    js << "{\n \"format\": \"cvc-plant-residual-1\",\n \"vehicle\": \"" << o.vehicle
       << "\", \"scene\": \"" << o.scene << "\", \"dt\": " << num(o.dt) << ", \"nsub\": " << o.nsub
       << ", \"mu\": " << num(o.mu) << ",\n \"skipped\": {\"contact\": " << res.skipped_contact
       << ", \"airborne\": " << res.skipped_airborne << ", \"impact\": " << res.skipped_impact
       << ", \"rest\": " << res.skipped_rest << "},\n \"dynamic\": " << bins_json(res.dynamic)
       << ",\n \"kinematic\": " << bins_json(res.kinematic);
    if (o.fit_dyn)
      js << ",\n \"fit\": {\"held_out_seed\": " << held << ", \"drive_lag\": " << num(fit.drive_lag)
         << ", \"steer_lag\": " << num(fit.steer_lag)
         << ", \"tire_b_front_factor\": " << num(fit.tire_b_front_factor)
         << ", \"tire_b_rear_factor\": " << num(fit.tire_b_rear_factor)
         << ", \"yaw_inertia_factor\": " << num(fit.yaw_inertia_factor)
         << ", \"resistance_factor\": " << num(fit.resistance_factor) << ", \"cost_fit_set\": ["
         << num(fit.cost_default) << ", " << num(fit.cost_fit) << "], \"cost_held_out\": ["
         << num(held_default) << ", " << num(held_fit)
         << "],\n  \"held_out_dynamic_spec\": " << bins_json(res_spec.dynamic)
         << ",\n  \"held_out_dynamic_fit\": " << bins_json(res_fit.dynamic)
         << ",\n  \"held_out_kinematic\": " << bins_json(res_spec.kinematic) << "}";
    if (o.fit_power)
      js << ",\n \"power\": {\"held_out_seed\": " << held << ", \"k_pos\": " << num(pfit.si.k_pos)
         << ", \"k_neg\": " << num(pfit.si.k_neg) << ", \"p_idle_W\": " << num(pfit.si.p_idle)
         << ", \"c_v\": " << num(pfit.si.c_v) << ", \"c_v3\": " << num(pfit.si.c_v3)
         << ", \"c_lat\": " << num(pfit.si.c_lat) << ", \"width_W\": " << num(pfit.si.width)
         << ", \"ticks\": " << pfit.ticks << ", \"r2_fit_set\": " << num(pfit.r2)
         << ", \"r2_held_out\": " << num(pheld.r2)
         << ", \"total_error_held_out\": " << num(pheld.total_error)
         << ", \"worst_run_error_held_out\": " << num(pheld.worst_run_error) << "}";
    js << "\n}\n";
  }
  {
    std::ofstream js(std::filesystem::path(o.out) / "meta.json");
    js << "{\n \"format\": \"cvc-plant-log-1\",\n \"vehicle\": \"" << o.vehicle
       << "\", \"scene\": \"" << o.scene << "\", \"bundle\": \"" << o.bundle
       << "\",\n \"dt\": " << num(o.dt) << ", \"rate_hz\": " << num(1.0 / o.dt)
       << ", \"physics_dt\": 0.01, \"mu\": " << num(o.mu) << ", \"lateral_g\": " << num(o.lateral_g)
       << ", \"seconds\": " << num(o.seconds) << ", \"accel_margin\": " << num(d.accel_margin)
       << ", \"brake_margin\": " << num(d.brake_margin)
       << ",\n \"layout\": [\"x\", \"y\", \"psi\", \"vx\", \"vy\", \"r\", \"drive\", \"steer\"],"
       << "\n \"units\": [\"m\", \"m\", \"rad\", \"m/s\", \"m/s\", \"rad/s\", \"m/s^2\", "
          "\"rad\"],"
       << "\n \"frame\": \"physics world; (x, y) the centre of mass, (vx, vy) its velocity in "
          "the body frame, psi continuous\","
       << "\n \"inputs\": \"row k's acted from t_k-1 to t_k; drive = nav's acceleration "
          "command (zero-order hold over the tick) within the vehicle's -max_decel..max_accel, "
          "steer = "
       << (o.steer == steer_input::actual ? "the road-wheel angle's mean over the tick"
                                          : "nav's steering command (held)")
       << "\",\n \"shape\": [" << f.flights << ", " << f.rows << ", " << kPlanarWidth
       << "], \"dropped\": " << f.dropped << ", \"peak_lateral_g\": " << num(f.peak_lateral_g)
       << ",\n \"seeds\": [";
    for (std::size_t k = 0; k < o.seeds.size(); ++k)
      js << (k ? ", " : "") << o.seeds[k];
    js << "],\n \"flight_seed\": [";
    for (std::size_t k = 0; k < f.flights; ++k)
      js << (k ? ", " : "") << d.run_seed[f.agent[k]];
    js << "],\n \"flight_agent\": [";
    for (std::size_t k = 0; k < f.flights; ++k)
      js << (k ? ", " : "") << d.run_agent[f.agent[k]];
    js << "],\n \"flight_first\": [";
    for (std::size_t k = 0; k < f.flights; ++k)
      js << (k ? ", " : "") << f.first[k];
    js << "]\n}\n";
  }
  say(o, "wrote %s/{flights.npy, meta.json, residual.json}\n", o.out.c_str());
}

} // namespace physics
} // namespace cvc
