/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// C5 part 2b (docs/roadmap/NAV-COPH-ROADMAP.md §4.5): coph_driver, CoPH driving a sim_world's
// vehicles through override_command. C++ only: the driver is the twin of GRL-SNAM's cars
// controller, whose learner G13 pins (nav_coph_cars_test) and whose allocation G11 pins.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/coph_driver.h>
#include <cvc/nav/sim_world.h>
#include <gtest/gtest.h>
#include <vector>

using namespace cvc::nav;

namespace {

// A RoboRacer's world at s = 1 (D3): 20 m x 12 m of 0.1 m cells, walled round its edge, with or
// without a 1 m block in the middle (x 9.5..10.5, y 5.6..6.6).
constexpr double kS = 0.1;                              // world -> normalized
constexpr double kL = 0.33, kLock = 0.42, kAccel = 3.0; // m, rad, m/s^2

struct field {
  static constexpr double W = 20.0, H = 12.0, cell = 0.1;
  int cols = static_cast<int>(W / cell) + 1, rows = static_cast<int>(H / cell) + 1;
  std::vector<std::uint8_t> occ;
  void box(double x0, double y0, double x1, double y1) {
    for (int r = 0; r < rows; ++r)
      for (int c = 0; c < cols; ++c) {
        const double x = c * cell, y = r * cell;
        if (x >= x0 - 1e-9 && x <= x1 + 1e-9 && y >= y0 - 1e-9 && y <= y1 + 1e-9)
          occ[static_cast<std::size_t>(r) * cols + c] = 1;
      }
  }
  explicit field(bool block) : occ(static_cast<std::size_t>((W / cell + 1) * (H / cell + 1)), 0) {
    box(0, 0, W, 0);
    box(0, H, W, H);
    box(0, 0, 0, H);
    box(W, 0, W, H);
    if (block)
      box(9.5, 5.6, 10.5, 6.6);
  }
};

sim_world::config racer_config(const field &f) {
  sim_world::config c;
  c.rows = f.rows;
  c.cols = f.cols;
  c.min_x = c.min_y = 0.0;
  c.max_x = field::W;
  c.max_y = field::H;
  c.cx = 0.5 * field::W;
  c.cy = 0.5 * field::H;
  c.scale = kS;
  c.veh.rr = static_cast<float>(0.25 * kS);
  c.veh.d_hat = 0.35f;
  c.veh.dt = 0.06f;
  c.veh.nsub = 6;
  c.veh.vmax = static_cast<float>(3.0 * kS);
  c.veh.L = static_cast<float>(kL * kS);
  c.veh.delta_max = static_cast<float>(kLock);
  c.veh.a_max = static_cast<float>(kAccel * kS);
  c.veh.a_lat_max = static_cast<float>(0.8 * 9.80665 * kS);
  c.veh.allow_reverse = true;
  c.freeze_sense = true;
  c.reach_tol = static_cast<float>(0.3 * kS);
  return c;
}

struct start {
  double x, y, heading, gx, gy;
};

sim_world make(const field &f, const std::vector<start> &cars) {
  const sim_world::config c = racer_config(f);
  auto n = [&](double v, double centre) { return static_cast<float>((v - centre) * kS); };
  std::vector<float> o, goal, col;
  for (const start &s : cars) {
    o.insert(o.end(), {n(s.x, c.cx), n(s.y, c.cy)});
    goal.insert(goal.end(), {n(s.gx, c.cx), n(s.gy, c.cy)});
    col.insert(col.end(), {1, 1, 1});
  }
  const int count = static_cast<int>(cars.size());
  sim_world sim(c, f.occ.data(), f.occ.data(), coef_mlp::default_biased(), o.data(), goal.data(),
                col.data(), count);
  for (int i = 0; i < count; ++i)
    sim.set_pose(i, o[2 * i], o[2 * i + 1], static_cast<float>(cars[i].heading));
  return sim;
}

// Two cars passing each other on lanes 1.4 m apart: car 0 east from (3, 5) to (17, 5.4), car 1 west
// from (17, 7) to (3, 6.6).
sim_world crossing(const field &f) {
  return make(f, {{3.0, 5.0, 0.0, 17.0, 5.4}, {17.0, 7.0, 3.14159265, 3.0, 6.6}});
}

coph_config profile(int steps) {
  coph_config cfg;
  cfg.controller_mode = "routed";
  cfg.shaping_mode = "progress";
  cfg.replay_objective = "mission";
  cfg.replay_domain = "retained";
  cfg.replay_horizon_mode = "supported_boundary";
  cfg.replay_update_cap = 1024;
  cfg.steps = steps;
  cfg.validate();
  return cfg;
}

coph_driver_config driver_config() {
  coph_driver_config dc;
  dc.cfg = profile(100000);
  dc.comm = coph_communication_profile();
  dc.seed = 31;
  return dc;
}

struct frame {
  std::vector<float> pos, heading, speed;
  std::vector<std::uint8_t> reached;
  bool operator==(const frame &o) const {
    return pos == o.pos && heading == o.heading && speed == o.speed && reached == o.reached;
  }
};

frame snap(const sim_world &s) {
  frame f;
  const int n = s.size();
  f.pos.resize(2 * static_cast<std::size_t>(n));
  f.heading.resize(n);
  f.speed.resize(n);
  f.reached.resize(n);
  s.snapshot(f.pos.data(), f.heading.data(), f.speed.data(), nullptr, f.reached.data());
  return f;
}

} // namespace

// Without a vehicle attached, a coph_driver leaves the run byte-unchanged: the sim's own drive
// moves both agents exactly as it does with no driver.
TEST(CophDriver, ARunWithNothingAttachedIsByteUnchanged) {
  const field f(true);
  sim_world plain = crossing(f), with = crossing(f);
  coph_driver drive(with, driver_config());
  for (int t = 0; t < 400; ++t) {
    plain.step();
    drive.apply();
    with.step();
    ASSERT_TRUE(snap(plain) == snap(with)) << "tick " << t;
  }
  EXPECT_EQ(drive.fleet(), nullptr);
  EXPECT_EQ(drive.steps(), 0);
}

// Froude scaling (D4, D6): CoPH's 0.12 s step is 0.12 sqrt(s) s of the sim's time.
TEST(CophDriver, StepsAtCophsOwnCadence) {
  const field f(false);
  sim_world sim = crossing(f);
  coph_driver_config dc = driver_config();
  EXPECT_EQ(coph_driver(sim, dc).ticks_per_step(), 2); // 0.12 / 0.06
  dc.froude = 25.0;                                    // a Humvee's: 0.6 s
  EXPECT_EQ(coph_driver(sim, dc).ticks_per_step(), 10);
}

TEST(CophDriver, RefusesConfigurationsOutsideTheProfile) {
  const field f(false);
  sim_world sim = crossing(f);
  coph_driver_config dc = driver_config();
  dc.froude = 0.0;
  EXPECT_THROW(coph_driver(sim, dc), std::invalid_argument);
  dc = driver_config();
  dc.local_goal = -1.0;
  EXPECT_THROW(coph_driver(sim, dc), std::invalid_argument);
  dc = driver_config();
  dc.comm.prediction_variant = "compact";
  EXPECT_THROW(coph_driver(sim, dc), std::invalid_argument);
  coph_driver drive(sim, driver_config());
  EXPECT_THROW(drive.attach(2), std::out_of_range);
  drive.attach(0);
  EXPECT_THROW(drive.attach(0), std::invalid_argument);
  drive.apply();
  EXPECT_THROW(drive.attach(1), std::logic_error);
}

// Two cars pass each other across the open field, each carrying a car's learner, with packets
// between them. Each reaches its goal (the sim's reach_tol), and its ledger books the force it
// took and the force it did not.
TEST(CophDriver, TwoCarsCrossTheField) {
  const field f(false);
  sim_world sim = crossing(f);
  coph_driver drive(sim, driver_config());
  drive.attach(0);
  drive.attach(1);
  int ticks = 0;
  for (; ticks < 3000 && !(drive.done(0) && drive.done(1)); ++ticks) {
    drive.apply();
    sim.step();
    const frame fr = snap(sim);
    if (std::getenv("COPH_DRIVER_TRACE") && ticks % 50 == 0 && drive.fleet())
      for (int i = 0; i < 2; ++i) {
        const auto &cs = drive.fleet()->controls(i);
        const coph_fleet::control *c = cs.empty() ? nullptr : &cs.back();
        std::printf(
            "t %4d car %d at (%.2f, %.2f) h %.2f v %.2f u (%.2f, %.2f) %s %s wp (%.2f, %.2f)\n",
            ticks, i, fr.pos[2 * i], fr.pos[2 * i + 1], fr.heading[i], fr.speed[i],
            c ? c->u[0] : 0.0, c ? c->u[1] : 0.0, c ? c->meta.reason.c_str() : "-",
            c ? c->meta.route_reason.c_str() : "-", c ? c->meta.waypoint[0] : 0.0,
            c ? c->meta.waypoint[1] : 0.0);
      }
  }
  const coph_fleet &fleet = *drive.fleet();
  int delivered = 0;
  for (const coph_fleet::event &e : fleet.events())
    delivered += e.kind == "delivered" && e.reason == "eligible";
  for (int v = 0; v < 2; ++v) {
    const coph_driver::ledger &l = drive.account(v);
    std::printf("car %d: %s after %lld CoPH steps; work %.4g, heat %.4g, kinetic %.4g, residual "
                "%.4g (worst step %.3g), unallocated work %.4g, allocation residual %.3f, risk "
                "cost %.4g, version %lld\n",
                v, drive.done(v) ? "arrived" : "still driving",
                static_cast<long long>(l.energy.steps), l.energy.work, l.energy.heat,
                l.energy.kinetic_change, l.energy.residual, l.max_step_residual,
                l.energy.unallocated_work, l.allocation_residual(), l.risk_cost,
                static_cast<long long>(fleet.agents()[v].state().version));
    EXPECT_TRUE(drive.done(v)) << "car " << v;
    EXPECT_TRUE(fleet.success(v)) << "car " << v;
    EXPECT_GT(l.energy.steps, 10);
    EXPECT_GT(l.energy.work, 0.0);
    EXPECT_GT(l.allocation_residual(), 0.0);
    EXPECT_LT(l.allocation_residual(), 1.0);
  }
  std::printf("%d ticks (%.1f s), %lld CoPH steps; %d eligible deliveries\n", ticks, ticks * 0.06,
              drive.steps(), delivered);
  EXPECT_GT(delivered, 0);
}

// A car heading straight at the block senses it only within CoPH's sensor radius (1.25 m at s = 1).
// The car cannot follow CoPH's point mass around it at once: its steer lock gives it a turning
// radius of about 0.74 m, and at low speed it takes almost none of the lateral force (§3.3.2).
// Whatever it then manages, the guard keeps it off the block. This test reports where it ends.
TEST(CophDriver, ABlockSensedLateIsNeverHit) {
  const field f(true);
  sim_world sim = make(f, {{17.0, 6.1, 3.14159265, 3.0, 6.1}});
  coph_driver drive(sim, driver_config());
  drive.attach(0);
  double closest = 1e9;
  int ticks = 0;
  for (; ticks < 1500 && !drive.done(0); ++ticks) {
    drive.apply();
    sim.step();
    const frame fr = snap(sim);
    const double dx = std::max(std::fabs(fr.pos[0] - 10.0) - 0.5, 0.0);
    const double dy = std::max(std::fabs(fr.pos[1] - 6.1) - 0.5, 0.0);
    closest = std::min(closest, std::sqrt(dx * dx + dy * dy));
  }
  const frame fr = snap(sim);
  const coph_control_meta &last = drive.fleet()->controls(0).back().meta;
  std::printf("the car %s after %d ticks at (%.2f, %.2f), %.2f m from the block at its closest; "
              "its last control: %s (%s); allocation residual %.3f\n",
              drive.done(0) ? "arrived" : "stopped", ticks, fr.pos[0], fr.pos[1], closest,
              last.reason.c_str(), last.route_reason.c_str(),
              drive.account(0).allocation_residual());
  EXPECT_GT(closest, 0.1); // its body (0.25 m radius) stays off the block
}
