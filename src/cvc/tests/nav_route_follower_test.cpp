/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::nav::route_follower (route_follower.h) and sim_world::override_command: a sim_world agent
// sized as a Humvee (3.30 m wheelbase, 0.457 rad lock: a 6.7 m turning radius) driven along a
// plan_vehicle_route, reverse legs and cusps included, on the kinematic arm. The ground is 1 m
// cells in nav's trained metric (1 normalized unit = 20 m).

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/route_follower.h>
#include <cvc/nav/sim_world.h>
#include <cvc/nav/vehicle_route.h>
#include <gtest/gtest.h>
#include <vector>

using namespace cvc::nav;

namespace {

constexpr double kS = 0.05;
constexpr double kL = 3.30, kLock = 0.457, kRadius = 1.4; // metres, rad, metres

// A W x H metre world of 1 m cells, walled round its edge, with an 8 m street closed at its east
// end (x 40..70, y 26..34).
struct dead_end {
  static constexpr int W = 100, H = 60;
  int cols = W + 1, rows = H + 1;
  std::vector<std::uint8_t> occ =
      std::vector<std::uint8_t>(static_cast<std::size_t>(W + 1) * (H + 1), 0);
  void box(int x0, int y0, int x1, int y1) {
    for (int y = y0; y <= y1; ++y)
      for (int x = x0; x <= x1; ++x)
        occ[static_cast<std::size_t>(y) * cols + x] = 1;
  }
  dead_end() {
    box(0, 0, W, 0);
    box(0, H, W, H);
    box(0, 0, 0, H);
    box(W, 0, W, H);
    box(40, 0, 70, 25);
    box(40, 35, 70, H);
    box(71, 0, W, H);
  }
};

sim_world::config humvee_config(const dead_end &g) {
  sim_world::config c;
  c.rows = g.rows;
  c.cols = g.cols;
  c.min_x = c.min_y = 0.0;
  c.max_x = dead_end::W;
  c.max_y = dead_end::H;
  c.cx = 0.5 * dead_end::W;
  c.cy = 0.5 * dead_end::H;
  c.scale = kS;
  c.veh.rr = static_cast<float>(kRadius * kS);
  c.veh.d_hat = 0.35f;
  c.veh.dt = 0.06f;
  c.veh.nsub = 6;
  c.veh.vmax = static_cast<float>(8.0 * kS);
  c.veh.L = static_cast<float>(kL * kS);
  c.veh.delta_max = static_cast<float>(kLock);
  c.veh.a_max = static_cast<float>(2.0 * kS);
  c.veh.a_lat_max = static_cast<float>(0.5 * 9.80665 * kS);
  c.veh.allow_reverse = true;
  c.freeze_sense = true;
  c.reach_tol = static_cast<float>(3.0 * kS);
  return c;
}

sim_world make(const dead_end &g, double sx, double sy, double gx, double gy) {
  const sim_world::config c = humvee_config(g);
  const float o[2] = {static_cast<float>((sx - c.cx) * kS), static_cast<float>((sy - c.cy) * kS)};
  const float goal[2] = {static_cast<float>((gx - c.cx) * kS),
                         static_cast<float>((gy - c.cy) * kS)};
  const float col[3] = {1, 1, 1};
  return sim_world(c, g.occ.data(), g.occ.data(), coef_mlp::default_biased(), o, goal, col, 1);
}

route_vehicle vehicle() {
  route_vehicle v;
  v.min_radius = kL / std::tan(kLock);
  v.disc_radius = kRadius;
  return v;
}

struct where {
  float x, y, heading, speed;
};

where at(const sim_world &s) {
  float p[2], h, v;
  s.snapshot(p, &h, &v, nullptr, nullptr);
  return {p[0], p[1], h, v};
}

} // namespace

// override_command drives the bicycle for one step with the given command, backing up from rest
// on a negative acceleration; the next step is the drive's again.
TEST(RouteFollower, OverrideCommandsOneStep) {
  const dead_end g;
  sim_world sim = make(g, 20, 30, 30, 30); // open ground, facing east
  const where w0 = at(sim);
  for (int t = 0; t < 10; ++t) {
    sim.override_command(0, 0.0f, static_cast<float>(-1.0 * kS));
    sim.step();
  }
  const where w1 = at(sim);
  std::printf("  ten backing steps: x %.3f -> %.3f m, speed %.3f m/s\n", w0.x, w1.x, w1.speed);
  EXPECT_LT(w1.speed, 0.0f) << "it backs up";
  EXPECT_LT(w1.x, w0.x) << "west, behind it";
  EXPECT_NEAR(w1.heading, w0.heading, 1e-6) << "straight";
}

// The Humvee facing an 8 m dead end, its goal outside behind it. The route backs it out (its
// circle does not fit the street); the follower drives that, and it reaches the goal.
TEST(RouteFollower, BacksAHumveeOutOfADeadEnd) {
  const dead_end g;
  sim_world sim = make(g, 58, 30, 15, 30);
  ASSERT_TRUE(sim.set_pose(0, static_cast<float>((58 - 50) * kS), 0.0f, 0.0f)); // facing the end
  const where w0 = at(sim);
  const vehicle_route r =
      plan_vehicle_route(g.occ.data(), g.rows, g.cols, 0.0, 0.0, dead_end::W, dead_end::H,
                         vehicle(), {w0.x, w0.y, w0.heading, false}, 15, 30);
  ASSERT_TRUE(r.reached);
  ASSERT_GT(r.reverse_length, 10.0);
  route_follower follow(sim);
  follow.set(0, r);
  bool reversed = false;
  int t = 0;
  double worst = 1e9;
  for (; t < 1500 && !(follow.done(0) && at(sim).speed == 0.0f); ++t) {
    follow.apply();
    reversed = reversed || follow.reversing(0);
    sim.step();
    const where w = at(sim);
    worst = std::min(worst, route_clearance(g.occ.data(), g.rows, g.cols, 0.0, 0.0, dead_end::W,
                                            dead_end::H, vehicle(), {w.x, w.y, w.heading, false}));
    if (follow.done(0) && std::hypot(w.x - 15.0, w.y - 30.0) < 4.0)
      break;
  }
  const where w = at(sim);
  std::printf("  followed: %d legs, at (%.1f, %.1f) after %d ticks (%.1f s), least clearance "
              "%.2f m\n",
              follow.legs(0), w.x, w.y, t, t * 0.06, worst);
  EXPECT_TRUE(reversed) << "it backed up";
  EXPECT_LT(std::hypot(w.x - 15.0, w.y - 30.0), 4.0) << "it reached the goal";
  EXPECT_GT(worst, -0.5) << "it kept off the walls";
}

// The same Humvee left to the trained drive alone, retargeted behind it: reported, for the
// contrast (nav's drive has no reverse manoeuvre).
TEST(RouteFollower, TheDriveAloneInTheDeadEnd) {
  const dead_end g;
  sim_world sim = make(g, 58, 30, 15, 30);
  ASSERT_TRUE(sim.set_pose(0, static_cast<float>((58 - 50) * kS), 0.0f, 0.0f)); // facing the end
  int t = 0;
  for (; t < 1500; ++t) {
    sim.step();
    const where w = at(sim);
    if (std::hypot(w.x - 15.0, w.y - 30.0) < 4.0)
      break;
  }
  const where w = at(sim);
  std::printf("  the drive alone: at (%.1f, %.1f) after %d ticks, %.1f m from the goal\n", w.x, w.y,
              t, std::hypot(w.x - 15.0, w.y - 30.0));
}

// A multi-point turn: out of the dead end to a goal round the corner, on the cheapest route
// (heuristic x1). It backs out, stops at the cusp, and drives forward to the goal; the last
// forward leg is the trained drive's, which parks at the end.
TEST(RouteFollower, StopsAtACuspAndDrivesOn) {
  const dead_end g;
  sim_world sim = make(g, 58, 30, 12, 52);
  ASSERT_TRUE(sim.set_pose(0, static_cast<float>((58 - 50) * kS), 0.0f, 0.0f));
  const where w0 = at(sim);
  route_options o;
  o.heuristic_weight = 1.0;
  const vehicle_route r =
      plan_vehicle_route(g.occ.data(), g.rows, g.cols, 0.0, 0.0, dead_end::W, dead_end::H,
                         vehicle(), {w0.x, w0.y, w0.heading, false}, 12, 52, o);
  ASSERT_TRUE(r.reached);
  std::printf("  the route: %.1f m, %.1f backing up, %d cusps\n", r.length, r.reverse_length,
              r.cusps);
  ASSERT_GE(r.cusps, 1);
  route_follower follow(sim);
  follow.set(0, r);
  int max_leg = 0, t = 0;
  for (; t < 2000; ++t) {
    follow.apply();
    max_leg = std::max(max_leg, follow.leg(0));
    sim.step();
    const where w = at(sim);
    if (follow.done(0) && std::hypot(w.x - 12.0, w.y - 52.0) < 4.0)
      break;
  }
  const where w = at(sim);
  std::printf("  followed: legs %d of %d, at (%.1f, %.1f) after %d ticks (%.1f s)\n", max_leg + 1,
              follow.legs(0), w.x, w.y, t, t * 0.06);
  EXPECT_EQ(max_leg + 1, follow.legs(0)) << "it took every leg";
  EXPECT_LT(std::hypot(w.x - 12.0, w.y - 52.0), 4.0) << "it reached the goal";
}

namespace {

// dead_end's walls round the edge, open inside: the prior, and the truth with a 2 m wall across
// x 30..31, y 20..40.
struct open_field {
  static constexpr int W = 100, H = 60;
  int cols = W + 1, rows = H + 1;
  std::vector<std::uint8_t> prior =
      std::vector<std::uint8_t>(static_cast<std::size_t>(W + 1) * (H + 1), 0);
  std::vector<std::uint8_t> truth;
  void box(std::vector<std::uint8_t> &occ, int x0, int y0, int x1, int y1) const {
    for (int y = y0; y <= y1; ++y)
      for (int x = x0; x <= x1; ++x)
        occ[static_cast<std::size_t>(y) * cols + x] = 1;
  }
  open_field() {
    box(prior, 0, 0, W, 0);
    box(prior, 0, H, W, H);
    box(prior, 0, 0, 0, H);
    box(prior, W, 0, W, H);
    truth = prior;
    box(truth, 30, 20, 31, 40);
  }
};

} // namespace

// Line of sight is each vehicle's own map. Two agents on one line toward the wall, each on its
// own belief plane. Agent 1 starts 14 m from the wall and senses it; agent 0, 27 m away with a
// 16 m sensor, does not, so only plane 1 holds the wall. Following straight routes through it with
// a 40 m lookahead, agent 1 takes the farthest pose it can see, short of the wall, while agent 0
// sees past it on its own map. (Reading plane 0 for both put agent 1's carrot behind the wall.)
TEST(RouteFollower, LineOfSightIsEachVehiclesOwnPlane) {
  const open_field f;
  dead_end g; // for its config's frame (same size)
  sim_world::config c = humvee_config(g);
  c.freeze_sense = false;
  c.range_m = 16.0;
  c.sense_every = 1;
  const float o[4] = {static_cast<float>((3 - c.cx) * kS), static_cast<float>((30 - c.cy) * kS),
                      static_cast<float>((16 - c.cx) * kS), static_cast<float>((30 - c.cy) * kS)};
  const float goal[4] = {static_cast<float>((80 - c.cx) * kS), static_cast<float>((30 - c.cy) * kS),
                         static_cast<float>((80 - c.cx) * kS),
                         static_cast<float>((30 - c.cy) * kS)};
  const float col[6] = {1, 1, 1, 1, 1, 1};
  const int map_id[2] = {0, 1};
  sim_world sim(c, f.truth.data(), f.prior.data(), coef_mlp::default_biased(), o, goal, col, 2,
                map_id, 2);
  ASSERT_TRUE(sim.set_pose(0, o[0], o[1], 0.0f));
  ASSERT_TRUE(sim.set_pose(1, o[2], o[3], 0.0f));
  for (int t = 0; t < 3; ++t)
    sim.step();
  const std::size_t wall = static_cast<std::size_t>(30) * f.cols + 30;
  ASSERT_EQ(sim.belief_occ(1)[wall], 1) << "agent 1 sensed the wall into its plane";
  ASSERT_EQ(sim.belief_occ(0)[wall], 0) << "agent 0's plane does not hold it";

  float p[4], h[2], v[2];
  sim.snapshot(p, h, v, nullptr, nullptr);
  follower_options fo;
  fo.lookahead = 40.0;
  route_follower follow(sim, fo);
  for (int i = 0; i < 2; ++i) {
    const vehicle_route r =
        plan_vehicle_route(f.prior.data(), f.rows, f.cols, 0.0, 0.0, open_field::W, open_field::H,
                           vehicle(), {p[2 * i], p[2 * i + 1], h[i], false}, 80, 30);
    ASSERT_TRUE(r.reached) << "agent " << i;
    follow.set(i, r);
  }
  follow.apply();
  float gw[4];
  sim.goals_world(gw);
  std::printf("  carrots: agent 0 (plane 0) at x %.1f, agent 1 (plane 1, the wall at x 30) at "
              "x %.1f\n",
              gw[0], gw[2]);
  EXPECT_GT(gw[0], 32.0f) << "agent 0 sees past where plane 1's wall is";
  EXPECT_LT(gw[2], 30.0f) << "agent 1's carrot is short of the wall it sensed";
  EXPECT_GT(gw[2], p[2] + 6.0f) << "and the farthest pose it can see";
}

// The drive leg's default carrot is 3 reach_tol along the route (here 9 m); an explicit lookahead
// is taken as given (12 m). (At 2 reach_tol, the old default's floor, the carrot sits right at the
// distance under which the follower gives the stretch to pure pursuit instead, and on a bend under
// it: the reason for the new default.)
TEST(RouteFollower, TheDefaultCarrotIsThreeReachAhead) {
  const dead_end g;
  for (const double look : {0.0, 12.0}) {
    sim_world sim = make(g, 5, 50, 35, 50); // open ground, facing east
    const where w0 = at(sim);
    const vehicle_route r =
        plan_vehicle_route(g.occ.data(), g.rows, g.cols, 0.0, 0.0, dead_end::W, dead_end::H,
                           vehicle(), {w0.x, w0.y, w0.heading, false}, 35, 50);
    ASSERT_TRUE(r.reached);
    ASSERT_EQ(r.cusps, 0);
    follower_options fo;
    fo.lookahead = look;
    route_follower follow(sim, fo);
    follow.set(0, r);
    follow.apply();
    float gw[2];
    sim.goals_world(gw);
    const double d = std::hypot(gw[0] - w0.x, gw[1] - w0.y);
    const double want = look > 0.0 ? look : 9.0;
    std::printf("  lookahead %.1f: the carrot %.2f m ahead (want %.0f)\n", look, d, want);
    EXPECT_NEAR(d, want, 1.0);
  }
}
