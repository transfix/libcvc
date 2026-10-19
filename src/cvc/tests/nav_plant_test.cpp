/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The plant seam (P4, docs/roadmap/NAV-VEHICLE-PHYSICS-ROADMAP.md; doc 31's gates):
//  1. every nav golden and parity test is unchanged (the nav suites; the kinematic arm is
//     untouched without a plant);
//  2. on flat dry ground below 0.3 g the Jolt plant tracks the kinematic trajectory within
//     0.5 m over 200 ticks;
//  3. at the limit the plants diverge, and the divergence is reported, not tuned away;
//  4. the cost per Jolt vehicle is measured and published;
//  5. the adapter test leaves a near-zero residual against the kinematic bicycle at nsub = k;
//  6. the per-tick drift is published, binned by lateral g;
//  7. in the Jolt arm the speed never exceeds the latency-aware stopping speed.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/geometry/height_field.h>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/coef_train.h>
#include <cvc/nav/grid_nav.h>
#include <cvc/nav/route_follower.h>
#include <cvc/nav/sim_world.h>
#include <cvc/nav/vehicle_route.h>
#include <cvc/physics/nav_plant.h>
#include <cvc/physics/world.h>
#include <cvc/state/state.h>
#include <functional>
#include <gtest/gtest.h>
#include <memory>
#include <random>
#include <string>
#include <vector>

using namespace cvc::physics;
using cvc::nav::follower_options;
using cvc::nav::plan_vehicle_route;
using cvc::nav::route_vehicle;
using cvc::nav::sim_world;
using cvc::nav::vehicle_route;

namespace {

bool have_jolt() { return backend_available(backend::jolt); }

#define REQUIRE_JOLT()                                                                             \
  if (!have_jolt())                                                                                \
  GTEST_SKIP() << "this libcvc was built without Jolt"

constexpr double kS = 0.05; // the trained metric: 1 normalized unit = 20 m
constexpr double kG = 9.80665;

world_config jolt_config() {
  world_config c;
  c.backend = backend::jolt;
  return c;
}

// The same ground in both worlds: nav's occupancy grid (1 m cells over +-100 m) and the physics
// world's flat ground with a static box for each wall.
struct scene {
  static constexpr int R = 201, C = 201;
  static constexpr double lo = -100.0, hi = 100.0;
  std::vector<std::uint8_t> occ = std::vector<std::uint8_t>(static_cast<std::size_t>(R) * C, 0);
  struct rect {
    double x0, y0, x1, y1;
  };
  std::vector<rect> walls;

  void wall(double x0, double y0, double x1, double y1) {
    walls.push_back({x0, y0, x1, y1});
    for (int r = 0; r < R; ++r)
      for (int c = 0; c < C; ++c) {
        const double x = lo + c * (hi - lo) / (C - 1), y = lo + r * (hi - lo) / (R - 1);
        if (x >= x0 && x <= x1 && y >= y0 && y <= y1)
          occ[static_cast<std::size_t>(r) * C + c] = 1;
      }
  }

  void build(world &w, double mu) const {
    const material_id m = w.add_material({"ground", static_cast<float>(mu), 0.0f, 0.015f});
    body_desc g;
    g.shape = box{{150.0, 150.0, 0.5}};
    g.motion = motion_type::static_;
    g.position = {0.0, 0.0, -0.5};
    g.material = m;
    w.add_body(g);
    for (const rect &r : walls) {
      body_desc b;
      b.shape = box{{0.5 * (r.x1 - r.x0) + 0.5, 0.5 * (r.y1 - r.y0) + 0.5, 1.5}};
      b.motion = motion_type::static_;
      b.position = {0.5 * (r.x0 + r.x1), 0.5 * (r.y0 + r.y1), 1.5};
      b.material = m;
      w.add_body(b);
    }
  }
};

// nav's footprint of the vehicle's body: four discs along the heading from the rear axle, from
// tail to nose, each 1.3 half-widths, which cover the body's corners.
std::vector<float> &footprint_of(const vehicle_desc &d) {
  static std::vector<float> offsets;
  double rear = 0.0;
  for (const axle_desc &a : d.axles)
    if (!a.steered)
      rear = std::min(rear, a.x);
  const double hy = d.half_extents.y;
  const double tail = -d.half_extents.x - rear + 0.7 * hy,
               nose = d.half_extents.x - rear - 0.7 * hy;
  offsets.clear();
  for (int k = 0; k < 4; ++k)
    offsets.push_back(static_cast<float>((tail + (nose - tail) * k / 3.0) * kS));
  return offsets;
}

// nav's vehicle in the trained metric, sized from the physics vehicle: its body, wheelbase and
// steering lock, an acceleration it can make, and a corner cap of `a_lat_g`.
sim_world::config nav_config(const vehicle_desc &d, double vmax_mps, double a_lat_g) {
  sim_world::config c;
  const std::vector<float> &body = footprint_of(d);
  c.veh.body_offsets = body.data();
  c.veh.n_body = static_cast<int>(body.size());
  c.veh.body_rr = static_cast<float>(1.3 * d.half_extents.y * kS);
  c.veh.body_gain = 1.0f / static_cast<float>(body.size());
  c.rows = scene::R;
  c.cols = scene::C;
  c.min_x = c.min_y = scene::lo;
  c.max_x = c.max_y = scene::hi;
  c.scale = kS;
  c.veh.rr = 0.15f;
  c.veh.d_hat = 0.35f;
  c.veh.dt = 0.06f;
  c.veh.nsub = 6; // the kinematic reference steps at the physics rate (D14)
  c.veh.vmax = static_cast<float>(vmax_mps * kS);
  c.veh.L = static_cast<float>(d.wheelbase() * kS);
  c.veh.delta_max = static_cast<float>(d.max_steer);
  c.veh.a_max = static_cast<float>(0.8 * d.max_accel * kS);
  c.veh.a_lat_max = static_cast<float>(a_lat_g * kG * kS);
  c.freeze_sense = true;
  c.reach_tol = static_cast<float>(4.0 * kS);
  return c;
}

// The gates below size nav by hand from the vehicle (nav_config), and the plant plans with that
// too, as the kinematic arm does: the plant's own plan (nav_plant_options::plan_with_vehicle) is
// PlansWithTheVehiclesOwnEnvelope's.
nav_plant_options nav_sized() {
  nav_plant_options o;
  o.plan_with_vehicle = false;
  return o;
}

sim_world make_sim(const sim_world::config &c, const scene &s, double sx, double sy, double gx,
                   double gy) {
  const float o[2] = {static_cast<float>(sx * kS), static_cast<float>(sy * kS)};
  const float g[2] = {static_cast<float>(gx * kS), static_cast<float>(gy * kS)};
  const float col[3] = {1.0f, 1.0f, 1.0f};
  return sim_world(c, s.occ.data(), s.occ.data(), cvc::nav::coef_mlp::default_biased(), o, g, col,
                   1);
}

struct pose {
  float x = 0, y = 0, heading = 0, speed = 0;
};

pose pose_of(const sim_world &w, int i = 0) {
  const int n = w.size();
  std::vector<float> p(2 * n), h(n), s(n);
  std::vector<int> m(n);
  std::vector<std::uint8_t> r(n);
  w.snapshot(p.data(), h.data(), s.data(), m.data(), r.data());
  return {p[2 * i], p[2 * i + 1], h[i], s[i]};
}

void print_bins(const nav_plant &p) {
  for (const nav_plant::drift_bin &b : p.drift_summary())
    if (b.ticks > 0)
      std::printf("    %.1f-%s g: %5llu ticks; position rms %.4f max %.4f m, heading rms %.5f max "
                  "%.5f rad, speed rms %.4f max %.4f m/s\n",
                  b.lo_g, std::isfinite(b.hi_g) ? std::to_string(b.hi_g).substr(0, 3).c_str() : "",
                  static_cast<unsigned long long>(b.ticks), b.position_rms_m, b.position_max_m,
                  b.heading_rms_rad, b.heading_max_rad, b.speed_rms_mps, b.speed_max_mps);
}

} // namespace

// The adapter reads a vehicle back as nav sees it: the rear axle where the agent started, the
// heading, at rest, in nav's frame; and its limits are the vehicle's.
TEST(NavPlant, ReadsTheVehicleBackInNavsFrame) {
  REQUIRE_JOLT();
  scene sc;
  world w(jolt_config());
  sc.build(w, 0.9);
  const vehicle_desc d = vehicle_desc::humvee();
  nav_plant plant(w, {d});
  cvc::world_units::normalized_frame f;
  f.scale = kS;
  f.cx = 10.0;
  f.cy = -5.0;
  cvc::nav::plant_state start;
  start.x = static_cast<float>((20.0 - f.cx) * kS);
  start.y = static_cast<float>((30.0 - f.cy) * kS);
  start.heading = 0.7f;
  plant.attach(1, &start, f, 0.06);
  for (int t = 0; t < 50; ++t)
    plant.advance(); // settle onto the suspension
  const cvc::nav::plant_state s = plant.state(0);
  std::printf("  rear axle (%.4f, %.4f) m, heading %.5f, speed %.5f m/s\n", s.x / kS + f.cx,
              s.y / kS + f.cy, s.heading, s.speed / kS);
  EXPECT_NEAR(s.x / kS + f.cx, 20.0, 0.01);
  EXPECT_NEAR(s.y / kS + f.cy, 30.0, 0.01);
  EXPECT_NEAR(s.heading, 0.7, 1e-3);
  EXPECT_NEAR(s.speed / kS, 0.0, 0.01);
  const cvc::nav::plant_limits l = plant.limits(0);
  EXPECT_NEAR(l.wheelbase, 3.30 * kS, 1e-6);
  EXPECT_FLOAT_EQ(l.max_steer, static_cast<float>(d.max_steer));
  EXPECT_FLOAT_EQ(l.steer_rate, static_cast<float>(d.max_steer_rate));
  EXPECT_NEAR(l.a_brake, 0.8 * d.max_decel * kS, 1e-6);
}

// The plant's own plan (nav_plant_options::plan_with_vehicle, on by default). Its limits are the
// vehicle's: 0.8 of its max_accel, its rated top speed, 0.5 g of cornering (under 0.8 of its
// static stability factor), and its body from the rear axle. A sim_world sized for no vehicle in
// particular (the trained metric's defaults: 18 m/s, 30 m/s^2, a 2 g corner cap, a 3 m disc)
// then drives a Humvee within the Humvee's envelope through a hard turn; planned with nav's own
// numbers, the same drive commands several times the Humvee's acceleration and corners harder.
TEST(NavPlant, PlansWithTheVehiclesOwnEnvelope) {
  REQUIRE_JOLT();
  const vehicle_desc d = vehicle_desc::humvee();
  double ahead = d.half_extents.x, behind = -d.half_extents.x, half_width = d.half_extents.y,
         rear = 0.0;
  for (const axle_desc &a : d.axles) {
    ahead = std::max(ahead, a.x + a.wheel_radius);
    behind = std::min(behind, a.x - a.wheel_radius);
    half_width = std::max(half_width, a.half_track + 0.5 * a.wheel_width);
    if (!a.steered)
      rear = std::min(rear, a.x);
  }
  {
    scene sc;
    world w(jolt_config());
    sc.build(w, 0.9);
    nav_plant plant(w, {d});
    cvc::world_units::normalized_frame f;
    f.scale = kS;
    cvc::nav::plant_state start;
    plant.attach(1, &start, f, 0.06);
    const cvc::nav::plant_limits l = plant.limits(0);
    EXPECT_NEAR(l.a_max, 0.8 * d.max_accel * kS, 1e-6);
    EXPECT_NEAR(l.vmax, 24.6 * kS, 1e-6);
    EXPECT_NEAR(l.a_lat, 0.5 * kG * kS, 1e-6);
    EXPECT_NEAR(l.half_width, half_width * kS, 1e-6);
    EXPECT_NEAR(l.front, (ahead - rear) * kS, 1e-6);
    EXPECT_NEAR(l.rear, (rear - behind) * kS, 1e-6);
  }
  const auto drive = [&](bool plan, double &peak_accel, double &peak_g, double &peak_speed,
                         sim_world::plant_plan &pp) {
    scene sc;
    sim_world::config c = nav_config(d, 18.0, 2.0);
    c.veh.n_body = 0;
    c.veh.body_offsets = nullptr;
    c.veh.body_rr = 0.0f;
    c.veh.body_gain = 1.0f;
    c.veh.rr = 0.15f;
    c.veh.a_max = 1.5f;
    c.veh.L = 0.035f;
    sim_world sim = make_sim(c, sc, -60, -20, 60, 20);
    world w(jolt_config());
    sc.build(w, 0.9);
    nav_plant_options o;
    o.plan_with_vehicle = plan;
    nav_plant plant(w, {d}, o);
    sim.set_plant(&plant);
    peak_accel = peak_g = peak_speed = 0.0;
    for (int t = 0; t < 260; ++t) {
      if (t == 120) // a hard turn at speed: the goal jumps to the left
        sim.retarget(0, static_cast<float>(10 * kS), static_cast<float>(80 * kS));
      sim.step();
      if (t < 20)
        continue; // settling
      peak_accel = std::max(peak_accel, static_cast<double>(sim.plant_accel_command()[0]) / kS);
      peak_g = std::max(peak_g, plant.last_drift(0).lateral_g);
      peak_speed = std::max(peak_speed, std::fabs(static_cast<double>(pose_of(sim).speed)));
    }
    EXPECT_TRUE(sim.plant_plan_of(0, pp));
  };
  double acc_on, g_on, v_on, acc_off, g_off, v_off;
  sim_world::plant_plan on, off;
  drive(true, acc_on, g_on, v_on, on);
  drive(false, acc_off, g_off, v_off, off);
  std::printf("  the vehicle's plan: commands up to %.2f m/s^2, %.3f g, %.2f m/s; nav's: %.2f "
              "m/s^2, %.3f g, %.2f m/s\n",
              acc_on, g_on, v_on, acc_off, g_off, v_off);
  std::printf("  footprint: %d discs of %.2f m at %.2f, %.2f, %.2f, %.2f m from the rear axle\n",
              sim_world::kPlantDiscs, on.body_rr / kS, on.body_offsets[0] / kS,
              on.body_offsets[1] / kS, on.body_offsets[2] / kS, on.body_offsets[3] / kS);
  // The plan the drive used is the vehicle's, but for a top speed capped at nav's 18 m/s.
  EXPECT_NEAR(on.a_max, 0.8 * d.max_accel * kS, 1e-6);
  EXPECT_NEAR(on.vmax, 18.0 * kS, 1e-6);
  EXPECT_NEAR(on.a_lat, 0.5 * kG * kS, 1e-6);
  EXPECT_TRUE(on.footprint);
  EXPECT_FALSE(off.footprint);
  EXPECT_NEAR(off.a_max, 1.5, 1e-6) << "nav's own, without the vehicle's plan";
  // The body is covered: the discs reach the tail and the nose, at least half a width wide.
  EXPECT_LE(on.body_offsets[0] - on.body_rr, (behind - rear) * kS + 1e-6);
  EXPECT_GE(on.body_offsets[sim_world::kPlantDiscs - 1] + on.body_rr, (ahead - rear) * kS - 1e-6);
  EXPECT_GE(on.body_rr, half_width * kS - 1e-6);
  // And it drove within the vehicle's envelope, where nav's numbers asked for more.
  EXPECT_LE(acc_on, 0.8 * d.max_accel + 1e-3);
  EXPECT_LE(v_on, 18.0 + 0.5);
  EXPECT_LE(g_on, 0.55);
  EXPECT_GT(acc_off, 2.0 * acc_on);
  EXPECT_GT(g_off, g_on + 0.2);
}

// Gate 5: Jolt set up to behave almost kinematically (an instant steering actuator, grippy flat
// ground) leaves a near-zero per-tick residual against the kinematic bicycle at nsub = k = 6.
TEST(NavPlant, TheAdapterResidualIsNearZero) {
  REQUIRE_JOLT();
  scene sc;
  vehicle_desc d = vehicle_desc::humvee();
  d.max_steer_rate = 100.0;
  sim_world sim = make_sim(nav_config(d, 6.0, 0.15), sc, -60, -20, 60, 20);
  world w(jolt_config());
  sc.build(w, 1.2);
  nav_plant plant(w, {d}, nav_sized());
  sim.set_plant(&plant);
  float worst_pos = 0, worst_head = 0, worst_speed = 0;
  for (int t = 0; t < 200; ++t) {
    if (t == 80)
      sim.retarget(0, static_cast<float>(20 * kS), static_cast<float>(70 * kS));
    sim.step();
    if (t < 20)
      continue; // the vehicle settles onto its suspension
    const nav_plant::drift_si &dr = plant.last_drift(0);
    worst_pos = std::max(worst_pos, static_cast<float>(dr.position_m));
    worst_head = std::max(worst_head, static_cast<float>(std::fabs(dr.heading_rad)));
    worst_speed = std::max(worst_speed, static_cast<float>(std::fabs(dr.speed_mps)));
  }
  const pose p = pose_of(sim);
  std::printf("  per tick, worst: position %.4f m, heading %.5f rad, speed %.4f m/s; ended at "
              "(%.1f, %.1f)\n",
              worst_pos, worst_head, worst_speed, p.x, p.y);
  print_bins(plant);
  EXPECT_LT(worst_pos, 0.05);
  EXPECT_LT(worst_head, 0.01);
  EXPECT_LT(worst_speed, 0.3);
}

// Gate 2: on flat dry ground below 0.3 g, the Jolt Humvee tracks the kinematic bicycle's
// trajectory within 0.5 m over 200 ticks (12 s): its path never strays more than 0.5 m from the
// bicycle's along a curving route (a goal that swings round as the vehicle drives). At a tick
// the two also differ along the path, by the engine's response to an instant acceleration,
// which is reported. Two harder routes are reported, not gated: a swerve round a block and a step
// change of goal, where the bicycle turns at once and the Humvee's steering at 0.35 rad/s
// (doc 31 D14's actuator lag); with NAV_PLANT_FAST_STEER set the Humvee steers at once too, which
// separates that lag from the rest.
enum class route { curve, block, step };

class NavPlantTrack : public ::testing::TestWithParam<route> {};

TEST_P(NavPlantTrack, TracksTheKinematicTrajectoryBelow0p3g) {
  const route r = GetParam();
  REQUIRE_JOLT();
  scene sc;
  if (r == route::block)
    sc.wall(-12.0, 0.0, -8.0, 4.0); // a block just beside the line from start to goal
  vehicle_desc d = vehicle_desc::humvee();
  if (std::getenv("NAV_PLANT_FAST_STEER"))
    d.max_steer_rate = 100.0;
  const sim_world::config cfg = nav_config(d, 8.0, 0.25);
  sim_world kin = make_sim(cfg, sc, -60, -20, 60, 20);
  sim_world jolt = make_sim(cfg, sc, -60, -20, 60, 20);
  world w(jolt_config());
  sc.build(w, 0.9);
  nav_plant plant(w, {d}, nav_sized());
  jolt.set_plant(&plant);
  double worst = 0.0, worst_along = 0.0, worst_across = 0.0, peak_g = 0.0, path = 0.0;
  pose last = pose_of(jolt);
  std::vector<pose> kin_path{pose_of(kin)}, jolt_path{last};
  const double g0 = std::atan2(40.0, 120.0);
  for (int t = 0; t < 200; ++t) {
    if (r == route::step && t == 80) {
      kin.retarget(0, static_cast<float>(20 * kS), static_cast<float>(70 * kS));
      jolt.retarget(0, static_cast<float>(20 * kS), static_cast<float>(70 * kS));
    }
    if (r == route::curve) {
      // The goal swings left round the start at 0.15 degrees a tick, 126 m out.
      const double a = g0 + 0.0026 * t;
      const float gx = static_cast<float>((-60.0 + 126.0 * std::cos(a)) * kS);
      const float gy = static_cast<float>((-20.0 + 126.0 * std::sin(a)) * kS);
      kin.retarget(0, gx, gy);
      jolt.retarget(0, gx, gy);
    }
    kin.step();
    jolt.step();
    const pose a = pose_of(kin), b = pose_of(jolt);
    worst = std::max(worst, static_cast<double>(std::hypot(a.x - b.x, a.y - b.y)));
    const double along = (b.x - a.x) * std::cos(a.heading) + (b.y - a.y) * std::sin(a.heading);
    const double across = -(b.x - a.x) * std::sin(a.heading) + (b.y - a.y) * std::cos(a.heading);
    worst_along = std::max(worst_along, std::fabs(along));
    worst_across = std::max(worst_across, std::fabs(across));
    kin_path.push_back(a);
    jolt_path.push_back(b);
    peak_g = std::max(peak_g, plant.last_drift(0).lateral_g);
    path += std::hypot(b.x - last.x, b.y - last.y);
    last = b;
  }
  // How far the Jolt path strays from the bicycle's: each pose's distance to its polyline.
  double stray = 0.0;
  for (const pose &b : jolt_path) {
    double best = 1e30;
    for (std::size_t k = 1; k < kin_path.size(); ++k) {
      const double ax = kin_path[k - 1].x, ay = kin_path[k - 1].y;
      const double ex = kin_path[k].x - ax, ey = kin_path[k].y - ay;
      const double l2 = ex * ex + ey * ey;
      const double u =
          l2 > 0.0 ? std::clamp(((b.x - ax) * ex + (b.y - ay) * ey) / l2, 0.0, 1.0) : 0.0;
      best = std::min(best, std::hypot(b.x - (ax + u * ex), b.y - (ay + u * ey)));
    }
    stray = std::max(stray, best);
  }
  const pose a = pose_of(kin);
  const char *name = r == route::curve   ? "a curving route"
                     : r == route::block ? "round a block"
                                         : "a step change of goal";
  std::printf("  %s: 200 ticks, %.1f m driven, peak %.3f g: the Jolt path strays at most %.3f m "
              "from the bicycle's; tick by tick they differ at most %.3f m (%.3f along, %.3f "
              "across); end (%.2f, %.2f) against (%.2f, %.2f)\n",
              name, path, peak_g, stray, worst, worst_along, worst_across, last.x, last.y, a.x,
              a.y);
  print_bins(plant);
  EXPECT_GT(path, 50.0);
  EXPECT_LT(peak_g, 0.3);
  if (r == route::curve)
    EXPECT_LE(stray, 0.5);
}

INSTANTIATE_TEST_SUITE_P(NavPlant, NavPlantTrack,
                         ::testing::Values(route::curve, route::block, route::step),
                         [](const ::testing::TestParamInfo<route> &i) {
                           return i.param == route::curve   ? std::string("ACurvingRoute")
                                  : i.param == route::block ? std::string("RoundABlock")
                                                            : std::string("AStepChangeOfGoal");
                         });

// Gate 3: at the limit (a corner cap above the ground's grip) the plants diverge; the drift
// grows with the lateral acceleration and the summary reports it, bin by bin.
TEST(NavPlant, DivergesAtTheLimitAndReportsIt) {
  REQUIRE_JOLT();
  scene sc;
  const vehicle_desc d = vehicle_desc::humvee();
  sim_world sim = make_sim(nav_config(d, 14.0, 1.0), sc, -80, -60, 80, -60);
  world w(jolt_config());
  sc.build(w, 0.6);
  nav_plant plant(w, {d}, nav_sized());
  sim.set_plant(&plant);
  for (int t = 0; t < 300; ++t) {
    if (t == 150)
      sim.retarget(0, static_cast<float>(-60 * kS), static_cast<float>(60 * kS));
    sim.step();
  }
  print_bins(plant);
  const std::vector<nav_plant::drift_bin> bins = plant.drift_summary();
  const nav_plant::drift_bin *low = nullptr, *high = nullptr;
  for (const nav_plant::drift_bin &b : bins)
    if (b.ticks > 0) {
      if (!low)
        low = &b;
      high = &b;
    }
  ASSERT_TRUE(low && high);
  EXPECT_GE(high->lo_g, 0.3) << "the run reached the limit";
  EXPECT_GT(high->position_rms_m + high->heading_rms_rad,
            2.0 * (low->position_rms_m + low->heading_rms_rad))
      << "the drift grows toward the limit";
}

// Gate 4: the cost of a Jolt vehicle at real time, measured and published.
TEST(NavPlant, MeasuresTheCostPerVehicle) {
  REQUIRE_JOLT();
  scene sc;
  const vehicle_desc d = vehicle_desc::humvee();
  const int n = 8;
  std::vector<float> o(2 * n), g(2 * n), col(3 * n, 1.0f);
  for (int i = 0; i < n; ++i) {
    o[2 * i] = static_cast<float>(-70.0 * kS);
    o[2 * i + 1] = static_cast<float>((-70.0 + 20.0 * i) * kS);
    g[2 * i] = static_cast<float>(70.0 * kS);
    g[2 * i + 1] = o[2 * i + 1];
  }
  sim_world sim(nav_config(d, 8.0, 0.25), sc.occ.data(), sc.occ.data(),
                cvc::nav::coef_mlp::default_biased(), o.data(), g.data(), col.data(), n);
  world w(jolt_config());
  sc.build(w, 0.9);
  nav_plant plant(w, {d}, nav_sized());
  sim.set_plant(&plant);
  for (int t = 0; t < 200; ++t)
    sim.step();
  std::printf("  %d Humvees: %.0f us a tick (%.0f us a vehicle) for 6 physics steps; real time "
              "allows 60000 us\n",
              n, plant.advance_us(), plant.advance_us_per_vehicle());
  EXPECT_GT(plant.advance_us_per_vehicle(), 0.0);
  EXPECT_LT(plant.advance_us(), 60000.0) << "faster than real time";
  cvc::app app;
  std::vector<std::string> names;
  for (int i = 0; i < n; ++i)
    names.push_back("v" + std::to_string(i));
  plant.publish(app, "physics", names);
  EXPECT_GT(
      std::stod(cvc::state::instance(app)("physics.nav_plant.advance_us_per_vehicle").value()),
      0.0);
}

// Gate 6: the per-tick drift is published, per vehicle and binned by lateral g.
TEST(NavPlant, PublishesTheDriftBinnedByLateralG) {
  REQUIRE_JOLT();
  scene sc;
  const vehicle_desc d = vehicle_desc::humvee();
  sim_world sim = make_sim(nav_config(d, 8.0, 0.25), sc, -60, -20, 60, 20);
  world w(jolt_config());
  sc.build(w, 0.9);
  nav_plant plant(w, {d}, nav_sized());
  sim.set_plant(&plant);
  for (int t = 0; t < 120; ++t)
    sim.step();
  cvc::app app;
  plant.publish(app, "physics", {"lead"});
  cvc::state &st = cvc::state::instance(app);
  EXPECT_FALSE(st("physics.vehicles.lead.nav.drift.position_m").value().empty());
  EXPECT_FALSE(st("physics.vehicles.lead.nav.steer_command").value().empty());
  std::uint64_t ticks = 0;
  for (std::size_t b = 0; b < plant.drift_summary().size(); ++b)
    ticks += std::stoull(st("physics.nav_plant.drift." + std::to_string(b) + ".ticks").value());
  EXPECT_EQ(ticks, 120u);
  EXPECT_EQ(st("physics.nav_plant.drift.0.lo_g").value().substr(0, 1), "0");
  EXPECT_FALSE(st("physics.vehicles.lead.nav.drift.stop_speed_mps").value().empty());
  // The fixed settings are written, the live ones seeded by pull() and applied from the tree;
  // an invalid one is ignored.
  EXPECT_EQ(st("physics.nav_plant.settings.physics_steps").value(), "6");
  EXPECT_EQ(plant.reference_substeps(), 6) << "D14's nsub = k";
  plant.pull(app, "physics");
  EXPECT_EQ(std::stod(st("physics.nav_plant.latency").value()), plant.options().latency);
  EXPECT_EQ(std::stod(st("physics.nav_plant.brake_margin").value()), 0.8);
  st("physics.nav_plant.latency").value(std::string("0.25"));
  st("physics.nav_plant.brake_margin").value(std::string("2"));
  st("physics.nav_plant.drift_nsub").value(std::string("12"));
  st("physics.nav_plant.modulation.tire_gain").value(std::string("3"));
  st("physics.nav_plant.modulation.tip_onset").value(std::string("-1"));
  plant.pull(app, "physics");
  EXPECT_EQ(plant.modulation().tire_gain, 3.0f);
  EXPECT_EQ(plant.modulation().tip_onset, 0.4f) << "an onset is >= 0";
  EXPECT_EQ(plant.options().latency, 0.25);
  EXPECT_EQ(plant.limits(0).latency, 0.25f);
  EXPECT_EQ(plant.options().brake_margin, 0.8) << "2 is not a share";
  EXPECT_EQ(plant.reference_substeps(), 12);
  sim.step(); // the drive takes the new latency from the plant's limits, and its modulation
  EXPECT_EQ(sim.current_plant_modulation().tire_gain, 3.0f);
  plant.publish(app, "physics", {"lead"});
  // Every key says what it is.
  for (const char *key :
       {"physics.nav_plant.latency", "physics.nav_plant.drift_nsub",
        "physics.nav_plant.settings.physics_steps", "physics.nav_plant.advance_us_per_vehicle",
        "physics.nav_plant.drift.2.position_rms_m",
        "physics.vehicles.lead.nav.drift.stop_speed_mps", "physics.vehicles.lead.nav.accel_command",
        "physics.nav_plant.modulation.tire_gain", "physics.vehicles.lead.nav.tire_use",
        "physics.vehicles.lead.nav.roll_rad"}) {
    EXPECT_FALSE(st(key).comment().empty()) << key;
    EXPECT_TRUE(st(key).has_property("access")) << key;
  }
  EXPECT_EQ(st("physics.nav_plant.latency").property("unit"), "s");
  EXPECT_EQ(st("physics.nav_plant.latency").property("max"), "0.5");
  EXPECT_EQ(st("physics.nav_plant.brake_margin").property("min"), "0.3");
  EXPECT_EQ(st("physics.nav_plant.settings.step_world").property("access"), "build");
  EXPECT_EQ(st("physics.nav_plant.modulation.tire_gain").property("max"), "10");
  EXPECT_FALSE(st("physics.vehicles.lead.nav.tire_use").value().empty());
}

// Gate 7: driving at a wall, the Jolt arm never goes faster than the stopping speed that allows
// for the vehicle's latency and achievable braking, and it never touches the wall.
TEST(NavPlant, NeverExceedsTheLatencyAwareStoppingSpeed) {
  REQUIRE_JOLT();
  scene sc;
  sc.wall(20.0, -30.0, 22.0, 30.0);
  const vehicle_desc d = vehicle_desc::humvee();
  sim_world sim = make_sim(nav_config(d, 12.0, 0.25), sc, -60, 0, 60, 0);
  world w(jolt_config());
  sc.build(w, 0.9);
  nav_plant plant(w, {d}, nav_sized());
  sim.set_plant(&plant);
  sim.begin_nav_stats();
  double worst = -1e9, closest = 1e9, fastest = 0.0;
  for (int t = 0; t < 400; ++t) {
    sim.step();
    const double v = pose_of(sim).speed;
    const double v_stop = sim.plant_stop_speed()[0] / kS;
    worst = std::max(worst, v - v_stop);
    fastest = std::max(fastest, v);
    closest = std::min(closest, 20.0 - pose_of(sim).x);
  }
  const cvc::nav::episode_nav_stats e = sim.nav_stats();
  std::printf("  fastest %.2f m/s; speed - v_stop at most %+.3f m/s; the rear axle came within "
              "%.2f m of the wall; %d building contacts\n",
              fastest, worst, closest, e.per_vehicle[0].building_contacts);
  EXPECT_GT(fastest, 3.0);
  EXPECT_LE(worst, 0.1);
  EXPECT_EQ(e.per_vehicle[0].building_contacts, 0);
}

// nav_stats carries the plant's energy: what the vehicle drew, per km, and its building contacts;
// the kinematic arm's record leaves them out.
TEST(NavPlant, NavStatsCarriesThePlantsEnergy) {
  REQUIRE_JOLT();
  scene sc;
  const vehicle_desc d = vehicle_desc::humvee();
  sim_world kin = make_sim(nav_config(d, 8.0, 0.25), sc, -60, -20, 60, 20);
  sim_world sim = make_sim(nav_config(d, 8.0, 0.25), sc, -60, -20, 60, 20);
  world w(jolt_config());
  sc.build(w, 0.9);
  nav_plant plant(w, {d}, nav_sized());
  sim.set_plant(&plant);
  kin.begin_nav_stats();
  sim.begin_nav_stats();
  for (int t = 0; t < 200; ++t) {
    kin.step();
    sim.step();
  }
  const cvc::nav::episode_nav_stats e = sim.nav_stats();
  const cvc::nav::veh_nav_stats &v = e.per_vehicle[0];
  const vehicle_energy le = w.vehicle_state(plant.vehicle(0)).energy;
  std::printf("  %.1f m: %.0f kJ drawn (%.0f kJ/km; the ledger's source %.0f kJ, %.3f kg of "
              "diesel)\n",
              v.total_path_m, v.energy_J / 1e3, v.energy_per_km / 1e3, le.source / 1e3,
              le.fuel_used);
  EXPECT_TRUE(v.energy_tracked);
  EXPECT_GT(v.energy_J, 0.0);
  EXPECT_NEAR(v.energy_J, le.source, 0.02 * le.source) << "the ticks since the episode began";
  EXPECT_NEAR(v.energy_per_km, v.energy_J / (v.total_path_m / 1000.0), 1e-6 * v.energy_per_km);
  EXPECT_NE(e.to_json().find("\"energy_per_km\""), std::string::npos);
  EXPECT_EQ(kin.nav_stats().to_json().find("energy"), std::string::npos);
}

// --- P5: terrain in the navigator's energy, on the Jolt plant ---------------------------------

namespace {

std::vector<float> terrain_heights(const std::function<double(double, double)> &h) {
  std::vector<float> z(static_cast<std::size_t>(scene::R) * scene::C);
  for (int r = 0; r < scene::R; ++r)
    for (int c = 0; c < scene::C; ++c)
      z[static_cast<std::size_t>(r) * scene::C + c] =
          static_cast<float>(h(scene::lo + c, scene::lo + r));
  return z;
}

// The same heights as the physics world's ground, a height field over +-100 m.
void hilly_ground(world &w, const std::vector<float> &z, double mu) {
  const material_id m = w.add_material({"ground", static_cast<float>(mu), 0.0f, 0.015f});
  auto field =
      std::make_shared<cvc::height_field>(scene::R, scene::C, scene::lo, scene::lo, 1.0, 1.0, z);
  body_desc g;
  g.shape = terrain{field};
  g.motion = motion_type::static_;
  g.material = m;
  w.add_body(g);
}

std::vector<double> astar_route(const std::vector<std::uint8_t> &occ, double sx, double sy,
                                double gx, double gy, const cvc::nav::climb_cost &climb = {}) {
  const std::vector<int> p =
      cvc::nav::astar(occ.data(), scene::R, scene::C, static_cast<int>(sy - scene::lo),
                      static_cast<int>(sx - scene::lo), static_cast<int>(gy - scene::lo),
                      static_cast<int>(gx - scene::lo), nullptr, climb);
  std::vector<double> xy;
  for (std::size_t k = 0; k + 1 < p.size(); k += 2) {
    xy.push_back(scene::lo + p[k + 1]);
    xy.push_back(scene::lo + p[k]);
  }
  return xy;
}

// Retarget the agent to the first route point `ahead` metres away, past the last reached.
struct route_follower {
  std::vector<double> xy;
  std::size_t next = 0;
  void steer(sim_world &w, double ahead = 15.0) {
    const pose p = pose_of(w);
    const std::size_t n = xy.size() / 2;
    while (next + 1 < n && std::hypot(xy[2 * next] - p.x, xy[2 * next + 1] - p.y) < ahead)
      ++next;
    w.retarget(0, static_cast<float>(xy[2 * next] * kS), static_cast<float>(xy[2 * next + 1] * kS));
  }
  bool done(const sim_world &w) const {
    const pose p = pose_of(w);
    return std::hypot(xy[xy.size() - 2] - p.x, xy.back() - p.y) < 5.0;
  }
};

} // namespace

// P5 gate 2: on a hilly route the navigator's physics lowers the energy per km, for a path at
// most half again as long. A 12 m hill stands between start and goal. The plain run takes the
// shortest route over it. The physics run takes the route that weighs the climbing energy (H5),
// with the grade force (H1) and gravity in the governors (H2). The Humvee drives both, and its
// energy ledger (nav_stats energy_J, energy_per_km) is the measure.
TEST(NavPlant, EnergyPerKmDropsOnAHillyRoute) {
  REQUIRE_JOLT();
  const auto hill = [](double x, double y) {
    return 12.0 * std::exp(-(x * x + y * y) / (2.0 * 18.0 * 18.0));
  };
  const std::vector<float> z = terrain_heights(hill);
  const vehicle_desc d = vehicle_desc::humvee();
  scene sc;
  double energy[2], per_km[2], path[2];
  for (int k = 0; k < 2; ++k) {
    const bool physics = k == 1;
    sim_world sim = make_sim(nav_config(d, 8.0, 0.25), sc, -70, 0, 70, 0);
    world w(jolt_config());
    hilly_ground(w, z, 0.9);
    nav_plant plant(w, {d}, nav_sized());
    sim.set_plant(&plant);
    cvc::nav::terrain_config tc = nav_plant::terrain_config_of(d);
    tc.lam_grade = physics ? 2.0f : 0.0f;
    tc.governors = physics;
    sim.set_terrain(z.data(), tc);
    cvc::nav::climb_cost cc;
    if (physics) {
      cc.height = z.data();
      cc.per_metre = cvc::nav::climb_weight(0.015, 1.0);
    }
    route_follower f{astar_route(sc.occ, -70, 0, 70, 0, cc)};
    ASSERT_FALSE(f.xy.empty());
    sim.begin_nav_stats();
    int t = 0;
    for (; t < 1500 && !f.done(sim); ++t) {
      f.steer(sim);
      sim.step();
    }
    const cvc::nav::veh_nav_stats v = sim.nav_stats().per_vehicle[0];
    energy[k] = v.energy_J;
    per_km[k] = v.energy_per_km;
    path[k] = v.total_path_m;
    std::printf("  %s: %.1f m in %.1f s, %.0f kJ, %.0f kJ/km; %d ticks on infeasible slope\n",
                physics ? "with the navigator's physics" : "plain", path[k], 0.06 * t,
                energy[k] / 1e3, per_km[k] / 1e3, sim.infeasible_entries()[0]);
    EXPECT_TRUE(f.done(sim)) << "it arrived";
  }
  EXPECT_LT(per_km[1], per_km[0]) << "less energy per km";
  EXPECT_LT(energy[1], energy[0]) << "less energy in all";
  EXPECT_LT(path[1], 1.5 * path[0]) << "for a bounded detour";
}

// P5 gate 3: zero entries into infeasible slope, on the Jolt plant. A ridge with slopes of 1.2
// (50 degrees, past the Humvee's climb and its tip limit) lies across the way; its cells plan as
// obstacles and the route goes round its end.
TEST(NavPlant, NeverEntersInfeasibleSlope) {
  REQUIRE_JOLT();
  const auto ridge = [](double x, double y) {
    return y > 40.0 ? 0.0 : std::max(0.0, 6.0 - 1.2 * std::fabs(x));
  };
  const std::vector<float> z = terrain_heights(ridge);
  const vehicle_desc d = vehicle_desc::humvee();
  scene sc;
  sim_world sim = make_sim(nav_config(d, 8.0, 0.25), sc, -40, -20, 40, -20);
  world w(jolt_config());
  hilly_ground(w, z, 0.9);
  nav_plant plant(w, {d}, nav_sized());
  sim.set_plant(&plant);
  const cvc::nav::terrain_config tc = nav_plant::terrain_config_of(d);
  sim.set_terrain(z.data(), tc);
  std::vector<std::uint8_t> blocked(sc.occ);
  for (std::size_t k = 0; k < blocked.size(); ++k)
    blocked[k] = static_cast<std::uint8_t>(blocked[k] | sim.terrain().infeasible[k]);
  route_follower f{astar_route(blocked, -40, -20, 40, -20)};
  ASSERT_FALSE(f.xy.empty());
  int t = 0;
  for (; t < 2500 && !f.done(sim); ++t) {
    f.steer(sim);
    sim.step();
  }
  std::printf("  the Humvee's limits: climbs %.2f, tips past %.2f (static stability factor); "
              "%d ticks on infeasible slope; arrived after %.1f s\n",
              tc.max_grade, tc.tip_ratio, sim.infeasible_entries()[0], 0.06 * t);
  EXPECT_NEAR(tc.tip_ratio, 0.91 / 0.74, 0.05) << "the M1097A2's CG height is 0.742 m";
  EXPECT_EQ(sim.infeasible_entries()[0], 0);
  EXPECT_TRUE(f.done(sim));
}

// H4: measured tire saturation as a detached modulator. Round a 30 m circle on mu 0.7 the drive's
// corner cap (0.9 g) asks more of the tires than they hold, so they saturate. With the modulator
// the drive slows as they near their limit, and they work less hard.
TEST(NavPlant, MeasuredSlipTightensTheDrive) {
  REQUIRE_JOLT();
  const vehicle_desc d = vehicle_desc::humvee();
  const double cx = 0.0, cy = -40.0, radius = 30.0;
  double worst[2], mean[2], fastest[2];
  for (int k = 0; k < 2; ++k) {
    scene sc;
    sim_world sim = make_sim(nav_config(d, 16.0, 0.9), sc, cx, cy - radius, cx + radius, cy);
    world w(jolt_config());
    sc.build(w, 0.7);
    nav_plant plant(w, {d}, nav_sized());
    cvc::nav::plant_modulation m;
    m.tire_gain = k == 0 ? 0.0f : 3.0f;
    plant.set_modulation(m);
    sim.set_plant(&plant);
    worst[k] = mean[k] = fastest[k] = 0.0;
    int counted = 0;
    for (int t = 0; t < 500; ++t) {
      // The goal leads the vehicle round the circle by 40 degrees.
      const pose p = pose_of(sim);
      const double a = std::atan2(p.y - cy, p.x - cx) + 0.7;
      sim.retarget(0, static_cast<float>((cx + radius * std::cos(a)) * kS),
                   static_cast<float>((cy + radius * std::sin(a)) * kS));
      sim.step();
      const cvc::nav::plant_state ps = plant.state(0);
      if (t >= 200) { // round the circle at speed
        worst[k] = std::max(worst[k], static_cast<double>(ps.tire_use));
        mean[k] += ps.tire_use;
        ++counted;
      }
      fastest[k] = std::max(fastest[k], static_cast<double>(ps.speed / kS));
    }
    mean[k] /= std::max(1, counted);
  }
  std::printf(
      "  tire use (combined slip over its peak) round the circle: mean %.2f, peak %.2f "
      "without the modulator (top speed %.1f m/s); mean %.2f, peak %.2f with it (%.1f m/s)\n",
      mean[0], worst[0], fastest[0], mean[1], worst[1], fastest[1]);
  EXPECT_GT(worst[0], 0.7) << "the run worked the tires to their limit";
  EXPECT_LT(mean[1], mean[0]);
  EXPECT_LT(worst[1], worst[0]);
}

// P9 R2: the Humvee on the Jolt plant facing the end of an 8 m dead end (walls as static boxes),
// its goal outside behind it. The trained drive alone has no way out: a U-turn needs a 13.4 m
// circle. A vehicle route (plan_vehicle_route, with the plant's own footprint and turning radius)
// followed by route_follower backs it out to the goal.
TEST(NavPlant, AVehicleRouteBacksTheHumveeOutOfADeadEnd) {
  REQUIRE_JOLT();
  scene sc;
  sc.wall(20, -100, 60, -5);   // south of the street (y -4..4)
  sc.wall(20, 5, 60, 100);     // north of it
  sc.wall(61, -100, 100, 100); // its end
  const vehicle_desc d = vehicle_desc::humvee();
  const auto run = [&](bool route, int &ticks, bool &reversed) {
    sim_world sim = make_sim(nav_config(d, 8.0, 0.5), sc, 50, 0, 0, 0);
    sim.set_pose(0, static_cast<float>(50 * kS), 0.0f, 0.0f); // facing the end, before the plant
    world w(jolt_config());
    sc.build(w, 0.9);
    nav_plant plant(w, {d});
    sim.set_plant(&plant);
    reversed = false;
    follower_options fo;
    vehicle_route r;
    if (route) {
      const cvc::nav::plant_limits l = plant.limits(0);
      sim_world::plant_plan pp;
      EXPECT_TRUE(sim.plant_plan_of(0, pp));
      route_vehicle v;
      v.min_radius = l.wheelbase / kS / std::tan(l.max_steer);
      v.disc_radius = pp.body_rr / kS;
      for (const float off : pp.body_offsets)
        v.disc_offsets.push_back(off / kS);
      const pose p0 = pose_of(sim);
      r = plan_vehicle_route(sc.occ.data(), scene::R, scene::C, scene::lo, scene::lo, scene::hi,
                             scene::hi, v, {p0.x, p0.y, p0.heading, false}, 0.0, 0.0);
      EXPECT_TRUE(r.reached);
      std::printf("  the route: %.1f m, %.1f backing up, %d cusps (radius %.2f m, %zu discs of "
                  "%.2f m)\n",
                  r.length, r.reverse_length, r.cusps, v.min_radius, v.disc_offsets.size(),
                  v.disc_radius);
      fo.wheelbase = l.wheelbase / kS;
      fo.max_steer = l.max_steer;
    }
    cvc::nav::route_follower follow(sim, fo);
    if (route)
      follow.set(0, r);
    for (ticks = 0; ticks < 1500; ++ticks) {
      if (route) {
        follow.apply();
        reversed = reversed || follow.reversing(0);
      }
      sim.step();
      const pose p = pose_of(sim);
      if (std::hypot(p.x, p.y) < 5.0)
        break;
    }
    const pose p = pose_of(sim);
    return std::hypot(p.x, p.y);
  };
  int t_alone = 0, t_route = 0;
  bool rev_alone = false, rev_route = false;
  const double alone = run(false, t_alone, rev_alone);
  const double routed = run(true, t_route, rev_route);
  std::printf("  the drive alone: %.1f m from the goal after %d ticks; on the route: %.1f m after "
              "%d ticks (%.1f s)\n",
              alone, t_alone, routed, t_route, t_route * 0.06);
  EXPECT_TRUE(rev_route) << "it backed up";
  EXPECT_LT(routed, 5.0) << "the route took it out to its goal";
}

// ── P8b's gate: a policy trained on the dynamic vehicle against one trained on the point mass,
// both driving Jolt RoboRacers ──

namespace {

// The trainer's city read as a RoboRacer's: veh_L 0.035 normalized is its 0.324 m wheelbase, so
// S is about 0.108 per metre and a world unit (0.05 normalized) is about 0.46 m. Buildings are
// 1.5 m static boxes, one per row run of occupied cells.
double city_reach_on_jolt(const cvc::nav::coef_mlp &policy, const cvc::nav::training_scene &sc,
                          int n, int ticks, unsigned seed) {
  const vehicle_desc d = vehicle_desc::roboracer();
  const double s_per_m = 0.035 / d.wheelbase();
  const double mpw = sc.scale / s_per_m; // metres per world unit
  sim_world::config cfg;
  cfg.rows = sc.rows;
  cfg.cols = sc.cols;
  cfg.min_x = sc.min_x;
  cfg.min_y = sc.min_y;
  cfg.max_x = sc.max_x;
  cfg.max_y = sc.max_y;
  cfg.scale = sc.scale;
  cfg.veh.rr = sc.rr;
  cfg.veh.d_hat = sc.d_hat;
  cfg.veh.dt = sc.dt;
  cfg.veh.vmax = sc.vmax;
  cfg.veh.nsub = 6;
  cfg.reach_tol = 0.15f;
  cfg.freeze_sense = true;
  sim_world w = sim_world::from_occupancy(cfg, sc.occ.data(), policy, n, seed);

  world phys(jolt_config());
  const material_id m = phys.add_material({"ground", 0.9f, 0.0f, 0.015f});
  body_desc g;
  g.shape = box{{(sc.max_x - sc.min_x) * mpw, (sc.max_y - sc.min_y) * mpw, 0.5}};
  g.motion = motion_type::static_;
  g.position = {0.0, 0.0, -0.5};
  g.material = m;
  phys.add_body(g);
  const double dx = (sc.max_x - sc.min_x) / (sc.cols - 1),
               dy = (sc.max_y - sc.min_y) / (sc.rows - 1);
  for (int r = 0; r < sc.rows; ++r)
    for (int c = 0; c < sc.cols;) {
      if (!sc.occ[static_cast<std::size_t>(r) * sc.cols + c]) {
        ++c;
        continue;
      }
      int e = c;
      while (e + 1 < sc.cols && sc.occ[static_cast<std::size_t>(r) * sc.cols + e + 1])
        ++e;
      body_desc b;
      b.shape = box{{0.5 * (e - c + 1) * dx * mpw, 0.5 * dy * mpw, 0.75}};
      b.motion = motion_type::static_;
      b.position = {(sc.min_x + 0.5 * (c + e) * dx) * mpw, (sc.min_y + r * dy) * mpw, 0.75};
      b.material = m;
      phys.add_body(b);
      c = e + 1;
    }
  nav_plant_options o = nav_sized(); // nav plans with the scene's numbers, as both were trained
  o.metres_per_world_unit = mpw;
  nav_plant plant(phys, {d}, o);
  w.set_plant(&plant);
  for (int t = 0; t < ticks; ++t)
    w.step(0);
  std::vector<std::uint8_t> reached(n);
  w.snapshot(nullptr, nullptr, nullptr, nullptr, reached.data());
  int k = 0;
  for (int i = 0; i < n; ++i)
    k += reached[i];
  w.set_plant(nullptr);
  return static_cast<double>(k) / n;
}

} // namespace

// The gate (the vehicle-physics plan's P8b): over >= 20 paired seeds on the Jolt plant, a policy
// trained through the dynamic vehicle should beat the one trained on the point-mass surrogate,
// with a 95% bootstrap CI. This reports the comparison; it is a heavy run (the "Convergence"
// suites run where CI runs those).
TEST(NavPlantConvergence, DynamicTrainedPolicyAgainstPointMassOnJolt) {
  REQUIRE_JOLT();
  const cvc::nav::training_scene sc = cvc::nav::city_scene(96);
  cvc::nav::train_config base;
  base.n = 128;
  base.hidden = 64;
  base.horizon = 28;
  base.window = 7;
  base.steps = 150;
  base.seed = 0;
  base.lr = 5e-5f;
  // Experiments: P8B_GATE_LR and P8B_GATE_STEPS override both policies' training.
  if (const char *e = std::getenv("P8B_GATE_LR"))
    base.lr = static_cast<float>(std::atof(e));
  if (const char *e = std::getenv("P8B_GATE_STEPS"))
    base.steps = std::atoi(e);
  cvc::nav::coef_trainer pm(base, 1);
  pm.train(sc);
  cvc::nav::train_config dc = base;
  dc.rollout = cvc::nav::rollout_kind::dynamic;
  dc.dyn = nav_plant::dyn_bike_params_at(vehicle_desc::roboracer(), dc.veh_L);
  cvc::nav::coef_trainer dy(dc, 1);
  dy.train(sc);
  const cvc::nav::coef_mlp p_pm = pm.to_coef_mlp(), p_dy = dy.to_coef_mlp(),
                           p_seed = cvc::nav::coef_mlp::default_biased();

  const int seeds = 20, n = 48, ticks = 400;
  std::vector<double> a(seeds), b(seeds), c(seeds), diff(seeds);
  double ma = 0.0, mb = 0.0, mc = 0.0;
  for (int s = 0; s < seeds; ++s) {
    const unsigned seed = 100u + static_cast<unsigned>(s);
    a[s] = city_reach_on_jolt(p_pm, sc, n, ticks, seed);
    b[s] = city_reach_on_jolt(p_dy, sc, n, ticks, seed);
    c[s] = city_reach_on_jolt(p_seed, sc, n, ticks, seed);
    diff[s] = b[s] - a[s];
    ma += a[s] / seeds;
    mb += b[s] / seeds;
    mc += c[s] / seeds;
  }
  // Paired bootstrap of the mean difference.
  std::mt19937 rng(0);
  std::uniform_int_distribution<int> pick(0, seeds - 1);
  std::vector<double> boot(2000);
  for (double &x : boot) {
    double sum = 0.0;
    for (int i = 0; i < seeds; ++i)
      sum += diff[pick(rng)];
    x = sum / seeds;
  }
  std::sort(boot.begin(), boot.end());
  const double lo = boot[static_cast<std::size_t>(0.025 * boot.size())],
               hi = boot[static_cast<std::size_t>(0.975 * boot.size()) - 1];
  std::printf("  P8b gate (lr %.0e, %d steps) on Jolt RoboRacers (city, %d seeds x %d agents, %d "
              "ticks): reach %.1f%% "
              "point-mass-trained, %.1f%% dynamic-trained, %.1f%% the untrained seed; dynamic - "
              "point mass = %+.1f points, 95%% CI [%+.1f, %+.1f]\n",
              base.lr, base.steps, seeds, n, ticks, 100 * ma, 100 * mb, 100 * mc, 100 * (mb - ma),
              100 * lo, 100 * hi);
  for (int s = 0; s < seeds; ++s)
    std::printf("    seed %d: %.3f %.3f %.3f\n", 100 + s, a[s], b[s], c[s]);
  EXPECT_TRUE(std::isfinite(lo) && std::isfinite(hi));
}
