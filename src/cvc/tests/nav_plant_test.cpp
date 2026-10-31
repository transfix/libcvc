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
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/geometry/height_field.h>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/coef_train.h>
#include <cvc/nav/dyn_power.h>
#include <cvc/nav/grid_nav.h>
#include <cvc/nav/route_follower.h>
#include <cvc/nav/sim_world.h>
#include <cvc/nav/vehicle_route.h>
#include <cvc/physics/jolt_rollout.h>
#include <cvc/physics/nav_es.h>
#include <cvc/physics/nav_eval.h>
#include <cvc/physics/nav_plant.h>
#include <cvc/physics/plant_log.h>
#include <cvc/physics/plant_log_scene.h>
#include <cvc/physics/world.h>
#include <cvc/state/state.h>
#include <filesystem>
#include <fstream>
#include <functional>
#include <gtest/gtest.h>
#include <memory>
#include <random>
#include <sstream>
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

// The ES runs on Jolt are experiments, not checks. Their one assertion, best >= start, holds by
// construction (es_finetune_on_plant keeps the start), NavPlant.EsFinetuneOnPlantMechanics covers
// the mechanics in every build, and each run takes 30 to 60+ minutes on a hosted CI runner:
// EsFinetuneOnJoltReach never finished inside ctest's hour on package-linux, and repeated three
// times on every push. They run when CVC_NAV_ES_EXPERIMENTS=1.
#define REQUIRE_ES_EXPERIMENTS()                                                                   \
  if (!std::getenv("CVC_NAV_ES_EXPERIMENTS") ||                                                    \
      std::string(std::getenv("CVC_NAV_ES_EXPERIMENTS")) != "1")                                   \
  GTEST_SKIP() << "an ES experiment: set CVC_NAV_ES_EXPERIMENTS=1 to run it"

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

// Vehicle against vehicle (nav_plant::vehicle_contact_step): one Humvee driven into the back of
// another standing 10 m ahead. Neither touches another before; both report each other, with a
// penetration, once they do.
TEST(NavPlant, ReportsWhichVehicleItTouched) {
  REQUIRE_JOLT();
  scene sc;
  world w(jolt_config());
  sc.build(w, 0.9);
  const vehicle_desc d = vehicle_desc::humvee();
  nav_plant plant(w, {d});
  cvc::world_units::normalized_frame f;
  f.scale = kS;
  cvc::nav::plant_state start[2];
  start[0].x = static_cast<float>(20.0 * kS);
  start[0].y = static_cast<float>(30.0 * kS);
  start[1] = start[0];
  start[1].x = static_cast<float>(30.0 * kS);
  plant.attach(2, start, f, 0.06);
  for (int t = 0; t < 30; ++t)
    plant.advance();
  EXPECT_EQ(plant.vehicle_contact_step(0), -1);
  EXPECT_EQ(plant.vehicle_contact_step(1), -1);
  int first = -1;
  double depth = 0.0;
  for (int t = 0; t < 200 && first < 0; ++t) {
    plant.command(0, 0.0f, static_cast<float>(2.0 * kS));
    plant.command(1, 0.0f, 0.0f);
    plant.advance();
    if (plant.vehicle_contact_step(0) >= 0) {
      first = t;
      depth = plant.vehicle_contact_depth(0);
      EXPECT_EQ(plant.vehicle_contact_step(0), 1);
      EXPECT_EQ(plant.vehicle_contact_step(1), 0);
    }
  }
  std::printf("  rear-ended at tick %d (%.2f s), penetration %.3f m\n", first, first * 0.06, depth);
  ASSERT_GE(first, 0) << "the vehicles should touch";
  EXPECT_GE(depth, 0.0);
}

// The ledger tick by tick (nav_plant::ledger_step): a Humvee accelerating, cornering and braking.
// Each tick's lines balance as the ledger does, they sum to the vehicle's own ledger, and the
// tick's energy_step is what the source paid of them.
TEST(NavPlant, LedgerStepLinesBalanceAndSumToTheLedger) {
  REQUIRE_JOLT();
  scene sc;
  world w(jolt_config());
  sc.build(w, 0.9);
  nav_plant plant(w, {vehicle_desc::humvee()});
  cvc::world_units::normalized_frame f;
  f.scale = kS;
  cvc::nav::plant_state start;
  plant.attach(1, &start, f, 0.06);
  const vehicle_energy &zero = plant.ledger_step(0);
  EXPECT_EQ(zero.supplied, 0.0);
  EXPECT_EQ(zero.stored, 0.0);
  vehicle_energy sum;
  double worst = 0.0, supplied = 0.0;
  for (int t = 0; t < 300; ++t) {
    const float steer = t < 100 ? 0.0f : t < 200 ? 0.3f : 0.0f;
    const float accel = static_cast<float>((t < 200 ? 2.0 : -4.0) * kS);
    plant.command(0, steer, accel);
    plant.advance();
    const vehicle_energy &l = plant.ledger_step(0);
    const double in = l.supplied + l.idle + l.fields + l.drivetrain_split;
    const double out = l.dissipated() + l.stored + l.residual;
    worst = std::max(worst, std::fabs(in - out) / std::max(1.0, std::fabs(in)));
    // The Humvee burns fuel: what it drew is the source's line, net of regeneration (none).
    EXPECT_NEAR(plant.energy_step(0), l.source - l.regen, 1e-9 * std::max(1.0, l.source))
        << "tick " << t;
    supplied += l.supplied;
    sum.brake += l.brake;
    sum.tire += l.tire;
    sum.stored += l.stored;
    sum.distance += l.distance;
  }
  const vehicle_energy e = w.vehicle_state(plant.vehicle(0)).energy;
  std::printf("  300 ticks: supplied %.0f J, tire %.0f J, brake %.0f J, %.1f m; worst tick "
              "imbalance %.2e\n",
              supplied, sum.tire, sum.brake, sum.distance, worst);
  EXPECT_LT(worst, 1e-9) << "a tick's lines do not balance";
  EXPECT_NEAR(supplied, e.supplied, 1e-6 * e.supplied);
  EXPECT_NEAR(sum.brake, e.brake, 1e-6 * std::max(1.0, e.brake));
  EXPECT_NEAR(sum.tire, e.tire, 1e-6 * std::max(1.0, e.tire));
  EXPECT_NEAR(sum.stored, e.stored, 1e-6 * std::max(1.0, std::fabs(e.stored)));
  EXPECT_NEAR(sum.distance, e.distance, 1e-9 * std::max(1.0, e.distance));
  EXPECT_GT(sum.brake, 0.0) << "the run should brake";
  EXPECT_EQ(plant.ledger_step(0).kinetic, e.kinetic) << "a level is the tick's end's";
}

// A run restored (nav_plant::save / restore) continues bit for bit: three Humvees driven for 20
// ticks, saved, driven 40 more; restored and driven the same 40 again, every state, sample and
// ledger line repeats exactly. A state of other agents is refused and changes nothing.
TEST(NavPlant, SaveRestoreReplaysBitForBit) {
  REQUIRE_JOLT();
  scene sc;
  world w(jolt_config());
  sc.build(w, 0.9);
  nav_plant plant(w, {vehicle_desc::humvee()});
  cvc::world_units::normalized_frame f;
  f.scale = kS;
  cvc::nav::plant_state start[3];
  for (int i = 0; i < 3; ++i) {
    start[i].x = static_cast<float>(20.0 * i * kS);
    start[i].heading = 0.3f * static_cast<float>(i);
  }
  plant.attach(3, start, f, 0.06);
  const auto drive = [&](int t) {
    for (int i = 0; i < 3; ++i)
      plant.command(i, 0.25f * std::sin(0.05f * static_cast<float>(t + 7 * i)),
                    static_cast<float>((1.5 - 0.02 * (t % 60)) * kS));
    plant.advance();
  };
  for (int t = 0; t < 20; ++t)
    drive(t);
  const std::vector<std::uint8_t> saved = plant.save();
  struct tick {
    cvc::nav::plant_state s;
    cvc::nav::plant_sample sample;
    double energy, supplied, tire, stored;
  };
  const auto run = [&] {
    std::vector<tick> out;
    for (int t = 20; t < 60; ++t) {
      drive(t);
      for (int i = 0; i < 3; ++i) {
        const vehicle_energy &l = plant.ledger_step(i);
        out.push_back(
            {plant.state(i), plant.sample(i), plant.energy_step(i), l.supplied, l.tire, l.stored});
      }
    }
    return out;
  };
  const std::vector<tick> first = run();
  const std::uint64_t hash_after = w.state_hash();
  plant.restore(saved);
  const std::vector<tick> again = run();
  ASSERT_EQ(first.size(), again.size());
  for (std::size_t k = 0; k < first.size(); ++k) {
    const tick &a = first[k], &b = again[k];
    ASSERT_EQ(a.s.x, b.s.x) << "tick " << k / 3 << " agent " << k % 3;
    ASSERT_EQ(a.s.y, b.s.y);
    ASSERT_EQ(a.s.heading, b.s.heading);
    ASSERT_EQ(a.s.speed, b.s.speed);
    ASSERT_EQ(a.sample.vy, b.sample.vy);
    ASSERT_EQ(a.sample.a_lat, b.sample.a_lat);
    ASSERT_EQ(a.energy, b.energy);
    ASSERT_EQ(a.supplied, b.supplied);
    ASSERT_EQ(a.tire, b.tire);
    ASSERT_EQ(a.stored, b.stored);
  }
  EXPECT_EQ(w.state_hash(), hash_after);
  EXPECT_GT(first.back().s.speed, 0.0f) << "the vehicles should be moving";

  // Another plant's state, of two agents on another world, is refused; this run is untouched.
  world w2(jolt_config());
  sc.build(w2, 0.9);
  nav_plant other(w2, {vehicle_desc::humvee()});
  other.attach(2, start, f, 0.06);
  const std::uint64_t before = w.state_hash();
  EXPECT_THROW(plant.restore(other.save()), cvc::physics_error);
  EXPECT_THROW(plant.restore(std::vector<std::uint8_t>(saved.begin(), saved.end() - 1)),
               cvc::physics_error);
  EXPECT_EQ(w.state_hash(), before);
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
  EXPECT_EQ(st("physics.nav_plant.drift.0.lo_mps2").value().substr(0, 1), "0");
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
  // Accelerations in the tree are m/s^2: 0.5 g seeded as 4.9, and 2.94 m/s^2 applied as 0.3 g.
  EXPECT_NEAR(std::stod(st("physics.nav_plant.plan.lateral_mps2").value()),
              plant.options().lateral_g * 9.80665, 1e-6);
  st("physics.nav_plant.plan.lateral_mps2").value(std::string("2.941995"));
  plant.pull(app, "physics");
  EXPECT_NEAR(plant.options().lateral_g, 0.3, 1e-9);
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

// The city's RoboRacer wheelbase in normalized units (train_config::veh_L's 0.035) as an exact
// double: the scale S = kCityL / 0.324 per metre. reach_on_plant is chaotic in it; passing the
// float veh_L instead (4e-9 relative) moves single seeds' reach by up to 0.1.
constexpr double kCityL = 0.035;

// The paired-bootstrap 95% CI of the mean of `diff`.
void paired_ci(const std::vector<double> &diff, double &lo, double &hi) {
  std::mt19937 rng(0);
  std::uniform_int_distribution<int> pick(0, static_cast<int>(diff.size()) - 1);
  std::vector<double> boot(2000);
  for (double &x : boot) {
    double sum = 0.0;
    for (std::size_t i = 0; i < diff.size(); ++i)
      sum += diff[static_cast<std::size_t>(pick(rng))];
    x = sum / static_cast<double>(diff.size());
  }
  std::sort(boot.begin(), boot.end());
  lo = boot[static_cast<std::size_t>(0.025 * boot.size())];
  hi = boot[static_cast<std::size_t>(0.975 * boot.size()) - 1];
}

} // namespace

// reach_on_plant is a fitness train() selects on, so it must be repeatable: the same policy,
// scene and seeds give the same per-seed reach (the world is single-threaded and runs in
// cross-platform determinism mode), one seed at a time or all at once. It also refuses what it
// cannot score.
TEST(NavPlant, ReachOnPlantIsRepeatable) {
  REQUIRE_JOLT();
  const cvc::nav::training_scene sc = cvc::nav::city_scene(96);
  const vehicle_desc rr = vehicle_desc::roboracer();
  const cvc::nav::coef_mlp p = cvc::nav::coef_mlp::default_biased();
  plant_reach_options o;
  o.agents = 16;
  o.ticks = 150;
  o.seeds = {100u, 101u, 102u};
  o.threads = 1;
  std::vector<double> a, b, c;
  const double ma = reach_on_plant(p, sc, rr, kCityL, o, &a);
  const double mb = reach_on_plant(p, sc, rr, kCityL, o, &b);
  o.threads = 0;
  const double mc = reach_on_plant(p, sc, rr, kCityL, o, &c);
  ASSERT_EQ(a.size(), 3u);
  EXPECT_EQ(a, b);
  EXPECT_EQ(a, c) << "seeds in parallel should score what they score one at a time";
  EXPECT_EQ(ma, mb);
  EXPECT_EQ(ma, mc);
  EXPECT_DOUBLE_EQ(ma, (a[0] + a[1] + a[2]) / 3.0);
  std::printf("  reach_on_plant, %d agents x %d ticks: seeds 100-102 %.3f %.3f %.3f (twice one "
              "at a time, once in parallel)\n",
              o.agents, o.ticks, a[0], a[1], a[2]);
  o.seeds.clear();
  EXPECT_THROW(reach_on_plant(p, sc, rr, kCityL, o), std::invalid_argument);
}

// Local goals (plant_reach_options::goal_max): reach scored to goals near their starts, which
// the same policy reaches more of in the same ticks than goals across the map. It repeats as the
// default does, one seed at a time or all at once, and an inverted range is refused.
TEST(NavPlant, ReachOnPlantWithLocalGoals) {
  REQUIRE_JOLT();
  const cvc::nav::training_scene sc = cvc::nav::city_scene(96);
  const vehicle_desc rr = vehicle_desc::roboracer();
  const cvc::nav::coef_mlp p = cvc::nav::coef_mlp::default_biased();
  plant_reach_options o;
  o.agents = 16;
  o.ticks = 150;
  o.seeds = {100u, 101u, 102u};
  o.threads = 1;
  const double across = reach_on_plant(p, sc, rr, kCityL, o);
  o.goal_min = 0.5f; // 4.6..14 m at the RoboRacer's scale
  o.goal_max = 1.5f;
  std::vector<double> a, b;
  const double ma = reach_on_plant(p, sc, rr, kCityL, o, &a);
  o.threads = 0;
  const double mb = reach_on_plant(p, sc, rr, kCityL, o, &b);
  EXPECT_EQ(a, b);
  EXPECT_EQ(ma, mb);
  EXPECT_GT(ma, across) << "goals within reach of their starts should be reached more";
  std::printf("  reach_on_plant, %d agents x %d ticks, seeds 100-102: goals across the map %.3f, "
              "%.1f..%.1f from their starts %.3f (%.3f %.3f %.3f)\n",
              o.agents, o.ticks, across, o.goal_min, o.goal_max, ma, a[0], a[1], a[2]);
  o.goal_min = 2.0f;
  EXPECT_THROW(reach_on_plant(p, sc, rr, kCityL, o), std::invalid_argument);
}

// score_on_plant (P8c): the same runs as reach_on_plant, so the same reach, plus the energy the
// agents drew and the distance they drove. Both repeat one seed at a time or all at once, and a
// RoboRacer fleet's draw per km is a plausible number for a 22 kg car.
TEST(NavPlant, ScoreOnPlantCountsEnergyAndDistance) {
  REQUIRE_JOLT();
  const cvc::nav::training_scene sc = cvc::nav::city_scene(96);
  const vehicle_desc rr = vehicle_desc::roboracer();
  const cvc::nav::coef_mlp p = cvc::nav::coef_mlp::default_biased();
  plant_reach_options o;
  o.agents = 16;
  o.ticks = 150;
  o.seeds = {100u, 101u, 102u};
  o.threads = 1;
  std::vector<double> reach;
  reach_on_plant(p, sc, rr, kCityL, o, &reach);
  std::vector<std::vector<plant_drive_score>> a, b;
  const std::vector<plant_drive_score> ma = score_on_plant({p}, sc, rr, kCityL, o, &a);
  o.threads = 0;
  const std::vector<plant_drive_score> mb = score_on_plant({p}, sc, rr, kCityL, o, &b);
  ASSERT_EQ(a.size(), 1u);
  ASSERT_EQ(a[0].size(), 3u);
  double e = 0.0, m = 0.0;
  for (int s = 0; s < 3; ++s) {
    EXPECT_EQ(a[0][s].reach, reach[s]);
    EXPECT_EQ(a[0][s].energy_J, b[0][s].energy_J);
    EXPECT_EQ(a[0][s].distance_m, b[0][s].distance_m);
    EXPECT_GT(a[0][s].energy_J, 0.0);
    EXPECT_GT(a[0][s].distance_m, 0.0);
    e += a[0][s].energy_J;
    m += a[0][s].distance_m;
  }
  EXPECT_DOUBLE_EQ(ma[0].energy_J, e / 3.0);
  EXPECT_DOUBLE_EQ(ma[0].distance_m, m / 3.0);
  EXPECT_EQ(ma[0].energy_J, mb[0].energy_J);
  const double whkm = ma[0].wh_per_km();
  std::printf("  score_on_plant, %d agents x %d ticks, seeds 100-102: reach %.3f, %.0f J over "
              "%.0f m, %.2f Wh/km\n",
              o.agents, o.ticks, ma[0].reach, ma[0].energy_J, ma[0].distance_m, whkm);
  EXPECT_GT(whkm, 1.0);
  EXPECT_LT(whkm, 200.0);
}

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

  const plant_reach_options eval; // 20 seeds (100..119) x 48 agents x 400 ticks
  const int seeds = static_cast<int>(eval.seeds.size()), n = eval.agents, ticks = eval.ticks;
  const vehicle_desc rr = vehicle_desc::roboracer();
  std::vector<double> a, b, c, diff(seeds);
  const double ma = reach_on_plant(p_pm, sc, rr, kCityL, eval, &a);
  const double mb = reach_on_plant(p_dy, sc, rr, kCityL, eval, &b);
  const double mc = reach_on_plant(p_seed, sc, rr, kCityL, eval, &c);
  for (int s = 0; s < seeds; ++s)
    diff[s] = b[s] - a[s];
  double lo, hi;
  paired_ci(diff, lo, hi);
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

// Checkpoint selection on Jolt reach (P8b): the same two trainings, each scoring its checkpoints
// by reach on Jolt RoboRacers over validation seeds (200.., never the evaluation's) and ending
// on the best, the untrained policy included; then the gate's comparison on the 20 held-out
// seeds. P8B_SELECT_LR / _STEPS / _EVERY override the training, P8B_SELECT_VAL the number of
// validation seeds (32: a mean over k seeds carries about 3/sqrt(k) points of the plant's chaos
// alone, and with 8 the best of 31 checkpoints was mostly luck).
TEST(NavPlantConvergence, CheckpointSelectionOnJoltReach) {
  REQUIRE_JOLT();
  const cvc::nav::training_scene sc = cvc::nav::city_scene(96);
  const vehicle_desc rr = vehicle_desc::roboracer();
  cvc::nav::train_config base;
  base.n = 128;
  base.hidden = 64;
  base.horizon = 28;
  base.window = 7;
  base.steps = 300;
  base.seed = 0;
  base.lr = 2e-4f;
  base.checkpoint_every = 30;
  if (const char *e = std::getenv("P8B_SELECT_LR"))
    base.lr = static_cast<float>(std::atof(e));
  if (const char *e = std::getenv("P8B_SELECT_STEPS"))
    base.steps = std::atoi(e);
  if (const char *e = std::getenv("P8B_SELECT_EVERY"))
    base.checkpoint_every = std::atoi(e);
  int nval = 32;
  if (const char *e = std::getenv("P8B_SELECT_VAL"))
    nval = std::atoi(e);
  plant_reach_options val;
  val.seeds.clear();
  for (int k = 0; k < nval; ++k)
    val.seeds.push_back(200u + static_cast<unsigned>(k));
  const auto jolt_reach = [&](const cvc::nav::coef_mlp &policy, int) {
    return reach_on_plant(policy, sc, rr, kCityL, val);
  };
  cvc::nav::coef_trainer pm(base, 1);
  pm.train(sc, false, jolt_reach);
  cvc::nav::train_config dc = base;
  dc.rollout = cvc::nav::rollout_kind::dynamic;
  dc.dyn = nav_plant::dyn_bike_params_at(rr, dc.veh_L);
  cvc::nav::coef_trainer dy(dc, 1);
  dy.train(sc, false, jolt_reach);
  const auto trace = [](const char *name, const cvc::nav::coef_trainer &t) {
    std::printf("  %s checkpoints (validation reach):", name);
    for (const cvc::nav::coef_trainer::checkpoint_score &k : t.checkpoints())
      std::printf(" %d:%.3f", k.step, k.fitness);
    std::printf("; selected step %d\n", t.selected_step());
  };
  trace("point-mass", pm);
  trace("dynamic   ", dy);

  const plant_reach_options eval; // the gate's 20 held-out seeds
  std::vector<double> a, b, c;
  const double ma = reach_on_plant(pm.to_coef_mlp(), sc, rr, kCityL, eval, &a);
  const double mb = reach_on_plant(dy.to_coef_mlp(), sc, rr, kCityL, eval, &b);
  const double mc = reach_on_plant(cvc::nav::coef_mlp::default_biased(), sc, rr, kCityL, eval, &c);
  std::vector<double> d_pm(a.size()), d_seed(a.size());
  for (std::size_t s = 0; s < a.size(); ++s) {
    d_pm[s] = b[s] - a[s];
    d_seed[s] = b[s] - c[s];
  }
  double lo1, hi1, lo2, hi2;
  paired_ci(d_pm, lo1, hi1);
  paired_ci(d_seed, lo2, hi2);
  std::printf("  selected on Jolt reach over %d validation seeds (lr %.0e, %d steps, every %d), "
              "held-out reach: %.1f%% "
              "point-mass-trained, %.1f%% dynamic-trained, %.1f%% the seed; dynamic - point "
              "mass %+.1f [%+.1f, %+.1f], dynamic - seed %+.1f [%+.1f, %+.1f]\n",
              nval, base.lr, base.steps, base.checkpoint_every, 100 * ma, 100 * mb, 100 * mc,
              100 * (mb - ma), 100 * lo1, 100 * hi1, 100 * (mb - mc), 100 * lo2, 100 * hi2);
  EXPECT_TRUE(std::isfinite(mb));
}

// ── Training on Jolt (rollout_kind::jolt) against the dynamic vehicle, scored on Jolt ──
//
// The Humvee at demo3's metric (0.05 normalized units per metre, its 3.30 m wheelbase 0.165), its
// own envelope in training and in the scoring (plan_with_vehicle): 0.8 of its 2.5 m/s^2, 0.5 g of
// cornering, its 0.457 rad lock. Each arm trains on the city with checkpoints selected on Jolt
// reach over validation seeds (200...), and the selected policies are scored on the 20 held-out
// seeds against the untrained seed with paired-bootstrap CIs. Arms (JOLT_EXP_ARMS, comma-separated
// kind:seed:lr[:horizon[:w_coll]]): jolt (the Jolt rollout, its four-wheel model calibrated to Jolt
// first: use_jolt_rollout) and dynamic (the single-track model). An experiment, not a check: it
// runs for about 20 minutes an arm and asserts only that its numbers are finite. It runs when
// CVC_JOLT_TRAIN_EXPERIMENT=1; JOLT_EXP_{STEPS, N, HORIZON, WINDOW, EVERY, VAL, TICKS, GOAL_MAX}
// size it (the defaults are the run docs/NAV_TRAINING.md reports), JOLT_EXP_OUT names a directory
// for the selected policies (.cvcnav), each arm's and the seed's per-seed held-out reach
// (.held.txt, for comparing two arms seed by seed) and a summary.
TEST(NavPlantConvergence, JoltTrainedHumveeOnJoltReach) {
  REQUIRE_JOLT();
  if (!std::getenv("CVC_JOLT_TRAIN_EXPERIMENT") ||
      std::string(std::getenv("CVC_JOLT_TRAIN_EXPERIMENT")) != "1")
    GTEST_SKIP() << "an experiment: set CVC_JOLT_TRAIN_EXPERIMENT=1 to run it";
  const auto env_int = [](const char *k, int d) {
    return std::getenv(k) ? std::atoi(std::getenv(k)) : d;
  };
  const std::string arms_spec = std::getenv("JOLT_EXP_ARMS")
                                    ? std::getenv("JOLT_EXP_ARMS")
                                    : "jolt:1:5e-5:60:10,dynamic:1:5e-5:60:10";
  const std::string out = std::getenv("JOLT_EXP_OUT") ? std::getenv("JOLT_EXP_OUT") : "";
  const cvc::nav::training_scene sc = cvc::nav::city_scene(96);
  const vehicle_desc hv = vehicle_desc::humvee();
  const double L = hv.wheelbase() * kS; // 0.165
  cvc::nav::train_config base;
  base.n = env_int("JOLT_EXP_N", 96);
  base.hidden = 64;
  base.horizon = env_int("JOLT_EXP_HORIZON", 60);
  base.window = env_int("JOLT_EXP_WINDOW", 10);
  base.steps = env_int("JOLT_EXP_STEPS", 1000);
  base.checkpoint_every = env_int("JOLT_EXP_EVERY", 50);
  base.veh_L = static_cast<float>(L);
  base.veh_delta_max = static_cast<float>(hv.max_steer);
  base.veh_a_max = static_cast<float>(0.8 * hv.max_accel * kS);
  base.veh_a_lat_max = static_cast<float>(0.5 * kG * kS);
  // Episodes of JOLT_EXP_TICKS of 0.06 s, goals JOLT_EXP_GOAL_MAX normalized units from the start
  // (100 m; 0 = anywhere, which the untrained seed reaches 8% of in 1000 ticks).
  plant_reach_options held; // the gate's 20 held-out seeds, 100..119
  held.plan_with_vehicle = true;
  held.ticks = env_int("JOLT_EXP_TICKS", 1000);
  const float goal_max = std::getenv("JOLT_EXP_GOAL_MAX")
                             ? static_cast<float>(std::atof(std::getenv("JOLT_EXP_GOAL_MAX")))
                             : 2.0f;
  if (goal_max > 0.0f) {
    held.goal_max = goal_max;
    held.goal_min = 0.25f * held.goal_max;
    base.goal_min = held.goal_min;
    base.goal_max = held.goal_max;
  }
  plant_reach_options val = held;
  val.seeds.clear();
  for (int k = 0; k < env_int("JOLT_EXP_VAL", 24); ++k)
    val.seeds.push_back(200u + static_cast<unsigned>(k));
  const auto jolt_reach = [&](const cvc::nav::coef_mlp &policy, int) {
    return reach_on_plant(policy, sc, hv, L, val);
  };
  const auto now = [] { return std::chrono::steady_clock::now(); };
  const auto mins = [](auto a, auto b) {
    return std::chrono::duration<double>(b - a).count() / 60.0;
  };

  // Each arm is scored on the held-out seeds and saved as soon as it is trained, so a later arm's
  // failure (reported, and the run goes on) cannot lose it.
  std::ofstream summary;
  if (!out.empty()) {
    std::filesystem::create_directories(out);
    summary.open(out + "/summary.txt", std::ios::app);
  }
  std::vector<double> seed_held;
  const double seed_mean =
      reach_on_plant(cvc::nav::coef_mlp::default_biased(), sc, hv, L, held, &seed_held);
  const auto report = [&](const std::string &line) {
    std::fputs(line.c_str(), stdout);
    std::fflush(stdout);
    if (summary) {
      summary << line;
      summary.flush();
    }
  };
  if (!out.empty()) {
    std::ofstream per_seed(out + "/seed.held.txt");
    for (std::size_t s = 0; s < seed_held.size(); ++s)
      per_seed << held.seeds[s] << " " << seed_held[s] << "\n";
  }
  {
    char line[256];
    std::snprintf(line, sizeof line,
                  "held-out reach on Jolt (%zu seeds, %d Humvees, %d ticks): seed (untrained) "
                  "%.1f%%\n",
                  held.seeds.size(), held.agents, held.ticks, 100 * seed_mean);
    report(line);
  }
  std::stringstream specs(arms_spec);
  std::string spec;
  while (std::getline(specs, spec, ',')) {
    if (spec.empty())
      continue;
    std::stringstream f(spec);
    std::string kind, seed_s, lr_s, horizon_s, wcoll_s;
    std::getline(f, kind, ':');
    std::getline(f, seed_s, ':');
    std::getline(f, lr_s, ':');
    std::getline(f, horizon_s, ':');
    std::getline(f, wcoll_s, ':');
    cvc::nav::train_config cfg = base;
    cfg.seed = static_cast<unsigned>(std::atoi(seed_s.c_str()));
    cfg.lr = static_cast<float>(std::atof(lr_s.c_str()));
    if (!horizon_s.empty())
      cfg.horizon = std::atoi(horizon_s.c_str());
    if (!wcoll_s.empty())
      cfg.w_coll = static_cast<float>(std::atof(wcoll_s.c_str()));
    try {
      const auto t0 = now();
      if (kind == "jolt") {
        const cvc::nav::dyn4_calibration cal =
            use_jolt_rollout(cfg, sc, hv, 32, 200, 7u + cfg.seed);
        std::printf(
            "[%s] calibrated in %.1f min: drive %.3f brake %.3f comp %.3f lag %.3f s, tire x%.3f "
            "drag x%.3f rolling x%.3f damper x%.3f, price %.3f; fitted books dissipated %+.3f "
            "drawn %+.3f, twist %.4f\n",
            spec.c_str(), mins(t0, now()), cal.drive_gain, cal.brake_gain, cal.resist_comp,
            cal.tau_drive, cal.tire, cal.drag, cal.rolling, cal.damper, cal.price.drive,
            cal.after.dissipated.error(), cal.after.drawn.error(), cal.after.twist_rms);
      } else if (kind == "dynamic") {
        cfg.rollout = cvc::nav::rollout_kind::dynamic;
        cfg.dyn = nav_plant::dyn_bike_params_at(hv, cfg.veh_L);
      } else {
        ADD_FAILURE() << "unknown arm kind " << kind;
        continue;
      }
      std::fflush(stdout);
      cvc::nav::coef_trainer tr(cfg, cfg.seed);
      const auto t1 = now();
      tr.train(sc, true, jolt_reach);
      std::printf("[%s] trained %d steps in %.1f min (%ld agents' gradients left out); "
                  "checkpoints (validation reach):",
                  spec.c_str(), cfg.steps, mins(t1, now()), tr.dropped_agents());
      for (const cvc::nav::coef_trainer::checkpoint_score &k : tr.checkpoints())
        std::printf(" %d:%.3f", k.step, k.fitness);
      std::printf("; selected step %d\n", tr.selected_step());
      std::fflush(stdout);
      const cvc::nav::coef_mlp policy = tr.to_coef_mlp();
      std::vector<double> a;
      const double mean = reach_on_plant(policy, sc, hv, L, held, &a);
      std::vector<double> d(a.size());
      for (std::size_t s = 0; s < d.size(); ++s)
        d[s] = a[s] - seed_held[s];
      double lo = 0.0, hi = 0.0;
      paired_ci(d, lo, hi);
      char line[256];
      std::snprintf(line, sizeof line,
                    "  %-22s %.1f%%  vs the seed %+.1f [%+.1f, %+.1f]  (selected step %d)\n",
                    spec.c_str(), 100 * mean, 100 * (mean - seed_mean), 100 * lo, 100 * hi,
                    tr.selected_step());
      report(line);
      if (!out.empty()) {
        std::string file = spec;
        for (char &c : file)
          if (c == ':')
            c = '_';
        // Each held-out seed's reach, in held.seeds order, for comparing two arms seed by seed.
        std::ofstream per_seed(out + "/" + file + ".held.txt");
        for (std::size_t s = 0; s < a.size(); ++s)
          per_seed << held.seeds[s] << " " << a[s] << "\n";
        policy.save(out + "/" + file + ".cvcnav",
                    "trained on the city at demo3's metric (" + spec + "), selected on Jolt reach");
      }
      EXPECT_TRUE(std::isfinite(mean));
    } catch (const std::exception &e) {
      report("  " + spec + " FAILED: " + e.what() + "\n");
      ADD_FAILURE() << spec << ": " << e.what();
    }
  }
}

// ── P8c's gate: a policy trained with the energy term against one trained without it, both on
// the dynamic vehicle, both driving Jolt RoboRacers ──

// The RoboRacer's power model is fitted to its own ledger first, as PlantLogPowerConvergence
// fits it (four seeds each of the swerve and the city, 30 s), and converted to the city's scale.
// The two trainings share everything but w_energy. On the gate's 20 held-out seeds it reports
// reach and Wh/km for both and the untrained seed, with paired-bootstrap CIs of the differences:
// the gate is that the energy-trained policy draws less per km for what it gives up in reach.
// P8C_GATE_W (the energy weight, 3), P8C_GATE_REF (train_config::energy_ref as a Wh/km; 0, the
// default, prices time), P8C_GATE_LR (5e-5) and P8C_GATE_STEPS (150) override it.
TEST(NavPlantConvergence, EnergyTrainedPolicyOnJolt) {
  REQUIRE_JOLT();
  const vehicle_desc rr = vehicle_desc::roboracer();
  std::vector<std::vector<planar_sample>> runs;
  double dt = 0.0;
  for (const char *scene : {"swerve", "city"}) {
    plant_log_options lo;
    lo.vehicle = "roboracer";
    lo.scene = scene;
    lo.seconds = 30.0;
    lo.seeds = {100, 101, 102, 103};
    lo.resolve();
    dt = lo.dt;
    const plant_log_drive drv = drive_plant_log_scene(lo, cvc::nav::coef_mlp::default_biased());
    runs.insert(runs.end(), drv.runs.begin(), drv.runs.end());
  }
  const cvc::nav::training_scene sc = cvc::nav::city_scene(96);
  // The dense city's tick, as plant_log reads it: the scene's dt in whole physics steps.
  const double fixed = world_config().fixed_dt;
  const double dense_dt = std::max(1.0, std::round(sc.dt / fixed)) * fixed;
  std::vector<power_runs> sets;
  // P8C_GATE_DENSE (on by default; 0 = off): also fit on the gate's own dense city, 8 seeds
  // (300..307, never the evaluation's or validation's) of the untrained policy.
  bool dense = true;
  if (const char *e = std::getenv("P8C_GATE_DENSE"))
    dense = std::atoi(e) != 0;
  if (dense) {
    plant_reach_options fo;
    fo.seeds.clear();
    for (unsigned k = 0; k < 8; ++k)
      fo.seeds.push_back(300u + k);
    std::vector<std::vector<planar_sample>> logs;
    score_on_plant({cvc::nav::coef_mlp::default_biased()}, sc, rr, kCityL, fo, nullptr, &logs);
    sets.push_back({std::move(logs), dense_dt});
  }
  sets.push_back({std::move(runs), dt});
  const power_fit fit = fit_power(sets, rr);
  std::printf(
      "  the RoboRacer's fitted power model%s: k_pos %.3f k_neg %.3f p_idle %.1f W c_v %.2f "
      "c_v3 %.4f c_lat %.4f tire_lat %.3f tire_lon %.3f (r2 %.3f; the tire term against "
      "the ledger's tire: %.3f lateral, %.3f longitudinal per J, r2 %.3f)\n",
      dense ? " (with the dense city)" : "", fit.si.k_pos, fit.si.k_neg, fit.si.p_idle, fit.si.c_v,
      fit.si.c_v3, fit.si.c_lat, fit.si.tire_lat, fit.si.tire_lon, fit.r2, fit.tire_cal_lat,
      fit.tire_cal_lon, fit.tire_r2);

  cvc::nav::train_config off;
  off.n = 128;
  off.hidden = 64;
  off.horizon = 28;
  off.window = 7;
  off.steps = 150;
  off.seed = 0;
  off.lr = 5e-5f;
  if (const char *e = std::getenv("P8C_GATE_LR"))
    off.lr = static_cast<float>(std::atof(e));
  if (const char *e = std::getenv("P8C_GATE_STEPS"))
    off.steps = std::atoi(e);
  off.rollout = cvc::nav::rollout_kind::dynamic;
  off.dyn = nav_plant::dyn_bike_params_at(rr, off.veh_L);
  cvc::world_units::normalized_frame f;
  f.scale = off.veh_L / rr.wheelbase(); // normalized units per metre, as dyn_bike_params_at
  off.power = cvc::nav::dyn_power_params_of(fit.si, rr.mass, f);
  cvc::nav::train_config on = off;
  on.w_energy = 3.0f;
  if (const char *e = std::getenv("P8C_GATE_W"))
    on.w_energy = static_cast<float>(std::atof(e));
  double ref_whkm = 0.0;
  if (const char *e = std::getenv("P8C_GATE_REF"))
    ref_whkm = std::atof(e);
  on.energy_ref = cvc::nav::dyn_energy_ref_of(ref_whkm, f);
  cvc::nav::coef_trainer t_off(off, 1), t_on(on, 1);
  t_off.train(sc);
  t_on.train(sc);

  plant_reach_options eval;   // 20 seeds (100..119) x 48 agents x 400 ticks
  eval.predict_energy = true; // and what the fitted model says the same motion drew
  eval.power = fit.si;
  std::vector<std::vector<plant_drive_score>> per;
  const std::vector<plant_drive_score> m = score_on_plant(
      {t_off.to_coef_mlp(), t_on.to_coef_mlp(), cvc::nav::coef_mlp::default_biased()}, sc, rr,
      kCityL, eval, &per);
  const std::size_t ns = eval.seeds.size();
  std::vector<double> d_reach(ns), d_whkm(ns), d_reach_seed(ns), d_whkm_seed(ns);
  for (std::size_t s = 0; s < ns; ++s) {
    d_reach[s] = per[1][s].reach - per[0][s].reach;
    d_whkm[s] = per[1][s].wh_per_km() - per[0][s].wh_per_km();
    d_reach_seed[s] = per[1][s].reach - per[2][s].reach;
    d_whkm_seed[s] = per[1][s].wh_per_km() - per[2][s].wh_per_km();
  }
  double rlo, rhi, wlo, whi, rlo2, rhi2, wlo2, whi2;
  paired_ci(d_reach, rlo, rhi);
  paired_ci(d_whkm, wlo, whi);
  paired_ci(d_reach_seed, rlo2, rhi2);
  paired_ci(d_whkm_seed, wlo2, whi2);
  std::printf("  P8c gate (w_energy %.2f, priced at %.1f Wh/km of progress (0 = time), lr %.0e, "
              "%d steps) on Jolt RoboRacers (city, %zu seeds "
              "x %d agents, %d ticks):\n"
              "    without the energy term: reach %.1f%%, %.2f Wh/km (%.0f m a seed)\n"
              "    with it:                 reach %.1f%%, %.2f Wh/km (%.0f m a seed)\n"
              "    the untrained seed:      reach %.1f%%, %.2f Wh/km (%.0f m a seed)\n"
              "    with - without: reach %+.1f points [%+.1f, %+.1f], %+.2f Wh/km [%+.2f, %+.2f]\n"
              "    with - seed:    reach %+.1f points [%+.1f, %+.1f], %+.2f Wh/km [%+.2f, %+.2f]\n",
              on.w_energy, ref_whkm, off.lr, off.steps, ns, eval.agents, eval.ticks,
              100 * m[0].reach, m[0].wh_per_km(), m[0].distance_m, 100 * m[1].reach,
              m[1].wh_per_km(), m[1].distance_m, 100 * m[2].reach, m[2].wh_per_km(),
              m[2].distance_m, 100 * (m[1].reach - m[0].reach), 100 * rlo, 100 * rhi,
              m[1].wh_per_km() - m[0].wh_per_km(), wlo, whi, 100 * (m[1].reach - m[2].reach),
              100 * rlo2, 100 * rhi2, m[1].wh_per_km() - m[2].wh_per_km(), wlo2, whi2);
  for (std::size_t s = 0; s < ns; ++s)
    std::printf("    seed %zu: reach %.3f %.3f %.3f, Wh/km %.2f %.2f %.2f, model %.2f %.2f %.2f\n",
                100 + s, per[0][s].reach, per[1][s].reach, per[2][s].reach, per[0][s].wh_per_km(),
                per[1][s].wh_per_km(), per[2][s].wh_per_km(), per[0][s].model_wh_per_km(),
                per[1][s].model_wh_per_km(), per[2][s].model_wh_per_km());

  // Does the model predict the differences it trains on? Over the same Jolt runs: its Wh/km per
  // policy and its error, its paired differences beside the measured ones, and how well its
  // per-seed differences follow the measured ones (Pearson r over the seeds).
  const char *names[3] = {"without", "with", "seed"};
  for (int k = 0; k < 3; ++k)
    std::printf("    model on %-7s: %.2f Wh/km against %.2f measured (%+.1f%%)\n", names[k],
                m[k].model_wh_per_km(), m[k].wh_per_km(),
                100.0 * (m[k].model_J - m[k].energy_J) / m[k].energy_J);
  const auto corr = [](const std::vector<double> &a, const std::vector<double> &b) {
    const double n = static_cast<double>(a.size());
    double ma = 0.0, mb = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
      ma += a[i] / n;
      mb += b[i] / n;
    }
    double sab = 0.0, saa = 0.0, sbb = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
      sab += (a[i] - ma) * (b[i] - mb);
      saa += (a[i] - ma) * (a[i] - ma);
      sbb += (b[i] - mb) * (b[i] - mb);
    }
    return saa > 0.0 && sbb > 0.0 ? sab / std::sqrt(saa * sbb) : 0.0;
  };
  const int pairs[3][2] = {{1, 0}, {1, 2}, {0, 2}};
  const char *pair_names[3] = {"with - without", "with - seed", "without - seed"};
  for (int q = 0; q < 3; ++q) {
    const int x = pairs[q][0], y = pairs[q][1];
    std::vector<double> dm(ns), dp(ns);
    for (std::size_t s = 0; s < ns; ++s) {
      dm[s] = per[x][s].wh_per_km() - per[y][s].wh_per_km();
      dp[s] = per[x][s].model_wh_per_km() - per[y][s].model_wh_per_km();
    }
    double plo, phi;
    paired_ci(dp, plo, phi);
    std::printf("    %-15s Wh/km: model %+.2f [%+.2f, %+.2f], measured %+.2f; per-seed r = %.2f\n",
                pair_names[q], m[x].model_wh_per_km() - m[y].model_wh_per_km(), plo, phi,
                m[x].wh_per_km() - m[y].wh_per_km(), corr(dp, dm));
  }
  std::vector<double> all_m, all_p; // every (policy, seed) run's Wh/km
  for (int k = 0; k < 3; ++k)
    for (std::size_t s = 0; s < ns; ++s) {
      all_m.push_back(per[k][s].wh_per_km());
      all_p.push_back(per[k][s].model_wh_per_km());
    }
  std::printf("    per-run Wh/km, model against measured over all %zu runs: r = %.2f\n",
              all_m.size(), corr(all_p, all_m));
  // Where each policy's energy went, from the ledger (Wh/km of the fleet's distance).
  for (int k = 0; k < 3; ++k) {
    const vehicle_energy &e = m[k].ledger;
    const double perkm = 1.0 / 3600.0 / (m[k].distance_m / 1000.0);
    std::printf("    ledger %-7s Wh/km: source %.2f engine %.2f idle %.2f | brake %.2f tire %.2f "
                "drivetrain %.2f rolling %.2f aero %.2f suspension %.2f chassis %.2f | stored "
                "%.2f residual %.2f regen %.2f\n",
                names[k], e.source * perkm, e.engine * perkm, e.idle * perkm, e.brake * perkm,
                e.tire * perkm, e.drivetrain * perkm, e.rolling * perkm, e.aero * perkm,
                e.suspension * perkm, e.chassis_damping * perkm, e.stored * perkm,
                e.residual * perkm, e.regen * perkm);
  }
  EXPECT_TRUE(std::isfinite(m[1].wh_per_km()));
  EXPECT_GT(m[1].model_J, 0.0);
}

// The zeroth-order fine-tune's mechanics, small and quick: a batch of policies scores what each
// scores alone; the trace has the start and every update; the result is never below the start
// on validation; and a rerun repeats it exactly.
TEST(NavPlant, EsFinetuneOnPlantMechanics) {
  REQUIRE_JOLT();
  const cvc::nav::training_scene sc = cvc::nav::city_scene(96);
  const vehicle_desc rr = vehicle_desc::roboracer();
  cvc::nav::train_config cfg;
  cfg.hidden = 16;
  const cvc::nav::coef_mlp p = cvc::nav::coef_trainer(cfg, 1).to_coef_mlp();
  const cvc::nav::coef_mlp q = cvc::nav::coef_trainer(cfg, 2).to_coef_mlp();
  plant_reach_options few;
  few.agents = 8;
  few.ticks = 80;
  few.seeds = {200u, 201u};
  std::vector<std::vector<double>> per;
  const std::vector<double> both = reach_on_plant({p, q}, sc, rr, kCityL, few, &per);
  std::vector<double> pa, qa;
  EXPECT_EQ(both[0], reach_on_plant(p, sc, rr, kCityL, few, &pa));
  EXPECT_EQ(both[1], reach_on_plant(q, sc, rr, kCityL, few, &qa));
  EXPECT_EQ(per[0], pa);
  EXPECT_EQ(per[1], qa);

  es_options o;
  o.iterations = 2;
  o.pairs = 2;
  o.train_seeds = 1;
  o.validate_every = 1;
  o.reach = few;
  const es_result a = es_finetune_on_plant(p, sc, rr, kCityL, o);
  ASSERT_EQ(a.trace.size(), 3u);
  EXPECT_EQ(a.trace[0].iteration, 0);
  EXPECT_TRUE(std::isnan(a.trace[0].population_mean));
  EXPECT_DOUBLE_EQ(a.trace[0].validation, 0.5 * (pa[0] + pa[1]));
  EXPECT_GE(a.best_validation, a.trace[0].validation);
  for (int i = 1; i <= 2; ++i) {
    EXPECT_EQ(a.trace[i].iteration, i);
    EXPECT_TRUE(std::isfinite(a.trace[i].population_mean));
    EXPECT_TRUE(std::isfinite(a.trace[i].validation));
  }
  const es_result b = es_finetune_on_plant(p, sc, rr, kCityL, o);
  EXPECT_EQ(b.best_iteration, a.best_iteration);
  EXPECT_EQ(b.best.export_flat().data, a.best.export_flat().data);
  std::printf("  ES mechanics: validation %.3f -> %.3f -> %.3f, best at %d\n",
              a.trace[0].validation, a.trace[1].validation, a.trace[2].validation,
              a.best_iteration);

  // With a fitness (P8c): the energy at held reach. The start's validation is minus its own
  // Wh/km (it loses no reach against itself), each validated centre carries its reach and Wh/km,
  // and a rerun repeats it.
  o.fitness = es_energy_at_held_reach(0.1, 1.0);
  const es_result c = es_finetune_on_plant(p, sc, rr, kCityL, o);
  std::vector<plant_drive_score> ps = score_on_plant({p}, sc, rr, kCityL, few);
  ASSERT_EQ(c.trace.size(), 3u);
  EXPECT_DOUBLE_EQ(c.trace[0].validation, -ps[0].wh_per_km());
  EXPECT_DOUBLE_EQ(c.trace[0].validation_reach, ps[0].reach);
  EXPECT_GE(c.best_validation, c.trace[0].validation);
  for (int i = 1; i <= 2; ++i) {
    EXPECT_TRUE(std::isfinite(c.trace[i].validation_wh_per_km));
    EXPECT_LE(c.trace[i].validation, -c.trace[i].validation_wh_per_km + 1e-12);
  }
  const es_result e = es_finetune_on_plant(p, sc, rr, kCityL, o);
  EXPECT_EQ(e.best.export_flat().data, c.best.export_flat().data);
  // The fitness itself: Wh/km, and the price of reach lost past the slack.
  plant_drive_score x, ref;
  x.energy_J = 3600.0;
  x.distance_m = 1000.0; // 1 Wh/km
  ref.reach = 0.50;
  x.reach = 0.45; // 5 points lost, 4 past the slack
  EXPECT_NEAR(es_energy_at_held_reach(0.1, 1.0)(x, ref), -1.0 - 0.4, 1e-12);
  x.reach = 0.6;
  EXPECT_NEAR(es_energy_at_held_reach(0.1, 1.0)(x, ref), -1.0, 1e-12);
  EXPECT_THROW(es_energy_at_held_reach(-1.0), std::invalid_argument);
}

// P8c's gate by ES on Jolt itself: the energy no differentiable model has priced (most of the
// trained policies' extra draw is tire slip in contacts, docs/NAV_TRAINING.md), climbed on the
// plant's own measured Wh/km at held reach (es_energy_at_held_reach). The start is the P8b gate's
// dynamic-trained policy (lr 5e-5, 150 steps, no energy term), which reaches about what the
// untrained seed does. Its fine-tune validates on 16 seeds (200..215) and is judged against the
// start on the gate's 20 held-out seeds, with paired-bootstrap CIs. P8C_ES_ITERS (30),
// P8C_ES_PAIRS (12), P8C_ES_SIGMA (0.02), P8C_ES_LR (0.005), P8C_ES_TAIL (0: every weight),
// P8C_ES_PRICE (Wh/km per point of reach, 0.1) and P8C_ES_SLACK (points, 1) override it.
TEST(NavPlantConvergence, EsEnergyAtHeldReachOnJolt) {
  REQUIRE_ES_EXPERIMENTS();
  REQUIRE_JOLT();
  const cvc::nav::training_scene sc = cvc::nav::city_scene(96);
  const vehicle_desc rr = vehicle_desc::roboracer();
  const auto env = [](const char *k, double v) {
    const char *e = std::getenv(k);
    return e ? std::atof(e) : v;
  };
  cvc::nav::train_config base;
  base.n = 128;
  base.hidden = 64;
  base.horizon = 28;
  base.window = 7;
  base.steps = 150;
  base.seed = 0;
  base.lr = 5e-5f;
  base.rollout = cvc::nav::rollout_kind::dynamic;
  base.dyn = nav_plant::dyn_bike_params_at(rr, base.veh_L);
  cvc::nav::coef_trainer tr(base, 1);
  tr.train(sc);
  const cvc::nav::coef_mlp start = tr.to_coef_mlp();

  es_options o;
  o.iterations = static_cast<int>(env("P8C_ES_ITERS", 30));
  o.pairs = static_cast<int>(env("P8C_ES_PAIRS", 12));
  o.sigma = static_cast<float>(env("P8C_ES_SIGMA", 0.02));
  o.lr = static_cast<float>(env("P8C_ES_LR", 0.005));
  o.tail = static_cast<int>(env("P8C_ES_TAIL", 0));
  o.train_seeds = 4;
  o.validate_every = 5;
  o.reach.seeds.clear();
  for (unsigned k = 0; k < 16; ++k)
    o.reach.seeds.push_back(200u + k);
  const double price = env("P8C_ES_PRICE", 0.1), slack = env("P8C_ES_SLACK", 1.0);
  o.fitness = es_energy_at_held_reach(price, slack);
  const es_result r = es_finetune_on_plant(start, sc, rr, kCityL, o, [](const es_iteration &t) {
    if (std::isfinite(t.validation))
      std::printf("    ES %3d: population %+.3f, validation %+.3f (reach %.3f, %.2f Wh/km)\n",
                  t.iteration, t.population_mean, t.validation, t.validation_reach,
                  t.validation_wh_per_km);
    else
      std::printf("    ES %3d: population %+.3f\n", t.iteration, t.population_mean);
    std::fflush(stdout);
  });

  const plant_reach_options eval; // the gate's 20 held-out seeds
  std::vector<std::vector<plant_drive_score>> per;
  const std::vector<plant_drive_score> m = score_on_plant(
      {start, r.best, cvc::nav::coef_mlp::default_biased()}, sc, rr, kCityL, eval, &per);
  const std::size_t ns = eval.seeds.size();
  std::vector<double> d_reach(ns), d_whkm(ns);
  for (std::size_t s = 0; s < ns; ++s) {
    d_reach[s] = per[1][s].reach - per[0][s].reach;
    d_whkm[s] = per[1][s].wh_per_km() - per[0][s].wh_per_km();
  }
  double rlo, rhi, wlo, whi;
  paired_ci(d_reach, rlo, rhi);
  paired_ci(d_whkm, wlo, whi);
  std::printf("  P8c by ES on Jolt (%d iterations x %d pairs, sigma %.3f, lr %.3f, tail %d; price "
              "%.2f Wh/km a point past %.1f), best at %d:\n"
              "    the start (dynamic-trained): reach %.1f%%, %.2f Wh/km (%.0f m a seed)\n"
              "    fine-tuned:                  reach %.1f%%, %.2f Wh/km (%.0f m a seed)\n"
              "    the untrained seed:          reach %.1f%%, %.2f Wh/km (%.0f m a seed)\n"
              "    fine-tuned - start: reach %+.1f points [%+.1f, %+.1f], %+.2f Wh/km [%+.2f, "
              "%+.2f]\n",
              o.iterations, o.pairs, o.sigma, o.lr, o.tail, price, slack, r.best_iteration,
              100 * m[0].reach, m[0].wh_per_km(), m[0].distance_m, 100 * m[1].reach,
              m[1].wh_per_km(), m[1].distance_m, 100 * m[2].reach, m[2].wh_per_km(),
              m[2].distance_m, 100 * (m[1].reach - m[0].reach), 100 * rlo, 100 * rhi,
              m[1].wh_per_km() - m[0].wh_per_km(), wlo, whi);
  const char *names[3] = {"start", "tuned", "seed"};
  for (int k = 0; k < 3; ++k) {
    const vehicle_energy &e = m[k].ledger;
    const double perkm = 1.0 / 3600.0 / (m[k].distance_m / 1000.0);
    std::printf("    ledger %-5s Wh/km: engine %.2f | brake %.2f tire %.2f drivetrain %.2f aero "
                "%.2f rolling %.2f | regen %.2f\n",
                names[k], e.engine * perkm, e.brake * perkm, e.tire * perkm, e.drivetrain * perkm,
                e.aero * perkm, e.rolling * perkm, e.regen * perkm);
  }
  for (std::size_t s = 0; s < ns; ++s)
    std::printf("    seed %zu: reach %.3f %.3f %.3f, Wh/km %.2f %.2f %.2f\n", 100 + s,
                per[0][s].reach, per[1][s].reach, per[2][s].reach, per[0][s].wh_per_km(),
                per[1][s].wh_per_km(), per[2][s].wh_per_km());
  EXPECT_GE(r.best_validation, r.trace[0].validation);
}

// The zeroth-order fine-tune on Jolt reach (P8b): the dynamic-trained policy at its selected
// checkpoint (1e-4, 300 steps, step 130 of the 32-seed selection) climbs reach_on_plant itself
// by antithetic ES, validated on 32 seeds (200..231), then faces the gate's 20 held-out seeds
// against its start and the seed. P8B_ES_ITERS / _PAIRS / _SIGMA / _LR / _TRAIN_SEEDS / _TAIL
// override es_options; P8B_ES_START=seed starts from coef_mlp::default_biased() instead (use it
// with P8B_ES_TAIL=3, the basin).
TEST(NavPlantConvergence, EsFinetuneOnJoltReach) {
  REQUIRE_ES_EXPERIMENTS();
  REQUIRE_JOLT();
  const cvc::nav::training_scene sc = cvc::nav::city_scene(96);
  const vehicle_desc rr = vehicle_desc::roboracer();
  cvc::nav::train_config dc;
  dc.n = 128;
  dc.hidden = 64;
  dc.horizon = 28;
  dc.window = 7;
  dc.steps = 300;
  dc.seed = 0;
  dc.lr = 1e-4f;
  dc.checkpoint_every = 10;
  dc.rollout = cvc::nav::rollout_kind::dynamic;
  dc.dyn = nav_plant::dyn_bike_params_at(rr, dc.veh_L);
  cvc::nav::coef_trainer dy(dc, 1);
  const char *which = std::getenv("P8B_ES_START");
  const bool from_seed = which && std::string(which) == "seed";
  cvc::nav::coef_mlp start = cvc::nav::coef_mlp::default_biased();
  if (!from_seed) {
    // Its selected checkpoint without re-scoring every one on Jolt: a fitness that peaks there.
    dy.train(sc, false,
             [](const cvc::nav::coef_mlp &, int step) { return step == 130 ? 1.0 : 0.0; });
    start = dy.to_coef_mlp();
  }

  es_options o;
  o.reach.seeds.clear();
  for (unsigned s = 200; s < 232; ++s)
    o.reach.seeds.push_back(s);
  if (const char *e = std::getenv("P8B_ES_ITERS"))
    o.iterations = std::atoi(e);
  if (const char *e = std::getenv("P8B_ES_PAIRS"))
    o.pairs = std::atoi(e);
  if (const char *e = std::getenv("P8B_ES_SIGMA"))
    o.sigma = static_cast<float>(std::atof(e));
  if (const char *e = std::getenv("P8B_ES_LR"))
    o.lr = static_cast<float>(std::atof(e));
  if (const char *e = std::getenv("P8B_ES_TRAIN_SEEDS"))
    o.train_seeds = std::atoi(e);
  if (const char *e = std::getenv("P8B_ES_TAIL"))
    o.tail = std::atoi(e);
  double rms = 0.0;
  for (const float x : start.export_flat().data)
    rms += static_cast<double>(x) * x;
  rms = std::sqrt(rms / static_cast<double>(start.export_flat().data.size()));
  std::printf("  ES from %s (weight RMS %.4f): %d iterations of %d pairs, sigma %.3f, lr %.4f, "
              "%d training seeds, tail %d\n",
              from_seed ? "the seed" : "the dynamic policy at step 130", rms, o.iterations, o.pairs,
              o.sigma, o.lr, o.train_seeds, o.tail);
  const es_result r = es_finetune_on_plant(start, sc, rr, kCityL, o, [](const es_iteration &t) {
    std::printf("    iteration %3d: population %.3f, validation %s\n", t.iteration,
                t.population_mean,
                std::isnan(t.validation) ? "-" : std::to_string(t.validation).substr(0, 5).c_str());
    std::fflush(stdout);
  });

  const plant_reach_options eval; // the gate's 20 held-out seeds
  std::vector<double> a, b, c;
  const double ma = reach_on_plant(start, sc, rr, kCityL, eval, &a);
  const double mb = reach_on_plant(r.best, sc, rr, kCityL, eval, &b);
  const double mc = reach_on_plant(cvc::nav::coef_mlp::default_biased(), sc, rr, kCityL, eval, &c);
  std::vector<double> d_start(a.size()), d_seed(a.size());
  for (std::size_t s = 0; s < a.size(); ++s) {
    d_start[s] = b[s] - a[s];
    d_seed[s] = b[s] - c[s];
  }
  double lo1, hi1, lo2, hi2;
  paired_ci(d_start, lo1, hi1);
  paired_ci(d_seed, lo2, hi2);
  std::printf("  ES best at iteration %d (validation %.3f from %.3f); held-out reach: %.1f%% "
              "tuned, %.1f%% its start, %.1f%% the seed; tuned - start %+.1f [%+.1f, %+.1f], "
              "tuned - seed %+.1f [%+.1f, %+.1f]\n",
              r.best_iteration, r.best_validation, r.trace[0].validation, 100 * mb, 100 * ma,
              100 * mc, 100 * (mb - ma), 100 * lo1, 100 * hi1, 100 * (mb - mc), 100 * lo2,
              100 * hi2);
  EXPECT_GE(r.best_validation, r.trace[0].validation);
}

// How much the policy moves Jolt reach at all (P8b's ES diagnosis): on the 32 validation seeds,
// the seed's constant coefficients halved and doubled one at a time, the dynamic-trained policy
// at its selected checkpoint, and random perturbations of it at three sizes.
TEST(NavPlantConvergence, CoefficientLeverageOnJoltReach) {
  REQUIRE_JOLT();
  const cvc::nav::training_scene sc = cvc::nav::city_scene(96);
  const vehicle_desc rr = vehicle_desc::roboracer();
  plant_reach_options val;
  val.seeds.clear();
  for (unsigned s = 200; s < 232; ++s)
    val.seeds.push_back(s);
  // A constant-coefficient policy: zero weights, the basin (alpha, beta, gamma) as its bias.
  const auto basin = [](float al, float be, float ga) {
    const std::vector<int> rows = {64, 64, 3}, cols = {5, 64, 64};
    std::vector<std::vector<float>> w(3), b(3);
    for (int L = 0; L < 3; ++L) {
      w[L].assign(static_cast<std::size_t>(rows[L] * cols[L]), 0.0f);
      b[L].assign(static_cast<std::size_t>(rows[L]), 0.0f);
    }
    return cvc::nav::coef_mlp::from_layers(5, 3, rows, cols, {1, 1, 0}, w, b, {al, be, ga});
  };
  std::vector<std::string> names;
  std::vector<cvc::nav::coef_mlp> pols;
  const auto add = [&](const std::string &name, const cvc::nav::coef_mlp &p) {
    names.push_back(name);
    pols.push_back(p);
  };
  add("seed (1, 3, 4)", basin(1, 3, 4));
  add("alpha x0.5", basin(0.5f, 3, 4));
  add("alpha x2", basin(2, 3, 4));
  add("beta x0.5", basin(1, 1.5f, 4));
  add("beta x2", basin(1, 6, 4));
  add("gamma x0.5", basin(1, 3, 2));
  add("gamma x2", basin(1, 3, 8));
  cvc::nav::train_config dc;
  dc.n = 128;
  dc.hidden = 64;
  dc.horizon = 28;
  dc.window = 7;
  dc.steps = 300;
  dc.seed = 0;
  dc.lr = 1e-4f;
  dc.checkpoint_every = 10;
  dc.rollout = cvc::nav::rollout_kind::dynamic;
  dc.dyn = nav_plant::dyn_bike_params_at(rr, dc.veh_L);
  cvc::nav::coef_trainer dy(dc, 1);
  dy.train(sc, false, [](const cvc::nav::coef_mlp &, int step) { return step == 130 ? 1.0 : 0.0; });
  const cvc::nav::coef_mlp start = dy.to_coef_mlp();
  add("dynamic-trained, step 130", start);
  const std::vector<float> theta = start.export_flat().data;
  std::mt19937 rng(7);
  for (const float sigma : {0.02f, 0.05f, 0.1f})
    for (int k = 0; k < 2; ++k) {
      std::vector<float> w = theta;
      for (float &x : w) {
        // Box-Muller on raw mt19937 output, as nav_es does.
        const double u1 = (static_cast<double>(rng()) + 0.5) / 4294967296.0,
                     u2 = (static_cast<double>(rng()) + 0.5) / 4294967296.0;
        x += sigma *
             static_cast<float>(std::sqrt(-2.0 * std::log(u1)) * std::cos(6.283185307179586 * u2));
      }
      add("  + sigma " + std::to_string(sigma).substr(0, 4) + " #" + std::to_string(k + 1),
          start.with_flat_weights(w));
    }
  const std::vector<double> m = reach_on_plant(pols, sc, rr, kCityL, val);
  std::printf("  reach on the 32 validation seeds (about 0.5 points of chaos each):\n");
  for (std::size_t i = 0; i < pols.size(); ++i)
    std::printf("    %-28s %.1f%%\n", names[i].c_str(), 100 * m[i]);
  EXPECT_EQ(m.size(), pols.size());
}

// A zeroth-order search over the coefficient basin alone (P8b): constant (alpha, beta, gamma)
// policies on a grid, scored on the 32 validation seeds; the top three then face the gate's 20
// held-out seeds against the seed's (1, 3, 4).
TEST(NavPlantConvergence, BasinGridOnJoltReach) {
  REQUIRE_JOLT();
  const cvc::nav::training_scene sc = cvc::nav::city_scene(96);
  const vehicle_desc rr = vehicle_desc::roboracer();
  plant_reach_options val;
  val.seeds.clear();
  for (unsigned s = 200; s < 232; ++s)
    val.seeds.push_back(s);
  const auto basin = [](float al, float be, float ga) {
    const std::vector<int> rows = {64, 64, 3}, cols = {5, 64, 64};
    std::vector<std::vector<float>> w(3), b(3);
    for (int L = 0; L < 3; ++L) {
      w[L].assign(static_cast<std::size_t>(rows[L] * cols[L]), 0.0f);
      b[L].assign(static_cast<std::size_t>(rows[L]), 0.0f);
    }
    return cvc::nav::coef_mlp::from_layers(5, 3, rows, cols, {1, 1, 0}, w, b, {al, be, ga});
  };
  std::vector<std::array<float, 3>> grid;
  for (const float be : {3.0f, 4.5f, 6.0f, 9.0f})
    for (const float ga : {2.0f, 3.0f, 4.0f})
      grid.push_back({1.0f, be, ga});
  std::vector<cvc::nav::coef_mlp> pols;
  for (const std::array<float, 3> &g : grid)
    pols.push_back(basin(g[0], g[1], g[2]));
  const std::vector<double> m = reach_on_plant(pols, sc, rr, kCityL, val);
  std::vector<std::size_t> order(grid.size());
  for (std::size_t i = 0; i < order.size(); ++i)
    order[i] = i;
  std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return m[a] > m[b]; });
  std::printf("  basin grid on the 32 validation seeds (alpha 1):\n");
  for (const std::size_t i : order)
    std::printf("    beta %.1f gamma %.1f: %.1f%%\n", grid[i][1], grid[i][2], 100 * m[i]);
  const plant_reach_options eval; // the gate's 20 held-out seeds
  std::vector<double> c;
  const double mc = reach_on_plant(basin(1, 3, 4), sc, rr, kCityL, eval, &c);
  for (int k = 0; k < 3; ++k) {
    const std::size_t i = order[k];
    std::vector<double> b;
    const double mb = reach_on_plant(pols[i], sc, rr, kCityL, eval, &b);
    std::vector<double> d(b.size());
    for (std::size_t s = 0; s < b.size(); ++s)
      d[s] = b[s] - c[s];
    double lo, hi;
    paired_ci(d, lo, hi);
    std::printf("  held out: beta %.1f gamma %.1f reaches %.1f%% against the seed's %.1f%%: %+.1f "
                "[%+.1f, %+.1f]\n",
                grid[i][1], grid[i][2], 100 * mb, 100 * mc, 100 * (mb - mc), 100 * lo, 100 * hi);
  }
  EXPECT_EQ(m.size(), grid.size());
}
