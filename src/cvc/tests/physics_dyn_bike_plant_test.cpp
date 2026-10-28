/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics::dyn_bike_plant (dyn_bike_plant.h): nav's agents on the dynamic single-track
// vehicle as their live plant. Its steps are dyn_plant's, its energy books close, and plant_host
// puts a sim_world's agents on it by name, with or without Jolt in the build.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/core/exception.h>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/sim_world.h>
#include <cvc/physics/dyn_bike_plant.h>
#include <cvc/physics/plant_host.h>
#include <gtest/gtest.h>
#include <string>
#include <vector>

using namespace cvc::physics;
using cvc::nav::plant_kind;
using cvc::nav::sim_world;
namespace diff = cvc::nav::diff;

namespace {

constexpr double kS = 0.05; // the trained metric: 1 normalized unit = 20 m

cvc::world_units::normalized_frame trained_frame() {
  cvc::world_units::normalized_frame f;
  f.scale = kS;
  return f;
}

// Three agents on an open 200 m square, in nav's trained metric, heading east to goals 120 m on.
sim_world open_sim() {
  constexpr int R = 101, C = 101;
  static const std::vector<std::uint8_t> occ(static_cast<std::size_t>(R) * C, 0);
  sim_world::config c;
  c.rows = R;
  c.cols = C;
  c.min_x = c.min_y = -100.0;
  c.max_x = c.max_y = 100.0;
  c.scale = kS;
  c.veh.rr = 0.15f;
  c.veh.d_hat = 0.35f;
  c.veh.dt = 0.06f;
  c.veh.nsub = 6;
  c.freeze_sense = true;
  c.reach_tol = static_cast<float>(4.0 * kS);
  const float o[6] = {static_cast<float>(-60 * kS), static_cast<float>(-30 * kS),
                      static_cast<float>(-60 * kS), 0.0f,
                      static_cast<float>(-60 * kS), static_cast<float>(30 * kS)};
  const float g[6] = {static_cast<float>(60 * kS), static_cast<float>(-30 * kS),
                      static_cast<float>(60 * kS), 0.0f,
                      static_cast<float>(60 * kS), static_cast<float>(30 * kS)};
  const float col[9] = {1, 1, 1, 1, 1, 1, 1, 1, 1};
  return sim_world(c, occ.data(), occ.data(), cvc::nav::coef_mlp::default_biased(), o, g, col, 3);
}

float east_of(const sim_world &sim, int i) {
  std::vector<float> p(2 * sim.size());
  sim.snapshot(p.data(), nullptr, nullptr, nullptr, nullptr);
  return p[2 * i];
}

// kinetic now - kinetic at attach - the books: zero to rounding, by construction.
double identity_gap(const dyn_bike_plant &plant, int i) {
  const dyn_bike_books &b = plant.books(i);
  return b.kinetic - plant.kinetic_at_attach(i) -
         (b.drive - b.brake - b.dissipated + b.floor + b.grade + b.limiter + b.residual);
}

} // namespace

// The plant's ticks are dyn_plant's substeps under the held command, in SI, bit for bit; nav
// reads the rear axle, the heading and the speed back through its frame.
TEST(DynBikePlant, StepsAsTheDynamicPlantDoes) {
  dyn_bike_plant plant({vehicle_desc::humvee()});
  cvc::nav::plant_state start;
  start.x = 1.5f;
  start.y = -0.5f;
  start.heading = 0.3f;
  plant.attach(1, &start, trained_frame(), 0.06);
  EXPECT_EQ(plant.substeps(), 6);
  const cvc::nav::plant_state s0 = plant.state(0);
  EXPECT_NEAR(s0.x, start.x, 1e-6);
  EXPECT_NEAR(s0.y, start.y, 1e-6);
  EXPECT_NEAR(s0.heading, start.heading, 1e-6);
  EXPECT_EQ(s0.speed, 0.0f);

  diff::dyn_state<double> ref = plant.body(0);
  for (int t = 0; t < 200; ++t) {
    const float steer = static_cast<float>(0.25 * std::sin(0.05 * t));
    const float accel = static_cast<float>((t < 120 ? 2.0 : -3.0) * kS); // normalized / s^2
    plant.command(0, steer, accel);
    plant.advance();
    const cvc::nav::plant_sample sm = plant.sample(0);
    for (int k = 0; k < plant.substeps(); ++k) {
      diff::dyn_state<double> o;
      diff::dyn_plant<double>(plant.params(0), plant.step_limits(0), ref, sm.steer_cmd,
                              sm.accel_cmd, 0.0, o);
      ref = o;
    }
    const diff::dyn_state<double> &b = plant.body(0);
    for (int j = 0; j < diff::dyn_state<double>::size; ++j)
      ASSERT_EQ(b[j], ref[j]) << "tick " << t << ", state " << j;
  }
  const cvc::nav::plant_state s = plant.state(0);
  const diff::dyn_state<double> &b = plant.body(0);
  EXPECT_NEAR(s.x, b.x * kS, 1e-5);
  EXPECT_NEAR(s.y, b.y * kS, 1e-5);
  EXPECT_NEAR(s.speed, b.vx * kS, 1e-6);
  EXPECT_FLOAT_EQ(s.steer, static_cast<float>(b.delta));
  std::printf("  12 s on the Humvee's dynamic bicycle: %.1f m from the start, at %.2f m/s\n",
              std::hypot(b.x - start.x / kS, b.y - start.y / kS), b.vx);
}

// A weave and a stop on the Humvee and the RoboRacer: the books close to rounding by
// construction, the integrator's residual is a fraction of a percent of the flow, and the drive
// energy nav_stats reads is the last tick's.
TEST(DynBikePlant, TheBooksClose) {
  const vehicle_desc descs[] = {vehicle_desc::humvee(), vehicle_desc::roboracer()};
  const char *names[] = {"Humvee", "RoboRacer"};
  for (int v = 0; v < 2; ++v) {
    dyn_bike_plant plant({descs[v]});
    cvc::nav::plant_state start;
    plant.attach(1, &start, trained_frame(), 0.06);
    double drive_ticks = 0.0;
    for (int t = 0; t < 500; ++t) {
      const double time = 0.06 * t;
      const float steer = static_cast<float>(0.3 * std::sin(2.0 * 3.14159265 * 0.4 * time));
      const float accel =
          static_cast<float>((time < 15.0 ? 2.0 * std::cos(0.5 * time) + 1.0 : -4.0) * kS);
      plant.command(0, steer, accel);
      plant.advance();
      EXPECT_EQ(plant.energy_step(0), plant.last_tick(0).drive);
      drive_ticks += plant.energy_step(0);
      EXPECT_GE(plant.last_tick(0).dissipated, 0.0);
      EXPECT_GE(plant.last_tick(0).brake, 0.0);
    }
    const dyn_bike_books &b = plant.books(0);
    std::printf("  %s, 30 s: drive %.4g J, brake %.4g, dissipated %.4g, floor %.3g, limiter %.3g, "
                "residual %.3g (%.2e of the flow)\n",
                names[v], b.drive, b.brake, b.dissipated, b.floor, b.limiter, b.residual,
                std::fabs(b.residual) / b.flow());
    EXPECT_NEAR(identity_gap(plant, 0), 0.0, 1e-9 * b.flow());
    EXPECT_NEAR(drive_ticks, b.drive, 1e-9 * b.drive);
    EXPECT_GT(b.drive, 0.0);
    EXPECT_GT(b.dissipated, 0.0);
    EXPECT_LT(std::fabs(b.residual) / b.flow(), 5e-3);
  }
}

// "dynamic" puts every agent on the Humvee's dynamic bicycle, which drives to its goal, with no
// physics world and whether or not this build has Jolt; "dynamic:<presets>" cycles presets; an
// unknown preset throws and changes nothing; "kinematic" takes them off again.
TEST(DynBikePlant, PlantHostPutsTheAgentsOnItByName) {
  sim_world sim = open_sim();
  plant_host host;
  EXPECT_TRUE(plant_host::is_dynamic("dynamic"));
  EXPECT_TRUE(plant_host::is_dynamic(" dynamic:roboracer"));
  EXPECT_FALSE(plant_host::is_dynamic("humvee"));
  EXPECT_NO_THROW(plant_host::check("dynamic"));
  EXPECT_NO_THROW(plant_host::check("dynamic:humvee, roboracer"));
  EXPECT_THROW(plant_host::check("dynamic:tank"), cvc::physics_error);

  EXPECT_EQ(host.use(sim, "dynamic"), "dynamic");
  EXPECT_FALSE(host.fallback());
  EXPECT_EQ(sim.plant_type(), plant_kind::dynamic);
  ASSERT_NE(host.dynamic(), nullptr);
  EXPECT_EQ(host.active(), host.dynamic());
  EXPECT_EQ(host.plant(), nullptr);
  EXPECT_EQ(host.physics(), nullptr);
  EXPECT_EQ(host.dynamic()->size(), 3);
  EXPECT_EQ(host.dynamic()->desc(1).mass, vehicle_desc::humvee().mass);

  const float x0 = east_of(sim, 1);
  std::vector<std::uint8_t> reached(3, 0);
  int ticks = 0;
  for (; ticks < 1500; ++ticks) {
    sim.step();
    sim.snapshot(nullptr, nullptr, nullptr, nullptr, reached.data());
    if (reached[0] && reached[1] && reached[2])
      break;
  }
  const double drove = east_of(sim, 1) - x0; // world metres
  std::printf("  on the dynamic Humvee: agent 1 drove %.1f m east; all reached in %.1f s\n", drove,
              ticks * 0.06);
  EXPECT_GT(drove, 100.0);
  EXPECT_TRUE(reached[0] && reached[1] && reached[2]) << "not every agent reached its goal";
  for (int i = 0; i < 3; ++i) {
    const dyn_bike_books &b = host.dynamic()->books(i);
    EXPECT_NEAR(identity_gap(*host.dynamic(), i), 0.0, 1e-9 * b.flow());
    EXPECT_LT(std::fabs(b.residual) / b.flow(), 5e-3) << "agent " << i;
  }

  const dyn_bike_plant *before = host.dynamic();
  EXPECT_THROW(host.use(sim, "dynamic:tank"), cvc::physics_error);
  EXPECT_EQ(host.dynamic(), before);
  EXPECT_EQ(host.driving(), "dynamic");

  EXPECT_EQ(host.use(sim, "dynamic:humvee,roboracer"), "dynamic:humvee,roboracer");
  ASSERT_NE(host.dynamic(), nullptr);
  EXPECT_EQ(host.dynamic()->desc(1).mass, vehicle_desc::roboracer().mass);
  EXPECT_EQ(host.dynamic()->desc(2).mass, vehicle_desc::humvee().mass);
  for (int t = 0; t < 10; ++t)
    sim.step();

  EXPECT_EQ(host.use(sim, "kinematic"), "kinematic");
  EXPECT_EQ(sim.plant_type(), plant_kind::kinematic);
  EXPECT_EQ(host.dynamic(), nullptr);
  EXPECT_EQ(host.active(), nullptr);
}
