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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/core/exception.h>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/sim_world.h>
#include <cvc/physics/dyn_bike_plant.h>
#include <cvc/physics/plant_host.h>
#include <gtest/gtest.h>
#include <memory>
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
         (b.drive - b.brake - b.dissipated + b.floor + b.grade + b.limiter + b.contact +
          b.residual);
}

// A ramp rising `slope` per metre eastward.
nav_plant_options ramp(double slope) {
  nav_plant_options o;
  o.ground_height = [slope](double x, double) { return slope * x; };
  return o;
}

// A world with a flat ground at z = 0 and a wall across the road at x = wall_x (its west face),
// for the chassis to hit. `ground` gets the ground's body.
std::unique_ptr<world> walled_world(double wall_x, body_id &ground) {
  auto w = std::make_unique<world>();
  body_desc g;
  g.shape = box{{500.0, 500.0, 0.5}};
  g.motion = motion_type::static_;
  g.position = {0.0, 0.0, -0.5};
  ground = w->add_body(g);
  body_desc wall;
  wall.shape = box{{0.5, 20.0, 3.0}};
  wall.motion = motion_type::static_;
  wall.position = {wall_x + 0.5, 0.0, 3.0};
  w->add_body(wall);
  return w;
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

// Up a 5% ramp the vehicle loses g sin(slope) of acceleration against the same commands on flat
// ground, the books gain the potential energy it climbed as their grade line, and they close.
TEST(DynBikePlant, ClimbsTheGround) {
  const double slope = 0.05;
  dyn_bike_plant_options flat_o, ramp_o;
  ramp_o.plant = ramp(slope);
  dyn_bike_plant flat({vehicle_desc::humvee()}, flat_o), up({vehicle_desc::humvee()}, ramp_o);
  cvc::nav::plant_state start; // at the origin, heading east: uphill
  flat.attach(1, &start, trained_frame(), 0.06);
  up.attach(1, &start, trained_frame(), 0.06);
  const int ticks = 100; // 6 s
  for (int t = 0; t < ticks; ++t) {
    const float accel = static_cast<float>(1.5 * kS);
    flat.command(0, 0.0f, accel);
    up.command(0, 0.0f, accel);
    flat.advance();
    up.advance();
  }
  const double lost = flat.body(0).vx - up.body(0).vx;
  const double expect = 9.80665 * slope / std::sqrt(1.0 + slope * slope) * ticks * 0.06;
  const dyn_bike_books &b = up.books(0);
  const double climbed = slope * up.body(0).x; // m
  const double m = vehicle_desc::humvee().mass;
  std::printf("  6 s up a 5%% ramp: %.3f m/s slower than on the flat (g sin(slope) t = %.3f); "
              "grade %.4g J against m g dz = %.4g J\n",
              lost, expect, b.grade, -m * 9.80665 * climbed);
  EXPECT_NEAR(lost, expect, 0.03 * expect);
  EXPECT_NEAR(b.grade, -m * 9.80665 * climbed, 0.02 * m * 9.80665 * climbed);
  EXPECT_NEAR(identity_gap(up, 0), 0.0, 1e-9 * b.flow());
  EXPECT_EQ(flat.books(0).grade, 0.0);
  EXPECT_FALSE(up.collides());
}

// In a world, the chassis meets a wall across its road: it stops at the wall's face instead of
// driving through it as it does without the world, the collisions take energy out on the books'
// contact line, and the books still close.
TEST(DynBikePlant, CollidesWithTheWorld) {
  if (!plant_host::available())
    GTEST_SKIP() << "no Jolt in this build";
  const double wall_x = 30.0;
  body_id ground;
  auto w = walled_world(wall_x, ground);
  dyn_bike_plant_options o;
  o.physics = w.get();
  o.ground_bodies = {ground};
  dyn_bike_plant walled({vehicle_desc::humvee()}, o), open({vehicle_desc::humvee()});
  cvc::nav::plant_state start;
  walled.attach(1, &start, trained_frame(), 0.06);
  open.attach(1, &start, trained_frame(), 0.06);
  EXPECT_TRUE(walled.collides());
  EXPECT_TRUE(walled.chassis(0).valid());
  const vehicle_desc d = vehicle_desc::humvee();
  const double front = d.half_extents.x - [&] {
    double rear = 0.0;
    for (const axle_desc &a : d.axles)
      if (!a.steered)
        rear = std::min(rear, a.x);
    return rear;
  }(); // the box's nose ahead of the rear axle
  for (int t = 0; t < 200; ++t) { // 12 s at 2 m/s^2: well into the wall
    walled.command(0, 0.0f, static_cast<float>(2.0 * kS));
    open.command(0, 0.0f, static_cast<float>(2.0 * kS));
    walled.advance();
    open.advance();
  }
  const dyn_bike_books &b = walled.books(0);
  const double nose = walled.body(0).x + front;
  std::printf("  into a wall at %.0f m: the nose stops at %.3f m (%ld colliding substeps, contact "
              "%.4g J, drive %.4g J); without the world it reaches %.1f m\n",
              wall_x, nose, walled.collisions(0), b.contact, b.drive, open.body(0).x + front);
  EXPECT_GT(open.body(0).x + front, wall_x + 10.0);
  EXPECT_LT(nose, wall_x + 0.05);
  EXPECT_GT(nose, wall_x - 1.0);
  EXPECT_GT(walled.collisions(0), 0);
  EXPECT_LT(b.contact, 0.0);
  EXPECT_LT(std::fabs(walled.body(0).vx), 0.5);
  EXPECT_NEAR(identity_gap(walled, 0), 0.0, 1e-9 * b.flow());
}

// "dynamic" puts every agent on the Humvee's dynamic bicycle, which drives to its goal, in a
// physics world when the build has Jolt and without one otherwise; "dynamic:<presets>" cycles
// presets; an unknown preset throws and changes nothing; "kinematic" takes them off again.
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
  EXPECT_EQ(host.physics() != nullptr, plant_host::available());
  EXPECT_EQ(host.dynamic()->collides(), plant_host::available());
  EXPECT_EQ(host.dynamic4w(), nullptr);
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
