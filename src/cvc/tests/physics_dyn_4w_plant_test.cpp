/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics::dyn_4w_plant (dyn_4w_plant.h): nav's agents on the dynamic four-wheel vehicle
// with suspension as their live plant. Its steps are dyn4_plant's, its energy books close, it
// climbs the ground it is given, its chassis collide in a world, and plant_host puts a
// sim_world's agents on it by name.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/core/exception.h>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/sim_world.h>
#include <cvc/physics/dyn_4w_plant.h>
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
constexpr double kG = 9.80665;

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

// stored now - stored at attach - the books: zero to rounding, by construction.
double identity_gap(const dyn_4w_plant &plant, int i) {
  const dyn_4w_books &b = plant.books(i);
  return b.stored - plant.stored_at_attach(i) -
         (b.drive - b.brake - b.dissipated + b.lever + b.grade + b.ground + b.limiter + b.contact +
          b.residual);
}

nav_plant_options ramp(double slope) {
  nav_plant_options o;
  o.ground_height = [slope](double x, double) { return slope * x; };
  return o;
}

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

double nose_ahead_of_rear_axle(const vehicle_desc &d) {
  double rear = 0.0;
  for (const axle_desc &a : d.axles)
    if (!a.steered)
      rear = std::min(rear, a.x);
  return d.half_extents.x - rear;
}

} // namespace

// The plant's ticks are dyn4_plant's substeps under the held command, in SI, on its ground
// sampler, bit for bit; it starts at rest on the ground.
TEST(Dyn4wPlant, StepsAsTheFourWheelPlantDoes) {
  dyn_4w_plant plant({vehicle_desc::humvee()});
  cvc::nav::plant_state start;
  start.x = 1.5f;
  start.y = -0.5f;
  start.heading = 0.3f;
  plant.attach(1, &start, trained_frame(), 0.06);
  EXPECT_EQ(plant.substeps(), 6);
  EXPECT_EQ(plant.body(0).z, 0.0);
  EXPECT_EQ(plant.body(0).phi, 0.0);
  EXPECT_EQ(plant.body(0).th, 0.0);
  const dyn_4w_plant::ground_sampler ground{&plant};
  diff::dyn4_state<double> ref = plant.body(0);
  for (int t = 0; t < 200; ++t) {
    const float steer = static_cast<float>(0.25 * std::sin(0.05 * t));
    const float accel = static_cast<float>((t < 120 ? 2.0 : -3.0) * kS);
    plant.command(0, steer, accel);
    plant.advance();
    const cvc::nav::plant_sample sm = plant.sample(0);
    for (int k = 0; k < plant.substeps(); ++k) {
      diff::dyn4_state<double> o;
      diff::dyn4_plant<double>(plant.params(0), plant.step_limits(0), ground, ref, sm.steer_cmd,
                               sm.accel_cmd, o);
      ref = o;
    }
    const diff::dyn4_state<double> &b = plant.body(0);
    for (int j = 0; j < diff::dyn4_state<double>::size; ++j)
      ASSERT_EQ(b[j], ref[j]) << "tick " << t << ", state " << j;
  }
  const cvc::nav::plant_state s = plant.state(0);
  EXPECT_NEAR(s.x, plant.body(0).x * kS, 1e-5);
  EXPECT_NEAR(s.speed, plant.body(0).vx * kS, 1e-6);
}

// A weave and a stop: the books close by construction, and the integrator's residual is a
// fraction of a percent of the flow.
TEST(Dyn4wPlant, TheBooksClose) {
  dyn_4w_plant plant({vehicle_desc::humvee()});
  cvc::nav::plant_state start;
  plant.attach(1, &start, trained_frame(), 0.06);
  for (int t = 0; t < 500; ++t) {
    const double time = 0.06 * t;
    const float steer = static_cast<float>(0.3 * std::sin(2.0 * 3.14159265 * 0.4 * time));
    const float accel =
        static_cast<float>((time < 15.0 ? 2.0 * std::cos(0.5 * time) + 1.0 : -4.0) * kS);
    plant.command(0, steer, accel);
    plant.advance();
    EXPECT_EQ(plant.energy_step(0), plant.last_tick(0).drive);
    EXPECT_GE(plant.last_tick(0).dissipated, 0.0);
  }
  const dyn_4w_books &b = plant.books(0);
  std::printf("  30 s on the four-wheel Humvee: drive %.4g J, brake %.4g, dissipated %.4g, lever "
              "%.3g, limiter %.3g, residual %.3g (%.2e of the flow)\n",
              b.drive, b.brake, b.dissipated, b.lever, b.limiter, b.residual,
              std::fabs(b.residual) / b.flow());
  EXPECT_NEAR(identity_gap(plant, 0), 0.0, 1e-9 * b.flow());
  EXPECT_GT(b.drive, 0.0);
  EXPECT_LT(std::fabs(b.residual) / b.flow(), 5e-3);
}

// Up a 5% ramp, with gravity along the slope (the plant's default) the vehicle loses g s of
// acceleration against the flat, and a little more: dyn4_plant's dampers act on the body's own
// vertical rate (skyhook), so a steady climb also pays c zdot^2 at each corner. Without the grade
// force the springs only lift it and it climbs for free.
TEST(Dyn4wPlant, ClimbsTheGround) {
  const double slope = 0.05;
  dyn_4w_plant_options flat_o, ramp_o, free_o;
  ramp_o.plant = free_o.plant = ramp(slope);
  free_o.grade_force = false;
  dyn_4w_plant flat({vehicle_desc::humvee()}, flat_o), up({vehicle_desc::humvee()}, ramp_o),
      free_up({vehicle_desc::humvee()}, free_o);
  cvc::nav::plant_state start;
  flat.attach(1, &start, trained_frame(), 0.06);
  up.attach(1, &start, trained_frame(), 0.06);
  free_up.attach(1, &start, trained_frame(), 0.06);
  EXPECT_LT(up.body(0).th, 0.0); // nose up the ramp
  const int ticks = 100;
  for (int t = 0; t < ticks; ++t) {
    for (dyn_4w_plant *p : {&flat, &up, &free_up}) {
      p->command(0, 0.0f, static_cast<float>(1.5 * kS));
      p->advance();
    }
  }
  const double lost = flat.body(0).vx - up.body(0).vx,
               lost_free = flat.body(0).vx - free_up.body(0).vx;
  const double expect = kG * slope * ticks * 0.06;
  const dyn_4w_books &b = up.books(0);
  std::printf(
      "  6 s up a 5%% ramp on four wheels: %.3f m/s slower than on the flat (g s t = %.3f); "
      "without the grade force %.3f; grade %.4g J, ground %.4g J, residual %.2e of the "
      "flow\n",
      lost, expect, lost_free, b.grade, b.ground, std::fabs(b.residual) / b.flow());
  EXPECT_GT(lost, 0.98 * expect);
  EXPECT_LT(lost, 1.15 * expect);
  EXPECT_LT(std::fabs(lost_free), 0.1 * expect);
  EXPECT_LT(b.grade, 0.0);
  EXPECT_NEAR(identity_gap(up, 0), 0.0, 1e-9 * b.flow());
  EXPECT_LT(std::fabs(b.residual) / b.flow(), 5e-3);
}

// In a world the chassis stops at a wall's face, where without one it drives through.
TEST(Dyn4wPlant, CollidesWithTheWorld) {
  if (!plant_host::available())
    GTEST_SKIP() << "no Jolt in this build";
  const double wall_x = 30.0;
  body_id ground;
  auto w = walled_world(wall_x, ground);
  dyn_4w_plant_options o;
  o.physics = w.get();
  o.ground_bodies = {ground};
  dyn_4w_plant walled({vehicle_desc::humvee()}, o), open({vehicle_desc::humvee()});
  cvc::nav::plant_state start;
  walled.attach(1, &start, trained_frame(), 0.06);
  open.attach(1, &start, trained_frame(), 0.06);
  EXPECT_TRUE(walled.collides());
  const double front = nose_ahead_of_rear_axle(vehicle_desc::humvee());
  for (int t = 0; t < 200; ++t) {
    walled.command(0, 0.0f, static_cast<float>(2.0 * kS));
    open.command(0, 0.0f, static_cast<float>(2.0 * kS));
    walled.advance();
    open.advance();
  }
  const dyn_4w_books &b = walled.books(0);
  const double nose = walled.body(0).x + front;
  std::printf("  four wheels into a wall at %.0f m: the nose stops at %.3f m (%ld colliding "
              "substeps, contact %.4g J); without the world it reaches %.1f m\n",
              wall_x, nose, walled.collisions(0), b.contact, open.body(0).x + front);
  EXPECT_GT(open.body(0).x + front, wall_x + 10.0);
  EXPECT_LT(nose, wall_x + 0.05);
  EXPECT_GT(nose, wall_x - 1.0);
  EXPECT_GT(walled.collisions(0), 0);
  EXPECT_LT(b.contact, 0.0);
  EXPECT_NEAR(identity_gap(walled, 0), 0.0, 1e-9 * b.flow());
}

// "dynamic4w" puts every agent on the Humvee's four-wheel model, which drives to its goal;
// "dynamic4w:<presets>" cycles presets; an unknown one throws and changes nothing.
TEST(Dyn4wPlant, PlantHostPutsTheAgentsOnItByName) {
  sim_world sim = open_sim();
  plant_host host;
  EXPECT_TRUE(plant_host::is_dynamic4w("dynamic4w"));
  EXPECT_TRUE(plant_host::is_dynamic4w(" dynamic4w:roboracer"));
  EXPECT_FALSE(plant_host::is_dynamic4w("dynamic"));
  EXPECT_FALSE(plant_host::is_dynamic("dynamic4w"));
  EXPECT_NO_THROW(plant_host::check("dynamic4w:humvee, roboracer"));
  EXPECT_THROW(plant_host::check("dynamic4w:tank"), cvc::physics_error);

  EXPECT_EQ(host.use(sim, "dynamic4w"), "dynamic4w");
  EXPECT_EQ(sim.plant_type(), plant_kind::dynamic);
  ASSERT_NE(host.dynamic4w(), nullptr);
  EXPECT_EQ(host.dynamic(), nullptr);
  EXPECT_EQ(host.active(), host.dynamic4w());
  EXPECT_EQ(host.physics() != nullptr, plant_host::available());
  EXPECT_EQ(host.dynamic4w()->collides(), plant_host::available());
  EXPECT_EQ(host.dynamic4w()->size(), 3);

  std::vector<std::uint8_t> reached(3, 0);
  int ticks = 0;
  for (; ticks < 1500; ++ticks) {
    sim.step();
    sim.snapshot(nullptr, nullptr, nullptr, nullptr, reached.data());
    if (reached[0] && reached[1] && reached[2])
      break;
  }
  std::printf("  on the four-wheel Humvee: all reached in %.1f s\n", ticks * 0.06);
  EXPECT_TRUE(reached[0] && reached[1] && reached[2]) << "not every agent reached its goal";
  for (int i = 0; i < 3; ++i) {
    const dyn_4w_books &b = host.dynamic4w()->books(i);
    EXPECT_NEAR(identity_gap(*host.dynamic4w(), i), 0.0, 1e-9 * b.flow());
    EXPECT_LT(std::fabs(b.residual) / b.flow(), 5e-3) << "agent " << i;
    EXPECT_EQ(host.dynamic4w()->collisions(i), 0) << "agent " << i;
  }

  const dyn_4w_plant *before = host.dynamic4w();
  EXPECT_THROW(host.use(sim, "dynamic4w:tank"), cvc::physics_error);
  EXPECT_EQ(host.dynamic4w(), before);

  EXPECT_EQ(host.use(sim, "dynamic"), "dynamic");
  EXPECT_EQ(host.dynamic4w(), nullptr);
  ASSERT_NE(host.dynamic(), nullptr);
  EXPECT_EQ(host.use(sim, "kinematic"), "kinematic");
  EXPECT_EQ(host.active(), nullptr);
  EXPECT_EQ(host.physics(), nullptr);
}
