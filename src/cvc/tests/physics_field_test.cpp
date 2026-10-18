/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Tests for cvc::physics fields (field.h): cvc::volume data as water, snow and
// wind. The G6 gates of docs/roadmap/WORLD-GEOMETRY-ROADMAP.md, with Jolt:
//   (1) slabs of density 300, 500 and 800 kg/m^3 float in water at drafts of
//       0.3, 0.5 and 0.8 of their height, and a tilted slab rights itself;
//   (2) a box sinks into a medium of modulus k to the Winkler depth m g / (k A);
//   (3) a ball in a uniform wind, without gravity, follows the quadratic-drag
//       curve v(t) = w - 1 / (1/w + k t), k = rho C_d A / (2 m);
//   (4) a field added over a sleeping body wakes it;
//   (5) the same run twice gives the same state hash.
// The refusals run on the null backend, everywhere.

#include <algorithm>
#include <cmath>
#include <cvc/core/app.h>
#include <cvc/core/exception.h>
#include <cvc/physics/world.h>
#include <cvc/volume/volume.h>
#include <gtest/gtest.h>
#include <memory>
#include <vector>

using namespace cvc::physics;

namespace {

constexpr double kG = 9.80665;

bool have_jolt() { return backend_available(backend::jolt); }

#define REQUIRE_JOLT()                                                                             \
  if (!have_jolt())                                                                                \
  GTEST_SKIP() << "this libcvc was built without Jolt"

world_config jolt_config() {
  world_config c;
  c.backend = backend::jolt;
  return c;
}

cvc::app &ctx() {
  static cvc::app app;
  return app;
}

// A uniform volume over the box [lo, hi].
std::shared_ptr<cvc::volume> uniform(double value, vec3 lo, vec3 hi) {
  auto v = std::make_shared<cvc::volume>(ctx(), cvc::dimension(2, 2, 2), cvc::Float,
                                         cvc::bounding_box(lo.x, lo.y, lo.z, hi.x, hi.y, hi.z));
  v->fill(value);
  return v;
}

// Water up to z = surface, 10 m across, damped about right for a metre-sized body
// (field.h).
field water(double surface = 0.0, double depth = 5.0, double damping = 1.0e4) {
  field f;
  f.kind = field_kind::fluid;
  f.volume = uniform(1000.0, {-5, -5, surface - depth}, {5, 5, surface});
  f.damping = damping;
  return f;
}

body_desc slab(double density, double z) {
  body_desc d;
  d.shape = box{{0.5, 0.5, 0.2}};
  d.density = density;
  d.position = {0.0, 0.0, z};
  return d;
}

// The angle between a body's z axis and the world's.
double tilt(const quat &q) { return std::acos(std::min(1.0, 1.0 - 2.0 * (q.x * q.x + q.y * q.y))); }

} // namespace

// --- refusals (null backend) -------------------------------------------------

TEST(PhysicsField, MalformedFieldsAreRefused) {
  world_config c;
  c.backend = backend::null;
  world w(c);
  field none;
  EXPECT_THROW(w.add_field(none), cvc::physics_error);

  field flat = water();
  flat.volume = uniform(1000.0, {-1, -1, 0}, {1, 1, 0});
  EXPECT_THROW(w.add_field(flat), cvc::physics_error);

  field negative = water();
  negative.volume = uniform(-1.0, {-1, -1, -1}, {1, 1, 0});
  EXPECT_THROW(w.add_field(negative), cvc::physics_error);

  field two = water();
  two.velocity[0] = uniform(1.0, {-1, -1, -1}, {1, 1, 0});
  two.velocity[1] = two.velocity[0];
  EXPECT_THROW(w.add_field(two), cvc::physics_error);

  field drag = water();
  drag.drag_coefficient = -1.0;
  EXPECT_THROW(w.add_field(drag), cvc::physics_error);

  field damping = water();
  damping.kind = field_kind::medium;
  damping.damping = std::nan("");
  EXPECT_THROW(w.add_field(damping), cvc::physics_error);

  EXPECT_EQ(w.field_count(), 0u);
  const field_id id = w.add_field(water());
  EXPECT_EQ(w.field_count(), 1u);
  EXPECT_THROW(w.set_field(id, none), cvc::physics_error);
  w.remove_field(id);
  EXPECT_EQ(w.field_count(), 0u);
  EXPECT_THROW(w.remove_field(id), cvc::physics_error);
  EXPECT_THROW(w.set_field(id, water()), cvc::physics_error);
  // Ids are never reused.
  EXPECT_NE(w.add_field(water()), id);
}

// --- fluids --------------------------------------------------------------------

// The first step's load is Archimedes': rho g times the volume below the surface.
TEST(PhysicsField, BuoyancyIsTheDisplacedWeight) {
  REQUIRE_JOLT();
  world w(jolt_config());
  w.add_field(water());
  // Centre 0.1 m below the surface: 0.3 of the slab's 0.4 m is under water.
  const body_id b = w.add_body(slab(500.0, -0.1));
  // The engine holds the position in float: -0.1 to 1.5e-9.
  const double draft = 0.0 - (w.state(b).position.z - 0.2);
  EXPECT_NEAR(draft, 0.3, 1e-8);
  w.step();
  const field_load l = w.field_load_on(b);
  EXPECT_NEAR(l.immersed_volume, draft, 1e-12);
  EXPECT_NEAR(l.force.z, 1000.0 * kG * draft, 1e-9);
  EXPECT_NEAR(l.force.x, 0.0, 1e-9);
  EXPECT_NEAR(l.force.y, 0.0, 1e-9);
  // Upright and centred: buoyancy acts through the centre of mass's vertical.
  EXPECT_NEAR(l.torque.x, 0.0, 1e-9);
  EXPECT_NEAR(l.torque.y, 0.0, 1e-9);
}

// Gate (1): each slab, dropped in half sunk, settles where its weight is the
// displaced water's.
TEST(PhysicsField, SlabsFloatAtTheirDensityRatio) {
  REQUIRE_JOLT();
  for (const double density : {300.0, 500.0, 800.0}) {
    world w(jolt_config());
    w.add_field(water());
    const body_id b = w.add_body(slab(density, 0.0));
    w.step(600);
    const body_state s = w.state(b);
    EXPECT_NEAR(0.0 - (s.position.z - 0.2), 0.4 * density / 1000.0, 0.002) << "density " << density;
    EXPECT_LT(std::fabs(s.linear_velocity.z), 0.01) << "density " << density;
    EXPECT_LT(tilt(s.rotation), 0.01) << "density " << density;
  }
}

// A tilted slab rights itself: the buoyancy torque, applied at the cells, has the
// right sign and lever.
TEST(PhysicsField, ATiltedSlabRightsItself) {
  REQUIRE_JOLT();
  world w(jolt_config());
  w.add_field(water());
  body_desc d = slab(500.0, 0.0);
  const double a = 20.0 * 3.14159265358979 / 180.0;
  d.rotation = {std::cos(a / 2), std::sin(a / 2), 0.0, 0.0};
  const body_id b = w.add_body(d);
  w.step(800);
  EXPECT_LT(tilt(w.state(b).rotation), 5.0 * 3.14159265358979 / 180.0);
}

// --- media -----------------------------------------------------------------------

// Gate (2): a 0.4 m box of 10 kg on a medium of modulus k sinks to m g / (k A).
TEST(PhysicsField, ABoxSinksToTheWinklerDepth) {
  REQUIRE_JOLT();
  world w(jolt_config());
  const double k = 2.0e4;
  field snow;
  snow.kind = field_kind::medium;
  snow.volume = uniform(k, {-5, -5, -1}, {5, 5, 0});
  snow.damping = 5.0e4;
  w.add_field(snow);
  body_desc d;
  d.shape = box{{0.2, 0.2, 0.2}};
  d.mass = 10.0;
  d.position = {0.0, 0.0, 0.2};
  const body_id b = w.add_body(d);
  w.step(300);
  const double sink = 0.0 - (w.state(b).position.z - 0.2);
  EXPECT_NEAR(sink, 10.0 * kG / (k * 0.16), 0.001);
  EXPECT_LT(std::fabs(w.state(b).linear_velocity.z), 0.01);
}

// --- flows -----------------------------------------------------------------------

// Gate (3): a 1 kg ball of radius 0.5 m starts at rest in a 10 m/s wind, with no
// gravity, and follows the quadratic-drag curve.
TEST(PhysicsField, WindCarriesABallAlongTheDragCurve) {
  REQUIRE_JOLT();
  world_config c = jolt_config();
  c.gravity = {0.0, 0.0, 0.0};
  world w(c);
  const double rho = 1.2, cd = 0.5, wind = 10.0;
  field air;
  air.kind = field_kind::flow;
  air.volume = uniform(rho, {-100, -100, -100}, {100, 100, 100});
  air.velocity = {uniform(wind, {-100, -100, -100}, {100, 100, 100}),
                  uniform(0.0, {-100, -100, -100}, {100, 100, 100}),
                  uniform(0.0, {-100, -100, -100}, {100, 100, 100})};
  air.drag_coefficient = cd;
  w.add_field(air);
  body_desc d;
  d.shape = sphere{0.5};
  d.mass = 1.0;
  d.linear_damping = 0.0;
  d.angular_damping = 0.0;
  d.allow_sleeping = false;
  const body_id b = w.add_body(d);
  const double k = 0.5 * rho * cd * 3.14159265358979 * 0.25 / 1.0;
  for (const int steps : {50, 100, 200}) {
    w.step(static_cast<std::uint64_t>(steps - static_cast<int>(w.tick())));
    const double t = w.time();
    const double expected = wind - 1.0 / (1.0 / wind + k * t);
    const body_state s = w.state(b);
    EXPECT_NEAR(s.linear_velocity.x, expected, 0.01 * expected) << "t = " << t;
    EXPECT_NEAR(s.linear_velocity.y, 0.0, 1e-6);
    EXPECT_NEAR(s.linear_velocity.z, 0.0, 1e-6);
  }
}

// --- sleep, hulls, determinism ------------------------------------------------------

// Gate (4): a slab asleep on the ground floats up once water is added over it.
TEST(PhysicsField, AFieldAddedLaterWakesTheBodiesInIt) {
  REQUIRE_JOLT();
  world w(jolt_config());
  body_desc ground;
  ground.shape = box{{5.0, 5.0, 0.5}};
  ground.motion = motion_type::static_;
  ground.position = {0.0, 0.0, -0.5};
  w.add_body(ground);
  const body_id b = w.add_body(slab(300.0, 0.2));
  w.step(300);
  ASSERT_FALSE(w.state(b).active) << "the slab should be asleep on the ground";
  w.add_field(water(1.0, 2.0));
  EXPECT_TRUE(w.state(b).active);
  w.step(400);
  // Floating at a draft of 0.12 m below a surface at z = 1: centre near 1.08.
  EXPECT_GT(w.state(b).position.z, 0.9);
}

// A convex hull is sampled through the engine: a unit cube hull with its origin at
// a corner has all its volume immersed and no torque about its centre of mass.
TEST(PhysicsField, HullsAreSampledByTheEngine) {
  REQUIRE_JOLT();
  world w(jolt_config());
  w.add_field(water());
  body_desc d;
  convex_hull h;
  for (int i = 0; i < 8; ++i)
    h.points.push_back({double(i & 1), double((i >> 1) & 1), double((i >> 2) & 1)});
  d.shape = h;
  d.density = 500.0;
  d.position = {0.0, 0.0, -2.0};
  const body_id b = w.add_body(d);
  w.step();
  const field_load l = w.field_load_on(b);
  EXPECT_NEAR(l.immersed_volume, 1.0, 1e-3);
  EXPECT_NEAR(l.force.z, 1000.0 * kG * l.immersed_volume, 1e-6);
  EXPECT_NEAR(l.torque.x, 0.0, 1e-6);
  EXPECT_NEAR(l.torque.y, 0.0, 1e-6);
}

// Static and kinematic bodies, and bodies outside every field, feel nothing.
TEST(PhysicsField, OnlyDynamicBodiesInAFieldArePushed) {
  REQUIRE_JOLT();
  world w(jolt_config());
  w.add_field(water());
  body_desc kin = slab(500.0, -1.0);
  kin.motion = motion_type::kinematic;
  const body_id k = w.add_body(kin);
  body_desc dry = slab(500.0, 10.0);
  const body_id out = w.add_body(dry);
  w.step();
  EXPECT_EQ(w.field_load_on(k).immersed_volume, 0.0);
  EXPECT_EQ(w.field_load_on(out).immersed_volume, 0.0);
  EXPECT_EQ(w.field_load_on(out).force.z, 0.0);
}

// Gate (5): fields keep a run a function of its inputs.
TEST(PhysicsField, TheSameRunTwiceGivesTheSameHash) {
  REQUIRE_JOLT();
  auto run = [](bool with_fields) {
    world w(jolt_config());
    if (with_fields) {
      w.add_field(water());
      field breeze;
      breeze.kind = field_kind::flow;
      breeze.volume = uniform(1.2, {-5, -5, 0}, {5, 5, 5});
      breeze.velocity = {uniform(3.0, {-5, -5, 0}, {5, 5, 5}), uniform(1.0, {-5, -5, 0}, {5, 5, 5}),
                         uniform(0.0, {-5, -5, 0}, {5, 5, 5})};
      w.add_field(breeze);
    }
    for (int i = 0; i < 4; ++i) {
      body_desc d = slab(300.0 + 150.0 * i, 0.5 + 0.6 * i);
      d.position.x = -2.0 + 1.3 * i;
      const double a = 0.2 * i;
      d.rotation = {std::cos(a), 0.0, std::sin(a), 0.0};
      w.add_body(d);
      body_desc ball;
      ball.shape = sphere{0.2};
      ball.density = 200.0 + 300.0 * i;
      ball.position = {1.0 - 1.1 * i, 1.5, 1.0 + 0.3 * i};
      w.add_body(ball);
    }
    w.step(300);
    return w.state_hash();
  };
  const std::uint64_t a = run(true);
  EXPECT_EQ(a, run(true));
  EXPECT_NE(a, run(false));
}
