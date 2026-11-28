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
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/core/exception.h>
#include <cvc/core/world_clock.h>
#include <cvc/ocean/h0.h>
#include <cvc/ocean/sea_surface.h>
#include <cvc/physics/dyn_boat_plant.h> // the tactical hull
#include <cvc/physics/ocean_surface.h>
#include <cvc/physics/shore_mask.h>
#include <cvc/physics/world.h>
#include <cvc/volume/volume.h>
#include <functional>
#include <gtest/gtest.h>
#include <memory>
#include <utility>
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

// A body that displaces only a share of its volume (body_desc::displacement; an avatar's box,
// OW9b) floats deeper by that share: a 300 kg/m^3 slab displacing half its volume floats as a
// 600 kg/m^3 one would, and as itself again once set_displacement gives it the whole. A share
// that is not finite and >= 0 is refused, as is an unknown body.
TEST(PhysicsField, ADisplacementShareFloatsABodyDeeper) {
  {
    world w{world_config{}};
    body_desc bad = slab(300.0, 0.0);
    bad.displacement = -0.5;
    EXPECT_THROW(w.add_body(bad), cvc::physics_error);
    bad.displacement = std::nan("");
    EXPECT_THROW(w.add_body(bad), cvc::physics_error);
    body_id nobody;
    nobody.value = 12345;
    EXPECT_THROW(w.set_displacement(nobody, 0.5), cvc::physics_error);
  }
  REQUIRE_JOLT();
  world w(jolt_config());
  w.add_field(water());
  body_desc d = slab(300.0, 0.0);
  d.displacement = 0.5;
  const body_id b = w.add_body(d);
  EXPECT_THROW(w.set_displacement(b, -1.0), cvc::physics_error);
  w.step(600);
  const double half = 0.0 - (w.state(b).position.z - 0.2);
  EXPECT_NEAR(half, 0.4 * 300.0 / 1000.0 / 0.5, 0.002) << "half the share, twice the draft";
  w.set_displacement(b, 1.0);
  w.step(600);
  const double whole = 0.0 - (w.state(b).position.z - 0.2);
  EXPECT_NEAR(whole, 0.4 * 300.0 / 1000.0, 0.002) << "the whole share, its own draft";
  EXPECT_LT(tilt(w.state(b).rotation), 0.01);
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

// --- free surfaces (field::surface, fluid_surface.h; OCEAN-AND-WEATHER-ROADMAP.md B5, B6) -------

namespace {

// Water 10 m across from z = -5 up to a box top of z = 1, its top a free surface.
field sea(std::shared_ptr<const fluid_surface> surface, double damping = 1.0e4) {
  field f;
  f.kind = field_kind::fluid;
  f.volume = uniform(1000.0, {-5, -5, -5}, {5, 5, 1});
  f.damping = damping;
  f.surface = std::move(surface);
  return f;
}

std::shared_ptr<wave_train_surface> swell(double amplitude, double period, double depth = 0.0) {
  wave_train t;
  t.amplitude = amplitude;
  t.period_s = period;
  t.dir_rad = 0.3;
  t.phase = 0.7;
  return std::make_shared<wave_train_surface>(0.0, depth, std::vector<wave_train>{t});
}

body_desc ball_at(double x, double y, double z) {
  body_desc ball;
  ball.shape = sphere{0.25};
  ball.density = 300.0;
  ball.position = {x, y, z};
  return ball;
}

} // namespace

TEST(PhysicsFluidSurface, StillWaterIsFlatAndCarriesItsCurrent) {
  const still_surface s(1.25, {0.5, -0.25, 0.0});
  EXPECT_EQ(s.level(3.0, -4.0), 1.25);
  EXPECT_EQ(s.height(3.0, -4.0, 17.0), 1.25);
  const vec3 v = s.velocity({1.0, 2.0, 0.0}, 5.0);
  EXPECT_EQ(v.x, 0.5);
  EXPECT_EQ(v.y, -0.25);
  EXPECT_EQ(v.z, 0.0);
  EXPECT_NE(s.digest(), still_surface(1.25).digest());
  EXPECT_NE(s.digest(), still_surface(1.0, {0.5, -0.25, 0.0}).digest());
  EXPECT_EQ(s.digest(), still_surface(1.25, {0.5, -0.25, 0.0}).digest());
  EXPECT_THROW(still_surface(std::nan("")), std::invalid_argument);
}

TEST(PhysicsFluidSurface, AWaveTrainIsAnAiryWave) {
  constexpr double kTau = 6.283185307179586;
  // Deep water: k = omega^2 / g, the speed at the still level A omega, decaying as e^(kz).
  const auto deep = swell(0.4, 6.0);
  const double w = kTau / 6.0, k = w * w / kG;
  EXPECT_NEAR(deep->wavenumber(0), k, 1e-12);
  EXPECT_NEAR(deep->omega(0), w, 1e-15);
  double hi = -1e9, lo = 1e9;
  for (int i = 0; i < 600; ++i) {
    const double t = 6.0 * i / 600.0;
    const double h = deep->height(2.0, -1.0, t);
    hi = std::max(hi, h);
    lo = std::min(lo, h);
    // A deep-water particle goes round a circle: its speed is constant.
    const vec3 v = deep->velocity({2.0, -1.0, 0.0}, t);
    EXPECT_NEAR(std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z), 0.4 * w, 1e-9);
  }
  EXPECT_NEAR(hi, 0.4, 1e-3);
  EXPECT_NEAR(lo, -0.4, 1e-3);
  const vec3 below = deep->velocity({2.0, -1.0, -3.0}, 1.0);
  EXPECT_NEAR(std::sqrt(below.x * below.x + below.y * below.y + below.z * below.z),
              0.4 * w * std::exp(-3.0 * k), 1e-9);
  // Above the still level, in a crest, the level's speed (linear theory does not reach higher).
  const vec3 a = deep->velocity({2.0, -1.0, 0.3}, 1.0), b = deep->velocity({2.0, -1.0, 0.0}, 1.0);
  EXPECT_EQ(a.x, b.x);
  EXPECT_EQ(a.z, b.z);
  // The horizontal motion is along the train's direction.
  EXPECT_NEAR(b.y / b.x, std::tan(0.3), 1e-9);
  // Finite depth: the dispersion relation holds, the wave is shorter than in deep water, and
  // the bottom does not move up or down.
  const auto shallow = swell(0.3, 8.0, 6.0);
  const double ks = shallow->wavenumber(0), ws = kTau / 8.0;
  EXPECT_NEAR(kG * ks * std::tanh(ks * 6.0), ws * ws, 1e-12);
  EXPECT_GT(ks, ws * ws / kG);
  EXPECT_NEAR(shallow->velocity({1.0, 1.0, -6.0}, 2.0).z, 0.0, 1e-12);
  EXPECT_EQ(shallow->velocity({1.0, 1.0, -9.0}, 2.0).x, shallow->velocity({1.0, 1.0, -6.0}, 2.0).x);
  // The digest follows every parameter.
  EXPECT_EQ(deep->digest(), swell(0.4, 6.0)->digest());
  EXPECT_NE(deep->digest(), swell(0.41, 6.0)->digest());
  EXPECT_NE(deep->digest(), swell(0.4, 6.1)->digest());
  EXPECT_NE(deep->digest(), swell(0.4, 6.0, 30.0)->digest());
  EXPECT_THROW(wave_train_surface(0.0, 0.0, {wave_train{0.1, 0.0, 0.0, 0.0}}),
               std::invalid_argument);
  EXPECT_THROW(wave_train_surface(0.0, 0.0, {wave_train{-0.1, 5.0, 0.0, 0.0}}),
               std::invalid_argument);
}

TEST(PhysicsFluidSurface, OnlyAFluidHasAFreeSurface) {
  world w{world_config{}};
  field snow;
  snow.kind = field_kind::medium;
  snow.volume = uniform(1e5, {-1, -1, -1}, {1, 1, 0});
  snow.surface = std::make_shared<still_surface>(0.0);
  EXPECT_THROW(w.add_field(snow), cvc::physics_error);
  field wind = snow;
  wind.kind = field_kind::flow;
  EXPECT_THROW(w.add_field(wind), cvc::physics_error);
}

TEST(PhysicsFluidSurface, TheFieldTimeFollowsTheClock) {
  cvc::world_clock::config cc;
  cc.fixed_dt = 0.01;
  cvc::world_clock clock(cc);
  clock.advance_ticks(12345); // the clock has run 123.45 s before the world joins it
  world w{world_config{}};
  w.step(7);
  EXPECT_NEAR(w.field_time(), 0.07, 1e-12) << "without a clock, the world's own time";
  EXPECT_EQ(w.follow(clock), 0u); // anchors
  EXPECT_NEAR(w.field_time(), clock.t(), 1e-9);
  clock.advance_ticks(40);
  EXPECT_EQ(w.follow(clock), 40u);
  EXPECT_NEAR(w.field_time(), clock.t(), 1e-9);
  EXPECT_NEAR(w.time(), 0.47, 1e-12) << "time() stays the world's own";
  w.set_field_time(10.0);
  w.step(3);
  EXPECT_NEAR(w.field_time(), 10.03, 1e-12);
  EXPECT_THROW(w.set_field_time(std::nan("")), cvc::physics_error);
}

// B6: a box at density 500 settles at half draft on calm water, where the surface is, not at the
// box's top.
TEST(PhysicsFluidSurface, ABoxFloatsAtHalfDraftOnCalmWater) {
  REQUIRE_JOLT();
  for (const double level : {0.0, -1.5}) {
    world w(jolt_config());
    w.add_field(sea(std::make_shared<still_surface>(level)));
    const body_id b = w.add_body(slab(500.0, level));
    w.step(600);
    const body_state s = w.state(b);
    EXPECT_NEAR(level - (s.position.z - 0.2), 0.2, 0.002) << "level " << level;
    EXPECT_LT(std::fabs(s.linear_velocity.z), 0.01) << "level " << level;
  }
}

// B6: in swell the box heaves at the swell's period, in phase with the surface at its centre.
// The box (1 m) is short beside the 25 m wavelength, and its own heave period (about 0.9 s) is
// far below the swell's 4 s, so it follows the surface up and down.
TEST(PhysicsFluidSurface, ABoxHeavesWithTheSwell) {
  REQUIRE_JOLT();
  const auto s = swell(0.2, 4.0);
  world w(jolt_config());
  w.add_field(sea(s));
  const body_id b = w.add_body(slab(500.0, 0.0));
  w.step(400); // settle in
  std::vector<double> z, eta;
  for (int i = 0; i < 1600; ++i) { // 16 s, four periods
    w.step();
    const body_state st = w.state(b);
    z.push_back(st.position.z);
    eta.push_back(s->height(st.position.x, st.position.y, w.field_time()));
  }
  double mz = 0.0, me = 0.0;
  for (std::size_t i = 0; i < z.size(); ++i) {
    mz += z[i] / double(z.size());
    me += eta[i] / double(eta.size());
  }
  double sz = 0.0, se = 0.0, sze = 0.0;
  int ups = 0, first = -1, last = -1;
  for (std::size_t i = 0; i < z.size(); ++i) {
    const double a = z[i] - mz, e = eta[i] - me;
    sz += a * a;
    se += e * e;
    sze += a * e;
    if (i > 0 && z[i - 1] - mz < 0.0 && a >= 0.0) {
      if (first < 0)
        first = int(i);
      last = int(i);
      ++ups;
    }
  }
  EXPECT_GT(sze / std::sqrt(sz * se), 0.97) << "in phase with the surface at its centre";
  EXPECT_NEAR(std::sqrt(sz / se), 1.0, 0.15) << "heaving about as high as the swell";
  ASSERT_GE(ups, 3);
  EXPECT_NEAR((last - first) * 0.01 / (ups - 1), 4.0, 0.12) << "at the swell's period";
}

// The surface is part of the state: its digest and the field time are in save() and
// state_hash(), runs with the same surface agree and runs with different ones do not, and restore
// refuses a world whose surface differs.
TEST(PhysicsFluidSurface, TheSurfaceIsInTheStateAndItsHash) {
  REQUIRE_JOLT();
  auto make = [](std::shared_ptr<const fluid_surface> surface, double t0) {
    auto w = std::make_unique<world>(jolt_config());
    w->add_field(sea(std::move(surface)));
    w->add_body(slab(500.0, 0.0));
    w->add_body(ball_at(1.5, -1.0, 0.5));
    w->set_field_time(t0);
    return w;
  };
  auto a = make(swell(0.2, 4.0), 3.0), b = make(swell(0.2, 4.0), 3.0);
  a->step(250);
  b->step(250);
  EXPECT_EQ(a->state_hash(), b->state_hash());
  EXPECT_NE(make(swell(0.2, 4.0), 3.0)->state_hash(), make(swell(0.21, 4.0), 3.0)->state_hash())
      << "the digest";
  auto later = make(swell(0.2, 4.0), 4.5);
  later->step(250);
  EXPECT_NE(a->state_hash(), later->state_hash()) << "the field time";
  // A save restores into a world holding the same surface, field time included, and the two
  // then step alike.
  const std::vector<std::uint8_t> saved = a->save();
  auto c = make(swell(0.2, 4.0), 99.0);
  c->restore(saved);
  EXPECT_NEAR(c->field_time(), a->field_time(), 1e-12);
  EXPECT_EQ(c->state_hash(), a->state_hash());
  a->step(100);
  c->step(100);
  EXPECT_EQ(c->state_hash(), a->state_hash());
  EXPECT_THROW(make(swell(0.25, 4.0), 3.0)->restore(saved), cvc::physics_error);
  EXPECT_THROW(make(nullptr, 3.0)->restore(saved), cvc::physics_error) << "no surface";
}

// The swell scene's hash, pinned. The surface and the field step compute with det::
// (cvc/algo/det_math.h), so a cross-platform deterministic Jolt gives it on every platform, and
// test_pycvc_physics.py builds the same scene through pycvc and checks it gets this value.
TEST(PhysicsFluidSurface, CrossPlatformGoldenHash) {
  REQUIRE_JOLT();
  if (!(backend_capabilities(backend::jolt) & cap_determinism_cross_platform))
    GTEST_SKIP() << "this Jolt is not cross-platform deterministic";
  world w(jolt_config());
  w.add_field(sea(swell(0.2, 4.0)));
  w.add_body(slab(500.0, 0.0));
  w.add_body(ball_at(1.5, -1.0, 0.5));
  w.set_field_time(3.0);
  w.step(300);
  const std::uint64_t h = w.state_hash();
  std::printf("[physics-golden] swell scene after 300 steps: %016llx\n",
              static_cast<unsigned long long>(h));
  EXPECT_EQ(h, 0xf0085b4212e1a8c0ull) << std::hex << "state hash " << h;
}

// --- the sea as a free surface (ocean_surface.h, cvc::ocean::sea_surface; B2-B6) ---------------

namespace {

// lsystem_coast's long cascade (the 251 m tile, waves from 20 m up) under a 9 m/s wind, on the
// open sea: what a crate rides. `cell` > 0 finds B3's source on a lattice (source_cell_m).
std::shared_ptr<const cvc::ocean::sea_surface> open_sea(double level = 0.0, double cell = 0.0) {
  cvc::ocean::sea_state s;
  s.wind.speed_mps = 9.0;
  s.wind.dir_rad = 0.4;
  const cvc::ocean::cascade c = cvc::ocean::make_cascades({251.0, 83.0, 29.0}, 0xC0A57ull)[0];
  cvc::ocean::sea_params p;
  p.level = level;
  p.band = {32, 64};
  p.chop = 1.15;
  p.source_cell_m = cell;
  return std::make_shared<cvc::ocean::sea_surface>(
      std::vector<cvc::ocean::sea_cascade>{{cvc::ocean::build_h0(64, c, s), 0.0, 0.025}}, p);
}

// open_sea under the wind's forcing (O13): crossfading from a 9 m/s sea to a 13 m/s one over field
// time 16.5..18.5 s, each cascade scaled by the fetch maps of an island 25 m upwind, and a ripple
// cascade that gusts and rides a 0.5 m/s current.
std::shared_ptr<const cvc::ocean::sea_surface> windy_sea(double cell) {
  namespace oc = cvc::ocean;
  oc::sea_state from, to;
  from.wind.speed_mps = 9.0;
  from.wind.dir_rad = 0.0;
  to.wind.speed_mps = 13.0;
  to.wind.dir_rad = 0.4;
  auto g = std::make_shared<oc::shore_grid>(); // the island, for the fetch maps only
  g->rows = g->cols = 81;
  g->min_x = g->min_y = -40.0;
  g->depth.resize(g->count());
  g->shore_m.assign(g->count(), 1.0e4f);
  g->normal_x.assign(g->count(), 0.0f);
  g->normal_y.assign(g->count(), 0.0f);
  for (int r = 0; r < g->rows; ++r)
    for (int c = 0; c < g->cols; ++c)
      g->depth[std::size_t(r) * std::size_t(g->cols) + std::size_t(c)] =
          std::hypot(g->x(c) + 25.0, g->y(r)) < 5.0 ? -1.0f : 20.0f;
  auto forcing = std::make_shared<oc::sea_forcing>();
  forcing->t0 = 16.5;
  forcing->t1 = 18.5;
  forcing->fetch = std::make_shared<oc::fetch_field>(oc::build_fetch(*g, 0.4, 2000.0));
  forcing->fetch_from = std::make_shared<oc::fetch_field>(oc::build_fetch(*g, 0.0, 2000.0));
  forcing->wind = {13.0, 0.4, 0.5};
  forcing->current.speed_mps = 0.5;
  const std::vector<oc::cascade> cs = oc::make_cascades({251.0, 83.0, 29.0}, 0xC0A57ull);
  std::vector<oc::sea_cascade> sc;
  for (int i : {0, 2}) {
    oc::sea_cascade c{oc::build_h0(64, cs[std::size_t(i)], to), 0.0, i == 0 ? 0.025 : 0.0};
    c.h0_from = oc::build_h0(64, cs[std::size_t(i)], from);
    c.full_fetch_m = 500.0;
    c.full_fetch_from_m = 300.0;
    sc.push_back(std::move(c));
  }
  sc[1].gust = 1.0;
  sc[1].advect = true;
  oc::sea_params p;
  p.band = {32, 64};
  p.chop = 1.15;
  p.source_cell_m = cell;
  return std::make_shared<oc::sea_surface>(std::move(sc), p, nullptr, nullptr, forcing);
}

// Water 20 m across, from z = -8 up to a box top of 3 (above the crests), its top the sea.
field ocean_water(std::shared_ptr<const fluid_surface> surface) {
  field f;
  f.kind = field_kind::fluid;
  f.volume = uniform(1000.0, {-10, -10, -8}, {10, 10, 3});
  f.damping = 1.0e4;
  f.surface = std::move(surface);
  return f;
}

} // namespace

TEST(PhysicsOceanSurface, ItIsTheSeaBehindFluidSurface) {
  const auto sea = open_sea(0.5);
  const ocean_surface s(sea);
  EXPECT_EQ(s.level(3.0, 4.0), 0.5);
  EXPECT_EQ(s.height(3.0, 4.0, 7.0), sea->height(3.0, 4.0, 7.0));
  const vec3 v = s.velocity({3.0, 4.0, 0.2}, 7.0);
  const std::array<double, 3> w = sea->velocity(3.0, 4.0, 0.2, 7.0);
  EXPECT_EQ(v.x, w[0]);
  EXPECT_EQ(v.y, w[1]);
  EXPECT_EQ(v.z, w[2]);
  EXPECT_NE(s.digest(), sea->digest()) << "a tag of its own";
  EXPECT_EQ(s.digest(), ocean_surface(open_sea(0.5)).digest());
  EXPECT_NE(s.digest(), ocean_surface(open_sea(0.4)).digest());
  EXPECT_THROW(ocean_surface(nullptr), std::invalid_argument);
}

// B6's exit on the open sea: a box rides the sea a renderer draws, heaving with the surface at its
// centre. The waves (20 m and longer) dwarf the 1 m box, so it follows them up and down.
TEST(PhysicsOceanSurface, ABoxRidesTheSea) {
  REQUIRE_JOLT();
  const auto sea = open_sea();
  world w(jolt_config());
  w.add_field(ocean_water(std::make_shared<ocean_surface>(sea)));
  const body_id b = w.add_body(slab(500.0, 0.0));
  w.step(300); // settle in
  std::vector<double> z, eta;
  for (int i = 0; i < 1500; ++i) {
    w.step();
    const body_state st = w.state(b);
    z.push_back(st.position.z);
    eta.push_back(sea->height(st.position.x, st.position.y, w.field_time()));
  }
  double mz = 0.0, me = 0.0;
  for (std::size_t i = 0; i < z.size(); ++i) {
    mz += z[i] / double(z.size());
    me += eta[i] / double(eta.size());
  }
  double sz = 0.0, se = 0.0, sze = 0.0;
  for (std::size_t i = 0; i < z.size(); ++i) {
    sz += (z[i] - mz) * (z[i] - mz);
    se += (eta[i] - me) * (eta[i] - me);
    sze += (z[i] - mz) * (eta[i] - me);
  }
  ASSERT_GT(se / double(z.size()), 0.01) << "the sea moves";
  EXPECT_GT(sze / std::sqrt(sz * se), 0.95) << "in step with the surface at its centre";
  EXPECT_NEAR(std::sqrt(sz / se), 1.0, 0.2) << "heaving about as high";
  EXPECT_NEAR(mz - me, 0.0, 0.05) << "floating at half draft about the mean surface";
}

// The sea's source on a 0.5 m lattice (sea_params::source_cell_m, what lsystem_coast's debris
// feel): the box rides it as it rides the exact sea, its heave within a centimetre.
TEST(PhysicsOceanSurface, ABoxRidesTheLatticeSeaAsTheExactOne) {
  REQUIRE_JOLT();
  std::vector<double> z[2];
  for (int k = 0; k < 2; ++k) {
    world w(jolt_config());
    w.add_field(ocean_water(std::make_shared<ocean_surface>(open_sea(0.0, k ? 0.5 : 0.0))));
    const body_id b = w.add_body(slab(500.0, 0.0));
    for (int i = 0; i < 1000; ++i) {
      w.step();
      z[k].push_back(w.state(b).position.z);
    }
  }
  double s = 0.0, worst = 0.0;
  for (std::size_t i = 0; i < z[0].size(); ++i) {
    const double d = std::fabs(z[1][i] - z[0][i]);
    s += d * d;
    worst = std::max(worst, d);
  }
  const double rms = std::sqrt(s / double(z[0].size()));
  std::printf("[physics-ocean] the lattice sea moves the box by %.3g um RMS from the exact one "
              "(%.3g um at most)\n",
              1e6 * rms, 1e6 * worst);
  EXPECT_LT(rms, 0.01);
}

// A fluid field reads each cell's height and velocity through one memo (height_with,
// velocity_with), so the sea finds the cell's source once. That changes no value: a world whose
// surface shares nothing between the two reads ends in the same state, bit for bit, on the exact
// sea and on the lattice. (OceanSea.ReportsWhatAPointCosts prints what the memo saves.)
TEST(PhysicsOceanSurface, TheCellsMemoChangesNoValue) {
  REQUIRE_JOLT();
  // The same surface without height_with / velocity_with: each read finds its own source.
  struct apart : fluid_surface {
    explicit apart(std::shared_ptr<const fluid_surface> s) : s(std::move(s)) {}
    double level(double x, double y) const override { return s->level(x, y); }
    double height(double x, double y, double t) const override { return s->height(x, y, t); }
    vec3 velocity(const vec3 &p, double t) const override { return s->velocity(p, t); }
    std::uint64_t digest() const override { return s->digest(); }
    std::shared_ptr<const fluid_surface> s;
  };
  for (double cell : {0.0, 0.5}) {
    std::uint64_t h[2];
    for (int k = 0; k < 2; ++k) {
      std::shared_ptr<const fluid_surface> s = std::make_shared<ocean_surface>(open_sea(0.0, cell));
      if (k)
        s = std::make_shared<apart>(s);
      world w(jolt_config());
      w.add_field(ocean_water(s));
      w.add_body(slab(500.0, 0.0));
      w.add_body(ball_at(2.5, -1.5, 0.3));
      w.set_field_time(17.0);
      w.step(200);
      h[k] = w.state_hash();
    }
    EXPECT_EQ(h[0], h[1]) << std::hex << "cell " << cell << ": " << h[0] << " against " << h[1];
  }
}

// The ocean scene's hash, pinned: the sea is det:: math and the FFT (fft.h), and the field step is
// deterministic, so a cross-platform deterministic Jolt gives it on every platform.
TEST(PhysicsOceanSurface, CrossPlatformGoldenHash) {
  REQUIRE_JOLT();
  if (!(backend_capabilities(backend::jolt) & cap_determinism_cross_platform))
    GTEST_SKIP() << "this Jolt is not cross-platform deterministic";
  world w(jolt_config());
  w.add_field(ocean_water(std::make_shared<ocean_surface>(open_sea())));
  w.add_body(slab(500.0, 0.0));
  w.add_body(ball_at(2.5, -1.5, 0.3));
  w.set_field_time(17.0);
  w.step(300);
  const std::uint64_t h = w.state_hash();
  std::printf("[physics-golden] ocean scene after 300 steps: %016llx\n",
              static_cast<unsigned long long>(h));
  EXPECT_EQ(h, 0xbe86208109133b1aull) << std::hex << "state hash " << h;
}

// --- the scene's wind (flow_velocity.h, O14) ------------------------------------------------

// A flow with a wind and no volume is air (1.225 kg/m^3) everywhere. A ball held in it 10 m up
// (the profile's reference height, where the wind is its U10) is carried along drag's curve,
// v(t) = U - 1 / (1/U + k t), k = rho C_d pi r^2 / (2 m), as in a flow of velocity volumes.
TEST(PhysicsField, TheSceneWindCarriesABallAlongTheDragCurve) {
  REQUIRE_JOLT();
  world_config c = jolt_config();
  c.gravity = {0.0, 0.0, 0.0};
  world w(c);
  const double rho = 1.225, cd = 0.5, wind = 10.0;
  field air;
  air.kind = field_kind::flow;
  air.drag_coefficient = cd;
  air.wind = std::make_shared<wind_flow>(cvc::algo::wind_state{wind, 0.0, 0.0}, 0.0);
  w.add_field(air);
  body_desc d;
  d.shape = sphere{0.5};
  d.mass = 1.0;
  d.position = {0.0, 0.0, 10.0};
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
  std::printf("  a ball 10 m up in a 10 m/s wind: %.3f m/s after %.2f s (drag's curve %.3f)\n",
              w.state(b).linear_velocity.x, w.time(), wind - 1.0 / (1.0 / wind + k * w.time()));
  // Under the wind's base (here 0: under the water, below the ground) there is none.
  EXPECT_EQ(air.wind->velocity({0.0, 0.0, -1.0}, 0.0).x, 0.0);
  // Its profile: 0.72 of U10 at 1 m, 1.26 at 50 m.
  EXPECT_NEAR(air.wind->velocity({0.0, 0.0, 1.0}, 0.0).x / wind, 0.7197, 1e-3);
  EXPECT_NEAR(air.wind->velocity({0.0, 0.0, 50.0}, 0.0).x / wind, 1.2585, 1e-3);
}

namespace {

// A crate 1 x 1 x 0.5 m at density 500 (it floats upright at half draft; a cube at that density
// floats on an edge) on calm water 120 m across, flowing at `current`, in a wind of `wind` m/s
// toward +y over the water's level, for `seconds`. Its velocity at the end, and the immersed
// volume and greatest tilt on the way.
struct crate_run {
  vec3 velocity;
  double immersed = 0.0, tilted = 0.0;
};
crate_run float_crate(double damping, bool inertial, const vec3 &current, double wind,
                      double seconds) {
  world w(jolt_config());
  field sea_f;
  sea_f.kind = field_kind::fluid;
  sea_f.volume = uniform(1000.0, {-60, -60, -5}, {60, 60, 1});
  sea_f.damping = damping;
  sea_f.inertial = inertial;
  sea_f.surface = std::make_shared<still_surface>(0.0, current);
  w.add_field(sea_f);
  if (wind > 0.0) {
    field air;
    air.kind = field_kind::flow;
    air.wind = std::make_shared<wind_flow>(cvc::algo::wind_state{wind, 1.5707963267948966, 0.0});
    w.add_field(air);
  }
  body_desc d;
  d.shape = box{{0.5, 0.5, 0.25}};
  d.density = 500.0;
  d.position = {0.0, 0.0, 0.0};
  d.linear_damping = 0.0; // the fields' drag alone
  d.allow_sleeping = false;
  const body_id b = w.add_body(d);
  crate_run out;
  const int n = static_cast<int>(seconds / w.config().fixed_dt + 0.5);
  for (int i = 0; i < n; ++i) {
    w.step();
    out.tilted = std::max(out.tilted, tilt(w.state(b).rotation));
  }
  out.immersed = w.field_load_on(b).immersed_volume;
  out.velocity = w.state(b).linear_velocity;
  return out;
}

// The speed a crate drifts at through the water where the air's drag on its top half balances
// the water's on its bottom half (the crate's 4 layers of cells of 0.125 m, the top two in the
// air; its 0.5 m^2 across, C_d 1, in both): rho_air sum_l (1/4) (U p(z_l) - v)^2 =
// rho_water (1/2) v^2.
double leeway(double U) {
  auto air_minus_water = [U](double v) {
    double air_drag = 0.0;
    for (int l = 0; l < 2; ++l) {
      const double u = U * cvc::algo::wind_profile(0.0625 + 0.125 * l) - v;
      air_drag += 0.25 * u * std::fabs(u);
    }
    return 1.225 * air_drag - 1000.0 * 0.5 * v * v;
  };
  double lo = 0.0, hi = U;
  for (int i = 0; i < 60; ++i)
    (air_minus_water(0.5 * (lo + hi)) > 0.0 ? lo : hi) = 0.5 * (lo + hi);
  return 0.5 * (lo + hi);
}

} // namespace

// A floating crate drifts downwind (leeway): the wind's drag on its half above the water, read
// up its height through the profile, against the water's drag on its half below. The air is
// not counted as immersed. With the water's drag alone the crate drifts at the speed the two
// balance at. A damping in every direction (1e4 N s/m^4, which makes a body follow the waves)
// holds it to millimetres a second; an inertial fluid's damping is on the vertical only, so the
// crate drifts as on drag alone.
TEST(PhysicsField, AFloatingCrateDriftsDownwind) {
  REQUIRE_JOLT();
  const double U = 10.0, balance = leeway(U);
  const crate_run loose = float_crate(0.0, false, {}, U, 30.0);
  std::printf("  a floating crate in a %.0f m/s wind drifts %.3f m/s (the balance %.3f)\n", U,
              loose.velocity.y, balance);
  EXPECT_NEAR(loose.velocity.y, balance, 0.1 * balance);
  EXPECT_NEAR(loose.velocity.x, 0.0, 0.01 * balance);
  EXPECT_NEAR(loose.immersed, 0.25, 0.0125)
      << "half its 0.5 m^3: the air is not counted as immersed";
  EXPECT_LT(loose.tilted, 0.1) << "it floats upright";
  const crate_run damped = float_crate(1.0e4, false, {}, U, 30.0);
  const crate_run inertial = float_crate(1.0e4, true, {}, U, 30.0);
  std::printf("  damped in every direction %.4f m/s; inertial %.3f m/s; tilted at most %.3f rad\n",
              damped.velocity.y, inertial.velocity.y, inertial.tilted);
  EXPECT_GT(damped.velocity.y, 0.0);
  EXPECT_LT(damped.velocity.y, 0.01);
  EXPECT_NEAR(inertial.velocity.y, balance, 0.1 * balance) << "the damping does not hold it";
  EXPECT_LT(inertial.tilted, 0.1);
}

// An inertial fluid carries a floating body on its current, and a wind blows it across the
// current at the leeway it has in still water: its velocity is the current plus the leeway.
TEST(PhysicsField, AnInertialCrateDriftsWithTheCurrent) {
  REQUIRE_JOLT();
  const vec3 current{0.5, 0.0, 0.0};
  const crate_run carried = float_crate(1.0e4, true, current, 0.0, 60.0);
  const crate_run both = float_crate(1.0e4, true, current, 10.0, 60.0);
  std::printf("  in a 0.5 m/s current: %.3f m/s after 60 s; with a 10 m/s wind across it "
              "(%.3f, %.3f) m/s, the leeway %.3f\n",
              carried.velocity.x, both.velocity.x, both.velocity.y, leeway(10.0));
  EXPECT_NEAR(carried.velocity.x, current.x, 0.1 * current.x) << "it takes the current";
  EXPECT_NEAR(carried.velocity.y, 0.0, 0.005);
  EXPECT_NEAR(both.velocity.x, current.x, 0.1 * current.x);
  EXPECT_NEAR(both.velocity.y, leeway(10.0), 0.15 * leeway(10.0)) << "and the wind's leeway";
}

// An inertial fluid's damping is on the vertical only, and sideways the water's acceleration
// carries a floating body: in a swell a body heaves with the surface as a fully damped one does
// (ABoxHeavesWithTheSwell), and its velocity along the swell follows the water's orbital
// velocity at its centre.
TEST(PhysicsFluidSurface, AnInertialBodyRidesTheSwell) {
  REQUIRE_JOLT();
  const auto s = swell(0.2, 4.0);
  world w(jolt_config());
  field f = sea(s);
  f.inertial = true;
  w.add_field(f);
  body_desc d = slab(500.0, 0.0);
  d.linear_damping = 0.0;
  const body_id b = w.add_body(d);
  w.step(400);                                         // settle in
  const double ux = std::cos(0.3), uy = std::sin(0.3); // along the swell
  std::vector<double> z, eta, v, u;
  vec3 start = w.state(b).position;
  for (int i = 0; i < 1600; ++i) { // 16 s, four periods
    w.step();
    const body_state st = w.state(b);
    z.push_back(st.position.z);
    eta.push_back(s->height(st.position.x, st.position.y, w.field_time()));
    v.push_back(st.linear_velocity.x * ux + st.linear_velocity.y * uy);
    const vec3 wv = s->velocity(st.position, w.field_time());
    u.push_back(wv.x * ux + wv.y * uy);
  }
  auto corr = [](const std::vector<double> &a, const std::vector<double> &b, double &ratio) {
    double ma = 0.0, mb = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
      ma += a[i] / double(a.size());
      mb += b[i] / double(b.size());
    }
    double saa = 0.0, sbb = 0.0, sab = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
      saa += (a[i] - ma) * (a[i] - ma);
      sbb += (b[i] - mb) * (b[i] - mb);
      sab += (a[i] - ma) * (b[i] - mb);
    }
    ratio = std::sqrt(saa / sbb);
    return sab / std::sqrt(saa * sbb);
  };
  double heave = 0.0, sway = 0.0;
  const double cz = corr(z, eta, heave), cv = corr(v, u, sway);
  const body_state end = w.state(b);
  const double moved = (end.position.x - start.x) * ux + (end.position.y - start.y) * uy;
  std::printf("  inertial in a swell: heave %.2f of the surface's (r %.3f); along the swell its "
              "velocity %.2f of the water's (r %.3f); moved %.2f m in 16 s\n",
              heave, cz, sway, cv, moved);
  EXPECT_GT(cz, 0.97) << "in phase with the surface";
  EXPECT_NEAR(heave, 1.0, 0.15) << "heaving about as high as the swell";
  EXPECT_GT(cv, 0.9) << "carried back and forth with the water";
  EXPECT_NEAR(sway, 1.0, 0.25);
  EXPECT_LT(std::fabs(moved), 2.0) << "the waves move it to and fro, not away";
}

// The wind is part of the world's state: two worlds with one wind hash alike, a different wind
// hashes differently and its state does not restore into the other, and a world with no wind
// hashes as it always did.
TEST(PhysicsField, TheSceneWindIsInTheStateHash) {
  world_config c;
  c.backend = backend::null;
  auto with = [&](double speed) {
    auto w = std::make_unique<world>(c);
    field air;
    air.kind = field_kind::flow;
    air.wind = std::make_shared<wind_flow>(cvc::algo::wind_state{speed, 0.3, 0.25}, 0.0);
    w->add_field(air);
    return w;
  };
  const std::unique_ptr<world> a = with(9.0), b = with(9.0), x = with(12.0);
  EXPECT_EQ(a->state_hash(), b->state_hash());
  EXPECT_NE(a->state_hash(), x->state_hash());
  const std::vector<std::uint8_t> s = a->save();
  EXPECT_NO_THROW(b->restore(s));
  EXPECT_THROW(x->restore(s), cvc::physics_error);
  // No wind: the hash of a world that never had one.
  world plain(c), fresh(c);
  EXPECT_EQ(plain.state_hash(), fresh.state_hash());
  EXPECT_NE(plain.state_hash(), a->state_hash());
  // Refusals: a wind on a fluid; a flow with neither a volume nor a wind.
  world r(c);
  field wet;
  wet.kind = field_kind::fluid;
  wet.volume = uniform(1000.0, {-1, -1, -1}, {1, 1, 0});
  wet.wind = std::make_shared<wind_flow>(cvc::algo::wind_state{5.0, 0.0, 0.0});
  EXPECT_THROW(r.add_field(wet), cvc::physics_error);
  field bare;
  bare.kind = field_kind::flow;
  EXPECT_THROW(r.add_field(bare), cvc::physics_error);
  EXPECT_THROW(wind_flow(cvc::algo::wind_state{-1.0, 0.0, 0.0}), std::invalid_argument);
}

// The wind's velocity is a pure function of the point and the time, the same on every platform.
TEST(PhysicsField, TheSceneWindIsTheSameOnEveryPlatform) {
  const wind_flow f(cvc::algo::wind_state{9.0, 0.6, 0.3}, 2.0);
  std::uint64_t h = 1469598103934665603ull;
  auto add = [&h](double v) {
    unsigned char b[sizeof v];
    std::memcpy(b, &v, sizeof v);
    for (unsigned char x : b)
      h = (h ^ x) * 1099511628211ull;
  };
  for (int k = 0; k < 5; ++k)
    for (int i = 0; i < 40; ++i) {
      // integer inputs: exact, so no platform contracts them differently
      const vec3 v = f.velocity({-300.0 + 17.0 * i, 120.0 - 9.0 * i, 2.0 + (i % 7)}, 11.0 * k);
      add(v.x);
      add(v.y);
      add(v.z);
    }
  std::printf("  wind digest 0x%016llx, samples 0x%016llx\n",
              static_cast<unsigned long long>(f.digest()), static_cast<unsigned long long>(h));
  EXPECT_EQ(f.digest(), 0xc7ec24178041550eull) << std::hex << "wind_flow digest 0x" << f.digest();
  EXPECT_EQ(h, 0x3ec1c28ca0601404ull) << std::hex << "wind samples 0x" << h;
}

// The fields' surface reads on worker threads (world_config::field_threads) give the same run:
// several bodies on the sea, with and without the source lattice, at 0, 1 and 3 workers, and the
// workers changed mid-run by reconfigure; on the plain sea and under the wind's forcing (the
// crossfade, the fetch, the gusts and the current).
TEST(PhysicsOceanSurface, TheFieldThreadsChangeNoValue) {
  REQUIRE_JOLT();
  std::uint64_t first[4] = {};
  for (int run = 0; run < 4; ++run) {
    const double cell = run % 2 == 0 ? 0.0 : 0.5;
    const bool windy = run >= 2;
    std::uint64_t h[4];
    for (int k = 0; k < 4; ++k) {
      world_config c = jolt_config();
      c.field_threads = k == 3 ? 0 : (k == 0 ? 0 : (k == 1 ? 1 : 3));
      world w(c);
      w.add_field(ocean_water(
          std::make_shared<ocean_surface>(windy ? windy_sea(cell) : open_sea(0.0, cell))));
      w.add_body(slab(500.0, 0.0));
      w.add_body(ball_at(2.5, -1.5, 0.3));
      w.add_body(ball_at(-3.0, 2.0, 0.1));
      body_desc tilted = slab(420.0, 0.2);
      tilted.position = {-1.0, -3.0, 0.2};
      const double half = 0.2; // 0.4 rad about (0.6, 0.8, 0)
      tilted.rotation = {std::cos(half), 0.6 * std::sin(half), 0.8 * std::sin(half), 0.0};
      w.add_body(tilted);
      w.set_field_time(17.0);
      if (k == 3) { // switched on and off as it runs
        w.step(60);
        c.field_threads = 2;
        w.reconfigure(c);
        w.step(80);
        c.field_threads = 0;
        w.reconfigure(c);
        w.step(60);
      } else {
        w.step(200);
      }
      h[k] = w.state_hash();
    }
    for (int k = 1; k < 4; ++k)
      EXPECT_EQ(h[0], h[k]) << std::hex << (windy ? "windy, " : "") << "cell " << cell << ", run "
                            << k << ": " << h[0] << " against " << h[k];
    first[run] = h[0];
  }
  EXPECT_NE(first[2], first[0]) << "the wind moves the bodies differently";
  EXPECT_NE(first[3], first[1]);
  world_config bad = jolt_config();
  bad.field_threads = -1;
  EXPECT_THROW(world w(bad), cvc::physics_error);
  bad.field_threads = 65;
  EXPECT_THROW(world w(bad), cvc::physics_error);
}

// Hulls on the sea (hull.h): dyn_boat_desc::tactical's lofted V hull, yawed and heeled, and a box
// hull, beside a ball of cells. Their pushes run on the field workers (field_threads) and give the
// same run at 0, 1 and 3 workers, and with the workers changed mid-run, on the plain sea and under
// the wind's forcing, with and without the source lattice. The windy lattice run's hash is pinned:
// the hulls read the sea the step holds at its time (fluid_surface::at), on the workers, from the
// lattice in shards, and that changed no value (it was taken when each hull read the sea itself,
// in turn, and the lattice had one lock).
TEST(PhysicsOceanSurface, TheHullsOnTheFieldWorkersChangeNoValue) {
  REQUIRE_JOLT();
  const auto run = [](const std::shared_ptr<const cvc::ocean::sea_surface> &sea, int threads,
                      bool switched) {
    world_config c = jolt_config();
    c.field_threads = threads;
    world w(c);
    w.add_field(ocean_water(std::make_shared<ocean_surface>(sea)));
    const dyn_boat_desc boat = dyn_boat_desc::tactical();
    body_desc v = boat.boat.hull;
    v.position = {0.5, -3.0, dyn_boat_plant::float_height(boat.boat, v.mass, 1000.0) + 0.05};
    // Yawed about z (cos and sin of the half angle 0.96 and 0.28), then heeled a little about its
    // own x; std::sqrt is correctly rounded everywhere, so the pinned hash is every platform's.
    const double cy = 0.96, sy = 0.28, ch = 0.998, sh = std::sqrt(1.0 - ch * ch);
    v.rotation = {cy * ch, cy * sh, sy * sh, sy * ch};
    w.add_body(v);
    body_desc b;
    b.shape = box{{2.0, 0.75, 0.3}};
    b.hull = box_hull({2.0, 0.75, 0.3});
    b.density = 400.0;
    b.position = {-2.0, 4.0, 0.0};
    b.allow_sleeping = false;
    w.add_body(b);
    w.add_body(ball_at(5.0, 3.0, 0.3));
    w.set_field_time(17.0);
    if (switched) {
      w.step(50);
      c.field_threads = 3;
      w.reconfigure(c);
      w.step(50);
      c.field_threads = 0;
      w.reconfigure(c);
      w.step(50);
    } else {
      w.step(150);
    }
    return w.state_hash();
  };
  for (int k = 0; k < 4; ++k) {
    const double cell = k % 2 == 0 ? 0.0 : 0.5;
    const bool windy = k >= 2;
    const auto sea = windy ? windy_sea(cell) : open_sea(0.0, cell);
    const std::uint64_t h = run(sea, 0, false);
    EXPECT_EQ(run(sea, 1, false), h) << (windy ? "windy, " : "") << "cell " << cell << ": 1 worker";
    EXPECT_EQ(run(sea, 3, false), h) << (windy ? "windy, " : "") << "cell " << cell << ": 3";
    EXPECT_EQ(run(sea, 0, true), h) << (windy ? "windy, " : "") << "cell " << cell << ": switched";
    if (k == 3 && (backend_capabilities(backend::jolt) & cap_determinism_cross_platform)) {
      std::printf("[physics-golden] hulls on the windy lattice sea after 150 steps: %016llx\n",
                  static_cast<unsigned long long>(h));
      EXPECT_EQ(h, 0xe23ce116800e66c4ull) << std::hex << "state hash " << h;
    }
  }
}

namespace {

// A surface that counts the times the world holds it at a time (fluid_surface::at), and those
// with the field workers to do its per-time work on; its values are the one it wraps.
class counted_at : public fluid_surface {
public:
  explicit counted_at(std::shared_ptr<const fluid_surface> s) : _s(std::move(s)) {}
  double level(double x, double y) const override { return _s->level(x, y); }
  double height(double x, double y, double t) const override { return _s->height(x, y, t); }
  vec3 velocity(const vec3 &p, double t) const override { return _s->velocity(p, t); }
  std::uint64_t digest() const override { return _s->digest(); }
  double height_with(double x, double y, double t, surface_memo &m) const override {
    return _s->height_with(x, y, t, m);
  }
  vec3 velocity_with(const vec3 &p, double t, surface_memo &m) const override {
    return _s->velocity_with(p, t, m);
  }
  std::shared_ptr<const fluid_surface> at(double t, const parallel_run &run) const override {
    ++calls; // from the world's calls, which come one at a time
    if (run)
      ++on_workers;
    return _s->at(t, run);
  }
  mutable int calls = 0, on_workers = 0;

private:
  std::shared_ptr<const fluid_surface> _s;
};

std::uint64_t bits_of(double v) {
  std::uint64_t b;
  std::memcpy(&b, &v, sizeof b);
  return b;
}

} // namespace

// The first read of a fluid's surface at a field time holds it there (fluid_surface::at), and
// every read at that time after it reads the one held: world::water_at's and the step's. A boat
// plant reads the water before each step, so that read makes the sea's frame for the step's time,
// on the field workers once a step has made them, and the step finds it held. One hold a field
// time; water_at reads what a sea of its own (its frame made serially) reads, bit for bit; a field
// given another surface, or removed and added again, at the same time reads the new one; and reads
// between the steps change no state.
TEST(PhysicsOceanSurface, TheFirstReadAtAFieldTimeHoldsTheSea) {
  REQUIRE_JOLT();
  const auto counted =
      std::make_shared<counted_at>(std::make_shared<ocean_surface>(windy_sea(0.5)));
  world_config c = jolt_config();
  c.field_threads = 3;
  world w(c);
  const field_id water = w.add_field(ocean_water(counted));
  w.add_body(slab(500.0, 0.0));
  w.set_field_time(17.0);
  const vec3 p{1.5, -2.0, -1.0};
  w.water_at(p);
  w.water_at({-3.0, 4.0, -0.5});
  EXPECT_EQ(counted->calls, 1) << "one hold a field time";
  EXPECT_EQ(counted->on_workers, 0) << "no step has made the workers yet: held on this thread";
  w.step();
  EXPECT_EQ(counted->calls, 1) << "the step reads the surface water_at held at its time";
  for (int s = 0; s < 5; ++s) { // as a boat plant steps: its reads, then the world's step
    w.water_at(p);
    EXPECT_EQ(counted->calls, 2 + s);
    EXPECT_EQ(counted->on_workers, 1 + s) << "held on the field workers";
    w.step();
    EXPECT_EQ(counted->calls, 2 + s) << "the step found it held";
  }

  const auto check = [&](const std::shared_ptr<const cvc::ocean::sea_surface> &own,
                         const char *what) {
    const double t = w.field_time();
    const water_sample ws = w.water_at(p);
    const std::array<double, 3> v = own->velocity(p.x, p.y, p.z, t);
    EXPECT_TRUE(ws.wet) << what;
    EXPECT_EQ(bits_of(ws.depth), bits_of(own->height(p.x, p.y, t) - p.z)) << what << ": depth";
    EXPECT_EQ(bits_of(ws.velocity.x), bits_of(v[0])) << what << ": u";
    EXPECT_EQ(bits_of(ws.velocity.y), bits_of(v[1])) << what << ": v";
    EXPECT_EQ(bits_of(ws.velocity.z), bits_of(v[2])) << what << ": w";
  };
  check(windy_sea(0.5), "the held sea");
  w.set_field(water, ocean_water(std::make_shared<ocean_surface>(open_sea(0.25, 0.5))));
  check(open_sea(0.25, 0.5), "another sea set at the same time");
  w.remove_field(water);
  w.add_field(ocean_water(std::make_shared<ocean_surface>(open_sea(-0.25, 0.0))));
  check(open_sea(-0.25, 0.0), "a sea added at the same time after the field was removed");

  const auto run = [](bool reads) {
    world_config rc = jolt_config();
    rc.field_threads = 3;
    world rw(rc);
    rw.add_field(ocean_water(std::make_shared<ocean_surface>(windy_sea(0.5))));
    rw.add_body(slab(500.0, 0.0));
    rw.add_body(ball_at(2.5, -1.5, 0.3));
    rw.set_field_time(17.0);
    for (int s = 0; s < 60; ++s) {
      if (reads)
        rw.water_at({1.5, -2.0, -1.0});
      rw.step();
    }
    return rw.state_hash();
  };
  EXPECT_EQ(run(true), run(false)) << "reads before the steps changed the run";
}

// --- wakes (field::wake; OCEAN-AND-WEATHER-ROADMAP.md C5, D6) ------------------------------------

namespace {

// Calm water 200 m across, inertial and damped as a water node's (scene_water.cpp), with a wake of
// 64 cells of 1 m when `wake`.
field wake_water(bool wake) {
  field f;
  f.kind = field_kind::fluid;
  f.volume = uniform(1000.0, {-100, -100, -8}, {100, 100, 4});
  f.damping = 1.0e4;
  f.inertial = true;
  f.surface = std::make_shared<still_surface>(0.0);
  if (wake) {
    f.wake.cells = 64;
    f.wake.cell = 1.0;
  }
  return f;
}

// A crate 1.6 x 1.6 x 0.8 m at density 500: afloat at rest with its centre at z = 0.
body_desc crate_at(double x, double y, double z) {
  body_desc d;
  d.shape = box{{0.8, 0.8, 0.4}};
  d.density = 500.0;
  d.position = {x, y, z};
  d.linear_damping = 0.0; // the water's drag alone
  return d;
}

// A crate dropped from 1.2 m at the origin beside one afloat at rest `apart` metres off along x
// (which falls asleep), for `steps` steps of 0.01 s: the resting one's largest heave, drift and
// tilt, the run's hash, the wake at the end, the most sources it had and the largest wave 2 m off
// the drop.
struct drop_run {
  double heave = 0.0, drift = 0.0, rolled = 0.0;
  std::uint64_t hash = 0;
  wake_info info;
  std::size_t most_sources = 0;
  double ring = 0.0;
  double fields_ms = 0.0; // the fields' push a step (step_report::fields_ms), the mean
};
drop_run drop_beside(bool wake, double apart, int steps, int threads = 0) {
  world_config c = jolt_config();
  c.field_threads = threads;
  world w(c);
  const field_id water = w.add_field(wake_water(wake));
  w.add_body(crate_at(0.0, 0.0, 1.2));
  const body_id resting = w.add_body(crate_at(apart, 0.0, 0.0));
  const vec3 p0 = w.state(resting).position;
  drop_run out;
  for (int i = 0; i < steps; ++i) {
    out.fields_ms += w.step().fields_ms / steps;
    const body_state s = w.state(resting);
    out.heave = std::max(out.heave, std::fabs(s.position.z - p0.z));
    out.drift = std::max(out.drift, std::hypot(s.position.x - p0.x, s.position.y - p0.y));
    out.rolled = std::max(out.rolled, tilt(s.rotation));
    out.ring = std::max(out.ring, std::fabs(w.field_wake_height(water, 0.0, 2.0)));
    out.most_sources = std::max(out.most_sources, w.field_wake_info(water).sources.size());
  }
  out.hash = w.state_hash();
  out.info = w.field_wake_info(water);
  return out;
}

} // namespace

// A wake is a fluid's with a free surface, with a window of a power of two in 8 .. 128 cells and
// its other parameters within their limits.
TEST(PhysicsFieldWake, MalformedWakesAreRefused) {
  world_config c;
  c.backend = backend::null;
  world w(c);
  field f = wake_water(true);
  f.wake.cells = 48;
  EXPECT_THROW(w.add_field(f), cvc::physics_error);
  f.wake.cells = 512;
  EXPECT_THROW(w.add_field(f), cvc::physics_error);
  f.wake.cells = 256;
  EXPECT_THROW(w.add_field(f), cvc::physics_error);
  f.wake.cells = -1;
  EXPECT_THROW(w.add_field(f), cvc::physics_error);
  f = wake_water(true);
  f.wake.cell = 0.0;
  EXPECT_THROW(w.add_field(f), cvc::physics_error);
  f = wake_water(true);
  f.wake.sources = 0;
  EXPECT_THROW(w.add_field(f), cvc::physics_error);
  f.wake.sources = 17; // the limits keep a wake's memory and its wave steps in bounds
  EXPECT_THROW(w.add_field(f), cvc::physics_error);
  f = wake_water(true);
  f.wake.period = 11.0;
  EXPECT_THROW(w.add_field(f), cvc::physics_error);
  f = wake_water(true);
  f.wake.cell = 0.01;
  EXPECT_THROW(w.add_field(f), cvc::physics_error);
  f = wake_water(true);
  f.surface = nullptr; // a flat-topped box has no waves to carry
  EXPECT_THROW(w.add_field(f), cvc::physics_error);
  field air;
  air.kind = field_kind::flow;
  air.volume = uniform(1.2, {-5, -5, 0}, {5, 5, 5});
  air.wake.cells = 64;
  EXPECT_THROW(w.add_field(air), cvc::physics_error);
  EXPECT_NO_THROW(w.add_field(wake_water(true)));
  EXPECT_NO_THROW(w.add_field(wake_water(false)));
}

// A body never reads its own waves: alone on the water, a crate dropped in makes a ring (the wake
// has it, and its one source while it moves), but falls, bobs and settles exactly as it does with
// no wake.
TEST(PhysicsFieldWake, ABodyNeverFeelsItsOwnWaves) {
  REQUIRE_JOLT();
  std::vector<vec3> path[2];
  double ring = 0.0;
  for (int k = 0; k < 2; ++k) {
    world w(jolt_config());
    const field_id water = w.add_field(wake_water(k == 1));
    const body_id b = w.add_body(crate_at(0.0, 0.0, 1.2));
    wake_info info; // while it bobs
    for (int i = 0; i < 400; ++i) {
      w.step();
      path[k].push_back(w.state(b).position);
      if (k == 1)
        ring = std::max(ring, std::fabs(w.field_wake_height(water, 0.0, 3.0)));
      if (k == 1 && i == 99)
        info = w.field_wake_info(water);
    }
    if (k == 1) {
      // Asleep at the end, it gives its slot up.
      EXPECT_FALSE(w.state(b).active);
      EXPECT_TRUE(w.field_wake_info(water).sources.empty());
      EXPECT_TRUE(info.placed);
      ASSERT_EQ(info.sources.size(), 1u);
      EXPECT_EQ(info.sources[0], b);
      EXPECT_EQ(info.cells, 64);
      // The window is centred on the crate, on whole cells.
      EXPECT_DOUBLE_EQ(info.x0, -32.0);
      EXPECT_DOUBLE_EQ(info.y0, -32.0);
    }
  }
  std::printf("[wake] a crate dropped from 1.2 m: its ring is up to %.3f m high 3 m off\n", ring);
  EXPECT_GT(ring, 0.01) << "it makes waves";
  for (std::size_t i = 0; i < path[0].size(); ++i) {
    ASSERT_EQ(path[0][i].x, path[1][i].x) << "step " << i;
    ASSERT_EQ(path[0][i].y, path[1][i].y) << "step " << i;
    ASSERT_EQ(path[0][i].z, path[1][i].z) << "step " << i;
  }
}

// One body's waves push another: a crate dropped beside one afloat at rest (asleep by the time
// the ring comes) rocks it with a wake, and leaves it still without one.
TEST(PhysicsFieldWake, ADroppedCratesRingRocksItsNeighbour) {
  REQUIRE_JOLT();
  const drop_run off = drop_beside(false, 5.0, 900);
  const drop_run on = drop_beside(true, 5.0, 900);
  std::printf("[wake] the neighbour 5 m off over 9 s: without a wake heave %.5f m, drift %.5f m; "
              "with one heave %.4f m, drift %.4f m, tilt %.4f rad (the ring %.3f m high 2 m off "
              "the drop)\n",
              off.heave, off.drift, on.heave, on.drift, on.rolled, on.ring);
  std::printf("[wake] the fields' push a step: %.3f ms without a wake, %.3f ms with one (a 64^2 "
              "window, two sources, its waves stepped every third step)\n",
              off.fields_ms, on.fields_ms);
  EXPECT_LT(off.heave, 1e-3);
  EXPECT_LT(off.drift, 1e-3);
  EXPECT_GT(on.heave, 5e-3) << "the ring lifts it";
  EXPECT_GT(on.heave, 10.0 * off.heave);
  EXPECT_NE(on.hash, off.hash);
  // Both make waves. (Not checked at the end: the dropped crate falls asleep and the neighbour's
  // waves wake it again and again, so whether it holds a slot at the last step is timing.)
  EXPECT_EQ(on.most_sources, 2u);
}

// The wake is world state: a save restores its waves, so the run goes on to the same hash; its
// section is in the hash; a world with another wake (or none) refuses the state; and the field
// workers change no value.
TEST(PhysicsFieldWake, TheWakeIsWorldState) {
  REQUIRE_JOLT();
  const auto make = [](bool wake, world &w) {
    w.add_field(wake_water(wake));
    w.add_body(crate_at(0.0, 0.0, 1.2));
    w.add_body(crate_at(4.0, 1.0, 0.0));
  };
  world a(jolt_config());
  make(true, a);
  a.step(150);
  const std::vector<std::uint8_t> saved = a.save();
  a.step(200);
  const std::uint64_t straight = a.state_hash();
  a.restore(saved);
  a.step(200);
  EXPECT_EQ(a.state_hash(), straight) << "the waves came back with the state";
  world b(jolt_config());
  make(true, b);
  b.restore(saved);
  b.step(200);
  EXPECT_EQ(b.state_hash(), straight) << "and into a fresh world";
  // Without a wake the state has no wake section; the two worlds refuse each other's states.
  world none(jolt_config());
  make(false, none);
  none.step(150);
  EXPECT_THROW(none.restore(saved), cvc::physics_error);
  EXPECT_THROW(b.restore(none.save()), cvc::physics_error);
  world other(jolt_config());
  field f = wake_water(true);
  f.wake.cells = 32;
  other.add_field(f);
  other.add_body(crate_at(0.0, 0.0, 1.2));
  other.add_body(crate_at(4.0, 1.0, 0.0));
  EXPECT_THROW(other.restore(saved), cvc::physics_error);
  // The field workers read the waves as the stepping thread does.
  EXPECT_EQ(drop_beside(true, 5.0, 300, 0).hash, drop_beside(true, 5.0, 300, 3).hash);
}

// set_field with the same wake keeps the waves (a new sea under them); another wake starts afresh.
TEST(PhysicsFieldWake, TheSameWakeKeepsItsWavesAcrossASetField) {
  REQUIRE_JOLT();
  world w(jolt_config());
  const field_id water = w.add_field(wake_water(true));
  w.add_body(crate_at(0.0, 0.0, 1.2));
  w.add_body(crate_at(4.0, 1.0, 0.0));
  w.step(150);
  const double before = w.field_wake_height(water, 0.0, 3.0);
  ASSERT_NE(before, 0.0);
  w.set_field(water, wake_water(true));
  EXPECT_EQ(w.field_wake_height(water, 0.0, 3.0), before);
  EXPECT_TRUE(w.field_wake_info(water).placed);
  field f = wake_water(true);
  f.wake.damping = 0.05;
  w.set_field(water, f);
  EXPECT_EQ(w.field_wake_height(water, 0.0, 3.0), 0.0);
  EXPECT_FALSE(w.field_wake_info(water).placed);
}

// The window follows the bodies by whole cells: a crate moved 40 m takes the window with it at
// the next of the waves' steps, and it still makes waves there.
TEST(PhysicsFieldWake, TheWindowFollowsTheBodies) {
  REQUIRE_JOLT();
  world w(jolt_config());
  const field_id water = w.add_field(wake_water(true));
  body_desc d = crate_at(0.0, 0.0, 0.0);
  d.allow_sleeping = false;
  const body_id b = w.add_body(d);
  w.step(10);
  EXPECT_DOUBLE_EQ(w.field_wake_info(water).x0, -32.0);
  body_state s = w.state(b);
  s.position = {40.0, 0.0, 0.3}; // floating high there: it bobs down and makes waves
  w.set_state(b, s);
  w.step(10);
  const wake_info info = w.field_wake_info(water);
  EXPECT_DOUBLE_EQ(info.x0, 40.0 - 32.0);
  EXPECT_DOUBLE_EQ(info.y0, -32.0);
  ASSERT_EQ(info.sources.size(), 1u);
  w.step(100);
  EXPECT_NE(w.field_wake_height(water, 40.0, 3.0), 0.0) << "its splash there";
}

// --- wakes: the review's cases ------------------------------------------------------------------

namespace {

// A launch-sized box hull (4 x 1.5 x 0.6 m at density 400): a boat to the wake (it has a hull).
body_desc boat_at(double x, double y) {
  body_desc d;
  d.shape = box{{2.0, 0.75, 0.3}};
  d.hull = box_hull({2.0, 0.75, 0.3});
  d.density = 400.0;
  d.position = {x, y, 0.0};
  d.linear_damping = 0.0;
  d.allow_sleeping = false;
  return d;
}

// Still water whose reads are counted (thread-safe), so a test can see when the step reads it.
class counted_still : public fluid_surface {
public:
  double level(double, double) const override { return 0.0; }
  double height(double, double, double) const override {
    reads.fetch_add(1, std::memory_order_relaxed);
    return 0.0;
  }
  vec3 velocity(const vec3 &, double) const override { return {}; }
  std::uint64_t digest() const override { return 0x636f756e74656421ull; }
  mutable std::atomic<std::uint64_t> reads{0};
};

// The sea of a water node, roughly: three cascades (251, 83 and 29 m tiles) of 64 modes in a
// 12 m/s wind, its source on a 0.5 m lattice.
std::shared_ptr<const cvc::ocean::sea_surface> three_cascade_sea() {
  cvc::ocean::sea_state s;
  s.wind.speed_mps = 12.0;
  s.wind.dir_rad = 0.4;
  const std::vector<cvc::ocean::cascade> cs =
      cvc::ocean::make_cascades({251.0, 83.0, 29.0}, 0xC0A57ull);
  std::vector<cvc::ocean::sea_cascade> sc;
  const double decay[3] = {0.025, 0.075, 0.2};
  for (int i = 0; i < 3; ++i)
    sc.push_back({cvc::ocean::build_h0(64, cs[i], s), 0.0, decay[i]});
  cvc::ocean::sea_params p;
  p.band = {32, 64};
  p.chop = 1.15;
  p.source_cell_m = 0.5;
  return std::make_shared<cvc::ocean::sea_surface>(std::move(sc), p);
}

// What a field's wake reads now at a few points round (cx, cy), the largest |height|.
double wake_peak(const world &w, field_id water, double cx, double cy) {
  double m = 0.0;
  for (int j = -4; j <= 4; ++j)
    for (int i = -4; i <= 4; ++i)
      m = std::max(m, std::fabs(w.field_wake_height(water, cx + 2.0 * i, cy + 2.0 * j)));
  return m;
}

// Nine crates packed 2.2 m apart (0.6 m gaps) on `surface`, dropped from 0.3 to 0.7 m, for
// `seconds`: how many sleep at the end, their fastest speed in the last tenth, and the wake's
// largest height over each tenth of the run.
struct packed_run {
  int asleep = 0;
  double late_speed = 0.0;
  std::vector<double> peaks;
};
packed_run packed(bool wake, std::shared_ptr<const fluid_surface> surface, double seconds,
                  double apart = 2.2) {
  world w(jolt_config());
  field f = wake_water(wake);
  f.surface = std::move(surface);
  const field_id water = w.add_field(f);
  std::vector<body_id> ids;
  for (int j = 0; j < 3; ++j)
    for (int i = 0; i < 3; ++i)
      ids.push_back(
          w.add_body(crate_at(apart * (i - 1), apart * (j - 1), 0.3 + 0.05 * (3 * j + i))));
  const int steps = static_cast<int>(seconds / w.config().fixed_dt + 0.5);
  packed_run out;
  out.peaks.assign(10, 0.0);
  for (int s = 0; s < steps; ++s) {
    w.step();
    const int tenth = std::min(9, 10 * s / steps);
    if (wake && s % 10 == 0)
      out.peaks[tenth] = std::max(out.peaks[tenth], wake_peak(w, water, 0.0, 0.0));
    if (tenth == 9)
      for (const body_id id : ids) {
        const vec3 v = w.state(id).linear_velocity;
        out.late_speed = std::max(out.late_speed, std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z));
      }
  }
  for (const body_id id : ids)
    out.asleep += w.state(id).active ? 0 : 1;
  return out;
}

} // namespace

// A restore that fails part-way through the wake's section (a truncated state) leaves the wake as
// it was, and the world steps on from it (no state left half-read, no reader of freed states).
TEST(PhysicsFieldWake, AFailedRestoreLeavesTheWakeAsItWas) {
  REQUIRE_JOLT();
  world w(jolt_config());
  const field_id water = w.add_field(wake_water(true));
  w.add_body(crate_at(0.0, 0.0, 1.2));
  w.add_body(crate_at(4.0, 1.0, 0.0));
  w.step(150);
  std::vector<std::uint8_t> saved = w.save();
  w.step(60);
  const wake_info before = w.field_wake_info(water);
  const double h0 = w.field_wake_height(water, 0.0, 3.0), h1 = w.field_wake_height(water, 3.0, 0.0);
  ASSERT_GT(before.windows, 0);
  saved.resize(saved.size() - 100); // into the wake's last state
  EXPECT_THROW(w.restore(saved), cvc::physics_error);
  const wake_info after = w.field_wake_info(water);
  EXPECT_EQ(after.windows, before.windows);
  EXPECT_EQ(after.sources, before.sources);
  EXPECT_EQ(after.x0, before.x0);
  EXPECT_EQ(w.field_wake_height(water, 0.0, 3.0), h0);
  EXPECT_EQ(w.field_wake_height(water, 3.0, 0.0), h1);
  w.step(60); // reads the wake's states: they must be whole
  EXPECT_EQ(w.field_wake_info(water).windows, before.windows);
}

// A saved window placed absurdly far off (a corrupt state) is refused: moving it from there would
// overflow its cells' arithmetic.
TEST(PhysicsFieldWake, AWindowOutOfRangeIsRefused) {
  REQUIRE_JOLT();
  world w(jolt_config());
  w.add_field(wake_water(true));
  w.add_body(crate_at(0.0, 0.0, 1.2));
  w.step(20);
  std::vector<std::uint8_t> saved = w.save();
  static const char magic[] = "CVCWAKE2";
  const auto at = std::search(saved.begin(), saved.end(), magic, magic + 8);
  ASSERT_NE(at, saved.end());
  // After the magic: the field's id (4 bytes), its parameters (3 ints, 7 doubles), its shore (a
  // flag and a digest, 9), placed (1).
  const std::size_t ci = std::size_t(at - saved.begin()) + 8 + 4 + 12 + 56 + 9 + 1;
  ASSERT_LE(ci + 8, saved.size());
  EXPECT_NO_THROW(w.restore(saved));
  const std::int64_t away = std::int64_t(1) << 62;
  std::memcpy(saved.data() + ci, &away, sizeof(away));
  EXPECT_THROW(w.restore(saved), cvc::physics_error);
}

// A state saved under one wake does not restore into a world with another: every parameter
// counts, not only the window's cells.
TEST(PhysicsFieldWake, AStateFromAnotherWakeIsRefused) {
  REQUIRE_JOLT();
  const auto make = [](world &w, double depth, double period) {
    field f = wake_water(true);
    f.wake.depth = depth;
    f.wake.period = period;
    w.add_field(f);
    w.add_body(crate_at(0.0, 0.0, 1.2));
  };
  world a(jolt_config());
  make(a, 5.0, 1.0 / 30);
  a.step(50);
  const std::vector<std::uint8_t> saved = a.save();
  world same(jolt_config());
  make(same, 5.0, 1.0 / 30);
  EXPECT_NO_THROW(same.restore(saved));
  world deep(jolt_config());
  make(deep, 0.0, 1.0 / 30);
  EXPECT_THROW(deep.restore(saved), cvc::physics_error) << "another depth";
  world slow(jolt_config());
  make(slow, 5.0, 1.0 / 15);
  EXPECT_THROW(slow.restore(saved), cvc::physics_error) << "another period";
}

// The window goes where it holds the most: two boats 100 m apart (a window of 64 m holds one, not
// the point between them), and a cluster of five crates over one boat 60 m from them.
TEST(PhysicsFieldWake, TheWindowHoldsTheBodies) {
  REQUIRE_JOLT();
  {
    world w(jolt_config());
    const field_id water = w.add_field(wake_water(true));
    w.add_body(boat_at(-50.0, 0.0));
    w.add_body(boat_at(50.0, 0.0));
    w.step(10);
    const wake_info info = w.field_wake_info(water);
    std::printf("[wake] two boats 100 m apart: the window at x %.0f, %zu source(s), %d outside\n",
                info.x0, info.sources.size(), info.outside);
    EXPECT_EQ(info.sources.size(), 1u) << "one boat in the window, not neither";
    EXPECT_EQ(info.outside, 1);
    // A restore brings back what the last wave step counted outside, not 0 until the next one.
    world again(jolt_config());
    const field_id water2 = again.add_field(wake_water(true));
    again.add_body(boat_at(-50.0, 0.0));
    again.add_body(boat_at(50.0, 0.0));
    again.restore(w.save());
    EXPECT_EQ(again.field_wake_info(water2).outside, 1) << "restored";
  }
  {
    world w(jolt_config());
    const field_id water = w.add_field(wake_water(true));
    w.add_body(boat_at(60.0, 0.0));
    for (int k = 0; k < 5; ++k) {
      body_desc d = crate_at(2.5 * (k % 3) - 2.5, 2.5 * (k / 3), 0.0);
      d.allow_sleeping = false;
      w.add_body(d);
    }
    w.step(10);
    const wake_info info = w.field_wake_info(water);
    std::printf("[wake] a boat 60 m from five crates: the window at x %.0f, %zu source(s)\n",
                info.x0, info.sources.size());
    EXPECT_EQ(info.sources.size(), 5u) << "the crates' cluster, not the lone boat";
  }
}

// More floating bodies than slots: six crates afloat at rest round the window's middle, a wake of
// four sources. Once they sleep they hold no slot, so a boat that comes later makes waves, and its
// waves wake the crates and push them. (When a sleeping body kept its slot, the crates held all
// four: the boat made no waves, and the crates slept on.)
TEST(PhysicsFieldWake, SleepingBodiesLeaveTheirSlotsToABoat) {
  REQUIRE_JOLT();
  world w(jolt_config());
  field f = wake_water(true);
  f.wake.sources = 4;
  const field_id water = w.add_field(f);
  std::vector<body_id> crates;
  for (int k = 0; k < 6; ++k)
    crates.push_back(w.add_body(crate_at(4.0 * (k % 3) - 4.0, k < 3 ? -2.0 : 2.0, 0.0)));
  const body_id boat = w.add_body(boat_at(-42.0, 0.0));
  const auto asleep = [&] {
    int n = 0;
    for (const body_id id : crates)
      n += w.state(id).active ? 0 : 1;
    return n;
  };
  // They settle, sleep and their waves die away.
  int settle = 0;
  for (; settle < 12000 && (asleep() < 6 || wake_peak(w, water, 0.0, 0.0) > 1e-5); ++settle)
    w.step();
  ASSERT_EQ(asleep(), 6) << "the crates settle and sleep";
  ASSERT_LE(wake_peak(w, water, 0.0, 0.0), 1e-5) << "and their waves die away";
  std::vector<vec3> rest;
  for (const body_id id : crates)
    rest.push_back(w.state(id).position);
  // The boat comes on toward them under a steady thrust, into the window, and brakes 20 m short of
  // them (it never touches one).
  bool sourced = false;
  int woke = 0;
  double moved = 0.0;
  double furthest = -1e9;
  for (int s = 0; s < 4000; ++s) {
    const body_state b = w.state(boat);
    furthest = std::max(furthest, b.position.x);
    if (b.position.x < -26.0)
      w.add_force(boat, {1500.0, 0.0, 0.0});
    else
      w.add_force(boat, {-3000.0 * b.linear_velocity.x, -3000.0 * b.linear_velocity.y, 0.0});
    w.step();
    for (const body_id id : w.field_wake_info(water).sources)
      sourced = sourced || id == boat;
    woke = std::max(woke, 6 - asleep());
  }
  for (std::size_t k = 0; k < crates.size(); ++k) {
    const vec3 p = w.state(crates[k]).position;
    moved = std::max(moved, std::sqrt((p.x - rest[k].x) * (p.x - rest[k].x) +
                                      (p.y - rest[k].y) * (p.y - rest[k].y) +
                                      (p.z - rest[k].z) * (p.z - rest[k].z)));
  }
  std::printf("[wake] six crates asleep and still after %.1f s, four slots: the boat %s (its bow "
              "as far as x %.1f); over 40 s, up to %d crate(s) awake, the farthest moved %.4f m\n",
              settle * w.config().fixed_dt, sourced ? "made waves" : "made none", furthest + 2.0,
              woke, moved);
  EXPECT_LT(furthest + 2.0, -6.0) << "the boat never reaches a crate";
  EXPECT_TRUE(sourced) << "the boat takes a slot the sleepers gave up";
  EXPECT_GT(woke, 0) << "its waves wake the crates";
  EXPECT_GT(moved, 0.005) << "its waves push the crates";
}

// Packed bodies (0.6 m apart) on calm water: their waves die down (they leave the window slowly,
// through its sponge), and the bodies fall asleep with a wake as they do without one, if later:
// within 120 s, where without one they sleep within 3. (The swell's half is a test of its own,
// PackedBodiesStayBoundedInASwell, so each has its own ctest budget: a Debug build takes minutes
// over either.)
TEST(PhysicsFieldWake, PackedBodiesStayBoundedAndSleep) {
  REQUIRE_JOLT();
  const auto still = std::make_shared<still_surface>(0.0);
  const packed_run off = packed(false, still, 120.0), on = packed(true, still, 120.0);
  std::printf("[wake] nine packed crates on calm water, 120 s: asleep %d of 9 without a wake, %d "
              "with one; fastest in the last 12 s %.4f / %.4f m/s; the wake's peak by tenths:",
              off.asleep, on.asleep, off.late_speed, on.late_speed);
  for (double p : on.peaks)
    std::printf(" %.4f", p);
  std::printf("\n");
  EXPECT_EQ(off.asleep, 9) << "every crate sleeps without a wake";
  EXPECT_EQ(on.asleep, off.asleep) << "the wake keeps no crate awake";
  EXPECT_LT(on.peaks[9], 0.05 * std::max(on.peaks[0], on.peaks[1])) << "the waves die down";
  // Each body cuts its columns against the others' waves, so one riding another's wave does not
  // send it on again: the first waves stay under 0.5 m (cut against the bare water they reached
  // 0.82 m, against 0.35 m now).
  EXPECT_LT(on.peaks[0], 0.5);
}

// The packed crates in a swell: no crate sleeps, with a wake or without, and the waves they
// scatter do not grow.
TEST(PhysicsFieldWake, PackedBodiesStayBoundedInASwell) {
  REQUIRE_JOLT();
  const auto swell = std::make_shared<wave_train_surface>(
      0.0, 0.0, std::vector<wave_train>{{0.25, 5.0, 0.3, 0.0}});
  const packed_run soff = packed(false, swell, 60.0), son = packed(true, swell, 60.0);
  std::printf("[wake] in a 0.25 m, 5 s swell, 60 s: asleep %d / %d; fastest late %.3f / %.3f m/s; "
              "the wake's peak by tenths:",
              soff.asleep, son.asleep, soff.late_speed, son.late_speed);
  for (double p : son.peaks)
    std::printf(" %.4f", p);
  std::printf("\n");
  EXPECT_EQ(soff.asleep, 0) << "no crate sleeps in the swell";
  EXPECT_EQ(son.asleep, 0) << "nor with a wake";
  double early = 0.0, late = 0.0;
  for (int k = 1; k < 5; ++k)
    early = std::max(early, son.peaks[k]);
  for (int k = 5; k < 10; ++k)
    late = std::max(late, son.peaks[k]);
  EXPECT_LT(late, 2.0 * early + 1e-3) << "the scattered waves do not grow";
  // Each body cuts its columns against the sea and the others' waves, so one riding another's
  // wave does not send it on again: the waves among the crates stay under 0.25 m (cut against the
  // bare sea they reached 0.32 m).
  EXPECT_LT(std::max(early, son.peaks[0]), 0.25);
  EXPECT_LT(son.late_speed, 2.0 * soff.late_speed + 0.05);
}

// A world whose bodies all sleep reads no surface for its wake once the waves have died away, as
// it reads none without one.
TEST(PhysicsFieldWake, ASleepingWorldReadsNoSurface) {
  REQUIRE_JOLT();
  auto surface = std::make_shared<counted_still>();
  world w(jolt_config());
  field f = wake_water(true);
  f.surface = surface;
  w.add_field(f);
  const body_id a = w.add_body(crate_at(0.0, 0.0, 0.0));
  const body_id b = w.add_body(crate_at(4.0, 0.0, 0.0));
  w.step(6000);
  ASSERT_FALSE(w.state(a).active);
  ASSERT_FALSE(w.state(b).active);
  surface->reads = 0;
  w.step(100);
  std::printf("[wake] two crates asleep: the surface read %llu times in 100 steps\n",
              static_cast<unsigned long long>(surface->reads.load()));
  EXPECT_EQ(surface->reads.load(), 0u);
}

// The cost of a wake on a real sea (three cascades on 3 field workers): the fields' push a step
// with and without one, over six crates and a boat that never sleep; three runs of each, in turn,
// and their medians.
TEST(PhysicsFieldWake, TheWakesCostOnTheSea) {
  REQUIRE_JOLT();
  const auto sea = three_cascade_sea();
  std::vector<double> ms[2];
  std::size_t sources = 0;
  for (int run = 0; run < 6; ++run) {
    const int k = run % 2;
    world_config c = jolt_config();
    c.field_threads = 3;
    world w(c);
    field f = wake_water(k == 1);
    f.surface = std::make_shared<ocean_surface>(sea);
    const field_id water = w.add_field(f);
    for (int i = 0; i < 6; ++i) {
      body_desc d = crate_at(3.0 * (i % 3) - 3.0, 3.0 * (i / 3) - 1.5, 0.0);
      d.allow_sleeping = false;
      w.add_body(d);
    }
    w.add_body(boat_at(0.0, 8.0));
    w.set_field_time(17.0);
    w.step(30); // the first frames and the workers
    const int steps = 300;
    double sum = 0.0;
    for (int s = 0; s < steps; ++s)
      sum += w.step().fields_ms;
    ms[k].push_back(sum / steps);
    if (k == 1)
      sources = w.field_wake_info(water).sources.size();
  }
  for (std::vector<double> &m : ms)
    std::sort(m.begin(), m.end());
  std::printf("[wake] on a three-cascade sea with 3 field workers: the fields' push %.3f ms a step "
              "without a wake (%.3f-%.3f), %.3f ms with one (%.3f-%.3f; %zu sources)\n",
              ms[0][1], ms[0][0], ms[0][2], ms[1][1], ms[1][0], ms[1][2], sources);
  EXPECT_EQ(sources, 7u);
}

// A world with a wake gives the same bits on every platform: a crate dropped beside another and a
// boat under way, in a swell, with a wake, its state hash after 400 steps pinned (measured on
// Windows, MSVC; CI checks Linux, macOS and the others).
TEST(PhysicsFieldWake, CrossPlatformGoldenHash) {
  REQUIRE_JOLT();
  world w(jolt_config());
  field f = wake_water(true);
  f.surface = std::make_shared<wave_train_surface>(
      0.0, 20.0, std::vector<wave_train>{{0.2, 6.0, 0.4, 0.0}, {0.05, 2.5, 1.2, 1.0}});
  const field_id water = w.add_field(f);
  w.add_body(crate_at(0.0, 0.0, 1.2));
  w.add_body(crate_at(4.0, 1.0, 0.0));
  const body_id boat = w.add_body(boat_at(-8.0, -6.0));
  for (int i = 0; i < 400; ++i) {
    w.add_force(boat, {1500.0, 300.0, 0.0});
    w.step();
  }
  const std::uint64_t h = w.state_hash();
  std::printf("[wake] golden: %zu sources, state hash 0x%016llx\n",
              w.field_wake_info(water).sources.size(), static_cast<unsigned long long>(h));
  EXPECT_EQ(w.field_wake_info(water).sources.size(), 3u);
  // Re-pinned for depth (the others' waves' motion and push decay below the top); before it
  // 0x7a5a40b81d1d2433, and first 0x8052b235300e8e23.
  EXPECT_EQ(h, 0xc72e26b4ef0f764aull) << std::hex << "wake world state hash 0x" << h;
}

// --- wakes: shores, depth and the water at a point (C5's gaps; C6) -------------------------------

namespace {

// A coast for a wake: a shore grid of 1 m over (-64, -64) .. (64, 64) whose signed distance to the
// shore is dist(x, y) (+ offshore), the ground's slope (rise over run) `slope` everywhere.
std::shared_ptr<const shore_grid_mask> coast(const std::function<double(double, double)> &dist,
                                             double slope) {
  auto g = std::make_shared<cvc::ocean::shore_grid>();
  g->rows = g->cols = 129;
  g->min_x = g->min_y = -64.0;
  g->cell = 1.0;
  g->depth.resize(g->count());
  g->shore_m.resize(g->count());
  for (int r = 0; r < g->rows; ++r)
    for (int c = 0; c < g->cols; ++c) {
      const double s = dist(g->x(c), g->y(r));
      const std::size_t i = std::size_t(r) * std::size_t(g->cols) + std::size_t(c);
      g->shore_m[i] = float(s);
      g->depth[i] = float(s > 0.0 ? std::min(8.0, 0.5 + s) : -1.0);
    }
  return std::make_shared<shore_grid_mask>(g, std::vector<float>(g->count(), float(slope)));
}

// A crate dropped at the origin from 1.2 m on wake_water with `shore` (null: open water), for
// `steps` steps of 0.01 s: the wake's height at each probe after every step, and the crate's path.
struct ring_run {
  std::vector<std::vector<double>> h; // [probe][step]
  std::vector<vec3> path;
  double fields_ms = 0.0; // the fields' push a step, the mean
};
ring_run ring_by(std::shared_ptr<const wake_shore> shore, const std::vector<vec3> &probes,
                 int steps) {
  world w(jolt_config());
  field f = wake_water(true);
  f.wake.shore = std::move(shore);
  const field_id water = w.add_field(f);
  const body_id b = w.add_body(crate_at(0.0, 0.0, 1.2));
  ring_run out;
  out.h.assign(probes.size(), {});
  for (int i = 0; i < steps; ++i) {
    out.fields_ms += w.step().fields_ms / steps;
    out.path.push_back(w.state(b).position);
    for (std::size_t p = 0; p < probes.size(); ++p)
      out.h[p].push_back(w.field_wake_height(water, probes[p].x, probes[p].y));
  }
  return out;
}

double rms_of(const std::vector<double> &a, std::size_t from = 0) {
  double s = 0.0;
  for (std::size_t i = from; i < a.size(); ++i)
    s += a[i] * a[i];
  return a.size() > from ? std::sqrt(s / double(a.size() - from)) : 0.0;
}

double largest(const std::vector<double> &a) {
  double m = 0.0;
  for (const double v : a)
    m = std::max(m, std::fabs(v));
  return m;
}

double correlation_of(const std::vector<double> &a, const std::vector<double> &b) {
  double ab = 0.0, aa = 0.0, bb = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    ab += a[i] * b[i];
    aa += a[i] * a[i];
    bb += b[i] * b[i];
  }
  return aa > 0.0 && bb > 0.0 ? ab / std::sqrt(aa * bb) : 0.0;
}

std::vector<double> minus(const std::vector<double> &a, const std::vector<double> &b) {
  std::vector<double> d(a.size());
  for (std::size_t i = 0; i < a.size(); ++i)
    d[i] = a[i] - b[i];
  return d;
}

vec3 crossed(const vec3 &a, const vec3 &b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

// v turned by the unit quaternion q.
vec3 turned(const quat &q, const vec3 &v) {
  const vec3 u{q.x, q.y, q.z};
  const vec3 t = crossed(u, v) * 2.0;
  return v + t * q.w + crossed(u, t);
}

// A launch with a propeller and a rudder 0.2 m under its waterline at the stern (the skiff of
// physics_boat_test on box_hull): 4 x 1.5 x 0.6 m, 900 kg.
boat_desc wake_skiff() {
  boat_desc d;
  const vec3 half{2.0, 0.75, 0.3};
  d.hull.shape = box{half};
  d.hull.hull = box_hull(half);
  d.hull.mass = 900.0;
  d.hull.linear_damping = 0.0;
  d.hull.angular_damping = 0.0;
  d.hull.allow_sleeping = false;
  d.propeller.position = {-2.1, 0.0, -0.35};
  d.propeller.max_thrust = 2000.0;
  d.propeller.max_inflow_speed = 8.0;
  d.rudder.position = {-2.3, 0.0, -0.35};
  d.rudder.area = 0.12;
  d.actuator_time_constant = 0.5;
  return d;
}

} // namespace

// C6: a wake's waves reflect off a steep shore crest for crest and soak into a beach, and never run
// over the land. A crate dropped 6 m from a straight coast (land from x = 6, the no-flux face at
// x = 5.5), on open water, under a cliff and behind a beach: between the drop and the coast, the
// cliff's run less the open water's is the reflection, the open water's ring at the image's
// distance (9 m) in shape and sign; the beach's is a fraction of it. On the land the wake is 0 at
// every step. The crate never feels its own reflected waves: its path is the same in all three.
TEST(PhysicsFieldWakeShore, AWakeReflectsOffACliffAndSoaksIntoABeach) {
  REQUIRE_JOLT();
  const std::vector<vec3> probes{{2.0, 0.0, 0.0}, {-9.0, 0.0, 0.0},  {6.0, 0.0, 0.0},
                                 {8.0, 3.0, 0.0}, {12.0, -4.0, 0.0}, {20.0, 0.0, 0.0}};
  const auto wall = [](double x, double) { return 6.0 - x; };
  const int steps = 1600;
  const ring_run open = ring_by(nullptr, probes, steps);
  const ring_run cliff = ring_by(coast(wall, 1.0), probes, steps);
  const ring_run beach = ring_by(coast(wall, 0.0), probes, steps);
  const std::vector<double> back = minus(cliff.h[0], open.h[0]);
  const std::vector<double> soaked = minus(beach.h[0], open.h[0]);
  const double image = rms_of(open.h[1]);
  const double r = correlation_of(back, open.h[1]);
  std::printf("[wake shore] 2 m from the drop: the cliff's reflection %.4f m rms against the open "
              "ring's %.4f m at 9 m (%.0f %%), correlated %.3f; the beach's %.4f m (%.0f %% of "
              "the cliff's)\n",
              rms_of(back), image, 100.0 * rms_of(back) / image, r, rms_of(soaked),
              100.0 * rms_of(soaked) / rms_of(back));
  double over_open = 0.0, over_land = 0.0;
  for (std::size_t p = 2; p <= 5; ++p) {
    over_open = std::max(over_open, largest(open.h[p]));
    over_land = std::max(over_land, std::max(largest(cliff.h[p]), largest(beach.h[p])));
  }
  std::printf("[wake shore] on the land: up to %.4f m on open water, %.4f m under the cliff or "
              "behind the beach. The fields' push a step: %.3f ms on open water, %.3f ms with the "
              "cliff (the no-flux step)\n",
              over_open, over_land, open.fields_ms, cliff.fields_ms);
  EXPECT_GT(r, 0.8) << "a crest comes back a crest";
  EXPECT_GT(rms_of(back), 0.6 * image);
  EXPECT_LT(rms_of(back), 1.4 * image);
  EXPECT_LT(rms_of(soaked), 0.5 * rms_of(back)) << "the beach takes most of it";
  EXPECT_GT(over_open, 1e-3) << "without a shore the ring runs over the land";
  EXPECT_EQ(over_land, 0.0) << "nothing crosses the coast";
  for (std::size_t i = 0; i < open.path.size(); ++i) {
    ASSERT_EQ(cliff.path[i].x, open.path[i].x) << "step " << i;
    ASSERT_EQ(cliff.path[i].z, open.path[i].z) << "step " << i;
    ASSERT_EQ(beach.path[i].z, open.path[i].z) << "step " << i;
  }
}

// C6: a pond inside the window reflects off all its banks, and its waves stay in it. A pond of 7 m
// round the drop, ringed by 8 m of land and the sea past that: halfway to each bank the ring
// keeps coming back long after it has left open water; the land holds no wave at any step; the
// sea past the land gets almost none.
TEST(PhysicsFieldWakeShore, APondReflectsOffAllItsBanks) {
  REQUIRE_JOLT();
  std::vector<vec3> probes{{4.0, 0.0, 0.0}, {-4.0, 0.0, 0.0}, {0.0, 4.0, 0.0}, {0.0, -4.0, 0.0}};
  for (int k = 0; k < 8; ++k) { // the land
    const double a = k * 0.7853981633974483;
    probes.push_back({9.0 * std::cos(a), 9.0 * std::sin(a), 0.0});
    probes.push_back({13.0 * std::cos(a), 13.0 * std::sin(a), 0.0});
  }
  for (int k = 0; k < 4; ++k) { // the sea
    const double a = 0.4 + k * 1.5707963267948966;
    probes.push_back({20.0 * std::cos(a), 20.0 * std::sin(a), 0.0});
  }
  const auto pond = [](double x, double y) {
    const double r = std::sqrt(x * x + y * y);
    return r < 11.0 ? 7.0 - r : r - 15.0;
  };
  const int steps = 2000, late = 800; // the last 12 s
  const ring_run open = ring_by(nullptr, probes, steps);
  const ring_run banked = ring_by(coast(pond, 1.0), probes, steps);
  double peak = 0.0, weakest = 1e9;
  for (int p = 0; p < 4; ++p) {
    peak = std::max(peak, largest(banked.h[p]));
    weakest = std::min(weakest, rms_of(banked.h[p], late) / rms_of(open.h[p], late));
  }
  double land = 0.0, sea = 0.0;
  for (std::size_t p = 4; p < 20; ++p)
    land = std::max(land, largest(banked.h[p]));
  for (std::size_t p = 20; p < probes.size(); ++p)
    sea = std::max(sea, largest(banked.h[p]));
  std::printf("[wake shore] a pond of 7 m: late in the run its waves halfway to the banks are at "
              "least %.1f times open water's; the land holds %.4f m; the sea past 8 m of land "
              "%.5f m (%.2f %% of the pond's %.3f m)\n",
              weakest, land, sea, 100.0 * sea / peak, peak);
  EXPECT_GT(weakest, 3.0) << "every bank sends the ring back";
  EXPECT_EQ(land, 0.0);
  EXPECT_LT(sea, 0.03 * peak);
}

// The waves' motion decays with depth (e^{-k d}, each wavenumber on its own): two neutrally buoyant
// 1 m cubes 5 m either side of a crate dropped from 1.2 m, one just under the surface and one 3.5
// m down, the ring the same toward each. The deep one is pushed a small part as hard, and moves
// less (by less than the push: what reaches it is the ring's longer waves, which decay least and
// push longest); read at the surface's values (as before) the two moved alike. The water read at
// a point decays too.
TEST(PhysicsFieldWakeDepth, ADeepBodyFeelsLessOfTheWavesThanOneAfloat) {
  REQUIRE_JOLT();
  world w(jolt_config());
  w.add_field(wake_water(true));
  w.add_body(crate_at(0.0, 0.0, 1.2));
  const auto cube = [](double x, double z) {
    body_desc d;
    d.shape = box{{0.5, 0.5, 0.5}};
    d.density = 1000.0;
    d.position = {x, 0.0, z};
    d.linear_damping = 0.0;
    d.allow_sleeping = false;
    return d;
  };
  const body_id shallow = w.add_body(cube(5.0, -0.65)), deep = w.add_body(cube(-5.0, -4.0));
  const vec3 s0 = w.state(shallow).position, d0 = w.state(deep).position;
  double moved[2] = {0.0, 0.0}, pushed[2] = {0.0, 0.0}, top = 0.0, down = 0.0;
  for (int i = 0; i < 1000; ++i) {
    w.step();
    const vec3 s = w.state(shallow).position, d = w.state(deep).position;
    moved[0] = std::max(moved[0], std::hypot(s.x - s0.x, s.y - s0.y));
    moved[1] = std::max(moved[1], std::hypot(d.x - d0.x, d.y - d0.y));
    const vec3 fs = w.field_load_on(shallow).force, fd = w.field_load_on(deep).force;
    pushed[0] = std::max(pushed[0], std::hypot(fs.x, fs.y));
    pushed[1] = std::max(pushed[1], std::hypot(fd.x, fd.y));
    // The water 3 m off the drop, at its top and 3 m below it.
    const double surface = w.water_at({3.0, 0.0, 5.0}).depth + 5.0;
    const vec3 a = w.water_at({3.0, 0.0, surface - 0.02}).velocity;
    const vec3 b = w.water_at({3.0, 0.0, surface - 3.0}).velocity;
    top = std::max(top, std::hypot(a.x, a.z));
    down = std::max(down, std::hypot(b.x, b.z));
  }
  std::printf("[wake depth] a cube under the surface moved %.4f m (push up to %.2f N); one 3.5 m "
              "down %.4f m (%.2f N). The water 3 m off the drop: up to %.4f m/s at its top, %.4f "
              "m/s 3 m down\n",
              moved[0], pushed[0], moved[1], pushed[1], top, down);
  EXPECT_GT(moved[0], 0.01) << "the ring moves the shallow cube";
  EXPECT_LT(moved[1], 0.5 * moved[0]);
  EXPECT_LT(pushed[1], 0.25 * pushed[0]);
  EXPECT_LT(down, 0.25 * top);
}

// The wake in world::water_at: a moored launch's propeller, in the wake of another launch passing
// at full throttle 5 m off, reads that wake's orbital motion (none without a wake: the water is
// still), and the launch's own step reads the same (its inflow is the propeller's motion through
// it). The launch under way finds its own waves at its propeller in water_at without a body, never
// as it feels them.
TEST(PhysicsFieldWakeWaterAt, APropellerInAnotherBoatsWakeReadsItsOrbitalMotion) {
  REQUIRE_JOLT();
  double reads[2] = {0.0, 0.0}, own_all = 0.0, own_felt = 0.0, worst = 0.0;
  for (int k = 0; k < 2; ++k) {
    world w(jolt_config());
    w.add_field(wake_water(k == 1));
    const boat_id moored = w.add_boat(wake_skiff(), {0.0, 0.0, 0.15});
    const boat_id runner = w.add_boat(wake_skiff(), {-30.0, 5.0, 0.15});
    w.set_boat_controls(runner, {1.0, 0.0});
    const boat_desc d = wake_skiff();
    for (int i = 0; i < 2000; ++i) {
      const boat_state before = w.boat_state(moored);
      w.step();
      const body_state &s = before.body;
      const vec3 prop = s.position + turned(s.rotation, d.propeller.position);
      const vec3 water = w.water_at(prop, before.hull).velocity;
      reads[k] = std::max(reads[k], std::sqrt(dot(water, water)));
      // Its step read the water there with the wake as it is after the step's fields.
      const vec3 dir = turned(s.rotation, {1.0, 0.0, 0.0});
      const vec3 at = s.linear_velocity + crossed(s.angular_velocity, prop - s.position);
      worst = std::max(worst, std::fabs(dot(at - water, dir) - w.boat_state(moored).inflow));
      if (k == 1) {
        const boat_state r = w.boat_state(runner);
        const vec3 rp = r.body.position + turned(r.body.rotation, d.propeller.position);
        const vec3 all = w.water_at(rp).velocity, felt = w.water_at(rp, r.hull).velocity;
        own_all = std::max(own_all, std::sqrt(dot(all, all)));
        own_felt = std::max(own_felt, std::sqrt(dot(felt, felt)));
      }
    }
  }
  std::printf("[wake water_at] the moored launch's propeller reads up to %.4f m/s of the "
              "runner's wake (%.4f without a wake); the runner's own propeller %.4f m/s of all the "
              "waves, %.4f of the others'; the inflow off by at most %.2g m/s\n",
              reads[1], reads[0], own_all, own_felt, worst);
  EXPECT_EQ(reads[0], 0.0);
  EXPECT_GT(reads[1], 0.02) << "the propeller is in the runner's wake";
  EXPECT_LT(own_felt, 0.25 * own_all) << "a boat does not read its own waves";
  EXPECT_LT(worst, 1e-9) << "the boat's step reads what water_at reads";
}

// A shore is part of the wake: a save restores into a world with the same coast (another object of
// one digest) and runs on to the same hash; a world with another coast, or with none, refuses it;
// set_field with the same coast keeps the waves, with another starts afresh.
TEST(PhysicsFieldWakeShore, AShoreIsPartOfTheWakesState) {
  REQUIRE_JOLT();
  const auto wall = [](double x, double) { return 6.0 - x; };
  const auto make = [](std::shared_ptr<const wake_shore> shore, world &w) {
    field f = wake_water(true);
    f.wake.shore = std::move(shore);
    const field_id id = w.add_field(f);
    w.add_body(crate_at(0.0, 0.0, 1.2));
    w.add_body(crate_at(3.0, 1.0, 0.0));
    return id;
  };
  world a(jolt_config());
  const field_id water = make(coast(wall, 1.0), a);
  a.step(150);
  const std::vector<std::uint8_t> saved = a.save();
  a.step(200);
  const std::uint64_t straight = a.state_hash();
  world b(jolt_config());
  make(coast(wall, 1.0), b);
  b.restore(saved);
  b.step(200);
  EXPECT_EQ(b.state_hash(), straight) << "the same coast, another object";
  world other(jolt_config());
  make(coast([](double x, double) { return 7.0 - x; }, 1.0), other);
  EXPECT_THROW(other.restore(saved), cvc::physics_error);
  world none(jolt_config());
  make(nullptr, none);
  EXPECT_THROW(none.restore(saved), cvc::physics_error);
  EXPECT_THROW(a.restore(none.save()), cvc::physics_error);
  const double before = a.field_wake_height(water, 2.0, 0.0);
  ASSERT_NE(before, 0.0);
  field f = wake_water(true);
  f.wake.shore = coast(wall, 1.0);
  a.set_field(water, f);
  EXPECT_EQ(a.field_wake_height(water, 2.0, 0.0), before);
  f.wake.shore = coast(wall, 0.0);
  a.set_field(water, f);
  EXPECT_EQ(a.field_wake_height(water, 2.0, 0.0), 0.0);
}

// A state saved before shores and depths (CVCWAKE1: no shore digest, no count of depths) restores
// into a world whose wake has no shore, and steps on; into one with a shore it is refused.
TEST(PhysicsFieldWakeShore, AStateFromBeforeShoresRestores) {
  REQUIRE_JOLT();
  const auto make = [](std::shared_ptr<const wake_shore> shore, world &w) {
    field f = wake_water(true);
    f.wake.shore = std::move(shore);
    w.add_field(f);
    w.add_body(crate_at(0.0, 0.0, 1.2));
    w.add_body(crate_at(4.0, 1.0, 0.0));
  };
  world w(jolt_config());
  make(nullptr, w);
  w.step(150);
  const std::vector<std::uint8_t> v2 = w.save();
  static const char magic[] = "CVCWAKE2";
  const auto at = std::search(v2.begin(), v2.end(), magic, magic + 8);
  ASSERT_NE(at, v2.end());
  // Version 1's layout: the magic's last byte, no shore (9 bytes after the parameters) and no
  // count of depths (4 bytes after the bodies outside the window).
  const std::size_t m = std::size_t(at - v2.begin());
  const std::size_t shore = m + 8 + 4 + 12 + 56, depths = shore + 9 + 1 + 8 + 8 + 8 + 4;
  std::vector<std::uint8_t> v1(v2.begin(), v2.begin() + std::ptrdiff_t(shore));
  v1.insert(v1.end(), v2.begin() + std::ptrdiff_t(shore + 9), v2.begin() + std::ptrdiff_t(depths));
  v1.insert(v1.end(), v2.begin() + std::ptrdiff_t(depths + 4), v2.end());
  v1[m + 7] = '1';
  world b(jolt_config());
  make(nullptr, b);
  EXPECT_NO_THROW(b.restore(v1));
  EXPECT_NO_THROW(b.step(50));
  world c(jolt_config());
  make(coast([](double x, double) { return 6.0 - x; }, 1.0), c);
  EXPECT_THROW(c.restore(v1), cvc::physics_error);
}

// The golden hash of a wake with a coast: two crates and a launch under way off a point with a
// steep face and gentler flanks, in a two-train swell, 400 steps. Measured on Windows; CI checks
// the other platforms.
TEST(PhysicsFieldWakeShore, CrossPlatformGoldenHash) {
  REQUIRE_JOLT();
  world w(jolt_config());
  field f = wake_water(true);
  f.surface = std::make_shared<wave_train_surface>(
      0.0, 20.0, std::vector<wave_train>{{0.2, 6.0, 0.4, 0.0}, {0.05, 2.5, 1.2, 1.0}});
  f.wake.shore = coast([](double x, double y) { return 7.0 - x - 0.2 * std::fabs(y); }, 0.4);
  const field_id water = w.add_field(f);
  w.add_body(crate_at(0.0, 0.0, 1.2));
  w.add_body(crate_at(4.0, 1.0, 0.0));
  const body_id boat = w.add_body(boat_at(-8.0, -6.0));
  for (int i = 0; i < 400; ++i) {
    w.add_force(boat, {1500.0, 300.0, 0.0});
    w.step();
  }
  const std::uint64_t h = w.state_hash();
  std::printf("[wake shore] golden: %zu sources, state hash 0x%016llx\n",
              w.field_wake_info(water).sources.size(), static_cast<unsigned long long>(h));
  EXPECT_EQ(w.field_wake_info(water).sources.size(), 3u);
  EXPECT_EQ(h, 0x89789ba92fbd1853ull) << std::hex << "wake world state hash 0x" << h;
}
