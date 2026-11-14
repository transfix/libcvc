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
#include <cstdint>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/exception.h>
#include <cvc/core/world_clock.h>
#include <cvc/ocean/h0.h>
#include <cvc/ocean/sea_surface.h>
#include <cvc/physics/ocean_surface.h>
#include <cvc/physics/world.h>
#include <cvc/volume/volume.h>
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
