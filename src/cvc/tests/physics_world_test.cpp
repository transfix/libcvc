/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Tests for cvc::physics::world.
//
// The null backend runs everywhere. The Jolt tests skip in a build without Jolt;
// with Jolt they are the P1 gates of docs/roadmap/NAV-VEHICLE-PHYSICS-ROADMAP.md:
//   (1) a box dropped on a height field rests at the sampled height within 1 cm;
//   (2) a contact reports the cell's material and μ, and μ decides stick vs slide;
//   (3) the same trace twice gives the same state hash;
//   (4) one thread and several give the same state hash;
//   (5) save at tick n, restore, re-run: the same state hash.

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/exception.h>
#include <cvc/core/world_clock.h>
#include <cvc/physics/world.h>
#include <cvc/physics/world_binding.h>
#include <cvc/state/state.h>
#include <gtest/gtest.h>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

using namespace cvc::physics;

namespace {

bool have_jolt() { return backend_available(backend::jolt); }

#define REQUIRE_JOLT()                                                                             \
  if (!have_jolt())                                                                                \
  GTEST_SKIP() << "this libcvc was built without Jolt"

world_config jolt_config() {
  world_config c;
  c.backend = backend::jolt;
  return c;
}

world_config null_config() {
  world_config c;
  c.backend = backend::null;
  return c;
}

// Gentle rolling terrain, centred on the origin: slopes under 7 degrees, so a box
// with friction 1 never slides.
heightfield rolling_terrain(int n = 33, double cell = 0.5, vec3 origin = {}) {
  heightfield h;
  h.rows = h.cols = n;
  h.dx = h.dy = cell;
  const double half = 0.5 * (n - 1) * cell;
  h.origin = {origin.x - half, origin.y - half, origin.z};
  h.heights.resize(static_cast<size_t>(n) * n);
  for (int r = 0; r < n; ++r)
    for (int c = 0; c < n; ++c) {
      const double x = c * cell - half, y = r * cell - half;
      h.heights[static_cast<size_t>(r) * n + c] =
          static_cast<float>(0.3 * std::sin(x / 3.0) + 0.2 * std::cos(y / 4.0) + 0.02 * x);
    }
  return h;
}

// The cosine of the terrain's slope at (x, y), from the bilinear gradient.
double slope_cos(const heightfield &h, double x, double y) {
  const double e = 1e-3;
  const double hx = (bilinear_height(h, x + e, y) - bilinear_height(h, x - e, y)) / (2 * e);
  const double hy = (bilinear_height(h, x, y + e) - bilinear_height(h, x, y - e)) / (2 * e);
  return 1.0 / std::sqrt(1.0 + hx * hx + hy * hy);
}

double speed(const body_state &s) { return std::sqrt(dot(s.linear_velocity, s.linear_velocity)); }

// A scene with enough going on to exercise the solver: terrain, sixteen boxes and
// spheres dropped with spin, and scripted forces.
struct busy_scene {
  std::vector<body_id> movers;

  explicit busy_scene(world &w) {
    w.add_body({rolling_terrain(), motion_type::static_});
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j) {
        body_desc d;
        if ((i + j) % 2 == 0)
          d.shape = box{{0.2, 0.15, 0.1}};
        else
          d.shape = sphere{0.15};
        d.position = {1.2 * (i - 1.5), 1.2 * (j - 1.5), 1.0 + 0.3 * (i + j)};
        d.linear_velocity = {0.3 * (j - 1.5), -0.2 * (i - 1.5), 0.0};
        d.angular_velocity = {0.5 * i, -0.4 * j, 0.3};
        movers.push_back(w.add_body(d));
      }
  }

  // One scripted input per step, the same in every run.
  void drive(world &w, int step) const {
    const body_id b = movers[static_cast<size_t>(step) % movers.size()];
    w.add_force(b, {20.0 * std::sin(0.1 * step), 15.0 * std::cos(0.07 * step), 0.0});
    if (step % 37 == 0)
      w.add_impulse(movers[static_cast<size_t>(step / 37) % movers.size()], {0.0, 0.0, 2.0});
  }

  std::vector<std::uint64_t> run(world &w, int steps, int every) const {
    std::vector<std::uint64_t> hashes;
    for (int s = 0; s < steps; ++s) {
      drive(w, static_cast<int>(w.tick()));
      w.step();
      if ((s + 1) % every == 0)
        hashes.push_back(w.state_hash());
    }
    return hashes;
  }
};

} // namespace

// --- the facade and the null backend -------------------------------------------

TEST(PhysicsWorld, NullBackendIsAlwaysAvailable) {
  const std::vector<backend> all = available_backends();
  ASSERT_FALSE(all.empty());
  EXPECT_EQ(all.front(), backend::null);
  EXPECT_STREQ(backend_name(backend::null), "null");
  EXPECT_STREQ(backend_name(backend::jolt), "jolt");
  EXPECT_TRUE(backend_capabilities(backend::null) & cap_exact_save_restore);
  EXPECT_FALSE(backend_capabilities(backend::null) & cap_dynamics);
}

TEST(PhysicsWorld, JoltIsRefusedLoudlyWhenNotBuilt) {
  if (have_jolt())
    GTEST_SKIP() << "this build has Jolt";
  EXPECT_THROW(world w(jolt_config()), cvc::unsupported_exception);
}

TEST(PhysicsWorld, ConfigIsValidated) {
  world_config c = null_config();
  c.fixed_dt = 0.0;
  EXPECT_THROW(world w(c), cvc::physics_error);
  c = null_config();
  c.collision_steps = 0;
  EXPECT_THROW(world w(c), cvc::physics_error);
  c = null_config();
  c.threads = -1;
  EXPECT_THROW(world w(c), cvc::physics_error);
  c = null_config();
  c.threads = 2; // the null backend has no worker threads to give
  EXPECT_THROW(world w(c), cvc::unsupported_exception);
}

TEST(PhysicsWorld, MaterialsStartWithANeutralDefault) {
  world w(null_config());
  ASSERT_EQ(w.material_count(), 1u);
  EXPECT_EQ(w.get_material(0).friction, 1.0f);
  const material_id ice = w.add_material({"ice", 0.1f, 0.0f, 0.0f, 7});
  EXPECT_EQ(ice, 1);
  EXPECT_EQ(w.get_material(ice).nav_material, 7);
  EXPECT_THROW(w.add_material({"bad", -1.0f}), cvc::physics_error);
  EXPECT_THROW(w.add_material({"bad", 0.5f, 2.0f}), cvc::physics_error);
  EXPECT_THROW(w.get_material(99), cvc::physics_error);
}

TEST(PhysicsWorld, NullWorldKeepsBodiesStillAndMovesKinematicOnes) {
  world w(null_config());
  const body_id rock = w.add_body({box{}, motion_type::dynamic, {0, 0, 5}});
  body_desc mover{sphere{}, motion_type::kinematic, {0, 0, 0}};
  mover.linear_velocity = {1.0, 0.0, 0.0};
  const body_id m = w.add_body(mover);
  w.step(100);
  EXPECT_EQ(w.tick(), 100u);
  EXPECT_NEAR(w.time(), 1.0, 1e-12);
  EXPECT_DOUBLE_EQ(w.state(rock).position.z, 5.0); // no gravity in the null backend
  EXPECT_NEAR(w.state(m).position.x, 1.0, 1e-9);
  EXPECT_TRUE(w.state(m).active);
  w.move_kinematic(m, {3.0, 0.0, 0.0}, quat{});
  w.step();
  EXPECT_NEAR(w.state(m).position.x, 3.0, 1e-12);
  EXPECT_NEAR(w.state(m).linear_velocity.x, (3.0 - 1.0) / 0.01, 1e-6);
  EXPECT_THROW(w.move_kinematic(rock, {0, 0, 0}, quat{}), cvc::physics_error);
}

TEST(PhysicsWorld, BodyIdsAreGenerationChecked) {
  world w(null_config());
  const body_id a = w.add_body({sphere{}, motion_type::dynamic});
  w.remove_body(a);
  EXPECT_FALSE(w.contains(a));
  const body_id b = w.add_body({sphere{}, motion_type::dynamic}); // reuses a's slot
  EXPECT_NE(a, b);
  EXPECT_TRUE(w.contains(b));
  EXPECT_THROW(w.state(a), cvc::physics_error);
  EXPECT_THROW(w.remove_body(a), cvc::physics_error);
  EXPECT_THROW(w.state(body_id{}), cvc::physics_error);
}

TEST(PhysicsWorld, BadBodiesAreRefused) {
  world w(null_config());
  EXPECT_THROW(w.add_body({sphere{-1.0}}), cvc::physics_error);
  body_desc d{box{}};
  d.rotation = {2.0, 0.0, 0.0, 0.0};
  EXPECT_THROW(w.add_body(d), cvc::physics_error);
  d = body_desc{box{}};
  d.material = 42;
  EXPECT_THROW(w.add_body(d), cvc::physics_error);
  heightfield h = rolling_terrain(5);
  EXPECT_THROW(w.add_body({h, motion_type::dynamic}), cvc::physics_error); // must be static
  h.heights.pop_back();
  EXPECT_THROW(w.add_body({h, motion_type::static_}), cvc::physics_error);
  h = rolling_terrain(5);
  h.cell_materials.assign(16, 3); // palette has one entry
  EXPECT_THROW(w.add_body({h, motion_type::static_}), cvc::physics_error);
  triangle_mesh m;
  m.vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  m.indices = {0, 1, 5};
  EXPECT_THROW(w.add_body({m, motion_type::static_}), cvc::physics_error);
}

TEST(PhysicsWorld, NullSaveRestoreReproducesTheRun) {
  world w(null_config());
  body_desc mover{sphere{}, motion_type::kinematic};
  mover.linear_velocity = {0.5, -0.25, 0.1};
  mover.angular_velocity = {0.0, 0.0, 1.0};
  const body_id m = w.add_body(mover);
  w.step(10);
  const std::vector<std::uint8_t> saved = w.save();
  w.step(10);
  const std::uint64_t h = w.state_hash();
  const vec3 p = w.state(m).position;
  w.restore(saved);
  EXPECT_EQ(w.tick(), 10u);
  w.step(10);
  EXPECT_EQ(w.state_hash(), h);
  EXPECT_DOUBLE_EQ(w.state(m).position.x, p.x);
}

TEST(PhysicsWorld, RestoreRefusesAMismatchedWorld) {
  world a(null_config());
  a.add_body({sphere{}});
  const std::vector<std::uint8_t> saved = a.save();
  world b(null_config());
  EXPECT_THROW(b.restore(saved), cvc::physics_error); // no bodies
  std::vector<std::uint8_t> bad = saved;
  bad[0] = 'X';
  EXPECT_THROW(a.restore(bad), cvc::physics_error);
  bad = saved;
  bad.pop_back();
  EXPECT_THROW(a.restore(bad), cvc::physics_error);
}

TEST(PhysicsWorld, FollowTakesTheStepsDueOnTheClock) {
  cvc::world_clock::config cc;
  cc.fixed_dt = 0.01;
  cvc::world_clock clock(cc);
  world_config c = null_config();
  c.fixed_dt = 0.02; // two clock quanta per physics step
  world w(c);
  EXPECT_EQ(w.follow(clock), 0u); // anchors
  clock.advance_ticks(10);
  EXPECT_EQ(w.follow(clock), 5u);
  EXPECT_EQ(w.tick(), 5u);
  clock.advance_ticks(3);
  EXPECT_EQ(w.follow(clock), 1u); // tick 12 is due, 13 is not
  world_config odd = null_config();
  odd.fixed_dt = 0.015; // not a whole number of quanta
  world w2(odd);
  EXPECT_THROW(w2.follow(clock), cvc::unsupported_exception);
}

TEST(PhysicsWorld, BilinearHeightMatchesTheGrid) {
  heightfield h;
  h.rows = h.cols = 2;
  h.dx = 2.0;
  h.dy = 1.0;
  h.origin = {10.0, 20.0, 1.0};
  h.heights = {0.0f, 2.0f, 4.0f, 6.0f}; // row 0: 0, 2; row 1: 4, 6
  EXPECT_DOUBLE_EQ(bilinear_height(h, 10.0, 20.0), 1.0);
  EXPECT_DOUBLE_EQ(bilinear_height(h, 12.0, 21.0), 7.0);
  EXPECT_DOUBLE_EQ(bilinear_height(h, 11.0, 20.5), 1.0 + 3.0);
  EXPECT_TRUE(std::isnan(bilinear_height(h, 9.0, 20.0)));
}

// --- Jolt ---------------------------------------------------------------------

TEST(PhysicsJolt, ReportsItsCapabilities) {
  REQUIRE_JOLT();
  const std::uint32_t caps = backend_capabilities(backend::jolt);
  EXPECT_TRUE(caps & cap_dynamics);
  EXPECT_TRUE(caps & cap_heightfield);
  EXPECT_TRUE(caps & cap_determinism_cross_platform) << backend_version(backend::jolt);
  std::printf("[physics] %s\n", backend_version(backend::jolt).c_str());
}

// Gate 1.
TEST(PhysicsJolt, BoxRestsOnTheHeightfieldAtTheSampledHeight) {
  REQUIRE_JOLT();
  world w(jolt_config());
  const heightfield terrain = rolling_terrain();
  w.add_body({terrain, motion_type::static_});
  const double half = 0.1;
  std::vector<body_id> boxes;
  for (const auto &xy :
       std::vector<std::pair<double, double>>{{-2, -3}, {2, -3}, {-2, 3}, {3, 1}}) {
    body_desc d{box{{half, half, half}}};
    d.position = {xy.first, xy.second, bilinear_height(terrain, xy.first, xy.second) + 0.5};
    boxes.push_back(w.add_body(d));
  }
  w.step(500); // 5 s
  for (const body_id b : boxes) {
    const body_state s = w.state(b);
    EXPECT_LT(speed(s), 0.01) << "box " << b.value << " did not come to rest";
    const double ground = bilinear_height(terrain, s.position.x, s.position.y);
    const double expect = ground + half / slope_cos(terrain, s.position.x, s.position.y);
    EXPECT_NEAR(s.position.z, expect, 0.01) << "box " << b.value;
  }
  // The engine's surface agrees with the raster where nothing rests on it, too.
  for (double x = -6.0; x <= 6.0; x += 1.5) {
    const auto hit = w.raycast({x, 0.37, 5.0}, {x, 0.37, -5.0});
    ASSERT_TRUE(hit.has_value());
    EXPECT_NEAR(hit->point.z, bilinear_height(terrain, x, 0.37), 0.005);
    EXPECT_GT(hit->normal.z, 0.9);
  }
}

// Gate 2, part one: the contact carries the cell's material, and the solver used
// the cell's μ (the default material is neutral under the product rule).
TEST(PhysicsJolt, ContactReportsTheCellMaterial) {
  REQUIRE_JOLT();
  world w(jolt_config());
  const material_id asphalt = w.add_material({"asphalt", 0.9f, 0.0f, 0.01f, 1});
  const material_id ice = w.add_material({"ice", 0.3f, 0.0f, 0.005f, 2});
  heightfield h;
  h.rows = h.cols = 9;
  h.heights.assign(81, 0.0f);
  h.materials = {asphalt, ice};
  h.cell_materials.assign(64, 0);
  for (int r = 0; r < 8; ++r)
    for (int c = 4; c < 8; ++c)
      h.cell_materials[static_cast<size_t>(r) * 8 + c] = 1; // x >= 4 m is ice
  const body_id ground = w.add_body({h, motion_type::static_});
  const body_id on_ice = w.add_body({box{{0.2, 0.2, 0.2}}, motion_type::dynamic, {6.0, 4.0, 0.5}});
  const body_id on_asphalt =
      w.add_body({box{{0.2, 0.2, 0.2}}, motion_type::dynamic, {2.0, 4.0, 0.5}});
  bool saw_ice = false, saw_asphalt = false;
  for (int i = 0; i < 100 && !(saw_ice && saw_asphalt); ++i) {
    w.step();
    for (const contact_event &e : w.contacts()) {
      if (e.type != contact_event::kind::added)
        continue;
      const bool ground_is_a = e.body_a == ground;
      const body_id other = ground_is_a ? e.body_b : e.body_a;
      const material_id cell = ground_is_a ? e.material_a : e.material_b;
      if (other == on_ice) {
        EXPECT_EQ(cell, ice);
        EXPECT_FLOAT_EQ(e.friction, 0.3f);
        saw_ice = true;
      } else if (other == on_asphalt) {
        EXPECT_EQ(cell, asphalt);
        EXPECT_FLOAT_EQ(e.friction, 0.9f);
        saw_asphalt = true;
      }
      EXPECT_NEAR(e.normal.z * (ground_is_a ? 1.0 : -1.0), 1.0, 1e-3); // ground -> box is up
    }
  }
  EXPECT_TRUE(saw_ice);
  EXPECT_TRUE(saw_asphalt);
  EXPECT_EQ(w.raycast({6.0, 1.0, 1.0}, {6.0, 1.0, -1.0})->material, ice);
  EXPECT_EQ(w.raycast({1.0, 1.0, 1.0}, {1.0, 1.0, -1.0})->material, asphalt);
  EXPECT_EQ(w.get_material(w.raycast({6.0, 1.0, 1.0}, {6.0, 1.0, -1.0})->material).nav_material, 2);
}

// Gate 2, part two: on a 20-degree slope (tan 20 = 0.364) a box sticks on
// asphalt (μ 0.9) and slides on ice (μ 0.2).
TEST(PhysicsJolt, FrictionDecidesStickOrSlide) {
  REQUIRE_JOLT();
  world w(jolt_config());
  const material_id asphalt = w.add_material({"asphalt", 0.9f});
  const material_id ice = w.add_material({"ice", 0.2f});
  const double theta = 20.0 * 3.14159265358979323846 / 180.0, t = std::tan(theta);
  heightfield h;
  h.cols = 41;
  h.rows = 9;
  h.dx = 0.5;
  h.dy = 1.0;
  h.heights.resize(static_cast<size_t>(h.rows) * h.cols);
  for (int r = 0; r < h.rows; ++r)
    for (int c = 0; c < h.cols; ++c)
      h.heights[static_cast<size_t>(r) * h.cols + c] = static_cast<float>(t * c * h.dx);
  h.max_error = 0.01;
  h.materials = {asphalt, ice};
  h.cell_materials.assign(static_cast<size_t>(h.rows - 1) * (h.cols - 1), 0);
  for (int r = 4; r < h.rows - 1; ++r)
    for (int c = 0; c < h.cols - 1; ++c)
      h.cell_materials[static_cast<size_t>(r) * (h.cols - 1) + c] = 1; // y >= 4 m is ice
  w.add_body({h, motion_type::static_});
  // Each box starts resting flat on the slope: rotated by -theta about y, its
  // centre half a side out along the surface normal.
  const double half = 0.2;
  const vec3 normal{-std::sin(theta), 0.0, std::cos(theta)};
  const quat tilt{std::cos(-theta / 2), 0.0, std::sin(-theta / 2), 0.0};
  auto place = [&](double x, double y) {
    body_desc d{box{{half, half, half}}};
    d.position = vec3{x, y, t * x} + normal * (half + 0.001);
    d.rotation = tilt;
    return w.add_body(d);
  };
  const body_id stuck = place(15.0, 2.0);
  const body_id slider = place(15.0, 6.0);
  const vec3 p_stuck = w.state(stuck).position, p_slider = w.state(slider).position;
  w.step(200); // 2 s
  const vec3 d_stuck = w.state(stuck).position - p_stuck;
  const vec3 d_slider = w.state(slider).position - p_slider;
  EXPECT_LT(std::sqrt(dot(d_stuck, d_stuck)), 0.01) << "the asphalt box moved";
  // g (sin 20 - 0.2 cos 20) = 1.51 m/s^2 for 2 s is about 3 m down the slope.
  EXPECT_LT(d_slider.x, -1.0) << "the ice box did not slide";
}

// Gate 3.
TEST(PhysicsJolt, SameTraceTwiceGivesTheSameHash) {
  REQUIRE_JOLT();
  world a(jolt_config()), b(jolt_config());
  const busy_scene sa(a), sb(b);
  const std::vector<std::uint64_t> ha = sa.run(a, 300, 25);
  const std::vector<std::uint64_t> hb = sb.run(b, 300, 25);
  EXPECT_EQ(ha, hb);
  std::printf("[physics-hash] busy scene after 300 steps: %016llx\n",
              static_cast<unsigned long long>(ha.back()));
}

// Gate 4. Jolt promises a deterministic step with any number of worker threads
// (PhysicsSettings::mDeterministicSimulation, set by the backend); this measures
// it rather than assuming it.
TEST(PhysicsJolt, OneThreadAndSeveralGiveTheSameHash) {
  REQUIRE_JOLT();
  world_config many = jolt_config();
  many.threads = 3;
  world a(jolt_config()), b(many);
  const busy_scene sa(a), sb(b);
  EXPECT_EQ(sa.run(a, 300, 25), sb.run(b, 300, 25));
}

// Gate 5.
TEST(PhysicsJolt, SaveRestoreAndRerunGiveTheSameHash) {
  REQUIRE_JOLT();
  world a(jolt_config());
  const busy_scene sa(a);
  sa.run(a, 100, 100);
  const std::vector<std::uint8_t> saved = a.save();
  const std::uint64_t h_after = sa.run(a, 100, 100).back();

  a.restore(saved); // the same world, rewound
  EXPECT_EQ(a.tick(), 100u);
  EXPECT_EQ(sa.run(a, 100, 100).back(), h_after);

  world b(jolt_config()); // a fresh world built the same way
  const busy_scene sb(b);
  b.restore(saved);
  EXPECT_EQ(sb.run(b, 100, 100).back(), h_after);
}

TEST(PhysicsJolt, ContactEventsAreSortedAndPaired) {
  REQUIRE_JOLT();
  world_config c = jolt_config();
  c.contacts = contact_reporting::all;
  world w(c);
  const busy_scene s(w);
  for (int i = 0; i < 150; ++i) {
    w.step();
    const std::vector<contact_event> &ev = w.contacts();
    for (size_t k = 0; k < ev.size(); ++k) {
      EXPECT_LT(ev[k].body_a, ev[k].body_b);
      if (k > 0) {
        const contact_event &p = ev[k - 1];
        EXPECT_FALSE(std::make_tuple(ev[k].body_a.value, ev[k].sub_shape_a, ev[k].body_b.value,
                                     ev[k].sub_shape_b) <
                     std::make_tuple(p.body_a.value, p.sub_shape_a, p.body_b.value, p.sub_shape_b));
      }
    }
  }
}

TEST(PhysicsJolt, FloatingOriginHoldsCityScaleCoordinates) {
  REQUIRE_JOLT();
  const vec3 site{620000.0, 3350000.0, 150.0}; // UTM-like
  world_config c = jolt_config();
  c.origin = site;
  world w(c);
  const heightfield terrain = rolling_terrain(33, 0.5, site);
  w.add_body({terrain, motion_type::static_});
  const double half = 0.1;
  body_desc d{box{{half, half, half}}};
  d.position = site + vec3{1.0, -2.0, 1.0};
  const body_id b = w.add_body(d);
  w.step(400);
  const body_state rest = w.state(b);
  const double ground = bilinear_height(terrain, rest.position.x, rest.position.y);
  EXPECT_NEAR(rest.position.z, ground + half / slope_cos(terrain, rest.position.x, rest.position.y),
              0.01);
  w.rebase(site + vec3{100.0, -50.0, 0.0});
  const body_state moved = w.state(b);
  EXPECT_NEAR(moved.position.x, rest.position.x, 1e-3);
  EXPECT_NEAR(moved.position.y, rest.position.y, 1e-3);
  EXPECT_NEAR(moved.position.z, rest.position.z, 1e-3);
  w.step(100);
  EXPECT_NEAR(w.state(b).position.z, rest.position.z, 2e-3);
}

TEST(PhysicsJolt, KinematicBodyPushesADynamicOne) {
  REQUIRE_JOLT();
  world w(jolt_config());
  w.add_body({box{{20.0, 20.0, 0.5}}, motion_type::static_, {0.0, 0.0, -0.5}});
  const body_id crate =
      w.add_body({box{{0.25, 0.25, 0.25}}, motion_type::dynamic, {0.0, 0.0, 0.25}});
  body_desc plow{box{{0.1, 1.0, 0.3}}, motion_type::kinematic, {-2.0, 0.0, 0.35}};
  plow.linear_velocity = {1.0, 0.0, 0.0};
  w.add_body(plow);
  w.step(300);
  EXPECT_GT(w.state(crate).position.x, 0.3);
}

TEST(PhysicsJolt, DeterminismModeTheBuildCannotDeliverIsRefused) {
  REQUIRE_JOLT();
  if (backend_capabilities(backend::jolt) & cap_determinism_cross_platform)
    GTEST_SKIP() << "this Jolt is cross-platform deterministic";
  EXPECT_THROW(world w(jolt_config()), cvc::unsupported_exception);
}

TEST(PhysicsJolt, HeightfieldTooRoughForMaxErrorIsRefused) {
  REQUIRE_JOLT();
  world w(jolt_config());
  heightfield h = rolling_terrain(9);
  h.heights[40] = 900.0f; // a 900 m spike in a field of decimetres
  h.max_error = 1e-6;
  EXPECT_THROW(w.add_body({h, motion_type::static_}), cvc::physics_error);
}

// --- reconfigure and set_material ------------------------------------------------

TEST(PhysicsWorld, ReconfigureChangesRuntimeSettingsOnly) {
  world w(null_config());
  world_config c = w.config();
  c.gravity = {0.0, 0.0, -1.62};
  c.contacts = contact_reporting::all;
  w.reconfigure(c);
  EXPECT_EQ(w.config().gravity.z, -1.62);
  EXPECT_EQ(w.config().contacts, contact_reporting::all);
  c.fixed_dt = 0.02;
  EXPECT_THROW(w.reconfigure(c), cvc::physics_error);
  EXPECT_EQ(w.config().fixed_dt, 0.01) << "a refused reconfigure changes nothing";
  c = w.config();
  c.origin = {1.0, 0.0, 0.0};
  EXPECT_THROW(w.reconfigure(c), cvc::physics_error); // the origin moves with rebase()
}

TEST(PhysicsWorld, SetMaterialUpdatesTheTable) {
  world w(null_config());
  const material_id road = w.add_material({"road", 0.9f});
  w.set_material(road, {"road", 0.4f, 0.0f, 0.02f, 3}); // rain
  EXPECT_EQ(w.get_material(road).friction, 0.4f);
  EXPECT_EQ(w.get_material(road).nav_material, 3);
  EXPECT_THROW(w.set_material(road, {"road", -0.1f}), cvc::physics_error);
  EXPECT_THROW(w.set_material(77, {"x"}), cvc::physics_error);
}

// --- the state tree --------------------------------------------------------------

namespace {
std::string key_value(cvc::app &ctx, const std::string &path) {
  return cvc::state::instance(ctx)(path).value();
}
void set_key(cvc::app &ctx, const std::string &path, const std::string &v) {
  cvc::state::instance(ctx)(path).value(v);
}
} // namespace

TEST(PhysicsBinding, ConfigFromStateSeedsDefaultsAndReadsWhatIsSet) {
  cvc::app ctx;
  set_key(ctx, "phys.backend", "null");
  set_key(ctx, "phys.fixed_dt", "0.02");
  set_key(ctx, "phys.gravity.z", "-1.62");
  set_key(ctx, "phys.contacts", "all");
  const world_config c = world_binding::config_from_state(ctx, "phys");
  EXPECT_EQ(c.backend, backend::null);
  EXPECT_DOUBLE_EQ(c.fixed_dt, 0.02);
  EXPECT_DOUBLE_EQ(c.gravity.z, -1.62);
  EXPECT_EQ(c.contacts, contact_reporting::all);
  // Unset keys were seeded with the defaults, so the tree shows the whole config.
  EXPECT_EQ(key_value(ctx, "phys.collision_steps"), "2");
  EXPECT_EQ(key_value(ctx, "phys.determinism"), "cross_platform");
  EXPECT_EQ(key_value(ctx, "phys.friction_combine"), "product");
  // A value it cannot read keeps the default instead of throwing.
  set_key(ctx, "phys.threads", "lots");
  EXPECT_EQ(world_binding::config_from_state(ctx, "phys").threads, 0);
}

TEST(PhysicsBinding, PullAppliesRuntimeKeysAndFlagsBuildKeys) {
  cvc::app ctx;
  world w(null_config());
  world_binding b(ctx, w, "phys");
  b.step();
  EXPECT_EQ(key_value(ctx, "phys.tick"), "1");
  EXPECT_EQ(key_value(ctx, "phys.backend"), "null");
  EXPECT_EQ(key_value(ctx, "phys.restart_required"), "0");
  set_key(ctx, "phys.gravity.z", "-3.71");
  set_key(ctx, "phys.velocity_steps", "12");
  b.pull();
  EXPECT_DOUBLE_EQ(w.config().gravity.z, -3.71);
  EXPECT_EQ(w.config().velocity_steps, 12);
  set_key(ctx, "phys.fixed_dt", "0.05"); // fixed when the world was built
  b.step();
  EXPECT_DOUBLE_EQ(w.config().fixed_dt, 0.01);
  EXPECT_EQ(key_value(ctx, "phys.restart_required"), "1");
  set_key(ctx, "phys.penetration_slop", "-1"); // invalid: ignored, the world keeps running
  EXPECT_NO_THROW(b.step());
  EXPECT_DOUBLE_EQ(w.config().penetration_slop, 0.002);
}

TEST(PhysicsBinding, MaterialsAreDeclaredInTheTree) {
  cvc::app ctx;
  world w(null_config());
  world_binding b(ctx, w, "phys");
  b.pull();
  EXPECT_EQ(b.material("default"), 0);
  EXPECT_EQ(key_value(ctx, "phys.materials.default.friction"), "1");
  set_key(ctx, "phys.materials.ice.friction", "0.1");
  set_key(ctx, "phys.materials.ice.nav_material", "4");
  b.pull();
  const material_id ice = b.material("ice");
  EXPECT_FLOAT_EQ(w.get_material(ice).friction, 0.1f);
  EXPECT_EQ(w.get_material(ice).nav_material, 4);
  EXPECT_EQ(key_value(ctx, "phys.materials.ice.restitution"), "0"); // seeded
  set_key(ctx, "phys.materials.ice.friction", "0.05");
  b.pull();
  EXPECT_FLOAT_EQ(w.get_material(ice).friction, 0.05f);
  set_key(ctx, "phys.materials.ice.friction", "-2"); // invalid: the last good value stays
  b.pull();
  EXPECT_FLOAT_EQ(w.get_material(ice).friction, 0.05f);
  set_key(ctx, "phys.materials.default.friction", "0.8");
  b.pull();
  EXPECT_FLOAT_EQ(w.get_material(0).friction, 0.8f);
  EXPECT_THROW(b.material("lava"), cvc::physics_error);
}

TEST(PhysicsBinding, EachAppConfiguresItsOwnWorld) {
  cvc::app a, b;
  set_key(a, "physics.gravity.z", "-1.0");
  world wa(world_binding::config_from_state(a));
  world wb(world_binding::config_from_state(b));
  EXPECT_DOUBLE_EQ(wa.config().gravity.z, -1.0);
  EXPECT_DOUBLE_EQ(wb.config().gravity.z, -9.80665);
}

TEST(PhysicsBinding, JoltWorldFallsUnderTheTreesGravity) {
  REQUIRE_JOLT();
  cvc::app ctx;
  set_key(ctx, "physics.gravity.z", "-1.62"); // the Moon
  set_key(ctx, "physics.hash_every", "1");
  world w(world_binding::config_from_state(ctx));
  world_binding b(ctx, w);
  body_desc d{sphere{0.1}, motion_type::dynamic, {0.0, 0.0, 10.0}};
  d.linear_damping = 0.0;
  const body_id ball = w.add_body(d);
  for (int i = 0; i < 100; ++i)
    b.step();
  // 1/2 g t^2 = 0.81 m after 1 s (semi-implicit Euler at 0.01 s adds about 0.008 m).
  EXPECT_NEAR(10.0 - w.state(ball).position.z, 0.81, 0.02);
  const std::string h = key_value(ctx, "physics.state_hash");
  ASSERT_EQ(h.size(), 16u);
  char expect[17];
  std::snprintf(expect, sizeof(expect), "%016llx", static_cast<unsigned long long>(w.state_hash()));
  EXPECT_EQ(h, std::string(expect));
  EXPECT_NE(key_value(ctx, "physics.version").find("Jolt"), std::string::npos);
}

TEST(PhysicsJolt, WorldsComeAndGoWithoutAProcessWideSetup) {
  REQUIRE_JOLT();
  // Jolt's globals live while some world does: create, destroy, create again,
  // and overlap two worlds.
  for (int i = 0; i < 3; ++i) {
    world a(jolt_config());
    const body_id ball = a.add_body({sphere{}, motion_type::dynamic, {0, 0, 1}});
    a.step(5);
    {
      world b(jolt_config());
      b.add_body({box{}, motion_type::dynamic, {0, 0, 1}});
      b.step(5);
    }
    a.step(5);
    EXPECT_LT(a.state(ball).position.z, 1.0);
  }
}

// One thread steps while others read, push forces, add bodies and pull tree
// changes through a binding. Every call is serialized by the world's lock, so
// nothing throws and the world ends in a consistent state.
TEST(PhysicsJolt, ConcurrentCallsAreSafe) {
  REQUIRE_JOLT();
  cvc::app ctx;
  world w(jolt_config());
  world_binding bind(ctx, w);
  const busy_scene scene(w);
  std::atomic<bool> stop{false};
  std::atomic<int> errors{0};
  std::atomic<int> added{0};
  auto guard = [&](auto &&f) {
    try {
      f();
    } catch (...) {
      ++errors;
    }
  };
  std::thread stepper([&] {
    for (int i = 0; i < 300; ++i)
      guard([&] { bind.step(); });
    stop = true;
  });
  std::thread reader([&] {
    while (!stop)
      guard([&] {
        for (const body_id b : scene.movers)
          (void)w.state(b);
        (void)w.contacts();
        (void)w.state_hash();
      });
  });
  std::thread pusher([&] {
    int k = 0;
    while (!stop)
      guard([&] {
        w.add_force(scene.movers[static_cast<size_t>(k++) % scene.movers.size()], {5.0, 0.0, 0.0});
        if (k % 50 == 0 && added < 10) {
          w.add_body({sphere{0.1}, motion_type::dynamic, {0.0, 0.0, 3.0}});
          ++added;
        }
      });
  });
  std::thread tuner([&] {
    int k = 0;
    while (!stop)
      guard([&] {
        set_key(ctx, "physics.gravity.z", (k++ % 2) ? "-9.80665" : "-9.7");
        bind.pull();
      });
  });
  stepper.join();
  reader.join();
  pusher.join();
  tuner.join();
  EXPECT_EQ(errors.load(), 0);
  EXPECT_EQ(w.tick(), 300u);
  EXPECT_EQ(w.body_count(), 1u + scene.movers.size() + static_cast<size_t>(added.load()));
}
