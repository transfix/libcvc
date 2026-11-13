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

#include <algorithm>
#include <atomic>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/exception.h>
#include <cvc/core/world_clock.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/height_field.h>
#include <cvc/physics/snapshot_json.h>
#include <cvc/physics/world.h>
#include <cvc/physics/world_binding.h>
#include <cvc/state/state.h>
#include <gtest/gtest.h>
#include <limits>
#include <memory>
#include <sstream>
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

// Gentle rolling terrain, centred on (origin.x, origin.y) at elevation origin.z:
// slopes under 7 degrees, so a box with friction 1 never slides. The world's own
// cvc::height_field, as a renderer and the navigator would share it.
std::shared_ptr<cvc::height_field> rolling_terrain(int n = 33, double cell = 0.5,
                                                   vec3 origin = {}) {
  const double half = 0.5 * (n - 1) * cell;
  std::vector<float> h(static_cast<size_t>(n) * n);
  for (int r = 0; r < n; ++r)
    for (int c = 0; c < n; ++c) {
      const double x = c * cell - half, y = r * cell - half;
      h[static_cast<size_t>(r) * n + c] = static_cast<float>(origin.z + 0.3 * std::sin(x / 3.0) +
                                                             0.2 * std::cos(y / 4.0) + 0.02 * x);
    }
  return std::make_shared<cvc::height_field>(n, n, origin.x - half, origin.y - half, cell, cell,
                                             std::move(h));
}

// The cosine of the terrain's slope at (x, y): the facet's normal.
double slope_cos(const cvc::height_field &h, double x, double y) { return h.normal(x, y)[2]; }

double speed(const body_state &s) { return std::sqrt(dot(s.linear_velocity, s.linear_velocity)); }

// A scene with enough going on to exercise the solver: terrain, sixteen boxes and
// spheres dropped with spin, and scripted forces.
struct busy_scene {
  std::vector<body_id> movers;

  explicit busy_scene(world &w) {
    w.add_body({terrain{rolling_terrain()}, motion_type::static_});
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
  const auto field = rolling_terrain(5);
  EXPECT_THROW(w.add_body({terrain{field}, motion_type::dynamic}), cvc::physics_error); // static
  EXPECT_THROW(w.add_body({terrain{}, motion_type::static_}), cvc::physics_error);      // no field
  EXPECT_THROW(w.add_body({terrain{field, {}, 0.0}, motion_type::static_}), cvc::physics_error);
  // Cell surface 3 with no table means world material 3, which does not exist; a
  // table maps it, and a table too short for it is refused.
  auto marked = std::make_shared<cvc::height_field>(*field);
  marked->set_cell_materials(std::vector<cvc::face_material_t>(16, 3));
  EXPECT_THROW(w.add_body({terrain{marked}, motion_type::static_}), cvc::physics_error);
  EXPECT_THROW(w.add_body({terrain{marked, {0, 0}}, motion_type::static_}), cvc::physics_error);
  EXPECT_NO_THROW(w.add_body({terrain{marked, {0, 0, 0, 0}}, motion_type::static_}));
  auto g = std::make_shared<cvc::geometry>();
  g->points() = {{{0, 0, 0}}, {{1, 0, 0}}, {{0, 1, 0}}};
  g->tris().push_back({{0, 1, 5}});
  EXPECT_THROW(w.add_body({mesh{g}, motion_type::static_}), cvc::physics_error); // bad index
  EXPECT_THROW(w.add_body({mesh{}, motion_type::static_}), cvc::physics_error);  // no geometry
  g->tris().back() = {{0, 1, 2}};
  EXPECT_THROW(w.add_body({mesh{g}, motion_type::dynamic}), cvc::physics_error); // static
  EXPECT_NO_THROW(w.add_body({mesh{g}, motion_type::static_}));
  auto empty = std::make_shared<cvc::geometry>();
  EXPECT_THROW(w.add_body({mesh{empty}, motion_type::static_}), cvc::physics_error); // no faces
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
  const auto field = rolling_terrain();
  w.add_body({terrain{field}, motion_type::static_});
  const double half = 0.1;
  std::vector<body_id> boxes;
  for (const auto &xy :
       std::vector<std::pair<double, double>>{{-2, -3}, {2, -3}, {-2, 3}, {3, 1}}) {
    body_desc d{box{{half, half, half}}};
    d.position = {xy.first, xy.second, field->height(xy.first, xy.second) + 0.5};
    boxes.push_back(w.add_body(d));
  }
  w.step(500); // 5 s
  for (const body_id b : boxes) {
    const body_state s = w.state(b);
    EXPECT_LT(speed(s), 0.01) << "box " << b.value << " did not come to rest";
    const double ground = field->height(s.position.x, s.position.y);
    const double expect = ground + half / slope_cos(*field, s.position.x, s.position.y);
    EXPECT_NEAR(s.position.z, expect, 0.01) << "box " << b.value;
  }
  // The engine's surface is the field's where nothing rests on it, too.
  for (double x = -6.0; x <= 6.0; x += 1.5) {
    const auto hit = w.raycast({x, 0.37, 5.0}, {x, 0.37, -5.0});
    ASSERT_TRUE(hit.has_value());
    EXPECT_NEAR(hit->point.z, field->height(x, 0.37), 0.005);
    EXPECT_GT(hit->normal.z, 0.9);
  }
}

// The engine collides with the height field's own triangles: the same diagonal
// split cvc::height_field::height() and the renderer's mesh use. A rough field
// makes the two possible diagonals differ by up to decimetres inside a cell, far
// more than the engine's height compression (max_error).
TEST(PhysicsJolt, TerrainCollidesWithTheFieldsOwnTriangles) {
  REQUIRE_JOLT();
  world w(jolt_config());
  const int n = 12;
  std::vector<float> h(n * n);
  for (int r = 0; r < n; ++r)
    for (int c = 0; c < n; ++c)
      h[r * n + c] = static_cast<float>(((r * 7 + c * 13) % 11) * 0.1); // 0 .. 1 m, rough
  const auto field = std::make_shared<cvc::height_field>(n, n, -3.0, 2.0, 0.5, 0.75, h);
  w.add_body({terrain{field}, motion_type::static_});
  double worst = 0.0;
  for (int i = 0; i < 200; ++i) {
    const double x = -3.0 + 5.5 * ((i * 37) % 200) / 200.0 + 0.0013;
    const double y = 2.0 + 8.25 * ((i * 91) % 200) / 200.0 + 0.0017;
    const auto hit = w.raycast({x, y, 5.0}, {x, y, -5.0});
    ASSERT_TRUE(hit.has_value()) << x << ", " << y;
    worst = std::max(worst, std::fabs(hit->point.z - field->height(x, y)));
  }
  EXPECT_LT(worst, 0.006) << "the engine's triangles are not the field's";
}

// A mesh is the world's cvc::geometry: its triangles and split quads, each face
// with its own surface material through the terrain-style materials table.
TEST(PhysicsJolt, MeshFacesCarryTheirSurfaceMaterials) {
  REQUIRE_JOLT();
  world w(jolt_config());
  const material_id asphalt = w.add_material({"asphalt", 0.9f});
  const material_id ice = w.add_material({"ice", 0.2f});
  // Two quads side by side: x in [0, 2] is surface 0, x in [2, 4] surface 1.
  auto floor = std::make_shared<cvc::geometry>();
  for (int i = 0; i < 3; ++i) {
    floor->points().push_back({{2.0 * i, 0.0, 0.0}});
    floor->points().push_back({{2.0 * i, 2.0, 0.0}});
  }
  floor->quads().push_back({{0, 2, 3, 1}});
  floor->quads().push_back({{2, 4, 5, 3}});
  floor->face_materials() = {0, 1};
  const body_id slab = w.add_body({mesh{floor, {asphalt, ice}}, motion_type::static_});
  EXPECT_EQ(w.raycast({1.0, 1.0, 1.0}, {1.0, 1.0, -1.0})->material, asphalt);
  EXPECT_EQ(w.raycast({3.0, 1.0, 1.0}, {3.0, 1.0, -1.0})->material, ice);
  EXPECT_EQ(w.raycast({3.0, 1.0, 1.0}, {3.0, 1.0, -1.0})->body, slab);
  // Without face materials the whole mesh is the body's material.
  auto plain = std::make_shared<cvc::geometry>(*floor);
  plain->face_materials().clear();
  body_desc d{mesh{plain}, motion_type::static_, {0.0, 10.0, 0.0}};
  d.material = ice;
  w.add_body(d);
  EXPECT_EQ(w.raycast({1.0, 11.0, 1.0}, {1.0, 11.0, -1.0})->material, ice);
}

// Gate 2, part one: the contact carries the cell's material, and the solver used
// the cell's μ (the default material is neutral under the product rule).
TEST(PhysicsJolt, ContactReportsTheCellMaterial) {
  REQUIRE_JOLT();
  world w(jolt_config());
  const material_id asphalt = w.add_material({"asphalt", 0.9f, 0.0f, 0.01f, 1});
  const material_id ice = w.add_material({"ice", 0.3f, 0.0f, 0.005f, 2});
  std::vector<cvc::face_material_t> cells(64, 0); // surface 0 is asphalt, 1 ice
  for (int r = 0; r < 8; ++r)
    for (int c = 4; c < 8; ++c)
      cells[static_cast<size_t>(r) * 8 + c] = 1; // x >= 4 m is ice
  const auto field = std::make_shared<cvc::height_field>(9, 9, 0.0, 0.0, 1.0, 1.0,
                                                         std::vector<float>(81, 0.0f), cells);
  const body_id ground = w.add_body({terrain{field, {asphalt, ice}}, motion_type::static_});
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
  const int rows = 9, cols = 41;
  const double dx = 0.5, dy = 1.0;
  std::vector<float> heights(static_cast<size_t>(rows) * cols);
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c)
      heights[static_cast<size_t>(r) * cols + c] = static_cast<float>(t * c * dx);
  std::vector<cvc::face_material_t> cells(static_cast<size_t>(rows - 1) * (cols - 1), 0);
  for (int r = 4; r < rows - 1; ++r)
    for (int c = 0; c < cols - 1; ++c)
      cells[static_cast<size_t>(r) * (cols - 1) + c] = 1; // y >= 4 m is ice
  const auto field =
      std::make_shared<cvc::height_field>(rows, cols, 0.0, 0.0, dx, dy, heights, cells);
  w.add_body({terrain{field, {asphalt, ice}, 0.01}, motion_type::static_});
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
  const auto field = rolling_terrain(33, 0.5, site);
  w.add_body({terrain{field}, motion_type::static_});
  const double half = 0.1;
  body_desc d{box{{half, half, half}}};
  d.position = site + vec3{1.0, -2.0, 1.0};
  const body_id b = w.add_body(d);
  w.step(400);
  const body_state rest = w.state(b);
  const double ground = field->height(rest.position.x, rest.position.y);
  EXPECT_NEAR(rest.position.z, ground + half / slope_cos(*field, rest.position.x, rest.position.y),
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
  auto field = rolling_terrain(9);
  field->set_height(4, 4, 900.0f); // a 900 m spike in a field of decimetres
  EXPECT_THROW(w.add_body({terrain{field, {}, 1e-6}, motion_type::static_}), cvc::physics_error);
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
  // A material's own keys are not materials: default and ice, and no more, however many pulls.
  b.pull();
  EXPECT_EQ(w.material_count(), 2u);
  EXPECT_THROW(b.material("friction"), cvc::physics_error);
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

// push() publishes the whole world as one snapshot, and writes the named bodies' keys from it.
TEST(PhysicsBinding, PushPublishesTheWorldAndItsNamedBodies) {
  REQUIRE_JOLT();
  cvc::app ctx;
  world w(jolt_config());
  world_binding b(ctx, w);
  w.add_body({box{{50.0, 50.0, 0.5}}, motion_type::static_, {0.0, 0.0, -0.5}});
  const body_id ball = w.add_body({sphere{0.2}, motion_type::dynamic, {0.0, 0.0, 2.0}});
  EXPECT_EQ(world_binding::snapshot_from_state(ctx), nullptr);
  b.publish_body("ball", ball);
  EXPECT_THROW(b.publish_body("a.b", ball), cvc::physics_error);
  EXPECT_THROW(b.publish_body("", ball), cvc::physics_error);
  body_id nobody;
  nobody.value = 12345;
  EXPECT_THROW(b.publish_body("ghost", nobody), cvc::physics_error);
  for (int i = 0; i < 30; ++i)
    b.step();
  const auto s = world_binding::snapshot_from_state(ctx);
  ASSERT_NE(s, nullptr);
  EXPECT_EQ(s, b.last_snapshot());
  EXPECT_EQ(s->tick, w.tick());
  ASSERT_EQ(s->bodies.size(), 1u);
  ASSERT_EQ(s->static_bodies->size(), 1u);
  EXPECT_EQ((*s->static_bodies)[0].motion, motion_type::static_);
  const body_state now = w.state(ball);
  EXPECT_EQ(s->bodies[0].state.position.z, now.position.z);
  // The keys hold that tick's values to the last digit.
  EXPECT_EQ(std::stod(key_value(ctx, "physics.bodies.ball.position.z")), now.position.z);
  EXPECT_EQ(std::stod(key_value(ctx, "physics.bodies.ball.velocity.z")), now.linear_velocity.z);
  EXPECT_EQ(key_value(ctx, "physics.bodies.ball.motion"), "dynamic");
  EXPECT_EQ(key_value(ctx, "physics.bodies.ball.active"), "1");
  EXPECT_EQ(key_value(ctx, "physics.bodies.ball.id"), std::to_string(ball.value));
  // snapshot_every = 0: no new snapshot, while the named keys keep up.
  set_key(ctx, "physics.snapshot_every", "0");
  for (int i = 0; i < 5; ++i)
    b.step();
  EXPECT_EQ(world_binding::snapshot_from_state(ctx)->tick, s->tick);
  EXPECT_EQ(std::stod(key_value(ctx, "physics.bodies.ball.position.z")), w.state(ball).position.z);
  // Every 4 steps: from tick 35, at 36 and 40.
  set_key(ctx, "physics.snapshot_every", "4");
  for (int i = 0; i < 6; ++i)
    b.step();
  EXPECT_EQ(world_binding::snapshot_from_state(ctx)->tick, 40u);
  // A body the world no longer has stops being published.
  w.remove_body(ball);
  for (int i = 0; i < 4; ++i) // to tick 44, the next snapshot
    EXPECT_NO_THROW(b.step());
  EXPECT_EQ(b.last_snapshot()->tick, 44u);
  EXPECT_EQ(b.last_snapshot()->bodies.size(), 0u);
  EXPECT_EQ(b.last_snapshot()->static_bodies->size(), 1u);
}

// What publishing costs, each part timed right after a step, on the stepping thread: a
// snapshot is one pass over the moving bodies, and the publish thread takes the key writes
// and the JSON off the step. The bodies never sleep here, so every step does the same work.
TEST(PhysicsBinding, WhatPublishingCostsAStep) {
  REQUIRE_JOLT();
  cvc::app ctx;
  world w(jolt_config());
  w.add_body({box{{50.0, 50.0, 0.5}}, motion_type::static_, {0.0, 0.0, -0.5}});
  std::vector<body_id> balls;
  for (int i = 0; i < 1000; ++i) {
    body_desc d{sphere{0.1},
                motion_type::dynamic,
                {-15.0 + 1.0 * (i % 32), -15.0 + 1.0 * (i / 32), 0.1 + 0.01 * (i % 7)}};
    d.allow_sleeping = false;
    balls.push_back(w.add_body(d));
  }
  world_binding b(ctx, w);
  for (int i = 0; i < 4; ++i)
    b.publish_body("ball" + std::to_string(i), balls[static_cast<std::size_t>(i)]);
  b.step(); // seeds the keys
  using clock = std::chrono::steady_clock;
  const auto us = [](clock::time_point a, clock::time_point z) {
    return std::chrono::duration<double, std::micro>(z - a).count();
  };
  const auto set = [&](const char *k, const char *v) {
    cvc::state::instance(ctx)(std::string("physics.") + k).value(std::string(v));
  };
  const int n = 100;
  // Medians, which a busy machine disturbs less than means.
  const auto median = [](std::vector<double> v) {
    std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(v.size() / 2), v.end());
    return v[v.size() / 2];
  };
  // Step, snapshot(), pull() with nothing changed, and push() as configured.
  const auto run = [&](double &step_us, double &snap_us, double &pull_us, double &push_us) {
    std::vector<double> steps, snaps, pulls, pushes;
    for (int i = 0; i < n; ++i) {
      const clock::time_point t0 = clock::now();
      w.step();
      const clock::time_point t1 = clock::now();
      const world_snapshot s = w.snapshot();
      const clock::time_point t2 = clock::now();
      b.pull();
      const clock::time_point t3 = clock::now();
      b.push();
      const clock::time_point t4 = clock::now();
      b.flush(); // not timed: the publish thread catches up
      steps.push_back(us(t0, t1));
      snaps.push_back(us(t1, t2));
      pulls.push_back(us(t2, t3));
      pushes.push_back(us(t3, t4));
      EXPECT_EQ(s.bodies.size(), 1000u);
    }
    step_us = median(steps);
    snap_us = median(snaps);
    pull_us = median(pulls);
    push_us = median(pushes);
  };
  double step_us, snap_us, pull_us, keys_us, data_us, json_us, thread_us;
  set("snapshot_every", "0");
  b.pull();
  run(step_us, snap_us, pull_us, keys_us); // the telemetry and four named bodies
  set("snapshot_every", "1");
  b.pull();
  run(step_us, snap_us, pull_us, data_us); // and the snapshot as data
  set("snapshot_json_every", "1");
  b.pull();
  run(step_us, snap_us, pull_us, json_us); // and as JSON
  set("publish_thread", "1");
  b.pull();
  run(step_us, snap_us, pull_us, thread_us); // all of it, written by the publish thread
  const std::string json = to_json(w.snapshot(), json_bodies::moving);
  std::printf("  1000 moving bodies: a step %.0f us; snapshot() %.0f us; pull() %.1f us with "
              "nothing changed\n",
              step_us, snap_us, pull_us);
  std::printf("  push() on the step: %.0f us telemetry and 4 named bodies, %.0f us with the "
              "snapshot, %.0f us with its JSON (%zu bytes), %.0f us with the publish thread\n",
              keys_us, data_us, json_us, json.size(), thread_us);
  EXPECT_LT(snap_us, 20000.0);
  EXPECT_LT(thread_us, json_us) << "the publish thread takes the JSON off the step";
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

// --- cross-platform determinism -------------------------------------------------

namespace {

// A scene built from exactly representable numbers only: no sin/cos (libm differs by
// an ULP across platforms), and no product that rounds (so an FMA-contracting
// compiler cannot change a value). Whatever differs between platforms after that is
// the engine's, which is what the golden below pins.
std::uint64_t exact_scene_hash(int steps) {
  world w(jolt_config());
  std::vector<float> heights(17 * 17);
  for (int r = 0; r < 17; ++r)
    for (int c = 0; c < 17; ++c)
      heights[r * 17 + c] = static_cast<float>((r % 5) * 0.125 + (c % 3) * 0.25);
  w.add_body({terrain{std::make_shared<cvc::height_field>(17, 17, -4.0, -4.0, 0.5, 0.5, heights)},
              motion_type::static_});
  std::vector<body_id> bodies;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      body_desc d;
      if ((i + j) % 2)
        d.shape = sphere{0.25};
      else
        d.shape = box{{0.25, 0.125, 0.25}};
      d.position = {i * 1.5 - 1.5, j * 1.5 - 1.5, 2.0 + 0.5 * (i + j)};
      d.linear_velocity = {0.25 * (i - 1), -0.25 * (j - 1), 0.0};
      bodies.push_back(w.add_body(d));
    }
  for (int s = 0; s < steps; ++s) {
    const body_id b = bodies[static_cast<size_t>(s) % bodies.size()];
    w.add_force(b, {(s % 7 - 3) * 2.0, (s % 5 - 2) * 4.0, 0.0});
    w.step();
  }
  return w.state_hash();
}

} // namespace

// P7's replay gate in miniature. With a cross-platform-deterministic Jolt
// (CROSS_PLATFORM_DETERMINISTIC, from joltphysics 5.6.0+cvc.5) the same calls must
// give the same state on every platform the build targets: Windows, Linux and macOS,
// x86_64 and arm64, and wasm. The constant was measured on Windows (MSVC, x86_64);
// a platform that disagrees has found a determinism leak, in the engine or in the
// facade's float conversions, and must be fixed, never re-baselined per platform.
TEST(PhysicsJolt, CrossPlatformGoldenHash) {
  REQUIRE_JOLT();
  if (!(backend_capabilities(backend::jolt) & cap_determinism_cross_platform))
    GTEST_SKIP() << "this Jolt is not cross-platform deterministic";
  const std::uint64_t h = exact_scene_hash(240);
  std::printf("[physics-golden] exact scene after 240 steps: %016llx\n",
              static_cast<unsigned long long>(h));
  EXPECT_EQ(h, exact_scene_hash(240)) << "the scene is not even deterministic in-process";
  EXPECT_EQ(h, 0x70b2cc0bd34ea4beull) << std::hex << "state hash " << h;
}

// --- forces at a point and torques (add_force_at, add_torque) -------------------------------

namespace {

vec3 cross(const vec3 &a, const vec3 &b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

// A 1 x 0.5 x 0.25 m box at 1000 kg/m^3 (125 kg) floating free: no gravity, no damping,
// far from the origin and with a floating origin of its own, so a point the backend left
// in world coordinates (instead of the engine's offsets) would torque it wildly.
struct free_box {
  static constexpr double mass = 125.0;
  // Principal moments of a solid box about its centre: m (b^2 + c^2) / 12 and so on.
  static constexpr double ixx = mass * (0.5 * 0.5 + 0.25 * 0.25) / 12.0;
  static constexpr double iyy = mass * (1.0 * 1.0 + 0.25 * 0.25) / 12.0;
  static constexpr double izz = mass * (1.0 * 1.0 + 0.5 * 0.5) / 12.0;

  world w;
  body_id b;

  static world_config config() {
    world_config c = jolt_config();
    c.gravity = {};
    c.origin = {1000.0, -500.0, 0.0};
    return c;
  }

  explicit free_box(const vec3 &angular_velocity = {}) : w(config()) {
    body_desc d{box{{0.5, 0.25, 0.125}}, motion_type::dynamic, {1003.0, -498.0, 20.0}};
    d.linear_damping = 0.0;
    d.angular_damping = 0.0;
    d.angular_velocity = angular_velocity;
    b = w.add_body(d);
  }
};

} // namespace

// Two forces off the centre of mass: after one step the body turns at sum(r x F) dt / I
// about each principal axis and its centre moves at sum(F) dt / m, as for add_force.
TEST(PhysicsJolt, AnOffCentreForceSpinsTheBody) {
  REQUIRE_JOLT();
  free_box s;
  ASSERT_NEAR(s.w.get_body(s.b).mass, free_box::mass, 1e-3);
  const vec3 com = s.w.state(s.b).position;
  const vec3 f1{0.0, 0.0, 100.0}, r1{0.5, 0.0, 0.0};  // up at the +x end: noses down, -y
  const vec3 f2{0.0, 40.0, 0.0}, r2{0.0, 0.0, 0.125}; // sideways at the top: rolls, -x
  s.w.add_force_at(s.b, f1, com + r1);
  s.w.add_force_at(s.b, f2, com + r2);
  const double dt = s.w.config().fixed_dt;
  s.w.step();
  const body_state st = s.w.state(s.b);
  const vec3 t = cross(r1, f1) + cross(r2, f2); // (-5, -50, 0) N m
  const vec3 want{t.x * dt / free_box::ixx, t.y * dt / free_box::iyy, 0.0};
  std::printf("  angular velocity (%.6f, %.6f, %.6f) rad/s, r x F dt / I gives (%.6f, %.6f, 0)\n",
              st.angular_velocity.x, st.angular_velocity.y, st.angular_velocity.z, want.x, want.y);
  EXPECT_LT(st.angular_velocity.x, 0.0);
  EXPECT_LT(st.angular_velocity.y, 0.0);
  EXPECT_NEAR(st.angular_velocity.x, want.x, 1e-3 * std::abs(want.x));
  EXPECT_NEAR(st.angular_velocity.y, want.y, 1e-3 * std::abs(want.y));
  // Not exactly 0: the second collision sub-step sees the inertia of a body that has
  // turned a little in the first.
  EXPECT_NEAR(st.angular_velocity.z, 0.0, 1e-5);
  EXPECT_NEAR(st.linear_velocity.x, 0.0, 1e-9);
  EXPECT_NEAR(st.linear_velocity.y, 40.0 * dt / free_box::mass, 1e-6);
  EXPECT_NEAR(st.linear_velocity.z, 100.0 * dt / free_box::mass, 1e-6);
  // Like add_force, the forces lasted one step: the next one coasts.
  s.w.step();
  const body_state next = s.w.state(s.b);
  EXPECT_NEAR(next.linear_velocity.z, st.linear_velocity.z, 1e-7);
  EXPECT_NEAR(next.angular_velocity.y, st.angular_velocity.y, 1e-3 * std::abs(want.y));
}

// A force at the centre of mass is add_force, to the bit: the same state hash after
// a run that pushes a spinning body every step.
TEST(PhysicsJolt, AForceAtTheCentreOfMassIsAddForce) {
  REQUIRE_JOLT();
  const vec3 spin{0.3, -0.2, 0.1};
  free_box a(spin), b(spin);
  for (int k = 0; k < 120; ++k) {
    const vec3 f{20.0 * std::sin(0.1 * k), 15.0 * std::cos(0.07 * k), 5.0};
    a.w.add_force(a.b, f);
    b.w.add_force_at(b.b, f, b.w.state(b.b).position);
    a.w.step();
    b.w.step();
  }
  EXPECT_EQ(a.w.state_hash(), b.w.state_hash());
  EXPECT_EQ(a.w.state(a.b).position.x, b.w.state(b.b).position.x);
  EXPECT_GT(a.w.state(a.b).position.z - 20.0, 0.02) << "the pushes moved it";
}

// body_state::position is the body's origin, not its centre of mass. For a box they
// coincide; for a hull whose points are not centred on the origin they do not, and a
// force at the origin torques it. A hull cube with its origin at a corner: its centre of
// mass is the cube's centre, half a metre along each axis.
TEST(PhysicsJolt, AForceAtAHullsOriginIsOffItsCentreOfMass) {
  REQUIRE_JOLT();
  world_config c = jolt_config();
  c.gravity = {};
  auto cube = [&](world &w) {
    convex_hull h;
    for (int i = 0; i < 8; ++i)
      h.points.push_back({double(i & 1), double((i >> 1) & 1), double((i >> 2) & 1)});
    body_desc d{h, motion_type::dynamic, {0.0, 0.0, 20.0}};
    d.linear_damping = 0.0;
    d.angular_damping = 0.0;
    return w.add_body(d);
  };
  const vec3 f{0.0, 0.0, 50.0};
  world at_origin(c);
  const body_id a = cube(at_origin);
  at_origin.add_force_at(a, f, at_origin.state(a).position);
  at_origin.step();
  // r = (-0.5, -0.5, -0.5) from the centre of mass, so r x F = (-25, 25, 0) N m.
  const vec3 wa = at_origin.state(a).angular_velocity;
  EXPECT_LT(wa.x, -1e-3);
  EXPECT_GT(wa.y, 1e-3);
  EXPECT_NEAR(wa.x, -wa.y, 1e-3 * std::abs(wa.y)) << "the cube is symmetric";
  // At the centre of mass (unrotated, so origin + (0.5, 0.5, 0.5)) it only pushes.
  world at_com(c);
  const body_id b = cube(at_com);
  at_com.add_force_at(b, f, at_com.state(b).position + vec3{0.5, 0.5, 0.5});
  at_com.step();
  const vec3 wb = at_com.state(b).angular_velocity;
  EXPECT_LT(std::abs(wb.x) + std::abs(wb.y) + std::abs(wb.z), 1e-3 * std::abs(wa.y));
  EXPECT_GT(at_com.state(b).linear_velocity.z, 0.0);
}

// A torque turns the body about its centre of mass and does not move it.
TEST(PhysicsJolt, ATorqueSpinsWithoutTranslating) {
  REQUIRE_JOLT();
  free_box s;
  const vec3 at = s.w.state(s.b).position;
  const vec3 t{2.0, -3.0, 5.0};
  s.w.add_torque(s.b, t);
  const double dt = s.w.config().fixed_dt;
  s.w.step();
  const body_state st = s.w.state(s.b);
  EXPECT_NEAR(st.angular_velocity.x, t.x * dt / free_box::ixx, 1e-3 * t.x * dt / free_box::ixx);
  EXPECT_NEAR(st.angular_velocity.y, t.y * dt / free_box::iyy, 1e-3 * -t.y * dt / free_box::iyy);
  EXPECT_NEAR(st.angular_velocity.z, t.z * dt / free_box::izz, 1e-3 * t.z * dt / free_box::izz);
  s.w.step(100);
  const body_state later = s.w.state(s.b);
  EXPECT_EQ(later.linear_velocity.x, 0.0);
  EXPECT_EQ(later.linear_velocity.y, 0.0);
  EXPECT_EQ(later.linear_velocity.z, 0.0);
  EXPECT_NEAR(later.position.x, at.x, 1e-9);
  EXPECT_NEAR(later.position.y, at.y, 1e-9);
  EXPECT_NEAR(later.position.z, at.z, 1e-9);
  EXPECT_GT(std::abs(later.rotation.x) + std::abs(later.rotation.y) + std::abs(later.rotation.z),
            1e-3)
      << "it turned";
}

// As add_force does, both wake a sleeping body.
TEST(PhysicsJolt, ForceAtAndTorqueWakeASleepingBody) {
  REQUIRE_JOLT();
  world w(jolt_config());
  w.add_body({box{{50.0, 50.0, 0.5}}, motion_type::static_, {0, 0, -0.5}});
  const body_id crate = w.add_body({box{{0.25, 0.25, 0.25}}, motion_type::dynamic, {0, 0, 0.25}});
  const auto sleep = [&] {
    w.step(300);
    return !w.state(crate).active;
  };
  ASSERT_TRUE(sleep()) << "the crate never fell asleep";
  w.add_force(crate, {0.0, 0.0, 1.0});
  EXPECT_TRUE(w.state(crate).active) << "add_force";
  ASSERT_TRUE(sleep());
  w.add_force_at(crate, {0.0, 0.0, 1.0}, w.state(crate).position + vec3{0.25, 0.0, 0.0});
  EXPECT_TRUE(w.state(crate).active) << "add_force_at";
  ASSERT_TRUE(sleep());
  w.add_torque(crate, {0.0, 0.0, 1.0});
  EXPECT_TRUE(w.state(crate).active) << "add_torque";
}

// Both refuse values that are not finite and ids that are not bodies, on every backend;
// the null backend checks the id and otherwise does nothing, as for add_force.
TEST(PhysicsWorld, ForceAtAndTorqueAreChecked) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<world_config> configs{null_config()};
  if (have_jolt())
    configs.push_back(jolt_config());
  for (const world_config &c : configs) {
    SCOPED_TRACE(backend_name(c.backend));
    world w(c);
    const body_id ground =
        w.add_body({box{{5.0, 5.0, 0.5}}, motion_type::static_, {0.0, 0.0, -10.0}});
    const body_id rock = w.add_body({box{}, motion_type::dynamic, {0, 0, 5}});
    const body_id gone = w.add_body({sphere{}, motion_type::dynamic, {3, 0, 5}});
    const body_id mover = w.add_body({box{}, motion_type::kinematic, {-6.0, 0.0, 5.0}});
    w.remove_body(gone);
    EXPECT_THROW(w.add_force_at(rock, {nan, 0, 0}, {0, 0, 5}), cvc::physics_error);
    EXPECT_THROW(w.add_force_at(rock, {0, 0, 1}, {0, inf, 5}), cvc::physics_error);
    EXPECT_THROW(w.add_torque(rock, {0, 0, -inf}), cvc::physics_error);
    EXPECT_THROW(w.add_force_at(gone, {0, 0, 1}, {0, 0, 5}), cvc::physics_error);
    EXPECT_THROW(w.add_torque(gone, {0, 0, 1}), cvc::physics_error);
    EXPECT_THROW(w.add_force_at(body_id{}, {0, 0, 1}, {0, 0, 5}), cvc::physics_error);
    EXPECT_THROW(w.add_torque(body_id{}, {0, 0, 1}), cvc::physics_error);
    EXPECT_THROW(w.add_force(gone, {0, 0, 1}), cvc::physics_error) << "as add_force does";
    // A static body and a kinematic one take them and do not move.
    const body_state g = w.state(ground);
    const body_state k = w.state(mover);
    EXPECT_NO_THROW(w.add_force_at(ground, {0, 0, 1e6}, {5.0, 0.0, -10.0}));
    EXPECT_NO_THROW(w.add_torque(ground, {1e6, 0, 0}));
    EXPECT_NO_THROW(w.add_force_at(mover, {1e6, 0, 0}, {-6.0, 0.5, 5.5}));
    EXPECT_NO_THROW(w.add_torque(mover, {0, 1e6, 0}));
    EXPECT_NO_THROW(w.add_force_at(rock, {1, 0, 0}, {0.0, 0.5, 5.0}));
    EXPECT_NO_THROW(w.add_torque(rock, {0, 0, 1}));
    w.step();
    EXPECT_EQ(w.state(ground).position.z, g.position.z);
    EXPECT_EQ(w.state(ground).rotation.w, g.rotation.w);
    const body_state k2 = w.state(mover);
    EXPECT_EQ(k2.position.x, k.position.x);
    EXPECT_EQ(k2.position.y, k.position.y);
    EXPECT_EQ(k2.position.z, k.position.z);
    EXPECT_EQ(k2.rotation.w, k.rotation.w);
    EXPECT_EQ(k2.rotation.x, k.rotation.x);
    EXPECT_EQ(k2.rotation.y, k.rotation.y);
    EXPECT_EQ(k2.rotation.z, k.rotation.z);
    EXPECT_EQ(k2.linear_velocity.x, 0.0);
    EXPECT_EQ(k2.linear_velocity.y, 0.0);
    EXPECT_EQ(k2.linear_velocity.z, 0.0);
    EXPECT_EQ(k2.angular_velocity.x, 0.0);
    EXPECT_EQ(k2.angular_velocity.y, 0.0);
    EXPECT_EQ(k2.angular_velocity.z, 0.0);
    if (c.backend == backend::null) {
      EXPECT_DOUBLE_EQ(w.state(rock).position.z, 5.0);
      EXPECT_EQ(w.state(rock).angular_velocity.z, 0.0);
    }
  }
}

// --- per-body settings, JSON, the publish thread -------------------------------------------

namespace {

boost::property_tree::ptree parse_json(const std::string &text) {
  std::istringstream in(text);
  boost::property_tree::ptree doc;
  boost::property_tree::read_json(in, doc);
  return doc;
}

double element(const boost::property_tree::ptree &array, int i) {
  auto it = array.begin();
  std::advance(it, i);
  return std::stod(it->second.get_value<std::string>());
}

// How far a box sliding at 4 m/s goes on default-material ground before it stops.
double slide(world &w, body_id box_id) {
  const vec3 from = w.state(box_id).position;
  w.add_impulse(box_id, {4.0 * w.get_body(box_id).mass, 0.0, 0.0});
  w.step(300);
  return w.state(box_id).position.x - from.x;
}

} // namespace

// set_body changes a running body: its material, mass, damping and sleeping.
TEST(PhysicsWorld, SetBodyChangesARunningBody) {
  REQUIRE_JOLT();
  world w(jolt_config());
  const material_id sticky = w.add_material({"sticky", 0.8f});
  const material_id slick = w.add_material({"slick", 0.2f});
  const body_id ground = w.add_body({box{{50.0, 50.0, 0.5}}, motion_type::static_, {0, 0, -0.5}});
  body_desc d{box{{0.25, 0.25, 0.25}}, motion_type::dynamic, {0.0, 0.0, 0.25}};
  d.material = sticky;
  d.linear_damping = 0.0;
  const body_id crate = w.add_body(d);
  w.step(50);
  body_settings s = w.get_body(crate);
  EXPECT_EQ(s.material, sticky);
  EXPECT_NEAR(s.mass, 1000.0 * 0.5 * 0.5 * 0.5, 1e-3) << "density times volume";
  const double sticky_m = slide(w, crate);
  s.material = slick;
  w.set_body(crate, s);
  const double slick_m = slide(w, crate);
  // v^2 / (2 mu g): 1.02 m at mu 0.8, 4.08 m at mu 0.2.
  std::printf("  slid %.3f m on mu 0.8, %.3f m after set_body gave it mu 0.2\n", sticky_m, slick_m);
  EXPECT_NEAR(slick_m / sticky_m, 4.0, 0.4);
  // Twice the mass: an impulse changes its velocity half as much.
  s.mass *= 2.0;
  w.set_body(crate, s);
  EXPECT_EQ(w.get_body(crate).mass, s.mass);
  const double v0 = w.state(crate).linear_velocity.z;
  w.add_impulse(crate, {0.0, 0.0, 250.0});
  EXPECT_NEAR(w.state(crate).linear_velocity.z - v0, 250.0 / s.mass, 1e-4);
  w.step(200);
  // Not allowed to sleep, it stays awake at rest.
  s.allow_sleeping = false;
  w.set_body(crate, s);
  w.step(300);
  EXPECT_TRUE(w.state(crate).active);
  EXPECT_EQ(w.snapshot().bodies[0].settings.allow_sleeping, false);
  // Refused, and nothing changes.
  body_settings g = w.get_body(ground);
  g.linear_damping = 1.0;
  EXPECT_THROW(w.set_body(ground, g), cvc::physics_error); // static: only its material
  g = w.get_body(ground);
  g.mass = 5.0;
  EXPECT_THROW(w.set_body(ground, g), cvc::physics_error);
  const std::uint64_t version = w.snapshot().static_version;
  g = w.get_body(ground);
  g.material = slick; // a static box's material may change
  w.set_body(ground, g);
  EXPECT_EQ(w.get_body(ground).material, slick);
  EXPECT_NE(w.snapshot().static_version, version);
  s = w.get_body(crate);
  s.material = 99;
  EXPECT_THROW(w.set_body(crate, s), cvc::physics_error);
  s = w.get_body(crate);
  s.mass = -1.0;
  EXPECT_THROW(w.set_body(crate, s), cvc::physics_error);
  EXPECT_EQ(w.get_body(crate).material, slick);
  body_id nobody;
  nobody.value = 777;
  EXPECT_THROW(w.get_body(nobody), cvc::physics_error);
  EXPECT_THROW(w.set_body(nobody, s), cvc::physics_error);
}

// A mesh's materials are its triangles': set_body leaves them alone.
TEST(PhysicsWorld, SetBodyRefusesAMeshsMaterial) {
  REQUIRE_JOLT();
  world w(jolt_config());
  const material_id m = w.add_material({"m", 0.5f});
  auto g = std::make_shared<cvc::geometry>();
  g->points() = {{{0, 0, 0}}, {{1, 0, 0}}, {{0, 1, 0}}};
  g->tris().push_back({{0, 1, 2}});
  const body_id id = w.add_body({mesh{g}, motion_type::static_});
  body_settings s = w.get_body(id);
  s.material = m;
  EXPECT_THROW(w.set_body(id, s), cvc::physics_error);
}

// The snapshot as JSON: snapshot_json without the static bodies, snapshot_json_static with
// them, rewritten only when they change.
TEST(PhysicsBinding, TheSnapshotIsPublishedAsJson) {
  REQUIRE_JOLT();
  cvc::app ctx;
  world w(jolt_config());
  world_binding b(ctx, w);
  w.add_material({"ice \"black\"\n", 0.1f}); // a name JSON must escape
  w.add_body({box{{50.0, 50.0, 0.5}}, motion_type::static_, {0.0, 0.0, -0.5}});
  const body_id ball = w.add_body({sphere{0.2}, motion_type::dynamic, {0.0, 0.0, 2.0}});
  set_key(ctx, "physics.snapshot_json_every", "2");
  for (int i = 0; i < 8; ++i)
    b.step();
  const world_snapshot at8 = w.snapshot();
  b.step(); // tick 9: no JSON
  const std::string text = key_value(ctx, "physics.snapshot_json");
  const boost::property_tree::ptree doc = parse_json(text);
  EXPECT_EQ(doc.get<std::string>("format"), "cvc.physics.snapshot/1");
  EXPECT_EQ(doc.get<std::uint64_t>("tick"), 8u) << "every 2 steps";
  EXPECT_EQ(doc.get<std::string>("config.backend"), "jolt");
  ASSERT_EQ(doc.get_child("bodies").size(), 1u) << "the moving bodies only";
  const boost::property_tree::ptree &body = doc.get_child("bodies").begin()->second;
  EXPECT_EQ(body.get<std::uint32_t>("id"), ball.value);
  EXPECT_EQ(body.get<std::string>("motion"), "dynamic");
  // A double reads back exactly: the ball's height at tick 8.
  EXPECT_EQ(element(body.get_child("position"), 2), at8.bodies[0].state.position.z);
  EXPECT_EQ(element(body.get_child("rotation"), 0), at8.bodies[0].state.rotation.w);
  EXPECT_NE(text.find("\"friction\":1.0,"), std::string::npos) << "a double keeps its point";
  std::vector<std::string> names;
  for (const auto &m : doc.get_child("materials"))
    names.push_back(m.second.get<std::string>("name"));
  ASSERT_EQ(names.size(), 2u);
  EXPECT_EQ(names[1], "ice \"black\"\n");
  // The static bodies, in their own document.
  const boost::property_tree::ptree statics =
      parse_json(key_value(ctx, "physics.snapshot_json_static"));
  ASSERT_EQ(statics.get_child("bodies").size(), 1u);
  EXPECT_EQ(statics.get_child("bodies").begin()->second.get<std::string>("motion"), "static");
  EXPECT_EQ(statics.get<std::uint64_t>("static_version"), doc.get<std::uint64_t>("static_version"));
  const std::uint64_t written = statics.get<std::uint64_t>("tick");
  for (int i = 0; i < 4; ++i)
    b.step();
  EXPECT_EQ(parse_json(key_value(ctx, "physics.snapshot_json_static")).get<std::uint64_t>("tick"),
            written)
      << "unchanged static bodies are not rewritten";
  w.add_body({box{{1.0, 1.0, 1.0}}, motion_type::static_, {10.0, 0.0, 1.0}});
  b.step();
  b.step();
  const boost::property_tree::ptree changed =
      parse_json(key_value(ctx, "physics.snapshot_json_static"));
  EXPECT_EQ(changed.get_child("bodies").size(), 2u);
  EXPECT_GT(changed.get<std::uint64_t>("tick"), written);
  EXPECT_EQ(
      changed.get<std::uint64_t>("static_version"),
      parse_json(key_value(ctx, "physics.snapshot_json")).get<std::uint64_t>("static_version"));
  // to_json(all) lists every body.
  EXPECT_EQ(parse_json(to_json(w.snapshot())).get_child("bodies").size(), 3u);
}

// publish_thread: a background thread writes what each push publishes; flush() waits for it.
TEST(PhysicsBinding, APublishThreadWritesWhatAStepPublishes) {
  REQUIRE_JOLT();
  cvc::app ctx;
  world w(jolt_config());
  {
    world_binding b(ctx, w);
    w.add_body({box{{50.0, 50.0, 0.5}}, motion_type::static_, {0.0, 0.0, -0.5}});
    const body_id ball = w.add_body({sphere{0.2}, motion_type::dynamic, {0.0, 0.0, 2.0}});
    b.publish_body("ball", ball);
    set_key(ctx, "physics.publish_thread", "1");
    set_key(ctx, "physics.snapshot_json_every", "1");
    for (int i = 0; i < 50; ++i)
      b.step();
    b.flush();
    // The last publication is written: this tick's.
    EXPECT_EQ(key_value(ctx, "physics.tick"), std::to_string(w.tick()));
    EXPECT_EQ(world_binding::snapshot_from_state(ctx)->tick, w.tick());
    EXPECT_EQ(std::stod(key_value(ctx, "physics.bodies.ball.position.z")),
              w.state(ball).position.z);
    EXPECT_EQ(parse_json(key_value(ctx, "physics.snapshot_json")).get<std::uint64_t>("tick"),
              w.tick());
    EXPECT_FALSE(key_value(ctx, "physics.publish_dropped").empty());
    // Back to writing on the step.
    set_key(ctx, "physics.publish_thread", "0");
    b.step();
    EXPECT_EQ(key_value(ctx, "physics.tick"), std::to_string(w.tick()));
    set_key(ctx, "physics.publish_thread", "1");
    for (int i = 0; i < 20; ++i)
      b.step();
  } // the binding joins its thread, which writes what it holds
  EXPECT_EQ(key_value(ctx, "physics.tick"), std::to_string(w.tick()));
}

// pull() reads the tree only after something under the prefix changed: a program's own
// reconfigure stands until the tree's gravity changes.
TEST(PhysicsBinding, PullReadsTheTreeOnlyWhenItChanged) {
  cvc::app ctx;
  world w(null_config());
  world_binding b(ctx, w);
  b.step();
  world_config c = w.config();
  c.gravity.z = -1.0;
  w.reconfigure(c);
  for (int i = 0; i < 3; ++i)
    b.step(); // the binding's own writes are not changes
  EXPECT_EQ(w.config().gravity.z, -1.0);
  set_key(ctx, "physics.gravity.z", "-3.71");
  b.step();
  EXPECT_EQ(w.config().gravity.z, -3.71);
}

namespace {

// What app `i`'s binding makes of its world: the gravity its tree sets, a program's reconfigure
// standing through steps (the binding's own writes are no change to pull), then the tree's next
// gravity, and the ticks it published.
std::string app_binding(cvc::app &ctx, int i) {
  set_key(ctx, "physics.gravity.z", std::to_string(-1.0 - i));
  world w(world_binding::config_from_state(ctx));
  world_binding b(ctx, w);
  std::ostringstream t;
  b.step();
  t << "tree g " << w.config().gravity.z;
  world_config c = w.config();
  c.gravity.z = -20.0 - i;
  w.reconfigure(c);
  for (int k = 0; k < 4; ++k)
    b.step();
  t << ", program's g " << w.config().gravity.z;
  set_key(ctx, "physics.gravity.z", std::to_string(-40.0 - i));
  b.step();
  t << ", tree's next g " << w.config().gravity.z << ", tick " << key_value(ctx, "physics.tick");
  return t.str();
}

} // namespace

// Bindings of several apps: each sees its own tree, and its own writes (not another binding's, on
// another thread at once) are no change to pull. The same alone, two apps in both orders, and on
// threads at once.
TEST(PhysicsBinding, BindingsOfSeveralAppsAreEachTheirOwn) {
  constexpr int kApps = 4;
  std::vector<std::string> alone(kApps), together(kApps);
  for (int i = 0; i < kApps; ++i) {
    cvc::app ctx;
    alone[i] = app_binding(ctx, i);
  }
  EXPECT_EQ(alone[1], "tree g -2, program's g -21, tree's next g -41, tick 6");
  {
    cvc::app a, b;
    EXPECT_EQ(app_binding(a, 0), alone[0]);
    EXPECT_EQ(app_binding(b, 1), alone[1]);
    cvc::app c, d;
    EXPECT_EQ(app_binding(d, 1), alone[1]);
    EXPECT_EQ(app_binding(c, 0), alone[0]);
  }
  std::vector<std::thread> threads;
  for (int i = 0; i < kApps; ++i)
    threads.emplace_back([i, &together] {
      cvc::app ctx;
      for (int k = 0; k < 3; ++k)
        together[i] = app_binding(ctx, i); // the same app again: the same answers
    });
  for (std::thread &th : threads)
    th.join();
  for (int i = 0; i < kApps; ++i)
    EXPECT_EQ(together[i], alone[i]) << i;
}

// A published body's settings are knobs: the material by name, the mass, damping, sleeping.
TEST(PhysicsBinding, APublishedBodysSettingsAreKnobs) {
  REQUIRE_JOLT();
  cvc::app ctx;
  world w(jolt_config());
  world_binding b(ctx, w);
  set_key(ctx, "physics.materials.ice.friction", "0.05");
  b.pull();
  const body_id crate = w.add_body({box{{0.5, 0.5, 0.5}}, motion_type::dynamic, {0, 0, 2}});
  b.publish_body("crate", crate);
  EXPECT_EQ(key_value(ctx, "physics.bodies.crate.material"), "default");
  // The engine's mass, from the density in float: 1000 kg within its rounding.
  EXPECT_EQ(std::stod(key_value(ctx, "physics.bodies.crate.mass")), w.get_body(crate).mass);
  EXPECT_NEAR(w.get_body(crate).mass, 1000.0, 1e-3);
  EXPECT_EQ(key_value(ctx, "physics.bodies.crate.allow_sleeping"), "1");
  set_key(ctx, "physics.bodies.crate.material", "ice");
  set_key(ctx, "physics.bodies.crate.mass", "250");
  set_key(ctx, "physics.bodies.crate.linear_damping", "0.5");
  set_key(ctx, "physics.bodies.crate.allow_sleeping", "false");
  b.step();
  const body_settings s = w.get_body(crate);
  EXPECT_EQ(s.material, b.material("ice"));
  EXPECT_EQ(s.mass, 250.0);
  EXPECT_EQ(s.linear_damping, 0.5);
  EXPECT_FALSE(s.allow_sleeping);
  set_key(ctx, "physics.bodies.crate.material", "lava"); // no such material: ignored
  set_key(ctx, "physics.bodies.crate.mass", "-3");       // refused: ignored
  EXPECT_NO_THROW(b.step());
  EXPECT_EQ(w.get_body(crate).material, b.material("ice"));
  EXPECT_EQ(w.get_body(crate).mass, 250.0);
}
