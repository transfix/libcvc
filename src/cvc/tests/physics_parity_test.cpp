/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Python parity (P6, docs/roadmap/NAV-VEHICLE-PHYSICS-ROADMAP.md): the reference scenarios that
// bindings/pycvc/test_pycvc_physics.py rebuilds through pycvc, call for call. Each runs twice
// here (the run is a function of its calls); with CVC_PHYSICS_PARITY_JSON set, the values are
// written to that file, and the Python test requires the same ones, bit for bit. Change a
// scenario here and its twin there together.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cvc/geometry/height_field.h>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/sim_world.h>
#include <cvc/physics/backend.h>
#include <cvc/physics/nav_plant.h>
#include <cvc/physics/vehicle.h>
#include <cvc/physics/world.h>
#include <gtest/gtest.h>
#include <map>
#include <memory>
#include <string>
#include <vector>

using namespace cvc::physics;

#define REQUIRE_JOLT()                                                                             \
  do {                                                                                             \
    if (!backend_available(backend::jolt))                                                         \
      GTEST_SKIP() << "this build has no Jolt backend";                                            \
  } while (0)

namespace {

using values = std::map<std::string, double>;

std::string hex(std::uint64_t h) {
  char b[24];
  std::snprintf(b, sizeof(b), "%016llx", static_cast<unsigned long long>(h));
  return b;
}

// The values file: {"<scenario>": {"hash": "<hex>", "<key>": <number>, ...}, ...}.
void write_reference(const std::string &name, std::uint64_t hash, const values &v) {
  const char *path = std::getenv("CVC_PHYSICS_PARITY_JSON");
  if (!path || !*path)
    return;
  // One file per run of the binary: each test appends its line; the Python side reads lines.
  std::FILE *f = std::fopen(path, "a");
  ASSERT_NE(f, nullptr) << path;
  std::fprintf(f, "{\"scenario\": \"%s\", \"hash\": \"%s\"", name.c_str(), hex(hash).c_str());
  for (const auto &kv : v)
    std::fprintf(f, ", \"%s\": %.17g", kv.first.c_str(), kv.second);
  std::fprintf(f, "}\n");
  std::fclose(f);
}

// 1. The cross-platform golden scene (physics_world_test's exact_scene_hash): a height field of
// exactly representable heights and nine bodies, pushed for 240 steps.
std::uint64_t exact_scene() {
  world w; // jolt, cross-platform determinism, 0.01 s
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
  for (int s = 0; s < 240; ++s) {
    w.add_force(bodies[static_cast<size_t>(s) % bodies.size()],
                {(s % 7 - 3) * 2.0, (s % 5 - 2) * 4.0, 0.0});
    w.step();
  }
  return w.state_hash();
}

// The ground both vehicle scenarios drive on: a 300 m box, mu 0.9, C_rr 0.015.
void ground(world &w) {
  const material_id m = w.add_material({"ground", 0.9f, 0.0f, 0.015f});
  body_desc g;
  g.shape = box{{150.0, 150.0, 0.5}};
  g.motion = motion_type::static_;
  g.position = {0.0, 0.0, -0.5};
  g.material = m;
  w.add_body(g);
}

// 2. A Humvee through the driver model: 3 s at 1.5 m/s^2 with a little left steer, then 1 s at
// -3 m/s^2.
std::uint64_t vehicle_scene(values &v) {
  world w;
  ground(w);
  const vehicle_id car = w.add_vehicle(vehicle_desc::humvee(), {0.0, 0.0, 1.2});
  for (int s = 0; s < 400; ++s) {
    drive_command c;
    c.steer = s < 300 ? 0.05 : 0.0;
    c.accel = s < 300 ? 1.5 : -3.0;
    w.drive(car, c);
    w.step();
  }
  const vehicle_state st = w.vehicle_state(car);
  v["x"] = st.body.position.x;
  v["y"] = st.body.position.y;
  v["z"] = st.body.position.z;
  v["qw"] = st.body.rotation.w;
  v["speed"] = st.speed;
  v["engine_rpm"] = st.engine_rpm;
  v["gear"] = st.gear;
  v["supplied"] = st.energy.supplied;
  v["source"] = st.energy.source;
  v["fuel_used"] = st.energy.fuel_used;
  v["distance"] = st.energy.distance;
  v["wheel0_load"] = st.wheels.at(0).load;
  return w.state_hash();
}

// 3. A sim_world's agent on the Jolt plant: an 81 m grid of 1 m cells in the trained metric
// (scale 0.05), the built-in policy, a Humvee from (-20, 0) toward (20, 0) for 150 ticks.
std::uint64_t plant_scene(values &v) {
  cvc::nav::sim_world::config c;
  c.rows = 81;
  c.cols = 81;
  c.min_x = -40.0;
  c.min_y = -40.0;
  c.max_x = 40.0;
  c.max_y = 40.0;
  c.cx = 0.0;
  c.cy = 0.0;
  c.scale = 0.05;
  c.range_m = 60.0;
  c.n_rays = 240;
  c.fov_rad = 6.283185307179586;
  c.veh.rr = static_cast<float>(0.15);
  c.veh.d_hat = static_cast<float>(0.35);
  c.veh.dt = static_cast<float>(0.06);
  c.veh.vmax = static_cast<float>(0.4);
  c.veh.L = static_cast<float>(0.165);
  c.veh.delta_max = static_cast<float>(0.6);
  c.veh.a_max = static_cast<float>(0.12);
  c.veh.a_lat_max = static_cast<float>(0.12);
  c.veh.k_steer = static_cast<float>(0.8);
  c.veh.nsub = 6;
  c.veh.allow_reverse = true;
  c.reach_tol = static_cast<float>(0.2);
  c.sense_every = 4;
  c.freeze_sense = true;
  c.l_occ = 2.2;
  c.l_free = -1.4;
  c.l_clamp = 8.0;
  c.optimistic = true;
  c.p_thresh = 0.5;
  c.band = 0.15;
  c.ttl_s = 4.0;
  const std::vector<std::uint8_t> occ(81 * 81, 0);
  const float o[2] = {-1.0f, 0.0f}, g[2] = {1.0f, 0.0f}, col[3] = {1.0f, 1.0f, 1.0f};
  cvc::nav::sim_world sim(c, occ.data(), occ.data(), cvc::nav::coef_mlp::default_biased(), o, g,
                          col, 1);
  world w;
  ground(w);
  nav_plant plant(w, {vehicle_desc::humvee()});
  sim.set_plant(&plant);
  for (int t = 0; t < 150; ++t)
    sim.step();
  const cvc::nav::plant_state s = plant.state(0);
  v["x"] = s.x;
  v["y"] = s.y;
  v["heading"] = s.heading;
  v["speed"] = s.speed;
  v["tire_use"] = s.tire_use;
  const vehicle_state vs = w.vehicle_state(plant.vehicle(0));
  v["source"] = vs.energy.source;
  v["distance"] = vs.energy.distance;
  const std::vector<nav_plant::drift_bin> bins = plant.drift_summary();
  v["bin0_ticks"] = static_cast<double>(bins.at(0).ticks);
  v["bin0_position_rms_m"] = bins.at(0).position_rms_m;
  v["sim_tick"] = static_cast<double>(sim.tick());
  return w.state_hash();
}

} // namespace

TEST(PhysicsParity, TheGoldenScene) {
  REQUIRE_JOLT();
  const std::uint64_t h = exact_scene();
  EXPECT_EQ(h, exact_scene());
  if (backend_capabilities(backend::jolt) & cap_determinism_cross_platform)
    EXPECT_EQ(h, 0x70b2cc0bd34ea4beull) << hex(h);
  write_reference("exact", h, {});
}

TEST(PhysicsParity, AHumveeThroughTheDriverModel) {
  REQUIRE_JOLT();
  values a, b;
  const std::uint64_t h = vehicle_scene(a);
  EXPECT_EQ(h, vehicle_scene(b));
  EXPECT_EQ(a, b);
  EXPECT_GT(a["distance"], 5.0) << "it drove";
  std::printf("  the Humvee: %.3f m, %.3f m/s, %.0f J from the fuel, hash %s\n", a["distance"],
              a["speed"], a["source"], hex(h).c_str());
  write_reference("vehicle", h, a);
}

TEST(PhysicsParity, AnAgentOnTheJoltPlant) {
  REQUIRE_JOLT();
  values a, b;
  const std::uint64_t h = plant_scene(a);
  EXPECT_EQ(h, plant_scene(b));
  EXPECT_EQ(a, b);
  EXPECT_GT(a["distance"], 10.0) << "it drove";
  std::printf("  the agent: at (%.4f, %.4f) normalized, %.3f m driven, hash %s\n", a["x"], a["y"],
              a["distance"], hex(h).c_str());
  write_reference("plant", h, a);
}
