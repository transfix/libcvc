/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics::plant_host (plant_host.h): the plant a sim_world drives on, chosen by name, in
// code and in the state tree.

#include <cmath>
#include <cstdint>
#include <cvc/core/app.h>
#include <cvc/core/exception.h>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/sim_world.h>
#include <cvc/physics/backend.h>
#include <cvc/physics/plant_host.h>
#include <cvc/state/state.h>
#include <gtest/gtest.h>
#include <string>
#include <vector>

using namespace cvc::physics;
using cvc::nav::plant_kind;
using cvc::nav::sim_world;

namespace {

#define REQUIRE_JOLT()                                                                             \
  if (!plant_host::available())                                                                    \
  GTEST_SKIP() << "this libcvc was built without Jolt"

constexpr double kS = 0.05; // the trained metric: 1 normalized unit = 20 m

// Three agents on an open 200 m square, in nav's trained metric, heading east.
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

std::string read(cvc::app &app, const std::string &key) {
  try {
    return cvc::state::instance(app)(key).value();
  } catch (const std::exception &) {
    return std::string();
  }
}

} // namespace

// "humvee" puts every agent on a Jolt Humvee, which drives; "kinematic" takes them off again,
// and the sim keeps stepping on nav's bicycle.
TEST(PlantHost, PutsTheAgentsOnVehiclesAndBack) {
  REQUIRE_JOLT();
  sim_world sim = open_sim();
  plant_host host;
  EXPECT_EQ(host.driving(), "kinematic");
  EXPECT_EQ(host.use(sim, "humvee"), "humvee");
  EXPECT_EQ(sim.plant_type(), plant_kind::jolt);
  ASSERT_NE(host.plant(), nullptr);
  ASSERT_NE(host.physics(), nullptr);
  EXPECT_EQ(host.plant()->size(), 3);
  const float x0 = east_of(sim, 1);
  for (int t = 0; t < 100; ++t)
    sim.step();
  std::printf("  on Humvees: agent 1 drove %.1f m east in 6 s\n", east_of(sim, 1) - x0);
  EXPECT_GT(east_of(sim, 1) - x0, 5.0f) << "the Humvee drove toward its goal";
  EXPECT_EQ(host.use(sim, "kinematic"), "kinematic");
  EXPECT_EQ(sim.plant_type(), plant_kind::kinematic);
  EXPECT_EQ(host.plant(), nullptr);
  EXPECT_EQ(host.physics(), nullptr);
  const float x1 = east_of(sim, 1);
  for (int t = 0; t < 20; ++t)
    sim.step();
  EXPECT_NE(east_of(sim, 1), x1) << "back on the bicycle, still driving";
}

// A list of presets is cycled over the agents.
TEST(PlantHost, CyclesAListOfPresets) {
  REQUIRE_JOLT();
  sim_world sim = open_sim();
  plant_host host;
  EXPECT_EQ(host.use(sim, "humvee, roboracer"), "humvee, roboracer");
  ASSERT_NE(host.plant(), nullptr);
  EXPECT_EQ(host.plant()->desc(0).mass, vehicle_desc::humvee().mass);
  EXPECT_EQ(host.plant()->desc(1).mass, vehicle_desc::roboracer().mass);
  EXPECT_EQ(host.plant()->desc(2).mass, vehicle_desc::humvee().mass);
  for (int t = 0; t < 10; ++t)
    sim.step();
}

// An unknown name throws and changes nothing.
TEST(PlantHost, RefusesAnUnknownVehicleAndKeepsThePlant) {
  REQUIRE_JOLT();
  sim_world sim = open_sim();
  plant_host host;
  host.use(sim, "humvee");
  const nav_plant *before = host.plant();
  EXPECT_THROW(host.use(sim, "tank"), cvc::physics_error);
  EXPECT_THROW(host.use(sim, "humvee,tank"), cvc::physics_error);
  EXPECT_EQ(host.plant(), before);
  EXPECT_EQ(host.driving(), "humvee");
  EXPECT_EQ(sim.plant_type(), plant_kind::jolt);
  EXPECT_THROW(plant_host::preset("tank"), cvc::physics_error);
  EXPECT_THROW(plant_host::check("humvee, tank"), cvc::physics_error);
  EXPECT_NO_THROW(plant_host::check("humvee, roboracer"));
  EXPECT_NO_THROW(plant_host::check("kinematic"));
}

// <prefix>.nav_plant.vehicle switches the plant live; the host's telemetry and the plant's own
// keys follow.
TEST(PlantHost, SwitchesInTheStateTree) {
  REQUIRE_JOLT();
  cvc::app app;
  sim_world sim = open_sim();
  plant_host host;
  host.pull(app, "demo", sim);
  EXPECT_EQ(read(app, "demo.nav_plant.vehicle"), "kinematic") << "seeded with the current one";
  EXPECT_EQ(read(app, "demo.nav_plant.available"), "1");
  cvc::state::instance(app)("demo.nav_plant.vehicle").value(std::string("roboracer"));
  host.pull(app, "demo", sim);
  EXPECT_EQ(sim.plant_type(), plant_kind::jolt);
  EXPECT_EQ(read(app, "demo.nav_plant.driving"), "roboracer");
  EXPECT_FALSE(read(app, "demo.nav_plant.latency").empty()) << "the plant's own keys";
  cvc::state::instance(app)("demo.nav_plant.vehicle").value(std::string("tank"));
  host.pull(app, "demo", sim);
  EXPECT_EQ(host.driving(), "roboracer") << "an unknown name is ignored";
  cvc::state::instance(app)("demo.nav_plant.vehicle").value(std::string("kinematic"));
  host.pull(app, "demo", sim);
  EXPECT_EQ(sim.plant_type(), plant_kind::kinematic);
  EXPECT_EQ(read(app, "demo.nav_plant.driving"), "kinematic");
}

// What a demo's convoy costs on the plant, and that worker threads change only that: 32 Humvees
// step to the same state_hash with Jolt's job system on 0 and 3 worker threads.
TEST(PlantHost, WorkerThreadsCostLessAndChangeNothing) {
  REQUIRE_JOLT();
  constexpr int n = 32;
  std::vector<float> o(2 * n), g(2 * n), col(3 * n, 1.0f);
  for (int i = 0; i < n; ++i) {
    o[2 * i] = static_cast<float>(-80.0 * kS);
    o[2 * i + 1] = static_cast<float>((-80.0 + 5.0 * i) * kS);
    g[2 * i] = static_cast<float>(80.0 * kS);
    g[2 * i + 1] = o[2 * i + 1];
  }
  const sim_world base = open_sim();
  const auto run = [&](int threads, double &us) {
    constexpr int R = 101, C = 101;
    static const std::vector<std::uint8_t> occ(static_cast<std::size_t>(R) * C, 0);
    sim_world::config c = base.current_config();
    sim_world sim(c, occ.data(), occ.data(), cvc::nav::coef_mlp::default_biased(), o.data(),
                  g.data(), col.data(), n);
    plant_host_options po;
    po.world.threads = threads;
    plant_host host(po);
    host.use(sim, "humvee");
    for (int t = 0; t < 100; ++t)
      sim.step();
    us = host.plant()->advance_us();
    return host.physics()->state_hash();
  };
  double us0 = 0.0, us3 = 0.0;
  const std::uint64_t h0 = run(0, us0), h3 = run(3, us3);
  std::printf("  %d Humvees: %.0f us a tick single-threaded, %.0f us on 3 workers (real time "
              "allows 60000 us)\n",
              n, us0, us3);
  EXPECT_EQ(h0, h3) << "the job system's threads changed the result";
}
