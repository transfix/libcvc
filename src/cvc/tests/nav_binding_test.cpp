/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// sim_world's settings in the state tree (sim_world::pull / publish): seeded from the world, in
// its world frame (metres), applied on a change, an invalid value ignored, and every key
// described (cvc/state/state_key_meta.h).

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cvc/core/app.h>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/material.h>
#include <cvc/nav/sim_world.h>
#include <cvc/nav/terrain.h>
#include <cvc/state/state.h>
#include <gtest/gtest.h>
#include <string>
#include <vector>

using namespace cvc::nav;

namespace {

constexpr double kS = 0.05; // the trained metric: 1 normalized unit = 20 m
constexpr int R = 81, C = 81;
constexpr double kLo = -40.0, kHi = 40.0; // 1 m cells

sim_world make_world() {
  sim_world::config c;
  c.rows = R;
  c.cols = C;
  c.min_x = c.min_y = kLo;
  c.max_x = c.max_y = kHi;
  c.scale = kS;
  c.veh.rr = 0.15f;
  c.veh.d_hat = 0.35f;
  c.veh.dt = 0.06f;
  c.veh.vmax = static_cast<float>(8.0 * kS);
  c.veh.L = static_cast<float>(3.3 * kS);
  c.freeze_sense = true;
  std::vector<std::uint8_t> occ(static_cast<std::size_t>(R) * C, 0);
  const float o[2] = {static_cast<float>(-20.0 * kS), 0.0f};
  const float goal[2] = {static_cast<float>(20.0 * kS), 0.0f};
  const float color[3] = {1.0f, 1.0f, 1.0f};
  return sim_world(c, occ.data(), occ.data(), coef_mlp::default_biased(), o, goal, color, 1);
}

double num(cvc::state &st, const std::string &key) { return std::stod(st(key).value()); }

} // namespace

TEST(NavBinding, SeedsTheTreeInMetresAndAppliesAChange) {
  sim_world w = make_world();
  cvc::app app;
  cvc::state &st = cvc::state::instance(app);
  w.pull(app, "scene");
  w.publish(app, "scene");
  // Seeded in the world frame: 8 m/s, a 3.3 m wheelbase, 1.5 normalized/s^2 = 30 m/s^2.
  EXPECT_NEAR(num(st, "scene.nav.vehicle.vmax"), 8.0, 1e-5);
  EXPECT_NEAR(num(st, "scene.nav.vehicle.wheelbase"), 3.3, 1e-5);
  EXPECT_NEAR(num(st, "scene.nav.vehicle.a_max"), 1.5 / kS, 1e-4);
  EXPECT_NEAR(num(st, "scene.nav.range_m"), 60.0, 1e-9);
  EXPECT_EQ(st("scene.nav.freeze_sense").value(), "1");
  EXPECT_EQ(st("scene.nav.settings.agents").value(), "1");
  EXPECT_EQ(st("scene.nav.settings.plant").value(), "kinematic");
  EXPECT_NEAR(num(st, "scene.nav.settings.scale"), kS, 1e-12);
  EXPECT_EQ(st("scene.nav.terrain.active").value(), "0");
  // A change in metres arrives normalized; an invalid one is ignored; a seeded key left alone
  // stays as the world has it.
  st("scene.nav.vehicle.vmax").value(std::string("12"));
  st("scene.nav.sep_radius").value(std::string("4"));
  st("scene.nav.n_rays").value(std::string("90.5"));
  st("scene.nav.p_thresh").value(std::string("2"));
  st("scene.nav.vehicle.allow_reverse").value(std::string("0"));
  w.pull(app, "scene");
  const sim_world::config &c = w.current_config();
  EXPECT_NEAR(c.veh.vmax, 12.0 * kS, 1e-6);
  EXPECT_NEAR(c.sep_radius, 4.0 * kS, 1e-6);
  EXPECT_EQ(c.n_rays, 240) << "a ray count is whole";
  EXPECT_EQ(c.p_thresh, 0.5) << "2 is not a probability";
  EXPECT_FALSE(c.veh.allow_reverse);
  EXPECT_NEAR(c.veh.L, 3.3 * kS, 1e-7);
  // The drive takes the new top speed: it never goes faster.
  double fastest = 0.0;
  for (int t = 0; t < 200; ++t) {
    w.step();
    float pos[2], heading, speed; // world frame: m/s
    int mode;
    std::uint8_t reached;
    w.snapshot(pos, &heading, &speed, &mode, &reached);
    fastest = std::max(fastest, static_cast<double>(speed));
  }
  EXPECT_LE(fastest, 12.0 + 1e-3);
  EXPECT_GT(fastest, 8.5) << "faster than the old cap";
  w.publish(app, "scene");
  EXPECT_EQ(st("scene.nav.tick").value(), "200");
}

TEST(NavBinding, EveryKeySaysWhatItIs) {
  sim_world w = make_world();
  cvc::app app;
  cvc::state &st = cvc::state::instance(app);
  w.pull(app, "");
  w.publish(app, "");
  for (const char *key :
       {"nav.range_m", "nav.reach_tol", "nav.min_gap_iters", "nav.tick", "nav.vehicle.vmax",
        "nav.vehicle.mu_probes", "nav.vehicle.mass", "nav.settings.rows", "nav.settings.plant",
        "nav.settings.dt", "nav.terrain.active"}) {
    EXPECT_FALSE(st(key).comment().empty()) << key;
    EXPECT_TRUE(st(key).has_property("access")) << key;
  }
  EXPECT_EQ(st("nav.vehicle.vmax").property("unit"), "m/s");
  EXPECT_EQ(st("nav.vehicle.vmax").property("access"), "knob");
  EXPECT_EQ(st("nav.vehicle.vmax").property("max"), "40");
  EXPECT_EQ(st("nav.reach_tol").property("unit"), "m");
  EXPECT_EQ(st("nav.settings.rows").property("access"), "build");
  EXPECT_EQ(st("nav.tick").property("access"), "telemetry");
  EXPECT_FALSE(st("nav.settings.plant").has_property("unit")) << "not a number";
}

TEST(NavBinding, MaterialWeightsAreKnobsOnceAMaterialIsSet) {
  sim_world w = make_world();
  cvc::app app;
  cvc::state &st = cvc::state::instance(app);
  w.pull(app, "scene");
  EXPECT_TRUE(st("scene.nav.material.lam_soft").value().empty()) << "no material yet";
  std::vector<float> risk(static_cast<std::size_t>(R) * C, 0.0f);
  std::vector<std::uint8_t> hard(static_cast<std::size_t>(R) * C, 0);
  material_config mc;
  w.set_material(risk.data(), hard.data(), mc);
  w.pull(app, "scene");
  EXPECT_NEAR(num(st, "scene.nav.material.lam_soft"), mc.lam_soft, 1e-6);
  st("scene.nav.material.lam_soft_scale").value(std::string("0.5"));
  st("scene.nav.material.lam_hard").value(std::string("-1"));
  w.pull(app, "scene");
  EXPECT_EQ(w.material_lam_soft_scale(), 0.5f);
  EXPECT_EQ(w.material_lam_hard(), mc.lam_hard) << "a weight is >= 0";
  EXPECT_FALSE(st("scene.nav.material.lam_soft").comment().empty());
}

// P5's terrain settings: pull seeds them and applies a change (rebuilding the planes), an invalid
// value is ignored, and publish says what the terrain is.
TEST(NavBinding, TerrainSettingsRebuildThePlanes) {
  sim_world w = make_world();
  // A ridge with flanks of slope 1.2 and a cliff end at y = 20.
  std::vector<float> z(static_cast<std::size_t>(R) * C);
  for (int r = 0; r < R; ++r)
    for (int c = 0; c < C; ++c) {
      const double x = kLo + c, y = kLo + r;
      z[static_cast<std::size_t>(r) * C + c] =
          static_cast<float>(y > 20.0 ? 0.0 : std::max(0.0, 6.0 - 1.2 * std::fabs(x)));
    }
  terrain_config tc;
  tc.max_grade = 0.68;
  tc.tip_ratio = 1.23;
  w.set_terrain(z.data(), tc);
  cvc::app app;
  cvc::state &st = cvc::state::instance(app);
  w.pull(app, "scene");
  w.publish(app, "scene");
  EXPECT_EQ(st("scene.nav.terrain.active").value(), "1");
  const long blocked = std::stol(st("scene.nav.terrain.infeasible_cells").value());
  EXPECT_GT(blocked, 0) << "the ridge";
  EXPECT_NEAR(num(st, "scene.nav.terrain.max_grade"), 0.68, 1e-9);
  // A vehicle that climbs 1.5 and does not tip finds the flanks feasible; only the cliff end
  // stays infeasible.
  st("scene.nav.terrain.max_grade").value(std::string("1.5"));
  st("scene.nav.terrain.tip_ratio").value(std::string("0"));
  st("scene.nav.terrain.tip_margin").value(std::string("3"));
  st("scene.nav.terrain.lam_grade").value(std::string("2"));
  w.pull(app, "scene");
  w.publish(app, "scene");
  EXPECT_EQ(w.terrain_settings().max_grade, 1.5);
  EXPECT_EQ(w.terrain_settings().tip_margin, 0.8) << "3 is not a share";
  EXPECT_EQ(w.terrain_settings().lam_grade, 2.0f);
  EXPECT_LT(std::stol(st("scene.nav.terrain.infeasible_cells").value()), blocked / 4);
  EXPECT_NEAR(num(st, "scene.nav.terrain.limit"), 1.5, 1e-9);
  w.step();
  EXPECT_EQ(st("scene.nav.terrain.max_grade").property("max"), "1.5");
  EXPECT_EQ(st("scene.nav.terrain.d_hat_m").property("unit"), "m");
  EXPECT_EQ(st("scene.nav.terrain.limit").property("access"), "telemetry");
}
