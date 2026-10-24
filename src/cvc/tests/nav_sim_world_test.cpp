/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// nav_sim_world_test — a headless smoke test for sim_world construction and the
// renderer-facing readback, exercising exactly the path nav_city_swarm runs at
// startup BEFORE it ever opens a GL context: city_scene -> from_occupancy ->
// size/planes/agent_planes -> snapshot -> step. No cvcGL, no VTK, no OpenGL.
//
// Why this exists: that path once crashed the demo at startup, and the root
// cause was an ABI/layout skew — sim_world embeds `config` (which contains
// veh_params) and `material_config`, both of which grew in the headers; a
// cvc.dll built from the OLDER layout returned a wrong-sized sim_world by value,
// so size()/planes() read garbage and the by-value return smashed the caller's
// stack. The garbage was invisible to AddressSanitizer (it was a layout
// mismatch, not a heap error). `EXPECT_EQ(w.size(), N)` below is the canary: it
// fails loudly ("expected 512, got 236190608") the instant a consumer is built
// against headers whose sim_world layout disagrees with the linked cvc.dll —
// e.g. the published bundle lagging the source tree. In a consistent build it is
// a cheap regression test for from_occupancy + snapshot.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/coef_train.h> // city_scene, training_scene
#include <cvc/nav/sim_world.h>
#include <gtest/gtest.h>
#include <vector>

using cvc::nav::city_scene;
using cvc::nav::coef_mlp;
using cvc::nav::sim_world;
using cvc::nav::training_scene;

namespace {

// Mirror nav_city_swarm's config assembly from a training_scene.
sim_world::config make_cfg(const training_scene &ts) {
  sim_world::config cfg;
  cfg.rows = ts.rows;
  cfg.cols = ts.cols;
  cfg.min_x = ts.min_x;
  cfg.min_y = ts.min_y;
  cfg.max_x = ts.max_x;
  cfg.max_y = ts.max_y;
  cfg.cx = ts.cx;
  cfg.cy = ts.cy;
  cfg.scale = ts.scale;
  cfg.veh.rr = ts.rr;
  cfg.veh.d_hat = ts.d_hat;
  cfg.veh.dt = ts.dt;
  cfg.veh.vmax = ts.vmax;
  cfg.freeze_sense = false;
  cfg.reach_tol = 0.8f;
  cfg.sep_radius = 3.0f * ts.rr;
  cfg.sep_gain = 1.5f * ts.rr;
  return cfg;
}

sim_world build(int n, sim_world::belief_mode mode, int clusters) {
  training_scene ts = city_scene(96);
  return sim_world::from_occupancy(make_cfg(ts), ts.occ.data(), coef_mlp::default_biased(), n,
                                   /*seed=*/7, mode, clusters);
}

// The renderer-facing invariants the demo relies on every frame. size() is the
// ABI canary; the rest guard from_occupancy/snapshot behaviour.
void check_world(const sim_world &w, int n, int expect_planes) {
  ASSERT_EQ(w.size(), n) << "sim_world::size() must equal the requested agent count; a wrong or "
                            "garbage value means the linked cvc.dll's sim_world layout disagrees "
                            "with this header (ABI skew) — rebuild libcvc.";
  EXPECT_EQ(w.planes(), expect_planes);

  const int *grp = w.agent_planes();
  ASSERT_NE(grp, nullptr);
  for (int i = 0; i < n; ++i)
    EXPECT_TRUE(grp[i] >= 0 && grp[i] < w.planes()) << "agent " << i << " -> plane " << grp[i];

  std::vector<float> pos(2 * n), head(n), spd(n);
  std::vector<int> md(n);
  std::vector<std::uint8_t> rch(n);
  w.snapshot(pos.data(), head.data(), spd.data(), md.data(), rch.data());
  for (int i = 0; i < 2 * n; ++i)
    ASSERT_TRUE(std::isfinite(pos[i])) << "non-finite pose component at " << i;
}

TEST(NavSimWorld, SharedBeliefConstructsAndSnapshots) {
  sim_world w = build(64, sim_world::belief_mode::shared, 1);
  check_world(w, 64, /*expect_planes=*/1); // shared => one belief plane
}

TEST(NavSimWorld, GroupedBeliefMatchesRequestedCounts) {
  const int n = 512;
  const int k = std::max(2, n / 64); // nav_city_swarm's grouped-cluster rule => 8
  sim_world w = build(n, sim_world::belief_mode::clustered, k);
  check_world(w, n, /*expect_planes=*/k);
}

TEST(NavSimWorld, StepsAdvanceWithoutCorruption) {
  const int n = 128;
  sim_world w = build(n, sim_world::belief_mode::clustered, 2);
  std::vector<float> pos(2 * n), head(n);
  for (int f = 0; f < 16; ++f) {
    w.step(0);
    w.snapshot(pos.data(), head.data(), nullptr, nullptr, nullptr);
    ASSERT_EQ(w.size(), n); // size stays stable across steps
  }
  for (int i = 0; i < 2 * n; ++i)
    EXPECT_TRUE(std::isfinite(pos[i]));
}

} // namespace
