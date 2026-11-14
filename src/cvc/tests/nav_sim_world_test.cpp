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
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/coef_train.h> // city_scene, training_scene
#include <cvc/nav/detail/seeded_draw.h>
#include <cvc/nav/sim_world.h>
#include <cvc/state/state.h>
#include <gtest/gtest.h>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
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

// What app `i` makes of a world of its own: 12 + 4i agents, its sensing range from its tree
// (pull), a few ticks, and what publish wrote back to its tree, with the agents' poses.
std::string app_world(cvc::app &app, int i) {
  cvc::state::instance(app)("multi.nav.range_m").value(std::to_string(20 + 5 * i));
  sim_world w = build(12 + 4 * i, sim_world::belief_mode::shared, 1);
  w.pull(app, "multi");
  for (int k = 0; k < 8; ++k)
    w.step(1);
  w.publish(app, "multi");
  std::vector<float> pos(2 * w.size()), head(w.size());
  w.snapshot(pos.data(), head.data(), nullptr, nullptr, nullptr);
  double sum = 0.0;
  for (float v : pos)
    sum += v;
  const auto key = [&](const char *k) { return cvc::state::instance(app)(k).value(); };
  char b[64];
  std::snprintf(b, sizeof(b), " poses %.5f", sum);
  return "tick " + key("multi.nav.tick") + " agents " + key("multi.nav.settings.agents") +
         " range " + key("multi.nav.range_m") + " policy 0 '" + w.policy_key(0) + "'" + b;
}

// Worlds of several apps are each their own: one reads its app's tree and writes it, and its
// answers are the same alone, with another app's world made and stepped first (in either order),
// and on threads at once.
TEST(NavSimWorld, WorldsOfSeveralAppsAreEachTheirOwn) {
  constexpr int kApps = 3;
  std::vector<std::string> alone(kApps), together(kApps);
  for (int i = 0; i < kApps; ++i) {
    cvc::app app;
    alone[i] = app_world(app, i);
  }
  for (int i = 1; i < kApps; ++i)
    EXPECT_NE(alone[i], alone[0]) << i;
  {
    cvc::app a, b;
    EXPECT_EQ(app_world(a, 0), alone[0]);
    EXPECT_EQ(app_world(b, 1), alone[1]);
    cvc::app c, d;
    EXPECT_EQ(app_world(d, 1), alone[1]);
    EXPECT_EQ(app_world(c, 0), alone[0]);
  }
  std::vector<std::thread> threads;
  for (int i = 0; i < kApps; ++i)
    threads.emplace_back([i, &together] {
      cvc::app app;
      together[i] = app_world(app, i);
    });
  for (std::thread &t : threads)
    t.join();
  for (int i = 0; i < kApps; ++i)
    EXPECT_EQ(together[i], alone[i]) << i;
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

// scatter_free picks the same cells for a seed on every standard library.
// std::uniform_int_distribution leaves its mapping to each, and libc++'s scattered other agents
// than libstdc++'s and MSVC's. An open 60 x 50 field whose world units are its cells, so each
// start and goal reads back as (column, row).
TEST(NavSimWorld, ScatterFreePicksTheSameCellsOnEveryStandardLibrary) {
  sim_world::config cfg;
  cfg.rows = 50;
  cfg.cols = 60;
  cfg.max_x = 59.0;
  cfg.max_y = 49.0;
  const std::vector<std::uint8_t> occ(50 * 60, 0);
  std::vector<float> o(8), goal(8), color(12);
  sim_world::scatter_free(cfg, occ.data(), 4, 11, o.data(), goal.data(), color.data());
  // Start (column, row), goal (column, row). libc++'s own draw began {25, 32, 24, 18}.
  const long want[4][4] = {{0, 9, 24, 3}, {42, 9, 14, 36}, {16, 24, 17, 11}, {18, 41, 5, 47}};
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(std::lround(o[2 * i]), want[i][0]) << "agent " << i;
    EXPECT_EQ(std::lround(o[2 * i + 1]), want[i][1]) << "agent " << i;
    EXPECT_EQ(std::lround(goal[2 * i]), want[i][2]) << "agent " << i;
    EXPECT_EQ(std::lround(goal[2 * i + 1]), want[i][3]) << "agent " << i;
  }
}

// from_occupancy's clustered groups are the same for a seed on every standard library. Its
// k-means starts from the first K of a shuffle of the agents, and std::shuffle's algorithm is each
// library's own: with it, MSVC and libc++ grouped these twelve agents {0 1 1 0 2 2 2 2 1 1 2 1}.
// The field is ScatterFreePicksTheSameCellsOnEveryStandardLibrary's. At every k-means step each
// agent's squared distance to its own centre is at least 15% below its next nearest, so no
// difference in float rounding changes a group.
TEST(NavSimWorld, ClusteredGroupsAreTheSameOnEveryStandardLibrary) {
  sim_world::config cfg;
  cfg.rows = 50;
  cfg.cols = 60;
  cfg.max_x = 59.0;
  cfg.max_y = 49.0;
  const std::vector<std::uint8_t> occ(50 * 60, 0);
  const sim_world w = sim_world::from_occupancy(cfg, occ.data(), coef_mlp::default_biased(), 12, 20,
                                                sim_world::belief_mode::clustered, 3);
  ASSERT_EQ(w.planes(), 3);
  const std::vector<int> groups(w.agent_planes(), w.agent_planes() + 12);
  EXPECT_EQ(groups, (std::vector<int>{0, 0, 0, 1, 1, 2, 1, 2, 2, 0, 0, 0}));
}

// detail::seeded_shuffle orders the same for a seed on every standard library: an odd length, an
// even one (a lone first swap, then pairs) and one past the paired draws (n * n over 32 bits).
TEST(NavSimWorld, SeededShuffleIsTheSameOnEveryStandardLibrary) {
  const auto first10 = [](int n) {
    std::vector<int> v(n);
    std::iota(v.begin(), v.end(), 0);
    std::mt19937 rng(43);
    cvc::nav::detail::seeded_shuffle(v.begin(), v.end(), rng);
    v.resize(std::min(n, 10));
    return v;
  };
  EXPECT_EQ(first10(9), (std::vector<int>{7, 3, 1, 5, 6, 0, 4, 8, 2}));
  EXPECT_EQ(first10(10), (std::vector<int>{6, 8, 9, 4, 2, 7, 5, 1, 3, 0}));
  EXPECT_EQ(first10(70000),
            (std::vector<int>{55025, 17613, 44640, 7929, 43870, 35975, 9981, 29948, 41220, 8475}));
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

// set_truth: a wall raised in an open field mid-run. truth() has it at once; the agent's belief
// gains it from the next sense tick (a ray reaches it), and never before; taken down again, the
// belief clears as the sensor sees free space there. Fog-of-war stories on the native world (a
// blocker that appears, traffic) rely on this.
TEST(NavSimWorld, SetTruthIsSensedFromTheNextSenseTick) {
  const training_scene ts = city_scene(96);
  sim_world::config cfg = make_cfg(ts);
  cfg.sense_every = 2;
  const std::size_t hw = static_cast<std::size_t>(ts.rows) * ts.cols;
  const std::vector<std::uint8_t> open(hw, 0);
  const float o[2] = {0.0f, 0.0f}, g[2] = {0.0f, 0.0f}, col[3] = {1, 1, 1};
  sim_world w(cfg, open.data(), open.data(), coef_mlp::default_biased(), o, g, col, 1);
  w.step(0);

  // The agent's cell, and a wall 4..5 cells east of it, 13 cells tall.
  float pos[2];
  w.snapshot(pos, nullptr, nullptr, nullptr, nullptr);
  const int r =
      static_cast<int>(std::lround((pos[1] - ts.min_y) / (ts.max_y - ts.min_y) * (ts.rows - 1)));
  const int c =
      static_cast<int>(std::lround((pos[0] - ts.min_x) / (ts.max_x - ts.min_x) * (ts.cols - 1)));
  std::vector<std::uint8_t> walled = open;
  for (int rr = r - 6; rr <= r + 6; ++rr)
    for (int cc = c + 4; cc <= c + 5; ++cc)
      walled[static_cast<std::size_t>(rr) * ts.cols + cc] = 1;
  const auto believed = [&] {
    int k = 0;
    for (std::size_t i = 0; i < hw; ++i)
      k += walled[i] && w.belief_occ(0)[i];
    return k;
  };
  ASSERT_EQ(believed(), 0) << "an open field, believed open";

  w.set_truth(walled.data());
  EXPECT_TRUE(std::equal(walled.begin(), walled.end(), w.truth())) << "truth() reads it at once";
  EXPECT_EQ(believed(), 0) << "nothing is believed before the agent senses it";
  for (int k = 0; k < cfg.sense_every; ++k)
    w.step(0);
  const int seen = believed();
  EXPECT_GT(seen, 4) << "the near face is believed after a sense tick";

  w.set_truth(open.data());
  for (int k = 0; k < 12 * cfg.sense_every; ++k)
    w.step(0);
  EXPECT_LT(believed(), seen) << "taken down, the belief clears where the sensor sees free space";
  EXPECT_THROW(w.set_truth(nullptr), std::invalid_argument);
}

} // namespace
