/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Yielding (nav/yield.h, nav/intent.h, sim_world::set_yield): the geometry, the default hooks,
// off changing nothing, and the two scenes it is for on the kinematic arm, in an open field (the
// city scene's frame with no buildings):
//   - a fast lead behind a slow column, every agent driving east (the demo3 start with the lead
//     at the back), with the hard de-overlap on so contact is a push;
//   - two agents head-on.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/coef_train.h> // city_scene
#include <cvc/nav/intent.h>
#include <cvc/nav/sim_world.h>
#include <cvc/nav/yield.h>
#include <gtest/gtest.h>
#include <vector>

using cvc::nav::agent_intent;
using cvc::nav::coef_mlp;
using cvc::nav::sim_world;
using cvc::nav::yield_config;

namespace {

struct field {
  cvc::nav::training_scene ts = cvc::nav::city_scene(96);
  std::vector<std::uint8_t> open;
  sim_world::config cfg;
  float half = 0.0f; // the field's half-width, normalized
  field() {
    open.assign(ts.occ.size(), 0);
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
    cfg.freeze_sense = true;
    cfg.reach_tol = 0.3f;
    half = static_cast<float>(0.5 * (ts.max_x - ts.min_x) * ts.scale);
  }
  sim_world make(const std::vector<float> &o, const std::vector<float> &goal) const {
    const int n = static_cast<int>(o.size() / 2);
    std::vector<float> color(3 * n, 0.5f);
    return sim_world(cfg, open.data(), open.data(), coef_mlp::default_biased(), o.data(),
                     goal.data(), color.data(), n);
  }
};

yield_config yielding() {
  yield_config y;
  y.horizon_s = 3.0f;
  return y;
}

// The agents' positions in nav's normalized frame (snapshot() gives world metres).
std::vector<float> positions(const sim_world &w) {
  std::vector<float> p(2 * w.size());
  w.snapshot(p.data(), nullptr, nullptr, nullptr, nullptr);
  const sim_world::config &c = w.current_config();
  for (int i = 0; i < w.size(); ++i) {
    p[2 * i] = static_cast<float>((p[2 * i] - c.cx) * c.scale);
    p[2 * i + 1] = static_cast<float>((p[2 * i + 1] - c.cy) * c.scale);
  }
  return p;
}

} // namespace

TEST(NavYield, PathDistanceAndHooks) {
  // The polyline (0,0) -> (2,0) -> (2,2).
  const float path[4] = {2.0f, 0.0f, 2.0f, 2.0f};
  float s, qx, qy, tx, ty;
  float d = cvc::nav::intent_path_distance(0, 0, 0, path, 2, 1.0f, 0.5f, s, qx, qy, tx, ty);
  EXPECT_FLOAT_EQ(d, 0.5f);
  EXPECT_FLOAT_EQ(s, 1.0f);
  EXPECT_FLOAT_EQ(tx, 1.0f);
  d = cvc::nav::intent_path_distance(0, 0, 0, path, 2, 2.5f, 1.0f, s, qx, qy, tx, ty);
  EXPECT_FLOAT_EQ(d, 0.5f);
  EXPECT_FLOAT_EQ(s, 3.0f);
  EXPECT_FLOAT_EQ(ty, 1.0f);
  // Perfect comms: everyone hears everyone else, at once.
  cvc::nav::perfect_intent_channel ch;
  agent_intent a[3];
  for (int i = 0; i < 3; ++i)
    a[i].agent = i;
  ch.publish(a, 3, 1.0);
  std::vector<agent_intent> heard;
  ch.heard(1, heard);
  ASSERT_EQ(heard.size(), 2u);
  EXPECT_EQ(heard[0].agent, 0);
  EXPECT_EQ(heard[1].agent, 2);
  // The priority resolver: equal splits, higher goes first.
  const auto r = cvc::nav::priority_resolver();
  a[0].priority = a[1].priority = 0.0f;
  EXPECT_FLOAT_EQ(r(a[0], a[1]), 0.5f);
  a[1].priority = 5.0f;
  EXPECT_GT(r(a[0], a[1]), 0.99f); // 0 gives way to 1
  EXPECT_LT(r(a[1], a[0]), 0.01f);
}

// Off (the default, or horizon 0) changes no bit of a run; on, it publishes every agent's intent.
TEST(NavYield, OffChangesNothing) {
  const field f;
  const std::vector<float> o = {-1.0f, 0.0f, 1.0f, 0.05f, 0.0f, -1.0f};
  const std::vector<float> g = {1.0f, 0.0f, -1.0f, 0.0f, 0.0f, 1.0f};
  sim_world a = f.make(o, g), b = f.make(o, g), c = f.make(o, g);
  yield_config zero;
  b.set_yield(zero);
  c.set_yield(yielding());
  for (int t = 0; t < 40; ++t) {
    a.step(1);
    b.step(1);
    c.step(1);
  }
  EXPECT_EQ(positions(a), positions(b));
  EXPECT_EQ(a.intents(), nullptr);
  ASSERT_NE(c.intents(), nullptr);
  EXPECT_EQ(c.intents()[2].agent, 2);
  EXPECT_GT(c.intents()[0].n_path, 0);
}

namespace {

struct pass_result {
  int contacts = 0;    // agent-ticks within 2 rr of another agent
  int lead_front = -1; // the first tick the lead is ahead (east) of every follower
  float lead_x = 0.0f; // where the lead ends
};

// A slow column of 5 at y = 0, 0.6 apart, and the lead 1.2 behind it, all bound 5 units east. The
// hard de-overlap is on, so contact is a push.
pass_result lead_behind_column(bool yield_on, int ticks) {
  field f;
  f.cfg.min_gap = 2.0f * f.cfg.veh.rr;
  const int n = 6;
  std::vector<float> o(2 * n), g(2 * n), vmax(n);
  const float x0 = -0.5f * f.half;
  for (int i = 1; i < n; ++i) {
    o[2 * i] = x0 - 0.6f * (i - 1);
    vmax[i] = 0.3f * f.cfg.veh.vmax;
  }
  o[0] = x0 - 0.6f * (n - 2) - 1.2f; // the lead, behind them all
  vmax[0] = f.cfg.veh.vmax;
  for (int i = 0; i < n; ++i) {
    g[2 * i] = x0 + 5.0f;
    g[2 * i + 1] = 0.0f;
  }
  f.cfg.veh.vmax_col = vmax.data();
  sim_world w = f.make(o, g);
  if (yield_on) {
    w.set_yield(yielding());
    w.set_priority(0, 10.0f);
  }
  pass_result r;
  const float touch = 2.0f * f.cfg.veh.rr * 1.02f;
  for (int t = 0; t < ticks; ++t) {
    w.step(1);
    const std::vector<float> p = positions(w);
    for (int i = 0; i < n; ++i)
      for (int j = i + 1; j < n; ++j)
        if (std::hypot(p[2 * i] - p[2 * j], p[2 * i + 1] - p[2 * j + 1]) < touch)
          ++r.contacts;
    bool front = true;
    for (int i = 1; i < n; ++i)
      front = front && p[0] > p[2 * i];
    if (front && r.lead_front < 0)
      r.lead_front = t;
    r.lead_x = p[0];
  }
  return r;
}

} // namespace

TEST(NavYield, TheLeadPassesTheColumnInsteadOfPushingThrough) {
  const int ticks = 300;
  const pass_result off = lead_behind_column(false, ticks), on = lead_behind_column(true, ticks);
  std::printf("  lead behind a slow column: off %d contact pair-ticks, ahead at tick %d (x %.2f); "
              "yielding %d, ahead at tick %d (x %.2f)\n",
              off.contacts, off.lead_front, off.lead_x, on.contacts, on.lead_front, on.lead_x);
  EXPECT_LT(on.contacts, off.contacts / 3 + 1) << "yielding should cut the pushing";
  ASSERT_GE(on.lead_front, 0) << "the lead should get to the front";
  if (off.lead_front >= 0)
    EXPECT_LE(on.lead_front, off.lead_front);
}

TEST(NavYield, TwoAgentsHeadOnPassEachOther) {
  const auto run = [](bool yield_on, float &closest) {
    const field f;
    const float a = 0.4f * f.half;
    const std::vector<float> o = {-a, 0.0f, a, 0.0f}, g = {a, 0.0f, -a, 0.0f};
    sim_world w = f.make(o, g);
    if (yield_on)
      w.set_yield(yielding());
    closest = 1e30f;
    int reached = 0;
    for (int t = 0; t < 600 && reached < 2; ++t) {
      w.step(1);
      const std::vector<float> p = positions(w);
      closest = std::min(closest, std::hypot(p[0] - p[2], p[1] - p[3]));
      reached = 0;
      for (int i = 0; i < 2; ++i)
        reached += std::hypot(p[2 * i] - g[2 * i], p[2 * i + 1] - g[2 * i + 1]) < 0.5f ? 1 : 0;
    }
    return reached;
  };
  float c_off, c_on;
  run(false, c_off);
  const int reached = run(true, c_on);
  const float rr = field().cfg.veh.rr;
  std::printf("  head-on: closest approach %.3f without yielding, %.3f with (2 rr = %.3f); %d of 2 "
              "reached\n",
              c_off, c_on, 2.0f * rr, reached);
  EXPECT_LT(c_off, rr) << "without yielding they drive through each other";
  EXPECT_GT(c_on, 2.0f * rr) << "yielding should keep them apart";
  EXPECT_EQ(reached, 2);
}

namespace {

// A channel that loses everything, and one that delivers a tick late.
struct deaf_channel : cvc::nav::intent_channel {
  void publish(const agent_intent *, int, double) override {}
  void heard(int, std::vector<agent_intent> &out) const override { out.clear(); }
};
struct late_channel : cvc::nav::intent_channel {
  std::vector<agent_intent> now, before;
  void publish(const agent_intent *all, int n, double) override {
    before = now;
    now.assign(all, all + n);
  }
  void heard(int i, std::vector<agent_intent> &out) const override {
    out.clear();
    for (const agent_intent &a : before)
      if (a.agent != i)
        out.push_back(a);
  }
};

} // namespace

// Yielding acts only on what was heard: deaf, nobody moves aside; a tick late, intents are
// carried forward by their age, so the shift is close to perfect comms' but not equal; and the
// resolver hook decides who gives way.
TEST(NavYield, ActsOnWhatTheChannelDelivers) {
  const field f;
  const float a = 0.4f * f.half; // 4 units apart, closing at full speed by tick 40
  const std::vector<float> o = {-a, 0.0f, a, 0.02f}, g = {a, 0.0f, -a, 0.0f};
  const auto shifts = [&](cvc::nav::intent_channel *ch, cvc::nav::yield_resolver r, int ticks) {
    sim_world w = f.make(o, g);
    w.set_yield(yielding());
    w.set_intent_channel(ch);
    if (r)
      w.set_yield_resolver(r);
    for (int t = 0; t < ticks; ++t)
      w.step(1);
    return std::vector<float>(w.yield_shifts(), w.yield_shifts() + 4);
  };
  deaf_channel deaf;
  late_channel late;
  const std::vector<float> perfect = shifts(nullptr, nullptr, 40);
  const std::vector<float> none = shifts(&deaf, nullptr, 40);
  const std::vector<float> lagged = shifts(&late, nullptr, 40);
  EXPECT_GT(std::hypot(perfect[0], perfect[1]), 0.0f);
  EXPECT_EQ(none, std::vector<float>(4, 0.0f));
  EXPECT_GT(std::hypot(lagged[0], lagged[1]), 0.0f);
  EXPECT_NE(lagged, perfect);
  // A resolver that makes agent 1 give way entirely: agent 0 shifts less than with the default.
  const auto one_yields = [](const agent_intent &self, const agent_intent &) {
    return self.agent == 1 ? 1.0f : 0.0f;
  };
  const std::vector<float> biased = shifts(nullptr, one_yields, 40);
  EXPECT_LT(std::hypot(biased[0], biased[1]), std::hypot(perfect[0], perfect[1]));
  EXPECT_GT(std::hypot(biased[2], biased[3]), std::hypot(perfect[2], perfect[3]));
}
