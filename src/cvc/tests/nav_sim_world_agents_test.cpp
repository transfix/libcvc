/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// nav_sim_world_agents_test — sim_world's agents at run time (L0, third part): added and removed
// with a stable id each, a policy per agent (nav@1, loaded once per hash by policy_library), and
// identity (name, team, colours, metadata) in the state tree.
//
// The worlds here freeze sensing and turn the crowd terms off (separation, de-overlap,
// yielding), so each agent drives on its own: a world that adds or removes an agent must then
// drive every other agent exactly as a world built without it.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/coef_train.h> // city_scene
#include <cvc/nav/policy_library.h>
#include <cvc/nav/sim_world.h>
#include <cvc/state/state.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using cvc::nav::coef_mlp;
using cvc::nav::policy_library;
using cvc::nav::policy_ref;
using cvc::nav::sim_world;

namespace {

struct scene {
  cvc::nav::training_scene ts;
  sim_world::config cfg;
  std::vector<float> o, goal, color; // a pool of starts, goals and colours to build worlds from
};

scene make_scene(int pool) {
  scene s{cvc::nav::city_scene(96), {}, {}, {}, {}};
  const auto &ts = s.ts;
  s.cfg.rows = ts.rows;
  s.cfg.cols = ts.cols;
  s.cfg.min_x = ts.min_x;
  s.cfg.min_y = ts.min_y;
  s.cfg.max_x = ts.max_x;
  s.cfg.max_y = ts.max_y;
  s.cfg.cx = ts.cx;
  s.cfg.cy = ts.cy;
  s.cfg.scale = ts.scale;
  s.cfg.veh.rr = ts.rr;
  s.cfg.veh.d_hat = ts.d_hat;
  s.cfg.veh.dt = ts.dt;
  s.cfg.veh.vmax = ts.vmax;
  s.cfg.freeze_sense = true; // each agent drives alone
  s.o.resize(2 * pool);
  s.goal.resize(2 * pool);
  s.color.resize(3 * pool);
  sim_world::scatter_free(s.cfg, ts.occ.data(), pool, 11, s.o.data(), s.goal.data(),
                          s.color.data());
  return s;
}

// A world of the pool's agents `which`, in that order.
sim_world world_of(const scene &s, const std::vector<int> &which,
                   coef_mlp model = coef_mlp::default_biased()) {
  std::vector<float> o, goal, color;
  for (const int j : which) {
    o.insert(o.end(), {s.o[2 * j], s.o[2 * j + 1]});
    goal.insert(goal.end(), {s.goal[2 * j], s.goal[2 * j + 1]});
    color.insert(color.end(), {s.color[3 * j], s.color[3 * j + 1], s.color[3 * j + 2]});
  }
  return sim_world(s.cfg, s.ts.occ.data(), s.ts.occ.data(), std::move(model), o.data(), goal.data(),
                   color.data(), static_cast<int>(which.size()));
}

sim_world::agent_spec spec_of(const scene &s, int j) {
  sim_world::agent_spec a;
  a.x = s.o[2 * j];
  a.y = s.o[2 * j + 1];
  a.goal_x = s.goal[2 * j];
  a.goal_y = s.goal[2 * j + 1];
  for (int k = 0; k < 3; ++k)
    a.color[k] = s.color[3 * j + k];
  return a;
}

struct poses {
  std::vector<float> pos, heading, speed;
};

poses poses_of(const sim_world &w) {
  poses p;
  p.pos.resize(2 * w.size());
  p.heading.resize(w.size());
  p.speed.resize(w.size());
  w.snapshot(p.pos.data(), p.heading.data(), p.speed.data(), nullptr, nullptr);
  return p;
}

void step(sim_world &w, int ticks) {
  for (int t = 0; t < ticks; ++t)
    w.step(1);
}

// Agent ia of a drove exactly as agent ib of b.
void expect_same_agent(const poses &a, int ia, const poses &b, int ib) {
  EXPECT_EQ(a.pos[2 * ia], b.pos[2 * ib]) << "agent " << ia << " vs " << ib;
  EXPECT_EQ(a.pos[2 * ia + 1], b.pos[2 * ib + 1]) << "agent " << ia << " vs " << ib;
  EXPECT_EQ(a.heading[ia], b.heading[ib]) << "agent " << ia << " vs " << ib;
  EXPECT_EQ(a.speed[ia], b.speed[ib]) << "agent " << ia << " vs " << ib;
}

// A policy other than the default: constant coefficients (2, 1, 6) for (1, 3, 4).
coef_mlp other_policy() {
  return coef_mlp::from_layers(5, 3, {3}, {5}, {0}, {std::vector<float>(15, 0.0f)},
                               {std::vector<float>(3, 0.0f)}, {2.0f, 1.0f, 6.0f});
}

} // namespace

TEST(NavSimWorldAgents, IdsAreStableAndNeverReused) {
  const scene s = make_scene(6);
  sim_world w = world_of(s, {0, 1, 2, 3});
  ASSERT_EQ(w.size(), 4);
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(w.id_of(i), static_cast<std::uint64_t>(i + 1));
    EXPECT_EQ(w.index_of(i + 1), i);
  }
  EXPECT_EQ(w.add_agent(spec_of(s, 4)), 5u);
  EXPECT_EQ(w.size(), 5);
  EXPECT_TRUE(w.remove_agent(2));
  EXPECT_FALSE(w.remove_agent(2));
  EXPECT_FALSE(w.remove_agent(99));
  ASSERT_EQ(w.size(), 4);
  const std::vector<std::uint64_t> ids(w.ids(), w.ids() + w.size());
  EXPECT_EQ(ids, (std::vector<std::uint64_t>{1, 3, 4, 5}));
  EXPECT_EQ(w.index_of(2), -1);
  EXPECT_EQ(w.index_of(3), 1); // past the removed one, an index drops by one
  EXPECT_EQ(w.id_of(4), 0u);
  EXPECT_EQ(w.add_agent(spec_of(s, 5)), 6u); // never 2 again
}

TEST(NavSimWorldAgents, RemovingAnAgentLeavesTheOthersAsAWorldWithoutIt) {
  const scene s = make_scene(5);
  sim_world a = world_of(s, {0, 1, 2, 3, 4});
  sim_world b = world_of(s, {0, 2, 4});
  step(a, 20);
  step(b, 20);
  // Mid-run, in a: agents 1 and 3 go. b never had them.
  ASSERT_TRUE(a.remove_agent(a.id_of(1)));
  ASSERT_TRUE(a.remove_agent(a.id_of(2))); // pool agent 3, now at index 2
  ASSERT_EQ(a.size(), 3);
  step(a, 60);
  step(b, 60);
  const poses pa = poses_of(a), pb = poses_of(b);
  for (int i = 0; i < 3; ++i)
    expect_same_agent(pa, i, pb, i);
}

TEST(NavSimWorldAgents, AnAddedAgentDrivesAsOneBuiltIn) {
  const scene s = make_scene(4);
  sim_world a = world_of(s, {0, 1});
  a.add_agent(spec_of(s, 2));
  a.add_agent(spec_of(s, 3));
  sim_world b = world_of(s, {0, 1, 2, 3});
  step(a, 80);
  step(b, 80);
  const poses pa = poses_of(a), pb = poses_of(b);
  for (int i = 0; i < 4; ++i)
    expect_same_agent(pa, i, pb, i);
  // An agent added mid-run drives as a fresh world's only agent from the same pose.
  const auto id = a.add_agent(spec_of(s, 1));
  sim_world c = world_of(s, {1});
  step(a, 40);
  step(c, 40);
  expect_same_agent(poses_of(a), a.index_of(id), poses_of(c), 0);
}

TEST(NavSimWorldAgents, TheWorldCanEmptyAndFillAgain) {
  const scene s = make_scene(3);
  sim_world w = world_of(s, {0, 1});
  step(w, 5);
  ASSERT_TRUE(w.remove_agent(1));
  ASSERT_TRUE(w.remove_agent(2));
  EXPECT_EQ(w.size(), 0);
  step(w, 3);
  const auto id = w.add_agent(spec_of(s, 2));
  EXPECT_EQ(id, 3u);
  step(w, 30);
  sim_world c = world_of(s, {2});
  step(c, 30);
  expect_same_agent(poses_of(w), 0, poses_of(c), 0);
}

TEST(NavSimWorldAgents, PerAgentColumnsFollowTheirAgents) {
  const scene s = make_scene(4);
  sim_world w = world_of(s, {0, 1, 2});
  const float vmax[3] = {1.0f, 2.0f, 3.0f};
  w.set_vehicle_kinematics(vmax, nullptr, nullptr, 3);
  w.set_team(2, 7);
  w.remove_agent(w.id_of(0));
  w.add_agent(spec_of(s, 3));
  std::vector<float> got(w.size());
  w.vehicle_kinematics(got.data(), nullptr, nullptr);
  EXPECT_EQ(got, (std::vector<float>{2.0f, 3.0f, s.cfg.veh.vmax})); // the new one: the scalar
  EXPECT_EQ(w.team(1), 7);
  EXPECT_EQ(w.team(2), 0);
  // The column's pointer follows the column: stepping reads it, not a freed one.
  step(w, 10);
  for (float v : poses_of(w).speed)
    EXPECT_TRUE(std::isfinite(v));
}

TEST(NavSimWorldAgents, AddAndRemoveRefuseWhatTheyCannotDo) {
  const scene s = make_scene(2);
  sim_world w = world_of(s, {0});
  auto bad = spec_of(s, 1);
  bad.plane = 1; // one belief plane only
  EXPECT_THROW(w.add_agent(bad), std::invalid_argument);
  bad = spec_of(s, 1);
  bad.goal_x = std::nanf("");
  EXPECT_THROW(w.add_agent(bad), std::invalid_argument);
  w.begin_nav_stats();
  EXPECT_THROW(w.add_agent(spec_of(s, 1)), std::logic_error);
  EXPECT_THROW(w.remove_agent(1), std::logic_error);
  EXPECT_EQ(w.size(), 1);
}

TEST(NavSimWorldAgents, EachPolicyDrivesItsOwnAgents) {
  const scene s = make_scene(4);
  auto other = std::make_shared<const coef_mlp>(other_policy());
  // Agents 1 and 3 on the other policy, 0 and 2 on the world's.
  sim_world mixed = world_of(s, {0, 1, 2, 3});
  mixed.set_policy(1, other, "other");
  mixed.set_policy(3, other, "other");
  EXPECT_EQ(mixed.policy_count(), 2);
  EXPECT_EQ(mixed.policy_of(0), 0);
  EXPECT_EQ(mixed.policy_of(3), 1);
  EXPECT_EQ(mixed.policy_key(1), "other");
  sim_world all_default = world_of(s, {0, 1, 2, 3});
  sim_world all_other = world_of(s, {0, 1, 2, 3}, other_policy());
  step(mixed, 80);
  step(all_default, 80);
  step(all_other, 80);
  const poses pm = poses_of(mixed), pd = poses_of(all_default), po = poses_of(all_other);
  expect_same_agent(pm, 0, pd, 0);
  expect_same_agent(pm, 2, pd, 2);
  expect_same_agent(pm, 1, po, 1);
  expect_same_agent(pm, 3, po, 3);
  // The two policies really drive differently.
  EXPECT_NE(pd.pos[2], po.pos[2]);
}

TEST(NavSimWorldAgents, PoliciesAreSharedByKeyAndDroppedWhenUnused) {
  const scene s = make_scene(3);
  sim_world w = world_of(s, {0, 1, 2});
  auto p1 = std::make_shared<const coef_mlp>(other_policy());
  auto p2 = std::make_shared<const coef_mlp>(other_policy());
  w.set_policy(0, p1, "k");
  w.set_policy(1, p2, "k"); // the same key: the first model given under it
  EXPECT_EQ(w.policy_count(), 2);
  EXPECT_EQ(w.policy_of(1), 1);
  EXPECT_EQ(&w.policy(1), p1.get());
  w.set_policy(2, p2); // no key: by object, a policy of its own
  EXPECT_EQ(w.policy_count(), 3);
  w.reset_policy(0);
  w.reset_policy(1); // "k" is unused now, and the keyless one moves down
  EXPECT_EQ(w.policy_count(), 2);
  EXPECT_EQ(w.policy_of(2), 1);
  EXPECT_EQ(&w.policy(1), p2.get());
  w.remove_agent(w.id_of(2));
  EXPECT_EQ(w.policy_count(), 1);
  // An added agent can come with its policy.
  auto a = spec_of(s, 2);
  a.policy = p1;
  a.policy_key = "k";
  const auto id = w.add_agent(a);
  EXPECT_EQ(w.policy_of(w.index_of(id)), 1);
  EXPECT_EQ(w.policy_key(1), "k");
  EXPECT_THROW(w.set_policy(0, nullptr), std::invalid_argument);
}

TEST(NavSimWorldAgents, IdentityIsKeptAndPublished) {
  const scene s = make_scene(3);
  sim_world w = world_of(s, {0, 1});
  const long r0 = w.identity_revision();
  w.set_name(0, "scout");
  const float red[3] = {1.0f, 0.0f, 0.0f}, blue[3] = {0.0f, 0.0f, 1.0f};
  w.set_colors(0, red, blue);
  w.set_metadata(0, "driver", "ada");
  auto a = spec_of(s, 2);
  a.name = "hauler";
  a.team = 3;
  const auto id3 = w.add_agent(a);
  EXPECT_GT(w.identity_revision(), r0);
  EXPECT_EQ(w.name(0), "scout");
  EXPECT_EQ(w.name(2), "hauler");
  EXPECT_EQ(w.team(2), 3);
  float p[3], q[3];
  w.colors(0, p, q);
  EXPECT_EQ(p[0], 1.0f);
  EXPECT_EQ(q[2], 1.0f);
  EXPECT_EQ(w.metadata(0).at("driver"), "ada");
  EXPECT_THROW(w.name(3), std::out_of_range);

  cvc::app app;
  cvc::state &st = cvc::state::instance(app);
  w.publish(app, "scene");
  EXPECT_EQ(st("scene.nav.agents.count").value(), "3");
  EXPECT_EQ(st("scene.nav.agents.ids").value(), "1,2,3");
  EXPECT_EQ(st("scene.nav.agents.1.name").value(), "scout");
  EXPECT_EQ(st("scene.nav.agents.1.color").value(), "#ff0000");
  EXPECT_EQ(st("scene.nav.agents.1.color2").value(), "#0000ff");
  EXPECT_EQ(st("scene.nav.agents.1.meta.driver").value(), "ada");
  EXPECT_EQ(st("scene.nav.agents.3.team").value(), "3");
  EXPECT_EQ(st("scene.nav.agents.3.index").value(), "2");
  EXPECT_EQ(st("scene.nav.agents.3.policy").value(), "");

  // A removal and a metadata key taken away reach the tree on the next publish.
  w.remove_agent(2);
  w.set_metadata(0, "driver", "");
  w.publish(app, "scene");
  EXPECT_EQ(st("scene.nav.agents.ids").value(), "1,3");
  EXPECT_EQ(st("scene.nav.agents.3.index").value(), "1");
  const auto kids = st("scene.nav.agents").childNames();
  EXPECT_EQ(std::count(kids.begin(), kids.end(), "2"), 0);
  EXPECT_EQ(std::count(kids.begin(), kids.end(), std::to_string(id3)), 1);
  const auto meta = st("scene.nav.agents.1").childNames();
  EXPECT_EQ(std::count(meta.begin(), meta.end(), "meta"), 0);
}

TEST(NavPolicyLibrary, LoadsOncePerHash) {
  namespace fs = std::filesystem;
  const fs::path dir = fs::temp_directory_path() / "cvc_nav_policy_library_test";
  fs::create_directories(dir);
  const fs::path a = dir / "a.cvcnav", b = dir / "b.cvcnav";
  other_policy().save(a.string());
  fs::copy_file(a, b, fs::copy_options::overwrite_existing);

  policy_library lib;
  const auto ea = lib.get({a.string(), ""});
  ASSERT_TRUE(ea.model);
  ASSERT_EQ(ea.key.substr(0, 7), "sha256:");
  EXPECT_EQ(ea.key.size(), 7u + 64u);
  // The same bytes from another file, or as a file:// URI: the same copy.
  EXPECT_EQ(lib.get({b.string(), ""}).model, ea.model);
  EXPECT_EQ(lib.get({"file://" + fs::absolute(a).generic_string(), ""}).model, ea.model);
  EXPECT_EQ(lib.size(), 1u);
  // With its hash it is found without the file.
  fs::remove(a);
  EXPECT_EQ(lib.get({a.string(), ea.key.substr(7)}).model, ea.model);
  // A hash the bytes don't have is refused.
  EXPECT_THROW(lib.get({b.string(), std::string(64, '0')}), std::runtime_error);
  EXPECT_THROW(lib.get({b.string(), "abc"}), std::invalid_argument);
  EXPECT_THROW(lib.get({"https://example.com/p.cvcnav", ""}), std::runtime_error);
  EXPECT_THROW(lib.get({(dir / "missing.cvcnav").string(), ""}), std::runtime_error);
  // The default policy.
  const auto ed = lib.get({});
  EXPECT_EQ(ed.key, "default");
  EXPECT_EQ(lib.get({}).model, ed.model);
  EXPECT_EQ(lib.size(), 2u);
  // The loaded policy drives as the one it was saved from.
  EXPECT_EQ(ea.model->in_features(), 5);
  fs::remove_all(dir);
}
