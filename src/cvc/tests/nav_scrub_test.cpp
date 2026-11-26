/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The checkpoint scrubber (cvc/nav/nav_scrub.h): a coef_trainer run's history and checkpoints are
// read and deployed on one scenario; a blob deployed twice gives the same run; checkpoint 0
// deployed is the untrained policy's arm; the runs are cached by blob and scenario; and the view
// shows checkpoint k beside step 0 as its controls pick it; a scenario names its plant, which a
// hook attaches, and the view shows the same checkpoint on each plant it was deployed on.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/coef_train.h>
#include <cvc/nav/nav_arms.h>
#include <cvc/nav/nav_panels.h>
#include <cvc/nav/nav_scrub.h>
#include <cvc/nav/policy_library.h>
#include <cvc/nav/sim_world.h>
#include <cvc/nav/sim_world_trace.h>
#include <cvc/nav/world_grid.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

using namespace cvc::nav;
namespace fs = std::filesystem;

namespace {

train_config small_cfg(const std::string &dir) {
  train_config c;
  c.n = 16;
  c.window = 4;
  c.hidden = 16;
  c.horizon = 8;
  c.steps = 10;
  c.lr = 3e-3f;
  c.seed = 0;
  c.checkpoint_every = 5;
  c.history_path = dir + "/history.jsonl";
  c.checkpoint_prefix = dir + "/coef";
  return c;
}

// A coef_trainer run in its own directory: its history and three checkpoints (steps 0, 5, 10).
struct trained {
  std::string dir, history;
  train_config cfg;
};

trained train_once(const std::string &name) {
  trained t;
  t.dir = (fs::path(::testing::TempDir()) / ("nav_scrub_" + name)).string();
  fs::remove_all(t.dir);
  fs::create_directories(t.dir);
  t.cfg = small_cfg(t.dir);
  t.history = t.cfg.history_path;
  coef_trainer tr(t.cfg, 7);
  tr.train(city_scene(48), false, [](const coef_mlp &, int step) { return step == 5 ? 1.0 : 0.5; });
  return t;
}

// A run of this test's own: ctest runs each test in a process of its own, in parallel, so a
// directory two tests shared would be one test's to delete under the other.
trained train_here() {
  return train_once(::testing::UnitTest::GetInstance()->current_test_info()->name());
}

bool same(double a, double b) { return (std::isnan(a) && std::isnan(b)) || a == b; }

// Every value but the wall times.
void expect_same_runs(const run_log &a, int ja, const run_log &b, int jb) {
  ASSERT_EQ(a.ticks(), b.ticks());
  ASSERT_EQ(a.agents(), b.agents());
  ASSERT_EQ(a.header().agent_columns, b.header().agent_columns);
  for (long k = 0; k < a.ticks(); ++k) {
    for (int i = 0; i < a.agents(); ++i)
      for (int c = 0; c < static_cast<int>(a.header().agent_columns.size()); ++c)
        ASSERT_TRUE(same(a.at(k, ja, i, c), b.at(k, jb, i, c)))
            << "tick " << k << " agent " << i << " " << a.header().agent_columns[c];
    for (const char *col : {"stats.clear_min_m", "stats.clear_mean_m", "stats.pen_steps",
                            "stats.path_m", "stats.elapsed_s", "stats.reached"})
      ASSERT_TRUE(same(a.arm_at(k, ja, col), b.arm_at(k, jb, col))) << "tick " << k << " " << col;
  }
}

} // namespace

TEST(NavScrub, AHistoryNamesItsCheckpointsAndCurves) {
  const trained t = train_here();
  const training_history h = read_training_history(t.history);
  EXPECT_EQ(h.schema, "cvc.nav.coef_train_history/v1");
  EXPECT_EQ(h.runs, 1);
  ASSERT_EQ(h.checkpoints.size(), 3u);
  EXPECT_EQ(h.checkpoints[0].step, 0);
  EXPECT_EQ(h.checkpoints[1].step, 5);
  EXPECT_EQ(h.checkpoints[2].step, 10);
  EXPECT_TRUE(h.checkpoints[1].selected); // the fitness's best: the end record's selected_step
  EXPECT_FALSE(h.checkpoints[2].selected);
  EXPECT_DOUBLE_EQ(h.checkpoints[1].fitness, 1.0);
  EXPECT_EQ(fs::path(h.checkpoints[0].path).filename(), "coef.step000000.cvcnav");
  const training_series *loss = h.find("step.loss");
  ASSERT_NE(loss, nullptr);
  EXPECT_EQ(loss->x.size(), 10u);
  EXPECT_NE(h.find("step.grad_norm.mean"), nullptr);
  EXPECT_NE(h.find("checkpoint.fitness"), nullptr);
  EXPECT_NE(h.find("checkpoint.heads.alpha.mean"), nullptr);
  EXPECT_EQ(h.find("nothing"), nullptr);
  EXPECT_THROW(read_training_history(t.history, 3), std::runtime_error);
  EXPECT_THROW(read_training_history(t.dir + "/none.jsonl"), std::runtime_error);
}

TEST(NavScrub, AHistoryOfAnotherTrainerAndSeveralRuns) {
  const trained t = train_here();
  const std::string dir = (fs::path(::testing::TempDir()) / "nav_scrub_other").string();
  fs::create_directories(dir);
  fs::copy_file(t.dir + "/coef.step000005.cvcnav", dir + "/a.cvcnav",
                fs::copy_options::overwrite_existing);
  const std::string path = dir + "/history.jsonl";
  {
    std::ofstream f(path);
    f << "{\"kind\":\"header\",\"schema\":\"another/v1\"}\n"
      << "{\"kind\":\"checkpoint\",\"epoch\":1,\"blob\":\"a.cvcnav\"}\n"
      << "{\"kind\":\"header\",\"schema\":\"another/v1\"}\n"
      << "{\"kind\":\"train\",\"step\":0,\"loss\":2.0}\n"
      << "{\"kind\":\"train\",\"step\":4,\"loss\":1.0}\n"
      << "{\"kind\":\"checkpoint\",\"step\":4,\"checkpoint\":\"a.cvcnav\",\"selected\":true,"
         "\"best\":true}\n"
      << "{\"kind\":\"checkpoint\",\"step\":8,\"blob\":\"gone.cvcnav\"}\n"
      << "{\"kind\":\"checkpoint\",\"step\":9,\"blob\":\"a.cvcnav.pt\"}\n";
  }
  std::ofstream(dir + "/a.cvcnav.pt") << "torch";
  const training_history last = read_training_history(path);
  EXPECT_EQ(last.runs, 2);
  EXPECT_EQ(last.run, 1);
  ASSERT_EQ(last.checkpoints.size(), 1u);
  EXPECT_EQ(last.checkpoints[0].step, 4);
  EXPECT_TRUE(last.checkpoints[0].selected && last.checkpoints[0].best);
  EXPECT_EQ(last.missing, std::vector<std::string>{"gone.cvcnav"});
  EXPECT_EQ(last.skipped, std::vector<std::string>{"a.cvcnav.pt"});
  EXPECT_EQ(last.find("train.loss")->y, (std::vector<double>{2.0, 1.0}));
  const training_history first = read_training_history(path, 0);
  ASSERT_EQ(first.checkpoints.size(), 1u);
  EXPECT_EQ(first.checkpoints[0].axis, "epoch");
  std::ofstream(dir + "/gz.jsonl", std::ios::binary) << "\x1f\x8b\x08";
  EXPECT_THROW(read_training_history(dir + "/gz.jsonl"), std::runtime_error);
  EXPECT_EQ(resolve_blob(path, ""), "");
  EXPECT_EQ(resolve_blob(path, "missing.cvcnav"), "");
}

TEST(NavScrub, TheScenarioAndItsHash) {
  const scrub_scenario s = default_scenario(3, 1, 200);
  EXPECT_EQ(s.ticks, 200);
  EXPECT_EQ(s.world.agents.size(), 3u);
  EXPECT_EQ(s.name, "blocks, 3 agents, seed 1");
  EXPECT_EQ(scenario_hash(s), scenario_hash(default_scenario(3, 1, 200)));
  EXPECT_EQ(scenario_hash(s).size(), 16u);
  EXPECT_NE(scenario_hash(s), scenario_hash(default_scenario(3, 2, 200)));
  EXPECT_NE(scenario_hash(s), scenario_hash(default_scenario(3, 1, 201)));
  EXPECT_NE(scenario_hash(s), scenario_hash(default_scenario(2, 1, 200)));
  EXPECT_EQ(scrub_cache_path("run/h.jsonl", std::string(64, 'a'), "0123456789abcdef"),
            (fs::path("run/h.jsonl.scrub") / (std::string(16, 'a') + "-0123456789abcdef.cvcrun"))
                .string());
  EXPECT_EQ(default_scenario(1, 0, 10).name, "blocks, 1 agent, seed 0");
  EXPECT_THROW(default_scenario(0, 0, 10), std::invalid_argument);
  // a recorded trace's world
  const std::string trace = (fs::path(::testing::TempDir()) / "nav_scrub_scenario.trace").string();
  {
    cvc::input_trace tr;
    tr.header = s.world;
    tr.save(trace);
  }
  const scrub_scenario r = scenario_from_trace(trace, 200);
  EXPECT_EQ(scenario_hash(r), scenario_hash(s));
  EXPECT_EQ(r.name, "nav-lab:blocks, 3 agents");
}

// The least distance (m, at `scale` units a metre) between any two of the agents' starts, and
// between any two of their goals.
static double least_apart(const scrub_scenario &s, bool goals, double scale = 0.05) {
  double d = 1e30;
  const auto &a = s.world.agents;
  for (std::size_t i = 0; i < a.size(); ++i)
    for (std::size_t j = i + 1; j < a.size(); ++j) {
      const auto &p = goals ? a[i].goal : a[i].start, &q = goals ? a[j].goal : a[j].start;
      d = std::min(d, std::hypot(p[0] - q[0], p[1] - q[1]) / scale);
    }
  return d;
}

TEST(NavScrub, NoTwoCarsStartOrEndOnTopOfEachOther) {
  // the built-in scenario's lanes are 8 m apart (a Humvee is 2.2 m wide)
  for (unsigned seed = 0; seed < 8; ++seed) {
    const scrub_scenario s = default_scenario(6, seed, 10);
    EXPECT_GE(least_apart(s, false), 8.0 - 1e-3) << "seed " << seed;
    EXPECT_GE(least_apart(s, true), 8.0 - 1e-3) << "seed " << seed;
  }
}

TEST(NavScrub, AScenarioOnAWorldGridAtItsRealSize) {
  // a 160 m flat square with a 40 m block in its middle, on 2 m cells
  world_grid g = flat_grid(80.0, 2.0);
  for (int r = 0; r < g.rows; ++r)
    for (int c = 0; c < g.cols; ++c) {
      const double x = g.min_x + c * g.cell(), y = g.min_y + r * g.cell();
      if (std::abs(x) <= 20.0 && std::abs(y) <= 20.0)
        g.occupancy[static_cast<std::size_t>(r) * g.cols + c] = 1;
    }
  const scrub_scenario s = grid_scenario(g, 5, 11, 300, "plaza");
  EXPECT_EQ(s.ticks, 300);
  EXPECT_EQ(s.name, "plaza, 5 agents, seed 11");
  EXPECT_EQ(s.world.world_id, "nav-lab:plaza");
  ASSERT_EQ(s.world.agents.size(), 5u);
  EXPECT_EQ(scenario_hash(s), scenario_hash(grid_scenario(g, 5, 11, 300, "plaza")));
  EXPECT_NE(scenario_hash(s), scenario_hash(grid_scenario(g, 5, 12, 300, "plaza")));
  EXPECT_GE(least_apart(s, false), 12.0 - 1e-3);
  EXPECT_GE(least_apart(s, true), 12.0 - 1e-3);
  for (const auto &a : s.world.agents) // no goal on another's start
    for (const auto &b : s.world.agents)
      EXPECT_GE(std::hypot(a.goal[0] - b.start[0], a.goal[1] - b.start[1]) / 0.05, 12.0 - 1e-3);
  for (const auto &a : s.world.agents) {
    // the world stays in metres: the normalized start is (x - cx) * 0.05
    const double x = a.start[0] / 0.05, y = a.start[1] / 0.05;
    const double gx = a.goal[0] / 0.05, gy = a.goal[1] / 0.05;
    EXPECT_LE(std::abs(x), 80.0);
    EXPECT_LE(std::abs(y), 80.0);
    EXPECT_FALSE(std::abs(x) <= 24.0 && std::abs(y) <= 24.0) << x << ", " << y; // 4 m clear
    EXPECT_FALSE(std::abs(gx) <= 24.0 && std::abs(gy) <= 24.0) << gx << ", " << gy;
    EXPECT_GE(std::hypot(gx - x, gy - y), 0.3 * 160.0 - 1e-3); // a goal across the world
    EXPECT_NEAR(a.start[2], std::atan2(gy - y, gx - x), 1e-5); // facing it
  }
  // the scenario deploys: the untrained policy runs on it
  {
    policy_library lib;
    EXPECT_EQ(world_from_trace(s.world, lib).size(), 5);
  }
  // the same world at another scale is another scenario; a scenario needs an agent and a scale
  EXPECT_NE(scenario_hash(grid_scenario(g, 5, 11, 300, "plaza", 0.1)), scenario_hash(s));
  EXPECT_THROW(grid_scenario(g, 0, 11, 300, "plaza"), std::invalid_argument);
  EXPECT_THROW(grid_scenario(g, 2, 11, 300, "plaza", 0.0), std::invalid_argument);
  world_grid shut = flat_grid(20.0, 2.0);
  std::fill(shut.occupancy.begin(), shut.occupancy.end(), std::uint8_t{1});
  EXPECT_THROW(grid_scenario(shut, 2, 0, 10, "shut"), std::runtime_error);
}

TEST(NavScrub, OneBlobDeployedTwiceIsTheSameRun) {
  const trained t = train_here();
  const scrub_scenario s = default_scenario(2, 0, 150);
  policy_library lib;
  const std::string blob = t.dir + "/coef.step000010.cvcnav";
  const run_log a = deploy_checkpoint(s, blob, "step 10", lib);
  const run_log b = deploy_checkpoint(s, blob, "step 10", lib);
  ASSERT_EQ(a.ticks(), 150);
  EXPECT_TRUE(a.finished());
  EXPECT_EQ(a.header().arms[0].uri, blob);
  EXPECT_EQ(a.header().arms[0].sha256.size(), 64u);
  expect_same_runs(a, 0, b, 0);
}

TEST(NavScrub, Checkpoint0DeployedIsTheUntrainedArm) {
  const trained t = train_here();
  const scrub_scenario s = default_scenario(2, 0, 150);
  policy_library lib;
  const run_log k0 = deploy_checkpoint(s, t.dir + "/coef.step000000.cvcnav", "step 0", lib);
  // the untrained policy, as the trainer initializes it, in an arm of its own beside the default
  auto untrained = std::make_shared<const coef_mlp>(coef_trainer(t.cfg, 7).to_coef_mlp());
  std::vector<std::unique_ptr<sim_world>> worlds;
  for (int a = 0; a < 2; ++a) {
    worlds.push_back(std::make_unique<sim_world>(world_from_trace(s.world, lib)));
    if (a == 1)
      for (int i = 0; i < worlds.back()->size(); ++i)
        worlds.back()->set_policy(i, untrained, "untrained");
  }
  arms_run run(std::move(worlds), {{"default"}, {"untrained"}});
  for (long k = 0; k < s.ticks; ++k)
    run.step(1);
  expect_same_runs(k0, 0, run.log(), 1);
}

TEST(NavScrub, EveryCheckpointIsDeployedOnceAndCached) {
  const trained t = train_once("cache");
  const training_history h = read_training_history(t.history);
  const scrub_scenario s = default_scenario(2, 0, 120);
  std::vector<std::string> said;
  scrub_options o;
  o.say = [&](const std::string &line) { said.push_back(line); };
  const std::vector<scrub_entry> first = scrub(h, s, o);
  ASSERT_EQ(first.size(), 3u);
  ASSERT_EQ(said.size(), 3u);
  EXPECT_NE(said[0].find("step 0 (1/3) on kinematic, deployed"), std::string::npos) << said[0];
  for (const scrub_entry &e : first) {
    EXPECT_FALSE(e.cached);
    EXPECT_TRUE(fs::is_regular_file(e.cache)) << e.cache;
    EXPECT_EQ(e.sha256.size(), 64u);
  }
  said.clear();
  const std::vector<scrub_entry> again = scrub(h, s, o);
  for (std::size_t i = 0; i < again.size(); ++i) {
    EXPECT_TRUE(again[i].cached);
    EXPECT_EQ(again[i].cache, first[i].cache);
    expect_same_runs(*again[i].log, 0, *first[i].log, 0);
    EXPECT_EQ(again[i].metrics.path_m, first[i].metrics.path_m);
  }
  EXPECT_NE(said[2].find("cached"), std::string::npos);
  // another scenario: deployed again, beside the first
  const std::vector<scrub_entry> other = scrub(h, default_scenario(2, 1, 120));
  EXPECT_FALSE(other[0].cached);
  EXPECT_NE(other[0].cache, first[0].cache);
  // without the cache: nothing read, nothing written
  scrub_options none;
  none.use_cache = false;
  fs::remove_all(t.history + ".scrub");
  EXPECT_FALSE(scrub(h, s, none)[1].cached);
  EXPECT_FALSE(fs::exists(t.history + ".scrub"));
  // the report
  const std::string report = scrub_report_json(h, s, first);
  EXPECT_NE(report.find("\"schema\":\"cvc.nav.scrub/v1\""), std::string::npos);
  EXPECT_NE(report.find("\"selected\":true"), std::string::npos);
  EXPECT_NE(report.find("\"path_m\":"), std::string::npos);
}

TEST(NavScrub, TheMetricsTheTableAndTheCharts) {
  const trained t = train_here();
  const training_history h = read_training_history(t.history);
  const std::vector<scrub_entry> e = scrub(h, default_scenario(2, 0, 120));
  const scrub_metrics &m = e[0].metrics;
  EXPECT_GE(m.reached, 0.0);
  EXPECT_LE(m.reached, 1.0);
  EXPECT_GT(m.path_m, 0.0);
  EXPECT_GT(m.goal_m, 0.0);
  EXPECT_EQ(scrub_metric_names().size(), 7u);
  EXPECT_TRUE(std::isnan(m.value(7)));
  const panel_table table = scrub_table(e, 2);
  ASSERT_EQ(table.rows.size(), 3u);
  EXPECT_EQ(table.cell(2, "at"), ">");
  EXPECT_EQ(table.cell(1, "flag"), "pick");
  EXPECT_EQ(table.cell(1, "step"), "5");
  std::string text;
  const chart_plot c = metric_chart(e, "goal_m", &text);
  ASSERT_EQ(c.series.size(), 2u);
  EXPECT_EQ(c.series[1].style, "c0 dashed");
  EXPECT_EQ(c.series[1].y, std::vector<double>(3, e[0].metrics.goal_m));
  ASSERT_EQ(c.markers.size(), 1u);
  EXPECT_EQ(c.markers[0].x, 5.0);
  EXPECT_NE(text.find("left to the goal (m): step 0"), std::string::npos) << text;
  EXPECT_NE(text.find("lowest"), std::string::npos) << text;
  bool log_y = true;
  const chart_plot curve = curve_chart(h, e, "step.loss", &log_y, &text);
  ASSERT_EQ(curve.series.size(), 1u);
  EXPECT_EQ(curve.series[0].x.size(), 10u);
  EXPECT_NE(text.find("step.loss: 10 points"), std::string::npos) << text;
  const chart_plot none = curve_chart(h, e, "nope", &log_y, &text);
  EXPECT_TRUE(none.series.empty());
  EXPECT_FALSE(log_y);
}

TEST(NavScrub, TheViewShowsCheckpointKBesideStep0) {
  const trained t = train_here();
  const training_history h = read_training_history(t.history);
  std::vector<scrub_entry> e = scrub(h, default_scenario(2, 0, 120));
  EXPECT_THROW(scrub_view(h, std::vector<scrub_entry>{}), std::invalid_argument);
  scrub_view view(h, e, -1, "blocks");
  EXPECT_EQ(view.checkpoint(), 1); // the pick
  cvc::app app;
  panel_set(app, "sim.paused", "1");
  scrub_view::frame f = view.update(app, "doc", 0.0);
  EXPECT_EQ(f.k, 1);
  EXPECT_EQ(panel_get(app, "doc.scrub.labels"), "step 0\nstep 5 (pick)\nstep 10");
  EXPECT_EQ(panel_get(app, "doc.scrub.choice"), "step 5 (pick)");
  EXPECT_EQ(panel_get(app, "doc.scrub.pos"), "0.500000");
  EXPECT_EQ(panel_get(app, "doc.scrub.scenario"), "blocks");
  EXPECT_EQ(panel_get(app, "doc.scrub.curve_name"), "step.loss");
  EXPECT_EQ(panel_get(app, "doc.arms.names"), "step 0\nstep 5");
  EXPECT_EQ(panel_get(app, "doc.arms.stats.1.arm"), "step 5");
  EXPECT_EQ(panel_get(app, "doc.scrub.table.1.at"), ">");
  EXPECT_EQ(panel_get(app, "doc.scrub.metric.s"), "2");
  EXPECT_NE(panel_get(app, "doc.scrub.now").find("checkpoint 2 of 3: step 5, the pick"),
            std::string::npos);
  ASSERT_EQ(f.replay.poses.size(), 2u);
  // the slider picks
  panel_set(app, "doc.scrub.pos", "1");
  f = view.update(app, "doc", 0.0);
  EXPECT_EQ(f.k, 2);
  EXPECT_EQ(panel_get(app, "doc.arms.stats.1.arm"), "step 10");
  // the combo picks
  panel_set(app, "doc.scrub.choice", "step 0");
  f = view.update(app, "doc", 0.0);
  EXPECT_EQ(f.k, 0);
  EXPECT_EQ(panel_get(app, "doc.arms.names"), "step 0\nstep 0 "); // the same checkpoint twice
  // previous, next, a click on a chart
  view.next();
  view.next();
  view.next();
  EXPECT_EQ(view.checkpoint(), 2);
  view.prev();
  EXPECT_EQ(view.checkpoint(), 1);
  panel_set(app, "doc.scrub.pick", "9.2");
  view.pick_step(app, "doc");
  EXPECT_EQ(view.checkpoint(), 2);
  panel_set(app, "doc.scrub.pick", "x");
  view.pick_step(app, "doc");
  EXPECT_EQ(view.checkpoint(), 2);
  // another metric, another curve
  panel_set(app, "doc.scrub.metric_name", "reached");
  panel_set(app, "doc.scrub.curve_name", "checkpoint.fitness");
  f = view.update(app, "doc", 0.0);
  EXPECT_NE(panel_get(app, "doc.scrub.metric_text").find("reached the goal"), std::string::npos);
  EXPECT_NE(panel_get(app, "doc.scrub.curve_text").find("checkpoint.fitness"), std::string::npos);
  // the play head carries over a pick
  view.replay().head().set(3.0);
  view.pick(0);
  f = view.update(app, "doc", 0.0);
  EXPECT_DOUBLE_EQ(f.replay.t, 3.0);
}

TEST(NavScrub, ThePlantIsTheScenarios) {
  const trained t = train_here();
  scrub_scenario s = default_scenario(2, 0, 100);
  EXPECT_EQ(s.plant, "kinematic");
  scrub_scenario blank = s;
  blank.plant = "";
  EXPECT_EQ(scenario_hash(blank), scenario_hash(s)); // "" is the bicycle
  scrub_scenario hv = s;
  hv.plant = "humvee";
  EXPECT_NE(scenario_hash(hv), scenario_hash(s));
  policy_library lib;
  const std::string blob = t.dir + "/coef.step000010.cvcnav";
  // another plant needs a hook: nav has none of its own
  EXPECT_THROW(deploy_checkpoint(hv, blob, "step 10", lib), std::invalid_argument);
  // a hook is called once a deploy, before the first tick, and its owner is kept for the run
  int calls = 0;
  std::weak_ptr<int> kept;
  const scrub_plant_fn hook = [&](sim_world &w, const std::string &plant) {
    ++calls;
    EXPECT_EQ(plant, "humvee");
    EXPECT_EQ(w.size(), 2);
    auto owner = std::make_shared<int>(7);
    kept = owner;
    return scrub_plant_attached{"humvee (a stand-in)", owner};
  };
  const run_log l = deploy_checkpoint(hv, blob, "step 10", lib, "{}", hook);
  EXPECT_EQ(calls, 1);
  EXPECT_TRUE(kept.expired());
  EXPECT_EQ(l.header().plant, "humvee (a stand-in)"); // what the agents drove
  EXPECT_NE(l.header().world.find("\"plant\":\"humvee\""), std::string::npos);
  EXPECT_EQ(deploy_checkpoint(s, blob, "step 10", lib, "{}", hook).header().plant, "kinematic");
  EXPECT_EQ(calls, 1); // the bicycle never calls it
  // scrub() deploys with the options' hook, and the report names the plant
  const training_history h = read_training_history(t.history);
  scrub_options o;
  o.plant = hook;
  o.use_cache = false;
  const std::vector<scrub_entry> e = scrub(h, hv, o);
  EXPECT_EQ(calls, 4);
  EXPECT_NE(scrub_report_json(h, hv, e).find("\"plant\":\"humvee\""), std::string::npos);
}

TEST(NavScrub, TheViewShowsTheSameCheckpointOnAnotherPlant) {
  const trained t = train_here();
  const training_history h = read_training_history(t.history);
  scrub_scenario s = default_scenario(2, 0, 120);
  const std::vector<scrub_entry> kin = scrub(h, s);
  scrub_options o;
  o.plant = [](sim_world &, const std::string &p) { return scrub_plant_attached{p, nullptr}; };
  s.plant = "other";
  const std::vector<scrub_entry> other = scrub(h, s, o);
  EXPECT_NE(other[1].cache, kin[1].cache);
  EXPECT_THROW(scrub_view(h, std::vector<scrub_plant_runs>{}), std::invalid_argument);
  EXPECT_THROW(
      scrub_view(h, std::vector<scrub_plant_runs>{{"kinematic", kin}, {"kinematic", other}}),
      std::invalid_argument); // named twice
  EXPECT_THROW(
      scrub_view(h, std::vector<scrub_plant_runs>{{"kinematic", kin}, {"other", {kin[0], kin[1]}}}),
      std::invalid_argument); // not the same checkpoints
  scrub_view view(h, std::vector<scrub_plant_runs>{{"kinematic", kin}, {"other", other}}, -1,
                  "blocks");
  cvc::app app;
  panel_set(app, "sim.paused", "1");
  scrub_view::frame f = view.update(app, "doc", 0.0);
  EXPECT_EQ(panel_get(app, "doc.scrub.plant_labels"), "kinematic\nother");
  EXPECT_EQ(panel_get(app, "doc.scrub.plant_count"), "2");
  EXPECT_EQ(panel_get(app, "doc.scrub.plant_choice"), "kinematic");
  EXPECT_EQ(panel_get(app, "doc.scrub.plant"), "kinematic");
  EXPECT_EQ(view.plant(), 0);
  EXPECT_EQ(f.k, 1);
  // the combo: the same checkpoint and play head on the other plant
  view.replay().head().set(2.0);
  panel_set(app, "doc.scrub.plant_choice", "other");
  f = view.update(app, "doc", 0.0);
  EXPECT_EQ(view.plant(), 1);
  EXPECT_EQ(f.k, 1);
  EXPECT_DOUBLE_EQ(f.replay.t, 2.0);
  EXPECT_EQ(view.entries()[1].cache, other[1].cache);
  EXPECT_EQ(panel_get(app, "doc.scrub.plant"), "other");
  EXPECT_NE(panel_get(app, "doc.play.title").find("on other"), std::string::npos)
      << panel_get(app, "doc.play.title");
  // in code, and a name it does not have is ignored
  view.use_plant(0);
  view.update(app, "doc", 0.0);
  EXPECT_EQ(panel_get(app, "doc.scrub.plant_choice"), "kinematic");
  panel_set(app, "doc.scrub.plant_choice", "nope");
  view.update(app, "doc", 0.0);
  EXPECT_EQ(view.plant(), 0);
  EXPECT_EQ(panel_get(app, "doc.scrub.plant_choice"), "kinematic");
  view.use_plant(9);
  EXPECT_EQ(view.plant(), 1);
  // one plant, unnamed: one, named from its runs
  scrub_view one(h, other);
  cvc::app app1;
  one.update(app1, "doc", 0.0);
  EXPECT_EQ(panel_get(app1, "doc.scrub.plant_count"), "1");
  EXPECT_EQ(panel_get(app1, "doc.scrub.plant"), "other");
}
