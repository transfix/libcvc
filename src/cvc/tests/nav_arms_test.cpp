/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Arms (cvc/nav/nav_arms.h), the run log (cvc/nav/run_log.h) and the panels drawn from it
// (cvc/nav/nav_panels.h), on a small kinematic world: the arms step in lockstep from the same
// start; one retarget moves every arm's goal on the same tick and starts a leg; the books close to
// float round-off; an arm runs bit for bit as its policy alone; the log reads back value for value,
// so the panels at a tick are the same cell for cell; and the play head, the replay and the
// state-tree layout do what the documents bind.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/coef_train.h>
#include <cvc/nav/nav_arms.h>
#include <cvc/nav/nav_panels.h>
#include <cvc/nav/plant_ledger.h>
#include <cvc/nav/run_log.h>
#include <cvc/nav/sim_world.h>
#include <cvc/state/state.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace cvc::nav;

namespace {

constexpr double kS = 0.05; // 1 normalized unit = 20 m
constexpr int R = 64, C = 64;
constexpr double kLo = -32.0, kHi = 32.0; // 1 m cells

std::vector<std::uint8_t> city() {
  std::vector<std::uint8_t> occ(static_cast<std::size_t>(R) * C, 0);
  auto block = [&](int r0, int r1, int c0, int c1) {
    for (int r = r0; r < r1; ++r)
      for (int c = c0; c < c1; ++c)
        occ[static_cast<std::size_t>(r) * C + c] = 1;
  };
  block(24, 40, 28, 36);
  block(8, 14, 40, 46);
  block(48, 56, 16, 22);
  return occ;
}

sim_world::config world_config() {
  sim_world::config c;
  c.rows = R;
  c.cols = C;
  c.min_x = c.min_y = kLo;
  c.max_x = c.max_y = kHi;
  c.scale = kS;
  c.veh.rr = 0.05f;
  c.veh.d_hat = 0.2f;
  c.veh.dt = 0.05f;
  c.veh.vmax = static_cast<float>(8.0 * kS);
  c.veh.L = static_cast<float>(3.3 * kS);
  c.range_m = 25.0;
  c.n_rays = 90;
  c.sense_every = 4;
  return c;
}

// n agents from the west side to the east, every world alike.
std::unique_ptr<sim_world> make_world(std::shared_ptr<const coef_mlp> policy = nullptr, int n = 1) {
  const std::vector<std::uint8_t> occ = city();
  const float o[6] = {static_cast<float>(-26 * kS), static_cast<float>(-4 * kS),
                      static_cast<float>(-26 * kS), static_cast<float>(10 * kS),
                      static_cast<float>(-20 * kS), static_cast<float>(20 * kS)};
  const float goal[6] = {static_cast<float>(26 * kS), static_cast<float>(2 * kS),
                         static_cast<float>(26 * kS), static_cast<float>(-6 * kS),
                         static_cast<float>(20 * kS), static_cast<float>(-22 * kS)};
  const float color[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  auto w = std::make_unique<sim_world>(world_config(), occ.data(), occ.data(),
                                       coef_mlp::default_biased(), o, goal, color, n);
  if (policy)
    for (int i = 0; i < n; ++i)
      w->set_policy(i, policy, "test");
  return w;
}

std::shared_ptr<const coef_mlp> random_policy(unsigned seed) {
  return std::make_shared<const coef_mlp>(coef_trainer(train_config{}, seed).to_coef_mlp());
}

// Three arms, each its own world: the default policy and two untrained nets.
std::unique_ptr<arms_run> three_arms(int n = 1, const std::string &log_path = "") {
  std::vector<std::unique_ptr<sim_world>> worlds;
  worlds.push_back(make_world(nullptr, n));
  worlds.push_back(make_world(random_policy(1), n));
  worlds.push_back(make_world(random_policy(2), n));
  arms_options o;
  o.seed = 7;
  o.scene = "city";
  o.world = R"({"kind":"test-city"})";
  o.log_path = log_path;
  return std::make_unique<arms_run>(std::move(worlds),
                                    std::vector<arm>{{"default"}, {"net1"}, {"net2"}}, o);
}

std::string tmp_path(const std::string &name) {
  return (std::filesystem::temp_directory_path() / ("nav_arms_test_" + name)).string();
}

bool same(double a, double b) { return (std::isnan(a) && std::isnan(b)) || a == b; }

void expect_same_logs(const run_log &a, const run_log &b) {
  ASSERT_EQ(a.ticks(), b.ticks());
  ASSERT_EQ(a.arms(), b.arms());
  ASSERT_EQ(a.agents(), b.agents());
  ASSERT_EQ(a.header().agent_columns, b.header().agent_columns);
  ASSERT_EQ(a.header().arm_columns, b.header().arm_columns);
  for (long k = 0; k < a.ticks(); ++k) {
    EXPECT_EQ(a.times()[k], b.times()[k]);
    for (int j = 0; j < a.arms(); ++j) {
      for (int i = 0; i < a.agents(); ++i)
        for (int c = 0; c < static_cast<int>(a.header().agent_columns.size()); ++c)
          ASSERT_TRUE(same(a.at(k, j, i, c), b.at(k, j, i, c)))
              << "tick " << k << " arm " << j << " " << a.header().agent_columns[c];
      for (int c = 0; c < static_cast<int>(a.header().arm_columns.size()); ++c)
        ASSERT_TRUE(same(a.arm_at(k, j, c), b.arm_at(k, j, c)))
            << "tick " << k << " arm " << j << " " << a.header().arm_columns[c];
    }
  }
  ASSERT_EQ(a.legs().size(), b.legs().size());
  for (std::size_t L = 0; L < a.legs().size(); ++L) {
    EXPECT_EQ(a.legs()[L].tick, b.legs()[L].tick);
    EXPECT_EQ(a.legs()[L].goals, b.legs()[L].goals);
    EXPECT_EQ(a.legs()[L].nav_stats, b.legs()[L].nav_stats);
  }
  EXPECT_EQ(a.finished(), b.finished());
  EXPECT_EQ(a.end_nav_stats(), b.end_nav_stats());
}

} // namespace

TEST(NavArms, TheBooksCloseAndBookAGoalChangeAsContext) {
  drive_telemetry t;
  t.potential = 2.0f;
  t.potential_end = 1.5f;
  t.projection_work = 0.25f;
  t.context_work = 0.0f;
  // first tick: dU starts at this tick's potential
  nav_books b = books_of(t, nullptr, 0.4, 0.0);
  EXPECT_DOUBLE_EQ(b.dU, -0.5);
  EXPECT_DOUBLE_EQ(b.drive, (1.5 - 0.25) - 2.0);
  EXPECT_DOUBLE_EQ(b.projection, 0.25);
  EXPECT_DOUBLE_EQ(b.dKE, 0.5 * 0.4 * 0.4);
  EXPECT_NEAR(b.residual, 0.0, 1e-15);
  // a goal moved between the ticks: U at the same pose went up by 0.75, booked as context
  const double prev_end = 1.25;
  t.context_work = 0.75f;
  b = books_of(t, &prev_end, 0.4, 0.4);
  EXPECT_DOUBLE_EQ(b.dU, 1.5 - 1.25);
  EXPECT_DOUBLE_EQ(b.context, 0.75);
  EXPECT_NEAR(b.residual, 0.0, 1e-12);
  EXPECT_NEAR(b.dKE, 0.0, 1e-15); // a fused multiply-add may leave a last bit (arm64)
  // something moved the agent between the ticks: the books do not explain it
  t.context_work = 0.0f;
  b = books_of(t, &prev_end, 0.4, 0.4);
  EXPECT_NEAR(b.residual, 0.75, 1e-12);
  ASSERT_EQ(book_names().size(), 6u);
  for (int i = 0; i < 6; ++i)
    EXPECT_TRUE(std::isfinite(book_value(b, i)));
  EXPECT_TRUE(std::isnan(book_value(b, 6)));
}

TEST(NavArms, TelemetryFieldsAreNamedAsPycvcNamesThem) {
  drive_telemetry t;
  t.alpha = 1.5f;
  t.terms.barrier = 0.25f;
  t.context_work = -2.0f;
  t.binding = 1;
  std::vector<std::string> names;
  for (const telemetry_field &f : telemetry_fields()) {
    names.push_back(f.name);
    if (std::string(f.name) == "alpha")
      EXPECT_EQ(f.get(t), 1.5);
    if (std::string(f.name) == "u_barrier")
      EXPECT_EQ(f.get(t), 0.25);
    if (std::string(f.name) == "context_work")
      EXPECT_EQ(f.get(t), -2.0);
    if (std::string(f.name) == "binding")
      EXPECT_EQ(f.get(t), 1.0);
  }
  EXPECT_EQ(names.size(), 25u);
  EXPECT_EQ(names.front(), "alpha");
  EXPECT_EQ(names.back(), "context_work");
}

TEST(NavArms, AStatsRowOverItsAgents) {
  episode_nav_stats e;
  e.ticks = 100;
  e.dt_s = 0.05;
  veh_nav_stats a, b;
  a.min_clearance_m = 3.0;
  a.penetration_steps = 2;
  a.total_path_m = 40.0;
  a.arrived = true;
  a.time_to_goal_s = 4.0;
  b.min_clearance_m = 1e30; // never measured
  b.penetration_steps = 1;
  b.total_path_m = 20.0;
  e.per_vehicle = {a, b};
  arm_stats s = stats_of(e, 0.5, 30.0, 10, 6.0, 3);
  EXPECT_DOUBLE_EQ(s.clear_min_m, 3.0);
  EXPECT_DOUBLE_EQ(s.clear_mean_m, 3.0);
  EXPECT_DOUBLE_EQ(s.pen_steps, 3.0);
  EXPECT_DOUBLE_EQ(s.path_m, 30.0);
  EXPECT_DOUBLE_EQ(s.elapsed_s, 5.0); // not every agent is there: the leg's time
  EXPECT_DOUBLE_EQ(s.ms_step, 2.0);
  EXPECT_DOUBLE_EQ(s.reached, 1.0);
  e.per_vehicle[1].arrived = true;
  e.per_vehicle[1].time_to_goal_s = 4.5;
  s = stats_of(e, 0.5, 30.0, 10, 6.0, 3);
  EXPECT_DOUBLE_EQ(s.elapsed_s, 4.5); // every agent there: the last arrival
  // nothing measured by the collector: the telemetry's least clearance; nothing at all: NaN
  e.per_vehicle = {b};
  s = stats_of(e, 0.5, 30.0, 10, 0.0, 0);
  EXPECT_DOUBLE_EQ(s.clear_min_m, 0.5);
  EXPECT_TRUE(std::isnan(s.ms_step));
  s = stats_of(e, 0.0, 0.0, 0, 0.0, 0);
  EXPECT_TRUE(std::isnan(s.clear_min_m));
  EXPECT_TRUE(std::isnan(s.clear_mean_m));
  EXPECT_EQ(stats_columns().size(), 7u);
  EXPECT_TRUE(std::isnan(s.value(7)));
}

TEST(NavArms, EveryArmStepsInLockstepFromTheSameStart) {
  auto run = three_arms();
  ASSERT_EQ(run->arms(), 3);
  ASSERT_EQ(run->agents(), 1);
  for (int k = 0; k < 120; ++k)
    run->step();
  const run_log &log = run->log();
  ASSERT_EQ(log.ticks(), 120);
  EXPECT_DOUBLE_EQ(log.times().back(), 120 * 0.05); // the float tick as its decimal
  EXPECT_DOUBLE_EQ(log.dt(), 0.05);
  for (int a = 0; a < 3; ++a) {
    EXPECT_EQ(run->world(a).tick(), 120);
    EXPECT_EQ(log.header().arms[a].slot, a);
  }
  EXPECT_EQ(log.header().start.size(), 3u);
  EXPECT_NEAR(log.header().start[0], -26.0, 1e-4);
  EXPECT_EQ(log.header().scene, "city");
  EXPECT_EQ(log.header().seed, 7u);
  EXPECT_EQ(log.header().plant, "kinematic");
  // they differ: the untrained nets steer otherwise than the default
  EXPECT_NE(log.at(119, 0, 0, "x"), log.at(119, 1, 0, "x"));
  // the keys their worlds gave them
  EXPECT_EQ(log.header().arms[0].key, "");
  EXPECT_EQ(log.header().arms[1].key, "test");
  // wall times measured, Stats rows filled
  for (int a = 0; a < 3; ++a) {
    EXPECT_GE(log.arm_at(119, a, "ms"), 0.0);
    EXPECT_TRUE(std::isfinite(log.arm_at(119, a, "stats.ms_step")));
    EXPECT_GT(log.arm_at(119, a, "stats.path_m"), 1.0);
    EXPECT_TRUE(std::isfinite(log.arm_at(119, a, "stats.clear_mean_m")));
  }
}

TEST(NavArms, TheBooksCloseOnTheKinematicPlant) {
  auto run = three_arms(3);
  for (int k = 0; k < 200; ++k)
    run->step();
  const run_log &log = run->log();
  double worst = 0.0;
  for (long k = 0; k < log.ticks(); ++k)
    for (int a = 0; a < 3; ++a)
      for (int i = 0; i < 3; ++i) {
        const double r = log.at(k, a, i, "book.residual");
        ASSERT_TRUE(std::isfinite(r));
        worst = std::max(worst, std::abs(r));
        // dU telescopes: the sum of the ticks' dU is U's change from the start to the end
      }
  EXPECT_LT(worst, 1e-5) << "the books leave " << worst << " unexplained";
  for (int a = 0; a < 3; ++a) {
    const double sum = log.sum_to(log.ticks() - 1, a, 0, "book.dU");
    const double u0 = log.at(0, a, 0, "tel.potential");
    const double u1 = log.at(log.ticks() - 1, a, 0, "tel.potential_end");
    EXPECT_NEAR(sum, u1 - u0, 1e-4);
  }
}

TEST(NavArms, AnArmRunsBitForBitAsItsPolicyAlone) {
  auto run = three_arms();
  auto alone = make_world(random_policy(1));
  for (int k = 0; k < 150; ++k) {
    run->step();
    alone->step();
    const sim_world &w = run->world(1);
    ASSERT_EQ(w.positions_normalized()[0], alone->positions_normalized()[0]) << "tick " << k;
    ASSERT_EQ(w.positions_normalized()[1], alone->positions_normalized()[1]) << "tick " << k;
    ASSERT_EQ(w.headings()[0], alone->headings()[0]) << "tick " << k;
    ASSERT_EQ(w.speeds_normalized()[0], alone->speeds_normalized()[0]) << "tick " << k;
  }
}

TEST(NavArms, ARetargetMovesEveryArmsGoalOnTheSameTickAndStartsALeg) {
  auto run = three_arms();
  for (int k = 0; k < 60; ++k)
    run->step();
  const double path_before = run->log().arm_at(59, 0, "stats.path_m");
  run->retarget(0, -20.0, 20.0);
  for (int k = 0; k < 5; ++k)
    run->step();
  const run_log &log = run->log();
  ASSERT_EQ(log.legs().size(), 2u);
  EXPECT_EQ(log.legs()[1].tick, 60);
  EXPECT_NEAR(log.legs()[1].goals[0], -20.0, 1e-4);
  EXPECT_NEAR(log.legs()[1].goals[1], 20.0, 1e-4);
  ASSERT_EQ(log.legs()[0].nav_stats.size(), 3u); // each arm's record when the leg ended
  EXPECT_NE(log.legs()[0].nav_stats[0].find("\"per_vehicle\""), std::string::npos);
  EXPECT_EQ(log.leg_of(59), 0);
  EXPECT_EQ(log.leg_of(60), 1);
  for (int a = 0; a < 3; ++a) {
    EXPECT_NEAR(log.at(60, a, 0, "goal_x"), -20.0, 1e-4); // every arm, the same tick
    EXPECT_NEAR(log.at(59, a, 0, "goal_x"), 26.0, 1e-4);
    // the new leg's Stats start afresh
    EXPECT_LT(log.arm_at(60, a, "stats.path_m"), path_before);
    // the goal's change is context work, and the books still close
    EXPECT_NE(log.at(60, a, 0, "book.context"), 0.0);
    EXPECT_LT(std::abs(log.at(60, a, 0, "book.residual")), 1e-5);
  }
  EXPECT_THROW(run->retarget(3, 0.0, 0.0), std::out_of_range);
  EXPECT_THROW(run->retarget(nullptr, 1), std::invalid_argument);
}

TEST(NavArms, TheKinematicPlantFeedsNoPlantLedgerItem) {
  auto run = three_arms();
  run->step();
  const run_log &log = run->log();
  int items = 0;
  for (const plant_ledger_item &it : plant_ledger_vehicle()) {
    EXPECT_TRUE(std::isnan(log.at(0, 0, 0, std::string("plant.") + it.key))) << it.key;
    ++items;
  }
  EXPECT_EQ(items, 20);
  const panel_table t = plant_table(log, 0, 0, 0);
  ASSERT_EQ(t.rows.size(), 20u);
  for (const std::vector<std::string> &r : t.rows)
    EXPECT_EQ(r[1], "n/a");
  EXPECT_EQ(plant_note(log, 0, 0, 0),
            "default on kinematic: no item applies (a physics plant feeds them)");
}

TEST(NavArms, TheLogReadsBackValueForValue) {
  const std::string path = tmp_path("stream.cvcrun");
  auto run = three_arms(2, path);
  for (int k = 0; k < 40; ++k)
    run->step();
  run->retarget(1, 0.0, -20.0);
  for (int k = 0; k < 30; ++k)
    run->step();
  run->finish();
  run->finish(); // idempotent
  EXPECT_THROW(run->step(), std::logic_error);
  const run_log &live = run->log();
  // written whole and read back
  std::stringstream ss;
  live.write(ss);
  const run_log back = run_log::read(ss);
  expect_same_logs(live, back);
  // streamed as it ran and read back
  const run_log streamed = run_log::read(path);
  expect_same_logs(live, streamed);
  EXPECT_TRUE(streamed.finished());
  EXPECT_EQ(streamed.header().world, R"({"kind":"test-city"})");
  EXPECT_EQ(streamed.header().arms[2].name, "net2");
  EXPECT_EQ(streamed.end_nav_stats().size(), 3u);
  // the same text, line for line
  std::stringstream file;
  {
    std::ifstream f(path);
    file << f.rdbuf();
  }
  EXPECT_EQ(file.str(), ss.str());
  run.reset();
  std::filesystem::remove(path); // finish() closed it, so it goes on Windows too
}

TEST(NavArms, ALogCutShortReadsToItsLastWholeTick) {
  auto run = three_arms();
  for (int k = 0; k < 10; ++k)
    run->step();
  std::stringstream ss;
  run->log().write(ss);
  std::string text = ss.str();
  text.resize(text.size() - 40); // the last tick's line cut short, no end
  std::stringstream cut(text);
  const run_log back = run_log::read(cut);
  EXPECT_EQ(back.ticks(), 9);
  EXPECT_FALSE(back.finished());
  ASSERT_EQ(back.notes().size(), 1u);
  EXPECT_NE(back.notes()[0].find("cut short"), std::string::npos);
}

TEST(NavArms, AFileThatIsNotARunLogIsRefused) {
  std::stringstream empty;
  EXPECT_THROW(run_log::read(empty), std::runtime_error);
  std::stringstream other("{\"schema\":\"cvc.input_trace/v1\"}\n");
  EXPECT_THROW(run_log::read(other), std::runtime_error);
  std::stringstream bad("{\"schema\":\"cvc.nav.run_log/v1\",\"agents\":1,\"arms\":[{}],"
                        "\"columns\":{\"agent\":[\"x\"],\"arm\":[]}}\n"
                        "not json\n{\"tick\":1}\n");
  EXPECT_THROW(run_log::read(bad), std::runtime_error);
  std::stringstream wrong("{\"schema\":\"cvc.nav.run_log/v1\",\"agents\":1,\"arms\":[{}],"
                          "\"columns\":{\"agent\":[\"x\"],\"arm\":[]}}\n"
                          "{\"tick\":1,\"t\":0.05,\"agent\":[[[1,2]]],\"arm\":[[]]}\n");
  EXPECT_THROW(run_log::read(wrong), std::runtime_error);
  EXPECT_THROW(run_log::read(tmp_path("does-not-exist")), std::runtime_error);
  run_log_header h;
  h.meta = "[1]";
  EXPECT_THROW(h.to_json(), std::invalid_argument);
}

TEST(NavArms, ArmsThatCannotRunTogetherAreRefused) {
  auto w0 = make_world(), w1 = make_world(nullptr, 2);
  EXPECT_THROW(arms_run(std::vector<sim_world *>{w0.get(), w1.get()}, {{"a"}, {"b"}}),
               std::invalid_argument);
  auto w2 = make_world();
  EXPECT_THROW(arms_run(std::vector<sim_world *>{w0.get(), w2.get()}, {{"a"}, {"a"}}),
               std::invalid_argument);
  EXPECT_THROW(arms_run(std::vector<sim_world *>{w0.get(), w0.get()}, {{"a"}, {"b"}}),
               std::invalid_argument);
  EXPECT_THROW(arms_run(std::vector<sim_world *>{w0.get()}, {{"a"}, {"b"}}), std::invalid_argument);
  EXPECT_THROW(arms_run(std::vector<sim_world *>{}, {}), std::invalid_argument);
  EXPECT_THROW(arms_run(std::vector<sim_world *>{nullptr}, {{"a"}}), std::invalid_argument);
  arms_run run(std::vector<sim_world *>{w0.get(), w2.get()}, {{"a"}, {"b"}});
  w0->step();
  run.record(0, 1.0);
  EXPECT_THROW(run.record(0, 1.0), std::logic_error);
  EXPECT_THROW(run.end_tick(), std::logic_error);            // arm b not recorded
  EXPECT_THROW(run.retarget(0, 1.0, 1.0), std::logic_error); // inside a tick
  EXPECT_THROW(run.record(2, 1.0), std::out_of_range);
  w2->step();
  run.record(1, 2.0);
  run.end_tick();
  EXPECT_EQ(run.log().ticks(), 1);
  EXPECT_EQ(run.log().arm_at(0, 1, "ms"), 2.0);
}

TEST(NavArms, AHostsSizesAreRecorded) {
  auto run = three_arms();
  run->step();
  EXPECT_NEAR(run->log().at(0, 0, 0, "wheelbase"), 3.3, 1e-5);
  EXPECT_NEAR(run->log().at(0, 0, 0, "width"), 2.0, 1e-5); // the footprint disc: 2 rr
  const float len = 4.84f, wid = 2.18f, wb = 3.3f;
  run->set_size(1, &len, &wid, &wb);
  run->step();
  EXPECT_FLOAT_EQ(static_cast<float>(run->log().at(1, 1, 0, "length")), 4.84f);
  EXPECT_FLOAT_EQ(static_cast<float>(run->log().at(1, 1, 0, "width")), 2.18f);
  EXPECT_THROW(run->set_size(5, &len, &wid, &wb), std::out_of_range);
}

TEST(NavPanels, TheTablesAtATickAreTheSameReadBack) {
  auto run = three_arms(2);
  for (int k = 0; k < 50; ++k)
    run->step();
  run->retarget(0, 10.0, 25.0);
  for (int k = 0; k < 20; ++k)
    run->step();
  std::stringstream ss;
  run->log().write(ss);
  const run_log back = run_log::read(ss);
  for (long k : {0L, 33L, 50L, 69L})
    for (int agent : {0, 1}) {
      const std::vector<panel_table> live = arms_tables(run->log(), k, agent);
      const std::vector<panel_table> replay = arms_tables(back, k, agent);
      ASSERT_EQ(live.size(), 5u);
      for (std::size_t t = 0; t < live.size(); ++t) {
        EXPECT_EQ(live[t].key, replay[t].key);
        EXPECT_EQ(live[t].columns, replay[t].columns);
        EXPECT_EQ(live[t].rows, replay[t].rows) << live[t].key << " at tick " << k;
      }
      EXPECT_EQ(plant_table(run->log(), k, 1, agent).rows, plant_table(back, k, 1, agent).rows);
    }
  const std::vector<panel_table> t = arms_tables(back, 69, 0);
  EXPECT_EQ(t[0].key, "arms.stats");
  EXPECT_EQ(t[0].rows.size(), 3u);
  EXPECT_EQ(t[0].cell(1, "arm"), "net1");
  EXPECT_EQ(t[0].cell(1, "colour"), "orange");
  EXPECT_NE(t[0].cell(0, "reached").find("/2"), std::string::npos);
  EXPECT_EQ(t[0].cell(9, "arm"), "");
  EXPECT_EQ(t[1].key, "ledger.coef");
  EXPECT_EQ(t[1].columns[1], "alpha");
  EXPECT_EQ(t[3].key, "ledger.books");
  EXPECT_EQ(t[4].key, "ledger.tick");
  // before the first tick every number is "-"
  const std::vector<panel_table> none = arms_tables(back, -1, 0);
  EXPECT_EQ(none[0].cell(0, "clear_min"), "-");
  EXPECT_EQ(none[2].cell(0, "potential"), "-");
}

TEST(NavPanels, NumbersAndColours) {
  EXPECT_EQ(panel_number(1.23456, 2), "1.23");
  EXPECT_EQ(panel_number(-0.0001, 2), "0.00");
  EXPECT_EQ(panel_number(std::nan(""), 2), "-");
  EXPECT_EQ(panel_number(12.0, 0), "12");
  EXPECT_STREQ(slot_colour(0), "blue");
  EXPECT_STREQ(slot_colour(9), "orange");
  EXPECT_STREQ(slot_colour(-1), "red");
}

TEST(NavPanels, TheChartsDrawEveryArmWithAMarkerAtEachLeg) {
  auto run = three_arms();
  for (int k = 0; k < 30; ++k)
    run->step();
  run->retarget(0, 0.0, 25.0);
  for (int k = 0; k < 30; ++k)
    run->step();
  const std::vector<chart_plot> c = arms_charts(run->log(), 1, 0, "context");
  ASSERT_EQ(c.size(), 3u);
  EXPECT_EQ(c[0].key, "arms.clearance");
  ASSERT_EQ(c[0].series.size(), 3u);
  EXPECT_EQ(c[0].series[2].style, "c2");
  EXPECT_EQ(c[0].series[0].x.size(), 60u);
  ASSERT_EQ(c[0].markers.size(), 1u);
  EXPECT_EQ(c[0].markers[0].label, "leg 2");
  EXPECT_NEAR(c[0].markers[0].x, 30 * 0.05, 1e-6);
  EXPECT_EQ(c[1].key, "ledger.cum");
  EXPECT_EQ(c[1].series.size(), 6u);
  EXPECT_EQ(c[2].key, "ledger.compare");
  EXPECT_EQ(c[2].series[1].label, "net1");
  // the cumulative books end at the summed table's value
  EXPECT_EQ(panel_number(c[2].series[1].y.back(), 3),
            arms_tables(run->log(), 59, 0)[3].cell(1, "context"));
}

TEST(NavPanels, TheStateTreeHoldsTablesAndChartsAsTheDocumentsBindThem) {
  cvc::app app;
  panel_table t{"demo.t", {"a", "b"}, {{"1", "2"}, {"3", "4"}, {"5", "6"}}};
  write_table(app, "doc", t);
  EXPECT_EQ(panel_get(app, "doc.demo.t"), "3");
  EXPECT_EQ(panel_get(app, "doc.demo.t.2.b"), "6");
  EXPECT_EQ(panel_get(app, "doc.demo.t_columns.1"), "b");
  t.rows.resize(1); // fewer rows: the rest go
  write_table(app, "doc", t);
  EXPECT_EQ(panel_get(app, "doc.demo.t"), "1");
  EXPECT_EQ(panel_get(app, "doc.demo.t.2.b", "gone"), "gone");
  chart_plot c{"demo.c",
               {{"one", "c1 dashed", {0, 1, 2.5}, {1.0, std::nan(""), 3.0}}},
               {{2.5, "here", "best"}}};
  write_chart(app, "", c);
  EXPECT_EQ(panel_get(app, "demo.c.s"), "1");
  EXPECT_EQ(panel_get(app, "demo.c.s.0.x"), "0,1,2.5");
  EXPECT_EQ(panel_get(app, "demo.c.s.0.y"), "1,nan,3");
  EXPECT_EQ(panel_get(app, "demo.c.s.0.style"), "c1 dashed");
  EXPECT_EQ(panel_get(app, "demo.c.m.0.kind"), "best");
  // a long series is cut to its extremes
  chart_series big{"big", "", {}, {}};
  for (int i = 0; i < 5000; ++i) {
    big.x.push_back(i);
    big.y.push_back(i == 1234 ? 99.0 : std::sin(i * 0.01));
  }
  write_chart(app, "", {"demo.big", {big}, {}}, 300);
  const std::string y = panel_get(app, "demo.big.s.0.y");
  EXPECT_LT(std::count(y.begin(), y.end(), ','), 300);
  EXPECT_NE(y.find("99"), std::string::npos);
  EXPECT_EQ(panel_path("", "k"), "k");
  EXPECT_EQ(panel_path("p", "k"), "p.k");
}

TEST(NavPanels, ThePlayHeadFollowsTheTransport) {
  cvc::app app;
  play_head h(10.0, 0.05);
  EXPECT_DOUBLE_EQ(h.update(app, "", 0.5), 0.0); // the first frame shows the start
  EXPECT_EQ(panel_get(app, "play.pos"), "0");
  EXPECT_DOUBLE_EQ(h.update(app, "", 0.5), 0.5);
  panel_set(app, "sim.speed", "2");
  EXPECT_DOUBLE_EQ(h.update(app, "", 0.5), 1.5);
  panel_set(app, "sim.paused", "1");
  EXPECT_DOUBLE_EQ(h.update(app, "", 0.5), 1.5);
  h.step();
  EXPECT_NEAR(h.time(), 1.55, 1e-12);
  panel_set(app, "play.pos", "0.5"); // dragged: the slider wins
  EXPECT_DOUBLE_EQ(h.update(app, "", 0.5), 5.0);
  EXPECT_EQ(panel_get(app, "play.t"), "5.0000");
  panel_set(app, "play.seek", "7.25");
  h.seek(app, "");
  EXPECT_DOUBLE_EQ(h.time(), 7.25);
  h.restart(app, "");
  EXPECT_EQ(panel_get(app, "sim.paused"), "0");
  EXPECT_DOUBLE_EQ(h.update(app, "", 0.0), 0.0);
  h.set(9.9); // the end: stop there
  h.update(app, "", 1.0);
  EXPECT_DOUBLE_EQ(h.time(), 10.0);
  EXPECT_EQ(panel_get(app, "sim.paused"), "1");
  panel_set(app, "sim.paused", "0"); // looping: back to the start
  panel_set(app, "play.loop", "1");
  h.update(app, "", 1.0);
  EXPECT_DOUBLE_EQ(h.time(), 0.0);
}

TEST(NavPanels, AReplayShowsTheTickAtThePlayHead) {
  auto run = three_arms();
  for (int k = 0; k < 40; ++k)
    run->step();
  run->finish();
  cvc::app app;
  replay_view view(run->shared_log(), 1.0);
  panel_set(app, "sim.paused", "1");
  panel_set(app, "ledger.agent", "net2");
  replay_view::frame f = view.update(app, "", 0.1);
  EXPECT_EQ(f.k, 19); // t = 1.0 s: the 20th tick's end
  EXPECT_EQ(f.choice.arm, 2);
  EXPECT_EQ(panel_get(app, "arms.time"), "t = 1.0 s, tick 20 of 40, leg 1");
  EXPECT_EQ(panel_get(app, "arms.names"), "default\nnet1\nnet2");
  EXPECT_EQ(panel_get(app, "arms.stats.2.arm"), "net2");
  EXPECT_EQ(panel_get(app, "ledger.plant"), "20");
  EXPECT_EQ(panel_get(app, "ledger.cum.s"), "6");
  EXPECT_EQ(panel_get(app, "arms.stats.1.path"), arms_tables(run->log(), 19, 0)[0].cell(1, "path"));
  ASSERT_EQ(f.poses.size(), 3u);
  EXPECT_DOUBLE_EQ(f.poses[1][0].x, run->log().at(19, 1, 0, "x"));
  // between two ticks the ghosts are between their poses
  view.head().set(1.025);
  f = view.update(app, "", 0.0);
  const double x0 = run->log().at(19, 0, 0, "x"), x1 = run->log().at(20, 0, 0, "x");
  EXPECT_NEAR(f.poses[0][0].x, 0.5 * (x0 + x1), 1e-5);
  EXPECT_THROW(replay_view(nullptr), std::invalid_argument);
}

TEST(NavPanels, TwoLogsSideBySide) {
  auto a = three_arms(), b = three_arms();
  for (int k = 0; k < 20; ++k) {
    a->step();
    b->step();
  }
  a->finish();
  b->finish();
  const run_log both = run_log::stack({&a->log(), &b->log()}, {0, 2}, {"step 0", "step 9"});
  EXPECT_EQ(both.arms(), 2);
  EXPECT_EQ(both.header().arms[1].name, "step 9");
  EXPECT_EQ(both.header().arms[1].slot, 1);
  EXPECT_EQ(both.header().arms[1].key, "test");
  EXPECT_EQ(both.at(19, 1, 0, "x"), b->log().at(19, 2, 0, "x"));
  EXPECT_EQ(both.end_nav_stats().size(), 2u);
  auto c = three_arms();
  c->step();
  EXPECT_THROW(run_log::stack({&a->log(), &c->log()}, {0, 0}, {"x", "y"}), std::invalid_argument);
  EXPECT_THROW(run_log::stack({&a->log()}, {5}, {"x"}), std::invalid_argument);
  EXPECT_THROW(run_log::stack({}, {}, {}), std::invalid_argument);
}
