/*
  Copyright 2007-2011 The University of Texas at Austin
        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>
  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// nav_stats_test — the base (RF-free) cvc::nav::nav_stats collector + scorecard,
// driven over a hand-computed scripted trajectory. Every accumulator is checked
// against a value worked out on paper; the same corpus + numbers are mirrored in
// CVC-DBG/cvcdbg tests/nav_stats_test.cpp and CVC-Lab/GRL-SNAM tests/test_scorecard.py
// (the shared cross-repo schema contract).

#include <array>
#include <cmath>
#include <cvc/nav/nav_stats.h>
#include <gtest/gtest.h>
#include <string>
#include <vector>

using namespace cvc::nav;

TEST(NavStats, ScriptedTrajectoryAccumulators) {
  const int N = 2;
  const float start[4] = {0, 0, 0, 100};
  const float goal[4] = {30, 0, 30, 100}; // straight_m = 30 for both

  nav_stats_params p;
  p.turn_event_rad = 0.2;
  p.clear_safety_m = 2.0;
  p.speed_eps_mps = 0.1;
  p.min_gap_m = 0.0;
  nav_stats_collector c(p);

  budget_policy budget;
  budget.time_budget_s = 3.5; // veh0 arrives at t=3.0 (ok); veh1 never (elapsed 4 > 3.5)
  c.begin_episode(N, 1.0, start, goal, budget, "scene", 7, "ckpt-A");
  c.set_identity(0, /*convoy*/ 1, /*class*/ 2, /*radius*/ 3.0, /*mass*/ 4000.0);

  const double kPi = 3.14159265358979323846;
  const float posS[4][4] = {{10, 0, 0, 110}, {20, 0, 0, 120}, {30, 0, 10, 120}, {30, 0, 10, 120}};
  const float headS[4][2] = {{0, (float)(kPi / 2)}, {0, (float)(kPi / 2)}, {0, 0}, {0, 0}};
  const float spdS[4][2] = {{10, 10}, {10, 10}, {10, 10}, {0, 0}};
  const std::uint8_t rchS[4][2] = {{0, 0}, {0, 0}, {1, 0}, {1, 0}};
  const double clrS[4][2] = {{5, 3}, {5, 1.0}, {5, 1.5}, {5, 4}};

  double clr[2];
  nav_samplers smp;
  smp.min_clearance_m = clr;
  smp.occupied = [](double x, double y) { return x < 5.0 && y > 115.0; };
  smp.material_id = [](double, double y) { return y >= 100.0 ? 5 : 3; };

  const int mode_seek[2] = {0, 0};
  for (int s = 0; s < 4; ++s) {
    clr[0] = clrS[s][0];
    clr[1] = clrS[s][1];
    c.step(posS[s], headS[s], spdS[s], mode_seek, rchS[s], smp);
  }
  episode_nav_stats e = c.finish();

  EXPECT_EQ(e.n_vehicles, 2);
  EXPECT_EQ(e.ticks, 4);
  EXPECT_EQ(e.scene_id, "scene");
  EXPECT_EQ(e.checkpoint, "ckpt-A");
  const veh_nav_stats &v0 = e.per_vehicle[0];
  const veh_nav_stats &v1 = e.per_vehicle[1];

  EXPECT_EQ(v0.convoy_id, 1);
  EXPECT_EQ(v0.vehicle_class, 2);
  EXPECT_DOUBLE_EQ(v0.robot_radius_m, 3.0);
  EXPECT_DOUBLE_EQ(v0.mass_kg, 4000.0);

  EXPECT_NEAR(v0.total_path_m, 30.0, 1e-4);
  EXPECT_NEAR(v1.total_path_m, 30.0, 1e-4);
  EXPECT_NEAR(v0.straight_m, 30.0, 1e-4);
  EXPECT_NEAR(v0.turn_total_rad, 0.0, 1e-6);
  EXPECT_NEAR(v1.turn_total_rad, kPi / 2, 1e-4);
  EXPECT_EQ(v0.turn_events, 0);
  EXPECT_EQ(v1.turn_events, 1);
  EXPECT_NEAR(v0.speed_mean, 7.5, 1e-6);
  EXPECT_NEAR(v0.speed_max, 10.0, 1e-6);
  EXPECT_NEAR(v0.time_moving_s, 3.0, 1e-6);
  EXPECT_NEAR(v0.time_stopped_s, 1.0, 1e-6);
  EXPECT_NEAR(v0.accel_integral, 10.0, 1e-6);
  EXPECT_NEAR(v0.fuel_used, 10.0, 1e-6);
  EXPECT_NEAR(v0.min_clearance_m, 5.0, 1e-6);
  EXPECT_NEAR(v0.time_below_clear_s, 0.0, 1e-6);
  EXPECT_NEAR(v1.min_clearance_m, 1.0, 1e-6);
  EXPECT_NEAR(v1.time_below_clear_s, 2.0, 1e-6);
  EXPECT_EQ(v1.penetration_steps, 1);
  EXPECT_EQ(v0.penetration_steps, 0);
  EXPECT_NEAR(v0.time_over_material_s[3], 4.0, 1e-6);
  EXPECT_NEAR(v0.dist_over_material_m[3], 30.0, 1e-4);
  EXPECT_NEAR(v1.time_over_material_s[5], 4.0, 1e-6);
  EXPECT_TRUE(v0.arrived);
  EXPECT_FALSE(v0.timed_out);
  EXPECT_NEAR(v0.time_to_goal_s, 3.0, 1e-6);
  EXPECT_FALSE(v1.arrived);
  EXPECT_TRUE(v1.timed_out);
  EXPECT_FALSE(v0.over_budget);
  EXPECT_TRUE(v1.over_budget);

  EXPECT_EQ(e.arrived, 1);
  EXPECT_FALSE(e.success);
  EXPECT_NEAR(e.makespan_s, 3.0, 1e-6);
  EXPECT_NEAR(e.mean_ttg_s, 3.0, 1e-6);
  EXPECT_NEAR(e.penetration_pct, 12.5, 1e-6);
  EXPECT_NEAR(e.mean_path_ratio, 1.0, 1e-6);
  EXPECT_NEAR(e.mean_turn_total_rad, kPi / 4, 1e-4);
  EXPECT_NEAR(e.total_fuel, 20.0, 1e-6);
  EXPECT_EQ(v0.goals_reached, 1);
  EXPECT_EQ(v1.goals_reached, 0);

  // Pin the base RECORD shape (the cross-repo JSON contract), not just the accumulators:
  // a regression in field names, ordering, or the 13-wide material arrays would slip past
  // the field-level asserts above. Both vehicles measured clearance here, so it is a real
  // value (not the sentinel -> null path, which SentinelFieldsSerializeAsNull covers).
  const std::string ej = e.to_json();
  EXPECT_NE(ej.find("\"per_vehicle\":["), std::string::npos);
  EXPECT_NE(ej.find("\"min_clearance_m\":5"), std::string::npos);  // v0 = 5 m, emitted
  EXPECT_NE(ej.find("\"min_clearance_m\":1,"), std::string::npos); // v1 = 1 m, emitted
  EXPECT_NE(ej.find("\"time_over_material_s\":["), std::string::npos);
  EXPECT_NE(ej.find("\"dist_over_material_m\":["), std::string::npos);
  EXPECT_EQ(ej.find("1e+30"), std::string::npos); // no raw sentinel leaks into the record
}

// Gap 2: the "unmeasured" sentinel (1e30) must serialize as null, not a huge finite
// number. A single vehicle with no clearance sampler leaves min_clearance_m and (n==1)
// min_sep_m at 1e30; the record must read "not measured", not "enormous clearance".
TEST(NavStats, SentinelFieldsSerializeAsNull) {
  const float start[2] = {0, 0};
  const float goal[2] = {10, 0};
  nav_stats_collector c;
  c.begin_episode(1, 1.0, start, goal);
  const float pos[2] = {1, 0};
  const float head[1] = {0};
  const float spd[1] = {1};
  const std::uint8_t rch[1] = {0};
  const int mode[1] = {0};
  c.step(pos, head, spd, mode, rch); // no nav_samplers -> min_clearance_m stays 1e30
  episode_nav_stats e = c.finish();

  EXPECT_GE(e.per_vehicle[0].min_clearance_m, 1e29); // still at the sentinel in-struct
  EXPECT_GE(e.min_sep_m, 1e29);                      // n==1 -> never lowered
  const std::string js = e.to_json();
  EXPECT_EQ(js.find("1e+30"), std::string::npos); // no raw sentinel
  EXPECT_NE(js.find("\"min_clearance_m\":null"), std::string::npos);
  EXPECT_NE(js.find("\"min_sep_m\":null"), std::string::npos);
}

// Gap 8: the eta_multiple and fuel_budget bounds (beyond the fixed time budget) each
// independently flip over_budget. The ETA path divides by speed_ref_mps, so a regression
// there or in the fuel bound would otherwise pass CI.
TEST(NavStats, BudgetEtaAndFuelBounds) {
  const int N = 1;
  const float start[2] = {0, 0};
  const float goal[2] = {100, 0}; // straight_m = 100
  const float pos[2] = {10, 0};
  const float head[1] = {0};
  const float spd[1] = {10};
  const std::uint8_t rch[1] = {0};
  const int mode[1] = {0};

  // ETA bound: straight 100 m at speed_ref 10 m/s -> 10 s nominal; eta_multiple 1.0 => a
  // 10 s budget. Run 12 steps of 1 s (never arriving) => used_t = 12 > 10 => over_budget.
  {
    nav_stats_collector c;
    budget_policy b;
    b.eta_multiple = 1.0;
    b.speed_ref_mps = 10.0;
    c.begin_episode(N, 1.0, start, goal, b);
    for (int s = 0; s < 12; ++s)
      c.step(pos, head, spd, mode, rch);
    episode_nav_stats e = c.finish();
    EXPECT_TRUE(e.per_vehicle[0].over_budget);
    EXPECT_FALSE(e.success);
  }
  // Fuel bound: fuel_used is Sigma|dspeed|. Accelerate 0->10 once (=10 effort) with a
  // fuel_budget of 5 => over_budget; the ETA/time bounds are off.
  {
    nav_stats_collector c;
    budget_policy b;
    b.fuel_budget = 5.0;
    c.begin_episode(N, 1.0, start, goal, b);
    const float spd0[1] = {0};
    c.step(pos, head, spd0, mode, rch); // first step: prev_spd seeded 0, no accel yet
    c.step(pos, head, spd, mode, rch);  // 0 -> 10 => accel_integral += 10
    episode_nav_stats e = c.finish();
    EXPECT_GE(e.per_vehicle[0].fuel_used, 5.0);
    EXPECT_TRUE(e.per_vehicle[0].over_budget);
  }
}

TEST(NavStats, WallDwellAndVehicleContacts) {
  const int N = 2;
  const float start[4] = {0, 0, 1, 0}; // 1 m apart
  const float goal[4] = {100, 0, 100, 0};
  nav_stats_params p;
  p.min_gap_m = 2.0; // 1 m apart < 2 -> contact every frame
  nav_stats_collector c(p);
  c.begin_episode(N, 1.0, start, goal);

  const float pos[4] = {0, 0, 1, 0};
  const float head[2] = {0, 0};
  const float spd[2] = {1, 1};
  const std::uint8_t rch[2] = {0, 0};
  const int mode0[2] = {1, 0}; // veh0 wall-follow, veh1 seek
  c.step(pos, head, spd, mode0, rch);
  c.step(pos, head, spd, mode0, rch);
  episode_nav_stats e = c.finish();

  EXPECT_NEAR(e.per_vehicle[0].time_in_wall_s, 2.0, 1e-6);
  EXPECT_EQ(e.per_vehicle[0].wall_entries, 1);
  EXPECT_DOUBLE_EQ(e.per_vehicle[1].time_in_wall_s, 0.0);
  EXPECT_EQ(e.per_vehicle[0].veh_contacts, 2);
  EXPECT_EQ(e.per_vehicle[1].veh_contacts, 2);
  EXPECT_NEAR(e.min_sep_m, 1.0, 1e-6);
  EXPECT_EQ(e.total_veh_contacts, 4);
}

static veh_nav_stats mkv(bool arrived, double ttg, double path, double straight, double turn,
                         double fuel, int contacts) {
  veh_nav_stats v;
  v.arrived = arrived;
  v.time_to_goal_s = ttg;
  v.total_path_m = path;
  v.straight_m = straight;
  v.turn_total_rad = turn;
  v.fuel_used = fuel;
  v.veh_contacts = contacts;
  return v;
}

TEST(NavStats, ScorecardAggregatesCorpus) {
  episode_nav_stats e0;
  e0.success = true;
  e0.makespan_s = 20;
  e0.penetration_pct = 0;
  e0.min_sep_m = 5;
  e0.per_vehicle = {mkv(true, 10, 100, 100, 2, 5, 0), mkv(true, 20, 150, 100, 4, 7, 1)};
  episode_nav_stats e1;
  e1.success = false;
  e1.makespan_s = 30;
  e1.penetration_pct = 10;
  e1.min_sep_m = 3;
  e1.per_vehicle = {mkv(true, 30, 200, 100, 6, 9, 0), mkv(false, -1, 120, 100, 3, 8, 2)};

  nav_scorecard s = aggregate_nav({e0, e1}, "ckpt-42");
  EXPECT_EQ(s.checkpoint, "ckpt-42");
  EXPECT_EQ(s.n_episodes, 2);
  EXPECT_EQ(s.n_vehicle_runs, 4);
  EXPECT_NEAR(s.success_rate, 0.5, 1e-9);
  EXPECT_NEAR(s.arrival_rate, 0.75, 1e-9);
  EXPECT_NEAR(s.mean_time_to_goal_s, 20.0, 1e-9);
  EXPECT_NEAR(s.p95_time_to_goal_s, 30.0, 1e-9);
  EXPECT_NEAR(s.mean_makespan_s, 25.0, 1e-9);
  EXPECT_NEAR(s.mean_path_ratio, 1.425, 1e-9);
  EXPECT_NEAR(s.mean_turn_total_rad, 3.75, 1e-9);
  EXPECT_NEAR(s.mean_fuel, 7.25, 1e-9);
  EXPECT_NEAR(s.mean_penetration_pct, 5.0, 1e-9);
  EXPECT_NEAR(s.veh_contacts_per_run, 0.75, 1e-9);
  EXPECT_NEAR(s.mean_min_sep_m, 4.0, 1e-9);

  const std::string js = s.to_json();
  EXPECT_NE(js.find("\"success_rate\""), std::string::npos);
  EXPECT_NE(js.find("\"material_time_share\""), std::string::npos);
}
