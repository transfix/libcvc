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

// Formation-holding path (feature ON): a follower converges on a fixed slot while its
// anchor drives to the objective. Exercises the formation_slot sampler, slot-error
// accumulation, the formation_arrived latch, and the scorecard follower/mission rates.
// Every case above leaves formation OFF (formation_tol_m=0, null sampler), so their
// pinned numbers are untouched; this is the only case that lights the new path up.
TEST(NavStats, FormationSlotAndScorecard) {
  const int N = 2;
  const float start[4] = {0, 0, 0, 0};
  const float goal[4] = {100, 0, 100, 0};

  nav_stats_params p;
  p.formation_tol_m = 1.0; // follower counts as "in slot" when final slot error < 1 m
  nav_stats_collector c(p);
  c.begin_episode(N, 1.0, start, goal, {}, "form-scene", 3, "ckpt-F");
  c.set_identity(0, /*convoy*/ 0, /*class*/ 0, /*radius*/ 2.0, /*mass*/ 4000.0, /*parent*/ -1);
  c.set_identity(1, /*convoy*/ 0, /*class*/ 0, /*radius*/ 2.0, /*mass*/ 4000.0, /*parent*/ 0);

  // veh0 = anchor driving +x to the objective; veh1 = follower converging on slot (10,0):
  // slot errors 3, 1, 0.5 -> mean 1.5, max 3, final 0.5 (< tol -> in slot).
  const float posS[3][4] = {{30, 0, 7, 0}, {60, 0, 9, 0}, {100, 0, 10.5f, 0}};
  const float headS[3][2] = {{0, 0}, {0, 0}, {0, 0}};
  const float spdS[3][2] = {{30, 3}, {30, 2}, {40, 1.5}};
  const std::uint8_t rchS[3][2] = {{0, 0}, {0, 0}, {1, 0}}; // anchor arrives at step 2

  nav_samplers smp;
  smp.formation_slot = [](int i, double &sx, double &sy) {
    if (i == 1) { // only the follower holds a slot; the anchor returns none
      sx = 10.0;
      sy = 0.0;
      return true;
    }
    return false;
  };

  const int mode_seek[2] = {0, 0};
  for (int s = 0; s < 3; ++s)
    c.step(posS[s], headS[s], spdS[s], mode_seek, rchS[s], smp);
  episode_nav_stats e0 = c.finish();

  // Anchor: no slot -> fields stay at their off defaults; parent -1.
  EXPECT_EQ(e0.per_vehicle[0].formation_parent, -1);
  EXPECT_DOUBLE_EQ(e0.per_vehicle[0].slot_error_mean_m, 0.0);
  EXPECT_DOUBLE_EQ(e0.per_vehicle[0].slot_error_max_m, 0.0);
  EXPECT_FALSE(e0.per_vehicle[0].formation_arrived);
  // Follower: errors 3, 1, 0.5 -> mean 1.5, max 3; final 0.5 < tol 1.0 -> in slot.
  EXPECT_EQ(e0.per_vehicle[1].formation_parent, 0);
  EXPECT_NEAR(e0.per_vehicle[1].slot_error_mean_m, 1.5, 1e-9);
  EXPECT_NEAR(e0.per_vehicle[1].slot_error_max_m, 3.0, 1e-9);
  EXPECT_TRUE(e0.per_vehicle[1].formation_arrived);

  const std::string ej = e0.to_json();
  EXPECT_NE(ej.find("\"formation_parent\""), std::string::npos);
  EXPECT_NE(ej.find("\"slot_error_mean_m\""), std::string::npos);
  EXPECT_NE(ej.find("\"formation_arrived\":true"), std::string::npos);

  // A second episode where the follower never reaches its slot (out of formation), so the
  // corpus rates come out fractional rather than trivially 1.0.
  episode_nav_stats e1;
  e1.success = false;
  e1.makespan_s = 40;
  e1.min_sep_m = 8;
  veh_nav_stats a = mkv(true, 30, 100, 100, 1, 5, 0); // anchor arrived
  a.convoy_id = 0;
  a.formation_parent = -1;
  veh_nav_stats f = mkv(false, -1, 200, 100, 2, 9, 0); // follower, out of slot
  f.convoy_id = 0;
  f.formation_parent = 0;
  f.slot_error_mean_m = 20.0;
  f.formation_arrived = false;
  e1.per_vehicle = {a, f};

  // Follower arrivals: 1 of 2 -> 0.5. Mission: e0 ok (anchor arrived + follower in-slot),
  // e1 not -> 1 of 2 -> 0.5. Mean follower slot error: (1.5 + 20)/2 = 10.75.
  nav_scorecard s = aggregate_nav({e0, e1}, "ckpt-F");
  EXPECT_NEAR(s.form_arrival_rate, 0.5, 1e-9);
  EXPECT_NEAR(s.form_mission_rate, 0.5, 1e-9);
  EXPECT_NEAR(s.mean_slot_error_m, 10.75, 1e-9);

  const std::string sj = s.to_json();
  EXPECT_NE(sj.find("\"form_arrival_rate\""), std::string::npos);
  EXPECT_NE(sj.find("\"form_mission_rate\""), std::string::npos);
  EXPECT_NE(sj.find("\"mean_slot_error_m\""), std::string::npos);
}

// Progress toward goal (Track 1b): closest_approach_m + stall_steps are always on, computed from
// the goal the collector holds — no sampler. A vehicle that alternates real progress with
// no-progress steps: dg = 7,7,3,3,0 (eps 0.05). Steps 1 and 3 fail to beat the closest-so-far -> 2
// stalls; the closest ever reached is 0. Then aggregate_nav folds both into the scorecard.
TEST(NavStats, ProgressStallAndClosestApproach) {
  const float start[2] = {0, 0};
  const float goal[2] = {10, 0}; // straight_m = 10 -> closest seeded at 10
  nav_stats_collector c;         // default params: stall_progress_eps_m = 0.05
  c.begin_episode(1, 1.0, start, goal);

  const float head[1] = {0};
  const float spd[1] = {1};
  const std::uint8_t rch[1] = {0};
  const int mode[1] = {0};
  const float xs[5] = {3, 3, 7, 7, 10}; // dg = 7, 7, 3, 3, 0
  for (int s = 0; s < 5; ++s) {
    const float pos[2] = {xs[s], 0};
    c.step(pos, head, spd, mode, rch);
  }
  episode_nav_stats e0 = c.finish();

  EXPECT_NEAR(e0.per_vehicle[0].closest_approach_m, 0.0, 1e-9);
  EXPECT_EQ(e0.per_vehicle[0].stall_steps, 2);

  const std::string ej = e0.to_json();
  EXPECT_NE(ej.find("\"closest_approach_m\":"), std::string::npos);
  EXPECT_NE(ej.find("\"stall_steps\":2"), std::string::npos);
  EXPECT_EQ(ej.find("1e+30"), std::string::npos); // closest_approach is finite here

  // Second episode by hand (closest 4, 6 stalls). Corpus: mean stall (2+6)/2 = 4;
  // mean closest approach (0 + 4)/2 = 2 (both finite).
  episode_nav_stats e1;
  e1.success = false;
  e1.makespan_s = 25;
  e1.min_sep_m = 9;
  veh_nav_stats v = mkv(false, -1, 120, 100, 2, 6, 0);
  v.closest_approach_m = 4.0;
  v.stall_steps = 6;
  e1.per_vehicle = {v};

  nav_scorecard s = aggregate_nav({e0, e1}, "ckpt-P");
  EXPECT_NEAR(s.mean_stall_steps, 4.0, 1e-9);
  EXPECT_NEAR(s.mean_closest_approach_m, 2.0, 1e-9);

  const std::string sj = s.to_json();
  EXPECT_NE(sj.find("\"mean_stall_steps\""), std::string::npos);
  EXPECT_NE(sj.find("\"mean_closest_approach_m\""), std::string::npos);
}

// Epistemic stats (Track 1c): the pure compute_coverage reducer over a hand-built raster, the
// per-vehicle sense_flips feed through the collector, and their scorecard aggregates. Coverage is
// computed from the same rasters both repos hold (over the binary to_occupancy output), so a Python
// twin reproduces every fraction; sense_flips is a bit-identical per-agent count, summed here.
TEST(NavStats, CoverageReducerAndSenseFlips) {
  // 2 belief planes over a 2x2 (4-cell) grid. truth (shared): cell 1 is a real obstacle.
  const std::uint8_t truth[4] = {0, 1, 0, 0};
  // belief = binary occupancy, plane0 then plane1. plane0 cell2 and plane1 cell3 are believed
  // occupied where truth is free -> two phantoms.
  const std::uint8_t belief[8] = {0, 1, 1, 0, /*plane1*/ 0, 0, 0, 1};
  const std::uint8_t everseen[8] = {1, 1, 1, 0, /*plane1*/ 1, 1, 0, 0}; // 5 of 8 ever seen
  const std::uint8_t lastvis[8] = {1, 0, 0, 0, /*plane1*/ 1, 0, 0, 0};  // 2 of 8 visible now
  nav_coverage cov = compute_coverage(truth, belief, everseen, lastvis, /*planes*/ 2, /*cells*/ 4);
  EXPECT_NEAR(cov.explored_frac, 0.625, 1e-12);      // 5/8
  EXPECT_NEAR(cov.visible_frac, 0.25, 1e-12);        // 2/8
  EXPECT_NEAR(cov.believed_free_frac, 0.625, 1e-12); // 5 belief==0 cells / 8
  EXPECT_NEAR(cov.phantom_frac, 0.25, 1e-12);        // 2/8 believed-occ where truth free

  // null / bad-arg guard -> all zero (feature off).
  nav_coverage off = compute_coverage(nullptr, belief, everseen, lastvis, 2, 4);
  EXPECT_DOUBLE_EQ(off.explored_frac, 0.0);
  EXPECT_DOUBLE_EQ(off.phantom_frac, 0.0);

  // sense_flips accumulation through the collector: per-step deltas 5,0,3 -> 8.
  nav_stats_collector c;
  const float start[2] = {0, 0};
  const float goal[2] = {10, 0};
  c.begin_episode(1, 1.0, start, goal);
  const float head[1] = {0};
  const float spd[1] = {1};
  const std::uint8_t rch[1] = {0};
  const int mode[1] = {0};
  int flips[1] = {0};
  nav_samplers smp;
  smp.sense_flips = flips;
  const int deltas[3] = {5, 0, 3};
  for (int s = 0; s < 3; ++s) {
    flips[0] = deltas[s];
    const float pos[2] = {(float)(s + 1), 0};
    c.step(pos, head, spd, mode, rch, smp);
  }
  episode_nav_stats e0 = c.finish();
  EXPECT_EQ(e0.per_vehicle[0].sense_flips, 8);
  e0.coverage =
      cov; // sim_world would fill this via compute_coverage at finish; the harness attaches it

  const std::string ej = e0.to_json();
  EXPECT_NE(ej.find("\"coverage\":{\"explored_frac\":"), std::string::npos);
  EXPECT_NE(ej.find("\"sense_flips\":8"), std::string::npos);

  // scorecard: mean coverage over episodes, mean sense_flips over vehicle-runs.
  episode_nav_stats e1;
  e1.success = true;
  e1.coverage = nav_coverage{0.375, 0.75, 0.375, 0.75};
  veh_nav_stats v = mkv(true, 10, 100, 100, 1, 5, 0);
  v.sense_flips = 2;
  e1.per_vehicle = {v};

  nav_scorecard s = aggregate_nav({e0, e1}, "ckpt-C");
  EXPECT_NEAR(s.mean_coverage.explored_frac, 0.5, 1e-12);      // (0.625+0.375)/2
  EXPECT_NEAR(s.mean_coverage.visible_frac, 0.5, 1e-12);       // (0.25+0.75)/2
  EXPECT_NEAR(s.mean_coverage.believed_free_frac, 0.5, 1e-12); // (0.625+0.375)/2
  EXPECT_NEAR(s.mean_coverage.phantom_frac, 0.5, 1e-12);       // (0.25+0.75)/2
  EXPECT_NEAR(s.mean_sense_flips, 5.0, 1e-12);                 // (8+2)/2

  const std::string sj = s.to_json();
  EXPECT_NE(sj.find("\"mean_coverage\":{"), std::string::npos);
  EXPECT_NE(sj.find("\"mean_sense_flips\":"), std::string::npos);
}

// Drive telemetry (Track 1d): the collector reduces a per-tick drive_sample feed into per-vehicle
// means/peaks, and aggregate_nav rolls them into corpus means over drive-carrying runs. Values are
// chosen as exact binary fractions so the arithmetic is unambiguous. (drive_step -> drive_telemetry
// is covered byte-identically by DriveTelemetry.NullTelIsByteIdenticalAndPopulates in
// nav_material_test.)
TEST(NavStats, DriveTelemetryReduction) {
  nav_stats_collector c;
  const float start[2] = {0, 0};
  const float goal[2] = {10, 0};
  c.begin_episode(1, 1.0, start, goal);
  const float head[1] = {0};
  const float spd[1] = {1};
  const std::uint8_t rch[1] = {0};
  const int mode[1] = {0};
  // per-step drive_sample: alpha 1,3,2 | beta 4,4,4 | gamma 3,1,2 | mu .5,.25,.75 | mrisk
  // .25,.75,.5 | ext 4,2,0 | steer .5,-.25,.75 | binding 1,0,1
  const drive_sample dss[3] = {
      {1, 4, 3, 0.5f, 0.25f, 4, 0.5f, 1},
      {3, 4, 1, 0.25f, 0.75f, 2, -0.25f, 0},
      {2, 4, 2, 0.75f, 0.5f, 0, 0.75f, 1},
  };
  drive_sample cur;
  nav_samplers smp;
  smp.drive = &cur;
  for (int s = 0; s < 3; ++s) {
    cur = dss[s];
    const float pos[2] = {(float)(s + 1), 0};
    c.step(pos, head, spd, mode, rch, smp);
  }
  episode_nav_stats e0 = c.finish();
  const veh_nav_stats &v = e0.per_vehicle[0];
  EXPECT_EQ(v.drive_steps, 3);
  EXPECT_NEAR(v.alpha_mean, 2.0, 1e-6); // (1+3+2)/3
  EXPECT_NEAR(v.beta_mean, 4.0, 1e-6);  // (4+4+4)/3
  EXPECT_NEAR(v.gamma_mean, 2.0, 1e-6); // (3+1+2)/3
  EXPECT_NEAR(v.mu_mean, 0.5, 1e-6);    // (.5+.25+.75)/3
  EXPECT_NEAR(v.mu_min, 0.25, 1e-6);
  EXPECT_NEAR(v.mrisk_mean, 0.5, 1e-6); // (.25+.75+.5)/3
  EXPECT_NEAR(v.mrisk_max, 0.75, 1e-6);
  EXPECT_NEAR(v.ext_force_mean, 2.0, 1e-6); // (4+2+0)/3
  EXPECT_NEAR(v.steer_abs_mean, 0.5, 1e-6); // (.5+.25+.75)/3
  EXPECT_NEAR(v.steer_abs_max, 0.75, 1e-6);
  EXPECT_EQ(v.binding_steps, 2);

  const std::string ej = e0.to_json();
  EXPECT_NE(ej.find("\"alpha_mean\":"), std::string::npos);
  EXPECT_NE(ej.find("\"mu_min\":"), std::string::npos);
  EXPECT_NE(ej.find("\"binding_steps\":2"), std::string::npos);
  EXPECT_EQ(ej.find("1e+30"), std::string::npos); // mu_min is finite here

  // corpus means over drive-carrying runs (one here).
  nav_scorecard s = aggregate_nav({e0}, "ckpt-D");
  EXPECT_NEAR(s.mean_alpha, 2.0, 1e-6);
  EXPECT_NEAR(s.mean_beta, 4.0, 1e-6);
  EXPECT_NEAR(s.mean_mu, 0.5, 1e-6);
  EXPECT_NEAR(s.mean_mrisk, 0.5, 1e-6);
  EXPECT_NEAR(s.mean_ext_force, 2.0, 1e-6);
  const std::string sj = s.to_json();
  EXPECT_NE(sj.find("\"mean_alpha\":"), std::string::npos);
  EXPECT_NE(sj.find("\"mean_mu\":"), std::string::npos);
}
