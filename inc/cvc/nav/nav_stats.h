/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick.

  VolMagick is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  VolMagick is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// nav_stats.h — the BASE navigation-statistics layer: per-vehicle + per-episode
// telemetry over a sim_world drive, plus a corpus scorecard for training fitness.
//
// This is the RF-FREE half of the two-layer nav-stats design (CVC-DBG/cvcdbg
// docs/nav-stats-design.md). It lives in cvc::nav — below cvc::dbg — so the
// non-DBG grl-snam swarm, any cvc::nav-only demo, AND the cvcdbg convoy demos all
// collect the SAME base row (motion, clearance, collisions/penetration, material
// dwell, time, fuel, budgets). The RF/comms-force extension (exposure, PDR/SINR,
// band/retunes) is a SEPARATE cvc::dbg record joined by veh_index; it never lives
// here. The collector consumes only the sim_world::snapshot arrays plus optional
// position samplers, so it has no libcvc-internal dependency beyond the stdlib.
// Field names mirror the Python grl_snam.metrics NavStats schema — reuse, not
// reinvent — and the two are pinned in cross-language parity tests.
#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace cvc {
namespace nav {

// The dense material palette id space (the 13-class material vocabulary). Buckets
// stay zero until a material raster is plumbed into the sim.
inline constexpr int kNumMaterials = 13;

// Tunables for the base accumulators. Defaults are sensible for city-scale drives.
struct nav_stats_params {
  double speed_eps_mps = 0.10;  // below this a vehicle counts as stopped
  double turn_event_rad = 0.20; // per-step |Δheading| above this is one "turn event"
  double clear_safety_m = 2.00; // dwell below this clearance feeds time_below_clear_s
  double min_gap_m = 0.0;       // pairwise distance below this is a vehicle contact (0 = off)
  double reach_eps = 0.5;       // treat reached[i] as a level flag; latch on its rising edge
};

// Per-episode budget. A vehicle is over_budget (and the episode fails) when EITHER
// active bound is breached. Both policies combine; a 0 field disables that bound.
struct budget_policy {
  double time_budget_s = 0; // fixed sim-seconds budget (fleet-wide)
  double eta_multiple = 0;  // per-vehicle: budget = eta_multiple · (straight_m / speed_ref)
  double speed_ref_mps = 0; // reference cruise speed for the per-vehicle ETA estimate
  double fuel_budget = 0;   // fixed fuel/effort budget in fuel_used units
};

// Optional per-position samplers; any left null degrades that stat gracefully to
// zero. material_id is abstracted so the source can be swapped with no schema change.
struct nav_samplers {
  std::function<int(double, double)> material_id; // -> 0..kNumMaterials-1, or -1 unknown
  std::function<bool(double, double)> occupied;   // truth-occupancy cell blocked?
  const double *min_clearance_m = nullptr; // per-agent clearance (from sim_world); null = skip
};

// One per vehicle, accumulated over an episode. Mirrors grl_snam NavStats + the
// per-vehicle heterogeneity of the runtime veh_params columns.
struct veh_nav_stats {
  // identity / heterogeneity — equal across the fleet today, will diverge.
  int veh_index = 0;
  int convoy_id = 0;
  int vehicle_class = 0;
  double robot_radius_m = 0;
  double mass_kg = 0;
  // outcome
  bool arrived = false;
  double time_to_goal_s = -1; // first step reached rose; <0 = never
  bool timed_out = false;
  bool over_budget = false;
  int goals_reached = 0;
  // motion
  double total_path_m = 0;
  double straight_m = 0;
  double turn_total_rad = 0;
  int turn_events = 0;
  double time_in_wall_s = 0; // drive mode == 1 (wall-follow / avoidance) dwell
  int wall_entries = 0;
  double time_moving_s = 0;
  double time_stopped_s = 0;
  double speed_mean = 0;
  double speed_max = 0;
  // clearance / collisions
  double min_clearance_m = 1e30;
  double time_below_clear_s = 0;
  int penetration_steps = 0; // frames the vehicle center sits in an occupied cell
  int veh_contacts = 0;      // frames within min_gap of another vehicle
  // material (needs a raster plumbed)
  std::array<double, kNumMaterials> time_over_material_s{};
  std::array<double, kNumMaterials> dist_over_material_m{};
  // effort / fuel
  double accel_integral = 0; // Σ|Δspeed| raw effort
  double fuel_used = 0;      // the accel_integral proxy until a real f(accel, mass, material) model
};

// One per episode; reduces the per-vehicle vector + carries campaign identity.
struct episode_nav_stats {
  std::string scene_id;
  unsigned seed = 0;
  std::string checkpoint;
  int n_vehicles = 0;
  long ticks = 0;
  double dt_s = 0;
  int arrived = 0;
  bool success = false;
  double makespan_s = 0;
  double mean_ttg_s = 0;
  double penetration_pct = 0;
  int total_veh_contacts = 0;
  double min_sep_m = 1e30;
  double mean_path_ratio = 0;
  double mean_turn_total_rad = 0;
  double total_fuel = 0;
  std::vector<veh_nav_stats> per_vehicle;

  std::string to_json() const; // the base record (a DBG consumer nests an "rf" member itself)
};

// Collector: begin_episode -> step (once per sim frame) -> finish. Consumes the
// sim_world::snapshot arrays, so grl-snam and the cvcdbg demos/harness drive it
// identically.
class nav_stats_collector {
public:
  explicit nav_stats_collector(nav_stats_params p = {}) : p_(p) {}

  void begin_episode(int n, double dt_s, const float *start_pos, const float *goal_pos,
                     const budget_policy &budget = {}, std::string scene_id = "", unsigned seed = 0,
                     std::string checkpoint = "");

  // pos[2N] head[N] spd[N] mode[N] reached[N] — the sim_world::snapshot arrays.
  void step(const float *pos, const float *head, const float *spd, const int *mode,
            const std::uint8_t *reached, const nav_samplers &smp = {});

  episode_nav_stats finish();

  // Optional per-vehicle identity, applied at finish (defaults 0).
  void set_identity(int i, int convoy_id, int vehicle_class, double robot_radius_m, double mass_kg);

private:
  nav_stats_params p_;
  budget_policy budget_;
  episode_nav_stats ep_;
  int n_ = 0;
  double dt_ = 0;
  long step_i_ = 0;
  std::vector<float> prev_pos_, prev_head_, prev_spd_, start_pos_;
  std::vector<int> prev_mode_;
  std::vector<double> speed_sum_;
  std::vector<int> conv_, cls_;
  std::vector<double> rr_, mass_;
};

// ── Scorecard: aggregate a CORPUS of episodes for training-fitness tracking ──
//
// The base nav-fitness row grl-snam uses to rank its OWN base-policy checkpoints
// over a scene corpus (RF-free). The DBG campaign layers an rf_scorecard on top.
struct nav_scorecard {
  std::string checkpoint;
  int n_episodes = 0;
  int n_vehicle_runs = 0;
  // outcome
  double success_rate = 0;        // fraction of episodes with episode.success
  double arrival_rate = 0;        // fraction of vehicle-runs that arrived
  double mean_time_to_goal_s = 0; // over arrivers
  double p95_time_to_goal_s = 0;  // over arrivers
  double mean_makespan_s = 0;
  // economy (means over vehicle-runs)
  double mean_path_ratio = 0;
  double mean_turn_total_rad = 0;
  double mean_fuel = 0;
  // safety
  double mean_penetration_pct = 0; // mean over episodes
  double veh_contacts_per_run = 0; // total veh_contacts / vehicle-runs
  double mean_min_sep_m = 0;       // mean over episodes (finite only)
  // material dwell share (fraction of moving time per material id, fleet)
  std::array<double, kNumMaterials> material_time_share{};

  std::string to_json() const;
};

// Reduce a corpus of per-episode records into a base scorecard row. `checkpoint`
// labels the row (the trained weights id under eval).
nav_scorecard aggregate_nav(const std::vector<episode_nav_stats> &episodes,
                            std::string checkpoint = "");

} // namespace nav
} // namespace cvc
