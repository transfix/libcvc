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
// The scorecard fields mirror the Python grl_snam.scorecard schema (the per-vehicle
// veh_nav_stats is a superset of grl_snam.metrics.NavStats) — reuse, not reinvent. C++
// and Python are held in step today by independent hand-computed corpora sharing the
// same literals (this repo's nav_stats_test, cvcdbg's, grl-snam's test_scorecard); a
// single shared-fixture parity gate is a tracked follow-up.
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
  double formation_tol_m = 0.0; // in-slot threshold for formation_arrived; 0 = formation stats OFF
  // A step counts as a "stall" (no progress) when it fails to beat the closest goal distance seen
  // so far by MORE than this many metres (a strict improvement of <= this threshold is still a
  // stall). Must be >= 0. This is the collector's OWN portable progress test — deliberately NOT
  // sim_world's stall_ counter, which resets on escapes/mode-transitions and lives in normalized
  // units, so it could not be mirrored field-for-field by the grl-snam Python twin. This default is
  // part of the shared cross-repo contract: grl_snam.metrics must use the SAME 0.05 to reproduce
  // stall_steps bit-for-bit; retune it only in lockstep across repos (see
  // NAV-STATS-INTRINSIC-ROADMAP).
  double stall_progress_eps_m = 0.05;
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
  // Per-agent clearance in METRES (compared against clear_safety_m); null = skip. Note
  // sim_world::min_clearance() returns NORMALIZED units, so the caller must convert —
  // clearance_m = min_clearance() / cfg.scale — before pointing this here.
  const double *min_clearance_m = nullptr;
  // Formation slot target for vehicle i in WORLD metres: return true + set (sx,sy) to the vehicle's
  // intended formation slot (the position it holds station on), or false if it has none this step.
  // null ⇒ formation stats OFF (the whole formation path degrades to zero, numbers unchanged). The
  // collector accumulates |pos_i − slot| into slot_error_mean/max and latches formation_arrived
  // when the final slot error is within params.formation_tol_m. Formation is a NAV concept, so it
  // lives in the base — a downstream RF layer references the same (veh_index, convoy_id).
  std::function<bool(int, double &, double &)> formation_slot;
  // Per-agent belief "sense flips" for THIS step: the number of cells whose occupied/free belief
  // bit flipped on the agent's most recent sense sweep (0 on ticks where the agent did not sense).
  // null ⇒ sense_flips stays 0 (feature off). The collector sums it per vehicle into
  // veh_nav_stats.sense_flips. The underlying count is already bit-identical C++/Python (the
  // sense_batch flips[] array), so the grl-snam twin reproduces the sum exactly; the caller is
  // responsible for passing the per-step DELTA (0 on non-sense ticks), never the same sweep's count
  // twice.
  const int *sense_flips = nullptr;
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
  // formation linkage: index of the vehicle this one holds station on; -1 = the objective (the
  // anchor/lead) or formation stats off. The linkage EDGE — with slot_error below it makes
  // formation holding a first-class, formation-agnostic base stat (see
  // NAV-STATS-INTRINSIC-ROADMAP).
  int formation_parent = -1;
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
  // progress toward goal (always on — computed from the goal the collector already holds).
  // closest_approach_m = the nearest the vehicle ever got to its objective. begin_episode SEEDS it
  // to straight_m (the start->goal distance), so it is finite from t=0 for any collected episode
  // and each step can only lower it (an arriver reaches ~0; a stuck vehicle's floor is how far
  // short it got). The 1e30 default is only for a hand-built / never-begun record (serialized as
  // null, aggregated out). A Python twin MUST seed to straight_m identically. stall_steps = steps
  // that failed to beat that closest distance by more than params.stall_progress_eps_m (the
  // collector's portable analogue of sim_world's non-mirrorable stall_ counter). NOTE: a step
  // improving by <= eps counts as a stall yet still lowers closest_approach_m, so stall_steps can
  // be high on a slow-but-steady closer.
  double closest_approach_m = 1e30;
  int stall_steps = 0;
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
  // formation holding (0 unless a formation_slot sampler is provided) — distance to this vehicle's
  // slot, and whether it ended in it. The continuous formation-holding measure, distinct from the
  // mission-arrival latch above (a deep-formation slot is legitimately far from the objective
  // point).
  double slot_error_mean_m = 0;
  double slot_error_max_m = 0;
  bool formation_arrived = false; // final slot error < params.formation_tol_m
  // epistemic: total belief "sense flips" over the episode (Σ per-sweep flipped-cell counts). 0
  // unless a nav_samplers.sense_flips feed is provided. A cheap proxy for how much the agent's
  // world-model churned (a high-churn agent is re-planning against a shifting belief). int64 (not
  // long) so the width is identical on LP64 and Windows/LLP32 — a corpus sum can exceed 2^31 and
  // must match the grl-snam twin's unbounded Python int bit-for-bit.
  std::int64_t sense_flips = 0;
};

// Fleet coverage fractions over an episode's belief planes — the epistemic counterpart to the
// motion stats. Each is a fraction in [0,1] of the (planes × cells) belief-raster entries. explored
// = ever seen; visible = in view on the final frame; believed_free = derived occupancy reads FREE;
// phantom = believed OCCUPIED where the truth is FREE (a hallucinated obstacle). Computed by
// compute_coverage from the same rasters both repos hold, over the binary to_occupancy() output
// (bit-identical C++/Python given matching p_thresh/band/unknown-policy), so the grl-snam twin
// reproduces every fraction.
struct nav_coverage {
  double explored_frac = 0;
  double visible_frac = 0;
  double believed_free_frac = 0;
  double phantom_frac = 0;
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
  // fleet belief coverage (all zero unless the caller fills it via compute_coverage). Episode-level
  // because the belief rasters are shared across the fleet; per-plane detail lives in the rasters
  // themselves (published for visualization), not here.
  nav_coverage coverage;
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

  // Optional per-vehicle identity, applied at finish (defaults 0). formation_parent (-1 default) is
  // the formation linkage edge (the vehicle index this one holds station on; -1 =
  // objective/anchor).
  void set_identity(int i, int convoy_id, int vehicle_class, double robot_radius_m, double mass_kg,
                    int formation_parent = -1);

private:
  nav_stats_params p_;
  budget_policy budget_;
  episode_nav_stats ep_;
  int n_ = 0;
  double dt_ = 0;
  long step_i_ = 0;
  std::vector<float> prev_pos_, prev_head_, prev_spd_, start_pos_, goal_pos_;
  std::vector<int> prev_mode_;
  std::vector<double> speed_sum_;
  std::vector<int> conv_, cls_;
  std::vector<double> rr_, mass_;
  // formation-holding accumulators (active only when nav_samplers.formation_slot is provided)
  std::vector<int> parent_; // formation linkage from set_identity
  std::vector<double> slot_err_sum_, slot_err_max_, slot_err_last_;
  std::vector<long> slot_err_cnt_;
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
  // progress (means over vehicle-runs). closest_approach = how near vehicles got to their objective
  // (small for a corpus that mostly arrives; the telling number when arrival_rate is low);
  // stall_steps = mean no-progress step count.
  // over runs with a finite closest_approach_m; 0 when none is finite (matches the mean_min_sep_m
  // convention). Every collector-produced run is finite, so 0-with-runs only arises for synthetic
  // corpora.
  double mean_closest_approach_m = 0;
  double mean_stall_steps = 0;
  // material dwell share (fraction of moving time per material id, fleet)
  std::array<double, kNumMaterials> material_time_share{};
  // formation holding (0 unless a formation_slot sampler was used). arrival = fraction of FOLLOWER
  // runs that ended in-slot; mission = fraction of episodes where every convoy's lead arrived AND
  // all its followers ended in-slot (the honest "formation arrived"); slot_error = mean follower
  // slot error.
  double form_arrival_rate = 0;
  double form_mission_rate = 0;
  double mean_slot_error_m = 0;
  // epistemic. mean_coverage divides by ALL n_episodes and mean_sense_flips by ALL vehicle-runs —
  // feature-off entries contribute 0 and STILL count in the denominator (a corpus is normally
  // all-on or all-off, so this dilutes only a pathological mixed corpus). This DELIBERATELY differs
  // from the finite-only convention used above for mean_min_sep_m / mean_closest_approach_m (which
  // have a sentinel to exclude "unmeasured"); coverage/sense_flips have no sentinel, so the rule is
  // divide-by-total. The grl-snam twin must use the same divide-by-total rule.
  nav_coverage mean_coverage;
  double mean_sense_flips = 0;

  std::string to_json() const;
};

// Reduce a corpus of per-episode records into a base scorecard row. `checkpoint`
// labels the row (the trained weights id under eval).
nav_scorecard aggregate_nav(const std::vector<episode_nav_stats> &episodes,
                            std::string checkpoint = "");

// Pure reducer: fleet belief coverage over one episode's rasters. `belief`, `everseen`, `lastvis`
// are [planes*cells] uint8 flattened PLANE-MAJOR (plane m at m*cells, C-order within a plane);
// `truth` is [cells] uint8 — ONE shared plane, BROADCAST across every belief plane (indexed by the
// in-plane cell, so a truly-free cell believed occupied on K planes counts K times). A twin MUST
// flatten idx=m*cells+c and index truth by c to match. Fractions are over (planes*cells): explored
// = everseen set, visible = lastvis set, believed_free = belief == 0, phantom = belief != 0 AND
// truth == 0 (a believed obstacle that is not really there). Any null pointer or non-positive size
// yields all-zero (feature off).
//
// SHARED CROSS-REPO CONTRACT: `belief` must be the BINARY to_occupancy() output, and believed_free
// / phantom are bit-identical to the grl-snam twin ONLY if both repos binarize with the SAME
// literals — p_thresh = 0.5, band = 0.15, unknown-cell policy = optimistic (unknown -> free). These
// are the sim_world config defaults (sim_world.h) and grl_snam BeliefGrid.to_occupancy defaults;
// treat them like stall_progress_eps_m — a shared constant, retuned only in lockstep across libcvc
// + grl_snam.
nav_coverage compute_coverage(const std::uint8_t *truth, const std::uint8_t *belief,
                              const std::uint8_t *everseen, const std::uint8_t *lastvis, int planes,
                              int cells);

} // namespace nav
} // namespace cvc
