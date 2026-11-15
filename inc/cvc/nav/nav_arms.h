/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Arms: several policies side by side, one sim_world each, on the same world from the same
// starts to the same goals with the same seed, stepped in lockstep, and what each did, tick by
// tick (docs/NAV_LAB.md).
//
//   std::vector<cvc::nav::sim_world *> worlds = {&w0, &w1};   // the host gave each its policy
//   cvc::nav::arms_run run(worlds, {{"default"}, {"trained"}});
//   for (int k = 0; k < 600; ++k)
//     run.step();                                 // every world one tick, timed, recorded
//   run.retarget(0, 120.0, -40.0);                // every arm's agent 0, on the same tick
//   ...
//   run.finish();
//   const cvc::nav::run_log &log = run.log();     // cvc/nav/run_log.h; nav_panels.h shows it
//
// What a tick records, per arm and agent (run_log's agent columns): the pose and goal, the size,
// every field of the drive telemetry (tel.<field>), the tick's books against U_nav
// (book.<name>) and the plant ledger (plant.<key>, NaN where an item does not apply); per arm
// (its arm columns): the wall time of its step (ms) and its Stats row over the current leg
// (stats.<column>).
//
// The books, per unit mass in the drive's normalized units (the vehicle pH white paper's
// "context work and projection work"):
//   dU          U's change over the tick, from where the last tick ended to where this one ends
//               (on the first tick, from where it starts);
//   drive       what the drive's motion did to U under the tick's own context:
//               (potential_end - projection_work) - potential;
//   projection  projection_work: U's change across what moved the agent after the drive;
//   context     context_work: what the sense, the field, the coefficients and the goal changed
//               between the ticks (a retarget books here);
//   dKE         the change of 1/2 v^2 (v the speed, normalized units per second);
//   residual    dU - (drive + projection + context): 0, to float round-off, unless something
//               moved the agent between the ticks.
// So dH = dKE + dU for H_nav = 1/2 v^2 + U_nav.
//
// The Stats row of an arm, over its agents and the current leg: the least clearance (m) and the
// mean of the drive telemetry's per-tick clearance (m), the ticks an agent's centre was in a
// building (summed), the path (m, the agents' mean), the elapsed time (s: to the last arrival once
// every agent is there, else the leg's ticks), the step's mean wall time (ms) and the agents
// that reached their goals. With one agent per arm, that vehicle's numbers. A retarget starts a
// leg: every arm's nav_stats is armed again at the new goals on the same tick.

#ifndef __CVC_NAV_NAV_ARMS_H__
#define __CVC_NAV_NAV_ARMS_H__

#include <cstdint>
#include <cvc/nav/drive.h>
#include <cvc/nav/nav_stats.h>
#include <cvc/nav/run_log.h>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace cvc {
class thread_pool;
namespace nav {

class sim_world;

// ── the books against U_nav ────────────────────────────────────────────────────────────────────

struct nav_books {
  double dU = 0, drive = 0, projection = 0, context = 0, dKE = 0, residual = 0;
};

// dU, drive, projection, context, dKE, residual: the books' names, and a book by its index.
std::span<const char *const> book_names();
double book_value(const nav_books &b, int i);

// One agent's books for a tick from its telemetry, last tick's potential_end (`prev_end` null on
// the first tick) and its speed before and after the tick (normalized units per second).
nav_books books_of(const drive_telemetry &t, const double *prev_end, double speed_n,
                   double prev_speed_n);

// ── the drive telemetry's fields by name ───────────────────────────────────────────────────────

// Each field of drive_telemetry, named as pycvc's nav_sim_world_drive_telemetry names it (alpha,
// beta, gamma, lam_soft, lam_hard, mu, mrisk, v_risk, ext_fx, ext_fy, steer, curvature, clearance,
// binding, potential, u_goal, u_barrier, u_soft, u_hard, u_terrain, force_x, force_y,
// potential_end, projection_work, context_work).
struct telemetry_field {
  const char *name;
  double (*get)(const drive_telemetry &);
};
std::span<const telemetry_field> telemetry_fields();

// ── the Stats row ──────────────────────────────────────────────────────────────────────────────

// The Stats columns (stats.<name> in a run log): clear_min_m, clear_mean_m, pen_steps, path_m,
// elapsed_s, ms_step, reached.
std::span<const char *const> stats_columns();

struct arm_stats {
  double clear_min_m = 0, clear_mean_m = 0; // NaN before a clearance is measured
  double pen_steps = 0;
  double path_m = 0;
  double elapsed_s = 0;
  double ms_step = 0;        // NaN before a tick
  double reached = 0;        // agents at their goals
  double value(int i) const; // by stats_columns()' index
};

// An arm's Stats row from its nav_stats record over the leg, the clearances its drive telemetry
// reported over the leg (m; their least and their sum and count), and its step's wall times over
// the leg (their sum and count).
arm_stats stats_of(const episode_nav_stats &e, double clear_min_m, double clear_sum_m, long clear_n,
                   double ms_sum, long ms_n);

// ── arms ───────────────────────────────────────────────────────────────────────────────────────

struct arm {
  std::string name;
  std::string uri;         // its policy's file, for the log ("" for the built-in default)
  std::string sha256;      // that file's SHA-256 ("" to leave it unstated)
  std::string meta = "{}"; // JSON object: the host's own, kept in the log
};

struct arms_options {
  unsigned seed = 0;
  std::string scene; // the host's name for the world (nav_stats' scene_id, the log's)
  std::string plant; // what the arms drive, for the log ("" = "kinematic", or the world's plant)
  nav_stats_params stats;
  std::string world = "{}";    // JSON object: what rebuilds the world, kept in the log
  std::string settings = "{}"; // JSON object: the settings, kept in the log
  std::string meta = "{}";     // JSON object: the host's own, kept in the log
  std::string log_path;        // stream the run log to this file as it runs ("" = memory only)
};

class arms_run {
public:
  // One world per arm, borrowed (they outlive the run), each with its policy as the host gave it
  // (sim_world::set_policy). Every world has the same agents, tick, frame and centre, and stands
  // at its start: the run turns on each one's drive telemetry, arms its nav_stats and takes its
  // agents' poses and goals as the start and leg 0. Throws std::invalid_argument for no arm, a
  // world for each arm missing, an arm's name empty or repeated, or worlds that differ.
  arms_run(std::vector<sim_world *> worlds, std::vector<arm> arms, arms_options o = {});
  // The same, owning the worlds.
  arms_run(std::vector<std::unique_ptr<sim_world>> worlds, std::vector<arm> arms,
           arms_options o = {});
  ~arms_run();
  arms_run(const arms_run &) = delete;
  arms_run &operator=(const arms_run &) = delete;

  int arms() const { return static_cast<int>(worlds_.size()); }
  int agents() const;
  long tick() const { return tick_; } // ticks run
  double dt() const;
  sim_world &world(int a) { return *worlds_.at(static_cast<std::size_t>(a)); }
  const sim_world &world(int a) const { return *worlds_.at(static_cast<std::size_t>(a)); }

  // One tick of every world, each first in turn (so no arm's wall time always pays for going
  // first), timed and recorded. `pool` and `num_threads` go to sim_world::step.
  void step(int num_threads = 0);
  // For a host that steps the worlds itself: after it stepped world `a` (taking `ms` of wall
  // time), record it; after every arm is recorded, end_tick() closes the tick. Throws
  // std::logic_error for an arm recorded twice in a tick, or end_tick before every arm was.
  void record(int a, double ms);
  void end_tick();

  // Every arm's agents' goals (world metres, [agents*2]) from the next tick on, on the same tick
  // in every arm: the leg ends, and every arm's nav_stats is armed again at the new goals.
  void retarget(const float *goals_world, int n);
  // Agent i's goal in every arm (the others keep theirs), as retarget does.
  void retarget(int i, double gx, double gy);
  // Agent sizes (m, [agents] each) of arm a from the next tick on, for a host that knows its
  // vehicles (a plant's); by default each world's vehicle dims, else its footprint disc.
  void set_size(int a, const float *length, const float *width, const float *wheelbase);

  // The run log so far, live (each tick appended as it is recorded).
  const run_log &log() const { return *log_; }
  std::shared_ptr<const run_log> shared_log() const { return log_; }
  // Write the end line (each arm's nav_stats record) and close the streamed file. Idempotent; the
  // run cannot step after it.
  void finish();
  bool finished() const { return finished_; }

private:
  void init(std::vector<arm> arms, arms_options o);
  void begin_stats();
  void stream(const std::string &line);

  std::vector<std::unique_ptr<sim_world>> owned_;
  std::vector<sim_world *> worlds_;
  arms_options opt_;
  std::shared_ptr<run_log> log_;
  std::unique_ptr<run_log_writer> writer_;
  long tick_ = 0;
  bool finished_ = false;
  // per arm: recorded this tick, its values this tick, its leg's accumulators
  struct arm_state {
    bool recorded = false;
    std::vector<double> agent_values;            // [agents][agent columns]
    std::vector<double> arm_values;              // [arm columns]
    std::vector<double> prev_end;                // [agents] last tick's potential_end
    std::vector<double> prev_speed;              // [agents]
    std::vector<float> length, width, wheelbase; // [agents], m; empty = the world's
    double clear_min = 0, clear_sum = 0, ms_sum = 0;
    long clear_n = 0, ms_n = 0;
    bool has_prev = false;
  };
  std::vector<arm_state> st_;
};

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_NAV_ARMS_H__
