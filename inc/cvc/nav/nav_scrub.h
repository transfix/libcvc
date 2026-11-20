/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The checkpoint scrubber: watch a policy learn. Every checkpoint a training history names is
// deployed on one scenario, headless, and a view shows checkpoint k beside step 0
// (docs/NAV_LAB.md, "The scrubber"; `cvc nav scrub HIST` in cvc-cli).
//
//   cvc::nav::training_history h = cvc::nav::read_training_history("run/history.jsonl");
//   cvc::nav::scrub_scenario s = cvc::nav::default_scenario(4, 0, 600);
//   std::vector<cvc::nav::scrub_entry> runs = cvc::nav::scrub(h, s);   // cached beside it
//   for (const cvc::nav::scrub_entry &e : runs)
//     ... e.checkpoint.step, e.metrics.path_m, *e.log (a run log, cvc/nav/run_log.h) ...
//
// A history is JSON lines: cvc::nav's own coef_trainer writes one (cvc.nav.coef_train_history/v1,
// train_config::history_path), and so do GRL-SNAM's trainers. A record's `kind` is its series
// (header, step, checkpoint, end, ...); a checkpoint record names the policy it saved, `blob` (or
// `checkpoint`), a .cvcnav the scrubber deploys (a relative path is the history's directory's).
// A .pt needs torch and is skipped. A file may hold several runs, each from its header.
//
// The scenario is a world an input trace's header describes (cvc/nav/sim_world_trace.h: a
// complete cvc.nav.sim_world/v1 world, its maps embedded), its agents, and the ticks to run: the
// built-in one (default_scenario), or a recorded trace's (scenario_from_trace). Each checkpoint's
// blob replaces the world's own policy for every agent, and the run is an arms_run of one arm, so
// checkpoint 0 deployed here is the untrained arm deployed anywhere else. A run is cached at
// `<history>.scrub/<blob sha256, 16 hex>-<scenario hash>.cvcrun`. The scenario hash covers the
// world, the agents, the settings, the plant, the ticks, the scrubber's version and libcvc's, so
// any of them changing deploys the checkpoints again.
//
// The plant: nav's kinematic bicycle has no collision response, so its agents can drive through a
// building nav did not keep them off. A scenario's `plant` names something else to drive, which a
// host's scrub_options::plant hook puts the agents on. cvc::physics::scrub_plant
// (cvc/physics/scrub_plant.h) is the usual hook: Jolt vehicles ("humvee", ...) that collide with
// the world's walls. nav itself does not depend on physics, so it has no default hook.

#ifndef __CVC_NAV_NAV_SCRUB_H__
#define __CVC_NAV_NAV_SCRUB_H__

#include <cvc/core/input_trace.h>
#include <cvc/nav/nav_panels.h>
#include <cvc/nav/run_log.h>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace cvc {
class app;
namespace nav {

class policy_library;
class sim_world;

inline constexpr const char *kScrubReportSchema = "cvc.nav.scrub/v1";
// Bumped when the scrubber's runs would change, so older caches are not read.
inline constexpr int kScrubVersion = 1;
inline constexpr const char *kScrubKinematic = "kinematic"; // nav's bicycle, no hook needed

// ── training histories ─────────────────────────────────────────────────────────────────────────

struct training_checkpoint {
  double step = 0;           // its record's step (else its epoch)
  std::string axis = "step"; // "step" or "epoch"
  std::string blob;          // the policy it saved, as the record names it
  std::string path;          // that file, found ("" when it is not there)
  bool best = false;         // the record says it was the best so far
  bool selected = false;     // the run's pick (the end record's selected_step, or "selected")
  double fitness = 0;        // NaN without one
};

struct training_series {
  std::string name; // <kind>.<dotted path>: step.loss, checkpoint.fitness, step.grad_norm.mean
  std::vector<double> x, y;
};

struct training_history {
  std::string path;
  std::string schema;                           // the header's, "" without one
  int run = 0;                                  // which run of the file (0, 1, ...)
  int runs = 0;                                 // how many it holds
  std::vector<training_checkpoint> checkpoints; // in order, those naming a .cvcnav that is there
  std::vector<std::string> missing;             // blobs named that are not there
  std::vector<std::string> skipped;             // blobs of another kind (a .pt needs torch)
  std::vector<training_series> series;
  // A series by name, or null.
  const training_series *find(const std::string &name) const;
};

// Run `run` of the history at `path` (-1: the last). Throws std::runtime_error for a file that
// can't be read, a line that is not JSON, or a run it does not hold.
training_history read_training_history(const std::string &path, int run = -1);
// The file a blob names: as given when absolute, else beside the history, else from the working
// directory, else its file name beside the history; "" when none exists.
std::string resolve_blob(const std::string &history, const std::string &blob);

// ── the scenario ───────────────────────────────────────────────────────────────────────────────

struct scrub_scenario {
  input_trace_header world; // a complete cvc.nav.sim_world/v1 world (sim_world_trace.h)
  long ticks = 600;
  std::string name; // what it is, for the views ("blocks, 4 agents, seed 0")
  // What the agents drive: "kinematic" (or "", nav's bicycle) or a name the deploy's
  // scrub_options::plant hook knows (cvc::physics::scrub_plant's: "humvee", "roboracer", ...).
  std::string plant = kScrubKinematic;
};

// The built-in scenario: a walled 120 m square with a 3 x 3 grid of buildings (each nudged and
// sized by `seed`) and streets between them, `agents` agents from starts on its west side to goals
// on its east (their lanes from `seed`), at 0.05 normalized units per metre and 0.05 s a tick, the
// map known (no sensing), run for `ticks`, on nav's kinematic bicycle (set `plant` for another).
scrub_scenario default_scenario(int agents = 4, unsigned seed = 0, long ticks = 600);
// A recorded input trace's world and agents (its header; its inputs are not replayed). Throws
// std::runtime_error for a trace whose world v1 cannot rebuild.
scrub_scenario scenario_from_trace(const std::string &trace_path, long ticks = 600);
// A short, stable hash of the scenario (16 hex digits): its world, agents and settings, its plant,
// its ticks, kScrubVersion and libcvc's version.
std::string scenario_hash(const scrub_scenario &s);
// `<history>.scrub/<blob sha 16>-<scenario hash>.cvcrun`
std::string scrub_cache_path(const std::string &history, const std::string &blob_sha256,
                             const std::string &scenario_hash);

// ── a deployed checkpoint ──────────────────────────────────────────────────────────────────────

// The deployed metrics, at a run's last tick (arm `a`): the share of agents at their goals, the
// time (s), the path (m), the ticks penetrating, the least and the mean clearance (m), and the
// mean distance left to the goals (m).
struct scrub_metrics {
  double reached = 0, time_s = 0, path_m = 0, pen_steps = 0, clear_min_m = 0, clear_mean_m = 0,
         goal_m = 0;
  double value(int i) const; // by scrub_metric_info's order
};
struct scrub_metric_info {
  const char *name;  // reached, time_s, path_m, pen_steps, clear_min_m, clear_mean_m, goal_m
  const char *label; // "reached the goal" ...
  int digits;
};
std::span<const scrub_metric_info> scrub_metric_names();
scrub_metrics metrics_of(const run_log &log, int a = 0);

// What a scrub_plant_fn put the agents on: what they drive (the plant's name, or "kinematic" after
// a fallback, in a build without the plant), and an owner of the plant (and its world) the deploy
// keeps until its run ends.
struct scrub_plant_attached {
  std::string driving;
  std::shared_ptr<void> owner;
};
// Puts a deployed world's agents on `plant`, at their poses, before its first tick. Throws for a
// plant it does not know.
using scrub_plant_fn = std::function<scrub_plant_attached(sim_world &, const std::string &plant)>;

// One blob deployed on the scenario: its run log, of one arm named `name` (its meta `meta`, a JSON
// object's text), on the scenario's plant (`plant` attaches it; the log's plant is what the agents
// drove). Throws std::runtime_error for a policy that can't be loaded, and std::invalid_argument
// for a plant other than kinematic with no hook.
run_log deploy_checkpoint(const scrub_scenario &s, const std::string &blob, const std::string &name,
                          policy_library &lib, const std::string &meta = "{}",
                          const scrub_plant_fn &plant = {});

struct scrub_entry {
  training_checkpoint checkpoint;
  std::string sha256;  // the blob's
  std::string cache;   // its run log's path
  bool cached = false; // read from the cache, not deployed now
  scrub_metrics metrics;
  std::shared_ptr<const run_log> log;
};

struct scrub_options {
  bool use_cache = true;                        // read and write the cache
  std::function<void(const std::string &)> say; // a line per checkpoint (null: quiet)
  scrub_plant_fn plant; // puts the agents on the scenario's plant (null: kinematic only)
};

// Every checkpoint of `h` on `s`: from the cache, else deployed (and cached).
std::vector<scrub_entry> scrub(const training_history &h, const scrub_scenario &s,
                               const scrub_options &o = {});
// Every checkpoint's metrics as a cvc.nav.scrub/v1 report (the scenario's plant in it).
std::string scrub_report_json(const training_history &h, const scrub_scenario &s,
                              const std::vector<scrub_entry> &entries);
// The checkpoint table (scrub.table: at, step, flag, reached, time_s, path_m, pen_steps,
// clear_min_m, goal_m) with checkpoint k marked, and a metric against the step with step 0's as a
// dashed line (scrub.metric) and its text.
panel_table scrub_table(const std::vector<scrub_entry> &entries, int k);
chart_plot metric_chart(const std::vector<scrub_entry> &entries, const std::string &metric,
                        std::string *text = nullptr);
// A history's series as a chart (scrub.curve): the checkpoints' steps marked, the pick as best.
chart_plot curve_chart(const training_history &h, const std::vector<scrub_entry> &entries,
                       const std::string &series, bool *log_y = nullptr,
                       std::string *text = nullptr);

// ── the view ───────────────────────────────────────────────────────────────────────────────────

// The same checkpoints deployed on one plant: scrub()'s entries for a scenario whose plant is
// `plant`.
struct scrub_plant_runs {
  std::string plant;
  std::vector<scrub_entry> entries;
};

// Checkpoint k beside step 0: the controls that pick k (scrub.choice, a combo over scrub.labels;
// scrub.pos, a slider 0..1; prev() and next(); a chart's click, scrub.pick, through pick_step()),
// the metric chart and the table (scrub.metric_name picks the metric), the training curve
// (scrub.curve_name, over scrub.curve_names), and the two runs over the episode: a replay_view of
// step 0's log stacked with k's, whose tables and poses the frame carries. With the checkpoints
// deployed on several plants, scrub.plant_choice (a combo over scrub.plant_labels;
// scrub.plant_count of them) or use_plant() shows the same checkpoint on another plant, at the same
// play head; scrub.plant is the plant shown.
class scrub_view {
public:
  // k < 0: the run's pick, else its last checkpoint. `scenario` names the scenario
  // (scrub.scenario). Throws std::invalid_argument with no entries.
  scrub_view(training_history h, std::vector<scrub_entry> entries, int k = -1,
             std::string scenario = "");
  // The checkpoints on each plant, the first shown first. Throws std::invalid_argument with no
  // plant, a plant without entries, a plant named twice, or plants whose checkpoints differ.
  scrub_view(training_history h, std::vector<scrub_plant_runs> plants, int k = -1,
             std::string scenario = "");
  struct frame {
    int k = 0;
    replay_view::frame replay;
  };
  frame update(cvc::app &app, const std::string &prefix, double dt, bool charts = true);
  void pick(int k);
  void prev() { pick(k_ - 1); }
  void next() { pick(k_ + 1); }
  void pick_step(cvc::app &app, const std::string &prefix);
  int checkpoint() const { return k_; }
  // Show plant p's runs (clamped), at the same checkpoint.
  void use_plant(int p);
  int plant() const { return p_; }
  const std::vector<scrub_plant_runs> &plants() const { return plants_; }
  // The plant shown's entries.
  const std::vector<scrub_entry> &entries() const {
    return plants_[static_cast<std::size_t>(p_)].entries;
  }
  replay_view &replay() { return *replay_; }

private:
  void show(cvc::app &app, const std::string &prefix, bool charts);
  training_history h_;
  std::vector<scrub_plant_runs> plants_;
  std::vector<std::string> labels_, plant_labels_;
  int k_ = 0, shown_ = -1, p_ = 0;
  std::unique_ptr<replay_view> replay_;
  bool wrote_ = false;
  double pos_ = 0.0;
  std::string scenario_, choice_, plant_choice_, metric_, curve_;
};

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_NAV_SCRUB_H__
