/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// What the arms' windows show (the shipped components nav_lab_stats.ari and nav_lab_ledger.ari),
// from a run log (cvc/nav/run_log.h) at a tick: a live run's log at its last tick, a replay's at
// the play head, so a replay shows at a tick what the live run showed there, cell for cell. Every
// host writes them through these functions (pycvc_gl.nav_lab, a C++ Ariadne host, the browser's),
// as pycvc's Host.set_table and set_plot lay them out in the state tree:
//
//   a table at `key`:  key = the row count, key.<i>.<column> = a cell's text, key_columns.<j>
//   a chart at `key`:  key.s.<i>.{label, x, y, style} (x and y comma-separated), key.m.<j>.{x,
//                      label, kind}, for a chart: widget with bind: key
//
// The keys (under the host's prefix):
//   arms.stats     a row per arm: arm, colour, clear_min, clear_mean, pen, path, elapsed, ms,
//   reached ledger.coef    a row per arm: arm, alpha, beta, gamma, lam_soft, lam_hard, mu
//   ledger.terms   a row per arm: arm, potential, u_goal, u_barrier, u_soft, u_hard, u_terrain
//   ledger.books   a row per arm: arm, dU, drive, projection, context, dKE, residual (summed)
//   ledger.tick    the same, this tick's
//   ledger.plant   the chosen arm's plant ledger: key, value ("n/a" where it does not apply), unit
//   ledger.plant_note
//   arms.clearance, ledger.cum, ledger.compare   the charts (every tick, a marker at each leg)
// The Ledger's rows and the plant ledger are for one agent of each arm (ledger.vehicle, 0..);
// with one agent per arm, the arm's vehicle.

#ifndef __CVC_NAV_NAV_PANELS_H__
#define __CVC_NAV_NAV_PANELS_H__

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
class app;
namespace nav {

class run_log;

// A number as a cell shows it: `digits` after the point, "-" when it is not finite.
std::string panel_number(double v, int digits);
// Ariadne chart:'s colour slot c<slot>'s name (blue, orange, green, amber, pink, dark green,
// purple, red), cycling past 8.
const char *slot_colour(int slot);

struct panel_table {
  std::string key;
  std::vector<std::string> columns;
  std::vector<std::vector<std::string>> rows; // a cell per column
  // The cell at row r, column `name` ("" for none).
  std::string cell(std::size_t r, const std::string &name) const;
};

struct chart_series {
  std::string label, style;
  std::vector<double> x, y; // NaN is a gap
};

struct chart_marker {
  double x = 0;
  std::string label;
  std::string kind = "note"; // best, elbow, plateau or note
};

struct chart_plot {
  std::string key;
  std::vector<chart_series> series;
  std::vector<chart_marker> markers;
};

// The Stats and Ledger tables at tick k (arms.stats, ledger.coef, ledger.terms, ledger.books,
// ledger.tick), the Ledger's for agent `agent` of each arm. Before the first tick (k < 0, or an
// empty log) every number is "-".
std::vector<panel_table> arms_tables(const run_log &log, long k, int agent);
// Arm a's agent's plant ledger at tick k (ledger.plant), and its note.
panel_table plant_table(const run_log &log, long k, int a, int agent);
std::string plant_note(const run_log &log, long k, int a, int agent);
// The charts over the whole run: every arm's clearance (m) for `agent`, arm a's agent's books
// summed over the ticks, and `book`'s for every arm; a marker where each leg after the first
// starts.
std::vector<chart_plot> arms_charts(const run_log &log, int a, int agent, const std::string &book);

// ── the state tree ─────────────────────────────────────────────────────────────────────────────

// `prefix`.`key`, or `key` with no prefix.
std::string panel_path(const std::string &prefix, const std::string &key);
// Read a key's text ("" or `dflt` when it is not there), and write one.
std::string panel_get(cvc::app &app, const std::string &path, const std::string &dflt = "");
void panel_set(cvc::app &app, const std::string &path, const std::string &value);
// A table or chart at its key under `prefix`. A chart's series longer than `max_points` keep their
// ends and each bucket's lowest and highest point and first gap, as Host.set_plot does.
void write_table(cvc::app &app, const std::string &prefix, const panel_table &t);
void write_chart(cvc::app &app, const std::string &prefix, const chart_plot &c,
                 int max_points = 2000);

// Which arm and agent the Ledger, the plant ledger and the charts show, read from the document:
// ledger.agent (an arm's name; the first when it names none), ledger.vehicle (an agent index),
// ledger.book (a book's name; dU by default). The arms' names go to arms.names (one a line) and
// the agents' count to arms.agents, for the combos.
struct panel_choice {
  int arm = 0, agent = 0;
  std::string book = "dU";
  bool operator==(const panel_choice &o) const {
    return arm == o.arm && agent == o.agent && book == o.book;
  }
};
panel_choice read_choice(cvc::app &app, const std::string &prefix, const run_log &log);
// The index of the arm `name` names in the log, or `dflt`.
int arm_index(const run_log &log, const std::string &name, int dflt = 0);

// Every table, the plant ledger and its note at tick k, for `c` (and arms.names, arms.agents).
void publish_tables(cvc::app &app, const std::string &prefix, const run_log &log, long k,
                    const panel_choice &c);
void publish_charts(cvc::app &app, const std::string &prefix, const run_log &log,
                    const panel_choice &c);

// ── the play head ──────────────────────────────────────────────────────────────────────────────

// The play head over a run of `duration` s with ticks of `dt` s, driven by the document's
// transport (the shipped sim_transport component): sim.paused and sim.speed; step() moves it a
// tick on and restart() back to the start (a host calls them on sim.step and sim.restart);
// play.pos (0..1) follows it, and moves it when dragged; play.loop wraps it at the end, else it
// stops there; seek() puts it at play.seek (a chart's click). update() writes play.pos, play.t
// (s, the charts' cursor) and play.time each frame, and returns the time.
class play_head {
public:
  play_head(double duration, double dt, double at = 0.0);
  void step();
  void restart(cvc::app &app, const std::string &prefix);
  void seek(cvc::app &app, const std::string &prefix);
  void set(double t);
  double update(cvc::app &app, const std::string &prefix, double frame_dt);
  double time() const { return t_; }
  double duration() const { return duration_; }

private:
  double clamp(double t) const;
  double duration_, dt_, t_;
  bool wrote_ = false;
  double pos_ = 0.0; // the position this play head last wrote
};

// A pose to draw an arm's agent at: rear axle (m), heading, and its size (m).
struct ghost_pose {
  double x = 0, y = 0, heading = 0;
  double length = 0, width = 0, wheelbase = 0;
};
// Every arm's agents' poses at time t ([arms][agents]), between the ticks either side of it.
std::vector<std::vector<ghost_pose>> poses_at(const run_log &log, double t);

// A recorded run played back: each frame, the play head moves, the tables show the tick at the
// play head and the charts the whole run (rewritten only when the choice changes). The host draws
// the ghosts from frame::poses and wires sim.step, sim.restart and the charts' seek to head().
class replay_view {
public:
  explicit replay_view(std::shared_ptr<const run_log> log, double at = 0.0);
  struct frame {
    long k = 0; // the tick showing
    double t = 0;
    int leg = 0;
    std::vector<std::vector<ghost_pose>> poses;
    panel_choice choice;
  };
  frame update(cvc::app &app, const std::string &prefix, double dt, bool charts = true);
  play_head &head() { return head_; }
  const run_log &log() const { return *log_; }
  // Forget the charts written, so the next update writes them again.
  void redraw() { charted_ = false; }

private:
  std::shared_ptr<const run_log> log_;
  play_head head_;
  bool charted_ = false;
  panel_choice shown_;
};

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_NAV_PANELS_H__
