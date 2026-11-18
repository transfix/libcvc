/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::nav's run log, cvc.nav.run_log/v1: what a run of one or more arms did, tick by tick (an
// arms_run writes it, cvc/nav/nav_arms.h), to replay and to analyse. Where an input trace
// (cvc/core/input_trace.h, docs/INPUT_TRACE.md) records a run's inputs so it can be run again, a
// run log records its outcomes: every arm's poses, drive telemetry, books, Stats and plant
// ledger, so a viewer shows them as they were without running anything.
//
// JSON lines (docs/NAV_LAB.md has the format):
//
//   {"schema":"cvc.nav.run_log/v1","libcvc":"3.7.15","dt":0.05,...,"columns":{...}}
//   {"leg":{"index":0,"tick":0,"goals":[[x,y],...]}}
//   {"tick":1,"t":0.05,"agent":[[[...],...],...],"arm":[[...],...]}
//   ...
//   {"leg_end":{"index":0,"tick":240,"nav_stats":[{...},...]}}
//   {"leg":{"index":1,"tick":240,"goals":[[x,y],...]}}
//   ...
//   {"end":{"ticks":600,"nav_stats":[{...},...]}}
//
// A tick's "agent" holds, per arm, per agent, a value per agent column; its "arm" a value per arm
// column. The header names the columns, so a reader finds a column by name and a later writer may
// add columns. A number is written in its shortest round-trip form, so it reads back as the same
// double; a value that is not applicable (a plant ledger item on the kinematic plant) or not
// measured is null, and reads back as NaN. A log whose writer stopped early (no end line, or a
// last line cut short) still reads, up to its last whole tick.

#ifndef __CVC_NAV_RUN_LOG_H__
#define __CVC_NAV_RUN_LOG_H__

#include <cstdint>
#include <iosfwd>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace cvc {
namespace nav {

inline constexpr const char *kRunLogSchema = "cvc.nav.run_log/v1";

// One arm as the log names it.
struct run_log_arm {
  std::string name;
  int slot = 0;            // its colour slot (Ariadne chart: c0..c7), the same everywhere
  std::string key;         // its policy's key in its world (sim_world::policy_key)
  std::string uri;         // its policy's file ("" for the built-in default)
  std::string sha256;      // that file's SHA-256 ("" for the default)
  std::string meta = "{}"; // JSON object: the host's own (a checkpoint's step, say)
};

struct run_log_header {
  std::string schema = kRunLogSchema;
  std::string libcvc;        // the version that wrote it
  double dt = 0.0;           // the tick (s)
  double scale = 1.0;        // the worlds' frame: normalized units per metre
  double cx = 0.0, cy = 0.0; // the worlds' centre (m)
  std::uint64_t seed = 0;
  std::string scene; // the host's name for the world
  std::string plant; // what the arms drive ("kinematic", "humvee", ...)
  int agents = 0;    // agents per arm
  std::vector<run_log_arm> arms;
  std::vector<double> start;              // [agents*3]: x, y (m), heading (rad) at the start
  std::string world = "{}";               // JSON object: what rebuilds the world (the host's keys)
  std::string settings = "{}";            // JSON object: the settings the run used
  std::string meta = "{}";                // JSON object: the host's own
  std::vector<std::string> agent_columns; // per arm, per agent, per tick
  std::vector<std::string> arm_columns;   // per arm, per tick

  std::string to_json() const; // the header line (no newline)
  static run_log_header from_json(const std::string &json);
};

// A leg: from tick index `tick` (the ticks recorded before it), every arm's agents to `goals`.
struct run_log_leg {
  long tick = 0;
  std::vector<double> goals; // [agents*2] (m)
  // Each arm's nav_stats record (episode_nav_stats::to_json) when the leg ended, [arms]; empty
  // for the leg that is still running.
  std::vector<std::string> nav_stats;
};

class run_log {
public:
  run_log() = default;
  explicit run_log(run_log_header h);

  const run_log_header &header() const { return h_; }
  int arms() const { return static_cast<int>(h_.arms.size()); }
  int agents() const { return h_.agents; }
  long ticks() const { return static_cast<long>(t_.size()); }
  bool empty() const { return t_.empty(); }
  double dt() const { return h_.dt; }
  double duration() const { return t_.empty() ? 0.0 : t_.back(); }
  const std::vector<double> &times() const { return t_; } // [ticks]: each tick's end (s)
  const std::vector<run_log_leg> &legs() const { return legs_; }
  bool finished() const { return finished_; } // it has its end line
  // Each arm's nav_stats record at the end ([arms]; empty before the end line).
  const std::vector<std::string> &end_nav_stats() const { return end_stats_; }
  // What the reader skipped (a last line cut short), one note each.
  const std::vector<std::string> &notes() const { return notes_; }

  // A column's index by name, or -1.
  int agent_column(std::string_view name) const;
  int arm_column(std::string_view name) const;
  // A value: tick k, arm a, agent i, column c (NaN for a column out of range).
  double at(long k, int a, int i, int c) const;
  double arm_at(long k, int a, int c) const;
  // By name (NaN for a column the log lacks).
  double at(long k, int a, int i, std::string_view column) const;
  double arm_at(long k, int a, std::string_view column) const;
  // A column over the ticks for arm a (and agent i), and its running sum, summed in tick order.
  std::vector<double> series(int a, int i, std::string_view column) const;
  std::vector<double> arm_series(int a, std::string_view column) const;
  std::vector<double> cumulative(int a, int i, std::string_view column) const;
  // The running sum to tick k inclusive (summed as cumulative() does).
  double sum_to(long k, int a, int i, std::string_view column) const;

  // The tick showing at time t: the last one ending at or before it, to a microsecond (the first
  // before that).
  long index(double t) const;
  // The leg tick k is in (0, 1, ...).
  int leg_of(long k) const;

  // ── building one (what arms_run does) ──
  void begin_leg(long tick, std::vector<double> goals);
  // Close the running leg with each arm's nav_stats record.
  void end_leg(std::vector<std::string> nav_stats);
  // A tick: its end time, agent values [arms][agents][agent columns], arm values [arms][arm
  // columns]. Throws std::invalid_argument for the wrong sizes.
  void add_tick(double t, const std::vector<double> &agent_values,
                const std::vector<double> &arm_values);
  void finish(std::vector<std::string> nav_stats);

  // The lines: the header, a leg, a tick k, a leg's end, the end (each without its newline).
  std::string leg_json(std::size_t leg) const;
  std::string leg_end_json(std::size_t leg) const;
  std::string tick_json(long k) const;
  std::string end_json() const;

  // The whole log, and back. read() throws std::runtime_error for a file that is not a run log
  // (another schema, or a malformed line before the last).
  void write(std::ostream &out) const;
  void write(const std::string &path) const;
  static run_log read(std::istream &in);
  static run_log read(const std::string &path);

  // Several logs of the same ticks side by side, one arm of each (arm `arms[j]` of logs[j]),
  // renamed `names[j]` in slot j: a scrubber's checkpoint beside step 0. Throws
  // std::invalid_argument for logs that differ in agents, columns or tick count.
  static run_log stack(const std::vector<const run_log *> &logs, const std::vector<int> &arms,
                       const std::vector<std::string> &names);

private:
  std::size_t agent_stride() const { return h_.agent_columns.size(); }
  std::size_t arm_stride() const { return h_.arm_columns.size(); }
  run_log_header h_;
  std::vector<double> t_;
  std::vector<double> agent_; // [ticks][arms][agents][agent columns]
  std::vector<double> arm_;   // [ticks][arms][arm columns]
  std::vector<run_log_leg> legs_;
  std::vector<std::string> end_stats_;
  std::vector<std::string> notes_;
  bool finished_ = false;
};

// Streams a run log's lines to a file as a run makes them, so a run that stops early leaves a log
// that reads up to its last tick.
class run_log_writer {
public:
  // Throws std::runtime_error when the file can't be opened.
  explicit run_log_writer(const std::string &path);
  ~run_log_writer();
  run_log_writer(const run_log_writer &) = delete;
  run_log_writer &operator=(const run_log_writer &) = delete;
  void line(const std::string &json); // one line, flushed
  const std::string &path() const { return path_; }

private:
  std::string path_;
  std::unique_ptr<std::ofstream> out_;
};

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_RUN_LOG_H__
