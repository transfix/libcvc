/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_INPUT_TRACE_H__
#define __CVC_INPUT_TRACE_H__

#include <cstdint>
#include <cvc/core/namespace.h>
#include <functional>
#include <iosfwd>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace cvc {

class app;
class world_clock;

// ---------------
// cvc::input_trace — record the input, not the trajectory
// ---------------
// A run on a world_clock is a function of its starting state, its seed and its inputs stamped by
// tick (docs/WORLD_CLOCK_API.md, "The determinism contract"). An input trace holds exactly
// those, so a live session replays tick for tick, and becomes a benchmark: replay it on another
// build, or with another policy, and see where the trajectory departs from it.
//
// The file is cvc.input_trace/v1, JSON lines (docs/INPUT_TRACE.md):
//
//   {"schema":"cvc.input_trace/v1","header":{seed, dt, fixed_dt, phase, start_tick, world_id,
//       world, agents, settings, watch, build, meta}}
//   {"tick":120,"set":{"path":"nav.vehicle.vmax","value":"6"}}
//   {"tick":300,"event":{"name":"nav.retarget","args":{"agent":2,"goal":[0.5,-0.25]}}}
//   {"tick":600,"check":"sha256:..."}
//   {"end":{"tick":1200,"ticks":1200,"hash":"sha256:..."}}
//
// A record stamped with tick T was made after the quanta up to T ran and before quantum T + 1,
// so a replay applies it there: the first simulation step on a quantum after T sees it. Records
// are in tick order. A check is the state's hash at its tick (the end's is the final state's),
// for finding where a replay departs.

inline constexpr const char *kInputTraceSchema = "cvc.input_trace/v1";

// One agent's starting state, as the host describes it.
struct input_trace_agent {
  std::uint64_t id = 0;
  std::vector<double> start; // its start in the world's frame (nav: x, y, heading, speed)
  std::vector<double> goal;  // its goal (nav: x, y)
  std::string policy;        // its policy: "sha256:<hex>" of the policy file, or a name ("default")
};

struct input_trace_header {
  std::uint64_t seed = 0;
  double dt = 0.0;              // the simulation's step, seconds (0: not stated)
  double fixed_dt = 0.0;        // the clock's quantum (start() fills it in)
  std::uint64_t phase = 0;      // the step falls on ticks t with t % (dt / fixed_dt) == phase
  std::uint64_t start_tick = 0; // the clock's tick when recording began (start() fills it in)
  std::string world_id;         // the host's name for the world
  std::string world = "{}";     // JSON object: what a replay needs to build the world, if it can
  std::vector<input_trace_agent> agents;
  std::map<std::string, std::string> settings; // the watched keys' values at the start
  std::vector<std::string> watch;              // the prefixes watched
  std::string build = "{}";                    // JSON object: the libcvc build that recorded it
  std::string meta = "{}";                     // JSON object: the host's own

  // The header member of the first line, as a JSON object, and back. from_json throws
  // std::runtime_error for a document that is not one, naming what is wrong.
  std::string to_json() const;
  static input_trace_header from_json(const std::string &json);
};

struct input_trace_record {
  enum class kind { set, event, check };
  kind type = kind::set;
  std::uint64_t tick = 0;
  std::string path;        // set: the state path
  std::string value;       // set: its new value ("" unsets it)
  std::string name;        // event: the host's verb
  std::string args = "{}"; // event: its arguments, a JSON object
  std::string hash;        // check: the state's hash at the tick

  static input_trace_record set(std::uint64_t tick, std::string path, std::string value);
  static input_trace_record event(std::uint64_t tick, std::string name, std::string args = "{}");
  static input_trace_record check(std::uint64_t tick, std::string hash);
  std::string to_json() const; // its line
};

struct input_trace_end {
  bool present = false;   // a trace whose recorder never finished has no end
  std::uint64_t tick = 0; // the clock's tick at the end
  std::string hash;       // the final state's hash
  std::uint64_t ticks(const input_trace_header &h) const {
    return tick > h.start_tick ? tick - h.start_tick : 0;
  }
};

// A whole trace in memory.
struct input_trace {
  input_trace_header header;
  std::vector<input_trace_record> records; // in tick order
  input_trace_end end;

  // Read a trace. Throws std::runtime_error naming the line for anything that is not
  // cvc.input_trace/v1: a missing or different schema, a record of an unknown kind, ticks that go
  // backwards, a line after the end.
  static input_trace parse(std::istream &in);
  static input_trace parse(const std::string &text);
  static input_trace read(const std::string &path);
  void write(std::ostream &out) const;
  std::string str() const;
  void save(const std::string &path) const;

  // The last tick a replay runs to: the end's tick, or with no end the last record's (at least
  // the start tick).
  std::uint64_t last_tick() const;
};

// What `cvc-cli trace info` prints: the world, the step, the span, the agents and their
// policies, and the records by kind and by key.
std::string describe(const input_trace &t);

// The build that is running, as a header's build object: {"libcvc", "compiler", "arch", "os"}.
std::string input_trace_build();

// ---------------
// cvc::input_trace_recorder
// ---------------
// Records the inputs of a run on `clock`: every change to a watched key of the app's state tree
// and every event the host reports, stamped with the clock's tick. A plain object the host owns
// (an app, an Ariadne Runtime, a Python host), one per recording; nothing in it is process-wide.
//
//   cvc::input_trace_recorder rec(app, app.world_clock());
//   rec.start(header, "session.trace");       // header: seed, dt, world, agents
//   for (;;) {                                  // the frame loop
//     ... UI writes to nav.* land here, between frames ...
//     auto r = binding.drive(wall_dt);
//     for (auto n = nav.take(clock); n > 0; --n) { world.pull(app, ""); world.step(); }
//   }
//   rec.finish(cvc::nav::state_hash(world));
//
// What is recorded: a key whose path is a watched prefix or under one, and neither an ignored
// prefix nor under one, whose value changed (from what was last recorded for it, or from the
// start's snapshot). With skip_telemetry, a key whose "access" property is "telemetry"
// (cvc/state/state_key_meta.h) is an output and is skipped. A key removed from the tree is not
// recorded (v1). The clock's pacing (speed, pause, mode) is not an input and is not recorded:
// pacing decides when quanta run, never which.
//
// Stamping. on_write (the default) stamps a change with the clock's tick as it lands, which is
// exact for a host whose writes happen between frames, on the thread that steps. on_sample
// collects the changed keys and records their values when the host calls sample(), at the
// clock's tick then: a host whose writes come from another thread calls sample() just before it
// reads its inputs (world.pull), so the trace holds exactly what the simulation consumed.
//
// Thread-safe: a write on any thread may record.
class input_trace_recorder {
public:
  enum class stamping { on_write, on_sample };

  struct options {
    std::vector<std::string> watch{"nav", "nav_plant", "run", "world"};
    // What sim_world publishes beside its knobs: nav.agents, its agents' identity (which changes
    // through the host's verbs, not the tree), and nav.settings, what it was built with (the
    // trace's header holds that).
    std::vector<std::string> ignore{"nav.agents", "nav.settings"};
    bool skip_telemetry = true;
    stamping stamp = stamping::on_write;
  };

  input_trace_recorder(cvc::app &app, cvc::world_clock &clock);
  input_trace_recorder(cvc::app &app, cvc::world_clock &clock, options o);
  ~input_trace_recorder(); // stops watching; a trace not finished has no end record
  input_trace_recorder(const input_trace_recorder &) = delete;
  input_trace_recorder &operator=(const input_trace_recorder &) = delete;

  // Begin recording. The header's fixed_dt, start_tick and watch are filled in from the clock and
  // the options, its settings with the watched keys' values now, and an empty build with this
  // one's. With a path, every line is written there as it is recorded (and flushed at finish);
  // without, the trace is kept in memory only. Throws std::logic_error when already recording,
  // std::runtime_error for a path that can't be opened.
  void start(input_trace_header header, const std::string &path = std::string());

  // on_sample: record the keys changed since the last sample, at the clock's tick. on_write: no
  // effect.
  void sample();

  // A host verb that is not a write to the tree (a retarget, an obstacle dropped), at the
  // clock's tick. `args` is a JSON object; std::invalid_argument for one that isn't.
  void event(const std::string &name, const std::string &args = "{}");

  // The state's hash at the clock's tick, for a replay to compare (call it after the steps that
  // fell due). The same hash again at the same tick records nothing.
  void check(const std::string &hash);

  // End the recording with the final state's hash: the end record, and the file closed. Throws
  // std::runtime_error when the recording broke (the clock went back while recording).
  void finish(const std::string &hash);

  bool recording() const;
  bool finished() const;
  // The trace as recorded so far, with its end once finished.
  input_trace trace() const;
  std::size_t size() const; // records so far

private:
  struct impl;
  std::shared_ptr<impl> _impl; // shared with the tree's signal slot, which holds a weak reference
};

// ---------------
// cvc::input_trace_player
// ---------------
// Plays a trace's inputs back at their ticks, on `clock` in replay mode: time moves only when the
// player moves it. The host builds its world as the header describes, then:
//
//   cvc::input_trace_player play(app, clock, trace);
//   play.on_event([&](const cvc::input_trace_record &e) { apply(world, e); });
//   play.set_state_hash([&] { return cvc::nav::state_hash(world); });
//   play.rewind();
//   auto nav = cvc::clock_follower(cvc::ticks_per_step(h.dt, h.fixed_dt), h.phase, clock.tick());
//   play.run([&](std::uint64_t) {
//     for (auto n = nav.take(clock); n > 0; --n) { world.pull(app, ""); world.step(); }
//   });
//   bool same = play.matched();
//
// or, quantum by quantum (as a Python host does):
//
//   while (!play.done()) {
//     for (auto &e : play.apply_due()) apply(world, e); // sets written to the tree; events back
//     play.next_quantum();
//     ...run what fell due...
//     if (!play.expected_hash().empty()) play.verify(cvc::nav::state_hash(world));
//   }
//
// There is no going back: a simulation without save and restore (sim_world) seeks backwards by
// being rebuilt and replayed from the start (rewind()).
class input_trace_player {
public:
  input_trace_player(cvc::app &app, cvc::world_clock &clock, input_trace trace);

  void on_event(std::function<void(const input_trace_record &)> fn);
  void set_state_hash(std::function<std::string()> fn);

  // Back to the start: the clock in replay mode at the header's start tick (and fixed_dt; a
  // clock of another quantum is configured to it, which restarts it), the header's settings
  // written to the tree, and nothing applied or compared yet.
  void rewind();

  // Apply every record stamped at or before the clock's tick not yet applied: a set is written to
  // the state tree; an event goes to on_event's handler. Returns the events (not the sets).
  std::vector<input_trace_record> apply_due();

  // Move the clock one quantum on (seek_tick) and return the new tick. std::logic_error past the
  // trace's last tick.
  std::uint64_t next_quantum();

  // The hash the trace holds for the clock's tick (a check, or the end's at the last tick), ""
  // for none. verify(h) compares h with it and records the first tick where they differ; true
  // when they match or nothing is expected.
  std::string expected_hash() const;
  bool verify(const std::string &hash);

  // Run up to n quanta: before each, apply_due(); then next_quantum() and quantum(tick), the host
  // running whatever fell due on that tick; then, with set_state_hash, verify() where the trace
  // holds a hash. Returns the quanta run, fewer at the end. run() goes to the end.
  std::uint64_t advance(std::uint64_t n, const std::function<void(std::uint64_t)> &quantum);
  std::uint64_t run(const std::function<void(std::uint64_t)> &quantum);

  bool done() const;               // the clock is at the trace's last tick
  std::uint64_t last_tick() const; // input_trace::last_tick
  const input_trace &trace() const { return _trace; }

  // What verify() found: hashes compared and matched, and the first tick where one differed.
  std::size_t compared() const { return _compared; }
  std::size_t matched_count() const { return _matched; }
  bool diverged() const { return _diverged; }
  std::uint64_t first_divergence() const { return _first_divergence; }
  // Every hash compared matched, the end's among them.
  bool matched() const;

private:
  cvc::app &_app;
  cvc::world_clock &_clock;
  input_trace _trace;
  std::function<void(const input_trace_record &)> _on_event;
  std::function<std::string()> _state_hash;
  std::size_t _next = 0;            // the next record to apply (sets and events)
  std::vector<std::size_t> _checks; // indices of the check records
  std::size_t _compared = 0, _matched = 0;
  bool _diverged = false;
  bool _end_compared = false;
  std::uint64_t _first_divergence = 0;
  std::uint64_t _verified_tick = ~std::uint64_t(0); // the tick verify() last ran on
};

} // namespace cvc

#endif // __CVC_INPUT_TRACE_H__
