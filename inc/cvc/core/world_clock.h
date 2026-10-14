/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_WORLD_CLOCK_H__
#define __CVC_WORLD_CLOCK_H__

#include <cstdint>
#include <cvc/core/namespace.h>
#include <mutex>

namespace cvc {

// ---------------
// cvc::world_clock
// ---------------
// The authoritative notion of *simulation* time, separated from wall time and
// from render cadence.
//
// Three clocks are easy to conflate and must not be:
//
//   wall time   - what a steady_clock reports; the only real input.
//   world time  - what the scene believes it is; this class.
//   render time - how often we present; a policy, not a clock.
//
// Simulation advances in whole `fixed_dt` quanta and rendering interpolates
// between the last two states by `alpha`. That decoupling buys three things
// we specifically need:
//
//   * determinism - the same scenario reproduces on a fast machine and a slow
//     one, which is what makes a benchmark or a replay trustworthy. A policy
//     integrated at a variable step is not comparable against itself.
//   * correct velocities - anything with units of per-second derives from
//     world dt, so rendering faster never makes anything move faster.
//   * smoothness independent of simulation rate.
//
// Usage, once per frame:
//
//     auto s = clock.advance(wall_dt_seconds);
//     for (int i = 0; i < s.steps; ++i) simulate(clock.fixed_dt());
//     render(lerp(prev_state, state, s.alpha));
//
// Thread-safe: every accessor and mutator takes an internal lock. The clock is
// deliberately *not* wired to the state tree here — it has no dependency on
// cvc::state or cvc::app, so it is usable from a bare simulation loop, from a
// test, or from a renderer with no cluster attached. Publishing it to a state
// tree (cvc/core/world_clock_binding.h) and, later, agreeing on it across a
// holarchy of peers layer on top.
//
// One clock, many simulations. A process has ONE driver per clock (the frame
// loop, a headless harness, or a sim worker with no renderer) that calls
// advance() once per frame. Everything else follows: a simulation whose step is
// k quanta long keeps a cvc::clock_follower (cvc/core/clock_follower.h) and runs
// the steps that fell due, so several simulations at different rates share one
// notion of time and one speed control without ever advancing the clock
// themselves. What a quantum computes never depends on the pacing mode: live,
// scaled, lockstep and stepping runs of the same input trace visit the same
// ticks in the same order.
//
class world_clock {
public:
  enum class mode {
    live,     // advance from wall time * scale
    replay,   // time is driven by seek_tick(), not by wall time
    paused,   // advance() yields no steps and banks no time
    stepping, // advance() yields nothing; step_once() yields exactly one tick
    lockstep, // advance() ignores wall time and yields exactly lockstep_steps
  };

  struct config {
    // The simulation quantum. Every step is exactly this long in world time.
    double fixed_dt = 1.0 / 120.0;

    // World seconds per wall second. 0 pauses; >1 is fast-forward; <1 is slow
    // motion. Negative is clamped to 0 -- running time backwards is a much
    // larger design question than a sign flip and is deliberately out of scope.
    // Non-finite throws from the constructor (a NaN scale would poison the
    // accumulator on the first frame and the clock would never step again).
    double scale = 1.0;

    // Spiral-of-death guard. If a frame stalls (a debugger breakpoint, a page
    // fault storm, a laptop lid), the banked time could demand hundreds of
    // steps, each of which makes the next frame later still. We clamp, and we
    // *report* what was dropped rather than silently swallowing it -- a
    // simulation that quietly skipped a second of world time while claiming
    // to be deterministic is worse than one that admits it.
    int max_steps_per_advance = 8;

    // Let the live cap grow with the rate: the effective cap becomes
    // max_steps_per_advance * max(1, ceil(scale)). At 2x a frame needs twice the
    // quanta, and a fixed cap sized for 1x would silently turn a requested 2x
    // into whatever the cap allows. Off by default, which keeps the cap fixed.
    bool cap_scales_with_rate = false;

    // Quanta per advance() in mode::lockstep: frame-by-frame simulation that
    // ignores wall time entirely (capture, inspection, a harness that renders).
    int lockstep_steps = 1;
  };

  // What one advance() call decided.
  struct step_result {
    int steps = 0;      // whole quanta to simulate now (<= the cap in live mode)
    double alpha = 0.0; // [0,1) interpolation into the *next* quantum
    // Quanta discarded by max_steps_per_advance. 64-bit because it counts
    // banked time, not steps run: a stall of hours at 120 Hz overflows a
    // 32-bit count, and a drop counter that wraps is worse than no counter.
    std::uint64_t dropped_steps = 0;
    // The tick before this call: the quanta to run are first_tick + 1 through
    // first_tick + steps. A follower stepping every k quanta needs exactly this
    // to know which of them were its own, and reading tick() again afterwards
    // would race with another advance.
    std::uint64_t first_tick = 0;
  };

  world_clock();
  explicit world_clock(config cfg);

  // Replace the configuration and return to tick 0. A new quantum changes what
  // every tick means (t() = tick * fixed_dt), so the tick, the bank, the drop
  // count and the speed measurement restart; the mode is kept. This is how a
  // host picks a quantum that every simulation it runs divides into whole
  // steps, before any of them starts. Throws on an invalid config, leaving the
  // clock unchanged.
  void configure(config cfg);
  config current_config() const;

  // Advance by a wall-clock delta (seconds). Returns how many fixed steps to
  // run and the presentation interpolation factor. Negative or non-finite
  // input is treated as zero, and an absurd delta (past ~1e15 world seconds,
  // which in practice means an absolute timestamp passed where a delta was
  // meant) is clamped -- the excess shows up in dropped_steps, not as a
  // silently bricked clock.
  step_result advance(double wall_dt);

  // Advance exactly one quantum, regardless of wall time. Intended for
  // mode::stepping (the mode you debug in), but valid in any mode. Any banked
  // remainder is left alone and reported as alpha, so stepping while time is
  // banked does not jerk the presentation backwards.
  step_result step_once();

  // Advance exactly n quanta now, regardless of mode and wall time, with no cap
  // (the caller asked for n). The batch primitive: a headless harness or an
  // episode generator runs as fast as the work allows and books exactly the
  // time it simulated. n <= 0 runs nothing.
  step_result advance_ticks(int n);

  // --- time ---------------------------------------------------------------

  // The current tick. This is the authoritative integer; world seconds are
  // derived from it.
  std::uint64_t tick() const;

  // World seconds. Computed as tick * fixed_dt, never accumulated -- summing
  // a float quantum a million times drifts, and a clock that drifts is not a
  // clock. This matters: at 120 Hz a day of simulation is ten million steps.
  double t() const;

  double fixed_dt() const;

  // Reposition in replay. Does not touch the accumulator's remainder.
  void seek_tick(std::uint64_t tick);

  // --- mode and rate ------------------------------------------------------

  mode current_mode() const;
  void set_mode(mode m);

  double scale() const;
  void set_scale(double s); // negative clamps to 0

  int lockstep_steps() const;
  void set_lockstep_steps(int n); // clamped to >= 1

  bool cap_scales_with_rate() const;
  void set_cap_scales_with_rate(bool on);

  // --- observability ------------------------------------------------------

  // Total quanta ever discarded by the clamp. Non-zero means this run is not
  // a faithful simulation of the elapsed wall time; surface it, do not hide
  // it.
  std::uint64_t total_dropped() const;

  // Banked fraction of a quantum not yet simulated, in [0, fixed_dt).
  double pending_seconds() const;

  // World seconds actually simulated per wall second, measured over the last
  // completed window of at least half a wall second of advance() calls (every
  // quantum counts, whichever call ran it). This is the speed the run achieved,
  // which is not the speed requested: a scene too heavy for 2x reads below 2
  // here (and drops quanta) instead of quietly running slower. 0 until the
  // first window completes.
  double realized_scale() const;

  // Back to tick 0 with an empty accumulator. Mode, scale and config persist.
  void reset();

private:
  mutable std::mutex _mutex;
  config _cfg;
  mode _mode = mode::live;
  std::uint64_t _tick = 0;
  double _accumulator = 0.0;
  std::uint64_t _dropped = 0;
  double _win_wall = 0.0;  // wall seconds in the open speed window
  double _win_world = 0.0; // world seconds run in the open speed window
  double _realized = 0.0;  // last completed window's world/wall ratio

  step_result advance_locked(double world_dt);
  step_result run_locked(std::uint64_t n); // run n quanta now, no cap, no bank
  void note_wall_locked(double wall_dt);
  void clear_measurement_locked();
};

} // namespace cvc

#endif // __CVC_WORLD_CLOCK_H__
