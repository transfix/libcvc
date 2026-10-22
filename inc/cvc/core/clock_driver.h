/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_CLOCK_DRIVER_H__
#define __CVC_CLOCK_DRIVER_H__

#include <chrono>
#include <cstdint>
#include <cvc/core/namespace.h>
#include <cvc/core/world_clock.h>
#include <functional>
#include <vector>

namespace cvc {

class world_clock_binding;

// ---------------
// cvc::clock_driver
// ---------------
// The one place a host advances a world_clock per frame and fans the result out
// to everything that follows it, in a fixed order, sharing a single reading of
// time.
//
// A world_clock says "one driver per clock; everything else follows" but leaves
// the driving to the host. In practice every real host grew the same loop by
// hand: measure a wall delta, call advance() exactly once, then -- in a
// load-bearing order -- step the simulations whose quanta fell due and present
// what the frame should show at the returned alpha. The Ariadne scene driver
// (tick_scene) is this loop; the conference host is this loop open-coded as an
// eleven-step pump; a holon will be this loop again. clock_driver is that loop
// factored out so there is exactly ONE advance per clock per frame and one place
// that owns the order.
//
// Two kinds of observer, mirroring the advance()/for-each-step/present shape the
// world_clock header sketches:
//
//   step observers  - fire first, in ascending `order`. Semantically "advance
//     your own state by the quanta that fell due this frame" (a physics world's
//     follow(), a navigator, a holon's simulation). They read frame.step
//     (first_tick + 1 .. first_tick + steps) or keep a cvc::clock_follower; they
//     must not advance the clock.
//   frame observers - fire after every step observer, in ascending `order`.
//     Semantically "do the once-per-frame work / present at frame.alpha" (drain
//     a transport, forward media stamped with frame.t, interpolate a pose,
//     publish telemetry). The conference's whole pump is frame observers.
//
// Both kinds receive the SAME clock_frame, computed once at the top of advance()
// -- so a media timestamp (frame.t) read by one observer is bit-identical to the
// one read by another, which is what lets a receiver align two streams that were
// stamped in the same frame.
//
// Threading: a clock_driver is single-threaded. advance() and the observers it
// calls run on whatever thread drives the frame (the SDL/render main thread for
// a window host). It takes no locks and must not be driven from two threads; a
// follower on another thread waits on the clock's wait_for_tick_change() and
// keeps its own cursor, it does not share a driver. Re-entering advance() from
// within an observer throws.
class clock_driver {
public:
  // A frame of time, computed once per advance() and handed to every observer.
  struct clock_frame {
    // What this advance decided: steps (whole quanta to run now), alpha
    // ([0,1) into the next quantum), dropped_steps, first_tick (the tick BEFORE
    // this advance; the quanta to run are first_tick + 1 .. first_tick + steps).
    world_clock::step_result step;
    std::uint64_t tick = 0; // clock.tick() AFTER this advance (first_tick + steps)
    double t = 0.0;         // world seconds, clock.t() (tick * fixed_dt)
    double alpha = 0.0;     // step.alpha; interpolation fraction into the next quantum
    double fixed_dt = 0.0;  // the clock's quantum
    // The raw wall delta this frame, in seconds, BEFORE any time scaling or
    // pause -- for work that must track real time no matter the sim rate
    // (a talk-hold decay, a camera, an fps readout). Never the scaled world dt.
    double wall_dt = 0.0;
    bool first = false;      // true only on the very first advance (step.steps == 0)
    std::uint64_t index = 0; // monotonic frame counter, 0 on the first advance
  };

  using observer_fn = std::function<void(const clock_frame &)>;

  // A registration token. Keep it to remove() the observer later; a default
  // (id == 0) handle names nothing.
  struct handle {
    std::uint64_t id = 0;
    explicit operator bool() const noexcept { return id != 0; }
    bool operator==(const handle &o) const noexcept { return id == o.id; }
  };

  explicit clock_driver(world_clock &clock);

  world_clock &clock() noexcept { return _clock; }
  const world_clock &clock() const noexcept { return _clock; }

  // Route advance() through a world_clock_binding instead of the clock directly,
  // so the sim.* control keys (speed/paused/mode/step) steer this driver and the
  // telemetry keys (tick/time/dropped) are published each frame. nullptr (the
  // default) drives the clock raw -- used where the clock is a private media
  // timebase with no state face. The binding is not owned.
  void set_binding(world_clock_binding *b) noexcept { _binding = b; }
  world_clock_binding *binding() const noexcept { return _binding; }

  // Register an observer. `order` sorts observers of the same kind ascending;
  // ties keep registration order (stable). Returns a handle for remove().
  // Registering from inside an observer is allowed; the new observer does not
  // fire until the next advance().
  handle add_step_observer(int order, observer_fn fn);
  handle add_frame_observer(int order, observer_fn fn);

  // Remove a previously registered observer. Safe to call from within an
  // observer, including an observer removing itself (a one-shot). Returns true
  // if a live observer with that handle was found.
  bool remove(handle h);

  // Advance the clock once and fire observers: every step observer in order,
  // then every frame observer in order, all with the same clock_frame.
  //
  // wall_dt < 0 (the default) measures a steady_clock delta since the previous
  // advance -- real-time animation that does not crawl under a slow frame. The
  // FIRST such call books no time (delta 0) and only seeds the baseline, so a
  // host that constructs the driver long before its first frame does not inject
  // a huge startup delta. wall_dt >= 0 uses the given delta verbatim (a fixed dt
  // for deterministic capture, or a test), on the first call too.
  //
  // Returns the frame it built; the same reference frame() returns until the
  // next advance.
  const clock_frame &advance(double wall_dt = -1.0);

  const clock_frame &frame() const noexcept { return _frame; }

private:
  struct entry {
    std::uint64_t id = 0;
    int order = 0;
    std::uint64_t seq = 0; // insertion order, breaks `order` ties stably
    observer_fn fn;
    bool dead = false;      // removed while firing; compacted after the fan-out
    bool want_step = false; // target list, used only for pending adds
  };

  handle add(std::vector<entry> &list, bool &dirty, int order, observer_fn fn);
  static void sort_if_dirty(std::vector<entry> &list, bool &dirty);
  void fire(std::vector<entry> &list);
  void flush_pending();

  world_clock &_clock;
  world_clock_binding *_binding = nullptr;

  std::vector<entry> _step_obs;
  std::vector<entry> _frame_obs;
  std::vector<entry> _pending; // observers added during a fan-out
  bool _step_dirty = false;
  bool _frame_dirty = false;

  std::uint64_t _next_id = 1;
  std::uint64_t _next_seq = 0;
  std::uint64_t _index = 0;

  bool _started = false;
  bool _in_advance = false;
  std::chrono::steady_clock::time_point _last_tp;

  clock_frame _frame;
};

} // namespace cvc

#endif // __CVC_CLOCK_DRIVER_H__
