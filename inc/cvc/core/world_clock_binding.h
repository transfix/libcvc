/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_WORLD_CLOCK_BINDING_H__
#define __CVC_WORLD_CLOCK_BINDING_H__

#include <cstdint>
#include <cvc/core/world_clock.h>
#include <string>

namespace cvc {

class app;

// ---------------
// cvc::world_clock_binding
// ---------------
// The state-tree face of a world_clock: the one place where speed, pause, mode
// and single-step are set for every simulation following that clock, and where
// the time they share is published. A UI slider, a Python script, a remote peer
// or a test writes the control keys; whoever drives the clock calls drive()
// once per frame.
//
// Keys are relative to `prefix` (default "sim", so a default binding speaks the
// keys Ariadne's sim_transport.ari binds: sim.speed, sim.paused, sim.time).
//
//   control, read by pull() and seeded from the clock when unset:
//     <prefix>.speed           double  world seconds per wall second; 0 pauses
//     <prefix>.paused          bool
//     <prefix>.mode            string  live | lockstep | stepping | replay
//     <prefix>.lockstep_steps  int     quanta per frame in lockstep
//     <prefix>.step            int     request counter: each increment runs one quantum
//
//   telemetry, written by push():
//     <prefix>.time            double  world seconds (tick * fixed_dt)
//     <prefix>.tick            uint64
//     <prefix>.fixed_dt        double
//     <prefix>.realized_speed  double  world seconds per wall second achieved
//     <prefix>.dropped         uint64  quanta discarded by the stall guard
//
// `paused` wins over `mode`: paused with mode live or lockstep runs nothing,
// and stepping and replay never advance from wall time anyway. A step request
// runs in any mode. An unreadable or unknown control value is ignored, never
// thrown into a frame.
class world_clock_binding {
public:
  world_clock_binding(cvc::app &ctx, cvc::world_clock &clock, std::string prefix = "sim");

  // Control keys -> clock. Seeds any unset key from the clock first.
  void pull();

  // Telemetry -> state tree.
  void push();

  // The one call a frame driver makes: pull(), advance(wall_dt), run any
  // requested single steps, push(). The returned result covers every quantum
  // run in this call: first_tick + 1 .. first_tick + steps.
  cvc::world_clock::step_result drive(double wall_dt);

  // Single steps requested through <prefix>.step and not yet run.
  std::uint64_t pending_steps() const { return _pending_steps; }

  // Absolute state path of a key under this binding's prefix.
  std::string key(const std::string &name) const;

  static const char *mode_name(cvc::world_clock::mode m);
  // false if `name` is not a mode.
  static bool parse_mode(const std::string &name, cvc::world_clock::mode &out);

private:
  cvc::app &_ctx;
  cvc::world_clock &_clock;
  std::string _prefix;
  bool _seeded = false;
  bool _paused = false;
  long long _step_seen = 0;
  std::uint64_t _pending_steps = 0;
  cvc::world_clock::mode _requested = cvc::world_clock::mode::live;

  void seed();
};

} // namespace cvc

#endif // __CVC_WORLD_CLOCK_BINDING_H__
