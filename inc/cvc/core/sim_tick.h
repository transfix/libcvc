/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

#ifndef __CVC_CORE_SIM_TICK_H__
#define __CVC_CORE_SIM_TICK_H__

#include <algorithm>
#include <cmath>
#include <cvc/core/world_clock.h>
#include <stdexcept>
#include <string>

namespace cvc {

// ----------------
// The simulation tick (WORLD-CLOCK-SYNC-ROADMAP, "One tick"; ARIADNE_WORLDS.md section 18.1)
// ----------------
// One clock for every simulation host and trainer, the holon's: 120 Hz quanta, and physics steps
// one a quantum. The navigator's step and the control period, one command per agent per tick, is a
// whole number of them. It is configurable everywhere (gym_options::tick, WorldVehicles::set_tick,
// pycvc_gl.cars' tick), and everything uses the same default:
//
//   kSimQuantum      1/120 s: the quantum every host's clock runs at (a holon's, a scene's, the
//                    gym's), and a physics step;
//   kSimTickQuanta   7: the navigator's tick in quanta;
//   kSimTick         7/120 s (0.0583 s): the navigator's tick, the nearest whole tick to the 0.06 s
//                    the shipped policies were trained at (owner, 2026-11-19: "7 quanta");
//   kSimCatchup      0.25 s: the world time a real-time frame may catch up at 1x. A stalled frame's
//                    time beyond it is dropped and reported (world_clock::step_result::
//                    dropped_steps, sim.dropped), never run late.
//
// Real time ticks against world time: the host's world_clock advances by wall time x speed in
// whole quanta (sim_clock_config), and each simulation follows it with a clock_follower, so a
// frame runs exactly the ticks that world time has passed.
inline constexpr double kSimQuantum = 1.0 / 120.0;
inline constexpr int kSimTickQuanta = 7;
inline constexpr double kSimTick = kSimTickQuanta * kSimQuantum;
inline constexpr double kSimCatchup = 0.25;

// The whole number of `step`s in `span` (both seconds). Throws std::invalid_argument when `span`
// or `step` is not finite and positive, or `span` is not a whole number of `step`s (to 1e-9 s).
inline int whole_steps(double span, double step) {
  if (!(span > 0.0) || !std::isfinite(span) || !(step > 0.0) || !std::isfinite(step))
    throw std::invalid_argument("whole_steps: the span and the step must be finite and > 0");
  const double k = std::round(span / step);
  if (k < 1.0 || std::fabs(k * step - span) > 1e-9)
    throw std::invalid_argument("whole_steps: " + std::to_string(span) +
                                " s is not a whole number of " + std::to_string(step) + " s steps");
  return static_cast<int>(k);
}

// ----------------
// A world's rate (ARIADNE_WORLDS.md section 18.9)
// ----------------
// 120 Hz is the default, not a law: a world that needs less (bandwidth, a relay's CPU) runs at a
// rate of its own, a whole number of ticks a second from kSimRateMin to kSimRateMax (a scene's
// `clock: { hz }`, arisrv --sim-hz). Its quantum is 1/hz, a physics step is one quantum, and the
// navigator's tick is the whole number of quanta nearest kSimTrainedTick (sim_tick_quanta: 7 at
// 120 Hz, 4 at 60, 2 at 30), so the shipped policies see the step they were trained at as nearly as
// the rate allows.
inline constexpr double kSimRateDefault = 120.0;
inline constexpr double kSimRateMin = 10.0;
inline constexpr double kSimRateMax = 240.0;
inline constexpr double kSimTrainedTick = 0.06; // the navigator's step the policies were trained at

// Whether HZ is a world's rate: a whole number from kSimRateMin to kSimRateMax.
inline bool sim_rate_valid(double hz) {
  return std::isfinite(hz) && hz >= kSimRateMin && hz <= kSimRateMax && hz == std::floor(hz);
}

// The quantum of a world at HZ ticks a second (kSimQuantum at the default 120).
inline double sim_quantum(double hz) { return hz == kSimRateDefault ? kSimQuantum : 1.0 / hz; }

// The navigator's tick in quanta of QUANTUM: the whole number nearest kSimTrainedTick, at least one
// (kSimTickQuanta at kSimQuantum).
inline int sim_tick_quanta(double quantum) {
  if (std::fabs(quantum - kSimQuantum) < 1e-15)
    return kSimTickQuanta;
  return std::max(1, static_cast<int>(std::lround(kSimTrainedTick / quantum)));
}

// The navigator's tick (s) on a clock of QUANTUM: sim_tick_quanta of them (kSimTick at
// kSimQuantum).
inline double sim_tick(double quantum) {
  if (std::fabs(quantum - kSimQuantum) < 1e-15)
    return kSimTick;
  return sim_tick_quanta(quantum) * quantum;
}

// A simulation host's clock: QUANTUM (the suite's by default), real time at `scale` (world seconds
// per wall second), and a catch-up of at most kSimCatchup of world time a frame at 1x, growing with
// the speed.
inline world_clock::config sim_clock_config(double scale = 1.0, double quantum = kSimQuantum) {
  world_clock::config c;
  c.fixed_dt = quantum;
  c.scale = scale;
  c.max_steps_per_advance = std::max(1, static_cast<int>(std::lround(kSimCatchup / quantum)));
  c.cap_scales_with_rate = true;
  return c;
}

// Put `clock` on sim_clock_config at QUANTUM (the suite's by default), keeping its speed, unless it
// has ticked (a quantum changes what every tick means, so a running clock is left as it is). True
// when it is at QUANTUM afterwards.
inline bool adopt_sim_clock(world_clock &clock, double quantum = kSimQuantum) {
  const bool at_quantum = std::fabs(clock.fixed_dt() - quantum) < 1e-12;
  if (clock.tick() != 0)
    return at_quantum;
  clock.configure(sim_clock_config(clock.current_config().scale, quantum));
  return true;
}

} // namespace cvc

#endif // __CVC_CORE_SIM_TICK_H__
