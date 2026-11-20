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

// A simulation host's clock: the suite quantum, real time at `scale` (world seconds per wall
// second), and a catch-up of at most kSimCatchup of world time a frame at 1x, growing with the
// speed.
inline world_clock::config sim_clock_config(double scale = 1.0) {
  world_clock::config c;
  c.fixed_dt = kSimQuantum;
  c.scale = scale;
  c.max_steps_per_advance = static_cast<int>(std::lround(kSimCatchup / kSimQuantum));
  c.cap_scales_with_rate = true;
  return c;
}

// Put `clock` on sim_clock_config (the quantum and the catch-up), keeping its speed, unless it has
// ticked (a quantum changes what every tick means, so a running clock is left as it is). True when
// it is at the suite quantum afterwards.
inline bool adopt_sim_clock(world_clock &clock) {
  const bool at_quantum = std::fabs(clock.fixed_dt() - kSimQuantum) < 1e-12;
  if (clock.tick() != 0)
    return at_quantum;
  clock.configure(sim_clock_config(clock.current_config().scale));
  return true;
}

} // namespace cvc

#endif // __CVC_CORE_SIM_TICK_H__
