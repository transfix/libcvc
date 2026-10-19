/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_CLOCK_FOLLOWER_H__
#define __CVC_CLOCK_FOLLOWER_H__

#include <cmath>
#include <cstdint>
#include <cvc/core/exception.h>
#include <cvc/core/world_clock.h>

namespace cvc {

// ---------------
// Following one world_clock at several rates
// ---------------
// A process has one driver per world_clock; every simulation follows it. A
// simulation whose step is `dt` seconds runs once every k = dt / fixed_dt
// quanta, on the ticks t with t % k == phase. So a physics world stepping every
// quantum and a navigator stepping every fourth one share one notion of time,
// one speed control and one pause, and neither advances the clock itself.

// Quanta per step for a simulation of step `dt` on a clock of quantum
// `fixed_dt`. Throws cvc::unsupported_exception unless dt is a positive whole
// multiple of fixed_dt: a step of 7.2 quanta cannot be scheduled without
// drifting, which is exactly the mismatch that books one time and integrates
// another. The tolerance (relative 1e-6) admits a dt stored as float32, such
// as a trained 0.06f, against a double quantum.
inline std::uint64_t ticks_per_step(double dt, double fixed_dt) {
  if (!(dt > 0.0) || !std::isfinite(dt) || !(fixed_dt > 0.0) || !std::isfinite(fixed_dt))
    throw cvc::unsupported_exception("ticks_per_step: dt and fixed_dt must be finite and > 0");
  const double ratio = dt / fixed_dt;
  const double k = std::round(ratio);
  if (k < 1.0 || k > 9.0e18 || std::fabs(k * fixed_dt - dt) > 1.0e-6 * dt)
    throw cvc::unsupported_exception(
        "ticks_per_step: the step is not a whole number of clock quanta; configure the "
        "world_clock with a fixed_dt that divides every simulation's step");
  return static_cast<std::uint64_t>(k);
}

// How many steps of a simulation with this period and phase fall among the
// quanta first_tick + 1 .. first_tick + steps, i.e. the ticks t in that range
// with t % period == phase % period.
inline std::uint64_t steps_due(std::uint64_t first_tick, std::uint64_t steps, std::uint64_t period,
                               std::uint64_t phase = 0) {
  if (period == 0 || steps == 0)
    return 0;
  const std::uint64_t r = phase % period;
  // Count of t in [0, x] with t % period == r.
  auto upto = [&](std::uint64_t x) -> std::uint64_t { return x < r ? 0 : (x - r) / period + 1; };
  const std::uint64_t last = first_tick + steps; // a 64-bit tick does not wrap in practice
  return upto(last) - upto(first_tick);
}

// One simulation's cursor on a shared clock. take() returns how many of its
// steps fell due since the last call; the caller runs that many. A follower
// never advances the clock, so any number of them can share one.
class clock_follower {
public:
  explicit clock_follower(std::uint64_t period = 1, std::uint64_t phase = 0,
                          std::uint64_t start_tick = 0)
      : _period(period == 0 ? 1 : period), _phase(phase % (period == 0 ? 1 : period)),
        _last(start_tick) {}

  // A follower of a simulation whose step is `dt` seconds on `clock`, starting
  // at the clock's current tick. Throws if dt is not a whole number of quanta.
  static clock_follower for_step(const world_clock &clock, double dt, std::uint64_t phase = 0) {
    return clock_follower(ticks_per_step(dt, clock.fixed_dt()), phase, clock.tick());
  }

  // Steps due since the last call, given the clock's current tick. A clock that
  // went backwards (reset, configure, a seek in replay) re-anchors the follower
  // at `now_tick` and yields 0: restoring the simulation's own state to match
  // is the simulation's job, not the cursor's.
  std::uint64_t take(std::uint64_t now_tick) {
    if (now_tick < _last) {
      _last = now_tick;
      return 0;
    }
    const std::uint64_t n = steps_due(_last, now_tick - _last, _period, _phase);
    _last = now_tick;
    return n;
  }
  std::uint64_t take(const world_clock &clock) { return take(clock.tick()); }

  // Is a step of this simulation due on `tick`? For a driver that runs several
  // simulations quantum by quantum in a fixed order.
  bool due_at(std::uint64_t tick) const { return tick % _period == _phase; }

  // Presentation fraction into this simulation's NEXT step, given the clock's
  // tick and the alpha advance() returned (a fraction of one quantum). A
  // renderer interpolating between a navigator's last two states needs this,
  // not the clock's alpha: with a step of 4 quanta, two quanta past a step is
  // halfway, whatever the clock's own alpha says. In [0, 1).
  double alpha(std::uint64_t tick, double clock_alpha) const {
    const std::uint64_t since =
        (tick + _period - _phase) % _period; // quanta since the last due tick
    double a = (static_cast<double>(since) + clock_alpha) / static_cast<double>(_period);
    if (!(a >= 0.0))
      a = 0.0;
    if (a >= 1.0)
      a = std::nextafter(1.0, 0.0);
    return a;
  }

  std::uint64_t period() const { return _period; }
  std::uint64_t phase() const { return _phase; }
  std::uint64_t last_tick() const { return _last; }
  void reset(std::uint64_t tick) { _last = tick; }

private:
  std::uint64_t _period;
  std::uint64_t _phase;
  std::uint64_t _last;
};

} // namespace cvc

#endif // __CVC_CLOCK_FOLLOWER_H__
