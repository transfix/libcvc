/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cmath>
#include <cvc/core/exception.h>
#include <cvc/core/world_clock.h>

namespace cvc {

namespace {

// Ceiling on the world delta a single advance() may bank. Anything past this
// is a caller bug, not a stall -- the classic one is passing an absolute
// timestamp where a delta was meant (seconds since the epoch is ~1.7e9, four
// million times inside this bound). The clamp is what keeps every downstream
// quantity provably finite: without it a large-but-finite wall_dt times a
// large-but-finite scale rounds to +inf, fmod(inf, fixed_dt) is NaN, and a NaN
// accumulator bricks the clock silently -- it simply never steps again.
constexpr double kMaxWorldDt = 1.0e15; // ~31 million years

// Largest quantum count we will convert out of a double. Below 2^63 with room
// to spare, so the conversion is always in range; the counter saturates rather
// than wrapping, because a drop count that wraps is worse than one that admits
// it stopped counting.
constexpr double kMaxCount = 9.0e18;

// A quantum of zero or less has no meaning and would divide by zero below; a
// non-finite one poisons every subsequent tick. Reject at construction rather
// than produce a clock that silently never advances. Same for a non-finite
// scale: NaN survives every comparison, so `scale < 0` does not catch it and
// the accumulator would take the NaN on the first frame.
void validate(const world_clock::config &c) {
  if (!(c.fixed_dt > 0.0) || !std::isfinite(c.fixed_dt))
    throw cvc::unsupported_exception("world_clock: fixed_dt must be finite and > 0");
  if (!std::isfinite(c.scale))
    throw cvc::unsupported_exception("world_clock: scale must be finite");
  if (c.max_steps_per_advance < 1)
    throw cvc::unsupported_exception("world_clock: max_steps_per_advance must be >= 1");
}

// Non-finite deltas (a NaN from an uninitialised timer, an inf from a divide)
// must not reach the accumulator, and time never runs backwards here.
double sanitize_dt(double dt) {
  if (!std::isfinite(dt) || dt < 0.0)
    return 0.0;
  if (dt > kMaxWorldDt)
    return kMaxWorldDt;
  return dt;
}

// alpha is what a renderer interpolates with; it must be in [0,1) no matter
// what the accumulator holds. Exactly 1.0 would present a frame belonging to a
// state we have not computed, and a NaN would propagate into the scene graph.
double clamp_alpha(double a) {
  if (!(a >= 0.0)) // false for NaN
    return 0.0;
  if (a >= 1.0)
    return std::nextafter(1.0, 0.0);
  return a;
}

} // namespace

world_clock::world_clock() : world_clock(config{}) {}

world_clock::world_clock(config cfg) : _cfg(cfg) {
  validate(_cfg);
  if (_cfg.scale < 0.0)
    _cfg.scale = 0.0;
}

world_clock::step_result world_clock::advance_locked(double world_dt) {
  step_result r;

  // sanitize_dt bounds the delta, so the accumulator stays finite and below
  // kMaxWorldDt + fixed_dt for every reachable input.
  _accumulator += sanitize_dt(world_dt);

  // Split the bank into whole quanta and a sub-quantum remainder in ONE
  // exact operation rather than a subtract-in-a-loop. fmod is exact, so the
  // phase is preserved bit-for-bit; repeated subtraction is not, and at a
  // large accumulator (where a quantum is only a few thousand ulps wide) the
  // rounding of each subtraction can walk the bank across a quantum boundary
  // -- a step neither run nor reported, which is precisely the silent loss
  // the drop counter exists to make impossible.
  const double remainder = std::fmod(_accumulator, _cfg.fixed_dt);
  // The quotient is mathematically an integer, but (acc - remainder) and the
  // division each carry up to an ulp, so it can land a hair BELOW it and
  // truncation would lose a quantum. Round, and saturate rather than convert
  // an out-of-range double (which is undefined behaviour, not a big number).
  double whole = (_accumulator - remainder) / _cfg.fixed_dt;
  if (!(whole > 0.0)) // false for NaN
    whole = 0.0;
  if (whole > kMaxCount)
    whole = kMaxCount;
  const std::uint64_t demanded = static_cast<std::uint64_t>(std::llround(whole));

  // Spiral-of-death guard: run at most max_steps_per_advance, report the rest.
  const std::uint64_t cap = static_cast<std::uint64_t>(_cfg.max_steps_per_advance);
  const std::uint64_t run = demanded < cap ? demanded : cap;
  const std::uint64_t discarded = demanded - run;

  _tick += run;
  _accumulator = remainder;
  r.steps = static_cast<int>(run); // <= max_steps_per_advance, an int by config
  r.dropped_steps = discarded;
  _dropped = (_dropped > UINT64_MAX - discarded) ? UINT64_MAX : _dropped + discarded;

  // Interpolation into the quantum we have not simulated yet.
  r.alpha = clamp_alpha(_accumulator / _cfg.fixed_dt);

  return r;
}

world_clock::step_result world_clock::advance(double wall_dt) {
  std::lock_guard<std::mutex> lk(_mutex);

  // paused banks nothing: resuming must not release a flood of stored time.
  // replay is driven by seek_tick, and stepping only by step_once.
  if (_mode == mode::paused || _mode == mode::replay || _mode == mode::stepping) {
    step_result r;
    r.alpha = clamp_alpha(_accumulator / _cfg.fixed_dt);
    return r;
  }

  // Sanitized on both sides of the multiply: two individually finite values
  // (a big dt, a big scale) can round to +inf, and inf must never reach the
  // accumulator.
  return advance_locked(sanitize_dt(sanitize_dt(wall_dt) * _cfg.scale));
}

world_clock::step_result world_clock::step_once() {
  std::lock_guard<std::mutex> lk(_mutex);
  ++_tick;
  step_result r;
  r.steps = 1;
  // Report the banked fraction, not 0: a caller that steps while a remainder
  // is banked would otherwise present a backwards jump in alpha and then jump
  // forward again on the next advance(). The bank itself is untouched --
  // step_once adds a quantum, it does not consume pending wall time.
  r.alpha = clamp_alpha(_accumulator / _cfg.fixed_dt);
  return r;
}

std::uint64_t world_clock::tick() const {
  std::lock_guard<std::mutex> lk(_mutex);
  return _tick;
}

double world_clock::t() const {
  std::lock_guard<std::mutex> lk(_mutex);
  // Derived, never accumulated -- see the header.
  return static_cast<double>(_tick) * _cfg.fixed_dt;
}

double world_clock::fixed_dt() const {
  std::lock_guard<std::mutex> lk(_mutex);
  return _cfg.fixed_dt;
}

void world_clock::seek_tick(std::uint64_t tick) {
  std::lock_guard<std::mutex> lk(_mutex);
  _tick = tick;
}

world_clock::mode world_clock::current_mode() const {
  std::lock_guard<std::mutex> lk(_mutex);
  return _mode;
}

void world_clock::set_mode(mode m) {
  std::lock_guard<std::mutex> lk(_mutex);
  // Leaving a non-live mode drops banked time rather than releasing it as a
  // burst of steps on the first live frame.
  if (_mode != m && m == mode::live)
    _accumulator = 0.0;
  _mode = m;
}

double world_clock::scale() const {
  std::lock_guard<std::mutex> lk(_mutex);
  return _cfg.scale;
}

void world_clock::set_scale(double s) {
  std::lock_guard<std::mutex> lk(_mutex);
  if (!std::isfinite(s) || s < 0.0)
    s = 0.0;
  _cfg.scale = s;
}

std::uint64_t world_clock::total_dropped() const {
  std::lock_guard<std::mutex> lk(_mutex);
  return _dropped;
}

double world_clock::pending_seconds() const {
  std::lock_guard<std::mutex> lk(_mutex);
  return _accumulator;
}

void world_clock::reset() {
  std::lock_guard<std::mutex> lk(_mutex);
  _tick = 0;
  _accumulator = 0.0;
  _dropped = 0;
}

} // namespace cvc
