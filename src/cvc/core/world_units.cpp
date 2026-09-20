/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cmath>
#include <cvc/core/exception.h>
#include <cvc/core/world_units.h>

namespace cvc {

namespace {

using system = world_units::system;
using dimension = world_units::dimension;

// A zero, negative or non-finite world scale has no meaning and would corrupt
// every coordinate that flows through it -- and, because it stays finite-looking,
// would do so silently. Reject at the source (construction and the setter)
// rather than produce a units object that reports confidently-wrong metres. The
// `!(m > 0.0)` form is deliberately false for NaN (NaN survives every
// comparison), so a NaN scale is caught here and not on the first conversion.
void validate(const world_units::config &c) {
  if (!(c.metres_per_world_unit > 0.0) || !std::isfinite(c.metres_per_world_unit))
    throw cvc::unsupported_exception("world_units: metres_per_world_unit must be finite and > 0");
}

// The display unit a length of the given magnitude (in canonical metres) is
// best shown in, together with how many metres one of those units is. This is
// the ONE place the promotion thresholds live, so format() and
// world_point_to_real() agree bit-for-bit. A non-finite magnitude falls back to
// the base unit rather than being misclassified by a comparison that is false
// for NaN.
struct length_tier {
  const char *unit;
  double metres_per_unit;
};

length_tier pick_length_tier(double metres_magnitude, system regime) {
  const bool finite = std::isfinite(metres_magnitude);
  const double mag = finite ? std::fabs(metres_magnitude) : 0.0;
  if (regime == system::imperial) {
    if (finite && mag >= units::metres_per_mile)
      return {"mi", units::metres_per_mile};
    return {"ft", units::metres_per_foot};
  }
  if (finite && mag >= units::metres_per_kilometre)
    return {"km", units::metres_per_kilometre};
  return {"m", 1.0};
}

// The metres-per-one-display-base-unit for every dimension in a regime. SI is
// the identity for its own base units; imperial and angle convert. The base
// unit is the un-promoted one (metre/foot, not km/mile) -- promotion is a
// length-only, magnitude-driven concern handled above.
//
// Returned as "how much SI is one display unit", so to_display divides and
// from_display multiplies -- one factor drives both directions. The round trip
// is bit-exact when the factor is exactly representable (SI, foot, pound) and
// otherwise correct to within a ULP (mph, foot-pound); tests tolerate that with
// EXPECT_NEAR, and no world state is ever stored in the display form anyway.
double si_per_display_unit(dimension d, system regime) {
  const bool imp = (regime == system::imperial);
  switch (d) {
  case dimension::length:
    return imp ? units::metres_per_foot : 1.0;
  case dimension::mass:
    return imp ? units::kilograms_per_pound : 1.0;
  case dimension::time:
    return 1.0; // both systems measure time in seconds
  case dimension::velocity:
    return imp ? units::metres_per_second_per_mph : 1.0;
  case dimension::acceleration:
    return imp ? units::metres_per_foot : 1.0; // ft/s^2 vs m/s^2
  case dimension::force:
    return imp ? units::newtons_per_pound_force : 1.0;
  case dimension::energy:
    return imp ? units::joules_per_foot_pound : 1.0;
  case dimension::angle:
    // Angle is shown in degrees in BOTH regimes; the canonical unit is the
    // radian. One degree is this many radians.
    return units::radians_per_degree;
  }
  return 1.0; // unreachable; every enumerator is handled
}

const char *base_unit_symbol(dimension d, system regime) {
  const bool imp = (regime == system::imperial);
  switch (d) {
  case dimension::length:
    return imp ? "ft" : "m";
  case dimension::mass:
    return imp ? "lb" : "kg";
  case dimension::time:
    return "s";
  case dimension::velocity:
    return imp ? "mph" : "m/s";
  case dimension::acceleration:
    return imp ? "ft/s^2" : "m/s^2";
  case dimension::force:
    return imp ? "lbf" : "N";
  case dimension::energy:
    return imp ? "ft-lbf" : "J";
  case dimension::angle:
    return "deg";
  }
  return "";
}

} // namespace

world_units::world_units() : world_units(config{}) {}

world_units::world_units(config cfg) : _cfg(cfg) { validate(_cfg); }

world_units::system world_units::regime() const {
  std::lock_guard<std::mutex> lk(_mutex);
  return _cfg.regime;
}

void world_units::set_regime(system s) {
  std::lock_guard<std::mutex> lk(_mutex);
  _cfg.regime = s;
}

double world_units::metres_per_world_unit() const {
  std::lock_guard<std::mutex> lk(_mutex);
  return _cfg.metres_per_world_unit;
}

void world_units::set_metres_per_world_unit(double metres) {
  // Validate BEFORE taking the lock and BEFORE mutating: a rejected scale must
  // leave the object exactly as it was, so a caught exception is fully
  // recoverable.
  config probe = config{};
  probe.metres_per_world_unit = metres;
  validate(probe);
  std::lock_guard<std::mutex> lk(_mutex);
  _cfg.metres_per_world_unit = metres;
}

double world_units::world_to_metres(double world_length) const {
  double scale;
  {
    std::lock_guard<std::mutex> lk(_mutex);
    scale = _cfg.metres_per_world_unit;
  }
  return world_length * scale;
}

double world_units::metres_to_world(double metres) const {
  double scale;
  {
    std::lock_guard<std::mutex> lk(_mutex);
    scale = _cfg.metres_per_world_unit;
  }
  // scale is validated finite and > 0, so this never divides by zero.
  return metres / scale;
}

double world_units::to_display(double si_value, dimension d) const {
  system regime;
  {
    std::lock_guard<std::mutex> lk(_mutex);
    regime = _cfg.regime;
  }
  return si_value / si_per_display_unit(d, regime);
}

double world_units::from_display(double display_value, dimension d) const {
  system regime;
  {
    std::lock_guard<std::mutex> lk(_mutex);
    regime = _cfg.regime;
  }
  return display_value * si_per_display_unit(d, regime);
}

std::string world_units::unit_symbol(dimension d) const {
  system regime;
  {
    std::lock_guard<std::mutex> lk(_mutex);
    regime = _cfg.regime;
  }
  return base_unit_symbol(d, regime);
}

world_units::measurement world_units::format(double si_value, dimension d) const {
  system regime;
  {
    std::lock_guard<std::mutex> lk(_mutex);
    regime = _cfg.regime;
  }
  measurement m;
  if (d == dimension::length) {
    const length_tier tier = pick_length_tier(si_value, regime);
    m.value = si_value / tier.metres_per_unit;
    m.unit = tier.unit;
  } else {
    m.value = si_value / si_per_display_unit(d, regime);
    m.unit = base_unit_symbol(d, regime);
  }
  return m;
}

world_units::coordinate world_units::world_point_to_real(double wx, double wy, double wz) const {
  system regime;
  double scale;
  {
    std::lock_guard<std::mutex> lk(_mutex);
    regime = _cfg.regime;
    scale = _cfg.metres_per_world_unit;
  }
  // World -> canonical metres, then choose ONE display tier for the whole point
  // from its largest component so the three axes never carry different units.
  const double mx = wx * scale, my = wy * scale, mz = wz * scale;
  // Take the max over FINITE components only: a comparison against NaN is always
  // false, so a naive running max would let a NaN/Inf on one axis pin the whole
  // point to a non-finite magnitude and demote a genuinely large finite axis to
  // the base unit. A non-finite component is still passed through below.
  double maxmag = 0.0;
  for (double m : {mx, my, mz}) {
    const double a = std::fabs(m);
    if (std::isfinite(a) && a > maxmag)
      maxmag = a;
  }
  const length_tier tier = pick_length_tier(maxmag, regime);
  coordinate c;
  c.x = mx / tier.metres_per_unit;
  c.y = my / tier.metres_per_unit;
  c.z = mz / tier.metres_per_unit;
  c.unit = tier.unit;
  return c;
}

} // namespace cvc
