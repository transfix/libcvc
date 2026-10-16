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

// The regime's default display unit for a built-in dimension, as a registry
// symbol. SI shows each dimension in its SI unit except where a display
// convention differs (degrees for angles, degrees Celsius, Wh/km); imperial uses
// the customary unit. A dimension that is not built in has no default here and
// is shown in its SI unit (effective_display_unit).
const char *regime_default(const std::string &dim, system regime) {
  const bool imp = (regime == system::imperial);
  static const struct {
    const char *dim, *si, *imperial;
  } table[] = {
      {"length", "m", "ft"},
      {"mass", "kg", "lb"},
      {"time", "s", "s"},
      {"velocity", "m/s", "mph"},
      {"acceleration", "m/s^2", "ft/s^2"},
      {"force", "N", "lbf"},
      {"energy", "J", "ft-lbf"},
      {"angle", "deg", "deg"},
      {"power", "W", "hp"},
      {"pressure", "Pa", "psi"},
      {"temperature", "degC", "degF"},
      {"area", "m^2", "ft^2"},
      {"volume", "m^3", "ft^3"},
      {"angular_velocity", "deg/s", "deg/s"},
      {"curvature", "1/m", "1/ft"},
      {"frequency", "Hz", "Hz"},
      {"electric_current", "A", "A"},
      {"voltage", "V", "V"},
      {"electric_charge", "C", "C"},
      {"density", "kg/m^3", "lb/ft^3"},
      {"torque", "N*m", "lbf*ft"},
      {"energy_per_length", "Wh/km", "Wh/mi"},
  };
  for (const auto &row : table)
    if (dim == row.dim)
      return imp ? row.imperial : row.si;
  return nullptr;
}

// factor^e by repeated multiplication (exact for e = 0 and +-1).
double ipow(double factor, int e) {
  double r = 1.0;
  for (int i = 0; i < (e < 0 ? -e : e); ++i)
    r *= factor;
  return r;
}

} // namespace

const char *world_units::dimension_name(dimension d) {
  switch (d) {
  case dimension::length:
    return "length";
  case dimension::mass:
    return "mass";
  case dimension::time:
    return "time";
  case dimension::velocity:
    return "velocity";
  case dimension::acceleration:
    return "acceleration";
  case dimension::force:
    return "force";
  case dimension::energy:
    return "energy";
  case dimension::angle:
    return "angle";
  case dimension::power:
    return "power";
  case dimension::pressure:
    return "pressure";
  case dimension::temperature:
    return "temperature";
  case dimension::area:
    return "area";
  case dimension::volume:
    return "volume";
  case dimension::angular_velocity:
    return "angular_velocity";
  case dimension::curvature:
    return "curvature";
  case dimension::frequency:
    return "frequency";
  case dimension::electric_current:
    return "electric_current";
  case dimension::voltage:
    return "voltage";
  case dimension::electric_charge:
    return "electric_charge";
  case dimension::density:
    return "density";
  case dimension::torque:
    return "torque";
  case dimension::energy_per_length:
    return "energy_per_length";
  }
  return "length"; // unreachable; every enumerator is handled
}

void world_units::normalized_frame::validate() const {
  if (!(scale > 0.0) || !std::isfinite(scale))
    throw cvc::unit_error("normalized_frame: scale must be finite and > 0");
  if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(cz))
    throw cvc::unit_error("normalized_frame: the centre must be finite");
}

world_units::world_units() : world_units(config{}) {}

world_units::world_units(config cfg) : _cfg(cfg), _registry(std::make_shared<unit_registry>()) {
  validate(_cfg);
}

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

// --- the registry ---------------------------------------------------------------

unit_registry &world_units::registry() { return *_registry; }
const unit_registry &world_units::registry() const { return *_registry; }

double world_units::to_si(double value, const std::string &symbol) const {
  return _registry->to_si(value, symbol);
}

double world_units::from_si(double canonical, const std::string &symbol) const {
  return _registry->from_si(canonical, symbol);
}

double world_units::convert(double value, const std::string &from_symbol,
                            const std::string &to_symbol) const {
  return _registry->convert(value, from_symbol, to_symbol);
}

// --- display units --------------------------------------------------------------

// The unit `dim` is shown in under `regime`: the override, else the regime
// default, else (a custom dimension) its SI unit. Called with _mutex NOT held.
std::string world_units::effective_display_unit(const std::string &dim, system regime) const {
  {
    std::lock_guard<std::mutex> lk(_mutex);
    auto it = _display_overrides.find(dim);
    if (it != _display_overrides.end())
      return it->second;
  }
  if (const char *d = regime_default(dim, regime))
    return d;
  return _registry->dimension(dim).si_symbol; // throws for an unknown dimension
}

void world_units::set_display_unit(const std::string &dim, const std::string &symbol) {
  // Validate outside our lock (the registry has its own): the unit must exist
  // and belong to this dimension.
  const std::string unit_dim = _registry->dimension_of(symbol);
  if (unit_dim != dim)
    throw cvc::unit_error("display unit '" + symbol + "' is " + unit_dim + ", not " + dim);
  std::lock_guard<std::mutex> lk(_mutex);
  _display_overrides[dim] = symbol;
}

void world_units::set_display_unit(dimension d, const std::string &symbol) {
  set_display_unit(std::string(dimension_name(d)), symbol);
}

void world_units::clear_display_unit(const std::string &dim) {
  std::lock_guard<std::mutex> lk(_mutex);
  _display_overrides.erase(dim);
}

void world_units::clear_display_unit(dimension d) {
  clear_display_unit(std::string(dimension_name(d)));
}

std::string world_units::display_unit(const std::string &dim) const {
  return effective_display_unit(dim, regime());
}

std::string world_units::display_unit(dimension d) const {
  return display_unit(std::string(dimension_name(d)));
}

double world_units::to_display(double canonical_value, const std::string &dim) const {
  return _registry->from_si(canonical_value, effective_display_unit(dim, regime()));
}

double world_units::from_display(double display_value, const std::string &dim) const {
  return _registry->to_si(display_value, effective_display_unit(dim, regime()));
}

std::string world_units::unit_symbol(const std::string &dim) const {
  return effective_display_unit(dim, regime());
}

double world_units::to_display(double canonical_value, dimension d) const {
  return to_display(canonical_value, std::string(dimension_name(d)));
}

double world_units::from_display(double display_value, dimension d) const {
  return from_display(display_value, std::string(dimension_name(d)));
}

std::string world_units::unit_symbol(dimension d) const {
  return unit_symbol(std::string(dimension_name(d)));
}

world_units::measurement world_units::format(double canonical_value, const std::string &dim) const {
  system regime;
  bool length_override;
  {
    std::lock_guard<std::mutex> lk(_mutex);
    regime = _cfg.regime;
    length_override = _display_overrides.count("length") != 0;
  }
  measurement m;
  if (dim == "length" && !length_override) {
    // Lengths are promoted by magnitude (m -> km, ft -> mi) unless the
    // application pinned a length unit.
    const length_tier tier = pick_length_tier(canonical_value, regime);
    m.value = canonical_value / tier.metres_per_unit;
    m.unit = tier.unit;
  } else {
    m.unit = effective_display_unit(dim, regime);
    m.value = _registry->from_si(canonical_value, m.unit);
  }
  return m;
}

world_units::measurement world_units::format(double canonical_value, dimension d) const {
  return format(canonical_value, std::string(dimension_name(d)));
}

world_units::coordinate world_units::world_point_to_real(double wx, double wy, double wz) const {
  system regime;
  double scale;
  std::string length_unit;
  {
    std::lock_guard<std::mutex> lk(_mutex);
    regime = _cfg.regime;
    scale = _cfg.metres_per_world_unit;
    auto it = _display_overrides.find("length");
    if (it != _display_overrides.end())
      length_unit = it->second;
  }
  // World -> canonical metres, then choose ONE display tier for the whole point
  // from its largest component so the three axes never carry different units.
  const double mx = wx * scale, my = wy * scale, mz = wz * scale;
  coordinate c;
  if (!length_unit.empty()) {
    c.x = _registry->from_si(mx, length_unit);
    c.y = _registry->from_si(my, length_unit);
    c.z = _registry->from_si(mz, length_unit);
    c.unit = length_unit;
    return c;
  }
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
  c.x = mx / tier.metres_per_unit;
  c.y = my / tier.metres_per_unit;
  c.z = mz / tier.metres_per_unit;
  c.unit = tier.unit;
  return c;
}

// --- SI <-> normalized ------------------------------------------------------------

// SI -> world (divide by metres_per_world_unit^e) -> normalized (multiply by S^e),
// as two separate steps so each matches the documented chain; for e < 0 the
// factors invert. e = 0 returns the value untouched.
double world_units::scale_to_normalized(double canonical_value, int e,
                                        const normalized_frame &f) const {
  f.validate();
  if (e == 0)
    return canonical_value;
  const double mpwu = metres_per_world_unit();
  const double world = e > 0 ? canonical_value / ipow(mpwu, e) : canonical_value * ipow(mpwu, e);
  return e > 0 ? world * ipow(f.scale, e) : world / ipow(f.scale, e);
}

double world_units::scale_from_normalized(double normalized_value, int e,
                                          const normalized_frame &f) const {
  f.validate();
  if (e == 0)
    return normalized_value;
  const double mpwu = metres_per_world_unit();
  const double world =
      e > 0 ? normalized_value / ipow(f.scale, e) : normalized_value * ipow(f.scale, e);
  return e > 0 ? world * ipow(mpwu, e) : world / ipow(mpwu, e);
}

double world_units::to_normalized(double canonical_value, const std::string &dim,
                                  const normalized_frame &f) const {
  return scale_to_normalized(canonical_value, _registry->dimension(dim).length_exponent, f);
}

double world_units::from_normalized(double normalized_value, const std::string &dim,
                                    const normalized_frame &f) const {
  return scale_from_normalized(normalized_value, _registry->dimension(dim).length_exponent, f);
}

double world_units::to_normalized(double canonical_value, dimension d,
                                  const normalized_frame &f) const {
  return to_normalized(canonical_value, std::string(dimension_name(d)), f);
}

double world_units::from_normalized(double normalized_value, dimension d,
                                    const normalized_frame &f) const {
  return from_normalized(normalized_value, std::string(dimension_name(d)), f);
}

world_units::xyz world_units::si_point_to_normalized(double x_m, double y_m, double z_m,
                                                     const normalized_frame &f) const {
  f.validate();
  const double mpwu = metres_per_world_unit();
  xyz n;
  n.x = (x_m / mpwu - f.cx) * f.scale;
  n.y = (y_m / mpwu - f.cy) * f.scale;
  n.z = (z_m / mpwu - f.cz) * f.scale;
  return n;
}

world_units::xyz world_units::normalized_point_to_si(double nx, double ny, double nz,
                                                     const normalized_frame &f) const {
  f.validate();
  const double mpwu = metres_per_world_unit();
  xyz m;
  m.x = (nx / f.scale + f.cx) * mpwu;
  m.y = (ny / f.scale + f.cy) * mpwu;
  m.z = (nz / f.scale + f.cz) * mpwu;
  return m;
}

} // namespace cvc
