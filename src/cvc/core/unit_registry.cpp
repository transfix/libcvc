/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cmath>
#include <cvc/core/exception.h>
#include <cvc/core/unit_registry.h>
#include <cvc/core/world_units.h>

namespace cvc {

namespace {

void require(bool ok, const std::string &what) {
  if (!ok)
    throw cvc::unit_error(what);
}

bool positive_finite(double v) { return v > 0.0 && std::isfinite(v); }

} // namespace

unit_registry::unit_registry() { install_builtins(*this); }

unit_registry::unit_registry(empty_tag) {}

unit_registry unit_registry::empty() { return unit_registry(empty_tag{}); }

unit_registry::unit_registry(const unit_registry &other) {
  std::lock_guard<std::mutex> lk(other._mutex);
  _dimensions = other._dimensions;
  _units = other._units; // entries are immutable and shared
}

unit_registry &unit_registry::operator=(const unit_registry &other) {
  if (this == &other)
    return *this;
  std::map<std::string, dimension_info> dims;
  std::map<std::string, std::shared_ptr<const entry>> units;
  {
    std::lock_guard<std::mutex> lk(other._mutex);
    dims = other._dimensions;
    units = other._units;
  }
  std::lock_guard<std::mutex> lk(_mutex);
  _dimensions.swap(dims);
  _units.swap(units);
  return *this;
}

// --- dimensions -------------------------------------------------------------

void unit_registry::add_dimension(const std::string &name, const std::string &si_symbol,
                                  int length_exponent) {
  require(!name.empty(), "a dimension needs a name");
  require(!si_symbol.empty(), "dimension '" + name + "' needs an SI unit symbol");
  {
    std::lock_guard<std::mutex> lk(_mutex);
    auto it = _dimensions.find(name);
    if (it != _dimensions.end()) {
      require(it->second.si_symbol == si_symbol && it->second.length_exponent == length_exponent,
              "dimension '" + name + "' already exists with SI unit '" + it->second.si_symbol +
                  "' and length exponent " + std::to_string(it->second.length_exponent));
      return;
    }
    require(_units.find(si_symbol) == _units.end(),
            "unit '" + si_symbol + "' already exists; it cannot be the SI unit of '" + name + "'");
    _dimensions[name] = dimension_info{name, si_symbol, length_exponent};
  }
  entry e;
  e.info.symbol = si_symbol;
  e.info.dimension = name;
  e.info.name = "SI unit of " + name;
  insert(std::move(e));
}

bool unit_registry::has_dimension(const std::string &name) const {
  std::lock_guard<std::mutex> lk(_mutex);
  return _dimensions.count(name) != 0;
}

unit_registry::dimension_info unit_registry::dimension(const std::string &name) const {
  std::lock_guard<std::mutex> lk(_mutex);
  auto it = _dimensions.find(name);
  require(it != _dimensions.end(), "unknown dimension '" + name + "'");
  return it->second;
}

std::vector<std::string> unit_registry::dimensions() const {
  std::lock_guard<std::mutex> lk(_mutex);
  std::vector<std::string> out;
  for (const auto &kv : _dimensions)
    out.push_back(kv.first);
  return out;
}

// --- units --------------------------------------------------------------------

void unit_registry::insert(entry e) {
  require(!e.info.symbol.empty(), "a unit needs a symbol");
  std::lock_guard<std::mutex> lk(_mutex);
  require(_dimensions.count(e.info.dimension) != 0,
          "unit '" + e.info.symbol + "': unknown dimension '" + e.info.dimension + "'");
  require(_units.find(e.info.symbol) == _units.end(),
          "unit '" + e.info.symbol + "' already exists");
  // Take the key before moving `e`: in `m[k] = f(std::move(e))` C++17 evaluates
  // the right-hand side first, so `e.info.symbol` would already be empty.
  const std::string symbol = e.info.symbol;
  _units[symbol] = std::make_shared<const entry>(std::move(e));
}

void unit_registry::add_linear(const std::string &symbol, const std::string &dimension,
                               double si_per_unit, const std::string &name) {
  require(positive_finite(si_per_unit),
          "unit '" + symbol + "': the SI-per-unit factor must be finite and > 0");
  entry e;
  e.info = unit_info{symbol, dimension, name, unit_kind::linear, si_per_unit, 0.0};
  insert(std::move(e));
}

void unit_registry::add_affine(const std::string &symbol, const std::string &dimension,
                               double si_per_unit, double si_offset, const std::string &name) {
  require(positive_finite(si_per_unit),
          "unit '" + symbol + "': the SI-per-unit factor must be finite and > 0");
  require(std::isfinite(si_offset), "unit '" + symbol + "': the SI offset must be finite");
  entry e;
  e.info = unit_info{symbol, dimension, name, unit_kind::affine, si_per_unit, si_offset};
  insert(std::move(e));
}

void unit_registry::add_function(const std::string &symbol, const std::string &dimension,
                                 conversion to_si, conversion from_si, const std::string &name) {
  require(static_cast<bool>(to_si) && static_cast<bool>(from_si),
          "unit '" + symbol + "': a function unit needs both to_si and from_si");
  entry e;
  e.info = unit_info{symbol, dimension, name, unit_kind::function, 1.0, 0.0};
  e.to_si = std::move(to_si);
  e.from_si = std::move(from_si);
  insert(std::move(e));
}

std::shared_ptr<const unit_registry::entry> unit_registry::find(const std::string &symbol) const {
  std::lock_guard<std::mutex> lk(_mutex);
  auto it = _units.find(symbol);
  require(it != _units.end(), "unknown unit '" + symbol + "'");
  return it->second;
}

bool unit_registry::has_unit(const std::string &symbol) const {
  std::lock_guard<std::mutex> lk(_mutex);
  return _units.count(symbol) != 0;
}

unit_registry::unit_info unit_registry::unit(const std::string &symbol) const {
  return find(symbol)->info;
}

std::vector<std::string> unit_registry::units(const std::string &dimension) const {
  std::lock_guard<std::mutex> lk(_mutex);
  require(_dimensions.count(dimension) != 0, "unknown dimension '" + dimension + "'");
  std::vector<std::string> out;
  for (const auto &kv : _units)
    if (kv.second->info.dimension == dimension)
      out.push_back(kv.first);
  return out;
}

std::string unit_registry::dimension_of(const std::string &symbol) const {
  return find(symbol)->info.dimension;
}

// --- conversion ---------------------------------------------------------------

namespace {

// One multiply (to SI) and one divide (from SI) for a linear unit: the same
// operations cvc::world_units has always used, so its conversions are unchanged
// bit for bit now that they go through the registry.
double entry_to_si(const unit_registry::unit_info &u, const unit_registry::conversion &f,
                   double v) {
  switch (u.kind) {
  case unit_registry::unit_kind::linear:
    return v * u.scale;
  case unit_registry::unit_kind::affine:
    return v * u.scale + u.offset;
  case unit_registry::unit_kind::function:
    return f(v);
  }
  return v;
}

double entry_from_si(const unit_registry::unit_info &u, const unit_registry::conversion &f,
                     double canonical) {
  switch (u.kind) {
  case unit_registry::unit_kind::linear:
    return canonical / u.scale;
  case unit_registry::unit_kind::affine:
    return (canonical - u.offset) / u.scale;
  case unit_registry::unit_kind::function:
    return f(canonical);
  }
  return canonical;
}

} // namespace

double unit_registry::to_si(double value, const std::string &symbol) const {
  const std::shared_ptr<const entry> e = find(symbol);
  return entry_to_si(e->info, e->to_si, value);
}

double unit_registry::from_si(double canonical, const std::string &symbol) const {
  const std::shared_ptr<const entry> e = find(symbol);
  return entry_from_si(e->info, e->from_si, canonical);
}

double unit_registry::convert(double value, const std::string &from_symbol,
                              const std::string &to_symbol) const {
  const std::shared_ptr<const entry> a = find(from_symbol);
  const std::shared_ptr<const entry> b = find(to_symbol);
  require(a->info.dimension == b->info.dimension, "cannot convert '" + from_symbol + "' (" +
                                                      a->info.dimension + ") to '" + to_symbol +
                                                      "' (" + b->info.dimension + ")");
  if (a == b)
    return value;
  return entry_from_si(b->info, b->from_si, entry_to_si(a->info, a->to_si, value));
}

// --- the built-in table -------------------------------------------------------

void unit_registry::install_builtins(unit_registry &r) {
  namespace u = cvc::units;

  // Dimensions with their SI unit and length exponent. The first eight are the
  // world_units::dimension members of libcvc 3.5; the rest were added with the
  // registry for the physics tier and its energy ledger.
  r.add_dimension("length", "m", 1);
  r.add_dimension("mass", "kg", 0);
  r.add_dimension("time", "s", 0);
  r.add_dimension("velocity", "m/s", 1);
  r.add_dimension("acceleration", "m/s^2", 1);
  r.add_dimension("force", "N", 1);
  r.add_dimension("energy", "J", 2);
  r.add_dimension("angle", "rad", 0);
  r.add_dimension("power", "W", 2);
  r.add_dimension("pressure", "Pa", -1);
  r.add_dimension("temperature", "K", 0);
  r.add_dimension("area", "m^2", 2);
  r.add_dimension("volume", "m^3", 3);
  r.add_dimension("angular_velocity", "rad/s", 0);
  r.add_dimension("curvature", "1/m", -1);
  r.add_dimension("frequency", "Hz", 0);
  r.add_dimension("electric_current", "A", 0);
  r.add_dimension("voltage", "V", 0);
  r.add_dimension("electric_charge", "C", 0);
  r.add_dimension("density", "kg/m^3", -3);
  r.add_dimension("torque", "N*m", 2);
  r.add_dimension("energy_per_length", "J/m", 1);

  // length
  r.add_linear("km", "length", u::metres_per_kilometre, "kilometre");
  r.add_linear("cm", "length", 0.01, "centimetre");
  r.add_linear("mm", "length", 0.001, "millimetre");
  r.add_linear("ft", "length", u::metres_per_foot, "international foot");
  r.add_linear("in", "length", u::metres_per_foot / 12.0, "inch");
  r.add_linear("yd", "length", u::metres_per_foot * 3.0, "yard");
  r.add_linear("mi", "length", u::metres_per_mile, "statute mile");
  r.add_linear("nmi", "length", u::metres_per_nautical_mile, "nautical mile");
  // mass
  r.add_linear("g", "mass", 0.001, "gram");
  r.add_linear("t", "mass", 1000.0, "tonne");
  r.add_linear("lb", "mass", u::kilograms_per_pound, "avoirdupois pound");
  r.add_linear("oz", "mass", u::kilograms_per_pound / 16.0, "avoirdupois ounce");
  // time
  r.add_linear("ms", "time", 0.001, "millisecond");
  r.add_linear("min", "time", 60.0, "minute");
  r.add_linear("h", "time", 3600.0, "hour");
  // velocity
  r.add_linear("km/h", "velocity", 1000.0 / 3600.0, "kilometre per hour");
  r.add_linear("mph", "velocity", u::metres_per_second_per_mph, "mile per hour");
  r.add_linear("ft/s", "velocity", u::metres_per_foot, "foot per second");
  r.add_linear("kn", "velocity", u::metres_per_nautical_mile / 3600.0, "knot");
  // acceleration
  r.add_linear("ft/s^2", "acceleration", u::metres_per_foot, "foot per second squared");
  r.add_linear("gn", "acceleration", u::standard_gravity, "standard gravity");
  // force
  r.add_linear("kN", "force", 1000.0, "kilonewton");
  r.add_linear("lbf", "force", u::newtons_per_pound_force, "pound-force");
  // energy
  r.add_linear("kJ", "energy", 1000.0, "kilojoule");
  r.add_linear("Wh", "energy", 3600.0, "watt-hour");
  r.add_linear("kWh", "energy", 3.6e6, "kilowatt-hour");
  r.add_linear("ft-lbf", "energy", u::joules_per_foot_pound, "foot-pound-force");
  r.add_linear("BTU", "energy", u::joules_per_btu, "British thermal unit (IT)");
  // angle
  r.add_linear("deg", "angle", u::radians_per_degree, "degree");
  // power
  r.add_linear("kW", "power", 1000.0, "kilowatt");
  r.add_linear("hp", "power", u::watts_per_horsepower, "mechanical horsepower");
  // dBm: decibels relative to one milliwatt, a logarithmic power unit (RF link
  // budgets). A function unit: from_si of a non-positive power is -inf or NaN.
  r.add_function(
      "dBm", "power", [](double dbm) { return std::pow(10.0, (dbm - 30.0) / 10.0); },
      [](double watts) { return 10.0 * std::log10(watts) + 30.0; }, "decibel-milliwatt");
  // pressure
  r.add_linear("kPa", "pressure", 1000.0, "kilopascal");
  r.add_linear("bar", "pressure", 1.0e5, "bar");
  r.add_linear("psi", "pressure", u::pascals_per_psi, "pound-force per square inch");
  // temperature: affine over kelvin
  r.add_affine("degC", "temperature", 1.0, 273.15, "degree Celsius");
  r.add_affine("degF", "temperature", 5.0 / 9.0, 459.67 * 5.0 / 9.0, "degree Fahrenheit");
  // area
  r.add_linear("km^2", "area", 1.0e6, "square kilometre");
  r.add_linear("ha", "area", 1.0e4, "hectare");
  r.add_linear("ft^2", "area", u::metres_per_foot * u::metres_per_foot, "square foot");
  r.add_linear("mi^2", "area", u::metres_per_mile * u::metres_per_mile, "square mile");
  r.add_linear("acre", "area", u::square_metres_per_acre, "international acre");
  // volume
  r.add_linear("L", "volume", 0.001, "litre");
  r.add_linear("ft^3", "volume", u::metres_per_foot * u::metres_per_foot * u::metres_per_foot,
               "cubic foot");
  r.add_linear("gal", "volume", u::cubic_metres_per_us_gallon, "US liquid gallon");
  // angular velocity
  r.add_linear("deg/s", "angular_velocity", u::radians_per_degree, "degree per second");
  r.add_linear("rpm", "angular_velocity", 2.0 * u::pi / 60.0, "revolution per minute");
  // curvature
  r.add_linear("1/ft", "curvature", 1.0 / u::metres_per_foot, "per foot");
  r.add_linear("1/km", "curvature", 1.0 / u::metres_per_kilometre, "per kilometre");
  // frequency
  r.add_linear("kHz", "frequency", 1.0e3, "kilohertz");
  r.add_linear("MHz", "frequency", 1.0e6, "megahertz");
  r.add_linear("GHz", "frequency", 1.0e9, "gigahertz");
  // electric
  r.add_linear("mA", "electric_current", 1.0e-3, "milliampere");
  r.add_linear("mV", "voltage", 1.0e-3, "millivolt");
  r.add_linear("Ah", "electric_charge", 3600.0, "ampere-hour");
  r.add_linear("mAh", "electric_charge", 3.6, "milliampere-hour");
  // density
  r.add_linear("lb/ft^3", "density",
               u::kilograms_per_pound /
                   (u::metres_per_foot * u::metres_per_foot * u::metres_per_foot),
               "pound per cubic foot");
  // torque
  r.add_linear("lbf*ft", "torque", u::newtons_per_pound_force * u::metres_per_foot,
               "pound-force foot");
  // energy per length (a vehicle's consumption)
  r.add_linear("Wh/km", "energy_per_length", 3600.0 / 1000.0, "watt-hour per kilometre");
  r.add_linear("Wh/mi", "energy_per_length", 3600.0 / u::metres_per_mile, "watt-hour per mile");
  r.add_linear("kWh/100km", "energy_per_length", 3.6e6 / 1.0e5, "kilowatt-hour per 100 km");
}

} // namespace cvc
