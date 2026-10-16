/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_UNIT_REGISTRY_H__
#define __CVC_UNIT_REGISTRY_H__

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace cvc {

// ---------------
// cvc::unit_registry
// ---------------
// The table every unit conversion in libcvc goes through, and the place to drop
// in a conversion for a specialized use case.
//
// A DIMENSION is a named physical quantity with exactly one canonical SI unit,
// the unit world state is stored in, and a length exponent: how the quantity
// scales when world units or nav's normalized frame rescale length (velocity 1,
// area 2, curvature -1, mass and time 0). The exponent is what lets one rule
// carry every dimension through cvc::world_units' world and normalized frames.
//
// A UNIT is a symbol in one dimension together with its map to that
// dimension's SI unit. Three kinds:
//
//   linear    si = v * scale                foot, mile, pound, knot, kWh, psi
//   affine    si = v * scale + offset       degC and degF over kelvin
//   function  si = to_si(v), v = from_si(si) dBm over watts, or anything else
//
// Linear and affine units invert exactly (one multiply or divide, one add), so a
// round trip is bit-exact whenever the factor is exactly representable.
// Function units invert through the caller's pair; the registry does not check
// that the pair are inverses.
//
// A default-constructed registry holds the built-in table: SI and imperial units
// for every dimension cvc::world_units knows, plus common extras (nautical
// miles, knots, kWh, Wh/km, mAh, psi, rpm, dBm, ...). unit_registry::empty()
// gives a registry with nothing in it, for a closed custom table.
//
// Symbols are unique within a registry: adding a symbol that exists, or a
// dimension name that exists with a different SI symbol or exponent, throws
// cvc::unit_error. So does converting between dimensions, or naming an unknown
// unit or dimension. A conversion never throws for a finite input to a linear
// or affine unit.
//
// Thread-safe. Registration takes an internal lock; a conversion copies the
// unit's entry under the lock and computes outside it, so a function unit's
// callable runs unlocked and may itself use the registry.
class unit_registry {
public:
  struct dimension_info {
    std::string name;
    std::string si_symbol;   // the canonical unit, e.g. "m/s"
    int length_exponent = 0; // world/normalized scaling: velocity 1, area 2, curvature -1
  };

  enum class unit_kind { linear, affine, function };

  struct unit_info {
    std::string symbol;
    std::string dimension;
    std::string name; // human-readable, e.g. "international foot"; may be empty
    unit_kind kind = unit_kind::linear;
    double scale = 1.0;  // linear/affine: SI per unit
    double offset = 0.0; // affine: SI value at zero in this unit
  };

  using conversion = std::function<double(double)>;

  // The built-in table.
  unit_registry();
  // A registry with no dimensions and no units.
  static unit_registry empty();

  unit_registry(const unit_registry &other);
  unit_registry &operator=(const unit_registry &other);

  // --- dimensions ---------------------------------------------------------

  // Declares a dimension and registers its SI unit (scale 1) under si_symbol.
  // Re-declaring an existing dimension with the same SI symbol and exponent is a
  // no-op; any other clash throws unit_error.
  void add_dimension(const std::string &name, const std::string &si_symbol,
                     int length_exponent = 0);
  bool has_dimension(const std::string &name) const;
  dimension_info dimension(const std::string &name) const; // throws if unknown
  std::vector<std::string> dimensions() const;             // sorted

  // --- units --------------------------------------------------------------

  // si = value * si_per_unit. si_per_unit must be finite and > 0.
  void add_linear(const std::string &symbol, const std::string &dimension, double si_per_unit,
                  const std::string &name = std::string());
  // si = value * si_per_unit + si_offset. Both finite, si_per_unit > 0.
  void add_affine(const std::string &symbol, const std::string &dimension, double si_per_unit,
                  double si_offset, const std::string &name = std::string());
  // si = to_si(value); value = from_si(si). Both callables are required.
  void add_function(const std::string &symbol, const std::string &dimension, conversion to_si,
                    conversion from_si, const std::string &name = std::string());

  bool has_unit(const std::string &symbol) const;
  unit_info unit(const std::string &symbol) const;                    // throws if unknown
  std::vector<std::string> units(const std::string &dimension) const; // sorted; throws if unknown
  // The dimension a unit belongs to; throws if the unit is unknown.
  std::string dimension_of(const std::string &symbol) const;

  // --- conversion ---------------------------------------------------------

  // A value in `symbol` to its dimension's SI unit, and back.
  double to_si(double value, const std::string &symbol) const;
  double from_si(double canonical, const std::string &symbol) const;
  // Between two units of the same dimension (through SI); throws unit_error
  // across dimensions.
  double convert(double value, const std::string &from_symbol, const std::string &to_symbol) const;

private:
  struct entry {
    unit_info info;
    conversion to_si;
    conversion from_si;
  };
  std::shared_ptr<const entry> find(const std::string &symbol) const;
  void insert(entry e);
  static void install_builtins(unit_registry &r);
  struct empty_tag {};
  explicit unit_registry(empty_tag);

  mutable std::mutex _mutex;
  std::map<std::string, dimension_info> _dimensions;
  std::map<std::string, std::shared_ptr<const entry>> _units;
};

} // namespace cvc

#endif // __CVC_UNIT_REGISTRY_H__
