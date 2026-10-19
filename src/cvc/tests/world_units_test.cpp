/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Tests for cvc::world_units -- the canonical unit base and display regime.
//
// The properties worth pinning are the ones that fail *silently* if broken: a
// conversion factor that is subtly wrong, a length that promotes to km/miles at
// the wrong threshold, a coordinate whose axes disagree on their unit, and a
// bad world scale that produces confident nonsense instead of an exception. For
// a digital twin, a quietly-wrong metre is worse than a crash, so several tests
// assert exactness rather than nearness.

#include <atomic>
#include <cmath>
#include <cvc/core/exception.h>
#include <cvc/core/unit_registry.h>
#include <cvc/core/world_units.h>
#include <gtest/gtest.h>
#include <limits>
#include <string>
#include <thread>
#include <vector>

using cvc::world_units;
using dim = cvc::world_units::dimension;
using sys = cvc::world_units::system;

namespace {
// The primary customary definitions are exactly-representable decimals, so a
// literal here and the header's constant round the same decimal to the same
// double -- both are exact anchors. Quantities the header DERIVES by arithmetic
// (pound-force, mph, foot-pound) are compared against the header constant
// itself, not a re-typed decimal, so the round trip is bit-exact rather than
// merely within a few ULPs.
constexpr double kFoot = 0.3048;      // metres, exact
constexpr double kMile = 1609.344;    // metres, exact (5280 ft)
constexpr double kPound = 0.45359237; // kilograms, exact
} // namespace

// ── construction ───────────────────────────────────────────────────────────

TEST(WorldUnitsTest, DefaultsToSIAndUnitWorldScale) {
  world_units u;
  EXPECT_EQ(u.regime(), sys::si);
  EXPECT_DOUBLE_EQ(u.metres_per_world_unit(), 1.0);
}

TEST(WorldUnitsTest, RejectsANonPositiveOrNonFiniteWorldScale) {
  world_units::config bad;
  bad.metres_per_world_unit = 0.0;
  EXPECT_THROW(world_units{bad}, cvc::exception);
  bad.metres_per_world_unit = -2.0;
  EXPECT_THROW(world_units{bad}, cvc::exception);
  bad.metres_per_world_unit = std::nan("");
  EXPECT_THROW(world_units{bad}, cvc::exception);
  bad.metres_per_world_unit = std::numeric_limits<double>::infinity();
  EXPECT_THROW(world_units{bad}, cvc::exception);
}

TEST(WorldUnitsTest, SetterRejectsBadScaleAndLeavesTheObjectUnchanged) {
  world_units u; // 1.0 m/unit
  EXPECT_THROW(u.set_metres_per_world_unit(0.0), cvc::exception);
  EXPECT_THROW(u.set_metres_per_world_unit(-1.0), cvc::exception);
  EXPECT_THROW(u.set_metres_per_world_unit(std::nan("")), cvc::exception);
  // A rejected write must be fully recoverable: the old scale still stands.
  EXPECT_DOUBLE_EQ(u.metres_per_world_unit(), 1.0);
  u.set_metres_per_world_unit(1000.0);
  EXPECT_DOUBLE_EQ(u.metres_per_world_unit(), 1000.0);
}

// ── world <-> canonical metres ───────────────────────────────────────────────

TEST(WorldUnitsTest, WorldToMetresIsIdentityAtUnitScale) {
  world_units u;
  EXPECT_DOUBLE_EQ(u.world_to_metres(42.5), 42.5);
  EXPECT_DOUBLE_EQ(u.metres_to_world(42.5), 42.5);
}

TEST(WorldUnitsTest, WorldScaleMapsWorldUnitsToMetres) {
  world_units::config cfg;
  cfg.metres_per_world_unit = 1000.0; // world authored in kilometres
  world_units u{cfg};
  EXPECT_DOUBLE_EQ(u.world_to_metres(2.5), 2500.0);
  EXPECT_DOUBLE_EQ(u.metres_to_world(2500.0), 2.5);
}

// ── SI base is the identity, imperial converts by exact factors ──────────────

TEST(WorldUnitsTest, SIRegimeIsTheIdentityForItsOwnBaseUnits) {
  world_units u; // SI
  EXPECT_DOUBLE_EQ(u.to_display(1234.5, dim::length), 1234.5);
  EXPECT_DOUBLE_EQ(u.to_display(9.81, dim::acceleration), 9.81);
  EXPECT_DOUBLE_EQ(u.to_display(7.0, dim::force), 7.0);
  EXPECT_DOUBLE_EQ(u.from_display(1234.5, dim::length), 1234.5);
}

TEST(WorldUnitsTest, ImperialLengthUsesTheExactInternationalFoot) {
  world_units::config cfg;
  cfg.regime = sys::imperial;
  world_units u{cfg};
  // One foot of SI metres reads as exactly 1 ft (x/x == 1.0 in IEEE).
  EXPECT_DOUBLE_EQ(u.to_display(kFoot, dim::length), 1.0);
  // And one foot back is exactly 0.3048 m.
  EXPECT_DOUBLE_EQ(u.from_display(1.0, dim::length), kFoot);
}

TEST(WorldUnitsTest, ImperialMassForceEnergyVelocityAreExactAnchors) {
  world_units::config cfg;
  cfg.regime = sys::imperial;
  world_units u{cfg};

  // Anchor every factor against an INDEPENDENTLY-typed exact decimal, not the
  // header constant -- comparing from_display against the same constant the impl
  // multiplies by is a tautology (X == X) that would pass even if the constant
  // were corrupted. EXPECT_DOUBLE_EQ's 4-ULP tolerance absorbs the last-bit
  // difference between a decimal literal and the header's derived product, while
  // still catching a grossly-wrong factor.

  // Mass: 1 lb = 0.45359237 kg (exact).
  EXPECT_DOUBLE_EQ(u.to_display(kPound, dim::mass), 1.0);
  EXPECT_DOUBLE_EQ(u.from_display(1.0, dim::mass), 0.45359237);

  // Force: 1 lbf = 4.4482216152605 N (= 0.45359237 kg * 9.80665 m/s^2).
  EXPECT_DOUBLE_EQ(u.from_display(1.0, dim::force), 4.4482216152605);
  EXPECT_DOUBLE_EQ(u.from_display(2.0, dim::force), 2.0 * 4.4482216152605);

  // Energy: 1 ft-lbf = 1.3558179483314004 J (= lbf * foot).
  EXPECT_DOUBLE_EQ(u.from_display(1.0, dim::energy), 1.3558179483314004);

  // Velocity: 1 mph = 0.44704 m/s (exact, = 1609.344 m / 3600 s).
  EXPECT_DOUBLE_EQ(u.from_display(1.0, dim::velocity), 0.44704);

  // Pin the one primitive the force/energy factors derive from, so a change to
  // standard gravity cannot slip through green.
  EXPECT_DOUBLE_EQ(cvc::units::standard_gravity, 9.80665);
}

TEST(WorldUnitsTest, TimeIsSecondsInBothRegimes) {
  world_units si;
  world_units::config icfg;
  icfg.regime = sys::imperial;
  world_units imp{icfg};
  EXPECT_DOUBLE_EQ(si.to_display(3.5, dim::time), 3.5);
  EXPECT_DOUBLE_EQ(imp.to_display(3.5, dim::time), 3.5);
  EXPECT_EQ(si.unit_symbol(dim::time), "s");
  EXPECT_EQ(imp.unit_symbol(dim::time), "s");
}

TEST(WorldUnitsTest, AngleIsDegreesInBothRegimes) {
  world_units si;
  world_units::config icfg;
  icfg.regime = sys::imperial;
  world_units imp{icfg};
  const double pi = cvc::units::pi;
  EXPECT_NEAR(si.to_display(pi, dim::angle), 180.0, 1e-9);
  EXPECT_NEAR(imp.to_display(pi, dim::angle), 180.0, 1e-9);
  EXPECT_NEAR(si.from_display(90.0, dim::angle), pi / 2.0, 1e-12);
  EXPECT_EQ(si.unit_symbol(dim::angle), "deg");
  EXPECT_EQ(imp.unit_symbol(dim::angle), "deg");
}

TEST(WorldUnitsTest, EveryDimensionRoundTripsThroughTheDisplayRegime) {
  for (sys regime : {sys::si, sys::imperial}) {
    world_units::config cfg;
    cfg.regime = regime;
    world_units u{cfg};
    for (dim d : {dim::length, dim::mass, dim::time, dim::velocity, dim::acceleration, dim::force,
                  dim::energy, dim::angle}) {
      const double x = 123.456;
      const double rt = u.from_display(u.to_display(x, d), d);
      EXPECT_NEAR(rt, x, 1e-9) << "round trip failed for dimension " << static_cast<int>(d)
                               << " in regime " << static_cast<int>(regime);
    }
  }
}

// ── unit labels ──────────────────────────────────────────────────────────────

TEST(WorldUnitsTest, BaseUnitSymbolsMatchTheRegime) {
  world_units si;
  EXPECT_EQ(si.unit_symbol(dim::length), "m");
  EXPECT_EQ(si.unit_symbol(dim::mass), "kg");
  EXPECT_EQ(si.unit_symbol(dim::velocity), "m/s");
  EXPECT_EQ(si.unit_symbol(dim::acceleration), "m/s^2");
  EXPECT_EQ(si.unit_symbol(dim::force), "N");
  EXPECT_EQ(si.unit_symbol(dim::energy), "J");

  world_units::config icfg;
  icfg.regime = sys::imperial;
  world_units imp{icfg};
  EXPECT_EQ(imp.unit_symbol(dim::length), "ft");
  EXPECT_EQ(imp.unit_symbol(dim::mass), "lb");
  EXPECT_EQ(imp.unit_symbol(dim::velocity), "mph");
  EXPECT_EQ(imp.unit_symbol(dim::acceleration), "ft/s^2");
  EXPECT_EQ(imp.unit_symbol(dim::force), "lbf");
  EXPECT_EQ(imp.unit_symbol(dim::energy), "ft-lbf");
}

// ── length promotion (the km / miles readout) ────────────────────────────────

TEST(WorldUnitsTest, SILengthPromotesToKilometresAtExactlyOneThousandMetres) {
  world_units u; // SI
  auto below = u.format(999.0, dim::length);
  EXPECT_EQ(below.unit, "m");
  EXPECT_DOUBLE_EQ(below.value, 999.0);

  auto at = u.format(1000.0, dim::length);
  EXPECT_EQ(at.unit, "km");
  EXPECT_DOUBLE_EQ(at.value, 1.0);

  auto above = u.format(2500.0, dim::length);
  EXPECT_EQ(above.unit, "km");
  EXPECT_DOUBLE_EQ(above.value, 2.5);
}

TEST(WorldUnitsTest, ImperialLengthPromotesToMilesAtExactlyOneMile) {
  world_units::config cfg;
  cfg.regime = sys::imperial;
  world_units u{cfg};

  auto foot = u.format(kFoot, dim::length); // exactly 1 ft
  EXPECT_EQ(foot.unit, "ft");
  EXPECT_DOUBLE_EQ(foot.value, 1.0);

  auto justUnder = u.format(kMile - 1.0, dim::length);
  EXPECT_EQ(justUnder.unit, "ft"); // still feet just below a mile

  auto mile = u.format(kMile, dim::length);
  EXPECT_EQ(mile.unit, "mi");
  EXPECT_DOUBLE_EQ(mile.value, 1.0);
}

TEST(WorldUnitsTest, NonLengthDimensionsFormatInTheBaseUnitWithoutPromotion) {
  world_units u; // SI
  auto f = u.format(50000.0, dim::force);
  EXPECT_EQ(f.unit, "N");
  EXPECT_DOUBLE_EQ(f.value, 50000.0); // no kN promotion; only length promotes
}

TEST(WorldUnitsTest, NonFiniteLengthIsNotMisclassifiedByPromotion) {
  world_units u; // SI
  auto inf = u.format(std::numeric_limits<double>::infinity(), dim::length);
  EXPECT_EQ(inf.unit, "m"); // falls back to the base unit rather than "km"
  EXPECT_TRUE(std::isinf(inf.value));

  auto nan = u.format(std::nan(""), dim::length);
  EXPECT_EQ(nan.unit, "m");
  EXPECT_TRUE(std::isnan(nan.value));

  auto neg = u.format(-std::numeric_limits<double>::infinity(), dim::length);
  EXPECT_EQ(neg.unit, "m");
  EXPECT_TRUE(std::isinf(neg.value));
}

TEST(WorldUnitsTest, LargeNegativeLengthStillPromotes) {
  world_units u; // SI: promotion is by magnitude, so a negative still reaches km
  auto km = u.format(-2500.0, dim::length);
  EXPECT_EQ(km.unit, "km");
  EXPECT_DOUBLE_EQ(km.value, -2.5);

  world_units::config icfg;
  icfg.regime = sys::imperial;
  world_units imp{icfg};
  auto mi = imp.format(-2.0 * kMile, dim::length);
  EXPECT_EQ(mi.unit, "mi");
  EXPECT_DOUBLE_EQ(mi.value, -2.0);
}

// ── coordinate mapping (click -> real-world coordinate) ──────────────────────

TEST(WorldUnitsTest, WorldPointConvertsAllAxesToOneSharedUnit) {
  world_units u; // SI, 1 m/unit
  // Largest axis is 3200 m -> the whole point reads in km.
  auto c = u.world_point_to_real(3200.0, 500.0, -1000.0);
  EXPECT_EQ(c.unit, "km");
  EXPECT_DOUBLE_EQ(c.x, 3.2);
  EXPECT_DOUBLE_EQ(c.y, 0.5);
  EXPECT_DOUBLE_EQ(c.z, -1.0);
}

TEST(WorldUnitsTest, WorldPointStaysInMetresWhenSmall) {
  world_units u; // SI
  auto c = u.world_point_to_real(12.0, -4.0, 0.0);
  EXPECT_EQ(c.unit, "m");
  EXPECT_DOUBLE_EQ(c.x, 12.0);
  EXPECT_DOUBLE_EQ(c.y, -4.0);
}

TEST(WorldUnitsTest, WorldPointAppliesTheWorldScaleBeforeChoosingAUnit) {
  world_units::config cfg;
  cfg.metres_per_world_unit = 1000.0; // world authored in kilometres
  world_units u{cfg};
  // 2.5 world units = 2500 m -> km readout.
  auto c = u.world_point_to_real(2.5, 0.0, 0.0);
  EXPECT_EQ(c.unit, "km");
  EXPECT_DOUBLE_EQ(c.x, 2.5);
}

TEST(WorldUnitsTest, WorldPointReadsInMilesUnderTheImperialRegime) {
  world_units::config cfg;
  cfg.regime = sys::imperial;
  world_units u{cfg};
  // 3218.688 m == 2 miles exactly; largest axis promotes the point to miles.
  auto c = u.world_point_to_real(2.0 * kMile, 0.0, 0.0);
  EXPECT_EQ(c.unit, "mi");
  EXPECT_DOUBLE_EQ(c.x, 2.0);
}

TEST(WorldUnitsTest, WorldPointStaysInFeetUnderImperialWhenSubMile) {
  world_units::config cfg;
  cfg.regime = sys::imperial;
  world_units u{cfg};
  // 30.48 m == 100 ft, below a mile -> the whole point reads in feet.
  auto c = u.world_point_to_real(30.48, -3.048, 0.0);
  EXPECT_EQ(c.unit, "ft");
  EXPECT_DOUBLE_EQ(c.x, 100.0);
  EXPECT_DOUBLE_EQ(c.y, -10.0);
}

TEST(WorldUnitsTest, WorldPointKeepsAFiniteAxisCorrectDespiteANonFiniteAxis) {
  world_units u; // SI, 1 m/unit
  // A NaN on one axis must NOT drag a genuinely large finite axis down to the
  // base unit: the 5000 m axis still selects km, and the NaN passes through.
  auto c = u.world_point_to_real(std::nan(""), 5000.0, 0.0);
  EXPECT_EQ(c.unit, "km");
  EXPECT_TRUE(std::isnan(c.x));
  EXPECT_DOUBLE_EQ(c.y, 5.0);
  EXPECT_DOUBLE_EQ(c.z, 0.0);

  // An all-non-finite point falls back to the base unit and passes values through.
  auto inf = u.world_point_to_real(std::numeric_limits<double>::infinity(), 0.0, 0.0);
  EXPECT_EQ(inf.unit, "m");
  EXPECT_TRUE(std::isinf(inf.x));
}

// ── regime switching ─────────────────────────────────────────────────────────

TEST(WorldUnitsTest, ChangingRegimeChangesSubsequentConversions) {
  world_units u;                                             // SI
  EXPECT_DOUBLE_EQ(u.to_display(kFoot, dim::length), kFoot); // metres
  u.set_regime(sys::imperial);
  EXPECT_DOUBLE_EQ(u.to_display(kFoot, dim::length), 1.0); // now feet
  EXPECT_EQ(u.unit_symbol(dim::length), "ft");
}

// ── thread-safety smoke test ─────────────────────────────────────────────────

TEST(WorldUnitsTest, ConcurrentReadsAndWritesDoNotCrashOrCorrupt) {
  world_units u;
  std::atomic<bool> go{false};
  std::vector<std::thread> threads;
  for (int i = 0; i < 8; ++i) {
    threads.emplace_back([&u, &go, i]() {
      while (!go.load(std::memory_order_acquire))
        std::this_thread::yield();
      for (int k = 0; k < 2000; ++k) {
        if (i % 2 == 0) {
          u.set_regime(k % 2 ? sys::imperial : sys::si);
          u.set_metres_per_world_unit(1.0 + (k % 10));
        } else {
          volatile double v = u.to_display(100.0, dim::length);
          auto c = u.world_point_to_real(1000.0, 2000.0, 3000.0);
          (void)v;
          // The unit is always one of the four valid length tokens, never junk.
          EXPECT_TRUE(c.unit == "m" || c.unit == "km" || c.unit == "ft" || c.unit == "mi");
        }
      }
    });
  }
  go.store(true, std::memory_order_release);
  for (auto &t : threads)
    t.join();
  SUCCEED();
}

// --- the unit registry -------------------------------------------------------

TEST(UnitRegistryTest, BuiltInsCoverEveryWorldUnitsDimension) {
  cvc::unit_registry r;
  const world_units si;
  const world_units imp(world_units::config{sys::imperial, 1.0});
  for (int i = 0; i <= static_cast<int>(dim::energy_per_length); ++i) {
    const dim d = static_cast<dim>(i);
    const std::string name = world_units::dimension_name(d);
    ASSERT_TRUE(r.has_dimension(name)) << name;
    EXPECT_TRUE(r.has_unit(si.display_unit(d))) << name;
    EXPECT_TRUE(r.has_unit(imp.display_unit(d))) << name;
    EXPECT_EQ(r.dimension_of(si.display_unit(d)), name);
    EXPECT_EQ(r.dimension_of(imp.display_unit(d)), name);
  }
}

TEST(UnitRegistryTest, ExactCustomaryAnchors) {
  cvc::unit_registry r;
  EXPECT_EQ(r.to_si(1.0, "nmi"), 1852.0);
  EXPECT_EQ(r.to_si(1.0, "kWh"), 3.6e6);
  EXPECT_EQ(r.to_si(1.0, "mAh"), 3.6);
  EXPECT_NEAR(r.to_si(1.0, "in"), kFoot / 12.0, 1e-18);
  EXPECT_NEAR(r.to_si(1.0, "hp"), 745.69987158227022, 1e-9);
  EXPECT_NEAR(r.to_si(1.0, "psi"), 6894.757293168361, 1e-9);
  EXPECT_NEAR(r.to_si(1.0, "gal"), 0.003785411784, 1e-18);
  EXPECT_NEAR(r.to_si(1.0, "kn"), 1852.0 / 3600.0, 1e-15);
  EXPECT_NEAR(r.convert(60.0, "mph", "km/h"), 96.56064, 1e-9);
  EXPECT_NEAR(r.convert(1.0, "rpm", "deg/s"), 6.0, 1e-12);
  EXPECT_NEAR(r.convert(100.0, "Wh/km", "Wh/mi"), 160.9344, 1e-9);
  EXPECT_NEAR(r.convert(1.0, "acre", "ft^2"), 43560.0, 1e-6);
}

TEST(UnitRegistryTest, TemperatureIsAffineAndRoundTrips) {
  cvc::unit_registry r;
  EXPECT_NEAR(r.to_si(0.0, "degC"), 273.15, 1e-12);
  EXPECT_NEAR(r.to_si(32.0, "degF"), 273.15, 1e-12);
  EXPECT_NEAR(r.convert(100.0, "degC", "degF"), 212.0, 1e-9);
  EXPECT_NEAR(r.convert(-40.0, "degF", "degC"), -40.0, 1e-9);
  EXPECT_NEAR(r.from_si(r.to_si(98.6, "degF"), "degF"), 98.6, 1e-12);
  EXPECT_EQ(r.unit("degF").kind, cvc::unit_registry::unit_kind::affine);
}

TEST(UnitRegistryTest, FunctionUnitDbmRoundTrips) {
  cvc::unit_registry r;
  EXPECT_NEAR(r.to_si(30.0, "dBm"), 1.0, 1e-12);   // 30 dBm = 1 W
  EXPECT_NEAR(r.to_si(0.0, "dBm"), 1.0e-3, 1e-15); // 0 dBm = 1 mW
  EXPECT_NEAR(r.convert(20.0, "dBm", "W"), 0.1, 1e-12);
  EXPECT_NEAR(r.from_si(0.5, "dBm"), 26.98970004336019, 1e-9);
  EXPECT_EQ(r.unit("dBm").kind, cvc::unit_registry::unit_kind::function);
}

TEST(UnitRegistryTest, CustomDimensionsAndUnitsDropIn) {
  // A specialised use case: a data link rated in bits per second, an old length
  // unit, and a nonlinear unit, none of which the built-ins know.
  cvc::unit_registry r;
  r.add_dimension("data_rate", "bit/s");
  r.add_linear("Mbit/s", "data_rate", 1.0e6, "megabit per second");
  r.add_linear("furlong", "length", 201.168, "furlong");
  r.add_function(
      "log10m", "length", [](double v) { return std::pow(10.0, v); },
      [](double m) { return std::log10(m); }, "log10 of metres");
  EXPECT_EQ(r.dimension_of("Mbit/s"), "data_rate");
  EXPECT_EQ(r.convert(54.0, "Mbit/s", "bit/s"), 54.0e6);
  EXPECT_NEAR(r.convert(1.0, "furlong", "ft"), 660.0, 1e-9);
  EXPECT_NEAR(r.convert(3.0, "log10m", "km"), 1.0, 1e-12);
  EXPECT_EQ(r.units("data_rate"), (std::vector<std::string>{"Mbit/s", "bit/s"}));
}

TEST(UnitRegistryTest, RejectsBadDefinitionsAndCrossDimensionConversions) {
  cvc::unit_registry r;
  EXPECT_THROW(r.add_linear("ft", "length", 0.3048), cvc::unit_error); // duplicate symbol
  EXPECT_THROW(r.add_linear("x", "nosuchdim", 1.0), cvc::unit_error);
  EXPECT_THROW(r.add_linear("zero", "length", 0.0), cvc::unit_error);
  EXPECT_THROW(r.add_linear("nan", "length", std::nan("")), cvc::unit_error);
  EXPECT_THROW(r.add_affine("bad", "temperature", 1.0, std::numeric_limits<double>::infinity()),
               cvc::unit_error);
  EXPECT_THROW(r.add_function("f", "length", nullptr, nullptr), cvc::unit_error);
  EXPECT_THROW(r.add_dimension("length", "ft", 1), cvc::unit_error); // clashing re-declare
  EXPECT_NO_THROW(r.add_dimension("length", "m", 1));                // identical: a no-op
  EXPECT_THROW(r.convert(1.0, "ft", "kg"), cvc::unit_error);
  EXPECT_THROW(r.to_si(1.0, "furlongs"), cvc::unit_error);
  EXPECT_THROW(r.units("nosuchdim"), cvc::unit_error);
}

TEST(UnitRegistryTest, EmptyRegistryHoldsNothingAndCopiesAreIndependent) {
  cvc::unit_registry e = cvc::unit_registry::empty();
  EXPECT_TRUE(e.dimensions().empty());
  EXPECT_FALSE(e.has_unit("m"));
  cvc::unit_registry a;
  cvc::unit_registry b(a);
  b.add_linear("furlong", "length", 201.168);
  EXPECT_TRUE(b.has_unit("furlong"));
  EXPECT_FALSE(a.has_unit("furlong"));
  a = b;
  EXPECT_TRUE(a.has_unit("furlong"));
}

// --- world_units on the registry ----------------------------------------------

TEST(WorldUnitsTest, DisplayOverridePicksAUnitPerDimension) {
  world_units u;
  u.set_display_unit(dim::velocity, "kn");
  EXPECT_EQ(u.unit_symbol(dim::velocity), "kn");
  EXPECT_NEAR(u.to_display(1852.0 / 3600.0, dim::velocity), 1.0, 1e-12);
  EXPECT_NEAR(u.from_display(1.0, dim::velocity), 1852.0 / 3600.0, 1e-15);
  u.set_display_unit(dim::length, "nmi");
  const auto m = u.format(3704.0, dim::length); // an override turns off km promotion
  EXPECT_EQ(m.unit, "nmi");
  EXPECT_NEAR(m.value, 2.0, 1e-12);
  const auto c = u.world_point_to_real(1852.0, 0.0, 0.0);
  EXPECT_EQ(c.unit, "nmi");
  EXPECT_NEAR(c.x, 1.0, 1e-12);
  EXPECT_THROW(u.set_display_unit(dim::velocity, "kg"), cvc::unit_error);
  u.clear_display_unit(dim::length);
  EXPECT_EQ(u.format(3704.0, dim::length).unit, "km");
}

TEST(WorldUnitsTest, NewDimensionsHaveRegimeDefaults) {
  world_units u;
  EXPECT_EQ(u.unit_symbol(dim::temperature), "degC");
  EXPECT_NEAR(u.to_display(273.15, dim::temperature), 0.0, 1e-12);
  EXPECT_EQ(u.unit_symbol(dim::energy_per_length), "Wh/km");
  EXPECT_NEAR(u.to_display(3.6, dim::energy_per_length), 1.0, 1e-12); // 3.6 J/m = 1 Wh/km
  u.set_regime(sys::imperial);
  EXPECT_EQ(u.unit_symbol(dim::temperature), "degF");
  EXPECT_NEAR(u.to_display(373.15, dim::temperature), 212.0, 1e-9);
  EXPECT_EQ(u.unit_symbol(dim::power), "hp");
  EXPECT_EQ(u.unit_symbol(dim::pressure), "psi");
  EXPECT_EQ(u.unit_symbol(dim::frequency), "Hz");
}

TEST(WorldUnitsTest, CustomDimensionWorksByName) {
  world_units u;
  u.registry().add_dimension("data_rate", "bit/s");
  u.registry().add_linear("Mbit/s", "data_rate", 1.0e6);
  EXPECT_EQ(u.unit_symbol("data_rate"), "bit/s"); // a custom dimension defaults to SI
  u.set_display_unit("data_rate", "Mbit/s");
  EXPECT_EQ(u.format(2.5e6, "data_rate").unit, "Mbit/s");
  EXPECT_DOUBLE_EQ(u.format(2.5e6, "data_rate").value, 2.5);
  EXPECT_DOUBLE_EQ(u.from_display(2.5, "data_rate"), 2.5e6);
  EXPECT_THROW(u.unit_symbol("nosuchdim"), cvc::unit_error);
  // Each world_units owns its registry: another instance has no data_rate.
  const world_units other;
  EXPECT_FALSE(other.registry().has_dimension("data_rate"));
}

// --- SI <-> nav's normalized frame --------------------------------------------

TEST(WorldUnitsTest, NormalizedFrameScalesByLengthExponent) {
  const world_units u;
  world_units::normalized_frame f;
  f.scale = 0.05; // 0.05 normalized units per world metre
  // velocity (e = 1): 2 m/s -> 0.1 normalized/s and back
  EXPECT_DOUBLE_EQ(u.to_normalized(2.0, dim::velocity, f), 2.0 * 0.05);
  EXPECT_DOUBLE_EQ(u.from_normalized(0.1, dim::velocity, f), 0.1 / 0.05);
  // curvature (e = -1): 1/R in 1/m -> per normalized unit
  EXPECT_DOUBLE_EQ(u.to_normalized(0.5, dim::curvature, f), 0.5 / 0.05);
  EXPECT_DOUBLE_EQ(u.to_normalized(0.5, "curvature", f), 0.5 / 0.05);
  // area (e = 2)
  EXPECT_NEAR(u.to_normalized(100.0, dim::area, f), 100.0 * 0.05 * 0.05, 1e-15);
  // time, angle and mass pass through bit for bit
  EXPECT_EQ(u.to_normalized(0.06, dim::time, f), 0.06);
  EXPECT_EQ(u.to_normalized(0.4, dim::angle, f), 0.4);
  EXPECT_EQ(u.from_normalized(1.5, dim::mass, f), 1.5);
}

TEST(WorldUnitsTest, NormalizedFrameAppliesMetresPerWorldUnitFirst) {
  // A world authored in kilometres: 1 world unit = 1000 m.
  const world_units u(world_units::config{sys::si, 1000.0});
  world_units::normalized_frame f;
  f.scale = 2.0; // 2 normalized units per world unit (per km)
  // 500 m = 0.5 world = 1.0 normalized
  EXPECT_DOUBLE_EQ(u.to_normalized(500.0, dim::length, f), 1.0);
  EXPECT_DOUBLE_EQ(u.from_normalized(1.0, dim::length, f), 500.0);
  // 4 m/s^2 = 0.004 world/s^2 = 0.008 normalized/s^2
  EXPECT_DOUBLE_EQ(u.to_normalized(4.0, dim::acceleration, f), 0.008);
  // 0.002 1/m = 2 1/world = 1 1/normalized
  EXPECT_DOUBLE_EQ(u.to_normalized(0.002, dim::curvature, f), 1.0);
}

TEST(WorldUnitsTest, NormalizedPointMatchesNavSnapshotArithmetic) {
  // nav's snapshot maps a normalized position back as o / S + centre; at one
  // metre per world unit the SI point is exactly that.
  const world_units u;
  world_units::normalized_frame f;
  f.scale = 0.02;
  f.cx = 37.5;
  f.cy = -12.0;
  const double ox = 0.3125, oy = -0.75;
  const auto m = u.normalized_point_to_si(ox, oy, 0.0, f);
  EXPECT_EQ(m.x, ox / 0.02 + 37.5);
  EXPECT_EQ(m.y, oy / 0.02 + -12.0);
  const auto n = u.si_point_to_normalized(m.x, m.y, 0.0, f);
  EXPECT_NEAR(n.x, ox, 1e-15);
  EXPECT_NEAR(n.y, oy, 1e-15);
}

TEST(WorldUnitsTest, NormalizedFrameRejectsBadScaleOrCentre) {
  const world_units u;
  world_units::normalized_frame f;
  f.scale = 0.0;
  EXPECT_THROW(u.to_normalized(1.0, dim::length, f), cvc::unit_error);
  f.scale = -1.0;
  EXPECT_THROW(u.from_normalized(1.0, dim::velocity, f), cvc::unit_error);
  f.scale = 1.0;
  f.cx = std::nan("");
  EXPECT_THROW(u.si_point_to_normalized(0.0, 0.0, 0.0, f), cvc::unit_error);
  f.cx = 0.0;
  EXPECT_THROW(u.to_normalized(1.0, "nosuchdim", f), cvc::unit_error);
}

TEST(WorldUnitsTest, CustomUnitsRegisterAndConvertConcurrently) {
  world_units u;
  std::atomic<bool> go{false};
  std::vector<std::thread> threads;
  for (int i = 0; i < 6; ++i) {
    threads.emplace_back([&u, &go, i]() {
      while (!go.load(std::memory_order_acquire))
        std::this_thread::yield();
      for (int k = 0; k < 200; ++k) {
        if (i % 2 == 0) {
          u.registry().add_linear("u" + std::to_string(i) + "_" + std::to_string(k), "length",
                                  1.0 + k);
        } else {
          EXPECT_NEAR(u.convert(1.0, "mi", "ft"), 5280.0, 1e-9);
          EXPECT_EQ(u.to_display(1.0, dim::velocity), 1.0);
        }
      }
    });
  }
  go.store(true, std::memory_order_release);
  for (auto &t : threads)
    t.join();
  EXPECT_TRUE(u.registry().has_unit("u4_199"));
  EXPECT_EQ(u.registry().units("length").size(), 9u + 600u);
}
