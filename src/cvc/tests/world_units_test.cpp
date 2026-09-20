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
#include <cvc/core/world_units.h>
#include <gtest/gtest.h>
#include <limits>
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
