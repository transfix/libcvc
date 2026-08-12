/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Tests for cvc::world_clock -- the fixed-quantum simulation clock.
//
// The properties worth pinning are the ones that fail *silently* if broken:
// world time drifting away from the tick count, banked time exploding after a
// stall, alpha leaving [0,1), and a paused clock releasing a burst on resume.

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cvc/core/exception.h>
#include <cvc/core/world_clock.h>
#include <gtest/gtest.h>
#include <limits>
#include <thread>
#include <vector>

using cvc::world_clock;

namespace {
constexpr double k120 = 1.0 / 120.0;
}

// ── construction ───────────────────────────────────────────────────────────

TEST(WorldClockTest, DefaultsTo120HzAndLiveMode) {
  world_clock c;
  EXPECT_DOUBLE_EQ(c.fixed_dt(), k120);
  EXPECT_EQ(c.current_mode(), world_clock::mode::live);
  EXPECT_EQ(c.tick(), 0u);
  EXPECT_DOUBLE_EQ(c.t(), 0.0);
}

TEST(WorldClockTest, RejectsANonPositiveOrNonFiniteQuantum) {
  world_clock::config bad;
  bad.fixed_dt = 0.0;
  EXPECT_THROW(world_clock{bad}, cvc::exception);
  bad.fixed_dt = -0.5;
  EXPECT_THROW(world_clock{bad}, cvc::exception);
  bad.fixed_dt = std::nan("");
  EXPECT_THROW(world_clock{bad}, cvc::exception);
}

TEST(WorldClockTest, RejectsAZeroStepClamp) {
  world_clock::config bad;
  bad.max_steps_per_advance = 0;
  EXPECT_THROW(world_clock{bad}, cvc::exception);
}

TEST(WorldClockTest, NegativeScaleClampsToPausedRatherThanRunningBackwards) {
  world_clock::config cfg;
  cfg.scale = -2.0;
  world_clock c{cfg};
  EXPECT_DOUBLE_EQ(c.scale(), 0.0);
  EXPECT_EQ(c.advance(1.0).steps, 0);
}

// ── the core stepping contract ─────────────────────────────────────────────

TEST(WorldClockTest, ConsumesWholeQuantaAndBanksTheRemainder) {
  world_clock c;                  // 120 Hz
  auto r = c.advance(1.0 / 60.0); // exactly two quanta
  EXPECT_EQ(r.steps, 2);
  EXPECT_EQ(c.tick(), 2u);
  EXPECT_NEAR(r.alpha, 0.0, 1e-12);

  r = c.advance(1.0 / 240.0); // half a quantum: no step, banked
  EXPECT_EQ(r.steps, 0);
  EXPECT_EQ(c.tick(), 2u);
  EXPECT_NEAR(r.alpha, 0.5, 1e-9);
}

TEST(WorldClockTest, AlphaAlwaysStaysInsideTheUnitInterval) {
  world_clock c;
  // Deliberately awkward deltas, including ones that land exactly on a
  // quantum boundary where naive code yields alpha == 1.0.
  const std::vector<double> deltas = {k120, k120 * 0.5,      k120 * 1.5, k120 * 2.0,
                                      0.0,  k120 * 0.999999, k120 * 3.7, 1e-9};
  for (double d : deltas) {
    auto r = c.advance(d);
    EXPECT_GE(r.alpha, 0.0) << "dt=" << d;
    EXPECT_LT(r.alpha, 1.0) << "dt=" << d;
  }
}

TEST(WorldClockTest, ManySmallDeltasAccumulateToTheRightTickCount) {
  // Frame-rate independence: 600 frames of 1/60 s must yield exactly the same
  // tick count as 300 frames of 1/30 s.
  world_clock a, b;
  for (int i = 0; i < 600; ++i)
    a.advance(1.0 / 60.0);
  for (int i = 0; i < 300; ++i)
    b.advance(1.0 / 30.0);
  EXPECT_EQ(a.tick(), b.tick());
  EXPECT_EQ(a.tick(), 1200u); // 10 s at 120 Hz
}

TEST(WorldClockTest, WorldTimeIsDerivedFromTheTickAndDoesNotDrift) {
  // The reason t() is computed rather than accumulated. Summing a float
  // quantum this many times drifts measurably; deriving it cannot.
  world_clock c;
  const std::uint64_t target = 1000000;
  c.seek_tick(target);
  EXPECT_DOUBLE_EQ(c.t(), static_cast<double>(target) * k120);

  double accumulated = 0.0;
  for (std::uint64_t i = 0; i < target; ++i)
    accumulated += k120;
  // Demonstrate the drift we are avoiding is real, then assert we avoided it.
  EXPECT_NE(accumulated, c.t());
  EXPECT_NEAR(accumulated, c.t(), 1e-3);
}

// ── the spiral-of-death clamp ──────────────────────────────────────────────

TEST(WorldClockTest, AStallIsClampedAndTheDroppedTimeIsReportedNotHidden) {
  world_clock::config cfg;
  cfg.max_steps_per_advance = 4;
  world_clock c{cfg};

  auto r = c.advance(1.0); // 120 quanta demanded, 4 allowed
  EXPECT_EQ(r.steps, 4);
  EXPECT_GT(r.dropped_steps, 0);
  EXPECT_EQ(r.steps + r.dropped_steps, 120);
  EXPECT_EQ(c.tick(), 4u);
  EXPECT_EQ(c.total_dropped(), static_cast<std::uint64_t>(r.dropped_steps));
}

TEST(WorldClockTest, EveryDemandedQuantumIsEitherRunOrReported) {
  // The silent-loss bug: (acc - remainder)/fixed_dt is mathematically an
  // integer, but each operation carries an ulp, so truncating the quotient
  // lands one BELOW it for a couple of percent of accumulators -- and that
  // quantum vanishes, neither simulated nor counted. Sweep deltas with
  // uncooperative remainders so the invariant is pinned, not lucked into.
  for (int i = 1; i <= 400; ++i) {
    world_clock::config cfg;
    cfg.max_steps_per_advance = 3;
    world_clock c{cfg};

    const double dt = 0.37 * static_cast<double>(i) + 0.0123456789;
    const auto r = c.advance(dt);
    const std::uint64_t demanded = static_cast<std::uint64_t>(std::floor(dt / k120));
    EXPECT_EQ(static_cast<std::uint64_t>(r.steps) + r.dropped_steps, demanded)
        << "lost a quantum at dt=" << dt;
    EXPECT_LT(c.pending_seconds(), c.fixed_dt());
    EXPECT_GE(c.pending_seconds(), 0.0);
  }
}

TEST(WorldClockTest, TotalDroppedAccumulatesAcrossAdvances) {
  // A regression turning `_dropped +=` into `_dropped =` must not pass.
  world_clock::config cfg;
  cfg.max_steps_per_advance = 4;
  world_clock c{cfg};

  const auto a = c.advance(1.0);
  const auto b = c.advance(1.0);
  EXPECT_EQ(c.total_dropped(), a.dropped_steps + b.dropped_steps);
  EXPECT_EQ(c.total_dropped(), 232u); // (120-4) twice
}

TEST(WorldClockTest, AnAbsurdDeltaIsClampedNotUndefined) {
  // Passing an absolute timestamp where a delta was meant is the classic
  // first-frame consumer bug. The old code cast the quotient to int: at these
  // magnitudes that is undefined behaviour, and in practice it reported a
  // NEGATIVE drop count that wrapped total_dropped() to ~1.8e19.
  world_clock c;
  const auto r = c.advance(1.7e9); // seconds since the epoch
  EXPECT_EQ(r.steps, 8);
  EXPECT_GT(r.dropped_steps, 0u);
  EXPECT_EQ(static_cast<std::uint64_t>(r.steps) + r.dropped_steps,
            static_cast<std::uint64_t>(std::floor(1.7e9 / k120)));
  EXPECT_EQ(c.total_dropped(), r.dropped_steps);
  EXPECT_LT(c.pending_seconds(), c.fixed_dt());

  // ...and the clock still works afterwards.
  EXPECT_EQ(c.advance(k120).steps, 1);
}

TEST(WorldClockTest, AFiniteDeltaTimesAFiniteScaleCannotBrickTheClock) {
  // Both values are individually finite and accepted, but their product
  // rounds to +inf; inf reaching the accumulator makes fmod() NaN, and a NaN
  // accumulator never satisfies `>= fixed_dt` again -- the clock stops
  // stepping forever while reporting nothing wrong.
  world_clock c;
  c.set_scale(1e200);
  c.advance(1e200);
  EXPECT_TRUE(std::isfinite(c.pending_seconds()));
  EXPECT_LT(c.pending_seconds(), c.fixed_dt());

  c.set_scale(1.0);
  EXPECT_EQ(c.advance(k120).steps, 1) << "clock was bricked by an overflowing product";
}

TEST(WorldClockTest, AlphaStaysInRangeForEveryReachableInput) {
  world_clock c;
  const double deltas[] = {0.0, 1e-12, k120 * 0.5, k120, k120 * 1.5, 1.0,
                           1e9, 1e300, -1.0,       1e15, 1e18};
  for (double d : deltas) {
    const auto r = c.advance(d);
    EXPECT_GE(r.alpha, 0.0) << "dt=" << d;
    EXPECT_LT(r.alpha, 1.0) << "dt=" << d;
    EXPECT_FALSE(std::isnan(r.alpha)) << "dt=" << d;
  }
}

TEST(WorldClockTest, TheClampDoesNotLeaveTimeBankedForTheNextFrame) {
  // The failure this guards: clamping the steps but keeping the unspent
  // seconds, so every subsequent frame is also over budget and the clock
  // never catches up -- the spiral the clamp exists to prevent.
  world_clock::config cfg;
  cfg.max_steps_per_advance = 4;
  world_clock c{cfg};

  c.advance(1.0);
  EXPECT_LT(c.pending_seconds(), c.fixed_dt());

  auto r = c.advance(k120); // a normal frame afterwards
  EXPECT_EQ(r.steps, 1);
  EXPECT_EQ(r.dropped_steps, 0);
}

TEST(WorldClockTest, NonFiniteAndNegativeDeltasAreIgnored) {
  world_clock c;
  EXPECT_EQ(c.advance(std::nan("")).steps, 0);
  EXPECT_EQ(c.advance(-5.0).steps, 0);
  EXPECT_EQ(c.advance(std::numeric_limits<double>::infinity()).steps, 0);
  EXPECT_EQ(c.tick(), 0u);
  EXPECT_EQ(c.total_dropped(), 0u);
}

// ── scale ──────────────────────────────────────────────────────────────────

TEST(WorldClockTest, ScaleChangesTheRateNotTheQuantum) {
  world_clock::config cfg;
  cfg.scale = 2.0;
  world_clock c{cfg};
  auto r = c.advance(1.0 / 60.0); // 2x -> four quanta of world time
  EXPECT_EQ(r.steps, 4);
  EXPECT_DOUBLE_EQ(c.fixed_dt(), k120); // quantum is unchanged
}

TEST(WorldClockTest, ScaleAppliesFromTheMomentItChanges) {
  // The banked remainder was accumulated at the OLD rate and must not be
  // retroactively rescaled; only subsequent wall time sees the new rate.
  world_clock c;
  c.advance(k120 * 0.5); // bank half a quantum at 1x
  ASSERT_EQ(c.tick(), 0u);
  c.set_scale(2.0);
  const auto r = c.advance(k120 * 0.75); // 1.5 quanta of world time -> 0.5+1.5
  EXPECT_EQ(r.steps, 2);
  EXPECT_NEAR(c.pending_seconds(), 0.0, 1e-12);
}

TEST(WorldClockTest, TheSetterClampsANegativeOrNonFiniteScale) {
  world_clock c;
  c.set_scale(-1.0);
  EXPECT_DOUBLE_EQ(c.scale(), 0.0);
  c.set_scale(std::nan(""));
  EXPECT_DOUBLE_EQ(c.scale(), 0.0);
  c.set_scale(std::numeric_limits<double>::infinity());
  EXPECT_DOUBLE_EQ(c.scale(), 0.0);
  // and a clock whose scale was poisoned still runs once it is set sanely
  c.set_scale(1.0);
  EXPECT_EQ(c.advance(k120).steps, 1);
}

TEST(WorldClockTest, RejectsANonFiniteScaleAtConstruction) {
  // Unlike the setter (a slider must not crash a renderer), construction is a
  // programming error: a NaN scale poisons the accumulator on the first frame
  // and the clock silently never advances again.
  world_clock::config cfg;
  cfg.scale = std::nan("");
  EXPECT_THROW(world_clock{cfg}, cvc::exception);
  cfg.scale = std::numeric_limits<double>::infinity();
  EXPECT_THROW(world_clock{cfg}, cvc::exception);
}

TEST(WorldClockTest, ScaleZeroPausesWithoutBanking) {
  world_clock c;
  c.set_scale(0.0);
  c.advance(10.0);
  EXPECT_EQ(c.tick(), 0u);
  EXPECT_DOUBLE_EQ(c.pending_seconds(), 0.0);
}

// ── modes ──────────────────────────────────────────────────────────────────

TEST(WorldClockTest, PausedYieldsNoStepsAndBanksNothing) {
  world_clock c;
  c.set_mode(world_clock::mode::paused);
  auto r = c.advance(5.0);
  EXPECT_EQ(r.steps, 0);
  EXPECT_EQ(c.tick(), 0u);
  EXPECT_DOUBLE_EQ(c.pending_seconds(), 0.0);
}

TEST(WorldClockTest, ResumingFromPauseDoesNotReleaseABurst) {
  // The bug this pins: banking wall time while paused, then dumping it all as
  // steps on the first live frame -- the scene lurches on unpause.
  world_clock c;
  c.advance(k120 * 0.75); // bank most of a quantum legitimately
  c.set_mode(world_clock::mode::paused);
  c.advance(30.0); // a long pause
  c.set_mode(world_clock::mode::live);
  auto r = c.advance(k120);
  EXPECT_LE(r.steps, 1);
}

TEST(WorldClockTest, ReturningToLiveClearsTheBankButStayingLiveDoesNot) {
  world_clock c;
  c.advance(k120 * 0.9); // bank most of a quantum
  ASSERT_NEAR(c.pending_seconds(), k120 * 0.9, 1e-12);

  c.set_mode(world_clock::mode::live); // already live: must NOT clear
  EXPECT_NEAR(c.pending_seconds(), k120 * 0.9, 1e-12);

  c.set_mode(world_clock::mode::paused);
  c.set_mode(world_clock::mode::live); // a real transition: clears
  EXPECT_DOUBLE_EQ(c.pending_seconds(), 0.0);
  EXPECT_EQ(c.advance(k120 * 0.5).steps, 0) << "released banked time as a step";
}

TEST(WorldClockTest, ReplayAndSteppingBankNoWallTimeEither) {
  // Only `paused` had this pinned; an early return that still accumulated
  // would have slipped through for the other two.
  for (auto m : {world_clock::mode::replay, world_clock::mode::stepping}) {
    world_clock c;
    c.set_mode(m);
    c.advance(10.0);
    EXPECT_DOUBLE_EQ(c.pending_seconds(), 0.0);
    EXPECT_EQ(c.tick(), 0u);
  }
}

TEST(WorldClockTest, AlphaSurvivesAPauseAndAStepInsteadOfJumpingBackwards) {
  // alpha is what the renderer interpolates with: it must reflect the banked
  // remainder in every mode, or pausing (or stepping) makes the scene jerk.
  world_clock c;
  c.advance(k120 * 0.5);
  const double live_alpha = c.advance(0.0).alpha;
  ASSERT_NEAR(live_alpha, 0.5, 1e-9);

  c.set_mode(world_clock::mode::paused);
  EXPECT_NEAR(c.advance(5.0).alpha, live_alpha, 1e-12);

  c.set_mode(world_clock::mode::stepping);
  const auto r = c.step_once();
  EXPECT_EQ(r.steps, 1);
  EXPECT_NEAR(r.alpha, live_alpha, 1e-12) << "step_once jerked alpha backwards";
  EXPECT_NEAR(c.pending_seconds(), k120 * 0.5, 1e-12) << "step_once ate the bank";
}

TEST(WorldClockTest, SeekPreservesTheBankedRemainder) {
  world_clock c;
  c.advance(k120 * 0.5);
  const double banked = c.pending_seconds();
  c.set_mode(world_clock::mode::replay);
  c.seek_tick(100);
  EXPECT_EQ(c.tick(), 100u);
  EXPECT_DOUBLE_EQ(c.pending_seconds(), banked);
  EXPECT_NEAR(c.advance(1.0).alpha, 0.5, 1e-9);
}

TEST(WorldClockTest, SteppingIgnoresWallTimeAndAdvancesExactlyOnce) {
  world_clock c;
  c.set_mode(world_clock::mode::stepping);
  EXPECT_EQ(c.advance(10.0).steps, 0);
  EXPECT_EQ(c.tick(), 0u);

  auto r = c.step_once();
  EXPECT_EQ(r.steps, 1);
  EXPECT_EQ(c.tick(), 1u);
  EXPECT_DOUBLE_EQ(c.t(), k120);
}

TEST(WorldClockTest, ReplayIsDrivenBySeekNotByWallTime) {
  world_clock c;
  c.set_mode(world_clock::mode::replay);
  EXPECT_EQ(c.advance(1.0).steps, 0);

  c.seek_tick(4242);
  EXPECT_EQ(c.tick(), 4242u);
  EXPECT_DOUBLE_EQ(c.t(), 4242.0 * k120);
}

TEST(WorldClockTest, ResetReturnsToZeroAndClearsDroppedCount) {
  world_clock::config cfg;
  cfg.max_steps_per_advance = 2;
  world_clock c{cfg};
  c.advance(1.0);
  ASSERT_GT(c.total_dropped(), 0u);

  c.set_mode(world_clock::mode::replay);
  c.set_scale(2.0);

  c.reset();
  EXPECT_EQ(c.tick(), 0u);
  EXPECT_EQ(c.total_dropped(), 0u);
  EXPECT_DOUBLE_EQ(c.pending_seconds(), 0.0);
  // ...but reset() is a rewind, not a factory reset: mode and rate persist.
  EXPECT_EQ(c.current_mode(), world_clock::mode::replay);
  EXPECT_DOUBLE_EQ(c.scale(), 2.0);
  EXPECT_DOUBLE_EQ(c.fixed_dt(), k120);
}

// ── determinism, which is the whole point ──────────────────────────────────

TEST(WorldClockTest, TheSameDeltaSequenceReproducesTheSameTicksExactly) {
  // A fast machine and a slow one differ in wall dt, not in outcome: what a
  // replay and a benchmark both depend on.
  const std::vector<double> jittery = {0.016, 0.031, 0.008, 0.0, 0.021, 0.099, 0.004, 0.017, 0.012};
  auto run = [&] {
    world_clock c;
    std::vector<int> steps;
    for (double d : jittery)
      steps.push_back(c.advance(d).steps);
    return std::make_pair(c.tick(), steps);
  };
  const auto a = run();
  const auto b = run();
  EXPECT_EQ(a.first, b.first);
  EXPECT_EQ(a.second, b.second);
}

TEST(WorldClockTest, TotalStepsMatchElapsedWorldTimeWhenNothingIsDropped) {
  world_clock c;
  int total = 0;
  for (int i = 0; i < 1000; ++i)
    total += c.advance(0.01).steps; // 10 s of wall time
  EXPECT_EQ(c.total_dropped(), 0u);
  EXPECT_EQ(static_cast<std::uint64_t>(total), c.tick());
  EXPECT_NEAR(c.t(), 10.0, k120); // within one quantum
}

// ── thread safety ──────────────────────────────────────────────────────────

TEST(WorldClockTest, ConcurrentReadersDoNotTearOrCrash) {
  world_clock c;
  std::atomic<bool> stop{false};
  std::thread reader([&] {
    while (!stop.load()) {
      (void)c.tick();
      (void)c.t();
      (void)c.pending_seconds();
    }
  });
  for (int i = 0; i < 20000; ++i)
    c.advance(0.001);
  stop.store(true);
  reader.join();
  EXPECT_GT(c.tick(), 0u);
}
