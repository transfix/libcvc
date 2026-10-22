/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Tests for cvc::clock_driver -- the one-advance-per-frame fan-out over a
// world_clock. The properties worth pinning are the ones a misordered or
// double-fired observer breaks silently: step observers all run before frame
// observers; the clock_frame every observer sees is the SAME reading of time;
// the clock advances exactly once per frame (never per observer); a measured
// first frame books no startup burst; and removing/adding an observer from
// inside the fan-out neither skips a live observer nor fires a new one early.

#include <cstdint>
#include <cvc/core/app.h>
#include <cvc/core/clock_driver.h>
#include <cvc/core/clock_follower.h>
#include <cvc/core/exception.h>
#include <cvc/core/world_clock.h>
#include <cvc/core/world_clock_binding.h>
#include <cvc/state/state.h>
#include <gtest/gtest.h>
#include <string>
#include <vector>

using cvc::clock_driver;
using cvc::world_clock;

namespace {
constexpr double k120 = 1.0 / 120.0;
} // namespace

// ── ordering: all step observers, then all frame observers ──────────────────

TEST(ClockDriverTest, FiresStepObserversThenFrameObserversEachInAscendingOrder) {
  world_clock c;
  clock_driver d(c);
  std::vector<std::string> log;

  // Register out of order and across kinds; a frame observer with a lower
  // `order` than a step observer must still fire AFTER it (kind wins).
  d.add_frame_observer(10, [&](const clock_driver::clock_frame &) { log.push_back("frame10"); });
  d.add_step_observer(5, [&](const clock_driver::clock_frame &) { log.push_back("step5"); });
  d.add_frame_observer(1, [&](const clock_driver::clock_frame &) { log.push_back("frame1"); });
  d.add_step_observer(2, [&](const clock_driver::clock_frame &) { log.push_back("step2"); });

  d.advance(k120);

  ASSERT_EQ(log.size(), 4u);
  EXPECT_EQ(log[0], "step2");
  EXPECT_EQ(log[1], "step5");
  EXPECT_EQ(log[2], "frame1");
  EXPECT_EQ(log[3], "frame10");
}

TEST(ClockDriverTest, TiesInOrderKeepRegistrationOrder) {
  world_clock c;
  clock_driver d(c);
  std::vector<std::string> log;
  d.add_frame_observer(0, [&](const clock_driver::clock_frame &) { log.push_back("a"); });
  d.add_frame_observer(0, [&](const clock_driver::clock_frame &) { log.push_back("b"); });
  d.add_frame_observer(0, [&](const clock_driver::clock_frame &) { log.push_back("c"); });
  d.advance(k120);
  ASSERT_EQ(log.size(), 3u);
  EXPECT_EQ(log[0], "a");
  EXPECT_EQ(log[1], "b");
  EXPECT_EQ(log[2], "c");
}

// ── one shared frame reading ────────────────────────────────────────────────

TEST(ClockDriverTest, EveryObserverSeesTheSameFrameAndTheClockAdvancesOnce) {
  world_clock c;
  clock_driver d(c);
  double t_a = -1.0, t_b = -1.0;
  std::uint64_t tick_a = 0, tick_b = 0;
  int calls_a = 0, calls_b = 0;

  d.add_frame_observer(0, [&](const clock_driver::clock_frame &f) {
    t_a = f.t;
    tick_a = f.tick;
    ++calls_a;
  });
  d.add_frame_observer(1, [&](const clock_driver::clock_frame &f) {
    t_b = f.t;
    tick_b = f.tick;
    ++calls_b;
  });

  const auto &f = d.advance(3.0 * k120);

  // Advanced exactly once: three whole quanta, not three-per-observer.
  EXPECT_EQ(f.step.steps, 3);
  EXPECT_EQ(f.tick, 3u);
  EXPECT_EQ(c.tick(), 3u);
  EXPECT_DOUBLE_EQ(f.t, 3.0 * k120);
  // Both observers fired once and read the identical time.
  EXPECT_EQ(calls_a, 1);
  EXPECT_EQ(calls_b, 1);
  EXPECT_DOUBLE_EQ(t_a, t_b);
  EXPECT_DOUBLE_EQ(t_a, f.t);
  EXPECT_EQ(tick_a, tick_b);
}

TEST(ClockDriverTest, FrameCarriesStepResultAndAlphaAndWallDt) {
  world_clock c;
  clock_driver d(c);
  const auto &f = d.advance(1.5 * k120); // one whole quantum, half-banked
  EXPECT_EQ(f.step.steps, 1);
  EXPECT_EQ(f.step.first_tick, 0u);
  EXPECT_NEAR(f.alpha, 0.5, 1e-9);
  EXPECT_DOUBLE_EQ(f.alpha, f.step.alpha);
  EXPECT_DOUBLE_EQ(f.fixed_dt, k120);
  EXPECT_DOUBLE_EQ(f.wall_dt, 1.5 * k120);
}

// ── first-frame seeding ─────────────────────────────────────────────────────

TEST(ClockDriverTest, MeasuredFirstAdvanceBooksNoTimeThenInjectedAdvancesNormally) {
  world_clock c;
  clock_driver d(c);
  int frames = 0;
  d.add_frame_observer(0, [&](const clock_driver::clock_frame &) { ++frames; });

  const auto &f0 = d.advance(); // wall_dt < 0 -> measure; first call seeds only
  EXPECT_TRUE(f0.first);
  EXPECT_EQ(f0.step.steps, 0);
  EXPECT_EQ(f0.tick, 0u);
  EXPECT_EQ(f0.index, 0u);
  EXPECT_EQ(frames, 1); // observers still fire on the seeding frame

  const auto &f1 = d.advance(2.0 * k120); // injected: advances normally
  EXPECT_FALSE(f1.first);
  EXPECT_EQ(f1.step.steps, 2);
  EXPECT_EQ(f1.tick, 2u);
  EXPECT_EQ(f1.index, 1u);
}

TEST(ClockDriverTest, InjectedDtIsHonoredOnTheFirstCallToo) {
  world_clock c;
  clock_driver d(c);
  const auto &f = d.advance(3.0 * k120);
  EXPECT_TRUE(f.first);
  EXPECT_EQ(f.step.steps, 3); // injected path does not suppress the first delta
}

// ── remove / add from inside the fan-out ────────────────────────────────────

TEST(ClockDriverTest, OneShotObserverRemovesItselfAndFiresExactlyOnce) {
  world_clock c;
  clock_driver d(c);
  int shots = 0, others = 0;
  clock_driver::handle h;
  h = d.add_frame_observer(0, [&](const clock_driver::clock_frame &) {
    ++shots;
    d.remove(h); // self-removal from inside the fan-out
  });
  d.add_frame_observer(1, [&](const clock_driver::clock_frame &) { ++others; });

  d.advance(k120);
  d.advance(k120);
  d.advance(k120);

  EXPECT_EQ(shots, 1);  // fired once, then gone
  EXPECT_EQ(others, 3); // the sibling keeps firing every frame
}

TEST(ClockDriverTest, RemovingASiblingDuringFireSkipsItThisFrame) {
  world_clock c;
  clock_driver d(c);
  int b_fires = 0;
  clock_driver::handle hb;
  // A (order 0) removes B (order 1) before B would fire this frame.
  d.add_frame_observer(0, [&](const clock_driver::clock_frame &) { d.remove(hb); });
  hb = d.add_frame_observer(1, [&](const clock_driver::clock_frame &) { ++b_fires; });

  d.advance(k120);
  EXPECT_EQ(b_fires, 0); // removed before its turn, never fires
  d.advance(k120);
  EXPECT_EQ(b_fires, 0); // stays gone
}

TEST(ClockDriverTest, AddingAnObserverDuringFireDefersItToTheNextFrame) {
  world_clock c;
  clock_driver d(c);
  int added_fires = 0;
  bool armed = true;
  d.add_frame_observer(0, [&](const clock_driver::clock_frame &) {
    if (armed) {
      armed = false;
      d.add_frame_observer(-100, // even a lower order must not jump into this frame
                           [&](const clock_driver::clock_frame &) { ++added_fires; });
    }
  });

  d.advance(k120);
  EXPECT_EQ(added_fires, 0); // not fired in the frame it was added
  d.advance(k120);
  EXPECT_EQ(added_fires, 1); // fires from the next frame on
}

// ── misuse ──────────────────────────────────────────────────────────────────

TEST(ClockDriverTest, ReenteringAdvanceFromAnObserverThrows) {
  world_clock c;
  clock_driver d(c);
  d.add_frame_observer(0, [&](const clock_driver::clock_frame &) { d.advance(k120); });
  EXPECT_THROW(d.advance(k120), cvc::exception);
  // The driver is left usable after the guarded throw.
  d.remove(clock_driver::handle{}); // no-op on a null handle
}

TEST(ClockDriverTest, RemoveReturnsFalseForAnUnknownOrNullHandle) {
  world_clock c;
  clock_driver d(c);
  EXPECT_FALSE(d.remove(clock_driver::handle{}));
  auto h = d.add_frame_observer(0, [](const clock_driver::clock_frame &) {});
  EXPECT_TRUE(d.remove(h));
  EXPECT_FALSE(d.remove(h)); // already gone
}

// ── step observers advance their own sim via a follower ─────────────────────

TEST(ClockDriverTest, StepObserverRunsDueQuantaViaFrameStep) {
  world_clock c; // 120 Hz quantum
  clock_driver d(c);
  // A sim stepping every 4 quanta: count the ticks it runs across frames.
  std::uint64_t sim_steps = 0;
  cvc::clock_follower follower(/*period=*/4);
  d.add_step_observer(
      0, [&](const clock_driver::clock_frame &f) { sim_steps += follower.take(f.tick); });

  d.advance(4.0 * k120); // 4 quanta -> exactly one due step
  EXPECT_EQ(sim_steps, 1u);
  d.advance(8.0 * k120); // 8 more quanta -> two due steps
  EXPECT_EQ(sim_steps, 3u);
}

// ── binding path: sim.* control steers the driver, telemetry is published ────

TEST(ClockDriverTest, BindingRoutesControlAndTelemetryThroughTheDriver) {
  cvc::app ctx;
  world_clock &wc = ctx.world_clock(); // per-app clock, 120 Hz / live
  cvc::world_clock_binding b(ctx, wc); // prefix "sim"
  clock_driver d(wc);
  d.set_binding(&b);

  d.advance(2.0 * k120);
  EXPECT_EQ(wc.tick(), 2u);
  // Telemetry was pushed to the tree.
  EXPECT_EQ(cvc::state::instance(ctx)("sim.tick").value(), std::string("2"));

  // Pause via the control key: pause wins, the driver books nothing.
  cvc::state::instance(ctx)("sim.paused").value(true);
  d.advance(5.0 * k120);
  EXPECT_EQ(wc.tick(), 2u);
}
