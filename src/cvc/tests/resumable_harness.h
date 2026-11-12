/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The T7 harness (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.7, §1.9): what every resumable
// stepper's tests check, in one call. A port's T7 test names its stepper, how to take its result,
// how to compare two results bit for bit, and (optionally) its blocking function:
//
//   cvc::test::t7_subject<cvc::contour_tree_builder, cvc::contour_tree> s;
//   s.name = "contour tree 24^3";
//   s.make = [&] { return std::make_unique<cvc::contour_tree_builder>(vol); };
//   s.take = [](cvc::contour_tree_builder &b) { return b.take(); };
//   s.equal = same_tree;
//   s.blocking = [&] { return cvc::compute_contour_tree(vol); };
//   const cvc::test::t7_report r = cvc::test::run_t7(s);
//
// It checks, with googletest's EXPECT_*:
//   1. results: the stepper run at budgets 1, 64, a seeded random budget per step and 0 gives the
//      blocking function's result, bit for bit (`equal`);
//   2. progress: in [0, 1], never falling, and every step's phase named (a non-empty string);
//   3. cancel: cancelling the stepper's result_job in each phase it passes through (once, after its
//      first step, for a stepper with no phase()) ends the job at its next step with no result,
//      and destroys the stepper there (no leak);
//   4. timing: the slowest step at the default budget (4096 units), and stepping's overhead over
//      budget 0, recorded (googletest's RecordProperty and a printed line); with a bound set, the
//      slowest step must stay under it;
//   5. frames: the job run as a cooperative sliced task on an app's scheduler (as
//      CVC_SLICED_THREADING=cooperative and single-threaded wasm run it), pumped a frame at a time:
//      the frames keep coming while it runs, the slowest frame is recorded (and bounded, with a
//      bound set), and the result is the same.
//
// Timing bounds are off by default (0): sanitizer and Debug builds are many times slower, so a
// port sets its bounds for the builds that measure them. Header-only, for tests.

#ifndef CVC_TESTS_RESUMABLE_HARNESS_H
#define CVC_TESTS_RESUMABLE_HARNESS_H

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cvc/core/app.h>
#include <cvc/core/result_job.h>
#include <cvc/core/sliced_task.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <functional>
#include <gtest/gtest.h>
#include <iostream>
#include <memory>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace cvc::test {

template <class S, class R> struct t7_subject {
  std::string name;
  std::function<std::unique_ptr<S>()> make;        // a fresh stepper over the input
  std::function<R(S &)> take;                      // the result of a finished stepper
  std::function<bool(const R &, const R &)> equal; // bit for bit
  std::function<R()> blocking;                     // the blocking function (optional)
};

struct t7_options {
  std::vector<std::size_t> budgets{1, 64, 0}; // fixed budgets (0: to the end in one step)
  bool random_budgets = true;                 // and a seeded random budget per step
  std::uint32_t seed = 20261109;
  std::size_t random_max = 4096;     // random budgets are 1 .. random_max
  std::size_t default_budget = 4096; // the budget the slowest step is recorded at
  double max_step_ms = 0.0;          // > 0: the slowest step's bound
  double slice_ms = 4.0;             // the cooperative run's slice
  double max_frame_ms = 0.0;         // > 0: the slowest frame's bound
  bool cancel_each_phase = true;
  bool frame_run = true;
  double frame_run_timeout_s = 120.0;
};

struct t7_report {
  std::vector<std::string> phases; // in the order the stepper went through them
  std::size_t steps_at_budget_1 = 0;
  double slowest_step_ms = 0.0; // at default_budget
  double budget0_ms = 0.0;      // one step of budget 0
  double stepped_ms = 0.0;      // all the steps at default_budget
  std::size_t frames = 0;       // the cooperative run's frames while the job ran
  double slowest_frame_ms = 0.0;
};

namespace detail {

template <class S>
concept named_phases = requires(const S &s) {
  { s.phase() } -> std::convertible_to<const char *>;
};

inline double ms_since(std::chrono::steady_clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

// A stepper that counts itself alive, so a test can see it destroyed.
template <class S> class tracked {
public:
  tracked(std::unique_ptr<S> s, std::shared_ptr<std::atomic<int>> live)
      : _s(std::move(s)), _live(std::move(live)) {
    ++*_live;
  }
  ~tracked() { --*_live; }
  tracked(const tracked &) = delete;
  tracked &operator=(const tracked &) = delete;
  bool step(std::size_t b) { return _s->step(b); }
  double progress() const { return _s->progress(); }
  const char *phase() const
    requires named_phases<S>
  {
    return _s->phase();
  }
  S &inner() { return *_s; }

private:
  std::unique_ptr<S> _s;
  std::shared_ptr<std::atomic<int>> _live;
};

// Run a stepper to the end with budgets from `next`; check progress and phases; return its result.
template <class S, class R, class Next>
R run_out(const t7_subject<S, R> &s, Next next, const std::string &how, std::size_t *steps,
          std::vector<std::string> *phases) {
  std::unique_ptr<S> st = s.make();
  double last = 0.0;
  std::size_t n = 0;
  bool done = false;
  const std::size_t guard = std::size_t(1) << 32; // no stepper this harness runs takes this many
  while (!done) {
    if constexpr (named_phases<S>) {
      const char *ph = st->phase();
      EXPECT_TRUE(ph && *ph) << s.name << ", " << how << ": an unnamed phase at step " << n;
      if (phases && ph && (phases->empty() || phases->back() != ph))
        phases->push_back(ph);
    }
    done = st->step(next());
    ++n;
    const double p = st->progress();
    EXPECT_GE(p, 0.0) << s.name << ", " << how << ": progress below 0 at step " << n;
    EXPECT_LE(p, 1.0) << s.name << ", " << how << ": progress above 1 at step " << n;
    EXPECT_GE(p, last) << s.name << ", " << how << ": progress fell at step " << n;
    last = std::max(last, p);
    if (n >= guard) {
      ADD_FAILURE() << s.name << ", " << how << ": never finished";
      break;
    }
  }
  EXPECT_DOUBLE_EQ(st->progress(), 1.0) << s.name << ", " << how << ": progress once done";
  if (steps)
    *steps = n;
  return s.take(*st);
}

// One frame, as an Ariadne Runtime pumps the app's scheduler (cvc::wait_sliced_job's loop body).
inline void pump_frame(cvc::app &a) {
  auto &sched = a.exec_scheduler();
  sched.drain_ingress();
  sched.wake_awaiting();
  sched.sync_step();
  sched.sync_run(200000, 0.1);
}

} // namespace detail

template <class S, class R>
t7_report run_t7(const t7_subject<S, R> &s, const t7_options &opt = {}) {
  t7_report rep;
  // The reference: the blocking function if there is one, else the stepper at budget 0.
  std::optional<R> ref;
  if (s.blocking)
    ref.emplace(s.blocking());

  // 1-2. Every budget gives the same result, with monotone progress and named phases.
  for (const std::size_t b : opt.budgets) {
    const std::string how = "budget " + std::to_string(b);
    std::size_t steps = 0;
    std::vector<std::string> phases;
    const auto t0 = std::chrono::steady_clock::now();
    R got = detail::run_out(s, [b] { return b; }, how, &steps, &phases);
    if (b == 0) {
      rep.budget0_ms = detail::ms_since(t0);
      EXPECT_EQ(steps, 1u) << s.name << ": budget 0 runs to the end in one step";
    }
    if (b == 1) {
      rep.steps_at_budget_1 = steps;
      rep.phases = phases;
    }
    if (!ref)
      ref.emplace(std::move(got));
    else
      EXPECT_TRUE(s.equal(*ref, got)) << s.name << ": " << how << " differs from the reference";
  }
  if (opt.random_budgets) {
    std::mt19937 rng(opt.seed);
    std::uniform_int_distribution<std::size_t> d(1, std::max<std::size_t>(1, opt.random_max));
    R got = detail::run_out(s, [&] { return d(rng); }, "random budgets", nullptr, nullptr);
    EXPECT_TRUE(s.equal(*ref, got)) << s.name << ": random budgets differ from the reference";
  }
  if (rep.phases.empty()) { // budget 1 was not run: learn the phases at 64
    std::vector<std::string> phases;
    detail::run_out(s, [] { return std::size_t(64); }, "phases", nullptr, &phases);
    rep.phases = phases;
  }

  // 4. The slowest step at the default budget, and the overhead over budget 0.
  {
    std::unique_ptr<S> st = s.make();
    bool done = false;
    while (!done) {
      const auto t0 = std::chrono::steady_clock::now();
      done = st->step(opt.default_budget);
      const double ms = detail::ms_since(t0);
      rep.stepped_ms += ms;
      rep.slowest_step_ms = std::max(rep.slowest_step_ms, ms);
    }
    R got = s.take(*st);
    EXPECT_TRUE(s.equal(*ref, got)) << s.name << ": the timed run differs";
  }
  const double overhead =
      rep.budget0_ms > 0.0 ? 100.0 * (rep.stepped_ms / rep.budget0_ms - 1.0) : 0.0;
  ::testing::Test::RecordProperty("t7_slowest_step_ms", std::to_string(rep.slowest_step_ms));
  ::testing::Test::RecordProperty("t7_overhead_pct", std::to_string(overhead));
  std::cout << "[ T7 ] " << s.name << ": slowest step at budget " << opt.default_budget << " "
            << rep.slowest_step_ms << " ms; stepping " << rep.stepped_ms << " ms vs budget 0 "
            << rep.budget0_ms << " ms (" << overhead << "%)\n";
  if (opt.max_step_ms > 0.0)
    EXPECT_LE(rep.slowest_step_ms, opt.max_step_ms) << s.name << ": a step overran its bound";

  // 3. A cancel in each phase: no result, and the stepper destroyed at the next step.
  if (opt.cancel_each_phase) {
    // A stepper with no phase() is cancelled once, after its first step ("").
    const std::vector<std::string> points =
        rep.phases.empty() ? std::vector<std::string>{std::string()} : rep.phases;
    for (const std::string &phase : points) {
      auto live = std::make_shared<std::atomic<int>>(0);
      auto take = s.take;
      std::unique_ptr<result_job<R>> job =
          make_result_job<R>(std::make_unique<detail::tracked<S>>(s.make(), live),
                             [take](detail::tracked<S> &t) { return take(t.inner()); });
      EXPECT_EQ(live->load(), 1);
      // Step until the job's next step works on `phase` (one step at least, so the stepper has
      // started and holds its working state).
      bool reached = false;
      do {
        job->step(1);
        reached = !job->done() && (phase.empty() || phase == job->phase());
      } while (!reached && !job->done());
      if (!reached) {
        // A phase the stepper only passes through inside one unit: nothing to cancel in.
        continue;
      }
      job->cancel();
      EXPECT_TRUE(job->done()) << s.name << ": done at once after a cancel in " << phase;
      EXPECT_TRUE(job->step(64)) << s.name << ": the step after a cancel in " << phase
                                 << " ends it";
      EXPECT_TRUE(job->cancelled()) << phase;
      EXPECT_FALSE(job->has_result()) << s.name << ": a result after a cancel in " << phase;
      EXPECT_FALSE(job->failed()) << phase;
      EXPECT_STREQ(job->phase(), "cancelled");
      EXPECT_EQ(job->status(), "cancelled");
      EXPECT_THROW((void)job->result(), result_job_error) << phase;
      EXPECT_EQ(live->load(), 0) << s.name << ": the stepper outlived a cancel in " << phase;
    }
  }

  // 5. The cooperative frame-time run.
  if (opt.frame_run) {
    cvc::app a;
    auto take = s.take;
    std::shared_ptr<result_job<R>> job(
        make_result_job<R>(s.make(), [take](S &st) { return take(st); }));
    std::optional<R> got;
    cvc::sliced_task_options o;
    o.kind = "t7";
    o.subject = s.name;
    o.threading = cvc::sliced_threading::cooperative;
    o.slice_ms = opt.slice_ms;
    o.on_done = [&](cvc::sliced_job &j) {
      got.emplace(static_cast<result_job<R> &>(j).take_result());
    };
    std::string why;
    const int pid = cvc::schedule_sliced_job(a, job, o, &why);
    EXPECT_GE(pid, 0) << s.name << ": " << why;
    const auto start = std::chrono::steady_clock::now();
    while (pid >= 0 && cvc::sliced_job_running(a, pid)) {
      const auto t0 = std::chrono::steady_clock::now();
      detail::pump_frame(a);
      const double ms = detail::ms_since(t0);
      ++rep.frames;
      rep.slowest_frame_ms = std::max(rep.slowest_frame_ms, ms);
      if (detail::ms_since(start) > 1000.0 * opt.frame_run_timeout_s) {
        ADD_FAILURE() << s.name << ": the cooperative run did not finish";
        cvc::cancel_sliced_job(a, pid);
        break;
      }
    }
    ::testing::Test::RecordProperty("t7_frames", std::to_string(rep.frames));
    ::testing::Test::RecordProperty("t7_slowest_frame_ms", std::to_string(rep.slowest_frame_ms));
    std::cout << "[ T7 ] " << s.name << ": cooperative run " << rep.frames << " frames, slowest "
              << rep.slowest_frame_ms << " ms (slice " << opt.slice_ms << " ms)\n";
    EXPECT_FALSE(job->failed()) << s.name << ": " << job->error();
    EXPECT_TRUE(got.has_value()) << s.name << ": the cooperative run gave no result";
    if (got)
      EXPECT_TRUE(s.equal(*ref, *got)) << s.name << ": the cooperative run differs";
    EXPECT_GE(rep.frames, 2u) << s.name << ": the job and its finish each get a frame";
    if (opt.max_frame_ms > 0.0)
      EXPECT_LE(rep.slowest_frame_ms, opt.max_frame_ms) << s.name << ": a frame overran";
  }
  return rep;
}

} // namespace cvc::test

#endif // CVC_TESTS_RESUMABLE_HARNESS_H
