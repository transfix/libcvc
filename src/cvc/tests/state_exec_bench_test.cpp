/*
  Copyright 2026 The University of Texas at Austin

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// state_exec micro-benchmarks.
// Gated on env var CVC_STATE_EXEC_BENCH=1 so they only run on demand.
// When the env var is not set, every test is a SUCCEED no-op so the
// suite stays green in normal CI runs.
//
// Output format (parseable):
//   BENCH <name> N=<count> wall_ns_total=<n> wall_ns_per_op=<n>
//
// Three benchmark categories:
//   1. Performance — evaluator throughput (steps/sec), parse throughput
//   2. Memory-pressure — scheduler under memory-limited processes
//   3. Message-flood — high-concurrency scheduler stress

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/state_exec/builtins.h>
#include <cvc/core/state_exec/parser.h>
#include <cvc/core/state_exec/scheduler.h>
#include <cvc/core/state_exec/stackless_evaluator.h>
#include <cvc/core/state_exec/types.h>
#include <gtest/gtest.h>
#include <string>

namespace {

bool bench_enabled() {
  const char *v = std::getenv("CVC_STATE_EXEC_BENCH");
  return v && std::string(v) == "1";
}

using hrclock = std::chrono::high_resolution_clock;
using ns = std::chrono::nanoseconds;

void report(const char *name, int64_t n, ns total) {
  auto t = total.count();
  auto per = n > 0 ? t / n : 0;
  std::printf("BENCH %s N=%lld wall_ns_total=%lld wall_ns_per_op=%lld\n", name, (long long)n,
              (long long)t, (long long)per);
}

using namespace cvc::state_exec;

} // namespace

// -----------------------------------------------------------------------
// 1. Performance benchmarks
// -----------------------------------------------------------------------

class ExecBenchTest : public ::testing::Test {};

TEST_F(ExecBenchTest, ParseThroughput) {
  if (!bench_enabled()) {
    SUCCEED();
    return;
  }

  const std::string src = "(begin "
                          "  (defun fib (n) "
                          "    (if (<= n 1) n "
                          "      (+ (fib (- n 1)) (fib (- n 2)))))"
                          "  (fib 20))";

  constexpr int N = 10000;
  auto t0 = hrclock::now();
  for (int i = 0; i < N; ++i) {
    auto ast = parse(src);
    (void)ast;
  }
  auto t1 = hrclock::now();
  report("parse_fib20_expr", N, std::chrono::duration_cast<ns>(t1 - t0));
}

TEST_F(ExecBenchTest, EvalArithmeticLoop) {
  if (!bench_enabled()) {
    SUCCEED();
    return;
  }

  // Tight arithmetic loop: sum 0..999
  const std::string src = "(begin "
                          "  (set sum 0) "
                          "  (set i 0) "
                          "  (while (< i 1000) "
                          "    (begin (set sum (+ sum i)) (set i (+ i 1)))) "
                          "  sum)";

  auto env = builtins::make_default_environment();

  constexpr int N = 100;
  auto t0 = hrclock::now();
  for (int i = 0; i < N; ++i) {
    stackless_evaluator ev(env);
    ev.evaluate_script(src);
  }
  auto t1 = hrclock::now();
  report("eval_sum_0_999", N, std::chrono::duration_cast<ns>(t1 - t0));
}

TEST_F(ExecBenchTest, EvalFibonacci) {
  if (!bench_enabled()) {
    SUCCEED();
    return;
  }

  // Recursive fib(15) — exercises call overhead
  const std::string src = "(begin "
                          "  (defun fib (n) "
                          "    (if (<= n 1) n "
                          "      (+ (fib (- n 1)) (fib (- n 2)))))"
                          "  (fib 15))";

  auto env = builtins::make_default_environment();

  constexpr int N = 50;
  auto t0 = hrclock::now();
  for (int i = 0; i < N; ++i) {
    stackless_evaluator ev(env);
    ev.evaluate_script(src);
  }
  auto t1 = hrclock::now();
  report("eval_fib15", N, std::chrono::duration_cast<ns>(t1 - t0));
}

TEST_F(ExecBenchTest, SchedulerThroughput) {
  if (!bench_enabled()) {
    SUCCEED();
    return;
  }

  constexpr int NUM_PROCS = 500;
  const std::string src = "(begin (set x 0) (set i 0) "
                          "(while (< i 100) "
                          "  (begin (set x (+ x i)) (set i (+ i 1)))) "
                          "x)";

  scheduler sched(scheduling_policy::round_robin);

  for (int i = 0; i < NUM_PROCS; ++i) {
    execute_options opts;
    opts.max_steps = 100000;
    opts.env = builtins::make_default_environment();
    sched.execute(src, opts);
  }

  auto t0 = hrclock::now();
  sched.run();
  auto t1 = hrclock::now();

  auto stats = sched.get_stats();
  report("scheduler_500_procs", NUM_PROCS, std::chrono::duration_cast<ns>(t1 - t0));
  std::printf("  total_steps=%llu terminated=%d\n", (unsigned long long)stats.total_steps,
              stats.terminated);
}

// -----------------------------------------------------------------------
// 2. Memory-pressure stress tests
// -----------------------------------------------------------------------

TEST_F(ExecBenchTest, MemoryPressureEnforcement) {
  if (!bench_enabled()) {
    SUCCEED();
    return;
  }

  constexpr int NUM_PROCS = 200;

  // Each process runs a tight loop that accumulates values until
  // hitting max_steps (memory tracking is internal to the scheduler).
  const std::string src = "(begin "
                          "  (set lst (list)) "
                          "  (set i 0) "
                          "  (while (< i 500) "
                          "    (begin (set lst (append lst (list i))) "
                          "           (set i (+ i 1)))) "
                          "  lst)";

  scheduler sched(scheduling_policy::round_robin);

  for (int i = 0; i < NUM_PROCS; ++i) {
    execute_options opts;
    opts.max_steps = 5000;
    opts.env = builtins::make_default_environment();
    sched.execute(src, opts);
  }

  auto t0 = hrclock::now();
  sched.run();
  auto t1 = hrclock::now();

  auto stats = sched.get_stats();
  report("memory_pressure_200", NUM_PROCS, std::chrono::duration_cast<ns>(t1 - t0));
  std::printf("  terminated=%d killed=%d total_steps=%llu\n", stats.terminated, stats.killed,
              (unsigned long long)stats.total_steps);

  // All should have terminated or been killed by step limit
  EXPECT_EQ(stats.terminated + stats.killed, NUM_PROCS);
}

// -----------------------------------------------------------------------
// 3. High-concurrency scheduler stress
// -----------------------------------------------------------------------

TEST_F(ExecBenchTest, HighConcurrencyFlood) {
  if (!bench_enabled()) {
    SUCCEED();
    return;
  }

  // Flood the scheduler with many short-lived processes.
  constexpr int NUM_PROCS = 1000;

  scheduler sched(scheduling_policy::round_robin);

  for (int i = 0; i < NUM_PROCS; ++i) {
    std::string src = "(begin (set x " + std::to_string(i) +
                      ") "
                      "(+ x x))";
    execute_options opts;
    opts.max_steps = 1000;
    opts.env = builtins::make_default_environment();
    sched.execute(src, opts);
  }

  auto t0 = hrclock::now();
  sched.run();
  auto t1 = hrclock::now();

  auto stats = sched.get_stats();
  report("flood_1000_procs", NUM_PROCS, std::chrono::duration_cast<ns>(t1 - t0));
  std::printf("  total_steps=%llu terminated=%d\n", (unsigned long long)stats.total_steps,
              stats.terminated);
  EXPECT_EQ(stats.terminated, NUM_PROCS);
}

TEST_F(ExecBenchTest, DeepRecursionStress) {
  if (!bench_enabled()) {
    SUCCEED();
    return;
  }

  // Stress: 100 processes each running deep mutual recursion
  constexpr int NUM_PROCS = 100;
  const std::string src = "(begin "
                          "  (defun even? (n) (if (= n 0) #t (odd? (- n 1)))) "
                          "  (defun odd?  (n) (if (= n 0) #f (even? (- n 1)))) "
                          "  (even? 200))";

  scheduler sched(scheduling_policy::round_robin);

  for (int i = 0; i < NUM_PROCS; ++i) {
    execute_options opts;
    opts.max_steps = 500000;
    opts.env = builtins::make_default_environment();
    sched.execute(src, opts);
  }

  auto t0 = hrclock::now();
  sched.run();
  auto t1 = hrclock::now();

  auto stats = sched.get_stats();
  report("deep_recursion_100", NUM_PROCS, std::chrono::duration_cast<ns>(t1 - t0));
  std::printf("  total_steps=%llu terminated=%d\n", (unsigned long long)stats.total_steps,
              stats.terminated);
  EXPECT_EQ(stats.terminated, NUM_PROCS);
}
