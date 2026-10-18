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
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/data_job.h>
#include <cvc/state/state_exec/parser.h>
#include <cvc/state/state_exec/scheduler.h>
#include <cvc/state/state_exec/stackless_evaluator.h>
#include <cvc/state/state_exec/types.h>
#include <gtest/gtest.h>
#include <string>
#include <vector>

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

TEST_F(ExecBenchTest, ValueDropThroughput) {
  if (!bench_enabled()) {
    SUCCEED();
    return;
  }

  // Building and dropping values: the release path every list/dict/closure takes.
  constexpr int64_t N = 1000000;
  auto t0 = hrclock::now();
  for (int64_t i = 0; i < N; ++i) {
    value_t v = make_list({value_t{i}, make_list({value_t{i}}), make_dict({{"k", value_t{i}}})});
    (void)v;
  }
  auto t1 = hrclock::now();
  report("value_drop_small", N, std::chrono::duration_cast<ns>(t1 - t0));

  // Copies dropped while another owner lives.
  const value_t shared = make_list({value_t{int64_t{1}}, value_t{int64_t{2}}});
  std::vector<value_t> copies(64);
  t0 = hrclock::now();
  for (int64_t i = 0; i < N; ++i)
    copies[static_cast<std::size_t>(i % 64)] = shared;
  t1 = hrclock::now();
  report("value_drop_shared_copy", N, std::chrono::duration_cast<ns>(t1 - t0));

  // 1000-level lists, built and dropped.
  constexpr int64_t kDepth = 1000, kReps = 1000;
  t0 = hrclock::now();
  for (int64_t r = 0; r < kReps; ++r) {
    value_t v = make_list({value_t{r}});
    for (int64_t i = 1; i < kDepth; ++i)
      v = make_list({std::move(v)});
  }
  t1 = hrclock::now();
  report("value_drop_deep1000", kDepth * kReps, std::chrono::duration_cast<ns>(t1 - t0));

  // The same from the DSL: a list nested 2000 deep, dropped when the evaluator is.
  const std::string src = "(begin "
                          "  (set acc (list)) "
                          "  (set i 0) "
                          "  (while (< i 2000) "
                          "    (begin (set acc (list i acc)) (set i (+ i 1)))) "
                          "  i)";
  auto env = builtins::make_default_environment();
  constexpr int M = 50;
  t0 = hrclock::now();
  for (int i = 0; i < M; ++i) {
    stackless_evaluator ev(env);
    ev.evaluate_script(src);
  }
  t1 = hrclock::now();
  report("eval_nest_list_2000", M, std::chrono::duration_cast<ns>(t1 - t0));
}

TEST_F(ExecBenchTest, DataWalkThroughput) {
  if (!bench_enabled()) {
    SUCCEED();
    return;
  }

  // Deep-data work per node, the unit of data.step_nodes: compare, print, copy, free. The value
  // nests lists and dicts 20000 levels, each level holding a string and a number too.
  constexpr int64_t kLevels = 20000;
  const auto build = [] {
    value_t v = value_t{int64_t{0}};
    for (int64_t i = 0; i < kLevels; ++i)
      v = i % 2 ? make_dict({{"i", value_t{i}}, {"v", std::move(v)}})
                : make_list({value_t{std::string("s")}, std::move(v), value_t{1.5}});
    return v;
  };
  const value_t a = build(), b = build();
  const auto per_node = [](const char *name, data_job &job) {
    auto t0 = hrclock::now();
    job.step(0);
    auto t1 = hrclock::now();
    report(name, static_cast<int64_t>(job.nodes()), std::chrono::duration_cast<ns>(t1 - t0));
  };
  auto eq = make_equal_job(a, b);
  per_node("data_equal_per_node", *eq);
  auto print = make_print_job(a);
  per_node("data_print_per_node", *print);
  auto copy = make_copy_job(a);
  per_node("data_copy_per_node", *copy);

  // Freeing, through a release backlog drained whole.
  value_t doomed = build();
  detail::release_backlog backlog;
  {
    detail::backlog_scope route(&backlog);
    doomed = value_t{};
  }
  auto t0 = hrclock::now();
  const std::size_t freed = backlog.drain(0);
  auto t1 = hrclock::now();
  report("data_free_per_node", static_cast<int64_t>(freed),
         std::chrono::duration_cast<ns>(t1 - t0));
}

TEST_F(ExecBenchTest, StdlibJobThroughput) {
  if (!bench_enabled()) {
    SUCCEED();
    return;
  }

  // The stdlib materializers' work per node (one per element, one per kDataNodeBytes bytes
  // scanned or copied), run in 1024-node data steps on a stackless evaluator: a node's cost
  // includes the stepping, and freeing the result.
  constexpr int64_t kN = 200000;
  auto env = builtins::make_default_environment();
  auto run_env = environment::extend(env);
  std::string csv;
  std::vector<value_t> nums, parts, rev;
  for (int64_t i = 0; i < kN; ++i) {
    csv += (i ? "," : "") + std::to_string(i);
    nums.emplace_back(i);
    parts.emplace_back(std::to_string(i));
    rev.emplace_back(kN - i);
  }
  run_env->set("csv", value_t{csv});
  run_env->set("nums", make_list(std::move(nums)));
  run_env->set("parts", make_list(std::move(parts)));
  run_env->set("rev", make_list(std::move(rev)));
  const auto per_node = [&](const char *name, const char *script) {
    stackless_evaluator ev(env);
    evaluator_state st = ev.create_state(parse(script), run_env);
    st.data_step_nodes = 1024;
    auto t0 = hrclock::now();
    while (!ev.step(st)) {
    }
    auto t1 = hrclock::now();
    report(name, static_cast<int64_t>(st.stats.data_nodes()),
           std::chrono::duration_cast<ns>(t1 - t0));
  };
  per_node("stdlib_split_per_node", "(string.split csv \",\")");
  per_node("stdlib_join_per_node", "(string.join parts \",\")");
  per_node("stdlib_replace_per_node", "(string.replace csv \",\" \";\")");
  per_node("stdlib_encode_per_node", "(string.encode csv)");
  per_node("stdlib_sort_per_node", "(collections.sort rev)");
  per_node("stdlib_map_native_per_node", "(collections.map nums math.abs)");
  per_node("stdlib_map_lambda_per_node", "(collections.map nums (lambda (x) x))");
  per_node("stdlib_dict_merge_per_node",
           "(collections.dict-merge (dict \"a\" 1 \"b\" 2) (dict \"b\" 3 \"c\" 4))");

  // Small calls, where a job's setup is most of the cost.
  const std::string loop = "(begin (set i 0) (while (< i 10000) (begin"
                           " (string.split \"a,b,c\" \",\") (collections.reverse (list 1 2 3))"
                           " (set i (+ i 1)))) i)";
  constexpr int M = 20;
  auto t0 = hrclock::now();
  for (int i = 0; i < M; ++i) {
    stackless_evaluator ev(env);
    ev.evaluate_script(loop);
  }
  auto t1 = hrclock::now();
  report("stdlib_small_calls_10000", M, std::chrono::duration_cast<ns>(t1 - t0));
}

TEST_F(ExecBenchTest, NativeCallThroughput) {
  if (!bench_enabled()) {
    SUCCEED();
    return;
  }

  // Native functions calling native functions (apply, send to a native method, a constructor
  // running inherited inits), 10000 calls a loop; native_call_plain is the loop with a direct
  // call, for comparison.
  constexpr int64_t kCalls = 10000;
  constexpr int M = 20;
  auto env = builtins::make_default_environment();
  const auto loop = [&](const char *name, const std::string &setup, const std::string &call) {
    const std::string src = "(begin " + setup + " (set i 0) (while (< i " + std::to_string(kCalls) +
                            ") (begin " + call + " (set i (+ i 1)))) i)";
    auto t0 = hrclock::now();
    for (int r = 0; r < M; ++r) {
      stackless_evaluator ev(env);
      ev.evaluate_script(src);
    }
    auto t1 = hrclock::now();
    report(name, M * kCalls, std::chrono::duration_cast<ns>(t1 - t0));
  };
  loop("native_call_plain", "", "(list i 1)");
  loop("native_call_apply", "", "(apply list (list i 1))");
  loop("native_call_apply_apply", "", "(apply apply (list list (list i 1)))");
  loop("native_call_send_native", "(defclass C) (set-attr C.__cls_dict__ \"m\" list) (set o (C))",
       "(send o \"m\" i)");
  loop("native_call_construct_inherited", "(defclass P) (defclass K P)", "(K)");
  loop("native_call_construct_chain4", "(defclass A) (defclass B A) (defclass C B) (defclass D C)",
       "(D)");
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
