/// @file state_exec_heap_test.cpp
/// @brief A process's heap: freeing leaked reference cycles, and its measured share for
/// max_memory (heap.h).

#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/heap.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/parser.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/scheduler.h>
#include <cvc/state/state_exec/stackless_evaluator.h>
#include <cvc/state/state_exec/state_value_codec.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

using namespace cvc::state_exec;

namespace {

// Run `script` to completion with a fresh default environment, keep weak handles to the
// environment and to the list bound to `big` (if any), and drop everything else.
struct leak_probe {
  std::weak_ptr<environment> env;
  std::weak_ptr<std::vector<value_t>> big;
};

leak_probe run_and_drop(const std::string &script) {
  leak_probe probe;
  scheduler sched;
  execute_options opts;
  opts.env = builtins::make_default_environment();
  probe.env = opts.env;
  sched.execute(script, opts);
  sched.run();
  if (const value_t *v = opts.env->lookup("big"))
    if (const auto *l = std::get_if<list_ptr>(&v->v))
      probe.big = *l;
  return probe; // the scheduler, its processes and `opts` go here
}

list_ptr int_list(std::size_t n) {
  std::vector<value_t> v;
  v.reserve(n);
  for (std::size_t i = 0; i < n; ++i)
    v.emplace_back(static_cast<int64_t>(i));
  return std::get<list_ptr>(make_list(std::move(v)).v);
}

// A process whose global environment (a default one, which only the process holds) has
// `bindings`: enough for measuring.
process proc_holding(std::vector<std::pair<std::string, value_t>> bindings) {
  auto env = builtins::make_default_environment();
  for (auto &[name, v] : bindings)
    env->set(name, std::move(v));
  process p;
  p.state.global_env = std::move(env);
  return p;
}

double ratio(std::size_t a, std::size_t b) {
  return static_cast<double>(a) / static_cast<double>(b);
}

} // namespace

// --- reference cycles --------------------------------------------------------------------------

TEST(CycleCollectorTest, NoCycleNeedsNoCollector) {
  const leak_probe p = run_and_drop("(set big (collections.range 1000))");
  EXPECT_TRUE(p.env.expired());
  EXPECT_TRUE(p.big.expired());
}

TEST(CycleCollectorTest, EveryLeakPatternIsFreed) {
  // Each of these used to keep its whole environment (and the 1000-element list in it) alive
  // forever once the program ended: a closure bound in the scope it captured, a container that
  // holds itself, a class whose methods capture the scope it is bound in, a generator stored where
  // its closure can reach it.
  const char *const scripts[] = {
      "(begin (set big (collections.range 1000)) (defun f (x) x))",
      "(begin (set big (collections.range 1000)) (set g (lambda (x) x)))",
      "(begin (set big (collections.range 1000)) (let ((h nil)) (set h (lambda () h)) 1))",
      "(begin (set big (collections.range 1000)) (append big big))",
      "(begin (set big (collections.range 1000)) (set d (dict)) (set-attr d \"self\" d) "
      "(set-attr d \"big\" big))",
      "(begin (set big (collections.range 1000)) (defclass Point (init (self x) (set-attr self "
      "\"x\" x) self) (getx (self) (get-attr self \"x\"))) (set p (Point 3)))",
      "(begin (set big (collections.range 1000)) (set gen (generator (lambda () (yield big)))))",
  };
  for (const char *script : scripts) {
    const leak_probe p = run_and_drop(script);
    collect_cycles();
    EXPECT_TRUE(p.env.expired()) << script;
    EXPECT_TRUE(p.big.expired()) << script;
  }
}

TEST(CycleCollectorTest, ClassesStillWork) {
  // defclass's natives became named types (so the collector can see their captures); classes,
  // inheritance and the default init/finalize behave as before.
  scheduler sched;
  const int pid =
      sched.execute(std::string("(begin (defclass A (init (self v) (set-attr self \"v\" v) self) "
                                "(get (self) (get-attr self \"v\"))) "
                                "(defclass B A (twice (self) (* 2 (send self \"get\")))) "
                                "(defclass C) (set c (C)) "
                                "(list (send (A 5) \"get\") (send (B 7) \"twice\") (is-null "
                                "(get-attr c \"__class__\"))))"),
                    {});
  sched.run();
  ASSERT_TRUE(sched.get_result(pid));
  EXPECT_EQ(to_string(*sched.get_result(pid)), "(5 14 #f)");
}

TEST(CycleCollectorTest, LiveDataIsNeverFreed) {
  // A host-held environment (pycvc's Exec, an Ariadne action context) with a function in it.
  auto env = builtins::make_default_environment();
  {
    scheduler sched;
    execute_options opts;
    opts.env = env;
    sched.execute(std::string("(begin (set base 40) (defun add2 (x) (+ x base)))"), opts);
    sched.run();
  }
  const cycle_stats s = collect_cycles();
  EXPECT_GE(s.candidates, 1u);
  scheduler sched;
  execute_options opts;
  opts.env = env;
  const int pid = sched.execute(std::string("(add2 2)"), opts);
  sched.run();
  ASSERT_TRUE(sched.get_result(pid));
  EXPECT_EQ(std::get<int64_t>(sched.get_result(pid)->v), 42);
}

TEST(CycleCollectorTest, ParkedProcessKeepsItsFunctions) {
  collect_cycles(); // earlier tests' leftovers
  async_scheduler sched;
  intrinsics_context ictx;
  ictx.sched = &sched;
  ictx.proc = make_process();
  execute_options opts;
  opts.env = builtins::make_default_environment();
  register_intrinsics(opts.env, &ictx);
  const int pid = sched.execute(std::string("(begin (defun twice (x) (* 2 x)) "
                                            "(twice (get-attr (msg-recv \"in\") \"n\")))"),
                                opts);
  sched.sync_run();
  const cycle_stats s = collect_cycles(); // the process is parked: everything it holds is live
  EXPECT_EQ(s.freed, 0u);
  sched.post_message("in", make_dict({{"n", value_t(int64_t(21))}}));
  sched.drain_ingress();
  sched.sync_run();
  ASSERT_TRUE(sched.get_result(pid)) << sched.get_exit_error(pid).value_or("");
  EXPECT_EQ(std::get<int64_t>(sched.get_result(pid)->v), 42);
}

TEST(CycleCollectorTest, DoesNothingInsideAStep) {
  auto env = builtins::make_default_environment();
  cycle_stats inside{1, 1, 1};
  builtins::register_fn(env, "collect-now", [&inside](std::span<const value_t>) -> value_t {
    inside = collect_cycles();
    return nil_value;
  });
  scheduler sched;
  execute_options opts;
  opts.env = env;
  sched.execute(std::string("(begin (defun f () 1) (collect-now))"), opts);
  sched.run();
  EXPECT_EQ(inside.candidates, 0u);
  EXPECT_EQ(inside.freed, 0u);
}

TEST(CycleCollectorTest, SchedulersCollectOnTheirOwn) {
  // Each of these programs leaks a let scope per iteration; the scheduler collects between steps
  // once enough piled up, so the first iterations' scopes are gone by the end without any
  // explicit collect_cycles().
  collect_cycles();
  std::weak_ptr<environment> first;
  bool remembered = false;
  scheduler sched;
  auto env = builtins::make_default_environment();
  builtins::register_fn(env, "remember",
                        [&first, &remembered](std::span<const value_t> a) -> value_t {
                          if (!remembered)
                            if (const auto *c = std::get_if<closure_ptr>(&a[0].v)) {
                              first = (*c)->env_snapshot; // the first iteration's let scope
                              remembered = true;
                            }
                          return nil_value;
                        });
  execute_options opts;
  opts.env = env;
  const int pid = sched.execute(
      std::string("(begin (set i 0) (while (< i 2000) (begin (let ((h nil)) (set h (lambda () h)) "
                  "(remember h)) (set i (+ i 1)))) i)"),
      opts);
  sched.run();
  ASSERT_TRUE(sched.get_result(pid));
  EXPECT_TRUE(first.expired());
  EXPECT_LT(tracked_objects(), 2000u);
}

// --- measuring a share -------------------------------------------------------------------------

TEST(HeapShareTest, CountsWhatTheProcessHolds) {
  // Each process is built in a statement of its own: a braced list's elements live to the end of
  // the full-expression, and would be a second owner while measuring.
  const process empty = proc_holding({});
  const process holder = proc_holding({{"big", value_t{int_list(10000)}}});
  const std::size_t base = measure_heap_share(empty);
  const std::size_t list = measure_heap_share(holder) - base;
  EXPECT_GE(list, 10000 * sizeof(value_t));
  EXPECT_LT(list, 10000 * sizeof(value_t) + 4096);
}

TEST(HeapShareTest, SharedValuesAreSplitBetweenHolders) {
  const process empty = proc_holding({});
  const process alone_holder = proc_holding({{"v", value_t{int_list(10000)}}});
  const std::size_t base = measure_heap_share(empty);
  const std::size_t alone = measure_heap_share(alone_holder) - base;
  list_ptr shared = int_list(10000);
  std::vector<process> holders;
  for (int i = 0; i < 4; ++i) {
    process p = proc_holding({{"v", value_t{shared}}});
    holders.push_back(std::move(p));
  }
  shared.reset(); // the four processes are its only owners
  std::size_t total = 0;
  for (const process &p : holders) {
    const std::size_t share = measure_heap_share(p) - base;
    EXPECT_NEAR(ratio(share, alone), 0.25, 0.02) << share << " of " << alone;
    total += share;
  }
  EXPECT_NEAR(ratio(total, alone), 1.0, 0.05); // the shares add up to the value, once
}

TEST(HeapShareTest, ASharedNodePassesItsShareDown) {
  // Two holders of an outer list whose inner list only the outer one holds: each pays half of
  // both, not half of the outer and all of the inner.
  const auto nested = [] {
    return std::get<list_ptr>(make_list({value_t{int_list(5000)}, value_t(int64_t(1))}).v);
  };
  const process empty = proc_holding({});
  const process whole_holder = proc_holding({{"o", value_t{nested()}}});
  list_ptr outer = nested();
  const process p1 = proc_holding({{"o", value_t{outer}}});
  const process p2 = proc_holding({{"o", value_t{outer}}});
  outer.reset(); // only the two processes hold it
  const std::size_t base = measure_heap_share(empty);
  const std::size_t whole = measure_heap_share(whole_holder) - base;
  const std::size_t s1 = measure_heap_share(p1) - base;
  const std::size_t s2 = measure_heap_share(p2) - base;
  EXPECT_NEAR(ratio(s1, whole), 0.5, 0.02) << s1 << " of " << whole;
  EXPECT_NEAR(ratio(s2, whole), 0.5, 0.02) << s2 << " of " << whole;
}

TEST(HeapShareTest, OptionsComeFromTheTree) {
  cvc::app app;
  auto &root = cvc::state::instance(app);
  EXPECT_EQ(heap_options_from_state(root, "s").check_steps, 4096u);
  root("state_exec.defaults.limits.heap_check_steps").value(std::string("100"));
  EXPECT_EQ(heap_options_from_state(root, "s").check_steps, 100u);
  root("state_exec.schedulers.s.limits.heap_check_steps").value(std::string("0"));
  EXPECT_EQ(heap_options_from_state(root, "s").check_steps, 0u);
  root("state_exec.schedulers.t.limits.heap_check_steps").value(std::string("x"));
  EXPECT_EQ(heap_options_from_state(root, "t").check_steps, 100u);
}

// --- max_memory covers the heap ----------------------------------------------------------------

TEST(HeapBudgetTest, KeptValuesCountAgainstMaxMemory) {
  // No state-tree writes at all: before heap accounting this ran to the end whatever its budget.
  scheduler sched;
  execute_options opts;
  opts.max_memory = 256 * 1024;
  const int pid = sched.execute(
      std::string("(begin (set keep (list)) (set i 0) (while (< i 60) "
                  "(begin (append keep (collections.range 200)) (set i (+ i 1)))) (length keep))"),
      opts);
  // Over the budget the process is killed (memory_limit_exceeded); or, when a measurement left
  // less room than the next call needs, that call fails its allocation limit (which the sync
  // scheduler rethrows). Without heap accounting neither happens: nothing is written to the tree.
  std::string error;
  try {
    sched.run();
  } catch (const std::exception &e) {
    error = e.what();
  }
  const auto info = sched.get_process_info(pid);
  ASSERT_TRUE(info);
  EXPECT_TRUE(info->status == process_status::killed ||
              error.find("allocation limit") != std::string::npos)
      << error;
  EXPECT_FALSE(sched.get_result(pid));
  EXPECT_GT(info->heap_measurements, 0u);
}

TEST(HeapBudgetTest, TemporariesDoNotCount) {
  scheduler sched;
  execute_options opts;
  opts.max_memory = 256 * 1024;
  const int pid = sched.execute(
      std::string("(begin (set i 0) (while (< i 60) (begin (set tmp (collections.range 200)) "
                  "(set i (+ i 1)))) i)"),
      opts);
  sched.run();
  ASSERT_TRUE(sched.get_result(pid));
  EXPECT_EQ(std::get<int64_t>(sched.get_result(pid)->v), 60);
  const auto info = sched.get_process_info(pid);
  ASSERT_TRUE(info);
  EXPECT_GT(info->heap_measurements, 0u);
  EXPECT_LT(info->heap_memory, opts.max_memory);
}

TEST(HeapBudgetTest, NoBudgetMeansNoMeasuring) {
  scheduler sched;
  const int pid = sched.execute(
      std::string("(begin (set keep (list)) (set i 0) (while (< i 60) "
                  "(begin (append keep (collections.range 200)) (set i (+ i 1)))) (length keep))"),
      {});
  sched.run();
  ASSERT_TRUE(sched.get_result(pid));
  EXPECT_EQ(sched.get_process_info(pid)->heap_measurements, 0u);
}

TEST(HeapBudgetTest, MeasuringRunsAsSlicedDataSteps) {
  cvc::app app;
  auto &root = cvc::state::instance(app);
  root("state_exec.defaults.data.step_nodes").value(std::string("8"));
  root("state_exec.defaults.limits.heap_check_steps").value(std::string("50"));
  scheduler sched;
  sched.set_watch_root(&root);
  sched.load_settings();
  execute_options opts;
  opts.max_memory = std::size_t{64} << 20;
  const int pid =
      sched.execute(std::string("(begin (set keep (list)) (set i 0) (while (< i 100) "
                                "(begin (append keep (list i i)) (set i (+ i 1)))) (length keep))"),
                    opts);
  sched.run();
  ASSERT_TRUE(sched.get_result(pid));
  EXPECT_EQ(std::get<int64_t>(sched.get_result(pid)->v), 100);
  const auto info = sched.get_process_info(pid);
  ASSERT_TRUE(info);
  EXPECT_GT(info->heap_measurements, 0u);
  EXPECT_GT(info->data_steps, info->heap_measurements); // a measurement took several slices
}

TEST(HeapBudgetTest, AsyncSchedulerCountsTheHeapToo) {
  async_scheduler sched;
  execute_options opts;
  opts.max_memory = 256 * 1024;
  const int pid = sched.execute(
      std::string("(begin (set keep (list)) (set i 0) (while (< i 60) "
                  "(begin (append keep (collections.range 200)) (set i (+ i 1)))) (length keep))"),
      opts);
  sched.sync_run();
  const std::string error = sched.get_exit_error(pid).value_or("");
  EXPECT_TRUE(error == "memory_limit_exceeded" ||
              error.find("allocation limit") != std::string::npos)
      << error;
}

// --- a measurement next to the evaluator's other deep-data work ------------------------------

// A measurement starts between two steps and waits on no frame: it records the whole stack as
// its base, so a snapshot taken while it runs keeps every frame, and evaluation resumes where it
// was. (The other deep-data work, a native call's job, waits on its call's frame instead.)
TEST(HeapBudgetTest, AMeasurementWaitsOnNoFrame) {
  cvc::app app;
  auto env = builtins::make_default_environment();
  env->set("keep", value_t{int_list(2000)});
  stackless_evaluator ev(env);
  process p;
  p.max_memory = std::size_t{64} << 20;
  p.state = ev.create_state(parse("(+ 1 (length keep) (length keep))"));
  p.state.global_env = env;
  p.state.data_step_nodes = 8;
  p.state.stats.start();
  for (int i = 0; i < 4; ++i)
    ASSERT_FALSE(ev.step(p.state));
  const std::size_t frames = p.state.stack.size();
  ASSERT_GT(frames, 1u);
  for (int i = 0; i < 64 && !p.state.data_work; ++i)
    heap_after_step(p, std::size_t{1} << 20, 0); // allocations call for a measurement
  ASSERT_TRUE(p.state.data_work);
  ASSERT_EQ(p.state.data_work->what, pending_data_work::kind::heap_measure);
  EXPECT_EQ(p.state.data_work->base, frames);
  EXPECT_EQ(p.state.data_work->frame_args, p.state.stack.back().results.size());

  auto &node = cvc::state::instance(app)("snapshot");
  encode_evaluator_state(node, p.state);
  EXPECT_EQ(decode_evaluator_state(node).stack.size(), frames);
  // Even a measurement made without a base (0) never shortens a snapshot's stack.
  p.state.data_work->base = 0;
  auto &again = cvc::state::instance(app)("snapshot2");
  encode_evaluator_state(again, p.state);
  EXPECT_EQ(decode_evaluator_state(again).stack.size(), frames);
  p.state.data_work->base = frames;

  const int steps = [&] {
    int n = 0;
    while (!ev.step(p.state))
      ++n;
    return n;
  }();
  EXPECT_GT(steps, 0);
  ASSERT_TRUE(p.state.heap_measured);
  EXPECT_EQ(std::get<int64_t>(p.state.result.v), 4001);
}

// fork refuses while deep-data work a call waits on is in flight, but a measurement holds no
// frame, so a process can fork in the middle of one.
TEST(HeapBudgetTest, AProcessForksDuringAMeasurement) {
  cvc::app app;
  auto &root = cvc::state::instance(app);
  // Small slices, so a measurement spans many steps; a measurement every 100 steps.
  root("state_exec.defaults.data.step_nodes").value(std::string("4"));
  root("state_exec.defaults.limits.heap_check_steps").value(std::string("100"));
  scheduler sched;
  sched.set_watch_root(&root);
  sched.load_settings();
  execute_options opts;
  opts.max_memory = std::size_t{64} << 20;
  const int pid =
      sched.execute(std::string("(begin (set keep (list)) (set i 0) (while (< i 400) "
                                "(begin (append keep (list i i)) (set i (+ i 1)))) (length keep))"),
                    opts);
  // The program has no other deep-data work, so a data step that does not finish a measurement
  // is one in progress (or, rarely, freeing). Fork there, three times.
  process_info before = *sched.get_process_info(pid);
  int forks = 0;
  while (forks < 3 && before.status != process_status::terminated) {
    sched.step();
    const process_info now = *sched.get_process_info(pid);
    if (now.data_steps > before.data_steps && now.heap_measurements == before.heap_measurements &&
        now.status != process_status::terminated) {
      const int child = sched.fork(pid);
      EXPECT_GE(child, 0);
      ++forks;
      if (child >= 0)
        sched.kill(child);
    }
    before = now;
  }
  EXPECT_EQ(forks, 3);
  sched.run();
  EXPECT_GT(sched.get_process_info(pid)->heap_measurements, 0u);
  EXPECT_EQ(std::get<int64_t>(sched.get_result(pid)->v), 400);
}
