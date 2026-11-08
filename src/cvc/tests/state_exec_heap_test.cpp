/// @file state_exec_heap_test.cpp
/// @brief A process's heap: freeing leaked reference cycles, and its measured share for
/// max_memory (heap.h).

#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_evaluator.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/evaluator.h>
#include <cvc/state/state_exec/heap.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/parser.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/scheduler.h>
#include <cvc/state/state_exec/stackless_evaluator.h>
#include <cvc/state/state_exec/state_value_codec.h>
#include <cvc/state/state_exec/stdlib.h>
#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

// A program's closure bound in the scope it captured (a defun, a lambda set at top level) and that
// scope hold each other, and only the cycle collector frees them
// (docs/STATE_EXEC_DEVELOPER_GUIDE.md, "Reference cycles"). Nothing here runs long enough for a
// collection to be due, so once every test is done one collection leaves LeakSanitizer only real
// leaks to report.
class collect_cycles_at_exit : public ::testing::Environment {
public:
  void TearDown() override { cvc::state_exec::collect_cycles(); }
};
[[maybe_unused]] ::testing::Environment *const g_collect_cycles =
    ::testing::AddGlobalTestEnvironment(new collect_cycles_at_exit);

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

namespace {
// A class with methods, and one inheriting it with no init or finalize of its own: each binds,
// in the scope its methods capture, a cycle through its class dict.
const char *const kClassScript =
    "(begin (defclass A (init (self v) (set-attr self \"v\" v) self) (get (self) (get-attr self "
    "\"v\"))) (defclass B A (twice (self) (* 2 (send self \"get\")))) "
    "(list (send (A 21) \"get\") (send (B 21) \"twice\")))";
} // namespace

TEST(CycleCollectorTest, TheRecursiveEvaluatorsClassesAreFreed) {
  // The recursive evaluator's defclass stores its methods, default init and finalize as named
  // natives (closure_natives.h), so the collector sees the methods' closures and the cycle.
  collect_cycles();
  std::weak_ptr<environment> seen;
  {
    auto env = builtins::make_default_environment();
    seen = env;
    evaluator ev(env);
    EXPECT_EQ(to_string(ev.evaluate_script(kClassScript)), "(21 42)");
  }
  EXPECT_GT(collect_cycles().freed, 0u);
  EXPECT_TRUE(seen.expired());
}

TEST(CycleCollectorTest, TheCoroutineEvaluatorsClassesAreFreed) {
  // The coroutine evaluator's too, and its constructor is the stackless evaluator's.
  collect_cycles();
  std::weak_ptr<environment> seen;
  {
    auto env = builtins::make_default_environment();
    seen = env;
    async_evaluator ev(env);
    EXPECT_EQ(to_string(ev.sync_evaluate_script(kClassScript)), "(21 42)");
  }
  EXPECT_GT(collect_cycles().freed, 0u);
  EXPECT_TRUE(seen.expired());
}

TEST(CycleCollectorTest, ARestoredProcessIsFreed) {
  // A restored process's closures capture scopes the codec made. They are tracked as the
  // evaluators track theirs, so a defun and a class's methods bound back into them are freed once
  // the process is gone.
  collect_cycles();
  std::weak_ptr<environment> seen;
  bool remembered = false;
  {
    cvc::app app;
    cvc::state &snap = cvc::state::instance(app)("snap");
    async_scheduler a, b;
    execute_options opts;
    opts.env = builtins::make_default_environment();
    const int pid = a.execute(
        std::string("(begin (defun twice (x) (* 2 x)) (defclass Box (init (self v) (set-attr "
                    "self \"v\" v) self) (get (self) (get-attr self \"v\"))) (set box (Box 21)) "
                    "(set i 0) (while (< i 200) (set i (+ i 1))) (remember twice) "
                    "(list (twice 21) (send box \"get\")))"),
        opts);
    for (int s = 0; s < 100; ++s)
      a.sync_step();
    ASSERT_FALSE(a.get_result(pid).has_value());
    ASSERT_TRUE(a.snapshot(pid, snap));
    a.kill(pid);
    auto natives = builtins::make_default_environment();
    builtins::register_fn(natives, "remember", [&](std::span<const value_t> args) -> value_t {
      if (const auto *c = std::get_if<closure_ptr>(&args[0].v)) {
        seen = (*c)->env_snapshot;
        remembered = true;
      }
      return nil_value;
    });
    const int moved = b.restore(snap, natives);
    b.sync_run();
    ASSERT_TRUE(b.get_result(moved)) << b.get_exit_error(moved).value_or("");
    EXPECT_EQ(to_string(*b.get_result(moved)), "(42 21)");
  }
  ASSERT_TRUE(remembered);
  EXPECT_GT(collect_cycles().freed, 0u);
  EXPECT_TRUE(seen.expired());
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

// --- a group's registry, stream and base (S1.1-S1.3) ---------------------------------------------

namespace {
const char *const kCycleScript = "(begin (set big (collections.range 1000)) (defun f (x) x))";
} // namespace

TEST(CycleRegistryTest, ABoundRegistryTakesWhatIsTracked) {
  // A scheduler binds its group's registry on every entry (td 4.7): whatever runs under the
  // binding tracks into it, and the no-argument forms follow it.
  collect_cycles();
  const std::size_t own = tracked_objects();
  cycle_registry_ptr r = make_cycle_registry();
  leak_probe p;
  {
    cycle_registry_scope bind(r.get());
    p = run_and_drop(kCycleScript);
    EXPECT_GT(tracked_objects(*r), 0u);
    EXPECT_EQ(tracked_objects(), tracked_objects(*r));
    note_process_exit();
    EXPECT_TRUE(exit_pending_for_test(*r));
  }
  EXPECT_EQ(tracked_objects(), own); // nothing went to this thread's registry
  collect_cycles();                  // so its collection does not see the cycle
  EXPECT_FALSE(p.env.expired());
  EXPECT_GT(collect_cycles(*r).freed, 0u);
  EXPECT_TRUE(p.env.expired());
  EXPECT_TRUE(p.big.expired());
  EXPECT_FALSE(exit_pending_for_test(*r));
}

TEST(CycleRegistryTest, TheDeleterFreesWhatADeadSchedulerLeft) {
  // A group's owner frees its registry after its scheduler and envs: the deleter's collection
  // frees the cycle the dead scheduler's env still makes.
  collect_cycles();
  cycle_registry_ptr r = make_cycle_registry();
  leak_probe p;
  {
    cycle_registry_scope bind(r.get());
    p = run_and_drop(kCycleScript); // the scheduler and its env are gone
  }
  collect_cycles();
  EXPECT_FALSE(p.env.expired());
  r.reset();
  EXPECT_TRUE(p.env.expired());
  EXPECT_TRUE(p.big.expired());
}

TEST(CycleRegistryTest, WhatOutlivesARegistryJoinsTheCallers) {
  // A host keeping a group's function past the group: the function and the scope it is bound in
  // survive the registry's final collection, and the host thread's registry frees them once the
  // host lets go.
  collect_cycles();
  std::weak_ptr<environment> seen;
  value_t held;
  cycle_registry_ptr r = make_cycle_registry();
  {
    cycle_registry_scope bind(r.get());
    scheduler sched;
    execute_options opts;
    opts.env = builtins::make_default_environment();
    seen = opts.env;
    const int pid = sched.execute(std::string("(begin (defun f (x) x) f)"), opts);
    sched.run();
    ASSERT_TRUE(sched.get_result(pid));
    held = *sched.get_result(pid);
  }
  const std::size_t own = tracked_objects();
  r.reset();
  EXPECT_FALSE(seen.expired());
  EXPECT_GT(tracked_objects(), own);
  held = nil_value;
  EXPECT_FALSE(seen.expired()); // the scope and f hold each other
  EXPECT_GT(collect_cycles().freed, 0u);
  EXPECT_TRUE(seen.expired());
}

TEST(CycleRegistryTest, EveryRegistryHasAnIdOfItsOwn) {
  // Ids are never reused, though a registry made after one is freed often takes its address.
  EXPECT_EQ(bound_cycle_registry_id(), 0u);
  std::set<std::uint64_t> ids;
  for (int i = 0; i < 100; ++i) {
    cycle_registry_ptr r = make_cycle_registry();
    cycle_registry_scope bind(r.get());
    EXPECT_NE(bound_cycle_registry_id(), 0u);
    EXPECT_TRUE(ids.insert(bound_cycle_registry_id()).second);
  }
  // Scopes nest and restore what they found; a null one binds none.
  cycle_registry_ptr a = make_cycle_registry();
  cycle_registry_ptr b = make_cycle_registry();
  {
    cycle_registry_scope bind_a(a.get());
    const std::uint64_t id_a = bound_cycle_registry_id();
    {
      cycle_registry_scope bind_b(b.get());
      EXPECT_NE(bound_cycle_registry_id(), id_a);
      {
        cycle_registry_scope none(nullptr);
        EXPECT_EQ(bound_cycle_registry_id(), 0u);
      }
    }
    EXPECT_EQ(bound_cycle_registry_id(), id_a);
  }
  EXPECT_EQ(bound_cycle_registry_id(), 0u);
}

TEST(CycleRegistryTest, AThreadThatOptsInIsCollectedWhenItEnds) {
  // S1-Q3: a thread's own registry is collected at its exit only if the thread asked.
  const auto on_thread = [](bool opt_in) {
    leak_probe p;
    std::thread t([&] {
      if (opt_in)
        collect_cycles_at_thread_exit();
      p = run_and_drop(kCycleScript);
    });
    t.join();
    return p;
  };
  const leak_probe in = on_thread(true);
  EXPECT_TRUE(in.env.expired());
  EXPECT_TRUE(in.big.expired());
  const leak_probe out = on_thread(false);
  EXPECT_FALSE(out.env.expired()); // its registry went with nothing collected
  if (environment_ptr env = out.env.lock())
    env->bindings.clear(); // break the cycle by hand, so LeakSanitizer finds no leak
  EXPECT_TRUE(out.env.expired());
}

TEST(CycleRegistryTest, ClosureBodiesAreSharedWithinARegistryOnly) {
  // The closures one lambda form makes share their (begin ...) body list on a thread, but only
  // under one registry: two groups taking turns on a worker never share one (td 4.7).
  const value_t form = parse("(lambda () 1 2)");
  auto env = builtins::make_default_environment();
  stackless_evaluator ev(env);
  const auto make = [&] { return std::get<closure_ptr>(ev.evaluate(form, env).v); };
  cycle_registry_ptr a = make_cycle_registry();
  cycle_registry_ptr b = make_cycle_registry();
  closure_ptr a1, a2, b1;
  {
    cycle_registry_scope bind(a.get());
    a1 = make();
    a2 = make();
  }
  {
    cycle_registry_scope bind(b.get());
    b1 = make();
  }
  ASSERT_TRUE(a1->body_expr && a2->body_expr && b1->body_expr);
  EXPECT_EQ(a1->body_expr, a2->body_expr);
  EXPECT_NE(a1->body_expr, b1->body_expr);
}

TEST(RandomStreamTest, ASeededStreamRepeatsOnAnyThread) {
  // S1.2: math.random draws from the bound stream, seeded on first use, so a group's sequence
  // does not depend on which thread pumps it (td 4.10).
  const auto draw = [](random_stream &s) {
    random_stream_scope bind(&s);
    auto env = builtins::make_default_environment();
    stackless_evaluator ev(env);
    return to_string(ev.evaluate_script("(list (math.random) (math.random) (math.random))", env));
  };
  random_stream a, b, c;
  a.seed = 42;
  b.seed = 42;
  c.seed = 43;
  const std::string first = draw(a);
  EXPECT_TRUE(a.seeded);
  EXPECT_EQ(draw(b), first);
  EXPECT_NE(draw(c), first);
  // Three draws here and three on another thread equal six in a row.
  const std::string more = draw(a);
  random_stream moved;
  moved.seed = 42;
  draw(moved);
  std::string elsewhere;
  std::thread([&] { elsewhere = draw(moved); }).join();
  EXPECT_EQ(elsewhere, more);
  // Unseeded, a stream seeds itself from the OS.
  random_stream d;
  draw(d);
  EXPECT_TRUE(d.seeded);
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
  // less room than the next call needs, that call fails its allocation limit: a MemoryError the
  // program doesn't catch, so the process fails. Without heap accounting neither happens:
  // nothing is written to the tree.
  sched.run();
  const auto info = sched.get_process_info(pid);
  ASSERT_TRUE(info);
  const std::string error = info->exit_error.value_or("");
  EXPECT_TRUE(info->status == process_status::killed ||
              (info->status == process_status::failed &&
               error.find("allocation limit") != std::string::npos))
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

// A measurement due in the middle of a call in flight (here collections.map calling a DSL
// function) runs on top of it, waiting on no frame, and the call goes on after it. The process
// cannot fork meanwhile: a frame still waits on the call.
TEST(HeapBudgetTest, AMeasurementRunsOnTopOfACallInFlight) {
  cvc::app app;
  auto env = builtins::make_default_environment();
  env->set("keep", value_t{int_list(200)});
  stackless_evaluator ev(env);
  process p;
  p.max_memory = std::size_t{64} << 20;
  p.state = ev.create_state(parse("(length (collections.map keep (lambda (x) (+ x 1))))"));
  p.state.global_env = env;
  p.state.data_step_nodes = 8;
  p.state.stats.start();
  while (!(p.state.data_work && p.state.data_work->calling))
    ASSERT_FALSE(ev.step(p.state));
  const std::shared_ptr<pending_data_work> call = p.state.data_work;
  const std::size_t frames = p.state.stack.size();
  heap_after_step(p, std::size_t{1} << 20, 0); // allocations call for a measurement
  ASSERT_TRUE(p.state.data_work);
  ASSERT_EQ(p.state.data_work->what, pending_data_work::kind::heap_measure);
  EXPECT_EQ(p.state.data_work->outer, call);
  EXPECT_EQ(p.state.data_work->base, frames);
  EXPECT_TRUE(waits_on_data_work(p.state));
  while (!p.state.heap_measured) {
    ASSERT_FALSE(ev.step(p.state));
    ASSERT_EQ(p.state.stack.size(), frames); // nothing else runs meanwhile
  }
  EXPECT_EQ(p.state.data_work, call);
  heap_after_step(p, 0, 0);
  EXPECT_GT(p.heap_bytes, 0u);
  EXPECT_FALSE(p.heap_measuring);
  while (!ev.step(p.state)) {
  }
  EXPECT_EQ(std::get<int64_t>(p.state.result.v), 200);
}

// --- work that never finishes is still held to max_memory ------------------------------------
//
// A call the evaluator makes can wait on another like it for as long as the budget lasts: a class
// whose init is its own constructor, or a closure init that constructs again. A measurement counts
// that work, with what its jobs hold (each waiting call's function and arguments), so max_memory
// stops the process; max_steps is only a backstop here.

const char *const kSelfInit = "(begin (defclass P) (set-attr P.__cls_dict__ \"init\" P) (P))";
const char *const kInitConstructs = "(begin (defclass P (init (self) (P))) (P))";

TEST(HeapBudgetTest, TheWorkAProcessWaitsOnCounts) {
  // After a hundred steps of kSelfInit, ~180 constructions wait, the k-th on a call with k
  // arguments: some 16000 values held by jobs alone.
  stackless_evaluator ev(builtins::make_default_environment());
  process p;
  p.state = ev.create_state(parse(kSelfInit));
  for (int i = 0; i < 100; ++i)
    ASSERT_FALSE(ev.step(p.state));
  EXPECT_GT(measure_heap_share(p), 10000 * sizeof(value_t));
}

TEST(HeapBudgetTest, ACallChainThatNeverEndsRunsOutOfMemory) {
  // kSelfInit's arguments grow quadratically: 2 MiB within a few hundred steps.
  for (const char *script : {kSelfInit, kInitConstructs}) {
    SCOPED_TRACE(script);
    const uint64_t backstop = script == kSelfInit ? 1000 : 1000000;
    {
      async_scheduler sched;
      execute_options opts;
      opts.max_memory = std::size_t{2} << 20;
      opts.max_steps = backstop;
      const int pid = sched.execute(std::string(script), opts);
      sched.sync_run();
      EXPECT_EQ(sched.get_exit_error(pid).value_or(""), "memory_limit_exceeded");
      EXPECT_GT(sched.get_process_info(pid)->heap_measurements, 0u);
    }
    {
      scheduler sched;
      execute_options opts;
      opts.max_memory = std::size_t{2} << 20;
      opts.max_steps = backstop;
      const int pid = sched.execute(std::string(script), opts);
      sched.run();
      const auto info = sched.get_process_info(pid);
      ASSERT_TRUE(info);
      EXPECT_EQ(info->status, process_status::killed);
      EXPECT_LT(info->step_count, backstop); // memory, not the step budget
      EXPECT_GE(info->heap_memory, opts.max_memory);
    }
  }
}
