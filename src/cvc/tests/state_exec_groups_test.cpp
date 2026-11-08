// Threaded execution, Stage 1 (docs/STATE_EXEC_THREADED.md 7, 8.2): execution groups and the
// app's exec_pool, the Groups suite (S1.md section 4: the GR, FF, RX, RB, AS, BF, OC, DT, GL, PO
// and PA cases). The sanitizer jobs run this binary (ci.yml's SAN_CXX_TESTS) and require each
// case's line. No case name contains "Stress": run.sh filters those out.

#include <atomic>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/core/async_lane.h> // cvc::pump_exec_frame
#include <cvc/core/exec_pool.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/exec_group.h>
#include <cvc/state/state_exec/heap.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/scheduler.h>
#include <gtest/gtest.h>
#include <memory>
#include <mutex>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace cvc::state_exec;

// Bare evaluators and bare schedulers track into the main thread's own registry, and the cases are
// too short for a collection to be due: once every test is done, one collection leaves
// LeakSanitizer only real leaks to report.
class collect_cycles_at_exit : public ::testing::Environment {
public:
  void TearDown() override { collect_cycles(); }
};
[[maybe_unused]] ::testing::Environment *const g_collect_cycles =
    ::testing::AddGlobalTestEnvironment(new collect_cycles_at_exit);

namespace {

using clock_type = std::chrono::steady_clock;

double seconds_since(clock_type::time_point t) {
  return std::chrono::duration<double>(clock_type::now() - t).count();
}

// The sanitizer jobs set CVC_LOCKS_SANITIZER=1. Their builds run 5-15x slower, so the cases on
// several threads run fewer groups and thread counts there.
bool under_sanitizer() {
  const char *v = std::getenv("CVC_LOCKS_SANITIZER");
  return v && *v && std::string(v) != "0";
}

// A process's backstop: a case's processes end long before this, unless a pump hangs one.
constexpr double kProcessCeiling = 300.0;
// The one-sided ceiling on what a case waits for (rule 7): far above what it takes, which is
// printed.
constexpr double kCeiling = 10.0;

// What `pid` came to: its result (a string bare), or its status and exit error.
std::string outcome_of(const async_scheduler &sched, int pid) {
  if (const auto r = sched.get_result(pid))
    return std::holds_alternative<std::string>(r->v) ? std::get<std::string>(r->v) : to_string(*r);
  const auto i = sched.get_process_info(pid);
  return std::string("<") + (i ? to_string(i->status) : "gone") + " " +
         sched.get_exit_error(pid).value_or("") + ">";
}

bool finished(const async_scheduler &sched, int pid) {
  const auto i = sched.get_process_info(pid);
  return i && is_finished(i->status);
}

// (observe f) keeps a weak handle on the scope closure f captured, so a case can tell when the
// cycle it closes is freed. release() breaks every cycle still alive (its scope's bindings go), so
// a case that fails leaves LeakSanitizer nothing to report.
struct scope_observer {
  std::mutex m;
  std::vector<std::weak_ptr<environment>> seen;

  void install(const environment_ptr &env, const std::string &name = "observe") {
    builtins::register_fn(env, name, [this](std::span<const value_t> a) -> value_t {
      if (!a.empty())
        if (const auto *c = std::get_if<closure_ptr>(&a[0].v)) {
          std::lock_guard<std::mutex> lock(m);
          seen.push_back((*c)->env_snapshot);
        }
      return nil_value;
    });
  }
  std::size_t count() {
    std::lock_guard<std::mutex> lock(m);
    return seen.size();
  }
  std::size_t alive() {
    std::lock_guard<std::mutex> lock(m);
    std::size_t n = 0;
    for (const auto &w : seen)
      n += w.expired() ? 0 : 1;
    return n;
  }
  void release() {
    std::lock_guard<std::mutex> lock(m);
    for (const auto &w : seen)
      if (const environment_ptr e = w.lock())
        e->bindings.clear();
  }
};

} // namespace

// --- The registry, the frozen scope and the pump owner (S1.1, S1.3, S1.4) ---

// RX-1 (S1-Q3; ASan): a bare scheduler on a boost::thread leaves a cycle, and the thread ends. If
// the thread called collect_cycles_at_thread_exit(), its registry is collected then, and the
// cycle's weak observer has expired after the join; without the call nothing collects it. The
// program is far shorter than the 64 steps between due checks, so no collection runs before the
// thread ends.
TEST(Groups, RX01_AnOptedInThreadsCyclesAreCollectedWhenItEnds) {
  for (const bool opted_in : {true, false}) {
    SCOPED_TRACE(opted_in ? "opted in" : "not opted in");
    scope_observer observer;
    std::string outcome;
    boost::thread t([&] {
      if (opted_in)
        collect_cycles_at_thread_exit();
      async_scheduler sched;
      execute_options opts;
      opts.env = builtins::make_default_environment();
      observer.install(opts.env);
      const int pid = sched.execute(
          std::string("(begin (let ((h nil)) (set h (lambda () h)) (observe h)) 1)"), opts);
      sched.sync_run();
      outcome = outcome_of(sched, pid);
    });
    t.join();
    EXPECT_EQ(outcome, "1");
    ASSERT_EQ(observer.count(), 1u);
    if (opted_in)
      EXPECT_EQ(observer.alive(), 0u) << "the thread's registry was not collected when it ended";
    else
      EXPECT_EQ(observer.alive(), 1u) << "a thread that did not opt in was collected at its exit";
    observer.release();
  }
}

// RX-2 (S1.1): on a bare scheduler given a registry, a kill(pid) from the host thread between
// pumps notes the exit in that registry, whatever the thread has bound: not in a registry bound
// around the kill, and with nothing bound, not in the thread's own (it lands in the scheduler's).
TEST(Groups, RX02_AHostKillNotesTheSchedulersRegistry) {
  cycle_registry_ptr mine = make_cycle_registry();
  cycle_registry_ptr other = make_cycle_registry();
  {
    async_scheduler sched;
    sched.set_cycle_registry(mine.get());
    execute_options opts;
    opts.max_time = kProcessCeiling;
    const int first = sched.execute(std::string("(while #t nil)"), opts);
    const int second = sched.execute(std::string("(while #t nil)"), opts);
    sched.sync_run_steps(200); // a pump; then the host acts between pumps
    collect_cycles(*mine);
    ASSERT_FALSE(exit_pending_for_test(*mine));
    {
      cycle_registry_scope bound(other.get());
      ASSERT_TRUE(sched.kill(first));
    }
    EXPECT_TRUE(exit_pending_for_test(*mine)) << "a kill bound elsewhere";
    EXPECT_FALSE(exit_pending_for_test(*other));
    collect_cycles(*mine);
    ASSERT_FALSE(exit_pending_for_test(*mine));
    ASSERT_TRUE(sched.kill(second)); // nothing bound: the thread's own registry is the fallback
    EXPECT_TRUE(exit_pending_for_test(*mine)) << "a kill with nothing bound";
  } // the scheduler goes before its registries, as an owner orders them
}

// RX-3 (S1.1): one scheduler given a registry, stepped through every driver: cvc::pump_exec_frame,
// sync_run, sync_step, and a restore. Each program leaves a live cycle (keepN, bound in the env
// the test holds) and garbage ones; the restored process holds a closure in its own scope, which
// the restore decodes. All of it is tracked in the scheduler's registry, whichever driver ran it:
// the thread's own registry gains nothing.
TEST(Groups, RX03_EveryDriverTracksIntoTheSchedulersRegistry) {
  cvc::app app;
  cvc::state &tree = cvc::state::instance(app);
  cycle_registry_ptr reg = make_cycle_registry();
  collect_cycles(); // this thread's garbage from earlier cases
  const std::size_t fallback = tracked_objects();
  {
    async_scheduler sched;
    sched.set_cycle_registry(reg.get());
    const environment_ptr env = builtins::make_default_environment();
    execute_options opts;
    opts.env = env;
    opts.max_time = kProcessCeiling;
    const auto program = [](int n) {
      const std::string k = std::to_string(n);
      return "(begin (set keep" + k + " (let ((h nil)) (set h (lambda () h)) h)) (set i 0) " +
             "(while (< i 50) (begin (let ((g nil)) (set g (lambda () g))) (set i (+ i 1)))) " + k +
             ")";
    };
    const auto check = [&](const char *driver, std::size_t live) {
      EXPECT_EQ(tracked_objects(), fallback) << driver << ": tracked in the thread's own registry";
      collect_cycles(*reg);
      EXPECT_GE(tracked_objects(*reg), live) << driver
                                             << ": its live cycles are not in the "
                                                "scheduler's registry";
    };

    const int p1 = sched.execute(program(1), opts);
    for (int f = 0; f < 1000 && !finished(sched, p1); ++f)
      cvc::pump_exec_frame(sched, {}, 20000, 1.0);
    EXPECT_EQ(outcome_of(sched, p1), "1");
    check("pump_exec_frame", 1);

    const int p2 = sched.execute(program(2), opts);
    sched.sync_run();
    EXPECT_EQ(outcome_of(sched, p2), "2");
    check("sync_run", 2);

    const int p3 = sched.execute(program(3), opts);
    for (int s = 0; s < 100000 && !finished(sched, p3); ++s)
      sched.sync_step();
    EXPECT_EQ(outcome_of(sched, p3), "3");
    check("sync_step", 3);

    // A process snapshotted in the middle of its loop, with a closure bound in its let scope.
    const int p4 = sched.execute(std::string("(let ((h nil)) (set h (lambda () h)) (set j 0) "
                                             "(while (< j 300) (set j (+ j 1))) 4)"),
                                 opts);
    for (int s = 0; s < 60; ++s)
      sched.sync_step();
    ASSERT_FALSE(finished(sched, p4));
    cvc::state &snap = tree("rx03.snap");
    ASSERT_TRUE(sched.snapshot(p4, snap));
    ASSERT_TRUE(sched.kill(p4));
    collect_cycles(*reg);
    const std::size_t before_restore = tracked_objects(*reg);
    const int moved = sched.restore(snap, env);
    EXPECT_GT(tracked_objects(*reg), before_restore) << "the restore's scopes are not in the "
                                                        "scheduler's registry";
    EXPECT_EQ(tracked_objects(), fallback) << "restore: tracked in the thread's own registry";
    sched.sync_run();
    EXPECT_EQ(outcome_of(sched, moved), "4");
    check("restore", 3);
  }
}

// BF-1a (S1.3): the frozen builtins scope refuses a set.
TEST(Groups, BF01a_TheFrozenScopeRefusesASet) {
  const environment_ptr frozen = frozen_builtins();
  ASSERT_NE(frozen, nullptr);
  EXPECT_TRUE(frozen->frozen);
  EXPECT_THROW(frozen->set("str-concat", nil_value), std::logic_error);
  EXPECT_THROW(frozen->set("bf01a-new-name", nil_value), std::logic_error);
  EXPECT_EQ(frozen->lookup("bf01a-new-name"), nullptr);
  const value_t *kept = frozen->lookup("str-concat");
  ASSERT_NE(kept, nullptr);
  EXPECT_TRUE(std::holds_alternative<native_fn>(kept->v)) << "the builtin was overwritten";
  EXPECT_EQ(frozen_builtins(), frozen) << "one frozen scope per process";
}

// PO-2 (S1.4, S1-Q4): the pump owner's rule, "no owner, or me". A scope nests on one thread; while
// one is held, another thread's scope is refused with kPumpedElsewhereError and that thread sees
// the scheduler pumped elsewhere; once released, another thread takes it, and the first is then
// refused. A sync scheduler pumped from successive threads, each holding the owner while it steps
// (test_lt07's shape), works.
TEST(Groups, PO02_TheOwnerRule) {
  async_scheduler sched;
  // Another thread's view: whether it sees the owner held, and what taking a scope did.
  const auto from_another_thread = [&sched] {
    std::pair<bool, std::string> seen;
    std::thread t([&] {
      seen.first = sched.pumped_elsewhere();
      try {
        async_scheduler::pump_owner_scope scope(sched);
        seen.second = "taken";
      } catch (const std::runtime_error &e) {
        seen.second = e.what();
      }
    });
    t.join();
    return seen;
  };

  {
    async_scheduler::pump_owner_scope outer(sched);
    EXPECT_FALSE(sched.pumped_elsewhere());
    {
      std::unique_ptr<async_scheduler::pump_owner_scope> inner;
      EXPECT_NO_THROW(inner = std::make_unique<async_scheduler::pump_owner_scope>(sched));
      EXPECT_FALSE(sched.pumped_elsewhere());
      const auto seen = from_another_thread();
      EXPECT_TRUE(seen.first) << "nested";
      EXPECT_EQ(seen.second, kPumpedElsewhereError) << "nested";
    }
    const auto seen = from_another_thread(); // the inner scope restored the outer's owner
    EXPECT_TRUE(seen.first) << "after the nested scope";
    EXPECT_EQ(seen.second, kPumpedElsewhereError) << "after the nested scope";
  }
  const auto free_now = from_another_thread();
  EXPECT_FALSE(free_now.first) << "released";
  EXPECT_EQ(free_now.second, "taken") << "released";

  // Another thread holds it now: this one is refused, until it lets go.
  std::atomic<int> phase{0};
  std::thread holder([&] {
    async_scheduler::pump_owner_scope scope(sched);
    phase.store(1);
    const auto t0 = clock_type::now();
    while (phase.load() != 2 && seconds_since(t0) < kCeiling)
      std::this_thread::yield();
  });
  const auto t0 = clock_type::now();
  while (phase.load() != 1 && seconds_since(t0) < kCeiling)
    std::this_thread::yield();
  EXPECT_TRUE(sched.pumped_elsewhere());
  try {
    async_scheduler::pump_owner_scope scope(sched);
    ADD_FAILURE() << "a scope was taken while another thread held the owner";
  } catch (const std::runtime_error &e) {
    EXPECT_EQ(std::string(e.what()), kPumpedElsewhereError);
  }
  phase.store(2);
  holder.join();
  EXPECT_FALSE(sched.pumped_elsewhere());
  EXPECT_NO_THROW(async_scheduler::pump_owner_scope{sched});

  // The sync scheduler, stepped by four threads in turn and then run to its end here.
  scheduler sync;
  execute_options opts;
  opts.max_time = kProcessCeiling;
  const int pid =
      sync.execute(std::string("(begin (set i 0) (while (< i 400) (set i (+ i 1))) i)"), opts);
  for (int k = 0; k < 4; ++k) {
    std::string error;
    std::thread t([&] {
      try {
        scheduler::pump_owner_scope scope(sync);
        sync.run(50);
      } catch (const std::exception &e) {
        error = e.what();
      }
    });
    t.join();
    EXPECT_EQ(error, "") << "thread " << k;
  }
  {
    scheduler::pump_owner_scope scope(sync);
    sync.run();
  }
  const auto r = sync.get_result(pid);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(to_string(*r), "400");
}
