// Threaded execution, Stage 1 (docs/STATE_EXEC_THREADED.md 7, 8.2): execution groups and the
// app's exec_pool, the Groups suite (S1.md section 4: the GR, FF, RX, RB, AS, BF, OC, DT, GL, PO
// and PA cases). The sanitizer jobs run this binary (ci.yml's SAN_CXX_TESTS) and require each
// case's line. No case name contains "Stress": run.sh filters those out.

#include <algorithm>
#include <atomic>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/core/async_lane.h> // cvc::pump_exec_frame
#include <cvc/core/exec_pool.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_host.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/exec_group.h>
#include <cvc/state/state_exec/heap.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/parser.h>
#include <cvc/state/state_exec/scheduler.h>
#include <functional>
#include <gtest/gtest.h>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <set>
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

// --- Groups and the fork-join pool (S1.5, S1.6, S1.7) ---

namespace {

constexpr double kNoTimeLimit = std::numeric_limits<double>::infinity();

#ifdef NDEBUG
constexpr bool kDebugBuild = false;
#else
constexpr bool kDebugBuild = true;
#endif

// Set an environment variable for a case (CVC_EXEC_FORCE_MOVE, which the pool reads when it is
// built), and restore what it was.
class scoped_env_var {
public:
  scoped_env_var(const char *name, const char *value) : name_(name) {
    if (const char *old = std::getenv(name))
      old_ = old;
    set(value);
  }
  ~scoped_env_var() { set(old_ ? old_->c_str() : nullptr); }
  scoped_env_var(const scoped_env_var &) = delete;
  scoped_env_var &operator=(const scoped_env_var &) = delete;

private:
  void set(const char *value) {
#ifdef _WIN32
    _putenv_s(name_.c_str(), value ? value : "");
#else
    if (value)
      setenv(name_.c_str(), value, 1);
    else
      unsetenv(name_.c_str());
#endif
  }
  std::string name_;
  std::optional<std::string> old_;
};

// An app whose exec_pool is built first, with `threads` workers asked for, before anything makes
// its lock manager, so threads above 0 make that manager shared (S1-Q1). Its tree charges an
// async call one step and no CPU time (S1.13), so step counts compare across runs.
struct pool_app {
  cvc::app app;
  cvc::state &tree;
  cvc::exec_pool &pool;

  explicit pool_app(unsigned threads, bool deterministic = false)
      : tree(cvc::state::instance(app)), pool(build(app, tree, threads, deterministic)) {}

  static cvc::exec_pool &build(cvc::app &a, cvc::state &t, unsigned threads, bool deterministic) {
    t("state_exec.defaults.limits.async_steps_per_cpu_ms").value(std::string("0"));
    cvc::pool_options opts;
    opts.threads = threads;
    opts.deterministic = deterministic;
    return a.exec_pool(opts);
  }
  unsigned effective() const { return pool.stats().effective_threads; }
};

// A program world's environment, chrooted at `chroot` on `tree` and run by `sched`: the
// intrinsics (spawn through make_spawn_context, ictx.host `host`); (keep x), which binds x as
// `kept` in this environment, so it outlives the program (a program's own set binds in its own
// scope); and (twice x) and (twice-async x), async functions on an inline executor.
void setup_env(intrinsics_context &ictx, const environment_ptr &env, async_scheduler &sched,
               async_host &host, cvc::state &tree, const std::string &chroot) {
  ictx.sched = &sched;
  ictx.proc = make_process();
  ictx.proc->pid = 0;
  apply_chroot(ictx, tree, chroot);
  ictx.host = &host;
  ictx.spawn_context = make_spawn_context();
  register_intrinsics(env, &ictx);
  builtins::register_fn(env, "keep",
                        [held = std::weak_ptr<environment>(env)](std::span<const value_t> a) {
                          if (const environment_ptr e = held.lock())
                            e->set("kept", a.empty() ? nil_value : a[0]);
                          return nil_value;
                        });
  async_options inline_call;
  inline_call.where = executor::inline_call();
  register_async_fn(
      env, host, "twice",
      [](std::span<const value_t> a) -> async_kernel {
        const int64_t x = a.empty() ? 0 : std::get<int64_t>(a[0].v);
        return [x](async_call &) -> value_t { return value_t(2 * x); };
      },
      inline_call);
}

// One group of a case: an exec_group of the app, named and chrooted `name`, with setup_env's
// environment. `tweak` sets more options (the test hooks, a seed, a base). The group goes first,
// then the environment and the context its natives point at.
struct group_world {
  intrinsics_context ictx;
  environment_ptr env = builtins::make_default_environment();
  std::unique_ptr<exec_group> group;

  group_world(pool_app &w, const std::string &name, group_affinity affinity = group_affinity::any,
              const std::function<void(exec_group::options &)> &tweak = {}) {
    exec_group::options opts;
    opts.name = name;
    opts.owner = name;
    opts.affinity = affinity;
    opts.app = &w.app;
    opts.watch_root = &w.tree; // the app's root, as null means
    if (tweak)
      tweak(opts);
    group = std::make_unique<exec_group>(std::move(opts));
    setup_env(ictx, env, group->sched(), group->host(), w.tree, name);
  }
  group_world(const group_world &) = delete;
  group_world &operator=(const group_world &) = delete;

  exec_group &g() { return *group; }
  async_scheduler &sched() { return group->sched(); }

  int submit(const std::string &script, std::function<void(value_t)> on_complete = {}) {
    execute_options opts;
    opts.env = environment::extend(env);
    opts.max_time = kProcessCeiling;
    opts.on_complete = std::move(on_complete);
    return group->execute(script, opts);
  }
  bool done() {
    for (const process_info &i : sched().list_processes())
      if (!is_finished(i.status))
        return false;
    return true;
  }
  std::string outcome(int pid) { return outcome_of(sched(), pid); }
};

// The same world on a bare async_scheduler, configured as a group's is (the host parser and data
// defaults, the app's root watched), and pumped as exec_group::pump pumps (DT-2).
struct bare_world {
  intrinsics_context ictx;
  environment_ptr env = builtins::make_default_environment();
  async_scheduler sched;
  std::unique_ptr<async_host> host;

  bare_world(pool_app &w, const std::string &name) {
    sched.set_watch_root(&w.tree);
    sched.set_parser_defaults(async_scheduler::host_parser_defaults());
    sched.set_data_defaults(async_scheduler::host_data_defaults());
    host = std::make_unique<async_host>(sched, &w.app);
    setup_env(ictx, env, sched, *host, w.tree, name);
  }
  ~bare_world() { host->shutdown(); }

  int submit(const std::string &script) {
    execute_options opts;
    opts.env = environment::extend(env);
    opts.max_time = kProcessCeiling;
    return sched.execute(script, opts);
  }
  void pump(const pump_budget &b) {
    sched.drain_ingress();
    sched.wake_awaiting();
    sched.sync_run_steps(b.steps, b.seconds);
  }
  bool done() {
    for (const process_info &i : sched.list_processes())
      if (!is_finished(i.status))
        return false;
    return true;
  }
};

// GR-1's programs for group k (td 8.2): fib, an exception loop, closure cycles (one kept in the
// group's env), set-attr on its own dict (which then holds itself), state-set/get/delete under
// the group's chroot, a broadcast to two receivers, a fork, a spawn (through make_spawn_context)
// whose child sends to its parent, and awaits on inline-executor kernels. No result names a pid:
// pids differ between runs.
std::vector<std::string> oracle_programs(int k) {
  const std::string n = std::to_string(10 + k % 5);
  return {
      "(begin (defun fib (n) (if (< n 2) n (+ (fib (- n 1)) (fib (- n 2))))) (fib " + n + "))",
      "(begin (set n 0) (for i (range 40) (try (raise (ValueError \"x\")) "
      "(catch (ValueError e) (set n (+ n 1))))) n)",
      "(begin (set i 0) (while (< i 300) (begin (let ((h nil)) (set h (lambda () h))) "
      "(set i (+ i 1)))) (keep (let ((h nil)) (set h (lambda () h)) h)) i)",
      "(begin (set d (dict)) (for i (range 60) (set-attr d (str i) (* i i))) "
      "(set-attr d \"self\" d) (get-attr d \"59\"))",
      "(begin (state-set \"x.y\" (str " + n +
          ")) (set v (state-get \"x.y\")) "
          "(state-delete \"x\") (list v (is-null (state-get \"x.y\"))))",
      "(get-attr (msg-recv \"bc\") \"payload\")",
      "(get-attr (msg-recv \"bc\") \"payload\")",
      "(begin (for i (range 30) nil) (msg-send \"bc\" \"hello\") \"sent\")",
      "(begin (set r (fork)) (if (= r 0) (begin (for i (range 10) nil) \"child\") \"parent\"))",
      "(begin (spawn \"(msg-send \\\"sp\\\" \\\"from-child\\\")\") "
      "(get-attr (msg-recv \"sp\") \"payload\"))",
      "(begin (set f (twice-async 21)) (list (await f) (twice 4) "
      "(await-all (list (twice-async 1) (twice-async 2)))))",
  };
}

// What a group came to, to compare with another run of it: each process in pid order (pids
// increase in creation order within a group), as its rank, status, steps and outcome; and, when
// it has a registry, the objects the registry tracks after a final collection.
struct group_record {
  std::vector<std::string> procs;
  std::optional<std::size_t> tracked;
};

group_record record(async_scheduler &sched, cycle_registry *reg) {
  group_record r;
  std::vector<process_info> infos = sched.list_processes();
  std::sort(infos.begin(), infos.end(),
            [](const process_info &a, const process_info &b) { return a.pid < b.pid; });
  for (std::size_t i = 0; i < infos.size(); ++i)
    r.procs.push_back("#" + std::to_string(i) + " " + to_string(infos[i].status) + " steps " +
                      std::to_string(infos[i].step_count) + ": " + outcome_of(sched, infos[i].pid));
  if (reg) {
    collect_cycles(*reg);
    r.tracked = tracked_objects(*reg);
  }
  return r;
}
group_record record(group_world &gw) { return record(gw.sched(), gw.g().registry()); }

// Compare a group's run with its solo run. A group without a registry is reported once a case.
void expect_same(const group_record &got, const group_record &solo, const std::string &label,
                 bool &no_registry_reported) {
  EXPECT_EQ(got.procs, solo.procs) << label;
  if (got.tracked && solo.tracked)
    EXPECT_EQ(*got.tracked, *solo.tracked) << label << ": tracked after a final collection";
  else if (!no_registry_reported) {
    no_registry_reported = true;
    ADD_FAILURE() << label << ": the group has no cycle registry";
  }
}

using budget_fn = std::function<pump_budget(std::size_t pump)>;

pump_budget steps_only(std::uint64_t steps) { return {steps, kNoTimeLimit}; }

std::optional<process_status> status_of(const async_scheduler &sched, int pid) {
  const auto i = sched.get_process_info(pid);
  return i ? std::optional<process_status>(i->status) : std::nullopt;
}

// Run a group alone on this thread, pump i with budget(i), until its processes have ended.
void pump_alone(group_world &gw, const budget_fn &budget, std::size_t max_pumps = 200000) {
  for (std::size_t i = 0; i < max_pumps && !gw.done(); ++i)
    gw.g().pump(budget(i));
}

// Pump `worlds` on the pool: one pump_all a frame, over the groups not done, with budget(frame)
// and no deadline that matters, until all are done. Returns the frames.
std::size_t pump_pooled(pool_app &w, const std::vector<group_world *> &worlds,
                        const budget_fn &budget, std::size_t max_frames = 200000) {
  const auto no_deadline = clock_type::now() + std::chrono::hours(1);
  for (std::size_t frame = 0; frame < max_frames; ++frame) {
    std::vector<exec_group *> live;
    for (group_world *gw : worlds)
      if (!gw->done())
        live.push_back(&gw->g());
    if (live.empty())
      return frame;
    w.pool.pump_all(live, budget(frame), no_deadline);
  }
  return max_frames;
}

// How a case makes group k and what the group runs: its options, then its environment's extra
// natives and its programs (by default, oracle_programs(k)).
struct group_plan {
  std::function<void(int k, exec_group::options &)> options;
  std::function<void(int k, group_world &)> setup;
};

void apply_plan(const group_plan &plan, int k, group_world &gw) {
  if (plan.setup)
    plan.setup(k, gw);
  else
    for (const std::string &p : oracle_programs(k))
      gw.submit(p);
}

// The oracle (GR-1, GR-2, GR-3, GR-12): each group run alone on this thread, pump i with
// budget(i), until its processes have ended...
std::vector<group_record> solo_records(int groups, const budget_fn &budget,
                                       const group_plan &plan = {}) {
  std::vector<group_record> solo;
  pool_app w(0);
  for (int k = 0; k < groups; ++k) {
    group_world gw(w, "g" + std::to_string(k));
    apply_plan(plan, k, gw);
    pump_alone(gw, budget);
    EXPECT_TRUE(gw.done()) << "group " << k << " alone never ended";
    solo.push_back(record(gw));
  }
  return solo;
}

// ...then all of them on a pool of `threads` workers: every group's record equals its solo run.
void check_against_solo(const char *label, int groups, unsigned threads,
                        const std::vector<group_record> &solo, const budget_fn &budget,
                        const group_plan &plan, bool &no_registry_reported) {
  pool_app w(threads);
  std::vector<std::unique_ptr<group_world>> worlds;
  std::vector<group_world *> raw;
  for (int k = 0; k < groups; ++k) {
    worlds.push_back(std::make_unique<group_world>(w, "g" + std::to_string(k), group_affinity::any,
                                                   [&](exec_group::options &o) {
                                                     if (plan.options)
                                                       plan.options(k, o);
                                                   }));
    raw.push_back(worlds.back().get());
    apply_plan(plan, k, *worlds.back());
  }
  const auto t0 = clock_type::now();
  const std::size_t frames = pump_pooled(w, raw, budget);
  std::printf("%s: %d groups at threads %u (effective %u): %zu frames, %.3f s\n", label, groups,
              threads, w.effective(), frames, seconds_since(t0));
  std::fflush(stdout);
  for (int k = 0; k < groups; ++k) {
    const std::string where = std::string(label) + ", " + std::to_string(groups) +
                              " groups at threads " + std::to_string(threads) + ", group " +
                              std::to_string(k);
    EXPECT_TRUE(worlds[k]->done()) << where << " never ended";
    expect_same(record(*worlds[k]), solo[k], where, no_registry_reported);
  }
}

// Hold the caller of a pump_all inside a host-affinity group's pump until `go` is set (or
// kCeiling passes), so the pool's workers take the call's other groups: the caller pumps the
// host-affinity groups first (S1.6). Its one process loops, so it always has a pump to take.
struct caller_hold {
  std::atomic<bool> go{false};
  std::unique_ptr<group_world> gw;

  caller_hold(pool_app &w, const std::string &name) {
    gw = std::make_unique<group_world>(w, name, group_affinity::host,
                                       [this](exec_group::options &o) {
                                         o.pump_hook_for_test = [this](exec_group &) {
                                           const auto t0 = clock_type::now();
                                           while (!go.load() && seconds_since(t0) < kCeiling)
                                             std::this_thread::yield();
                                         };
                                       });
    gw->submit("(while #t nil)");
  }
  exec_group &g() { return gw->g(); }
};

// The threads a group's pumps ran on, in order (its pump_hook_for_test records them).
struct pump_threads {
  std::mutex m;
  std::vector<std::thread::id> ids;
  void note() {
    std::lock_guard<std::mutex> lock(m);
    ids.push_back(std::this_thread::get_id());
  }
  std::vector<std::thread::id> get() {
    std::lock_guard<std::mutex> lock(m);
    return ids;
  }
};

// Where a native ran: its thread and the group the thread was pumping then (exec_group::current(),
// null outside a pump).
struct native_call {
  std::thread::id thread;
  exec_group *in = nullptr;
};

struct native_calls {
  std::mutex m;
  std::vector<native_call> calls;
  void note() {
    std::lock_guard<std::mutex> lock(m);
    calls.push_back({std::this_thread::get_id(), exec_group::current()});
  }
  std::vector<native_call> get() {
    std::lock_guard<std::mutex> lock(m);
    return calls;
  }
};

} // namespace

// GR-1 (td 8.2): the oracle. G groups (2, 8, 32) of oracle_programs, each run alone on this thread
// with steps-only budgets, then all on the pool at threads 1, 2, 4, 8, 16 (the effective count
// printed): every group's processes (results, steps, statuses) and its registry's tracked objects
// after a final collection equal its solo run.
TEST(Groups, GR01_EveryGroupEqualsItsSoloRun) {
  const std::vector<int> group_counts =
      under_sanitizer() ? std::vector<int>{2, 8} : std::vector<int>{2, 8, 32};
  const std::vector<unsigned> thread_counts =
      under_sanitizer() ? std::vector<unsigned>{1, 4} : std::vector<unsigned>{1, 2, 4, 8, 16};
  const budget_fn budget = [](std::size_t) { return steps_only(97); };
  const std::vector<group_record> solo = solo_records(group_counts.back(), budget);
  bool no_registry_reported = false;
  for (const int groups : group_counts)
    for (const unsigned threads : thread_counts)
      check_against_solo("GR-1", groups, threads, solo, budget, {}, no_registry_reported);
}

// GR-2 (td 8.2): GR-1 with CVC_EXEC_FORCE_MOVE (each group pumped on a different worker each
// pump) and seeded random step budgets, the solo runs given the same budget for each pump: the
// same.
TEST(Groups, GR02_ForcedMovesAndRandomBudgets) {
  scoped_env_var force("CVC_EXEC_FORCE_MOVE", "1");
  std::mt19937 rng(2026);
  std::uniform_int_distribution<std::uint64_t> steps(1, 400);
  std::vector<pump_budget> budgets;
  for (int i = 0; i < 4096; ++i)
    budgets.push_back(steps_only(steps(rng)));
  const budget_fn budget = [&budgets](std::size_t i) { return budgets[i % budgets.size()]; };
  const int groups = under_sanitizer() ? 4 : 8;
  const std::vector<group_record> solo = solo_records(groups, budget);
  bool no_registry_reported = false;
  for (const unsigned threads :
       under_sanitizer() ? std::vector<unsigned>{2} : std::vector<unsigned>{2, 4, 8})
    check_against_solo("GR-2", groups, threads, solo, budget, {}, no_registry_reported);
}

// GR-3 (td 8.2): GR-1 with host-affinity groups mixed in (every third), each group also running
// a loop of (note-thread), a native recording the thread it runs on: the same as the solo runs,
// and every pump of a host-affinity group, and every call of its native, ran on the caller's
// thread.
TEST(Groups, GR03_HostAffinityGroupsStayOnTheCaller) {
  constexpr int kGroups = 8;
  const budget_fn budget = [](std::size_t) { return steps_only(61); };
  const std::string loop = "(begin (for i (range 200) (note-thread)) \"noted\")";
  group_plan solo_plan;
  solo_plan.setup = [&](int k, group_world &gw) {
    builtins::register_fn(gw.env, "note-thread",
                          [](std::span<const value_t>) -> value_t { return nil_value; });
    for (const std::string &p : oracle_programs(k))
      gw.submit(p);
    gw.submit(loop);
  };
  const std::vector<group_record> solo = solo_records(kGroups, budget, solo_plan);
  std::vector<std::unique_ptr<pump_threads>> pumps;
  std::vector<std::unique_ptr<native_calls>> notes;
  for (int k = 0; k < kGroups; ++k) {
    pumps.push_back(std::make_unique<pump_threads>());
    notes.push_back(std::make_unique<native_calls>());
  }
  group_plan plan;
  plan.options = [&](int k, exec_group::options &o) {
    if (k % 3 == 0)
      o.affinity = group_affinity::host;
    pump_threads *p = pumps[k].get();
    o.pump_hook_for_test = [p](exec_group &) { p->note(); };
  };
  plan.setup = [&](int k, group_world &gw) {
    native_calls *n = notes[k].get();
    builtins::register_fn(gw.env, "note-thread", [n](std::span<const value_t>) -> value_t {
      n->note();
      return nil_value;
    });
    for (const std::string &p : oracle_programs(k))
      gw.submit(p);
    gw.submit(loop);
  };
  bool no_registry_reported = false;
  check_against_solo("GR-3", kGroups, 4, solo, budget, plan, no_registry_reported);
  const auto caller = std::this_thread::get_id();
  for (int k = 0; k < kGroups; k += 3) {
    const std::vector<std::thread::id> ids = pumps[k]->get();
    EXPECT_FALSE(ids.empty()) << "group " << k;
    for (const std::thread::id &id : ids)
      EXPECT_EQ(id, caller) << "group " << k << ": a host-affinity group was pumped off the caller";
    const std::vector<native_call> calls = notes[k]->get();
    EXPECT_EQ(calls.size(), 200u) << "group " << k;
    for (const native_call &c : calls)
      EXPECT_EQ(c.thread, caller) << "group " << k << ": its native ran off the caller";
  }
}

// GR-4 (td 8.2): a cycle made in group A while group B, on another thread of the pool, makes and
// collects its own garbage. A's process makes its cycle and then parks on msg-recv, so no process
// of A ends and A's registry is not collected (S1.1's idle collection waits for one to end): B's
// collections and B's tracked objects never see A's cycle (its weak observer is alive after B's
// last collection), while A's registry tracks it. Released, A's process ends, and A's registry
// frees the cycle (its idle collection, or a collection of it here).
TEST(Groups, GR04_ACycleBelongsToItsGroup) {
  pool_app w(2);
  scope_observer observer;
  group_world a(w, "gr04a"), b(w, "gr04b");
  observer.install(a.env);
  const int made = a.submit("(begin (let ((h nil)) (set h (lambda () h)) (observe h)) "
                            "(msg-recv \"gr04.go\") 1)");
  const int churn = b.submit("(begin (set i 0) (while (< i 3000) (begin (let ((h nil)) "
                             "(set h (lambda () h))) (set i (+ i 1)))) i)");
  const auto no_deadline = clock_type::now() + std::chrono::hours(1);
  for (int f = 0;
       f < 100000 && !(b.done() && status_of(a.sched(), made) == process_status::waiting); ++f)
    w.pool.pump_all(std::vector<exec_group *>{&a.g(), &b.g()}, steps_only(500), no_deadline);
  EXPECT_EQ(b.outcome(churn), "3000");
  ASSERT_EQ(status_of(a.sched(), made), process_status::waiting) << a.outcome(made);
  ASSERT_EQ(observer.count(), 1u);
  cycle_registry *ra = a.g().registry();
  cycle_registry *rb = b.g().registry();
  if (!ra || !rb) {
    observer.release();
    FAIL() << "a group has no cycle registry";
  }
  const cycle_stats sb = collect_cycles(*rb);
  std::printf("GR-4: B's last collection freed %zu, B tracks %zu, A tracks %zu\n", sb.freed,
              tracked_objects(*rb), tracked_objects(*ra));
  EXPECT_EQ(tracked_objects(*rb), 0u) << "B's registry keeps something";
  EXPECT_EQ(observer.alive(), 1u) << "B's collections freed A's cycle";
  EXPECT_GE(tracked_objects(*ra), 1u) << "A's registry does not track A's cycle";

  a.sched().post_message(resolve_channel_key("gr04a", "gr04.go"), value_t(std::string("go")));
  for (int f = 0; f < 1000 && !finished(a.sched(), made); ++f)
    w.pool.pump_all(std::vector<exec_group *>{&a.g()}, steps_only(500), no_deadline);
  EXPECT_EQ(a.outcome(made), "1");
  collect_cycles(*ra); // nothing left to free if A's idle collection already did
  EXPECT_EQ(observer.alive(), 0u) << "A's registry did not free its cycle";
  observer.release();
}

// GR-5 (td 8.2, 4.7): a group destroyed after its last process frees its registry and every cycle
// in it, those through its global env included. Its process runs env-less (its global env extends
// the group's, whose base is frozen, as the builtins are): its top-level defun's closure and that
// global env hold each other, the env holds a let scope's cycle as `keep`, and the process's
// result is the closure, so all of it stays alive in the group's process table. A third cycle is
// garbage. Weak observers on all three expire once the group is gone.
TEST(Groups, GR05_ARegistryDiesWithItsGroup) {
  pool_app w(0);
  scope_observer observer;
  const environment_ptr base = builtins::make_default_environment();
  observer.install(base);
  base->frozen = true;
  auto gw = std::make_unique<exec_group>([&] {
    exec_group::options o;
    o.name = "gr05";
    o.owner = "gr05";
    o.app = &w.app;
    o.watch_root = &w.tree;
    o.base = base;
    return o;
  }());
  execute_options opts;
  opts.max_time = kProcessCeiling;
  const int pid =
      gw->execute(std::string("(begin (defun f () 1) (observe f) "
                              "(set keep (let ((h nil)) (set h (lambda () h)) h)) (observe keep) "
                              "(let ((g nil)) (set g (lambda () g)) (observe g)) f)"),
                  opts);
  for (int i = 0; i < 1000 && !finished(gw->sched(), pid); ++i)
    gw->pump(steps_only(100));
  ASSERT_TRUE(finished(gw->sched(), pid));
  EXPECT_EQ(status_of(gw->sched(), pid), process_status::terminated)
      << outcome_of(gw->sched(), pid);
  EXPECT_EQ(observer.count(), 3u);
  if (cycle_registry *reg = gw->registry())
    EXPECT_GE(tracked_objects(*reg), 2u) << "the live cycles are not in the group's registry";
  else
    ADD_FAILURE() << "the group has no cycle registry";
  EXPECT_GE(observer.alive(), 2u) << "the result's cycles went while the group lived";
  gw.reset();
  EXPECT_EQ(observer.alive(), 0u) << "cycles outlived their group";
  observer.release();
}

// GR-6 (td 8.2, 4.10): math.random in a group with a fixed seed, pumped a few steps at a time by
// a different worker each pump (CVC_EXEC_FORCE_MOVE), gives the sequence a one-thread run of the
// same seed gives.
TEST(Groups, GR06_ARandomStreamFollowsItsGroup) {
  scoped_env_var force("CVC_EXEC_FORCE_MOVE", "1");
  const std::string program =
      "(begin (set out (list)) (for i (range 40) (append out (math.random))) out)";
  const auto seeded = [](exec_group::options &o) { o.seed = 1234; };
  std::string alone;
  {
    pool_app w(0);
    group_world gw(w, "gr06", group_affinity::any, seeded);
    const int pid = gw.submit(program);
    pump_alone(gw, [](std::size_t) { return steps_only(3); });
    alone = gw.outcome(pid);
  }
  pool_app w(4);
  group_world gw(w, "gr06", group_affinity::any, seeded);
  group_world other(w, "gr06other"); // a second group, so a pump_all has work for two
  const int pid = gw.submit(program);
  other.submit("(begin (set i 0) (while (< i 5000) (set i (+ i 1))) i)");
  pump_pooled(w, {&gw, &other}, [](std::size_t) { return steps_only(3); });
  std::printf("GR-6: threads 4 (effective %u)\n", w.effective());
  EXPECT_EQ(gw.outcome(pid), alone);
  EXPECT_NE(alone.find('.'), std::string::npos) << alone; // floats, not an error
}

// GR-7 (td 8.2, B32): pids from 8 groups and the app scheduler on one pool, submitted in turn and
// spawned during the pumps on several threads: unique app-wide, and increasing within each group
// in the order they were submitted.
TEST(Groups, GR07_PidsAreUniqueAcrossGroups) {
  pool_app w(4);
  async_scheduler &app_sched = w.app.exec_scheduler();
  constexpr int kGroups = 8;
  std::vector<std::unique_ptr<group_world>> worlds;
  std::vector<group_world *> raw;
  for (int k = 0; k < kGroups; ++k) {
    worlds.push_back(std::make_unique<group_world>(w, "gr07g" + std::to_string(k)));
    raw.push_back(worlds.back().get());
  }
  std::vector<std::vector<int>> submitted(kGroups);
  std::vector<int> app_pids;
  for (int round = 0; round < 5; ++round) {
    for (int k = 0; k < kGroups; ++k)
      submitted[k].push_back(worlds[k]->submit(
          "(begin (spawn \"1\") (spawn \"(for i (range 20) nil)\") (for i (range 50) nil) 1)"));
    app_pids.push_back(app_sched.execute(std::string("1")));
  }
  pump_pooled(w, raw, [](std::size_t) { return steps_only(40); });
  app_sched.sync_run();
  std::set<int> all;
  std::size_t count = 0;
  for (int k = 0; k < kGroups; ++k) {
    EXPECT_TRUE(std::is_sorted(submitted[k].begin(), submitted[k].end()) &&
                std::adjacent_find(submitted[k].begin(), submitted[k].end()) == submitted[k].end())
        << "group " << k << "'s pids do not increase";
    for (const process_info &i : worlds[k]->sched().list_processes()) {
      all.insert(i.pid);
      ++count;
    }
  }
  for (const process_info &i : app_sched.list_processes()) {
    all.insert(i.pid);
    ++count;
  }
  EXPECT_TRUE(std::is_sorted(app_pids.begin(), app_pids.end()));
  EXPECT_EQ(count, static_cast<std::size_t>(kGroups * 5 * 3 + 5));
  EXPECT_EQ(all.size(), count) << "a pid was issued twice";
}

// GR-8 (td 8.2, 4.10): pump_all of 16 groups, each with 0.1 s of work a pump, threads 4, a frame
// deadline 0.1 s away. The groups that did not start are counted in stats().skipped, and go first
// in the next frame. The frame's time is printed and bounded only by a generous ceiling (rule 7).
TEST(Groups, GR08_TheFrameDeadlineBoundsAFrame) {
  pool_app w(4);
  constexpr int kGroups = 16;
  std::atomic<int> frame{0};
  std::mutex m;
  std::vector<std::vector<int>> pumped(kGroups); // the frames each group was pumped in
  std::vector<std::unique_ptr<group_world>> worlds;
  std::vector<exec_group *> groups;
  for (int k = 0; k < kGroups; ++k) {
    worlds.push_back(std::make_unique<group_world>(
        w, "gr08g" + std::to_string(k), group_affinity::any, [&, k](exec_group::options &o) {
          o.pump_hook_for_test = [&, k](exec_group &) {
            std::lock_guard<std::mutex> lock(m);
            pumped[k].push_back(frame.load());
          };
        }));
    worlds.back()->submit("(while #t nil)");
    groups.push_back(&worlds.back()->g());
  }
  const pump_budget budget{std::numeric_limits<std::uint64_t>::max() / 2, 0.1};
  const auto pumped_in = [&](int f) {
    std::set<int> s;
    std::lock_guard<std::mutex> lock(m);
    for (int k = 0; k < kGroups; ++k)
      if (std::find(pumped[k].begin(), pumped[k].end(), f) != pumped[k].end())
        s.insert(k);
    return s;
  };
  const std::uint64_t skipped0 = w.pool.stats().skipped;
  frame.store(1);
  const auto t0 = clock_type::now();
  w.pool.pump_all(groups, budget, clock_type::now() + std::chrono::milliseconds(100));
  const double took = seconds_since(t0);
  const std::set<int> first = pumped_in(1);
  const std::uint64_t skipped1 = w.pool.stats().skipped - skipped0;
  std::printf("GR-8: frame 1 took %.3f s at threads 4 (effective %u): %zu pumped, %llu skipped\n",
              took, w.effective(), first.size(), static_cast<unsigned long long>(skipped1));
  EXPECT_LT(took, kCeiling);
  EXPECT_EQ(skipped1, static_cast<std::uint64_t>(kGroups) - first.size());
  EXPECT_GE(skipped1, 1u) << "every group started before the deadline";
  std::set<int> skipped;
  for (int k = 0; k < kGroups; ++k)
    if (!first.count(k))
      skipped.insert(k);

  frame.store(2);
  w.pool.pump_all(groups, budget, clock_type::now() + std::chrono::milliseconds(100));
  const std::set<int> second = pumped_in(2);
  std::printf("GR-8: frame 2 pumped %zu\n", second.size());
  EXPECT_FALSE(second.empty());
  if (second.size() <= skipped.size()) {
    for (const int k : second)
      EXPECT_TRUE(skipped.count(k)) << "group " << k << " started ahead of a skipped group";
  } else {
    for (const int k : skipped)
      EXPECT_TRUE(second.count(k)) << "skipped group " << k << " did not start next frame";
  }
}

// GR-9 (td 8.2, 4.4): with the groups' last pump durations injected under 100 us in total,
// pump_all wakes no helper (stats().helper_wakes does not move); injected above it, it wakes
// helpers.
TEST(Groups, GR09_SmallFramesWakeNoHelper) {
  pool_app w(4);
  std::vector<std::unique_ptr<group_world>> worlds;
  std::vector<exec_group *> groups;
  for (int k = 0; k < 8; ++k) {
    worlds.push_back(std::make_unique<group_world>(w, "gr09g" + std::to_string(k)));
    worlds.back()->submit("(begin (set i 0) (while (< i 100000) (set i (+ i 1))) i)");
    groups.push_back(&worlds.back()->g());
  }
  const auto no_deadline = clock_type::now() + std::chrono::hours(1);
  for (exec_group *g : groups)
    w.pool.set_last_pump_for_test(*g, 5e-6); // 40 us in all
  const std::uint64_t wakes0 = w.pool.stats().helper_wakes;
  w.pool.pump_all(groups, steps_only(50), no_deadline);
  const std::uint64_t below = w.pool.stats().helper_wakes - wakes0;
  for (exec_group *g : groups)
    w.pool.set_last_pump_for_test(*g, 1e-3); // 8 ms in all
  w.pool.pump_all(groups, steps_only(50), no_deadline);
  const std::uint64_t above = w.pool.stats().helper_wakes - wakes0 - below;
  std::printf("GR-9: threads 4 (effective %u): %llu helper wakes for 40 us, %llu for 8 ms\n",
              w.effective(), static_cast<unsigned long long>(below),
              static_cast<unsigned long long>(above));
  EXPECT_EQ(below, 0u);
  if (w.pool.threads() == 0)
    ADD_FAILURE() << "the pool has no worker to wake";
  else
    EXPECT_GT(above, 0u);
}

// GR-12 (td 8.2, 4.17): pump_hook_for_test throws a C++ exception out of one group's pump, on a
// worker and, in a second pool, on the caller (a host-affinity group). That group is in
// failed_groups(), its processes are killed with "internal error", pump_all does not throw, and
// every other group's results equal their solo runs.
TEST(Groups, GR12_AnExceptionEscapingAPumpFailsOnlyItsGroup) {
  constexpr int kOthers = 4;
  const budget_fn budget = [](std::size_t) { return steps_only(97); };
  const std::vector<group_record> solo = solo_records(kOthers, budget);
  const auto caller = std::this_thread::get_id();
  bool no_registry_reported = false;
  for (const bool on_worker : {true, false}) {
    SCOPED_TRACE(on_worker ? "thrown on a worker" : "thrown on the caller");
    pool_app w(4);
    std::unique_ptr<caller_hold> hold;
    if (on_worker) {
      if (w.pool.threads() == 0) {
        ADD_FAILURE() << "the pool has no worker";
        continue;
      }
      hold = std::make_unique<caller_hold>(w, "gr12hold");
    }
    std::atomic<int> thrown{0};
    std::thread::id thrown_on;
    group_world target(w, "gr12target", on_worker ? group_affinity::any : group_affinity::host,
                       [&](exec_group::options &o) {
                         o.pump_hook_for_test = [&](exec_group &) {
                           if (hold)
                             hold->go.store(true);
                           if (thrown.fetch_add(1) == 0) {
                             thrown_on = std::this_thread::get_id();
                             throw std::runtime_error("gr12: an exception out of a pump");
                           }
                         };
                       });
    const int busy = target.submit("(while #t nil)");
    const int parked = target.submit("(begin (msg-recv \"never\") 1)");
    std::vector<std::unique_ptr<group_world>> others;
    std::vector<group_world *> raw;
    for (int k = 0; k < kOthers; ++k) {
      others.push_back(std::make_unique<group_world>(w, "g" + std::to_string(k)));
      raw.push_back(others.back().get());
      for (const std::string &p : oracle_programs(k))
        others.back()->submit(p);
    }
    std::vector<exec_group *> first;
    if (hold) {
      first.push_back(&hold->g());
      w.pool.set_last_pump_for_test(hold->g(), 1e-3);
      w.pool.set_last_pump_for_test(target.g(), 1e-3);
    }
    first.push_back(&target.g());
    for (group_world *gw : raw)
      first.push_back(&gw->g());
    EXPECT_NO_THROW(w.pool.pump_all(first, budget(0), clock_type::now() + std::chrono::hours(1)));
    ASSERT_GE(thrown.load(), 1);
    if (on_worker)
      EXPECT_NE(thrown_on, caller) << "the throwing pump ran on the caller";
    else
      EXPECT_EQ(thrown_on, caller);
    const std::vector<exec_group *> failed = w.pool.failed_groups();
    EXPECT_EQ(failed, std::vector<exec_group *>{&target.g()});
    EXPECT_TRUE(target.g().failed());
    for (const int pid : {busy, parked}) {
      EXPECT_EQ(status_of(target.sched(), pid), process_status::killed);
      EXPECT_NE(target.sched().get_exit_error(pid).value_or("").find("internal error"),
                std::string::npos)
          << target.sched().get_exit_error(pid).value_or("");
    }
    // The rest run on to their ends without the failed group, frame 1 counted.
    const std::size_t frames = pump_pooled(w, raw, [&](std::size_t i) { return budget(i + 1); });
    std::printf("GR-12 (%s): effective %u, %zu more frames\n", on_worker ? "worker" : "caller",
                w.effective(), frames);
    for (int k = 0; k < kOthers; ++k)
      expect_same(record(*others[k]), solo[k], "group " + std::to_string(k), no_registry_reported);
  }
}

// RB-1 (S1.1, section 0 item 9): two groups run one AST (submitted through their schedulers,
// which do not copy it), so their closures come from the same lambda form, whose atom body the
// closure-body cache shares per thread. Pumped alternately on one thread, and then on the pool
// with CVC_EXEC_FORCE_MOVE, each group's body lists are its own: none is the other's.
TEST(Groups, RB01_ClosureBodiesAreNotSharedAcrossGroups) {
  const value_t ast = parse("(for i (range 6) (body-of (lambda () 1 2)))");
  const auto run = [&](bool pooled) {
    std::unique_ptr<scoped_env_var> force;
    if (pooled)
      force = std::make_unique<scoped_env_var>("CVC_EXEC_FORCE_MOVE", "1");
    pool_app w(pooled ? 2 : 0);
    group_world a(w, "rb01a"), b(w, "rb01b");
    std::mutex m;
    std::vector<list_ptr> bodies[2];
    for (int side = 0; side < 2; ++side)
      builtins::register_fn((side ? b : a).env, "body-of",
                            [&, side](std::span<const value_t> args) -> value_t {
                              if (const auto *c = std::get_if<closure_ptr>(&args[0].v)) {
                                std::lock_guard<std::mutex> lock(m);
                                bodies[side].push_back((*c)->body_expr);
                              }
                              return nil_value;
                            });
    execute_options oa, ob;
    oa.env = environment::extend(a.env);
    ob.env = environment::extend(b.env);
    const int pa = a.sched().execute(ast, oa);
    const int pb = b.sched().execute(ast, ob);
    if (pooled) {
      pump_pooled(w, {&a, &b}, [](std::size_t) { return steps_only(5); });
    } else {
      for (int i = 0; i < 10000 && !(a.done() && b.done()); ++i) {
        a.g().pump(steps_only(5));
        b.g().pump(steps_only(5));
      }
    }
    EXPECT_TRUE(finished(a.sched(), pa) && finished(b.sched(), pb));
    std::size_t shared = 0;
    for (const list_ptr &x : bodies[0])
      for (const list_ptr &y : bodies[1])
        shared += (x && x == y) ? 1 : 0;
    EXPECT_FALSE(bodies[0].empty());
    EXPECT_FALSE(bodies[1].empty());
    EXPECT_EQ(shared, 0u) << (pooled ? "on the pool" : "alternating on one thread")
                          << ": a body list is shared by two groups";
  };
  run(false);
  run(true);
}

// AS-1 (td B12, S1.5): one value_t with a quoted literal, executed into two groups through
// exec_group::execute. The first mutates its literal; the second still sees the original, and so
// does the host's AST.
TEST(Groups, AS01_ASharedAstIsCopied) {
  pool_app w(0);
  group_world a(w, "as01a"), b(w, "as01b");
  builtins::register_fn(a.env, "mutate?",
                        [](std::span<const value_t>) -> value_t { return value_t(true); });
  builtins::register_fn(b.env, "mutate?",
                        [](std::span<const value_t>) -> value_t { return value_t(false); });
  const value_t ast = parse("(begin (set l (quote (1 2 3))) (if (mutate?) (set-nth l 0 9)) l)");
  execute_options oa, ob;
  oa.env = environment::extend(a.env);
  ob.env = environment::extend(b.env);
  const int pa = a.g().execute(ast, oa);
  pump_alone(a, [](std::size_t) { return steps_only(1000); });
  EXPECT_EQ(a.outcome(pa), "(9 2 3)");
  const int pb = b.g().execute(ast, ob);
  pump_alone(b, [](std::size_t) { return steps_only(1000); });
  EXPECT_EQ(b.outcome(pb), "(1 2 3)") << "the second group saw the first's mutation";
  EXPECT_NE(to_string(ast).find("(1 2 3)"), std::string::npos) << to_string(ast);
}

// BF-1 (S1.3): groups' global envs extend one frozen builtins scope. An env-less top-level set of
// a builtin's name in group A binds in A's program's own scope: A sees it, group B does not, and
// the frozen scope keeps the builtin (and refuses a set).
TEST(Groups, BF01_TheFrozenBaseIsShared) {
  pool_app w(0);
  const auto make = [&](const char *name) {
    exec_group::options o;
    o.name = name;
    o.owner = name;
    o.app = &w.app;
    o.watch_root = &w.tree;
    return std::make_unique<exec_group>(std::move(o));
  };
  auto a = make("bf01a");
  auto b = make("bf01b");
  const auto run = [](exec_group &g, const std::string &script) {
    const int pid = g.execute(script);
    for (int i = 0; i < 1000 && !finished(g.sched(), pid); ++i)
      g.pump(steps_only(1000));
    return outcome_of(g.sched(), pid);
  };
  EXPECT_EQ(
      run(*a, "(begin (set str-concat (lambda (x y) \"hijacked\")) (str-concat \"a\" \"b\"))"),
      "hijacked");
  EXPECT_EQ(run(*b, "(str-concat \"a\" \"b\")"), "ab") << "B saw A's set";
  const environment_ptr frozen = frozen_builtins();
  ASSERT_NE(frozen, nullptr);
  const value_t *kept = frozen->lookup("str-concat");
  ASSERT_NE(kept, nullptr);
  EXPECT_TRUE(std::holds_alternative<native_fn>(kept->v)) << "the frozen scope was written";
  EXPECT_THROW(frozen->set("str-concat", nil_value), std::logic_error);
}

// OC-1 (td 4.9, S1.6): on_complete callbacks of a group a worker pumped run on the caller after
// the join, outside any pump, in completion order; one that throws is rethrown from pump_all after
// the others ran. A group the caller pumps runs them inline, inside its pump.
TEST(Groups, OC01_OnCompleteRunsOnTheCallerAfterTheJoin) {
  const auto caller = std::this_thread::get_id();
  struct callback {
    int tag;
    std::thread::id thread;
    exec_group *in;
  };
  std::mutex m;
  std::vector<callback> calls;
  const auto note = [&](int tag) {
    std::lock_guard<std::mutex> lock(m);
    calls.push_back({tag, std::this_thread::get_id(), exec_group::current()});
  };
  {
    SCOPED_TRACE("pumped by a worker");
    pool_app w(4);
    if (w.pool.threads() == 0) {
      ADD_FAILURE() << "the pool has no worker";
    } else {
      caller_hold hold(w, "oc01hold");
      std::thread::id pumped_on;
      group_world target(w, "oc01", group_affinity::any, [&](exec_group::options &o) {
        o.pump_hook_for_test = [&](exec_group &) {
          pumped_on = std::this_thread::get_id();
          hold.go.store(true);
        };
      });
      // Submitted long, short, middle; they end short, middle, long.
      target.submit("(begin (for i (range 300) nil) 3)", [&](value_t) { note(3); });
      target.submit("1", [&](value_t) { note(1); });
      target.submit("(begin (for i (range 100) nil) 2)", [&](value_t) {
        note(2);
        throw std::runtime_error("oc01: a callback threw");
      });
      w.pool.set_last_pump_for_test(hold.g(), 1e-3);
      w.pool.set_last_pump_for_test(target.g(), 1e-3);
      std::string thrown;
      try {
        w.pool.pump_all(std::vector<exec_group *>{&hold.g(), &target.g()}, steps_only(100000),
                        clock_type::now() + std::chrono::hours(1));
      } catch (const std::runtime_error &e) {
        thrown = e.what();
      }
      EXPECT_NE(pumped_on, caller) << "the group was pumped on the caller";
      EXPECT_EQ(thrown, "oc01: a callback threw");
      ASSERT_EQ(calls.size(), 3u);
      for (std::size_t i = 0; i < calls.size(); ++i) {
        EXPECT_EQ(calls[i].tag, static_cast<int>(i + 1)) << "not in completion order";
        EXPECT_EQ(calls[i].thread, caller) << "callback " << calls[i].tag;
        EXPECT_EQ(calls[i].in, nullptr) << "callback " << calls[i].tag << " ran inside a pump";
      }
    }
  }
  calls.clear();
  {
    SCOPED_TRACE("pumped by the caller");
    pool_app w(0);
    group_world target(w, "oc01c");
    target.submit("(begin (for i (range 50) nil) 2)", [&](value_t) { note(2); });
    target.submit("1", [&](value_t) { note(1); });
    w.pool.pump_all(std::vector<exec_group *>{&target.g()}, steps_only(100000),
                    clock_type::now() + std::chrono::hours(1));
    ASSERT_EQ(calls.size(), 2u);
    for (std::size_t i = 0; i < calls.size(); ++i) {
      EXPECT_EQ(calls[i].tag, static_cast<int>(i + 1));
      EXPECT_EQ(calls[i].thread, caller);
      EXPECT_EQ(calls[i].in, &target.g()) << "not inline in the group's pump";
    }
  }
}

namespace {

// One deterministic run (DT-1): `groups` groups at threads 4 in deterministic mode, each with
// GR-1's programs, a math.random program that posts to the host between draws, and on_complete
// callbacks. The pumps get small seconds budgets and a near deadline, which deterministic mode
// ignores. Recorded: each group's steps at the start of every pump, its processes, and the order
// of the host posts and callbacks.
struct deterministic_run {
  std::vector<std::vector<std::uint64_t>> steps_at_pump;
  std::vector<std::vector<std::string>> procs;
  std::vector<std::string> host_order;
};

deterministic_run run_deterministic(int groups) {
  deterministic_run out;
  out.steps_at_pump.resize(groups);
  pool_app w(4, true);
  std::mutex m;
  std::vector<std::unique_ptr<group_world>> worlds;
  std::vector<group_world *> raw;
  for (int k = 0; k < groups; ++k) {
    const std::string name = "dt01g" + std::to_string(k);
    worlds.push_back(
        std::make_unique<group_world>(w, name, group_affinity::any, [&, k](exec_group::options &o) {
          o.pump_hook_for_test = [&, k](exec_group &g) {
            out.steps_at_pump[k].push_back(g.sched().get_stats().total_steps);
          };
        }));
    group_world &gw = *worlds.back();
    gw.g().install(
        gw.env, "post-host",
        [&, name](std::span<const value_t> a) -> value_t {
          std::lock_guard<std::mutex> lock(m);
          out.host_order.push_back(name + " post " + (a.empty() ? "" : to_string(a[0])));
          return nil_value;
        },
        provider_affinity::host_proxied);
    raw.push_back(&gw);
    std::vector<std::string> programs = oracle_programs(k);
    programs.push_back("(begin (set out (list)) (for i (range 6) (begin (append out "
                       "(math.random)) (post-host i))) out)");
    for (std::size_t j = 0; j < programs.size(); ++j)
      gw.submit(programs[j], [&, name, j](value_t) {
        std::lock_guard<std::mutex> lock(m);
        out.host_order.push_back(name + " done " + std::to_string(j));
      });
  }
  for (int frame = 0; frame < 100000; ++frame) {
    std::vector<exec_group *> live;
    for (group_world *gw : raw)
      if (!gw->done())
        live.push_back(&gw->g());
    if (live.empty())
      break;
    w.pool.pump_all(live, pump_budget{150, 0.002},
                    clock_type::now() + std::chrono::milliseconds(10));
  }
  std::printf("DT-1: a run at threads 4 (effective %u), deterministic\n", w.effective());
  for (group_world *gw : raw) {
    EXPECT_TRUE(gw->done());
    out.procs.push_back(record(*gw).procs);
  }
  return out;
}

} // namespace

// DT-1 (td 8.9): deterministic mode, one program set run twice at threads 4: identical per-group
// step counts at every pump, identical results (math.random's streams included), and identical
// order of the host posts and callbacks.
TEST(Groups, DT01_DeterministicRunsRepeat) {
  const int groups = under_sanitizer() ? 4 : 8;
  const deterministic_run first = run_deterministic(groups);
  const deterministic_run second = run_deterministic(groups);
  for (int k = 0; k < groups; ++k) {
    EXPECT_EQ(first.steps_at_pump[k], second.steps_at_pump[k]) << "group " << k;
    EXPECT_EQ(first.procs[k], second.procs[k]) << "group " << k;
  }
  EXPECT_EQ(first.host_order, second.host_order);
  EXPECT_FALSE(first.host_order.empty());
}

// DT-2 (td 8.9, 4.14): GR-1's programs through the pool at threads 0 equal the same programs on
// bare async_schedulers pumped with the same budgets: results, step counts, statuses.
TEST(Groups, DT02_ThreadsZeroEqualsBareSchedulers) {
  const int groups = under_sanitizer() ? 2 : 8;
  const pump_budget budget = steps_only(97);
  std::vector<std::vector<std::string>> bare;
  {
    pool_app w(0);
    for (int k = 0; k < groups; ++k) {
      bare_world bw(w, "g" + std::to_string(k));
      for (const std::string &p : oracle_programs(k))
        bw.submit(p);
      for (int i = 0; i < 200000 && !bw.done(); ++i)
        bw.pump(budget);
      EXPECT_TRUE(bw.done()) << "bare " << k;
      bare.push_back(record(bw.sched, nullptr).procs);
    }
  }
  pool_app w(0);
  std::vector<std::unique_ptr<group_world>> worlds;
  std::vector<group_world *> raw;
  for (int k = 0; k < groups; ++k) {
    worlds.push_back(std::make_unique<group_world>(w, "g" + std::to_string(k)));
    raw.push_back(worlds.back().get());
    for (const std::string &p : oracle_programs(k))
      worlds.back()->submit(p);
  }
  pump_pooled(w, raw, [&](std::size_t) { return budget; });
  for (int k = 0; k < groups; ++k)
    EXPECT_EQ(record(*worlds[k]).procs, bare[k]) << "group " << k;
}

// GL-1 (S1-Q1): a holder in group A and a waiter in group B on one app-global key, at threads 0
// (a single lock manager) and 4 (shared): B waits while A holds, across pumps of both; once A has
// released, B is granted at its next pump.
TEST(Groups, GL01_TwoGroupsExcludeOneKey) {
  for (const unsigned threads : {0u, 4u}) {
    SCOPED_TRACE("threads " + std::to_string(threads));
    pool_app w(threads);
    group_world a(w, "gl01a"), b(w, "gl01b");
    const int holder =
        a.submit(R"((with-lock "/gl01" (begin (set i 0) (while (< i 3000) (set i (+ i 1))) "a")))");
    const int waiter = b.submit(R"((with-lock "/gl01" "b"))");
    const auto no_deadline = clock_type::now() + std::chrono::hours(1);
    const auto frame = [&](std::vector<exec_group *> gs) {
      w.pool.pump_all(gs, steps_only(300), no_deadline);
    };
    frame({&a.g()});
    ASSERT_FALSE(finished(a.sched(), holder)) << "the holder ended in one pump";
    frame({&b.g()});
    EXPECT_FALSE(finished(b.sched(), waiter)) << "B ran while A held the key";
    EXPECT_EQ(b.sched().lock_waiting_count(), 1u);
    for (int i = 0; i < 3 && !finished(a.sched(), holder); ++i) {
      frame({&a.g(), &b.g()});
      if (!finished(a.sched(), holder))
        EXPECT_FALSE(finished(b.sched(), waiter)) << "B ran while A held the key";
    }
    for (int i = 0; i < 1000 && !finished(a.sched(), holder); ++i)
      frame({&a.g()});
    ASSERT_EQ(a.outcome(holder), "a");
    EXPECT_FALSE(finished(b.sched(), waiter)) << "B ran without a pump";
    frame({&b.g()});
    EXPECT_EQ(b.outcome(waiter), "b") << "B was not granted at its next pump";
  }
}

// GL-2 (S1-Q1, td 4.2): group B is destroyed with a waiter queued behind A's hold, and group C's
// waiter behind B's. B's teardown_hook_for_test, between closing B's lock port and its kills,
// kills A's holder, so the key is granted to B's waiter then. B's kill of that waiter releases the
// grant, and C's waiter is granted.
TEST(Groups, GL02_TeardownClosesThePortBeforeItsKills) {
  for (const unsigned threads : {0u, 4u}) {
    SCOPED_TRACE("threads " + std::to_string(threads));
    pool_app w(threads);
    group_world a(w, "gl02a"), c(w, "gl02c");
    int holder = -1;
    bool hook_ran = false;
    auto b =
        std::make_unique<group_world>(w, "gl02b", group_affinity::any, [&](exec_group::options &o) {
          o.teardown_hook_for_test = [&](exec_group &) {
            hook_ran = true;
            a.sched().kill(holder);
          };
        });
    holder = a.submit(R"((with-lock "/gl02" (msg-recv "never")))");
    const int bw = b->submit(R"((with-lock "/gl02" "b"))");
    const int cw = c.submit(R"((with-lock "/gl02" "c"))");
    const auto no_deadline = clock_type::now() + std::chrono::hours(1);
    const auto frame = [&](exec_group &g) {
      w.pool.pump_all(std::vector<exec_group *>{&g}, steps_only(300), no_deadline);
    };
    frame(a.g());
    frame(b->g());
    frame(c.g());
    EXPECT_FALSE(finished(b->sched(), bw)) << "B's waiter ran while A held";
    EXPECT_FALSE(finished(c.sched(), cw)) << "C's waiter ran while A held";
    b.reset();
    EXPECT_TRUE(hook_ran) << "the teardown hook was not called";
    EXPECT_EQ(status_of(a.sched(), holder), process_status::killed);
    for (int i = 0; i < 10 && !finished(c.sched(), cw); ++i)
      frame(c.g());
    EXPECT_EQ(c.outcome(cw), "c") << "C's waiter was not granted";
  }
}

// PA-1 (S1-Q11): two threads call pump_all on one pool at once, with disjoint groups, one of them
// making a host_proxied call: both finish, the proxied body runs on its own call's caller (each
// caller services only its own queue), and, in debug builds, a group given to two calls at once
// asserts.
TEST(Groups, PA01_TwoCallersShareThePool) {
  const auto twice_given = [] {
    pool_app w(2);
    std::atomic<bool> in_pump{false};
    group_world g(w, "pa01dup", group_affinity::any, [&](exec_group::options &o) {
      o.pump_hook_for_test = [&](exec_group &) {
        in_pump.store(true);
        const auto t0 = clock_type::now();
        while (seconds_since(t0) < kCeiling)
          std::this_thread::yield();
      };
    });
    g.submit("(while #t nil)");
    const std::vector<exec_group *> one{&g.g()};
    const auto no_deadline = clock_type::now() + std::chrono::hours(1);
    std::thread first([&] { w.pool.pump_all(one, steps_only(100), no_deadline); });
    const auto t0 = clock_type::now();
    while (!in_pump.load() && seconds_since(t0) < kCeiling)
      std::this_thread::yield();
    w.pool.pump_all(one, steps_only(100), no_deadline); // the group is the first call's
    first.join();
  };
  if (kDebugBuild) {
    GTEST_FLAG_SET(death_test_style, "threadsafe");
    EXPECT_DEATH(twice_given(), "");
  } else {
    std::printf("PA-1: a group given to two calls asserts only in debug builds\n");
  }

  pool_app w(4);
  std::mutex m;
  std::vector<std::thread::id> proxied_on;
  std::vector<std::unique_ptr<group_world>> mine, theirs;
  for (int k = 0; k < 3; ++k) {
    mine.push_back(std::make_unique<group_world>(w, "pa01a" + std::to_string(k)));
    theirs.push_back(std::make_unique<group_world>(w, "pa01b" + std::to_string(k)));
  }
  mine[0]->g().install(
      mine[0]->env, "proxied",
      [&](std::span<const value_t>) -> value_t {
        std::lock_guard<std::mutex> lock(m);
        proxied_on.push_back(std::this_thread::get_id());
        return value_t(int64_t(7));
      },
      provider_affinity::host_proxied);
  const int uses = mine[0]->submit("(begin (for i (range 500) nil) (+ (proxied) 1))");
  for (int k = 0; k < 3; ++k) {
    mine[k]->submit("(begin (set i 0) (while (< i 20000) (set i (+ i 1))) i)");
    theirs[k]->submit("(begin (set i 0) (while (< i 20000) (set i (+ i 1))) i)");
  }
  const auto drive = [&](std::vector<std::unique_ptr<group_world>> &worlds, bool &all_done) {
    const auto t0 = clock_type::now();
    while (seconds_since(t0) < kCeiling) {
      std::vector<exec_group *> live;
      for (auto &gw : worlds)
        if (!gw->done())
          live.push_back(&gw->g());
      if (live.empty()) {
        all_done = true;
        return;
      }
      w.pool.pump_all(live, steps_only(200), clock_type::now() + std::chrono::hours(1));
    }
  };
  std::atomic<int> ready{0};
  bool mine_done = false, theirs_done = false;
  std::thread::id my_caller;
  std::thread other([&] {
    my_caller = std::this_thread::get_id();
    ready.fetch_add(1);
    while (ready.load() < 2)
      std::this_thread::yield();
    drive(mine, mine_done);
  });
  ready.fetch_add(1);
  while (ready.load() < 2)
    std::this_thread::yield();
  const auto t0 = clock_type::now();
  drive(theirs, theirs_done);
  other.join();
  std::printf("PA-1: two callers at threads 4 (effective %u), %.3f s\n", w.effective(),
              seconds_since(t0));
  EXPECT_TRUE(mine_done);
  EXPECT_TRUE(theirs_done);
  EXPECT_EQ(mine[0]->outcome(uses), "8");
  ASSERT_EQ(proxied_on.size(), 1u);
  EXPECT_EQ(proxied_on[0], my_caller) << "another caller ran the proxied body";
}
