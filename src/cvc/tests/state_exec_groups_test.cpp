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
#include <cvc/state/state_exec/exceptions.h>
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

// No registry is kept per thread. Each world's scheduler tracks into a registry that goes after the
// world's environment (its group's, or bare_world's own), so that registry's last collection frees
// the cycles its programs left, and LeakSanitizer has only real leaks to report.
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
      return nil_value();
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

// RX-1 (ASan): a bare scheduler on a boost::thread leaves a cycle, and the thread ends. The
// scheduler's own registry (its evaluator's: nothing is kept per thread) is collected as the
// scheduler goes, so the cycle's weak observer has expired after the join. The program is far
// shorter than the 64 steps between due checks, so no collection runs before then.
TEST(Groups, RX01_ABareSchedulersCyclesGoWithIt) {
  {
    scope_observer observer;
    std::string outcome;
    boost::thread t([&] {
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
    EXPECT_EQ(observer.alive(), 0u) << "the scheduler's registry was not collected as it went";
    observer.release();
  }
}

// RX-2 (S1.1): on a bare scheduler given a registry, a kill(pid) from the host thread between
// pumps notes the exit in that registry, whatever else the host has made: not in another registry.
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
    ASSERT_TRUE(sched.kill(first));
    EXPECT_TRUE(exit_pending_for_test(*mine)) << "a host kill";
    EXPECT_FALSE(exit_pending_for_test(*other));
    collect_cycles(*mine);
    ASSERT_FALSE(exit_pending_for_test(*mine));
    ASSERT_TRUE(sched.kill(second));
    EXPECT_TRUE(exit_pending_for_test(*mine)) << "a second host kill";
  } // the scheduler goes before its registries, as an owner orders them
}

// RX-3 (S1.1): one scheduler given a registry, stepped through every driver: cvc::pump_exec_frame,
// sync_run, sync_step, and a restore. Each program leaves a live cycle (keepN, bound in the env
// the test holds) and garbage ones; the restored process holds a closure in its own scope, which
// the restore decodes. All of it is tracked in the scheduler's registry, whichever driver ran it.
TEST(Groups, RX03_EveryDriverTracksIntoTheSchedulersRegistry) {
  cvc::app app;
  cvc::state &tree = cvc::state::instance(app);
  cycle_registry_ptr reg = make_cycle_registry();
  {
    async_scheduler sched;
    sched.set_cycle_registry(reg.get());
    EXPECT_EQ(sched.evaluation_context().registry, reg.get());
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
    sched.sync_run();
    EXPECT_EQ(outcome_of(sched, moved), "4");
    check("restore", 3);
  }
}

// BF-1a (S1.3): the frozen builtins scope refuses a set.
TEST(Groups, BF01a_TheFrozenScopeRefusesASet) {
  cvc::app app;
  const environment_ptr frozen = frozen_builtins(app);
  ASSERT_NE(frozen, nullptr);
  EXPECT_TRUE(frozen->frozen);
  EXPECT_THROW(frozen->set("str-concat", nil_value()), std::logic_error);
  EXPECT_THROW(frozen->set("bf01a-new-name", nil_value()), std::logic_error);
  EXPECT_EQ(frozen->lookup("bf01a-new-name"), nullptr);
  const value_t *kept = frozen->lookup("str-concat");
  ASSERT_NE(kept, nullptr);
  EXPECT_TRUE(std::holds_alternative<native_fn>(kept->v)) << "the builtin was overwritten";
  EXPECT_EQ(frozen_builtins(app), frozen) << "one frozen scope per app";
  cvc::app other;
  EXPECT_NE(frozen_builtins(other), frozen) << "another app has a frozen scope of its own";
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
                            e->set("kept", a.empty() ? nil_value() : a[0]);
                          return nil_value();
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
  // The world lets go of the environment before the group goes, as a host does (pycvc's pooled
  // Exec): the group's registry, which goes last, then frees the cycles its programs left there
  // (keepN bound in it), with nothing else holding them.
  ~group_world() { env.reset(); }

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
// defaults, the app's root watched, a registry of its own), and pumped as exec_group::pump pumps
// (DT-2). The registry goes last, as a group's does: its last collection, once the world has let
// go of the environment, frees the cycles its programs left there.
struct bare_world {
  cycle_registry_ptr registry = make_cycle_registry();
  intrinsics_context ictx;
  environment_ptr env = builtins::make_default_environment();
  async_scheduler sched;
  std::unique_ptr<async_host> host;

  bare_world(pool_app &w, const std::string &name) {
    sched.set_cycle_registry(registry.get());
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
    std::weak_ptr<environment> env;
    {
      group_world gw(w, "g" + std::to_string(k));
      env = gw.env;
      apply_plan(plan, k, gw);
      pump_alone(gw, budget);
      EXPECT_TRUE(gw.done()) << "group " << k << " alone never ended";
      solo.push_back(record(gw));
    }
    // The cycles its programs left in the environment went with the group's registry.
    EXPECT_TRUE(env.expired()) << "group " << k << "'s environment outlived its world";
  }
  return solo;
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

// ...then all of them on a pool of `threads` workers: every group's record equals its solo run.
// The first frame holds the caller (caller_hold) until a worker has started one of the groups'
// pumps, so a pool with workers is shown to use them (stats().worker_pumps), whatever the pumps'
// timing: a fresh group's last pump counts as 0 s, so without it the first frames might wake no
// helper, and a regression that never woke one would leave the oracle green (R1-mid S-m2). The
// hold gets its own frame and no budget of the groups': their pumps are as in the solo runs.
void check_against_solo(const char *label, int groups, unsigned threads,
                        const std::vector<group_record> &solo, const budget_fn &budget,
                        const group_plan &plan, bool &no_registry_reported) {
  pool_app w(threads);
  const auto caller = std::this_thread::get_id();
  caller_hold hold(w, "hold");
  if (w.pool.threads() == 0)
    hold.go.store(true); // no worker to wait for
  std::vector<std::unique_ptr<group_world>> worlds;
  std::vector<group_world *> raw;
  for (int k = 0; k < groups; ++k) {
    worlds.push_back(std::make_unique<group_world>(
        w, "g" + std::to_string(k), group_affinity::any, [&](exec_group::options &o) {
          if (plan.options)
            plan.options(k, o);
          o.pump_hook_for_test = [inner = o.pump_hook_for_test, &hold, caller](exec_group &g) {
            if (std::this_thread::get_id() != caller)
              hold.go.store(true); // a worker has a group: the caller may go on
            if (inner)
              inner(g);
          };
        }));
    raw.push_back(worlds.back().get());
    apply_plan(plan, k, *worlds.back());
  }
  const auto t0 = clock_type::now();
  std::vector<exec_group *> first{&hold.g()};
  w.pool.set_last_pump_for_test(hold.g(), 1e-3);
  for (group_world *gw : raw) {
    first.push_back(&gw->g());
    w.pool.set_last_pump_for_test(gw->g(), 1e-3); // above inline_below: helpers are woken
  }
  w.pool.pump_all(first, budget(0), clock_type::now() + std::chrono::hours(1));
  const std::size_t frames = 1 + pump_pooled(w, raw, [&](std::size_t i) { return budget(i + 1); });
  const cvc::pool_stats stats = w.pool.stats();
  std::printf("%s: %d groups at threads %u (effective %u): %zu frames, %llu of %llu pumps on a "
              "worker, %.3f s\n",
              label, groups, threads, w.effective(), frames,
              static_cast<unsigned long long>(stats.worker_pumps),
              static_cast<unsigned long long>(stats.pumps), seconds_since(t0));
  std::fflush(stdout);
  if (w.pool.threads() > 0)
    EXPECT_GT(stats.worker_pumps, 0u) << label << ": no worker pumped a group";
  for (int k = 0; k < groups; ++k) {
    const std::string where = std::string(label) + ", " + std::to_string(groups) +
                              " groups at threads " + std::to_string(threads) + ", group " +
                              std::to_string(k);
    EXPECT_TRUE(worlds[k]->done()) << where << " never ended";
    expect_same(record(*worlds[k]), solo[k], where, no_registry_reported);
  }
}

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

// Where a native ran: its thread and the group whose pump ran it (exec_group::current(ctx) of the
// call's context, null outside a pump).
struct native_call {
  std::thread::id thread;
  exec_group *in = nullptr;
};

struct native_calls {
  std::mutex m;
  std::vector<native_call> calls;
  void note(const exec_context *ctx) {
    std::lock_guard<std::mutex> lock(m);
    calls.push_back({std::this_thread::get_id(), exec_group::current(ctx)});
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
                          [](std::span<const value_t>) -> value_t { return nil_value(); });
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
    builtins::register_fn(gw.env, "note-thread", [n](call_args a) -> value_t {
      n->note(a.context());
      return nil_value();
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
    std::vector<exec_group *> failed;
    EXPECT_NO_THROW(
        failed = w.pool.pump_all(first, budget(0), clock_type::now() + std::chrono::hours(1)));
    ASSERT_GE(thrown.load(), 1);
    if (on_worker)
      EXPECT_NE(thrown_on, caller) << "the throwing pump ran on the caller";
    else
      EXPECT_EQ(thrown_on, caller);
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
                              return nil_value();
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

// BF-1 (S1.3): an app's groups' global envs extend the app's one frozen builtins scope. An
// env-less top-level set of a builtin's name in group A binds in A's program's own scope: A sees
// it, group B does not, and the frozen scope keeps the builtin (and refuses a set).
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
  const environment_ptr frozen = frozen_builtins(w.app);
  ASSERT_NE(frozen, nullptr);
  const value_t *kept = frozen->lookup("str-concat");
  ASSERT_NE(kept, nullptr);
  EXPECT_TRUE(std::holds_alternative<native_fn>(kept->v)) << "the frozen scope was written";
  EXPECT_THROW(frozen->set("str-concat", nil_value()), std::logic_error);
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
  exec_group *watched = nullptr; // the target group: whether this thread pumps it
  const auto note = [&](int tag) {
    std::lock_guard<std::mutex> lock(m);
    calls.push_back(
        {tag, std::this_thread::get_id(), watched && watched->pumping_here() ? watched : nullptr});
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
        // The completion order below is round robin's, one step a pick (Stage S's runs would let
        // the middle program finish within its first run, under CVC_EXEC_SLICE_STEPS).
        o.slice_steps = 1;
        o.pump_hook_for_test = [&](exec_group &) {
          pumped_on = std::this_thread::get_id();
          hold.go.store(true);
        };
      });
      // Submitted long, short, middle; they end short, middle, long.
      watched = &target.g();
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
    // One step a pick, as above: a 256-step run would finish the 50-iteration loop first.
    group_world target(w, "oc01c", group_affinity::any,
                       [](exec_group::options &o) { o.slice_steps = 1; });
    watched = &target.g();
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
          return nil_value();
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
      std::weak_ptr<environment> env;
      {
        bare_world bw(w, "g" + std::to_string(k));
        env = bw.env;
        for (const std::string &p : oracle_programs(k))
          bw.submit(p);
        for (int i = 0; i < 200000 && !bw.done(); ++i)
          bw.pump(budget);
        EXPECT_TRUE(bw.done()) << "bare " << k;
        bare.push_back(record(bw.sched, nullptr).procs);
      }
      // The cycles its programs left in the environment went with the world's registry.
      EXPECT_TRUE(env.expired()) << "bare " << k << "'s environment outlived its world";
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
  // Long enough that the two calls overlap. A debug build interprets several times slower: at the
  // release length, on a busy four-CPU runner, it outran the ceiling.
  const std::string loop = std::string("(begin (set i 0) (while (< i ") +
                           (kDebugBuild ? "5000" : "20000") + ") (set i (+ i 1))) i)";
  for (int k = 0; k < 3; ++k) {
    mine[k]->submit(loop);
    theirs[k]->submit(loop);
  }
  // The two calls' own ceiling (rule 7), above kCeiling. On a quiet runner a debug build takes
  // 5-7 s for the whole case. A loaded runner slows it several times over: the pool's workers and
  // the two callers hand work to each other, and each handoff waits for a CPU. With 66 busy loops
  // on 22 CPUs a release run took 7x as long, and in debug CI jobs that ran the Groups cases
  // 2.5-3x slower than usual, 10 s ran out with all three tries unfinished. A hang still fails.
  constexpr double kTwoCallsCeiling = 60.0;
  const auto drive = [&](std::vector<std::unique_ptr<group_world>> &worlds, bool &all_done) {
    const auto t0 = clock_type::now();
    while (seconds_since(t0) < kTwoCallsCeiling) {
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

// --- Foreign futures and host-thread natives (S1.8) ---

namespace {

std::string exception_class(const async_scheduler &sched, int pid) {
  const auto exc = sched.get_exit_exception(pid);
  return exc ? class_name(class_of_instance(*exc)) : "<no exception>";
}

std::string exception_text(const async_scheduler &sched, int pid) {
  const auto exc = sched.get_exit_exception(pid);
  return exc ? exception_message(*exc) : sched.get_exit_error(pid).value_or("<none>");
}

// Two bare async schedulers of one app, each with its own async host on the app's background pool
// and its own environment (FF-1, FF-2). A has (hold) and (hold-async), a kernel that runs until
// `release` is set, then returns 21; (twice) and (twice-async) on an inline executor; and
// (stash f), which keeps f. B has (fetch), which returns what A stashed.
struct two_schedulers {
  struct side {
    async_scheduler sched;
    std::unique_ptr<async_host> host;
    intrinsics_context ictx;
    environment_ptr env = builtins::make_default_environment();

    void setup(cvc::app &app, cvc::state &tree) {
      sched.set_watch_root(&tree);
      host = std::make_unique<async_host>(sched, &app);
      ictx.sched = &sched;
      ictx.root = &tree;
      ictx.proc = make_process();
      ictx.host = host.get();
      register_intrinsics(env, &ictx);
    }
    int submit(const std::string &script) {
      execute_options opts;
      opts.env = environment::extend(env);
      opts.max_time = kProcessCeiling;
      return sched.execute(script, opts);
    }
    void pump() {
      sched.drain_ingress();
      sched.sync_run_steps(100000, 0.02);
    }
  };

  cvc::app app;
  cvc::state &tree = cvc::state::instance(app);
  side a, b;
  std::atomic<bool> release{false};
  std::mutex m;
  value_t stashed;

  two_schedulers() {
    tree("state_exec.defaults.limits.async_steps_per_cpu_ms").value(std::string("0"));
    a.setup(app, tree);
    b.setup(app, tree);
    register_async_fn(a.env, *a.host, "hold", [this](std::span<const value_t>) -> async_kernel {
      return [this](async_call &call) -> value_t {
        while (!release.load() && !call.cancelled())
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return value_t(int64_t(21));
      };
    });
    async_options inline_call;
    inline_call.where = executor::inline_call();
    register_async_fn(
        a.env, *a.host, "twice",
        [](std::span<const value_t> args) -> async_kernel {
          const int64_t x = std::get<int64_t>(args[0].v);
          return [x](async_call &) -> value_t { return value_t(2 * x); };
        },
        inline_call);
    builtins::register_fn(a.env, "stash", [this](std::span<const value_t> args) -> value_t {
      std::lock_guard<std::mutex> lock(m);
      stashed = args[0];
      return nil_value();
    });
    builtins::register_fn(b.env, "fetch", [this](std::span<const value_t>) -> value_t {
      std::lock_guard<std::mutex> lock(m);
      return stashed;
    });
  }
  ~two_schedulers() {
    release.store(true);
    b.host->shutdown();
    a.host->shutdown();
  }
  bool has_stash() {
    std::lock_guard<std::mutex> lock(m);
    return !std::holds_alternative<std::monostate>(stashed.v);
  }
};

} // namespace

// FF-1 (S1-Q2, td 4.9): a threaded future made on scheduler A, awaited from scheduler B, raises
// RuntimeError "await: the future belongs to another scheduler" at once, through await, await-all
// and msg-recv on the future. A's call list is untouched (A's finish_async_call never ran on B's
// thread), and A's own await still gets the value once the kernel ends.
TEST(Groups, FF01_AnotherSchedulersFutureRaises) {
  for (const char *const wait : {"(begin (await (fetch)))", "(begin (await-all (list (fetch))))",
                                 "(begin (msg-recv (fetch)))"}) {
    SCOPED_TRACE(wait);
    two_schedulers w;
    const int owner = w.a.submit("(begin (set f (hold-async)) (stash f) (await f))");
    const auto t0 = clock_type::now();
    while (!(w.has_stash() && status_of(w.a.sched, owner) == process_status::waiting) &&
           seconds_since(t0) < kCeiling)
      w.a.pump();
    ASSERT_TRUE(w.has_stash());
    ASSERT_EQ(w.a.sched.async_calls_in_flight(), 1u);
    const int awaiter = w.b.submit(wait);
    for (int i = 0; i < 3; ++i)
      w.b.pump();
    EXPECT_EQ(status_of(w.b.sched, awaiter), process_status::failed)
        << "B's await did not raise: " << outcome_of(w.b.sched, awaiter);
    EXPECT_EQ(exception_class(w.b.sched, awaiter), "RuntimeError");
    EXPECT_NE(exception_text(w.b.sched, awaiter).find(kForeignFutureError), std::string::npos)
        << exception_text(w.b.sched, awaiter);
    EXPECT_EQ(w.a.sched.async_calls_in_flight(), 1u) << "A's call list changed";
    EXPECT_EQ(status_of(w.a.sched, owner), process_status::waiting);
    w.release.store(true);
    const auto t1 = clock_type::now();
    while (!finished(w.a.sched, owner) && seconds_since(t1) < kCeiling)
      w.a.pump();
    EXPECT_EQ(outcome_of(w.a.sched, owner), "21");
    if (!finished(w.b.sched, awaiter))
      w.b.sched.kill(awaiter);
  }
}

// FF-2 (S1-Q2): an inline future (a call on an inline executor, settled before it returned, with
// no owner) is a plain value: awaited from another scheduler, through await, await-all and
// msg-recv, it gives its value.
TEST(Groups, FF02_AnInlineFutureIsAValue) {
  for (const auto &[wait, want] : std::vector<std::pair<std::string, std::string>>{
           {"(begin (await (fetch)))", "42"},
           {"(begin (await-all (list (fetch))))", "(42)"},
           {"(begin (msg-recv (fetch)))", "42"}}) {
    SCOPED_TRACE(wait);
    two_schedulers w;
    const int maker = w.a.submit("(begin (stash (twice-async 21)) 1)");
    for (int i = 0; i < 10 && !finished(w.a.sched, maker); ++i)
      w.a.pump();
    ASSERT_EQ(outcome_of(w.a.sched, maker), "1");
    const int awaiter = w.b.submit(wait);
    for (int i = 0; i < 10 && !finished(w.b.sched, awaiter); ++i)
      w.b.pump();
    EXPECT_EQ(outcome_of(w.b.sched, awaiter), want);
  }
}

// GR-10 (td 4.9, S1.8): natives installed by kind (exec_group::install). In an `any` group a
// worker pumps, a host_only native is not bound (a call is a NameError), and a host_proxied one
// runs on the caller's thread, outside any pump, its process resuming at the group's next pump.
// At threads 0 the caller pumps the group and the wrapper runs the body inline, in the pump.
TEST(Groups, GR10_HostOnlyAndHostProxiedNatives) {
  const auto caller = std::this_thread::get_id();
  native_calls proxied;
  const auto install = [&](group_world &gw) {
    gw.g().install(
        gw.env, "host-only-fn",
        [](std::span<const value_t>) -> value_t { return value_t(int64_t(1)); },
        provider_affinity::host_only);
    gw.g().install(
        gw.env, "proxied",
        [&](call_args a) -> value_t {
          proxied.note(a.context());
          return value_t(int64_t(7));
        },
        provider_affinity::host_proxied);
  };
  const auto no_deadline = clock_type::now() + std::chrono::hours(1);
  {
    SCOPED_TRACE("a worker pumps the group");
    pool_app w(4);
    if (w.pool.threads() == 0) {
      ADD_FAILURE() << "the pool has no worker";
    } else {
      caller_hold hold(w, "gr10hold");
      std::thread::id pumped_on;
      group_world target(w, "gr10", group_affinity::any, [&](exec_group::options &o) {
        o.pump_hook_for_test = [&](exec_group &) {
          if (!hold.go.load())
            pumped_on = std::this_thread::get_id();
          hold.go.store(true);
        };
      });
      install(target);
      const int unbound = target.submit("(host-only-fn)");
      const int uses = target.submit("(+ (proxied) 1)");
      w.pool.set_last_pump_for_test(hold.g(), 1e-3);
      w.pool.set_last_pump_for_test(target.g(), 1e-3);
      w.pool.pump_all(std::vector<exec_group *>{&hold.g(), &target.g()}, steps_only(10000),
                      no_deadline);
      EXPECT_NE(pumped_on, caller) << "the group was pumped on the caller";
      EXPECT_EQ(status_of(target.sched(), unbound), process_status::failed);
      EXPECT_EQ(exception_class(target.sched(), unbound), "NameError")
          << exception_text(target.sched(), unbound);
      const std::vector<native_call> calls = proxied.get();
      ASSERT_EQ(calls.size(), 1u);
      EXPECT_EQ(calls[0].thread, caller) << "the proxied body ran off the caller";
      EXPECT_EQ(calls[0].in, nullptr) << "the proxied body ran inside a pump";
      EXPECT_FALSE(finished(target.sched(), uses)) << "it resumed in the pump that called it";
      w.pool.pump_all(std::vector<exec_group *>{&target.g()}, steps_only(10000), no_deadline);
      EXPECT_EQ(target.outcome(uses), "8") << "it did not resume at the group's next pump";
    }
  }
  proxied.calls.clear();
  {
    SCOPED_TRACE("threads 0");
    pool_app w(0);
    group_world target(w, "gr10c");
    install(target);
    const int uses = target.submit("(+ (proxied) 1)");
    w.pool.pump_all(std::vector<exec_group *>{&target.g()}, steps_only(10000), no_deadline);
    EXPECT_EQ(target.outcome(uses), "8") << "not run inline in its pump";
    const std::vector<native_call> calls = proxied.get();
    ASSERT_EQ(calls.size(), 1u);
    EXPECT_EQ(calls[0].thread, caller);
    EXPECT_EQ(calls[0].in, &target.g()) << "not inline in the group's pump";
  }
}

// GR-11 (td 4.9, S1.6): a host_proxied native called inside a generator body, which cannot park,
// on a worker: the worker blocks until the caller runs the body while it waits at the join, and
// the process ends in that same frame (no deadlock; the frame's time is printed, under a generous
// ceiling). The same group pumped by the caller runs it inline.
TEST(Groups, GR11_AProxiedCallInAGeneratorBlocksItsWorker) {
  const auto caller = std::this_thread::get_id();
  native_calls proxied;
  const auto install = [&](group_world &gw) {
    gw.g().install(
        gw.env, "proxied",
        [&](call_args a) -> value_t {
          proxied.note(a.context());
          return value_t(int64_t(7));
        },
        provider_affinity::host_proxied);
  };
  const std::string program = "(begin (set g (generator (lambda () (yield (proxied))))) "
                              "(+ (next g) 1))";
  const auto no_deadline = clock_type::now() + std::chrono::hours(1);
  {
    SCOPED_TRACE("a worker pumps the group");
    pool_app w(4);
    if (w.pool.threads() == 0) {
      ADD_FAILURE() << "the pool has no worker";
    } else {
      caller_hold hold(w, "gr11hold");
      std::thread::id pumped_on;
      group_world target(w, "gr11", group_affinity::any, [&](exec_group::options &o) {
        o.pump_hook_for_test = [&](exec_group &) {
          if (!hold.go.load())
            pumped_on = std::this_thread::get_id();
          hold.go.store(true);
        };
      });
      install(target);
      const int uses = target.submit(program);
      w.pool.set_last_pump_for_test(hold.g(), 1e-3);
      w.pool.set_last_pump_for_test(target.g(), 1e-3);
      const auto t0 = clock_type::now();
      w.pool.pump_all(std::vector<exec_group *>{&hold.g(), &target.g()}, steps_only(100000),
                      no_deadline);
      const double took = seconds_since(t0);
      std::printf("GR-11: the frame took %.3f s (effective %u)\n", took, w.effective());
      EXPECT_LT(took, kCeiling);
      EXPECT_NE(pumped_on, caller) << "the group was pumped on the caller";
      EXPECT_EQ(target.outcome(uses), "8") << "the worker did not wait for the body";
      const std::vector<native_call> calls = proxied.get();
      ASSERT_EQ(calls.size(), 1u);
      EXPECT_EQ(calls[0].thread, caller) << "the proxied body ran off the caller";
      EXPECT_EQ(calls[0].in, nullptr) << "the proxied body ran inside a pump";
    }
  }
  proxied.calls.clear();
  {
    SCOPED_TRACE("the caller pumps the group");
    pool_app w(0);
    group_world target(w, "gr11c");
    install(target);
    const int uses = target.submit(program);
    w.pool.pump_all(std::vector<exec_group *>{&target.g()}, steps_only(100000), no_deadline);
    EXPECT_EQ(target.outcome(uses), "8");
    const std::vector<native_call> calls = proxied.get();
    ASSERT_EQ(calls.size(), 1u);
    EXPECT_EQ(calls[0].thread, caller);
    EXPECT_EQ(calls[0].in, &target.g()) << "not inline in the group's pump";
  }
}

// --- Stage 2a continuous pool (td 8.3, 8.11) ---
//
// S2.md section 4's CT cases and ST-3 (S2a.9): attached groups in continuous mode (exec_pool's
// attach, pump_host, wait_for_work, with_group, detach), against GR-1's oracle, under teardown,
// with host bodies, on their attaching thread, and churned with kernels in flight. The attaching
// thread runs exec_pool.h's host loop and reads a group's scheduler only under its token
// (with_group) or once it is detached.

// clang-format off
#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <tlhelp32.h>
#else
#include <dirent.h>
#if defined(__APPLE__)
#include <mach/mach.h>
#endif
#endif
#include <condition_variable>
// clang-format on

namespace {

#if !defined(_WIN32)
long s2a_entries(const char *dir) {
  DIR *d = opendir(dir);
  if (!d)
    return -1;
  long n = 0;
  while (dirent *e = readdir(d))
    if (e->d_name[0] != '.')
      ++n;
  closedir(d);
  return n;
}
#endif

// This process's threads, or -1 where they can't be counted.
long s2a_thread_count() {
#if defined(_WIN32)
  HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
  if (snap == INVALID_HANDLE_VALUE)
    return -1;
  const DWORD self = GetCurrentProcessId();
  THREADENTRY32 te{};
  te.dwSize = sizeof te;
  long n = 0;
  if (Thread32First(snap, &te)) {
    do {
      if (te.th32OwnerProcessID == self)
        ++n;
      te.dwSize = sizeof te;
    } while (Thread32Next(snap, &te));
  }
  CloseHandle(snap);
  return n;
#elif defined(__APPLE__)
  thread_act_array_t threads = nullptr;
  mach_msg_type_number_t n = 0;
  if (task_threads(mach_task_self(), &threads, &n) != KERN_SUCCESS)
    return -1;
  for (mach_msg_type_number_t i = 0; i < n; ++i)
    mach_port_deallocate(mach_task_self(), threads[i]);
  vm_deallocate(mach_task_self(), reinterpret_cast<vm_address_t>(threads), n * sizeof(*threads));
  return static_cast<long>(n);
#else
  return s2a_entries("/proc/self/task");
#endif
}

// This process's handles (Windows) or open file descriptors, or -1 where they can't be counted.
long s2a_handle_count() {
#if defined(_WIN32)
  DWORD n = 0;
  return GetProcessHandleCount(GetCurrentProcess(), &n) ? static_cast<long>(n) : -1;
#else
#if defined(__APPLE__)
  const long n = s2a_entries("/dev/fd");
#else
  const long n = s2a_entries("/proc/self/fd");
#endif
  return n < 0 ? n : n - 1; // the directory stream's own descriptor
#endif
}

// pool_app with Stage 2's pump budget: the app's exec_pool built first with `threads` workers and
// pumps of `pump_steps` steps and `pump_seconds` (infinite: steps only, for the oracles), an async
// call charged one step and no CPU time.
struct s2a_app {
  cvc::app app;
  cvc::state &tree;
  cvc::exec_pool &pool;

  s2a_app(unsigned threads, std::uint64_t pump_steps, double pump_seconds = kNoTimeLimit,
          bool deterministic = false)
      : tree(cvc::state::instance(app)),
        pool(build(app, tree, threads, pump_steps, pump_seconds, deterministic)) {}

  static cvc::exec_pool &build(cvc::app &a, cvc::state &t, unsigned threads,
                               std::uint64_t pump_steps, double pump_seconds, bool deterministic) {
    t("state_exec.defaults.limits.async_steps_per_cpu_ms").value(std::string("0"));
    cvc::pool_options opts;
    opts.threads = threads;
    opts.pump_steps = pump_steps;
    opts.pump_seconds = pump_seconds;
    opts.deterministic = deterministic;
    return a.exec_pool(opts);
  }
};

// group_world for an app given by its parts: an exec_group named, owned and chrooted `name`, with
// setup_env's environment.
struct s2a_group {
  intrinsics_context ictx;
  environment_ptr env = builtins::make_default_environment();
  std::unique_ptr<exec_group> group;

  s2a_group(cvc::app &app, cvc::state &tree, const std::string &name,
            group_affinity affinity = group_affinity::any,
            const std::function<void(exec_group::options &)> &tweak = {}) {
    exec_group::options opts;
    opts.name = name;
    opts.owner = name;
    opts.affinity = affinity;
    opts.app = &app;
    opts.watch_root = &tree;
    if (tweak)
      tweak(opts);
    group = std::make_unique<exec_group>(std::move(opts));
    setup_env(ictx, env, group->sched(), group->host(), tree, name);
  }
  s2a_group(const s2a_group &) = delete;
  s2a_group &operator=(const s2a_group &) = delete;
  ~s2a_group() { env.reset(); }

  exec_group &g() { return *group; }
  async_scheduler &sched() { return group->sched(); }
  // Before the group is attached, or under its token (with_group).
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

// The attaching thread's host loop (exec_pool.h): until done(), pump_host(pump_seconds), and
// wait_for_work(beat) when that had nothing to do. `in_loop`, when given, is set while the thread
// is inside either. A throw (S2a.0's stubs: "not built") or `ceiling` seconds passing fails the
// case; true when done() held.
bool s2a_drive(cvc::exec_pool &pool, const std::function<bool()> &done, double ceiling = 60.0,
               double beat = 0.01, double pump_seconds = kNoTimeLimit,
               std::atomic<bool> *in_loop = nullptr) {
  const auto t0 = clock_type::now();
  try {
    while (!done()) {
      if (seconds_since(t0) > ceiling) {
        ADD_FAILURE() << "the host loop's ceiling (" << ceiling << " s) passed";
        return false;
      }
      if (in_loop)
        in_loop->store(true);
      if (!pool.pump_host(pump_seconds))
        pool.wait_for_work(beat);
      if (in_loop)
        in_loop->store(false);
    }
    return true;
  } catch (const std::exception &e) {
    if (in_loop)
      in_loop->store(false);
    ADD_FAILURE() << "the host loop threw: " << e.what();
    return false;
  }
}

// The groups a case attached, detached when it goes if still attached, so a case that fails or
// throws leaves nothing attached (~exec_pool asserts so). Declared after the groups.
class s2a_attached {
public:
  explicit s2a_attached(cvc::exec_pool &pool) : pool_(pool) {}
  ~s2a_attached() { detach_all(); }
  s2a_attached(const s2a_attached &) = delete;
  s2a_attached &operator=(const s2a_attached &) = delete;

  void attach(exec_group &g) {
    pool_.attach(g);
    groups_.push_back(&g);
  }
  // A group the case destroys attached (its teardown detaches it).
  void forget(exec_group &g) {
    groups_.erase(std::remove(groups_.begin(), groups_.end(), &g), groups_.end());
  }
  void detach_all() noexcept {
    for (exec_group *g : groups_)
      try {
        if (pool_.attached(*g))
          pool_.detach(*g);
      } catch (...) {
      }
    groups_.clear();
  }

private:
  cvc::exec_pool &pool_;
  std::vector<exec_group *> groups_;
};

// A kernel that runs until its call is cancelled (its process killed, its host shut down), or
// kCeiling passes: `at_cancel` runs in the call's on_cancel hook, on the thread that cancels it.
// `started` and `ended` count the kernels.
async_kernel s2a_kernel_until_cancelled(std::atomic<int> &started, std::atomic<int> &ended,
                                        std::function<void()> at_cancel = {}) {
  return [&started, &ended, at_cancel](async_call &call) -> value_t {
    struct gate {
      std::mutex m;
      std::condition_variable cv;
      bool cancelled = false;
    };
    auto g = std::make_shared<gate>();
    call.on_cancel([g, at_cancel] {
      if (at_cancel)
        at_cancel();
      std::lock_guard<std::mutex> lock(g->m);
      g->cancelled = true;
      g->cv.notify_all();
    });
    started.fetch_add(1);
    {
      std::unique_lock<std::mutex> lock(g->m);
      g->cv.wait_for(lock, std::chrono::duration<double>(kCeiling), [&] { return g->cancelled; });
    }
    ended.fetch_add(1);
    return value_t(int64_t(0));
  };
}

} // namespace

// CT-1 (S2.md section 4; td 8.3): GR-1's oracle in continuous mode. Each group's programs run alone
// on this thread with steps-only pumps of 97 steps; then the groups, attached to the app's pool at
// threads 0, 1, 2, 4 and 8 (the effective count printed), pumped 97 steps at a time with no time
// budget, run until every process has ended (the host loop, reading each group under its token):
// every group's processes (results, steps, statuses) and its registry's tracked objects after a
// final collection equal its solo run. At threads 0 the attaching thread ran every pump (the pump
// hooks' threads, stats().host_pumps == pumps, no worker pump); with workers, workers pumped.
TEST(Groups, CT01_ContinuousEqualsSolo) {
  const int groups = under_sanitizer() ? 4 : 8;
  const std::vector<unsigned> thread_counts =
      under_sanitizer() ? std::vector<unsigned>{0, 1, 4} : std::vector<unsigned>{0, 1, 2, 4, 8};
  const budget_fn budget = [](std::size_t) { return steps_only(97); };
  const std::vector<group_record> solo = solo_records(groups, budget);
  bool no_registry_reported = false;
  for (const unsigned threads : thread_counts) {
    SCOPED_TRACE("threads " + std::to_string(threads));
    s2a_app w(threads, 97);
    const auto caller = std::this_thread::get_id();
    pump_threads where;
    std::vector<std::unique_ptr<s2a_group>> worlds;
    for (int k = 0; k < groups; ++k) {
      worlds.push_back(std::make_unique<s2a_group>(
          w.app, w.tree, "g" + std::to_string(k), group_affinity::any, [&](exec_group::options &o) {
            o.pump_hook_for_test = [&](exec_group &) { where.note(); };
          }));
      for (const std::string &p : oracle_programs(k))
        worlds.back()->submit(p);
    }
    s2a_attached att(w.pool);
    for (auto &gw : worlds)
      att.attach(gw->g());
    std::vector<char> done(groups, 0);
    const auto t0 = clock_type::now();
    const bool drove = s2a_drive(w.pool, [&] {
      bool all = true;
      for (int k = 0; k < groups; ++k) {
        if (!done[k]) {
          bool d = false;
          w.pool.with_group(worlds[k]->g(), [&] { d = worlds[k]->done(); });
          done[k] = d;
        }
        all = all && done[k];
      }
      return all;
    });
    att.detach_all();
    const cvc::pool_stats stats = w.pool.stats();
    std::printf("CT-1: %d groups at threads %u (effective %u): %llu pumps, %llu on the attaching "
                "thread, %llu on a worker, %.3f s\n",
                groups, threads, w.pool.threads(), static_cast<unsigned long long>(stats.pumps),
                static_cast<unsigned long long>(stats.host_pumps),
                static_cast<unsigned long long>(stats.worker_pumps), seconds_since(t0));
    std::fflush(stdout);
    if (!drove)
      return;
    for (int k = 0; k < groups; ++k) {
      const std::string label =
          "CT-1 at threads " + std::to_string(threads) + ", group " + std::to_string(k);
      EXPECT_TRUE(worlds[k]->done()) << label << " never ended";
      expect_same(record(worlds[k]->sched(), worlds[k]->g().registry()), solo[k], label,
                  no_registry_reported);
    }
    const std::vector<std::thread::id> ids = where.get();
    EXPECT_FALSE(ids.empty());
    if (w.pool.threads() == 0) {
      EXPECT_TRUE(std::all_of(ids.begin(), ids.end(), [&](std::thread::id t) {
        return t == caller;
      })) << "a pump ran off the attaching thread with no worker";
      EXPECT_EQ(stats.worker_pumps, 0u);
      EXPECT_EQ(stats.host_pumps, stats.pumps);
      EXPECT_EQ(stats.pumps, static_cast<std::uint64_t>(ids.size()));
    } else {
      EXPECT_GT(stats.worker_pumps, 0u) << "no worker pumped a group";
    }
  }
}

// CT-2 (S2.md section 4; S2-Q4): attached groups destroyed while workers re-pump them, under
// CVC_EXEC_FUZZ, 10^3 rounds (10^2 in Debug and under a sanitizer). Each round attaches four groups
// (three busy loops and one whose process waits on a background kernel), lets each be pumped twice
// and the kernel start, then destroys them in a shuffled order while the workers still re-pump the
// busy ones. Every teardown returns (no hang; debug builds: no assertion); no group is pumped once
// its teardown holds the token (its teardown hook sets the flag its pump hook reads); and from its
// detach on no post reaches the pool: in the kernel's cancel hook, which the teardown's kill runs
// after the detach, the group has no notifier and is not attached, and a post to it there moves no
// wake counter (stats().wakes: the busy groups are requeued, not woken).
TEST(Groups, CT02_TeardownDuringContinuousPumps) {
  scoped_env_var fuzz("CVC_EXEC_FUZZ", "2026");
  const int rounds = under_sanitizer() || kDebugBuild ? 100 : 1000;
  s2a_app w(4, 300); // the pool reads CVC_EXEC_FUZZ now, the lock manager at the first group
  std::mt19937 rng(2026);
  std::atomic<int> late_pumps{0}, started{0}, ended{0};
  int checked = 0, with_notifier = 0, still_attached = 0, woke = 0;
  const auto t0 = clock_type::now();
  int round = 0;
  for (; round < rounds && !::testing::Test::HasFailure(); ++round) {
    constexpr int kGroups = 4;
    std::vector<std::shared_ptr<std::atomic<bool>>> torn;
    std::vector<std::shared_ptr<std::atomic<int>>> pumps;
    std::vector<std::unique_ptr<s2a_group>> gs;
    for (int k = 0; k < kGroups; ++k) {
      auto t = std::make_shared<std::atomic<bool>>(false);
      auto p = std::make_shared<std::atomic<int>>(0);
      torn.push_back(t);
      pumps.push_back(p);
      gs.push_back(std::make_unique<s2a_group>(
          w.app, w.tree, "ct02g" + std::to_string(k), group_affinity::any,
          [&late_pumps, t, p](exec_group::options &o) {
            o.pump_hook_for_test = [&late_pumps, t, p](exec_group &) {
              p->fetch_add(1);
              if (t->load())
                late_pumps.fetch_add(1);
            };
            o.teardown_hook_for_test = [t](exec_group &) { t->store(true); };
          }));
    }
    exec_group *const waiter = &gs[kGroups - 1]->g();
    register_async_fn(gs[kGroups - 1]->env, waiter->host(), "ct02-wait",
                      [&, waiter](std::span<const value_t>) -> async_kernel {
                        return s2a_kernel_until_cancelled(started, ended, [&, waiter] {
                          ++checked;
                          if (waiter->sched().has_wake_notifier())
                            ++with_notifier;
                          if (w.pool.attached(*waiter))
                            ++still_attached;
                          const std::uint64_t before = w.pool.stats().wakes;
                          waiter->sched().post_message("ct02.late", value_t(int64_t(1)));
                          if (w.pool.stats().wakes != before)
                            ++woke;
                        });
                      });
    for (int k = 0; k + 1 < kGroups; ++k)
      gs[k]->submit("(begin (set i 0) (while #t (set i (+ i 1))))");
    gs[kGroups - 1]->submit("(begin (ct02-wait) \"never\")");
    // Read before attaching: a worker may start the waiter's kernel as soon as it is attached.
    const int target = started.load() + 1;
    s2a_attached att(w.pool);
    for (auto &gw : gs)
      att.attach(gw->g());
    const bool drove = s2a_drive(
        w.pool,
        [&] {
          return started.load() >= target &&
                 std::all_of(pumps.begin(), pumps.end(),
                             [](const auto &p) { return p->load() >= 2; });
        },
        kCeiling);
    if (!drove)
      break; // att detaches them
    std::vector<int> order(kGroups);
    for (int k = 0; k < kGroups; ++k)
      order[k] = k;
    std::shuffle(order.begin(), order.end(), rng);
    unsigned left = kGroups;
    for (const int k : order) {
      att.forget(gs[k]->g());
      gs[k].reset(); // attached, with the workers re-pumping it (or the kernel in flight)
      --left;
      EXPECT_EQ(w.pool.stats().attached, left)
          << "round " << round << ": a teardown left it attached";
    }
  }
  std::printf(
      "CT-2: %d of %d rounds in %.3f s; late pumps %d; cancel hooks %d (with a notifier %d, "
      "attached %d, a post that woke %d); kernels %d started, %d ended\n",
      round, rounds, seconds_since(t0), late_pumps.load(), checked, with_notifier, still_attached,
      woke, started.load(), ended.load());
  EXPECT_EQ(round, rounds);
  EXPECT_EQ(late_pumps.load(), 0) << "a group was pumped after its teardown took the token";
  EXPECT_EQ(checked, round) << "a teardown's kill did not cancel the kernel";
  EXPECT_EQ(with_notifier, 0) << "a group still had its notifier after its detach";
  EXPECT_EQ(still_attached, 0) << "a group was still attached at its kills";
  EXPECT_EQ(woke, 0) << "a post after the detach woke the group in the pool";
  EXPECT_EQ(started.load(), ended.load());
  EXPECT_EQ(w.pool.stats().attached, 0u);
  EXPECT_EQ(detail::live_groups(w.app), 0u);
}

// CT-3 (S2.md section 4; S2-Q4, S2-Q5): host bodies and the token in continuous mode, the group
// pumped by workers (an `any` group of a shared manager), a busy process of its own keeping it
// pumped and queued.
// - A non-waiting host_proxied body runs on the attaching thread inside its host loop (pump_host or
//   wait_for_work), under the group's token: not inside a pump (pumping_here() false); no thread
//   but this one holds the token (pumped_elsewhere() false here, before and after, and true from a
//   helper thread in between); no pump of the group begins while it runs.
// - A waiting body (the proxied call in a generator, which cannot park) runs on the attaching
//   thread while its blocked worker holds the token (pumped_elsewhere() true), and no other pump
//   begins.
// - with_group while the busy group is queued: a worker pops it meanwhile and leaves it notified
//   (the state word's `notified`), no pump begins under with_group, and it is pumped after the
//   release.
TEST(Groups, CT03_HostBodiesAndTheToken) {
  s2a_app w(2, 200);
  if (w.pool.threads() == 0) {
    ADD_FAILURE() << "the pool has no worker";
    return;
  }
  const auto caller = std::this_thread::get_id();
  std::atomic<std::uint64_t> pumps{0};
  s2a_group gw(w.app, w.tree, "ct03", group_affinity::any, [&](exec_group::options &o) {
    o.pump_hook_for_test = [&](exec_group &) { pumps.fetch_add(1); };
  });
  exec_group &g = gw.g();
  std::atomic<bool> in_host_loop{false};
  struct body_view {
    std::atomic<bool> ran{false};
    std::thread::id thread;
    bool in_host_loop = false, pumping_here = true, mine_before = false, mine_after = false,
         helper_saw_owner = false;
    std::uint64_t pumps_before = 0, pumps_after = 0;
  };
  body_view plain, waiting;
  const auto observe = [&](body_view &v) {
    v.thread = std::this_thread::get_id();
    v.in_host_loop = in_host_loop.load();
    v.pumping_here = g.pumping_here();
    v.pumps_before = pumps.load();
    v.mine_before = !g.sched().pumped_elsewhere();
    bool seen = false;
    std::thread([&] { seen = g.sched().pumped_elsewhere(); }).join();
    v.helper_saw_owner = seen;
    std::this_thread::sleep_for(std::chrono::milliseconds(2)); // a pump beside it would begin now
    v.mine_after = !g.sched().pumped_elsewhere();
    v.pumps_after = pumps.load();
    v.ran.store(true);
  };
  g.install(
      gw.env, "plain-body",
      [&](std::span<const value_t>) -> value_t {
        observe(plain);
        return value_t(int64_t(1));
      },
      provider_affinity::host_proxied);
  g.install(
      gw.env, "waiting-body",
      [&](std::span<const value_t>) -> value_t {
        observe(waiting);
        return value_t(int64_t(2));
      },
      provider_affinity::host_proxied);
  const int busy = gw.submit("(begin (set i 0) (while #t (set i (+ i 1))))");
  const int p1 = gw.submit("(begin (for i (range 50) nil) (+ (plain-body) 10))");
  const int p2 = gw.submit("(begin (for i (range 50) nil) (set gen (generator (lambda () "
                           "(yield (waiting-body))))) (+ (next gen) 20))");
  s2a_attached att(w.pool);
  att.attach(g);
  bool drove = s2a_drive(
      w.pool, [&] { return plain.ran.load() && waiting.ran.load(); }, kCeiling, 0.01, kNoTimeLimit,
      &in_host_loop);
  bool notified_seen = false;
  std::uint64_t held_before = 0, held_after = 0, at_release = 0;
  if (drove) {
    try {
      w.pool.with_group(g, [&] {
        held_before = pumps.load();
        const auto t0 = clock_type::now();
        const auto notified = [&] {
          return (g.pool_link().state.load() & detail::group_pool_link::notified) != 0;
        };
        while (!notified() && seconds_since(t0) < kCeiling)
          std::this_thread::yield();
        notified_seen = notified();
        held_after = pumps.load();
      });
      at_release = pumps.load();
    } catch (const std::exception &e) {
      ADD_FAILURE() << "with_group threw: " << e.what();
      drove = false;
    }
  }
  if (drove)
    drove = s2a_drive(w.pool, [&] { return pumps.load() > at_release; }, kCeiling);
  if (drove) {
    w.pool.with_group(g, [&] { g.sched().kill(busy); });
    drove = s2a_drive(
        w.pool,
        [&] {
          bool d = false;
          w.pool.with_group(g, [&] { d = finished(g.sched(), p1) && finished(g.sched(), p2); });
          return d;
        },
        kCeiling);
  }
  att.detach_all();
  ASSERT_TRUE(drove);
  EXPECT_EQ(gw.outcome(p1), "11");
  EXPECT_EQ(gw.outcome(p2), "22");
  EXPECT_EQ(plain.thread, caller) << "the plain body ran off the attaching thread";
  EXPECT_TRUE(plain.in_host_loop) << "the plain body ran outside pump_host and wait_for_work";
  EXPECT_FALSE(plain.pumping_here) << "the plain body ran inside a pump";
  EXPECT_TRUE(plain.mine_before && plain.mine_after) << "another thread held the token";
  EXPECT_TRUE(plain.helper_saw_owner) << "the plain body ran without the group's token";
  EXPECT_EQ(plain.pumps_after, plain.pumps_before) << "a pump began beside the plain body";
  EXPECT_EQ(waiting.thread, caller) << "the waiting body ran off the attaching thread";
  EXPECT_TRUE(waiting.in_host_loop) << "the waiting body ran outside pump_host and wait_for_work";
  EXPECT_FALSE(waiting.pumping_here);
  EXPECT_FALSE(waiting.mine_before) << "the waiting body's worker did not hold the token";
  EXPECT_TRUE(waiting.helper_saw_owner);
  EXPECT_EQ(waiting.pumps_after, waiting.pumps_before) << "a pump began beside the waiting body";
  EXPECT_TRUE(notified_seen) << "a worker's pop under with_group did not leave the group notified";
  EXPECT_EQ(held_after, held_before) << "a pump began under with_group";
  EXPECT_GT(pumps.load(), at_release) << "not pumped after with_group's release";
}

// CT-4 (S2.md section 4; S2-Q4, td 4.14): groups that stay on their attaching thread. A pool with
// four workers, built apart from the apps, has three groups attached: one of an app whose lock
// manager is single (caller_only), a host-affinity one of an app whose manager is shared, and an
// `any` group of that app beside them. Each runs a loop over many 200-step pumps; the pump hooks
// record the threads: the first two were pumped by the attaching thread only, the `any` one by
// workers only.
TEST(Groups, CT04_CallerOnlyGroupsStayOnTheirThread) {
  cvc::pool_options po;
  po.threads = 4;
  po.pump_steps = 200;
  po.pump_seconds = kNoTimeLimit;
  cvc::exec_pool pool(po);
  if (pool.threads() == 0) {
    ADD_FAILURE() << "the pool has no worker";
    return;
  }
  cvc::app single_app, shared_app;
  cvc::state &single_tree = cvc::state::instance(single_app);
  cvc::state &shared_tree = cvc::state::instance(shared_app);
  single_tree("state_exec.locks.threading").value(std::string("single"));
  shared_tree("state_exec.locks.threading").value(std::string("shared"));
  pump_threads single_on, host_on, any_on;
  const auto noting = [](pump_threads &where) {
    return [&where](exec_group::options &o) {
      o.pump_hook_for_test = [&where](exec_group &) { where.note(); };
    };
  };
  s2a_group single(single_app, single_tree, "ct04single", group_affinity::any, noting(single_on));
  s2a_group host(shared_app, shared_tree, "ct04host", group_affinity::host, noting(host_on));
  s2a_group roaming(shared_app, shared_tree, "ct04any", group_affinity::any, noting(any_on));
  ASSERT_TRUE(single.g().pool_link().caller_only);
  ASSERT_FALSE(host.g().pool_link().caller_only);
  ASSERT_FALSE(roaming.g().pool_link().caller_only);
  std::atomic<int> completed{0};
  const auto count = [&](value_t) { completed.fetch_add(1); };
  const std::string loop = "(begin (set i 0) (while (< i 20000) (set i (+ i 1))) \"done\")";
  const int ps = single.submit(loop, count), ph = host.submit(loop, count),
            pa = roaming.submit(loop, count);
  s2a_attached att(pool);
  att.attach(single.g());
  att.attach(host.g());
  att.attach(roaming.g());
  const bool drove = s2a_drive(pool, [&] { return completed.load() == 3; }, 60.0, 0.01, 0.05);
  att.detach_all();
  const cvc::pool_stats s = pool.stats();
  const auto caller = std::this_thread::get_id();
  const std::vector<std::thread::id> si = single_on.get(), hi = host_on.get(), ai = any_on.get();
  const auto all_caller = [&](const std::vector<std::thread::id> &ids) {
    return std::all_of(ids.begin(), ids.end(), [&](std::thread::id t) { return t == caller; });
  };
  const auto none_caller = [&](const std::vector<std::thread::id> &ids) {
    return std::none_of(ids.begin(), ids.end(), [&](std::thread::id t) { return t == caller; });
  };
  std::printf("CT-4: pumps: single-manager %zu, host-affinity %zu, any %zu; host pumps %llu, "
              "worker pumps %llu\n",
              si.size(), hi.size(), ai.size(), static_cast<unsigned long long>(s.host_pumps),
              static_cast<unsigned long long>(s.worker_pumps));
  ASSERT_TRUE(drove);
  EXPECT_EQ(single.outcome(ps), "done");
  EXPECT_EQ(host.outcome(ph), "done");
  EXPECT_EQ(roaming.outcome(pa), "done");
  EXPECT_GT(si.size(), 1u);
  EXPECT_GT(hi.size(), 1u);
  EXPECT_TRUE(all_caller(si)) << "a worker pumped the single-manager group";
  EXPECT_TRUE(all_caller(hi)) << "a worker pumped the host-affinity group";
  EXPECT_TRUE(none_caller(ai)) << "the attaching thread pumped the any group";
  EXPECT_GE(s.host_pumps, static_cast<std::uint64_t>(si.size() + hi.size()));
}

// ST-3 (td 8.8): group churn with kernels in flight. 10^4 groups in Release (500 in Debug, 300
// under a sanitizer), eight at a time, each attached with one program parked on a background
// kernel that runs until cancelled, destroyed attached once its kernel has started: the teardown's
// kill cancels the kernel and its host waits for it. After a warm-up batch (the kernel pool's
// threads), the process's thread count stays flat (at most 4 more) and its handles too (at most
// 64 + 1 per 50 groups more), every kernel ended, and no group is left (detail::live_groups) or
// attached. ASan in CI checks for leaks.
TEST(Groups, ST03_GroupChurnWithKernelsInFlight) {
  const int total = under_sanitizer() ? 300 : kDebugBuild ? 500 : 10000;
  constexpr int kBatch = 8;
  s2a_app w(4, 2000);
  std::atomic<int> started{0}, ended{0};
  int made = 0;
  const auto churn = [&](int n) -> bool {
    for (int done = 0; done < n; done += kBatch) {
      std::vector<std::unique_ptr<s2a_group>> batch;
      const int target = started.load() + kBatch;
      for (int k = 0; k < kBatch; ++k) {
        batch.push_back(std::make_unique<s2a_group>(w.app, w.tree, "st03g" + std::to_string(k)));
        s2a_group &gw = *batch.back();
        register_async_fn(gw.env, gw.g().host(), "st03-wait",
                          [&](std::span<const value_t>) -> async_kernel {
                            return s2a_kernel_until_cancelled(started, ended);
                          });
        gw.submit("(begin (st03-wait) \"never\")");
        w.pool.attach(gw.g());
        ++made;
      }
      if (!s2a_drive(w.pool, [&] { return started.load() >= target; }, kCeiling)) {
        for (auto &gw : batch)
          if (w.pool.attached(gw->g()))
            w.pool.detach(gw->g());
        return false;
      }
      batch.clear(); // destroyed attached, their kernels in flight
      if (w.pool.stats().attached != 0) {
        ADD_FAILURE() << "a destroyed group was left attached";
        return false;
      }
    }
    return true;
  };
  ASSERT_TRUE(churn(kBatch)) << "the warm-up batch";
  const long threads0 = s2a_thread_count(), handles0 = s2a_handle_count();
  const auto t0 = clock_type::now();
  const bool churned = churn(total);
  const double took = seconds_since(t0);
  const long threads1 = s2a_thread_count(), handles1 = s2a_handle_count();
  std::printf("ST-3: %d groups (%d with the warm-up) in %.3f s at threads %u; threads %ld -> %ld, "
              "handles %ld -> %ld; kernels %d started, %d ended\n",
              total, made, took, w.pool.threads(), threads0, threads1, handles0, handles1,
              started.load(), ended.load());
  ASSERT_TRUE(churned);
  EXPECT_EQ(started.load(), made);
  EXPECT_EQ(ended.load(), started.load()) << "a kernel outlived its group";
  EXPECT_EQ(detail::live_groups(w.app), 0u);
  EXPECT_EQ(w.pool.stats().attached, 0u);
  if (threads0 >= 0 && threads1 >= 0)
    EXPECT_LE(threads1, threads0 + 4) << "threads grew with the groups";
  if (handles0 >= 0 && handles1 >= 0)
    EXPECT_LE(handles1, handles0 + 64 + total / 50) << "handles grew with the groups";
}
