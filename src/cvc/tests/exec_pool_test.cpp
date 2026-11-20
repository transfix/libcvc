// exec_group and exec_pool unit tests (threaded execution Stage 1, S1.md S1.5-S1.8): the pool's
// own mechanics (placement, clamps, the helper wake, the deadline, failures, on_complete, the
// host-call queue, several callers), the group's (teardown, the AST copy, install by kind,
// wants_pump) and the app's (exec_pool(), the lock-mode hint, exec_pump_frame, pids). The Groups
// suite (state_exec_groups_test.cpp) holds the plan's GR/DT/... cases. No case name contains
// "Stress": run.sh filters those out.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/core/cpu_placement.h>
#include <cvc/core/exec_pool.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/exec_group.h>
#include <cvc/state/state_exec/heap.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/lock_manager.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/stdlib.h>
#include <cvc/state/state_exec/types.h>
#include <functional>
#include <gtest/gtest.h>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace cvc::state_exec;
using clock_type = std::chrono::steady_clock;

namespace {

constexpr pump_budget kSteps{100000, std::numeric_limits<double>::infinity()};

clock_type::time_point far_deadline() { return clock_type::time_point::max(); }

exec_group::options opts_for(cvc::app &app, const std::string &name,
                             group_affinity affinity = group_affinity::any) {
  exec_group::options o;
  o.name = name;
  o.app = &app;
  o.affinity = affinity;
  return o;
}

// A process env with a `tid` native that records the pumping thread's id. One per group: a
// top-level set binds in the env it is given, which two groups must never share.
struct thread_log {
  std::mutex m;
  std::vector<std::thread::id> ids;
  void add() {
    std::lock_guard<std::mutex> lock(m);
    ids.push_back(std::this_thread::get_id());
  }
  std::set<std::thread::id> distinct() {
    std::lock_guard<std::mutex> lock(m);
    return {ids.begin(), ids.end()};
  }
};

environment_ptr env_with_tid(thread_log &log) {
  environment_ptr env = environment::extend(builtins::make_default_environment());
  builtins::register_fn(env, "tid", [&log](std::span<const value_t>) -> value_t {
    log.add();
    return value_t(int64_t(1));
  });
  return env;
}

// A program that records its thread, then counts to n.
std::string counting(int n) {
  return "(begin (tid) (set i 0) (while (< i " + std::to_string(n) + ") (set i (+ i 1))) (tid) i)";
}

// As counting, but it notes its thread on every pass of the loop, so every pump of it (which runs
// hundreds of steps) shows up in the log, not only the pumps that hold its first or last step.
std::string counting_noting_each_pass(int n) {
  return "(begin (set i 0) (while (< i " + std::to_string(n) +
         ") (begin (tid) (set i (+ i 1)))) i)";
}

bool finished(exec_group &g, int pid) {
  const auto info = g.sched().get_process_info(pid);
  return info && is_finished(info->status);
}

std::string result_of(exec_group &g, int pid) {
  const auto r = g.sched().get_result(pid);
  return r ? to_string(*r) : std::string("<none>");
}

// Pump every group until each pid in `pids` has finished (at most `rounds` pump_all calls).
void pump_until_done(cvc::exec_pool &pool, std::vector<exec_group *> &groups,
                     const std::vector<std::pair<exec_group *, int>> &pids, int rounds = 1000) {
  for (int r = 0; r < rounds; ++r) {
    bool done = true;
    for (const auto &[g, pid] : pids)
      done = done && finished(*g, pid);
    if (done)
      return;
    pool.pump_all(groups, kSteps, far_deadline());
  }
}

// Give `app` a shared lock manager before anything makes it: a pool with workers pumps a group on
// a single manager on its caller only (S1-Q1), and asserts so in debug builds.
void share_locks(cvc::app &app) {
  app.root()("state_exec.locks.threading").value(std::string("shared"));
}

// A group with the intrinsics (await, spawn, with-lock...) in an env of its own, chrooted at its
// name on the app's tree.
struct intrinsic_world {
  intrinsics_context ictx;
  environment_ptr env = environment::extend(builtins::make_default_environment());
  std::unique_ptr<exec_group> group;
  intrinsic_world(cvc::app &app, const std::string &name,
                  group_affinity affinity = group_affinity::any, const std::string &chroot = "") {
    group = std::make_unique<exec_group>(opts_for(app, name, affinity));
    ictx.sched = &group->sched();
    ictx.proc = make_process();
    ictx.proc->pid = 0;
    apply_chroot(ictx, app.root(), chroot.empty() ? name : chroot);
    ictx.host = &group->host();
    register_intrinsics(env, &ictx);
  }
  int submit(const std::string &src, std::function<void(value_t)> on_complete = {}) {
    execute_options eo;
    eo.env = environment::extend(env);
    eo.on_complete = std::move(on_complete);
    return group->execute(src, eo);
  }
};

constexpr double kInf = std::numeric_limits<double>::infinity();

// Keeps the caller of a pump_all inside a host-affinity group's pump until `go` is set (or 10 s
// pass), so a worker takes the call's other groups (the caller pumps its own first); a target
// group's pump hook sets `go`. Its one process loops, so it always has a pump.
struct hold_caller {
  std::atomic<bool> go{false};
  std::unique_ptr<exec_group> g;
  explicit hold_caller(cvc::app &app) {
    exec_group::options o = opts_for(app, "hold", group_affinity::host);
    o.pump_hook_for_test = [this](exec_group &) {
      const auto t0 = clock_type::now();
      while (!go.load() && clock_type::now() - t0 < std::chrono::seconds(10))
        std::this_thread::yield();
    };
    g = std::make_unique<exec_group>(o);
    g->execute(std::string("(while #t nil)"));
  }
};

constexpr bool kDebugBuild =
#ifdef NDEBUG
    false;
#else
    true;
#endif

} // namespace

// --- exec_group and exec_pool unit tests (P) ---

TEST(CpuPlacement, PlacementListsAreSubsetsOfTheProcessCpus) {
  const std::vector<cvc::logical_cpu> all = cvc::logical_cpus();
  const std::vector<cvc::logical_cpu> perf = cvc::placement_cpus(cvc::cpu_set::performance);
  const std::vector<cvc::logical_cpu> every = cvc::placement_cpus(cvc::cpu_set::all);
  std::cout << "logical cpus " << all.size() << ", performance " << perf.size()
            << ", performance_core_count " << cvc::performance_core_count() << "\n";
  EXPECT_EQ(every.size(), all.size());
  EXPECT_LE(perf.size(), all.size());
  EXPECT_GE(cvc::performance_core_count(), 1u);
  if (!all.empty()) {
    EXPECT_EQ(cvc::performance_core_count(), perf.size());
    // `all`: the faster class first (the driver's CPU, last, aside).
    for (std::size_t i = 1; i + 1 < every.size(); ++i)
      EXPECT_GE(every[i - 1].efficiency, every[i].efficiency);
    // One logical CPU per physical core before any sibling: the first workers never share a core.
    std::set<int> cores;
    for (const cvc::logical_cpu &c : perf)
      cores.insert(c.core);
    const std::size_t distinct = cores.size();
    std::set<int> first_cores;
    for (std::size_t i = 0; i + 1 < distinct && i < perf.size(); ++i)
      EXPECT_TRUE(first_cores.insert(perf[i].core).second)
          << "entry " << i << " shares core " << perf[i].core << " with an earlier one";
    // The driver's CPU is last in both lists, off logical CPU 0's core when there is another.
    EXPECT_EQ(perf.back().index, every.back().index);
    int core0 = -1;
    for (const cvc::logical_cpu &c : all)
      if (c.index == 0)
        core0 = c.core;
    if (distinct > 1)
      EXPECT_NE(perf.back().core, core0);
#if defined(_WIN32) || defined(__linux__)
    // A thread of its own, pinned and then gone: the caller's affinity is left alone.
    cvc::pin_result r;
    std::thread([&] { r = cvc::pin_current_thread(); }).join();
    std::cout << "pin: cpu " << r.cpu << " (" << r.how << ")\n";
    EXPECT_GE(r.cpu, 0);
    EXPECT_EQ(r.cpu, perf.back().index) << "auto pins to the CPU no pool worker takes";
    bool placed = false;
    std::thread([&] { placed = cvc::place_current_thread(cvc::cpu_set::all, 0); }).join();
    EXPECT_TRUE(placed);
#endif
  }
}

TEST(ExecPool, ThreadsAreClampedAndReported) {
  cvc::pool_options o;
  o.threads = 100000;
  cvc::exec_pool pool(o);
  const unsigned hw = std::max(1u, std::thread::hardware_concurrency());
  EXPECT_LE(pool.threads(), hw - 1);
  EXPECT_EQ(pool.stats().effective_threads, pool.threads());
  EXPECT_EQ(pool.options().threads, 100000u);
  if (threads_compiled_in && hw > 1)
    EXPECT_FALSE(pool.take_notes().empty()) << "the clamp is noted for the owner to log";
  EXPECT_TRUE(pool.take_notes().empty()) << "notes are taken once";
  cvc::pool_options a;
  a.threads = cvc::pool_options::auto_threads;
  cvc::exec_pool automatic(a);
  EXPECT_EQ(automatic.threads(),
            threads_compiled_in ? std::min(cvc::performance_core_count(), hw) - 1 : 0u);
}

TEST(ExecPool, ThreadsZeroPumpsEveryGroupOnTheCallerInCreationOrder) {
  cvc::app app;
  cvc::exec_pool pool(cvc::pool_options{});
  thread_log log;
  std::vector<std::unique_ptr<exec_group>> owned;
  std::vector<exec_group *> groups;
  std::vector<std::string> order;
  for (int i = 0; i < 4; ++i) {
    exec_group::options o = opts_for(app, "g" + std::to_string(i));
    o.pump_hook_for_test = [&order](exec_group &g) { order.push_back(g.opts().name); };
    owned.push_back(std::make_unique<exec_group>(o));
  }
  // Given in reverse: pumped in creation order all the same (td 4.14).
  for (int i = 3; i >= 0; --i)
    groups.push_back(owned[i].get());
  std::vector<std::pair<exec_group *, int>> pids;
  for (auto &g : owned) {
    execute_options eo;
    eo.env = env_with_tid(log);
    pids.emplace_back(g.get(), g->execute(counting(50), eo));
  }
  pool.pump_all(groups, kSteps, far_deadline());
  EXPECT_EQ(order, (std::vector<std::string>{"g0", "g1", "g2", "g3"}));
  for (const auto &[g, pid] : pids) {
    ASSERT_TRUE(finished(*g, pid));
    EXPECT_EQ(result_of(*g, pid), "50");
  }
  EXPECT_EQ(log.distinct(), std::set<std::thread::id>{std::this_thread::get_id()});
  EXPECT_EQ(pool.stats().pumps, 4u);
  EXPECT_EQ(pool.stats().worker_pumps, 0u);
  EXPECT_EQ(pool.stats().helper_wakes, 0u);
}

TEST(ExecPool, HeavyGroupsGoToWorkersAndHostGroupsStayOnTheCaller) {
  cvc::pool_options po;
  po.threads = 3;
  cvc::exec_pool pool(po);
  if (pool.threads() == 0)
    GTEST_SKIP() << "no worker on this machine";
  cvc::app app;
  share_locks(app);
  thread_log any_log, host_log;
  std::vector<std::unique_ptr<exec_group>> owned;
  std::vector<exec_group *> groups;
  for (int i = 0; i < 6; ++i) {
    owned.push_back(std::make_unique<exec_group>(opts_for(
        app, "g" + std::to_string(i), i == 0 ? group_affinity::host : group_affinity::any)));
    groups.push_back(owned.back().get());
  }
  std::vector<std::pair<exec_group *, int>> pids;
  for (auto &g : owned) {
    execute_options eo;
    eo.env = env_with_tid(g->opts().affinity == group_affinity::host ? host_log : any_log);
    pids.emplace_back(g.get(), g->execute(counting_noting_each_pass(20000), eo));
  }
  // Injected as heavy (GR-9's hook), so the helpers are woken from the first frame.
  for (int round = 0; round < 5000; ++round) {
    for (exec_group *g : groups)
      pool.set_last_pump_for_test(*g, 1.0);
    pool.pump_all(groups, pump_budget{500, std::numeric_limits<double>::infinity()},
                  far_deadline());
    bool done = true;
    for (const auto &[g, pid] : pids)
      done = done && finished(*g, pid);
    if (done)
      break;
  }
  for (const auto &[g, pid] : pids) {
    ASSERT_TRUE(finished(*g, pid)) << g->opts().name;
    EXPECT_EQ(result_of(*g, pid), "20000");
  }
  const cvc::pool_stats s = pool.stats();
  std::cout << "pumps " << s.pumps << ", worker pumps " << s.worker_pumps << ", helper wakes "
            << s.helper_wakes << ", threads " << s.effective_threads << "\n";
  EXPECT_GT(s.helper_wakes, 0u);
  EXPECT_GT(s.worker_pumps, 0u);
  EXPECT_EQ(host_log.distinct(), std::set<std::thread::id>{std::this_thread::get_id()})
      << "a host-affinity group is pumped by the caller only";
  // Every worker pump of an any group runs some of its loop and notes a worker's thread. (Which
  // threads ran an any group's first and last steps is up to the scheduler: on a two-thread runner
  // the caller can take them all while a worker pumps the middles.)
  const std::set<std::thread::id> any_threads = any_log.distinct();
  EXPECT_TRUE(std::any_of(any_threads.begin(), any_threads.end(), [](std::thread::id t) {
    return t != std::this_thread::get_id();
  })) << "workers pumped the any groups too";
}

TEST(ExecPool, TheHelperWakeFollowsTheLastPumps) {
  cvc::pool_options po;
  po.threads = 2;
  cvc::exec_pool pool(po);
  cvc::app app;
  share_locks(app);
  exec_group a(opts_for(app, "a")), b(opts_for(app, "b"));
  std::vector<exec_group *> groups{&a, &b};
  for (int round = 0; round < 5; ++round) {
    pool.set_last_pump_for_test(a, 10e-6);
    pool.set_last_pump_for_test(b, 10e-6);
    pool.pump_all(groups, kSteps, far_deadline());
  }
  EXPECT_EQ(pool.stats().helper_wakes, 0u);
  EXPECT_EQ(pool.stats().worker_pumps, 0u);
  EXPECT_EQ(pool.stats().pumps, 10u);
  if (pool.threads() < 1)
    return;
  // Above inline_below: one helper per group beyond the caller's first, counted as it is woken,
  // whichever thread then gets to the second group.
  pool.set_last_pump_for_test(a, 1.0);
  pool.set_last_pump_for_test(b, 1.0);
  pool.pump_all(groups, kSteps, far_deadline());
  EXPECT_EQ(pool.stats().helper_wakes, 1u);
}

TEST(ExecPool, APassedDeadlineSkipsGroupsAndTheyGoFirstNextTime) {
  for (unsigned threads : {0u, 2u}) {
    cvc::pool_options po;
    po.threads = threads;
    cvc::exec_pool pool(po);
    cvc::app app;
    share_locks(app);
    std::mutex order_m; // a woken helper would pump beside the caller
    std::vector<std::string> order;
    std::vector<std::unique_ptr<exec_group>> owned;
    std::vector<exec_group *> groups;
    for (int i = 0; i < 3; ++i) {
      exec_group::options o = opts_for(app, "g" + std::to_string(i));
      o.pump_hook_for_test = [&order, &order_m](exec_group &g) {
        std::lock_guard<std::mutex> lock(order_m);
        order.push_back(g.opts().name);
      };
      owned.push_back(std::make_unique<exec_group>(o));
      groups.push_back(owned.back().get());
    }
    pool.pump_all(groups, kSteps, clock_type::now() - std::chrono::seconds(1));
    EXPECT_TRUE(order.empty()) << "nothing starts past the deadline";
    EXPECT_EQ(pool.stats().skipped, 3u);
    // g1 alone is pumped, then skipped g0 and g2 go before it. The order is the caller's: with
    // the last pumps under inline_below no helper is woken, so the caller pumps every group. (g1's
    // measured pump can pass 100 us on a loaded runner; a helper then pumped beside the caller,
    // in no set order, and the unguarded push here lost an entry or corrupted the heap.)
    std::vector<exec_group *> one{owned[1].get()};
    pool.pump_all(one, kSteps, far_deadline());
    for (exec_group *g : groups)
      pool.set_last_pump_for_test(*g, 10e-6);
    pool.pump_all(groups, kSteps, far_deadline());
    EXPECT_EQ(pool.stats().helper_wakes, 0u) << "threads " << threads;
    ASSERT_EQ(order.size(), 4u);
    EXPECT_EQ(order[0], "g1");
    EXPECT_EQ(std::set<std::string>(order.begin() + 1, order.begin() + 3),
              (std::set<std::string>{"g0", "g2"}))
        << "threads " << threads;
    EXPECT_EQ(order[3], "g1") << "threads " << threads;
  }
}

TEST(ExecPool, AnExceptionEscapingAPumpFailsOnlyItsGroup) {
  for (unsigned threads : {0u, 2u}) {
    cvc::pool_options po;
    po.threads = threads;
    cvc::exec_pool pool(po);
    cvc::app app;
    share_locks(app);
    exec_group::options bad_opts = opts_for(app, "bad");
    bad_opts.pump_hook_for_test = [](exec_group &) { throw std::runtime_error("boom"); };
    exec_group bad(bad_opts), good(opts_for(app, "good"));
    const int bad_pid = bad.execute(std::string("(begin (sleep 10) 1)"));
    const int good_pid = good.execute(std::string("(+ 40 2)"));
    std::vector<exec_group *> groups{&bad, &good};
    pool.set_last_pump_for_test(bad, 1.0);
    pool.set_last_pump_for_test(good, 1.0);
    std::vector<exec_group *> failed;
    EXPECT_NO_THROW(failed = pool.pump_all(groups, kSteps, far_deadline()));
    EXPECT_TRUE(bad.failed());
    EXPECT_FALSE(good.failed());
    EXPECT_EQ(failed, std::vector<exec_group *>{&bad});
    const auto info = bad.sched().get_process_info(bad_pid);
    ASSERT_TRUE(info);
    EXPECT_EQ(info->status, process_status::killed) << "threads " << threads;
    EXPECT_EQ(bad.sched().get_exit_error(bad_pid).value_or(""), "internal error");
    ASSERT_TRUE(finished(good, good_pid));
    EXPECT_EQ(result_of(good, good_pid), "42");
    // Failed groups are per call: a later pump_all without one reports none.
    std::vector<exec_group *> just_good{&good};
    EXPECT_TRUE(pool.pump_all(just_good, kSteps, far_deadline()).empty());
  }
}

TEST(ExecPool, OnCompleteOfAWorkerPumpedGroupRunsOnTheCallerAfterTheJoin) {
  cvc::pool_options po;
  po.threads = 2;
  cvc::exec_pool pool(po);
  if (pool.threads() == 0)
    GTEST_SKIP() << "no worker on this machine";
  cvc::app app;
  share_locks(app);
  exec_group a(opts_for(app, "a")), b(opts_for(app, "b"));
  std::vector<exec_group *> groups{&a, &b};
  std::mutex m;
  std::vector<std::pair<std::string, std::thread::id>> calls;
  bool returned = false; // pump_all has returned
  bool in_pump_all_seen = false;
  const auto record = [&](const std::string &what) {
    return [&, what](value_t) {
      std::lock_guard<std::mutex> lock(m);
      calls.emplace_back(what, std::this_thread::get_id());
      in_pump_all_seen = in_pump_all_seen || !returned;
      if (what == "a2")
        throw std::runtime_error("a2 throws");
    };
  };
  execute_options o1, o2, o3;
  o1.on_complete = record("a1");
  o2.on_complete = record("a2");
  o3.on_complete = record("b1");
  a.execute(std::string("1"), o1);
  a.execute(std::string("2"), o2);
  b.execute(std::string("3"), o3);
  // Both groups must go to the workers: the caller takes host groups only, and there is none,
  // so it takes one group itself; run until a worker pumped a group with callbacks.
  bool threw = false;
  try {
    pool.set_last_pump_for_test(a, 1.0);
    pool.set_last_pump_for_test(b, 1.0);
    pool.pump_all(groups, kSteps, far_deadline());
  } catch (const std::runtime_error &e) {
    threw = true;
    EXPECT_STREQ(e.what(), "a2 throws");
  }
  returned = true;
  EXPECT_TRUE(threw) << "the throw is rethrown from pump_all";
  ASSERT_EQ(calls.size(), 3u) << "the other callbacks still ran";
  std::vector<std::string> a_order;
  for (const auto &[what, id] : calls) {
    if (what[0] == 'a')
      a_order.push_back(what);
  }
  EXPECT_EQ(a_order, (std::vector<std::string>{"a1", "a2"})) << "completion order per group";
  for (const auto &[what, id] : calls)
    EXPECT_EQ(id, std::this_thread::get_id()) << what << " ran on the caller";
  EXPECT_TRUE(in_pump_all_seen) << "inline (caller-pumped) or after the join, inside pump_all";
}

TEST(ExecPool, RunOnHostRunsInlineOutsideAWorker) {
  cvc::exec_pool pool(cvc::pool_options{});
  cvc::app app;
  exec_group g(opts_for(app, "g"));
  bool ran = false;
  pool.run_on_host(g, [&] { ran = true; });
  EXPECT_TRUE(ran);
  EXPECT_EQ(cvc::detail::worker_pool(g), nullptr);
}

TEST(ExecGroup, InstallByKind) {
  cvc::pool_options po;
  po.threads = 2;
  cvc::exec_pool pool(po);
  cvc::app app;
  share_locks(app);
  exec_group any_group(opts_for(app, "any")),
      host_group(opts_for(app, "host", group_affinity::host));
  std::atomic<int> proxied_calls{0};
  std::mutex m;
  std::set<std::thread::id> proxied_threads;
  const native_fn proxied = [&](std::span<const value_t> args) -> value_t {
    ++proxied_calls;
    std::lock_guard<std::mutex> lock(m);
    proxied_threads.insert(std::this_thread::get_id());
    return value_t(std::get<int64_t>(args[0].v) * 2);
  };
  const native_fn only = [](std::span<const value_t>) -> value_t { return value_t(int64_t(7)); };
  environment_ptr any_env = environment::extend(builtins::make_default_environment());
  environment_ptr host_env = environment::extend(builtins::make_default_environment());
  for (auto [g, env] : {std::pair{&any_group, any_env}, std::pair{&host_group, host_env}}) {
    g->install(env, "proxied", proxied, provider_affinity::host_proxied);
    g->install(env, "only", only, provider_affinity::host_only);
  }
  EXPECT_EQ(any_env->lookup("only"), nullptr) << "host_only is not installed in any group";
  EXPECT_NE(host_env->lookup("only"), nullptr);
  execute_options ae, he;
  ae.env = any_env;
  he.env = host_env;
  const int a_name = any_group.execute(std::string("(only)"), ae);
  // Lists cross as copies, and a generator body blocks its worker instead of parking.
  const int a_proxy = any_group.execute(std::string("(proxied (car (list 21)))"), ae);
  const int a_gen =
      any_group.execute(std::string("(next (generator (lambda () (yield (proxied 4)))))"), ae);
  const int h_proxy = host_group.execute(std::string("(proxied 5)"), he);
  std::vector<exec_group *> groups{&any_group, &host_group};
  std::vector<std::pair<exec_group *, int>> pids{
      {&any_group, a_name}, {&any_group, a_proxy}, {&any_group, a_gen}, {&host_group, h_proxy}};
  for (int round = 0; round < 1000; ++round) {
    pool.set_last_pump_for_test(any_group, 1.0);
    pool.pump_all(groups, kSteps, far_deadline());
    bool done = true;
    for (const auto &[g, pid] : pids)
      done = done && finished(*g, pid);
    if (done)
      break;
  }
  for (const auto &[g, pid] : pids)
    ASSERT_TRUE(finished(*g, pid));
  EXPECT_NE(any_group.sched().get_exit_error(a_name).value_or(""), "") << "a NameError";
  EXPECT_EQ(result_of(any_group, a_proxy), "42");
  EXPECT_EQ(result_of(any_group, a_gen), "8");
  EXPECT_EQ(result_of(host_group, h_proxy), "10");
  EXPECT_EQ(proxied_calls.load(), 3);
  EXPECT_EQ(proxied_threads, std::set<std::thread::id>{std::this_thread::get_id()})
      << "a host_proxied body runs on the pump_all's caller";
}

TEST(ExecGroup, ExecuteCopiesASharedAst) {
  cvc::app app;
  exec_group a(opts_for(app, "a")), b(opts_for(app, "b"));
  const value_t ast =
      make_list({value_t(symbol{"begin"}),
                 make_list({value_t(symbol{"quote"}),
                            make_list({value_t(int64_t(1)), value_t(int64_t(2))})})});
  const value_t held = ast; // the host keeps it: shared, so each group gets a copy
  const int pa = a.execute(ast);
  const int pb = b.execute(ast);
  cvc::exec_pool pool(cvc::pool_options{});
  std::vector<exec_group *> groups{&a, &b};
  pool.pump_all(groups, kSteps, far_deadline());
  ASSERT_TRUE(finished(a, pa) && finished(b, pb));
  const value_t ra = *a.sched().get_result(pa), rb = *b.sched().get_result(pb);
  const list_ptr la = std::get<list_ptr>(ra.v), lb = std::get<list_ptr>(rb.v);
  const list_ptr literal =
      std::get<list_ptr>(std::get<list_ptr>(std::get<list_ptr>(held.v)->at(1).v)->at(1).v);
  EXPECT_NE(la.get(), literal.get()) << "group a got a copy";
  EXPECT_NE(lb.get(), literal.get()) << "group b got a copy";
  EXPECT_NE(la.get(), lb.get());
  EXPECT_EQ(to_string(ra), "(1 2)");
}

// A program that runs for long: a pump with a small step budget leaves it alive.
const char *const kLongLoop = "(begin (set i 0) (while (< i 100000000) (set i (+ i 1))) i)";

TEST(ExecGroup, WantsPumpAndTheLiveCount) {
  cvc::app app;
  EXPECT_EQ(detail::live_groups(app), 0u);
  auto g = std::make_unique<exec_group>(opts_for(app, "g"));
  EXPECT_EQ(detail::live_groups(app), 1u);
  EXPECT_FALSE(g->wants_pump()) << "no process, no ingress, no timer";
  const int pid = g->execute(std::string("(+ 1 2)"));
  EXPECT_TRUE(g->wants_pump()) << "a runnable process";
  g->pump(kSteps);
  ASSERT_TRUE(finished(*g, pid));
  EXPECT_FALSE(g->wants_pump());
  g->sched().post_message("box", value_t(int64_t(1)));
  EXPECT_TRUE(g->wants_pump()) << "pending ingress";
  g.reset();
  EXPECT_EQ(detail::live_groups(app), 0u);
}

// A group's creation order (pool_link().seq) and the live count are its app's own: two apps in
// both orders, apps on threads at once, and a later app (made after another is gone) counts from 1
// again. A group with no app has no creation order (0).
TEST(ExecGroup, CreationOrderAndLiveCountArePerApp) {
  for (const bool a_first : {true, false}) {
    cvc::app a, b;
    cvc::app &first = a_first ? a : b;
    cvc::app &second = a_first ? b : a;
    auto f1 = std::make_unique<exec_group>(opts_for(first, "f1"));
    auto s1 = std::make_unique<exec_group>(opts_for(second, "s1"));
    auto f2 = std::make_unique<exec_group>(opts_for(first, "f2"));
    EXPECT_EQ(f1->pool_link().seq, 1u) << a_first;
    EXPECT_EQ(s1->pool_link().seq, 1u) << a_first;
    EXPECT_EQ(f2->pool_link().seq, 2u) << a_first;
    EXPECT_EQ(detail::live_groups(first), 2u);
    EXPECT_EQ(detail::live_groups(second), 1u);
    f1.reset();
    EXPECT_EQ(detail::live_groups(first), 1u);
    EXPECT_EQ(detail::live_groups(second), 1u);
    f2.reset();
    s1.reset();
    EXPECT_EQ(detail::live_groups(first), 0u);
    EXPECT_EQ(detail::live_groups(second), 0u);
  }
  constexpr int kThreads = 4;
  std::vector<std::vector<std::uint64_t>> seqs(kThreads);
  std::vector<std::thread> threads;
  for (int t = 0; t < kThreads; ++t)
    threads.emplace_back([&seqs, t] {
      cvc::app app;
      std::vector<std::unique_ptr<exec_group>> groups;
      for (int i = 0; i < 3; ++i) {
        groups.push_back(std::make_unique<exec_group>(opts_for(app, "t" + std::to_string(i))));
        seqs[t].push_back(groups.back()->pool_link().seq);
      }
    });
  for (std::thread &th : threads)
    th.join();
  for (int t = 0; t < kThreads; ++t)
    EXPECT_EQ(seqs[t], (std::vector<std::uint64_t>{1, 2, 3})) << "app on thread " << t;
  {
    cvc::app gone;
    exec_group g1(opts_for(gone, "g1"));
    exec_group g2(opts_for(gone, "g2"));
  }
  cvc::app later;
  exec_group g(opts_for(later, "g"));
  EXPECT_EQ(g.pool_link().seq, 1u) << "a later app counts from 1";
  exec_group::options no_app;
  no_app.name = "no-app";
  exec_group orphan(std::move(no_app));
  EXPECT_EQ(orphan.pool_link().seq, 0u);
}

TEST(ExecGroup, TeardownRunsItsHookBeforeTheKills) {
  cvc::app app;
  int pid = -1;
  bool alive_in_hook = false;
  exec_group::options o = opts_for(app, "g");
  o.teardown_hook_for_test = [&](exec_group &g) {
    const auto info = g.sched().get_process_info(pid);
    alive_in_hook = info && !is_finished(info->status);
  };
  auto g = std::make_unique<exec_group>(o);
  pid = g->execute(std::string(kLongLoop));
  g->pump(pump_budget{10, std::numeric_limits<double>::infinity()});
  ASSERT_FALSE(finished(*g, pid));
  g.reset();
  EXPECT_TRUE(alive_in_hook) << "the hook runs before the kills (GL-2's order)";
}

TEST(ExecGroup, PumpBindsCurrentAndHonoursTheBudget) {
  cvc::app app;
  exec_group g(opts_for(app, "g"));
  exec_group *seen = nullptr;
  environment_ptr env = environment::extend(builtins::make_default_environment());
  bool pumping = false;
  builtins::register_fn(env, "seen", [&seen, &pumping, &g](call_args a) -> value_t {
    seen = exec_group::current(a.context());
    pumping = g.pumping_here();
    return nil_value();
  });
  execute_options eo;
  eo.env = env;
  g.execute(std::string("(begin (seen) (set i 0) (while (< i 1000) (set i (+ i 1))) i)"), eo);
  const pump_result first = g.pump(pump_budget{0, std::numeric_limits<double>::infinity()});
  EXPECT_EQ(first.steps, 0u) << "a 0-step budget only drains and wakes";
  const pump_result r = g.pump(pump_budget{10, std::numeric_limits<double>::infinity()});
  EXPECT_EQ(r.steps, 10u);
  EXPECT_FALSE(r.failed);
  EXPECT_EQ(seen, &g);
  EXPECT_TRUE(pumping);
  EXPECT_EQ(exec_group::current(&g.sched().evaluation_context()), nullptr);
  EXPECT_FALSE(g.pumping_here());
}

TEST(ExecPool, TwoCallersShareThePool) {
  cvc::pool_options po;
  po.threads = 2;
  cvc::exec_pool pool(po);
  cvc::app app;
  share_locks(app);
  std::vector<std::unique_ptr<exec_group>> owned;
  for (int i = 0; i < 4; ++i)
    owned.push_back(std::make_unique<exec_group>(opts_for(app, "g" + std::to_string(i))));
  std::vector<exec_group *> first{owned[0].get(), owned[1].get()},
      second{owned[2].get(), owned[3].get()};
  std::vector<int> pids;
  for (auto &g : owned)
    pids.push_back(
        g->execute(std::string("(begin (set i 0) (while (< i 3000) (set i (+ i 1))) i)")));
  const auto drive = [&](std::vector<exec_group *> &groups) {
    for (int round = 0; round < 2000; ++round) {
      for (exec_group *g : groups)
        pool.set_last_pump_for_test(*g, 1.0);
      pool.pump_all(groups, pump_budget{100, std::numeric_limits<double>::infinity()},
                    far_deadline());
    }
  };
  std::thread other([&] { drive(second); });
  drive(first);
  other.join();
  for (std::size_t i = 0; i < owned.size(); ++i) {
    ASSERT_TRUE(finished(*owned[i], pids[i]));
    EXPECT_EQ(result_of(*owned[i], pids[i]), "3000");
  }
}

TEST(AppExecPool, ThreadsBeforeTheLockManagerMakeItShared) {
  cvc::app app;
  cvc::pool_options po;
  po.threads = 1;
  cvc::exec_pool &pool = app.exec_pool(po);
  EXPECT_EQ(&pool, app.exec_pool_probe());
  EXPECT_EQ(&pool, &app.exec_pool()) << "built once";
  if (threads_compiled_in && std::thread::hardware_concurrency() > 1) {
    EXPECT_EQ(app.lock_manager()->threading(), lock_threading::shared);
    EXPECT_EQ(pool.threads(), 1u);
  }
  // Another app is not affected (the hint is this app's, not note_threaded_host's).
  cvc::app other;
  EXPECT_EQ(other.exec_pool_probe(), nullptr);
}

TEST(AppExecPool, ASingleLockManagerKeepsThePoolInline) {
  cvc::app app;
  app.root()("state_exec.locks.threading").value(std::string("single"));
  ASSERT_EQ(app.lock_manager()->threading(), lock_threading::single);
  cvc::pool_options po;
  po.threads = 2;
  EXPECT_EQ(app.exec_pool(po).threads(), 0u);
}

TEST(AppExecPool, TheTreeSizesThePool) {
  cvc::app app;
  app.root()("state_exec.threads").value(std::string("1"));
  app.root()("state_exec.cpu_set").value(std::string("all"));
  cvc::exec_pool &pool = app.exec_pool();
  EXPECT_EQ(pool.options().threads, 1u);
  EXPECT_EQ(pool.options().placement, cvc::cpu_set::all);
  EXPECT_EQ(pool.threads(),
            threads_compiled_in && std::thread::hardware_concurrency() > 1 ? 1u : 0u);
}

TEST(AppExecPool, PumpFramePumpsGroupZeroAndFrameGroupsAndAdvances) {
  cvc::app app;
  exec_group g(opts_for(app, "frame"));
  async_scheduler &zero = app.exec_scheduler();
  const int z = zero.execute(std::string("(+ 1 2)"));
  const int p = g.execute(std::string("(+ 3 4)"));
  const std::uint64_t gen = app.exec_pump_generation();
  {
    cvc::frame_group_handle handle = app.register_frame_group(g);
    app.exec_pump_frame();
  }
  EXPECT_EQ(app.exec_pump_generation(), gen + 1);
  ASSERT_TRUE(zero.get_result(z));
  EXPECT_EQ(to_string(*zero.get_result(z)), "3");
  ASSERT_TRUE(finished(g, p));
  EXPECT_EQ(result_of(g, p), "7");
  // Unregistered: the next frame does not pump it.
  const int q = g.execute(std::string("(+ 5 6)"));
  app.exec_pump_frame();
  EXPECT_FALSE(finished(g, q));
  EXPECT_EQ(app.exec_pump_generation(), gen + 2);
}

TEST(AppExecPool, GroupsAndTheAppSchedulerShareOnePidSource) {
  cvc::app app;
  exec_group a(opts_for(app, "a")), b(opts_for(app, "b"));
  std::set<int> seen;
  std::vector<int> in_a;
  for (int i = 0; i < 5; ++i) {
    in_a.push_back(a.execute(std::string("1")));
    seen.insert(in_a.back());
    seen.insert(b.execute(std::string("1")));
    seen.insert(app.exec_scheduler().execute(std::string("1")));
  }
  EXPECT_EQ(app.exec_pid_source().get(), app.exec_pid_source().get());
  EXPECT_EQ(seen.size(), 15u) << "unique app-wide";
  for (std::size_t i = 1; i < in_a.size(); ++i)
    EXPECT_LT(in_a[i - 1], in_a[i]) << "increasing within a group";
  EXPECT_EQ(app.exec_pid_source()->load(), 16) << "the next pid";
}

// --- Stage 2a continuous mode: the pool's own (P; S2a.2, S2a.4) ---
//
// Each case detaches what it attached before its groups go (the teardown of an attached group is
// S2a.5's). The host loop is K's: pump_host, else wait_for_work, on the attaching thread.

namespace {

// Drive `pool` on this (the attaching) thread until `done()`, or false after `limit` seconds (a
// hang becomes a failure, not a wall-clock assertion).
bool drive(cvc::exec_pool &pool, const std::function<bool()> &done, double limit = 30.0) {
  const auto t0 = clock_type::now();
  while (!done()) {
    if (clock_type::now() - t0 > std::chrono::duration<double>(limit))
      return false;
    if (!pool.pump_host(kInf))
      pool.wait_for_work(0.002);
  }
  return true;
}

// Whether `pid` of the attached group `g` finished, read under its token.
bool finished_held(cvc::exec_pool &pool, exec_group &g, int pid) {
  bool f = false;
  pool.with_group(g, [&] { f = finished(g, pid); });
  return f;
}

// Poll `cond` with plain sleeps (servicing nothing) for up to `limit` seconds.
bool settles(const std::function<bool()> &cond, double limit = 10.0) {
  const auto t0 = clock_type::now();
  while (!cond()) {
    if (clock_type::now() - t0 > std::chrono::duration<double>(limit))
      return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return true;
}

// CVC_EXEC_FUZZ for the pools built while it lives.
struct fuzz_scope {
  explicit fuzz_scope(const char *seed) { set(seed); }
  ~fuzz_scope() { set(nullptr); }
  static void set(const char *v) {
#ifdef _WIN32
    _putenv_s("CVC_EXEC_FUZZ", v ? v : "");
#else
    if (v)
      setenv("CVC_EXEC_FUZZ", v, 1);
    else
      unsetenv("CVC_EXEC_FUZZ");
#endif
  }
};

// A group whose pump hook records which group was pumped, in order, on a shared list.
struct pump_order {
  std::mutex m;
  std::vector<std::string> names;
  std::vector<std::thread::id> threads;
  void add(const exec_group &g) {
    std::lock_guard<std::mutex> lock(m);
    names.push_back(g.opts().name);
    threads.push_back(std::this_thread::get_id());
  }
  std::size_t size() {
    std::lock_guard<std::mutex> lock(m);
    return names.size();
  }
};

exec_group::options logged_opts(cvc::app &app, const std::string &name, pump_order &order) {
  exec_group::options o = opts_for(app, name);
  o.pump_hook_for_test = [&order](exec_group &g) { order.add(g); };
  return o;
}

// A program that counts to n, then calls `fin` (which sets `flag`): with workers nothing of the
// host's drives an attached `any` group, so the case watches the flag and reads the result after
// the detach.
std::string counting_then_fin(int n) {
  return "(begin (set i 0) (while (< i " + std::to_string(n) + ") (set i (+ i 1))) (fin) i)";
}

environment_ptr env_with_fin(std::atomic<bool> &flag) {
  environment_ptr env = environment::extend(builtins::make_default_environment());
  builtins::register_fn(env, "fin", [&flag](std::span<const value_t>) -> value_t {
    flag = true;
    return nil_value();
  });
  return env;
}

} // namespace

// attach refuses a group attached already, to this pool or another; detach and attached() of a
// group not attached do nothing; attach installs the notifier, detach clears it.
TEST(ExecPoolContinuous, AttachIsRefusedTwice) {
  cvc::exec_pool pool(cvc::pool_options{}), other(cvc::pool_options{});
  cvc::app app;
  exec_group g(opts_for(app, "twice"));
  EXPECT_FALSE(pool.attached(g));
  pool.detach(g); // nothing
  pool.attach(g);
  EXPECT_TRUE(pool.attached(g));
  EXPECT_FALSE(other.attached(g));
  EXPECT_EQ(pool.stats().attached, 1u);
  EXPECT_THROW(pool.attach(g), std::logic_error);
  EXPECT_THROW(other.attach(g), std::logic_error);
  other.detach(g); // not its group: nothing
  EXPECT_TRUE(pool.attached(g));
  EXPECT_TRUE(g.sched().has_wake_notifier());
  pool.detach(g);
  EXPECT_FALSE(pool.attached(g));
  EXPECT_FALSE(g.sched().has_wake_notifier());
  EXPECT_EQ(pool.stats().attached, 0u);
  EXPECT_EQ(g.pool_link().state.load(), 0u);
}

// Attached groups run on the workers to their results, with no host loop: attach wakes each once,
// and the release queues a group again while its pump says it is runnable (a small pump_steps
// splits each run over many pumps), on the workers only.
TEST(ExecPoolContinuous, AttachedGroupsRunOnTheWorkers) {
  cvc::pool_options po;
  po.threads = 2;
  po.pump_steps = 200;
  cvc::exec_pool pool(po);
  if (pool.threads() == 0)
    GTEST_SKIP() << "no worker on this machine";
  cvc::app app;
  share_locks(app);
  pump_order order;
  std::atomic<bool> fa{false}, fb{false};
  exec_group a(logged_opts(app, "a", order)), b(logged_opts(app, "b", order));
  execute_options ea, eb;
  ea.env = env_with_fin(fa);
  eb.env = env_with_fin(fb);
  const int pa = a.execute(counting_then_fin(2000), ea);
  const int pb = b.execute(counting_then_fin(3000), eb);
  pool.attach(a);
  pool.attach(b);
  const bool done = settles([&] { return fa.load() && fb.load(); }, 30.0);
  pool.detach(a);
  pool.detach(b);
  ASSERT_TRUE(done);
  EXPECT_EQ(result_of(a, pa), "2000");
  EXPECT_EQ(result_of(b, pb), "3000");
  const cvc::pool_stats s = pool.stats();
  std::cout << s.pumps << " pumps, " << s.requeues << " requeues, " << s.wakes << " wakes, "
            << s.parks << " parks, " << s.moves << " moves\n";
  EXPECT_GT(s.pumps, 10u) << "the budget did not split the runs";
  EXPECT_EQ(s.worker_pumps, s.pumps);
  EXPECT_EQ(s.host_pumps, 0u);
  EXPECT_GE(s.requeues, 10u);
  EXPECT_GE(s.wakes, 2u) << "attach wakes each group once";
  for (const std::thread::id &t : order.threads)
    EXPECT_NE(t, std::this_thread::get_id()) << "an any group was pumped on its attaching thread";
}

// CVC_EXEC_FORCE_MOVE in continuous mode (pool_options::force_move): each pump of a group is on
// another worker than its last (stats().moves counts them).
TEST(ExecPoolContinuous, ForceMoveGivesEachPumpAnotherWorker) {
  cvc::pool_options po;
  po.threads = 2;
  po.force_move = true;
  po.pump_steps = 200;
  cvc::exec_pool pool(po);
  if (pool.threads() < 2)
    GTEST_SKIP() << "needs two workers";
  cvc::app app;
  share_locks(app);
  pump_order order;
  std::atomic<bool> fin{false};
  exec_group g(logged_opts(app, "moving", order));
  execute_options eo;
  eo.env = env_with_fin(fin);
  const int pid = g.execute(counting_then_fin(2000), eo);
  pool.attach(g);
  const bool done = settles([&] { return fin.load(); }, 30.0);
  pool.detach(g);
  ASSERT_TRUE(done);
  EXPECT_EQ(result_of(g, pid), "2000");
  const cvc::pool_stats s = pool.stats();
  ASSERT_GE(order.threads.size(), 4u);
  for (std::size_t i = 1; i < order.threads.size(); ++i)
    EXPECT_NE(order.threads[i], order.threads[i - 1]) << "pump " << i << " stayed on its thread";
  EXPECT_EQ(s.moves, s.pumps - 1);
}

// S2-Q13's clamp: a group attached after another has run for a while starts at the ready queue's
// floor less one pump, not at virtual time 0, so on one worker the two alternate rather than the
// newcomer taking every pump until its time catches up.
TEST(ExecPoolContinuous, ANewGroupDoesNotStarveABusyOne) {
  cvc::pool_options po;
  po.threads = 1;
  cvc::exec_pool pool(po);
  if (pool.threads() == 0)
    GTEST_SKIP() << "no worker on this machine";
  cvc::app app;
  share_locks(app);
  pump_order order;
  exec_group busy(logged_opts(app, "busy", order)), late(logged_opts(app, "late", order));
  busy.execute(std::string("(while #t nil)"));
  late.execute(std::string("(while #t nil)"));
  pool.attach(busy);
  ASSERT_TRUE(settles([&] { return order.size() >= 50; }, 30.0));
  std::size_t from = 0;
  {
    std::lock_guard<std::mutex> lock(order.m);
    from = order.names.size();
  }
  pool.attach(late);
  const bool done = settles([&] { return order.size() >= from + 40; }, 30.0);
  pool.detach(busy);
  pool.detach(late);
  ASSERT_TRUE(done);
  std::size_t busy_pumps = 0, late_pumps = 0;
  for (std::size_t i = from; i < from + 40; ++i)
    ++(order.names[i] == "busy" ? busy_pumps : late_pumps);
  std::cout << "after the late attach: busy " << busy_pumps << ", late " << late_pumps << "\n";
  // Without the clamp the late group takes all 40 (the busy one is ~50 pumps ahead). The bounds
  // leave room for a pump the OS stretched (its time is its virtual time).
  EXPECT_GE(busy_pumps, 4u) << "the late group took the worker";
  EXPECT_GE(late_pumps, 4u);
}

// The eventcount and CVC_EXEC_FUZZ: idle workers park (stats().parks), none is active once the
// work is done, and the pool's fuzz points are reached and counted.
TEST(ExecPoolContinuous, IdleWorkersParkAndFuzzPointsAreCounted) {
  std::unique_ptr<cvc::exec_pool> pool;
  {
    fuzz_scope fuzz("1");
    cvc::pool_options po;
    po.threads = 2;
    pool = std::make_unique<cvc::exec_pool>(po);
  }
  if (pool->threads() == 0)
    GTEST_SKIP() << "no worker on this machine";
  cvc::app app;
  share_locks(app);
  exec_group g(opts_for(app, "idle"));
  std::atomic<bool> fin{false};
  execute_options eo;
  eo.env = env_with_fin(fin);
  g.execute(counting_then_fin(100), eo);
  pool->attach(g);
  const bool done = settles([&] { return fin.load(); });
  const bool parked = settles([&] {
    const cvc::pool_stats s = pool->stats();
    return s.parks > 0 && s.active_workers == 0;
  });
  const cvc::pool_stats s = pool->stats();
  pool->detach(g);
  ASSERT_TRUE(done);
  std::cout << "parks " << s.parks << ", empty wakes " << s.empty_wakes << ", fuzz points "
            << s.fuzz_points << "\n";
  EXPECT_TRUE(parked) << "the workers never parked";
  EXPECT_EQ(s.active_workers, 0u);
  EXPECT_GT(s.fuzz_points, 0u);
}

// --- the host side (S2a.4) ---

// Attached groups run to their results in continuous mode: at threads 0 every pump on the
// attaching thread (host_pumps), with workers every pump on a worker; a small pump_steps splits
// each run over many pumps, which the release queues again while its pump says runnable.
TEST(ExecPoolContinuous, AttachedGroupsRunToTheirResults) {
  for (unsigned threads : {0u, 2u}) {
    cvc::pool_options po;
    po.threads = threads;
    po.pump_steps = 200;
    cvc::exec_pool pool(po);
    cvc::app app;
    share_locks(app);
    thread_log la, lb;
    exec_group a(opts_for(app, "a")), b(opts_for(app, "b"));
    execute_options ea, eb;
    ea.env = env_with_tid(la);
    eb.env = env_with_tid(lb);
    const int pa = a.execute(counting_noting_each_pass(1000), ea);
    const int pb = b.execute(counting(3000), eb);
    const cvc::pool_stats before = pool.stats();
    pool.attach(a);
    pool.attach(b);
    EXPECT_TRUE(pool.attached(a));
    EXPECT_EQ(pool.stats().attached, 2u);
    const bool done =
        drive(pool, [&] { return finished_held(pool, a, pa) && finished_held(pool, b, pb); });
    pool.detach(a);
    pool.detach(b);
    ASSERT_TRUE(done) << "threads " << threads;
    EXPECT_EQ(result_of(a, pa), "1000");
    EXPECT_EQ(result_of(b, pb), "3000");
    const cvc::pool_stats s = pool.stats();
    EXPECT_EQ(s.attached, 0u);
    EXPECT_FALSE(pool.attached(a));
    const std::uint64_t pumps = s.pumps - before.pumps;
    std::cout << "threads " << pool.threads() << ": " << pumps << " pumps, " << s.requeues
              << " requeues, " << s.wakes << " wakes\n";
    EXPECT_GT(pumps, 10u) << "the budget did not split the runs";
    EXPECT_GE(s.requeues, 10u);
    const std::thread::id me = std::this_thread::get_id();
    if (pool.threads() == 0) {
      EXPECT_EQ(s.host_pumps - before.host_pumps, pumps);
      EXPECT_EQ(s.worker_pumps, before.worker_pumps);
      EXPECT_EQ(la.distinct(), std::set<std::thread::id>{me});
    } else {
      EXPECT_EQ(s.host_pumps, before.host_pumps);
      EXPECT_EQ(s.worker_pumps - before.worker_pumps, pumps);
      EXPECT_EQ(la.distinct().count(me), 0u) << "an any group was pumped on its attaching thread";
    }
  }
}

// A post to an idle attached group calls its notifier, wake(g), which queues it: the receiver runs
// with nothing else driving the group (its result comes back through on_complete, so no with_group
// probe wakes it instead).
TEST(ExecPoolContinuous, APostWakesAnIdleGroup) {
  for (unsigned threads : {0u, 2u}) {
    cvc::pool_options po;
    po.threads = threads;
    cvc::exec_pool pool(po);
    cvc::app app;
    share_locks(app);
    intrinsic_world w(app, "box");
    std::atomic<bool> parked{false}, got{false};
    std::string result;
    builtins::register_fn(w.env, "mark", [&](std::span<const value_t>) -> value_t {
      parked = true;
      return nil_value();
    });
    w.submit("(begin (mark) (msg-recv \"in\"))", [&](value_t v) {
      result = to_string(v);
      got = true;
    });
    pool.attach(*w.group);
    ASSERT_TRUE(drive(pool, [&] { return parked.load(); }));
    pool.wait_for_work(0.02); // its pump ends parked on the receive: idle
    const cvc::pool_stats before = pool.stats();
    w.group->sched().post_message(resolve_channel_key("box", "in"), value_t(int64_t(7)));
    const bool done = drive(pool, [&] { return got.load(); }, 10.0);
    const cvc::pool_stats after = pool.stats();
    pool.detach(*w.group);
    ASSERT_TRUE(done) << "threads " << threads << ": the post did not wake the group";
    EXPECT_EQ(result, "7");
    EXPECT_GE(after.wakes - before.wakes, 1u);
  }
}

// with_group keeps workers off an attached group: a post during the hold wakes it, a worker that
// pops it leaves it notified (no pump while held: a pump would need the token), and it is pumped
// after the hold ends, so the receiver runs (S2-Q4).
TEST(ExecPoolContinuous, AGroupPostedDuringWithGroupIsPumpedAfterIt) {
  cvc::pool_options po;
  po.threads = 1;
  cvc::exec_pool pool(po);
  if (pool.threads() == 0)
    GTEST_SKIP() << "no worker on this machine";
  cvc::app app;
  share_locks(app);
  std::atomic<int> pumps{0};
  intrinsic_world w(app, "held");
  std::atomic<bool> parked{false}, got{false};
  builtins::register_fn(w.env, "mark", [&](std::span<const value_t>) -> value_t {
    parked = true;
    return nil_value();
  });
  w.submit("(begin (mark) (msg-recv \"in\") 1)", [&](value_t) { got = true; });
  pool.attach(*w.group);
  ASSERT_TRUE(drive(pool, [&] { return parked.load(); }));
  pool.wait_for_work(0.02);
  const cvc::pool_stats before = pool.stats();
  int pumped_in_hold = -1;
  bool here = true;
  pool.with_group(*w.group, [&] {
    here = w.group->pumping_here();
    const std::uint64_t p0 = pool.stats().pumps;
    w.group->sched().post_message(resolve_channel_key("held", "in"), value_t(int64_t(1)));
    std::this_thread::sleep_for(std::chrono::milliseconds(50)); // a worker pops it meanwhile
    pumped_in_hold = static_cast<int>(pool.stats().pumps - p0);
  });
  const bool done = drive(pool, [&] { return got.load(); }, 10.0);
  const cvc::pool_stats after = pool.stats();
  pool.detach(*w.group);
  EXPECT_FALSE(here) << "with_group's fn ran inside a pump";
  EXPECT_EQ(pumped_in_hold, 0) << "the group was pumped while with_group held it";
  ASSERT_TRUE(done) << "the group posted during the hold was never pumped";
  std::cout << "requeues across the hold: " << after.requeues - before.requeues << "\n";
}

// on_complete of a worker-pumped attached group runs on its attaching thread, outside every pump
// (S2-Q5: the host queue), in wait_for_work; the pump ran on a worker.
TEST(ExecPoolContinuous, OnCompleteRunsOnTheAttachingThread) {
  cvc::pool_options po;
  po.threads = 2;
  cvc::exec_pool pool(po);
  if (pool.threads() == 0)
    GTEST_SKIP() << "no worker on this machine";
  cvc::app app;
  share_locks(app);
  std::atomic<std::thread::id> pumped_on{};
  exec_group::options o = opts_for(app, "cb");
  o.pump_hook_for_test = [&](exec_group &) { pumped_on = std::this_thread::get_id(); };
  exec_group g(o);
  std::atomic<bool> ran{false};
  std::thread::id ran_on;
  bool inside = true;
  execute_options eo;
  eo.on_complete = [&](value_t) {
    ran_on = std::this_thread::get_id();
    inside = g.pumping_here();
    ran = true;
  };
  g.execute(std::string("(+ 40 2)"), eo);
  pool.attach(g);
  const bool done = drive(pool, [&] { return ran.load(); });
  pool.detach(g);
  ASSERT_TRUE(done);
  EXPECT_EQ(ran_on, std::this_thread::get_id());
  EXPECT_FALSE(inside) << "the callback ran inside a pump";
  EXPECT_NE(pumped_on.load(), std::this_thread::get_id());
}

// A non-waiting host_proxied body queued by a worker runs on the attaching thread under the
// group's token (no other thread owns it then), and its value resumes the process at a later
// pump, woken by the completion's post (S2-Q5).
TEST(ExecPoolContinuous, AHostProxiedBodyRunsOnTheAttachingThreadUnderTheToken) {
  cvc::pool_options po;
  po.threads = 2;
  cvc::exec_pool pool(po);
  if (pool.threads() == 0)
    GTEST_SKIP() << "no worker on this machine";
  cvc::app app;
  share_locks(app);
  exec_group g(opts_for(app, "proxied"));
  environment_ptr env = environment::extend(builtins::make_default_environment());
  std::thread::id body_on;
  bool body_inside = true, elsewhere = true;
  g.install(
      env, "hp",
      [&](call_args) -> value_t {
        body_on = std::this_thread::get_id();
        body_inside = g.pumping_here();
        elsewhere = g.sched().pumped_elsewhere();
        return value_t(int64_t(5));
      },
      provider_affinity::host_proxied);
  std::atomic<bool> got{false};
  std::string result;
  execute_options eo;
  eo.env = environment::extend(env);
  eo.on_complete = [&](value_t v) {
    result = to_string(v);
    got = true;
  };
  g.execute(std::string("(+ (hp) 1)"), eo);
  pool.attach(g);
  const bool done = drive(pool, [&] { return got.load(); }, 10.0);
  pool.detach(g);
  ASSERT_TRUE(done);
  EXPECT_EQ(result, "6");
  EXPECT_EQ(body_on, std::this_thread::get_id()) << "the body ran off its attaching thread";
  EXPECT_FALSE(body_inside);
  EXPECT_FALSE(elsewhere) << "the body ran while another thread held the group's token";
}

// A pump that a C++ exception escapes fails its attached group: its processes are killed with
// "internal error", and take_failed on its attaching thread returns it once, detached.
TEST(ExecPoolContinuous, AFailedGroupIsTakenOnItsAttachingThread) {
  for (unsigned threads : {0u, 2u}) {
    cvc::pool_options po;
    po.threads = threads;
    cvc::exec_pool pool(po);
    cvc::app app;
    share_locks(app);
    exec_group::options bad_opts = opts_for(app, "bad");
    bad_opts.pump_hook_for_test = [](exec_group &) { throw std::runtime_error("boom"); };
    exec_group bad(bad_opts);
    const int pid = bad.execute(std::string("(begin (sleep 10) 1)"));
    pool.attach(bad);
    std::vector<exec_group *> taken;
    const bool done = drive(pool, [&] {
      for (exec_group *g : pool.take_failed())
        taken.push_back(g);
      return !taken.empty();
    });
    if (!done)
      pool.detach(bad);
    ASSERT_TRUE(done) << "threads " << threads;
    EXPECT_EQ(taken, std::vector<exec_group *>{&bad});
    EXPECT_FALSE(pool.attached(bad));
    EXPECT_EQ(pool.stats().attached, 0u);
    EXPECT_TRUE(pool.take_failed().empty()) << "returned twice";
    EXPECT_TRUE(bad.failed());
    const auto info = bad.sched().get_process_info(pid);
    ASSERT_TRUE(info);
    EXPECT_EQ(info->status, process_status::killed);
    EXPECT_EQ(bad.sched().get_exit_error(pid).value_or(""), "internal error");
  }
}

// Deterministic mode pumps its attached groups on the attaching thread, in a rotation by creation
// order (td 4.4), whatever order they were attached in, on steps only.
TEST(ExecPoolContinuous, DeterministicModeRotatesInCreationOrder) {
  cvc::pool_options po;
  po.threads = 2;
  po.deterministic = true;
  po.pump_steps = 100;
  cvc::exec_pool pool(po);
  cvc::app app;
  share_locks(app);
  pump_order order;
  exec_group g1(logged_opts(app, "g1", order)), g2(logged_opts(app, "g2", order)),
      g3(logged_opts(app, "g3", order));
  const std::string loop = "(begin (set i 0) (while (< i 300) (set i (+ i 1))) i)";
  const int p1 = g1.execute(loop), p2 = g2.execute(loop), p3 = g3.execute(loop);
  pool.attach(g3);
  pool.attach(g1);
  pool.attach(g2);
  const bool done = drive(pool, [&] {
    return finished_held(pool, g1, p1) && finished_held(pool, g2, p2) &&
           finished_held(pool, g3, p3);
  });
  pool.detach(g1);
  pool.detach(g2);
  pool.detach(g3);
  ASSERT_TRUE(done);
  for (const auto &[g, pid] : {std::pair{&g1, p1}, std::pair{&g2, p2}, std::pair{&g3, p3}})
    EXPECT_EQ(result_of(*g, pid), "300");
  ASSERT_GE(order.names.size(), 6u);
  const std::vector<std::string> names{"g1", "g2", "g3"};
  for (std::size_t i = 0; i < order.names.size(); ++i) {
    EXPECT_EQ(order.names[i], names[i % 3]) << "pump " << i;
    EXPECT_EQ(order.threads[i], std::this_thread::get_id()) << "pump " << i;
  }
}

// detach runs the group's queued host entries on the calling thread before it returns (S2-Q4's
// step 5): an on_complete a worker deferred, never serviced before the detach.
TEST(ExecPoolContinuous, DetachRunsTheGroupsQueuedEntries) {
  cvc::pool_options po;
  po.threads = 2;
  cvc::exec_pool pool(po);
  if (pool.threads() == 0)
    GTEST_SKIP() << "no worker on this machine";
  cvc::app app;
  share_locks(app);
  exec_group g(opts_for(app, "queued"));
  std::atomic<int> ran{0};
  execute_options eo;
  eo.on_complete = [&](value_t) { ++ran; };
  g.execute(std::string("(+ 1 2)"), eo);
  const std::uint64_t pumps = pool.stats().pumps;
  pool.attach(g); // its first pump ends the process: the callback waits on this thread's queue
  ASSERT_TRUE(settles([&] { return pool.stats().pumps > pumps; }));
  const int before = ran.load();
  pool.detach(g);
  EXPECT_EQ(before, 0) << "the callback ran before this thread serviced its queue";
  EXPECT_EQ(ran.load(), 1) << "detach did not run the queued callback";
}

// The teardown's steps (detail::detach_begin, then the token, then detach_end): the group is no
// longer attached, and its queued entries are dropped unrun (its kills settle what they were for).
TEST(ExecPoolContinuous, DetachBeginAndEndDropTheGroupsEntries) {
  cvc::pool_options po;
  po.threads = 2;
  cvc::exec_pool pool(po);
  if (pool.threads() == 0)
    GTEST_SKIP() << "no worker on this machine";
  cvc::app app;
  share_locks(app);
  exec_group g(opts_for(app, "torn"));
  std::atomic<int> ran{0};
  execute_options eo;
  eo.on_complete = [&](value_t) { ++ran; };
  g.execute(std::string("(+ 1 2)"), eo);
  const std::uint64_t pumps = pool.stats().pumps;
  pool.attach(g);
  ASSERT_TRUE(settles([&] { return pool.stats().pumps > pumps; }));
  exec_group lone(opts_for(app, "lone"));
  EXPECT_FALSE(cvc::detail::detach_begin(lone)) << "a group not attached";
  ASSERT_TRUE(cvc::detail::detach_begin(g));
  {
    async_scheduler::pump_owner_scope token(g.sched());
    cvc::detail::detach_end(g);
  }
  EXPECT_FALSE(pool.attached(g));
  EXPECT_EQ(pool.stats().attached, 0u);
  EXPECT_EQ(ran.load(), 0) << "a dropped entry ran";
  EXPECT_FALSE(g.sched().has_wake_notifier());
  EXPECT_EQ(g.pool_link().state.load(), 0u);
}

// The amended host transition (PL-1's model): with_group's release wakes the group when no pop was
// deferred, since a submit under the token posts nothing, so a program submitted there to an idle
// attached group runs; inside fn this thread holds the group's host hold (detail::host_held_here,
// what the teardown's abort reads), outside it does not.
TEST(ExecPoolContinuous, ASubmitInsideWithGroupIsPumped) {
  cvc::pool_options po;
  po.threads = 2;
  cvc::exec_pool pool(po);
  if (pool.threads() == 0)
    GTEST_SKIP() << "no worker on this machine";
  cvc::app app;
  share_locks(app);
  exec_group g(opts_for(app, "submit"));
  pool.attach(g);
  pool.wait_for_work(0.02); // its first pump, of nothing: idle
  std::atomic<bool> fin{false};
  bool held_inside = false;
  int pid = 0;
  pool.with_group(g, [&] {
    held_inside = cvc::detail::host_held_here(g);
    execute_options eo;
    eo.env = env_with_fin(fin);
    pid = g.execute(counting_then_fin(10), eo);
  });
  const bool held_after = cvc::detail::host_held_here(g);
  const bool done = settles([&] { return fin.load(); });
  pool.detach(g);
  EXPECT_TRUE(held_inside);
  EXPECT_FALSE(held_after);
  ASSERT_TRUE(done) << "the program submitted under with_group was never pumped";
  EXPECT_EQ(result_of(g, pid), "10");
}

// fn may detach the group: with_group's release then leaves the pool's record alone, and the group
// is an unattached one afterwards (with_group takes its token as for any other).
TEST(ExecPoolContinuous, WithGroupMayDetachTheGroup) {
  cvc::pool_options po;
  po.threads = 2;
  cvc::exec_pool pool(po);
  cvc::app app;
  share_locks(app);
  exec_group g(opts_for(app, "detached"));
  pool.attach(g);
  pool.with_group(g, [&] { pool.detach(g); });
  EXPECT_FALSE(pool.attached(g));
  EXPECT_EQ(pool.stats().attached, 0u);
  EXPECT_EQ(g.pool_link().state.load(), 0u);
  bool ran = false;
  pool.with_group(g, [&] { ran = !g.pumping_here(); });
  EXPECT_TRUE(ran);
}

// A group being detached is taken as an unattached one by with_group on the thread finishing its
// detach (the amended transition): a failed group, before take_failed, is read at once on its
// attaching thread, with no hold set and no wait.
TEST(ExecPoolContinuous, AFailedGroupIsReadBeforeTakeFailed) {
  cvc::pool_options po;
  po.threads = 2;
  cvc::exec_pool pool(po);
  if (pool.threads() == 0)
    GTEST_SKIP() << "no worker on this machine";
  cvc::app app;
  share_locks(app);
  exec_group::options bad_opts = opts_for(app, "bad");
  bad_opts.pump_hook_for_test = [](exec_group &) { throw std::runtime_error("boom"); };
  exec_group bad(bad_opts);
  const int pid = bad.execute(std::string("(begin (sleep 10) 1)"));
  pool.attach(bad);
  ASSERT_TRUE(settles([&] { return bad.failed(); }));
  process_status status = process_status::running;
  bool held = true;
  pool.with_group(bad, [&] {
    held = cvc::detail::host_held_here(bad);
    status = bad.sched().get_process_info(pid)->status;
  });
  EXPECT_FALSE(held) << "a hold was set on a group being detached";
  EXPECT_EQ(status, process_status::killed);
  std::vector<exec_group *> taken;
  ASSERT_TRUE(drive(pool, [&] {
    for (exec_group *g : pool.take_failed())
      taken.push_back(g);
    return !taken.empty();
  }));
  EXPECT_EQ(taken, std::vector<exec_group *>{&bad});
  EXPECT_FALSE(pool.attached(bad));
}

// A worker that fires a due timer at its pop pumps that group itself: it takes no parked worker for
// it. Taking one (off the parked list, owed a notify) without notifying it left that worker in an
// untimed wait for good, out of the eventcount's reach, so ~exec_pool's join hung. Three workers
// start (a short group, a busy one, a frame waiter); the busy one's tiny pumps keep its worker
// popping, so it fires the waiter's coarse deadlines while another worker is parked without one.
// The pool then goes on a thread of its own and must join (a 30 s ceiling turns a hang into a
// failure, leaking the pool).
TEST(ExecPoolContinuous, AWorkerFiringATimerTakesNoParkedWorker) {
  auto pool = std::make_unique<cvc::exec_pool>([] {
    cvc::pool_options po;
    po.threads = 3;
    po.pump_steps = 50;
    return po;
  }());
  if (pool->threads() < 3)
    GTEST_SKIP() << "needs three workers";
  cvc::app app;
  share_locks(app);
  exec_group extra(opts_for(app, "extra"));
  extra.execute(std::string("(+ 1 2)"));
  pool->attach(extra);
  exec_group busy(opts_for(app, "busy"));
  intrinsic_world sleeper(app, "sleeper");
  std::atomic<int> ticks{0};
  builtins::register_fn(sleeper.env, "tick", [&](std::span<const value_t>) -> value_t {
    ++ticks;
    return nil_value();
  });
  busy.execute(std::string("(while #t nil)"));
  sleeper.submit("(while #t (begin (tick) (await 1)))"); // a frame waiter: a coarse 1 ms deadline
  pool->attach(busy);
  pool->attach(*sleeper.group);
  const bool ran = settles([&] { return ticks.load() >= 1000; }, 30.0);
  pool->detach(busy);
  pool->detach(*sleeper.group);
  pool->detach(extra);
  const cvc::pool_stats s = pool->stats();
  std::cout << "sleeper ticks " << ticks.load() << ", timer wakes " << s.timer_wakes << ", parks "
            << s.parks << "\n";
  ASSERT_TRUE(ran);
  auto joined = std::make_shared<std::atomic<bool>>(false);
  cvc::exec_pool *raw = pool.release();
  std::thread([raw, joined] {
    delete raw;
    joined->store(true);
  }).detach();
  EXPECT_TRUE(settles([&] { return joined->load(); }, 30.0))
      << "~exec_pool did not join its workers: one was left waiting un-notified";
}

// A with_group waiting for a group its worker keeps re-pumping (always runnable) gets it: the
// release that queues it again sets the hold for the waiting host (as the host's own look would),
// where before the worker popped it again in the same critical section and the host never found
// it free (CT-3's with_group waited minutes). The pop that follows leaves it notified, and it is
// pumped again after the hold. A 10 s ceiling turns starvation into a failure (the case then leaks
// what the stuck thread uses).
TEST(ExecPoolContinuous, WithGroupGetsAGroupItsWorkerKeepsRepumping) {
  auto *pool = new cvc::exec_pool([] {
    cvc::pool_options po;
    po.threads = 1;
    return po;
  }());
  if (pool->threads() == 0) {
    delete pool;
    GTEST_SKIP() << "no worker on this machine";
  }
  auto *app = new cvc::app;
  share_locks(*app);
  std::atomic<std::uint64_t> pumps{0};
  exec_group::options o = opts_for(*app, "busy");
  o.pump_hook_for_test = [&pumps](exec_group &) { pumps.fetch_add(1); };
  auto *g = new exec_group(o);
  g->execute(std::string("(while #t nil)"));
  pool->attach(*g);
  ASSERT_TRUE(settles([&] { return pumps.load() >= 20; }));
  auto done = std::make_shared<std::atomic<bool>>(false);
  auto notified = std::make_shared<std::atomic<bool>>(false);
  std::thread holder([pool, g, done, notified] {
    pool->with_group(*g, [&] {
      const auto t0 = clock_type::now();
      while ((g->pool_link().state.load() & detail::group_pool_link::notified) == 0 &&
             clock_type::now() - t0 < std::chrono::seconds(5))
        std::this_thread::yield();
      notified->store((g->pool_link().state.load() & detail::group_pool_link::notified) != 0);
    });
    done->store(true);
  });
  if (!settles([&] { return done->load(); })) {
    holder.detach(); // starved: everything it uses stays leaked
    FAIL() << "with_group never got the group its worker kept re-pumping";
  }
  holder.join();
  const std::uint64_t at_release = pumps.load();
  const bool pumped_after = settles([&] { return pumps.load() > at_release; });
  pool->detach(*g);
  EXPECT_TRUE(notified->load()) << "the worker's pop under the hold did not leave it notified";
  EXPECT_TRUE(pumped_after) << "not pumped after the hold";
  delete g;
  delete pool;
  delete app;
}

// --- the R2 reviews' probes, adopted (R2a C-M1..C-M4, R2b S-M1..S-M4, S-m1) ---

namespace {

// What a case that may hang uses, on the heap: on a hang it is leaked with the stuck thread.
struct r2_world {
  cvc::exec_pool pool;
  cvc::app app;
  std::unique_ptr<exec_group> g;
  explicit r2_world(unsigned threads)
      : pool([threads] {
          cvc::pool_options po;
          po.threads = threads;
          return po;
        }()) {
    share_locks(app);
  }
};

// The group's pumps so far, from its pump hook (any thread).
struct pump_count {
  std::atomic<int> n{0};
  exec_group::options opts(cvc::app &app, const std::string &name) {
    exec_group::options o = opts_for(app, name);
    o.pump_hook_for_test = [this](exec_group &) { ++n; };
    return o;
  }
};

} // namespace

// C-M1 (R2a): an on_complete deferred to the attaching thread's queue (service_once) that detaches
// its own group: the entry's release must leave the freed record alone (it was queued again, and a
// worker pumped a detached group). Counts taken under Q right after the callback.
TEST(ExecPoolContinuous, DetachInsideItsOwnDeferredCallbackLeavesThePoolAlone) {
  cvc::pool_options po;
  po.threads = 1;
  cvc::exec_pool pool(po);
  if (pool.threads() == 0)
    GTEST_SKIP() << "no worker on this machine";
  cvc::app app;
  share_locks(app);
  exec_group g(opts_for(app, "self-detach"));
  std::atomic<bool> ran{false};
  bool attached_after = true;
  bool on_host = false;
  cvc::pool_stats at_detach;
  execute_options eo;
  eo.on_complete = [&](value_t) {
    on_host = !g.pumping_here();
    pool.detach(g);
    attached_after = pool.attached(g);
    at_detach = pool.stats();
    ran = true;
  };
  g.execute(std::string("(+ 1 2)"), eo);
  pool.attach(g);
  ASSERT_TRUE(drive(pool, [&] { return ran.load(); }, 10.0));
  const cvc::pool_stats after = pool.stats();
  EXPECT_TRUE(on_host) << "the callback ran inside a pump (not the deferred path)";
  EXPECT_FALSE(attached_after);
  EXPECT_EQ(after.wakes + after.requeues, at_detach.wakes + at_detach.requeues)
      << "the host entry's release queued the group's freed attachment after the detach";
  EXPECT_EQ(g.pool_link().state.load(), 0u);
}

// S-M1 (R2b): the same with a group that ran a while, watched 200 ms more: no wake and no pump of
// it after its detach, its word 0.
TEST(ExecPoolContinuous, DetachFromADeferredOnCompleteLeavesTheGroupAlone) {
  cvc::app app;
  share_locks(app);
  pump_count count;
  exec_group g(count.opts(app, "sm1"));
  cvc::pool_options po;
  po.threads = 2;
  cvc::exec_pool pool(po); // declared after the group: it goes first, joining its workers
  if (pool.threads() == 0)
    GTEST_SKIP() << "no worker on this machine";
  std::atomic<bool> ran{false};
  bool attached_after = true;
  std::uint64_t wakes_at_detach = 0, pumps_at_detach = 0;
  int hook_pumps_at_detach = 0;
  execute_options eo;
  eo.on_complete = [&](value_t) {
    pool.detach(g);
    const cvc::pool_stats s = pool.stats();
    wakes_at_detach = s.wakes;
    pumps_at_detach = s.pumps;
    hook_pumps_at_detach = count.n.load();
    attached_after = pool.attached(g);
    ran = true;
  };
  g.execute(std::string("(begin (set i 0) (while (< i 1000) (set i (+ i 1))) i)"), eo);
  pool.attach(g);
  const bool done = drive(pool, [&] { return ran.load(); }, 10.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(200)); // a worker's pump of it, if queued
  const cvc::pool_stats s = pool.stats();
  ASSERT_TRUE(done) << "the deferred callback never ran";
  EXPECT_FALSE(attached_after);
  EXPECT_EQ(s.attached, 0u);
  EXPECT_EQ(s.wakes, wakes_at_detach) << "the hold's release woke the group after its detach";
  EXPECT_EQ(s.pumps, pumps_at_detach) << "the pool pumped the group after its detach";
  EXPECT_EQ(count.n.load(), hook_pumps_at_detach) << "the group was pumped after its detach";
  EXPECT_EQ(g.pool_link().state.load(), 0u);
}

// C-M2 / S-M2: detach from inside a caller-pumped group's own pump (its on_complete, inline on the
// attaching thread) would wait for that pump: it is refused (std::logic_error) and returns; the
// group is detached afterwards. A 10 s ceiling turns the old hang into a failure (leaked).
TEST(ExecPoolContinuous, DetachInsideItsOwnPumpIsRefused) {
  auto *w = new r2_world(0);
  w->g = std::make_unique<exec_group>(opts_for(w->app, "inline-detach"));
  auto done = std::make_shared<std::atomic<bool>>(false);
  auto refused = std::make_shared<std::atomic<bool>>(false);
  auto attached_after = std::make_shared<std::atomic<bool>>(false);
  std::thread t([w, done, refused, attached_after] {
    execute_options eo;
    eo.on_complete = [w, refused](value_t) {
      try {
        w->pool.detach(*w->g);
      } catch (const std::logic_error &) {
        refused->store(true);
      }
    };
    w->g->execute(std::string("(+ 1 2)"), eo);
    w->pool.attach(*w->g);
    w->pool.pump_host(kInf);
    attached_after->store(w->pool.attached(*w->g));
    w->pool.detach(*w->g);
    done->store(true);
  });
  if (!settles([&] { return done->load(); }, 10.0)) {
    t.detach(); // stuck in detach inside its own pump: what it uses stays leaked
    FAIL() << "detach from inside a caller-pumped group's own pump never returned";
  }
  t.join();
  EXPECT_TRUE(refused->load()) << "detach inside the group's own pump was not refused";
  EXPECT_TRUE(attached_after->load());
  w->g.reset();
  delete w;
}

// C-M3: with_group (and detach) of a group from inside its own waiting body (a host_proxied call
// in a generator, run under its blocked worker's token) would wait for that worker, which waits for
// the body: refused (std::logic_error); host_held_here is true there. The body goes on and the
// program ends. A 10 s ceiling turns the old deadlock into a failure (leaked).
TEST(ExecPoolContinuous, WithGroupInsideItsOwnWaitingBodyIsRefused) {
  auto *w = new r2_world(1);
  if (w->pool.threads() == 0) {
    delete w;
    GTEST_SKIP() << "no worker on this machine";
  }
  w->g = std::make_unique<exec_group>(opts_for(w->app, "waiting"));
  auto refused = std::make_shared<std::atomic<int>>(0);
  auto held = std::make_shared<std::atomic<bool>>(false);
  auto finished_flag = std::make_shared<std::atomic<bool>>(false);
  environment_ptr env = environment::extend(builtins::make_default_environment());
  w->g->install(
      env, "proxied",
      [w, refused, held](std::span<const value_t>) -> value_t {
        held->store(cvc::detail::host_held_here(*w->g));
        try {
          w->pool.with_group(*w->g, [] {});
        } catch (const std::logic_error &) {
          ++*refused;
        }
        try {
          w->pool.detach(*w->g);
        } catch (const std::logic_error &) {
          ++*refused;
        }
        return value_t(int64_t(1));
      },
      provider_affinity::host_proxied);
  auto done = std::make_shared<std::atomic<bool>>(false);
  std::thread t([w, env, finished_flag, done] {
    execute_options eo;
    eo.env = environment::extend(env);
    eo.on_complete = [finished_flag](value_t) { finished_flag->store(true); };
    w->g->execute(
        std::string("(begin (set gen (generator (lambda () (yield (proxied 1))))) (next gen))"),
        eo);
    w->pool.attach(*w->g);
    const bool ok = drive(w->pool, [&] { return finished_flag->load(); }, 30.0);
    w->pool.detach(*w->g);
    done->store(ok);
  });
  if (!settles([&] { return done->load(); }, 10.0)) {
    t.detach(); // the body waits for its own worker, which waits for the body: leaked
    FAIL() << "with_group inside its own group's waiting body never returned";
  }
  t.join();
  EXPECT_EQ(refused->load(), 2) << "with_group and detach inside the waiting body were not refused";
  EXPECT_TRUE(held->load()) << "host_held_here was false in the group's waiting body";
  w->g.reset();
  delete w;
}

// C-M3's control: the same body, parked (no generator), runs under host_held on the attaching
// thread, and with_group of its own group nests.
TEST(ExecPoolContinuous, WithGroupInsideItsOwnParkedBodyNests) {
  cvc::pool_options po;
  po.threads = 1;
  cvc::exec_pool pool(po);
  if (pool.threads() == 0)
    GTEST_SKIP() << "no worker on this machine";
  cvc::app app;
  share_locks(app);
  exec_group g(opts_for(app, "parked"));
  environment_ptr env = environment::extend(builtins::make_default_environment());
  bool nested = false;
  g.install(
      env, "proxied",
      [&](std::span<const value_t>) -> value_t {
        pool.with_group(g, [&] { nested = true; });
        return value_t(int64_t(1));
      },
      provider_affinity::host_proxied);
  std::atomic<bool> fin{false};
  execute_options eo;
  eo.env = environment::extend(env);
  eo.on_complete = [&](value_t) { fin = true; };
  g.execute(std::string("(proxied 1)"), eo);
  pool.attach(g);
  const bool ok = drive(pool, [&] { return fin.load(); }, 10.0);
  pool.detach(g);
  ASSERT_TRUE(ok);
  EXPECT_TRUE(nested);
}

// C-M4: a group a worker's step wakes goes into that worker's next slot; with the other worker
// parked it must not wait out the waker's whole step: tx's native posts to rx and then runs on
// until rx has received (a 2 s ceiling). An idle worker is woken and steals the slot.
TEST(ExecPoolContinuous, AGroupWokenByABusyWorkerIsPumpedByAnIdleOne) {
  cvc::pool_options po;
  po.threads = 2;
  cvc::exec_pool pool(po);
  if (pool.threads() < 2)
    GTEST_SKIP() << "needs two workers";
  cvc::app app;
  share_locks(app);
  intrinsic_world rx(app, "rx");
  std::atomic<bool> parked{false}, got{false};
  builtins::register_fn(rx.env, "mark", [&](std::span<const value_t>) -> value_t {
    parked = true;
    return nil_value();
  });
  builtins::register_fn(rx.env, "got", [&](std::span<const value_t>) -> value_t {
    got = true;
    return nil_value();
  });
  rx.submit("(begin (mark) (msg-recv \"in\") (got))");
  pool.attach(*rx.group);
  ASSERT_TRUE(drive(pool, [&] { return parked.load(); }));
  pool.wait_for_work(0.02);                                   // rx's pump ends on the receive
  std::this_thread::sleep_for(std::chrono::milliseconds(50)); // both workers park
  exec_group tx(opts_for(app, "tx"));
  environment_ptr env = environment::extend(builtins::make_default_environment());
  std::atomic<bool> seen{false};
  double waited_ms = -1.0;
  builtins::register_fn(env, "poke", [&](std::span<const value_t>) -> value_t {
    rx.group->sched().post_message(resolve_channel_key("rx", "in"), value_t(int64_t(1)));
    const auto t0 = clock_type::now();
    while (!got.load() && clock_type::now() - t0 < std::chrono::seconds(2))
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    waited_ms = std::chrono::duration<double, std::milli>(clock_type::now() - t0).count();
    seen = got.load();
    return nil_value();
  });
  std::atomic<bool> txdone{false};
  execute_options eo;
  eo.env = environment::extend(env);
  eo.on_complete = [&](value_t) { txdone = true; };
  tx.execute(std::string("(poke)"), eo);
  pool.attach(tx);
  const bool ok = drive(pool, [&] { return txdone.load() && got.load(); }, 30.0);
  pool.detach(tx);
  pool.detach(*rx.group);
  ASSERT_TRUE(ok);
  std::cout << "rx received after " << waited_ms << " ms of tx's step\n";
  EXPECT_TRUE(seen.load()) << "rx waited in tx's worker's next slot while the other worker was "
                              "parked, until tx's step ended";
}

// S-M3 (R2b): a callback queued for the attaching thread must not freeze a busy group while that
// thread is busy elsewhere (not in the pool): the counter ends on the workers meanwhile; the
// callback runs at the next service.
TEST(ExecPoolContinuous, AQueuedCallbackDoesNotFreezeABusyGroup) {
  cvc::app app;
  share_locks(app);
  pump_count count;
  exec_group g(count.opts(app, "sm3"));
  cvc::pool_options po;
  po.threads = 2;
  po.pump_steps = 200;
  cvc::exec_pool pool(po);
  if (pool.threads() == 0)
    GTEST_SKIP() << "no worker on this machine";
  std::atomic<bool> called{false}, fin{false};
  execute_options quick;
  quick.on_complete = [&](value_t) { called = true; };
  g.execute(std::string("(+ 1 2)"), quick); // ends in the first pump: its callback is queued here
  execute_options slow;
  slow.env = env_with_fin(fin);
  const int pid = g.execute(counting_then_fin(20000), slow); // runs over many pumps
  pool.attach(g);
  const auto t0 = clock_type::now(); // busy elsewhere, servicing nothing, for up to 10 s
  while (!fin.load() && clock_type::now() - t0 < std::chrono::seconds(10))
    std::this_thread::yield();
  const bool ran_alone = fin.load();
  const int pumps_alone = count.n.load();
  const bool called_alone = called.load();
  const bool done = drive(pool, [&] { return fin.load() && called.load(); }, 30.0);
  pool.detach(g);
  std::printf("S-M3: %d pumps while the host serviced nothing\n", pumps_alone);
  ASSERT_TRUE(done);
  EXPECT_EQ(result_of(g, pid), "20000");
  EXPECT_FALSE(called_alone) << "the callback ran off its attaching thread's service";
  EXPECT_TRUE(ran_alone) << "the busy group stopped being pumped once a callback of it was queued "
                         << "for its attaching thread (" << pumps_alone << " pumps)";
}

// S-m1 (R2b): a host's exec_group::execute on an idle attached group, outside with_group, runs
// under the pool's with_group (exec_group.h): the program runs.
TEST(ExecPoolContinuous, AnExecuteOutsideWithGroupRuns) {
  cvc::app app;
  share_locks(app);
  pump_count count;
  exec_group g(count.opts(app, "smx"));
  cvc::pool_options po;
  po.threads = 2;
  cvc::exec_pool pool(po);
  if (pool.threads() == 0)
    GTEST_SKIP() << "no worker on this machine";
  pool.attach(g);
  ASSERT_TRUE(settles([&] { return count.n.load() >= 1 && pool.stats().active_workers == 0; }));
  std::atomic<bool> fin{false};
  execute_options eo;
  eo.env = env_with_fin(fin);
  const int pid = g.execute(counting_then_fin(10), eo);
  const bool ran = drive(pool, [&] { return fin.load(); }, 10.0);
  pool.detach(g);
  ASSERT_TRUE(ran) << "a submission outside with_group never ran (" << count.n.load() << " pumps)";
  EXPECT_EQ(result_of(g, pid), "10");
}

namespace {

// One run of S-M4's two deterministic groups: what the frame waiter saw of the busy group's
// progress, frame by frame.
std::vector<std::int64_t> sm4_run(bool continuous) {
  cvc::app app;
  share_locks(app);
  cvc::pool_options po;
  po.threads = 0;
  po.deterministic = true;
  po.pump_steps = 100;
  cvc::exec_pool pool(po);
  std::atomic<std::int64_t> ticks{0};
  std::vector<std::int64_t> seen;
  intrinsic_world frames(app, "sm4.frames"), busy(app, "sm4.busy");
  builtins::register_fn(frames.env, "snap", [&](std::span<const value_t>) -> value_t {
    seen.push_back(ticks.load());
    return nil_value();
  });
  builtins::register_fn(busy.env, "tick", [&](std::span<const value_t>) -> value_t {
    ++ticks;
    return nil_value();
  });
  std::atomic<int> ended{0};
  const auto on_end = [&ended](value_t) { ++ended; };
  const int pf = frames.submit(
      "(begin (set k 0) (while (< k 30) (begin (snap) (await 0) (set k (+ k 1)))) k)", on_end);
  const int pb =
      busy.submit("(begin (set i 0) (while (< i 30000) (begin (tick) (set i (+ i 1)))) i)", on_end);
  if (continuous) {
    pool.attach(*frames.group);
    pool.attach(*busy.group);
    drive(pool, [&] { return ended.load() == 2; });
    pool.detach(*frames.group);
    pool.detach(*busy.group);
  } else {
    std::vector<exec_group *> gs{frames.group.get(), busy.group.get()};
    pump_until_done(pool, gs, {{frames.group.get(), pf}, {busy.group.get(), pb}}, 100000);
  }
  return seen;
}

std::string sm4_show(const std::vector<std::int64_t> &v) {
  std::string s;
  for (std::size_t i = 0; i < v.size() && i < 12; ++i)
    s += (i ? " " : "") + std::to_string(v[i]);
  return s + (v.size() > 12 ? " ..." : "");
}

} // namespace

// S-M4 (R2b): deterministic continuous mode is reproducible with a frame waiter: a frame is the
// host's next pump_host pass, not the clock's 1 ms beat, so two runs see the same progress of the
// other group frame by frame, as two deterministic pump_all runs do (the control).
TEST(ExecPoolContinuous, DeterministicRunsRepeatWithAFrameWaiter) {
  const std::vector<std::int64_t> f1 = sm4_run(false), f2 = sm4_run(false);
  const std::vector<std::int64_t> c1 = sm4_run(true), c2 = sm4_run(true);
  std::printf("S-M4 fork-join: [%s] / [%s]\n", sm4_show(f1).c_str(), sm4_show(f2).c_str());
  std::printf("S-M4 continuous: [%s] / [%s]\n", sm4_show(c1).c_str(), sm4_show(c2).c_str());
  ASSERT_EQ(f1.size(), 30u);
  EXPECT_EQ(f1, f2) << "the fork-join control differs: the case is not deterministic itself";
  ASSERT_EQ(c1.size(), 30u);
  EXPECT_EQ(c1, c2) << "two deterministic continuous runs differ";
}

// C-m3: attach refuses a scheduler whose wake notifier another owner installed (a host batch's):
// installing its own would take that owner's slot, and clearing it at detach would leave the other
// with none.
TEST(ExecPoolContinuous, AttachRefusesASchedulerWithANotifier) {
  cvc::exec_pool pool(cvc::pool_options{});
  cvc::app app;
  exec_group g(opts_for(app, "batched"));
  g.sched().set_wake_notifier([](void *) noexcept {}, nullptr);
  EXPECT_THROW(pool.attach(g), std::logic_error);
  EXPECT_FALSE(pool.attached(g));
  g.sched().set_wake_notifier(nullptr, nullptr);
  pool.attach(g);
  EXPECT_TRUE(pool.attached(g));
  pool.detach(g);
}

// S-m7: wait_for_work counts a failed group once, until take_failed takes it, so a host loop that
// has not taken it yet waits instead of spinning.
TEST(ExecPoolContinuous, AFailedGroupIsWorkOnceUntilTaken) {
  cvc::exec_pool pool(cvc::pool_options{}); // threads 0: the host pumps it
  cvc::app app;
  exec_group::options bad_opts = opts_for(app, "bad");
  bad_opts.pump_hook_for_test = [](exec_group &) { throw std::runtime_error("boom"); };
  exec_group bad(bad_opts);
  bad.execute(std::string("(+ 1 2)"));
  pool.attach(bad);
  ASSERT_TRUE(drive(pool, [&] { return bad.failed(); }, 10.0));
  const bool first = pool.wait_for_work(0.0);
  const bool second = pool.wait_for_work(0.05);
  const std::vector<exec_group *> taken = pool.take_failed();
  EXPECT_TRUE(first) << "the failed group was not work";
  EXPECT_FALSE(second) << "the failed group was work again before it was taken";
  EXPECT_EQ(taken, std::vector<exec_group *>{&bad});
}

// S-m5: a worker blocked over 10 ms on a waiting body counts in long_host_waits while it waits, not
// once the attaching thread runs the body.
TEST(ExecPoolContinuous, ALongHostWaitIsCountedWhileItWaits) {
  cvc::pool_options po;
  po.threads = 1;
  cvc::exec_pool pool(po);
  if (pool.threads() == 0)
    GTEST_SKIP() << "no worker on this machine";
  cvc::app app;
  share_locks(app);
  exec_group g(opts_for(app, "slow-host"));
  environment_ptr env = environment::extend(builtins::make_default_environment());
  g.install(
      env, "proxied", [](std::span<const value_t>) -> value_t { return value_t(int64_t(1)); },
      provider_affinity::host_proxied);
  std::atomic<bool> fin{false};
  execute_options eo;
  eo.env = environment::extend(env);
  eo.on_complete = [&](value_t) { fin = true; };
  g.execute(std::string("(begin (set gen (generator (lambda () (yield (proxied 1))))) (next gen))"),
            eo);
  pool.attach(g); // the worker blocks on the body; this thread services nothing for a while
  const bool counted = settles([&] { return pool.stats().long_host_waits >= 1; }, 10.0);
  const bool done = drive(pool, [&] { return fin.load(); }, 10.0);
  pool.detach(g);
  EXPECT_TRUE(counted) << "the long wait was not counted while the worker waited";
  EXPECT_TRUE(done);
  EXPECT_EQ(pool.stats().long_host_waits, 1u);
}

// S-m6: an attaching thread inside a pump_all (fork-join, of another group) services its
// continuous host queue there: the waiting body of an attached group's blocked worker runs while
// the pump_all's group, on a helper worker, waits for it (a 2 s ceiling).
TEST(ExecPoolContinuous, APumpAllOfAnAttachingThreadServicesItsHostQueue) {
  cvc::pool_options po;
  po.threads = 2;
  cvc::exec_pool pool(po);
  if (pool.threads() < 2)
    GTEST_SKIP() << "needs two workers";
  cvc::app app;
  share_locks(app);
  hold_caller hold(app); // keeps the caller in its host group until b's pump began on a helper
  exec_group::options bo = opts_for(app, "forked");
  bo.pump_hook_for_test = [&hold](exec_group &) { hold.go.store(true); };
  exec_group a(opts_for(app, "attached")), b(bo);
  std::atomic<bool> body_ran{false}, a_done{false}, b_saw{false};
  environment_ptr env = environment::extend(builtins::make_default_environment());
  a.install(
      env, "proxied",
      [&](std::span<const value_t>) -> value_t {
        body_ran = true;
        return value_t(int64_t(1));
      },
      provider_affinity::host_proxied);
  execute_options ea;
  ea.env = environment::extend(env);
  ea.on_complete = [&](value_t) { a_done = true; };
  a.execute(std::string("(begin (set gen (generator (lambda () (yield (proxied 1))))) (next gen))"),
            ea);
  environment_ptr benv = environment::extend(builtins::make_default_environment());
  builtins::register_fn(benv, "wait-for-a", [&](std::span<const value_t>) -> value_t {
    const auto t0 = clock_type::now();
    while (!body_ran.load() && clock_type::now() - t0 < std::chrono::seconds(2))
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    b_saw = body_ran.load();
    return nil_value();
  });
  execute_options eb;
  eb.env = benv;
  b.execute(std::string("(wait-for-a)"), eb);
  pool.attach(a);
  ASSERT_TRUE(settles([&] { return pool.stats().long_host_waits >= 1; }, 10.0))
      << "a's worker never blocked on its body";
  pool.set_last_pump_for_test(b, 1.0); // a helper pumps b; the caller joins
  pool.set_last_pump_for_test(*hold.g, 1.0);
  std::vector<exec_group *> gs{hold.g.get(), &b};
  pool.pump_all(gs, pump_budget{50, kInf}, far_deadline());
  const bool done = drive(pool, [&] { return a_done.load(); }, 10.0);
  pool.detach(a);
  EXPECT_TRUE(b_saw.load()) << "the waiting body did not run in the pump_all's join";
  EXPECT_TRUE(done);
}

// C-m6 (release builds; a debug build asserts on the held token first): a worker's pump that
// throws (the group's token held outside the pool, a host's misuse) reaches the attaching thread,
// whose wait_for_work rethrows it, and the group is not re-pumped at once in a loop.
TEST(ExecPoolContinuous, AWorkersPumpExceptionReachesTheAttachingThread) {
  if (kDebugBuild)
    GTEST_SKIP() << "exec_group::pump asserts on a token held elsewhere";
  cvc::pool_options po;
  po.threads = 1;
  cvc::exec_pool pool(po);
  if (pool.threads() == 0)
    GTEST_SKIP() << "no worker on this machine";
  cvc::app app;
  share_locks(app);
  intrinsic_world w(app, "misused");
  std::atomic<bool> got{false};
  w.submit("(begin (msg-recv \"in\"))", [&](value_t) { got = true; });
  pool.attach(*w.group);
  pool.wait_for_work(0.02); // parked on its receive, idle
  std::atomic<bool> holding{false}, release_it{false};
  std::thread holder([&] {
    async_scheduler::pump_owner_scope token(w.group->sched()); // a host's misuse
    holding = true;
    while (!release_it.load())
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
  });
  while (!holding.load())
    std::this_thread::yield();
  const std::uint64_t pumps0 = pool.stats().pumps;
  w.group->sched().post_message(resolve_channel_key("misused", "in"), value_t(int64_t(1)));
  bool thrown = false;
  const auto t0 = clock_type::now();
  while (!thrown && clock_type::now() - t0 < std::chrono::seconds(10)) {
    try {
      pool.wait_for_work(0.01);
    } catch (const std::exception &) {
      thrown = true;
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  const std::uint64_t pumps = pool.stats().pumps - pumps0;
  release_it = true;
  holder.join();
  bool done = false;
  try {
    done = drive(pool, [&] { return got.load(); }, 10.0);
  } catch (const std::exception &) {
    done = drive(pool, [&] { return got.load(); }, 10.0); // the queued exception, once more
  }
  pool.detach(*w.group);
  EXPECT_TRUE(thrown) << "the pump's exception never reached the attaching thread";
  EXPECT_LE(pumps, 5u) << "the worker re-pumped the group in a loop";
  EXPECT_TRUE(done) << "the group was not pumped again once the token was free";
}

// --- the skeptic pass (K-1..K-13), adopted ---

// K-1: the detach of group A waits for A's pump (step 2), servicing this thread's queue, and runs a
// callback of group B that calls with_group(A). On the detaching thread A is taken as unattached,
// at once: A's worker holds the token, so with_group throws, and the detach goes on (A's pump then
// runs into a waiting body for this thread, which the detach services). It waited for A's token,
// servicing nothing, before: a hang (a 15 s ceiling; leaked on failure).
TEST(ExecPoolContinuous, AWithGroupInsideADetachOfTheSameGroupDoesNotWaitForItsPump) {
  auto *w = new r2_world(2);
  if (w->pool.threads() < 2) {
    delete w;
    GTEST_SKIP() << "needs two workers";
  }
  auto b = std::make_shared<std::unique_ptr<exec_group>>();
  w->g = std::make_unique<exec_group>(opts_for(w->app, "a"));
  *b = std::make_unique<exec_group>(opts_for(w->app, "b"));
  auto gate = std::make_shared<std::atomic<bool>>(false);
  auto in_gate = std::make_shared<std::atomic<bool>>(false);
  auto outcome = std::make_shared<std::atomic<int>>(0); // 1 ran, 2 threw
  environment_ptr env = environment::extend(builtins::make_default_environment());
  builtins::register_fn(env, "gate", [gate, in_gate](std::span<const value_t>) -> value_t {
    in_gate->store(true);
    const auto t0 = clock_type::now();
    while (!gate->load() && clock_type::now() - t0 < std::chrono::seconds(5))
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return nil_value();
  });
  w->g->install(
      env, "body", [](std::span<const value_t>) -> value_t { return value_t(int64_t(1)); },
      provider_affinity::host_proxied);
  execute_options ea;
  ea.env = environment::extend(env);
  w->g->execute(
      std::string("(begin (gate) (set gen (generator (lambda () (yield (body))))) (next gen))"),
      ea);
  execute_options eb;
  eb.on_complete = [w, gate, outcome](value_t) {
    gate->store(true); // A's pump goes on, into its waiting body
    try {
      w->pool.with_group(*w->g, [] {});
      outcome->store(1);
    } catch (const std::exception &) {
      outcome->store(2);
    }
  };
  (*b)->execute(std::string("(+ 1 2)"), eb);
  auto done = std::make_shared<std::atomic<bool>>(false);
  std::thread t([w, b, in_gate, done] {
    w->pool.attach(*w->g);
    settles([&] { return in_gate->load(); });
    w->pool.attach(**b);
    settles([&] { return w->pool.stats().pumps >= 1; }); // b's pump ended: its callback queued
    w->pool.detach(*w->g);                               // step 2 runs b's callback first
    w->pool.detach(**b);
    done->store(true);
  });
  if (!settles([&] { return done->load(); }, 15.0)) {
    t.detach(); // this thread waits for A's token, A's worker for its waiting body: leaked
    FAIL() << "a with_group of the group being detached, from an entry run by that detach, hung "
              "(outcome "
           << outcome->load() << ")";
  }
  t.join();
  EXPECT_EQ(outcome->load(), 2) << "with_group on the detaching thread did not take the token at "
                                   "once (and throw, A's worker holding it)";
  b->reset();
  w->g.reset();
  delete w;
}

// C-m5 (adopted with K-1): a with_group that raced a detach (it saw the group attached, then waited
// for the detach) waits for the detach's token (its step 5 runs a queued host body for 300 ms
// here), not throwing.
TEST(ExecPoolContinuous, AWithGroupThatRacedADetachWaitsForItsToken) {
  cvc::pool_options po;
  po.threads = 1;
  cvc::exec_pool pool(po);
  if (pool.threads() == 0)
    GTEST_SKIP() << "no worker on this machine";
  cvc::app app;
  share_locks(app);
  exec_group g(opts_for(app, "raced"));
  std::atomic<bool> started{false}, release{false}, step5{false};
  environment_ptr env = environment::extend(builtins::make_default_environment());
  builtins::register_fn(env, "hold", [&](std::span<const value_t>) -> value_t {
    pool.run_on_host(g, [&] { // a host body: the detach's step 5 runs it, holding the token
      step5 = true;
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
    });
    started = true;
    const auto t0 = clock_type::now();
    while (!release.load() && clock_type::now() - t0 < std::chrono::seconds(5))
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return nil_value();
  });
  execute_options eo;
  eo.env = env;
  g.execute(std::string("(hold)"), eo);
  pool.attach(g);
  ASSERT_TRUE(settles([&] { return started.load(); }));
  std::atomic<int> outcome{0}; // 1 ran, 2 threw
  std::atomic<bool> calling{false};
  std::thread racer([&] {
    while (!(g.pool_link().state.load() & cvc::state_exec::detail::group_pool_link::detaching))
      std::this_thread::yield();
    calling = true;
    try {
      pool.with_group(g, [&] {});
      outcome = 1;
    } catch (const std::exception &) {
      outcome = 2;
    }
  });
  std::thread releaser([&] {
    while (!calling.load())
      std::this_thread::yield();
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // the racer waits on detaching
    release = true;
  });
  pool.detach(g); // waits for the pump, then step 5 runs the body for 300 ms
  racer.join();
  releaser.join();
  EXPECT_TRUE(step5.load());
  EXPECT_EQ(outcome.load(), 1) << "a with_group that raced a detach threw on the detach's token";
}

// K-2: a native of attached group X (on a worker) submits to attached group Y while Y's native
// (on the other worker) submits to X: from inside a pump of the pool each execute is refused
// (std::logic_error) rather than waiting in with_group for the other's pump. Both return, each
// with the refusal; they deadlocked before (a 12 s ceiling; leaked on failure).
TEST(ExecPoolContinuous, CrossSubmitsFromTwoWorkersAreRefused) {
  auto *w = new r2_world(2);
  if (w->pool.threads() < 2) {
    delete w;
    GTEST_SKIP() << "needs two workers";
  }
  auto y = std::make_shared<std::unique_ptr<exec_group>>();
  w->g = std::make_unique<exec_group>(opts_for(w->app, "x"));
  *y = std::make_unique<exec_group>(opts_for(w->app, "y"));
  auto in = std::make_shared<std::atomic<int>>(0);
  auto out = std::make_shared<std::atomic<int>>(0);
  auto refused = std::make_shared<std::atomic<int>>(0);
  const auto cross_env = [&](exec_group *other) {
    environment_ptr env = environment::extend(builtins::make_default_environment());
    builtins::register_fn(
        env, "cross", [in, out, refused, other](std::span<const value_t>) -> value_t {
          ++*in;
          const auto t0 = clock_type::now();
          while (in->load() < 2 && clock_type::now() - t0 < std::chrono::seconds(2))
            std::this_thread::yield();
          try {
            other->execute(std::string("(+ 1 2)"));
          } catch (const std::logic_error &) {
            ++*refused;
          }
          ++*out;
          return nil_value();
        });
    return env;
  };
  execute_options ex, ey;
  ex.env = cross_env(y->get());
  ey.env = cross_env(w->g.get());
  w->g->execute(std::string("(cross)"), ex);
  (*y)->execute(std::string("(cross)"), ey);
  auto done = std::make_shared<std::atomic<bool>>(false);
  std::thread t([w, y, out, done] {
    w->pool.attach(*w->g);
    w->pool.attach(**y);
    drive(w->pool, [&] { return out->load() == 2; }, 8.0);
    w->pool.detach(*w->g);
    w->pool.detach(**y);
    done->store(true);
  });
  if (!settles([&] { return done->load(); }, 12.0)) {
    t.detach(); // both workers wait in each other's with_group: leaked
    FAIL() << "two natives submitting to each other's attached group from two workers deadlocked ("
           << out->load() << " of 2 returned)";
  }
  t.join();
  EXPECT_EQ(out->load(), 2);
  EXPECT_EQ(refused->load(), 2) << "an execute from inside a pump of the pool was not refused";
  y->reset();
  w->g.reset();
  delete w;
}

// K-2, the host-hold form: the host H inside with_group(A) executes into B while B's native (on a
// worker) executes into A. B's native is refused (inside a pump); H, a host outside any pump,
// takes the with_group path and its submission lands once B's pump ends. They deadlocked before
// (B's native waited for A's hold, H for B's pump; a 12 s ceiling; leaked on failure).
TEST(ExecPoolContinuous, AHostInsideWithGroupAndANativeSubmittingBackDoNotDeadlock) {
  auto *w = new r2_world(1);
  if (w->pool.threads() == 0) {
    delete w;
    GTEST_SKIP() << "no worker on this machine";
  }
  auto b = std::make_shared<std::unique_ptr<exec_group>>();
  w->g = std::make_unique<exec_group>(opts_for(w->app, "a"));
  *b = std::make_unique<exec_group>(opts_for(w->app, "b"));
  auto in_native = std::make_shared<std::atomic<bool>>(false);
  auto in_hold = std::make_shared<std::atomic<bool>>(false);
  auto refused = std::make_shared<std::atomic<bool>>(false);
  auto landed = std::make_shared<std::atomic<bool>>(false);
  auto fin = std::make_shared<std::atomic<bool>>(false);
  environment_ptr benv = environment::extend(builtins::make_default_environment());
  builtins::register_fn(
      benv, "back", [w, in_native, in_hold, refused](std::span<const value_t>) -> value_t {
        in_native->store(true);
        const auto t0 = clock_type::now();
        while (!in_hold->load() && clock_type::now() - t0 < std::chrono::seconds(2))
          std::this_thread::yield();
        try {
          w->g->execute(std::string("(+ 1 2)")); // into A, which H holds
        } catch (const std::logic_error &) {
          refused->store(true);
        }
        return nil_value();
      });
  execute_options eb;
  eb.env = benv;
  (*b)->execute(std::string("(back)"), eb);
  auto done = std::make_shared<std::atomic<bool>>(false);
  std::thread t([w, b, in_native, in_hold, landed, fin, done] {
    w->pool.attach(*w->g);
    w->pool.attach(**b);
    settles([&] { return in_native->load(); });
    w->pool.with_group(*w->g, [&] {
      in_hold->store(true);
      execute_options eo;
      eo.env = env_with_fin(*fin);
      (*b)->execute(counting_then_fin(5), eo); // into B, whose native runs on the worker
      landed->store(true);
    });
    drive(w->pool, [&] { return fin->load(); }, 8.0);
    w->pool.detach(*w->g);
    w->pool.detach(**b);
    done->store(true);
  });
  if (!settles([&] { return done->load(); }, 12.0)) {
    t.detach(); // H waits for B's pump, B's native for A's hold: leaked
    FAIL() << "a host in with_group(A) submitting to B, and B's native submitting to A, deadlocked";
  }
  t.join();
  EXPECT_TRUE(refused->load()) << "B's native's execute into A was not refused";
  EXPECT_TRUE(landed->load());
  EXPECT_TRUE(fin->load()) << "the host's submission into B never ran";
  b->reset();
  w->g.reset();
  delete w;
}

// K-3: a failed group torn down before take_failed must not hide the next failure from
// wait_for_work (a count of failures seen went stale).
TEST(ExecPoolContinuous, AGroupFailingAfterAnotherIsTornDownIsWork) {
  cvc::exec_pool pool(cvc::pool_options{}); // threads 0: the host pumps them
  cvc::app app;
  const auto bad = [&](const std::string &name) {
    exec_group::options o = opts_for(app, name);
    o.pump_hook_for_test = [](exec_group &) { throw std::runtime_error("boom"); };
    auto g = std::make_unique<exec_group>(o);
    g->execute(std::string("(+ 1 2)"));
    return g;
  };
  std::unique_ptr<exec_group> f1 = bad("f1"), f2 = bad("f2");
  pool.attach(*f1);
  pool.attach(*f2);
  ASSERT_TRUE(drive(pool, [&] { return f1->failed() && f2->failed(); }, 10.0));
  EXPECT_TRUE(pool.wait_for_work(0.0)); // both reported
  f1.reset();                           // torn down attached and failed, before take_failed
  std::unique_ptr<exec_group> f3 = bad("f3");
  pool.attach(*f3);
  pool.pump_host(kInf);
  ASSERT_TRUE(f3->failed());
  const bool work = pool.wait_for_work(0.05);
  const std::vector<exec_group *> taken = pool.take_failed();
  EXPECT_TRUE(work) << "a group that failed after another failed group's teardown was not work";
  EXPECT_EQ(taken.size(), 2u);
}

// K-4: the attaching thread pumping its own caller-pumped group inside pump_host is not waiting:
// a worker group with a callback queued for it is not handed over meanwhile. The host's native
// blocks until the busy worker group has been pumped 20 more times (10 s only as a hang guard); a
// handed-over group froze until the native's guard ran out.
TEST(ExecPoolContinuous, AHostPumpingItsOwnGroupDoesNotFreezeABusyOne) {
  cvc::app app;
  share_locks(app);
  pump_count busy_count;
  cvc::pool_options po;
  po.threads = 1;
  po.pump_steps = 200;
  exec_group busy(busy_count.opts(app, "busy"));
  exec_group slow(opts_for(app, "slow", group_affinity::host));
  cvc::exec_pool pool(po); // declared after the groups: it goes first, joining its worker
  if (pool.threads() == 0)
    GTEST_SKIP() << "no worker on this machine";
  std::atomic<bool> called{false}, fin{false}, reached{false};
  environment_ptr senv = environment::extend(builtins::make_default_environment());
  builtins::register_fn(senv, "await-busy", [&](std::span<const value_t>) -> value_t {
    // Once busy's first pumps queued its callback, wait for 20 more of its pumps.
    const auto t0 = clock_type::now();
    while (busy_count.n.load() < 3 && clock_type::now() - t0 < std::chrono::seconds(10))
      std::this_thread::yield();
    const int start = busy_count.n.load();
    while (busy_count.n.load() < start + 20 && clock_type::now() - t0 < std::chrono::seconds(10))
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    reached = busy_count.n.load() >= start + 20;
    return nil_value();
  });
  execute_options se;
  se.env = senv;
  slow.execute(std::string("(await-busy)"), se);
  execute_options quick;
  quick.on_complete = [&](value_t) { called = true; };
  busy.execute(std::string("(+ 1 2)"), quick); // its callback is queued for this thread
  execute_options eo;
  eo.env = env_with_fin(fin);
  busy.execute(counting_then_fin(200000), eo); // runs over many pumps
  pool.attach(slow);
  pool.attach(busy);
  pool.pump_host(kInf); // pumps slow: its native waits on busy's progress
  const bool done = drive(pool, [&] { return fin.load() && called.load(); }, 60.0);
  pool.detach(busy);
  pool.detach(slow);
  ASSERT_TRUE(done);
  EXPECT_TRUE(reached.load()) << "the busy worker group stopped while its attaching thread pumped "
                                 "its own caller-pumped group inside pump_host";
}

// K-8: a worker's pump that throws twice (or more: a wake left from before may add one) while the
// group's token is held outside the pool (a host's misuse), its attaching thread servicing nothing
// meanwhile, leaves one rethrowing entry: the attaching thread gets one exception, and the later
// ones are logged through the group's app and dropped (release builds; a debug build asserts on
// the held token).
TEST(ExecPoolContinuous, TwoWorkerPumpExceptionsLeaveOneRethrow) {
  if (kDebugBuild)
    GTEST_SKIP() << "exec_group::pump asserts on a token held elsewhere";
  cvc::pool_options po;
  po.threads = 1;
  cvc::exec_pool pool(po);
  if (pool.threads() == 0)
    GTEST_SKIP() << "no worker on this machine";
  cvc::app app;
  share_locks(app);
  const std::string log_key("pool_log");
  app.data(log_key, std::string());
  app.properties("system.log_output", log_key);
  intrinsic_world w(app, "misused2");
  std::atomic<bool> got{false};
  w.submit("(begin (msg-recv \"in\") (msg-recv \"in\"))", [&](value_t) { got = true; });
  pool.attach(*w.group);
  pool.wait_for_work(0.02); // parked on its receive, idle
  std::atomic<bool> holding{false}, release_it{false};
  std::thread holder([&] {
    async_scheduler::pump_owner_scope token(w.group->sched()); // a host's misuse
    holding = true;
    while (!release_it.load())
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
  });
  while (!holding.load())
    std::this_thread::yield();
  const std::string key = resolve_channel_key("misused2", "in");
  for (int i = 0; i < 2; ++i) { // two wakes, two pumps that throw (this thread services nothing)
    const std::uint64_t p0 = pool.stats().pumps;
    w.group->sched().post_message(key, value_t(int64_t(i)));
    ASSERT_TRUE(settles([&] { return pool.stats().pumps > p0; }));
  }
  release_it = true;
  holder.join();
  int rethrows = 0;
  const auto t0 = clock_type::now();
  while (!got.load() && clock_type::now() - t0 < std::chrono::seconds(10)) {
    try {
      if (!pool.pump_host(kInf))
        pool.wait_for_work(0.002);
    } catch (const std::exception &) {
      ++rethrows;
    }
  }
  pool.detach(*w.group);
  const std::string log = app.data<std::string>(log_key);
  std::size_t drops = 0;
  for (std::size_t at = log.find("dropped"); at != std::string::npos;
       at = log.find("dropped", at + 1))
    ++drops;
  std::cout << "rethrows " << rethrows << ", drops logged " << drops << "\n";
  EXPECT_TRUE(got.load());
  EXPECT_EQ(rethrows, 1) << "not one rethrowing entry a group at a time";
  EXPECT_GE(drops, 1u) << "the second pump's exception was not logged and dropped";
}

// K-10: a host entry (a deferred callback) that detaches its thread's last group lets the thread's
// record go: wait_for_work then has nothing to wait for and returns false at once, rather than
// waiting out its time (10 s here; 5 s is only the guard that tells the two apart).
TEST(ExecPoolContinuous, ALastGroupDetachedByAnEntryLeavesNoRecord) {
  cvc::pool_options po;
  po.threads = 1;
  cvc::exec_pool pool(po);
  if (pool.threads() == 0)
    GTEST_SKIP() << "no worker on this machine";
  cvc::app app;
  share_locks(app);
  exec_group g(opts_for(app, "last"));
  std::atomic<bool> ran{false};
  execute_options eo;
  eo.on_complete = [&](value_t) {
    pool.detach(g); // the thread's last group, from its own deferred callback
    ran = true;
  };
  g.execute(std::string("(+ 1 2)"), eo);
  pool.attach(g);
  ASSERT_TRUE(drive(pool, [&] { return ran.load(); }, 10.0));
  ASSERT_FALSE(pool.attached(g));
  const auto t0 = clock_type::now();
  const bool work = pool.wait_for_work(10.0);
  const double took = std::chrono::duration<double>(clock_type::now() - t0).count();
  std::cout << "wait_for_work with nothing attached returned after " << took << " s\n";
  EXPECT_FALSE(work);
  EXPECT_LT(took, 5.0) << "the thread's record outlived its last group: wait_for_work waited";
}

// --- R1-mid fixes (P) ---

// C-M1 (S1-Q1): groups on a single-mode lock manager, given to a pool with workers built apart
// from their app, are pumped on the caller only: their with-lock sections never meet on two
// threads, and no worker pumps them.
TEST(ExecPool, SingleManagerGroupsStayOnTheCaller) {
  if (kDebugBuild)
    GTEST_SKIP() << "a pool with workers given a single-manager group asserts in debug builds";
  cvc::app app;
  app.root()("probe.count").value(std::string("0"));
  ASSERT_EQ(app.lock_manager()->threading(), lock_threading::single);
  cvc::pool_options po;
  po.threads = 4;
  cvc::exec_pool pool(po);
  if (pool.threads() == 0)
    GTEST_SKIP() << "no worker on this machine";
  thread_log log;
  std::vector<std::unique_ptr<intrinsic_world>> worlds;
  std::vector<exec_group *> groups;
  std::vector<int> pids;
  for (int i = 0; i < 6; ++i) {
    worlds.push_back(std::make_unique<intrinsic_world>(app, "single" + std::to_string(i),
                                                       group_affinity::any, "probe"));
    builtins::register_fn(worlds.back()->env, "tid", [&log](std::span<const value_t>) -> value_t {
      log.add();
      return nil_value();
    });
    EXPECT_TRUE(worlds.back()->group->pool_link().caller_only);
    groups.push_back(worlds.back()->group.get());
    pids.push_back(worlds.back()->submit(
        "(begin (set i 0) (while (< i 100) (begin (tid) (with-lock \"/probe\" (state-set "
        "\"count\" (str (+ 1 (int (state-get \"count\")))))) (set i (+ i 1)))) \"done\")"));
  }
  for (int round = 0; round < 20000; ++round) {
    bool all = true;
    for (std::size_t i = 0; i < groups.size(); ++i)
      all = all && finished(*groups[i], pids[i]);
    if (all)
      break;
    for (exec_group *g : groups)
      pool.set_last_pump_for_test(*g, 1.0);
    pool.pump_all(groups, pump_budget{500, std::numeric_limits<double>::infinity()},
                  far_deadline());
  }
  for (std::size_t i = 0; i < groups.size(); ++i)
    EXPECT_EQ(result_of(*groups[i], pids[i]), "\"done\"");
  EXPECT_EQ(app.root()("probe.count").value(), "600");
  EXPECT_EQ(pool.stats().worker_pumps, 0u) << "a worker pumped a group on a single manager";
  EXPECT_EQ(log.distinct(), std::set<std::thread::id>{std::this_thread::get_id()});
}

// S-M1: a busy group 0 in exec_pump_frame does not keep a frame group from ever starting, at
// threads 0 and for a host-affinity frame group with workers: a group skipped last frame starts
// before group 0.
TEST(AppExecPool, ABusyGroupZeroDoesNotStarveFrameGroups) {
  struct cfg {
    unsigned threads;
    group_affinity affinity;
  };
  for (const cfg c : {cfg{0, group_affinity::any}, cfg{2, group_affinity::host}}) {
    cvc::app app;
    cvc::pool_options po;
    po.threads = c.threads;
    app.exec_pool(po);
    async_scheduler &zero = app.exec_scheduler();
    environment_ptr env0 = environment::extend(builtins::make_default_environment());
    builtins::register_fn(env0, "spin", [](std::span<const value_t>) -> value_t {
      const auto t0 = clock_type::now();
      while (clock_type::now() - t0 < std::chrono::milliseconds(2)) {
      }
      return nil_value();
    });
    execute_options o0;
    o0.env = env0;
    const int spinner = zero.execute(std::string("(while #t (spin))"), o0);
    exec_group fg(opts_for(app, "frame", c.affinity));
    const int pid = fg.execute(std::string("(begin (set i 0) (while (< i 10) (set i (+ i 1))) i)"));
    cvc::frame_group_handle handle = app.register_frame_group(fg);
    const auto t0 = clock_type::now();
    for (int f = 0; f < 5 && !finished(fg, pid); ++f)
      app.exec_pump_frame();
    std::cout << "threads " << c.threads << ": "
              << std::chrono::duration<double>(clock_type::now() - t0).count() << " s\n";
    EXPECT_TRUE(finished(fg, pid)) << "threads " << c.threads << ": the frame group never ran";
    handle.reset();
    zero.kill(spinner);
  }
}

// N1: the other direction. Two frame groups that each fill a frame, pumped on the caller
// (threads 0, or host affinity), do not starve group 0: of the k caller-pumped units (group 0 and
// the two frame groups) each starts at least once in k frames, so group 0 never goes more than
// k - 1 frames without a step, and both frame groups keep running. On a fast build group 0 also
// fits beside one of them and runs every other frame; on a slow one (Debug, sanitizers) every unit
// fills its frame and they take turns. Before N1's fix group 0 took no step for 11 frames running.
TEST(AppExecPool, TwoBusyFrameGroupsDoNotStarveGroupZero) {
  struct cfg {
    unsigned threads;
    group_affinity affinity;
  };
  for (const cfg c : {cfg{0, group_affinity::any}, cfg{2, group_affinity::host}}) {
    cvc::app app;
    cvc::pool_options po;
    po.threads = c.threads;
    app.exec_pool(po);
    async_scheduler &zero = app.exec_scheduler();
    std::atomic<long> ticks{0}, spins_a{0}, spins_b{0};
    environment_ptr env0 = environment::extend(builtins::make_default_environment());
    builtins::register_fn(env0, "tick", [&ticks](std::span<const value_t>) -> value_t {
      ++ticks;
      return nil_value();
    });
    execute_options o0;
    o0.env = env0;
    const int ticker = zero.execute(std::string("(while #t (tick))"), o0);
    const auto spin_env = [](std::atomic<long> &count) {
      environment_ptr e = environment::extend(builtins::make_default_environment());
      builtins::register_fn(e, "spin", [&count](std::span<const value_t>) -> value_t {
        ++count;
        const auto t0 = clock_type::now();
        while (clock_type::now() - t0 < std::chrono::milliseconds(2)) {
        }
        return nil_value();
      });
      return e;
    };
    exec_group a(opts_for(app, "busyA", c.affinity)), b(opts_for(app, "busyB", c.affinity));
    execute_options ea, eb;
    ea.env = spin_env(spins_a);
    eb.env = spin_env(spins_b);
    const int pa = a.execute(std::string("(while #t (spin))"), ea);
    const int pb = b.execute(std::string("(while #t (spin))"), eb);
    cvc::frame_group_handle ha = app.register_frame_group(a);
    cvc::frame_group_handle hb = app.register_frame_group(b);
    std::string per_frame;
    int longest_without = 0, run = 0;
    for (int f = 0; f < 12; ++f) {
      const long before = ticks.load();
      app.exec_pump_frame();
      const long got = ticks.load() - before;
      per_frame += std::to_string(got) + " ";
      run = got == 0 ? run + 1 : 0;
      longest_without = std::max(longest_without, run);
    }
    std::cout << "threads " << c.threads << ": group 0 ticks per frame: " << per_frame
              << "; spins A " << spins_a.load() << ", B " << spins_b.load() << "\n";
    constexpr int kCallerUnits = 3; // group 0 and the two frame groups
    EXPECT_LE(longest_without, kCallerUnits - 1) << "threads " << c.threads << ": group 0 starved";
    EXPECT_GT(spins_a.load(), 0) << "threads " << c.threads;
    EXPECT_GT(spins_b.load(), 0) << "threads " << c.threads;
    ha.reset();
    hb.reset();
    a.sched().kill(pa);
    b.sched().kill(pb);
    zero.kill(ticker);
  }
}

// S-M2's pool half:// S-M2's pool half: a process waiting for the next frame ((await x) on a plain
// value) is work for the next pump.
TEST(ExecGroup, WantsPumpCountsAFrameWaiter) {
  cvc::app app;
  intrinsic_world w(app, "frames");
  const int pid = w.submit("(begin (await 1) 2)");
  w.group->pump(kSteps);
  ASSERT_FALSE(finished(*w.group, pid));
  EXPECT_EQ(w.group->sched().awaiting_frame_count(), 1u);
  EXPECT_TRUE(w.group->wants_pump()) << "a frame waiter wants the pump that wakes it";
  w.group->pump(kSteps);
  ASSERT_TRUE(finished(*w.group, pid));
  EXPECT_FALSE(w.group->wants_pump());
}

// C-m1, C-m7: in a threaded pump_all every on_complete runs after the join (a caller-pumped
// group's too, so host code never runs beside the workers), and host bodies and callbacks run
// with their group's registry bound. The `any` group is held to a worker (hold_caller), so its
// host body is queued and run by the caller, not inline in its pump.
TEST(ExecPool, ThreadedCallbacksRunAfterTheJoinWithTheirRegistry) {
  cvc::pool_options po;
  po.threads = 2;
  cvc::exec_pool pool(po);
  if (pool.threads() == 0)
    GTEST_SKIP() << "no worker on this machine";
  cvc::app app;
  share_locks(app);
  hold_caller hold(app);
  std::mutex m;
  std::uint64_t pumped_id = 0, body_id = 0;
  std::thread::id body_thread;
  std::vector<bool> callbacks; // whether a callback ran while this thread pumped a group
  exec_group::options ao = opts_for(app, "any");
  std::thread::id pumped_on;
  ao.pump_hook_for_test = [&](exec_group &) {
    pumped_on = std::this_thread::get_id();
    hold.go.store(true);
  };
  exec_group any_group(ao);
  environment_ptr env = environment::extend(builtins::make_default_environment());
  builtins::register_fn(env, "note-id", [&](call_args a) -> value_t {
    std::lock_guard<std::mutex> lock(m);
    pumped_id = cycle_registry_id(a.context());
    return nil_value();
  });
  any_group.install(
      env, "proxied",
      [&](call_args a) -> value_t {
        std::lock_guard<std::mutex> lock(m);
        body_id = cycle_registry_id(a.context());
        body_thread = std::this_thread::get_id();
        return value_t(int64_t(5));
      },
      provider_affinity::host_proxied);
  const auto record = [&](value_t) {
    std::lock_guard<std::mutex> lock(m);
    callbacks.push_back(hold.g->pumping_here() || any_group.pumping_here());
  };
  execute_options he, ae;
  he.on_complete = record;
  ae.env = env;
  ae.on_complete = record;
  hold.g->execute(std::string("1"), he);
  const int pid = any_group.execute(std::string("(begin (note-id) (proxied))"), ae);
  for (int round = 0; round < 100 && !finished(any_group, pid); ++round) {
    hold.go.store(false);
    pool.set_last_pump_for_test(*hold.g, 1.0);
    pool.set_last_pump_for_test(any_group, 1.0);
    pool.pump_all(std::vector<exec_group *>{hold.g.get(), &any_group}, pump_budget{50, kInf},
                  far_deadline());
  }
  ASSERT_TRUE(finished(any_group, pid));
  EXPECT_EQ(result_of(any_group, pid), "5");
  ASSERT_NE(pumped_on, std::this_thread::get_id()) << "the any group was pumped on the caller";
  ASSERT_GE(callbacks.size(), 2u);
  for (const bool in : callbacks)
    EXPECT_FALSE(in) << "a callback ran inside a pump, beside the workers";
  EXPECT_NE(pumped_id, 0u);
  EXPECT_EQ(pumped_id, cycle_registry_id(*any_group.registry()));
  EXPECT_EQ(body_thread, std::this_thread::get_id()) << "the host body ran off the caller";
  EXPECT_EQ(body_id, pumped_id) << "the host body ran without its group's registry";
}

// N4: one group's host bodies run in the order they were queued. Process x parks on its call,
// then process y blocks its worker on one inside a generator body: the caller runs x's body
// first (y's worker being blocked, no thread pumps the group), then y's.
TEST(ExecPool, OneGroupsHostBodiesRunInQueueOrder) {
  cvc::pool_options po;
  po.threads = 2;
  cvc::exec_pool pool(po);
  if (pool.threads() == 0)
    GTEST_SKIP() << "no worker on this machine";
  cvc::app app;
  share_locks(app);
  hold_caller hold(app);
  std::mutex m;
  std::vector<std::string> order;
  std::thread::id pumped_on;
  exec_group::options ao = opts_for(app, "fifo");
  ao.pump_hook_for_test = [&](exec_group &) {
    pumped_on = std::this_thread::get_id();
    hold.go.store(true);
  };
  exec_group g(ao);
  environment_ptr env = environment::extend(builtins::make_default_environment());
  g.install(
      env, "proxied",
      [&](std::span<const value_t> args) -> value_t {
        std::lock_guard<std::mutex> lock(m);
        order.push_back(to_string(args[0]));
        return args[0];
      },
      provider_affinity::host_proxied);
  execute_options eo;
  eo.env = env;
  const int x = g.execute(std::string("(proxied \"x\")"), eo);
  execute_options eo2;
  eo2.env = environment::extend(env);
  const int y = g.execute(
      std::string("(begin (set gen (generator (lambda () (yield (proxied \"y\"))))) (next gen))"),
      eo2);
  for (int round = 0; round < 100 && !(finished(g, x) && finished(g, y)); ++round) {
    hold.go.store(false);
    pool.set_last_pump_for_test(*hold.g, 1.0);
    pool.set_last_pump_for_test(g, 1.0);
    pool.pump_all(std::vector<exec_group *>{hold.g.get(), &g}, pump_budget{1000, kInf},
                  far_deadline());
  }
  ASSERT_TRUE(finished(g, x) && finished(g, y));
  ASSERT_NE(pumped_on, std::this_thread::get_id()) << "the group was pumped on the caller";
  EXPECT_EQ(order, (std::vector<std::string>{"\"x\"", "\"y\""}));
}

// S-m4: execute() submits with the group's owner when the options give none.
TEST(ExecGroup, ExecuteDefaultsTheOwner) {
  cvc::app app;
  exec_group::options o = opts_for(app, "owned");
  o.owner = "doc";
  exec_group g(o);
  g.execute(std::string("(begin (set i 0) (while #t (set i (+ i 1))))"));
  execute_options mine;
  mine.owner = "other";
  g.execute(std::string("(begin (set i 0) (while #t (set i (+ i 1))))"), mine);
  EXPECT_EQ(g.sched().kill_owner("doc"), 1);
  EXPECT_EQ(g.sched().kill_owner("other"), 1);
}

// S-m3, C-m5: deterministic mode reaches group 0 in exec_pump_frame: its math.random stream is
// seeded from a fixed name, so two apps draw the same numbers.
TEST(AppExecPool, DeterministicModeSeedsGroupZero) {
  const auto draw = [] {
    cvc::app app;
    cvc::pool_options po;
    po.deterministic = true;
    app.exec_pool(po);
    exec_group fg(opts_for(app, "frame"));
    cvc::frame_group_handle handle = app.register_frame_group(fg);
    async_scheduler &zero = app.exec_scheduler();
    const int pid = zero.execute(std::string("(list (math.random) (math.random))"));
    app.exec_pump_frame();
    const auto r = zero.get_result(pid);
    return r ? to_string(*r) : std::string("<none>");
  };
  const std::string a = draw(), b = draw();
  EXPECT_NE(a, "<none>");
  EXPECT_EQ(a, b);
}

// C-m9: destroying a group from inside its own pump aborts, rather than freeing the scheduler
// under its run loop.
TEST(ExecGroupDeathTest, DestroyedFromInsideItsOwnPumpAborts) {
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_DEATH(
      {
        cvc::app app;
        auto *g = new exec_group(opts_for(app, "doomed"));
        environment_ptr env = environment::extend(builtins::make_default_environment());
        builtins::register_fn(env, "die", [g](std::span<const value_t>) -> value_t {
          delete g;
          return nil_value();
        });
        execute_options eo;
        eo.env = env;
        g->execute(std::string("(die)"), eo);
        g->pump(kSteps);
      },
      "destroyed from inside its own pump");
}

// N3 (and C-m10): a deferred on_complete, or a host body, that destroys a group of its own
// pump_all aborts in every build, rather than leaving the call to use the freed group.
TEST(ExecGroupDeathTest, DestroyedFromACallbackOfItsCallAborts) {
  if (std::thread::hardware_concurrency() < 2)
    GTEST_SKIP() << "the case needs a pool with a worker";
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_DEATH(
      {
        cvc::pool_options po;
        po.threads = 1;
        cvc::exec_pool pool(po);
        cvc::app app;
        share_locks(app);
        auto *g = new exec_group(opts_for(app, "doomed"));
        execute_options eo;
        eo.on_complete = [g](value_t) { delete g; };
        g->execute(std::string("1"), eo);
        pool.pump_all(std::vector<exec_group *>{g}, kSteps, far_deadline());
      },
      "destroyed while a pump_all holds it");
}

TEST(ExecGroupDeathTest, DestroyedFromItsOwnHostBodyAborts) {
  if (std::thread::hardware_concurrency() < 2)
    GTEST_SKIP() << "the case needs a pool with a worker";
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_DEATH(
      {
        cvc::pool_options po;
        po.threads = 2;
        cvc::exec_pool pool(po);
        cvc::app app;
        share_locks(app);
        hold_caller hold(app);
        exec_group::options o = opts_for(app, "doomed");
        o.pump_hook_for_test = [&](exec_group &) { hold.go.store(true); };
        auto *g = new exec_group(o);
        environment_ptr env = environment::extend(builtins::make_default_environment());
        g->install(
            env, "proxied",
            [g](std::span<const value_t>) -> value_t {
              delete g;
              return nil_value();
            },
            provider_affinity::host_proxied);
        execute_options eo;
        eo.env = env;
        g->execute(std::string("(proxied)"), eo);
        for (int round = 0; round < 100; ++round) {
          hold.go.store(false);
          pool.set_last_pump_for_test(*hold.g, 1.0);
          pool.set_last_pump_for_test(*g, 1.0);
          pool.pump_all(std::vector<exec_group *>{hold.g.get(), g}, pump_budget{50, kInf},
                        far_deadline());
        }
      },
      "destroyed while a pump_all holds it");
}

// C-m11: the failed groups are the call's own: another thread's call, on the same pool or on
// another, returns none of them.
TEST(ExecPool, FailedGroupsAreTheCallersOwn) {
  cvc::app app;
  cvc::exec_pool pool(cvc::pool_options{});
  exec_group::options bad = opts_for(app, "bad");
  bad.pump_hook_for_test = [](exec_group &) { throw std::runtime_error("boom"); };
  exec_group g(bad), good(opts_for(app, "good"));
  std::vector<exec_group *> groups{&g};
  EXPECT_EQ(pool.pump_all(groups, kSteps, far_deadline()), groups);
  std::vector<exec_group *> just_good{&good};
  std::vector<exec_group *> seen{&g};
  std::thread([&] { seen = pool.pump_all(just_good, kSteps, far_deadline()); }).join();
  EXPECT_TRUE(seen.empty()) << "another thread's call returned this call's failed groups";
  cvc::exec_pool other(cvc::pool_options{});
  EXPECT_TRUE(other.pump_all(just_good, kSteps, far_deadline()).empty());
}

// One thread pumps two apps' pools in turn, in both orders: each pump_all returns only its own
// failed groups, and runs only its own groups' on_complete callbacks (on this thread, after its
// own join), whatever the other pool did on this thread just before.
TEST(ExecPool, OneThreadPumpsTwoAppsPoolsInTurn) {
  for (int order = 0; order < 2; ++order) {
    cvc::pool_options po;
    po.threads = 2;
    cvc::exec_pool pa(po), pb(po);
    cvc::app a, b;
    share_locks(a);
    share_locks(b);
    exec_group::options bad_a = opts_for(a, "bad-a"), bad_b = opts_for(b, "bad-b");
    bad_a.pump_hook_for_test = [](exec_group &) { throw std::runtime_error("boom a"); };
    bad_b.pump_hook_for_test = [](exec_group &) { throw std::runtime_error("boom b"); };
    exec_group ga(bad_a), gb(bad_b), oka(opts_for(a, "ok-a")), okb(opts_for(b, "ok-b"));
    std::mutex m;
    std::vector<std::string> heard;
    const std::thread::id caller = std::this_thread::get_id();
    const auto record = [&](const std::string &who) {
      execute_options o;
      o.on_complete = [&, who](value_t) {
        std::lock_guard<std::mutex> lock(m);
        heard.push_back(std::this_thread::get_id() == caller ? who : who + " off the caller");
      };
      return o;
    };
    oka.execute(std::string("(+ 1 1)"), record("a"));
    okb.execute(std::string("(+ 2 2)"), record("b"));
    std::vector<exec_group *> groups_a{&ga, &oka}, groups_b{&gb, &okb};
    cvc::exec_pool &first = order == 0 ? pa : pb;
    cvc::exec_pool &second = order == 0 ? pb : pa;
    std::vector<exec_group *> &first_groups = order == 0 ? groups_a : groups_b;
    std::vector<exec_group *> &second_groups = order == 0 ? groups_b : groups_a;
    const std::vector<exec_group *> failed_first =
        first.pump_all(first_groups, kSteps, far_deadline());
    std::vector<std::string> after_first;
    {
      std::lock_guard<std::mutex> lock(m);
      after_first = heard;
    }
    const std::vector<exec_group *> failed_second =
        second.pump_all(second_groups, kSteps, far_deadline());
    EXPECT_EQ(failed_first, std::vector<exec_group *>{first_groups[0]}) << "order " << order;
    EXPECT_EQ(failed_second, std::vector<exec_group *>{second_groups[0]}) << "order " << order;
    EXPECT_EQ(after_first, std::vector<std::string>{order == 0 ? "a" : "b"})
        << "the first pool ran only its own group's callback";
    std::lock_guard<std::mutex> lock(m);
    EXPECT_EQ(heard.size(), 2u);
  }
}

// A pump_all of one app's pool from inside a group of another app's, on one thread: each group's
// natives see only their own pool (a host_proxied body of the inner group runs inline on this
// thread, the inner pool's caller), and the outer group sees its own pool again afterwards.
TEST(ExecPool, ANestedPumpOfAnotherAppsPoolSeesOnlyItsOwnGroup) {
  cvc::pool_options po;
  po.threads = 2;
  cvc::exec_pool pa(po);
  cvc::exec_pool pb(cvc::pool_options{}); // no workers: its caller pumps its groups
  cvc::app a, b;
  share_locks(a);
  share_locks(b);
  exec_group ga(opts_for(a, "ga")), gb(opts_for(b, "gb"));
  std::atomic<int> wrong{0}, inner_runs{0}, nests{0};
  const std::thread::id caller = std::this_thread::get_id();
  environment_ptr eb = environment::extend(builtins::make_default_environment());
  builtins::register_fn(eb, "probe-b", [&](std::span<const value_t>) -> value_t {
    if (cvc::detail::worker_pool(gb) != nullptr) // pumped by pb's caller: never a worker of pa's
      wrong.fetch_add(1);
    inner_runs.fetch_add(1);
    return value_t(int64_t(1));
  });
  environment_ptr ea = environment::extend(builtins::make_default_environment());
  builtins::register_fn(ea, "nest", [&](std::span<const value_t>) -> value_t {
    const bool on_worker = std::this_thread::get_id() != caller;
    cvc::exec_pool *const mine = on_worker ? &pa : nullptr;
    if (cvc::detail::worker_pool(ga) != mine)
      wrong.fetch_add(1);
    std::vector<exec_group *> inner{&gb};
    if (!pb.pump_all(inner, kSteps, far_deadline()).empty())
      wrong.fetch_add(1);
    if (cvc::detail::worker_pool(ga) != mine) // the nested pump left this group's pool as it was
      wrong.fetch_add(1);
    nests.fetch_add(1);
    return value_t(int64_t(1));
  });
  execute_options oa, ob;
  oa.env = ea;
  ob.env = eb;
  for (int k = 0; k < 4; ++k)
    gb.execute(std::string("(probe-b)"), ob);
  ga.execute(std::string("(begin (nest) (nest))"), oa);
  std::vector<exec_group *> outer{&ga};
  pa.set_last_pump_for_test(ga, 1.0);
  for (int r = 0; r < 100 && nests.load() < 2; ++r)
    pa.pump_all(outer, kSteps, far_deadline());
  EXPECT_EQ(nests.load(), 2);
  EXPECT_EQ(inner_runs.load(), 4);
  EXPECT_EQ(wrong.load(), 0);
}

// Dispatch cost, a report rather than a check (run it with --gtest_also_run_disabled_tests): the
// best of 5 runs of one pump_all over 8 groups, idle or each running a short loop per pump, at 0
// and 3 workers, in microseconds per pump_all.
TEST(ExecPool, DISABLED_DispatchCostReport) {
  for (unsigned threads : {0u, 3u}) {
    cvc::pool_options po;
    po.threads = threads;
    cvc::exec_pool pool(po);
    cvc::app app;
    share_locks(app);
    std::vector<std::unique_ptr<exec_group>> owned;
    std::vector<exec_group *> groups;
    for (int i = 0; i < 8; ++i) {
      owned.push_back(std::make_unique<exec_group>(opts_for(app, "g" + std::to_string(i))));
      groups.push_back(owned.back().get());
    }
    const auto report = [&](const char *what, int reps) {
      double best = 1e30;
      for (int run = 0; run < 5; ++run) {
        const auto t0 = clock_type::now();
        for (int r = 0; r < reps; ++r)
          pool.pump_all(groups, pump_budget{200, std::numeric_limits<double>::infinity()},
                        far_deadline());
        best = std::min(
            best, std::chrono::duration<double, std::micro>(clock_type::now() - t0).count() / reps);
      }
      std::printf("pump_all threads %u, %-12s %10.2f us\n", threads, what, best);
    };
    report("idle", 5000);
    for (exec_group *g : groups)
      g->execute(std::string("(while #t (set i 0) (while (< i 50) (set i (+ i 1))))"));
    report("busy", 2000);
    for (exec_group *g : groups)
      for (const process_info &p : g->sched().list_processes())
        g->sched().kill(p.pid);
    pool.pump_all(groups, kSteps, far_deadline());
  }
}

// C-m11, sharper (the R1 skeptic's): a thread whose id was an ended thread's gets none of that
// thread's failed groups from its own call. It skips when no id is reused within 5000 threads.
TEST(ExecPool, FailedGroupsOfAReusedThreadId) {
  cvc::app app;
  cvc::exec_pool pool(cvc::pool_options{});
  exec_group::options bad = opts_for(app, "bad");
  bad.pump_hook_for_test = [](exec_group &) { throw std::runtime_error("boom"); };
  exec_group g(bad), good(opts_for(app, "good"));
  std::vector<exec_group *> groups{&g}, just_good{&good};
  std::thread::id first;
  std::thread([&] {
    first = std::this_thread::get_id();
    pool.pump_all(groups, kSteps, far_deadline());
  }).join();
  bool reused = false;
  int tries = 0;
  std::vector<exec_group *> seen;
  for (; tries < 5000 && !reused; ++tries)
    std::thread([&] {
      if (std::this_thread::get_id() == first) {
        reused = true;
        seen = pool.pump_all(just_good, kSteps, far_deadline());
      }
    }).join();
  std::cout << "id reused after " << tries << " threads: " << reused << "; saw " << seen.size()
            << " failed groups\n";
  if (!reused)
    GTEST_SKIP() << "no thread id was reused";
  EXPECT_TRUE(seen.empty()) << "a thread with a reused id read an ended thread's failed groups";
}

namespace {

// CVC_EXEC_FORCE_MOVE for the pools built while it lives (a pool reads the knob when it is built).
struct force_move_scope {
  force_move_scope() { set("1"); }
  ~force_move_scope() { set(nullptr); }
  static void set(const char *v) {
#ifdef _WIN32
    _putenv_s("CVC_EXEC_FORCE_MOVE", v ? v : "");
#else
    if (v)
      setenv("CVC_EXEC_FORCE_MOVE", v, 1);
    else
      unsetenv("CVC_EXEC_FORCE_MOVE");
#endif
  }
};

// How long the busy group's native keeps the pool's only worker.
constexpr auto kForceMoveNap = std::chrono::milliseconds(1000);

struct force_moved_call {
  unsigned threads = 0;
  bool returned = false;
  double seconds = 0.0;      // until the case saw it return (the busy worker's nap included)
  double call_seconds = 0.0; // the call's own pump_all, timed in its caller thread
  double busy_after = 0.0;   // how long the worker still napped once the call began
  bool skipped = false;      // the call's group was skipped (its pool link)
  int last_thread = -1;      // which thread pumped the call's group last (0: a caller)
  std::uint64_t skips = 0;   // stats().skipped across the call
};

std::int64_t now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(clock_type::now().time_since_epoch())
      .count();
}

// One CVC_EXEC_FORCE_MOVE pump_all of a group the caller pumped last, so its caller may not take
// it and hands it to the workers, while the pool's only worker is kept kForceMoveNap in another
// caller's group (S1-Q11). `tight`: the call's deadline is 50 ms away, so the worker reaches it
// after.
// Should pump_all never return (the hang this guards), the call's objects are leaked and its thread
// detached, so the case fails instead of hanging the binary.
force_moved_call run_force_moved_call(bool tight) {
  auto *app = new cvc::app;
  share_locks(*app); // groups pumped on workers, not kept on the caller (S1-Q1)
  cvc::exec_pool *pool = nullptr;
  {
    force_move_scope knob;
    cvc::pool_options po;
    po.threads = 1;
    pool = new cvc::exec_pool(po);
  }
  force_moved_call out;
  out.threads = pool->threads();
  auto napping = std::make_shared<std::atomic<bool>>(false);
  auto nap_end = std::make_shared<std::atomic<std::int64_t>>(0);
  auto *busy = new exec_group(opts_for(*app, "busy"));
  environment_ptr env = environment::extend(builtins::make_default_environment());
  builtins::register_fn(env, "nap", [napping, nap_end](std::span<const value_t>) -> value_t {
    napping->store(true);
    std::this_thread::sleep_for(kForceMoveNap);
    nap_end->store(now_ns());
    return value_t(int64_t(1));
  });
  execute_options eo;
  eo.env = env;
  busy->execute(std::string("(nap)"), eo);
  auto *light = new exec_group(opts_for(*app, "light"));
  busy->pool_link().last_thread = 0; // both last pumped by a caller (thread 0)
  light->pool_link().last_thread = 0;
  std::thread other([pool, busy] {
    std::vector<exec_group *> gs{busy};
    pool->pump_all(gs, kSteps, far_deadline());
  });
  while (!napping->load())
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  const auto before = pool->stats();
  auto returned = std::make_shared<std::atomic<bool>>(false);
  auto call_seconds = std::make_shared<std::atomic<double>>(0.0);
  auto call_start = std::make_shared<std::atomic<std::int64_t>>(0);
  const auto t0 = clock_type::now();
  std::thread caller([pool, light, tight, returned, call_seconds, call_start] {
    std::vector<exec_group *> gs{light};
    const auto c0 = clock_type::now();
    call_start->store(now_ns());
    pool->pump_all(gs, kSteps, tight ? c0 + std::chrono::milliseconds(50) : far_deadline());
    call_seconds->store(std::chrono::duration<double>(clock_type::now() - c0).count());
    returned->store(true);
  });
  other.join(); // the worker is free again kForceMoveNap in
  // A loaded machine may start the caller thread late, even after the nap (seen once): the
  // worker was then free for the call, and the case says nothing about the deadline.
  out.busy_after = static_cast<double>(nap_end->load() - call_start->load()) * 1e-9;
  while (!returned->load() && clock_type::now() - t0 < std::chrono::seconds(10))
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  out.returned = returned->load();
  out.seconds = std::chrono::duration<double>(clock_type::now() - t0).count();
  out.call_seconds = call_seconds->load();
  if (!out.returned) {
    caller.detach(); // still waiting in pump_all: everything it uses stays leaked
    return out;
  }
  caller.join();
  const auto after = pool->stats();
  out.skipped = light->pool_link().skipped;
  out.last_thread = light->pool_link().last_thread;
  out.skips = after.skipped - before.skipped;
  delete light;
  delete busy;
  delete pool;
  delete app;
  return out;
}

} // namespace

// R1b-mid's force-move hang: under CVC_EXEC_FORCE_MOVE a caller may not take a group it pumped
// last, so it waits for a worker. A worker that reached the call past its deadline skipped the
// group and told no one, and the caller waited for ever (exec_pump_frame and Ariadne's drain hung
// on a loaded machine). Now the caller waits no longer than its deadline and skips the group
// itself, and a worker that skips tells it. The call is timed in its own thread: it must end long
// before the busy worker is free (a worker-side skip alone would end it only then, R1b skeptic
// m7). Control: a far deadline waits for the worker and pumps the group. One-sided ceilings: 10 s
// for a return at all (rule 7), half the worker's nap for the tight call; the times are printed.
// The tight call counts only when the worker napped on for over half its nap after the call
// began (a caller thread started late on a loaded machine is tried again, up to five times).
// Should no try count, the case passes on its hang check alone, which every try made, with a
// note: a skip would print no "OK ]" and fail the sanitizer jobs' required list.
TEST(ExecPool, AForceMovedCallEndsAtItsDeadline) {
  const force_moved_call control = run_force_moved_call(false);
  std::cout << "far deadline: " << control.threads << " worker(s), returned " << control.returned
            << " after " << control.seconds << " s (the call " << control.call_seconds
            << " s), last thread " << control.last_thread << ", skips " << control.skips << "\n";
  if (control.threads != 1)
    GTEST_SKIP() << "needs a pool with one worker";
  ASSERT_TRUE(control.returned) << "pump_all with a far deadline did not return";
  EXPECT_FALSE(control.skipped);
  EXPECT_NE(control.last_thread, 0) << "the caller pumped a group it pumped last";
  EXPECT_EQ(control.skips, 0u);
  const double half_nap = std::chrono::duration<double>(kForceMoveNap).count() / 2;
  force_moved_call tight;
  for (int attempt = 1; attempt <= 5; ++attempt) {
    tight = run_force_moved_call(true);
    std::cout << "deadline 50 ms away: returned " << tight.returned << " after " << tight.seconds
              << " s (the call " << tight.call_seconds << " s, the worker busy " << tight.busy_after
              << " s after it began), last thread " << tight.last_thread << ", skips "
              << tight.skips << "\n";
    ASSERT_TRUE(tight.returned) << "pump_all did not return within 10 s: its caller still waits";
    if (tight.busy_after > half_nap + 0.1)
      break;
  }
  if (tight.busy_after <= half_nap + 0.1) {
    std::cout << "NOTE: the caller thread started too late in all five tries (a loaded "
                 "machine): the deadline was not judged; every call returned\n";
    return;
  }
  EXPECT_LT(tight.call_seconds, half_nap)
      << "the call ended when the busy worker came, not at its deadline";
  EXPECT_TRUE(tight.skipped) << "the group was not skipped at the deadline";
  EXPECT_EQ(tight.skips, 1u);
}

// --- Stage 2a: an attached group's teardown (S2a.5; S2-Q4) ---

// The includes the cases below add to the file's own.
#include <condition_variable>
#include <cvc/state/state_exec/async_host.h>
#include <functional>
#include <span>

namespace {

// A kernel that runs until its call is cancelled, calling `at_cancel` from the call's cancel hook
// (which the teardown's kill runs); it counts its starts.
async_kernel teardown_kernel(std::atomic<int> &started, std::function<void()> at_cancel) {
  return [&started, at_cancel](async_call &call) -> value_t {
    struct gate {
      std::mutex m;
      std::condition_variable cv;
      bool cancelled = false;
    };
    auto g = std::make_shared<gate>();
    call.on_cancel([g, at_cancel] {
      at_cancel();
      std::lock_guard<std::mutex> lock(g->m);
      g->cancelled = true;
      g->cv.notify_all();
    });
    started.fetch_add(1);
    std::unique_lock<std::mutex> lock(g->m);
    g->cv.wait_for(lock, std::chrono::seconds(30), [&] { return g->cancelled; });
    return value_t(int64_t(0));
  };
}

} // namespace

// S2a.5 (S2-Q4's order): an attached group destroyed with a kernel in flight is still attached at
// its teardown hook (once it holds the token and has closed its lock port), and has left the pool
// by its kills: in the kernel's cancel hook, which the kill runs, it is not attached and has no
// notifier. The pool counts nothing attached after.
TEST(ExecGroupTeardown, AnAttachedGroupLeavesItsPoolBeforeTheKills) {
  cvc::pool_options po;
  po.threads = 0;
  cvc::exec_pool pool(po);
  cvc::app app;
  std::atomic<int> started{0};
  bool attached_at_hook = false, attached_at_kill = true, notifier_at_kill = true;
  int cancels = 0;
  exec_group::options o = opts_for(app, "s2a5");
  o.teardown_hook_for_test = [&](exec_group &g) { attached_at_hook = pool.attached(g); };
  auto g = std::make_unique<exec_group>(std::move(o));
  exec_group *const raw = g.get();
  environment_ptr env = environment::extend(builtins::make_default_environment());
  register_async_fn(env, g->host(), "wait-cancel",
                    [&, raw](std::span<const value_t>) -> async_kernel {
                      return teardown_kernel(started, [&, raw] {
                        ++cancels;
                        attached_at_kill = pool.attached(*raw);
                        notifier_at_kill = raw->sched().has_wake_notifier();
                      });
                    });
  execute_options eo;
  eo.env = env;
  g->execute(std::string("(begin (wait-cancel) 1)"), eo);
  const auto t0 = clock_type::now();
  while (started.load() == 0 && clock_type::now() - t0 < std::chrono::seconds(10)) {
    g->pump(kSteps);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_EQ(started.load(), 1) << "the kernel never started";
  pool.attach(*g);
  ASSERT_TRUE(pool.attached(*g));
  g.reset();
  EXPECT_TRUE(attached_at_hook) << "it left the pool before its token and the port's close";
  EXPECT_EQ(cancels, 1) << "the teardown's kill did not cancel the call";
  EXPECT_FALSE(attached_at_kill) << "it was still attached at its kills";
  EXPECT_FALSE(notifier_at_kill) << "it still had its notifier at its kills";
  EXPECT_EQ(pool.stats().attached, 0u);
}

// S2a.5 (the amended transitions): destroying an attached group inside its own with_group, on the
// thread holding it, aborts in every build, as one inside its own pump does. Skipped until
// with_group is built for attached groups (S2a.4).
TEST(ExecGroupDeathTest, DestroyedInsideItsOwnWithGroupAborts) {
  {
    cvc::pool_options po;
    po.threads = 0;
    cvc::exec_pool pool(po);
    cvc::app app;
    exec_group g(opts_for(app, "probe"));
    pool.attach(g);
    bool built = true;
    try {
      pool.with_group(g, [] {});
    } catch (const std::logic_error &) {
      built = false;
    }
    pool.detach(g);
    if (!built)
      GTEST_SKIP() << "with_group is not built for attached groups yet (S2a.4)";
  }
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_DEATH(
      {
        cvc::pool_options po;
        po.threads = 0;
        cvc::exec_pool pool(po);
        cvc::app app;
        auto *g = new exec_group(opts_for(app, "doomed"));
        pool.attach(*g);
        pool.with_group(*g, [g] { delete g; });
      },
      "destroyed from inside its own with_group");
}

namespace {

// An attached group pumped by a pool's worker whose process calls (body), a host_proxied native, in
// a generator, which cannot park: the call runs as a waiting body on this, the attaching thread,
// in its host loop, under the blocked worker's token. The loop runs until `done()` or 10 s.
template <class Done>
void run_a_waiting_body(cvc::exec_pool &pool, exec_group &g, environment_ptr env, Done done) {
  execute_options eo;
  eo.env = std::move(env);
  g.execute(std::string("(begin (set gen (generator (lambda () (yield (body))))) (next gen))"), eo);
  pool.attach(g);
  const auto t0 = clock_type::now();
  while (!done() && clock_type::now() - t0 < std::chrono::seconds(10)) {
    pool.pump_host(0.01);
    pool.wait_for_work(0.01);
  }
}

} // namespace

// The waiting-body twin (the amended transitions, R2a C-M3): destroying an attached group inside
// its own waiting body, which its blocked worker waits for, aborts in every build. Skipped where
// the pool has no worker (the body would run inline). A waiting body is probed first:
// host_held_here must be true in it (K-7), or the teardown would wait for the body's worker for
// good.
TEST(ExecGroupDeathTest, DestroyedInsideItsOwnWaitingBodyAborts) {
  {
    cvc::pool_options po;
    po.threads = 1;
    cvc::exec_pool pool(po);
    if (pool.threads() == 0)
      GTEST_SKIP() << "the pool has no worker";
    cvc::app app;
    share_locks(app);
    exec_group g(opts_for(app, "probe"));
    environment_ptr env = environment::extend(builtins::make_default_environment());
    std::atomic<int> held{-1};
    bool waiting = false; // its worker held the token: a waiting body, not an inline call
    g.install(
        env, "body",
        [&](std::span<const value_t>) -> value_t {
          waiting = g.sched().pumped_elsewhere();
          held.store(cvc::detail::host_held_here(g) ? 1 : 0);
          return value_t(int64_t(1));
        },
        provider_affinity::host_proxied);
    run_a_waiting_body(pool, g, env, [&] { return held.load() >= 0; });
    pool.detach(g);
    ASSERT_GE(held.load(), 0) << "the waiting body never ran";
    ASSERT_TRUE(waiting) << "the body ran inline, not as a waiting body";
    ASSERT_EQ(held.load(), 1) << "host_held_here is false in the group's own waiting body";
  }
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_DEATH(
      {
        cvc::pool_options po;
        po.threads = 1;
        cvc::exec_pool pool(po);
        cvc::app app;
        share_locks(app);
        auto *g = new exec_group(opts_for(app, "doomed"));
        environment_ptr env = environment::extend(builtins::make_default_environment());
        g->install(
            env, "body",
            [g](std::span<const value_t>) -> value_t {
              delete g;
              return nil_value();
            },
            provider_affinity::host_proxied);
        run_a_waiting_body(pool, *g, env, [] { return false; });
      },
      "destroyed from inside its own with_group");
}

// --- Stage 2a continuous pool (td 8.3, 8.11) ---
//
// S2.md section 4's PL cases for continuous mode (S2a.9): the eventcount, a parked pool, the
// pool's timers, its virtual-time queue, a blocked worker and the long-step monitor, and the
// pool's teardown. The attaching thread runs exec_pool.h's host loop (pump_host, then
// wait_for_work when it had nothing to do); the cases assert counts, outcomes and thread ids, and
// print their times (rule 7).

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
#include <cvc/state/state_exec/async_host.h>
#include <functional>
#include <optional>
#include <random>
// clang-format on

namespace {

// The sanitizer jobs set CVC_LOCKS_SANITIZER=1: their builds run 5-15x slower, so the long cases
// run smaller there.
bool s2a_under_sanitizer() {
  const char *v = std::getenv("CVC_LOCKS_SANITIZER");
  return v && *v && std::string(v) != "0";
}

// An environment variable set while it lives (CVC_EXEC_FUZZ, which a pool and a lock manager read
// when they are built), and restored after.
class s2a_env_var {
public:
  s2a_env_var(const char *name, const char *value) : name_(name) {
    if (const char *old = std::getenv(name))
      old_ = old;
    set(value);
  }
  ~s2a_env_var() { set(old_ ? old_->c_str() : nullptr); }
  s2a_env_var(const s2a_env_var &) = delete;
  s2a_env_var &operator=(const s2a_env_var &) = delete;

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

double s2a_seconds_since(clock_type::time_point t0) {
  return std::chrono::duration<double>(clock_type::now() - t0).count();
}

// Spin (no sleep, whose granularity on Windows is a timer tick) for `us` microseconds.
void s2a_spin_us(int us) {
  const auto until = clock_type::now() + std::chrono::microseconds(us);
  while (clock_type::now() < until) {
  }
}

// The attaching thread's host loop (exec_pool.h): until done(), pump_host(pump_seconds), and
// wait_for_work(beat) when that had nothing to do. `in_loop`, when given, is set while the thread
// is inside either. A throw (S2a.0's stubs: "not built") or `ceiling` seconds passing fails the
// case; true when done() held.
bool s2a_drive(cvc::exec_pool &pool, const std::function<bool()> &done, double ceiling = 30.0,
               double beat = 0.01, double pump_seconds = kInf,
               std::atomic<bool> *in_loop = nullptr) {
  const auto t0 = clock_type::now();
  try {
    while (!done()) {
      if (s2a_seconds_since(t0) > ceiling) {
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

// A group with the intrinsics in an env of its own, chrooted at its name, built with `tweak`'s
// options (its test hooks). A host message reaches a program's (msg-recv "/x") as "x".
struct s2a_world {
  intrinsics_context ictx;
  environment_ptr env = environment::extend(builtins::make_default_environment());
  std::unique_ptr<exec_group> group;

  s2a_world(cvc::app &app, const std::string &name,
            const std::function<void(exec_group::options &)> &tweak = {},
            group_affinity affinity = group_affinity::any) {
    exec_group::options o = opts_for(app, name, affinity);
    if (tweak)
      tweak(o);
    group = std::make_unique<exec_group>(std::move(o));
    ictx.sched = &group->sched();
    ictx.proc = make_process();
    ictx.proc->pid = 0;
    apply_chroot(ictx, app.root(), name);
    ictx.host = &group->host();
    register_intrinsics(env, &ictx);
  }
  s2a_world(const s2a_world &) = delete;
  s2a_world &operator=(const s2a_world &) = delete;

  exec_group &g() { return *group; }
  // Before the group is attached, or under its token (with_group).
  int submit(const std::string &src, std::function<void(value_t)> on_complete = {}) {
    execute_options eo;
    eo.env = environment::extend(env);
    eo.on_complete = std::move(on_complete);
    return group->execute(src, eo);
  }
};

} // namespace

// PL-2 (td 8.3; S2-Q3, S2-Q15): a post between a worker's last look and its sleep, under
// CVC_EXEC_FUZZ (the pool's fuzz points yield in the eventcount's prepare and commit). Each stream
// is one attached group whose receiver notes message i, and a poster thread that posts message i+1
// once i is noted: at once, or after a spin of 10-40 us for one post in four (so the posts land
// across the worker's spin, prepare and commit), or after 200 us for one in 64 (the worker has
// parked). Every post's receiver runs: a lost wake leaves the receiver parked, which the per-post
// ceiling (10 s, the only time limit: a hang guard) turns into a failure. A fixed count, every post
// of it (posts are counted, not seconds: K-14's class): 5*10^5 in Release, about 4 s on a quiet
// box (2*10^4 in Debug, 10^4 under a sanitizer; CVC_EXEC_PL02_POSTS), over up to four streams and
// workers; the time is printed. With workers, the workers parked (stats().parks).
TEST(ExecPool, PL02_EventcountNeverMissesAWake) {
  s2a_env_var fuzz("CVC_EXEC_FUZZ", "2026");
  cvc::pool_options po;
  po.threads = 4;
  cvc::exec_pool pool(po); // reads CVC_EXEC_FUZZ now
  cvc::app app;
  share_locks(app);
  const unsigned streams = std::max(1u, std::min(4u, pool.threads()));
  const char *const sized = std::getenv("CVC_EXEC_PL02_POSTS");
  const std::uint64_t asked = sized && *sized         ? std::strtoull(sized, nullptr, 10)
                              : s2a_under_sanitizer() ? 10000
                              : kDebugBuild           ? 20000
                                                      : 500000;
  const std::uint64_t per_stream = (asked + streams - 1) / streams;
  const std::uint64_t total = per_stream * streams;
  struct stream {
    std::unique_ptr<s2a_world> w;
    std::atomic<std::int64_t> got{-1};
    std::uint64_t posted = 0;
    std::int64_t lost_at = -1; // the post whose receiver never ran
  };
  std::vector<std::unique_ptr<stream>> ss;
  for (unsigned s = 0; s < streams; ++s) {
    auto st = std::make_unique<stream>();
    const std::string name = "pl02s" + std::to_string(s);
    st->w = std::make_unique<s2a_world>(app, name);
    stream *raw = st.get();
    builtins::register_fn(st->w->env, "got", [raw](std::span<const value_t> a) -> value_t {
      raw->got.store(std::get<int64_t>(a[0].v), std::memory_order_release);
      return nil_value();
    });
    st->w->submit("(while #t (got (msg-recv \"/" + name + "\")))");
    ss.push_back(std::move(st));
  }
  s2a_attached att(pool);
  for (auto &st : ss)
    att.attach(st->w->g());
  std::atomic<bool> stop{false};
  std::atomic<unsigned> posters_done{0};
  const auto t0 = clock_type::now();
  std::vector<std::thread> posters;
  for (unsigned s = 0; s < streams; ++s)
    posters.emplace_back([&, s] {
      stream &st = *ss[s];
      std::mt19937 rng(1000 + s);
      std::uniform_int_distribution<int> spin(10, 40);
      const std::string key = "pl02s" + std::to_string(s);
      for (std::uint64_t i = 0; i < per_stream && !stop.load(); ++i) {
        if (i % 64 == 63)
          s2a_spin_us(200);
        else if (rng() % 4 == 0)
          s2a_spin_us(spin(rng));
        st.w->g().sched().post_message(key, value_t(static_cast<int64_t>(i)));
        const auto p0 = clock_type::now();
        while (st.got.load(std::memory_order_acquire) < static_cast<std::int64_t>(i) &&
               !stop.load()) {
          if (s2a_seconds_since(p0) > 10.0) {
            st.lost_at = static_cast<std::int64_t>(i);
            stop.store(true);
            break;
          }
          std::this_thread::yield();
        }
        if (st.got.load(std::memory_order_acquire) < static_cast<std::int64_t>(i))
          break;
        st.posted = i + 1;
      }
      posters_done.fetch_add(1);
    });
  // No overall ceiling: each post has its own (10 s), and every poster ends.
  const bool drove = s2a_drive(pool, [&] { return posters_done.load() == streams; }, kInf, 0.005);
  stop.store(true);
  for (std::thread &t : posters)
    t.join();
  const double took = s2a_seconds_since(t0);
  att.detach_all();
  std::uint64_t posted = 0;
  for (auto &st : ss)
    posted += st->posted;
  const cvc::pool_stats s = pool.stats();
  std::printf("PL-2: %llu of %llu posts over %u stream(s) at threads %u in %.3f s; wakes %llu, "
              "parks %llu, empty wakes %llu\n",
              static_cast<unsigned long long>(posted), static_cast<unsigned long long>(total),
              streams, pool.threads(), took, static_cast<unsigned long long>(s.wakes),
              static_cast<unsigned long long>(s.parks),
              static_cast<unsigned long long>(s.empty_wakes));
  ASSERT_TRUE(drove);
  for (unsigned i = 0; i < streams; ++i)
    EXPECT_EQ(ss[i]->lost_at, -1) << "stream " << i << ": post " << ss[i]->lost_at
                                  << "'s receiver never ran (a lost wake)";
  EXPECT_EQ(posted, total) << "a stream stopped early (above)";
  if (pool.threads() > 0)
    EXPECT_GT(s.parks, 0u) << "no worker parked between the posts";
}

// PL-3 (td 8.3; S2-Q3): everything parked, then a host thread posts a message: the receiver runs.
// "Parked": the receiver waits on msg-recv, no worker is pumping (stats().active_workers) and the
// workers have parked (stats().parks), then 20 ms more; at threads 0 the attaching thread waits in
// wait_for_work. The time from the post to the receiver's step is printed, not asserted.
TEST(ExecPool, PL03_AHostPostWakesAParkedPool) {
  cvc::pool_options po;
  po.threads = 2;
  cvc::exec_pool pool(po);
  cvc::app app;
  share_locks(app);
  std::atomic<bool> ready{false}, stop{false}, posted{false};
  std::atomic<std::int64_t> got_ns{0}, post_ns{0};
  std::atomic<std::uint64_t> parks_at_post{0};
  s2a_world w(app, "pl03");
  builtins::register_fn(w.env, "ready", [&](std::span<const value_t>) -> value_t {
    ready.store(true);
    return nil_value();
  });
  builtins::register_fn(w.env, "got", [&](std::span<const value_t>) -> value_t {
    got_ns.store(now_ns());
    return nil_value();
  });
  const int pid = w.submit("(begin (ready) (got (msg-recv \"/pl03\")) \"received\")");
  s2a_attached att(pool);
  att.attach(w.g());
  std::thread poster([&] {
    const auto t0 = clock_type::now();
    while (!stop.load() && s2a_seconds_since(t0) < 30.0) {
      const cvc::pool_stats s = pool.stats();
      if (ready.load() && (pool.threads() == 0 || (s.parks > 0 && s.active_workers == 0)))
        break;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (stop.load() || s2a_seconds_since(t0) >= 30.0)
      return;
    std::this_thread::sleep_for(std::chrono::milliseconds(20)); // past any worker's spin
    parks_at_post.store(pool.stats().parks);
    post_ns.store(now_ns());
    w.g().sched().post_message("pl03", value_t(std::string("hello")));
    posted.store(true);
  });
  const bool drove = s2a_drive(pool, [&] { return got_ns.load() != 0; }, 30.0);
  stop.store(true);
  poster.join();
  att.detach_all();
  std::printf("PL-3: threads %u, parks before the post %llu; the receiver ran %.1f us after it\n",
              pool.threads(), static_cast<unsigned long long>(parks_at_post.load()),
              posted.load() ? static_cast<double>(got_ns.load() - post_ns.load()) / 1e3 : -1.0);
  ASSERT_TRUE(drove);
  EXPECT_TRUE(posted.load()) << "the pool never looked parked";
  EXPECT_EQ(result_of(w.g(), pid), "\"received\"");
  if (pool.threads() > 0)
    EXPECT_GT(parks_at_post.load(), 0u) << "posted before any worker parked";
}

// PL-4 (td 8.3; S2-Q3, S2-Q4): one attached group whose one process sleeps 5 ms, nothing else, at
// threads 2. The group is pumped once after the sleep began (the pump hook counts the pumps begun
// between the process entering the sleep and its end), that pump finds the sleep elapsed, and no
// worker wakeup found nothing to pump (stats().empty_wakes 0: no worker woke early, nor two at the
// deadline). The time from the deadline to the pump is printed.
TEST(ExecPool, PL04_ASleeperIsPumpedAtItsDeadline) {
  cvc::pool_options po;
  po.threads = 2;
  cvc::exec_pool pool(po);
  cvc::app app;
  share_locks(app);
  std::atomic<std::int64_t> slept_at{0}, first_after{0};
  std::atomic<int> after{0}, pumps{0};
  std::atomic<bool> woke{false}, completed{false};
  s2a_world w(app, "pl04", [&](exec_group::options &o) {
    o.pump_hook_for_test = [&](exec_group &) {
      pumps.fetch_add(1);
      const std::int64_t s = slept_at.load();
      if (s != 0 && !woke.load() && after.fetch_add(1) == 0)
        first_after.store(now_ns() - s);
    };
  });
  builtins::register_fn(w.env, "mark", [&](std::span<const value_t>) -> value_t {
    slept_at.store(now_ns());
    return nil_value();
  });
  builtins::register_fn(w.env, "woke", [&](std::span<const value_t>) -> value_t {
    woke.store(true);
    return nil_value();
  });
  const int pid = w.submit("(begin (mark) (sleep 0.005) (woke) \"woke\")",
                           [&](value_t) { completed.store(true); });
  s2a_attached att(pool);
  att.attach(w.g());
  const bool drove = s2a_drive(pool, [&] { return completed.load(); }, 30.0);
  const cvc::pool_stats s = pool.stats();
  att.detach_all();
  std::printf(
      "PL-4: threads %u, %d pumps, %d after the sleep began, the first %.3f ms after it; "
      "parks %llu, timer wakes %llu, empty wakes %llu\n",
      pool.threads(), pumps.load(), after.load(), static_cast<double>(first_after.load()) / 1e6,
      static_cast<unsigned long long>(s.parks), static_cast<unsigned long long>(s.timer_wakes),
      static_cast<unsigned long long>(s.empty_wakes));
  ASSERT_TRUE(drove);
  EXPECT_EQ(result_of(w.g(), pid), "\"woke\"");
  EXPECT_EQ(after.load(), 1) << "not pumped exactly once between the sleep and its end";
  EXPECT_GE(first_after.load(), 5000000) << "pumped before the sleep's deadline";
  EXPECT_EQ(s.empty_wakes, 0u) << "a worker woke and found nothing to pump";
}

// PL-5 (td 8.3; S2-Q13): a CPU-heavy group (always runnable: each pump takes its whole budget) and
// a light one (a receiver of short host messages) on one worker. A host thread posts message i
// once message i-1 was taken. The light group's wake-to-run latency, counted in heavy pumps begun
// from the post's return to the receiver taking it, is at most 1 (the heavy pump in progress as the
// post landed): a woken light group goes before the heavy one (virtual time, the clamp), so the
// light group is pumped at least once per heavy pump while it has work. Counts from the pump hooks;
// the pump counts and the latencies are printed.
TEST(ExecPool, PL05_ALightGroupIsNotStarved) {
  cvc::pool_options po;
  po.threads = 1;
  cvc::exec_pool pool(po);
  if (pool.threads() == 0)
    GTEST_SKIP() << "no worker on this machine";
  cvc::app app;
  share_locks(app);
  std::atomic<std::uint64_t> heavy_pumps{0}, light_pumps{0};
  s2a_world heavy(app, "pl05heavy", [&](exec_group::options &o) {
    o.pump_hook_for_test = [&](exec_group &) { heavy_pumps.fetch_add(1); };
  });
  s2a_world light(app, "pl05light", [&](exec_group::options &o) {
    o.pump_hook_for_test = [&](exec_group &) { light_pumps.fetch_add(1); };
  });
  const int n = s2a_under_sanitizer() || kDebugBuild ? 100 : 400;
  std::vector<std::uint64_t> at_post(n, 0), at_take(n, 0);
  std::atomic<int> taken{-1};
  builtins::register_fn(light.env, "took", [&](std::span<const value_t> a) -> value_t {
    const int i = static_cast<int>(std::get<int64_t>(a[0].v));
    if (i >= 0 && i < n)
      at_take[i] = heavy_pumps.load();
    taken.store(i, std::memory_order_release);
    return nil_value();
  });
  heavy.submit("(begin (set i 0) (while #t (set i (+ i 1))))");
  light.submit("(while #t (took (msg-recv \"/pl05\")))");
  s2a_attached att(pool);
  att.attach(heavy.g());
  att.attach(light.g());
  std::atomic<bool> stop{false}, all_posted{false};
  int lost_at = -1;
  std::thread poster([&] {
    const auto t0 = clock_type::now();
    while (heavy_pumps.load() < 2 && !stop.load() && s2a_seconds_since(t0) < 30.0)
      std::this_thread::yield();
    for (int i = 0; i < n && !stop.load(); ++i) {
      light.g().sched().post_message("pl05", value_t(static_cast<int64_t>(i)));
      at_post[i] = heavy_pumps.load();
      const auto p0 = clock_type::now();
      while (taken.load(std::memory_order_acquire) < i && !stop.load()) {
        if (s2a_seconds_since(p0) > 10.0) {
          lost_at = i;
          stop.store(true);
          break;
        }
        std::this_thread::yield();
      }
    }
    all_posted.store(!stop.load());
    stop.store(true);
  });
  const bool drove = s2a_drive(pool, [&] { return stop.load(); }, 120.0);
  stop.store(true);
  poster.join();
  att.detach_all();
  std::int64_t worst = 0;
  std::vector<int> spread(4, 0); // latencies 0, 1, 2, more
  for (int i = 0; i < n && i <= taken.load(); ++i) {
    const std::int64_t late =
        std::max<std::int64_t>(0, static_cast<std::int64_t>(at_take[i] - at_post[i]));
    worst = std::max(worst, late);
    ++spread[static_cast<std::size_t>(std::min<std::int64_t>(late, 3))];
  }
  std::printf(
      "PL-5: %d messages; heavy pumps %llu, light pumps %llu; latency in heavy pumps: 0 x%d, "
      "1 x%d, 2 x%d, more x%d\n",
      taken.load() + 1, static_cast<unsigned long long>(heavy_pumps.load()),
      static_cast<unsigned long long>(light_pumps.load()), spread[0], spread[1], spread[2],
      spread[3]);
  ASSERT_TRUE(drove);
  EXPECT_EQ(lost_at, -1) << "message " << lost_at << " was never taken";
  EXPECT_TRUE(all_posted.load());
  EXPECT_LE(worst, 1) << "the light group waited out more than the heavy pump in progress";
  EXPECT_GT(heavy_pumps.load(), 2u) << "the heavy group was not pumped throughout";
}

// PL-9 (td 8.3, 4.9; S2-Q11): in one group a native blocks its worker 100 ms inline (an async
// function on the inline executor), with two busy groups beside it, at threads 2: the busy groups
// keep being pumped while it blocks (their pump counts move between the kernel's start and end),
// and the long-step monitor, on for the groups a pool with workers pumps, counts the step
// (long_steps()) and reports it to the scheduler's sink under the native's name.
TEST(ExecPool, PL09_ABlockedWorkerBlocksOnlyItsGroup) {
  cvc::pool_options po;
  po.threads = 2;
  cvc::exec_pool pool(po);
  if (pool.threads() < 2)
    GTEST_SKIP() << "needs two workers";
  cvc::app app;
  share_locks(app);
  std::atomic<std::uint64_t> pumps_a{0}, pumps_b{0};
  s2a_world a(app, "pl09a", [&](exec_group::options &o) {
    o.pump_hook_for_test = [&](exec_group &) { pumps_a.fetch_add(1); };
  });
  s2a_world b(app, "pl09b", [&](exec_group::options &o) {
    o.pump_hook_for_test = [&](exec_group &) { pumps_b.fetch_add(1); };
  });
  s2a_world blocker(app, "pl09block");
  std::atomic<std::uint64_t> a_start{0}, b_start{0}, a_end{0}, b_end{0};
  async_options inline_call;
  inline_call.where = executor::inline_call();
  inline_call.future_variant = false;
  register_async_fn(
      blocker.env, blocker.g().host(), "pl09-block",
      [&](std::span<const value_t>) -> async_kernel {
        return [&](async_call &) -> value_t {
          a_start.store(pumps_a.load());
          b_start.store(pumps_b.load());
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          a_end.store(pumps_a.load());
          b_end.store(pumps_b.load());
          return value_t(int64_t(1));
        };
      },
      inline_call);
  std::mutex m;
  std::vector<std::pair<std::string, double>> reported;
  blocker.g().sched().set_long_step_sink([&](const std::string &native, double seconds) {
    std::lock_guard<std::mutex> lock(m);
    reported.emplace_back(native, seconds);
  });
  std::atomic<bool> completed{false};
  a.submit("(begin (set i 0) (while #t (set i (+ i 1))))");
  b.submit("(begin (set i 0) (while #t (set i (+ i 1))))");
  const int pid = blocker.submit("(begin (for i (range 10) nil) (pl09-block) \"unblocked\")",
                                 [&](value_t) { completed.store(true); });
  s2a_attached att(pool);
  att.attach(a.g());
  att.attach(b.g());
  att.attach(blocker.g());
  const bool drove = s2a_drive(pool, [&] { return completed.load(); }, 30.0);
  att.detach_all();
  const std::uint64_t caught = blocker.g().sched().long_steps();
  std::vector<std::pair<std::string, double>> seen;
  {
    std::lock_guard<std::mutex> lock(m);
    seen = reported;
  }
  std::printf("PL-9: while the native blocked, group a pumped %llu times and b %llu; long steps "
              "%llu, reported %zu\n",
              static_cast<unsigned long long>(a_end.load() - a_start.load()),
              static_cast<unsigned long long>(b_end.load() - b_start.load()),
              static_cast<unsigned long long>(caught), seen.size());
  for (const auto &[native, seconds] : seen)
    std::printf("PL-9: long step: %s, %.1f ms\n", native.c_str(), seconds * 1e3);
  ASSERT_TRUE(drove);
  EXPECT_EQ(result_of(blocker.g(), pid), "\"unblocked\"");
  EXPECT_GT(a_end.load(), a_start.load()) << "group a was not pumped while the worker blocked";
  EXPECT_GT(b_end.load(), b_start.load()) << "group b was not pumped while the worker blocked";
  EXPECT_GE(caught, 1u) << "the long-step monitor counted nothing";
  EXPECT_TRUE(std::any_of(seen.begin(), seen.end(), [](const auto &r) {
    return r.first == "pl09-block";
  })) << "the sink never heard the native's name";
}

// PL-10 (td 8.3; S2-Q5): eight busy groups attached to a pool of eight workers, each detached while
// the workers re-pump it, the groups destroyed, then the pool: no group is pumped after its detach,
// the groups are gone (detail::live_groups), ~exec_pool joins its workers (the process's thread
// count falls by about the workers and is back to about where it was before the pool), and the
// handle count is flat. The counts are one-sided with a small slack, and printed; ASan in CI checks
// for leaks and uses after free.
TEST(ExecPool, PL10_DestroyingABusyPool) {
  cvc::app app;
  share_locks(app);
  const long threads_before = s2a_thread_count(), handles_before = s2a_handle_count();
  cvc::pool_options po;
  po.threads = 8;
  auto pool = std::make_unique<cvc::exec_pool>(po);
  const unsigned workers = pool->threads();
  long threads_with_pool = -1;
  bool drove = false;
  std::vector<std::uint64_t> at_detach, later;
  {
    constexpr int kGroups = 8;
    std::vector<std::unique_ptr<std::atomic<std::uint64_t>>> pumps;
    std::vector<std::unique_ptr<s2a_world>> worlds;
    for (int k = 0; k < kGroups; ++k) {
      pumps.push_back(std::make_unique<std::atomic<std::uint64_t>>(0));
      std::atomic<std::uint64_t> *count = pumps.back().get();
      worlds.push_back(std::make_unique<s2a_world>(
          app, "pl10g" + std::to_string(k), [count](exec_group::options &o) {
            o.pump_hook_for_test = [count](exec_group &) { count->fetch_add(1); };
          }));
      worlds.back()->submit("(begin (set i 0) (while #t (set i (+ i 1))))");
    }
    s2a_attached att(*pool);
    for (auto &w : worlds)
      att.attach(w->g());
    threads_with_pool = s2a_thread_count();
    drove = s2a_drive(*pool, [&] {
      return std::all_of(pumps.begin(), pumps.end(), [](const auto &c) { return c->load() >= 3; });
    });
    for (int k = 0; k < kGroups; ++k) {
      pool->detach(worlds[k]->g());
      at_detach.push_back(pumps[k]->load());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    for (int k = 0; k < kGroups; ++k)
      later.push_back(pumps[k]->load());
    att.detach_all();
  } // the groups go, detached
  EXPECT_EQ(detail::live_groups(app), 0u);
  EXPECT_EQ(pool->stats().attached, 0u);
  pool.reset(); // joins
  const long threads_after = s2a_thread_count(), handles_after = s2a_handle_count();
  std::printf("PL-10: %u workers; threads %ld before the pool, %ld with it, %ld after; handles "
              "%ld before, %ld after\n",
              workers, threads_before, threads_with_pool, threads_after, handles_before,
              handles_after);
  ASSERT_TRUE(drove);
  EXPECT_EQ(later, at_detach) << "a group was pumped after its detach";
  if (threads_before >= 0 && threads_after >= 0 && threads_with_pool >= 0) {
    EXPECT_LE(threads_after, threads_before + 2) << "the pool's workers were not joined";
    EXPECT_GE(threads_with_pool - threads_after, static_cast<long>(workers) - 2);
  }
  if (handles_before >= 0 && handles_after >= 0)
    EXPECT_LE(handles_after, handles_before + 64) << "handles leaked";
}
