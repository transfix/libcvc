// exec_group and exec_pool unit tests (threaded execution Stage 1, S1.md S1.5-S1.8): the pool's
// own mechanics (placement, clamps, the helper wake, the deadline, failures, on_complete, the
// host-call queue, several callers), the group's (teardown, the AST copy, install by kind,
// wants_pump) and the app's (exec_pool(), the lock-mode hint, exec_pump_frame, pids). The Groups
// suite (state_exec_groups_test.cpp) holds the plan's GR/DT/... cases. No case name contains
// "Stress": run.sh filters those out.

#include <atomic>
#include <chrono>
#include <cvc/core/app.h>
#include <cvc/core/cpu_placement.h>
#include <cvc/core/exec_pool.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/exec_group.h>
#include <cvc/state/state_exec/heap.h>
#include <cvc/state/state_exec/lock_manager.h>
#include <cvc/state/state_exec/types.h>
#include <gtest/gtest.h>
#include <iostream>
#include <limits>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace cvc::state_exec;
using clock_type = std::chrono::steady_clock;

class collect_cycles_at_exit : public ::testing::Environment {
public:
  void TearDown() override { collect_cycles(); }
};
[[maybe_unused]] ::testing::Environment *const g_collect_cycles =
    ::testing::AddGlobalTestEnvironment(new collect_cycles_at_exit);

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
// top-level set writes the env's outermost unfrozen scope, which two groups must never share.
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
  thread_log any_log, host_log;
  std::vector<std::unique_ptr<exec_group>> owned;
  std::vector<exec_group *> groups;
  for (int i = 0; i < 6; ++i) {
    owned.push_back(std::make_unique<exec_group>(opts_for(
        app, "g" + std::to_string(i), i == 0 ? group_affinity::host : group_affinity::any)));
    groups.push_back(owned.back().get());
  }
  const std::uint64_t total_before = cvc::exec_pool_worker_pumps_total();
  std::vector<std::pair<exec_group *, int>> pids;
  for (auto &g : owned) {
    execute_options eo;
    eo.env = env_with_tid(g->opts().affinity == group_affinity::host ? host_log : any_log);
    pids.emplace_back(g.get(), g->execute(counting(20000), eo));
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
  EXPECT_EQ(cvc::exec_pool_worker_pumps_total() - total_before, s.worker_pumps);
  EXPECT_EQ(host_log.distinct(), std::set<std::thread::id>{std::this_thread::get_id()})
      << "a host-affinity group is pumped by the caller only";
  EXPECT_GT(any_log.distinct().size(), 1u) << "workers pumped the any groups too";
}

TEST(ExecPool, TheHelperWakeFollowsTheLastPumps) {
  cvc::pool_options po;
  po.threads = 2;
  cvc::exec_pool pool(po);
  cvc::app app;
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
    std::vector<std::string> order;
    std::vector<std::unique_ptr<exec_group>> owned;
    std::vector<exec_group *> groups;
    for (int i = 0; i < 3; ++i) {
      exec_group::options o = opts_for(app, "g" + std::to_string(i));
      o.pump_hook_for_test = [&order](exec_group &g) { order.push_back(g.opts().name); };
      owned.push_back(std::make_unique<exec_group>(o));
      groups.push_back(owned.back().get());
    }
    pool.pump_all(groups, kSteps, clock_type::now() - std::chrono::seconds(1));
    EXPECT_TRUE(order.empty()) << "nothing starts past the deadline";
    EXPECT_EQ(pool.stats().skipped, 3u);
    // g1 alone is pumped, then skipped g0 and g2 go before it.
    std::vector<exec_group *> one{owned[1].get()};
    pool.pump_all(one, kSteps, far_deadline());
    pool.pump_all(groups, kSteps, far_deadline());
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
    exec_group::options bad_opts = opts_for(app, "bad");
    bad_opts.pump_hook_for_test = [](exec_group &) { throw std::runtime_error("boom"); };
    exec_group bad(bad_opts), good(opts_for(app, "good"));
    const int bad_pid = bad.execute(std::string("(begin (sleep 10) 1)"));
    const int good_pid = good.execute(std::string("(+ 40 2)"));
    std::vector<exec_group *> groups{&bad, &good};
    pool.set_last_pump_for_test(bad, 1.0);
    pool.set_last_pump_for_test(good, 1.0);
    EXPECT_NO_THROW(pool.pump_all(groups, kSteps, far_deadline()));
    EXPECT_TRUE(bad.failed());
    EXPECT_FALSE(good.failed());
    EXPECT_EQ(pool.failed_groups(), std::vector<exec_group *>{&bad});
    const auto info = bad.sched().get_process_info(bad_pid);
    ASSERT_TRUE(info);
    EXPECT_EQ(info->status, process_status::killed) << "threads " << threads;
    EXPECT_EQ(bad.sched().get_exit_error(bad_pid).value_or(""), "internal error");
    ASSERT_TRUE(finished(good, good_pid));
    EXPECT_EQ(result_of(good, good_pid), "42");
    // Failed groups are per call: a later pump_all without one reports none.
    std::vector<exec_group *> just_good{&good};
    pool.pump_all(just_good, kSteps, far_deadline());
    EXPECT_TRUE(pool.failed_groups().empty());
  }
}

TEST(ExecPool, OnCompleteOfAWorkerPumpedGroupRunsOnTheCallerAfterTheJoin) {
  cvc::pool_options po;
  po.threads = 2;
  cvc::exec_pool pool(po);
  if (pool.threads() == 0)
    GTEST_SKIP() << "no worker on this machine";
  cvc::app app;
  exec_group a(opts_for(app, "a")), b(opts_for(app, "b"));
  std::vector<exec_group *> groups{&a, &b};
  std::mutex m;
  std::vector<std::pair<std::string, std::thread::id>> calls;
  bool in_pump_all_seen = false;
  const auto record = [&](const std::string &what) {
    return [&, what](value_t) {
      std::lock_guard<std::mutex> lock(m);
      calls.emplace_back(what, std::this_thread::get_id());
      in_pump_all_seen = in_pump_all_seen || pool.in_pump_all();
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
  bool ran = false;
  pool.run_on_host([&] { ran = true; });
  EXPECT_TRUE(ran);
  EXPECT_FALSE(pool.in_pump_all());
  EXPECT_EQ(cvc::detail::worker_pool(), nullptr);
}

TEST(ExecGroup, InstallByKind) {
  cvc::pool_options po;
  po.threads = 2;
  cvc::exec_pool pool(po);
  cvc::app app;
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
  EXPECT_EQ(detail::live_groups(&app), 0u);
  auto g = std::make_unique<exec_group>(opts_for(app, "g"));
  EXPECT_EQ(detail::live_groups(&app), 1u);
  EXPECT_FALSE(g->wants_pump()) << "no process, no ingress, no timer";
  const int pid = g->execute(std::string("(+ 1 2)"));
  EXPECT_TRUE(g->wants_pump()) << "a runnable process";
  g->pump(kSteps);
  ASSERT_TRUE(finished(*g, pid));
  EXPECT_FALSE(g->wants_pump());
  g->sched().post_message("box", value_t(int64_t(1)));
  EXPECT_TRUE(g->wants_pump()) << "pending ingress";
  g.reset();
  EXPECT_EQ(detail::live_groups(&app), 0u);
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
  builtins::register_fn(env, "seen", [&seen](std::span<const value_t>) -> value_t {
    seen = exec_group::current();
    return nil_value;
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
  EXPECT_EQ(exec_group::current(), nullptr);
}

TEST(ExecPool, TwoCallersShareThePool) {
  cvc::pool_options po;
  po.threads = 2;
  cvc::exec_pool pool(po);
  cvc::app app;
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
