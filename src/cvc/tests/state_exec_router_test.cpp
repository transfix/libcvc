// Stage 2's router and process control across groups (docs/STATE_EXEC_THREADED.md 4.5, 4.6, 8.4;
// S2.md S2b, section 4: MS-1..MS-12, PL-6..PL-8, KO-3, RT-1, DT-3, DT-4).
#include <cvc/core/app.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/exec_group.h>
#include <cvc/state/state_exec/exec_router.h>
#include <gtest/gtest.h>

namespace se = cvc::state_exec;

namespace {

// S2b.0: the handles and the registrations, which S2b.0 builds (the rest are its stubs).
TEST(RouterInterfaces, AClosedHandleRefusesEveryPost) {
  se::group_ref none;
  EXPECT_FALSE(none);
  EXPECT_TRUE(none.closed());
  EXPECT_FALSE(none.post_message("k", se::value_t(std::string("x"))));

  se::async_scheduler sched;
  const se::group_ref ref = se::group_ref::make(sched, nullptr);
  ASSERT_TRUE(ref);
  EXPECT_TRUE(ref.is_group0());
  EXPECT_FALSE(ref.closed());
  bool visited = false;
  EXPECT_TRUE(ref.visit([&](se::async_scheduler &s) { visited = &s == &sched; }));
  EXPECT_TRUE(visited);
  ref.close();
  EXPECT_TRUE(ref.closed());
  EXPECT_FALSE(ref.post_message("k", se::value_t(std::string("x"))));
  EXPECT_FALSE(ref.visit([](se::async_scheduler &) {}));
}

TEST(RouterInterfaces, TheLongestRegisteredPrefixIsTheHome) {
  cvc::app app;
  se::exec_router &router = app.exec_router();
  EXPECT_FALSE(router.active());
  se::exec_group::options a;
  a.name = "a";
  a.app = &app;
  a.channel_prefixes = {"docs.a"};
  se::exec_group ga(a);
  se::exec_group::options ab;
  ab.name = "ab";
  ab.app = &app;
  ab.channel_prefixes = {"docs.a.b"};
  se::exec_group gab(ab);
  EXPECT_TRUE(router.active());
  EXPECT_EQ(router.home_of("docs.a.x"), ga.route());
  EXPECT_EQ(router.home_of("docs.a"), ga.route());
  EXPECT_EQ(router.home_of("docs.a.b.c"), gab.route());
  EXPECT_EQ(router.home_of("docs.ab"), router.home_of("other")); // not under "docs.a"
  EXPECT_TRUE(router.home_of("other").is_group0());              // group 0, built for it
}

// S2b.0a: a teardown's forward skips closed homes; a plain group is an island; broadcast ids are
// unique in the app.
TEST(RouterInterfaces, ATeardownForwardsPastClosedHomes) {
  cvc::app app;
  se::exec_router &router = app.exec_router();
  se::exec_group::options a;
  a.name = "a";
  a.app = &app;
  a.channel_prefixes = {"docs.a"};
  se::exec_group ga(a);
  se::exec_group::options ab;
  ab.name = "ab";
  ab.app = &app;
  ab.channel_prefixes = {"docs.a.b"};
  se::exec_group gab(ab);
  se::exec_group::options plain;
  plain.name = "plain";
  plain.app = &app;
  se::exec_group gp(plain);
  EXPECT_TRUE(router.has_prefix(ga.route()));
  EXPECT_FALSE(router.has_prefix(gp.route())); // an island
  EXPECT_TRUE(router.has_prefix(router.home_of("other")));
  gab.route().close();                                  // as its teardown's first step does
  EXPECT_EQ(router.home_of("docs.a.b.c"), gab.route()); // posters wait for the unregister
  EXPECT_EQ(router.new_home_of("docs.a.b.c"), ga.route());
  ga.route().close();
  EXPECT_TRUE(router.new_home_of("docs.a.b.c").is_group0());
  const std::uint64_t b1 = router.new_broadcast_id();
  EXPECT_NE(b1, 0u);
  EXPECT_NE(router.new_broadcast_id(), b1);
}

} // namespace

// --- C's unit tests (S2b.2, S2b.3, S2b.5) ---
// The directory, published info, the owner rule and pool.kill_owner. The directory's writes at
// create and reap and the publish at each run loop's end are R's calls in async_scheduler.cpp: a
// case that needs one says so and is skipped until they are in.

#include <atomic>
#include <chrono>
#include <cvc/core/exec_pool.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/process_table.h>
#include <cvc/state/state_exec/scheduler.h>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <vector>

namespace c_tests {

namespace se = cvc::state_exec;
using clock_type = std::chrono::steady_clock;

constexpr double kNoTime = std::numeric_limits<double>::infinity();
constexpr se::pump_budget kRun{100000, kNoTime}; // steps only (S2.md section 4's oracles)
constexpr const char *kWaits = "(begin (msg-recv \"never\") 1)";
constexpr const char *kSpins = "(begin (set i 0) (while #t (set i (+ i 1))))";

void quiet_rates(cvc::app &app) {
  app.root()("state_exec.defaults.limits.async_steps_per_cpu_ms").value(std::string("0"));
}

// Programs with the intrinsics on `sched`, chrooted at `name` on the app's tree. The scheduler's
// owner (a group) goes first, then the environment and the context its natives point at.
struct programs {
  se::intrinsics_context ictx;
  se::environment_ptr env = se::environment::extend(se::builtins::make_default_environment());
  programs(cvc::app &app, se::scheduler_base &sched, const std::string &name) {
    ictx.sched = &sched;
    ictx.proc = se::make_process();
    ictx.proc->pid = 0;
    se::apply_chroot(ictx, app.root(), name);
    se::register_intrinsics(env, &ictx);
  }
  se::execute_options opts(const std::string &owner) const {
    se::execute_options eo;
    eo.env = se::environment::extend(env);
    eo.owner = owner;
    return eo;
  }
};

// An exec_group of `app` with its programs (the group, a base made first, goes first too).
struct group_holder {
  std::unique_ptr<se::exec_group> group;
};
struct group_world : group_holder, programs {
  group_world(cvc::app &app, const std::string &name,
              const std::function<void(se::exec_group::options &)> &tweak = {})
      : group_holder{make(app, name, tweak)}, programs(app, group->sched(), name) {}
  ~group_world() { group.reset(); }
  se::exec_group &g() { return *group; }
  se::async_scheduler &sched() { return group->sched(); }
  int submit(const std::string &src, const std::string &owner) {
    return group->execute(src, opts(owner));
  }

private:
  static std::unique_ptr<se::exec_group>
  make(cvc::app &app, const std::string &name,
       const std::function<void(se::exec_group::options &)> &tweak) {
    se::exec_group::options o;
    o.name = name;
    o.app = &app;
    if (tweak)
      tweak(o);
    return std::make_unique<se::exec_group>(std::move(o));
  }
};

// Group 0 (app.exec_scheduler()) with its programs, pumped as exec_pump_frame pumps it.
struct group0_world : programs {
  explicit group0_world(cvc::app &app) : programs(app, app.exec_scheduler(), "g0"), app(app) {}
  cvc::app &app;
  se::async_scheduler &sched() { return app.exec_scheduler(); }
  int submit(const std::string &src, const std::string &owner) {
    return sched().execute(src, opts(owner));
  }
  void pump() {
    se::async_scheduler::pump_owner_scope token(sched());
    sched().drain_ingress();
    sched().wake_awaiting();
    sched().sync_run_steps(kRun.steps, std::nullopt);
  }
};

// R's directory write at execute (S2b.4) when it is in, else the entry it would make.
void listed(cvc::app &app, int pid, const se::group_ref &group, const std::string &owner) {
  se::exec_directory &d = app.exec_router().directory();
  if (!d.find(pid))
    d.add(pid, group, owner);
}

se::process_status status_of(se::async_scheduler &s, int pid) {
  const auto info = s.get_process_info(pid);
  return info ? info->status : se::process_status::terminated;
}

template <class Sched> std::string result_of(Sched &s, int pid) {
  const auto info = s.get_process_info(pid);
  if (!info || info->status != se::process_status::terminated)
    return "<" + std::string(info ? se::to_string(info->status) : "gone") + ">";
  const std::optional<se::value_t> r = s.get_result(pid);
  if (!r)
    return "<none>";
  if (const auto *text = std::get_if<std::string>(&r->v))
    return *text;
  return se::to_string(*r);
}

// --- S2b.2 the directory ---

TEST(ExecDirectory, AddFindRemoveAndGroupsOfAnOwner) {
  cvc::app app;
  group_world a(app, "a"), b(app, "b");
  const se::group_ref zero = app.exec_scheduler().route_self();
  ASSERT_TRUE(zero);
  se::exec_directory d;
  EXPECT_EQ(d.size(), 0u);
  // 300 pids over the three groups and two owners and none, in every shard.
  for (int pid = 1000; pid < 1300; ++pid) {
    const se::group_ref &g = pid % 3 == 0 ? b.g().route() : pid % 3 == 1 ? a.g().route() : zero;
    d.add(pid, g, pid % 2 == 0 ? "X" : pid % 5 == 0 ? "" : "Y");
  }
  EXPECT_EQ(d.size(), 300u);
  const std::optional<se::directory_entry> e = d.find(1002), f = d.find(1003);
  ASSERT_TRUE(e.has_value());
  ASSERT_TRUE(f.has_value());
  EXPECT_EQ(e->pid, 1002);
  EXPECT_EQ(e->group, b.g().route());
  EXPECT_EQ(e->owner, "X");
  EXPECT_EQ(f->group, a.g().route());
  EXPECT_EQ(f->owner, "Y");
  EXPECT_FALSE(d.find(999).has_value());
  // Each group once, in group id order: group 0 first, then a and b as they were made.
  const std::vector<se::group_ref> x = d.groups_of("X");
  ASSERT_EQ(x.size(), 3u);
  EXPECT_TRUE(x[0].is_group0());
  EXPECT_EQ(x[1], a.g().route());
  EXPECT_EQ(x[2], b.g().route());
  EXPECT_TRUE(d.groups_of("nobody").empty());
  // Removing every pid of group a's: X's groups are 0 and b now.
  for (int pid = 1000; pid < 1300; ++pid)
    if (pid % 3 == 1)
      d.remove(pid);
  d.remove(999); // unknown: nothing
  EXPECT_EQ(d.size(), 200u);
  EXPECT_FALSE(d.find(1003).has_value());
  EXPECT_TRUE(d.find(1002).has_value());
  const std::vector<se::group_ref> left = d.groups_of("X");
  ASSERT_EQ(left.size(), 2u);
  EXPECT_TRUE(left[0].is_group0());
  EXPECT_EQ(left[1], b.g().route());
}

// Threads adding, finding and removing disjoint pids at once (the shards' mutexes; TSan reads it).
TEST(ExecDirectory, ThreadsShareTheShards) {
  cvc::app app;
  group_world a(app, "a");
  se::exec_directory d;
  constexpr int kThreads = 4, kEach = 2000;
  std::vector<std::thread> threads;
  std::atomic<int> missing{0};
  for (int t = 0; t < kThreads; ++t)
    threads.emplace_back([&, t] {
      for (int i = 0; i < kEach; ++i) {
        const int pid = t * kEach + i + 1;
        d.add(pid, a.g().route(), "T" + std::to_string(t));
        if (!d.find(pid))
          ++missing;
        if (i % 2 == 0)
          d.remove(pid);
      }
    });
  for (std::thread &t : threads)
    t.join();
  EXPECT_EQ(missing.load(), 0);
  EXPECT_EQ(d.size(), static_cast<std::size_t>(kThreads * kEach / 2));
  EXPECT_EQ(d.groups_of("T2").size(), 1u);
}

// A pid is in the directory from its creation (execute, fork) to its reap, so one made since its
// group's last publish is found (S2-Q8). Needs R's writes at create and reap.
TEST(ExecDirectory, APidIsFoundBetweenItsCreationAndItsGroupsFirstPublish) {
  cvc::app app;
  quiet_rates(app);
  group_world a(app, "a");
  const int pid = a.submit(kWaits, "X");
  const std::optional<se::directory_entry> e = app.exec_router().directory().find(pid);
  if (!e)
    GTEST_SKIP() << "the directory's write at execute is R's call (S2b.4), not in this build";
  EXPECT_EQ(e->group, a.g().route());
  EXPECT_EQ(e->owner, "X");
  EXPECT_EQ(a.sched().published().publishes, 0u) << "found before any publish";
  EXPECT_FALSE(app.exec_router().info(pid).has_value()) << "no row before the first publish";
  // A fork's child, from the host before any step, is listed too, with its parent's owner.
  const int child = a.sched().fork(pid);
  ASSERT_GE(child, 0);
  const std::optional<se::directory_entry> c = app.exec_router().directory().find(child);
  ASSERT_TRUE(c.has_value());
  EXPECT_EQ(c->owner, "X");
  // Reaped, both leave it.
  a.g().pump(kRun);
  EXPECT_TRUE(a.sched().kill(pid));
  EXPECT_TRUE(a.sched().kill(child));
  EXPECT_TRUE(a.sched().reap(pid));
  EXPECT_TRUE(a.sched().reap(child));
  EXPECT_FALSE(app.exec_router().directory().find(pid).has_value());
  EXPECT_FALSE(app.exec_router().directory().find(child).has_value());
}

// --- S2b.3 published info ---

// process_table::generation() moves at every insert, erase, status change and priority change,
// and not between ready and running (no index changes then, and a publish never sees running).
TEST(PublishedInfo, TheGenerationMovesWhereARowWouldChange) {
  se::detail::process_table table;
  std::uint64_t g = table.generation();
  const auto moved = [&] {
    const bool m = table.generation() != g;
    g = table.generation();
    return m;
  };
  se::process_ptr p = se::make_process();
  p->pid = 1;
  se::process &proc = *table.insert(std::move(p));
  EXPECT_TRUE(moved()) << "insert";
  table.set_status(proc, se::process_status::running);
  EXPECT_FALSE(moved()) << "ready -> running";
  table.set_status(proc, se::process_status::ready);
  EXPECT_FALSE(moved()) << "running -> ready";
  proc.recv_path = std::string("k");
  table.set_status(proc, se::process_status::waiting);
  EXPECT_TRUE(moved()) << "a park";
  proc.recv_path.reset();
  table.set_status(proc, se::process_status::ready);
  EXPECT_TRUE(moved()) << "a wake";
  proc.priority = 5;
  table.update(proc);
  EXPECT_TRUE(moved()) << "a priority change";
  table.set_status(proc, se::process_status::killed);
  EXPECT_TRUE(moved()) << "an end";
  EXPECT_TRUE(table.erase(1));
  EXPECT_TRUE(moved()) << "an erase";
  EXPECT_FALSE(table.erase(1));
  EXPECT_FALSE(moved()) << "nothing erased";
  EXPECT_TRUE(table.check());
}

// A row is make_info's process, field for field, and the summary the run loop's. Needs R's call.
TEST(PublishedInfo, ARowIsTheProcessAsMakeInfoReadsIt) {
  cvc::app app;
  quiet_rates(app);
  group_world a(app, "a");
  const int waiter = a.submit("(with-lock \"k\" (msg-recv \"in\"))", "X");
  const int spinner = a.submit(kSpins, "X");
  a.g().pump(se::pump_budget{500, kNoTime});
  if (a.sched().published().publishes == 0)
    GTEST_SKIP() << "the publish at the run loop's end is R's call (S2b.4), not in this build";
  ASSERT_EQ(status_of(a.sched(), waiter), se::process_status::waiting);
  const std::optional<se::published_process> row = a.sched().published_info(waiter);
  const std::optional<se::process_info> info = a.sched().get_process_info(waiter);
  ASSERT_TRUE(row.has_value());
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(row->pid, waiter);
  EXPECT_EQ(row->name, info->name);
  EXPECT_EQ(row->owner, "X");
  EXPECT_EQ(row->uid, info->uid);
  EXPECT_EQ(row->scope, info->scope);
  EXPECT_EQ(row->parent_pid, info->parent_pid);
  EXPECT_EQ(row->status, se::process_status::waiting);
  EXPECT_EQ(row->base_priority, info->priority);
  EXPECT_EQ(row->effective_priority, info->effective_priority);
  EXPECT_EQ(row->floor, info->priority_floor);
  EXPECT_EQ(row->locks_held, std::vector<std::string>{"/a.k"});
  EXPECT_EQ(info->locks_held, 1u);
  EXPECT_EQ(row->waits_on, info->waiting_on.value_or(""));
  EXPECT_EQ(row->waits_on, "msg " + se::resolve_channel_key("a", "in"));
  EXPECT_EQ(row->steps, info->step_count);
  // Every row in pid order; the summary from the table (no notifier: a fork-join group).
  const std::vector<se::published_process> rows = a.sched().published_rows();
  ASSERT_EQ(rows.size(), 2u);
  EXPECT_EQ(rows[0].pid, waiter);
  EXPECT_EQ(rows[1].pid, spinner);
  const se::async_scheduler::published_summary s = a.sched().published();
  EXPECT_TRUE(s.runnable);
  EXPECT_EQ(s.best_runnable, std::optional<int>(0));
  EXPECT_EQ(s.processes, 2);
  EXPECT_EQ(s.next_deadline, clock_type::time_point::max());
  // The router's and the pool's reads: the directory's group, its published row.
  listed(app, waiter, a.g().route(), "X");
  const std::optional<se::published_process> via = app.exec_pool().info(waiter);
  ASSERT_TRUE(via.has_value());
  EXPECT_EQ(via->status, se::process_status::waiting);
  EXPECT_FALSE(app.exec_pool().info(999999).has_value());
  EXPECT_TRUE(a.sched().kill(spinner));
  EXPECT_TRUE(a.sched().kill(waiter));
}

// A hold taken or dropped without a wait moves no status, so not the table's generation: the
// publish compares the lock mirrors with its rows. Needs R's call.
TEST(PublishedInfo, AHoldTakenOrDroppedWithoutAWaitIsPublished) {
  cvc::app app;
  quiet_rates(app);
  group_world a(app, "a");
  // Counts to 200, holds k while it counts to 400, then spins.
  const int pid = a.submit("(begin (set i 0) (while (< i 200) (set i (+ i 1)))"
                           " (with-lock \"k\" (while (< i 400) (set i (+ i 1))))"
                           " (while #t (set i (+ i 1))))",
                           "");
  const auto row = [&] { return a.sched().published_info(pid); };
  const auto held = [&] {
    const auto info = a.sched().get_process_info(pid);
    return info ? info->locks_held : std::size_t(0);
  };
  a.g().pump(se::pump_budget{20, kNoTime});
  if (a.sched().published().publishes == 0)
    GTEST_SKIP() << "the publish at the run loop's end is R's call (S2b.4), not in this build";
  ASSERT_TRUE(row().has_value());
  ASSERT_EQ(held(), 0u);
  EXPECT_TRUE(row()->locks_held.empty());
  // Pump in small steps to the hold, then past it: each publish shows what the mirror has.
  bool saw_hold = false, saw_release = false;
  for (int n = 0; n < 2000 && !saw_release; ++n) {
    a.g().pump(se::pump_budget{20, kNoTime});
    ASSERT_EQ(status_of(a.sched(), pid), se::process_status::ready);
    const std::size_t h = held();
    ASSERT_EQ(row()->locks_held.size(), h) << "pump " << n;
    if (h == 1) {
      EXPECT_EQ(row()->locks_held[0], "/a.k");
      saw_hold = true;
    } else if (saw_hold) {
      saw_release = true;
    }
  }
  EXPECT_TRUE(saw_hold);
  EXPECT_TRUE(saw_release);
  EXPECT_TRUE(a.sched().kill(pid));
}

// Rows are rebuilt only when the generation moved, and their counters otherwise refreshed at most
// every 100 ms, read by counts: across many quick pumps of a spinning process, the published steps
// change at most once per 100 ms of the loop's own time (plus the first build), however slow the
// machine; after a change that moves the generation the next publish is exact; and with nothing
// moving, a refresh still comes. Needs R's call.
TEST(PublishedInfo, RowsAreRebuiltOnGenerationMovesAndCountersRefreshEvery100Ms) {
  cvc::app app;
  quiet_rates(app);
  group_world a(app, "a");
  const int spinner = a.submit(kSpins, "");
  const int waiter = a.submit(kWaits, "");
  const auto published_steps = [&] { return a.sched().published_info(spinner)->steps; };
  const auto actual_steps = [&] { return a.sched().get_process_info(spinner)->step_count; };
  a.g().pump(se::pump_budget{20, kNoTime});
  if (a.sched().published().publishes == 0)
    GTEST_SKIP() << "the publish at the run loop's end is R's call (S2b.4), not in this build";
  EXPECT_EQ(published_steps(), actual_steps()) << "the first publish builds the rows";

  constexpr int kPumps = 2000;
  const std::uint64_t publishes0 = a.sched().published().publishes;
  int changes = 0;
  std::uint64_t last = published_steps();
  const auto t0 = clock_type::now();
  for (int n = 0; n < kPumps; ++n) {
    a.g().pump(se::pump_budget{20, kNoTime});
    const std::uint64_t now = published_steps();
    if (now != last)
      ++changes;
    last = now;
  }
  const double seconds = std::chrono::duration<double>(clock_type::now() - t0).count();
  EXPECT_EQ(a.sched().published().publishes - publishes0, static_cast<std::uint64_t>(kPumps))
      << "one publish per run loop";
  const int ceiling = static_cast<int>(seconds / 0.1) + 1;
  std::cout << "PublishedInfo: " << kPumps << " pumps in " << seconds * 1e3 << " ms, the published"
            << " steps changed " << changes << " times (at most " << ceiling << ")\n";
  EXPECT_LE(changes, ceiling) << "counters refreshed more often than every 100 ms";

  // A generation move (a host's set_priority of the waiter) rebuilds at the next publish: exact.
  for (int k = 0; k < 5; ++k) {
    ASSERT_TRUE(a.sched().set_priority(waiter, k + 1));
    a.g().pump(se::pump_budget{20, kNoTime});
    EXPECT_EQ(published_steps(), actual_steps()) << "rebuild " << k;
    EXPECT_EQ(a.sched().published_info(waiter)->base_priority, k + 1);
  }

  // With nothing moving, a refresh comes (the event, with a hang guard only): exact when it does.
  last = published_steps();
  const auto t1 = clock_type::now();
  bool refreshed = false;
  while (!refreshed && clock_type::now() - t1 < std::chrono::seconds(30)) {
    a.g().pump(se::pump_budget{20, kNoTime});
    if (published_steps() != last) {
      refreshed = true;
      EXPECT_EQ(published_steps(), actual_steps());
    }
  }
  EXPECT_TRUE(refreshed) << "the counters were never refreshed";
  EXPECT_TRUE(a.sched().kill(spinner));
  EXPECT_TRUE(a.sched().kill(waiter));
}

// --- S2b.5 the owner rule and pool.kill_owner ---

// Locally (S2-Q8): an owned caller sees and controls only its owner's processes; an unowned one,
// every process, as before. Both schedulers (the sync one has scheduler_base's default).
template <class Sched> void local_owner_rule(const char *kind) {
  cvc::app app;
  quiet_rates(app);
  Sched sched;
  programs w(app, sched, "s");
  const auto run = [&] {
    if constexpr (std::is_same_v<Sched, se::async_scheduler>) {
      sched.drain_ingress();
      sched.sync_run_steps(kRun.steps, std::nullopt);
    } else {
      sched.run(kRun.steps);
    }
  };
  const int mine = sched.execute(std::string(kWaits), w.opts("X"));
  const int theirs = sched.execute(std::string(kWaits), w.opts("Y"));
  const int spinner = sched.execute(std::string(kSpins), w.opts("Y"));
  run();
  const auto probe = [&](int target) {
    const std::string t = std::to_string(target);
    return "(str-concat (if (inspect " + t + ") \"seen\" \"none\") \" \" (if (pause " + t +
           ") \"t\" \"f\") (if (resume " + t + ") \"t\" \"f\") (if (kill " + t + ") \"t\" \"f\"))";
  };
  // X's caller: its own process seen, killed; Y's spinner unknown (none, f f f) and untouched.
  const int x = sched.execute("(str-concat (str (length (ps))) \" \" " + probe(spinner) +
                                  " \" \" " + probe(mine) + ")",
                              w.opts("X"));
  run();
  EXPECT_EQ(result_of(sched, x), "2 none fff seen fft") << kind;
  const auto st = [&](int pid) { return sched.get_process_info(pid)->status; };
  EXPECT_EQ(st(spinner), se::process_status::ready) << kind;
  EXPECT_EQ(st(theirs), se::process_status::waiting) << kind;
  EXPECT_EQ(st(mine), se::process_status::killed) << kind;
  // An unowned caller, as before: every row (the four so far, and itself), pause and resume of the
  // spinner, the kill of Y's waiter.
  const int u = sched.execute("(str-concat (str (length (ps))) \" \" " + probe(spinner) +
                                  " \" \" " + probe(theirs) + ")",
                              w.opts(""));
  run();
  EXPECT_EQ(result_of(sched, u), "5 seen ttt seen fft") << kind;
  EXPECT_EQ(st(spinner), se::process_status::killed) << kind;
  EXPECT_EQ(st(theirs), se::process_status::killed) << kind;
}

TEST(OwnerRule, LocalControlIsFilteredForAnOwnedCaller) {
  local_owner_rule<se::async_scheduler>("async");
  local_owner_rule<se::scheduler>("sync");
}

// Across groups (S2-Q8): a caller's kill, pause and resume of a pid the directory gives its own
// owner in another group are posted there and give #t; another owner's, an unowned caller's or an
// unknown pid's give #f and post nothing. Group 0 is a group as any other.
TEST(OwnerRule, AcrossGroupsTheCallersOwnerIsPosted) {
  cvc::app app;
  quiet_rates(app);
  group_world a(app, "a"), b(app, "b");
  group0_world zero(app);
  const int waiter = a.submit(kWaits, "X");
  const int spinner = a.submit(kSpins, "X");
  const int other = a.submit(kWaits, "Y");
  const int in0 = zero.submit(kWaits, "X");
  a.g().pump(kRun);
  zero.pump();
  listed(app, waiter, a.g().route(), "X");
  listed(app, spinner, a.g().route(), "X");
  listed(app, other, a.g().route(), "Y");
  listed(app, in0, app.exec_scheduler().route_self(), "X");
  const auto tf = [](const std::string &call) { return "(if " + call + " \"t\" \"f\")"; };
  const auto pid = [](int p) { return std::to_string(p); };
  // Refused: Y's caller on X's pids, an unowned caller, X on Y's pid, an unknown pid.
  const int y =
      b.submit("(str-concat " + tf("(kill " + pid(waiter) + ")") +
                   tf("(pause " + pid(spinner) + ")") + tf("(kill " + pid(in0) + ")") + ")",
               "Y");
  const int none = b.submit(
      "(str-concat " + tf("(kill " + pid(waiter) + ")") + tf("(kill " + pid(in0) + ")") + ")", "");
  const int x_bad =
      b.submit("(str-concat " + tf("(kill " + pid(other) + ")") + tf("(kill 999999)") + ")", "X");
  b.g().pump(kRun);
  EXPECT_EQ(result_of(b.sched(), y), "fff");
  EXPECT_EQ(result_of(b.sched(), none), "ff");
  EXPECT_EQ(result_of(b.sched(), x_bad), "ff");
  EXPECT_FALSE(a.sched().has_pending_control()) << "a refused call posted";
  EXPECT_FALSE(app.exec_scheduler().has_pending_control()) << "a refused call posted";
  // X's caller: posted, #t ("requested"), applied at the target's next pump.
  const int x = b.submit("(str-concat " + tf("(pause " + pid(spinner) + ")") +
                             tf("(kill " + pid(waiter) + ")") + tf("(kill " + pid(in0) + ")") + ")",
                         "X");
  b.g().pump(kRun);
  EXPECT_EQ(result_of(b.sched(), x), "ttt");
  EXPECT_TRUE(a.sched().has_pending_control());
  a.g().pump(se::pump_budget{1, kNoTime});
  zero.pump();
  EXPECT_EQ(status_of(a.sched(), spinner), se::process_status::paused);
  EXPECT_EQ(status_of(a.sched(), waiter), se::process_status::killed);
  EXPECT_EQ(a.sched().get_process_info(waiter)->exit_error.value_or(""), "killed_by_user");
  EXPECT_EQ(status_of(a.sched(), other), se::process_status::waiting);
  EXPECT_EQ(status_of(app.exec_scheduler(), in0), se::process_status::killed);
  // A resume is #t even where the target would say #f; this one applies.
  const int r = b.submit("(str-concat " + tf("(resume " + pid(spinner) + ")") +
                             tf("(resume " + pid(other) + ")") + ")",
                         "X");
  b.g().pump(kRun);
  EXPECT_EQ(result_of(b.sched(), r), "tf");
  a.g().pump(se::pump_budget{1, kNoTime});
  EXPECT_NE(status_of(a.sched(), spinner), se::process_status::paused);
  EXPECT_TRUE(a.sched().kill(spinner));
  EXPECT_TRUE(a.sched().kill(other));
}

// pool.kill_owner's world: groups a, b and c and group 0, each with a waiting and a spinning
// process of X and one of Y, listed in the directory.
struct kill_world {
  cvc::app app;
  std::vector<std::unique_ptr<group_world>> groups;
  std::unique_ptr<group0_world> zero;
  struct proc {
    se::async_scheduler *sched;
    int pid;
    std::string owner;
  };
  std::vector<proc> procs;

  explicit kill_world(unsigned threads) {
    quiet_rates(app);
    app.root()("state_exec.locks.threading").value(std::string("shared"));
    cvc::pool_options po;
    po.threads = threads;
    po.pump_steps = 200;
    app.exec_pool(po);
    for (const char *name : {"a", "b", "c"})
      groups.push_back(std::make_unique<group_world>(app, name));
    zero = std::make_unique<group0_world>(app);
    for (auto &g : groups)
      add(g->sched(), g->g().route(),
          [&](const char *src, const char *owner) { return g->submit(src, owner); });
    add(zero->sched(), app.exec_scheduler().route_self(),
        [&](const char *src, const char *owner) { return zero->submit(src, owner); });
  }
  ~kill_world() {
    groups.clear();
    zero.reset();
  }
  void add(se::async_scheduler &s, const se::group_ref &route,
           const std::function<int(const char *, const char *)> &submit) {
    for (const auto &[src, owner] :
         {std::pair{kWaits, "X"}, std::pair{kSpins, "X"}, std::pair{kWaits, "Y"}}) {
      const int pid = submit(src, owner);
      listed(app, pid, route, owner);
      procs.push_back({&s, pid, owner});
    }
  }
  cvc::exec_pool &pool() { return app.exec_pool(); }
  // Every X process killed by the command (owner_reaped), every Y process live.
  void expect_killed(const std::string &where) {
    for (const proc &p : procs) {
      const auto info = p.sched->get_process_info(p.pid);
      ASSERT_TRUE(info.has_value()) << where;
      if (p.owner == "X") {
        EXPECT_EQ(info->status, se::process_status::killed) << where << " pid " << p.pid;
        EXPECT_EQ(info->exit_error.value_or(""), "owner_reaped") << where << " pid " << p.pid;
      } else {
        EXPECT_FALSE(se::is_finished(info->status)) << where << " pid " << p.pid;
      }
    }
  }
};

// Fork-join (td 4.5): nothing pumps the groups or group 0, so kill_owner drains each inline and
// returns with every one of the owner's processes killed, at threads 0, 1 and 2.
TEST(PoolKillOwner, ForkJoinDrainsEachGroupInline) {
  for (const unsigned threads : {0u, 1u, 2u}) {
    kill_world w(threads);
    std::vector<se::exec_group *> gs;
    for (auto &g : w.groups)
      gs.push_back(&g->g());
    w.pool().pump_all(gs, kRun, clock_type::time_point::max());
    w.zero->pump();
    w.pool().kill_owner("X");
    w.expect_killed("threads " + std::to_string(threads));
    w.pool().kill_owner("X"); // none left live: each group applies it again, killing nothing
    w.pool().kill_owner("nobody");
    w.expect_killed("again, threads " + std::to_string(threads));
  }
}

// Continuous (S2-Q8): groups attached to this thread are drained under with_group; one attached
// to another thread is waited for (its pumps apply the command), as is group 0 when that thread
// holds its pump owner. At threads 0, 1 and 2.
TEST(PoolKillOwner, ContinuousWaitsForEachGroup) {
  for (const unsigned threads : {0u, 1u, 2u}) {
    kill_world w(threads);
    cvc::exec_pool &pool = w.pool();
    se::exec_group &c = w.groups[2]->g();
    pool.attach(w.groups[0]->g());
    pool.attach(w.groups[1]->g());
    // c is attached to, and pumped (or serviced) by, another thread until the end.
    std::atomic<bool> attached{false}, stop{false};
    std::thread other([&] {
      pool.attach(c);
      attached = true;
      while (!stop.load())
        if (!pool.pump_host(kNoTime))
          pool.wait_for_work(0.002);
      pool.detach(c);
    });
    while (!attached.load())
      std::this_thread::yield();
    pool.kill_owner("X");
    stop = true;
    other.join();
    pool.detach(w.groups[0]->g());
    pool.detach(w.groups[1]->g());
    w.expect_killed("threads " + std::to_string(threads));
  }
}

// KO-3's shape (C's own case, at each mode): a group torn down while kill_owner waits on it ends
// the wait: its teardown holds its token (so kill_owner cannot drain it) until the command is in
// its ingress; its destructor then refuses the command (done(false)), and kill_owner returns with
// the other groups' processes killed.
TEST(PoolKillOwner, AGroupTornDownMeanwhileEndsTheWait) {
  for (const bool continuous : {false, true}) {
    kill_world w(1);
    cvc::exec_pool &pool = w.pool();
    std::atomic<bool> in_teardown{false};
    // The dying group: d, with one X process, its teardown hook waiting for the command.
    auto d = std::make_unique<group_world>(w.app, "d", [&](se::exec_group::options &o) {
      o.teardown_hook_for_test = [&](se::exec_group &g) {
        in_teardown = true;
        const auto t0 = clock_type::now();
        while (!g.sched().has_pending_control() &&
               clock_type::now() - t0 < std::chrono::seconds(30))
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
      };
    });
    const int dpid = d->submit(kWaits, "X");
    listed(w.app, dpid, d->g().route(), "X");
    std::thread owner_thread([&, dd = d.release()]() mutable {
      std::unique_ptr<group_world> mine(dd);
      if (continuous) {
        pool.attach(mine->g());
        pool.pump_host(kNoTime);
      }
      mine.reset(); // the teardown (detaching it first when attached)
    });
    while (!in_teardown.load())
      std::this_thread::yield();
    pool.kill_owner("X");
    owner_thread.join();
    w.expect_killed(continuous ? "continuous" : "fork-join");
  }
}

// Refused from inside a pump, host body or callback of the pool (std::logic_error): here a native
// of a group pumped by pump_all, on its caller (threads 0).
TEST(PoolKillOwner, RefusedFromInsideAPumpOfThePool) {
  kill_world w(0);
  std::string seen;
  group_world &a = *w.groups[0];
  se::builtins::register_fn(a.env, "ko", [&](std::span<const se::value_t>) -> se::value_t {
    try {
      w.pool().kill_owner("Y");
      seen = "ran";
    } catch (const std::logic_error &) {
      seen = "refused";
    }
    return se::nil_value();
  });
  a.submit("(ko)", "Z");
  std::vector<se::exec_group *> gs{&a.g()};
  w.pool().pump_all(gs, kRun, clock_type::time_point::max());
  EXPECT_EQ(seen, "refused");
  EXPECT_NO_THROW(w.pool().kill_owner("Z")) << "outside a pump it runs";
  for (const kill_world::proc &p : w.procs)
    if (p.owner == "Y")
      EXPECT_FALSE(se::is_finished(status_of(*p.sched, p.pid)));
}

// exec_pump_frame's group 0 (S2b): held by another thread (pycvc's ExecPool.run_all drains group 0
// on its own; pool.kill_owner inline), it is skipped this frame, never a throw, and pumped the next
// frame once free. With no frame group (group 0 pumped alone) and with one (host_first).
TEST(GroupZeroDriver, AFrameSkipsGroupZeroWhileAnotherThreadHoldsIt) {
  for (const bool with_frame_group : {false, true}) {
    cvc::app app;
    quiet_rates(app);
    group0_world zero(app);
    const int pid = zero.submit("(begin (set i 0) (while (< i 50) (set i (+ i 1))) i)", "");
    std::unique_ptr<group_world> frame;
    std::optional<cvc::frame_group_handle> handle;
    if (with_frame_group) {
      frame = std::make_unique<group_world>(app, "f");
      handle.emplace(app.register_frame_group(frame->g()));
    }
    std::atomic<bool> held{false}, release{false};
    std::thread holder([&] {
      se::async_scheduler::pump_owner_scope token(zero.sched());
      held = true;
      while (!release.load())
        std::this_thread::yield();
    });
    while (!held.load())
      std::this_thread::yield();
    EXPECT_NO_THROW(app.exec_pump_frame()) << with_frame_group;
    EXPECT_EQ(status_of(zero.sched(), pid), se::process_status::ready) << "pumped while held";
    release = true;
    holder.join();
    app.exec_pump_frame();
    EXPECT_EQ(result_of(zero.sched(), pid), "50") << with_frame_group;
  }
}

} // namespace c_tests
