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

} // namespace c_tests
