/// @file state_exec_priority_test.cpp
/// @brief Process priorities (docs/STATE_EXEC_LOCKS.md 7, 13.6; locks plan L2): the schedulers'
///        priority overrides and the floors, on both schedulers; the permission rules and the
///        run queue's level key have unit cases of their own.

#include <chrono>
#include <climits>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/detail/lock_home.h>
#include <cvc/state/state_exec/detail/priority.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/lock_manager.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/process_table.h>
#include <cvc/state/state_exec/scheduler.h>
#include <deque>
#include <gtest/gtest.h>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

using namespace cvc::state_exec;

namespace {

// One app and one scheduler of the given kind ("sync" or "async") and policy, with one intrinsics
// context per program (as state_exec_locks_test.cpp's locks_world), each installing the spawn
// context, so a spawned child has the natives and its parent's owner and uid.
struct prio_world {
  cvc::app app;
  cvc::state &tree;
  std::unique_ptr<scheduler> sync;
  std::unique_ptr<async_scheduler> async;
  std::deque<intrinsics_context> contexts; // stable addresses: the natives point at them

  prio_world(const std::string &kind, scheduling_policy policy) : tree(cvc::state::instance(app)) {
    if (kind == "sync")
      sync = std::make_unique<scheduler>(policy);
    else
      async = std::make_unique<async_scheduler>(policy);
    base().set_watch_root(&tree);
  }
  prio_world(const prio_world &) = delete;
  prio_world &operator=(const prio_world &) = delete;

  scheduler_base &base() { return sync ? static_cast<scheduler_base &>(*sync) : *async; }

  // Who a submitted program runs as.
  struct as {
    std::string owner;
    std::string uid = "test";
    int priority = 0;
    std::optional<int> floor; // a host's floor (explicit)
    int parent_pid = -1;
  };

  int submit(const std::string &script, const as &who) {
    intrinsics_context &ctx = contexts.emplace_back();
    ctx.sched = &base();
    ctx.root = &tree;
    ctx.proc = make_process();
    ctx.proc->owner = who.owner;
    ctx.uid = who.uid;
    ctx.cluster_id = "local";
    ctx.node_id = "local";
    ctx.spawn_context = make_spawn_context();
    environment_ptr env = builtins::make_default_environment();
    register_intrinsics(env, &ctx);
    execute_options opts;
    opts.env = env;
    opts.owner = who.owner;
    opts.uid = who.uid;
    opts.priority = who.priority;
    opts.priority_floor = who.floor;
    opts.parent_pid = who.parent_pid;
    const int pid = base().execute(script, opts);
    ctx.pid = pid;
    ctx.proc->pid = pid;
    return pid;
  }
  int submit(const std::string &script) { return submit(script, as{}); }

  int step() {
    if (sync)
      return sync->step();
    async->drain_ingress();
    return async->sync_step();
  }

  // Step until `done()` holds or `seconds` pass. True when done() held.
  template <class Pred> bool step_until(Pred done, double seconds = 30.0) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (!done()) {
      if (std::chrono::steady_clock::now() >= deadline)
        return false;
      if (!step())
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    return true;
  }

  process_info info(int pid) {
    const auto i = base().get_process_info(pid);
    EXPECT_TRUE(i.has_value()) << "no pid " << pid;
    return i ? *i : process_info{};
  }
  bool finished(int pid) { return is_finished(info(pid).status); }
  bool parked(int pid) { return info(pid).status == process_status::waiting; }

  value_t result(int pid) {
    const auto i = base().get_process_info(pid);
    EXPECT_TRUE(i.has_value());
    if (i && i->exit_error)
      ADD_FAILURE() << "pid " << pid << " ended with " << *i->exit_error;
    const std::optional<value_t> r = sync ? sync->get_result(pid) : async->get_result(pid);
    EXPECT_TRUE(r.has_value()) << "pid " << pid << " has no result"
                               << (i && i->exit_exception ? ": " + to_string(*i->exit_exception)
                                                          : std::string());
    return r ? *r : nil_value;
  }

  // `script` submitted as `who` and run to its end; its result.
  value_t run(const std::string &script, const as &who) {
    const int pid = submit(script, who);
    EXPECT_TRUE(step_until([&] { return finished(pid); })) << "did not finish: " << script;
    return result(pid);
  }
  value_t run(const std::string &script) { return run(script, as{}); }
};

class Priority : public ::testing::TestWithParam<std::string> {
protected:
  std::unique_ptr<prio_world> w =
      std::make_unique<prio_world>(GetParam(), scheduling_policy::round_robin);
  // A fresh world under `policy` (the policy is fixed when a scheduler is built, design 7.3).
  prio_world &world(scheduling_policy policy) {
    w = std::make_unique<prio_world>(GetParam(), policy);
    return *w;
  }
};

INSTANTIATE_TEST_SUITE_P(Schedulers, Priority, ::testing::Values("sync", "async"),
                         [](const auto &info) { return info.param; });

} // namespace

// The schedulers' overrides from the host (design 7.4): the checked pair answers as the DSL's
// natives do, the host's calls are unchecked, and the process info and the stats report the
// effective priority, the floor and the clamps.
TEST_P(Priority, SchedulerOverrides) {
  prio_world &pw = world(scheduling_policy::priority_rr);
  scheduler_base &s = pw.base();
  const int a = pw.submit(R"((begin (msg-recv "a") 1))", {.priority = 2, .floor = 0});
  const int b = pw.submit(R"((begin (msg-recv "b") 1))", {.priority = 4});
  const int c = pw.submit(R"((begin (msg-recv "c") 1))", {.owner = "other"});
  // Who sees whom: the same owner and uid, never another owner or an unknown pid; a process
  // always sees itself.
  EXPECT_TRUE(s.get_priority_as(a, b).has_value());
  EXPECT_FALSE(s.get_priority_as(a, c).has_value());
  EXPECT_FALSE(s.get_priority_as(a, 999).has_value());
  EXPECT_TRUE(s.get_priority_as(c, c).has_value());
  EXPECT_EQ(s.set_priority_checked(a, c, 9), priority_change::unknown);
  EXPECT_EQ(s.set_priority_checked(a, 999, 9), priority_change::unknown);
  // b's floor is its submit priority (4), a's its host's (0): b may come back to 4, not past it.
  EXPECT_EQ(s.set_priority_checked(a, b, 9), priority_change::applied);
  EXPECT_EQ(s.set_priority_checked(a, b, 3), priority_change::refused);
  EXPECT_EQ(s.get_priority(b)->base, 9);
  EXPECT_EQ(s.set_priority_checked(a, b, 4), priority_change::applied);
  EXPECT_EQ(s.set_priority_checked(a, a, 0), priority_change::applied);
  EXPECT_EQ(s.set_priority_checked(a, a, -1), priority_change::refused);
  // The host's calls are unchecked, and a floor it sets is explicit.
  EXPECT_TRUE(s.set_priority(b, -10));
  EXPECT_TRUE(s.set_priority_floor(b, -12));
  EXPECT_FALSE(s.set_priority(999, 1));
  const std::optional<priority_info> pb = s.get_priority(b);
  ASSERT_TRUE(pb.has_value());
  EXPECT_EQ(pb->base, -10);
  EXPECT_EQ(pb->effective, -10);
  EXPECT_EQ(pb->floor, -12);
  EXPECT_EQ(pb->policy, scheduling_policy::priority_rr);
  const process_info ib = pw.info(b);
  EXPECT_EQ(ib.priority, -10);
  EXPECT_EQ(ib.effective_priority, -10);
  EXPECT_EQ(ib.priority_floor, -12);
  EXPECT_EQ(pw.info(a).priority_floor, 0);
  EXPECT_TRUE(s.check_tables());
  // A spawn's clamp is counted through the base (gap G-40).
  EXPECT_EQ(s.get_stats().priority_clamped, 0u);
  s.note_priority_clamped();
  s.note_priority_clamped();
  EXPECT_EQ(s.get_stats().priority_clamped, 2u);
  pw.submit(R"((begin (msg-send "a" "x") (msg-send "b" "x") (msg-send "c" "x")))");
  ASSERT_TRUE(pw.step_until([&] { return pw.finished(a) && pw.finished(b) && pw.finished(c); }));
}

// Forked children copy the floor and whether a host gave it (design 7.2).
TEST_P(Priority, ForkCopiesTheFloor) {
  const int parent = w->submit("1", {.priority = 4, .floor = 1});
  const int child = w->base().fork(parent);
  ASSERT_GT(child, 0);
  const process_info ci = w->info(child);
  EXPECT_EQ(ci.priority, 4);
  EXPECT_EQ(ci.priority_floor, 1);
  EXPECT_EQ(ci.parent_pid, parent);
  ASSERT_TRUE(w->step_until([&] { return w->finished(parent) && w->finished(child); }));
}

// ===========================================================================
// The level key (design 8.7): the run queue keys on the effective priority under a priority
// policy, and on one level under round_robin.
// ===========================================================================

TEST(PriorityTable, LevelKeyFollowsEffective) {
  for (const bool by_priority : {true, false}) {
    detail::process_table table(by_priority);
    for (int pid = 1; pid <= 3; ++pid) {
      auto p = make_process();
      p->pid = pid;
      p->priority = pid == 1 ? 0 : 5;
      p->status = process_status::ready;
      table.insert(std::move(p));
    }
    process &two = *table.find(2);
    const auto first = [&] {
      const process_ptr *pp = table.pick_first();
      return pp ? (*pp)->pid : -1;
    };
    // No inheritance: the key is the base, today's.
    EXPECT_EQ(first(), 1);
    // Pid 2 inherits -3: under a priority policy it is picked first; round robin is unchanged.
    two.inherited_priority = -3;
    table.update(two);
    EXPECT_TRUE(table.check());
    EXPECT_EQ(first(), by_priority ? 2 : 1);
    if (by_priority) {
      int rr = 0;
      for (int k = 0; k < 3; ++k) {
        const process_ptr *pp = table.pick_round_robin(rr);
        ASSERT_NE(pp, nullptr);
        EXPECT_EQ((*pp)->pid, 2); // alone at the best level
      }
    }
    // A base better than the inherited value wins; withdrawn, the base is the key again.
    two.priority = -7;
    table.update(two);
    EXPECT_TRUE(table.check());
    EXPECT_EQ(two.effective_priority(), -7);
    two.priority = 5;
    two.inherited_priority = INT_MAX;
    table.update(two);
    EXPECT_TRUE(table.check());
    EXPECT_EQ(first(), 1);
    // A change of the field without update() is what check() catches (a stale level).
    if (by_priority) {
      two.inherited_priority = -1;
      EXPECT_FALSE(table.check());
      table.update(two);
      EXPECT_TRUE(table.check());
    }
  }
}

// ===========================================================================
// The permission rules (design 7.2), on bare processes
// ===========================================================================

namespace {
struct procs {
  std::map<int, process_ptr> by_pid;
  process &add(int pid, const std::string &owner, const std::string &uid, int priority,
               std::optional<int> floor = std::nullopt, int parent = -1) {
    auto p = make_process();
    p->pid = pid;
    p->owner = owner;
    p->uid = uid;
    p->priority = priority;
    p->priority_floor = floor;
    p->parent_pid = parent;
    p->status = process_status::ready;
    return *(by_pid[pid] = std::move(p));
  }
  std::function<const process *(int)> find() const {
    return [this](int pid) -> const process * {
      const auto it = by_pid.find(pid);
      return it == by_pid.end() || is_finished(it->second->status) ? nullptr : it->second.get();
    };
  }
};
} // namespace

TEST(PriorityRules, SelfLowersAlwaysAndRaisesToItsFloor) {
  procs ps;
  process &me = ps.add(1, "", "u", 0, -2);
  using detail::check_priority_change;
  EXPECT_EQ(check_priority_change(me, me, 19, ps.find()), priority_change::applied);
  EXPECT_EQ(check_priority_change(me, me, -2, ps.find()), priority_change::applied);
  EXPECT_EQ(check_priority_change(me, me, -3, ps.find()), priority_change::refused);
  // No floor (a process no scheduler submitted): the floor reads as the priority.
  process &bare = ps.add(2, "", "u", 3);
  EXPECT_EQ(detail::floor_of(bare), 3);
  EXPECT_EQ(check_priority_change(bare, bare, 3, ps.find()), priority_change::applied);
  EXPECT_EQ(check_priority_change(bare, bare, 2, ps.find()), priority_change::refused);
  // A process that demoted itself comes back to its floor, not past it.
  me.priority = 10;
  EXPECT_EQ(check_priority_change(me, me, -2, ps.find()), priority_change::applied);
  EXPECT_EQ(check_priority_change(me, me, -5, ps.find()), priority_change::refused);
  // Itself it always sees, ended or not.
  me.status = process_status::terminated;
  EXPECT_TRUE(detail::visible_to(me, me, ps.find()));
}

TEST(PriorityRules, OthersByOwnerUidAndDescent) {
  procs ps;
  using detail::check_priority_change;
  using detail::visible_to;
  process &caller = ps.add(1, "doc", "u", 0, 0);
  process &peer = ps.add(2, "doc", "u", 5, 1);
  process &stranger = ps.add(3, "other", "u", 5, 1);
  process &eve = ps.add(4, "doc", "eve", 5, 1);
  // Another owner, or another uid that doesn't descend: unknown, whatever the change.
  EXPECT_FALSE(visible_to(caller, stranger, ps.find()));
  EXPECT_FALSE(visible_to(caller, eve, ps.find()));
  EXPECT_EQ(check_priority_change(caller, stranger, 9, ps.find()), priority_change::unknown);
  EXPECT_EQ(check_priority_change(caller, eve, 9, ps.find()), priority_change::unknown);
  // The same owner and uid: lowering always; raising to the worse of the two floors.
  EXPECT_TRUE(visible_to(caller, peer, ps.find()));
  EXPECT_EQ(check_priority_change(caller, peer, 19, ps.find()), priority_change::applied);
  EXPECT_EQ(check_priority_change(caller, peer, 1, ps.find()), priority_change::applied);
  EXPECT_EQ(check_priority_change(caller, peer, 0, ps.find()), priority_change::refused);
  caller.priority_floor = 3; // now the caller's floor is the worse
  EXPECT_EQ(check_priority_change(caller, peer, 3, ps.find()), priority_change::applied);
  EXPECT_EQ(check_priority_change(caller, peer, 2, ps.find()), priority_change::refused);
  caller.priority_floor = 0;
  // An ended target looks unknown.
  peer.status = process_status::killed;
  EXPECT_EQ(check_priority_change(caller, peer, 9, ps.find()), priority_change::unknown);
  // Unowned hosts count "" as one owner.
  process &a = ps.add(5, "", "x", 0, 0);
  process &b = ps.add(6, "", "x", 0, 0);
  EXPECT_TRUE(visible_to(a, b, ps.find()));
}

TEST(PriorityRules, DescentIsWalkedForSixtyFourHops) {
  procs ps;
  process &root = ps.add(1, "doc", "u", 0, 0);
  // A chain of uid "eve" below the caller: pid 2 its child, pid k+1 the child of pid k.
  for (int pid = 2; pid <= 70; ++pid)
    ps.add(pid, "doc", "eve", 5, 5, pid - 1);
  const auto at = [&](int hops) -> const process & { return *ps.by_pid.at(1 + hops); };
  EXPECT_TRUE(detail::visible_to(root, at(1), ps.find()));
  EXPECT_TRUE(detail::visible_to(root, at(64), ps.find()));
  EXPECT_FALSE(detail::visible_to(root, at(65), ps.find()));
  EXPECT_EQ(detail::check_priority_change(root, at(64), 9, ps.find()), priority_change::applied);
  EXPECT_EQ(detail::check_priority_change(root, at(64), 0, ps.find()), priority_change::refused);
  EXPECT_EQ(detail::check_priority_change(root, at(65), 9, ps.find()), priority_change::unknown);
  // A descendant may not see its ancestor of another uid: descent runs one way.
  EXPECT_FALSE(detail::visible_to(at(1), root, ps.find()));
  // An ended link ends the descent: the ones below it are orphans.
  ps.by_pid.at(10)->status = process_status::terminated;
  EXPECT_TRUE(detail::visible_to(root, at(8), ps.find()));
  EXPECT_FALSE(detail::visible_to(root, at(12), ps.find()));
  // So does a pid that names no process, and a cycle stops at the hop limit.
  procs loop;
  process &c = loop.add(1, "doc", "u", 0, 0);
  loop.add(2, "doc", "eve", 0, 0, 3);
  loop.add(3, "doc", "eve", 0, 0, 2);
  EXPECT_FALSE(detail::visible_to(c, *loop.by_pid.at(2), loop.find()));
  loop.add(4, "doc", "eve", 0, 0, 99);
  EXPECT_FALSE(detail::visible_to(c, *loop.by_pid.at(4), loop.find()));
}

TEST(PriorityRules, SpawnClamp) {
  procs ps;
  process &caller = ps.add(1, "", "u", 2, 0);
  bool clamped = true;
  using detail::clamp_spawn_priority;
  EXPECT_EQ(clamp_spawn_priority(caller, -5, scheduling_policy::round_robin, &clamped), -5);
  EXPECT_FALSE(clamped);
  EXPECT_EQ(clamp_spawn_priority(caller, -5, scheduling_policy::priority, &clamped), 0);
  EXPECT_TRUE(clamped);
  EXPECT_EQ(clamp_spawn_priority(caller, -5, scheduling_policy::priority_rr, &clamped), 0);
  EXPECT_TRUE(clamped);
  EXPECT_EQ(clamp_spawn_priority(caller, 7, scheduling_policy::priority_rr, &clamped), 7);
  EXPECT_FALSE(clamped);
  EXPECT_EQ(clamp_spawn_priority(caller, 0, scheduling_policy::priority_rr, &clamped), 0);
  EXPECT_FALSE(clamped);
  // An explicit floor clamps under every policy.
  caller.priority_floor_explicit = true;
  EXPECT_EQ(clamp_spawn_priority(caller, -5, scheduling_policy::round_robin, &clamped), 0);
  EXPECT_TRUE(clamped);
  EXPECT_EQ(clamp_spawn_priority(caller, -5, scheduling_policy::round_robin, nullptr), 0);
}

TEST(PriorityRules, CfsWeights) {
  EXPECT_EQ(detail::weight_of(0), 1024);
  EXPECT_EQ(detail::weight_of(-10), 9548);
  EXPECT_EQ(detail::weight_of(-20), 88761);
  EXPECT_EQ(detail::weight_of(19), 15);
  EXPECT_EQ(detail::weight_of(-100), 88761);
  EXPECT_EQ(detail::weight_of(100), 15);
  for (int n = -20; n < 19; ++n) {
    EXPECT_GT(detail::weight_of(n), detail::weight_of(n + 1)) << n;
    const double ratio =
        static_cast<double>(detail::weight_of(n)) / static_cast<double>(detail::weight_of(n + 1));
    EXPECT_GT(ratio, 1.15) << n;
    EXPECT_LT(ratio, 1.35) << n;
  }
}
