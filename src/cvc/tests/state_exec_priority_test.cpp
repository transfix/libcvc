/// @file state_exec_priority_test.cpp
/// @brief Process priorities (docs/STATE_EXEC_LOCKS.md 7, 13.6; locks plan L2): set-priority and
///        get-priority, the floors and their permissions, spawn's priority and parent, the run
///        queue keyed on the effective priority. PR-1..PR-4, PR-6 and PR-7 run on both
///        schedulers (PR-10); the permission rules and the level key have unit cases of their own.

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

std::vector<value_t> as_list(const value_t &v) {
  const auto *l = std::get_if<list_ptr>(&v.v);
  EXPECT_NE(l, nullptr) << "expected a list, got " << to_string(v);
  return l ? std::vector<value_t>((*l)->begin(), (*l)->end()) : std::vector<value_t>{};
}

int64_t as_int(const value_t &v) {
  const auto *i = std::get_if<int64_t>(&v.v);
  EXPECT_NE(i, nullptr) << "expected an int, got " << to_string(v);
  return i ? *i : 0;
}

// get-priority's dict for these values.
std::string prio_dict(int base, int effective, int floor, const char *policy = "round_robin") {
  return "{\"base\": " + std::to_string(base) + ", \"effective\": " + std::to_string(effective) +
         ", \"floor\": " + std::to_string(floor) + ", \"policy\": \"" + policy + "\"}";
}

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

// PR-1: a process lowers itself, comes back to its floor, may not go past it; out of range
// values are a ValueError, nothing clamped.
TEST_P(Priority, PR01_SelfChanges) {
  const auto r = as_list(w->run(R"S((list (set-priority 5) (get-priority)
      (set-priority 0)
      (try (set-priority -1) (catch (PermissionError e) "refused"))
      (try (set-priority -21) (catch (ValueError e) "range"))
      (try (set-priority 20) (catch (ValueError e) "range"))
      (try (set-priority 1.5) (catch (ValueError e) "range"))
      (try (set-priority "x" 1) (catch (TypeError e) "type"))
      (set-priority (self-pid) 19) (set-priority -20 19)
      (get-priority) (get-priority (self-pid))))S"));
  ASSERT_EQ(r.size(), 12u);
  EXPECT_EQ(to_string(r[0]), "#t");
  EXPECT_EQ(to_string(r[1]), prio_dict(5, 5, 0));
  EXPECT_EQ(to_string(r[2]), "#t"); // back to the floor
  EXPECT_EQ(to_string(r[3]), "\"refused\"");
  EXPECT_EQ(to_string(r[4]), "\"range\"");
  EXPECT_EQ(to_string(r[5]), "\"range\"");
  EXPECT_EQ(to_string(r[6]), "\"range\"");
  EXPECT_EQ(to_string(r[7]), "\"type\"");
  EXPECT_EQ(to_string(r[8]), "#t");
  EXPECT_EQ(to_string(r[9]), "#f"); // pid -20 is no process
  EXPECT_EQ(to_string(r[10]), prio_dict(19, 19, 0));
  EXPECT_EQ(to_string(r[11]), prio_dict(19, 19, 0));

  // A host's floor below the submit priority: the process may raise itself down to it.
  const auto h = as_list(w->run(R"S((list (set-priority -2)
      (try (set-priority -3) (catch (PermissionError e) (list (get-attr e "pid") (get-attr e "priority"))))
      (get-priority)))S",
                                {.priority = 4, .floor = -2}));
  ASSERT_EQ(h.size(), 3u);
  EXPECT_EQ(to_string(h[0]), "#t");
  const auto who = as_list(h[1]);
  ASSERT_EQ(who.size(), 2u);
  EXPECT_EQ(as_int(who[1]), -3);
  EXPECT_EQ(to_string(h[2]), prio_dict(-2, -2, -2));
}

// PR-2: who a process may see and change. Its spawned child, its forked child, a process of
// the same owner and uid, and, by parent_pid, a descendant of another uid: #t and a dict.
// Another owner, an unknown pid, the same owner with another uid and no descent: #f and nil.
TEST_P(Priority, PR02_OtherProcesses) {
  const int sib = w->submit(R"((begin (msg-recv "sib.go") 1))", {.owner = "doc"});
  const int other = w->submit(R"((begin (msg-recv "other.go") 1))", {.owner = "other"});
  const int eve = w->submit(R"((begin (msg-recv "eve.go") 1))", {.owner = "doc", .uid = "eve"});
  // The parent. The host forks it before its first step (a DSL (fork) inside a step re-applies
  // the call in the child), so its forked child runs the same program and tells itself apart by
  // its parent_pid. The forked child and the host-made descendants of uid eve take the next pids.
  const std::string probe = [&] {
    std::string s;
    for (const std::string &pid :
         {std::string("kid"), std::string("(+ (self-pid) 1)"), std::to_string(sib),
          std::to_string(other), std::string("9999"), std::to_string(eve),
          std::string("(+ (self-pid) 2)"), std::string("(+ (self-pid) 3)")})
      s += " (set-priority " + pid + " 6) (get-priority " + pid + ")";
    return s;
  }();
  const int parent = w->submit(R"S((if (>= (get-attr (inspect (self-pid)) "parent_pid") 0)
      (msg-recv "fork.go")
      (begin
        (set kid (spawn "(begin (msg-recv \"kid.go\") 1)"))
        (list (list kid (+ (self-pid) 1)))S" +
                                   probe + ")))",
                               {.owner = "doc"});
  const int forked = w->base().fork(parent);
  const int desc = w->submit(R"((begin (msg-recv "desc.go") 1))",
                             {.owner = "doc", .uid = "eve", .parent_pid = parent});
  const int grand = w->submit(R"((begin (msg-recv "grand.go") 1))",
                              {.owner = "doc", .uid = "eve", .parent_pid = desc});
  ASSERT_EQ(forked, parent + 1);
  ASSERT_EQ(desc, parent + 2);
  ASSERT_EQ(grand, parent + 3);
  ASSERT_TRUE(w->step_until([&] { return w->finished(parent); }));
  const auto r = as_list(w->result(parent));
  ASSERT_EQ(r.size(), 17u);
  const auto kids = as_list(r[0]);
  ASSERT_EQ(kids.size(), 2u);
  const int kid = static_cast<int>(as_int(kids[0]));
  EXPECT_EQ(as_int(kids[1]), forked);
  const char *names[] = {"spawned", "forked",    "sibling",    "other owner",
                         "unknown", "other uid", "descendant", "grandchild"};
  const bool visible[] = {true, true, true, false, false, false, true, true};
  for (int i = 0; i < 8; ++i) {
    EXPECT_EQ(to_string(r[1 + 2 * i]), visible[i] ? "#t" : "#f") << names[i];
    if (visible[i])
      EXPECT_EQ(to_string(r[2 + 2 * i]), prio_dict(6, 6, 0)) << names[i];
    else
      EXPECT_TRUE(r[2 + 2 * i].is_nil()) << names[i] << ": " << to_string(r[2 + 2 * i]);
  }
  // The changes landed, and nothing else changed.
  for (int pid : {kid, forked, sib, desc, grand})
    EXPECT_EQ(w->info(pid).priority, 6) << pid;
  for (int pid : {other, eve})
    EXPECT_EQ(w->info(pid).priority, 0) << pid;
  EXPECT_EQ(w->info(kid).parent_pid, parent);
  EXPECT_EQ(w->info(forked).parent_pid, parent);
  for (const char *chan :
       {"kid.go", "fork.go", "sib.go", "other.go", "eve.go", "desc.go", "grand.go"})
    w->submit(std::string("(msg-send \"") + chan + "\" \"x\")", {.owner = "doc"});
  ASSERT_TRUE(w->step_until([&] {
    for (int pid : {kid, forked, sib, other, eve, desc, grand})
      if (!w->finished(pid))
        return false;
    return true;
  }));
}

// PR-3: a parent may raise its child only down to the worse of the two floors.
TEST_P(Priority, PR03_RaiseChildAboveCallersFloor) {
  // Its own child, demoted and then raised past the caller's floor (0).
  const auto r = as_list(w->run(R"S((begin
      (set kid (spawn "(begin (msg-recv \"kid.go\") 1)"))
      (list (set-priority kid 5)
            (try (set-priority kid -1) (catch (PermissionError e) "refused"))
            (set-priority kid 0)
            (get-priority kid)
            (msg-send "kid.go" "x"))))S"));
  ASSERT_EQ(r.size(), 5u);
  EXPECT_EQ(to_string(r[0]), "#t");
  EXPECT_EQ(to_string(r[1]), "\"refused\"");
  EXPECT_EQ(to_string(r[2]), "#t");
  EXPECT_EQ(to_string(r[3]), prio_dict(0, 0, 0));

  // A caller whose host floor (3) is worse than its child's (0): the caller's floor binds. The
  // child, of another uid, is the caller's by parent_pid, and takes the next pid.
  const int caller = w->submit(R"S((begin (set kid (+ (self-pid) 1))
      (list (try (set-priority kid 2) (catch (PermissionError e) "refused"))
            (set-priority kid 3) (get-priority kid))))S",
                               {.priority = 3, .floor = 3});
  const int child = w->submit(R"((begin (msg-recv "child.go") 1))",
                              {.uid = "eve", .priority = 8, .floor = 0, .parent_pid = caller});
  ASSERT_EQ(child, caller + 1);
  ASSERT_TRUE(w->step_until([&] { return w->finished(caller); }));
  const auto c = as_list(w->result(caller));
  ASSERT_EQ(c.size(), 3u);
  EXPECT_EQ(to_string(c[0]), "\"refused\"");
  EXPECT_EQ(to_string(c[1]), "#t");
  EXPECT_EQ(to_string(c[2]), prio_dict(3, 3, 0));
  w->submit(R"((msg-send "child.go" "x"))");
  ASSERT_TRUE(w->step_until([&] { return w->finished(child); }));
}

// PR-4: under priority_rr a change moves the process to its new level at once: the next pick
// follows it, and the process table's indexes agree after every step.
TEST_P(Priority, PR04_PriorityRrNextPick) {
  prio_world &pw = world(scheduling_policy::priority_rr);
  const std::string loop = "(set i 0) (while (< i 40) (set i (+ i 1)))";
  // a: runs with b and c, raises itself to -3 (its host floor is -5) and runs alone, then lowers
  // itself to 9 and runs only once b and c have ended.
  const int a = pw.submit("(begin " + loop + " (set-priority -3) " + loop + " (set-priority 9) " +
                              loop + " \"a\")",
                          {.floor = -5});
  const int b = pw.submit("(begin " + loop + loop + loop + loop + " \"b\")");
  const int c = pw.submit("(begin " + loop + loop + loop + loop + " \"c\")");
  const int pids[] = {a, b, c};
  std::map<int, uint64_t> last;
  for (int pid : pids)
    last[pid] = 0;
  int phase = 0; // 0: a at 0; 1: a at -3; 2: a at 9
  int alone = 0, shared_before = 0, behind = 0;
  bool b_and_c_stepped_in_2[2] = {false, false};
  for (int n = 0; n < 200000; ++n) {
    if (pw.finished(a) && pw.finished(b) && pw.finished(c))
      break;
    ASSERT_EQ(pw.step(), 1) << "step " << n;
    ASSERT_TRUE(pw.base().check_tables()) << "step " << n;
    int stepped = -1;
    for (int pid : pids) {
      const uint64_t s = pw.info(pid).step_count;
      if (s != last[pid]) {
        ASSERT_EQ(stepped, -1) << "two processes stepped in step " << n;
        stepped = pid;
        last[pid] = s;
      }
    }
    if (stepped < 0)
      continue; // a step that counted none (none expected; nothing to check)
    if (phase == 1) {
      ASSERT_EQ(stepped, a) << "a at -3 is alone at the best level, step " << n;
      ++alone;
    } else if (phase == 2 && !(pw.finished(b) && pw.finished(c))) {
      ASSERT_NE(stepped, a) << "a at 9 runs after b and c, step " << n;
      b_and_c_stepped_in_2[stepped == b ? 0 : 1] = true;
    } else if (phase == 2) {
      ++behind;
    } else if (stepped != a) {
      ++shared_before;
    }
    const int base = pw.base().get_priority(a)->base;
    if (phase == 0 && base == -3)
      phase = 1;
    else if (phase == 1 && base == 9)
      phase = 2;
  }
  EXPECT_EQ(phase, 2);
  EXPECT_GT(shared_before, 0);
  EXPECT_GT(alone, 0);
  EXPECT_TRUE(b_and_c_stepped_in_2[0] && b_and_c_stepped_in_2[1]);
  EXPECT_GT(behind, 0);
  EXPECT_EQ(to_string(pw.result(a)), "\"a\"");
  EXPECT_EQ(to_string(pw.result(b)), "\"b\"");
  EXPECT_EQ(to_string(pw.result(c)), "\"c\"");
  std::printf("PR-4 %s: %d steps alone at -3, %d with a behind at 9\n", GetParam().c_str(), alone,
              behind);
}

// PR-6: under round_robin a change is stored and reported, and the manager does no inheritance
// work: a better waiter behind a holder boosts nothing.
TEST_P(Priority, PR06_RoundRobin) {
  EXPECT_EQ(to_string(w->run("(list (set-priority 5) (get-priority))")),
            "(#t " + prio_dict(5, 5, 0) + ")");
  const int holder = w->submit(R"((with-lock "k" (msg-recv "go")))", {.priority = 10});
  ASSERT_TRUE(w->step_until([&] { return w->parked(holder); }));
  const int waiter = w->submit(R"((with-lock "k" 1))", {.priority = -10});
  ASSERT_TRUE(w->step_until([&] { return w->parked(waiter); }));
  ASSERT_NE(w->base().locks(), nullptr);
  const lock_stats st = w->base().locks()->manager().stats();
  EXPECT_EQ(st.waits, 1u);
  EXPECT_EQ(st.relevels, 0u);
  EXPECT_EQ(st.relevels_coalesced, 0u);
  EXPECT_EQ(w->info(holder).effective_priority, 10);
  EXPECT_EQ(w->base().get_priority(holder)->effective, 10);
  // A holder's change reaches its rec (protocol 5.8) and still boosts nothing.
  EXPECT_TRUE(w->base().set_priority(holder, 12));
  EXPECT_EQ(w->info(holder).effective_priority, 12);
  EXPECT_EQ(w->base().locks()->manager().stats().relevels, 0u);
  w->submit(R"((msg-send "go" "x"))");
  ASSERT_TRUE(w->step_until([&] { return w->finished(holder) && w->finished(waiter); }));
  EXPECT_EQ(as_int(w->result(waiter)), 1);
}

// PR-7: spawn's priority. From a caller at floor 0, a spawn at -5: under round_robin stored
// unchecked (-5, floor -5); under priority_rr clamped to 0 and counted; with an explicit host
// floor of 0, 0 under both.
TEST_P(Priority, PR07_SpawnPriority) {
  const std::string spawn = R"S((begin (set kid (spawn "1" "kid" -5))
      (list kid (get-attr (inspect kid) "priority") (get-attr (inspect kid) "priority_floor")
            (get-attr (inspect kid) "effective_priority") (get-attr (inspect kid) "parent_pid"))))S";
  struct row {
    scheduling_policy policy;
    std::optional<int> floor;
    int priority, child_floor;
    uint64_t clamped;
  };
  const row rows[] = {
      {scheduling_policy::round_robin, std::nullopt, -5, -5, 0},
      {scheduling_policy::priority_rr, std::nullopt, 0, 0, 1},
      {scheduling_policy::priority, std::nullopt, 0, 0, 1},
      {scheduling_policy::round_robin, 0, 0, 0, 1},
      {scheduling_policy::priority_rr, 0, 0, 0, 1},
  };
  for (const row &rw : rows) {
    prio_world &pw = world(rw.policy);
    const int parent = pw.submit(spawn, {.floor = rw.floor});
    ASSERT_TRUE(pw.step_until([&] { return pw.finished(parent); }));
    const auto r = as_list(pw.result(parent));
    ASSERT_EQ(r.size(), 5u);
    const int kid = static_cast<int>(as_int(r[0]));
    const std::string what =
        "policy " + std::to_string(static_cast<int>(rw.policy)) + (rw.floor ? ", host floor" : "");
    EXPECT_EQ(as_int(r[1]), rw.priority) << what;
    EXPECT_EQ(as_int(r[2]), rw.child_floor) << what;
    EXPECT_EQ(as_int(r[3]), rw.priority) << what;
    EXPECT_EQ(as_int(r[4]), parent) << what;
    const process_info ki = pw.info(kid);
    EXPECT_EQ(ki.priority, rw.priority) << what;
    EXPECT_EQ(ki.priority_floor, rw.child_floor) << what;
    EXPECT_EQ(ki.parent_pid, parent) << what;
    EXPECT_EQ(pw.base().get_stats().priority_clamped, rw.clamped) << what;
  }
  // A spawn at a worse priority than the caller's floor is never clamped; under a priority policy
  // the child keeps the caller's floor, so it may come back to it.
  prio_world &pw = world(scheduling_policy::priority_rr);
  const auto r = as_list(pw.run(R"S((begin (set kid (spawn "(begin (msg-recv \"go\") 1)" "kid" 6))
      (list (get-priority kid) (set-priority kid 0) (msg-send "go" "x"))))S"));
  ASSERT_EQ(r.size(), 3u);
  EXPECT_EQ(to_string(r[0]), prio_dict(6, 6, 0, "priority_rr"));
  EXPECT_EQ(to_string(r[1]), "#t");
  EXPECT_EQ(pw.base().get_stats().priority_clamped, 0u);
}

// Forked children copy the floor and whether a host gave it (design 7.2).
TEST_P(Priority, ForkCopiesTheFloor) {
  // The parent spawns at -9; so does its child, forked by the host before the first step.
  const int parent = w->submit(R"((spawn "1" "g" -9))", {.priority = 4, .floor = 1});
  const int child = w->base().fork(parent);
  ASSERT_GT(child, 0);
  const process_info ci = w->info(child);
  EXPECT_EQ(ci.priority, 4);
  EXPECT_EQ(ci.priority_floor, 1);
  EXPECT_EQ(ci.parent_pid, parent);
  // The copied floor is a host's: the child's spawn clamps even under round_robin, as the
  // parent's does.
  ASSERT_TRUE(w->step_until([&] { return w->finished(parent) && w->finished(child); }));
  for (int pid : {parent, child}) {
    const int spawned = static_cast<int>(as_int(w->result(pid)));
    EXPECT_EQ(w->info(spawned).priority, 1) << pid;
    EXPECT_EQ(w->info(spawned).priority_floor, 1) << pid;
    EXPECT_EQ(w->info(spawned).parent_pid, pid);
  }
  EXPECT_EQ(w->base().get_stats().priority_clamped, 2u);
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
