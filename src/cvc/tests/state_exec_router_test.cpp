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

// --- R's unit tests (S2b.1, S2b.4) ---

#include <chrono>
#include <cstdlib>
#include <cvc/core/exec_pool.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_call.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using se::value_t;
using route_kind = se::detail::route_entry::kind;

// A scheduler of an app for R's cases, with the intrinsics bound and no chroot (a channel's key is
// its name), pumped by hand on the test thread with steps-only budgets: an exec_group's (with the
// prefixes it is the home of), or group 0's. The context goes after the group, whose teardown kills
// and reaps its processes; group 0's processes must have ended before the world goes.
struct r_world {
  se::intrinsics_context ictx;
  se::environment_ptr env = se::builtins::make_default_environment();
  std::unique_ptr<se::exec_group> group; // null: group 0
  se::async_scheduler *sched = nullptr;

  r_world(cvc::app &app, const std::string &name, std::vector<std::string> prefixes = {}) {
    se::exec_group::options o;
    o.name = name;
    o.app = &app;
    o.watch_root = &cvc::state::instance(app);
    o.channel_prefixes = std::move(prefixes);
    group = std::make_unique<se::exec_group>(std::move(o));
    sched = &group->sched();
    bind(app);
  }
  explicit r_world(cvc::app &app) : sched(&app.exec_scheduler()) { bind(app); }
  ~r_world() { env.reset(); }
  r_world(const r_world &) = delete;
  r_world &operator=(const r_world &) = delete;

  void bind(cvc::app &app) {
    ictx.sched = sched;
    ictx.proc = se::make_process();
    ictx.proc->pid = 0;
    se::apply_chroot(ictx, cvc::state::instance(app), "");
    se::register_intrinsics(env, &ictx);
  }
  int submit(const std::string &src) {
    se::execute_options o;
    o.env = se::environment::extend(env);
    o.max_time = 300.0; // a backstop only
    return group ? group->execute(src, o) : sched->execute(src, o);
  }
  void pump() {
    if (group) {
      group->pump(se::pump_budget{100000, std::numeric_limits<double>::infinity()});
      return;
    }
    sched->drain_ingress();
    sched->wake_awaiting();
    sched->sync_run_steps(100000);
  }
  std::string result(int pid) const {
    const std::optional<value_t> r = sched->get_result(pid);
    const std::string *s = r ? std::get_if<std::string>(&r->v) : nullptr;
    return s ? *s : "(no string result)";
  }
  se::process_status status(int pid) const { return sched->get_process_info(pid)->status; }
  se::async_scheduler::ingress_stats counters() const { return sched->ingress_counters(); }
};

value_t str(const char *s) { return value_t(std::string(s)); }

// A message as msg-send's envelope carries it, with its payload.
value_t envelope(const char *payload) { return se::make_dict({{"payload", str(payload)}}); }

se::detail::route_entry entry(route_kind what, const std::string &key, value_t value = {}) {
  se::detail::route_entry e;
  e.what = what;
  e.key = key;
  e.value = std::move(value);
  return e;
}

se::control_cmd query(std::function<void(se::scheduler_base &)> fn) {
  se::control_cmd q;
  q.what = se::control_cmd::kind::query;
  q.query = std::move(fn);
  return q;
}

// The pending FIFO of `key`, emptied in order (strings, or envelopes' payloads).
std::vector<std::string> drain_fifo(se::async_scheduler &s, const std::string &key) {
  std::vector<std::string> out;
  while (const std::optional<value_t> m = s.pop_pending_message(key)) {
    if (const std::string *t = std::get_if<std::string>(&m->v)) {
      out.push_back(*t);
    } else if (const se::dict_ptr *d = std::get_if<se::dict_ptr>(&m->v)) {
      for (const auto &[k, v] : **d)
        if (k == "payload")
          out.push_back(std::get<std::string>(v.v));
    }
  }
  return out;
}

const char *const kReceive = R"((get-attr (msg-recv "h.k") "payload"))";

// S2b.1: messages, commands and router entries apply in post order across kinds (S2-Q10); the
// settled calls after them. Before, a drain applied every message, then the calls, then the
// commands.
TEST(RouterR, TheOrderedIngressAppliesInPostOrder) {
  se::async_scheduler sched;
  std::vector<std::size_t> seen;
  const auto look = [&] {
    return query([&](se::scheduler_base &s) { seen.push_back(s.pending_message_count("k")); });
  };
  ASSERT_TRUE(sched.post_message("k", str("a")));
  ASSERT_TRUE(sched.post_control(-1, look()));
  ASSERT_TRUE(sched.post_route(entry(route_kind::send, "k", str("b"))));
  ASSERT_TRUE(sched.post_control(-1, look()));
  ASSERT_TRUE(sched.post_message("k", str("c")));
  ASSERT_TRUE(sched.post_control(-1, look()));
  sched.drain_ingress();
  EXPECT_EQ(seen, (std::vector<std::size_t>{1, 2, 3}));
  EXPECT_EQ(drain_fifo(sched, "k"), (std::vector<std::string>{"a", "b", "c"}));

  // A settled call is applied after the queue, a command posted after it included: the command
  // sees the call's waiter still waiting.
  const int pid = sched.execute(std::string("(+ 1 2)"));
  auto call = std::make_shared<se::async_call>("probe", pid); // not tracked: its finish wakes
  ASSERT_TRUE(sched.receive_message(pid, call->channel()));
  se::process_status at_command = se::process_status::ready;
  sched.post_async_completion(call);
  sched.post_control(
      -1, query([&](se::scheduler_base &s) { at_command = s.get_process_info(pid)->status; }));
  sched.drain_ingress();
  EXPECT_EQ(at_command, se::process_status::waiting);
  EXPECT_EQ(sched.get_process_info(pid)->status, se::process_status::ready);
}

// The throw rule (exec_router.h; S0.7): the entry that threw is not put back and is counted; a
// command that threw is refused; what was not applied goes back to the front, ahead of later
// posts, and the next drain applies it.
TEST(RouterR, TheThrowRuleCountsTheEntryAndRequeuesTheRest) {
  se::async_scheduler sched;
  bool paused = false;
  const int pid = sched.execute(std::string("(begin (set i 0) (while #t (set i (+ i 1))))"));
  ASSERT_TRUE(sched.post_message("k", str("a")));
  ASSERT_TRUE(sched.post_control(
      -1, query([](se::scheduler_base &) { throw std::runtime_error("the query threw"); })));
  ASSERT_TRUE(sched.post_message("k", str("b")));
  se::control_cmd pause;
  pause.what = se::control_cmd::kind::pause;
  pause.done = [&](bool ok) { paused = ok; };
  ASSERT_TRUE(sched.post_control(pid, std::move(pause)));
  EXPECT_THROW(sched.drain_ingress(), std::runtime_error);
  EXPECT_EQ(sched.pending_message_count("k"), 1u) << "applied past the throw, or not before it";
  EXPECT_FALSE(paused);
  EXPECT_TRUE(sched.has_pending_ingress());
  EXPECT_TRUE(sched.has_pending_control());
  const se::async_scheduler::ingress_stats c = sched.ingress_counters();
  EXPECT_EQ(c.throw_dropped, 1u);
  EXPECT_EQ(c.control_refused, 1u);
  ASSERT_TRUE(sched.post_message("k", str("c"))); // after the throw: behind what went back
  sched.drain_ingress();
  EXPECT_TRUE(paused);
  EXPECT_EQ(sched.get_process_info(pid)->status, se::process_status::paused);
  EXPECT_EQ(drain_fifo(sched, "k"), (std::vector<std::string>{"a", "b", "c"}));
  EXPECT_EQ(sched.ingress_counters().throw_dropped, 1u);
}

// The destructor (S2-Q10): commands refused, plain messages dropped and counted, router entries
// bounced to their homes: a deliver as a requeue to its home (at the front there), a send to the
// key's home; a stale copy of a message a local waiter took is dropped and counted. With no router
// a router entry is dropped and counted.
TEST(RouterR, TheDestructorBouncesRouterEntries) {
  cvc::app app;
  r_world h(app, "h", {"h"});
  bool refused = false;
  se::async_scheduler::ingress_stats at_end;
  {
    se::async_scheduler x;
    x.set_router(&app.exec_router(), se::group_ref::make(x, nullptr)); // in no registration
    se::detail::route_entry d = entry(route_kind::deliver, "h.k", envelope("d"));
    d.from = h.group->route();
    d.pid = 99999;
    d.seq = 1;
    ASSERT_TRUE(x.post_route(std::move(d)));
    ASSERT_TRUE(x.post_route(entry(route_kind::send, "h.k", envelope("s"))));
    se::detail::route_entry stale = entry(route_kind::deliver, "h.k", envelope("z"));
    stale.from = h.group->route();
    stale.pid = 99998;
    stale.seq = 1;
    stale.requeue_if_stale = false;
    ASSERT_TRUE(x.post_route(std::move(stale)));
    ASSERT_TRUE(x.post_message("x#plain", str("p"))); // a '#' key: never routed, queued here
    se::control_cmd kill;
    kill.what = se::control_cmd::kind::kill;
    kill.done = [&](bool applied) {
      refused = !applied;
      at_end = x.ingress_counters(); // the entries before it are done with
    };
    ASSERT_TRUE(x.post_control(1, std::move(kill)));
  }
  EXPECT_TRUE(refused);
  EXPECT_EQ(at_end.destroy_dropped, 1u); // the plain message
  EXPECT_EQ(at_end.stale_dropped, 1u);
  h.sched->drain_ingress();
  EXPECT_EQ(drain_fifo(*h.sched, "h.k"), (std::vector<std::string>{"d", "s"}));
  EXPECT_EQ(h.counters().requeued, 1u);

  se::async_scheduler::ingress_stats lone;
  {
    se::async_scheduler y; // no router: nowhere to bounce to
    ASSERT_TRUE(y.post_route(entry(route_kind::send, "k", str("v"))));
    ASSERT_TRUE(y.post_message("k", str("w")));
    se::control_cmd kill;
    kill.what = se::control_cmd::kind::kill;
    kill.done = [&](bool) { lone = y.ingress_counters(); };
    ASSERT_TRUE(y.post_control(1, std::move(kill)));
  }
  EXPECT_EQ(lone.destroy_dropped, 2u);
}

// The cap (S2-Q10): capped entries (host messages, sends not uncapped) past ingress_cap() are
// refused and counted; nothing else is; a drain empties the count; 0 is no cap. The tree's
// state_exec.groups.ingress_cap applies unless set_ingress_cap was called. A msg-send to a home
// at its cap gives {status "error", path, reason "ingress full"}.
TEST(RouterR, TheIngressCapRefusesCappedEntriesOnly) {
  se::async_scheduler s;
  EXPECT_EQ(s.ingress_cap(), 65536u);
  s.set_ingress_cap(2);
  EXPECT_TRUE(s.post_message("k", str("a")));
  EXPECT_TRUE(s.post_message("k", str("b")));
  EXPECT_FALSE(s.post_message("k", str("c")));
  EXPECT_FALSE(s.post_route(entry(route_kind::send, "k", str("d"))));
  se::detail::route_entry rehomed = entry(route_kind::send, "k", str("e"));
  rehomed.uncapped = true;
  EXPECT_TRUE(s.post_route(std::move(rehomed)));
  EXPECT_TRUE(s.post_route(entry(route_kind::requeue, "k", str("f"))));
  EXPECT_TRUE(s.post_control(-1, query([](se::scheduler_base &) {})));
  EXPECT_EQ(s.ingress_counters().ingress_refused, 2u);
  s.drain_ingress();
  EXPECT_TRUE(s.post_message("k", str("g"))) << "a drain empties the cap's count";
  s.set_ingress_cap(0);
  for (int i = 0; i < 100; ++i)
    EXPECT_TRUE(s.post_message("k", str("h")));
  EXPECT_EQ(s.ingress_counters().ingress_refused, 2u);

  cvc::app app;
  cvc::state &tree = cvc::state::instance(app);
  tree("state_exec.groups.ingress_cap").value(std::string("1"));
  r_world home(app, "h", {"h"});
  home.pump(); // a run loop reads the tree's settings
  EXPECT_EQ(home.sched->ingress_cap(), 1u);
  r_world sender(app, "s");
  const int pid = sender.submit(R"((begin (msg-send "h.k" "one") (msg-send "h.k" "two")))");
  sender.pump();
  const std::optional<value_t> r = sender.sched->get_result(pid);
  ASSERT_TRUE(r && std::holds_alternative<se::dict_ptr>(r->v));
  std::vector<std::string> keys;
  std::string status, reason;
  for (const auto &[k, v] : *std::get<se::dict_ptr>(r->v)) {
    keys.push_back(k);
    if (k == "status")
      status = std::get<std::string>(v.v);
    if (k == "reason")
      reason = std::get<std::string>(v.v);
  }
  EXPECT_EQ(keys, (std::vector<std::string>{"status", "path", "reason"}));
  EXPECT_EQ(status, "error");
  EXPECT_EQ(reason, "ingress full");
  EXPECT_EQ(home.counters().ingress_refused, 1u);
  home.sched->set_ingress_cap(5); // wins over the tree's from now on
  tree("state_exec.groups.ingress_cap").value(std::string("3"));
  home.pump();
  EXPECT_EQ(home.sched->ingress_cap(), 5u);
  EXPECT_EQ(drain_fifo(*home.sched, "h.k").size(), 1u);
}

// deliver_to (td 4.6): only the wait it was meant for, by pid, recv_seq and key; recv_seq moves at
// every park, so an answered or older wait is stale.
TEST(RouterR, DeliverToIsForOneWaitOnly) {
  se::async_scheduler s;
  const int pid = s.execute(std::string("(+ 1 2)"));
  ASSERT_TRUE(s.receive_message(pid, "k")); // its first park: recv_seq 1
  EXPECT_FALSE(s.deliver_to(pid, 2, "k", str("x")));
  EXPECT_FALSE(s.deliver_to(pid, 1, "j", str("x")));
  EXPECT_FALSE(s.deliver_to(pid + 1, 1, "k", str("x")));
  EXPECT_EQ(s.get_process_info(pid)->status, se::process_status::waiting);
  EXPECT_TRUE(s.deliver_to(pid, 1, "k", str("x")));
  EXPECT_EQ(s.get_process_info(pid)->status, se::process_status::ready);
  EXPECT_FALSE(s.deliver_to(pid, 1, "k", str("x"))) << "an answered wait took a second message";
  ASSERT_TRUE(s.receive_message(pid, "k")); // recv_seq 2
  EXPECT_FALSE(s.deliver_to(pid, 1, "k", str("x")));
  EXPECT_TRUE(s.deliver_to(pid, 2, "k", str("y")));
  EXPECT_TRUE(s.check_tables());
}

// A requeue (S2-Q10) goes to the front of the pending FIFO, past max_pending_messages, whose
// drops are counted.
TEST(RouterR, ARequeueGoesToTheFrontPastTheCap) {
  se::async_scheduler s;
  s.max_pending_messages = 2;
  s.post_message("k", str("a"));
  s.post_message("k", str("b"));
  s.post_message("k", str("c")); // the FIFO is full: dropped
  s.drain_ingress();
  EXPECT_EQ(s.ingress_counters().pending_dropped, 1u);
  ASSERT_TRUE(s.post_route(entry(route_kind::requeue, "k", str("z"))));
  s.drain_ingress();
  EXPECT_EQ(s.pending_message_count("k"), 3u);
  EXPECT_EQ(s.ingress_counters().requeued, 1u);
  EXPECT_EQ(drain_fifo(s, "k"), (std::vector<std::string>{"z", "a", "b"}));
}

// Broadcast tracking (exec_router.h): with no local waiter and two remote ones, a message whose
// copies all come back stale is queued once at its home; with one copy taken, the other stale copy
// is dropped there. A copy of a message a local waiter took is dropped at its receiver when stale.
TEST(RouterR, BroadcastTrackingQueuesOnceOrDrops) {
  for (const bool one_taken : {false, true}) {
    cvc::app app;
    r_world h(app, "h", {"h"}), r1(app, "r1"), r2(app, "r2");
    const int p1 = r1.submit(kReceive), p2 = r2.submit(kReceive);
    r1.pump();
    r2.pump(); // each parks, and subscribes at h
    h.pump();  // two remote waiters
    ASSERT_EQ(r1.status(p1), se::process_status::waiting);
    ASSERT_TRUE(h.sched->post_message("h.k", envelope("hi")));
    h.pump(); // a tracked copy to each
    if (!one_taken)
      r1.sched->kill(p1);
    r2.sched->kill(p2);
    r1.pump();
    r2.pump(); // a stale copy goes back; a taken one says so
    h.pump();
    if (one_taken) {
      EXPECT_EQ(r1.result(p1), "hi");
      EXPECT_EQ(h.sched->pending_message_count("h.k"), 0u) << "delivered twice";
      EXPECT_EQ(h.counters().stale_dropped, 1u);
      EXPECT_EQ(h.counters().requeued, 0u);
    } else {
      EXPECT_EQ(h.sched->pending_message_count("h.k"), 1u) << "lost, or queued twice";
      EXPECT_EQ(h.counters().requeued, 1u);
    }
  }
  // A local waiter at the home took it: the remote copy goes untracked, dropped when stale.
  cvc::app app;
  r_world h(app, "h", {"h"}), r1(app, "r1");
  const int p1 = r1.submit(kReceive);
  r1.pump();
  const int ph = h.submit(kReceive);
  h.pump(); // r1's record, and h's own waiter
  ASSERT_TRUE(h.sched->post_message("h.k", envelope("hi")));
  h.pump();
  EXPECT_EQ(h.result(ph), "hi");
  r1.sched->kill(p1);
  r1.pump();
  h.pump();
  EXPECT_EQ(r1.counters().stale_dropped, 1u);
  EXPECT_EQ(h.sched->pending_message_count("h.k"), 0u);
  EXPECT_EQ(h.counters().requeued, 0u);
}

// A home's teardown (exec_router.h, S2b.0a): its pending messages (in FIFO order), its
// remote-waiter records (subscribes with the same seq), then its queued sends and host posts of
// keys it homed (uncapped, in queue order), go to the keys' new home, here group 0, built for it:
// group 0's FIFO keeps the order the dying home took them in, past group 0's own cap.
TEST(RouterR, ATornDownHomeForwardsWhatItHoldsInOrder) {
  cvc::app app;
  r_world r(app, "r");
  auto g = std::make_unique<r_world>(app, "g", std::vector<std::string>{"g"});
  ASSERT_TRUE(g->sched->post_message("g.k", envelope("a")));
  ASSERT_TRUE(g->sched->post_message("g.k", envelope("b")));
  const int p = r.submit(R"((get-attr (msg-recv "g.j") "payload"))");
  r.pump(); // subscribes at g
  g->pump();
  ASSERT_EQ(g->sched->pending_message_count("g.k"), 2u);
  ASSERT_TRUE(g->sched->post_route(entry(route_kind::send, "g.k", envelope("q")))); // queued
  ASSERT_TRUE(g->sched->post_message("g.k", envelope("h")));                        // queued
  se::async_scheduler &g0 = app.exec_scheduler();
  g0.max_pending_messages = 1; // a rehomed message is never dropped by its move
  g.reset();
  g0.drain_ingress();
  EXPECT_EQ(drain_fifo(g0, "g.k"), (std::vector<std::string>{"a", "b", "q", "h"}));
  EXPECT_EQ(g0.ingress_counters().pending_dropped, 0u);
  // r's wait moved with its record: a message there reaches it.
  ASSERT_TRUE(g0.post_message("g.j", envelope("moved")));
  g0.drain_ingress();
  r.pump();
  EXPECT_EQ(r.result(p), "moved");
}

// A teardown forwards its broadcast trackers (adopt, S2b.0a): a copy taken and one stale after
// the home went are answered at the new home, which adopted the tracker, so the stale copy is
// dropped there, not delivered a second time.
TEST(RouterR, ATornDownHomesTrackerIsAdopted) {
  cvc::app app;
  r_world r1(app, "r1"), r2(app, "r2");
  auto h = std::make_unique<r_world>(app, "h", std::vector<std::string>{"h"});
  const int p1 = r1.submit(kReceive), p2 = r2.submit(kReceive);
  r1.pump();
  r2.pump();
  h->pump(); // two records
  ASSERT_TRUE(h->sched->post_message("h.k", envelope("once")));
  h->pump(); // a tracked copy to each
  r2.sched->kill(p2);
  se::async_scheduler &g0 = app.exec_scheduler();
  h.reset(); // its tracker goes to group 0 as an adopt
  r1.pump(); // taken: to h, closed and unregistered, resolved again to group 0
  r2.pump(); // stale: a requeue, likewise
  g0.drain_ingress();
  EXPECT_EQ(r1.result(p1), "once");
  EXPECT_EQ(g0.total_pending_messages(), 0u) << "the stale copy was queued: a second delivery";
  EXPECT_EQ(g0.ingress_counters().stale_dropped, 1u);
  EXPECT_EQ(g0.ingress_counters().requeued, 0u);
}

// The next waiter for a requeue (S2b.0a) is the earliest local waiter (in park order, whatever
// the pids), else the earliest record: one waiter, not every one.
TEST(RouterR, ARequeueGoesToTheEarliestWaiterOnly) {
  se::async_scheduler s;
  const int p1 = s.execute(std::string("(+ 1 2)"));
  const int p2 = s.execute(std::string("(+ 1 2)"));
  ASSERT_TRUE(s.receive_message(p2, "k")); // parks first
  ASSERT_TRUE(s.receive_message(p1, "k"));
  ASSERT_TRUE(s.post_route(entry(route_kind::requeue, "k", str("v"))));
  s.drain_ingress();
  EXPECT_EQ(s.get_process_info(p2)->status, se::process_status::ready);
  EXPECT_EQ(s.get_process_info(p1)->status, se::process_status::waiting);
  EXPECT_TRUE(s.check_tables());

  cvc::app app;
  r_world h(app, "h", {"h"}), r1(app, "r1"), r2(app, "r2");
  const int q1 = r1.submit(kReceive), q2 = r2.submit(kReceive);
  r2.pump(); // subscribes first
  r1.pump();
  h.pump(); // records r2's wait, then r1's
  ASSERT_TRUE(h.sched->post_route(entry(route_kind::requeue, "h.k", envelope("next"))));
  h.pump();
  r1.pump();
  r2.pump();
  EXPECT_EQ(r2.result(q2), "next");
  EXPECT_EQ(r1.status(q1), se::process_status::waiting);
  EXPECT_EQ(h.sched->pending_message_count("h.k"), 0u);
}

// A deliver that a receiving group's closed handle refuses is stale at once at the home (S2b.0a):
// applied there as a requeue (the front of the FIFO, here), not lost.
TEST(RouterR, ADeliverAClosedReceiverRefusesIsStaleAtOnce) {
  cvc::app app;
  r_world h(app, "h", {"h"}), r(app, "r");
  r.submit(kReceive);
  r.pump();
  h.pump();                 // r's record
  r.group->route().close(); // as its teardown's first step does
  ASSERT_TRUE(h.sched->post_message("h.k", envelope("kept")));
  h.pump();
  EXPECT_EQ(drain_fifo(*h.sched, "h.k"), (std::vector<std::string>{"kept"}));
  EXPECT_EQ(h.counters().requeued, 1u);
}

// An uncapped send (a rehomed message, S2b.0a) that finds no waiter joins the FIFO past
// max_pending_messages; a capped one is dropped there, and counted.
TEST(RouterR, AnUncappedSendGoesPastTheFifosCap) {
  se::async_scheduler s;
  s.max_pending_messages = 1;
  ASSERT_TRUE(s.post_route(entry(route_kind::send, "k", str("a"))));
  ASSERT_TRUE(s.post_route(entry(route_kind::send, "k", str("dropped"))));
  se::detail::route_entry moved = entry(route_kind::send, "k", str("b"));
  moved.uncapped = true;
  ASSERT_TRUE(s.post_route(std::move(moved)));
  s.drain_ingress();
  EXPECT_EQ(s.ingress_counters().pending_dropped, 1u);
  EXPECT_EQ(drain_fifo(s, "k"), (std::vector<std::string>{"a", "b"}));
}

// Islands (S2b.0a): with the router on (a group with a prefix exists), a group that registered no
// prefix keeps the keys no other group's prefix covers, as today: its own send and receive, and a
// host's post to it, stay in it, and another island does not see them. A group with a prefix sends
// such a key to group 0.
TEST(RouterR, AnIslandKeepsItsOwnKeys) {
  cvc::app app;
  r_world p(app, "p", {"p"}), a(app, "a"), b(app, "b");
  ASSERT_TRUE(app.exec_router().active());
  const int pa =
      a.submit(R"((begin (msg-send "a.k" "mine") (get-attr (msg-recv "a.k") "payload")))");
  const int pb = b.submit(R"((msg-pending "a.k"))");
  const int pp = p.submit(R"((begin (msg-send "z.k" "to group 0") "sent"))");
  a.pump();
  b.pump();
  p.pump();
  EXPECT_EQ(a.result(pa), "mine");
  const std::optional<value_t> nb = b.sched->get_result(pb);
  ASSERT_TRUE(nb);
  EXPECT_EQ(se::to_string(*nb), "0");
  ASSERT_TRUE(a.sched->post_message("a.j", str("host")));
  a.sched->drain_ingress();
  EXPECT_EQ(a.sched->pending_message_count("a.j"), 1u);
  se::async_scheduler &g0 = app.exec_scheduler();
  g0.drain_ingress();
  EXPECT_EQ(g0.pending_message_count("z.k"), 1u);
  EXPECT_EQ(g0.total_pending_messages(), 1u) << "an island's key reached group 0";
}

// An island homes nothing for another group (S2b.0a): its teardown forwards none of its own
// keys' messages, pending or queued; they die with it, counted (destroy_dropped).
TEST(RouterR, AnIslandsTeardownForwardsNothing) {
  cvc::app app;
  r_world p(app, "p", {"p"}); // the router on
  auto a = std::make_unique<r_world>(app, "a");
  ASSERT_TRUE(a->sched->post_message("a.k", str("pending")));
  a->pump();
  ASSERT_TRUE(a->sched->post_message("a.k", str("queued")));
  se::async_scheduler::ingress_stats at_end;
  se::control_cmd kill;
  kill.what = se::control_cmd::kind::kill;
  se::async_scheduler *const raw = a->sched;
  kill.done = [&](bool) { at_end = raw->ingress_counters(); }; // refused by its destructor
  ASSERT_TRUE(a->sched->post_control(1, std::move(kill)));
  a.reset();
  EXPECT_EQ(at_end.destroy_dropped, 2u);
  if (se::async_scheduler *g0 = app.exec_scheduler_probe()) {
    g0->drain_ingress();
    EXPECT_EQ(g0->total_pending_messages(), 0u) << "an island's message was forwarded";
  }
}

// C's calls (S2b.2, S2b.3) at R's sites: a pid is in the directory from its creation to its reap,
// and each run loop publishes; a floor set between run loops moves the table's generation, so the
// next publish shows it (no update() runs on that path).
TEST(RouterR, TheDirectoryAndThePublishFollowTheProcess) {
  cvc::app app;
  r_world a(app, "a");
  const int pid = a.submit(R"((get-attr (msg-recv "never") "payload"))");
  ASSERT_TRUE(app.exec_router().directory().find(pid).has_value());
  a.pump(); // parks: no status change from here on
  ASSERT_TRUE(a.sched->published_info(pid).has_value());
  EXPECT_EQ(a.sched->published_info(pid)->floor, 0);
  ASSERT_TRUE(a.sched->set_priority_floor(pid, 7));
  a.pump();
  EXPECT_EQ(a.sched->published_info(pid)->floor, 7);
  ASSERT_TRUE(a.sched->kill(pid));
  ASSERT_TRUE(a.sched->reap(pid));
  EXPECT_FALSE(app.exec_router().directory().find(pid).has_value());
}

// Ariadne's move (A's retire_from_router): the old group leaves the router under its token, after
// its processes were killed and reaped and the new group registered the same prefix; its teardown
// leaves again and finds nothing. When the token was held elsewhere the move only unregistered
// it: its teardown still forwards what it held for the prefix, to the group that has it now.
TEST(RouterR, LeavingTheRouterTwiceAndAfterAMove) {
  cvc::app app;
  auto old_group = std::make_unique<r_world>(app, "old", std::vector<std::string>{"d"});
  ASSERT_TRUE(old_group->sched->post_message("d.k", envelope("held")));
  const int pid = old_group->submit(R"((get-attr (msg-recv "never") "payload"))");
  old_group->pump();
  r_world moved(app, "new", {"d"}); // the same prefix, registered after
  ASSERT_TRUE(old_group->sched->kill(pid));
  ASSERT_TRUE(old_group->sched->reap(pid));
  old_group->sched->leave_router(); // no lock port closed first
  moved.sched->drain_ingress();
  EXPECT_EQ(drain_fifo(*moved.sched, "d.k"), (std::vector<std::string>{"held"}));
  const se::async_scheduler::ingress_stats before = old_group->counters();
  old_group->sched->leave_router(); // again: nothing to send
  old_group.reset();                // and its teardown's
  EXPECT_FALSE(moved.sched->has_pending_ingress());
  EXPECT_EQ(before.destroy_dropped, 0u);

  // Only unregistered by the move: the teardown forwards to the prefix's home now.
  auto held = std::make_unique<r_world>(app, "held", std::vector<std::string>{"e"});
  ASSERT_TRUE(held->sched->post_message("e.k", envelope("kept")));
  held->pump();
  r_world next(app, "next", {"e"});
  app.exec_router().unregister(held->group->route());
  held.reset();
  next.sched->drain_ingress();
  EXPECT_EQ(drain_fifo(*next.sched, "e.k"), (std::vector<std::string>{"kept"}));
}

// The cache of homes is checked against the router's epoch: a key homed at group 0 is sent to a
// group registered for it since.
TEST(RouterR, TheCacheOfHomesFollowsTheEpoch) {
  cvc::app app;
  r_world s(app, "s", {"s"}); // turns the router on
  const int a = s.submit(R"((begin (msg-send "x.k" "one") "sent"))");
  s.pump();
  ASSERT_EQ(s.result(a), "sent");
  se::async_scheduler &g0 = app.exec_scheduler();
  g0.drain_ingress();
  EXPECT_EQ(g0.pending_message_count("x.k"), 1u);
  r_world x(app, "x", {"x"});
  const int b = s.submit(R"((begin (msg-send "x.k" "two") "sent"))");
  s.pump();
  ASSERT_EQ(s.result(b), "sent");
  x.sched->drain_ingress();
  g0.drain_ingress();
  EXPECT_EQ(x.sched->pending_message_count("x.k"), 1u);
  // The first stays where it was (a registration moves nothing), and group 0 now reads the key as
  // homed at x: its own count of it is 0.
  EXPECT_EQ(g0.total_pending_messages(), 1u);
  EXPECT_EQ(g0.pending_message_count("x.k"), 0u);
}

// A post through a closed handle (its group torn down) is resolved again: a stale deliver's
// requeue to a home gone meanwhile reaches the key's new home. A host's post on a key homed
// elsewhere is forwarded there (async_scheduler::post_message, exec_router::post_message).
TEST(RouterR, AClosedHomeIsResolvedAgain) {
  cvc::app app;
  r_world r(app, "r");
  auto h = std::make_unique<r_world>(app, "h", std::vector<std::string>{"h"});
  se::detail::route_entry d = entry(route_kind::deliver, "h.k", envelope("back"));
  d.from = h->group->route();
  d.pid = 99999; // waits for nothing: stale
  d.seq = 1;
  ASSERT_TRUE(r.sched->post_route(std::move(d)));
  h.reset(); // unregistered and closed
  r.pump();
  se::async_scheduler &g0 = app.exec_scheduler();
  g0.drain_ingress();
  EXPECT_EQ(drain_fifo(g0, "h.k"), (std::vector<std::string>{"back"}));

  r_world y(app, "y", {"y"});
  ASSERT_TRUE(g0.post_message("y.k", str("from group 0")));
  ASSERT_TRUE(app.exec_router().post_message("y.k", str("from the pool")));
  y.sched->drain_ingress();
  EXPECT_EQ(drain_fifo(*y.sched, "y.k"),
            (std::vector<std::string>{"from group 0", "from the pool"}));
}

// RT-1: with groups that register no prefix, messaging never consults the router.
TEST(RouterR, NoPrefixNoConsultation) {
  cvc::app app;
  r_world a(app, "a"), b(app, "b");
  r_world g0(app);
  const int pa = a.submit(R"((get-attr (msg-recv "h.k") "payload"))");
  const int pb = b.submit(R"((begin (msg-send "h.k" "x") (msg-pending "h.k")))");
  const int p0 = g0.submit(R"((begin (msg-send "z" "y") (get-attr (msg-recv "z") "payload")))");
  a.pump();
  b.pump();
  g0.pump();
  EXPECT_EQ(a.status(pa), se::process_status::waiting); // b's send stays in b's group
  EXPECT_TRUE(se::is_finished(b.status(pb)));
  EXPECT_EQ(g0.result(p0), "y");
  EXPECT_FALSE(app.exec_router().active());
  EXPECT_EQ(app.exec_router().consulted(), 0u);
  a.sched->kill(pa);
}

// The protocol on threads: groups attached to a continuous pool (threads 0, 1, 2, 4), a sender in S
// and a receiver in R on a key homed at H, both remote. Every message arrives once, in the order
// sent (FIFO per sender group). CVC_ROUTER_R_THREADS=<n> runs one thread count.
TEST(RouterR, ARemoteReceiverGetsEveryMessageInOrderOnAPool) {
  std::vector<unsigned> counts{0, 1, 2, 4};
  if (const char *v = std::getenv("CVC_ROUTER_R_THREADS"); v && *v)
    counts = {static_cast<unsigned>(std::strtoul(v, nullptr, 10))};
  constexpr int kMessages = 300;
  for (const unsigned threads : counts) {
    SCOPED_TRACE("threads " + std::to_string(threads));
    cvc::app app;
    cvc::state::instance(app)("state_exec.defaults.limits.async_steps_per_cpu_ms")
        .value(std::string("0"));
    cvc::pool_options opts;
    opts.threads = threads;
    opts.pump_steps = 97;
    opts.pump_seconds = std::numeric_limits<double>::infinity();
    cvc::exec_pool &pool = app.exec_pool(opts);
    r_world h(app, "h", {"h"}), r(app, "r"), s(app, "s");
    const std::string n = std::to_string(kMessages);
    const int pr = r.submit("(begin (set n 0) (set ok 0) (while (< n " + n +
                            ") (begin (if (= (get-attr (msg-recv \"h.k\") \"payload\") (str n)) "
                            "(set ok (+ ok 1)) nil) (set n (+ n 1)))) (str ok))");
    const int ps =
        s.submit("(begin (for i (range " + n + ") (msg-send \"h.k\" (str i))) \"sent\")");
    for (r_world *w : {&h, &r, &s})
      pool.attach(*w->group);
    bool done = false;
    const auto t0 = std::chrono::steady_clock::now();
    while (!done &&
           std::chrono::steady_clock::now() - t0 < std::chrono::seconds(60)) { // hang guard
      if (!pool.pump_host(std::numeric_limits<double>::infinity()))
        pool.wait_for_work(0.01);
      pool.with_group(*r.group, [&] { done = se::is_finished(r.status(pr)); });
    }
    for (r_world *w : {&h, &r, &s})
      pool.detach(*w->group);
    ASSERT_TRUE(done) << "the receiver never ended";
    EXPECT_EQ(r.result(pr), n);
    EXPECT_EQ(s.result(ps), "sent");
    EXPECT_EQ(h.sched->pending_message_count("h.k"), 0u);
    const se::async_scheduler::ingress_stats c = h.counters();
    EXPECT_EQ(c.requeued + c.stale_dropped + c.ingress_refused + c.throw_dropped, 0u);
  }
}

} // namespace

// --- Stage 2b router and control across groups (td 8.4, S2.md 4) ---
//
// K's cases (S2b.8), written against S2b.0's frozen interface (exec_router.h's header comment is
// the contract) before the implementation. A case drives its groups in an order it chooses: each
// step one pump of one group, by hand (exec_group::pump on a steps-only budget; group 0 under its
// pump owner) or through the pool (one pump_all of that group, deterministic or threaded), so the
// posts between groups land in a known order. Oracles are outcomes (results, statuses, queued
// counts and the ingress counters), each cross-group outcome compared with the same program in one
// group. Nothing asserts time: a wait for a deadline waits for the clock to pass it, and a ceiling
// only turns a hang into a failure (hard rule 7).

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/exec_pool.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_host.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <vector>

namespace {

using namespace cvc::state_exec;
using rt_clock = std::chrono::steady_clock;

constexpr double kRtInf = std::numeric_limits<double>::infinity();
// No pump_all deadline: a steps-only pump, as a hand pump is (exec_pool.cpp reads max() as none).
constexpr rt_clock::time_point kRtNoDeadline = rt_clock::time_point::max();
// A pump's steps: far more than any scenario's programs take between two of its steps.
constexpr std::uint64_t kRtSteps = 5000;
// A process's own backstop; and the ceiling on any wait, far above what it takes (rule 7).
constexpr double kRtProcessCeiling = 300.0;
constexpr double kRtCeiling = 60.0;
// A timed receive's limit where a case waits for it to pass (MS-5, MS-12): short, since a case
// waits for the clock to pass it and assumes nothing else about it.
constexpr double kRtTimed = 0.02;

#ifdef NDEBUG
constexpr bool kRtDebug = false;
#else
constexpr bool kRtDebug = true;
#endif

// The sanitizer jobs set CVC_LOCKS_SANITIZER=1; their builds run 5-15x slower.
bool rt_sanitizer() {
  const char *v = std::getenv("CVC_LOCKS_SANITIZER");
  return v && *v && std::string(v) != "0";
}

// A bulk case's size: Release, Debug, under a sanitizer.
int rt_scaled(int release, int debug, int sanitizer) {
  return rt_sanitizer() ? sanitizer : kRtDebug ? debug : release;
}

double rt_since(rt_clock::time_point t) {
  return std::chrono::duration<double>(rt_clock::now() - t).count();
}

// The pool sizes a concurrent case runs at: 1 and 2 besides 4 (clamped to the machine), as the CI
// runners' 4 vCPUs give; 2 alone under a sanitizer, whose legs run near their timeout.
std::vector<unsigned> rt_thread_counts() {
  if (rt_sanitizer())
    return {2};
  return {1, 2, 4};
}

// What `pid` came to: its result (a string bare), or "<status exit-error>".
std::string rt_outcome(const async_scheduler &sched, int pid) {
  if (const auto r = sched.get_result(pid))
    return std::holds_alternative<std::string>(r->v) ? std::get<std::string>(r->v) : to_string(*r);
  const auto i = sched.get_process_info(pid);
  return std::string("<") + (i ? to_string(i->status) : "gone") + " " +
         sched.get_exit_error(pid).value_or("") + ">";
}

std::string rt_state(const async_scheduler &sched, int pid) {
  const auto i = sched.get_process_info(pid);
  return i ? to_string(i->status) : "gone";
}

bool rt_finished(const async_scheduler &sched, int pid) {
  const auto i = sched.get_process_info(pid);
  return !i || is_finished(i->status);
}

// Programs. Keys are resolved keys unless a case gives its world a chroot.
std::string rt_recv(const std::string &key) {
  return "(get-attr (msg-recv \"" + key + "\") \"payload\")";
}
std::string rt_send(const std::string &key, const std::string &payload) {
  return "(begin (msg-send \"" + key + "\" \"" + payload + "\") \"sent\")";
}
// A timed receive: (await f seconds) on a hand-built future of `key` (td 4.6: a future channel
// follows its key), its TimeoutError caught.
std::string rt_timed_recv(const std::string &key, double seconds) {
  return "(try (get-attr (await (dict \"__future__\" \"" + key + "\") " + std::to_string(seconds) +
         ") \"payload\") (catch (TimeoutError e) \"timeout\"))";
}
std::string rt_future_recv(const std::string &key) {
  return "(get-attr (await (dict \"__future__\" \"" + key + "\")) \"payload\")";
}
// An endless loop that calls the native `fn` each time round (a target for kills and pauses).
std::string rt_loop(const std::string &fn = "") {
  return "(begin (set i 0) (while #t (begin " + (fn.empty() ? std::string() : "(" + fn + ") ") +
         "(set i (+ i 1)))))";
}

// An app whose tree charges an async call no CPU time (S1.13), with its exec_pool built first when
// `po` is given, so threads above 0 make its lock manager shared (S1-Q1).
struct rt_app {
  cvc::app app;
  explicit rt_app(std::optional<cvc::pool_options> po = std::nullopt) {
    app.root()("state_exec.defaults.limits.async_steps_per_cpu_ms").value(std::string("0"));
    // A shared lock manager, whoever pumps the groups (a thread of the case's, or the pool's).
    app.root()("state_exec.locks.threading").value(std::string("shared"));
    if (po)
      app.exec_pool(*po);
  }
  cvc::exec_pool &pool() { return app.exec_pool(); }
  exec_router &router() { return app.exec_router(); }
};

cvc::pool_options rt_pool(unsigned threads, bool deterministic = false) {
  cvc::pool_options po;
  po.threads = threads;
  po.deterministic = deterministic;
  return po;
}

// One chroot's program environment on a scheduler: the intrinsics bound to it, chrooted on the
// app's tree. Natives a case adds go in `env`.
struct rt_env {
  intrinsics_context ictx;
  environment_ptr env = builtins::make_default_environment();
};

// A scheduler of an app as a case uses it: group 0 (the app scheduler), or an exec_group named
// `name`, owned by `owner`, registering `prefixes` (exec_group::options::channel_prefixes), with
// environments by chroot. Group 0's processes this node made are killed and reaped when it goes,
// before their environments; a group goes before its environments, as group_world's do.
class rt_node {
public:
  explicit rt_node(cvc::app &app) : app_(app), sched_(&app.exec_scheduler()), name_("group 0") {}
  rt_node(cvc::app &app, const std::string &name, std::vector<std::string> prefixes,
          const std::string &owner = "",
          const std::function<void(exec_group::options &)> &tweak = {})
      : app_(app), name_(name) {
    exec_group::options o;
    o.name = name;
    o.owner = owner;
    o.app = &app;
    o.watch_root = &app.root();
    o.channel_prefixes = std::move(prefixes);
    if (tweak)
      tweak(o);
    group_ = std::make_unique<exec_group>(std::move(o));
    sched_ = &group_->sched();
  }
  rt_node(const rt_node &) = delete;
  rt_node &operator=(const rt_node &) = delete;
  ~rt_node() {
    for (auto &[chroot, e] : envs_)
      e->env.reset();
    if (group_) {
      group_.reset();
      return;
    }
    try {
      async_scheduler::pump_owner_scope token(*sched_);
      for (const int pid : pids_) {
        sched_->kill(pid);
        sched_->reap(pid);
      }
    } catch (...) {
    }
  }

  async_scheduler &sched() { return *sched_; }
  exec_group *group() { return group_.get(); }
  const std::string &name() const { return name_; }
  bool is_group0() const { return !group_; }

  environment_ptr env(const std::string &chroot = "") {
    std::unique_ptr<rt_env> &slot = envs_[chroot];
    if (!slot) {
      slot = std::make_unique<rt_env>();
      slot->ictx.sched = sched_;
      slot->ictx.proc = make_process();
      slot->ictx.proc->pid = 0;
      apply_chroot(slot->ictx, app_.root(), chroot);
      slot->ictx.host = group_ ? &group_->host() : &app_.exec_async_host();
      register_intrinsics(slot->env, &slot->ictx);
    }
    return slot->env;
  }

  // Submit, with `owner` when given (else the group's; group 0: unowned). On an attached group from
  // a host thread exec_group::execute takes the group's token itself.
  int submit(const std::string &script, const std::string &chroot = "",
             const std::string &owner = "") {
    execute_options o;
    o.env = environment::extend(env(chroot));
    o.max_time = kRtProcessCeiling;
    o.owner = owner;
    const int pid = group_ ? group_->execute(script, o) : sched_->execute(script, o);
    pids_.push_back(pid);
    return pid;
  }

  // One hand pump: the steps it took.
  std::uint64_t pump(std::uint64_t steps = kRtSteps) {
    if (group_)
      return group_->pump({steps, kRtInf}).steps;
    async_scheduler::pump_owner_scope token(*sched_);
    sched_->drain_ingress();
    sched_->wake_awaiting();
    return sched_->sync_run_steps(steps, std::nullopt);
  }
  // Whether a pump has work (between pumps, on the host thread).
  bool busy() {
    if (group_)
      return group_->wants_pump();
    const scheduler_stats s = sched_->get_stats();
    return sched_->has_pending_ingress() || s.ready + s.running > 0;
  }

  std::string outcome(int pid) { return rt_outcome(*sched_, pid); }
  std::string state(int pid) { return rt_state(*sched_, pid); }
  bool finished(int pid) { return rt_finished(*sched_, pid); }
  std::uint64_t steps_of(int pid) {
    const auto i = sched_->get_process_info(pid);
    return i ? i->step_count : 0;
  }
  bool posted() const { return sched_->has_pending_ingress(); }

private:
  cvc::app &app_;
  async_scheduler *sched_ = nullptr;
  std::string name_;
  std::map<std::string, std::unique_ptr<rt_env>> envs_; // before group_: it goes first
  std::unique_ptr<exec_group> group_;
  std::vector<int> pids_;
};

// Pump `nodes` by hand, round after round, until none has work (true), or the rounds run out.
bool rt_settle(const std::vector<rt_node *> &nodes, int rounds = 20000) {
  for (int round = 0; round < rounds; ++round) {
    for (rt_node *n : nodes)
      n->pump();
    if (std::none_of(nodes.begin(), nodes.end(), [](rt_node *n) { return n->busy(); }))
      return true;
  }
  return false;
}

void rt_wait_past(rt_clock::time_point t) {
  while (rt_clock::now() <= t)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

// Wait for `cond` (a counter, a flag), checking `give_up` too; false when the ceiling passed.
bool rt_wait_for(const std::function<bool()> &cond, double ceiling = kRtCeiling,
                 const std::function<bool()> &give_up = {}) {
  const auto t0 = rt_clock::now();
  while (!cond()) {
    if ((give_up && give_up()) || rt_since(t0) > ceiling)
      return false;
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
  return true;
}

// --- MS-12's matrix, which MS-5, MS-6 and DT-3 share ---

enum class rt_driver { hand, deterministic, threaded };

const char *rt_driver_name(rt_driver d) {
  return d == rt_driver::hand            ? "hand-pumped"
         : d == rt_driver::deterministic ? "deterministic"
                                         : "threaded";
}

// Where a cell's roles are: H, the key's home (always the first group, prefix "docs.h"); S, the
// sender; R, the receivers; Q, a second receiving group (the broadcast cells). The others' groups
// register "docs.x<i>", so the router is on in every layout, the single group's too.
struct rt_layout {
  const char *name;
  int s, r, q;
};
constexpr rt_layout kRtLayouts[] = {{"same group", 0, 0, 0},
                                    {"home remote", 1, 1, 1},
                                    {"receiver remote", 0, 1, 2},
                                    {"both remote", 1, 2, 3}};

// One cell's world: a fresh app, its group 0 and its groups, driven as `driver` says. pump(n) is
// one pump of n's group alone: by hand, or a pump_all of it (deterministic: on this thread;
// threaded: on any thread of the pool, each pump moved to another thread than the last). settle()
// pumps every group, group 0 by hand, until none has work. teardown_home() destroys H, whose keys
// then come home to group 0 (S2b.0a). Protocol expectations a cell checks on the way (a subscribe,
// send or deliver posted where the frozen protocol posts it, a counter) are collected in `misses`.
class rt_cell {
public:
  rt_cell(rt_driver driver, const rt_layout &layout, unsigned threads) : driver_(driver) {
    std::optional<cvc::pool_options> po;
    if (driver == rt_driver::deterministic) {
      po = rt_pool(0, true);
    } else if (driver == rt_driver::threaded) {
      po = rt_pool(threads);
      po->inline_below = 0;
      po->force_move = true;
    }
    app_ = std::make_unique<rt_app>(po);
    g0_ = std::make_unique<rt_node>(app_->app);
    const int n = std::max({layout.s, layout.r, layout.q}) + 1;
    for (int i = 0; i < n; ++i) {
      const std::string name = i == 0 ? "h" : "x" + std::to_string(i);
      nodes_.push_back(std::make_unique<rt_node>(app_->app, name,
                                                 std::vector<std::string>{"docs." + name}, name));
    }
    h_ = nodes_[0].get();
    s_ = nodes_[layout.s].get();
    r_ = nodes_[layout.r].get();
    q_ = nodes_[layout.q].get();
  }
  rt_cell(const rt_cell &) = delete;
  rt_cell &operator=(const rt_cell &) = delete;
  ~rt_cell() {
    while (!nodes_.empty())
      nodes_.pop_back();
    g0_.reset();
  }

  rt_node &H() { return *h_; }
  rt_node &S() { return *s_; }
  rt_node &R() { return *r_; }
  rt_node &Q() { return *q_; }
  bool remote(const rt_node &n) const { return &n != h_; }

  void pump(rt_node &n) {
    if (driver_ == rt_driver::hand || n.is_group0()) {
      n.pump();
      return;
    }
    exec_group *g = n.group();
    const auto failed = app_->pool().pump_all(std::span<exec_group *const>(&g, 1),
                                              {kRtSteps, kRtInf}, kRtNoDeadline);
    if (!failed.empty())
      misses.push_back("group " + n.name() + "'s pump failed");
  }
  void settle() {
    for (int round = 0; round < 20000; ++round) {
      if (driver_ == rt_driver::hand) {
        for (auto &n : nodes_)
          n->pump();
      } else {
        std::vector<exec_group *> gs;
        for (auto &n : nodes_)
          gs.push_back(n->group());
        if (!app_->pool().pump_all(gs, {kRtSteps, kRtInf}, kRtNoDeadline).empty())
          misses.push_back("a pump failed");
      }
      g0_->pump();
      if (!g0_->busy() &&
          std::none_of(nodes_.begin(), nodes_.end(), [](auto &n) { return n->busy(); }))
        return;
    }
    misses.push_back("the groups did not settle");
  }
  // Destroy H (its teardown forwards what it holds to group 0, S2b.0a). The cell must not touch
  // H, or S, R or Q where one is H, afterwards.
  void teardown_home() {
    nodes_.erase(std::find_if(nodes_.begin(), nodes_.end(),
                              [this](const auto &n) { return n.get() == h_; }));
    home_gone_ = true;
  }
  // The frozen protocol posts here now (td 4.6; exec_router.h): `what` names the entry.
  void expect_posted(rt_node &to, const std::string &what) {
    if (!to.posted())
      misses.push_back(what + " was not posted to group " + to.name());
  }
  void kill(rt_node &n, int pid) {
    if (!n.sched().kill(pid))
      misses.push_back("the host's kill of " + std::to_string(pid) + " found nothing");
  }
  // Where messages on `key` are queued: its home's FIFO (H, or group 0 once H is gone), and every
  // other scheduler's (none, in every layout: only a home queues).
  std::string queues(const std::string &key) {
    rt_node *home = home_gone_ ? g0_.get() : h_;
    std::size_t elsewhere = home == g0_.get() ? 0 : g0_->sched().total_pending_messages();
    for (auto &n : nodes_)
      if (n.get() != home)
        elsewhere += n->sched().total_pending_messages();
    return "queued at the home " + std::to_string(home->sched().pending_message_count(key)) +
           ", elsewhere " + std::to_string(elsewhere);
  }
  std::uint64_t sum(std::uint64_t async_scheduler::ingress_stats::*field) {
    std::uint64_t n = g0_->sched().ingress_counters().*field;
    for (auto &node : nodes_)
      n += node->sched().ingress_counters().*field;
    return n;
  }
  void expect_count(const std::string &what, std::uint64_t got, std::uint64_t want) {
    if (got != want)
      misses.push_back(what + " " + std::to_string(got) + ", not " + std::to_string(want));
  }
  // "<role>: <outcome>".
  std::string line(const std::string &role, rt_node &n, int pid) {
    return role + ": " + n.outcome(pid);
  }

  std::vector<std::string> misses;

private:
  rt_driver driver_;
  std::unique_ptr<rt_app> app_; // before g0_ and nodes_: the groups go first, then group 0's node
  std::unique_ptr<rt_node> g0_;
  std::vector<std::unique_ptr<rt_node>> nodes_;
  rt_node *h_ = nullptr, *s_ = nullptr, *r_ = nullptr, *q_ = nullptr;
  bool home_gone_ = false;
};

constexpr char kMsKey[] = "docs.h.k";

// A waiter present: R parks first (a subscribe to H), then S sends.
std::vector<std::string> ms12_waiter_present(rt_cell &c) {
  const int r = c.R().submit(rt_recv(kMsKey));
  c.pump(c.R());
  if (c.remote(c.R()))
    c.expect_posted(c.H(), "R's subscribe");
  c.pump(c.H());
  const int s = c.S().submit(rt_send(kMsKey, "present"));
  c.pump(c.S());
  if (c.remote(c.S()))
    c.expect_posted(c.H(), "S's send");
  c.settle();
  return {c.line("receiver", c.R(), r), c.line("sender", c.S(), s), c.queues(kMsKey)};
}

// No waiter: S sends, the home queues it, then R receives it from the home's FIFO.
std::vector<std::string> ms12_waiter_absent(rt_cell &c) {
  const int s = c.S().submit(rt_send(kMsKey, "absent"));
  c.pump(c.S());
  if (c.remote(c.S()))
    c.expect_posted(c.H(), "S's send");
  c.settle();
  const std::string before = c.queues(kMsKey);
  const int r = c.R().submit(rt_recv(kMsKey));
  c.settle();
  return {c.line("sender", c.S(), s), "before the receive: " + before, c.line("receiver", c.R(), r),
          c.queues(kMsKey)};
}

// A broadcast to three waiters: each gets it.
std::vector<std::string> ms12_broadcast(rt_cell &c) {
  std::vector<int> rs;
  for (int k = 0; k < 3; ++k)
    rs.push_back(c.R().submit(rt_recv(kMsKey)));
  c.pump(c.R());
  c.pump(c.H());
  const int s = c.S().submit(rt_send(kMsKey, "broadcast"));
  c.pump(c.S());
  c.settle();
  std::vector<std::string> out;
  for (const int r : rs)
    out.push_back(c.line("receiver", c.R(), r));
  out.push_back(c.line("sender", c.S(), s));
  out.push_back(c.queues(kMsKey));
  return out;
}

// A timed receive expiring with a message in flight (MS-5's cell): R parks with a limit, H records
// it, S's send reaches H's ingress before the limit wherever S is not H, the limit passes, R's
// next pump times it out (an unsubscribe to H, behind the send), and H then applies both: the
// message, delivered to R stale and requeued (or never delivered, the record past its deadline),
// is queued at H, and the next receiver gets it. Where S is H the send comes after the timeout,
// since H would deliver it to R before R's timer fires. One group: the send after the timeout.
std::vector<std::string> ms12_timed_in_flight(rt_cell &c) {
  const int r = c.R().submit(rt_timed_recv(kMsKey, kRtTimed));
  c.pump(c.R());
  const auto parked = rt_clock::now();
  if (c.remote(c.R()))
    c.expect_posted(c.H(), "R's subscribe");
  c.pump(c.H());
  int s = -1;
  if (c.remote(c.S())) {
    s = c.S().submit(rt_send(kMsKey, "in-flight"));
    c.pump(c.S());
    c.expect_posted(c.H(), "S's send");
  }
  rt_wait_past(parked + std::chrono::duration_cast<rt_clock::duration>(
                            std::chrono::duration<double>(kRtTimed)));
  c.pump(c.R()); // the limit has passed: its timer fires here, before anything reaches R
  if (!c.remote(c.S()))
    s = c.S().submit(rt_send(kMsKey, "in-flight"));
  c.settle();
  const std::string before = c.queues(kMsKey);
  const int n = c.R().submit(rt_recv(kMsKey));
  c.settle();
  const int n2 = c.R().submit(rt_recv(kMsKey));
  c.settle();
  return {c.line("receiver", c.R(), r),   c.line("sender", c.S(), s),
          "after the timeout: " + before, c.line("next receiver", c.R(), n),
          c.line("one after", c.R(), n2), c.queues(kMsKey)};
}

// The waiter killed with the delivery in flight (MS-6's cell): R parks, H records it, S's send is
// applied at H (a deliver to R), then the host kills R's process before R pumps: the deliver is
// stale, requeued to H, and the next receiver gets it. One group: killed before the send (after
// it, the waiter would already hold the message).
std::vector<std::string> ms12_killed_in_flight(rt_cell &c) {
  const int r = c.R().submit(rt_recv(kMsKey));
  c.pump(c.R());
  c.pump(c.H());
  if (!c.remote(c.R()))
    c.kill(c.R(), r);
  const int s = c.S().submit(rt_send(kMsKey, "killed"));
  c.pump(c.S());
  if (c.remote(c.S()))
    c.pump(c.H());
  if (c.remote(c.R())) {
    c.expect_posted(c.R(), "H's deliver");
    c.kill(c.R(), r);
  }
  c.settle();
  if (c.remote(c.R()))
    c.expect_count("requeued", c.sum(&async_scheduler::ingress_stats::requeued), 1);
  const std::string before = c.queues(kMsKey);
  const int n = c.R().submit(rt_recv(kMsKey));
  c.settle();
  const int n2 = c.R().submit(rt_recv(kMsKey));
  c.settle();
  return {c.line("receiver", c.R(), r),   c.line("sender", c.S(), s),
          "after the kill: " + before,    c.line("next receiver", c.R(), n),
          c.line("one after", c.R(), n2), c.queues(kMsKey)};
}

// The pending cap (max_pending_messages, applied at the home, its drops counted): 4 sends into a
// FIFO of 2 with no waiter keep the first 2.
std::vector<std::string> ms12_pending_cap(rt_cell &c) {
  c.H().sched().max_pending_messages = 2;
  const std::string k = std::string("\"") + kMsKey + "\"";
  const int s =
      c.S().submit("(begin (msg-send " + k + " \"m0\") (msg-send " + k + " \"m1\") (msg-send " + k +
                   " \"m2\") (msg-send " + k + " \"m3\") \"sent\")");
  c.settle();
  const std::string before = c.queues(kMsKey);
  const int r = c.R().submit("(list (get-attr (msg-recv " + k +
                             ") \"payload\") (get-attr (msg-recv " + k + ") \"payload\"))");
  c.settle();
  const int r3 = c.R().submit(rt_recv(kMsKey));
  c.settle();
  return {c.line("sender", c.S(), s),
          "before the receive: " + before,
          c.line("receiver", c.R(), r),
          c.line("third", c.R(), r3),
          c.queues(kMsKey),
          "dropped at the home " +
              std::to_string(c.H().sched().ingress_counters().pending_dropped)};
}

// A chroot-resolved key: every process chrooted at "docs.h", so "k" resolves to
// "docs.h.channels.k" (resolve_channel_key), homed at H.
std::vector<std::string> ms12_chroot(rt_cell &c) {
  const std::string resolved = "docs.h.channels.k";
  const int r = c.R().submit(rt_recv("k"), "docs.h");
  c.pump(c.R());
  if (c.remote(c.R()))
    c.expect_posted(c.H(), "R's subscribe");
  c.pump(c.H());
  const int s = c.S().submit(rt_send("k", "chrooted"), "docs.h");
  c.pump(c.S());
  if (c.remote(c.S()))
    c.expect_posted(c.H(), "S's send");
  c.settle();
  return {c.line("receiver", c.R(), r), c.line("sender", c.S(), s), c.queues(resolved)};
}

// A future channel: a hand-built {"__future__": key} on the routed key. Its timed receive with no
// sender times out wherever it is; an untimed one gets the next send.
std::vector<std::string> ms12_future(rt_cell &c) {
  const int r = c.R().submit(rt_timed_recv(kMsKey, kRtTimed));
  c.pump(c.R());
  const auto parked = rt_clock::now();
  if (c.remote(c.R()))
    c.expect_posted(c.H(), "R's subscribe");
  c.pump(c.H());
  rt_wait_past(parked + std::chrono::duration_cast<rt_clock::duration>(
                            std::chrono::duration<double>(kRtTimed)));
  c.pump(c.R());
  c.settle();
  const int r2 = c.R().submit(rt_future_recv(kMsKey));
  c.pump(c.R());
  if (c.remote(c.R()))
    c.expect_posted(c.H(), "R's subscribe");
  c.pump(c.H());
  const int s = c.S().submit(rt_send(kMsKey, "future"));
  c.pump(c.S());
  c.settle();
  return {c.line("timed", c.R(), r), c.line("untimed", c.R(), r2), c.line("sender", c.S(), s),
          c.queues(kMsKey)};
}

// A broadcast whose copies all go stale: no waiter at H, one each in R and Q (two remote waiters:
// tracked at H), both killed with their copies in flight. H queues it once: the next receiver
// gets it, and the one after finds nothing. One group: both killed before the send.
std::vector<std::string> ms12_broadcast_all_stale(rt_cell &c) {
  const int r = c.R().submit(rt_recv(kMsKey));
  const int q = c.Q().submit(rt_recv(kMsKey));
  c.pump(c.R());
  c.pump(c.Q());
  c.pump(c.H());
  if (!c.remote(c.R()))
    c.kill(c.R(), r);
  if (!c.remote(c.Q()))
    c.kill(c.Q(), q);
  const int s = c.S().submit(rt_send(kMsKey, "all-stale"));
  c.pump(c.S());
  if (c.remote(c.S()))
    c.pump(c.H());
  if (c.remote(c.R()))
    c.kill(c.R(), r);
  if (c.remote(c.Q()))
    c.kill(c.Q(), q);
  c.settle();
  const std::string before = c.queues(kMsKey);
  const int n = c.R().submit(rt_recv(kMsKey));
  c.settle();
  const int n2 = c.R().submit(rt_recv(kMsKey));
  c.settle();
  return {c.line("R's waiter", c.R(), r),
          c.line("Q's waiter", c.Q(), q),
          c.line("sender", c.S(), s),
          "after the kills: " + before,
          c.line("next receiver", c.R(), n),
          c.line("one after", c.R(), n2),
          c.queues(kMsKey)};
}

// A broadcast one waiter takes while the other copy goes stale: R's waiter takes it, Q's is killed
// with its copy in flight; R applies its copy first (`r_first`) or Q does. No duplicate: the next
// receiver finds nothing. One group: Q's waiter killed before the send.
std::vector<std::string> ms12_broadcast_one_taken(rt_cell &c, bool r_first) {
  const int r = c.R().submit(rt_recv(kMsKey));
  const int q = c.Q().submit(rt_recv(kMsKey));
  c.pump(c.R());
  c.pump(c.Q());
  c.pump(c.H());
  if (!c.remote(c.Q()))
    c.kill(c.Q(), q);
  const int s = c.S().submit(rt_send(kMsKey, "one-taken"));
  c.pump(c.S());
  if (c.remote(c.S()))
    c.pump(c.H());
  if (c.remote(c.Q()))
    c.kill(c.Q(), q);
  if (r_first) {
    c.pump(c.R());
    c.pump(c.Q());
  } else {
    c.pump(c.Q());
    c.pump(c.R());
  }
  c.settle();
  const int n = c.R().submit(rt_recv(kMsKey));
  c.settle();
  return {c.line("R's waiter", c.R(), r), c.line("Q's waiter", c.Q(), q),
          c.line("sender", c.S(), s), c.line("next receiver", c.R(), n), c.queues(kMsKey)};
}
std::vector<std::string> ms12_one_taken_r_first(rt_cell &c) {
  return ms12_broadcast_one_taken(c, true);
}
std::vector<std::string> ms12_one_taken_q_first(rt_cell &c) {
  return ms12_broadcast_one_taken(c, false);
}

// A local waiter at H takes the message, so its remote copy goes with requeue_if_stale false: R's
// waiter, killed with that copy in flight, drops it (counted), and nothing is queued. One group:
// R's waiter killed before the send.
std::vector<std::string> ms12_local_taker(rt_cell &c) {
  const int h = c.H().submit(rt_recv(kMsKey));
  const int r = c.R().submit(rt_recv(kMsKey));
  c.pump(c.R());
  c.pump(c.H());
  if (!c.remote(c.R()))
    c.kill(c.R(), r);
  const int s = c.S().submit(rt_send(kMsKey, "local-taker"));
  c.pump(c.S());
  if (c.remote(c.S()))
    c.pump(c.H());
  if (c.remote(c.R())) {
    c.expect_posted(c.R(), "H's deliver");
    c.kill(c.R(), r);
  }
  c.settle();
  if (c.remote(c.R()))
    c.expect_count("stale_dropped", c.sum(&async_scheduler::ingress_stats::stale_dropped), 1);
  const int n = c.R().submit(rt_recv(kMsKey));
  c.settle();
  return {c.line("H's waiter", c.H(), h), c.line("R's waiter", c.R(), r),
          c.line("sender", c.S(), s), c.line("next receiver", c.R(), n), c.queues(kMsKey)};
}

// A tracked broadcast whose home is torn down with its copies outstanding (S2b.0a: the teardown
// forwards its tracker to the new home, group 0, as `adopt`): no waiter at H, one each in R and Q,
// the send applied at H (two copies in flight), the sender's outcome read, then Q's waiter killed
// (and R's too, `all_stale`) and H destroyed before R and Q pump. One taken: R gets it, Q's stale
// copy is dropped, the next receiver finds nothing (never delivered twice). All stale: the new home
// applies it once as a requeue, and the next receiver gets it once. One group: the kills before the
// send, and no teardown (the home is everyone's group).
std::vector<std::string> ms12_broadcast_home_torn_down(rt_cell &c, bool all_stale) {
  const int r = c.R().submit(rt_recv(kMsKey));
  const int q = c.Q().submit(rt_recv(kMsKey));
  c.pump(c.R());
  c.pump(c.Q());
  c.pump(c.H());
  const bool cross = c.remote(c.R()); // R and Q are both H or neither, in every layout
  if (!cross) {
    c.kill(c.Q(), q);
    if (all_stale)
      c.kill(c.R(), r);
  }
  const int s = c.S().submit(rt_send(kMsKey, "torn-down"));
  c.pump(c.S());
  if (c.remote(c.S()))
    c.pump(c.H());
  const std::string sender = c.line("sender", c.S(), s);
  if (cross) {
    c.expect_posted(c.R(), "H's deliver to R");
    c.expect_posted(c.Q(), "H's deliver to Q");
    c.kill(c.Q(), q);
    if (all_stale)
      c.kill(c.R(), r);
    c.teardown_home(); // S too, where S is H: its outcome is read
  }
  c.pump(c.R());
  c.pump(c.Q());
  c.settle();
  const int n = c.R().submit(rt_recv(kMsKey));
  c.settle();
  const int n2 = c.R().submit(rt_recv(kMsKey));
  c.settle();
  return {c.line("R's waiter", c.R(), r),    c.line("Q's waiter", c.Q(), q), sender,
          c.line("next receiver", c.R(), n), c.line("one after", c.R(), n2), c.queues(kMsKey)};
}
std::vector<std::string> ms12_torn_down_one_taken(rt_cell &c) {
  return ms12_broadcast_home_torn_down(c, false);
}
std::vector<std::string> ms12_torn_down_all_stale(rt_cell &c) {
  return ms12_broadcast_home_torn_down(c, true);
}

struct ms12_column {
  const char *name;
  std::vector<std::string> (*run)(rt_cell &);
};
constexpr ms12_column kMs12Columns[] = {
    {"waiter present", ms12_waiter_present},
    {"waiter absent", ms12_waiter_absent},
    {"broadcast to 3 waiters", ms12_broadcast},
    {"timed receive expiring, message in flight", ms12_timed_in_flight},
    {"waiter killed, delivery in flight", ms12_killed_in_flight},
    {"pending cap", ms12_pending_cap},
    {"chroot-resolved key", ms12_chroot},
    {"future channel", ms12_future},
    {"broadcast, every remote copy stale", ms12_broadcast_all_stale},
    {"broadcast, one taken (taker first), one stale", ms12_one_taken_r_first},
    {"broadcast, one taken (stale first), one stale", ms12_one_taken_q_first},
    {"local taker at the home, remote copy stale", ms12_local_taker},
    {"broadcast, home torn down with copies out, one taken", ms12_torn_down_one_taken},
    {"broadcast, home torn down with copies out, all stale", ms12_torn_down_all_stale},
};

// One cell: its outcome, and its protocol misses (prefixed with where they happened) in `misses`.
std::vector<std::string> ms12_run(const ms12_column &column, const rt_layout &layout,
                                  rt_driver driver, unsigned threads,
                                  std::vector<std::string> &misses) {
  std::vector<std::string> out;
  std::vector<std::string> mine;
  {
    rt_cell c(driver, layout, threads);
    out = column.run(c);
    mine = c.misses;
  }
  for (const std::string &m : mine)
    misses.push_back(std::string(column.name) + ", " + layout.name + ", " + rt_driver_name(driver) +
                     ": " + m);
  return out;
}

std::string rt_join(const std::vector<std::string> &v, const char *sep = "\n  ") {
  std::string s;
  for (const std::string &x : v)
    s += sep + x;
  return s;
}

} // namespace

// MS-1 (td 8.4): a send from group S to a key homed at H, with a waiter in H, is posted to H (S's
// pump leaves it in H's ingress, the waiter still waiting) and delivered once at H's next drain:
// one pump of H ends the waiter with it, and a second receiver at H finds nothing.
TEST(Router, MS01_ASendReachesAWaiterAtItsHomeOnce) {
  rt_app a;
  rt_node h(a.app, "h", {"docs.h"}, "h"), s(a.app, "s", {"docs.s"}, "s");
  const int w = h.submit(rt_recv(kMsKey));
  h.pump();
  ASSERT_EQ(h.state(w), "waiting");
  const int snd = s.submit(rt_send(kMsKey, "ms1"));
  s.pump();
  EXPECT_EQ(s.outcome(snd), "sent");
  EXPECT_TRUE(h.posted()) << "the send was not posted to its home";
  EXPECT_EQ(h.state(w), "waiting") << "delivered before its home drained";
  h.pump();
  EXPECT_EQ(h.outcome(w), "ms1") << "not delivered at the home's next drain";
  const int w2 = h.submit(rt_recv(kMsKey));
  EXPECT_TRUE(rt_settle({&h, &s}));
  EXPECT_EQ(h.state(w2), "waiting") << "delivered twice";
  EXPECT_EQ(h.sched().pending_message_count(kMsKey), 0u);
  EXPECT_EQ(s.sched().total_pending_messages(), 0u) << "queued at the sender";
}

// MS-2: a send to H with no waiter joins H's pending FIFO (and no other group's); a receive in H
// then takes it from there.
TEST(Router, MS02_ASendWithNoWaiterIsTakenFromTheHomesQueue) {
  rt_app a;
  rt_node h(a.app, "h", {"docs.h"}, "h"), s(a.app, "s", {"docs.s"}, "s");
  const int snd = s.submit(rt_send(kMsKey, "ms2"));
  s.pump();
  EXPECT_EQ(s.outcome(snd), "sent");
  EXPECT_TRUE(h.posted()) << "the send was not posted to its home";
  h.pump();
  EXPECT_EQ(h.sched().pending_message_count(kMsKey), 1u) << "not queued at its home";
  EXPECT_EQ(s.sched().total_pending_messages(), 0u) << "queued at the sender";
  const int w = h.submit(rt_recv(kMsKey));
  h.pump();
  EXPECT_EQ(h.outcome(w), "ms2");
  EXPECT_EQ(h.sched().pending_message_count(kMsKey), 0u);
}

// MS-3: a receive in R on a key homed at H parks the process and subscribes at H; a send from S
// then reaches H, which delivers it to R: R's process gets it once.
TEST(Router, MS03_AReceiveElsewhereThenASendIsDeliveredOnce) {
  rt_app a;
  rt_node h(a.app, "h", {"docs.h"}, "h"), r(a.app, "r", {"docs.r"}, "r"),
      s(a.app, "s", {"docs.s"}, "s");
  const int w = r.submit(rt_recv(kMsKey));
  r.pump();
  ASSERT_EQ(r.state(w), "waiting");
  EXPECT_TRUE(h.posted()) << "the receive did not subscribe at the home";
  h.pump();
  const int snd = s.submit(rt_send(kMsKey, "ms3"));
  s.pump();
  EXPECT_EQ(s.outcome(snd), "sent");
  EXPECT_TRUE(h.posted()) << "the send was not posted to its home";
  h.pump();
  EXPECT_TRUE(r.posted()) << "the home did not deliver to the receiver's group";
  EXPECT_EQ(r.state(w), "waiting");
  r.pump();
  EXPECT_EQ(r.outcome(w), "ms3");
  const int w2 = r.submit(rt_recv(kMsKey));
  EXPECT_TRUE(rt_settle({&h, &r, &s}));
  EXPECT_EQ(r.state(w2), "waiting") << "delivered twice";
  EXPECT_EQ(h.sched().pending_message_count(kMsKey), 0u);
  EXPECT_EQ(r.sched().total_pending_messages() + s.sched().total_pending_messages(), 0u);
}

// MS-4: a broadcast at H with waiters in H, R1 and R2: all three get it, each group's envelope its
// own. H's waiter changes its envelope's payload in the pump that applies the send, before R1 and
// R2 pump; R1's waiter changes its own before R2 pumps: R1 and R2 still read the message as sent.
TEST(Router, MS04_ABroadcastReachesThreeGroupsEachWithItsOwnEnvelope) {
  rt_app a;
  rt_node h(a.app, "h", {"docs.h"}, "h"), r1(a.app, "r1", {"docs.r1"}, "r1"),
      r2(a.app, "r2", {"docs.r2"}, "r2"), s(a.app, "s", {"docs.s"}, "s");
  const std::string k = std::string("\"") + kMsKey + "\"";
  const int wh = h.submit("(begin (set e (msg-recv " + k +
                          ")) (set-attr e \"payload\" \"changed in h\") (get-attr e \"payload\"))");
  const int w1 = r1.submit("(begin (set e (msg-recv " + k +
                           ")) (set p (get-attr e \"payload\")) (set-attr e \"payload\" \"changed "
                           "in r1\") p)");
  const int w2 = r2.submit(rt_recv(kMsKey));
  r1.pump();
  r2.pump();
  h.pump(); // H's waiter parks; the two subscribes are recorded
  const int snd = s.submit(rt_send(kMsKey, "ms4"));
  s.pump();
  h.pump(); // the send: H's waiter takes it (and changes it), copies go to R1 and R2
  EXPECT_EQ(h.outcome(wh), "changed in h");
  EXPECT_TRUE(r1.posted() && r2.posted()) << "a remote waiter's group got no deliver";
  r1.pump();
  r2.pump();
  EXPECT_TRUE(rt_settle({&h, &r1, &r2, &s}));
  EXPECT_EQ(s.outcome(snd), "sent");
  EXPECT_EQ(r1.outcome(w1), "ms4") << "R1's envelope is H's";
  EXPECT_EQ(r2.outcome(w2), "ms4") << "R2's envelope is another group's";
  EXPECT_EQ(h.sched().pending_message_count(kMsKey), 0u);
}

// MS-5 (td 8.4; S2-Q10's re-send rule): a delivery to R in flight when R's timed wait expires, in
// the brief's order (S, R and H three groups): R parks with a limit (a subscribe), H records it,
// S's send reaches H's ingress, the limit passes, R's pump times it out (an unsubscribe, behind
// the send), H applies the send (a deliver to R, stale there, requeued; or none, R's record past
// its deadline) and the unsubscribe. R returns its timeout; the message is queued at H, not lost;
// the next receiver gets it, and the one after finds nothing.
TEST(Router, MS05_ATimedWaitExpiringWithADeliveryInFlight) {
  std::vector<std::string> misses;
  const std::vector<std::string> got =
      ms12_run(kMs12Columns[3], kRtLayouts[3], rt_driver::hand, 0, misses);
  const std::vector<std::string> want = {"receiver: timeout",
                                         "sender: sent",
                                         "after the timeout: queued at the home 1, elsewhere 0",
                                         "next receiver: in-flight",
                                         "one after: <waiting >",
                                         "queued at the home 0, elsewhere 0"};
  EXPECT_EQ(got, want) << rt_join(got);
  EXPECT_TRUE(misses.empty()) << rt_join(misses);
}

// MS-6: the same when R's process is killed, or killed and reaped, with the delivery in flight: the
// deliver is stale (the process no longer waits with that seq, or is gone), requeued at H (H's
// requeued counter 1) to the front of its FIFO, and the next receiver gets it.
TEST(Router, MS06_AKilledOrReapedWaiterRequeues) {
  for (const bool reap : {false, true}) {
    SCOPED_TRACE(reap ? "killed and reaped" : "killed");
    rt_app a;
    rt_node h(a.app, "h", {"docs.h"}, "h"), r(a.app, "r", {"docs.r"}, "r"),
        s(a.app, "s", {"docs.s"}, "s");
    const int w = r.submit(rt_recv(kMsKey));
    r.pump();
    h.pump();
    s.submit(rt_send(kMsKey, "ms6"));
    s.pump();
    h.pump();
    EXPECT_TRUE(r.posted()) << "no deliver in flight to R";
    ASSERT_TRUE(r.sched().kill(w));
    if (reap)
      EXPECT_TRUE(r.sched().reap(w));
    EXPECT_TRUE(rt_settle({&h, &r, &s}));
    EXPECT_EQ(h.sched().ingress_counters().requeued, 1u) << "the stale deliver was not requeued";
    EXPECT_EQ(h.sched().pending_message_count(kMsKey), 1u) << "lost";
    const int n = r.submit(rt_recv(kMsKey));
    EXPECT_TRUE(rt_settle({&h, &r, &s}));
    EXPECT_EQ(r.outcome(n), "ms6");
    const int n2 = r.submit(rt_recv(kMsKey));
    EXPECT_TRUE(rt_settle({&h, &r, &s}));
    EXPECT_EQ(r.state(n2), "waiting") << "delivered twice";
    EXPECT_EQ(h.sched().pending_message_count(kMsKey), 0u);
  }
}

// MS-7: 8 sender groups send N messages each to one key homed at H, received in a fourth place, R:
// every message arrives once, and each sender's in its order (FIFO per sender group). Fork-join
// (pump_all frames) and continuous (attached, workers), at threads 1, 2 and 4.
TEST(Router, MS07_EightGroupsSendingToOneKeyKeepFifoPerSender) {
  const int per = rt_scaled(1000, 200, 100);
  constexpr int kSenders = 8;
  const int total = per * kSenders;
  for (const bool continuous : {false, true}) {
    for (const unsigned threads : rt_thread_counts()) {
      SCOPED_TRACE(std::string(continuous ? "continuous" : "fork-join") + " at threads " +
                   std::to_string(threads));
      rt_app a(rt_pool(threads));
      std::mutex m;
      std::vector<std::string> got;
      std::atomic<int> count{0};
      rt_node h(a.app, "h", {"docs.h"}, "h"), r(a.app, "r", {"docs.r"}, "r");
      h.sched().max_pending_messages = 0; // unlimited: a slow receiver drops nothing
      builtins::register_fn(r.env(), "rec", [&](std::span<const value_t> args) -> value_t {
        std::lock_guard<std::mutex> lock(m);
        got.push_back(std::get<std::string>(args[0].v));
        count.fetch_add(1);
        return nil_value();
      });
      const int w = r.submit("(begin (set n 0) (while (< n " + std::to_string(total) +
                             ") (begin (rec " + rt_recv(kMsKey) + ") (set n (+ n 1)))) n)");
      std::vector<std::unique_ptr<rt_node>> senders;
      for (int g = 0; g < kSenders; ++g) {
        senders.push_back(std::make_unique<rt_node>(
            a.app, "s" + std::to_string(g), std::vector<std::string>{"docs.s" + std::to_string(g)},
            "s" + std::to_string(g)));
        senders.back()->submit("(begin (set j 0) (while (< j " + std::to_string(per) +
                               ") (begin (msg-send \"" + kMsKey + "\" (str-concat \"" +
                               std::to_string(g) + ":\" (str j))) (set j (+ j 1)))) \"sent\")");
      }
      std::vector<exec_group *> all{h.group(), r.group()};
      for (auto &sn : senders)
        all.push_back(sn->group());
      // Until every message has arrived, or none has for a while (a hang guard, rule 7).
      const auto t0 = rt_clock::now();
      int seen = 0;
      auto last_progress = t0;
      const auto going = [&] {
        const int now = count.load();
        if (now != seen) {
          seen = now;
          last_progress = rt_clock::now();
        }
        return now < total && rt_since(last_progress) < kRtCeiling / 3;
      };
      if (!continuous) {
        while (going())
          a.pool().pump_all(all, {kRtSteps, kRtInf}, kRtNoDeadline);
      } else {
        for (exec_group *g : all)
          a.pool().attach(*g);
        while (going())
          if (!a.pool().pump_host(kRtInf))
            a.pool().wait_for_work(0.01);
        for (exec_group *g : all)
          a.pool().detach(*g);
      }
      std::printf("MS-7: %s at threads %u (%u workers): %d of %d messages in %.3f s\n",
                  continuous ? "continuous" : "fork-join", threads, a.pool().threads(),
                  count.load(), total, rt_since(t0));
      std::fflush(stdout);
      ASSERT_EQ(count.load(), total) << "lost (none arrived for " << kRtCeiling / 3 << " s)";
      EXPECT_EQ(r.outcome(w), std::to_string(total));
      std::vector<int> next(kSenders, 0);
      int disorder = 0;
      for (const std::string &x : got) {
        const auto colon = x.find(':');
        const int g = std::stoi(x.substr(0, colon));
        const int j = std::stoi(x.substr(colon + 1));
        if (j != next[g])
          ++disorder;
        next[g] = j + 1;
      }
      EXPECT_EQ(disorder, 0) << "out of order within a sender, duplicated or lost";
      for (int g = 0; g < kSenders; ++g)
        EXPECT_EQ(next[g], per) << "sender " << g;
      EXPECT_EQ(h.sched().pending_message_count(kMsKey), 0u);
    }
  }
}

// MS-7, across a home's teardown (S2b.0a: the teardown forwards the pending messages of the keys it
// homed, then its records, then its queued entries, in order, to the keys' new home, group 0, and
// unregisters last): S sends m0-m4 to H's key with no waiter (H queues them), then m5-m9 (still in
// H's ingress), and R2's receive on a second key of H's is recorded there. H is destroyed. S then
// sends m10-m12 (now to group 0). A receiver in R takes m0-m12 in S's order, nothing lost or
// doubled; and S's send on the second key reaches R2's forwarded wait at group 0.
TEST(Router, MS07b_FifoPerSenderHoldsAcrossItsHomesTeardown) {
  rt_app a;
  rt_node g0(a.app), s(a.app, "s", {"docs.s"}, "s"), r(a.app, "r", {"docs.r"}, "r"),
      r2(a.app, "r2", {"docs.r2"}, "r2");
  auto h = std::make_unique<rt_node>(a.app, "h", std::vector<std::string>{"docs.h"}, "h");
  const auto sends = [&](int from, int to) {
    std::string body;
    for (int i = from; i < to; ++i)
      body += std::string(" (msg-send \"") + kMsKey + "\" \"m" + std::to_string(i) + "\")";
    return s.submit("(begin" + body + " \"sent\")");
  };
  const int first = sends(0, 5);
  s.pump();
  h->pump();
  EXPECT_EQ(h->sched().pending_message_count(kMsKey), 5u);
  const int second = sends(5, 10);
  s.pump(); // m5-m9 wait in H's ingress
  const int w2 = r2.submit(rt_recv("docs.h.k2"));
  r2.pump();
  h->pump(); // applies m5-m9 and R2's subscribe: 10 queued, one record
  const int third_early = sends(10, 12);
  s.pump(); // m10, m11 queued in H's ingress at its teardown
  h.reset();
  EXPECT_TRUE(a.router().home_of(kMsKey).is_group0());
  const int third = sends(12, 13); // m12, to the key's new home
  s.pump();
  std::vector<std::string> got;
  builtins::register_fn(r.env(), "got", [&](std::span<const value_t> args) -> value_t {
    got.push_back(std::get<std::string>(args[0].v));
    return nil_value();
  });
  const int w = r.submit(std::string("(begin (for i (range 13) (got ") + rt_recv(kMsKey).c_str() +
                         ")) \"done\")");
  const int k2 = s.submit(rt_send("docs.h.k2", "to r2"));
  EXPECT_TRUE(rt_settle({&g0, &s, &r, &r2}));
  for (const int pid : {first, second, third_early, third, k2})
    EXPECT_EQ(s.outcome(pid), "sent");
  EXPECT_EQ(r.outcome(w), "done");
  std::vector<std::string> want;
  for (int i = 0; i < 13; ++i)
    want.push_back("m" + std::to_string(i));
  EXPECT_EQ(got, want) << "out of S's order, lost or doubled across the teardown";
  EXPECT_EQ(r2.outcome(w2), "to r2") << "R2's wait was not forwarded";
  EXPECT_EQ(g0.sched().total_pending_messages(), 0u);
}

// MS-8: an app-global key ("/ms8.global" from a document group's chroot) is homed at group 0 for a
// group with a prefix: a receiver there gets the send. And (S2.md section 0, row 22;
// async_scheduler::has_pending_ingress) a send homed at group 0 when group 0 has no live process is
// not stranded: app::exec_pump_frame pumps group 0 for it, and it is queued there. A group with no
// prefix is an island (S2b.0a): with the router on, its unprefixed keys stay its own (its receiver
// gets its send, nothing is posted to group 0), while its send on a key under D's prefix is routed
// to D.
TEST(Router, MS08_AnAppGlobalKeyIsHomedAtGroupZero) {
  rt_app a;
  rt_node g0(a.app), d(a.app, "d", {"docs.d"}, "d");
  EXPECT_TRUE(a.router().home_of("ms8.global").is_group0());
  const int w = g0.submit(rt_recv("ms8.global"));
  g0.pump();
  ASSERT_EQ(g0.state(w), "waiting");
  const int snd = d.submit(rt_send("/ms8.global", "ms8"), "docs.d");
  d.pump();
  EXPECT_EQ(d.outcome(snd), "sent");
  EXPECT_TRUE(g0.posted()) << "the send was not posted to group 0";
  g0.pump();
  EXPECT_EQ(g0.outcome(w), "ms8");
  EXPECT_EQ(d.sched().total_pending_messages(), 0u) << "queued in the document's group";

  // Group 0 with no live process now (its receiver ended): a frame's driver still drains the send.
  const int later = d.submit(rt_send("/ms8.later", "later"), "docs.d");
  d.pump();
  EXPECT_EQ(d.outcome(later), "sent");
  EXPECT_TRUE(g0.posted());
  a.app.exec_pump_frame();
  EXPECT_FALSE(g0.posted()) << "exec_pump_frame left a router entry in group 0's ingress";
  EXPECT_EQ(g0.sched().pending_message_count("ms8.later"), 1u);
  const int w2 = g0.submit(rt_recv("ms8.later"));
  g0.pump();
  EXPECT_EQ(g0.outcome(w2), "later");

  // An island beside them.
  rt_node island(a.app, "island", {}, "island");
  ASSERT_TRUE(a.router().active());
  EXPECT_FALSE(a.router().has_prefix(island.group()->route()));
  const int own = island.submit(rt_recv("ms8.island"));
  const int to_d = d.submit(rt_recv("docs.d.k"));
  island.pump();
  d.pump();
  const int own_send = island.submit(rt_send("ms8.island", "kept"));
  const int routed = island.submit(rt_send("docs.d.k", "routed"));
  island.pump();
  EXPECT_FALSE(g0.posted()) << "an island's own key went to group 0";
  EXPECT_TRUE(d.posted()) << "an island's send on D's key was not posted to D";
  EXPECT_TRUE(rt_settle({&g0, &d, &island}));
  EXPECT_EQ(island.outcome(own_send), "sent");
  EXPECT_EQ(island.outcome(routed), "sent");
  EXPECT_EQ(island.outcome(own), "kept");
  EXPECT_EQ(d.outcome(to_d), "routed");
  EXPECT_EQ(g0.sched().total_pending_messages(), 0u);
}

// MS-9: a '#' key is never routed. A receiver in document group D on "docs.d#tick" gets the host's
// posts to D; another group's msg-send on the same key stays in that group (its own receiver gets
// it, D's does not); and the router is never consulted for it.
TEST(Router, MS09_AHashKeyStaysInTheGroupThatResolvesIt) {
  rt_app a;
  rt_node d(a.app, "d", {"docs.d"}, "d"), e(a.app, "e", {"docs.e"}, "e");
  const std::string tick = "docs.d#tick";
  const std::uint64_t consulted = a.router().consulted();
  const int w = d.submit("(begin (set m (msg-recv \"" + tick + "\")) m)");
  d.pump();
  EXPECT_FALSE(e.posted());
  EXPECT_TRUE(d.sched().post_message(tick, value_t(std::string("t1"))));
  d.pump();
  EXPECT_EQ(d.outcome(w), "t1") << "the host's post did not reach the group";
  const int wd = d.submit(rt_recv(tick));
  const int we = e.submit(rt_recv(tick));
  d.pump();
  e.pump();
  const int snd = e.submit(rt_send(tick, "from e"));
  e.pump();
  EXPECT_FALSE(d.posted()) << "a '#' send was posted to another group";
  EXPECT_TRUE(rt_settle({&d, &e}));
  EXPECT_EQ(e.outcome(snd), "sent");
  EXPECT_EQ(e.outcome(we), "from e");
  EXPECT_EQ(d.state(wd), "waiting");
  EXPECT_EQ(a.router().consulted(), consulted) << "the router was consulted for a '#' key";
}

// MS-10 (S2-Q10's cap): H's ingress at its cap (set_ingress_cap(3)). S's fourth and fifth msg-send
// to H's key are refused: {status "error", reason "ingress full"} (the first three carry no
// reason); host posts are refused (false): async_scheduler::post_message on H, the pool's, and S's
// post_message forwarded to H. Each counts in H's ingress_refused. Once H drains, posts are taken
// again, and a receiver at H gets the 3 sends and the later posts in order.
TEST(Router, MS10_AFullIngressRefusesSendsAndPosts) {
  rt_app a(rt_pool(0));
  rt_node h(a.app, "h", {"docs.h"}, "h"), s(a.app, "s", {"docs.s"}, "s");
  h.sched().set_ingress_cap(3);
  std::vector<std::string> notes;
  builtins::register_fn(s.env(), "note", [&](std::span<const value_t> args) -> value_t {
    notes.push_back(std::get<std::string>(args[0].v));
    return nil_value();
  });
  const int snd = s.submit(
      std::string("(begin (for i (range 5) (begin (set r (msg-send \"") + kMsKey +
      "\" (str-concat \"m\" (str i)))) (note (try (str-concat (get-attr r \"status\") \" \" "
      "(get-attr r \"reason\")) (catch (KeyError e) \"no reason\"))))) \"sent\")");
  s.pump();
  EXPECT_EQ(s.outcome(snd), "sent");
  const std::vector<std::string> want = {"no reason", "no reason", "no reason",
                                         "error ingress full", "error ingress full"};
  EXPECT_EQ(notes, want);
  const value_t v(std::string("host"));
  EXPECT_FALSE(h.sched().post_message(kMsKey, v)) << "a host post past the cap";
  EXPECT_FALSE(a.pool().post_message(kMsKey, v)) << "the pool's post past the cap";
  EXPECT_FALSE(s.sched().post_message(kMsKey, v)) << "a forwarded post past the cap";
  EXPECT_EQ(h.sched().ingress_counters().ingress_refused, 5u);
  h.pump();
  EXPECT_EQ(h.sched().pending_message_count(kMsKey), 3u);
  EXPECT_TRUE(h.sched().post_message(kMsKey, value_t(std::string("p1"))));
  EXPECT_TRUE(a.pool().post_message(kMsKey, value_t(std::string("p2"))));
  std::vector<std::string> got;
  builtins::register_fn(h.env(), "got", [&](std::span<const value_t> args) -> value_t {
    const value_t &m = args[0];
    if (const auto *d = std::get_if<dict_ptr>(&m.v))
      for (const auto &[k, v] : **d)
        if (k == "payload")
          got.push_back(std::get<std::string>(v.v));
    if (const auto *str = std::get_if<std::string>(&m.v))
      got.push_back(*str);
    return nil_value();
  });
  const int w = h.submit(std::string("(begin (for i (range 5) (got (msg-recv \"") + kMsKey +
                         "\"))) \"done\")");
  EXPECT_TRUE(rt_settle({&h, &s}));
  EXPECT_EQ(h.outcome(w), "done");
  const std::vector<std::string> order = {"m0", "m1", "m2", "p1", "p2"};
  EXPECT_EQ(got, order);
}

// MS-12 (td 8.4; S2.md 4): the semantics matrix. Every column (scenario) in every layout of S, R
// and H, each outcome equal to the same programs' in one group (requeue order being the documented
// exception, which no cell depends on: one message in flight at a time); hand-pumped and in the
// pool's deterministic mode (each step a pump_all of one group on this thread), the two equal too.
TEST(Router, MS12_SemanticsMatrix) {
  std::vector<std::string> misses;
  for (const ms12_column &column : kMs12Columns) {
    std::vector<std::string> hand_oracle;
    for (const rt_driver driver : {rt_driver::hand, rt_driver::deterministic}) {
      const std::vector<std::string> oracle = ms12_run(column, kRtLayouts[0], driver, 0, misses);
      if (driver == rt_driver::hand)
        hand_oracle = oracle;
      else
        EXPECT_EQ(oracle, hand_oracle) << column.name << ": deterministic mode's single group";
      for (int l = 1; l < 4; ++l) {
        const std::vector<std::string> got = ms12_run(column, kRtLayouts[l], driver, 0, misses);
        EXPECT_EQ(got, oracle) << column.name << ", " << kRtLayouts[l].name << ", "
                               << rt_driver_name(driver) << ":" << rt_join(got)
                               << "\n against the single group's:" << rt_join(oracle);
      }
    }
  }
  EXPECT_TRUE(misses.empty()) << rt_join(misses);
}

// PL-6 (td 8.3; S2-Q8): exec_pool::post_control's kill while the target pumps (a thread pumping
// it by hand, or a worker of the attached group) lands within one more step of the target: its
// process, stopped in a native at its 50th call while the host posts, takes at most one more step
// (its step count) and never calls the native again; the command's done is called once, true. And
// `(kill p)` from a process of the same owner in another group returns #t, and p is killed at its
// group's next drain, with no more step, as the host's kill left its target; pause and resume
// across groups give #t and take effect there.
TEST(Router, PL06_ControlAcrossGroups) {
  std::string host_killed_outcome;
  for (const bool attached : {false, true}) {
    SCOPED_TRACE(attached ? "attached, a worker pumping" : "a thread pumping by hand");
    rt_app a(rt_pool(attached ? 2 : 0));
    if (attached && a.pool().threads() == 0) {
      ADD_FAILURE() << "the pool has no worker";
      continue;
    }
    std::atomic<int> phase{0}, calls{0};
    std::uint64_t steps_at_post = 0;
    rt_node t(a.app, "t", {"docs.t"}, "doc1");
    builtins::register_fn(t.env(), "gate", [&](std::span<const value_t>) -> value_t {
      if (calls.fetch_add(1) + 1 == 50) {
        const auto i = t.sched().get_process_info(t.sched().current_pid());
        steps_at_post = i ? i->step_count : 0;
        phase.store(1);
        rt_wait_for([&] { return phase.load() == 2; });
      }
      return nil_value();
    });
    const int p = t.submit(rt_loop("gate"));
    std::atomic<bool> stop{false};
    std::thread pumper;
    if (attached)
      a.pool().attach(*t.group());
    else
      pumper = std::thread([&] {
        const auto t0 = rt_clock::now();
        while (!stop.load() && !t.finished(p) && rt_since(t0) < kRtCeiling)
          t.pump(100);
      });
    const bool reached = rt_wait_for([&] { return phase.load() == 1; });
    EXPECT_TRUE(reached) << "the target never reached its 50th call";
    std::atomic<int> done_calls{0};
    std::atomic<bool> done_value{false};
    control_cmd cmd;
    cmd.what = control_cmd::kind::kill;
    cmd.done = [&](bool ok) {
      done_value.store(ok);
      done_calls.fetch_add(1);
    };
    const bool posted = a.pool().post_control(p, std::move(cmd));
    phase.store(2);
    EXPECT_TRUE(posted) << "post_control refused a live pid";
    if (attached) {
      const auto t0 = rt_clock::now();
      bool ended = false;
      while (posted && !ended && rt_since(t0) < kRtCeiling) {
        if (!a.pool().pump_host(kRtInf))
          a.pool().wait_for_work(0.01);
        a.pool().with_group(*t.group(), [&] { ended = t.finished(p); });
      }
      a.pool().detach(*t.group());
    } else {
      if (!posted)
        stop.store(true);
      pumper.join();
    }
    if (!posted)
      continue;
    EXPECT_EQ(t.state(p), "killed");
    EXPECT_LE(t.steps_of(p), steps_at_post + 1) << "more than one step after the post";
    EXPECT_EQ(calls.load(), 50) << "the target called its native again after the post";
    EXPECT_EQ(done_calls.load(), 1);
    EXPECT_TRUE(done_value.load());
    host_killed_outcome = t.outcome(p);
  }

  // A program's kill, pause and resume of a pid in another group of its owner.
  rt_app a;
  rt_node ta(a.app, "a", {"docs.a"}, "doc1"), tb(a.app, "b", {"docs.b"}, "doc1");
  const int p = ta.submit(rt_loop());
  const int p3 = ta.submit(rt_loop());
  ta.pump(200);
  const int ctl =
      tb.submit("(str-concat (if (kill " + std::to_string(p) +
                ") \"yes\" \"no\") \" \" (if (pause " + std::to_string(p3) + ") \"yes\" \"no\"))");
  tb.pump();
  EXPECT_EQ(tb.outcome(ctl), "yes yes") << "kill and pause across groups";
  EXPECT_TRUE(ta.posted()) << "nothing was posted to the target's group";
  const std::uint64_t before = ta.steps_of(p);
  ta.pump(1);
  EXPECT_EQ(ta.state(p), "killed");
  EXPECT_LE(ta.steps_of(p), before + 1);
  if (!host_killed_outcome.empty())
    EXPECT_EQ(ta.outcome(p), host_killed_outcome) << "not the host's kill's result";
  EXPECT_EQ(ta.state(p3), "paused");
  const std::uint64_t paused_at = ta.steps_of(p3);
  ta.pump(200);
  EXPECT_EQ(ta.steps_of(p3), paused_at) << "a paused process stepped";
  const int res = tb.submit("(if (resume " + std::to_string(p3) + ") \"yes\" \"no\")");
  tb.pump();
  EXPECT_EQ(tb.outcome(res), "yes");
  ta.pump(200);
  EXPECT_GT(ta.steps_of(p3), paused_at) << "resumed across groups, but it did not step";
}

// PL-7 (S2-Q7, S2-Q8): a program cannot reach another owner's pid. Across groups, a doc2 process's
// kill, pause and resume of a doc1 pid give #f and post nothing, and so do an unowned process's
// (owner "" has no rights across groups); the target runs on. In group 0, a doc2 process cannot
// kill or pause a doc1 process, nor see it in ps or inspect; doc1's own process and an unowned
// one still can (the local rule).
TEST(Router, PL07_AnotherOwnersPid) {
  rt_app a;
  rt_node ta(a.app, "a", {"docs.a"}, "doc1"), tb(a.app, "b", {"docs.b"}, "doc2"),
      tc(a.app, "c", {"docs.c"}, "");
  const int p = ta.submit(rt_loop());
  ta.pump(100);
  const std::string pid = std::to_string(p);
  const std::string all_three = "(str-concat (if (kill " + pid +
                                ") \"yes\" \"no\") \" \" (if (pause " + pid +
                                ") \"yes\" \"no\") \" \" (if (resume " + pid + ") \"yes\" \"no\"))";
  const int other = tb.submit(all_three);
  const int unowned = tc.submit(all_three);
  tb.pump();
  tc.pump();
  EXPECT_EQ(tb.outcome(other), "no no no") << "another owner";
  EXPECT_EQ(tc.outcome(unowned), "no no no") << "an unowned caller";
  EXPECT_FALSE(ta.posted()) << "something was posted to the target's group";
  const std::uint64_t before = ta.steps_of(p);
  ta.pump(100);
  EXPECT_EQ(ta.state(p), "ready");
  EXPECT_GT(ta.steps_of(p), before) << "the target stopped";

  // Group 0: two documents' processes and an unowned one.
  rt_node g0(a.app);
  const int q1 = g0.submit(rt_loop(), "", "doc1");
  const int q1b = g0.submit(rt_loop(), "", "doc1");
  g0.pump(100);
  const std::string q = std::to_string(q1);
  const int nosy =
      g0.submit("(begin (set listed \"unlisted\") (for row (ps) (if (= (get-attr row \"pid\") " +
                    q + ") (set listed \"listed\"))) (str-concat (if (kill " + q +
                    ") \"yes\" \"no\") \" \" (if (pause " + q +
                    ") \"yes\" \"no\") \" \" (if (is-null (inspect " + q +
                    ")) \"hidden\" \"seen\") \" \" listed))",
                "", "doc2");
  g0.pump();
  EXPECT_EQ(g0.outcome(nosy), "no no hidden unlisted") << "another document's process in group 0";
  EXPECT_EQ(g0.state(q1), "ready") << "another owner's kill or pause took effect";
  const int own = g0.submit("(if (kill " + q + ") \"yes\" \"no\")", "", "doc1");
  const int anyone = g0.submit("(if (kill " + std::to_string(q1b) + ") \"yes\" \"no\")", "", "");
  g0.pump();
  EXPECT_EQ(g0.outcome(own), "yes") << "the owner's own process";
  EXPECT_EQ(g0.outcome(anyone), "yes") << "an unowned process";
  EXPECT_EQ(g0.state(q1), "killed");
  EXPECT_EQ(g0.state(q1b), "killed");
}

namespace {

// PL-8's world: 4 groups (owner "other") each running two processes of owner "doc" and one of
// "other", and group 0 one of each. A "doc" process calls (touch-doc n), which reads the object
// its context points at (`context`, which the host frees once kill_owner has returned) unless
// `freed` is set, when it counts a violation; an "other" process calls (touch-other n), which
// touches nothing of the host's. Counts per process.
struct pl08_world {
  static constexpr int kGroups = 4;
  rt_app a;
  std::unique_ptr<std::atomic<std::int64_t>> context =
      std::make_unique<std::atomic<std::int64_t>>(0);
  std::atomic<std::int64_t> *context_raw = context.get();
  std::atomic<bool> freed{false};
  std::atomic<int> violations{0};
  std::vector<std::unique_ptr<std::atomic<std::int64_t>>> touches;
  // After `touches` and `context`, so every process goes first.
  std::unique_ptr<rt_node> g0;
  std::vector<std::unique_ptr<rt_node>> groups;
  struct proc {
    rt_node *node;
    int pid;
    bool doc;
    int tag;
  };
  std::vector<proc> procs;

  explicit pl08_world(unsigned threads) : a(rt_pool(threads)) {
    g0 = std::make_unique<rt_node>(a.app);
    for (int k = 0; k < kGroups; ++k)
      groups.push_back(std::make_unique<rt_node>(
          a.app, "pl8g" + std::to_string(k),
          std::vector<std::string>{"docs.pl8g" + std::to_string(k)}, "other"));
    add_natives(*g0);
    for (auto &g : groups)
      add_natives(*g);
    add(*g0, true);
    add(*g0, false);
    for (auto &g : groups) {
      add(*g, true);
      add(*g, true);
      add(*g, false);
    }
  }
  void add_natives(rt_node &n) {
    builtins::register_fn(n.env(), "touch-doc", [this](std::span<const value_t> args) -> value_t {
      if (freed.load(std::memory_order_acquire))
        violations.fetch_add(1);
      else
        context_raw->fetch_add(1);
      touches[std::get<std::int64_t>(args[0].v)]->fetch_add(1);
      return nil_value();
    });
    builtins::register_fn(n.env(), "touch-other", [this](std::span<const value_t> args) -> value_t {
      touches[std::get<std::int64_t>(args[0].v)]->fetch_add(1);
      return nil_value();
    });
  }
  void add(rt_node &n, bool doc) {
    const int tag = static_cast<int>(touches.size());
    touches.push_back(std::make_unique<std::atomic<std::int64_t>>(0));
    const std::string fn = (doc ? "touch-doc " : "touch-other ") + std::to_string(tag);
    procs.push_back({&n, n.submit(rt_loop(fn), "", doc ? "doc" : "other"), doc, tag});
  }
  std::vector<exec_group *> exec_groups() {
    std::vector<exec_group *> gs;
    for (auto &g : groups)
      gs.push_back(g->group());
    return gs;
  }
  bool every_process_touched(std::int64_t at_least) {
    return std::all_of(procs.begin(), procs.end(),
                       [&](const proc &p) { return touches[p.tag]->load() >= at_least; });
  }
  void pump_group0() { g0->pump(200); }
};

} // namespace

// PL-8 (td 4.5; S2-Q8): exec_pool::kill_owner("doc") with the owner's processes in 4 groups and
// group 0, then the host frees what their contexts point at. Every group acknowledged first: when
// kill_owner returns, each "doc" process is killed (its group drained inline, or waited for), so
// none touches the freed object after (no violation; ASan in CI sees no use after free); the
// "other" processes run on. Fork-join with nothing pumping the groups, fork-join with another
// thread pumping them meanwhile (pump_all frames), and continuous (attached, workers pumping);
// group 0 has no pump owner, so its ingress is drained inline. At threads 1 and 2.
TEST(Router, PL08_KillOwnerIsAcknowledged) {
  enum class mode { idle, pumped_elsewhere, continuous };
  for (const mode md : {mode::idle, mode::pumped_elsewhere, mode::continuous}) {
    for (const unsigned threads : {1u, 2u}) {
      const char *name = md == mode::idle               ? "fork-join, idle"
                         : md == mode::pumped_elsewhere ? "fork-join, pumped elsewhere"
                                                        : "continuous";
      SCOPED_TRACE(std::string(name) + " at threads " + std::to_string(threads));
      pl08_world w(threads);
      std::vector<exec_group *> gs = w.exec_groups();
      // Warm up: every process has touched.
      const auto t0 = rt_clock::now();
      if (md == mode::continuous)
        for (exec_group *g : gs)
          w.a.pool().attach(*g);
      while (!w.every_process_touched(1) && rt_since(t0) < kRtCeiling) {
        w.pump_group0();
        if (md == mode::continuous) {
          if (!w.a.pool().pump_host(0.001))
            w.a.pool().wait_for_work(0.001);
        } else {
          w.a.pool().pump_all(gs, {200, kRtInf}, kRtNoDeadline);
        }
      }
      ASSERT_TRUE(w.every_process_touched(1)) << "the processes never started";
      std::atomic<bool> stop{false};
      std::atomic<int> frames{0};
      std::thread elsewhere;
      if (md == mode::pumped_elsewhere) {
        elsewhere = std::thread([&] {
          while (!stop.load()) {
            w.a.pool().pump_all(gs, {200, kRtInf}, kRtNoDeadline);
            frames.fetch_add(1);
          }
        });
        rt_wait_for([&] { return frames.load() >= 2; });
      }
      bool returned = false;
      try {
        w.a.pool().kill_owner("doc");
        returned = true;
      } catch (const std::exception &e) {
        ADD_FAILURE() << "kill_owner threw: " << e.what();
      }
      if (returned) {
        // The host frees what the "doc" processes' contexts point at.
        w.freed.store(true, std::memory_order_release);
        w.context.reset();
      }
      // The rest run on a while: each "other" process touches 50 more times.
      std::vector<std::int64_t> at;
      for (const auto &p : w.procs)
        at.push_back(w.touches[p.tag]->load());
      const auto more = [&] {
        for (std::size_t i = 0; i < w.procs.size(); ++i)
          if (!w.procs[i].doc && w.touches[w.procs[i].tag]->load() < at[i] + 50)
            return false;
        return true;
      };
      const auto t1 = rt_clock::now();
      while (!more() && rt_since(t1) < kRtCeiling) {
        w.pump_group0();
        if (md == mode::continuous) {
          if (!w.a.pool().pump_host(0.001))
            w.a.pool().wait_for_work(0.001);
        } else if (md == mode::idle) {
          w.a.pool().pump_all(gs, {200, kRtInf}, kRtNoDeadline);
        } else {
          std::this_thread::yield();
        }
      }
      stop.store(true);
      if (elsewhere.joinable())
        elsewhere.join();
      if (md == mode::continuous)
        for (exec_group *g : gs)
          w.a.pool().detach(*g);
      EXPECT_TRUE(more()) << "an \"other\" process stopped";
      EXPECT_EQ(w.violations.load(), 0) << "a \"doc\" process ran after kill_owner returned";
      for (std::size_t i = 0; i < w.procs.size(); ++i) {
        const auto &p = w.procs[i];
        if (p.doc && returned)
          EXPECT_EQ(p.node->state(p.pid), "killed") << p.node->name() << " pid " << p.pid;
        else if (!p.doc)
          EXPECT_FALSE(p.node->finished(p.pid)) << p.node->name() << " pid " << p.pid;
      }
    }
  }
}

// KO-3 (S2-Q8): a group destroyed while kill_owner waits on it. Its one "doc" process is stopped in
// a native on a worker (pool pump_steps 1, so that pump ends with that step); kill_owner, on
// another thread, posts its command and waits; the group's attaching thread begins its teardown
// (detaching set) and only then is the native let go: the pump ends without draining the command,
// the teardown refuses it (the destructor's done(false)), and the wait ends, refused, not hung.
// The teardown hook sees the process not yet killed, so the command was never applied.
TEST(Router, KO03_KillOwnerSurvivesADyingGroup) {
  for (const unsigned threads : {1u, 2u}) {
    SCOPED_TRACE("threads " + std::to_string(threads));
    cvc::pool_options po = rt_pool(threads);
    po.pump_steps = 1;
    rt_app a(po);
    if (a.pool().threads() == 0) {
      ADD_FAILURE() << "the pool has no worker";
      return;
    }
    std::atomic<int> phase{0}; // 1: in the native; 2: let go
    std::atomic<bool> destroy{false}, destroyed{false}, ko_done{false};
    std::string status_at_teardown;
    rt_node *victim = nullptr;
    std::atomic<rt_node *> victim_ready{nullptr};
    // The group's attaching thread: makes it, attaches it, services its host queue, destroys it.
    std::thread owner([&] {
      std::unique_ptr<rt_node> g;
      int pid = -1;
      g = std::make_unique<rt_node>(a.app, "ko3", std::vector<std::string>{"docs.ko3"}, "doc",
                                    [&](exec_group::options &o) {
                                      o.teardown_hook_for_test = [&](exec_group &grp) {
                                        status_at_teardown = rt_state(grp.sched(), pid);
                                      };
                                    });
      builtins::register_fn(g->env(), "gate", [&](std::span<const value_t>) -> value_t {
        if (phase.load() == 0) {
          phase.store(1);
          rt_wait_for([&] { return phase.load() == 2; });
        }
        return nil_value();
      });
      pid = g->submit(rt_loop("gate"));
      a.pool().attach(*g->group());
      victim_ready.store(g.get());
      const auto t0 = rt_clock::now();
      while (!destroy.load() && rt_since(t0) < kRtCeiling)
        a.pool().wait_for_work(0.005);
      g.reset(); // attached: the teardown detaches it first
      destroyed.store(true);
    });
    const bool started =
        rt_wait_for([&] { return phase.load() == 1 && victim_ready.load() != nullptr; });
    EXPECT_TRUE(started) << "the process never reached its native";
    victim = victim_ready.load();
    // Read before the teardown begins: rt_node's group pointer is cleared as it goes, while the
    // group itself lives until its teardown has finished, which waits for the native.
    exec_group *const vg = victim ? victim->group() : nullptr;
    async_scheduler *const vs = victim ? &victim->sched() : nullptr;
    std::thread killer([&] {
      try {
        a.pool().kill_owner("doc");
      } catch (const std::exception &e) {
        ADD_FAILURE() << "kill_owner threw: " << e.what();
      }
      ko_done.store(true);
    });
    const bool posted =
        started &&
        rt_wait_for([&] { return vs->has_pending_control() || vs->has_pending_ingress(); },
                    kRtCeiling, [&] { return ko_done.load(); });
    EXPECT_TRUE(posted) << "kill_owner posted nothing to the group";
    destroy.store(true);
    if (vg)
      rt_wait_for(
          [&] { return (vg->pool_link().state.load() & detail::group_pool_link::detaching) != 0; });
    phase.store(2);
    const bool ended = rt_wait_for([&] { return ko_done.load(); });
    if (!ended) {
      std::fprintf(stderr, "KO-3: kill_owner is still waiting on a destroyed group (hung)\n");
      std::abort();
    }
    killer.join();
    owner.join();
    EXPECT_TRUE(destroyed.load());
    if (posted)
      EXPECT_NE(status_at_teardown, "killed") << "the command was applied before the teardown";
  }
}

namespace {

// RT-1's programs: a ping-pong of 20 rounds, a broadcast to two receivers, msg-pending, and
// receivers of two host posts.
std::vector<std::string> rt01_programs() {
  return {
      "(begin (set n 0) (while (< n 20) (begin (set m (msg-recv \"rt1.ping\")) (msg-send "
      "\"rt1.pong\" (get-attr m \"payload\")) (set n (+ n 1)))) n)",
      "(begin (set n 0) (set last \"\") (while (< n 20) (begin (msg-send \"rt1.ping\" (str n)) "
      "(set last (get-attr (msg-recv \"rt1.pong\") \"payload\")) (set n (+ n 1)))) last)",
      rt_recv("rt1.bc"),
      rt_recv("rt1.bc"),
      "(begin (for i (range 30) nil) (msg-send \"rt1.bc\" \"all\") \"sent\")",
      "(begin (msg-send \"rt1.q\" \"a\") (msg-send \"rt1.q\" \"b\") (msg-pending \"rt1.q\"))",
      "(begin (set m (msg-recv \"rt1.host\")) m)",
      "(begin (set m (msg-recv \"rt1.host2\")) m)",
  };
}

// A scheduler's processes in pid order: rank, status, steps and outcome.
std::vector<std::string> rt01_record(async_scheduler &sched, const std::vector<int> &pids) {
  std::vector<std::string> out;
  for (std::size_t i = 0; i < pids.size(); ++i) {
    const auto info = sched.get_process_info(pids[i]);
    out.push_back("#" + std::to_string(i) + " " + (info ? to_string(info->status) : "gone") +
                  " steps " + std::to_string(info ? info->step_count : 0) + ": " +
                  rt_outcome(sched, pids[i]));
  }
  return out;
}

// Pump `pump` (one pump of every scheduler involved) until `done`, the pumps' budgets the same in
// every run.
void rt01_drive(const std::function<void()> &pump, const std::function<bool()> &done) {
  for (int round = 0; round < 10000 && !done(); ++round)
    pump();
}

} // namespace

// RT-1 (S2.md hard rule; td 5): with group 0 alone (no pool built, and with a pool), and with a
// group that registers no prefix beside it, the router is off and never consulted
// (exec_router::consulted() stays 0) across msg-send, msg-recv, msg-pending, a broadcast and host
// posts (async_scheduler::post_message, exec_pool::post_message); and the programs' per-process
// results and step counts equal a bare scheduler's (configured as app::exec_scheduler() is: the
// host parser and data defaults, the tree watched), pumped with the same budgets.
TEST(Router, RT01_OneGroupNeverRoutes) {
  constexpr std::uint64_t kBudget = 50;
  // The bare scheduler's record.
  std::vector<std::string> bare_record;
  {
    cvc::app tree_app;
    rt_env e;
    async_scheduler bare;
    bare.set_watch_root(&tree_app.root());
    bare.set_parser_defaults(async_scheduler::host_parser_defaults());
    bare.set_data_defaults(async_scheduler::host_data_defaults());
    e.ictx.sched = &bare;
    e.ictx.proc = make_process();
    e.ictx.proc->pid = 0;
    apply_chroot(e.ictx, tree_app.root(), "");
    register_intrinsics(e.env, &e.ictx);
    std::vector<int> pids;
    for (const std::string &p : rt01_programs()) {
      execute_options o;
      o.env = environment::extend(e.env);
      o.max_time = kRtProcessCeiling;
      pids.push_back(bare.execute(p, o));
    }
    bare.post_message("rt1.host", value_t(std::string("h1")));
    bare.post_message("rt1.host2", value_t(std::string("h2")));
    rt01_drive(
        [&] {
          bare.drain_ingress();
          bare.wake_awaiting();
          bare.sync_run_steps(kBudget, std::nullopt);
        },
        [&] {
          return std::all_of(pids.begin(), pids.end(),
                             [&](int pid) { return rt_finished(bare, pid); });
        });
    bare_record = rt01_record(bare, pids);
    for (const int pid : pids)
      bare.reap(pid);
    e.env.reset();
  }
  ASSERT_EQ(bare_record.size(), rt01_programs().size());

  enum class config { group0_alone, group0_with_pool, group0_and_a_plain_group };
  for (const config cf :
       {config::group0_alone, config::group0_with_pool, config::group0_and_a_plain_group}) {
    const char *name = cf == config::group0_alone       ? "group 0 alone, no pool"
                       : cf == config::group0_with_pool ? "group 0 alone, a pool"
                                                        : "group 0 and a group with no prefix";
    SCOPED_TRACE(name);
    rt_app a(cf == config::group0_alone ? std::nullopt
                                        : std::optional<cvc::pool_options>(rt_pool(0)));
    rt_node g0(a.app);
    std::unique_ptr<rt_node> plain;
    if (cf == config::group0_and_a_plain_group)
      plain = std::make_unique<rt_node>(a.app, "plain", std::vector<std::string>{}, "plain");
    EXPECT_FALSE(a.router().active());
    std::vector<rt_node *> nodes{&g0};
    if (plain)
      nodes.push_back(plain.get());
    std::vector<std::vector<int>> pids(nodes.size());
    for (std::size_t n = 0; n < nodes.size(); ++n) {
      for (const std::string &p : rt01_programs())
        pids[n].push_back(nodes[n]->submit(p));
      EXPECT_TRUE(nodes[n]->sched().post_message("rt1.host", value_t(std::string("h1"))));
      if (cf == config::group0_with_pool && n == 0)
        EXPECT_TRUE(a.pool().post_message("rt1.host2", value_t(std::string("h2"))));
      else
        EXPECT_TRUE(nodes[n]->sched().post_message("rt1.host2", value_t(std::string("h2"))));
    }
    for (std::size_t n = 0; n < nodes.size(); ++n) {
      rt01_drive([&] { nodes[n]->pump(kBudget); },
                 [&] {
                   return std::all_of(pids[n].begin(), pids[n].end(),
                                      [&](int pid) { return nodes[n]->finished(pid); });
                 });
      EXPECT_EQ(rt01_record(nodes[n]->sched(), pids[n]), bare_record) << nodes[n]->name();
    }
    EXPECT_EQ(a.router().consulted(), 0u) << "the router was consulted with one group";
    EXPECT_FALSE(a.router().active());
    if (cf == config::group0_alone)
      EXPECT_EQ(a.app.exec_pool_probe(), nullptr);
  }
}

// DT-3 (td 8.9; S2.md 4): MS-12's cells in deterministic mode against threaded mode (each step a
// pump_all of one group on any of the pool's threads, each pump moved to another thread than the
// last; settling pumps them all at once): the same outcomes, at threads 1, 2 and 4.
TEST(Router, DT03_DeterministicEqualsThreaded) {
  std::vector<std::string> misses;
  for (const ms12_column &column : kMs12Columns) {
    for (const rt_layout &layout : kRtLayouts) {
      const std::vector<std::string> det =
          ms12_run(column, layout, rt_driver::deterministic, 0, misses);
      for (const unsigned threads : rt_thread_counts()) {
        const std::vector<std::string> thr =
            ms12_run(column, layout, rt_driver::threaded, threads, misses);
        EXPECT_EQ(thr, det) << column.name << ", " << layout.name << ", threads " << threads << ":"
                            << rt_join(thr) << "\n against deterministic mode's:" << rt_join(det);
      }
    }
  }
  EXPECT_TRUE(misses.empty()) << rt_join(misses);
}

namespace {

// DT-4's run: 8 groups in deterministic continuous mode (attached; pump_host one pass a call, in
// creation order, 50 steps a pump), each sending `rounds` messages to each of the 7 others' keys
// and receiving its own: what each group received, in order.
std::vector<std::vector<std::string>> dt04_run(int rounds) {
  constexpr int kGroups = 8;
  cvc::pool_options po = rt_pool(0, true);
  po.pump_steps = 50;
  rt_app a(po);
  std::vector<std::vector<std::string>> got(kGroups);
  std::atomic<int> count{0};
  std::vector<std::unique_ptr<rt_node>> nodes;
  for (int i = 0; i < kGroups; ++i) {
    const std::string name = "dt4g" + std::to_string(i);
    nodes.push_back(
        std::make_unique<rt_node>(a.app, name, std::vector<std::string>{"docs." + name}, name));
    builtins::register_fn(nodes[i]->env(), "rec",
                          [&got, &count, i](std::span<const value_t> args) -> value_t {
                            got[i].push_back(std::get<std::string>(args[0].v));
                            count.fetch_add(1);
                            return nil_value();
                          });
  }
  const int each = (kGroups - 1) * rounds;
  for (int i = 0; i < kGroups; ++i) {
    nodes[i]->submit("(begin (set n 0) (while (< n " + std::to_string(each) + ") (begin (rec " +
                     rt_recv("docs.dt4g" + std::to_string(i) + ".in") + ") (set n (+ n 1)))) n)");
    std::string sends;
    for (int r = 0; r < rounds; ++r)
      for (int j = 0; j < kGroups; ++j)
        if (j != i)
          sends += " (msg-send \"docs.dt4g" + std::to_string(j) + ".in\" \"" + std::to_string(i) +
                   ":" + std::to_string(r) + "\") (for k (range " +
                   std::to_string((i + j + r) % 5) + ") nil)";
    nodes[i]->submit("(begin" + sends + " \"sent\")");
  }
  for (auto &n : nodes)
    a.pool().attach(*n->group());
  const int total = kGroups * each;
  int idle = 0;
  const auto t0 = rt_clock::now();
  while (count.load() < total && idle < 50 && rt_since(t0) < kRtCeiling) {
    if (a.pool().pump_host(kRtInf))
      idle = 0;
    else if (!a.pool().wait_for_work(0.01))
      ++idle;
  }
  for (auto &n : nodes)
    a.pool().detach(*n->group());
  return got;
}

} // namespace

// DT-4 (td 8.9's DT-1, its cross-group half): deterministic continuous mode, two runs of 8 groups
// exchanging messages: every group receives all 7 x rounds messages sent to it, in an order that is
// identical in both runs.
TEST(Router, DT04_CrossGroupOrderRepeats) {
  const int rounds = rt_scaled(20, 10, 5);
  const std::vector<std::vector<std::string>> first = dt04_run(rounds), second = dt04_run(rounds);
  for (std::size_t i = 0; i < first.size(); ++i) {
    EXPECT_EQ(first[i].size(), static_cast<std::size_t>(7 * rounds)) << "group dt4g" << i;
    EXPECT_EQ(first[i], second[i]) << "group dt4g" << i << "'s delivery order differs between runs";
  }
}

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
