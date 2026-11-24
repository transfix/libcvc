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

// A home's teardown (exec_router.h): its pending messages (in FIFO order), its remote-waiter
// records (subscribes with the same seq) and its queued router entries go to the keys' new home,
// here group 0, built for it.
TEST(RouterR, ATornDownHomeRehomesWhatItHolds) {
  cvc::app app;
  r_world r(app, "r");
  auto g = std::make_unique<r_world>(app, "g", std::vector<std::string>{"g"});
  ASSERT_TRUE(g->sched->post_message("g.k", envelope("a")));
  ASSERT_TRUE(g->sched->post_message("g.k", envelope("b")));
  const int p = r.submit(R"((get-attr (msg-recv "g.j") "payload"))");
  r.pump(); // subscribes at g
  g->pump();
  ASSERT_EQ(g->sched->pending_message_count("g.k"), 2u);
  ASSERT_TRUE(g->sched->post_route(entry(route_kind::send, "g.q", envelope("q")))); // queued
  g.reset();
  se::async_scheduler &g0 = app.exec_scheduler();
  g0.drain_ingress();
  EXPECT_EQ(drain_fifo(g0, "g.k"), (std::vector<std::string>{"a", "b"}));
  EXPECT_EQ(drain_fifo(g0, "g.q"), (std::vector<std::string>{"q"}));
  // r's wait moved with its record: a message there reaches it.
  ASSERT_TRUE(g0.post_message("g.j", envelope("moved")));
  g0.drain_ingress();
  r.pump();
  EXPECT_EQ(r.result(p), "moved");
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
