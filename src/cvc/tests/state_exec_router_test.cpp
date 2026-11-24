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

// One cell's world: a fresh app and its groups, driven as `driver` says. pump(n) is one pump of
// n's group alone: by hand, or a pump_all of it (deterministic: on this thread; threaded: on any
// thread of the pool, each pump moved to another thread than the last). settle() pumps every group
// until none has work. Protocol expectations a cell checks on the way (a subscribe, send or deliver
// posted where the frozen protocol posts it, a counter) are collected in `misses`.
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
  }

  rt_node &H() { return *h_; }
  rt_node &S() { return *s_; }
  rt_node &R() { return *r_; }
  rt_node &Q() { return *q_; }
  bool remote(const rt_node &n) const { return &n != h_; }

  void pump(rt_node &n) {
    if (driver_ == rt_driver::hand) {
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
      if (std::none_of(nodes_.begin(), nodes_.end(), [](auto &n) { return n->busy(); }))
        return;
    }
    misses.push_back("the groups did not settle");
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
  // Where messages on `key` are queued: the home's FIFO, and every other group's (none, in every
  // layout: only a home queues).
  std::string queues(const std::string &key) {
    std::size_t elsewhere = 0;
    for (auto &n : nodes_)
      if (n.get() != h_)
        elsewhere += n->sched().total_pending_messages();
    return "queued at the home " + std::to_string(h_->sched().pending_message_count(key)) +
           ", elsewhere " + std::to_string(elsewhere);
  }
  std::uint64_t sum(std::uint64_t async_scheduler::ingress_stats::*field) {
    std::uint64_t n = 0;
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
  std::unique_ptr<rt_app> app_; // before nodes_: the groups go first
  std::vector<std::unique_ptr<rt_node>> nodes_;
  rt_node *h_ = nullptr, *s_ = nullptr, *r_ = nullptr, *q_ = nullptr;
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

// MS-8: an app-global key ("/ms8.global" from a document group's chroot) is homed at group 0: a
// receiver there gets the send. And (S2.md section 0, row 22; async_scheduler::has_pending_ingress)
// a send homed at group 0 when group 0 has no live process is not stranded: app::exec_pump_frame
// pumps group 0 for it, and it is queued there.
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
