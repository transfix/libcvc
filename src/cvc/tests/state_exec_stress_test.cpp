// Threaded execution, Stage 2b (docs/STATE_EXEC_THREADED.md 8.8; S2.md 4): ST-1 and ST-2, a random
// program generator over 16 groups of one app (messages across groups, broadcasts, timed receives,
// kills and pauses within and across groups, group churn), checked for no loss, no duplicate, no
// hang and no crash. The CTest form is short (seconds in Release; smaller in Debug and under a
// sanitizer). The nightly lane's long forms: CVC_EXEC_STRESS_MINUTES=<m> runs ST-1's seeds for m
// minutes; CVC_EXEC_STRESS_SEEDS=<n> runs ST-2 over n CVC_EXEC_FUZZ seeds. CVC_EXEC_STRESS_LOCAL=1
// checks the generator itself: every key's receivers in its home group and every sender sending
// only to its own group's keys, with no churn, so the run needs no router (it holds on S2b.0's
// stubs). No case name contains "Stress", so the PR lanes and the sanitizer legs (run.sh's
// -*Stress*) run the short form too.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/core/exec_pool.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_host.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/exec_group.h>
#include <cvc/state/state_exec/exec_router.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <functional>
#include <gtest/gtest.h>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <set>
#include <span>
#include <string>
#include <thread>
#include <vector>

using namespace cvc::state_exec;

namespace {

using st_clock = std::chrono::steady_clock;

constexpr double kStInf = std::numeric_limits<double>::infinity();
constexpr st_clock::time_point kStNoDeadline = st_clock::time_point::max();
constexpr std::uint64_t kStPump = 300;      // one pump's steps, both modes
constexpr std::uint64_t kStAlone = 20000;   // a churned group's own pumps
constexpr double kStProcessCeiling = 600.0; // a process's own backstop
// A run's ceiling (rule 7: a hang guard only, far above what a run takes).
constexpr double kStRunCeiling = 300.0;

#ifdef NDEBUG
constexpr bool kStDebug = false;
#else
constexpr bool kStDebug = true;
#endif

bool st_sanitizer() {
  const char *v = std::getenv("CVC_LOCKS_SANITIZER");
  return v && *v && std::string(v) != "0";
}

double st_since(st_clock::time_point t) {
  return std::chrono::duration<double>(st_clock::now() - t).count();
}

long st_env_number(const char *name, long fallback) {
  const char *v = std::getenv(name);
  if (!v || !*v)
    return fallback;
  char *end = nullptr;
  const long n = std::strtol(v, &end, 10);
  return end && *end == '\0' && n > 0 ? n : fallback;
}

// An environment variable set while it lives (CVC_EXEC_FUZZ, read when a pool or a scheduler is
// built), and restored after.
class st_env_var {
public:
  st_env_var(const char *name, const std::optional<std::string> &value) : name_(name) {
    if (const char *old = std::getenv(name))
      old_ = old;
    set(value ? value->c_str() : nullptr);
  }
  ~st_env_var() { set(old_ ? old_->c_str() : nullptr); }
  st_env_var(const st_env_var &) = delete;
  st_env_var &operator=(const st_env_var &) = delete;

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

// A run's size.
struct st_params {
  int groups;         // groups besides group 0
  int senders;        // sender processes per group at the start
  int messages;       // messages per sender
  int min_frames;     // host-loop passes at least, for the storms
  int churn_every;    // passes between two churns
  bool local = false; // CVC_EXEC_STRESS_LOCAL: every message in its home group, no churn
};

st_params st_default_params() {
  st_params p = st_sanitizer() ? st_params{6, 1, 8, 60, 25}
                : kStDebug     ? st_params{8, 1, 12, 120, 30}
                               : st_params{16, 2, 25, 600, 40};
  p.local = st_env_number("CVC_EXEC_STRESS_LOCAL", 0) == 1;
  return p;
}

// A scheduler of the run's app: group 0 or an exec_group (owner "stress", one prefix), with one
// environment (the intrinsics, unchrooted, plus the run's natives). Group 0's processes are killed
// and reaped when its node goes, before the environment; a group goes before its environment.
class st_node {
public:
  explicit st_node(cvc::app &app) : app_(app), sched_(&app.exec_scheduler()), name_("group 0") {
    init_env();
  }
  st_node(cvc::app &app, const std::string &name, const std::string &prefix)
      : app_(app), name_(name) {
    exec_group::options o;
    o.name = name;
    o.owner = "stress";
    o.app = &app;
    o.watch_root = &app.root();
    o.channel_prefixes = {prefix};
    group_ = std::make_unique<exec_group>(std::move(o));
    sched_ = &group_->sched();
    init_env();
  }
  st_node(const st_node &) = delete;
  st_node &operator=(const st_node &) = delete;
  ~st_node() {
    env_.reset();
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
  environment_ptr env() { return env_; }

  int submit(const std::string &script) {
    execute_options o;
    o.env = environment::extend(env_);
    o.max_time = kStProcessCeiling;
    o.owner = "stress";
    const int pid = group_ ? group_->execute(script, o) : sched_->execute(script, o);
    pids_.push_back(pid);
    return pid;
  }
  std::uint64_t pump(std::uint64_t steps) {
    if (group_)
      return group_->pump({steps, kStInf}).steps;
    async_scheduler::pump_owner_scope token(*sched_);
    sched_->drain_ingress();
    sched_->wake_awaiting();
    return sched_->sync_run_steps(steps, std::nullopt);
  }
  bool busy() {
    if (group_)
      return group_->wants_pump();
    const scheduler_stats s = sched_->get_stats();
    return sched_->has_pending_ingress() || s.ready + s.running > 0;
  }
  std::optional<process_status> status(int pid) {
    const auto i = sched_->get_process_info(pid);
    return i ? std::optional<process_status>(i->status) : std::nullopt;
  }

private:
  void init_env() {
    env_ = builtins::make_default_environment();
    ictx_.sched = sched_;
    ictx_.proc = make_process();
    ictx_.proc->pid = 0;
    apply_chroot(ictx_, app_.root(), "");
    ictx_.host = group_ ? &group_->host() : &app_.exec_async_host();
    register_intrinsics(env_, &ictx_);
  }

  cvc::app &app_;
  async_scheduler *sched_ = nullptr;
  std::string name_;
  intrinsics_context ictx_;
  environment_ptr env_;
  std::unique_ptr<exec_group> group_; // after ictx_ and env_: it goes first
  std::vector<int> pids_;
};

// One run of the generator: a seed, fork-join or continuous, the pool's threads.
class st_run {
public:
  st_run(std::uint64_t seed, bool continuous, unsigned threads, const st_params &p)
      : seed_(seed), continuous_(continuous), p_(p), rng_(seed) {
    app_.root()("state_exec.defaults.limits.async_steps_per_cpu_ms").value(std::string("0"));
    app_.root()("state_exec.locks.threading").value(std::string("shared"));
    cvc::pool_options po;
    po.threads = threads;
    po.pump_steps = kStPump;
    app_.exec_pool(po);
  }
  st_run(const st_run &) = delete;
  st_run &operator=(const st_run &) = delete;
  ~st_run() {
    // Groups before group 0, all before the app.
    detach_all();
    for (auto &sl : slots_)
      sl.node.reset();
    g0_.reset();
  }

  // The whole run: true when every check held (failures are reported with the seed).
  bool go();

private:
  enum class kind { sender, receiver, victim, killer, drainer };
  struct proc {
    st_node *node;
    int pid;
    kind k;
    int tag;           // a receiver's index in receivers_
    bool done = false; // a sender or killer seen finished
  };
  struct key {
    std::string name;
    bool broadcast;
    int slot;       // -1: homed at group 0 from the start ("global.*")
    int version;    // the slot's version that made it
    bool live_home; // its first home is alive (else rehomed to group 0)
  };
  struct receiver {
    int key;
    st_node *node;
    int pid;
    bool alive;
  };
  struct message {
    std::string id;
    int key;
  };
  struct slot {
    std::unique_ptr<st_node> node;
    int version = 0;
  };

  std::string label() const {
    return "seed " + std::to_string(seed_) + (continuous_ ? " (continuous" : " (fork-join") +
           ", threads " +
           std::to_string(app_.exec_pool_probe() ? app_.exec_pool_probe()->threads() : 0u) + ")";
  }
  void fail(const std::string &what) {
    ok_ = false;
    if (failures_++ < 20)
      ADD_FAILURE() << label() << ": " << what;
  }
  int pick(int n) { return n <= 0 ? 0 : static_cast<int>(rng_() % static_cast<unsigned>(n)); }
  bool chance(int percent) { return pick(100) < percent; }
  cvc::exec_pool &pool() { return app_.exec_pool(); }

  void install(st_node &n);
  st_node *home_node(int k);
  st_node *receiver_node(int k) { return p_.local ? home_node(k) : random_node(); }
  st_node *make_slot_node(int i);
  void make_keys(int i);
  st_node *random_node();
  int add_receiver(int k, st_node *n);
  void add_sender(st_node &n);
  void add_victim(st_node &n);
  void add_killer(st_node &n);
  std::vector<int> live_keys() const;

  // Run `fn` with n's scheduler held: under with_group for an attached group, else directly.
  void hold(st_node &n, const std::function<void()> &fn) {
    if (attached_ && n.group() && pool().attached(*n.group()))
      pool().with_group(*n.group(), fn);
    else
      fn();
  }
  void pass();
  void act();
  void detach_all();
  void churn(int i);
  bool quiet();
  bool senders_done();
  void settle(const char *what);
  void finish();
  void check();

  const std::uint64_t seed_;
  const bool continuous_;
  const st_params p_;
  std::mt19937_64 rng_;
  bool ok_ = true;
  int failures_ = 0;
  bool attached_ = false; // continuous mode, until the end detaches every group

  // The recorded outcome, written by the natives on whichever thread runs them.
  std::mutex mu_;
  std::vector<std::pair<int, std::string>> received_; // receiver tag, payload
  std::vector<std::string> drained_;

  std::vector<key> keys_;
  std::vector<receiver> receivers_;
  std::vector<message> sent_;
  std::vector<proc> procs_;
  std::set<std::pair<st_node *, int>> paused_;
  int next_sender_ = 0;
  int frames_ = 0;
  int churns_ = 0;
  int host_kills_ = 0, receiver_kills_ = 0, pauses_ = 0, killers_ = 0;

  cvc::app app_; // after the records: the nodes below go before it
  std::unique_ptr<st_node> g0_;
  std::vector<slot> slots_;
};

void st_run::install(st_node &n) {
  builtins::register_fn(n.env(), "rec", [this](std::span<const value_t> a) -> value_t {
    std::lock_guard<std::mutex> lock(mu_);
    received_.emplace_back(static_cast<int>(std::get<std::int64_t>(a[0].v)),
                           std::get<std::string>(a[1].v));
    return nil_value();
  });
  builtins::register_fn(n.env(), "drained", [this](std::span<const value_t> a) -> value_t {
    std::lock_guard<std::mutex> lock(mu_);
    drained_.push_back(std::get<std::string>(a[0].v));
    return nil_value();
  });
  // A home never drops a message (max_pending_messages 0), so every message is accounted for.
  n.sched().max_pending_messages = 0;
}

st_node *st_run::make_slot_node(int i) {
  slot &s = slots_[i];
  const std::string name = "st" + std::to_string(i) + "v" + std::to_string(s.version);
  s.node = std::make_unique<st_node>(app_, name, "docs." + name);
  install(*s.node);
  make_keys(i);
  return s.node.get();
}

void st_run::make_keys(int i) {
  const std::string prefix =
      "docs.st" + std::to_string(i) + "v" + std::to_string(slots_[i].version);
  keys_.push_back({prefix + ".u", false, i, slots_[i].version, true});
  keys_.push_back({prefix + ".b", true, i, slots_[i].version, true});
}

// The node a key is homed at now: its slot's group while that lives, else group 0.
st_node *st_run::home_node(int k) {
  const key &ky = keys_[k];
  return ky.slot >= 0 && ky.live_home ? slots_[ky.slot].node.get() : g0_.get();
}

st_node *st_run::random_node() {
  const int n = static_cast<int>(slots_.size());
  const int k = pick(n + 1);
  return k == n ? g0_.get() : slots_[k].node.get();
}

std::vector<int> st_run::live_keys() const {
  std::vector<int> out;
  for (int k = 0; k < static_cast<int>(keys_.size()); ++k)
    if (keys_[k].live_home)
      out.push_back(k);
  return out;
}

int st_run::add_receiver(int k, st_node *n) {
  const int tag = static_cast<int>(receivers_.size());
  const std::string name = keys_[k].name;
  std::string get;
  switch (pick(3)) {
  case 0: { // a timed receive, its timeout caught
    static constexpr const char *limits[] = {"0.001", "0.003", "0.01"};
    get = "(try (rec " + std::to_string(tag) + " (get-attr (await (dict \"__future__\" \"" + name +
          "\") " + limits[pick(3)] + ") \"payload\")) (catch (TimeoutError e) nil))";
    break;
  }
  default:
    get = "(rec " + std::to_string(tag) + " (get-attr (msg-recv \"" + name + "\") \"payload\"))";
  }
  int pid = -1;
  hold(*n, [&] { pid = n->submit("(while #t " + get + ")"); });
  receivers_.push_back({k, n, pid, true});
  procs_.push_back({n, pid, kind::receiver, tag});
  return tag;
}

void st_run::add_sender(st_node &n) {
  // Messages to random keys: mostly live ones, now and then one whose home was torn down (now homed
  // at group 0), with a little work or a short sleep between.
  std::string body;
  std::vector<int> live = live_keys();
  if (p_.local) { // only the keys homed here
    live.erase(std::remove_if(live.begin(), live.end(), [&](int k) { return home_node(k) != &n; }),
               live.end());
    if (live.empty())
      return;
  }
  for (int m = 0; m < p_.messages; ++m) {
    int k = live.empty() || (!p_.local && chance(10)) ? pick(static_cast<int>(keys_.size()))
                                                      : live[pick(static_cast<int>(live.size()))];
    const std::string id =
        "s" + std::to_string(next_sender_) + "." + std::to_string(m) + "@" + std::to_string(k);
    sent_.push_back({id, k});
    body += " (msg-send \"" + keys_[k].name + "\" \"" + id + "\")";
    switch (pick(4)) {
    case 0:
      body += " (sleep 0.0003)";
      break;
    case 1:
      body += " (for i (range " + std::to_string(pick(20)) + ") nil)";
      break;
    default:
      break;
    }
  }
  ++next_sender_;
  int pid = -1;
  hold(n, [&] { pid = n.submit("(begin" + body + " \"sent\")"); });
  procs_.push_back({&n, pid, kind::sender, -1});
}

void st_run::add_victim(st_node &n) {
  const std::string body =
      chance(50) ? "(while #t (for i (range 10) nil))" : "(while #t (sleep 0.0005))";
  int pid = -1;
  hold(n, [&] { pid = n.submit(body); });
  procs_.push_back({&n, pid, kind::victim, -1});
}

void st_run::add_killer(st_node &n) {
  std::vector<const proc *> victims;
  for (const proc &p : procs_)
    if (p.k == kind::victim)
      victims.push_back(&p);
  if (victims.empty())
    return;
  const int target = victims[pick(static_cast<int>(victims.size()))]->pid;
  int pid = -1;
  hold(n, [&] { pid = n.submit("(begin (kill " + std::to_string(target) + ") \"killed\")"); });
  procs_.push_back({&n, pid, kind::killer, -1});
  ++killers_;
}

// One pass of the groups: a pump_all frame with group 0 first (fork-join), or the attaching
// thread's pump_host or wait_for_work and a pump of group 0 (continuous).
void st_run::pass() {
  if (!attached_) {
    std::vector<exec_group *> gs;
    for (auto &s : slots_)
      gs.push_back(s.node->group());
    pool().pump_all(gs, {kStPump, kStInf}, kStNoDeadline, [this] { g0_->pump(kStPump); });
  } else {
    g0_->pump(kStPump);
    if (!pool().pump_host(0.002))
      pool().wait_for_work(0.001);
  }
  ++frames_;
}

// The host's random actions between passes.
void st_run::act() {
  // Kill a victim: directly, or by the pool's post_control.
  if (chance(20)) {
    std::vector<proc *> victims;
    for (proc &p : procs_)
      if (p.k == kind::victim)
        victims.push_back(&p);
    if (!victims.empty()) {
      proc *v = victims[pick(static_cast<int>(victims.size()))];
      if (chance(50)) {
        control_cmd cmd;
        cmd.what = control_cmd::kind::kill;
        pool().post_control(v->pid, std::move(cmd));
      } else {
        hold(*v->node, [&] { v->node->sched().kill(v->pid); });
      }
      ++host_kills_;
    }
  }
  // Kill a receiver that waits (it holds no message then), and give its key a new one.
  if (chance(10) && !receivers_.empty()) {
    receiver &r = receivers_[pick(static_cast<int>(receivers_.size()))];
    if (r.alive) {
      bool killed = false;
      hold(*r.node, [&] {
        if (r.node->status(r.pid) == process_status::waiting)
          killed = r.node->sched().kill(r.pid);
      });
      if (killed) {
        r.alive = false;
        ++receiver_kills_;
        const int k = r.key;
        add_receiver(k, receiver_node(k));
      }
    }
  }
  // Pause a ready process (a receiver may hold its message while paused): a few tries at random
  // ones, since most wait or have ended. Or resume one.
  if (chance(15)) {
    for (int tries = 0; tries < 8 && !procs_.empty(); ++tries) {
      proc &p = procs_[pick(static_cast<int>(procs_.size()))];
      bool paused = false;
      hold(*p.node, [&] {
        const auto s = p.node->status(p.pid);
        if (s && *s == process_status::ready)
          paused = p.node->sched().pause(p.pid);
      });
      if (paused) {
        paused_.insert({p.node, p.pid});
        ++pauses_;
        break;
      }
    }
  }
  if (chance(15) && !paused_.empty()) {
    auto it = paused_.begin();
    std::advance(it, pick(static_cast<int>(paused_.size())));
    st_node *n = it->first;
    const int pid = it->second;
    hold(*n, [&] { n->sched().resume(pid); });
    paused_.erase(it);
  }
  if (chance(5))
    add_killer(*random_node());
  if (chance(5))
    add_victim(*random_node());
  if (next_sender_ < p_.groups * p_.senders * 2 && chance(10))
    add_sender(*random_node());
  if (!p_.local && frames_ > 0 && frames_ % p_.churn_every == 0)
    churn(pick(static_cast<int>(slots_.size())));
}

// Replace slot i's group: let its senders and killers finish, resume what is paused in it, kill
// its victims, pump it alone (detached first in continuous mode) until its run loop ends with
// nothing ready, so no receiver holds an unrecorded message; then destroy it at once (its teardown
// rehomes what it holds; its waiting receivers die holding nothing) and make the slot's next
// version, with new keys and receivers for them.
void st_run::churn(int i) {
  st_node *n = slots_[i].node.get();
  if (attached_)
    pool().detach(*n->group());
  for (auto it = paused_.begin(); it != paused_.end();) {
    if (it->first == n) {
      n->sched().resume(it->second);
      it = paused_.erase(it);
    } else {
      ++it;
    }
  }
  for (proc &p : procs_)
    if (p.node == n && p.k == kind::victim)
      n->sched().kill(p.pid);
  const auto t0 = st_clock::now();
  for (;;) {
    const std::uint64_t steps = n->pump(kStAlone);
    bool pending_work = false;
    for (proc &p : procs_) {
      if (p.node != n)
        continue;
      const auto s = n->status(p.pid);
      if ((p.k == kind::sender || p.k == kind::killer) && s && !is_finished(*s))
        pending_work = true;
      if (p.k == kind::receiver && s && *s == process_status::ready)
        pending_work = true;
    }
    if (!pending_work && steps < kStAlone)
      break;
    if (st_since(t0) > 60.0) {
      fail("a churned group's senders never finished");
      break;
    }
    if (steps == 0)
      std::this_thread::sleep_for(std::chrono::microseconds(200)); // its senders' sleeps
  }
  for (receiver &r : receivers_)
    if (r.node == n)
      r.alive = false;
  procs_.erase(
      std::remove_if(procs_.begin(), procs_.end(), [n](const proc &p) { return p.node == n; }),
      procs_.end());
  for (key &k : keys_)
    if (k.slot == i && k.version == slots_[i].version)
      k.live_home = false;
  slots_[i].node.reset();
  ++slots_[i].version;
  st_node *fresh = make_slot_node(i);
  if (attached_)
    pool().attach(*fresh->group());
  const int k0 = static_cast<int>(keys_.size()) - 2;
  add_receiver(k0, receiver_node(k0));
  add_receiver(k0 + 1, receiver_node(k0 + 1));
  add_receiver(k0 + 1, receiver_node(k0 + 1));
  add_sender(*fresh);
  ++churns_;
}

bool st_run::senders_done() {
  for (proc &p : procs_) {
    if ((p.k != kind::sender && p.k != kind::killer) || p.done)
      continue;
    hold(*p.node, [&] {
      const auto s = p.node->status(p.pid);
      p.done = !s || is_finished(*s);
    });
    if (!p.done)
      return false;
  }
  return true;
}

// Nothing to pump anywhere: no group wants a pump, group 0 included.
bool st_run::quiet() {
  if (g0_->busy())
    return false;
  for (auto &s : slots_) {
    bool busy = false;
    hold(*s.node, [&] { busy = s.node->busy(); });
    if (busy)
      return false;
  }
  return true;
}

void st_run::settle(const char *what) {
  const auto t0 = st_clock::now();
  int calm = 0;
  while (calm < 3) {
    if (st_since(t0) > 60.0) {
      fail(std::string("never settled ") + what);
      return;
    }
    pass();
    calm = quiet() ? calm + 1 : 0;
  }
}

void st_run::detach_all() {
  if (!attached_)
    return;
  for (auto &sl : slots_)
    if (pool().attached(*sl.node->group()))
      pool().detach(*sl.node->group());
  attached_ = false;
}

// The end, every group detached (so the host reads them between passes, as in fork-join): every
// victim killed and everything resumed; the senders and killers left to finish; every receiver
// killed once it waits (it holds nothing then; a ready one is pumped on until it parks again);
// quiet; then a drainer at each key's home takes what is queued there. Nothing is left queued
// anywhere after that.
void st_run::finish() {
  detach_all();
  for (proc &p : procs_)
    if (p.k == kind::victim)
      p.node->sched().kill(p.pid);
  for (const auto &entry : paused_)
    entry.first->sched().resume(entry.second);
  paused_.clear();
  const auto t0 = st_clock::now();
  while (!senders_done()) {
    if (st_since(t0) > 120.0) {
      fail("the senders never finished");
      return;
    }
    pass();
  }
  const auto t1 = st_clock::now();
  for (;;) {
    bool all = true;
    for (receiver &r : receivers_) {
      if (!r.alive)
        continue;
      const auto s = r.node->status(r.pid);
      if (!s || is_finished(*s)) {
        if (s && *s == process_status::failed)
          fail("receiver " + std::to_string(r.pid) +
               " failed: " + r.node->sched().get_exit_error(r.pid).value_or(""));
        r.alive = false;
      } else if (*s == process_status::waiting) {
        r.node->sched().kill(r.pid);
        r.alive = false;
      } else {
        all = false;
      }
    }
    if (all)
      break;
    if (st_since(t1) > 60.0) {
      fail("a receiver never parked");
      return;
    }
    pass();
  }
  settle("after the receivers' kills");
  // A drainer per key at its home: msg-pending is the home's own count there.
  exec_router &router = app_.exec_router();
  for (const key &k : keys_) {
    const group_ref home = router.home_of(k.name);
    st_node *at = nullptr;
    if (home.is_group0())
      at = g0_.get();
    for (auto &sl : slots_)
      if (sl.node->group() && sl.node->group()->route() == home)
        at = sl.node.get();
    if (!at) {
      fail("no live home for " + k.name);
      continue;
    }
    const int pid = at->submit("(begin (while (> (msg-pending \"" + k.name +
                               "\") 0) (drained (get-attr (msg-recv \"" + k.name +
                               "\") \"payload\"))) \"drained\")");
    procs_.push_back({at, pid, kind::drainer, -1});
  }
  settle("after the drains");
  std::size_t queued = g0_->sched().total_pending_messages();
  for (auto &sl : slots_)
    queued += sl.node->sched().total_pending_messages();
  if (queued != 0)
    fail(std::to_string(queued) + " message(s) still queued after the drains (stranded)");
}

// Every message sent: received (by receivers of its key, each at most once), or drained once,
// never both, never neither. A unicast key's message reaches at most one receiver. A broadcast
// key whose home was torn down may see a copy requeued at its new home after another copy was
// taken (exec_router.h: an untracked requeue), so its messages are checked for loss only.
void st_run::check() {
  std::map<std::string, std::vector<int>> got;
  std::map<std::string, int> drained;
  {
    std::lock_guard<std::mutex> lock(mu_);
    for (const auto &[tag, id] : received_)
      got[id].push_back(tag);
    for (const std::string &id : drained_)
      ++drained[id];
  }
  std::set<std::string> ids;
  int lost = 0, both = 0, twice = 0, misrouted = 0, teardown_dups = 0, delivered = 0;
  for (const message &m : sent_) {
    ids.insert(m.id);
    const std::vector<int> &r = got[m.id];
    const int d = drained[m.id];
    const bool relaxed = keys_[m.key].broadcast && !keys_[m.key].live_home;
    for (const int tag : r)
      if (receivers_[tag].key != m.key && misrouted++ < 5)
        fail(m.id + " reached a receiver of " + keys_[receivers_[tag].key].name);
    if (r.empty() && d == 0) {
      if (lost++ < 5)
        fail(m.id + " (to " + keys_[m.key].name + ") was lost");
      continue;
    }
    delivered += r.empty() ? 0 : 1;
    std::set<int> distinct(r.begin(), r.end());
    const bool dup = (!r.empty() && d > 0) || d > 1 || distinct.size() != r.size() ||
                     (!keys_[m.key].broadcast && r.size() > 1);
    if (!dup)
      continue;
    if (relaxed) {
      ++teardown_dups;
      continue;
    }
    if (!r.empty() && d > 0) {
      if (both++ < 5)
        fail(m.id + " was received and also left queued");
    } else if (twice++ < 5) {
      fail(m.id + " was delivered twice (" + std::to_string(r.size()) + " receipts, " +
           std::to_string(d) + " drained)");
    }
  }
  int phantoms = 0;
  for (const auto &[id, tags] : got)
    if (!ids.count(id) && phantoms++ < 5)
      fail("a receiver got " + id + ", which nobody sent");
  for (const auto &[id, n] : drained)
    if (!ids.count(id) && phantoms++ < 5)
      fail("a drainer found " + id + ", which nobody sent");
  std::printf("%s: %zu messages (%d received, %zu drained), %d passes, %d churns, %d host kills, "
              "%d receiver kills, %d pauses, %d killers; lost %d, duplicated %d, misrouted %d, "
              "duplicates the teardown rule allows %d\n",
              label().c_str(), sent_.size(), delivered, drained_.size(), frames_, churns_,
              host_kills_, receiver_kills_, pauses_, killers_, lost, both + twice, misrouted,
              teardown_dups);
  std::fflush(stdout);
}

bool st_run::go() {
  const auto t0 = st_clock::now();
  g0_ = std::make_unique<st_node>(app_);
  install(*g0_);
  keys_.push_back({"global.u", false, -1, 0, true});
  keys_.push_back({"global.b", true, -1, 0, true});
  slots_.resize(p_.groups);
  for (int i = 0; i < p_.groups; ++i)
    make_slot_node(i);
  // Receivers: one per unicast key, two per broadcast key, anywhere; senders and victims.
  for (int k = 0; k < static_cast<int>(keys_.size()); ++k) {
    add_receiver(k, receiver_node(k));
    if (keys_[k].broadcast)
      add_receiver(k, receiver_node(k));
  }
  for (int i = 0; i < p_.groups; ++i) {
    for (int s = 0; s < p_.senders; ++s)
      add_sender(*slots_[i].node);
    if (chance(50))
      add_victim(*slots_[i].node);
  }
  if (continuous_) {
    for (auto &sl : slots_)
      pool().attach(*sl.node->group());
    attached_ = true;
  }
  while (frames_ < p_.min_frames || !senders_done()) {
    if (st_since(t0) > kStRunCeiling) {
      fail("the run's ceiling passed");
      break;
    }
    pass();
    act();
  }
  finish();
  detach_all();
  check();
  return ok_;
}

// Run seed `seed` (its mode and thread count from it), CVC_EXEC_FUZZ set to `fuzz` meanwhile.
bool st_one(std::uint64_t seed, const std::optional<std::string> &fuzz, const st_params &p) {
  static constexpr unsigned kThreads[] = {1, 2, 4};
  st_env_var env("CVC_EXEC_FUZZ", fuzz);
  st_run run(seed, seed % 2 == 1, kThreads[seed % 3], p);
  return run.go();
}

} // namespace

// ST-1 (td 8.8): the generator over 16 groups (fewer in Debug and under a sanitizer), seeds 1-6
// in CTest (fork-join and continuous, at threads 1, 2 and 4); for CVC_EXEC_STRESS_MINUTES
// minutes when it is set (the nightly lane), seed after seed.
TEST(RandomPrograms, ST01_SixteenGroupsLoseAndDuplicateNothing) {
  const st_params p = st_default_params();
  const long minutes = st_env_number("CVC_EXEC_STRESS_MINUTES", 0);
  const auto t0 = st_clock::now();
  std::uint64_t seed = 1;
  int runs = 0;
  for (;; ++seed) {
    if (minutes == 0 && seed > 6)
      break;
    if (minutes > 0 && st_since(t0) > 60.0 * static_cast<double>(minutes))
      break;
    ++runs;
    if (!st_one(seed, std::nullopt, p))
      break; // the first failing seed is reported; the rest would bury it
  }
  std::printf("ST-1: %d run(s)%s in %.1f s\n", runs, p.local ? " (local)" : "", st_since(t0));
}

// ST-2 (td 8.8): the same generator with CVC_EXEC_FUZZ set to the seed (the pool's, the ingress
// posts' and the router's fuzz points): 3 seeds in CTest, CVC_EXEC_STRESS_SEEDS when it is set
// (1000 in the nightly lane).
TEST(RandomPrograms, ST02_TheSameUnderFuzzSeeds) {
  const st_params p = st_default_params();
  const long seeds = st_env_number("CVC_EXEC_STRESS_SEEDS", 3);
  const auto t0 = st_clock::now();
  long runs = 0;
  for (long k = 0; k < seeds; ++k) {
    const std::uint64_t seed = 1000 + static_cast<std::uint64_t>(k);
    ++runs;
    if (!st_one(seed, std::to_string(seed), p))
      break;
  }
  std::printf("ST-2: %ld fuzz seed(s)%s in %.1f s\n", runs, p.local ? " (local)" : "",
              st_since(t0));
}
