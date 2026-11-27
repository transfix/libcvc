// Stage 2's router (docs/STATE_EXEC_THREADED.md 4.6; S2.md S2b, S2-Q10): group handles, channel
// homes by registered prefix, the host's post, and async_scheduler's ends of the protocol (S2b.4:
// send, subscribe, unsubscribe, deliver, requeue, taken; broadcast tracking; rehoming).
// exec_router.h has the rules.
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cvc/core/app.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/exec_group.h>
#include <cvc/state/state_exec/exec_router.h>
#include <limits>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace cvc::state_exec {

namespace detail {
// What a group_ref points at. A post holds `mu` shared while it pushes into the scheduler's
// ingress, and close() takes it exclusively, so once close() returns no post touches the scheduler.
// `closed` is read without the lock (closed()), so a scheduler reached under one handle's lock
// never takes it again (S2b.R2, C-M3). Once closed (S2b.R3), a post takes only `late_mu`, never
// holding `mu` then: `late`, `sealed` and `done` are written under `late_mu`; `addresses` before
// `done` is stored (release), and never after, so a poster that loads `done` true (acquire) reads
// them without a lock, and a sealed poster waits on `done` without taking any.
struct route_target {
  mutable std::shared_mutex mu;
  std::atomic<bool> closed{false};
  async_scheduler *sched = nullptr; // null once closed
  exec_group *group = nullptr;      // null for group 0
  bool group0 = false;
  std::uint64_t id = 0;
  std::mutex late_mu;
  std::atomic<bool> done{false};
  bool sealed = false; // only a teardown's forward goes into `late` (the teardown's backstop)
  std::vector<route_entry> late;
  std::unordered_map<std::string, group_ref> addresses; // once done: key -> where its entries went
};
} // namespace detail

// --- group_ref ---

group_ref group_ref::make(async_scheduler &sched, exec_group *group) {
  group_ref r;
  r.t_ = std::make_shared<detail::route_target>();
  r.t_->sched = &sched;
  r.t_->group = group;
  r.t_->group0 = group == nullptr;
  r.t_->id = group ? group->pool_link().seq : 0;
  return r;
}

void group_ref::close() const noexcept {
  if (!t_)
    return;
  std::unique_lock<std::shared_mutex> lock(t_->mu);
  t_->sched = nullptr;
  t_->closed.store(true, std::memory_order_release);
}

bool group_ref::closed() const noexcept {
  return !t_ || t_->closed.load(std::memory_order_acquire);
}

bool group_ref::done() const noexcept { return !t_ || t_->done.load(std::memory_order_acquire); }

group_ref group_ref::forwarded(std::string_view key) const {
  if (!t_ || !t_->done.load(std::memory_order_acquire) || t_->addresses.empty())
    return group_ref();
  const auto it = t_->addresses.find(std::string(key)); // never written once done
  return it == t_->addresses.end() ? group_ref() : it->second;
}

bool group_ref::finish(std::vector<detail::route_entry> &late,
                       std::unordered_map<std::string, group_ref> &addresses,
                       const std::function<void()> &at_done) const {
  if (!t_)
    return true;
  std::lock_guard<std::mutex> lock(t_->late_mu); // every poster of a closed handle takes it
  if (t_->done.load(std::memory_order_relaxed))
    return true;
  if (!t_->late.empty()) {
    late.clear();
    late.swap(t_->late);
    return false;
  }
  // Under the lock, so no poster sees the handle done while its prefixes are still registered (a
  // refused poster's home_of then gives the new home). The router's lock is a leaf.
  if (at_done)
    at_done();
  t_->addresses = std::move(addresses);
  t_->done.store(true, std::memory_order_release);
  return true;
}

void group_ref::seal() const {
  if (!t_)
    return;
  std::lock_guard<std::mutex> lock(t_->late_mu);
  t_->sealed = true;
}

std::vector<detail::route_entry> group_ref::close_for_good() const noexcept {
  std::vector<detail::route_entry> late;
  if (!t_)
    return late;
  {
    std::unique_lock<std::shared_mutex> lock(t_->mu); // waits for a post in progress
    t_->sched = nullptr;
    t_->closed.store(true, std::memory_order_release);
  }
  std::lock_guard<std::mutex> lock(t_->late_mu);
  t_->done.store(true, std::memory_order_release);
  late.swap(t_->late);
  return late;
}

bool group_ref::is_group0() const noexcept { return t_ && t_->group0; }

exec_group *group_ref::group() const noexcept { return t_ ? t_->group : nullptr; }

std::uint64_t group_ref::id() const noexcept { return t_ ? t_->id : 0; }

group_ref::post_result group_ref::try_post(detail::route_entry e, bool teardown) const {
  if (!t_)
    return post_result::done;
  {
    std::shared_lock<std::shared_mutex> lock(t_->mu);
    if (t_->sched)
      return t_->sched->post_route(std::move(e)) ? post_result::taken : post_result::refused;
  }
  // Closed (for good: close() is never undone), so `mu` is no longer needed.
  std::lock_guard<std::mutex> late(t_->late_mu);
  if (t_->done.load(std::memory_order_relaxed))
    return post_result::done; // its poster goes on by the forwarding address
  if (t_->sealed && !teardown)
    return post_result::sealed; // its poster waits for done (post_with_rehome)
  // Its teardown still forwarding: the late queue, which it forwards before it is done (S2b.R3),
  // so this entry arrives after the state it refers to. No cap, no wait.
  t_->late.push_back(std::move(e));
  return post_result::taken;
}

bool group_ref::post(detail::route_entry e, bool teardown) const {
  return try_post(std::move(e), teardown) == post_result::taken;
}

bool group_ref::post_message(const std::string &key, const value_t &value) const {
  if (!t_)
    return false;
  // Into this group, unrouted (exec_router.h, "Host posts"): a send entry, applied there as a
  // host's message is, counted against its cap unless the key has a '#' (a runtime reply). Nothing
  // is routed, and no other handle's lock taken, under this one's (S2b.R2, C-M3).
  detail::route_entry e;
  e.what = detail::route_entry::kind::send;
  e.key = key;
  e.value = value;
  e.uncapped = key.find('#') != std::string::npos;
  std::shared_lock<std::shared_mutex> lock(t_->mu);
  return t_->sched && t_->sched->post_route(std::move(e));
}

bool group_ref::post_control(int pid, control_cmd cmd) const {
  if (!t_)
    return false;
  std::shared_lock<std::shared_mutex> lock(t_->mu);
  return t_->sched && t_->sched->post_control(pid, std::move(cmd));
}

bool group_ref::visit(const std::function<void(async_scheduler &)> &fn) const {
  if (!t_)
    return false;
  std::shared_lock<std::shared_mutex> lock(t_->mu);
  if (!t_->sched)
    return false;
  fn(*t_->sched);
  return true;
}

// --- exec_router ---

struct exec_router::impl {
  mutable std::shared_mutex mu;
  struct registration {
    std::string prefix;
    group_ref group;
  };
  std::vector<registration> regs; // every prefix but group 0's "", in registration order
  group_ref group0;               // the default home, once group 0 registered ""
  std::atomic<std::uint64_t> epoch{0};
  mutable std::atomic<std::uint64_t> consulted{0};
  std::atomic<std::uint64_t> broadcasts{0}; // new_broadcast_id
  // CVC_EXEC_FUZZ's yields at the host's posts (S2-Q15), read as the router is made; null: off.
  std::unique_ptr<detail::lock_fuzz> fuzz;

  static bool covers(std::string_view key, const std::string &p) noexcept {
    return key.size() >= p.size() && key.compare(0, p.size(), p) == 0 &&
           (key.size() == p.size() || p.empty() || key[p.size()] == '.');
  }
};

exec_router::exec_router(cvc::app *app) : app_(app), impl_(std::make_unique<impl>()) {
  if (const std::uint64_t seed = detail::lock_fuzz_seed_from_environment()) {
    impl_->fuzz = std::make_unique<detail::lock_fuzz>();
    impl_->fuzz->seed.store(seed, std::memory_order_relaxed);
  }
}

exec_router::~exec_router() = default;

void exec_router::register_prefix(const group_ref &group, std::string prefix) {
  if (!group)
    return;
  // "" is group 0's alone, the default home (S2b.R2, S-m8): another group's is ignored.
  if (prefix.empty() && !group.is_group0()) {
    assert(false && "exec_router::register_prefix: \"\" from a group other than group 0");
    return;
  }
  impl &m = *impl_;
  {
    std::unique_lock<std::shared_mutex> lock(m.mu);
    if (prefix.empty() && group.is_group0()) {
      m.group0 = group;
    } else {
      m.regs.push_back({std::move(prefix), group});
      if (!group.is_group0())
        others_.fetch_add(1, std::memory_order_relaxed);
    }
    m.epoch.fetch_add(1, std::memory_order_release);
  }
}

void exec_router::unregister(const group_ref &group) {
  if (!group)
    return;
  impl &m = *impl_;
  std::unique_lock<std::shared_mutex> lock(m.mu);
  const auto end = std::remove_if(m.regs.begin(), m.regs.end(),
                                  [&](const impl::registration &r) { return r.group == group; });
  const auto removed = static_cast<int>(m.regs.end() - end);
  m.regs.erase(end, m.regs.end());
  if (!group.is_group0())
    others_.fetch_sub(removed, std::memory_order_relaxed);
  if (m.group0 == group)
    m.group0 = group_ref();
  m.epoch.fetch_add(1, std::memory_order_release);
}

group_ref exec_router::home_of(std::string_view key) const {
  impl &m = *impl_;
  m.consulted.fetch_add(1, std::memory_order_relaxed);
  for (int attempt = 0; attempt < 2; ++attempt) {
    {
      std::shared_lock<std::shared_mutex> lock(m.mu);
      const impl::registration *best = nullptr;
      for (const impl::registration &r : m.regs)
        if (impl::covers(key, r.prefix) && (!best || r.prefix.size() > best->prefix.size()))
          best = &r;
      if (best)
        return best->group;
      if (m.group0 || !app_)
        return m.group0;
    }
    // Group 0 is not built yet: build it (it registers "" here), outside the router's lock.
    if (attempt == 0)
      app_->exec_scheduler();
  }
  std::shared_lock<std::shared_mutex> lock(m.mu);
  return m.group0;
}

group_ref exec_router::new_home_of(std::string_view key) const {
  impl &m = *impl_;
  // The covering registrations, longest first, taken under the router's lock and checked for a
  // closed handle after it (the router's lock stays a leaf: no group_ref lock under it).
  std::vector<std::pair<std::size_t, group_ref>> candidates;
  group_ref zero;
  {
    std::shared_lock<std::shared_mutex> lock(m.mu);
    for (const impl::registration &r : m.regs)
      if (impl::covers(key, r.prefix))
        candidates.emplace_back(r.prefix.size(), r.group);
    zero = m.group0;
  }
  std::stable_sort(candidates.begin(), candidates.end(),
                   [](const auto &a, const auto &b) { return a.first > b.first; });
  for (const auto &[length, group] : candidates)
    if (!group.closed())
      return group;
  if (zero && !zero.closed())
    return zero;
  if (!zero && app_) {
    app_->exec_scheduler(); // group 0 not built yet: built now, it registers ""
    std::shared_lock<std::shared_mutex> lock(m.mu);
    return m.group0;
  }
  return group_ref(); // group 0 is going too (~app): nothing to forward to
}

group_ref exec_router::prefix_home_of(std::string_view key) const {
  impl &m = *impl_;
  m.consulted.fetch_add(1, std::memory_order_relaxed);
  std::shared_lock<std::shared_mutex> lock(m.mu);
  const impl::registration *best = nullptr;
  for (const impl::registration &r : m.regs)
    if (!r.prefix.empty() && impl::covers(key, r.prefix) &&
        (!best || r.prefix.size() > best->prefix.size()))
      best = &r;
  return best ? best->group : group_ref();
}

bool exec_router::has_prefix(const group_ref &group) const {
  impl &m = *impl_;
  std::shared_lock<std::shared_mutex> lock(m.mu);
  if (group && m.group0 == group)
    return true;
  for (const impl::registration &r : m.regs)
    if (r.group == group)
      return true;
  return false;
}

bool exec_router::prefix_registered(std::string_view prefix) const {
  impl &m = *impl_;
  std::shared_lock<std::shared_mutex> lock(m.mu);
  for (const impl::registration &r : m.regs)
    if (r.prefix == prefix && !r.group.is_group0())
      return true;
  return false;
}

std::uint64_t exec_router::new_broadcast_id() noexcept {
  return impl_->broadcasts.fetch_add(1, std::memory_order_relaxed) + 1;
}

std::uint64_t exec_router::epoch() const noexcept {
  return impl_->epoch.load(std::memory_order_acquire);
}

std::uint64_t exec_router::consulted() const noexcept {
  return impl_->consulted.load(std::memory_order_relaxed);
}

void exec_router::note_remote_waiters(int delta) noexcept {
  remote_waiters_.fetch_add(delta, std::memory_order_acq_rel);
}

namespace {

// A poster's wait for a sealed teardown to be done (exec_router.h): yields first, then short
// sleeps. A sealed teardown waits on nothing and has finitely many entries left, so this ends.
void sealed_backoff(unsigned n) {
  if (n < 64)
    std::this_thread::yield();
  else
    std::this_thread::sleep_for(std::chrono::microseconds(100));
}

// Post `e` to `home` (exec_router.h, "Teardown and rehoming", as amended in S2b.R3). A closed
// handle still forwarding takes it into its late queue (once sealed, only a teardown's forward:
// another poster waits until it is done); a done one refuses it, and the poster goes on at once:
// to the done handle's forwarding address for the key (where its
// teardown sent that key's entries, so `e` arrives after them), else the key's home (home_of: the
// done handle is unregistered), else, when that is the same handle (group 0 during ~app, never
// unregistered), its home without closed groups (new_home_of). A teardown's forward (`teardown`)
// skips the home_of step: its targets are open groups only, so no forward comes back to a teardown
// still forwarding (its own late queue among them), but a deliver's requeue to its sender.
// Broadcast ids are unique in the app, so an answer keeps its id (the new home adopted the
// tracker). An empty `home` is resolved first. 1: taken (`posted_to` gets the group that took it);
// 0: refused by the home (its cap); -1: no home took it.
int post_with_rehome(const exec_router *router, group_ref home, detail::route_entry e,
                     detail::lock_fuzz *fuzz, group_ref *posted_to, bool teardown) {
  // A forwarding address is followed whatever the chain's length (its targets close in turn, so
  // it ends; the cap on hops is a guard only); the home is resolved again at most 16 times.
  int resolved = 0;
  for (int hops = 0; hops < 4096; ++hops) {
    if (home) {
      detail::lock_fuzz_point(fuzz, detail::lock_fuzz_at::post_enter); // S2-Q15's router posts
      // A copy: a refused post consumes what it was given.
      const group_ref::post_result r = home.try_post(e, teardown);
      if (r == group_ref::post_result::taken) {
        if (posted_to)
          *posted_to = std::move(home);
        return 1;
      }
      if (r == group_ref::post_result::refused)
        return 0; // a capped send at the open home's cap, whatever it does since
      for (unsigned n = 0; r == group_ref::post_result::sealed && !home.done(); ++n)
        sealed_backoff(n); // sealed: its teardown has only a finite forward left
      if (group_ref next = home.forwarded(e.key)) {
        home = std::move(next);
        continue;
      }
    }
    if (!router || ++resolved > 16)
      return -1;
    group_ref next;
    if (!teardown) {
      next = router->home_of(e.key);
      if (home && next == home)
        next = router->new_home_of(e.key);
    } else {
      next = router->new_home_of(e.key);
    }
    if (!next || next == home)
      return -1;
    home = std::move(next);
  }
  return -1;
}

// A copy of a message's lists and dicts, keeping their sharing and cycles; everything else shared
// (td B9, B12): each group a message reaches gets its own envelope. No native recursion.
value_t copy_envelope(const value_t &v) {
  if (!std::holds_alternative<list_ptr>(v.v) && !std::holds_alternative<dict_ptr>(v.v))
    return v;
  std::unordered_map<const void *, value_t> made;   // source container -> its copy
  std::vector<std::pair<value_t, value_t>> pending; // (source, copy) containers left to fill
  const auto copy_of = [&](const value_t &src) -> value_t {
    const void *key = nullptr;
    value_t out;
    if (const list_ptr *l = std::get_if<list_ptr>(&src.v); l && *l) {
      key = l->get();
      if (auto it = made.find(key); it != made.end())
        return it->second;
      out = make_list(std::vector<value_t>((*l)->size()));
    } else if (const dict_ptr *d = std::get_if<dict_ptr>(&src.v); d && *d) {
      key = d->get();
      if (auto it = made.find(key); it != made.end())
        return it->second;
      out = make_dict(std::vector<std::pair<std::string, value_t>>((*d)->size()));
    } else {
      return src;
    }
    made.emplace(key, out);
    pending.emplace_back(src, out);
    return out;
  };
  const value_t root = copy_of(v);
  while (!pending.empty()) {
    const auto [src, dst] = std::move(pending.back());
    pending.pop_back();
    if (const list_ptr *l = std::get_if<list_ptr>(&src.v)) {
      std::vector<value_t> &out = *std::get<list_ptr>(dst.v);
      for (std::size_t i = 0; i < (*l)->size(); ++i)
        out[i] = copy_of((**l)[i]);
    } else {
      const dict_ptr &in = std::get<dict_ptr>(src.v);
      auto &out = *std::get<dict_ptr>(dst.v);
      for (std::size_t i = 0; i < in->size(); ++i) {
        out[i].first = (*in)[i].first;
        out[i].second = copy_of((*in)[i].second);
      }
    }
  }
  return root;
}

bool never_routed(const std::string &key) { return key.find('#') != std::string::npos; }

detail::route_entry make_entry(detail::route_entry::kind what, const std::string &key) {
  detail::route_entry e;
  e.what = what;
  e.key = key;
  return e;
}

} // namespace

// The host's post (exec_pool::post_message): to the key's home as a send, which counts against
// the home's cap; group 0's post_message while the router is off (every key is homed there).
bool exec_router::post_message(const std::string &key, const value_t &value) {
  if (!app_)
    return false;
  if (!active() || never_routed(key))
    return app_->exec_scheduler().post_message(key, value);
  detail::route_entry e = make_entry(detail::route_entry::kind::send, key);
  e.value = value;
  return post_with_rehome(this, home_of(key), std::move(e), impl_->fuzz.get(), nullptr, false) > 0;
}

// --- async_scheduler's ends of the protocol (S2b.4; exec_router.h, as amended in S2b.0a) ---

namespace {
// The cache of homes is capped (computed keys are unbounded): full, it starts over.
constexpr std::size_t kHomeCacheCap = 4096;
} // namespace

group_ref async_scheduler::cached_home(const std::string &key) const {
  const std::uint64_t epoch = router_->epoch(); // before the lookup: a move meanwhile clears it
  if (epoch != home_cache_epoch_) {
    home_cache_.clear();
    home_cache_epoch_ = epoch;
    home_cache_prefixed_ = router_->has_prefix(route_self_); // registrations move the epoch
    if (home_cache_prefixed_)
      prefixed_once_.store(true, std::memory_order_relaxed);
  }
  if (const auto it = home_cache_.find(key); it != home_cache_.end())
    return it->second;
  // An island asks only whether another group's prefix covers the key (never building group 0);
  // empty means its own.
  group_ref home = home_cache_prefixed_ ? router_->home_of(key) : router_->prefix_home_of(key);
  if (home_cache_.size() >= kHomeCacheCap)
    home_cache_.clear();
  home_cache_.emplace(key, home);
  return home;
}

bool async_scheduler::homed_elsewhere_slow(const std::string &key, group_ref *home) const {
  // The router is on (homed_elsewhere's inline test, RT-1); a '#' key is never routed.
  if (never_routed(key))
    return false;
  group_ref h = cached_home(key);
  if (!h || h == route_self_)
    return false;
  // An island (S2b.0a): a group that registered no prefix keeps the keys no other group's prefix
  // covers (home_of gives group 0 for those), router on or off, as today.
  if (h.is_group0() && !home_cache_prefixed_)
    return false;
  if (home)
    *home = std::move(h);
  return true;
}

int async_scheduler::post_routed(group_ref home, detail::route_entry e, group_ref *posted_to,
                                 bool teardown) const {
  return post_with_rehome(router_, std::move(home), std::move(e), ingress_fuzz_.get(), posted_to,
                          teardown);
}

std::uint64_t async_scheduler::new_broadcast_id() {
  return router_ ? router_->new_broadcast_id() : ++broadcast_seq_;
}

void async_scheduler::subscribe_if_remote(process &proc, const std::string &path) {
  // msg-recv on a key homed elsewhere (td 4.6): the process is parked here with its new recv_seq,
  // and subscribes at the home, which answers from its pending FIFO or records a remote waiter.
  group_ref home;
  if (!homed_elsewhere(path, &home))
    return;
  detail::route_entry e = make_entry(detail::route_entry::kind::subscribe, path);
  e.from = route_self_;
  e.pid = proc.pid;
  e.seq = proc.recv_seq;
  e.deadline = proc.recv_deadline;
  group_ref at;
  if (post_routed(std::move(home), std::move(e), &at) > 0) {
    const bool timed = proc.recv_deadline.has_value();
    remote_waits_[proc.pid] = remote_wait{path, proc.recv_seq, std::move(at), timed};
    remote_timed_waits_ += timed ? 1 : 0;
  }
}

void async_scheduler::forget_remote_wait(std::unordered_map<int, remote_wait>::iterator it) {
  if (it->second.timed)
    --remote_timed_waits_;
  remote_waits_.erase(it);
}

void async_scheduler::end_remote_wait(int pid) noexcept {
  const auto it = remote_waits_.find(pid);
  if (it == remote_waits_.end())
    return;
  try {
    remote_wait w = std::move(it->second);
    forget_remote_wait(it);
    // A record already used means a deliver is in flight: it comes back as a requeue if stale.
    detail::route_entry e = make_entry(detail::route_entry::kind::unsubscribe, w.key);
    e.from = route_self_;
    e.pid = pid;
    e.seq = w.seq;
    post_routed(std::move(w.home), std::move(e));
  } catch (...) {
    // Not posted: the home keeps a stale record, whose deliver comes back as a requeue.
  }
}

void async_scheduler::sweep_remote_waits() noexcept {
  try {
    std::vector<int> ended;
    for (const auto &[pid, w] : remote_waits_) {
      const process *p = procs_.find(pid);
      if (!p || p->status != process_status::waiting || !p->recv_path || p->recv_seq != w.seq ||
          *p->recv_path != w.key)
        ended.push_back(pid);
    }
    std::sort(ended.begin(), ended.end()); // no hash order reaches a post order (DT-4)
    for (const int pid : ended)
      end_remote_wait(pid);
  } catch (...) {
  }
}

void async_scheduler::apply_route(detail::route_entry &e) {
  using kind = detail::route_entry::kind;
  switch (e.what) {
  case kind::send: // a msg-send (or a host's post) for a key homed here; uncapped: rehomed
    deliver_home(e.key, e.value, e.uncapped);
    break;
  case kind::subscribe:
    home_subscribe(e);
    break;
  case kind::unsubscribe:
    home_unsubscribe(e);
    break;
  case kind::deliver:
    receiver_deliver(e);
    break;
  case kind::requeue:
    apply_requeue(e.key, std::move(e.value), e.broadcast);
    break;
  case kind::taken:
    home_taken(e);
    break;
  case kind::adopt:
    home_adopt(e);
    break;
  }
}

std::size_t async_scheduler::deliver_remote(const std::string &key, const value_t &msg,
                                            bool local_took) {
  const auto it = remote_waiters_.find(key);
  if (it == remote_waiters_.end())
    return 0;
  const std::deque<remote_waiter> records = std::move(it->second);
  remote_waiters_.erase(it);
  if (router_)
    router_->note_remote_waiters(-static_cast<int>(records.size())); // used, or skipped
  // A record whose deadline passed is skipped: its process times out, or did, and unsubscribes.
  const bool timed = std::any_of(records.begin(), records.end(),
                                 [](const remote_waiter &r) { return r.deadline.has_value(); });
  const clock::time_point now = timed ? clock::now() : clock::time_point{};
  std::vector<const remote_waiter *> live;
  live.reserve(records.size());
  for (const remote_waiter &r : records)
    if (!r.deadline || *r.deadline > now)
      live.push_back(&r);
  if (live.empty())
    return 0;
  // Broadcast tracking (exec_router.h): a local waiter took it, so a stale copy is dropped; one
  // remote waiter alone, untracked; k >= 2, tracked here by an id unique in the app until every
  // copy answers.
  const bool requeue_if_stale = !local_took;
  const std::uint64_t id = !local_took && live.size() >= 2 ? new_broadcast_id() : 0;
  // One envelope copy per receiving group, made at its first record, in subscription order.
  std::vector<std::pair<group_ref, value_t>> copies;
  std::size_t posted = 0, refused = 0;
  for (const remote_waiter *r : live) {
    std::size_t c = 0;
    while (c < copies.size() && copies[c].first != r->from)
      ++c;
    if (c == copies.size())
      copies.emplace_back(r->from, copy_envelope(msg));
    detail::route_entry d = make_entry(detail::route_entry::kind::deliver, key);
    d.value = copies[c].second;
    d.from = route_self_;
    d.pid = r->pid;
    d.seq = r->seq;
    d.broadcast = id;
    d.requeue_if_stale = requeue_if_stale;
    detail::lock_fuzz_point(ingress_fuzz_.get(), detail::lock_fuzz_at::post_enter);
    if (r->from.post(std::move(d)))
      ++posted;
    else if (local_took) // its group's closed handle refused it: a stale copy at once, dropped
      ingress_counts_.stale_dropped.fetch_add(1, std::memory_order_relaxed);
    else
      ++refused;
  }
  // A copy a closed receiver refused is stale at once (S2b.0a): a tracked one counts as answered,
  // so the tracker waits for the copies posted; with none posted, the message is applied as the
  // requeue its last stale copy would make.
  if (id != 0 && posted > 0)
    tracked_.emplace(id, tracked_broadcast{key, msg, posted, false});
  if (posted == 0 && refused > 0) {
    ingress_counts_.requeued.fetch_add(1, std::memory_order_relaxed);
    give_next(key, msg);
    return refused;
  }
  return posted;
}

void async_scheduler::give_next(const std::string &key, value_t msg) {
  // The earliest local waiter: the lowest recv_seq (one count per scheduler, in park order). The
  // others go back into the receiver index.
  std::vector<process *> list;
  list.swap(woken_);
  procs_.take_receivers(key, list);
  process *first = nullptr;
  for (process *p : list)
    if (!first || p->recv_seq < first->recv_seq)
      first = p;
  for (process *p : list)
    if (p != first)
      procs_.update(*p);
  list.clear();
  woken_.swap(list);
  if (first) {
    if (!remote_waits_.empty())
      end_remote_wait(first->pid); // subscribed elsewhere before this key's home changed
    deliver_to(first->pid, first->recv_seq, key, msg);
    return;
  }
  // Else the earliest record whose deadline has not passed, by an untracked deliver; one its
  // closed receiver refuses is stale at once, and the next is tried.
  const auto it = remote_waiters_.find(key);
  if (it != remote_waiters_.end()) {
    std::deque<remote_waiter> &records = it->second;
    std::optional<clock::time_point> now;
    bool given = false;
    while (!given && !records.empty()) {
      const remote_waiter r = std::move(records.front());
      records.pop_front();
      if (router_)
        router_->note_remote_waiters(-1);
      if (r.deadline) {
        if (!now)
          now = clock::now();
        if (*r.deadline <= *now)
          continue; // skipped: its process times out, or did
      }
      detail::route_entry d = make_entry(detail::route_entry::kind::deliver, key);
      d.value = copy_envelope(msg);
      d.from = route_self_;
      d.pid = r.pid;
      d.seq = r.seq;
      detail::lock_fuzz_point(ingress_fuzz_.get(), detail::lock_fuzz_at::post_enter);
      given = r.from.post(std::move(d));
    }
    if (records.empty())
      remote_waiters_.erase(it);
    if (given)
      return;
  }
  pending_messages_[key].push_front(std::move(msg)); // past the cap
}

void async_scheduler::apply_requeue(const std::string &key, value_t msg, std::uint64_t broadcast) {
  // A tracked broadcast's stale copy: dropped when another copy was taken; applied once as a
  // requeue when every copy came back stale (the home's own copy of the message); else it waits for
  // the others. An id this home does not track is applied as untracked.
  if (broadcast != 0) {
    const auto it = tracked_.find(broadcast);
    if (it != tracked_.end()) {
      tracked_broadcast &t = it->second;
      const bool last = --t.outstanding == 0;
      if (t.taken) {
        ingress_counts_.stale_dropped.fetch_add(1, std::memory_order_relaxed);
        if (last)
          tracked_.erase(it);
        return;
      }
      if (!last)
        return;
      msg = std::move(t.value);
      tracked_.erase(it);
    }
  }
  ingress_counts_.requeued.fetch_add(1, std::memory_order_relaxed);
  give_next(key, std::move(msg));
}

void async_scheduler::home_subscribe(detail::route_entry &e) {
  // A record forwarded here from a torn-down home for a process of this group: its wait is local
  // now (the receiver index has it), and a pending message is given to it at once.
  if (route_self_ && e.from == route_self_) {
    const auto w = remote_waits_.find(e.pid);
    if (w != remote_waits_.end() && w->second.seq == e.seq)
      forget_remote_wait(w);
    const auto pit = pending_messages_.find(e.key);
    if (pit != pending_messages_.end() && !pit->second.empty()) {
      value_t msg = std::move(pit->second.front());
      pit->second.pop_front();
      if (!deliver_to(e.pid, e.seq, e.key, msg))
        pit->second.push_front(std::move(msg)); // it waits no more: the message stays first
      else if (pit->second.empty())
        pending_messages_.erase(pit);
    }
    return;
  }
  // A home skips a record whose deadline passed.
  if (e.deadline && *e.deadline <= clock::now())
    return;
  // The pending FIFO answers first (MS-2): its front, by a deliver, in an envelope of its own. A
  // closed receiver's refusal makes it stale at once: a requeue here.
  const auto pit = pending_messages_.find(e.key);
  if (pit != pending_messages_.end() && !pit->second.empty()) {
    value_t msg = std::move(pit->second.front());
    pit->second.pop_front();
    if (pit->second.empty())
      pending_messages_.erase(pit);
    detail::route_entry d = make_entry(detail::route_entry::kind::deliver, e.key);
    d.value = copy_envelope(msg);
    d.from = route_self_;
    d.pid = e.pid;
    d.seq = e.seq;
    detail::lock_fuzz_point(ingress_fuzz_.get(), detail::lock_fuzz_at::post_enter);
    if (!e.from.post(std::move(d)))
      apply_requeue(e.key, std::move(msg), 0);
    return;
  }
  remote_waiters_[e.key].push_back(remote_waiter{e.from, e.pid, e.seq, e.deadline});
  if (router_)
    router_->note_remote_waiters(1);
}

void async_scheduler::home_unsubscribe(const detail::route_entry &e) {
  // Its record, if not used yet (a used one has a deliver in flight, which comes back if stale).
  const auto it = remote_waiters_.find(e.key);
  if (it == remote_waiters_.end())
    return;
  std::deque<remote_waiter> &records = it->second;
  for (auto r = records.begin(); r != records.end(); ++r) {
    if (r->pid == e.pid && r->seq == e.seq && r->from == e.from) {
      records.erase(r);
      if (router_)
        router_->note_remote_waiters(-1);
      break;
    }
  }
  if (records.empty())
    remote_waiters_.erase(it);
}

void async_scheduler::home_taken(const detail::route_entry &e) {
  const auto it = tracked_.find(e.broadcast);
  if (it == tracked_.end())
    return; // untracked: nothing to do
  it->second.taken = true;
  if (--it->second.outstanding == 0)
    tracked_.erase(it);
}

void async_scheduler::home_adopt(detail::route_entry &e) {
  // A tracker a torn-down home forwarded (S2b.0a): tracked here from now on, so its copies' late
  // answers, which reach this home after it, find it. Its id is unique in the app.
  if (e.broadcast == 0 || e.outstanding == 0)
    return;
  tracked_.insert_or_assign(e.broadcast,
                            tracked_broadcast{e.key, std::move(e.value), e.outstanding, e.taken});
}

void async_scheduler::receiver_deliver(detail::route_entry &e) {
  deliver_patched_ = false; // the throw rule: a deliver that throws before patching is stale
  if (deliver_to(e.pid, e.seq, e.key, e.value)) {
    if (e.broadcast != 0) { // a tracked copy reached its waiter
      detail::route_entry t = make_entry(detail::route_entry::kind::taken, e.key);
      t.broadcast = e.broadcast;
      post_routed(std::move(e.from), std::move(t));
    }
    return;
  }
  // Stale (it timed out, was killed or reaped, or waits again with a newer seq): back to its home
  // as a requeue, unless a local waiter there took the message.
  if (!e.requeue_if_stale) {
    ingress_counts_.stale_dropped.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  detail::route_entry r = make_entry(detail::route_entry::kind::requeue, e.key);
  r.value = std::move(e.value);
  r.broadcast = e.broadcast;
  if (post_routed(std::move(e.from), std::move(r)) < 0)
    ingress_counts_.destroy_dropped.fetch_add(1, std::memory_order_relaxed); // no home: ~app
}

void async_scheduler::deliver_threw(detail::route_entry &&e) noexcept {
  try {
    // Its process still waits with that seq, and its record at the home was used: a subscribe for
    // the wait goes after the stale copy's requeue (S2b.0a).
    detail::route_entry again = make_entry(detail::route_entry::kind::subscribe, e.key);
    again.from = route_self_;
    again.pid = e.pid;
    again.seq = e.seq;
    if (const process *p = procs_.find(e.pid))
      again.deadline = p->recv_deadline;
    const group_ref home = e.from;
    if (e.requeue_if_stale) {
      detail::route_entry r = make_entry(detail::route_entry::kind::requeue, e.key);
      r.value = std::move(e.value);
      r.broadcast = e.broadcast;
      if (post_routed(home, std::move(r)) < 0)
        ingress_counts_.destroy_dropped.fetch_add(1, std::memory_order_relaxed);
    } else {
      ingress_counts_.stale_dropped.fetch_add(1, std::memory_order_relaxed);
    }
    group_ref at;
    if (post_routed(home, std::move(again), &at) > 0) {
      const auto w = remote_waits_.find(e.pid);
      if (w != remote_waits_.end() && w->second.seq == e.seq)
        w->second.home = std::move(at);
    }
  } catch (...) {
  }
}

bool async_scheduler::forwards_messages() const {
  // A group that registered a prefix forwards every message it holds but '#' ones, even once
  // unregistered (Ariadne's move unregisters it early) and for a key a longer prefix covers now
  // (S2b.R2, C-M4, C-m4); an island homes nothing for another group: its own keys' messages die
  // with it, counted.
  return prefixed_once_.load(std::memory_order_relaxed) ||
         (router_ && router_->has_prefix(route_self_));
}

void async_scheduler::forward_keyed(detail::route_entry &&e, forward_memo *memo) {
  // One target per key for the whole teardown (S2b.R3): the first entry's goes to the key's home
  // among the open groups (new_home_of), every later one to wherever the first went, so they arrive
  // in order after it (a done target sends them on by its forwarding address).
  forward_memo::iterator slot;
  group_ref to;
  if (memo) {
    slot = memo->try_emplace(e.key).first;
    to = slot->second;
  }
  if (!to && router_)
    to = router_->new_home_of(e.key);
  group_ref at;
  if (post_routed(std::move(to), std::move(e), &at, true) <= 0) {
    ingress_counts_.destroy_dropped.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  if (memo)
    slot->second = std::move(at);
}

void async_scheduler::forward_on(detail::route_entry &&e, forward_memo *memo,
                                 bool prefixed) noexcept {
  // As a teardown forwards its queued and late router entries (exec_router.h): a deliver as a stale
  // one (a requeue to the home that sent it, or dropped when requeue_if_stale is false); a send of
  // a key it homed uncapped, of one it did not home ('#', an island's own) dropped and counted
  // (S2b.R3, K-2); the rest as they are, to the key's target. What no home takes is dropped and
  // counted.
  using kind = detail::route_entry::kind;
  try {
    if (e.what == kind::deliver) {
      if (!e.requeue_if_stale) {
        ingress_counts_.stale_dropped.fetch_add(1, std::memory_order_relaxed);
        return;
      }
      detail::route_entry r = make_entry(kind::requeue, e.key);
      r.value = std::move(e.value);
      r.broadcast = e.broadcast;
      if (!e.from) {
        forward_keyed(std::move(r), memo);
        return;
      }
      if (post_routed(std::move(e.from), std::move(r), nullptr, true) <= 0)
        ingress_counts_.destroy_dropped.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    if (e.what == kind::send) {
      if (!prefixed || never_routed(e.key)) {
        ingress_counts_.destroy_dropped.fetch_add(1, std::memory_order_relaxed);
        return;
      }
      e.uncapped = true;
    }
    forward_keyed(std::move(e), memo);
  } catch (...) {
    ingress_counts_.destroy_dropped.fetch_add(1, std::memory_order_relaxed);
  }
}

void async_scheduler::bounce_route(detail::route_entry &&e) noexcept {
  // A router entry the destructor will not apply (S2-Q10): sent on as a teardown's queued one is,
  // to the key's home among the open groups; a send whatever group it came to (only group 0, at
  // ~app, has router entries left by then), but never on a '#' key.
  forward_on(std::move(e), nullptr, true);
}

void async_scheduler::leave_router() {
  // A second call (Ariadne's move, then the teardown) finds the handle done: nothing to do.
  if (!router_ || route_self_.done())
    return;
  // Each key's target for this teardown, kept by the handle as its forwarding addresses (S2b.R3).
  forward_memo memo;
  // However this ends: a throw still unregisters and marks the handle done, so no group stays
  // closed and registered (S2b.R2, C-m2); what its late queue still holds is dropped, counted.
  struct finish_at_end {
    async_scheduler &s;
    bool armed = true;
    ~finish_at_end() {
      if (!armed)
        return;
      try {
        s.router_->unregister(s.route_self_);
      } catch (...) {
      }
      const std::size_t left = s.route_self_.close_for_good().size();
      s.ingress_counts_.destroy_dropped.fetch_add(left, std::memory_order_relaxed);
    }
  } at_end{*this};
  // (1) A message posted to it from now on goes through its handle as a send (post_message; S2b.R3,
  // K-3), so a host's posts keep their order whichever way they come; then its handle closes: no
  // post reaches this scheduler through it from now on, and a router entry posted through it goes
  // into its late queue until it is done (3).
  {
    std::lock_guard<std::mutex> lock(ingress_mutex_);
    routed_out_ = true;
  }
  route_self_.close();
  // A group that registered a prefix forwards every message it holds but '#' ones; an island homes
  // nothing for another group. No router lookup either way (RT-1).
  const bool prefixed = forwards_messages();
  const auto homed = [&](const std::string &key) { return prefixed && !never_routed(key); };
  const auto forward = [&](detail::route_entry &&e) { forward_keyed(std::move(e), &memo); };
  // (2) What it holds goes on, each key's to its target, in the order it took it, and in key order
  // where no order was taken (DT-4).
  // (2a) The pending messages of the keys it homed, as uncapped sends, each key's in FIFO order.
  std::vector<std::string> keys;
  for (const auto &[key, q] : pending_messages_)
    if (homed(key))
      keys.push_back(key);
  std::sort(keys.begin(), keys.end());
  for (const std::string &key : keys) {
    const auto it = pending_messages_.find(key);
    const std::deque<value_t> q = std::move(it->second);
    pending_messages_.erase(it);
    for (const value_t &msg : q) {
      detail::route_entry e = make_entry(detail::route_entry::kind::send, key);
      e.value = msg;
      e.uncapped = true;
      forward(std::move(e));
    }
  }
  // The pending messages of the keys it did not home (an island's own keys, '#' keys) die with it,
  // counted.
  std::uint64_t dying = 0;
  for (const auto &[key, q] : pending_messages_)
    dying += q.size();
  pending_messages_.clear();
  ingress_counts_.destroy_dropped.fetch_add(dying, std::memory_order_relaxed);
  // (2b) Its remote-waiter records, as subscribes with the same seq, in subscription order; counted
  // out of the router as they leave it (the new home counts each in as it records it; this group's
  // registration, until (3), keeps the router on meanwhile).
  std::map<std::string, std::deque<remote_waiter>> records;
  records.swap(remote_waiters_);
  int moved = 0;
  for (const auto &[key, list] : records)
    moved += static_cast<int>(list.size());
  router_->note_remote_waiters(-moved);
  for (const auto &[key, list] : records) {
    for (const remote_waiter &r : list) {
      detail::route_entry e = make_entry(detail::route_entry::kind::subscribe, key);
      e.from = r.from;
      e.pid = r.pid;
      e.seq = r.seq;
      e.deadline = r.deadline;
      forward(std::move(e));
    }
  }
  // (2c) Its broadcast trackers (adopt), by id: the new home takes each over, and the copies' late
  // answers find it there (this handle's late queue, or its forwarding address, sends them after).
  std::vector<std::uint64_t> ids;
  ids.reserve(tracked_.size());
  for (const auto &[id, t] : tracked_)
    ids.push_back(id);
  std::sort(ids.begin(), ids.end());
  for (const std::uint64_t id : ids) {
    tracked_broadcast &t = tracked_.at(id);
    detail::route_entry e = make_entry(detail::route_entry::kind::adopt, t.key);
    e.value = std::move(t.value);
    e.broadcast = id;
    e.outstanding = static_cast<std::uint32_t>(t.outstanding);
    e.taken = t.taken;
    forward(std::move(e));
  }
  tracked_.clear();
  // (2d) Its queued messages and router entries, in queue order (taken out under the lock, sent on
  // outside it): a host post of a key it homed as an uncapped send, the router entries as
  // forward_on sends them on; a message of a key it did not home ('#' keys among them) dies with
  // it, counted. Its commands stay for the destructor to refuse.
  std::vector<ingress_entry> taken;
  {
    std::lock_guard<std::mutex> lock(ingress_mutex_);
    std::vector<ingress_entry> kept;
    for (ingress_entry &entry : ingress_)
      (std::holds_alternative<posted_control>(entry) ? kept : taken).push_back(std::move(entry));
    ingress_.swap(kept);
    ingress_capped_ = 0; // what is left counts against no cap: nothing capped stays
  }
  for (ingress_entry &entry : taken) {
    if (posted_message *m = std::get_if<posted_message>(&entry)) {
      if (!homed(m->key)) {
        ingress_counts_.destroy_dropped.fetch_add(1, std::memory_order_relaxed);
        continue;
      }
      detail::route_entry e = make_entry(detail::route_entry::kind::send, m->key);
      e.value = std::move(m->value);
      e.uncapped = true;
      forward(std::move(e));
    } else {
      forward_on(std::move(std::get<detail::route_entry>(entry)), &memo, prefixed);
    }
  }
  // (2e), (3) Its late entries, in arrival order, until none are left; then, under the handle's
  // lock, it unregisters its prefixes, keeps `memo` as the forwarding addresses and is done. Every
  // target was open when chosen, so nothing forwarded comes back here. Posters as fast as this
  // forward could keep the loop going: once a round's batch does not shrink, reaches the ingress
  // cap, or a few rounds have gone, it seals the handle, so only another teardown's (finite)
  // forward still comes in, and the loop ends.
  constexpr int kSealAfterRounds = 4;
  const std::size_t cap = ingress_cap_.load(std::memory_order_relaxed);
  const std::size_t seal_at = cap != 0 ? cap : std::size_t{65536};
  std::size_t last = std::numeric_limits<std::size_t>::max();
  bool sealed = false;
  for (int round = 0;; ++round) {
    std::vector<detail::route_entry> late;
    if (route_self_.finish(late, memo, [this] { router_->unregister(route_self_); }))
      break;
    if (!sealed &&
        (round + 1 >= kSealAfterRounds || late.size() >= last || late.size() >= seal_at)) {
      route_self_.seal();
      sealed = true;
    }
    last = late.size();
    for (detail::route_entry &e : late)
      forward_on(std::move(e), &memo, prefixed);
  }
  at_end.armed = false;
  // Its kills then unsubscribe its processes' remote waits.
}

} // namespace cvc::state_exec
