// Stage 2's router (docs/STATE_EXEC_THREADED.md 4.6; S2.md S2b, S2-Q10): group handles, channel
// homes by registered prefix, the host's post, and async_scheduler's ends of the protocol (S2b.4:
// send, subscribe, unsubscribe, deliver, requeue, taken; broadcast tracking; rehoming).
// exec_router.h has the rules.
#include <algorithm>
#include <cvc/core/app.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/exec_group.h>
#include <cvc/state/state_exec/exec_router.h>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace cvc::state_exec {

namespace detail {
// What a group_ref points at. A post holds `mu` shared while it pushes into the scheduler's
// ingress, and close() takes it exclusively, so once close() returns no post touches the scheduler.
struct route_target {
  mutable std::shared_mutex mu;
  async_scheduler *sched = nullptr; // null once closed
  exec_group *group = nullptr;      // null for group 0
  bool group0 = false;
  std::uint64_t id = 0;
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
}

bool group_ref::closed() const noexcept {
  if (!t_)
    return true;
  std::shared_lock<std::shared_mutex> lock(t_->mu);
  return t_->sched == nullptr;
}

bool group_ref::is_group0() const noexcept { return t_ && t_->group0; }

exec_group *group_ref::group() const noexcept { return t_ ? t_->group : nullptr; }

std::uint64_t group_ref::id() const noexcept { return t_ ? t_->id : 0; }

bool group_ref::post(detail::route_entry e) const {
  if (!t_)
    return false;
  std::shared_lock<std::shared_mutex> lock(t_->mu);
  return t_->sched && t_->sched->post_route(std::move(e));
}

bool group_ref::post_message(const std::string &key, const value_t &value) const {
  if (!t_)
    return false;
  std::shared_lock<std::shared_mutex> lock(t_->mu);
  return t_->sched && t_->sched->post_message(key, value);
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
  std::atomic<int> others{0};          // registrations of groups other than group 0
  std::atomic<long> remote_waiters{0}; // note_remote_waiters
  mutable std::atomic<std::uint64_t> consulted{0};
  // CVC_EXEC_FUZZ's yields at the host's posts (S2-Q15), read as the router is made; null: off.
  std::unique_ptr<detail::lock_fuzz> fuzz;
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
  impl &m = *impl_;
  {
    std::unique_lock<std::shared_mutex> lock(m.mu);
    if (prefix.empty() && group.is_group0()) {
      m.group0 = group;
    } else {
      m.regs.push_back({std::move(prefix), group});
      if (!group.is_group0())
        m.others.fetch_add(1, std::memory_order_relaxed);
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
    m.others.fetch_sub(removed, std::memory_order_relaxed);
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
      for (const impl::registration &r : m.regs) {
        const std::string &p = r.prefix;
        const bool covers = key.size() >= p.size() && key.compare(0, p.size(), p) == 0 &&
                            (key.size() == p.size() || p.empty() || key[p.size()] == '.');
        if (covers && (!best || p.size() > best->prefix.size()))
          best = &r;
      }
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

bool exec_router::active() const noexcept {
  return impl_->others.load(std::memory_order_acquire) > 0 ||
         impl_->remote_waiters.load(std::memory_order_acquire) > 0;
}

std::uint64_t exec_router::epoch() const noexcept {
  return impl_->epoch.load(std::memory_order_acquire);
}

std::uint64_t exec_router::consulted() const noexcept {
  return impl_->consulted.load(std::memory_order_relaxed);
}

void exec_router::note_remote_waiters(int delta) noexcept {
  impl_->remote_waiters.fetch_add(delta, std::memory_order_acq_rel);
}

namespace {

// Post `e` to `home` (exec_router.h, "Teardown and rehoming"): a handle closed meanwhile (its
// group's teardown unregisters, then closes) is resolved again, and `e` posted to the key's new
// home, so nothing is lost. Sent on to another home than the first, a requeue's broadcast id (the
// first home's) is cleared, and a taken is dropped: nothing else tracks it. An empty `home` is
// resolved first. 1: taken (`posted_to` gets the home); 0: refused by the home (its cap); -1: no
// home took it (group 0's teardown, or no router to ask).
int post_with_rehome(const exec_router *router, group_ref home, detail::route_entry e,
                     detail::lock_fuzz *fuzz, group_ref *posted_to) {
  for (int tries = 0; tries < 16; ++tries) {
    if (home) {
      detail::lock_fuzz_point(fuzz, detail::lock_fuzz_at::post_enter); // S2-Q15's router posts
      if (home.post(e)) { // a copy: a refused post consumes what it was given
        if (posted_to)
          *posted_to = std::move(home);
        return 1;
      }
      if (!home.closed())
        return 0; // refused: a capped send at the home's cap
    }
    if (!router || e.what == detail::route_entry::kind::taken)
      return -1;
    group_ref next = router->home_of(e.key);
    if (!next || next == home)
      return -1; // still the closed one: group 0's teardown
    e.broadcast = 0;
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

} // namespace

// The host's post (exec_pool::post_message): to the key's home as a send, which counts against
// the home's cap; group 0's post_message while the router is off (every key is homed there).
bool exec_router::post_message(const std::string &key, const value_t &value) {
  if (!app_)
    return false;
  if (!active() || never_routed(key))
    return app_->exec_scheduler().post_message(key, value);
  detail::route_entry e;
  e.what = detail::route_entry::kind::send;
  e.key = key;
  e.value = value;
  return post_with_rehome(this, home_of(key), std::move(e), impl_->fuzz.get(), nullptr) > 0;
}

// --- async_scheduler's ends of the protocol (S2b.4; exec_router.h) ---

namespace {
// The cache of homes is capped (computed keys are unbounded): full, it starts over.
constexpr std::size_t kHomeCacheCap = 4096;
} // namespace

group_ref async_scheduler::cached_home(const std::string &key) const {
  const std::uint64_t epoch = router_->epoch(); // before the lookup: a move meanwhile clears it
  if (epoch != home_cache_epoch_) {
    home_cache_.clear();
    home_cache_epoch_ = epoch;
  }
  if (const auto it = home_cache_.find(key); it != home_cache_.end())
    return it->second;
  group_ref home = router_->home_of(key);
  if (home_cache_.size() >= kHomeCacheCap)
    home_cache_.clear();
  home_cache_.emplace(key, home);
  return home;
}

bool async_scheduler::homed_elsewhere(const std::string &key, group_ref *home) const {
  // RT-1: with one group (or no router) this is two atomic loads, and the router is not consulted.
  if (!router_ || !router_->active() || never_routed(key))
    return false;
  group_ref h = cached_home(key);
  if (!h || h == route_self_)
    return false;
  if (home)
    *home = std::move(h);
  return true;
}

int async_scheduler::post_routed(group_ref home, detail::route_entry e,
                                 group_ref *posted_to) const {
  return post_with_rehome(router_, std::move(home), std::move(e), ingress_fuzz_.get(), posted_to);
}

void async_scheduler::subscribe_if_remote(process &proc, const std::string &path) {
  // msg-recv on a key homed elsewhere (td 4.6): the process is parked here with its new recv_seq,
  // and subscribes at the home, which answers from its pending FIFO or records a remote waiter.
  group_ref home;
  if (!homed_elsewhere(path, &home))
    return;
  detail::route_entry e;
  e.what = detail::route_entry::kind::subscribe;
  e.key = path;
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
    detail::route_entry e;
    e.what = detail::route_entry::kind::unsubscribe;
    e.key = std::move(w.key);
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
  case kind::send: // a msg-send (or a host's post) for a key homed here
    deliver_home(e.key, e.value, false);
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
    home_requeue(e);
    break;
  case kind::taken:
    home_taken(e);
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
  // remote waiter alone, untracked; k >= 2, tracked here by an id until every copy answers.
  const bool requeue_if_stale = !local_took;
  const std::uint64_t id = !local_took && live.size() >= 2 ? ++broadcast_seq_ : 0;
  // One envelope copy per receiving group, made at its first record, in subscription order.
  std::vector<std::pair<group_ref, value_t>> copies;
  std::size_t posted = 0;
  for (const remote_waiter *r : live) {
    std::size_t c = 0;
    while (c < copies.size() && copies[c].first != r->from)
      ++c;
    if (c == copies.size())
      copies.emplace_back(r->from, copy_envelope(msg));
    detail::route_entry d;
    d.what = detail::route_entry::kind::deliver;
    d.key = key;
    d.value = copies[c].second;
    d.from = route_self_;
    d.pid = r->pid;
    d.seq = r->seq;
    d.broadcast = id;
    d.requeue_if_stale = requeue_if_stale;
    detail::lock_fuzz_point(ingress_fuzz_.get(), detail::lock_fuzz_at::post_enter);
    if (r->from.post(std::move(d)))
      ++posted;
    else if (local_took) // its group is gone: a stale copy, dropped as one would be
      ingress_counts_.stale_dropped.fetch_add(1, std::memory_order_relaxed);
  }
  // A copy its group refused (torn down) is a stale one already: the outstanding count is the
  // copies posted. None posted: the caller queues the message, as with no waiter.
  if (id != 0 && posted > 0)
    tracked_.emplace(id, tracked_broadcast{key, msg, posted, false});
  return posted;
}

void async_scheduler::home_subscribe(detail::route_entry &e) {
  // A record rehomed here from a torn-down home for a process of this group: its wait is local
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
  // The pending FIFO answers first (MS-2): its front, by a deliver, in an envelope of its own.
  const auto pit = pending_messages_.find(e.key);
  if (pit != pending_messages_.end() && !pit->second.empty()) {
    detail::route_entry d;
    d.what = detail::route_entry::kind::deliver;
    d.key = e.key;
    d.value = copy_envelope(pit->second.front());
    d.from = route_self_;
    d.pid = e.pid;
    d.seq = e.seq;
    detail::lock_fuzz_point(ingress_fuzz_.get(), detail::lock_fuzz_at::post_enter);
    if (e.from.post(std::move(d))) { // else its group is gone: the message stays
      pit->second.pop_front();
      if (pit->second.empty())
        pending_messages_.erase(pit);
    }
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

void async_scheduler::home_requeue(detail::route_entry &e) {
  // A tracked broadcast's stale copy: dropped when another copy was taken; queued once when every
  // copy came back stale (the home's own copy of the message); else it waits for the others. An id
  // not tracked here (the home was torn down meanwhile) is applied as untracked.
  value_t msg = std::move(e.value);
  if (e.broadcast != 0) {
    const auto it = tracked_.find(e.broadcast);
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
  // Back at its home: its waiters now, else the front of the pending FIFO, past its cap.
  ingress_counts_.requeued.fetch_add(1, std::memory_order_relaxed);
  deliver_home(e.key, msg, true);
}

void async_scheduler::home_taken(const detail::route_entry &e) {
  const auto it = tracked_.find(e.broadcast);
  if (it == tracked_.end())
    return; // untracked (torn down and rehomed meanwhile): nothing to do
  it->second.taken = true;
  if (--it->second.outstanding == 0)
    tracked_.erase(it);
}

void async_scheduler::receiver_deliver(detail::route_entry &e) {
  deliver_patched_ = false; // the throw rule: a deliver that throws before patching goes home
  if (deliver_to(e.pid, e.seq, e.key, e.value)) {
    if (e.broadcast != 0) { // a tracked copy reached its waiter
      detail::route_entry t;
      t.what = detail::route_entry::kind::taken;
      t.key = std::move(e.key);
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
  detail::route_entry r;
  r.what = detail::route_entry::kind::requeue;
  r.key = std::move(e.key);
  r.value = std::move(e.value);
  r.broadcast = e.broadcast;
  if (post_routed(std::move(e.from), std::move(r)) < 0)
    ingress_counts_.destroy_dropped.fetch_add(1, std::memory_order_relaxed); // no home: ~app
}

void async_scheduler::bounce_route(detail::route_entry &&e) noexcept {
  // A router entry this scheduler will not apply (its teardown, its destructor; a deliver that
  // threw before patching): a deliver goes back to its home as a requeue (or is dropped as a stale
  // copy of one a local waiter took); a send (uncapped now), a subscribe, an unsubscribe and a
  // requeue go to the key's home, a requeue untracked (its id was this home's); a taken names a
  // broadcast this home tracked, gone with it. What no home takes is dropped and counted.
  using kind = detail::route_entry::kind;
  try {
    group_ref to;
    switch (e.what) {
    case kind::deliver:
      if (!e.requeue_if_stale) {
        ingress_counts_.stale_dropped.fetch_add(1, std::memory_order_relaxed);
        return;
      }
      e.what = kind::requeue;
      to = std::move(e.from);
      e.from = group_ref();
      e.pid = -1;
      e.seq = 0;
      break;
    case kind::taken:
      ingress_counts_.destroy_dropped.fetch_add(1, std::memory_order_relaxed);
      return;
    case kind::send:
      e.uncapped = true;
      break;
    case kind::requeue:
      e.broadcast = 0;
      break;
    case kind::subscribe:
    case kind::unsubscribe:
      break;
    }
    if (!to && router_)
      to = router_->home_of(e.key);
    if (post_routed(std::move(to), std::move(e)) <= 0)
      ingress_counts_.destroy_dropped.fetch_add(1, std::memory_order_relaxed);
  } catch (...) {
    ingress_counts_.destroy_dropped.fetch_add(1, std::memory_order_relaxed);
  }
}

void async_scheduler::leave_router() {
  if (!router_)
    return;
  // The keys of its pending messages it is the home of, found before its prefixes go. With the
  // router off it homes none (it registered no prefix): its FIFO was local messaging, which goes
  // with it, and nothing is asked of the router (RT-1). '#' keys are never routed.
  std::vector<std::string> keys;
  if (router_->active()) {
    for (const auto &[key, q] : pending_messages_)
      if (!never_routed(key) && cached_home(key) == route_self_)
        keys.push_back(key);
    std::sort(keys.begin(), keys.end()); // no hash order reaches a post order (DT-4)
  }
  // (1) Its prefixes go, then its handle closes: a poster refused from now on resolves the key's
  // home again and finds the new one (exec_router.h, "Teardown and rehoming").
  router_->unregister(route_self_);
  route_self_.close();
  // (2) The router entries still queued here leave the queue (its messages and commands stay, for
  // the destructor to drop or refuse).
  std::vector<detail::route_entry> queued;
  {
    std::lock_guard<std::mutex> lock(ingress_mutex_);
    std::vector<ingress_entry> kept;
    kept.reserve(ingress_.size());
    for (ingress_entry &entry : ingress_) {
      if (detail::route_entry *r = std::get_if<detail::route_entry>(&entry)) {
        if (r->what == detail::route_entry::kind::send && !r->uncapped && ingress_capped_ > 0)
          --ingress_capped_;
        queued.push_back(std::move(*r));
      } else {
        kept.push_back(std::move(entry));
      }
    }
    ingress_.swap(kept);
  }
  // (3) The pending messages of the keys it homed, as uncapped sends to their new homes, each
  // key's in FIFO order, the keys in order.
  for (const std::string &key : keys) {
    const auto it = pending_messages_.find(key);
    const std::deque<value_t> q = std::move(it->second);
    pending_messages_.erase(it);
    for (const value_t &msg : q) {
      detail::route_entry e;
      e.what = detail::route_entry::kind::send;
      e.key = key;
      e.value = msg;
      e.uncapped = true;
      if (post_routed(router_->home_of(key), std::move(e)) <= 0)
        ingress_counts_.destroy_dropped.fetch_add(1, std::memory_order_relaxed);
    }
  }
  // (4) Its remote-waiter records, as subscribes with the same seq, in key and subscription order.
  // Counted out of the router only once they are posted, so it stays on meanwhile (the new home
  // counts each in as it records it).
  std::map<std::string, std::deque<remote_waiter>> records;
  records.swap(remote_waiters_);
  int moved = 0;
  for (const auto &[key, list] : records) {
    moved += static_cast<int>(list.size());
    for (const remote_waiter &r : list) {
      detail::route_entry e;
      e.what = detail::route_entry::kind::subscribe;
      e.key = key;
      e.from = r.from;
      e.pid = r.pid;
      e.seq = r.seq;
      e.deadline = r.deadline;
      if (post_routed(router_->home_of(key), std::move(e)) <= 0)
        ingress_counts_.destroy_dropped.fetch_add(1, std::memory_order_relaxed);
    }
  }
  router_->note_remote_waiters(-moved);
  // Its tracked broadcasts go: a copy answering now reaches the key's new home untracked.
  tracked_.clear();
  // (5) The queued entries, in their order. Its kills then unsubscribe its processes' remote waits.
  for (detail::route_entry &e : queued)
    bounce_route(std::move(e));
}

} // namespace cvc::state_exec
