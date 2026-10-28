// A scheduler's lock home: the home actions of docs/STATE_EXEC_LOCKS_PROTOCOL.md section 5.

#include <algorithm>
#include <cvc/state/state_exec/detail/lock_home.h>
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/process_table.h>
#include <cvc/state/state_exec/stackless_evaluator.h>
#include <sstream>
#include <unordered_set>

namespace cvc::state_exec::detail {

namespace {

constexpr std::size_t kDefaultCacheCapacity = 1024;

bool overlap(std::string_view a, std::string_view b) {
  return key_prefix(a, b) || key_prefix(b, a);
}

bool compat(lock_mode a, lock_mode b) { return a == lock_mode::shared && b == lock_mode::shared; }

// Whether one of `h`'s keys is `k` or an ancestor of it (protocol 1's under).
bool under(const mhold &h, std::string_view k) {
  for (const lock_node *n : h.keys)
    if (key_prefix(n->key, k))
      return true;
  return false;
}

const char *mode_name(lock_mode m) { return m == lock_mode::exclusive ? "exclusive" : "shared"; }

// shown(keys) of a request or hold (protocol 5.10): a string for one key, a list for several.
template <class Keys, class Text>
value_t shown_keys(const Keys &keys, Text text, std::string_view root) {
  if (keys.size() == 1)
    return value_t(lock_home::show(text(keys[0]), root));
  std::vector<value_t> out;
  out.reserve(keys.size());
  for (const auto &k : keys)
    out.emplace_back(lock_home::show(text(k), root));
  return make_list(std::move(out));
}

[[noreturn]] void raise_lock_error(const std::string &reason, const home_request &req,
                                   std::string_view root) {
  value_t key = shown_keys(req.keys, [](const home_request::key &k) { return k.text(); }, root);
  throw dsl_error("LockError", "with-lock: " + reason, "dsl",
                  {{"reason", value_t(reason)},
                   {"key", std::move(key)},
                   {"mode", value_t(std::string(mode_name(req.mode)))}});
}

// The lock_enter frame of a parked waiter: the frame below its await frame (protocol 5.9).
eval_frame *lock_enter_frame(process &p) {
  auto &st = p.state.stack;
  if (st.size() < 2 || st.back().phase != eval_phase::await_call)
    return nullptr;
  eval_frame &f = st[st.size() - 2];
  if (f.phase != eval_phase::lock_enter ||
      f.extra_vals.size() <= static_cast<std::size_t>(lock_frame_slot::wait_or_outcome))
    return nullptr;
  return &f;
}

void clear_wait_fields(process &p) {
  p.lock_waiting = false;
  p.lock_deadline.reset();
  p.lock_expired_pending = false;
}

} // namespace

// ---------------------------------------------------------------------------------------------
// Requests
// ---------------------------------------------------------------------------------------------

void canonicalize(home_request &r) {
  std::stable_sort(r.keys.begin(), r.keys.end(),
                   [](const home_request::key &a, const home_request::key &b) {
                     return key_less(a.text(), b.text());
                   });
  // Sorted, an ancestor (or an equal key) comes right before what it covers, so it is enough
  // to compare each key with the last one kept.
  std::size_t kept = 0;
  for (std::size_t i = 0; i < r.keys.size(); ++i) {
    if (kept > 0 && key_prefix(r.keys[kept - 1].text(), r.keys[i].text()))
      continue;
    if (kept != i)
      r.keys[kept] = std::move(r.keys[i]);
    ++kept;
  }
  r.keys.erase(r.keys.begin() + static_cast<std::ptrdiff_t>(kept), r.keys.end());
}

value_t outcome_value(const frame_outcome &o) {
  if (o.hold)
    return value_t(static_cast<int64_t>(*o.hold));
  return o.error;
}

std::optional<frame_outcome> outcome_of(const value_t &slot) {
  if (const auto *id = std::get_if<int64_t>(&slot.v))
    return frame_outcome{static_cast<std::uint64_t>(*id), nil_value};
  if (std::holds_alternative<dict_ptr>(slot.v))
    return frame_outcome{std::nullopt, slot};
  return std::nullopt; // nil, or the wait's handle: not collected yet
}

// ---------------------------------------------------------------------------------------------
// The home
// ---------------------------------------------------------------------------------------------

std::size_t lock_home::cache_hash::operator()(const cache_key &k) const noexcept {
  return std::hash<std::string_view>{}(k.written) ^ (std::size_t(k.scope) * 0x9E3779B97F4A7C15ull);
}

lock_home::lock_home(std::shared_ptr<lock_manager> mgr, lock_sched_ops &ops)
    : mgr_(std::move(mgr)), table_(mgr_->table()), ops_(ops) {
  const lock_test_hooks *hooks = lock_test_hooks_get();
  const std::size_t capacity =
      hooks && hooks->home_cache_capacity ? hooks->home_cache_capacity : kDefaultCacheCapacity;
  entries_.resize(capacity);
  free_.reserve(capacity);
  for (std::size_t i = capacity; i-- > 0;)
    free_.push_back(i);
  index_.reserve(capacity);
  lock_notes notes;
  id_ = table_.attach_home(false, nullptr, nullptr, notes); // phase L1: no priorities, no port
  deliver(notes);
}

lock_home::~lock_home() {
  // 5.7. Phase L1 has no port to close. The detach ends every process of the home with
  // abandon and drops the cache's references; nothing it notifies targets this home.
  try {
    lock_notes notes;
    table_.detach_home(id_, notes);
  } catch (...) {
    // A destructor can't report it; the table's own check() finds what is left.
  }
  ops_.for_each_process([](process &p) {
    if (p.locks) {
      p.locks->holds.clear();
      p.locks->ticket.reset();
    }
    clear_wait_fields(p);
    p.inherited_priority = INT_MAX;
  });
}

lock_pid lock_home::pid_of(const process &p) const { return {id_, p.pid}; }

rec_info lock_home::info_of(const process &p) { return rec_info{p.owner, p.priority}; }

process_locks &lock_home::locks_of(process &p) {
  if (!p.locks)
    p.locks = std::make_shared<process_locks>();
  return *p.locks;
}

void lock_home::begin_request() {
  ++pin_epoch_;
  misses_.clear();
}

std::uint32_t lock_home::scope_id(std::string_view root_path) {
  const auto it = scopes_.find(root_path);
  if (it != scopes_.end())
    return it->second;
  const auto id = static_cast<std::uint32_t>(scopes_.size());
  scopes_.emplace(std::string(root_path), id);
  return id;
}

lock_node *lock_home::lookup(std::uint32_t scope, std::string_view written) {
  const auto it = index_.find(cache_key{scope, written});
  if (it == index_.end())
    return nullptr;
  entry &e = entries_[it->second];
  if (!e.node)
    return nullptr; // reserved for a miss of this request
  e.used = true;
  e.pinned = pin_epoch_;
  return e.node;
}

void lock_home::free_entry(std::size_t slot) {
  entry &e = entries_[slot];
  if (e.live)
    index_.erase(cache_key{e.scope, e.written});
  e.live = false;
  e.node = nullptr;
  e.written.clear();
  e.used = false;
  free_.push_back(slot);
}

std::optional<std::size_t> lock_home::victim() {
  // CLOCK: a used entry gets a second chance; an entry of this request (pinned, or reserved for
  // one of its misses) is never taken. Twice round the clock finds one if any can go.
  const std::size_t n = entries_.size();
  for (std::size_t tries = 0; tries < 2 * n; ++tries) {
    const std::size_t slot = hand_;
    hand_ = (hand_ + 1) % n;
    entry &e = entries_[slot];
    if (!e.live || !e.node || e.pinned == pin_epoch_)
      continue;
    if (e.used) {
      e.used = false;
      continue;
    }
    return slot;
  }
  return std::nullopt;
}

void lock_home::build_table_request(const home_request &req, lock_request &treq,
                                    cache_delta &delta) {
  treq.mode = req.mode;
  treq.keys.clear();
  misses_.clear();
  for (std::size_t i = 0; i < req.keys.size(); ++i) {
    const home_request::key &k = req.keys[i];
    key_ref r;
    r.node = k.node;
    r.key = k.node ? std::string_view() : std::string_view(k.absolute);
    if (!k.node) {
      // A miss: a slot for it, by eviction if none is free. The entry is made now (its key
      // copied, indexed), so nothing after the table call allocates; the node comes after.
      std::optional<std::size_t> slot;
      if (!free_.empty()) {
        slot = free_.back();
        free_.pop_back();
      } else if ((slot = victim())) {
        delta.drop.push_back(entries_[*slot].node);
        entry &old = entries_[*slot];
        index_.erase(cache_key{old.scope, old.written});
        old.live = false;
        old.node = nullptr;
      }
      if (slot) {
        entry &e = entries_[*slot];
        e.scope = k.scope;
        e.written.assign(k.written);
        e.node = nullptr;
        e.pinned = pin_epoch_;
        e.used = true;
        e.live = index_.emplace(cache_key{e.scope, e.written}, *slot).second;
        if (e.live) {
          r.ref = true;
          misses_.emplace_back(i, *slot);
        } else {
          // The same key written twice in one request: the first entry serves both.
          e.written.clear();
          free_.push_back(*slot);
        }
      }
    }
    treq.keys.push_back(r);
  }
}

void lock_home::commit_misses(const lock_request &treq) noexcept {
  for (const auto &[i, slot] : misses_) {
    entry &e = entries_[slot];
    e.node = treq.keys[i].node;
    if (!e.node) // the table took no reference (it can't happen): don't keep the entry
      free_entry(slot);
  }
  misses_.clear();
}

void lock_home::abandon_misses() noexcept {
  for (const auto &[i, slot] : misses_)
    free_entry(slot);
  misses_.clear();
}

enter_result lock_home::try_enter(process &p, home_request &req, std::string_view root) {
  const bool in_handler = p.in_signal_handler || p.in_watch_handler;
  if (p.locks && !p.locks->holds.empty()) {
    std::vector<mhold> &holds = p.locks->holds;
    // 1. The handler rule (design 5.7): a handler may not take, or nest under, a hold of the
    // program it interrupted that overlaps its request, unless both are shared and neither
    // covers the other.
    if (in_handler) {
      for (const home_request::key &k : req.keys)
        for (const mhold &h : holds) {
          if (h.in_handler)
            continue;
          const bool covered = under(h, k.text()) &&
                               (h.mode == lock_mode::exclusive || req.mode == lock_mode::shared);
          bool conflicting = false;
          for (const lock_node *hk : h.keys)
            conflicting = conflicting || (overlap(hk->key, k.text()) && !compat(h.mode, req.mode));
          if (covered || conflicting)
            raise_lock_error("handler", req, root);
        }
    }
    const auto in_view = [in_handler](const mhold &h) { return !in_handler || h.in_handler; };
    // 2. Upgrade: an X request under an own S hold and no own X hold.
    if (req.mode == lock_mode::exclusive) {
      for (const home_request::key &k : req.keys) {
        bool s_under = false, x_under = false;
        for (const mhold &h : holds) {
          if (!in_view(h) || !under(h, k.text()))
            continue;
          (h.mode == lock_mode::shared ? s_under : x_under) = true;
        }
        if (s_under && !x_under)
          raise_lock_error("upgrade", req, root);
      }
    }
    // 3. Covered: every key under an own hold that admits the mode. The manager isn't called.
    const auto covers = [&](const mhold &h, std::string_view k) {
      return in_view(h) && under(h, k) &&
             (h.mode == lock_mode::exclusive || req.mode == lock_mode::shared);
    };
    bool all = true;
    for (const home_request::key &k : req.keys) {
      bool any = false;
      for (const mhold &h : holds)
        any = any || covers(h, k.text());
      all = all && any;
    }
    if (all) {
      for (mhold &h : holds)
        if (covers(h, req.keys.front().text())) {
          ++h.count;
          return {enter_result::kind::covered, h.id, h.gen, false};
        }
    }
  }

  // 4. The fast path. The mirror's slot and the hold's key storage are made first: nothing after
  // a grant may throw (protocol 5.1).
  process_locks &pl = locks_of(p);
  pl.holds.reserve(pl.holds.size() + 1);
  mhold made;
  made.keys.reserve(req.keys.size());
  lock_request treq;
  cache_delta delta;
  build_table_request(req, treq, delta);
  std::optional<grant> g;
  try {
    g = table_.try_acquire(pid_of(p), info_of(p), treq, delta);
  } catch (...) {
    abandon_misses();
    throw;
  }
  commit_misses(treq);
  if (!g)
    return {enter_result::kind::busy, 0, 0, false};
  made.id = g->hold;
  for (const key_ref &r : treq.keys)
    made.keys.push_back(r.node);
  made.mode = req.mode;
  made.count = 1;
  made.gen = g->gen;
  made.abandoned = g->abandoned;
  made.in_handler = in_handler;
  pl.holds.push_back(std::move(made));
  if (const lock_test_hooks *hooks = lock_test_hooks_get(); hooks && hooks->after_grant)
    hooks->after_grant(p.pid);
  return {enter_result::kind::granted, g->hold, g->gen, g->abandoned};
}

std::optional<frame_outcome> lock_home::wait(process &p, home_request &req,
                                             std::optional<double> timeout, std::string_view root) {
  // The busy path's one clock read (protocol 5.2).
  std::optional<lock_clock::time_point> deadline;
  if (timeout)
    deadline = time_after(lock_clock::now(), *timeout);
  process_locks &pl = locks_of(p);
  pl.holds.reserve(pl.holds.size() + 1); // collect's grant can't fail on the mirror
  lock_request treq;
  cache_delta delta;
  build_table_request(req, treq, delta);
  lock_notes notes;
  block_result br;
  try {
    br = table_.block(pid_of(p), info_of(p), treq, delta, notes);
  } catch (...) {
    abandon_misses();
    throw;
  }
  commit_misses(treq);
  pl.ticket = br.tid;
  pl.ticket_mode = req.mode;
  pl.ticket_keys.clear();
  for (const key_ref &r : treq.keys)
    pl.ticket_keys.emplace_back(r.text());
  pl.ticket_in_handler = p.in_signal_handler || p.in_watch_handler;
  pl.ticket_root.assign(root);
  pl.ticket_timeout = timeout;
  deliver(notes); // relevels only (phase L2); none in L1
  if (br.state == ticket_state::queued) {
    p.lock_waiting = true;
    p.lock_deadline = deadline;
    ops_.park_lock(p);
    return std::nullopt;
  }
  return collect(p, br.tid); // granted or failed at once: not parked
}

frame_outcome lock_home::collect(process &p, std::uint64_t tid) {
  process_locks &pl = locks_of(p);
  lock_outcome out = table_.resume(pid_of(p), tid);
  pl.ticket.reset();
  frame_outcome o;
  if (out.what == lock_outcome::kind::granted) {
    mhold h;
    h.id = out.g.hold;
    const std::span<lock_node *const> keys = table_.hold_keys(out.g.hold);
    h.keys.assign(keys.begin(), keys.end());
    h.mode = pl.ticket_mode;
    h.count = 1;
    h.gen = out.g.gen;
    h.abandoned = out.g.abandoned;
    h.in_handler = pl.ticket_in_handler;
    pl.holds.push_back(std::move(h));
    o.hold = out.g.hold;
    if (const lock_test_hooks *hooks = lock_test_hooks_get(); hooks && hooks->after_grant)
      hooks->after_grant(p.pid);
  } else {
    o.error = outcome_error(out, pl);
  }
  return o;
}

value_t lock_home::outcome_error(const lock_outcome &out, const process_locks &pl) const {
  const std::string &root = pl.ticket_root;
  value_t key =
      shown_keys(pl.ticket_keys, [](const std::string &k) { return std::string_view(k); }, root);
  const value_t mode(std::string(mode_name(pl.ticket_mode)));
  if (out.what == lock_outcome::kind::deadlock) {
    std::vector<value_t> cycle;
    for (const deadlock_edge &e : out.cycle)
      cycle.push_back(make_dict({{"pid", e.pid ? value_t(static_cast<int64_t>(*e.pid)) : nil_value},
                                 {"key", value_t(show(e.key, root))}}));
    return exception_from(dsl_error("DeadlockError", "with-lock: deadlock", "dsl",
                                    {{"reason", value_t(std::string("deadlock"))},
                                     {"key", std::move(key)},
                                     {"mode", mode},
                                     {"cycle", make_list(std::move(cycle))}}));
  }
  std::ostringstream msg;
  msg << "with-lock: not granted";
  if (pl.ticket_timeout)
    msg << " within " << *pl.ticket_timeout << " s";
  return exception_from(
      dsl_error("TimeoutError", msg.str(), "dsl", {{"key", std::move(key)}, {"mode", mode}}));
}

void lock_home::start_body(process &p, std::uint64_t hold) { table_.start_body(pid_of(p), hold); }

void lock_home::release_frame(process &p, std::optional<std::uint64_t> hold,
                              bool lock_enter) noexcept {
  try {
    if (!p.locks)
      return;
    process_locks &pl = *p.locks;
    if (hold) {
      const auto it = std::find_if(pl.holds.begin(), pl.holds.end(),
                                   [&](const mhold &h) { return h.id == *hold; });
      if (it == pl.holds.end())
        return; // another process's frame (a restored copy): nothing of this one's
      if (--it->count == 0) {
        const std::uint64_t id = it->id;
        pl.holds.erase(it); // the vector keeps its capacity
        lock_notes notes;
        table_.release(pid_of(p), id, notes);
        deliver(notes);
      }
    }
    if (lock_enter && pl.ticket) {
      // Defensive (protocol 5.4): a lock_enter frame unwound with its ticket still queued.
      const std::uint64_t tid = *pl.ticket;
      lock_notes notes;
      table_.cancel(pid_of(p), tid, notes);
      deliver(notes);
      table_.resume(pid_of(p), tid);
      pl.ticket.reset();
      clear_wait_fields(p);
    }
  } catch (...) {
    // noexcept: a release can't fail on a consistent table; check() reports an inconsistent one.
  }
}

void lock_home::process_ended(process &p, bool abandon) {
  if (!p.locks || (p.locks->holds.empty() && !p.locks->ticket))
    return;
  if (!abandon && !p.locks->holds.empty())
    table_.note_leaked_at_exit();
  lock_notes notes;
  table_.end(pid_of(p), abandon, notes);
  p.locks->holds.clear();
  p.locks->ticket.reset();
  clear_wait_fields(p);
  p.inherited_priority = INT_MAX;
  deliver(notes);
}

void lock_home::wait_expired(std::vector<process *> &expired) {
  // A waiter flagged by a wake_due with no vector is collected here too (design 8.7).
  if (ops_.lock_expired_count() > 0)
    ops_.take_lock_expired(expired);
  for (process *p : expired) {
    if (!p->locks || !p->locks->ticket || !p->lock_waiting || is_finished(p->status))
      continue;
    const std::uint64_t tid = *p->locks->ticket;
    lock_notes notes;
    table_.cancel(pid_of(*p), tid, notes); // cancelled, or granted already: adopted
    deliver(notes);
    apply_wake(p->pid, tid);
  }
  expired.clear();
}

void lock_home::set_base(process &p) {
  if (!p.locks || (p.locks->holds.empty() && !p.locks->ticket))
    return;
  lock_notes notes;
  table_.set_base(pid_of(p), p.priority, notes);
  deliver(notes);
}

void lock_home::deliver(const lock_notes &notes) {
  for (const lock_note &n : notes) {
    if (n.home != id_)
      continue; // another home's: phase L3 posts it to that home's port
    if (n.post.what == lock_post::kind::wake)
      apply_wake(n.post.pid, n.post.tid);
    else
      apply_relevel(n.post.pid);
  }
}

void lock_home::apply_wake(int pid, std::uint64_t tid) {
  process *p = ops_.find_process(pid);
  if (!p || is_finished(p->status) || !p->locks || p->locks->ticket != tid || !p->lock_waiting)
    return; // stale: adopted by a timeout, the process ended, or a newer request (LX-11)
  eval_frame *f = lock_enter_frame(*p);
  const frame_outcome o = collect(*p, tid);
  if (f)
    f->extra_vals[static_cast<std::size_t>(lock_frame_slot::wait_or_outcome)] = outcome_value(o);
  clear_wait_fields(*p);
  deliver_await(p->state, nil_value);
  ops_.ready_lock_waiter(*p);
}

void lock_home::apply_relevel(int pid) {
  process *p = ops_.find_process(pid);
  if (!p)
    return;
  const int v = table_.read_inh(pid_of(*p));
  if (p->inherited_priority != v) {
    p->inherited_priority = v;
    ops_.relevel(*p);
  }
}

// ---------------------------------------------------------------------------------------------
// Reads
// ---------------------------------------------------------------------------------------------

bool lock_home::holds_locks(const process &p) { return p.locks && !p.locks->holds.empty(); }

std::string lock_home::show(std::string_view abs, std::string_view root) {
  if (root.empty())
    return std::string(abs);
  if (abs.size() > root.size() && key_prefix(root, abs))
    return std::string(abs.substr(root.size() + 1));
  std::string out = "/";
  out += abs;
  return out;
}

std::string lock_home::describe_holds(const process &p) {
  std::string out;
  if (!p.locks)
    return out;
  for (const mhold &h : p.locks->holds)
    for (const lock_node *n : h.keys) {
      if (!out.empty())
        out += ", ";
      out += "/";
      out += n->key;
    }
  return out;
}

std::string lock_home::waiting_key(const process &p) {
  if (!p.locks || !p.locks->ticket || p.locks->ticket_keys.empty())
    return std::string();
  return p.locks->ticket_keys[0];
}

// ---------------------------------------------------------------------------------------------
// Checks
// ---------------------------------------------------------------------------------------------

void lock_home::check_quiescent() const {
  const auto fail = [](const char *inv, const std::string &what) {
    throw lock_violation(inv, what);
  };
  ops_.for_each_process([&](process &p) {
    const lock_pid P = pid_of(p);
    const std::string who = "pid " + std::to_string(p.pid);
    const bool has_state = p.locks && (!p.locks->holds.empty() || p.locks->ticket);
    const std::optional<lock_table::rec_view> rec = table_.rec_of(P);
    // I3, home side: a record exactly while the mirror or the ticket is set.
    if (has_state != rec.has_value())
      fail("I3",
           who + (has_state ? " has lock state but no record" : " has a record but no state"));
    if (!p.locks || !p.locks->ticket) {
      if (p.lock_waiting)
        fail("I2", who + " is lock_waiting with no ticket");
    }
    if (!rec)
      return;
    // I8's base clause.
    if (rec->base != p.priority)
      fail("I8", who + " base " + std::to_string(rec->base) + " != priority " +
                     std::to_string(p.priority));
    // I3: the mirror's ids are the record's holds, less a granted ticket's hold.
    std::vector<std::uint64_t> want = rec->holds;
    if (p.locks->ticket) {
      if (rec->ticket != p.locks->ticket)
        fail("I2", who + "'s ticket is not its record's");
      const std::optional<lock_table::ticket_view> t = table_.ticket_of(*p.locks->ticket);
      if (!t)
        fail("I2", who + "'s ticket is gone");
      // At quiescence every wake is applied: a parked process waits on a queued ticket (I6).
      if (!p.lock_waiting || t->state != ticket_state::queued)
        fail("I6", who + " parks on a ticket that is not queued, or is not parked on it");
    } else if (rec->ticket) {
      fail("I2", who + "'s record has a ticket the process doesn't know");
    }
    std::vector<std::uint64_t> mirror;
    for (const mhold &h : p.locks->holds) {
      mirror.push_back(h.id);
      const std::optional<lock_table::hold_view> hv = table_.hold_of(h.id);
      if (!hv || hv->pid != P || hv->mode != h.mode || hv->gen != h.gen)
        fail("I3", who + "'s mirror hold " + std::to_string(h.id) + " is not its hold");
      if (h.count < 1)
        fail("I3", who + "'s mirror hold " + std::to_string(h.id) + " has count 0");
    }
    std::sort(mirror.begin(), mirror.end());
    std::sort(want.begin(), want.end());
    if (mirror != want)
      fail("I3", who + "'s mirror and record disagree");
    // I2's last clause: a lock_enter frame with a granted outcome is a count-1 hold whose body
    // hasn't begun.
    for (const eval_frame &f : p.state.stack) {
      if (f.phase != eval_phase::lock_enter ||
          f.extra_vals.size() <= static_cast<std::size_t>(lock_frame_slot::wait_or_outcome))
        continue;
      const std::optional<frame_outcome> o =
          outcome_of(f.extra_vals[static_cast<std::size_t>(lock_frame_slot::wait_or_outcome)]);
      if (!o || !o->hold)
        continue;
      const auto it = std::find_if(p.locks->holds.begin(), p.locks->holds.end(),
                                   [&](const mhold &h) { return h.id == *o->hold; });
      const std::optional<lock_table::hold_view> hv = table_.hold_of(*o->hold);
      if (it == p.locks->holds.end() || it->count != 1 || !hv || hv->body_started)
        fail("I2", who + "'s lock_enter frame's grant is not a fresh hold");
    }
  });
  // I4, home side: the cache holds exactly this home's references, one per entry.
  std::size_t live = 0;
  std::unordered_map<const lock_node *, std::size_t> per_node;
  for (const entry &e : entries_)
    if (e.live && e.node) {
      ++live;
      ++per_node[e.node];
    }
  if (live != table_.home_cache_refs(id_))
    throw lock_violation("I4", "the key cache holds " + std::to_string(live) +
                                   " entries; the manager counts " +
                                   std::to_string(table_.home_cache_refs(id_)) + " references");
  for (const auto &[node, count] : per_node)
    if (table_.home_cache_refs(id_, node) != count)
      throw lock_violation("I4", "the key cache's references on " + node->key + " disagree");
}

} // namespace cvc::state_exec::detail
