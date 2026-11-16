// A scheduler's lock home: the home actions of docs/STATE_EXEC_LOCKS_PROTOCOL.md section 5.

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <cvc/state/state_exec/detail/lock_home.h>
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/process_table.h>
#include <cvc/state/state_exec/stackless_evaluator.h>
#include <exception>
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
  // A stable sort by absolute key. A request has a few keys: an insertion sort, which allocates
  // nothing (libstdc++'s std::stable_sort takes a temporary buffer even for one key, an
  // allocation on every uncontended section).
  const auto less = [](const home_request::key &a, const home_request::key &b) {
    return key_less(a.text(), b.text());
  };
  if (r.keys.size() > 16) {
    std::stable_sort(r.keys.begin(), r.keys.end(), less);
  } else {
    for (std::size_t i = 1; i < r.keys.size(); ++i)
      for (std::size_t j = i; j > 0 && less(r.keys[j], r.keys[j - 1]); --j)
        std::swap(r.keys[j], r.keys[j - 1]);
  }
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
    return frame_outcome{static_cast<std::uint64_t>(*id), nil_value()};
  if (std::holds_alternative<dict_ptr>(slot.v))
    return frame_outcome{std::nullopt, slot};
  return std::nullopt; // nil, or the wait's handle: not collected yet
}

// ---------------------------------------------------------------------------------------------
// The home
// ---------------------------------------------------------------------------------------------

// ---------------------------------------------------------------------------------------------
// The port (protocol 2.3)
// ---------------------------------------------------------------------------------------------

lock_port::lock_port(std::shared_ptr<wakeup> wake, std::shared_ptr<lock_fuzz> fuzz)
    : wake_(std::move(wake)), fuzz_(std::move(fuzz)) {}
lock_port::~lock_port() = default;

bool lock_port::post(const lock_post &x) {
  lock_fuzz_point(fuzz_.get(), lock_fuzz_at::post_enter);
  bool landed = false;
  {
    std::lock_guard<std::mutex> g(mu_);
    if (!closed_) {
      queue_.push_back(x);
      pending_.store(true, std::memory_order_relaxed);
      if (wake_)
        wake_->notify();
      landed = true;
    }
  }
  lock_fuzz_point(fuzz_.get(), lock_fuzz_at::post_exit);
  return landed;
}

void lock_port::close() {
  std::lock_guard<std::mutex> g(mu_);
  closed_ = true;
}

bool lock_port::closed() const {
  std::lock_guard<std::mutex> g(mu_);
  return closed_;
}

void lock_port::drain(std::vector<lock_post> &out) {
  out.clear();
  std::lock_guard<std::mutex> g(mu_);
  out.swap(queue_);
  pending_.store(false, std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------------------------
// CVC_EXEC_FUZZ (L3.7; LT-4, gap G-27)
// ---------------------------------------------------------------------------------------------

namespace {

std::uint64_t fuzz_mix(std::uint64_t x) { // splitmix64's finalizer
  x += 0x9e3779b97f4a7c15ull;
  x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ull;
  x = (x ^ (x >> 27)) * 0x94d049bb133111ebull;
  return x ^ (x >> 31);
}

} // namespace

std::uint64_t lock_fuzz_seed_from_environment() {
  const char *v = std::getenv("CVC_EXEC_FUZZ");
  if (!v || !*v)
    return 0;
  char *end = nullptr;
  const unsigned long long n = std::strtoull(v, &end, 0);
  if (end && *end == '\0')
    return n;
  std::uint64_t h = 1469598103934665603ull; // FNV-1a
  for (const char *c = v; *c; ++c)
    h = (h ^ static_cast<unsigned char>(*c)) * 1099511628211ull;
  return h ? h : 1;
}

namespace {

// The fuzzed point itself, out of the off path: run on, yield, or pause 0-50 us. The pause yields
// until it has passed, rather than sleeping: a sleep rounds up to the OS tick (up to ~16 ms on
// Windows), far longer than the windows LT-4 means to widen. Each point draws the stream's next
// number (a splitmix64 counter), mixed with the seed and the point.
void fuzz_yield(lock_fuzz &fuzz, std::uint64_t seed, lock_fuzz_at where) noexcept {
  fuzz.hits.fetch_add(1, std::memory_order_relaxed);
  const std::uint64_t n = fuzz.stream.fetch_add(1, std::memory_order_relaxed);
  const std::uint64_t x = fuzz_mix(seed ^ fuzz_mix(n + static_cast<std::uint64_t>(where)));
  switch (x & 7) {
  case 0:
  case 1:
    return; // run on
  case 7: {
    const auto until = lock_clock::now() + std::chrono::microseconds((x >> 8) % 51);
    do
      std::this_thread::yield();
    while (lock_clock::now() < until);
    return;
  }
  default:
    std::this_thread::yield();
  }
}

} // namespace

void lock_fuzz_point(lock_fuzz *fuzz, lock_fuzz_at where) noexcept {
  if (!fuzz)
    return;
  const std::uint64_t seed = fuzz->seed.load(std::memory_order_relaxed);
  if (seed != 0)
    fuzz_yield(*fuzz, seed, where);
}

// ---------------------------------------------------------------------------------------------
// The home
// ---------------------------------------------------------------------------------------------

std::size_t lock_home::cache_hash::operator()(const cache_key &k) const noexcept {
  return std::hash<std::string_view>{}(k.written) ^ (std::size_t(k.scope) * 0x9E3779B97F4A7C15ull);
}

#ifdef NDEBUG
constexpr bool kHomeToken = false;
#else
constexpr bool kHomeToken = true;
#endif

// The home token (protocol 0, design 9.1): only one thread runs a home's actions at a time, its
// pumping thread, or for a pycvc Exec with no run() active whichever thread holds its control
// mutex. Taken at an action's entry and released at its exit; the holder may re-enter (an action
// delivers, the delivery applies a post, the post calls the table and delivers again). Release
// builds compile it to nothing.
class lock_home::action {
public:
  explicit action(const lock_home &h) noexcept : h_(h) {
    if constexpr (kHomeToken) {
      const std::thread::id me = std::this_thread::get_id();
      std::thread::id none{};
      const bool mine = h_.token_.compare_exchange_strong(none, me) || none == me;
      assert(mine && "lock_home: two threads run one home's actions at once");
      (void)mine;
      ++h_.token_depth_;
    }
  }
  ~action() {
    if constexpr (kHomeToken)
      if (--h_.token_depth_ == 0)
        h_.token_.store(std::thread::id{});
  }
  action(const action &) = delete;
  action &operator=(const action &) = delete;

private:
  const lock_home &h_;
};

lock_home::lock_home(std::shared_ptr<lock_manager> mgr, lock_sched_ops &ops,
                     scheduling_policy policy, std::shared_ptr<wakeup> wake, lock_attach attach)
    : mgr_(std::move(mgr)), table_(mgr_->table()), ops_(ops),
      prio_(policy != scheduling_policy::round_robin),
      port_(std::make_shared<lock_port>(std::move(wake), mgr_->fuzz())),
      progress_(std::make_shared<std::atomic<std::int64_t>>(
          lock_clock::now().time_since_epoch().count())) {
  const lock_test_hooks *hooks = mgr_->test_hooks();
  const std::size_t capacity =
      hooks && hooks->home_cache_capacity ? hooks->home_cache_capacity : kDefaultCacheCapacity;
  entries_.resize(capacity);
  free_.reserve(capacity);
  for (std::size_t i = capacity; i-- > 0;)
    free_.push_back(i);
  index_.reserve(capacity);
  lock_notes notes;
  id_ = table_.attach_home(prio_, port_, progress_, notes, attach);
  deliver_op(notes); // none: an attach changes no edge and no base
}

void lock_home::drain_port() {
  action a(*this);
  port_->drain(drained_);
  // Every post taken is applied even when one throws (as deliver does), then the first error is
  // rethrown.
  std::exception_ptr first;
  for (std::size_t i = 0; i < drained_.size(); ++i) {
    if (i > 0)
      lock_fuzz_point(mgr_->fuzz().get(), lock_fuzz_at::drain_between);
    try {
      apply_post(drained_[i]);
    } catch (...) {
      if (!first)
        first = std::current_exception();
    }
  }
  drained_.clear();
  if (first)
    std::rethrow_exception(first);
}

std::vector<std::pair<lock_pid, std::int64_t>> lock_home::blocking_sinks(int pid) const {
  return table_.blocking_sinks(lock_pid{id_, pid});
}

std::vector<std::pair<lock_pid, std::int64_t>>
lock_home::blocking_sinks(std::span<const int> pids) const {
  small_vector<lock_pid, 8> from;
  for (int pid : pids)
    from.push_back(lock_pid{id_, pid});
  return table_.blocking_sinks(std::span<const lock_pid>(from.data(), from.size()));
}

lock_home::~lock_home() {
  // 5.7, in its order. Nothing here throws out: a destructor can't report a failure, and the
  // table's own check() finds whatever a failed step leaves.
  action a(*this);
  try {
    // The port closes first, under its own mutex: a post racing it lands before (and is
    // discarded below) or is dropped (2.3). detach_home's precondition, asserted outside M.mu.
    port_->close();
    assert(port_->closed());
  } catch (...) {
  }
  lock_fuzz_point(mgr_->fuzz().get(), lock_fuzz_at::close_then_detach);
  try {
    // Every process of the home ended with abandon, the cache's references dropped. No note
    // targets this home; the others' wakes and relevels go to their ports.
    lock_notes notes;
    table_.detach_home(id_, notes);
    deliver_op(notes);
  } catch (...) {
  }
  try {
    // The posts that landed before the close: their processes have just ended.
    port_->drain(drained_);
    if (!drained_.empty())
      mgr_->posts_discarded_.fetch_add(drained_.size(), std::memory_order_relaxed);
    drained_.clear();
  } catch (...) {
  }
  port_.reset();
  try {
    ops_.for_each_process([](process &p) {
      if (p.locks) {
        p.locks->holds.clear();
        p.locks->ticket.reset();
      }
      clear_wait_fields(p);
      p.inherited_priority = INT_MAX;
    });
  } catch (...) {
  }
}

lock_pid lock_home::pid_of(const process &p) const { return {id_, p.pid}; }

rec_info lock_home::info_of(const process &p) { return rec_info{p.owner, p.priority}; }

process_locks &lock_home::locks_of(process &p) {
  if (!p.locks)
    p.locks = std::make_shared<process_locks>();
  return *p.locks;
}

void lock_home::begin_request() {
  action a(*this);
  ++pin_epoch_;
  misses_.clear();
}

std::uint32_t lock_home::scope_id(std::string_view root_path) {
  action a(*this);
  if (last_scope_valid_ && root_path == last_root_)
    return last_scope_;
  const auto it = scopes_.find(root_path);
  std::uint32_t id;
  if (it != scopes_.end()) {
    id = it->second;
  } else {
    id = static_cast<std::uint32_t>(scopes_.size());
    scopes_.emplace(std::string(root_path), id);
  }
  last_root_.assign(root_path);
  last_scope_ = id;
  last_scope_valid_ = true;
  return id;
}

std::size_t lock_home::memo_index(std::uint32_t scope, std::string_view written) noexcept {
  const auto a = reinterpret_cast<std::uintptr_t>(written.data());
  return static_cast<std::size_t>(((a >> 3) ^ (a >> 11) ^ (std::uintptr_t(scope) * 0x9E3779B1u)) &
                                  (kLookupMemo - 1));
}

lock_node *lock_home::lookup(std::uint32_t scope, std::string_view written) {
  action a(*this);
  const std::size_t m = memo_index(scope, written);
  std::size_t slot = lookup_memo_[m];
  if (slot >= entries_.size() || !entries_[slot].live || entries_[slot].scope != scope ||
      entries_[slot].written != written) {
    const auto it = index_.find(cache_key{scope, written});
    if (it == index_.end())
      return nullptr;
    slot = it->second;
    lookup_memo_[m] = slot;
  }
  entry &e = entries_[slot];
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

std::optional<std::size_t> lock_home::victim(const home_request &req) {
  // CLOCK: a used entry gets a second chance; an entry of this request (pinned, or reserved for
  // one of its misses) is never taken, nor one whose node is a key of the request under another
  // spelling ("x" and "x " both name x). Twice round the clock finds one if any can go.
  const auto names_a_key = [&req](const lock_node *node) {
    for (const home_request::key &k : req.keys)
      if (k.node ? k.node == node : node->key == k.absolute)
        return true;
    return false;
  };
  const std::size_t n = entries_.size();
  for (std::size_t tries = 0; tries < 2 * n; ++tries) {
    const std::size_t slot = hand_;
    hand_ = (hand_ + 1) % n;
    entry &e = entries_[slot];
    if (!e.live || !e.node || e.pinned == pin_epoch_ || names_a_key(e.node))
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
      } else if ((slot = victim(req))) {
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
  action a(*this);
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
  // The after_grant test hook runs in the evaluator, once the lock frame records this grant
  // (stackless_evaluator.cpp lock_acquire): an error it injects then fails a step whose unwind
  // gives the grant back (LX-8). Called here, it fired before any frame held the grant, and the
  // hold outlived the failed step.
  return {enter_result::kind::granted, g->hold, g->gen, g->abandoned};
}

std::optional<frame_outcome> lock_home::wait(process &p, home_request &req,
                                             std::optional<double> timeout, std::string_view root) {
  action a(*this);
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
  deliver_op(notes); // relevels only: the processes P now blocks, downstream
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
    if (const lock_test_hooks *hooks = mgr_->test_hooks(); hooks && hooks->after_grant)
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
      cycle.push_back(
          make_dict({{"pid", e.pid ? value_t(static_cast<int64_t>(*e.pid)) : nil_value()},
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

void lock_home::start_body(process &p, std::uint64_t hold) {
  action a(*this);
  table_.start_body(pid_of(p), hold);
}

void lock_home::release_frame(process &p, std::optional<std::uint64_t> hold,
                              bool lock_enter) noexcept {
  action a(*this);
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
        deliver_op(notes);
      }
    }
    if (lock_enter && pl.ticket) {
      // Defensive (protocol 5.4): a lock_enter frame unwound with its ticket still set, as when
      // the step that queued it throws. Another home may have granted the ticket meanwhile: the
      // cancel then finds it granted, and that grant, which never reached the mirror, is given
      // back as any lock_enter frame's grant is. Each operation's notes go right after it.
      const std::uint64_t tid = *pl.ticket;
      lock_notes notes;
      table_.cancel(pid_of(p), tid, notes);
      try {
        deliver_op(notes); // the cancel's own, now: they don't wait on the resume or a release
      } catch (...) {
        // delivered in full all the same (deliver); the resume must still happen
      }
      const lock_outcome out = table_.resume(pid_of(p), tid);
      pl.ticket.reset();
      clear_wait_fields(p);
      if (out.what == lock_outcome::kind::granted) {
        lock_notes freed;
        table_.release(pid_of(p), out.g.hold, freed);
        deliver_op(freed);
      }
    }
  } catch (...) {
    // noexcept: a release can't fail on a consistent table; check() reports an inconsistent one.
  }
}

void lock_home::process_ended(process &p, bool abandon) {
  action a(*this);
  // A process that never reached the manager has nothing there. One that did is ended there
  // whatever its mirror says (an end that finds no record changes nothing), so a record the
  // mirror missed can't outlive the process: a grant whose collect failed after the resume.
  if (!p.locks)
    return;
  if (!abandon && !p.locks->holds.empty())
    table_.note_leaked_at_exit();
  lock_notes notes;
  table_.end(pid_of(p), abandon, notes);
  p.locks->holds.clear();
  p.locks->ticket.reset();
  clear_wait_fields(p);
  p.inherited_priority = INT_MAX;
  deliver_op(notes);
}

void lock_home::wait_expired(std::vector<process *> &expired) {
  action a(*this);
  // A waiter flagged by a wake_due with no vector is collected here too (design 8.7).
  if (ops_.lock_expired_count() > 0)
    ops_.take_lock_expired(expired);
  for (process *p : expired) {
    if (!p->locks || !p->locks->ticket || !p->lock_waiting || is_finished(p->status))
      continue;
    const std::uint64_t tid = *p->locks->ticket;
    lock_notes notes;
    table_.cancel(pid_of(*p), tid, notes); // cancelled, or granted already: adopted
    deliver_op(notes);
    apply_wake(p->pid, tid);
  }
  expired.clear();
}

void lock_home::set_base(process &p) {
  action a(*this);
  if (!p.locks || (p.locks->holds.empty() && !p.locks->ticket))
    return;
  lock_notes notes;
  table_.set_base(pid_of(p), p.priority, notes);
  deliver_op(notes);
}

void lock_home::deliver(const lock_notes &notes) {
  action a(*this);
  // Every note is delivered even when one throws (a scheduler's callback, an allocation): the
  // note after it may be a granted waiter's only wake. The first error is rethrown after.
  std::exception_ptr first;
  for (const lock_note &n : notes) {
    try {
      if (n.home == id_) {
        apply_post(n.post); // this home's: at once (4.14)
      } else if (n.port) {
        // Another home's: to its port, or dropped when the port is closed (2.3). The counters
        // are the manager's, bumped outside M.mu.
        if (const lock_test_hooks *hooks = mgr_->test_hooks(); hooks && hooks->cross_home_note)
          hooks->cross_home_note(n);
        std::atomic<std::uint64_t> &count =
            n.port->post(n.post) ? mgr_->posts_ : mgr_->posts_dropped_;
        count.fetch_add(1, std::memory_order_relaxed);
      }
    } catch (...) {
      if (!first)
        first = std::current_exception();
    }
  }
  if (first)
    std::rethrow_exception(first);
}

void lock_home::deliver_op(const lock_notes &notes) {
  if (notes.empty())
    return;
  // Another home may act between the two (protocol 7).
  lock_fuzz_point(mgr_->fuzz().get(), lock_fuzz_at::after_op);
  deliver(notes);
}

void lock_home::apply_post(const lock_post &post) {
  action a(*this);
  if (post.what == lock_post::kind::wake)
    apply_wake(post.pid, post.tid);
  else
    apply_relevel(post.pid);
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
  deliver_await(p->state, nil_value());
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
  action a(*this);
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
    // I8's quiescent clause: on a home that applies priorities a process carries read_inh's value
    // (read here without its side effect on the coalescing flag); on any other home a relevel is
    // never sent, so it carries none
    const int inh = prio_ && rec ? rec->inh : lock_inf;
    if (p.inherited_priority != inh)
      fail("I8", who + " inherited_priority " +
                     (p.inherited_priority == lock_inf ? std::string("INF")
                                                       : std::to_string(p.inherited_priority)) +
                     " != " + (inh == lock_inf ? std::string("INF") : std::to_string(inh)) +
                     (prio_ ? ", the manager's" : " on a home that applies no priorities"));
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
