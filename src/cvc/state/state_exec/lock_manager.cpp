// The state_exec lock manager (docs/STATE_EXEC_LOCKS_PROTOCOL.md sections 2-4 and 6).
//
// lock_table implements the manager operations of protocol section 4, one member function each,
// over the structures of 2.4: a path trie whose nodes count the holds and queued tickets of their
// subtree, records, holds and tickets in slot pools found through open-addressing indexes, and the
// queue as an intrusive list in seq order, each queued ticket also linked into the `wq` of each of
// its keys. Phases L1 and L2: single mode (one thread calls the table, no mutex) and one home per
// manager.
//
// Inheritance (3.3, 4.3; phase L2) follows the implementation note of 4.3: while no attached home
// applies priorities nothing is computed; otherwise each operation recomputes the inherited
// priority of the processes within 8 edges downstream of its seeds, in the wait-for graph before
// and after it, each by a reverse breadth-first search over predecessor edges found through the
// trie. check() compares every record with its own from-scratch search over edges recomputed from
// the definitions (I8), never with the code relevel_all uses.
//
// No trie node is freed inside an operation: a node that may have emptied is queued and checked
// when the operation ends (end_op), so the regions an operation re-examines, which are made of
// the nodes of the holds and tickets it removed, stay valid until then.

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <cvc/state/state_exec/detail/lock_home.h>
#include <cvc/state/state_exec/detail/lock_table.h>
#include <cvc/state/state_exec/lock_manager.h>
#include <deque>
#include <set>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace cvc::state_exec {

namespace detail {

namespace {
std::atomic<const lock_test_hooks *> g_lock_test_hooks{nullptr};

constexpr std::uint64_t seq_inf = UINT64_MAX; // INF as a seq: a request not yet queued
constexpr int mode_s = int(lock_mode::shared);
constexpr int mode_x = int(lock_mode::exclusive);

bool compat(lock_mode a, lock_mode b) { return a == lock_mode::shared && b == lock_mode::shared; }

/// Ascending (home, pid), pids signed: protocol 4.6's order.
bool pid_less(lock_pid a, lock_pid b) { return a.home != b.home ? a.home < b.home : a.pid < b.pid; }

bool key_overlap(std::string_view a, std::string_view b) {
  return key_prefix(a, b) || key_prefix(b, a);
}

bool node_overlap(const lock_node *a, const lock_node *b) {
  return node_prefix(a, b) || node_prefix(b, a);
}

/// conflicts(A, B) of protocol 1 over trie nodes.
bool conflicts(lock_mode am, std::span<lock_node *const> ak, lock_mode bm,
               std::span<lock_node *const> bk) {
  if (compat(am, bm))
    return false;
  for (const lock_node *a : ak)
    for (const lock_node *b : bk)
      if (node_overlap(a, b))
        return true;
  return false;
}

/// conflicts(A, B) with A's keys as text (a request whose keys may have no node).
bool conflicts_text(lock_mode am, std::span<const std::string_view> ak, lock_mode bm,
                    std::span<lock_node *const> bk) {
  if (compat(am, bm))
    return false;
  for (std::string_view a : ak)
    for (const lock_node *b : bk)
      if (key_overlap(a, b->key))
        return true;
  return false;
}

/// A span over a small_vector (whose iterators need not model contiguous_iterator).
template <class V> std::span<typename V::value_type const> view(const V &v) {
  return {v.data(), v.size()};
}

std::string_view segment_of(const lock_node *n) {
  if (n->depth <= 1)
    return n->key;
  return std::string_view(n->key).substr(n->key.rfind('.') + 1);
}

std::uint64_t mix64(std::uint64_t x) {
  x += 0x9e3779b97f4a7c15ull;
  x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ull;
  x = (x ^ (x >> 27)) * 0x94d049bb133111ebull;
  return x ^ (x >> 31);
}

[[noreturn]] void violation(const char *clause, const std::string &detail) {
  throw lock_violation(clause, detail);
}

std::string pid_text(lock_pid p) {
  return "(" + std::to_string(p.home) + "," + std::to_string(p.pid) + ")";
}

const char *mode_text(lock_mode m) { return m == lock_mode::exclusive ? "X" : "S"; }

const char *state_text(ticket_state s) {
  switch (s) {
  case ticket_state::queued:
    return "queued";
  case ticket_state::granted:
    return "granted";
  case ticket_state::cancelled:
    return "cancelled";
  case ticket_state::failed:
    return "failed";
  }
  return "?";
}

/// Debug builds: the single-mode owner-thread token (design 8.3), asserted free on entry to
/// every operation and released on exit.
class owner_scope {
public:
  explicit owner_scope(std::atomic<std::thread::id> &token) : token_(token) {
#ifndef NDEBUG
    std::thread::id none{};
    const bool was_free = token_.compare_exchange_strong(none, std::this_thread::get_id());
    assert(was_free && "lock_table: two threads in a single-mode lock manager at once");
    (void)was_free;
#endif
  }
  ~owner_scope() {
#ifndef NDEBUG
    token_.store(std::thread::id{});
#endif
  }
  owner_scope(const owner_scope &) = delete;
  owner_scope &operator=(const owner_scope &) = delete;

private:
  [[maybe_unused]] std::atomic<std::thread::id> &token_;
};
} // namespace

void set_lock_test_hooks(const lock_test_hooks *hooks) {
  g_lock_test_hooks.store(hooks, std::memory_order_release);
}
const lock_test_hooks *lock_test_hooks_get() {
  return g_lock_test_hooks.load(std::memory_order_acquire);
}

// -- 2.1's Hold and Ticket ----------------------------------------------------------------------

struct lock_hold {
  std::uint64_t id = 0;
  lock_pid pid;
  small_vector<lock_node *, 4> keys; // canonical order
  lock_mode mode = lock_mode::exclusive;
  std::uint64_t gen = 0;
  bool body_started = false;
  small_vector<lock_node *, 2> taken; // canonical order; a subset of keys
};

enum class ticket_error : std::uint8_t { none, timeout, deadlock };

struct lock_ticket {
  std::uint64_t seq = 0; // = its TicketId
  lock_pid pid;
  lock_mode mode = lock_mode::exclusive;
  ticket_state state = ticket_state::queued;
  ticket_error error = ticket_error::none; // set when failed
  std::uint64_t hold = 0;                  // set when granted; 0: none
  small_vector<lock_node *, 4> nodes;      // the keys' nodes while queued or granted
  std::vector<std::string> text;           // the keys (first nkeys), storage reused
  std::size_t nkeys = 0;
  std::vector<deadlock_edge> cycle; // the DeadlockError's
  lock_ticket *prev = nullptr;      // M.queue, in seq order
  lock_ticket *next = nullptr;
  std::uint64_t mark = 0; // re_examine's de-duplication
};

// -- the table ------------------------------------------------------------------------------------

struct lock_table::impl {
  /// Objects with stable addresses, recycled through a free list (2.4, design 8.8).
  template <class T> class pool {
  public:
    T *acquire() {
      if (!free_.empty()) {
        T *t = free_.back();
        free_.pop_back();
        return t;
      }
      store_.emplace_back();
      return &store_.back();
    }
    void release(T *t) { free_.push_back(t); }

  private:
    std::deque<T> store_;
    std::vector<T *> free_;
  };

  /// An open-addressing map from a 64-bit id to a non-null pointer: linear probing, deletion by
  /// backward shift, so neither an insert nor an erase allocates once the table has grown.
  template <class T> class index {
  public:
    T *find(std::uint64_t k) const {
      if (slots_.empty())
        return nullptr;
      const std::size_t mask = slots_.size() - 1;
      for (std::size_t i = mix64(k) & mask;; i = (i + 1) & mask) {
        if (!slots_[i].val)
          return nullptr;
        if (slots_[i].key == k)
          return slots_[i].val;
      }
    }
    void insert(std::uint64_t k, T *v) {
      if ((size_ + 1) * 2 > slots_.size())
        grow();
      place(k, v);
      ++size_;
    }
    bool erase(std::uint64_t k) {
      if (slots_.empty())
        return false;
      const std::size_t mask = slots_.size() - 1;
      std::size_t i = mix64(k) & mask;
      for (;; i = (i + 1) & mask) {
        if (!slots_[i].val)
          return false;
        if (slots_[i].key == k)
          break;
      }
      for (std::size_t j = i;;) {
        j = (j + 1) & mask;
        if (!slots_[j].val)
          break;
        const std::size_t h = mix64(slots_[j].key) & mask;
        if (((i - h) & mask) < ((j - h) & mask)) { // i lies on j's probe path: shift it back
          slots_[i] = slots_[j];
          i = j;
        }
      }
      slots_[i] = slot{};
      --size_;
      return true;
    }
    std::size_t size() const { return size_; }
    template <class F> void for_each(F &&f) const {
      for (const slot &s : slots_)
        if (s.val)
          f(s.key, s.val);
    }

  private:
    struct slot {
      std::uint64_t key = 0;
      T *val = nullptr;
    };
    void place(std::uint64_t k, T *v) {
      const std::size_t mask = slots_.size() - 1;
      std::size_t i = mix64(k) & mask;
      while (slots_[i].val)
        i = (i + 1) & mask;
      slots_[i] = slot{k, v};
    }
    void grow() {
      std::vector<slot> old = std::move(slots_);
      slots_.assign(old.empty() ? 16 : old.size() * 2, slot{});
      for (const slot &s : old)
        if (s.val)
          place(s.key, s.val);
    }
    std::vector<slot> slots_;
    std::size_t size_ = 0;
  };

  /// 2.1's Rec.
  struct rec {
    lock_pid pid;
    std::string owner; // copied only when the record is made (gap G-43); capacity reused
    int base = 0;
    small_vector<std::uint64_t, 4> holds; // ascending
    std::uint64_t ticket = 0;             // 0: none
    int inh = lock_inf;
    bool relevel_pending = false; // 4.3's coalescing flag (phase L2)
  };

  /// 2.1's Home, and the home's M.cache_refs entry.
  struct home_entry {
    bool attached = false;
    bool prio = false;
    std::shared_ptr<lock_port> port;
    std::shared_ptr<std::atomic<std::int64_t>> progress;
    bool has_refs = false; // M.cache_refs has an entry for the home
    std::unordered_map<lock_node *, std::uint32_t> refs;
  };

  /// A request key's place in the trie: its node (`exact`), or the deepest existing ancestor.
  struct kpos {
    lock_node *node = nullptr;
    bool exact = false;
  };
  using kposes = small_vector<kpos, 4>;
  using region = small_vector<lock_node *, 8>;
  using pid_list = small_vector<lock_pid, 4>;
  using pred_map = std::unordered_map<std::uint64_t, std::vector<lock_pid>>; // by packed pid

  explicit impl(const lock_table::options &o)
      : opts(o), divergence_pending((o.mutations & lock_table::plant_divergence) != 0) {
    root.live = true;
  }

  bool mut(std::uint32_t bit) const { return (opts.mutations & bit) != 0; }

  // -- state ------------------------------------------------------------------------------------
  lock_table::options opts;
  std::atomic<std::thread::id> owner{}; // debug builds: the single-mode token (design 8.3)

  std::vector<home_entry> homes; // by home_id
  std::size_t attached = 0;

  pool<rec> rec_pool;
  index<rec> recs; // by lock_pid::packed()
  pool<lock_hold> hold_pool;
  index<lock_hold> holds; // by HoldId
  pool<lock_ticket> ticket_pool;
  index<lock_ticket> tickets; // by TicketId
  lock_ticket *q_head = nullptr, *q_tail = nullptr;
  std::size_t queued = 0;

  std::uint64_t next_seq = 0, next_hold = 0, next_gen = 0;

  lock_node root;
  std::deque<lock_node> node_store;
  std::vector<lock_node *> node_free;
  std::size_t trie_nodes = 1;
  std::vector<lock_node *> pending_free;
  std::size_t flagged = 0;    // nodes with the abandoned flag: |M.abandoned|
  std::size_t refs_total = 0; // every home's references

  // lock_stats
  std::uint64_t n_grants = 0, n_fast = 0, n_waits = 0, n_timeouts = 0, n_deadlocks = 0;
  std::uint64_t n_abandoned = 0, n_ends = 0, n_relevels = 0, n_coalesced = 0;
  bool divergence_pending = false; // plant_divergence, until a release grants
  bool in_release = false;
  bool in_cancel = false; // cancel_skips_relevel's test

  // ghosts (self_check's grant-time assertions, I11's conservation)
  std::uint64_t g_last_x_gen = 0, g_max_gen = 0;
  std::set<std::string, std::less<>> g_pending_report; // abandonment reports not yet seen

  // per operation
  lock_notes *out = nullptr;
  bool lending_before = false;
  bool queued_before = false; // a ticket was queued when the operation started: the graph had edges
  std::vector<lock_ticket *> scratch;
  std::uint64_t mark_epoch = 0;
  // 4.3's implementation note, while lending: the seeds the operation knows when it starts (the
  // process whose request, holds or base it changes; the processes it ends), their inh then (INF
  // without a record), and every process within 8 edges downstream of them in the graph before
  pid_list op_seeds;
  small_vector<std::pair<lock_pid, int>, 4> op_old;
  std::vector<lock_pid> op_before;
  // relevel_all's scratch: the predecessors found through the trie, per process, this operation;
  // the set A; the processes whose inh changed; a breadth-first search's layers
  std::unordered_map<std::uint64_t, std::vector<lock_pid>> pred_memo;
  std::vector<lock_pid> relevel_set, relevel_changed, bfs_layer, bfs_next;
  std::unordered_set<std::uint64_t> bfs_seen;

  // -- lookups ----------------------------------------------------------------------------------
  rec *rec_of(lock_pid p) const { return recs.find(p.packed()); }
  lock_hold *hold_of(std::uint64_t id) const { return id ? holds.find(id) : nullptr; }
  lock_ticket *ticket_of(std::uint64_t id) const { return id ? tickets.find(id) : nullptr; }

  bool applies(lock_pid p) const {
    return p.home < homes.size() && homes[p.home].attached && homes[p.home].prio;
  }
  bool lends(lock_pid p) const { return opts.inheritance && applies(p); }
  bool lending() const {
    if (!opts.inheritance)
      return false;
    for (const home_entry &h : homes)
      if (h.attached && h.prio)
        return true;
    return false;
  }

  // -- the trie (2.4) ---------------------------------------------------------------------------
  kpos locate(std::string_view key) {
    lock_node *n = &root;
    if (key.empty())
      return {n, true};
    for (std::size_t pos = 0;;) {
      const std::size_t dot = key.find('.', pos);
      const std::string_view seg =
          key.substr(pos, dot == std::string_view::npos ? std::string_view::npos : dot - pos);
      const auto it = n->children.find(seg);
      if (it == n->children.end())
        return {n, false};
      n = it->second;
      if (dot == std::string_view::npos)
        return {n, true};
      pos = dot + 1;
    }
  }
  kpos locate(const key_ref &k) { return k.node ? kpos{k.node, true} : locate(k.key); }

  lock_node *new_node(lock_node *parent, std::string_view seg) {
    lock_node *n;
    if (!node_free.empty()) {
      n = node_free.back();
      node_free.pop_back();
    } else {
      node_store.emplace_back();
      n = &node_store.back();
    }
    n->key.clear();
    if (parent != &root) {
      n->key.append(parent->key);
      n->key.push_back('.');
    }
    n->key.append(seg);
    n->parent = parent;
    n->depth = parent->depth + 1;
    n->live = true;
    n->free_pending = false;
    parent->children.emplace(std::string(seg), n);
    ++trie_nodes;
    return n;
  }

  /// The node of `key`, made with its missing ancestors if absent.
  lock_node *node_for(std::string_view key) {
    lock_node *n = &root;
    if (key.empty())
      return n;
    for (std::size_t pos = 0; pos <= key.size();) {
      const std::size_t dot = key.find('.', pos);
      const std::string_view seg =
          key.substr(pos, dot == std::string_view::npos ? std::string_view::npos : dot - pos);
      const auto it = n->children.find(seg);
      n = it != n->children.end() ? it->second : new_node(n, seg);
      if (dot == std::string_view::npos)
        break;
      pos = dot + 1;
    }
    return n;
  }
  lock_node *node_for(const key_ref &k) { return k.node ? k.node : node_for(k.key); }

  static bool node_empty(const lock_node *n) {
    return n->here[0].empty() && n->here[1].empty() && n->sub[0] == 0 && n->sub[1] == 0 &&
           n->wq.empty() && n->subw == 0 && !n->abandoned && n->refs == 0;
  }

  void maybe_free(lock_node *n) {
    if (n != &root && !n->free_pending) {
      n->free_pending = true;
      pending_free.push_back(n);
    }
  }

  /// 2.4: free every queued node that is empty and childless, repeating up the path.
  void sweep() {
    for (lock_node *n : pending_free) {
      n->free_pending = false;
      while (n->live && n != &root && n->children.empty() && node_empty(n)) {
        lock_node *parent = n->parent;
        parent->children.erase(parent->children.find(segment_of(n)));
        n->parent = nullptr;
        n->depth = 0;
        n->live = false;
        node_free.push_back(n);
        --trie_nodes;
        n = parent;
      }
    }
    pending_free.clear();
  }

  void trie_add_hold(lock_hold *h) {
    const int m = int(h->mode);
    for (lock_node *n : h->keys) {
      n->here[m].push_back(h);
      for (lock_node *a = n; a; a = a->parent)
        ++a->sub[m];
    }
  }
  void trie_remove_hold(lock_hold *h) {
    const int m = int(h->mode);
    for (lock_node *n : h->keys) {
      auto &v = n->here[m];
      const auto it = std::find(v.begin(), v.end(), h);
      *it = v.back();
      v.pop_back();
      for (lock_node *a = n; a; a = a->parent)
        --a->sub[m];
      maybe_free(n);
    }
  }

  void set_flag(lock_node *n) {
    if (!n->abandoned) {
      n->abandoned = true;
      ++flagged;
    }
  }
  void clear_flag(lock_node *n) {
    if (n->abandoned) {
      n->abandoned = false;
      --flagged;
      maybe_free(n);
    }
  }

  // -- 2.4's conflict enumeration -----------------------------------------------------------------
  /// Visits every hold with a key that overlaps the key at `kp` and a mode that conflicts with
  /// `mode`: the here lists of the root-to-key path, then the subtree, entered only where `sub`
  /// counts a conflicting mode. Stops, returning false, when `f` returns false.
  template <class F> bool visit_holds(const kpos &kp, lock_mode mode, F &&f) {
    const bool x_req = mode == lock_mode::exclusive;
    for (lock_node *a = kp.node; a; a = a->parent) {
      if (x_req)
        for (lock_hold *h : a->here[mode_s])
          if (!f(h))
            return false;
      for (lock_hold *h : a->here[mode_x])
        if (!f(h))
          return false;
    }
    if (!kp.exact)
      return true;
    auto below = [x_req](const lock_node *n) { // conflicting holds strictly below n
      return n->sub[mode_x] != n->here[mode_x].size() ||
             (x_req && n->sub[mode_s] != n->here[mode_s].size());
    };
    auto counts = [x_req](const lock_node *n) {
      return n->sub[mode_x] != 0 || (x_req && n->sub[mode_s] != 0);
    };
    if (!below(kp.node))
      return true;
    small_vector<lock_node *, 16> stack;
    for (const auto &c : kp.node->children)
      if (counts(c.second))
        stack.push_back(c.second);
    while (!stack.empty()) {
      lock_node *n = stack.back();
      stack.pop_back();
      if (x_req)
        for (lock_hold *h : n->here[mode_s])
          if (!f(h))
            return false;
      for (lock_hold *h : n->here[mode_x])
        if (!f(h))
          return false;
      if (below(n))
        for (const auto &c : n->children)
          if (counts(c.second))
            stack.push_back(c.second);
    }
    return true;
  }

  /// Visits every queued ticket with a key that overlaps the key at `kp` (any mode), through
  /// `wq` on the path and `subw` below it.
  template <class F> bool visit_waiters(const kpos &kp, F &&f) {
    for (lock_node *a = kp.node; a; a = a->parent)
      for (lock_ticket *t : a->wq)
        if (!f(t))
          return false;
    if (!kp.exact || kp.node->subw == kp.node->wq.size())
      return true;
    small_vector<lock_node *, 16> stack;
    for (const auto &c : kp.node->children)
      if (c.second->subw != 0)
        stack.push_back(c.second);
    while (!stack.empty()) {
      lock_node *n = stack.back();
      stack.pop_back();
      for (lock_ticket *t : n->wq)
        if (!f(t))
          return false;
      if (n->subw != n->wq.size())
        for (const auto &c : n->children)
          if (c.second->subw != 0)
            stack.push_back(c.second);
    }
    return true;
  }

  // -- 3.1 ----------------------------------------------------------------------------------------
  /// exempt(P, U): U conflicts with a hold P already has.
  bool exempt(lock_pid p, const lock_ticket *u) const {
    if (mut(lock_table::no_exemption))
      return false;
    const rec *r = rec_of(p);
    if (!r)
      return false;
    for (std::uint64_t id : r->holds) {
      const lock_hold *h = hold_of(id);
      if (conflicts(u->mode, view(u->nodes), h->mode, view(h->keys)))
        return true;
    }
    return false;
  }

  bool waits_on(lock_pid p, lock_mode mode, std::uint64_t s, const lock_ticket *u) const {
    return u->seq < s && u->pid != p && !compat(mode, u->mode) && !exempt(p, u);
  }

  /// Calls on_hold for each member of blk_hold(P, R) and on_wait for each of blk_wait(P, R, s);
  /// a member may come more than once (one per key of R it overlaps).
  template <class FH, class FW>
  void blockers(lock_pid p, lock_mode mode, const kposes &keys, std::uint64_t s, FH &&on_hold,
                FW &&on_wait) {
    for (const kpos &kp : keys)
      visit_holds(kp, mode, [&](lock_hold *h) {
        if (h->pid != p)
          on_hold(h);
        return true;
      });
    for (const kpos &kp : keys)
      visit_waiters(kp, [&](lock_ticket *u) {
        if (waits_on(p, mode, s, u))
          on_wait(u);
        return true;
      });
  }

  /// grantable(P, R, s), stopping at the first blocker.
  bool grantable_fast(lock_pid p, lock_mode mode, const kposes &keys, std::uint64_t s) {
    if (root.sub[0] == 0 && root.sub[1] == 0 && root.subw == 0)
      return true; // nothing held, nothing queued
    for (const kpos &kp : keys)
      if (!visit_holds(kp, mode, [&](lock_hold *h) { return h->pid == p; }))
        return false;
    if (root.subw == 0)
      return true;
    for (const kpos &kp : keys)
      if (!visit_waiters(kp, [&](lock_ticket *u) { return !waits_on(p, mode, s, u); }))
        return false;
    return true;
  }

  /// grantable(P, R, s); under self_check the trie's enumeration is compared with the
  /// definition (I4, as the model's cross-check does).
  bool grantable(lock_pid p, lock_mode mode, const kposes &keys,
                 std::span<const std::string_view> text, std::uint64_t s) {
    const bool ok = grantable_fast(p, mode, keys, s);
    if (opts.self_check)
      cross_check(p, mode, keys, text, s, ok);
    return ok;
  }

  void cross_check(lock_pid p, lock_mode mode, const kposes &keys,
                   std::span<const std::string_view> text, std::uint64_t s, bool ok) {
    std::set<std::uint64_t> th, tw, dh, dw;
    blockers(
        p, mode, keys, s, [&](lock_hold *h) { th.insert(h->id); },
        [&](lock_ticket *u) { tw.insert(u->seq); });
    holds.for_each([&](std::uint64_t, lock_hold *h) {
      if (h->pid != p && conflicts_text(mode, text, h->mode, view(h->keys)))
        dh.insert(h->id);
    });
    for (const lock_ticket *u = q_head; u; u = u->next)
      if (u->seq < s && u->pid != p && conflicts_text(mode, text, u->mode, view(u->nodes)) &&
          !exempt_def(p, u))
        dw.insert(u->seq);
    if (th != dh)
      violation("I4", "the trie's hold enumeration differs from blk_hold for a request of " +
                          pid_text(p));
    if (tw != dw)
      violation("I4", "the trie's waiter enumeration differs from blk_wait for a request of " +
                          pid_text(p));
    if (ok != (dh.empty() && dw.empty()))
      violation("I4", "grantable's early exit differs from the definition for " + pid_text(p));
  }

  /// exempt(P, U) from the definition: over every hold.
  bool exempt_def(lock_pid p, const lock_ticket *u) const {
    if (mut(lock_table::no_exemption))
      return false;
    bool found = false;
    holds.for_each([&](std::uint64_t, lock_hold *h) {
      if (h->pid == p && conflicts(u->mode, view(u->nodes), h->mode, view(h->keys)))
        found = true;
    });
    return found;
  }

  /// grantable(T) from the definition (check()'s I6).
  bool grantable_def(const lock_ticket *t) const {
    bool ok = true;
    holds.for_each([&](std::uint64_t, lock_hold *h) {
      if (h->pid != t->pid && conflicts(t->mode, view(t->nodes), h->mode, view(h->keys)))
        ok = false;
    });
    for (const lock_ticket *u = q_head; ok && u; u = u->next)
      if (u->seq < t->seq && u->pid != t->pid &&
          conflicts(t->mode, view(t->nodes), u->mode, view(u->nodes)) && !exempt_def(t->pid, u))
        ok = false;
    return ok;
  }

  // -- 3.2 ----------------------------------------------------------------------------------------
  static void sort_unique(std::vector<lock_pid> &v) {
    std::sort(v.begin(), v.end(), pid_less);
    v.erase(std::unique(v.begin(), v.end()), v.end());
  }

  /// succ(A), ascending.
  std::vector<lock_pid> successors(lock_pid a) {
    std::vector<lock_pid> out;
    const rec *r = rec_of(a);
    lock_ticket *t = r ? ticket_of(r->ticket) : nullptr;
    if (!t || t->state != ticket_state::queued)
      return out;
    kposes kp;
    for (lock_node *n : t->nodes)
      kp.push_back({n, true});
    blockers(
        a, t->mode, kp, t->seq, [&](lock_hold *h) { out.push_back(h->pid); },
        [&](lock_ticket *u) { out.push_back(u->pid); });
    sort_unique(out);
    return out;
  }

  /// succ(A) from the definition (check()'s I7).
  std::vector<lock_pid> successors_def(const lock_ticket *t) const {
    std::vector<lock_pid> out;
    holds.for_each([&](std::uint64_t, lock_hold *h) {
      if (h->pid != t->pid && conflicts(t->mode, view(t->nodes), h->mode, view(h->keys)))
        out.push_back(h->pid);
    });
    for (const lock_ticket *u = q_head; u; u = u->next)
      if (u->seq < t->seq && u->pid != t->pid &&
          conflicts(t->mode, view(t->nodes), u->mode, view(u->nodes)) && !exempt_def(t->pid, u))
        out.push_back(u->pid);
    sort_unique(out);
    return out;
  }

  /// 4.6's deadlock_cycle: a depth-first search from the targets in ascending order, each
  /// successor list ascending, stopping at the first path back to P. Empty when P is not
  /// reachable from the targets (no cycle).
  std::vector<deadlock_edge> deadlock_cycle(lock_pid p, const lock_request &req,
                                            const kposes &pkeys,
                                            const std::vector<lock_pid> &targets) {
    struct frame {
      lock_pid a;
      std::vector<lock_pid> succ;
      std::size_t i = 0;
    };
    std::unordered_set<std::uint64_t> visited;
    std::vector<frame> stack;
    std::vector<lock_pid> path;
    bool found = false;
    for (lock_pid t : targets) {
      if (!visited.insert(t.packed()).second)
        continue;
      stack.push_back({t, successors(t)});
      while (!stack.empty() && !found) {
        frame &f = stack.back();
        if (f.i == f.succ.size()) {
          stack.pop_back();
          continue;
        }
        const lock_pid b = f.succ[f.i++];
        if (b == p) {
          for (const frame &g : stack)
            path.push_back(g.a);
          found = true;
        } else if (visited.insert(b.packed()).second) {
          stack.push_back({b, successors(b)}); // invalidates f
        }
      }
      if (found)
        break;
    }
    std::vector<deadlock_edge> cycle;
    if (!found)
      return cycle;
    std::vector<lock_pid> nodes{p};
    nodes.insert(nodes.end(), path.begin(), path.end());
    const rec *pr = rec_of(p);
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      const lock_pid a = nodes[i], b = nodes[(i + 1) % nodes.size()];
      const rec *ar = rec_of(a);
      deadlock_edge e;
      if (a.home == p.home && ar->owner == pr->owner)
        e.pid = a.pid;
      e.key = edge_key(a, b, p, req, pkeys);
      cycle.push_back(std::move(e));
    }
    return cycle;
  }

  /// The first key, in canonical order, of A's request that overlaps the hold or ticket of B
  /// that makes the edge A -> B (4.6).
  std::string edge_key(lock_pid a, lock_pid b, lock_pid p, const lock_request &req,
                       const kposes &pkeys) {
    lock_mode mode;
    kposes kp;
    std::vector<std::string_view> text;
    std::uint64_t s;
    if (a == p) {
      mode = req.mode;
      kp = pkeys;
      for (const key_ref &k : req.keys)
        text.push_back(k.text());
      s = seq_inf;
    } else {
      const lock_ticket *t = ticket_of(rec_of(a)->ticket);
      mode = t->mode;
      for (lock_node *n : t->nodes) {
        kp.push_back({n, true});
        text.push_back(n->key);
      }
      s = t->seq;
    }
    std::vector<const lock_node *> bk;
    blockers(
        a, mode, kp, s,
        [&](lock_hold *h) {
          if (h->pid == b)
            bk.insert(bk.end(), h->keys.begin(), h->keys.end());
        },
        [&](lock_ticket *u) {
          if (u->pid == b)
            bk.insert(bk.end(), u->nodes.begin(), u->nodes.end());
        });
    for (std::string_view k : text)
      for (const lock_node *n : bk)
        if (key_overlap(k, n->key))
          return std::string(k);
    violation("I7",
              "a deadlock cycle's edge " + pid_text(a) + " -> " + pid_text(b) + " has no key");
  }

  // -- 4.1 ----------------------------------------------------------------------------------------
  rec *ensure_rec(lock_pid p, const rec_info &info) {
    if (rec *r = rec_of(p))
      return r;
    rec *r = rec_pool.acquire();
    r->pid = p;
    r->owner.assign(info.owner.data(), info.owner.size());
    r->base = info.base;
    r->holds.clear();
    r->ticket = 0;
    r->inh = lock_inf;
    r->relevel_pending = false;
    recs.insert(p.packed(), r);
    return r;
  }

  void maybe_drop_rec(lock_pid p) {
    rec *r = rec_of(p);
    if (r && r->holds.empty() && !r->ticket) {
      recs.erase(p.packed());
      rec_pool.release(r);
    }
  }

  std::uint64_t new_hold(lock_pid p, lock_mode mode, std::span<lock_node *const> keys, bool started,
                         std::uint64_t s) {
    if (opts.self_check)
      grant_checks(p, mode, keys, s);
    lock_hold *h = hold_pool.acquire();
    h->id = ++next_hold;
    h->pid = p;
    h->keys.assign(keys.begin(), keys.end());
    h->mode = mode;
    h->body_started = started;
    h->taken.clear();
    trie_add_hold(h); // first, so taking a flag never leaves its node empty
    if (mode == lock_mode::exclusive) {
      h->gen = ++next_gen;
      for (lock_node *n : h->keys)
        if (n->abandoned) {
          h->taken.push_back(n);
          clear_flag(n);
        }
    } else {
      h->gen = next_gen;
    }
    // I15 at the grant (gap G-14)
    if (mode == lock_mode::exclusive) {
      if (opts.self_check && h->gen <= g_max_gen)
        violation("I15", "X generation " + std::to_string(h->gen) +
                             " is not above an earlier one, " + std::to_string(g_max_gen));
      g_last_x_gen = h->gen;
    } else if (opts.self_check && h->gen != g_last_x_gen) {
      violation("I15", "S generation " + std::to_string(h->gen) + " is not the last X one, " +
                           std::to_string(g_last_x_gen));
    }
    g_max_gen = std::max(g_max_gen, h->gen);
    holds.insert(h->id, h);
    rec_of(p)->holds.push_back(h->id);
    if (in_release && divergence_pending)
      divergence_pending = false; // plant_divergence: this grant goes uncounted
    else
      ++n_grants;
    if (started)
      body_start(h);
    return h->id;
  }

  /// I1 and I5 at a grant, from the definitions (gap G-14; the model's new_hold).
  void grant_checks(lock_pid p, lock_mode mode, std::span<lock_node *const> keys, std::uint64_t s) {
    holds.for_each([&](std::uint64_t, lock_hold *h) {
      if (h->pid != p && conflicts(mode, keys, h->mode, view(h->keys)))
        violation("I1", "a hold for " + pid_text(p) + " conflicts with hold " +
                            std::to_string(h->id) + " of " + pid_text(h->pid));
    });
    for (const lock_ticket *u = q_head; u; u = u->next)
      if (u->seq < s && u->pid != p && conflicts(mode, keys, u->mode, view(u->nodes)) &&
          !exempt_def(p, u))
        violation("I5", "a hold for " + pid_text(p) + " granted ahead of the earlier waiter " +
                            std::to_string(u->seq) + " of " + pid_text(u->pid));
  }

  /// I11's ghost at a body's start: each report taken is seen once.
  void body_start(const lock_hold *h) {
    for (const lock_node *n : h->taken) {
      const auto it = g_pending_report.find(n->key);
      if (it != g_pending_report.end())
        g_pending_report.erase(it);
      else if (opts.self_check)
        violation("I11", "the abandoned report for " + n->key + " is seen twice");
    }
    if (opts.self_check && h->mode == lock_mode::exclusive)
      for (const lock_node *n : h->keys)
        if (n->abandoned)
          violation("I11", "an X body on " + n->key + " starts while its flag is set");
  }

  /// drop_hold(id, abandon): the hold's keys are appended to `reg`.
  void drop_hold(lock_hold *h, bool abandon, region &reg) {
    if (!h->body_started) {
      for (lock_node *n : h->taken)
        set_flag(n); // a report its holder never saw goes back
    } else if (abandon && h->mode == lock_mode::exclusive) {
      for (lock_node *n : h->keys) { // the section may be half written
        if (!n->abandoned)
          ++n_abandoned;
        set_flag(n);
        g_pending_report.insert(n->key);
      }
    }
    rec *r = rec_of(h->pid);
    r->holds.erase(std::find(r->holds.begin(), r->holds.end(), h->id));
    trie_remove_hold(h);
    reg.insert(reg.end(), h->keys.begin(), h->keys.end());
    holds.erase(h->id);
    h->keys.clear();
    h->taken.clear();
    hold_pool.release(h);
  }

  void enqueue(lock_ticket *t) {
    t->prev = q_tail;
    t->next = nullptr;
    (q_tail ? q_tail->next : q_head) = t;
    q_tail = t;
    ++queued;
    for (lock_node *n : t->nodes) {
      n->wq.push_back(t);
      for (lock_node *a = n; a; a = a->parent)
        ++a->subw;
    }
  }

  void dequeue(lock_ticket *t) {
    (t->prev ? t->prev->next : q_head) = t->next;
    (t->next ? t->next->prev : q_tail) = t->prev;
    t->prev = t->next = nullptr;
    --queued;
    for (lock_node *n : t->nodes) {
      auto &v = n->wq;
      const auto it = std::find(v.begin(), v.end(), t);
      *it = v.back();
      v.pop_back();
      for (lock_node *a = n; a; a = a->parent)
        --a->subw;
      maybe_free(n);
    }
  }

  lock_ticket *new_ticket(lock_pid p, std::uint64_t s, const lock_request &req) {
    lock_ticket *t = ticket_pool.acquire();
    t->seq = s;
    t->pid = p;
    t->mode = req.mode;
    t->state = ticket_state::queued;
    t->error = ticket_error::none;
    t->hold = 0;
    t->nodes.clear();
    t->cycle.clear();
    t->prev = t->next = nullptr;
    t->mark = 0;
    if (t->text.size() < req.keys.size())
      t->text.resize(req.keys.size());
    t->nkeys = req.keys.size();
    for (std::size_t i = 0; i < req.keys.size(); ++i) {
      const std::string_view k = req.keys[i].text();
      t->text[i].assign(k.data(), k.size());
    }
    tickets.insert(s, t);
    return t;
  }

  void delete_ticket(lock_ticket *t) {
    tickets.erase(t->seq);
    t->nodes.clear();
    t->cycle.clear();
    ticket_pool.release(t);
  }

  /// 4.11's end_locked(P, abandon): the region is appended to `reg`; false when P has no record.
  bool end_locked(lock_pid p, bool abandon, region &reg) {
    rec *r = rec_of(p);
    if (!r)
      return false;
    ++n_ends;
    if (r->ticket) {
      lock_ticket *t = ticket_of(r->ticket);
      if (t->state == ticket_state::queued) {
        reg.insert(reg.end(), t->nodes.begin(), t->nodes.end());
        dequeue(t);
      }
      // granted: its hold is in r->holds and is dropped below, unstarted, so clean
      delete_ticket(t);
      r->ticket = 0;
    }
    const small_vector<std::uint64_t, 4> ids(r->holds.begin(), r->holds.end());
    for (std::uint64_t id : ids)
      drop_hold(hold_of(id), abandon, reg);
    recs.erase(p.packed());
    rec_pool.release(r);
    return true;
  }

  // -- 4.15 -------------------------------------------------------------------------------------
  home_entry &refs_entry(home_id h, const char *what) {
    if (h >= homes.size() || !homes[h].attached || !homes[h].has_refs)
      violation("4.15", std::string(what) + ": home " + std::to_string(h) +
                            " is not attached or has no key-cache entry");
    return homes[h];
  }

  lock_node *ref(home_id h, const key_ref &k) {
    home_entry &e = refs_entry(h, "ref");
    lock_node *n = node_for(k);
    ++e.refs[n];
    ++n->refs;
    ++refs_total;
    return n;
  }

  void unref(home_id h, lock_node *n) {
    home_entry *e = h < homes.size() && homes[h].has_refs ? &homes[h] : nullptr;
    if (!e || e->refs.find(n) == e->refs.end())
      violation("4.15", "unref: no reference of home " + std::to_string(h) + " on " +
                            (n ? n->key : std::string("<null>")));
    const auto it = e->refs.find(n);
    if (--it->second == 0)
      e->refs.erase(it);
    if (e->refs.empty() && mut(lock_table::unref_pops_entry))
      e->has_refs = false; // the rev-4 behaviour; I9 catches it
    --n->refs;
    --refs_total;
    maybe_free(n);
  }

  void apply_refs(home_id h, lock_request &req, const cache_delta &refs) {
    for (key_ref &k : req.keys)
      if (k.ref)
        k.node = ref(h, k);
    for (lock_node *n : refs.drop) {
      if (opts.self_check)
        for (const key_ref &k : req.keys)
          if (k.node == n)
            violation("4.15", "refs.drop names a key of the request: '" + n->key + "'");
      unref(h, n);
    }
  }

  // -- 4.2, 4.3 ---------------------------------------------------------------------------------
  void re_examine(const region &reg, pid_list &granted) {
    if (root.subw == 0)
      return; // nothing queued: the uncontended release
    ++mark_epoch;
    scratch.clear();
    for (lock_node *n : reg)
      visit_waiters(kpos{n, true}, [&](lock_ticket *t) {
        if (t->mark != mark_epoch) {
          t->mark = mark_epoch;
          scratch.push_back(t);
        }
        return true;
      });
    if (scratch.empty())
      return;
    std::sort(scratch.begin(), scratch.end(),
              [](const lock_ticket *a, const lock_ticket *b) { return a->seq < b->seq; });
    for (lock_ticket *t : scratch) {
      if (t->state != ticket_state::queued)
        continue;
      kposes kp;
      small_vector<std::string_view, 4> text;
      for (lock_node *n : t->nodes) {
        kp.push_back({n, true});
        text.push_back(n->key);
      }
      if (!grantable(t->pid, t->mode, kp, view(text), t->seq))
        continue;
      dequeue(t);
      t->hold = new_hold(t->pid, t->mode, view(t->nodes), false, t->seq);
      t->state = ticket_state::granted;
      out->push_back(lock_note{t->pid.home, homes[t->pid.home].port,
                               lock_post{lock_post::kind::wake, t->pid.pid, t->seq}});
      granted.push_back(t->pid);
    }
  }

  void finish(const region &reg, std::span<const lock_pid> seeds, std::span<const lock_pid> skip) {
    pid_list granted;
    re_examine(reg, granted);
    for (lock_pid p : seeds)
      maybe_drop_rec(p);
    for (lock_pid p : granted)
      maybe_drop_rec(p);
    relevel_all(skip, granted);
  }

  // -- 3.3, 4.3: inheritance ----------------------------------------------------------------------
  /// The processes with an edge to `a` (3.2), found through the trie: the queued tickets of other
  /// processes that conflict with a hold of `a`, and the later queued tickets of other processes
  /// that conflict with a's queued ticket and are not exempt from it. Memoized per operation.
  const std::vector<lock_pid> &predecessors(lock_pid a) {
    const auto [it, fresh] = pred_memo.try_emplace(a.packed());
    std::vector<lock_pid> &into = it->second;
    if (!fresh)
      return into;
    const rec *r = rec_of(a);
    if (!r)
      return into;
    for (std::uint64_t id : r->holds) {
      const lock_hold *h = hold_of(id);
      for (lock_node *n : h->keys)
        visit_waiters(kpos{n, true}, [&](lock_ticket *u) { // u overlaps the key
          if (u->pid != a && !compat(u->mode, h->mode))
            into.push_back(u->pid);
          return true;
        });
    }
    const lock_ticket *t = ticket_of(r->ticket);
    if (t && t->state == ticket_state::queued)
      for (lock_node *n : t->nodes)
        visit_waiters(kpos{n, true}, [&](lock_ticket *u) {
          if (u->seq > t->seq && u->pid != a && !compat(u->mode, t->mode) && !exempt(u->pid, t))
            into.push_back(u->pid);
          return true;
        });
    sort_unique(into);
    return into;
  }

  /// Appends to `into` every process within 8 edges downstream of `from` (succ, 3.2), and `from`.
  void reach8(std::span<const lock_pid> from, std::vector<lock_pid> &into) {
    if (!q_head) { // no queued ticket, no edge
      into.insert(into.end(), from.begin(), from.end());
      return;
    }
    bfs_seen.clear();
    bfs_layer.clear();
    for (lock_pid p : from)
      if (bfs_seen.insert(p.packed()).second) {
        bfs_layer.push_back(p);
        into.push_back(p);
      }
    for (int d = 0; d < 8 && !bfs_layer.empty(); ++d) {
      bfs_next.clear();
      for (lock_pid a : bfs_layer)
        for (lock_pid b : successors(a))
          if (bfs_seen.insert(b.packed()).second) {
            bfs_next.push_back(b);
            into.push_back(b);
          }
      bfs_layer.swap(bfs_next);
    }
  }

  /// inh_formula(X) for relevel_all (3.3): a reverse breadth-first search from X of depth 8 over
  /// predecessor edges, so anc8(X) as a set, taking min of base over the lenders it finds. Never a
  /// recursion over effective or inherited values: a process that lends nothing still passes on
  /// what it inherits, and the cap bounds paths.
  int recompute_inh(lock_pid x) {
    bfs_seen.clear();
    bfs_seen.insert(x.packed());
    bfs_layer.assign(1, x);
    int best = lock_inf;
    for (int d = 0; d < 8 && !bfs_layer.empty(); ++d) {
      bfs_next.clear();
      for (lock_pid a : bfs_layer)
        for (lock_pid y : predecessors(a)) {
          if (!bfs_seen.insert(y.packed()).second)
            continue;
          bfs_next.push_back(y);
          if (lends(y))
            best = std::min(best, rec_of(y)->base);
        }
      bfs_layer.swap(bfs_next);
    }
    return best;
  }

  /// 4.3's relevel_all, by its implementation note. `lending` before and after the operation
  /// decides the case: false and false, nothing (every inh is INF, I8); true then false, every inh
  /// INF and no post (no attached home applies priorities); false then true, the reference loop
  /// over every record; true and true, only the set A. A record whose inh changes, of a process
  /// not in `skip` whose home applies priorities, gets a relevel note, after the operation's
  /// wakes and in (home, pid) order; under options.coalesce_relevels at most one is pending per
  /// record (relevel_pending, cleared by read_inh), the others counted as coalesced.
  void relevel_all(std::span<const lock_pid> skip, std::span<const lock_pid> granted) {
    if (in_cancel && mut(lock_table::cancel_skips_relevel))
      return; // the mutation: a timeout leaves every inherited priority as it was
    const bool lending_after = lending();
    if (!lending_before && !lending_after)
      return;
    if (!lending_after) {
      recs.for_each([](std::uint64_t, rec *r) { r->inh = lock_inf; });
      return;
    }
    if (!queued_before && !q_head)
      return; // no edge before or after: every inh was INF (I8) and stays so (the uncontended path)
    pred_memo.clear();
    relevel_changed.clear();
    const auto update = [&](lock_pid x, int old) {
      rec *r = rec_of(x);
      const int now = r ? recompute_inh(x) : lock_inf; // no record any more: INF, as in the loop
      if (r)
        r->inh = now;
      if (now != old)
        relevel_changed.push_back(x);
    };
    if (!lending_before) {
      relevel_set.clear();
      recs.for_each([&](std::uint64_t, rec *r) { relevel_set.push_back(r->pid); });
      for (lock_pid x : relevel_set)
        update(x, lock_inf); // lending was false: every inh was INF (I8)
    } else {
      set_a(granted);
      for (lock_pid x : relevel_set) {
        // a seed's record may be gone or new: its inh when the operation started; any other
        // process of A kept its record through the operation, inh not yet rewritten
        std::optional<int> old;
        for (const auto &[p, v] : op_old)
          if (p == x)
            old = v;
        if (!old)
          old = rec_of(x) ? rec_of(x)->inh : lock_inf;
        update(x, *old);
      }
      if (opts.self_check)
        check_impl_note();
    }
    sort_unique(relevel_changed);
    for (lock_pid x : relevel_changed) {
      if (std::find(skip.begin(), skip.end(), x) != skip.end() || !applies(x))
        continue;
      rec *r = rec_of(x);
      if (opts.coalesce_relevels && r) {
        if (r->relevel_pending) {
          ++n_coalesced; // one is pending already: its apply reads the current value (5.9)
          continue;
        }
        r->relevel_pending = true;
      }
      out->push_back(
          lock_note{x.home, homes[x.home].port, lock_post{lock_post::kind::relevel, x.pid, 0}});
      ++n_relevels;
    }
  }

  /// 4.3's set A into relevel_set, sorted: the seeds and the processes granted, and every process
  /// within 8 edges downstream of them in the graph before the operation (op_before) and after
  /// it. A process granted here had, before, edges only to the seeds (its blockers were what the
  /// operation removed: a grant in the same pass would have blocked it), so its share of the
  /// graph before is the seeds'. Under impl_note_after_only, the graph after only.
  void set_a(std::span<const lock_pid> granted) {
    relevel_set.assign(op_before.begin(), op_before.end());
    pid_list from(op_seeds.begin(), op_seeds.end());
    from.insert(from.end(), granted.begin(), granted.end());
    reach8(view(from), relevel_set);
    sort_unique(relevel_set);
  }

  /// self_check: protocol 4.3's implementation note itself. After the operation no record outside
  /// A may differ from inh_formula computed from scratch (inh_def, check()'s own search), i.e.
  /// the reference loop changes nothing outside A (lockmodel.py's 'impl-note').
  void check_impl_note() const {
    const pred_map pred = preds_def();
    recs.for_each([&](std::uint64_t, rec *r) {
      if (std::binary_search(relevel_set.begin(), relevel_set.end(), r->pid, pid_less))
        return;
      const int want = inh_def(r->pid, pred, true);
      if (r->inh != want) {
        std::string seeds;
        for (lock_pid p : op_seeds)
          seeds += (seeds.empty() ? "" : " ") + pid_text(p);
        violation("impl-note", "the inherited priority of " + pid_text(r->pid) + " is now " +
                                   std::to_string(want) + ", not " + std::to_string(r->inh) +
                                   ", but it is outside the set A of seeds [" + seeds + "]");
      }
    });
  }

  // -- operation framing --------------------------------------------------------------------------
  /// `seeds`: those of 4.3's implementation note that the operation knows when it starts (none
  /// for an operation that changes no edge and no base).
  void begin_op(lock_notes *notes, std::span<const lock_pid> seeds = {}) {
    out = notes;
    in_release = false;
    in_cancel = false;
    lending_before = lending();
    queued_before = q_head != nullptr;
    op_seeds.assign(seeds.begin(), seeds.end());
    op_old.clear();
    op_before.clear();
    if (!lending_before || seeds.empty())
      return;
    for (lock_pid p : seeds) {
      const rec *r = rec_of(p);
      op_old.emplace_back(p, r ? r->inh : lock_inf);
    }
    if (!mut(lock_table::impl_note_after_only))
      reach8(seeds, op_before);
  }
  void end_op() {
    sweep();
    out = nullptr;
    if (opts.self_check)
      check();
  }

  /// Protocol 1's canonical request (self_check): non-empty, valid keys, sorted by their segment
  /// tuples, none a prefix of another; a cached key's node live.
  void check_request(const lock_request &req) const {
    if (req.keys.empty())
      violation("1", "a request with no key");
    std::string_view prev;
    for (std::size_t i = 0; i < req.keys.size(); ++i) {
      const key_ref &k = req.keys[i];
      if (k.node && !k.node->live)
        violation("1", "a request key's node is not in the trie");
      const std::string_view t = k.text();
      if (t.empty() || t.front() == '.' || t.back() == '.' ||
          t.find("..") != std::string_view::npos || t.find('#') != std::string_view::npos)
        violation("1", "the request key \"" + std::string(t) + "\" is not canonical");
      if (i > 0 && (!key_less(prev, t) || key_prefix(prev, t)))
        violation("1", "the request keys \"" + std::string(prev) + "\" and \"" + std::string(t) +
                           "\" are out of order or nested");
      prev = t;
    }
  }

  // -- reads
  // ----------------------------------------------------------------------------------------
  std::vector<const lock_node *> all_nodes() const {
    std::vector<const lock_node *> v;
    std::vector<const lock_node *> stack{&root};
    while (!stack.empty()) {
      const lock_node *n = stack.back();
      stack.pop_back();
      v.push_back(n);
      for (const auto &c : n->children)
        stack.push_back(c.second);
    }
    return v;
  }

  std::vector<const rec *> sorted_recs() const {
    std::vector<const rec *> v;
    recs.for_each([&](std::uint64_t, rec *r) { v.push_back(r); });
    std::sort(v.begin(), v.end(),
              [](const rec *a, const rec *b) { return pid_less(a->pid, b->pid); });
    return v;
  }
  std::vector<const lock_hold *> sorted_holds() const {
    std::vector<const lock_hold *> v;
    holds.for_each([&](std::uint64_t, lock_hold *h) { v.push_back(h); });
    std::sort(v.begin(), v.end(),
              [](const lock_hold *a, const lock_hold *b) { return a->id < b->id; });
    return v;
  }
  std::vector<const lock_ticket *> sorted_tickets() const {
    std::vector<const lock_ticket *> v;
    tickets.for_each([&](std::uint64_t, lock_ticket *t) { v.push_back(t); });
    std::sort(v.begin(), v.end(),
              [](const lock_ticket *a, const lock_ticket *b) { return a->seq < b->seq; });
    return v;
  }

  lock_query_result query(std::string_view prefix, std::optional<home_id> viewer_home,
                          std::string_view viewer_owner) {
    auto visible = [&](lock_pid q) {
      if (!viewer_home || q.home != *viewer_home)
        return false;
      const rec *r = rec_of(q);
      return r && r->owner == viewer_owner;
    };
    lock_query_result res;
    const kpos start = locate(prefix);
    if (start.exact && (start.node->sub[0] != 0 || start.node->sub[1] != 0)) {
      std::vector<const lock_node *> keys;
      std::vector<const lock_node *> stack{start.node};
      while (!stack.empty()) {
        const lock_node *n = stack.back();
        stack.pop_back();
        if (!n->here[0].empty() || !n->here[1].empty())
          keys.push_back(n);
        for (const auto &c : n->children)
          if (c.second->sub[0] != 0 || c.second->sub[1] != 0)
            stack.push_back(c.second);
      }
      std::sort(keys.begin(), keys.end(),
                [](const lock_node *a, const lock_node *b) { return key_less(a->key, b->key); });
      for (const lock_node *n : keys) {
        std::vector<const lock_hold *> hs(n->here[0].begin(), n->here[0].end());
        hs.insert(hs.end(), n->here[1].begin(), n->here[1].end());
        std::sort(hs.begin(), hs.end(),
                  [](const lock_hold *a, const lock_hold *b) { return a->id < b->id; });
        lock_info info;
        info.key = n->key;
        info.mode = n->here[mode_x].empty() ? lock_mode::shared : lock_mode::exclusive;
        for (const lock_hold *h : hs)
          if (visible(h->pid))
            info.pids.push_back(h->pid.pid);
        info.count = hs.size();
        for (const lock_hold *h : n->here[mode_x]) {
          const bool better =
              !info.generation || (mut(lock_table::query_first_gen) ? h->gen < *info.generation
                                                                    : h->gen > *info.generation);
          if (better)
            info.generation = h->gen;
        }
        info.abandoned = n->abandoned;
        res.held.push_back(std::move(info));
      }
    }
    for (const lock_ticket *t = q_head; t; t = t->next) {
      bool under = false;
      for (const lock_node *n : t->nodes)
        under = under || key_prefix(prefix, n->key);
      if (!under)
        continue;
      lock_waiter_info w;
      for (const lock_node *n : t->nodes)
        w.keys.push_back(n->key);
      w.mode = t->mode;
      if (visible(t->pid))
        w.pid = t->pid.pid;
      res.waiting.push_back(std::move(w));
    }
    return res;
  }

  std::string canonical() const {
    auto join_keys = [](auto begin, auto end) {
      std::vector<std::string_view> v;
      for (auto it = begin; it != end; ++it)
        v.push_back(*it);
      std::sort(v.begin(), v.end(), key_less);
      std::string s;
      for (std::string_view k : v) {
        if (!s.empty())
          s += ',';
        s += k;
      }
      return s;
    };
    auto node_keys = [&](std::span<lock_node *const> nodes) {
      std::vector<std::string_view> v;
      for (const lock_node *n : nodes)
        v.push_back(n->key);
      return join_keys(v.begin(), v.end());
    };
    std::string s = "homes\n";
    for (std::size_t h = 0; h < homes.size(); ++h)
      if (homes[h].attached)
        s += "H" + std::to_string(h) + " prio=" + (homes[h].prio ? "1" : "0") + "\n";
    s += "recs\n";
    for (const rec *r : sorted_recs()) {
      s += pid_text(r->pid) + " owner=\"" + r->owner + "\" base=" + std::to_string(r->base) +
           " holds=[";
      for (std::size_t i = 0; i < r->holds.size(); ++i)
        s += (i ? "," : "") + std::to_string(r->holds[i]);
      s += "] ticket=" + (r->ticket ? std::to_string(r->ticket) : std::string("-")) +
           " inh=" + (r->inh == lock_inf ? std::string("INF") : std::to_string(r->inh)) + "\n";
    }
    s += "holds\n";
    for (const lock_hold *h : sorted_holds())
      s += std::to_string(h->id) + " pid=" + pid_text(h->pid) + " keys=[" +
           node_keys(view(h->keys)) + "] mode=" + mode_text(h->mode) +
           " gen=" + std::to_string(h->gen) + " started=" + (h->body_started ? "1" : "0") +
           " taken=[" + node_keys(view(h->taken)) + "]\n";
    s += "tickets\n";
    for (const lock_ticket *t : sorted_tickets()) {
      s += std::to_string(t->seq) + " pid=" + pid_text(t->pid) + " keys=[" +
           join_keys(t->text.begin(), t->text.begin() + t->nkeys) + "] mode=" + mode_text(t->mode) +
           " state=" + state_text(t->state) +
           " hold=" + (t->hold ? std::to_string(t->hold) : std::string("-")) + " error=";
      if (t->error == ticket_error::none) {
        s += "-";
      } else if (t->error == ticket_error::timeout) {
        s += "TimeoutError";
      } else {
        s += "DeadlockError[";
        for (std::size_t i = 0; i < t->cycle.size(); ++i)
          s += std::string(i ? "," : "") + "(" +
               (t->cycle[i].pid ? std::to_string(*t->cycle[i].pid) : std::string("nil")) + ":" +
               t->cycle[i].key + ")";
        s += "]";
      }
      s += "\n";
    }
    s += "queue=[";
    for (const lock_ticket *t = q_head; t; t = t->next)
      s += (t == q_head ? "" : ",") + std::to_string(t->seq);
    s += "]\n";
    std::vector<std::string_view> flags;
    for (const lock_node *n : all_nodes())
      if (n->abandoned)
        flags.push_back(n->key);
    s += "abandoned=[" + join_keys(flags.begin(), flags.end()) + "]\n";
    s += "next seq=" + std::to_string(next_seq) + " hold=" + std::to_string(next_hold) +
         " gen=" + std::to_string(next_gen) + "\n";
    s += "cache_refs\n";
    for (std::size_t h = 0; h < homes.size(); ++h) {
      if (!homes[h].has_refs)
        continue;
      std::vector<std::pair<std::string_view, std::uint32_t>> v;
      for (const auto &kv : homes[h].refs)
        v.emplace_back(kv.first->key, kv.second);
      std::sort(v.begin(), v.end(),
                [](const auto &a, const auto &b) { return key_less(a.first, b.first); });
      s += "H" + std::to_string(h) + " {";
      for (std::size_t i = 0; i < v.size(); ++i)
        s +=
            std::string(i ? "," : "") + std::string(v[i].first) + ":" + std::to_string(v[i].second);
      s += "}\n";
    }
    s += "counters grants=" + std::to_string(n_grants) + "\n";
    return s;
  }

  // -- section 6 ----------------------------------------------------------------------------------
  void check() const;
  void check_trie() const;

  /// I8's own predecessor edges: succ of every queued ticket from the definitions of 3.1
  /// (successors_def, over every hold and the queue, not the trie), inverted.
  pred_map preds_def() const {
    pred_map pred;
    for (const lock_ticket *t = q_head; t; t = t->next)
      for (lock_pid b : successors_def(t))
        pred[b.packed()].push_back(t->pid);
    return pred;
  }

  /// inh_formula(X) from scratch (3.3), for check() and the implementation note's self-check: a
  /// breadth-first search backwards from X layered by distance and cut at 8 edges, which finds
  /// anc8(X) as a set (in a graph without cycles, I7, exactly the processes with some path of 1 to
  /// 8 edges to X) in O(8E), then min of base over its lenders. Shares no code with relevel_all.
  int inh_def(lock_pid x, const pred_map &pred, bool lend) const {
    if (!lend)
      return lock_inf;
    std::unordered_set<std::uint64_t> seen{x.packed()};
    std::vector<lock_pid> layer{x}, next;
    int best = lock_inf;
    for (int dist = 1; dist <= 8 && !layer.empty(); ++dist) {
      next.clear();
      for (lock_pid a : layer) {
        const auto it = pred.find(a.packed());
        if (it == pred.end())
          continue;
        for (lock_pid y : it->second) {
          if (!seen.insert(y.packed()).second)
            continue;
          next.push_back(y);
          const rec *ry = rec_of(y);
          if (ry && opts.inheritance && applies(y) && ry->base < best)
            best = ry->base;
        }
      }
      layer.swap(next);
    }
    return best;
  }
};

void lock_table::impl::check() const {
  const auto rs = sorted_recs();
  const auto hs = sorted_holds();
  const auto ts = sorted_tickets();

  // I3: records
  for (const rec *r : rs) {
    if (r->holds.empty() && !r->ticket)
      violation("I3", "the record of " + pid_text(r->pid) + " has no hold and no ticket");
    for (std::size_t i = 0; i < r->holds.size(); ++i) {
      const lock_hold *h = hold_of(r->holds[i]);
      if (!h || h->pid != r->pid || (i && r->holds[i - 1] >= r->holds[i]))
        violation("I3", "the record of " + pid_text(r->pid) + " lists hold " +
                            std::to_string(r->holds[i]) + ", which is not one of its own");
    }
    if (r->pid.home >= homes.size() || !homes[r->pid.home].attached)
      violation("4.13", "the record of " + pid_text(r->pid) + " is on a home not attached");
  }
  for (const lock_hold *h : hs) {
    const rec *r = rec_of(h->pid);
    if (!r || std::find(r->holds.begin(), r->holds.end(), h->id) == r->holds.end())
      violation("I3", "hold " + std::to_string(h->id) + " is not in its process's record");
    if (h->id > next_hold || h->keys.empty())
      violation("I3", "hold " + std::to_string(h->id) + " is malformed");
  }

  // I2: tickets and the queue
  std::size_t n_queue = 0;
  for (const lock_ticket *t = q_head; t; t = t->next) {
    ++n_queue;
    if (t->state != ticket_state::queued || ticket_of(t->seq) != t)
      violation("I2", "queue entry " + std::to_string(t->seq) + " is not a queued ticket");
    if ((t->prev ? t->prev->next : q_head) != t || (t->next ? t->next->prev : q_tail) != t ||
        (t->next && t->next->seq <= t->seq))
      violation("I2", "the queue's links or seq order are broken at " + std::to_string(t->seq));
  }
  if (n_queue != queued)
    violation("I2", "the queue holds " + std::to_string(n_queue) + " tickets, counted " +
                        std::to_string(queued));
  std::size_t owned = 0;
  for (const rec *r : rs) {
    if (!r->ticket)
      continue;
    ++owned;
    const lock_ticket *t = ticket_of(r->ticket);
    if (!t || t->pid != r->pid)
      violation("I2", "the record of " + pid_text(r->pid) + " names ticket " +
                          std::to_string(r->ticket) + ", which is not its own");
  }
  std::size_t n_queued = 0;
  for (const lock_ticket *t : ts) {
    const rec *r = rec_of(t->pid);
    if (!r || r->ticket != t->seq)
      violation("I2", "ticket " + std::to_string(t->seq) + " is not its process's ticket");
    if (t->seq > next_seq || t->nkeys == 0)
      violation("I2", "ticket " + std::to_string(t->seq) + " is malformed");
    switch (t->state) {
    case ticket_state::queued: {
      ++n_queued;
      if (t->nodes.size() != t->nkeys || t->hold || t->error != ticket_error::none)
        violation("I2", "queued ticket " + std::to_string(t->seq) + " is malformed");
      break;
    }
    case ticket_state::granted: {
      const lock_hold *h = hold_of(t->hold);
      if (!h || h->pid != t->pid || h->body_started)
        violation("I2", "granted ticket " + std::to_string(t->seq) +
                            " has no unstarted hold of its process");
      break;
    }
    case ticket_state::cancelled:
      if (t->hold || t->error != ticket_error::none)
        violation("I2", "cancelled ticket " + std::to_string(t->seq) + " is malformed");
      break;
    case ticket_state::failed:
      if (t->hold || t->error == ticket_error::none ||
          (t->error == ticket_error::deadlock && t->cycle.empty()) ||
          (t->error == ticket_error::timeout && !mut(lock_table::cancel_marks_failed)))
        violation("I2", "failed ticket " + std::to_string(t->seq) + " has no error");
      break;
    }
  }
  if (owned != ts.size())
    violation("I2", "the tickets are not exactly the records' tickets");
  if (n_queued != queued)
    violation("I2", "the queue is not exactly the queued tickets");

  // I1
  for (std::size_t i = 0; i < hs.size(); ++i)
    for (std::size_t j = i + 1; j < hs.size(); ++j)
      if (hs[i]->pid != hs[j]->pid &&
          conflicts(hs[i]->mode, view(hs[i]->keys), hs[j]->mode, view(hs[j]->keys)))
        violation("I1", "holds " + std::to_string(hs[i]->id) + " of " + pid_text(hs[i]->pid) +
                            " and " + std::to_string(hs[j]->id) + " of " + pid_text(hs[j]->pid) +
                            " conflict");

  // I15's state part: X generations distinct, none above next_gen
  std::set<std::uint64_t> xgens;
  for (const lock_hold *h : hs) {
    if (h->gen > next_gen)
      violation("I15", "hold " + std::to_string(h->id) + " has a generation above next_gen");
    if (h->mode == lock_mode::exclusive && !xgens.insert(h->gen).second)
      violation("I15", "two X holds share generation " + std::to_string(h->gen));
  }

  // I4, I11's flags
  check_trie();
  std::set<const lock_node *> taken;
  std::set<std::string, std::less<>> expect_reports;
  for (const lock_node *n : all_nodes())
    if (n->abandoned)
      expect_reports.insert(n->key);
  for (const lock_hold *h : hs) {
    if (h->mode == lock_mode::shared && !h->taken.empty())
      violation("I11", "S hold " + std::to_string(h->id) + " took an abandoned flag");
    for (const lock_node *n : h->taken) {
      if (std::find(h->keys.begin(), h->keys.end(), n) == h->keys.end())
        violation("I11", "hold " + std::to_string(h->id) + " took a flag not of its keys");
      if (n->abandoned || !taken.insert(n).second)
        violation("I11", "the abandoned flags and taken sets are not disjoint at " + n->key);
      if (!h->body_started)
        expect_reports.insert(n->key);
    }
  }
  if (expect_reports != g_pending_report)
    violation("I11", "abandonment reports are not conserved");

  // I6's first half
  for (const lock_ticket *t = q_head; t; t = t->next)
    if (grantable_def(t))
      violation("I6", "queued ticket " + std::to_string(t->seq) + " of " + pid_text(t->pid) +
                          " is grantable");

  // I7: the wait-for graph is acyclic
  std::unordered_map<std::uint64_t, std::vector<lock_pid>> g;
  for (const lock_ticket *t = q_head; t; t = t->next)
    g[t->pid.packed()] = successors_def(t);
  std::unordered_map<std::uint64_t, int> colour; // 1: on the path, 2: done
  for (const auto &kv : g) {
    if (colour[kv.first])
      continue;
    struct frame {
      std::uint64_t a;
      std::size_t i;
    };
    std::vector<frame> stack{{kv.first, 0}};
    colour[kv.first] = 1;
    while (!stack.empty()) {
      frame &f = stack.back();
      const auto it = g.find(f.a);
      if (it == g.end() || f.i == it->second.size()) {
        colour[f.a] = 2;
        stack.pop_back();
        continue;
      }
      const std::uint64_t b = it->second[f.i++].packed();
      if (colour[b] == 1)
        violation("I7", "the wait-for graph has a cycle");
      if (colour[b] == 0) {
        colour[b] = 1;
        stack.push_back({b, 0}); // invalidates f
      }
    }
  }

  // I8's manager clause: every inh is inh_formula, computed from scratch over the edges just
  // recomputed from the definitions (g), never through relevel_all's code or its set A
  const bool lend = lending();
  pred_map pred;
  if (lend)
    for (const lock_ticket *t = q_head; t; t = t->next)
      for (lock_pid b : g[t->pid.packed()])
        pred[b.packed()].push_back(t->pid);
  for (const rec *r : rs) {
    const int want = inh_def(r->pid, pred, lend);
    if (r->inh != want)
      violation("I8", "the record of " + pid_text(r->pid) + " has inh " +
                          (r->inh == lock_inf ? std::string("INF") : std::to_string(r->inh)) +
                          ", inh_formula " +
                          (want == lock_inf ? std::string("INF") : std::to_string(want)) +
                          (lend ? "" : " (no lending home)"));
  }

  // I9's key-cache clause
  for (std::size_t h = 0; h < homes.size(); ++h) {
    if (homes[h].attached && !homes[h].has_refs)
      violation("I9", "attached home H" + std::to_string(h) + " has no key-cache entry");
    if (!homes[h].attached && homes[h].has_refs)
      violation("I9",
                "the key-cache references of detached home H" + std::to_string(h) + " are kept");
  }
}

void lock_table::impl::check_trie() const {
  struct expect {
    std::vector<const lock_hold *> here[2];
    std::uint32_t sub[2] = {0, 0};
    std::vector<const lock_ticket *> wq;
    std::uint32_t subw = 0;
    std::uint32_t refs = 0;
  };
  std::unordered_map<const lock_node *, expect> e;
  e[&root];
  auto path = [&](const lock_node *n, auto &&bump) {
    for (const lock_node *a = n; a; a = a->parent)
      bump(e[a]);
  };
  holds.for_each([&](std::uint64_t, lock_hold *h) {
    const int m = int(h->mode);
    for (const lock_node *n : h->keys) {
      if (!n->live)
        violation("I4", "hold " + std::to_string(h->id) + " has a freed key node");
      e[n].here[m].push_back(h);
      path(n, [m](expect &x) { ++x.sub[m]; });
    }
  });
  for (const lock_ticket *t = q_head; t; t = t->next)
    for (const lock_node *n : t->nodes) {
      if (!n->live)
        violation("I4", "queued ticket " + std::to_string(t->seq) + " has a freed key node");
      e[n].wq.push_back(t);
      path(n, [](expect &x) { ++x.subw; });
    }
  std::size_t total_refs = 0;
  for (std::size_t h = 0; h < homes.size(); ++h)
    for (const auto &kv : homes[h].refs) {
      if (kv.second == 0 || !kv.first->live)
        violation("I4", "home H" + std::to_string(h) + " has a bad key-cache reference");
      e[kv.first].refs += kv.second;
      total_refs += kv.second;
      path(kv.first, [](expect &) {});
    }
  if (total_refs != refs_total)
    violation("I4", "the key-cache reference total is off");

  std::size_t n_nodes = 0, n_flags = 0;
  std::vector<const lock_node *> stack{&root};
  while (!stack.empty()) {
    const lock_node *n = stack.back();
    stack.pop_back();
    ++n_nodes;
    n_flags += n->abandoned ? 1 : 0;
    const std::string where = n == &root ? std::string("the root") : "node " + n->key;
    if (!n->live || n->free_pending)
      violation("I4", where + " is in the trie but marked free");
    for (const auto &c : n->children) {
      const lock_node *k = c.second;
      const std::string want = n == &root ? c.first : n->key + "." + c.first;
      if (k->parent != n || k->depth != n->depth + 1 || k->key != want || c.first.empty() ||
          c.first.find('.') != std::string::npos)
        violation("I4", "the trie link " + where + " -> " + c.first + " is broken");
      stack.push_back(k);
    }
    const auto it = e.find(n);
    const expect none{};
    const expect &x = it == e.end() ? none : it->second;
    for (int m = 0; m < 2; ++m) {
      std::vector<const lock_hold *> have(n->here[m].begin(), n->here[m].end());
      std::vector<const lock_hold *> want = x.here[m];
      std::sort(have.begin(), have.end());
      std::sort(want.begin(), want.end());
      if (have != want || n->sub[m] != x.sub[m])
        violation("I4", where + "'s " + mode_text(lock_mode(m)) + " counts differ");
    }
    std::vector<const lock_ticket *> have(n->wq.begin(), n->wq.end());
    std::vector<const lock_ticket *> want = x.wq;
    std::sort(have.begin(), have.end());
    std::sort(want.begin(), want.end());
    if (have != want || n->subw != x.subw)
      violation("I4", where + "'s waiter counts differ");
    if (n->refs != x.refs)
      violation("I4", where + "'s key-cache references differ");
    if (n != &root && n->children.empty() && node_empty(n))
      violation("I4", "empty " + where + " is kept");
    if (it != e.end())
      e.erase(it);
  }
  if (!e.empty())
    violation("I4", "a node counted by the definition is not in the trie");
  if (n_nodes != trie_nodes)
    violation("I4", "the trie has " + std::to_string(n_nodes) + " nodes, counted " +
                        std::to_string(trie_nodes));
  if (n_flags != flagged)
    violation("I4", "the abandoned flags are miscounted");
  if (root.parent || root.depth || !root.key.empty())
    violation("I4", "the root is malformed");
}

// -- lock_table ----------------------------------------------------------------------------------

lock_table::lock_table(options opts) : opts_(opts), impl_(std::make_unique<impl>(opts)) {}
lock_table::~lock_table() = default;

home_id lock_table::attach_home(bool prio, std::shared_ptr<lock_port> port,
                                std::shared_ptr<std::atomic<std::int64_t>> progress,
                                lock_notes &out) {
  impl &m = *impl_;
  owner_scope token(m.owner);
  if (opts_.one_home && m.attached > 0)
    throw std::logic_error("lock_table::attach_home: before L3 a manager accepts one home");
  m.begin_op(&out);
  const home_id h = home_id(m.homes.size());
  impl::home_entry &e = m.homes.emplace_back();
  e.attached = true;
  e.prio = prio;
  e.port = std::move(port);
  e.progress = std::move(progress);
  e.has_refs = true; // M.cache_refs[H] = {}
  ++m.attached;
  m.relevel_all({}, {}); // lending may have become true
  m.end_op();
  return h;
}

std::optional<grant> lock_table::try_acquire(lock_pid p, rec_info info, lock_request &req,
                                             const cache_delta &refs) {
  impl &m = *impl_;
  owner_scope token(m.owner);
  if (const impl::rec *r = m.rec_of(p); r && r->ticket)
    violation("4.5", "try_acquire: " + pid_text(p) + " has a ticket");
  if (opts_.self_check)
    m.check_request(req);
  m.begin_op(nullptr);
  m.apply_refs(p.home, req, refs);
  impl::kposes kp;
  small_vector<std::string_view, 4> text;
  for (const key_ref &k : req.keys) {
    kp.push_back(m.locate(k));
    text.push_back(k.text());
  }
  std::optional<grant> res;
  if (m.grantable(p, req.mode, kp, view(text), seq_inf)) {
    m.ensure_rec(p, info);
    small_vector<lock_node *, 4> nodes;
    for (std::size_t i = 0; i < req.keys.size(); ++i) {
      lock_node *n = kp[i].exact ? kp[i].node : m.node_for(req.keys[i].key);
      req.keys[i].node = n;
      nodes.push_back(n);
    }
    const std::uint64_t id = m.new_hold(p, req.mode, view(nodes), true, seq_inf);
    ++m.n_fast;
    const lock_hold *h = m.hold_of(id);
    res = grant{id, h->gen, !h->taken.empty()};
  }
  m.end_op();
  return res;
}

block_result lock_table::block(lock_pid p, rec_info info, lock_request &req,
                               const cache_delta &refs, lock_notes &out) {
  impl &m = *impl_;
  owner_scope token(m.owner);
  if (const impl::rec *r = m.rec_of(p); r && r->ticket)
    violation("4.6", "block: " + pid_text(p) + " has a ticket");
  if (opts_.self_check)
    m.check_request(req);
  const lock_pid seeds[] = {p};
  m.begin_op(&out, seeds);
  m.apply_refs(p.home, req, refs);
  impl::rec *r = m.ensure_rec(p, info);
  const std::uint64_t s = ++m.next_seq;
  impl::kposes kp;
  small_vector<std::string_view, 4> text;
  for (const key_ref &k : req.keys) {
    kp.push_back(m.locate(k));
    text.push_back(k.text());
  }
  auto fill_nodes = [&](small_vector<lock_node *, 4> &nodes) {
    for (std::size_t i = 0; i < req.keys.size(); ++i) {
      lock_node *n = kp[i].exact ? kp[i].node : m.node_for(req.keys[i].key);
      req.keys[i].node = n;
      nodes.push_back(n);
    }
  };
  block_result res{s, ticket_state::queued};
  if (m.grantable(p, req.mode, kp, view(text), seq_inf)) { // the state changed since try_acquire
    small_vector<lock_node *, 4> nodes;
    fill_nodes(nodes);
    const std::uint64_t id = m.new_hold(p, req.mode, view(nodes), false, s);
    lock_ticket *t = m.new_ticket(p, s, req);
    t->state = ticket_state::granted;
    t->hold = id;
    t->nodes = nodes;
    r->ticket = s;
    res.state = ticket_state::granted;
    m.end_op();
    return res;
  }
  std::vector<lock_pid> targets;
  m.blockers(
      p, req.mode, kp, seq_inf, [&](lock_hold *h) { targets.push_back(h->pid); },
      [&](lock_ticket *u) { targets.push_back(u->pid); });
  impl::sort_unique(targets);
  std::vector<deadlock_edge> cycle = m.deadlock_cycle(p, req, kp, targets);
  if (!cycle.empty()) {
    lock_ticket *t = m.new_ticket(p, s, req);
    t->state = ticket_state::failed;
    t->error = ticket_error::deadlock;
    t->cycle = std::move(cycle);
    r->ticket = s;
    ++m.n_deadlocks;
    res.state = ticket_state::failed;
    m.end_op();
    return res;
  }
  lock_ticket *t = m.new_ticket(p, s, req);
  fill_nodes(t->nodes);
  r->ticket = s;
  m.enqueue(t);
  ++m.n_waits;
  m.relevel_all({}, {}); // P has edges out now
  m.end_op();
  return res;
}

lock_outcome lock_table::resume(lock_pid p, std::uint64_t tid) {
  impl &m = *impl_;
  owner_scope token(m.owner);
  lock_ticket *t = m.ticket_of(tid);
  impl::rec *r = m.rec_of(p);
  if (!t || t->pid != p || !r || r->ticket != tid)
    violation("4.7", "resume: ticket " + std::to_string(tid) + " is not " + pid_text(p) + "'s");
  m.begin_op(nullptr);
  lock_outcome o;
  if (t->state == ticket_state::queued) { // unreachable (I6); nothing changes
    m.end_op();
    return o;
  }
  if (t->state == ticket_state::granted) {
    const lock_hold *h = m.hold_of(t->hold);
    o.what = lock_outcome::kind::granted;
    o.g = grant{t->hold, h->gen, !h->taken.empty()};
  } else if (t->state == ticket_state::cancelled || t->error == ticket_error::timeout) {
    o.what = lock_outcome::kind::timeout;
  } else {
    o.what = lock_outcome::kind::deadlock;
    o.cycle = std::move(t->cycle);
  }
  m.delete_ticket(t);
  r->ticket = 0;
  m.maybe_drop_rec(p);
  m.end_op();
  return o;
}

void lock_table::start_body(lock_pid p, std::uint64_t hold) {
  impl &m = *impl_;
  owner_scope token(m.owner);
  lock_hold *h = m.hold_of(hold);
  if (!h || h->pid != p)
    violation("4.8", "start_body: hold " + std::to_string(hold) + " is not " + pid_text(p) + "'s");
  m.begin_op(nullptr);
  h->body_started = true;
  m.body_start(h);
  m.end_op();
}

std::optional<ticket_state> lock_table::cancel(lock_pid p, std::uint64_t tid, lock_notes &out) {
  impl &m = *impl_;
  owner_scope token(m.owner);
  const lock_pid seeds[] = {p};
  m.begin_op(&out, seeds);
  lock_ticket *t = m.ticket_of(tid);
  std::optional<ticket_state> res;
  if (!t || t->pid != p) {
    m.end_op();
    return res;
  }
  if (t->state != ticket_state::queued) { // granted: the waiter adopts the grant
    res = t->state;
    m.end_op();
    return res;
  }
  impl::region reg(t->nodes.begin(), t->nodes.end());
  m.dequeue(t);
  t->nodes.clear();
  if (m.mut(cancel_marks_failed)) {
    t->state = ticket_state::failed;
    t->error = ticket_error::timeout;
  } else {
    t->state = ticket_state::cancelled;
  }
  ++m.n_timeouts;
  if (m.mut(cancel_reexamines_holds_only))
    reg.clear(); // the mutation re-examines nothing for a ticket's region
  m.in_cancel = true;
  m.finish(reg, seeds, {});
  m.in_cancel = false;
  res = ticket_state::cancelled;
  m.end_op();
  return res;
}

void lock_table::release(lock_pid p, std::uint64_t hold, lock_notes &out) {
  impl &m = *impl_;
  owner_scope token(m.owner);
  lock_hold *h = m.hold_of(hold);
  const bool own = h && h->pid == p; // the owner check (I12)
  const lock_pid seeds[] = {p};
  m.begin_op(&out, own ? std::span<const lock_pid>(seeds) : std::span<const lock_pid>());
  if (own) {
    impl::region reg;
    m.drop_hold(h, false, reg);
    m.in_release = true;
    m.finish(reg, seeds, {});
    m.in_release = false;
  }
  m.end_op();
}

void lock_table::end(lock_pid p, bool abandon, lock_notes &out) {
  impl &m = *impl_;
  owner_scope token(m.owner);
  const lock_pid ended[] = {p};
  m.begin_op(&out, ended);
  impl::region reg;
  m.end_locked(p, abandon, reg);
  m.finish(reg, {}, ended); // ended: the seed of 4.3's note, and skipped
  m.end_op();
}

void lock_table::detach_home(home_id h, lock_notes &out) {
  impl &m = *impl_;
  owner_scope token(m.owner);
  if (h >= m.homes.size() || !m.homes[h].attached)
    violation("4.11", "detach_home: home " + std::to_string(h) + " is not attached");
  std::vector<lock_pid> ended;
  m.recs.for_each([&](std::uint64_t, impl::rec *r) {
    if (r->pid.home == h)
      ended.push_back(r->pid);
  });
  std::sort(ended.begin(), ended.end(), pid_less);
  m.begin_op(&out, ended); // the ended processes: the seeds of 4.3's note
  impl::home_entry &e = m.homes[h];
  if (!m.mut(keep_detached_refs)) {
    for (const auto &kv : e.refs) { // every cache reference of H (4.15's unref)
      kv.first->refs -= kv.second;
      m.refs_total -= kv.second;
      m.maybe_free(kv.first);
    }
    e.refs.clear();
    e.has_refs = false; // delete M.cache_refs[H]
  }
  impl::region reg;
  for (lock_pid p : ended)
    m.end_locked(p, true, reg);
  e.attached = false; // delete M.homes[H]
  e.prio = false;
  e.port.reset();
  e.progress.reset();
  --m.attached;
  m.finish(reg, {}, ended);
  m.end_op();
}

void lock_table::set_base(lock_pid p, int base, lock_notes &out) {
  impl &m = *impl_;
  owner_scope token(m.owner);
  impl::rec *r = m.rec_of(p);
  const lock_pid seeds[] = {p};
  m.begin_op(&out, r ? std::span<const lock_pid>(seeds) : std::span<const lock_pid>());
  if (r) {
    r->base = base;
    m.relevel_all({}, {});
  }
  m.end_op();
}

int lock_table::read_inh(lock_pid p) {
  impl &m = *impl_;
  owner_scope token(m.owner);
  impl::rec *r = m.rec_of(p);
  if (!r)
    return lock_inf;
  if (!m.mut(coalesce_no_clear))
    r->relevel_pending = false; // the apply clears the coalescing flag (4.3, 5.9)
  return r->inh;
}

lock_query_result lock_table::query(std::string_view abs_prefix, std::optional<home_id> viewer_home,
                                    std::string_view viewer_owner) const {
  owner_scope token(impl_->owner);
  return impl_->query(abs_prefix, viewer_home, viewer_owner);
}

void lock_table::ref_nodes(home_id h, std::span<key_ref> keys) {
  impl &m = *impl_;
  owner_scope token(m.owner);
  m.begin_op(nullptr);
  for (key_ref &k : keys)
    k.node = m.ref(h, k);
  m.end_op();
}

void lock_table::unref_nodes(home_id h, std::span<lock_node *const> nodes) {
  impl &m = *impl_;
  owner_scope token(m.owner);
  m.begin_op(nullptr);
  for (lock_node *n : nodes)
    m.unref(h, n);
  m.end_op();
}

std::span<lock_node *const> lock_table::hold_keys(std::uint64_t hold) const {
  owner_scope token(impl_->owner);
  const lock_hold *h = impl_->hold_of(hold);
  if (!h)
    return {};
  return std::span<lock_node *const>(h->keys.data(), h->keys.size());
}

std::optional<lock_table::rec_view> lock_table::rec_of(lock_pid p) const {
  owner_scope token(impl_->owner);
  const impl::rec *r = impl_->rec_of(p);
  if (!r)
    return std::nullopt;
  rec_view v;
  v.owner = r->owner;
  v.base = r->base;
  v.holds.assign(r->holds.begin(), r->holds.end());
  if (r->ticket)
    v.ticket = r->ticket;
  v.inh = r->inh;
  return v;
}

std::optional<lock_table::ticket_view> lock_table::ticket_of(std::uint64_t tid) const {
  owner_scope token(impl_->owner);
  const lock_ticket *t = impl_->ticket_of(tid);
  if (!t)
    return std::nullopt;
  ticket_view v;
  v.pid = t->pid;
  v.state = t->state;
  if (t->hold)
    v.hold = t->hold;
  return v;
}

std::optional<lock_table::hold_view> lock_table::hold_of(std::uint64_t id) const {
  owner_scope token(impl_->owner);
  const lock_hold *h = impl_->hold_of(id);
  if (!h)
    return std::nullopt;
  return hold_view{h->pid, h->mode, h->gen, h->body_started};
}

std::size_t lock_table::home_cache_refs(home_id h, const lock_node *n) const {
  owner_scope token(impl_->owner);
  if (h >= impl_->homes.size())
    return 0;
  const auto &refs = impl_->homes[h].refs;
  const auto it = refs.find(const_cast<lock_node *>(n));
  return it == refs.end() ? 0 : it->second;
}

std::size_t lock_table::home_cache_refs(home_id h) const {
  owner_scope token(impl_->owner);
  if (h >= impl_->homes.size())
    return 0;
  std::size_t total = 0;
  for (const auto &kv : impl_->homes[h].refs)
    total += kv.second;
  return total;
}

lock_stats lock_table::stats() const {
  const impl &m = *impl_;
  owner_scope token(impl_->owner);
  lock_stats s;
  s.holds = m.holds.size();
  s.queued = m.queued;
  s.grants = m.n_grants;
  s.fast_grants = m.n_fast;
  s.waits = m.n_waits;
  s.timeouts = m.n_timeouts;
  s.deadlocks = m.n_deadlocks;
  s.abandoned = m.n_abandoned;
  s.leaked_at_exit = leaked_at_exit_;
  s.ends = m.n_ends;
  s.trie_nodes = m.trie_nodes;
  s.cache_refs = m.refs_total;
  s.relevels = m.n_relevels;
  s.relevels_coalesced = m.n_coalesced;
  return s;
}

void lock_table::check() const {
  owner_scope token(impl_->owner);
  impl_->check();
}

bool lock_table::empty() const {
  const impl &m = *impl_;
  owner_scope token(impl_->owner);
  return m.recs.size() == 0 && m.holds.size() == 0 && m.tickets.size() == 0 && m.queued == 0 &&
         m.root.sub[0] == 0 && m.root.sub[1] == 0 && m.root.subw == 0;
}

std::string lock_table::canonical() const {
  owner_scope token(impl_->owner);
  return impl_->canonical();
}

} // namespace detail

namespace {
bool check_forced_by_environment() {
  static const bool forced = [] {
    const char *v = std::getenv("CVC_STATE_EXEC_CHECK");
    return v && *v && *v != '0';
  }();
  return forced;
}
} // namespace

lock_manager::lock_manager() : lock_manager(options{}) {}

lock_manager::lock_manager(options opts) : opts_(opts) {
  if (opts_.threading != lock_threading::single)
    throw std::invalid_argument("lock_manager: shared mode is phase L3");
  detail::lock_table::options t;
  t.inheritance = opts_.inheritance;
  t.self_check = opts_.self_check || check_forced_by_environment();
  t.coalesce_relevels = opts_.coalesce_relevels;
  table_ = std::make_unique<detail::lock_table>(t);
}

lock_manager::~lock_manager() = default;

lock_query_result lock_manager::query(std::string_view abs_prefix, const detail::lock_home *viewer,
                                      std::string_view owner) const {
  // Pids are shown only for the viewer's own home (and owner): pids repeat across schedulers.
  return table_->query(abs_prefix,
                       viewer ? std::optional<detail::home_id>(viewer->id()) : std::nullopt, owner);
}

lock_stats lock_manager::stats() const { return table_->stats(); }
lock_threading lock_manager::threading() const { return opts_.threading; }
const lock_manager::options &lock_manager::opts() const { return opts_; }
void lock_manager::check() const { table_->check(); }

} // namespace cvc::state_exec
