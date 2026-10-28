/**
 * @file detail/lock_table.h
 * @brief The lock manager's table: protocol sections 2-4 (docs/STATE_EXEC_LOCKS_PROTOCOL.md), one
 *        member function per manager operation.
 *
 * Internal: a lock home (detail/lock_home.h) and the lock tests call it; programs and hosts use
 * lock_manager.h. Each operation is atomic (single mode: one thread calls the table; shared mode,
 * phase L3, adds the manager's mutex around each). Each returns its result and appends its
 * notifications (protocol 2.3) to the `out` list it is given; the caller delivers them after the
 * operation returns (protocol 4.14). No operation calls into a scheduler, a port, a process or the
 * state tree.
 *
 * Keys are absolute and canonical (protocol 1): segments joined by '.', each non-empty with no '.'
 * and no '#'; a request's keys are sorted by their segment tuples and none is a prefix of another.
 * The home canonicalizes; the table asserts it when options::self_check is on.
 */
#ifndef CVC_STATE_EXEC_DETAIL_LOCK_TABLE_H
#define CVC_STATE_EXEC_DETAIL_LOCK_TABLE_H

#include <atomic>
#include <boost/container/small_vector.hpp>
#include <climits>
#include <cstddef>
#include <cstdint>
#include <cvc/state/state_exec/lock_manager.h>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace cvc::state_exec {

struct process;

namespace detail {

template <class T, std::size_t N> using small_vector = boost::container::small_vector<T, N>;

/// A home's id in its manager (protocol 2.1): handed out by attach_home, never reused.
using home_id = std::uint32_t;

/// Pid = (HomeId, int): a process. Pids repeat across homes; pairs do not.
struct lock_pid {
  home_id home = 0;
  std::int32_t pid = 0;
  std::uint64_t packed() const { return (std::uint64_t(home) << 32) | std::uint32_t(pid); }
  friend bool operator==(lock_pid a, lock_pid b) { return a.home == b.home && a.pid == b.pid; }
  friend bool operator!=(lock_pid a, lock_pid b) { return !(a == b); }
  friend bool operator<(lock_pid a, lock_pid b) { return a.packed() < b.packed(); }
};

/// INF of the protocol: no inherited priority.
inline constexpr int lock_inf = INT_MAX;

/// A violation of an invariant found by lock_table::check(), lock_home::check_quiescent() or a
/// grant-time assertion (options::self_check). `invariant` names it: "I1".."I15", or "4.13" and
/// the like for a clause that is not one of section 6's invariants.
class lock_violation : public std::logic_error {
public:
  lock_violation(std::string invariant, const std::string &detail)
      : std::logic_error(invariant + ": " + detail), invariant_(std::move(invariant)) {}
  const std::string &invariant() const noexcept { return invariant_; }

private:
  std::string invariant_;
};

struct lock_hold;
struct lock_ticket;

/// One trie node per key that has a hold, a queued ticket, an abandoned flag or a key-cache
/// reference, and one per proper prefix of such a key (protocol 2.4). The public part (key,
/// parent, depth) is stable for the node's life and is what homes read; the counts are the
/// table's.
struct lock_node {
  // -- public: the key ---------------------------------------------------------------------
  std::string key;             // absolute; "" for the root
  lock_node *parent = nullptr; // null for the root
  std::uint32_t depth = 0;     // segments in `key`; 0 for the root

  // -- the table's (protocol 2.4) ------------------------------------------------------------
  std::map<std::string, lock_node *, std::less<>> children; // by segment
  small_vector<lock_hold *, 2> here[2];                     // holds with this key, by lock_mode
  std::uint32_t sub[2] = {0, 0};     // here[m].size() over this node and every descendant
  small_vector<lock_ticket *, 2> wq; // queued tickets with this key
  std::uint32_t subw = 0;            // wq.size() over this node and every descendant
  bool abandoned = false;            // the key is in M.abandoned
  std::uint32_t refs = 0;            // key-cache references, over every home
  bool live = false;                 // in the trie (false while on the table's free list)
  bool free_pending = false;         // queued for the end-of-operation free check (2.4)
};

/// Whether key node `a` is node `b` or one of its ancestors (protocol 1's prefix).
inline bool node_prefix(const lock_node *a, const lock_node *b) {
  while (b && b->depth > a->depth)
    b = b->parent;
  return b == a;
}

/// Whether absolute key `a` is key `b` or one of its ancestors, on segment boundaries.
inline bool key_prefix(std::string_view a, std::string_view b) {
  if (a.empty())
    return true;
  return b.size() >= a.size() && b.compare(0, a.size(), a) == 0 &&
         (b.size() == a.size() || b[a.size()] == '.');
}

/// A request's key: its trie node when the caller has one (a key-cache hit), else the absolute
/// key. `ref` asks the table to take one key-cache reference of the caller's home on the key
/// (a miss the home's cache will keep; protocol 4.15's refs.add). The table fills `node` in for
/// every key it takes a reference on, and for every key of a granted or queued request (the hold
/// or ticket keeps the node alive); a busy key without a reference may be left null, since its
/// node may be freed at once.
struct key_ref {
  lock_node *node = nullptr;
  std::string_view key;
  bool ref = false;
  std::string_view text() const { return node ? std::string_view(node->key) : key; }
};

/// A canonical request (protocol 1). Up to 4 keys inline.
struct lock_request {
  lock_mode mode = lock_mode::exclusive;
  small_vector<key_ref, 4> keys;
};

/// The home's key-cache evictions that ride a request (protocol 4.15's refs.drop): one
/// reference of the home each. The home pins the entries of the request it builds, so no node
/// here is a key of the request.
struct cache_delta {
  small_vector<lock_node *, 4> drop;
};

/// What the home passes about the process with each request (protocol 4.1's info; gap G-43: the
/// owner is a view, copied into the record only when the table makes one).
struct rec_info {
  std::string_view owner;
  int base = 0;
};

enum class ticket_state : std::uint8_t { queued, granted, cancelled, failed };

/// A grant: the hold's id, its generation, and whether it took an abandoned flag.
struct grant {
  std::uint64_t hold = 0;
  std::uint64_t gen = 0;
  bool abandoned = false;
};

/// One edge of a DeadlockError's cycle (protocol 4.6): `pid` is the edge's source when it shares
/// the requester's home and owner, else nullopt; `key` is absolute (gap G-31: captured as a
/// string when the cycle is found).
struct deadlock_edge {
  std::optional<int> pid;
  std::string key;
};

/// block's result (protocol 4.6).
struct block_result {
  std::uint64_t tid = 0;
  ticket_state state = ticket_state::queued;
};

/// resume's outcome (protocol 4.7).
struct lock_outcome {
  enum class kind : std::uint8_t { queued, granted, timeout, deadlock };
  kind what = kind::queued;
  grant g;                          // when granted
  std::vector<deadlock_edge> cycle; // when deadlock
};

/// Ports are phase L3's (protocol 2.3); declared so a notification can carry one.
class lock_port;

/// A post: wake { pid, tid } or relevel { pid } (protocol 2.3).
struct lock_post {
  enum class kind : std::uint8_t { wake, relevel };
  kind what = kind::wake;
  int pid = 0;
  std::uint64_t tid = 0; // wake only
};

/// A notification: a post for a home, with that home's port captured in the operation (null
/// before phase L3).
struct lock_note {
  home_id home = 0;
  std::shared_ptr<lock_port> port;
  lock_post post;
};
using lock_notes = small_vector<lock_note, 4>;

/// Test-only injection points (gap G-4). Production has none: the library loads a null pointer at
/// each point. The tests install a set through the exported functions below, never through a
/// global (the Windows DLL exports functions only).
struct lock_test_hooks {
  void (*after_grant)(int pid) = nullptr; // a grant is recorded: on the fast path once its lock
                                          // frame holds it (5.1), after a wait when it is
                                          // collected into the mirror (5.9)
  void (*in_step_lock_enter)(int pid) = nullptr; // inside the lock_enter step, before start_body
  void (*before_terminate_end)(process &) = nullptr; // terminate_process, before the end hook
  std::size_t home_cache_capacity = 0;               // 0: the default (1024); read once per home
};
void set_lock_test_hooks(const lock_test_hooks *hooks);
const lock_test_hooks *lock_test_hooks_get();

/// The manager's state and operations (protocol 2.1, 4).
class lock_table {
public:
  /// Test-only mutations of the protocol (docs/STATE_EXEC_LOCKS_PROTOCOL.md section 7 and
  /// tools/locks_model/lockmodel.py's `mutation` names), read only on slow paths. The spec oracle
  /// of state_exec_locks_model_test takes the same bits.
  enum mutation : std::uint32_t {
    no_exemption = 1u << 0,                 // 'no-exempt': 3.1's exemption removed
    cancel_reexamines_holds_only = 1u << 1, // 'region-holds-only': cancel re-examines nothing
    keep_detached_refs = 1u << 2,           // 'no-unref-on-detach': detach keeps cache refs
    unref_pops_entry = 1u << 3,             // 'unref-pops-entry': unref erases an emptied entry
    cancel_marks_failed = 1u << 4,          // 'cancel-marks-failed': a timeout marks failed
    query_first_gen = 1u << 5,              // 'query-first-gen': query shows the first X gen
    plant_divergence = 1u << 6,             // no protocol meaning: the first grant a release
                                            // makes is left out of the grants count, once (the
                                            // lockstep test's own check)
    cancel_skips_relevel = 1u << 7,         // 'no-relevel-cancel': cancel skips relevel_all
    coalesce_no_clear = 1u << 8,    // 'coalesce-no-clear': read_inh leaves relevel_pending set
    impl_note_after_only = 1u << 9, // 'impl-note-after-only': 4.3's set A from the graph after
                                    // the operation only
  };

  struct options {
    bool inheritance = true;
    bool self_check = false;
    bool coalesce_relevels = true;
    bool one_home = true; // before phase L3 a manager accepts one home (protocol 4.4)
    std::uint32_t mutations = 0;
  };

  explicit lock_table(options opts);
  ~lock_table();
  lock_table(const lock_table &) = delete;
  lock_table &operator=(const lock_table &) = delete;

  const options &opts() const { return opts_; }

  // -- 4.4 ----------------------------------------------------------------------------------
  /// Attach a home. `prio`: the home applies priorities (its scheduler uses priority or
  /// priority_rr), taken as given; the first such home makes `lending` true and recomputes every
  /// record's inherited priority (4.3). `port` and `progress` are phase L3's (null before).
  /// Before phase L3 a second home is refused (std::logic_error; protocol 4.4).
  home_id attach_home(bool prio, std::shared_ptr<lock_port> port,
                      std::shared_ptr<std::atomic<std::int64_t>> progress, lock_notes &out);

  // -- 4.5 - 4.12 ------------------------------------------------------------------------------
  /// The fast path: a hold for `p` when the request is grantable now, else nullopt (busy, which
  /// changes nothing but key-cache references). `req`'s nodes are filled as key_ref says.
  std::optional<grant> try_acquire(lock_pid p, rec_info info, lock_request &req,
                                   const cache_delta &refs);
  /// Queue the request, or grant it now, or fail it with a DeadlockError (protocol 4.6).
  block_result block(lock_pid p, rec_info info, lock_request &req, const cache_delta &refs,
                     lock_notes &out);
  /// Collect a ticket's outcome and delete it (protocol 4.7). Requires the ticket to be p's.
  lock_outcome resume(lock_pid p, std::uint64_t tid);
  /// The body of a hold granted through a ticket begins (protocol 4.8).
  void start_body(lock_pid p, std::uint64_t hold);
  /// A timeout: cancel a queued ticket (protocol 4.9). nullopt when the ticket is not p's or
  /// gone; its state when it is not queued (granted: the waiter adopts the grant).
  std::optional<ticket_state> cancel(lock_pid p, std::uint64_t tid, lock_notes &out);
  /// A section ends (protocol 4.10). Nothing when the hold is not p's or gone.
  void release(lock_pid p, std::uint64_t hold, lock_notes &out);
  /// A process ends (protocol 4.11): every hold released, its ticket withdrawn, its record gone.
  void end(lock_pid p, bool abandon, lock_notes &out);
  /// A home is destroyed (protocol 4.11): its cache references dropped, every process of it ended
  /// with abandon, the home deleted.
  void detach_home(home_id h, lock_notes &out);
  /// A priority change (protocol 4.12): the record's base, then every inherited priority it
  /// reaches is recomputed (relevels in `out`).
  void set_base(lock_pid p, int base, lock_notes &out);

  // -- 4.13 -----------------------------------------------------------------------------------
  /// The record's inherited priority, lock_inf without one; clears the record's coalesced-relevel
  /// flag (protocol 4.3, 5.9). A relevel's apply calls it; a check reads rec_of's `inh` instead.
  int read_inh(lock_pid p);
  lock_query_result query(std::string_view abs_prefix, std::optional<home_id> viewer_home,
                          std::string_view viewer_owner) const;

  // -- 4.15 -----------------------------------------------------------------------------------
  /// Take one key-cache reference of home `h` on each key (filling each key_ref's node).
  void ref_nodes(home_id h, std::span<key_ref> keys);
  /// Drop one key-cache reference of home `h` on each node.
  void unref_nodes(home_id h, std::span<lock_node *const> nodes);

  // -- reads for homes ---------------------------------------------------------------------------
  /// A hold's keys (its trie nodes, canonical order), mode and generation; empty keys when the
  /// hold is gone.
  std::span<lock_node *const> hold_keys(std::uint64_t hold) const;
  /// Count a hold left at a normal termination (protocol 5.6; lock_stats::leaked_at_exit).
  void note_leaked_at_exit() { ++leaked_at_exit_; }

  // -- reads for checks (lock_home::check_quiescent's home-side clauses) -----------------------
  struct rec_view {
    std::string owner;
    int base = 0;
    std::vector<std::uint64_t> holds; // sorted
    std::optional<std::uint64_t> ticket;
    int inh = lock_inf;
  };
  /// P's record; nullopt when P has none.
  std::optional<rec_view> rec_of(lock_pid p) const;
  struct ticket_view {
    lock_pid pid;
    ticket_state state = ticket_state::queued;
    std::optional<std::uint64_t> hold;
  };
  /// A ticket; nullopt when it is gone.
  std::optional<ticket_view> ticket_of(std::uint64_t tid) const;
  struct hold_view {
    lock_pid pid;
    lock_mode mode = lock_mode::exclusive;
    std::uint64_t gen = 0;
    bool body_started = false;
  };
  /// A hold; nullopt when it is gone.
  std::optional<hold_view> hold_of(std::uint64_t id) const;
  /// Home h's key-cache references on node n (0 if none).
  std::size_t home_cache_refs(home_id h, const lock_node *n) const;
  /// Every key-cache reference home h holds.
  std::size_t home_cache_refs(home_id h) const;

  // -- checks ---------------------------------------------------------------------------------
  lock_stats stats() const;
  /// The manager's clauses of protocol section 6 (I1; I2, I3, I4 manager clauses; I6 first half;
  /// I7; I8 manager clause; I9's entry per attached home; I11; I15's state part). Throws
  /// lock_violation.
  void check() const;
  /// Whether the table is empty as I9 says it must be once every process has ended: no rec,
  /// hold, ticket or queued ticket, every trie count zero.
  bool empty() const;

  /// The mathematical state (protocol 2.1), one entry per line, every section sorted, so two
  /// implementations of the protocol (this one and the model test's spec oracle) compare equal
  /// exactly when their states are equal. The format, section by section, each line as shown:
  ///
  ///   homes
  ///   H<home> prio=<0|1>
  ///   recs
  ///   (<home>,<pid>) owner="<owner>" base=<base> holds=[<id>,...] ticket=<tid|-> inh=<inh|INF>
  ///   holds
  ///   <id> pid=(<home>,<pid>) keys=[<key>,...] mode=<S|X> gen=<gen> started=<0|1>
  ///       taken=[<key>,...]                                                 (one line)
  ///   tickets
  ///   <tid> pid=(<home>,<pid>) keys=[<key>,...] mode=<S|X>
  ///       state=<queued|granted|cancelled|failed> hold=<id|->
  ///       error=<-|TimeoutError|DeadlockError[(<pid|nil>:<key>),...]>       (one line)
  ///   queue=[<tid>,...]
  ///   abandoned=[<key>,...]
  ///   next seq=<n> hold=<n> gen=<n>
  ///   cache_refs
  ///   H<home> {<key>:<count>,...}
  ///   counters grants=<n>
  ///
  /// Recs sort by (home, pid), holds and tickets by id, keys by their segment tuples (which is
  /// the order of the strings with '.' sorting before every other character), cache_refs by home
  /// then key. A rec's `holds` list is sorted. `inh` prints INF for lock_inf.
  std::string canonical() const;

private:
  struct impl;
  options opts_;
  std::unique_ptr<impl> impl_;
  std::uint64_t leaked_at_exit_ = 0;
};

/// Sorts canonical keys by their segment tuples: '.' orders before every other character.
inline bool key_less(std::string_view a, std::string_view b) {
  const std::size_t n = a.size() < b.size() ? a.size() : b.size();
  for (std::size_t i = 0; i < n; ++i) {
    if (a[i] == b[i])
      continue;
    if (a[i] == '.')
      return true;
    if (b[i] == '.')
      return false;
    return static_cast<unsigned char>(a[i]) < static_cast<unsigned char>(b[i]);
  }
  return a.size() < b.size();
}

} // namespace detail
} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_DETAIL_LOCK_TABLE_H
