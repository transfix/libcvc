/**
 * @file lock_manager.h
 * @brief The state_exec lock manager: named, scoped locks on state-tree paths.
 *
 * docs/STATE_EXEC_LOCKS.md is the design and docs/STATE_EXEC_LOCKS_PROTOCOL.md the exact
 * protocol. One manager serves an app (app::lock_manager(), from phase L1.12) or a single
 * scheduler (its private one, made on first use). A scheduler reaches it through its lock home
 * (detail::lock_home, one per scheduler), which runs the home actions of protocol section 5; the
 * manager itself is the table of protocol sections 2-4 (detail::lock_table).
 *
 * Phase L1: single mode only (one thread calls the manager), one home per manager. Shared mode,
 * ports and several homes are phase L3's; inheritance is phase L2's.
 */
#ifndef CVC_STATE_EXEC_LOCK_MANAGER_H
#define CVC_STATE_EXEC_LOCK_MANAGER_H

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace cvc::state_exec {

/// A request's mode: shared (S) or exclusive (X). Two shared requests never conflict.
enum class lock_mode : std::uint8_t { shared, exclusive };

/// How a manager is called: by one thread (single), or by several under its mutex (shared,
/// phase L3). Fixed at construction.
enum class lock_threading : std::uint8_t { single, shared };

/// One locked key, as lock-query reports it (protocol 4.13). Keys are absolute here; the DSL shows
/// them relative to the caller's chroot.
struct lock_info {
  std::string key;
  lock_mode mode = lock_mode::shared;      // exclusive if an X hold has the key
  std::vector<int> pids;                   // the visible holders' pids (same home and owner)
  std::size_t count = 0;                   // every hold with the key, visible or not
  std::optional<std::uint64_t> generation; // the latest X grant's generation on the key
  bool abandoned = false;                  // the key's abandoned flag is set
};

/// One queued request, as lock-query reports it, in queue order.
struct lock_waiter_info {
  std::vector<std::string> keys; // absolute, canonical
  lock_mode mode = lock_mode::shared;
  std::optional<int> pid; // when visible to the viewer
};

/// What lock-query returns (gap G-13: the design's query returned only the held part).
struct lock_query_result {
  std::vector<lock_info> held;
  std::vector<lock_waiter_info> waiting;
};

/// The manager's counters (gap G-35). The layout is declared in full in phase L1 so it settles
/// once; fields a later phase counts stay 0 before it.
struct lock_stats {
  std::size_t holds = 0;         // holds now
  std::size_t queued = 0;        // queued tickets now
  std::uint64_t grants = 0;      // every grant: fast path, block's immediate grant, re-examination
  std::uint64_t fast_grants = 0; // grants by try_acquire
  std::uint64_t waits = 0;       // tickets queued by block
  std::uint64_t timeouts = 0;    // tickets cancelled
  std::uint64_t deadlocks = 0;   // block calls that failed with a DeadlockError
  std::uint64_t abandoned = 0;   // keys flagged abandoned (an X holder's body ended by a kill)
  std::uint64_t leaked_at_exit = 0;     // holds left at a normal termination (counted by the home)
  std::uint64_t ends = 0;               // end calls that found a record
  std::size_t trie_nodes = 0;           // trie nodes now, the root included
  std::size_t cache_refs = 0;           // key-cache references now, over every home
  std::uint64_t relevels = 0;           // relevel notifications emitted (phase L2)
  std::uint64_t relevels_coalesced = 0; // relevels coalesced away (phase L2)
  std::uint64_t posts = 0;              // posts that landed in a port (phase L3)
  std::uint64_t posts_dropped = 0;      // posts refused by a closed port (phase L3)
  std::uint64_t posts_discarded = 0;    // posts accepted, then discarded at the close (phase L3)
};

namespace detail {
class lock_table;
class lock_home;
} // namespace detail

/// The lock manager. Owned by shared_ptr: the app (or a scheduler, for its private one) and every
/// lock home attached to it.
class lock_manager {
public:
  struct options {
    lock_threading threading = lock_threading::single;
    bool inheritance = true; // priority inheritance (phase L2); a test-only switch (design 6.2)
    bool self_check = false; // check() after every operation; also forced on by the environment
                             // variable CVC_STATE_EXEC_CHECK=1, read once (gap G-4)
    bool coalesce_relevels = true; // at most one relevel per process in a port (protocol 4.3)
  };

  lock_manager();
  /// Phase L1: options.threading must be single (shared mode is phase L3's): throws
  /// std::invalid_argument otherwise.
  explicit lock_manager(options opts);
  ~lock_manager();
  lock_manager(const lock_manager &) = delete;
  lock_manager &operator=(const lock_manager &) = delete;

  /// lock-query (protocol 4.13): the held keys under `abs_prefix` (an absolute key, or "" for
  /// every key) and the queued requests with a key under it. Pids are shown only for processes of
  /// the viewer's home and `owner`; a null viewer (a host with no home yet) sees none.
  lock_query_result query(std::string_view abs_prefix, const detail::lock_home *viewer,
                          std::string_view owner) const;
  lock_stats stats() const;
  lock_threading threading() const;
  const options &opts() const;

  /// Asserts the manager's invariant clauses (protocol section 6); throws detail::lock_violation
  /// naming the invariant on a violation.
  void check() const;

  /// The table, for lock homes and tests.
  detail::lock_table &table() { return *table_; }
  const detail::lock_table &table() const { return *table_; }

private:
  options opts_;
  std::unique_ptr<detail::lock_table> table_;
};

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_LOCK_MANAGER_H
