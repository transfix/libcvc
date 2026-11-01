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
 * Phases L1 and L2 built single mode (one thread calls the manager) with one home per manager, and
 * priority inheritance (protocol 3.3) among the processes of a home whose scheduler uses a
 * priority policy. Phase L3 adds shared mode (every operation under the manager's mutex, M.mu),
 * several homes per manager, and the homes' ports (detail::lock_port), through which an operation
 * on one home's thread reaches a process of another (protocol 2.3, 4.14).
 */
#ifndef CVC_STATE_EXEC_LOCK_MANAGER_H
#define CVC_STATE_EXEC_LOCK_MANAGER_H

#include <atomic>
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

/// How a manager is called: by one thread (single), or by several, each operation under the
/// manager's mutex (shared). Fixed at construction.
enum class lock_threading : std::uint8_t { single, shared };

/// Whether this build has threads. False only on Emscripten without pthreads, where every home is
/// pumped on the one thread: app::lock_manager()'s auto mode picks single there, and a host that
/// attaches with cross_thread passes this, so such a build never asks for a cross-thread attach.
#if defined(__EMSCRIPTEN__) && !defined(__EMSCRIPTEN_PTHREADS__)
inline constexpr bool threads_compiled_in = false;
#else
inline constexpr bool threads_compiled_in = true;
#endif

/// How a scheduler's home attaches to a manager (scheduler::set_lock_manager). `cross_thread`:
/// the home may be pumped on a thread other than the one other homes of the manager use (a pycvc
/// Exec, a scheduler of its own thread). A single-mode manager refuses it with std::logic_error
/// naming state_exec.locks.threading (design 8.3; gap G-24).
struct lock_attach {
  bool cross_thread = false;
};

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
  std::uint64_t relevels = 0;           // relevel notifications emitted (from phase L2)
  std::uint64_t relevels_coalesced = 0; // relevels coalesced away (from phase L2)
  std::uint64_t posts = 0;              // posts that landed in a port (phase L3)
  std::uint64_t posts_dropped = 0;      // posts refused by a closed port (phase L3)
  std::uint64_t posts_discarded = 0;    // posts accepted, then discarded at the close (phase L3)
};

namespace detail {
class lock_table;
class lock_home;

/// A host that pumps schedulers on threads of its own (pycvc, at import) announces itself, so
/// app::lock_manager()'s auto mode picks shared (gap G-24). Process-wide; never cleared.
void note_threaded_host() noexcept;
bool threaded_host_noted() noexcept;
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
  /// A single manager is called by one thread at a time (debug builds assert it); a shared one
  /// takes its mutex (M.mu) around every operation, so homes on several threads can share it
  /// (design 8.3). Either accepts any number of homes; only a shared one accepts a home attached
  /// with lock_attach::cross_thread.
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
  friend class detail::lock_home; // counts its posts (below)

  options opts_;
  std::unique_ptr<detail::lock_table> table_;
  // lock_stats' port counters (protocol 4.14, 5.7): bumped by the homes with relaxed adds outside
  // M.mu, read by stats().
  std::atomic<std::uint64_t> posts_{0};
  std::atomic<std::uint64_t> posts_dropped_{0};
  std::atomic<std::uint64_t> posts_discarded_{0};
};

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_LOCK_MANAGER_H
