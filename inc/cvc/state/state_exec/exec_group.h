/**
 * @file exec_group.h
 * @brief An execution group: one async_scheduler with its own async host, cycle registry and
 * math.random stream, pumped by one thread at a time (docs/STATE_EXEC_THREADED.md 4.1, 4.2, 4.7,
 * 4.9; Stage 1).
 *
 * A group is the unit the app's exec_pool (exec_pool.h) pumps: an isolated Ariadne document, a
 * pooled pycvc Exec, a host's own groups. Its token is its scheduler's pump owner
 * (async_scheduler::pump_owner_scope), which pump() takes for the pump: the pool takes it per
 * pump, and hands the group between threads through it (an acquire and a release), so one thread
 * pumps a group at a time. Between pumps the host touches the group from its own thread.
 */
#ifndef CVC_STATE_EXEC_EXEC_GROUP_H
#define CVC_STATE_EXEC_EXEC_GROUP_H

#include <atomic>
#include <cstdint>
#include <cvc/core/thread_class.h>
#include <cvc/state/state_exec/heap.h>           // cycle_registry
#include <cvc/state/state_exec/intrinsics.h>     // provider_affinity
#include <cvc/state/state_exec/scheduler_base.h> // execute_options
#include <cvc/state/state_exec/types.h>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace cvc {
class app;
class state;
} // namespace cvc

namespace cvc::state_exec {

class async_host;
class async_scheduler;
struct random_stream; // stdlib.h

namespace detail {
/// What the pools keep on a group, between pumps and during a pump_all (exec_pool.cpp, S1.6); not
/// for hosts. A pump_all's caller and its workers touch it under the pool's mutex, `call` under
/// the pools' membership mutex, `failure` on the thread pumping the group.
struct group_pool_link {
  std::uint64_t seq = 0;           ///< creation order (deterministic mode; threads 0)
  double last_pump_seconds = 0.0;  ///< its last pump's time: the LPT order and the helper wake
  bool skipped = false;            ///< not started before its pump_all's deadline: first next time
  std::uint64_t skipped_since = 0; ///< the pump_all that first skipped it (the oldest goes first)
  int last_thread = -1;            ///< which thread pumped it last (CVC_EXEC_FORCE_MOVE)
  std::atomic<void *> call{nullptr}; ///< the pump_all it belongs to now, or null
  random_stream *random = nullptr;   ///< its math.random stream (seeded from its name)
  std::string failure;               ///< what failed its last failed pump
  /// Its lock home is attached to a single-mode manager (S1-Q1): every pool pumps it on the
  /// caller of pump_all only, as a host-affinity group. That keeps one pump_all's pumps of it on
  /// one thread; a single manager also needs one thread to drive the whole app (every pump_all of
  /// its groups, group 0's drivers, direct pumps), which the host keeps.
  bool caller_only = false;
};
/// The groups alive that were built with `app` (~app asserts in debug builds that none is).
std::size_t live_groups(const cvc::app *app) noexcept;
} // namespace detail

/// Where a group may be pumped: `any` thread of the pool, or only the `host` thread (the caller
/// of pump_all), as a group whose natives or subscribers need it is.
enum class group_affinity { any, host };

/// What one pump may spend: at most `steps` steps (0: none, so the pump only drains and wakes)
/// and `seconds` of the group's own time (a time_budget, td 4.10; infinity: no limit). The pool
/// gives a group the smaller of `seconds` and what its frame has left.
struct pump_budget {
  std::uint64_t steps;
  double seconds;
};

/// What one pump did: the steps it took, the processes wake_awaiting() woke, and whether a C++
/// exception escaped it (the pool then fails the group, S1.6).
struct pump_result {
  std::uint64_t steps = 0;
  int woken = 0;
  bool failed = false;
};

class exec_group {
public:
  struct options {
    std::string name; ///< for logs and stats; seeds the random stream in deterministic mode
    /// The owner tag of what it runs (an Ariadne Runtime's): execute() submits with it when the
    /// execute_options give none, so kill_owner(owner) reaches the group's own submissions.
    std::string owner;
    group_affinity affinity = group_affinity::any;
    cvc::app *app = nullptr;          ///< its settings, lock manager, pid source and kernel pool
    cvc::state *watch_root = nullptr; ///< null: the app's root
    /// The scope its global env extends; null: frozen_builtins(). A program's set binds in its
    /// own run env, never in the base, so a base is only read; but a native or host that writes
    /// an unfrozen base shared by several groups writes it from whichever thread pumps each, so a
    /// shared base should be frozen. An unfrozen base one group owns is fine (the bench's
    /// driver=group base=own), so it is not refused. (The env a host passes as
    /// execute_options::env is the program's, and two groups must never share one.)
    environment_ptr base;
    std::optional<std::uint64_t> seed; ///< math.random's seed; unset: random_device words
    /// Declared for L4 and Stage 2; ignored in Stage 1.
    cvc::thread_class thread_class = cvc::thread_class::normal;
    std::vector<std::string> channel_prefixes;
    /// Tests: called inside each pump, holding the token (GR-12 throws from it).
    std::function<void(exec_group &)> pump_hook_for_test;
    /// Tests: called in the teardown between closing the lock port and the kills (GL-2).
    std::function<void(exec_group &)> teardown_hook_for_test;
  };

  /// Build the group's scheduler as app::exec_scheduler() builds the app's: the policy from the
  /// tree, the watch root, the host parser and data defaults, the base env, the pool's pid source,
  /// its own registry and random stream, and the app's lock manager (S1.5). That makes the
  /// manager when nothing has yet, in the mode the tree says: a host that wants workers gives
  /// app::exec_pool(opts) first (S1-Q1), else a single manager keeps this group, and every group
  /// of the app, on the caller of pump_all (logged once).
  explicit exec_group(options opts);
  /// The teardown (td 4.2, S1.5), in order, each step catching and logging what it throws: take
  /// the token (waiting for a pump in progress), close the lock port, kill every process, shut the
  /// host down (waiting for its kernels), purge the group's host-call queue entries, reap every
  /// process. Then the host, the scheduler and its envs go, and last the registry, collected and
  /// freed. Never while a pump_all holds the group (from that call's host bodies or callbacks,
  /// or from another thread) nor from inside the group's own pump (an inline native or callback
  /// of it): each aborts, in every build, since the call or the run loop would go on using it.
  ~exec_group();
  exec_group(const exec_group &) = delete;
  exec_group &operator=(const exec_group &) = delete;

  /// The group's scheduler. A host that submits through it directly (sched().execute, restore)
  /// gets none of execute()'s handling: an on_complete given there runs where the group is pumped,
  /// on a pool worker included, inside the pump; the AST is not copied; no owner is defaulted.
  async_scheduler &sched();
  /// The group's async host, async_host(sched(), app), built on first use.
  async_host &host();

  /// One pump: take the token and bind current() (the scheduler binds the registry and the
  /// stream), then drain_ingress(), wake_awaiting() and sync_run_steps(budget), then release
  /// both.
  pump_result pump(const pump_budget &budget);
  /// Whether a pump has work: a runnable process, a process waiting for the next frame (the pump's
  /// wake_awaiting readies it), a pending ingress, or a timer due.
  bool wants_pump() const;

  /// Submit to the group's scheduler, with the group's owner when `opts` names none. The value
  /// form always copies the AST's lists and dicts (td B12), so neither another group nor the host
  /// shares its quoted literals. An on_complete runs on the pump_all caller after the join when a
  /// threaded pump_all pumps the group (on a worker or on the caller), inline otherwise (threads 0,
  /// deterministic mode, no pump_all), as exec_pool::pump_all says.
  int execute(const std::string &script, const execute_options &opts = {});
  int execute(const value_t &expr, const execute_options &opts = {});

  /// Bind a host native `name` into `env` (one of this group's envs) by its kind (td 4.9, S1.8):
  /// in a host-affinity group every kind is bound as it is; in an `any` group a `host_only`
  /// native is not bound (a call is a NameError), and a `host_proxied` one is bound as a wrapper
  /// that runs `fn` on the pump_all caller's thread through the host-call queue (inline when the
  /// calling thread is that caller), parking its process until the result comes back.
  /// "The host thread" here is the caller of the pump_all pumping the group: two pump_all callers
  /// (S1-Q11) run their groups' bodies at once, each on its own thread, so a provider whose state
  /// only one thread may touch serializes its bodies itself.
  void install(const environment_ptr &env, const std::string &name, native_fn fn,
               provider_affinity kind);

  /// Whether a pump failed the group (a C++ exception escaped it).
  bool failed() const;
  /// The group's cycle registry, which its scheduler binds.
  cycle_registry *registry() const;
  /// The group the calling thread is pumping, or null.
  static exec_group *current();
  const options &opts() const;

private:
  struct impl;
  std::unique_ptr<impl> impl_;

public:
  /// The pools' record of this group (exec_pool.cpp); not for hosts.
  detail::group_pool_link &pool_link() noexcept;
};

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_EXEC_GROUP_H
