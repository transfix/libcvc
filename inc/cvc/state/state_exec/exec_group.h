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

#include <cstdint>
#include <cvc/core/thread_class.h>
#include <cvc/state/state_exec/heap.h>           // cycle_registry
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
    std::string name;  ///< for logs and stats; seeds the random stream in deterministic mode
    std::string owner; ///< the owner tag of what it runs (an Ariadne Runtime's)
    group_affinity affinity = group_affinity::any;
    cvc::app *app = nullptr;           ///< its settings, lock manager, pid source and kernel pool
    cvc::state *watch_root = nullptr;  ///< null: the app's root
    environment_ptr base;              ///< the scope its global env extends; null: the builtins
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
  /// its own registry and random stream, and the app's lock manager (S1.5).
  explicit exec_group(options opts);
  /// The teardown (td 4.2, S1.5), in order, each step catching and logging what it throws: take
  /// the token (waiting for a pump in progress), close the lock port, kill every process, shut the
  /// host down (waiting for its kernels), purge the group's host-call queue entries, reap every
  /// process. Then the host, the scheduler and its envs go, and last the registry, collected and
  /// freed. Not inside a pump_all of the group (a debug assertion).
  ~exec_group();
  exec_group(const exec_group &) = delete;
  exec_group &operator=(const exec_group &) = delete;

  async_scheduler &sched();
  /// The group's async host, async_host(sched(), app), built on first use.
  async_host &host();

  /// One pump: take the token and bind current() (the scheduler binds the registry and the
  /// stream), then drain_ingress(), wake_awaiting() and sync_run_steps(budget), then release
  /// both.
  pump_result pump(const pump_budget &budget);
  /// Whether a pump has work: a runnable process, a pending ingress, or a timer due.
  bool wants_pump() const;

  /// Submit to the group's scheduler. The value form copies an AST that is shared elsewhere first
  /// (td B12), so two groups never share quoted literals.
  int execute(const std::string &script, const execute_options &opts = {});
  int execute(const value_t &expr, const execute_options &opts = {});

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
};

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_EXEC_GROUP_H
