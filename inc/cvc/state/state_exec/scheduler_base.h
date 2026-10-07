/**
 * @file scheduler_base.h
 * @brief Abstract scheduler interface + shared scheduler vocabulary.
 *
 * `scheduler_base` is the intrinsic-facing surface of a process scheduler: the
 * exact set of operations the DSL intrinsics (spawn/sleep/msg-send/msg-recv/
 * self/ps/kill/pause/resume + the state-watch registry) invoke through
 * `intrinsics_context.sched`.  Both the synchronous `scheduler` and the
 * coroutine `async_scheduler` derive from it, so the same intrinsics drive
 * either executor without knowing which one it is (§8 P2-prereq: "re-type
 * intrinsics_context.sched to a shared base").
 *
 * The driver-facing surface (step/run/stop/has_runnable/load_settings/…) is
 * deliberately NOT here — the host that pumps a scheduler holds the concrete
 * type and calls those directly.  This keeps the base minimal and its
 * implementers honest about the contract the DSL actually depends on.
 *
 * The shared value types (`scheduling_policy`, `execute_options`,
 * `process_info`, `scheduler_stats`) live here so both schedulers and the base
 * share one definition; `scheduler.h` / `async_scheduler.h` include this.
 */
#ifndef CVC_STATE_EXEC_SCHEDULER_BASE_H
#define CVC_STATE_EXEC_SCHEDULER_BASE_H

#include <cstdint>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/types.h>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace cvc {
class state;
}

namespace cvc::state_exec {

/// Scheduling policies for process selection.
enum class scheduling_policy {
  round_robin, // Equal time slices for all runnable processes
  priority,    // Unix nice-style (-20..19, lower = higher priority)
  priority_rr  // Priority-based, round-robin within same priority
};

/// Options for executing a new process.
struct execute_options {
  std::string name;
  int priority = 0;
  std::string uid;
  std::string gid;
  std::string root_path; // Chroot: confine to subtree (empty = full tree)
  std::string owner;     // Owner-scope tag (Ariadne document/Runtime); "" = unowned
  uint64_t max_steps = 0;
  double max_time = 0.0;      // TOTAL wall-clock run-time seconds
  double max_step_time = 0.0; // §13.8 per-STEP wall-clock cap (fresh each step); 0 = unlimited
  uint64_t max_memory = 0;
  uint64_t max_messages = 0;
  uint64_t max_message_bytes = 0;
  std::unordered_map<std::string, value_t> signal_handlers;
  std::function<void(value_t)> on_complete;
  environment_ptr env;
};

/// Snapshot of a single process for query purposes.
struct process_info {
  int pid;
  std::string name;
  process_status status;
  int priority;
  std::string uid;
  std::string gid;
  uint64_t step_count;
  double elapsed_time;
  uint64_t current_memory;
  uint64_t peak_memory;
  uint64_t max_memory;
  double max_time;
  uint64_t message_count;
  uint64_t max_messages;
  uint64_t message_bytes;
  uint64_t max_message_bytes;
  int parent_pid;
};

/// Aggregate scheduler statistics.
struct scheduler_stats {
  int total_processes = 0;
  int running = 0;
  int ready = 0;
  int paused = 0;
  int terminated = 0;
  int killed = 0;
  uint64_t total_steps = 0;
};

/// Abstract scheduler — the surface the DSL intrinsics call through
/// `intrinsics_context.sched`.  Implemented by both `scheduler` (sync) and
/// `async_scheduler` (coroutine).  Every method here is invoked by
/// intrinsics.cpp; nothing else belongs in this interface.
///
/// No default arguments on the virtuals (a default arg on a virtual is bound
/// to the static type, a well-known footgun) — the intrinsics always pass
/// arguments explicitly, and the concrete schedulers keep their own defaults
/// for direct callers.
class scheduler_base {
public:
  virtual ~scheduler_base() = default;

  // --- Process submission (spawn) ---
  virtual int execute(const std::string &script, const execute_options &opts) = 0;
  virtual int execute(const value_t &expr, const execute_options &opts) = 0;

  // --- Process control (kill/pause/resume/fork/sleep) ---
  virtual bool pause(int pid) = 0;
  virtual bool resume(int pid) = 0;
  virtual bool kill(int pid) = 0;
  virtual int fork(int pid) = 0;

  /// Cooperative sleep: put a process into `waiting`; the pump wakes it once
  /// the deadline passes.
  virtual bool sleep(int pid, double seconds) = 0;

  /// (await expr): yield the process until the next frame boundary. On a frame-driven
  /// scheduler (async_scheduler under the Ariadne pump) it parks the process, re-readied by
  /// wake_awaiting() once per pump. On a scheduler with no frame concept (the sync scheduler,
  /// CLI/pycvc) it is a no-op and `await` degrades to identity (returns its value, no park).
  virtual bool yield_frame(int pid) = 0;

  // --- Inter-process messaging (msg-send/msg-recv) ---
  virtual bool receive_message(int pid, const std::string &path) = 0;
  virtual int deliver_to_receivers(const std::string &path, const value_t &msg) = 0;
  virtual std::optional<value_t> pop_pending_message(const std::string &path) = 0;
  virtual std::size_t pending_message_count(const std::string &path) const = 0;

  // --- Process info (self/ps) ---
  virtual std::vector<process_info> list_processes() const = 0;
  virtual std::optional<process_info> get_process_info(int pid) const = 0;
  virtual scheduler_stats get_stats() const = 0;
  virtual int current_pid() const = 0;
  virtual const process_ptr &current_process() const = 0;

  // --- State-watch registry (state-watch/state-unwatch) ---
  virtual void set_watch_root(cvc::state *root) = 0;
  virtual void register_watch_handler(int pid, int watch_id, value_t handler,
                                      const std::string &path) = 0;
  virtual void unregister_watch_handler(int pid, int watch_id) = 0;
};

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_SCHEDULER_BASE_H
