/**
 * @file scheduler.h
 * @brief Round-robin process scheduler with resource enforcement.
 *
 * Manages a set of processes, executing them in round-robin order with
 * configurable time-slice granularity.  Enforces per-process step limits,
 * timeouts, and memory limits.  Supports spawn, fork, kill, pause/resume,
 * and inter-process messaging.
 */
#ifndef CVC_STATE_EXEC_SCHEDULER_H
#define CVC_STATE_EXEC_SCHEDULER_H

#include <chrono>
#include <cvc/core/state_exec/memory_tracker.h>
#include <cvc/core/state_exec/process.h>
#include <cvc/core/state_exec/stackless_evaluator.h>
#include <cvc/core/state_exec/types.h>
#include <functional>
#include <optional>
#include <queue>
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
  uint64_t max_steps = 0;
  double max_time = 0.0;
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

/// Synchronous process scheduler.
///
/// Manages multiple processes backed by stackless_evaluator.  Uses
/// scheduling policies to select which process(es) to step.  Enforces
/// per-process resource limits (max_steps, max_time, max_memory,
/// max_messages) at each step boundary.
class scheduler {
public:
  explicit scheduler(scheduling_policy policy = scheduling_policy::round_robin);

  // --- Process submission ---

  /// Parse and execute a script, returning the assigned PID.
  int execute(const std::string &script, const execute_options &opts = {});

  /// Execute a pre-parsed expression.
  int execute(const value_t &expr, const execute_options &opts = {});

  // --- Stepping ---

  /// Execute one step on the next eligible process.
  /// Returns number of steps executed (0 or 1).
  int step();

  /// Run until all processes complete or global limits reached.
  /// Returns map of pid → exit_code for terminated processes.
  std::unordered_map<int, value_t> run(std::optional<uint64_t> max_steps = std::nullopt,
                                       std::optional<double> max_time = std::nullopt);

  /// Stop the run loop.
  void stop();

  /// Is the run loop active?
  bool is_running() const { return running_; }

  // --- Process control ---

  bool pause(int pid);
  bool resume(int pid);
  bool kill(int pid);

  /// Put a process to sleep for the given number of seconds.
  /// The process enters `waiting` status and is automatically woken
  /// by the scheduler loop once the deadline expires.
  bool sleep(int pid, double seconds);

  /// Put a process into `waiting` status until a message arrives at
  /// the given state-tree path.  Returns false if the PID is invalid
  /// or the process is not in a runnable state.
  bool receive_message(int pid, const std::string &path);

  /// Deliver a message dict to all processes waiting on `path`.
  /// Each matching process has the message pushed to its inbox,
  /// its evaluator result set to the message, and its status
  /// changed to `ready`.  Returns the number of processes woken.
  ///
  /// When no process is currently waiting on `path`, the message
  /// is stored in a per-path FIFO queue so that a future
  /// `msg-recv` call can retrieve it without blocking.  The queue
  /// is bounded by `max_pending_messages` (default 1024); excess
  /// messages are silently dropped.
  int deliver_to_receivers(const std::string &path, const value_t &msg);

  /// Pop one queued message for `path`, or nullopt if the queue
  /// is empty.  Called by `msg-recv` before suspending.
  std::optional<value_t> pop_pending_message(const std::string &path);

  /// Number of queued (undelivered) messages for `path`.
  std::size_t pending_message_count(const std::string &path) const;

  /// Total queued messages across all paths.
  std::size_t total_pending_messages() const;

  /// Maximum messages that can be queued per path (0 = unlimited).
  std::size_t max_pending_messages = 1024;

  /// Fork a running/ready process.  Returns child PID or -1 on failure.
  int fork(int pid);

  bool set_priority(int pid, int priority);
  bool set_max_steps(int pid, uint64_t max_steps);
  bool set_max_time(int pid, double seconds);
  bool set_max_memory(int pid, uint64_t bytes);
  bool set_max_messages(int pid, uint64_t count);
  bool set_max_message_bytes(int pid, uint64_t bytes);

  // --- Signal handling ---

  /// Send a signal to a process.  SIGKILL kills immediately.
  bool send_signal(int pid, const std::string &signal);

  // --- Process info ---

  std::vector<process_info> list_processes() const;
  std::optional<process_info> get_process_info(int pid) const;
  std::optional<value_t> get_result(int pid) const;
  std::unordered_map<int, value_t> get_results() const;
  scheduler_stats get_stats() const;

  // --- Accessors ---

  scheduling_policy policy() const { return policy_; }
  const memory_tracker &mem_tracker() const { return mem_tracker_; }
  int process_count() const { return static_cast<int>(processes_.size()); }

  /// Check if any process is still runnable.
  bool has_runnable() const;

  /// PID of the process currently being stepped (-1 if none).
  int current_pid() const { return current_pid_; }

  /// Process pointer of the process currently being stepped (nullptr if none).
  const process_ptr &current_process() const { return current_proc_; }

  /// Queue a watch event for a process by PID.
  void queue_watch_event(int pid, process::watch_event evt);

  /// Register a watch handler with its watched path on the scheduler's process.
  void register_watch_handler(int pid, int watch_id, value_t handler, const std::string &path);

  /// Remove a watch handler from the scheduler's process object.
  void unregister_watch_handler(int pid, int watch_id);

  /// Set the state-tree root for watch polling.
  void set_watch_root(cvc::state *root) { watch_root_ = root; }

  /// Set a unique identifier for this scheduler instance.
  /// Used as the key under `state_exec.schedulers.<id>` in the state tree.
  void set_id(const std::string &id) { id_ = id; }
  const std::string &id() const { return id_; }

  /// Load settings from the state tree.
  ///
  /// Resolution order (per setting):
  ///   1. Per-scheduler: `state_exec.schedulers.<id>.<key>` (if id set)
  ///   2. Global default: `state_exec.defaults.<key>`
  ///   3. Hardcoded fallback
  ///
  /// After resolution, effective values are published back to the
  /// per-scheduler subtree so they are visible in the state tree.
  /// Requires set_watch_root() and (for per-scheduler) set_id() first.
  void load_settings();

  /// Well-known state tree paths for scheduler configuration.
  static constexpr const char *settings_root   = "state_exec";
  static constexpr const char *defaults_prefix = "state_exec.defaults";
  static constexpr const char *sched_prefix    = "state_exec.schedulers";

private:
  std::string id_;
  scheduling_policy policy_;
  int next_pid_ = 1;
  int rr_index_ = 0;
  bool running_ = false;
  bool stop_requested_ = false;
  uint64_t total_steps_ = 0;
  int current_pid_ = -1;
  process_ptr current_proc_;

  stackless_evaluator evaluator_;
  memory_tracker mem_tracker_;
  cvc::state *watch_root_ = nullptr;

  std::unordered_map<int, process_ptr> processes_;

  /// Per-path FIFO queue for messages sent when no receiver is waiting.
  std::unordered_map<std::string, std::queue<value_t>> pending_messages_;

  /// Select the next process to run based on scheduling policy.
  process_ptr select_process();

  /// Execute one step on a specific process and check resource limits.
  void execute_process_step(process &proc);

  /// Handle pending signals for a process.
  void handle_signal(process &proc);

  /// Handle pending watch events for a process.
  void handle_watch_event(process &proc);

  /// Restore process state after signal/watch handler completes.
  void restore_from_signal(process &proc);

  /// Poll all watch_handlers entries for value changes, queuing events.
  void poll_watches();

  /// Wake sleeping processes whose deadline has passed.
  void wake_sleeping_processes();

  /// Check resource limits after a step.  May kill the process.
  void check_limits(process &proc);

  /// Mark a process as terminated with a result.
  void terminate_process(process &proc, value_t result);

  /// Mark a process as killed with an error.
  void kill_process(process &proc, const std::string &reason);

  /// Build process_info from a process.
  process_info make_info(const process &proc) const;
};

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_SCHEDULER_H
