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
#include <cvc/state_exec/memory_tracker.h>
#include <cvc/state_exec/process.h>
#include <cvc/state_exec/stackless_evaluator.h>
#include <cvc/state_exec/types.h>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

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
  uint64_t max_steps = 0;
  double max_time = 0.0;
  uint64_t max_memory = 0;
  uint64_t max_messages = 0;
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

  /// Fork a running/ready process.  Returns child PID or -1 on failure.
  int fork(int pid);

  bool set_priority(int pid, int priority);
  bool set_max_steps(int pid, uint64_t max_steps);
  bool set_max_time(int pid, double seconds);
  bool set_max_memory(int pid, uint64_t bytes);
  bool set_max_messages(int pid, uint64_t count);

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

private:
  scheduling_policy policy_;
  int next_pid_ = 1;
  int rr_index_ = 0;
  bool running_ = false;
  bool stop_requested_ = false;
  uint64_t total_steps_ = 0;

  stackless_evaluator evaluator_;
  memory_tracker mem_tracker_;

  std::unordered_map<int, process_ptr> processes_;

  /// Select the next process to run based on scheduling policy.
  process_ptr select_process();

  /// Execute one step on a specific process and check resource limits.
  void execute_process_step(process &proc);

  /// Handle pending signals for a process.
  void handle_signal(process &proc);

  /// Restore process state after signal handler completes.
  void restore_from_signal(process &proc);

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
