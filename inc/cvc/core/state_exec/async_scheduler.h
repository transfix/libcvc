/**
 * @file async_scheduler.h
 * @brief Coroutine-enabled scheduler using async_stackless_evaluator.
 *
 * Extends the synchronous scheduler with task<T> coroutine integration,
 * enabling cooperative multitasking via co_await.  Each run_one() call
 * advances a single process by one time slice and returns a task<T>.
 */
#ifndef CVC_STATE_EXEC_ASYNC_SCHEDULER_H
#define CVC_STATE_EXEC_ASYNC_SCHEDULER_H

#include <cvc/core/state_exec/async_stackless_evaluator.h>
#include <cvc/core/state_exec/memory_tracker.h>
#include <cvc/core/state_exec/process.h>
#include <cvc/core/state_exec/scheduler.h>
#include <cvc/core/state_exec/scheduler_base.h>
#include <cvc/core/state_exec/task.h>
#include <cvc/core/state_exec/types.h>
#include <optional>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

namespace cvc {
class state;
}

namespace cvc::state_exec {

/// Async process scheduler using C++20 coroutines.
///
/// Same semantics as the sync scheduler but all stepping methods are
/// coroutines that yield between steps via suspend_point, enabling
/// cooperative multitasking in a coroutine executor.
///
/// Implements scheduler_base — the intrinsic-facing surface — so the DSL
/// intrinsics (spawn/sleep/msg-send/msg-recv/self/ps/kill/pause/resume + the
/// state-watch registry) drive it identically to the sync scheduler.
class async_scheduler : public scheduler_base {
public:
  explicit async_scheduler(scheduling_policy policy = scheduling_policy::round_robin);

  // --- Process submission ---

  int execute(const std::string &script, const execute_options &opts = {}) override;

  int execute(const value_t &expr, const execute_options &opts = {}) override;

  // --- Stepping (coroutines) ---

  /// Execute one step as a coroutine.
  task<int> step();

  /// Run until all done or limits.
  task<std::unordered_map<int, value_t>> run(std::optional<uint64_t> max_steps = std::nullopt,
                                             std::optional<double> max_time = std::nullopt);

  /// Blocking wrappers.
  int sync_step();
  std::unordered_map<int, value_t> sync_run(std::optional<uint64_t> max_steps = std::nullopt,
                                            std::optional<double> max_time = std::nullopt);

  void stop();
  bool is_running() const { return running_; }

  // --- Process control ---

  bool pause(int pid) override;
  bool resume(int pid) override;
  bool kill(int pid) override;
  int fork(int pid) override;

  /// Cooperative sleep: put a process into `waiting`; woken by the pump
  /// (wake_sleeping_processes, called at the top of step()) once the deadline
  /// passes.  Identical semantics to the sync scheduler.
  bool sleep(int pid, double seconds) override;

  // --- Inter-process messaging (msg-send/msg-recv) ---
  bool receive_message(int pid, const std::string &path) override;
  int deliver_to_receivers(const std::string &path, const value_t &msg) override;
  std::optional<value_t> pop_pending_message(const std::string &path) override;
  std::size_t pending_message_count(const std::string &path) const override;
  std::size_t total_pending_messages() const;
  /// Max messages queued per path when no receiver waits (0 = unlimited).
  std::size_t max_pending_messages = 1024;

  bool set_priority(int pid, int priority);
  bool set_max_steps(int pid, uint64_t max_steps);
  bool set_max_time(int pid, double seconds);
  bool set_max_memory(int pid, uint64_t bytes);
  bool set_max_messages(int pid, uint64_t count);
  bool set_max_message_bytes(int pid, uint64_t bytes);

  // --- Signal handling ---

  bool send_signal(int pid, const std::string &signal);

  // --- Process info ---

  std::vector<process_info> list_processes() const override;
  std::optional<process_info> get_process_info(int pid) const override;
  std::optional<value_t> get_result(int pid) const;
  std::unordered_map<int, value_t> get_results() const;
  scheduler_stats get_stats() const override;

  // --- Accessors ---

  scheduling_policy policy() const { return policy_; }
  const memory_tracker &mem_tracker() const { return mem_tracker_; }
  int process_count() const { return static_cast<int>(processes_.size()); }
  bool has_runnable() const;

  /// PID / process pointer of the process currently being stepped (-1 / nullptr
  /// if none).  Set around evaluator_.step() so intrinsics can query `self`.
  int current_pid() const override { return current_pid_; }
  const process_ptr &current_process() const override { return current_proc_; }

  void queue_watch_event(int pid, process::watch_event evt);

  void register_watch_handler(int pid, int watch_id, value_t handler,
                              const std::string &path) override;
  void unregister_watch_handler(int pid, int watch_id) override;

  void set_watch_root(cvc::state *root) override { watch_root_ = root; }

private:
  scheduling_policy policy_;
  int next_pid_ = 1;
  int rr_index_ = 0;
  bool running_ = false;
  bool stop_requested_ = false;
  uint64_t total_steps_ = 0;

  int current_pid_ = -1;
  process_ptr current_proc_;

  // The scheduled evaluator IS the async_stackless_evaluator (roadmap §8.12): a coroutine
  // wrapper whose step() yields a suspend_point, so the scheduler's task<> step chain can
  // interleave processes cooperatively when driven from a coroutine executor.
  async_stackless_evaluator evaluator_;
  memory_tracker mem_tracker_;
  cvc::state *watch_root_ = nullptr;

  std::unordered_map<int, process_ptr> processes_;

  /// Per-path FIFO for messages sent when no receiver is waiting (mirrors sync scheduler).
  std::unordered_map<std::string, std::queue<value_t>> pending_messages_;

  process_ptr select_process();
  void execute_process_step(process &proc);
  void handle_signal(process &proc);
  void handle_watch_event(process &proc);
  void restore_from_signal(process &proc);
  void poll_watches();
  void wake_sleeping_processes();
  void check_limits(process &proc);
  void terminate_process(process &proc, value_t result);
  void kill_process(process &proc, const std::string &reason);
  process_info make_info(const process &proc) const;
};

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_ASYNC_SCHEDULER_H
