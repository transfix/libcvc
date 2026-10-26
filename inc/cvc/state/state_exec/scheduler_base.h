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
  double max_time = 0.0;      // TOTAL run-time seconds (a time_budget: see process::time_left)
  double max_step_time = 0.0; // §13.8 per-STEP time cap (fresh each step); 0 = unlimited
  uint64_t max_memory = 0;
  uint64_t max_messages = 0;
  uint64_t max_message_bytes = 0;
  /// The process's own limits.async_steps_per_cpu_ms (docs/STATE_EXEC_EXCEPTIONS.md 3.8): the
  /// steps a CPU millisecond of its async calls is charged, ahead of the tree's settings.
  /// Unset: the tree's, read at every charge (async_steps_per_cpu_ms_from_state).
  std::optional<double> async_steps_per_cpu_ms;
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
  // What the steps were spent on (step_kind): reading the script a slice at a time
  // (parser.step_tokens), deep-data work a slice at a time (data.step_nodes), evaluating the
  // rest. data_nodes counts the deep-data work in nodes, whichever steps did it; the times are
  // seconds of scheduler-timed steps.
  uint64_t read_steps = 0;
  uint64_t eval_steps = 0;
  uint64_t data_steps = 0;
  uint64_t data_nodes = 0;
  double read_time = 0.0;
  double eval_time = 0.0;
  double data_time = 0.0;
  double elapsed_time;
  uint64_t current_memory;
  uint64_t peak_memory;
  uint64_t heap_memory = 0;       // measured heap share (heap.h); max_memory covers it too
  uint64_t heap_measurements = 0; // measurements taken so far
  uint64_t heap_alloc_bytes = 0;  // what natives and async kernels reported allocating since
  // What its async calls were charged (docs/STATE_EXEC_EXCEPTIONS.md 3.8): calls made, their run
  // time and CPU time (seconds), and the steps (step_kind::async, in step_count too).
  // elapsed_time includes their run time.
  uint64_t async_calls = 0;
  double async_time = 0.0;
  double async_cpu_time = 0.0;
  uint64_t async_steps = 0;
  uint64_t max_memory;
  double max_time;
  uint64_t message_count;
  uint64_t max_messages;
  uint64_t message_bytes;
  uint64_t max_message_bytes;
  int parent_pid;
  /// Why it ended, when it didn't terminate normally: the kill reason, or "Class: message".
  std::optional<std::string> exit_error;
  /// A failed process's exception (docs/STATE_EXEC_EXCEPTIONS.md 2.6).
  std::optional<value_t> exit_exception;
};

/// Fill `info`'s budget-by-kind fields from a process's evaluation stats.
inline void fill_budget_usage(process_info &info, const evaluation_stats &stats) {
  info.read_steps = stats.steps(step_kind::read);
  info.eval_steps = stats.steps(step_kind::eval);
  info.data_steps = stats.steps(step_kind::data);
  info.data_nodes = stats.data_nodes();
  info.read_time = stats.step_time(step_kind::read);
  info.eval_time = stats.step_time(step_kind::eval);
  info.data_time = stats.step_time(step_kind::data);
}

/// Fill `info`'s async-call fields from the process.
inline void fill_async_usage(process_info &info, const process &proc) {
  info.async_calls = proc.async_calls;
  info.async_time = proc.async_time;
  info.async_cpu_time = proc.async_cpu_time;
  info.async_steps = proc.async_steps;
  info.heap_alloc_bytes = proc.heap_alloc_bytes;
}

/// Aggregate scheduler statistics.
struct scheduler_stats {
  int total_processes = 0;
  int running = 0;
  int ready = 0;
  int paused = 0;
  int terminated = 0;
  int killed = 0;
  int failed = 0;
  uint64_t total_steps = 0;
};

/// Abstract scheduler — the surface the DSL intrinsics call through
/// `intrinsics_context.sched`.  Implemented by both `scheduler` (sync) and
/// `async_scheduler` (coroutine).  Every method here is invoked by
/// intrinsics.cpp, or by the async host (async_host.h), which charges a call
/// to the process a scheduler is stepping; nothing else belongs in this
/// interface.
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

  // --- Async calls (await, and the async host: docs/STATE_EXEC_EXCEPTIONS.md 3) ---

  /// receive_message with a time limit: past `seconds`, a call waiting in the await_call phase
  /// raises TimeoutError ((await f seconds)).
  virtual bool receive_message_until(int pid, const std::string &path, double seconds) = 0;
  /// The steps a CPU millisecond of `proc`'s async calls is charged: its own
  /// async_steps_per_cpu_ms, else the tree's (async_steps_per_cpu_ms_from_state), read now.
  virtual double async_steps_per_cpu_ms(const process &proc) = 0;
};

/// limits.async_steps_per_cpu_ms when nothing sets it: what the steps cost, a step being about
/// 0.4 to 0.45 CPU microseconds on the async scheduler and 0.26 to 0.3 on the sync one (a loop of
/// arithmetic, DSL function calls, list and dict work; MSVC Release, x64, 2026-10-24: the
/// StepCostSetsTheDefaultRate case of state_exec_async_calls_test), rounded down.
inline constexpr double kDefaultAsyncStepsPerCpuMs = 2000.0;

/// limits.async_steps_per_cpu_ms from the state tree under `root`:
/// `state_exec.schedulers.<scheduler_id>.limits.async_steps_per_cpu_ms` (when scheduler_id is
/// not empty), then `state_exec.defaults.limits.async_steps_per_cpu_ms`, then
/// kDefaultAsyncStepsPerCpuMs. A node that isn't a non-negative decimal number is skipped.
double async_steps_per_cpu_ms_from_state(cvc::state &root, const std::string &scheduler_id);

/// Publish the effective limits.async_steps_per_cpu_ms under the scheduler's own subtree, at
/// `state_exec.schedulers.<scheduler_id>.effective.async_steps_per_cpu_ms`. Not at its
/// `limits.` key: that one is an input, read ahead of the defaults, so writing the value there
/// would pin it.
void publish_async_steps_per_cpu_ms(cvc::state &root, const std::string &scheduler_id, double rate);

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_SCHEDULER_BASE_H
