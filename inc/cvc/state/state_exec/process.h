/**
 * @file process.h
 * @brief Process abstraction for scheduled DSL execution.
 *
 * A process wraps a DSL program with its own PID, environment, message
 * queue, execution state (ready/running/blocked/terminated), and
 * per-process resource limits (max steps, timeout, max memory).
 */
#ifndef CVC_STATE_EXEC_PROCESS_H
#define CVC_STATE_EXEC_PROCESS_H

#include <algorithm>
#include <chrono>
#include <cvc/state/state_exec/stackless_evaluator.h>
#include <cvc/state/state_exec/types.h>
#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

namespace cvc::state_exec {

/// Status of a managed process.
enum class process_status {
  ready,      // Waiting to run
  running,    // Currently being stepped
  paused,     // Paused by user
  waiting,    // Waiting for signal or I/O
  terminated, // Finished execution normally
  killed,     // Killed by user or resource limit
  failed      // Ended by an exception the program didn't catch (docs/STATE_EXEC_EXCEPTIONS.md)
};

/// Whether a process with status `s` has ended: terminated, killed or failed.
inline bool is_finished(process_status s) {
  return s == process_status::terminated || s == process_status::killed ||
         s == process_status::failed;
}

/// Convert process_status to string.
const char *to_string(process_status s);

/// Parse process_status from string.  Returns ready on unknown input.
process_status parse_process_status(const std::string &s);

/// A managed process in the scheduler.
struct process {
  int pid = -1;
  std::string name;
  process_status status = process_status::ready;

  // Scheduling
  int priority = 0;            // Nice value: -20 (high) to +19 (low)
  std::string uid;             // User identity
  std::string gid;             // Group identity
  std::string root_path;       // Chroot path (empty = full tree)
  std::string owner;           // Owner-scope tag (e.g. an Ariadne document/Runtime); "" = unowned.
                               // Lets a host reap a whole process group on teardown (kill_owner).
  bool awaiting_frame = false; // (await expr) parked this process until the next frame boundary;
                               // re-readied by async_scheduler::wake_awaiting() once per pump.

  // Resource limits (0 = unlimited)
  uint64_t max_steps = 0;
  double max_time = 0.0;          // Seconds (TOTAL run-time, all activations): see time_left()
  uint64_t max_memory = 0;        // Bytes
  uint64_t max_messages = 0;      // Outbound message count
  uint64_t max_message_bytes = 0; // Total outbound message bytes
  // §13.8 per-STEP time cap (0 = unlimited): the deadline armed for a SINGLE evaluator step,
  // fresh each step (never accumulates), so a long-lived resident with total max_time == 0 still
  // can never HANG the host's per-frame drain — a single step that loops in-place (e.g. a runaway
  // native builtin) is aborted at the cap and the process killed, while a healthy step (far under
  // the cap) and a normal event-loop are untouched. (A per-node DSL loop yields per step, so the
  // pump's own per-drain cap bounds it; this cap is the backstop for an in-STEP loop the pump can't
  // interrupt.)
  double max_step_time = 0.0;
  // The most a single native call may allocate (0 = no limit of its own): the tree setting
  // limits.max_alloc_bytes, resolved when the process is submitted. See alloc_limit().
  uint64_t max_alloc_bytes = kDefaultMaxAllocBytes;

  // Heap accounting (heap.h), for a process with max_memory > 0: max_memory covers its state-tree
  // writes plus heap_bytes, its measured share of the values it can reach. A measurement runs as
  // the process's own data steps; heap_alloc_bytes (what natives reported allocating) and
  // heap_steps since the last one decide when the next is due.
  uint64_t heap_bytes = 0;
  uint64_t heap_alloc_bytes = 0;
  uint64_t heap_steps = 0;
  uint64_t heap_measurements = 0;
  std::size_t heap_check_steps = 4096; // limits.heap_check_steps, resolved at submission
  bool heap_measuring = false;         // a measurement is in proc.state.data_work

  // Evaluator state (stackless — serializable)
  evaluator_state state;

  // Timing
  std::chrono::steady_clock::time_point create_time;
  double accumulated_time = 0.0; // Seconds running so far (TOTAL run-time; parked time uncounted)
  // The second half of max_time used so far: see charge_cpu_time().
  double cpu_time_charged = 0.0;
  std::chrono::steady_clock::time_point last_run_start;

  // Sleep support: process is in `waiting` status until this deadline.
  std::optional<std::chrono::steady_clock::time_point> wake_time;

  // Message receive support: process is in `waiting` status until a
  // message arrives at this state-tree path.  When msg-send delivers
  // to the matching path the scheduler pushes a dict into `inbox`
  // and wakes the process.
  std::optional<std::string> recv_path;
  std::queue<value_t> inbox;

  // Completion
  value_t exit_code;
  std::optional<std::string> exit_error;
  /// A failed process's exception (exit_error is its "Class: message"), and the C++ exception it
  /// was made from, if it was (a native's, or a Python function's): a host may rethrow that.
  std::optional<value_t> exit_exception;
  std::exception_ptr exit_original;
  std::function<void(value_t)> on_complete;

  // Signal handling
  std::unordered_map<std::string, value_t> signal_handlers;
  std::vector<std::string> pending_signals;
  bool in_signal_handler = false;
  bool in_watch_handler = false;
  std::vector<eval_frame> saved_stack;
  value_t saved_result;

  // State-watch events (queued when a watched path's value changes).
  struct watch_event {
    int watch_id;
  };
  std::vector<watch_event> pending_watch_events;
  int next_watch_id = 1;

  // Watch handler registry: watch_id -> {handler, path, last_value}.
  // The scheduler polls watched paths each step instead of using
  // boost::signals2, which avoids ABI issues on macOS / Apple Clang.
  struct watch_handler_entry {
    value_t handler;
    std::string path;
    std::string last_value; // snapshot for change detection
  };
  std::unordered_map<int, watch_handler_entry> watch_handlers;

  // Message tracking
  uint64_t message_count = 0;
  uint64_t message_bytes = 0;

  // Forking
  int parent_pid = -1;

  /// Elapsed running time (only while status == running).
  double elapsed_time() const;

  /// max_time is charged like a time_budget: elapsed_time() up to half of it, then the CPU
  /// time of the steps run from there (cpu_time_charged) for the other half, each step charging
  /// at most a sixteenth of it -- so a stall in a step, however long (descheduled on a loaded
  /// machine), never runs a process out of time. Whether the next step's CPU time is charged:
  bool charges_cpu_time() const { return max_time > 0.0 && elapsed_time() >= 0.5 * max_time; }
  /// Charge a step's CPU time.
  void charge_cpu_time(double seconds) { cpu_time_charged += std::min(seconds, max_time / 32.0); }
  /// Whether max_time is used up (never, when it is 0: unlimited).
  bool over_time_limit() const { return charges_cpu_time() && cpu_time_charged >= 0.5 * max_time; }
  /// Seconds of max_time left (nullopt when unlimited): the budget of the next step's deadline.
  std::optional<double> time_left() const;

  /// The allocation limit for the next step's eval_alloc_guard (nullopt when unlimited): the
  /// tighter of max_alloc_bytes and what is left of max_memory once `used_bytes` (the memory
  /// tracker's count for this process plus heap_bytes) is taken out.
  std::optional<std::size_t> alloc_limit(std::size_t used_bytes) const;

  /// Whether to start a heap measurement after this step: the process has a memory budget, none
  /// is running, and natives reported enough allocation (heap_remeasure_bytes), or the next
  /// allocation could cross the budget, or limits.heap_check_steps steps passed.
  /// `state_bytes` is the memory tracker's count for this process.
  bool heap_measure_due(uint64_t state_bytes) const;

  /// Step count from evaluator stats.
  uint64_t step_count() const { return state.stats.get_step_count(); }

  /// Drop what a finished process evaluated with (its stack, environment, program, handlers and
  /// inbox), keeping its result, error and statistics. A scheduler calls it once the process is
  /// terminated or killed and not running, so what the program left in reference cycles becomes
  /// garbage the cycle collector can free (heap.h), even before the process is reaped.
  void release_state();
};

/// Shared ownership alias for process.
using process_ptr = std::shared_ptr<process>;

/// Create a new managed process.
inline process_ptr make_process() { return std::make_shared<process>(); }

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_PROCESS_H
