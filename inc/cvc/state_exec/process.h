#ifndef CVC_STATE_EXEC_PROCESS_H
#define CVC_STATE_EXEC_PROCESS_H

#include <cvc/state_exec/stackless_evaluator.h>
#include <cvc/state_exec/types.h>

#include <chrono>
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
    ready,       // Waiting to run
    running,     // Currently being stepped
    paused,      // Paused by user
    waiting,     // Waiting for signal or I/O
    terminated,  // Finished execution normally
    killed       // Killed by user or resource limit
};

/// Convert process_status to string.
const char* to_string(process_status s);

/// Parse process_status from string.  Returns ready on unknown input.
process_status parse_process_status(const std::string& s);

/// A managed process in the scheduler.
struct process {
    int                 pid = -1;
    std::string         name;
    process_status      status     = process_status::ready;

    // Scheduling
    int                 priority   = 0;   // Nice value: -20 (high) to +19 (low)
    std::string         uid;              // User identity
    std::string         gid;              // Group identity

    // Resource limits (0 = unlimited)
    uint64_t            max_steps    = 0;
    double              max_time     = 0.0;   // Wall-clock seconds
    uint64_t            max_memory   = 0;     // Bytes
    uint64_t            max_messages = 0;     // Outbound message count

    // Evaluator state (stackless — serializable)
    evaluator_state     state;

    // Timing
    std::chrono::steady_clock::time_point create_time;
    double              accumulated_time = 0.0;  // Seconds running so far
    std::chrono::steady_clock::time_point last_run_start;

    // Completion
    value_t             exit_code;
    std::optional<std::string> exit_error;
    std::function<void(value_t)> on_complete;

    // Signal handling
    std::unordered_map<std::string, value_t> signal_handlers;
    std::vector<std::string> pending_signals;
    bool                in_signal_handler = false;
    std::optional<eval_frame> saved_frame;

    // Message tracking
    uint64_t            message_count = 0;

    // Forking
    int                 parent_pid = -1;

    /// Elapsed running time (only while status == running).
    double elapsed_time() const;

    /// Step count from evaluator stats.
    uint64_t step_count() const {
        return state.stats.get_step_count();
    }
};

/// Shared ownership alias for process.
using process_ptr = std::shared_ptr<process>;

/// Create a new managed process.
inline process_ptr make_process() { return std::make_shared<process>(); }

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_PROCESS_H
