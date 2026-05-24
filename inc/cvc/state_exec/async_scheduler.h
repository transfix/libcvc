#ifndef CVC_STATE_EXEC_ASYNC_SCHEDULER_H
#define CVC_STATE_EXEC_ASYNC_SCHEDULER_H

#include <cvc/state_exec/async_stackless_evaluator.h>
#include <cvc/state_exec/memory_tracker.h>
#include <cvc/state_exec/process.h>
#include <cvc/state_exec/scheduler.h>
#include <cvc/state_exec/task.h>
#include <cvc/state_exec/types.h>

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace cvc::state_exec {

/// Async process scheduler using C++20 coroutines.
///
/// Same semantics as the sync scheduler but all stepping methods are
/// coroutines that yield between steps via suspend_point, enabling
/// cooperative multitasking in a coroutine executor.
class async_scheduler {
public:
    explicit async_scheduler(
        scheduling_policy policy = scheduling_policy::round_robin);

    // --- Process submission ---

    int execute(const std::string& script,
                const execute_options& opts = {});

    int execute(const value_t& expr,
                const execute_options& opts = {});

    // --- Stepping (coroutines) ---

    /// Execute one step as a coroutine.
    task<int> step();

    /// Run until all done or limits.
    task<std::unordered_map<int, value_t>> run(
        std::optional<uint64_t> max_steps = std::nullopt,
        std::optional<double>   max_time  = std::nullopt);

    /// Blocking wrappers.
    int sync_step();
    std::unordered_map<int, value_t> sync_run(
        std::optional<uint64_t> max_steps = std::nullopt,
        std::optional<double>   max_time  = std::nullopt);

    void stop();
    bool is_running() const { return running_; }

    // --- Process control ---

    bool pause(int pid);
    bool resume(int pid);
    bool kill(int pid);
    int  fork(int pid);

    bool set_priority(int pid, int priority);
    bool set_max_steps(int pid, uint64_t max_steps);
    bool set_max_time(int pid, double seconds);
    bool set_max_memory(int pid, uint64_t bytes);
    bool set_max_messages(int pid, uint64_t count);

    // --- Signal handling ---

    bool send_signal(int pid, const std::string& signal);

    // --- Process info ---

    std::vector<process_info> list_processes() const;
    std::optional<process_info> get_process_info(int pid) const;
    std::optional<value_t> get_result(int pid) const;
    std::unordered_map<int, value_t> get_results() const;
    scheduler_stats get_stats() const;

    // --- Accessors ---

    scheduling_policy policy() const { return policy_; }
    const memory_tracker& mem_tracker() const { return mem_tracker_; }
    int process_count() const { return static_cast<int>(processes_.size()); }
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

    std::unordered_map<int, process> processes_;

    process* select_process();
    void execute_process_step(process& proc);
    void handle_signal(process& proc);
    void restore_from_signal(process& proc);
    void check_limits(process& proc);
    void terminate_process(process& proc, value_t result);
    void kill_process(process& proc, const std::string& reason);
    process_info make_info(const process& proc) const;
};

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_ASYNC_SCHEDULER_H
