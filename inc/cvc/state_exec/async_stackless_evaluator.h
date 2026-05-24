#ifndef CVC_STATE_EXEC_ASYNC_STACKLESS_EVALUATOR_H
#define CVC_STATE_EXEC_ASYNC_STACKLESS_EVALUATOR_H

#include <cvc/state_exec/stackless_evaluator.h>
#include <cvc/state_exec/task.h>

#include <functional>
#include <optional>
#include <string>

namespace cvc::state_exec {

/// Asynchronous stackless evaluator — the mandatory evaluator for all
/// scheduler-managed processes.
///
/// Wraps `stackless_evaluator` and exposes coroutine-returning methods
/// so the scheduler can cooperatively interleave process steps without
/// blocking.  The `evaluator_state` is identical to the sync version
/// and fully serializable for pause/resume and cross-node migration.
///
/// Key differences from `stackless_evaluator`:
///   - `step()` returns `task<bool>` — yields after each step
///   - `run()` returns `task<value_t>` — yields between steps
///   - `(await expr)` form adds `await_result` phase
class async_stackless_evaluator {
public:
    explicit async_stackless_evaluator(environment_ptr global_env);

    /// Create a fresh evaluator_state (delegates to inner evaluator).
    evaluator_state create_state(const value_t& expr,
                                 environment_ptr env = nullptr);
    evaluator_state create_state(const std::string& script,
                                 environment_ptr env = nullptr);

    /// Single step as a coroutine — yields after the step.
    task<bool> step(evaluator_state& state);

    /// Run until done/limits as a coroutine — yields between steps.
    task<value_t> run(evaluator_state& state,
                      std::optional<uint64_t> max_steps = std::nullopt,
                      std::optional<double> timeout_sec = std::nullopt,
                      std::function<void(const value_t&)> on_complete = nullptr);

    /// Blocking convenience wrappers.
    value_t sync_evaluate(const value_t& expr,
                          environment_ptr env = nullptr,
                          std::optional<double> timeout_sec = std::nullopt,
                          std::function<void(const value_t&)> on_complete = nullptr);
    value_t sync_evaluate_script(const std::string& script,
                                 environment_ptr env = nullptr,
                                 std::optional<double> timeout_sec = std::nullopt,
                                 std::function<void(const value_t&)> on_complete = nullptr);

    void interrupt();
    void reset_interrupt();
    void pause();
    void resume();
    bool is_paused() const;

private:
    stackless_evaluator inner_;
};

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_ASYNC_STACKLESS_EVALUATOR_H
