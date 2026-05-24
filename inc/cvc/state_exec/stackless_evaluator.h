/**
 * @file stackless_evaluator.h
 * @brief Stackless (continuation-based) evaluator for the state_exec DSL.
 *
 * Evaluates parsed ASTs without C++ call-stack recursion by maintaining
 * an explicit continuation stack.  Supports step-limited execution,
 * pause/resume, timeout, tail-call optimisation, and full state
 * serialisation for process migration.
 */
#ifndef CVC_STATE_EXEC_STACKLESS_EVALUATOR_H
#define CVC_STATE_EXEC_STACKLESS_EVALUATOR_H

#include <cvc/state_exec/evaluator.h>
#include <cvc/state_exec/types.h>

#include <any>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace cvc::state_exec {

/// Phase of evaluation for a single frame.
enum class eval_phase {
    init,
    eval_args,
    apply,
    if_cond,
    if_branch,
    begin_next,
    while_cond,
    while_body,
    for_body,
    set_value,
    let_bindings,
    let_body,
    return_value,
    eval_inner,
    super_args,
    defclass_methods,
};

/// A single evaluation frame on the explicit stack.
struct eval_frame {
    value_t             expr;
    environment_ptr     env;
    eval_phase          phase  = eval_phase::init;
    int64_t             index  = 0;
    std::vector<value_t> results;
    // Extra state for complex forms (keyed by name; avoids map overhead)
    std::vector<value_t>                    extra_vals;
    std::vector<std::string>                extra_strs;
    std::vector<std::vector<value_t>>       extra_lists;
    environment_ptr                         extra_env;
};

/// Serializable evaluator state — the entire execution snapshot.
struct evaluator_state {
    std::vector<eval_frame> stack;
    value_t                 result;
    bool                    done       = false;
    value_t                 root_expr;
    environment_ptr         global_env;
    evaluator::macro_map    user_macros;
    evaluation_stats        stats;
};

/// Synchronous stackless evaluator (Tier 2 — local contexts only).
///
/// Uses an explicit stack of eval_frame objects instead of the C++ call stack.
/// The evaluator_state is fully serializable and can be paused / resumed.
///
/// Key API:
///   create_state()  → evaluator_state
///   step()          → bool (true when done)
///   run()           → value_t (runs until done or limit)
///   evaluate()      → value_t (convenience wrapper)
class stackless_evaluator {
public:
    explicit stackless_evaluator(environment_ptr global_env);

    /// Create a fresh evaluator_state from a parsed expression.
    evaluator_state create_state(const value_t& expr,
                                 environment_ptr env = nullptr);

    /// Create a fresh evaluator_state from a script string.
    evaluator_state create_state(const std::string& script,
                                 environment_ptr env = nullptr);

    /// Execute one evaluation step.  Returns true when done.
    bool step(evaluator_state& state);

    /// Run until done or limits reached.  Returns result or nil if not done.
    value_t run(evaluator_state& state,
                std::optional<uint64_t> max_steps = std::nullopt,
                std::optional<double> timeout_sec = std::nullopt,
                std::function<void(const value_t&)> on_complete = nullptr);

    /// Convenience: create + run to completion.
    value_t evaluate(const value_t& expr,
                     environment_ptr env = nullptr,
                     std::optional<double> timeout_sec = std::nullopt,
                     std::function<void(const value_t&)> on_complete = nullptr);

    /// Convenience: parse + create + run.
    value_t evaluate_script(const std::string& script,
                            environment_ptr env = nullptr,
                            std::optional<double> timeout_sec = std::nullopt,
                            std::function<void(const value_t&)> on_complete = nullptr);

    void interrupt();
    void reset_interrupt();
    void pause();
    void resume();
    bool is_paused() const;

private:
    void check_interrupted();

    // Frame management
    void pop_frame(evaluator_state& state, value_t result);
    void push_frame(evaluator_state& state, value_t expr,
                    environment_ptr env);

    // Step implementations — one per phase
    void step_init(evaluator_state& state, eval_frame& frame);
    void step_eval_args(evaluator_state& state, eval_frame& frame);
    void step_apply(evaluator_state& state, eval_frame& frame);
    void step_if_cond(evaluator_state& state, eval_frame& frame);
    void step_if_branch(evaluator_state& state, eval_frame& frame);
    void step_begin_next(evaluator_state& state, eval_frame& frame);
    void step_while_cond(evaluator_state& state, eval_frame& frame);
    void step_while_body(evaluator_state& state, eval_frame& frame);
    void step_for_body(evaluator_state& state, eval_frame& frame);
    void step_set_value(evaluator_state& state, eval_frame& frame);
    void step_let_bindings(evaluator_state& state, eval_frame& frame);
    void step_let_body(evaluator_state& state, eval_frame& frame);
    void step_return_value(evaluator_state& state, eval_frame& frame);
    void step_eval_inner(evaluator_state& state, eval_frame& frame);
    void step_super_args(evaluator_state& state, eval_frame& frame);

    // Helpers
    void call_closure(evaluator_state& state, eval_frame& frame,
                      const closure_ptr& cls, const std::vector<value_t>& args);
    void handle_defclass(evaluator_state& state, eval_frame& frame,
                         const std::vector<value_t>& args);
    value_t substitute(const value_t& tmpl,
                       const std::unordered_map<std::string, value_t>& subst);

    environment_ptr global_env_;

    std::atomic<bool> interrupted_{false};
    std::atomic<bool> paused_{false};
    mutable std::mutex pause_mu_;
    std::condition_variable pause_cv_;
    std::recursive_mutex eval_mu_;
};

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_STACKLESS_EVALUATOR_H
