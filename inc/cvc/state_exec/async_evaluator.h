/**
 * @file async_evaluator.h
 * @brief Coroutine-based async wrapper around the recursive evaluator.
 *
 * Wraps the tree-walking evaluator in a task<T> coroutine interface,
 * yielding control at each step boundary.  Primarily useful for testing
 * and simple async scenarios; production code should prefer
 * async_stackless_evaluator.
 */
#ifndef CVC_STATE_EXEC_ASYNC_EVALUATOR_H
#define CVC_STATE_EXEC_ASYNC_EVALUATOR_H

#include <atomic>
#include <cvc/state_exec/evaluator.h>
#include <cvc/state_exec/task.h>
#include <cvc/state_exec/types.h>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace cvc::state_exec {

/// Asynchronous recursive evaluator using C++20 coroutines.
///
/// Mirrors the sync `evaluator` API but every eval method is a coroutine
/// returning `task<value_t>`.  Supports cooperative yielding via
/// `co_await suspend_point{}` at interruption check points and an
/// `(await expr)` special form for user-level suspension.
///
/// Use `sync_evaluate()` / `sync_evaluate_script()` for blocking calls
/// or `co_await` the task<value_t> from `evaluate()` in an async context.
class async_evaluator {
public:
  explicit async_evaluator(environment_ptr global_env);

  /// Evaluate a parsed expression (coroutine).
  task<value_t> evaluate(const value_t &expr, environment_ptr env = nullptr);

  /// Parse + evaluate a script string (coroutine).
  task<value_t> evaluate_script(const std::string &script, environment_ptr env = nullptr);

  /// Blocking wrappers that run the coroutine to completion.
  value_t sync_evaluate(const value_t &expr, environment_ptr env = nullptr,
                        std::optional<double> timeout_sec = std::nullopt,
                        std::function<void(const value_t &)> on_complete = nullptr);

  value_t sync_evaluate_script(const std::string &script, environment_ptr env = nullptr,
                               std::optional<double> timeout_sec = std::nullopt,
                               std::function<void(const value_t &)> on_complete = nullptr);

  void interrupt();
  void reset_interrupt();
  void pause();
  void resume();
  bool is_paused() const;

  evaluation_stats &stats() { return stats_; }
  const evaluation_stats &stats() const { return stats_; }

  using macro_map = evaluator::macro_map;
  macro_map &user_macros() { return user_macros_; }

private:
  task<value_t> eval_internal(const value_t &expr, environment_ptr env);
  task<void> check_interrupted_async();

  // Special form handlers (coroutines)
  task<value_t> do_if(const std::vector<value_t> &args, environment_ptr env);
  task<value_t> do_begin(const std::vector<value_t> &args, environment_ptr env);
  task<value_t> do_while(const std::vector<value_t> &args, environment_ptr env);
  task<value_t> do_for(const std::vector<value_t> &args, environment_ptr env);
  task<value_t> do_set(const std::vector<value_t> &args, environment_ptr env);
  task<value_t> do_quote(const std::vector<value_t> &args);
  task<value_t> do_lambda(const std::vector<value_t> &args, environment_ptr env);
  task<value_t> do_return(const std::vector<value_t> &args, environment_ptr env);
  task<value_t> do_let(const std::vector<value_t> &args, environment_ptr env);
  task<value_t> do_super(const std::vector<value_t> &args, environment_ptr env);
  task<value_t> do_defun(const std::vector<value_t> &args, environment_ptr env);
  task<value_t> do_defclass(const std::vector<value_t> &args, environment_ptr env);
  task<value_t> do_defmacro(const std::vector<value_t> &args, environment_ptr env);
  task<value_t> do_eval(const std::vector<value_t> &args, environment_ptr env);
  task<value_t> do_root();
  task<value_t> do_await(const std::vector<value_t> &args, environment_ptr env);

  task<value_t> apply_closure(const closure_ptr &cls, const std::vector<value_t> &args);
  value_t substitute(const value_t &tmpl, const std::unordered_map<std::string, value_t> &subst);

  environment_ptr global_env_;
  value_t root_expr_;
  macro_map user_macros_;
  evaluation_stats stats_;

  std::atomic<bool> interrupted_{false};
  std::atomic<bool> paused_{false};
  std::mutex pause_mu_;
  std::condition_variable pause_cv_;
  std::recursive_mutex eval_mu_;
};

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_ASYNC_EVALUATOR_H
