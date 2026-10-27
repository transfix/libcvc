/**
 * @file evaluator.h
 * @brief Recursive tree-walking evaluator for the state_exec DSL.
 *
 * Evaluates parsed ASTs using C++ call-stack recursion.  Provides
 * step-limited execution, timeout support, and serializable evaluator
 * state for pause/resume.  The evaluator is the reference implementation;
 * for production use prefer stackless_evaluator or async variants.
 */
#ifndef CVC_STATE_EXEC_EVALUATOR_H
#define CVC_STATE_EXEC_EVALUATOR_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cvc/state/state_exec/types.h>
#include <exception>
#include <functional>
#include <mutex>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace cvc::state_exec {

// evaluation_interrupted and evaluation_timeout are in types.h, next to the evaluation deadline.

/// Exception used internally for (return ...) unwinding.
class return_exception : public std::exception {
public:
  explicit return_exception(value_t val) : value(std::move(val)) {}
  value_t value;
  const char *what() const noexcept override { return "return"; }
};

/// What a step was spent on: evaluating code, reading the script a slice at a time
/// (parser.step_tokens), deep-data work a slice at a time (data.step_nodes: comparing,
/// printing, copying or expanding a nested value, or freeing one), or the CPU time of the
/// process's async calls, converted at limits.async_steps_per_cpu_ms
/// (docs/STATE_EXEC_EXCEPTIONS.md 3.8). Async steps aren't evaluator steps: the host charges
/// them with add_steps().
enum class step_kind { eval = 0, read = 1, data = 2, async = 3 };

/// Thread-safe evaluation statistics.
struct evaluation_stats {
  evaluation_stats() = default;
  // Move-only: transfer ownership of internals
  evaluation_stats(evaluation_stats &&o) noexcept
      : start_time(o.start_time), end_time(o.end_time), complete(o.complete),
        last_kind(o.last_kind) {
    copy_counts(o);
  }
  evaluation_stats &operator=(evaluation_stats &&o) noexcept {
    start_time = o.start_time;
    end_time = o.end_time;
    complete = o.complete;
    last_kind = o.last_kind;
    copy_counts(o);
    return *this;
  }
  evaluation_stats(const evaluation_stats &) = delete;
  evaluation_stats &operator=(const evaluation_stats &) = delete;

  /// Start counting afresh. (Not while an evaluator steps the state on another thread: the
  /// evaluators count their own eval steps without a locked add, see count_eval_step(), so a
  /// reset made meanwhile can be lost.)
  void start() {
    std::lock_guard lk(mu);
    start_time = std::chrono::steady_clock::now();
    step_count.store(0, std::memory_order_relaxed);
    eval_step_count.store(0, std::memory_order_relaxed);
    for (std::size_t k = 0; k < kKinds; ++k) {
      kind_steps[k].store(0, std::memory_order_relaxed);
      kind_ns[k].store(0, std::memory_order_relaxed);
    }
    data_node_count.store(0, std::memory_order_relaxed);
    complete = false;
  }
  void mark_complete() {
    std::lock_guard lk(mu);
    end_time = std::chrono::steady_clock::now();
    complete = true;
  }
  /// Count a step, as spent on `kind`. Any thread may count, and read, at any time.
  void increment_step(step_kind kind = step_kind::eval) {
    if (kind == step_kind::eval) { // eval steps are the rest, so the common step costs no more
      eval_step_count.fetch_add(1, std::memory_order_relaxed);
    } else {
      step_count.fetch_add(1, std::memory_order_relaxed);
      kind_steps[static_cast<std::size_t>(kind)].fetch_add(1, std::memory_order_relaxed);
    }
    last_kind = kind;
  }
  /// Count `n` steps of `kind` that no evaluator step took: what an async call is charged
  /// (step_kind::async), or the counts a restored process had. The last step's kind stays.
  void add_steps(step_kind kind, uint64_t n) {
    if (n == 0)
      return;
    step_count.fetch_add(n, std::memory_order_relaxed);
    if (kind != step_kind::eval)
      kind_steps[static_cast<std::size_t>(kind)].fetch_add(n, std::memory_order_relaxed);
  }
  uint64_t get_step_count() const {
    return step_count.load(std::memory_order_relaxed) +
           eval_step_count.load(std::memory_order_relaxed);
  }
  /// Steps spent on `kind`.
  uint64_t steps(step_kind kind) const {
    if (kind != step_kind::eval)
      return kind_steps[static_cast<std::size_t>(kind)].load(std::memory_order_relaxed);
    const uint64_t other =
        steps(step_kind::read) + steps(step_kind::data) + steps(step_kind::async);
    const uint64_t all = get_step_count();
    return all > other ? all - other : 0;
  }
  /// The kind of the last step counted.
  step_kind last_step_kind() const { return last_kind; }

  /// Count nodes of deep-data work (elements walked, owners freed), in whatever step did it.
  void add_data_nodes(uint64_t nodes) {
    data_node_count.fetch_add(nodes, std::memory_order_relaxed);
  }
  uint64_t data_nodes() const { return data_node_count.load(std::memory_order_relaxed); }

  /// A scheduler that times its steps charges each step's seconds here, to the step's kind.
  void add_step_time(double seconds) {
    if (seconds > 0.0)
      kind_ns[static_cast<std::size_t>(last_kind)].fetch_add(static_cast<uint64_t>(seconds * 1e9),
                                                             std::memory_order_relaxed);
  }
  /// Seconds of scheduler-timed steps spent on `kind`.
  double step_time(step_kind kind) const {
    return 1e-9 * static_cast<double>(
                      kind_ns[static_cast<std::size_t>(kind)].load(std::memory_order_relaxed));
  }
  bool is_complete() const {
    std::lock_guard lk(mu);
    return complete;
  }
  double elapsed_seconds() const {
    std::lock_guard lk(mu);
    auto end = complete ? end_time : std::chrono::steady_clock::now();
    return std::chrono::duration<double>(end - start_time).count();
  }

private:
  friend class evaluator;
  friend class stackless_evaluator;
  /// increment_step(step_kind::eval), by the evaluator stepping the state: the only thread that
  /// counts the state's eval steps while it steps, one step at a time, so the count is a load and
  /// a store of a counter of its own, not a locked read-modify-write (anyone may still read it).
  void count_eval_step() {
    eval_step_count.store(eval_step_count.load(std::memory_order_relaxed) + 1,
                          std::memory_order_relaxed);
    last_kind = step_kind::eval;
  }
  static constexpr std::size_t kKinds = 4;
  void copy_counts(const evaluation_stats &o) {
    step_count.store(o.step_count.load(std::memory_order_relaxed), std::memory_order_relaxed);
    eval_step_count.store(o.eval_step_count.load(std::memory_order_relaxed),
                          std::memory_order_relaxed);
    for (std::size_t k = 0; k < kKinds; ++k) {
      kind_steps[k].store(o.kind_steps[k].load(std::memory_order_relaxed),
                          std::memory_order_relaxed);
      kind_ns[k].store(o.kind_ns[k].load(std::memory_order_relaxed), std::memory_order_relaxed);
    }
    data_node_count.store(o.data_node_count.load(std::memory_order_relaxed),
                          std::memory_order_relaxed);
  }

  mutable std::mutex mu;
  std::chrono::steady_clock::time_point start_time{};
  std::chrono::steady_clock::time_point end_time{};
  std::atomic<uint64_t> step_count{0};      // steps counted by add_steps() or of a kind not eval
  std::atomic<uint64_t> eval_step_count{0}; // eval steps the stepping thread counted
  std::atomic<uint64_t> kind_steps[kKinds] = {};
  std::atomic<uint64_t> kind_ns[kKinds] = {};
  std::atomic<uint64_t> data_node_count{0};
  bool complete{false};
  step_kind last_kind = step_kind::eval;
};

/// Synchronous recursive evaluator (Tier 2 — local contexts only).
///
/// Evaluates S-expressions by recursive descent.  Supports all special forms:
///   if, begin, while, for, set, quote, lambda, return, let, super,
///   defun, defclass, defmacro, eval, root
///
/// Thread-safe: concurrent evaluate() calls are serialized via mutex.
/// Supports interrupt() and pause()/resume() from other threads.
class evaluator {
public:
  /// Create an evaluator with the given global environment.
  /// Typically supply builtins::make_default_environment().
  explicit evaluator(environment_ptr global_env);

  /// Evaluate a parsed expression.
  value_t evaluate(const value_t &expr, environment_ptr env = nullptr,
                   std::optional<double> timeout_sec = std::nullopt,
                   std::function<void(const value_t &)> on_complete = nullptr);

  /// Convenience: parse then evaluate a string.
  value_t evaluate_script(const std::string &script, environment_ptr env = nullptr,
                          std::optional<double> timeout_sec = std::nullopt,
                          std::function<void(const value_t &)> on_complete = nullptr);

  /// Thread-safe interrupt.
  void interrupt();
  void reset_interrupt();

  /// Thread-safe pause / resume.
  void pause();
  void resume();
  bool is_paused() const;

  /// Access evaluation statistics.
  evaluation_stats &stats() { return stats_; }
  const evaluation_stats &stats() const { return stats_; }

  /// Access user-defined macros.
  using macro_map = std::unordered_map<std::string, std::pair<std::vector<symbol>, value_t>>;
  macro_map &user_macros() { return user_macros_; }

private:
  // Evaluate `expr`, keeping it on the trail meanwhile (an exception's traceback).
  value_t eval_internal(const value_t &expr, const environment_ptr &env);
  value_t eval_form(const value_t &expr, const environment_ptr &env);
  void check_interrupted();

  // Special form handlers
  value_t do_if(const std::vector<value_t> &args, const environment_ptr &env);
  value_t do_begin(std::span<const value_t> args, const environment_ptr &env);
  value_t do_while(const std::vector<value_t> &args, const environment_ptr &env);
  value_t do_for(const std::vector<value_t> &args, const environment_ptr &env);
  value_t do_set(const std::vector<value_t> &args, const environment_ptr &env);
  value_t do_quote(const std::vector<value_t> &args);
  value_t do_lambda(const std::vector<value_t> &args, const environment_ptr &env);
  value_t do_return(const std::vector<value_t> &args, const environment_ptr &env);
  value_t do_let(const std::vector<value_t> &args, const environment_ptr &env);
  value_t do_super(const std::vector<value_t> &args, const environment_ptr &env);
  value_t do_defun(const std::vector<value_t> &args, const environment_ptr &env);
  value_t do_defclass(const std::vector<value_t> &args, const environment_ptr &env);
  value_t do_defmacro(const std::vector<value_t> &args, const environment_ptr &env);
  value_t do_eval(const std::vector<value_t> &args, const environment_ptr &env);
  value_t do_root();
  value_t do_try(const std::vector<value_t> &args, const environment_ptr &env);
  value_t do_raise(const std::vector<value_t> &args, const environment_ptr &env);

  // Exceptions (docs/STATE_EXEC_EXCEPTIONS.md). Where one is raised is recorded once, by a
  // raise or by the first trail guard unwinding passes (record_raise_point), and the exception
  // made from it when a try catches it (caught): no level catches and rethrows on the way.
  void record_raise_point() noexcept;
  // Fill in where `exc` was raised (the recorded point): its traceback, the exception being
  // handled there as its cause, and the function that failed. What it has already is kept.
  void annotate(value_t &exc);
  // The exception a C++ exception a try caught is, annotated; nullopt for a kill.
  std::optional<value_t> caught(const std::exception &e);

  // Helpers
  value_t apply_closure(const closure_ptr &cls, const std::vector<value_t> &args);
  value_t substitute(const value_t &tmpl, const std::unordered_map<std::string, value_t> &subst);

  environment_ptr global_env_;
  value_t root_expr_;
  macro_map user_macros_;
  evaluation_stats stats_;
  // Emptied vectors of values kept between forms (a form's arguments and their values).
  std::vector<std::vector<value_t>> spare_vectors_;

  // The expressions being evaluated, innermost last: an exception's traceback.
  std::vector<const value_t *> trail_;
  // Counts evaluations: the first trail guard an exception unwinds records the raise point.
  uint64_t generation_ = 0;
  // The last raise point: the innermost trail (kTracebackFrames of it) and the exception being
  // handled there.
  std::vector<value_t> raise_trail_;
  value_t raise_handled_;
  uint64_t raise_generation_ = ~uint64_t{0};
  // The exceptions being handled, innermost last: by a catch handler (true), or by a finally on
  // their way out (false).
  std::vector<std::pair<value_t, bool>> handling_;
  // The last exception made from a C++ exception, and that exception: evaluate() throws it as
  // the original when nothing caught it.
  value_t converted_;
  std::exception_ptr converted_original_;

  std::atomic<bool> interrupted_{false};
  std::atomic<bool> paused_{false};
  std::mutex pause_mu_;
  std::condition_variable pause_cv_;
  std::recursive_mutex eval_mu_;
};

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_EVALUATOR_H
