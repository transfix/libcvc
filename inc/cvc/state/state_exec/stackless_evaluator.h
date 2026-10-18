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

#include <any>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cvc/state/state_exec/data_job.h>
#include <cvc/state/state_exec/evaluator.h>
#include <cvc/state/state_exec/parser.h> // parse_options
#include <cvc/state/state_exec/types.h>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
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
  yield_value,  // Generator: evaluate yield expression then pause
  break_unwind, // Loop break: evaluate optional value then unwind to loop
};

/// A single evaluation frame on the explicit stack.
struct eval_frame {
  value_t expr;
  environment_ptr env;
  eval_phase phase = eval_phase::init;
  int64_t index = 0;
  std::vector<value_t> results;
  // Extra state for complex forms (keyed by name; avoids map overhead)
  std::vector<value_t> extra_vals;
  std::vector<std::string> extra_strs;
  std::vector<std::vector<value_t>> extra_lists;
  environment_ptr extra_env;
};

/// Deep-data work a step left unfinished (data_options::step_nodes): a native call's job (its
/// frame waits in the apply phase), the expansion of a macro call (its frame waits to be
/// re-initialized with it), or a measurement of the process's heap that its scheduler started
/// between two steps (heap.h; its result goes to evaluator_state::heap_measured, and evaluation
/// resumes where it was).
struct pending_data_work {
  enum class kind { native_call, macro_expansion, heap_measure };
  std::unique_ptr<data_job> job;
  std::function<value_t(value_t)> then; // native_call: turns the job's result into the call's
  kind what = kind::native_call;
};

/// Serializable evaluator state — the entire execution snapshot.
struct evaluator_state {
  std::vector<eval_frame> stack;
  value_t result;
  bool done = false;
  value_t root_expr;
  environment_ptr global_env;
  evaluator::macro_map user_macros;
  evaluation_stats stats;
  bool yielded = false; // Set by (yield val) — signals generator_next to stop
  /// Set while the script is still being read, a slice per step (parse_options::step_tokens);
  /// step() reads instead of evaluating until the whole script is read.
  std::shared_ptr<script_reader> reading;
  /// How many nodes of deep-data work a step may do (data_options::step_nodes; a scheduler sets
  /// it). 0 does all of it in the step that needs it.
  std::size_t data_step_nodes = 0;
  /// Set while deep-data work runs a slice per step; step() runs it instead of evaluating until
  /// it is done, then completes what started it. Not serialized: after a restore the waiting
  /// call or macro expansion simply runs again.
  std::shared_ptr<pending_data_work> data_work;
  /// Last owners this state's steps dropped and have not freed yet (data_step_nodes > 0); each
  /// step frees them with what is left of its slice, and steps that only free them follow
  /// while any are left.
  std::shared_ptr<detail::release_backlog> backlog;
  /// A heap measurement's result in bytes, once its data steps finish; the scheduler that started
  /// it takes it from here. Not serialized.
  std::optional<uint64_t> heap_measured;
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

  /// The global environment new states extend (the cycle collector walks it from a generator).
  const environment_ptr &global_env() const { return global_env_; }

  /// Create a fresh evaluator_state from a parsed expression.
  evaluator_state create_state(const value_t &expr, environment_ptr env = nullptr);

  /// Create a fresh evaluator_state from a script string, parsed with `popts` (a scheduler passes
  /// the parser settings it resolved from the state tree). With popts.step_tokens > 0 nothing is
  /// parsed yet: the state reads the script popts.step_tokens tokens per step() first, and a
  /// syntax error is thrown from step() rather than from here.
  evaluator_state create_state(const std::string &script, environment_ptr env = nullptr,
                               const parse_options &popts = {});

  /// Execute one evaluation step.  Returns true when done.
  bool step(evaluator_state &state);

  /// Run until done or limits reached.  Returns result or nil if not done.
  value_t run(evaluator_state &state, std::optional<uint64_t> max_steps = std::nullopt,
              std::optional<double> timeout_sec = std::nullopt,
              std::function<void(const value_t &)> on_complete = nullptr);

  /// Convenience: create + run to completion.
  value_t evaluate(const value_t &expr, environment_ptr env = nullptr,
                   std::optional<double> timeout_sec = std::nullopt,
                   std::function<void(const value_t &)> on_complete = nullptr);

  /// Convenience: parse + create + run.
  value_t evaluate_script(const std::string &script, environment_ptr env = nullptr,
                          std::optional<double> timeout_sec = std::nullopt,
                          std::function<void(const value_t &)> on_complete = nullptr);

  void interrupt();
  void reset_interrupt();
  void pause();
  void resume();
  bool is_paused() const;

  /// Restrict which special forms this evaluator permits (a security/capability control for
  /// restricted contexts such as a per-frame UI predicate). `allowed` lists the permitted
  /// special-form names (e.g. {"if","begin","let","quote"}); using any OTHER special form
  /// (defclass/defmacro/eval/while/…) throws. Pass nullptr (the default) for NO restriction —
  /// every existing caller is unaffected. Special forms are evaluator-intrinsic and cannot be
  /// gated by the environment, so this is the only way to bar them. Applies to nested
  /// evaluate()/run() on THIS evaluator, not to a separately-constructed nested evaluator.
  void restrict_special_forms(std::shared_ptr<const std::set<std::string>> allowed);

  /// The set of all special-form names this evaluator recognises (for building an allowlist).
  static const std::set<std::string> &all_special_forms();

private:
  void check_interrupted();

  // A state that evaluates `forms` as a script: done when empty, the form itself when one, a
  // (begin ...) of them otherwise.
  evaluator_state create_program_state(std::vector<value_t> forms, environment_ptr env);
  // One step of a state that is still reading its script (evaluator_state::reading).
  bool step_read(evaluator_state &state);
  // One step of deep-data work left unfinished (evaluator_state::data_work).
  bool step_data(evaluator_state &state);
  // Re-initialize the top frame to evaluate a macro call's expansion.
  void begin_expansion(evaluator_state &state, value_t expansion);
  // One evaluation step of the top frame.
  void step_frame(evaluator_state &state);

  // Frame management
  void pop_frame(evaluator_state &state, value_t result);
  void push_frame(evaluator_state &state, value_t expr, environment_ptr env);

  // Step implementations — one per phase
  void step_init(evaluator_state &state, eval_frame &frame);
  void step_eval_args(evaluator_state &state, eval_frame &frame);
  void step_apply(evaluator_state &state, eval_frame &frame);
  void step_if_cond(evaluator_state &state, eval_frame &frame);
  void step_if_branch(evaluator_state &state, eval_frame &frame);
  void step_begin_next(evaluator_state &state, eval_frame &frame);
  void step_while_cond(evaluator_state &state, eval_frame &frame);
  void step_while_body(evaluator_state &state, eval_frame &frame);
  void step_for_body(evaluator_state &state, eval_frame &frame);
  void step_set_value(evaluator_state &state, eval_frame &frame);
  void step_let_bindings(evaluator_state &state, eval_frame &frame);
  void step_let_body(evaluator_state &state, eval_frame &frame);
  void step_return_value(evaluator_state &state, eval_frame &frame);
  void step_eval_inner(evaluator_state &state, eval_frame &frame);
  void step_super_args(evaluator_state &state, eval_frame &frame);
  void step_yield_value(evaluator_state &state, eval_frame &frame);
  void step_break_unwind(evaluator_state &state, eval_frame &frame);

  // Helpers
  void call_closure(evaluator_state &state, eval_frame &frame, const closure_ptr &cls,
                    const std::vector<value_t> &args);
  void handle_defclass(evaluator_state &state, eval_frame &frame, const std::vector<value_t> &args);

  environment_ptr global_env_;
  std::shared_ptr<const std::set<std::string>>
      allowed_forms_; // nullptr = all special forms allowed

  std::atomic<bool> interrupted_{false};
  std::atomic<bool> paused_{false};
  mutable std::mutex pause_mu_;
  std::condition_variable pause_cv_;
  std::recursive_mutex eval_mu_;
};

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_STACKLESS_EVALUATOR_H
