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
#include <cstdint>
#include <cvc/state/state_exec/data_job.h>
#include <cvc/state/state_exec/evaluator.h>
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/parser.h> // parse_options
#include <cvc/state/state_exec/types.h>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <span>
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
  // try (docs/STATE_EXEC_EXCEPTIONS.md): the body, a catch handler, or the finally cleanup runs
  try_body,
  try_handler,
  try_finally,
  raise_value, // (raise e): evaluate e then raise it
  // A native call parked its process (docs/STATE_EXEC_EXCEPTIONS.md 3.9): the frame waits for
  // its result to be delivered, then completes the call with it or raises at the call.
  await_call,
  // with-lock (docs/STATE_EXEC_LOCKS.md 8.5): a request being waited for (its outcome kept in
  // the frame), then the section's body.
  lock_enter,
  lock_body,
  // (and ...) / (or ...): an argument evaluated, then the next, until one decides the form.
  // Last, so the phases before keep their numbers.
  and_next,
  or_next,
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
  /// The frame evaluates a DSL function's body: a `return` inside it unwinds no further.
  bool function_body = false;
  /// A call's function, as found when the call was initialized, so that applying it needn't
  /// look the name up again: the scopes that lookup walked, from `env` out to the one that binds
  /// it (how many, 0 for no record, and a stamp of their sizes and identities: address and
  /// environment::serial), and the binding (a node of that scope's `bindings`). The apply step
  /// uses the binding only while those scopes are as they were: a scope that gained a binding
  /// meanwhile (one that might shadow it), or a chain a host rewired (even to a scope made where
  /// a dropped one was), makes it look the name up again, as a frame without a record does (a
  /// restored one). Not serialized; not part of what the frame means. (A host that erases a
  /// name from a scope's `bindings`, rather than adding or setting one, must first clear the
  /// record of every frame waiting to apply a function found in that scope.)
  std::uint32_t callee_depth = 0;
  std::uint64_t callee_stamp = 0;
  const std::pair<const std::string, value_t> *callee = nullptr;
};

/// Deep-data work a step left unfinished (data_options::step_nodes, or a job waiting on a call):
/// a native call's job (its frame waits in the apply phase), the expansion of a macro call (its
/// frame waits to be re-initialized with it), or a measurement of the process's heap that its
/// scheduler started between two steps (heap.h; its result goes to
/// evaluator_state::heap_measured, and evaluation resumes where it was).
struct pending_data_work {
  enum class kind { native_call, macro_expansion, heap_measure };
  std::unique_ptr<data_job> job;
  std::function<value_t(value_t)> then; // native_call: turns the job's result into the call's
  kind what = kind::native_call;
  /// The stack size when the work began. The frame waiting on it is stack[base - 1]; a DSL
  /// function the job calls runs in the frames above it.
  std::size_t base = 0;
  /// How many values the waiting frame held then: the call's arguments, which a restore runs
  /// the call with again.
  std::size_t frame_args = 0;
  /// Set while a DSL function the job called runs; its value arrives on the waiting frame, and
  /// `return`/`break` in it unwind no further than that frame.
  bool calling = false;
  /// The work this work runs inside of, if any: the job of a call that was waiting on a DSL
  /// function when this work began, or (feeds_outer) the job whose native call left this work.
  std::shared_ptr<pending_data_work> outer;
  /// The result is the value of a call outer's job made, not the waiting frame's.
  bool feeds_outer = false;

  /// Frees the chain of outer work a link at a time, not one native frame per link: a native
  /// call that waits on one like it, with more to do after it (a class constructor whose init
  /// constructs again), makes a chain as long as the process's steps allow.
  ~pending_data_work();
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
  /// it is done (except while a DSL function it called runs), then completes what started it.
  /// The innermost work; pending_data_work::outer chains the rest. Not serialized: after a
  /// restore the call or macro expansion waiting on the outermost work simply runs again.
  std::shared_ptr<pending_data_work> data_work;
  /// Last owners this state's steps dropped and have not freed yet (data_step_nodes > 0); each
  /// step frees them with what is left of its slice, and steps that only free them follow
  /// while any are left.
  std::shared_ptr<detail::release_backlog> backlog;
  /// A heap measurement's result in bytes, once its data steps finish; the scheduler that started
  /// it takes it from here. Not serialized.
  std::optional<uint64_t> heap_measured;
  /// The exception no `try` caught (docs/STATE_EXEC_EXCEPTIONS.md): the state is done, and
  /// step() throws it again. Not serialized.
  std::optional<value_t> uncaught;
  /// The last exception made from a C++ exception, and that exception: an uncaught exception
  /// that is this one carries it as its dsl_exception's original().
  value_t converted;
  std::exception_ptr converted_original;
  /// A generator's state (builtin_generator): with-lock refuses to run in it (LockError
  /// "generator"; docs/STATE_EXEC_LOCKS.md 5.3). Not serialized.
  bool in_generator = false;
};

/// Whether a frame of `st` waits on deep-data work: any work but a heap measurement, which waits
/// on no frame. A process cannot fork while one does (the child could not finish the call).
bool waits_on_data_work(const evaluator_state &st);

// ---------------------------------------------------------------------------
// Parking at a call (docs/STATE_EXEC_EXCEPTIONS.md 3.9)
// ---------------------------------------------------------------------------
//
// A native function that waits for something (an async call's kernel, a future) parks its
// process at the call: the scheduler makes the process wait on a channel, and the call's frame
// stays on the stack in the await_call phase. The scheduler delivers to that frame
// (deliver_await), or times it out (expire_await); the next step completes the call with what
// was delivered, or raises at the call. A frame stepped without either (a process restored from
// a snapshot while it waited) raises RuntimeError: its call can't resume.

namespace detail {

/// What an await_call frame waits for when it isn't a plain channel: an async call. A handle
/// value (make_awaitable_handle) holds one.
class awaitable {
public:
  virtual ~awaitable() = default;
  /// The function's name, for errors.
  virtual const std::string &function_name() const = 0;
  /// Whether its outcome is known.
  virtual bool finished() const = 0;
  /// Block until it is finished or `seconds` pass (nullopt: no limit); whether it finished.
  virtual bool wait(std::optional<double> seconds) = 0;
  /// Its value, or throws the exception it raises at the call (a dsl_exception). Only once
  /// finished().
  virtual value_t outcome() = 0;
};

/// The data_object type of a handle to an awaitable. Its payload doesn't survive a snapshot.
inline constexpr const char *kAwaitableType = "async_call";

/// A value holding `a`, for an await_call frame or a future.
value_t make_awaitable_handle(std::shared_ptr<awaitable> a);

/// The awaitable a handle holds, or null (not a handle, or one decoded from a snapshot).
std::shared_ptr<awaitable> awaitable_of(const value_t &handle);

/// A native call's request to park its process at the call (request_await).
struct await_request {
  std::string channel;  ///< the channel the scheduler made the process wait on
  std::string function; ///< the function's name, for errors
  value_t handle;       ///< what it waits for (make_awaitable_handle), or nil for a channel
  /// On delivery, apply the call again with the same arguments instead of completing it with
  /// what was delivered: (await-all ...) waits for each future in turn this way.
  bool retry = false;
  /// With retry: what was delivered first takes the place of item `slot` of the call's first
  /// argument (a list), as awaited_value(it). (await-all ...) collects channel futures' messages
  /// this way, so a snapshot keeps the ones it has.
  std::optional<std::size_t> slot;
  /// Delivered already (a pending message the native took): the frame waits for nothing, and the
  /// next step completes it (or, with retry, applies the call again).
  std::optional<value_t> ready;
};

/// {"__awaited__": v}: what a retried call's `slot` became (await_request::slot).
value_t awaited_value(value_t v);

/// The value an awaited_value() holds, or null when `v` isn't one.
const value_t *awaited_of(const value_t &v);

} // namespace detail

/// Whether the native call being applied now may park its process at the call: the stackless
/// evaluator is applying it at its own level (not from inside another native call) in the
/// outermost evaluator step on this thread (not in a generator's or a nested evaluator's).
/// Whether a scheduler will deliver to the process is the native's to know.
bool can_await();

/// Park at the call: once the native call returns (what it returns is ignored), its frame
/// waits in the await_call phase. The native has made its scheduler wait on `r.channel`.
/// Throws std::logic_error when !can_await().
void request_await(detail::await_request r);

/// Whether parking is forbidden on this thread now (park_forbidden_scope): an async function
/// called here raises RuntimeError naming it, instead of waiting or running its kernel inline.
bool parking_forbidden();

/// RAII: forbid parking on this thread while alive. Ariadne's reactive and render lane arms one.
class park_forbidden_scope {
public:
  park_forbidden_scope();
  ~park_forbidden_scope();
  park_forbidden_scope(const park_forbidden_scope &) = delete;
  park_forbidden_scope &operator=(const park_forbidden_scope &) = delete;

private:
  bool outer_;
};

/// For the schedulers: deliver `msg` to the await_call frame on top of `st`'s stack (an async
/// call's frame ignores it and reads its call). False when the top frame doesn't wait.
bool deliver_await(evaluator_state &st, value_t msg);

/// For the schedulers: time out the await_call frame on top of `st`'s stack; its call raises
/// TimeoutError. False when the top frame doesn't wait.
bool expire_await(evaluator_state &st);

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

  /// Execute one evaluation step.  Returns true when done. An exception the program doesn't
  /// catch is thrown as a dsl_exception, with the C++ exception it was made from, if any, as its
  /// original(). The state is then done, and stepping it again throws it again.
  bool step(evaluator_state &state);

  /// Raise `exc` (an exception instance, docs/STATE_EXEC_EXCEPTIONS.md) in `state` at the frame
  /// it is evaluating, as if that frame had raised it: the nearest `try` handles it. If none
  /// does, the state is done with it uncaught, and the next step() throws it. Throws
  /// std::invalid_argument for a value that isn't an exception.
  void raise(evaluator_state &state, value_t exc);

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
  void push_frame(evaluator_state &state, const value_t &expr, const environment_ptr &env);
  void push_frame(evaluator_state &state, value_t &&expr, environment_ptr &&env);

  // Step implementations — one per phase
  void step_init(evaluator_state &state, eval_frame &frame);
  void step_eval_args(evaluator_state &state, eval_frame &frame);
  void step_apply(evaluator_state &state, eval_frame &frame);
  void step_if_cond(evaluator_state &state, eval_frame &frame);
  void step_if_branch(evaluator_state &state, eval_frame &frame);
  void step_begin_next(evaluator_state &state, eval_frame &frame);
  void step_and_or_next(evaluator_state &state, eval_frame &frame);
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
  void step_try_done(evaluator_state &state, eval_frame &frame); // the body or a handler finished
  void step_try_finally(evaluator_state &state, eval_frame &frame);
  void step_raise_value(evaluator_state &state, eval_frame &frame);
  void step_await_call(evaluator_state &state, eval_frame &frame);
  // with-lock (docs/STATE_EXEC_LOCKS.md 8.5): a lock form's entry, its wait's step and its body.
  void step_lock_enter(evaluator_state &state, eval_frame &frame);
  void step_lock_body(evaluator_state &state, eval_frame &frame);
  /// The frame at `frame_index` calls the lock form `ops` (a lock_ops object): parse its options,
  /// lay the frame out, and acquire (or evaluate KEY and :timeout first).
  void begin_lock_form(evaluator_state &state, std::size_t frame_index, const value_t &ops);
  /// Protocol 5.1 steps 1-5 for the lock frame at `frame_index`, KEY and :timeout known.
  void lock_acquire(evaluator_state &state, std::size_t frame_index, const value_t &key,
                    std::optional<double> timeout);
  /// The lock frame at `frame_index` holds what it asked for (or runs :else): push its body.
  void lock_body_begins(evaluator_state &state, std::size_t frame_index);
  /// A restored lock wait (the top frame, its lock_enter frame below): apply its wait again.
  void resume_lock_wait(evaluator_state &state);

  // Exceptions (docs/STATE_EXEC_EXCEPTIONS.md)
  // The step that raised `e` (the top frame's) failed: raise its exception from that frame.
  void fail_step(evaluator_state &state, const std::exception &e);

  // Helpers
  // A for loop over a DSL generator: push the call that advances it as a job.
  void advance_generator(evaluator_state &state, eval_frame &frame);
  // Apply native `fn` to `args` as the top frame's call: its value pops the frame, or, if it
  // left deep-data work (a job, or a DSL function to call), the frame waits for that.
  void apply_native(evaluator_state &state, eval_frame &frame, const native_fn &fn,
                    std::span<const value_t> args);
  void call_closure(evaluator_state &state, eval_frame &frame, const closure_ptr &cls,
                    const std::vector<value_t> &args);
  // Push the frame that evaluates `cls`'s body in `local` (its parameters already bound).
  void push_closure_body(evaluator_state &state, const closure_ptr &cls, environment_ptr local);
  void handle_defclass(evaluator_state &state, eval_frame &frame, std::span<const value_t> args);

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
