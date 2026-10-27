#include "closure_body.h"
#include "closure_natives.h"
#include "scope_mark.h"
#include "special_forms.h"

#include <chrono>
#include <cstdint>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/generator.h>
#include <cvc/state/state_exec/heap.h>
#include <cvc/state/state_exec/parser.h>
#include <cvc/state/state_exec/stackless_evaluator.h>
#include <iterator>
#include <set>
#include <stdexcept>

namespace cvc::state_exec {

pending_data_work::~pending_data_work() {
  // Each link is unlinked from the next before it goes, so its own destructor has nothing to free.
  std::shared_ptr<pending_data_work> next = std::move(outer);
  while (next && next.use_count() == 1)
    next = std::move(next->outer);
}

bool waits_on_data_work(const evaluator_state &st) {
  for (const pending_data_work *w = st.data_work.get(); w; w = w->outer.get())
    if (w->what != pending_data_work::kind::heap_measure)
      return true;
  return false;
}

// ---------------------------------------------------------------------------
// ctor
// ---------------------------------------------------------------------------

stackless_evaluator::stackless_evaluator(environment_ptr global_env)
    : global_env_(std::move(global_env)) {}

const std::set<std::string> &stackless_evaluator::all_special_forms() {
  static const std::set<std::string> forms = {
      "if",  "begin", "while",    "for",  "set",  "quote", "lambda",   "return", "yield", "break",
      "let", "defun", "defmacro", "eval", "root", "super", "defclass", "try",    "raise"};
  return forms;
}

void stackless_evaluator::restrict_special_forms(
    std::shared_ptr<const std::set<std::string>> allowed) {
  allowed_forms_ = std::move(allowed);
}

// ---------------------------------------------------------------------------
// public API
// ---------------------------------------------------------------------------

evaluator_state stackless_evaluator::create_state(const value_t &expr, environment_ptr env) {
  auto run_env = env ? env : environment::extend(global_env_);
  evaluator_state st;
  st.root_expr = expr;
  st.global_env = run_env;
  st.stack.push_back({.expr = expr, .env = run_env});
  return st;
}

evaluator_state stackless_evaluator::create_state(const std::string &script, environment_ptr env,
                                                  const parse_options &popts) {
  if (popts.step_tokens > 0) {
    // Read inside the process: step_read() parses a slice per step, then sets up evaluation.
    evaluator_state st;
    st.global_env = env ? env : environment::extend(global_env_);
    st.reading = std::make_shared<script_reader>(script, popts);
    return st;
  }
  return create_program_state(parse_all(script, popts), env);
}

evaluator_state stackless_evaluator::create_program_state(std::vector<value_t> forms,
                                                          environment_ptr env) {
  if (forms.empty()) {
    evaluator_state st;
    st.done = true;
    return st;
  }
  if (forms.size() == 1)
    return create_state(forms[0], env);
  std::vector<value_t> begin_exprs;
  begin_exprs.reserve(1 + forms.size());
  begin_exprs.push_back(value_t{symbol{"begin"}});
  begin_exprs.insert(begin_exprs.end(), std::make_move_iterator(forms.begin()),
                     std::make_move_iterator(forms.end()));
  return create_state(make_list(std::move(begin_exprs)), env);
}

// Read the next slice of the script. Once all of it is read, the state becomes exactly what
// create_state(script) would have built, keeping the step count and timing of the reading. A
// parse_error propagates like any error from step(): the sync scheduler rethrows it to its
// caller, and the async scheduler kills the process with it.
bool stackless_evaluator::step_read(evaluator_state &state) {
  state.stats.increment_step(step_kind::read);
  script_reader &reader = *state.reading;
  if (!reader.step(reader.options().step_tokens))
    return false;
  evaluator_state program = create_program_state(std::move(reader.forms()), state.global_env);
  program.stats = std::move(state.stats);
  program.data_step_nodes = state.data_step_nodes;
  program.backlog = std::move(state.backlog);
  state = std::move(program); // also releases the reader
  return state.done;
}

namespace {
// -- frame vectors recycled on this thread ----------------------------------------------------
//
// Nearly every list form gives its frame a vector or two: the call's arguments, the values its
// children return, a set's name. Allocated for the frame and freed when it pops, they were most
// of the allocations a step made. A popped frame gives its (emptied) vectors back here instead,
// kept by capacity, and the next frame takes one of exactly the capacity it needs: a frame holds
// no more than it would have with a fresh vector, and the stack's memory (and its measurement,
// heap.h) stays what it was. Per thread and bounded, like the scopes environment::extend() keeps:
// at most 16 vectors of each capacity from 1 to 8 values (576 values of 72 bytes on 64-bit MSVC,
// about 41 KB) and 16 one-string vectors, kept until the thread exits.
struct frame_vectors {
  static constexpr std::size_t kMaxCapacity = 8; // larger vectors are freed
  static constexpr std::size_t kPerCapacity = 16;
  std::vector<value_t> values[kMaxCapacity + 1][kPerCapacity];
  std::size_t nvalues[kMaxCapacity + 1] = {};
  std::vector<std::string> strings[kPerCapacity]; // capacity 1: a set's or a for's name
  std::size_t nstrings = 0;
  ~frame_vectors();
};

// Set once the thread has destroyed its vectors: a frame popped later in the thread's exit frees
// its own.
thread_local bool t_frame_vectors_gone = false;
thread_local frame_vectors t_frame_vectors;

frame_vectors::~frame_vectors() { t_frame_vectors_gone = true; }

frame_vectors *kept_vectors() noexcept { return t_frame_vectors_gone ? nullptr : &t_frame_vectors; }

// An empty vector with room for `n` values: a kept one of capacity n, if there is one.
std::vector<value_t> take_values(std::size_t n) {
  if (n - 1 < frame_vectors::kMaxCapacity)
    if (frame_vectors *kept = kept_vectors(); kept && kept->nvalues[n] > 0)
      return std::move(kept->values[n][--kept->nvalues[n]]);
  std::vector<value_t> v;
  v.reserve(n);
  return v;
}

// Keep `v` (emptied) for a frame to take, or free it.
void give_values(std::vector<value_t> &v) noexcept {
  const std::size_t n = v.capacity();
  if (n - 1 < frame_vectors::kMaxCapacity)
    if (frame_vectors *kept = kept_vectors();
        kept && kept->nvalues[n] < frame_vectors::kPerCapacity)
      kept->values[n][kept->nvalues[n]++] = std::move(v); // v is empty: nothing is dropped
}

// Make `v` (empty) able to hold `n` values without growing: a kept vector if it has no room.
void make_room(std::vector<value_t> &v, std::size_t n) {
  if (v.capacity() >= n)
    return;
  if (v.capacity() == 0)
    v = take_values(n);
  else
    v.reserve(n);
}

// Fill `v` (empty) with copies of `from`.
void fill_values(std::vector<value_t> &v, std::span<const value_t> from) {
  make_room(v, from.size());
  v.assign(from.begin(), from.end());
}

// Set `strs` (empty) to the one string `s`.
void set_one_string(std::vector<std::string> &strs, const std::string &s) {
  if (strs.capacity() == 0)
    if (frame_vectors *kept = kept_vectors(); kept && kept->nstrings > 0)
      strs = std::move(kept->strings[--kept->nstrings]);
  strs.push_back(s);
}

void give_strings(std::vector<std::string> &strs) noexcept {
  if (strs.capacity() == 1)
    if (frame_vectors *kept = kept_vectors(); kept && kept->nstrings < frame_vectors::kPerCapacity)
      kept->strings[kept->nstrings++] = std::move(strs);
}

// Pop the top frame, keeping its vectors. Its parts are freed in the order its destructor frees
// them (the last member first), so the owners it drops reach a release backlog in the same order.
void pop_top(std::vector<eval_frame> &stack) {
  eval_frame &f = stack.back();
  f.extra_env.reset();
  f.extra_lists.clear();
  f.extra_strs.clear();
  give_strings(f.extra_strs);
  f.extra_vals.clear();
  give_values(f.extra_vals);
  f.results.clear();
  give_values(f.results);
  stack.pop_back(); // env, then expr
}

// Push a frame evaluating `expr` in `env`, built in place.
void push_new_frame(std::vector<eval_frame> &stack, value_t &&expr, environment_ptr &&env) {
  eval_frame &f = stack.emplace_back();
  f.expr = std::move(expr);
  f.env = std::move(env);
}

// Push the frame that evaluates a DSL function's body: a `return` inside it returns from here.
void push_body_frame(std::vector<eval_frame> &stack, value_t &&body, environment_ptr &&local) {
  push_new_frame(stack, std::move(body), std::move(local));
  stack.back().function_body = true;
}

// The function a call frame applies: the binding its init found, while the scopes it walked are
// as they were and it still binds the call's head, else whatever the name finds now (as if
// looked up again: a scope may have gained a binding of the name, or the head changed).
const value_t *callee_of(const eval_frame &frame, const std::string &name) {
  if (frame.callee &&
      detail::mark_holds(frame.env.get(), {frame.callee_depth, frame.callee_stamp}) &&
      frame.callee->first == name)
    return &frame.callee->second;
  return frame.env->lookup(name);
}

// Pop the top frame with the value `v`, which goes to the frame below (or is the state's result).
// It is taken before the top frame goes, since it may be in that frame, or in a scope only that
// frame holds; the frame's parts are then freed as pop_frame frees them, and for the last frame
// before the state's old result is (what pop_frame did: a release backlog sees the same order).
void hand_up(evaluator_state &state, const value_t &v) {
  std::vector<eval_frame> &stack = state.stack;
  if (stack.size() > 1) {
    stack[stack.size() - 2].results.push_back(v);
    pop_top(stack);
  } else {
    value_t result = v;
    pop_top(stack);
    state.result = std::move(result);
    state.done = true;
  }
}

void hand_up(evaluator_state &state, value_t &&v) {
  std::vector<eval_frame> &stack = state.stack;
  if (stack.size() > 1) {
    stack[stack.size() - 2].results.push_back(std::move(v));
    pop_top(stack);
  } else {
    value_t result = std::move(v);
    pop_top(stack);
    state.result = std::move(result);
    state.done = true;
  }
}

// -- the (begin ...) bodies shared by closures (closure_body.h) --------------------------------
//
// One slot per lambda form, by its address, holding (weakly) the (begin ...) the last closure it
// made shares. A slot whose form was freed and its address reused is harmless: what a slot holds
// is used only while it is exactly (begin body...) for the new closure's body, each form the same
// object or an atom that evaluates the same (same_form: a number bit for bit, so -0.0 is not
// 0.0). Per thread, and bounded (64 slots, each a weak reference: the list's block stays
// allocated while the slot holds it, about 40 bytes, until the slot is reused).
struct shared_bodies {
  static constexpr std::size_t kSlots = 64;
  struct slot {
    const void *form = nullptr;
    std::weak_ptr<std::vector<value_t>> expr;
  };
  slot slots[kSlots];
  ~shared_bodies();
};

thread_local bool t_shared_bodies_gone = false;
thread_local shared_bodies t_shared_bodies;

shared_bodies::~shared_bodies() { t_shared_bodies_gone = true; }

// A loop's body: the form at args[first], or, when there are several, all of them in order as a
// (begin ...). A single-form body stays as it is, so its frames are what they always were.
value_t loop_body(std::span<const value_t> args, std::size_t first) {
  if (args.size() == first + 1)
    return args[first];
  std::vector<value_t> forms;
  forms.reserve(1 + args.size() - first);
  forms.emplace_back(symbol{"begin"});
  forms.insert(forms.end(), args.begin() + static_cast<std::ptrdiff_t>(first), args.end());
  return make_list(std::move(forms));
}

// RAII: while alive, the native call a step applies may leave deep-data work behind
// (run_data_job), in the step's data scope.
struct native_call_scope {
  explicit native_call_scope(detail::data_step_scope *data) : data(data) {
    if (data)
      data->allow_defer(true);
  }
  ~native_call_scope() {
    if (data)
      data->allow_defer(false);
  }
  native_call_scope(const native_call_scope &) = delete;
  native_call_scope &operator=(const native_call_scope &) = delete;
  detail::data_step_scope *data;
};

// New innermost deep-data work, waiting on stack[base - 1].
std::shared_ptr<pending_data_work>
make_work(std::unique_ptr<data_job> job, std::function<value_t(value_t)> then,
          pending_data_work::kind what, std::size_t base, std::size_t frame_args,
          std::shared_ptr<pending_data_work> outer, bool feeds_outer) {
  auto work = std::make_shared<pending_data_work>();
  work->job = std::move(job);
  work->then = std::move(then);
  work->what = what;
  work->base = base;
  work->frame_args = frame_args;
  work->outer = std::move(outer);
  work->feeds_outer = feeds_outer;
  return work;
}

// Make the job a native call left the innermost work: waiting on stack[base - 1] or, with
// feeds_outer, for the call outer's job made. Work the job's own pending call left
// (deferred_data_job::waits_on) goes inside it, its result being that call's value.
void await_deferred(evaluator_state &state, detail::deferred_data_job &&deferred, std::size_t base,
                    std::size_t frame_args, std::shared_ptr<pending_data_work> outer,
                    bool feeds_outer) {
  for (detail::deferred_data_job *d = &deferred; d; d = d->waits_on.get()) {
    state.data_work =
        make_work(std::move(d->job), std::move(d->then), pending_data_work::kind::native_call, base,
                  frame_args, std::move(outer), feeds_outer);
    outer = state.data_work;
    feeds_outer = true;
  }
}

// Whether the innermost work waits on a DSL function it called that is still running.
bool calling_out(const evaluator_state &state) {
  const pending_data_work *work = state.data_work.get();
  return work && work->calling && state.stack.size() > work->base;
}

// How far down a `return` or `break` may unwind: to the frame waiting on a DSL function a job
// called, whose value that is (0 = no such call).
std::size_t unwind_floor(const evaluator_state &state) {
  const pending_data_work *work = state.data_work.get();
  return work && work->calling ? work->base : 0;
}

// -- exceptions (docs/STATE_EXEC_EXCEPTIONS.md) -------------------------------------------------

// A try frame (phases try_body, try_handler, try_finally) keeps
//   extra_vals  = {the body, the cleanup (nil without a finally), the exception a handler handles
//                  or what leaves through the finally},
//   extra_strs  = {what that is: "" before either, "handling", or the way out the finally holds:
//                  "value", "raise", "return" or "break"},
//   extra_lists = {the catch clauses}.
enum try_slot : std::size_t { try_body_slot, try_cleanup_slot, try_payload_slot };

bool has_finally(const eval_frame &f) {
  return std::holds_alternative<list_ptr>(f.extra_vals[try_cleanup_slot].v);
}

// `forms` as one expression: (begin forms...), or the form itself when there is one.
value_t body_of(std::span<const value_t> forms) {
  if (forms.size() == 1)
    return forms[0];
  std::vector<value_t> exprs;
  exprs.reserve(1 + forms.size());
  exprs.emplace_back(symbol{"begin"});
  exprs.insert(exprs.end(), forms.begin(), forms.end());
  return make_list(std::move(exprs));
}

// The traceback of an exception raised at the top frame: the expression of each frame, innermost
// last.
value_t traceback_of(const evaluator_state &state) {
  const std::size_t n = state.stack.size();
  std::vector<value_t> lines;
  for (std::size_t i = n > kTracebackFrames ? n - kTracebackFrames : 0; i < n; ++i)
    lines.emplace_back(traceback_line(state.stack[i].expr));
  return make_list(std::move(lines));
}

// The exception being handled where the top frame is: a catch handler's, or (in_finally) one a
// finally is running for on its way out.
const value_t *handled_exception(const evaluator_state &state, bool in_finally) {
  for (auto it = state.stack.rbegin(); it != state.stack.rend(); ++it)
    if (it->phase == eval_phase::try_handler ||
        (in_finally && it->phase == eval_phase::try_finally && it->extra_strs[0] == "raise"))
      return &it->extra_vals[try_payload_slot];
  return nullptr;
}

// Record where an exception raised at the top frame was raised: its traceback, the exception
// being handled there as its cause, and the function that failed, for one from C++. What a
// re-raised exception already has is kept.
void annotate(const evaluator_state &state, value_t &exc) {
  if (const value_t *tb = field(exc, "traceback"); !tb || tb->is_nil())
    set_field(exc, "traceback", traceback_of(state));
  if (const value_t *cause = field(exc, "cause"); !cause || cause->is_nil())
    if (const value_t *handled = handled_exception(state, true);
        handled && !in_cause_chain(*handled, exc))
      set_field(exc, "cause", *handled);
  const value_t *origin = field(exc, "origin");
  if (!origin) {
    set_field(exc, "origin", value_t(std::string("dsl")));
    return;
  }
  const auto *from = std::get_if<std::string>(&origin->v);
  if (!from || *from == "dsl" || field(exc, "function") || state.stack.empty())
    return;
  const eval_frame &top = state.stack.back();
  const auto *lp = std::get_if<list_ptr>(&top.expr.v);
  if (top.phase != eval_phase::apply || !lp || !*lp || (*lp)->empty())
    return;
  if (const auto *head = std::get_if<symbol>(&(**lp)[0].v))
    set_field(exc, "function", value_t(head->name));
}

// Throw the exception a state left uncaught as a dsl_exception, with the C++ exception it was
// made from, if it was one, as its original().
[[noreturn]] void rethrow_uncaught(const evaluator_state &state) {
  const value_t &exc = *state.uncaught;
  const bool converted = state.converted_original && same_exception(state.converted, exc);
  throw dsl_exception(exc, converted ? state.converted_original : nullptr);
}

// Drop the deep-data work whose waiting frame unwinding popped: its native call raised.
void drop_orphaned_work(evaluator_state &state) {
  while (state.data_work && state.data_work->base > state.stack.size()) {
    std::shared_ptr<pending_data_work> outer = state.data_work->outer;
    state.data_work = std::move(outer);
  }
}

// Leave a try frame through its finally: run the cleanup, holding the way out ("value", "raise",
// "return" or "break") and its value until the cleanup is done.
void start_finally(evaluator_state &state, eval_frame &frame, const char *way, value_t payload) {
  frame.phase = eval_phase::try_finally;
  frame.extra_strs[0] = way;
  frame.extra_vals[try_payload_slot] = std::move(payload);
  frame.results.clear();
  value_t cleanup = frame.extra_vals[try_cleanup_slot];
  environment_ptr env = frame.env;
  state.stack.push_back({.expr = std::move(cleanup), .env = std::move(env)}); // last: moves frame
}

// Unwind `exc` from the top frame to the nearest try that handles it: a try running its body
// tries its catches, and one with a finally runs it on the way out. With none, the state is
// done, the exception uncaught.
void unwind_exception(evaluator_state &state, value_t exc) {
  for (;;) {
    drop_orphaned_work(state);
    if (state.stack.empty()) {
      state.result = nil_value;
      state.done = true;
      state.uncaught = std::move(exc);
      return;
    }
    eval_frame &top = state.stack.back();
    if (top.phase == eval_phase::try_body) {
      try {
        for (const value_t &clause : top.extra_lists[0]) {
          std::string var;
          if (!catch_matches(top.env, clause, exc, var))
            continue;
          value_t handler = body_of(catch_handler(clause));
          top.phase = eval_phase::try_handler;
          top.extra_strs[0] = "handling";
          top.extra_vals[try_payload_slot] = exc;
          top.results.clear();
          environment_ptr scope = environment::extend(top.env);
          if (!var.empty())
            scope->set(var, std::move(exc));
          state.stack.push_back({.expr = std::move(handler), .env = std::move(scope)}); // last
          return;
        }
      } catch (const std::exception &e) {
        // A catch naming no class: its NameError replaces the exception.
        if (is_uncatchable(e))
          throw;
        value_t replaced = exception_from(e);
        set_field(replaced, "cause", exc);
        annotate(state, replaced);
        exc = std::move(replaced);
      }
    }
    if ((top.phase == eval_phase::try_body || top.phase == eval_phase::try_handler) &&
        has_finally(top)) {
      start_finally(state, top, "raise", std::move(exc));
      return;
    }
    state.stack.pop_back();
  }
}

// Unwind a `return` (to the caller of the function it is in) or a `break` (out of the innermost
// loop) with `value` from the top frame, but not out of a DSL function a deep-data job called:
// there the value is the function's. A try with a finally on the way runs it first, and the
// unwinding goes on from that try once the cleanup is done.
void unwind_jump(evaluator_state &state, bool is_return, value_t value) {
  const std::size_t floor = unwind_floor(state);
  while (state.stack.size() > floor) {
    eval_frame &top = state.stack.back();
    if ((top.phase == eval_phase::try_body || top.phase == eval_phase::try_handler) &&
        has_finally(top)) {
      start_finally(state, top, is_return ? "return" : "break", std::move(value));
      return;
    }
    const bool out = is_return ? top.function_body || top.phase == eval_phase::apply
                               : top.phase == eval_phase::while_cond ||
                                     top.phase == eval_phase::while_body ||
                                     top.phase == eval_phase::for_body;
    state.stack.pop_back();
    if (out)
      break;
  }
  if (!state.stack.empty()) {
    state.stack.back().results.push_back(std::move(value));
  } else {
    state.result = std::move(value);
    state.done = true;
  }
}

// A scope for a call of `cls` with `args`: its parameters bound, extending the closure's own.
environment_ptr bind_closure_args(const closure_ptr &cls, std::span<const value_t> args) {
  auto local = environment::extend(cls->env_snapshot);
  if (cls->variadic) {
    if (cls->params.empty())
      throw dsl_error("TypeError", "closure: variadic but no param name");
    local->set(cls->params.back().name, make_list(std::vector<value_t>(args.begin(), args.end())));
  } else {
    if (args.size() != cls->params.size())
      throw dsl_error("TypeError", "closure: expected " + std::to_string(cls->params.size()) +
                                       " args, got " + std::to_string(args.size()));
    for (size_t i = 0; i < cls->params.size(); ++i)
      local->set(cls->params[i].name, args[i]);
  }
  return local;
}

// -- parking at a call (docs/STATE_EXEC_EXCEPTIONS.md 3.9) ------------------------------------

// Evaluator steps running on this thread: only the outermost one's calls can park, since a
// scheduler delivers to the process's own state, not to a generator's or a nested evaluator's.
thread_local int t_step_depth = 0;
thread_local bool t_park_forbidden = false;

struct step_depth_guard {
  step_depth_guard() { ++t_step_depth; }
  ~step_depth_guard() { --t_step_depth; }
  step_depth_guard(const step_depth_guard &) = delete;
  step_depth_guard &operator=(const step_depth_guard &) = delete;
};

// Armed around a native call the evaluator makes at its own level: the park the call asks for
// (request_await), if any.
struct await_slot {
  await_slot();
  ~await_slot();
  await_slot(const await_slot &) = delete;
  await_slot &operator=(const await_slot &) = delete;

  await_slot *outer;
  bool enabled;      // the outermost evaluator step on this thread
  std::size_t depth; // the native nesting of the call itself
  std::optional<detail::await_request> request;
};

thread_local await_slot *t_await = nullptr;

await_slot::await_slot()
    : outer(t_await), enabled(t_step_depth == 1), depth(detail::native_nesting::depth()) {
  t_await = this;
}

await_slot::~await_slot() { t_await = outer; }

// An await_call frame keeps
//   extra_strs  = {the channel its process waits on, the function's name, "retry" or "",
//                  the retried call's slot or ""},
//   extra_vals  = {the handle of what it waits for (nil: a channel)},
//   extra_lists = {the call's arguments},
// and index 0 while it waits, 1 once delivered to (results = {what was}), 2 once timed out.
enum await_slot_index : std::size_t { await_channel, await_function, await_retry, await_item };

void begin_await(eval_frame &frame, detail::await_request r) {
  frame.phase = eval_phase::await_call;
  frame.index = 0;
  frame.extra_lists.assign(1, std::move(frame.results));
  frame.results.clear();
  frame.extra_vals.assign(1, std::move(r.handle));
  frame.extra_strs = {std::move(r.channel), std::move(r.function),
                      r.retry ? std::string("retry") : std::string(),
                      r.slot ? std::to_string(*r.slot) : std::string()};
  frame.extra_env = nullptr;
  if (r.ready) {
    frame.results.assign(1, std::move(*r.ready));
    frame.index = 1;
  }
}

// A native call a job made (or the value a job's work feeds to another job) parked: its
// await_call frame goes on top, as a DSL function the job called would, and its value arrives on
// the frame waiting on the job.
void push_await(evaluator_state &state, detail::await_request r, std::span<const value_t> args) {
  std::vector<value_t> call;
  call.reserve(1 + args.size());
  call.emplace_back(symbol{r.function});
  call.insert(call.end(), args.begin(), args.end());
  eval_frame frame{.expr = make_list(std::move(call)),
                   .env = state.stack.empty() ? nullptr : state.stack.back().env};
  frame.results.assign(args.begin(), args.end());
  begin_await(frame, std::move(r));
  state.stack.push_back(std::move(frame));
}
} // namespace

list_ptr detail::shared_body_expr(const void *form, const std::vector<value_t> &body) {
  if (body.size() < 2)
    return nullptr;
  shared_bodies::slot *slot = nullptr;
  if (!t_shared_bodies_gone) {
    slot = &t_shared_bodies
                .slots[(reinterpret_cast<std::uintptr_t>(form) >> 4) % shared_bodies::kSlots];
    if (slot->form == form)
      if (list_ptr made = slot->expr.lock(); made && is_begin_of(*made, body))
        return made;
  }
  list_ptr made = std::get<list_ptr>(begin_of(body).v);
  if (slot) {
    slot->form = form;
    slot->expr = made;
  }
  return made;
}

bool can_await() {
  const await_slot *slot = t_await;
  return slot && slot->enabled && !slot->request && !t_park_forbidden &&
         detail::native_nesting::depth() == slot->depth;
}

void request_await(detail::await_request r) {
  if (!can_await())
    throw std::logic_error("request_await: " + r.function + " can't park its process here");
  t_await->request = std::move(r);
}

bool parking_forbidden() { return t_park_forbidden; }

park_forbidden_scope::park_forbidden_scope() : outer_(t_park_forbidden) { t_park_forbidden = true; }

park_forbidden_scope::~park_forbidden_scope() { t_park_forbidden = outer_; }

bool deliver_await(evaluator_state &st, value_t msg) {
  if (st.stack.empty() || st.stack.back().phase != eval_phase::await_call)
    return false;
  eval_frame &frame = st.stack.back();
  frame.results.assign(1, std::move(msg));
  frame.index = 1;
  return true;
}

bool expire_await(evaluator_state &st) {
  if (st.stack.empty() || st.stack.back().phase != eval_phase::await_call)
    return false;
  st.stack.back().index = 2;
  return true;
}

value_t detail::make_awaitable_handle(std::shared_ptr<awaitable> a) {
  auto obj = std::make_shared<data_object>();
  obj->type_name = kAwaitableType;
  obj->payload = std::move(a);
  return value_t(std::move(obj));
}

value_t detail::awaited_value(value_t v) { return make_dict({{"__awaited__", std::move(v)}}); }

const value_t *detail::awaited_of(const value_t &v) {
  const auto *d = std::get_if<dict_ptr>(&v.v);
  if (!d || !*d || (*d)->size() != 1 || (*d)->front().first != "__awaited__")
    return nullptr;
  return &(*d)->front().second;
}

std::shared_ptr<detail::awaitable> detail::awaitable_of(const value_t &handle) {
  const auto *obj = std::get_if<data_object_ptr>(&handle.v);
  if (!obj || !*obj || (*obj)->type_name != kAwaitableType)
    return nullptr;
  const auto *a = boost::any_cast<std::shared_ptr<awaitable>>(&(*obj)->payload);
  return a ? *a : nullptr;
}

// Run a slice of the deep-data work a step left, making the calls its job needs: a native
// function here, at the evaluator's own level (a native call a native function made from deeper
// than the step's own call waits for this), a DSL function as frames on the stack (the job
// resumes in the step after it returns). Once the work is done, finish the native call or macro
// expansion that started it.
bool stackless_evaluator::step_data(evaluator_state &state) {
  state.stats.increment_step(step_kind::data);
  detail::data_step_scope *data = detail::data_step_scope::current();
  const bool slicing = data && data->step_nodes() > 0;
  const std::shared_ptr<pending_data_work> work = state.data_work;
  data_job &job = *work->job;
  if (work->calling) {
    // The DSL function the job called has returned: its value is the waiting frame's last.
    work->calling = false;
    std::vector<value_t> &results = state.stack.back().results;
    value_t value = std::move(results.back());
    results.pop_back();
    job.call_returned(std::move(value));
  }
  for (;;) {
    std::optional<data_job::call_request> call = job.take_call(); // one it stopped for before
    if (!call) {
      const std::size_t before = job.nodes();
      const bool finished = job.step(slicing ? data->remaining() : 0);
      count_data_nodes(job.nodes() - before);
      if (finished)
        break;
      call = job.take_call();
      if (!call)
        return false; // the slice is spent
    }
    if (auto *cp = std::get_if<closure_ptr>(&call->fn->v)) {
      environment_ptr local = bind_closure_args(*cp, call->args);
      work->calling = true;
      push_closure_body(state, *cp, std::move(local));
      return false;
    }
    auto *fn = std::get_if<native_fn>(&call->fn->v);
    if (!fn)
      throw dsl_error("TypeError",
                      std::string(call->who) + ": not a function: " + call->fn->type_name());
    value_t value;
    await_slot parked;
    {
      native_call_scope scope(data);
      value = (*fn)(call->args);
    }
    if (parked.request) {
      // The function parked: its frame waits on top, and its value is the call's.
      if (data)
        data->take_deferred();
      work->calling = true;
      push_await(state, std::move(*parked.request), call->args);
      return false;
    }
    if (auto deferred = data ? data->take_deferred() : nullptr) {
      // The function left deep-data work of its own (a call of its own, say): it runs first, and
      // its result is the call's. A job that only forwards the call's value gives it its place.
      if (job.forwards_call() && !work->then) {
        state.data_work = work->outer;
        await_deferred(state, std::move(*deferred), work->base, work->frame_args, work->outer,
                       work->feeds_outer);
      } else {
        await_deferred(state, std::move(*deferred), work->base, work->frame_args, work, true);
      }
      return false;
    }
    job.call_returned(std::move(value));
    if (slicing && data->nodes() >= data->step_nodes())
      return false;
  }
  state.data_work = work->outer;
  value_t result = job.result();
  if (work->what == pending_data_work::kind::heap_measure) {
    // Record the measurement for the scheduler; evaluation resumes where it was.
    if (const auto *bytes = std::get_if<int64_t>(&result.v))
      state.heap_measured = static_cast<uint64_t>(*bytes);
    return state.done;
  }
  if (work->what == pending_data_work::kind::macro_expansion) {
    begin_expansion(state, std::move(result));
    return false;
  }
  if (work->then) {
    native_call_scope call(data); // `then` may end in another job, or park
    await_slot parked;
    result = work->then(std::move(result));
    if (parked.request) {
      if (data)
        data->take_deferred();
      if (work->feeds_outer) {
        // The call outer's job made parked: its frame waits on top for the job, as a DSL
        // function the job called would.
        work->outer->calling = true;
        push_await(state, std::move(*parked.request), {});
      } else {
        begin_await(state.stack.back(), std::move(*parked.request)); // the frame waiting on it
      }
      return false;
    }
    if (auto deferred = data ? data->take_deferred() : nullptr) {
      await_deferred(state, std::move(*deferred), work->base, work->frame_args, work->outer,
                     work->feeds_outer);
      return false;
    }
  }
  if (work->feeds_outer) {
    work->outer->job->call_returned(std::move(result));
    return false;
  }
  pop_frame(state, std::move(result));
  return state.done;
}

void stackless_evaluator::begin_expansion(evaluator_state &state, value_t expansion) {
  eval_frame &frame = state.stack.back();
  frame.expr = std::move(expansion);
  frame.phase = eval_phase::init;
  frame.index = 0;
  frame.results.clear();
  frame.extra_vals.clear();
  frame.extra_strs.clear();
  frame.extra_lists.clear();
  frame.extra_env = nullptr;
  frame.callee = nullptr;
}

bool stackless_evaluator::step(evaluator_state &state) {
  detail::check_not_in_async_kernel("stackless_evaluator::step");
  check_interrupted();
  const step_depth_guard depth;
  if (state.uncaught)
    rethrow_uncaught(state);
  if (state.data_step_nodes > 0 && !state.backlog)
    state.backlog = std::make_shared<detail::release_backlog>();
  // Held here so the backlog outlives this step's scope even if the step replaces the state.
  const std::shared_ptr<detail::release_backlog> backlog = state.backlog;
  detail::data_step_scope data(state.data_step_nodes, state.stats, backlog.get());
  bool done;
  try {
    if (state.reading) {
      done = step_read(state);
    } else if (state.data_work && !calling_out(state)) {
      done = step_data(state);
    } else if (backlog && !backlog->empty()) {
      // A step that only frees what earlier steps dropped.
      state.stats.increment_step(step_kind::data);
      data.drain_backlog();
      return false;
    } else if (state.done || state.stack.empty()) {
      state.done = true;
      return true;
    } else {
      state.stats.count_eval_step();
      step_frame(state);
      done = state.done;
    }
  } catch (const std::exception &e) {
    // The program's error, raised in it for a try to catch (docs/STATE_EXEC_EXCEPTIONS.md). Not
    // while the script is still being read, a kill, or a failed heap measurement (the
    // scheduler's own work): those leave the state as it is.
    if (state.reading || is_uncatchable(e) ||
        (state.data_work && state.data_work->what == pending_data_work::kind::heap_measure))
      throw;
    data.take_deferred(); // work the failed call left goes with it
    fail_step(state, e);
    done = state.done;
  }
  data.drain_backlog();
  // An exception nothing caught, raised this step or let go by a finally that ran to its end.
  if (state.uncaught)
    rethrow_uncaught(state);
  // Done only once what it dropped is freed too: the steps that free the rest come first.
  return done && !(backlog && !backlog->empty());
}

void stackless_evaluator::step_frame(evaluator_state &state) {
  auto &frame = state.stack.back();

  switch (frame.phase) {
  case eval_phase::init:
    step_init(state, frame);
    break;
  case eval_phase::eval_args:
    step_eval_args(state, frame);
    break;
  case eval_phase::apply:
    step_apply(state, frame);
    break;
  case eval_phase::if_cond:
    step_if_cond(state, frame);
    break;
  case eval_phase::if_branch:
    step_if_branch(state, frame);
    break;
  case eval_phase::begin_next:
    step_begin_next(state, frame);
    break;
  case eval_phase::while_cond:
    step_while_cond(state, frame);
    break;
  case eval_phase::while_body:
    step_while_body(state, frame);
    break;
  case eval_phase::for_body:
    step_for_body(state, frame);
    break;
  case eval_phase::set_value:
    step_set_value(state, frame);
    break;
  case eval_phase::let_bindings:
    step_let_bindings(state, frame);
    break;
  case eval_phase::let_body:
    step_let_body(state, frame);
    break;
  case eval_phase::return_value:
    step_return_value(state, frame);
    break;
  case eval_phase::eval_inner:
    step_eval_inner(state, frame);
    break;
  case eval_phase::super_args:
    step_super_args(state, frame);
    break;
  case eval_phase::defclass_methods: /* handled inline */
    break;
  case eval_phase::yield_value:
    step_yield_value(state, frame);
    break;
  case eval_phase::break_unwind:
    step_break_unwind(state, frame);
    break;
  case eval_phase::try_body:
  case eval_phase::try_handler:
    step_try_done(state, frame);
    break;
  case eval_phase::try_finally:
    step_try_finally(state, frame);
    break;
  case eval_phase::raise_value:
    step_raise_value(state, frame);
    break;
  case eval_phase::await_call:
    step_await_call(state, frame);
    break;
  }
}

void stackless_evaluator::fail_step(evaluator_state &state, const std::exception &e) {
  value_t exc = exception_from(e);
  const auto *thrown = dynamic_cast<const dsl_exception *>(&e);
  if (std::exception_ptr original = thrown ? thrown->original() : std::current_exception()) {
    state.converted = exc;
    state.converted_original = std::move(original);
  }
  annotate(state, exc);
  if (!state.stack.empty())
    state.stack.pop_back(); // the frame that raised it
  unwind_exception(state, std::move(exc));
}

void stackless_evaluator::raise(evaluator_state &state, value_t exc) {
  if (!is_exception(exc))
    throw std::invalid_argument("raise: not an exception: " + exc.type_name());
  annotate(state, exc);
  unwind_exception(state, std::move(exc));
}

value_t stackless_evaluator::run(evaluator_state &state, std::optional<uint64_t> max_steps,
                                 std::optional<double> timeout_sec,
                                 std::function<void(const value_t &)> on_complete) {
  std::lock_guard lk(eval_mu_);
  // Arm the per-thread evaluation deadline (tighten-only). A NESTED run() — e.g. a defclass
  // method body evaluated on a fresh evaluator, or any re-entrant evaluate() — INHERITS this
  // deadline through the thread-local, so a loop that lives inside a single outer step still
  // aborts at the time budget instead of running to completion past the caps.
  eval_deadline_guard deadline_guard(timeout_sec);

  state.stats.start();
  try {
    while (!state.done) {
      step(state);
      if (max_steps && state.stats.get_step_count() >= *max_steps)
        break;
      if (eval_deadline_expired()) {
        state.stats.mark_complete();
        throw evaluation_timeout("evaluation exceeded timeout");
      }
    }
    if (state.done) {
      state.stats.mark_complete();
      if (on_complete)
        on_complete(state.result);
    }
    return state.done ? state.result : nil_value;
  } catch (...) {
    state.stats.mark_complete();
    throw;
  }
}

value_t stackless_evaluator::evaluate(const value_t &expr, environment_ptr env,
                                      std::optional<double> timeout_sec,
                                      std::function<void(const value_t &)> on_complete) {
  auto st = create_state(expr, env);
  return run(st, std::nullopt, timeout_sec, std::move(on_complete));
}

value_t stackless_evaluator::evaluate_script(const std::string &script, environment_ptr env,
                                             std::optional<double> timeout_sec,
                                             std::function<void(const value_t &)> on_complete) {
  auto st = create_state(script, env);
  return run(st, std::nullopt, timeout_sec, std::move(on_complete));
}

// ---------------------------------------------------------------------------
// interrupt / pause
// ---------------------------------------------------------------------------

void stackless_evaluator::interrupt() { interrupted_.store(true, std::memory_order_release); }
void stackless_evaluator::reset_interrupt() {
  interrupted_.store(false, std::memory_order_release);
}
void stackless_evaluator::pause() { paused_.store(true, std::memory_order_release); }
void stackless_evaluator::resume() {
  paused_.store(false, std::memory_order_release);
  pause_cv_.notify_all();
}
bool stackless_evaluator::is_paused() const { return paused_.load(std::memory_order_acquire); }
void stackless_evaluator::check_interrupted() {
  if (interrupted_.load(std::memory_order_acquire))
    throw evaluation_interrupted("evaluation was interrupted");
  if (paused_.load(std::memory_order_acquire)) {
    std::unique_lock lk(pause_mu_);
    pause_cv_.wait(lk, [this] { return !paused_.load(std::memory_order_acquire); });
  }
}

// ---------------------------------------------------------------------------
// frame management
// ---------------------------------------------------------------------------

void stackless_evaluator::pop_frame(evaluator_state &state, value_t result) {
  pop_top(state.stack);
  if (!state.stack.empty()) {
    state.stack.back().results.push_back(std::move(result));
  } else {
    state.result = std::move(result);
    state.done = true;
  }
}

void stackless_evaluator::push_frame(evaluator_state &state, const value_t &expr,
                                     const environment_ptr &env) {
  // Copied before the stack grows: `expr` and `env` may be in the top frame, which growing moves,
  // and a copy that throws (out of memory) leaves the stack as it was.
  value_t e = expr;
  environment_ptr scope = env;
  push_new_frame(state.stack, std::move(e), std::move(scope));
}

void stackless_evaluator::push_frame(evaluator_state &state, value_t &&expr,
                                     environment_ptr &&env) {
  push_new_frame(state.stack, std::move(expr), std::move(env));
}

// ---------------------------------------------------------------------------
// step_init — dispatch on expression type
// ---------------------------------------------------------------------------

void stackless_evaluator::step_init(evaluator_state &state, eval_frame &frame) {
  auto &expr = frame.expr;
  auto &env = frame.env;

  // Symbol
  if (auto *sym = std::get_if<symbol>(&expr.v)) {
    if (sym->name == "nil") {
      hand_up(state, nil_value);
      return;
    }
    if (sym->name == "t") {
      hand_up(state, true_value);
      return;
    }
    auto *val = env->lookup(sym->name);
    if (!val)
      throw dsl_error("NameError", "undefined symbol: " + sym->name);
    hand_up(state, *val);
    return;
  }

  // Non-list atom: its own value (the frame's copy of it goes with the frame)
  if (!std::holds_alternative<list_ptr>(expr.v)) {
    hand_up(state, std::move(expr));
    return;
  }

  auto *lp = std::get_if<list_ptr>(&expr.v);
  if (!lp || !*lp) {
    pop_frame(state, expr);
    return;
  }
  auto &elems = **lp;
  if (elems.empty()) {
    pop_frame(state, make_list());
    return;
  }

  auto *head = std::get_if<symbol>(&elems[0].v);
  if (!head)
    throw dsl_error("TypeError", "invalid expression head: " + to_string(elems[0]));

  const auto &name = head->name;
  // The form's arguments, read in place: what a frame keeps across steps it copies (a program
  // may change its own code meanwhile, and a frame evaluates the form as it was).
  const std::span<const value_t> args(elems.data() + 1, elems.size() - 1);
  const detail::special_form form = detail::special_form_of(name);

  // --- Special-form gate (capability control) ---
  // If this evaluator is restricted, a special form NOT on the allowlist is refused. A
  // non-special-form head falls through to normal application (the environment governs it).
  if (allowed_forms_ && form != detail::special_form::none && !allowed_forms_->count(name))
    throw dsl_error("RuntimeError", "special form '" + name + "' is not permitted in this context");

  // --- Special forms ---
  switch (form) {
  case detail::special_form::none:
    break;
  case detail::special_form::if_: {
    if (args.size() < 2 || args.size() > 3)
      throw dsl_error("TypeError", "if: expected a condition, a then branch and an optional else "
                                   "branch, got " +
                                       std::to_string(args.size()) + " args");
    frame.phase = eval_phase::if_cond;
    fill_values(frame.extra_vals, args);
    make_room(frame.results, 2); // the condition's value, then the branch's
    push_frame(state, frame.extra_vals[0], env);
    return;
  }
  case detail::special_form::begin: {
    if (args.empty()) {
      pop_frame(state, nil_value);
      return;
    }
    frame.phase = eval_phase::begin_next;
    fill_values(frame.extra_vals, args);
    make_room(frame.results, args.size());
    frame.index = 0;
    push_frame(state, frame.extra_vals[0], env);
    return;
  }
  case detail::special_form::while_: {
    if (args.size() < 2)
      throw dsl_error("TypeError", "while: expected a condition and a body");
    frame.phase = eval_phase::while_cond;
    make_room(frame.extra_vals, 2);
    frame.extra_vals.push_back(args[0]);            // [0]=cond
    frame.extra_vals.push_back(loop_body(args, 1)); // [1]=body
    make_room(frame.results, 1);
    push_frame(state, frame.extra_vals[0], env);
    return;
  }
  case detail::special_form::for_: {
    if (args.size() < 3)
      throw dsl_error("TypeError", "for: expected a variable, a collection and a body");
    auto *var_sym = std::get_if<symbol>(&args[0].v);
    if (!var_sym)
      throw dsl_error("TypeError", "for: first arg must be symbol");
    frame.phase = eval_phase::for_body;
    set_one_string(frame.extra_strs, var_sym->name);
    make_room(frame.extra_vals, 1);
    frame.extra_vals.push_back(loop_body(args, 2)); // body expression
    frame.extra_lists.clear();                      // will hold iterator
    make_room(frame.results, 1);
    frame.index = -1;                // -1 = still evaluating collection
    push_frame(state, args[1], env); // evaluate collection
    return;
  }
  case detail::special_form::set: {
    if (args.size() != 2)
      throw dsl_error("TypeError", "set: expected 2 args");
    auto *var_sym = std::get_if<symbol>(&args[0].v);
    if (!var_sym)
      throw dsl_error("TypeError", "set: first arg must be symbol");
    frame.phase = eval_phase::set_value;
    set_one_string(frame.extra_strs, var_sym->name);
    make_room(frame.results, 1);
    push_frame(state, args[1], env);
    return;
  }
  case detail::special_form::quote: {
    if (args.size() != 1)
      throw dsl_error("TypeError", "quote: expected 1 arg");
    pop_frame(state, args[0]);
    return;
  }
  case detail::special_form::lambda: {
    if (args.size() < 2)
      throw dsl_error("TypeError", "lambda: expected at least 2 args");
    std::vector<symbol> params;
    bool variadic = false;
    if (auto *pl = std::get_if<list_ptr>(&args[0].v); pl && *pl) {
      for (auto &p : **pl) {
        auto *ps = std::get_if<symbol>(&p.v);
        if (!ps)
          throw dsl_error("TypeError", "lambda: param must be symbol");
        if (ps->name == "&rest") {
          variadic = true;
          continue;
        }
        params.push_back(*ps);
      }
    }
    auto cls = std::make_shared<closure>();
    cls->params = std::move(params);
    cls->variadic = variadic;
    cls->body = std::vector<value_t>(args.begin() + 1, args.end());
    cls->body_expr = detail::shared_body_expr(lp->get(), cls->body);
    cls->env_snapshot = env;
    track_captured_env(env); // bound back into env (or anything it holds), it is a cycle
    pop_frame(state, value_t{std::move(cls)});
    return;
  }
  case detail::special_form::return_: {
    if (args.size() != 1)
      throw dsl_error("TypeError", "return: expected 1 arg, got " + std::to_string(args.size()));
    frame.phase = eval_phase::return_value;
    push_frame(state, args[0], env);
    return;
  }
  case detail::special_form::yield: {
    if (args.size() != 1)
      throw dsl_error("TypeError", "yield: expected 1 arg, got " + std::to_string(args.size()));
    frame.phase = eval_phase::yield_value;
    push_frame(state, args[0], env);
    return;
  }
  case detail::special_form::break_: {
    if (args.size() > 1)
      throw dsl_error("TypeError",
                      "break: expected at most 1 arg, got " + std::to_string(args.size()));
    frame.phase = eval_phase::break_unwind;
    if (args.empty()) {
      // (break) — no value, use nil
      frame.results.push_back(nil_value);
    } else {
      // (break val) — evaluate the value first
      push_frame(state, args[0], env);
      return;
    }
    // Fall through to step_break_unwind on next step
    return;
  }
  case detail::special_form::let: {
    if (args.size() < 2)
      throw dsl_error("TypeError", "let: expected at least 2 args");
    auto *bindings_lp = std::get_if<list_ptr>(&args[0].v);
    if (!bindings_lp)
      throw dsl_error("TypeError", "let: first arg must be binding list");
    auto local = environment::extend(env);
    frame.phase = eval_phase::let_bindings;
    fill_values(frame.extra_vals, args); // [0]=bindings, [1..]=body
    frame.extra_env = local;
    frame.index = 0;
    if (!(**bindings_lp).empty()) {
      auto &first_binding = (**bindings_lp)[0];
      auto *bp = std::get_if<list_ptr>(&first_binding.v);
      if (!bp || (*bp)->size() != 2)
        throw dsl_error("TypeError", "let: each binding must be (name value)");
      push_frame(state, (**bp)[1], local);
    } else {
      // No bindings, go to body
      frame.phase = eval_phase::let_body;
      frame.index = 0;
      auto body_begin = 1;
      if (static_cast<size_t>(body_begin) < frame.extra_vals.size()) {
        push_frame(state, frame.extra_vals[body_begin], local);
      } else {
        pop_frame(state, nil_value);
      }
    }
    return;
  }
  case detail::special_form::defun: {
    if (args.size() < 3)
      throw dsl_error("TypeError", "defun: expected at least 3 args");
    auto *name_sym = std::get_if<symbol>(&args[0].v);
    if (!name_sym)
      throw dsl_error("TypeError", "defun: first arg must be symbol");
    // Build closure directly
    std::vector<symbol> params;
    bool variadic = false;
    if (auto *pl = std::get_if<list_ptr>(&args[1].v); pl && *pl) {
      for (auto &p : **pl) {
        auto *ps = std::get_if<symbol>(&p.v);
        if (!ps)
          throw dsl_error("TypeError", "defun: param must be symbol");
        if (ps->name == "&rest") {
          variadic = true;
          continue;
        }
        params.push_back(*ps);
      }
    }
    auto cls = std::make_shared<closure>();
    cls->params = std::move(params);
    cls->variadic = variadic;
    cls->body = std::vector<value_t>(args.begin() + 2, args.end());
    cls->body_expr = detail::shared_body_expr(lp->get(), cls->body);
    cls->env_snapshot = env;
    track_captured_env(env); // bound in the env it captured: a cycle
    env->set(name_sym->name, value_t{cls});
    pop_frame(state, value_t{std::move(cls)});
    return;
  }
  case detail::special_form::defmacro: {
    if (args.size() < 3)
      throw dsl_error("TypeError", "defmacro: expected at least 3 args");
    auto *name_sym = std::get_if<symbol>(&args[0].v);
    if (!name_sym)
      throw dsl_error("TypeError", "defmacro: first arg must be symbol");
    std::vector<symbol> params;
    if (auto *pl = std::get_if<list_ptr>(&args[1].v); pl && *pl) {
      for (auto &p : **pl) {
        auto *ps = std::get_if<symbol>(&p.v);
        if (!ps)
          throw dsl_error("TypeError", "defmacro: param must be symbol");
        params.push_back(*ps);
      }
    }
    value_t body = (args.size() == 3) ? args[2] : make_list([&] {
      std::vector<value_t> v{value_t{symbol{"begin"}}};
      v.insert(v.end(), args.begin() + 2, args.end());
      return v;
    }());
    state.user_macros[name_sym->name] = {std::move(params), body};
    pop_frame(state, value_t{symbol{name_sym->name}});
    return;
  }
  case detail::special_form::eval: {
    if (args.size() != 1)
      throw dsl_error("TypeError", "eval: expected 1 arg");
    frame.phase = eval_phase::eval_inner;
    push_frame(state, args[0], env);
    return;
  }
  case detail::special_form::root: {
    pop_frame(state, state.root_expr);
    return;
  }
  case detail::special_form::super: {
    if (args.size() < 2)
      throw dsl_error("TypeError", "super: expected at least 2 args");
    auto *method_sym = std::get_if<symbol>(&args[1].v);
    frame.phase = eval_phase::super_args;
    frame.extra_strs = {method_sym ? method_sym->name : ""};
    fill_values(frame.extra_vals, args.subspan(2));
    frame.index = 0;
    push_frame(state, args[0], env); // evaluate self
    return;
  }
  case detail::special_form::defclass: {
    handle_defclass(state, frame, args);
    return;
  }
  case detail::special_form::try_: {
    // (try body... (catch (Type var) handler...)... (finally cleanup...))
    try_form form = parse_try(args);
    frame.phase = eval_phase::try_body;
    make_room(frame.extra_vals, 3); // moved in, not copied from an initializer list
    frame.extra_vals.push_back(body_of(form.body));
    frame.extra_vals.push_back(std::move(form.cleanup));
    frame.extra_vals.push_back(nil_value);
    set_one_string(frame.extra_strs, std::string());
    frame.extra_lists.clear();
    frame.extra_lists.push_back(std::move(form.catches));
    make_room(frame.results, 1);
    push_frame(state, frame.extra_vals[try_body_slot], env);
    return;
  }
  case detail::special_form::raise: {
    if (args.size() > 1)
      throw dsl_error("TypeError",
                      "raise: expected at most 1 arg, got " + std::to_string(args.size()));
    if (args.empty()) {
      // (raise) in a catch handler re-raises the exception it handles.
      const value_t *handled = handled_exception(state, false);
      if (!handled)
        throw dsl_error("RuntimeError", "raise: no exception is being handled");
      throw dsl_exception(*handled);
    }
    frame.phase = eval_phase::raise_value;
    push_frame(state, args[0], env);
    return;
  }
  }

  // User macros — expand then re-evaluate
  if (auto it = state.user_macros.empty() ? state.user_macros.end() : state.user_macros.find(name);
      it != state.user_macros.end()) {
    auto &[macro_params, body] = it->second;
    std::unordered_map<std::string, value_t> subst;
    for (size_t i = 0; i < macro_params.size(); ++i)
      subst[macro_params[i].name] = (i < args.size()) ? args[i] : nil_value;
    // Re-init current frame with expanded form for evaluation. The expansion is deep-data work:
    // a step that slices does one slice of it here and leaves the rest to step_data().
    auto job = make_substitute_job(body, std::move(subst));
    detail::data_step_scope *data = detail::data_step_scope::current();
    const bool finished = job->step(data && data->step_nodes() > 0 ? data->remaining() : 0);
    count_data_nodes(job->nodes());
    if (!finished) {
      state.data_work = make_work(std::move(job), {}, pending_data_work::kind::macro_expansion,
                                  state.stack.size(), frame.results.size(), state.data_work, false);
      return;
    }
    begin_expansion(state, job->result());
    return;
  }

  // Built-in or user function: it must exist now; the apply step uses what is found here unless
  // the scopes changed meanwhile (callee_of).
  detail::scope_mark mark;
  const auto *callee = detail::find_marked(env.get(), name, mark);
  if (!callee)
    throw dsl_error("NameError", "unknown function or special form: " + name);
  frame.callee = callee;
  frame.callee_depth = mark.depth;
  frame.callee_stamp = mark.stamp;

  frame.phase = eval_phase::eval_args;
  fill_values(frame.extra_vals, args);
  make_room(frame.results, args.size()); // the arguments' values
  frame.index = 0;
  if (!frame.extra_vals.empty()) {
    push_frame(state, frame.extra_vals[0], env);
  } else {
    frame.phase = eval_phase::apply;
  }
}

// ---------------------------------------------------------------------------
// step: eval_args
// ---------------------------------------------------------------------------

void stackless_evaluator::step_eval_args(evaluator_state &state, eval_frame &frame) {
  frame.index++;
  if (static_cast<size_t>(frame.index) < frame.extra_vals.size()) {
    push_frame(state, frame.extra_vals[frame.index], frame.env);
  } else {
    frame.phase = eval_phase::apply;
  }
}

// ---------------------------------------------------------------------------
// step: apply
// ---------------------------------------------------------------------------

void stackless_evaluator::step_apply(evaluator_state &state, eval_frame &frame) {
  auto &evaled = frame.results;

  // Determine the function
  auto *head_sym = std::get_if<symbol>(&(*std::get<list_ptr>(frame.expr.v))[0].v);
  if (!head_sym)
    throw dsl_error("TypeError", "apply: no function head");

  const value_t *func_val = callee_of(frame, head_sym->name);
  if (!func_val)
    throw dsl_error("NameError", "apply: undefined function: " + head_sym->name);

  if (auto *fn = std::get_if<native_fn>(&func_val->v)) {
    apply_native(state, frame, *fn, evaled);
    return;
  }
  if (auto *cp = std::get_if<closure_ptr>(&func_val->v)) {
    call_closure(state, frame, *cp, evaled);
    return;
  }
  throw dsl_error("TypeError", "apply: not callable: " + head_sym->name);
}

void stackless_evaluator::apply_native(evaluator_state &state, eval_frame &frame,
                                       const native_fn &fn, std::span<const value_t> args) {
  detail::data_step_scope *data = detail::data_step_scope::current();
  value_t result;
  await_slot parked;
  {
    native_call_scope call(data);
    result = fn(args);
  }
  if (parked.request) {
    // The call parked its process: this frame waits for what its scheduler delivers.
    if (data)
      data->take_deferred();
    begin_await(frame, std::move(*parked.request));
    return;
  }
  if (auto deferred = data ? data->take_deferred() : nullptr) {
    // The call left deep-data work: this frame waits while step_data() runs it.
    await_deferred(state, std::move(*deferred), state.stack.size(), frame.results.size(),
                   state.data_work, false);
    return;
  }
  hand_up(state, std::move(result));
}

// ---------------------------------------------------------------------------
// step: if
// ---------------------------------------------------------------------------

void stackless_evaluator::step_if_cond(evaluator_state &state, eval_frame &frame) {
  bool cond = frame.results.back().is_truthy();
  auto &args = frame.extra_vals;
  frame.phase = eval_phase::if_branch;
  if (cond) {
    push_frame(state, args[1], frame.env);
  } else if (args.size() > 2) {
    push_frame(state, args[2], frame.env);
  } else {
    pop_frame(state, nil_value);
  }
}

void stackless_evaluator::step_if_branch(evaluator_state &state, eval_frame &frame) {
  hand_up(state, std::move(frame.results.back()));
}

// ---------------------------------------------------------------------------
// step: begin
// ---------------------------------------------------------------------------

void stackless_evaluator::step_begin_next(evaluator_state &state, eval_frame &frame) {
  auto &args = frame.extra_vals;
  frame.index++;
  if (static_cast<size_t>(frame.index) < args.size()) {
    push_frame(state, args[frame.index], frame.env);
  } else {
    hand_up(state, std::move(frame.results.back()));
  }
}

// ---------------------------------------------------------------------------
// step: while
// ---------------------------------------------------------------------------

void stackless_evaluator::step_while_cond(evaluator_state &state, eval_frame &frame) {
  if (frame.results.back().is_truthy()) {
    frame.phase = eval_phase::while_body;
    frame.results.clear();
    push_frame(state, frame.extra_vals[1], frame.env);
  } else {
    pop_frame(state, nil_value);
  }
}

void stackless_evaluator::step_while_body(evaluator_state &state, eval_frame &frame) {
  frame.phase = eval_phase::while_cond;
  frame.results.clear();
  push_frame(state, frame.extra_vals[0], frame.env);
}

// ---------------------------------------------------------------------------
// step: for
// ---------------------------------------------------------------------------

// The names a for frame's generator call (advance_generator) binds in its extra_env. A user's
// script cannot reach that scope: it is the call's own.
const char *const kForGenerator = "%generator";
const char *const kForGeneratorNext = "%generator-next";

void stackless_evaluator::step_for_body(evaluator_state &state, eval_frame &frame) {
  // A DSL generator's next value came back from the call advance_generator() pushed: a list of
  // the value, or empty once the generator is exhausted. extra_strs[1] marks the wait.
  if (frame.extra_strs.size() > 1) {
    frame.extra_strs.pop_back();
    const value_t got = std::move(frame.results.back());
    frame.results.clear();
    const auto &l = *std::get<list_ptr>(got.v);
    if (l.empty()) {
      pop_frame(state, nil_value);
      return;
    }
    frame.env->set(frame.extra_strs[0], l.front());
    push_frame(state, frame.extra_vals[0], frame.env);
    return;
  }

  if (frame.index == -1) {
    // Collection was just evaluated
    auto collection = frame.results.back();
    if (std::holds_alternative<generator_ptr>(collection.v)) {
      // Generator: store in extra_vals[1] for lazy iteration
      frame.extra_vals.push_back(collection);
      frame.extra_lists = {{}}; // placeholder
      frame.index = 0;
      frame.results.clear();
    } else {
      auto *lp = std::get_if<list_ptr>(&collection.v);
      if (!lp)
        throw dsl_error("TypeError", "for: collection must be a list or generator");
      // The list as it is now (one copy: the loop sees later changes to it no more than before).
      frame.extra_lists.clear();
      frame.extra_lists.emplace_back((*lp)->begin(), (*lp)->end());
      frame.index = 0;
      frame.results.clear();
    }
  } else {
    frame.index++;
    frame.results.clear();
  }

  // Generator path: extra_vals has size > 1 when a generator is stored
  if (frame.extra_vals.size() > 1) {
    auto *gp = std::get_if<generator_ptr>(&frame.extra_vals[1].v);
    if (gp && *gp && !(*gp)->native_step) {
      advance_generator(state, frame);
      return;
    }
    if (gp) { // range and friends: one cheap value, here
      auto next = *gp ? generator_next(**gp) : std::nullopt;
      if (!next) {
        pop_frame(state, nil_value);
        return;
      }
      frame.env->set(frame.extra_strs[0], *next);
      push_frame(state, frame.extra_vals[0], frame.env);
      return;
    }
  }

  // List path
  auto &iter = frame.extra_lists[0];
  if (static_cast<size_t>(frame.index) < iter.size()) {
    frame.env->set(frame.extra_strs[0], iter[frame.index]);
    push_frame(state, frame.extra_vals[0], frame.env);
  } else {
    pop_frame(state, nil_value);
  }
}

void stackless_evaluator::advance_generator(evaluator_state &state, eval_frame &frame) {
  // The call (%generator-next %generator), in a scope of the frame's own: an ordinary native
  // call, so the job it runs (make_generator_job) slices and waits like any other, and its
  // value comes back to this frame.
  if (!frame.extra_env) {
    frame.extra_env = environment::extend(frame.env);
    frame.extra_env->set(kForGenerator, frame.extra_vals[1]);
    frame.extra_env->set(
        kForGeneratorNext, value_t{native_fn{[](std::span<const value_t> args) -> value_t {
          return run_data_job(make_generator_job(std::get<generator_ptr>(args[0].v)));
        }}});
  }
  frame.extra_strs.push_back(kForGeneratorNext); // waiting for the value
  environment_ptr scope = frame.extra_env;
  push_frame(state, make_list({value_t{symbol{kForGeneratorNext}}, value_t{symbol{kForGenerator}}}),
             std::move(scope)); // last: it may move `frame`
}

// ---------------------------------------------------------------------------
// step: set
// ---------------------------------------------------------------------------

void stackless_evaluator::step_set_value(evaluator_state &state, eval_frame &frame) {
  // The value is bound (a copy) and passed on (moved): its own copy goes with the frame.
  frame.env->set(frame.extra_strs[0], frame.results.back());
  hand_up(state, std::move(frame.results.back()));
}

// ---------------------------------------------------------------------------
// step: let
// ---------------------------------------------------------------------------

void stackless_evaluator::step_let_bindings(evaluator_state &state, eval_frame &frame) {
  auto &bindings_val = frame.extra_vals[0];
  auto &bindings = *std::get<list_ptr>(bindings_val.v);
  auto &local = frame.extra_env;

  // Store binding result
  auto &binding = bindings[frame.index];
  auto &bp = *std::get<list_ptr>(binding.v);
  auto *sym = std::get_if<symbol>(&bp[0].v);
  local->set(sym->name, frame.results.back());
  frame.index++;

  if (static_cast<size_t>(frame.index) < bindings.size()) {
    auto &next_binding = bindings[frame.index];
    auto &nbp = *std::get<list_ptr>(next_binding.v);
    push_frame(state, nbp[1], local);
  } else {
    // All bindings done, evaluate body
    frame.phase = eval_phase::let_body;
    frame.index = 1; // body starts at extra_vals[1]
    frame.results.clear();
    if (frame.extra_vals.size() > 1) {
      push_frame(state, frame.extra_vals[1], local);
    } else {
      pop_frame(state, nil_value);
    }
  }
}

void stackless_evaluator::step_let_body(evaluator_state &state, eval_frame &frame) {
  frame.index++;
  if (static_cast<size_t>(frame.index) < frame.extra_vals.size()) {
    push_frame(state, frame.extra_vals[frame.index], frame.extra_env);
  } else {
    hand_up(state, std::move(frame.results.back()));
  }
}

// ---------------------------------------------------------------------------
// step: return
// ---------------------------------------------------------------------------

void stackless_evaluator::step_return_value(evaluator_state &state, eval_frame &frame) {
  // Unwind to the function's caller; a DSL function a deep-data job called returns to the job.
  unwind_jump(state, true, frame.results.back());
}

// ---------------------------------------------------------------------------
// step: yield (generator support)
// ---------------------------------------------------------------------------

void stackless_evaluator::step_yield_value(evaluator_state &state, eval_frame &frame) {
  auto val = frame.results.back();
  state.result = val; // Store value where generator_next can find it
  pop_frame(state, val);
  state.yielded = true; // Signal generator_next to stop stepping
}

// ---------------------------------------------------------------------------
// step: break (loop exit)
// ---------------------------------------------------------------------------

void stackless_evaluator::step_break_unwind(evaluator_state &state, eval_frame &frame) {
  // Unwind out of the innermost loop, but not out of a DSL function a deep-data job called:
  // there the value is the function's.
  unwind_jump(state, false, frame.results.back());
}

// ---------------------------------------------------------------------------
// step: try / raise
// ---------------------------------------------------------------------------

void stackless_evaluator::step_try_done(evaluator_state &state, eval_frame &frame) {
  // The body or a handler finished: its value is the try's, once the finally has run.
  value_t value = std::move(frame.results.back());
  if (has_finally(frame))
    start_finally(state, frame, "value", std::move(value));
  else
    pop_frame(state, std::move(value));
}

void stackless_evaluator::step_try_finally(evaluator_state &state, eval_frame &frame) {
  // The cleanup finished (its value is dropped): leave the way the try was leaving.
  const std::string way = frame.extra_strs[0];
  value_t payload = std::move(frame.extra_vals[try_payload_slot]);
  if (way == "raise")
    unwind_exception(state, std::move(payload)); // from this try, past its catches
  else if (way == "return" || way == "break")
    unwind_jump(state, way == "return", std::move(payload));
  else
    pop_frame(state, std::move(payload));
}

void stackless_evaluator::step_raise_value(evaluator_state & /*state*/, eval_frame &frame) {
  const value_t &exc = frame.results.back();
  if (!is_exception(exc))
    throw dsl_error("TypeError", "raise: expected an exception, got " + exc.type_name());
  throw dsl_exception(exc);
}

// ---------------------------------------------------------------------------
// step: eval
// ---------------------------------------------------------------------------

void stackless_evaluator::step_await_call(evaluator_state &state, eval_frame &frame) {
  const std::string function = frame.extra_strs[await_function];
  const std::shared_ptr<detail::awaitable> awaited = detail::awaitable_of(frame.extra_vals[0]);
  const std::vector<std::pair<std::string, value_t>> fields = {{"function", value_t(function)}};
  if (frame.index == 2)
    throw dsl_error("TimeoutError", function + ": no result within the time it was given", "async",
                    fields);
  const bool on_channel = frame.extra_vals[0].is_nil();
  if (frame.index == 0 && !on_channel && !(awaited && awaited->finished()))
    // Stepped while it waits: a process restored from a snapshot, whose call is gone.
    throw dsl_error("RuntimeError",
                    function + ": the call can't resume (the process was restored from a snapshot "
                               "while it waited)",
                    "async", fields);
  // Stepped while it waits on a channel (restored from a snapshot): apply the call again, and it
  // waits again.
  const bool again = frame.index == 0 && on_channel;
  if (again || frame.extra_strs[await_retry] == "retry") {
    if (frame.index == 1 && frame.extra_strs.size() > await_item &&
        !frame.extra_strs[await_item].empty()) {
      // What was delivered takes its item's place in the call's list, as itself.
      std::vector<value_t> &args = frame.extra_lists[0];
      const std::size_t item = std::stoul(frame.extra_strs[await_item]);
      const list_ptr *lp = args.empty() ? nullptr : std::get_if<list_ptr>(&args[0].v);
      if (!lp || !*lp || item >= (*lp)->size())
        throw std::logic_error(function + ": the retried call's list has no item " +
                               frame.extra_strs[await_item]);
      std::vector<value_t> items = **lp;
      items[item] = detail::awaited_value(std::move(frame.results[0]));
      args[0] = make_list(std::move(items));
    }
    // Apply the call again, with the same arguments.
    frame.phase = eval_phase::apply;
    frame.index = 0;
    frame.results = std::move(frame.extra_lists[0]);
    frame.extra_lists.clear();
    frame.extra_vals.clear();
    frame.extra_strs.clear();
    return;
  }
  value_t value = awaited ? awaited->outcome() : std::move(frame.results[0]);
  pop_frame(state, std::move(value));
}

void stackless_evaluator::step_eval_inner(evaluator_state &state, eval_frame &frame) {
  value_t code = frame.results.back();
  environment_ptr env = frame.env;
  const bool body = frame.function_body; // an (eval ...) that is a function's body stays it
  state.stack.pop_back();
  push_frame(state, std::move(code), std::move(env));
  state.stack.back().function_body = body;
}

// ---------------------------------------------------------------------------
// step: super
// ---------------------------------------------------------------------------

void stackless_evaluator::step_super_args(evaluator_state &state, eval_frame &frame) {
  auto &remaining = frame.extra_vals;
  if (static_cast<size_t>(frame.index) < remaining.size()) {
    frame.index++;
    push_frame(state, remaining[frame.index - 1], frame.env);
  } else {
    // results[0] = self, rest are method args
    auto &self_val = frame.results[0];
    std::string method_name = frame.extra_strs[0];

    auto &obj_dict = *std::get<dict_ptr>(self_val.v);
    value_t *cls_val = nullptr;
    for (auto &[k, v] : obj_dict)
      if (k == "__class__") {
        cls_val = &v;
        break;
      }
    if (!cls_val)
      throw dsl_error("TypeError", "super: no __class__");
    auto &cls_dict = *std::get<dict_ptr>(cls_val->v);
    value_t *super_val = nullptr;
    for (auto &[k, v] : cls_dict)
      if (k == "__super__") {
        super_val = &v;
        break;
      }
    if (!super_val)
      throw dsl_error("TypeError", "super: no __super__");

    auto *search = &(*std::get<dict_ptr>(super_val->v));
    while (search) {
      for (auto &[k, v] : *search) {
        if (k == method_name) {
          if (auto *fn = std::get_if<native_fn>(&v.v)) {
            std::vector<value_t> call_args;
            call_args.push_back(self_val);
            for (size_t i = 1; i < frame.results.size(); ++i)
              call_args.push_back(frame.results[i]);
            const native_fn method = *fn; // it may change the class dict under `v`
            apply_native(state, frame, method, call_args);
            return;
          }
          if (auto *cp = std::get_if<closure_ptr>(&v.v)) {
            std::vector<value_t> call_args;
            call_args.push_back(self_val);
            for (size_t i = 1; i < frame.results.size(); ++i)
              call_args.push_back(frame.results[i]);
            call_closure(state, frame, *cp, call_args);
            return;
          }
        }
      }
      value_t *sp = nullptr;
      for (auto &[k, v] : *search)
        if (k == "__super__") {
          sp = &v;
          break;
        }
      if (!sp)
        break;
      search = &(*std::get<dict_ptr>(sp->v));
    }
    throw dsl_error("KeyError", "super: method '" + method_name + "' not found");
  }
}

// ---------------------------------------------------------------------------
// closure call
// ---------------------------------------------------------------------------

void stackless_evaluator::call_closure(evaluator_state &state, eval_frame & /*frame*/,
                                       const closure_ptr &cls, const std::vector<value_t> &args) {
  // Everything the call needs is taken from `cls` and `args` before the frame goes: they may be
  // in it, or in a scope only it holds (a function passed as a parameter, called in tail
  // position: its binding is in the scope this frame evaluates in).
  environment_ptr local = bind_closure_args(cls, args);
  value_t body = detail::body_expr_of(*cls);

  // Replace current frame with body evaluation
  pop_top(state.stack);
  push_body_frame(state.stack, std::move(body), std::move(local));
}

void stackless_evaluator::push_closure_body(evaluator_state &state, const closure_ptr &cls,
                                            environment_ptr local) {
  push_body_frame(state.stack, detail::body_expr_of(*cls), std::move(local));
}

// ---------------------------------------------------------------------------
// defclass
// ---------------------------------------------------------------------------

void stackless_evaluator::handle_defclass(evaluator_state &state, eval_frame &frame,
                                          std::span<const value_t> args) {
  if (args.empty())
    throw dsl_error("TypeError", "defclass: expected at least a name");
  auto *name_sym = std::get_if<symbol>(&args[0].v);
  if (!name_sym)
    throw dsl_error("TypeError", "defclass: first arg must be symbol");
  const std::string &cls_name = name_sym->name;
  auto &env = frame.env;

  size_t methods_start = 1;
  dict_ptr super_cls_dict;

  // Check for parent class
  if (args.size() > 1) {
    if (auto *parent_sym = std::get_if<symbol>(&args[1].v)) {
      auto *parent_val = env->lookup(parent_sym->name);
      if (parent_val) {
        auto *cd = env->lookup(parent_sym->name + ".__cls_dict__");
        if (cd)
          super_cls_dict = std::get<dict_ptr>(cd->v);
      }
      methods_start = 2;
    } else if (auto *plp = std::get_if<list_ptr>(&args[1].v)) {
      auto &pl = **plp;
      if (pl.size() == 1) {
        if (auto *ps = std::get_if<symbol>(&pl[0].v)) {
          auto *cd = env->lookup(ps->name + ".__cls_dict__");
          if (cd)
            super_cls_dict = std::get<dict_ptr>(cd->v);
          methods_start = 2;
        }
      }
    }
  }

  auto cls_dict = std::make_shared<std::vector<std::pair<std::string, value_t>>>();
  cls_dict->emplace_back("__name__", value_t{cls_name});
  if (super_cls_dict)
    cls_dict->emplace_back("__super__", value_t{super_cls_dict});

  // Collect method names
  std::vector<std::string> method_names;
  for (size_t i = methods_start; i < args.size(); ++i) {
    auto *ml = std::get_if<list_ptr>(&args[i].v);
    if (ml && *ml && !(**ml).empty()) {
      if (auto *ms = std::get_if<symbol>(&(**ml)[0].v))
        method_names.push_back(ms->name);
    }
  }
  auto has_method = [&](const std::string &n) {
    for (auto &m : method_names)
      if (m == n)
        return true;
    return false;
  };

  // Default init and finalize: the superclass's, else init returns self and finalize nil. The
  // natives are named types (closure_natives.h) so the cycle collector sees what they hold.
  if (!has_method("init"))
    cls_dict->emplace_back(
        "init", value_t{native_fn{detail::class_inherited_method{super_cls_dict, "init"}}});
  if (!has_method("finalize"))
    cls_dict->emplace_back(
        "finalize", value_t{native_fn{detail::class_inherited_method{super_cls_dict, "finalize"}}});

  // Process methods
  for (size_t i = methods_start; i < args.size(); ++i) {
    auto *ml = std::get_if<list_ptr>(&args[i].v);
    if (!ml || !*ml || (**ml).empty())
      continue;
    auto &mlist = **ml;
    auto *msym = std::get_if<symbol>(&mlist[0].v);
    if (!msym)
      continue;

    // Build closure
    std::vector<symbol> params;
    bool variadic = false;
    if (auto *pl = std::get_if<list_ptr>(&mlist[1].v); pl && *pl) {
      for (auto &p : **pl) {
        auto *ps = std::get_if<symbol>(&p.v);
        if (!ps)
          throw dsl_error("TypeError", "defclass: param must be symbol");
        if (ps->name == "&rest") {
          variadic = true;
          continue;
        }
        params.push_back(*ps);
      }
    }
    auto mcls = std::make_shared<closure>();
    mcls->params = std::move(params);
    mcls->variadic = variadic;
    mcls->body = std::vector<value_t>(mlist.begin() + 2, mlist.end());
    mcls->body_expr = detail::shared_body_expr(&mlist, mcls->body);
    mcls->env_snapshot = env;
    track_captured_env(env); // the class is bound in env: a cycle candidate

    // A method is a closure taking self first: send and super call it like any function, its
    // body running as frames of the calling process (call_function), not on an evaluator of
    // its own.
    cls_dict->emplace_back(msym->name, value_t{std::move(mcls)});
  }

  // Constructor
  env->set(cls_name, value_t{native_fn{detail::class_constructor{cls_dict}}});
  env->set(cls_name + ".__cls_dict__", value_t{cls_dict});
  pop_frame(state, *env->lookup(cls_name));
}

namespace {
bool is_callable(const value_t &v) {
  const auto *cls = std::get_if<closure_ptr>(&v.v);
  return std::holds_alternative<native_fn>(v.v) || (cls && *cls);
}
} // namespace

value_t detail::class_constructor::operator()(std::span<const value_t> init_args) const {
  auto inst = std::make_shared<std::vector<std::pair<std::string, value_t>>>();
  inst->emplace_back("__class__", value_t{cls_dict});
  value_t inst_val{inst};
  for (auto &[k, v] : *cls_dict) {
    if (k == "init") {
      if (!is_callable(v))
        break;
      // An inherited init is followed up the chain here, and with no class defining one there
      // is nothing to run.
      const value_t *method = &v;
      if (const auto *fn = std::get_if<native_fn>(&v.v))
        if (const auto *inherited = fn->target<class_inherited_method>();
            inherited && inherited->name == "init")
          method = inherited->resolve();
      if (!method)
        break;
      std::vector<value_t> a;
      a.push_back(inst_val);
      a.insert(a.end(), init_args.begin(), init_args.end());
      const value_t init = *method; // init may change the class dict under `method`
      // Whatever init returns, the call's value is the instance, once init has run.
      return call_function(init, a, "constructor", [inst_val](value_t) { return inst_val; });
    }
  }
  return inst_val;
}

const value_t *detail::class_inherited_method::resolve() const {
  const class_inherited_method *at = this;
  for (int hop = 1;; ++hop) {
    if (!at->super_cls)
      return nullptr;
    const value_t *found = nullptr;
    for (auto &[k, v] : *at->super_cls)
      if (k == name) {
        found = &v;
        break;
      }
    if (!found || !is_callable(*found))
      return nullptr;
    const auto *fn = std::get_if<native_fn>(&found->v);
    const auto *next = fn ? fn->target<class_inherited_method>() : nullptr;
    if (!next || next->name != name || hop == kInheritedHops)
      return found;
    at = next;
  }
}

value_t detail::class_inherited_method::operator()(std::span<const value_t> args) const {
  if (const value_t *method = resolve()) {
    const value_t inherited = *method; // it may change the class dict under `method`
    return call_function(inherited, args, name == "init" ? "init" : "finalize");
  }
  if (name == "init")
    return args.empty() ? nil_value : args[0];
  return nil_value;
}

} // namespace cvc::state_exec
