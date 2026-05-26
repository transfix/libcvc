#include <chrono>
#include <cvc/core/state_exec/builtins.h>
#include <cvc/core/state_exec/generator.h>
#include <cvc/core/state_exec/parser.h>
#include <cvc/core/state_exec/stackless_evaluator.h>

namespace cvc::state_exec {

// ---------------------------------------------------------------------------
// ctor
// ---------------------------------------------------------------------------

stackless_evaluator::stackless_evaluator(environment_ptr global_env)
    : global_env_(std::move(global_env)) {}

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

evaluator_state stackless_evaluator::create_state(const std::string &script, environment_ptr env) {
  auto exprs = parse_all(script);
  if (exprs.empty()) {
    evaluator_state st;
    st.done = true;
    return st;
  }
  if (exprs.size() == 1)
    return create_state(exprs[0], env);
  std::vector<value_t> begin_exprs;
  begin_exprs.reserve(1 + exprs.size());
  begin_exprs.push_back(value_t{symbol{"begin"}});
  begin_exprs.insert(begin_exprs.end(), exprs.begin(), exprs.end());
  return create_state(make_list(std::move(begin_exprs)), env);
}

bool stackless_evaluator::step(evaluator_state &state) {
  check_interrupted();
  if (state.done || state.stack.empty()) {
    state.done = true;
    return true;
  }
  state.stats.increment_step();
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
  }
  return state.done;
}

value_t stackless_evaluator::run(evaluator_state &state, std::optional<uint64_t> max_steps,
                                 std::optional<double> timeout_sec,
                                 std::function<void(const value_t &)> on_complete) {
  std::lock_guard lk(eval_mu_);
  auto start = timeout_sec ? std::optional{std::chrono::steady_clock::now()} : std::nullopt;

  state.stats.start();
  try {
    while (!state.done) {
      step(state);
      if (max_steps && state.stats.get_step_count() >= *max_steps)
        break;
      if (start) {
        auto elapsed = std::chrono::steady_clock::now() - *start;
        if (std::chrono::duration<double>(elapsed).count() > *timeout_sec) {
          state.stats.mark_complete();
          throw evaluation_timeout("evaluation exceeded timeout");
        }
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
  state.stack.pop_back();
  if (!state.stack.empty()) {
    state.stack.back().results.push_back(std::move(result));
  } else {
    state.result = std::move(result);
    state.done = true;
  }
}

void stackless_evaluator::push_frame(evaluator_state &state, value_t expr, environment_ptr env) {
  state.stack.push_back({.expr = std::move(expr), .env = std::move(env)});
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
      pop_frame(state, nil_value);
      return;
    }
    if (sym->name == "t") {
      pop_frame(state, true_value);
      return;
    }
    auto *val = env->lookup(sym->name);
    if (!val)
      throw std::runtime_error("undefined symbol: " + sym->name);
    pop_frame(state, *val);
    return;
  }

  // Non-list atom
  if (!std::holds_alternative<list_ptr>(expr.v)) {
    pop_frame(state, expr);
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
    throw std::runtime_error("invalid expression head: " + to_string(elems[0]));

  const auto &name = head->name;
  std::vector<value_t> args(elems.begin() + 1, elems.end());

  // --- Special forms ---
  if (name == "if") {
    if (args.size() < 2)
      throw std::runtime_error("if: expected at least 2 args");
    frame.phase = eval_phase::if_cond;
    frame.extra_vals = std::move(args);
    push_frame(state, frame.extra_vals[0], env);
    return;
  }
  if (name == "begin") {
    if (args.empty()) {
      pop_frame(state, nil_value);
      return;
    }
    frame.phase = eval_phase::begin_next;
    frame.extra_vals = std::move(args);
    frame.index = 0;
    push_frame(state, frame.extra_vals[0], env);
    return;
  }
  if (name == "while") {
    if (args.size() < 2)
      throw std::runtime_error("while: expected 2 args");
    frame.phase = eval_phase::while_cond;
    frame.extra_vals = std::move(args); // [0]=cond, [1]=body
    push_frame(state, frame.extra_vals[0], env);
    return;
  }
  if (name == "for") {
    if (args.size() < 3)
      throw std::runtime_error("for: expected 3 args");
    auto *var_sym = std::get_if<symbol>(&args[0].v);
    if (!var_sym)
      throw std::runtime_error("for: first arg must be symbol");
    frame.phase = eval_phase::for_body;
    frame.extra_strs = {var_sym->name};
    frame.extra_vals = {args[2]};    // body expression
    frame.extra_lists = {};          // will hold iterator
    frame.index = -1;                // -1 = still evaluating collection
    push_frame(state, args[1], env); // evaluate collection
    return;
  }
  if (name == "set") {
    if (args.size() != 2)
      throw std::runtime_error("set: expected 2 args");
    auto *var_sym = std::get_if<symbol>(&args[0].v);
    if (!var_sym)
      throw std::runtime_error("set: first arg must be symbol");
    frame.phase = eval_phase::set_value;
    frame.extra_strs = {var_sym->name};
    push_frame(state, args[1], env);
    return;
  }
  if (name == "quote") {
    if (args.size() != 1)
      throw std::runtime_error("quote: expected 1 arg");
    pop_frame(state, args[0]);
    return;
  }
  if (name == "lambda") {
    if (args.size() < 2)
      throw std::runtime_error("lambda: expected at least 2 args");
    std::vector<symbol> params;
    bool variadic = false;
    if (auto *pl = std::get_if<list_ptr>(&args[0].v); pl && *pl) {
      for (auto &p : **pl) {
        auto *ps = std::get_if<symbol>(&p.v);
        if (!ps)
          throw std::runtime_error("lambda: param must be symbol");
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
    cls->env_snapshot = env;
    pop_frame(state, value_t{std::move(cls)});
    return;
  }
  if (name == "return") {
    if (args.empty())
      throw std::runtime_error("return: expected 1 arg");
    frame.phase = eval_phase::return_value;
    push_frame(state, args[0], env);
    return;
  }
  if (name == "yield") {
    if (args.empty())
      throw std::runtime_error("yield: expected 1 arg");
    frame.phase = eval_phase::yield_value;
    push_frame(state, args[0], env);
    return;
  }
  if (name == "break") {
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
  if (name == "let") {
    if (args.size() < 2)
      throw std::runtime_error("let: expected at least 2 args");
    auto *bindings_lp = std::get_if<list_ptr>(&args[0].v);
    if (!bindings_lp)
      throw std::runtime_error("let: first arg must be binding list");
    auto local = environment::extend(env);
    frame.phase = eval_phase::let_bindings;
    frame.extra_vals = std::vector<value_t>(args.begin(), args.end()); // [0]=bindings, [1..]=body
    frame.extra_env = local;
    frame.index = 0;
    if (!(**bindings_lp).empty()) {
      auto &first_binding = (**bindings_lp)[0];
      auto *bp = std::get_if<list_ptr>(&first_binding.v);
      if (!bp || (*bp)->size() != 2)
        throw std::runtime_error("let: each binding must be (name value)");
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
  if (name == "defun") {
    if (args.size() < 3)
      throw std::runtime_error("defun: expected at least 3 args");
    auto *name_sym = std::get_if<symbol>(&args[0].v);
    if (!name_sym)
      throw std::runtime_error("defun: first arg must be symbol");
    // Build closure directly
    std::vector<symbol> params;
    bool variadic = false;
    if (auto *pl = std::get_if<list_ptr>(&args[1].v); pl && *pl) {
      for (auto &p : **pl) {
        auto *ps = std::get_if<symbol>(&p.v);
        if (!ps)
          throw std::runtime_error("defun: param must be symbol");
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
    cls->env_snapshot = env;
    env->set(name_sym->name, value_t{cls});
    pop_frame(state, value_t{std::move(cls)});
    return;
  }
  if (name == "defmacro") {
    if (args.size() < 3)
      throw std::runtime_error("defmacro: expected at least 3 args");
    auto *name_sym = std::get_if<symbol>(&args[0].v);
    if (!name_sym)
      throw std::runtime_error("defmacro: first arg must be symbol");
    std::vector<symbol> params;
    if (auto *pl = std::get_if<list_ptr>(&args[1].v); pl && *pl) {
      for (auto &p : **pl) {
        auto *ps = std::get_if<symbol>(&p.v);
        if (!ps)
          throw std::runtime_error("defmacro: param must be symbol");
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
  if (name == "eval") {
    if (args.size() != 1)
      throw std::runtime_error("eval: expected 1 arg");
    frame.phase = eval_phase::eval_inner;
    push_frame(state, args[0], env);
    return;
  }
  if (name == "root") {
    pop_frame(state, state.root_expr);
    return;
  }
  if (name == "super") {
    if (args.size() < 2)
      throw std::runtime_error("super: expected at least 2 args");
    auto *method_sym = std::get_if<symbol>(&args[1].v);
    frame.phase = eval_phase::super_args;
    frame.extra_strs = {method_sym ? method_sym->name : ""};
    frame.extra_vals = std::vector<value_t>(args.begin() + 2, args.end());
    frame.index = 0;
    push_frame(state, args[0], env); // evaluate self
    return;
  }
  if (name == "defclass") {
    handle_defclass(state, frame, args);
    return;
  }

  // User macros — expand then re-evaluate
  if (auto it = state.user_macros.find(name); it != state.user_macros.end()) {
    auto &[macro_params, body] = it->second;
    std::unordered_map<std::string, value_t> subst;
    for (size_t i = 0; i < macro_params.size(); ++i)
      subst[macro_params[i].name] = (i < args.size()) ? args[i] : nil_value;
    // Re-init current frame with expanded form for evaluation
    frame.expr = substitute(body, subst);
    frame.phase = eval_phase::init;
    frame.index = 0;
    frame.results.clear();
    frame.extra_vals.clear();
    frame.extra_strs.clear();
    frame.extra_lists.clear();
    frame.extra_env = nullptr;
    return;
  }

  // Built-in or user function
  auto *func_val = env->lookup(name);
  if (!func_val)
    throw std::runtime_error("unknown function or special form: " + name);

  frame.phase = eval_phase::eval_args;
  frame.extra_vals = std::move(args);
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
    throw std::runtime_error("apply: no function head");

  auto *func_val = frame.env->lookup(head_sym->name);
  if (!func_val)
    throw std::runtime_error("apply: undefined function: " + head_sym->name);

  if (auto *fn = std::get_if<native_fn>(&func_val->v)) {
    pop_frame(state, (*fn)(evaled));
    return;
  }
  if (auto *cp = std::get_if<closure_ptr>(&func_val->v)) {
    call_closure(state, frame, *cp, evaled);
    return;
  }
  throw std::runtime_error("apply: not callable: " + head_sym->name);
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
  pop_frame(state, frame.results.back());
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
    pop_frame(state, frame.results.back());
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

void stackless_evaluator::step_for_body(evaluator_state &state, eval_frame &frame) {
  if (frame.index == -1) {
    // Collection was just evaluated
    auto collection = frame.results.back();
    if (auto *gp = std::get_if<generator_ptr>(&collection.v)) {
      // Generator: store in extra_vals[1] for lazy iteration
      frame.extra_vals.push_back(collection);
      frame.extra_lists = {{}}; // placeholder
      auto next = generator_next(**gp);
      if (!next) {
        pop_frame(state, nil_value);
        return;
      }
      frame.index = 0;
      frame.results.clear();
      frame.env->set(frame.extra_strs[0], *next);
      push_frame(state, frame.extra_vals[0], frame.env);
      return;
    }
    auto *lp = std::get_if<list_ptr>(&collection.v);
    if (!lp)
      throw std::runtime_error("for: collection must be a list or generator");
    frame.extra_lists = {std::vector<value_t>((*lp)->begin(), (*lp)->end())};
    frame.index = 0;
    frame.results.clear();
  } else {
    frame.index++;
    frame.results.clear();
  }

  // Generator path: extra_vals has size > 1 when a generator is stored
  if (frame.extra_vals.size() > 1) {
    auto *gp = std::get_if<generator_ptr>(&frame.extra_vals[1].v);
    if (gp) {
      auto next = generator_next(**gp);
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

// ---------------------------------------------------------------------------
// step: set
// ---------------------------------------------------------------------------

void stackless_evaluator::step_set_value(evaluator_state &state, eval_frame &frame) {
  auto val = frame.results.back();
  frame.env->set(frame.extra_strs[0], val);
  pop_frame(state, val);
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
    pop_frame(state, frame.results.back());
  }
}

// ---------------------------------------------------------------------------
// step: return
// ---------------------------------------------------------------------------

void stackless_evaluator::step_return_value(evaluator_state &state, eval_frame &frame) {
  auto val = frame.results.back();
  state.stack.pop_back();
  // Unwind to function boundary
  while (!state.stack.empty()) {
    auto &top = state.stack.back();
    if (top.phase == eval_phase::apply) {
      state.stack.pop_back();
      break;
    }
    state.stack.pop_back();
  }
  if (!state.stack.empty()) {
    state.stack.back().results.push_back(val);
  } else {
    state.result = val;
    state.done = true;
  }
}

// ---------------------------------------------------------------------------
// step: yield (generator support)
// ---------------------------------------------------------------------------

void stackless_evaluator::step_yield_value(evaluator_state &state, eval_frame &frame) {
  auto val = frame.results.back();
  state.result = val;   // Store value where generator_next can find it
  pop_frame(state, val);
  state.yielded = true; // Signal generator_next to stop stepping
}

// ---------------------------------------------------------------------------
// step: break (loop exit)
// ---------------------------------------------------------------------------

void stackless_evaluator::step_break_unwind(evaluator_state &state, eval_frame &frame) {
  auto val = frame.results.back();
  state.stack.pop_back(); // Remove the break frame
  // Unwind to loop boundary
  while (!state.stack.empty()) {
    auto &top = state.stack.back();
    if (top.phase == eval_phase::while_cond || top.phase == eval_phase::while_body ||
        top.phase == eval_phase::for_body) {
      state.stack.pop_back(); // Remove the loop frame
      break;
    }
    state.stack.pop_back();
  }
  if (!state.stack.empty()) {
    state.stack.back().results.push_back(val);
  } else {
    state.result = val;
    state.done = true;
  }
}

// ---------------------------------------------------------------------------
// step: eval
// ---------------------------------------------------------------------------

void stackless_evaluator::step_eval_inner(evaluator_state &state, eval_frame &frame) {
  auto code = frame.results.back();
  state.stack.pop_back();
  push_frame(state, code, frame.env);
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
      throw std::runtime_error("super: no __class__");
    auto &cls_dict = *std::get<dict_ptr>(cls_val->v);
    value_t *super_val = nullptr;
    for (auto &[k, v] : cls_dict)
      if (k == "__super__") {
        super_val = &v;
        break;
      }
    if (!super_val)
      throw std::runtime_error("super: no __super__");

    auto *search = &(*std::get<dict_ptr>(super_val->v));
    while (search) {
      for (auto &[k, v] : *search) {
        if (k == method_name) {
          if (auto *fn = std::get_if<native_fn>(&v.v)) {
            std::vector<value_t> call_args;
            call_args.push_back(self_val);
            for (size_t i = 1; i < frame.results.size(); ++i)
              call_args.push_back(frame.results[i]);
            pop_frame(state, (*fn)(call_args));
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
    throw std::runtime_error("super: method '" + method_name + "' not found");
  }
}

// ---------------------------------------------------------------------------
// closure call
// ---------------------------------------------------------------------------

void stackless_evaluator::call_closure(evaluator_state &state, eval_frame & /*frame*/,
                                       const closure_ptr &cls, const std::vector<value_t> &args) {
  auto local = environment::extend(cls->env_snapshot);
  if (cls->variadic) {
    if (cls->params.empty())
      throw std::runtime_error("closure: variadic but no param name");
    local->set(cls->params.back().name, make_list(std::vector<value_t>(args.begin(), args.end())));
  } else {
    if (args.size() != cls->params.size())
      throw std::runtime_error("closure: expected " + std::to_string(cls->params.size()) +
                               " args, got " + std::to_string(args.size()));
    for (size_t i = 0; i < cls->params.size(); ++i)
      local->set(cls->params[i].name, args[i]);
  }

  // Replace current frame with body evaluation
  state.stack.pop_back();
  if (cls->body.size() == 1) {
    push_frame(state, cls->body[0], local);
  } else {
    std::vector<value_t> begin_exprs;
    begin_exprs.reserve(1 + cls->body.size());
    begin_exprs.push_back(value_t{symbol{"begin"}});
    begin_exprs.insert(begin_exprs.end(), cls->body.begin(), cls->body.end());
    push_frame(state, make_list(std::move(begin_exprs)), local);
  }
}

// ---------------------------------------------------------------------------
// defclass
// ---------------------------------------------------------------------------

void stackless_evaluator::handle_defclass(evaluator_state &state, eval_frame &frame,
                                          const std::vector<value_t> &args) {
  if (args.empty())
    throw std::runtime_error("defclass: expected at least a name");
  auto *name_sym = std::get_if<symbol>(&args[0].v);
  if (!name_sym)
    throw std::runtime_error("defclass: first arg must be symbol");
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

  // Default init
  if (!has_method("init")) {
    auto sc = super_cls_dict;
    native_fn default_init = [sc](std::span<const value_t> a) -> value_t {
      if (sc) {
        for (auto &[k, v] : *sc) {
          if (k == "init") {
            if (auto *fn = std::get_if<native_fn>(&v.v))
              return (*fn)(a);
            break;
          }
        }
      }
      return a.empty() ? nil_value : a[0];
    };
    cls_dict->emplace_back("init", value_t{std::move(default_init)});
  }

  // Default finalize
  if (!has_method("finalize")) {
    auto sc = super_cls_dict;
    native_fn default_finalize = [sc](std::span<const value_t> a) -> value_t {
      if (sc) {
        for (auto &[k, v] : *sc) {
          if (k == "finalize") {
            if (auto *fn = std::get_if<native_fn>(&v.v))
              return (*fn)(a);
            break;
          }
        }
      }
      return nil_value;
    };
    cls_dict->emplace_back("finalize", value_t{std::move(default_finalize)});
  }

  // Process methods
  stackless_evaluator *self = this;
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
          throw std::runtime_error("defclass: param must be symbol");
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
    mcls->env_snapshot = env;

    auto genv = global_env_;
    native_fn wrapper = [self, mcls, genv](std::span<const value_t> call_args) -> value_t {
      // Create a temp state and run to completion
      std::vector<value_t> all_args(call_args.begin(), call_args.end());
      auto local = environment::extend(mcls->env_snapshot);
      if (mcls->variadic) {
        local->set(mcls->params.back().name, make_list(all_args));
      } else {
        for (size_t j = 0; j < mcls->params.size(); ++j)
          local->set(mcls->params[j].name, all_args[j]);
      }
      if (mcls->body.size() == 1) {
        // Need a recursive evaluation — use a temporary evaluator
        stackless_evaluator temp(genv);
        return temp.evaluate(mcls->body[0], local);
      }
      std::vector<value_t> be;
      be.reserve(1 + mcls->body.size());
      be.push_back(value_t{symbol{"begin"}});
      be.insert(be.end(), mcls->body.begin(), mcls->body.end());
      stackless_evaluator temp(genv);
      return temp.evaluate(make_list(std::move(be)), local);
    };
    cls_dict->emplace_back(msym->name, value_t{std::move(wrapper)});
  }

  // Constructor
  auto cls_dict_val = value_t{cls_dict};
  native_fn constructor = [cls_dict, cls_dict_val](std::span<const value_t> init_args) -> value_t {
    auto inst = std::make_shared<std::vector<std::pair<std::string, value_t>>>();
    inst->emplace_back("__class__", cls_dict_val);
    value_t inst_val{inst};
    for (auto &[k, v] : *cls_dict) {
      if (k == "init") {
        std::vector<value_t> a;
        a.push_back(inst_val);
        a.insert(a.end(), init_args.begin(), init_args.end());
        if (auto *fn = std::get_if<native_fn>(&v.v))
          (*fn)(a);
        break;
      }
    }
    return inst_val;
  };

  env->set(cls_name, value_t{std::move(constructor)});
  env->set(cls_name + ".__cls_dict__", cls_dict_val);
  pop_frame(state, *env->lookup(cls_name));
}

// ---------------------------------------------------------------------------
// macro substitution
// ---------------------------------------------------------------------------

value_t stackless_evaluator::substitute(const value_t &tmpl,
                                        const std::unordered_map<std::string, value_t> &subst) {
  if (auto *sym = std::get_if<symbol>(&tmpl.v)) {
    auto it = subst.find(sym->name);
    if (it != subst.end())
      return it->second;
    return tmpl;
  }
  if (auto *lp = std::get_if<list_ptr>(&tmpl.v)) {
    if (!*lp)
      return tmpl;
    auto result = std::make_shared<std::vector<value_t>>();
    result->reserve((*lp)->size());
    for (auto &e : **lp)
      result->push_back(substitute(e, subst));
    return value_t{std::move(result)};
  }
  return tmpl;
}

} // namespace cvc::state_exec
