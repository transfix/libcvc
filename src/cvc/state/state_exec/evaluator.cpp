#include "closure_natives.h" // detail::class_constructor
#include "special_forms.h"

#include <atomic>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/data_job.h>
#include <cvc/state/state_exec/evaluator.h>
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/parser.h>
#include <thread>

namespace cvc::state_exec {

// ---------------------------------------------------------------------------
// construction
// ---------------------------------------------------------------------------

evaluator::evaluator(environment_ptr global_env) : global_env_(std::move(global_env)) {}

// ---------------------------------------------------------------------------
// public API
// ---------------------------------------------------------------------------

value_t evaluator::evaluate(const value_t &expr, environment_ptr env,
                            std::optional<double> timeout_sec,
                            std::function<void(const value_t &)> on_complete) {
  detail::check_not_in_async_kernel("evaluator::evaluate");
  std::lock_guard lk(eval_mu_);
  root_expr_ = expr;
  stats_.start();

  const environment_ptr run_env = env ? env : environment::extend(global_env_);

  try {
    value_t result;
    if (timeout_sec) {
      // Run in a background thread with timeout
      value_t bg_result;
      std::exception_ptr ex;
      std::atomic<bool> done{false};
      std::thread worker([&] {
        // A loop inside one native call (a generator body, a data job) never sees interrupt():
        // the thread's evaluation deadline cuts it, so join() below does not wait on it.
        eval_deadline_guard deadline_guard(timeout_sec);
        try {
          bg_result = eval_internal(expr, run_env);
        } catch (...) {
          ex = std::current_exception();
        }
        done.store(true, std::memory_order_release);
      });
      auto deadline =
          std::chrono::steady_clock::now() + std::chrono::duration<double>(*timeout_sec);
      while (!done.load(std::memory_order_acquire)) {
        if (std::chrono::steady_clock::now() >= deadline) {
          interrupt();
          worker.join();
          reset_interrupt();
          stats_.mark_complete();
          throw evaluation_timeout("evaluation exceeded timeout");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      worker.join();
      if (ex) {
        stats_.mark_complete();
        std::rethrow_exception(ex);
      }
      result = std::move(bg_result);
    } else {
      result = eval_internal(expr, run_env);
    }
    stats_.mark_complete();
    if (on_complete)
      on_complete(result);
    return result;
  } catch (const return_exception &) {
    stats_.mark_complete();
    throw;
  } catch (const std::exception &e) {
    // Nothing caught it: it reaches the caller as a dsl_exception, with where it was raised and,
    // for one made from a C++ exception, that exception as its original().
    stats_.mark_complete();
    if (is_uncatchable(e))
      throw;
    std::optional<value_t> exc = caught(e);
    std::exception_ptr original;
    if (const auto *d = dynamic_cast<const dsl_exception *>(&e))
      original = d->original();
    if (!original && converted_original_ && same_exception(*exc, converted_))
      original = converted_original_;
    throw dsl_exception(std::move(*exc), original);
  } catch (...) {
    stats_.mark_complete();
    throw;
  }
}

value_t evaluator::evaluate_script(const std::string &script, environment_ptr env,
                                   std::optional<double> timeout_sec,
                                   std::function<void(const value_t &)> on_complete) {
  auto exprs = parse_all(script);
  if (exprs.empty())
    return nil_value;
  if (exprs.size() == 1)
    return evaluate(exprs[0], env, timeout_sec, std::move(on_complete));
  std::vector<value_t> begin_exprs;
  begin_exprs.reserve(1 + exprs.size());
  begin_exprs.push_back(value_t{symbol{"begin"}});
  begin_exprs.insert(begin_exprs.end(), exprs.begin(), exprs.end());
  return evaluate(make_list(std::move(begin_exprs)), env, timeout_sec, std::move(on_complete));
}

// ---------------------------------------------------------------------------
// interrupt / pause
// ---------------------------------------------------------------------------

void evaluator::interrupt() { interrupted_.store(true, std::memory_order_release); }
void evaluator::reset_interrupt() { interrupted_.store(false, std::memory_order_release); }

void evaluator::pause() { paused_.store(true, std::memory_order_release); }

void evaluator::resume() {
  paused_.store(false, std::memory_order_release);
  pause_cv_.notify_all();
}

bool evaluator::is_paused() const { return paused_.load(std::memory_order_acquire); }

void evaluator::check_interrupted() {
  if (interrupted_.load(std::memory_order_acquire))
    throw evaluation_interrupted("evaluation was interrupted");
  if (paused_.load(std::memory_order_acquire)) {
    std::unique_lock lk(pause_mu_);
    pause_cv_.wait(lk, [this] { return !paused_.load(std::memory_order_acquire); });
  }
}

// ---------------------------------------------------------------------------
// core eval
// ---------------------------------------------------------------------------

namespace {
// An empty vector of values with room for `n`, from `spares` (the vectors the evaluator keeps
// between forms), given back to them emptied when it goes, so that a form's argument vectors
// aren't allocated each time. A few small ones are kept.
struct spare_values {
  spare_values(std::vector<std::vector<value_t>> &spares, std::size_t n) : spares(spares) {
    if (!spares.empty()) {
      values = std::move(spares.back());
      spares.pop_back();
    }
    values.reserve(n);
  }
  ~spare_values() {
    values.clear();
    if (values.capacity() <= 16 && spares.size() < 64) {
      try {
        spares.push_back(std::move(values));
      } catch (...) { // out of memory: just free it
      }
    }
  }
  spare_values(const spare_values &) = delete;
  spare_values &operator=(const spare_values &) = delete;

  std::vector<std::vector<value_t>> &spares;
  std::vector<value_t> values;
};
} // namespace

value_t evaluator::eval_internal(const value_t &expr, const environment_ptr &env) {
  ++generation_;
  trail_.push_back(&expr);
  // Pops the trail on the way out. The first one an exception unwinds (eval_form didn't return)
  // records where it was raised, while the trail still has it.
  struct trail_guard {
    evaluator &ev;
    bool returned = false;
    ~trail_guard() {
      if (!returned && ev.raise_generation_ != ev.generation_)
        ev.record_raise_point();
      ev.trail_.pop_back();
    }
  } guard{*this};
  value_t result = eval_form(expr, env);
  guard.returned = true; // (returning `result` moves it, which can't throw)
  return result;
}

void evaluator::record_raise_point() noexcept {
  try {
    const std::size_t n = trail_.size();
    raise_trail_.clear();
    for (std::size_t i = n > kTracebackFrames ? n - kTracebackFrames : 0; i < n; ++i)
      raise_trail_.push_back(*trail_[i]);
    raise_handled_ = handling_.empty() ? nil_value : handling_.back().first;
    raise_generation_ = generation_;
  } catch (...) { // out of memory: the traceback is what is left of it
  }
}

std::optional<value_t> evaluator::caught(const std::exception &e) {
  if (is_uncatchable(e))
    return std::nullopt;
  value_t exc;
  if (const auto *d = dynamic_cast<const dsl_exception *>(&e)) {
    exc = d->value();
  } else {
    exc = exception_from(e);
    converted_ = exc;
    converted_original_ = std::current_exception();
  }
  annotate(exc);
  return exc;
}

void evaluator::annotate(value_t &exc) {
  if (const value_t *tb = field(exc, "traceback"); !tb || tb->is_nil()) {
    std::vector<value_t> lines;
    lines.reserve(raise_trail_.size());
    for (const value_t &expr : raise_trail_)
      lines.emplace_back(traceback_line(expr));
    set_field(exc, "traceback", make_list(std::move(lines)));
  }
  if (const value_t *cause = field(exc, "cause"); !cause || cause->is_nil())
    if (!raise_handled_.is_nil() && !in_cause_chain(raise_handled_, exc))
      set_field(exc, "cause", raise_handled_);
  const value_t *origin = field(exc, "origin");
  if (!origin) {
    set_field(exc, "origin", value_t(std::string("dsl")));
    return;
  }
  const auto *from = std::get_if<std::string>(&origin->v);
  if (!from || *from == "dsl" || field(exc, "function") || raise_trail_.empty())
    return;
  const auto *lp = std::get_if<list_ptr>(&raise_trail_.back().v);
  if (!lp || !*lp || (*lp)->empty())
    return;
  if (const auto *head = std::get_if<symbol>(&(**lp)[0].v);
      head && detail::special_form_of(head->name) == detail::special_form::none)
    set_field(exc, "function", value_t(head->name));
}

value_t evaluator::eval_form(const value_t &expr, const environment_ptr &env) {
  check_interrupted();
  stats_.increment_step();

  // Symbol
  if (auto *sym = std::get_if<symbol>(&expr.v)) {
    if (sym->name == "nil")
      return nil_value;
    if (sym->name == "t")
      return true_value;
    if (auto *val = env->lookup(sym->name))
      return *val;
    throw dsl_error("NameError", "undefined symbol: " + sym->name);
  }

  // Non-list atom (number, string, bool, etc.)
  if (!std::holds_alternative<list_ptr>(expr.v))
    return expr;

  auto *lp = std::get_if<list_ptr>(&expr.v);
  if (!lp || !*lp)
    return expr;
  auto &elems = **lp;
  if (elems.empty())
    return make_list(); // empty list literal

  // Head must be a symbol for function/special-form dispatch
  auto *head = std::get_if<symbol>(&elems[0].v);
  if (!head)
    throw dsl_error("TypeError", "invalid expression head: " + to_string(elems[0]));

  const auto &name = head->name;
  // A copy of the arguments (the program may change its own code while they run), in a vector
  // this evaluator keeps between forms.
  spare_values args_copy(spare_vectors_, elems.size() - 1);
  std::vector<value_t> &args = args_copy.values;
  args.assign(elems.begin() + 1, elems.end());

  // --- Special forms --- (all of the stackless evaluator's but yield and break)
  switch (detail::special_form_of(name)) {
  case detail::special_form::if_:
    return do_if(args, env);
  case detail::special_form::begin:
    return do_begin(args, env);
  case detail::special_form::while_:
    return do_while(args, env);
  case detail::special_form::for_:
    return do_for(args, env);
  case detail::special_form::set:
    return do_set(args, env);
  case detail::special_form::quote:
    return do_quote(args);
  case detail::special_form::lambda:
    return do_lambda(args, env);
  case detail::special_form::return_:
    return do_return(args, env);
  case detail::special_form::let:
    return do_let(args, env);
  case detail::special_form::super:
    return do_super(args, env);
  case detail::special_form::defun:
    return do_defun(args, env);
  case detail::special_form::defclass:
    return do_defclass(args, env);
  case detail::special_form::defmacro:
    return do_defmacro(args, env);
  case detail::special_form::eval:
    return do_eval(args, env);
  case detail::special_form::root:
    return do_root();
  case detail::special_form::try_:
    return do_try(args, env);
  case detail::special_form::raise:
    return do_raise(args, env);
  case detail::special_form::none:
  case detail::special_form::yield:
  case detail::special_form::break_:
    break;
  }

  // --- User macros ---
  if (auto it = user_macros_.empty() ? user_macros_.end() : user_macros_.find(name);
      it != user_macros_.end()) {
    auto &[params, body] = it->second;
    std::unordered_map<std::string, value_t> subst;
    for (size_t i = 0; i < params.size(); ++i)
      subst[params[i].name] = (i < args.size()) ? args[i] : nil_value;
    return eval_internal(substitute(body, subst), env);
  }

  // --- Function call (built-in or user-defined) ---
  auto *func_val = env->lookup(name);
  if (!func_val)
    throw dsl_error("NameError", "unknown function: " + name);

  // Evaluate arguments
  spare_values evaled_values(spare_vectors_, args.size());
  std::vector<value_t> &evaled = evaled_values.values;
  for (auto &a : args)
    evaled.push_back(eval_internal(a, env));

  // native_fn
  if (auto *fn = std::get_if<native_fn>(&func_val->v))
    return (*fn)(evaled);

  // closure
  if (auto *cp = std::get_if<closure_ptr>(&func_val->v))
    return apply_closure(*cp, evaled);

  throw dsl_error("TypeError", "not callable: " + name);
}

// ---------------------------------------------------------------------------
// special forms
// ---------------------------------------------------------------------------

value_t evaluator::do_if(const std::vector<value_t> &args, const environment_ptr &env) {
  check_interrupted();
  if (args.size() < 2 || args.size() > 3)
    throw dsl_error("TypeError", "if: expected a condition, a then branch and an optional else "
                                 "branch, got " +
                                     std::to_string(args.size()) + " arguments");
  if (eval_internal(args[0], env).is_truthy())
    return eval_internal(args[1], env);
  if (args.size() > 2)
    return eval_internal(args[2], env);
  return nil_value;
}

value_t evaluator::do_begin(std::span<const value_t> args, const environment_ptr &env) {
  if (args.empty())
    return nil_value;
  value_t result = nil_value;
  for (auto &a : args) {
    check_interrupted();
    result = eval_internal(a, env);
  }
  return result;
}

value_t evaluator::do_while(const std::vector<value_t> &args, const environment_ptr &env) {
  if (args.size() < 2)
    throw dsl_error("TypeError", "while: expected a condition and a body");
  while (eval_internal(args[0], env).is_truthy()) {
    check_interrupted();
    for (std::size_t i = 1; i < args.size(); ++i) // several body forms run in order
      eval_internal(args[i], env);
  }
  return nil_value;
}

value_t evaluator::do_for(const std::vector<value_t> &args, const environment_ptr &env) {
  if (args.size() < 3)
    throw dsl_error("TypeError", "for: expected a variable, a collection and a body");
  auto *var_sym = std::get_if<symbol>(&args[0].v);
  if (!var_sym)
    throw dsl_error("TypeError", "for: first arg must be symbol");
  auto collection = eval_internal(args[1], env);
  auto *lp = std::get_if<list_ptr>(&collection.v);
  if (!lp)
    throw dsl_error("TypeError", "for: second arg must evaluate to list");
  for (auto &item : **lp) {
    check_interrupted();
    env->set(var_sym->name, item);
    for (std::size_t i = 2; i < args.size(); ++i) // several body forms run in order
      eval_internal(args[i], env);
  }
  return nil_value;
}

value_t evaluator::do_set(const std::vector<value_t> &args, const environment_ptr &env) {
  if (args.size() != 2)
    throw dsl_error("TypeError", "set: expected 2 arguments");
  auto *sym = std::get_if<symbol>(&args[0].v);
  if (!sym)
    throw dsl_error("TypeError", "set: first arg must be symbol");
  auto val = eval_internal(args[1], env);
  env->set(sym->name, val);
  return val;
}

value_t evaluator::do_quote(const std::vector<value_t> &args) {
  if (args.size() != 1)
    throw dsl_error("TypeError", "quote: expected 1 argument");
  return args[0];
}

value_t evaluator::do_lambda(const std::vector<value_t> &args, const environment_ptr &env) {
  if (args.size() < 2)
    throw dsl_error("TypeError", "lambda: expected at least 2 arguments");
  // Parse parameter list
  std::vector<symbol> params;
  bool variadic = false;
  auto *params_val = std::get_if<list_ptr>(&args[0].v);
  if (params_val && *params_val) {
    for (auto &p : **params_val) {
      auto *ps = std::get_if<symbol>(&p.v);
      if (!ps)
        throw dsl_error("TypeError", "lambda: param must be symbol");
      if (ps->name == "&rest") {
        variadic = true;
        continue;
      }
      params.push_back(*ps);
    }
  } else if (auto *nil_sym = std::get_if<symbol>(&args[0].v); nil_sym && nil_sym->name == "nil") {
    // empty params
  } else if (!std::holds_alternative<list_ptr>(args[0].v)) {
    // single-symbol shorthand - not standard but handle gracefully
    throw dsl_error("TypeError", "lambda: params must be a list");
  }

  std::vector<value_t> body(args.begin() + 1, args.end());
  auto cls = std::make_shared<closure>();
  cls->params = std::move(params);
  cls->variadic = variadic;
  cls->body = std::move(body);
  cls->env_snapshot = env;
  return value_t{std::move(cls)};
}

value_t evaluator::do_return(const std::vector<value_t> &args, const environment_ptr &env) {
  if (args.size() > 1)
    throw dsl_error("TypeError",
                    "return: expected at most 1 argument, got " + std::to_string(args.size()));
  if (args.empty())
    throw return_exception(nil_value);
  throw return_exception(eval_internal(args[0], env));
}

value_t evaluator::do_let(const std::vector<value_t> &args, const environment_ptr &env) {
  if (args.size() < 2)
    throw dsl_error("TypeError", "let: expected at least 2 arguments");
  auto local = environment::extend(env);
  // First arg: list of (name value) pairs
  auto *bindings = std::get_if<list_ptr>(&args[0].v);
  if (!bindings)
    throw dsl_error("TypeError", "let: first arg must be binding list");
  for (auto &b : **bindings) {
    auto *bp = std::get_if<list_ptr>(&b.v);
    if (!bp || (*bp)->size() != 2)
      throw dsl_error("TypeError", "let: each binding must be (name value)");
    auto *sym = std::get_if<symbol>(&(**bp)[0].v);
    if (!sym)
      throw dsl_error("TypeError", "let: binding name must be symbol");
    local->set(sym->name, eval_internal((**bp)[1], local));
  }
  // Evaluate body expressions (the arguments are this form's own copy)
  return do_begin(std::span<const value_t>(args).subspan(1), local);
}

value_t evaluator::do_super(const std::vector<value_t> &args, const environment_ptr &env) {
  if (args.size() < 2)
    throw dsl_error("TypeError", "super: expected at least 2 arguments");
  auto self_val = eval_internal(args[0], env);
  auto *method_sym = std::get_if<symbol>(&args[1].v);
  std::string method_name;
  if (method_sym)
    method_name = method_sym->name;
  else
    method_name = std::get<std::string>(eval_internal(args[1], env).v);

  std::vector<value_t> call_args;
  call_args.reserve(args.size());
  // send obj method args... with super=true
  call_args.push_back(self_val);
  call_args.push_back(value_t{method_name});
  for (size_t i = 2; i < args.size(); ++i)
    call_args.push_back(eval_internal(args[i], env));

  // Walk __class__.__super__ chain
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
  auto *super_dict = &(*std::get<dict_ptr>(super_val->v));

  // Walk chain looking for method
  while (super_dict) {
    for (auto &[k, v] : *super_dict) {
      if (k == method_name) {
        if (auto *fn = std::get_if<native_fn>(&v.v)) {
          // first arg is self
          std::vector<value_t> fn_args;
          fn_args.push_back(self_val);
          for (size_t i = 2; i < args.size(); ++i)
            fn_args.push_back(eval_internal(args[i], env));
          return (*fn)(fn_args);
        }
        if (auto *cp = std::get_if<closure_ptr>(&v.v)) {
          std::vector<value_t> fn_args;
          fn_args.push_back(self_val);
          for (size_t i = 2; i < args.size(); ++i)
            fn_args.push_back(eval_internal(args[i], env));
          return apply_closure(*cp, fn_args);
        }
        throw dsl_error("TypeError", "super: method not callable");
      }
    }
    // Walk __super__
    value_t *sp = nullptr;
    for (auto &[k, v] : *super_dict)
      if (k == "__super__") {
        sp = &v;
        break;
      }
    if (!sp)
      break;
    super_dict = &(*std::get<dict_ptr>(sp->v));
  }
  throw dsl_error("KeyError", "super: method '" + method_name + "' not found");
}

value_t evaluator::do_defun(const std::vector<value_t> &args, const environment_ptr &env) {
  if (args.size() < 3)
    throw dsl_error("TypeError", "defun: expected at least 3 arguments");
  auto *name_sym = std::get_if<symbol>(&args[0].v);
  if (!name_sym)
    throw dsl_error("TypeError", "defun: first arg must be symbol");

  // Build (lambda (params...) body...)
  std::vector<value_t> lambda_args(args.begin() + 1, args.end());
  auto func = do_lambda(lambda_args, env);
  env->set(name_sym->name, func);
  return func;
}

value_t evaluator::do_defclass(const std::vector<value_t> &args, const environment_ptr &env) {
  if (args.empty())
    throw dsl_error("TypeError", "defclass: expected at least a name");
  auto *name_sym = std::get_if<symbol>(&args[0].v);
  if (!name_sym)
    throw dsl_error("TypeError", "defclass: first arg must be symbol");
  const std::string &name = name_sym->name;

  size_t methods_start = 1;
  dict_ptr super_cls_dict;

  // Check for optional parent class
  if (args.size() > 1) {
    if (auto *parent_sym = std::get_if<symbol>(&args[1].v)) {
      // (defclass Name Parent (...methods...))
      auto *parent_val = env->lookup(parent_sym->name);
      if (parent_val && std::holds_alternative<native_fn>(parent_val->v)) {
        // Constructor is a native_fn — we need the class dict
        // For now, call it to see if it builds class dicts
      }
      // Try looking for _cls_dict in the closure's env or attached data
      // We store cls_dict in the constructor closure's env under "__cls_dict__"
      if (parent_val) {
        if (auto *parent_cls = std::get_if<closure_ptr>(&parent_val->v)) {
          auto *cd = (*parent_cls)->env_snapshot->lookup("__cls_dict__");
          if (cd)
            super_cls_dict = std::get<dict_ptr>(cd->v);
        }
        // Could also be a native_fn with an attached env
        if (!super_cls_dict) {
          if (auto *nf = std::get_if<native_fn>(&parent_val->v)) {
            // Try the env
            auto *cd = env->lookup(parent_sym->name + ".__cls_dict__");
            if (cd)
              super_cls_dict = std::get<dict_ptr>(cd->v);
          }
        }
      }
      methods_start = 2;
    } else if (auto *parent_list = std::get_if<list_ptr>(&args[1].v)) {
      // (defclass Name (Parent) (...methods...))
      auto &pl = **parent_list;
      if (pl.size() == 1) {
        if (auto *ps = std::get_if<symbol>(&pl[0].v)) {
          auto *parent_val = env->lookup(ps->name);
          if (parent_val) {
            if (auto *parent_cls = std::get_if<closure_ptr>(&parent_val->v)) {
              auto *cd = (*parent_cls)->env_snapshot->lookup("__cls_dict__");
              if (cd)
                super_cls_dict = std::get<dict_ptr>(cd->v);
            }
          }
          methods_start = 2;
        }
      }
    }
  }

  // Create class dict
  auto cls_dict = std::make_shared<std::vector<std::pair<std::string, value_t>>>();
  cls_dict->emplace_back("__name__", value_t{name});
  if (super_cls_dict)
    cls_dict->emplace_back("__super__", value_t{super_cls_dict});

  // Collect method names to know if init/finalize are user-defined
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
    auto super_copy = super_cls_dict;
    native_fn default_init = [super_copy](std::span<const value_t> a) -> value_t {
      if (super_copy) {
        // Call parent init
        for (auto &[k, v] : *super_copy) {
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
    auto super_copy = super_cls_dict;
    native_fn default_finalize = [super_copy](std::span<const value_t> a) -> value_t {
      if (super_copy) {
        for (auto &[k, v] : *super_copy) {
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

  // Process method definitions
  evaluator *self = this;
  for (size_t i = methods_start; i < args.size(); ++i) {
    auto *ml = std::get_if<list_ptr>(&args[i].v);
    if (!ml || !*ml || (**ml).empty())
      continue;
    auto &mlist = **ml;
    auto *method_name_sym = std::get_if<symbol>(&mlist[0].v);
    if (!method_name_sym)
      continue;

    // Create closure for the method
    std::vector<value_t> lambda_args(mlist.begin() + 1, mlist.end());
    auto method_val = do_lambda(lambda_args, env);
    auto method_closure = std::get<closure_ptr>(method_val.v);

    // Wrap closure in a native_fn so it can be called via send
    auto cls_env = env;
    native_fn wrapper = [self, method_closure,
                         cls_env](std::span<const value_t> call_args) -> value_t {
      std::vector<value_t> v(call_args.begin(), call_args.end());
      return self->apply_closure(method_closure, v);
    };
    cls_dict->emplace_back(method_name_sym->name, value_t{std::move(wrapper)});
  }

  // Create constructor: the stackless evaluator's, so isinstance and catch find the class. Its
  // init is a native here (the wrapper above), which it calls directly.
  auto cls_dict_val = value_t{cls_dict};
  native_fn constructor{detail::class_constructor{cls_dict}};

  // Store cls_dict in env so inheritance works
  auto ctor_env = environment::extend(env);
  ctor_env->set("__cls_dict__", cls_dict_val);
  auto ctor_closure = std::make_shared<closure>();
  ctor_closure->env_snapshot = ctor_env;
  // We'll store the constructor as a native_fn but wrap it in a closure
  // so inheriting classes can find __cls_dict__
  env->set(name, value_t{std::move(constructor)});
  // Also store the cls_dict for inheritance lookup
  env->set(name + ".__cls_dict__", cls_dict_val);

  return env->lookup(name) ? *env->lookup(name) : nil_value;
}

value_t evaluator::do_defmacro(const std::vector<value_t> &args, const environment_ptr &env) {
  if (args.size() < 3)
    throw dsl_error("TypeError", "defmacro: expected at least 3 arguments");
  auto *name_sym = std::get_if<symbol>(&args[0].v);
  if (!name_sym)
    throw dsl_error("TypeError", "defmacro: name must be symbol");

  std::vector<symbol> params;
  auto *params_val = std::get_if<list_ptr>(&args[1].v);
  if (params_val && *params_val) {
    for (auto &p : **params_val) {
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

  user_macros_[name_sym->name] = {std::move(params), body};
  return value_t{symbol{name_sym->name}};
}

value_t evaluator::do_eval(const std::vector<value_t> &args, const environment_ptr &env) {
  if (args.size() != 1)
    throw dsl_error("TypeError", "eval: expected 1 argument");
  auto code = eval_internal(args[0], env);
  return eval_internal(code, env);
}

value_t evaluator::do_root() { return root_expr_; }

namespace {
// While alive, `exc` is being handled: in a catch handler, or by a finally on its way out.
struct handling_scope {
  handling_scope(std::vector<std::pair<value_t, bool>> &handling, value_t exc, bool in_handler)
      : handling(handling) {
    handling.emplace_back(std::move(exc), in_handler);
  }
  ~handling_scope() { handling.pop_back(); }
  handling_scope(const handling_scope &) = delete;
  handling_scope &operator=(const handling_scope &) = delete;
  std::vector<std::pair<value_t, bool>> &handling;
};
} // namespace

value_t evaluator::do_try(const std::vector<value_t> &args, const environment_ptr &env) {
  // (try body... (catch (Type var) handler...)... (finally cleanup...)). Handlers and the cleanup
  // run after the catch that stopped the exception has finished, so the stack is unwound first.
  const try_form form = parse_try(args);
  value_t result;
  std::exception_ptr leaving; // an exception or a return leaving the try, after the finally
  value_t raised;             // the exception leaving, handled while the finally runs
  try {
    result = do_begin(form.body, env);
  } catch (const return_exception &) {
    leaving = std::current_exception();
  } catch (const std::exception &e) {
    std::optional<value_t> exc = caught(e);
    if (!exc)
      throw; // a kill: no catch or finally runs
    raised = std::move(*exc);
    leaving = std::make_exception_ptr(dsl_exception(raised));
  }
  if (!raised.is_nil()) {
    const value_t exc = raised;
    for (const value_t &clause : form.catches) {
      std::string var;
      bool matched = false;
      try {
        matched = catch_matches(env, clause, exc, var);
      } catch (const dsl_error &bad) {
        // A catch naming no class: its NameError replaces the exception.
        value_t replaced = exception_from(bad);
        set_field(replaced, "cause", exc);
        record_raise_point();
        annotate(replaced);
        raised = replaced;
        leaving = std::make_exception_ptr(dsl_exception(std::move(replaced)));
        break;
      }
      if (!matched)
        continue;
      try {
        auto scope = environment::extend(env);
        if (!var.empty())
          scope->set(var, exc);
        handling_scope handling(handling_, exc, true);
        const auto handler = catch_handler(clause);
        result = do_begin(std::vector<value_t>(handler.begin(), handler.end()), scope);
        leaving = nullptr;
        raised = nil_value;
      } catch (const return_exception &) {
        leaving = std::current_exception();
        raised = nil_value;
      } catch (const std::exception &e) {
        std::optional<value_t> again = caught(e);
        if (!again)
          throw;
        raised = std::move(*again);
        leaving = std::make_exception_ptr(dsl_exception(raised));
      }
      break;
    }
  }
  if (!form.cleanup.is_nil()) {
    // The cleanup runs however the try is left; an exception or a return in it replaces that.
    std::optional<handling_scope> handling;
    if (!raised.is_nil())
      handling.emplace(handling_, raised, false);
    eval_internal(form.cleanup, env);
  }
  if (leaving)
    std::rethrow_exception(leaving);
  return result;
}

value_t evaluator::do_raise(const std::vector<value_t> &args, const environment_ptr &env) {
  if (args.size() > 1)
    throw dsl_error("TypeError",
                    "raise: expected at most 1 arg, got " + std::to_string(args.size()));
  if (args.empty()) {
    // (raise) in a catch handler re-raises the exception it handles.
    for (auto it = handling_.rbegin(); it != handling_.rend(); ++it)
      if (it->second)
        throw dsl_exception(it->first);
    throw dsl_error("RuntimeError", "raise: no exception is being handled");
  }
  value_t exc = eval_internal(args[0], env);
  if (!is_exception(exc))
    throw dsl_error("TypeError", "raise: expected an exception, got " + exc.type_name());
  record_raise_point();
  annotate(exc);
  throw dsl_exception(std::move(exc));
}

// ---------------------------------------------------------------------------
// closure application
// ---------------------------------------------------------------------------

value_t evaluator::apply_closure(const closure_ptr &cls, const std::vector<value_t> &args) {
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
  try {
    value_t result = nil_value;
    for (auto &b : cls->body) {
      check_interrupted();
      result = eval_internal(b, local);
    }
    return result;
  } catch (return_exception &e) {
    return std::move(e.value);
  }
}

// ---------------------------------------------------------------------------
// macro substitution
// ---------------------------------------------------------------------------

value_t evaluator::substitute(const value_t &tmpl,
                              const std::unordered_map<std::string, value_t> &subst) {
  // The stackless evaluator's walk, run whole: no native recursion per template level.
  return run_to_completion(*make_substitute_job(tmpl, subst));
}

} // namespace cvc::state_exec
