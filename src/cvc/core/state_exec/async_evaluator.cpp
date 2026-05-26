/*
  Copyright 2026 The University of Texas at Austin

  async_evaluator — C++20 coroutine-based recursive evaluator.

  Mirrors the synchronous evaluator word-for-word, but every recursive
  call uses co_await to enable cooperative yielding.  The (await expr)
  special form suspends the coroutine after evaluating expr.
*/

#include <cvc/core/state_exec/async_evaluator.h>
#include <cvc/core/state_exec/builtins.h>
#include <cvc/core/state_exec/parser.h>
#include <thread>

namespace cvc::state_exec {

// ---------------------------------------------------------------------------
// construction
// ---------------------------------------------------------------------------

async_evaluator::async_evaluator(environment_ptr global_env) : global_env_(std::move(global_env)) {}

// ---------------------------------------------------------------------------
// public API — coroutines
// ---------------------------------------------------------------------------

task<value_t> async_evaluator::evaluate(value_t expr, environment_ptr env) {
  auto run_env = env ? env : environment::extend(global_env_);
  root_expr_ = expr;
  stats_.start();
  try {
    value_t result = co_await eval_internal(expr, run_env);
    stats_.mark_complete();
    co_return result;
  } catch (...) {
    stats_.mark_complete();
    throw;
  }
}

task<value_t> async_evaluator::evaluate_script(const std::string &script, environment_ptr env) {
  // Non-coroutine: parse outside any coroutine frame then delegate to
  // evaluate().  Keeping the parser/vector work out of the coroutine frame
  // avoids an MSVC coroutine-frame-corruption bug that manifests as
  // "bad variant access" on symmetric transfer.
  auto exprs = parse_all(script);
  if (exprs.empty())
    return evaluate(nil_value, env);
  if (exprs.size() == 1)
    return evaluate(std::move(exprs[0]), env);
  std::vector<value_t> begin_exprs;
  begin_exprs.reserve(1 + exprs.size());
  begin_exprs.push_back(value_t{symbol{"begin"}});
  begin_exprs.insert(begin_exprs.end(), exprs.begin(), exprs.end());
  return evaluate(make_list(std::move(begin_exprs)), env);
}

// ---------------------------------------------------------------------------
// public API — synchronous wrappers
// ---------------------------------------------------------------------------

value_t async_evaluator::sync_evaluate(const value_t &expr, environment_ptr env,
                                       std::optional<double> timeout_sec,
                                       std::function<void(const value_t &)> on_complete) {
  std::lock_guard lk(eval_mu_);
  auto t = evaluate(expr, env);
  if (timeout_sec) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(*timeout_sec);
    // Use the trampoline to avoid stack overflow from deep recursive coroutines.
    t.handle().promise().continuation_ = std::noop_coroutine();
    detail::trampoline_next() = t.handle();
    while (!t.done()) {
      if (std::chrono::steady_clock::now() >= deadline) {
        interrupt();
        // Drive coroutine to propagate the interrupt
        try {
          while (!t.done()) {
            auto next = std::exchange(detail::trampoline_next(), std::noop_coroutine());
            if (next == std::noop_coroutine())
              break;
            next.resume();
          }
        } catch (...) {
        }
        reset_interrupt();
        throw evaluation_timeout("evaluation exceeded timeout");
      }
      auto next = std::exchange(detail::trampoline_next(), std::noop_coroutine());
      if (next == std::noop_coroutine())
        break;
      next.resume();
    }
    auto &r = t.handle().promise().result_;
    if (r.index() == 2)
      std::rethrow_exception(std::get<2>(r));
    auto result = std::move(std::get<1>(r));
    if (on_complete)
      on_complete(result);
    return result;
  }
  auto result = t.sync_wait();
  if (on_complete)
    on_complete(result);
  return result;
}

value_t async_evaluator::sync_evaluate_script(const std::string &script, environment_ptr env,
                                              std::optional<double> timeout_sec,
                                              std::function<void(const value_t &)> on_complete) {
  auto exprs = parse_all(script);
  if (exprs.empty())
    return nil_value;
  if (exprs.size() == 1)
    return sync_evaluate(exprs[0], env, timeout_sec, std::move(on_complete));
  std::vector<value_t> begin_exprs;
  begin_exprs.reserve(1 + exprs.size());
  begin_exprs.push_back(value_t{symbol{"begin"}});
  begin_exprs.insert(begin_exprs.end(), exprs.begin(), exprs.end());
  return sync_evaluate(make_list(std::move(begin_exprs)), env, timeout_sec, std::move(on_complete));
}

// ---------------------------------------------------------------------------
// interrupt / pause
// ---------------------------------------------------------------------------

void async_evaluator::interrupt() { interrupted_.store(true, std::memory_order_release); }
void async_evaluator::reset_interrupt() { interrupted_.store(false, std::memory_order_release); }

void async_evaluator::pause() { paused_.store(true, std::memory_order_release); }

void async_evaluator::resume() {
  paused_.store(false, std::memory_order_release);
  pause_cv_.notify_all();
}

bool async_evaluator::is_paused() const { return paused_.load(std::memory_order_acquire); }

task<void> async_evaluator::check_interrupted_async() {
  if (interrupted_.load(std::memory_order_acquire))
    throw evaluation_interrupted("evaluation was interrupted");
  if (paused_.load(std::memory_order_acquire)) {
    std::unique_lock lk(pause_mu_);
    pause_cv_.wait(lk, [this] { return !paused_.load(std::memory_order_acquire); });
  }
  co_return;
}

// ---------------------------------------------------------------------------
// core eval
// ---------------------------------------------------------------------------

task<value_t> async_evaluator::eval_internal(const value_t &expr, environment_ptr env) {
  co_await check_interrupted_async();
  stats_.increment_step();

  // Symbol
  if (auto *sym = std::get_if<symbol>(&expr.v)) {
    if (sym->name == "nil")
      co_return nil_value;
    if (sym->name == "t")
      co_return true_value;
    if (auto *val = env->lookup(sym->name))
      co_return *val;
    throw std::runtime_error("undefined symbol: " + sym->name);
  }

  // Non-list atom
  if (!std::holds_alternative<list_ptr>(expr.v))
    co_return expr;

  auto *lp = std::get_if<list_ptr>(&expr.v);
  if (!lp || !*lp)
    co_return expr;
  auto &elems = **lp;
  if (elems.empty())
    co_return make_list();

  auto *head = std::get_if<symbol>(&elems[0].v);
  if (!head)
    throw std::runtime_error("invalid expression head: " + to_string(elems[0]));

  const auto &name = head->name;
  std::vector<value_t> args(elems.begin() + 1, elems.end());

  // --- Special forms ---
  if (name == "if")
    co_return co_await do_if(args, env);
  if (name == "begin")
    co_return co_await do_begin(args, env);
  if (name == "while")
    co_return co_await do_while(args, env);
  if (name == "for")
    co_return co_await do_for(args, env);
  if (name == "set")
    co_return co_await do_set(args, env);
  if (name == "quote")
    co_return co_await do_quote(args);
  if (name == "lambda")
    co_return co_await do_lambda(args, env);
  if (name == "return")
    co_return co_await do_return(args, env);
  if (name == "let")
    co_return co_await do_let(args, env);
  if (name == "super")
    co_return co_await do_super(args, env);
  if (name == "defun")
    co_return co_await do_defun(args, env);
  if (name == "defclass")
    co_return co_await do_defclass(args, env);
  if (name == "defmacro")
    co_return co_await do_defmacro(args, env);
  if (name == "eval")
    co_return co_await do_eval(args, env);
  if (name == "root")
    co_return co_await do_root();
  if (name == "await")
    co_return co_await do_await(args, env);

  // --- User macros ---
  if (auto it = user_macros_.find(name); it != user_macros_.end()) {
    auto &[params, body] = it->second;
    std::unordered_map<std::string, value_t> subst;
    for (size_t i = 0; i < params.size(); ++i)
      subst[params[i].name] = (i < args.size()) ? args[i] : nil_value;
    co_return co_await eval_internal(substitute(body, subst), env);
  }

  // --- Function call ---
  auto *func_val = env->lookup(name);
  if (!func_val)
    throw std::runtime_error("unknown function: " + name);

  std::vector<value_t> evaled;
  evaled.reserve(args.size());
  for (auto &a : args)
    evaled.push_back(co_await eval_internal(a, env));

  if (auto *fn = std::get_if<native_fn>(&func_val->v))
    co_return (*fn)(evaled);

  if (auto *cp = std::get_if<closure_ptr>(&func_val->v))
    co_return co_await apply_closure(*cp, evaled);

  throw std::runtime_error("not callable: " + name);
}

// ---------------------------------------------------------------------------
// special forms
// ---------------------------------------------------------------------------

task<value_t> async_evaluator::do_if(const std::vector<value_t> &args, environment_ptr env) {
  if (args.size() < 2)
    throw std::runtime_error("if: expected at least 2 arguments");
  auto cond = co_await eval_internal(args[0], env);
  if (cond.is_truthy())
    co_return co_await eval_internal(args[1], env);
  if (args.size() > 2)
    co_return co_await eval_internal(args[2], env);
  co_return nil_value;
}

task<value_t> async_evaluator::do_begin(const std::vector<value_t> &args, environment_ptr env) {
  if (args.empty())
    co_return nil_value;
  value_t result = nil_value;
  for (auto &a : args) {
    co_await check_interrupted_async();
    result = co_await eval_internal(a, env);
  }
  co_return result;
}

task<value_t> async_evaluator::do_while(const std::vector<value_t> &args, environment_ptr env) {
  if (args.size() < 2)
    throw std::runtime_error("while: expected 2 arguments");
  while ((co_await eval_internal(args[0], env)).is_truthy()) {
    co_await check_interrupted_async();
    co_await eval_internal(args[1], env);
  }
  co_return nil_value;
}

task<value_t> async_evaluator::do_for(const std::vector<value_t> &args, environment_ptr env) {
  if (args.size() < 3)
    throw std::runtime_error("for: expected 3 arguments");
  auto *var_sym = std::get_if<symbol>(&args[0].v);
  if (!var_sym)
    throw std::runtime_error("for: first arg must be symbol");
  auto collection = co_await eval_internal(args[1], env);
  auto *lp = std::get_if<list_ptr>(&collection.v);
  if (!lp)
    throw std::runtime_error("for: second arg must evaluate to list");
  for (auto &item : **lp) {
    co_await check_interrupted_async();
    env->set(var_sym->name, item);
    co_await eval_internal(args[2], env);
  }
  co_return nil_value;
}

task<value_t> async_evaluator::do_set(const std::vector<value_t> &args, environment_ptr env) {
  if (args.size() != 2)
    throw std::runtime_error("set: expected 2 arguments");
  auto *sym = std::get_if<symbol>(&args[0].v);
  if (!sym)
    throw std::runtime_error("set: first arg must be symbol");
  auto val = co_await eval_internal(args[1], env);
  env->set(sym->name, val);
  co_return val;
}

task<value_t> async_evaluator::do_quote(const std::vector<value_t> &args) {
  if (args.size() != 1)
    throw std::runtime_error("quote: expected 1 argument");
  co_return args[0];
}

task<value_t> async_evaluator::do_lambda(const std::vector<value_t> &args, environment_ptr env) {
  if (args.size() < 2)
    throw std::runtime_error("lambda: expected at least 2 arguments");
  std::vector<symbol> params;
  bool variadic = false;
  auto *params_val = std::get_if<list_ptr>(&args[0].v);
  if (params_val && *params_val) {
    for (auto &p : **params_val) {
      auto *ps = std::get_if<symbol>(&p.v);
      if (!ps)
        throw std::runtime_error("lambda: param must be symbol");
      if (ps->name == "&rest") {
        variadic = true;
        continue;
      }
      params.push_back(*ps);
    }
  } else if (auto *nil_sym = std::get_if<symbol>(&args[0].v); nil_sym && nil_sym->name == "nil") {
    // empty params
  } else if (!std::holds_alternative<list_ptr>(args[0].v)) {
    throw std::runtime_error("lambda: params must be a list");
  }

  std::vector<value_t> body(args.begin() + 1, args.end());
  auto cls = std::make_shared<closure>();
  cls->params = std::move(params);
  cls->variadic = variadic;
  cls->body = std::move(body);
  cls->env_snapshot = env;
  co_return value_t{std::move(cls)};
}

task<value_t> async_evaluator::do_return(const std::vector<value_t> &args, environment_ptr env) {
  if (args.empty())
    throw return_exception(nil_value);
  auto val = co_await eval_internal(args[0], env);
  throw return_exception(std::move(val));
}

task<value_t> async_evaluator::do_let(const std::vector<value_t> &args, environment_ptr env) {
  if (args.size() < 2)
    throw std::runtime_error("let: expected at least 2 arguments");
  auto local = environment::extend(env);
  auto *bindings = std::get_if<list_ptr>(&args[0].v);
  if (!bindings)
    throw std::runtime_error("let: first arg must be binding list");
  for (auto &b : **bindings) {
    auto *bp = std::get_if<list_ptr>(&b.v);
    if (!bp || (*bp)->size() != 2)
      throw std::runtime_error("let: each binding must be (name value)");
    auto *sym = std::get_if<symbol>(&(**bp)[0].v);
    if (!sym)
      throw std::runtime_error("let: binding name must be symbol");
    local->set(sym->name, co_await eval_internal((**bp)[1], local));
  }
  std::vector<value_t> body(args.begin() + 1, args.end());
  co_return co_await do_begin(body, local);
}

task<value_t> async_evaluator::do_super(const std::vector<value_t> &args, environment_ptr env) {
  if (args.size() < 2)
    throw std::runtime_error("super: expected at least 2 arguments");
  auto self_val = co_await eval_internal(args[0], env);
  auto *method_sym = std::get_if<symbol>(&args[1].v);
  std::string method_name;
  if (method_sym)
    method_name = method_sym->name;
  else
    method_name = std::get<std::string>((co_await eval_internal(args[1], env)).v);

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
  auto *super_dict = &(*std::get<dict_ptr>(super_val->v));

  while (super_dict) {
    for (auto &[k, v] : *super_dict) {
      if (k == method_name) {
        if (auto *fn = std::get_if<native_fn>(&v.v)) {
          std::vector<value_t> fn_args;
          fn_args.push_back(self_val);
          for (size_t i = 2; i < args.size(); ++i)
            fn_args.push_back(co_await eval_internal(args[i], env));
          co_return (*fn)(fn_args);
        }
        if (auto *cp = std::get_if<closure_ptr>(&v.v)) {
          std::vector<value_t> fn_args;
          fn_args.push_back(self_val);
          for (size_t i = 2; i < args.size(); ++i)
            fn_args.push_back(co_await eval_internal(args[i], env));
          co_return co_await apply_closure(*cp, fn_args);
        }
        throw std::runtime_error("super: method not callable");
      }
    }
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
  throw std::runtime_error("super: method '" + method_name + "' not found");
}

task<value_t> async_evaluator::do_defun(const std::vector<value_t> &args, environment_ptr env) {
  if (args.size() < 3)
    throw std::runtime_error("defun: expected at least 3 arguments");
  auto *name_sym = std::get_if<symbol>(&args[0].v);
  if (!name_sym)
    throw std::runtime_error("defun: first arg must be symbol");
  std::vector<value_t> lambda_args(args.begin() + 1, args.end());
  auto func = co_await do_lambda(lambda_args, env);
  env->set(name_sym->name, func);
  co_return func;
}

task<value_t> async_evaluator::do_defclass(const std::vector<value_t> &args, environment_ptr env) {
  if (args.empty())
    throw std::runtime_error("defclass: expected at least a name");
  auto *name_sym = std::get_if<symbol>(&args[0].v);
  if (!name_sym)
    throw std::runtime_error("defclass: first arg must be symbol");
  const std::string &name = name_sym->name;

  size_t methods_start = 1;
  dict_ptr super_cls_dict;

  if (args.size() > 1) {
    if (auto *parent_sym = std::get_if<symbol>(&args[1].v)) {
      auto *parent_val = env->lookup(parent_sym->name);
      if (parent_val) {
        if (auto *parent_cls = std::get_if<closure_ptr>(&parent_val->v)) {
          auto *cd = (*parent_cls)->env_snapshot->lookup("__cls_dict__");
          if (cd)
            super_cls_dict = std::get<dict_ptr>(cd->v);
        }
        if (!super_cls_dict) {
          auto *cd = env->lookup(parent_sym->name + ".__cls_dict__");
          if (cd)
            super_cls_dict = std::get<dict_ptr>(cd->v);
        }
      }
      methods_start = 2;
    } else if (auto *parent_list = std::get_if<list_ptr>(&args[1].v)) {
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

  auto cls_dict = std::make_shared<std::vector<std::pair<std::string, value_t>>>();
  if (super_cls_dict)
    cls_dict->emplace_back("__super__", value_t{super_cls_dict});

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

  if (!has_method("init")) {
    auto super_copy = super_cls_dict;
    native_fn default_init = [super_copy](std::span<const value_t> a) -> value_t {
      if (super_copy) {
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

  if (!has_method("finalize")) {
    auto super_copy = super_cls_dict;
    native_fn default_finalize = [super_copy](std::span<const value_t>) -> value_t {
      if (super_copy) {
        for (auto &[k, v] : *super_copy) {
          if (k == "finalize") {
            if (auto *fn = std::get_if<native_fn>(&v.v))
              return (*fn)({});
            break;
          }
        }
      }
      return nil_value;
    };
    cls_dict->emplace_back("finalize", value_t{std::move(default_finalize)});
  }

  // Process methods — note: we capture `this` for apply_closure
  async_evaluator *self = this;
  for (size_t i = methods_start; i < args.size(); ++i) {
    auto *ml = std::get_if<list_ptr>(&args[i].v);
    if (!ml || !*ml || (**ml).empty())
      continue;
    auto &mlist = **ml;
    auto *method_name_sym = std::get_if<symbol>(&mlist[0].v);
    if (!method_name_sym)
      continue;

    std::vector<value_t> lambda_args(mlist.begin() + 1, mlist.end());
    auto method_val = co_await do_lambda(lambda_args, env);
    auto method_closure = std::get<closure_ptr>(method_val.v);

    // Wrap in native_fn — the native_fn is synchronous, so we use
    // sync_wait() on the async apply_closure
    native_fn wrapper = [self, method_closure](std::span<const value_t> call_args) -> value_t {
      std::vector<value_t> v(call_args.begin(), call_args.end());
      return self->apply_closure(method_closure, v).sync_wait();
    };
    cls_dict->emplace_back(method_name_sym->name, value_t{std::move(wrapper)});
  }

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

  auto ctor_env = environment::extend(env);
  ctor_env->set("__cls_dict__", cls_dict_val);
  env->set(name, value_t{std::move(constructor)});
  env->set(name + ".__cls_dict__", cls_dict_val);

  co_return env->lookup(name) ? *env->lookup(name) : nil_value;
}

task<value_t> async_evaluator::do_defmacro(const std::vector<value_t> &args, environment_ptr env) {
  if (args.size() < 3)
    throw std::runtime_error("defmacro: expected at least 3 arguments");
  auto *name_sym = std::get_if<symbol>(&args[0].v);
  if (!name_sym)
    throw std::runtime_error("defmacro: name must be symbol");

  std::vector<symbol> params;
  auto *params_val = std::get_if<list_ptr>(&args[1].v);
  if (params_val && *params_val) {
    for (auto &p : **params_val) {
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

  user_macros_[name_sym->name] = {std::move(params), body};
  co_return value_t{symbol{name_sym->name}};
}

task<value_t> async_evaluator::do_eval(const std::vector<value_t> &args, environment_ptr env) {
  if (args.size() != 1)
    throw std::runtime_error("eval: expected 1 argument");
  auto code = co_await eval_internal(args[0], env);
  co_return co_await eval_internal(code, env);
}

task<value_t> async_evaluator::do_root() { co_return root_expr_; }

task<value_t> async_evaluator::do_await(const std::vector<value_t> &args, environment_ptr env) {
  if (args.size() != 1)
    throw std::runtime_error("await: expected 1 argument");
  // Evaluate the expression, then yield control once.
  // Note: suspend_point yields from the innermost coroutine. When
  // driven via sync_wait(), the outermost handle_.resume() picks up
  // where the suspension happened. In a true scheduler context the
  // scheduler would resume the outermost handle on the next tick.
  co_return co_await eval_internal(args[0], env);
}

// ---------------------------------------------------------------------------
// closure application
// ---------------------------------------------------------------------------

task<value_t> async_evaluator::apply_closure(const closure_ptr &cls,
                                             const std::vector<value_t> &args) {
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
  try {
    value_t result = nil_value;
    for (auto &b : cls->body) {
      co_await check_interrupted_async();
      result = co_await eval_internal(b, local);
    }
    co_return result;
  } catch (return_exception &e) {
    co_return std::move(e.value);
  }
}

// ---------------------------------------------------------------------------
// macro substitution (non-coroutine — pure transformation)
// ---------------------------------------------------------------------------

value_t async_evaluator::substitute(const value_t &tmpl,
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
