#include <atomic>
#include <cvc/state_exec/builtins.h>
#include <cvc/state_exec/evaluator.h>
#include <cvc/state_exec/parser.h>
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
  std::lock_guard lk(eval_mu_);
  root_expr_ = expr;
  stats_.start();

  auto run_env = env ? env : environment::extend(global_env_);
  if (!env)
    run_env = environment::extend(global_env_);

  try {
    value_t result;
    if (timeout_sec) {
      // Run in a background thread with timeout
      value_t bg_result;
      std::exception_ptr ex;
      std::atomic<bool> done{false};
      std::thread worker([&] {
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

value_t evaluator::eval_internal(const value_t &expr, environment_ptr env) {
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
    throw std::runtime_error("undefined symbol: " + sym->name);
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
    throw std::runtime_error("invalid expression head: " + to_string(elems[0]));

  const auto &name = head->name;
  std::vector<value_t> args(elems.begin() + 1, elems.end());

  // --- Special forms ---
  if (name == "if")
    return do_if(args, env);
  if (name == "begin")
    return do_begin(args, env);
  if (name == "while")
    return do_while(args, env);
  if (name == "for")
    return do_for(args, env);
  if (name == "set")
    return do_set(args, env);
  if (name == "quote")
    return do_quote(args);
  if (name == "lambda")
    return do_lambda(args, env);
  if (name == "return")
    return do_return(args, env);
  if (name == "let")
    return do_let(args, env);
  if (name == "super")
    return do_super(args, env);
  if (name == "defun")
    return do_defun(args, env);
  if (name == "defclass")
    return do_defclass(args, env);
  if (name == "defmacro")
    return do_defmacro(args, env);
  if (name == "eval")
    return do_eval(args, env);
  if (name == "root")
    return do_root();

  // --- User macros ---
  if (auto it = user_macros_.find(name); it != user_macros_.end()) {
    auto &[params, body] = it->second;
    std::unordered_map<std::string, value_t> subst;
    for (size_t i = 0; i < params.size(); ++i)
      subst[params[i].name] = (i < args.size()) ? args[i] : nil_value;
    return eval_internal(substitute(body, subst), env);
  }

  // --- Function call (built-in or user-defined) ---
  auto *func_val = env->lookup(name);
  if (!func_val)
    throw std::runtime_error("unknown function: " + name);

  // Evaluate arguments
  std::vector<value_t> evaled;
  evaled.reserve(args.size());
  for (auto &a : args)
    evaled.push_back(eval_internal(a, env));

  // native_fn
  if (auto *fn = std::get_if<native_fn>(&func_val->v))
    return (*fn)(evaled);

  // closure
  if (auto *cp = std::get_if<closure_ptr>(&func_val->v))
    return apply_closure(*cp, evaled);

  throw std::runtime_error("not callable: " + name);
}

// ---------------------------------------------------------------------------
// special forms
// ---------------------------------------------------------------------------

value_t evaluator::do_if(const std::vector<value_t> &args, environment_ptr env) {
  check_interrupted();
  if (args.size() < 2)
    throw std::runtime_error("if: expected at least 2 arguments");
  if (eval_internal(args[0], env).is_truthy())
    return eval_internal(args[1], env);
  if (args.size() > 2)
    return eval_internal(args[2], env);
  return nil_value;
}

value_t evaluator::do_begin(const std::vector<value_t> &args, environment_ptr env) {
  if (args.empty())
    return nil_value;
  value_t result = nil_value;
  for (auto &a : args) {
    check_interrupted();
    result = eval_internal(a, env);
  }
  return result;
}

value_t evaluator::do_while(const std::vector<value_t> &args, environment_ptr env) {
  if (args.size() < 2)
    throw std::runtime_error("while: expected 2 arguments");
  while (eval_internal(args[0], env).is_truthy()) {
    check_interrupted();
    eval_internal(args[1], env);
  }
  return nil_value;
}

value_t evaluator::do_for(const std::vector<value_t> &args, environment_ptr env) {
  if (args.size() < 3)
    throw std::runtime_error("for: expected 3 arguments");
  auto *var_sym = std::get_if<symbol>(&args[0].v);
  if (!var_sym)
    throw std::runtime_error("for: first arg must be symbol");
  auto collection = eval_internal(args[1], env);
  auto *lp = std::get_if<list_ptr>(&collection.v);
  if (!lp)
    throw std::runtime_error("for: second arg must evaluate to list");
  for (auto &item : **lp) {
    check_interrupted();
    env->set(var_sym->name, item);
    eval_internal(args[2], env);
  }
  return nil_value;
}

value_t evaluator::do_set(const std::vector<value_t> &args, environment_ptr env) {
  if (args.size() != 2)
    throw std::runtime_error("set: expected 2 arguments");
  auto *sym = std::get_if<symbol>(&args[0].v);
  if (!sym)
    throw std::runtime_error("set: first arg must be symbol");
  auto val = eval_internal(args[1], env);
  env->set(sym->name, val);
  return val;
}

value_t evaluator::do_quote(const std::vector<value_t> &args) {
  if (args.size() != 1)
    throw std::runtime_error("quote: expected 1 argument");
  return args[0];
}

value_t evaluator::do_lambda(const std::vector<value_t> &args, environment_ptr env) {
  if (args.size() < 2)
    throw std::runtime_error("lambda: expected at least 2 arguments");
  // Parse parameter list
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
    // single-symbol shorthand - not standard but handle gracefully
    throw std::runtime_error("lambda: params must be a list");
  }

  std::vector<value_t> body(args.begin() + 1, args.end());
  auto cls = std::make_shared<closure>();
  cls->params = std::move(params);
  cls->variadic = variadic;
  cls->body = std::move(body);
  cls->env_snapshot = env;
  return value_t{std::move(cls)};
}

value_t evaluator::do_return(const std::vector<value_t> &args, environment_ptr env) {
  if (args.empty())
    throw return_exception(nil_value);
  throw return_exception(eval_internal(args[0], env));
}

value_t evaluator::do_let(const std::vector<value_t> &args, environment_ptr env) {
  if (args.size() < 2)
    throw std::runtime_error("let: expected at least 2 arguments");
  auto local = environment::extend(env);
  // First arg: list of (name value) pairs
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
    local->set(sym->name, eval_internal((**bp)[1], local));
  }
  // Evaluate body expressions
  std::vector<value_t> body(args.begin() + 1, args.end());
  return do_begin(body, local);
}

value_t evaluator::do_super(const std::vector<value_t> &args, environment_ptr env) {
  if (args.size() < 2)
    throw std::runtime_error("super: expected at least 2 arguments");
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
        throw std::runtime_error("super: method not callable");
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
  throw std::runtime_error("super: method '" + method_name + "' not found");
}

value_t evaluator::do_defun(const std::vector<value_t> &args, environment_ptr env) {
  if (args.size() < 3)
    throw std::runtime_error("defun: expected at least 3 arguments");
  auto *name_sym = std::get_if<symbol>(&args[0].v);
  if (!name_sym)
    throw std::runtime_error("defun: first arg must be symbol");

  // Build (lambda (params...) body...)
  std::vector<value_t> lambda_args(args.begin() + 1, args.end());
  auto func = do_lambda(lambda_args, env);
  env->set(name_sym->name, func);
  return func;
}

value_t evaluator::do_defclass(const std::vector<value_t> &args, environment_ptr env) {
  if (args.empty())
    throw std::runtime_error("defclass: expected at least a name");
  auto *name_sym = std::get_if<symbol>(&args[0].v);
  if (!name_sym)
    throw std::runtime_error("defclass: first arg must be symbol");
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

  // Create constructor
  auto cls_dict_val = value_t{cls_dict};
  evaluator *ev = this;
  native_fn constructor = [ev, cls_dict,
                           cls_dict_val](std::span<const value_t> init_args) -> value_t {
    auto inst = std::make_shared<std::vector<std::pair<std::string, value_t>>>();
    inst->emplace_back("__class__", cls_dict_val);
    value_t inst_val{inst};
    // Call init
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

value_t evaluator::do_defmacro(const std::vector<value_t> &args, environment_ptr env) {
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
  return value_t{symbol{name_sym->name}};
}

value_t evaluator::do_eval(const std::vector<value_t> &args, environment_ptr env) {
  if (args.size() != 1)
    throw std::runtime_error("eval: expected 1 argument");
  auto code = eval_internal(args[0], env);
  return eval_internal(code, env);
}

value_t evaluator::do_root() { return root_expr_; }

// ---------------------------------------------------------------------------
// closure application
// ---------------------------------------------------------------------------

value_t evaluator::apply_closure(const closure_ptr &cls, const std::vector<value_t> &args) {
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
