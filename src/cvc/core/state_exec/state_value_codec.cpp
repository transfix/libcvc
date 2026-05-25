/*
  Copyright 2026 The University of Texas at Austin

  state_value_codec — round-trip DSL values and evaluator state through
                      cvc::state subtrees.
*/

#include <cvc/core/state.h>
#include <cvc/core/state_exec/parser.h>
#include <cvc/core/state_exec/state_value_codec.h>
#include <cvc/core/state_list.h>
#include <stdexcept>
#include <string>

namespace cvc::state_exec {

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------
namespace {

const char *TYPE_KEY = "__type__";

void set_type(cvc::state &n, const char *t) { n(TYPE_KEY).value(std::string(t)); }

// Use findDescendant for read-only lookup — children() returns full
// qualified names (e.g. "parent.child") which makes direct name
// comparison unreliable, and operator() auto-creates missing nodes.
std::string get_type(cvc::state &n) {
  cvc::state *child = n.findDescendant(TYPE_KEY);
  if (child)
    return child->value();
  return "";
}

bool has_child(cvc::state &n, const std::string &name) { return n.findDescendant(name) != nullptr; }

} // namespace

// ---------------------------------------------------------------------------
// encode_value
// ---------------------------------------------------------------------------

void encode_value(cvc::state &node, const value_t &val) {
  std::visit(
      [&](auto &&arg) {
        using T = std::decay_t<decltype(arg)>;

        if constexpr (std::is_same_v<T, std::monostate>) {
          set_type(node, "nil");
        } else if constexpr (std::is_same_v<T, bool>) {
          set_type(node, "bool");
          node.value(std::string(arg ? "true" : "false"));
        } else if constexpr (std::is_same_v<T, int64_t>) {
          set_type(node, "int");
          node.value(std::to_string(arg));
        } else if constexpr (std::is_same_v<T, double>) {
          set_type(node, "double");
          // Use enough precision for round-trip
          char buf[32];
          std::snprintf(buf, sizeof(buf), "%.17g", arg);
          node.value(std::string(buf));
        } else if constexpr (std::is_same_v<T, std::string>) {
          set_type(node, "string");
          node.value(arg);
        } else if constexpr (std::is_same_v<T, symbol>) {
          set_type(node, "symbol");
          node.value(arg.name);
        } else if constexpr (std::is_same_v<T, list_ptr>) {
          set_type(node, "list");
          if (arg) {
            state_list sl(node("__elems__"));
            for (auto &e : *arg) {
              auto &elem = sl.push_back();
              encode_value(elem, e);
            }
          }
        } else if constexpr (std::is_same_v<T, dict_ptr>) {
          set_type(node, "dict");
          if (arg) {
            auto &entries_node = node("__entries__");
            state_list sl(entries_node);
            for (auto &[k, v] : *arg) {
              auto &entry = sl.push_back();
              entry("__key__").value(k);
              encode_value(entry("__val__"), v);
            }
          }
        } else if constexpr (std::is_same_v<T, closure_ptr>) {
          set_type(node, "closure");
          if (arg) {
            // Params
            auto &params_node = node("__params__");
            state_list pl(params_node);
            for (auto &p : arg->params) {
              pl.push_back().value(p.name);
            }
            // Variadic flag
            node("__variadic__").value(std::string(arg->variadic ? "1" : "0"));
            // Body expressions
            auto &body_node = node("__body__");
            state_list bl(body_node);
            for (auto &b : arg->body) {
              encode_value(bl.push_back(), b);
            }
            // Environment snapshot
            if (arg->env_snapshot) {
              encode_environment(node("__env__"), arg->env_snapshot);
            }
          }
        } else if constexpr (std::is_same_v<T, native_fn>) {
          set_type(node, "native_fn");
          // Can't serialize functions — store a marker
          node.value(std::string("__native__"));
        } else if constexpr (std::is_same_v<T, data_object_ptr>) {
          set_type(node, "data_object");
          if (arg) {
            node("__data_type__").value(arg->type_name);
            // The payload is boost::any — store it via state::data()
            node.data(arg->payload);
          }
        }
      },
      val.v);
}

// ---------------------------------------------------------------------------
// decode_value
// ---------------------------------------------------------------------------

value_t decode_value(cvc::state &node) {
  std::string type = get_type(node);

  if (type.empty() || type == "nil")
    return nil_value;

  if (type == "bool")
    return value_t(node.value() == "true");

  if (type == "int")
    return value_t(static_cast<int64_t>(std::stoll(node.value())));

  if (type == "double")
    return value_t(std::stod(node.value()));

  if (type == "string")
    return value_t(node.value());

  if (type == "symbol")
    return value_t(symbol{node.value()});

  if (type == "list") {
    auto elems = std::make_shared<std::vector<value_t>>();
    if (has_child(node, "__elems__")) {
      state_list sl(node("__elems__"));
      elems->reserve(sl.size());
      for (size_t i = 0; i < sl.size(); ++i)
        elems->push_back(decode_value(sl.at(i)));
    }
    return value_t(std::move(elems));
  }

  if (type == "dict") {
    auto entries = std::make_shared<std::vector<std::pair<std::string, value_t>>>();
    if (has_child(node, "__entries__")) {
      state_list sl(node("__entries__"));
      entries->reserve(sl.size());
      for (size_t i = 0; i < sl.size(); ++i) {
        auto &entry = sl.at(i);
        std::string key = entry("__key__").value();
        value_t val = decode_value(entry("__val__"));
        entries->emplace_back(std::move(key), std::move(val));
      }
    }
    return value_t(std::move(entries));
  }

  if (type == "closure") {
    auto cls = std::make_shared<closure>();
    // Params
    if (has_child(node, "__params__")) {
      state_list pl(node("__params__"));
      for (size_t i = 0; i < pl.size(); ++i)
        cls->params.push_back(symbol{pl.at(i).value()});
    }
    // Variadic
    if (has_child(node, "__variadic__"))
      cls->variadic = node("__variadic__").value() == "1";
    // Body
    if (has_child(node, "__body__")) {
      state_list bl(node("__body__"));
      for (size_t i = 0; i < bl.size(); ++i)
        cls->body.push_back(decode_value(bl.at(i)));
    }
    // Environment
    if (has_child(node, "__env__"))
      cls->env_snapshot = decode_environment(node("__env__"));
    return value_t(std::move(cls));
  }

  if (type == "native_fn") {
    // Can't restore native functions — return nil
    return nil_value;
  }

  if (type == "data_object") {
    auto dobj = std::make_shared<data_object>();
    if (has_child(node, "__data_type__"))
      dobj->type_name = node("__data_type__").value();
    dobj->payload = node.data();
    return value_t(std::move(dobj));
  }

  throw std::runtime_error("decode_value: unknown type: " + type);
}

// ---------------------------------------------------------------------------
// encode_environment
// ---------------------------------------------------------------------------

void encode_environment(cvc::state &node, const environment_ptr &env) {
  if (!env) {
    set_type(node, "nil");
    return;
  }

  set_type(node, "env_chain");

  // Flatten the scope chain into an ordered list (innermost first)
  state_list sl(node("__scopes__"));
  auto current = env;
  while (current) {
    auto &scope_node = sl.push_back();
    // Store bindings as key/value entries in a state_list so we
    // don't rely on iterating children() (which returns full paths).
    state_list bindings(scope_node("__bindings__"));
    for (auto &[name, val] : current->bindings) {
      // Skip native_fn bindings — they can't be serialized and will
      // be re-registered from builtins on decode
      if (std::holds_alternative<native_fn>(val.v))
        continue;
      auto &entry = bindings.push_back();
      entry("__key__").value(name);
      encode_value(entry("__val__"), val);
    }
    current = current->outer;
  }
}

// ---------------------------------------------------------------------------
// decode_environment
// ---------------------------------------------------------------------------

environment_ptr decode_environment(cvc::state &node) {
  std::string type = get_type(node);
  if (type == "nil" || type.empty())
    return nullptr;

  if (!has_child(node, "__scopes__"))
    return nullptr;

  state_list sl(node("__scopes__"));
  if (sl.empty())
    return nullptr;

  // Reconstruct chain from outermost to innermost
  // Scopes are stored innermost-first, so we reverse
  std::vector<environment_ptr> scopes;
  for (size_t i = 0; i < sl.size(); ++i) {
    auto env = std::make_shared<environment>();
    auto &scope_node = sl.at(i);
    if (has_child(scope_node, "__bindings__")) {
      state_list bindings(scope_node("__bindings__"));
      for (size_t j = 0; j < bindings.size(); ++j) {
        auto &entry = bindings.at(j);
        std::string key = entry("__key__").value();
        env->bindings[key] = decode_value(entry("__val__"));
      }
    }
    scopes.push_back(std::move(env));
  }

  // Link chain: scopes[0] is innermost, scopes[N-1] is outermost
  for (size_t i = 0; i + 1 < scopes.size(); ++i)
    scopes[i]->outer = scopes[i + 1];

  return scopes[0];
}

// ---------------------------------------------------------------------------
// encode_eval_frame
// ---------------------------------------------------------------------------
namespace {

std::string phase_to_string(eval_phase p) {
  switch (p) {
  case eval_phase::init:
    return "init";
  case eval_phase::eval_args:
    return "eval_args";
  case eval_phase::apply:
    return "apply";
  case eval_phase::if_cond:
    return "if_cond";
  case eval_phase::if_branch:
    return "if_branch";
  case eval_phase::begin_next:
    return "begin_next";
  case eval_phase::while_cond:
    return "while_cond";
  case eval_phase::while_body:
    return "while_body";
  case eval_phase::for_body:
    return "for_body";
  case eval_phase::set_value:
    return "set_value";
  case eval_phase::let_bindings:
    return "let_bindings";
  case eval_phase::let_body:
    return "let_body";
  case eval_phase::return_value:
    return "return_value";
  case eval_phase::eval_inner:
    return "eval_inner";
  case eval_phase::super_args:
    return "super_args";
  case eval_phase::defclass_methods:
    return "defclass_methods";
  }
  return "init";
}

eval_phase string_to_phase(const std::string &s) {
  if (s == "eval_args")
    return eval_phase::eval_args;
  if (s == "apply")
    return eval_phase::apply;
  if (s == "if_cond")
    return eval_phase::if_cond;
  if (s == "if_branch")
    return eval_phase::if_branch;
  if (s == "begin_next")
    return eval_phase::begin_next;
  if (s == "while_cond")
    return eval_phase::while_cond;
  if (s == "while_body")
    return eval_phase::while_body;
  if (s == "for_body")
    return eval_phase::for_body;
  if (s == "set_value")
    return eval_phase::set_value;
  if (s == "let_bindings")
    return eval_phase::let_bindings;
  if (s == "let_body")
    return eval_phase::let_body;
  if (s == "return_value")
    return eval_phase::return_value;
  if (s == "eval_inner")
    return eval_phase::eval_inner;
  if (s == "super_args")
    return eval_phase::super_args;
  if (s == "defclass_methods")
    return eval_phase::defclass_methods;
  return eval_phase::init;
}

void encode_value_list(cvc::state &node, const std::vector<value_t> &vals) {
  state_list sl(node);
  for (auto &v : vals)
    encode_value(sl.push_back(), v);
}

std::vector<value_t> decode_value_list(cvc::state &node) {
  state_list sl(node);
  std::vector<value_t> result;
  result.reserve(sl.size());
  for (size_t i = 0; i < sl.size(); ++i)
    result.push_back(decode_value(sl.at(i)));
  return result;
}

void encode_frame(cvc::state &node, const eval_frame &f) {
  encode_value(node("expr"), f.expr);
  encode_environment(node("env"), f.env);
  node("phase").value(phase_to_string(f.phase));
  node("index").value(std::to_string(f.index));

  // results
  if (!f.results.empty())
    encode_value_list(node("results"), f.results);

  // extra_vals
  if (!f.extra_vals.empty())
    encode_value_list(node("extra_vals"), f.extra_vals);

  // extra_strs
  if (!f.extra_strs.empty()) {
    state_list sl(node("extra_strs"));
    for (auto &s : f.extra_strs)
      sl.push_back().value(s);
  }

  // extra_lists
  if (!f.extra_lists.empty()) {
    state_list sl(node("extra_lists"));
    for (auto &inner : f.extra_lists)
      encode_value_list(sl.push_back(), inner);
  }

  // extra_env
  if (f.extra_env)
    encode_environment(node("extra_env"), f.extra_env);
}

eval_frame decode_frame(cvc::state &node) {
  eval_frame f;
  f.expr = decode_value(node("expr"));
  f.env = decode_environment(node("env"));
  f.phase = string_to_phase(node("phase").value());
  f.index = std::stoll(node("index").value());

  if (has_child(node, "results"))
    f.results = decode_value_list(node("results"));

  if (has_child(node, "extra_vals"))
    f.extra_vals = decode_value_list(node("extra_vals"));

  if (has_child(node, "extra_strs")) {
    state_list sl(node("extra_strs"));
    for (size_t i = 0; i < sl.size(); ++i)
      f.extra_strs.push_back(sl.at(i).value());
  }

  if (has_child(node, "extra_lists")) {
    state_list sl(node("extra_lists"));
    for (size_t i = 0; i < sl.size(); ++i)
      f.extra_lists.push_back(decode_value_list(sl.at(i)));
  }

  if (has_child(node, "extra_env"))
    f.extra_env = decode_environment(node("extra_env"));

  return f;
}

} // namespace

// ---------------------------------------------------------------------------
// encode_evaluator_state
// ---------------------------------------------------------------------------

void encode_evaluator_state(cvc::state &node, const evaluator_state &es) {
  set_type(node, "evaluator_state");

  // Root expression
  encode_value(node("root_expr"), es.root_expr);

  // Result
  encode_value(node("result"), es.result);

  // Done flag
  node("done").value(std::string(es.done ? "1" : "0"));

  // Frame stack
  {
    state_list sl(node("stack"));
    for (auto &f : es.stack)
      encode_frame(sl.push_back(), f);
  }

  // Global environment
  if (es.global_env)
    encode_environment(node("global_env"), es.global_env);

  // User macros — store as state_list entries with explicit names
  if (!es.user_macros.empty()) {
    state_list ml(node("user_macros"));
    for (auto &[name, macro] : es.user_macros) {
      auto &entry = ml.push_back();
      entry("__name__").value(name);
      // Params
      state_list pl(entry("params"));
      for (auto &p : macro.first)
        pl.push_back().value(p.name);
      // Body
      encode_value(entry("body"), macro.second);
    }
  }

  // Stats snapshot
  node("step_count").value(std::to_string(es.stats.get_step_count()));
}

// ---------------------------------------------------------------------------
// decode_evaluator_state
// ---------------------------------------------------------------------------

evaluator_state decode_evaluator_state(cvc::state &node) {
  evaluator_state es;

  // Root expression
  es.root_expr = decode_value(node("root_expr"));

  // Result
  es.result = decode_value(node("result"));

  // Done
  es.done = has_child(node, "done") && node("done").value() == "1";

  // Frame stack
  if (has_child(node, "stack")) {
    state_list sl(node("stack"));
    for (size_t i = 0; i < sl.size(); ++i)
      es.stack.push_back(decode_frame(sl.at(i)));
  }

  // Global environment
  if (has_child(node, "global_env"))
    es.global_env = decode_environment(node("global_env"));

  // User macros
  if (has_child(node, "user_macros")) {
    state_list ml(node("user_macros"));
    for (size_t i = 0; i < ml.size(); ++i) {
      auto &entry = ml.at(i);
      std::string name = entry("__name__").value();
      std::vector<symbol> params;
      if (has_child(entry, "params")) {
        state_list pl(entry("params"));
        for (size_t j = 0; j < pl.size(); ++j)
          params.push_back(symbol{pl.at(j).value()});
      }
      value_t body = decode_value(entry("body"));
      es.user_macros[name] = {std::move(params), std::move(body)};
    }
  }

  return es;
}

} // namespace cvc::state_exec
