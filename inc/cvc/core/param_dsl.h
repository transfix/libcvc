/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// A params struct in the DSL, from its cvc::param_table (cvc/core/param_table.h): the dict an
// intrinsic takes as its parameters, and the app's registry of tables behind
// (params.defaults name) and (params.check name dict).
//
//   params_from_value<blur_params>(arg)  nil -> the defaults; a dict -> its keys over the
//                                        defaults (an unknown key is an error naming the valid
//                                        ones); then validated. Values may be DSL numbers, bools,
//                                        strings, symbols (an enum's name) or lists, or text in
//                                        param_table.h's forms.
//   params_to_dict(p)                    the struct as a dict: numbers as numbers (an unsigned
//                                        value past int64 as its text), bools as #t/#f, enums by
//                                        name, lists as lists.
//
// The registry is the app's own (in its data map, so it is created and destroyed with the app; no
// process-wide table): register_params<P>(app, "segment.seeded") once at setup, and then
//
//   (params.defaults "segment.seeded")         -> the defaults as a dict
//   (params.check "segment.seeded" (dict ...)) -> a list of what is wrong; () when nothing is
//
// in every environment register_param_intrinsics(env, app) was given. An unknown name raises
// ValueError listing the registered ones. cvc/core/job_intrinsics.h registers each operation's
// table under the operation's name.

#ifndef CVC_CORE_PARAM_DSL_H
#define CVC_CORE_PARAM_DSL_H

#include <cmath>
#include <cstdint>
#include <cvc/core/param_table.h>
#include <cvc/state/state_exec/types.h>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

namespace cvc {
class app;

namespace param_detail {

namespace se = cvc::state_exec;

template <class T> se::value_t scalar_value(const T &v) {
  if constexpr (std::is_same_v<T, bool>)
    return se::value_t(static_cast<bool>(v));
  else if constexpr (is_integer<T>) {
    if constexpr (std::is_unsigned_v<T> && sizeof(T) >= sizeof(std::int64_t)) {
      if (v > static_cast<T>(std::numeric_limits<std::int64_t>::max()))
        return se::value_t(std::to_string(v)); // past int64: its text, which reads back exactly
    }
    return se::value_t(static_cast<std::int64_t>(v));
  } else if constexpr (std::is_floating_point_v<T>)
    return se::value_t(static_cast<double>(v));
  else if constexpr (std::is_same_v<T, std::string>)
    return se::value_t(v);
  else
    return se::value_t(scalar_text(v)); // an enum, by name
}

template <class T> se::value_t field_value(const T &v) {
  if constexpr (is_vector<T>::value) {
    std::vector<se::value_t> items;
    items.reserve(v.size());
    for (std::size_t i = 0; i < v.size(); ++i)
      items.push_back(scalar_value<typename T::value_type>(v[i]));
    return se::make_list(std::move(items));
  } else {
    return scalar_value(v);
  }
}

// What a DSL value is called in an error.
inline std::string dsl_kind(const se::value_t &v) {
  if (v.is_nil())
    return "nil";
  if (std::holds_alternative<bool>(v.v))
    return "a bool";
  if (std::holds_alternative<std::int64_t>(v.v))
    return "an integer";
  if (std::holds_alternative<double>(v.v))
    return "a float";
  if (std::holds_alternative<std::string>(v.v))
    return "a string";
  if (std::holds_alternative<se::symbol>(v.v))
    return "a symbol";
  if (std::holds_alternative<se::list_ptr>(v.v))
    return "a list";
  if (std::holds_alternative<se::dict_ptr>(v.v))
    return "a dict";
  return v.type_name();
}

template <class T> bool scalar_from_value(const se::value_t &v, T &out, std::string &why) {
  if (const std::string *s = std::get_if<std::string>(&v.v)) {
    if (!scalar_from_text<T>(*s, out)) {
      why = "'" + *s + "' is not " + expected_of<T>();
      return false;
    }
    return true;
  }
  if constexpr (is_described_enum<T>) {
    if (const se::symbol *sym = std::get_if<se::symbol>(&v.v)) {
      if (enum_from_name(sym->name, out))
        return true;
      why = "'" + sym->name + "' is not " + expected_of<T>();
      return false;
    }
  }
  if constexpr (std::is_same_v<T, bool>) {
    if (const bool *b = std::get_if<bool>(&v.v)) {
      out = *b;
      return true;
    }
    if (const std::int64_t *i = std::get_if<std::int64_t>(&v.v)) {
      if (*i == 0 || *i == 1) {
        out = *i == 1;
        return true;
      }
    }
  } else if constexpr (is_integer<T>) {
    if (const std::int64_t *i = std::get_if<std::int64_t>(&v.v)) {
      if constexpr (std::is_unsigned_v<T>) {
        if (*i >= 0 && static_cast<std::uint64_t>(*i) <= std::numeric_limits<T>::max()) {
          out = static_cast<T>(*i);
          return true;
        }
      } else {
        if (*i >= static_cast<std::int64_t>(std::numeric_limits<T>::lowest()) &&
            *i <= static_cast<std::int64_t>(std::numeric_limits<T>::max())) {
          out = static_cast<T>(*i);
          return true;
        }
      }
      why = std::to_string(*i) + " is out of range for " + expected_of<T>();
      return false;
    }
    if (const double *d = std::get_if<double>(&v.v)) {
      if (integral_in_range<T>(*d)) {
        out = static_cast<T>(*d);
        return true;
      }
      why = format_double(*d) + " is not " + expected_of<T>();
      return false;
    }
  } else if constexpr (std::is_floating_point_v<T>) {
    if (const double *d = std::get_if<double>(&v.v)) {
      out = static_cast<T>(*d);
      return true;
    }
    if (const std::int64_t *i = std::get_if<std::int64_t>(&v.v)) {
      out = static_cast<T>(*i);
      return true;
    }
  } else if constexpr (std::is_same_v<T, std::string>) {
    if (const se::symbol *sym = std::get_if<se::symbol>(&v.v)) {
      out = sym->name;
      return true;
    }
  }
  why = dsl_kind(v) + " is not " + expected_of<T>();
  return false;
}

template <class T> bool field_from_value(const se::value_t &v, T &out, std::string &why) {
  if constexpr (is_vector<T>::value) {
    using E = typename T::value_type;
    if (const std::string *s = std::get_if<std::string>(&v.v))
      return value_from_text(*s, out, why);
    const se::list_ptr *l = std::get_if<se::list_ptr>(&v.v);
    if (!l || !*l) {
      why = dsl_kind(v) + " is not a list";
      return false;
    }
    T items;
    items.reserve((*l)->size());
    for (std::size_t i = 0; i < (*l)->size(); ++i) {
      E e{};
      std::string w;
      if (!scalar_from_value<E>((**l)[i], e, w)) {
        why = "element " + std::to_string(i) + ": " + w;
        return false;
      }
      items.push_back(std::move(e));
    }
    out = std::move(items);
    return true;
  } else {
    return scalar_from_value(v, out, why);
  }
}

} // namespace param_detail

// The struct as a DSL dict, in table order.
template <class P> state_exec::value_t params_to_dict(const P &p) {
  std::vector<std::pair<std::string, state_exec::value_t>> entries;
  for_each_param<P>([&](const auto &f) {
    entries.emplace_back(f.name, param_detail::field_value(p.*(f.member)));
  });
  return state_exec::make_dict(std::move(entries));
}

// The struct from an intrinsic's parameters argument over `base`: nil means base as it is; a dict
// sets the fields it names (an unknown key is an error naming the valid ones; a value of the wrong
// kind is an error, and its field keeps base's value). Anything else is an error. Then validated.
template <class P> param_result<P> params_from_value(const state_exec::value_t &v, P base = {}) {
  param_result<P> r;
  r.value = std::move(base);
  if (!v.is_nil()) {
    const state_exec::dict_ptr *d = std::get_if<state_exec::dict_ptr>(&v.v);
    if (!d || !*d) {
      r.errors.push_back("parameters are a dict, not " + param_detail::dsl_kind(v));
      return r;
    }
    for (const auto &kv : **d) {
      bool found = false;
      for_each_param<P>([&](const auto &f) {
        if (found || kv.first != f.name)
          return;
        found = true;
        std::string why;
        if (!param_detail::field_from_value(kv.second, r.value.*(f.member), why))
          r.errors.push_back(std::string(f.name) + ": " + why);
      });
      if (!found) {
        std::string why = "unknown parameter '" + kv.first + "'; valid: ";
        const std::vector<std::string> names = param_names<P>();
        for (std::size_t i = 0; i < names.size(); ++i)
          why += (i ? ", " : "") + names[i];
        r.errors.push_back(why);
      }
    }
  }
  for (std::string &p : params_validate(r.value))
    r.errors.push_back(std::move(p));
  return r;
}

// One table in an app's registry, type-erased.
struct param_table_entry {
  std::string name; // the operation it belongs to ("segment.seeded")
  std::vector<param_field_info> fields;
  std::function<state_exec::value_t()> defaults;                              // as a dict
  std::function<std::vector<std::string>(const state_exec::value_t &)> check; // a dict's problems
};

// The entry for P's table.
template <class P> param_table_entry make_param_table_entry(const std::string &name) {
  param_table_entry e;
  e.name = name;
  e.fields = param_field_infos<P>();
  e.defaults = [] { return params_to_dict(P{}); };
  e.check = [](const state_exec::value_t &v) { return params_from_value<P>(v).errors; };
  return e;
}

// Add (or replace) a table in the app's registry. Safe from any thread.
void register_param_table(cvc::app &app, param_table_entry entry);

template <class P> void register_params(cvc::app &app, const std::string &name) {
  register_param_table(app, make_param_table_entry<P>(name));
}

// The app's table named `name`, or null.
std::shared_ptr<const param_table_entry> find_param_table(cvc::app &app, const std::string &name);

// The names of the app's tables, sorted.
std::vector<std::string> param_table_names(cvc::app &app);

// Bind (params.defaults name) and (params.check name dict) in `env`, reading `app`'s registry
// (which must outlive the environment's evaluations).
void register_param_intrinsics(state_exec::environment_ptr env, cvc::app &app);

} // namespace cvc

#endif // CVC_CORE_PARAM_DSL_H
