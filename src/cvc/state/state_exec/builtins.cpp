#if defined(_WIN32) && !defined(_CRT_RAND_S)
#define _CRT_RAND_S // rand_s: the CRT's CSPRNG (RtlGenRandom), for random-token
#endif
#include "closure_body.h"

#include <algorithm>
#include <cmath>
#include <cvc/state/state_blob_store.h> // sha256_hex
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/data_job.h>
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/generator.h>
#include <cvc/state/state_exec/heap.h>
#include <cvc/state/state_exec/stdlib.h>
#include <cvc/state/state_exec/utf8.h> // SE-1: (length "...") counts codepoints for a string
#include <iostream>
#include <sstream>
#include <stdexcept>
#if defined(_WIN32)
#include <stdlib.h> // rand_s
#elif defined(__APPLE__)
#include <sys/random.h> // getentropy
#else
#include <unistd.h> // getentropy (glibc 2.25+, musl, Emscripten: crypto.getRandomValues)
#endif

namespace cvc::state_exec {

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

namespace {

/// Extract a numeric value as double, supporting int64 and double.
double as_double(const value_t &v) {
  if (auto *i = std::get_if<int64_t>(&v.v))
    return static_cast<double>(*i);
  if (auto *d = std::get_if<double>(&v.v))
    return *d;
  throw dsl_error("TypeError", "expected number, got " + v.type_name());
}

/// True when the value is an integer.
bool is_int(const value_t &v) { return std::holds_alternative<int64_t>(v.v); }

/// True when the value can hold other values, so storing it in a container in place may close a
/// reference cycle (heap.h): the cycle collector then tracks that container.
bool can_close_cycle(const value_t &v) {
  return std::holds_alternative<list_ptr>(v.v) || std::holds_alternative<dict_ptr>(v.v) ||
         std::holds_alternative<closure_ptr>(v.v) || std::holds_alternative<generator_ptr>(v.v) ||
         std::holds_alternative<native_fn>(v.v);
}

/// True when every arg in [first, last) is integral (recurses into lists).
bool all_int(std::span<const value_t> args) {
  for (auto &a : args) {
    if (auto *lp = std::get_if<list_ptr>(&a.v)) {
      for (auto &item : **lp)
        if (!is_int(item))
          return false;
    } else if (!is_int(a)) {
      return false;
    }
  }
  return true;
}

void expect_min(std::span<const value_t> args, size_t n, const char *name) {
  if (args.size() < n)
    throw dsl_error("TypeError", std::string(name) + ": expected at least " + std::to_string(n) +
                                     " argument(s), got " + std::to_string(args.size()));
}

void expect_exact(std::span<const value_t> args, size_t n, const char *name) {
  if (args.size() != n)
    throw dsl_error("TypeError", std::string(name) + ": expected " + std::to_string(n) +
                                     " argument(s), got " + std::to_string(args.size()));
}

const std::vector<value_t> &as_list(const value_t &v, const char *name) {
  if (auto *p = std::get_if<list_ptr>(&v.v))
    return **p;
  throw dsl_error("TypeError", std::string(name) + ": expected list, got " + v.type_name());
}

std::vector<value_t> &as_list_mut(const value_t &v, const char *name) {
  if (auto *p = std::get_if<list_ptr>(&v.v))
    return **p;
  throw dsl_error("TypeError", std::string(name) + ": expected list, got " + v.type_name());
}

using dict_type = std::vector<std::pair<std::string, value_t>>;

dict_type &as_dict_mut(const value_t &v, const char *name) {
  if (auto *p = std::get_if<dict_ptr>(&v.v))
    return **p;
  throw dsl_error("TypeError", std::string(name) + ": expected dict, got " + v.type_name());
}

const dict_type &as_dict(const value_t &v, const char *name) { return as_dict_mut(v, name); }

const std::string &as_string(const value_t &v, const char *name) {
  if (auto *s = std::get_if<std::string>(&v.v))
    return *s;
  throw dsl_error("TypeError", std::string(name) + ": expected string, got " + v.type_name());
}

int64_t as_int(const value_t &v, const char *name) {
  if (auto *i = std::get_if<int64_t>(&v.v))
    return *i;
  throw dsl_error("TypeError", std::string(name) + ": expected integer, got " + v.type_name());
}

/// Find an entry in a dict by key; returns nullptr if not found.
value_t *dict_find(dict_type &d, const std::string &key) {
  for (auto &[k, v] : d)
    if (k == key)
      return &v;
  return nullptr;
}

/// Walk the __class__ / __super__ chain to find a method.
value_t dispatch_method(const value_t &obj, const std::string &method,
                        std::span<const value_t> args, bool super) {
  auto &obj_dict = as_dict_mut(obj, "send");
  // find __class__
  value_t *cls_val = dict_find(obj_dict, "__class__");
  if (!cls_val)
    throw dsl_error("TypeError", "send: object has no __class__");

  auto *cls_dict = &as_dict_mut(*cls_val, "send");

  if (super) {
    // skip to __super__
    value_t *sp = dict_find(*cls_dict, "__super__");
    if (!sp)
      throw dsl_error("TypeError", "send: no super class");
    cls_dict = &as_dict_mut(*sp, "send");
  }

  while (cls_dict) {
    if (value_t *m = dict_find(*cls_dict, method)) {
      if (!std::holds_alternative<native_fn>(m->v) && !std::holds_alternative<closure_ptr>(m->v))
        throw dsl_error("TypeError", "send: method is not callable");
      // Invoke: first arg is self. A closure runs as the program's own frames, and a native
      // method nests no deeper than one level (call_function).
      std::vector<value_t> call_args;
      call_args.reserve(1 + args.size());
      call_args.push_back(obj);
      call_args.insert(call_args.end(), args.begin(), args.end());
      const value_t fn = *m; // the dict may change while a closure runs
      return call_function(fn, call_args, "send");
    }
    // walk __super__
    value_t *sp = dict_find(*cls_dict, "__super__");
    if (!sp)
      break;
    cls_dict = &as_dict_mut(*sp, "send");
  }
  throw dsl_error("KeyError", "send: method '" + method + "' not found");
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// Arithmetic
// ---------------------------------------------------------------------------

value_t builtin_add(std::span<const value_t> args) {
  if (args.empty())
    return value_t{int64_t{0}};
  if (all_int(args)) {
    int64_t r = 0;
    for (auto &a : args) {
      // flatten lists
      if (auto *lp = std::get_if<list_ptr>(&a.v)) {
        for (auto &item : **lp)
          r += std::get<int64_t>(item.v);
      } else {
        r += std::get<int64_t>(a.v);
      }
    }
    return value_t{r};
  }
  double r = 0.0;
  for (auto &a : args) {
    if (auto *lp = std::get_if<list_ptr>(&a.v)) {
      for (auto &item : **lp)
        r += as_double(item);
    } else {
      r += as_double(a);
    }
  }
  return value_t{r};
}

value_t builtin_sub(std::span<const value_t> args) {
  if (args.empty())
    return value_t{int64_t{0}};
  if (args.size() == 1) {
    if (is_int(args[0]))
      return value_t{-std::get<int64_t>(args[0].v)};
    return value_t{-as_double(args[0])};
  }
  if (all_int(args)) {
    int64_t r = std::get<int64_t>(args[0].v);
    for (size_t i = 1; i < args.size(); ++i)
      r -= std::get<int64_t>(args[i].v);
    return value_t{r};
  }
  double r = as_double(args[0]);
  for (size_t i = 1; i < args.size(); ++i)
    r -= as_double(args[i]);
  return value_t{r};
}

value_t builtin_mul(std::span<const value_t> args) {
  if (args.empty())
    return value_t{int64_t{1}};
  if (all_int(args)) {
    int64_t r = 1;
    for (auto &a : args) {
      if (auto *lp = std::get_if<list_ptr>(&a.v)) {
        for (auto &item : **lp)
          r *= std::get<int64_t>(item.v);
      } else {
        r *= std::get<int64_t>(a.v);
      }
    }
    return value_t{r};
  }
  double r = 1.0;
  for (auto &a : args) {
    if (auto *lp = std::get_if<list_ptr>(&a.v)) {
      for (auto &item : **lp)
        r *= as_double(item);
    } else {
      r *= as_double(a);
    }
  }
  return value_t{r};
}

value_t builtin_div(std::span<const value_t> args) {
  if (args.empty())
    return value_t{int64_t{1}};
  if (args.size() == 1)
    return value_t{1.0 / as_double(args[0])};
  // Division always produces double (like Python3)
  double r = as_double(args[0]);
  for (size_t i = 1; i < args.size(); ++i) {
    double d = as_double(args[i]);
    if (d == 0.0)
      throw dsl_error("ArithmeticError", "/: division by zero");
    r /= d;
  }
  return value_t{r};
}

value_t builtin_mod(std::span<const value_t> args) {
  expect_exact(args, 2, "%");
  if (all_int(args)) {
    int64_t b = std::get<int64_t>(args[1].v);
    if (b == 0)
      throw dsl_error("ArithmeticError", "%: division by zero");
    return value_t{std::get<int64_t>(args[0].v) % b};
  }
  double b = as_double(args[1]);
  if (b == 0.0)
    throw dsl_error("ArithmeticError", "%: division by zero");
  return value_t{std::fmod(as_double(args[0]), b)};
}

// ---------------------------------------------------------------------------
// Comparison
// ---------------------------------------------------------------------------

value_t builtin_lt(std::span<const value_t> args) {
  expect_exact(args, 2, "<");
  return value_t{as_double(args[0]) < as_double(args[1])};
}

value_t builtin_gt(std::span<const value_t> args) {
  expect_exact(args, 2, ">");
  return value_t{as_double(args[0]) > as_double(args[1])};
}

value_t builtin_le(std::span<const value_t> args) {
  expect_exact(args, 2, "<=");
  return value_t{as_double(args[0]) <= as_double(args[1])};
}

value_t builtin_ge(std::span<const value_t> args) {
  expect_exact(args, 2, ">=");
  return value_t{as_double(args[0]) >= as_double(args[1])};
}

value_t builtin_eq(std::span<const value_t> args) {
  expect_exact(args, 2, "=");
  if (!is_compound(args[0]) || !is_compound(args[1]))
    return value_t{values_equal(args[0], args[1])};
  return run_data_job(make_equal_job(args[0], args[1]));
}

value_t builtin_ne(std::span<const value_t> args) {
  expect_exact(args, 2, "!=");
  if (!is_compound(args[0]) || !is_compound(args[1]))
    return value_t{!values_equal(args[0], args[1])};
  return run_data_job(make_equal_job(args[0], args[1]),
                      [](value_t equal) { return value_t{!std::get<bool>(equal.v)}; });
}

// ---------------------------------------------------------------------------
// String
// ---------------------------------------------------------------------------

value_t builtin_str_concat(std::span<const value_t> args) {
  if (std::any_of(args.begin(), args.end(), is_compound))
    return run_data_job(make_concat_job({args.begin(), args.end()}, "", "str-concat"));
  // The same string may be passed many times, so the result can outgrow every argument: check
  // each part against the allocation limit before appending it.
  std::string result;
  const auto add = [&result](const std::string &part) {
    eval_alloc_check(result.size() > SIZE_MAX - part.size() ? SIZE_MAX
                                                            : result.size() + part.size(),
                     "str-concat");
    result += part;
  };
  for (auto &a : args) {
    if (auto *s = std::get_if<std::string>(&a.v))
      add(*s); // raw string, no quoting
    else
      add(to_string(a));
  }
  return value_t{std::move(result)};
}

value_t builtin_str(std::span<const value_t> args) {
  expect_exact(args, 1, "str");
  if (is_compound(args[0]))
    return run_data_job(make_print_job(args[0]));
  return value_t{to_string(args[0])};
}

std::string percent_encode(const std::string &s) {
  static const char *const kHex = "0123456789ABCDEF";
  std::string out;
  out.reserve(s.size() * 3);
  for (unsigned char c : s) {
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' ||
        c == '_' || c == '.' || c == '~') {
      out.push_back(static_cast<char>(c));
    } else {
      out.push_back('%');
      out.push_back(kHex[c >> 4]);
      out.push_back(kHex[c & 0x0F]);
    }
  }
  return out;
}

std::string percent_decode(const std::string &s) {
  const auto nibble = [](char c) -> int {
    if (c >= '0' && c <= '9')
      return c - '0';
    if (c >= 'a' && c <= 'f')
      return c - 'a' + 10;
    if (c >= 'A' && c <= 'F')
      return c - 'A' + 10;
    return -1;
  };
  std::string out;
  out.reserve(s.size());
  for (std::size_t i = 0; i < s.size(); ++i) {
    const char c = s[i];
    if (c == '%' && i + 2 < s.size()) {
      const int hi = nibble(s[i + 1]);
      const int lo = nibble(s[i + 2]);
      if (hi >= 0 && lo >= 0) {
        out.push_back(static_cast<char>((hi << 4) | lo));
        i += 2;
        continue;
      }
    }
    out.push_back(c == '+' ? ' ' : c); // '+' is space in form-encoded strings
  }
  return out;
}

value_t builtin_url_encode(std::span<const value_t> args) {
  expect_exact(args, 1, "url-encode");
  const std::string *s = std::get_if<std::string>(&args[0].v);
  return value_t{percent_encode(s ? *s : to_string(args[0]))};
}

value_t builtin_url_decode(std::span<const value_t> args) {
  expect_exact(args, 1, "url-decode");
  const std::string *s = std::get_if<std::string>(&args[0].v);
  return value_t{percent_decode(s ? *s : to_string(args[0]))};
}

value_t builtin_int(std::span<const value_t> args) {
  expect_exact(args, 1, "int");
  auto &a = args[0];
  if (auto *i = std::get_if<int64_t>(&a.v))
    return a;
  if (auto *d = std::get_if<double>(&a.v))
    return value_t{static_cast<int64_t>(*d)};
  if (auto *b = std::get_if<bool>(&a.v))
    return value_t{int64_t{*b ? 1 : 0}};
  if (auto *s = std::get_if<std::string>(&a.v)) {
    try {
      return value_t{static_cast<int64_t>(std::stoll(*s))};
    } catch (...) {
      throw dsl_error("ValueError", "int: cannot convert string \"" + *s + "\" to integer");
    }
  }
  throw dsl_error("TypeError", "int: cannot convert " + a.type_name() + " to integer");
}

value_t builtin_float(std::span<const value_t> args) {
  expect_exact(args, 1, "float");
  auto &a = args[0];
  if (auto *d = std::get_if<double>(&a.v))
    return a;
  if (auto *i = std::get_if<int64_t>(&a.v))
    return value_t{static_cast<double>(*i)};
  if (auto *b = std::get_if<bool>(&a.v))
    return value_t{*b ? 1.0 : 0.0};
  if (auto *s = std::get_if<std::string>(&a.v)) {
    try {
      return value_t{std::stod(*s)};
    } catch (...) {
      throw dsl_error("ValueError", "float: cannot convert string \"" + *s + "\" to float");
    }
  }
  throw dsl_error("TypeError", "float: cannot convert " + a.type_name() + " to float");
}

value_t builtin_is_int(std::span<const value_t> args) {
  expect_exact(args, 1, "is-int");
  return value_t{std::holds_alternative<int64_t>(args[0].v)};
}

value_t builtin_is_float(std::span<const value_t> args) {
  expect_exact(args, 1, "is-float");
  return value_t{std::holds_alternative<double>(args[0].v)};
}

value_t builtin_is_string(std::span<const value_t> args) {
  expect_exact(args, 1, "is-string");
  return value_t{std::holds_alternative<std::string>(args[0].v)};
}

value_t builtin_is_bytes(std::span<const value_t> args) { // SE-3
  expect_exact(args, 1, "is-bytes");
  return value_t{std::holds_alternative<bytes_value>(args[0].v)};
}

// ---------------------------------------------------------------------------
// List
// ---------------------------------------------------------------------------

value_t builtin_list(std::span<const value_t> args) {
  return make_list(std::vector<value_t>(args.begin(), args.end()));
}

value_t builtin_car(std::span<const value_t> args) {
  expect_exact(args, 1, "car");
  auto &lst = as_list(args[0], "car");
  if (lst.empty())
    throw dsl_error("IndexError", "car: empty list");
  return lst.front();
}

value_t builtin_cdr(std::span<const value_t> args) {
  expect_exact(args, 1, "cdr");
  auto &lst = as_list(args[0], "cdr");
  if (lst.empty())
    return make_list();
  return make_list(std::vector<value_t>(lst.begin() + 1, lst.end()));
}

value_t builtin_cons(std::span<const value_t> args) {
  expect_exact(args, 2, "cons");
  auto &lst = as_list(args[1], "cons");
  eval_alloc_note(alloc_bytes(lst.size() + 1, sizeof(value_t))); // a copy: heap accounting
  std::vector<value_t> result;
  result.reserve(1 + lst.size());
  result.push_back(args[0]);
  result.insert(result.end(), lst.begin(), lst.end());
  return make_list(std::move(result));
}

value_t builtin_nth(std::span<const value_t> args) {
  expect_exact(args, 2, "nth");
  auto &lst = as_list(args[0], "nth");
  int64_t idx = as_int(args[1], "nth");
  if (idx < 0 || static_cast<size_t>(idx) >= lst.size())
    throw dsl_error("IndexError", "nth: index out of range");
  return lst[static_cast<size_t>(idx)];
}

value_t builtin_set_nth(std::span<const value_t> args) {
  expect_exact(args, 3, "set-nth");
  auto &lst = as_list_mut(args[0], "set-nth");
  int64_t idx = as_int(args[1], "set-nth");
  if (idx < 0 || static_cast<size_t>(idx) >= lst.size())
    throw dsl_error("IndexError", "set-nth: index out of range");
  lst[static_cast<size_t>(idx)] = args[2];
  if (can_close_cycle(args[2]))
    track_mutated(std::get<list_ptr>(args[0].v));
  return args[2];
}

value_t builtin_length(std::span<const value_t> args) {
  expect_exact(args, 1, "length");
  if (auto *lp = std::get_if<list_ptr>(&args[0].v))
    return value_t{static_cast<int64_t>((*lp)->size())};
  if (auto *s = std::get_if<std::string>(&args[0].v))
    return value_t{static_cast<int64_t>(utf8::count(*s))}; // SE-1 (A6): codepoints, not bytes
  if (auto *dp = std::get_if<dict_ptr>(&args[0].v))
    return value_t{static_cast<int64_t>((*dp)->size())};
  throw dsl_error("TypeError", "length: expected list, string, or dict");
}

value_t builtin_append(std::span<const value_t> args) {
  expect_exact(args, 2, "append");
  auto &lst = as_list_mut(args[0], "append");
  lst.push_back(args[1]);
  if (can_close_cycle(args[1]))
    track_mutated(std::get<list_ptr>(args[0].v));
  return args[0];
}

value_t builtin_slice(std::span<const value_t> args) {
  if (args.size() < 2 || args.size() > 3)
    throw dsl_error("TypeError", "slice: expected 2 or 3 arguments");
  auto &lst = as_list(args[0], "slice");
  int64_t start = as_int(args[1], "slice");
  int64_t end = (args.size() == 3) ? as_int(args[2], "slice") : static_cast<int64_t>(lst.size());
  if (start < 0)
    start = 0;
  if (end < 0)
    end = 0;
  if (static_cast<size_t>(start) > lst.size())
    start = static_cast<int64_t>(lst.size());
  if (static_cast<size_t>(end) > lst.size())
    end = static_cast<int64_t>(lst.size());
  if (start >= end)
    return make_list();
  eval_alloc_note(alloc_bytes(static_cast<size_t>(end - start), sizeof(value_t))); // a copy
  return make_list(std::vector<value_t>(lst.begin() + start, lst.begin() + end));
}

value_t builtin_del_nth(std::span<const value_t> args) {
  expect_exact(args, 2, "del-nth");
  auto &lst = as_list_mut(args[0], "del-nth");
  int64_t idx = as_int(args[1], "del-nth");
  if (idx < 0 || static_cast<size_t>(idx) >= lst.size())
    throw dsl_error("IndexError", "del-nth: index out of range");
  value_t removed = lst[static_cast<size_t>(idx)];
  lst.erase(lst.begin() + idx);
  return removed;
}

// ---------------------------------------------------------------------------
// Dict
// ---------------------------------------------------------------------------

value_t builtin_dict(std::span<const value_t> args) {
  if (args.size() % 2 != 0)
    throw dsl_error("TypeError", "dict: expected even number of arguments");
  std::vector<std::pair<std::string, value_t>> entries;
  entries.reserve(args.size() / 2);
  for (size_t i = 0; i < args.size(); i += 2) {
    std::string key;
    if (auto *s = std::get_if<std::string>(&args[i].v))
      key = *s;
    else
      key = to_string(args[i]);
    entries.emplace_back(std::move(key), args[i + 1]);
  }
  return make_dict(std::move(entries));
}

value_t builtin_get_attr(std::span<const value_t> args) {
  expect_exact(args, 2, "get-attr");
  auto &d = as_dict(args[0], "get-attr");
  const std::string &key = as_string(args[1], "get-attr");
  for (auto &[k, v] : d)
    if (k == key)
      return v;
  throw dsl_error("KeyError", "get-attr: key '" + key + "' not found");
}

value_t builtin_set_attr(std::span<const value_t> args) {
  expect_exact(args, 3, "set-attr");
  auto &d = as_dict_mut(args[0], "set-attr");
  const std::string &key = as_string(args[1], "set-attr");
  if (can_close_cycle(args[2]))
    track_mutated(std::get<dict_ptr>(args[0].v));
  for (auto &[k, v] : d) {
    if (k == key) {
      v = args[2];
      return args[2];
    }
  }
  d.emplace_back(key, args[2]);
  return args[2];
}

value_t builtin_del_attr(std::span<const value_t> args) {
  expect_exact(args, 2, "del-attr");
  auto &d = as_dict_mut(args[0], "del-attr");
  const std::string &key = as_string(args[1], "del-attr");
  for (auto it = d.begin(); it != d.end(); ++it) {
    if (it->first == key) {
      value_t removed = it->second;
      d.erase(it);
      return removed;
    }
  }
  throw dsl_error("KeyError", "del-attr: key '" + key + "' not found");
}

// ---------------------------------------------------------------------------
// OOP
// ---------------------------------------------------------------------------

value_t builtin_send(std::span<const value_t> args) {
  expect_min(args, 2, "send");
  const std::string &method = as_string(args[1], "send");
  auto method_args = args.subspan(2);
  return dispatch_method(args[0], method, method_args, /*super=*/false);
}

// ---------------------------------------------------------------------------
// Higher-order
// ---------------------------------------------------------------------------

value_t builtin_apply(std::span<const value_t> args) {
  expect_exact(args, 2, "apply");
  auto &arg_list = as_list(args[1], "apply");
  // A closure runs as the program's own frames; a native function is called here, unless this
  // apply was itself called by a native function, when the evaluator calls it (call_function).
  return call_function(args[0], arg_list, "apply");
}

// ---------------------------------------------------------------------------
// Type predicates
// ---------------------------------------------------------------------------

value_t builtin_is_null(std::span<const value_t> args) {
  expect_exact(args, 1, "is-null");
  return value_t{args[0].is_nil()};
}

value_t builtin_is_list(std::span<const value_t> args) {
  expect_exact(args, 1, "is-list");
  return value_t{std::holds_alternative<list_ptr>(args[0].v)};
}

value_t builtin_type_of(std::span<const value_t> args) {
  expect_exact(args, 1, "type-of");
  return value_t{args[0].type_name()};
}

value_t builtin_isinstance(std::span<const value_t> args) {
  expect_exact(args, 2, "isinstance");
  // The class is its constructor (what `Name` is bound to), or its class dict.
  dict_ptr cls = class_of_constructor(args[1]);
  if (!cls)
    if (const auto *d = std::get_if<dict_ptr>(&args[1].v); d && field(args[1], "__name__"))
      cls = *d;
  if (!cls)
    throw dsl_error("TypeError", "isinstance: not a class: " + args[1].type_name());
  return value_t{is_instance(args[0], cls)};
}

// ---------------------------------------------------------------------------
// Exceptions
// ---------------------------------------------------------------------------

value_t builtin_error(std::span<const value_t> args) {
  if (args.empty() || args.size() > 2)
    throw dsl_error("TypeError", "error: expected a message and optional data, got " +
                                     std::to_string(args.size()) + " args");
  const std::string message = std::holds_alternative<std::string>(args[0].v)
                                  ? std::get<std::string>(args[0].v)
                                  : to_string(args[0]);
  throw dsl_exception(make_exception(exception_class("RuntimeError"), message,
                                     args.size() > 1 ? args[1] : nil_value, "dsl"));
}

// (assert cond [message [data]]): nil when cond is truthy, else raises an AssertionError. The
// message is "assertion failed", or "assertion failed: <message>"; the exception's traceback ends
// at the (assert ...) form, so an uncaught one reports the condition that failed.
value_t builtin_assert(std::span<const value_t> args) {
  if (args.empty() || args.size() > 3)
    throw dsl_error("TypeError",
                    "assert: expected a condition, an optional message and data, got " +
                        std::to_string(args.size()) + " args");
  if (args[0].is_truthy())
    return nil_value;
  std::string message = "assertion failed";
  if (args.size() > 1)
    message +=
        ": " + (std::holds_alternative<std::string>(args[1].v) ? std::get<std::string>(args[1].v)
                                                               : to_string(args[1]));
  throw dsl_exception(make_exception(exception_class("AssertionError"), message,
                                     args.size() > 2 ? args[2] : nil_value, "dsl"));
}

// ---------------------------------------------------------------------------
// Hashing and secure randomness
// ---------------------------------------------------------------------------

namespace {

// The octets a hash reads: a string's UTF-8 bytes, or a bytes value's.
const std::string &hash_input(std::span<const value_t> args, const char *name) {
  if (args.size() != 1)
    throw dsl_error("TypeError", std::string(name) + ": expected 1 argument, got " +
                                     std::to_string(args.size()));
  if (const auto *s = std::get_if<std::string>(&args[0].v))
    return *s;
  if (const auto *b = std::get_if<bytes_value>(&args[0].v))
    return b->data;
  throw dsl_error("TypeError",
                  std::string(name) + ": expected a string or bytes, got " + args[0].type_name());
}

std::string sha256_of(const std::string &data) {
  return cvc::sha256_hex(reinterpret_cast<const unsigned char *>(data.data()), data.size());
}

// `n` bytes from the operating system's CSPRNG. Throws when it cannot supply them.
std::string os_random_bytes(std::size_t n) {
  std::string out(n, '\0');
#if defined(_WIN32)
  for (std::size_t i = 0; i < n; i += 4) {
    unsigned int word = 0;
    if (rand_s(&word) != 0)
      throw std::runtime_error("random-token: the system's random source failed");
    for (std::size_t k = 0; k < 4 && i + k < n; ++k)
      out[i + k] = static_cast<char>((word >> (8 * k)) & 0xff);
  }
#else
  for (std::size_t i = 0; i < n; i += 256) { // getentropy hands out at most 256 bytes a call
    const std::size_t chunk = std::min<std::size_t>(256, n - i);
    if (getentropy(out.data() + i, chunk) != 0)
      throw std::runtime_error("random-token: the system's random source failed");
  }
#endif
  return out;
}

} // namespace

value_t builtin_sha256_hex(std::span<const value_t> args) {
  return value_t(sha256_of(hash_input(args, "sha256-hex")));
}

value_t builtin_sha256(std::span<const value_t> args) {
  const std::string hex = sha256_of(hash_input(args, "sha256"));
  std::string raw(hex.size() / 2, '\0');
  for (std::size_t i = 0; i < raw.size(); ++i)
    raw[i] = static_cast<char>(std::stoi(hex.substr(2 * i, 2), nullptr, 16));
  return make_bytes(std::move(raw));
}

value_t builtin_random_token(std::span<const value_t> args) {
  if (args.empty() || args.size() > 2)
    throw dsl_error("TypeError", "random-token: expected a byte count and an optional encoding, "
                                 "got " +
                                     std::to_string(args.size()) + " args");
  const auto *n = std::get_if<int64_t>(&args[0].v);
  if (!n)
    throw dsl_error("TypeError",
                    "random-token: the byte count is an integer, not " + args[0].type_name());
  if (*n < 1 || *n > kRandomTokenMaxBytes)
    throw dsl_error("ValueError", "random-token: the byte count is 1.." +
                                      std::to_string(kRandomTokenMaxBytes) + ", not " +
                                      std::to_string(*n));
  std::string encoding = "base64url";
  if (args.size() > 1) {
    const auto *e = std::get_if<std::string>(&args[1].v);
    if (!e)
      throw dsl_error("TypeError",
                      "random-token: the encoding is a string, not " + args[1].type_name());
    encoding = *e;
  }
  if (encoding != "base64url" && encoding != "hex")
    throw dsl_error("ValueError", "random-token: the encoding is \"base64url\" or \"hex\", not \"" +
                                      encoding + "\"");
  const std::string raw = os_random_bytes(static_cast<std::size_t>(*n));
  std::string out;
  if (encoding == "hex") {
    static const char kHex[] = "0123456789abcdef";
    for (unsigned char c : raw) {
      out += kHex[c >> 4];
      out += kHex[c & 0xf];
    }
    return value_t(std::move(out));
  }
  // RFC 4648 §5 base64url, unpadded (as a PKCE verifier is written).
  static const char kB64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";
  std::size_t i = 0;
  for (; i + 3 <= raw.size(); i += 3) {
    const unsigned v = (static_cast<unsigned char>(raw[i]) << 16) |
                       (static_cast<unsigned char>(raw[i + 1]) << 8) |
                       static_cast<unsigned char>(raw[i + 2]);
    for (int k = 18; k >= 0; k -= 6)
      out += kB64[(v >> k) & 0x3f];
  }
  if (const std::size_t rest = raw.size() - i; rest > 0) {
    unsigned v = static_cast<unsigned char>(raw[i]) << 16;
    if (rest == 2)
      v |= static_cast<unsigned char>(raw[i + 1]) << 8;
    out += kB64[(v >> 18) & 0x3f];
    out += kB64[(v >> 12) & 0x3f];
    if (rest == 2)
      out += kB64[(v >> 6) & 0x3f];
  }
  return value_t(std::move(out));
}

// ---------------------------------------------------------------------------
// I/O
// ---------------------------------------------------------------------------

value_t builtin_print(std::span<const value_t> args) {
  if (std::any_of(args.begin(), args.end(), is_compound)) {
    // The line goes out once all of it is printed. A string prints quoted here, as to_string
    // prints it, so it goes in already printed.
    std::vector<value_t> parts;
    parts.reserve(args.size());
    for (auto &a : args)
      parts.push_back(std::holds_alternative<std::string>(a.v) ? value_t{to_string(a)} : a);
    return run_data_job(make_concat_job(std::move(parts), " ", "print"), [](value_t line) {
      std::cout << std::get<std::string>(line.v) << '\n';
      return nil_value;
    });
  }
  for (size_t i = 0; i < args.size(); ++i) {
    if (i > 0)
      std::cout << ' ';
    std::cout << to_string(args[i]);
  }
  std::cout << '\n';
  return nil_value;
}

// ---------------------------------------------------------------------------
// Generators
// ---------------------------------------------------------------------------

value_t builtin_generator(std::span<const value_t> args) {
  expect_exact(args, 1, "generator");
  auto *cp = std::get_if<closure_ptr>(&args[0].v);
  if (!cp || !*cp)
    throw dsl_error("TypeError", "generator: expected closure");
  auto &cls = **cp;

  auto gen = std::make_shared<generator>(cls.env_snapshot ? cls.env_snapshot
                                                          : std::make_shared<environment>());

  // Set up evaluator state to run the closure body
  auto env = environment::extend(cls.env_snapshot);
  // A body of its own: (root) in the generator returns it, and a program may change it.
  gen->state = gen->evaluator.create_state(
      cls.body.size() == 1 ? cls.body[0] : detail::begin_of(cls.body), env);
  gen->state.in_generator = true; // with-lock refuses here (docs/STATE_EXEC_LOCKS.md 5.3)
  gen->state.stats.start();
  track_captured_env(env); // bound back where the closure can reach, it is a cycle

  return value_t{std::move(gen)};
}

value_t builtin_next(std::span<const value_t> args) {
  expect_exact(args, 1, "next");
  auto *gp = std::get_if<generator_ptr>(&args[0].v);
  if (!gp || !*gp)
    throw dsl_error("TypeError", "next: expected generator");
  if ((*gp)->native_step) { // range and friends: one cheap value
    auto result = generator_next(**gp);
    return result ? *result : nil_value;
  }
  // A DSL generator's body runs as a job, a slice per step when the process slices.
  return run_data_job(make_generator_job(*gp), [](value_t got) {
    const auto &l = *std::get<list_ptr>(got.v);
    return l.empty() ? nil_value : l.front();
  });
}

value_t builtin_is_generator(std::span<const value_t> args) {
  expect_exact(args, 1, "generator?");
  return value_t{std::holds_alternative<generator_ptr>(args[0].v)};
}

value_t builtin_gen_done(std::span<const value_t> args) {
  expect_exact(args, 1, "gen-done?");
  auto *gp = std::get_if<generator_ptr>(&args[0].v);
  if (!gp || !*gp)
    throw dsl_error("TypeError", "gen-done?: expected generator");
  return value_t{(*gp)->exhausted};
}

value_t builtin_range(std::span<const value_t> args) {
  if (args.size() < 1 || args.size() > 3)
    throw dsl_error("TypeError", "range: expected 1-3 arguments (end) or (start end [step])");

  int64_t start = 0, end = 0, step = 1;
  if (args.size() == 1) {
    if (auto *i = std::get_if<int64_t>(&args[0].v))
      end = *i;
    else
      throw dsl_error("TypeError", "range: expected integer argument");
  } else {
    if (auto *i = std::get_if<int64_t>(&args[0].v))
      start = *i;
    else
      throw dsl_error("TypeError", "range: expected integer start");
    if (auto *i = std::get_if<int64_t>(&args[1].v))
      end = *i;
    else
      throw dsl_error("TypeError", "range: expected integer end");
    if (args.size() == 3) {
      if (auto *i = std::get_if<int64_t>(&args[2].v))
        step = *i;
      else
        throw dsl_error("TypeError", "range: expected integer step");
      if (step == 0)
        throw dsl_error("ValueError", "range: step must be non-zero");
    }
  }

  auto gen = std::make_shared<generator>(std::make_shared<environment>());
  gen->native_step = [current = start, end, step]() mutable -> std::optional<value_t> {
    if ((step > 0 && current >= end) || (step < 0 && current <= end))
      return std::nullopt;
    auto val = value_t(current);
    current += step;
    return val;
  };

  return value_t{std::move(gen)};
}

value_t builtin_collect(std::span<const value_t> args) {
  expect_exact(args, 1, "collect");
  auto *gp = std::get_if<generator_ptr>(&args[0].v);
  if (!gp || !*gp)
    throw dsl_error("TypeError", "collect: expected generator");
  // Draining a generator is a job: a value (or a step of a DSL generator's body) is a node, so
  // a huge or unbounded one (e.g. (collect (range 0 1e11))) spreads over the process's steps,
  // within its budget, and the allocation limit checks every value kept.
  return run_data_job(make_generator_job(*gp, true));
}

// ---------------------------------------------------------------------------
// Logic
// ---------------------------------------------------------------------------

value_t builtin_not(std::span<const value_t> args) {
  expect_exact(args, 1, "not");
  return value_t{!args[0].is_truthy()};
}

value_t builtin_and(std::span<const value_t> args) {
  for (auto &a : args)
    if (!a.is_truthy())
      return false_value;
  return args.empty() ? true_value : args.back();
}

value_t builtin_or(std::span<const value_t> args) {
  for (auto &a : args)
    if (a.is_truthy())
      return a;
  return false_value;
}

// ---------------------------------------------------------------------------
// Registry
// ---------------------------------------------------------------------------

void builtins::register_fn(environment_ptr env, const std::string &name, native_fn fn) {
  env->set(name, value_t{std::move(fn)});
}

environment_ptr builtins::make_default_environment() {
  auto env = std::make_shared<environment>();
  // The stdlib modules (string.*, bytes.*, math.*, collections.*) are reached through the
  // shared, frozen stdlib scope rather than copied into every environment.
  env->outer = stdlib_registry::shared_scope();

  // Arithmetic
  register_fn(env, "+", builtin_add);
  register_fn(env, "-", builtin_sub);
  register_fn(env, "*", builtin_mul);
  register_fn(env, "/", builtin_div);
  register_fn(env, "%", builtin_mod);

  // Comparison
  register_fn(env, "<", builtin_lt);
  register_fn(env, ">", builtin_gt);
  register_fn(env, "<=", builtin_le);
  register_fn(env, ">=", builtin_ge);
  register_fn(env, "=", builtin_eq);
  register_fn(env, "!=", builtin_ne);

  // String / conversion
  register_fn(env, "str-concat", builtin_str_concat);
  register_fn(env, "str", builtin_str);
  register_fn(env, "url-encode", builtin_url_encode);
  register_fn(env, "url-decode", builtin_url_decode);
  register_fn(env, "json-parse", builtin_json_parse);
  register_fn(env, "json-encode", builtin_json_encode);
  register_fn(env, "int", builtin_int);
  register_fn(env, "float", builtin_float);
  register_fn(env, "is-int", builtin_is_int);
  register_fn(env, "is-float", builtin_is_float);
  register_fn(env, "is-string", builtin_is_string);
  register_fn(env, "is-bytes", builtin_is_bytes);

  // List
  register_fn(env, "list", builtin_list);
  register_fn(env, "car", builtin_car);
  register_fn(env, "cdr", builtin_cdr);
  register_fn(env, "cons", builtin_cons);
  register_fn(env, "nth", builtin_nth);
  register_fn(env, "set-nth", builtin_set_nth);
  register_fn(env, "length", builtin_length);
  register_fn(env, "append", builtin_append);
  register_fn(env, "slice", builtin_slice);
  register_fn(env, "del-nth", builtin_del_nth);

  // Dict
  register_fn(env, "dict", builtin_dict);
  register_fn(env, "get-attr", builtin_get_attr);
  register_fn(env, "set-attr", builtin_set_attr);
  register_fn(env, "del-attr", builtin_del_attr);

  // OOP
  register_fn(env, "send", builtin_send);

  // Higher-order
  register_fn(env, "apply", builtin_apply);

  // Type predicates
  register_fn(env, "is-null", builtin_is_null);
  register_fn(env, "is-list", builtin_is_list);
  register_fn(env, "type-of", builtin_type_of);
  register_fn(env, "isinstance", builtin_isinstance);

  // Exceptions: the built-in classes, and (error message [data])
  register_exception_classes(env);
  register_fn(env, "error", builtin_error);
  register_fn(env, "assert", builtin_assert);

  // Hashing and secure randomness
  register_fn(env, "sha256", builtin_sha256);
  register_fn(env, "sha256-hex", builtin_sha256_hex);
  register_fn(env, "random-token", builtin_random_token);

  // Generators
  register_fn(env, "generator", builtin_generator);
  register_fn(env, "next", builtin_next);
  register_fn(env, "generator?", builtin_is_generator);
  register_fn(env, "gen-done?", builtin_gen_done);
  register_fn(env, "range", builtin_range);
  register_fn(env, "collect", builtin_collect);

  // I/O
  register_fn(env, "print", builtin_print);

  // Logic
  register_fn(env, "not", builtin_not);
  register_fn(env, "and", builtin_and);
  register_fn(env, "or", builtin_or);

  return env;
}

} // namespace cvc::state_exec
