/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::param_table: a params struct declared once, as a table of its fields, from which generic
// code gives everything else (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.5):
//
//   struct blur_params {
//     double sigma = 1.0;
//     int passes = 2;
//     edge_mode edges = edge_mode::clamp; // BOOST_DESCRIBE_ENUM(edge_mode, clamp, wrap, zero)
//   };
//   template <> struct cvc::param_table<blur_params> : cvc::param_fields<blur_params> {
//     static constexpr auto fields = std::tuple{
//         field("sigma", &blur_params::sigma, unit("voxel"), range(0.0, 64.0),
//               doc("the Gaussian's standard deviation")),
//         field("passes", &blur_params::passes, range(1, 16), doc("how many times it runs")),
//         field("edges", &blur_params::edges, doc("what lies past the volume's edge")),
//     };
//     // Optional: checks across fields, appended to validate()'s list.
//     static void check(const blur_params &p, std::vector<std::string> &problems);
//   };
//
// From that table:
//   - params_validate(p): a list of what is wrong (out of range, not a number, no such choice, and
//     the table's own check), empty when nothing is;
//   - params_to_state(p, node) / params_from_state(node): one key per field under the node
//     (<node>.sigma), missing keys seeded with their defaults, values that do not parse reported
//     rather than replaced by the default, and the properties written on each key: its comment
//     and "doc", "unit", "min", "max", "choices", "default", "type" and "access" = "build"
//     (cvc/state/state_key_meta.h's convention);
//   - param_field_infos<P>() and params_markdown<P>(): the table as data, and as the Markdown
//     table a port's docs include;
//   - cvc/core/param_cli.h: the CLI flags and their help (boost::program_options);
//   - cvc/core/param_dsl.h: the DSL dict an intrinsic takes, (params.defaults name) and
//     (params.check name dict);
//   - pycvc's keywords (bindings/pycvc/pycvc_params.i).
//
// The fields. A field's type is bool, an integer type (not a character type), float, double,
// std::string, an enum described with BOOST_DESCRIBE_ENUM (its choices are its enumerators'
// names), or a std::vector of any of those. Its name is the state key under the node (dots make
// nested keys: "smooth.iterations"); the CLI flag is the name with '.' and '_' as '-'
// (--smooth-iterations); the Python keyword and the DSL dict key are the name with '.' as '_' and
// the name itself. A field's name, with dots as underscores, should be its member's name, which
// is what pycvc's keywords set.
//
// Values as text (state keys, CLI flags): numbers in their shortest round-trip form (so a struct
// read back from the tree is the same bit for bit), bools as 1 / 0 (true/false, yes/no and on/off
// are read too), enums by name, lists as JSON arrays ([1, 2.5] or ["a", "b"]; a DSL-printed list
// (1 2.5) and bare 1,2.5 or 1 2.5 are read too). Everything here is header-only and holds no
// state of its own: the table is constexpr data.

#ifndef CVC_CORE_PARAM_TABLE_H
#define CVC_CORE_PARAM_TABLE_H

#include <boost/describe/enum.hpp>
#include <boost/describe/enumerators.hpp>
#include <boost/mp11/algorithm.hpp>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cvc/state/state.h>
#include <exception>
#include <limits>
#include <locale>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

namespace cvc {

// Declared for each params struct, as a specialization deriving from param_fields<P> (which
// provides field(), unit(), doc() and range()); see the top of this file.
template <class P> struct param_table;

// A params struct with a table.
template <class P>
concept has_param_table = requires { param_table<P>::fields; };

// One field of a table: built with param_fields<P>::field(), read by the generic code below.
template <class P, class M> struct param_field {
  using struct_type = P;
  using value_type = M;
  const char *name = "";
  M P::*member = nullptr;
  const char *unit = nullptr; // nullptr: none
  const char *doc = nullptr;
  double lo = std::numeric_limits<double>::quiet_NaN(); // NaN: no bound
  double hi = std::numeric_limits<double>::quiet_NaN();
};

namespace param_detail {

struct unit_opt {
  const char *text;
};
struct doc_opt {
  const char *text;
};
struct range_opt {
  double lo, hi;
};

template <class P, class M> constexpr void apply(param_field<P, M> &f, unit_opt o) {
  f.unit = o.text;
}
template <class P, class M> constexpr void apply(param_field<P, M> &f, doc_opt o) {
  f.doc = o.text;
}
template <class P, class M> constexpr void apply(param_field<P, M> &f, range_opt o) {
  f.lo = o.lo;
  f.hi = o.hi;
}

template <class T> struct is_vector : std::false_type {};
template <class T, class A> struct is_vector<std::vector<T, A>> : std::true_type {};

template <class T>
inline constexpr bool is_char_type =
    std::is_same_v<T, char> || std::is_same_v<T, signed char> || std::is_same_v<T, unsigned char> ||
    std::is_same_v<T, wchar_t> || std::is_same_v<T, char8_t> || std::is_same_v<T, char16_t> ||
    std::is_same_v<T, char32_t>;

template <class T>
inline constexpr bool is_integer =
    std::is_integral_v<T> && !std::is_same_v<T, bool> && !is_char_type<T>;

template <class T>
inline constexpr bool is_described_enum =
    std::is_enum_v<T> && boost::describe::has_describe_enumerators<T>::value;

template <class T>
inline constexpr bool is_scalar_param =
    std::is_same_v<T, bool> || is_integer<T> || std::is_floating_point_v<T> ||
    std::is_same_v<T, std::string> || is_described_enum<T>;

template <class T> struct param_type_ok : std::bool_constant<is_scalar_param<T>> {};
template <class T, class A>
struct param_type_ok<std::vector<T, A>> : std::bool_constant<is_scalar_param<T>> {};

// The kind a field's "type" property names.
template <class T> constexpr const char *kind_of() {
  if constexpr (is_vector<T>::value)
    return "list";
  else if constexpr (std::is_same_v<T, bool>)
    return "bool";
  else if constexpr (is_integer<T>)
    return "int";
  else if constexpr (std::is_floating_point_v<T>)
    return "number";
  else if constexpr (std::is_same_v<T, std::string>)
    return "string";
  else
    return "enum";
}

// ---- numbers as text ----

inline std::string format_double(double d) {
  if (std::isnan(d))
    return "nan";
  if (std::isinf(d))
    return d > 0 ? "inf" : "-inf";
#if defined(__cpp_lib_to_chars) && __cpp_lib_to_chars >= 201611L
  char buf[64];
  const std::to_chars_result r = std::to_chars(buf, buf + sizeof buf, d);
  if (r.ec == std::errc())
    return std::string(buf, r.ptr);
#endif
  // No floating to_chars (libc++ on Emscripten and older macOS targets): 17 significant digits
  // always read back as the same double, if not always in the shortest form.
  char buf2[64];
  std::snprintf(buf2, sizeof buf2, "%.17g", d);
  // The shortest of %.15g .. %.17g that reads back the same.
  for (int prec = 15; prec < 17; ++prec) {
    char b[64];
    std::snprintf(b, sizeof b, "%.*g", prec, d);
    std::istringstream in(b);
    in.imbue(std::locale::classic());
    double back = 0.0;
    if ((in >> back) && back == d)
      return b;
  }
  return buf2;
}

inline std::string_view trim(std::string_view s) {
  const char *ws = " \t\r\n";
  const std::size_t b = s.find_first_not_of(ws);
  if (b == std::string_view::npos)
    return std::string_view();
  const std::size_t e = s.find_last_not_of(ws);
  return s.substr(b, e - b + 1);
}

inline std::string lower(std::string_view s) {
  std::string out(s);
  for (char &c : out)
    if (c >= 'A' && c <= 'Z')
      c = static_cast<char>(c - 'A' + 'a');
  return out;
}

inline bool parse_double(std::string_view s, double &out) {
  s = trim(s);
  if (s.empty())
    return false;
  const std::string l = lower(s);
  if (l == "nan" || l == "+nan" || l == "-nan") {
    out = std::numeric_limits<double>::quiet_NaN();
    return true;
  }
  if (l == "inf" || l == "+inf" || l == "infinity" || l == "+infinity") {
    out = std::numeric_limits<double>::infinity();
    return true;
  }
  if (l == "-inf" || l == "-infinity") {
    out = -std::numeric_limits<double>::infinity();
    return true;
  }
  if (s.front() == '+')
    s.remove_prefix(1);
#if defined(__cpp_lib_to_chars) && __cpp_lib_to_chars >= 201611L
  const std::from_chars_result r = std::from_chars(s.data(), s.data() + s.size(), out);
  return r.ec == std::errc() && r.ptr == s.data() + s.size();
#else
  std::istringstream in{std::string(s)};
  in.imbue(std::locale::classic());
  double v = 0.0;
  if (!(in >> v))
    return false;
  char extra = 0;
  if (in >> extra)
    return false;
  out = v;
  return true;
#endif
}

// Whether `d` is an integer that T holds exactly: lowest() <= d < 2^digits (T's max + 1, which a
// double holds exactly where max itself may round up to it).
template <class T> bool integral_in_range(double d) {
  return std::isfinite(d) && d == std::floor(d) &&
         d >= static_cast<double>(std::numeric_limits<T>::lowest()) &&
         d < std::ldexp(1.0, std::numeric_limits<T>::digits);
}

template <class T> bool parse_integer(std::string_view s, T &out) {
  s = trim(s);
  if (!s.empty() && s.front() == '+')
    s.remove_prefix(1);
  if (s.empty())
    return false;
  T v{};
  const std::from_chars_result r = std::from_chars(s.data(), s.data() + s.size(), v);
  if (r.ec == std::errc() && r.ptr == s.data() + s.size()) {
    out = v;
    return true;
  }
  // An integral value written as a number ("3.0", "1e3"), as the DSL or a JSON writer may.
  double d = 0.0;
  if (r.ec != std::errc::result_out_of_range && parse_double(s, d) && integral_in_range<T>(d)) {
    out = static_cast<T>(d);
    return true;
  }
  return false;
}

inline bool parse_bool(std::string_view s, bool &out) {
  const std::string l = lower(trim(s));
  if (l == "1" || l == "true" || l == "yes" || l == "on" || l == "#t") {
    out = true;
    return true;
  }
  if (l == "0" || l == "false" || l == "no" || l == "off" || l == "#f") {
    out = false;
    return true;
  }
  return false;
}

// ---- enums ----

template <class E> std::vector<std::string> enum_names() {
  std::vector<std::string> names;
  boost::mp11::mp_for_each<boost::describe::describe_enumerators<E>>(
      [&](auto D) { names.emplace_back(D.name); });
  return names;
}

template <class E> bool enum_name(E e, std::string &out) {
  bool found = false;
  boost::mp11::mp_for_each<boost::describe::describe_enumerators<E>>([&](auto D) {
    if (!found && D.value == e) {
      out = D.name;
      found = true;
    }
  });
  return found;
}

template <class E> bool enum_from_name(std::string_view s, E &out) {
  bool found = false;
  boost::mp11::mp_for_each<boost::describe::describe_enumerators<E>>([&](auto D) {
    if (!found && s == D.name) {
      out = D.value;
      found = true;
    }
  });
  return found;
}

// ---- strings (JSON-style quoting inside lists) ----

inline std::string quote(std::string_view s) {
  std::string out = "\"";
  for (const char c : s) {
    switch (c) {
    case '"':
      out += "\\\"";
      break;
    case '\\':
      out += "\\\\";
      break;
    case '\n':
      out += "\\n";
      break;
    case '\t':
      out += "\\t";
      break;
    case '\r':
      out += "\\r";
      break;
    default:
      out += c;
    }
  }
  out += '"';
  return out;
}

// ---- one scalar as text ----

template <class T> std::string scalar_text(const T &v) {
  if constexpr (std::is_same_v<T, bool>)
    return v ? "1" : "0";
  else if constexpr (is_integer<T>)
    return std::to_string(v);
  else if constexpr (std::is_floating_point_v<T>)
    return format_double(static_cast<double>(v));
  else if constexpr (std::is_same_v<T, std::string>)
    return v;
  else {
    std::string name;
    if (enum_name(v, name))
      return name;
    return std::to_string(static_cast<std::underlying_type_t<T>>(v));
  }
}

// What a value that does not parse is said not to be.
template <class T> std::string expected_of() {
  if constexpr (std::is_same_v<T, bool>)
    return "a bool (1 or 0)";
  else if constexpr (is_integer<T>) {
    if constexpr (std::is_unsigned_v<T>)
      return "a non-negative integer";
    else
      return "an integer";
  } else if constexpr (std::is_floating_point_v<T>)
    return "a number";
  else if constexpr (std::is_same_v<T, std::string>)
    return "a string";
  else {
    std::string s = "one of ";
    const std::vector<std::string> names = enum_names<T>();
    for (std::size_t i = 0; i < names.size(); ++i)
      s += (i ? ", " : "") + names[i];
    return s;
  }
}

template <class T> bool scalar_from_text(std::string_view s, T &out) {
  if constexpr (std::is_same_v<T, bool>)
    return parse_bool(s, out);
  else if constexpr (is_integer<T>)
    return parse_integer(s, out);
  else if constexpr (std::is_floating_point_v<T>) {
    double d = 0.0;
    if (!parse_double(s, d))
      return false;
    out = static_cast<T>(d);
    return true;
  } else if constexpr (std::is_same_v<T, std::string>) {
    out = std::string(s);
    return true;
  } else
    return enum_from_name(trim(s), out);
}

// Split a list's text into its elements' texts: a JSON array ([1, 2] / ["a", "b"]), a DSL-printed
// list ((1 2)), or bare elements separated by commas or blanks. Quoted elements are unquoted.
// False when the brackets or quotes do not match.
inline bool split_list(std::string_view s, std::vector<std::string> &out) {
  out.clear();
  s = trim(s);
  if (s.empty())
    return true;
  if ((s.front() == '[' && s.back() == ']') || (s.front() == '(' && s.back() == ')'))
    s = s.substr(1, s.size() - 2);
  else if (s.front() == '[' || s.front() == '(' || s.back() == ']' || s.back() == ')')
    return false;
  std::size_t i = 0;
  const auto sep = [](char c) {
    return c == ',' || c == ' ' || c == '\t' || c == '\r' || c == '\n';
  };
  while (i < s.size()) {
    while (i < s.size() && sep(s[i]))
      ++i;
    if (i >= s.size())
      break;
    std::string item;
    if (s[i] == '"') {
      ++i;
      bool closed = false;
      while (i < s.size()) {
        const char c = s[i++];
        if (c == '"') {
          closed = true;
          break;
        }
        if (c == '\\' && i < s.size()) {
          const char e = s[i++];
          item += e == 'n' ? '\n' : e == 't' ? '\t' : e == 'r' ? '\r' : e;
        } else {
          item += c;
        }
      }
      if (!closed)
        return false;
      if (i < s.size() && !sep(s[i]))
        return false;
    } else {
      while (i < s.size() && !sep(s[i]))
        item += s[i++];
    }
    out.push_back(std::move(item));
  }
  return true;
}

// ---- any field value as text ----

template <class T> std::string value_text(const T &v) {
  if constexpr (is_vector<T>::value) {
    using E = typename T::value_type;
    std::string out = "[";
    for (std::size_t i = 0; i < v.size(); ++i) {
      if (i)
        out += ", ";
      if constexpr (std::is_same_v<E, std::string> || is_described_enum<E>)
        out += quote(scalar_text<E>(v[i]));
      else if constexpr (std::is_same_v<E, bool>)
        out += v[i] ? "true" : "false";
      else
        out += scalar_text<E>(v[i]);
    }
    return out + "]";
  } else {
    return scalar_text(v);
  }
}

// Parse `s` into `out`; on failure leave `out` alone and say why.
template <class T> bool value_from_text(std::string_view s, T &out, std::string &why) {
  if constexpr (is_vector<T>::value) {
    using E = typename T::value_type;
    std::vector<std::string> items;
    if (!split_list(s, items)) {
      why = "'" + std::string(trim(s)) + "' is not a list";
      return false;
    }
    T v;
    v.reserve(items.size());
    for (std::size_t i = 0; i < items.size(); ++i) {
      E e{};
      if (!scalar_from_text<E>(items[i], e)) {
        why = "element " + std::to_string(i) + " '" + items[i] + "' is not " + expected_of<E>();
        return false;
      }
      v.push_back(std::move(e));
    }
    out = std::move(v);
    return true;
  } else {
    T v{};
    if (!scalar_from_text<T>(s, v)) {
      why = "'" + std::string(s) + "' is not " + expected_of<T>();
      return false;
    }
    out = std::move(v);
    return true;
  }
}

// ---- range checks ----

template <class T>
void check_scalar(const char *name, const T &v, double lo, double hi,
                  std::vector<std::string> &problems, const std::string &at) {
  if constexpr (is_integer<T> || std::is_floating_point_v<T>) {
    const double d = static_cast<double>(v);
    const std::string who = std::string(name) + at;
    if constexpr (std::is_floating_point_v<T>) {
      if (std::isnan(d)) {
        problems.push_back(who + " is not a number");
        return;
      }
    }
    if (!std::isnan(lo) && d < lo)
      problems.push_back(who + " = " + scalar_text(v) + " is below its minimum " +
                         format_double(lo));
    else if (!std::isnan(hi) && d > hi)
      problems.push_back(who + " = " + scalar_text(v) + " is above its maximum " +
                         format_double(hi));
  } else if constexpr (is_described_enum<T>) {
    std::string n;
    if (!enum_name(v, n))
      problems.push_back(std::string(name) + at + " = " +
                         std::to_string(static_cast<std::underlying_type_t<T>>(v)) + " is not " +
                         expected_of<T>());
  } else {
    (void)name, (void)v, (void)lo, (void)hi, (void)problems, (void)at;
  }
}

template <class T>
void check_value(const char *name, const T &v, double lo, double hi,
                 std::vector<std::string> &problems) {
  if constexpr (is_vector<T>::value) {
    for (std::size_t i = 0; i < v.size(); ++i)
      check_scalar(name, v[i], lo, hi, problems, "[" + std::to_string(i) + "]");
  } else {
    check_scalar(name, v, lo, hi, problems, std::string());
  }
}

template <class P>
concept has_check =
    requires(const P &p, std::vector<std::string> &v) { param_table<P>::check(p, v); };

} // namespace param_detail

// The base every param_table<P> specialization derives from: the table's vocabulary, so a table
// names field(), unit(), doc() and range() without qualification.
template <class P> struct param_fields {
  using params_type = P;
  template <class M, class... Opts>
  static constexpr param_field<P, M> field(const char *name, M P::*member, Opts... opts) {
    static_assert(param_detail::param_type_ok<M>::value,
                  "cvc::param_table: a field is a bool, an integer, float, double, std::string, "
                  "an enum described with BOOST_DESCRIBE_ENUM, or a std::vector of one of them");
    param_field<P, M> f;
    f.name = name;
    f.member = member;
    (param_detail::apply(f, opts), ...);
    return f;
  }
  static constexpr param_detail::unit_opt unit(const char *u) { return {u}; }
  static constexpr param_detail::doc_opt doc(const char *d) { return {d}; }
  static constexpr param_detail::range_opt range(double lo, double hi) { return {lo, hi}; }
};

// Call f(field) for every field of P's table, in table order.
template <class P, class F> void for_each_param(F &&f) {
  std::apply([&](const auto &...fld) { (f(fld), ...); }, param_table<P>::fields);
}

// What validate(), from_state() and the other readers return: the struct, and what was wrong.
template <class P> struct param_result {
  P value{};
  std::vector<std::string> errors; // empty: every value read and valid
  bool ok() const { return errors.empty(); }
  // The errors as one line ("t_low: 'abc' is not a number; gamma = 2 is above its maximum 0").
  std::string message() const {
    std::string m;
    for (std::size_t i = 0; i < errors.size(); ++i)
      m += (i ? "; " : "") + errors[i];
    return m;
  }
};

// A field of a table as data, for generic tools (the DSL's (params.defaults), docs, inspectors).
struct param_field_info {
  std::string name;
  std::string kind;      // bool | int | number | string | enum | list
  std::string elem_kind; // a list's element kind, else ""
  std::string unit;
  std::string doc;
  double min = std::numeric_limits<double>::quiet_NaN(); // NaN: no bound
  double max = std::numeric_limits<double>::quiet_NaN();
  std::vector<std::string> choices; // an enum's (or a list of enums') names
  std::string default_text;         // the default, as text (value_text)
};

// The name a field's CLI flag has: '.' and '_' as '-'.
inline std::string param_flag_name(std::string_view name) {
  std::string s(name);
  for (char &c : s)
    if (c == '.' || c == '_')
      c = '-';
  return s;
}

// The name a field's Python keyword (and member) has: '.' as '_'.
inline std::string param_keyword_name(std::string_view name) {
  std::string s(name);
  for (char &c : s)
    if (c == '.')
      c = '_';
  return s;
}

// The fields' names, in table order.
template <class P> std::vector<std::string> param_names() {
  std::vector<std::string> names;
  for_each_param<P>([&](const auto &f) { names.emplace_back(f.name); });
  return names;
}

// The fields as data, with the defaults of a default-constructed P.
template <class P> std::vector<param_field_info> param_field_infos() {
  std::vector<param_field_info> out;
  const P defaults{};
  for_each_param<P>([&](const auto &f) {
    using M = typename std::decay_t<decltype(f)>::value_type;
    param_field_info i;
    i.name = f.name;
    i.kind = param_detail::kind_of<M>();
    if constexpr (param_detail::is_vector<M>::value) {
      using E = typename M::value_type;
      i.elem_kind = param_detail::kind_of<E>();
      if constexpr (param_detail::is_described_enum<E>)
        i.choices = param_detail::enum_names<E>();
    } else if constexpr (param_detail::is_described_enum<M>) {
      i.choices = param_detail::enum_names<M>();
    }
    i.unit = f.unit ? f.unit : "";
    i.doc = f.doc ? f.doc : "";
    i.min = f.lo;
    i.max = f.hi;
    i.default_text = param_detail::value_text(defaults.*(f.member));
    out.push_back(std::move(i));
  });
  return out;
}

// What is wrong with `p`: each numeric value outside its range (or NaN), each enum value that is
// not one of its choices, then the table's own check(). Empty when nothing is.
template <class P> std::vector<std::string> params_validate(const P &p) {
  std::vector<std::string> problems;
  for_each_param<P>([&](const auto &f) {
    param_detail::check_value(f.name, p.*(f.member), f.lo, f.hi, problems);
  });
  if constexpr (param_detail::has_check<P>)
    param_table<P>::check(p, problems);
  return problems;
}

// Field `name`'s value as text, or false when P has no such field.
template <class P> bool param_get_text(const P &p, std::string_view name, std::string &out) {
  bool found = false;
  for_each_param<P>([&](const auto &f) {
    if (!found && name == f.name) {
      out = param_detail::value_text(p.*(f.member));
      found = true;
    }
  });
  return found;
}

// Set field `name` from text. False (with `why`) when P has no such field or the text does not
// parse; the field is left as it was then. Does not check ranges (params_validate does).
template <class P>
bool param_set_text(P &p, std::string_view name, std::string_view text, std::string &why) {
  bool found = false, ok = false;
  for_each_param<P>([&](const auto &f) {
    if (found || name != f.name)
      return;
    found = true;
    std::string w;
    ok = param_detail::value_from_text(text, p.*(f.member), w);
    if (!ok)
      why = std::string(f.name) + ": " + w;
  });
  if (!found) {
    why = "unknown parameter '" + std::string(name) + "'; valid: ";
    const std::vector<std::string> names = param_names<P>();
    for (std::size_t i = 0; i < names.size(); ++i)
      why += (i ? ", " : "") + names[i];
  }
  return found && ok;
}

// The struct from name -> text pairs over `base` (unknown names and unparseable texts are errors),
// then validated.
template <class P, class Map> param_result<P> params_from_strings(const Map &values, P base = {}) {
  param_result<P> r;
  r.value = std::move(base);
  for (const auto &kv : values) {
    std::string why;
    if (!param_set_text(r.value, kv.first, kv.second, why))
      r.errors.push_back(why);
  }
  for (std::string &p : params_validate(r.value))
    r.errors.push_back(std::move(p));
  return r;
}

namespace param_detail {

// Write a field's properties on its node (cvc/state/state_key_meta.h's names, plus "doc",
// "default", "choices" and "type").
template <class F, class P> void write_properties(cvc::state &s, const F &f, const P &defaults) {
  using M = typename F::value_type;
  const std::string d = f.doc ? f.doc : "";
  s.comment(d);
  s.property("doc", d);
  s.property("access", "build");
  s.property("type", kind_of<M>());
  s.property("default", value_text(defaults.*(f.member)));
  if (f.unit)
    s.property("unit", f.unit);
  if (!std::isnan(f.lo))
    s.property("min", format_double(f.lo));
  if (!std::isnan(f.hi))
    s.property("max", format_double(f.hi));
  std::vector<std::string> choices;
  if constexpr (is_vector<M>::value) {
    if constexpr (is_described_enum<typename M::value_type>)
      choices = enum_names<typename M::value_type>();
  } else if constexpr (is_described_enum<M>) {
    choices = enum_names<M>();
  }
  if (!choices.empty()) {
    std::string c;
    for (std::size_t i = 0; i < choices.size(); ++i)
      c += (i ? "," : "") + choices[i];
    s.property("choices", c);
  }
}

} // namespace param_detail

// Write every field of `p` under `node` (<node>.<name>), with its properties. A key that cannot be
// written (read-only through a link) is left as it was.
template <class P> void params_to_state(const P &p, cvc::state &node) {
  const P defaults{};
  for_each_param<P>([&](const auto &f) {
    try {
      cvc::state &s = node.childThroughLinks(f.name);
      s.value(param_detail::value_text(p.*(f.member)));
      param_detail::write_properties(s, f, defaults);
    } catch (const std::exception &) {
    }
  });
}

// Write each field's properties under `node` without touching its value (a node seeding its keys'
// metadata at realize time). Missing keys are created empty.
template <class P> void params_describe_state(cvc::state &node) {
  const P defaults{};
  for_each_param<P>([&](const auto &f) {
    try {
      param_detail::write_properties(node.childThroughLinks(f.name), f, defaults);
    } catch (const std::exception &) {
    }
  });
}

// Read the struct from the keys under `node`. A missing (or empty) key is seeded with its field's
// value in `base` (by default P's default) and gets its properties; a key that does not parse is
// an error (its field keeps base's value, and the key is left as it was). The result is then
// validated, so `errors` lists everything a caller must report before using the value.
template <class P> param_result<P> params_from_state(cvc::state &node, P base = {}) {
  param_result<P> r;
  r.value = std::move(base);
  const P defaults{};
  for_each_param<P>([&](const auto &f) {
    cvc::state *s = node.findThroughLinks(f.name);
    std::string text;
    try {
      if (s)
        text = s->value();
    } catch (const std::exception &) {
    }
    if (param_detail::trim(text).empty() &&
        !(std::is_same_v<typename std::decay_t<decltype(f)>::value_type, std::string> && s &&
          s->has_property("type"))) {
      // Missing: seed it (a string field whose key exists and was described is an empty string).
      try {
        cvc::state &w = node.childThroughLinks(f.name);
        w.value(param_detail::value_text(r.value.*(f.member)));
        param_detail::write_properties(w, f, defaults);
      } catch (const std::exception &) {
      }
      return;
    }
    std::string why;
    if (!param_detail::value_from_text(text, r.value.*(f.member), why))
      r.errors.push_back(std::string(f.name) + ": " + why);
  });
  for (std::string &p : params_validate(r.value))
    r.errors.push_back(std::move(p));
  return r;
}

// The table as Markdown, for a port's docs: | key | type | default | unit | range | description |.
template <class P> std::string params_markdown() {
  std::string out = "| Key | Type | Default | Unit | Range | Description |\n"
                    "|---|---|---|---|---|---|\n";
  for (const param_field_info &i : param_field_infos<P>()) {
    std::string type = i.kind;
    if (!i.elem_kind.empty())
      type += " of " + i.elem_kind;
    if (!i.choices.empty()) {
      type += ": ";
      for (std::size_t c = 0; c < i.choices.size(); ++c)
        type += (c ? ", " : "") + ("`" + i.choices[c] + "`");
    }
    std::string range;
    if (!std::isnan(i.min) || !std::isnan(i.max))
      range = (std::isnan(i.min) ? std::string() : param_detail::format_double(i.min)) + " .. " +
              (std::isnan(i.max) ? std::string() : param_detail::format_double(i.max));
    std::string doc = i.doc;
    for (char &c : doc)
      if (c == '|' || c == '\n')
        c = c == '|' ? '/' : ' ';
    out += "| `" + i.name + "` | " + type + " | `" + i.default_text + "` | " + i.unit + " | " +
           range + " | " + doc + " |\n";
  }
  return out;
}

} // namespace cvc

#endif // CVC_CORE_PARAM_TABLE_H
