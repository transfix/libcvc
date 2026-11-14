/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The small JSON reader and writer cvc.input_trace/v1 lines go through (input_trace.cpp, and
// nav's sim_world_trace.cpp for a header's world section). Not installed: the trace's public
// face keeps JSON objects as text.
//
// A number keeps its literal, so an integer tick or a seed past 2^53 reads back exactly and a
// double written in its shortest round-trip form reads back as the same double. Objects keep
// their members in document order. dump() writes compact JSON.

#ifndef __CVC_CORE_INPUT_TRACE_JSON_H__
#define __CVC_CORE_INPUT_TRACE_JSON_H__

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace cvc {
namespace trace_json {

struct value {
  enum class type { null, boolean, number, string, array, object };
  type t = type::null;
  bool b = false;
  std::string s; // a string's text, or a number's literal
  // An array's items; an object's member values, in document order, keyed by `keys` (one each).
  // Not a vector of (key, value) pairs: a pair needs `value` complete, and here it is not yet.
  std::vector<value> a;
  std::vector<std::string> keys;

  bool is_object() const { return t == type::object; }
  bool is_array() const { return t == type::array; }
  bool is_string() const { return t == type::string; }
  bool is_number() const { return t == type::number; }
  // The member `key` of an object, or nullptr (for a missing key, or a value that isn't one).
  const value *get(std::string_view key) const;
  // A number as a double, or as an unsigned 64-bit integer (a literal with a sign, a fraction or
  // an exponent, or one out of range, throws). A value of another type throws std::runtime_error
  // naming `what`.
  double number(const char *what) const;
  std::uint64_t u64(const char *what) const;
  std::int64_t i64(const char *what) const;
  const std::string &str(const char *what) const;
  std::string dump() const;
};

// Parse one JSON text (surrounding whitespace allowed). Throws std::runtime_error naming the
// byte offset of a syntax error. Nesting is bounded (256 levels).
value parse(std::string_view text);

// Writers for hand-built documents.
void append_string(std::string &out, std::string_view s); // quoted and escaped
std::string number(double d);                             // shortest round trip; NaN, inf: null
std::string number(std::uint64_t u);
std::string number(std::int64_t i);

value make_string(std::string s);
value make_number(std::string literal);

} // namespace trace_json
} // namespace cvc

#endif // __CVC_CORE_INPUT_TRACE_JSON_H__
