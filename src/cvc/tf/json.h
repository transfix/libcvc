/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// A minimal JSON reader / number writer for the cvc::tf state strings (the model and the overlay
// layers). Private to src/cvc/tf: the strings are small, machine-written and read every time a
// value changes, so a 150-line recursive-descent reader beats pulling a JSON library into core.
// Locale-independent both ways: '.' is the decimal point whatever the global C locale
// (setlocale's LC_NUMERIC, whose decimal point may be any character or a multi-byte sequence)
// and the global C++ locale say. Numbers never go through printf / strtod, which follow the C
// locale: std::to_chars / std::from_chars where the standard library has the floating-point
// overloads, else a stream imbued with the classic locale (libc++ and libstdc++ format and parse
// those through an explicit "C" locale).

#ifndef CVC_TF_JSON_H
#define CVC_TF_JSON_H

#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <locale>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

// Floating-point <charconv>. libc++ is left on the stream path: its floating-point overloads are
// unavailable before macOS 13.3 (libcvc deploys to 13.0) and it does not advertise them anyway.
// MSVC's streams are NOT safe on that path (they splice localeconv()'s first decimal-point byte
// into a printf / strtod in the C locale), so MSVC must take this one -- it always can.
#if defined(__cpp_lib_to_chars) && __cpp_lib_to_chars >= 201611L && !defined(_LIBCPP_VERSION)
#define CVC_TF_JSON_FP_CHARCONV 1
#else
#define CVC_TF_JSON_FP_CHARCONV 0
#endif

namespace cvc {
namespace tf {
namespace json {

struct value {
  enum class kind { null, boolean, number, string, array, object };
  kind k = kind::null;
  bool b = false;
  double num = 0.0;
  std::string str;
  std::vector<value> arr;
  std::vector<std::pair<std::string, value>> obj;

  bool is_number() const { return k == kind::number; }
  bool is_array() const { return k == kind::array; }
  bool is_object() const { return k == kind::object; }
  bool is_string() const { return k == kind::string; }
  // The member named `key` of an object, or nullptr.
  const value *find(const std::string &key) const {
    if (k == kind::object)
      for (const auto &kv : obj)
        if (kv.first == key)
          return &kv.second;
    return nullptr;
  }
};

namespace detail {
struct reader {
  const char *p, *end;
  std::string err;

  void ws() {
    while (p < end && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r'))
      ++p;
  }
  bool fail(const char *what) {
    if (err.empty())
      err = what;
    return false;
  }
  bool lit(const char *w) {
    const char *q = p;
    for (; *w; ++w, ++q)
      if (q >= end || *q != *w)
        return false;
    p = q;
    return true;
  }
  bool string(std::string &out) {
    if (p >= end || *p != '"')
      return fail("expected a string");
    ++p;
    out.clear();
    while (p < end && *p != '"') {
      char c = *p++;
      if (c == '\\') {
        if (p >= end)
          return fail("unterminated escape");
        const char e = *p++;
        switch (e) {
        case 'n':
          c = '\n';
          break;
        case 't':
          c = '\t';
          break;
        case 'r':
          c = '\r';
          break;
        case 'b':
          c = '\b';
          break;
        case 'f':
          c = '\f';
          break;
        case 'u': // keep BMP code points as UTF-8 (ids are ASCII in practice)
          if (end - p < 4)
            return fail("short \\u escape");
          {
            const unsigned cp =
                static_cast<unsigned>(std::strtoul(std::string(p, 4).c_str(), nullptr, 16));
            p += 4;
            if (cp < 0x80) {
              out += static_cast<char>(cp);
            } else if (cp < 0x800) {
              out += static_cast<char>(0xC0 | (cp >> 6));
              out += static_cast<char>(0x80 | (cp & 0x3F));
            } else {
              out += static_cast<char>(0xE0 | (cp >> 12));
              out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
              out += static_cast<char>(0x80 | (cp & 0x3F));
            }
          }
          continue;
        default:
          c = e;
          break; // \" \\ \/
        }
      }
      out += c;
    }
    if (p >= end)
      return fail("unterminated string");
    ++p;
    return true;
  }
  bool number(double &out) {
    // Scan the JSON number grammar, then convert without the C locale (see the top of the file):
    // correctly rounded (a %.17g value round-trips exactly) whatever the global decimal point.
    const char *s = p;
    if (p < end && (*p == '-' || *p == '+'))
      ++p;
    bool digits = false;
    while (p < end && *p >= '0' && *p <= '9') {
      ++p;
      digits = true;
    }
    if (p < end && *p == '.') {
      ++p;
      while (p < end && *p >= '0' && *p <= '9') {
        ++p;
        digits = true;
      }
    }
    if (!digits) {
      p = s;
      return fail("expected a number");
    }
    if (p < end && (*p == 'e' || *p == 'E')) {
      const char *e = p++;
      if (p < end && (*p == '-' || *p == '+'))
        ++p;
      if (p >= end || *p < '0' || *p > '9')
        p = e; // a bare 'e' is not part of the number
      while (p < end && *p >= '0' && *p <= '9')
        ++p;
    }
    const char *b = *s == '+' ? s + 1 : s;
#if CVC_TF_JSON_FP_CHARCONV
    // Out of range (overflow, or underflow to zero) fails, as the stream path does.
    double v = 0.0;
    const std::from_chars_result r = std::from_chars(b, p, v);
    if (r.ec != std::errc() || r.ptr != p)
      return fail("bad number");
    out = v;
#else
    std::istringstream is(std::string(b, p));
    is.imbue(std::locale::classic());
    is >> out;
    if (is.fail())
      return fail("bad number");
#endif
    return true;
  }
  bool parse(value &v, int depth) {
    if (depth > 64)
      return fail("nested too deeply");
    ws();
    if (p >= end)
      return fail("unexpected end");
    const char c = *p;
    if (c == '{') {
      ++p;
      v.k = value::kind::object;
      ws();
      if (p < end && *p == '}') {
        ++p;
        return true;
      }
      for (;;) {
        ws();
        std::string key;
        if (!string(key))
          return false;
        ws();
        if (p >= end || *p != ':')
          return fail("expected ':'");
        ++p;
        value m;
        if (!parse(m, depth + 1))
          return false;
        v.obj.emplace_back(std::move(key), std::move(m));
        ws();
        if (p < end && *p == ',') {
          ++p;
          continue;
        }
        if (p < end && *p == '}') {
          ++p;
          return true;
        }
        return fail("expected ',' or '}'");
      }
    }
    if (c == '[') {
      ++p;
      v.k = value::kind::array;
      ws();
      if (p < end && *p == ']') {
        ++p;
        return true;
      }
      for (;;) {
        value e;
        if (!parse(e, depth + 1))
          return false;
        v.arr.push_back(std::move(e));
        ws();
        if (p < end && *p == ',') {
          ++p;
          continue;
        }
        if (p < end && *p == ']') {
          ++p;
          return true;
        }
        return fail("expected ',' or ']'");
      }
    }
    if (c == '"') {
      v.k = value::kind::string;
      return string(v.str);
    }
    if (lit("true")) {
      v.k = value::kind::boolean;
      v.b = true;
      return true;
    }
    if (lit("false")) {
      v.k = value::kind::boolean;
      v.b = false;
      return true;
    }
    if (lit("null")) {
      v.k = value::kind::null;
      return true;
    }
    v.k = value::kind::number;
    return number(v.num);
  }
};
} // namespace detail

// Parse `s` (one JSON value, surrounding whitespace allowed). On failure returns false with a short
// reason in *error.
inline bool parse(const std::string &s, value &out, std::string *error = nullptr) {
  detail::reader r{s.data(), s.data() + s.size(), {}};
  out = value{};
  if (!r.parse(out, 0)) {
    if (error)
      *error = r.err;
    return false;
  }
  r.ws();
  if (r.p != r.end) {
    if (error)
      *error = "trailing characters";
    return false;
  }
  return true;
}

// Read [b, e) as exactly one number (the JSON grammar, plus a leading '+'); false on anything else,
// including surrounding blanks, "nan" and "inf". The strict scalar the CSV and .vinay readers
// share.
inline bool parse_number(const char *b, const char *e, double &out) {
  detail::reader r{b, e, {}};
  return r.number(out) && r.p == e;
}
inline bool parse_number(const std::string &s, double &out) {
  return parse_number(s.data(), s.data() + s.size(), out);
}

// Append a number with up to `sig` (1..17) significant digits in the shortest %g form -- exactly
// printf's "%.*g" in the "C" locale, whatever the global C / C++ locale ('.' decimal point, no
// digit grouping); non-finite values are written as 0, JSON has no NaN/Inf.
inline void append_number(std::string &out, double v, int sig = 9) {
  if (!std::isfinite(v))
    v = 0.0;
  sig = sig < 1 ? 1 : sig > 17 ? 17 : sig;
#if CVC_TF_JSON_FP_CHARCONV
  char buf[64];
  const std::to_chars_result r =
      std::to_chars(buf, buf + sizeof(buf), v, std::chars_format::general, sig);
  out.append(buf, r.ptr);
#else
  std::ostringstream os;
  os.imbue(std::locale::classic());
  os.precision(sig); // no floatfield: %g
  os << v;
  out += os.str();
#endif
}

// Append the shortest of %.15g..%.17g that parse_number reads back as exactly `v`: bit-exact round
// trips without writing 0.1 as 0.10000000000000001.
inline void append_exact(std::string &out, double v) {
  for (int sig = 15; sig < 17; ++sig) {
    std::string s;
    append_number(s, v, sig);
    double back = 0.0;
    if (parse_number(s, back) && back == v) {
      out += s;
      return;
    }
  }
  append_number(out, v, 17);
}

// The same for a float that is read back through a double and narrowed (%.6g..%.9g).
inline void append_exact_float(std::string &out, float v) {
  for (int sig = 6; sig < 9; ++sig) {
    std::string s;
    append_number(s, v, sig);
    double back = 0.0;
    if (parse_number(s, back) && static_cast<float>(back) == v) {
      out += s;
      return;
    }
  }
  append_number(out, v, 9);
}

// Append a JSON string literal.
inline void append_string(std::string &out, const std::string &s) {
  out += '"';
  for (const char c : s) {
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (static_cast<unsigned char>(c) < 0x20) {
      char buf[8];
      std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(c));
      out += buf;
    } else {
      out += c;
    }
  }
  out += '"';
}

} // namespace json
} // namespace tf
} // namespace cvc

#endif // CVC_TF_JSON_H
