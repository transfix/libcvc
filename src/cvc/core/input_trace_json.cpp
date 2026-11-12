/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "input_trace_json.h"

#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace cvc {
namespace trace_json {

namespace {

constexpr int kMaxDepth = 256;

[[noreturn]] void fail(const char *what, std::size_t at) {
  throw std::runtime_error(std::string("JSON: ") + what + " at byte " + std::to_string(at));
}

class parser {
public:
  explicit parser(std::string_view s) : s_(s) {}

  value document() {
    ws();
    value v = parse_value(0);
    ws();
    if (i_ != s_.size())
      fail("text after the value", i_);
    return v;
  }

private:
  std::string_view s_;
  std::size_t i_ = 0;

  void ws() {
    while (i_ < s_.size() && (s_[i_] == ' ' || s_[i_] == '\t' || s_[i_] == '\n' || s_[i_] == '\r'))
      ++i_;
  }

  bool eat(char c) {
    if (i_ < s_.size() && s_[i_] == c) {
      ++i_;
      return true;
    }
    return false;
  }

  void word(const char *w) {
    for (const char *p = w; *p; ++p)
      if (!eat(*p))
        fail("a bad literal", i_);
  }

  value parse_value(int depth) {
    if (depth > kMaxDepth)
      fail("nesting too deep", i_);
    if (i_ >= s_.size())
      fail("a value expected", i_);
    value v;
    const char c = s_[i_];
    if (c == '{') {
      ++i_;
      v.t = value::type::object;
      ws();
      if (eat('}'))
        return v;
      for (;;) {
        ws();
        if (i_ >= s_.size() || s_[i_] != '"')
          fail("a member name expected", i_);
        std::string key = parse_string();
        ws();
        if (!eat(':'))
          fail("':' expected", i_);
        ws();
        v.o.emplace_back(std::move(key), parse_value(depth + 1));
        ws();
        if (eat('}'))
          return v;
        if (!eat(','))
          fail("',' or '}' expected", i_);
      }
    }
    if (c == '[') {
      ++i_;
      v.t = value::type::array;
      ws();
      if (eat(']'))
        return v;
      for (;;) {
        ws();
        v.a.push_back(parse_value(depth + 1));
        ws();
        if (eat(']'))
          return v;
        if (!eat(','))
          fail("',' or ']' expected", i_);
      }
    }
    if (c == '"') {
      v.t = value::type::string;
      v.s = parse_string();
      return v;
    }
    if (c == 't') {
      word("true");
      v.t = value::type::boolean;
      v.b = true;
      return v;
    }
    if (c == 'f') {
      word("false");
      v.t = value::type::boolean;
      return v;
    }
    if (c == 'n') {
      word("null");
      return v;
    }
    if (c == '-' || (c >= '0' && c <= '9')) {
      const std::size_t start = i_;
      eat('-');
      if (eat('0')) {
      } else if (i_ < s_.size() && s_[i_] >= '1' && s_[i_] <= '9') {
        while (i_ < s_.size() && s_[i_] >= '0' && s_[i_] <= '9')
          ++i_;
      } else {
        fail("a bad number", i_);
      }
      if (eat('.')) {
        if (!(i_ < s_.size() && s_[i_] >= '0' && s_[i_] <= '9'))
          fail("a bad fraction", i_);
        while (i_ < s_.size() && s_[i_] >= '0' && s_[i_] <= '9')
          ++i_;
      }
      if (eat('e') || eat('E')) {
        if (!eat('+'))
          eat('-');
        if (!(i_ < s_.size() && s_[i_] >= '0' && s_[i_] <= '9'))
          fail("a bad exponent", i_);
        while (i_ < s_.size() && s_[i_] >= '0' && s_[i_] <= '9')
          ++i_;
      }
      v.t = value::type::number;
      v.s.assign(s_.substr(start, i_ - start));
      return v;
    }
    fail("an unexpected character", i_);
  }

  static void put_utf8(std::string &out, unsigned cp) {
    if (cp < 0x80) {
      out += static_cast<char>(cp);
    } else if (cp < 0x800) {
      out += static_cast<char>(0xC0 | (cp >> 6));
      out += static_cast<char>(0x80 | (cp & 0x3F));
    } else if (cp < 0x10000) {
      out += static_cast<char>(0xE0 | (cp >> 12));
      out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
      out += static_cast<char>(0x80 | (cp & 0x3F));
    } else {
      out += static_cast<char>(0xF0 | (cp >> 18));
      out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
      out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
      out += static_cast<char>(0x80 | (cp & 0x3F));
    }
  }

  unsigned hex4() {
    if (i_ + 4 > s_.size())
      fail("a short \\u escape", i_);
    unsigned v = 0;
    for (int k = 0; k < 4; ++k) {
      const char h = s_[i_++];
      v <<= 4;
      if (h >= '0' && h <= '9')
        v |= static_cast<unsigned>(h - '0');
      else if (h >= 'a' && h <= 'f')
        v |= static_cast<unsigned>(h - 'a' + 10);
      else if (h >= 'A' && h <= 'F')
        v |= static_cast<unsigned>(h - 'A' + 10);
      else
        fail("a bad \\u escape", i_ - 1);
    }
    return v;
  }

  std::string parse_string() {
    ++i_; // the opening quote
    std::string out;
    for (;;) {
      if (i_ >= s_.size())
        fail("an unterminated string", i_);
      const char c = s_[i_++];
      if (c == '"')
        return out;
      if (static_cast<unsigned char>(c) < 0x20)
        fail("a control character in a string", i_ - 1);
      if (c != '\\') {
        out += c;
        continue;
      }
      if (i_ >= s_.size())
        fail("an unterminated escape", i_);
      const char e = s_[i_++];
      switch (e) {
      case '"':
      case '\\':
      case '/':
        out += e;
        break;
      case 'b':
        out += '\b';
        break;
      case 'f':
        out += '\f';
        break;
      case 'n':
        out += '\n';
        break;
      case 'r':
        out += '\r';
        break;
      case 't':
        out += '\t';
        break;
      case 'u': {
        unsigned cp = hex4();
        if (cp >= 0xD800 && cp <= 0xDBFF && i_ + 1 < s_.size() && s_[i_] == '\\' &&
            s_[i_ + 1] == 'u') {
          const std::size_t save = i_;
          i_ += 2;
          const unsigned lo = hex4();
          if (lo >= 0xDC00 && lo <= 0xDFFF)
            cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
          else
            i_ = save; // a lone high surrogate: written as U+FFFD below
        }
        if (cp >= 0xD800 && cp <= 0xDFFF)
          cp = 0xFFFD;
        put_utf8(out, cp);
        break;
      }
      default:
        fail("a bad escape", i_ - 1);
      }
    }
  }
};

void dump_to(const value &v, std::string &out) {
  switch (v.t) {
  case value::type::null:
    out += "null";
    break;
  case value::type::boolean:
    out += v.b ? "true" : "false";
    break;
  case value::type::number:
    out += v.s;
    break;
  case value::type::string:
    append_string(out, v.s);
    break;
  case value::type::array:
    out += '[';
    for (std::size_t k = 0; k < v.a.size(); ++k) {
      if (k)
        out += ',';
      dump_to(v.a[k], out);
    }
    out += ']';
    break;
  case value::type::object:
    out += '{';
    for (std::size_t k = 0; k < v.o.size(); ++k) {
      if (k)
        out += ',';
      append_string(out, v.o[k].first);
      out += ':';
      dump_to(v.o[k].second, out);
    }
    out += '}';
    break;
  }
}

[[noreturn]] void wrong(const char *what, const char *want) {
  throw std::runtime_error(std::string(what ? what : "a value") + " is not " + want);
}

} // namespace

const value *value::get(std::string_view key) const {
  if (t != type::object)
    return nullptr;
  for (const auto &m : o)
    if (m.first == key)
      return &m.second;
  return nullptr;
}

double value::number(const char *what) const {
  if (t != type::number)
    wrong(what, "a number");
  // strtod reads the locale's decimal point; JSON's is always '.'.
  std::istringstream in(s);
  in.imbue(std::locale::classic());
  double d = 0.0;
  in >> d;
  if (!in)
    wrong(what, "a number in range");
  return d;
}

std::uint64_t value::u64(const char *what) const {
  if (t != type::number || s.empty() || s[0] == '-' || s.find_first_of(".eE") != std::string::npos)
    wrong(what, "a whole number >= 0");
  std::uint64_t u = 0;
  const auto r = std::from_chars(s.data(), s.data() + s.size(), u);
  if (r.ec != std::errc() || r.ptr != s.data() + s.size())
    wrong(what, "a whole number in range");
  return u;
}

std::int64_t value::i64(const char *what) const {
  if (t != type::number || s.find_first_of(".eE") != std::string::npos)
    wrong(what, "a whole number");
  std::int64_t i = 0;
  const auto r = std::from_chars(s.data(), s.data() + s.size(), i);
  if (r.ec != std::errc() || r.ptr != s.data() + s.size())
    wrong(what, "a whole number in range");
  return i;
}

const std::string &value::str(const char *what) const {
  if (t != type::string)
    wrong(what, "a string");
  return s;
}

std::string value::dump() const {
  std::string out;
  dump_to(*this, out);
  return out;
}

value parse(std::string_view text) { return parser(text).document(); }

void append_string(std::string &out, std::string_view s) {
  static const char *const kHex = "0123456789abcdef";
  out += '"';
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
    case '\r':
      out += "\\r";
      break;
    case '\t':
      out += "\\t";
      break;
    case '\b':
      out += "\\b";
      break;
    case '\f':
      out += "\\f";
      break;
    default:
      if (static_cast<unsigned char>(c) < 0x20) {
        out += "\\u00";
        out += kHex[(static_cast<unsigned char>(c) >> 4) & 0xF];
        out += kHex[static_cast<unsigned char>(c) & 0xF];
      } else {
        out += c;
      }
    }
  }
  out += '"';
}

std::string number(double d) {
  if (!std::isfinite(d))
    return "null";
#if defined(__cpp_lib_to_chars) && __cpp_lib_to_chars >= 201611L
  char buf[64];
  const auto r = std::to_chars(buf, buf + sizeof buf, d);
  if (r.ec == std::errc())
    return std::string(buf, r.ptr);
#endif
  // No floating to_chars (libc++ on Emscripten and older macOS targets): the shortest of
  // %.15g .. %.17g that reads back the same; 17 significant digits always do.
  for (int prec = 15; prec <= 17; ++prec) {
    char b[64];
    std::snprintf(b, sizeof b, "%.*g", prec, d);
    std::istringstream in(b);
    in.imbue(std::locale::classic());
    double back = 0.0;
    if (prec == 17 || ((in >> back) && back == d))
      return b;
  }
  return "null"; // not reached
}

std::string number(std::uint64_t u) { return std::to_string(u); }
std::string number(std::int64_t i) { return std::to_string(i); }

value make_string(std::string s) {
  value v;
  v.t = value::type::string;
  v.s = std::move(s);
  return v;
}

value make_number(std::string literal) {
  value v;
  v.t = value::type::number;
  v.s = std::move(literal);
  return v;
}

} // namespace trace_json
} // namespace cvc
