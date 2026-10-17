/**
 * @file state_exec_grammar_dump.h
 * @brief Exact, type-tagged rendering of parser output for the grammar tests.
 *
 * state_exec_grammar_test and state_exec_corpus_test compare parses through this dump, not
 * through to_string(): to_string is lossy (unescaped strings, 6-digit floats) and changes in
 * phase P5 of docs/STATE_EXEC_GRAMMAR.md, while a golden must mean the same thing before and
 * after every phase. The format:
 *
 *   nil / #t / #f       nil and the bools
 *   42, -7              int64
 *   1.5, 31.0, 1e+20    double: whole numbers below 1e16 as N.0, others in the shortest
 *                       form that reads back exactly; always with '.', 'e' or a special
 *   +inf.0 -inf.0 +nan.0
 *   "..."               string: \\ \" \n \t \r, other ASCII controls as \xNN, invalid UTF-8
 *                       bytes as \xNN (always >= \x80), invisible or control code points as
 *                       \u{X}, everything else verbatim
 *   b"..."              bytes: printable ASCII verbatim, the rest \xNN
 *   foo, |1+|           symbol; |...| (same escapes, plus \|) when the bare text would read
 *                       as something else
 *   (a b)               list
 *
 * dump_all() joins top-level values with '\n' and renders a parse_error as "!error line:col".
 */
#ifndef CVC_STATE_EXEC_GRAMMAR_DUMP_H
#define CVC_STATE_EXEC_GRAMMAR_DUMP_H

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/state/state_exec/parser.h>
#include <cvc/state/state_exec/types.h>
#include <cvc/state/state_exec/utf8.h>
#include <string>
#include <string_view>

namespace cvc::state_exec::grammar_dump {

/// Code points a dump writes as \u{X}: C1 controls, bidi controls, U+FEFF, and the
/// whitespace that is not ASCII (so an NBSP in a golden is visible).
inline bool invisible(char32_t cp) {
  return (cp >= 0x80 && cp <= 0x9F) || cp == 0x61C || cp == 0xA0 || cp == 0x1680 ||
         (cp >= 0x2000 && cp <= 0x200F) || (cp >= 0x2028 && cp <= 0x202F) || cp == 0x205F ||
         (cp >= 0x2066 && cp <= 0x2069) || cp == 0x3000 || cp == 0xFEFF;
}

inline void hex_byte(unsigned char b, std::string &out) {
  static const char kHex[] = "0123456789abcdef";
  out += "\\x";
  out += kHex[b >> 4];
  out += kHex[b & 0x0F];
}

/// Escape UTF-8 text for a string or |symbol| dump; `quote` is the closing delimiter.
inline void text(std::string_view s, char quote, std::string &out) {
  std::size_t pos = 0;
  while (pos < s.size()) {
    char32_t cp;
    std::size_t len;
    if (!utf8::decode(s, pos, cp, len)) {
      hex_byte(static_cast<unsigned char>(s[pos]), out);
      pos += 1;
      continue;
    }
    if (cp == '\\' || cp == static_cast<char32_t>(quote)) {
      out += '\\';
      out += static_cast<char>(cp);
    } else if (cp == '\n') {
      out += "\\n";
    } else if (cp == '\t') {
      out += "\\t";
    } else if (cp == '\r') {
      out += "\\r";
    } else if (cp < 0x20 || cp == 0x7F) {
      hex_byte(static_cast<unsigned char>(cp), out);
    } else if (invisible(cp)) {
      char buf[16];
      std::snprintf(buf, sizeof buf, "\\u{%X}", static_cast<unsigned>(cp));
      out += buf;
    } else {
      out.append(s.substr(pos, len));
    }
    pos += len;
  }
}

/// Shortest decimal form that reads back to exactly `d`, marked as a double.
inline std::string number(double d) {
  if (std::isnan(d))
    return "+nan.0";
  if (std::isinf(d))
    return d > 0 ? "+inf.0" : "-inf.0";
  char buf[40];
  if (d == std::trunc(d) && std::fabs(d) < 1e16) { // whole numbers read as 1000.0, not 1e+03
    std::snprintf(buf, sizeof buf, "%.1f", d);
    return buf;
  }
  for (int prec = 1; prec <= 17; ++prec) {
    std::snprintf(buf, sizeof buf, "%.*g", prec, d);
    if (std::strtod(buf, nullptr) == d)
      break;
  }
  std::string s(buf);
  if (s.find_first_of(".e") == std::string::npos)
    s += ".0";
  return s;
}

/// True when a symbol's bare name would read back as something other than that symbol.
inline bool needs_bars(std::string_view name) {
  if (name.empty() || name == "nil" || name == "true" || name == "false" || name[0] == '#')
    return true;
  const char c0 = name[0];
  if (c0 >= '0' && c0 <= '9')
    return true;
  if ((c0 == '+' || c0 == '-' || c0 == '.') && name.size() > 1) {
    const char c1 = name[1];
    if ((c1 >= '0' && c1 <= '9') || c1 == '.' || c1 == 'i' || c1 == 'n')
      return true;
  }
  std::size_t pos = 0;
  while (pos < name.size()) {
    char32_t cp;
    std::size_t len;
    if (!utf8::decode(name, pos, cp, len) || cp <= 0x20 || cp == 0x7F || invisible(cp))
      return true;
    switch (cp) {
    case '(':
    case ')':
    case '"':
    case '\'':
    case ';':
    case ',':
    case '[':
    case ']':
    case '\\':
    case '`':
    case '{':
    case '|':
    case '}':
      return true;
    default:
      break;
    }
    pos += len;
  }
  return false;
}

inline void value(const value_t &v, std::string &out) {
  std::visit(
      [&](auto &&arg) {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, std::monostate>) {
          out += "nil";
        } else if constexpr (std::is_same_v<T, bool>) {
          out += arg ? "#t" : "#f";
        } else if constexpr (std::is_same_v<T, int64_t>) {
          out += std::to_string(arg);
        } else if constexpr (std::is_same_v<T, double>) {
          out += number(arg);
        } else if constexpr (std::is_same_v<T, std::string>) {
          out += '"';
          text(arg, '"', out);
          out += '"';
        } else if constexpr (std::is_same_v<T, bytes_value>) {
          out += "b\"";
          for (unsigned char b : arg.data) {
            if (b == '"' || b == '\\') {
              out += '\\';
              out += static_cast<char>(b);
            } else if (b >= 0x20 && b < 0x7F) {
              out += static_cast<char>(b);
            } else {
              hex_byte(b, out);
            }
          }
          out += '"';
        } else if constexpr (std::is_same_v<T, symbol>) {
          if (needs_bars(arg.name)) {
            out += '|';
            text(arg.name, '|', out);
            out += '|';
          } else {
            out += arg.name;
          }
        } else if constexpr (std::is_same_v<T, list_ptr>) {
          out += '(';
          if (arg) {
            for (std::size_t i = 0; i < arg->size(); ++i) {
              if (i > 0)
                out += ' ';
              value((*arg)[i], out);
            }
          }
          out += ')';
        } else {
          out += "#<" + v.type_name() + ">"; // the parser never produces these
        }
      },
      v.v);
}

inline std::string value(const value_t &v) {
  std::string out;
  value(v, out);
  return out;
}

inline std::string error(const parse_error &e) {
  return "!error " + std::to_string(e.line()) + ":" + std::to_string(e.col());
}

/// parse_all(src), one dumped value per line.
inline std::string dump_all(const std::string &src) {
  try {
    std::string out;
    for (const value_t &v : parse_all(src)) {
      if (!out.empty())
        out += '\n';
      value(v, out);
    }
    return out;
  } catch (const parse_error &e) {
    return error(e);
  }
}

/// parse(src), dumped.
inline std::string dump_one(const std::string &src) {
  try {
    return value(parse(src));
  } catch (const parse_error &e) {
    return error(e);
  }
}

} // namespace cvc::state_exec::grammar_dump

#endif // CVC_STATE_EXEC_GRAMMAR_DUMP_H
