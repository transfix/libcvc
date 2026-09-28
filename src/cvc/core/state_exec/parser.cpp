#include <cctype>
#include <charconv>
#include <cstring>
#include <cvc/core/state_exec/parser.h>
#include <cvc/core/state_exec/utf8.h> // SE-2: \u/\x escapes -> UTF-8, and UTF-8 identifier bytes
#include <sstream>

namespace cvc::state_exec {

// -- Free functions ----------------------------------------------------------

value_t parse(const std::string &input) {
  parser p(input);
  auto result = p.parse_expr();
  return result;
}

std::vector<value_t> parse_all(const std::string &input) {
  parser p(input);
  std::vector<value_t> results;
  while (!p.at_end()) {
    results.push_back(p.parse_expr());
  }
  return results;
}

// -- parser ------------------------------------------------------------------

parser::parser(std::string_view input) : _input(input) {}

bool parser::at_end() const {
  // Skip whitespace conceptually
  auto pos = _pos;
  while (pos < _input.size()) {
    char c = _input[pos];
    if (c == ';') {
      while (pos < _input.size() && _input[pos] != '\n')
        ++pos;
    } else if (std::isspace(static_cast<unsigned char>(c))) {
      ++pos;
    } else {
      return false;
    }
  }
  return true;
}

char parser::peek() const {
  if (_pos >= _input.size())
    return '\0';
  return _input[_pos];
}

char parser::advance() {
  if (_pos >= _input.size())
    error("unexpected end of input");
  char c = _input[_pos++];
  if (c == '\n') {
    ++_line;
    _col = 1;
  } else {
    ++_col;
  }
  return c;
}

void parser::skip_whitespace_and_comments() {
  while (_pos < _input.size()) {
    char c = _input[_pos];
    if (c == ';') {
      // Line comment — skip to end of line
      while (_pos < _input.size() && _input[_pos] != '\n')
        advance();
    } else if (std::isspace(static_cast<unsigned char>(c))) {
      advance();
    } else {
      break;
    }
  }
}

void parser::error(const std::string &msg) { throw parse_error(msg, _line, _col); }

value_t parser::parse_expr() {
  skip_whitespace_and_comments();
  if (_pos >= _input.size())
    return nil_value;

  char c = peek();

  if (c == '(')
    return parse_list();
  if (c == '\'') {
    // Quote: 'expr → (quote expr)
    advance();
    auto quoted = parse_expr();
    return make_list({value_t(symbol{"quote"}), quoted});
  }
  if (c == '"')
    return parse_string();

  return parse_atom();
}

value_t parser::parse_list() {
  advance(); // consume '('
  std::vector<value_t> elements;

  skip_whitespace_and_comments();
  while (peek() != ')') {
    if (_pos >= _input.size())
      error("unclosed parenthesis");
    elements.push_back(parse_expr());
    skip_whitespace_and_comments();
  }
  advance(); // consume ')'

  return make_list(std::move(elements));
}

value_t parser::parse_string() {
  advance(); // consume opening '"'
  std::string result;

  while (_pos < _input.size()) {
    char c = _input[_pos];
    if (c == '"') {
      advance(); // consume closing '"'
      return value_t(std::move(result));
    }
    if (c == '\\') {
      advance(); // consume backslash
      if (_pos >= _input.size())
        error("unexpected end of input in string escape");
      char esc = advance();
      // SE-2 (A8): read one hex digit, or fail. `error()` is [[noreturn]].
      auto hex1 = [&]() -> unsigned {
        if (_pos >= _input.size())
          error("unexpected end of input in numeric string escape");
        const char h = advance();
        if (h >= '0' && h <= '9')
          return static_cast<unsigned>(h - '0');
        if (h >= 'a' && h <= 'f')
          return static_cast<unsigned>(h - 'a' + 10);
        if (h >= 'A' && h <= 'F')
          return static_cast<unsigned>(h - 'A' + 10);
        error("invalid hex digit in string escape");
      };
      switch (esc) {
      case 'n':
        result += '\n';
        break;
      case 't':
        result += '\t';
        break;
      case 'r':
        result += '\r';
        break;
      case '\\':
        result += '\\';
        break;
      case '"':
        result += '"';
        break;
      case '0': // NUL
        result += '\0';
        break;
      case 'x': { // \xNN -> codepoint U+00NN, encoded UTF-8 (text semantics, like Python 3 str)
        char32_t cp = static_cast<char32_t>((hex1() << 4) | hex1());
        utf8::encode(cp, result);
        break;
      }
      case 'u': { // \uXXXX (exactly 4 hex) or \u{H..H} (1-6 hex) -> codepoint, encoded UTF-8
        char32_t cp = 0;
        if (_pos < _input.size() && _input[_pos] == '{') {
          advance(); // consume '{'
          int ndig = 0;
          while (_pos < _input.size() && _input[_pos] != '}') {
            cp = (cp << 4) | hex1();
            if (++ndig > 6)
              error("\\u{...} has more than 6 hex digits");
          }
          if (_pos >= _input.size())
            error("unterminated \\u{...} escape");
          advance(); // consume '}'
          if (ndig == 0)
            error("empty \\u{} escape");
        } else {
          for (int i = 0; i < 4; ++i)
            cp = (cp << 4) | hex1();
        }
        utf8::encode(cp, result); // out-of-range / surrogate -> U+FFFD (see utf8::encode)
        break;
      }
      default:
        result += '\\';
        result += esc;
        break;
      }
    } else {
      result += c;
      advance();
    }
  }
  error("unterminated string literal");
}

value_t parser::parse_atom() {
  char first = advance();
  return parse_number_or_symbol(first);
}

static bool is_symbol_char(char c) {
  const unsigned char uc = static_cast<unsigned char>(c);
  // SE-2 (A7): any byte >= 0x80 is a UTF-8 lead/continuation byte — accept it as an identifier
  // constituent so symbols/variable/function names may contain non-ASCII letters (e.g. `café`),
  // instead of being truncated at the first high byte.
  if (uc >= 0x80)
    return true;
  // SE-2 (A11): classify ASCII locale-INDEPENDENTLY (not std::isalnum, whose result for bytes
  // 0x80-0xFF is locale-dependent — moot now that >=0x80 is handled above, but this also pins the
  // ASCII set regardless of the process locale).
  if ((uc >= 'a' && uc <= 'z') || (uc >= 'A' && uc <= 'Z') || (uc >= '0' && uc <= '9'))
    return true;
  switch (c) {
  case '_':
  case '-':
  case '+':
  case '*':
  case '/':
  case '%':
  case '<':
  case '>':
  case '=':
  case '!':
  case '?':
  case '.':
  case '&':
  case '@':
  case '#':
  case '$':
  case '^':
  case '~':
  case ':':
    return true;
  default:
    return false;
  }
}

value_t parser::parse_number_or_symbol(char first) {
  std::string token(1, first);

  while (_pos < _input.size() && is_symbol_char(peek())) {
    token += advance();
  }

  // Check for boolean literals
  if (token == "#t" || token == "true")
    return value_t(true);
  if (token == "#f" || token == "false" || token == "nil")
    return nil_value;

  // Try integer
  {
    int64_t ival = 0;
    auto [ptr, ec] = std::from_chars(token.data(), token.data() + token.size(), ival);
    if (ec == std::errc{} && ptr == token.data() + token.size())
      return value_t(ival);
  }

  // Try float
  {
    // std::from_chars for double may not be available everywhere;
    // fall back to strtod for robustness
    char *end = nullptr;
    double dval = std::strtod(token.c_str(), &end);
    if (end == token.c_str() + token.size() && end != token.c_str())
      return value_t(dval);
  }

  // It's a symbol
  return value_t(symbol{std::move(token)});
}

} // namespace cvc::state_exec
