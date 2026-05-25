#include <cctype>
#include <charconv>
#include <cstring>
#include <cvc/state_exec/parser.h>
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
  if (std::isalnum(static_cast<unsigned char>(c)))
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
