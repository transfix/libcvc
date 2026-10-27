#include <algorithm>
#include <cctype>
#include <charconv>
#include <cstring>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/parser.h>
#include <cvc/state/state_exec/scheduler.h> // defaults_prefix / sched_prefix
#include <cvc/state/state_exec/utf8.h> // SE-2: \u/\x escapes -> UTF-8, and UTF-8 identifier bytes
#include <iterator>
#include <optional>
#include <sstream>

namespace cvc::state_exec {

// -- Free functions ----------------------------------------------------------

value_t parse(const std::string &input, const parse_options &opts) {
  parser p(input, opts);
  auto result = p.parse_expr();
  return result;
}

std::vector<value_t> parse_all(const std::string &input, const parse_options &opts) {
  parser p(input, opts);
  std::vector<value_t> results;
  while (!p.at_end()) {
    results.push_back(p.parse_expr());
  }
  return results;
}

// A non-negative integer setting at `path` (positive unless `allow_zero`), or nullopt when it is
// missing, empty, malformed or out of range, so the caller falls through to the next place in
// the resolution order.
static std::optional<std::size_t> read_size_setting(cvc::state &root, const std::string &path,
                                                    bool allow_zero) {
  cvc::state *node = root.findDescendant(path);
  if (!node)
    return std::nullopt;
  const std::string v = node->value();
  std::size_t n = 0;
  const auto [end, ec] = std::from_chars(v.data(), v.data() + v.size(), n);
  if (ec != std::errc{} || end != v.data() + v.size() || v.empty() || (n == 0 && !allow_zero))
    return std::nullopt;
  return n;
}

parse_options parse_options_from_state(cvc::state &root, const std::string &scheduler_id,
                                       const parse_options &base) {
  parse_options opts = base;
  const auto resolve = [&](const char *key, bool allow_zero, std::size_t &out) {
    const std::string suffix = std::string(".parser.") + key;
    if (auto v =
            read_size_setting(root, std::string(scheduler::defaults_prefix) + suffix, allow_zero))
      out = *v;
    if (!scheduler_id.empty())
      if (auto v = read_size_setting(
              root, std::string(scheduler::sched_prefix) + "." + scheduler_id + suffix, allow_zero))
        out = *v;
  };
  resolve("max_depth", /*allow_zero=*/false, opts.max_depth);
  resolve("step_tokens", /*allow_zero=*/true, opts.step_tokens); // 0 = parse at submit
  return opts;
}

// -- parser ------------------------------------------------------------------

parser::parser(std::string_view input, const parse_options &opts) : _input(input), _opts(opts) {
  _frames.reserve(32); // one allocation covers real scripts (the corpus nests at most 13 deep)
}

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
  _frames.clear(); // a datum left half-read by an earlier parse_error is abandoned
  std::size_t unlimited = static_cast<std::size_t>(-1);
  value_t out;
  return read(out, unlimited) == read_status::datum ? out : nil_value;
}

// Push an open list or quote. The depth check comes first, so the error points at the `(` or `'`
// that would go one level too deep.
void parser::open_frame(bool quote) {
  if (_frames.size() >= _opts.max_depth)
    error("nesting deeper than " + std::to_string(_opts.max_depth) +
          " levels (state_exec setting parser.max_depth)");
  frame f{quote, {}};
  if (!quote) {
    // Collect the elements in a kept vector, or one with room for a typical form's, so that
    // collecting doesn't grow it an element at a time.
    if (!_spare_elements.empty()) {
      f.elements = std::move(_spare_elements.back());
      _spare_elements.pop_back();
    } else {
      f.elements.reserve(8);
    }
  }
  _frames.push_back(std::move(f));
}

parser::read_status parser::read(value_t &out, std::size_t &budget) {
  while (budget > 0) {
    skip_whitespace_and_comments();
    value_t datum;
    if (_pos >= _input.size()) {
      if (_frames.empty())
        return read_status::end;
      if (!_frames.back().quote)
        error("unclosed parenthesis");
      datum = nil_value; // a quote at end of input quotes nil (the pre-grammar behavior)
    } else {
      --budget;
      const char c = peek();
      if (c == '(') {
        open_frame(false);
        advance();
        continue;
      }
      if (c == '\'') { // 'expr -> (quote expr)
        open_frame(true);
        advance();
        continue;
      }
      if (c == ')' && !_frames.empty() && !_frames.back().quote) {
        advance();
        // The list gets a vector of its exact size; the one it was collected in is kept.
        std::vector<value_t> &elements = _frames.back().elements;
        datum = make_list(std::vector<value_t>(std::make_move_iterator(elements.begin()),
                                               std::make_move_iterator(elements.end())));
        elements.clear();
        if (_spare_elements.size() < _frames.capacity())
          _spare_elements.push_back(std::move(elements));
        _frames.pop_back();
      } else if (c == '"') {
        datum = parse_string();
      } else if (c == 'b' && _pos + 1 < _input.size() && _input[_pos + 1] == '"') {
        // SE-3: a `b` immediately followed by `"` (no space) is a bytes literal b"...". A lone
        // `b`, or `b` followed by anything else, stays an ordinary symbol.
        advance(); // consume the 'b' prefix
        datum = parse_bytes_string();
      } else {
        // Everything else is an atom, including a `)` with no open list around it (it reads as
        // a symbol until the grammar's P2 makes it an error).
        datum = parse_atom();
      }
    }

    // A complete datum closes every quote waiting on it, then joins the innermost open list, or
    // is the result when nothing is open.
    while (!_frames.empty() && _frames.back().quote) {
      _frames.pop_back();
      datum = make_list({value_t(symbol{"quote"}), std::move(datum)});
    }
    if (_frames.empty()) {
      out = std::move(datum);
      return read_status::datum;
    }
    _frames.back().elements.push_back(std::move(datum));
  }
  return read_status::more;
}

// -- script_reader -------------------------------------------------------------

script_reader::script_reader(std::string source, const parse_options &opts)
    : _source(std::move(source)), _opts(opts), _parser(_source, opts) {}

bool script_reader::step(std::size_t budget) {
  for (;;) {
    value_t form;
    switch (_parser.read(form, budget)) {
    case parser::read_status::datum:
      _forms.push_back(std::move(form));
      break;
    case parser::read_status::end:
      return true;
    case parser::read_status::more:
      return false;
    }
  }
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
      // A run of plain characters up to the next quote or escape, added at once.
      std::size_t stop = _pos + 1;
      while (stop < _input.size() && _input[stop] != '"' && _input[stop] != '\\')
        ++stop;
      skip_run(stop, result);
    }
  }
  error("unterminated string literal");
}

void parser::skip_run(std::size_t stop, std::string &into) {
  const std::string_view run = _input.substr(_pos, stop - _pos);
  into.append(run);
  // The line and column advance() would have left.
  const std::size_t last_newline = run.rfind('\n');
  if (last_newline == std::string_view::npos) {
    _col += run.size();
  } else {
    _line += static_cast<std::size_t>(std::count(run.begin(), run.end(), '\n'));
    _col = 1 + (run.size() - last_newline - 1);
  }
  _pos = stop;
}

// SE-3: b"..." bytes literal. Unlike parse_string (text), escapes are BYTE-valued: \xNN is a raw
// byte (0-255), NOT a codepoint, and there is no \u (codepoints are meaningless for opaque octets).
// Raw source bytes (incl. >= 0x80) are taken verbatim.
value_t parser::parse_bytes_string() {
  advance(); // consume opening '"'
  std::string bytes;
  auto hex1 = [&]() -> unsigned {
    if (_pos >= _input.size())
      error("unexpected end of input in bytes escape");
    const char h = advance();
    if (h >= '0' && h <= '9')
      return static_cast<unsigned>(h - '0');
    if (h >= 'a' && h <= 'f')
      return static_cast<unsigned>(h - 'a' + 10);
    if (h >= 'A' && h <= 'F')
      return static_cast<unsigned>(h - 'A' + 10);
    error("invalid hex digit in bytes escape");
  };
  while (_pos < _input.size()) {
    char c = _input[_pos];
    if (c == '"') {
      advance(); // consume closing '"'
      return make_bytes(std::move(bytes));
    }
    if (c == '\\') {
      advance();
      if (_pos >= _input.size())
        error("unexpected end of input in bytes escape");
      char esc = advance();
      switch (esc) {
      case 'n':
        bytes += '\n';
        break;
      case 't':
        bytes += '\t';
        break;
      case 'r':
        bytes += '\r';
        break;
      case '\\':
        bytes += '\\';
        break;
      case '"':
        bytes += '"';
        break;
      case '0':
        bytes += '\0';
        break;
      case 'x':
        bytes += static_cast<char>((hex1() << 4) | hex1()); // a raw byte, not a codepoint
        break;
      default:
        bytes += '\\';
        bytes += esc;
        break;
      }
    } else {
      std::size_t stop = _pos + 1;
      while (stop < _input.size() && _input[stop] != '"' && _input[stop] != '\\')
        ++stop;
      skip_run(stop, bytes);
    }
  }
  error("unterminated bytes literal");
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

// Whether strtod could read a number from the whole of `token` (symbol characters only): it
// reads an optional sign, then digits, a '.', "inf"/"infinity" or "nan" (any case), or a hex
// "0x" number, so a token starting otherwise is a symbol without asking it.
static bool may_be_float(std::string_view token) {
  std::size_t i = token[0] == '+' || token[0] == '-' ? 1 : 0;
  if (i >= token.size())
    return false;
  const char c = token[i];
  return (c >= '0' && c <= '9') || c == '.' || c == 'i' || c == 'I' || c == 'n' || c == 'N';
}

value_t parser::parse_number_or_symbol(char first) {
  // The token is `first` (just read) and the symbol characters after it, read in one go: none is
  // a newline, so the column moves on by the token's length.
  (void)first;
  const std::size_t start = _pos - 1;
  std::size_t end = _pos;
  while (end < _input.size() && is_symbol_char(_input[end]))
    ++end;
  _col += end - _pos;
  _pos = end;
  const std::string_view token = _input.substr(start, end - start);

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

  std::string text(token);

  // Try float
  if (may_be_float(token)) {
    // std::from_chars for double may not be available everywhere;
    // fall back to strtod for robustness
    char *stop = nullptr;
    double dval = std::strtod(text.c_str(), &stop);
    if (stop == text.c_str() + text.size() && stop != text.c_str())
      return value_t(dval);
  }

  // It's a symbol
  return value_t(symbol{std::move(text)});
}

} // namespace cvc::state_exec
