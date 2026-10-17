/**
 * @file parser.h
 * @brief S-expression parser for the state_exec DSL.
 *
 * Converts source text into a tree of value_t nodes.  Supports atoms
 * (integers, floats, strings, byte strings, symbols), lists, quote ('x), and
 * line comments.  Reports parse errors with line/column information.  The
 * syntax is specified in docs/STATE_EXEC_GRAMMAR.md, which also lists where
 * this parser still differs from it.
 */
#ifndef CVC_STATE_EXEC_PARSER_H
#define CVC_STATE_EXEC_PARSER_H

#include <cstddef>
#include <cvc/state/state_exec/types.h>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace cvc {
class state;
}

namespace cvc::state_exec {

/// Parse error with location information.
class parse_error : public std::runtime_error {
public:
  parse_error(const std::string &msg, std::size_t line, std::size_t col)
      : std::runtime_error(msg), _line(line), _col(col) {}

  std::size_t line() const { return _line; }
  std::size_t col() const { return _col; }

private:
  std::size_t _line;
  std::size_t _col;
};

/// Parser settings ("Parser settings" in docs/STATE_EXEC_GRAMMAR.md). Hosts keep them on the
/// state tree under `parser.*`; parse_options_from_state() reads them from there.
struct parse_options {
  /// How many lists and quotes may nest (`'x` reads as `(quote x)`, one level). The parser
  /// recurses on the native stack once per level, so deeper input is a parse_error rather than a
  /// stack overflow. State-tree key: `parser.max_depth`.
  std::size_t max_depth = 256;
};

/// Resolve parser settings from a state tree, per key: `state_exec.schedulers.<scheduler_id>.
/// parser.<key>` (when scheduler_id is not empty), then `state_exec.defaults.parser.<key>`, then
/// the parse_options default. A value that is missing, empty, not a positive integer or out of
/// range is skipped.
parse_options parse_options_from_state(cvc::state &root, const std::string &scheduler_id = {});

/// Parse a single S-expression from the input string.
/// Returns the parsed value_t.
/// @throws parse_error on syntax errors.
value_t parse(const std::string &input, const parse_options &opts = {});

/// Parse all S-expressions from the input string.
/// Returns a vector of parsed value_t values.
/// @throws parse_error on syntax errors.
std::vector<value_t> parse_all(const std::string &input, const parse_options &opts = {});

/// Internal tokenizer/parser — exposed for testing.
class parser {
public:
  explicit parser(std::string_view input, const parse_options &opts = {});

  /// Parse one expression.  Returns nil if at end-of-input.
  value_t parse_expr();

  /// True if all input has been consumed.
  bool at_end() const;

  std::size_t line() const { return _line; }
  std::size_t col() const { return _col; }

private:
  class depth_guard;

  std::string_view _input;
  parse_options _opts;
  std::size_t _pos = 0;
  std::size_t _line = 1;
  std::size_t _col = 1;
  std::size_t _depth = 0; // lists and quotes open around the current position

  char peek() const;
  char advance();
  void skip_whitespace_and_comments();
  [[noreturn]] void error(const std::string &msg);

  value_t parse_atom();
  value_t parse_list();
  value_t parse_string();
  value_t parse_bytes_string(); // SE-3: b"..." byte-string literal (\xNN = a raw byte, no \u)
  value_t parse_number_or_symbol(char first);
};

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_PARSER_H
