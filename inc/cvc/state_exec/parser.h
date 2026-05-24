/**
 * @file parser.h
 * @brief S-expression parser for the state_exec DSL.
 *
 * Converts source text into a tree of value_t nodes.  Supports atoms
 * (integers, floats, strings, symbols), lists, quote/quasiquote/unquote,
 * and line comments.  Reports parse errors with line/column information.
 */
#ifndef CVC_STATE_EXEC_PARSER_H
#define CVC_STATE_EXEC_PARSER_H

#include <cvc/state_exec/types.h>

#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace cvc::state_exec {

/// Parse error with location information.
class parse_error : public std::runtime_error {
public:
    parse_error(const std::string& msg, std::size_t line, std::size_t col)
        : std::runtime_error(msg), _line(line), _col(col) {}

    std::size_t line() const { return _line; }
    std::size_t col() const { return _col; }

private:
    std::size_t _line;
    std::size_t _col;
};

/// Parse a single S-expression from the input string.
/// Returns the parsed value_t.
/// @throws parse_error on syntax errors.
value_t parse(const std::string& input);

/// Parse all S-expressions from the input string.
/// Returns a vector of parsed value_t values.
/// @throws parse_error on syntax errors.
std::vector<value_t> parse_all(const std::string& input);

/// Internal tokenizer/parser — exposed for testing.
class parser {
public:
    explicit parser(std::string_view input);

    /// Parse one expression.  Returns nil if at end-of-input.
    value_t parse_expr();

    /// True if all input has been consumed.
    bool at_end() const;

    std::size_t line() const { return _line; }
    std::size_t col() const { return _col; }

private:
    std::string_view _input;
    std::size_t _pos = 0;
    std::size_t _line = 1;
    std::size_t _col = 1;

    char peek() const;
    char advance();
    void skip_whitespace_and_comments();
    [[noreturn]] void error(const std::string& msg);

    value_t parse_atom();
    value_t parse_list();
    value_t parse_string();
    value_t parse_number_or_symbol(char first);
};

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_PARSER_H
