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
  /// How many lists and quotes may nest (`'x` reads as `(quote x)`, one level); deeper input is
  /// a parse_error. The parser is stackless, and so is what walks parsed data in libcvc
  /// (freeing, printing, comparing, copying, macro expansion, the snapshot codec, the Ariadne
  /// channel lint), each tested 100000 levels deep on a 64 KiB stack. So the default is that
  /// depth: a bound on runaway input, not a guard for the native stack. (Values crossing into
  /// Python go through pycvc's converter, which has its own depth limit.)
  /// State-tree key: `parser.max_depth`.
  std::size_t max_depth = 100000;

  /// How a scheduler parses a submitted script. 0 parses all of it when it is submitted, so
  /// execute() throws a parse_error. N > 0 parses it inside the new process, N tokens per
  /// scheduler step, before evaluation starts. Those steps count against the process's max_steps
  /// and max_time, and a syntax error then surfaces like a runtime error. State-tree key:
  /// `parser.step_tokens`.
  std::size_t step_tokens = 0;
};

/// Resolve parser settings from a state tree, per key: `state_exec.schedulers.<scheduler_id>.
/// parser.<key>` (when scheduler_id is not empty), then `state_exec.defaults.parser.<key>`, then
/// `base` (a host's built-in defaults; parse_options{} unless it has its own). A value that is
/// missing, empty, malformed or out of range is skipped; max_depth must also be positive.
parse_options parse_options_from_state(cvc::state &root, const std::string &scheduler_id = {},
                                       const parse_options &base = {});

/// Parse a single S-expression from the input string.
/// Returns the parsed value_t.
/// @throws parse_error on syntax errors.
value_t parse(const std::string &input, const parse_options &opts = {});

/// Parse all S-expressions from the input string.
/// Returns a vector of parsed value_t values.
/// @throws parse_error on syntax errors.
std::vector<value_t> parse_all(const std::string &input, const parse_options &opts = {});

/// Internal tokenizer/parser — exposed for testing.
///
/// Stackless: lists and quotes are tracked on a heap stack of open frames, not by native
/// recursion, so the native stack depth is constant however deeply the input nests. Reading is
/// also resumable (read() with a token budget), which lets a scheduler parse a script a slice at
/// a time inside a process's steps.
class parser {
public:
  explicit parser(std::string_view input, const parse_options &opts = {});

  /// Parse one expression.  Returns nil if at end-of-input.
  value_t parse_expr();

  /// What read() stopped at.
  enum class read_status {
    datum, ///< a top-level datum is complete; it is in `out`
    end,   ///< only whitespace and comments remain; nothing is open
    more,  ///< the token budget ran out; call read() again to continue
  };

  /// Read until a top-level datum completes, the input ends, or `budget` tokens (parens, quotes,
  /// atoms, strings) have been read; `budget` is decremented by the tokens read. Open lists and
  /// quotes carry over between calls.
  /// @throws parse_error on syntax errors, including nesting past parse_options::max_depth.
  read_status read(value_t &out, std::size_t &budget);

  /// True if all input has been consumed.
  bool at_end() const;

  std::size_t line() const { return _line; }
  std::size_t col() const { return _col; }

private:
  /// An open list (`(` read, `)` not yet) or quote (`'` read, its datum not yet complete).
  struct frame {
    bool quote = false;
    std::vector<value_t> elements; // a list's elements so far
  };

  std::string_view _input;
  parse_options _opts;
  std::size_t _pos = 0;
  std::size_t _line = 1;
  std::size_t _col = 1;
  std::vector<frame> _frames; // innermost last; its size is the current nesting depth
  // Element vectors of closed lists, emptied, for the next lists to collect into: a list is
  // then made with one allocation of its exact size, not grown an element at a time.
  std::vector<std::vector<value_t>> _spare_elements;

  char peek() const;
  char advance();
  void skip_whitespace_and_comments();
  [[noreturn]] void error(const std::string &msg);
  void open_frame(bool quote);

  value_t parse_atom();
  value_t parse_string();
  value_t parse_bytes_string(); // SE-3: b"..." byte-string literal (\xNN = a raw byte, no \u)
  value_t parse_number_or_symbol(char first);
  // Append the input from here up to `stop` (no quote or escape in it) to `into`, moving past it.
  void skip_run(std::size_t stop, std::string &into);
};

/// Reads a whole script a slice at a time. A scheduler drives it from a process's steps when
/// parse_options::step_tokens is set. It owns its copy of the source, which its parser views, so
/// it can be neither copied nor moved; hold it by pointer.
class script_reader {
public:
  script_reader(std::string source, const parse_options &opts);
  script_reader(const script_reader &) = delete;
  script_reader &operator=(const script_reader &) = delete;

  /// Read up to `budget` more tokens. Returns true once the whole script has been read; forms()
  /// then holds every top-level datum, as parse_all() would return them.
  /// @throws parse_error on syntax errors.
  bool step(std::size_t budget);

  std::vector<value_t> &forms() { return _forms; }
  const std::string &source() const { return _source; }
  const parse_options &options() const { return _opts; }

private:
  std::string _source; // declared before _parser, which views it
  parse_options _opts;
  parser _parser;
  std::vector<value_t> _forms;
};

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_PARSER_H
