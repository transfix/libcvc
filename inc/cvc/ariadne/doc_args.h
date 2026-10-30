// Document arguments: a .ari document's own command line.
//
// A runtime (ari, arigl, arisrv) takes only its general flags, before the document path. Every
// argument after the path belongs to the document, as with `python script.py args`. The runtime
// publishes them under the document root before init: runs:
//
//   <root>.argv          read-only; its data is a DSL list of the strings as given, its value
//                        the same strings joined by spaces (for display)
//   <root>.args.<name>   read-only, one per argument the document declares in an `args:` block
//                        and was given or has a default: its value the canonical text, its data
//                        the typed DSL value (string, bool, int or float)
//   <root>.args          read-only; its data is a DSL dict of all of them, name -> typed value
//
// A program reads them with (state-data-get "argv") and (state-get "args.room"). Programs are
// chrooted to their own subtree, so a load:-mounted module sees neither unless its parent grants
// a read-only hole, which grants a node and its subtree: `link: {args: {to: args, mode: ro}}`
// hands the module the dict, (get-attr (state-data-get "args") "room") or (state-get
// "args.room"); `link: {room: {to: args.room, mode: ro}}` hands it one argument, (state-get
// "room").
//
// With an `args:` block the runtime parses argv against it: `--name value`, `--name=value`,
// `--flag` / `--no-flag` for a bool, declared positionals in order, `--` ends the options. An
// unknown option, a malformed value, a missing required argument or an extra positional is an
// error, and `--help` / `-h` asks for the usage format_document_usage builds. Without one, nothing
// is checked and argv is all the document gets.
#pragma once

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
class app;
namespace ariadne {

struct ArgSpec {
  enum class Type { String, Bool, Int, Float };
  std::string name; // `--name` (or the positional's name in usage); the state key args.<name>
  Type type = Type::String;
  std::string help;
  std::string default_value; // canonical text; used when has_default
  bool has_default = false;
  bool required = false;
  bool positional = false; // taken in declaration order instead of as `--name`
};

const char *arg_type_name(ArgSpec::Type type);

// Canonical text for `in` as `type` ("true"/"false" for a bool; an int or float as written, once
// it parses in full). False when `in` is not a value of that type.
bool canonical_arg_value(ArgSpec::Type type, const std::string &in, std::string &out);

// A valid argument name: a letter, then letters, digits, '-' or '_'. Not "help".
bool valid_arg_name(const std::string &name);

struct DocumentArgs {
  bool help = false;               // --help / -h was given
  std::vector<std::string> errors; // one line each; empty = argv matches the spec
  // name -> canonical text, for each declared argument that was given or has a default
  std::vector<std::pair<std::string, std::string>> values;
};

// Parse a document's argv against its args: spec. Never throws.
DocumentArgs parse_document_args(const std::vector<ArgSpec> &spec,
                                 const std::vector<std::string> &argv);

// Usage text for a document: "usage: <invocation> [options] <positionals>", the description, and
// one line per argument with its type, default and help.
std::string format_document_usage(const std::string &invocation, const std::string &description,
                                  const std::vector<ArgSpec> &spec);

// Write argv, and the parsed values when `spec` is given, under `root` (the document prefix), all
// read-only. Replaces whatever an earlier call wrote there, so a reload starts clean.
void publish_document_args(cvc::app &app, const std::string &root,
                           const std::vector<std::string> &argv, const std::vector<ArgSpec> *spec,
                           const DocumentArgs *parsed);

// Thrown by AppRuntime::load when the arguments do not match the document's args: block (`help`
// false; what() lists the errors) or ask for help (`help` true). `usage` is the document's usage
// text either way; a runtime prints it and exits 2 on an error, 0 on help.
class document_args_error : public std::runtime_error {
public:
  document_args_error(const std::string &what, std::string usage, bool help)
      : std::runtime_error(what), usage(std::move(usage)), help(help) {}
  std::string usage;
  bool help;
};

} // namespace ariadne
} // namespace cvc
