// The command line every Ariadne runtime shares (ari, arigl, arisrv):
//
//   <runtime> <command> [general flags] doc.ari [document args...]
//
// `<runtime> info [flags]` takes no document: it reports what the host provides (host_info.h).
//
// The runtime takes only the general flags below, and only before the document path. Everything
// after the path belongs to the document (doc_args.h), as with `python script.py args`: the
// document reads it as argv, or declares an args: block the runtime parses it against. A runtime
// may add a few boolean flags of its own (arigl's --terminal) through `extra_flags`.
//
//   --prefix P            the document's state root
//   --component-path DIR  a cvc:// component search dir (repeatable)
//   --backend NAME        the terminal backend: ftxui or tvision
//   --fps N               the interactive loop's frame rate
//   --no-mouse            no mouse input
//   --width W, --height H snapshot size (cells in a terminal, pixels in a window)
//   --frames N            snapshot: frames to run before the capture
//   --ansi                snapshot: keep colour escapes
//   --png FILE            snapshot: also write a PNG (where the runtime can render one)
//   --graphics MODE       how images draw in a terminal: auto, kitty, sixel, color or mono
//   --image NAME=FILE     publish the picture in FILE under NAME, for image: widgets (repeatable)
//   --cell-size WxH       snapshot: a terminal cell's size in pixels, where it can't be asked
//
// boost::program_options reads them: --flag VALUE and --flag=VALUE both work, and an unknown flag
// or one missing its value is an error.
#pragma once

#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace ariadne {

struct HostOptions {
  std::string command;               // argv[1] (the runtime decides which it accepts)
  std::string doc;                   // the document path
  std::vector<std::string> doc_args; // everything after it
  std::string prefix;                // empty = the runtime's default
  std::vector<std::string> component_paths;
  std::string backend; // empty = the runtime's default
  int fps = 0;         // 0 = the backend's default
  bool mouse = true;
  int width = 0, height = 0, frames = 0; // 0 = the runtime's default
  bool ansi = false;
  std::string png;
  std::string graphics; // empty = auto (FtxuiBackend::Options::ImageColor)
  std::vector<std::pair<std::string, std::string>> images; // --image NAME=FILE, in order
  int cell_w = 0, cell_h = 0;                              // --cell-size; 0 = ask the terminal
  std::vector<std::string> extra_flags; // the runtime's own flags that were given
  bool help = false;                    // -h / --help, or `help` as the command
  std::string error;                    // a usage error: print it with the usage and exit 2
};

// Parse argv. `extra_flags` names the runtime's own boolean flags (e.g. {"--terminal"}). Never
// throws; a problem is reported in `error`.
HostOptions parse_host_args(int argc, const char *const *argv,
                            const std::vector<std::string> &extra_flags = {});

// The general flags' help lines, for a runtime's usage text.
std::string host_flags_help();

// For a document that declares no args: block: a note naming any of its arguments that is a
// general flag, which was probably meant for the runtime ("" when there is none).
std::string misplaced_flag_note(const std::string &runtime,
                                const std::vector<std::string> &doc_args);

} // namespace ariadne
} // namespace cvc
