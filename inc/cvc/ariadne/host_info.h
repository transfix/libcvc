#ifndef CVC_ARIADNE_HOST_INFO_H
#define CVC_ARIADNE_HOST_INFO_H
// What an Ariadne runtime provides, as `ari info` / `arigl info` print it: the terminal it runs in
// (size, colours, kitty / Sixel graphics), what libcvc was built with, which capability tokens hold
// (requires.h), and every function a document's programs can call.
//
// gather_host_info() fills what libcvc can know from a Runtime: its backend's Capabilities (for a
// terminal backend, after the host has probed the terminal and handed the answer to
// FtxuiBackend::set_terminal_graphics), the build's features and the program env. A runtime adds
// what only it knows (arigl: whether it has a GL renderer) before formatting.
#include <cvc/ariadne/term_graphics.h>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace ariadne {

class Runtime;

struct HostInfo {
  std::string runtime; // "ari", "arigl --terminal", ...
  std::string libcvc_version;
  std::string backend; // "ftxui", "tvision", "imgui", ...

  // The terminal, when stdout is one.
  bool tty = false;
  std::string term, colorterm; // $TERM, $COLORTERM
  int cols = 0, rows = 0;      // 0 = unknown
  std::string colors;          // "truecolor", "256", "16" or "none", from the environment
  TermGraphics graphics;       // what the terminal answered (all false when not probed)
  bool probed = false;         // the terminal was asked (graphics is its answer)

  // What libcvc was built with.
  bool yaml = false, json_schema = false, ftxui = false, tvision = false;
  bool gl = false;     // a GL renderer is available to this runtime (the runtime sets it)
  std::string gl_info; // what it is: vendor, renderer and version (the runtime sets it)
  bool http = false;   // an HTTP backend (cvc::net: libcurl, or the browser's fetch)

  // The network the host gave its documents (net_host.h).
  bool net = false;
  std::vector<std::string> net_allowed; // the origins requests may reach
  std::string credential_store;         // where credentials are kept ("" = in memory)
  std::vector<std::string> env_credentials; // names --credential-env supplied a token for

  std::vector<std::pair<std::string, bool>> capabilities; // every token, and whether it holds
  std::vector<std::string> functions;                     // what programs can call, sorted
};

// The terminal on stdout: whether it is one, its size, and its colour depth from the environment.
// Fills tty, term, colorterm, cols, rows and colors.
void describe_terminal(HostInfo &info);

// Everything libcvc knows about this host: the terminal (describe_terminal), the build features,
// the capability tokens as `rt` answers them, and its program names.
HostInfo gather_host_info(const Runtime &rt, const std::string &runtime);

// A report: readable text, or JSON (one object). `functions` lists every function name (the text
// form otherwise prints only their count).
std::string format_host_info(const HostInfo &info, bool json, bool functions);

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_HOST_INFO_H
