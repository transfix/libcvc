// What an Ariadne runtime provides — see the header.

#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/ftxui_backend.h>
#include <cvc/ariadne/host_info.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/net_host.h>
#include <cvc/ariadne/requires.h>
#include <cvc/ariadne/tvision_backend.h>
#include <cvc/net/http_client.h>
#include <sstream>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <io.h>
#include <windows.h>
#elif !defined(__EMSCRIPTEN__)
#include <sys/ioctl.h>
#include <unistd.h>
#endif

namespace cvc {
namespace ariadne {

namespace {

std::string env(const char *name) {
  const char *v = std::getenv(name);
  return v ? v : "";
}

std::string json_string(const std::string &s) {
  std::string out = "\"";
  for (const char c : s) {
    switch (c) {
    case '"':
      out += "\\\"";
      break;
    case '\\':
      out += "\\\\";
      break;
    case '\n':
      out += "\\n";
      break;
    case '\t':
      out += "\\t";
      break;
    default:
      if (static_cast<unsigned char>(c) < 0x20) {
        char buf[8];
        std::snprintf(buf, sizeof buf, "\\u%04x", static_cast<unsigned>(c));
        out += buf;
      } else {
        out += c;
      }
    }
  }
  return out + "\"";
}

const char *yes(bool b) { return b ? "yes" : "no"; }

} // namespace

void describe_terminal(HostInfo &info) {
  info.term = env("TERM");
  info.colorterm = env("COLORTERM");
#if defined(_WIN32)
  info.tty = _isatty(_fileno(stdout)) != 0;
  CONSOLE_SCREEN_BUFFER_INFO csbi;
  if (info.tty && GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &csbi)) {
    info.cols = csbi.srWindow.Right - csbi.srWindow.Left + 1;
    info.rows = csbi.srWindow.Bottom - csbi.srWindow.Top + 1;
  }
#elif !defined(__EMSCRIPTEN__)
  info.tty = isatty(STDOUT_FILENO) != 0;
  struct winsize ws {};
  if (info.tty && ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0) {
    info.cols = ws.ws_col;
    info.rows = ws.ws_row;
  }
#endif
  // The colour depth, from the environment, the way FTXUI decides it.
  if (!info.tty)
    info.colors = "none";
  else if (info.colorterm == "truecolor" || info.colorterm == "24bit" || !env("WT_SESSION").empty())
    info.colors = "truecolor";
  else if (info.term.find("256") != std::string::npos)
    info.colors = "256";
  else if (info.term == "dumb")
    info.colors = "none";
  else
    info.colors = "16";
}

HostInfo gather_host_info(const Runtime &rt, const std::string &runtime) {
  HostInfo info;
  info.runtime = runtime;
  info.libcvc_version = libcvc_version();
  describe_terminal(info);
  info.yaml = have_yaml();
  info.json_schema = have_jsonschema();
  info.ftxui = FtxuiBackend::available();
  info.tvision = TvisionBackend::available();
  info.http = cvc::net::have_http_backend();
  const NetHostStatus net = net_host_status();
  info.net = net.enabled;
  info.net_allowed = net.allowed;
  info.credential_store = net.credential_store;
  info.env_credentials = net.env_credentials;
  const HostFacts facts = rt.host_facts();
  info.graphics.kitty = facts.caps.kitty;
  info.graphics.sixel = facts.caps.sixel;
  for (const CapabilityToken &t : capability_tokens())
    info.capabilities.emplace_back(t.name, capability_holds(t.name, facts));
  info.functions = rt.program_names();
  return info;
}

std::string format_host_info(const HostInfo &info, bool json, bool functions) {
  std::ostringstream o;
  if (json) {
    o << "{\n  \"runtime\": " << json_string(info.runtime)
      << ",\n  \"libcvc\": " << json_string(info.libcvc_version)
      << ",\n  \"backend\": " << json_string(info.backend) << ",\n  \"terminal\": {"
      << "\"tty\": " << (info.tty ? "true" : "false") << ", \"term\": " << json_string(info.term)
      << ", \"colorterm\": " << json_string(info.colorterm) << ", \"cols\": " << info.cols
      << ", \"rows\": " << info.rows << ", \"colors\": " << json_string(info.colors)
      << ", \"probed\": " << (info.probed ? "true" : "false")
      << ", \"kitty\": " << (info.graphics.kitty ? "true" : "false")
      << ", \"sixel\": " << (info.graphics.sixel ? "true" : "false")
      << ", \"iterm\": " << (info.graphics.iterm ? "true" : "false")
      << ", \"sixel_colors\": " << info.graphics.sixel_colors << ", \"cell_px\": ["
      << info.graphics.cell_w << ", " << info.graphics.cell_h << "]},\n"
      << "  \"built_with\": {\"yaml\": " << (info.yaml ? "true" : "false")
      << ", \"json_schema\": " << (info.json_schema ? "true" : "false")
      << ", \"ftxui\": " << (info.ftxui ? "true" : "false")
      << ", \"tvision\": " << (info.tvision ? "true" : "false")
      << ", \"gl\": " << (info.gl ? "true" : "false")
      << ", \"gl_info\": " << json_string(info.gl_info)
      << ", \"http\": " << (info.http ? "true" : "false")
      << "},\n  \"network\": {\"on\": " << (info.net ? "true" : "false") << ", \"allow\": [";
    for (std::size_t i = 0; i < info.net_allowed.size(); ++i)
      o << (i ? ", " : "") << json_string(info.net_allowed[i]);
    o << "], \"credential_store\": " << json_string(info.credential_store)
      << ", \"env_credentials\": [";
    for (std::size_t i = 0; i < info.env_credentials.size(); ++i)
      o << (i ? ", " : "") << json_string(info.env_credentials[i]);
    o << "]},\n  \"capabilities\": {";
    for (std::size_t i = 0; i < info.capabilities.size(); ++i)
      o << (i ? ", " : "") << json_string(info.capabilities[i].first) << ": "
        << (info.capabilities[i].second ? "true" : "false");
    o << "},\n  \"functions\": [";
    for (std::size_t i = 0; i < info.functions.size(); ++i)
      o << (i ? ", " : "") << json_string(info.functions[i]);
    o << "]\n}\n";
    return o.str();
  }
  o << info.runtime << " (libcvc " << info.libcvc_version << ")";
  if (!info.backend.empty())
    o << ", backend " << info.backend;
  o << "\n\nterminal\n";
  if (!info.tty) {
    o << "  stdout is not a terminal\n";
  } else {
    o << "  TERM            " << (info.term.empty() ? "(unset)" : info.term) << "\n";
    if (!info.colorterm.empty())
      o << "  COLORTERM       " << info.colorterm << "\n";
    if (info.cols > 0)
      o << "  size            " << info.cols << " x " << info.rows << " cells\n";
    o << "  colours         " << info.colors << "\n";
    if (info.probed) {
      o << "  kitty graphics  " << yes(info.graphics.kitty) << "\n";
      o << "  Sixel           " << yes(info.graphics.sixel);
      if (info.graphics.sixel && info.graphics.sixel_colors > 0)
        o << " (" << info.graphics.sixel_colors << " colours)";
      o << "\n";
      o << "  iTerm2 images   " << yes(info.graphics.iterm) << "\n";
      if (info.graphics.cell_w > 0)
        o << "  cell            " << info.graphics.cell_w << " x " << info.graphics.cell_h
          << " px\n";
      if (!info.graphics.answered)
        o << "  (the terminal did not answer the graphics query)\n";
    } else {
      o << "  graphics        not asked\n";
    }
  }
  o << "\nbuilt with\n"
    << "  .ari loader (yaml-cpp)        " << yes(info.yaml) << "\n"
    << "  JSON-Schema validation        " << yes(info.json_schema) << "\n"
    << "  terminal backend (FTXUI)      " << yes(info.ftxui) << "\n"
    << "  desktop backend (tvision)     " << yes(info.tvision) << "\n"
    << "  GL rendering                  " << yes(info.gl) << "\n";
  if (!info.gl_info.empty()) {
    std::istringstream lines(info.gl_info);
    for (std::string line; std::getline(lines, line);)
      o << "    " << line << "\n";
  }
  o << "  HTTP (cvc::net)               " << yes(info.http) << "\n";
  o << "\nnetwork\n";
  if (!info.net) {
    o << "  off (--net, --net-allow ORIGIN or --net-policy FILE turn it on)\n";
  } else {
    if (info.net_allowed.empty())
      o << "  on, but no origin is allowed\n";
    for (const std::string &origin : info.net_allowed)
      o << "  allow           " << origin << "\n";
    o << "  credentials     "
      << (info.credential_store.empty() ? std::string("in memory") : info.credential_store) << "\n";
    for (const std::string &name : info.env_credentials)
      o << "  from the env    " << name << "\n";
  }
  o << "\ncapabilities (requires: / has?)\n";
  for (const auto &[name, holds] : info.capabilities) {
    std::string meaning;
    for (const CapabilityToken &t : capability_tokens())
      if (name == t.name)
        meaning = t.meaning;
    std::string pad(name.size() < 18 ? 18 - name.size() : 1, ' ');
    o << "  " << name << pad << (holds ? "yes" : "no ") << "  " << meaning << "\n";
  }
  o << "\nfunctions programs can call: " << info.functions.size();
  if (functions) {
    o << "\n";
    for (const std::string &f : info.functions)
      o << "  " << f << "\n";
  } else {
    o << " (--functions lists them)\n";
  }
  return o.str();
}

} // namespace ariadne
} // namespace cvc
