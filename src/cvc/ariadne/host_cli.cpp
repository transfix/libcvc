// The command line every Ariadne runtime shares — see inc/cvc/ariadne/host_cli.h.

#include <algorithm>
#include <charconv>
#include <cvc/ariadne/host_cli.h>

namespace cvc {
namespace ariadne {

namespace {

constexpr const char *kValueFlags[] = {"--prefix",   "--component-path", "--backend",  "--fps",
                                       "--width",    "--height",         "--frames",   "--png",
                                       "--graphics", "--image",          "--cell-size"};
constexpr const char *kBoolFlags[] = {"--no-mouse", "--ansi"};

bool is_general_flag(const std::string &a) {
  const std::string name = a.substr(0, a.find('='));
  for (const char *f : kValueFlags)
    if (name == f)
      return true;
  for (const char *f : kBoolFlags)
    if (name == f)
      return true;
  return false;
}

bool to_int(const std::string &s, int &out) {
  const auto [end, ec] = std::from_chars(s.data(), s.data() + s.size(), out);
  return !s.empty() && ec == std::errc() && end == s.data() + s.size();
}

} // namespace

HostOptions parse_host_args(int argc, const char *const *argv,
                            const std::vector<std::string> &extra_flags) {
  HostOptions o;
  if (argc < 2) {
    o.error = "no command given";
    return o;
  }
  o.command = argv[1];
  if (o.command == "-h" || o.command == "--help" || o.command == "help") {
    o.help = true;
    return o;
  }
  for (int i = 2; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--") { // the next argument is the document, even if it starts with '-'
      if (i + 1 < argc) {
        o.doc = argv[i + 1];
        for (int j = i + 2; j < argc; ++j)
          o.doc_args.emplace_back(argv[j]);
      }
      break;
    }
    if (a.empty() || a[0] != '-') { // the document: the rest is its own
      o.doc = a;
      for (int j = i + 1; j < argc; ++j)
        o.doc_args.emplace_back(argv[j]);
      break;
    }
    if (a == "-h" || a == "--help") {
      o.help = true;
      continue;
    }
    if (std::find(extra_flags.begin(), extra_flags.end(), a) != extra_flags.end()) {
      o.extra_flags.push_back(a);
      continue;
    }
    if (a == "--no-mouse") {
      o.mouse = false;
      continue;
    }
    if (a == "--ansi") {
      o.ansi = true;
      continue;
    }
    std::string value;
    const std::size_t eq = a.find('=');
    if (eq != std::string::npos) {
      value = a.substr(eq + 1);
      a.resize(eq);
    }
    if (!is_general_flag(a)) {
      o.error = "unknown option '" + a + "' (a document's own options go after its path)";
      return o;
    }
    if (eq == std::string::npos) {
      if (i + 1 >= argc) {
        o.error = a + " needs a value";
        return o;
      }
      value = argv[++i];
    }
    int n = 0;
    const bool numeric = a == "--fps" || a == "--width" || a == "--height" || a == "--frames";
    if (numeric && (!to_int(value, n) || n < 0)) {
      o.error = a + " needs a whole number, not '" + value + "'";
      return o;
    }
    if (a == "--prefix")
      o.prefix = value;
    else if (a == "--component-path")
      o.component_paths.push_back(value);
    else if (a == "--backend")
      o.backend = value;
    else if (a == "--png")
      o.png = value;
    else if (a == "--graphics") {
      if (value != "auto" && value != "kitty" && value != "sixel" && value != "color" &&
          value != "mono") {
        o.error = "--graphics is auto, kitty, sixel, color or mono, not '" + value + "'";
        return o;
      }
      o.graphics = value;
    } else if (a == "--image") {
      const std::size_t at = value.find('=');
      if (at == std::string::npos || at == 0 || at + 1 == value.size()) {
        o.error = "--image is NAME=FILE, not '" + value + "'";
        return o;
      }
      o.images.emplace_back(value.substr(0, at), value.substr(at + 1));
    } else if (a == "--cell-size") {
      const std::size_t x = value.find('x');
      int w = 0, h = 0;
      if (x == std::string::npos || !to_int(value.substr(0, x), w) ||
          !to_int(value.substr(x + 1), h) || w <= 0 || h <= 0) {
        o.error = "--cell-size is WxH in pixels, not '" + value + "'";
        return o;
      }
      o.cell_w = w;
      o.cell_h = h;
    } else if (a == "--fps")
      o.fps = n;
    else if (a == "--width")
      o.width = n;
    else if (a == "--height")
      o.height = n;
    else if (a == "--frames")
      o.frames = n;
  }
  if (o.doc.empty() && !o.help && o.command != "info") // info describes the host, not a document
    o.error = "no document given";
  return o;
}

std::string host_flags_help() {
  return "  --prefix P             the document's state root\n"
         "  --component-path DIR   a cvc:// component search dir (repeatable)\n"
         "  --backend NAME         the terminal backend: ftxui or tvision\n"
         "  --fps N                the interactive loop's frame rate\n"
         "  --no-mouse             no mouse input\n"
         "  --width W, --height H  snapshot size\n"
         "  --frames N             snapshot: frames to run before the capture\n"
         "  --ansi                 snapshot: keep colour escapes\n"
         "  --png FILE             snapshot: also write a PNG\n"
         "  --graphics MODE        images in a terminal: auto, kitty, sixel, color or mono\n"
         "  --image NAME=FILE      publish FILE's picture as NAME, for image: widgets\n"
         "  --cell-size WxH        snapshot: a terminal cell's size in pixels\n";
}

std::string misplaced_flag_note(const std::string &runtime,
                                const std::vector<std::string> &doc_args) {
  for (const std::string &a : doc_args) {
    if (a == "--")
      break;
    if (is_general_flag(a))
      return runtime + ": note: '" + a.substr(0, a.find('=')) +
             "' comes after the document, so the document gets it, not " + runtime + "; put " +
             runtime + "'s own flags before the document path";
  }
  return {};
}

} // namespace ariadne
} // namespace cvc
