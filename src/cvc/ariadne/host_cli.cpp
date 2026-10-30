// The command line every Ariadne runtime shares — see inc/cvc/ariadne/host_cli.h.
//
// boost::program_options reads the runtime's flags: it splits --flag=value and --flag value, knows
// which flags take a value, and rejects one it does not know or one missing its value. Two things
// are this file's own: the first argument that is not a flag (or the one after "--") is the
// document, and it and everything after it are the document's, never the runtime's; and the
// flags' meaning (a repeated flag's last value wins, repeatable ones collect).

#include <algorithm>
#include <boost/program_options.hpp>
#include <charconv>
#include <cstdlib> // strtod (--timeout)
#include <cvc/ariadne/host_cli.h>
#include <cvc/net/net_policy.h>

namespace cvc {
namespace ariadne {

namespace {

namespace po = boost::program_options;

constexpr const char *kValueFlags[] = {"--prefix",     "--component-path",
                                       "--backend",    "--fps",
                                       "--width",      "--height",
                                       "--frames",     "--png",
                                       "--graphics",   "--image",
                                       "--cell-size",  "--net-allow",
                                       "--net-policy", "--credential-store",
                                       "--timeout"};
constexpr const char *kBoolFlags[] = {"--no-mouse", "--ansi", "--net"};

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

// The document and its arguments, as one option with no name: the first argument that is not a
// flag, or the one after "--", and everything after it.
std::vector<po::option> take_document(std::vector<std::string> &args) {
  if (args.empty())
    return {};
  const std::string &a = args.front();
  std::size_t from = 0;
  if (a == "--")
    from = 1;
  else if (!a.empty() && a[0] == '-')
    return {}; // a flag: program_options' own parsers read it
  po::option doc;
  doc.position_key = 0;
  doc.value.assign(args.begin() + static_cast<std::ptrdiff_t>(from), args.end());
  doc.original_tokens = doc.value;
  args.clear();
  if (doc.value.empty())
    return {}; // "--" with nothing after it
  return {doc};
}

po::options_description describe(const std::vector<std::string> &extra_flags) {
  po::options_description desc;
  desc.add_options()("help,h", po::bool_switch());
  for (const char *f : kValueFlags)
    desc.add_options()(f + 2, po::value<std::string>());
  for (const char *f : kBoolFlags)
    desc.add_options()(f + 2, po::bool_switch());
  for (const std::string &f : extra_flags)
    if (f.rfind("--", 0) == 0 && !is_general_flag(f))
      desc.add_options()(f.c_str() + 2, po::bool_switch());
  return desc;
}

// The runtime's flags in `o`, from the parsed options. False with o.error set.
bool apply(const po::option &opt, HostOptions &o) {
  const std::string a = "--" + opt.string_key;
  const std::string value = opt.value.empty() ? std::string() : opt.value.front();
  if (opt.string_key == "help") {
    o.help = true;
  } else if (a == "--no-mouse") {
    o.mouse = false;
  } else if (a == "--ansi") {
    o.ansi = true;
  } else if (a == "--net") {
    o.net.enabled = true;
  } else if (!is_general_flag(a)) {
    o.extra_flags.push_back(a); // one of the runtime's own
  } else if (a == "--fps" || a == "--width" || a == "--height" || a == "--frames") {
    int n = 0;
    if (!to_int(value, n) || n < 0) {
      o.error = a + " needs a whole number, not '" + value + "'";
      return false;
    }
    (a == "--fps" ? o.fps : a == "--width" ? o.width : a == "--height" ? o.height : o.frames) = n;
  } else if (a == "--timeout") {
    char *end = nullptr;
    const double t = std::strtod(value.c_str(), &end);
    if (value.empty() || end != value.c_str() + value.size() || !(t > 0)) {
      o.error = "--timeout needs a number of seconds above 0, not '" + value + "'";
      return false;
    }
    o.timeout = t;
  } else if (a == "--prefix") {
    o.prefix = value;
  } else if (a == "--component-path") {
    o.component_paths.push_back(value);
  } else if (a == "--backend") {
    o.backend = value;
  } else if (a == "--png") {
    o.png = value;
  } else if (a == "--graphics") {
    if (value != "auto" && value != "kitty" && value != "sixel" && value != "iterm" &&
        value != "color" && value != "mono") {
      o.error = "--graphics is auto, kitty, sixel, iterm, color or mono, not '" + value + "'";
      return false;
    }
    o.graphics = value;
  } else if (a == "--image") {
    const std::size_t at = value.find('=');
    if (at == std::string::npos || at == 0 || at + 1 == value.size()) {
      o.error = "--image is NAME=FILE, not '" + value + "'";
      return false;
    }
    o.images.emplace_back(value.substr(0, at), value.substr(at + 1));
  } else if (a == "--cell-size") {
    const std::size_t x = value.find('x');
    int w = 0, h = 0;
    if (x == std::string::npos || !to_int(value.substr(0, x), w) ||
        !to_int(value.substr(x + 1), h) || w <= 0 || h <= 0) {
      o.error = "--cell-size is WxH in pixels, not '" + value + "'";
      return false;
    }
    o.cell_w = w;
    o.cell_h = h;
  } else if (a == "--net-allow") {
    std::string why;
    if (!cvc::net::parse_origin_spec(value, &why)) {
      o.error = "--net-allow: " + why;
      return false;
    }
    o.net.allow.push_back(value);
    o.net.enabled = true;
  } else if (a == "--net-policy") {
    o.net.policy_files.push_back(value);
    o.net.enabled = true;
  } else if (a == "--credential-store") {
    if (value.empty()) {
      o.error = "--credential-store needs a name or a path";
      return false;
    }
    o.net.credential_store = value;
    o.net.enabled = true;
  }
  return true;
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
  const std::vector<std::string> args(argv + 2, argv + argc);
  namespace style = po::command_line_style;
  po::parsed_options parsed(nullptr);
  try {
    parsed = po::command_line_parser(args)
                 .options(describe(extra_flags))
                 .style(style::allow_long | style::long_allow_adjacent | style::long_allow_next |
                        style::allow_short | style::short_allow_next | style::allow_dash_for_short)
                 .extra_style_parser(take_document)
                 .run();
  } catch (const po::unknown_option &e) {
    o.error =
        "unknown option '" + e.get_option_name() + "' (a document's own options go after its path)";
    return o;
  } catch (const po::invalid_command_line_syntax &e) {
    const std::string name = e.get_option_name();
    if (e.kind() == po::invalid_command_line_syntax::missing_parameter)
      o.error = name + " needs a value";
    else if (e.kind() == po::invalid_command_line_syntax::extra_parameter)
      o.error = name + " takes no value";
    else
      o.error = e.what();
    return o;
  } catch (const po::error &e) {
    o.error = e.what();
    return o;
  }
  std::vector<std::string> positional; // the document, then its own arguments
  for (const po::option &opt : parsed.options) {
    if (opt.string_key.empty()) { // program_options may hand them back one option each
      positional.insert(positional.end(), opt.value.begin(), opt.value.end());
      continue;
    }
    if (!apply(opt, o))
      return o;
  }
  if (!positional.empty()) {
    o.doc = positional.front();
    o.doc_args.assign(positional.begin() + 1, positional.end());
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
         "  --frames N             snapshot: frames to run before the capture; test: stop\n"
         "                         after N frames\n"
         "  --timeout S            test: fail a document still running after S seconds\n"
         "  --ansi                 snapshot: keep colour escapes\n"
         "  --png FILE             snapshot: also write a PNG\n"
         "  --graphics MODE        images in a terminal: auto, kitty, sixel, iterm, color or mono\n"
         "  --image NAME=FILE      publish FILE's picture as NAME, for image: widgets\n"
         "  --cell-size WxH        snapshot: a terminal cell's size in pixels\n"
         "  --net                  let the document use the network (off by default); in a\n"
         "                         browser it may reach the page's own origin\n"
         "  --net-allow ORIGIN     ... and reach ORIGIN, scheme://host[:port] (repeatable)\n"
         "  --net-policy FILE      ... with the allow and credential lines in FILE (repeatable)\n"
         "  --credential-store S   keep sign-in credentials across runs in S: a name in the\n"
         "                         user's config dir, or a path (not in a browser)\n";
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
