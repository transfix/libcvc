// requires: — see the header. The capability-token table and the free-head scan that derives what a
// program calls; AppRuntime::load applies both.

#include <cvc/ariadne/requires.h>
#include <cvc/state/state_exec/parser.h>
#include <cvc/state/state_exec/stackless_evaluator.h>
#include <cvc/state/state_exec/types.h>
#include <map>
#include <mutex>
#include <set>
#include <unordered_set>

namespace cvc {
namespace ariadne {

namespace {

// The token table: name, meaning, and how the host answers it.
struct TokenRule {
  CapabilityToken token;
  bool (*holds)(const HostFacts &);
};

const std::vector<TokenRule> &rules() {
  static const std::vector<TokenRule> kRules = {
      // GLSL runs in two places, and a host can have one without the other: arigl --terminal
      // renders the scene offscreen through GL (its shaders run) but draws the UI as text cells.
      {{"gl.scene", "scene: blocks are realized (a scene adapter is present)"},
       [](const HostFacts &f) { return f.scene; }},
      {{"gl.scene.shaders", "the scene runs the document's GLSL (a node's shader: block)"},
       [](const HostFacts &f) { return f.scene_shaders; }},
      {{"gl.ui.shaders", "the UI itself draws GLSL (shader_canvas): a GL window, not a terminal"},
       [](const HostFacts &f) { return f.caps.glsl; }},
      {{"gl.shaders", "both gl.scene.shaders and gl.ui.shaders (name the one you need)"},
       [](const HostFacts &f) { return f.scene_shaders && f.caps.glsl; }},
      {{"gl.view", "a live render view can be embedded in a widget"},
       [](const HostFacts &f) { return f.caps.view_embed; }},
      {{"ui.windows", "real movable windows (free layout)"},
       [](const HostFacts &f) { return f.caps.windows; }},
      {{"ui.menubar", "a menu bar"}, [](const HostFacts &f) { return f.caps.menubar; }},
      {{"ui.color", "coloured styling"}, [](const HostFacts &f) { return f.caps.color; }},
      {{"input.mouse", "pointer input"}, [](const HostFacts &f) { return f.caps.mouse; }},
      {{"input.keyboard", "key and text input"},
       [](const HostFacts &f) { return f.caps.keyboard; }},
      {{"raster.bitmap", "images in real pixels (a GL window, or a kitty or Sixel terminal)"},
       [](const HostFacts &f) { return f.caps.bitmap; }},
      {{"term", "the document is drawn in a text terminal"},
       [](const HostFacts &f) { return f.caps.terminal; }},
      {{"term.kitty", "the terminal speaks the kitty graphics protocol"},
       [](const HostFacts &f) { return f.caps.kitty; }},
      {{"term.sixel", "the terminal draws DEC Sixel"},
       [](const HostFacts &f) { return f.caps.sixel; }},
      {{"net", "the host turned the network on: http-get, http-request and http(s) URIs reach "
               "the origins it allows"},
       [](const HostFacts &f) { return f.net; }},
      {{"net.auth", "the host keeps credentials for requests (auth:, capture_auth:)"},
       [](const HostFacts &f) { return f.net_auth; }},
      {{"net.auth.persist", "the host keeps credentials across runs"},
       [](const HostFacts &f) { return f.net_auth_persist; }},
      {{"image.url", "image: shows a picture by URI (§5.11): http(s) where the network is on, "
                     "file://, cvc://, state://"},
       [](const HostFacts &) { return true; }},
      {{"image.background", "a window, overlay or group draws a background: picture (§5.12)"},
       [](const HostFacts &) { return true; }},
      {{"image.animated", "image: and background: animate a GIF or WebP (APNG: its first frame)"},
       [](const HostFacts &) { return true; }},
      {{"video.decorative", "video: takes controls: false, and a video URL in image: or "
                            "background: plays as a decorative video (with a player; else its "
                            "poster or placeholder)"},
       [](const HostFacts &) { return true; }},
      {{"media", "video: and audio: play here (a player, and not an SSH session it would play "
                 "remotely from, unless --media-play always)"},
       [](const HostFacts &f) { return f.media; }},
      {{"media.player",
        "the host has a player for video: and audio: (ari-media, arigl, a browser)"},
       [](const HostFacts &f) { return f.media_player; }},
      {{"oidc", "the host signs in with OAuth 2.0 / OpenID Connect providers (oidc-sign-in)"},
       [](const HostFacts &f) { return f.oidc; }},
      {{"oidc.loopback", "sign-in through the system browser and a loopback redirect (RFC 8252)"},
       [](const HostFacts &f) { return f.oidc_loopback; }},
      {{"oidc.device", "sign-in with a code entered on another device (RFC 8628)"},
       [](const HostFacts &f) { return f.oidc_device; }},
      {{"oidc.redirect", "sign-in by sending the browser page to the provider and back"},
       [](const HostFacts &f) { return f.oidc_redirect; }},
      {{"browser", "open-url opens the user's web browser here (not over SSH or --no-browser)"},
       [](const HostFacts &f) { return f.browser; }},
      // A script language's tokens answer from the engine a scripting host registered
      // (set_script_engine), not from HostFacts; the language name stands for itself here.
      {{"python", "an embedded Python runs the document's python: block"},
       [](const HostFacts &) { return has_script_engine("python"); }},
      {{"python.<module>", "<module> is importable in the embedded Python (python.numpy, ...)"},
       [](const HostFacts &) { return false; }},
  };
  return kRules;
}

// The script engines the host embeds, by language (set_script_engine).
std::mutex &engines_mutex() {
  static std::mutex m;
  return m;
}
std::map<std::string, std::function<bool(const std::string &)>> &engines() {
  static std::map<std::string, std::function<bool(const std::string &)>> e;
  return e;
}

// The script languages a document can ask for, and the hosts that run each.
struct ScriptLanguage {
  const char *name;
  const char *hosts;
};
constexpr ScriptLanguage kScriptLanguages[] = {{"python", "arix or ariglx"}};

// `name`'s language when it is "<language>.<module>" with a module, else nullptr.
const ScriptLanguage *module_token_language(const std::string &name, std::string *module) {
  for (const ScriptLanguage &l : kScriptLanguages) {
    const std::string prefix = std::string(l.name) + ".";
    if (name.size() > prefix.size() && name.compare(0, prefix.size(), prefix) == 0) {
      if (module)
        *module = name.substr(prefix.size());
      return &l;
    }
  }
  return nullptr;
}

const TokenRule *find_rule(const std::string &name) {
  for (const TokenRule &r : rules())
    if (name == r.token.name)
      return &r;
  return nullptr;
}

namespace se = cvc::state_exec;

// The free-head walk. `bound` holds the names visible as bindings where an expression sits;
// `defined` collects names the program binds anywhere at top level (defun, set, defclass,
// defmacro), which are subtracted at the end because a call may precede its definition in source
// order.
class HeadScan {
public:
  void scan(const se::value_t &v, const std::set<std::string> &bound, int depth) {
    if (depth > kMaxDepth)
      return; // a pathological nesting: stop (the scan only ever warns)
    const auto *lp = std::get_if<se::list_ptr>(&v.v);
    if (!lp || !*lp || (*lp)->empty())
      return;
    const std::vector<se::value_t> &l = **lp;
    const auto *head = std::get_if<se::symbol>(&l[0].v);
    if (!head) { // ((lambda ...) args), or a list of data
      for (const se::value_t &e : l)
        scan(e, bound, depth + 1);
      return;
    }
    const std::string &h = head->name;
    if (h == "quote")
      return; // data, never called
    if (h == "lambda") {
      std::set<std::string> b = bound;
      bind_params(l, 1, b);
      scan_from(l, 2, b, depth);
      return;
    }
    if (h == "defun" || h == "defmacro") {
      std::set<std::string> b = bound;
      if (const std::string *n = symbol_at(l, 1)) {
        defined_.insert(*n);
        b.insert(*n);
      }
      bind_params(l, 2, b);
      scan_from(l, 3, b, depth);
      return;
    }
    if (h == "let") {
      std::set<std::string> b = bound;
      if (l.size() > 1)
        if (const auto *bl = std::get_if<se::list_ptr>(&l[1].v); bl && *bl)
          for (const se::value_t &binding : **bl)
            if (const auto *bp = std::get_if<se::list_ptr>(&binding.v);
                bp && *bp && !(*bp)->empty()) {
              if ((*bp)->size() > 1)
                scan((**bp)[1], b, depth + 1); // each value sees the bindings before it
              if (const auto *n = std::get_if<se::symbol>(&(**bp)[0].v))
                b.insert(n->name);
            }
      scan_from(l, 2, b, depth);
      return;
    }
    if (h == "for") {
      if (l.size() > 2)
        scan(l[2], bound, depth + 1); // the collection
      std::set<std::string> b = bound;
      if (const std::string *n = symbol_at(l, 1))
        b.insert(*n);
      scan_from(l, 3, b, depth);
      return;
    }
    if (h == "set") {
      if (const std::string *n = symbol_at(l, 1))
        defined_.insert(*n);
      scan_from(l, 2, bound, depth);
      return;
    }
    if (h == "defclass") { // (defclass Name [Parent] (method (self args...) body...)...)
      std::set<std::string> b = bound;
      if (const std::string *n = symbol_at(l, 1)) {
        defined_.insert(*n);
        b.insert(*n);
      }
      for (std::size_t i = 2; i < l.size(); ++i) {
        const auto *mp = std::get_if<se::list_ptr>(&l[i].v);
        if (!mp || !*mp || (*mp)->empty())
          continue; // the parent class: a value, not a call
        std::set<std::string> mb = b;
        bind_params(**mp, 1, mb);
        scan_from(**mp, 2, mb, depth + 1);
      }
      return;
    }
    if (h == "try") { // (try body... (catch (Type var) handler...)... (finally cleanup...))
      for (std::size_t i = 1; i < l.size(); ++i) {
        const auto *cp = std::get_if<se::list_ptr>(&l[i].v);
        const se::symbol *ch =
            cp && *cp && !(*cp)->empty() ? std::get_if<se::symbol>(&(**cp)[0].v) : nullptr;
        if (ch && ch->name == "catch") {
          // (Type var), (Type) or (var): bind every name in the spec. A lone symbol may be a class
          // or a variable; binding it can only hide a call, never invent one.
          std::set<std::string> b = bound;
          bind_params(**cp, 1, b);
          scan_from(**cp, 2, b, depth + 1);
        } else if (ch && ch->name == "finally") {
          scan_from(**cp, 1, bound, depth + 1);
        } else {
          scan(l[i], bound, depth + 1);
        }
      }
      return;
    }
    if (!se::stackless_evaluator::all_special_forms().count(h) && !bound.count(h) &&
        seen_.insert(h).second)
      heads_.push_back(h);
    scan_from(l, 1, bound, depth);
  }

  std::vector<std::string> result() const {
    std::vector<std::string> out;
    for (const std::string &h : heads_)
      if (!defined_.count(h))
        out.push_back(h);
    return out;
  }

private:
  static constexpr int kMaxDepth = 4096;

  static const std::string *symbol_at(const std::vector<se::value_t> &l, std::size_t i) {
    if (i >= l.size())
      return nullptr;
    const auto *s = std::get_if<se::symbol>(&l[i].v);
    return s ? &s->name : nullptr;
  }
  // Bind the symbols of the parameter list at l[i] ((a b &rest c) — &rest is not a name).
  static void bind_params(const std::vector<se::value_t> &l, std::size_t i,
                          std::set<std::string> &b) {
    if (i >= l.size())
      return;
    if (const auto *pl = std::get_if<se::list_ptr>(&l[i].v); pl && *pl)
      for (const se::value_t &p : **pl)
        if (const auto *ps = std::get_if<se::symbol>(&p.v); ps && ps->name != "&rest")
          b.insert(ps->name);
  }
  void scan_from(const std::vector<se::value_t> &l, std::size_t from,
                 const std::set<std::string> &bound, int depth) {
    for (std::size_t i = from; i < l.size(); ++i)
      scan(l[i], bound, depth + 1);
  }

  std::vector<std::string> heads_;
  std::unordered_set<std::string> seen_;
  std::unordered_set<std::string> defined_;
};

} // namespace

const std::vector<CapabilityToken> &capability_tokens() {
  static const std::vector<CapabilityToken> kTokens = [] {
    std::vector<CapabilityToken> t;
    for (const TokenRule &r : rules())
      t.push_back(r.token);
    return t;
  }();
  return kTokens;
}

bool is_capability_token(const std::string &name) {
  return find_rule(name) != nullptr || module_token_language(name, nullptr) != nullptr;
}

bool capability_holds(const std::string &name, const HostFacts &facts) {
  std::string module;
  if (const ScriptLanguage *l = module_token_language(name, &module)) {
    std::function<bool(const std::string &)> has_module;
    {
      std::lock_guard<std::mutex> lock(engines_mutex());
      auto it = engines().find(l->name);
      if (it != engines().end())
        has_module = it->second;
    }
    return has_module && has_module(module); // outside the lock: it may take the engine's own
  }
  const TokenRule *r = find_rule(name);
  return r && r->holds(facts);
}

void set_script_engine(const std::string &language,
                       std::function<bool(const std::string &module)> has_module) {
  std::lock_guard<std::mutex> lock(engines_mutex());
  if (has_module)
    engines()[language] = std::move(has_module);
  else
    engines().erase(language);
}

bool has_script_engine(const std::string &language) {
  std::lock_guard<std::mutex> lock(engines_mutex());
  return engines().count(language) != 0;
}

std::string capability_hint(const std::string &name) {
  for (const ScriptLanguage &l : kScriptLanguages)
    if ((name == l.name || module_token_language(name, nullptr) == &l) &&
        !has_script_engine(l.name))
      return std::string("a document's ") + l.name + " runs in the scripting hosts, " + l.hosts;
  return {};
}

std::vector<std::string> free_heads(const std::string &program) {
  const std::size_t i = program.find_first_not_of(" \t\r\n");
  if (i == std::string::npos || program[i] != '(')
    return {}; // a bare event name or an empty program calls nothing
  std::vector<se::value_t> forms;
  try {
    forms = se::parse_all(program);
  } catch (...) {
    return {}; // its parse error surfaces when it runs
  }
  HeadScan scan;
  const std::set<std::string> none;
  for (const se::value_t &f : forms)
    scan.scan(f, none, 0);
  return scan.result();
}

} // namespace ariadne
} // namespace cvc
