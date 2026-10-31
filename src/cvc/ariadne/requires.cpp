// requires: — see the header. The capability-token table and the free-head scan that derives what a
// program calls; AppRuntime::load applies both.

#include <cvc/ariadne/requires.h>
#include <cvc/state/state_exec/parser.h>
#include <cvc/state/state_exec/stackless_evaluator.h>
#include <cvc/state/state_exec/types.h>
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
  };
  return kRules;
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

bool is_capability_token(const std::string &name) { return find_rule(name) != nullptr; }

bool capability_holds(const std::string &name, const HostFacts &facts) {
  const TokenRule *r = find_rule(name);
  return r && r->holds(facts);
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
