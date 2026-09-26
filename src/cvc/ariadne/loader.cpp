// Ariadne — the .ari YAML loader (see the header). Parses a .ari document into a
// cvc::ariadne Widget tree. Pure libcvc: it produces the backend-neutral tree,
// unaware of any UI toolkit. Optional yaml-cpp dependency — when libcvc is built
// without it (CVC_ARIADNE_HAVE_YAML undefined), load_* return an error and
// have_yaml() is false, so the rest of Ariadne still builds and runs.
//
// Slice 2 adds the meta block (§3.1a): document provenance and the min_libcvc
// LOAD GATE (checked first — a document that needs a newer libcvc fails to load
// rather than half-rendering), plus semantic validation surfaced as warnings.

#include <cvc/ariadne/loader.h>

#include <cvc/core/config.h> // CVC_VERSION_STRING (generated from project(VERSION))

#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

#ifdef CVC_ARIADNE_HAVE_YAML
#include <yaml-cpp/yaml.h>
#endif

namespace cvc {
namespace ariadne {

// ---- version helpers (available in both builds; no yaml needed) -------------

namespace {
std::vector<int> parse_semver(const std::string &v) {
  std::vector<int> out;
  std::string cur;
  for (char c : v) {
    if (c == '-' || c == '+')
      break; // drop any pre-release / build metadata
    if (c == '.') {
      out.push_back(cur.empty() ? 0 : std::atoi(cur.c_str()));
      cur.clear();
    } else if (c >= '0' && c <= '9') {
      cur += c;
    } else if (c == ' ' || c == 'v' || c == 'V') {
      continue; // tolerate a leading 'v' / stray spaces
    } else {
      break; // a non-numeric component ends the version
    }
  }
  if (!cur.empty())
    out.push_back(std::atoi(cur.c_str()));
  return out;
}
} // namespace

bool version_at_least(const std::string &have, const std::string &need) {
  const std::vector<int> h = parse_semver(have), n = parse_semver(need);
  for (std::size_t i = 0; i < 3; ++i) {
    const int hi = i < h.size() ? h[i] : 0;
    const int ni = i < n.size() ? n[i] : 0;
    if (hi != ni)
      return hi > ni;
  }
  return true; // equal → satisfies ">="
}

std::string libcvc_version() { return CVC_VERSION_STRING; }

#ifdef CVC_ARIADNE_HAVE_YAML

namespace {

struct Ctx {
  std::vector<std::string> warnings;
  void warn(std::string m) { warnings.push_back(std::move(m)); }
};

bool has(const YAML::Node &n, const char *key) { return n[key].IsDefined(); }

std::string str(const YAML::Node &n, const char *key, const std::string &dflt = std::string()) {
  const YAML::Node v = n[key];
  if (v && v.IsScalar())
    return v.Scalar();
  return dflt;
}

double num(const YAML::Node &n, const char *key, double dflt) {
  const YAML::Node v = n[key];
  if (v && v.IsScalar()) {
    try {
      return v.as<double>();
    } catch (const std::exception &) {
    }
  }
  return dflt;
}

bool flag(const YAML::Node &n, const char *key, bool dflt = false) {
  const YAML::Node v = n[key];
  if (!v || !v.IsScalar())
    return dflt;
  const std::string s = v.Scalar();
  return s == "true" || s == "1" || s == "yes" || s == "on";
}

std::string action(const YAML::Node &n) {
  const YAML::Node v = n["on"];
  if (v && v.IsScalar())
    return v.Scalar();
  return std::string();
}

Widget parse_widget(Ctx &ctx, const YAML::Node &n);

std::vector<Widget> parse_seq(Ctx &ctx, const YAML::Node &seq) {
  std::vector<Widget> out;
  if (seq && seq.IsSequence())
    for (const YAML::Node &item : seq)
      out.push_back(parse_widget(ctx, item));
  return out;
}

std::string widget_type(const YAML::Node &n, std::string &label) {
  static const char *kKeys[] = {"group",   "menubar",     "menu",         "menu_item",
                                "window",  "overlay",     "panel",        "text",
                                "checkbox", "slider_int", "slider_float",
                                "combo",   "button",      "separator"};
  for (const char *k : kKeys) {
    const YAML::Node v = n[k];
    if (v.IsDefined()) {
      label = v.IsScalar() ? v.Scalar() : std::string();
      return k;
    }
  }
  if (n["type"].IsDefined()) {
    label = str(n, "title", str(n, "label", str(n, "widget")));
    return n["type"].Scalar();
  }
  return std::string();
}

void parse_pos_size(const YAML::Node &n, Widget &w) {
  const YAML::Node p = n["pos"];
  if (p && p.IsSequence() && p.size() >= 2) {
    w.has_pos = true;
    w.pos_x = static_cast<float>(p[0].as<double>());
    w.pos_y = static_cast<float>(p[1].as<double>());
  }
  const YAML::Node s = n["size"];
  if (s && s.IsSequence() && s.size() >= 2) {
    w.has_size = true;
    w.size_w = static_cast<float>(s[0].as<double>());
    w.size_h = static_cast<float>(s[1].as<double>());
  }
}

// Semantic checks on a bound widget: a two-way widget with no state path is
// almost always an authoring mistake (it renders but drives nothing).
void check_bind(Ctx &ctx, const std::string &type, const std::string &label,
                const std::string &bind) {
  if (bind.empty())
    ctx.warn("ari: " + type + " '" + label + "' has no bind: — it will not read or write state");
}

Widget parse_widget(Ctx &ctx, const YAML::Node &n) {
  if (n.IsScalar()) {
    const std::string s = n.Scalar();
    if (s == "separator")
      return separator();
    return text(s);
  }
  if (!n.IsMap()) {
    ctx.warn("ari: a widget list item is neither a map nor a scalar — ignored");
    return group({});
  }

  std::string label;
  const std::string type = widget_type(n, label);

  if (type == "separator")
    return separator();
  if (type == "menubar") {
    Widget w;
    w.kind = Kind::Menubar;
    w.children = parse_seq(ctx, has(n, "children") ? n["children"] : n["items"]);
    return w;
  }
  if (type == "menu")
    return menu(label, parse_seq(ctx, has(n, "items") ? n["items"] : n["children"]));
  if (type == "menu_item") {
    if (has(n, "bind"))
      return menu_toggle(label, str(n, "bind"), flag(n, "def"));
    const std::string on = action(n);
    if (on.empty())
      ctx.warn("ari: menu_item '" + label + "' has neither bind: nor on: — it does nothing");
    return menu_action(label, on);
  }
  if (type == "window" || type == "overlay") {
    Widget w = window(label, parse_seq(ctx, n["children"]));
    w.id = str(n, "id");
    parse_pos_size(n, w);
    return w;
  }
  if (type == "text") {
    if (has(n, "bind"))
      return text_bound(label, str(n, "bind"));
    return text(label);
  }
  if (type == "checkbox") {
    const std::string bind = str(n, "bind");
    check_bind(ctx, "checkbox", label, bind);
    return checkbox(label, bind, flag(n, "def"));
  }
  if (type == "slider_int") {
    const std::string bind = str(n, "bind");
    check_bind(ctx, "slider_int", label, bind);
    const int lo = static_cast<int>(num(n, "lo", 0)), hi = static_cast<int>(num(n, "hi", 100));
    if (lo >= hi)
      ctx.warn("ari: slider_int '" + label + "' has lo >= hi");
    return slider_int(label, bind, lo, hi, static_cast<int>(num(n, "def", 0)));
  }
  if (type == "slider_float") {
    const std::string bind = str(n, "bind");
    check_bind(ctx, "slider_float", label, bind);
    const double lo = num(n, "lo", 0.0), hi = num(n, "hi", 1.0);
    if (lo >= hi)
      ctx.warn("ari: slider_float '" + label + "' has lo >= hi");
    return slider_float(label, bind, lo, hi, num(n, "def", 0.0), str(n, "fmt", "%.3f"));
  }
  if (type == "combo") {
    const std::string bind = str(n, "bind");
    check_bind(ctx, "combo", label, bind);
    std::vector<std::string> opts;
    const YAML::Node o = n["options"];
    if (o && o.IsSequence())
      for (const YAML::Node &e : o)
        if (e.IsScalar())
          opts.push_back(e.Scalar());
    if (opts.empty())
      ctx.warn("ari: combo '" + label + "' has no options:");
    return combo(label, bind, std::move(opts), str(n, "default", str(n, "def")));
  }
  if (type == "button") {
    const std::string on = action(n);
    if (on.empty())
      ctx.warn("ari: button '" + label + "' has no on: action");
    return button(label, on);
  }

  if (!type.empty()) {
    if (type != "group" && type != "panel")
      ctx.warn("ari: unknown widget type '" + type + "' — loaded as a plain group");
  } else if (!has(n, "children") && n.size() > 0) {
    // No recognized type key and no `type:`; a bare `{ children: [...] }` is a
    // valid anonymous group, but a map with some OTHER first key is a typo'd
    // widget (e.g. `- frobnicate: ...` instead of a real widget kind).
    std::string first_key;
    for (const auto &kv : n) {
      first_key = kv.first.Scalar();
      break;
    }
    ctx.warn("ari: unrecognized widget key '" + first_key +
             "' — not a known widget type; loaded as an empty group");
  }
  return group(parse_seq(ctx, n["children"]));
}

Widget parse_document(Ctx &ctx, const YAML::Node &doc) {
  std::vector<Widget> roots;
  if (doc.IsSequence()) {
    roots = parse_seq(ctx, doc);
  } else if (doc.IsMap()) {
    if (doc["menubar"].IsDefined()) {
      Widget mb;
      mb.kind = Kind::Menubar;
      mb.children = parse_seq(ctx, doc["menubar"]);
      roots.push_back(std::move(mb));
    }
    for (const char *k : {"windows", "overlays"})
      if (doc[k].IsDefined())
        for (Widget &w : parse_seq(ctx, doc[k]))
          roots.push_back(std::move(w));
    const YAML::Node r = doc["root"].IsDefined() ? doc["root"] : doc["children"];
    if (r.IsDefined())
      for (Widget &w : parse_seq(ctx, r))
        roots.push_back(std::move(w));
  } else {
    ctx.warn("ari: document root is neither a map nor a sequence — nothing to load");
  }
  return group(std::move(roots));
}

Meta parse_meta(const YAML::Node &m) {
  Meta meta;
  if (!m || !m.IsMap())
    return meta;
  meta.name = str(m, "name");
  meta.author = str(m, "author");
  meta.description = str(m, "description");
  meta.version = str(m, "version");
  meta.min_libcvc = str(m, "min_libcvc");
  return meta;
}

// Parse an already-loaded YAML node into a LoadResult, applying the meta gate.
LoadResult load_node(const YAML::Node &doc) {
  LoadResult r;
  Ctx ctx;
  r.meta = doc.IsMap() ? parse_meta(doc["meta"]) : Meta{};

  // The min_libcvc gate fires FIRST (roadmap §3.1a): a document that needs a
  // newer libcvc fails to load rather than half-rendering against a stale API.
  if (!r.meta.min_libcvc.empty() &&
      !version_at_least(libcvc_version(), r.meta.min_libcvc)) {
    r.error = "ari: document requires libcvc >= " + r.meta.min_libcvc +
              " but this build is " + libcvc_version();
    return r;
  }

  r.root = parse_document(ctx, doc);
  if (r.meta.min_libcvc.empty())
    ctx.warn("ari: no meta.min_libcvc declared — the provenance gate is skipped "
             "(roadmap §3.1a asks every .ari to declare it)");
  r.warnings = std::move(ctx.warnings);
  r.ok = true;
  return r;
}

} // namespace

LoadResult load_string(const std::string &yaml) {
  try {
    return load_node(YAML::Load(yaml));
  } catch (const YAML::Exception &e) {
    LoadResult r;
    r.error = std::string("ari: YAML parse error: ") + e.what();
    return r;
  } catch (const std::exception &e) {
    LoadResult r;
    r.error = std::string("ari: load error: ") + e.what();
    return r;
  }
}

LoadResult load_file(const std::string &path) {
  try {
    return load_node(YAML::LoadFile(path));
  } catch (const YAML::BadFile &e) {
    LoadResult r;
    r.error = std::string("ari: cannot open '") + path + "': " + e.what();
    return r;
  } catch (const YAML::Exception &e) {
    LoadResult r;
    r.error = std::string("ari: YAML parse error in '") + path + "': " + e.what();
    return r;
  } catch (const std::exception &e) {
    LoadResult r;
    r.error = std::string("ari: load error: ") + e.what();
    return r;
  }
}

bool have_yaml() { return true; }

#else // !CVC_ARIADNE_HAVE_YAML — stub: the rest of Ariadne still builds/runs.

namespace {
LoadResult unavailable() {
  LoadResult r;
  r.ok = false;
  r.error = "ari: libcvc was built without yaml-cpp (.ari loading unavailable)";
  return r;
}
} // namespace

LoadResult load_string(const std::string &) { return unavailable(); }
LoadResult load_file(const std::string &) { return unavailable(); }
bool have_yaml() { return false; }

#endif

} // namespace ariadne
} // namespace cvc
