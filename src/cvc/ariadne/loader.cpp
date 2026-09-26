// Ariadne — the .ari YAML loader (see the header). Parses a .ari document into a
// cvc::ariadne Widget tree. Pure libcvc: it produces the backend-neutral tree,
// unaware of any UI toolkit. Optional yaml-cpp dependency — when libcvc is built
// without it (CVC_ARIADNE_HAVE_YAML undefined), load_* return an error and
// have_yaml() is false, so the rest of Ariadne still builds and runs.

#include <cvc/ariadne/loader.h>

#include <string>
#include <utility>
#include <vector>

#ifdef CVC_ARIADNE_HAVE_YAML
#include <yaml-cpp/yaml.h>
#endif

namespace cvc {
namespace ariadne {

#ifdef CVC_ARIADNE_HAVE_YAML

namespace {

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

// The `on:` action. P0 supports only the scalar event-name form (`on: reset`);
// a nested program form (roadmap §4) is not evaluated yet — treated as no action.
std::string action(const YAML::Node &n) {
  const YAML::Node v = n["on"];
  if (v && v.IsScalar())
    return v.Scalar();
  return std::string();
}

Widget parse_widget(const YAML::Node &n);

std::vector<Widget> parse_seq(const YAML::Node &seq) {
  std::vector<Widget> out;
  if (seq && seq.IsSequence())
    for (const YAML::Node &item : seq)
      out.push_back(parse_widget(item));
  return out;
}

// The type of a widget map is named by which known key is present; its scalar
// value is the label (roadmap §3.5: `- slider_int: agents`). Falls back to the
// explicit `type:` + `title:`/`label:`/`widget:` form (§3.0.1).
std::string widget_type(const YAML::Node &n, std::string &label) {
  static const char *kKeys[] = {"group",   "menubar",      "menu",     "menu_item",
                                "window",  "overlay",      "panel",    "text",
                                "checkbox", "slider_int",  "slider_float",
                                "combo",   "button",       "separator"};
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

Widget parse_widget(const YAML::Node &n) {
  // Bare scalar: `- separator`, or `- some caption` (a literal text line).
  if (n.IsScalar()) {
    const std::string s = n.Scalar();
    if (s == "separator")
      return separator();
    return text(s);
  }
  if (!n.IsMap())
    return group({}); // unknown shape → empty group (renders as nothing)

  std::string label;
  const std::string type = widget_type(n, label);

  if (type == "separator")
    return separator();
  if (type == "menubar") {
    Widget w;
    w.kind = Kind::Menubar;
    w.children = parse_seq(has(n, "children") ? n["children"] : n["items"]);
    return w;
  }
  if (type == "menu")
    return menu(label, parse_seq(has(n, "items") ? n["items"] : n["children"]));
  if (type == "menu_item") {
    if (has(n, "bind"))
      return menu_toggle(label, str(n, "bind"), flag(n, "def"));
    return menu_action(label, action(n));
  }
  if (type == "window" || type == "overlay") {
    Widget w = window(label, parse_seq(n["children"]));
    w.id = str(n, "id");
    parse_pos_size(n, w);
    return w;
  }
  if (type == "text") {
    if (has(n, "bind"))
      return text_bound(label, str(n, "bind"));
    return text(label);
  }
  if (type == "checkbox")
    return checkbox(label, str(n, "bind"), flag(n, "def"));
  if (type == "slider_int")
    return slider_int(label, str(n, "bind"), static_cast<int>(num(n, "lo", 0)),
                      static_cast<int>(num(n, "hi", 100)), static_cast<int>(num(n, "def", 0)));
  if (type == "slider_float")
    return slider_float(label, str(n, "bind"), num(n, "lo", 0.0), num(n, "hi", 1.0),
                        num(n, "def", 0.0), str(n, "fmt", "%.3f"));
  if (type == "combo") {
    std::vector<std::string> opts;
    const YAML::Node o = n["options"];
    if (o && o.IsSequence())
      for (const YAML::Node &e : o)
        if (e.IsScalar())
          opts.push_back(e.Scalar());
    return combo(label, str(n, "bind"), std::move(opts), str(n, "default", str(n, "def")));
  }
  if (type == "button")
    return button(label, action(n));

  // "group", "panel", explicit type: containers, and anything unknown: a group
  // of its children (so a fuller document still loads its structure).
  return group(parse_seq(n["children"]));
}

// A document is a map with any of `menubar:` / `windows:` / `overlays:` (the
// documented top-level lists, §3.2–3.4) and/or a unified `root:`/`children:`
// list; or a bare sequence of widgets. Everything collapses to one root Group.
Widget parse_document(const YAML::Node &doc) {
  std::vector<Widget> roots;
  if (doc.IsSequence()) {
    roots = parse_seq(doc);
  } else if (doc.IsMap()) {
    if (doc["menubar"].IsDefined()) {
      Widget mb;
      mb.kind = Kind::Menubar;
      mb.children = parse_seq(doc["menubar"]);
      roots.push_back(std::move(mb));
    }
    for (const char *k : {"windows", "overlays"})
      if (doc[k].IsDefined())
        for (Widget &w : parse_seq(doc[k]))
          roots.push_back(std::move(w));
    const YAML::Node r = doc["root"].IsDefined() ? doc["root"] : doc["children"];
    if (r.IsDefined())
      for (Widget &w : parse_seq(r))
        roots.push_back(std::move(w));
  }
  return group(std::move(roots));
}

} // namespace

LoadResult load_string(const std::string &yaml) {
  LoadResult r;
  try {
    r.root = parse_document(YAML::Load(yaml));
    r.ok = true;
  } catch (const YAML::Exception &e) {
    r.error = std::string("ari: YAML parse error: ") + e.what();
  } catch (const std::exception &e) {
    r.error = std::string("ari: load error: ") + e.what();
  }
  return r;
}

LoadResult load_file(const std::string &path) {
  LoadResult r;
  try {
    r.root = parse_document(YAML::LoadFile(path));
    r.ok = true;
  } catch (const YAML::BadFile &e) {
    r.error = std::string("ari: cannot open '") + path + "': " + e.what();
  } catch (const YAML::Exception &e) {
    r.error = std::string("ari: YAML parse error in '") + path + "': " + e.what();
  } catch (const std::exception &e) {
    r.error = std::string("ari: load error: ") + e.what();
  }
  return r;
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
