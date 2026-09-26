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

#include <cvc/ariadne/ariadne.h> // has_widget_type (custom-widget load-time check)
#include <cvc/core/config.h>     // CVC_VERSION_STRING (generated from project(VERSION))

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#ifdef CVC_ARIADNE_HAVE_YAML
#include <yaml-cpp/yaml.h>
#endif

#ifdef CVC_ARIADNE_HAVE_JSONSCHEMA
#include <nlohmann/json-schema.hpp> // pulls in <nlohmann/json.hpp>
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

// ---- custom top-level block registry (both builds; no yaml needed) ----------

namespace {
std::mutex &block_mutex() {
  static std::mutex m;
  return m;
}
std::map<std::string, AriBlockParser> &block_registry() {
  static std::map<std::string, AriBlockParser> r;
  return r;
}
bool is_builtin_block(const std::string &k) {
  return k == "meta" || k == "menubar" || k == "windows" || k == "overlays" ||
         k == "root" || k == "children" || k == "scene" || k == "customs" || k == "init";
}
} // namespace

void register_ari_block(const std::string &key, AriBlockParser parse) {
  if (!parse || is_builtin_block(key)) // built-ins own their own dispatch
    return;
  std::lock_guard<std::mutex> lock(block_mutex());
  block_registry()[key] = std::move(parse);
}

bool has_ari_block(const std::string &key) {
  std::lock_guard<std::mutex> lock(block_mutex());
  return block_registry().find(key) != block_registry().end();
}

// ---- the .ari JSON Schema (Layer 2, roadmap §15) ----------------------------
// A permissive draft-2020-12 schema: it enforces the meta block (min_libcvc
// required) and the FIELD types of widgets (bind is a string, lo/hi numbers,
// options an array of strings, …), while leaving the widget type-key and any
// forward-compat keys unconstrained (additionalProperties defaults true) so a
// newer document still validates its known structure. Exposed for tooling even
// when the validator itself is not linked.
namespace {
const char *const kAriSchema = R"ARISCHEMA({
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://cvcpkg.org/schemas/ari.schema.json",
  "title": "Ariadne .ari document",
  "type": "object",
  "properties": {
    "meta": {
      "type": "object",
      "properties": {
        "name": {"type": "string"},
        "author": {"type": "string"},
        "description": {"type": "string"},
        "version": {"type": "string"},
        "min_libcvc": {"type": "string"}
      },
      "required": ["min_libcvc"]
    },
    "menubar":  {"type": "array", "items": {"$ref": "#/$defs/widget"}},
    "windows":  {"type": "array", "items": {"$ref": "#/$defs/widget"}},
    "overlays": {"type": "array", "items": {"$ref": "#/$defs/widget"}},
    "root":     {"type": "array", "items": {"$ref": "#/$defs/widget"}},
    "children": {"type": "array", "items": {"$ref": "#/$defs/widget"}}
  },
  "$defs": {
    "widget": {
      "type": ["string", "object"],
      "properties": {
        "id": {"type": "string"},
        "title": {"type": "string"},
        "type": {"type": "string"},
        "bind": {"type": "string"},
        "on": {"type": ["string", "object"]},
        "lo": {"type": "number"},
        "hi": {"type": "number"},
        "def": {"type": ["number", "boolean", "string"]},
        "fmt": {"type": "string"},
        "options": {"type": "array", "items": {"type": "string"}},
        "default": {"type": "string"},
        "pos": {"type": "array", "items": {"type": "number"}},
        "size": {"type": "array", "items": {"type": "number"}},
        "items": {"type": "array", "items": {"$ref": "#/$defs/widget"}},
        "children": {"type": "array", "items": {"$ref": "#/$defs/widget"}}
      }
    }
  }
})ARISCHEMA";
} // namespace

std::string ari_schema_json() { return kAriSchema; }

bool have_jsonschema() {
#ifdef CVC_ARIADNE_HAVE_JSONSCHEMA
  return true;
#else
  return false;
#endif
}

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
  // Case-insensitive over the YAML boolean spellings (yaml-cpp's Scalar() is the raw,
  // UNNORMALIZED text, so "True"/"YES"/"On" would otherwise miss) — keeps "1"/"0" too.
  // This matters most for `required:` on a customs entry, a fail-fast SAFETY flag.
  std::string s = v.Scalar();
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (s == "true" || s == "1" || s == "yes" || s == "on" || s == "y")
    return true;
  if (s == "false" || s == "0" || s == "no" || s == "off" || s == "n")
    return false;
  return dflt;
}

std::string action(const YAML::Node &n) {
  const YAML::Node v = n["on"];
  if (v && v.IsScalar())
    return v.Scalar();
  return std::string();
}

Widget parse_widget(Ctx &ctx, const YAML::Node &n);
Value to_value(const YAML::Node &n); // defined below; used by the Kind::Custom fallback

// Widget keys the loader consumes directly; everything else on a custom widget flows
// into Widget::props for a registered emit fn to read.
inline bool known_widget_key(const std::string &k) {
  return k == "type" || k == "title" || k == "label" || k == "widget" || k == "bind" ||
         k == "on" || k == "children" || k == "items" || k == "id" || k == "visible_when";
}

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

void parse_pos(const YAML::Node &n, Widget &w) {
  const YAML::Node p = n["pos"];
  if (p && p.IsSequence() && p.size() >= 2) {
    w.has_pos = true;
    w.pos_x = static_cast<float>(p[0].as<double>());
    w.pos_y = static_cast<float>(p[1].as<double>());
  }
}

// A §3.0.3b size/track value: "50%" (or a bare 0<v<1 fraction) -> percent;
// "auto"/empty -> auto (fit); any other number -> pixels.
Extent parse_extent(const YAML::Node &n) {
  Extent e;
  if (!n || !n.IsScalar())
    return e; // Auto
  const std::string s = n.Scalar();
  if (s.empty() || s == "auto")
    return e;
  if (s.back() == '%') {
    e.unit = Unit::Percent;
    try {
      e.value = std::stof(s.substr(0, s.size() - 1));
    } catch (const std::exception &) {
    }
    return e;
  }
  try {
    const float v = std::stof(s);
    if (v == 0.0f) {
      // 0 = auto axis / no constraint (§3.0.3: "0 = auto", "0 = unbounded")
    } else if (v > 0.0f && v < 1.0f) { // a fraction -> percent
      e.unit = Unit::Percent;
      e.value = v * 100.0f;
    } else {
      e.unit = Unit::Px;
      e.value = v;
    }
  } catch (const std::exception &) {
  }
  return e;
}

Size parse_size(const YAML::Node &n) {
  Size sz;
  if (!n)
    return sz;
  if (n.IsSequence() && n.size() >= 2) { // size: [w, h]
    sz.w = parse_extent(n[0]);
    sz.h = parse_extent(n[1]);
  } else if (n.IsMap()) { // size: { hint:[w,h], min:[..], max:[..] }
    auto pair = [&](const char *key, Extent &a, Extent &b) {
      const YAML::Node v = n[key];
      if (v && v.IsSequence() && v.size() >= 2) {
        a = parse_extent(v[0]);
        b = parse_extent(v[1]);
      }
    };
    pair("hint", sz.w, sz.h);
    pair("min", sz.min_w, sz.min_h);
    pair("max", sz.max_w, sz.max_h);
  }
  return sz;
}

Layout parse_layout(const YAML::Node &n) {
  Layout L;
  if (!n || !n.IsMap())
    return L;
  const std::string kind = str(n, "kind");
  if (kind == "horizontal")
    L.kind = LayoutKind::Horizontal;
  else if (kind == "grid")
    L.kind = LayoutKind::Grid;
  else
    L.kind = LayoutKind::Vertical;
  auto tracks = [](const YAML::Node &seq, std::vector<Track> &out) {
    if (seq && seq.IsSequence())
      for (const YAML::Node &t : seq) {
        const Extent e = parse_extent(t);
        Track tr;
        tr.unit = e.unit;
        tr.value = e.value;
        out.push_back(tr);
      }
  };
  tracks(n["col_widths"], L.col_widths);
  tracks(n["row_heights"], L.row_heights);
  L.resizable = flag(n, "resizable");
  const YAML::Node b = n["borders"];
  if (b && b.IsMap()) {
    const std::string show = str(b, "show");
    if (show == "inner")
      L.borders = BorderShow::Inner;
    else if (show == "outer")
      L.borders = BorderShow::Outer;
    else if (show == "all")
      L.borders = BorderShow::All;
    const YAML::Node col = b["color"];
    if (col && col.IsSequence() && col.size() >= 3) {
      L.has_border_color = true;
      L.border_color[3] = 1.0f;
      for (std::size_t i = 0; i < 4 && i < col.size(); ++i)
        L.border_color[i] = static_cast<float>(col[i].as<double>());
    }
  }
  return L;
}

// frame: { border: N } -> the widget/window border width in px; -1 = unset.
float parse_frame_border(const YAML::Node &n) {
  if (n && n.IsMap()) {
    const YAML::Node b = n["border"];
    if (b && b.IsScalar())
      try {
        return static_cast<float>(b.as<double>());
      } catch (const std::exception &) {
      }
  }
  return -1.0f;
}

// Semantic checks on a bound widget: a two-way widget with no state path is
// almost always an authoring mistake (it renders but drives nothing).
void check_bind(Ctx &ctx, const std::string &type, const std::string &label,
                const std::string &bind) {
  if (bind.empty())
    ctx.warn("ari: " + type + " '" + label + "' has no bind: — it will not read or write state");
}

Widget parse_widget_impl(Ctx &ctx, const YAML::Node &n) {
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
    parse_pos(n, w);
    w.size = parse_size(n["size"]);            // §3.0.3b window sizing (%/px/auto)
    w.layout = parse_layout(n["layout"]);      // §3.0.3b window-body layout (grid/tracks)
    w.frame_border = parse_frame_border(n["frame"]); // §3.0.3b border width
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

  if (!type.empty() && type != "group" && type != "panel") {
    // A CUSTOM widget type (§ extensibility) — preserved as Kind::Custom carrying its
    // props + children, for a registered emit fn (register_widget_type) to render.
    // (No longer silently collapsed to an empty group.) An unregistered type draws a
    // labelled placeholder at emit; validation still warns if it is a typo.
    Widget c;
    c.kind = Kind::Custom;
    c.custom_type = type;
    c.label = label;
    c.bind = str(n, "bind");
    c.on = action(n);
    const YAML::Node kids = n["children"].IsDefined() ? n["children"] : n["items"];
    c.children = parse_seq(ctx, kids);
    c.props.kind = Value::Kind::Map;
    if (n.IsMap())
      for (const auto &kv : n) {
        if (!kv.first.IsScalar())
          continue;
        const std::string key = kv.first.Scalar();
        // known_widget_key already excludes "type"; do NOT also skip a prop whose name
        // equals the type VALUE (e.g. `gauge: 0.8` on `type: gauge`) — that dropped
        // real config. Matches the scene-node capture.
        if (!known_widget_key(key))
          c.props.entries.emplace_back(key, to_value(kv.second));
      }
    if (!has_widget_type(type))
      ctx.warn("ari: widget type '" + type +
               "' has no registered handler (register_widget_type) — draws a placeholder");
    return c;
  }
  // Here `type` is empty (no `type:` and no short-form key) or group/panel. A bare
  // `{ children: [...] }` is a valid anonymous group; a map with some OTHER first key
  // and no `type:` is a typo'd widget (e.g. `- frobnicate: ...`) — a custom widget is
  // declared with `type:` (which returned a Kind::Custom above), so a lone unknown key
  // is more likely a mistake.
  if (type.empty() && !has(n, "children") && n.size() > 0) {
    std::string first_key;
    for (const auto &kv : n) {
      first_key = kv.first.Scalar();
      break;
    }
    ctx.warn("ari: unrecognized widget key '" + first_key +
             "' — not a known widget type; loaded as an empty group");
  }
  Widget g = group(parse_seq(ctx, n["children"]));
  g.layout = parse_layout(n["layout"]); // §3.0.3b: a group can be a grid / sized tracks
  g.size = parse_size(n["size"]);
  return g;
}

// Build the kind-specific widget, then attach the fields common to EVERY kind. Keeps
// per-kind branches focused on their own params and guarantees a new shared field is
// wired for all kinds at once. §4 read-lane: visible_when is a state_exec predicate the
// Runtime re-evaluates each frame (empty = always visible).
Widget parse_widget(Ctx &ctx, const YAML::Node &n) {
  Widget w = parse_widget_impl(ctx, n);
  if (n.IsMap())
    w.visible_when = str(n, "visible_when");
  return w;
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

void vec3(const YAML::Node &n, const char *key, float out[3]) {
  const YAML::Node v = n[key];
  if (v && v.IsSequence() && v.size() >= 3)
    for (int i = 0; i < 3; ++i)
      out[i] = static_cast<float>(v[i].as<double>());
}

// Convert a YAML subtree into the backend-neutral Value tree (§ extensibility), so a
// custom node's props / a custom block's content can be read without yaml-cpp. Keys
// that are not scalars are skipped (YAML keys are scalars in practice). Never throws.
Value to_value(const YAML::Node &n) {
  Value v;
  if (n.IsSequence()) {
    v.kind = Value::Kind::Sequence;
    for (const YAML::Node &c : n)
      v.items.push_back(to_value(c));
  } else if (n.IsMap()) {
    v.kind = Value::Kind::Map;
    for (const auto &kv : n) {
      if (!kv.first.IsScalar())
        continue;
      v.entries.emplace_back(kv.first.Scalar(), to_value(kv.second));
    }
  } else if (n.IsScalar()) {
    v.kind = Value::Kind::Scalar;
    v.scalar = n.Scalar();
  }
  return v; // a null/undefined node stays a default (empty scalar)
}

// The scene-node keys the built-in parser consumes; everything else on a node flows
// into SceneNode::props for a custom node type to read.
bool known_scene_node_key(const std::string &k) {
  return k == "node" || k == "id" || k == "type" || k == "source" || k == "material" ||
         k == "transform" || k == "visible" || k == "volren" || k == "volslice" ||
         k == "children";
}

// A transfer function (§9): points [{value, color:[r,g,b,a]}], optional window and
// auto_domain. Shared by volren + volslice (one TF vocabulary across renderers).
SceneTransferFunction parse_scene_tf(const YAML::Node &t) {
  SceneTransferFunction tf;
  if (!t || !t.IsMap())
    return tf;
  tf.auto_domain = flag(t, "auto_domain", true);
  const YAML::Node win = t["window"];
  if (win && win.IsSequence() && win.size() >= 2) {
    tf.has_window = true;
    tf.auto_domain = false; // an explicit window fixes the domain
    tf.window_min = win[0].as<double>();
    tf.window_max = win[1].as<double>();
  }
  const YAML::Node pts = t["points"];
  if (pts && pts.IsSequence())
    for (const YAML::Node &p : pts) {
      SceneTFPoint tp;
      tp.value = num(p, "value", 0.0);
      const YAML::Node c = p["color"];
      if (c && c.IsSequence())
        for (std::size_t i = 0; i < 4 && i < c.size(); ++i)
          tp.color[i] = static_cast<float>(c[i].as<double>());
      tf.points.push_back(tp);
    }
  return tf;
}

// A scene node (§9.3), recursive over children.
SceneNode parse_scene_node(const YAML::Node &n) {
  SceneNode sn;
  sn.id = str(n, "node", str(n, "id"));
  sn.type = str(n, "type", "geometry");
  const YAML::Node src = n["source"];
  if (src && src.IsMap())
    sn.source_file = str(src, "file");
  const YAML::Node mat = n["material"];
  if (mat && mat.IsMap()) {
    sn.has_material = true;
    const YAML::Node col = mat["color"];
    if (col && col.IsSequence() && col.size() >= 3)
      for (int i = 0; i < 3; ++i)
        sn.color[i] = static_cast<float>(col[i].as<double>());
    sn.ambient = static_cast<float>(num(mat, "ambient", sn.ambient));
    sn.diffuse = static_cast<float>(num(mat, "diffuse", sn.diffuse));
  }
  const YAML::Node tf = n["transform"];
  if (tf && tf.IsMap()) {
    sn.has_transform = true;
    vec3(tf, "position", sn.position);
    vec3(tf, "rotation", sn.rotation);
    const YAML::Node sc = tf["scale"];
    if (sc && sc.IsSequence() && sc.size() >= 3)
      for (int i = 0; i < 3; ++i)
        sn.scale[i] = static_cast<float>(sc[i].as<double>());
    else if (sc && sc.IsScalar()) {
      const float s = static_cast<float>(num(tf, "scale", 1.0));
      sn.scale[0] = sn.scale[1] = sn.scale[2] = s; // scalar → uniform scale
    }
  }
  const YAML::Node vis = n["visible"];
  if (vis) {
    if (vis.IsMap()) {
      // Map form: bind AND a start default together, so a bound node can default
      // hidden even when no widget owns the key: visible: { bind: p, default: false }
      sn.visible_bind = str(vis, "bind");
      if (vis["default"])
        sn.visible_default = flag(vis, "default", true);
    } else if (vis.IsScalar()) {
      const std::string v = vis.Scalar();
      if (v == "true" || v == "false")
        sn.visible_default = (v == "true"); // literal only (bind stays empty)
      else
        sn.visible_bind = v; // a state path (or, later, an expression); default true
    }
  }
  const YAML::Node vr = n["volren"];
  if (vr && vr.IsMap()) {
    sn.has_volren = true;
    sn.volren.shaded = flag(vr, "shaded", true);
    sn.volren.unshaded = flag(vr, "unshaded", false);
    sn.volren.distance_field = flag(vr, "distance_field", false);
    sn.volren.steps = static_cast<int>(num(vr, "steps", 512));
    sn.volren.ambient = static_cast<float>(num(vr, "ambient", 0.0));
    sn.volren.resolution_scale = static_cast<float>(num(vr, "resolution_scale", 0.5));
    sn.volren.backend = str(vr, "backend", "cpu");
    sn.volren.tf = parse_scene_tf(vr["transfer_function"]);
    const YAML::Node iso = vr["isosurfaces"];
    if (iso && iso.IsSequence())
      for (const YAML::Node &s : iso) {
        SceneIsosurface si;
        si.value = num(s, "value", 0.0);
        si.opacity = static_cast<float>(num(s, "opacity", 1.0));
        const YAML::Node c = s["color"];
        if (c && c.IsSequence() && c.size() >= 3)
          for (int i = 0; i < 3; ++i)
            si.color[i] = static_cast<float>(c[i].as<double>());
        si.shininess = static_cast<float>(num(s, "shininess", 10.0));
        sn.volren.isosurfaces.push_back(si);
      }
    const YAML::Node lts = vr["lights"];
    if (lts && lts.IsSequence())
      for (const YAML::Node &l : lts) {
        SceneVolRenLight vl;
        vec3(l, "color", vl.color);
        vec3(l, "direction", vl.direction);
        sn.volren.lights.push_back(vl);
      }
  }
  const YAML::Node vsl = n["volslice"];
  if (vsl && vsl.IsMap()) {
    sn.has_volslice = true;
    sn.volslice.quality = static_cast<float>(num(vsl, "quality", 0.5));
    sn.volslice.max_planes = static_cast<int>(num(vsl, "max_planes", 1000));
    sn.volslice.near_plane = static_cast<float>(num(vsl, "near_plane", 0.0));
    sn.volslice.nearest_filter = (str(vsl, "filter", "linear") == "nearest");
    sn.volslice.opacity_correction = flag(vsl, "opacity_correction", false);
    sn.volslice.tf = parse_scene_tf(vsl["transfer_function"]);
  }
  // Capture every key the built-ins did NOT consume into props (a neutral Map), so a
  // custom node type registered on the cvcGL side reads its own config from there.
  sn.props.kind = Value::Kind::Map;
  if (n.IsMap())
    for (const auto &kv : n) {
      if (!kv.first.IsScalar())
        continue;
      const std::string key = kv.first.Scalar();
      if (!known_scene_node_key(key))
        sn.props.entries.emplace_back(key, to_value(kv.second));
    }
  const YAML::Node kids = n["children"];
  if (kids && kids.IsSequence())
    for (const YAML::Node &c : kids)
      sn.children.push_back(parse_scene_node(c));
  return sn;
}

SceneLight parse_scene_light(const YAML::Node &n) {
  SceneLight sl;
  sl.id = str(n, "light", str(n, "id"));
  sl.rig = str(n, "rig");
  sl.kind = str(n, "kind", "directional");
  vec3(n, "pos", sl.pos);
  vec3(n, "target", sl.target);
  vec3(n, "color", sl.color);
  sl.cone = static_cast<float>(num(n, "cone", sl.cone));
  sl.azimuth = static_cast<float>(num(n, "azimuth", sl.azimuth));
  sl.elevation = static_cast<float>(num(n, "elevation", sl.elevation));
  sl.intensity = static_cast<float>(num(n, "intensity", sl.intensity));
  return sl;
}

Scene parse_scene(const YAML::Node &s) {
  Scene sc;
  if (!s || !s.IsMap())
    return sc;
  const YAML::Node nodes = s["nodes"];
  if (nodes && nodes.IsSequence())
    for (const YAML::Node &n : nodes)
      sc.nodes.push_back(parse_scene_node(n));
  const YAML::Node lights = s["lights"];
  if (lights && lights.IsSequence())
    for (const YAML::Node &l : lights)
      sc.lights.push_back(parse_scene_light(l));
  const YAML::Node sh = s["shadows"];
  if (sh && sh.IsMap()) {
    sc.has_shadows = true;
    sc.shadows_enabled = flag(sh, "enabled");
  }
  return sc;
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

#ifdef CVC_ARIADNE_HAVE_JSONSCHEMA
// Convert a yaml-cpp node to nlohmann::json so json-schema-validator can check it.
// yaml-cpp scalars are untyped strings; coerce to int/double/bool where the whole
// scalar parses as one, else keep the string (so "3.4.0" and "%.2f" stay strings).
nlohmann::json yaml_to_json(const YAML::Node &n) {
  using nlohmann::json;
  switch (n.Type()) {
  case YAML::NodeType::Null:
    return json(nullptr);
  case YAML::NodeType::Sequence: {
    json a = json::array();
    for (const YAML::Node &e : n)
      a.push_back(yaml_to_json(e));
    return a;
  }
  case YAML::NodeType::Map: {
    json o = json::object();
    for (const auto &kv : n)
      o[kv.first.Scalar()] = yaml_to_json(kv.second);
    return o;
  }
  case YAML::NodeType::Scalar: {
    const std::string s = n.Scalar();
    if (s == "true" || s == "false")
      return json(s == "true");
    try {
      std::size_t pos = 0;
      const long long i = std::stoll(s, &pos);
      if (pos == s.size())
        return json(i);
    } catch (...) {
    }
    try {
      std::size_t pos = 0;
      const double d = std::stod(s, &pos);
      if (pos == s.size())
        return json(d);
    } catch (...) {
    }
    return json(s);
  }
  default:
    return json(nullptr);
  }
}

// Layer 2 (roadmap §15): validate the document against the .ari JSON Schema,
// collecting violations as warnings (advisory — the lenient loader still loads
// what it can; the min_libcvc gate above is the only fatal provenance check).
void schema_validate(const YAML::Node &doc, Ctx &ctx) {
  struct Collector : nlohmann::json_schema::error_handler {
    Ctx &ctx;
    explicit Collector(Ctx &c) : ctx(c) {}
    void error(const nlohmann::json::json_pointer &ptr, const nlohmann::json &,
               const std::string &msg) override {
      const std::string at = ptr.to_string();
      ctx.warn("ari: schema: " + (at.empty() ? std::string("<root>") : at) + ": " + msg);
    }
  } collector(ctx);
  try {
    nlohmann::json_schema::json_validator validator;
    validator.set_root_schema(nlohmann::json::parse(kAriSchema));
    validator.validate(yaml_to_json(doc), collector);
  } catch (const std::exception &e) {
    ctx.warn(std::string("ari: schema validation could not run: ") + e.what());
  }
}
#endif // CVC_ARIADNE_HAVE_JSONSCHEMA

// Parse the `customs:` block (§ extensibility): the custom types the document uses.
// Each entry names exactly one of widget:/node:/block: with an optional `required:`
// flag (default false — a missing one warns; `required: true` fails the load).
std::vector<CustomRequirement> parse_customs(Ctx &ctx, const YAML::Node &c) {
  std::vector<CustomRequirement> out;
  if (!c || !c.IsSequence())
    return out;
  for (const YAML::Node &e : c) {
    if (!e.IsMap())
      continue;
    const int kinds = static_cast<int>(has(e, "widget")) + static_cast<int>(has(e, "node")) +
                      static_cast<int>(has(e, "block"));
    if (kinds == 0)
      continue; // no widget:/node:/block: key — not a custom declaration
    if (kinds > 1) {
      ctx.warn("ari: customs entry names more than one of widget:/node:/block: — ignored "
               "(declare one custom per entry)");
      continue;
    }
    CustomRequirement req;
    if (has(e, "widget")) {
      req.kind = CustomRequirement::Kind::Widget;
      req.name = str(e, "widget");
    } else if (has(e, "node")) {
      req.kind = CustomRequirement::Kind::Node;
      req.name = str(e, "node");
    } else {
      req.kind = CustomRequirement::Kind::Block;
      req.name = str(e, "block");
    }
    req.required = flag(e, "required", false); // default optional (warn); required: true = fail
    if (req.name.empty()) {
      ctx.warn("ari: customs entry declares a custom with no name — ignored");
      continue;
    }
    out.push_back(req);
  }
  return out;
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

  // Custom-object gate (§ extensibility): a declared `customs:` entry that this build
  // can't satisfy fails the load (required) or warns (optional). Widget/block customs
  // are checked here (their registries are core); NODE customs are checked by
  // cvc::gl::ariadne::verify_scene_customs before realize (that registry is cvcGL).
  const YAML::Node customs_node = doc.IsMap() ? doc["customs"] : YAML::Node();
  if (customs_node.IsDefined() && !customs_node.IsSequence())
    ctx.warn("ari: customs: must be a SEQUENCE of {widget|node|block: name, required?} entries "
             "— this customs block is not a sequence and was ignored, so its declarations "
             "(including any required:) are NOT enforced");
  r.customs = parse_customs(ctx, customs_node);
  for (const CustomRequirement &req : r.customs) {
    const char *kind = "widget";
    bool present = true;
    if (req.kind == CustomRequirement::Kind::Widget) {
      present = has_widget_type(req.name);
    } else if (req.kind == CustomRequirement::Kind::Block) {
      kind = "block";
      present = has_ari_block(req.name);
    } else {
      continue; // Node: deferred to verify_scene_customs (cvcGL)
    }
    if (present)
      continue;
    if (req.required) {
      r.error = std::string("ari: requires custom ") + kind + " '" + req.name +
                "' which is not registered on this system";
      return r; // fail fast — no tree built (r.ok stays false)
    }
    ctx.warn(std::string("ari: optional custom ") + kind + " '" + req.name +
             "' is not registered — it will render as a placeholder / be skipped");
  }

#ifdef CVC_ARIADNE_HAVE_JSONSCHEMA
  // Layer 2: structural validation (before the semantic Layer-3 walk below).
  schema_validate(doc, ctx);
#endif

  r.root = parse_document(ctx, doc);
  if (doc.IsMap()) {
    r.scene = parse_scene(doc["scene"]); // §9: the scene graph, alongside the widgets
    // init: a state_exec script the host runs once at load (cvc::ariadne::run_init).
    // The loader only captures the text — it has no app and never runs the DSL.
    const YAML::Node in = doc["init"];
    if (in && in.IsScalar())
      r.init_script = in.Scalar();
    else if (in && !in.IsScalar())
      ctx.warn("ari: init: must be a scalar state_exec script string — ignored");
  }
  if (r.meta.min_libcvc.empty())
    ctx.warn("ari: no meta.min_libcvc declared — the provenance gate is skipped "
             "(roadmap §3.1a asks every .ari to declare it)");
  r.warnings = std::move(ctx.warnings);

  // Custom top-level blocks (register_ari_block): dispatch any non-built-in key with a
  // registered parser. Run AFTER warnings are moved, so a parser may append to
  // r.warnings; copy the parser out from under the lock, then call it unlocked.
  if (doc.IsMap())
    for (const auto &kv : doc) {
      if (!kv.first.IsScalar())
        continue;
      const std::string key = kv.first.Scalar();
      if (is_builtin_block(key))
        continue;
      AriBlockParser parser;
      {
        std::lock_guard<std::mutex> lock(block_mutex());
        auto it = block_registry().find(key);
        if (it != block_registry().end())
          parser = it->second;
      }
      if (parser)
        parser(to_value(kv.second), r);
    }

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
