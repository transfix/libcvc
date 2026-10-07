// Ariadne — the .ari YAML loader (see the header). Parses a .ari document into a
// cvc::ariadne Widget tree. Pure libcvc: it produces the backend-neutral tree,
// unaware of any UI toolkit. Optional yaml-cpp dependency — when libcvc is built
// without it (CVC_ARIADNE_HAVE_YAML undefined), load_* return an error and
// have_yaml() is false, so the rest of Ariadne still builds and runs.
//
// Slice 2 adds the meta block (§3.1a): document provenance and the min_libcvc
// LOAD GATE (checked first — a document that needs a newer libcvc fails to load
// rather than half-rendering), plus semantic validation surfaced as warnings.

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cvc/ariadne/ariadne.h> // has_widget_type (custom-widget load-time check)
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/uri.h> // §12/§13 import: resolve library URIs (file/state/http)
#include <cvc/core/config.h> // CVC_VERSION_STRING (generated from project(VERSION))
// §12 channel lint: parse a script (parser.h) and walk its value_t / symbol / list_ptr AST
// (types.h) for the static msg-* channel references.
#include <cvc/core/state_exec/parser.h>
#include <cvc/core/state_exec/types.h>
#include <filesystem> // §12 import: dirname of a resolved library for nested bases
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <set>
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
  return k == "meta" || k == "menubar" || k == "windows" || k == "overlays" || k == "root" ||
         k == "children" || k == "scene" || k == "customs" || k == "init" || k == "on_tick" ||
         k == "on_key" || k == "on_pointer" || k == "units" || k == "import" || k == "channels" ||
         k == "lint";
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

// §12.5: a file's mtime as an int64 (implementation-defined ticks — only equality/inequality is
// meaningful), or 0 when the file is missing/unreadable. Always available (no yaml needed).
std::int64_t file_mtime(const std::string &path) {
  std::error_code ec;
  const std::filesystem::file_time_type t = std::filesystem::last_write_time(path, ec);
  return ec ? 0 : static_cast<std::int64_t>(t.time_since_epoch().count());
}

// §12.5 hot-reload polling — see loader.h. Always available (no yaml needed): compares each
// source's on-disk mtime to a caller-held stamp. First sight of a path records its stamp and is
// NOT a change (the caller just loaded it — or, when `stamps` is seeded with LoadResult::
// source_stamps, a difference from the load-time stamp IS caught); a later differing mtime is a
// change. A missing/unreadable file is skipped (a mid-write flicker must not thrash a reload).
// Prunes stamps for paths no longer in `sources` so the map stays bounded and a re-added path is
// re-baselined. Never throws.
bool sources_changed(const std::vector<std::string> &sources,
                     std::map<std::string, std::int64_t> &stamps) {
  bool changed = false;
  for (const std::string &p : sources) {
    const std::int64_t mtime = file_mtime(p);
    if (mtime == 0)
      continue; // missing/unreadable -> treat as unchanged
    const auto it = stamps.find(p);
    if (it == stamps.end())
      stamps.emplace(p, mtime); // baseline; not a change
    else if (it->second != mtime) {
      it->second = mtime;
      changed = true;
    }
  }
  // Prune stamps for sources that dropped out of the graph (bounds growth; re-baselines a path
  // that later returns). Walk whenever there are stamps — NOT gated on size, because a live source
  // that is missing/unreadable this poll is skipped above (never stamped) yet still occupies a
  // `sources` slot, so a size comparison can hide a genuinely-stale entry.
  if (!stamps.empty()) {
    const std::set<std::string> live(sources.begin(), sources.end());
    for (auto it = stamps.begin(); it != stamps.end();)
      it = live.count(it->first) ? std::next(it) : stamps.erase(it);
  }
  return changed;
}

#ifdef CVC_ARIADNE_HAVE_YAML

namespace {

struct Ctx {
  std::vector<std::string> warnings;
  void warn(std::string m) { warnings.push_back(std::move(m)); }

  // §12 modularization: in-document reusable widget templates (the `units:` block), stored as
  // raw YAML; an `include:` clones the template, substitutes its args (`{name}` tokens) into
  // the cloned scalar values, and parses it. `expanding` holds the unit names currently on the
  // expansion stack — refusing to re-enter one catches BOTH direct/indirect recursion (a→b→a)
  // AND a fan-out bomb (a unit including itself N times), where a mere depth cap would let the
  // count explode exponentially.
  std::map<std::string, YAML::Node> units;
  std::set<std::string> expanding;

  // §12 cross-file libraries: `import:` merges another .ari file's units in. base_dir is the
  // importing file's directory (relative imports resolve against it; empty for load_string).
  // imported holds the canonical paths already pulled in — dedup + cycle guard (a → b → a).
  std::string base_dir;
  std::set<std::string> imported;

  // §12 module mounts: `load:` mounts a fragment as an ISOLATED sub-module (its own units, its
  // own chroot sub-prefix). `mounting` is the resolved-URI stack — refusing a canonical already
  // on it stops a mount cycle (a→b→a) and, with a depth cap, a runaway chain. `mount_ids` are
  // the mount ids (`as:`) already used AT THIS level, so a default/duplicate id is disambiguated
  // rather than colliding two fragments onto one sub-prefix. `mount_count` is a document-wide
  // SHARED total-mount counter (the depth cap alone lets a branching graph fan out K^depth, since
  // the cycle guard only blocks re-mounting an ANCESTOR, not sibling reuse) — a fresh sub-Ctx
  // copies the shared_ptr, so the count aggregates across the whole mount graph.
  std::set<std::string> mounting;
  std::set<std::string> mount_ids;
  std::shared_ptr<int> mount_count;

  // §12.5 hot-reload: the local FILE paths read across this document's whole import/mount graph,
  // shared (like mount_count) so a fresh sub-Ctx contributes to the one list. Copied into
  // LoadResult::sources for a host to watch. state/http canonicals are skipped (not watchable).
  // source_stamps captures each file's mtime AT READ TIME (paired with sources) -> LoadResult, so
  // a host can seed the change-poll baseline with load-time values.
  std::shared_ptr<std::vector<std::string>> sources;
  std::shared_ptr<std::map<std::string, std::int64_t>> source_stamps;
};

// §12.5: record a resolved FILE canonical for hot-reload watching — skip a state/http canonical
// (has "://", not file-watchable) and dedup. Shared across the whole import/mount graph.
void note_source(Ctx &ctx, const std::string &canonical) {
  if (!ctx.sources || canonical.empty() || canonical.find("://") != std::string::npos)
    return;
  std::vector<std::string> &v = *ctx.sources;
  if (std::find(v.begin(), v.end(), canonical) == v.end()) {
    v.push_back(canonical);
    if (ctx.source_stamps) // stamp its mtime at read time for the hot-reload baseline
      (*ctx.source_stamps)[canonical] = file_mtime(canonical);
  }
}

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
Widget expand_include(Ctx &ctx, const YAML::Node &n);   // §12 include: expand a units: template
Widget expand_load(Ctx &ctx, const YAML::Node &n);      // §12 load: mount an isolated fragment
Widget parse_document(Ctx &ctx, const YAML::Node &doc); // §12 load: parses a mounted fragment
Value to_value(const YAML::Node &n); // defined below; used by the Kind::Custom fallback

// Widget keys the loader consumes directly; everything else on a custom widget flows
// into Widget::props for a registered emit fn to read.
inline bool known_widget_key(const std::string &k) {
  return k == "type" || k == "title" || k == "label" || k == "widget" || k == "bind" || k == "on" ||
         k == "on_click" || k == "on_hover" || k == "on_drag" || k == "on_drag_start" ||
         k == "on_drag_end" || k == "children" || k == "items" || k == "id" ||
         k == "visible_when" || k == "enabled_when" || k == "disabled_when" || k == "tooltip" ||
         k == "repeat";
}

std::vector<Widget> parse_seq(Ctx &ctx, const YAML::Node &seq) {
  std::vector<Widget> out;
  if (seq && seq.IsSequence())
    for (const YAML::Node &item : seq)
      out.push_back(parse_widget(ctx, item));
  return out;
}

std::string widget_type(const YAML::Node &n, std::string &label) {
  static const char *kKeys[] = {"group",    "menubar",    "menu",         "menu_item",
                                "window",   "overlay",    "panel",        "text",
                                "checkbox", "slider_int", "slider_float", "combo",
                                "color",    "image",      "button",       "separator"};
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

  // §12 modularization: `include: <unit>` instantiates an in-document `units:` template with
  // its args substituted. Handled before the widget-type dispatch (include is not a widget kind).
  if (n["include"].IsDefined())
    return expand_include(ctx, n);
  // §12 module mount: `load: <uri>` mounts an external .ari fragment as an isolated, scoped
  // sub-module. Also handled before the widget-type dispatch (load is not a widget kind).
  if (n["load"].IsDefined())
    return expand_load(ctx, n);

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
    w.size = parse_size(n["size"]);                  // §3.0.3b window sizing (%/px/auto)
    w.layout = parse_layout(n["layout"]);            // §3.0.3b window-body layout (grid/tracks)
    w.frame_border = parse_frame_border(n["frame"]); // §3.0.3b border width
    w.closable = flag(n, "closable", false);         // a close (X) button + tree.<id>.open state
    return w;
  }
  if (type == "text") {
    Widget w = has(n, "bind") ? text_bound(label, str(n, "bind")) : text(label);
    // Optional fixed tint: `color: [r, g, b]` in [0,1] (same sequence convention as a layout
    // border colour). A colour-capable backend renders it; others fall back to plain text.
    const YAML::Node col = n["color"];
    if (col && col.IsSequence() && col.size() >= 3)
      w = with_text_color(w, col[0].as<float>(1.0f), col[1].as<float>(1.0f),
                          col[2].as<float>(1.0f));
    return w;
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
    // options: a SEQUENCE is a static list; a SCALAR is a §4 computed expression (a list)
    // re-evaluated each frame.
    std::string opts_expr;
    if (o && o.IsSequence()) {
      for (const YAML::Node &e : o)
        if (e.IsScalar())
          opts.push_back(e.Scalar());
    } else if (o && o.IsScalar()) {
      opts_expr = o.Scalar();
    }
    if (opts.empty() && opts_expr.empty())
      ctx.warn("ari: combo '" + label + "' has no options:");
    // values: a parallel list mapping each option label -> a stored value (e.g. an int key). Each
    // entry is stringified (a YAML int/float/string all become the stored string).
    std::vector<std::string> values;
    if (const YAML::Node v = n["values"]; v && v.IsSequence())
      for (const YAML::Node &e : v)
        if (e.IsScalar())
          values.push_back(e.Scalar());
    Widget w = combo(label, bind, std::move(opts), str(n, "default", str(n, "def")));
    w.options_expr = opts_expr;
    if (!values.empty() && !opts_expr.empty())
      ctx.warn("ari: combo '" + label +
               "' has both computed options: and values: — values ignored");
    else if (!values.empty() && values.size() != w.options.size())
      ctx.warn("ari: combo '" + label + "' has " + std::to_string(values.size()) + " values: for " +
               std::to_string(w.options.size()) + " options: — value mapping ignored");
    else
      w.values = std::move(values);
    return w;
  }
  if (type == "color") { // §G7 an RGB picker bound to an "r,g,b" key
    const std::string bind = str(n, "bind");
    check_bind(ctx, "color", label, bind);
    return color_widget(label, bind, str(n, "default", str(n, "def")));
  }
  if (type == "image") { // a host-published image viewer (a live raster)
    Widget w = image_widget(label, str(n, "src"), num(n, "size", 256.0));
    w.bind = str(n, "bind"); // optional: the image name comes from this key instead of src
    return w;
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
// True if any widget in the subtree (excluding the root) carries a `repeat`.
bool any_descendant_repeats(const Widget &w) {
  for (const Widget &c : w.children) {
    if (!c.repeat.empty() || any_descendant_repeats(c))
      return true;
  }
  return false;
}

Widget parse_widget(Ctx &ctx, const YAML::Node &n) {
  Widget w = parse_widget_impl(ctx, n);
  // Assign each common field only when the node actually carries it. For a plain widget the
  // node IS its own definition (so an absent key = the "" default, unchanged). But for an
  // `include:` node, parse_widget_impl already returned the EXPANDED unit widget with these
  // fields set from the TEMPLATE — an unconditional overwrite here would erase them (the
  // include node has none). Present-guarding preserves the template's fields; an include site
  // that DOES set one overrides for that instance.
  if (n.IsMap()) {
    if (has(n, "visible_when"))
      w.visible_when = str(n, "visible_when");
    if (has(n, "enabled_when"))
      w.enabled_when = str(n, "enabled_when");
    if (has(n, "disabled_when"))
      w.disabled_when = str(n, "disabled_when");
    if (has(n, "tooltip"))
      w.tooltip = str(n, "tooltip"); // literal, or a computed expr (starts with '(')
    if (has(n, "repeat"))
      w.repeat = str(n, "repeat"); // §3: a count expression -> emit this template N times
    if (has(n, "on_click"))
      w.on_click = str(n, "on_click"); // §4.6 widget-level click handler (any kind); like `on`
    if (has(n, "on_hover"))
      w.on_hover = str(n, "on_hover"); // §4.6 continuous: fires each frame the item is hovered
    if (has(n, "on_drag"))
      w.on_drag = str(n, "on_drag"); // §4.6 continuous: fires each frame the item is dragged
    if (has(n, "on_drag_start"))
      w.on_drag_start = str(n, "on_drag_start"); // §4.6 edge: gesture begin (item activated)
    if (has(n, "on_drag_end"))
      w.on_drag_end = str(n, "on_drag_end"); // §4.6 edge: gesture end (item deactivated)
  }
  // Nested repeat is single-level: only one `{i}` token exists, so an inner repeat cannot
  // reference the outer index. Surface it rather than let it silently alias state.
  if (!w.repeat.empty() && any_descendant_repeats(w))
    ctx.warn("ari: nested 'repeat' is not supported (a single {i} index) — the inner repeat "
             "cannot see the outer index");
  return w;
}

// §12: merge a `units:` map (name -> raw widget template) into the unit table. Stored RAW so
// an include: can substitute its args before parsing, and so units may be defined in any order.
void merge_units(Ctx &ctx, const YAML::Node &unitsNode, bool warn_collision) {
  if (!unitsNode || !unitsNode.IsMap())
    return;
  for (const auto &kv : unitsNode) {
    if (!kv.first.IsScalar())
      continue;
    const std::string name = kv.first.Scalar();
    if (warn_collision && ctx.units.count(name))
      ctx.warn("ari: import: unit '" + name + "' is defined by more than one library — last wins");
    ctx.units[name] = kv.second;
  }
}

// §12/§13: process a document's `import:` (a URI or list of URIs) — each names another .ari
// library whose units merge into this document's table. Resolved through the shared
// uri_resolver (file:// built in; state://, http(s):// via a registered handler), relative to
// `base_dir`. Recurses into an imported library's OWN imports (based at its directory), and
// dedups/cycle-guards on the resolved canonical identity (a → b → a terminates).
// The `imported` canonical set stops cycles and exact repeats, but not a linear chain of DISTINCT
// fragments (a→b→c→…), which recurses one native C++ frame per link — a deep enough chain would
// overflow the stack. Bound the depth (and the aggregate library count) explicitly, mirroring the
// include: expansion guard's intent, and warn rather than crash.
constexpr int kMaxImportDepth = 32;
constexpr std::size_t kMaxImports = 4096;

void process_imports(Ctx &ctx, const YAML::Node &doc, const std::string &base_dir, int depth = 0) {
  if (!doc.IsMap())
    return;
  if (depth > kMaxImportDepth) {
    ctx.warn("ari: import chain exceeds the maximum depth (" + std::to_string(kMaxImportDepth) +
             ") — deeper imports skipped");
    return;
  }
  const YAML::Node imp = doc["import"];
  std::vector<std::string> uris;
  if (imp && imp.IsScalar())
    uris.push_back(imp.Scalar());
  else if (imp && imp.IsSequence())
    for (const auto &p : imp)
      if (p.IsScalar())
        uris.push_back(p.Scalar());
  for (const std::string &uri : uris) {
    if (ctx.imported.size() >= kMaxImports) {
      ctx.warn("ari: import limit reached (" + std::to_string(kMaxImports) +
               " libraries) — remaining imports skipped");
      break;
    }
    const UriResult res = resolve(uri, base_dir);
    if (!res.ok) {
      ctx.warn("ari: import '" + uri + "' could not be resolved: " + res.error);
      continue;
    }
    note_source(ctx, res.canonical); // §12.5 watch this imported file for hot-reload
    if (!ctx.imported.insert(res.canonical).second)
      continue; // already imported (dedup) or on the stack (cycle) — skip
    YAML::Node sub;
    try {
      sub = YAML::Load(res.content);
    } catch (const std::exception &e) {
      ctx.warn("ari: import '" + uri + "' parse error: " + e.what());
      continue;
    }
    const std::string sub_base = std::filesystem::path(res.canonical).parent_path().string();
    process_imports(ctx, sub, sub_base, depth + 1); // the library's own imports, based at its dir
    merge_units(ctx, sub["units"], /*warn=*/true);  // then its units (import-vs-import warns)
  }
}

// §12: collect this document's reusable units — imported libraries first (so local units may
// build on them), then the document's own `units:` last (a local unit silently overrides an
// imported one of the same name — the customization case).
void parse_units(Ctx &ctx, const YAML::Node &doc) {
  process_imports(ctx, doc, ctx.base_dir);
  if (doc.IsMap())
    merge_units(ctx, doc["units"], /*warn=*/false);
}

// §12: substitute `{name}` arg tokens in a scalar in a SINGLE pass — a substituted value is
// never re-scanned (no order-dependent double substitution), and an unknown `{x}` is left
// literal. (Distinct from repeat's runtime `{i}`: this is load-time, over named args.)
std::string substitute_tokens(const std::string &s,
                              const std::map<std::string, std::string> &args) {
  std::string out;
  out.reserve(s.size());
  for (std::size_t i = 0; i < s.size();) {
    if (s[i] == '{') {
      const std::size_t close = s.find('}', i);
      if (close != std::string::npos) {
        const auto it = args.find(s.substr(i + 1, close - i - 1));
        if (it != args.end()) {
          out += it->second;
          i = close + 1;
          continue;
        }
      }
    }
    out += s[i++];
  }
  return out;
}

// Recursively substitute args into a (cloned) YAML node's SCALAR values. Mutating a cloned
// tree — rather than splicing into dumped text — avoids YAML re-quoting/escaping hazards (a
// value with a backslash or quote no longer breaks the re-parse) and touches each scalar once.
void substitute_node(YAML::Node node, const std::map<std::string, std::string> &args) {
  if (node.IsScalar()) {
    node = substitute_tokens(node.Scalar(), args);
  } else if (node.IsSequence()) {
    for (std::size_t i = 0; i < node.size(); ++i)
      substitute_node(node[i], args);
  } else if (node.IsMap()) {
    for (auto it = node.begin(); it != node.end(); ++it)
      substitute_node(it->second, args); // values only (keys are structural)
  }
}

// §12: expand `include: <unit>` — instantiate a units: template with its `args:` substituted.
Widget expand_include(Ctx &ctx, const YAML::Node &n) {
  const std::string name = str(n, "include");
  const auto it = ctx.units.find(name);
  if (it == ctx.units.end()) {
    ctx.warn("ari: include: unknown unit '" + name + "' (no such units: entry)");
    return group({});
  }
  // Refuse to re-enter a unit already on the expansion stack: this catches direct/indirect
  // recursion (a→b→a) AND a fan-out bomb (a unit including itself N times) — a depth cap alone
  // would let the expansion count explode exponentially before tripping.
  if (!ctx.expanding.insert(name).second) {
    ctx.warn("ari: include: recursive unit '" + name + "' — expansion refused");
    return group({});
  }
  std::map<std::string, std::string> args;
  const YAML::Node a = n["args"];
  if (a && a.IsMap())
    for (const auto &kv : a)
      if (kv.first.IsScalar() && kv.second.IsScalar())
        args[kv.first.Scalar()] = kv.second.Scalar();
  YAML::Node inst = YAML::Clone(it->second); // never mutate the stored template
  substitute_node(inst, args);
  Widget w = parse_widget(ctx, inst);
  ctx.expanding.erase(name);
  // §12/§3.0.3b geometry override at the include site: a window `include:` may carry its own
  // pos/size/layout/frame to place and size the shared panel WITHOUT forking the unit — so a demo
  // arranges reusable panels (e.g. tile stage_lighting / camera_controls / sim_transport) while the
  // component stays layout-agnostic. Applied only when the instantiated root is a window/overlay
  // (the panel case); each key overrides only if present, so an unspecified axis keeps the unit's.
  if (w.kind == Kind::Window) {
    if (n["pos"])
      parse_pos(n, w);
    if (n["size"])
      w.size = parse_size(n["size"]);
    if (n["layout"])
      w.layout = parse_layout(n["layout"]);
    if (n["frame"])
      w.frame_border = parse_frame_border(n["frame"]);
    if (n["closable"])
      w.closable = flag(n, "closable", false);
  }
  return w;
}

// §12 load: mount depth cap — a fragment that loads a fragment that loads … this deep is
// almost certainly a mistake or an attack; refuse past it (the cycle guard handles a → b → a).
constexpr std::size_t kMaxMountDepth = 32;
// Document-wide cap on TOTAL mounts (depth × breadth) — a branching mount graph past this is
// pathological or hostile. Bounds the K^depth fan-out the per-chain depth cap cannot see.
constexpr int kMaxMounts = 256;

// A mount id must be a single, clean state-path segment (it becomes `includes.<id>`), so map
// anything that is not [A-Za-z0-9_] to '_' and never let it be empty or start with a digit.
std::string sanitize_mount_id(const std::string &raw) {
  std::string out;
  out.reserve(raw.size());
  for (char c : raw)
    out.push_back((std::isalnum(static_cast<unsigned char>(c)) || c == '_') ? c : '_');
  if (out.empty())
    out = "mount";
  if (std::isdigit(static_cast<unsigned char>(out.front())))
    out.insert(out.begin(), '_');
  return out;
}

// Default a mount id from the resource. For a PATH-LIKE scheme (file/bare, http, https) it is the
// last path segment minus its filename extension ("file://panels/rf.ari" → "rf",
// "http://h/rf.ari" → "rf" — the codebase treats an http URL as an extensioned file). For an
// OPAQUE-IDENTITY scheme (state, and any custom one) the path is a dotted identity, not a
// filename, so the trailing ".x" is a real segment — keep it whole ("state://ui.libs.forms" →
// "ui.libs.forms", so ui.libs.forms and ui.libs.panels stay distinct). The caller sanitizes
// ('.' → '_'). A bare/empty tail falls back to "mount".
std::string default_mount_id(const std::string &uri) {
  std::string s = uri;
  bool strip_ext = true; // a bare/relative path is the file scheme (§13.1) → path-like
  if (const std::size_t scheme = s.find("://"); scheme != std::string::npos) {
    std::string sch = s.substr(0, scheme);
    std::transform(sch.begin(), sch.end(), sch.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    strip_ext = (sch == "file" || sch == "http" || sch == "https");
    s = s.substr(scheme + 3);
  }
  if (const std::size_t q = s.find('?'); q != std::string::npos)
    s = s.substr(0, q);
  if (const std::size_t slash = s.find_last_of('/'); slash != std::string::npos)
    s = s.substr(slash + 1);
  if (strip_ext) // a path-like scheme carries a filename extension to strip
    if (const std::size_t dot = s.find_last_of('.'); dot != std::string::npos && dot != 0)
      s = s.substr(0, dot);
  return s.empty() ? "mount" : s;
}

// §12: mount a `load: <uri>` fragment — resolve + parse another .ari as an ISOLATED module and
// return a scoped wrapper Group. The wrapper carries `scope = includes.<as>` (§12.1), so the
// fragment's binds, reactive predicates, and (later) init: resolve under that sub-prefix; the
// mount widget's OWN reactive fields (from this load: block, applied by parse_widget) stay in
// the parent scope. `args:` substitute before parse like include:. The fragment parses in a
// FRESH sub-Ctx (its units are its own, not the host's), based at its own directory so ITS
// relative imports/loads resolve there. A resolved-URI cycle guard + depth cap stop recursion.
Widget expand_load(Ctx &ctx, const YAML::Node &n) {
  const std::string uri = str(n, "load");
  if (uri.empty()) {
    ctx.warn("ari: load: entry has no URI");
    return group({});
  }
  const UriResult res = resolve(uri, ctx.base_dir);
  if (!res.ok) {
    ctx.warn("ari: load '" + uri + "' could not be resolved: " + res.error);
    return group({});
  }
  note_source(ctx, res.canonical); // §12.5 watch this mounted fragment for hot-reload
  if (ctx.mounting.count(res.canonical)) {
    ctx.warn("ari: load: mount cycle on '" + res.canonical + "' — refused");
    return group({});
  }
  if (ctx.mounting.size() >= kMaxMountDepth) {
    ctx.warn("ari: load: mount chain exceeds the maximum depth (" + std::to_string(kMaxMountDepth) +
             ") — refused");
    return group({});
  }
  // Aggregate (document-wide) mount cap: the depth guard alone lets a BRANCHING mount graph fan
  // out K^depth (the cycle guard only blocks re-mounting an ancestor, not sibling reuse). Count
  // total mounts on a shared counter and refuse past the cap — the load: analogue of kMaxImports.
  if (!ctx.mount_count)
    ctx.mount_count = std::make_shared<int>(0);
  if (*ctx.mount_count >= kMaxMounts) {
    ctx.warn("ari: load: mount limit reached (" + std::to_string(kMaxMounts) +
             " fragments) — refused");
    return group({});
  }
  ++*ctx.mount_count;
  YAML::Node frag;
  try {
    frag = YAML::Load(res.content);
  } catch (const std::exception &e) {
    ctx.warn("ari: load '" + uri + "' parse error: " + e.what());
    return group({});
  }
  // Args substitute into the cloned fragment before parse (same single-pass rule as include:).
  std::map<std::string, std::string> args;
  const YAML::Node a = n["args"];
  if (a && a.IsMap())
    for (const auto &kv : a)
      if (kv.first.IsScalar() && kv.second.IsScalar())
        args[kv.first.Scalar()] = kv.second.Scalar();
  substitute_node(frag, args);
  // Parse the fragment in an ISOLATED sub-context: fresh units (module boundary), based at the
  // fragment's own directory, carrying the mount stack (with this URI pushed) + the shared mount
  // counter for the cycle/depth/aggregate guards. Its warnings bubble up to the host document.
  Ctx sub;
  sub.base_dir = std::filesystem::path(res.canonical).parent_path().string();
  sub.mounting = ctx.mounting;
  sub.mounting.insert(res.canonical);
  sub.mount_count = ctx.mount_count;
  sub.sources = ctx.sources; // §12.5 the fragment's own imports/mounts feed the same watch list
  sub.source_stamps = ctx.source_stamps;
  parse_units(sub, frag); // the fragment's OWN import:/units:
  Widget frag_root = parse_document(sub, frag);
  for (const std::string &w : sub.warnings)
    ctx.warn(w);
  // Mount id: explicit `as:`, else derived from the resource; sanitized, and disambiguated
  // against sibling mounts at this level so two fragments never collide onto one sub-prefix.
  // Reserved only NOW — after the fragment has actually loaded+parsed — so a mount that failed
  // above never burns an id (which would spuriously bump a later valid sibling to <id>_2).
  std::string as = sanitize_mount_id(has(n, "as") ? str(n, "as") : default_mount_id(uri));
  if (!ctx.mount_ids.insert(as).second) {
    std::string uniq;
    for (int i = 2; uniq.empty(); ++i) {
      const std::string cand = as + "_" + std::to_string(i);
      if (ctx.mount_ids.insert(cand).second)
        uniq = cand;
    }
    ctx.warn("ari: load: duplicate mount id '" + as + "' — using '" + uniq + "'");
    as = uniq;
  }
  // The wrapper carries the scope; the fragment is its child so the fragment's own root reactive
  // fields evaluate at the sub-prefix, while the parent's load:-block fields land on the wrapper.
  Widget wrapper = group({std::move(frag_root)});
  wrapper.scope = "includes." + as;
  // §12: carry the fragment's own init: so the Runtime can run it once at the sub-prefix (the
  // loader is state-free; it captures the script, the Runtime executes it). A non-scalar init: is
  // a malformed script — warn (matching the top-level document path), never silently drop it.
  if (frag.IsMap() && frag["init"]) {
    if (frag["init"].IsScalar())
      wrapper.init_script = frag["init"].Scalar();
    else
      ctx.warn("ari: load '" + uri +
               "': the fragment's init: must be a scalar state_exec script string — ignored");
  }
  // §12 link: parent-scope holes — each entry becomes a transparent link node the Runtime plants
  // inside the mount's sub-prefix (created there, not here — the loader is state-free). Value form
  // `name: target` (rw); map form `name: { to: target, mode: ro|rw }`.
  if (const YAML::Node lk = n["link"]; lk && lk.IsMap()) {
    for (const auto &kv : lk) {
      if (!kv.first.IsScalar())
        continue;
      LinkHole h;
      h.name = kv.first.Scalar();
      if (const YAML::Node v = kv.second; v.IsScalar()) {
        h.target = v.Scalar(); // scalar form defaults rw
      } else if (v.IsMap()) {
        h.target = str(v, "to");
        // Fail CLOSED: only the exact token "rw" (case-insensitive) is writable; a missing /
        // mistyped mode is treated as read-only, and an unrecognized one warns — a security
        // field must never fail open (a typo silently granting write).
        std::string mode = str(v, "mode", "rw");
        std::transform(mode.begin(), mode.end(), mode.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        h.writable = (mode == "rw");
        if (mode != "rw" && mode != "ro")
          ctx.warn("ari: load '" + uri + "': link '" + h.name + "' has unknown mode '" + mode +
                   "' — treated as read-only");
      }
      if (h.name.empty() || h.target.empty()) {
        ctx.warn("ari: load '" + uri + "': link '" + h.name + "' needs a target — skipped");
        continue;
      }
      // A hole name must stay INSIDE the mount's chroot: a leading '/' would make resolve_bind
      // plant it at the app root (escaping the sandbox), so reject it. (Dotted names are fine —
      // they nest within the sub-prefix.)
      if (h.name[0] == '/') {
        ctx.warn("ari: load '" + uri + "': link name '" + h.name +
                 "' must not start with '/' (it would escape the mount) — skipped");
        continue;
      }
      wrapper.links.push_back(std::move(h));
    }
  }
  // §12 channels: grant — the messaging analogue of link:. Each entry grants the mounted child a
  // channel its parent exposes; it LOWERS into a transparent-link hole under the reserved
  // `channels.` subtree, so wire_holes plants it and intrinsics.cpp's resolve_channel follows it (a
  // granted channel resolves to the parent's key). Value form `local: parent` (rw); map form
  // `local: { to: parent, mode: ro|rw }`. Fail-closed on mode, like link:.
  if (const YAML::Node ch = n["channels"]; ch && ch.IsMap()) {
    for (const auto &kv : ch) {
      if (!kv.first.IsScalar())
        continue;
      const std::string local = kv.first.Scalar();
      std::string target;
      bool writable = true;
      if (const YAML::Node v = kv.second; v.IsScalar()) {
        target = v.Scalar();
      } else if (v.IsMap()) {
        target = str(v, "to");
        std::string mode = str(v, "mode", "rw");
        std::transform(mode.begin(), mode.end(), mode.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        writable = (mode == "rw");
        if (mode != "rw" && mode != "ro")
          ctx.warn("ari: load '" + uri + "': channel grant '" + local + "' has unknown mode '" +
                   mode + "' — treated as read-only");
      }
      if (local.empty() || target.empty()) {
        ctx.warn("ari: load '" + uri + "': channel grant '" + local + "' needs a target — skipped");
        continue;
      }
      if (local[0] == '/') {
        ctx.warn("ari: load '" + uri + "': channel grant name '" + local +
                 "' must not start with '/' (it would escape the mount) — skipped");
        continue;
      }
      LinkHole h;
      h.name = "channels." + local;    // planted inside the mount's own channels. subtree
      h.target = "channels." + target; // resolved against the PARENT scope by wire_holes
      h.writable = writable;
      wrapper.links.push_back(std::move(h));
    }
  }
  // §12 needs: the fragment can DECLARE the hole names it expects the mount to grant (a scalar or
  // list) — the parent-scope analogue of §7.6 requires:. A declared need the mount did not wire is
  // surfaced as a warning (fail-safe: the module still renders, but its ungranted name reads its
  // own sandboxed local node), so a module documents and self-checks its imports.
  if (frag.IsMap()) {
    if (const YAML::Node needs = frag["needs"]) {
      std::vector<std::string> need_names;
      if (needs.IsScalar())
        need_names.push_back(needs.Scalar());
      else if (needs.IsSequence())
        for (const auto &nn : needs)
          if (nn.IsScalar())
            need_names.push_back(nn.Scalar());
      for (const std::string &nm : need_names) {
        const bool granted = std::any_of(wrapper.links.begin(), wrapper.links.end(),
                                         [&](const LinkHole &h) { return h.name == nm; });
        if (!granted)
          ctx.warn("ari: load '" + uri + "': fragment needs link '" + nm +
                   "' but the mount granted none — the module will see its own local node");
      }
    }
  }
  return wrapper;
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

void vec2(const YAML::Node &n, const char *key, float out[2]) {
  const YAML::Node v = n[key];
  if (v && v.IsSequence() && v.size() >= 2)
    for (int i = 0; i < 2; ++i)
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
         k == "transform" || k == "fit" || k == "visible" || k == "volren" || k == "volslice" ||
         k == "volume" || k == "shader" || k == "clip" || k == "children";
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
// A shader surface (§9): `shader: <preset>` (a scalar shorthand) OR a map with an optional
// `preset`, a `disable_coord_shift` flag, and `vertex`/`fragment` lists of {at, code} GLSL splices.
SceneShader parse_scene_shader(const YAML::Node &s) {
  SceneShader sh;
  if (!s)
    return sh;
  sh.present = true;
  if (s.IsScalar()) { // `shader: terrain_bump` — name a preset
    sh.preset = s.Scalar();
    return sh;
  }
  if (!s.IsMap())
    return sh;
  if (s["preset"])
    sh.preset = str(s, "preset");
  sh.disable_coord_shift = flag(s, "disable_coord_shift", false);
  const auto stages = [](const YAML::Node &seq, std::vector<SceneShaderStage> &out) {
    if (seq && seq.IsSequence())
      for (const YAML::Node &st : seq) {
        if (!st.IsMap())
          continue;
        SceneShaderStage stage;
        stage.at = str(st, "at");
        stage.code = str(st, "code");
        if (!stage.at.empty())
          out.push_back(stage);
      }
  };
  stages(s["vertex"], sh.vertex);
  stages(s["fragment"], sh.fragment);
  return sh;
}

// Clip planes (§9): `clip: { box, planes: [{origin, normal, offset}], children }`, every coordinate
// in the node's own frame. A non-numeric value leaves that field at its default (the realizer warns
// about what is left unusable: a zero normal, an inverted box).
SceneClip parse_scene_clip(const YAML::Node &c) {
  SceneClip clip;
  if (!c || !c.IsMap())
    return clip;
  clip.present = true;
  const auto dbl = [](const YAML::Node &v, double &out) {
    if (v && v.IsScalar())
      try {
        out = v.as<double>();
        return true;
      } catch (const std::exception &) {
      }
    return false;
  };
  const auto triple = [&](const YAML::Node &v, double out[3]) {
    if (v && v.IsSequence() && v.size() >= 3)
      for (std::size_t i = 0; i < 3; ++i)
        dbl(v[i], out[i]);
  };
  const YAML::Node box = c["box"];
  if (box && box.IsSequence() && box.size() >= 6) {
    clip.has_box = true;
    for (std::size_t i = 0; i < 6; ++i)
      dbl(box[i], clip.box[i]);
  } else if (box && box.IsMap() && box["min"] && box["max"]) {
    clip.has_box = true;
    triple(box["min"], clip.box);
    triple(box["max"], clip.box + 3);
  }
  const YAML::Node planes = c["planes"];
  if (planes && planes.IsSequence())
    for (const YAML::Node &p : planes) {
      if (!p.IsMap())
        continue;
      SceneClipPlane cp;
      triple(p["origin"], cp.origin);
      triple(p["normal"], cp.normal);
      const YAML::Node off = p["offset"];
      if (off && off.IsMap()) { // { bind: <path>, default: <d> }
        cp.offset_bind = str(off, "bind");
        cp.offset = num(off, "default", 0.0);
      } else if (off && off.IsScalar() && !dbl(off, cp.offset)) {
        cp.offset_bind = off.Scalar(); // not a number: a state path
      }
      clip.planes.push_back(cp);
    }
  clip.children = flag(c, "children", false);
  return clip;
}

SceneNode parse_scene_node(const YAML::Node &n) {
  SceneNode sn;
  sn.id = str(n, "node", str(n, "id"));
  sn.type = str(n, "type", "geometry");
  // §13.4: a node's source is a URI (file/state/http/…) resolved through the shared resolver.
  // Accept `source: <uri>` (scalar), `source: { uri: <uri> }`, or the legacy `{ file: <path> }`.
  const YAML::Node src = n["source"];
  if (src) {
    if (src.IsScalar()) {
      sn.source_file = src.Scalar();
    } else if (src.IsMap()) {
      // A procedural primitive (no asset) takes precedence over a file: source: { plane: { size } }
      // (or the shorthand source: { plane: <size> }). Otherwise a URI/file.
      if (const YAML::Node plane = src["plane"]) {
        sn.source_primitive = "plane";
        if (plane.IsMap())
          sn.plane_size = static_cast<float>(num(plane, "size", sn.plane_size));
        else if (plane.IsScalar())
          sn.plane_size = static_cast<float>(plane.as<double>());
      } else if (const YAML::Node sdf = src["sdf"]) {
        // source: { sdf: { mesh: <uri>, dim: N } } — a volren/volslice volume computed as the
        // signed distance field of a mesh (no pre-baked volume asset).
        sn.source_sdf_mesh = str(sdf, "mesh", str(sdf, "file"));
        sn.sdf_dim = static_cast<int>(num(sdf, "dim", sn.sdf_dim));
      } else if (const YAML::Node hf = src["heightfield"]; hf && hf.IsMap()) {
        // source: { heightfield: { size, resolution, layers: [...], colors: [...] } } — a
        // procedural displaced grid mesh (the reusable terrain primitive; no asset).
        sn.has_heightfield = true;
        sn.heightfield.size = static_cast<float>(num(hf, "size", sn.heightfield.size));
        sn.heightfield.resolution =
            static_cast<int>(num(hf, "resolution", sn.heightfield.resolution));
        if (const YAML::Node layers = hf["layers"]; layers && layers.IsSequence())
          for (const YAML::Node &l : layers) {
            SceneHeightLayer hl;
            hl.kind = str(l, "kind", str(l, "type", ""));
            hl.amplitude = static_cast<float>(num(l, "amplitude", hl.amplitude));
            hl.radius = static_cast<float>(num(l, "radius", hl.radius));
            hl.wavelength = static_cast<float>(num(l, "wavelength", hl.wavelength));
            vec2(l, "direction", hl.direction);
            hl.phase = static_cast<float>(num(l, "phase", hl.phase));
            sn.heightfield.layers.push_back(hl);
          }
        if (const YAML::Node colors = hf["colors"]; colors && colors.IsSequence())
          for (const YAML::Node &c : colors) {
            SceneHeightColorBand band;
            band.max_height =
                static_cast<float>(num(c, "max_height", num(c, "below", band.max_height)));
            const YAML::Node cc = c["color"];
            if (cc && cc.IsSequence() && cc.size() >= 3)
              for (int i = 0; i < 3; ++i)
                band.color[i] = static_cast<float>(cc[i].as<double>());
            sn.heightfield.colors.push_back(band);
          }
      } else {
        sn.source_file = str(src, "uri", str(src, "file"));
      }
    }
  }
  const YAML::Node mat = n["material"];
  if (mat && mat.IsMap()) {
    sn.has_material = true;
    const YAML::Node col = mat["color"];
    if (col && col.IsSequence() && col.size() >= 3)
      for (int i = 0; i < 3; ++i)
        sn.color[i] = static_cast<float>(col[i].as<double>());
    sn.ambient = static_cast<float>(num(mat, "ambient", sn.ambient));
    sn.diffuse = static_cast<float>(num(mat, "diffuse", sn.diffuse));
    sn.use_single_color = flag(mat, "single_color", true);
    if (mat["specular"] || mat["specular_power"]) {
      sn.has_specular = true;
      sn.specular = static_cast<float>(num(mat, "specular", sn.specular));
      sn.specular_power = static_cast<float>(num(mat, "specular_power", sn.specular_power));
    }
    if (const YAML::Node tex = mat["texture"]) {
      // material: { texture: <uri> } or { texture: { uri|file: <uri> } }
      if (tex.IsScalar())
        sn.material_texture = tex.Scalar();
      else if (tex.IsMap())
        sn.material_texture = str(tex, "uri", str(tex, "file"));
    }
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
  const YAML::Node fit = n["fit"];
  if (fit && fit.IsMap()) {
    sn.has_fit = true;
    sn.fit_height = static_cast<float>(num(fit, "height", sn.fit_height));
    sn.fit_up_y = (str(fit, "up", "z") == "y"); // up: y -> rotate a Y-up mesh to Z-up first
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
  const YAML::Node vol = n["volume"];
  if (vol && vol.IsMap()) {
    sn.has_volume = true;
    if (vol["shaded"]) {
      sn.volume.has_shaded = true;
      sn.volume.shaded = flag(vol, "shaded", true);
    }
    sn.volume.tf = parse_scene_tf(vol["transfer_function"]);
  }
  if (const YAML::Node sh = n["shader"])
    sn.shader = parse_scene_shader(sh);
  sn.clip = parse_scene_clip(n["clip"]);
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
  // rig: TUNING — only meaningful when `rig` is set; each block gates its StageLighting setter.
  if (const YAML::Node stage = n["stage"]; stage && stage.IsMap()) {
    sl.has_stage = true;
    vec3(stage, "center", sl.stage_center);
    sl.stage_radius = static_cast<float>(num(stage, "radius", sl.stage_radius));
  }
  if (const YAML::Node key = n["key"]; key && key.IsMap()) {
    sl.has_key = true;
    sl.key_intensity = static_cast<float>(num(key, "intensity", sl.key_intensity));
    sl.key_azimuth = static_cast<float>(num(key, "azimuth", sl.key_azimuth));
    sl.key_elevation = static_cast<float>(num(key, "elevation", sl.key_elevation));
    sl.key_cone = static_cast<float>(num(key, "cone", sl.key_cone));
  }
  if (n["fill"]) {
    sl.has_fill = true;
    sl.fill = static_cast<float>(num(n, "fill", sl.fill));
  }
  if (n["back"]) {
    sl.has_back = true;
    sl.back = static_cast<float>(num(n, "back", sl.back));
  }
  if (n["warmth"]) {
    sl.has_warmth = true;
    sl.warmth = static_cast<float>(num(n, "warmth", sl.warmth));
  }
  if (n["environment"]) {
    sl.has_environment = true;
    sl.environment = static_cast<float>(num(n, "environment", sl.environment));
  }
  if (n["ambient"]) { // the rig's fill-ambient (a non-rig light has no ambient of its own)
    sl.has_rig_ambient = true;
    sl.rig_ambient = static_cast<float>(num(n, "ambient", sl.rig_ambient));
  }
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
    if (sh["resolution"]) {
      sc.has_shadow_resolution = true;
      sc.shadow_resolution = static_cast<int>(num(sh, "resolution", sc.shadow_resolution));
    }
    if (sh["update_interval"]) {
      sc.has_shadow_interval = true;
      sc.shadow_interval = static_cast<int>(num(sh, "update_interval", sc.shadow_interval));
    }
  }
  if (s["chrome"]) { // scene-level: the diagnostic grid/axis/bounding-box chrome
    sc.has_chrome = true;
    sc.chrome_visible = flag(s, "chrome", true);
  }
  if (const YAML::Node bg = s["background"]) {
    // background: [r,g,b] (solid) OR { top: [r,g,b], bottom: [r,g,b] } (vertical gradient).
    sc.has_background = true;
    if (bg.IsSequence() && bg.size() >= 3) {
      for (int i = 0; i < 3; ++i)
        sc.background_top[i] = sc.background_bottom[i] = static_cast<float>(bg[i].as<double>());
      sc.background_gradient = false;
    } else if (bg.IsMap()) {
      sc.background_gradient = true;
      if (const YAML::Node top = bg["top"]; top && top.IsSequence() && top.size() >= 3)
        for (int i = 0; i < 3; ++i)
          sc.background_top[i] = static_cast<float>(top[i].as<double>());
      if (const YAML::Node bot = bg["bottom"]; bot && bot.IsSequence() && bot.size() >= 3)
        for (int i = 0; i < 3; ++i)
          sc.background_bottom[i] = static_cast<float>(bot[i].as<double>());
    }
  }
  if (const YAML::Node ck = s["clock"]; ck && ck.IsMap()) {
    // clock: { scale, paused, speed_key, paused_key, time_key, tick_key } — the scene's simulation
    // clock (SceneClock). All fields optional: a bare `clock: {}` wires to sim_transport.ari with
    // the defaults. An explicit empty string for a *_key disables that lane (no steer / no
    // publish).
    sc.clock.present = true;
    sc.clock.scale = num(ck, "scale", sc.clock.scale);
    sc.clock.paused = flag(ck, "paused", sc.clock.paused);
    if (ck["speed_key"])
      sc.clock.speed_key = str(ck, "speed_key");
    if (ck["paused_key"])
      sc.clock.paused_key = str(ck, "paused_key");
    if (ck["time_key"])
      sc.clock.time_key = str(ck, "time_key");
    if (ck["tick_key"])
      sc.clock.tick_key = str(ck, "tick_key");
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
// §12 channel scoping: parse the top-level `channels:` declaration — a SEQUENCE of channel names,
// each a bare scalar (`- nav.done`) or a map (`- channel: nav.done`, with `shared:`/`global:`
// flags). Purely declarative (documentation + the globals allowlist + the enforcement input); a
// duplicate name warns and is dropped. Mirrors parse_customs.
std::vector<ChannelDecl> parse_channels(Ctx &ctx, const YAML::Node &c) {
  std::vector<ChannelDecl> out;
  if (!c || !c.IsSequence())
    return out;
  for (const YAML::Node &e : c) {
    ChannelDecl d;
    if (e.IsScalar()) {
      d.name = e.Scalar(); // bare "- nav.done" shorthand
    } else if (e.IsMap()) {
      d.name = str(e, "channel");
      d.shared = flag(e, "shared", false);
      d.global = flag(e, "global", false);
    } else {
      continue;
    }
    if (d.name.empty()) {
      ctx.warn("ari: channels entry declares a channel with no name — ignored");
      continue;
    }
    if (std::any_of(out.begin(), out.end(),
                    [&](const ChannelDecl &x) { return x.name == d.name; })) {
      ctx.warn("ari: channels declares '" + d.name + "' more than once — later entry ignored");
      continue;
    }
    out.push_back(std::move(d));
  }
  return out;
}

// §12 lint policy: parse the doc-level `lint:` MAP. `channels: strict|warn|off` sets how an
// undeclared channel reference is treated (default strict); `quiet: true` suppresses the relaxed-
// enforcement notice. Unknown keys/values warn and fall back to the default (fail-safe).
LintConfig parse_lint(Ctx &ctx, const YAML::Node &l) {
  LintConfig cfg;
  if (!l || !l.IsMap())
    return cfg;
  if (const YAML::Node ch = l["channels"]; ch && ch.IsScalar()) {
    std::string m = ch.Scalar();
    std::transform(m.begin(), m.end(), m.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (m == "strict")
      cfg.channels = LintConfig::Mode::Strict;
    else if (m == "warn")
      cfg.channels = LintConfig::Mode::Warn;
    else if (m == "off" || m == "none" || m == "false")
      cfg.channels = LintConfig::Mode::Off;
    else
      ctx.warn("ari: lint.channels '" + m +
               "' unknown — using 'strict' (expected strict|warn|off)");
  }
  cfg.quiet = flag(l, "quiet", false);
  return cfg;
}

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

// §12 channel lint (A) — the fail-fast, load-time pass over a document's STATIC msg-* channel
// references, gated on the document declaring a `channels:` block. Only literal channel names are
// visible here; a dynamic `(msg-recv (expr))` name is the runtime policy's job (a follow-up).
// Walk a parsed script's value_t tree, collecting the string-literal FIRST arg of every
// (msg-send|msg-recv|msg-pending …) call — the static channel references. Depth-bounded.
void collect_channel_refs(const cvc::state_exec::value_t &v, std::vector<std::string> &out,
                          int depth) {
  using namespace cvc::state_exec;
  if (depth > 256)
    return;
  const auto *lp = std::get_if<list_ptr>(&v.v);
  if (!lp || !*lp)
    return;
  const std::vector<value_t> &lst = **lp;
  if (!lst.empty()) {
    if (const auto *head = std::get_if<symbol>(&lst[0].v)) {
      if ((head->name == "msg-send" || head->name == "msg-recv" || head->name == "msg-pending") &&
          lst.size() >= 2)
        if (const auto *s = std::get_if<std::string>(&lst[1].v))
          out.push_back(*s);
    }
  }
  for (const value_t &e : lst)
    collect_channel_refs(e, out, depth + 1);
}
// Collect static channel refs from one script string. Only an s-expr (a program, first non-space
// '(') can hold a msg-* call; a bare event name or empty script has none. A syntactically-invalid
// script is skipped here — its parse error surfaces at run time (run_init / drain), not the lint.
void refs_from_script(const std::string &script, std::vector<std::string> &out) {
  const std::size_t i = script.find_first_not_of(" \t\r\n");
  if (i == std::string::npos || script[i] != '(')
    return;
  try {
    collect_channel_refs(cvc::state_exec::parse(script), out, 0);
  } catch (...) {
  }
}
// Recurse the widget tree collecting each widget's action-program (`on:`) channel refs.
void refs_from_widget(const Widget &w, std::vector<std::string> &out) {
  refs_from_script(w.on, out);
  for (const Widget &c : w.children)
    refs_from_widget(c, out);
}

// Run the §12 channel lint against a fully-built LoadResult (root + scripts populated). Sets
// r.error on a STRICT violation (fails the load), else appends warnings. A no-op unless the doc
// declared a `channels:` block and lint.channels != Off.
void lint_channels(Ctx &ctx, LoadResult &r) {
  if (!r.has_channels_block)
    return; // no declaration -> enforcement is not active for this document
  if (r.lint.channels == LintConfig::Mode::Off) {
    if (!r.lint.quiet)
      ctx.warn("ari: channel lint is OFF (lint.channels: off) — undeclared channel use is not "
               "checked for this document");
    return;
  }
  const bool warn_only = (r.lint.channels == LintConfig::Mode::Warn);
  if (warn_only && !r.lint.quiet)
    ctx.warn("ari: channel lint is in WARN mode (lint.channels: warn) — undeclared channel use "
             "warns instead of failing the load");
  std::vector<std::string> refs;
  refs_from_script(r.init_script, refs);
  refs_from_script(r.on_tick_script, refs);
  refs_from_script(r.on_key_script, refs);
  refs_from_script(r.on_pointer_script, refs);
  refs_from_widget(r.root, refs);
  for (const std::string &ch : refs) {
    if (ch.find('#') != std::string::npos)
      continue; // a '#'-runtime channel (tick/key/pointer) is never linted
    bool declared;
    std::string what;
    if (!ch.empty() && ch.front() == '/') {
      const std::string g = ch.substr(1); // app-root-global escape: must be a DECLARED global
      declared = std::any_of(r.channels.begin(), r.channels.end(),
                             [&](const ChannelDecl &d) { return d.global && d.name == g; });
      what = "app-root-global '/" + g + "'";
    } else {
      declared = std::any_of(r.channels.begin(), r.channels.end(),
                             [&](const ChannelDecl &d) { return d.name == ch; });
      what = "channel '" + ch + "'";
    }
    if (declared)
      continue;
    const std::string msg =
        "ari: undeclared " + what + " is used but not declared in the channels: block";
    if (warn_only) {
      ctx.warn(msg);
    } else { // Strict: fail the load
      r.error = msg;
      return;
    }
  }
}

// Parse an already-loaded YAML node into a LoadResult, applying the meta gate. `base_dir` is
// the document's directory (relative `import:`s resolve against it; empty = cwd for a string);
// `self_path` is its canonical identity, seeded into the import cycle-guard so a library that
// imports back to this document is caught.
LoadResult load_node(const YAML::Node &doc, const std::string &base_dir = std::string(),
                     const std::string &self_path = std::string()) {
  LoadResult r;
  Ctx ctx;
  ctx.base_dir = base_dir;
  ctx.sources = std::make_shared<std::vector<std::string>>(); // §12.5 hot-reload watch list
  ctx.source_stamps = std::make_shared<std::map<std::string, std::int64_t>>();
  if (!self_path.empty()) {
    ctx.imported.insert(self_path);
    note_source(ctx, self_path); // the main document is itself watched
  }
  r.meta = doc.IsMap() ? parse_meta(doc["meta"]) : Meta{};

  // The min_libcvc gate fires FIRST (roadmap §3.1a): a document that needs a
  // newer libcvc fails to load rather than half-rendering against a stale API.
  if (!r.meta.min_libcvc.empty() && !version_at_least(libcvc_version(), r.meta.min_libcvc)) {
    r.error = "ari: document requires libcvc >= " + r.meta.min_libcvc + " but this build is " +
              libcvc_version();
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

  // §12 channel scoping: capture the `channels:` declaration + the doc-level `lint:` policy. The
  // loader only PARSES them (like init:/customs:); the enforcement layer (lint pass + runtime
  // policy, PR-C) consumes them. has_channels_block records PRESENCE (even if empty) — the gate
  // that enforcement is active for this document at all.
  if (doc.IsMap()) {
    if (const YAML::Node cn = doc["channels"]) {
      r.has_channels_block = true;
      if (!cn.IsSequence())
        ctx.warn("ari: channels: must be a SEQUENCE of channel names or "
                 "{channel:, shared?, global?} entries — this channels block is not a sequence and "
                 "was ignored");
      r.channels = parse_channels(ctx, cn);
    }
    r.lint = parse_lint(ctx, doc["lint"]);
  }

#ifdef CVC_ARIADNE_HAVE_JSONSCHEMA
  // Layer 2: structural validation (before the semantic Layer-3 walk below).
  schema_validate(doc, ctx);
#endif

  parse_units(ctx, doc); // §12: collect units: templates BEFORE the body (so include: resolves)
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

    // §7.1/§4.6 document-level resident handlers: scalar state_exec scripts, captured like init:.
    // The host wires each to a Runtime setter: on_tick -> set_tick_program (per frame), on_key ->
    // set_key_program (per keyboard event), on_pointer -> set_pointer_program (per mouse event);
    // events arrive via Runtime::post_input from the host's input source (cvc::gl::SdlInput).
    const YAML::Node tk = doc["on_tick"];
    if (tk && tk.IsScalar())
      r.on_tick_script = tk.Scalar();
    else if (tk && !tk.IsScalar())
      ctx.warn("ari: on_tick: must be a scalar state_exec script string — ignored");
    const YAML::Node ky = doc["on_key"];
    if (ky && ky.IsScalar())
      r.on_key_script = ky.Scalar();
    else if (ky && !ky.IsScalar())
      ctx.warn("ari: on_key: must be a scalar state_exec script string — ignored");
    const YAML::Node pt = doc["on_pointer"];
    if (pt && pt.IsScalar())
      r.on_pointer_script = pt.Scalar();
    else if (pt && !pt.IsScalar())
      ctx.warn("ari: on_pointer: must be a scalar state_exec script string — ignored");
  }
  // §12 channel lint (A): now that root + all scripts are populated, scan static msg-* refs
  // (only if the doc declared channels:). A strict violation fails the load (r.error set).
  lint_channels(ctx, r);
  if (!r.error.empty()) {
    r.warnings = std::move(ctx.warnings); // keep the warnings gathered before the violation
    return r;
  }
  if (r.meta.min_libcvc.empty())
    ctx.warn("ari: no meta.min_libcvc declared — the provenance gate is skipped "
             "(roadmap §3.1a asks every .ari to declare it)");
  r.warnings = std::move(ctx.warnings);
  if (ctx.sources)
    r.sources = std::move(*ctx.sources); // §12.5 the files a host watches for hot-reload
  if (ctx.source_stamps)
    r.source_stamps = std::move(*ctx.source_stamps); // load-time mtimes to seed the poll baseline

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
    // §12: a relative `import:` in this file resolves against the file's directory, and the
    // file's own canonical path seeds the import cycle-guard.
    const std::string self = resolve_file_path(path, std::string());
    const std::string dir = std::filesystem::path(self).parent_path().string();
    return load_node(YAML::LoadFile(path), dir, self);
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
