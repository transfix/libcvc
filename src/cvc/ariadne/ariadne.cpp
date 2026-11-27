// Ariadne core walker + runtime (cvc::ariadne). Pure libcvc: NO ImGui/VTK.
//
// The walk drives a pluggable Backend (backend.h): the core owns the retained
// tree, the reconcile boundary, and ALL cvc::state binding; the backend only
// draws value-in / edit-out. For each bound widget the core reads (and seeds) the
// state value, hands it to a backend primitive, and writes back ONLY when the
// backend reports the edit committed — so a slider drag costs one state write on
// release, not one per frame (writes fan out to observers / replicated peers).

#include "chart_emit.h"     // Kind::Chart (series / marker parse caches, the pick edge)
#include "file_picker.h"    // §10.8 the Runtime's own "Open file" window
#include "heatmap_emit.h"   // Kind::Heatmap (the subtree read on a write, the cell events)
#include "settings_emit.h"  // Kind::Settings (the key walk, its write watch, the edits)
#include "tf_editor_emit.h" // Kind::TfEditor (model / overlay / view state edges)

#include <algorithm>
#include <atomic>
#include <boost/signals2/connection.hpp> // the read-lane cache's watch on the state tree
#include <cassert>
#include <cctype>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/doc_args.h> // ARIADNE_SHELL.md 4.6 a mount's command line
#include <cvc/ariadne/files.h> // §10.8 (file-pick): the files the app's programs hold
#include <cvc/ariadne/grants.h> // ARIADNE_SHELL.md 4.5 what a mount asks for and holds
#include <cvc/ariadne/icons.h> // §5.22 a tray icon's named glyphs
#include <cvc/ariadne/input.h>  // §4.6 InputEvent — Runtime::post_input feeds on_key/on_pointer
#include <cvc/ariadne/loader.h> // is_builtin_widget_type (has?)
#include <cvc/ariadne/media.h>  // §5.11 pictures by URI
#include <cvc/ariadne/media_player.h> // §5.13 video: / audio: players
#include <cvc/ariadne/media_save.h>   // §5.29 saving a picture or a video
#include <cvc/ariadne/net_host.h>     // net_host_status (the net capability tokens)
#include <cvc/ariadne/oidc_host.h>    // pump_oidc
#include <cvc/ariadne/requires.h>     // free_heads: the isolation rules' oidc check
#include <cvc/ariadne/rich_text.h>    // §5.20 parse_text_style
#include <cvc/ariadne/stream_intrinsics.h> // retarget_document_streams: a move takes its streams
#include <cvc/ariadne/uri.h>               // parse_uri: a mount's principal (ARIADNE_SHELL.md 4.12)
#include <cvc/core/app.h>
#include <cvc/core/exec_pool.h>    // an isolated document's pump; in_pump_all (host-thread calls)
#include <cvc/core/notify.h>       // §5.24 the feed the tray's bell shows
#include <cvc/core/server_clock.h> // §5.22 the relay's time, in the clock's tooltip
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_host.h>      // ictx.host: each lane's async host (S1.9)
#include <cvc/state/state_exec/async_scheduler.h> // §4.7 app-wide action scheduler (exec_scheduler)
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/evaluator.h> // evaluation_timeout / evaluation_interrupted
#include <cvc/state/state_exec/exceptions.h> // dsl_error (has?)
#include <cvc/state/state_exec/exec_group.h> // install_providers: a group installs by kind
#include <cvc/state/state_exec/heap.h>       // the init lane's own cycle registry
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/parser.h> // parse, parse_error
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/scheduler.h>
#include <cvc/state/state_exec/stackless_evaluator.h> // §4 read-lane predicate evaluator
#include <deque>
#include <exception>
#include <functional>
#include <locale> // §11.4 geometry (de)serialize: pin std::locale::classic() so digit grouping
                  // in a global locale can't corrupt the comma-joined "x,y,w,h" round-trip
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility> // exchange
#include <vector>

namespace cvc {
namespace ariadne {

namespace {

// read_or_seed<T> / write<T> / resolve_bind now live in cvc/ariadne/bind.h so the
// widget walk here and the scene binder (§9) share ONE definition and can't drift.
// They are used unqualified below (same cvc::ariadne namespace).

// Read a state value as a plain string with no seeding (read-only Text view). Follows a
// transparent-link hole to its target (§12), on every hop, so a bound Text in a mounted module
// shows the granted parent value (or a value under it), matching every other read path.
std::string read_string(cvc::app &ctx, const std::string &path) {
  try {
    cvc::state *n = read_node(ctx, path);
    return n ? n->value() : std::string();
  } catch (const std::exception &) {
    return "<unset>";
  }
}

// A slider's `range: key`: the bound key's "min" and "max" properties over `lo` / `hi`, each kept
// where the key has none (or one that is not a number). Follows a transparent link as reads do.
void key_range(cvc::app &ctx, const std::string &path, double &lo, double &hi) {
  try {
    cvc::state *n = read_node(ctx, path);
    if (!n)
      return;
    for (auto [prop, out] : {std::pair<const char *, double *>{"min", &lo}, {"max", &hi}}) {
      const std::string t = n->property(prop);
      char *end = nullptr;
      const double d = std::strtod(t.c_str(), &end);
      if (!t.empty() && end && *end == '\0' && std::isfinite(d))
        *out = d;
    }
  } catch (const std::exception &) {
  }
}

// Not inlined into its caller: emit_node recurses with the widget tree, and MSVC gives it one
// frame for every case's locals, so a case's work that has its own goes in a function of its own
// (a 40-deep tree's walk stays well inside a thread's stack).
#if defined(_MSC_VER)
#define CVC_ARI_NOINLINE __declspec(noinline)
#else
#define CVC_ARI_NOINLINE __attribute__((noinline))
#endif

// §4 read-lane: is this bind an s-EXPRESSION (a computed value) rather than a state path?
// State paths are dotted identifiers and never begin with '('; an s-expr always does. So the
// first non-space character decides — no ambiguity, and no new field/key needed.
bool is_expr(const std::string &bind) {
  for (char c : bind) {
    if (c == ' ' || c == '\t' || c == '\n' || c == '\r')
      continue;
    return c == '(';
  }
  return false;
}

// §G7 colour: parse an "r,g,b" state string into rgb[3] (comma- or space-separated). Returns false
// (leaving `out` at the caller's default) unless exactly three numbers parse.
bool parse_rgb3(const std::string &s, float out[3]) {
  std::string t = s;
  for (char &c : t)
    if (c == ',')
      c = ' ';
  std::istringstream is(t);
  float a = 0, b = 0, c = 0;
  if (is >> a >> b >> c) {
    out[0] = a;
    out[1] = b;
    out[2] = c;
    return true;
  }
  return false;
}
std::string format_rgb3(const float rgb[3]) {
  std::ostringstream os;
  os << rgb[0] << "," << rgb[1] << "," << rgb[2];
  return os.str();
}

// §11.4 runtime geometry persistence: (de)serialize a WindowGeom / track list to the comma-joined
// string the tree.<id>.geometry / tree.<id>.tracks state keys use (§11.2 spells geometry
// "x,y,w,h"). Rounded to whole pixels so a static window's write is byte-stable and cvc::state's
// equality guard suppresses per-frame observer churn (write<> no-ops when the string is unchanged).
std::string format_geom(const WindowGeom &g) {
  std::ostringstream os;
  os.imbue(std::locale::classic()); // no digit grouping: a "C"-locale-independent "x,y,w,h" (a
                                    // grouping global locale would emit "1,000" and break the join)
  os << std::lround(g.x) << "," << std::lround(g.y) << "," << std::lround(g.w) << ","
     << std::lround(g.h);
  return os.str();
}
// Parse "x,y,w,h" (comma- or space-separated). Returns false (leaving `out` untouched) unless four
// numbers parse; has_size stays false when w/h are non-positive (a collapsed/unsized sentinel), so
// a window with position-only persisted geometry seeds its position but not a zero size.
bool parse_geom(const std::string &s, WindowGeom &out) {
  std::string t = s;
  for (char &c : t)
    if (c == ',')
      c = ' ';
  std::istringstream is(t);
  is.imbue(std::locale::classic()); // match format_geom's locale-independent encoding
  float x = 0, y = 0, w = 0, h = 0;
  if (!(is >> x >> y >> w >> h))
    return false;
  out.x = x;
  out.y = y;
  out.w = w;
  out.h = h;
  out.has_pos = true;
  out.has_size = (w > 0.0f && h > 0.0f);
  return true;
}
std::string format_tracks(const std::vector<float> &tr) {
  std::ostringstream os;
  os.imbue(std::locale::classic()); // no digit grouping (see format_geom)
  for (std::size_t i = 0; i < tr.size(); ++i) {
    if (i)
      os << ",";
    os << std::lround(tr[i]);
  }
  return os.str();
}
std::vector<float> parse_tracks(const std::string &s) {
  std::vector<float> out;
  std::string t = s;
  for (char &c : t)
    if (c == ',')
      c = ' ';
  std::istringstream is(t);
  is.imbue(std::locale::classic()); // match format_tracks (see format_geom)
  float v = 0.0f;
  while (is >> v)
    out.push_back(v);
  return out;
}

// Replace every occurrence of `token` in `s` with `rep`, in place: a string without it is left
// as it is, not copied.
void replace_in(std::string &s, std::string_view token, const std::string &rep) {
  if (token.empty())
    return;
  for (std::size_t pos = s.find(token); pos != std::string::npos;
       pos = s.find(token, pos + rep.size()))
    s.replace(pos, token.size(), rep);
}

// The same, on a copy.
std::string replace_all(std::string s, std::string_view token, const std::string &rep) {
  replace_in(s, token, rep);
  return s;
}

// A repeat's loop-index token.
constexpr std::string_view kIndexToken = "{i}";

// §3 repeat: substitute the loop-index token `{i}` with the 0-based index `idx`, in place on an
// instance's own copy of its template, in every field an instance uses to address distinct state
// or to display (id/label/bind/on and the pointer handlers/predicates/tooltip/options). Goes
// through its children (substitute_index_in), without recursing. (Single-level: a nested `repeat`
// shares the same `{i}` token and is not independently indexed — a follow-up.)
void substitute_index_one(Widget &out, const std::string &idx) {
  constexpr std::string_view kTok = kIndexToken;
  replace_in(out.id, kTok, idx);
  replace_in(out.label, kTok, idx);
  replace_in(out.bind, kTok, idx);
  replace_in(out.on, kTok, idx);
  replace_in(out.pick, kTok, idx); // a chart's or a heatmap's pick: key
  // A chart's series, cursor and log axis: a row's own curves (series: [{bind: runs.{i}.y}]).
  for (ChartSeriesBind &s : out.chart_series) {
    replace_in(s.bind, kTok, idx);
    replace_in(s.label, kTok, idx);
  }
  replace_in(out.cursor, kTok, idx);
  replace_in(out.log_y, kTok, idx);
  // §4.6 the pointer handlers, a text_view's on_top, and a message_input's hint and send label: a
  // row's handler acts on that row (on_click: (state-set "tree.cam_{i}.open" "1")).
  replace_in(out.on_click, kTok, idx);
  replace_in(out.on_hover, kTok, idx);
  replace_in(out.on_drag, kTok, idx);
  replace_in(out.on_drag_start, kTok, idx);
  replace_in(out.on_drag_end, kTok, idx);
  replace_in(out.on_hover_end, kTok, idx);
  replace_in(out.on_top, kTok, idx);
  replace_in(out.hint, kTok, idx);
  replace_in(out.send, kTok, idx);
  replace_in(out.on_change, kTok, idx);
  // A list's selection key and edit handlers, and a tab bar's titles.
  replace_in(out.list_current, kTok, idx);
  replace_in(out.on_add, kTok, idx);
  replace_in(out.on_remove, kTok, idx);
  for (std::string &t : out.tab_titles)
    replace_in(t, kTok, idx);
  // A settings editor's roots and filter (root: vehicles.{i}).
  for (std::string &r : out.settings_roots)
    replace_in(r, kTok, idx);
  for (std::string &f : out.settings_filter)
    replace_in(f, kTok, idx);
  replace_in(out.visible_when, kTok, idx);
  replace_in(out.enabled_when, kTok, idx);
  replace_in(out.disabled_when, kTok, idx);
  replace_in(out.tooltip, kTok, idx);
  replace_in(out.badge, kTok, idx); // §5.11 a picture's "+3"
  replace_in(out.options_expr, kTok, idx);
  replace_in(out.src, kTok, idx); // §5.11: a picture per item (src: feed.{i}.png)
  replace_in(out.media_status, kTok, idx);
  replace_in(out.background.src, kTok, idx); // §5.12
  replace_in(out.background.bind, kTok, idx);
  replace_in(out.background.status, kTok, idx);
  replace_in(out.background.color_src, kTok, idx); // §5.26
  replace_in(out.color_src, kTok, idx);            // §5.26 a row's colour (palette.{i})
  replace_in(out.poster, kTok, idx); // §5.13
  replace_in(out.emoji, kTok, idx);  // §5.11 a post's own shortcodes
  replace_in(out.button_icon, kTok, idx); // §5.27 a row's button: its picture, whether it's chosen
  replace_in(out.selected, kTok, idx);
  // §5.20 rich text: a row's runs (runs: feed.items.{i}.runs) and its handlers
  replace_in(out.runs_src, kTok, idx);
  replace_in(out.on_link, kTok, idx);
  for (TextRunSpec &r : out.runs) {
    replace_in(r.text, kTok, idx);
    replace_in(r.href, kTok, idx);
    replace_in(r.on_click, kTok, idx);
    replace_in(r.tooltip, kTok, idx);
  }
  // §12: a repeated MOUNT (Widget::scope set) gets a per-instance sub-prefix — append the loop
  // index so each instance's fragment binds to its OWN state subtree (includes.<as>.<i>), rather
  // than all N instances sharing one, which would violate repeat's per-instance-state contract.
  if (!out.scope.empty())
    out.scope += "." + idx;
  replace_in(out.app_name_bind, kTok, idx); // §5.25 and its name: (slots.{i}.name)
  // §12: and its parent-scope holes are {i}-substituted too, so an instance can grant a
  // per-instance target (link: { slot: fleet.{i} }) — mirroring the bind/on substitution above.
  for (LinkHole &h : out.links) {
    replace_in(h.name, kTok, idx);
    replace_in(h.target, kTok, idx);
  }
}

// `out` and its subtree (its children and its context menu's items), each node on its own: no
// call per level, and only a node with a subtree of its own waits in `todo`.
void substitute_index_in(Widget &out, const std::string &idx) {
  std::vector<Widget *> todo;
  for (Widget *w = &out;;) {
    substitute_index_one(*w, idx);
    // A nested `repeat` template is left UNtouched so its own expansion owns its `{i}` (rather
    // than the outer index clobbering the inner template's bindings). Single-level still: the
    // inner cannot also reference the outer index — the loader warns on a nested repeat.
    // §5.30 a row's right-click menu acts on that row (on: (delete-row {i})).
    for (std::vector<Widget> *list : {&w->children, &w->context_items})
      for (Widget &c : *list) {
        if (!c.repeat.empty())
          continue;
        if (c.children.empty() && c.context_items.empty())
          substitute_index_one(c, idx);
        else
          todo.push_back(&c);
      }
    if (todo.empty())
      break;
    w = todo.back();
    todo.pop_back();
  }
}

// One deep copy of the template, then the index substituted through it in place. (Copying each
// level and then its children again copied a subtree once per level above it, each frame, for
// every instance of a feed's repeat.)
Widget substitute_index(const Widget &w, int index) {
  Widget out = w;
  substitute_index_in(out, std::to_string(index));
  return out;
}

} // namespace

// --- custom widget type registry (§ extensibility) --------------------------
namespace {
// An app's custom widget types.
struct widget_table {
  static constexpr const char *app_slot = "cvc.ariadne.widget-types";
  std::mutex m;
  std::unordered_map<std::string, WidgetEmitFn> emitters;
  // Widget types a BACKEND renders natively (no compositional emit fn) — so the loader does not
  // warn they are unregistered.
  std::set<std::string> known;
};
WidgetEmitFn lookup_widget(cvc::app &app, const std::string &type) {
  widget_table &t = app.slot<widget_table>();
  std::lock_guard<std::mutex> lock(t.m);
  auto it = t.emitters.find(type);
  return it != t.emitters.end() ? it->second : WidgetEmitFn{};
}
} // namespace

void register_widget_type(cvc::app &app, const std::string &type, WidgetEmitFn emit) {
  if (!emit || type.empty())
    return;
  widget_table &t = app.slot<widget_table>();
  std::lock_guard<std::mutex> lock(t.m);
  t.emitters[type] = std::move(emit);
}

void register_known_widget_type(cvc::app &app, const std::string &type) {
  if (type.empty())
    return;
  widget_table &t = app.slot<widget_table>();
  std::lock_guard<std::mutex> lock(t.m);
  t.known.insert(type);
}

bool has_widget_type(cvc::app &app, const std::string &type) {
  widget_table &t = app.slot<widget_table>();
  std::lock_guard<std::mutex> lock(t.m);
  return t.emitters.find(type) != t.emitters.end() || t.known.find(type) != t.known.end();
}

// --- the init: block runner (state_exec) --------------------------------------

// Host-contributed program-lane intrinsics (nav verbs et al.), each app's own list (app.slot).
// ActionIntrinsicProvider is state_exec-typed via forward-decls. Each provider keeps its kind
// (STATE_EXEC_THREADED.md 4.9, S1-Q7), and install_providers puts it into a program's env by it:
// - In group 0 (the app scheduler), in a host-affinity group, in the init lane and in the name
//   probes, every provider runs as it is, into the env itself.
// - In an `any` group (an isolated document the pool's workers may pump), an `any` provider runs
//   as it is, and a `host_only` one does not run, so its names are NameErrors. A `host_proxied`
//   one runs into a scratch scope over the env (environment::extend), so it reads the env's
//   bindings as it would, and what it bound there is then moved into the env: each native through
//   exec_group::install(..., host_proxied), which wraps it to run on the pump_all caller's thread
//   (S1.8), and anything else (a constant) as it is.
// A provider's body runs where the env is built: on the host thread for an action, a resident and
// the init lane, but on the pumping thread for a spawned child, which is a worker in an `any`
// group. So a provider an `any` group runs (`any` or `host_proxied`) has a thread-safe body.
// A `host_proxied` provider binds synchronous natives: an async one, so wrapped, would run its
// kernel inline on the caller (its host has no running process there), uncharged. What a provider
// binds that is not a native goes into the env as it is, so it is made fresh for each env or is
// immutable (c-m7): two groups' programs must never share a mutable value.
namespace {
namespace se_reg = cvc::state_exec;
struct registered_provider {
  std::string key; // empty: appended, never replaced
  ActionIntrinsicProvider provider;
  se_reg::provider_affinity affinity = se_reg::provider_affinity::host_only;
};
struct action_intrinsics_table {
  static constexpr const char *app_slot = "cvc.ariadne.action_intrinsics";
  std::mutex m;
  std::vector<registered_provider> providers;
  // Bumped by every register / clear, so a Runtime's cached program names know when to rebuild.
  std::atomic<std::uint64_t> generation{0};
};
std::uint64_t action_intrinsics_generation(cvc::app &app) {
  return app.slot<action_intrinsics_table>().generation.load();
}
std::vector<registered_provider> action_intrinsics_snapshot(cvc::app &app) {
  action_intrinsics_table &t = app.slot<action_intrinsics_table>();
  std::lock_guard<std::mutex> lock(t.m);
  return t.providers;
}

// Run each of `providers` into `env` for a program whose context is `ictx`, by kind when `group`
// is an `any` group (above); null `group`: group 0, the init lane or a probe. A Runtime's own
// (Runtime::add_program_intrinsics) go the same way, after the app's.
void install_providers(const std::vector<registered_provider> &providers,
                       const se_reg::environment_ptr &env, se_reg::intrinsics_context &ictx,
                       se_reg::exec_group *group) {
  const bool by_kind = group && group->opts().affinity == se_reg::group_affinity::any;
  for (const registered_provider &p : providers) {
    if (!p.provider)
      continue;
    if (!by_kind || p.affinity == se_reg::provider_affinity::any) {
      p.provider(env, ictx);
      continue;
    }
    if (p.affinity == se_reg::provider_affinity::host_only)
      continue;
    const se_reg::environment_ptr scratch = se_reg::environment::extend(env);
    p.provider(scratch, ictx);
    for (auto &[name, value] : scratch->bindings) {
      if (auto *fn = std::get_if<se_reg::native_fn>(&value.v))
        group->install(env, name, std::move(*fn), se_reg::provider_affinity::host_proxied);
      else
        env->set(name, std::move(value));
    }
  }
}

// `app`'s providers (register_action_intrinsics), as above.
void install_providers(cvc::app &app, const se_reg::environment_ptr &env,
                       se_reg::intrinsics_context &ictx, se_reg::exec_group *group) {
  install_providers(action_intrinsics_snapshot(app), env, ictx, group);
}

// Bind (has? "name") into a program or predicate env. It answers from `has` (Runtime::has), so a
// document can ask what its host provides, e.g. hide a GL panel in a terminal. A string or a
// quoted symbol names the thing.
void bind_has(const std::shared_ptr<cvc::state_exec::environment> &env,
              const std::function<bool(const std::string &)> &has) {
  namespace se = cvc::state_exec;
  if (!has)
    return;
  se::builtins::register_fn(env, "has?", [has](std::span<const se::value_t> args) -> se::value_t {
    if (args.size() != 1)
      throw se::dsl_error("TypeError", "has?: expected one argument, a name");
    if (const auto *s = std::get_if<std::string>(&args[0].v))
      return se::value_t(has(*s));
    if (const auto *sym = std::get_if<se::symbol>(&args[0].v))
      return se::value_t(has(sym->name));
    throw se::dsl_error("TypeError", "has?: the name must be a string or a symbol");
  });
}
} // namespace

void register_action_intrinsics(cvc::app &app, ActionIntrinsicProvider provider) {
  register_action_intrinsics(app, std::string(), std::move(provider),
                             cvc::state_exec::provider_affinity::host_only);
}
void register_action_intrinsics(cvc::app &app, const std::string &key,
                                ActionIntrinsicProvider provider) {
  register_action_intrinsics(app, key, std::move(provider),
                             cvc::state_exec::provider_affinity::host_only);
}
void register_action_intrinsics(cvc::app &app, ActionIntrinsicProvider provider,
                                cvc::state_exec::provider_affinity affinity) {
  register_action_intrinsics(app, std::string(), std::move(provider), affinity);
}
void register_action_intrinsics(cvc::app &app, const std::string &key,
                                ActionIntrinsicProvider provider,
                                cvc::state_exec::provider_affinity affinity) {
  if (!provider)
    return;
  action_intrinsics_table &t = app.slot<action_intrinsics_table>();
  std::lock_guard<std::mutex> lock(t.m);
  auto same = key.empty()
                  ? t.providers.end()
                  : std::find_if(t.providers.begin(), t.providers.end(),
                                 [&key](const registered_provider &p) { return p.key == key; });
  if (same != t.providers.end()) {
    same->provider = std::move(provider);
    same->affinity = affinity;
  } else {
    t.providers.push_back({key, std::move(provider), affinity});
  }
  ++t.generation;
}
void clear_action_intrinsics(cvc::app &app) {
  action_intrinsics_table &t = app.slot<action_intrinsics_table>();
  std::lock_guard<std::mutex> lock(t.m);
  t.providers.clear();
  ++t.generation;
}

namespace {
// The unsigned integer the tree setting `key` holds; `fallback` when it's missing or unparsable.
uint64_t limit_setting(cvc::state &root, const char *key, uint64_t fallback) {
  cvc::state *node = root.findDescendant(key);
  if (!node)
    return fallback;
  const std::string v = node->value();
  uint64_t n = 0;
  const auto [end, ec] = std::from_chars(v.data(), v.data() + v.size(), n);
  return !v.empty() && ec == std::errc() && end == v.data() + v.size() ? n : fallback;
}

// §7.4 an action's memory budget: the tree setting ariadne.limits.action_max_memory (bytes;
// 0 = no limit), read on every submission so a host can tune it live. It also bounds each
// native call's allocations in the action (state_exec's per-step allocation limit takes what
// is left of max_memory). A missing or unparsable value keeps the 4 MiB default.
uint64_t action_max_memory(cvc::state &root) {
  return limit_setting(root, "ariadne.limits.action_max_memory", uint64_t{4} << 20);
}

// §7.4 an action's step and time caps: the tree settings ariadne.limits.action_max_steps (default
// 5000) and ariadne.limits.action_max_time (seconds, default 0.1), read on every submission like
// the memory budget; 0 = no limit. The action's async calls are charged to both
// (docs/STATE_EXEC_EXCEPTIONS.md 3.8), so a host whose documents call long-running async verbs
// raises them. They are app-wide, outside any document's prefix, so a document can't raise its
// own. A missing or unparsable value keeps the default.
uint64_t action_max_steps(cvc::state &root) {
  return limit_setting(root, "ariadne.limits.action_max_steps", 5000);
}

// An action's with-lock default timeout (docs/STATE_EXEC_LOCKS.md 5.5, 14.2 Q1): the tree setting
// ariadne.exec.lock_timeout (seconds, default 1.0), read on every submission like the limits. An
// action and the children it spawns that name no :timeout give up after it (TimeoutError), so a
// document never hangs on another's lock. 0 never waits. A missing or unparsable value keeps
// the default.
double action_lock_timeout(cvc::state &root) {
  constexpr double kDefault = 1.0;
  cvc::state *node = root.findDescendant("ariadne.exec.lock_timeout");
  if (!node)
    return kDefault;
  const std::string v = node->value();
  char *end = nullptr;
  const double seconds = v.empty() ? -1.0 : std::strtod(v.c_str(), &end);
  return end == v.c_str() + v.size() && std::isfinite(seconds) && seconds >= 0.0 ? seconds
                                                                                 : kDefault;
}

double action_max_time(cvc::state &root) {
  constexpr double kDefault = 0.1;
  cvc::state *node = root.findDescendant("ariadne.limits.action_max_time");
  if (!node)
    return kDefault;
  const std::string v = node->value();
  char *end = nullptr;
  const double seconds = v.empty() ? -1.0 : std::strtod(v.c_str(), &end);
  return end == v.c_str() + v.size() && std::isfinite(seconds) && seconds >= 0.0 ? seconds
                                                                                 : kDefault;
}

// §13.8 a resident's per-step time cap: the tree setting ariadne.limits.resident_max_step_time
// (seconds; 0 = none), read as each resident starts. The cap keeps one runaway step from hanging
// the host's drain. It is a se::time_budget, so time the OS deschedules the thread for counts
// only for its first half; but single-threaded wasm has no thread CPU clock (thread_cpu_seconds
// is the steady clock there), so on a loaded machine a step that checks often is charged for its
// waits too, up to a sixteenth of the half per check, and the default there is higher (cypkg's
// browser test, single-threaded, load 10-33: 1 run of 9 lost a module tick at 0.05 s, none at 2).
double resident_max_step_time(cvc::state &root) {
#ifdef __EMSCRIPTEN__
  constexpr double kDefault = 1.0;
#else
  constexpr double kDefault = 0.05;
#endif
  cvc::state *node = root.findDescendant("ariadne.limits.resident_max_step_time");
  if (!node)
    return kDefault;
  const std::string v = node->value();
  char *end = nullptr;
  const double seconds = v.empty() ? -1.0 : std::strtod(v.c_str(), &end);
  return end == v.c_str() + v.size() && std::isfinite(seconds) && seconds >= 0.0 ? seconds
                                                                                 : kDefault;
}
} // namespace

namespace {
// Run `script` as a FULL-ENV state_exec program chrooted to `prefix`, bounded by (max_steps,
// max_seconds, max_bytes). Returns "" on a clean, normal finish, else a human diagnostic (no
// leading "ari:" — the caller frames it). Used by the load-time `init:` lane (run_init) and a
// mounted fragment's init: — a synchronous run-to-completion in ONE local scope (scheduler, proc,
// and intrinsics_context all live only for this call). The per-interaction ACTION lane does NOT
// come through here: a program `on:` is submitted as a persistent process via
// Runtime::Impl::submit_action, whose ActionContext keeps its intrinsics_context alive across
// frames (and installs ictx.document for the per-document resource scope — which this local,
// no-owner init: context deliberately leaves null). Same trust model as the action lane (see the
// ReactiveEngine note on the full-env lanes): the full intrinsics env with writes confined to the
// document prefix, resource-capped, and it never throws.
// A failed program's exception for a report (docs/STATE_EXEC_EXCEPTIONS.md 2.6): "Class: message",
// and where it was raised, the innermost line of its traceback.
std::string failure_report(const std::optional<cvc::state_exec::value_t> &exc,
                           const std::optional<std::string> &exit_error) {
  namespace se = cvc::state_exec;
  if (!exc)
    return exit_error.value_or("failed");
  std::string report = se::exception_summary(*exc);
  if (const se::value_t *tb = se::field(*exc, "traceback"))
    if (const auto *lines = std::get_if<se::list_ptr>(&tb->v);
        lines && *lines && !(*lines)->empty())
      if (const auto *where = std::get_if<std::string>(&(*lines)->back().v))
        report += " (at " + *where + ")";
  return report;
}

// " [program]": which program a warning is about, on one line and at most 80 characters.
std::string program_tag(const std::string &script) {
  std::string one;
  bool space = false;
  for (char c : script) {
    const bool ws = c == ' ' || c == '\n' || c == '\t' || c == '\r';
    if (ws && (space || one.empty()))
      continue;
    space = ws;
    one += ws ? ' ' : c;
  }
  while (!one.empty() && one.back() == ' ')
    one.pop_back();
  if (one.size() > 80)
    one = one.substr(0, 77) + "...";
  return one.empty() ? std::string() : " [" + one + "]";
}

std::string run_scoped_program(cvc::app &app, const std::string &prefix, const std::string &script,
                               uint64_t max_steps, double max_seconds, uint64_t max_bytes,
                               const std::function<bool(const std::string &)> &has = {},
                               const ActionIntrinsicProvider &own = {}) {
  namespace se = cvc::state_exec;
  try {
    // All of these must outlive execute()+run(): the intrinsics capture &ictx by pointer. A
    // synchronous run-to-completion in this one scope satisfies that.
    // The run's own cycle registry (S1.1, S1-Q3), which the scheduler binds: the script's cycles
    // are collected and freed when the run ends, after the scheduler and the env. Declared first,
    // so it goes last.
    se::cycle_registry_ptr registry = se::make_cycle_registry();
    se::scheduler sched;
    sched.set_cycle_registry(registry.get());
    // Its with-lock table is the app's (docs/STATE_EXEC_LOCKS.md 8.4), so a request meets the
    // residents' and actions' holds. This thread pumps it: not cross_thread. The home attaches
    // here and detaches with the scheduler, at the end of this scope.
    sched.set_lock_manager(app.lock_manager());
    // Init's own inline host (S1-Q2): a provider registers its async verbs on *ictx.host, so here
    // they run inline, charged to init's process. Declared after sched, so it goes first.
    se::async_host init_host(&sched);
    se::memory_tracker tracker;
    auto proc = se::make_process();
    proc->pid = 1;
    proc->status = se::process_status::ready;
    se::intrinsics_context ictx;
    cvc::state &root = cvc::state::instance(app);
    ictx.sched = &sched;
    ictx.host = &init_host;
    ictx.tracker = &tracker;
    ictx.proc = proc;
    ictx.pid = 1;
    // The init lane never waits for a lock (docs/STATE_EXEC_LOCKS.md 5.5): this thread steps the
    // residents too, so a holder could never release meanwhile. A busy request is as :timeout 0
    // (TimeoutError, or :else with :if-available).
    ictx.lock_wait_allowed = false;
    ictx.lock_default_timeout = 0.0;
    // A child the script spawns copies both (so it never waits either), the chroot and this
    // scheduler, and gets the intrinsics, the host's providers, the document's own and has?
    // (docs/STATE_EXEC_LOCKS.md 5.5). This scope's sched.run steps it, and its context goes with
    // the scheduler. No Impl here, so no document scope: the library's callback, not
    // Runtime::Impl::make_action_context.
    ictx.spawn_context = se::make_spawn_context(
        nullptr, [has, own, &app](const se::environment_ptr &e, se::intrinsics_context &c) {
          install_providers(app, e, c, nullptr);
          if (own)
            own(e, c);
          bind_has(e, has);
        });
    se::apply_chroot(ictx, root, prefix); // scope writes under the document/mount prefix
    auto env = se::builtins::make_default_environment();
    se::register_intrinsics(env, &ictx);
    // Let a host bind extra program-lane intrinsics (nav verbs, …) into this env — AFTER the
    // standard ones, so it can add or override. Snapshot the registry so a provider is free to
    // register/throw without holding the lock. Every kind as it is: init runs on this thread.
    install_providers(app, env, ictx, nullptr);
    if (own) // the document's own, after the app's (Runtime::add_program_intrinsics)
      own(env, ictx);
    bind_has(env, has);
    // Bound the run so a looping or blocking script (an accidental infinite loop, or a
    // (msg-recv ...) with no sender) can never hang: cap the process (check_limits kills a runner
    // at the cap) AND the run loop (breaks out even when the sole process is blocked, which
    // check_limits can't catch).
    se::execute_options opts;
    opts.env = env;
    opts.max_steps = max_steps;
    opts.max_time = max_seconds;
    opts.max_memory = max_bytes;
    const int pid = sched.execute(script, opts); // throws se::parse_error on a syntax error
    sched.run(max_steps, max_seconds);           // bounded run to completion
    if (auto info = sched.get_process_info(pid)) {
      // Success is ONLY a normal finish. Any other terminal/non-terminal state — killed
      // (runtime error / resource limit) or still ready/running/waiting after the cap
      // (it blocked or looped past the budget) — is a reported failure, not silent.
      if (info->status == se::process_status::failed)
        return "failed: " +
               failure_report(sched.get_exit_exception(pid), sched.get_exit_error(pid));
      if (info->status != se::process_status::terminated)
        return "did not complete normally (a resource limit, or it blocked/looped past the "
               "budget)";
    }
    return std::string();
  } catch (const std::exception &e) {
    return std::string("failed: ") + e.what();
  }
}
} // namespace

bool run_init(cvc::app &app, const std::string &prefix, const std::string &script,
              std::vector<std::string> *errors, const std::function<bool(const std::string &)> &has,
              const ActionIntrinsicProvider &own) {
  const auto err = [&](const std::string &m) {
    if (errors)
      errors->push_back(m);
  };
  if (script.empty())
    return true; // nothing to run
  // A load-time seed/compute is generous — well above the per-tick action budget (§7.4).
  static constexpr uint64_t kInitMaxSteps = 10'000'000;
  static constexpr double kInitMaxSeconds = 5.0;
  const std::string e =
      run_scoped_program(app, prefix, script, kInitMaxSteps, kInitMaxSeconds, 0, has, own);
  if (!e.empty()) {
    err("ari: init: script " + e);
    return false;
  }
  return true;
}

void seed_state(cvc::app &app, const std::string &prefix, const StateSeeds &seeds) {
  for (const auto &[path, value] : seeds) // a write through a link hole reaches its target
    write<std::string>(app, resolve_bind(prefix, path), value);
}

// The read lane's builtins (read_lane_builtins, ariadne.h): what a reactive slot's predicate may
// call, and where a LaneProcess's `allow` starts.
namespace {
constexpr const char *kReadLaneBuiltins[] = {
    // arithmetic / comparison / coercion / type predicates / logic
    "+", "-", "*", "/", "%", "<", ">", "<=", ">=", "=", "!=", "int", "float", "str", "is-int",
    "is-float", "is-string", "is-null", "is-list", "type-of", "not", "and", "or",
    // bounded compound construction (by-reference; not the doubling materializers) + access
    "list", "cons", "car", "cdr", "nth", "length", "dict", "get-attr",
    // side-effect-free state readers (state-data-get deep-copies; no aliasing, no callables;
    // state-json writes its document as sliced data steps, under the allocation limit)
    "state-get", "state-exists", "state-children", "state-data-get", "state-json",
    "state-root-path", "state-has-expiry", "state-is-expired",
    // JSON text to values and back: pure, sliced data steps under the allocation limit
    "json-parse", "json-encode",
    // the wall clock (a post's "2h ago"): one read, nothing written
    "time-now",
    // the stdlib: the pure, bounded functions (a scalar, or a result no bigger than its
    // argument), and the materializers, which do their work as sliced data steps (each
    // eval slices, see data_step_nodes_) under the per-call allocation limit. A function
    // collections.map/filter/reduce call runs as this evaluator's own frames, so the
    // special-form gate and the caps cover it too. math.random stays out because a
    // predicate must be deterministic.
    "string.length", "string.byte-length", "string.char-at", "string.substring", "string.upper",
    "string.lower", "string.trim", "string.starts-with", "string.ends-with", "string.contains",
    "string.split", "string.join", "string.replace", "string.encode", "bytes.length",
    "bytes.byte-at", "bytes.slice", "bytes.concat", "bytes.decode", "math.sqrt", "math.abs",
    "math.floor", "math.ceil", "math.round", "math.pow", "math.log", "math.sin", "math.cos",
    "math.tan", "math.min", "math.max", "math.clamp", "math.pi", "math.e", "collections.map",
    "collections.filter", "collections.reduce", "collections.zip", "collections.flatten",
    "collections.sort", "collections.reverse", "collections.range", "collections.unique",
    "collections.dict-keys", "collections.dict-values", "collections.dict-merge"};
} // namespace

std::span<const char *const> read_lane_builtins() { return kReadLaneBuiltins; }

// --- §4 read-lane: the per-frame reactive predicate evaluator ----------------
namespace {
namespace se = cvc::state_exec;

// Evaluates a widget's read-lane expressions (visible_when, …) each frame on ONE
// long-lived stackless evaluator over a DEFAULT-DENY, SCALAR-ONLY environment. Only scalar
// arithmetic/comparison/coercion/logic and the side-effect-free scalar/bool state readers
// are bound; every writer, scheduler op, watch, and I/O is excluded — as is anything that
// could do unbounded work in one native step (see the ctor for why: the caps are checked
// only BETWEEN steps, so compound builders/walkers and `apply` are barred, not just the
// obvious looping builtins). A predicate that reaches off the allowlist hits an unbound
// symbol and fails fail-safe. Special forms (if/begin/while/for/let/lambda/…) and the
// true/false/nil literals are parser/evaluator-intrinsic, so an empty base still runs them.
//
// Each distinct expression is parsed ONCE and cached. Every eval is bounded three ways:
// a per-eval step cap, a per-eval time cap, and a per-FRAME aggregate time budget (so many
// reactive widgets cannot together stall a frame). The time caps are se::time_budgets, which
// time the thread spends not running cannot use up: a predicate that does next to no work
// never fails, however loaded the machine. It never throws out.
//
// state-get keys are prefix-relative (the context is chroot'd to the document prefix),
// so `(state-get "demo.n")` reads the SAME key a widget `bind: demo.n` resolves to.
//
// §2.1 results are cached: an expression that ran is kept with the state paths it read and the
// (has? …) answers it got, and it is reused, not run, until a write lands on one of those paths (or
// on an ancestor or a descendant of one), or one of those names answers otherwise. An expression
// that read the clock, an expiry, or a node through a link is never kept. The writes are seen
// through the tree root's childChanged, on whatever thread made them.
// §5.20 the most runs a rich text takes (from an expression or a state path); extras are dropped.
constexpr std::size_t kMaxRuns = 4096;

class ReactiveEngine {
public:
  // `has` answers (has? "name") in predicates (Runtime::has); empty leaves it unbound.
  ReactiveEngine(cvc::app &app, const std::string &prefix,
                 const std::function<bool(const std::string &)> &has = {}) {
    // The intrinsics capture &ctx_ by pointer; sched_/tracker_/proc_ back it and share
    // this object's lifetime. No scheduler op is ever exposed, so the scheduler stays
    // idle — it exists only to give the readers a well-formed context (mirrors run_init).
    proc_ = se::make_process();
    proc_->pid = 1;
    proc_->status = se::process_status::ready;
    cvc::state &root = cvc::state::instance(app);
    tree_ = &root;
    ctx_.sched = &sched_;
    ctx_.tracker = &tracker_;
    ctx_.proc = proc_;
    ctx_.pid = 1;
    se::apply_chroot(ctx_, root, prefix);
    root_path_ = prefix;

    // DEFAULT-DENY on two axes — builtins (via the env) AND special forms (via the evaluator
    // gate) — but now leaning on the hardened MECHANISMS rather than a bare scalar allowlist:
    //   * values_equal / to_string are memoized + bounded, so `=`/`!=`/`str` over a shared or
    //     cyclic structure cost its PHYSICAL size, never an exponential unfolding;
    //   * run() arms a per-thread deadline that nested evaluators inherit, so a loop that
    //     lives inside one native step (a method body, a generator drive) aborts at the budget.
    // With those in place a predicate can safely use compound values and structural equality.
    // What stays OUT: the SIZE-DOUBLING materializers (str-concat, append, slice, set-*), the
    // internal-loop / IO / capability builtins (generator/next/range/collect, print, apply,
    // send, every writer/scheduler/watch). And via the special-form gate below, the code-gen
    // and object-graph forms (defmacro/eval/root/defclass/super) — usable in the full-env
    // init:/action lanes, just not in a per-frame predicate.
    env_ = std::make_shared<se::environment>();
    se::environment_ptr full = se::builtins::make_default_environment();
    se::register_intrinsics(full, &ctx_);
    for (const char *name : read_lane_builtins())
      if (const se::value_t *v = full->lookup(name))
        env_->set(name, *v);
    // pure and bounded: a capability / name lookup, answered once a frame per name, and noted
    if (has) {
      has_ = has;
      bind_has(env_, [this](const std::string &name) { return has_answer(name); });
    }
    watch_reads();
    scope_ = &cache_[root_path_];
    watch_ =
        root.childChanged.connect([log = writes_](const std::string &path) { log->note(path); });
    ev_ = std::make_unique<se::stackless_evaluator>(env_);
    // Special-form gate: allow control flow, binding, functions, literals and loops (all
    // step-capped or bounded); DENY the code-generation / object-graph forms.
    ev_->restrict_special_forms(std::make_shared<const std::set<std::string>>(
        std::set<std::string>{"if", "begin", "and", "or", "let", "while", "for", "lambda", "defun",
                              "set", "return", "quote", "yield", "break"}));
    resolve_data_settings();
  }

  // Reset the per-FRAME aggregate budget — call once at the top of each render frame,
  // before the walk. Once the frame's cumulative predicate-eval time is spent, the
  // remaining predicates this frame skip evaluation, and the Runtime keeps their last value,
  // so N reactive widgets cannot together stall the frame. A cached result is not run, so it
  // costs the budget nothing.
  void begin_frame(double budget_seconds = kFrameBudgetSeconds) {
    frame_ = se::time_budget(budget_seconds);
    resolve_data_settings();
    has_frame_.clear();
    if (++frame_no_ % kSweepFrames == 0)
      sweep();
  }

  // How many expressions ran (a result reused from the cache is not one), since it was made.
  std::uint64_t evaluations() const { return evaluated_; }

  // The write generation of the newest write at or under the absolute `path`, or at one of its
  // ancestors (the generation every result is stale from, when the write log overflowed): it
  // moves when, and only when, something there may have changed since. A Kind::Heatmap reads its
  // subtree again only then.
  std::uint64_t written(const std::string &path) {
    sync_writes();
    std::uint64_t g = all_gen_;
    const auto take = [&](const std::unordered_map<std::string, std::uint64_t> &m,
                          const std::string &key) {
      const auto it = m.find(key);
      if (it != m.end())
        g = std::max(g, it->second);
    };
    take(sub_gen_, path);
    for (std::size_t i = path.rfind('.'); i != std::string::npos && i > 0;
         i = path.rfind('.', i - 1))
      take(node_gen_, path.substr(0, i));
    return g;
  }

  // §12: re-chroot the read-lane to `path` so a mounted fragment's predicates read state
  // relative to its own sub-prefix, exactly as its widget binds resolve there. The evaluator,
  // env, and compile cache are unchanged — only ctx_.root moves (apply_chroot repoints it,
  // creating the subtree if absent). A no-op when the scope is unchanged (the common case).
  void set_root_path(cvc::app &app, const std::string &path) {
    if (path == root_path_)
      return;
    se::apply_chroot(ctx_, cvc::state::instance(app), path);
    root_path_ = path;
    scope_ = &cache_[root_path_]; // unordered_map: the element stays where it is
  }

  // §5.30 bind `event` to `ev` (a right-click menu's, while its items are drawn), or back to nil
  // (nullptr). `ev` must outlive the binding. While one is bound no result is reused or kept.
  void set_event(const se::value_t *ev) {
    if (ev == event_)
      return;
    event_ = ev;
    env_->set("event", ev ? *ev : se::value_t());
  }

  // A read function (Runtime::register_read_function), bound beside the allowlist, with this
  // engine's environment for the readers it calls. A result kept from before may have come from
  // the function it replaces, so the kept results go (each scope's map stays where it is).
  void bind_read_function(const std::string &name, const Runtime::ReadFunction &fn) {
    se::environment *env = env_.get();
    env_->set(name,
              se::value_t(se::native_fn([fn, env](se::call_args args) { return fn(args, *env); })));
    for (auto &[scope, kept] : cache_)
      kept.clear();
  }

  struct Outcome {
    bool value;           // predicate result, or the caller's `dflt` on any failure
    std::string error;    // empty on success; a diagnostic otherwise
    bool skipped = false; // not evaluated: the frame's budget was spent (not a broken predicate)
  };
  struct StringOutcome {
    std::string value;    // display string, or the caller's `dflt` on any failure
    std::string error;    // empty on success; a diagnostic otherwise
    bool skipped = false; // not evaluated: the frame's budget was spent
  };

  // Evaluate `src` as a boolean predicate (visible_when / enabled_when / …). Fail-safe: a
  // parse error, a runtime error, or a step/time-cap overrun returns {dflt, <why>}.
  Outcome eval_bool(const std::string &src, bool dflt) {
    const RawOutcome o = eval_raw(src);
    if (!o.error.empty())
      return {dflt, o.error, o.skipped};
    return {o.value.is_truthy(), std::string()};
  }

  // Evaluate `src` and coerce the result to a display string (computed text / fmt / …).
  // Same bounding + fail-safe as eval_bool; on any failure returns {dflt, <why>}.
  StringOutcome eval_string(const std::string &src, const std::string &dflt) {
    const RawOutcome o = eval_raw(src);
    if (!o.error.empty())
      return {dflt, o.error, o.skipped};
    return {display_string(o.value), std::string()};
  }

  struct StringListOutcome {
    std::vector<std::string> value; // empty on failure or a non-list/nil result
    std::string error;              // empty on success
    bool skipped = false;           // not evaluated: the frame's budget was spent
  };

  // Evaluate `src` and coerce a LIST result to display strings (computed combo options). A
  // non-list, non-nil result becomes a single-element list; nil / failure yields empty. The
  // list length is capped (a combo with thousands of options is pathological) — extras are
  // dropped. Same bounding + fail-safe as the others.
  StringListOutcome eval_string_list(const std::string &src) {
    const RawOutcome o = eval_raw(src);
    if (!o.error.empty())
      return {{}, o.error, o.skipped};
    std::vector<std::string> out;
    if (auto *l = std::get_if<se::list_ptr>(&o.value.v)) {
      if (*l)
        for (const auto &e : **l) {
          if (out.size() >= kMaxOptions)
            break;
          out.push_back(display_string(e));
        }
    } else if (!o.value.is_nil()) {
      out.push_back(display_string(o.value));
    }
    return {std::move(out), std::string()};
  }

  struct RunsOutcome {
    std::vector<TextRunSpec> value; // empty on failure or nil
    std::string error;              // empty on success
    bool skipped = false;           // not evaluated: the frame's budget was spent
  };

  // Evaluate `src` to a rich text's runs (§5.20): a list of strings (plain runs) and dicts of
  // text / style / href / tooltip; a lone string or dict is one run. A dict's on_click is not
  // taken: runs a program computes are data, and data never runs. Capped at kMaxRuns.
  RunsOutcome eval_runs(const std::string &src) {
    const RawOutcome o = eval_raw(src);
    if (!o.error.empty())
      return {{}, o.error, o.skipped};
    RunsOutcome out;
    const auto field = [](const se::dict_ptr &d, const char *key) {
      for (const auto &[k, v] : *d)
        if (k == key)
          return display_string(v);
      return std::string();
    };
    const auto add = [&](const se::value_t &e) {
      TextRunSpec r;
      if (const auto *d = std::get_if<se::dict_ptr>(&e.v)) {
        if (!*d)
          return;
        r.text = field(*d, "text");
        r.style = field(*d, "style");
        r.href = field(*d, "href");
        r.tooltip = field(*d, "tooltip");
      } else if (e.is_nil()) {
        return;
      } else {
        r.text = display_string(e);
      }
      out.value.push_back(std::move(r));
    };
    if (const auto *l = std::get_if<se::list_ptr>(&o.value.v)) {
      if (*l)
        for (const se::value_t &e : **l) {
          if (out.value.size() >= kMaxRuns)
            break;
          add(e);
        }
    } else {
      add(o.value);
    }
    return out;
  }

  struct ColorsOutcome {
    std::vector<ItemColor> value; // none on failure or nil
    std::string error;            // empty on success
    bool skipped = false;         // not evaluated: the frame's budget was spent
  };

  // §5.26 evaluate `src` to colours: a colour's text or a ';' list of them (color_value.h), three
  // numbers [r, g, b], or a list of colours (each a text or [r, g, b], nil unset). nil is none.
  ColorsOutcome eval_colors(const std::string &src) {
    const RawOutcome o = eval_raw(src);
    if (!o.error.empty())
      return {{}, o.error, o.skipped};
    const auto bad = [&src]() {
      return ColorsOutcome{{},
                           "ari: color: \"" + src +
                               "\" did not give a colour (r,g,b, [r, g, b], #rrggbb, c0..c7, or "
                               "a list of them)"};
    };
    // Three numbers, clamped to 0..1.
    const auto rgb_of = [](const se::list_ptr &l, float out[3]) {
      if (!l || l->size() != 3)
        return false;
      for (std::size_t i = 0; i < 3; ++i) {
        const se::value_t &e = (*l)[i];
        if (const auto *n = std::get_if<int64_t>(&e.v))
          out[i] = static_cast<float>(*n);
        else if (const auto *d = std::get_if<double>(&e.v); d && std::isfinite(*d))
          out[i] = static_cast<float>(*d);
        else
          return false;
        out[i] = std::clamp(out[i], 0.0f, 1.0f);
      }
      return true;
    };
    ColorsOutcome out;
    if (o.value.is_nil())
      return out;
    if (const auto *t = std::get_if<std::string>(&o.value.v)) {
      if (!parse_color_list(*t, out.value))
        return bad();
      return out;
    }
    const auto *l = std::get_if<se::list_ptr>(&o.value.v);
    if (!l)
      return bad();
    ItemColor one;
    if ((one.set = rgb_of(*l, one.rgb))) {
      out.value.push_back(one);
      return out;
    }
    if (*l)
      for (const se::value_t &e : **l) {
        if (out.value.size() >= kMaxOptions)
          break;
        ItemColor c;
        if (const auto *et = std::get_if<std::string>(&e.v)) {
          if (!is_no_color(*et) && !(c.set = parse_color_text(*et, c.rgb)))
            return bad();
        } else if (const auto *el = std::get_if<se::list_ptr>(&e.v)) {
          if (!(c.set = rgb_of(*el, c.rgb)))
            return bad();
        } else if (!e.is_nil()) {
          return bad();
        }
        out.value.push_back(c);
      }
    return out;
  }

  struct IntOutcome {
    int64_t value;        // the caller's `dflt` on any failure
    std::string error;    // empty on success
    bool skipped = false; // not evaluated: the frame's budget was spent
  };

  // Evaluate `src` and coerce the result to an integer (a repeat count). int as-is, double
  // truncated, a numeric string parsed; anything else is an error. Bounded + fail-safe.
  IntOutcome eval_int(const std::string &src, int64_t dflt) {
    const RawOutcome o = eval_raw(src);
    if (!o.error.empty())
      return {dflt, o.error, o.skipped};
    if (auto *i = std::get_if<int64_t>(&o.value.v))
      return {*i, std::string()};
    if (auto *d = std::get_if<double>(&o.value.v)) {
      // Casting a non-finite or out-of-range double to int64 is UB — guard it (a repeat
      // count that computes to inf/nan/huge is a broken expr -> fail-safe dflt).
      if (!std::isfinite(*d) || *d < -9.2e18 || *d > 9.2e18)
        return {dflt, "ari: reactive expr \"" + src + "\" did not yield an in-range integer"};
      return {static_cast<int64_t>(*d), std::string()};
    }
    if (auto *s = std::get_if<std::string>(&o.value.v)) {
      try {
        return {static_cast<int64_t>(std::stoll(*s)), std::string()};
      } catch (const std::exception &) {
        return {dflt, "ari: reactive expr \"" + src + "\" did not yield an integer"};
      }
    }
    return {dflt, "ari: reactive expr \"" + src + "\" did not yield a number"};
  }

private:
  struct RawOutcome {
    se::value_t value;    // valid only when error is empty
    std::string error;    // empty on success; a diagnostic otherwise (contains src + cause)
    bool skipped = false; // the per-frame budget was spent before it ran
  };

  // The shared evaluate path for every read-lane expression: compile-cached, bounded three
  // ways (per-eval step cap + per-eval time cap + per-frame aggregate), never throws out.
  RawOutcome eval_raw(const std::string &src) {
    const se::value_t *expr = compile(src);
    if (!expr)
      return {{}, "ari: reactive expr parse error in \"" + src + "\""};
    sync_writes();
    // §5.30 with a menu's `event` bound, a result may depend on it: neither reused nor kept.
    const auto hit = event_ ? scope_->end() : scope_->find(src);
    if (hit != scope_->end()) {
      if (fresh(hit->second)) { // nothing it read was written: last time's result
        hit->second.used = frame_no_;
        return {hit->second.value, std::string()};
      }
      scope_->erase(hit);
    }
    const double remaining = frame_.remaining();
    if (remaining <= 0.0)
      return {{}, "ari: reactive expr \"" + src + "\" skipped — per-frame budget exhausted", true};
    struct charge_frame { // charges the eval, and its steps, to the frame however run() ends
      se::time_budget &frame;
      const se::evaluator_state &st;
      ~charge_frame() { frame.stop(st.stats.get_step_count()); }
    };
    Reads reads; // what it reads, noted by the readers' wrappers
    rec_ = &reads;
    struct stop_noting {
      Reads *&rec;
      ~stop_noting() { rec = nullptr; }
    } const stop{rec_};
    const std::uint64_t made_at = gen_;
    ++evaluated_;
    try {
      se::evaluator_state st = ev_->create_state(*expr);
      st.data_step_nodes = data_step_nodes_;
      // The render lane can't wait (docs/STATE_EXEC_EXCEPTIONS.md 3.3): an async function is
      // unbound here, and one reached anyway raises RuntimeError instead of parking or blocking.
      const se::park_forbidden_scope no_parking(ev_->context());
      frame_.start();
      const charge_frame charge{frame_, st};
      se::value_t r = ev_->run(st, kMaxSteps, std::min(kMaxSeconds, remaining));
      if (!st.done) // step cap: run() returns nil with done==false
        return {{}, "ari: reactive expr \"" + src + "\" exceeded the eval budget (step cap)"};
      if (!reads.live && !event_)
        keep(src, r, std::move(reads), made_at);
      return {std::move(r), std::string()};
    } catch (const se::evaluation_timeout &) {
      return {{}, "ari: reactive expr \"" + src + "\" exceeded the eval budget (time cap)"};
    } catch (const se::evaluation_interrupted &) {
      return {{}, "ari: reactive expr \"" + src + "\" exceeded the eval budget (interrupted)"};
    } catch (const std::exception &e) {
      return {{}, "ari: reactive expr \"" + src + "\" failed at eval (" + e.what() + ")"};
    }
  }

  // Coerce a result value to a display string: a string is shown RAW (no quotes), a bool as
  // true/false, nil as empty; anything else via the bounded to_string (numbers → digits, a
  // list/dict → its readable, length-capped form).
  static std::string display_string(const se::value_t &v) {
    if (auto *s = std::get_if<std::string>(&v.v))
      return *s;
    if (auto *b = std::get_if<bool>(&v.v))
      return *b ? "true" : "false";
    if (v.is_nil())
      return std::string();
    return se::to_string(v);
  }

  // Parse once; cache the AST. A parse failure caches std::nullopt so a broken predicate
  // is not re-parsed every frame. Returns nullptr on a (cached) parse failure.
  const se::value_t *compile(const std::string &src) {
    auto it = compiled_.find(src);
    if (it != compiled_.end())
      return it->second ? &*it->second : nullptr;
    try {
      se::value_t v = se::parse(src, se::parse_options_from_state(*tree_)); // parser.* settings
      auto ins = compiled_.emplace(src, std::move(v));
      return &*ins.first->second;
    } catch (const se::parse_error &) {
      compiled_.emplace(src, std::nullopt);
      return nullptr;
    }
  }

  // The deep-data slice (data.step_nodes) every eval runs with: the tree's state_exec.defaults
  // over the host default. The caps are checked only between steps, so a predicate always
  // slices: a 0 there (no slicing) gets the host default here.
  void resolve_data_settings() {
    const se::data_options host = se::async_scheduler::host_data_defaults();
    data_step_nodes_ = se::data_options_from_state(*tree_, {}, host).step_nodes;
    if (data_step_nodes_ == 0)
      data_step_nodes_ = host.step_nodes;
  }

  // ---- §2.1 the result cache ----------------------------------------------------------------

  // What one evaluation read: the state paths (absolute, normalized; a whole scope as its root),
  // the (has? …) names it asked and their answers, and whether it read something no write signals
  // (the clock, an expiry, a node through a link), which makes its result one not to keep.
  struct Reads {
    std::vector<std::string> paths;
    std::vector<std::pair<std::string, bool>> asked;
    bool live = false;
  };
  struct Cached {
    se::value_t value;
    Reads reads;
    std::uint64_t gen = 0;  // the write generation it was made at
    std::uint64_t used = 0; // the frame it was last used in
  };

  // The paths written since the frame thread last looked, noted on the writer's thread.
  struct WriteLog {
    std::mutex mu;
    std::vector<std::string> paths;
    bool overflow = false; // more than kMaxPendingWrites: every result is stale
    std::atomic<std::uint64_t> count{0};
    void note(const std::string &path) {
      std::lock_guard<std::mutex> lock(mu);
      if (paths.size() < kMaxPendingWrites)
        paths.push_back(path);
      else
        overflow = true;
      count.fetch_add(1, std::memory_order_release);
    }
  };

  // Wrap each state reader to note the path it reads, and the clock and expiry readers to mark a
  // result live (never kept). One bound here but not as a plain native can't be wrapped: an
  // expression that names it is never kept.
  void watch_reads() {
    const auto wrap = [this](const char *name, bool live) {
      const se::value_t *v = env_->lookup(name);
      if (!v)
        return; // not bound here: nothing to note
      const se::native_fn *fn = std::get_if<se::native_fn>(&v->v);
      if (!fn) {
        unwatched_.emplace_back(name);
        return;
      }
      se::native_fn inner = *fn;
      env_->set(name,
                se::value_t(se::native_fn([this, inner, live](se::call_args args) -> se::value_t {
                  if (rec_) {
                    if (live)
                      rec_->live = true;
                    else
                      note_read(args);
                  }
                  return inner(args);
                })));
    };
    for (const char *name :
         {"state-get", "state-exists", "state-children", "state-data-get", "state-json"})
      wrap(name, false);
    for (const char *name : {"time-now", "state-is-expired", "state-has-expiry"})
      wrap(name, true);
  }

  // A reader's path, absolute: under the scope; no path (or not a string) reads the whole scope.
  // A read through a link sees the target, whose writes are signalled under the target's own
  // path, so such a result is not kept.
  void note_read(std::span<const se::value_t> args) {
    const std::string *rel = args.empty() ? nullptr : std::get_if<std::string>(&args[0].v);
    if (rel && !rel->empty() && ctx_.root) {
      std::vector<cvc::state::state_ptr> chain;
      ctx_.root->findThroughLinksShared(*rel, &chain);
      for (const cvc::state::state_ptr &n : chain)
        if (n && n->isLink()) {
          rec_->live = true;
          return;
        }
    }
    const std::string path = !rel || rel->empty() ? root_path_
                             : root_path_.empty() ? *rel
                                                  : root_path_ + "." + *rel;
    rec_->paths.push_back(cvc::state::normalize_path(path));
  }

  // (has? NAME), asked once a frame per name; an evaluation notes what it asked. A grant token
  // (ARIADNE_SHELL.md 4.5) is answered for the scope the engine reads, so once a frame per scope.
  bool has_answer(const std::string &name) {
    const std::string key = is_grant_token(name) ? root_path_ + '\x1f' + name : name;
    const auto it = has_frame_.find(key);
    const bool v = it != has_frame_.end() ? it->second : (has_frame_[key] = has_(name));
    if (rec_)
      rec_->asked.emplace_back(name, v);
    return v;
  }

  // The writes since the last look, as generations: a written path's own (node_gen_), and the
  // newest write at or under each path (sub_gen_, the root's "" included).
  void sync_writes() {
    if (writes_->count.load(std::memory_order_acquire) == writes_seen_)
      return;
    std::vector<std::string> paths;
    bool overflow = false;
    {
      std::lock_guard<std::mutex> lock(writes_->mu);
      paths.swap(writes_->paths);
      overflow = writes_->overflow;
      writes_->overflow = false;
      writes_seen_ = writes_->count.load(std::memory_order_relaxed);
    }
    ++gen_;
    if (overflow || node_gen_.size() + sub_gen_.size() > kMaxWriteKeys) {
      node_gen_.clear();
      sub_gen_.clear();
      all_gen_ = gen_; // every result made before now is stale
      return;
    }
    for (const std::string &p : paths) {
      node_gen_[p] = gen_;
      sub_gen_[p] = gen_;
      for (std::size_t i = p.rfind('.'); i != std::string::npos && i > 0; i = p.rfind('.', i - 1))
        sub_gen_[p.substr(0, i)] = gen_;
      sub_gen_[std::string()] = gen_;
    }
  }

  // Whether `c` is what running it now would give: no write since on what it read (the path, an
  // ancestor or a descendant), and the same (has? …) answers.
  bool fresh(const Cached &c) {
    for (const auto &[name, v] : c.reads.asked)
      if (has_answer(name) != v)
        return false;
    if (c.gen == gen_)
      return true;
    if (c.gen < all_gen_)
      return false;
    const auto newer = [&](const std::unordered_map<std::string, std::uint64_t> &m,
                           const std::string &key) {
      const auto it = m.find(key);
      return it != m.end() && it->second > c.gen;
    };
    for (const std::string &r : c.reads.paths) {
      if (newer(sub_gen_, r))
        return false;
      for (std::size_t i = r.rfind('.'); i != std::string::npos && i > 0; i = r.rfind('.', i - 1))
        if (newer(node_gen_, r.substr(0, i)))
          return false;
    }
    return true;
  }

  void keep(const std::string &src, const se::value_t &v, Reads reads, std::uint64_t made_at) {
    for (const std::string &name : unwatched_)
      if (src.find(name) != std::string::npos)
        return; // it may have read what no wrapper noted
    std::sort(reads.paths.begin(), reads.paths.end());
    reads.paths.erase(std::unique(reads.paths.begin(), reads.paths.end()), reads.paths.end());
    Cached &c = (*scope_)[src];
    c.value = v;
    c.reads = std::move(reads);
    c.gen = made_at;
    c.used = frame_no_;
  }

  // Drop the results no frame used for kSweepFrames (a repeat's rows scrolled away, a closed
  // window), and the scopes left empty (not the current one).
  void sweep() {
    for (auto s = cache_.begin(); s != cache_.end();) {
      for (auto it = s->second.begin(); it != s->second.end();)
        it = frame_no_ - it->second.used > kSweepFrames ? s->second.erase(it) : std::next(it);
      s = s->second.empty() && &s->second != scope_ ? cache_.erase(s) : std::next(s);
    }
  }

  static constexpr std::size_t kMaxPendingWrites = 4096; // more in one frame: all stale
  static constexpr std::size_t kMaxWriteKeys = 65536;    // more paths written: start over
  static constexpr std::uint64_t kSweepFrames = 300;

  std::function<bool(const std::string &)> has_;
  std::unordered_map<std::string, bool> has_frame_; // this frame's (has? …) answers
  std::vector<std::string> unwatched_;              // readers bound but not wrapped
  // scope -> expression source -> result
  std::unordered_map<std::string, std::unordered_map<std::string, Cached>> cache_;
  std::unordered_map<std::string, Cached> *scope_ = nullptr; // the current scope's
  std::shared_ptr<WriteLog> writes_ = std::make_shared<WriteLog>();
  boost::signals2::scoped_connection watch_;
  std::uint64_t writes_seen_ = 0, gen_ = 0, all_gen_ = 0;
  std::unordered_map<std::string, std::uint64_t> node_gen_, sub_gen_;
  Reads *rec_ = nullptr; // the evaluation running now, noting what it reads
  std::uint64_t frame_no_ = 0, evaluated_ = 0;
  const se::value_t *event_ = nullptr; // set_event's: bound as `event` while a menu's items draw

  static constexpr uint64_t kMaxSteps = 100'000; // per-eval step cap (a predicate is tiny)
  static constexpr double kMaxSeconds = 0.005;   // per-eval time cap (5 ms)
  // Aggregate reactive-eval time across ALL predicates in a frame. This is a runaway guard, not a
  // normal operating point: a handful of scalar predicates finish in well under a millisecond. It
  // was 10 ms, but a live conference re-evaluates ~5-7 predicates PER PEER every frame (roster
  // rows' visible_when, the per-peer camera-window count/title/texture binds), and the stackless
  // evaluator is markedly slower under wasm while the browser's single thread is also decoding
  // video — so at a realistic peer count the 10 ms guard tripped mid-roster, skipping the rest and
  // blinking the participant list / peer windows empty. 30 ms clears a ~6-7 peer conference with
  // margin and is still a hard bound on a pathological document (eval time the thread spends
  // descheduled doesn't count against it, so a merely-loaded machine never trips it). See
  // last_good_count for the structural backstop that keeps a repeated list stable across any
  // residual skip.
  static constexpr double kFrameBudgetSeconds = 0.030; // aggregate across all predicates/frame
  static constexpr std::size_t kMaxOptions = 1024;     // cap on computed combo options

  std::size_t data_step_nodes_ = 0;            // see resolve_data_settings()
  se::time_budget frame_{kFrameBudgetSeconds}; // the current frame's predicate evals' time
  std::string root_path_;      // §12: the scope the ctx_ is currently chrooted to (set_root_path)
  cvc::state *tree_ = nullptr; // the app's state tree, for state_exec.defaults.parser.*
  se::scheduler sched_;
  se::memory_tracker tracker_;
  std::shared_ptr<se::process> proc_;
  se::intrinsics_context ctx_;
  se::environment_ptr env_;
  std::unique_ptr<se::stackless_evaluator> ev_;
  std::unordered_map<std::string, std::optional<se::value_t>> compiled_;
};

// The app's exec scheduler if something has built it, else nullptr; never builds it. app::
// exec_scheduler() builds it on first call and cvc::app has no accessor that does not, so this
// reads the same handle under the same mutex, through pointers to its protected members that a
// class derived from cvc::app may form ([class.protected]). Once built, the scheduler lives as long
// as its app, so a Runtime may keep the pointer.
struct exec_scheduler_probe : cvc::app {
  static cvc::state_exec::async_scheduler *built(cvc::app &a) {
    boost::mutex::scoped_lock lock(a.*(&exec_scheduler_probe::_execSchedulerMutex));
    return (a.*(&exec_scheduler_probe::_execScheduler)).get();
  }
};

// The frame boundaries an app's Runtimes share. Each Runtime pumps the one app scheduler in its own
// drain(), so with N Runtimes an app frame holds N pumps, and only one of them may re-ready the
// (await …) frame-yielders, or an await would cross a 1/N of a frame. A pump starts the next frame
// exactly when its own Runtime has pumped since the last boundary: every Runtime drains once a
// frame, so its second pump since a boundary belongs to the next frame. There is no leader, so a
// Runtime that stops drawing holds nothing up, and a lone Runtime starts a frame at every pump.
// Kept in the app's data map (like the app's other per-app registries), so it dies with the app.
struct exec_pump_clock {
  std::mutex mutex;
  std::uint64_t frames = 0; // boundaries started so far
};

std::shared_ptr<exec_pump_clock> pump_clock_of(cvc::app &app) {
  static const char kKey[] = "__ariadne_exec_pump_clock";
  const cvc::mutex_ptr guard = app.mutex(kKey); // check-then-create, as for the app's state root
  boost::mutex::scoped_lock lock(*guard);
  std::shared_ptr<exec_pump_clock> clock;
  try {
    clock = app.data<std::shared_ptr<exec_pump_clock>>(kKey);
  } catch (const std::exception &) {
    // not yet present
  }
  if (!clock) {
    clock = std::make_shared<exec_pump_clock>();
    app.data(kKey, clock);
  }
  return clock;
}

// One pump of a document's scheduler a frame (drain_end, exec_pump_frame): a quick action
// completes within it, and the step and time backstops bound it.
constexpr uint64_t kPumpMaxSteps = 200000;
constexpr double kPumpMaxSeconds = 0.1;

// The host-thread calls (S1.10): post_input, post_signal and render start, post to and kill
// residents, so they run on the host thread between pumps, never inside the app pool's pump_all
// (from a native, an on_complete callback or a host_proxied body). Debug builds check it (AR-3's
// latch), through the probe, which never builds the pool. Stage 2: nor inside a pump of the
// document's own group outside any pump_all (an attached group's, on a worker or its attaching
// thread; S2-Q6 keeps Ariadne's groups fork-join).
void assert_outside_pump(const cvc::app &app, const cvc::state_exec::exec_group *group) {
#ifndef NDEBUG
  if (const cvc::exec_pool *pool = app.exec_pool_probe())
    assert(!pool->in_pump_all() && "ariadne: a host-thread call made inside a pump");
  assert(!(group && group->pumping_here()) && "ariadne: a host-thread call made inside a pump");
#else
  (void)app;
  (void)group;
#endif
}

// Tearing a Runtime down and moving a document out of its group (c-m4, R1b skeptic m2) kill and
// reap on that group's scheduler and take its group away: while a pump_all holds the group (from a
// host verb or body of that call, or from another thread), a worker may be pumping it, and the
// group's own teardown would abort anyway. That is refused in every build, from any thread: logged,
// then aborted before anything is touched. A group-0 document's teardown or move is not refused.
// An attached group (Stage 2; Ariadne's never are, S2-Q6) is held by no pump_all: the teardown or
// move waits for a worker's pump instead (detach_begin, S2-Q4), and no worker pumps it after.
void refuse_if_held(cvc::app &app, cvc::state_exec::exec_group *group,
                    const std::string &what) noexcept {
  if (!group || cvc::detail::detach_begin(*group))
    return;
  if (group->pool_link().call.load(std::memory_order_acquire) == nullptr)
    return; // held from its pump_all's entry until that call returns (exec_pool.cpp)
  const std::string text = "ari: " + what +
                           " while a pump of the app's pool holds its group: refused (host thread, "
                           "outside a pump)";
  std::fprintf(stderr, "%s\n", text.c_str());
  std::fflush(stderr);
  try {
    app.log(1, text);
  } catch (...) {
  }
  std::abort();
}

// ariadne.exec.isolate_documents (S1-Q6): on for "1" or "true".
bool isolate_documents_on(cvc::state &root) {
  cvc::state *node = root.findDescendant("ariadne.exec.isolate_documents");
  if (!node)
    return false;
  std::string v = node->value();
  std::transform(v.begin(), v.end(), v.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v == "1" || v == "true";
}

// The channels a program names statically: the string-literal first argument of each (msg-send
// ...), (msg-recv ...) and (msg-pending ...), as the loader's channel lint reads them. The walk
// keeps its open lists on a heap stack, so any depth the parser accepts is read.
void static_channels(const cvc::state_exec::value_t &v, std::vector<std::string> &out) {
  namespace se = cvc::state_exec;
  std::vector<std::pair<const std::vector<se::value_t> *, std::size_t>> stack;
  const auto enter = [&](const se::value_t &e) {
    const auto *lp = std::get_if<se::list_ptr>(&e.v);
    if (!lp || !*lp)
      return;
    const std::vector<se::value_t> &l = **lp;
    if (l.size() >= 2)
      if (const auto *head = std::get_if<se::symbol>(&l[0].v))
        if (head->name == "msg-send" || head->name == "msg-recv" || head->name == "msg-pending")
          if (const auto *s = std::get_if<std::string>(&l[1].v))
            out.push_back(*s);
    stack.emplace_back(&l, 0);
  };
  enter(v);
  while (!stack.empty()) {
    auto &[elems, next] = stack.back();
    if (next == elems->size()) {
      stack.pop_back();
      continue;
    }
    enter((*elems)[next++]); // may push, invalidating the reference
  }
}
} // namespace

namespace {
// The owner ids of an app's Runtimes: each tags its processes on the app's scheduler.
struct runtime_owner_ids {
  static constexpr const char *app_slot = "cvc.ariadne.runtime-owner-ids";
  std::atomic<std::uint64_t> next{0};
};
std::uint64_t next_runtime_owner_id(cvc::app &app) {
  return app.slot<runtime_owner_ids>().next.fetch_add(1);
}
} // namespace

struct Runtime::Impl {
  cvc::app &app;
  Backend *backend = nullptr;
  std::string prefix;
  // The backend as the host verbs see it (bind_backend_verbs): a program may outlive this Runtime
  // (a spawned child), and then finds none.
  std::shared_ptr<Backend *> backend_ref = std::make_shared<Backend *>(nullptr);
  // §10.7 the verbs the core answers from the backend: (clipboard-set TEXT) where it has a
  // clipboard, and (fullscreen [ON]) everywhere (#f where the window can't).
  void bind_backend_verbs(const std::shared_ptr<cvc::state_exec::environment> &env) const;
  // §5.24 what a program's notify verbs (bind_notify_verbs) take from the Runtime: the scope it
  // runs in, the source the feed names for it and a copy of the mounts' names. Taken on the host
  // thread, where mount_names is written, when its context is made (notify_scope); a spawned
  // child, made on the thread that pumps its group (a worker in an `any` group), binds from its
  // parent's.
  struct NotifyScope {
    std::string scope;
    std::string source;
    std::map<std::string, std::string> mounts;
  };
  // bind_backend_verbs and bind_notify_verbs (for `notify`) by the kind of `group`
  // (finish_context, the name probe): host_proxied in an `any` group, as they are elsewhere
  // (null: group 0, the init lane). `host` is the program's async host (ictx.host), which
  // (file-pick) parks it on.
  void install_host_verbs(const std::shared_ptr<cvc::state_exec::environment> &env,
                          cvc::state_exec::exec_group *group, cvc::state_exec::async_host &host,
                          const NotifyScope &notify) const;
  // §10.8 (file-pick): whether a picker is open (one at a time), and the Runtime's own "Open
  // file" window, drawn by render() where the host has no picker. Shared with the verb, as a
  // program may outlive this Runtime. Drawn and opened on the drawing thread; `cancel` is set by a
  // killed program's hook, from wherever that runs, and render() closes the window then.
  struct FilePickHub {
    std::mutex m;
    bool busy = false;
    std::atomic<bool> cancel{false};
    ComposedFilePicker composed;
  };
  std::shared_ptr<FilePickHub> file_pick = std::make_shared<FilePickHub>();
  // (file-pick), (file-info) and (file-forget), bound into the program's env as they are in every
  // group (install_host_verbs): file-info and file-forget lock the app's PickedFiles, and
  // file-pick parks its program on `host` and opens the picker on the host thread: through the
  // host-call queue of the pump_all holding `group` (null: group 0, the init lane, inline).
  void bind_file_verbs(const std::shared_ptr<cvc::state_exec::environment> &env,
                       cvc::state_exec::exec_group *group, cvc::state_exec::async_host &host) const;

  // §5.29 saving a picture or a video: a context menu's Save image... / Save video..., and
  // (save-url). A save's bytes are fetched as the widget's are (image_cache().fetch), the file is
  // named from the URL and what the bytes are, and then the backend's Save dialog asks where (the
  // file is written there), a browser page downloads it, or it goes into the Downloads folder as a
  // new file. Each step that answers off the frame (a fetch, a dialog, a write) hands the next one
  // to the hub, which drain() runs on the drawing thread; one a Runtime that is gone never ran is
  // abandoned, and its save told why. The hub is shared with the verb, whose program may outlive
  // this Runtime.
  struct SaveOutcome {
    bool ok = false;
    bool cancelled = false; // the user closed the dialog
    bool browser = false;   // a browser page downloads it: no path here
    std::string name;       // the file's name
    std::string path;       // where it was written (natively)
    std::uint64_t size = 0;
    std::string error;
  };
  struct SaveJob {
    MediaSave save;
    bool menu = false; // a context menu's: a failure is a notification
    // (save-url)'s program: called once, from any thread, with the outcome.
    std::function<void(const SaveOutcome &)> done;
  };
  struct SaveHub {
    struct Step {
      std::function<void()> run;     // on the drawing thread, from drain()
      std::function<void()> abandon; // when the Runtime goes first: tell the save
    };
    std::mutex m;
    std::vector<SaveJob> asked; // (save-url)'s, from wherever its program runs
    std::vector<Step> steps;
    // The drawing thread's alone: a Save dialog is open (one at a time), and where a save with no
    // dialog goes ("" the Downloads folder).
    bool dialog_open = false;
    std::string dir;
    void post(Step s) {
      std::lock_guard<std::mutex> lock(m);
      steps.push_back(std::move(s));
    }
    void ask(SaveJob j) {
      std::lock_guard<std::mutex> lock(m);
      asked.push_back(std::move(j));
    }
    ~SaveHub() {
      for (Step &s : steps)
        if (s.abandon)
          s.abandon();
      SaveOutcome gone;
      gone.error = "the document is gone";
      for (SaveJob &j : asked)
        if (j.done)
          j.done(gone);
    }
  };
  std::shared_ptr<SaveHub> saves = std::make_shared<SaveHub>();
  // A browser page's save a context menu opened on (Backend::media_save_prepare): its bytes are
  // fetched and staged on the page, so the pointer's release over Save downloads them inside the
  // click. The saves chosen before they came wait for them.
  struct PreparedSave {
    std::string key; // media_save_key
    long gen = 0;
    bool staged = false;
    std::chrono::steady_clock::time_point staged_at; // the page lets it go after two minutes
    SaveOutcome outcome;                             // what its saves are told (name, size)
    std::vector<SaveJob> waiting;
  };
  std::unique_ptr<PreparedSave> prepared_save;
  long prepared_gen = 0;
  void pump_saves();                     // drain(): the saves asked for, and their steps
  void start_save(SaveJob job);          // fetch the bytes
  void prepare_save(const MediaSave &m); // a browser page: fetch now, stage on the page
  void save_fetched(SaveJob job, media::FetchResult r);
  void save_deliver(SaveJob job, const std::string &name, const std::string &mime,
                    std::shared_ptr<const std::string> bytes);
  void save_write(SaveJob job, const std::string &path, const std::string &name,
                  std::shared_ptr<const std::string> bytes, bool fresh);
  void finish_save(const SaveJob &job, const SaveOutcome &o);
  // The file name and type of `bytes` saved for `m` (its own name and mime, else the URL's and
  // what the bytes are).
  static void save_naming(const MediaSave &m, const std::string &bytes, std::string &name,
                          std::string &mime);
  // §5.29 the picture just drawn for `w` offers Save (unless `save: false`).
  void offer_save(const Widget &w, const std::string &uri, bool video) {
    if (!w.save_menu || !media::is_uri(uri))
      return;
    MediaSave m;
    m.url = uri;
    m.auth = w.media_auth;
    m.video = video;
    backend->item_save(m);
  }

  // The document's own program-lane intrinsics (add_program_intrinsics), in order, each with its
  // kind: bound after the app's into every program env this Runtime builds, and into no other
  // Runtime's, by kind in an `any` group as the app's are (install_providers). Added on the
  // drain's thread and read where an env is built, a spawned child's on its pumping thread, so
  // under own_mutex: a reader copies the list and runs it unlocked (a provider binds, and adds
  // none).
  mutable std::mutex own_mutex;
  std::vector<registered_provider> own_intrinsics;
  std::vector<registered_provider> own_snapshot() const {
    std::lock_guard<std::mutex> lock(own_mutex);
    return own_intrinsics;
  }
  // `group` as install_providers takes it: null for group 0, the init lane and a probe.
  void bind_own_intrinsics(const std::shared_ptr<cvc::state_exec::environment> &env,
                           cvc::state_exec::intrinsics_context &ictx,
                           cvc::state_exec::exec_group *group) const {
    install_providers(own_snapshot(), env, ictx, group);
  }

  // What this host provides (requires:, has?). The names a program can reference are read from a
  // probe env built exactly like an action's, then cached; the cache is rebuilt when a provider is
  // registered (the registry's generation moves), add_program_intrinsics adds one, or
  // refresh_program_names() drops it.
  bool scene_capable = false;
  bool scene_shaders = false;
  bool relay_document = false;
  mutable std::mutex names_mutex;
  mutable bool names_valid = false;
  mutable std::uint64_t names_generation = 0;
  mutable std::set<std::string> names;
  void load_names() const; // names_mutex held
  bool has_function(const std::string &name) const {
    if (cvc::state_exec::stackless_evaluator::is_special_form(name))
      return true;
    std::lock_guard<std::mutex> lock(names_mutex);
    load_names();
    return names.count(name) != 0;
  }
  HostFacts facts() const {
    HostFacts f;
    f.app = &app;
    if (backend)
      f.caps = backend->capabilities();
    f.scene = scene_capable;
    f.scene_shaders = scene_capable && scene_shaders;
    const NetHostStatus net = net_host_status(app);
    f.net = net.enabled;
    f.net_auth = net.auth;
    f.net_auth_persist = net.persist;
    f.media_player = media::have_player_factory(app); // §5.13
    f.media = f.media_player && media::may_play_here(app);
    f.oidc = net.oidc;
    f.oidc_loopback = net.oidc && net.oidc_flows.loopback;
    f.oidc_device = net.oidc && net.oidc_flows.device;
    f.oidc_redirect = net.oidc && net.oidc_flows.redirect;
    f.browser = net.enabled && net.browser;
    f.relay = relay_document;
    return f;
  }
  // `name` here, for the document's own programs: a grant token as the outermost document alone
  // holds it (ARIADNE_SHELL.md 4.5; a mount's are grant_holds').
  bool has(const std::string &name) const {
    if (is_grant_token(name))
      return held_alone(name);
    if (is_capability_token(name))
      return capability_holds(name, facts());
    return has_function(name) || is_builtin_widget_type(name) || has_widget_type(app, name);
  }

  // §12 channel enforcement: the document's channel policy, installed on every action/resident
  // ictx. Default (enforce=false, empty) = permissive; set_channel_policy fills it from a
  // LoadResult. Pointed-to by ictx.channels (non-owning), so it must outlive the contexts — it
  // lives here in the Impl, which owns the live_actions_ / residents that reference it. Declared
  // before them so it destructs after them.
  cvc::state_exec::channel_policy channel_policy_;

  // Per-document resource owner (Phase-2 streaming PR6): the non-singleton home for resources a DSL
  // intrinsic opens during this document's life — e.g. a (stream-open) stream. Installed on every
  // action/resident ictx (ictx.document), so a verb reaches it via ictx, NOT a process-static table
  // keyed by app*. ~Impl clear()s it FIRST (below), so anything the author left open is torn down
  // here while the app and its state tree are still alive, never at process exit against a dead
  // app.
  cvc::state_exec::document_scope doc_scope_;

  Widget root;
  Widget pending;
  bool has_pending = false;

  std::unordered_map<std::string, std::function<void()>> handlers;
  // A queued action fired this frame (§7.2/§4.7 deferred intent buffer). `event` is EITHER a bare
  // event name (looked up in `handlers`, the host C++ seam) OR — when it starts with '(' — a
  // state_exec PROGRAM (the action lane), run at drain against `prefix` (the mount scope captured
  // at enqueue time, so a mounted fragment's program writes its own subtree, like its binds).
  struct QueuedAction {
    std::string event;
    std::string prefix;
    // §4.6 event scope: pointer data for a widget-level on_click/hover/drag/drag_start/drag_end
    // handler. When has_event, a PROGRAM action runs with an `event` dict bound (x/y/dx/dy/button).
    bool has_event = false;
    float ex = 0.0f, ey = 0.0f, edx = 0.0f, edy = 0.0f;
    int ebutton = 0;
    // Otherwise, an `event` dict of these strings (a message_input's sent text), when any, and
    // these values (a heatmap cell's row, col and v).
    std::vector<std::pair<std::string, std::string>> event_fields;
    std::vector<std::pair<std::string, cvc::state_exec::value_t>> event_values;
    // Runtime::run_program: told how the program ended instead of a warning.
    std::function<void(const std::string &)> done;
    // §6.6 the functions of the scope it was started in (a FunctionScope prelude), run first.
    std::string functions;
    // A latest-only handler's key (on_change: its draft's path): a later one with the same key
    // replaces it while it waits for the drain.
    std::string coalesce;
  };
  std::vector<QueuedAction> queued_events;

  // §4 read-lane: the reactive predicate evaluator (lazily built on first use so a UI
  // with no reactive fields pays nothing), plus de-duplicated diagnostics surfaced by
  // take_reactive_warnings(). reactive_warned keeps the dedup set across drains.
  std::unique_ptr<ReactiveEngine> reactive;
  // register_read_function's, bound into the engine when it is made, and into a live one at once.
  std::map<std::string, Runtime::ReadFunction> read_functions;
  double reactive_budget = 0.030; // set_reactive_frame_budget; the engine's default
  std::vector<std::string> reactive_warnings;
  std::set<std::string> reactive_warned;
  int frame_instances = 0; // §3: repeat instances emitted this frame (bounds many repeats)
  // Where the walk is in the tree, as child indices from the root: names an unnamed grid
  // (emit_container), so two in one window never share a backend id.
  std::vector<std::size_t> emit_path;
  // The repeat instances the walk is inside, outermost first (emit, emit_container): with
  // emit_path, one widget instance's identity across frames (instance_key).
  std::vector<int> instance_path;
  // A widget's backend id as an item (begin_item_group): its id:, else its place in the tree.
  std::string item_id(const Widget &w) const {
    if (!w.id.empty())
      return w.id;
    std::string id = "~";
    for (std::size_t i = 0; i < emit_path.size(); ++i)
      id += (i ? "_" : "") + std::to_string(emit_path[i]);
    return id;
  }
  std::string instance_key() const {
    std::string key;
    for (const std::size_t k : emit_path)
      key += std::to_string(k) + ".";
    for (const int i : instance_path)
      key += "#" + std::to_string(i);
    return key;
  }
  // §4.6 on_click handlers and button presses fired so far this walk: a group fires its own
  // on_click only when nothing inside it took the click.
  int clicks_fired = 0;
  // §4.6 on_hover_end: the instances (instance_key) whose item was hovered last walk and this one,
  // with the program to run, and its scope, when the pointer leaves.
  struct HoverEnd {
    std::string handler, prefix, functions;
  };
  std::unordered_map<std::string, HoverEnd> hovered_last, hovered_now;
  // §5.13 the fullscreen view: a picture or player shown alone in place of the tree. Its instance
  // as drawn when it opened, where it was in the tree and its scope (so a player keeps playing,
  // its slot keyed as before), and whether the core made the window fullscreen for it (made_os),
  // which it then saw (saw_os): once the window is no longer, the view closes too.
  struct FullscreenItem {
    bool active = false;
    Widget w;
    std::vector<std::size_t> emit_path;
    std::vector<int> instance_path;
    std::vector<std::string> scope_stack;
    std::shared_ptr<const FunctionScope> functions;
    bool made_os = false, saw_os = false;
  } fullscreen;
  bool in_fullscreen = false;   // drawing the fullscreen view's widget
  bool fullscreen_exit = false; // ... and it asked to leave (its Exit, a click on its picture)
  void open_fullscreen(const Widget &w);
  void close_fullscreen();
  bool emit_fullscreen(); // the view; false when it closed instead (the tree is drawn)
  // `autofocus:` widgets drawn last frame and this one, by tree position (and repeat index): one
  // takes focus when it appears, not on every frame it stays.
  std::set<std::string> autofocus_shown, autofocus_now;
  // §5.9 windows drawn maximized last frame and this one (their tree.<id>.maximized paths): one
  // that becomes maximized comes to the front.
  std::set<std::string> maximized_shown, maximized_now;
  void autofocus(const Widget &inst, int i);
  // §3: last successful repeat-count per expression. A reactive repeat-count eval fail-safes to 0
  // on a transient per-frame budget exhaustion (reactive eval is slow in wasm, and video decode can
  // eat the frame); collapsing a repeated list — the conference roster, the per-peer camera windows
  // — to EMPTY for that frame and back flickers it, and under sustained load leaves an apparently
  // empty conference even though the peers are present in state. Reusing the last-good count
  // bridges those skips so the list stays stable; a genuine count change is picked up on the next
  // unskipped frame.
  std::unordered_map<std::string, int> last_good_count;
  // Kind::TfEditor parse caches, one per editor instance ("<model path>#<id>"): a state string is
  // re-parsed only when it changes.
  std::unordered_map<std::string, TfEditorCache> tf_caches;
  // Kind::Chart parse caches, one per chart instance: a series is re-parsed only when its text
  // changes. Swept of the charts no frame drew for a while (`chart_frame` counts render()s).
  std::unordered_map<std::string, ChartCache> chart_caches;
  std::uint64_t chart_frame = 0;
  // Kind::Heatmap caches, one per heatmap instance (heatmap_emit.h), swept with the charts'.
  std::unordered_map<std::string, HeatmapCache> heatmap_caches;
  // Kind::Settings: one cache per editor instance, rebuilt only after a write under its roots,
  // which `settings_watch` counts from the tree root's childChanged (connected when the first
  // editor is drawn, on whatever thread writes). Swept with the charts' (`chart_frame`).
  std::unordered_map<std::string, SettingsCache> settings_caches;
  std::shared_ptr<SettingsWatch> settings_watch;
  boost::signals2::scoped_connection settings_watch_conn;

  // §5.11 pictures by URI: loaded off the frame by the cache, published to the backend under the
  // cache's names (each once per raster, tracked in `published` against the backend it went to).
  std::unique_ptr<media::ImageCache> images;
  std::string media_base;                          // relative file:// URIs resolve against it
  std::unordered_map<std::string, long> published; // name -> the raster generation published
  Backend *published_to = nullptr;                 // the backend `published` describes
  std::unordered_map<std::string, std::string> media_status_written; // status path -> last value
  bool drew_loading = false;  // the last frame drew a picture still loading (media_stale)
  long frame_publish_gen = 0; // §5.13 a player's frames, as published generations
  media::ImageCache &image_cache() {
    if (!images) {
      images = std::make_unique<media::ImageCache>(app);
      images->set_base(media_base);
    }
    return *images;
  }
  void emit_media_image(const Widget &w, const std::string &uri, const char *label);
  // §5.11 `badge:` on an image: or a video:: its text over the picture just drawn (not in the
  // fullscreen view, where the picture is shown alone).
  void emit_badge(const Widget &w);
  // §5.11: the name of what to draw for a picture request now: its fitted raster (published), the
  // frame of its animation that is due, or the placeholder of its box. `v` gets the cache's view.
  std::string media_picture(const media::ImageRequest &rq, media::ImageView &v);
  // §5.11 `size: "N%"`: a picture's or a player's width in px, that share of the width the backend
  // gives it on its line (at least its own width where the window fits its content), else its
  // own width (img_size).
  int fill_width(const Widget &w);
  // §5.11 a text: with emoji:: its line with the `:name:`s the map names as pictures, or as the
  // text the map gives; where the backend can't put a picture in a line, the text with them as
  // their `:name:`. False when the map names none of them (the caller draws the line as it is).
  bool emit_text_runs(const Widget &w, const char *label, const std::string &value, bool literal);
  // §5.11: `value`'s `:name:`s that the emoji: map at `map` (a resolved path) names, appended to
  // `out` as runs of `style` in link `link`: a picture where `pictures_here` (the backend draws one
  // in a line), else its :name:, and a name the map gives text for as that text. `plain` gets the
  // line with the map's text in it and its pictures as their :name:; `pictures` says whether one
  // went in. False when the map names none of them (nothing is appended).
  bool expand_emoji(const std::string &value, const std::string &map, bool pictures_here,
                    std::vector<TextRun> &out, std::string &plain, bool &pictures, unsigned style,
                    int link);
  // §5.20 a text: with runs:: the paragraph (Backend::rich_text) and the link the user activated,
  // whose handler runs (inner_click: the text's own on_click doesn't, nor a group's around it).
  void emit_rich_text(const Widget &w, const char *label, bool &inner_click);
  std::vector<TextRunSpec> eval_runs(const std::string &expr);
  std::unordered_map<std::string, std::vector<TextRunSpec>> last_good_runs;
  // §5.11: publish a ready picture once per raster generation; keep a status: key current.
  void publish_media(const media::ImageView &v);
  void write_media_status(const std::string &status_path, const media::ImageView &v);
  // §5.12: open a container's background (its picture, loading through the cache) before its
  // content; true when it did, so the caller closes it with backend->end_backdrop().
  bool open_backdrop(const Widget &w, bool window, bool cell = false);

  // §5.13 video: / audio: players, one per widget instance (its bind:, else its place and source),
  // kept while the widget is drawn: paused at once when a walk does not draw it, dropped after 5 s.
  struct PlayerSlot {
    std::string uri, auth;
    std::unique_ptr<media::Player> player;
    media::PlayerState state = media::PlayerState::Idle;
    bool decided = false; // state chosen for this source (a player made, or Remote/Unsupported)
    std::string error;
    std::string frame_name; // the published picture's name
    long frame_gen = 0;
    bool have_frame = false;
    cvc::image pending; // a frame waiting for the terminal's frame-rate cap
    bool has_pending = false;
    double last_publish = 0.0;
    bool autoplay_done = false;
    // An autoplaying player the user (or the document) hasn't played or paused since: it plays
    // while its picture is in view, and `offscreen_paused` says it was paused for leaving it.
    bool autoplay_owned = false, offscreen_paused = false;
    float aspect = 0.0f; // the last frame's height / width (0: none yet)
    std::string wrote_playing, wrote_muted, wrote_state, wrote_duration, wrote_error;
    std::string wrote_fullscreen;
    std::string wrote_loop;   // the Loop box: "1" / "0" (or the document's "true" / "false")
    int box_w = 0, box_h = 0; // the frame box its player decodes for (set_frame_box)
    double wrote_position = -1.0, wrote_position_at = 0.0;
    std::uint64_t drawn_walk = 0;
    double last_drawn = 0.0;
    bool hidden_paused = false;
    double made_at = 0.0; // when its player was made (a video with no frame 3 s on says why)
  };
  std::unordered_map<std::string, PlayerSlot> players;
  std::uint64_t walk_count = 0;
  void emit_player(const Widget &w, const char *label);
  void players_end_walk();
  // The player for `key` playing `ps` (made, or reported unsupported / remote, on first sight or a
  // new source), marked drawn this walk.
  PlayerSlot &player_slot(const std::string &key, const media::PlayerSource &ps);
  // Publish its newest frame fitted to the box (8 a second at most in a terminal); true once one
  // is up.
  bool player_frame(PlayerSlot &p, int bw, int bh, ImageFit fit, ImageShape shape, double now);
  // §5.13 the animated pictures' frame on screen: by published name, the index and when it went.
  struct AnimShown {
    std::size_t index = 0;
    double at = 0.0;
  };
  std::unordered_map<std::string, AnimShown> anim_shown;

  // §4.7 action lane: a unique owner tag for this Runtime's processes on the app-wide
  // scheduler (app.exec_scheduler()), or its own group's, so teardown reaps exactly this
  // document's process group via kill_owner. Counted per app (the app's Runtimes share its
  // scheduler or groups it built), so two Runtimes on the same prefix never collide.
  std::string owner_ = "ari#" + std::to_string(next_runtime_owner_id(app)) + ":" + prefix;

  Impl(cvc::app &a, std::string p) : app(a), prefix(std::move(p)) {}
  ~Impl();

  // A submitted `on:` program's persistent context. The intrinsics capture &ictx by
  // pointer, so — unlike the load-time init: lane's one-scope run — an action that
  // suspends (await/sleep/msg-recv) needs its context kept alive across frames until the
  // process terminates or is reaped. Field order matters for teardown: ictx (which points
  // into tracker/proc) is declared last so it destructs first. Shared, so one type serves its
  // three owners: live_actions_ (an action's), the resident slots (a resident's) and a spawned
  // child's process (process::keep_alive, the child's own).
  struct ActionContext {
    cvc::state_exec::memory_tracker tracker;
    cvc::state_exec::process_ptr proc;
    cvc::state_exec::environment_ptr env;
    cvc::state_exec::intrinsics_context ictx;
    bool ended = false;        // the action's own process has ended and been reaped
    std::vector<int> children; // pids it forked that may still be running (they share ictx)
    std::function<void(const std::string &)> done; // run_program: how its own process ended
    std::string script;                            // the program's text, to say which one failed
    std::string scope; // the prefix it runs at: a quit application's are stopped (§5.25)
    std::shared_ptr<const NotifyScope> notify; // its notify verbs', which its children bind too
    // The frame budget of the mounts: entry its scope is under (ARIADNE_SHELL.md 4.11), or null.
    std::shared_ptr<cvc::state_exec::frame_account> account;
    // The grant tokens the mount its scope is under holds (ARIADNE_SHELL.md 4.5), which its has?
    // answers from, taken on the host thread; null: the document's own scope. Its children's too.
    std::shared_ptr<const std::set<std::string>> held;
  };
  std::unordered_map<int, std::shared_ptr<ActionContext>> live_actions_;
  // A program's process carries its context's chroot, as a spawned child's does, and this
  // document's prefix as its owner's root: `(ps)` names whose it is from the two ("" for the
  // document's own, "includes.<as>" for a mount's; docs/roadmap/ARIADNE_SHELL.md 4.10), and its
  // async calls post their progress under its chroot (docs/STATE_EXEC_EXCEPTIONS.md 3.10).
  void own_scope(cvc::state_exec::execute_options &opts, const ActionContext &ac) const {
    opts.root_path = ac.ictx.root_path;
    opts.owner_root = prefix;
    opts.account = ac.account; // a mount's budget (ARIADNE_SHELL.md 4.11), its children keep it
  }
  // Runtime::start_process while the sweep walks live_actions_ (a done that starts the next one):
  // joined to them once the walk is over.
  bool sweeping_actions_ = false;
  std::vector<std::pair<int, std::shared_ptr<ActionContext>>> started_in_sweep_;
  // Pids of the children actions and residents spawned (spawn_child), reaped once they finish.
  std::vector<int> spawned_children_;
  bool used_scheduler_ = false; // did we ever submit to the app scheduler? (gate teardown reap)

  // ---- isolated documents (threaded Stage 1, S1.10, S1-Q6) ----
  // Where the document's programs run is decided at load (ensure_placement, again after each
  // set_root; AppRuntime::load asks at its end): in group_, an exec_group of its own (owner
  // owner_, a frame group of the app), when ariadne.exec.isolate_documents is on and the document
  // is eligible; else in group 0, the app scheduler. Every process the Runtime starts, every post
  // it makes and every kill go there. Declared after channel_policy_, doc_scope_ and the contexts,
  // which the group's processes point at; ~Impl destroys it first anyway, after the kills and the
  // reaps.
  std::unique_ptr<cvc::state_exec::exec_group> group_;
  cvc::frame_group_handle frame_handle_; // group_'s registration; goes before group_
  bool document_set_ = false;            // set_root has given it a document
  bool placement_decided_ = false;       // for the document set_root last gave
  std::string isolation_reason_ = "no document has been set yet";
  std::string logged_reason_; // the last reason decided with a document set (the log's)
  std::vector<std::string> thread_affine_;     // declare_thread_affine_prefix
  std::string host_thread_why_;                // require_host_thread: "" until declared
  std::vector<std::string> document_programs_; // set_document_programs: init:, tests:
  // Decide where the programs run: when `submitting` (a program is about to start) or a document
  // has been set, unless that document's placement is decided already. Host thread.
  void ensure_placement(bool submitting);
  // A placement change (ensure_placement) moves the document: move_out kills and reaps its
  // processes where they ran and drops their records, so its residents start again in the new
  // place; the streams it owns are re-pointed there (retarget_document_streams). Its old group, if
  // any, is retired: unregistered (as a frame group, and from the router: retire_from_router) and
  // unpumped, it keeps the contexts taken from it and is destroyed at a later drain_begin once its
  // host has no kernel in flight (R1b skeptic m1, m5), so a move never waits for them. Only its own
  // kernels' completions can post to it then: its processes are gone, no stream names it, the
  // Runtime posts to the new place and its host-call queue is empty outside a pump. The contexts
  // taken from group 0 stay until ~Impl, for a process a failed kill left there.
  struct RetiredGroup {
    std::unique_ptr<cvc::state_exec::exec_group> group;
    std::vector<std::shared_ptr<ActionContext>> contexts; // go after the group
  };
  std::vector<RetiredGroup> retired_groups_;
  std::vector<std::shared_ptr<ActionContext>> retired_contexts_;
  // Kill and reap what the document runs where it runs, drop its records, and hand back the
  // contexts taken from them.
  std::vector<std::shared_ptr<ActionContext>> move_out();
  // drain_begin: destroy the retired groups whose hosts have no kernel in flight.
  void destroy_idle_retired_groups();
  // A group the document leaves leaves the app's router at once (S2b), not at its teardown, which
  // waits for its kernels: its prefix is unregistered and what it holds for the router (pending
  // messages, waits of other groups, router entries) goes to the keys' new homes. Called once the
  // document's new group, if any, has registered the prefix. Logs what it throws.
  void retire_from_router(cvc::state_exec::exec_group &g) noexcept;
  // Late registrations (R1b skeptic m3, the lead's rule): a registration after the placement
  // (refresh_program_names, or the providers' registry moving past placement_generation_) has it
  // decided again only when a move would kill nothing; else it waits for the document's next
  // restart (set_root, set_*_program, a load), noted once.
  std::uint64_t placement_generation_ = 0;  // the providers' registry when it was decided
  bool registered_since_placement_ = false; // refresh_program_names since then
  bool late_noted_ = false;                 // the wait is logged once per placement
  // Nothing of the document runs where it runs: no live action, started resident or mount tick,
  // nor spawned child.
  bool nothing_running() const;
  // Kill every process of the owner on `sched` and reap each one the Runtime knows of, each step
  // catching and logging what it throws (`when`: "tearing down", "moving").
  void kill_and_reap(cvc::state_exec::async_scheduler &sched, const char *when) noexcept;
  // "" when the document may have a group (S1-Q6, S2-Q16), else the first rule it fails;
  // `host_affinity` tells whether that group must be pumped on the host thread.
  std::string isolation_verdict(bool &host_affinity) const;
  // Whether a host_only provider binds anything in an action's env (the group is then host).
  bool host_only_bindings() const;
  // Build a context like an action's, chrooted at the prefix and never run, and call `fill` with
  // it and its env (with the intrinsics): load_names' and host_only_bindings' probes.
  void probe_env(const std::function<void(const cvc::state_exec::environment_ptr &,
                                          cvc::state_exec::intrinsics_context &)> &fill) const;
  // The scheduler the document's processes run on now: its group's, or group 0's.
  cvc::state_exec::async_scheduler &current_sched();
  // has? in an `any` group, which a worker may pump: the facts and names drain_begin snapshotted
  // on the host thread (S1-Q7), read under names_mutex.
  HostFacts facts_snapshot_;
  void snapshot_has();
  bool has_snapshot(const std::string &name) const;
  // has? for a program in `group` (null: group 0): live on the host thread, else the snapshot. A
  // grant token from `held`, what the program's mount holds, taken on the host thread when its
  // context was made (null: the document's own scope, which holds them as it would alone).
  std::function<bool(const std::string &)>
  program_has(cvc::state_exec::exec_group *group,
              std::shared_ptr<const std::set<std::string>> held);
  // The drain's halves (Runtime::drain_begin/drain_end).
  std::uint64_t begin_generation_ = 0; // app.exec_pump_generation() at drain_begin
  bool drain_begun_ = false;
  // Every process of the owner the Runtime knows of: its actions, their forks, its spawned
  // children and its residents.
  std::vector<int> own_pids() const;
  // Teardown and moves, each step catching and logging what it throws.
  void teardown_note(const char *when, const char *step, const char *what) noexcept;

  // The context an action, a resident or a child of one runs in: the document's scheduler and
  // host (its group's, else group 0's), a placeholder proc, writes chrooted to `scope_prefix`,
  // the document's channel policy and resource scope, the intrinsics, the host's action intrinsic
  // providers (by kind) and has?, and the with-lock default (nullopt: a request with no :timeout
  // waits as long as it takes).
  std::shared_ptr<ActionContext> make_action_context(const std::string &scope_prefix,
                                                     std::optional<double> lock_default_timeout);
  // A context's env: the intrinsics, the providers by kind for `group` (null: group 0) and has?.
  void finish_context(ActionContext &ac, cvc::state_exec::exec_group *group);
  // The spawn_context of an action's or a resident's context (docs/STATE_EXEC_THREADED.md 4.15): a
  // child runs in a context of its own, made from its parent's: its scheduler and host, its
  // chroot as the parent holds it (a child of a deleted chroot writes the parent's detached node),
  // its lock fields, so a child of an action gives up after the action's default, and its spawn
  // context. It runs on the pumping thread (a worker in an `any` group). The child's process
  // keeps it alive, and the child keeps the document's owner (intrinsic_spawn), so ~Impl's
  // kill_owner reaps it. `notify` is its parent's notify scope, taken on the host thread, and
  // `held` its parent's grants (ActionContext::held).
  cvc::state_exec::spawn_setup spawn_child(const cvc::state_exec::intrinsics_context &parent,
                                           std::shared_ptr<const NotifyScope> notify,
                                           std::shared_ptr<const std::set<std::string>> held);

  // Submit a program `on:` as a process on the document's scheduler (owner-tagged, chrooted
  // to `prefix`), keeping its ActionContext alive in live_actions_. Returns immediately.
  void submit_action(const QueuedAction &action);
  // Group 0 when drain_end pumps it, or nullptr to skip: when this Runtime has submitted to it or
  // any process is live on it (whoever submitted it). Never builds it.
  cvc::state_exec::async_scheduler *scheduler_to_pump();
  cvc::state_exec::async_scheduler *seen_scheduler_ = nullptr; // the app's, once something built it
  // Whether this pump starts the next app frame (exec_pump_clock), advancing the shared clock.
  bool starts_frame();
  std::shared_ptr<exec_pump_clock> pump_clock_; // shared by the app's Runtimes; got at first pump
  std::uint64_t pumped_at_ = 0; // the clock's frame count at this Runtime's last pump
  // drain_end's pump, when no exec_pump_frame() ran since drain_begin: group 0 a bounded slice (as
  // scheduler_to_pump says), and the document's group through the app's pool.
  void pump_frame();
  // Sweep finished actions out of live_actions_ (reporting a killed one; a still-parked one
  // persists to a later frame), and the residents and children.
  void sweep_actions(cvc::state_exec::async_scheduler &sched);
  // sweep_actions where the document's processes run: its group's scheduler, else group 0's once
  // it has used it (nothing to sweep before).
  void sweep_document();

  // §7.1 resident on:tick handler. tick_script_ is the author's per-frame body (empty = none).
  // ensure_tick_resident() submits ONE long-lived process (idempotent) that loops
  // (while t (begin (msg-recv "<owner>#tick") <body>)) — parking between frames; post_tick()
  // wakes it once per drain. Reaped by ~Impl's kill_owner (owner-tagged) or set_tick_program.
  std::string tick_script_;    // §7.1 on:tick body (woken every frame, coalesced)
  std::string key_script_;     // §4.6 document-level on_key body (keyboard events, via post_input)
  std::string pointer_script_; // §4.6 document-level on_pointer body (mouse events, via post_input)
  std::string signal_script_;  // document-level on_signal body (signals, via post_signal)
  // A resident's process and context, and how it has been restarted. A resident that is killed
  // (its per-step cap, a syntax error read inside it) or fails (an exception it didn't catch) is
  // reported once per reason, then restarted on a later drain: at once the first time, then after
  // 1, 2, 4 ... up to kRestartMaxDelay drains while it keeps stopping within kQuickStop drains of
  // starting. After kMaxQuickStops such stops in a row it is given up, with a warning, until its
  // program is set again.
  struct Resident {
    std::shared_ptr<ActionContext> ctx;
    int pid = -1;
    std::uint64_t started = 0;  // the drain it last started on
    int quick_stops = 0;        // stops soon after starting, in a row
    std::uint64_t retry_at = 0; // not restarted before this drain
    bool given_up = false;
  };
  static constexpr std::uint64_t kQuickStop = 100;
  static constexpr int kMaxQuickStops = 8;
  static constexpr std::uint64_t kRestartMaxDelay = 64;
  std::uint64_t drains_ = 0; // drain() calls so far: the restart clock
  Resident tick_resident_, key_resident_, pointer_resident_, signal_resident_;
  // Stop a resident now (a new program, a dropped mount) and forget its restart history.
  void stop_resident(Resident &r);
  // A resident that was killed or failed: report it, reap it, and schedule its restart (or give
  // it up). A no-op for one that is running or parked.
  void retire_if_stopped(cvc::state_exec::async_scheduler &sched, Resident &r,
                         const std::string &lane, const std::string &channel);
  std::string tick_channel() const { return owner_ + "#tick"; }
  std::string key_channel() const { return owner_ + "#key"; }
  std::string pointer_channel() const { return owner_ + "#pointer"; }
  std::string signal_channel() const { return owner_ + "#signal"; }
  // Submit ONE long-lived resident wrapping `body` on `channel`, storing its context/pid in the
  // slots (idempotent — no-op if body is empty or a resident is already running). The wrapper is
  // (while t (let ((event (msg-recv "<channel>"))) <body>)): msg-recv parks the process between
  // deliveries and binds each delivered value as `event`, so an on_key/on_pointer body reads
  // (get-attr event "key") etc. (on:tick's body simply ignores `event`).
  // Not before its restart drain, nor once it has been given up.
  void ensure_resident(const std::string &channel, const std::string &body, Resident &r);
  // The same, chrooted to `chroot` with `functions` (a FunctionScope prelude) defined first: a
  // mount's tick resident. The three-argument form is the document's (prefix, doc_functions).
  void ensure_resident(const std::string &channel, const std::string &body, Resident &r,
                       const std::string &chroot, const std::string &functions);
  void ensure_tick_resident();   // submit the tick resident (woken by the coalesced post_tick)
  void post_tick();              // wake the tick resident once this drain
  void ensure_input_residents(); // submit the key + pointer residents (woken by post_input)
  // Deliver one input event: serialize it to a dict and post_message it (NO coalesce — a burst in
  // one frame must all arrive) onto the key channel (keyboard) or pointer channel (mouse), and
  // queue it for the tap when it takes that kind.
  void deliver_input(const InputEvent &ev);
  // set_event_tap: the kinds it takes (read by post_input and post_signal, on the host thread),
  // and the events queued for it until the next drain hands them over (deliver_tapped).
  std::atomic<unsigned> tap_kinds_{0};
  Runtime::EventTap tap_;
  std::mutex tapped_mutex_;
  std::vector<std::pair<std::string, cvc::state_exec::value_t>> tapped_;
  void tap(const char *kind, cvc::state_exec::value_t event) {
    std::lock_guard<std::mutex> lock(tapped_mutex_);
    tapped_.emplace_back(kind, std::move(event));
  }
  void deliver_tapped();

  // set_key_bindings. post_input (any thread) queues key downs; drain() (the host thread) rebinds
  // or runs them, so a key's action is queued like a button's.
  std::vector<KeyBinding> key_bindings_;
  std::atomic<bool> have_key_bindings_{false};
  std::mutex pressed_mutex_;
  std::vector<std::string> pressed_keys_;
  std::vector<std::string> pressed_pads_; // post_pad_press, for a gamepad Change
  // The Keys window waits for a key: a Change button named a binding, and the window is open (a
  // capture left over in a closed window is cleared by the next drain).
  bool key_capture_pending() const {
    return !read_or<std::string>(app, resolve_bind(prefix, kKeyCapture), "").empty() &&
           read_bool_or(app, resolve_bind(prefix, kKeysWindowOpen), 0) != 0;
  }
  void run_key_bindings();

  // forget_layout(): applied at the next render. Each window's and grid's first frame after it
  // (by its tree.<id>.geometry / .tracks / .columns path) seeds only what the document authored.
  bool forget_layout_pending = false;
  // The frame clock (Runtime::frame_clock): seconds since clock_epoch, read at each render(), or
  // the host's pinned time (set_frame_clock).
  std::chrono::steady_clock::time_point clock_epoch = std::chrono::steady_clock::now();
  std::optional<double> pinned_clock;
  double frame_t = 0.0;
  // §5.22 the tray's clock: the host's default (Runtime::set_clock), the document's `clock:`
  // (WindowsMenuPlan::clock_setting), and the time it reads (set_wall_clock; empty: the system
  // clock).
  bool clock_default = false;
  std::function<std::chrono::system_clock::time_point()> wall_clock;
  bool notify_default = true; // §5.24 set_notifications
  // §5.24 each mount's composed prefix and its name relative to the document's (its `as:`), as the
  // walk last met it: what the feed's entries are named by.
  std::map<std::string, std::string> mount_names;
  std::map<std::string, std::string> mount_functions; // each mount's functions: (prelude)
  std::string notify_source(const std::string &path) const;
  NotifyScope notify_scope(const std::string &scope) const; // on the host thread
  // §5.24 the Notifications window ("Show all..." in the bell's menu), rebuilt when the feed
  // changes, and the built-in events the bell and it raise (handled by drain, not a document).
  static constexpr const char *kNotifyEvent = "\x1F"
                                              "notify.";
  static constexpr const char *kNotifyWindowId = "_notifications";
  static constexpr const char *kBudgetWindowId = "_budgets"; // ARIADNE_SHELL.md 4.11
  Widget notify_window;
  Widget budget_window; // ARIADNE_SHELL.md 4.11 the Budgets window, made each frame it shows
  std::uint64_t notify_window_rev = ~std::uint64_t(0);
  void emit_notify_window();
  bool builtin_event(const QueuedAction &a);
  void bind_notify_verbs(const std::shared_ptr<cvc::state_exec::environment> &env,
                         const NotifyScope &ns) const;
  bool windows_forgotten = false;
  std::set<std::string> laid_out;

  // §12: the active mount scope during emit. Empty stack = the document prefix; each entry is
  // a composed sub-prefix pushed when the walk enters a mounted subtree (Widget::scope) and
  // popped on exit. current_prefix() is what binds and read-lane reads resolve against.
  std::vector<std::string> scope_stack;
  const std::string &current_prefix() const {
    return scope_stack.empty() ? prefix : scope_stack.back();
  }
  // §6.6 the function scopes the walk is inside (the root's, then each mount's), parallel to
  // scope_stack, and the document's own (the root's, from set_root) for the programs no widget
  // starts: residents, key bindings, run_program. A program runs its scope's prelude first.
  std::vector<const FunctionScope *> fn_stack;
  std::string doc_functions;
  const std::string &current_functions() const {
    return fn_stack.empty() ? doc_functions : fn_stack.back()->prelude;
  }

  // §12 load: sub-prefixes whose parent-scope holes (link:) have been planted, so we wire each
  // mount's link nodes ONCE (idempotent, but findDescendant+linkTo per hole per frame is waste).
  // Cleared on a reconcile (the swapped-in tree may mount different fragments).
  std::set<std::string> wired_mounts;
  // §12 load: the state paths of every hole planted for the CURRENT tree. On a reconcile these
  // are torn down (clearLink) BEFORE the new tree re-wires — otherwise a fragment re-mounted at
  // the same sub-prefix (same `as`) would inherit the previous tree's grants it was never given,
  // defeating default-deny (a hole node persists in cvc::state; clearing wired_mounts alone only
  // re-plants, never removes).
  std::vector<std::string> planted_holes;
  // Plant the transparent link-node holes for a mount: for each, a node at <sub_prefix>.<name>
  // links to `target` resolved against the mount's PARENT scope (leading '/' = app root).
  void wire_holes(const std::string &parent_prefix, const std::string &sub_prefix,
                  const std::vector<LinkHole> &links);

  // §12 load: sub-prefixes whose mounted fragment's init: has already run. Init is a ONE-SHOT
  // seed, so unlike wired_mounts this is NOT cleared on a reconcile — the fragment's init runs
  // once when the mount first appears, never re-running when the tree is rebuilt.
  std::set<std::string> ran_inits;
  // Run a mounted fragment's init: once at its sub-prefix (chrooted there by run_init); any error
  // is surfaced as a one-time runtime warning rather than thrown into the frame.
  void run_mount_init(const std::string &sub_prefix, const std::string &script);

  // §12 load: a mounted fragment's on_tick: — one resident per mount sub-prefix, chrooted there
  // with the mount's functions, on its own channel, woken by post_tick like the document's. Started
  // as the mount is first drawn (after its init:), so a module in a closed tab starts when the tab
  // opens; it then runs whether or not the mount is drawn, until a reconcile drops it
  // (drop_mount_ticks) or the Runtime goes (kill_owner: it carries owner_). A repeated mount's
  // instance also stops when its repeat's count drops below it (sweep_repeated_ticks).
  struct MountTick {
    std::string body;      // the on_tick: body
    std::string functions; // and the mount's prelude, defined before its loop
    std::string channel;
    Resident resident;
    const Widget *repeat = nullptr; // the outermost repeat: template it was emitted under, if any
    int instance = -1;              // and that repeat's instance index
  };
  std::map<std::string, MountTick> mount_ticks_; // by sub-prefix
  // The outermost repeat: template being emitted and the instance index (a template in `root`,
  // whose address holds until the next reconcile; an inner one is copied per frame).
  const Widget *emitting_repeat_ = nullptr;
  int emitting_instance_ = -1;
  // §12 a repeated mount's instances, made once from its template (in `root`, whose address holds
  // until the next reconcile, which clears them) rather than copied each frame: an instance is a
  // whole loaded document, and its copy each frame cost as much as drawing it.
  std::map<std::pair<const Widget *, int>, Widget> mount_instances_;
  // §5.25 a mount's `name:`, worked out in the current scope: a key's value, or an expression's;
  // "" when it has none or it comes out empty.
  std::string mount_name_of(const std::string &spec) {
    if (spec.empty())
      return {};
    return is_expr(spec) ? eval_text(spec) : read_string(app, resolve(spec));
  }
  void ensure_mount_tick(const std::string &sub_prefix, const Widget &w);
  // On a reconcile: stop every mount tick the new tree does not have at the same sub-prefix with
  // the same program (a repeated mount's restart at their next frame).
  void drop_mount_ticks();
  // A repeat of `tmpl` made `count` instances this frame: stop the ticks of the ones past it.
  void sweep_repeated_ticks(const Widget *tmpl, int count);

  // Resolve a widget bind path to an absolute cvc::state path. Delegates to the
  // shared rule (bind.h) so widget binds and scene `visible:` binds collide on the
  // same key for the same relative path — against the active mount scope (§12).
  std::string resolve(const std::string &bind) const {
    return resolve_bind(current_prefix(), bind);
  }

  // Lazily build the read-lane engine (a UI with no reactive fields pays nothing), then point
  // it at the active mount scope so a mounted fragment's predicates read its own sub-prefix,
  // matching where its binds resolve. Every reactive helper goes through here.
  ReactiveEngine &ensure_reactive() {
    if (!reactive) {
      // A grant token holds per mount: the walk's scope is the one the engine reads.
      reactive = std::make_unique<ReactiveEngine>(app, prefix, [this](const std::string &n) {
        return is_grant_token(n) ? grant_holds(current_prefix(), n) : has(n);
      });
      for (const auto &[name, fn] : read_functions)
        reactive->bind_read_function(name, fn);
      reactive->begin_frame(reactive_budget); // made mid-frame: this frame's budget is the host's
    }
    reactive->set_root_path(app, current_prefix());
    reactive->set_event(menu_event ? &menu_event->dict : nullptr); // §5.30 inside a menu's items
    return *reactive;
  }

  void enqueue(const std::string &event) {
    if (!event.empty()) {
      queued_events.push_back({event, current_prefix()}); // capture the mount scope for a program
      queued_events.back().functions = current_functions(); // and its functions
      if (menu_event) // §5.30 a right-click menu's item: its program gets the menu's `event`
        queued_events.back().event_values = menu_event->fields;
    }
  }

  // §5.30 right-click menus. Each open menu's `event`, taken when it opened and kept while it
  // stays open, by the menu's instance key, with the walk that last drew it (one no walk drew is
  // closed, and goes); the menu whose items the walk is emitting, whose event their visible_when:
  // reads and the programs they run get; the host's pick in the main view (set_main_view_pick).
  struct MenuEvent {
    std::vector<std::pair<std::string, cvc::state_exec::value_t>> fields;
    cvc::state_exec::value_t dict;
    std::uint64_t walk = 0;
  };
  std::unordered_map<std::string, MenuEvent> menu_events; // nodes stay put (menu_event points in)
  const MenuEvent *menu_event = nullptr;
  std::uint64_t walk_no = 0;
  Runtime::ContextPick main_view_pick;
  MenuEvent context_event(const Widget &w, ContextMenuOver over, const ContextMenuOpen &o,
                          const Runtime::ContextPick &pick) const;
  void emit_context_menu(const Widget &w, ContextMenuOver over, const Runtime::ContextPick &pick);
  std::string context_id(const Widget &w) const { return "ctx:" + item_id(w); }
  // §4.6: enqueue a widget POINTER handler carrying its event data (a program gets an `event`
  // dict).
  void enqueue_event(const std::string &event, float x, float y, float dx, float dy, int button) {
    if (!event.empty()) {
      queued_events.push_back({event, current_prefix(), true, x, y, dx, dy, button});
      queued_events.back().functions = current_functions();
    }
  }
  // As enqueue_fields, but only the latest for `key` (a message field's draft path) waits for the
  // drain (on_change: a burst of keys between two drains runs it once, with the draft as it is).
  void enqueue_latest(const std::string &key, const std::string &event,
                      std::vector<std::pair<std::string, std::string>> fields) {
    if (event.empty())
      return;
    for (QueuedAction &q : queued_events)
      if (q.coalesce == key && q.event == event) {
        q.event_fields = std::move(fields);
        return;
      }
    enqueue_fields(event, std::move(fields));
    queued_events.back().coalesce = key;
  }
  // Enqueue a heatmap's handler: its program gets the cell as `event` (row, col and v as numbers,
  // v nil for a cell with no value; text, row_label and col_label; mark 1 or 0).
  void enqueue_cell(const std::string &event, const HeatmapCellEvent &cell) {
    if (event.empty())
      return;
    namespace se = cvc::state_exec;
    enqueue_fields(
        event, {{"text", cell.text}, {"row_label", cell.row_label}, {"col_label", cell.col_label}});
    std::vector<std::pair<std::string, se::value_t>> &v = queued_events.back().event_values;
    v.emplace_back("row", se::value_t(static_cast<int64_t>(cell.row)));
    v.emplace_back("col", se::value_t(static_cast<int64_t>(cell.col)));
    v.emplace_back("v", cell.has_value ? se::value_t(cell.v) : se::value_t());
    v.emplace_back("mark", se::value_t(static_cast<int64_t>(cell.mark ? 1 : 0)));
  }
  // Enqueue a handler whose program gets an `event` dict of FIELDS (strings).
  void enqueue_fields(const std::string &event,
                      std::vector<std::pair<std::string, std::string>> fields) {
    if (!event.empty()) {
      queued_events.push_back({event, current_prefix()});
      queued_events.back().functions = current_functions();
      queued_events.back().event_fields = std::move(fields);
    }
  }
  // As enqueue_fields, in the document's own scope (its prefix and functions) wherever the walk
  // is, as the programs no widget owns run (a scene node's on_pick:, on_key:).
  void enqueue_document_fields(const std::string &event,
                               std::vector<std::pair<std::string, std::string>> fields) {
    if (!event.empty()) {
      queued_events.push_back({event, prefix});
      queued_events.back().functions = doc_functions;
      queued_events.back().event_fields = std::move(fields);
    }
  }

  // The walk. Nothing in it recurses: a container's children, and what the container does after
  // them (its end_*s, pop_ids and scope pops, in the order a recursive walk did them), go on walk_,
  // a work stack on the heap, and run_walk steps it until it is back where it started. So a
  // document nested any depth draws in the same native stack (ariadne_deep_nesting_test runs it on
  // a 256 KiB one). The begin_* and emit_* below that take a widget start its work and return: what
  // is left of it is on walk_, for the run_walk they were called under.
  struct WalkFrame {
    enum class Op : std::uint8_t {
      Children,    // emit_children: `w`'s children (context_items, when `items`) from `next`
      Cells,       // a grid's children from `next`, each instance a cell (emit_container)
      Instances,   // a repeat's instances from `next` (begin_repeat), as cells when `cell`
      Container,   // emit_container after the children: end_grid and its writes, the scope pops
      Node,        // emit_node after a container's children, by w->kind and `stage`
      ContextMenu, // emit_context_menu after its items
      Then,        // `then`, run once the work above it is done
    };
    Op op = Op::Then;
    std::uint8_t stage = 0;
    bool items = false;     // Children: the context menu's items
    bool cell = false;      // Instances: as a grid's cells
    bool outermost = false; // Instances: the outermost repeat (emitting_repeat_ is its template)
    bool open_item = false; // Instances: an instance's id and instance_path entry are pushed
    bool scoped = false;    // Container: a mount pushed its scope
    bool fns = false;       // Container: and its functions
    bool grid = false;      // Container: begin_grid opened one
    bool dis = false;       // Node: begin_disabled is open
    bool flag = false;      // Node: Window open, Menu merges the Windows menu, Group inner click
    bool backdrop = false;  // Node: a backdrop is open (Window, Group)
    bool item = false;      // Node(Group): an item group is open
    std::size_t next = 0;   // Children, Cells, Instances: the next index
    int count = 0;          // Instances: how many; Node(Group): clicks_fired before its children
    const Widget *w = nullptr;
    std::unique_ptr<std::optional<Widget>> inst; // Instances: the instance drawn now
    std::string path, path2;          // Container: tracks, columns; Node(Window): its open path
    const MenuEvent *outer = nullptr; // ContextMenu: the menu event it took the place of
    std::function<void()> then;
  };
  std::vector<WalkFrame> walk_;
  void push_then(std::function<void()> then);
  template <class Start> void run_walk(Start &&start);
  void step_walk();
  void leave_node(WalkFrame &f);
  void emit_node_tail(const Widget &w, bool dis, bool inner_click);

  void emit(const Widget &w);       // repeat + visibility gates (§4), then emit_node (runs a walk)
  void begin_emit(const Widget &w); // as emit, on the walk running now
  void emit_node(const Widget &w);  // draw the widget assuming it is visible (on the walk)
  // §3 repetition: each instance of `w`, once (i == -1) for a plain widget, or N times (i=0..N-1,
  // `{i}` substituted) for a `repeat` template: drawn by begin_emit, or as a grid's cells.
  void begin_repeat(const Widget &w, bool cell);
  void begin_cells(const Widget &c); // a grid's child: its instances, each in a cell
  // Out of emit_node, so its frame (the stack one step of the walk takes) stays small:
  void emit_mounts(const Widget &w);      // ARIADNE_SHELL.md 4.2 a mounts: slot's mounts
  bool menu_item_action(const Widget &w); // a menu item, its label an expression or not
  void emit_children(const Widget &w);
  void emit_container(const Widget &w); // lay children out per w.layout (§3.0.3b)
  void render();

  // §5.9 the built-in Windows menu: a checkable item per window, bound to its tree.<id>.open, so
  // a closed window can be opened again. Worked out once per tree (plan_windows_menu, when a new
  // root is swapped in) and drawn at most once a frame.
  struct WindowsMenuPlan {
    bool on = true; // a `windows_menu: false` turns it off
    std::string title = "Windows";
    bool placed = false; // the document's menubar places it itself (a windows_menu entry)
    bool has_menubar = false;
    bool any_closable = false;
    // The menubar already has a menu of this title (fem_gallery.ari's own "Windows"): the items go
    // at the end of that one, after a separator, rather than in a second menu of the same name.
    bool merged = false;
    // The windows no menu item of the document already toggles: title, resolved open-state and
    // raise paths.
    struct Item {
      std::string title, open_path, raise_path;
    };
    std::vector<Item> items;
    // The windows a `repeat:` makes (conference_roster's per-peer cameras): their template in
    // `root` and the scope it resolves in. How many there are changes, so each frame's menu lists
    // that frame's instances, each under its own title.
    std::vector<std::pair<const Widget *, std::string>> repeated;
    std::set<std::string> toggled; // the open-state paths a menu item of the document toggles
    // §5.22 the tray's clock: the document's `clock:` (-1 none, 0 off, 1 on), and whether a
    // `- clock` entry in a menubar places it (else it goes last, at the bar's right end).
    int clock_setting = -1;
    bool clock_placed = false;
    // §5.24 the tray's bell: the document's `notifications:` and whether `- bell` places it.
    int bell_setting = -1;
    bool bell_placed = false;
    bool bell_always = false; // `notifications: always`: shown with an empty feed too
    // §5.25 the loaded documents with a menubar of their own, in tree order: each is one menu of
    // the outer bar, its bar is not drawn where it is, and its tray icons join the outer tray.
    struct App {
      const Widget *mount = nullptr;
      const Widget *bar = nullptr;
      std::string name;  // its meta.name, else its mount id
      std::string scope; // its composed prefix
      const FunctionScope *functions = nullptr;
      // A repeated mount (`repeat:`): its template, and the scope the template sits in. Its
      // instances are worked out each frame (plan_frame_apps), one application each.
      const Widget *repeat = nullptr;
      std::string parent_scope;
      // The load: node's `name:` (Widget::app_name_bind), worked out each frame in parent_scope:
      // when not empty, the application's name (an instance's, `{i}` filled in).
      std::string name_bind;
    };
    std::vector<App> apps;
  };
  WindowsMenuPlan windows_menu;
  int menubar_depth = 0;           // > 0 while a menubar's items are being drawn
  int menu_depth = 0;              // > 0 while a menu's items are being drawn
  bool windows_menu_drawn = false; // this frame
  // §5.25 the applications (loaded documents with a menubar): drawn once a frame, in the first bar
  // that is the document's own; the ones the user quit (by prefix), kept for the run.
  bool apps_drawn = false;
  // The first of the document's bars this frame has been drawn: the clock and the bell were its.
  bool first_bar_drawn = false;
  // This frame's applications: the plan's, with each repeated one expanded into its instances
  // (their bars per-instance copies, kept here for the frame).
  std::vector<WindowsMenuPlan::App> frame_apps;
  std::deque<Widget> frame_app_bars;
  void plan_frame_apps();
  bool is_app_scope(const std::string &scope) const;
  std::set<std::string> quit_apps;
  int app_menu_depth = 0; // > 0 while an application's menu is being drawn
  void emit_app_menus();
  void app_menu(std::size_t k); // frame_apps[k]'s menu (the next running one from k), then the rest
  void app_commands(const WindowsMenuPlan::App &a); // its menu's Restart, Quit (Close), its end
  void quit_app(const std::string &scope);
  void restart_app(const std::string &scope);
  bool app_running(const std::string &scope) const;
  // The applications mirrored into the state tree under the document's prefix, as the feed
  // mirrors itself to notify.* (docs/roadmap/ARIADNE_SHELL.md 4.9): ariadne.apps.<key>.{name,
  // scope, running}, and ariadne.name, the document's own meta.name, for a Processes view to read.
  // What was last written, so a frame writes only what changed.
  struct MirroredApp {
    std::string name, scope, principal;
    bool running = true;
  };
  std::map<std::string, MirroredApp> mirrored_apps;
  std::string mirrored_name;
  void mirror_apps();

  // ---- on-demand mounts (docs/roadmap/ARIADNE_SHELL.md 4.2, 4.11; docs/ARIADNE.md §6.8) ----
  // The host's loader (AppRuntime's, which keeps the loader itself free of state): a mounts:
  // entry's document, parsed as a load: of it would be, its requires: checked against this host.
  Runtime::MountLoader mount_loader_;
  // One child of a mounts: slot's key, by its sub-prefix (<the slot's scope>.includes.<as>).
  struct MountSlot {
    std::string slot_key;     // the slot's state key, resolved: the entry is <slot_key>.<as>
    std::string parent_scope; // the scope the slot sits in
    std::string as;
    std::string uri; // as the entry gives it
    std::vector<std::pair<std::string, std::string>> args;
    std::string canonical;           // where its bytes came from
    std::string principal;           // local, file:<dir> or remote:<origin> (4.12)
    std::string name;                // its application name
    std::unique_ptr<Widget> wrapper; // the mount, as expand_load builds a load:'s; null: refused
    std::string refused;             // why it is not mounted ("" when it is)
    std::vector<std::string> sources;
    std::map<std::string, std::int64_t> stamps;
    bool stale = false; // a source changed: parse it again
    std::shared_ptr<cvc::state_exec::frame_account> account;
    Runtime::MountBudget budget;
    double held_since = -1.0; // the frame clock since which it has been held every frame; -1 not
    bool noticed = false;     // the notice for this stretch at its limit was posted
    bool by_memory = false;   // its memory, not its time, is what holds it
    // ARIADNE_SHELL.md 4.5, 4.6 the entry's own grants (picked from the slot's; has_grants: it
    // names any) and argv (has_argv: it has one), as the entry gave them when it opened.
    bool has_grants = false;
    std::vector<std::string> grants;
    bool has_argv = false;
    std::vector<std::string> argv;
    // 4.12 the network origins it holds beyond its principal's (SH11's net_origins).
    std::vector<std::string> net_origins;
    std::vector<std::pair<std::string, std::string>> tests; // its tests: (ari test --mounted)
    std::vector<std::string> warnings;                      // what its last parse warned
  };
  std::map<std::string, MountSlot> mount_slots_;
  // The slots in the tree, each with the scope it sits in, as plan_windows_menu last walked it
  // (into the open mounts too, for theirs).
  std::vector<std::pair<const Widget *, std::string>> mounts_widgets_;
  // The document's own first slot's key, which (app-open)/(app-close) write: set with the tree
  // (set_root) and by each reconcile, read by the verbs when they are called, on any thread.
  // Shared, so a program that outlives the Runtime still reads valid memory.
  struct SlotKey {
    std::mutex m;
    std::string key;
    void set(const std::string &k) {
      std::lock_guard<std::mutex> lock(m);
      key = k;
    }
    std::string get() {
      std::lock_guard<std::mutex> lock(m);
      return key;
    }
  };
  std::shared_ptr<SlotKey> doc_slot_ = std::make_shared<SlotKey>();
  double mounts_measured_at_ = -1.0; // the frame clock their memory and holds were last read at
  double mounts_mirrored_at_ = -1.0; // ... and their budgets mirrored at
  static constexpr double kMountHeldNotice = 3.0; // seconds held at the limit before the notice
  static constexpr int kMaxOpenMounts = 256;      // as a document's load: total (kMaxMounts)
  static constexpr int kMaxMountDepth = 32;
  void reconcile_mounts(); // render: open, close, parse again; plans the windows menu after
  bool reconcile_mounts_once();
  void open_mount(MountSlot &slot, const Widget &w);
  void close_mount(const std::string &sub_prefix, bool forget_init);
  void refuse_mount(MountSlot &slot, const std::string &why);
  void publish_mount_status(const MountSlot &slot);
  // What the loader is asked for an entry: its URI, args, grants and argv (4.2, 4.5, 4.6).
  MountRequest mount_request(const MountSlot &slot, const MountsSpec &spec) const;
  std::vector<std::string> mount_list(const std::vector<std::string> &literal,
                                      const std::string &bind, const std::string &scope) const;
  std::string mount_norm(const std::string &uri, const std::string &base) const;
  std::string mount_principal(const Widget &w, const std::string &scope, const std::string &uri,
                              const std::string &canonical) const;
  Runtime::MountBudget mount_budget(const MountSlot &slot) const;
  void apply_mount_budget(MountSlot &slot);
  void measure_mounts(); // their memory, their time held at the limit, the notice, the mirror
  void emit_budget_window();
  std::shared_ptr<cvc::state_exec::frame_account> account_for(const std::string &scope) const;
  void bind_mount_verbs(const cvc::state_exec::environment_ptr &env,
                        const std::string &scope) const;
  bool is_mounts_app(const std::string &scope) const { return mount_slots_.count(scope) != 0; }

  // ---- asks and grants: what a mount holds, its input, signals and command line
  // (docs/roadmap/ARIADNE_SHELL.md 4.5, 4.6) ----
  // The static load:s' mounts as the walk last met them, by sub-prefix, and what each holds (null:
  // a wrapper built without grants, which holds the defaults). The mounts: slots' are on their
  // wrappers. Cleared with the tree.
  std::map<std::string, std::shared_ptr<const MountGrants>> load_grants_;
  // The static mounts whose command line and key defaults are published (publish_command_line,
  // once each since the tree was set, before their init:).
  std::set<std::string> prepared_mounts_;
  MountGrants default_mount_grants_ = hold_grants(default_grant_set(), {});
  // The mount at sub-prefix `sub` (a mounts: slot's open one, or a static load: the walk met):
  // what it holds, or null when there is none there.
  const MountGrants *grants_of(const std::string &sub) const;
  // The innermost mount `scope` is in, its sub-prefix; "" for the document's own scope.
  std::string app_of(const std::string &scope) const;
  // What a program at `scope` holds: its mount's tokens (null: the document's own scope).
  std::shared_ptr<const std::set<std::string>> held_at(const std::string &scope) const;
  // Whether grant token `name` holds at `scope` (the read lane's has?).
  bool grant_holds(const std::string &scope, const std::string &name) const {
    const std::string sub = app_of(scope);
    if (sub.empty())
      return held_alone(name);
    const MountGrants *g = grants_of(sub);
    return g && g->holds(name);
  }
  // 4.6 publish a mount's command line at its sub-prefix, before its init: runs.
  void publish_command_line(const std::string &sub, const Widget &wrapper);
  // The keyboard focus: the application (a mount's sub-prefix, "" none) that keys go to, when it
  // holds delegate.keys. It moves to the application whose window takes the focus
  // (Backend::window_focused), off every one when the focus leaves every window (a backend that
  // reports it), and to the one (app-focus) names.
  std::string focused_app_;
  std::optional<std::string> last_focus_window_; // the scope of the window focused last frame
  // This frame's, as the walk meets them, and the last whole frame's: the scope of the focused
  // window, of the window the pointer is over, of the text field that holds the keyboard.
  std::optional<std::string> frame_focus_, frame_hover_, frame_keyboard_;
  std::optional<std::string> hover_scope_, keyboard_scope_;
  void note_window(Backend &b) {
    if (b.window_focused())
      frame_focus_ = current_prefix();
    if (b.window_hovered())
      frame_hover_ = current_prefix();
  }
  void note_text_field(Backend &b) {
    if (b.item_has_keyboard())
      frame_keyboard_ = current_prefix();
  }
  void end_frame_focus(); // the frame's focus, hover and keyboard become the last whole frame's
  // The input events a mount may take, queued by post_input (any thread) when one could take it,
  // routed by the next drain (route_mount_input): a key to the focused application, the pointer to
  // the one whose window it is over.
  std::mutex mount_input_mutex_;
  std::vector<InputEvent> mount_input_;
  std::atomic<bool> mount_input_wanted_{false};  // a known mount holds delegate.keys or .pointer
  std::map<std::string, std::string> keys_down_; // a key that went down to an application: its up
  std::string pointer_app_; // the application a button went down over: its moves and up go there
  void want_mount_input();  // mount_input_wanted_ from the mounts as they stand
  void route_mount_input();
  // Run `body` (a mount's on_key:, on_pointer: or on_signal:, or a binding's on:) as an action in
  // the mount at `sub`, with its functions and `event` bound to `fields`.
  void run_in_mount(const std::string &sub, const std::string &body,
                    std::vector<std::pair<std::string, cvc::state_exec::value_t>> fields);
  // 4.6 `signal` to the mounts that hold delegate.signals for it and have an on_signal:: an
  // interrupt to the focused one, a request to end to every one. True when an interrupt reached
  // one (the host then does not apply its default; a request to end always ends the shell).
  bool signal_mounts(const std::string &signal, const std::string &source);
  bool signal_mount(const std::string &sub, const std::string &signal, const std::string &source);
  // (app-focus AS) and (app-signal AS NAME) from the document's programs (any thread), handled by
  // the next drain. Shared, as the slot key is.
  struct AppRequests {
    std::mutex m;
    std::optional<std::string> focus;
    std::vector<std::pair<std::string, std::string>> signals; // as, signal
  };
  std::shared_ptr<AppRequests> app_requests_ = std::make_shared<AppRequests>();
  void take_app_requests();
  static constexpr const char *kAppEvent = "\x1F"
                                           "app.";
  // Draw as if inside the mount at `scope` (its prefix and functions), for an application's menus
  // and tray icons drawn in the outer bar. An empty scope changes nothing.
  struct ScopedTo {
    Impl &m;
    bool on;
    bool fns;
    ScopedTo(Impl &impl, const std::string &scope, const FunctionScope *functions)
        : m(impl), on(!scope.empty()), fns(on && functions) {
      if (on)
        m.scope_stack.push_back(scope);
      if (fns)
        m.fn_stack.push_back(functions);
    }
    ~ScopedTo() {
      if (fns)
        m.fn_stack.pop_back();
      if (on)
        m.scope_stack.pop_back();
    }
    ScopedTo(const ScopedTo &) = delete;
    ScopedTo &operator=(const ScopedTo &) = delete;
  };
  // ScopedTo's push and pop, for a scope held across steps of the walk (run_walk pops what is
  // left of them if a step throws). push_scope returns what pop_scope takes off again.
  struct PushedScope {
    bool on = false, fns = false;
  };
  PushedScope push_scope(const std::string &scope, const FunctionScope *functions) {
    PushedScope p{!scope.empty(), !scope.empty() && functions};
    if (p.on)
      scope_stack.push_back(scope);
    if (p.fns)
      fn_stack.push_back(functions);
    return p;
  }
  void pop_scope(PushedScope p) {
    if (p.fns)
      fn_stack.pop_back();
    if (p.on)
      scope_stack.pop_back();
  }
  void plan_windows_menu();
  // One instance of a repeated window, as the Windows menu lists it: its title and open state.
  struct RepeatedWindow {
    std::string id, title, open_path, raise_path;
  };
  std::vector<RepeatedWindow> repeated_windows(); // this frame's instances
  bool windows_menu_has_items();                  // anything to list this frame
  void emit_windows_menu(const std::string &title);
  void emit_windows_menu_items();

  // §5.22 the menubar's tray: its TrayIcon children and the clock, at the bar's right end
  // (Backend::begin_tray). The clock shows while kClockKey is true, seeded from the document's
  // `clock:`, else the host's default.
  static constexpr const char *kClockKey = "ariadne.tray.clock";
  bool clock_on_by_default() const;
  static constexpr const char *kBellKey = "ariadne.tray.bell"; // §5.24
  bool bell_on_by_default() const;
  struct TrayEntry {
    const Widget *w = nullptr; // null for the clock or the bell when no entry places it
    bool clock = false;
    bool bell = false;
    // §5.25 a loaded document's icon: drawn in its mount's scope, with its functions
    std::string scope;
    const FunctionScope *functions = nullptr;
    Backend::TrayIcon spec;
    std::string tip;
  };
  Backend::TrayIcon tray_spec(const Widget &w);
  std::string tray_picture(const Widget &w);
  std::vector<TrayEntry> tray_entries(const Widget &bar); // this frame's, in the bar's order
  // The tray, on the walk: begin_tray, the icons (shown ones and the overflow menu's), end_tray.
  struct TrayWalk {
    std::vector<TrayEntry> icons;
    std::vector<Backend::TrayIcon> specs;
    int hidden = 0; // the icons in the overflow menu, from the first
  };
  void emit_tray(std::vector<TrayEntry> icons);
  void tray_menus(const std::shared_ptr<TrayWalk> &t, int k); // no tray: a menu per icon from k
  void tray_more(const std::shared_ptr<TrayWalk> &t, int k);  // the overflow menu's from k
  void tray_shown(const std::shared_ptr<TrayWalk> &t, int k); // the shown icons from k
  void tray_item(const std::shared_ptr<TrayWalk> &t, int k);  // icon k as an item of a menu
  // A tray icon's backend id: the clock's and the bell's own, else its id:, else its place.
  static std::string tray_id(const TrayEntry &i, int k);
  void emit_tray_items(const TrayEntry &icon); // a tray menu's items: its children, or the bell's
  void emit_bell_items();
  // §5.23 the bar's menus folded into one menu under the "menu" icon, when the document lets them
  // and they do not fit beside the tray (Backend::menus_fit).
  std::vector<std::string> bar_titles(const Widget &bar);
  bool collapse_menus(const Widget &bar, const std::vector<TrayEntry> &tray);

  // §4 read-lane: is `w` shown this frame? True when it has no visible_when; otherwise
  // the predicate's result (fail-safe HIDDEN on a broken predicate).
  bool visible(const Widget &w);
  // A visible_when / enabled_when / disabled_when predicate's value: `dflt` when it is broken
  // (warned once), and last frame's value when this frame's budget ran out before it.
  bool predicate(const std::string &src, bool dflt);
  // §2.1 a read-lane expression the frame's budget skipped is not the document's fault, so it is
  // never a warning: it keeps the value it had last frame, or its fail-safe one where it never
  // ran. These hold each expression's last value; `reactive_skips` counts the skips.
  std::unordered_map<std::string, bool> last_good_predicate;
  std::unordered_map<std::string, std::string> last_good_text;
  std::unordered_map<std::string, std::vector<std::string>> last_good_options;
  std::uint64_t reactive_skips = 0;
  // §4 read-lane: is `w` disabled (greyed, non-interactive) this frame? True when
  // enabled_when is falsy OR disabled_when is truthy. Fail-safe DISABLED on a broken
  // predicate.
  bool disabled(const Widget &w);
  // §4 read-lane: evaluate a computed-value expression `expr` to a display string.
  // Fail-safe EMPTY string on a broken predicate (§4.1 fmt→""); warns once.
  std::string eval_text(const std::string &expr);
  // Like eval_text, but for free-text fields (tooltip): if `expr` does NOT evaluate cleanly it
  // is almost certainly a literal that merely starts with '(' — return it VERBATIM and do NOT
  // warn. A genuinely computed value is returned; a broken one degrades to its own text.
  std::string eval_text_or_literal(const std::string &expr);
  // §4 read-lane: evaluate a computed combo-options expression to a list of strings.
  // Fail-safe EMPTY list on a broken expr; warns once.
  std::vector<std::string> eval_options(const std::string &expr);
  // §5.26 the colours a color: key holds or its expression gives this frame (a ';' list, or an
  // expression's list). Fail-safe none on a broken one, warned once.
  std::vector<ItemColor> eval_colors(const std::string &src);
  std::unordered_map<std::string, std::vector<ItemColor>> last_good_colors;
  std::set<std::string> color_warned; // the color: keys warned about a bad value
  // §5.26 the colour, or on a radio the colours, `w` is drawn in this frame: its color: key's or
  // expression's, else the one the document wrote. Empty: the backend's own.
  std::vector<ItemColor> item_colors(const Widget &w);
  // The one colour of `w` (its first) into `out`; false: the backend's own.
  bool item_color(const Widget &w, float out[3]);
  // §5.26 the next emit_node draws a cell of a grid: a group's fill covers the cell.
  bool next_is_cell = false;
  // §3/§4: evaluate a repeat-count expression to a non-negative, capped int. Fail-safe 0.
  int eval_count(const std::string &expr);
  void warn_once(const std::string &msg) {
    if (reactive_warned.insert(msg).second) {
      reactive_warnings.push_back(msg);
      // §5.24 to the app's feed too, from where it was raised, so the bell names its document
      cvc::notification n;
      n.text = msg;
      n.level = 1;
      n.path = current_prefix();
      cvc::notify(app, std::move(n));
    }
  }
};

bool Runtime::Impl::predicate(const std::string &src, bool dflt) {
  const ReactiveEngine::Outcome o = ensure_reactive().eval_bool(src, dflt);
  if (o.skipped) {
    // The frame's budget ran out before it (a slow frame in wasm, a big feed): keep last frame's
    // answer rather than hide or disable the widget for a frame, which flickers it; never ran, the
    // fail-safe one. No warning: the document did nothing wrong.
    ++reactive_skips;
    const auto it = last_good_predicate.find(src);
    return it != last_good_predicate.end() ? it->second : dflt;
  }
  if (!o.error.empty())
    warn_once(o.error);
  else
    last_good_predicate[src] = o.value;
  return o.value;
}

bool Runtime::Impl::visible(const Widget &w) {
  if (w.visible_when.empty())
    return true;
  return predicate(w.visible_when, /*dflt=*/false);
}

bool Runtime::Impl::disabled(const Widget &w) {
  if (w.enabled_when.empty() && w.disabled_when.empty())
    return false;
  // enabled_when falsy -> disabled (fail-safe dflt=false: a broken predicate disables).
  if (!w.enabled_when.empty() && !predicate(w.enabled_when, /*dflt=*/false))
    return true;
  // disabled_when truthy -> disabled (fail-safe dflt=true: a broken predicate disables).
  if (!w.disabled_when.empty() && predicate(w.disabled_when, /*dflt=*/true))
    return true;
  return false;
}

namespace {

// Remember `v` as `key`'s last value, copying only when it changed.
template <class Map, class V> void keep_last(Map &m, const std::string &key, const V &v) {
  auto it = m.find(key);
  if (it == m.end())
    m.emplace(key, v);
  else if (!(it->second == v))
    it->second = v;
}

} // namespace

std::string Runtime::Impl::eval_text(const std::string &expr) {
  const ReactiveEngine::StringOutcome o =
      ensure_reactive().eval_string(expr, /*dflt=*/std::string());
  if (o.skipped) { // last frame's text (none: empty), not a warning
    ++reactive_skips;
    const auto it = last_good_text.find(expr);
    return it != last_good_text.end() ? it->second : std::string();
  }
  if (!o.error.empty())
    warn_once(o.error);
  else
    keep_last(last_good_text, expr, o.value);
  return o.value;
}

std::string Runtime::Impl::eval_text_or_literal(const std::string &expr) {
  const ReactiveEngine::StringOutcome o =
      ensure_reactive().eval_string(expr, /*dflt=*/std::string());
  if (o.skipped) { // last frame's text (none: empty), not the expression's own text
    ++reactive_skips;
    const auto it = last_good_text.find(expr);
    return it != last_good_text.end() ? it->second : std::string();
  }
  // Success -> the computed value; failure -> the literal verbatim (no warning: a tooltip that
  // starts with '(' is far more likely plain text than a broken expression).
  if (!o.error.empty())
    return expr;
  keep_last(last_good_text, expr, o.value);
  return o.value;
}

std::vector<std::string> Runtime::Impl::eval_options(const std::string &expr) {
  const ReactiveEngine::StringListOutcome o = ensure_reactive().eval_string_list(expr);
  if (o.skipped) { // last frame's options (none: empty), not a warning
    ++reactive_skips;
    const auto it = last_good_options.find(expr);
    return it != last_good_options.end() ? it->second : std::vector<std::string>();
  }
  if (!o.error.empty())
    warn_once(o.error);
  else
    keep_last(last_good_options, expr, o.value);
  return o.value;
}

std::vector<ItemColor> Runtime::Impl::eval_colors(const std::string &src) {
  std::vector<ItemColor> out;
  if (!is_expr(src)) { // a key: its value, read each frame
    std::string bad;
    if (!parse_color_list(read_string(app, resolve(src)), out, &bad) &&
        color_warned.insert(src).second)
      warn_once("ari: color: " + src + " holds '" + bad +
                "', which is not r,g,b, [r, g, b], #rrggbb or c0..c7 (drawn in the default "
                "colour)");
    return out;
  }
  const ReactiveEngine::ColorsOutcome o = ensure_reactive().eval_colors(src);
  if (o.skipped) { // last frame's colours (none: the default), not a warning
    ++reactive_skips;
    const auto it = last_good_colors.find(src);
    return it != last_good_colors.end() ? it->second : std::vector<ItemColor>();
  }
  if (!o.error.empty())
    warn_once(o.error);
  else
    keep_last(last_good_colors, src, o.value);
  return o.value;
}

std::vector<ItemColor> Runtime::Impl::item_colors(const Widget &w) {
  if (!w.color_src.empty())
    return eval_colors(w.color_src);
  if (!w.option_colors.empty())
    return w.option_colors;
  if (!w.has_text_color)
    return {};
  ItemColor c;
  c.set = true;
  std::copy(w.text_color, w.text_color + 3, c.rgb);
  return {c};
}

bool Runtime::Impl::item_color(const Widget &w, float out[3]) {
  if (w.color_src.empty()) { // what the document wrote (no list: that is a radio's)
    if (w.has_text_color)
      std::copy(w.text_color, w.text_color + 3, out);
    return w.has_text_color;
  }
  const std::vector<ItemColor> c = eval_colors(w.color_src);
  if (c.empty() || !c.front().set)
    return false;
  std::copy(c.front().rgb, c.front().rgb + 3, out);
  return true;
}

int Runtime::Impl::eval_count(const std::string &expr) {
  constexpr int64_t kMaxRepeat = 4096; // a UI with thousands of repeated widgets is pathological
  const ReactiveEngine::IntOutcome o = ensure_reactive().eval_int(expr, /*dflt=*/0);
  if (o.skipped)
    ++reactive_skips; // the budget's, not the document's: no warning
  else if (!o.error.empty())
    warn_once(o.error);
  if (!o.error.empty()) {
    // The eval fail-safed (budget exhausted / broken). Rather than collapse the repeated list to 0
    // this frame, reuse the last count this expression yielded successfully (if any) — a transient
    // skip then leaves the roster/peer-window list unchanged instead of blinking empty. With no
    // prior success the fallback is still 0, so a genuinely-never-valid expr behaves as before.
    const auto it = last_good_count.find(expr);
    if (it != last_good_count.end())
      return it->second;
  }
  int64_t n = o.value;
  if (n < 0)
    n = 0;
  if (n > kMaxRepeat) {
    warn_once("ari: repeat count " + std::to_string(n) + " exceeds the cap (" +
              std::to_string(kMaxRepeat) + ") — clamped");
    n = kMaxRepeat;
  }
  // Remember this frame's good value only when the eval actually succeeded (o.error empty): a
  // fail-safe 0 must never be cached as "last good", or it would pin the list empty.
  if (o.error.empty())
    last_good_count[expr] = static_cast<int>(n);
  return static_cast<int>(n);
}

namespace {
// §5.30 a context menu's items in emit_path: apart from the widget's children (instance keys,
// unnamed ids).
constexpr std::size_t kItemsPath = std::size_t(1) << 30;
// §3 the repeat instances one frame draws, across all its repeats: each repeat is capped on its
// own (kMaxRepeat), but many could still compound into an unbounded walk (an instance's deep copy
// and its emit), which the eval-time budget doesn't see.
constexpr int kMaxFrameInstances = 16384;
} // namespace

void Runtime::Impl::push_then(std::function<void()> then) {
  WalkFrame f;
  f.op = WalkFrame::Op::Then;
  f.then = std::move(then);
  walk_.push_back(std::move(f));
}

// Start the work `start` pushes and step walk_ until it is back where it was. Called again from
// inside a step (a registered widget fn's ctx.emit), it steps only what it pushed. If a step
// throws, the work it left is dropped and the scopes, functions and repeat it held are popped, as
// the guards of a recursive walk did when they unwound.
template <class Start> void Runtime::Impl::run_walk(Start &&start) {
  const std::size_t base = walk_.size();
  const std::size_t scopes = scope_stack.size();
  const std::size_t functions = fn_stack.size();
  const Widget *const repeat = emitting_repeat_;
  const int instance = emitting_instance_;
  try {
    start();
    while (walk_.size() > base)
      step_walk();
  } catch (...) {
    walk_.resize(base);
    scope_stack.resize(scopes);
    fn_stack.resize(functions);
    emitting_repeat_ = repeat;
    emitting_instance_ = instance;
    throw;
  }
}

// ARIADNE_SHELL.md 4.2 on the walk: each mount open in the slot, in the slot's order, under its
// own id. (The slots were reconciled at the top of the frame, so they stay as they are while the
// walk runs.)
CVC_ARI_NOINLINE void Runtime::Impl::emit_mounts(const Widget &w) {
  if (w.bind.empty())
    return;
  const std::string key = resolve(w.bind);
  std::vector<std::pair<std::string, const Widget *>> open;
  for (const auto &[sub, slot] : mount_slots_)
    if (slot.slot_key == key && slot.parent_scope == current_prefix() && slot.wrapper)
      open.emplace_back(sub, slot.wrapper.get());
  for (std::size_t k = open.size(); k-- > 0;) // the last first: the first is on top
    push_then([this, k, sub = std::move(open[k].first), wrapper = open[k].second] {
      emit_path.push_back(k);
      backend->push_id(sub.c_str());
      push_then([this] { // after the mount
        backend->pop_id();
        emit_path.pop_back();
      });
      begin_emit(*wrapper);
    });
}

// A label that is an expression is worked out each frame on the read lane, as a text's bind is
// (ARIADNE_SHELL.md 4.9: the shell's Apps menu lists its catalog by name).
CVC_ARI_NOINLINE bool Runtime::Impl::menu_item_action(const Widget &w) {
  if (!is_expr(w.label))
    return backend->menu_item_action(w.label.c_str());
  const std::string shown = eval_text_or_literal(w.label);
  return backend->menu_item_action(shown.c_str());
}

// One step of the walk: the top frame does what the recursive walk did next for it. A step never
// holds a reference into walk_ across a call that can push to it.
void Runtime::Impl::step_walk() {
  using Op = WalkFrame::Op;
  WalkFrame &f = walk_.back();
  switch (f.op) {
  case Op::Then: {
    const std::function<void()> then = std::move(f.then);
    walk_.pop_back();
    then();
    return;
  }
  case Op::Children: {
    const std::vector<Widget> &list = f.items ? f.w->context_items : f.w->children;
    if (f.next > 0)
      emit_path.pop_back(); // the child before is done
    if (f.next == list.size()) {
      walk_.pop_back();
      return;
    }
    const std::size_t k = f.next++;
    emit_path.push_back(f.items ? kItemsPath + k : k);
    begin_emit(list[k]);
    return;
  }
  case Op::Cells: {
    if (f.next > 0)
      emit_path.pop_back();
    if (f.next == f.w->children.size()) {
      walk_.pop_back();
      return;
    }
    const std::size_t k = f.next++;
    const Widget &c = f.w->children[k];
    emit_path.push_back(k);
    begin_cells(c);
    return;
  }
  case Op::Instances: {
    if (f.open_item) { // the instance before is done
      f.open_item = false;
      instance_path.pop_back();
      backend->pop_id();
    }
    if (f.next < static_cast<std::size_t>(f.count) && frame_instances >= kMaxFrameInstances) {
      warn_once("ari: too many repeated widget instances this frame (cap " +
                std::to_string(kMaxFrameInstances) + ") — remaining instances skipped");
      f.next = static_cast<std::size_t>(f.count);
    }
    if (f.next >= static_cast<std::size_t>(f.count)) {
      if (f.outermost) {
        emitting_repeat_ = nullptr;
        emitting_instance_ = -1;
      }
      walk_.pop_back();
      return;
    }
    const int i = static_cast<int>(f.next++);
    ++frame_instances;
    if (f.outermost)
      emitting_instance_ = i;
    const Widget *inst = nullptr;
    if (f.outermost && !f.w->scope.empty()) { // a repeated mount: its instance, made once
      auto it = mount_instances_.find({f.w, i});
      if (it == mount_instances_.end()) {
        Widget made = substitute_index(*f.w, i);
        made.repeat.clear();
        it = mount_instances_.emplace(std::make_pair(f.w, i), std::move(made)).first;
      }
      inst = &it->second;
    } else {
      // The instance before is no longer drawn: this one is made in its place.
      if (!f.inst)
        f.inst = std::make_unique<std::optional<Widget>>();
      Widget &made = f.inst->emplace(*f.w);
      substitute_index_in(made, std::to_string(i));
      made.repeat.clear(); // the instance renders once
      inst = &made;
    }
    if (f.cell) {
      // Visibility is decided before the cell advances, so a hidden instance takes no cell.
      if (!visible(*inst))
        return; // the next instance, at the next step
      backend->grid_next_cell();
      backend->push_id(std::to_string(i).c_str());
      instance_path.push_back(i);
      walk_.back().open_item = true;
      autofocus(*inst, i);
      next_is_cell = true; // §5.26 a group's fill covers its cell
      emit_node(*inst);
      return;
    }
    backend->push_id(std::to_string(i).c_str()); // per-instance identity
    instance_path.push_back(i);
    f.open_item = true;
    if (visible(*inst)) { // §4: a falsy visible_when hides it + subtree
      autofocus(*inst, i);
      emit_node(*inst);
    }
    return;
  }
  case Op::Container: {
    // Nothing here pushes to walk_, so the frame is popped once it is done with.
    if (f.grid) {
      Backend &b = *backend;
      b.end_grid();
      if (!f.path.empty()) { // a resizable row split: its pane sizes, as dragged
        const std::vector<float> cur = b.grid_tracks();
        if (!cur.empty())
          write<std::string>(app, f.path, format_tracks(cur));
      }
      if (!f.path2.empty()) { // a resizable table: its column widths
        const std::vector<float> cur = b.grid_columns();
        if (!cur.empty())
          write<std::string>(app, f.path2, format_tracks(cur));
      }
    }
    if (f.scoped)
      scope_stack.pop_back();
    if (f.fns)
      fn_stack.pop_back();
    walk_.pop_back();
    return;
  }
  case Op::Node: {
    WalkFrame d = std::move(f);
    walk_.pop_back();
    leave_node(d);
    return;
  }
  case Op::ContextMenu: {
    const MenuEvent *const outer = f.outer;
    walk_.pop_back();
    menu_event = outer;
    if (reactive) // unbound now: the next menu binds its own, even one made where this one was
      reactive->set_event(menu_event ? &menu_event->dict : nullptr);
    backend->end_context_menu();
    return;
  }
  }
}

void Runtime::Impl::emit_children(const Widget &w) {
  if (w.children.empty())
    return;
  WalkFrame f;
  f.op = WalkFrame::Op::Children;
  f.w = &w;
  walk_.push_back(std::move(f));
}

// Lay a container's children out per its layout (§3.0.3b): a Grid/Horizontal
// layout flows them into the backend's sized tracks; anything else is a plain
// vertical stack. Used for both a Window's body and a Group.
void Runtime::Impl::wire_holes(const std::string &parent_prefix, const std::string &sub_prefix,
                               const std::vector<LinkHole> &links) {
  cvc::state &root = cvc::state::instance(app);
  for (const LinkHole &h : links) {
    // The hole node the module sees inside its chroot; the target it grants, resolved against the
    // mount's PARENT scope. A transparent link so a read sees through (bind.h read_effective /
    // state-get read_through_link); setLinkWritable gates the write (rw -> target, ro -> local).
    const std::string hole_path = resolve_bind(sub_prefix, h.name);
    const std::string target = resolve_bind(parent_prefix, h.target);
    // Record the path BEFORE linking, so even if linkTo throws mid-way the node is still torn
    // down on the next reconcile (clearLink is idempotent on a non-link node).
    planted_holes.push_back(hole_path);
    try {
      cvc::state &node = root(hole_path);
      node.linkTo(target, cvc::state::link_mode::transparent);
      node.setLinkWritable(h.writable);
    } catch (const std::exception &) {
      // Never throw into a frame; a hole that cannot be wired simply isn't (the module then
      // reads/writes its own local node — sandboxed, no parent reach).
    }
  }
}

void Runtime::Impl::run_mount_init(const std::string &sub_prefix, const std::string &script) {
  std::vector<std::string> errs;
  // (has? ...) and the document's own intrinsics as in its own init: and every action: a module's
  // host is the same host. A grant token answers from what the mount holds (ARIADNE_SHELL.md 4.5).
  const std::shared_ptr<const std::set<std::string>> held = held_at(sub_prefix);
  run_init(
      app, sub_prefix, script, &errs,
      [this, held](const std::string &n) {
        if (is_grant_token(n))
          return held ? held->count(n) != 0 : held_alone(n);
        return has(n);
      },
      [this](const std::shared_ptr<cvc::state_exec::environment> &env,
             cvc::state_exec::intrinsics_context &ictx) {
        bind_own_intrinsics(env, ictx, nullptr);
      });
  for (const std::string &e : errs)
    warn_once("ari: mount init at '" + sub_prefix + "': " + e);
}

void Runtime::Impl::emit_container(const Widget &w) {
  Backend &b = *backend;
  // §12: a mount point (Widget::scope) scopes its CHILDREN — the loaded fragment — to a deeper
  // sub-prefix; the mount widget's own reactive fields already evaluated in the parent scope
  // (in emit() before we got here). Push the composed scope for the children walk, pop after
  // (Op::Container; run_walk pops it if a backend throws, so the stack is never left unbalanced).
  const bool scoped = !w.scope.empty();
  if (scoped && !quit_apps.empty() && !app_running(resolve_bind(current_prefix(), w.scope)))
    return; // §5.25 an application the user quit: nothing of it is drawn, nor its tick started
  if (scoped) {
    const std::string parent_prefix = current_prefix(); // before the push
    const std::string sub_prefix = resolve_bind(parent_prefix, w.scope);
    // Plant the mount's parent-scope holes once, BEFORE emitting the fragment that reads them.
    if (!w.links.empty() && wired_mounts.insert(sub_prefix).second)
      wire_holes(parent_prefix, sub_prefix, w.links);
    // ARIADNE_SHELL.md 4.6 a load:'s command line and its keys' defaults, once, before its init:
    // (a mounts: entry's are published as it opens, open_mount).
    if ((w.command_line || w.grants) && !is_mounts_app(sub_prefix) &&
        prepared_mounts_.insert(sub_prefix).second)
      publish_command_line(sub_prefix, w);
    // Run the fragment's init: once (after its holes exist, so init may seed through them), its
    // state: seeds (§5.3) just before. Kept in ran_inits, NOT cleared on reconcile — a one-shot
    // seed, never re-run per frame/rebuild.
    if ((!w.state_seeds.empty() || !w.init_script.empty()) && ran_inits.insert(sub_prefix).second) {
      seed_state(app, sub_prefix, w.state_seeds);
      if (!w.init_script.empty())
        run_mount_init(sub_prefix, (w.functions ? w.functions->prelude : "") + w.init_script);
    }
    if (!w.tick_script.empty())
      ensure_mount_tick(sub_prefix, w); // after its init:, so the first tick sees the seed
    // §5.25 the application's name, as its menu has it: the mount's name:, else the loaded
    // document's, else its mount id (worked out here, in the parent's scope).
    std::string name = mount_name_of(w.app_name_bind);
    if (name.empty())
      name = !w.app_name.empty() ? w.app_name : w.scope.substr(w.scope.rfind('.') + 1);
    mount_names[sub_prefix] = std::move(name);
    if (w.functions)
      mount_functions[sub_prefix] = w.functions->prelude;
    scope_stack.push_back(sub_prefix);
  }
  // §6.6 the root's and a mount's functions are the ones programs started in its subtree get.
  if (w.functions)
    fn_stack.push_back(w.functions.get());
  // After the children (WalkFrame::Op::Container): the grid ended, then these popped. A plain
  // stack that pushed nothing needs no frame.
  WalkFrame after;
  after.op = WalkFrame::Op::Container;
  after.w = &w;
  after.scoped = scoped;
  after.fns = static_cast<bool>(w.functions);

  if (w.layout.kind == LayoutKind::Grid || w.layout.kind == LayoutKind::Horizontal) {
    // The grid's identity: its id:, else its label, else where it sits in the tree. Backends key
    // their per-grid state by it (ImGui its table, whose column widths it keeps), and so does the
    // persisted tree.<id>, so two unnamed grids in one window must not share it. The generated id
    // is one state-path segment ("~0_2_1"), so tree.<id>.tracks stays one key.
    std::string id = !w.id.empty() ? w.id : w.label;
    if (id.empty()) {
      id = "~";
      for (std::size_t i = 0; i < emit_path.size(); ++i)
        id += (i ? "_" : "") + std::to_string(emit_path[i]);
    }
    // §11.4 two-way track edge: a resizable row-split grid persists its dragged pane sizes to
    // tree.<id>.tracks, and a resizable table its column widths to tree.<id>.columns. Seed the
    // backend from the persisted sizes before opening (apply-once on the backend side), then write
    // the post-drag sizes back after closing. A table that is not resizable has nothing to drag,
    // so it neither seeds nor persists.
    const bool split = w.layout.is_row_split();
    const bool table = w.layout.resizable && !split;
    std::string tracks_path, columns_path;
    // After forget_layout(), as for a window: the first frame takes no sizes the last document
    // persisted.
    const auto persisted = [&](const std::string &path) {
      const bool fresh = windows_forgotten && laid_out.insert(path).second;
      return fresh ? std::vector<float>{} : parse_tracks(read_string(app, path));
    };
    if (split) {
      tracks_path = resolve("tree." + id + ".tracks");
      const std::vector<float> seed = persisted(tracks_path);
      if (!seed.empty())
        b.seed_grid_tracks(seed);
    }
    if (table) {
      columns_path = resolve("tree." + id + ".columns");
      const std::vector<float> seed = persisted(columns_path);
      if (!seed.empty())
        b.seed_grid_columns(seed);
    }
    if (b.begin_grid(w.layout, id.c_str())) {
      // Each child a cell, a `repeat` child a cell per instance (step_walk: Op::Cells,
      // begin_cells), then end_grid and the sizes written back (Op::Container).
      after.grid = true;
      after.path = std::move(tracks_path);
      after.path2 = std::move(columns_path);
      walk_.push_back(std::move(after));
      if (!w.children.empty()) {
        WalkFrame cells;
        cells.op = WalkFrame::Op::Cells;
        cells.w = &w;
        walk_.push_back(std::move(cells));
      }
      return;
    }
  } else {
    if (after.scoped || after.fns)
      walk_.push_back(std::move(after));
    emit_children(w);
    return;
  }
  // No grid opened: nothing of it is drawn.
  if (after.scoped)
    scope_stack.pop_back();
  if (after.fns)
    fn_stack.pop_back();
}

// A `repeat` template: the count and the outermost repeat's bookkeeping, then its instances, one a
// step (step_walk, Op::Instances).
void Runtime::Impl::begin_repeat(const Widget &w, bool cell) {
  const int n = eval_count(w.repeat); // capped, fail-safe 0
  // §12 the outermost repeat's instance, for the mount ticks started under it (a nested repeat is
  // a per-frame copy, so its instances are tracked by the outer one's).
  const bool outermost = emitting_repeat_ == nullptr;
  if (outermost && !mount_ticks_.empty())
    sweep_repeated_ticks(&w, n);
  if (outermost)
    emitting_repeat_ = &w; // until its last instance is done (Op::Instances)
  WalkFrame f;
  f.op = WalkFrame::Op::Instances;
  f.w = &w;
  f.count = n;
  f.cell = cell;
  f.outermost = outermost;
  walk_.push_back(std::move(f));
}

// A grid's child: a plain one is one cell; a `repeat` child a cell per instance. Visibility is
// decided BEFORE the cell advances, so a hidden child or instance takes no cell (else it would
// leave an empty one and shift the siblings after it).
void Runtime::Impl::begin_cells(const Widget &c) {
  if (!c.repeat.empty()) {
    begin_repeat(c, /*cell=*/true);
    return;
  }
  if (!visible(c))
    return;
  backend->grid_next_cell();
  autofocus(c, -1);
  next_is_cell = true; // §5.26 a group's fill covers its cell
  emit_node(c);
}

void Runtime::Impl::autofocus(const Widget &inst, int i) {
  if (!inst.autofocus)
    return;
  std::string key;
  for (const std::size_t k : emit_path)
    key += std::to_string(k) + ".";
  if (i >= 0)
    key += "#" + std::to_string(i);
  if (!autofocus_shown.count(key))
    backend->focus_next_widget();
  autofocus_now.insert(std::move(key));
}

void Runtime::Impl::emit(const Widget &w) {
  run_walk([&] { begin_emit(w); });
}

void Runtime::Impl::begin_emit(const Widget &w) {
  if (!w.repeat.empty()) {
    begin_repeat(w, /*cell=*/false); // each instance under its own id (Op::Instances)
    return;
  }
  if (visible(w)) { // §4: a falsy visible_when hides it + subtree
    autofocus(w, -1);
    emit_node(w);
  }
}

// §5.30 a right-click menu's `event`: what it was opened over (`on`: group, viewport, window or
// main; a custom widget other than a viewport by its type) and that widget's `id`, the menu's
// `id`, how (`via`) and where (`px`, `py`) it was opened, and the pick at that point over a scene
// view. The pick's fields are always there, empty (hit "0") over a widget that is no scene view, so
// a visible_when: can read them anywhere; they are strings, as a viewport's pick: key holds them,
// but `tags` is a list.
Runtime::Impl::MenuEvent Runtime::Impl::context_event(const Widget &w, ContextMenuOver over,
                                                      const ContextMenuOpen &o,
                                                      const Runtime::ContextPick &pick) const {
  namespace se = cvc::state_exec;
  MenuEvent e;
  auto &f = e.fields;
  std::string on = "group";
  if (over == ContextMenuOver::Main)
    on = "main";
  else if (over == ContextMenuOver::Window)
    on = "window";
  else if (w.kind == Kind::Custom)
    on = w.custom_type;
  f.emplace_back("on", se::value_t(on));
  f.emplace_back("id", se::value_t(over == ContextMenuOver::Main                     ? std::string()
                                   : over == ContextMenuOver::Window && w.id.empty() ? w.label
                                                                                     : w.id));
  f.emplace_back("menu", se::value_t(w.context_menu_id));
  f.emplace_back("via", se::value_t(o.via.empty() ? std::string("pointer") : o.via));
  f.emplace_back("px", se::value_t(static_cast<double>(o.x)));
  f.emplace_back("py", se::value_t(static_cast<double>(o.y)));
  std::vector<std::pair<std::string, std::string>> hit;
  if (pick)
    hit = pick();
  const auto field = [&hit](const char *name, const char *dflt) {
    for (const auto &kv : hit)
      if (kv.first == name)
        return kv.second;
    return std::string(dflt);
  };
  for (const char *k : {"node", "x", "y", "z", "i", "j", "k", "value", "axis", "view", "mods"})
    f.emplace_back(k, se::value_t(field(k, "")));
  f.emplace_back("hit", se::value_t(field("hit", "0")));
  std::vector<se::value_t> tags;
  {
    std::istringstream words(field("tags", ""));
    for (std::string t; words >> t;)
      tags.emplace_back(t);
  }
  f.emplace_back("tags", se::make_list(std::move(tags)));
  e.dict = se::make_dict(f);
  return e;
}

void Runtime::Impl::emit_context_menu(const Widget &w, ContextMenuOver over,
                                      const Runtime::ContextPick &pick) {
  // Its instance (a repeated group's menus are apart), and the id the backend keys the popup on.
  const std::string key = instance_key() + (over == ContextMenuOver::Main ? "|main" : "|menu");
  const std::string id = over == ContextMenuOver::Main ? std::string("ctx:main") : context_id(w);
  ContextMenuOpen opened;
  if (!backend->begin_context_menu(id.c_str(), over, opened)) {
    menu_events.erase(key);
    return;
  }
  auto it = menu_events.find(key);
  if (opened.now || it == menu_events.end())
    it = menu_events.insert_or_assign(key, context_event(w, over, opened, pick)).first;
  it->second.walk = walk_no;
  // After its items: the menu event it took the place of back, and end_context_menu
  // (step_walk, Op::ContextMenu).
  WalkFrame after;
  after.op = WalkFrame::Op::ContextMenu;
  after.outer = std::exchange(menu_event, &it->second);
  walk_.push_back(std::move(after));
  // Its items, apart from the widget's children in the tree's paths (instance keys, unnamed ids).
  if (!w.context_items.empty()) {
    WalkFrame items;
    items.op = WalkFrame::Op::Children;
    items.items = true;
    items.w = &w;
    walk_.push_back(std::move(items));
  }
}

void Runtime::Impl::emit_node(const Widget &w) {
  if (w.kind == Kind::TrayIcon) // emit_tray draws it, after the bar's menus; not here, where its
    return;                     // tooltip and pointer handlers would land on the item before it
  Backend &b = *backend;
  const bool cell = std::exchange(next_is_cell, false); // a grid's cell (§5.26)
  // §4 read-lane: wrap the widget (and its subtree, emitted within the cases) in the
  // backend's disabled scope when a reactive enabled_when/disabled_when says so.
  const bool dis = disabled(w);
  // §5.26 the colour scope around one leaf (a button's, a menu item's): drawn in the widget's
  // colour this frame, when it has one.
  const auto tinted = [&](auto &&draw) {
    float rgb[3];
    const bool tint = item_color(w, rgb);
    if (tint)
      b.begin_text_color(rgb);
    const auto r = draw();
    if (tint)
      b.end_text_color();
    return r;
  };
  if (dis)
    b.begin_disabled();
  const char *label = w.label.c_str();
  bool inner_click = false; // a group: something inside it took this walk's click
  switch (w.kind) {
  case Kind::Mounts: // ARIADNE_SHELL.md 4.2: each mount open in this slot, in its own scope
    emit_mounts(w);
    break;
  case Kind::Menubar:
    // §5.25 a loaded document's (a repeated one's instance too): one menu of the outer bar
    if (!scope_stack.empty() && is_app_scope(current_prefix()))
      break;
    if (b.begin_main_menu_bar()) {
      ++menubar_depth;
      // §5.22 seed the clock's key before the bar's menus, so a Show clock toggle among them (which
      // seeds with its own default) finds the host's or the document's choice already there.
      read_bool_or_seed(app, resolve(kClockKey), clock_on_by_default() ? 1 : 0);
      read_bool_or_seed(app, resolve(kBellKey), bell_on_by_default() ? 1 : 0); // §5.24
      std::vector<TrayEntry> tray = tray_entries(w);
      const bool collapse = collapse_menus(w, tray);
      // The rest goes on the walk, last first: the bar's own menus, the loaded documents' (§5.25),
      // the Windows menu (§5.9: last unless the document placed it), then the tray, after every
      // menu at the bar's right end, then the bar ends (leave_node).
      WalkFrame after;
      after.op = WalkFrame::Op::Node;
      after.w = &w;
      after.dis = dis;
      walk_.push_back(std::move(after));
      push_then([this, tray = std::move(tray)]() mutable { emit_tray(std::move(tray)); });
      const auto windows_menu_last = [this] {
        if (!windows_menu.placed)
          emit_windows_menu(windows_menu.title);
      };
      if (collapse) {
        // §5.23 one menu, its items the bar's menus as submenus. menu_depth stays 0 for it, so a
        // menu titled as the Windows menu still takes the Windows menu's items (§5.9).
        const std::string glyph(icon_glyph("menu", b.unicode_glyphs()));
        if (b.begin_menu(glyph.c_str())) {
          push_then([this] { backend->end_menu(); });
          push_then(windows_menu_last);
          push_then([this] { emit_app_menus(); });
          emit_children(w);
        }
      } else {
        push_then(windows_menu_last);
        push_then([this] { emit_app_menus(); });
        emit_children(w);
      }
      return;
    }
    break;

  case Kind::TrayIcon: // drawn by emit_tray, after the bar's menus
    break;

  case Kind::WindowsMenu: // §5.9: placed here; outside a menubar it is only the document's setting
    if (menubar_depth > 0 && app_menu_depth == 0 && w.bdef)
      emit_windows_menu(w.label.empty() ? windows_menu.title : w.label);
    break;

  case Kind::Menu: {
    // §5.9: a menu straight in the bar with the Windows menu's title takes its items.
    const bool merge = windows_menu.merged && menubar_depth > 0 && menu_depth == 0 &&
                       app_menu_depth == 0 && w.label == windows_menu.title;
    if (b.begin_menu(label)) {
      ++menu_depth;
      WalkFrame after; // its items, then the Windows menu's when it merges, end_menu (leave_node)
      after.op = WalkFrame::Op::Node;
      after.w = &w;
      after.dis = dis;
      after.flag = merge;
      walk_.push_back(std::move(after));
      emit_children(w);
      return;
    }
    if (merge)
      windows_menu_drawn = true; // open or not, it stands for the Windows menu this frame
    break;
  }

  case Kind::MenuItemToggle: {
    const std::string path = resolve(w.bind);
    const bool cur =
        read_bool_or_seed(app, path, w.bdef ? 1 : 0) != 0; // tolerant of "true"/"false"
    const BoolEdit e = tinted([&] { return b.menu_item_toggle(label, cur); });
    if (e.committed) {
      write<int>(app, path, e.value ? 1 : 0);
      if (!w.on.empty())
        enqueue(w.on);
    }
    break;
  }

  case Kind::MenuItemAction:
    if (tinted([&] { return menu_item_action(w); }))
      enqueue(w.on);
    break;

  case Kind::Window: {
    // The stable identity is the id (defaults to the label); the backend keeps a
    // window's geometry keyed on it. Begin/End pair unconditionally.
    const std::string &id = w.id.empty() ? w.label : w.id;
    // The window's open state lives at tree.<id>.open (default open). A closed window is not
    // rendered at all. A `closable:` window's close (X) button (p_open below) flips it, as does a
    // menu_toggle bound to the same key (the built-in Windows menu, §5.9, or the document's own).
    // Only a closable window seeds the key: another is closed only when something set it to 0.
    const std::string open_path = resolve("tree." + id + ".open");
    bool open =
        (w.closable ? read_bool_or_seed(app, open_path, 1) : read_bool_or(app, open_path, 1)) != 0;
    if (!open)
      break; // closed -> skip the geometry seed + body entirely
    // §11.4 two-way geometry edge: seed the window from a persisted tree.<id>.geometry (if any)
    // BEFORE opening it, then write the post-interaction geometry back AFTER — so the state node is
    // the authority next frame. The backend never touches cvc::state; it only seeds/reports.
    const std::string geo_path = resolve("tree." + id + ".geometry");
    WindowGeom seed;
    // After forget_layout(), a window's first frame ignores what is persisted: the last document's.
    const bool fresh = windows_forgotten && laid_out.insert(geo_path).second;
    bool have_seed = !fresh && parse_geom(read_string(app, geo_path), seed);
    // §3.0.3b first placement: with no persisted geometry yet, seed the window's INITIAL position
    // from the authored `pos:` (ImGui otherwise stacks every new window at the same default spot,
    // so a declared pos would be dead). Seed as FirstUseEver — the user's drag then persists via
    // tree.<id>.geometry and wins next frame. Authored `size:` already reaches begin_window below;
    // only pos needs this bridge, so a pos-only seed leaves begin_window to apply the size.
    if (!have_seed && w.has_pos) {
      seed.has_pos = true;
      seed.x = w.pos_x;
      seed.y = w.pos_y;
      seed.authored = true;
      have_seed = true;
    }
    if (have_seed)
      b.seed_window_geometry(seed);
    // Raise edge: a one-shot request at tree.<id>.raise (set to "1" by e.g. a "click a user"
    // handler, alongside tree.<id>.open) brings this window to the front this frame. Consume it so
    // it fires once (not every frame). Opening a closed window + raising it = set both keys in the
    // handler.
    const std::string raise_path = resolve("tree." + id + ".raise");
    bool raise = read_string(app, raise_path) == "1";
    if (raise)
      write<std::string>(app, raise_path, ""); // consume the one-shot
    // A window may BIND its visible title (e.g. a per-peer camera window titled by the peer's
    // name): resolve w.bind as the title when present, else the static label. begin_window uses
    // "title###id", so the title may change each frame while the ###id identity stays stable.
    std::string wtitle = w.label;
    if (!w.bind.empty()) {
      const std::string t = is_expr(w.bind) ? eval_text(w.bind) : read_string(app, resolve(w.bind));
      if (!t.empty())
        wtitle = t;
    }
    // §5.9 maximize: tree.<id>.maximized (1: the window fills the screen under the menu bar), and a
    // button in its title bar where the backend has one (`maximizable: false`: none). While it is
    // maximized its geometry is not persisted, so tree.<id>.geometry keeps where it goes back to.
    const std::string max_path = resolve("tree." + id + ".maximized");
    // `maximized: true | phone` is how it starts, until the user (or the document) writes the key.
    const bool start_max =
        w.start_maximized == "true" || (w.start_maximized == "phone" && b.view_is_phone());
    bool maximized = w.maximizable && read_bool_or(app, max_path, start_max ? 1 : 0) != 0;
    const bool can_maximize = w.maximizable && b.next_window_maximize(&maximized);
    if (!can_maximize)
      maximized = false;
    const bool was_maximized = maximized;
    // A window that is maximized now and was not drawn maximized last frame (its button, a double
    // click, or a document writing the key) comes to the front, as it covers the others.
    if (maximized && !maximized_shown.count(max_path))
      raise = true;
    if (raise)
      b.focus_next_window();
    b.next_window_resizable(w.resizable);
    // The backend's identity for it: a mounted document's window is its own, apart from the same
    // id in the document or in another mount of the same document (§5.25: one loaded twice).
    const std::string backend_id = scope_stack.empty() ? id : id + "@" + current_prefix();
    const bool visible = b.begin_window(wtitle.c_str(), backend_id.c_str(), w.size, w.frame_border,
                                        w.closable ? &open : nullptr);
    note_window(b); // ARIADNE_SHELL.md 4.6 whose window has the focus, and the pointer
    if (can_maximize && maximized != was_maximized)
      write<int>(app, max_path, maximized ? 1 : 0); // its button
    if (maximized)
      maximized_now.insert(max_path);
    // Persist the current geometry. When the backend can't report a size (a collapsed window),
    // back-fill it from the seed so a collapse doesn't clobber the stored content size.
    WindowGeom persist = b.window_geometry();
    if (!persist.has_size && have_seed && seed.has_size) {
      persist.w = seed.w;
      persist.h = seed.h;
      persist.has_size = true;
    }
    if ((persist.has_pos || persist.has_size) && !was_maximized && !maximized)
      write<std::string>(app, geo_path, format_geom(persist));
    if (visible) {
      b.push_id(id.c_str());
      if (w.context_menu) // §5.30 its right-click menu: over its body, inside the window
        b.begin_context_area(context_id(w).c_str(), ContextMenuOver::Window);
      // After its body: the backdrop and context area end, its menu, pop_id, end_window and the
      // open state written (leave_node).
      WalkFrame after;
      after.op = WalkFrame::Op::Node;
      after.w = &w;
      after.dis = dis;
      after.flag = open;
      after.path = open_path;
      after.backdrop = open_backdrop(w, /*window=*/true); // §5.12
      walk_.push_back(std::move(after));
      emit_container(w); // §3.0.3b: the window body honours w.layout
      return;
    }
    b.end_window();
    if (w.closable)
      write<int>(app, open_path, open ? 1 : 0); // the close (X) button may have cleared `open`
    break;
  }

  case Kind::Group: {
    // A group with an id scopes its children's identities, as a window does: two groups can each
    // hold a "Change" button.
    if (!w.id.empty())
      b.push_id(w.id.c_str());
    // §5.30 its right-click menu is over all of it (the document root's is the main view's, which
    // render() opens after the walk).
    const bool menu = w.context_menu && &w != &root;
    if (menu)
      b.begin_context_area(context_id(w).c_str(), ContextMenuOver::Item);
    // §4.6 a group with pointer handlers of its own, or `hover: highlight`, is one item over its
    // whole area (a feed's card): named by its id:, else by its place in the tree, so two unnamed
    // ones never share a backend id.
    const bool item =
        !w.on_click.empty() || !w.on_hover.empty() || !w.on_hover_end.empty() || w.hover_highlight;
    if (item) {
      ItemGroupOptions o;
      o.focusable = !w.on_click.empty();
      o.highlight = w.hover_highlight;
      b.begin_item_group(item_id(w).c_str(), o);
    }
    // After its children: the backdrop and item group end, its menu, pop_id (leave_node).
    WalkFrame after;
    after.op = WalkFrame::Op::Node;
    after.w = &w;
    after.dis = dis;
    after.item = item;
    after.count = clicks_fired;
    after.backdrop = open_backdrop(w, /*window=*/false, cell); // §5.12, §5.26
    walk_.push_back(std::move(after));
    emit_container(w); // §3.0.3b: a group honours w.layout (grid / vertical)
    return;
  }

  case Kind::Text:
    if (!w.copy.empty()) // §5.28 its context menu's Copy
      b.next_item_copy(w.copy);
    if (w.has_runs) { // §5.20 rich text: styled runs and links in a paragraph
      emit_rich_text(w, label, inner_click);
      break;
    }
    if (w.literal_text) {
      // §5.11 emoji: the caption's `:name:`s as pictures, where the backend draws them in a line.
      if (!w.emoji.empty() && emit_text_runs(w, "", label, true))
        break;
      // A colour (§5.26: written, or computed this frame) -> the *_colored leaf; else the plain
      // draw.
      float tint[3];
      if (item_color(w, tint))
        b.text_line_colored(label, tint);
      else
        b.text_line(label);
    } else {
      // §4 read-lane (homoiconic): a `bind` that is an s-expression (starts with '(') is a
      // COMPUTED value re-evaluated each frame, not a state path. State paths never start
      // with '(', so this is unambiguous. Read-only, bounded, fail-safe empty.
      const std::string val =
          is_expr(w.bind) ? eval_text(w.bind) : read_string(app, resolve(w.bind));
      if (!w.emoji.empty() && emit_text_runs(w, label, val, false))
        break;
      float tint[3];
      if (item_color(w, tint))
        b.text_value_colored(label, val, tint);
      else
        b.text_value(label, val);
    }
    break;

  case Kind::Separator:
    b.separator();
    break;

  case Kind::Spinner: {
    // The host's frame clock turns it, whatever the document's programs are doing; disabled, it
    // stands still. A bind (a state path, or a computed expression) is its label.
    const std::string text = w.bind.empty()    ? w.label
                             : is_expr(w.bind) ? eval_text(w.bind)
                                               : read_string(app, resolve(w.bind));
    b.spinner(text.c_str(), dis ? 0.0 : frame_t, w.ascii_glyphs);
    break;
  }

  case Kind::Progress: {
    // Read-only: the number (a state path or a computed expression) placed in [lo, hi].
    const std::string v = is_expr(w.bind) ? eval_text(w.bind) : read_string(app, resolve(w.bind));

    const double x = v.empty() ? w.lo : std::strtod(v.c_str(), nullptr);
    const double span = w.hi - w.lo;
    const double f = span > 0.0 && std::isfinite(x) ? (x - w.lo) / span : 0.0;
    std::string overlay;
    if (!w.overlay.empty())
      overlay = is_expr(w.overlay) ? eval_text(w.overlay) : read_string(app, resolve(w.overlay));
    b.progress_bar(label, static_cast<float>(f < 0.0 ? 0.0 : (f > 1.0 ? 1.0 : f)), overlay);
    break;
  }

  case Kind::Plot: {
    // Read-only: the comma-separated numbers (a state path or a computed expression), oldest
    // first; anything that is not a number is skipped.
    const std::string v = is_expr(w.bind) ? eval_text(w.bind) : read_string(app, resolve(w.bind));
    std::vector<float> values;
    for (std::size_t i = 0; i <= v.size();) {
      const std::size_t j = std::min(v.find(',', i), v.size());
      const std::string item = v.substr(i, j - i);
      char *end = nullptr;
      const double x = std::strtod(item.c_str(), &end);
      if (end != item.c_str() && std::isfinite(x))
        values.push_back(static_cast<float>(x));
      i = j + 1;
    }
    std::string overlay;
    if (!w.overlay.empty())
      overlay = is_expr(w.overlay) ? eval_text(w.overlay) : read_string(app, resolve(w.overlay));
    b.plot_lines(label, values, w.lo, w.hi, overlay, w.plot_height);
    break;
  }

  case Kind::Chart: {
    ChartContext ctx{app,
                     b,
                     [this](const std::string &bind) { return resolve(bind); },
                     [this](const std::string &expr) { return eval_text(expr); },
                     [this](const std::string &event) { enqueue(event); },
                     [this](const std::string &msg) { warn_once(msg); },
                     chart_caches,
                     chart_frame};
    emit_chart(ctx, w);
    break;
  }

  case Kind::Heatmap: {
    HeatmapContext ctx{app,
                       b,
                       [this](const std::string &bind) { return resolve(bind); },
                       [this](const std::string &expr) { return eval_text_or_literal(expr); },
                       [this](const std::string &path) { return ensure_reactive().written(path); },
                       [this](const std::string &event, const HeatmapCellEvent &cell) {
                         enqueue_cell(event, cell);
                       },
                       [this](const std::string &msg) { warn_once(msg); },
                       heatmap_caches,
                       chart_frame};
    if (emit_heatmap(ctx, w)) // a cell's click is the heatmap's, not a clickable card's around it
      ++clicks_fired;
    break;
  }

  case Kind::Settings: {
    if (!settings_watch) {
      settings_watch = std::make_shared<SettingsWatch>();
      settings_watch_conn = cvc::state::instance(app).childChanged.connect(
          [watch = settings_watch](const std::string &path) { watch->note(path); });
    }
    SettingsContext ctx{app,
                        b,
                        [this](const std::string &bind) { return resolve(bind); },
                        [this](const std::string &event) { enqueue(event); },
                        [this](const std::string &msg) { warn_once(msg); },
                        *settings_watch,
                        settings_caches,
                        chart_frame};
    emit_settings(ctx, w);
    break;
  }

  case Kind::Checkbox: {
    const std::string path = resolve(w.bind);
    const bool cur =
        read_bool_or_seed(app, path, w.bdef ? 1 : 0) != 0; // tolerant of "true"/"false"
    const BoolEdit e = b.checkbox(label, cur);
    if (e.committed) {
      write<int>(app, path, e.value ? 1 : 0);
      if (!w.on.empty()) // fire AFTER the write so the handler sees the new value in state
        enqueue(w.on);
    }
    break;
  }

  case Kind::SliderInt: {
    const std::string path = resolve(w.bind);
    const int cur = read_or_seed<int>(app, path, w.idef);
    int lo = w.ilo, hi = w.ihi;
    if (w.range_from_key) {
      double klo = lo, khi = hi;
      key_range(app, path, klo, khi);
      lo = static_cast<int>(std::lround(klo));
      hi = std::max(lo, static_cast<int>(std::lround(khi)));
    }
    const IntEdit e = b.slider_int(label, path.c_str(), cur, lo, hi);
    if (e.committed || (e.changed && !w.commit_on_release)) // live while dragged, by default
      write<int>(app, path, e.value);
    if (e.committed && !w.on.empty()) // on: fires once per commit (drag end / key step), not
      enqueue(w.on);                  // every live frame of a drag
    break;
  }

  case Kind::SliderFloat: {
    const std::string path = resolve(w.bind);
    const double cur = read_or_seed<double>(app, path, w.def);
    double lo = w.lo, hi = w.hi;
    if (w.range_from_key)
      key_range(app, path, lo, hi);
    const DoubleEdit e = w.log_scale
                             ? b.slider_double_log(label, path.c_str(), cur, lo, hi, w.fmt.c_str())
                             : b.slider_double(label, path.c_str(), cur, lo, hi, w.fmt.c_str());
    if (e.committed || (e.changed && !w.commit_on_release))
      write<double>(app, path, e.value);
    if (e.committed && !w.on.empty())
      enqueue(w.on);
    break;
  }

  case Kind::Combo:
  case Kind::Radio: {
    // §4 read-lane: options are either a static list or a computed expression re-evaluated
    // each frame. A local holds the computed list so its lifetime spans the emit.
    std::vector<std::string> computed;
    if (!w.options_expr.empty())
      computed = eval_options(w.options_expr);
    const std::vector<std::string> &opts = w.options_expr.empty() ? w.options : computed;
    if (opts.empty())
      break;
    // A `values:` list (validated equal-length at load, static options only) maps each displayed
    // option to the value STORED in the key (e.g. an int); otherwise the key holds the option TEXT.
    const bool mapped =
        !w.values.empty() && w.options_expr.empty() && w.values.size() == opts.size();
    const std::vector<std::string> &store = mapped ? w.values : opts;
    const std::string path = resolve(w.bind);
    const std::string fallback = w.sdef.empty() ? store.front() : w.sdef;
    const std::string cur = read_or_seed<std::string>(app, path, fallback);
    int idx = 0;
    for (std::size_t i = 0; i < store.size(); ++i)
      if (store[i] == cur) { // match the STORED value (or text), display opts[idx]
        idx = static_cast<int>(i);
        break;
      }
    // §5.26 a radio's colour: one for all of it, or one per option.
    const std::vector<ItemColor> tint =
        w.kind == Kind::Radio ? item_colors(w) : std::vector<ItemColor>();
    IndexEdit e;
    if (w.kind != Kind::Radio)
      e = b.combo(label, idx, opts); // DISPLAY the labels
    else if (tint.size() > 1)
      e = b.radio_colored(label, idx, opts, w.radio_row, tint);
    else if (!tint.empty() && tint.front().set) {
      b.begin_text_color(tint.front().rgb);
      e = b.radio(label, idx, opts, w.radio_row);
      b.end_text_color();
    } else
      e = b.radio(label, idx, opts, w.radio_row);
    if (e.changed && e.index >= 0 && e.index < static_cast<int>(store.size())) {
      write<std::string>(app, path, store[e.index]); // store the mapped value (or the text)
      if (!w.on.empty())
        enqueue(w.on);
    }
    break;
  }

  case Kind::Tabs: {
    // The tabs: the children's titles, or (a bar alone) the options. `bind` holds the selected
    // tab's title (or, with values:, its value); without one the selection is kept per widget.
    std::vector<std::string> computed;
    if (w.children.empty() && !w.options_expr.empty())
      computed = eval_options(w.options_expr);
    const std::vector<std::string> &titles = !w.children.empty()      ? w.tab_titles
                                             : w.options_expr.empty() ? w.options
                                                                      : computed;
    if (titles.empty())
      break;
    const bool mapped = w.children.empty() && !w.values.empty() && w.options_expr.empty() &&
                        w.values.size() == titles.size();
    const std::vector<std::string> &store = mapped ? w.values : titles;
    const std::string id = w.id.empty() ? std::string("tabs:") + w.label : w.id;
    const std::string path = resolve(!w.bind.empty() ? w.bind : "tree." + id + ".tab");
    const std::string fallback = w.sdef.empty() ? store.front() : w.sdef;
    const std::string cur = read_or_seed<std::string>(app, path, fallback);
    int idx = 0;
    for (std::size_t i = 0; i < store.size(); ++i)
      if (store[i] == cur) {
        idx = static_cast<int>(i);
        break;
      }
    b.push_id(id.c_str());
    const IndexEdit e = b.tab_bar(label, id.c_str(), idx, titles);
    if (e.changed && e.index >= 0 && e.index < static_cast<int>(store.size()) && e.index != idx) {
      write<std::string>(app, path, store[e.index]);
      idx = e.index;
      if (!w.on.empty())
        enqueue(w.on);
    }
    if (!w.children.empty()) {
      WalkFrame after; // pop_id after the tab's subtree (leave_node)
      after.op = WalkFrame::Op::Node;
      after.w = &w;
      after.dis = dis;
      walk_.push_back(std::move(after));
      begin_emit(w.children[static_cast<std::size_t>(idx)]); // only the selected tab's subtree
      return;
    }
    b.pop_id();
    break;
  }

  case Kind::List: {
    // Rows of a state array: the count at bind, row i at bind.i, each showing its list_field.
    const std::string path = resolve(w.bind);
    const std::string cur_path =
        w.list_current.empty() ? path + ".current" : resolve(w.list_current);
    constexpr int kMaxRows = 100000;
    int count = 0;
    try {
      count = std::clamp(std::stoi(read_string(app, path)), 0, kMaxRows);
    } catch (...) {
      count = 0;
    }
    std::vector<std::string> items;
    items.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i)
      items.push_back(read_string(app, path + "." + std::to_string(i) + "." + w.list_field));
    int current = -1;
    try {
      current = std::stoi(read_string(app, cur_path));
    } catch (...) {
      current = -1;
    }
    if (current >= count)
      current = count - 1;
    if (current < -1)
      current = -1;
    ListOptions o;
    o.add = w.list_add;
    o.remove = w.list_remove;
    o.rows = w.list_rows;
    const std::string id = w.id.empty() ? std::string("list:") + path : w.id;
    b.push_id(id.c_str());
    const ListEdit e = b.list(label, id.c_str(), current, items, o);
    b.pop_id();
    cvc::state &root = cvc::state::instance(app);
    if (e.selected && e.index >= 0 && e.index < count && e.index != current) {
      write<std::string>(app, cur_path, std::to_string(e.index));
      current = e.index;
      if (!w.on.empty())
        enqueue(w.on);
    }
    if (e.add && w.list_add && count < kMaxRows) {
      // A new last row: the new_item fields ("{n}": its number from 1), then the count, then the
      // selection on it, so a watcher of the count sees the row's fields already there.
      const std::string row = path + "." + std::to_string(count);
      for (const auto &f : w.list_new) {
        std::string v = f.second;
        for (std::size_t at = v.find("{n}"); at != std::string::npos; at = v.find("{n}", at))
          v.replace(at, 3, std::to_string(count + 1));
        write<std::string>(app, row + "." + f.first, v);
      }
      if (w.list_new.empty())
        write<std::string>(app, row + "." + w.list_field, std::to_string(count + 1));
      write<std::string>(app, path, std::to_string(count + 1));
      write<std::string>(app, cur_path, std::to_string(count));
      if (!w.on_add.empty())
        enqueue(w.on_add);
    } else if (e.remove && w.list_remove && current >= 0 && current < count) {
      // The selected row goes: each row after it moves up one (its subtree copied over the one
      // before), the last row's subtree is deleted, and the count drops.
      for (int i = current; i + 1 < count; ++i) {
        cvc::state &dst = root(path + "." + std::to_string(i));
        cvc::state &src = root(path + "." + std::to_string(i + 1));
        const boost::property_tree::ptree values = src.ptree();
        dst.reset(false); // its children destroyed: the next row's fields replace them
        dst.ptree(values);
      }
      root(path).removeChild(std::to_string(count - 1));
      write<std::string>(app, path, std::to_string(count - 1));
      write<std::string>(app, cur_path, std::to_string(std::min(current, count - 2)));
      if (!w.on_remove.empty())
        enqueue(w.on_remove);
    }
    break;
  }

  case Kind::InputText: {
    const std::string path = resolve(w.bind);
    const std::string cur = read_or_seed<std::string>(app, path, w.sdef);
    const StringEdit e = b.input_text(label, path.c_str(), cur, w.password);
    note_text_field(b); // ARIADNE_SHELL.md 4.6 a field being typed into keeps keys from mounts
    if (e.committed) {
      write<std::string>(app, path, e.value);
      if (!w.on.empty()) // send-on-commit (e.g. a chat box's Enter) — fire AFTER the write so the
        enqueue(w.on);   // handler sees the committed value in state
    }
    break;
  }

  case Kind::MessageInput: {
    const std::string path = resolve(w.bind);
    const std::string cur = read_or_seed<std::string>(app, path, w.sdef);
    MessageInputOptions o;
    o.hint = w.hint;
    o.multiline = w.multiline;
    o.max_lines = w.max_lines;
    o.max_length = w.max_length;
    o.send = w.send;
    o.live = w.live || !w.on_change.empty();
    const MessageEdit e = b.message_input(label, path.c_str(), cur, o);
    note_text_field(b);
    if (e.submitted) {
      write<std::string>(app, path, std::string()); // sent: the draft is done with
      enqueue_fields(w.on, {{"text", e.value}});    // a program reads (get-attr event "text")
      enqueue_latest(path, w.on_change, {{"text", std::string()}});
    } else if (o.live && e.changed) {
      write<std::string>(app, path, e.value);                 // live: the draft as it is typed
      enqueue_latest(path, w.on_change, {{"text", e.value}}); // keys between drains: the last
    } else if (e.committed) {
      write<std::string>(app, path, e.value); // focus left: keep the draft
    }
    break;
  }

  case Kind::TextView: {
    // Read-only: the text at a state path, or a computed expression's. The backend keeps its
    // scroll position under the bound path, or the widget's place in the tree for an expression.
    const bool computed = is_expr(w.bind);
    std::string key = computed ? std::string() : resolve(w.bind);
    const std::string text = computed ? eval_text(w.bind) : read_string(app, key);
    if (computed) {
      key = current_prefix() + "#text_view";
      for (const std::size_t k : emit_path)
        key += "." + std::to_string(k);
    }
    TextViewOptions o;
    o.height = static_cast<float>(w.view_height);
    o.follow = w.follow;
    o.wrap = w.wrap;
    const TextViewEvent e = b.text_view(label, key.c_str(), text, o);
    if (e.at_top && !dis && !w.on_top.empty())
      enqueue(w.on_top);
    break;
  }

  case Kind::InputInt: {
    const std::string path = resolve(w.bind);
    const int cur = read_or_seed<int>(app, path, w.idef);
    const StringEdit e = b.input_text(label, path.c_str(), std::to_string(cur), false);
    note_text_field(b);
    if (e.committed) {
      int next = cur; // fail-safe: a non-numeric entry keeps the current value
      try {
        next = std::stoi(e.value);
      } catch (const std::exception &) {
      }
      if (w.ilo != w.ihi) { // clamp only when a real range was authored
        if (next < w.ilo)
          next = w.ilo;
        if (next > w.ihi)
          next = w.ihi;
      }
      write<int>(app, path, next);
      if (!w.on.empty())
        enqueue(w.on);
    }
    break;
  }

  case Kind::Color: {
    const std::string path = resolve(w.bind);
    const std::string dflt = w.sdef.empty() ? std::string("1,1,1") : w.sdef;
    const std::string cur = read_or_seed<std::string>(app, path, dflt);
    float rgb[3] = {1.0f, 1.0f, 1.0f};
    parse_rgb3(cur, rgb); // leaves the 1,1,1 default on a parse failure
    const ColorEdit e = w.fmt == "hex" ? b.color_hex(label, rgb) : b.color(label, rgb);
    if (!e.drawn) {
      b.text_value(label, cur); // a backend with no colour widget -> show the "r,g,b" value
      break;
    }
    if (e.committed) {
      write<std::string>(app, path, format_rgb3(e.rgb)); // store "r,g,b"
      if (!w.on.empty())
        enqueue(w.on);
    }
    break;
  }

  case Kind::Video:
  case Kind::Audio:
    emit_player(w, label); // §5.13
    break;

  case Kind::Image: {
    // §5.13 `fullscreen: true`: the picture is one item that takes focus, and a click on it (Enter,
    // in a terminal) shows it alone, filling the window. In that view a click on it leaves.
    const bool opens = w.fullscreen_control && !in_fullscreen && b.capabilities().fullscreen_view;
    if (opens) {
      ItemGroupOptions o;
      o.focusable = true;
      b.begin_item_group(item_id(w).c_str(), o);
    }
    // The image name: a bound key's value (a combo can switch rasters) else the static src. Either
    // may be a computed expression, and either may be a URI (§5.11), which the cache loads.
    const std::string src = is_expr(w.src) ? eval_text(w.src) : w.src;
    const std::string name =
        w.bind.empty() ? src
                       : (is_expr(w.bind) ? eval_text(w.bind)
                                          : read_or<std::string>(app, resolve(w.bind), src));
    if (media::is_video_uri(name)) { // §5.13 a video avatar or picture: a decorative video
      Widget vw = w;
      vw.kind = Kind::Video;
      vw.src = name;
      vw.bind.clear();
      vw.controls = false;
      vw.autoplay = vw.loop = vw.muted = true;
      vw.fullscreen_control = false;         // the picture's, above
      if (vw.img_h <= 0 && vw.img_fill <= 0) // one that fills the width takes its frames' shape
        vw.img_h = media::placeholder_height(static_cast<int>(vw.img_size), 0, vw.img_shape);
      emit_player(vw, label);
    } else if (media::is_uri(name)) {
      emit_media_image(w, name, label);
    } else {
      // A host-published picture: in the fullscreen view, fitted to it; else at its width.
      const bool drew = in_fullscreen
                            ? b.draw_image_fit(name.c_str(), static_cast<float>(w.img_size),
                                               static_cast<float>(w.img_h))
                            : b.draw_image(name.c_str(), static_cast<float>(fill_width(w)));
      if (!drew) // a backend with no image path -> show the name as text
        b.text_value(w.label.empty() ? "image" : w.label.c_str(),
                     name.empty() ? std::string("(no image)") : name);
    }
    if (!media::is_video_uri(name))
      emit_badge(w); // a video URL's player drew its own
    if (opens) {
      b.end_item_group();
      if (b.item_clicked()) {
        open_fullscreen(w);
        ++clicks_fired; // the picture's, not a clickable card's around it
      }
    } else if (in_fullscreen && b.item_clicked()) {
      fullscreen_exit = true;
    }
    break;
  }

  case Kind::Button: {
    // §5.27 a picture in place of the label, and the chosen one of a row drawn pressed in. A plain
    // button goes the way it always has.
    bool hit = false;
    if (w.button_icon.empty() && w.selected.empty()) {
      hit = tinted([&] { return b.button(label); });
    } else {
      ButtonLook look;
      look.icon_h = w.icon_size;
      look.selected = w.selected == "true" || (is_expr(w.selected) && predicate(w.selected, false));
      const std::string icon = is_expr(w.button_icon) ? eval_text(w.button_icon) : w.button_icon;
      if (media::is_uri(icon) && !media::is_video_uri(icon)) {
        // Fetched and kept as an image:'s picture is, at its own shape: the label shows until it is
        // ready, and if it fails. Three times the height it's drawn (a line, about 16 px, unless
        // icon_size: says), so a phone's dense screen gets a sharp one.
        media::ImageRequest rq;
        rq.uri = icon;
        rq.auth = w.media_auth;
        rq.w = 3 * std::max(16, static_cast<int>(std::lround(w.icon_size)));
        media::ImageView v;
        const std::string name = media_picture(rq, v);
        if (v.status == media::Status::Ready)
          look.icon = name;
      } else if (!icon.empty() && !media::is_uri(icon)) {
        look.icon = icon; // a host-published picture
      }
      hit = tinted([&] { return b.button_with(label, look); });
    }
    if (hit) {
      enqueue(w.on);
      ++clicks_fired; // the click is the button's, not a group's around it (§4.6)
    }
    break;
  }

  case Kind::TfEditor: {
    TfEditorContext ctx{app,
                        b,
                        [this](const std::string &bind) { return resolve(bind); },
                        [this](const std::string &event) { enqueue(event); },
                        [this](const std::string &msg) { warn_once(msg); },
                        tf_caches};
    emit_tf_editor(ctx, w);
    break;
  }

  case Kind::Custom: {
    // Three ways a custom widget renders, in priority order:
    //  1. a COMPOSITIONAL fn (register_widget_type) — composes built-ins through a
    //     context that reuses the core's binding; works on every backend;
    //  2. else the BACKEND's novel-primitive escape (custom_widget) — the core reads
    //     the bound value, hands it in, and writes back on the committed edge;
    //  3. else a placeholder (neither knows this type).
    // §5.30 its right-click menu is over all of it; a scene view (a viewport) adds its pick.
    if (w.context_menu)
      b.begin_context_area(context_id(w).c_str(), ContextMenuOver::Item);
    Runtime::ContextPick pick;
    if (const WidgetEmitFn fn = lookup_widget(app, w.custom_type)) {
      WidgetEmitContext ctx;
      // A child is drawn where the fn asks for it, between its own calls: a walk of its own, which
      // returns once the child's subtree is drawn. (Only this nests natively, a level per custom
      // widget whose fn draws a custom widget.)
      ctx.emit = [this](const Widget &child) { emit(child); };
      ctx.read = [this](const std::string &bind) { return read_string(app, resolve(bind)); };
      ctx.fire = [this](const std::string &event) { enqueue(event); };
      fn(w, ctx);
    } else {
      const std::string path = w.bind.empty() ? std::string() : resolve(w.bind);
      const std::string cur = w.bind.empty() ? std::string() : read_string(app, path);
      b.set_custom_scope(current_prefix()); // where its bind: resolved, for keys of its own
      CustomEdit e = b.custom_widget(w.custom_type.c_str(), cur, w);
      b.set_custom_scope(std::string());
      if (!e.handled)
        b.text_line(("[" + w.custom_type + "?]").c_str()); // neither composed nor backend-drawn
      else if (e.committed && !w.bind.empty())
        write<std::string>(app, path, e.value);
      if (e.handled && !e.fire.empty()) // an event it raised, with its fields
        enqueue_fields(e.fire, std::move(e.fields));
      for (CustomEdit::Event &ev : e.events) // and any more, in order
        if (e.handled && ev.document)
          enqueue_document_fields(ev.fire, std::move(ev.fields));
        else if (e.handled)
          enqueue_fields(ev.fire, std::move(ev.fields));
      if (e.handled)
        pick = std::move(e.context_pick);
    }
    if (w.context_menu) {
      b.end_context_area();
      WalkFrame after; // the rest of the widget after its menu (leave_node)
      after.op = WalkFrame::Op::Node;
      after.w = &w;
      after.dis = dis;
      walk_.push_back(std::move(after));
      emit_context_menu(w, ContextMenuOver::Item, pick);
      return;
    }
    break;
  }
  }
  emit_node_tail(w, dis, inner_click);
}

// What every widget does after it is drawn: its pointer handlers and tooltip, on the item just
// drawn, then the disabled scope ends.
void Runtime::Impl::emit_node_tail(const Widget &w, bool dis, bool inner_click) {
  Backend &b = *backend;
  // §4.6 widget-level on_click: if this widget carries an on_click and the backend reports the item
  // just drawn was clicked, enqueue it — a fire-once action drained + submitted like a Button's on:
  // (a program on: runs through state_exec; a bare name routes to a host handler). Sourced from the
  // backend's per-item click (ImGui::IsItemClicked), so it works wherever ImGui gets input (native
  // VTK-interactor AND wasm) — no raw hit-test. Defaults off on a backend without per-item input.
  // Inside the disabled scope (below), so a disabled widget never fires (IsItemClicked is false).
  // §4.6 widget-level pointer handlers, each fired when the backend reports the matching per-item
  // state (a program runs through state_exec with an `event` dict of the item's pointer coords; a
  // bare name routes to a host handler). on_click is a one-shot edge; on_hover/on_drag are
  // continuous; on_drag_start/on_drag_end are the gesture edges (activate/deactivate). Sourced from
  // the backend's per-item state so it works native (VTK-interactor) AND wasm, no raw hit-test. The
  // backend query is short-circuited when the handler is empty, and a disabled widget fires none
  // (the backend reports false while disabled). item_pointer fills the event coords once per fire.
  auto fire_pointer = [&](const std::string &handler) {
    float x, y, dx, dy;
    int btn;
    b.item_pointer(x, y, dx, dy, btn);
    enqueue_event(handler, x, y, dx, dy, btn);
  };
  // A group's click is the innermost taker's: a widget inside it with an on_click of its own (an
  // avatar opening a profile in a card opening a post), or a button, keeps it.
  // A heatmap's on_click, on_hover and tooltip are per cell, its own (emit_heatmap).
  const bool own_pointer = w.kind == Kind::Heatmap;
  if (!own_pointer && !w.on_click.empty() && !inner_click && b.item_clicked()) {
    fire_pointer(w.on_click);
    ++clicks_fired;
  }
  if (!own_pointer && !w.on_hover.empty() && b.item_hovered())
    fire_pointer(w.on_hover);
  // on_hover_end: kept while the item is hovered, run once the walk after it is not (render()).
  if (!w.on_hover_end.empty() && b.item_hovered())
    hovered_now[instance_key()] = HoverEnd{w.on_hover_end, current_prefix(), current_functions()};
  if (!w.on_drag.empty() && b.item_dragged())
    fire_pointer(w.on_drag);
  if (!w.on_drag_start.empty() && b.item_drag_started())
    fire_pointer(w.on_drag_start);
  if (!w.on_drag_end.empty() && b.item_drag_ended())
    fire_pointer(w.on_drag_end);
  // §4 read-lane: attach a hover tooltip to the widget just drawn (the backend's last item).
  // A tooltip is free human text, so — unlike a bind (a dotted state path never begins with
  // '(') — one that merely LOOKS like an expression ("(optional) …", "(beta)") must not be
  // mistaken for one. So: try to evaluate an expr-looking tooltip, but if it does NOT evaluate
  // cleanly, fall back to showing it verbatim (no spurious warning). A genuine computed tooltip
  // still works; a literal that happens to start with '(' is shown as-is.
  if (!own_pointer && !w.tooltip.empty()) {
    std::string tip = w.tooltip;
    if (is_expr(w.tooltip))
      tip = eval_text_or_literal(w.tooltip);
    if (!tip.empty())
      b.set_tooltip(tip.c_str());
  }
  if (dis)
    b.end_disabled();
}

// emit_node after a container's children (or a custom widget's menu): what its case did after
// them, then emit_node_tail. A step that opens a context menu comes back at the next stage.
void Runtime::Impl::leave_node(WalkFrame &f) {
  Backend &b = *backend;
  const Widget &w = *f.w;
  bool inner_click = false;
  switch (w.kind) {
  case Kind::Menubar: // its menus, the Windows menu and the tray are drawn
    first_bar_drawn = true;
    --menubar_depth;
    b.end_main_menu_bar();
    break;
  case Kind::Menu:
    --menu_depth;
    if (f.flag && windows_menu.on && !windows_menu_drawn && windows_menu_has_items()) {
      b.separator();
      emit_windows_menu_items();
    }
    b.end_menu();
    if (f.flag)
      windows_menu_drawn = true; // open or not, it stands for the Windows menu this frame
    break;
  case Kind::Window:
    if (f.stage == 0) {
      if (f.backdrop)
        b.end_backdrop();
      if (w.context_menu) {
        b.end_context_area();
        f.stage = 1;
        walk_.push_back(std::move(f));
        emit_context_menu(w, ContextMenuOver::Window, {});
        return;
      }
    }
    b.pop_id();
    b.end_window();
    if (w.closable)
      write<int>(app, f.path, f.flag ? 1 : 0); // the close (X) button may have cleared `open`
    break;
  case Kind::Group:
    if (f.stage == 0) {
      if (f.backdrop)
        b.end_backdrop();
      f.flag = clicks_fired != f.count; // something inside it took this walk's click
      if (f.item)
        b.end_item_group();
      if (w.context_menu && &w != &root) {
        b.end_context_area();
        f.stage = 1;
        walk_.push_back(std::move(f));
        emit_context_menu(w, ContextMenuOver::Item, {});
        return;
      }
    }
    inner_click = f.flag;
    if (!w.id.empty())
      b.pop_id();
    break;
  case Kind::Tabs:
    b.pop_id();
    break;
  default: // Kind::Custom, after its menu
    break;
  }
  emit_node_tail(w, f.dis, inner_click);
}

namespace {

double steady_seconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// A string as a state_exec literal: "..." with \ and " escaped.
std::string program_string(const std::string &s) {
  std::string out = "\"";
  for (char c : s) {
    if (c == '\\' || c == '"')
      out += '\\';
    out += c;
  }
  return out + "\"";
}

} // namespace

namespace {

// A width that fills a line (§5.11 `size: "N%"`) changes while a window is resized; its raster is
// fitted at the next multiple of 64 px up, so a resize refits it every 64 px, not every frame, and
// the backend scales it the rest of the way.
int fill_box(int px) { return std::max(64, (px + 63) / 64 * 64); }

// The width a picture `draw_w` px wide is drawn at when its height follows its shape (`aspect`,
// height / width; 0 unknown) and must stay within `max_h` px (0: no bound): narrower, not cut.
float within_height(float draw_w, float aspect, double max_h) {
  if (!(max_h > 0.0) || !(aspect > 0.0f))
    return draw_w;
  return std::min(draw_w, static_cast<float>(max_h) / aspect);
}

} // namespace

void Runtime::Impl::emit_badge(const Widget &w) {
  if (w.badge.empty() || in_fullscreen)
    return;
  // Free text, like a tooltip: an expression gives it, and one that only looks like one is shown.
  const std::string text = is_expr(w.badge) ? eval_text_or_literal(w.badge) : w.badge;
  if (!text.empty())
    backend->item_badge(text.c_str());
}

int Runtime::Impl::fill_width(const Widget &w) {
  const double own = w.img_size > 0 ? w.img_size : 256.0;
  if (!(w.img_fill > 0.0))
    return std::max(1, static_cast<int>(std::lround(own)));
  const float avail = backend->available_width(static_cast<float>(own * 100.0 / w.img_fill));
  if (!(avail > 0.0f))
    return std::max(1, static_cast<int>(std::lround(own)));
  return std::max(1, static_cast<int>(std::floor(avail * w.img_fill / 100.0)));
}

std::string Runtime::Impl::media_picture(const media::ImageRequest &rq, media::ImageView &v) {
  v = image_cache().image(rq);
  drew_loading = drew_loading || v.status == media::Status::Loading;
  std::string name;
  if (v.status == media::Status::Ready) {
    name = v.name;
    if (v.animation) {
      // An animated GIF or WebP: the frame for now on a loop every widget showing it shares, a
      // new one published as it comes (8 a second at most in a terminal).
      const double now =
          std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
              .count();
      const std::size_t i = v.animation->frame_at(now);
      auto it = anim_shown.find(name);
      const double gap = backend->capabilities().terminal ? 1.0 / 8.0 : 0.0;
      if (it == anim_shown.end() || published.count(name) == 0 ||
          (it->second.index != i && now - it->second.at >= gap)) {
        backend->set_image(name, v.animation->frames[i]);
        published[name] = v.gen;
        anim_shown[name] = AnimShown{i, now};
      }
    } else {
      publish_media(v);
    }
  } else {
    // One placeholder per box shape and state, shared by every widget with that box.
    const bool failed = v.status == media::Status::Failed;
    const int ph = media::placeholder_height(rq.w, rq.h, rq.shape);
    name = std::string("media:placeholder:") + (failed ? "failed:" : "loading:") +
           std::to_string(rq.w) + "x" + std::to_string(ph) +
           (rq.shape == ImageShape::Circle ? ":circle" : "");
    if (!published.count(name)) {
      backend->set_image(name, media::placeholder_image(rq.w, rq.h, rq.shape, failed));
      published[name] = 0;
    }
  }
  return name;
}

// §5.11: draw a picture by URI — the cache's fitted raster once it is ready, else a placeholder
// of the same box — and report its state to the widget's status: path.
void Runtime::Impl::emit_media_image(const Widget &w, const std::string &uri, const char *label) {
  const int draw_w = fill_width(w);
  const bool fill = w.img_fill > 0.0;
  media::ImageRequest rq;
  rq.uri = uri;
  rq.auth = w.media_auth;
  rq.w = fill ? fill_box(draw_w) : draw_w;
  // A box keeps its height however the width is rounded: the raster's is scaled with it.
  rq.h = w.img_h > 0 ? std::max(1, static_cast<int>(std::lround(w.img_h * rq.w / draw_w))) : 0;
  rq.fit = w.img_fit;
  rq.shape = w.img_shape;
  media::ImageView v;
  const std::string name = media_picture(rq, v);
  // Its height follows its shape (16:9 until it is known), within max_height:.
  float aspect = 0.0f;
  if (rq.h <= 0)
    aspect = v.status == media::Status::Ready && v.image.width() > 0
                 ? static_cast<float>(v.image.height()) / static_cast<float>(v.image.width())
                 : static_cast<float>(media::placeholder_height(rq.w, 0, rq.shape)) /
                       static_cast<float>(rq.w);
  const float width = within_height(static_cast<float>(draw_w), aspect, w.img_max_h);
  const std::string caption = (label && *label) ? std::string(label) : std::string("image");
  if (!backend->draw_image(name.c_str(), width)) // a backend with no image path: say what it is
    backend->text_value(caption.c_str(),
                        v.status == media::Status::Ready
                            ? uri
                            : std::string("(") + media::status_name(v.status) + ")");
  else
    offer_save(w, uri, /*video=*/false); // §5.29 Save image... in its context menu
  if (v.status == media::Status::Failed)
    backend->set_tooltip((caption + ": " + v.error).c_str());
  write_media_status(w.media_status, v);
}

bool Runtime::Impl::expand_emoji(const std::string &value, const std::string &map,
                                 bool pictures_here, std::vector<TextRun> &out, std::string &plain,
                                 bool &pictures, unsigned style, int link) {
  if (value.find(':') == std::string::npos)
    return false;
  // A `:name:` as txwtf, Slack and Mastodon find one: 1-64 of [A-Za-z0-9_+-] between two colons
  // that aren't glued to a word character outside them, so the :50: in 14:50:47 stays a time.
  const auto word = [](char c) {
    const unsigned char u = static_cast<unsigned char>(c);
    return (u < 0x80 && std::isalnum(u)) || c == '_';
  };
  const auto name_char = [&](char c) { return word(c) || c == '+' || c == '-'; };
  const std::size_t first = out.size();
  const std::size_t plain_at = plain.size();
  std::string text; // the run being built
  bool changed = false;
  const auto text_run = [&](std::string t) {
    TextRun r;
    r.text = std::move(t);
    r.style = style;
    r.link = link;
    out.push_back(std::move(r));
  };
  std::size_t i = 0;
  while (i < value.size()) {
    if (value[i] == ':' && (i == 0 || !word(value[i - 1]))) {
      std::size_t j = i + 1;
      while (j < value.size() && j - i <= 65 && name_char(value[j]))
        ++j;
      const std::size_t len = j - i - 1;
      if (j < value.size() && value[j] == ':' && len >= 1 && len <= 64 &&
          (j + 1 == value.size() || !word(value[j + 1]))) {
        const std::string name = value.substr(i + 1, len);
        std::string to = read_or<std::string>(app, map + "." + name, std::string());
        if (to.empty()) { // the map's names are case-insensitive, as txwtf's are
          std::string lower = name;
          for (char &c : lower)
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
          if (lower != name)
            to = read_or<std::string>(app, map + "." + lower, std::string());
        }
        if (media::is_uri(to) && pictures_here) {
          std::string pic;
          if (media::is_video_uri(to)) {
            // A short video, as some sites allow for an emoji: silent, looping, playing by itself
            // (a decorative video, §5.13), its frames one line tall. One player per line and URI.
            std::string key = "emoji";
            for (std::size_t k = 0; k < emit_path.size(); ++k)
              key += "_" + std::to_string(emit_path[k]);
            key += "#" + to;
            media::PlayerSource ps;
            ps.uri = to;
            ps.video = true;
            ps.frame_w = ps.frame_h = 64;
            ps.loop = ps.muted = true;
            ps.audio = false;
            PlayerSlot &p = player_slot(key, ps);
            if (p.player && !p.autoplay_done && p.state == media::PlayerState::Ready) {
              p.autoplay_done = true;
              p.player->play();
            }
            if (player_frame(p, 64, 0, ImageFit::Contain, ImageShape::Rect, steady_seconds())) {
              pic = p.frame_name;
            } else {
              const bool failed = p.state == media::PlayerState::Failed;
              pic = std::string("media:placeholder:") + (failed ? "failed:" : "loading:") + "32x32";
              if (!published.count(pic)) {
                backend->set_image(pic, media::placeholder_image(32, 32, ImageShape::Rect, failed));
                published[pic] = 0;
              }
            }
          } else {
            // A picture one line tall: fitted at 32 px wide (64 on a HiDPI screen), its own
            // shape; an animated GIF or WebP moves, as an image:'s does.
            media::ImageRequest rq;
            rq.uri = to;
            rq.w = 32;
            media::ImageView v;
            pic = media_picture(rq, v);
          }
          if (!text.empty())
            text_run(std::move(text));
          text.clear();
          TextRun r;
          r.text = value.substr(i, len + 2);
          r.image = pic;
          r.style = style;
          r.link = link;
          out.push_back(std::move(r));
          plain += value.substr(i, len + 2);
          changed = pictures = true;
          i = j + 1;
          continue;
        }
        if (!to.empty() && !media::is_uri(to)) { // text, such as a unicode emoji's character
          text += to;
          plain += to;
          changed = true;
          i = j + 1;
          continue;
        }
      }
    }
    text += value[i];
    plain += value[i++];
  }
  if (!changed) {
    out.resize(first);
    plain.resize(plain_at);
    return false;
  }
  if (!text.empty())
    text_run(std::move(text));
  return true;
}

bool Runtime::Impl::emit_text_runs(const Widget &w, const char *label, const std::string &value,
                                   bool literal) {
  // A terminal can't put a picture in a line: its pictures aren't fetched (or played) at all.
  const bool pictures_here = !backend->capabilities().terminal;
  std::vector<TextRun> runs;
  std::string plain; // the line with the text the map gives, its pictures as their :name:
  bool pictures = false;
  if (!expand_emoji(value, resolve(w.emoji), pictures_here, runs, plain, pictures, 0u, -1))
    return false;
  float tint[3];
  const float *rgb = item_color(w, tint) ? tint : nullptr; // §5.26
  if (pictures) {
    if (label && *label)
      runs.insert(runs.begin(), TextRun{std::string(label) + ": ", std::string()});
    if (backend->text_runs(runs, rgb))
      return true;
  }
  // No pictures, or a backend that can't put one in a line: the text, the map's text in it.
  if (literal && rgb)
    backend->text_line_colored(plain.c_str(), rgb);
  else if (literal)
    backend->text_line(plain.c_str());
  else if (rgb)
    backend->text_value_colored(label, plain, rgb);
  else
    backend->text_value(label, plain);
  return true;
}

namespace {

// §5.20 the runs a state path holds: a list as state-set-tree writes one (its node holds the
// count, its elements are children 0..n-1, each a dict of text / style / href / tooltip or a
// plain value). A node with no elements holds none: an empty list is a "0" with no children,
// which is not a run of text. Read through links.
std::vector<TextRunSpec> read_runs(cvc::app &app, const std::string &path, std::size_t cap) {
  std::vector<TextRunSpec> out;
  try {
    cvc::state *n = read_node(app, path);
    if (!n || n->numChildren() == 0)
      return out;
    const auto field = [](cvc::state *e, const char *key) {
      cvc::state *f = e->findThroughLinks(key);
      return f ? f->value() : std::string();
    };
    for (std::size_t i = 0; i < cap; ++i) {
      cvc::state *e = n->findThroughLinks(std::to_string(i));
      if (!e)
        break;
      TextRunSpec r;
      if (e->numChildren() == 0) {
        r.text = e->value();
      } else {
        r.text = field(e, "text");
        r.style = field(e, "style");
        r.href = field(e, "href");
        r.tooltip = field(e, "tooltip");
      }
      out.push_back(std::move(r));
    }
  } catch (const std::exception &) {
  }
  return out;
}

} // namespace

std::vector<TextRunSpec> Runtime::Impl::eval_runs(const std::string &expr) {
  const ReactiveEngine::RunsOutcome o = ensure_reactive().eval_runs(expr);
  if (o.skipped) { // last frame's runs (none: empty), not a warning
    ++reactive_skips;
    const auto it = last_good_runs.find(expr);
    return it != last_good_runs.end() ? it->second : std::vector<TextRunSpec>();
  }
  if (!o.error.empty())
    warn_once(o.error);
  else
    keep_last(last_good_runs, expr, o.value);
  return o.value;
}

void Runtime::Impl::emit_rich_text(const Widget &w, const char *label, bool &inner_click) {
  // The document's own list (whose runs may run on_click:), or the runs a state path or an
  // expression gives (data: their links take on_link:).
  const bool literal = w.runs_src.empty();
  std::vector<TextRunSpec> computed;
  if (!literal)
    computed =
        is_expr(w.runs_src) ? eval_runs(w.runs_src) : read_runs(app, resolve(w.runs_src), kMaxRuns);
  const std::vector<TextRunSpec> &specs = literal ? w.runs : computed;
  constexpr std::size_t kMaxBytes = 256 * 1024; // a paragraph's text, all its runs together
  const std::string map = w.emoji.empty() ? std::string() : resolve(w.emoji);
  const bool pictures_here = !backend->capabilities().terminal;
  std::vector<TextRun> runs;
  std::vector<std::size_t> link_spec; // per link: the run it is
  RichTextOptions o;
  if (label && *label)
    runs.push_back(TextRun{std::string(label) + ": ", std::string()});
  std::size_t bytes = 0;
  for (std::size_t i = 0; i < specs.size(); ++i) {
    const TextRunSpec &s = specs[i];
    if (s.text.empty())
      continue;
    if ((bytes += s.text.size()) > kMaxBytes) {
      warn_once("ari: text '" + w.label + "': its runs are over 256 KiB -- the rest is left out");
      break;
    }
    int link = -1;
    if (!s.href.empty() || (literal && !s.on_click.empty())) {
      link = static_cast<int>(link_spec.size());
      link_spec.push_back(i);
      o.link_tips.push_back(s.tooltip.empty() ? s.href : s.tooltip);
    }
    const unsigned style = parse_text_style(s.style);
    std::string plain;
    bool pictures = false;
    if (map.empty() ||
        !expand_emoji(s.text, map, pictures_here, runs, plain, pictures, style, link)) {
      TextRun r;
      r.text = s.text;
      r.style = style;
      r.link = link;
      runs.push_back(std::move(r));
    }
  }
  float tint[3];
  o.rgb = item_color(w, tint) ? tint : nullptr; // §5.26
  o.wrap = w.wrap;
  o.max_lines = w.text_max_lines;
  if (w.size.w.unit == Unit::Px && w.size.w.value > 0.0f)
    o.max_width = w.size.w.value; // size: N
  o.links = static_cast<int>(link_spec.size());
  const RichTextEvent ev = backend->rich_text(runs, o);
  if (ev.clicked < 0 || ev.clicked >= o.links)
    return;
  // The link's: a run's own on_click (the document's list), else the text's on_link, with what
  // the link is in `event`.
  const std::size_t at = link_spec[static_cast<std::size_t>(ev.clicked)];
  const TextRunSpec &s = specs[at];
  const std::string &handler = literal && !s.on_click.empty() ? s.on_click : w.on_link;
  if (handler.empty())
    warn_once("ari: a link in text '" + w.label + "' was activated, but it has no on_link:");
  else
    enqueue_fields(handler, {{"href", s.href},
                             {"text", s.text},
                             {"index", std::to_string(at)},
                             {"link", std::to_string(ev.clicked)}});
  ++clicks_fired; // the link's, not the text's own on_click nor a clickable card's around it
  inner_click = true;
}

void Runtime::Impl::publish_media(const media::ImageView &v) {
  auto it = published.find(v.name);
  if (it == published.end() || it->second != v.gen) {
    backend->set_image(v.name, v.image);
    published[v.name] = v.gen;
  }
}

void Runtime::Impl::write_media_status(const std::string &status_path, const media::ImageView &v) {
  if (status_path.empty())
    return;
  const std::string path = resolve(status_path);
  const std::string st = media::status_name(v.status);
  std::string &seen = media_status_written[path];
  if (seen != st) {
    seen = st;
    write<std::string>(app, path, st);
    write<std::string>(app, path + ".error", v.error);
  }
}

bool Runtime::Impl::open_backdrop(const Widget &w, bool window, bool cell) {
  const Background &bg = w.background;
  if (!bg.any())
    return false;
  Backdrop bd;
  bd.fit = bg.fit;
  bd.opacity = bg.opacity;
  bd.terminal = bg.terminal;
  bd.banner_rows = bg.banner_rows;
  bd.window = window;
  bd.size = w.size;
  bd.cell = cell;
  // §5.26 the fill: written in the document, or its key's or expression's this frame.
  if (!bg.color_src.empty()) {
    const std::vector<ItemColor> c = eval_colors(bg.color_src);
    if (!c.empty() && c.front().set) {
      bd.has_fill = true;
      std::copy(c.front().rgb, c.front().rgb + 3, bd.fill);
    }
  } else if (bg.color.set) {
    bd.has_fill = true;
    std::copy(bg.color.rgb, bg.color.rgb + 3, bd.fill);
  }
  if (!bg.has_picture()) {
    if (!bd.has_fill)
      return false; // no colour this frame: the content alone
    backend->begin_backdrop(bd);
    return true;
  }
  const std::string src = is_expr(bg.src) ? eval_text(bg.src) : bg.src;
  const std::string ref =
      bg.bind.empty() ? src
                      : (is_expr(bg.bind) ? eval_text(bg.bind)
                                          : read_or<std::string>(app, resolve(bg.bind), src));
  if (media::is_video_uri(ref)) {
    // §5.13 a video background (a banner, a card's back): a silent player looping by itself, its
    // frames at their own shape, fitted by the backend as a picture is.
    std::string key = "bg";
    for (std::size_t i = 0; i < emit_path.size(); ++i)
      key += "_" + std::to_string(emit_path[i]);
    key += "#" + ref;
    media::PlayerSource ps;
    ps.uri = ref;
    ps.auth = bg.auth;
    ps.video = true;
    ps.frame_w = 960;
    ps.frame_h = 540;
    ps.loop = ps.muted = true;
    ps.audio = false;
    PlayerSlot &p = player_slot(key, ps);
    if (p.player && !p.autoplay_done && p.state == media::PlayerState::Ready) {
      p.autoplay_done = true;
      p.player->play();
    }
    if (player_frame(p, 960, 0, ImageFit::Contain, ImageShape::Rect, steady_seconds()))
      bd.image = p.frame_name;
    if (!bg.status.empty()) {
      const std::string path = resolve(bg.status);
      const std::string st = media::player_state_name(p.state);
      std::string &seen = media_status_written[path];
      if (seen != st) {
        seen = st;
        write<std::string>(app, path, st);
        write<std::string>(app, path + ".error", p.error);
      }
    }
  } else if (media::is_uri(ref)) {
    // The picture at its own shape (the backend fits it to the area the content takes), no larger
    // than the cache's cap.
    media::ImageRequest rq;
    rq.uri = ref;
    rq.auth = bg.auth;
    rq.w = std::max(1, image_cache().limits().max_image_px / 2);
    rq.h = 0;
    const media::ImageView v = image_cache().image(rq);
    drew_loading = drew_loading || v.status == media::Status::Loading;
    if (v.status == media::Status::Ready) {
      publish_media(v);
      bd.image = v.name;
    }
    write_media_status(bg.status, v);
  } else {
    bd.image = ref; // a host-published image, by name
  }
  backend->begin_backdrop(bd);
  return true;
}

Runtime::Impl::PlayerSlot &Runtime::Impl::player_slot(const std::string &key,
                                                      const media::PlayerSource &ps) {
  PlayerSlot &p = players[key];
  p.drawn_walk = walk_count;
  p.last_drawn = steady_seconds();
  p.hidden_paused = false; // back on screen: it stays paused; the user (or the document) plays it
  // A new source: a new player.
  if (p.uri != ps.uri || p.auth != ps.auth) {
    p.player.reset();
    p.decided = false;
    p.uri = ps.uri;
    p.auth = ps.auth;
    p.have_frame = false;
    p.has_pending = false;
    p.autoplay_done = p.autoplay_owned = p.offscreen_paused = false;
    p.aspect = 0.0f;
    p.error.clear();
    p.frame_name = "media:player:" + std::to_string(std::hash<std::string>()(key));
  }
  if (!p.decided) {
    p.decided = true;
    if (ps.uri.empty()) {
      p.state = media::PlayerState::Idle;
    } else if (!media::have_player_factory(app)) {
      p.state = media::PlayerState::Unsupported;
      p.error = "no player in this host (ari-media and arigl play it)";
    } else if (ps.audio && !media::may_play_here(app)) { // silent pictures play anywhere
      p.state = media::PlayerState::Remote;
      p.error = media::in_ssh_session() && media::play_policy(app) == media::PlayPolicy::Auto
                    ? "not played over SSH (it would play remotely)"
                    : "playing is off here (--media-play never)";
    } else {
      media::PlayerSource src = ps;
      src.base = media_base;
      p.player = media::make_player(app, src);
      p.made_at = steady_seconds();
      p.state = p.player ? media::PlayerState::Loading : media::PlayerState::Unsupported;
      if (!p.player)
        p.error = "this host's player cannot play it";
    }
  }
  if (media::Player *pl = p.player.get()) {
    p.state = pl->state();
    p.error = p.state == media::PlayerState::Failed ? pl->error() : std::string();
  }
  drew_loading = drew_loading || p.state == media::PlayerState::Loading;
  return p;
}

bool Runtime::Impl::player_frame(PlayerSlot &p, int bw, int bh, ImageFit fit, ImageShape shape,
                                 double now) {
  if (media::Player *pl = p.player.get()) {
    cvc::image f;
    if (pl->frame(f, p.frame_gen)) {
      p.pending = media::fit_image(f, bw, bh > 0 ? bh : 0, fit, shape, 1.0);
      p.has_pending = !p.pending.empty();
      if (p.has_pending)
        p.aspect = static_cast<float>(p.pending.height()) / static_cast<float>(p.pending.width());
    }
    // At most 8 frames a second to a terminal (each is a picture sent).
    const double min_gap = backend->capabilities().terminal ? 1.0 / 8.0 : 0.0;
    if (p.has_pending && now - p.last_publish >= min_gap) {
      backend->set_image(p.frame_name, p.pending);
      published[p.frame_name] = ++frame_publish_gen;
      p.have_frame = true;
      p.has_pending = false;
      p.last_publish = now;
    }
  }
  return p.have_frame;
}

void Runtime::Impl::emit_player(const Widget &w, const char *label) {
  Backend &b = *backend;
  const bool video = w.kind == Kind::Video;
  const std::string uri = is_expr(w.src) ? eval_text(w.src) : w.src;
  const std::string caption =
      (label && *label) ? std::string(label) : std::string(video ? "video" : "audio");
  // The instance: its state prefix where it has one (a feed binds each post's own), else where it
  // sits and what it plays.
  std::string key;
  if (!w.bind.empty() && !is_expr(w.bind)) {
    key = resolve(w.bind);
  } else {
    for (std::size_t i = 0; i < emit_path.size(); ++i)
      key += (i ? "_" : "") + std::to_string(emit_path[i]);
    key += "#" + uri;
  }
  const std::string prefix =
      !w.bind.empty() && !is_expr(w.bind)
          ? key
          : resolve("ariadne.media.players.p" +
                    std::to_string(std::hash<std::string>()(key) % 1000000007));
  // The picture's box: its size:, or (§5.11 "N%") as wide as its share of the line, its frames
  // fitted at that width rounded up to 64 px and drawn at the width itself.
  const int draw_w = fill_width(w);
  const int bw = w.img_fill > 0.0 ? fill_box(draw_w) : draw_w;
  const int bh =
      w.img_h > 0 ? std::max(1, static_cast<int>(std::lround(w.img_h * bw / draw_w))) : 0;
  // A decorative picture (controls: false, muted) never makes a sound, so it opens no audio device
  // and plays wherever it can, an SSH session included.
  const bool decorative = !w.controls;
  media::PlayerSource ps;
  ps.uri = uri;
  ps.auth = w.media_auth;
  ps.video = video;
  ps.frame_w = bw;
  ps.frame_h = bh > 0 ? bh : bw;
  // The Loop box's value, once it has one: a new source's player starts as the one before it ends.
  const auto slot = players.find(key);
  const std::string loop_was = slot != players.end() ? slot->second.wrote_loop : std::string();
  ps.loop = loop_was.empty() ? w.loop : (loop_was == "1" || loop_was == "true");
  ps.muted = w.muted;
  ps.audio = !(decorative && w.muted);
  PlayerSlot &p = player_slot(key, ps);
  const double now = steady_seconds();
  media::Player *pl = p.player.get();
  // A new box (the fullscreen view, and back): the frames from now on are decoded for it.
  if (pl && (p.box_w != ps.frame_w || p.box_h != ps.frame_h)) {
    if (p.box_w > 0 || p.box_h > 0)
      pl->set_frame_box(ps.frame_w, ps.frame_h);
    p.box_w = ps.frame_w;
    p.box_h = ps.frame_h;
  }
  double pos = pl ? pl->position() : 0.0;
  const double dur = pl ? pl->duration() : 0.0;

  // What the document wrote to the player's keys since the last frame.
  const auto key_of = [&](const char *k) { return prefix + "." + k; };
  if (pl) {
    const std::string want = read_or<std::string>(app, key_of("playing"), p.wrote_playing);
    if (want != p.wrote_playing &&
        (want == "1" || want == "0" || want == "true" || want == "false")) {
      if (want == "1" || want == "true")
        pl->play();
      else
        pl->pause();
      p.wrote_playing = want;
      p.autoplay_owned = p.offscreen_paused = false; // the document's now, in view or not
    }
    const std::string seek = read_or<std::string>(app, key_of("seek"), std::string());
    if (!seek.empty()) {
      char *end = nullptr;
      const double t = std::strtod(seek.c_str(), &end);
      if (end != seek.c_str() && std::isfinite(t))
        pl->seek(t);
      write<std::string>(app, key_of("seek"), std::string());
    }
    const std::string mute = read_or<std::string>(app, key_of("muted"), p.wrote_muted);
    if (mute != p.wrote_muted && !mute.empty()) {
      pl->set_muted(mute == "1" || mute == "true");
      p.wrote_muted = mute;
    }
  }
  // <bind>.loop: 1 or 0 from the document, read with or without a player (a new one takes it).
  const std::string loop_s = read_or<std::string>(app, key_of("loop"), p.wrote_loop);
  if (loop_s != p.wrote_loop && !loop_s.empty()) {
    p.wrote_loop = loop_s;
    if (pl)
      pl->set_loop(loop_s == "1" || loop_s == "true");
  }
  // §5.13 <bind>.fullscreen: a document writes 1 to show it alone (fullscreen: true), 0 to leave.
  if (video && w.fullscreen_control && !w.bind.empty() && !is_expr(w.bind)) {
    const std::string want = read_or<std::string>(app, key_of("fullscreen"), p.wrote_fullscreen);
    if (want != p.wrote_fullscreen) {
      if (!in_fullscreen && (want == "1" || want == "true"))
        open_fullscreen(w);
      else if (in_fullscreen && (want == "0" || want == "false"))
        fullscreen_exit = true;
    }
  }

  // The picture: the newest frame (at most 8 a second in a terminal), else the poster, else a
  // placeholder of the box. One whose height follows its shape stays within max_height:.
  bool in_view = true;
  if (video) {
    std::string name;
    float aspect = 0.0f; // its height / width, where the box doesn't set it
    if (player_frame(p, bw, bh, w.img_fit, w.img_shape, now)) {
      name = p.frame_name;
      aspect = p.aspect;
    } else if (media::is_uri(w.poster)) {
      media::ImageRequest rq;
      rq.uri = w.poster;
      rq.auth = w.media_auth;
      rq.w = bw;
      rq.h = bh;
      rq.fit = w.img_fit;
      rq.shape = w.img_shape;
      const media::ImageView v = image_cache().image(rq);
      drew_loading = drew_loading || v.status == media::Status::Loading;
      if (v.status == media::Status::Ready) {
        publish_media(v);
        name = v.name;
        if (v.image.width() > 0)
          aspect = static_cast<float>(v.image.height()) / static_cast<float>(v.image.width());
      }
    }
    if (name.empty()) {
      const bool failed = p.state == media::PlayerState::Failed;
      const int ph = media::placeholder_height(bw, bh, w.img_shape);
      name = std::string("media:placeholder:") + (failed ? "failed:" : "loading:") +
             std::to_string(bw) + "x" + std::to_string(ph) +
             (w.img_shape == ImageShape::Circle ? ":circle" : "");
      if (!published.count(name)) {
        b.set_image(name, media::placeholder_image(bw, bh, w.img_shape, failed));
        published[name] = 0;
      }
      aspect = static_cast<float>(ph) / static_cast<float>(bw);
    }
    const float width =
        within_height(static_cast<float>(draw_w), bh > 0 ? 0.0f : aspect, w.img_max_h);
    const bool drew = b.draw_image(name.c_str(), width);
    if (!drew)
      b.text_value(caption.c_str(), "(" + std::string(media::player_state_name(p.state)) + ")");
    in_view = b.item_visible();
    // No frame yet: the box says why, over the poster or the placeholder: its error once it fails
    // (with controls, "failed": their line has the error), what the player says keeps the picture
    // away (a browser waiting for a tap), or, loading or playing 3 s with no frame, that state.
    // Not where a badge: of its own is, in a box too small for words (an avatar, an emoji), or in
    // a terminal (a badge there is a line of its own, under the picture).
    if (drew && pl && !p.have_frame && w.badge.empty() && draw_w >= 96 &&
        !b.capabilities().terminal) {
      std::string why;
      if (p.state == media::PlayerState::Failed)
        why = decorative && !p.error.empty() ? p.error : std::string("failed");
      else
        why = pl->notice();
      if (why.empty() &&
          (p.state == media::PlayerState::Loading || p.state == media::PlayerState::Playing) &&
          now - p.made_at >= 3.0)
        why = media::player_state_name(p.state);
      if (!why.empty())
        b.item_badge(why.c_str());
    }
    emit_badge(w);
    if (drew && !decorative) // §5.29 Save video... in its context menu (a decorative one has none)
      offer_save(w, uri, /*video=*/true);
  }

  // autoplay: it starts once it can, and until the user (or the document) plays or pauses it, it
  // plays while its picture is in view: a feed's videos play as they scroll by, not all at once.
  if (pl && w.autoplay) {
    if (!p.autoplay_done && p.state == media::PlayerState::Ready && in_view) {
      p.autoplay_done = p.autoplay_owned = true;
      pl->play();
    } else if (p.autoplay_owned && !in_view && p.state == media::PlayerState::Playing) {
      pl->pause();
      p.offscreen_paused = true;
    } else if (p.autoplay_owned && p.offscreen_paused && in_view &&
               p.state == media::PlayerState::Paused) {
      pl->play();
      p.offscreen_paused = false;
    }
  }

  const bool playing = p.state == media::PlayerState::Playing;
  const bool elsewhere = p.state == media::PlayerState::Remote ||
                         p.state == media::PlayerState::Unsupported ||
                         p.state == media::PlayerState::Failed;
  if (!decorative) {
    // The controls, with the widgets every backend has: play / pause, a seek slider, the time,
    // mute, and "Open" (the user's browser) where it can't play here.
    // Play works while the player loads (it starts once it can), so it is the first stop for
    // autofocus and Tab; only a player that failed, or none, can't take it.
    const bool usable = pl && p.state != media::PlayerState::Failed;
    b.push_id(("player:" + key).c_str());
    Layout row;
    row.kind = LayoutKind::Horizontal;
    if (b.begin_grid(row, ("player:" + key).c_str())) {
      b.grid_next_cell();
      if (!usable)
        b.begin_disabled();
      if (b.button(playing ? "Pause" : "Play") && usable) {
        if (playing)
          pl->pause();
        else
          pl->play();
        p.autoplay_owned = p.offscreen_paused = false; // the user's now, in view or not
      }
      b.grid_next_cell();
      // The slider's value shows as the time line: a format with no conversion prints as it is,
      // in ImGui and the terminal backends alike.
      const std::string times = media::format_time(pos) + " / " + media::format_time(dur);
      const DoubleEdit seek = b.slider_double("", ("seek:" + key).c_str(), pos, 0.0,
                                              std::max({dur, pos, 0.001}), times.c_str());
      if (seek.committed && usable) {
        pl->seek(seek.value);
        pos = seek.value;
      }
      if (!usable)
        b.end_disabled();
      b.grid_next_cell();
      bool muted_now =
          p.wrote_muted.empty() ? w.muted : (p.wrote_muted == "1" || p.wrote_muted == "true");
      const BoolEdit mute = b.checkbox("Mute", muted_now);
      if (mute.committed) {
        muted_now = mute.value;
        if (pl)
          pl->set_muted(muted_now);
        p.wrote_muted = muted_now ? "1" : "0";
        write<std::string>(app, key_of("muted"), p.wrote_muted);
      }
      b.grid_next_cell();
      const bool loop_now =
          p.wrote_loop.empty() ? w.loop : (p.wrote_loop == "1" || p.wrote_loop == "true");
      const BoolEdit loop = b.checkbox("Loop", loop_now);
      if (loop.committed) {
        if (pl)
          pl->set_loop(loop.value);
        p.wrote_loop = loop.value ? "1" : "0";
        write<std::string>(app, key_of("loop"), p.wrote_loop);
      }
      // §5.13 fullscreen: true: the picture alone, filling the window; in that view, the way back.
      if (video && ((w.fullscreen_control && b.capabilities().fullscreen_view) || in_fullscreen)) {
        b.grid_next_cell();
        if (b.button(in_fullscreen ? "Exit full screen" : "Full screen")) {
          if (in_fullscreen)
            fullscreen_exit = true;
          else
            open_fullscreen(w);
          ++clicks_fired;
        }
      }
      if (elsewhere && !uri.empty() && media::is_uri(uri) &&
          (uri.rfind("http://", 0) == 0 || uri.rfind("https://", 0) == 0)) {
        b.grid_next_cell();
        if (b.button("Open")) {
          if (has_function("open-url"))
            enqueue("(open-url " + program_string(uri) + ")");
          else
            warn_once("ari: " + caption + ": this host cannot open a browser; the URL is " + uri);
        }
        b.set_tooltip(!w.media_auth.empty() ? "Open in your browser (it needs its own sign-in)"
                                            : "Open in your browser");
      }
      b.end_grid();
    }
    b.pop_id();
    if (elsewhere && !p.error.empty())
      b.text_line((caption + ": " + p.error).c_str());
  }

  // The state keys, written when they change (position at most 4 times a second while playing).
  const std::string st = media::player_state_name(p.state);
  if (st != p.wrote_state) {
    p.wrote_state = st;
    write<std::string>(app, key_of("state"), st);
  }
  // status: (an image:'s key, on a video it was promoted to): the player's state, and its error.
  if (!w.media_status.empty()) {
    const std::string path = resolve(w.media_status);
    std::string &seen = media_status_written[path];
    if (seen != st) {
      seen = st;
      write<std::string>(app, path, st);
      write<std::string>(app, path + ".error", p.error);
    }
  }
  const std::string playing_s = playing ? "1" : "0";
  if (playing_s != p.wrote_playing) {
    p.wrote_playing = playing_s;
    write<std::string>(app, key_of("playing"), playing_s);
  }
  if (std::fabs(pos - p.wrote_position) > 0.01 && (!playing || now - p.wrote_position_at >= 0.25)) {
    p.wrote_position = pos;
    p.wrote_position_at = now;
    write<double>(app, key_of("position"), pos);
  }
  const std::string dur_s = std::to_string(dur);
  if (dur_s != p.wrote_duration) {
    p.wrote_duration = dur_s;
    write<double>(app, key_of("duration"), dur);
  }
  if (p.error != p.wrote_error) {
    p.wrote_error = p.error;
    write<std::string>(app, key_of("error"), p.error);
  }
  if (p.wrote_loop.empty()) { // its loop: from the start, so a document can read it
    p.wrote_loop = w.loop ? "1" : "0";
    write<std::string>(app, key_of("loop"), p.wrote_loop);
  }
  if (video && w.fullscreen_control) {
    const bool shown = fullscreen.active && !fullscreen_exit && fullscreen.emit_path == emit_path &&
                       fullscreen.instance_path == instance_path;
    const std::string fs = shown ? "1" : "0";
    if (fs != p.wrote_fullscreen) {
      p.wrote_fullscreen = fs;
      write<std::string>(app, key_of("fullscreen"), fs);
    }
  }
}

void Runtime::Impl::open_fullscreen(const Widget &w) {
  if (!backend || !backend->capabilities().fullscreen_view)
    return; // no view to show it in (the tvision desktop): the picture stays where it is
  FullscreenItem &f = fullscreen;
  f = FullscreenItem{};
  f.active = true;
  f.w = w;
  f.emit_path = emit_path;
  f.instance_path = instance_path;
  f.scope_stack = scope_stack;
  if (!fn_stack.empty())
    f.functions = std::make_shared<const FunctionScope>(*fn_stack.back());
  // The window too, where it can (within the click's gesture, which a browser asks for).
  if (backend && backend->capabilities().fullscreen && !backend->is_fullscreen())
    f.made_os = backend->set_fullscreen(true);
}

void Runtime::Impl::close_fullscreen() {
  if (fullscreen.made_os && backend && backend->is_fullscreen())
    backend->set_fullscreen(false);
  fullscreen = FullscreenItem{};
}

bool Runtime::Impl::emit_fullscreen() {
  Backend &b = *backend;
  FullscreenItem &f = fullscreen;
  const FullscreenView v = b.begin_fullscreen_view();
  // Left by Esc, or by the window's own way out (a browser page's Esc never reaches the page).
  const bool os_now = f.made_os && b.is_fullscreen();
  f.saw_os = f.saw_os || os_now;
  bool open = v.shown && !v.exit && !(f.saw_os && !os_now) && v.w > 0.0f && v.h > 0.0f;
  if (open) {
    // Drawn where it was, so a player keeps its slot; boxed to the view, a player's controls under
    // it.
    emit_path = f.emit_path;
    instance_path = f.instance_path;
    scope_stack = f.scope_stack;
    if (f.functions)
      fn_stack.push_back(f.functions.get());
    Widget w = f.w;
    const bool player = (w.kind == Kind::Video || w.kind == Kind::Audio) && w.controls;
    w.img_fill = 0.0;
    w.img_max_h = 0.0;
    w.img_fit = ImageFit::Contain;
    w.img_shape = ImageShape::Rect;
    w.img_size = std::max(1.0, std::floor(static_cast<double>(v.w)));
    w.img_h = std::max(1.0, std::floor(static_cast<double>(v.h - (player ? v.controls_h : 0.0f))));
    w.tooltip.clear();
    in_fullscreen = true;
    fullscreen_exit = false;
    run_walk([&] { emit_node(w); });
    in_fullscreen = false;
    fn_stack.clear();
    scope_stack.clear();
    emit_path.clear();
    instance_path.clear();
    open = !fullscreen_exit;
  }
  b.end_fullscreen_view();
  if (!open && fullscreen.active)
    close_fullscreen();
  return open;
}

void Runtime::Impl::players_end_walk() {
  const double now = steady_seconds();
  for (auto it = players.begin(); it != players.end();) {
    PlayerSlot &p = it->second;
    if (p.drawn_walk != walk_count) {
      // Not drawn this walk (its window closed, scrolled out of a repeat): stop the sound now,
      // and let it go after a while.
      if (p.player && !p.hidden_paused && p.player->state() == media::PlayerState::Playing) {
        p.player->pause();
        p.hidden_paused = true;
        p.offscreen_paused = p.autoplay_owned; // an autoplaying one plays again when it is back
      }
      if (now - p.last_drawn > 5.0) {
        if (backend)
          backend->drop_image(p.frame_name);
        published.erase(p.frame_name);
        it = players.erase(it);
        continue;
      }
    }
    ++it;
  }
  ++walk_count;
}

void Runtime::Impl::render() {
  // Render starts mount ticks and runs mount init:s, and its reconcile kills residents: on the host
  // thread, between pumps (S1.10). A new document is placed before its tree is drawn, so its
  // holes are planted and its mount ticks started where it runs.
  assert_outside_pump(app, group_.get());
  ensure_placement(/*submitting=*/false);
  if (!backend)
    return;
  // §5.11: published pictures belong to the backend they went to; the cache's dropped ones go.
  if (published_to != backend) {
    published.clear();
    published_to = backend;
  }
  drew_loading = false;
  if (images)
    for (const std::string &name : images->take_dropped()) {
      backend->drop_image(name);
      published.erase(name);
    }
  // Reconcile commit boundary (roadmap §11.5.1): a queued tree swap is applied
  // here, at the top of the frame, before any node is emitted — never mid-walk.
  if (has_pending) {
    root = std::move(pending);
    pending = Widget{};
    has_pending = false;
    mount_instances_.clear(); // §12 made from the old tree's templates
    plan_windows_menu(); // §5.9: which windows the built-in Windows menu lists
    // §12: tear down the OLD tree's parent-scope holes before the new tree re-wires its own, so a
    // fragment re-mounted at the same sub-prefix cannot inherit a grant the new manifest omits
    // (clearLink turns the hole back into a plain, sandboxed local node). Re-wiring happens as
    // this same frame walks the new tree, so there is no window where a stale grant is live.
    cvc::state &root_state = cvc::state::instance(app);
    for (const std::string &hole : planted_holes) {
      try {
        if (cvc::state *n = root_state.findDescendant(hole))
          n->clearLink();
      } catch (const std::exception &) {
      }
    }
    planted_holes.clear();
    wired_mounts.clear();
    drop_mount_ticks(); // §12 the old tree's module ticks the new one does not keep
    // ARIADNE_SHELL.md 4.6 the new tree's load:s publish their command lines again as the walk
    // meets them (plan_windows_menu, above, found them and what they hold).
    prepared_mounts_.clear();
    if (fullscreen.active)
      close_fullscreen(); // §5.13 its widget is the old tree's
  }
  frame_t =
      pinned_clock
          ? *pinned_clock
          : std::chrono::duration<double>(std::chrono::steady_clock::now() - clock_epoch).count();
  if (forget_layout_pending) { // a new document: its windows are laid out afresh
    forget_layout_pending = false;
    laid_out.clear();
    windows_forgotten = true;
    backend->reset_layout();
  }
  if (reactive)
    reactive->begin_frame(reactive_budget); // §4: reset the per-frame reactive eval budget
  frame_instances = 0;       // §3: reset the per-frame repeat-expansion budget
  emit_path.clear();
  scope_stack.clear(); // §12: start every frame at the document scope (run_walk keeps it balanced;
                       // this is belt-and-suspenders so one bad frame can't leak into the next)
  windows_menu_drawn = false;
  apps_drawn = false;
  first_bar_drawn = false;
  reconcile_mounts(); // ARIADNE_SHELL.md 4.2 the mounts: entries opened, closed or changed
  plan_frame_apps();  // §5.25 the repeated applications' instances this frame
  measure_mounts();   // 4.11 their memory, their time at the limit, the notice
  mirror_apps();
  backend->begin_frame();
  clicks_fired = 0;
  instance_path.clear();
  // §5.13 the fullscreen view, when one is open, in place of the whole tree.
  if (!fullscreen.active || !emit_fullscreen()) {
    // §5.9: a document with no menubar but a closable window gets a menubar holding just the
    // Windows menu, so a window closed with its X can be opened again.
    // §5.25 and one whose loaded documents have menubars gets one holding theirs.
    if (!windows_menu.has_menubar &&
        (!frame_apps.empty() ||
         (windows_menu.any_closable && windows_menu.on && windows_menu_has_items())) &&
        backend->begin_main_menu_bar()) {
      ++menubar_depth;
      Widget bare;
      bare.kind = Kind::Menubar;
      std::vector<TrayEntry> tray = tray_entries(bare);
      // only the applications' icons: this bar is not the document's, so no clock or bell
      tray.erase(std::remove_if(tray.begin(), tray.end(),
                                [](const TrayEntry &t) { return t.scope.empty(); }),
                 tray.end());
      run_walk([&] {
        push_then([this] {
          --menubar_depth;
          backend->end_main_menu_bar();
        });
        push_then([this, tray = std::move(tray)]() mutable { emit_tray(std::move(tray)); });
        push_then([this] { emit_windows_menu(windows_menu.title); });
        emit_app_menus();
      });
    }
    emit(root);
    // §5.30 the document's own right-click menu, over the main view: after every window, so a
    // window's or a group's menu under the pointer opens instead.
    if (root.context_menu && root.repeat.empty() && visible(root))
      run_walk([&] { emit_context_menu(root, ContextMenuOver::Main, main_view_pick); });
    emit_notify_window(); // §5.24, while Show all... has it open
    emit_budget_window(); // ARIADNE_SHELL.md 4.11, while a notice's click has it open
    // §10.8 the Runtime's own "Open file" window, over the document, while a (file-pick) waits
    // on it.
    if (file_pick->cancel.exchange(false))
      file_pick->composed.close();
    file_pick->composed.emit(*backend);
  }
  players_end_walk(); // §5.13: a player whose widget this walk did not draw stops
  // §5.30 a menu this walk did not draw (its widget hidden, its window closed) is closed.
  std::erase_if(menu_events, [this](const auto &kv) { return kv.second.walk != walk_no; });
  ++walk_no;
  backend->end_frame();
  end_frame_focus(); // ARIADNE_SHELL.md 4.6 where the keyboard and the pointer were this frame
  autofocus_shown.swap(autofocus_now);
  autofocus_now.clear();
  maximized_shown.swap(maximized_now);
  maximized_now.clear();
  // §4.6 on_hover_end: an item hovered last walk and not this one (the pointer left, or the item
  // went), in its own scope.
  for (const auto &[key, h] : hovered_last)
    if (!hovered_now.count(key)) {
      queued_events.push_back({h.handler, h.prefix});
      queued_events.back().functions = h.functions;
    }
  hovered_last.swap(hovered_now);
  hovered_now.clear();
  if (++chart_frame % 64 == 0) {
    if (!chart_caches.empty())
      sweep_chart_caches(chart_caches, chart_frame);
    if (!heatmap_caches.empty())
      sweep_heatmap_caches(heatmap_caches, chart_frame);
    if (!settings_caches.empty())
      sweep_settings_caches(settings_caches, chart_frame);
  }
}

void Runtime::Impl::plan_windows_menu() {
  WindowsMenuPlan p;
  std::vector<std::pair<const Widget *, std::string>> mounts; // the mounts: slots met
  std::map<std::string, std::shared_ptr<const MountGrants>> loads; // the load:s' mounts met
  std::set<std::string> toggled;   // open-state paths a menu item of the document already toggles
  std::set<std::string> bar_menus; // the titles of the menus straight in a menubar
  std::vector<WindowsMenuPlan::Item> windows;
  // Walk the tree with the scope each node resolves its binds in (a load: mount scopes its
  // children, §12). A repeated widget is made per frame: a repeated window's instances are listed
  // as the menu is drawn (repeated_windows).
  // A node to visit, in tree order (a stack on the heap, not recursion): the scope it resolves in,
  // whether it is in a menubar, and `mount`, the innermost mount it is in (itself, when it is one),
  // null in the document's own tree, with that mount's scope and the scope the mount sits in
  // (`mount_parent`, where its name: is read).
  struct Visit {
    const Widget *w;
    std::string scope;
    bool in_menubar;
    const Widget *mount;
    std::string mount_scope, mount_parent;
    // `w` null: the mounts: entry `slot` has been walked (ARIADNE_SHELL.md 4.9).
    const std::pair<const std::string, MountSlot> *slot = nullptr;
  };
  // A repeated mount's bar: the first menubar in it, in tree order, not in a nested mount.
  const auto first_bar = [](const Widget &m) -> const Widget * {
    std::vector<const Widget *> todo;
    for (auto c = m.children.rbegin(); c != m.children.rend(); ++c)
      todo.push_back(&*c);
    while (!todo.empty()) {
      const Widget &c = *todo.back();
      todo.pop_back();
      if (c.kind == Kind::Menubar)
        return &c;
      if (!c.scope.empty())
        continue; // a nested mount's bar is its own
      for (auto g = c.children.rbegin(); g != c.children.rend(); ++g)
        todo.push_back(&*g);
    }
    return nullptr;
  };
  std::vector<Visit> todo;
  todo.push_back({&root, prefix, false, nullptr, {}, {}});
  while (!todo.empty()) {
    Visit v = std::move(todo.back());
    todo.pop_back();
    if (!v.w) { // a mounts: entry is an application, with a bar or without
      const auto &[sub, slot] = *v.slot;
      bool has_app = false;
      for (const WindowsMenuPlan::App &a : p.apps)
        has_app = has_app || a.mount == slot.wrapper.get();
      if (!has_app) {
        WindowsMenuPlan::App a;
        a.mount = slot.wrapper.get();
        a.name = slot.name;
        a.scope = sub;
        a.functions = slot.wrapper->functions.get();
        a.parent_scope = v.scope;
        p.apps.push_back(std::move(a));
      }
      continue;
    }
    const Widget &w = *v.w;
    const std::string &scope = v.scope;
    bool in_menubar = v.in_menubar;
    {
      if (!w.repeat.empty()) {
        if (w.kind == Kind::Window) {
          p.repeated.emplace_back(&w, scope);
          p.any_closable = p.any_closable || w.closable;
        }
        // §5.25 a repeated mount with a menubar of its own: an application per instance.
        if (!w.scope.empty()) {
          if (const Widget *bar = first_bar(w)) {
            WindowsMenuPlan::App a;
            a.mount = &w;
            a.bar = bar;
            a.name = !w.app_name.empty() ? w.app_name : w.scope.substr(w.scope.rfind('.') + 1);
            a.functions = w.functions.get();
            a.repeat = &w;
            a.parent_scope = scope;
            a.name_bind = w.app_name_bind;
            p.apps.push_back(std::move(a));
          }
        }
        continue;
      }
      const std::string inner = w.scope.empty() ? scope : resolve_bind(scope, w.scope);
      if (!w.scope.empty()) {
        v.mount = &w;
        v.mount_scope = inner;
        v.mount_parent = scope;
        // ARIADNE_SHELL.md 4.5 a load:'s mount and what it holds (a mounts: entry's is its slot's)
        if (!is_mounts_app(inner))
          loads[inner] = w.grants;
      }
      const Widget *const mount = v.mount;
      const std::string &mount_scope = v.mount_scope;
      const std::string &mount_parent = v.mount_parent;
      switch (w.kind) {
      case Kind::Menubar:
        in_menubar = true;
        if (mount) { // §5.25 a loaded document's: an application, once per mount
          bool seen = false;
          for (const WindowsMenuPlan::App &a : p.apps)
            seen = seen || a.mount == mount;
          if (!seen) {
            WindowsMenuPlan::App a;
            a.mount = mount;
            a.bar = &w;
            a.name = !mount->app_name.empty() ? mount->app_name
                                              : mount->scope.substr(mount->scope.rfind('.') + 1);
            a.scope = mount_scope;
            a.functions = mount->functions.get();
            a.parent_scope = mount_parent;
            a.name_bind = mount->app_name_bind;
            p.apps.push_back(std::move(a));
          }
          break;
        }
        p.has_menubar = true;
        for (const Widget &c : w.children)
          if (c.kind == Kind::Menu)
            bar_menus.insert(c.label);
        break;
      case Kind::MenuItemToggle:
        if (!w.bind.empty() && !is_expr(w.bind))
          toggled.insert(resolve_bind(scope, w.bind));
        break;
      case Kind::TrayIcon: // §5.22 the clock, §5.24 the bell: placed in a menubar, else the
                           // document's setting (a loaded document's are its own: ignored)
        if (mount)
          break;
        if (w.tray_clock && in_menubar)
          p.clock_placed = true;
        else if (w.tray_clock)
          p.clock_setting = w.bdef ? 1 : 0;
        else if (w.tray_bell && in_menubar)
          p.bell_placed = true;
        else if (w.tray_bell) {
          p.bell_setting = w.bdef ? 1 : 0;
          p.bell_always = w.bell_always;
        }
        break;
      case Kind::WindowsMenu:
        if (mount) // a loaded document's Windows menu is the outer document's
          break;
        if (!w.bdef)
          p.on = false;
        else if (in_menubar)
          p.placed = true;
        if (!w.label.empty())
          p.title = w.label;
        break;
      case Kind::Mounts: { // ARIADNE_SHELL.md 4.2: the slot, and each mount open in it
        mounts.emplace_back(&w, scope);
        if (w.bind.empty())
          break;
        const std::string key = resolve_bind(scope, w.bind);
        std::vector<const std::pair<const std::string, MountSlot> *> open;
        for (const auto &e : mount_slots_)
          if (e.second.slot_key == key && e.second.parent_scope == scope && e.second.wrapper)
            open.push_back(&e);
        for (auto e = open.rbegin(); e != open.rend(); ++e) { // the first on top: tree order
          todo.push_back({nullptr, scope, false, nullptr, {}, {}, *e}); // then its application
          todo.push_back({(*e)->second.wrapper.get(), scope, false, nullptr, {}, {}});
        }
        break;
      }
      case Kind::Window: {
        const std::string &id = w.id.empty() ? w.label : w.id;
        windows.push_back({w.label.empty() ? id : w.label,
                           resolve_bind(scope, "tree." + id + ".open"),
                           resolve_bind(scope, "tree." + id + ".raise")});
        p.any_closable = p.any_closable || w.closable;
        in_menubar = false;
        break;
      }
      default:
        break;
      }
      for (auto c = w.children.rbegin(); c != w.children.rend(); ++c)
        todo.push_back({&*c, inner, in_menubar, mount, mount_scope, mount_parent});
    }
  }
  for (auto &win : windows)
    if (!toggled.count(win.open_path))
      p.items.push_back(std::move(win));
  p.merged = !p.placed && bar_menus.count(p.title) != 0;
  // §5.25 two mounts of one document share its name: each is told apart by its mount id.
  std::map<std::string, int> named; // (a repeated one's instances are numbered instead)
  for (const WindowsMenuPlan::App &a : p.apps)
    if (!a.repeat)
      ++named[a.name];
  for (WindowsMenuPlan::App &a : p.apps)
    if (!a.repeat && named[a.name] > 1)
      a.name += " (" + a.mount->scope.substr(a.mount->scope.rfind('.') + 1) + ")";
  p.toggled = std::move(toggled);
  windows_menu = std::move(p);
  mounts_widgets_ = std::move(mounts);
  load_grants_ = std::move(loads);
}

std::vector<Runtime::Impl::RepeatedWindow> Runtime::Impl::repeated_windows() {
  std::vector<RepeatedWindow> out;
  constexpr std::string_view kTok = kIndexToken;
  for (const auto &[tmpl, scope] : windows_menu.repeated) {
    const int n = eval_count(tmpl->repeat); // the count its windows are drawn with this frame
    for (int i = 0; i < n; ++i) {
      const std::string idx = std::to_string(i);
      RepeatedWindow r;
      r.id = replace_all(tmpl->id.empty() ? tmpl->label : tmpl->id, kTok, idx);
      r.open_path = resolve_bind(scope, "tree." + r.id + ".open");
      r.raise_path = resolve_bind(scope, "tree." + r.id + ".raise");
      if (windows_menu.toggled.count(r.open_path))
        continue;
      // Its title as the window shows it: the bound one (a peer's name), else its label.
      r.title = replace_all(tmpl->label, kTok, idx);
      if (!tmpl->bind.empty()) {
        const std::string bind = replace_all(tmpl->bind, kTok, idx);
        const std::string t =
            is_expr(bind) ? eval_text(bind) : read_string(app, resolve_bind(scope, bind));
        if (!t.empty())
          r.title = t;
      }
      out.push_back(std::move(r));
    }
  }
  return out;
}

bool Runtime::Impl::windows_menu_has_items() {
  return !windows_menu.items.empty() || !frame_apps.empty() ||
         (!windows_menu.repeated.empty() && !repeated_windows().empty());
}

void Runtime::Impl::emit_windows_menu(const std::string &title) {
  if (!windows_menu.on || windows_menu_drawn || !windows_menu_has_items())
    return;
  windows_menu_drawn = true;
  Backend &b = *backend;
  if (!b.begin_menu(title.c_str()))
    return;
  emit_windows_menu_items();
  b.end_menu();
}

void Runtime::Impl::emit_windows_menu_items() {
  Backend &b = *backend;
  // As a window manager's Windows menu: picking a window shows it, in front. Its tick says it is
  // open; picking an open one raises it rather than closing it (its X closes it).
  const auto item = [&](const std::string &title, const std::string &open_path,
                        const std::string &raise_path) {
    const bool cur = read_bool_or_seed(app, open_path, 1) != 0;
    if (b.menu_item_toggle(title.c_str(), cur).committed) {
      write<int>(app, open_path, 1);
      write<std::string>(app, raise_path, "1");
    }
  };
  for (const WindowsMenuPlan::Item &it : windows_menu.items)
    if (quit_apps.empty() || app_running(it.open_path)) // §5.25 a quit application's come back
      item(it.title, it.open_path, it.raise_path);      // with it
  for (const RepeatedWindow &r : repeated_windows()) {
    b.push_id(r.id.c_str()); // two instances may share a title (the label, before their names)
    item(r.title, r.open_path, r.raise_path);
    b.pop_id();
  }
  // §5.25 the loaded documents with a menubar: ticked while running; untick to quit one, tick to
  // open it again as it was.
  if (!frame_apps.empty()) {
    b.separator();
    b.text_line("Applications");
    for (const WindowsMenuPlan::App &a : frame_apps) {
      b.push_id(("##app." + a.scope).c_str());
      const bool running = app_running(a.scope);
      const BoolEdit e = b.menu_item_toggle(a.name.c_str(), running);
      if (e.committed && e.value != running)
        queued_events.push_back(
            {kAppEvent + std::string(e.value ? "open:" : "quit:") + a.scope, prefix});
      b.pop_id();
    }
  }
}

// A tray icon as the backend draws it this frame: its glyph (a named icon's, cvc/ariadne/icons.h,
// or its label as written), its badge (a text or an expression, as a picture's badge is, §5.11; ""
// and "0" show none) and the badge's tone.
Backend::TrayIcon Runtime::Impl::tray_spec(const Widget &w) {
  Backend::TrayIcon t;
  t.glyph = std::string(icon_glyph(w.label, backend->unicode_glyphs()));
  if (named_icon(w.label))
    t.icon = w.label;
  t.badge = is_expr(w.badge) ? eval_text_or_literal(w.badge) : w.badge;
  if (t.badge == "0")
    t.badge.clear();
  const std::string tone = is_expr(w.tone) ? eval_text(w.tone) : w.tone;
  t.tone = tone == "error" ? 2 : tone == "warn" ? 1 : 0;
  t.menu = !w.children.empty();
  t.picture = tray_picture(w);
  return t;
}

// §5.22 a tray icon's `icon:`: the published name of its picture, round and a little over the
// bar's height (64 px, sharp on a phone's dense screen), or "" while it loads, if it fails, in a
// terminal, or with none. A video URL (an avatar can be one) plays as a decorative video: silent,
// looping, by itself; an animated GIF or WebP moves, as an image:'s does.
std::string Runtime::Impl::tray_picture(const Widget &w) {
  if (w.button_icon.empty() || backend->capabilities().terminal)
    return std::string();
  const std::string icon = is_expr(w.button_icon) ? eval_text(w.button_icon) : w.button_icon;
  if (icon.empty())
    return std::string();
  if (!media::is_uri(icon))
    return icon; // a host-published picture
  constexpr int kPx = 64;
  if (media::is_video_uri(icon)) {
    media::PlayerSource ps;
    ps.uri = icon;
    ps.auth = w.media_auth;
    ps.video = true;
    ps.frame_w = ps.frame_h = kPx;
    ps.loop = ps.muted = true;
    ps.audio = false;
    PlayerSlot &p = player_slot("tray#" + icon, ps);
    if (p.player && !p.autoplay_done && p.state == media::PlayerState::Ready) {
      p.autoplay_done = true;
      p.player->play();
    }
    if (player_frame(p, kPx, kPx, ImageFit::Cover, ImageShape::Circle, steady_seconds()))
      return p.frame_name;
    return std::string();
  }
  media::ImageRequest rq;
  rq.uri = icon;
  rq.auth = w.media_auth;
  rq.w = rq.h = kPx;
  rq.fit = media::Fit::Cover;
  rq.shape = media::Shape::Circle;
  media::ImageView v;
  const std::string name = media_picture(rq, v);
  return v.status == media::Status::Ready ? name : std::string();
}

namespace {
// `when` in local time, as strftime formats it with `fmt`.
std::string local_time(std::chrono::system_clock::time_point when, const char *fmt) {
  const std::time_t t = std::chrono::system_clock::to_time_t(when);
  std::tm tm{};
#ifdef _WIN32
  if (localtime_s(&tm, &t) != 0)
    return std::string();
#else
  if (!localtime_r(&t, &tm))
    return std::string();
#endif
  char buf[64] = "";
  return std::strftime(buf, sizeof buf, fmt, &tm) ? std::string(buf) : std::string();
}
} // namespace

bool Runtime::Impl::clock_on_by_default() const {
  return windows_menu.clock_setting >= 0 ? windows_menu.clock_setting == 1 : clock_default;
}

namespace {
// `path` is `scope` or under it.
bool path_under(const std::string &path, const std::string &scope) {
  return path == scope ||
         (path.size() > scope.size() && path.compare(0, scope.size(), scope) == 0 &&
          path[scope.size()] == cvc::state::SEPARATOR[0]);
}
} // namespace

// §5.25 this frame's applications: the plan's, a repeated one as many as its repeat: makes now,
// each named "<name> <n>" with its instance's scope and its bar's per-instance copy.
void Runtime::Impl::plan_frame_apps() {
  frame_apps.clear();
  frame_app_bars.clear();
  // Each application's name once: a second one a name: gives again is told apart by a number
  // ("tx.wtf 2"), as two menus of one title would be one menu to a terminal's bar.
  const auto unique = [&](std::string name) {
    const auto taken = [&](const std::string &n) {
      for (const WindowsMenuPlan::App &o : frame_apps)
        if (o.name == n)
          return true;
      return false;
    };
    if (!taken(name))
      return name;
    for (int k = 2;; ++k)
      if (const std::string n = name + " " + std::to_string(k); !taken(n))
        return n;
  };
  for (const WindowsMenuPlan::App &a : windows_menu.apps) {
    if (!a.repeat) {
      WindowsMenuPlan::App app_ = a;
      if (!a.name_bind.empty()) { // §5.25 the mount's name:, read in the scope it sits in
        const ScopedTo in(*this, a.parent_scope, nullptr);
        if (std::string n = mount_name_of(a.name_bind); !n.empty())
          app_.name = std::move(n);
      }
      app_.name = unique(std::move(app_.name));
      frame_apps.push_back(std::move(app_));
      continue;
    }
    int n = 0;
    {
      const ScopedTo in(*this, a.parent_scope, nullptr);
      n = eval_count(a.repeat->repeat);
    }
    for (int i = 0; i < n; ++i) {
      WindowsMenuPlan::App inst = a;
      inst.scope = resolve_bind(a.parent_scope, a.repeat->scope + "." + std::to_string(i));
      std::string name;
      if (!a.name_bind.empty()) { // this instance's name:, {i} filled in
        std::string spec = a.name_bind;
        replace_in(spec, kIndexToken, std::to_string(i));
        const ScopedTo in(*this, a.parent_scope, nullptr);
        name = mount_name_of(spec);
      }
      inst.name = unique(name.empty() ? a.name + " " + std::to_string(i + 1) : std::move(name));
      frame_app_bars.push_back(substitute_index(*a.bar, i));
      inst.bar = &frame_app_bars.back();
      inst.repeat = nullptr;
      frame_apps.push_back(std::move(inst));
    }
  }
}

bool Runtime::Impl::is_app_scope(const std::string &scope) const {
  for (const WindowsMenuPlan::App &a : frame_apps)
    if (a.scope == scope)
      return true;
  return false;
}

bool Runtime::Impl::app_running(const std::string &path) const {
  for (const std::string &q : quit_apps)
    if (path_under(path, q))
      return false;
  return true;
}

namespace {
// §5.25 an application's key under ariadne.apps: its scope from the document's prefix with the
// `includes` segments left out and the rest joined by '_' (includes.fe is fe, a repeated mount's
// includes.cam.0 is cam_0, a nested includes.a.includes.b is a_b).
std::string app_key(const std::string &scope) {
  std::string key;
  std::size_t at = 0;
  while (at <= scope.size()) {
    std::size_t dot = scope.find(cvc::state::SEPARATOR[0], at);
    if (dot == std::string::npos)
      dot = scope.size();
    const std::string segment = scope.substr(at, dot - at);
    if (!segment.empty() && segment != "includes")
      key += (key.empty() ? "" : "_") + segment;
    at = dot + 1;
  }
  return key;
}
} // namespace

// §5.25 this frame's applications, mirrored under the document's prefix
// (docs/roadmap/ARIADNE_SHELL.md 4.9): ariadne.apps.<key>.{name, scope, running}, scope written
// from the prefix as (ps) writes a process's, and ariadne.name. Only what changed is written; an
// application gone from the tree is taken out.
void Runtime::Impl::mirror_apps() {
  if (root.app_name != mirrored_name) {
    write<std::string>(app, resolve_bind(prefix, "ariadne.name"), root.app_name);
    mirrored_name = root.app_name;
  }
  if (frame_apps.empty() && mirrored_apps.empty())
    return;
  const std::string base = resolve_bind(prefix, "ariadne.apps");
  std::map<std::string, MirroredApp> now;
  // ARIADNE_SHELL.md 4.11 a mounts: application's budget and its share of it, for the Processes
  // view: its last frame's steps and seconds, its memory, each limit, and whether it was held; at
  // most twice a second, as the view polls.
  const bool budgets = mounts_mirrored_at_ < 0.0 || frame_t - mounts_mirrored_at_ >= 0.5 ||
                       frame_t < mounts_mirrored_at_;
  if (budgets)
    mounts_mirrored_at_ = frame_t;
  for (const WindowsMenuPlan::App &a : frame_apps) {
    MirroredApp m;
    m.name = a.name;
    m.scope = cvc::state_exec::process_scope(a.scope, prefix);
    m.running = app_running(a.scope);
    const auto slot = mount_slots_.find(a.scope);
    if (slot != mount_slots_.end())
      m.principal = slot->second.principal;
    std::string key = app_key(m.scope);
    if (key.empty())
      key = "app";
    for (int n = 2; now.count(key); ++n) // two scopes that read alike: the later one numbered
      key = app_key(m.scope) + "_" + std::to_string(n);
    const auto was = mirrored_apps.find(key);
    const std::string at = base + cvc::state::SEPARATOR + key + cvc::state::SEPARATOR;
    if (was == mirrored_apps.end() || was->second.name != m.name)
      write<std::string>(app, at + "name", m.name);
    if (was == mirrored_apps.end() || was->second.scope != m.scope)
      write<std::string>(app, at + "scope", m.scope);
    if (was == mirrored_apps.end() || was->second.running != m.running)
      write<int>(app, at + "running", m.running ? 1 : 0);
    if (was == mirrored_apps.end() || was->second.principal != m.principal)
      write<std::string>(app, at + "principal", m.principal);
    if (slot != mount_slots_.end() && slot->second.account &&
        (budgets || was == mirrored_apps.end())) {
      const cvc::state_exec::frame_account &acc = *slot->second.account;
      const Runtime::MountBudget &b = slot->second.budget;
      const std::string bt = at + "budget" + cvc::state::SEPARATOR;
      write<double>(app, bt + "steps", static_cast<double>(acc.last_steps.load()));
      write<double>(app, bt + "max_steps", static_cast<double>(b.steps));
      write<double>(app, bt + "time", acc.last_seconds.load());
      write<double>(app, bt + "max_time", b.time);
      write<double>(app, bt + "memory", static_cast<double>(acc.memory.load()));
      write<double>(app, bt + "max_memory", static_cast<double>(b.memory));
      write<int>(app, bt + "held", slot->second.held_since >= 0.0 ? 1 : 0);
    }
    now.emplace(std::move(key), std::move(m));
  }
  for (const auto &[key, m] : mirrored_apps)
    if (!now.count(key)) {
      try {
        cvc::state::instance(app)(base).removeChild(key);
      } catch (const std::exception &) {
      }
    }
  mirrored_apps = std::move(now);
}

// §5.25 the loaded documents' menus, each under its name: its menubar's entries (its menus as
// submenus), drawn in its scope with its functions, then Restart <name> and Quit <name>. Once a
// frame, in the first bar that is the document's own.
void Runtime::Impl::emit_app_menus() {
  if (apps_drawn || frame_apps.empty())
    return;
  apps_drawn = true;
  app_menu(0);
}

// On the walk: the first running application from k, its bar's entries walked in its scope, then
// app_menu(k + 1) for the rest. (frame_apps stays as plan_frame_apps made it for the frame.)
void Runtime::Impl::app_menu(std::size_t k) {
  while (k < frame_apps.size() && !app_running(frame_apps[k].scope))
    ++k; // reopened from Windows > Applications
  if (k == frame_apps.size())
    return;
  push_then([this, k] { app_menu(k + 1); });
  Backend &b = *backend;
  const WindowsMenuPlan::App &a = frame_apps[k];
  b.push_id(("##app." + a.scope).c_str());
  if (!b.begin_menu(a.name.c_str())) {
    b.pop_id();
    return;
  }
  if (!a.bar) { // ARIADNE_SHELL.md 4.9 a mounts: application without a bar: its commands alone
    app_commands(a);
    return;
  }
  const PushedScope in = push_scope(a.scope, a.functions);
  ++app_menu_depth;
  push_then([this, k, in] { // after its entries
    --app_menu_depth;
    pop_scope(in);
    backend->separator();
    app_commands(frame_apps[k]);
  });
  emit_children(*a.bar);
}

// The end of an application's menu: Restart and Quit, and Close for a mounts: entry's.
void Runtime::Impl::app_commands(const WindowsMenuPlan::App &a) {
  Backend &b = *backend;
  if (b.menu_item_action(("Restart " + a.name).c_str()))
    queued_events.push_back({kAppEvent + std::string("restart:") + a.scope, prefix});
  if (b.menu_item_action(("Quit " + a.name).c_str()))
    queued_events.push_back({kAppEvent + std::string("quit:") + a.scope, prefix});
  if (is_mounts_app(a.scope) && b.menu_item_action(("Close " + a.name).c_str()))
    queued_events.push_back({kAppEvent + std::string("close:") + a.scope, prefix});
  b.end_menu();
  b.pop_id();
}

// §5.25 Quit: the application's windows, menus and tray icons go, its on_tick: and the programs it
// started stop. Its state stays, so opening it again (Windows > Applications) resumes it.
void Runtime::Impl::quit_app(const std::string &scope) {
  for (auto it = mount_ticks_.begin(); it != mount_ticks_.end();) {
    if (path_under(it->first, scope)) {
      stop_resident(it->second.resident);
      it = mount_ticks_.erase(it);
    } else {
      ++it;
    }
  }
  if (group_ || used_scheduler_) { // where they run: its group's scheduler, else group 0's
    cvc::state_exec::async_scheduler &sched = current_sched();
    for (const auto &[pid, ac] : live_actions_)
      if (ac && !ac->ended && path_under(ac->scope, scope)) {
        sched.kill(pid);
        for (const int child : ac->children)
          sched.kill(child);
      }
  }
  quit_apps.insert(scope);
}

// §5.25 Restart: Quit, then a full reset. Its state is dropped, its notifications too, and it
// starts again as when it was first loaded: its holes planted, its state: written (§5.3), its init:
// run, its on_tick: started.
void Runtime::Impl::restart_app(const std::string &scope) {
  quit_app(scope);
  const std::size_t dot = scope.rfind(cvc::state::SEPARATOR[0]);
  try {
    if (dot == std::string::npos)
      cvc::state::instance(app).removeChild(scope);
    else
      cvc::state::instance(app)(scope.substr(0, dot)).removeChild(scope.substr(dot + 1));
  } catch (const std::exception &) {
  }
  for (std::set<std::string> *done : {&wired_mounts, &ran_inits})
    for (auto it = done->begin(); it != done->end();)
      it = path_under(*it, scope) ? done->erase(it) : std::next(it);
  cvc::notify_clear_under(app, scope);
  quit_apps.erase(scope);
}

// ---- on-demand mounts (docs/roadmap/ARIADNE_SHELL.md 4.2, 4.11; docs/ARIADNE.md §6.8) ----------

namespace {
// A mount id as a mounts: entry may give it: one clean state-path segment, [A-Za-z0-9_], not
// starting with a digit (the loader's sanitize_mount_id would rename anything else).
bool clean_mount_id(const std::string &as) {
  if (as.empty() || std::isdigit(static_cast<unsigned char>(as.front())))
    return false;
  return std::all_of(as.begin(), as.end(), [](char c) {
    return std::isalnum(static_cast<unsigned char>(c)) || c == '_';
  });
}

// ARIADNE_SHELL.md 4.5, 4.6 a mounts: entry's list at `path` (its grants, its argv): its children's
// values, in order (numbered children by their number), else its own value split at spaces (and
// at commas, with `commas`). `has` says whether the entry has the key at all.
std::vector<std::string> entry_list(cvc::app &app, const std::string &path, bool &has,
                                    bool commas) {
  std::vector<std::string> out;
  has = false;
  try {
    cvc::state *n = cvc::state::instance(app).findDescendant(path);
    if (!n)
      return out;
    has = true;
    std::vector<std::string> kids = n->childNames();
    if (!kids.empty()) {
      const auto number = [](const std::string &s) {
        return !s.empty() &&
               std::all_of(s.begin(), s.end(), [](unsigned char c) { return std::isdigit(c); });
      };
      if (std::all_of(kids.begin(), kids.end(), number))
        std::sort(kids.begin(), kids.end(), [](const std::string &a, const std::string &b) {
          return a.size() != b.size() ? a.size() < b.size() : a < b;
        });
      for (const std::string &k : kids)
        out.push_back(read_string(app, path + cvc::state::SEPARATOR + k));
      return out;
    }
    std::string item;
    for (const char c : read_string(app, path)) {
      if (c == ' ' || c == '\t' || c == '\n' || (commas && c == ',')) {
        if (!item.empty())
          out.push_back(std::move(item));
        item.clear();
      } else {
        item.push_back(c);
      }
    }
    if (!item.empty())
      out.push_back(std::move(item));
  } catch (const std::exception &) {
  }
  return out;
}

// `uri` as an id: its last path segment without its extension, made a clean segment.
std::string mount_id_from_uri(const std::string &uri) {
  std::string s = uri;
  if (const std::size_t q = s.find_first_of("?#"); q != std::string::npos)
    s = s.substr(0, q);
  while (!s.empty() && (s.back() == '/' || s.back() == '\\'))
    s.pop_back();
  if (const std::size_t slash = s.find_last_of("/\\"); slash != std::string::npos)
    s = s.substr(slash + 1);
  if (const std::size_t dot = s.find_last_of('.'); dot != std::string::npos && dot != 0)
    s = s.substr(0, dot);
  std::string out;
  for (const char c : s)
    out.push_back(std::isalnum(static_cast<unsigned char>(c)) || c == '_' ? c : '_');
  if (out.empty())
    out = "app";
  if (std::isdigit(static_cast<unsigned char>(out.front())))
    out.insert(out.begin(), '_');
  return out;
}

// How many mounts deep `sub` is below `prefix` (its "includes" segments).
int mount_depth(const std::string &sub, const std::string &prefix) {
  const std::string rel = cvc::state_exec::process_scope(sub, prefix);
  int n = 0;
  std::size_t at = 0;
  while (at <= rel.size()) {
    std::size_t dot = rel.find(cvc::state::SEPARATOR[0], at);
    if (dot == std::string::npos)
      dot = rel.size();
    if (rel.compare(at, dot - at, "includes") == 0)
      ++n;
    at = dot + 1;
  }
  return n;
}

// A number at `path`, or `def` when there is none (or it is not a number).
double read_number_or(cvc::app &app, const std::string &path, double def) {
  const std::string s = read_string(app, path);
  if (s.empty())
    return def;
  try {
    std::size_t used = 0;
    const double v = std::stod(s, &used);
    return used > 0 && std::isfinite(v) ? v : def;
  } catch (const std::exception &) {
    return def;
  }
}
} // namespace

Runtime::MountBudget Runtime::default_mount_budget(const std::string &principal) {
  // Settled against the shipped examples (docs/ARIADNE.md §6.8): measured mounted in the shell, the
  // busiest one's frame stays far under the local default. A document from a file or the network
  // gets a fifth of the steps and the time, and a quarter of the memory.
  MountBudget b;
  if (principal == "local") {
    b.steps = 100000;
    b.time = 0.025;
    b.memory = std::uint64_t{256} << 20;
  } else {
    b.steps = 20000;
    b.time = 0.005;
    b.memory = std::uint64_t{64} << 20;
  }
  return b;
}

// The URIs a slot lists (allow:, catalog:): its literal list, else the bound key's children's
// values (a list as state-set-tree writes one), else the key's own value.
std::vector<std::string> Runtime::Impl::mount_list(const std::vector<std::string> &literal,
                                                   const std::string &bind,
                                                   const std::string &scope) const {
  if (bind.empty())
    return literal;
  std::vector<std::string> out;
  const std::string key = resolve_bind(scope, bind);
  try {
    if (cvc::state *node = cvc::state::instance(app).findDescendant(key)) {
      for (const std::string &c : node->childNames()) {
        const std::string v = read_string(app, key + cvc::state::SEPARATOR + c);
        if (!v.empty())
          out.push_back(v);
      }
      if (out.empty()) {
        const std::string v = read_string(app, key);
        if (!v.empty())
          out.push_back(v);
      }
    }
  } catch (const std::exception &) {
  }
  return out;
}

// A URI as the slot compares it: a local path made absolute against `base` and made canonical the
// way the resolver makes a document's own path (so `base` and the entry's path are in one form:
// on Windows a short name such as RUNNER~1 is spelled out, and a link is followed). A path written
// with a trailing slash keeps it, so an allow: prefix matches only under that directory. Any other
// scheme's as written.
std::string Runtime::Impl::mount_norm(const std::string &uri, const std::string &base) const {
  const Uri u = parse_uri(uri);
  if (u.scheme != "file")
    return uri;
  std::filesystem::path p(u.path);
  if (p.is_relative() && !base.empty())
    p = std::filesystem::path(base) / p;
  std::error_code ec;
  const std::filesystem::path canon = std::filesystem::weakly_canonical(p, ec);
  std::string out = (ec ? p.lexically_normal() : canon).generic_string();
  const bool dir = !u.path.empty() && (u.path.back() == '/' || u.path.back() == '\\');
  if (dir && (out.empty() || out.back() != '/'))
    out += '/';
  return out;
}

// 4.12 who a mount runs as, from where its bytes came: `local` for a cvc:// or state:// document
// and for one the slot's catalog: lists, `remote:<scheme>://<host>` for one fetched from the
// network, `file:<its directory>` for any other local file.
std::string Runtime::Impl::mount_principal(const Widget &w, const std::string &scope,
                                           const std::string &uri,
                                           const std::string &canonical) const {
  const Uri u = parse_uri(uri);
  if (u.scheme == "cvc" || u.scheme == "state")
    return "local";
  if (u.scheme == "http" || u.scheme == "https")
    return "remote:" + u.scheme + "://" + u.path.substr(0, u.path.find('/'));
  const MountsSpec none{}; // a slot without a spec (a local, so no static state)
  const MountsSpec &spec = w.mounts ? *w.mounts : none;
  const std::string mine = mount_norm(uri, spec.base);
  for (const std::string &c : mount_list(spec.catalog, spec.catalog_bind, scope)) {
    const std::string entry = mount_norm(c, spec.base);
    if (entry == mine || (!entry.empty() && entry.back() == '/' && mine.rfind(entry, 0) == 0))
      return "local";
  }
  const std::string path = canonical.empty() ? mine : canonical;
  return "file:" + std::filesystem::path(path).parent_path().generic_string();
}

// 4.11 a mount's budget: its principal's default, each part the entry's budget.<part> when set.
Runtime::MountBudget Runtime::Impl::mount_budget(const MountSlot &slot) const {
  Runtime::MountBudget b = Runtime::default_mount_budget(slot.principal);
  const std::string at = slot.slot_key + cvc::state::SEPARATOR + slot.as + cvc::state::SEPARATOR +
                         "budget" + cvc::state::SEPARATOR;
  const double steps = read_number_or(app, at + "steps", -1.0);
  if (steps >= 0.0)
    b.steps = static_cast<std::uint64_t>(steps);
  const double time = read_number_or(app, at + "time", -1.0);
  if (time >= 0.0)
    b.time = time;
  const double memory = read_number_or(app, at + "memory", -1.0);
  if (memory >= 0.0)
    b.memory = static_cast<std::uint64_t>(memory);
  return b;
}

void Runtime::Impl::apply_mount_budget(MountSlot &slot) {
  if (!slot.account)
    return;
  slot.budget = mount_budget(slot);
  slot.account->max_steps.store(slot.budget.steps, std::memory_order_relaxed);
  slot.account->max_seconds.store(slot.budget.time, std::memory_order_relaxed);
  slot.account->max_memory.store(slot.budget.memory, std::memory_order_relaxed);
}

std::shared_ptr<cvc::state_exec::frame_account>
Runtime::Impl::account_for(const std::string &scope) const {
  // The deepest mounts: entry the scope is under: a mount's own nested mounts run on its budget.
  const std::string *best = nullptr;
  std::shared_ptr<cvc::state_exec::frame_account> account;
  for (const auto &[sub, slot] : mount_slots_)
    if (slot.account && path_under(scope, sub) && (!best || sub.size() > best->size())) {
      best = &sub;
      account = slot.account;
    }
  return account;
}

// The entry's status for the slot's document to show: <slot_key>.<as>.status (open, refused) and
// .error (why it was refused), .principal and .name.
void Runtime::Impl::publish_mount_status(const MountSlot &slot) {
  const std::string at = slot.slot_key + cvc::state::SEPARATOR + slot.as + cvc::state::SEPARATOR;
  try {
    write<std::string>(app, at + "status", slot.wrapper ? "open" : "refused");
    write<std::string>(app, at + "error", slot.refused);
    write<std::string>(app, at + "principal", slot.principal);
    write<std::string>(app, at + "name", slot.name);
    // ARIADNE_SHELL.md 4.5 the grant tokens it holds, and 4.12 the origins it reaches beyond its
    // principal's, each a comma-separated list ("" when refused).
    std::string holds, origins;
    if (slot.wrapper && slot.wrapper->grants)
      for (const std::string &t : slot.wrapper->grants->held)
        holds += (holds.empty() ? "" : ",") + t;
    else if (slot.wrapper)
      for (const std::string &t : default_mount_grants_.held)
        holds += (holds.empty() ? "" : ",") + t;
    for (const std::string &o : slot.wrapper ? slot.net_origins : std::vector<std::string>{})
      origins += (origins.empty() ? "" : ",") + o;
    write<std::string>(app, at + "holds", holds);
    write<std::string>(app, at + "net_origins", origins);
  } catch (const std::exception &) {
  }
}

MountRequest Runtime::Impl::mount_request(const MountSlot &slot, const MountsSpec &spec) const {
  MountRequest rq;
  rq.uri = slot.uri;
  rq.as = slot.as;
  rq.args = slot.args;
  rq.base_dir = spec.base;
  // 4.5 what the entry is granted: its slot's grants: (the defaults without one), picked by its
  // own grants, which the loader bounds by what the document holding the slot holds.
  GrantSet granted = spec.grants ? *spec.grants : default_grant_set();
  if (slot.has_grants)
    granted = pick_tokens(std::move(granted), slot.grants);
  rq.grants = std::make_shared<const GrantSet>(std::move(granted));
  if (const std::string up = app_of(slot.parent_scope); !up.empty())
    if (const MountGrants *g = grants_of(up))
      rq.held_by = std::make_shared<const MountGrants>(*g);
  // 4.6 and its command line.
  rq.has_argv = slot.has_argv;
  rq.argv = slot.argv;
  return rq;
}

void Runtime::Impl::refuse_mount(MountSlot &slot, const std::string &why) {
  slot.wrapper.reset();
  slot.account.reset();
  slot.refused = why;
  publish_mount_status(slot);
  // 4.2 a refusal is posted to the feed, by the document that holds the slot.
  cvc::notification n;
  n.text = "Not opened: " + slot.uri + ": " + why;
  n.level = 1;
  n.path = slot.parent_scope;
  n.source = notify_source(slot.parent_scope);
  cvc::notify(app, std::move(n));
}

void Runtime::Impl::open_mount(MountSlot &slot, const Widget &w) {
  slot.name = slot.as;
  // allow: (4.2) — by default any file or link; a list narrows it to its prefixes.
  const MountsSpec none{}; // a slot without a spec (a local, so no static state)
  const MountsSpec &spec = w.mounts ? *w.mounts : none;
  const std::vector<std::string> allow = mount_list(spec.allow, spec.allow_bind, slot.parent_scope);
  if (!allow.empty()) {
    const std::string mine = mount_norm(slot.uri, spec.base);
    const bool ok = std::any_of(allow.begin(), allow.end(), [&](const std::string &a) {
      const std::string pre = mount_norm(a, spec.base);
      return !pre.empty() && mine.rfind(pre, 0) == 0;
    });
    if (!ok)
      return refuse_mount(slot, "it is not in this document's allow: list");
  }
  int open = 0;
  for (const auto &[sub, s] : mount_slots_)
    open += s.wrapper ? 1 : 0;
  if (open >= kMaxOpenMounts)
    return refuse_mount(slot, "too many mounts are open (" + std::to_string(kMaxOpenMounts) + ")");
  if (mount_depth(resolve_bind(slot.parent_scope, "includes." + slot.as), prefix) > kMaxMountDepth)
    return refuse_mount(slot, "it is mounted too deep (" + std::to_string(kMaxMountDepth) + ")");
  if (!mount_loader_)
    return refuse_mount(slot, "this host does not mount documents (it has no loader)");
  const MountRequest rq = mount_request(slot, spec);
  MountResult r;
  try {
    r = mount_loader_(rq);
  } catch (const std::exception &e) {
    r.ok = false;
    r.error = e.what();
  }
  const std::string sub = resolve_bind(slot.parent_scope, "includes." + slot.as);
  for (const std::string &warning : r.warnings) { // its own parse's, named as the mount's
    cvc::notification n;
    n.text = warning;
    n.level = 1;
    n.path = sub;
    n.source = slot.as;
    cvc::notify(app, std::move(n));
  }
  slot.warnings = r.warnings;
  slot.canonical = r.canonical;
  slot.sources = std::move(r.sources);
  slot.stamps = std::move(r.source_stamps);
  if (!r.ok)
    return refuse_mount(slot, r.error.empty() ? "it could not be loaded" : r.error);
  slot.principal = mount_principal(w, slot.parent_scope, slot.uri, slot.canonical);
  slot.wrapper = std::make_unique<Widget>(std::move(r.wrapper));
  if (!slot.wrapper->app_name.empty())
    slot.name = slot.wrapper->app_name;
  slot.refused.clear();
  // 4.12 the origins it holds beyond its principal's default (SH11 reaches them), and its tests:.
  slot.net_origins =
      slot.wrapper->grants ? slot.wrapper->grants->net_origins : std::vector<std::string>{};
  slot.tests = std::move(r.tests);
  slot.account = std::make_shared<cvc::state_exec::frame_account>();
  apply_mount_budget(slot);
  publish_mount_status(slot);
  // As an application runs whether its windows show or not, its state: is written (§5.3), its
  // init: runs and its on_tick: starts as it opens, not as it is first drawn, as a load: module's
  // would. A reopened one resumes: its state: and init: ran already.
  mount_names[sub] = slot.name;
  const std::string fns = slot.wrapper->functions ? slot.wrapper->functions->prelude : "";
  if (!fns.empty())
    mount_functions[sub] = fns;
  const Widget &wrapper = *slot.wrapper;
  publish_command_line(sub, wrapper); // 4.6 its argv and args, before its init: reads them
  if ((!wrapper.state_seeds.empty() || !wrapper.init_script.empty()) &&
      ran_inits.insert(sub).second) {
    seed_state(app, sub, wrapper.state_seeds); // just before init:, which may override them
    if (!wrapper.init_script.empty())
      run_mount_init(sub, fns + wrapper.init_script);
  }
  if (!slot.wrapper->tick_script.empty())
    ensure_mount_tick(sub, *slot.wrapper);
}

// 4.2 Closing: the mount is quit, every process under it killed, and the slot freed with its
// widgets; its state stays under includes.<as>, so the same entry resumes it. `forget_init` runs
// its init: again when it opens next (another document at the same id).
void Runtime::Impl::close_mount(const std::string &sub, bool forget_init) {
  const auto it = mount_slots_.find(sub);
  if (it == mount_slots_.end())
    return;
  quit_app(sub);
  // What quit_app leaves: the children its programs spawned or forked, which share its scope.
  if (group_ || used_scheduler_) {
    try {
      cvc::state_exec::async_scheduler &sched = current_sched();
      const std::string rel = cvc::state_exec::process_scope(sub, prefix);
      for (const cvc::state_exec::process_info &p : sched.list_processes())
        if (p.owner == owner_ && !cvc::state_exec::is_finished(p.status) &&
            path_under(p.scope, rel))
          sched.kill(p.pid);
    } catch (const std::exception &) {
    }
  }
  for (auto q = quit_apps.begin(); q != quit_apps.end();)
    q = path_under(*q, sub) ? quit_apps.erase(q) : std::next(q);
  // 4.6 the keyboard focus, the keys held down and the pointer's button go with it.
  if (!focused_app_.empty() && path_under(focused_app_, sub))
    focused_app_.clear();
  if (!pointer_app_.empty() && path_under(pointer_app_, sub))
    pointer_app_.clear();
  std::erase_if(keys_down_, [&](const auto &kv) { return path_under(kv.second, sub); });
  for (std::map<std::string, std::string> *m : {&mount_names, &mount_functions})
    for (auto e = m->begin(); e != m->end();)
      e = path_under(e->first, sub) ? m->erase(e) : std::next(e);
  if (forget_init)
    for (auto e = ran_inits.begin(); e != ran_inits.end();)
      e = path_under(*e, sub) ? ran_inits.erase(e) : std::next(e);
  mount_instances_.clear(); // made from templates that may be the mount's
  // Its nested slots' mounts go with it (their slot was in its tree).
  for (auto s = mount_slots_.begin(); s != mount_slots_.end();) {
    if (s->first != sub && path_under(s->first, sub)) {
      const std::string nested = s->first;
      ++s;
      close_mount(nested, forget_init);
      s = mount_slots_.upper_bound(nested);
    } else {
      ++s;
    }
  }
  const auto again = mount_slots_.find(sub);
  if (again != mount_slots_.end()) {
    try {
      if (cvc::state *entry = cvc::state::instance(app).findDescendant(
              again->second.slot_key + cvc::state::SEPARATOR + again->second.as))
        if (cvc::state *st = entry->findDescendant("status"))
          st->value(std::string("closed"));
    } catch (const std::exception &) {
    }
    mount_slots_.erase(again);
  }
}

// One pass: read every slot's entries, close what went or changed (and return), else open what is
// new and parse again what changed on disk. True when anything changed: the caller plans the
// windows menu again, which finds the slots of the mounts just opened, and calls again.
bool Runtime::Impl::reconcile_mounts_once() {
  struct Want {
    const Widget *w;
    std::string key, scope, as, uri, sub;
    std::vector<std::pair<std::string, std::string>> args;
    bool has_grants = false, has_argv = false; // ARIADNE_SHELL.md 4.5, 4.6
    std::vector<std::string> grants, argv;
  };
  std::vector<Want> want;
  std::set<std::string> wanted;
  std::string doc_key;
  cvc::state &tree = cvc::state::instance(app);
  for (const auto &[w, scope] : mounts_widgets_) {
    if (w->bind.empty())
      continue;
    const std::string key = resolve_bind(scope, w->bind);
    if (doc_key.empty() && scope == prefix)
      doc_key = key;
    cvc::state *node = nullptr;
    std::vector<std::string> children;
    try {
      node = tree.findDescendant(key);
      if (node)
        children = node->childNames();
    } catch (const std::exception &) {
      continue;
    }
    for (const std::string &as : children) {
      const std::string entry = key + cvc::state::SEPARATOR + as;
      const std::string uri = read_string(app, entry + cvc::state::SEPARATOR + "uri");
      if (uri.empty())
        continue; // not an entry (yet)
      if (!clean_mount_id(as)) {
        warn_once("ari: mounts: entry '" + as + "' under " + key +
                  " is not an id ([A-Za-z0-9_], not starting with a digit) — not mounted");
        continue;
      }
      Want x{w, key, scope, as, uri, resolve_bind(scope, "includes." + as), {}};
      try {
        if (cvc::state *args = tree.findDescendant(entry + cvc::state::SEPARATOR + "args"))
          for (const std::string &a : args->childNames())
            x.args.emplace_back(a, read_string(app, entry + cvc::state::SEPARATOR + "args" +
                                                        cvc::state::SEPARATOR + a));
      } catch (const std::exception &) {
      }
      // Its own grants (picked from the slot's) and its argv, when it names them.
      x.grants = entry_list(app, entry + cvc::state::SEPARATOR + "grants", x.has_grants, true);
      x.argv = entry_list(app, entry + cvc::state::SEPARATOR + "argv", x.has_argv, false);
      if (!wanted.insert(x.sub).second) {
        warn_once("ari: two mounts: slots mount '" + as + "' in one scope — the first is kept");
        continue;
      }
      want.push_back(std::move(x));
    }
  }
  doc_slot_->set(doc_key);
  // Close the mounts whose entry went, or whose document or args changed, deepest first.
  std::vector<std::pair<std::string, bool>> closing; // sub-prefix, a different document there
  for (const auto &[sub, slot] : mount_slots_) {
    const auto x =
        std::find_if(want.begin(), want.end(), [&](const Want &v) { return v.sub == sub; });
    if (x == want.end() || x->key != slot.slot_key)
      closing.emplace_back(sub, false);
    else if (x->uri != slot.uri || x->args != slot.args || x->has_argv != slot.has_argv ||
             x->argv != slot.argv) // another command line: its state: and init: as at a new run
      closing.emplace_back(sub, true);
    else if (x->has_grants != slot.has_grants || x->grants != slot.grants)
      closing.emplace_back(sub, false); // other grants: opened again with them, resumed
  }
  if (!closing.empty()) {
    std::sort(closing.begin(), closing.end(),
              [](const auto &a, const auto &b) { return a.first.size() > b.first.size(); });
    for (const auto &[sub, different] : closing)
      close_mount(sub, different);
    return true;
  }
  bool changed = false;
  for (const Want &x : want) {
    const auto it = mount_slots_.find(x.sub);
    if (it == mount_slots_.end()) {
      MountSlot slot;
      slot.slot_key = x.key;
      slot.parent_scope = x.scope;
      slot.as = x.as;
      slot.uri = x.uri;
      slot.args = x.args;
      slot.has_grants = x.has_grants;
      slot.grants = x.grants;
      slot.has_argv = x.has_argv;
      slot.argv = x.argv;
      MountSlot &s = mount_slots_.emplace(x.sub, std::move(slot)).first->second;
      open_mount(s, *x.w);
      changed = true;
      continue;
    }
    MountSlot &s = it->second;
    if (s.stale && s.wrapper) {
      // 4.2 Caching: parsed again only now that a file changed. Its nested slots close first
      // (they live in the old tree) and reopen from the new one.
      s.stale = false;
      bool nested = false;
      for (const auto &[sub, o] : mount_slots_)
        nested = nested || (sub != x.sub && path_under(sub, x.sub));
      if (nested) {
        for (auto n = mount_slots_.begin(); n != mount_slots_.end();) {
          if (n->first != x.sub && path_under(n->first, x.sub)) {
            const std::string sub = n->first;
            close_mount(sub, false);
            n = mount_slots_.lower_bound(sub);
          } else {
            ++n;
          }
        }
        mount_slots_.at(x.sub).stale = true; // parsed at the next pass, with the plan made again
        return true;
      }
      const MountsSpec none{}; // a slot without a spec (a local, so no static state)
      const MountRequest rq = mount_request(s, x.w->mounts ? *x.w->mounts : none);
      MountResult r;
      try {
        r = mount_loader_ ? mount_loader_(rq) : MountResult{};
      } catch (const std::exception &e) {
        r.error = e.what();
      }
      s.stamps = r.source_stamps.empty() ? s.stamps : r.source_stamps;
      if (!r.ok) { // a file caught mid-edit: the running mount stays as it was
        warn_once("ari: mount '" + s.as + "' was not reloaded: " + r.error);
        continue;
      }
      s.sources = std::move(r.sources);
      s.canonical = r.canonical;
      s.wrapper = std::make_unique<Widget>(std::move(r.wrapper));
      if (!s.wrapper->app_name.empty())
        s.name = s.wrapper->app_name;
      // 4.5, 4.6 what it holds now, and its command line as the new text has it.
      s.net_origins =
          s.wrapper->grants ? s.wrapper->grants->net_origins : std::vector<std::string>{};
      s.tests = std::move(r.tests);
      publish_mount_status(s);
      publish_command_line(x.sub, *s.wrapper);
      mount_instances_.clear();
      // Its on_tick: resident stays when its program is the same (drop_mount_ticks compares).
      drop_mount_ticks();
      if (!s.wrapper->tick_script.empty())
        ensure_mount_tick(x.sub, *s.wrapper);
      const std::string fns = s.wrapper->functions ? s.wrapper->functions->prelude : "";
      if (!fns.empty())
        mount_functions[x.sub] = fns;
      changed = true;
      continue;
    }
    if (s.wrapper)
      apply_mount_budget(s); // its entry's budget.* may have changed
  }
  return changed;
}

void Runtime::Impl::reconcile_mounts() {
  if (mounts_widgets_.empty() && mount_slots_.empty())
    return;
  // A pass per level of nesting: a mount opened in one pass has its own slots found by the plan
  // made after it. Bounded; what is left waits for the next frame.
  for (int pass = 0; pass < 8; ++pass) {
    if (!reconcile_mounts_once())
      break;
    plan_windows_menu();
  }
}

// ── asks and grants: what a mount holds, its input, signals and command line ────────────────────
// (docs/roadmap/ARIADNE_SHELL.md 4.5, 4.6)

const MountGrants *Runtime::Impl::grants_of(const std::string &sub) const {
  if (const auto s = mount_slots_.find(sub); s != mount_slots_.end()) {
    const Widget *w = s->second.wrapper.get();
    return !w ? nullptr : w->grants ? w->grants.get() : &default_mount_grants_;
  }
  if (const auto l = load_grants_.find(sub); l != load_grants_.end())
    return l->second ? l->second.get() : &default_mount_grants_;
  return nullptr;
}

std::string Runtime::Impl::app_of(const std::string &scope) const {
  std::string best;
  for (const auto &[sub, slot] : mount_slots_)
    if (slot.wrapper && sub.size() > best.size() && path_under(scope, sub))
      best = sub;
  for (const auto &[sub, g] : load_grants_)
    if (sub.size() > best.size() && path_under(scope, sub))
      best = sub;
  return best;
}

std::shared_ptr<const std::set<std::string>>
Runtime::Impl::held_at(const std::string &scope) const {
  const std::string sub = app_of(scope);
  if (sub.empty())
    return nullptr;
  const MountGrants *g = grants_of(sub);
  return std::make_shared<const std::set<std::string>>(g ? g->held : std::set<std::string>{});
}

void Runtime::Impl::publish_command_line(const std::string &sub, const Widget &wrapper) {
  // Its keys: bindings' defaults too, where nothing has rebound them yet (keys.h): its keys live at
  // <sub>.keys.<id>, its own, as the document's live at keys.<id>.
  if (wrapper.grants)
    for (const KeyBinding &b : wrapper.grants->keys) {
      const std::string at = resolve_bind(sub, key_binding_path(b));
      try {
        if (!cvc::state::instance(app).findDescendant(at))
          write<std::string>(app, at, join_keys(b.keys));
      } catch (const std::exception &) {
      }
    }
  if (!wrapper.command_line)
    return;
  const MountCommandLine &cl = *wrapper.command_line;
  try {
    publish_document_args(app, sub, cl.argv, cl.has_args_block ? &cl.args : nullptr,
                          cl.has_args_block ? &cl.parsed : nullptr);
  } catch (const std::exception &e) {
    warn_once("ari: mount '" + sub + "': its command line was not published: " + e.what());
  }
}

void Runtime::Impl::end_frame_focus() {
  hover_scope_ = std::move(frame_hover_);
  keyboard_scope_ = std::move(frame_keyboard_);
  // The focus follows the window that has it, when the backend says which: to its application,
  // or off every application when it went to the document's own window or to none. Between moves
  // it stays where (app-focus) put it.
  if (backend && backend->reports_focus()) {
    if (frame_focus_ != last_focus_window_)
      focused_app_ = frame_focus_ ? app_of(*frame_focus_) : std::string();
    last_focus_window_ = frame_focus_;
  }
  frame_focus_.reset();
  frame_hover_.reset();
  frame_keyboard_.reset();
  if (!focused_app_.empty() && !grants_of(focused_app_))
    focused_app_.clear(); // its mount went
  // The focused application, for the document to show: its scope ("includes.<as>"), as the
  // ariadne.apps mirror names it.
  const std::string focus = focused_app_.empty() ? std::string()
                            : prefix.empty()     ? focused_app_
                            : path_under(focused_app_, prefix) && focused_app_ != prefix
                                ? focused_app_.substr(prefix.size() + 1)
                                : focused_app_;
  const std::string at = resolve_bind(prefix, "ariadne.focus");
  if (read_or<std::string>(app, at, std::string(1, '\x01')) != focus)
    write<std::string>(app, at, focus);
  want_mount_input();
}

void Runtime::Impl::want_mount_input() {
  const auto takes = [this](const std::string &sub) {
    const MountGrants *g = grants_of(sub);
    return g && ((g->holds("delegate.keys") && (!g->on_key.empty() || !g->keys.empty())) ||
                 (g->holds("delegate.pointer") && !g->on_pointer.empty()));
  };
  bool want = false;
  for (const auto &[sub, slot] : mount_slots_)
    want = want || takes(sub);
  for (const auto &[sub, g] : load_grants_)
    want = want || takes(sub);
  mount_input_wanted_.store(want);
}

void Runtime::Impl::run_in_mount(
    const std::string &sub, const std::string &body,
    std::vector<std::pair<std::string, cvc::state_exec::value_t>> fields) {
  QueuedAction a;
  // A handler's body may be several forms, as the document's on_key: is; a binding's on: is a
  // program or a bare event name, as it is as the document's.
  a.event = fields.empty() && !is_expr(body) ? body : "(begin\n" + body + "\n)";
  a.prefix = sub;
  a.event_values = std::move(fields);
  if (const auto fn = mount_functions.find(sub); fn != mount_functions.end())
    a.functions = fn->second;
  queued_events.push_back(std::move(a));
}

namespace {
// An input event as the dict an on_key:/on_pointer: program reads as `event` (deliver_input's).
std::vector<std::pair<std::string, cvc::state_exec::value_t>> input_fields(const InputEvent &ev) {
  namespace se = cvc::state_exec;
  std::vector<std::pair<std::string, se::value_t>> f;
  f.emplace_back("kind", se::value_t(std::string(ev.kind_name())));
  f.emplace_back("key", se::value_t(ev.key));
  f.emplace_back("mods", se::value_t(static_cast<int64_t>(ev.mods)));
  f.emplace_back("repeat", se::value_t(ev.repeat));
  f.emplace_back("x", se::value_t(static_cast<double>(ev.x)));
  f.emplace_back("y", se::value_t(static_cast<double>(ev.y)));
  f.emplace_back("dx", se::value_t(static_cast<double>(ev.dx)));
  f.emplace_back("dy", se::value_t(static_cast<double>(ev.dy)));
  f.emplace_back("button", se::value_t(static_cast<int64_t>(ev.button)));
  f.emplace_back("clicks", se::value_t(static_cast<int64_t>(ev.clicks)));
  return f;
}
} // namespace

void Runtime::Impl::route_mount_input() {
  std::vector<InputEvent> events;
  {
    std::lock_guard<std::mutex> lock(mount_input_mutex_);
    events.swap(mount_input_);
  }
  if (events.empty())
    return;
  // A binding of the document's, or of a mount between it and `target`, that has `key`: the key
  // is theirs, and never reaches the application (4.6).
  const auto bound_above = [this](const std::string &target, const std::string &key) {
    for (const KeyBinding &b : key_bindings_)
      if (key_matches(read_or<std::string>(app, resolve_bind(prefix, key_binding_path(b)), ""),
                      key))
        return true;
    for (std::string up = app_of(target.substr(0, target.rfind('.'))); !up.empty();
         up = app_of(up.substr(0, up.rfind('.'))))
      if (const MountGrants *g = grants_of(up))
        for (const KeyBinding &b : g->keys)
          if (key_matches(read_or<std::string>(app, resolve_bind(up, key_binding_path(b)), ""),
                          key))
            return true;
    return false;
  };
  const bool capture = key_capture_pending(); // the Keys window waits for a key: it is its
  for (const InputEvent &ev : events) {
    if (ev.is_keyboard()) {
      if (capture)
        continue;
      const std::string key = canonical_key(ev.key);
      if (ev.kind == InputEvent::Kind::KeyUp) { // to the application its key went down to
        const auto it = keys_down_.find(key);
        if (it == keys_down_.end())
          continue;
        const std::string target = it->second;
        keys_down_.erase(it);
        const MountGrants *g = grants_of(target);
        if (g && g->holds("delegate.keys") && !g->on_key.empty())
          run_in_mount(target, g->on_key, input_fields(ev));
        continue;
      }
      // A key goes to the focused application, when it holds delegate.keys, no text field outside
      // it has the keyboard, and no binding above it has the key.
      const std::string &target = focused_app_;
      const MountGrants *g = target.empty() ? nullptr : grants_of(target);
      if (!g || !g->holds("delegate.keys") || !app_running(target))
        continue;
      if (keyboard_scope_ && !path_under(*keyboard_scope_, target))
        continue;
      if (bound_above(target, ev.key))
        continue;
      keys_down_[key] = target;
      if (!g->on_key.empty())
        run_in_mount(target, g->on_key, input_fields(ev));
      for (const KeyBinding &b : g->keys)
        if (!b.on.empty() &&
            key_matches(read_or<std::string>(app, resolve_bind(target, key_binding_path(b)), ""),
                        ev.key))
          run_in_mount(target, b.on, {});
      continue;
    }
    // The pointer: to the application whose window it is over, when it holds delegate.pointer; a
    // button's moves and its release to the one it went down over.
    std::string target = hover_scope_ ? app_of(*hover_scope_) : std::string();
    if (ev.kind == InputEvent::Kind::MouseButtonDown)
      pointer_app_ = target;
    else if (!pointer_app_.empty() &&
             (ev.kind == InputEvent::Kind::MouseMove || ev.kind == InputEvent::Kind::MouseButtonUp))
      target = pointer_app_;
    if (ev.kind == InputEvent::Kind::MouseButtonUp)
      pointer_app_.clear();
    const MountGrants *g = target.empty() ? nullptr : grants_of(target);
    if (g && g->holds("delegate.pointer") && !g->on_pointer.empty() && app_running(target))
      run_in_mount(target, g->on_pointer, input_fields(ev));
  }
}

bool Runtime::Impl::signal_mount(const std::string &sub, const std::string &signal,
                                 const std::string &source) {
  namespace se = cvc::state_exec;
  const MountGrants *g = grants_of(sub);
  if (!g || g->on_signal.empty() || !g->takes_signal(signal) || !app_running(sub))
    return false;
  run_in_mount(sub, g->on_signal,
               {{"kind", se::value_t(std::string("signal"))},
                {"signal", se::value_t(signal)},
                {"source", se::value_t(source)}});
  return true;
}

bool Runtime::Impl::signal_mounts(const std::string &signal, const std::string &source) {
  const std::string name = signal_name(signal);
  // An interrupt (Ctrl-C, Ctrl-\) is the focused application's, as a terminal's is its foreground
  // program's: taken, it does not end the shell.
  if (name == "SIGINT" || name == "SIGQUIT")
    return !focused_app_.empty() && signal_mount(focused_app_, signal, source);
  // A request to end goes to every one, so each can save: their handlers run in the next drain,
  // before the host's default ends the shell.
  std::vector<std::string> subs;
  for (const auto &[sub, slot] : mount_slots_)
    if (slot.wrapper)
      subs.push_back(sub);
  for (const auto &[sub, g] : load_grants_)
    subs.push_back(sub);
  for (const std::string &sub : subs)
    signal_mount(sub, signal, source);
  return false;
}

void Runtime::Impl::take_app_requests() {
  std::optional<std::string> focus;
  std::vector<std::pair<std::string, std::string>> signals;
  {
    std::lock_guard<std::mutex> lock(app_requests_->m);
    focus.swap(app_requests_->focus);
    signals.swap(app_requests_->signals);
  }
  if (!focus && signals.empty())
    return;
  // The document's own application named AS: an entry of its first mounts: slot, else its load:
  // of that id.
  const std::string key = doc_slot_->get();
  const auto sub_of = [&](const std::string &as) -> std::string {
    for (const auto &[sub, slot] : mount_slots_)
      if (slot.slot_key == key && slot.as == as && slot.wrapper)
        return sub;
    const std::string sub = resolve_bind(prefix, "includes." + as);
    return load_grants_.count(sub) ? sub : std::string();
  };
  if (focus)
    focused_app_ = focus->empty() ? std::string() : sub_of(*focus);
  for (const auto &[as, signal] : signals)
    if (const std::string sub = sub_of(as); !sub.empty())
      signal_mount(sub, signal, "app-signal");
}

// 4.11 Each frame: every mount's memory (once every quarter second: the bytes each of its processes
// holds, summed into its account), how long it has been held to its budget, and after
// kMountHeldNotice seconds of it the shell's notice, whose click opens the Budgets window.
void Runtime::Impl::measure_mounts() {
  if (mount_slots_.empty())
    return;
  if (mounts_measured_at_ < 0.0 || frame_t - mounts_measured_at_ >= 0.25 ||
      frame_t < mounts_measured_at_) {
    mounts_measured_at_ = frame_t;
    std::map<std::string, std::uint64_t> bytes; // by sub-prefix
    if (group_ || used_scheduler_) {
      try {
        for (const cvc::state_exec::process_info &p : current_sched().list_processes()) {
          if (p.owner != owner_ || cvc::state_exec::is_finished(p.status))
            continue;
          const std::string abs = p.scope.empty() ? prefix : resolve_bind(prefix, p.scope);
          std::string best;
          for (const auto &[sub, slot] : mount_slots_)
            if (slot.account && path_under(abs, sub) && sub.size() > best.size())
              best = sub;
          if (!best.empty())
            bytes[best] += p.current_memory + p.heap_memory;
        }
      } catch (const std::exception &) {
      }
    }
    for (auto &[sub, slot] : mount_slots_)
      if (slot.account)
        slot.account->memory.store(bytes[sub], std::memory_order_relaxed);
  }
  for (auto &[sub, slot] : mount_slots_) {
    if (!slot.account)
      continue;
    const cvc::state_exec::frame_account &a = *slot.account;
    const bool held = a.last_held.load(std::memory_order_relaxed) ||
                      a.held.load(std::memory_order_relaxed) || a.spent();
    if (!held || !app_running(sub)) {
      slot.held_since = -1.0;
      slot.noticed = false;
      continue;
    }
    slot.by_memory =
        a.max_memory.load(std::memory_order_relaxed) > 0 &&
        a.memory.load(std::memory_order_relaxed) > a.max_memory.load(std::memory_order_relaxed);
    if (slot.held_since < 0.0 || frame_t < slot.held_since)
      slot.held_since = frame_t;
    if (!slot.noticed && frame_t - slot.held_since >= kMountHeldNotice) {
      slot.noticed = true;
      cvc::notification n;
      n.text = slot.name + " is using all the " + (slot.by_memory ? "memory" : "time") +
               " it is allowed. Click for Allow more, Quit or Restart.";
      n.level = 1;
      n.path = sub;
      n.source = notify_source(slot.parent_scope);
      n.on = kAppEvent + std::string("budget:") + sub;
      n.scope = prefix;
      cvc::notify(app, std::move(n));
    }
  }
}

// 4.11 the Budgets window, while tree._budgets.open is 1 (a notice's click sets it): each mount at
// its limit, with Allow more, Quit and Restart.
void Runtime::Impl::emit_budget_window() {
  const std::string open = resolve_bind(prefix, std::string("tree.") + kBudgetWindowId + ".open");
  if (mount_slots_.empty() || read_bool_or_seed(app, open, 0) == 0)
    return;
  std::vector<Widget> kids;
  for (const auto &[sub, slot] : mount_slots_) {
    if (!slot.account || slot.held_since < 0.0)
      continue;
    const cvc::state_exec::frame_account &a = *slot.account;
    std::string line = slot.name + " (" + slot.principal + "): ";
    line += slot.by_memory ? std::to_string(a.memory.load() >> 20) + " of " +
                                 std::to_string(slot.budget.memory >> 20) + " MB"
                           : std::to_string(a.last_steps.load()) + " of " +
                                 std::to_string(slot.budget.steps) + " steps a frame";
    kids.push_back(text(line));
    Widget row = group({button("Allow more##" + sub, kAppEvent + std::string("allow:") + sub),
                        button("Quit##" + sub, kAppEvent + std::string("quit:") + sub),
                        button("Restart##" + sub, kAppEvent + std::string("restart:") + sub)});
    row.layout.kind = LayoutKind::Horizontal;
    kids.push_back(std::move(row));
    kids.push_back(separator());
  }
  if (kids.empty())
    kids.push_back(text("No application is at its limit."));
  Widget w = window("Budgets", std::move(kids));
  w.id = kBudgetWindowId;
  w.closable = true;
  budget_window = std::move(w);
  emit(budget_window);
}

// ARIADNE_SHELL.md 4.2 the verbs, on the document that calls them (`scope` its programs' prefix):
//   (app-open URI [AS [ARGS]])  write the entry <slot>.<AS> (uri, and args.<name> from the ARGS
//                               dict) in the document's own first mounts: slot, and return AS;
//                               without AS one is made from the URI's name, and an entry already
//                               open on the same URI is returned as it is
//   (app-close AS)              remove the entry; #t when there was one
// A mounted document's programs get an error: opening others needs a grant (4.12 request.open).
void Runtime::Impl::bind_mount_verbs(const cvc::state_exec::environment_ptr &env,
                                     const std::string &scope) const {
  namespace se = cvc::state_exec;
  const bool own = scope == prefix;
  cvc::app *const on_app = &app;
  const auto check = [own, slot_key = doc_slot_](const char *verb) {
    if (!own)
      throw se::dsl_error("PermissionError",
                          std::string(verb) +
                              ": a mounted document opens and closes no applications without "
                              "a grant (docs/roadmap/ARIADNE_SHELL.md 4.12, request.open)");
    std::string key = slot_key->get();
    if (key.empty())
      throw se::dsl_error("RuntimeError",
                          std::string(verb) + ": this document has no mounts: slot");
    return key;
  };
  se::builtins::register_fn(
      env, "app-open", [on_app, check](std::span<const se::value_t> args) -> se::value_t {
        const std::string key = check("app-open");
        if (args.empty() || args.size() > 3)
          throw se::dsl_error("TypeError", "app-open: expected (app-open URI [AS [ARGS]])");
        const auto *uri = std::get_if<std::string>(&args[0].v);
        if (!uri || uri->empty())
          throw se::dsl_error("TypeError", "app-open: URI is a string");
        cvc::state &tree = cvc::state::instance(*on_app);
        const std::string sep = cvc::state::SEPARATOR;
        std::string as;
        if (args.size() > 1 && !std::holds_alternative<std::monostate>(args[1].v)) {
          const auto *s = std::get_if<std::string>(&args[1].v);
          if (!s)
            throw se::dsl_error("TypeError", "app-open: AS is a string");
          as = *s;
          if (!clean_mount_id(as))
            throw se::dsl_error("ValueError", "app-open: AS '" + as +
                                                  "' is not an id ([A-Za-z0-9_], not starting "
                                                  "with a digit)");
        } else {
          // One already open on this URI is the one asked for (4.9: picking a running one).
          cvc::state *slot = tree.findDescendant(key);
          if (slot)
            for (const std::string &c : slot->childNames())
              if (read_string(*on_app, key + sep + c + sep + "uri") == *uri)
                return se::value_t(c);
          // Taken: an entry with a uri. A child without one (its budget set ahead, say) is free.
          const auto taken = [&](const std::string &id) {
            return slot && !read_string(*on_app, key + sep + id + sep + "uri").empty();
          };
          const std::string base = mount_id_from_uri(*uri);
          as = base;
          for (int n = 2; taken(as); ++n)
            as = base + "_" + std::to_string(n);
        }
        const std::string entry = key + sep + as;
        try {
          if (cvc::state *old = tree.findDescendant(entry))
            old->removeChild("args"); // its args are the ones given now
        } catch (const std::exception &) {
        }
        if (args.size() > 2 && !std::holds_alternative<std::monostate>(args[2].v)) {
          const auto *d = std::get_if<se::dict_ptr>(&args[2].v);
          if (!d || !*d)
            throw se::dsl_error("TypeError", "app-open: ARGS is a dict");
          for (const auto &[name, v] : **d) {
            const se::value_t &value = v;
            const auto *s = std::get_if<std::string>(&value.v);
            write<std::string>(*on_app, entry + sep + "args" + sep + name,
                               s ? *s : se::to_string(value));
          }
        }
        write<std::string>(*on_app, entry + sep + "uri", *uri); // last: the entry is whole
        return se::value_t(as);
      });
  se::builtins::register_fn(
      env, "app-close", [on_app, check](std::span<const se::value_t> args) -> se::value_t {
        const std::string key = check("app-close");
        if (args.size() != 1)
          throw se::dsl_error("TypeError", "app-close: expected (app-close AS)");
        const auto *as = std::get_if<std::string>(&args[0].v);
        if (!as)
          throw se::dsl_error("TypeError", "app-close: AS is a string");
        try {
          if (cvc::state *slot = cvc::state::instance(*on_app).findDescendant(key))
            return se::value_t(slot->removeChild(*as));
        } catch (const std::exception &) {
        }
        return se::value_t(false);
      });
  // ARIADNE_SHELL.md 4.6 the document's own applications (its first mounts: slot's entries, else
  // its load:s by id): which one the keys go to, and a signal passed on, both taken by the next
  // drain. A mounted document's programs may do neither: the focus and the signals are its
  // parent's to give.
  const auto mine = [own](const char *verb) {
    if (!own)
      throw se::dsl_error("PermissionError",
                          std::string(verb) + ": a mounted document moves no focus and passes on "
                                              "no signals (docs/roadmap/ARIADNE_SHELL.md 4.6)");
  };
  const auto id_arg = [](const char *verb, const se::value_t &v, bool empty_ok) {
    const auto *s = std::get_if<std::string>(&v.v);
    if (!s)
      throw se::dsl_error("TypeError", std::string(verb) + ": AS is a string");
    if (!(empty_ok && s->empty()) && !clean_mount_id(*s))
      throw se::dsl_error("ValueError", std::string(verb) + ": AS '" + *s +
                                            "' is not an id ([A-Za-z0-9_], not starting with a "
                                            "digit)");
    return *s;
  };
  se::builtins::register_fn(
      env, "app-focus",
      [mine, id_arg, requests = app_requests_](std::span<const se::value_t> args) -> se::value_t {
        mine("app-focus");
        if (args.size() != 1)
          throw se::dsl_error("TypeError", "app-focus: expected (app-focus AS), \"\" for none");
        const std::string as = id_arg("app-focus", args[0], true);
        std::lock_guard<std::mutex> lock(requests->m);
        requests->focus = as;
        return se::value_t(true);
      });
  se::builtins::register_fn(
      env, "app-signal",
      [mine, id_arg, requests = app_requests_](std::span<const se::value_t> args) -> se::value_t {
        mine("app-signal");
        if (args.size() != 2)
          throw se::dsl_error("TypeError", "app-signal: expected (app-signal AS NAME)");
        const std::string as = id_arg("app-signal", args[0], false);
        const auto *name = std::get_if<std::string>(&args[1].v);
        if (!name || signal_name(*name).size() <= 3)
          throw se::dsl_error("TypeError", "app-signal: NAME is a signal's name (\"SIGTERM\")");
        std::lock_guard<std::mutex> lock(requests->m);
        requests->signals.emplace_back(as, signal_name(*name));
        return se::value_t(true);
      });
}

bool Runtime::Impl::bell_on_by_default() const {
  return windows_menu.bell_setting >= 0 ? windows_menu.bell_setting == 1 : notify_default;
}

namespace {
// §5.24 who an entry posted from `path` came from: the mount `path` is in (the deepest) among
// `mounts` (composed prefix -> name), named as the document named it; "" for the document itself
// or the host.
std::string mount_source(const std::map<std::string, std::string> &mounts,
                         const std::string &path) {
  std::string best_prefix, best;
  for (const auto &[p, name] : mounts)
    if ((path == p || (path.size() > p.size() && path.compare(0, p.size(), p) == 0 &&
                       path[p.size()] == cvc::state::SEPARATOR[0])) &&
        p.size() > best_prefix.size()) {
      best_prefix = p;
      best = name;
    }
  return best;
}
} // namespace

std::string Runtime::Impl::notify_source(const std::string &path) const {
  return mount_source(mount_names, path);
}

Runtime::Impl::NotifyScope Runtime::Impl::notify_scope(const std::string &scope) const {
  return NotifyScope{scope, notify_source(scope), mount_names};
}

namespace {
constexpr int kBellShown = 8; // the newest entries the bell's menu lists

std::chrono::system_clock::time_point epoch_time(double seconds) {
  return std::chrono::system_clock::time_point(
      std::chrono::duration_cast<std::chrono::system_clock::duration>(
          std::chrono::duration<double>(seconds)));
}

// An entry as a line: unread mark, level, source, its first line of text (cut at 80 characters),
// how many times it came, and when (`time_fmt`).
std::string notify_line(const cvc::notification_entry &e, const std::string &source, bool unicode,
                        const char *time_fmt) {
  std::string line = e.read ? "  " : (unicode ? "\xE2\x80\xA2 " : "* "); // a bullet: unread
  if (e.level >= 2)
    line += std::string(icon_glyph("error", unicode)) + " ";
  else if (e.level == 1)
    line += std::string(icon_glyph("warning", unicode)) + " ";
  if (!source.empty())
    line += source + ": ";
  std::string text = e.text.substr(0, e.text.find('\n'));
  if (text.size() > 80) {
    std::size_t cut = 77;
    while (cut > 0 && (static_cast<unsigned char>(text[cut]) & 0xC0) == 0x80)
      --cut; // not inside a UTF-8 sequence
    text = text.substr(0, cut) + "...";
  }
  line += text;
  if (e.repeat > 1)
    line += " (x" + std::to_string(e.repeat) + ")";
  line += "  " + local_time(epoch_time(e.time), time_fmt);
  return line;
}
} // namespace

void Runtime::Impl::emit_tray_items(const TrayEntry &icon) {
  if (icon.bell)
    emit_bell_items();
  else if (icon.w)
    emit_children(*icon.w);
}

// §5.24 the bell's menu: the newest entries (a click marks one read and runs its on:), then Mark
// all read, Clear all, and Show all..., which opens the Notifications window.
void Runtime::Impl::emit_bell_items() {
  Backend &b = *backend;
  const std::vector<cvc::notification_entry> all = cvc::notifications(app);
  int shown = 0;
  for (auto it = all.rbegin(); it != all.rend() && shown < kBellShown; ++it, ++shown) {
    const std::string line =
        notify_line(*it, it->source.empty() ? notify_source(it->path) : it->source,
                    b.unicode_glyphs(), "%H:%M");
    b.push_id(("##notify." + std::to_string(it->id)).c_str());
    b.next_item_copy(it->text); // §5.28 a right-click copies the whole message
    if (b.menu_item_action(line.c_str()))
      queued_events.push_back(
          {kNotifyEvent + std::string("open:") + std::to_string(it->id), prefix});
    b.pop_id();
  }
  if (all.size() > static_cast<std::size_t>(kBellShown))
    b.text_line(("and " + std::to_string(all.size() - kBellShown) + " more").c_str());
  b.separator();
  if (b.menu_item_action("Mark all read"))
    queued_events.push_back({kNotifyEvent + std::string("read_all"), prefix});
  if (b.menu_item_action("Clear all"))
    queued_events.push_back({kNotifyEvent + std::string("clear_all"), prefix});
  if (b.menu_item_action("Show all..."))
    queued_events.push_back({kNotifyEvent + std::string("show"), prefix});
}

// §5.24 the Notifications window, while its tree._notifications.open is 1 (Show all... sets it;
// its X clears it): every entry, newest first, under Mark all read and Clear all.
void Runtime::Impl::emit_notify_window() {
  const std::string open = resolve(std::string("tree.") + kNotifyWindowId + ".open");
  if (read_bool_or_seed(app, open, 0) == 0)
    return;
  const cvc::notify_summary s = cvc::notify_status(app);
  if (s.rev != notify_window_rev || notify_window.kind != Kind::Window) {
    std::vector<Widget> kids;
    Widget row = group({button("Mark all read", kNotifyEvent + std::string("read_all")),
                        button("Clear all", kNotifyEvent + std::string("clear_all"))});
    row.layout.kind = LayoutKind::Horizontal;
    kids.push_back(std::move(row));
    kids.push_back(separator());
    const std::vector<cvc::notification_entry> all = cvc::notifications(app);
    if (all.empty())
      kids.push_back(text("No notifications."));
    for (auto it = all.rbegin(); it != all.rend(); ++it) {
      kids.push_back(
          text(notify_line(*it, it->source.empty() ? notify_source(it->path) : it->source,
                           backend->unicode_glyphs(), "%d %b %H:%M:%S")));
      kids.back().copy = it->text; // §5.28 a right-click copies the whole message
    }
    Widget w = window("Notifications", std::move(kids));
    w.id = kNotifyWindowId;
    w.closable = true;
    notify_window = std::move(w);
    notify_window_rev = s.rev;
  }
  emit(notify_window);
}

// §5.24 the bell's and the Notifications window's own events.
bool Runtime::Impl::builtin_event(const QueuedAction &a) {
  const std::string_view app_tag = kAppEvent; // §5.25 an application's Quit, Restart, reopen
  if (a.event.compare(0, app_tag.size(), app_tag) == 0) {
    const std::string what = a.event.substr(app_tag.size());
    const std::size_t colon = what.find(':');
    const std::string verb = what.substr(0, colon);
    const std::string scope = colon == std::string::npos ? std::string() : what.substr(colon + 1);
    bool known = false;
    for (const WindowsMenuPlan::App &app_ : frame_apps)
      known = known || app_.scope == scope;
    if (!known)
      return true;
    if (verb == "quit") {
      quit_app(scope);
    } else if (verb == "restart") {
      restart_app(scope);
      if (const auto s = mount_slots_.find(scope); s != mount_slots_.end())
        s->second.held_since = -1.0; // its stretch at the limit starts again
    } else if (verb == "open") {
      quit_apps.erase(scope);
    } else if (verb == "close") { // ARIADNE_SHELL.md 4.2: a mounts: application's entry goes
      if (const auto s = mount_slots_.find(scope); s != mount_slots_.end())
        try {
          if (cvc::state *slot = cvc::state::instance(app).findDescendant(s->second.slot_key))
            slot->removeChild(s->second.as);
        } catch (const std::exception &) {
        }
    } else if (verb == "allow") { // 4.11 Allow more: twice its budget, kept in its entry
      if (const auto s = mount_slots_.find(scope); s != mount_slots_.end()) {
        MountSlot &m = s->second;
        const std::string at = m.slot_key + cvc::state::SEPARATOR + m.as + cvc::state::SEPARATOR +
                               "budget" + cvc::state::SEPARATOR;
        if (m.budget.steps > 0)
          write<double>(app, at + "steps", static_cast<double>(m.budget.steps) * 2.0);
        if (m.budget.time > 0.0)
          write<double>(app, at + "time", m.budget.time * 2.0);
        if (m.budget.memory > 0)
          write<double>(app, at + "memory", static_cast<double>(m.budget.memory) * 2.0);
        apply_mount_budget(m);
        m.held_since = -1.0;
        m.noticed = false;
      }
    } else if (verb == "budget") { // the notice's click: the window with its three choices
      write<int>(app, resolve_bind(prefix, std::string("tree.") + kBudgetWindowId + ".open"), 1);
      write<std::string>(
          app, resolve_bind(prefix, std::string("tree.") + kBudgetWindowId + ".raise"), "1");
    }
    mirror_apps(); // its running at once, not at the next frame
    return true;
  }
  const std::string_view tag = kNotifyEvent;
  if (a.event.compare(0, tag.size(), tag) != 0)
    return false;
  const std::string what = a.event.substr(tag.size());
  if (what == "read_all") {
    cvc::notify_read_all(app);
  } else if (what == "clear_all") {
    cvc::notify_clear_all(app);
  } else if (what == "show") {
    write<int>(app, resolve(std::string("tree.") + kNotifyWindowId + ".open"), 1);
    write<std::string>(app, resolve(std::string("tree.") + kNotifyWindowId + ".raise"), "1");
  } else if (what.rfind("open:", 0) == 0) {
    std::uint64_t id = 0;
    try {
      id = std::stoull(what.substr(5));
    } catch (const std::exception &) {
      return true;
    }
    for (const cvc::notification_entry &e : cvc::notifications(app))
      if (e.id == id) {
        cvc::notify_read(app, id);
        if (!e.on.empty()) { // its program, in the scope that posted it, from the next drain
          const std::string scope = e.scope.empty() ? prefix : e.scope;
          queued_events.push_back({e.on, scope});
          const auto fn = mount_functions.find(scope);
          queued_events.back().functions = fn != mount_functions.end() ? fn->second : doc_functions;
        }
        break;
      }
  }
  return true;
}

// §5.24 the feed's verbs, for a program run in `scope`:
//   (notify TEXT [LEVEL [ON]])  post; LEVEL info (the default), warn or error; ON a program run
//                               in this scope when the entry is clicked. Returns its id.
//   (notify-read ID) (notify-clear ID)   mark one read / drop it; #t when there was one
//   (notify-read-all)                    mark every entry read
//   (notify-clear-all)                   drop the entries this scope posted (its own and its
//                                        mounts'); how many
//   (notify-status)                      {count unread worst rev}
//   (notify-list [N])                    the newest N entries (all), newest first, each
//                                        {id text level source time read repeat}
void Runtime::Impl::bind_notify_verbs(const std::shared_ptr<cvc::state_exec::environment> &env,
                                      const NotifyScope &ns) const {
  namespace se = cvc::state_exec;
  cvc::app *const on_app = &app;
  const std::string &scope = ns.scope;
  const std::string &source = ns.source;
  const auto text_arg = [](const se::value_t &v, const char *verb, const char *what) {
    const auto *s = std::get_if<std::string>(&v.v);
    if (!s)
      throw se::dsl_error("TypeError",
                          std::string(verb) + ": " + what + " is a string, not " + v.type_name());
    return *s;
  };
  const auto id_arg = [](std::span<const se::value_t> args, const char *verb) -> std::uint64_t {
    if (args.size() != 1)
      throw se::dsl_error("TypeError", std::string(verb) + ": expected one argument, an id");
    if (const auto *i = std::get_if<int64_t>(&args[0].v))
      return *i > 0 ? static_cast<std::uint64_t>(*i) : 0;
    if (const auto *d = std::get_if<double>(&args[0].v))
      return *d > 0 ? static_cast<std::uint64_t>(*d) : 0;
    throw se::dsl_error("TypeError", std::string(verb) + ": the id is a number");
  };
  se::builtins::register_fn(
      env, "notify",
      [on_app, scope, source, text_arg](std::span<const se::value_t> args) -> se::value_t {
        if (args.empty() || args.size() > 3)
          throw se::dsl_error("TypeError", "notify: (notify TEXT [LEVEL [ON]])");
        cvc::notification n;
        n.text = text_arg(args[0], "notify", "TEXT");
        if (args.size() > 1 && !args[1].is_nil())
          n.level = cvc::notify_level(text_arg(args[1], "notify", "LEVEL"));
        if (args.size() > 2 && !args[2].is_nil())
          n.on = text_arg(args[2], "notify", "ON");
        n.source = source;
        n.path = scope;
        n.scope = scope;
        return se::value_t(static_cast<int64_t>(cvc::notify(*on_app, std::move(n))));
      });
  se::builtins::register_fn(
      env, "notify-read", [on_app, id_arg](std::span<const se::value_t> args) -> se::value_t {
        return se::value_t(cvc::notify_read(*on_app, id_arg(args, "notify-read")));
      });
  se::builtins::register_fn(
      env, "notify-clear", [on_app, id_arg](std::span<const se::value_t> args) -> se::value_t {
        return se::value_t(cvc::notify_clear(*on_app, id_arg(args, "notify-clear")));
      });
  se::builtins::register_fn(
      env, "notify-read-all", [on_app](std::span<const se::value_t> args) -> se::value_t {
        if (!args.empty())
          throw se::dsl_error("TypeError", "notify-read-all: expected no arguments");
        cvc::notify_read_all(*on_app);
        return se::value_t(true);
      });
  se::builtins::register_fn(
      env, "notify-status", [on_app](std::span<const se::value_t> args) -> se::value_t {
        if (!args.empty())
          throw se::dsl_error("TypeError", "notify-status: expected no arguments");
        const cvc::notify_summary s = cvc::notify_status(*on_app);
        return se::make_dict({
            {"count", se::value_t(static_cast<int64_t>(s.count))},
            {"unread", se::value_t(static_cast<int64_t>(s.unread))},
            {"worst", se::value_t(std::string(s.worst < 0 ? "" : cvc::notify_level_name(s.worst)))},
            {"rev", se::value_t(static_cast<int64_t>(s.rev))},
        });
      });
  // The mounts as they are now: a child program may outlive this Runtime, so it keeps a copy.
  se::builtins::register_fn(
      env, "notify-list",
      [on_app, mounts = ns.mounts](std::span<const se::value_t> args) -> se::value_t {
        if (args.size() > 1)
          throw se::dsl_error("TypeError", "notify-list: expected at most one argument, a count");
        std::size_t n = cvc::notify_capacity;
        if (!args.empty()) {
          if (const auto *i = std::get_if<int64_t>(&args[0].v))
            n = *i > 0 ? static_cast<std::size_t>(*i) : 0;
          else
            throw se::dsl_error("TypeError", "notify-list: the count is an int");
        }
        const std::vector<cvc::notification_entry> all = cvc::notifications(*on_app);
        std::vector<se::value_t> out;
        for (auto it = all.rbegin(); it != all.rend() && out.size() < n; ++it)
          out.push_back(se::make_dict({
              {"id", se::value_t(static_cast<int64_t>(it->id))},
              {"text", se::value_t(it->text)},
              {"level", se::value_t(std::string(cvc::notify_level_name(it->level)))},
              {"source",
               se::value_t(it->source.empty() ? mount_source(mounts, it->path) : it->source)},
              {"time", se::value_t(it->time)},
              {"read", se::value_t(it->read)},
              {"repeat", se::value_t(static_cast<int64_t>(it->repeat))},
          }));
        return se::make_list(std::move(out));
      });
  se::builtins::register_fn(
      env, "notify-clear-all", [on_app, scope](std::span<const se::value_t> args) -> se::value_t {
        if (!args.empty())
          throw se::dsl_error("TypeError", "notify-clear-all: expected no arguments");
        return se::value_t(static_cast<int64_t>(cvc::notify_clear_under(*on_app, scope)));
      });
}

// The titles the bar's menus show this frame, in order: its entries (a repeated one once), then the
// Windows menu when it goes last (§5.9).
std::vector<std::string> Runtime::Impl::bar_titles(const Widget &bar) {
  std::vector<std::string> out;
  for (const Widget &c : bar.children) {
    if (c.kind == Kind::TrayIcon || c.kind == Kind::Separator || !visible(c))
      continue;
    if (c.kind == Kind::WindowsMenu) {
      if (c.bdef && windows_menu.on && windows_menu_has_items())
        out.push_back(c.label.empty() ? windows_menu.title : c.label);
      continue;
    }
    out.push_back(c.label);
  }
  if (!apps_drawn) // §5.25 the loaded documents' menus, after the document's
    for (const WindowsMenuPlan::App &a : frame_apps)
      if (app_running(a.scope))
        out.push_back(a.name);
  if (!windows_menu.placed && !windows_menu.merged && windows_menu.on && windows_menu_has_items())
    out.push_back(windows_menu.title);
  return out;
}

// §5.23 whether the bar's menus fold into one: always or never when the document says so, else
// when the backend finds they do not fit beside the whole tray.
bool Runtime::Impl::collapse_menus(const Widget &bar, const std::vector<TrayEntry> &tray) {
  if (bar.collapse == "never")
    return false;
  if (bar.collapse == "always")
    return true;
  const std::vector<std::string> titles = bar_titles(bar);
  if (titles.empty())
    return false;
  std::vector<Backend::TrayIcon> specs;
  for (const TrayEntry &t : tray)
    specs.push_back(t.spec);
  return !backend->menus_fit(titles, specs);
}

// The bar's tray this frame: its TrayIcon children and the clock (§5.22), in the order the bar
// shows them. The clock goes where a `- clock` entry places it, else last, at the right end.
std::vector<Runtime::Impl::TrayEntry> Runtime::Impl::tray_entries(const Widget &bar) {
  using Icon = TrayEntry;
  const bool clock_on =
      !first_bar_drawn && read_bool_or_seed(app, resolve(kClockKey), clock_on_by_default()) != 0;
  const auto clock_icon = [&](const Widget *at) {
    const auto now = wall_clock ? wall_clock() : std::chrono::system_clock::now();
    Icon c;
    c.w = at;
    c.clock = true;
    c.spec.glyph = local_time(now, "%H:%M");
    c.spec.text = true;
    c.tip = local_time(now, "%A %d %B %Y, %H:%M:%S");
    // Joined to a relay: its time too, and how far its clock is from this one's.
    if (const cvc::server_clock_reading sc = cvc::server_clock(app); sc.known) {
      const auto there = now + std::chrono::duration_cast<std::chrono::system_clock::duration>(
                                   std::chrono::duration<double, std::milli>(sc.offset_ms));
      char off[48];
      std::snprintf(off, sizeof off, "%+.2f s, round trip %.0f ms", sc.offset_ms / 1000.0,
                    sc.rtt_ms < 0.0 ? 0.0 : sc.rtt_ms);
      c.tip += "\n" + sc.name + ": " + local_time(there, "%H:%M:%S") + " (" + off + ")";
    }
    c.spec.tooltip = c.tip;
    return c;
  };
  // §5.24 the bell: while the feed has entries (always, for `notifications: always`); its badge the
  // unread, its tone the worst of them.
  const cvc::notify_summary feed = cvc::notify_status(app);
  // The clock and the bell are the document's first bar's (a second bar of its own has neither).
  const bool bell_on = !first_bar_drawn && (feed.count > 0 || windows_menu.bell_always) &&
                       read_bool_or_seed(app, resolve(kBellKey), bell_on_by_default()) != 0;
  const auto bell_icon = [&](const Widget *at) {
    Icon c;
    c.w = at;
    c.bell = true;
    c.spec.icon = "bell";
    c.spec.glyph = std::string(icon_glyph("bell", backend->unicode_glyphs()));
    c.spec.menu = true;
    if (feed.unread > 0) {
      c.spec.badge = std::to_string(feed.unread);
      c.spec.tone = std::max(feed.worst, 0);
      c.tip = std::to_string(feed.unread) +
              (feed.unread == 1 ? " unread notification" : " unread notifications");
    } else {
      c.tip = "Notifications";
    }
    c.spec.tooltip = c.tip;
    return c;
  };
  std::vector<Icon> icons;
  for (const Widget &c : bar.children) {
    if (c.kind != Kind::TrayIcon || !c.repeat.empty() || !visible(c))
      continue;
    if (c.tray_bell) {
      if (bell_on)
        icons.push_back(bell_icon(&c));
      continue;
    }
    if (c.tray_clock) {
      if (clock_on)
        icons.push_back(clock_icon(&c));
      continue;
    }
    Icon i;
    i.w = &c;
    i.spec = tray_spec(c);
    i.tip = is_expr(c.tooltip) ? eval_text_or_literal(c.tooltip) : c.tooltip;
    i.spec.tooltip = i.tip;
    icons.push_back(std::move(i));
  }
  // §5.25 the running loaded documents' icons, after the document's own (in the first bar only),
  // each worked out in its mount's scope.
  if (!apps_drawn)
    for (const WindowsMenuPlan::App &a : frame_apps) {
      if (!app_running(a.scope) || !a.bar)
        continue;
      // ARIADNE_SHELL.md 4.5 only one that holds extend.tray (granted by default; deny: takes it).
      if (!grant_holds(a.scope, "extend.tray"))
        continue;
      const ScopedTo in(*this, a.scope, a.functions);
      for (const Widget &c : a.bar->children) {
        if (c.kind != Kind::TrayIcon || c.tray_clock || c.tray_bell || !c.repeat.empty() ||
            !visible(c))
          continue;
        Icon i;
        i.w = &c;
        i.scope = a.scope;
        i.functions = a.functions;
        i.spec = tray_spec(c);
        i.tip = is_expr(c.tooltip) ? eval_text_or_literal(c.tooltip) : c.tooltip;
        i.spec.tooltip = i.tip;
        icons.push_back(std::move(i));
      }
    }
  if (bell_on && !windows_menu.bell_placed)
    icons.push_back(bell_icon(nullptr));
  if (clock_on && !windows_menu.clock_placed)
    icons.push_back(clock_icon(nullptr));
  return icons;
}

// The bar's tray, right-aligned after the menus. The icons the backend has no room for go, in
// order, into an overflow menu at the tray's left end, whose badge counts what their badges count
// and takes the strongest of their tones. A backend without a tray shows each as an ordinary menu
// at the end of the bar.
// On the walk: each icon's menu items (emit_tray_items: a bar's children) are walked, and what
// follows them is a Then frame, in the order the backend saw it when this recursed.
void Runtime::Impl::emit_tray(std::vector<TrayEntry> icons) {
  if (icons.empty())
    return;
  Backend &b = *backend;
  const auto t = std::make_shared<TrayWalk>();
  t->icons = std::move(icons);
  for (const TrayEntry &i : t->icons)
    t->specs.push_back(i.spec);
  const int n = static_cast<int>(t->icons.size());
  const int fit = b.begin_tray(t->specs);
  if (fit < 0) { // no tray: a menu per icon, at the end of the bar
    tray_menus(t, 0);
    return;
  }
  t->hidden = n - std::min(fit, n);
  push_then([this, t] { tray_shown(t, t->hidden); }); // then the shown icons and end_tray
  if (t->hidden > 0) {
    Backend::TrayIcon more;
    more.glyph = std::string(icon_glyph("more", b.unicode_glyphs()));
    more.menu = true;
    long count = 0;
    for (int k = 0; k < t->hidden; ++k) {
      more.tone = std::max(more.tone, t->specs[k].tone);
      try {
        count += t->specs[k].badge.empty() ? 0 : std::stol(t->specs[k].badge);
      } catch (const std::exception &) {
        ++count; // a badge that is not a number counts once
      }
    }
    if (count > 0)
      more.badge = std::to_string(count);
    b.push_id("##tray.more");
    if (b.begin_tray_menu(more)) {
      push_then([this] {
        backend->end_tray_menu();
        backend->pop_id();
      });
      tray_more(t, 0);
    } else {
      b.pop_id();
    }
  }
}

// No tray: icon k as a menu at the end of the bar (its own items, or itself as an item), then the
// icons after it.
void Runtime::Impl::tray_menus(const std::shared_ptr<TrayWalk> &t, int k) {
  if (k == static_cast<int>(t->icons.size()))
    return;
  push_then([this, t, k] { tray_menus(t, k + 1); });
  Backend &b = *backend;
  const TrayEntry &i = t->icons[k];
  const PushedScope in = push_scope(i.scope, i.functions);
  b.push_id(tray_id(i, k).c_str());
  std::string title = i.spec.glyph;
  if (!i.spec.badge.empty())
    title += " " + i.spec.badge;
  if (b.begin_menu(title.c_str())) {
    push_then([this, in] {
      backend->end_menu();
      backend->pop_id();
      pop_scope(in);
    });
    if (i.spec.menu)
      emit_tray_items(i);
    else
      tray_item(t, k);
    return;
  }
  b.pop_id();
  pop_scope(in);
}

// The overflow menu's items: icon k as an item, then the hidden icons after it.
void Runtime::Impl::tray_more(const std::shared_ptr<TrayWalk> &t, int k) {
  if (k == t->hidden)
    return;
  push_then([this, t, k] { tray_more(t, k + 1); });
  backend->push_id(std::to_string(k).c_str());
  push_then([this] { backend->pop_id(); });
  tray_item(t, k);
}

// The shown icons from k, each a tray menu or a button, then end_tray.
void Runtime::Impl::tray_shown(const std::shared_ptr<TrayWalk> &t, int k) {
  Backend &b = *backend;
  if (k == static_cast<int>(t->icons.size())) {
    b.end_tray();
    return;
  }
  push_then([this, t, k] { tray_shown(t, k + 1); });
  const TrayEntry &i = t->icons[k];
  const PushedScope in = push_scope(i.scope, i.functions);
  b.push_id(tray_id(i, k).c_str());
  if (i.spec.menu) {
    if (b.begin_tray_menu(i.spec)) {
      push_then([this, in] {
        backend->end_tray_menu();
        backend->pop_id();
        pop_scope(in);
      });
      emit_tray_items(i);
      return;
    }
    if (!i.tip.empty())
      b.set_tooltip(i.tip.c_str());
  } else {
    if (b.tray_button(i.spec) && !i.clock && !i.w->on.empty())
      enqueue(i.w->on);
    if (!i.tip.empty())
      b.set_tooltip(i.tip.c_str());
  }
  b.pop_id();
  pop_scope(in);
}

// Icon k as an item of a menu (the overflow menu, or a backend without a tray): its glyph and
// badge, then its tooltip; its own menu as a submenu.
void Runtime::Impl::tray_item(const std::shared_ptr<TrayWalk> &t, int k) {
  Backend &b = *backend;
  const TrayEntry &i = t->icons[k];
  const PushedScope in = push_scope(i.scope, i.functions);
  std::string title = i.spec.glyph;
  if (!i.spec.badge.empty())
    title += " " + i.spec.badge;
  if (!i.tip.empty())
    title += "  " + i.tip;
  if (i.spec.menu) {
    if (b.begin_menu(title.c_str())) {
      push_then([this, in] {
        backend->end_menu();
        pop_scope(in);
      });
      emit_tray_items(i);
      return;
    }
  } else if (b.menu_item_action(title.c_str()) && !i.clock && !i.w->on.empty()) {
    enqueue(i.w->on);
  }
  pop_scope(in);
}

std::string Runtime::Impl::tray_id(const TrayEntry &i, int k) {
  if (i.clock || i.bell)
    return std::string(i.clock ? "##tray.clock" : "##tray.bell");
  const std::string own = i.w->id.empty() ? std::to_string(k) : i.w->id;
  return i.scope.empty() ? own : i.scope + "/" + own; // a loaded document's: its own ids
}

// --------------------------------------------------------------------------

Runtime::Impl::~Impl() {
  // Host thread, outside a pump: the kills below would race a worker on the group, and its
  // destruction abort. Refused in every build, before anything is touched.
  refuse_if_held(app, group_.get(), "tearing down the document at '" + prefix + "'");
  // Teardown (S1.10, S1-Q8), each step catching and logging what it throws. First close anything
  // the document left open (a (stream-open) with no (stream-close)), while the app, its state tree
  // and the document's scheduler are still alive: each owned resource's destructor runs here
  // (~stream marks its descriptor closed, releases its state pins and posts to the scheduler its
  // context named) — the non-singleton teardown a process-static owning table could not give (it
  // would run at process exit against a dead app and hang).
  try {
    doc_scope_.clear();
  } catch (const std::exception &e) {
    teardown_note("tearing down", "closing its resources", e.what());
  }
  // Then reap the document's whole process group, so a parked (await/sleep/msg-recv) action never
  // lingers after its Runtime is gone (kill_and_reap). Group 0 is touched only if this document
  // used it (exec_scheduler() builds it lazily).
  cvc::state_exec::async_scheduler *sched = nullptr;
  try {
    sched = group_ ? &group_->sched() : used_scheduler_ ? &app.exec_scheduler() : nullptr;
  } catch (const std::exception &e) {
    teardown_note("tearing down", "finding its scheduler", e.what());
  }
  if (sched)
    kill_and_reap(*sched, "tearing down");
  // Last the group, which the app's frames stop pumping first: its own teardown (S1.5) kills and
  // reaps whatever is left, before channel_policy_ and doc_scope_ go, which its processes'
  // contexts point at; then any retired group, waiting for its kernels as a teardown does. Both
  // catch and log what they throw.
  frame_handle_.reset();
  group_.reset();
  retired_groups_.clear();
}

void Runtime::Impl::kill_and_reap(cvc::state_exec::async_scheduler &sched,
                                  const char *when) noexcept {
  // kill_owner kills every process of the owner, the children its actions and residents spawned
  // and forked included, and each is reaped, the actions' own processes too (S1-Q8: a killed one
  // left in the table is a record whose context is gone).
  try {
    sched.kill_owner(owner_);
  } catch (const std::exception &e) {
    teardown_note(when, "killing its processes", e.what());
  }
  std::vector<int> pids;
  try {
    pids = own_pids();
  } catch (const std::exception &e) {
    teardown_note(when, "listing its processes", e.what());
  }
  for (const int pid : pids) {
    try {
      sched.reap(pid);
    } catch (const std::exception &e) {
      teardown_note(when, "reaping a process", e.what());
    }
  }
}

void Runtime::Impl::teardown_note(const char *when, const char *step, const char *what) noexcept {
  try {
    app.log(1, std::string("ari: ") + when + " the document at '" + prefix + "': " + step + ": " +
                   what);
  } catch (...) {
  }
}

std::vector<int> Runtime::Impl::own_pids() const {
  std::vector<int> pids;
  for (const auto &[pid, ac] : live_actions_) {
    pids.push_back(pid);
    pids.insert(pids.end(), ac->children.begin(), ac->children.end());
  }
  pids.insert(pids.end(), spawned_children_.begin(), spawned_children_.end());
  for (const Resident *r : {&tick_resident_, &key_resident_, &pointer_resident_, &signal_resident_})
    if (r->pid >= 0)
      pids.push_back(r->pid);
  for (const auto &[sub_prefix, t] : mount_ticks_)
    if (t.resident.pid >= 0)
      pids.push_back(t.resident.pid);
  return pids;
}

cvc::state_exec::async_scheduler &Runtime::Impl::current_sched() {
  return group_ ? group_->sched() : app.exec_scheduler();
}

bool Runtime::Impl::nothing_running() const {
  if (!live_actions_.empty() || !spawned_children_.empty())
    return false;
  for (const Resident *r : {&tick_resident_, &key_resident_, &pointer_resident_, &signal_resident_})
    if (r->pid >= 0)
      return false;
  for (const auto &[sub_prefix, t] : mount_ticks_)
    if (t.resident.pid >= 0)
      return false;
  return true;
}

void Runtime::Impl::ensure_placement(bool submitting) {
  namespace se = cvc::state_exec;
  if (!submitting && !document_set_)
    return;
  if (placement_decided_) {
    // A registration since (a late host verb or provider) may change the answer: decided again
    // only when a move would kill nothing, else at the document's next restart (m3).
    if (!registered_since_placement_ && action_intrinsics_generation(app) == placement_generation_)
      return;
    if (!nothing_running()) {
      if (!late_noted_ && group_ && group_->opts().affinity == se::group_affinity::any) {
        late_noted_ = true;
        app.log(1, "ari: the document at '" + prefix +
                       "' started its programs before a host function or provider was registered: "
                       "it binds once nothing of the document runs, or at its next load (until "
                       "then a host_only one is a NameError in its isolated group)");
      }
      return;
    }
  }
  placement_decided_ = true;
  placement_generation_ = action_intrinsics_generation(app);
  registered_since_placement_ = false;
  late_noted_ = false;
  bool host_affinity = false;
  std::string reason;
  try {
    reason = isolation_verdict(host_affinity);
  } catch (const std::exception &e) { // a provider that throws in the probe, say
    reason = std::string("its eligibility could not be read: ") + e.what();
  }
  const se::group_affinity affinity =
      host_affinity ? se::group_affinity::host : se::group_affinity::any;
  const bool stays = reason.empty() ? group_ && group_->opts().affinity == affinity : !group_;
  if (!stays) {
    // Where its programs and streams were (null: nothing ran yet).
    se::async_scheduler *const from = group_            ? &group_->sched()
                                      : used_scheduler_ ? &app.exec_scheduler()
                                                        : nullptr;
    // A move kills and reaps on the group a worker may be pumping, and takes the group away: host
    // thread, outside a pump. Refused in every build, before the kills, while a pump holds it.
    refuse_if_held(app, group_.get(), "moving the document at '" + prefix + "'");
    // What an earlier document runs, killed and reaped where it runs.
    std::vector<std::shared_ptr<ActionContext>> taken = move_out();
    std::unique_ptr<se::exec_group> old_group = std::move(group_);
    frame_handle_.reset(); // the app's frames stop pumping the old group now
    if (!old_group)        // group 0's contexts stay with the Runtime (m5)
      retired_contexts_.insert(retired_contexts_.end(), std::make_move_iterator(taken.begin()),
                               std::make_move_iterator(taken.end()));
    if (reason.empty()) {
      try {
        se::exec_group::options o;
        o.name = owner_;
        o.owner = owner_;
        o.affinity = affinity;
        o.app = &app;
        // The home of the keys under its prefix, its channels' among them (S2b, S2.md section 0
        // row 23): the constructor registers it with the app's router, so a message sent on one
        // from another group comes here.
        o.channel_prefixes = {prefix};
        group_ = std::make_unique<se::exec_group>(std::move(o));
        frame_handle_ = app.register_frame_group(*group_);
      } catch (const std::exception &e) {
        frame_handle_.reset();
        group_.reset();
        reason = std::string("its group could not be made: ") + e.what();
      }
    }
    // The streams it owns post where its programs run from now on (M1: by owner, whatever their
    // scope; no other owner's). If they cannot be re-pointed they are closed: none may be left
    // naming the old place.
    if (from) {
      try {
        retarget_document_streams(&doc_scope_, from,
                                  group_ ? &group_->sched() : &app.exec_scheduler());
      } catch (const std::exception &e) {
        teardown_note("moving", "re-pointing its streams (they are closed)", e.what());
        try {
          close_document_streams(&doc_scope_);
        } catch (const std::exception &e2) {
          teardown_note("moving", "closing its streams (its resources are dropped)", e2.what());
          doc_scope_.clear();
        }
      }
    }
    // The old group is retired, with the contexts taken from it, until its kernels end (m1). It
    // leaves the router now, after the new group registered the prefix (S2b): nothing pumps it,
    // so the messages and waits it is the home of go to the new home.
    if (old_group) {
      retire_from_router(*old_group);
      retired_groups_.push_back(RetiredGroup{std::move(old_group), std::move(taken)});
    }
  }
  // Why a document stays in group 0 is logged when the reason changes (a new document's, or a
  // first), and only when the setting asks for isolation; one reason is logged once. Not before a
  // document is set: a process a first load starts (a guest's ari_init, an init: call) is placed
  // before its channels are known, and the load decides again once they are (set_root).
  if (document_set_) {
    if (!reason.empty() && reason != logged_reason_ &&
        isolate_documents_on(cvc::state::instance(app)))
      app.log(1, "ari: the document at '" + prefix + "' runs in group 0: " + reason);
    logged_reason_ = reason;
  }
  isolation_reason_ = reason;
  if (!stays) {
    std::lock_guard<std::mutex> lock(names_mutex);
    names_valid = false; // has? and requires: see the providers as the group installs them
  }
  snapshot_has();
}

std::vector<std::shared_ptr<Runtime::Impl::ActionContext>> Runtime::Impl::move_out() {
  std::vector<std::shared_ptr<ActionContext>> taken;
  cvc::state_exec::async_scheduler *sched = group_            ? &group_->sched()
                                            : used_scheduler_ ? &app.exec_scheduler()
                                                              : nullptr;
  if (!sched)
    return taken; // nothing ran yet
  kill_and_reap(*sched, "moving");
  // Its records go; their contexts go with the place they ran in (the caller keeps them), for a
  // process a failed kill left there.
  for (auto &[pid, ac] : live_actions_) {
    if (!ac->ended && ac->done) // run_program: told how it ended
      ac->done("did not complete: the document moved to another group");
    taken.push_back(ac);
  }
  live_actions_.clear();
  spawned_children_.clear();
  for (Resident *r : {&tick_resident_, &key_resident_, &pointer_resident_, &signal_resident_}) {
    if (r->ctx)
      taken.push_back(r->ctx);
    *r = Resident{}; // started again where the document goes, at the next drain
  }
  for (auto &[sub_prefix, t] : mount_ticks_) {
    if (t.resident.ctx)
      taken.push_back(t.resident.ctx);
    t.resident = Resident{};
  }
  used_scheduler_ = false;
  return taken;
}

void Runtime::Impl::destroy_idle_retired_groups() {
  std::erase_if(retired_groups_, [](const RetiredGroup &r) {
    return !r.group || r.group->host().in_flight() == 0; // its kernels have ended: it goes now
  });
}

void Runtime::Impl::retire_from_router(cvc::state_exec::exec_group &g) noexcept {
  // As its teardown leaves (exec_router.h, "Teardown and rehoming"), holding its token, which is
  // free: no pump_all holds the group (refuse_if_held) and the frames pump it no more. The
  // teardown leaves again later and finds nothing to send. If the token is held elsewhere (a host
  // attached the group and a worker pumps it), only its prefix goes now, and what it holds goes at
  // its teardown.
  try {
    cvc::state_exec::async_scheduler::pump_owner_scope token(g.sched());
    g.sched().leave_router();
    return;
  } catch (const std::exception &e) {
    teardown_note("moving", "leaving the router", e.what());
  } catch (...) {
    teardown_note("moving", "leaving the router", "an unknown exception");
  }
  try {
    app.exec_router().unregister(g.route());
  } catch (...) {
  }
}

std::string Runtime::Impl::isolation_verdict(bool &host_affinity) const {
  namespace se = cvc::state_exec;
  host_affinity = false;
  if (!isolate_documents_on(cvc::state::instance(app)))
    return "ariadne.exec.isolate_documents is off";
  // S1-Q6's rules, in order, but its channel rules (S2-Q16): its channels no longer keep it in
  // group 0, since one outside its prefix (a `/` channel, global: true or not) is homed at group 0
  // or another document's group and routed (S2b). A '#' key is never routed, so oidc still does.
  if (prefix.empty())
    return "it has no prefix (it runs at the app root)";
  // The programs it runs as written: its lanes, its key bindings, its functions and every program
  // in its tree (a mount's init:, on_tick: and functions too).
  const Widget &tree = has_pending ? pending : root;
  std::vector<const std::string *> programs = {&tick_script_, &key_script_, &pointer_script_,
                                               &signal_script_, &doc_functions};
  for (const KeyBinding &b : key_bindings_)
    programs.push_back(&b.on);
  for (const std::string &p : document_programs_) // its init: and tests:, run elsewhere
    programs.push_back(&p);
  for (std::vector<const Widget *> todo{&tree}; !todo.empty();) { // tree order, no recursion
    const Widget &w = *todo.back();
    todo.pop_back();
    for (const std::string *p : {&w.on, &w.on_click, &w.on_hover, &w.on_drag, &w.on_drag_start,
                                 &w.on_drag_end, &w.on_top, &w.init_script, &w.tick_script})
      programs.push_back(p);
    if (w.functions)
      programs.push_back(&w.functions->prelude);
    for (auto c = w.children.rbegin(); c != w.children.rend(); ++c)
      todo.push_back(&*c);
  }
  std::string oidc_use;
  for (const std::string *p : programs) {
    if (!oidc_use.empty())
      break;
    if (!is_expr(*p))
      continue;
    std::vector<std::string> channels;
    try {
      static_channels(se::parse(*p), channels);
    } catch (const std::exception &) {
      continue; // its parse error shows when it runs
    }
    for (const std::string &c : channels)
      // oidc's completions go to "oidc#NAME" on group 0 (oidc_host.cpp)
      if (oidc_use.empty() && c.rfind("oidc#", 0) == 0)
        oidc_use = "it waits on oidc's channel '" + c + "'";
    if (oidc_use.empty())
      for (const std::string &name : free_heads(*p))
        if (name.rfind("oidc-", 0) == 0) {
          oidc_use = "it uses oidc (" + name + ")";
          break;
        }
  }
  if (!oidc_use.empty())
    return oidc_use;
  // A writable link: hole whose target leaves the prefix would write outside it from the group's
  // thread. A hole's target resolves against its mount's parent scope ('/' is the app root).
  const std::string inside = prefix + cvc::state::SEPARATOR;
  std::string hole;
  std::vector<std::pair<const Widget *, std::string>> todo{{&tree, prefix}}; // node, its scope
  while (!todo.empty()) {
    const auto [w, scope] = std::move(todo.back());
    todo.pop_back();
    std::string inner = scope;
    if (!w->scope.empty()) {
      inner = resolve_bind(scope, w->scope);
      for (const LinkHole &h : w->links) {
        const std::string target = resolve_bind(scope, h.target);
        if (hole.empty() && h.writable && target != prefix && target.rfind(inside, 0) != 0)
          hole = "the mount at '" + inner + "' grants '" + h.name + "' writable to '" + h.target +
                 "', outside its prefix";
      }
    }
    for (auto c = w->children.rbegin(); c != w->children.rend(); ++c)
      todo.emplace_back(&*c, inner);
  }
  if (!hole.empty())
    return hole;
  // Eligible. Its group is pumped on the host thread when its host declared a thread-affine
  // subscriber at or under its prefix (S1.11), when its programs run host code inline
  // (require_host_thread: a scripting engine's activations), or when a host_only provider binds
  // anything in its env, which an `any` group would leave out.
  for (const std::string &p : thread_affine_)
    if (p == prefix || p.rfind(inside, 0) == 0 || p.empty() ||
        prefix.rfind(p + cvc::state::SEPARATOR, 0) == 0) // at, under or above: a write under the
      host_affinity = true; // document emits childChanged on every ancestor (S-M2)
  if (!host_thread_why_.empty())
    host_affinity = true;
  if (!host_affinity)
    host_affinity = host_only_bindings();
  return std::string();
}

void Runtime::Impl::probe_env(
    const std::function<void(const cvc::state_exec::environment_ptr &,
                             cvc::state_exec::intrinsics_context &)> &fill) const {
  namespace se = cvc::state_exec;
  // The env an action runs in (submit_action), never executed: only its bindings are read. The
  // context mirrors a real action's, so a provider that reads it while binding sees what it would.
  se::scheduler sched;
  se::async_host probe_host(&sched); // inline, never called: a provider registers on *ictx.host
  se::memory_tracker tracker;
  auto proc = se::make_process();
  proc->pid = 1;
  proc->status = se::process_status::ready;
  se::intrinsics_context ictx;
  ictx.sched = &sched;
  ictx.host = &probe_host;
  ictx.tracker = &tracker;
  ictx.proc = proc;
  ictx.pid = 1;
  se::apply_chroot(ictx, cvc::state::instance(app), prefix);
  ictx.channels = const_cast<se::channel_policy *>(&channel_policy_);
  ictx.document = const_cast<se::document_scope *>(&doc_scope_);
  se::environment_ptr env = se::builtins::make_default_environment();
  se::register_intrinsics(env, &ictx);
  fill(env, ictx);
}

bool Runtime::Impl::host_only_bindings() const {
  namespace se = cvc::state_exec;
  bool bound = false;
  probe_env([&](const se::environment_ptr &env, se::intrinsics_context &ictx) {
    // The app's, then the document's own (add_program_intrinsics), as a program gets them.
    for (const std::vector<registered_provider> &list :
         {action_intrinsics_snapshot(app), own_snapshot()})
      for (const registered_provider &p : list) {
        if (!p.provider)
          continue;
        if (p.affinity != se::provider_affinity::host_only) {
          p.provider(env, ictx);
          continue;
        }
        const se::environment_ptr scratch = se::environment::extend(env);
        p.provider(scratch, ictx);
        bound = bound || !scratch->bindings.empty();
        for (auto &[name, value] : scratch->bindings)
          env->set(name, std::move(value));
      }
  });
  return bound;
}

void Runtime::Impl::snapshot_has() {
  if (!group_ || group_->opts().affinity != cvc::state_exec::group_affinity::any)
    return;
  const HostFacts f = facts();
  std::lock_guard<std::mutex> lock(names_mutex);
  load_names();
  facts_snapshot_ = f;
}

bool Runtime::Impl::has_snapshot(const std::string &name) const {
  if (is_capability_token(name)) {
    std::lock_guard<std::mutex> lock(names_mutex);
    return capability_holds(name, facts_snapshot_);
  }
  if (cvc::state_exec::stackless_evaluator::all_special_forms().count(name))
    return true;
  {
    std::lock_guard<std::mutex> lock(names_mutex);
    if (names.count(name))
      return true;
  }
  return is_builtin_widget_type(name) || has_widget_type(app, name);
}

std::function<bool(const std::string &)>
Runtime::Impl::program_has(cvc::state_exec::exec_group *group,
                           std::shared_ptr<const std::set<std::string>> held) {
  const bool snapshot = group && group->opts().affinity == cvc::state_exec::group_affinity::any;
  return [this, snapshot, held](const std::string &n) {
    if (is_grant_token(n)) // ARIADNE_SHELL.md 4.5: the mount's own, fixed when it was made
      return held ? held->count(n) != 0 : held_alone(n);
    return snapshot ? has_snapshot(n) : has(n);
  };
}

std::shared_ptr<Runtime::Impl::ActionContext>
Runtime::Impl::make_action_context(const std::string &scope_prefix,
                                   std::optional<double> lock_default_timeout) {
  namespace se = cvc::state_exec;
  ensure_placement(/*submitting=*/true);
  se::exec_group *const group = group_.get();
  se::async_scheduler &sched = group ? group->sched() : app.exec_scheduler();
  if (!group)
    used_scheduler_ = true;
  auto ac = std::make_shared<ActionContext>();
  ac->proc = se::make_process(); // placeholder for ictx.proc; the RUNNING process is the
  ac->proc->status = se::process_status::ready; // scheduler's own, reached via current_process()
  ac->ictx.sched = &sched;
  ac->ictx.host = group ? &group->host() : &app.exec_async_host(); // its group's (S1.9, S1.10)
  ac->ictx.tracker = &ac->tracker;
  ac->ictx.proc = ac->proc;
  // Confine writes to the document/mount subtree.
  se::apply_chroot(ac->ictx, cvc::state::instance(app), scope_prefix);
  ac->ictx.channels = &channel_policy_; // §12 channel enforcement (no-op unless strict)
  ac->ictx.document = &doc_scope_;      // per-document owner for (stream-open) & friends
  ac->ictx.lock_default_timeout = lock_default_timeout; // with-lock with no :timeout
  ac->scope = scope_prefix;
  ac->notify = std::make_shared<const NotifyScope>(notify_scope(scope_prefix));
  ac->account = account_for(scope_prefix); // under a mounts: entry: its frame budget
  ac->held = held_at(scope_prefix);        // ... and what its mount holds (ARIADNE_SHELL.md 4.5)
  finish_context(*ac, group);
  return ac;
}

void Runtime::Impl::finish_context(ActionContext &ac, cvc::state_exec::exec_group *group) {
  namespace se = cvc::state_exec;
  ac.env = se::builtins::make_default_environment();
  se::register_intrinsics(ac.env, &ac.ictx);
  // Host program-lane intrinsics (nav verbs, …), AFTER the standard ones (add or override): the
  // app's, then this document's own, each by its kind.
  install_providers(app, ac.env, ac.ictx, group);
  bind_own_intrinsics(ac.env, ac.ictx, group);
  install_host_verbs(ac.env, group, *ac.ictx.host, *ac.notify);
  bind_has(ac.env, program_has(group, ac.held));
}

void Runtime::Impl::install_host_verbs(const cvc::state_exec::environment_ptr &env,
                                       cvc::state_exec::exec_group *group,
                                       cvc::state_exec::async_host &host,
                                       const NotifyScope &notify) const {
  namespace se = cvc::state_exec;
  // The file verbs bind into the env as they are, in every group (bind_file_verbs): (file-info)
  // and (file-forget) are thread-safe, and (file-pick) is async, which a host_proxied wrapper
  // could not park; it opens its picker on the host thread itself.
  bind_file_verbs(env, group, host);
  // The backend's verbs drive its window and clipboard, which are the host thread's: they are
  // host_proxied (S1-Q7). In an `any` group each native runs on the pump_all caller's thread
  // (exec_group::install), as install_providers binds a host_proxied provider; in group 0, in a
  // host-affinity group and in the init lane they bind as they are. The notify verbs (§5.24) go
  // the same way: cvc::notify mirrors the app's feed into its notify.* subtree raw, on the
  // caller's thread.
  if (!group || group->opts().affinity != se::group_affinity::any) {
    bind_backend_verbs(env);
    bind_notify_verbs(env, notify);
    bind_mount_verbs(env, notify.scope); // ARIADNE_SHELL.md 4.2 (app-open), (app-close)
    return;
  }
  const se::environment_ptr scratch = se::environment::extend(env);
  bind_backend_verbs(scratch);
  bind_notify_verbs(scratch, notify);
  bind_mount_verbs(scratch, notify.scope);
  for (auto &[name, value] : scratch->bindings) {
    if (auto *fn = std::get_if<se::native_fn>(&value.v))
      group->install(env, name, std::move(*fn), se::provider_affinity::host_proxied);
    else
      env->set(name, std::move(value));
  }
}

void Runtime::Impl::bind_backend_verbs(
    const std::shared_ptr<cvc::state_exec::environment> &env) const {
  namespace se = cvc::state_exec;
  const std::weak_ptr<Backend *> ref = backend_ref;
  // Where the backend is now, if there still is one (a child may outlive the Runtime).
  const auto live = [ref]() -> Backend * {
    const std::shared_ptr<Backend *> b = ref.lock();
    return b ? *b : nullptr;
  };
  cvc::app *const on_app = &app;
  if (backend && backend->capabilities().clipboard)
    se::builtins::register_fn(
        env, "clipboard-set", [live, on_app](std::span<const se::value_t> args) -> se::value_t {
          if (args.size() != 1)
            throw se::dsl_error("TypeError", "clipboard-set: expected one argument, the text");
          const auto *text = std::get_if<std::string>(&args[0].v);
          if (!text)
            throw se::dsl_error("TypeError",
                                "clipboard-set: the text is a string, not " + args[0].type_name());
          Backend *b = live();
          return se::value_t(b && b->set_clipboard(*on_app, *text));
        });
  se::builtins::register_fn(
      env, "fullscreen", [live](std::span<const se::value_t> args) -> se::value_t {
        if (args.size() > 1)
          throw se::dsl_error("TypeError", "fullscreen: expected at most one argument, on or off");
        Backend *b = live();
        const bool can = b && b->capabilities().fullscreen;
        if (args.empty())
          return se::value_t(can && b->is_fullscreen());
        return se::value_t(can && b->set_fullscreen(args[0].is_truthy()));
      });
}

namespace {

// {handle name size type}: a picked file as a program sees it; never its path or its bytes.
cvc::state_exec::value_t file_dict(const std::string &handle, const FileSource &f) {
  namespace se = cvc::state_exec;
  return se::make_dict({
      {"handle", se::value_t(handle)},
      {"name", se::value_t(f.name())},
      {"size", se::value_t(static_cast<int64_t>(f.size()))},
      {"type", se::value_t(f.type())},
  });
}

std::string trimmed(const std::string &s) {
  const std::size_t a = s.find_first_not_of(" \t"), b = s.find_last_not_of(" \t");
  return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
}

// (file-pick [OPTS])'s options: accept (a list of ".ext" / "type/sub" / "type/*", or one string of
// them joined by commas, as an <input accept> is), multiple, title, dir.
FilePickRequest file_pick_request(std::span<const cvc::state_exec::value_t> args) {
  namespace se = cvc::state_exec;
  FilePickRequest req;
  if (args.size() > 1)
    throw se::dsl_error("TypeError", "file-pick: expected at most one argument, an options dict");
  if (args.empty() || args[0].is_nil())
    return req;
  const se::dict_ptr *d = std::get_if<se::dict_ptr>(&args[0].v);
  if (!d)
    throw se::dsl_error("TypeError",
                        "file-pick: the options are a dict, not " + args[0].type_name());
  const auto text = [](const se::value_t &v, const char *key) {
    const std::string *s = std::get_if<std::string>(&v.v);
    if (!s)
      throw se::dsl_error("TypeError", std::string("file-pick: '") + key + "' is a string");
    return *s;
  };
  for (const auto &[key, v] : **d) {
    if (key == "accept") {
      std::vector<std::string> parts;
      if (const se::list_ptr *l = std::get_if<se::list_ptr>(&v.v)) {
        for (const se::value_t &e : **l)
          parts.push_back(text(e, "accept"));
      } else {
        const std::string all = text(v, "accept");
        for (std::size_t at = 0; at <= all.size();) {
          const std::size_t comma = std::min(all.find(',', at), all.size());
          parts.push_back(all.substr(at, comma - at));
          at = comma + 1;
        }
      }
      for (const std::string &raw : parts) {
        const std::string p = trimmed(raw);
        if (p.empty())
          continue;
        if (p[0] != '.' && p.find('/') == std::string::npos)
          throw se::dsl_error("ValueError", "file-pick: an 'accept' entry is \".ext\", "
                                            "\"type/subtype\" or \"type/*\", not \"" +
                                                p + "\"");
        req.accept.push_back(p);
      }
    } else if (key == "multiple") {
      req.multiple = v.is_truthy();
    } else if (key == "title") {
      req.title = text(v, "title");
    } else if (key == "dir") {
      req.dir = text(v, "dir");
    } else {
      throw se::dsl_error("ValueError",
                          "file-pick: no option '" + key + "' (accept, multiple, title, dir)");
    }
  }
  return req;
}

// (save-url URL [OPTS])'s arguments (§5.29): the URL, an http(s), file, cvc or state URI, and the
// options name (the file's name), mime (its type) and auth (a credential name, as image:'s).
MediaSave save_url_request(std::span<const cvc::state_exec::value_t> args) {
  namespace se = cvc::state_exec;
  if (args.empty() || args.size() > 2)
    throw se::dsl_error("TypeError", "save-url: (save-url URL [OPTS])");
  const std::string *url = std::get_if<std::string>(&args[0].v);
  if (!url)
    throw se::dsl_error("TypeError", "save-url: the URL is a string, not " + args[0].type_name());
  if (!media::is_uri(*url))
    throw se::dsl_error("ValueError",
                        "save-url: \"" + *url + "\" is not an http(s), file, cvc or state URI");
  MediaSave m;
  m.url = *url;
  if (args.size() < 2 || args[1].is_nil())
    return m;
  const se::dict_ptr *d = std::get_if<se::dict_ptr>(&args[1].v);
  if (!d)
    throw se::dsl_error("TypeError",
                        "save-url: the options are a dict, not " + args[1].type_name());
  for (const auto &[key, v] : **d) {
    const std::string *s = std::get_if<std::string>(&v.v);
    if (key != "name" && key != "mime" && key != "auth")
      throw se::dsl_error("ValueError", "save-url: no option '" + key + "' (name, mime, auth)");
    if (!s)
      throw se::dsl_error("TypeError", "save-url: '" + key + "' is a string");
    (key == "name" ? m.name : key == "mime" ? m.mime : m.auth) = *s;
  }
  return m;
}

std::string handle_arg(const char *verb, std::span<const cvc::state_exec::value_t> args) {
  namespace se = cvc::state_exec;
  const std::string *h = args.size() == 1 ? std::get_if<std::string>(&args[0].v) : nullptr;
  if (!h)
    throw se::dsl_error("TypeError",
                        std::string(verb) + ": expected one argument, a handle file-pick returned");
  return *h;
}

} // namespace

void Runtime::Impl::bind_file_verbs(const std::shared_ptr<cvc::state_exec::environment> &env,
                                    cvc::state_exec::exec_group *group,
                                    cvc::state_exec::async_host &host) const {
  namespace se = cvc::state_exec;
  cvc::app *const app_ptr = &app;
  const std::weak_ptr<FilePickHub> weak_hub = file_pick;
  const std::weak_ptr<Backend *> ref = backend_ref;
  // The wait is the user's, not the program's: an I/O call, charged one step however long it is.
  se::async_options io;
  io.io = true;
  // (file-pick [OPTS]) / (file-pick-async ...): ask the user for a file; the program parks until
  // they pick or cancel. A list of {handle name size type}, empty when they cancelled. Bound where
  // there is a backend, so somewhere to ask: (has? "file-pick"). The program parks on its own
  // host (ictx.host: its group's, or the app's in group 0). The call starts on whatever thread
  // pumps the program, a pool worker in an `any` group: it claims the hub there, under its lock,
  // and opens the picker on the host thread (exec_pool::run_on_host for the program's group;
  // inline when this is that thread, or in group 0), where the window and the Runtime's own picker
  // are drawn. The group is the env's, as `host` is: both live as long as its programs. The answer
  // goes through the
  // host's finisher, as the host may be gone by then (an isolated document's group goes with its
  // Runtime).
  if (backend)
    se::register_async_fn(
        env, host, "file-pick",
        [app_ptr, weak_hub, ref, group,
         &host](std::span<const se::value_t> args) -> se::async_work {
          const FilePickRequest req = file_pick_request(args);
          se::async_work work; // no kernel: a program that can't park can't wait for the user
          work.start = [app_ptr, weak_hub, ref, req, group,
                        fin = host.make_finisher()](const std::shared_ptr<se::async_call> &call) {
            const std::shared_ptr<FilePickHub> hub = weak_hub.lock();
            if (!hub || !ref.lock())
              throw std::runtime_error("file-pick: the document is gone");
            {
              std::lock_guard<std::mutex> lock(hub->m);
              if (hub->busy)
                throw std::runtime_error("file-pick: a file picker is open already");
              hub->busy = true;
              hub->cancel = false;
            }
            auto settled = std::make_shared<std::atomic<bool>>(false);
            const std::weak_ptr<FilePickHub> weak = hub;
            FilePickDone done = [app_ptr, fin, call, settled,
                                 weak](std::vector<std::shared_ptr<FileSource>> files,
                                       std::string error) {
              if (settled->exchange(true))
                return;
              if (const std::shared_ptr<FilePickHub> h = weak.lock()) {
                std::lock_guard<std::mutex> lock(h->m);
                h->busy = false;
              }
              if (call->cancel_requested())
                return; // its host may be gone
              se::value_t value;
              std::exception_ptr failure;
              try {
                if (!error.empty())
                  throw std::runtime_error("file-pick: " + error);
                PickedFiles &table = picked_files(*app_ptr);
                if (table.size() + files.size() > PickedFiles::kMaxFiles)
                  throw std::runtime_error("file-pick: the app holds " +
                                           std::to_string(PickedFiles::kMaxFiles) +
                                           " files already; (file-forget H) some first");
                std::vector<se::value_t> out;
                for (const std::shared_ptr<FileSource> &f : files)
                  if (f)
                    out.push_back(file_dict(table.add(f), *f));
                value = se::make_list(std::move(out));
              } catch (...) {
                failure = std::current_exception();
              }
              if (call->settle(std::move(value), failure))
                fin.finish(call);
            };
            call->on_cancel([weak, settled] {
              if (settled->exchange(true))
                return;
              if (const std::shared_ptr<FilePickHub> h = weak.lock()) {
                std::lock_guard<std::mutex> lock(h->m);
                h->busy = false;
                h->cancel = true; // render() closes the window
              }
            });
            std::function<void()> open = [weak, ref, req, settled, done] {
              if (settled->load())
                return; // cancelled before the host thread came to it
              const std::shared_ptr<FilePickHub> h = weak.lock();
              const std::shared_ptr<Backend *> drawing = ref.lock();
              if (!h || !drawing) {
                done({}, "the document is gone");
                return;
              }
              h->composed.close(); // one a cancelled program left (busy was clear)
              if (*drawing && (*drawing)->pick_files(req, done))
                return; // the host's own (arigl: the system's file dialog)
              if (browser_pick_files(req, done))
                return; // a browser page's <input type=file>
              if (!*drawing) {
                done({}, "there is no window here to pick a file in");
                return;
              }
              h->composed.open(req, done); // drawn by render() from the next frame
            };
            if (group)
              app_ptr->exec_pool().run_on_host(*group, std::move(open));
            else
              open();
          };
          return work;
        },
        io);
  // (file-info H): {handle name size type} of a file the app holds, or nil.
  se::builtins::register_fn(env, "file-info",
                            [app_ptr](std::span<const se::value_t> args) -> se::value_t {
                              const std::string h = handle_arg("file-info", args);
                              const std::shared_ptr<FileSource> f = picked_files(*app_ptr).find(h);
                              return f ? file_dict(h, *f) : se::value_t();
                            });
  // (file-forget H): let go of a picked file; #t when the app held it.
  se::builtins::register_fn(
      env, "file-forget", [app_ptr](std::span<const se::value_t> args) -> se::value_t {
        return se::value_t(picked_files(*app_ptr).forget(handle_arg("file-forget", args)));
      });
  // §5.29 (save-url URL [OPTS]) / (save-url-async ...): save what URL names to a file the user
  // keeps, as a context menu's Save image... does. The program parks until it is saved, the user
  // cancelled the dialog, or it failed: {ok cancelled name size error}, never the path. Bound where
  // there is a backend: (has? "save-url"). The call hands the save to the Runtime's hub, under its
  // lock, from wherever its program runs; drain() starts it on the drawing thread.
  const std::weak_ptr<SaveHub> weak_saves = saves;
  if (backend)
    se::register_async_fn(
        env, host, "save-url",
        [weak_saves, &host](std::span<const se::value_t> args) -> se::async_work {
          const MediaSave m = save_url_request(args);
          se::async_work work;
          work.start = [weak_saves, m,
                        fin = host.make_finisher()](const std::shared_ptr<se::async_call> &call) {
            const std::shared_ptr<SaveHub> hub = weak_saves.lock();
            if (!hub)
              throw std::runtime_error("save-url: the document is gone");
            auto settled = std::make_shared<std::atomic<bool>>(false);
            SaveJob job;
            job.save = m;
            job.done = [call, fin, settled](const SaveOutcome &o) {
              if (settled->exchange(true) || call->cancel_requested())
                return; // its host may be gone
              se::value_t value = se::make_dict({
                  {"ok", se::value_t(o.ok)},
                  {"cancelled", se::value_t(o.cancelled)},
                  {"name", se::value_t(o.name)},
                  {"size", se::value_t(static_cast<int64_t>(o.size))},
                  {"error", se::value_t(o.error)},
              });
              if (call->settle(std::move(value), nullptr))
                fin.finish(call);
            };
            call->on_cancel([settled] { settled->store(true); });
            hub->ask(std::move(job));
          };
          return work;
        },
        io);
}

// ---- §5.29 saving a picture or a video ----------------------------------------------------------

void Runtime::Impl::pump_saves() {
  std::vector<SaveJob> asked;
  std::vector<SaveHub::Step> steps;
  {
    std::lock_guard<std::mutex> lock(saves->m);
    asked.swap(saves->asked);
    steps.swap(saves->steps);
  }
  if (backend)
    for (Backend::MediaSaveAsk &a : backend->take_media_saves()) {
      if (a.prepare) {
        prepare_save(a.save);
        continue;
      }
      SaveJob job;
      job.save = std::move(a.save);
      job.menu = true;
      start_save(std::move(job));
    }
  for (SaveJob &j : asked)
    start_save(std::move(j));
  for (SaveHub::Step &s : steps)
    s.run();
}

void Runtime::Impl::save_naming(const MediaSave &m, const std::string &bytes, std::string &name,
                                std::string &mime) {
  mime = m.mime;
  if (mime.empty())
    mime = sniff_mime(bytes);
  if (mime.empty()) { // what its name says it is, if anything
    const std::string guess = guess_mime_type(save_file_name(m.url, m.name, "", m.video));
    if (guess != "application/octet-stream")
      mime = guess;
  }
  name = save_file_name(m.url, m.name, mime, m.video);
}

void Runtime::Impl::prepare_save(const MediaSave &m) {
#ifdef __EMSCRIPTEN__
  const std::string key = media_save_key(m);
  const bool fresh = prepared_save && (!prepared_save->staged ||
                                       std::chrono::steady_clock::now() - prepared_save->staged_at <
                                           std::chrono::seconds(100));
  if (prepared_save && ((prepared_save->key == key && fresh) || !prepared_save->waiting.empty()))
    return; // this one's, or one a save waits for
  if (prepared_save && prepared_save->staged)
    browser_save_drop(prepared_save->key);
  prepared_save = std::make_unique<PreparedSave>();
  prepared_save->key = key;
  const long gen = prepared_save->gen = ++prepared_gen;
  const std::weak_ptr<SaveHub> weak = saves;
  image_cache().fetch(m.url, m.auth, [this, weak, gen, m](media::FetchResult r) {
    const std::shared_ptr<SaveHub> hub = weak.lock();
    if (!hub)
      return;
    auto got = std::make_shared<media::FetchResult>(std::move(r));
    hub->post({[this, gen, m, got] {
                 if (!prepared_save || prepared_save->gen != gen)
                   return; // another menu's since
                 PreparedSave &p = *prepared_save;
                 std::vector<SaveJob> waiting = std::move(p.waiting);
                 SaveOutcome o;
                 if (got->ok) {
                   std::string name, mime;
                   save_naming(m, got->bytes, name, mime);
                   o.name = name;
                   o.size = got->bytes.size();
                   p.staged = browser_save_stage(p.key, name, mime, got->bytes);
                   p.staged_at = std::chrono::steady_clock::now();
                 }
                 if (!p.staged) { // a failed fetch: a save asked for again fetches again
                   prepared_save.reset();
                   for (SaveJob &j : waiting) {
                     if (got->ok) {
                       start_save(std::move(j));
                       continue;
                     }
                     SaveOutcome f;
                     f.error = got->error;
                     f.name = save_file_name(j.save.url, j.save.name, j.save.mime, j.save.video);
                     finish_save(j, f);
                   }
                   return;
                 }
                 p.outcome = o;
                 if (waiting.empty())
                   return; // staged for the pointer's release, or a Save to come
                 browser_save_commit(p.key);
                 prepared_save.reset();
                 o.ok = o.browser = true;
                 for (SaveJob &j : waiting)
                   finish_save(j, o);
               },
               nullptr});
  });
#else
  (void)m; // natively a Save fetches when it is picked: no click rules to keep
#endif
}

void Runtime::Impl::start_save(SaveJob job) {
  if (!media::is_uri(job.save.url)) {
    SaveOutcome o;
    o.error = "it is not a URL (http(s), file, cvc or state)";
    finish_save(job, o);
    return;
  }
#ifdef __EMSCRIPTEN__
  // A menu's prepared save: downloaded now if its bytes are staged, else when they come.
  if (prepared_save && prepared_save->key == media_save_key(job.save)) {
    if (!prepared_save->staged) {
      prepared_save->waiting.push_back(std::move(job));
      return;
    }
    const bool done = browser_save_commit(prepared_save->key);
    SaveOutcome o = prepared_save->outcome;
    prepared_save.reset();
    if (done) {
      o.ok = o.browser = true;
      finish_save(job, o);
      return;
    } // its page let it go: fetch it again
  }
#endif
  const std::weak_ptr<SaveHub> weak = saves;
  auto held = std::make_shared<SaveJob>(std::move(job));
  image_cache().fetch(held->save.url, held->save.auth, [this, weak, held](media::FetchResult r) {
    const std::shared_ptr<SaveHub> hub = weak.lock();
    if (!hub) {
      if (held->done) {
        SaveOutcome o;
        o.error = "the document is gone";
        held->done(o);
      }
      return;
    }
    auto got = std::make_shared<media::FetchResult>(std::move(r));
    hub->post({[this, held, got] { save_fetched(std::move(*held), std::move(*got)); },
               [held] {
                 if (held->done) {
                   SaveOutcome o;
                   o.error = "the document is gone";
                   held->done(o);
                 }
               }});
  });
}

void Runtime::Impl::save_fetched(SaveJob job, media::FetchResult r) {
  if (!r.ok) {
    SaveOutcome o;
    o.error = r.error.empty() ? std::string("the fetch failed") : r.error;
    o.name = save_file_name(job.save.url, job.save.name, job.save.mime, job.save.video);
    finish_save(job, o);
    return;
  }
  std::string name, mime;
  save_naming(job.save, r.bytes, name, mime);
  save_deliver(std::move(job), name, mime, std::make_shared<const std::string>(std::move(r.bytes)));
}

void Runtime::Impl::save_deliver(SaveJob job, const std::string &name, const std::string &mime,
                                 std::shared_ptr<const std::string> bytes) {
  // 1. The host's own Save dialog (arigl: the system's), one at a time.
  if (backend && saves->dialog_open) {
    SaveOutcome o;
    o.name = name;
    o.error = "a Save dialog is open already";
    finish_save(job, o);
    return;
  }
  if (backend) {
    SavePathRequest req;
    req.name = name;
    req.mime = mime;
    const std::string major = mime.substr(0, mime.find('/'));
    req.title = major == "image" ? "Save image" : major == "video" ? "Save video" : "Save file";
    const std::weak_ptr<SaveHub> weak = saves;
    auto held = std::make_shared<SaveJob>(job);
    SavePathDone done = [this, weak, held, name, bytes](std::string path, std::string error) {
      const std::shared_ptr<SaveHub> hub = weak.lock();
      if (!hub) {
        if (held->done) {
          SaveOutcome o;
          o.error = "the document is gone";
          held->done(o);
        }
        return;
      }
      hub->post({[this, held, name, bytes, path, error] {
                   saves->dialog_open = false;
                   SaveOutcome o;
                   o.name = name;
                   if (!error.empty()) {
                     o.error = error;
                     finish_save(*held, o);
                   } else if (path.empty()) {
                     o.cancelled = true;
                     finish_save(*held, o);
                   } else {
                     save_write(std::move(*held), path, name, bytes, /*fresh=*/false);
                   }
                 },
                 nullptr});
    };
    if (backend->ask_save_path(req, std::move(done))) {
      saves->dialog_open = true;
      return;
    }
  }
  // 2. A browser page: a download.
  if (browser_save_file(name, mime, *bytes)) {
    SaveOutcome o;
    o.ok = o.browser = true;
    o.name = name;
    o.size = bytes->size();
    finish_save(job, o);
    return;
  }
  // 3. A new file in the Downloads folder (or the host's set_save_dir).
  std::string dir = saves->dir, why;
  if (dir.empty())
    dir = downloads_dir(&why);
  if (dir.empty()) {
    SaveOutcome o;
    o.name = name;
    o.error = why;
    finish_save(job, o);
    return;
  }
  save_write(std::move(job), dir, name, std::move(bytes), /*fresh=*/true);
}

void Runtime::Impl::save_write(SaveJob job, const std::string &path, const std::string &name,
                               std::shared_ptr<const std::string> bytes, bool fresh) {
  // Off the frame: a video can be tens of MiB. `fresh`: a new file in the folder `path`, never
  // over one there; else the file `path` the user chose, replaced.
  const std::weak_ptr<SaveHub> weak = saves;
  auto held = std::make_shared<SaveJob>(std::move(job));
  app.submitPooled([this, weak, held, path, name, bytes, fresh] {
    SaveOutcome o;
    o.name = name;
    o.size = bytes->size();
    std::string error;
    if (fresh) {
      o.path = save_new_file(path, name, *bytes, &error);
      o.ok = !o.path.empty();
    } else {
      o.ok = save_file_at(path, *bytes, &error);
      o.path = path;
    }
    if (o.ok) {
      const std::size_t slash = o.path.find_last_of("/\\");
      o.name = slash == std::string::npos ? o.path : o.path.substr(slash + 1);
    } else {
      o.error = error;
    }
    const std::shared_ptr<SaveHub> hub = weak.lock();
    if (!hub) {
      if (held->done)
        held->done(o); // written, or not, all the same
      return;
    }
    hub->post({[this, held, o] { finish_save(*held, o); },
               [held, o] {
                 if (held->done)
                   held->done(o);
               }});
  });
}

void Runtime::Impl::finish_save(const SaveJob &job, const SaveOutcome &o) {
  // Where a file was written, the user is told where: a terminal shows no download.
  if (o.ok && !o.browser && !o.path.empty()) {
    cvc::notification n;
    n.text = "Saved " + o.name + " to " + o.path;
    n.path = prefix;
    cvc::notify(app, std::move(n));
  } else if (job.menu && !o.ok && !o.cancelled) {
    cvc::notification n;
    n.text = "Could not save " + (o.name.empty() ? std::string("it") : o.name) + ": " + o.error;
    n.level = 2;
    n.path = prefix;
    cvc::notify(app, std::move(n));
  }
  if (job.done)
    job.done(o);
}

cvc::state_exec::spawn_setup
Runtime::Impl::spawn_child(const cvc::state_exec::intrinsics_context &parent,
                           std::shared_ptr<const NotifyScope> notify,
                           std::shared_ptr<const std::set<std::string>> held) {
  namespace se = cvc::state_exec;
  // Everything from the parent's context (td 4.15), nothing by path: no tree walk, so it is safe on
  // the pumping thread, and a child of a deleted chroot keeps writing its parent's detached node,
  // as the library's default context does.
  auto ac = std::make_shared<ActionContext>();
  ac->proc = se::make_process();
  ac->proc->status = se::process_status::ready;
  ac->ictx.sched = parent.sched;
  ac->ictx.host = parent.host;
  ac->ictx.tracker = &ac->tracker;
  ac->ictx.proc = ac->proc;
  ac->ictx.root = parent.root;
  ac->ictx.root_pins = parent.root_pins;
  ac->ictx.root_path = parent.root_path;
  ac->ictx.uid = parent.uid;
  ac->ictx.cluster_id = parent.cluster_id;
  ac->ictx.node_id = parent.node_id;
  ac->ictx.channels = parent.channels;
  ac->ictx.document = parent.document;
  ac->ictx.lock_default_timeout = parent.lock_default_timeout;
  ac->ictx.lock_wait_allowed = parent.lock_wait_allowed;
  ac->ictx.spawn_context = parent.spawn_context; // grandchildren get one too
  ac->scope = parent.root_path;
  ac->notify = std::move(notify); // its parent's: mount_names is the host thread's
  ac->held = std::move(held);     // ... and its grants, for the same reason
  se::exec_group *const group = group_ && parent.sched == &group_->sched() ? group_.get() : nullptr;
  finish_context(*ac, group);
  // A child it forks shares `ac`, which the fork's process keeps alive too (process::keep_alive).
  ac->ictx.on_child = [this](int pid) { spawned_children_.push_back(pid); };
  // Recorded so the drain reaps it once it finishes: its process keeps `ac` alive.
  return se::spawn_setup{ac->env, ac, [this](int pid) { spawned_children_.push_back(pid); }};
}

void Runtime::Impl::submit_action(const QueuedAction &action) {
  namespace se = cvc::state_exec;
  const std::string &action_prefix = action.prefix;
  const std::string &script = action.event;
  // §7.4 per-activation caps, tree settings read on every submission (action_max_steps,
  // action_max_time, action_max_memory): the STEP cap is the deterministic guard on an action's
  // real work, its async calls' included; time (a se::time_budget, which time the thread spends
  // descheduled cannot use up) is a generous anti-hang backstop; memory the §7.4 cap. Full env,
  // writes chrooted to the action's mount prefix.
  try {
    cvc::state &root = cvc::state::instance(app);
    const uint64_t max_bytes = action_max_memory(root);
    std::shared_ptr<ActionContext> ac =
        make_action_context(action_prefix, action_lock_timeout(root));
    se::async_scheduler &sched = current_sched(); // the document's: its group's, else group 0's
    // A child it spawns runs in the document's scope with the action's lock default.
    ac->ictx.spawn_context = [this, notify = ac->notify,
                              held = ac->held](const se::intrinsics_context &parent) {
      return spawn_child(parent, notify, held);
    };
    // §4.6 event scope: a widget pointer handler runs with an `event` dict of the item's local
    // pointer coords, so the program can read (get-attr event "x"/"y"/"dx"/"dy"/"button").
    if (action.has_event) {
      std::vector<std::pair<std::string, se::value_t>> f;
      f.emplace_back("x", se::value_t(static_cast<double>(action.ex)));
      f.emplace_back("y", se::value_t(static_cast<double>(action.ey)));
      f.emplace_back("dx", se::value_t(static_cast<double>(action.edx)));
      f.emplace_back("dy", se::value_t(static_cast<double>(action.edy)));
      f.emplace_back("button", se::value_t(static_cast<int64_t>(action.ebutton)));
      ac->env->set("event", se::make_dict(std::move(f)));
    } else if (!action.event_fields.empty() || !action.event_values.empty()) {
      std::vector<std::pair<std::string, se::value_t>> f;
      for (const auto &kv : action.event_fields)
        f.emplace_back(kv.first, se::value_t(kv.second));
      for (const auto &kv : action.event_values)
        f.push_back(kv);
      ac->env->set("event", se::make_dict(std::move(f)));
    }
    // A child the action forks shares this context: recorded, so the context stays until it ends.
    ac->ictx.on_child = [raw = ac.get()](int pid) { raw->children.push_back(pid); };
    ac->done = action.done;
    ac->script = script;
    se::execute_options opts;
    opts.env = ac->env;
    opts.owner = owner_; // so ~Impl's kill_owner reaps exactly this document's processes
    own_scope(opts, *ac);
    opts.max_steps = action_max_steps(root);
    opts.max_time = action_max_time(root);
    opts.max_memory = max_bytes;
    // §6.6 its scope's functions first. Throws se::parse_error on a syntax error.
    const int pid = sched.execute(action.functions + script, opts);
    live_actions_[pid] = std::move(ac); // keep the context alive until the process (and the
                                        // children it forked) end
  } catch (const std::exception &e) {
    if (action.done)
      action.done(std::string("failed: ") + e.what());
    else
      warn_once(std::string("ari: on: action program failed: ") + e.what() + " [" + script + "]");
  }
}

cvc::state_exec::async_scheduler *Runtime::Impl::scheduler_to_pump() {
  namespace se = cvc::state_exec;
  if (used_scheduler_)
    return &app.exec_scheduler(); // this document's own processes: pumped every drain, as always
  if (!seen_scheduler_)
    seen_scheduler_ = exec_scheduler_probe::built(app);
  if (!seen_scheduler_)
    return nullptr; // nothing ever used it: nothing to pump, and it stays unbuilt
  // Someone else's processes (a scene's overlay tasks, a host verb's): pumped while any is live,
  // and while anything posted waits there (S2.md section 0 row 22): a router entry homed at group
  // 0, such as an isolated document's send or wait on a `/` channel, would wait for a live process
  // there, which may never come. A post while none is live joins its pending messages, in order.
  const se::scheduler_stats s = seen_scheduler_->get_stats();
  const int live = s.total_processes - s.terminated - s.killed - s.failed;
  return live > 0 || seen_scheduler_->has_pending_ingress() ? seen_scheduler_ : nullptr;
}

void Runtime::Impl::sweep_document() {
  if (cvc::state_exec::async_scheduler *sched = group_            ? &group_->sched()
                                                : used_scheduler_ ? &app.exec_scheduler()
                                                                  : nullptr)
    sweep_actions(*sched);
}

bool Runtime::Impl::starts_frame() {
  if (!pump_clock_)
    pump_clock_ = pump_clock_of(app);
  std::lock_guard<std::mutex> lock(pump_clock_->mutex);
  const bool next = pumped_at_ == pump_clock_->frames; // pumped since the last boundary?
  if (next)
    ++pump_clock_->frames;
  pumped_at_ = pump_clock_->frames;
  return next;
}

void Runtime::Impl::pump_frame() {
  namespace se = cvc::state_exec;
  se::async_scheduler *sched = scheduler_to_pump();
  // Group 0's token while it is pumped here, as exec_pump_frame takes it (S1-Q4). Another thread
  // may hold it a moment (pycvc's ExecPool.run_all drains group 0's posts under it, S2b): group 0
  // is skipped for this drain then, and what it has waits for the next.
  std::optional<se::async_scheduler::pump_owner_scope> token;
  if (sched) {
    try {
      token.emplace(*sched);
    } catch (const std::runtime_error &) {
      sched = nullptr;
    }
  }
  if (sched) {
    // Apply any cross-thread post_message() deliveries FIRST, so a delivery that readies a parked
    // (msg-recv …) receiver is seen by sync_run's has_runnable() gate — otherwise, with everything
    // parked, sync_run would skip stepping and never drain the ingress itself.
    sched->drain_ingress();
    // Frame boundary: re-ready any process parked by (await …) in a PRIOR frame. Called here, once
    // per app frame however many Runtimes pump (starts_frame), NOT inside step()/sync_run, so an
    // await crosses exactly one frame.
    if (starts_frame())
      sched->wake_awaiting();
    // Pump a bounded slice: a quick (non-suspending) action completes within THIS drain (so a
    // fire-then-check interaction still sees its effect immediately), while an action that parks
    // on await/sleep/msg-recv yields and resumes on a later frame. Global step + time backstops
    // bound the pump -- se::time_budgets, so on a loaded machine, where the OS deschedules this
    // thread, a quick action still completes within this drain; per-process step caps kill
    // runaways.
    sched->sync_run(kPumpMaxSteps, kPumpMaxSeconds);
  }
  token.reset(); // not held across the document's group's pump
  // An isolated document's group, one pump through the app's pool, as exec_pump_frame() would
  // pump it: a host with one Runtime that opts in may keep calling drain() (td 4.12). Its pump
  // drains, wakes the frame-waiters and runs a slice of the same budget.
  if (group_) {
    se::exec_group *const groups[] = {group_.get()};
    app.exec_pool().pump_all(groups, se::pump_budget{kPumpMaxSteps, kPumpMaxSeconds},
                             std::chrono::steady_clock::now() + std::chrono::milliseconds(100));
  }
}

void Runtime::Impl::sweep_actions(cvc::state_exec::async_scheduler &sched) {
  namespace se = cvc::state_exec;
  // Sweep: a finished process is reaped (nothing reads its result), a killed or failed one is
  // reported, and a still-parked one is kept for a later frame. A run_program one's caller is told
  // how it ended instead.
  const auto reap_ended = [&](int pid, const char *what, const std::string &script = std::string(),
                              const std::function<void(const std::string &)> &done = {}) {
    const auto info = sched.get_process_info(pid);
    if (info && !se::is_finished(info->status))
      return false; // ready/running/waiting — still in flight (e.g. parked on await/msg-recv)
    if (done) {     // run_program: its caller reports it
      std::string how;
      if (info && info->status == se::process_status::killed)
        how = "did not complete: " + sched.get_exit_error(pid).value_or("killed");
      else if (info && info->status == se::process_status::failed)
        how = "failed: " + failure_report(info->exit_exception, info->exit_error);
      sched.reap(pid);
      done(how);
      return true;
    }
    if (info && info->status == se::process_status::killed)
      // The exit error says why: a syntax error (scripts are read inside the process), a runtime
      // error or a resource limit.
      warn_once(std::string("ari: on: ") + what + " did not complete: " +
                sched.get_exit_error(pid).value_or("killed") + program_tag(script));
    else if (info && info->status == se::process_status::failed)
      // An exception the program didn't catch: its class, message and where it was raised.
      warn_once(std::string("ari: on: ") + what + " failed: " +
                failure_report(info->exit_exception, info->exit_error) + program_tag(script));
    sched.reap(pid);
    return true;
  };
  sweeping_actions_ = true;
  for (auto it = live_actions_.begin(); it != live_actions_.end();) {
    ActionContext &ac = *it->second;
    if (!ac.ended)
      ac.ended = reap_ended(it->first, "action program", ac.script, ac.done);
    // The children it forked share its context, so the context stays until they end too.
    std::erase_if(ac.children,
                  [&](int pid) { return reap_ended(pid, "forked program", ac.script); });
    if (ac.ended && ac.children.empty())
      it = live_actions_.erase(it);
    else
      ++it;
  }
  sweeping_actions_ = false;
  for (auto &[pid, ac] : started_in_sweep_)
    live_actions_[pid] = std::move(ac);
  started_in_sweep_.clear();
  // Spawned children, and what they and the residents fork: a finished one's process would keep
  // its context (an environment with every intrinsic) for the app's life.
  std::erase_if(spawned_children_, [&](int pid) { return reap_ended(pid, "spawned program"); });
  // A resident never finishes on its own, so one that was killed or failed is reported and
  // restarted (retire_if_stopped).
  retire_if_stopped(sched, tick_resident_, "on_tick:", tick_channel());
  retire_if_stopped(sched, key_resident_, "on_key:", key_channel());
  retire_if_stopped(sched, pointer_resident_, "on_pointer:", pointer_channel());
  retire_if_stopped(sched, signal_resident_, "on_signal:", signal_channel());
  for (auto &[sub_prefix, t] : mount_ticks_)
    retire_if_stopped(sched, t.resident, "on_tick: of mount '" + sub_prefix + "'", t.channel);
}

void Runtime::Impl::retire_if_stopped(cvc::state_exec::async_scheduler &sched, Resident &r,
                                      const std::string &lane, const std::string &channel) {
  namespace se = cvc::state_exec;
  if (r.pid < 0)
    return;
  const auto info = sched.get_process_info(r.pid);
  if (!info || !se::is_finished(info->status))
    return;
  if (info->status == se::process_status::killed)
    warn_once("ari: " + lane + " resident [" + channel +
              "] stopped: " + sched.get_exit_error(r.pid).value_or("killed"));
  else if (info->status == se::process_status::failed)
    warn_once("ari: " + lane + " resident [" + channel +
              "] failed: " + failure_report(info->exit_exception, info->exit_error));
  sched.reap(r.pid);
  r.pid = -1;
  r.ctx.reset(); // a child it forked keeps the context alive itself (process::keep_alive)
  r.quick_stops = drains_ - r.started < kQuickStop ? r.quick_stops + 1 : 1;
  if (r.quick_stops > kMaxQuickStops) {
    r.given_up = true;
    warn_once("ari: " + lane + " resident [" + channel + "] stopped " +
              std::to_string(kMaxQuickStops) +
              " times in a row soon after starting: not restarted "
              "again");
    return;
  }
  // At once after its first stop, then 1, 2, 4 ... drains later.
  const std::uint64_t delay =
      r.quick_stops <= 1 ? 0 : std::min(kRestartMaxDelay, std::uint64_t{1} << (r.quick_stops - 2));
  r.retry_at = drains_ + delay;
}

void Runtime::Impl::stop_resident(Resident &r) {
  if (r.pid >= 0) { // killed and reaped (S1-Q8): its record goes with it, on a reload too
    cvc::state_exec::async_scheduler &sched = current_sched();
    sched.kill(r.pid);
    sched.reap(r.pid);
  }
  r = Resident{};
}

void Runtime::Impl::ensure_resident(const std::string &channel, const std::string &body,
                                    Resident &r) {
  ensure_resident(channel, body, r, prefix, doc_functions);
}

void Runtime::Impl::ensure_resident(const std::string &channel, const std::string &body,
                                    Resident &r, const std::string &chroot,
                                    const std::string &functions) {
  if (body.empty() || r.pid >= 0 || r.given_up || drains_ < r.retry_at)
    return; // nothing to run, already running (residents submit ONCE), or not restarted yet
  namespace se = cvc::state_exec;
  try {
    // Resident writes confined to its document or mount. A resident has no with-lock default
    // (docs/STATE_EXEC_LOCKS.md 14.2 Q1): a request with no :timeout waits as long as it takes.
    std::shared_ptr<ActionContext> ac = make_action_context(chroot, std::nullopt);
    se::async_scheduler &sched = current_sched(); // the document's: its group's, else group 0's
    // A child it spawns runs in the resident's scope (its document's or its mount's); one it forks
    // shares the resident's context, which lives until the Runtime does. Both are reaped once they
    // end.
    ac->ictx.spawn_context = [this, notify = ac->notify,
                              held = ac->held](const se::intrinsics_context &parent) {
      return spawn_child(parent, notify, held);
    };
    ac->ictx.on_child = [this](int pid) { spawned_children_.push_back(pid); };
    // Park-then-run loop on `channel`: msg-recv parks the process between deliveries and BINDS each
    // delivered value as `event`, so an on_key/on_pointer body reads (get-attr event "key");
    // on:tick simply ignores `event`. The body runs once per delivery, then loops back to park. No
    // total step cap (a resident loops forever; the per-frame pump budget bounds each activation,
    // and the msg-recv guarantees a park every iteration).
    const std::string wrapped = functions + // §6.6 defined once, before the loop
                                "(while t (let ((event (msg-recv \"" + channel + "\"))) " + body +
                                "))";
    se::execute_options opts;
    opts.env = ac->env;
    opts.owner = owner_; // reaped by ~Impl's kill_owner(owner_) on teardown
    own_scope(opts, *ac);
    opts.max_steps = 0;  // unlimited total — a resident loops forever (see above)
    opts.max_time = 0.0; // unlimited TOTAL run-time; the per-STEP cap below is the hang backstop
    // §13.8 resident per-STEP time cap: a resident has no total budget, so give it a per-step
    // deadline (fresh each step) so a single step that loops in-place (a runaway native builtin)
    // can't HANG the host's per-frame drain — it is aborted + the resident killed. A healthy
    // on:tick/on_key body is far under this, and a per-node DSL loop is bounded by the pump's own
    // per-drain cap; this only bites an in-STEP loop the pump can't interrupt. One it kills is
    // restarted (retire_if_stopped).
    opts.max_step_time = resident_max_step_time(cvc::state::instance(app));
    r.pid = sched.execute(wrapped, opts);
    r.ctx = std::move(ac);
    r.started = drains_;
  } catch (const std::exception &e) {
    r.given_up = true; // its text is what failed: starting it again would fail the same way
    warn_once("ari: resident failed to start [" + channel + "]: " + e.what());
  }
}

void Runtime::Impl::ensure_tick_resident() {
  ensure_resident(tick_channel(), tick_script_, tick_resident_);
  // A mount's starts as the mount is first drawn (ensure_mount_tick); this restarts a stopped one,
  // drawn or not.
  for (auto &[sub_prefix, t] : mount_ticks_)
    ensure_resident(t.channel, t.body, t.resident, sub_prefix, t.functions);
}

void Runtime::Impl::post_tick() {
  if (tick_resident_.pid < 0 && mount_ticks_.empty())
    return; // no tick resident registered
  // Wake each tick resident for one activation this frame. Post only when nothing is queued on its
  // channel, so a resident that can't keep up doesn't accrue a backlog of ticks (coalesce). To the
  // document's scheduler, where the residents run, between pumps.
  auto &sched = current_sched();
  const auto wake = [&](const std::string &channel) {
    if (sched.pending_message_count(channel) == 0)
      sched.post_message(channel, cvc::state_exec::value_t(std::string("tick")));
  };
  if (tick_resident_.pid >= 0)
    wake(tick_channel());
  for (const auto &[sub_prefix, t] : mount_ticks_)
    if (t.resident.pid >= 0)
      wake(t.channel);
}

void Runtime::Impl::ensure_mount_tick(const std::string &sub_prefix, const Widget &w) {
  MountTick &t = mount_ticks_[sub_prefix];
  if (!t.channel.empty())
    return; // started once per mount, not per frame (ensure_tick_resident restarts it)
  t.body = w.tick_script;
  t.functions = w.functions ? w.functions->prelude : std::string();
  t.channel = owner_ + "#tick@" + sub_prefix;
  t.repeat = emitting_repeat_;
  t.instance = emitting_instance_;
  ensure_resident(t.channel, t.body, t.resident, sub_prefix, t.functions);
}

void Runtime::Impl::drop_mount_ticks() {
  if (mount_ticks_.empty())
    return;
  // The new tree's mounts outside any repeat: (a repeated one is made per frame), by sub-prefix,
  // composed as emit_container composes them.
  std::map<std::string, std::string> kept;
  std::vector<std::pair<const Widget *, std::string>> todo{{&root, prefix}}; // node, its scope
  for (const auto &[sub, slot] : mount_slots_) // ARIADNE_SHELL.md 4.2 a mounts: entry's, kept
    if (slot.wrapper)
      todo.emplace_back(slot.wrapper.get(), slot.parent_scope);
  while (!todo.empty()) {
    auto [w, inner] = std::move(todo.back());
    todo.pop_back();
    if (!w->repeat.empty())
      continue;
    if (!w->scope.empty()) {
      inner = resolve_bind(inner, w->scope);
      if (!w->tick_script.empty())
        kept[inner] = (w->functions ? w->functions->prelude : std::string()) + w->tick_script;
    }
    for (auto c = w->children.rbegin(); c != w->children.rend(); ++c)
      todo.emplace_back(&*c, inner);
  }
  for (auto it = mount_ticks_.begin(); it != mount_ticks_.end();) {
    const auto k = kept.find(it->first);
    if (it->second.repeat || k == kept.end() ||
        k->second != it->second.functions + it->second.body) {
      stop_resident(it->second.resident);
      it = mount_ticks_.erase(it);
    } else {
      ++it;
    }
  }
}

void Runtime::Impl::sweep_repeated_ticks(const Widget *tmpl, int count) {
  for (auto it = mount_ticks_.begin(); it != mount_ticks_.end();) {
    if (it->second.repeat == tmpl && it->second.instance >= count) {
      stop_resident(it->second.resident);
      it = mount_ticks_.erase(it);
    } else {
      ++it;
    }
  }
}

void Runtime::Impl::ensure_input_residents() {
  ensure_resident(key_channel(), key_script_, key_resident_);
  ensure_resident(pointer_channel(), pointer_script_, pointer_resident_);
}

void Runtime::Impl::deliver_input(const InputEvent &ev) {
  namespace se = cvc::state_exec;
  // Keyboard events feed the on_key resident, mouse events the on_pointer resident, and the tap
  // the ones it takes. Drop the event if nothing takes it (no point queueing events nothing will
  // drain).
  const bool keyboard = ev.is_keyboard();
  const bool resident = (keyboard ? key_resident_ : pointer_resident_).pid >= 0;
  const bool tapped =
      (tap_kinds_.load() & (keyboard ? Runtime::tap_key : Runtime::tap_pointer)) != 0;
  if (!resident && !tapped)
    return;
  // A dict with EVERY field always populated: get-attr throws on a missing key, so an on_* body can
  // read any field of any event kind (unused fields are 0/""). These names are the .ari event
  // contract — (get-attr event "kind"/"key"/"mods"/"x"/"y"/"dx"/"dy"/"button"/"clicks"/"repeat").
  std::vector<std::pair<std::string, se::value_t>> f;
  f.emplace_back("kind", se::value_t(std::string(ev.kind_name())));
  f.emplace_back("key", se::value_t(ev.key));
  f.emplace_back("mods", se::value_t(static_cast<int64_t>(ev.mods)));
  f.emplace_back("repeat", se::value_t(ev.repeat));
  f.emplace_back("x", se::value_t(static_cast<double>(ev.x)));
  f.emplace_back("y", se::value_t(static_cast<double>(ev.y)));
  f.emplace_back("dx", se::value_t(static_cast<double>(ev.dx)));
  f.emplace_back("dy", se::value_t(static_cast<double>(ev.dy)));
  f.emplace_back("button", se::value_t(static_cast<int64_t>(ev.button)));
  f.emplace_back("clicks", se::value_t(static_cast<int64_t>(ev.clicks)));
  if (tapped) // a dict of its own: a program may change the one it gets
    tap(keyboard ? "key" : "pointer", se::make_dict(f));
  // NO coalesce (unlike post_tick): every event must arrive. A burst in one frame FIFO-queues on
  // the channel and the resident drains them one per loop iteration during the pump.
  if (resident)
    current_sched().post_message(keyboard ? key_channel() : pointer_channel(),
                                 se::make_dict(std::move(f)));
}

void Runtime::Impl::deliver_tapped() {
  std::vector<std::pair<std::string, cvc::state_exec::value_t>> events;
  {
    std::lock_guard<std::mutex> lock(tapped_mutex_);
    events.swap(tapped_);
  }
  if (tap_)
    for (const auto &[kind, event] : events)
      tap_(kind, event);
}

Runtime::Runtime(cvc::app &app, std::string prefix) : m_(new Impl(app, std::move(prefix))) {}

Runtime::~Runtime() = default;

cvc::app &Runtime::app() const { return m_->app; }

void Runtime::Impl::load_names() const {
  namespace se = cvc::state_exec;
  const std::uint64_t generation = action_intrinsics_generation(app);
  if (names_valid && names_generation == generation)
    return;
  std::set<std::string> found;
  const NotifyScope notify = notify_scope(prefix);
  probe_env([&](const se::environment_ptr &env, se::intrinsics_context &ictx) {
    // As the document's programs get them: by kind in an `any` group, the host verbs too.
    install_providers(app, env, ictx, group_.get());
    bind_own_intrinsics(env, ictx, group_.get());
    install_host_verbs(env, group_.get(), *ictx.host, notify);
    for (const se::environment *e = env.get(); e; e = e->outer.get())
      for (const auto &kv : e->bindings)
        found.insert(kv.first);
  });
  found.insert("has?"); // bound per program (bind_has), not by a provider
  names = std::move(found);
  names_valid = true;
  names_generation = generation;
}

bool Runtime::has_function(const std::string &name) const { return m_->has_function(name); }

std::vector<std::string> Runtime::program_names() const {
  std::lock_guard<std::mutex> lock(m_->names_mutex);
  m_->load_names();
  return {m_->names.begin(), m_->names.end()};
}

void Runtime::set_scene_capable(bool on) { m_->scene_capable = on; }
void Runtime::set_scene_shaders(bool on) { m_->scene_shaders = on; }
void Runtime::set_relay_document(bool on) { m_->relay_document = on; }

HostFacts Runtime::host_facts() const { return m_->facts(); }

bool Runtime::has(const std::string &name) const { return m_->has(name); }

void Runtime::refresh_program_names() {
  std::lock_guard<std::mutex> lock(m_->names_mutex);
  m_->names_valid = false;
  // A host verb or function registered since the placement may need the host thread (s-m4): the
  // placement is decided again before the next program starts if that kills nothing, else at the
  // document's next restart (R1b skeptic m3).
  m_->registered_since_placement_ = true;
}

void Runtime::add_program_intrinsics(ActionIntrinsicProvider provider) {
  add_program_intrinsics(std::move(provider), cvc::state_exec::provider_affinity::host_only);
}

void Runtime::add_program_intrinsics(ActionIntrinsicProvider provider,
                                     cvc::state_exec::provider_affinity affinity) {
  if (!provider)
    return;
  {
    std::lock_guard<std::mutex> lock(m_->own_mutex);
    m_->own_intrinsics.push_back(registered_provider{std::string(), std::move(provider), affinity});
  }
  std::lock_guard<std::mutex> lock(m_->names_mutex);
  m_->names_valid = false;
  // As a host verb registered late (refresh_program_names): it may need the host thread, so the
  // placement is decided again before the next program starts if that kills nothing, else at the
  // document's next restart.
  m_->registered_since_placement_ = true;
}

ActionIntrinsicProvider Runtime::program_intrinsics() const {
  namespace se = cvc::state_exec;
  const Impl *impl = m_.get();
  // Every kind as it is: run_init builds its env, and its children's, on its own thread.
  return [impl](std::shared_ptr<se::environment> env, se::intrinsics_context &ictx) {
    impl->bind_own_intrinsics(env, ictx, nullptr);
  };
}

void Runtime::register_read_function(const std::string &name, ReadFunction fn) {
  if (m_->reactive)
    m_->reactive->bind_read_function(name, fn);
  m_->read_functions[name] = std::move(fn);
}

void Runtime::set_backend(Backend *backend) {
  m_->backend = backend;
  *m_->backend_ref = backend;
  std::lock_guard<std::mutex> lock(m_->names_mutex);
  m_->names_valid = false; // the host verbs it answers (bind_backend_verbs) are its own
}

bool Runtime::media_busy() const {
  if (m_->images && m_->images->busy())
    return true;
  for (const auto &kv : m_->players) // §5.13 a player still fetching or opening its source
    if (kv.second.player && kv.second.player->state() == media::PlayerState::Loading)
      return true;
  return false;
}
bool Runtime::media_stale() const { return m_->drew_loading || media_busy(); }

void Runtime::set_media_base(std::string dir) {
  m_->media_base = std::move(dir);
  if (m_->images)
    m_->images->set_base(m_->media_base);
}

void Runtime::set_save_dir(std::string dir) { m_->saves->dir = std::move(dir); }

void Runtime::set_root(Widget root) {
  // ARIADNE_SHELL.md 4.2 the document's own first mounts: slot (none inside a mount), whose key
  // (app-open) writes: known before the first frame, so a resident started first finds it.
  {
    std::string key;
    std::vector<const Widget *> todo{&root}; // in tree order, on the heap: any depth
    while (!todo.empty()) {
      const Widget &w = *todo.back();
      todo.pop_back();
      if (!w.scope.empty() || !w.repeat.empty())
        continue;
      if (w.kind == Kind::Mounts && !w.bind.empty()) {
        key = resolve_bind(m_->prefix, w.bind);
        break;
      }
      for (auto c = w.children.rbegin(); c != w.children.rend(); ++c)
        todo.push_back(&*c);
    }
    m_->doc_slot_->set(key);
  }
  // Now, not at the swap: a resident submitted in the next drain() runs with these.
  m_->doc_functions = root.functions ? root.functions->prelude : std::string();
  m_->pending = std::move(root);
  m_->has_pending = true;
  m_->document_set_ = true;       // a document to place (ensure_placement)
  m_->placement_decided_ = false; // a new one is placed again, before it runs or is drawn
}

void Runtime::on(std::string event, std::function<void()> handler) {
  m_->handlers[std::move(event)] = std::move(handler);
}

void Runtime::set_main_view_pick(ContextPick pick) { m_->main_view_pick = std::move(pick); }

// A lane's program and the key bindings are read by the isolation rules (S1-Q6), and the channel
// policy was until S2-Q16: setting one decides the placement again before the next program starts
// (s-m2). Host thread, outside a pump (they kill residents).
void Runtime::set_tick_program(std::string script) {
  assert_outside_pump(m_->app, m_->group_.get());
  // Replacing the program kills any running resident so the new body starts fresh next drain.
  m_->stop_resident(m_->tick_resident_);
  m_->tick_script_ = std::move(script);
  m_->placement_decided_ = false;
}

void Runtime::set_key_program(std::string script) {
  assert_outside_pump(m_->app, m_->group_.get());
  m_->stop_resident(m_->key_resident_);
  m_->key_script_ = std::move(script);
  m_->placement_decided_ = false;
}

void Runtime::set_pointer_program(std::string script) {
  assert_outside_pump(m_->app, m_->group_.get());
  m_->stop_resident(m_->pointer_resident_);
  m_->pointer_script_ = std::move(script);
  m_->placement_decided_ = false;
}

void Runtime::set_signal_program(std::string script) {
  assert_outside_pump(m_->app, m_->group_.get());
  m_->stop_resident(m_->signal_resident_);
  m_->signal_script_ = std::move(script);
  m_->placement_decided_ = false;
}

bool Runtime::has_signal_program() const { return !m_->signal_script_.empty(); }

bool Runtime::post_signal(const std::string &signal, const std::string &source) {
  assert_outside_pump(m_->app, m_->group_.get()); // host thread, outside a pump (S1.10)
  const bool tapped = (m_->tap_kinds_.load() & tap_signal) != 0;
  bool resident = false;
  if (!m_->signal_script_.empty()) {
    // A stopped handler waiting out its restart delay is started now: the signal is for it.
    if (m_->signal_resident_.pid < 0)
      m_->signal_resident_.retry_at = 0;
    m_->ensure_resident(m_->signal_channel(), m_->signal_script_, m_->signal_resident_);
    resident = m_->signal_resident_.pid >= 0; // one that failed to start has warned
  }
  // ARIADNE_SHELL.md 4.6 without an on_signal: of its own (which passes signals on itself, with
  // app-signal), the document passes them on to its applications that hold delegate.signals: an
  // interrupt to the focused one, which takes it over, a request to end to every one.
  const bool taken = !resident && m_->signal_mounts(signal, source);
  if (!resident && !tapped)
    return taken; // no handler (or it failed to start): the host applies the signal's default
  namespace se = cvc::state_exec;
  std::vector<std::pair<std::string, se::value_t>> f;
  f.emplace_back("kind", se::value_t(std::string("signal")));
  f.emplace_back("signal", se::value_t(signal));
  f.emplace_back("source", se::value_t(source));
  if (tapped)
    m_->tap("signal", se::make_dict(f));
  if (resident)
    m_->current_sched().post_message(m_->signal_channel(), se::make_dict(std::move(f)));
  return true;
}

void Runtime::set_event_tap(unsigned kinds, EventTap tap) {
  m_->tap_ = std::move(tap);
  m_->tap_kinds_.store(m_->tap_ ? kinds : 0u);
  if (!m_->tap_kinds_.load()) {
    std::lock_guard<std::mutex> lock(m_->tapped_mutex_);
    m_->tapped_.clear();
  }
}

void Runtime::set_channel_policy(std::vector<std::string> declared, std::vector<std::string> global,
                                 bool strict, bool quiet) {
  // Installed on every action/resident ictx via the &channel_policy_ pointer set in submit_action /
  // submit_resident, so this applies from the next activation on. `strict` is the enforce switch;
  // warn/off pass strict=false (the load-time lint already surfaced those), leaving runtime
  // enforcement disabled. Empty lists + strict=false (the caller's default for an undeclared doc)
  // is fully permissive.
  m_->channel_policy_.declared = std::move(declared);
  m_->channel_policy_.global = std::move(global);
  m_->channel_policy_.enforce = strict;
  m_->channel_policy_.quiet = quiet;
  m_->placement_decided_ = false; // decided again (s-m2); the rules no longer read it (S2-Q16)
}

void Runtime::post_input(const InputEvent &ev) {
  // Submit the input residents on first use so the delivered event is consumed, not dropped, then
  // deliver it. The pump in drain() steps the resident; feed input BEFORE drain() for same-frame
  // delivery (see the header contract). Host thread, outside a pump (S1.10).
  assert_outside_pump(m_->app, m_->group_.get());
  // ARIADNE_SHELL.md 4.6 a mounted application that holds delegate.keys or delegate.pointer may
  // take it: the next drain routes it (route_mount_input), after the document's bindings.
  if (m_->mount_input_wanted_.load()) {
    std::lock_guard<std::mutex> lock(m_->mount_input_mutex_);
    if (m_->mount_input_.size() < 1024) // a host that feeds input and never drains
      m_->mount_input_.push_back(ev);
  }
  if (ev.ui)
    return; // the host's UI took it: never the document's own
  m_->ensure_input_residents();
  if (ev.kind == InputEvent::Kind::KeyDown && m_->have_key_bindings_.load()) {
    const bool capture = m_->key_capture_pending();
    {
      std::lock_guard<std::mutex> lock(m_->pressed_mutex_);
      m_->pressed_keys_.push_back(ev.key);
    }
    if (capture)
      return; // a new key for the Keys window, not a key press for the document
  }
  m_->deliver_input(ev);
}

bool Runtime::key_capture_pending() const { return m_->key_capture_pending(); }

void Runtime::post_pad_press(const std::string &input) {
  if (input.empty() || !m_->have_key_bindings_.load())
    return;
  std::lock_guard<std::mutex> lock(m_->pressed_mutex_);
  m_->pressed_pads_.push_back(input);
}

void Runtime::forget_layout() { m_->forget_layout_pending = true; }

void Runtime::set_key_bindings(std::vector<KeyBinding> bindings) {
  m_->key_bindings_ = std::move(bindings);
  m_->have_key_bindings_.store(!m_->key_bindings_.empty());
  m_->placement_decided_ = false; // their programs are scanned too (s-m2)
}

void Runtime::Impl::run_key_bindings() {
  std::vector<std::string> keys, pads;
  {
    std::lock_guard<std::mutex> lock(pressed_mutex_);
    keys.swap(pressed_keys_);
    pads.swap(pressed_pads_);
  }
  const std::string capture_path = resolve_bind(prefix, kKeyCapture);
  if (!key_capture_pending() && !read_or<std::string>(app, capture_path, "").empty())
    write<std::string>(app, capture_path, ""); // the Keys window closed while it waited
  const std::string pad_capture = kPadCapture;
  const auto waiting_pad = [&](std::string *id) {
    const std::string capture = read_or<std::string>(app, capture_path, "");
    if (!key_capture_pending() || capture.compare(0, pad_capture.size(), pad_capture) != 0)
      return false;
    *id = capture.substr(pad_capture.size());
    return true;
  };
  for (const std::string &key : keys) {
    if (key_capture_pending()) {
      const std::string capture = read_or<std::string>(app, capture_path, "");
      const std::string stored = stored_key_name(key);
      if (stored.empty())
        continue; // a key with no name can't be bound: still waiting
      std::string pad_id;
      if (waiting_pad(&pad_id)) {
        if (canonical_key(key) == "escape") // a gamepad Change takes no key, but Escape
          write<std::string>(app, capture_path, "");
        continue;
      }
      if (canonical_key(key) != "escape")
        for (const KeyBinding &b : key_bindings_)
          if (b.id == capture)
            write<std::string>(app, resolve_bind(prefix, key_binding_path(b)), stored);
      write<std::string>(app, capture_path, "");
      continue;
    }
    for (const KeyBinding &b : key_bindings_)
      if (!b.on.empty() &&
          key_matches(read_or<std::string>(app, resolve_bind(prefix, key_binding_path(b)), ""),
                      key)) {
        queued_events.push_back({b.on, prefix});
        queued_events.back().functions = doc_functions;
      }
  }
  // A gamepad input for the binding whose gamepad Change waits; one pressed otherwise is the
  // host's.
  for (const std::string &input : pads) {
    std::string pad_id;
    if (!waiting_pad(&pad_id))
      continue;
    for (const KeyBinding &b : key_bindings_)
      if (b.id == pad_id && !b.pad_path.empty())
        write<std::string>(app, resolve_bind(prefix, b.pad_path), input);
    write<std::string>(app, capture_path, "");
  }
  // What the Keys window shows for each binding, written only when it changes.
  const std::string waiting =
      key_capture_pending() ? read_or<std::string>(app, capture_path, "") : std::string();
  const auto show = [&](const std::string &path, const std::string &shown) {
    if (read_or<std::string>(app, path, std::string(1, '\x01')) != shown) // \x01: not there yet
      write<std::string>(app, path, shown);
  };
  for (const KeyBinding &b : key_bindings_) {
    show(resolve_bind(prefix, key_shown_path(b)),
         !waiting.empty() && b.id == waiting
             ? std::string(kKeyPrompt)
             : read_or<std::string>(app, resolve_bind(prefix, key_binding_path(b)), ""));
    if (!b.pad_path.empty())
      show(resolve_bind(prefix, pad_shown_path(b)),
           !waiting.empty() && waiting == pad_capture + b.id
               ? std::string(kPadPrompt)
               : read_or<std::string>(app, resolve_bind(prefix, b.pad_path), ""));
  }
}

void Runtime::render() { m_->render(); }

void Runtime::set_frame_clock(std::optional<double> seconds) { m_->pinned_clock = seconds; }
void Runtime::set_clock(bool on) { m_->clock_default = on; }
void Runtime::set_notifications(bool on) { m_->notify_default = on; }
void Runtime::set_wall_clock(std::function<std::chrono::system_clock::time_point()> now) {
  m_->wall_clock = std::move(now);
}
double Runtime::frame_clock() const { return m_->frame_t; }
std::uint64_t Runtime::reactive_skip_count() const { return m_->reactive_skips; }

std::uint64_t Runtime::reactive_eval_count() const {
  return m_->reactive ? m_->reactive->evaluations() : 0;
}

int Runtime::eval_count(const std::string &expr) {
  return m_->eval_count(expr); // outside a walk: the document's own scope
}

void Runtime::set_reactive_frame_budget(double seconds) {
  m_->reactive_budget = seconds > 0.0 ? seconds : 0.0;
}

void Runtime::run_program(std::string script, std::function<void(const std::string &)> done) {
  Impl::QueuedAction a;
  a.event = std::move(script);
  a.prefix = m_->prefix;
  a.done = std::move(done);
  a.functions = m_->doc_functions;
  m_->queued_events.push_back(std::move(a));
}

bool Runtime::run_program_in_mount(const std::string &as, std::string script,
                                   std::function<void(const std::string &)> done) {
  for (const auto &[sub, slot] : m_->mount_slots_) {
    if (slot.as != as || !slot.wrapper)
      continue;
    Impl::QueuedAction a;
    a.event = std::move(script);
    a.prefix = sub;
    a.done = std::move(done);
    if (slot.wrapper->functions)
      a.functions = slot.wrapper->functions->prelude;
    m_->queued_events.push_back(std::move(a));
    return true;
  }
  return false;
}

int Runtime::start_process(LaneProcess p) {
  namespace se = cvc::state_exec;
  // Host thread, from a step too: an exported script call and an on: hook start their activations
  // from the calling step, which in a document that uses an engine runs on the host thread (its
  // group 0 or host-affinity group, pumped by the frame's caller). So no assert_outside_pump.
  // Natives of its own are host code run inline (a scripting engine's activation): the document
  // stays on the host thread. Decided now when nothing of it runs (ensure_placement), else it is
  // refused until then, since an `any` group's worker would run them.
  if (p.extend) {
    require_host_thread("a host extension's process (" + p.name + ")");
    m_->ensure_placement(/*submitting=*/true);
    if (m_->group_ && m_->group_->opts().affinity == se::group_affinity::any)
      throw std::runtime_error("Runtime::start_process: '" + p.name +
                               "' binds natives of its own, which need the host thread, and the "
                               "document at '" +
                               m_->prefix +
                               "' runs in a group the pool's workers pump: it moves to the host "
                               "thread once nothing of it runs, or at its next load");
  }
  cvc::state &root = cvc::state::instance(m_->app);
  const bool action = p.lane != ProgramLane::resident; // an init process waits as an action does
  // A process's own cap only lowers its lane's; 0 is no cap.
  const auto lower = [](auto lane, auto own) {
    return lane == 0 ? own : own == 0 ? lane : std::min(lane, own);
  };
  std::shared_ptr<Impl::ActionContext> ac = m_->make_action_context(
      p.scope.empty() ? m_->prefix : p.scope,
      action ? std::optional<double>(action_lock_timeout(root)) : std::nullopt);
  ac->ictx.spawn_context = [this, notify = ac->notify,
                            held = ac->held](const se::intrinsics_context &parent) {
    return m_->spawn_child(parent, notify, held);
  };
  ac->ictx.on_child = [raw = ac.get()](int pid) { raw->children.push_back(pid); };
  ac->done = std::move(p.done); // without one, the sweep warns of a failure as it does an action's
  ac->script = p.name;
  se::async_scheduler &sched = m_->current_sched(); // as placed: its group's, else group 0's
  se::environment_ptr env = ac->env;
  if (!p.allow.empty()) {
    // Default-deny: a scope of its own with only the allowed names (no parent to reach the rest
    // through), as the read lane's evaluator is made.
    env = std::make_shared<se::environment>();
    for (const std::string &name : p.allow)
      if (const se::value_t *v = ac->env->lookup(name))
        env->set(name, *v);
  }
  if (p.extend) {
    env = se::environment::extend(env);
    p.extend(env, ac->ictx);
  }
  se::execute_options opts;
  opts.name = p.name;
  opts.env = env;
  opts.owner = m_->owner_; // reaped with the document, by ~Impl's kill_owner
  m_->own_scope(opts, *ac);
  opts.priority = p.priority;
  if (p.lane == ProgramLane::init) {
    // run_init's caps (a load-time seed or compute is generous, §7.4).
    opts.max_steps = lower(uint64_t{10'000'000}, p.max_steps);
    opts.max_time = lower(5.0, p.max_time);
    opts.max_memory = p.max_memory;
  } else if (action) {
    opts.max_steps = lower(action_max_steps(root), p.max_steps);
    opts.max_time = lower(action_max_time(root), p.max_time);
    opts.max_memory = lower(action_max_memory(root), p.max_memory);
  } else {
    opts.max_steps = p.max_steps;
    opts.max_time = p.max_time;
    opts.max_memory = p.max_memory;
    opts.max_step_time = resident_max_step_time(root);
  }
  // §6.6 in the document's own scope, its functions first, as an action has them.
  const bool own_scope = p.scope.empty() || p.scope == m_->prefix;
  const int pid = sched.execute(own_scope ? m_->doc_functions + p.program : p.program, opts);
  if (m_->sweeping_actions_) // from a done the sweep is calling: not into the map it walks
    m_->started_in_sweep_.emplace_back(pid, std::move(ac));
  else
    m_->live_actions_[pid] = std::move(ac); // swept, and its done told, as an action's is
  return pid;
}

void Runtime::pump() {
  // drain_end's pump and sweep, whatever exec_pump_frame did this frame: group 0 a bounded slice
  // (as scheduler_to_pump says) and the document's group through the app's pool, then the sweep
  // where its processes run. Host thread, outside a pump, which this would re-enter.
  assert_outside_pump(m_->app, m_->group_.get());
  m_->pump_frame();
  m_->sweep_document();
}

std::uint64_t exec_frame(cvc::app &app) {
  const std::shared_ptr<exec_pump_clock> clock = pump_clock_of(app);
  std::lock_guard<std::mutex> lock(clock->mutex);
  return clock->frames;
}

void Runtime::drain() {
  drain_begin();
  drain_end();
}

void Runtime::drain_begin() {
  Impl &m = *m_;
  m.drain_begun_ = true;
  // A frame pump of the app between here and drain_end() pumps this document's processes.
  m.begin_generation_ = m.app.exec_pump_generation();
  // Where its programs run is decided once a document is set, before any of this frame's start;
  // a group a move left goes once its kernels have ended.
  m.ensure_placement(/*submitting=*/false);
  if (!m.retired_groups_.empty())
    m.destroy_idle_retired_groups();
  ++m.drains_; // the residents' restart clock
  // A sign-in in progress (oidc_host.h) takes its step first: a reply, a browser's redirect, a
  // device poll that is due. It writes its status to the state tree on this thread.
  pump_oidc(m_->app);
  // Run the queued action events on the host thread, then clear. Never called
  // from inside render() (the deferred intent-buffer discipline, §7.2/§4.7). The keys pressed since
  // the last drain queue theirs first.
  m.run_key_bindings();
  // ARIADNE_SHELL.md 4.6 then the applications': (app-focus) and (app-signal) since, and the input
  // a mounted application holds the token for, each queued as an action in its mount.
  m.take_app_requests();
  m.route_mount_input();
  m.pump_saves(); // §5.29 the saves asked for since, and the steps of those under way
  if (m.images) // §5.11: a build without threads loads its pictures here, a step per drain
    m.images->pump();
  std::vector<Impl::QueuedAction> events;
  events.swap(m.queued_events);
  for (const Impl::QueuedAction &a : events) {
    // A program action (`on:` starting with '(', like a computed `bind:`/`tooltip:`) runs through
    // state_exec — the north-star lane: a flag toggle or reset is pure .ari, no C++ handler.
    if (a.done || is_expr(a.event)) { // a program on: (whitespace-tolerant, like a computed bind:)
      // SUBMIT it to the document's scheduler (owner-tagged, chrooted to the action's mount
      // prefix) and return — no run-to-completion inside the drain. The frame's pump advances it;
      // a quick action still finishes this frame, while an (await …)/(msg-recv …)/(sleep …) parks
      // and resumes on a later frame instead of blocking the UI thread (§4.7).
      m.submit_action(a);
      continue;
    }
    if (m.builtin_event(a)) // §5.24 the bell's own
      continue;
    auto it = m.handlers.find(a.event); // a bare event name -> the host C++ handler seam
    if (it != m.handlers.end() && it->second)
      it->second();
  }
  // §7.1 resident on:tick: submit the resident once (idempotent), then post it this frame's tick.
  m.ensure_tick_resident();
  m.post_tick();
  // §4.6 input residents (on_key/on_pointer): ensure they exist (idempotent; post_input also does,
  // but a host that set a program yet fed no input this frame still gets its resident submitted).
  m.ensure_input_residents();
  // The tap's events since the last drain, before the frame's pump: what they start runs in it.
  m.deliver_tapped();
  // has? in an `any` group reads what this thread sees now (S1-Q7).
  m.snapshot_has();
}

void Runtime::drain_end() {
  Impl &m = *m_;
  if (!m.drain_begun_)
    drain_begin(); // drain_end() alone is drain()
  m.drain_begun_ = false;
  // Advance submitted actions + the residents — and anything still-parked from earlier frames — a
  // bounded slice, unless the host's app.exec_pump_frame() did since drain_begin() (one pump a
  // frame for all its Runtimes, td 4.12). Group 0 is pumped every drain once this Runtime has used
  // it, and otherwise whenever any process is live on it: other components run processes there
  // too (a scene's transfer-function overlay tasks, cvc/tf/overlay_tasks.h), and only a pump
  // advances them, whatever host this is. An app nothing has used is not pumped (nor built).
  if (m.app.exec_pump_generation() == m.begin_generation_)
    m.pump_frame();
  // Then sweep the finished ones, where the document's processes run.
  m.sweep_document();
}

cvc::state_exec::exec_group *Runtime::isolated_group() const {
  m_->ensure_placement(/*submitting=*/false);
  return m_->group_.get();
}

std::string Runtime::isolation_reason() const {
  m_->ensure_placement(/*submitting=*/false);
  return m_->isolation_reason_;
}

void Runtime::declare_thread_affine_prefix(const std::string &prefix) {
  m_->thread_affine_.push_back(prefix);
  m_->placement_decided_ = false;
}

void Runtime::require_host_thread(const std::string &why) {
  if (!m_->host_thread_why_.empty())
    return;
  m_->host_thread_why_ = why.empty() ? std::string("a host extension") : why;
  // As a late registration (m3): decided again once nothing of the document runs, so a document
  // whose programs run is not moved under them.
  m_->registered_since_placement_ = true;
}

cvc::state_exec::exec_group *Runtime::current_group() const { return m_->group_.get(); }

std::size_t Runtime::retired_calls_in_flight() const {
  std::size_t n = 0;
  for (const Impl::RetiredGroup &r : m_->retired_groups_)
    if (r.group)
      n += r.group->host().in_flight();
  return n;
}

void Runtime::wait_retired_calls() {
  for (Impl::RetiredGroup &r : m_->retired_groups_)
    if (r.group)
      r.group->host().wait_idle();
}

void Runtime::set_document_programs(std::vector<std::string> programs) {
  m_->document_programs_ = std::move(programs);
  m_->placement_decided_ = false; // read when the document is placed again
}

std::vector<std::string> Runtime::take_reactive_warnings() {
  std::vector<std::string> out;
  out.swap(m_->reactive_warnings); // reactive_warned stays -> still deduped across drains
  return out;
}

// The same per-document scope submit_action/ensure_resident install on each lane as
// ictx.document — exposed so a scene-bound host can publish a routing handle for its verbs.
cvc::state_exec::document_scope &Runtime::document_scope() { return m_->doc_scope_; }

// ---- on-demand mounts (docs/roadmap/ARIADNE_SHELL.md 4.2) ----

void Runtime::set_mount_loader(MountLoader loader) { m_->mount_loader_ = std::move(loader); }

std::vector<Runtime::MountState> Runtime::mounts() const {
  std::vector<MountState> out;
  for (const auto &[sub, slot] : m_->mount_slots_) {
    MountState m;
    m.as = slot.as;
    m.scope = sub;
    m.uri = slot.uri;
    m.canonical = slot.canonical;
    m.principal = slot.principal;
    m.refused = slot.refused;
    m.mounted = slot.wrapper != nullptr;
    m.budget = slot.budget;
    if (slot.wrapper) {
      const MountGrants &g =
          slot.wrapper->grants ? *slot.wrapper->grants : m_->default_mount_grants_;
      m.grants.assign(g.held.begin(), g.held.end());
      m.net_origins = slot.net_origins;
      m.tests = slot.tests;
    }
    m.warnings = slot.warnings;
    if (slot.account) {
      m.steps = slot.account->last_steps.load(std::memory_order_relaxed);
      m.time = slot.account->last_seconds.load(std::memory_order_relaxed);
      m.memory = slot.account->memory.load(std::memory_order_relaxed);
      m.held = slot.account->last_held.load(std::memory_order_relaxed) ||
               slot.account->held.load(std::memory_order_relaxed);
      m.holds = slot.account->holds.load(std::memory_order_relaxed);
    }
    out.push_back(std::move(m));
  }
  return out;
}

std::vector<std::string> Runtime::mount_sources() const {
  std::vector<std::string> out;
  for (const auto &[sub, slot] : m_->mount_slots_)
    if (slot.wrapper)
      for (const std::string &f : slot.sources)
        if (std::find(out.begin(), out.end(), f) == out.end())
          out.push_back(f);
  return out;
}

bool Runtime::reload_mounts_if_changed() {
  bool any = false;
  for (auto &[sub, slot] : m_->mount_slots_)
    if (slot.wrapper && !slot.sources.empty() && sources_changed(slot.sources, slot.stamps)) {
      slot.stale = true; // parsed again by the next frame's reconcile
      any = true;
    }
  return any;
}

// A backend with no list draws the rows as stacked radio buttons (the first shown picked while
// none is) and its buttons under them.
ListEdit Backend::list(const char *label, const char *id, int current,
                       const std::vector<std::string> &items, const ListOptions &o) {
  (void)id;
  ListEdit e;
  if (!items.empty()) {
    const IndexEdit r = radio(label, current < 0 ? 0 : current, items, false);
    if (r.changed && r.index >= 0) {
      e.selected = true;
      e.index = r.index;
    }
  } else if (label && *label) {
    text_line(label);
  }
  if (o.add && button("Add"))
    e.add = true;
  if (o.remove && current >= 0 && button("Remove"))
    e.remove = true;
  return e;
}

} // namespace ariadne
} // namespace cvc
