#ifndef CVC_ARIADNE_WIDGET_H
#define CVC_ARIADNE_WIDGET_H

// Ariadne — the context-agnostic UI & scene DSL (cvc::ariadne, pure libcvc).
//
// widget.h is the retained widget-tree node model — the in-memory "DOM" the
// per-frame walk (ariadne.cpp) renders through a pluggable Backend (backend.h).
// It is deliberately backend-free (a plain data struct): no ImGui, no VTK, no
// terminal library — any consumer can build a tree without pulling a UI toolkit,
// and any Backend (ImGui-over-VTK, a terminal, …) can render the same tree.
//
// This is the P0 slice: a useful subset of widget kinds, built programmatically
// (see the builder helpers). The YAML loader (slice 1) will construct the same
// Widget tree from a .ari document; the state-tree layout, expressions, and the
// full widget vocabulary follow the roadmap (docs/roadmap/CVCGL-UI-DSL-ROADMAP.md).

#include <cvc/ariadne/value.h> // Widget::props for a Kind::Custom widget
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace ariadne {

// The widget kinds realized in the P0 walker. Structure (menubar/menu/window/
// group) plus the common state-bound leaves and fire-once actions.
enum class Kind {
  Group,          // a plain vertical container (the default)
  Menubar,        // the main menu bar (top strip)
  Menu,           // a menu inside the menubar
  MenuItemToggle, // a menu item bound to a bool state path (a tick)
  MenuItemAction, // a fire-once menu item -> raises `on` event
  Window,         // a floating window (title bar / move / resize / collapse)
  Text,           // static caption OR a read-only view of a state path
  Separator,      // a horizontal rule
  Progress,       // a read-only bar: the number at a state path, over [lo, hi]
  Checkbox,       // bound bool state
  SliderInt,      // bound int state, [ilo, ihi]
  SliderFloat,    // bound double state, [lo, hi]
  Combo,          // bound enum-as-text state
  InputText,      // bound string state (a single-line text field)
  InputInt,       // bound int state (a single-line numeric field)
  Color,          // bound "r,g,b" state (an RGB picker; §G7)
  Image,          // a host-published image viewer (a live raster); src / bound name
  Button,         // a fire-once button -> raises `on` event
  TfEditor,       // a transfer-function editor bound to a cvc::tf::model key (§10; props = config)
  Custom,         // a registered custom widget type (custom_type + props); §extensibility
  WindowsMenu,    // the built-in Windows menu (reopen a closed window): placed here in a menubar,
                  // or, outside one, the document's setting for it (bdef false: no such menu)
};

// ---------------------------------------------------------------------------
// Sizing & layout (roadmap §3.0.3 / §3.0.3b). A size or track value is one of:
//   auto     — fit content
//   px       — an absolute pixel count
//   percent  — a fraction of the parent's content rect (a "50%" string, or a
//              0..1 fraction the loader multiplies to 0..100)
// ---------------------------------------------------------------------------
enum class Unit { Auto, Px, Percent };

// A size value along one axis. `value` is pixels (Px) or 0..100 (Percent);
// unused for Auto.
struct Extent {
  Unit unit = Unit::Auto;
  float value = 0.0f;
  bool is_set() const { return unit != Unit::Auto; }
};

// A widget's sizing within its parent's layout (§3.0.3): a hint plus optional
// min/max floors/ceilings, per axis. All-Auto = let the backend/toolkit size it.
struct Size {
  Extent w, h;         // preferred (hint) size
  Extent min_w, min_h; // floor (Auto = none)
  Extent max_w, max_h; // ceiling (Auto = none)
  bool any() const {
    return w.is_set() || h.is_set() || min_w.is_set() || min_h.is_set() || max_w.is_set() ||
           max_h.is_set();
  }
};

// One layout track — a column width (or, later, a row height): Px → fixed,
// Percent → stretch weight, Auto → fit content (§3.0.3b).
struct Track {
  Unit unit = Unit::Auto;
  float value = 0.0f;
};

enum class LayoutKind { Vertical, Horizontal, Grid };
// `Off`, not `None`: X11 <X11/X.h> defines `None` as a macro, which would break this
// declaration in any translation unit that includes an X11 header first.
enum class BorderShow { Off, Inner, Outer, All };

// A container's layout (§3.0.2 / §3.0.3b). `col_widths` sizes the tracks of a
// Grid/Horizontal layout; `resizable` lets the user drag the column seams (native
// for columns; default off); `borders` shows the inter-cell seams.
struct Layout {
  LayoutKind kind = LayoutKind::Vertical;
  std::vector<Track> col_widths;  // Grid/Horizontal column tracks
  std::vector<Track> row_heights; // Grid row tracks (§3.0.3b increment 2)
  bool resizable = false;
  BorderShow borders = BorderShow::Off;
  bool has_border_color = false;
  float border_color[4] = {0.3f, 0.3f, 0.35f, 1.0f};
  bool is_set() const {
    return kind != LayoutKind::Vertical || !col_widths.empty() || !row_heights.empty() ||
           resizable || borders != BorderShow::Off;
  }
  // A resizable grid with sized rows is realized as a vertical split-pane stack
  // with draggable seams (§3.0.3b: "rows are a manual splitter"); anything else
  // is a table.
  bool is_row_split() const { return resizable && !row_heights.empty(); }
};

// §12 load: one parent-scope "hole" a mount grants its module (link:). The Runtime plants a
// transparent link node at `<mount-sub-prefix>.<name>`, so the module — otherwise chrooted to
// its own subtree — reads/writes exactly this `target` through it. `target` is resolved against
// the mount's PARENT scope (leading '/' = app root). `writable` false = a read-only hole (reads
// see the target; a write stays on the hole's own local value, never scribbling the target).
struct LinkHole {
  std::string name; // segment the module sees inside its chroot
  std::string
      target; // granted path (parent-scope-relative, or /-absolute), resolved by the Runtime
  bool writable = true; // false -> read-only (setLinkWritable(false))
};

// A function a document declares in its `functions:` block (docs/ARIADNE.md §6.6): its name and
// its whole (defun name (params) body...) source.
struct FunctionDef {
  std::string name;
  std::string source;
};

// The functions every program in one scope can call: at the document's root, its own and the ones
// it imported; at a mount, the ones the mount granted and the module's own. `prelude` is their
// sources, one a line, which the Runtime runs ahead of each program in the scope (so each program
// defines them in its own process: code is shared, never a closure or a state grant).
struct FunctionScope {
  std::vector<FunctionDef> defs;
  std::string prelude;
};

// A scope over `defs` (prelude built from their sources).
inline std::shared_ptr<const FunctionScope> make_function_scope(std::vector<FunctionDef> defs) {
  auto scope = std::make_shared<FunctionScope>();
  for (const FunctionDef &d : defs)
    scope->prelude += d.source + "\n";
  scope->defs = std::move(defs);
  return scope;
}

// One node of the retained tree. `bind` is a cvc::state path (relative to the
// Runtime's prefix, or absolute with a leading '/'); `on` is an event name a
// host handler is registered for (Runtime::on). Empty fields are simply unused
// by the kinds that don't read them.
struct Widget {
  Kind kind = Kind::Group;

  std::string id;    // stable widget id suffix; defaults to `label`
  std::string label; // display text / window title / menu name
  std::string bind;  // cvc::state path for bound widgets
  std::string on;    // event name for action widgets (Button / MenuItemAction)
  // §4.6 widget-level pointer handlers: each a two-shape value (an event name OR a program starting
  // with '(') fired for THIS widget — enqueued in the walk when the backend reports the matching
  // per-item pointer state, like Button's `on`. Any widget kind may carry any of them.
  //   on_click — the item was clicked this frame (Backend::item_clicked / ImGui::IsItemClicked).
  //              A one-shot edge: fires once per click.
  //   on_hover — the item is hovered (Backend::item_hovered / ImGui::IsItemHovered). CONTINUOUS:
  //              fires EVERY frame the pointer is over the item, so its handler should be
  //              idempotent/cheap (typically a state-set), like an on:tick body.
  //   on_drag  — the item is being actively dragged (Backend::item_dragged). CONTINUOUS: fires
  //              every frame the drag is active.
  //   on_drag_start — the item just became active this frame (Backend::item_drag_started /
  //              ImGui::IsItemActivated). A one-shot EDGE at the start of a press/drag gesture.
  //   on_drag_end — the item just stopped being active this frame (Backend::item_drag_ended /
  //              ImGui::IsItemDeactivated). A one-shot EDGE at the end of the gesture (release).
  // §4.6 event scope: when the handler is a PROGRAM, it runs with an `event` dict bound (read with
  // (get-attr event "x")): x/y are the pointer position in the item's own content-rect-local
  // pixels, dx/dy the drag delta this gesture, button the 1-based button. (A bare-name handler
  // routes to a host C++ handler and carries no payload.) The document-level on_pointer resident
  // remains the way to see the raw pointer stream; these give a widget handler its own local
  // coordinates. Disabled widgets fire none of these (the backend reports false while disabled).
  std::string on_click;
  std::string on_hover;
  std::string on_drag;
  std::string on_drag_start;
  std::string on_drag_end;

  // §4 read-lane (reactive): a state_exec predicate re-evaluated each frame; when it
  // is non-empty and evaluates falsy the widget (and its subtree) is skipped this
  // frame. Read-only, prefix-scoped, step/time-capped, fail-safe (a broken predicate
  // HIDES the widget and warns once — see Runtime). Empty = always visible.
  std::string visible_when;

  // §4 read-lane (reactive): predicates that grey a widget (and its subtree) out —
  // still drawn, but non-interactive. `enabled_when` disables when it is falsy;
  // `disabled_when` disables when it is truthy (both may be set). Fail-safe DISABLES on a
  // broken predicate (§4.1). Empty = always enabled. Realized via the backend's
  // begin_disabled()/end_disabled() scope.
  std::string enabled_when;
  std::string disabled_when;

  // §4 read-lane (reactive): a Combo's options as a computed expression (a list) evaluated
  // each frame, instead of the static `options` list. Empty = use the static list. Fail-safe
  // to an empty list on a broken expr.
  std::string options_expr;

  // §4 read-lane: a hover tooltip. A literal string, OR (if it starts with '(') a computed
  // expression re-evaluated each frame. Empty = no tooltip.
  std::string tooltip;

  // §3/§4 dynamic repetition: when non-empty, this widget is a TEMPLATE emitted `repeat`
  // times, where `repeat` is a count expression re-evaluated each frame (capped, fail-safe
  // 0). Each instance gets its own backend id and has the `{i}` token (the 0-based index)
  // substituted into its bind / on / label / predicate / tooltip strings, so instances can
  // address distinct state (e.g. bind: items.{i}.on). Empty = emitted once.
  std::string repeat;

  // numeric widget params
  double lo = 0.0, hi = 1.0, def = 0.0; // SliderFloat
  int ilo = 0, ihi = 100, idef = 0;     // SliderInt
  bool bdef = false;                    // Checkbox / MenuItemToggle default
  std::string fmt = "%.3f";             // SliderFloat printf format
  // A slider writes its key on every change while it is dragged, so what it drives follows the
  // drag. `commit: release` (true here) writes once, when the drag ends: for a key whose change is
  // too costly to redo every frame.
  bool commit_on_release = false;
  std::string sdef; // Combo default (an option label, or — with `values` — a value)
  std::vector<std::string> options; // Combo options (the DISPLAYED labels)
  // §4 Combo value mapping: a parallel list the same length as `options`, so the bound key holds
  // `values[i]` (e.g. an int) while the combo shows `options[i]`. Empty = the key holds the option
  // TEXT (the default). Ignored when `options_expr` computes the options (no parallel values).
  std::vector<std::string> values;

  // Window initial placement (a backend hint; seeded first-use, user drag wins).
  bool has_pos = false;
  float pos_x = 0.f, pos_y = 0.f;

  Size size;                  // §3.0.3 / §3.0.3b: hint/min/max, px|percent|auto per axis
  Layout layout;              // §3.0.2 / §3.0.3b: container layout + tracks + borders
  float frame_border = -1.0f; // §3.0.3b: widget/window border width in px; <0 = backend default

  // Window `closable: true` — give the window a close (X) button; its open state persists at
  // tree.<id>.open (default open). Closed → not rendered; a menu_toggle bound to the same key (e.g.
  // a "Settings" menu) re-opens it. Default false = a window with no X (the historical behaviour).
  bool closable = false;

  bool literal_text = false; // Text: `label` is a literal caption, not a path

  // §G? Text COLOUR (severity styling). A Kind::Text (literal OR bound) may carry a fixed RGB
  // tint; the walker then emits the *_colored backend leaf instead of the plain one. Backends
  // that cannot colour text fall back to the uncoloured draw (Backend default impls), so this is
  // purely additive — a widget with has_text_color=false is byte-identical to before. Kept
  // deliberately general (any Text, not a demo-specific status widget) so every HUD can reuse it.
  bool has_text_color = false;
  float text_color[3] = {1.0f, 1.0f, 1.0f};

  // Kind::Image (§ raster viewer): the host-published image to show. `src` is a static image name;
  // a non-empty `bind` overrides it with the name held in that state key (so a combo can switch
  // which raster is shown). `img_size` is the display width in px (height follows the image's
  // aspect).
  std::string src;
  double img_size = 256.0;

  // Kind::Progress: the bar is the number at `bind` placed in [lo, hi] (clamped); `overlay` is a
  // state path (or a computed expression) whose text is drawn over the bar, else the percent.
  std::string overlay;

  // §12 module mount (load:): when non-empty, this widget is a MOUNT POINT — a relative
  // state-path segment (e.g. "includes.stats") appended to the enclosing scope, so its whole
  // SUBTREE resolves binds and evaluates reactive/read-lane expressions against that deeper
  // sub-prefix (its own chroot), while the mount widget's own reactive fields stay in the
  // parent scope. Empty for ordinary widgets. This is how a loaded fragment is isolated
  // (roadmap §12.1: ui.docs.<doc>.includes.<as>).
  std::string scope;

  // §12 load: parent-scope "holes" (link:) — the Runtime plants a transparent link node inside
  // the mount's sub-prefix for each, so the otherwise-sandboxed module can reach exactly the
  // granted parent/shared scopes and nothing else (§7.8.3). Only set on a mount wrapper.
  std::vector<LinkHole> links;

  // §12 load: the mounted fragment's own init: script (a state_exec program), run ONCE at the
  // mount's sub-prefix when it first appears — after its holes are wired, so init may seed through
  // them. A seed, not a per-frame action: it does not re-run on a reconcile. Only on a mount
  // wrapper; empty otherwise.
  std::string init_script;

  // §12 load: the mounted fragment's own on_tick: body. The Runtime runs it as one resident per
  // mount, chrooted to the sub-prefix with the mount's functions, woken once per drain like the
  // document's: from the mount's first frame (after its init:) until a reconcile drops it. Only on
  // a mount wrapper; empty otherwise.
  std::string tick_script;

  // §6.6 functions: set on the document's root and on every mount wrapper, null elsewhere. The
  // programs started inside this subtree (on: actions, the mount's init:) run with these
  // functions defined, and no others: a mount gets an empty scope unless it grants some.
  std::shared_ptr<const FunctionScope> functions;

  // Kind::Custom: the registered type name, and the config bag (every widget key the
  // loader did not consume) a registered emit fn (register_widget_type) reads. Empty
  // for the built-in kinds.
  std::string custom_type;
  Value props;

  std::vector<Widget> children;
};

// ---------------------------------------------------------------------------
// Builder helpers — ergonomic programmatic tree construction for the P0 slice
// and for tests, until the YAML loader lands. Each returns a Widget by value.
// ---------------------------------------------------------------------------

inline Widget menubar(std::vector<Widget> kids) {
  Widget w;
  w.kind = Kind::Menubar;
  w.children = std::move(kids);
  return w;
}

inline Widget menu(std::string name, std::vector<Widget> kids) {
  Widget w;
  w.kind = Kind::Menu;
  w.label = std::move(name);
  w.children = std::move(kids);
  return w;
}

inline Widget menu_toggle(std::string label, std::string bind, bool def = false) {
  Widget w;
  w.kind = Kind::MenuItemToggle;
  w.label = std::move(label);
  w.bind = std::move(bind);
  w.bdef = def;
  return w;
}

inline Widget menu_action(std::string label, std::string on) {
  Widget w;
  w.kind = Kind::MenuItemAction;
  w.label = std::move(label);
  w.on = std::move(on);
  return w;
}

// The built-in Windows menu (ARIADNE.md §5.9). In a menubar (or a menu), it is drawn there under
// `title` ("" = "Windows") instead of at the end of the bar. Anywhere else it only sets the menu
// up: `on` false turns it off for the whole document, and a `title` renames it.
inline Widget windows_menu(std::string title = std::string(), bool on = true) {
  Widget w;
  w.kind = Kind::WindowsMenu;
  w.label = std::move(title);
  w.bdef = on;
  return w;
}

inline Widget window(std::string title, std::vector<Widget> kids) {
  Widget w;
  w.kind = Kind::Window;
  w.label = std::move(title);
  w.children = std::move(kids);
  return w;
}

inline Widget group(std::vector<Widget> kids) {
  Widget w;
  w.kind = Kind::Group;
  w.children = std::move(kids);
  return w;
}

inline Widget text(std::string caption) {
  Widget w;
  w.kind = Kind::Text;
  w.label = std::move(caption);
  w.literal_text = true;
  return w;
}

inline Widget text_bound(std::string label, std::string bind) {
  Widget w;
  w.kind = Kind::Text;
  w.label = std::move(label);
  w.bind = std::move(bind);
  return w;
}

// Tint ANY Text widget (literal or bound) a fixed RGB — the general severity-colour mutator, e.g.
// `with_text_color(text_bound("Min clearance", "stats.min_clearance"), 0.9f, 0.2f, 0.2f)`.
// Chainable; a no-op-safe pass-through for non-Text widgets (the walker only reads text_color on
// Kind::Text).
inline Widget with_text_color(Widget w, float r, float g, float b) {
  w.has_text_color = true;
  w.text_color[0] = r;
  w.text_color[1] = g;
  w.text_color[2] = b;
  return w;
}

inline Widget separator() {
  Widget w;
  w.kind = Kind::Separator;
  return w;
}

inline Widget progress(std::string label, std::string bind, double lo = 0.0, double hi = 1.0,
                       std::string overlay = "") {
  Widget w;
  w.kind = Kind::Progress;
  w.label = std::move(label);
  w.bind = std::move(bind);
  w.lo = lo;
  w.hi = hi;
  w.overlay = std::move(overlay);
  return w;
}

inline Widget checkbox(std::string label, std::string bind, bool def = false) {
  Widget w;
  w.kind = Kind::Checkbox;
  w.label = std::move(label);
  w.bind = std::move(bind);
  w.bdef = def;
  return w;
}

inline Widget slider_int(std::string label, std::string bind, int lo, int hi, int def = 0) {
  Widget w;
  w.kind = Kind::SliderInt;
  w.label = std::move(label);
  w.bind = std::move(bind);
  w.ilo = lo;
  w.ihi = hi;
  w.idef = def;
  return w;
}

inline Widget slider_float(std::string label, std::string bind, double lo, double hi,
                           double def = 0.0, std::string fmt = "%.3f") {
  Widget w;
  w.kind = Kind::SliderFloat;
  w.label = std::move(label);
  w.bind = std::move(bind);
  w.lo = lo;
  w.hi = hi;
  w.def = def;
  w.fmt = std::move(fmt);
  return w;
}

inline Widget combo(std::string label, std::string bind, std::vector<std::string> options,
                    std::string def = std::string()) {
  Widget w;
  w.kind = Kind::Combo;
  w.label = std::move(label);
  w.bind = std::move(bind);
  w.options = std::move(options);
  w.sdef = std::move(def);
  return w;
}

// A single-line text field bound to a string state key (chat input, display name, room id). `on`
// (optional) is an event raised when the edit commits (Enter / focus-leave) — e.g. send-on-enter
// for a chat box. `def` seeds the key when it is unset.
inline Widget input_text(std::string label, std::string bind, std::string on = std::string(),
                         std::string def = std::string()) {
  Widget w;
  w.kind = Kind::InputText;
  w.label = std::move(label);
  w.bind = std::move(bind);
  w.on = std::move(on);
  w.sdef = std::move(def);
  return w;
}

// A single-line numeric field bound to an int state key. Reuses the text primitive; the core parses
// the field to an int on commit (fail-safe: a non-numeric entry keeps the current value) and clamps
// to [lo, hi] when they differ (lo == hi = no clamp). `def` seeds the key when it is unset.
inline Widget input_int(std::string label, std::string bind, int lo = 0, int hi = 0, int def = 0,
                        std::string on = std::string()) {
  Widget w;
  w.kind = Kind::InputInt;
  w.label = std::move(label);
  w.bind = std::move(bind);
  w.ilo = lo;
  w.ihi = hi;
  w.idef = def;
  w.on = std::move(on);
  return w;
}

inline Widget button(std::string label, std::string on) {
  Widget w;
  w.kind = Kind::Button;
  w.label = std::move(label);
  w.on = std::move(on);
  return w;
}

// An RGB colour picker bound to an "r,g,b" state key (each channel 0..1). `def` seeds the key when
// unset (e.g. "1,1,1"). §G7.
inline Widget color_widget(std::string label, std::string bind, std::string def = std::string()) {
  Widget w;
  w.kind = Kind::Color;
  w.label = std::move(label);
  w.bind = std::move(bind);
  w.sdef = std::move(def);
  return w;
}

// A host-published image viewer (a live raster). `src` = the image name; a non-empty `bind` names a
// state key whose value selects the image instead. `size` = display width in px.
inline Widget image_widget(std::string label, std::string src, double size = 256.0) {
  Widget w;
  w.kind = Kind::Image;
  w.label = std::move(label);
  w.src = std::move(src);
  w.img_size = size;
  return w;
}

// A transfer-function editor (§10) bound to a cvc::tf::model state key — the ColorTable2 port.
// Overlay layers are read from keys beside `bind` (bind.histogram, bind.contour_spectrum, ...);
// `props` carries the optional config (interactive_updates, height, layers, preset, ...; see
// docs/TRANSFER_FUNCTIONS.md).
inline Widget tf_editor(std::string label, std::string bind) {
  Widget w;
  w.kind = Kind::TfEditor;
  w.label = std::move(label);
  w.bind = std::move(bind);
  w.props.kind = Value::Kind::Map;
  return w;
}

// A custom widget (§ extensibility) — rendered by the emit fn registered for `type`
// via register_widget_type (or a labelled placeholder if none). Set `props` for extra
// config; `label`/`bind` are the common ones.
inline Widget custom(std::string type, std::string label = std::string(),
                     std::string bind = std::string()) {
  Widget w;
  w.kind = Kind::Custom;
  w.custom_type = std::move(type);
  w.label = std::move(label);
  w.bind = std::move(bind);
  return w;
}

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_WIDGET_H
