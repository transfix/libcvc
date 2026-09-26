#ifndef CVC_ARIADNE_BACKEND_H
#define CVC_ARIADNE_BACKEND_H

// Ariadne — the pluggable UI backend (the "arbitrary UI handler"), §16 of the
// roadmap. The Backend is what actually renders a Widget tree into a concrete
// surface: cvc::gl::ImGuiBackend draws ImGui over VTK (the reference backend);
// a future terminal backend (FTXUI / tvision) draws the SAME tree into a screen.
//
// The split (roadmap §16.1): the CORE (Runtime, ariadne.cpp) owns the retained
// tree, the reconcile boundary, and ALL cvc::state binding — it reads a bound
// path, hands the current value to a Backend primitive to draw, and writes the
// value back only when the primitive reports it was *committed*. The Backend
// owns only the surface: it draws value-in / edit-out and NEVER touches
// cvc::state. That keeps the core pure libcvc (no VTK/ImGui/terminal), and keeps
// every commit-policy decision (immediate vs on-release) reported, not hidden.
//
// Each state-bound primitive returns an Edit result:
//   changed   — the value moved this frame (per-frame; e.g. mid-drag)
//   committed — the edit is final and should be written to state now
//               (discrete widgets: == changed; sliders: on release only)
//   value     — the current widget value
// The core writes to cvc::state on `committed`, so a slider drag costs ONE state
// write (which fans out to observers / replicated peers), not one per frame.

#include <cvc/ariadne/widget.h> // Size / Layout / Extent (§3.0.3b) passed to the backend
#include <string>
#include <vector>

namespace cvc {
namespace ariadne {

struct BoolEdit {
  bool changed = false;
  bool committed = false;
  bool value = false;
};
struct IntEdit {
  bool changed = false;
  bool committed = false;
  int value = 0;
};
struct DoubleEdit {
  bool changed = false;
  bool committed = false;
  double value = 0.0;
};
// Combo carries an option INDEX (the core maps it back to the option text it
// stores in state). index < 0 means "no change / nothing selected".
struct IndexEdit {
  bool changed = false;
  bool committed = false;
  int index = -1;
};
// The result of a backend-drawn CUSTOM widget (a novel primitive the compositional
// path can't express — §16.1b). `handled` is the backend's signal that it actually
// drew this custom_type (false = it doesn't know it, so the core shows a placeholder).
// The value channel is a STRING (custom widgets bind arbitrary types via cvc::state's
// string channel, like combo stores the option text); `committed` is the write edge.
struct CustomEdit {
  bool handled = false;
  bool changed = false;
  bool committed = false;
  std::string value;
};

// What a surface can and cannot do. The core (and, later, the loader's
// `requires:` preflight, §16.3/§7.6) reads this to fail-safe or substitute —
// e.g. a shader widget needs `glsl`, `free` layout needs `windows`. `owns_loop`
// distinguishes a host-pumped backend (ImGui: the host render pass calls
// Runtime::render()) from a framework-owned loop (FTXUI/tvision: the backend's
// own loop drives it). This seam is locked in P0 even though only the ImGui
// backend exists yet, because it is the one contract expensive to retrofit.
struct Capabilities {
  bool windows = false;    // real movable child windows (free layout)
  bool menubar = false;    // a top menu strip
  bool mouse = false;      // pointer input
  bool keyboard = false;   // key input / text entry
  bool color = false;      // color styling
  bool glsl = false;       // can render GLSL (scene-node shaders / shader_canvas)
  bool view_embed = false; // can embed a live render-to-texture view
  bool owns_loop = false;  // the backend drives its own event loop
};

// The pluggable UI handler. The core walks the tree and calls these; structural
// Begin*/End* pair unconditionally (End* is always called, even when Begin*
// returned false), mirroring immediate-mode contracts. `key` on the sliders is
// an opaque, stable per-widget string (the core passes the resolved state path)
// the backend may use to hold an in-progress edit across frames.
class Backend {
public:
  virtual ~Backend() = default;

  virtual Capabilities capabilities() const = 0;

  // Frame brackets. A host-pumped backend (ImGui) may leave these empty (the
  // host already brackets the frame); a framework-owned backend uses them.
  virtual void begin_frame() {}
  virtual void end_frame() {}

  // ---- structure ----------------------------------------------------------
  virtual bool begin_main_menu_bar() = 0;
  virtual void end_main_menu_bar() = 0;
  virtual bool begin_menu(const char *label) = 0;
  virtual void end_menu() = 0;
  // A visible window. `id` is a stable identity (the backend keeps a window's
  // geometry/state keyed on it, independent of the visible title). `size` is the
  // §3.0.3b sizing spec (px | percent-of-parent | auto per axis, with min/max) —
  // the backend resolves any percent against its own surface; `border` is the
  // §3.0.3b window border width in px (<0 = the backend's default).
  virtual bool begin_window(const char *title, const char *id, const Size &size, float border) = 0;
  virtual void end_window() = 0;
  virtual void push_id(const char *id) = 0;
  virtual void pop_id() = 0;

  // Grid / column layout (§3.0.3b). A container whose children flow into sized
  // tracks: begin_grid opens it (false → clipped, do not emit or call end_grid),
  // grid_next_cell advances before each child, end_grid closes it. The backend
  // realizes `layout` (col_widths tracks, resizable seams, cell borders); a
  // backend that cannot lay out in columns may treat it as a plain vertical flow.
  virtual bool begin_grid(const Layout &layout, const char *id) = 0;
  virtual void grid_next_cell() = 0;
  virtual void end_grid() = 0;

  // ---- leaves -------------------------------------------------------------
  virtual void text_line(const char *text) = 0;                             // literal caption
  virtual void text_value(const char *label, const std::string &value) = 0; // "label: value"
  virtual void separator() = 0;
  virtual bool button(const char *label) = 0;           // returns clicked
  virtual bool menu_item_action(const char *label) = 0; // returns clicked

  // ---- state-bound (value-in / edit-out) ----------------------------------
  virtual BoolEdit menu_item_toggle(const char *label, bool current) = 0;
  virtual BoolEdit checkbox(const char *label, bool current) = 0;
  virtual IntEdit slider_int(const char *label, const char *key, int current, int lo, int hi) = 0;
  virtual DoubleEdit slider_double(const char *label, const char *key, double current, double lo,
                                   double hi, const char *fmt) = 0;
  virtual IndexEdit combo(const char *label, int current_index,
                          const std::vector<std::string> &options) = 0;

  // ---- custom widgets: the novel-primitive escape (§16.1b) ----------------
  // Draw a registered custom widget the compositional path can't express (a colour
  // wheel, a GLSL canvas, …). `type` is the widget's custom_type, `current` its bound
  // value as a string (empty when unbound), and `w` the full Widget (props/label). A
  // backend that knows `type` draws it and returns {handled:true, …}; the DEFAULT is a
  // no-op {handled:false}, so a backend need NOT implement this (it is not a forced
  // override) — the core then shows a placeholder. The core owns the state read/write;
  // the backend only draws value-in and reports the edit. Most custom widgets should
  // use the compositional register_widget_type instead — this is for genuinely new
  // primitives, and only a backend that supports them renders them.
  virtual CustomEdit custom_widget(const char *type, const std::string &current, const Widget &w) {
    (void)type;
    (void)current;
    (void)w;
    return {};
  }
};

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_BACKEND_H
