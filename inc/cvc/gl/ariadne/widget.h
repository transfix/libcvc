#ifndef CVC_GL_ARIADNE_WIDGET_H
#define CVC_GL_ARIADNE_WIDGET_H

// Ariadne — the cvcGL UI & scene DSL (cvc::gl::ariadne).
//
// widget.h is the retained widget-tree node model — the in-memory "DOM" the
// per-frame walk (ariadne.cpp) renders as Dear ImGui. It is deliberately
// imgui-free (a plain data struct), so any consumer can build a tree without
// pulling in ImGui headers; the walker emits the actual ImGui calls.
//
// This is the P0 slice: a useful subset of widget kinds, built programmatically
// (see the builder helpers). The YAML loader (slice 1) will construct the same
// Widget tree from a .ari document; the state-tree layout, expressions, and the
// full widget vocabulary follow the roadmap (docs/roadmap/CVCGL-UI-DSL-ROADMAP.md).

#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace gl {
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
  Checkbox,       // bound bool state
  SliderInt,      // bound int state, [ilo, ihi]
  SliderFloat,    // bound double state, [lo, hi]
  Combo,          // bound enum-as-text state
  Button,         // a fire-once button -> raises `on` event
};

// One node of the retained tree. `bind` is a cvc::state path (relative to the
// Runtime's prefix, or absolute with a leading '/'); `on` is an event name a
// host handler is registered for (Runtime::on). Empty fields are simply unused
// by the kinds that don't read them.
struct Widget {
  Kind kind = Kind::Group;

  std::string id;    // stable ImGui id suffix (### ); defaults to `label`
  std::string label; // display text / window title / menu name
  std::string bind;  // cvc::state path for bound widgets
  std::string on;    // event name for action widgets (Button / MenuItemAction)

  // numeric widget params
  double lo = 0.0, hi = 1.0, def = 0.0; // SliderFloat
  int ilo = 0, ihi = 100, idef = 0;     // SliderInt
  bool bdef = false;                    // Checkbox / MenuItemToggle default
  std::string fmt = "%.3f";             // SliderFloat printf format
  std::string sdef;                     // Combo default option text
  std::vector<std::string> options;     // Combo options

  // Window initial placement (seeded ImGuiCond_FirstUseEver; user drag wins).
  bool has_pos = false;
  float pos_x = 0.f, pos_y = 0.f;
  bool has_size = false;
  float size_w = 0.f, size_h = 0.f;

  bool literal_text = false; // Text: `label` is a literal caption, not a path

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

inline Widget separator() {
  Widget w;
  w.kind = Kind::Separator;
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

inline Widget slider_float(std::string label, std::string bind, double lo, double hi, double def = 0.0,
                           std::string fmt = "%.3f") {
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

inline Widget button(std::string label, std::string on) {
  Widget w;
  w.kind = Kind::Button;
  w.label = std::move(label);
  w.on = std::move(on);
  return w;
}

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_WIDGET_H
