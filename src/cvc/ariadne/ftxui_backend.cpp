// FtxuiBackend — the pure-terminal Ariadne backend (see the header).
//
// The bridge: FTXUI is a functional/reactive DOM (you build an Element tree and
// render it), while the Ariadne Backend interface is immediate-mode (the core
// walk calls begin_*/checkbox/... every frame). So this accumulates an FTXUI
// Element tree DURING the walk, using a stack of child-lists: begin_frame opens
// the root list, each leaf appends an Element to the top list, and begin/end of
// a window or the menubar push/pop a list and wrap it (a window becomes an
// fx::window, the menubar an hbox). end_frame folds the root list into one vbox.
// render_to_string then rasterizes it — no TTY, no ImGui, no VTK.
//
// State-bound widgets render their CURRENT value (the core reads cvc::state and
// passes it in): a checkbox as [x]/[ ], a slider as a gauge + value, a combo as
// < option >. This P0 backend is display-focused (it reads state and shows it);
// interactive editing on the terminal is a follow-up (it needs FTXUI's Component
// layer wired to the commit protocol) and is why the state-bound methods return
// an empty (uncommitted) Edit for now.
//
// NB: fx:: is an alias for ftxui. It is NOT `using namespace ftxui`, on purpose:
// cvc::ariadne already has widget builders named text()/window()/separator()/
// checkbox()/... (widget.h), which would shadow FTXUI's same-named free
// functions inside this namespace. Every FTXUI call is qualified fx:: to pick
// the terminal DOM function, never the Widget builder.

#include <cvc/ariadne/ftxui_backend.h>

#include <cvc/ariadne/ariadne.h>

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#ifdef CVC_ARIADNE_FTXUI
#include <ftxui/component/component.hpp>          // Renderer, CatchEvent
#include <ftxui/component/screen_interactive.hpp> // ScreenInteractive (owns_loop)
#include <ftxui/dom/elements.hpp>
#include <ftxui/screen/screen.hpp>
#endif

namespace cvc {
namespace ariadne {

#ifdef CVC_ARIADNE_FTXUI

namespace fx = ftxui;

struct FtxuiBackend::Impl {
  std::vector<fx::Elements> stack;   // container stack; top holds the current list
  std::vector<std::string> titles;   // window titles, parallel to window pushes
  std::vector<Layout> grids;         // open grids, parallel to grid container pushes
  fx::Element root = fx::text("");

  void open() { stack.emplace_back(); }
  fx::Elements close() {
    fx::Elements e = std::move(stack.back());
    stack.pop_back();
    return e;
  }
  void add(fx::Element e) {
    if (!stack.empty())
      stack.back().push_back(std::move(e));
  }
};

FtxuiBackend::FtxuiBackend() : m_(new Impl) {}
FtxuiBackend::~FtxuiBackend() = default;

Capabilities FtxuiBackend::capabilities() const {
  Capabilities c;
  c.windows = false;  // FTXUI is fullscreen/tiled: `free` degrades to tiled (§16.2)
  c.menubar = true;
  c.mouse = true;
  c.keyboard = true;
  c.color = true;
  c.glsl = false;     // no GLSL on a terminal — a shader widget degrades (§16.3)
  c.view_embed = false;
  c.owns_loop = true; // FTXUI's ScreenInteractive owns the loop (§16.2)
  return c;
}

void FtxuiBackend::begin_frame() {
  m_->stack.clear();
  m_->titles.clear();
  m_->grids.clear();
  m_->open(); // the root child-list
}

void FtxuiBackend::end_frame() {
  fx::Elements items = m_->stack.empty() ? fx::Elements{} : m_->close();
  m_->root = items.empty() ? fx::text("(empty)") : fx::vbox(std::move(items));
}

bool FtxuiBackend::begin_main_menu_bar() {
  m_->open();
  return true;
}
void FtxuiBackend::end_main_menu_bar() {
  fx::Elements items = m_->close();
  if (!items.empty())
    m_->add(fx::hbox(std::move(items)) | fx::inverted);
}

bool FtxuiBackend::begin_menu(const char *label) {
  // A terminal has no live drop-downs in this static render: show the menu name
  // in the bar and skip its items (returning false stops the core emitting them,
  // and end_menu is then not called).
  m_->add(fx::text(std::string("  ") + label + "  "));
  return false;
}
void FtxuiBackend::end_menu() {}

bool FtxuiBackend::begin_window(const char *title, const char * /*id*/, const Size & /*size*/,
                                float /*border*/) {
  // §3.0.3b pixel sizing / border width don't translate to a terminal — FTXUI
  // auto-sizes the window box to its content (the §16.2 degradation). The size
  // spec is still honoured for TILED/grid tracks via begin_grid below.
  m_->open();
  m_->titles.emplace_back(title);
  return true;
}
void FtxuiBackend::end_window() {
  fx::Elements body = m_->close();
  std::string title = m_->titles.empty() ? std::string() : m_->titles.back();
  if (!m_->titles.empty())
    m_->titles.pop_back();
  m_->add(fx::window(fx::text(" " + title + " "), fx::vbox(std::move(body))));
}

bool FtxuiBackend::begin_grid(const Layout &layout, const char * /*id*/) {
  m_->grids.push_back(layout); // copy — the walk's Widget outlives the call, but copy is cheap + safe
  m_->open();
  return true;
}
void FtxuiBackend::grid_next_cell() {} // each child accumulates as its own cell

void FtxuiBackend::end_grid() {
  fx::Elements cells = m_->close();
  Layout layout = m_->grids.empty() ? Layout{} : m_->grids.back();
  if (!m_->grids.empty())
    m_->grids.pop_back();
  // Realize the §3.0.3b tracks as a terminal row: px -> a fixed cell width,
  // percent -> a flexible (growing) cell, auto -> natural width. Resizable
  // splitters degrade to a static split (§16.2).
  for (std::size_t i = 0; i < cells.size(); ++i) {
    if (i >= layout.col_widths.size())
      continue;
    const Track &t = layout.col_widths[i];
    if (t.unit == Unit::Px)
      cells[i] = cells[i] | fx::size(fx::WIDTH, fx::EQUAL, static_cast<int>(t.value));
    else if (t.unit == Unit::Percent)
      cells[i] = fx::flex(cells[i]);
  }
  fx::Element row = cells.empty() ? fx::text("") : fx::hbox(std::move(cells));
  if (layout.borders != BorderShow::None)
    row = fx::border(row); // any cell-border request -> a box around the row
  m_->add(row);
}

void FtxuiBackend::push_id(const char * /*id*/) {}
void FtxuiBackend::pop_id() {}

void FtxuiBackend::text_line(const char *t) { m_->add(fx::text(t)); }
void FtxuiBackend::text_value(const char *label, const std::string &value) {
  m_->add(fx::text(std::string(label) + ": " + value));
}
void FtxuiBackend::separator() { m_->add(fx::separator()); }

bool FtxuiBackend::button(const char *label) {
  m_->add(fx::text(std::string("[ ") + label + " ]") | fx::bold);
  return false; // display-only in this P0 backend
}
bool FtxuiBackend::menu_item_action(const char * /*label*/) { return false; }

BoolEdit FtxuiBackend::menu_item_toggle(const char *label, bool current) {
  m_->add(fx::text(std::string(current ? "[x] " : "[ ] ") + label));
  return {};
}

BoolEdit FtxuiBackend::checkbox(const char *label, bool current) {
  m_->add(fx::text(std::string(current ? "[x] " : "[ ] ") + label));
  return {};
}

IntEdit FtxuiBackend::slider_int(const char *label, const char * /*key*/, int current, int lo,
                                 int hi) {
  const float p = hi > lo ? std::clamp(float(current - lo) / float(hi - lo), 0.f, 1.f) : 0.f;
  m_->add(fx::hbox(fx::Elements{fx::text(std::string(label) + ": "),
                                fx::gauge(p) | fx::size(fx::WIDTH, fx::EQUAL, 20),
                                fx::text(" " + std::to_string(current))}));
  return {};
}

DoubleEdit FtxuiBackend::slider_double(const char *label, const char * /*key*/, double current,
                                       double lo, double hi, const char *fmt) {
  const float p = hi > lo ? std::clamp(float((current - lo) / (hi - lo)), 0.f, 1.f) : 0.f;
  char buf[64];
  std::snprintf(buf, sizeof(buf), fmt ? fmt : "%.3f", current);
  m_->add(fx::hbox(fx::Elements{fx::text(std::string(label) + ": "),
                                fx::gauge(p) | fx::size(fx::WIDTH, fx::EQUAL, 20),
                                fx::text(std::string(" ") + buf)}));
  return {};
}

IndexEdit FtxuiBackend::combo(const char *label, int current_index,
                              const std::vector<std::string> &options) {
  std::string v;
  if (current_index >= 0 && current_index < static_cast<int>(options.size()))
    v = options[current_index];
  m_->add(fx::text(std::string(label) + ": < " + v + " >"));
  return {};
}

std::string FtxuiBackend::render_to_string(Runtime &rt, int width) {
  rt.render(); // walk the tree → (re)builds m_->root through the methods above
  fx::Screen screen = fx::Screen::Create(fx::Dimension::Fixed(width), fx::Dimension::Fit(m_->root));
  fx::Render(screen, m_->root);
  return screen.ToString();
}

void FtxuiBackend::run(Runtime &rt) {
  // owns_loop: FTXUI's ScreenInteractive owns the event loop and drives the walk.
  fx::ScreenInteractive screen = fx::ScreenInteractive::Fullscreen();
  fx::Component renderer = fx::Renderer([&] {
    rt.render(); // rebuilds m_->root through the Backend methods above
    return m_->root;
  });
  // Drain queued action intents after FTXUI processes each event (render only
  // enqueues; drain runs the host handlers off the walk — §7.2). A "quit"
  // handler registered on the Runtime can call screen.Exit() to leave the loop;
  // otherwise the terminal's interrupt does.
  fx::Component app = fx::CatchEvent(renderer, [&](fx::Event) {
    rt.drain();
    return false; // let the event keep propagating
  });
  screen.Loop(app);
}

bool FtxuiBackend::available() { return true; }

#else // !CVC_ARIADNE_FTXUI — inert stub so libcvc builds without FTXUI.

struct FtxuiBackend::Impl {};
FtxuiBackend::FtxuiBackend() : m_(new Impl) {}
FtxuiBackend::~FtxuiBackend() = default;
Capabilities FtxuiBackend::capabilities() const { return {}; }
void FtxuiBackend::begin_frame() {}
void FtxuiBackend::end_frame() {}
bool FtxuiBackend::begin_main_menu_bar() { return false; }
void FtxuiBackend::end_main_menu_bar() {}
bool FtxuiBackend::begin_menu(const char *) { return false; }
void FtxuiBackend::end_menu() {}
bool FtxuiBackend::begin_window(const char *, const char *, const Size &, float) { return false; }
void FtxuiBackend::end_window() {}
bool FtxuiBackend::begin_grid(const Layout &, const char *) { return false; }
void FtxuiBackend::grid_next_cell() {}
void FtxuiBackend::end_grid() {}
void FtxuiBackend::push_id(const char *) {}
void FtxuiBackend::pop_id() {}
void FtxuiBackend::text_line(const char *) {}
void FtxuiBackend::text_value(const char *, const std::string &) {}
void FtxuiBackend::separator() {}
bool FtxuiBackend::button(const char *) { return false; }
bool FtxuiBackend::menu_item_action(const char *) { return false; }
BoolEdit FtxuiBackend::menu_item_toggle(const char *, bool) { return {}; }
BoolEdit FtxuiBackend::checkbox(const char *, bool) { return {}; }
IntEdit FtxuiBackend::slider_int(const char *, const char *, int, int, int) { return {}; }
DoubleEdit FtxuiBackend::slider_double(const char *, const char *, double, double, double,
                                       const char *) {
  return {};
}
IndexEdit FtxuiBackend::combo(const char *, int, const std::vector<std::string> &) { return {}; }
std::string FtxuiBackend::render_to_string(Runtime &, int) {
  return "ari: libcvc was built without FTXUI (terminal backend unavailable)\n";
}
void FtxuiBackend::run(Runtime &) {}
bool FtxuiBackend::available() { return false; }

#endif

} // namespace ariadne
} // namespace cvc
