// FtxuiBackend — the pure-terminal Ariadne backend (see the header).
//
// The bridge. FTXUI is a functional DOM (build an Element tree, render it); the Ariadne Backend
// interface is immediate-mode (the core walk calls begin_* / checkbox / ... every frame). So each
// walk accumulates an Element tree on a stack of containers — begin_frame opens the root list,
// leaves append to the top container, and begin/end of a window, menu, or grid push/pop one — and
// end_frame composes the result into one screen: the menu bar over the root content, then every
// floating window as its own z-ordered layer, then any open popup (a menu dropdown, a combo list)
// and the tooltip line on top.
//
// Interaction is ImGui-style. Every leaf gets an id (the push_id path plus a per-kind base) and its
// Element is wrapped in fx::reflect, so FTXUI records the leaf's on-screen Box when it lays the
// frame out. An input event is then handled in two steps (feed):
//   1. route: things that need no widget — Tab focus moves, menu / combo popups, window drag and
//      raise, and the mouse hit-test against the boxes of the last frame DRAWN — update the
//      cross-frame state (focused / pressed / active ids, open popup, window placement);
//   2. a walk: rt.render() runs the tree, and the widget the event landed on reads it (a key on the
//      focused widget, a press on the pressed one) and returns its edit, which the core commits.
// Commit policy follows the ImGui backend: discrete widgets commit on the edit; a slider dragged
// with the mouse reports changes while held and commits on release (arrow-key steps commit at
// once); a text field commits on Enter or when focus leaves it.
//
// NB: fx:: is an alias for ftxui. It is NOT `using namespace ftxui`, on purpose: cvc::ariadne
// already has widget builders named text()/window()/separator()/checkbox()/... (widget.h), which
// would shadow FTXUI's same-named free functions inside this namespace. Every FTXUI call is
// qualified fx:: to pick the terminal DOM function, never the Widget builder.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/ftxui_backend.h>
#include <cvc/ariadne/input.h> // InputEvent::Kind, for forwarding pointer events
#include <cvc/image/image.h>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#ifdef CVC_ARIADNE_FTXUI
#include <ftxui/component/component.hpp> // Renderer, CatchEvent
#include <ftxui/component/event.hpp>
#include <ftxui/component/loop.hpp>
#include <ftxui/component/screen_interactive.hpp> // ScreenInteractive (owns_loop)
#include <ftxui/dom/elements.hpp>
#include <ftxui/dom/node.hpp>
#include <ftxui/screen/screen.hpp>
#include <ftxui/screen/terminal.hpp>
#ifdef __EMSCRIPTEN__
#include <emscripten.h> // emscripten_sleep: the run loop yields to the browser (Asyncify)
#endif
#endif

namespace cvc {
namespace ariadne {

// ---- TermEvent ---------------------------------------------------------------------------------

TermEvent TermEvent::key_event(std::string name, bool shift, bool ctrl, bool alt) {
  TermEvent e;
  e.kind = Kind::Key;
  e.key = std::move(name);
  e.shift = shift;
  e.ctrl = ctrl;
  e.alt = alt;
  return e;
}

TermEvent TermEvent::text_event(std::string utf8) {
  TermEvent e;
  e.kind = Kind::Text;
  if (utf8 == " ")
    e.key = "Space";
  else if (utf8.size() == 1 && utf8[0] >= 'a' && utf8[0] <= 'z')
    e.key = std::string(1, static_cast<char>(utf8[0] - 'a' + 'A'));
  else if (utf8.size() == 1 && static_cast<unsigned char>(utf8[0]) < 0x80)
    e.key = utf8;
  e.text = std::move(utf8);
  return e;
}

TermEvent TermEvent::mouse_event(Mouse m, int x, int y, int button) {
  TermEvent e;
  e.kind = Kind::Mouse;
  e.mouse = m;
  e.x = x;
  e.y = y;
  e.button = button;
  return e;
}

#ifdef CVC_ARIADNE_FTXUI

namespace fx = ftxui;

namespace {

std::size_t utf8_prev(const std::string &s, std::size_t i) {
  if (i == 0)
    return 0;
  --i;
  while (i > 0 && (static_cast<unsigned char>(s[i]) & 0xC0) == 0x80)
    --i;
  return i;
}

std::size_t utf8_next(const std::string &s, std::size_t i) {
  if (i >= s.size())
    return s.size();
  ++i;
  while (i < s.size() && (static_cast<unsigned char>(s[i]) & 0xC0) == 0x80)
    ++i;
  return i;
}

// A caption that may span lines: one text row per line.
fx::Element text_block(const std::string &s) {
  if (s.find('\n') == std::string::npos)
    return fx::text(s);
  fx::Elements rows;
  std::size_t start = 0;
  while (start <= s.size()) {
    const std::size_t nl = s.find('\n', start);
    rows.push_back(fx::text(s.substr(start, nl == std::string::npos ? nl : nl - start)));
    if (nl == std::string::npos)
      break;
    start = nl + 1;
  }
  return fx::vbox(std::move(rows));
}

std::uint8_t to_u8(float v) {
  return static_cast<std::uint8_t>(std::lround(std::clamp(v, 0.0f, 1.0f) * 255.0f));
}

fx::Color rgb_color(const float rgb[3]) {
  return fx::Color::RGB(to_u8(rgb[0]), to_u8(rgb[1]), to_u8(rgb[2]));
}

// Place `e` with its top-left at cell (x, y) of a dbox layer.
fx::Element place(fx::Element e, int x, int y) {
  return fx::vbox({fx::emptyElement() | fx::size(fx::HEIGHT, fx::EQUAL, std::max(0, y)),
                   fx::hbox({fx::emptyElement() | fx::size(fx::WIDTH, fx::EQUAL, std::max(0, x)),
                             std::move(e)})});
}

bool inside(const fx::Box &b, int x, int y) {
  return x >= b.x_min && x <= b.x_max && y >= b.y_min && y <= b.y_max;
}

// A layer (a floating window, a menu dropdown, a combo list) drawn over the rest of the screen.
// FTXUI joins adjacent box-drawing characters across the whole screen when it finishes a frame, so
// a window overlapping another would fuse its border into the one beneath ("│" + "─" -> "├"). This
// renders the child into a scratch screen, joins ITS lines there (a window's own separators still
// meet its frame), and copies the result into its box with joining switched off.
class Isolated : public fx::Node {
public:
  explicit Isolated(fx::Element child) : fx::Node(fx::Elements{std::move(child)}) {}

  void ComputeRequirement() override {
    children_[0]->ComputeRequirement();
    requirement_ = children_[0]->requirement();
  }

  void SetBox(fx::Box box) override {
    fx::Node::SetBox(box);
    children_[0]->SetBox(box);
  }

  void Render(fx::Screen &screen) override {
    fx::Screen scratch(screen.dimx(), screen.dimy());
    children_[0]->Render(scratch);
    scratch.ApplyShader();
    const int x1 = std::min(box_.x_max, screen.dimx() - 1);
    const int y1 = std::min(box_.y_max, screen.dimy() - 1);
    for (int y = std::max(0, box_.y_min); y <= y1; ++y)
      for (int x = std::max(0, box_.x_min); x <= x1; ++x) {
        fx::Cell &out = screen.CellAt(x, y);
        out = scratch.CellAt(x, y);
        out.automerge = false;
      }
  }
};

fx::Element isolated(fx::Element e) { return std::make_shared<Isolated>(std::move(e)); }

// An RGB image drawn as half-block cells: each cell is two vertically stacked pixels, the upper
// one the glyph's foreground and the lower one its background ('▀'). On a terminal with no colour
// it falls back to a luminance ramp, one glyph per cell. `pixels` is cols x (2 * rows) RGB8.
class HalfBlockImage : public fx::Node {
public:
  HalfBlockImage(cvc::image pixels, int cols, int rows, FtxuiBackend::Options::ImageColor mode)
      : pixels_(std::move(pixels)), cols_(cols), rows_(rows), mode_(mode) {}

  void ComputeRequirement() override {
    requirement_.min_x = cols_;
    requirement_.min_y = rows_;
  }

  void Render(fx::Screen &screen) override {
    using Mode = FtxuiBackend::Options::ImageColor;
    const bool mono =
        mode_ == Mode::Mono ||
        (mode_ == Mode::Auto && fx::Terminal::ColorSupport() == fx::Terminal::Color::Palette1);
    const unsigned char *px = std::as_const(pixels_).data(); // const: no copy-on-write fork
    const int w = pixels_.width();
    auto at = [&](int x, int y) { return px + (static_cast<std::size_t>(y) * w + x) * 3; };
    static const char *kRamp[] = {" ", ".", ":", "-", "=", "+", "*", "#", "%", "@"};
    for (int r = 0; r < rows_; ++r) {
      const int y = box_.y_min + r;
      if (y > box_.y_max)
        break;
      for (int c = 0; c < cols_; ++c) {
        const int x = box_.x_min + c;
        if (x > box_.x_max)
          break;
        const unsigned char *top = at(c, 2 * r);
        const unsigned char *bot = at(c, std::min(2 * r + 1, pixels_.height() - 1));
        fx::Cell &cell = screen.CellAt(x, y);
        if (mono) {
          const int lum =
              (top[0] * 3 + top[1] * 6 + top[2] + bot[0] * 3 + bot[1] * 6 + bot[2]) / 20;
          cell.character = kRamp[std::clamp(lum * 10 / 256, 0, 9)];
        } else {
          cell.character = "▀"; // ▀
          cell.foreground_color = fx::Color::RGB(top[0], top[1], top[2]);
          cell.background_color = fx::Color::RGB(bot[0], bot[1], bot[2]);
        }
      }
    }
  }

private:
  cvc::image pixels_;
  int cols_, rows_;
  FtxuiBackend::Options::ImageColor mode_;
};

} // namespace

struct FtxuiBackend::Impl {
  Options opt;

  // ---- one walk --------------------------------------------------------------------------------

  // A container being filled during the walk.
  struct Frame {
    enum class Type { Root, Menubar, Menu, InlineMenu, Window, Grid };
    Type type = Type::Root;
    fx::Elements items;
    std::vector<fx::Elements> cells; // Grid: one list per cell
    Layout layout;                   // Grid
    std::vector<float> tracks;       // Grid: seeded row-split sizes (px)
    std::string id;                  // Window / Menu
    std::string title;               // Window
    Size size;                       // Window: the authored size spec (min / max)
    bool *p_open = nullptr;          // Window
    void add(fx::Element e) {
      if (type == Type::Grid) {
        if (cells.empty())
          cells.emplace_back();
        cells.back().push_back(std::move(e));
      } else {
        items.push_back(std::move(e));
      }
    }
  };

  // A draggable boundary between two grid tracks: `index` is the track after it (rows of a row
  // split, or columns of a resizable table), `sizes` the GridSizes key, `box` its reflected box.
  struct Seam {
    std::string sizes;
    int index = 0;
    bool rows = true;
    std::string win;
    std::string box;
  };

  // One emitted leaf, kept for the next event's focus walk and hit-test.
  struct Item {
    std::string id;
    std::string win;  // owning window id; "" = the root layer
    std::string menu; // owning dropdown; "" = none
    bool focusable = false;
  };

  // What a walk produced, beyond the Element tree.
  struct Walk {
    std::vector<Item> items;
    std::vector<std::string> focus_order; // focusable leaves outside any dropdown, in walk order
    std::vector<std::string> menu_titles; // menubar menus, left to right
    std::map<std::string, std::vector<std::string>> menu_items; // dropdown -> its items
    std::vector<std::string> windows;                           // windows, in emit order
    std::set<std::string> closable;                             // windows drawn with an [x]
    std::string menu_popup;                                     // dropdown drawn, if any
    std::string combo_popup;                                    // combo list drawn, if any
    int combo_n = 0;
    bool has_menubar = false;
    std::vector<Seam> seams; // draggable grid seams, for the hit-test
  };

  struct Layer {
    long z;
    fx::Element el;
  };

  std::vector<Frame> stack;
  std::vector<std::string> ids;  // the push_id path
  std::vector<std::string> wins; // windows open in the walk
  std::string cur_menu;          // the dropdown being emitted
  int disabled = 0;
  std::unordered_map<std::string, int> id_uses;
  Walk walk;  // being built
  Walk last;  // the last walk finished
  Walk shown; // the walk whose frame was last drawn (what the hit-test sees)
  std::vector<Layer> layers;
  fx::Elements popups;
  fx::Element menubar;
  fx::Element root = fx::text("");
  std::string tooltip;
  std::string last_item; // the leaf emitted last (item_* queries, set_tooltip)
  bool menu_close_req = false;
  WindowGeom pending_geom;
  bool have_geom = false;
  std::string last_window; // the window begin_window opened last (window_geometry)
  std::vector<float> pending_tracks;
  bool have_tracks = false;
  std::vector<float> last_tracks;
  int term_w = 80, term_h = 24;

  // ---- across frames ---------------------------------------------------------------------------

  struct Win {
    int x = 0, y = 0; // top-left cell
    int w = 0, h = 0; // outer size in cells when `sized`
    bool sized = false;
    bool placed = false;
    bool seeded = false;
    long z = 0;
  };
  struct Edit {
    std::string text;
    std::size_t cursor = 0; // byte offset, on a UTF-8 boundary
  };
  struct Picture {
    cvc::image rgb;    // the published image as RGB8
    cvc::image scaled; // rgb resampled to the last drawn cell grid
  };
  // A grid's track sizes in cells, kept across frames: `fixed` tracks hold `size` (seeded or
  // dragged); the rest lay out naturally.
  struct GridSizes {
    std::vector<int> size;
    std::vector<bool> fixed;
    bool seeded = false;
  };
  struct SeamDrag {
    bool active = false;
    std::string sizes;
    int index = 0;
    bool rows = true;
    int origin = 0;     // pointer row / column when the drag began
    int a0 = 0, b0 = 0; // the two tracks' sizes then
  };

  std::string focused;
  std::string hovered;
  std::string active; // the item the left button went down on, until it comes up
  std::string open_menu;
  std::string focus_before_menu;
  std::string open_combo;
  int combo_hi = 0;
  std::string drag_win;
  int drag_dx = 0, drag_dy = 0;
  std::map<std::string, Edit> edits;
  std::set<std::string> blur_commit;      // fields that lost focus with an edit pending
  std::map<std::string, double> inflight; // slider value while the mouse holds it
  std::map<std::string, int> color_chan;
  std::map<std::string, Win> windows;
  std::map<std::string, GridSizes> grid_sizes;
  SeamDrag seam_drag;
  long z_top = 0;
  std::unordered_map<std::string, fx::Box> boxes; // node-based: reflect() keeps references
  std::map<std::string, Picture> images;

  // ---- the event the current walk handles ------------------------------------------------------

  bool ev_live = false; // a key / text event still waiting for the focused widget
  TermEvent ev;
  bool consumed = false;
  std::string press_id, release_id, close_win;
  int combo_pick = -1;
  bool moved = false;
  int ptr_x = 0, ptr_y = 0, press_x = 0, press_y = 0;

  explicit Impl(const Options &o) : opt(o) {
    opt.cell_px_w = std::max(1, opt.cell_px_w);
    opt.cell_px_h = std::max(1, opt.cell_px_h);
  }

  int px_to_cols(float px) const { return static_cast<int>(std::lround(px / opt.cell_px_w)); }
  int px_to_rows(float px) const { return static_cast<int>(std::lround(px / opt.cell_px_h)); }
  int extent_cols(const Extent &e) const {
    if (e.unit == Unit::Px)
      return px_to_cols(e.value);
    if (e.unit == Unit::Percent)
      return static_cast<int>(std::lround(term_w * e.value / 100.0f));
    return 0;
  }
  int extent_rows(const Extent &e) const {
    if (e.unit == Unit::Px)
      return px_to_rows(e.value);
    if (e.unit == Unit::Percent)
      return static_cast<int>(std::lround(term_h * e.value / 100.0f));
    return 0;
  }
  int top_row() const { return (walk.has_menubar || last.has_menubar) ? 1 : 0; }

  Frame &top() { return stack.back(); }
  void add(fx::Element e) {
    if (!stack.empty())
      top().add(std::move(e));
  }
  Frame pop() {
    Frame f = std::move(stack.back());
    stack.pop_back();
    return f;
  }

  // A per-walk unique, frame-to-frame stable id: the push_id path plus `base`, with a repeat
  // counter when the same base recurs under the same path.
  std::string uid(const std::string &base) {
    std::string id;
    for (const std::string &s : ids) {
      id += s;
      id += '/';
    }
    id += base;
    const int n = ++id_uses[id];
    if (n > 1)
      id += "#" + std::to_string(n);
    return id;
  }

  bool is_disabled() const { return disabled > 0; }
  bool has_focus(const std::string &id) const { return !is_disabled() && focused == id; }

  // Record a leaf and append its Element (reflected, so the next event can hit-test it).
  void leaf(const std::string &id, bool focusable, fx::Element el) {
    Item it;
    it.id = id;
    it.win = wins.empty() ? std::string() : wins.back();
    it.menu = cur_menu;
    it.focusable = focusable && !is_disabled();
    if (it.focusable) {
      if (!cur_menu.empty())
        walk.menu_items[cur_menu].push_back(id);
      else
        walk.focus_order.push_back(id);
    }
    if (is_disabled())
      el = el | fx::dim;
    if (it.focusable && focused == id)
      el = el | fx::focus; // a scrolled window keeps its focused widget in view
    add(el | fx::reflect(boxes[id]));
    walk.items.push_back(std::move(it));
    last_item = id;
  }

  // The focused widget takes the pending key event `name` (consuming it).
  bool key(const std::string &id, const char *name) {
    if (!ev_live || !has_focus(id) || ev.kind == TermEvent::Kind::Mouse || ev.key != name)
      return false;
    ev_live = false;
    consumed = true;
    return true;
  }
  bool pressed(const std::string &id) const { return !is_disabled() && press_id == id; }
  bool activated(const std::string &id) {
    return pressed(id) || key(id, "Return") || key(id, "Space");
  }

  // ---- focus / windows / menus -----------------------------------------------------------------

  const Item *find_item(const Walk &w, const std::string &id) const {
    for (const Item &it : w.items)
      if (it.id == id)
        return &it;
    return nullptr;
  }

  void raise(const std::string &win) {
    if (!win.empty())
      windows[win].z = ++z_top;
  }

  void set_focus(const std::string &id) {
    if (focused == id)
      return;
    if (!focused.empty() && edits.count(focused))
      blur_commit.insert(focused); // the field it leaves commits on the next walk
    focused = id;
    if (const Item *it = find_item(last, id))
      raise(it->win);
  }

  void focus_step(int dir) {
    const std::vector<std::string> &order = last.focus_order;
    if (order.empty())
      return;
    const int n = static_cast<int>(order.size());
    const auto at = std::find(order.begin(), order.end(), focused);
    int i = at == order.end() ? (dir > 0 ? 0 : n - 1)
                              : (static_cast<int>(at - order.begin()) + dir + n) % n;
    set_focus(order[static_cast<std::size_t>(i)]);
  }

  void open_menu_at(const std::string &title) {
    if (open_menu.empty())
      focus_before_menu = focused;
    open_menu = title;
    const auto it = last.menu_items.find(title);
    focused = (it != last.menu_items.end() && !it->second.empty()) ? it->second.front() : "";
  }

  void close_menu() {
    if (open_menu.empty())
      return;
    open_menu.clear();
    focused = focus_before_menu;
  }

  std::string window_at(int x, int y) const {
    std::string best;
    long best_z = -1;
    for (const std::string &w : shown.windows) {
      const auto b = boxes.find("win|" + w);
      const auto wi = windows.find(w);
      if (b == boxes.end() || wi == windows.end() || !inside(b->second, x, y))
        continue;
      if (wi->second.z > best_z) {
        best_z = wi->second.z;
        best = w;
      }
    }
    return best;
  }

  const Item *item_at(int x, int y, const std::string &win, const std::string &menu) const {
    for (const Item &it : shown.items) {
      if (it.win != win || it.menu != menu)
        continue;
      const auto b = boxes.find(it.id);
      if (b != boxes.end() && inside(b->second, x, y))
        return &it;
    }
    return nullptr;
  }

  // ---- routing (step 1 of feed) ----------------------------------------------------------------

  void route(const TermEvent &e) {
    if (e.kind == TermEvent::Kind::Mouse) {
      route_mouse(e);
      return;
    }
    if (!open_menu.empty()) {
      route_menu_key(e);
      return;
    }
    if (!open_combo.empty()) {
      route_combo_key(e);
      return;
    }
    if (e.key == "F10" && !last.menu_titles.empty()) {
      open_menu_at(last.menu_titles.front());
      use();
      return;
    }
    if (e.kind == TermEvent::Kind::Key && e.key == "Tab") {
      focus_step(e.shift ? -1 : 1);
      use();
    }
    // Anything else waits for the focused widget in the walk.
  }

  void use() {
    consumed = true;
    ev_live = false;
  }

  void route_menu_key(const TermEvent &e) {
    const std::vector<std::string> &titles = last.menu_titles;
    if (e.key == "Escape" || e.key == "F10" || e.key == "Tab") {
      close_menu();
    } else if (e.key == "Left" || e.key == "Right") {
      const auto at = std::find(titles.begin(), titles.end(), open_menu);
      if (at != titles.end()) {
        const int n = static_cast<int>(titles.size());
        const int i = (static_cast<int>(at - titles.begin()) + (e.key == "Left" ? -1 : 1) + n) % n;
        open_menu_at(titles[static_cast<std::size_t>(i)]);
      }
    } else if (e.key == "Up" || e.key == "Down") {
      const auto mi = last.menu_items.find(open_menu);
      if (mi != last.menu_items.end() && !mi->second.empty()) {
        const std::vector<std::string> &v = mi->second;
        const int n = static_cast<int>(v.size());
        const auto at = std::find(v.begin(), v.end(), focused);
        const int i = at == v.end()
                          ? 0
                          : (static_cast<int>(at - v.begin()) + (e.key == "Up" ? -1 : 1) + n) % n;
        focused = v[static_cast<std::size_t>(i)];
      }
    } else if (e.key == "Return" || e.key == "Space") {
      consumed = true;
      return; // stays live: the focused menu item activates in the walk
    }
    use(); // a menu swallows every other key while it is open
  }

  void route_combo_key(const TermEvent &e) {
    const int n = std::max(1, last.combo_n);
    if (e.key == "Escape" || e.key == "Tab")
      open_combo.clear();
    else if (e.key == "Up")
      combo_hi = std::max(0, combo_hi - 1);
    else if (e.key == "Down")
      combo_hi = std::min(n - 1, combo_hi + 1);
    else if (e.key == "Home")
      combo_hi = 0;
    else if (e.key == "End")
      combo_hi = n - 1;
    else if (e.key == "Return" || e.key == "Space")
      combo_pick = combo_hi;
    use();
  }

  void route_mouse(const TermEvent &e) {
    ptr_x = e.x;
    ptr_y = e.y;
    switch (e.mouse) {
    case TermEvent::Mouse::Move:
      if (seam_drag.active) {
        drag_seam(seam_drag.rows ? e.y : e.x);
        consumed = true;
      } else if (!drag_win.empty()) {
        Win &w = windows[drag_win];
        w.x = std::clamp(e.x - drag_dx, 0, std::max(0, term_w - 4));
        w.y = std::clamp(e.y - drag_dy, top_row(), std::max(top_row(), term_h - 2));
        consumed = true;
      } else if (!active.empty()) {
        moved = true;
        consumed = true;
      } else {
        const Item *it = item_at(e.x, e.y, window_at(e.x, e.y), "");
        hovered = it ? it->id : "";
      }
      return;
    case TermEvent::Mouse::Release:
      if (seam_drag.active) {
        seam_drag.active = false;
        consumed = true;
      } else if (!drag_win.empty()) {
        drag_win.clear();
        consumed = true;
      } else if (!active.empty()) {
        release_id = active;
        consumed = true;
      }
      return;
    case TermEvent::Mouse::WheelUp:
    case TermEvent::Mouse::WheelDown:
      if (!open_combo.empty()) {
        const int n = std::max(1, last.combo_n);
        combo_hi = std::clamp(combo_hi + (e.mouse == TermEvent::Mouse::WheelUp ? -1 : 1), 0, n - 1);
        consumed = true;
      }
      return;
    case TermEvent::Mouse::Press:
      break;
    }
    if (e.button != 1)
      return; // only the left button drives widgets; the rest go to the document's on_pointer
    consumed = true;
    const int x = e.x, y = e.y;
    auto on_title = [&](const std::string &t) {
      const auto b = boxes.find(t);
      return b != boxes.end() && inside(b->second, x, y);
    };
    if (!open_menu.empty()) {
      const auto dd = boxes.find("menud|" + open_menu);
      if (shown.menu_popup == open_menu && dd != boxes.end() && inside(dd->second, x, y)) {
        if (const Item *it = item_at(x, y, "", open_menu)) {
          focused = it->id;
          press_id = it->id;
        }
        return;
      }
      for (const std::string &t : shown.menu_titles)
        if (on_title(t)) {
          if (t == open_menu)
            close_menu();
          else
            open_menu_at(t);
          return;
        }
      close_menu();
      return;
    }
    if (!open_combo.empty()) {
      if (shown.combo_popup == open_combo)
        for (int i = 0; i < shown.combo_n; ++i)
          if (on_title("combo|" + open_combo + "#" + std::to_string(i))) {
            combo_pick = i;
            return;
          }
      open_combo.clear();
      return;
    }
    for (const std::string &t : shown.menu_titles)
      if (on_title(t)) {
        open_menu_at(t);
        return;
      }
    const std::string win = window_at(x, y);
    if (!win.empty()) {
      raise(win);
      if (shown.closable.count(win) && on_title("winx|" + win)) {
        close_win = win;
        return;
      }
      if (y == boxes["win|" + win].y_min) { // the title bar: start a move
        const Win &w = windows[win];
        drag_win = win;
        drag_dx = x - w.x;
        drag_dy = y - w.y;
        return;
      }
    }
    for (const Seam &sm : shown.seams) {
      const auto b = boxes.find(sm.box);
      if (sm.win == win && b != boxes.end() && inside(b->second, x, y)) {
        begin_seam_drag(sm, sm.rows ? y : x);
        return;
      }
    }
    if (const Item *it = item_at(x, y, win, "")) {
      press_id = it->id;
      active = it->id;
      hovered = it->id;
      press_x = x;
      press_y = y;
      if (it->focusable)
        set_focus(it->id);
    } else {
      set_focus(""); // a click on empty space blurs (and so commits) the focused field
    }
  }

  // A seam drag resizes the two tracks either side of it, keeping their sum (each at least 1 cell).
  // A track that was laying out naturally is fixed at its drawn size when the drag begins.
  int drawn_size(const Seam &sm, int track) const {
    const std::string key = (sm.rows ? "pane|" : "col|") + sm.sizes + "#" + std::to_string(track);
    const auto b = boxes.find(key);
    if (b == boxes.end())
      return 1;
    return std::max(1, sm.rows ? b->second.y_max - b->second.y_min + 1
                               : b->second.x_max - b->second.x_min + 1);
  }
  void begin_seam_drag(const Seam &sm, int at) {
    GridSizes &gs = grid_sizes[sm.sizes];
    const std::size_t a = static_cast<std::size_t>(sm.index - 1),
                      b = static_cast<std::size_t>(sm.index);
    if (b >= gs.size.size())
      return;
    for (std::size_t t : {a, b})
      if (!gs.fixed[t]) {
        gs.size[t] = drawn_size(sm, static_cast<int>(t));
        gs.fixed[t] = true;
      }
    seam_drag = SeamDrag{true, sm.sizes, sm.index, sm.rows, at, gs.size[a], gs.size[b]};
  }
  void drag_seam(int at) {
    GridSizes &gs = grid_sizes[seam_drag.sizes];
    const std::size_t a = static_cast<std::size_t>(seam_drag.index - 1),
                      b = static_cast<std::size_t>(seam_drag.index);
    if (b >= gs.size.size())
      return;
    const int total = seam_drag.a0 + seam_drag.b0;
    const int na = std::clamp(seam_drag.a0 + (at - seam_drag.origin), 1, std::max(1, total - 1));
    gs.size[a] = na;
    gs.size[b] = std::max(1, total - na);
  }

  // Clear the per-event state after the walk that handled it.
  void end_event() {
    if (!release_id.empty())
      active.clear();
    ev_live = false;
    press_id.clear();
    release_id.clear();
    close_win.clear();
    combo_pick = -1;
    moved = false;
  }

  // ---- composing the frame ---------------------------------------------------------------------

  void compose() {
    fx::Elements base = stack.empty() ? fx::Elements{} : std::move(stack.front().items);
    fx::Elements column;
    if (menubar)
      column.push_back(menubar);
    column.push_back(base.empty() ? fx::emptyElement() : fx::vbox(std::move(base)));
    fx::Elements screen{fx::vbox(std::move(column))};
    std::stable_sort(layers.begin(), layers.end(),
                     [](const Layer &a, const Layer &b) { return a.z < b.z; });
    for (Layer &l : layers)
      screen.push_back(std::move(l.el));
    for (fx::Element &p : popups)
      screen.push_back(std::move(p));
    if (!tooltip.empty())
      screen.push_back(place(fx::text(" " + tooltip + " ") | fx::inverted | fx::clear_under, 0,
                             std::max(0, term_h - 1)));
    root = fx::dbox(std::move(screen));
  }

  // The menu-and-focus bookkeeping a finished walk settles.
  void settle() {
    if (menu_close_req) {
      close_menu();
      menu_close_req = false;
    }
    if (!open_menu.empty()) {
      if (std::find(walk.menu_titles.begin(), walk.menu_titles.end(), open_menu) ==
          walk.menu_titles.end()) {
        close_menu(); // its menu is gone
      } else {
        const std::vector<std::string> &v = walk.menu_items[open_menu];
        if (!v.empty() && std::find(v.begin(), v.end(), focused) == v.end())
          focused = v.front();
      }
    }
    if (!open_combo.empty() && walk.combo_popup != open_combo)
      open_combo.clear(); // its combo is no longer drawn
    last = std::move(walk);
    walk = Walk{};
  }
};

FtxuiBackend::FtxuiBackend() : m_(new Impl(Options{})) {}
FtxuiBackend::FtxuiBackend(const Options &opts) : m_(new Impl(opts)) {}
FtxuiBackend::~FtxuiBackend() = default;

Capabilities FtxuiBackend::capabilities() const {
  Capabilities c;
  c.windows = true; // floating, movable, z-ordered windows (cell-quantized)
  c.menubar = true;
  c.mouse = true;
  c.keyboard = true;
  c.color = true;
  c.glsl = false; // no GLSL on a terminal: a shader widget degrades (§16.3)
  c.view_embed = false;
  c.owns_loop = true; // run() owns the terminal's event loop (§16.2)
  return c;
}

void FtxuiBackend::begin_frame() {
  Impl &m = *m_;
  m.stack.clear();
  m.stack.emplace_back(); // the root list
  m.ids.clear();
  m.wins.clear();
  m.cur_menu.clear();
  m.disabled = 0;
  m.id_uses.clear();
  m.walk = Impl::Walk{};
  m.layers.clear();
  m.popups.clear();
  m.menubar = nullptr;
  m.tooltip.clear();
  m.last_item.clear();
  m.have_geom = false;
  m.have_tracks = false;
}

void FtxuiBackend::end_frame() {
  Impl &m = *m_;
  // An unbalanced walk (a backend throw mid-tree) leaves frames open; fold them into the root so
  // the screen still shows what was emitted.
  while (m.stack.size() > 1) {
    Impl::Frame f = m.pop();
    for (fx::Element &e : f.items)
      m.add(std::move(e));
  }
  m.compose();
  m.settle();
}

// ---- menus
// ---------------------------------------------------------------------------------------

bool FtxuiBackend::begin_main_menu_bar() {
  Impl &m = *m_;
  Impl::Frame f;
  f.type = Impl::Frame::Type::Menubar;
  m.stack.push_back(std::move(f));
  m.walk.has_menubar = true;
  return true;
}

void FtxuiBackend::end_main_menu_bar() {
  Impl &m = *m_;
  Impl::Frame f = m.pop();
  f.items.push_back(fx::filler() | fx::inverted);
  m.menubar = fx::hbox(std::move(f.items));
}

bool FtxuiBackend::begin_menu(const char *label) {
  Impl &m = *m_;
  if (m.top().type == Impl::Frame::Type::Menubar) {
    const std::string mid = std::string("menu|") + label;
    m.walk.menu_titles.push_back(mid);
    const bool open = m.open_menu == mid;
    fx::Element t = fx::text(std::string(" ") + label + " ");
    if (!open)
      t = t | fx::inverted; // the bar is inverted; the open menu's title stands out by NOT being
    m.add(t | fx::reflect(m.boxes[mid]));
    if (!open)
      return false;
    Impl::Frame f;
    f.type = Impl::Frame::Type::Menu;
    f.id = mid;
    m.stack.push_back(std::move(f));
    m.cur_menu = mid;
    return true;
  }
  // A submenu, or a menu outside the bar: an always-open list under a header line.
  m.add(fx::text(std::string(label) + " ▸") | fx::bold);
  Impl::Frame f;
  f.type = Impl::Frame::Type::InlineMenu;
  m.stack.push_back(std::move(f));
  return true;
}

void FtxuiBackend::end_menu() {
  Impl &m = *m_;
  Impl::Frame f = m.pop();
  if (f.type == Impl::Frame::Type::Menu) {
    m.cur_menu.clear();
    fx::Element dd = (f.items.empty() ? fx::text(" (empty) ") : fx::vbox(std::move(f.items))) |
                     fx::border | fx::reflect(m.boxes["menud|" + f.id]) | fx::clear_under;
    const fx::Box &title = m.boxes[f.id];
    m.popups.push_back(place(isolated(std::move(dd)), title.x_min, title.y_max + 1));
    m.walk.menu_popup = f.id;
    return;
  }
  m.add(fx::hbox({fx::text("  "), fx::vbox(std::move(f.items))}));
}

bool FtxuiBackend::menu_item_action(const char *label) {
  Impl &m = *m_;
  const std::string id = m.uid("mi|" + m.cur_menu + "/" + label);
  const bool hit = m.activated(id);
  fx::Element el = fx::text(std::string(" ") + label + " ");
  if (m.has_focus(id))
    el = el | fx::inverted;
  m.leaf(id, true, el);
  if (hit && !m.cur_menu.empty())
    m.menu_close_req = true;
  return hit;
}

BoolEdit FtxuiBackend::menu_item_toggle(const char *label, bool current) {
  Impl &m = *m_;
  const std::string id = m.uid("mt|" + m.cur_menu + "/" + label);
  BoolEdit e;
  e.value = current;
  if (m.activated(id)) {
    e.value = !current;
    e.changed = e.committed = true;
    if (!m.cur_menu.empty())
      m.menu_close_req = true;
  }
  fx::Element el = fx::text(std::string(e.value ? " [x] " : " [ ] ") + label + " ");
  if (m.has_focus(id))
    el = el | fx::inverted;
  m.leaf(id, true, el);
  return e;
}

// ---- windows
// -------------------------------------------------------------------------------------

void FtxuiBackend::seed_window_geometry(const WindowGeom &g) {
  m_->pending_geom = g;
  m_->have_geom = true;
}

bool FtxuiBackend::begin_window(const char *title, const char *id, const Size &size,
                                float /*border*/, bool *p_open) {
  Impl &m = *m_;
  const std::string wid = id ? id : title;
  Impl::Win &w = m.windows[wid];
  const bool fresh = !w.placed;
  // §11.4 apply-once seed: the persisted geometry (or the authored pos) places the window the
  // first time it appears; after that the user's moves own it.
  if (m.have_geom && !w.seeded) {
    w.seeded = true;
    if (m.pending_geom.has_pos) {
      w.x = m.px_to_cols(m.pending_geom.x);
      w.y = m.px_to_rows(m.pending_geom.y);
      w.placed = true;
    }
    if (m.pending_geom.has_size && m.pending_geom.w > 0 && m.pending_geom.h > 0) {
      w.w = m.px_to_cols(m.pending_geom.w);
      w.h = m.px_to_rows(m.pending_geom.h);
      w.sized = true;
    }
  }
  m.have_geom = false;
  if (!w.placed) { // cascade new windows so they don't stack exactly
    const int k = static_cast<int>(m.walk.windows.size());
    w.x = 2 + 3 * k;
    w.y = m.top_row() + 1 + k;
    w.placed = true;
  }
  if (!w.sized && (size.w.is_set() || size.h.is_set())) {
    w.w = m.extent_cols(size.w);
    w.h = m.extent_rows(size.h);
  }
  if (fresh)
    w.z = ++m.z_top;
  w.x = std::clamp(w.x, 0, std::max(0, m.term_w - 4));
  w.y = std::clamp(w.y, m.top_row(), std::max(m.top_row(), m.term_h - 2));
  if (p_open && m.close_win == wid) {
    *p_open = false;
    m.close_win.clear();
  }
  Impl::Frame f;
  f.type = Impl::Frame::Type::Window;
  f.id = wid;
  f.title = title ? title : "";
  f.size = size;
  f.p_open = p_open;
  m.stack.push_back(std::move(f));
  m.wins.push_back(wid);
  m.last_window = wid;
  return true;
}

WindowGeom FtxuiBackend::window_geometry() const {
  const Impl &m = *m_;
  WindowGeom g;
  const auto it = m.windows.find(m.last_window);
  if (it == m.windows.end())
    return g;
  const Impl::Win &w = it->second;
  g.has_pos = true;
  g.x = static_cast<float>(w.x * m.opt.cell_px_w);
  g.y = static_cast<float>(w.y * m.opt.cell_px_h);
  if (w.sized) {
    g.has_size = true;
    g.w = static_cast<float>(w.w * m.opt.cell_px_w);
    g.h = static_cast<float>(w.h * m.opt.cell_px_h);
  }
  return g;
}

void FtxuiBackend::end_window() {
  Impl &m = *m_;
  Impl::Frame f = m.pop();
  m.wins.pop_back();
  const Impl::Win &w = m.windows[f.id];
  // The window holding focus (or, failing that, the topmost) draws a heavy frame.
  bool holds_focus = false;
  for (const Impl::Item &it : m.walk.items)
    if (it.win == f.id && it.id == m.focused)
      holds_focus = true;
  fx::Element body = f.items.empty() ? fx::emptyElement() : fx::vbox(std::move(f.items));
  if (w.h > 0)
    body = body | fx::yframe | fx::flex; // scrolls to keep the focused widget in view
  fx::Elements bar{fx::text(" " + f.title + " ") | (holds_focus ? fx::bold : fx::nothing),
                   fx::filler()};
  if (f.p_open) {
    bar.push_back(fx::text("[x]") | fx::reflect(m.boxes["winx|" + f.id]));
    m.walk.closable.insert(f.id);
  }
  fx::Element el =
      fx::window(fx::hbox(std::move(bar)), body, holds_focus ? fx::HEAVY : fx::ROUNDED);
  if (w.w > 0)
    el = el | fx::size(fx::WIDTH, fx::EQUAL, std::max(4, w.w));
  else {
    if (f.size.min_w.is_set())
      el = el | fx::size(fx::WIDTH, fx::GREATER_THAN, m.extent_cols(f.size.min_w));
    if (f.size.max_w.is_set())
      el = el | fx::size(fx::WIDTH, fx::LESS_THAN, m.extent_cols(f.size.max_w));
  }
  if (w.h > 0)
    el = el | fx::size(fx::HEIGHT, fx::EQUAL, std::max(3, w.h));
  else {
    if (f.size.min_h.is_set())
      el = el | fx::size(fx::HEIGHT, fx::GREATER_THAN, m.extent_rows(f.size.min_h));
    if (f.size.max_h.is_set())
      el = el | fx::size(fx::HEIGHT, fx::LESS_THAN, m.extent_rows(f.size.max_h));
  }
  el = el | fx::reflect(m.boxes["win|" + f.id]) | fx::clear_under;
  m.layers.push_back({w.z, place(isolated(std::move(el)), w.x, w.y)});
  m.walk.windows.push_back(f.id);
}

void FtxuiBackend::push_id(const char *id) { m_->ids.emplace_back(id ? id : ""); }
void FtxuiBackend::pop_id() {
  if (!m_->ids.empty())
    m_->ids.pop_back();
}

// ---- grids
// ---------------------------------------------------------------------------------------

void FtxuiBackend::seed_grid_tracks(const std::vector<float> &sizes) {
  m_->pending_tracks = sizes;
  m_->have_tracks = true;
}

std::vector<float> FtxuiBackend::grid_tracks() const { return m_->last_tracks; }

bool FtxuiBackend::begin_grid(const Layout &layout, const char *id) {
  Impl &m = *m_;
  Impl::Frame f;
  f.type = Impl::Frame::Type::Grid;
  f.layout = layout;
  f.id = m.uid(std::string("grid|") + (id ? id : "")); // keys the grid's draggable sizes
  if (m.have_tracks)
    f.tracks = m.pending_tracks;
  m.have_tracks = false;
  m.stack.push_back(std::move(f));
  return true;
}

void FtxuiBackend::grid_next_cell() { m_->top().cells.emplace_back(); }

void FtxuiBackend::end_grid() {
  Impl &m = *m_;
  Impl::Frame f = m.pop();
  const Layout &L = f.layout;
  const std::string &gid = f.id;
  const std::string win = m.wins.empty() ? std::string() : m.wins.back();
  std::vector<fx::Element> cells;
  for (fx::Elements &c : f.cells)
    cells.push_back(c.empty() ? fx::emptyElement() : fx::vbox(std::move(c)));
  // An authored border colour tints the frames and seams, never the cell content.
  const fx::Decorator tint =
      L.has_border_color ? fx::color(rgb_color(L.border_color)) : fx::Decorator(fx::nothing);
  const fx::Decorator frame = L.has_border_color
                                  ? fx::borderStyled(fx::ROUNDED, rgb_color(L.border_color))
                                  : fx::borderStyled(fx::ROUNDED);
  // A seam the mouse can drag: tagged so route_mouse finds it, and drawn bold while held.
  auto seam = [&](fx::Element e, const std::string &sizes, int index, bool rows,
                  const std::string &box_key) {
    m.walk.seams.push_back({sizes, index, rows, win, box_key});
    const bool held =
        m.seam_drag.active && m.seam_drag.sizes == sizes && m.seam_drag.index == index;
    return e | tint | (held ? fx::bold : fx::nothing) | fx::reflect(m.boxes[box_key]);
  };

  if (L.is_row_split()) {
    // A row split: one framed stack of panes with a draggable seam between each pair. A pane's
    // height comes, once, from the persisted size (seed_grid_tracks) or the authored px track; a
    // pane with neither stretches until a drag fixes it. Sizes are content rows.
    const std::string key = "rows|" + gid;
    Impl::GridSizes &gs = m.grid_sizes[key];
    if (gs.size.size() != cells.size()) {
      gs.size.assign(cells.size(), 0);
      gs.fixed.assign(cells.size(), false);
      gs.seeded = false;
    }
    if (!gs.seeded) {
      for (std::size_t i = 0; i < cells.size(); ++i) {
        float px = 0.0f;
        if (i < f.tracks.size() && f.tracks[i] > 0.0f)
          px = f.tracks[i];
        else if (i < L.row_heights.size() && L.row_heights[i].unit == Unit::Px)
          px = L.row_heights[i].value;
        if (px > 0.0f) {
          gs.size[i] = std::max(1, m.px_to_rows(px));
          gs.fixed[i] = true;
        }
      }
      gs.seeded = true;
    }
    fx::Elements stack;
    m.last_tracks.clear();
    for (std::size_t i = 0; i < cells.size(); ++i) {
      if (i > 0)
        stack.push_back(seam(fx::separator(), key, static_cast<int>(i), true,
                             "seam|" + key + "#" + std::to_string(i)));
      const std::string pane_key = "pane|" + key + "#" + std::to_string(i);
      fx::Element pane = cells[i] | fx::reflect(m.boxes[pane_key]);
      pane = gs.fixed[i] ? pane | fx::yframe | fx::size(fx::HEIGHT, fx::EQUAL, gs.size[i])
                         : pane | fx::yflex;
      stack.push_back(std::move(pane));
      const fx::Box &b = m.boxes[pane_key];
      const int rows = gs.fixed[i] ? gs.size[i] : std::max(0, b.y_max - b.y_min + 1);
      m.last_tracks.push_back(static_cast<float>(rows * m.opt.cell_px_h));
    }
    m.add(stack.empty() ? fx::emptyElement() : fx::vbox(std::move(stack)) | frame);
    return;
  }
  m.last_tracks.clear();

  // A table: cells flow row-major into col_widths.size() columns (one per cell for a bare
  // horizontal layout, one column for a bare grid). px tracks fix a width, percent tracks stretch.
  // A `resizable` table gets a draggable seam between columns (drawn as a blank column when the
  // table shows no inner borders); a dragged column keeps its width.
  const int n = static_cast<int>(cells.size());
  int cols = static_cast<int>(L.col_widths.size());
  if (cols == 0)
    cols = L.kind == LayoutKind::Horizontal ? std::max(1, n) : 1;
  const int rows = n == 0 ? 0 : (n + cols - 1) / cols;
  const bool inner = L.borders == BorderShow::Inner || L.borders == BorderShow::All;
  const bool outer = L.borders == BorderShow::Outer || L.borders == BorderShow::All;
  const bool resizable = L.resizable && cols > 1;
  const std::string key = "cols|" + gid;
  Impl::GridSizes *cs = nullptr;
  if (resizable) {
    cs = &m.grid_sizes[key];
    if (cs->size.size() != static_cast<std::size_t>(cols)) {
      cs->size.assign(static_cast<std::size_t>(cols), 0);
      cs->fixed.assign(static_cast<std::size_t>(cols), false);
    }
  }
  std::vector<fx::Elements> lines;
  for (int r = 0; r < rows; ++r) {
    if (inner && r > 0) {
      fx::Elements rule;
      for (int c = 0; c < cols; ++c) {
        if (c > 0)
          rule.push_back(fx::separator() | tint);
        rule.push_back(fx::separator() | tint);
      }
      lines.push_back(std::move(rule));
    }
    fx::Elements line;
    for (int c = 0; c < cols; ++c) {
      const int i = r * cols + c;
      fx::Element cell = i < n ? cells[static_cast<std::size_t>(i)] : fx::emptyElement();
      if (cs && cs->fixed[static_cast<std::size_t>(c)]) {
        cell = cell | fx::size(fx::WIDTH, fx::EQUAL, cs->size[static_cast<std::size_t>(c)]);
      } else if (c < static_cast<int>(L.col_widths.size())) {
        const Track &t = L.col_widths[static_cast<std::size_t>(c)];
        if (t.unit == Unit::Px)
          cell = cell | fx::size(fx::WIDTH, fx::EQUAL, std::max(1, m.px_to_cols(t.value)));
        else if (t.unit == Unit::Percent)
          cell = cell | fx::xflex;
      }
      if (r < static_cast<int>(L.row_heights.size()) &&
          L.row_heights[static_cast<std::size_t>(r)].unit == Unit::Px)
        cell = cell | fx::size(fx::HEIGHT, fx::GREATER_THAN,
                               m.px_to_rows(L.row_heights[static_cast<std::size_t>(r)].value));
      if (r == 0 && cs) // a column's drawn width, for the drag that fixes it
        cell = cell | fx::reflect(m.boxes["col|" + key + "#" + std::to_string(c)]);
      if (c > 0 && (inner || resizable)) {
        fx::Element sep = inner ? fx::separator() : fx::separatorEmpty();
        line.push_back(resizable
                           ? seam(sep, key, c, false,
                                  "seam|" + key + "#" + std::to_string(c) + "@" + std::to_string(r))
                           : sep | tint);
      }
      line.push_back(std::move(cell));
    }
    lines.push_back(std::move(line));
  }
  fx::Element box = lines.empty() ? fx::emptyElement() : fx::gridbox(std::move(lines));
  if (outer)
    box = box | frame;
  m.add(box);
}

// ---- disabled / tooltip / pointer queries
// ---------------------------------------------------------

void FtxuiBackend::begin_disabled() { ++m_->disabled; }
void FtxuiBackend::end_disabled() {
  if (m_->disabled > 0)
    --m_->disabled;
}

void FtxuiBackend::set_tooltip(const char *text) {
  Impl &m = *m_;
  if (text && !m.last_item.empty() && (m.last_item == m.focused || m.last_item == m.hovered))
    m.tooltip = text;
}

bool FtxuiBackend::item_clicked() { return !m_->last_item.empty() && m_->pressed(m_->last_item); }
bool FtxuiBackend::item_hovered() {
  return !m_->last_item.empty() && !m_->is_disabled() && m_->hovered == m_->last_item;
}
bool FtxuiBackend::item_dragged() {
  return !m_->last_item.empty() && !m_->is_disabled() && m_->moved && m_->active == m_->last_item;
}
bool FtxuiBackend::item_drag_started() { return item_clicked(); }
bool FtxuiBackend::item_drag_ended() {
  return !m_->last_item.empty() && !m_->is_disabled() && m_->release_id == m_->last_item;
}

bool FtxuiBackend::item_pointer(float &x, float &y, float &dx, float &dy, int &button) {
  Impl &m = *m_;
  x = y = dx = dy = 0.0f;
  button = 0;
  if (m.last_item.empty())
    return false;
  const fx::Box &b = m.boxes[m.last_item];
  const float cw = static_cast<float>(m.opt.cell_px_w), ch = static_cast<float>(m.opt.cell_px_h);
  x = (m.ptr_x - b.x_min) * cw + cw / 2;
  y = (m.ptr_y - b.y_min) * ch + ch / 2;
  if (m.active == m.last_item) {
    dx = (m.ptr_x - m.press_x) * cw;
    dy = (m.ptr_y - m.press_y) * ch;
    button = 1;
  }
  return true;
}

// ---- leaves
// --------------------------------------------------------------------------------------

void FtxuiBackend::text_line(const char *t) { m_->leaf(m_->uid("t|"), false, text_block(t)); }

void FtxuiBackend::text_value(const char *label, const std::string &value) {
  m_->leaf(m_->uid("t|"), false, text_block(std::string(label) + ": " + value));
}

void FtxuiBackend::text_line_colored(const char *t, const float rgb[3]) {
  m_->leaf(m_->uid("t|"), false, text_block(t) | fx::color(rgb_color(rgb)));
}

void FtxuiBackend::text_value_colored(const char *label, const std::string &value,
                                      const float rgb[3]) {
  m_->leaf(m_->uid("t|"), false,
           text_block(std::string(label) + ": " + value) | fx::color(rgb_color(rgb)));
}

void FtxuiBackend::separator() { m_->add(fx::separator()); }

bool FtxuiBackend::button(const char *label) {
  Impl &m = *m_;
  const std::string id = m.uid(std::string("b|") + label);
  const bool hit = m.activated(id);
  fx::Element el = fx::text(std::string("[ ") + label + " ]");
  if (m.has_focus(id))
    el = el | fx::inverted;
  m.leaf(id, true, el);
  return hit;
}

BoolEdit FtxuiBackend::checkbox(const char *label, bool current) {
  Impl &m = *m_;
  const std::string id = m.uid(std::string("c|") + label);
  BoolEdit e;
  e.value = current;
  if (m.activated(id)) {
    e.value = !current;
    e.changed = e.committed = true;
  }
  fx::Element box = fx::text(e.value ? "[x]" : "[ ]");
  if (m.has_focus(id))
    box = box | fx::inverted;
  m.leaf(id, true, fx::hbox({box, fx::text(std::string(" ") + label)}));
  return e;
}

namespace {

// The keyboard step of a slider over [lo, hi]: Left/Right one step, PageDown/PageUp a tenth of the
// range, Home/End the ends. Returns whether a key moved it, with the new (unclamped) value.
template <class Take>
bool slider_key(Take &&take, double v, double lo, double hi, double step, double &out) {
  const double page = std::max(step, (hi - lo) / 10.0);
  if (take("Left"))
    out = v - step;
  else if (take("Right"))
    out = v + step;
  else if (take("PageDown"))
    out = v - page;
  else if (take("PageUp"))
    out = v + page;
  else if (take("Home"))
    out = lo;
  else if (take("End"))
    out = hi;
  else
    return false;
  return true;
}

constexpr int kGaugeWidth = 20;

} // namespace

IntEdit FtxuiBackend::slider_int(const char *label, const char *key, int current, int lo, int hi) {
  Impl &m = *m_;
  const std::string id = m.uid(std::string("si|") + (key ? key : label));
  const std::string gid = "g|" + id;
  IntEdit e;
  e.value = current;
  const auto fl = m.inflight.find(id);
  int v = fl != m.inflight.end() ? static_cast<int>(fl->second) : current;
  if (!m.is_disabled()) {
    double nv = v;
    if (slider_key([&](const char *k) { return m.key(id, k); }, v, lo, hi, 1.0, nv)) {
      const int iv = std::clamp(static_cast<int>(std::lround(nv)), lo, hi);
      m.inflight.erase(id);
      e = {iv != current, iv != current, iv};
      v = iv;
    } else {
      const fx::Box &g = m.boxes[gid];
      const bool on_gauge = m.press_id == id && inside(g, m.ptr_x, m.ptr_y);
      if ((on_gauge || (m.active == id && m.moved && m.inflight.count(id))) && hi > lo) {
        const double t =
            std::clamp(double(m.ptr_x - g.x_min) / std::max(1, g.x_max - g.x_min), 0.0, 1.0);
        v = lo + static_cast<int>(std::lround(t * (hi - lo)));
        m.inflight[id] = v;
        e.changed = v != current;
        e.value = v;
      }
      if (m.release_id == id && m.inflight.count(id)) {
        e.value = static_cast<int>(m.inflight[id]);
        e.changed = e.committed = true;
        m.inflight.erase(id);
        v = e.value;
      }
    }
  }
  const float p = hi > lo ? std::clamp(float(v - lo) / float(hi - lo), 0.f, 1.f) : 0.f;
  fx::Element name = fx::text(std::string(label) + " ");
  if (m.has_focus(id))
    name = name | fx::inverted;
  m.leaf(id, true,
         fx::hbox({name,
                   fx::gauge(p) | fx::size(fx::WIDTH, fx::EQUAL, kGaugeWidth) |
                       fx::reflect(m.boxes[gid]),
                   fx::text(" " + std::to_string(v))}));
  return e;
}

DoubleEdit FtxuiBackend::slider_double(const char *label, const char *key, double current,
                                       double lo, double hi, const char *fmt) {
  Impl &m = *m_;
  const std::string id = m.uid(std::string("sd|") + (key ? key : label));
  const std::string gid = "g|" + id;
  DoubleEdit e;
  e.value = current;
  const auto fl = m.inflight.find(id);
  double v = fl != m.inflight.end() ? fl->second : current;
  if (!m.is_disabled()) {
    const double step = (hi - lo) / (m.ev.ctrl ? 1000.0 : 100.0);
    double nv = v;
    if (hi > lo && slider_key([&](const char *k) { return m.key(id, k); }, v, lo, hi, step, nv)) {
      nv = std::clamp(nv, lo, hi);
      m.inflight.erase(id);
      e = {nv != current, nv != current, nv};
      v = nv;
    } else {
      const fx::Box &g = m.boxes[gid];
      const bool on_gauge = m.press_id == id && inside(g, m.ptr_x, m.ptr_y);
      if ((on_gauge || (m.active == id && m.moved && m.inflight.count(id))) && hi > lo) {
        const double t =
            std::clamp(double(m.ptr_x - g.x_min) / std::max(1, g.x_max - g.x_min), 0.0, 1.0);
        v = lo + t * (hi - lo);
        m.inflight[id] = v;
        e.changed = v != current;
        e.value = v;
      }
      if (m.release_id == id && m.inflight.count(id)) {
        e.value = m.inflight[id];
        e.changed = e.committed = true;
        m.inflight.erase(id);
        v = e.value;
      }
    }
  }
  const float p = hi > lo ? std::clamp(float((v - lo) / (hi - lo)), 0.f, 1.f) : 0.f;
  char buf[64];
  std::snprintf(buf, sizeof(buf), fmt && *fmt ? fmt : "%.3f", v);
  fx::Element name = fx::text(std::string(label) + " ");
  if (m.has_focus(id))
    name = name | fx::inverted;
  m.leaf(id, true,
         fx::hbox({name,
                   fx::gauge(p) | fx::size(fx::WIDTH, fx::EQUAL, kGaugeWidth) |
                       fx::reflect(m.boxes[gid]),
                   fx::text(std::string(" ") + buf)}));
  return e;
}

IndexEdit FtxuiBackend::combo(const char *label, int current_index,
                              const std::vector<std::string> &options) {
  Impl &m = *m_;
  const std::string id = m.uid(std::string("co|") + label);
  const int n = static_cast<int>(options.size());
  IndexEdit e;
  const int cur = n == 0 ? -1 : std::clamp(current_index, 0, n - 1);
  int shown = cur;
  if (n > 0 && !m.is_disabled()) {
    if (m.open_combo == id) {
      if (m.combo_pick >= 0 && m.combo_pick < n) {
        e = {true, true, m.combo_pick};
        shown = m.combo_pick;
        m.open_combo.clear();
        m.combo_pick = -1;
      }
    } else if (m.activated(id)) {
      m.open_combo = id;
      m.combo_hi = std::max(0, cur);
    } else if (m.key(id, "Left")) {
      shown = (cur - 1 + n) % n;
      e = {true, true, shown};
    } else if (m.key(id, "Right")) {
      shown = (cur + 1) % n;
      e = {true, true, shown};
    }
  }
  fx::Element value = fx::text("< " + (shown >= 0 ? options[shown] : std::string()) + " >");
  if (m.has_focus(id))
    value = value | fx::inverted;
  m.leaf(id, true, fx::hbox({fx::text(std::string(label) + ": "), value}));
  if (m.open_combo == id && n > 0) {
    fx::Elements rows;
    for (int i = 0; i < n; ++i) {
      fx::Element row = fx::text(" " + options[static_cast<std::size_t>(i)] + " ");
      if (i == m.combo_hi)
        row = row | fx::inverted;
      rows.push_back(row | fx::reflect(m.boxes["combo|" + id + "#" + std::to_string(i)]));
    }
    const fx::Box &anchor = m.boxes[id];
    int y = anchor.y_max + 1;
    if (y + n + 2 > m.term_h)
      y = std::max(0, anchor.y_min - n - 2); // no room below: open upward
    m.popups.push_back(
        place(isolated(fx::vbox(std::move(rows)) | fx::border | fx::clear_under), anchor.x_min, y));
    m.walk.combo_popup = id;
    m.walk.combo_n = n;
  }
  return e;
}

StringEdit FtxuiBackend::input_text(const char *label, const char *key,
                                    const std::string &current) {
  Impl &m = *m_;
  const std::string id = m.uid(std::string("in|") + (key ? key : label));
  StringEdit e;
  e.value = current;
  auto it = m.edits.find(id);
  auto edit = [&]() -> Impl::Edit & {
    if (it == m.edits.end())
      it = m.edits.emplace(id, Impl::Edit{current, current.size()}).first;
    return it->second;
  };
  if (m.blur_commit.erase(id)) {
    if (it != m.edits.end()) { // focus left with an edit pending: commit it
      e.value = it->second.text;
      e.changed = e.value != current;
      e.committed = true;
      m.edits.erase(it);
      it = m.edits.end();
    }
  } else if (m.ev_live && m.has_focus(id) && m.ev.kind != TermEvent::Kind::Mouse) {
    const TermEvent &ev = m.ev;
    bool used = true;
    if (ev.kind == TermEvent::Kind::Text && !ev.ctrl && !ev.alt) {
      Impl::Edit &ed = edit();
      ed.text.insert(ed.cursor, ev.text);
      ed.cursor += ev.text.size();
    } else if (ev.key == "Backspace") {
      Impl::Edit &ed = edit();
      if (ed.cursor > 0) {
        const std::size_t p = utf8_prev(ed.text, ed.cursor);
        ed.text.erase(p, ed.cursor - p);
        ed.cursor = p;
      }
    } else if (ev.key == "Delete") {
      Impl::Edit &ed = edit();
      ed.text.erase(ed.cursor, utf8_next(ed.text, ed.cursor) - ed.cursor);
    } else if (ev.key == "Left") {
      Impl::Edit &ed = edit();
      ed.cursor = utf8_prev(ed.text, ed.cursor);
    } else if (ev.key == "Right") {
      Impl::Edit &ed = edit();
      ed.cursor = utf8_next(ed.text, ed.cursor);
    } else if (ev.key == "Home") {
      edit().cursor = 0;
    } else if (ev.key == "End") {
      Impl::Edit &ed = edit();
      ed.cursor = ed.text.size();
    } else if (ev.key == "Return") {
      // Enter commits even when nothing changed (a chat box's send), as the ImGui backend does.
      e.value = it != m.edits.end() ? it->second.text : current;
      e.changed = e.value != current;
      e.committed = true;
      if (it != m.edits.end()) {
        m.edits.erase(it);
        it = m.edits.end();
      }
    } else if (ev.key == "Escape" && it != m.edits.end()) {
      m.edits.erase(it); // revert to the value in state
      it = m.edits.end();
    } else {
      used = false;
    }
    if (used)
      m.use();
  }
  if (it != m.edits.end() && !e.committed) {
    e.value = it->second.text;
    e.changed = e.value != current;
  }
  // "label: [text]", with the cursor drawn as an inverted cell while focused.
  fx::Elements row{fx::text(std::string(label) + ": [")};
  if (m.has_focus(id)) {
    const std::string &t = it != m.edits.end() ? it->second.text : e.value;
    const std::size_t c = it != m.edits.end() ? it->second.cursor : t.size();
    const std::size_t c2 = utf8_next(t, c);
    row.push_back(fx::text(t.substr(0, c)));
    row.push_back(fx::text(c < t.size() ? t.substr(c, c2 - c) : std::string(" ")) | fx::inverted);
    row.push_back(fx::text(c2 < t.size() ? t.substr(c2) : std::string()));
  } else {
    row.push_back(fx::text(e.value));
  }
  row.push_back(fx::text("]"));
  m.leaf(id, true, fx::hbox(std::move(row)));
  return e;
}

ColorEdit FtxuiBackend::color(const char *label, const float rgb[3]) {
  Impl &m = *m_;
  const std::string id = m.uid(std::string("cl|") + label);
  ColorEdit e;
  e.drawn = true;
  for (int i = 0; i < 3; ++i)
    e.rgb[i] = rgb[i];
  // Space / Enter picks the channel, Left / Right moves it (Ctrl for a fine step).
  int &chan = m.color_chan[id];
  if (m.key(id, "Space") || m.key(id, "Return"))
    chan = (chan + 1) % 3;
  const float step = m.ev.ctrl ? 1.0f / 255.0f : 1.0f / 32.0f;
  if (m.key(id, "Left") || m.key(id, "Right")) {
    const float v = std::clamp(e.rgb[chan] + (m.ev.key == "Left" ? -step : step), 0.0f, 1.0f);
    if (v != e.rgb[chan]) {
      e.rgb[chan] = v;
      e.changed = e.committed = true;
    }
  }
  const bool f = m.has_focus(id);
  fx::Elements row{fx::text(std::string(label) + ": "),
                   fx::text("████") | fx::color(rgb_color(e.rgb)), fx::text(" ")};
  static const char kChan[3] = {'R', 'G', 'B'};
  for (int i = 0; i < 3; ++i) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%c %.2f", kChan[i], e.rgb[i]);
    fx::Element c = fx::text(buf);
    if (f && i == chan)
      c = c | fx::inverted;
    row.push_back(c);
    row.push_back(fx::text(" "));
  }
  m.leaf(id, true, fx::hbox(std::move(row)));
  return e;
}

// ---- images
// --------------------------------------------------------------------------------------

void FtxuiBackend::set_image(const std::string &name, const cvc::image &img) {
  Impl::Picture &p = m_->images[name];
  p.rgb = img.empty() ? cvc::image()
                      : img.converted(cvc::image::pixel_format::RGB, cvc::image::data_type::u8);
  p.scaled = cvc::image();
}

void FtxuiBackend::clear_image(const std::string &name) { m_->images.erase(name); }

bool FtxuiBackend::draw_image(const char *name, float width) {
  Impl &m = *m_;
  const auto it = m.images.find(name ? name : "");
  if (it == m.images.end() || it->second.rgb.empty())
    return false;
  Impl::Picture &p = it->second;
  // `width` px -> columns; rows keep the image's aspect given the cell's (each cell is two pixels
  // tall, so a cell_px_w x cell_px_h cell shows a 1 x 2 block of square pixels at the right scale).
  const int cols = std::clamp(m.px_to_cols(width), 1, std::max(1, m.term_w - 2));
  const double aspect = double(p.rgb.height()) / double(p.rgb.width());
  const int rows =
      std::max(1, static_cast<int>(std::lround(cols * aspect * m.opt.cell_px_w / m.opt.cell_px_h)));
  if (p.scaled.width() != cols || p.scaled.height() != 2 * rows)
    p.scaled = p.rgb.resized(cols, 2 * rows, cvc::image::resize_filter::box);
  m.leaf(m.uid("img|"), false,
         std::make_shared<HalfBlockImage>(p.scaled, cols, rows, m.opt.image_color));
  return true;
}

// ---- headless driving
// ----------------------------------------------------------------------------

std::string FtxuiBackend::render_to_string(Runtime &rt, int width, int height, bool ansi) {
  Impl &m = *m_;
  m.term_w = std::max(1, width);
  m.term_h = height > 0 ? height : 24; // percent sizes resolve against a nominal 24 rows
  rt.render(); // walk the tree -> (re)builds m.root through the methods above
  fx::Screen screen =
      fx::Screen::Create(fx::Dimension::Fixed(m.term_w),
                         height > 0 ? fx::Dimension::Fixed(height) : fx::Dimension::Fit(m.root));
  fx::Render(screen, m.root);
  m.shown = m.last; // the boxes now hold this frame's layout
  if (ansi)
    return screen.ToString();
  std::string out;
  for (int y = 0; y < screen.dimy(); ++y) {
    std::string line;
    for (int x = 0; x < screen.dimx(); ++x) {
      const std::string &c = screen.CellAt(x, y).character;
      line += c.empty() ? std::string(" ") : c;
    }
    const std::size_t end = line.find_last_not_of(' ');
    out += end == std::string::npos ? std::string() : line.substr(0, end + 1);
    out += '\n';
  }
  return out;
}

bool FtxuiBackend::feed(Runtime &rt, const TermEvent &ev) {
  Impl &m = *m_;
  m.ev = ev;
  m.ev_live = ev.kind != TermEvent::Kind::Mouse;
  m.consumed = false;
  m.route(ev);
  rt.render(); // the walk in which the target widget reports its edit
  // A key no widget took: Up / Down fall back to moving focus.
  if (m.ev_live && !ev.ctrl && !ev.alt && (ev.key == "Up" || ev.key == "Down")) {
    m.focus_step(ev.key == "Up" ? -1 : 1);
    m.use();
  }
  const bool used = m.consumed;
  m.end_event();
  return used;
}

// ---- the interactive loop
// ------------------------------------------------------------------------

namespace {

// FTXUI event -> TermEvent. False for events that are not input (the idle redraw tick, terminal
// capability reports).
bool translate(fx::Event e, TermEvent &out) {
  if (e == fx::Event::Custom)
    return false;
  if (e.is_mouse()) {
    const fx::Mouse mo = e.mouse();
    out = TermEvent{};
    out.kind = TermEvent::Kind::Mouse;
    out.x = mo.x;
    out.y = mo.y;
    out.shift = mo.shift;
    out.ctrl = mo.control;
    out.alt = mo.meta;
    switch (mo.button) {
    case fx::Mouse::Left:
      out.button = 1;
      break;
    case fx::Mouse::Middle:
      out.button = 2;
      break;
    case fx::Mouse::Right:
      out.button = 3;
      break;
    case fx::Mouse::None:
      out.button = 0;
      break;
    case fx::Mouse::WheelUp:
      out.mouse = TermEvent::Mouse::WheelUp;
      return true;
    case fx::Mouse::WheelDown:
      out.mouse = TermEvent::Mouse::WheelDown;
      return true;
    default:
      return false;
    }
    out.mouse = mo.motion == fx::Mouse::Pressed    ? TermEvent::Mouse::Press
                : mo.motion == fx::Mouse::Released ? TermEvent::Mouse::Release
                                                   : TermEvent::Mouse::Move;
    return true;
  }
  if (e.is_character()) {
    out = TermEvent::text_event(e.character());
    return true;
  }
  struct Special {
    const fx::Event *ev;
    const char *name;
    bool shift, ctrl;
  };
  static const Special kSpecials[] = {
      {&fx::Event::Tab, "Tab", false, false},
      {&fx::Event::TabReverse, "Tab", true, false},
      {&fx::Event::Return, "Return", false, false},
      {&fx::Event::Escape, "Escape", false, false},
      {&fx::Event::Backspace, "Backspace", false, false},
      {&fx::Event::Delete, "Delete", false, false},
      {&fx::Event::Insert, "Insert", false, false},
      {&fx::Event::ArrowLeft, "Left", false, false},
      {&fx::Event::ArrowRight, "Right", false, false},
      {&fx::Event::ArrowUp, "Up", false, false},
      {&fx::Event::ArrowDown, "Down", false, false},
      {&fx::Event::ArrowLeftCtrl, "Left", false, true},
      {&fx::Event::ArrowRightCtrl, "Right", false, true},
      {&fx::Event::ArrowUpCtrl, "Up", false, true},
      {&fx::Event::ArrowDownCtrl, "Down", false, true},
      {&fx::Event::Home, "Home", false, false},
      {&fx::Event::End, "End", false, false},
      {&fx::Event::PageUp, "PageUp", false, false},
      {&fx::Event::PageDown, "PageDown", false, false},
      {&fx::Event::F1, "F1", false, false},
      {&fx::Event::F2, "F2", false, false},
      {&fx::Event::F3, "F3", false, false},
      {&fx::Event::F4, "F4", false, false},
      {&fx::Event::F5, "F5", false, false},
      {&fx::Event::F6, "F6", false, false},
      {&fx::Event::F7, "F7", false, false},
      {&fx::Event::F8, "F8", false, false},
      {&fx::Event::F9, "F9", false, false},
      {&fx::Event::F10, "F10", false, false},
      {&fx::Event::F11, "F11", false, false},
      {&fx::Event::F12, "F12", false, false},
  };
  for (const Special &s : kSpecials)
    if (e == *s.ev) {
      out = TermEvent::key_event(s.name, s.shift, s.ctrl);
      return true;
    }
  const std::string &in = e.input();
  if (in.size() == 1 && in[0] >= 1 && in[0] <= 26) { // Ctrl-A .. Ctrl-Z
    out = TermEvent::key_event(std::string(1, static_cast<char>('A' + in[0] - 1)), false, true);
    return true;
  }
  if (in.size() == 2 && in[0] == 27 && in[1] > 32 && in[1] < 127) { // Alt-<key>
    char c = in[1];
    if (c >= 'a' && c <= 'z')
      c = static_cast<char>(c - 'a' + 'A');
    out = TermEvent::key_event(std::string(1, c), false, false, true);
    return true;
  }
  return false;
}

// Wait out the rest of a frame. In the browser the loop runs on the page's main thread, so the wait
// has to hand control back to the event loop — that is when xterm.js delivers input and paints.
// emscripten_sleep does under Asyncify (link the executable with -sASYNCIFY); sleep_for would spin
// and freeze the page.
void wait_frame(std::chrono::milliseconds period) {
#ifdef __EMSCRIPTEN__
  emscripten_sleep(static_cast<unsigned>(period.count()));
#else
  std::this_thread::sleep_for(period);
#endif
}

// Whether Ctrl-Z suspends (FTXUI's own SIGTSTP handling). A browser page has no job control to
// suspend to, so there the key is swallowed.
#ifdef __EMSCRIPTEN__
constexpr bool kCanSuspend = false;
#else
constexpr bool kCanSuspend = true;
#endif

} // namespace

FtxuiBackend::RunExit
FtxuiBackend::run_loop(Runtime &rt, const RunOptions &opts, const std::function<void()> &drain,
                       const std::function<bool()> &should_close,
                       const std::function<void(const TermEvent &)> &unhandled) {
  Impl &m = *m_;
  RunExit result;
  // Interrupt keys are signals: Ctrl-C is SIGINT, Ctrl-\ SIGQUIT. A document with an on_signal:
  // program gets them and decides; without one they leave the loop. Three Ctrl-Cs inside two
  // seconds always leave, so a handler that never quits can't trap the user.
  using clock = std::chrono::steady_clock;
  clock::time_point burst_start{};
  int burst = 0;
  auto interrupt = [&](const char *signal) {
    const clock::time_point now = clock::now();
    if (now - burst_start > std::chrono::seconds(2)) {
      burst_start = now;
      burst = 0;
    }
    return ++burst >= 3 || !rt.post_signal(signal, "terminal");
  };
  fx::ScreenInteractive screen = fx::ScreenInteractive::Fullscreen();
  screen.TrackMouse(opts.mouse);
  // Ctrl-C is handled here (see `interrupt`), so it returns to the host. FTXUI's own handling
  // re-raises SIGINT, which would end the whole process under a host that embeds this loop.
  screen.ForceHandleCtrlC(false);
  fx::Component renderer = fx::Renderer([&] {
    const fx::Dimensions d = fx::Terminal::Size();
    m.term_w = std::max(1, d.dimx);
    m.term_h = std::max(1, d.dimy);
    rt.render();
    m.shown = m.last; // FTXUI lays this frame out next, which fills the boxes
    return m.root;
  });
  fx::Component app = fx::CatchEvent(renderer, [&](fx::Event e) {
    if (e == fx::Event::CtrlC || e.input() == "\x1c") {
      const char *signal = e == fx::Event::CtrlC ? "SIGINT" : "SIGQUIT";
      if (interrupt(signal)) {
        result.signal = signal;
        screen.Exit();
      }
      return true;
    }
    if (e == fx::Event::CtrlZ)
      return !kCanSuspend; // FTXUI's own suspend, where there is one
    TermEvent te;
    if (!translate(e, te))
      return false;
    if (!feed(rt, te) && unhandled)
      unhandled(te);
    return true;
  });
  fx::Loop loop(&screen, app); // sets the terminal up (and FTXUI's signal handlers)
  if (opts.on_start)
    opts.on_start();
  const auto period = std::chrono::milliseconds(1000 / std::max(1, opts.fps));
  while (!loop.HasQuitted()) {
    loop.RunOnce();
    // OS signals the host collected: the document's on_signal: decides, else they end the loop.
    if (opts.take_signal)
      for (std::string sig = opts.take_signal(); !sig.empty(); sig = opts.take_signal())
        if (!rt.post_signal(sig, "os") && result.signal.empty()) {
          result = RunExit{sig, true};
          screen.Exit();
        }
    if (drain)
      drain(); // queued actions run here, off the walk (§7.2)
    if (should_close && should_close())
      screen.Exit();
    if (loop.HasQuitted())
      break;
    screen.PostEvent(fx::Event::Custom); // redraw, so state changed off the UI shows
    wait_frame(period);
  }
  if (!loop.HasQuitted())
    loop.RunOnce(); // let a pending Exit() take effect before the terminal is restored
  if (opts.on_stop)
    opts.on_stop();
  return result;
}

FtxuiBackend::RunExit FtxuiBackend::run(Runtime &rt) { return run(rt, RunOptions{}); }

FtxuiBackend::RunExit FtxuiBackend::run(Runtime &rt, const RunOptions &opts) {
  return run_loop(rt, opts, [&] { rt.drain(); }, nullptr, nullptr);
}

FtxuiBackend::RunExit FtxuiBackend::run(AppRuntime &app) { return run(app, RunOptions{}); }

FtxuiBackend::RunExit FtxuiBackend::run(AppRuntime &app, const RunOptions &opts) {
  const int cw = m_->opt.cell_px_w, ch = m_->opt.cell_px_h;
  // What the UI didn't use goes to the document's on_key / on_pointer, keys with SDL's modifier
  // bits (shift 0x1, ctrl 0x40, alt 0x100) and a press + release pair (a terminal reports no
  // key-up).
  auto unhandled = [&app, cw, ch](const TermEvent &e) {
    if (e.kind != TermEvent::Kind::Mouse) {
      if (e.key.empty())
        return;
      const int mods = (e.shift ? 0x1 : 0) | (e.ctrl ? 0x40 : 0) | (e.alt ? 0x100 : 0);
      app.post_key(true, e.key, mods);
      app.post_key(false, e.key, mods);
      return;
    }
    const double x = e.x * cw + cw / 2.0, y = e.y * ch + ch / 2.0;
    switch (e.mouse) {
    case TermEvent::Mouse::Press:
      app.post_pointer(static_cast<int>(InputEvent::Kind::MouseButtonDown), x, y, 0, 0, e.button,
                       1);
      break;
    case TermEvent::Mouse::Release:
      app.post_pointer(static_cast<int>(InputEvent::Kind::MouseButtonUp), x, y, 0, 0, e.button, 1);
      break;
    case TermEvent::Mouse::Move:
      app.post_pointer(static_cast<int>(InputEvent::Kind::MouseMove), x, y, 0, 0, e.button, 0);
      break;
    case TermEvent::Mouse::WheelUp:
    case TermEvent::Mouse::WheelDown:
      app.post_pointer(static_cast<int>(InputEvent::Kind::MouseWheel), x, y, 0,
                       e.mouse == TermEvent::Mouse::WheelUp ? 1.0 : -1.0, 0, 0);
      break;
    }
  };
  return run_loop(
      app.runtime(), opts, [&] { app.drain(); }, [&] { return app.should_close(); }, unhandled);
}

bool FtxuiBackend::available() { return true; }

#else // !CVC_ARIADNE_FTXUI — inert stub so libcvc builds without FTXUI.

struct FtxuiBackend::Impl {};
FtxuiBackend::FtxuiBackend() : m_(new Impl) {}
FtxuiBackend::FtxuiBackend(const Options &) : m_(new Impl) {}
FtxuiBackend::~FtxuiBackend() = default;
Capabilities FtxuiBackend::capabilities() const { return {}; }
void FtxuiBackend::begin_frame() {}
void FtxuiBackend::end_frame() {}
bool FtxuiBackend::begin_main_menu_bar() { return false; }
void FtxuiBackend::end_main_menu_bar() {}
bool FtxuiBackend::begin_menu(const char *) { return false; }
void FtxuiBackend::end_menu() {}
bool FtxuiBackend::begin_window(const char *, const char *, const Size &, float, bool *) {
  return false;
}
void FtxuiBackend::end_window() {}
void FtxuiBackend::seed_window_geometry(const WindowGeom &) {}
WindowGeom FtxuiBackend::window_geometry() const { return {}; }
void FtxuiBackend::push_id(const char *) {}
void FtxuiBackend::pop_id() {}
bool FtxuiBackend::begin_grid(const Layout &, const char *) { return false; }
void FtxuiBackend::grid_next_cell() {}
void FtxuiBackend::end_grid() {}
void FtxuiBackend::seed_grid_tracks(const std::vector<float> &) {}
std::vector<float> FtxuiBackend::grid_tracks() const { return {}; }
void FtxuiBackend::begin_disabled() {}
void FtxuiBackend::end_disabled() {}
void FtxuiBackend::set_tooltip(const char *) {}
void FtxuiBackend::text_line(const char *) {}
void FtxuiBackend::text_value(const char *, const std::string &) {}
void FtxuiBackend::text_line_colored(const char *, const float[3]) {}
void FtxuiBackend::text_value_colored(const char *, const std::string &, const float[3]) {}
void FtxuiBackend::separator() {}
bool FtxuiBackend::button(const char *) { return false; }
bool FtxuiBackend::menu_item_action(const char *) { return false; }
bool FtxuiBackend::item_clicked() { return false; }
bool FtxuiBackend::item_hovered() { return false; }
bool FtxuiBackend::item_dragged() { return false; }
bool FtxuiBackend::item_drag_started() { return false; }
bool FtxuiBackend::item_drag_ended() { return false; }
bool FtxuiBackend::item_pointer(float &x, float &y, float &dx, float &dy, int &button) {
  x = y = dx = dy = 0.0f;
  button = 0;
  return false;
}
BoolEdit FtxuiBackend::menu_item_toggle(const char *, bool) { return {}; }
BoolEdit FtxuiBackend::checkbox(const char *, bool) { return {}; }
IntEdit FtxuiBackend::slider_int(const char *, const char *, int, int, int) { return {}; }
DoubleEdit FtxuiBackend::slider_double(const char *, const char *, double, double, double,
                                       const char *) {
  return {};
}
IndexEdit FtxuiBackend::combo(const char *, int, const std::vector<std::string> &) { return {}; }
StringEdit FtxuiBackend::input_text(const char *, const char *, const std::string &) { return {}; }
bool FtxuiBackend::draw_image(const char *, float) { return false; }
ColorEdit FtxuiBackend::color(const char *, const float[3]) { return {}; }
void FtxuiBackend::set_image(const std::string &, const cvc::image &) {}
void FtxuiBackend::clear_image(const std::string &) {}
std::string FtxuiBackend::render_to_string(Runtime &, int, int, bool) {
  return "ari: libcvc was built without FTXUI (terminal backend unavailable)\n";
}
bool FtxuiBackend::feed(Runtime &, const TermEvent &) { return false; }
FtxuiBackend::RunExit FtxuiBackend::run_loop(Runtime &, const RunOptions &,
                                             const std::function<void()> &,
                                             const std::function<bool()> &,
                                             const std::function<void(const TermEvent &)> &) {
  return {};
}
FtxuiBackend::RunExit FtxuiBackend::run(Runtime &) { return {}; }
FtxuiBackend::RunExit FtxuiBackend::run(Runtime &, const RunOptions &) { return {}; }
FtxuiBackend::RunExit FtxuiBackend::run(AppRuntime &) { return {}; }
FtxuiBackend::RunExit FtxuiBackend::run(AppRuntime &, const RunOptions &) { return {}; }
bool FtxuiBackend::available() { return false; }

#endif

} // namespace ariadne
} // namespace cvc
