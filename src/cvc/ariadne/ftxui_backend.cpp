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
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/ftxui_backend.h>
#include <cvc/ariadne/input.h> // InputEvent::Kind, for forwarding pointer events
#include <cvc/image/image.h>
#include <iostream>
#include <map>
#include <memory>
#include <random>
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
#include <ftxui/screen/string.hpp> // string_width
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

using ImageMode = FtxuiBackend::Options::ImageColor;

// The UTF-8 encoding of U+10EEEE, the kitty placeholder's base character.
constexpr char kPlaceholderBase[] = "\xF4\x8E\xBB\xAE";

// A block of a Kitty image's cells shown by one placement. A placement addresses at most
// gfx::kKittyMaxCells rows and columns (one diacritic each), so a bigger image goes as tiles.
struct KittyTile {
  std::string key;                        // what the backend records it sent under
  std::uint32_t id = 0;                   // the image id the placeholders' colour names
  int c0 = 0, r0 = 0, cols = 0, rows = 0; // its cells within the image
  int px = 0, py = 0, pw = 0, ph = 0;     // its pixels within the raster
};

// An image drawn with pixel graphics in one walk: what its node draws and, once the frame is laid
// out, where it went (and, for Sixel, which of its cells still show it).
struct PixelGfx {
  std::string key; // stable across frames: the leaf id, or "bg|" for the backdrop
  ImageMode mode = ImageMode::Kitty;
  cvc::image raster; // the pixels to send, RGB8
  cvc::image cells;  // Kitty: one pixel per cell (the cells' colour); Sixel: two (the underlay)
  long gen = 0;      // the published picture's generation, for "unchanged since it was sent"
  int cols = 0, rows = 0;
  int cell_w = 0, cell_h = 0; // the raster's pixels per cell (Sixel)
  std::vector<KittyTile> tiles;
  bool drawn = false;
  fx::Box box{};                      // where its node was laid out (may reach past the screen)
  std::vector<unsigned char> visible; // Sixel: cols x rows, the cells nothing covers
  int screen_h = 0;

  const KittyTile &tile(int c, int r) const {
    const int k = gfx::kKittyMaxCells, across = (cols + k - 1) / k;
    return tiles[static_cast<std::size_t>((r / k) * across + c / k)];
  }
  const unsigned char *cell_rgb(int c, int r) const { // Kitty: cell (c, r); Sixel: half-row r
    return std::as_const(cells).data() + (static_cast<std::size_t>(r) * cells.width() + c) * 3;
  }
};

fx::Color id_color(std::uint32_t id) {
  return fx::Color::RGB((id >> 16) & 0xFF, (id >> 8) & 0xFF, id & 0xFF);
}

// An image in pixel graphics. Kitty: placeholder cells, which the terminal covers with the image
// the backend sends out of band, each cell's background its average colour (what shows until the
// image arrives). Sixel: the half-block picture, which the bitmap is drawn over after the frame.
class PixelImage : public fx::Node {
public:
  explicit PixelImage(std::shared_ptr<PixelGfx> g) : g_(std::move(g)) {}

  void ComputeRequirement() override {
    requirement_.min_x = g_->cols;
    requirement_.min_y = g_->rows;
  }

  void Render(fx::Screen &screen) override {
    PixelGfx &g = *g_;
    g.drawn = true;
    g.box = box_;
    for (int r = 0; r < g.rows && box_.y_min + r <= box_.y_max; ++r)
      for (int c = 0; c < g.cols && box_.x_min + c <= box_.x_max; ++c) {
        fx::Cell &cell = screen.CellAt(box_.x_min + c, box_.y_min + r); // clipped by the stencil
        cell = fx::Cell();
        if (g.mode == ImageMode::Kitty) {
          const KittyTile &t = g.tile(c, r);
          const unsigned char *p = g.cell_rgb(c, r);
          cell.character = gfx::kitty_placeholder(r - t.r0, c - t.c0);
          cell.foreground_color = id_color(t.id);
          cell.background_color = fx::Color::RGB(p[0], p[1], p[2]);
        } else {
          const unsigned char *top = g.cell_rgb(c, 2 * r), *bot = g.cell_rgb(c, 2 * r + 1);
          cell.character = "\xe2\x96\x80"; // ▀
          cell.foreground_color = fx::Color::RGB(top[0], top[1], top[2]);
          cell.background_color = fx::Color::RGB(bot[0], bot[1], bot[2]);
        }
      }
  }

private:
  std::shared_ptr<PixelGfx> g_;
};

// Drawn last, over the whole frame: settles what the layers above each pixel image did to it.
//   - Kitty: text drawn over placeholder cells keeps their foreground, the colour that names the
//     image, so it gets a readable one; and an inverted placeholder (a menu bar's filler) would
//     still show the image, so it becomes a blank.
//   - Sixel: records which cells still show the underlay, so the bitmap skips covered cells; text
//     and inverted cells over the underlay are settled as for Kitty.
class PixelFinish : public fx::Node {
public:
  explicit PixelFinish(std::vector<std::shared_ptr<PixelGfx>> list) : list_(std::move(list)) {}

  void ComputeRequirement() override { requirement_ = fx::Requirement{}; }

  void Render(fx::Screen &screen) override {
    for (const std::shared_ptr<PixelGfx> &gp : list_) {
      PixelGfx &g = *gp;
      if (!g.drawn)
        continue;
      g.screen_h = screen.dimy();
      if (g.mode != ImageMode::Kitty) // Sixel / Iterm: a bitmap, drawn where nothing covers it
        g.visible.assign(static_cast<std::size_t>(g.cols) * g.rows, 0);
      for (int r = 0; r < g.rows; ++r) {
        const int y = g.box.y_min + r;
        if (y < 0 || y > g.box.y_max || y >= screen.dimy())
          continue;
        for (int c = 0; c < g.cols; ++c) {
          const int x = g.box.x_min + c;
          if (x < 0 || x > g.box.x_max || x >= screen.dimx())
            continue;
          fx::Cell &cell = screen.CellAt(x, y);
          if (g.mode == ImageMode::Kitty) {
            const bool placeholder = cell.character.compare(0, 4, kPlaceholderBase) == 0;
            if (placeholder && !cell.inverted)
              continue;
            const fx::Color id = id_color(g.tile(c, r).id);
            if (placeholder)
              cell.character = " ";
            if (cell.foreground_color == id) {
              const unsigned char *p = g.cell_rgb(c, r);
              const int lum = (p[0] * 3 + p[1] * 6 + p[2]) / 10;
              cell.foreground_color = lum > 140 ? fx::Color::Black : fx::Color::White;
            }
          } else {
            const unsigned char *top = g.cell_rgb(c, 2 * r), *bot = g.cell_rgb(c, 2 * r + 1);
            const fx::Color top_c = fx::Color::RGB(top[0], top[1], top[2]);
            const bool half = cell.character == "\xe2\x96\x80";
            g.visible[static_cast<std::size_t>(r) * g.cols + c] =
                half && !cell.inverted && cell.foreground_color == top_c &&
                cell.background_color == fx::Color::RGB(bot[0], bot[1], bot[2]);
            // Text over the underlay, and an inverted underlay cell (a menu bar's filler), as for
            // Kitty.
            if (cell.foreground_color == top_c && (!half || cell.inverted)) {
              if (half)
                cell.character = " ";
              const int lum = (bot[0] * 3 + bot[1] * 6 + bot[2]) / 10;
              cell.foreground_color = lum > 140 ? fx::Color::Black : fx::Color::White;
            }
          }
        }
      }
    }
  }

private:
  std::vector<std::shared_ptr<PixelGfx>> list_;
};

// A hash of a pixel block, for "the frame published again is the frame already sent".
std::uint64_t hash_pixels(const unsigned char *p, int w, int h, std::size_t stride) {
  std::uint64_t x = 0x9E3779B97F4A7C15ull ^ ((std::uint64_t(w) << 32) | std::uint32_t(h));
  const std::size_t row = static_cast<std::size_t>(w) * 3;
  for (int y = 0; y < h; ++y) {
    const unsigned char *s = p + static_cast<std::size_t>(y) * stride;
    std::size_t i = 0;
    for (; i + 8 <= row; i += 8) {
      std::uint64_t v;
      std::memcpy(&v, s + i, 8);
      x = (x ^ v) * 0x100000001B3ull;
      x ^= x >> 29;
    }
    for (; i < row; ++i)
      x = (x ^ s[i]) * 0x100000001B3ull;
  }
  return x;
}

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
    std::vector<float> columns;      // Grid: seeded table column widths (px)
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
    // Hosted chrome (Options::hosted): each window's body ("" = outside every window) and the menu
    // bar's model, for the host to draw.
    struct Body {
      std::string title;
      bool closable = false;
      fx::Element body;
      fx::Elements popups;
    };
    std::map<std::string, Body> bodies;
    std::vector<FtxuiBackend::HostedMenu> menus;
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
  std::vector<float> pending_columns;
  bool have_columns = false;
  std::vector<float> last_columns;
  int term_w = 80, term_h = 24;

  // ---- across frames ---------------------------------------------------------------------------

  struct Win {
    int x = 0, y = 0; // top-left cell, as placed (by the document, the persisted geometry, a drag)
    int sx = 0, sy = 0; // where it is drawn this frame: x / y kept on the current screen
    int w = 0, h = 0;   // outer size in cells when `sized`
    bool sized = false;
    bool placed = false;
    bool seeded = false;
    long z = 0;
  };
  struct Edit {
    std::string text;
    std::size_t cursor = 0; // byte offset, on a UTF-8 boundary
  };
  // A pixel-mode image's resampled pixels, kept while the picture and its cells stay the same.
  struct PixCache {
    cvc::image raster; // at the size image_raster gives
    cvc::image cells;  // raster per cell (PixelGfx::cells)
    long gen = -1;
  };
  struct Picture {
    cvc::image rgb;    // the published image as RGB8
    cvc::image scaled; // rgb resampled to the last drawn cell grid
    long gen = 0;      // bumped by each set_image
    PixCache px;       // the pixel modes' resampling
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
  bool focus_next = false; // autofocus: the next focusable leaf of this walk takes focus
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
  std::map<std::string, fx::Elements> win_popups; // hosted: popups by owning window, this walk
  std::map<std::string, int> body_h;              // hosted: each body's last rendered height
  std::string host_window;                        // hosted: the window the current input is for
  long walks = 0;               // walks finished (hosted menus join from the second)
  std::string background;       // the published image drawn behind everything ("" = none)
  cvc::image background_scaled; // that image resampled to the screen's cells
  long z_top = 0;
  std::unordered_map<std::string, fx::Box> boxes; // node-based: reflect() keeps references
  std::map<std::string, Picture> images;
  long image_gen = 0;

  // ---- pixel graphics (Kitty / Sixel) ----------------------------------------------------------

  TermGraphics term_gfx; // what the terminal draws (probed by run, or set by the host)
  bool term_gfx_set = false;
  PixCache background_px;                           // the backdrop's resampling
  std::vector<std::shared_ptr<PixelGfx>> gfx_walk;  // pixel images of the walk being built
  std::vector<std::shared_ptr<PixelGfx>> gfx_last;  // ... of the last walk finished
  std::vector<std::shared_ptr<PixelGfx>> gfx_shown; // ... of the frame last drawn
  bool drew = false; // run(): the renderer laid a frame out this iteration
  // What the terminal holds, per Kitty tile key: the id its placeholders name and what was sent.
  // A Kitty tile's images in the terminal, double-buffered. Re-sending a picture under the id the
  // placeholders name makes the terminal drop the shown image for the whole transfer, so a picture
  // that changes every frame (an animated scene behind the widgets) never shows at all: the cells
  // only ever get their average colour. So a changed picture goes to the OTHER id, which nothing
  // names yet, and the next layout switches the placeholders to it (kitty_id), by which time it
  // has fully arrived; the picture after that reuses the first id. The first send, and a change of
  // size, go to the id the placeholders already name.
  struct KittySent {
    std::uint32_t ids[2] = {0, 0};
    int shown = 0;                // the slot the placeholders name
    bool pending = false;         // ids[shown ^ 1] holds a newer picture: switch at the next layout
    bool has[2] = {false, false}; // the slot holds an image in the terminal
    long gen = -1;
    std::uint64_t hash = 0; // of the latest picture sent (shown, or pending)
    int w = 0, h = 0, cols = 0, rows = 0;
    long frame = 0; // the last emit that drew it
  };
  std::map<std::string, KittySent> kitty;
  std::uint32_t kitty_next = 0;
  // Encoded Sixel / iTerm2 images by image key and cell rectangle, while their picture stays the
  // same.
  struct SixelCached {
    long gen = -1;
    std::string bytes;
    long frame = 0;
    // What the bytes were cut from: the same picture (gen) laid out over other cells, resampled
    // to another raster, or cut to another size (a Sixel's last band) needs new bytes for the same
    // rectangle.
    int raster_w = 0, raster_h = 0, cols = 0, rows = 0, pw = 0, ph = 0;
    bool stale(const PixelGfx &g, int w, int h) const {
      return bytes.empty() || gen != g.gen || raster_w != g.raster.width() ||
             raster_h != g.raster.height() || cols != g.cols || rows != g.rows || pw != w ||
             ph != h;
    }
    void made_from(const PixelGfx &g, int w, int h) {
      gen = g.gen;
      raster_w = g.raster.width();
      raster_h = g.raster.height();
      cols = g.cols;
      rows = g.rows;
      pw = w;
      ph = h;
    }
  };
  std::map<std::string, SixelCached> sixel_cache;
  long gfx_frames = 0;

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
  int top_row() const { return !opt.hosted && (walk.has_menubar || last.has_menubar) ? 1 : 0; }

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
    if (it.focusable && focus_next && cur_menu.empty()) {
      focus_next = false;
      set_focus(id); // autofocus: (focus_next_widget)
    }
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

  // ---- pixel graphics --------------------------------------------------------------------------

  // Options::image_color with Auto resolved. Hosted output is tvision's, so it stays half-blocks
  // (Auto there means: colour, or the ramp on a terminal with none, decided as it draws).
  ImageMode mode() const {
    const ImageMode c = opt.image_color;
    if (opt.hosted)
      return pixel_mode(c) ? ImageMode::Auto : c;
    if (c != ImageMode::Auto)
      return c;
    if (term_gfx.kitty)
      return ImageMode::Kitty;
    if (term_gfx.sixel && term_gfx.cell_w > 0 && term_gfx.cell_h > 0)
      return ImageMode::Sixel; // a bitmap is placed in real pixels, so it needs the cell size
    if (term_gfx.iterm)
      return ImageMode::Iterm; // scaled to its cells by the terminal: no cell size needed
    return ImageMode::Auto;
  }
  static bool pixel_mode(ImageMode md) {
    return md == ImageMode::Kitty || md == ImageMode::Sixel || md == ImageMode::Iterm;
  }
  bool pixels() const { return pixel_mode(mode()); }
  int cell_w() const { return term_gfx.cell_w > 0 ? term_gfx.cell_w : opt.cell_px_w; }
  int cell_h() const { return term_gfx.cell_h > 0 ? term_gfx.cell_h : opt.cell_px_h; }

  void raster_size(int cols, int rows, ImageMode md, int &w, int &h) const {
    if (!pixel_mode(md)) {
      w = std::max(1, cols);
      h = std::max(1, 2 * rows);
      return;
    }
    w = std::max(1, cols * cell_w());
    h = std::max(1, rows * cell_h());
    const double px = double(w) * double(h);
    if (md != ImageMode::Sixel && opt.max_image_pixels > 0 && px > opt.max_image_pixels) {
      const double k = std::sqrt(opt.max_image_pixels / px); // the terminal scales it back up
      w = std::max(1, static_cast<int>(w * k));
      h = std::max(1, static_cast<int>(h * k));
    }
  }

  std::uint32_t next_kitty_id() {
    if (kitty_next == 0) // a random base, so another program's images in the window are safe
      kitty_next = 1 + ((std::random_device{}() & 0x7FFFu) << 8);
    const std::uint32_t id = kitty_next;
    kitty_next = kitty_next >= 0xFFFFFFu ? 1 : kitty_next + 1; // ids fit the 24-bit colour
    return id;
  }

  // The id a tile's placeholders name in this layout. A picture sent to the other slot since the
  // last layout has fully arrived by now, so they switch to it here.
  std::uint32_t kitty_id(const std::string &key) {
    KittySent &k = kitty[key];
    if (k.ids[0] == 0) {
      k.ids[0] = next_kitty_id();
      k.ids[1] = next_kitty_id();
    }
    if (k.pending) {
      k.shown ^= 1;
      k.pending = false;
    }
    return k.ids[k.shown];
  }

  // Lay picture `p` out as a pixel-mode image of cols x rows cells, recorded for this walk.
  std::shared_ptr<PixelGfx> pixel_image(const std::string &key, const Picture &p, PixCache &pc,
                                        int cols, int rows, ImageMode md) {
    int w = 0, h = 0;
    raster_size(cols, rows, md, w, h);
    if (pc.gen != p.gen || pc.raster.width() != w || pc.raster.height() != h) {
      pc.raster = p.rgb.width() == w && p.rgb.height() == h
                      ? p.rgb
                      : p.rgb.resized(w, h, cvc::image::resize_filter::box);
      pc.gen = p.gen;
      pc.cells = cvc::image();
    }
    const int cells_h = md == ImageMode::Kitty ? rows : 2 * rows; // a bitmap's half-block underlay
    if (pc.cells.width() != cols || pc.cells.height() != cells_h)
      pc.cells = pc.raster.resized(cols, cells_h, cvc::image::resize_filter::box);
    auto g = std::make_shared<PixelGfx>();
    g->key = key;
    g->mode = md;
    g->raster = pc.raster;
    g->cells = pc.cells;
    g->gen = p.gen;
    g->cols = cols;
    g->rows = rows;
    g->cell_w = cell_w();
    g->cell_h = cell_h();
    if (md == ImageMode::Kitty) {
      const int k = gfx::kKittyMaxCells;
      for (int r0 = 0; r0 < rows; r0 += k)
        for (int c0 = 0; c0 < cols; c0 += k) {
          KittyTile t;
          t.c0 = c0;
          t.r0 = r0;
          t.cols = std::min(k, cols - c0);
          t.rows = std::min(k, rows - r0);
          t.px = c0 * w / cols;
          t.py = r0 * h / rows;
          t.pw = std::max(1, (c0 + t.cols) * w / cols - t.px);
          t.ph = std::max(1, (r0 + t.rows) * h / rows - t.py);
          t.key = key + "#" + std::to_string(c0) + "," + std::to_string(r0);
          t.id = kitty_id(t.key);
          g->tiles.push_back(std::move(t));
        }
    }
    gfx_walk.push_back(g);
    return g;
  }

  // The bytes that bring the terminal up to date with the frame last drawn (take_graphics).
  std::string emit_graphics() {
    std::string out;
    const long frame = ++gfx_frames;
    const int colors = term_gfx.sixel_colors > 0 ? term_gfx.sixel_colors : 256;
    for (const std::shared_ptr<PixelGfx> &gp : gfx_shown) {
      const PixelGfx &g = *gp;
      if (!g.drawn || g.raster.empty())
        continue;
      const unsigned char *raster = std::as_const(g.raster).data();
      const std::size_t stride = g.raster.row_stride_bytes();
      if (g.mode == ImageMode::Kitty) {
        for (const KittyTile &t : g.tiles) {
          KittySent &k = kitty[t.key];
          k.frame = frame;
          const bool same_place =
              k.has[k.shown] && k.w == t.pw && k.h == t.ph && k.cols == t.cols && k.rows == t.rows;
          const int other = k.shown ^ 1;
          // Once the picture stops changing, the other id's older picture is no longer needed.
          const auto free_other = [&] {
            if (!k.pending && k.has[other]) {
              out += gfx::kitty_delete(k.ids[other]);
              k.has[other] = false;
            }
          };
          if (same_place && k.gen == g.gen) {
            free_other();
            continue; // the same picture, already there
          }
          const unsigned char *origin =
              raster + static_cast<std::size_t>(t.py) * stride + static_cast<std::size_t>(t.px) * 3;
          const std::uint64_t hash = hash_pixels(origin, t.pw, t.ph, stride);
          if (!same_place) { // the first picture, or a new size: under the id the cells name now
            out += gfx::kitty_transmit(origin, t.pw, t.ph, stride, k.ids[k.shown], t.cols, t.rows);
            k.has[k.shown] = true;
            k.pending = false;
          } else if (hash != k.hash) { // a new picture: to the other id (see KittySent)
            out += gfx::kitty_transmit(origin, t.pw, t.ph, stride, k.ids[other], t.cols, t.rows);
            k.has[other] = true;
            k.pending = true;
          } else { // a frame published again unchanged is not re-sent
            free_other();
          }
          k.gen = g.gen;
          k.hash = hash;
          k.w = t.pw;
          k.h = t.ph;
          k.cols = t.cols;
          k.rows = t.rows;
        }
        continue;
      }
      if (g.visible.size() != static_cast<std::size_t>(g.cols) * g.rows)
        continue; // Sixel / Iterm from here
      // A bitmap that reaches the screen's last row makes some terminals scroll (the cursor moves
      // below it), so that row keeps the half-blocks.
      std::vector<unsigned char> mask = g.visible;
      for (int r = 0; r < g.rows; ++r)
        if (g.box.y_min + r >= g.screen_h - 1)
          std::fill_n(mask.begin() + static_cast<std::ptrdiff_t>(r) * g.cols, g.cols, 0);
      // A bitmap is whole six-pixel bands tall, so its last band may reach into the row below.
      // Where that row shows the image too, let it: those pixels are the image's own there.
      // Elsewhere stop short, and the half-blocks show in the few pixels left. A terminal that
      // gives a bitmap the cells it touches crops an older one under a newer one, so draw top
      // first: the rectangle reached into always starts lower than the one reaching.
      std::vector<gfx::CellRect> rects = gfx::cover_rects(mask, g.cols, g.rows);
      std::stable_sort(rects.begin(), rects.end(),
                       [](const gfx::CellRect &a, const gfx::CellRect &b) { return a.y < b.y; });
      for (const gfx::CellRect &rc : rects) {
        SixelCached &sc =
            sixel_cache[g.key + "@" + std::to_string(rc.x) + "," + std::to_string(rc.y) + "," +
                        std::to_string(rc.w) + "," + std::to_string(rc.h)];
        if (g.mode == ImageMode::Iterm) {
          // The terminal scales the image to the rectangle's cells, so its pixels are the raster's
          // share of them, at whatever resolution the raster has.
          const int x0 = rc.x * g.raster.width() / g.cols;
          const int x1 = (rc.x + rc.w) * g.raster.width() / g.cols;
          const int y0 = rc.y * g.raster.height() / g.rows;
          const int y1 = (rc.y + rc.h) * g.raster.height() / g.rows;
          if (x1 <= x0 || y1 <= y0)
            continue;
          if (sc.stale(g, x1 - x0, y1 - y0)) {
            const unsigned char *origin =
                raster + static_cast<std::size_t>(y0) * stride + static_cast<std::size_t>(x0) * 3;
            sc.bytes = gfx::iterm_image(origin, x1 - x0, y1 - y0, stride, rc.w, rc.h);
            sc.made_from(g, x1 - x0, y1 - y0);
          }
        } else {
          bool below_shows = rc.y + rc.h < g.rows;
          for (int k = rc.x; k < rc.x + rc.w && below_shows; ++k)
            below_shows = mask[static_cast<std::size_t>(rc.y + rc.h) * g.cols + k] != 0;
          const int pw = rc.w * g.cell_w;
          const int ph = (rc.h * g.cell_h + (below_shows ? 5 : 0)) / 6 * 6;
          if (pw <= 0 || ph <= 0 || (rc.y * g.cell_h + ph) > g.raster.height() ||
              (rc.x + rc.w) * g.cell_w > g.raster.width())
            continue;
          if (sc.stale(g, pw, ph)) {
            const unsigned char *origin = raster +
                                          static_cast<std::size_t>(rc.y) * g.cell_h * stride +
                                          static_cast<std::size_t>(rc.x) * g.cell_w * 3;
            sc.bytes = gfx::sixel_encode(origin, pw, ph, stride, colors);
            sc.made_from(g, pw, ph);
          }
        }
        sc.frame = frame;
        // Save the cursor (DECSC), draw at the rectangle's top-left cell, put it back (DECRC).
        out += '\x1b';
        out += "7\x1b[" + std::to_string(g.box.y_min + rc.y + 1) + ";" +
               std::to_string(g.box.x_min + rc.x + 1) + "H";
        out += sc.bytes;
        out += '\x1b';
        out += '8';
      }
    }
    for (auto it = kitty.begin(); it != kitty.end();) {
      if (it->second.frame == frame) {
        ++it;
        continue;
      }
      for (int slot = 0; slot < 2; ++slot) // no longer drawn: free it in the terminal
        if (it->second.has[slot])
          out += gfx::kitty_delete(it->second.ids[slot]);
      it = kitty.erase(it);
    }
    for (auto it = sixel_cache.begin(); it != sixel_cache.end();)
      it = it->second.frame == frame ? std::next(it) : sixel_cache.erase(it);
    return out;
  }

  std::string release_graphics() {
    std::string out;
    for (const auto &k : kitty)
      for (int slot = 0; slot < 2; ++slot)
        if (k.second.has[slot])
          out += gfx::kitty_delete(k.second.ids[slot]);
    kitty.clear();
    sixel_cache.clear();
    return out;
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
    std::vector<std::string> scoped; // hosted: focus moves stay inside the window input is for
    if (opt.hosted)
      for (const std::string &id : last.focus_order)
        if (const Item *it = find_item(last, id); it && it->win == host_window)
          scoped.push_back(id);
    const std::vector<std::string> &order = opt.hosted ? scoped : last.focus_order;
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
        w.x = w.sx = std::clamp(e.x - drag_dx, 0, std::max(0, term_w - 4));
        w.y = w.sy = std::clamp(e.y - drag_dy, top_row(), std::max(top_row(), term_h - 2));
        consumed = true;
      } else if (!active.empty()) {
        moved = true;
        consumed = true;
      } else {
        const Item *it = item_at(e.x, e.y, opt.hosted ? host_window : window_at(e.x, e.y), "");
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
    const std::string win = opt.hosted ? host_window : window_at(x, y);
    if (!win.empty() && !opt.hosted) { // a hosted window's chrome is the host's
      raise(win);
      if (shown.closable.count(win) && on_title("winx|" + win)) {
        close_win = win;
        return;
      }
      if (y == boxes["win|" + win].y_min) { // the title bar: start a move
        const Win &w = windows[win];
        drag_win = win;
        drag_dx = x - w.sx;
        drag_dy = y - w.sy;
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
      if (win.empty())
        consumed = false; // the background's: RunOptions::background_input, then the document
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

  // The backdrop (set_background) as a screen-filling half-block image, or nullptr.
  fx::Element backdrop() {
    const auto it = images.find(background);
    if (background.empty() || it == images.end() || it->second.rgb.empty())
      return nullptr;
    const int cols = std::max(1, term_w), rows = std::max(1, term_h);
    if (pixels())
      return std::make_shared<PixelImage>(
          pixel_image("bg|", it->second, background_px, cols, rows, mode()));
    if (background_scaled.width() != cols || background_scaled.height() != 2 * rows)
      background_scaled = it->second.rgb.resized(cols, 2 * rows, cvc::image::resize_filter::box);
    return std::make_shared<HalfBlockImage>(background_scaled, cols, rows, opt.image_color);
  }

  void compose() {
    fx::Elements base = stack.empty() ? fx::Elements{} : std::move(stack.front().items);
    fx::Element backdrop_el = backdrop();
    if (opt.hosted) {
      root = base.empty() ? fx::emptyElement() : fx::vbox(std::move(base));
      if (backdrop_el)
        root = fx::dbox({backdrop_el, root});
      walk.bodies[""] = Walk::Body{"", false, root, std::move(win_popups[""])};
      return;
    }
    fx::Elements column;
    if (menubar)
      column.push_back(menubar);
    column.push_back(base.empty() ? fx::emptyElement() : fx::vbox(std::move(base)));
    fx::Elements screen;
    if (backdrop_el)
      screen.push_back(backdrop_el); // under everything; widgets draw over it
    screen.push_back(fx::vbox(std::move(column)));
    std::stable_sort(layers.begin(), layers.end(),
                     [](const Layer &a, const Layer &b) { return a.z < b.z; });
    for (Layer &l : layers)
      screen.push_back(std::move(l.el));
    for (fx::Element &p : popups)
      screen.push_back(std::move(p));
    if (!tooltip.empty())
      screen.push_back(place(fx::text(" " + tooltip + " ") | fx::inverted | fx::clear_under, 0,
                             std::max(0, term_h - 1)));
    if (!gfx_walk.empty())
      screen.push_back(std::make_shared<PixelFinish>(gfx_walk)); // after every layer
    root = fx::dbox(std::move(screen));
  }

  // The menu-and-focus bookkeeping a finished walk settles.
  void settle() {
    focus_next = false; // an autofocus with nothing focusable after it lapses with its walk
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
  c.glsl = false; // the cells draw no GLSL: a shader widget degrades (§16.3); a GL scene adapter
                  // behind them still runs the scene's (gl.scene.shaders)
  c.view_embed = false;
  c.owns_loop = true; // run() owns the terminal's event loop (§16.2)
  c.terminal = true;
  // What the terminal can draw, as far as is known: run() probes it, and a host that wants the
  // answer before loading a document (a requires: check) probes first, set_terminal_graphics().
  c.kitty = m_->term_gfx.kitty;
  c.sixel = m_->term_gfx.sixel;
  c.bitmap = m_->pixels();
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
  m.win_popups.clear();
  m.menubar = nullptr;
  m.tooltip.clear();
  m.last_item.clear();
  m.have_geom = false;
  m.have_tracks = false;
  m.have_columns = false;
  m.gfx_walk.clear();
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
  m.gfx_last = std::move(m.gfx_walk);
  m.gfx_walk.clear();
  m.settle();
  ++m.walks;
}

// ---- menus
// ---------------------------------------------------------------------------------------

bool FtxuiBackend::begin_main_menu_bar() {
  Impl &m = *m_;
  // Hosted menus are "open" to every walk (so the host can show and activate their items), which
  // would run a menu toggle before the widgets after the bar — and the core seeds a bound key with
  // the first default it meets, so a toggle for a window's tree.<id>.open would seed it closed
  // before the window could seed it open. A menu otherwise first runs when the user opens it, after
  // the first frame; hosted menus join the walk from the second frame to keep that order.
  if (m.opt.hosted && m.walks == 0)
    return false;
  Impl::Frame f;
  f.type = Impl::Frame::Type::Menubar;
  m.stack.push_back(std::move(f));
  m.walk.has_menubar = true;
  return true;
}

void FtxuiBackend::end_main_menu_bar() {
  Impl &m = *m_;
  Impl::Frame f = m.pop();
  if (m.opt.hosted)
    return; // the host draws the bar from hosted_menus()
  f.items.push_back(fx::filler() | fx::inverted);
  // Over a picture behind the widgets the bar is its own: inverted over the image's colours its
  // titles would take them (the same colour on both sides of a cell, often).
  m.menubar = fx::hbox(std::move(f.items)) | fx::clear_under;
}

bool FtxuiBackend::begin_menu(const char *label) {
  Impl &m = *m_;
  if (m.top().type == Impl::Frame::Type::Menubar && m.opt.hosted) {
    // Hosted: every menu is "open" to the walk, so its items are emitted (and so can be activated)
    // and recorded into the model; nothing is drawn.
    const std::string mid = std::string("menu|") + label;
    m.walk.menus.push_back(HostedMenu{label, {}});
    Impl::Frame f;
    f.type = Impl::Frame::Type::Menu;
    f.id = mid;
    m.stack.push_back(std::move(f));
    m.cur_menu = mid;
    return true;
  }
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
  if (f.type == Impl::Frame::Type::Menu && m.opt.hosted) {
    m.cur_menu.clear();
    return;
  }
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
  if (m.opt.hosted && !m.cur_menu.empty() && !m.walk.menus.empty())
    m.walk.menus.back().items.push_back(HostedMenuItem{id, label, false, false, false});
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
  if (m.opt.hosted && !m.cur_menu.empty() && !m.walk.menus.empty())
    m.walk.menus.back().items.push_back(HostedMenuItem{id, label, true, e.value, false});
  return e;
}

// ---- windows
// -------------------------------------------------------------------------------------

void FtxuiBackend::seed_window_geometry(const WindowGeom &g) {
  m_->pending_geom = g;
  m_->have_geom = true;
}

void FtxuiBackend::reset_layout() {
  m_->windows.clear();
  m_->drag_win.clear();
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
  // Keep it on screen where it is DRAWN, never by moving where it is placed: a frame drawn on a
  // momentarily tiny screen (a terminal or browser page before its first resize) must not pile
  // every window into the corner for good.
  w.sx = std::clamp(w.x, 0, std::max(0, m.term_w - 4));
  w.sy = std::clamp(w.y, m.top_row(), std::max(m.top_row(), m.term_h - 2));
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
  if (m.opt.hosted) { // keep the body for the host's window; the frame is the host's
    m.walk.bodies[f.id] =
        Impl::Walk::Body{f.title, f.p_open != nullptr,
                         f.items.empty() ? fx::emptyElement() : fx::vbox(std::move(f.items)),
                         std::move(m.win_popups[f.id])};
    m.walk.windows.push_back(f.id);
    return;
  }
  const Impl::Win &w = m.windows[f.id];
  // The window holding focus (or, failing that, the topmost) draws a heavy frame.
  bool holds_focus = false;
  for (const Impl::Item &it : m.walk.items)
    if (it.win == f.id && it.id == m.focused)
      holds_focus = true;
  fx::Element body = f.items.empty() ? fx::emptyElement() : fx::vbox(std::move(f.items));
  // §3.0.3b: a window with no size is as big as its content, held to its min: / max: and stopped at
  // the screen's edge from where it is drawn. Content past the window's size, authored or not,
  // scrolls: the body becomes a frame that keeps the focused widget in view, with a scroll bar.
  body->ComputeRequirement();
  // The window's size on one axis (0: the content's) for content needing `need` cells there,
  // including the window's frame.
  auto window_extent = [](int set, int need, int floor, int room, int lo, int hi) {
    if (set > 0)
      return std::max(floor, set);
    hi = std::min(hi, std::max(floor, room));
    return need > std::max(lo, hi) ? std::max({floor, lo, hi}) : 0;
  };
  const int min_w = f.size.min_w.is_set() ? m.extent_cols(f.size.min_w) : 0;
  const int max_w = f.size.max_w.is_set() ? m.extent_cols(f.size.max_w) : INT_MAX;
  const int min_h = f.size.min_h.is_set() ? m.extent_rows(f.size.min_h) : 0;
  const int max_h = f.size.max_h.is_set() ? m.extent_rows(f.size.max_h) : INT_MAX;
  int need_w = body->requirement().min_x + 2;
  int need_h = body->requirement().min_y + 2;
  int win_h = window_extent(w.h, need_h, 3, m.term_h - w.sy, min_h, max_h);
  bool scroll_y = win_h && need_h > win_h;
  need_w += scroll_y ? 1 : 0; // the vertical scroll bar's column
  const int win_w = window_extent(w.w, need_w, 4, m.term_w - w.sx, min_w, max_w);
  const bool scroll_x = win_w && need_w > win_w;
  if (scroll_x && !scroll_y) { // the horizontal scroll bar's row
    win_h = window_extent(w.h, ++need_h, 3, m.term_h - w.sy, min_h, max_h);
    scroll_y = win_h && need_h > win_h;
  }
  if (scroll_y)
    body = body | fx::vscroll_indicator;
  if (scroll_x)
    body = body | fx::hscroll_indicator;
  if (scroll_x)
    body = body | (win_h ? fx::frame : fx::xframe) | fx::flex;
  else if (win_h)
    body = body | fx::yframe | fx::flex; // scrolls to keep the focused widget in view
  fx::Elements bar{fx::text(" " + f.title + " ") | (holds_focus ? fx::bold : fx::nothing),
                   fx::filler()};
  if (f.p_open) {
    bar.push_back(fx::text("[x]") | fx::reflect(m.boxes["winx|" + f.id]));
    m.walk.closable.insert(f.id);
  }
  fx::Element el =
      fx::window(fx::hbox(std::move(bar)), body, holds_focus ? fx::HEAVY : fx::ROUNDED);
  if (win_w)
    el = el | fx::size(fx::WIDTH, fx::EQUAL, win_w);
  else if (min_w)
    el = el | fx::size(fx::WIDTH, fx::GREATER_THAN, min_w);
  if (win_h)
    el = el | fx::size(fx::HEIGHT, fx::EQUAL, win_h);
  else if (min_h)
    el = el | fx::size(fx::HEIGHT, fx::GREATER_THAN, min_h);
  el = el | fx::reflect(m.boxes["win|" + f.id]) | fx::clear_under;
  m.layers.push_back({w.z, place(isolated(std::move(el)), w.sx, w.sy)});
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

void FtxuiBackend::seed_grid_columns(const std::vector<float> &widths) {
  m_->pending_columns = widths;
  m_->have_columns = true;
}

std::vector<float> FtxuiBackend::grid_columns() const { return m_->last_columns; }

bool FtxuiBackend::begin_grid(const Layout &layout, const char *id) {
  Impl &m = *m_;
  Impl::Frame f;
  f.type = Impl::Frame::Type::Grid;
  f.layout = layout;
  f.id = m.uid(std::string("grid|") + (id ? id : "")); // keys the grid's draggable sizes
  if (m.have_tracks)
    f.tracks = m.pending_tracks;
  if (m.have_columns)
    f.columns = m.pending_columns;
  m.have_tracks = m.have_columns = false;
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
  // What a grid reports for persistence (grid_tracks / grid_columns): a track's size in px when it
  // is fixed (authored, seeded or dragged), else 0, "not set". A track laid out naturally has no
  // size until FTXUI lays this frame out, and persisting the last frame's size would pin it there
  // on the next load (a first walk would even persist the 1 cell of a box never laid out).
  m.last_columns.clear();
  auto report = [](std::vector<float> &out) {
    if (std::all_of(out.begin(), out.end(), [](float v) { return v <= 0.0f; }))
      out.clear(); // nothing set: nothing to persist
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
      m.last_tracks.push_back(gs.fixed[i] ? static_cast<float>(gs.size[i] * m.opt.cell_px_h)
                                          : 0.0f);
    }
    report(m.last_tracks);
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
      cs->seeded = false;
    }
    if (!cs->seeded) { // the persisted widths (seed_grid_columns) fix their columns, once
      for (std::size_t c = 0; c < cs->size.size() && c < f.columns.size(); ++c)
        if (f.columns[c] > 0.0f) {
          cs->size[c] = std::max(1, m.px_to_cols(f.columns[c]));
          cs->fixed[c] = true;
        }
      cs->seeded = true;
    }
    for (int c = 0; c < cols; ++c) {
      const std::size_t k = static_cast<std::size_t>(c);
      float px = 0.0f;
      if (cs->fixed[k])
        px = static_cast<float>(cs->size[k] * m.opt.cell_px_w);
      else if (k < L.col_widths.size() && L.col_widths[k].unit == Unit::Px)
        px = L.col_widths[k].value; // authored: as written, so a reload never drifts it
      m.last_columns.push_back(px);
    }
    report(m.last_columns);
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

// "label: value"; with no label just the value, as the ImGui backend draws it.
static std::string labelled(const char *label, const std::string &value) {
  return label && *label ? std::string(label) + ": " + value : value;
}

void FtxuiBackend::text_value(const char *label, const std::string &value) {
  m_->leaf(m_->uid("t|"), false, text_block(labelled(label, value)));
}

void FtxuiBackend::text_line_colored(const char *t, const float rgb[3]) {
  m_->leaf(m_->uid("t|"), false, text_block(t) | fx::color(rgb_color(rgb)));
}

void FtxuiBackend::text_value_colored(const char *label, const std::string &value,
                                      const float rgb[3]) {
  m_->leaf(m_->uid("t|"), false, text_block(labelled(label, value)) | fx::color(rgb_color(rgb)));
}

void FtxuiBackend::separator() {
  Impl &m = *m_;
  if (m.opt.hosted && !m.cur_menu.empty() && !m.walk.menus.empty())
    m.walk.menus.back().items.push_back(HostedMenuItem{"", "", false, false, true});
  m.add(fx::separator());
}

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
    // Hosted: the list belongs to the combo's window and is laid out in that window's cells.
    const std::string win = m.wins.empty() ? std::string() : m.wins.back();
    const auto bh = m.body_h.find(win);
    const int room = m.opt.hosted && bh != m.body_h.end() ? bh->second : m.term_h;
    int y = anchor.y_max + 1;
    if (y + n + 2 > room)
      y = std::max(0, anchor.y_min - n - 2); // no room below: open upward
    fx::Element list =
        place(isolated(fx::vbox(std::move(rows)) | fx::border | fx::clear_under), anchor.x_min, y);
    (m.opt.hosted ? m.win_popups[win] : m.popups).push_back(std::move(list));
    m.walk.combo_popup = id;
    m.walk.combo_n = n;
  }
  return e;
}

// One focusable leaf: the caption, then "(*) a  ( ) b" side by side (row) or stacked under it.
// A click on an option picks it; Left / Right pick the previous / next option and Space / Enter
// the next, wrapping, like a combo's Left / Right.
IndexEdit FtxuiBackend::radio(const char *label, int current_index,
                              const std::vector<std::string> &options, bool row) {
  Impl &m = *m_;
  const std::string id = m.uid(std::string("ra|") + label);
  const int n = static_cast<int>(options.size());
  const auto opt_box = [&](int i) -> fx::Box & { return m.boxes[id + "#" + std::to_string(i)]; };
  IndexEdit e;
  const int cur = n == 0 ? -1 : std::clamp(current_index, 0, n - 1);
  int shown = cur;
  if (n > 0 && !m.is_disabled()) {
    int pick = -1;
    if (m.pressed(id)) {
      for (int i = 0; i < n; ++i)
        if (inside(opt_box(i), m.ptr_x, m.ptr_y))
          pick = i;
    } else if (m.key(id, "Left")) {
      pick = (cur - 1 + n) % n;
    } else if (m.key(id, "Right") || m.key(id, "Space") || m.key(id, "Return")) {
      pick = (cur + 1) % n;
    }
    if (pick >= 0) {
      e = {true, true, pick};
      shown = pick;
    }
  }
  const bool f = m.has_focus(id);
  fx::Elements opts;
  for (int i = 0; i < n; ++i) {
    fx::Element o = fx::text((i == shown ? "(*) " : "( ) ") + options[static_cast<std::size_t>(i)]);
    if (f && i == shown)
      o = o | fx::inverted;
    o = o | fx::reflect(opt_box(i));
    if (row) {
      opts.push_back(fx::text(i == 0 ? "" : "  "));
      opts.push_back(o);
    } else {
      opts.push_back(fx::hbox({fx::text("  "), o}));
    }
  }
  fx::Element caption = fx::text(std::string(label) + ":" + (row ? " " : ""));
  if (f && n == 0)
    caption = caption | fx::inverted;
  m.leaf(id, true,
         row ? fx::hbox({caption, fx::hbox(std::move(opts))})
             : fx::vbox({caption, fx::vbox(std::move(opts))}));
  return e;
}

void FtxuiBackend::focus_next_widget() { m_->focus_next = true; }

StringEdit FtxuiBackend::input_text(const char *label, const char *key, const std::string &current,
                                    bool masked) {
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
  // "label: [text]", with the cursor drawn as an inverted cell while focused. A password field
  // draws one mark per character in place of the text.
  const auto shown = [masked](const std::string &s) {
    if (!masked)
      return s;
    std::string out;
    // U+2022 as UTF-8 bytes: MSVC encodes a \u literal in the code page unless built with /utf-8.
    for (std::size_t p = 0; p < s.size(); p = utf8_next(s, p))
      out += "\xe2\x80\xa2";
    return out;
  };
  fx::Elements row{fx::text(std::string(label) + ": [")};
  if (m.has_focus(id)) {
    const std::string &t = it != m.edits.end() ? it->second.text : e.value;
    const std::size_t c = it != m.edits.end() ? it->second.cursor : t.size();
    const std::size_t c2 = utf8_next(t, c);
    row.push_back(fx::text(shown(t.substr(0, c))));
    row.push_back(fx::text(c < t.size() ? shown(t.substr(c, c2 - c)) : std::string(" ")) |
                  fx::inverted);
    row.push_back(fx::text(c2 < t.size() ? shown(t.substr(c2)) : std::string()));
  } else {
    row.push_back(fx::text(shown(e.value)));
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
  p.gen = ++m_->image_gen;
  if (name == m_->background)
    m_->background_scaled = cvc::image(); // a new frame for the backdrop: resample it next draw
}

void FtxuiBackend::clear_image(const std::string &name) { m_->images.erase(name); }

void FtxuiBackend::set_background(const std::string &image_name) {
  m_->background = image_name;
  m_->background_scaled = cvc::image();
  m_->background_px = Impl::PixCache{};
}

int FtxuiBackend::screen_width() const { return m_->term_w; }
int FtxuiBackend::screen_height() const { return m_->term_h; }

void FtxuiBackend::set_terminal_graphics(const TermGraphics &g) {
  m_->term_gfx = g;
  m_->term_gfx_set = true;
}

TermGraphics FtxuiBackend::terminal_graphics() const { return m_->term_gfx; }

FtxuiBackend::Options::ImageColor FtxuiBackend::image_mode() const {
  const ImageMode md = m_->mode();
  if (md != ImageMode::Auto)
    return md;
  return fx::Terminal::ColorSupport() == fx::Terminal::Color::Palette1 ? ImageMode::Mono
                                                                       : ImageMode::Color;
}

int FtxuiBackend::cell_pixel_width() const { return m_->cell_w(); }
int FtxuiBackend::cell_pixel_height() const { return m_->cell_h(); }

void FtxuiBackend::image_raster(int cols, int rows, int &w, int &h) const {
  m_->raster_size(cols, rows, m_->mode(), w, h);
}

std::string FtxuiBackend::take_graphics() { return m_->emit_graphics(); }
std::string FtxuiBackend::release_graphics() { return m_->release_graphics(); }

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
  if (m.pixels()) { // real pixels: the rows follow the terminal's real cell shape
    const int rows =
        std::max(1, static_cast<int>(std::lround(cols * aspect * m.cell_w() / double(m.cell_h()))));
    const std::string id = m.uid("img|");
    m.leaf(id, false,
           std::make_shared<PixelImage>(m.pixel_image(id, p, p.px, cols, rows, m.mode())));
    return true;
  }
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
  // The placeholders' 24-bit colour names their image, and a terminal with the kitty protocol has
  // true colour, so lay out and print as one would. For the whole walk, not only the print: FTXUI
  // quantizes an RGB colour to the terminal's palette when the colour is made, during layout, and
  // a palette colour names no image.
  const bool true_color = ansi && m.mode() == ImageMode::Kitty;
  const fx::Terminal::Color colors = fx::Terminal::ColorSupport();
  if (true_color)
    fx::Terminal::SetColorSupport(fx::Terminal::Color::TrueColor);
  rt.render(); // walk the tree -> (re)builds m.root through the methods above
  m.gfx_shown = m.gfx_last;
  fx::Screen screen =
      fx::Screen::Create(fx::Dimension::Fixed(m.term_w),
                         height > 0 ? fx::Dimension::Fixed(height) : fx::Dimension::Fit(m.root));
  fx::Render(screen, m.root);
  m.shown = m.last; // the boxes now hold this frame's layout
  if (ansi) {
    std::string out = screen.ToString();
    if (true_color)
      fx::Terminal::SetColorSupport(colors);
    return out;
  }
  std::string out;
  for (int y = 0; y < screen.dimy(); ++y) {
    std::string line;
    bool wide = false; // the cell before held a full-width glyph (an emoji, CJK)
    for (int x = 0; x < screen.dimx(); ++x) {
      const std::string &c = screen.CellAt(x, y).character;
      // A full-width glyph takes this cell too: a terminal advances two columns for it, so the
      // cell isn't written, as Screen::ToString() leaves it out. Written as a space, it pushed the
      // rest of the line a column right.
      if (wide) {
        wide = false;
        continue;
      }
      line += c.empty() ? std::string(" ") : c;
      wide = !c.empty() && fx::string_width(c) == 2;
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

// ---- hosted chrome
// -------------------------------------------------------------------------------

namespace {

std::string sgr(const fx::Color &c, bool background) {
  return c == fx::Color(fx::Color::Default) ? std::string() : c.Print(background);
}

} // namespace

std::vector<FtxuiBackend::HostedWindow> FtxuiBackend::hosted_windows() const {
  const Impl &m = *m_;
  std::vector<HostedWindow> out;
  for (const std::string &id : m.last.windows) {
    const auto b = m.last.bodies.find(id);
    if (b == m.last.bodies.end())
      continue;
    HostedWindow hw;
    hw.id = id;
    hw.title = b->second.title;
    hw.closable = b->second.closable;
    if (const auto w = m.windows.find(id); w != m.windows.end()) {
      hw.x = w->second.x;
      hw.y = w->second.y;
      hw.w = w->second.w;
      hw.h = w->second.h;
    }
    if (b->second.body) {
      b->second.body->ComputeRequirement();
      hw.content_w = b->second.body->requirement().min_x;
      hw.content_h = b->second.body->requirement().min_y;
    }
    out.push_back(std::move(hw));
  }
  return out;
}

std::vector<FtxuiBackend::HostedMenu> FtxuiBackend::hosted_menus() const { return m_->last.menus; }

bool FtxuiBackend::hosted_root() const {
  if (!m_->background.empty())
    return true;
  for (const Impl::Item &it : m_->last.items)
    if (it.win.empty() && it.menu.empty())
      return true;
  return false;
}

std::string FtxuiBackend::tooltip() const { return m_->tooltip; }

TermRows FtxuiBackend::render_window(const std::string &id, int width, int height) {
  Impl &m = *m_;
  TermRows rows;
  const auto b = m.last.bodies.find(id);
  if (b == m.last.bodies.end() || width <= 0 || height <= 0)
    return rows;
  m.body_h[id] = height;
  fx::Elements stack{b->second.body ? b->second.body | fx::yframe : fx::emptyElement()};
  for (const fx::Element &p : b->second.popups)
    stack.push_back(p);
  fx::Element el = fx::dbox(std::move(stack));
  fx::Screen screen(width, height);
  fx::Render(screen, el);
  m.shown = m.last; // this window's boxes now hold its layout, in its own cells
  rows.resize(static_cast<std::size_t>(height));
  for (int y = 0; y < height; ++y) {
    std::vector<TermCell> &row = rows[static_cast<std::size_t>(y)];
    row.resize(static_cast<std::size_t>(width));
    for (int x = 0; x < width; ++x) {
      const fx::Cell &c = screen.CellAt(x, y);
      TermCell &t = row[static_cast<std::size_t>(x)];
      t.ch = c.character;
      t.fg = sgr(c.foreground_color, false);
      t.bg = sgr(c.background_color, true);
      t.bold = c.bold;
      t.dim = c.dim;
      t.italic = c.italic;
      t.underlined = c.underlined;
      t.inverted = c.inverted;
    }
  }
  return rows;
}

bool FtxuiBackend::feed_window(Runtime &rt, const std::string &id, const TermEvent &ev) {
  m_->host_window = id;
  return feed(rt, ev);
}

void FtxuiBackend::activate_menu_item(Runtime &rt, const std::string &item_id) {
  Impl &m = *m_;
  m.ev_live = false;
  m.consumed = false;
  m.press_id = item_id; // the walk's activated() takes it as a press
  rt.render();
  m.end_event();
}

void FtxuiBackend::close_window(Runtime &rt, const std::string &id) {
  Impl &m = *m_;
  m.close_win = id;
  rt.render();
  m.end_event();
}

void FtxuiBackend::set_window_geometry(const std::string &id, int x, int y, int w, int h) {
  Impl::Win &win = m_->windows[id];
  win.x = x;
  win.y = y;
  win.w = w;
  win.h = h;
  win.placed = win.seeded = win.sized = true;
}

void FtxuiBackend::set_screen_size(int width, int height) {
  m_->term_w = std::max(1, width);
  m_->term_h = std::max(1, height);
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
  // Pixel graphics: ask the terminal what it can draw before FTXUI takes it over. A reply to a
  // kitty query arriving later would reach FTXUI's input parser (it has no APC case) as typed text.
  const bool want_pixels = !m.opt.hosted && (m.opt.image_color == ImageMode::Auto ||
                                             Impl::pixel_mode(m.opt.image_color));
  if (want_pixels && !m.term_gfx_set) {
    m.term_gfx = gfx::probe_terminal();
    m.term_gfx_set = true;
  }
  fx::ScreenInteractive screen = fx::ScreenInteractive::Fullscreen();
  screen.TrackMouse(opts.mouse);
  // Ctrl-C is handled here (see `interrupt`), so it returns to the host. FTXUI's own handling
  // re-raises SIGINT, which would end the whole process under a host that embeds this loop.
  screen.ForceHandleCtrlC(false);
  bool pixels = false; // a pixel image mode is on (decided once the terminal is set up)
  fx::Component renderer = fx::Renderer([&] {
    const fx::Dimensions d = fx::Terminal::Size();
    m.term_w = std::max(1, d.dimx);
    m.term_h = std::max(1, d.dimy);
    int cw = 0, ch = 0;
    if (pixels && gfx::cell_pixel_size(1, cw, ch)) { // follows a font-size change
      m.term_gfx.cell_w = cw;
      m.term_gfx.cell_h = ch;
    }
    rt.render();
    m.shown = m.last; // FTXUI lays this frame out next, which fills the boxes
    m.gfx_shown = m.gfx_last;
    m.drew = true;
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
    route(rt, te, opts.background_input, unhandled);
    return true;
  });
  fx::Loop loop(&screen, app); // sets the terminal up (and FTXUI's signal handlers)
  // FTXUI's own DA1 exchange (just now) also reports Sixel.
  if (want_pixels)
    for (const int cap : screen.TerminalCapabilities())
      if (cap == 4)
        m.term_gfx.sixel = true;
  pixels = m.pixels();
  const fx::Terminal::Color colors = fx::Terminal::ColorSupport();
  if (m.mode() == ImageMode::Kitty) // the placeholders' 24-bit colour names their image
    fx::Terminal::SetColorSupport(fx::Terminal::Color::TrueColor);
  if (opts.on_start)
    opts.on_start();
  const auto period = std::chrono::milliseconds(1000 / std::max(1, opts.fps));
  const bool bitmaps = m.mode() == ImageMode::Sixel || m.mode() == ImageMode::Iterm;
  while (!loop.HasQuitted()) {
    // A repaint goes out between synchronized-output marks, so a terminal that has them shows it
    // whole. Sixel / iTerm2 bitmaps go inside: the repaint just covered the ones it replaces. Kitty
    // images go after: a big one would overflow the terminal's synchronized-update buffer, and a
    // changed one goes to an id the repaint does not name yet (KittySent), so the shown image
    // stays whole while the next one arrives.
    if (pixels)
      std::cout << "\x1b[?2026h";
    m.drew = false;
    loop.RunOnce();
    if (pixels) {
      const std::string g = m.drew ? m.emit_graphics() : std::string();
      if (bitmaps)
        std::cout << g << "\x1b[?2026l";
      else
        std::cout << "\x1b[?2026l" << g;
      std::cout << std::flush;
    }
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
  if (pixels)
    std::cout << m.release_graphics() << std::flush;
  fx::Terminal::SetColorSupport(colors);
  if (opts.on_stop)
    opts.on_stop();
  return result;
}

FtxuiBackend::RunExit FtxuiBackend::run(Runtime &rt) { return run(rt, RunOptions{}); }

FtxuiBackend::RunExit FtxuiBackend::run(Runtime &rt, const RunOptions &opts) {
  return run_loop(rt, opts, [&] { rt.drain(); }, nullptr, nullptr);
}

FtxuiBackend::RunExit FtxuiBackend::run(AppRuntime &app) { return run(app, RunOptions{}); }

void FtxuiBackend::forward_to_document(AppRuntime &app, const TermEvent &e) const {
  const int cw = m_->opt.cell_px_w, ch = m_->opt.cell_px_h;
  // Keys carry SDL's modifier bits (shift 0x1, ctrl 0x40, alt 0x100) and go as a press + release
  // pair (a terminal reports no key-up).
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
    app.post_pointer(static_cast<int>(InputEvent::Kind::MouseButtonDown), x, y, 0, 0, e.button, 1);
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
}

FtxuiBackend::RunExit FtxuiBackend::run(AppRuntime &app, const RunOptions &opts) {
  // What the UI didn't use goes to the document's on_key / on_pointer.
  return run_loop(
      app.runtime(), opts, [&] { app.drain(); }, [&] { return app.should_close(); },
      [&](const TermEvent &e) { forward_to_document(app, e); });
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
void FtxuiBackend::reset_layout() {}
WindowGeom FtxuiBackend::window_geometry() const { return {}; }
void FtxuiBackend::push_id(const char *) {}
void FtxuiBackend::pop_id() {}
bool FtxuiBackend::begin_grid(const Layout &, const char *) { return false; }
void FtxuiBackend::grid_next_cell() {}
void FtxuiBackend::end_grid() {}
void FtxuiBackend::seed_grid_tracks(const std::vector<float> &) {}
std::vector<float> FtxuiBackend::grid_tracks() const { return {}; }
void FtxuiBackend::seed_grid_columns(const std::vector<float> &) {}
std::vector<float> FtxuiBackend::grid_columns() const { return {}; }
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
IndexEdit FtxuiBackend::radio(const char *, int, const std::vector<std::string> &, bool) {
  return {};
}
StringEdit FtxuiBackend::input_text(const char *, const char *, const std::string &, bool) {
  return {};
}
void FtxuiBackend::focus_next_widget() {}
bool FtxuiBackend::draw_image(const char *, float) { return false; }
ColorEdit FtxuiBackend::color(const char *, const float[3]) { return {}; }
void FtxuiBackend::set_image(const std::string &, const cvc::image &) {}
void FtxuiBackend::clear_image(const std::string &) {}
void FtxuiBackend::set_background(const std::string &) {}
int FtxuiBackend::screen_width() const { return 0; }
int FtxuiBackend::screen_height() const { return 0; }
void FtxuiBackend::set_terminal_graphics(const TermGraphics &) {}
TermGraphics FtxuiBackend::terminal_graphics() const { return {}; }
FtxuiBackend::Options::ImageColor FtxuiBackend::image_mode() const {
  return Options::ImageColor::Color;
}
int FtxuiBackend::cell_pixel_width() const { return 8; }
int FtxuiBackend::cell_pixel_height() const { return 16; }
void FtxuiBackend::image_raster(int cols, int rows, int &w, int &h) const {
  w = cols;
  h = 2 * rows;
}
std::string FtxuiBackend::take_graphics() { return {}; }
std::string FtxuiBackend::release_graphics() { return {}; }
std::string FtxuiBackend::render_to_string(Runtime &, int, int, bool) {
  return "ari: libcvc was built without FTXUI (terminal backend unavailable)\n";
}
bool FtxuiBackend::feed(Runtime &, const TermEvent &) { return false; }
std::vector<FtxuiBackend::HostedWindow> FtxuiBackend::hosted_windows() const { return {}; }
std::vector<FtxuiBackend::HostedMenu> FtxuiBackend::hosted_menus() const { return {}; }
bool FtxuiBackend::hosted_root() const { return false; }
std::string FtxuiBackend::tooltip() const { return {}; }
TermRows FtxuiBackend::render_window(const std::string &, int, int) { return {}; }
bool FtxuiBackend::feed_window(Runtime &, const std::string &, const TermEvent &) { return false; }
void FtxuiBackend::activate_menu_item(Runtime &, const std::string &) {}
void FtxuiBackend::close_window(Runtime &, const std::string &) {}
void FtxuiBackend::set_window_geometry(const std::string &, int, int, int, int) {}
void FtxuiBackend::set_screen_size(int, int) {}
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
void FtxuiBackend::forward_to_document(AppRuntime &, const TermEvent &) const {}
bool FtxuiBackend::available() { return false; }

#endif

void FtxuiBackend::route(Runtime &rt, const TermEvent &ev,
                         const std::function<bool(const TermEvent &)> &background,
                         const std::function<void(const TermEvent &)> &unhandled) {
  if (ev.kind != TermEvent::Kind::Mouse && unhandled && rt.key_capture_pending()) {
    unhandled(ev);
    return;
  }
  if (!feed(rt, ev) && !(background && background(ev)) && unhandled)
    unhandled(ev);
}

// ── custom widgets: the same in both builds (a host's draw fn calls the primitives above)
// ─────────

void FtxuiBackend::register_widget(std::string custom_type, CustomDrawFn draw) {
  if (draw)
    customWidgets_[std::move(custom_type)] = std::move(draw);
  else
    customWidgets_.erase(custom_type);
}

CustomEdit FtxuiBackend::custom_widget(const char *type, const std::string &current,
                                       const Widget &w) {
  const auto it = customWidgets_.find(type ? type : "");
  if (it == customWidgets_.end())
    return {}; // not handled: the core draws a placeholder
  CustomEdit e = it->second(current, w);
  e.handled = true; // a registered handler drew it, whatever its edit says
  return e;
}

} // namespace ariadne
} // namespace cvc
