#ifndef CVC_ARIADNE_FTXUI_BACKEND_H
#define CVC_ARIADNE_FTXUI_BACKEND_H

// FtxuiBackend — the pure-terminal Ariadne backend, built on FTXUI (roadmap §16.4: the portable,
// zero-dependency default terminal backend). It renders the SAME cvc::ariadne Widget tree the
// ImGui/VTK backend renders, with no ImGui, no VTK, no GL — just a terminal.
//
// It is the framework-owned-loop counterpart to the host-pumped ImGui backend:
// capabilities().owns_loop == true (run() drives FTXUI's loop, which calls Runtime::render()),
// where cvc::gl::ImGuiBackend is owns_loop == false (VTK's render pass drives it).
//
// The model is IMMEDIATE-MODE over FTXUI's DOM, the way ImGui works: each walk of the tree rebuilds
// an FTXUI Element tree, and the backend keeps only the cross-frame interaction state (focus, the
// pressed / hovered item, open menu / combo, in-progress text and slider edits, window placement).
// Each Element is tagged with its on-screen Box when FTXUI lays it out, so a mouse event is
// hit-tested against the last frame drawn. That maps the Backend contract one-to-one: a primitive
// draws value-in and reports {changed, committed, value} out, and never touches cvc::state.
//
// What a terminal gets:
//   - keyboard: Tab / Shift-Tab (and Up / Down) move focus; Enter / Space activate; Left / Right
//     step a slider or cycle a combo; text fields edit in place and commit on Enter or focus loss;
//     Esc reverts an edit or closes a popup; F10 opens the menu bar.
//   - mouse: click to focus / activate, drag a slider, drag a window by its title bar, click [x] to
//     close it, click a menu title to open it.
//   - free floating windows (capabilities().windows): placed from the authored pos/size or the
//     persisted tree.<id>.geometry, raised on focus, z-ordered.
//   - graphics: draw_image renders a published cvc::image as half-block cells (two pixels per cell,
//     24-bit colour where the terminal has it).
//
// Units. Ariadne authors positions and sizes in px. A terminal lays out in character cells, so this
// backend converts px to cells by a nominal cell size (Options::cell_px_w / cell_px_h, default
// 8x16) and converts back when it reports geometry (window_geometry, grid_tracks, item_pointer). A
// layout persisted under one backend therefore stays meaningful under the other.
//
// Optional: compiled with the real FTXUI only when libcvc is built with it (CVC_ARIADNE_FTXUI, via
// find_package(ftxui)); otherwise every method is an inert stub and available() is false, so libcvc
// still builds without FTXUI. The FTXUI types are hidden behind a Pimpl, so this header pulls in no
// FTXUI headers — a consumer needs FTXUI only to LINK a build that uses it.

#include <cvc/ariadne/backend.h>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
class image;
namespace ariadne {

class Runtime;
class AppRuntime;

// One terminal input event, toolkit-neutral. run() translates FTXUI's events into these; a test or
// a non-FTXUI driver builds them directly and hands them to FtxuiBackend::feed().
struct TermEvent {
  enum class Kind {
    Key,   // a named key ("Tab", "Return", ...) or a Ctrl / Alt chord on a letter
    Text,  // one printable UTF-8 character
    Mouse, // a pointer event at a cell
  };
  enum class Mouse { Press, Release, Move, WheelUp, WheelDown };

  Kind kind = Kind::Key;
  // Key: the key name, in the vocabulary SDL uses so a forwarded key reads the same in an on_key
  // handler under either backend — "Tab", "Return", "Escape", "Space", "Backspace", "Delete",
  // "Left", "Right", "Up", "Down", "Home", "End", "PageUp", "PageDown", "F1".."F12", or an
  // upper-case letter / digit for a chord. Text: the key the character came from ("A" for 'a',
  // "Space" for ' ').
  std::string key;
  std::string text; // Text: the character itself
  bool shift = false, ctrl = false, alt = false;

  Mouse mouse = Mouse::Press;
  int button = 1;   // Mouse: 1 left, 2 middle, 3 right; 0 for a move with no button held
  int x = 0, y = 0; // Mouse: 0-based cell column / row from the screen's top-left

  static TermEvent key_event(std::string name, bool shift = false, bool ctrl = false,
                             bool alt = false);
  static TermEvent text_event(std::string utf8);
  static TermEvent mouse_event(Mouse m, int x, int y, int button = 1);
};

// One rendered terminal cell, toolkit-neutral — what render_window hands a host to draw.
struct TermCell {
  std::string ch;     // the glyph (UTF-8); "" for the cell a wide glyph spills into
  std::string fg, bg; // SGR colour parameters, "" for the default: "31", "38;5;21", "48;2;0;0;255"
  bool bold = false, dim = false, italic = false, underlined = false, inverted = false;
};
using TermRows = std::vector<std::vector<TermCell>>;

class FtxuiBackend : public Backend {
public:
  struct Options {
    // The nominal pixel size of one terminal cell, for the px <-> cell conversion (see the header).
    int cell_px_w = 8;
    int cell_px_h = 16;
    // How draw_image paints: Auto follows the terminal's detected colour support (a terminal with
    // no colour gets a luminance ramp), Mono always draws the ramp, Color always draws coloured
    // half-blocks.
    enum class ImageColor { Auto, Mono, Color };
    ImageColor image_color = ImageColor::Auto;
    // Hosted chrome: draw no window frames and no menu bar, for a host that supplies its own (see
    // "hosted chrome" below).
    bool hosted = false;
  };

  FtxuiBackend();
  explicit FtxuiBackend(const Options &opts);
  ~FtxuiBackend() override;

  FtxuiBackend(const FtxuiBackend &) = delete;
  FtxuiBackend &operator=(const FtxuiBackend &) = delete;

  Capabilities capabilities() const override;

  void begin_frame() override;
  void end_frame() override;

  bool begin_main_menu_bar() override;
  void end_main_menu_bar() override;
  bool begin_menu(const char *label) override;
  void end_menu() override;
  bool begin_window(const char *title, const char *id, const Size &size, float border,
                    bool *p_open = nullptr) override;
  void end_window() override;
  void seed_window_geometry(const WindowGeom &g) override;
  WindowGeom window_geometry() const override;
  void push_id(const char *id) override;
  void pop_id() override;

  bool begin_grid(const Layout &layout, const char *id) override;
  void grid_next_cell() override;
  void end_grid() override;
  void seed_grid_tracks(const std::vector<float> &sizes) override;
  std::vector<float> grid_tracks() const override;

  void begin_disabled() override;
  void end_disabled() override;
  void set_tooltip(const char *text) override;

  void text_line(const char *text) override;
  void text_value(const char *label, const std::string &value) override;
  void text_line_colored(const char *text, const float rgb[3]) override;
  void text_value_colored(const char *label, const std::string &value, const float rgb[3]) override;
  void separator() override;
  bool button(const char *label) override;
  bool menu_item_action(const char *label) override;

  bool item_clicked() override;
  bool item_hovered() override;
  bool item_dragged() override;
  bool item_drag_started() override;
  bool item_drag_ended() override;
  bool item_pointer(float &x, float &y, float &dx, float &dy, int &button) override;

  BoolEdit menu_item_toggle(const char *label, bool current) override;
  BoolEdit checkbox(const char *label, bool current) override;
  IntEdit slider_int(const char *label, const char *key, int current, int lo, int hi) override;
  DoubleEdit slider_double(const char *label, const char *key, double current, double lo, double hi,
                           const char *fmt) override;
  IndexEdit combo(const char *label, int current_index,
                  const std::vector<std::string> &options) override;
  StringEdit input_text(const char *label, const char *key, const std::string &current) override;
  bool draw_image(const char *name, float width) override;
  ColorEdit color(const char *label, const float rgb[3]) override;

  // Publish (or replace) the image a Kind::Image widget named `name` shows — the terminal
  // counterpart of cvc::gl::ImGuiBackend::set_image. Any pixel format / data type; it is converted
  // to 8-bit RGB once here and box-filtered to the cell grid at draw time.
  void set_image(const std::string &name, const cvc::image &img);
  void clear_image(const std::string &name);

  // ---- headless (no TTY) ----

  // Render the current tree at `width` x `height` cells (height 0 = fit the content) and return it
  // as text: plain characters, one line per row with trailing blanks trimmed, or — with `ansi` —
  // the escape-coded string a terminal would show. Calls rt.render() to (re)build the frame. The
  // frame this draws is the one the next feed()'s mouse hit-test runs against.
  std::string render_to_string(Runtime &rt, int width = 80, int height = 0, bool ansi = false);

  // Deliver one input event: route it (focus, popups, window drag, hit-test against the last frame
  // drawn) and run a walk so the widget it lands on reports its edit to the core. Returns whether
  // the UI used the event; an unused key is what a host forwards to the document's on_key.
  bool feed(Runtime &rt, const TermEvent &ev);

  // ---- hosted chrome ----
  // With Options::hosted this backend draws no window frames and no menu bar. It keeps each
  // window's body for a host that supplies its own chrome — cvc::ariadne::TvisionBackend, with
  // native movable windows and menus — to render into its window, exposes the menu bar as a model
  // whose items the host activates, and takes input per window in window-local cells. Widgets
  // outside every window form the body of the window id "" (the host's background).

  struct HostedWindow {
    std::string id, title;
    bool closable = false;
    int x = 0, y = 0; // placement from the authored pos / persisted geometry (cells)
    int w = 0, h = 0; // outer size when authored / persisted / host-set, else 0
    int content_w = 0, content_h = 0; // the body's natural size (cells)
  };
  struct HostedMenuItem {
    std::string id, label;
    bool toggle = false, checked = false, separator = false;
  };
  struct HostedMenu {
    std::string label;
    std::vector<HostedMenuItem> items;
  };

  std::vector<HostedWindow> hosted_windows() const; // the windows the last walk drew, in order
  std::vector<HostedMenu> hosted_menus() const;     // the menu bar the last walk drew
  bool hosted_root() const;                         // whether body "" has any widgets
  std::string tooltip() const;                      // the focused / hovered widget's tooltip

  // Lay out window `id`'s body (from the last walk) into width x height cells. Mouse events fed
  // for that window hit-test against this layout.
  TermRows render_window(const std::string &id, int width, int height);
  // feed() for one window: mouse coordinates are local to that window's body, and Tab / arrow
  // focus moves stay inside it.
  bool feed_window(Runtime &rt, const std::string &id, const TermEvent &ev);
  // The host's chrome acted: a menu item was chosen, a window's close box was hit. Each runs a
  // walk, so the widget reports it to the core.
  void activate_menu_item(Runtime &rt, const std::string &item_id);
  void close_window(Runtime &rt, const std::string &id);
  // The host's current bounds for window `id` (cells), reported back as its geometry (§11.4).
  void set_window_geometry(const std::string &id, int x, int y, int w, int h);
  // The host's screen size in cells, which percent sizes resolve against.
  void set_screen_size(int width, int height);

  // ---- interactive (owns the terminal) ----

  struct RunOptions {
    int fps = 20;      // redraw rate while idle, so state changed off the UI (on_tick, async verbs,
                       // replicated peers) shows up; input redraws immediately regardless
    bool mouse = true; // enable mouse reporting

    // OS signals (SIGTERM, SIGHUP, a SIGINT from `kill`, ...). Signal handlers are process-wide,
    // so they are the host application's to own; the loop only offers the seams. `on_start` runs
    // once the terminal is set up, after FTXUI has installed its own handlers, so a handler the
    // host installs there takes precedence; `on_stop` runs before the terminal is restored — put
    // the previous handlers back there. `take_signal` is polled every iteration and returns the
    // name of one pending signal ("SIGTERM") or "" for none; each goes to the document's
    // on_signal: (source "os") and, when the document has no handler, ends the loop. See
    // src/ari/main.cpp for a host that wires these to sigaction.
    std::function<void()> on_start;
    std::function<void()> on_stop;
    std::function<std::string()> take_signal;
  };

  // Why run() returned.
  struct RunExit {
    std::string signal;   // the signal that ended the loop, or "" (a quit action / should_close)
    bool from_os = false; // it came from take_signal rather than an interrupt key
  };

  // Run FTXUI's event loop fullscreen on the terminal: each input is fed, each iteration drains
  // rt's queued actions, and the screen redraws. Ctrl-C and Ctrl-\ are signals (SIGINT, SIGQUIT):
  // they go to the document's on_signal: program when it has one (Runtime::post_signal), and
  // otherwise end the loop; three Ctrl-Cs within two seconds always end it. Returns when the loop
  // ends. A no-op in a build without FTXUI.
  RunExit run(Runtime &rt);
  RunExit run(Runtime &rt, const RunOptions &opts);

  // The same loop over a full application: drains `app` (actions, scheduler slice, on_tick) every
  // iteration, forwards keys and pointer events the UI did not use to the document's on_key /
  // on_pointer, and returns once app.should_close() (e.g. a "quit" action) or on an interrupt the
  // document does not handle.
  RunExit run(AppRuntime &app);
  RunExit run(AppRuntime &app, const RunOptions &opts);

  // Whether this build was compiled with FTXUI.
  static bool available();

protected:
  // Hand an event the UI did not use to the document's on_key / on_pointer, the way
  // run(AppRuntime&) does: keys as a press + release pair with SDL's modifier bits, pointer events
  // at the cell's centre in px.
  void forward_to_document(AppRuntime &app, const TermEvent &e) const;

private:
  struct Impl;
  std::unique_ptr<Impl> m_;
  RunExit run_loop(Runtime &rt, const RunOptions &opts, const std::function<void()> &drain,
                   const std::function<bool()> &should_close,
                   const std::function<void(const TermEvent &)> &unhandled);
};

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_FTXUI_BACKEND_H
