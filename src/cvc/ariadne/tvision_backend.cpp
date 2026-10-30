// TvisionBackend — see the header. tvision supplies the desktop (windows, menu bar, status line,
// the event loop); each window's content is the FTXUI backend's, in hosted mode.
//
// The bridge, per iteration of tvision's loop:
//   - sync(): run one walk (Runtime::render), then make tvision match it — a TWindow per hosted
//     window (created at the document's placement, destroyed when the document stops drawing it,
//     retitled), the menu bar rebuilt from the hosted menu model when it changes, the background
//     view for widgets outside any window — and report every window's bounds back
//     (set_window_geometry) so a move or resize persists. Then redraw.
//   - a content view's draw() asks the backend to lay its window's body out at the view's size
//     (render_window) and blits the cells; its handleEvent() turns tvision input into TermEvents
//     in window-local cells (feed_window).
//   - chrome actions come back as backend calls: a menu command -> activate_menu_item, a window's
//     close box -> close_window. Destroying a window waits for the next sync, which runs only
//     after the event that asked for it has finished, so no view is deleted under its own feet.

#include <chrono>
#include <cstdlib>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/tvision_backend.h>
#include <map>
#include <string>
#include <vector>

#ifdef CVC_ARIADNE_TVISION
#include <ftxui/screen/terminal.hpp> // SetColorSupport: lay out at full colour; tvision quantizes

#define Uses_TApplication
#define Uses_TBackground
#define Uses_TDeskTop
#define Uses_TDrawBuffer
#define Uses_TEvent
#define Uses_TFrame
#define Uses_TKeys
#define Uses_TMenu
#define Uses_TMenuBar
#define Uses_TMenuItem
#define Uses_TPalette
#define Uses_TProgram
#define Uses_TRect
#define Uses_TStatusDef
#define Uses_TStatusItem
#define Uses_TStatusLine
#define Uses_TSubMenu
#define Uses_TView
#define Uses_TWindow
#include <tvision/tv.h>

#if !defined(_WIN32) && !defined(__EMSCRIPTEN__)
#include <dlfcn.h>
#include <sys/stat.h>
#endif
#endif

namespace cvc {
namespace ariadne {

TvisionBackend::TvisionBackend() : TvisionBackend(Options{}) {}

TvisionBackend::TvisionBackend(Options opts)
    : FtxuiBackend([&] {
        opts.hosted = true;
        return opts;
      }()) {}

Capabilities TvisionBackend::capabilities() const {
  Capabilities c = FtxuiBackend::capabilities();
  c.windows = available(); // native movable / resizable / overlapping windows
  c.menubar = available();
  return c;
}

FtxuiBackend::RunExit TvisionBackend::run(Runtime &rt) { return run(rt, RunOptions{}); }

FtxuiBackend::RunExit TvisionBackend::run(Runtime &rt, const RunOptions &opts) {
  return run_desktop(rt, opts, [&] { rt.drain(); }, nullptr, nullptr);
}

FtxuiBackend::RunExit TvisionBackend::run(AppRuntime &app) { return run(app, RunOptions{}); }

FtxuiBackend::RunExit TvisionBackend::run(AppRuntime &app, const RunOptions &opts) {
  return run_desktop(
      app.runtime(), opts, [&] { app.drain(); }, [&] { return app.should_close(); },
      [&](const TermEvent &e) { forward_to_document(app, e); });
}

#ifdef CVC_ARIADNE_TVISION

namespace {

constexpr ushort cmAriMenuBase = 1000; // menu item k is command cmAriMenuBase + k

#if !defined(_WIN32) && !defined(__EMSCRIPTEN__)
// tvision reads the terminal's description through ncurses. A cvcpkg prefix ships ncurses' own
// terminfo database (share/terminfo) beside libcvc, while the bundle's compiled-in search path
// names the builder's prefix, which no user machine has. So point ncurses at the database next to
// this library, ahead of the system's, unless the user already chose. The trailing ':' keeps
// ncurses' built-in search after it. Done here, not in a host, so every host (ari, arigl, cvcGL's
// ariadne_terminal) gets it.
void use_bundled_terminfo() {
  if (std::getenv("TERMINFO_DIRS"))
    return;
  Dl_info info;
  if (!dladdr(reinterpret_cast<const void *>(&use_bundled_terminfo), &info) || !info.dli_fname)
    return;
  const std::string lib = info.dli_fname;
  const std::size_t slash = lib.find_last_of('/');
  if (slash == std::string::npos)
    return;
  const std::string dir = lib.substr(0, slash) + "/../share/terminfo";
  struct stat st;
  if (stat(dir.c_str(), &st) == 0 && S_ISDIR(st.st_mode))
    setenv("TERMINFO_DIRS", (dir + ":").c_str(), 0);
}
#endif

// ---- colour: the hosted cells carry SGR parameters; tvision wants TColor ------------------------

TColor sgr_color(const std::string &sgr, TColor fallback) {
  if (sgr.empty())
    return fallback;
  std::vector<int> v;
  std::size_t start = 0;
  while (start <= sgr.size()) {
    const std::size_t semi = sgr.find(';', start);
    v.push_back(std::atoi(sgr.substr(start, semi - start).c_str()));
    if (semi == std::string::npos)
      break;
    start = semi + 1;
  }
  if (v.size() >= 5 && (v[0] == 38 || v[0] == 48) && v[1] == 2)
    return TColorRGB(static_cast<uint8_t>(v[2]), static_cast<uint8_t>(v[3]),
                     static_cast<uint8_t>(v[4]));
  if (v.size() >= 3 && (v[0] == 38 || v[0] == 48) && v[1] == 5)
    return TColorXTerm(static_cast<uint8_t>(v[2]));
  const int c = v.empty() ? 0 : v[0];
  if (c >= 30 && c <= 37)
    return TColorXTerm(static_cast<uint8_t>(c - 30));
  if (c >= 90 && c <= 97)
    return TColorXTerm(static_cast<uint8_t>(c - 90 + 8));
  if (c >= 40 && c <= 47)
    return TColorXTerm(static_cast<uint8_t>(c - 40));
  if (c >= 100 && c <= 107)
    return TColorXTerm(static_cast<uint8_t>(c - 100 + 8));
  return fallback; // 39 / 49: the default
}

// ---- tvision input -> TermEvent -----------------------------------------------------------------

bool key_event(const KeyDownEvent &k, TermEvent &out) {
  const bool shift = (k.controlKeyState & kbShift) != 0;
  const bool ctrl = (k.controlKeyState & kbCtrlShift) != 0;
  const bool alt = (k.controlKeyState & kbAltShift) != 0;
  struct Named {
    ushort code;
    const char *name;
    bool shift, ctrl;
  };
  static const Named kNamed[] = {
      {kbTab, "Tab", false, false},        {kbShiftTab, "Tab", true, false},
      {kbEnter, "Return", false, false},   {kbEsc, "Escape", false, false},
      {kbBack, "Backspace", false, false}, {kbDel, "Delete", false, false},
      {kbIns, "Insert", false, false},     {kbLeft, "Left", false, false},
      {kbRight, "Right", false, false},    {kbUp, "Up", false, false},
      {kbDown, "Down", false, false},      {kbCtrlLeft, "Left", false, true},
      {kbCtrlRight, "Right", false, true}, {kbHome, "Home", false, false},
      {kbEnd, "End", false, false},        {kbPgUp, "PageUp", false, false},
      {kbPgDn, "PageDown", false, false},  {kbF1, "F1", false, false},
      {kbF2, "F2", false, false},          {kbF3, "F3", false, false},
      {kbF4, "F4", false, false},          {kbF5, "F5", false, false},
      {kbF7, "F7", false, false},          {kbF8, "F8", false, false},
      {kbF9, "F9", false, false},          {kbF11, "F11", false, false},
      {kbF12, "F12", false, false},
  };
  for (const Named &n : kNamed)
    if (k.keyCode == n.code) {
      out = TermEvent::key_event(n.name, n.shift || shift, n.ctrl || ctrl, alt);
      return true;
    }
  if (k.textLength > 0 && !ctrl && !alt) {
    out = TermEvent::text_event(std::string(k.getText()));
    return true;
  }
  const uchar c = k.charScan.charCode;
  if (ctrl && c >= 1 && c <= 26) {
    out = TermEvent::key_event(std::string(1, static_cast<char>('A' + c - 1)), shift, true, alt);
    return true;
  }
  return false;
}

struct Host;

// The interior of one window (or, for id "", the desktop background): draws the hosted cells and
// feeds input to the backend.
class ContentView : public TView {
public:
  ContentView(const TRect &r, Host &host, std::string id);
  void draw() override;
  void handleEvent(TEvent &ev) override;
  TPalette &getPalette() const override;
  std::string id;

private:
  Host &host_;
  bool feed_mouse(TermEvent::Mouse kind, TPoint where, int button);
};

class ContentWindow : public TWindow {
public:
  ContentWindow(const TRect &r, Host &host, const std::string &id, const std::string &title,
                bool closable)
      : TWindowInit(&TWindow::initFrame), TWindow(r, title, wnNoNumber), id(id), host_(host) {
    flags = wfMove | wfGrow | wfZoom | (closable ? wfClose : 0);
    options |= ofFirstClick; // the click that raises a window also reaches the widget under it
    TRect inner = getExtent();
    inner.grow(-1, -1);
    content = new ContentView(inner, host, id);
    content->growMode = gfGrowHiX | gfGrowHiY;
    insert(content);
  }
  void close() override; // the close box: the document decides (tree.<id>.open)
  ContentView *content = nullptr;
  std::string id;
  std::string shown_title;
  // A window the document did not size fits its content on every walk, as in the FTXUI loop
  // (ARIADNE.md §5.8), in the axes it did not size, until the user resizes it: fitted is the size
  // the host last gave it, so a size that differs is the user's.
  bool fit_w = false, fit_h = false;
  TPoint fitted{0, 0};

private:
  Host &host_;
};

// The menu bar knows when it is executing, so a rebuild never swaps the menu out from under it.
class HostMenuBar : public TMenuBar {
public:
  HostMenuBar(const TRect &r, TMenu *m) : TMenuBar(r, m) {}
  ushort execute() override {
    ++executing;
    const ushort r = TMenuBar::execute();
    --executing;
    return r;
  }
  void replace(TMenu *m) {
    TMenu *old = menu;
    menu = m;
    delete old;
    drawView();
  }
  int executing = 0;
};

class HostStatusLine : public TStatusLine {
public:
  HostStatusLine(const TRect &r, TStatusDef &d) : TStatusLine(r, d) {}
  const char *hint(ushort) override { return tip.c_str(); } // the focused widget's tooltip
  std::string tip;
};

class App : public TApplication {
public:
  explicit App(Host &host)
      : TProgInit(&App::initStatusLine, &App::initMenuBar, &TApplication::initDeskTop),
        host_(host) {}
  static TMenuBar *initMenuBar(TRect r) {
    r.b.y = r.a.y + 1;
    return new HostMenuBar(r, new TMenu());
  }
  static TStatusLine *initStatusLine(TRect r) {
    r.a.y = r.b.y - 1;
    return new HostStatusLine(r, *new TStatusDef(0, 0xFFFF) +
                                     *new TStatusItem("~F10~ Menu", kbF10, cmMenu) +
                                     *new TStatusItem("~F6~ Next", kbF6, cmNext) +
                                     *new TStatusItem("~Alt-X~ Exit", kbAltX, cmQuit));
  }
  void getEvent(TEvent &ev) override;
  void handleEvent(TEvent &ev) override;
  void idle() override;

private:
  Host &host_;
};

struct Host {
  TvisionBackend &be;
  Runtime &rt;
  const FtxuiBackend::RunOptions &opts;
  const std::function<void()> &drain;
  const std::function<bool()> &should_close;
  const std::function<void(const TermEvent &)> &unhandled;

  FtxuiBackend::RunExit result;
  std::map<std::string, ContentWindow *> windows;
  ContentView *background = nullptr;
  std::vector<std::string> menu_items; // command cmAriMenuBase + k -> item id
  std::string menu_signature;
  bool dirty = true; // a sync is due (after input, or the redraw tick)
  std::chrono::steady_clock::time_point last_sync{};
  std::chrono::steady_clock::time_point burst_start{};
  int burst = 0;

  // Interrupt keys are signals, as in the FTXUI loop: the document's on_signal: decides, else they
  // end the run; three Ctrl-Cs in two seconds always do.
  void interrupt(const char *signal) {
    const auto now = std::chrono::steady_clock::now();
    if (now - burst_start > std::chrono::seconds(2)) {
      burst_start = now;
      burst = 0;
    }
    if (++burst >= 3 || !rt.post_signal(signal, "terminal"))
      quit(signal, false);
  }

  void quit(const std::string &signal, bool from_os) {
    if (result.signal.empty() && !signal.empty())
      result = FtxuiBackend::RunExit{signal, from_os};
    TProgram::application->endModal(cmQuit);
  }

  bool feed(const std::string &id, const TermEvent &e) {
    // While the Keys window waits for a key, every key is the document's, as in the FTXUI loop
    // (FtxuiBackend::route): Tab, the arrows and Return would otherwise move the body's focus.
    if (e.kind != TermEvent::Kind::Mouse && unhandled && rt.key_capture_pending()) {
      unhandled(e);
      dirty = true;
      return true;
    }
    bool used = be.feed_window(rt, id, e);
    if (!used && id.empty() && opts.background_input && opts.background_input(e))
      used = true; // e.g. cvcGL's TerminalScene steering its camera from the desktop
    if (!used && unhandled)
      unhandled(e);
    dirty = true;
    return used;
  }

  void activate(ushort command) {
    const std::size_t k = command - cmAriMenuBase;
    if (k < menu_items.size())
      be.activate_menu_item(rt, menu_items[k]);
    dirty = true;
  }

  void close(const std::string &id) {
    be.close_window(rt, id); // the window goes in the next sync, when the document drops it
    dirty = true;
  }

  // One walk, then make the desktop match it.
  void sync() {
    TDeskTop *desk = TProgram::deskTop;
    // Report the windows' current bounds first, so a move / resize persists in this walk.
    for (auto &[id, w] : windows)
      be.set_window_geometry(id, w->origin.x, w->origin.y, w->size.x, w->size.y);
    be.set_screen_size(desk->size.x, desk->size.y);
    rt.render();
    dirty = false;
    last_sync = std::chrono::steady_clock::now();

    std::map<std::string, bool> seen;
    for (const FtxuiBackend::HostedWindow &hw : be.hosted_windows()) {
      seen[hw.id] = true;
      auto it = windows.find(hw.id);
      // The outer size for a body of the given natural size, on this desktop.
      const auto fit_w = [&](int content) {
        return std::max(16, std::min(content + 2, desk->size.x));
      };
      const auto fit_h = [&](int content) {
        return std::max(4, std::min(content + 2, desk->size.y));
      };
      if (it == windows.end()) {
        const int w = hw.w > 0 ? std::max(16, std::min(hw.w, desk->size.x)) : fit_w(hw.content_w);
        const int h = hw.h > 0 ? std::max(4, std::min(hw.h, desk->size.y)) : fit_h(hw.content_h);
        TRect r(hw.x, hw.y, hw.x + w, hw.y + h);
        auto *win = new ContentWindow(r, *this, hw.id, hw.title, hw.closable);
        win->shown_title = hw.title;
        win->fit_w = hw.w <= 0;
        win->fit_h = hw.h <= 0;
        win->fitted = win->size;
        desk->insert(win);
        it = windows.emplace(hw.id, win).first;
      } else if (ContentWindow *win = it->second; win->fit_w || win->fit_h) {
        if (win->size != win->fitted) { // the user resized it: theirs from now on
          win->fit_w = win->fit_h = false;
        } else {
          const TPoint want{win->fit_w ? fit_w(hw.content_w) : win->size.x,
                            win->fit_h ? fit_h(hw.content_h) : win->size.y};
          if (want != win->size) {
            TRect r(win->origin.x, win->origin.y, win->origin.x + want.x, win->origin.y + want.y);
            win->locate(r);
          }
          win->fitted = win->size;
        }
      }
      if (it->second->shown_title != hw.title) {
        ContentWindow *win = it->second;
        delete[] const_cast<char *>(win->title);
        win->title = newStr(hw.title);
        win->shown_title = hw.title;
        win->frame->drawView();
      }
    }
    for (auto it = windows.begin(); it != windows.end();) {
      if (seen.count(it->first)) {
        ++it;
        continue;
      }
      TObject::destroy(it->second); // the document stopped drawing it (e.g. closed)
      it = windows.erase(it);
    }

    // Widgets outside every window live on a view just in front of the desktop's background.
    if (be.hosted_root() && !background) {
      background = new ContentView(desk->getExtent(), *this, "");
      background->growMode = gfGrowHiX | gfGrowHiY;
      desk->insert(background);
      if (desk->background) // just in front of tvision's own backdrop: behind every window
        background->putInFrontOf(desk->background);
    } else if (!be.hosted_root() && background) {
      TObject::destroy(background);
      background = nullptr;
    }

    sync_menus();
    if (auto *status = dynamic_cast<HostStatusLine *>(TProgram::statusLine)) {
      const std::string tip = be.tooltip();
      if (tip != status->tip) {
        status->tip = tip;
        status->drawView();
      }
    }
    for (auto &[id, w] : windows)
      w->content->drawView();
    if (background)
      background->drawView();
  }

  void sync_menus() {
    auto *bar = dynamic_cast<HostMenuBar *>(TProgram::menuBar);
    if (!bar || bar->executing)
      return;
    const std::vector<FtxuiBackend::HostedMenu> menus = be.hosted_menus();
    std::string sig;
    for (const auto &m : menus) {
      sig += m.label + '\x1f';
      for (const auto &i : m.items)
        sig += i.id + (i.checked ? "+" : "-") + i.label + '\x1e';
    }
    if (sig == menu_signature)
      return;
    menu_signature = sig;
    menu_items.clear();
    TMenuItem *first = nullptr, **next = &first;
    for (const auto &m : menus) {
      const char hot = m.label.empty() ? 0 : static_cast<char>(std::toupper(m.label[0]));
      auto *sub = new TSubMenu("~" + m.label.substr(0, 1) + "~" + m.label.substr(1),
                               hot ? TKey(hot, kbAltShift) : TKey(kbNoKey));
      TMenuItem **inner = &sub->subMenu->items;
      for (const auto &i : m.items) {
        if (i.separator) {
          *inner = &newLine();
        } else {
          const std::string label = (i.toggle ? (i.checked ? "[x] " : "[ ] ") : "") + i.label;
          *inner =
              new TMenuItem(label, static_cast<ushort>(cmAriMenuBase + menu_items.size()), kbNoKey);
          menu_items.push_back(i.id);
        }
        inner = &(*inner)->next;
      }
      *next = sub;
      next = &sub->next;
    }
    bar->replace(first ? new TMenu(*first) : new TMenu());
  }
};

ContentView::ContentView(const TRect &r, Host &host, std::string id)
    : TView(r), id(std::move(id)), host_(host) {
  options |= ofSelectable | ofFirstClick;
  eventMask |= evMouseMove | evMouseAuto | evMouseUp | evMouseWheel;
}

TPalette &ContentView::getPalette() const {
  // Inside a window: its scroller text colours (TWindow palette entries 6-7), like TScroller.
  static TPalette window_text("\x06\x07", 2);
  static TPalette desktop_text("\x01\x01", 2);
  return id.empty() ? desktop_text : window_text;
}

void ContentView::draw() {
  const TColorAttr normal = getColor(1);
  const TermRows rows = host_.be.render_window(id, size.x, size.y);
  for (int y = 0; y < size.y; ++y) {
    TDrawBuffer b;
    b.moveChar(0, ' ', normal, size.x);
    if (y < static_cast<int>(rows.size())) {
      const std::vector<TermCell> &row = rows[static_cast<std::size_t>(y)];
      for (int x = 0; x < size.x && x < static_cast<int>(row.size()); ++x) {
        const TermCell &c = row[static_cast<std::size_t>(x)];
        if (c.ch.empty())
          continue; // the second half of a wide glyph
        TColor fg = sgr_color(c.fg, normal.getForeground());
        TColor bg = sgr_color(c.bg, normal.getBackground());
        if (c.inverted)
          std::swap(fg, bg);
        const ushort style =
            (c.bold ? slBold : 0) | (c.italic ? slItalic : 0) | (c.underlined ? slUnderline : 0);
        b.moveStr(static_cast<ushort>(x), c.ch, TColorAttr(fg, bg, style));
      }
    }
    writeLine(0, static_cast<short>(y), static_cast<short>(size.x), 1, b);
  }
}

bool ContentView::feed_mouse(TermEvent::Mouse kind, TPoint where, int button) {
  const TPoint p = makeLocal(where);
  return host_.feed(id, TermEvent::mouse_event(kind, p.x, p.y, button));
}

void ContentView::handleEvent(TEvent &ev) {
  TView::handleEvent(ev); // selects (focuses) the view on a click
  if (ev.what == evMouseDown) {
    const int button = (ev.mouse.buttons & mbLeftButton) ? 1 : (ev.mouse.buttons & 0x02) ? 3 : 2;
    feed_mouse(TermEvent::Mouse::Press, ev.mouse.where, button);
    drawView();
    // tvision's drag idiom: follow the pointer until the button comes up.
    TPoint last = ev.mouse.where;
    while (mouseEvent(ev, evMouseMove | evMouseAuto)) {
      if (ev.mouse.where != last) {
        last = ev.mouse.where;
        feed_mouse(TermEvent::Mouse::Move, last, button);
        drawView();
      }
    }
    feed_mouse(TermEvent::Mouse::Release, last, button);
    drawView();
    clearEvent(ev);
  } else if (ev.what == evMouseMove) {
    feed_mouse(TermEvent::Mouse::Move, ev.mouse.where, 0);
  } else if (ev.what == evMouseWheel) {
    feed_mouse(ev.mouse.wheel == mwUp ? TermEvent::Mouse::WheelUp : TermEvent::Mouse::WheelDown,
               ev.mouse.where, 0);
    drawView();
    clearEvent(ev);
  } else if (ev.what == evKeyDown) {
    TermEvent te;
    if (key_event(ev.keyDown, te) && host_.feed(id, te)) {
      drawView();
      clearEvent(ev);
    }
  }
}

void ContentWindow::close() {
  if (flags & wfClose)
    host_.close(id);
}

void App::getEvent(TEvent &ev) {
  TApplication::getEvent(ev);
  if (ev.what == evKeyDown &&
      (ev.keyDown.keyCode == kbCtrlC || ev.keyDown.charScan.charCode == '\x1c')) {
    host_.interrupt(ev.keyDown.keyCode == kbCtrlC ? "SIGINT" : "SIGQUIT");
    clearEvent(ev);
  }
}

void App::handleEvent(TEvent &ev) {
  if (ev.what == evCommand && ev.message.command >= cmAriMenuBase) {
    host_.activate(ev.message.command);
    clearEvent(ev);
  }
  TApplication::handleEvent(ev);
  if (host_.dirty) // after the event has finished with every view, so a dropped window can go
    host_.sync();
}

void App::idle() {
  TApplication::idle();
  if (host_.opts.take_signal)
    for (std::string sig = host_.opts.take_signal(); !sig.empty(); sig = host_.opts.take_signal())
      if (!host_.rt.post_signal(sig, "os"))
        host_.quit(sig, true);
  if (host_.drain)
    host_.drain(); // queued actions run here, off the walk (§7.2)
  if (host_.should_close && host_.should_close()) {
    host_.quit("", false);
    return;
  }
  const auto period = std::chrono::milliseconds(1000 / std::max(1, host_.opts.fps));
  if (host_.dirty || std::chrono::steady_clock::now() - host_.last_sync >= period)
    host_.sync(); // the redraw tick, so state changed off the UI shows
}

} // namespace

FtxuiBackend::RunExit
TvisionBackend::run_desktop(Runtime &rt, const RunOptions &opts, const std::function<void()> &drain,
                            const std::function<bool()> &should_close,
                            const std::function<void(const TermEvent &)> &unhandled) {
#if !defined(_WIN32) && !defined(__EMSCRIPTEN__) // the browser build has no ncurses
  use_bundled_terminfo(); // before TApplication starts ncurses
#endif
  // Lay window bodies out at full colour; tvision quantizes to what the terminal can show.
  const ftxui::Terminal::Color colors = ftxui::Terminal::ColorSupport();
  ftxui::Terminal::SetColorSupport(ftxui::Terminal::Color::TrueColor);
  Host host{*this, rt, opts, drain, should_close, unhandled};
  {
    TProgram::eventTimeoutMs = std::max(1, 1000 / std::max(1, opts.fps));
    App app(host);
    if (opts.on_start)
      opts.on_start();
    host.sync();
    app.run();
    if (opts.on_stop)
      opts.on_stop();
    app.shutDown();
  }
  ftxui::Terminal::SetColorSupport(colors);
  return host.result;
}

bool TvisionBackend::available() { return true; }

#else // !CVC_ARIADNE_TVISION

FtxuiBackend::RunExit TvisionBackend::run_desktop(Runtime &, const RunOptions &,
                                                  const std::function<void()> &,
                                                  const std::function<bool()> &,
                                                  const std::function<void(const TermEvent &)> &) {
  return {};
}

bool TvisionBackend::available() { return false; }

#endif

} // namespace ariadne
} // namespace cvc
