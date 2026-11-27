// Ariadne documents nested far deeper than the recursive walk survived draw on a small stack.
//
// The walk (Runtime::render: emit, emit_node, emit_container, a repeat's instances, the menus, the
// tray, the context menus) keeps its place in a work stack on the heap rather than in native
// frames, and a Widget copies and frees its tree a node at a time, so neither recurses once per
// level of the tree. The recursive walk took about 19 KB of stack a widget level (MSVC Release): it
// drew 10 levels on 256 KiB and 435 on 8 MiB. Each test draws on a thread with a small fixed stack
// (kStackKiB), with a counting backend and, where libcvc has FTXUI, with FtxuiBackend and
// TvisionBackend, so a walk that recursed once a level would fail here on every platform.
//
// The loader still recurses, a level of the document at a time (yaml-cpp's parser, the schema
// check, parse_widget), so most documents here load on the test's main thread and draw on the small
// stack. Its parse takes about 1 KB of stack a widget level in MSVC Release, and a document nests
// at most kMaxNesting (256) levels (docs/ARIADNE.md §6.7): one at the limit loads, and draws, on
// the 2 MiB stack a browser build of ari runs on.
//
// For probing, CVC_ARI_DEEP_DEPTH overrides the depths and CVC_ARI_DEEP_STACK_KIB the stack (0
// runs the body on the calling thread). DISABLED_Probe* load on the small stack, to find where
// the loader runs out (--gtest_also_run_disabled_tests).

#include <algorithm>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <cstdlib>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/ftxui_backend.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/tvision_backend.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <gtest/gtest.h>
#include <string>
#include <utility>
#include <vector>

using namespace cvc::ariadne;

namespace {

constexpr std::size_t kStackKiB = 256;
// Widget levels the drawn trees nest, by default (the tests that each level costs more pick less).
constexpr std::size_t kDepth = 5000;
// include: levels a loaded document nests (a unit per level, each its group around the next). The
// loader recurses a level at a time, and these load on the main thread: 40 is what
// AriadneLoader.ADocumentNested40DeepThroughIncludesLoadsAndDraws loads there on every CI lane.
constexpr std::size_t kIncludeDepth = 40;

std::size_t env_size(const char *name, std::size_t fallback) {
  const char *s = std::getenv(name);
  return s && *s ? static_cast<std::size_t>(std::strtoull(s, nullptr, 10)) : fallback;
}

std::size_t depth(std::size_t fallback = kDepth) {
  return env_size("CVC_ARI_DEEP_DEPTH", fallback);
}

// Run `body` on a thread with a `kib` stack (kStackKiB, or CVC_ARI_DEEP_STACK_KIB, by default) and
// rethrow what it throws.
void on_small_stack(const std::function<void()> &body, std::size_t kib = 0) {
  if (kib == 0)
    kib = env_size("CVC_ARI_DEEP_STACK_KIB", kStackKiB);
  if (kib == 0) {
    body();
    return;
  }
  std::exception_ptr error;
  boost::thread::attributes attrs;
  attrs.set_stack_size(kib * 1024);
  boost::thread t(attrs, [&] {
    try {
      body();
    } catch (...) {
      error = std::current_exception();
    }
  });
  t.join();
  if (error)
    std::rethrow_exception(error);
}

// Counts what the walk opens and closes (every begin_* it gets true from, every push_id) and
// what it draws. Menus, the tray menus and context menus all open, so the walk goes into all of
// them; the tray fits one icon, so the rest go to its overflow menu.
struct CountingBackend : Backend {
  int open = 0, deepest = 0; // begin_*s not yet ended
  int ids = 0, deepest_ids = 0;
  int stray = 0; // an end_* or pop_id with nothing open
  int frames = 0, texts = 0, items = 0, cells = 0, windows = 0, menus = 0, tabs = 0;
  std::vector<std::string> drawn; // the texts and menu items, in order

  void begin() { deepest = std::max(deepest, ++open); }
  void end() {
    if (open == 0)
      ++stray;
    else
      --open;
  }
  bool saw(const std::string &s) const {
    return std::find(drawn.begin(), drawn.end(), s) != drawn.end();
  }
  int times(const std::string &s) const {
    return static_cast<int>(std::count(drawn.begin(), drawn.end(), s));
  }
  // Everything opened was closed, and nothing closed that was not open.
  void expect_balanced() const {
    EXPECT_EQ(open, 0);
    EXPECT_EQ(ids, 0);
    EXPECT_EQ(stray, 0);
  }

  Capabilities capabilities() const override {
    Capabilities c;
    c.windows = true;
    c.menubar = true;
    return c;
  }
  void begin_frame() override { ++frames; }
  bool begin_main_menu_bar() override {
    begin();
    return true;
  }
  void end_main_menu_bar() override { end(); }
  bool begin_menu(const char *) override {
    ++menus;
    begin();
    return true;
  }
  void end_menu() override { end(); }
  int begin_tray(const std::vector<TrayIcon> &) override {
    begin();
    return 1;
  }
  bool begin_tray_menu(const TrayIcon &) override {
    begin();
    return true;
  }
  void end_tray_menu() override { end(); }
  void end_tray() override { end(); }
  bool begin_window(const char *, const char *, const Size &, float, bool *) override {
    ++windows;
    begin();
    return true;
  }
  void end_window() override { end(); }
  void push_id(const char *) override { deepest_ids = std::max(deepest_ids, ++ids); }
  void pop_id() override {
    if (ids == 0)
      ++stray;
    else
      --ids;
  }
  bool begin_grid(const Layout &, const char *) override {
    begin();
    return true;
  }
  void grid_next_cell() override { ++cells; }
  void end_grid() override { end(); }
  void begin_disabled() override { begin(); }
  void end_disabled() override { end(); }
  void begin_item_group(const char *, const ItemGroupOptions &) override { begin(); }
  void end_item_group() override { end(); }
  void begin_context_area(const char *, ContextMenuOver) override { begin(); }
  void end_context_area() override { end(); }
  bool begin_context_menu(const char *, ContextMenuOver, ContextMenuOpen &opened) override {
    opened.now = true;
    begin();
    return true;
  }
  void end_context_menu() override { end(); }
  void begin_backdrop(const Backdrop &) override { begin(); }
  void end_backdrop() override { end(); }
  void begin_text_color(const float[3]) override { begin(); }
  void end_text_color() override { end(); }
  void text_line(const char *t) override {
    ++texts;
    drawn.emplace_back(t);
  }
  void text_value(const char *l, const std::string &v) override {
    ++texts;
    drawn.push_back(std::string(l) + ": " + v);
  }
  void separator() override {}
  bool button(const char *) override { return false; }
  bool menu_item_action(const char *l) override {
    ++items;
    drawn.emplace_back(l);
    return false;
  }
  BoolEdit menu_item_toggle(const char *, bool) override { return {}; }
  BoolEdit checkbox(const char *, bool) override { return {}; }
  IntEdit slider_int(const char *, const char *, int, int, int) override { return {}; }
  DoubleEdit slider_double(const char *, const char *, double, double, double,
                           const char *) override {
    return {};
  }
  IndexEdit combo(const char *, int, const std::vector<std::string> &) override { return {}; }
  IndexEdit tab_bar(const char *, const char *, int current,
                    const std::vector<std::string> &) override {
    ++tabs;
    IndexEdit e;
    e.index = current;
    return e;
  }
  StringEdit input_text(const char *, const char *, const std::string &, bool) override {
    return {};
  }
};

// `w` as a list of one, moved in: a braced list would copy it (an initializer_list's elements are
// const), a whole subtree at every level.
std::vector<Widget> one(Widget w) {
  std::vector<Widget> v;
  v.push_back(std::move(w));
  return v;
}

std::vector<Widget> two(Widget a, Widget b) {
  std::vector<Widget> v = one(std::move(a));
  v.push_back(std::move(b));
  return v;
}

// `items` as `w`'s right-click menu.
Widget with_menu(Widget w, std::vector<Widget> items) {
  w.context_menu = true;
  w.context_items = std::move(items);
  return w;
}

// `inner` wrapped in `n` levels made by `wrap` (given the level, 0 outermost, and what it wraps).
// Built from the inside out, so building it does not recurse either.
Widget nest(std::size_t n, Widget inner, const std::function<Widget(std::size_t, Widget)> &wrap) {
  for (std::size_t i = n; i-- > 0;)
    inner = wrap(i, std::move(inner));
  return inner;
}

Widget plain_groups(std::size_t n, const std::string &deepest) {
  return nest(n, text(deepest), [](std::size_t, Widget w) { return group(one(std::move(w))); });
}

// How many levels deep `w` nests along its last children.
std::size_t chain_depth(const Widget &w) {
  std::size_t n = 0;
  for (const Widget *at = &w; !at->children.empty(); at = &at->children.back())
    ++n;
  return n;
}

// Draw `root` `frames` times on a counting backend, on the small stack.
CountingBackend draw(Widget root, int frames = 2) {
  CountingBackend b;
  on_small_stack([&] {
    cvc::app app;
    Runtime rt(app);
    rt.set_backend(&b);
    rt.set_root(std::move(root));
    for (int i = 0; i < frames; ++i)
      rt.render();
    // The Runtime, and the tree it holds, go here too.
  });
  return b;
}

// Draw `root` with FTXUI on the small stack and return the screen.
std::string draw_ftxui(Widget root, int width = 60) {
  std::string screen;
  on_small_stack([&] {
    cvc::app app;
    Runtime rt(app);
    FtxuiBackend b;
    rt.set_backend(&b);
    rt.set_root(std::move(root));
    screen = b.render_to_string(rt, width, 0);
  });
  return screen;
}

#define SKIP_WITHOUT_FTXUI()                                                                       \
  do {                                                                                             \
    if (!FtxuiBackend::available())                                                                \
      GTEST_SKIP() << "libcvc built without FTXUI";                                                \
  } while (0)

#define SKIP_WITHOUT_YAML()                                                                        \
  do {                                                                                             \
    cvc::app yaml_probe;                                                                           \
    if (!load_string(yaml_probe, "meta: { min_libcvc: \"0.0.0\" }\n").ok)                          \
      GTEST_SKIP() << "libcvc built without yaml-cpp";                                             \
  } while (0)

// ---- Widget ------------------------------------------------------------------------------------

TEST(AriadneDeepNesting, WidgetCopiesAndFreesADeepTreeOnASmallStack) {
  on_small_stack([] {
    const std::size_t n = depth(20000);
    Widget w = with_menu(plain_groups(n, "deepest"),
                         two(plain_groups(n / 10, "item"), menu_action("Copy", "copy")));
    Widget copy = w;
    EXPECT_EQ(chain_depth(copy), n);
    EXPECT_EQ(copy.context_items.size(), 2u);
    EXPECT_EQ(chain_depth(copy.context_items.front()), n / 10);
    copy = copy.children.front(); // from inside its own tree
    EXPECT_EQ(chain_depth(copy), n - 1);
    w = Widget{};
  });
}

// ---- the walk ----------------------------------------------------------------------------------

TEST(AriadneDeepNesting, PlainGroups) {
  const std::size_t n = depth(10000);
  CountingBackend b = draw(group(two(plain_groups(n, "deepest"), text("after"))));
  EXPECT_EQ(b.frames, 2);
  EXPECT_EQ(b.times("deepest"), 2);
  EXPECT_EQ(b.times("after"), 2);
  ASSERT_GE(b.drawn.size(), 2u);
  EXPECT_EQ(b.drawn[0], "deepest"); // in tree order
  EXPECT_EQ(b.drawn[1], "after");
  b.expect_balanced();
}

// Each level one of the containers the walk opens: a group with an id, a pointer handler, a
// right-click menu and a disabled scope; a grid (the level a cell, beside a text); a tab bar over
// one tab; a mount (scoped, with functions); a window. Every begin_* the walk opened is ended.
TEST(AriadneDeepNesting, EveryContainerKind) {
  const std::size_t n = depth(3000);
  auto functions = std::make_shared<FunctionScope>();
  functions->prelude = "(begin)";
  Widget root = nest(n, text("deepest"), [&](std::size_t i, Widget w) {
    switch (i % 6) {
    case 0: {
      Widget g = with_menu(group(one(std::move(w))),
                           one(menu_action("Open " + std::to_string(i), "open")));
      g.id = "g" + std::to_string(i);
      g.on_click = "(begin)";
      g.enabled_when = "(= 1 2)";
      return g;
    }
    case 1: {
      Widget g = group(two(text("cell " + std::to_string(i)), std::move(w)));
      g.layout.kind = LayoutKind::Grid;
      g.layout.col_widths = {Track{}, Track{}};
      return g;
    }
    case 2:
      return tabs("", {"tab " + std::to_string(i)}, one(std::move(w)));
    case 3: {
      Widget g = group(one(std::move(w)));
      g.scope = "m";
      g.functions = functions;
      return g;
    }
    case 4:
      return window("Win " + std::to_string(i), one(std::move(w)));
    default: {
      Widget g = group(one(std::move(w)));
      g.layout.kind = LayoutKind::Horizontal;
      return g;
    }
    }
  });
  CountingBackend b = draw(std::move(root));
  EXPECT_EQ(b.times("deepest"), 2);
  EXPECT_EQ(b.times("Open 0"), 2); // the outermost group's menu, after everything inside it
  EXPECT_EQ(b.times("Open " + std::to_string((n - 1) / 6 * 6)), 2); // and the innermost's
  EXPECT_GE(b.deepest, static_cast<int>(n / 2));
  EXPECT_GE(b.tabs, static_cast<int>(2 * (n / 6)));
  EXPECT_GE(b.windows, static_cast<int>(2 * (n / 6)));
  b.expect_balanced();
}

// A menubar whose menu opens submenus n deep, and a tray icon whose items do, both in the
// overflow menu and in the bar: every menu is opened, so the walk goes to the bottom of each.
TEST(AriadneDeepNesting, MenusAndTrayMenus) {
  const std::size_t n = depth(5000);
  const auto submenus = [n](const std::string &bottom) {
    return nest(n, menu_action(bottom, "go"), [](std::size_t i, Widget w) {
      return menu("Menu " + std::to_string(i), one(std::move(w)));
    });
  };
  std::vector<Widget> entries = one(submenus("deepest menu item"));
  entries.push_back(tray_icon("A", "", "", one(submenus("deepest tray item A"))));
  entries.push_back(tray_icon("B", "", "", one(submenus("deepest tray item B"))));
  CountingBackend b = draw(group(two(menubar(std::move(entries)), text("after"))));
  EXPECT_EQ(b.times("deepest menu item"), 2);
  EXPECT_EQ(b.times("deepest tray item A"), 2); // in the overflow menu (the tray fits one)
  EXPECT_EQ(b.times("deepest tray item B"), 2);
  EXPECT_EQ(b.times("after"), 2);
  EXPECT_GE(b.deepest, static_cast<int>(n));
  b.expect_balanced();
}

// A right-click menu whose item is a group with its own right-click menu, n deep. (Each menu's key
// spells out its place in the tree, so a level costs more the deeper it is.)
TEST(AriadneDeepNesting, ContextMenusInContextMenus) {
  const std::size_t n = depth(2000);
  Widget root = nest(n, text("deepest"), [](std::size_t, Widget w) {
    return with_menu(group(one(text("body"))), one(std::move(w)));
  });
  CountingBackend b = draw(group(two(std::move(root), text("after"))));
  EXPECT_EQ(b.times("deepest"), 2);
  EXPECT_EQ(b.times("after"), 2);
  EXPECT_GE(b.deepest, static_cast<int>(n));
  b.expect_balanced();
}

// A repeat over a deep template copies the template for each instance, each frame; repeats
// nested in it each make their instances inside the outer one's.
TEST(AriadneDeepNesting, RepeatsOverADeepTemplate) {
  const std::size_t n = depth(5000);
  Widget inner = group(one(plain_groups(n, "deepest {i}")));
  inner.repeat = "2";
  Widget middle = group(two(plain_groups(n / 2, "middle"), std::move(inner)));
  middle.repeat = "2";
  Widget root = group(one(std::move(middle)));
  root.layout.kind = LayoutKind::Grid; // the outer repeat's instances are cells
  root.layout.col_widths = {Track{}, Track{}};
  CountingBackend b = draw(std::move(root));
  // Two middles, each with two inner instances, two frames. The inner {i} is left for the inner
  // repeat (single-level substitution), which makes it its own index.
  EXPECT_EQ(b.times("middle"), 4);
  EXPECT_EQ(b.times("deepest 0"), 4);
  EXPECT_EQ(b.times("deepest 1"), 4);
  EXPECT_EQ(b.cells, 4);
  b.expect_balanced();
}

// ---- FTXUI and tvision --------------------------------------------------------------------------

TEST(AriadneDeepNesting, FtxuiDrawsPlainGroups) {
  SKIP_WITHOUT_FTXUI();
  const std::string screen =
      draw_ftxui(window("Deep", two(plain_groups(depth(10000), "deepest"), text("after"))));
  EXPECT_NE(screen.find("deepest"), std::string::npos) << screen;
  EXPECT_NE(screen.find("after"), std::string::npos) << screen;
}

TEST(AriadneDeepNesting, FtxuiDrawsMenusAndContextMenus) {
  SKIP_WITHOUT_FTXUI();
  const std::size_t n = depth(2000);
  Widget menus = nest(n, menu_action("deepest item", "go"), [](std::size_t i, Widget w) {
    return menu("M" + std::to_string(i), one(std::move(w)));
  });
  Widget body = nest(n, text("deepest"), [](std::size_t, Widget w) {
    return with_menu(group(one(std::move(w))), one(menu_action("Copy", "copy")));
  });
  const std::string screen =
      draw_ftxui(group(two(menubar(one(std::move(menus))), window("Deep", one(std::move(body))))));
  EXPECT_NE(screen.find("M0"), std::string::npos) << screen; // the bar's menu
  EXPECT_NE(screen.find("deepest"), std::string::npos) << screen;
}

// Containers FTXUI makes boxes of its own for, nested: grids, item groups (a pointer handler) and
// groups with a right-click menu. The walk opens each, from the heap; FTXUI itself lays its boxes
// out recursively (about 400 bytes a box in Release, 627 nested boxes on 256 KiB), so this stays
// at 100, still ten times the depth the recursive walk drew on this stack.
TEST(AriadneDeepNesting, FtxuiDrawsNestedContainers) {
  SKIP_WITHOUT_FTXUI();
  const std::size_t n = depth(100);
  Widget body = nest(n, text("deepest"), [](std::size_t i, Widget w) {
    Widget g = group(one(std::move(w)));
    if (i % 3 == 0) {
      g.layout.kind = LayoutKind::Grid;
      g.layout.col_widths = {Track{}};
    } else if (i % 3 == 1) {
      g.on_click = "(begin)";
    } else {
      g = with_menu(std::move(g), one(menu_action("Copy", "copy")));
    }
    return g;
  });
  const std::string screen = draw_ftxui(window("Deep", one(std::move(body))));
  EXPECT_NE(screen.find("Deep"), std::string::npos) << screen;
}

TEST(AriadneDeepNesting, TvisionDrawsPlainGroups) {
  SKIP_WITHOUT_FTXUI();
  if (!TvisionBackend::available())
    GTEST_SKIP() << "libcvc built without tvision";
  on_small_stack([] {
    cvc::app app;
    Runtime rt(app);
    TvisionBackend b;
    rt.set_backend(&b);
    rt.set_root(group(one(window("Deep", one(plain_groups(depth(10000), "deepest"))))));
    b.render_to_string(rt, 60, 20);
    ASSERT_EQ(b.hosted_windows().size(), 1u);
    const TermRows rows = b.render_window("Deep", 40, 5);
    std::string text;
    for (const auto &row : rows)
      for (const auto &cell : row)
        text += cell.ch;
    EXPECT_NE(text.find("deepest"), std::string::npos) << text;
  });
}

// ---- loaded documents --------------------------------------------------------------------------

// A document nesting `n` units through include:, each a group around the next, with args (as the
// 40-deep loader test, AriadneLoader.ADocumentNested40DeepThroughIncludesLoadsAndDraws).
std::string include_chain(std::size_t n) {
  std::string doc = "meta: { min_libcvc: \"0.0.0\" }\nunits:\n";
  for (std::size_t i = 0; i < n; ++i) {
    doc += "  level" + std::to_string(i) + ":\n    type: group\n    children:\n";
    if (i + 1 < n)
      doc += "      - include: level" + std::to_string(i + 1) + "\n        args: { name: \"" +
             std::to_string(i + 1) + "\" }\n";
    else
      doc += "      - text: \"deepest {name}\"\n";
  }
  doc += "windows:\n  - window: Deep\n    children:\n      - include: level0\n"
         "        args: { name: \"0\" }\n";
  return doc;
}

// A document whose window nests `n` groups written out inline (in flow style, so its lines do not
// grow an indent a level).
std::string inline_chain(std::size_t n) {
  std::string doc = "meta: { min_libcvc: \"0.0.0\" }\nwindows:\n  - window: Deep\n    children: ";
  for (std::size_t i = 0; i < n; ++i)
    doc += "[{type: group, children: ";
  doc += "[{text: \"deepest\"}]";
  for (std::size_t i = 0; i < n; ++i)
    doc += "}]";
  return doc + "\n";
}

LoadResult load(const std::string &doc) {
  cvc::app app;
  return load_string(app, doc);
}

TEST(AriadneDeepNesting, ADocumentNestedThroughIncludesDraws) {
  SKIP_WITHOUT_YAML();
  const std::size_t n = depth(kIncludeDepth);
  LoadResult r = load(include_chain(n)); // on the main thread: the loader recurses
  ASSERT_TRUE(r.ok) << r.error;
  const std::string deepest = "deepest " + std::to_string(n - 1);
  Widget copy = r.root;
  CountingBackend b = draw(std::move(copy));
  EXPECT_EQ(b.times(deepest), 2);
  b.expect_balanced();
  if (!FtxuiBackend::available())
    return;
  const std::string screen = draw_ftxui(std::move(r.root));
  EXPECT_NE(screen.find(deepest), std::string::npos) << screen;
}

// load: mounts a document in a document, kMaxMountDepth (32) deep at most: each mount a scoped
// wrapper around the loaded document's own tree, with its own scope and init:.
TEST(AriadneDeepNesting, ADocumentNestedThroughLoadsDraws) {
  SKIP_WITHOUT_YAML();
  const std::filesystem::path dir =
      std::filesystem::temp_directory_path() /
      ("ari_deep_loads_" + std::to_string(static_cast<long long>(
                               std::chrono::steady_clock::now().time_since_epoch().count())));
  std::filesystem::create_directories(dir);
  constexpr int kLoads = 31; // the outer document, then 31 mounts: the loader's 32-deep cap
  for (int i = 0; i <= kLoads; ++i) {
    std::ofstream out(dir / ("level" + std::to_string(i) + ".ari"));
    out << "meta: { min_libcvc: \"0.0.0\" }\n";
    out << "init: (state-set \"seen\" \"" << i << "\")\n";
    out << "children:\n  - text: \"level " << i << "\"\n    bind: seen\n";
    if (i < kLoads)
      out << "  - load: level" << (i + 1) << ".ari\n";
  }
  LoadResult r;
  {
    cvc::app app;
    r = load_file(app, (dir / "level0.ari").string());
  }
  std::filesystem::remove_all(dir);
  ASSERT_TRUE(r.ok) << r.error;
  for (const std::string &w : r.warnings)
    EXPECT_EQ(w.find("load:"), std::string::npos) << w;
  CountingBackend b = draw(std::move(r.root));
  // The innermost mount's init: ran in its own scope, before its text was drawn.
  EXPECT_EQ(b.times("level " + std::to_string(kLoads) + ": " + std::to_string(kLoads)), 2);
  b.expect_balanced();
}

// ---- mounts: (docs/roadmap/ARIADNE_SHELL.md 4.2) ------------------------------------------------

// A mounts: slot's entries, opened and drawn on the small stack. One mount's window nests n groups;
// one's menubar opens submenus n deep, its bar a menu of the outer bar; one has no bar, so its
// menu holds only Restart, Quit and Close (4.9). The host's mount loader builds the trees here (a
// loaded document nests at most kMaxNesting levels, §6.7), and the Runtime opens them as each frame
// starts.
TEST(AriadneDeepNesting, AMountsSlotsEntriesDraw) {
  const std::size_t n = depth(3000);
  constexpr int kFrames = 3;
  CountingBackend b;
  std::size_t opened = 0;
  on_small_stack([&] {
    cvc::app app;
    Runtime rt(app);
    rt.set_backend(&b);
    rt.set_mount_loader([n](const MountRequest &rq) {
      MountResult r;
      r.ok = true;
      r.wrapper.scope = "includes." + rq.as;
      r.wrapper.app_name = "App " + rq.as;
      if (rq.uri == "deep-window")
        r.wrapper.children = one(window("Deep", one(plain_groups(n, "deepest in " + rq.as))));
      else if (rq.uri == "deep-menus")
        r.wrapper.children = one(menubar(
            one(nest(n, menu_action("deepest menu item", "go"), [](std::size_t i, Widget w) {
              return menu("Menu " + std::to_string(i), one(std::move(w)));
            }))));
      else
        r.wrapper.children = one(text("no bar"));
      return r;
    });
    Widget slot;
    slot.kind = Kind::Mounts;
    slot.bind = "apps";
    rt.set_root(
        group(two(menubar(one(menu("File", one(menu_action("New", "new"))))), std::move(slot))));
    cvc::state &tree = cvc::state::instance(app);
    tree("apps.a.uri").value(std::string("deep-window"));
    tree("apps.b.uri").value(std::string("deep-menus"));
    tree("apps.c.uri").value(std::string("plain"));
    for (int i = 0; i < kFrames; ++i)
      rt.render();
    opened = rt.mounts().size();
  });
  EXPECT_EQ(opened, 3u);
  EXPECT_EQ(b.times("deepest in a"), kFrames);
  EXPECT_EQ(b.times("deepest menu item"), kFrames);
  EXPECT_EQ(b.times("no bar"), kFrames);
  for (const char *as : {"a", "b", "c"})
    EXPECT_EQ(b.times(std::string("Close App ") + as), kFrames) << as;
  EXPECT_GE(b.deepest, static_cast<int>(n));
  b.expect_balanced();
}

// ---- the nesting limit (docs/ARIADNE.md §6.7) --------------------------------------------------
//
// A document nests at most kMaxNesting levels: a widget a level, and an include: or load: entry a
// level of its own. One at the limit loads, and draws, on the smallest stack a host loads on:
// wasm's 2 MiB (-sSTACK_SIZE for ari's browser build, arigl web, arigl-web, cyprty and the
// examples). An unoptimized build's frames are several times larger, so a Debug build checks
// 8 MiB.
#ifdef NDEBUG
constexpr std::size_t kLimitStackKiB = 2048;
#else
constexpr std::size_t kLimitStackKiB = 8192;
#endif

// Load `doc` (and, when it loads, draw it twice) on a kLimitStackKiB stack.
LoadResult load_and_draw(const std::function<LoadResult(cvc::app &)> &load_doc,
                         CountingBackend &b) {
  LoadResult out;
  on_small_stack(
      [&] {
        cvc::app app;
        LoadResult r = load_doc(app);
        if (r.ok) {
          Runtime rt(app);
          rt.set_backend(&b);
          rt.set_root(std::move(r.root));
          rt.render();
          rt.render();
        }
        out.ok = r.ok;
        out.error = r.error;
        out.error_line = r.error_line;
      },
      kLimitStackKiB);
  return out;
}

TEST(AriadneDeepNesting, ADocumentIncludedToTheNestingLimitLoadsAndDrawsOn2MiB) {
  SKIP_WITHOUT_YAML();
  ASSERT_EQ(kMaxNesting % 2, 0u);
  // n units: the window's include: entry is level 2, each unit two levels, its text at 2n + 2.
  const std::size_t n = kMaxNesting / 2 - 1;
  CountingBackend b;
  const LoadResult r =
      load_and_draw([&](cvc::app &app) { return load_string(app, include_chain(n)); }, b);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(b.times("deepest " + std::to_string(n - 1)), 2);
  b.expect_balanced();

  CountingBackend none;
  const LoadResult past =
      load_and_draw([&](cvc::app &app) { return load_string(app, include_chain(n + 1)); }, none);
  EXPECT_FALSE(past.ok);
  EXPECT_NE(past.error.find("a widget nested deeper than " + std::to_string(kMaxNesting)),
            std::string::npos)
      << past.error;
  EXPECT_GT(past.error_line, 0);
}

// Fragments, each 15 groups written inline around a load: of the next, to the limit: the costliest
// way down a level is a load:'s (its fragment's sub-context, its group).
TEST(AriadneDeepNesting, ADocumentLoadedToTheNestingLimitLoadsAndDrawsOn2MiB) {
  SKIP_WITHOUT_YAML();
  const std::filesystem::path dir =
      std::filesystem::temp_directory_path() /
      ("ari_deep_limit_" + std::to_string(static_cast<long long>(
                               std::chrono::steady_clock::now().time_since_epoch().count())));
  std::filesystem::create_directories(dir);
  const auto groups = [](std::size_t n, const std::string &inner) {
    std::string s;
    for (std::size_t i = 0; i < n; ++i)
      s += "{type: group, children: [";
    s += inner;
    for (std::size_t i = 0; i < n; ++i)
      s += "]}";
    return s;
  };
  // Each fragment 16 levels (15 groups, then the load: entry); the last, its groups and its text.
  const std::size_t loads = kMaxNesting / 16 - 1, last = kMaxNesting - 16 * loads - 1;
  const auto write = [&](std::size_t last_groups) {
    for (std::size_t i = 0; i <= loads; ++i) {
      std::ofstream out(dir / ("level" + std::to_string(i) + ".ari"));
      out << "meta: { min_libcvc: \"0.0.0\" }\nchildren:\n  - "
          << (i < loads ? groups(15, "{load: level" + std::to_string(i + 1) + ".ari}")
                        : groups(last_groups, "{text: deepest}"))
          << "\n";
    }
  };
  const auto load_chain = [&](cvc::app &app) {
    return load_file(app, (dir / "level0.ari").string());
  };
  write(last); // the text at kMaxNesting
  CountingBackend b;
  const LoadResult r = load_and_draw(load_chain, b);
  write(last + 1); // a level past it
  CountingBackend none;
  const LoadResult past = load_and_draw(load_chain, none);
  std::filesystem::remove_all(dir);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(b.times("deepest"), 2);
  b.expect_balanced();
  EXPECT_FALSE(past.ok);
  EXPECT_NE(past.error.find("a widget nested deeper than " + std::to_string(kMaxNesting)),
            std::string::npos)
      << past.error;
}

// ---- probes ------------------------------------------------------------------------------------
//
// Where the loader runs out of stack: these load a document nested depth() deep on the small
// stack. Not run by default (the loader still recurses, a level at a time); for measuring, e.g.
//   CVC_ARI_DEEP_STACK_KIB=256 CVC_ARI_DEEP_DEPTH=10 ariadne_deep_nesting_test
//     --gtest_also_run_disabled_tests --gtest_filter=*Probe*

TEST(AriadneDeepNesting, DISABLED_ProbeLoadIncludes) {
  SKIP_WITHOUT_YAML();
  const std::string doc = include_chain(depth(kIncludeDepth));
  on_small_stack([&] {
    LoadResult r = load(doc);
    EXPECT_TRUE(r.ok) << r.error;
  });
}

TEST(AriadneDeepNesting, DISABLED_ProbeLoadInlineGroups) {
  SKIP_WITHOUT_YAML();
  const std::size_t n = depth(kIncludeDepth);
  const std::string doc = inline_chain(n);
  on_small_stack([&] {
    LoadResult r = load(doc);
    EXPECT_TRUE(r.ok) << r.error;
    EXPECT_GE(chain_depth(r.root), n + 2); // the document's group, its window, then the n groups
  });
}

// The parts of a load that go down the YAML a level at a time without the widget parse: the
// schema check alone (validate_string: yaml-cpp's parse, the document as JSON, the validator),
// and yaml-cpp's parse with the conversion to a Value (parse_value).
TEST(AriadneDeepNesting, DISABLED_ProbeValidateInlineGroups) {
  SKIP_WITHOUT_YAML();
  if (!have_jsonschema())
    GTEST_SKIP() << "libcvc built without the .ari schema validator";
  const std::string doc = inline_chain(depth(kIncludeDepth));
  on_small_stack([&] {
    cvc::app app;
    const ValidationResult v = validate_string(app, doc);
    EXPECT_TRUE(v.ok()) << v.error;
  });
}

TEST(AriadneDeepNesting, DISABLED_ProbeParseInlineGroups) {
  SKIP_WITHOUT_YAML();
  const std::string doc = inline_chain(depth(kIncludeDepth));
  on_small_stack([&] {
    Value v;
    std::string why;
    EXPECT_TRUE(parse_value(doc, v, &why)) << why;
  });
}

// The most stack a refused load takes: a chain of include:s to level 254 whose last unit loads a
// fragment of depth() groups written inline, which yaml-cpp reads and the schema check walks, a
// level of its YAML at a time, at the bottom of the chain, before its parse is refused. (246 groups
// is as deep as yaml-cpp parses; MSVC Release needs more than 1536 KiB for it, and 1792 will do.)
TEST(AriadneDeepNesting, DISABLED_ProbeLoadDeepFragmentAtTheLimit) {
  SKIP_WITHOUT_YAML();
  const std::filesystem::path dir =
      std::filesystem::temp_directory_path() /
      ("ari_probe_fragment_" + std::to_string(static_cast<long long>(
                                   std::chrono::steady_clock::now().time_since_epoch().count())));
  std::filesystem::create_directories(dir);
  std::ofstream(dir / "fragment.ari") << inline_chain(depth(246));
  std::string doc = "meta: { min_libcvc: \"0.0.0\" }\nunits:\n";
  const std::size_t units = kMaxNesting / 2 - 2; // the load: entry at level 254
  for (std::size_t i = 0; i < units; ++i) {
    doc += "  level" + std::to_string(i) + ":\n    type: group\n    children:\n";
    doc += i + 1 < units ? "      - include: level" + std::to_string(i + 1) + "\n"
                         : "      - load: fragment.ari\n";
  }
  std::ofstream(dir / "main.ari")
      << doc << "windows:\n  - window: Deep\n    children:\n      - include: level0\n";
  LoadResult r;
  on_small_stack([&] {
    cvc::app app;
    r = load_file(app, (dir / "main.ari").string());
  });
  std::filesystem::remove_all(dir);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.error.find("a widget nested deeper than"), std::string::npos) << r.error;
}

// Where the walk ran out before: draws a document of depth() include: levels (loaded on the main
// thread) on the small stack, with FTXUI when it is built, else the counting backend.
TEST(AriadneDeepNesting, DISABLED_ProbeDrawIncludes) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load(include_chain(depth(kIncludeDepth)));
  ASSERT_TRUE(r.ok) << r.error;
  if (FtxuiBackend::available())
    EXPECT_NE(draw_ftxui(std::move(r.root)).find("deepest"), std::string::npos);
  else
    EXPECT_EQ(draw(std::move(r.root)).times("deepest " + std::to_string(depth() - 1)), 2);
}

} // namespace
