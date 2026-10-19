// Ariadne FTXUI terminal backend — headless tests. Each test mounts a widget tree on a Runtime over
// a real FtxuiBackend, renders it to text (render_to_string), and drives it with TermEvents (feed),
// the same path the interactive loop takes — no TTY. They check what reaches cvc::state through the
// core's commit protocol: focus and activation, slider / combo / text edits and their commit edges,
// menus, window placement / drag / close and its §11.4 persistence, disabled scopes, images.
// Skipped when libcvc is built without FTXUI.

#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/ftxui_backend.h>
#include <cvc/ariadne/tvision_backend.h>
#include <cvc/core/app.h>
#include <cvc/image/image.h>
#include <cvc/state/state.h>
#include <filesystem>
#include <gtest/gtest.h>
#include <string>
#include <utility>
#include <vector>

using namespace cvc::ariadne;

namespace {

#define REQUIRE_FTXUI()                                                                            \
  do {                                                                                             \
    if (!FtxuiBackend::available())                                                                \
      GTEST_SKIP() << "libcvc built without FTXUI";                                                \
  } while (0)

std::string val(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app)(path).value();
}

// The cell (column, row) where `needle` first appears on the rendered screen. Columns count UTF-8
// code points (every glyph these tests look for is one cell wide).
bool find_cell(const std::string &screen, const std::string &needle, int &x, int &y) {
  int row = 0;
  std::size_t start = 0;
  while (start < screen.size()) {
    const std::size_t nl = screen.find('\n', start);
    const std::string line =
        screen.substr(start, nl == std::string::npos ? std::string::npos : nl - start);
    const std::size_t at = line.find(needle);
    if (at != std::string::npos) {
      int col = 0;
      for (std::size_t i = 0; i < at; ++i)
        if ((static_cast<unsigned char>(line[i]) & 0xC0) != 0x80)
          ++col;
      x = col;
      y = row;
      return true;
    }
    if (nl == std::string::npos)
      break;
    start = nl + 1;
    ++row;
  }
  return false;
}

// A Runtime over a real FtxuiBackend. Member order matters: the backend outlives the Runtime.
struct Term {
  cvc::app app;
  FtxuiBackend be;
  Runtime rt{app, ""};
  std::string screen;
  int width = 80, height = 24;

  explicit Term(Widget root, FtxuiBackend::Options opts = {}) : be(opts) {
    rt.set_backend(&be);
    rt.set_root(std::move(root));
    draw();
  }
  const std::string &draw() {
    screen = be.render_to_string(rt, width, height);
    return screen;
  }
  bool key(const char *k, bool shift = false) {
    return be.feed(rt, TermEvent::key_event(k, shift));
  }
  bool type(const std::string &s) {
    bool used = true;
    for (char c : s)
      used = be.feed(rt, TermEvent::text_event(std::string(1, c))) && used;
    return used;
  }
  bool mouse(TermEvent::Mouse m, int x, int y) {
    return be.feed(rt, TermEvent::mouse_event(m, x, y));
  }
  // Left-click the first cell of `needle` (+dx) on the current screen, then redraw.
  bool click(const std::string &needle, int dx = 0) {
    int x = 0, y = 0;
    if (!find_cell(screen, needle, x, y))
      return false;
    mouse(TermEvent::Mouse::Press, x + dx, y);
    mouse(TermEvent::Mouse::Release, x + dx, y);
    draw();
    return true;
  }
  bool shows(const std::string &s) const { return screen.find(s) != std::string::npos; }
};

} // namespace

TEST(AriadneFtxui, RendersMenubarWindowAndWidgets) {
  REQUIRE_FTXUI();
  Term t(group({menubar({menu("File", {menu_action("Quit", "quit")})}),
                window("Controls", {text("hello"), checkbox("Wire", "wire"),
                                    slider_int("Agents", "agents", 0, 10, 3),
                                    combo("Mode", "mode", {"a", "b"}), button("Go", "go")})}));
  EXPECT_TRUE(t.shows("File")) << t.screen;
  EXPECT_TRUE(t.shows("Controls")) << t.screen;
  EXPECT_TRUE(t.shows("hello")) << t.screen;
  EXPECT_TRUE(t.shows("[ ] Wire")) << t.screen;
  EXPECT_TRUE(t.shows(" 3")) << t.screen;
  EXPECT_TRUE(t.shows("Mode: < a >")) << t.screen;
  EXPECT_TRUE(t.shows("[ Go ]")) << t.screen;
  EXPECT_FALSE(t.shows("Quit")) << "a closed menu shows only its title\n" << t.screen;
  EXPECT_TRUE(t.be.capabilities().owns_loop);
  EXPECT_TRUE(t.be.capabilities().windows);
}

TEST(AriadneFtxui, TabFocusesAndSpaceOrEnterToggles) {
  REQUIRE_FTXUI();
  Term t(group({checkbox("A", "a"), checkbox("B", "b")}));
  EXPECT_TRUE(t.key("Tab")); // focus A
  EXPECT_TRUE(t.type(" "));  // toggle it
  EXPECT_EQ(val(t.app, "a"), "1");
  EXPECT_EQ(val(t.app, "b"), "0");
  EXPECT_TRUE(t.key("Tab"));    // focus B
  EXPECT_TRUE(t.key("Return")); // toggle it
  EXPECT_EQ(val(t.app, "b"), "1");
  EXPECT_TRUE(t.key("Tab", /*shift=*/true)); // back to A
  EXPECT_TRUE(t.type(" "));
  EXPECT_EQ(val(t.app, "a"), "0");
}

TEST(AriadneFtxui, UpDownMoveFocusWhenNoWidgetTakesThem) {
  REQUIRE_FTXUI();
  Term t(group({checkbox("A", "a"), checkbox("B", "b")}));
  EXPECT_TRUE(t.key("Down")); // A
  EXPECT_TRUE(t.key("Down")); // B
  t.type(" ");
  EXPECT_EQ(val(t.app, "a"), "0");
  EXPECT_EQ(val(t.app, "b"), "1");
}

TEST(AriadneFtxui, ClickTogglesCheckbox) {
  REQUIRE_FTXUI();
  Term t(group({checkbox("A", "a"), checkbox("B", "b")}));
  ASSERT_TRUE(t.click("[ ] B")) << t.screen;
  EXPECT_EQ(val(t.app, "b"), "1");
  EXPECT_TRUE(t.shows("[x] B")) << t.screen;
  EXPECT_EQ(val(t.app, "a"), "0");
}

TEST(AriadneFtxui, UnusedKeysAreReportedForTheDocument) {
  REQUIRE_FTXUI();
  Term t(group({button("Go", "go")}));
  t.key("Tab");
  EXPECT_FALSE(t.type("q"));     // a button has no use for 'q': the host forwards it to on_key
  EXPECT_FALSE(t.key("Escape")); // nothing open, nothing to revert
  EXPECT_TRUE(t.key("Return"));  // ... but the focused button takes Enter
}

TEST(AriadneFtxui, ButtonActionRunsOnDrain) {
  REQUIRE_FTXUI();
  Term t(group({button("Go", "go")}));
  int fired = 0;
  t.rt.on("go", [&] { ++fired; });
  t.key("Tab");
  t.key("Return");
  EXPECT_EQ(fired, 0) << "actions are queued by the walk, never run inside it";
  t.rt.drain();
  EXPECT_EQ(fired, 1);
  ASSERT_TRUE(t.click("[ Go ]"));
  t.rt.drain();
  EXPECT_EQ(fired, 2);
}

TEST(AriadneFtxui, SliderKeysCommitEachStep) {
  REQUIRE_FTXUI();
  Term t(group({slider_int("N", "n", 0, 10, 3)}));
  t.key("Tab");
  t.key("Right");
  EXPECT_EQ(val(t.app, "n"), "4");
  t.key("End");
  EXPECT_EQ(val(t.app, "n"), "10");
  t.key("Right"); // clamped: no change, no write
  EXPECT_EQ(val(t.app, "n"), "10");
  t.key("Home");
  EXPECT_EQ(val(t.app, "n"), "0");
  t.key("PageUp");
  EXPECT_EQ(val(t.app, "n"), "1");
}

TEST(AriadneFtxui, SliderMouseDragCommitsOnRelease) {
  REQUIRE_FTXUI();
  Term t(group({slider_int("N", "n", 0, 10, 3)}));
  int x = 0, y = 0;
  ASSERT_TRUE(find_cell(t.screen, "N ", x, y)) << t.screen;
  const int gauge = x + 2; // the 20-cell gauge follows "N "
  t.mouse(TermEvent::Mouse::Press, gauge + 10, y);
  t.draw();
  EXPECT_EQ(val(t.app, "n"), "3") << "a held slider has not committed yet";
  EXPECT_TRUE(t.shows(" 5")) << "but it shows the in-progress value\n" << t.screen;
  t.mouse(TermEvent::Mouse::Move, gauge + 19, y);
  t.mouse(TermEvent::Mouse::Release, gauge + 19, y);
  EXPECT_EQ(val(t.app, "n"), "10");
}

TEST(AriadneFtxui, SliderFloatStepsAndFormats) {
  REQUIRE_FTXUI();
  Term t(group({slider_float("Speed", "speed", 0.0, 2.0, 1.0, "%.2f")}));
  EXPECT_TRUE(t.shows("1.00")) << t.screen;
  t.key("Tab");
  t.key("Right"); // a hundredth of the range
  EXPECT_NEAR(std::stod(val(t.app, "speed")), 1.02, 1e-9);
  t.draw();
  EXPECT_TRUE(t.shows("1.02")) << t.screen;
}

TEST(AriadneFtxui, ComboPopupPicksAnOption) {
  REQUIRE_FTXUI();
  Term t(group({combo("Mode", "mode", {"alpha", "beta", "gamma"})}));
  t.key("Tab");
  t.key("Return"); // open the list
  t.draw();
  EXPECT_TRUE(t.shows(" gamma ")) << t.screen;
  t.key("Down");
  t.key("Down");
  t.key("Return");
  EXPECT_EQ(val(t.app, "mode"), "gamma");
  t.draw();
  EXPECT_FALSE(t.shows(" beta ")) << "the list closed\n" << t.screen;
  t.key("Right"); // Left / Right cycle without opening it
  EXPECT_EQ(val(t.app, "mode"), "alpha");
  t.key("Left");
  EXPECT_EQ(val(t.app, "mode"), "gamma");
}

TEST(AriadneFtxui, ComboPopupByMouse) {
  REQUIRE_FTXUI();
  Term t(group({combo("Mode", "mode", {"alpha", "beta", "gamma"})}));
  ASSERT_TRUE(t.click("< alpha >"));
  ASSERT_TRUE(t.shows(" beta ")) << t.screen;
  int x = 0, y = 0;
  ASSERT_TRUE(find_cell(t.screen, " beta ", x, y));
  t.mouse(TermEvent::Mouse::Press, x + 1, y);
  EXPECT_EQ(val(t.app, "mode"), "beta");
}

TEST(AriadneFtxui, TextFieldCommitsOnEnterAndOnBlur) {
  REQUIRE_FTXUI();
  Term t(group({input_text("Name", "name", "sent"), checkbox("X", "x")}));
  int sent = 0;
  t.rt.on("sent", [&] { ++sent; });
  t.key("Tab");
  EXPECT_TRUE(t.type("hi"));
  EXPECT_EQ(val(t.app, "name"), "") << "typing edits locally";
  t.draw();
  EXPECT_TRUE(t.shows("Name: [hi")) << t.screen;
  t.key("Return");
  EXPECT_EQ(val(t.app, "name"), "hi");
  t.rt.drain();
  EXPECT_EQ(sent, 1);

  t.type("!");
  t.key("Tab"); // focus leaves: the pending edit commits
  EXPECT_EQ(val(t.app, "name"), "hi!");
  t.rt.drain();
  EXPECT_EQ(sent, 2);

  t.key("Tab", /*shift=*/true);
  t.type("zz");
  t.key("Escape"); // revert
  t.key("Tab");
  EXPECT_EQ(val(t.app, "name"), "hi!");
}

TEST(AriadneFtxui, TextFieldEditsAtTheCursor) {
  REQUIRE_FTXUI();
  Term t(group({input_text("Name", "name")}));
  t.key("Tab");
  t.type("ac");
  t.key("Left");
  t.type("b"); // "abc"
  t.key("Home");
  t.key("Delete"); // "bc"
  t.key("End");
  t.key("Backspace"); // "b"
  t.type(
      "\xc3\xa9"); // a two-byte UTF-8 character arrives as one event in the loop; here, two bytes
  t.key("Return");
  EXPECT_EQ(val(t.app, "name"), "b\xc3\xa9");
}

TEST(AriadneFtxui, MenuByKeyboard) {
  REQUIRE_FTXUI();
  Term t(group({menubar({menu("File", {menu_toggle("Wire", "wire"), menu_action("Quit", "quit")})}),
                checkbox("A", "a")}));
  int quit = 0;
  t.rt.on("quit", [&] { ++quit; });
  EXPECT_TRUE(t.key("F10"));
  t.draw();
  EXPECT_TRUE(t.shows("[ ] Wire")) << t.screen;
  EXPECT_TRUE(t.shows("Quit")) << t.screen;
  t.key("Return"); // the first item is focused
  EXPECT_EQ(val(t.app, "wire"), "1");
  t.draw();
  EXPECT_FALSE(t.shows("Quit")) << "activating an item closes the menu\n" << t.screen;

  t.key("F10");
  t.draw();
  t.key("Down");
  t.key("Return");
  t.rt.drain();
  EXPECT_EQ(quit, 1);

  t.key("F10");
  t.draw();
  t.key("Escape");
  t.draw();
  EXPECT_FALSE(t.shows("Quit")) << t.screen;
}

TEST(AriadneFtxui, MenuByMouse) {
  REQUIRE_FTXUI();
  Term t(group({menubar({menu("File", {menu_action("Quit", "quit")})})}));
  int quit = 0;
  t.rt.on("quit", [&] { ++quit; });
  ASSERT_TRUE(t.click("File"));
  ASSERT_TRUE(t.shows("Quit")) << t.screen;
  ASSERT_TRUE(t.click("Quit"));
  t.rt.drain();
  EXPECT_EQ(quit, 1);
  EXPECT_FALSE(t.shows("Quit")) << t.screen;
}

TEST(AriadneFtxui, WindowsCascadeAndUseTheAuthoredPos) {
  REQUIRE_FTXUI();
  Widget a = window("Alpha", {text("a")});
  Widget b = window("Beta", {text("b")});
  b.has_pos = true;
  b.pos_x = 400; // 50 columns
  b.pos_y = 160; // 10 rows
  Term t(group({a, b}));
  int x = 0, y = 0;
  ASSERT_TRUE(find_cell(t.screen, "\xe2\x95\xad Beta", x, y)) << t.screen; // "╭ Beta"
  EXPECT_EQ(x, 50);
  EXPECT_EQ(y, 10);
  ASSERT_TRUE(find_cell(t.screen, "\xe2\x95\xad Alpha", x, y)) << t.screen;
  EXPECT_EQ(x, 2);
  EXPECT_EQ(y, 1);
}

TEST(AriadneFtxui, WindowDragPersistsGeometry) {
  REQUIRE_FTXUI();
  Widget w = window("Win", {text("body")});
  w.id = "win";
  Term t(group({w}));
  int x = 0, y = 0;
  ASSERT_TRUE(find_cell(t.screen, " Win ", x, y)) << t.screen;
  t.mouse(TermEvent::Mouse::Press, x + 1, y);
  t.mouse(TermEvent::Mouse::Move, x + 11, y + 3);
  t.mouse(TermEvent::Mouse::Release, x + 11, y + 3);
  t.draw();
  int nx = 0, ny = 0;
  ASSERT_TRUE(find_cell(t.screen, " Win ", nx, ny)) << t.screen;
  EXPECT_EQ(nx, x + 10);
  EXPECT_EQ(ny, y + 3);
  // §11.4: the core persisted the moved position, in px (8 x 16 per cell).
  const std::string geo = val(t.app, "tree.win.geometry");
  EXPECT_EQ(geo.substr(0, geo.find(',', geo.find(',') + 1)),
            std::to_string((nx - 1) * 8) + "," + std::to_string(ny * 16))
      << geo;
}

TEST(AriadneFtxui, PersistedGeometrySeedsTheWindow) {
  REQUIRE_FTXUI();
  cvc::app app;
  FtxuiBackend be;
  Runtime rt(app, "");
  rt.set_backend(&be);
  cvc::state::instance(app)("tree.win.geometry").value("160,80,240,160"); // 20,5 cols/rows, 30x10
  Widget w = window("Win", {text("body")});
  w.id = "win";
  rt.set_root(group({w}));
  const std::string s = be.render_to_string(rt, 80, 24);
  int x = 0, y = 0;
  ASSERT_TRUE(find_cell(s, "\xe2\x95\xad Win", x, y)) << s;
  EXPECT_EQ(x, 20);
  EXPECT_EQ(y, 5);
  int bx = 0, by = 0;
  ASSERT_TRUE(find_cell(s, "\xe2\x95\xb0", bx, by)) << s; // "╰", the bottom-left corner
  EXPECT_EQ(by, y + 10 - 1) << "the persisted size (10 rows) was applied\n" << s;
}

TEST(AriadneFtxui, ATinyFrameDoesNotMoveWindowsForGood) {
  REQUIRE_FTXUI();
  Widget w = window("Beta", {text("b")});
  w.has_pos = true;
  w.pos_x = 400; // 50 columns
  w.pos_y = 160; // 10 rows
  Term t(group({w}));
  t.screen = t.be.render_to_string(t.rt, 12, 4); // e.g. a page before its first resize
  t.draw();                                      // back at 80 x 24
  int x = 0, y = 0;
  ASSERT_TRUE(find_cell(t.screen, "\xe2\x95\xad Beta", x, y)) << t.screen;
  EXPECT_EQ(x, 50);
  EXPECT_EQ(y, 10);
}

TEST(AriadneFtxui, CloseButtonClosesTheWindow) {
  REQUIRE_FTXUI();
  Widget w = window("Win", {text("body")});
  w.id = "win";
  w.closable = true;
  Term t(group({w}));
  ASSERT_TRUE(t.shows("[x]")) << t.screen;
  ASSERT_TRUE(t.click("[x]"));
  EXPECT_EQ(val(t.app, "tree.win.open"), "0");
  t.draw();
  EXPECT_FALSE(t.shows("body")) << t.screen;
}

TEST(AriadneFtxui, FocusRaisesItsWindow) {
  REQUIRE_FTXUI();
  // Two overlapping windows: Beta is on top. Focusing Alpha's checkbox brings Alpha up.
  Widget a = window("Alpha", {checkbox("Hidden", "h")});
  Widget b = window("Beta", {text("covering")});
  a.has_pos = b.has_pos = true;
  a.pos_x = b.pos_x = 16;
  a.pos_y = b.pos_y = 32;
  Term t(group({a, b}));
  EXPECT_FALSE(t.shows("Hidden")) << t.screen;
  t.key("Tab");
  t.draw();
  EXPECT_TRUE(t.shows("Hidden")) << t.screen;
}

TEST(AriadneFtxui, DisabledWidgetsAreSkipped) {
  REQUIRE_FTXUI();
  Widget a = checkbox("A", "a");
  a.enabled_when = "(= 1 2)";
  Term t(group({a, checkbox("B", "b")}));
  t.key("Tab"); // A is disabled: focus lands on B
  t.type(" ");
  EXPECT_EQ(val(t.app, "a"), "0");
  EXPECT_EQ(val(t.app, "b"), "1");
  ASSERT_TRUE(t.click("[ ] A"));
  EXPECT_EQ(val(t.app, "a"), "0") << "a click on a disabled widget does nothing";
}

TEST(AriadneFtxui, TooltipShowsForTheFocusedWidget) {
  REQUIRE_FTXUI();
  Widget c = checkbox("A", "a");
  c.tooltip = "toggles the thing";
  Term t(group({c}));
  EXPECT_FALSE(t.shows("toggles the thing"));
  t.key("Tab");
  t.draw();
  EXPECT_TRUE(t.shows("toggles the thing")) << t.screen;
}

TEST(AriadneFtxui, GridLaysCellsIntoColumns) {
  REQUIRE_FTXUI();
  Widget g = group({text("a1"), text("b1"), text("a2"), text("b2")});
  g.layout.kind = LayoutKind::Grid;
  g.layout.col_widths = {Track{Unit::Px, 80.0f}, Track{}};
  g.layout.borders = BorderShow::Inner;
  Term t(group({g}));
  int x1 = 0, y1 = 0, x2 = 0, y2 = 0;
  ASSERT_TRUE(find_cell(t.screen, "a1", x1, y1)) << t.screen;
  ASSERT_TRUE(find_cell(t.screen, "b1", x2, y2)) << t.screen;
  EXPECT_EQ(y1, y2);
  EXPECT_EQ(x2, x1 + 10 + 1) << "an 80px (10-cell) column, then the seam\n" << t.screen;
  ASSERT_TRUE(find_cell(t.screen, "a2", x2, y2));
  EXPECT_EQ(x2, x1);
  EXPECT_EQ(y2, y1 + 2) << "the next row, under a seam\n" << t.screen;
}

TEST(AriadneFtxui, RowSplitSeamDragResizesPanesAndPersists) {
  REQUIRE_FTXUI();
  Widget g = group({text("top"), text("bottom")});
  g.id = "split";
  g.layout.kind = LayoutKind::Grid;
  g.layout.row_heights = {Track{Unit::Px, 80.0f}, Track{Unit::Px, 80.0f}}; // 5 rows each
  g.layout.resizable = true;
  Term t(group({g}));
  int x = 0, top = 0, bx = 0, bottom = 0;
  ASSERT_TRUE(find_cell(t.screen, "top", x, top)) << t.screen;
  ASSERT_TRUE(find_cell(t.screen, "bottom", bx, bottom)) << t.screen;
  EXPECT_EQ(bottom, top + 6) << "5 rows, then the seam\n" << t.screen;
  EXPECT_EQ(val(t.app, "tree.split.tracks"), "80,80");
  const int seam = top + 5;
  t.mouse(TermEvent::Mouse::Press, x, seam);
  t.mouse(TermEvent::Mouse::Move, x, seam + 2);
  t.mouse(TermEvent::Mouse::Release, x, seam + 2);
  t.draw();
  ASSERT_TRUE(find_cell(t.screen, "bottom", bx, bottom)) << t.screen;
  EXPECT_EQ(bottom, top + 8) << t.screen;
  EXPECT_EQ(val(t.app, "tree.split.tracks"), "112,48") << "7 and 3 rows, persisted in px";
}

TEST(AriadneFtxui, PersistedTracksSeedTheSplit) {
  REQUIRE_FTXUI();
  cvc::app app;
  FtxuiBackend be;
  Runtime rt(app, "");
  rt.set_backend(&be);
  cvc::state::instance(app)("tree.split.tracks").value("32,128"); // 2 rows, 8 rows
  Widget g = group({text("top"), text("bottom")});
  g.id = "split";
  g.layout.kind = LayoutKind::Grid;
  g.layout.row_heights = {Track{Unit::Px, 80.0f}, Track{Unit::Px, 80.0f}};
  g.layout.resizable = true;
  rt.set_root(group({g}));
  const std::string s = be.render_to_string(rt, 40, 20);
  int x = 0, top = 0, bottom = 0;
  ASSERT_TRUE(find_cell(s, "top", x, top)) << s;
  ASSERT_TRUE(find_cell(s, "bottom", x, bottom)) << s;
  EXPECT_EQ(bottom, top + 3) << s;
}

TEST(AriadneFtxui, ResizableTableColumnSeamDrags) {
  REQUIRE_FTXUI();
  Widget g = group({text("a1"), text("b1"), text("a2"), text("b2")});
  g.layout.kind = LayoutKind::Grid;
  g.layout.col_widths = {Track{Unit::Px, 80.0f}, Track{Unit::Px, 80.0f}};
  g.layout.borders = BorderShow::Inner;
  g.layout.resizable = true;
  Term t(group({g}));
  int ax = 0, ay = 0, bx = 0, by = 0;
  ASSERT_TRUE(find_cell(t.screen, "a1", ax, ay)) << t.screen;
  ASSERT_TRUE(find_cell(t.screen, "b1", bx, by)) << t.screen;
  ASSERT_EQ(bx, ax + 11) << t.screen;
  t.mouse(TermEvent::Mouse::Press, bx - 1, by); // the seam left of column 2
  t.mouse(TermEvent::Mouse::Move, bx + 3, by);
  t.mouse(TermEvent::Mouse::Release, bx + 3, by);
  t.draw();
  int nbx = 0, nby = 0;
  ASSERT_TRUE(find_cell(t.screen, "b1", nbx, nby)) << t.screen;
  EXPECT_EQ(nbx, bx + 4) << t.screen;
  ASSERT_TRUE(find_cell(t.screen, "b2", nbx, nby));
  EXPECT_EQ(nbx, bx + 4) << "every row follows the column\n" << t.screen;
}

TEST(AriadneFtxui, ResizableTableWithoutBordersStillHasAGrabbableSeam) {
  REQUIRE_FTXUI();
  Widget g = group({text("a1"), text("b1")});
  g.layout.kind = LayoutKind::Grid;
  g.layout.col_widths = {Track{Unit::Px, 80.0f}, Track{}};
  g.layout.resizable = true;
  Term t(group({g}));
  int ax = 0, ay = 0, bx = 0, by = 0;
  ASSERT_TRUE(find_cell(t.screen, "a1", ax, ay)) << t.screen;
  ASSERT_TRUE(find_cell(t.screen, "b1", bx, by)) << t.screen;
  ASSERT_EQ(bx, ax + 11) << "a blank one-cell seam\n" << t.screen;
  t.mouse(TermEvent::Mouse::Press, bx - 1, by);
  t.mouse(TermEvent::Mouse::Move, bx - 4, by);
  t.mouse(TermEvent::Mouse::Release, bx - 4, by);
  t.draw();
  ASSERT_TRUE(find_cell(t.screen, "b1", bx, by)) << t.screen;
  EXPECT_EQ(bx, ax + 8) << t.screen;
}

// ---- hosted chrome (the mode the tvision backend drives) ----

namespace {

std::string row_text(const TermRows &rows, std::size_t y) {
  std::string out;
  if (y < rows.size())
    for (const TermCell &c : rows[y])
      out += c.ch.empty() ? std::string(" ") : c.ch;
  return out;
}

bool rows_show(const TermRows &rows, const std::string &s) {
  for (std::size_t y = 0; y < rows.size(); ++y)
    if (row_text(rows, y).find(s) != std::string::npos)
      return true;
  return false;
}

FtxuiBackend::Options hosted_opts() {
  FtxuiBackend::Options o;
  o.hosted = true;
  return o;
}

} // namespace

TEST(AriadneFtxuiHosted, KeepsWindowBodiesAndAMenuModel) {
  REQUIRE_FTXUI();
  Widget w = window("Win", {checkbox("A", "a"), checkbox("B", "b")});
  w.id = "w";
  w.closable = true;
  Term t(group({menubar({menu("File", {menu_toggle("Wire", "wire"), separator(),
                                       menu_action("Quit", "quit")})}),
                text("background"), w}),
         hosted_opts());
  EXPECT_FALSE(t.shows("File")) << "no menu bar is drawn\n" << t.screen;
  EXPECT_FALSE(t.shows("Win")) << "no window frame is drawn\n" << t.screen;
  EXPECT_TRUE(t.shows("background")) << "the root body still renders\n" << t.screen;
  EXPECT_TRUE(t.be.hosted_root());
  EXPECT_TRUE(t.be.hosted_menus().empty()) << "menus join the walk from the second frame";
  t.draw();

  const auto wins = t.be.hosted_windows();
  ASSERT_EQ(wins.size(), 1u);
  EXPECT_EQ(wins[0].id, "w");
  EXPECT_EQ(wins[0].title, "Win");
  EXPECT_TRUE(wins[0].closable);
  EXPECT_GE(wins[0].content_w, 5); // "[ ] A"
  EXPECT_EQ(wins[0].content_h, 2);

  const auto menus = t.be.hosted_menus();
  ASSERT_EQ(menus.size(), 1u);
  EXPECT_EQ(menus[0].label, "File");
  ASSERT_EQ(menus[0].items.size(), 3u);
  EXPECT_TRUE(menus[0].items[0].toggle);
  EXPECT_FALSE(menus[0].items[0].checked);
  EXPECT_TRUE(menus[0].items[1].separator);
  EXPECT_EQ(menus[0].items[2].label, "Quit");

  const TermRows rows = t.be.render_window("w", 20, 4);
  ASSERT_EQ(rows.size(), 4u);
  EXPECT_EQ(row_text(rows, 0).substr(0, 5), "[ ] A");
  EXPECT_EQ(row_text(rows, 1).substr(0, 5), "[ ] B");
}

TEST(AriadneFtxuiHosted, AMenuToggleDoesNotSeedAWindowClosed) {
  REQUIRE_FTXUI();
  // A "Status window" toggle bound to the window's own open key, as terminal_tour.ari has: the
  // window must still start open.
  Widget w = window("Status", {text("body")});
  w.id = "status";
  w.closable = true;
  Term t(group({menubar({menu("File", {menu_toggle("Status window", "tree.status.open")})}), w}),
         hosted_opts());
  t.draw();
  t.draw();
  EXPECT_EQ(val(t.app, "tree.status.open"), "1");
  ASSERT_EQ(t.be.hosted_windows().size(), 1u);
  ASSERT_EQ(t.be.hosted_menus().size(), 1u);
  EXPECT_TRUE(t.be.hosted_menus()[0].items[0].checked);
}

TEST(AriadneFtxuiHosted, InputGoesToTheWindowItIsFedFor) {
  REQUIRE_FTXUI();
  Widget a = window("A", {checkbox("One", "one")});
  Widget b = window("B", {checkbox("Two", "two")});
  Term t(group({a, b}), hosted_opts());
  t.be.render_window("A", 20, 3);
  t.be.render_window("B", 20, 3);
  EXPECT_TRUE(t.be.feed_window(t.rt, "B", TermEvent::key_event("Tab")));
  EXPECT_TRUE(t.be.feed_window(t.rt, "B", TermEvent::text_event(" ")));
  EXPECT_EQ(val(t.app, "two"), "1") << "Tab stayed inside window B";
  EXPECT_EQ(val(t.app, "one"), "0");
  // A click in window A's own cells.
  t.draw();
  t.be.render_window("A", 20, 3);
  t.be.feed_window(t.rt, "A", TermEvent::mouse_event(TermEvent::Mouse::Press, 1, 0));
  t.be.feed_window(t.rt, "A", TermEvent::mouse_event(TermEvent::Mouse::Release, 1, 0));
  EXPECT_EQ(val(t.app, "one"), "1");
}

TEST(AriadneFtxuiHosted, MenuItemsActivateThroughTheModel) {
  REQUIRE_FTXUI();
  Term t(
      group({menubar({menu("File", {menu_toggle("Wire", "wire"), menu_action("Quit", "quit")})})}),
      hosted_opts());
  int quit = 0;
  t.rt.on("quit", [&] { ++quit; });
  t.draw();
  auto menus = t.be.hosted_menus();
  ASSERT_EQ(menus.size(), 1u);
  ASSERT_EQ(menus[0].items.size(), 2u);
  t.be.activate_menu_item(t.rt, menus[0].items[0].id);
  EXPECT_EQ(val(t.app, "wire"), "1");
  t.draw();
  menus = t.be.hosted_menus();
  EXPECT_TRUE(menus[0].items[0].checked) << "the model follows the state";
  t.be.activate_menu_item(t.rt, menus[0].items[1].id);
  t.rt.drain();
  EXPECT_EQ(quit, 1);
}

TEST(AriadneFtxuiHosted, TheHostClosesAndPlacesWindows) {
  REQUIRE_FTXUI();
  Widget w = window("Win", {text("body")});
  w.id = "w";
  w.closable = true;
  Term t(group({w}), hosted_opts());
  t.be.set_window_geometry("w", 10, 3, 30, 8);
  t.draw();
  EXPECT_EQ(val(t.app, "tree.w.geometry"), "80,48,240,128") << "host bounds persist in px";
  t.be.close_window(t.rt, "w");
  EXPECT_EQ(val(t.app, "tree.w.open"), "0");
  t.draw();
  EXPECT_TRUE(t.be.hosted_windows().empty());
}

TEST(AriadneFtxuiHosted, ComboListOpensInsideItsWindow) {
  REQUIRE_FTXUI();
  Widget w = window("Win", {combo("Mode", "mode", {"alpha", "beta"})});
  Term t(group({w}), hosted_opts());
  t.be.render_window("Win", 24, 6);
  t.be.feed_window(t.rt, "Win", TermEvent::key_event("Tab"));
  t.be.feed_window(t.rt, "Win", TermEvent::key_event("Return"));
  t.draw();
  const TermRows rows = t.be.render_window("Win", 24, 6);
  EXPECT_TRUE(rows_show(rows, " beta ")) << row_text(rows, 1) << "\n" << row_text(rows, 2);
  t.be.feed_window(t.rt, "Win", TermEvent::key_event("Down"));
  t.be.feed_window(t.rt, "Win", TermEvent::key_event("Return"));
  EXPECT_EQ(val(t.app, "mode"), "beta");
}

// The tvision backend is an FtxuiBackend in hosted mode: it draws no frames of its own, it
// reports native windows / menus when it was built with tvision, and its window bodies come from
// the shared layout. (Its desktop needs a real terminal; tmux covers that by hand.)
TEST(AriadneTvision, IsTheHostedFtxuiBackend) {
  cvc::app app;
  TvisionBackend be;
  Runtime rt(app, "");
  rt.set_backend(&be);
  EXPECT_EQ(be.capabilities().windows, TvisionBackend::available());
  // Without FTXUI (macOS / Windows builds today) the backend is an inert stub whose capabilities
  // are all false, so the rest only holds where FTXUI is built in.
  if (!FtxuiBackend::available())
    GTEST_SKIP() << "libcvc built without FTXUI";
  EXPECT_TRUE(be.capabilities().owns_loop);
  Widget w = window("Win", {checkbox("A", "a")});
  rt.set_root(group({w}));
  const std::string s = be.render_to_string(rt, 40, 10);
  EXPECT_EQ(s.find("Win"), std::string::npos) << "hosted: tvision draws the frame\n" << s;
  ASSERT_EQ(be.hosted_windows().size(), 1u);
  const TermRows rows = be.render_window("Win", 20, 3);
  EXPECT_EQ(row_text(rows, 0).substr(0, 5), "[ ] A");
}

TEST(AriadneFtxui, ColorEditsAChannel) {
  REQUIRE_FTXUI();
  Term t(group({color_widget("Tint", "tint", "0.5,0.5,0.5")}));
  EXPECT_TRUE(t.shows("R 0.50 G 0.50 B 0.50")) << t.screen;
  t.key("Tab");
  t.type(" "); // pick G
  t.key("Right");
  const std::string v = val(t.app, "tint");
  EXPECT_EQ(v.substr(0, v.find(',')), "0.5") << v;
  EXPECT_NE(v, "0.5,0.5,0.5");
}

TEST(AriadneFtxui, ImageDrawsHalfBlocks) {
  REQUIRE_FTXUI();
  // 2x2: a red top row over a blue bottom row -> at 16px (2 columns) one row of two upper halves.
  const unsigned char px[] = {255, 0, 0, 255, 0, 0, 0, 0, 255, 0, 0, 255};
  cvc::image img(2, 2, cvc::image::pixel_format::RGB, cvc::image::data_type::u8, px);
  FtxuiBackend::Options o;
  o.image_color = FtxuiBackend::Options::ImageColor::Color;
  Term t(group({image_widget("pic", "map", 16.0)}), o);
  EXPECT_TRUE(t.shows("pic: map")) << "unpublished: the core shows the name\n" << t.screen;
  t.be.set_image("map", img);
  t.draw();
  EXPECT_TRUE(t.shows("\xe2\x96\x80\xe2\x96\x80")) << t.screen; // "▀▀"
  EXPECT_FALSE(t.shows("pic: map")) << t.screen;
}

TEST(AriadneFtxui, ImageRampOnAMonochromeTerminal) {
  REQUIRE_FTXUI();
  const unsigned char px[] = {255, 255, 255, 0, 0, 0};
  cvc::image img(2, 1, cvc::image::pixel_format::RGB, cvc::image::data_type::u8, px);
  FtxuiBackend::Options o;
  o.image_color = FtxuiBackend::Options::ImageColor::Mono;
  Term t(group({text("["), image_widget("pic", "map", 16.0), text("]")}), o);
  t.be.set_image("map", img);
  t.draw();
  EXPECT_TRUE(t.shows("@")) << "white maps to the dense end of the ramp\n" << t.screen;
}

TEST(AriadneFtxui, EmptyRuntimeRendersSafely) {
  REQUIRE_FTXUI();
  cvc::app app;
  FtxuiBackend be;
  Runtime rt(app, "");
  rt.set_backend(&be);
  EXPECT_NO_THROW(be.render_to_string(rt, 40, 0));
  EXPECT_NO_THROW(be.feed(rt, TermEvent::key_event("Tab")));
  EXPECT_NO_THROW(be.feed(rt, TermEvent::mouse_event(TermEvent::Mouse::Press, 3, 3)));
}

TEST(AriadneFtxui, TerminalTourDocumentRuns) {
  REQUIRE_FTXUI();
  if (!AppRuntime::have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  namespace fs = std::filesystem;
  const fs::path doc =
      fs::path(__FILE__).parent_path().parent_path() / "ariadne" / "examples" / "terminal_tour.ari";
  if (!fs::exists(doc))
    GTEST_SKIP() << "example not found at " << doc.string();
  cvc::app app;
  FtxuiBackend be;
  AppRuntime art(app, "ui.docs.tour");
  art.set_backend(be);
  bool quit = false, verb_quit = false;
  art.on("quit", [&] { quit = true; });
  art.register_verb("quit", [&] { verb_quit = true; }); // the (quit) the ari runner provides
  const std::vector<std::string> warns = art.load(doc.string());
  EXPECT_TRUE(warns.empty()) << warns.front();
  art.drain();
  std::string s = be.render_to_string(art.runtime(), 100, 30);
  EXPECT_NE(s.find("Controls"), std::string::npos) << s;
  EXPECT_NE(s.find("agents: 64"), std::string::npos) << s;
  // Type a name and press Enter: the field's on: program updates the log the Status window shows.
  be.feed(art.runtime(), TermEvent::key_event("Tab")); // Agents
  for (int i = 0; i < 4; ++i)
    be.feed(art.runtime(), TermEvent::key_event("Tab")); // Speed, Wireframe, Belief, Name
  for (char c : std::string("ada"))
    be.feed(art.runtime(), TermEvent::text_event(std::string(1, c)));
  be.feed(art.runtime(), TermEvent::key_event("Return"));
  for (int i = 0; i < 20; ++i)
    art.drain();
  s = be.render_to_string(art.runtime(), 100, 30);
  EXPECT_NE(s.find("log: hello, ada"), std::string::npos) << s;
  // Ctrl-C is SIGINT to the document's on_signal: the first press asks, the second quits.
  EXPECT_TRUE(art.post_signal("SIGINT", "terminal"));
  art.drain();
  s = be.render_to_string(art.runtime(), 100, 30);
  EXPECT_NE(s.find("Ctrl-C again to quit"), std::string::npos) << s;
  EXPECT_FALSE(verb_quit);
  art.post_signal("SIGINT", "terminal");
  art.drain();
  EXPECT_TRUE(verb_quit);
  // File > Quit raises the host's quit handler.
  be.feed(art.runtime(), TermEvent::key_event("F10"));
  be.render_to_string(art.runtime(), 100, 30);
  be.feed(art.runtime(), TermEvent::key_event("Down")); // from "Status window" to "Quit"
  be.feed(art.runtime(), TermEvent::key_event("Return"));
  art.drain();
  EXPECT_TRUE(quit);
  clear_action_intrinsics(); // register_verb's provider is process-wide: drop it with this test
}
