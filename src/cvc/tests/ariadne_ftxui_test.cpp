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
#include <cvc/tf/model.h> // §10 the tf_editor's model
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

// While the Keys window waits for a key, the terminal hands every key to the document -- the
// capture -- rather than to the UI, where Tab would move the focus; a click still goes to the UI.
TEST(AriadneFtxui, KeysGoToTheDocumentWhileTheKeysWindowWaits) {
  REQUIRE_FTXUI();
  Term t(group({button("One", "one"), button("Two", "two")}));
  std::vector<std::string> doc;
  const auto to_doc = [&](const TermEvent &e) { doc.push_back(e.key); };
  t.be.route(t.rt, TermEvent::key_event("Tab"), {}, to_doc);
  EXPECT_TRUE(doc.empty()) << "Tab moves the UI's focus";

  auto &st = cvc::state::instance(t.app);
  st("tree.ariadne_keys.open").value(1);
  st("tree.ariadne_keys.capture").value(std::string("next"));
  ASSERT_TRUE(t.rt.key_capture_pending());
  t.be.route(t.rt, TermEvent::key_event("Tab"), {}, to_doc);
  t.be.route(t.rt, TermEvent::key_event("Return"), {}, to_doc);
  EXPECT_EQ(doc, (std::vector<std::string>{"Tab", "Return"}));
  t.be.route(t.rt, TermEvent::mouse_event(TermEvent::Mouse::Press, 0, 0), {}, to_doc);
  EXPECT_EQ(doc.size(), 2u) << "not a click";
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

TEST(AriadneFtxui, SliderMouseDragWritesAsItMoves) {
  REQUIRE_FTXUI();
  Term t(group({slider_int("N", "n", 0, 10, 3)}));
  int x = 0, y = 0;
  ASSERT_TRUE(find_cell(t.screen, "N ", x, y)) << t.screen;
  const int gauge = x + 2; // the 20-cell gauge follows "N "
  t.mouse(TermEvent::Mouse::Press, gauge + 10, y);
  t.draw();
  EXPECT_EQ(val(t.app, "n"), "5") << "a held slider writes as it goes";
  t.mouse(TermEvent::Mouse::Move, gauge + 19, y);
  t.draw();
  EXPECT_EQ(val(t.app, "n"), "10");
  t.mouse(TermEvent::Mouse::Release, gauge + 19, y);
  EXPECT_EQ(val(t.app, "n"), "10");
}

TEST(AriadneFtxui, SliderCommitOnReleaseWaitsForTheRelease) {
  REQUIRE_FTXUI();
  Widget s = slider_int("N", "n", 0, 10, 3);
  s.commit_on_release = true;
  Term t(group({s}));
  int x = 0, y = 0;
  ASSERT_TRUE(find_cell(t.screen, "N ", x, y)) << t.screen;
  const int gauge = x + 2;
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

// The cypkg tmux finding: a combo's on: never ran. Right picks the next option; the on: program
// runs on drain, after the write.
TEST(AriadneFtxui, ComboPickRunsItsOn) {
  REQUIRE_FTXUI();
  Widget c = combo("Mode", "mode", {"alpha", "beta", "gamma"});
  c.on = "(state-set \"x\" \"1\")";
  Term t(group({c}));
  t.key("Tab");
  t.key("Right");
  EXPECT_EQ(val(t.app, "mode"), "beta");
  t.rt.drain();
  EXPECT_EQ(val(t.app, "x"), "1");
}

// A radio shows every option. Left / Right (and Space / Enter) pick, a click on an option picks
// it, and each pick runs the on:.
TEST(AriadneFtxui, RadioPicksByKeyAndByClick) {
  REQUIRE_FTXUI();
  Widget r = radio("Mode", "mode", {"alpha", "beta", "gamma"});
  r.on = "(state-set \"x\" (state-get \"mode\"))";
  Term t(group({r}));
  EXPECT_TRUE(t.shows("Mode:")) << t.screen;
  EXPECT_TRUE(t.shows("(*) alpha")) << t.screen;
  EXPECT_TRUE(t.shows("( ) beta")) << t.screen;
  int ax = 0, ay = 0, bx = 0, by = 0;
  ASSERT_TRUE(find_cell(t.screen, "alpha", ax, ay));
  ASSERT_TRUE(find_cell(t.screen, "beta", bx, by));
  EXPECT_EQ(by, ay + 1) << "stacked\n" << t.screen;

  t.key("Tab");
  t.key("Right");
  EXPECT_EQ(val(t.app, "mode"), "beta");
  t.rt.drain();
  EXPECT_EQ(val(t.app, "x"), "beta");
  t.key("Left");
  t.key("Left"); // wraps
  EXPECT_EQ(val(t.app, "mode"), "gamma");
  t.key("Space");
  EXPECT_EQ(val(t.app, "mode"), "alpha");

  t.draw();
  ASSERT_TRUE(t.click("( ) gamma"));
  EXPECT_EQ(val(t.app, "mode"), "gamma");
  t.rt.drain();
  EXPECT_EQ(val(t.app, "x"), "gamma");
  ASSERT_TRUE(t.click("Mode:")); // the caption focuses, picks nothing
  EXPECT_EQ(val(t.app, "mode"), "gamma");
}

TEST(AriadneFtxui, RadioRowPutsTheOptionsOnOneLine) {
  REQUIRE_FTXUI();
  Term t(group({radio("Mode", "mode", {"alpha", "beta"}, "beta", /*row*/ true)}));
  EXPECT_TRUE(t.shows("Mode: ( ) alpha  (*) beta")) << t.screen;
  ASSERT_TRUE(t.click("( ) alpha", 4));
  EXPECT_EQ(val(t.app, "mode"), "alpha");
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

// autofocus: a person can type into the field as soon as it is drawn, no Tab first; moving focus
// away keeps it away (focus is taken when the field appears, not every frame).
TEST(AriadneFtxui, AutofocusFieldTakesTheFirstKeys) {
  REQUIRE_FTXUI();
  Widget name = input_text("Name", "name");
  name.autofocus = true;
  Term t(group({checkbox("X", "x"), name}));
  EXPECT_TRUE(t.type("hi"));
  t.key("Return");
  EXPECT_EQ(val(t.app, "name"), "hi");
  t.key("Tab"); // to the checkbox (the walk's first focusable)
  t.draw();
  t.type("z");
  t.key("Return");
  EXPECT_EQ(val(t.app, "name"), "hi") << "the field did not take focus back";
}

// password: the field draws a mark per character, never the text; the text still commits.
TEST(AriadneFtxui, PasswordFieldShowsAMask) {
  REQUIRE_FTXUI();
  Widget pw = input_text("Password", "pw");
  pw.password = true;
  Term t(group({pw}));
  t.key("Tab");
  t.type("s3cret");
  t.draw();
  EXPECT_FALSE(t.shows("s3cret")) << t.screen;
  // Six U+2022 as UTF-8 bytes: MSVC encodes a "\u2022" literal in the code page (this target has no
  // /utf-8), so the screen's UTF-8 would never match it.
  EXPECT_TRUE(t.shows("\xe2\x80\xa2\xe2\x80\xa2\xe2\x80\xa2\xe2\x80\xa2\xe2\x80\xa2\xe2\x80\xa2"))
      << t.screen;
  t.key("Return");
  t.draw();
  EXPECT_EQ(val(t.app, "pw"), "s3cret");
  EXPECT_FALSE(t.shows("s3cret")) << "masked when not focused too:\n" << t.screen;
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

// Another document's window of the same id is placed as a new one: the last document's drag
// does not carry over.
TEST(AriadneFtxui, ANewDocumentPlacesItsWindowsAfresh) {
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
  ASSERT_EQ(nx, x + 10);

  t.rt.forget_layout();
  t.rt.set_root(group({w}));
  t.draw();
  ASSERT_TRUE(find_cell(t.screen, " Win ", nx, ny)) << t.screen;
  EXPECT_EQ(nx, x);
  EXPECT_EQ(ny, y);
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

// §3.0.3b: a window with no size: is as big as its content, but stops at the screen's edge; past
// that its body scrolls, with a scroll bar, and keeps the focused widget in view.
TEST(AriadneFtxui, AWindowTallerThanTheScreenStopsAtItsEdgeAndScrolls) {
  REQUIRE_FTXUI();
  std::vector<Widget> items;
  for (int i = 0; i < 30; ++i)
    items.push_back(checkbox("Item " + std::to_string(100 + i), "item" + std::to_string(i)));
  Term t(group({window("Long", items)}));
  EXPECT_TRUE(t.shows("\xe2\x95\xb0")) << "its bottom edge (\"╰\") is on screen\n" << t.screen;
  EXPECT_TRUE(t.shows("Item 100")) << t.screen;
  EXPECT_FALSE(t.shows("Item 129")) << t.screen;
  // An unfocused window has a light frame, so a heavy bar is the scroll bar.
  EXPECT_TRUE(t.shows("\xe2\x94\x83") || t.shows("\xe2\x95\xbb")) << "a scroll bar\n" << t.screen;
  for (int i = 0; i < 30; ++i)
    t.key("Tab");
  t.draw();
  EXPECT_TRUE(t.shows("Item 129")) << "the focused widget scrolled into view\n" << t.screen;
  EXPECT_FALSE(t.shows("Item 100")) << t.screen;
}

TEST(AriadneFtxui, AWindowNearTheBottomStopsAtTheEdgeToo) {
  REQUIRE_FTXUI();
  std::vector<Widget> items;
  for (int i = 0; i < 10; ++i)
    items.push_back(text("line " + std::to_string(i)));
  Widget w = window("Low", items);
  w.has_pos = true;
  w.pos_x = 80;  // 10 columns
  w.pos_y = 288; // 18 rows: 6 left on a 24-row screen
  Term t(group({w}));
  int x = 0, y = 0;
  ASSERT_TRUE(find_cell(t.screen, "\xe2\x95\xb0", x, y)) << t.screen; // "╰"
  EXPECT_EQ(y, 23) << "the window ends on the screen's last row\n" << t.screen;
  EXPECT_TRUE(t.shows("line 0")) << t.screen;
  EXPECT_FALSE(t.shows("line 9")) << t.screen;
}

TEST(AriadneFtxui, ContentWiderThanTheScreenScrollsSideways) {
  REQUIRE_FTXUI();
  const std::string wide(120, 'w');
  Term t(group({window("Wide", {text("start " + wide + " end")})}));
  int x = 0, y = 0;
  ASSERT_TRUE(find_cell(t.screen, "\xe2\x95\xae", x, y)) << t.screen; // "╮", the top-right corner
  EXPECT_LT(x, 80) << "the window's right edge is on screen\n" << t.screen;
  EXPECT_TRUE(t.shows("start")) << t.screen;
  EXPECT_FALSE(t.shows("end")) << t.screen;
  // The bar is a light line along the body's last row, starting at the window's left edge ("│─").
  EXPECT_TRUE(t.shows("\xe2\x94\x82\xe2\x94\x80")) << "a horizontal scroll bar\n" << t.screen;
}

TEST(AriadneFtxui, ASizedWindowScrollsOnlyWhenItsContentOverflows) {
  REQUIRE_FTXUI();
  Widget fits = window("Fits", {text("one"), text("two")});
  fits.size.w = Extent{Unit::Px, 160}; // 20 columns
  fits.size.h = Extent{Unit::Px, 96};  // 6 rows
  Term a(group({fits}));
  EXPECT_FALSE(a.shows("\xe2\x94\x83") || a.shows("\xe2\x95\xb9") || a.shows("\xe2\x95\xbb"))
      << "no scroll bar when the content fits\n"
      << a.screen;
  Widget over = window(
      "Over", {text("one"), text("two"), text("three"), text("four"), text("five"), text("six")});
  over.size = fits.size;
  Term b(group({over}));
  EXPECT_TRUE(b.shows("\xe2\x94\x83") || b.shows("\xe2\x95\xb9") || b.shows("\xe2\x95\xbb"))
      << "a scroll bar when it overflows\n"
      << b.screen;
  EXPECT_FALSE(b.shows("six")) << b.screen;
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

// §5.9: a window closed with its [x] comes back from the built-in Windows menu.
TEST(AriadneFtxui, AClosedWindowReopensFromTheWindowsMenu) {
  REQUIRE_FTXUI();
  Widget w = window("Panel", {text("body")});
  w.id = "panel";
  w.closable = true;
  Term t(group({menubar({menu("File", {menu_action("Quit", "quit")})}), w}));
  ASSERT_TRUE(t.shows("Windows")) << t.screen;
  ASSERT_TRUE(t.click("[x]"));
  EXPECT_EQ(val(t.app, "tree.panel.open"), "0");
  t.draw();
  EXPECT_FALSE(t.shows("body")) << t.screen;
  ASSERT_TRUE(t.click("Windows"));
  ASSERT_TRUE(t.shows("[ ] Panel")) << "unticked while closed\n" << t.screen;
  ASSERT_TRUE(t.click("[ ] Panel"));
  EXPECT_EQ(val(t.app, "tree.panel.open"), "1");
  t.draw();
  EXPECT_TRUE(t.shows("body")) << t.screen;
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

TEST(AriadneFtxui, ANaturalPaneIsNotPinnedByTheFirstWalk) {
  REQUIRE_FTXUI();
  // A row split whose second pane has no authored height lays out naturally. The first walk runs
  // before FTXUI has laid anything out, so measuring that pane then read a box never laid out (one
  // cell, 16 px) and persisted it, and the next load pinned the pane at one row.
  auto make = [] {
    Widget g = group({text("top"), text("one\ntwo\nthree")});
    g.id = "split";
    g.layout.kind = LayoutKind::Grid;
    g.layout.row_heights = {Track{Unit::Px, 80.0f}, Track{}};
    g.layout.resizable = true;
    return g;
  };
  Term t(group({make()}));
  EXPECT_EQ(val(t.app, "tree.split.tracks"), "80,0") << "the natural pane is not set";
  t.draw();
  EXPECT_EQ(val(t.app, "tree.split.tracks"), "80,0") << "nor once it has been laid out";

  // A reload from what was persisted keeps the pane natural: all three lines show.
  cvc::app app;
  FtxuiBackend be;
  Runtime rt(app, "");
  rt.set_backend(&be);
  cvc::state::instance(app)("tree.split.tracks").value(val(t.app, "tree.split.tracks"));
  rt.set_root(group({make()}));
  const std::string s = be.render_to_string(rt, 40, 20);
  EXPECT_NE(s.find("three"), std::string::npos) << s;

  // Dragging the seam fixes both panes, and both persist.
  int x = 0, top = 0;
  ASSERT_TRUE(find_cell(t.screen, "top", x, top)) << t.screen;
  t.mouse(TermEvent::Mouse::Press, x, top + 5);
  t.mouse(TermEvent::Mouse::Move, x, top + 4);
  t.mouse(TermEvent::Mouse::Release, x, top + 4);
  t.draw();
  EXPECT_EQ(val(t.app, "tree.split.tracks"), "64,64") << "4 rows and the 3-row pane plus 1";
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

TEST(AriadneFtxui, DraggedTableColumnsPersistAndSeedTheNextLoad) {
  REQUIRE_FTXUI();
  auto make = [] {
    Widget g = group({text("a1"), text("b1"), text("c1"), text("a2"), text("b2"), text("c2")});
    g.id = "tbl";
    g.layout.kind = LayoutKind::Grid;
    g.layout.col_widths = {Track{Unit::Px, 80.0f}, Track{Unit::Px, 80.0f},
                           Track{Unit::Percent, 50.0f}};
    g.layout.borders = BorderShow::Inner;
    g.layout.resizable = true;
    return g;
  };
  Term t(group({make()}));
  EXPECT_EQ(val(t.app, "tree.tbl.columns"), "80,80,0")
      << "authored px as written; the stretch column is not set";
  int ax = 0, ay = 0, bx = 0, by = 0;
  ASSERT_TRUE(find_cell(t.screen, "a1", ax, ay)) << t.screen;
  ASSERT_TRUE(find_cell(t.screen, "b1", bx, by)) << t.screen;
  ASSERT_EQ(bx, ax + 11) << t.screen;
  t.mouse(TermEvent::Mouse::Press, bx - 1, by); // the seam: 4 cells from column 2 to column 1
  t.mouse(TermEvent::Mouse::Move, bx + 3, by);
  t.mouse(TermEvent::Mouse::Release, bx + 3, by);
  t.draw();
  EXPECT_EQ(val(t.app, "tree.tbl.columns"), "112,48,0") << "14 and 6 cells, in px";

  // A new load seeds the widths from state: column 2 starts 14 cells (and the seam) in, column 3
  // 6 cells after that.
  cvc::app app;
  FtxuiBackend be;
  Runtime rt(app, "");
  rt.set_backend(&be);
  cvc::state::instance(app)("tree.tbl.columns").value("112,48,0");
  rt.set_root(group({make()}));
  const std::string s = be.render_to_string(rt, 80, 10);
  int cx = 0, cy = 0;
  ASSERT_TRUE(find_cell(s, "a1", ax, ay)) << s;
  ASSERT_TRUE(find_cell(s, "b1", bx, by)) << s;
  ASSERT_TRUE(find_cell(s, "c1", cx, cy)) << s;
  EXPECT_EQ(bx, ax + 15) << s;
  EXPECT_EQ(cx, bx + 7) << s;
  EXPECT_EQ(val(app, "tree.tbl.columns"), "112,48,0") << "and reports them back unchanged";
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
  ASSERT_EQ(menus.size(), 2u);
  EXPECT_EQ(menus[0].label, "File");
  ASSERT_EQ(menus[0].items.size(), 3u);
  EXPECT_TRUE(menus[0].items[0].toggle);
  EXPECT_FALSE(menus[0].items[0].checked);
  EXPECT_TRUE(menus[0].items[1].separator);
  EXPECT_EQ(menus[0].items[2].label, "Quit");
  // §5.9: the built-in Windows menu reaches the host's menu bar too, ticked while "Win" is open.
  EXPECT_EQ(menus[1].label, "Windows");
  ASSERT_EQ(menus[1].items.size(), 1u);
  EXPECT_EQ(menus[1].items[0].label, "Win");
  EXPECT_TRUE(menus[1].items[0].toggle);
  EXPECT_TRUE(menus[1].items[0].checked);

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
  // Without FTXUI (a libcvc built without the ftxui dep) the backend is an inert stub whose
  // capabilities are all false, so the rest only holds where FTXUI is built in.
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

TEST(AriadneFtxui, BackgroundImageFillsTheScreenUnderTheWidgets) {
  REQUIRE_FTXUI();
  const unsigned char px[] = {200, 20, 20, 20, 20, 200, 20, 200, 20, 200, 200, 20};
  cvc::image img(2, 2, cvc::image::pixel_format::RGB, cvc::image::data_type::u8, px);
  FtxuiBackend::Options o;
  o.image_color = FtxuiBackend::Options::ImageColor::Color;
  Term t(group({text("hello")}), o);
  t.be.set_image("scene", img);
  t.be.set_background("scene");
  t.width = 20;
  t.height = 4;
  t.draw();
  int x = 0, y = 0;
  ASSERT_TRUE(find_cell(t.screen, "hello", x, y)) << t.screen;
  EXPECT_EQ(y, 0);
  ASSERT_TRUE(find_cell(t.screen, "\xe2\x96\x80", x, y)) << t.screen; // "▀"
  int n = 0; // every cell but the five "hello" ones is the backdrop
  for (std::size_t i = 0; (i = t.screen.find("\xe2\x96\x80", i)) != std::string::npos; i += 3)
    ++n;
  EXPECT_EQ(n, 20 * 4 - 5) << t.screen;
  EXPECT_EQ(t.be.screen_width(), 20);
  EXPECT_EQ(t.be.screen_height(), 4);
  // A new frame published under the same name shows on the next draw (the backdrop resamples).
  const std::string before = t.be.render_to_string(t.rt, 20, 4, /*ansi=*/true);
  const unsigned char px2[] = {0, 0, 0, 0, 0, 0, 255, 255, 255, 255, 255, 255};
  t.be.set_image("scene",
                 cvc::image(2, 2, cvc::image::pixel_format::RGB, cvc::image::data_type::u8, px2));
  EXPECT_NE(t.be.render_to_string(t.rt, 20, 4, /*ansi=*/true), before);
}

// A menu bar over a picture behind the widgets is its own: no picture cells are left under it, so
// its inverted titles don't take the picture's colours (often the same on both sides of a cell).
TEST(AriadneFtxui, AMenuBarOverABackgroundImageIsItsOwn) {
  REQUIRE_FTXUI();
  const unsigned char px[] = {200, 20, 20, 20, 20, 200, 20, 200, 20, 200, 200, 20};
  cvc::image img(2, 2, cvc::image::pixel_format::RGB, cvc::image::data_type::u8, px);
  FtxuiBackend::Options o;
  o.image_color = FtxuiBackend::Options::ImageColor::Color;
  Term t(group({menubar({menu("File", {menu_action("Quit", "quit")})}), text("hello")}), o);
  t.be.set_image("scene", img);
  t.be.set_background("scene");
  t.width = 20;
  t.height = 4;
  t.draw();
  const std::string bar = t.screen.substr(0, t.screen.find('\n'));
  EXPECT_NE(bar.find("File"), std::string::npos) << t.screen;
  EXPECT_EQ(bar.find("\xe2\x96\x80"), std::string::npos) << "no picture under the bar\n"
                                                         << t.screen;
  EXPECT_NE(t.screen.find("\xe2\x96\x80"), std::string::npos) << "but below it\n" << t.screen;
}

// ---- pixel graphics (Kitty / Sixel) ----------------------------------------------------------

namespace {

using ImageColor = FtxuiBackend::Options::ImageColor;

const std::string kPlaceholder = "\xf4\x8e\xbb\xae"; // U+10EEEE
const std::string kEsc = "\x1b";
const std::string kSaveCursor = kEsc + "7", kRestoreCursor = kEsc + "8"; // DECSC / DECRC

int count(const std::string &s, const std::string &needle) {
  int n = 0;
  for (std::size_t i = 0; (i = s.find(needle, i)) != std::string::npos; i += needle.size())
    ++n;
  return n;
}

cvc::image solid(int w, int h, unsigned char r, unsigned char g, unsigned char b) {
  std::vector<unsigned char> px(static_cast<std::size_t>(w) * h * 3);
  for (std::size_t i = 0; i < px.size(); i += 3) {
    px[i] = r;
    px[i + 1] = g;
    px[i + 2] = b;
  }
  return cvc::image(w, h, cvc::image::pixel_format::RGB, cvc::image::data_type::u8, px.data());
}

// A window at (x, y) px of w x h px.
Widget placed_window(const std::string &title, std::vector<Widget> kids, float x, float y, float w,
                     float h) {
  Widget win = window(title, std::move(kids));
  win.has_pos = true;
  win.pos_x = x;
  win.pos_y = y;
  win.size.w = Extent{Unit::Px, w};
  win.size.h = Extent{Unit::Px, h};
  return win;
}

TermGraphics kitty_term() {
  TermGraphics g;
  g.kitty = true;
  g.cell_w = 10;
  g.cell_h = 20;
  g.answered = true;
  return g;
}

TermGraphics iterm_term() {
  TermGraphics g;
  g.iterm = true; // what $TERM_PROGRAM says; no cell size (Windows reports none)
  return g;
}

TermGraphics sixel_term(int cell_h = 12) {
  TermGraphics g;
  g.sixel = true;
  g.cell_w = 8;
  g.cell_h = cell_h;
  g.sixel_colors = 256;
  g.answered = true;
  return g;
}

} // namespace

TEST(AriadneFtxuiGraphics, AutoPicksKittyThenSixelThenHalfBlocks) {
  REQUIRE_FTXUI();
  FtxuiBackend be;
  EXPECT_NE(be.image_mode(), ImageColor::Kitty) << "nothing detected: half-blocks";
  EXPECT_NE(be.image_mode(), ImageColor::Sixel);
  be.set_terminal_graphics(sixel_term());
  EXPECT_EQ(be.image_mode(), ImageColor::Sixel);
  TermGraphics both = sixel_term();
  both.kitty = true;
  be.set_terminal_graphics(both);
  EXPECT_EQ(be.image_mode(), ImageColor::Kitty) << "Kitty is preferred";
  TermGraphics no_size = sixel_term();
  no_size.cell_w = no_size.cell_h = 0;
  be.set_terminal_graphics(no_size);
  EXPECT_NE(be.image_mode(), ImageColor::Sixel) << "a bitmap needs the real cell size";

  FtxuiBackend::Options forced;
  forced.image_color = ImageColor::Color;
  FtxuiBackend color(forced);
  color.set_terminal_graphics(both);
  EXPECT_EQ(color.image_mode(), ImageColor::Color) << "an explicit mode wins";

  TvisionBackend tv; // hosted: tvision owns the output, so always half-blocks
  tv.set_terminal_graphics(both);
  EXPECT_NE(tv.image_mode(), ImageColor::Kitty);
  EXPECT_NE(tv.image_mode(), ImageColor::Sixel);
}

TEST(AriadneFtxuiGraphics, AutoPicksItermAfterKittyAndSixel) {
  REQUIRE_FTXUI();
  FtxuiBackend be;
  be.set_terminal_graphics(iterm_term());
  EXPECT_EQ(be.image_mode(), ImageColor::Iterm) << "needs no cell size: the terminal scales it";
  TermGraphics also = sixel_term();
  also.iterm = true;
  be.set_terminal_graphics(also);
  EXPECT_EQ(be.image_mode(), ImageColor::Sixel);
  also.kitty = true;
  be.set_terminal_graphics(also);
  EXPECT_EQ(be.image_mode(), ImageColor::Kitty);
  TvisionBackend tv;
  tv.set_terminal_graphics(iterm_term());
  EXPECT_NE(tv.image_mode(), ImageColor::Iterm);
}

TEST(AriadneFtxuiGraphics, ItermIsSentAfterEveryDrawScaledToTheUncoveredCells) {
  REQUIRE_FTXUI();
  FtxuiBackend::Options o;
  o.image_color = ImageColor::Iterm;
  Term t(group({text("hello")}), o);
  t.be.set_terminal_graphics(iterm_term());
  t.be.set_image("scene", solid(4, 4, 255, 0, 0));
  t.be.set_background("scene");
  t.width = 20;
  t.height = 5;
  t.draw();
  EXPECT_EQ(count(t.screen, "\xe2\x96\x80"), 20 * 5 - 5) << "the half-block underlay";
  const std::string g = t.be.take_graphics();
  // As for Sixel: row 1 (1-based) from column 6, and rows 2..4 whole; but each image names its
  // cells, so no band rounding.
  const std::string inline_image = "\x1b]1337;File=inline=1;";
  EXPECT_EQ(count(g, inline_image), 2) << g.substr(0, 120);
  const std::size_t a = g.find(kSaveCursor + "\x1b[2;1H" + inline_image);
  const std::size_t b = g.find(kSaveCursor + "\x1b[1;6H" + inline_image);
  ASSERT_NE(a, std::string::npos);
  ASSERT_NE(b, std::string::npos);
  EXPECT_NE(g.find(";width=20;height=3;", a), std::string::npos);
  EXPECT_NE(g.find(";width=15;height=1;", b), std::string::npos);
  EXPECT_EQ(count(g, kRestoreCursor), 2);
  t.draw();
  EXPECT_EQ(t.be.take_graphics(), g) << "a repaint covers the image, so it is sent again";
}

// The same picture over the same cells, laid over a terminal of another size, is another share
// of it: the cached bytes for that rectangle are not reused.
TEST(AriadneFtxuiGraphics, ItermImageIsRemadeWhenTheTerminalResizes) {
  REQUIRE_FTXUI();
  const auto stripes = [] { // a row of each colour, top to bottom
    const unsigned char rgb[4][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 255}};
    std::vector<unsigned char> px(4 * 4 * 3);
    for (int y = 0; y < 4; ++y)
      for (int x = 0; x < 4; ++x)
        for (int c = 0; c < 3; ++c)
          px[(y * 4 + x) * 3 + c] = rgb[y][c];
    return cvc::image(4, 4, cvc::image::pixel_format::RGB, cvc::image::data_type::u8, px.data());
  };
  const auto first_row_image = [](const std::string &g) { // row 1, beside "hello"
    const std::size_t at = g.find("\x1b[1;6H\x1b]1337;");
    return at == std::string::npos ? std::string() : g.substr(at, g.find('\x07', at) - at);
  };
  FtxuiBackend::Options o;
  o.image_color = ImageColor::Iterm;
  const auto term = [&](int height) {
    auto t = std::make_unique<Term>(group({text("hello")}), o);
    t->be.set_terminal_graphics(iterm_term());
    t->be.set_image("scene", stripes());
    t->be.set_background("scene");
    t->width = 20;
    t->height = height;
    t->draw();
    return t;
  };
  auto t = term(5);
  const std::string tall = first_row_image(t->be.take_graphics());
  ASSERT_FALSE(tall.empty());
  t->height = 2; // the first row is the same cells, but now half the picture's height
  t->draw();
  const std::string resized = first_row_image(t->be.take_graphics());
  const std::string fresh = first_row_image(term(2)->be.take_graphics());
  ASSERT_FALSE(fresh.empty());
  EXPECT_NE(fresh, tall) << "the test needs the two shares to differ";
  EXPECT_EQ(resized, fresh);
}

TEST(AriadneFtxuiGraphics, ImageRasterIsTheCellsRealPixels) {
  REQUIRE_FTXUI();
  FtxuiBackend::Options o;
  o.max_image_pixels = 100000;
  FtxuiBackend be(o);
  int w = 0, h = 0;
  be.image_raster(30, 10, w, h);
  EXPECT_EQ(w, 30) << "half-blocks: a pixel per column";
  EXPECT_EQ(h, 20) << "and two per row";
  be.set_terminal_graphics(kitty_term());
  EXPECT_EQ(be.cell_pixel_width(), 10);
  EXPECT_EQ(be.cell_pixel_height(), 20);
  be.image_raster(30, 10, w, h);
  EXPECT_EQ(w, 300);
  EXPECT_EQ(h, 200);
  be.image_raster(100, 50, w, h); // 1000 x 1000 px: scaled to fit max_image_pixels, same shape
  EXPECT_LE(w * h, 100000);
  EXPECT_NEAR(double(w) / h, 1.0, 0.01);
  be.set_terminal_graphics(sixel_term());
  be.image_raster(100, 50, w, h);
  EXPECT_EQ(w, 800) << "Sixel is never scaled: the terminal draws it pixel for pixel";
  EXPECT_EQ(h, 600);
}

TEST(AriadneFtxuiGraphics, KittyImageIsPlaceholdersAndIsSentOnlyWhenItChanges) {
  REQUIRE_FTXUI();
  FtxuiBackend::Options o;
  o.image_color = ImageColor::Kitty;
  Term t(group({text("above"), image_widget("pic", "map", 40.0)}), o);
  t.be.set_terminal_graphics(kitty_term());
  t.be.set_image("map", solid(10, 8, 200, 30, 30));
  t.draw();
  // 40 px = 5 columns; a 10 x 8 picture in 10 x 20 px cells is 2 rows.
  EXPECT_EQ(count(t.screen, kPlaceholder), 10) << t.screen;
  int x = 0, y = 0;
  ASSERT_TRUE(find_cell(t.screen, kPlaceholder, x, y));
  EXPECT_EQ(x, 0);
  EXPECT_EQ(y, 1);

  const std::string first = t.be.take_graphics();
  EXPECT_EQ(count(first, "a=T"), 1) << "sent once";
  EXPECT_NE(first.find("s=50,v=40,c=5,r=2"), std::string::npos) << "the cells' real pixels";
  const std::size_t at = first.find(",i=");
  ASSERT_NE(at, std::string::npos);
  const std::string id = first.substr(at + 3, first.find(',', at + 3) - at - 3);

  t.draw();
  EXPECT_EQ(t.be.take_graphics(), "") << "unchanged: nothing to send";
  t.be.set_image("map", solid(10, 8, 200, 30, 30));
  t.draw();
  EXPECT_EQ(t.be.take_graphics(), "") << "the same pixels published again are not re-sent";
  t.be.set_image("map", solid(10, 8, 30, 30, 200));
  t.draw();
  const std::string changed = t.be.take_graphics();
  EXPECT_EQ(count(changed, "a=T"), 1) << "new pixels are sent";
  // ... under the image's second id, which nothing names yet: re-sent under the shown id, the
  // terminal would drop the shown image for the whole transfer (an animated scene never shows).
  const std::size_t at2 = changed.find(",i=");
  ASSERT_NE(at2, std::string::npos);
  const std::string id2 = changed.substr(at2 + 3, changed.find(',', at2 + 3) - at2 - 3);
  EXPECT_NE(id2, id) << "the other id";
  // The next layout switches the placeholders to it (their colour names the image).
  const auto names = [&](const std::string &image_id) {
    const unsigned long v = std::stoul(image_id);
    const std::string colour = "38;2;" + std::to_string((v >> 16) & 0xFF) + ";" +
                               std::to_string((v >> 8) & 0xFF) + ";" + std::to_string(v & 0xFF);
    return t.be.render_to_string(t.rt, t.width, t.height, /*ansi=*/true).find(colour) !=
           std::string::npos;
  };
  EXPECT_TRUE(names(id2));
  EXPECT_FALSE(names(id));
  // The picture no longer changes, so the first id's older picture is freed ...
  EXPECT_EQ(t.be.take_graphics(), "\x1b_Ga=d,d=I,i=" + id + ",q=2\x1b\\");
  // ... and the next new picture goes to it again, while the cells still name the second.
  t.be.set_image("map", solid(10, 8, 30, 200, 30));
  t.draw();
  const std::string again = t.be.take_graphics();
  EXPECT_EQ(count(again, "a=T"), 1);
  EXPECT_NE(again.find(",i=" + id + ","), std::string::npos) << "back to the first id";

  t.rt.set_root(group({text("gone")}));
  t.draw();
  EXPECT_EQ(count(t.screen, kPlaceholder), 0);
  const std::string gone = t.be.take_graphics();
  EXPECT_NE(gone.find("\x1b_Ga=d,d=I,i=" + id + ",q=2\x1b\\"), std::string::npos) << gone;
  EXPECT_NE(gone.find("\x1b_Ga=d,d=I,i=" + id2 + ",q=2\x1b\\"), std::string::npos)
      << "an image no longer drawn is deleted, both its ids";
  EXPECT_EQ(t.be.release_graphics(), "");
}

TEST(AriadneFtxuiGraphics, KittyBackgroundShowsWhereNothingCoversIt) {
  REQUIRE_FTXUI();
  FtxuiBackend::Options o;
  o.image_color = ImageColor::Kitty;
  Term t(group({text("hello"), placed_window("W", {text("in a window")}, 80, 32, 120, 48)}), o);
  t.be.set_terminal_graphics(kitty_term());
  t.be.set_image("scene", solid(4, 4, 20, 120, 20));
  t.be.set_background("scene");
  t.width = 40;
  t.height = 6;
  t.draw();
  int x = 0, y = 0;
  ASSERT_TRUE(find_cell(t.screen, "in a window", x, y)) << t.screen;
  ASSERT_TRUE(find_cell(t.screen, "hello", x, y)) << t.screen;
  // Every cell shows the scene but the five of "hello" and the 15 x 3 window.
  EXPECT_EQ(count(t.screen, kPlaceholder), 40 * 6 - 5 - 15 * 3) << t.screen;
  const std::string g = t.be.take_graphics();
  EXPECT_EQ(count(g, "a=T"), 1);
  EXPECT_NE(g.find("c=40,r=6"), std::string::npos) << "one placement over the whole screen";
  // The text over the scene does not keep the colour that names the image.
  const std::size_t at = g.find(",i=");
  const unsigned id = static_cast<unsigned>(std::stoul(g.substr(at + 3)));
  const std::string ansi = t.be.render_to_string(t.rt, 40, 6, /*ansi=*/true);
  const std::string id_fg = "38;2;" + std::to_string((id >> 16) & 255) + ";" +
                            std::to_string((id >> 8) & 255) + ";" + std::to_string(id & 255);
  const std::size_t hello = ansi.find("hello");
  ASSERT_NE(hello, std::string::npos);
  const std::size_t sgr = ansi.rfind("\x1b[", hello);
  EXPECT_EQ(ansi.substr(sgr, hello - sgr).find(id_fg), std::string::npos)
      << "the text's own colour, not the id's";
}

TEST(AriadneFtxuiGraphics, KittyTilesAnImageWiderThanAPlacement) {
  REQUIRE_FTXUI();
  FtxuiBackend::Options o;
  o.image_color = ImageColor::Kitty;
  o.max_image_pixels = 0; // unbounded, so the tiles' pixels are exact
  Term t(group({}), o);
  t.be.set_terminal_graphics(kitty_term());
  t.be.set_image("scene", solid(8, 8, 90, 90, 90));
  t.be.set_background("scene");
  t.width = 300;
  t.height = 3;
  t.draw();
  const std::string g = t.be.take_graphics();
  EXPECT_EQ(count(g, "a=T"), 2) << "297 columns, then 3";
  EXPECT_NE(g.find("s=2970,v=60,c=297,r=3"), std::string::npos);
  EXPECT_NE(g.find("s=30,v=60,c=3,r=3"), std::string::npos);
  EXPECT_EQ(count(t.screen, kPlaceholder), 300 * 3);
}

TEST(AriadneFtxuiGraphics, SixelIsSentAfterEveryDrawClippedToTheUncoveredCells) {
  REQUIRE_FTXUI();
  FtxuiBackend::Options o;
  o.image_color = ImageColor::Sixel;
  Term t(group({text("hello")}), o);
  t.be.set_terminal_graphics(sixel_term(16));
  t.be.set_image("scene", solid(4, 4, 255, 0, 0));
  t.be.set_background("scene");
  t.width = 20;
  t.height = 5;
  t.draw();
  // The underlay is half-blocks, for a terminal that drops the bitmap.
  EXPECT_EQ(count(t.screen, "\xe2\x96\x80"), 20 * 5 - 5) << t.screen;
  const std::string g = t.be.take_graphics();
  // "hello" covers row 0's first five cells and the last row stays half-blocks, so two bitmaps:
  // row 1 (1-based) from column 6, and rows 2..4 whole. Rows 2..4 are 48 px, whole bands; row 1's
  // 16 px end in a partial band, which may reach into row 2 (the image shows there), so it is 18.
  EXPECT_EQ(count(g, "\x1bP"), 2) << g.substr(0, 200);
  const std::size_t a = g.find(kSaveCursor + "\x1b[2;1H\x1bP0;1;0q\"1;1;160;48");
  const std::size_t b = g.find(kSaveCursor + "\x1b[1;6H\x1bP0;1;0q\"1;1;120;18");
  EXPECT_NE(a, std::string::npos);
  EXPECT_NE(b, std::string::npos);
  EXPECT_LT(b, a) << "top first: the one reaching into row 2 goes before the one it reaches";
  EXPECT_EQ(count(g, kRestoreCursor), 2) << "the cursor goes back each time";
  t.draw();
  EXPECT_EQ(t.be.take_graphics(), g) << "a repaint covers the bitmap, so it is sent again";
}

TEST(AriadneFtxuiGraphics, SixelSendsWholeBandsAndSkipsAWindowOverTheImage) {
  REQUIRE_FTXUI();
  FtxuiBackend::Options o;
  o.image_color = ImageColor::Sixel;
  // An image of 6 x 3 cells at 8 x 16 px, with a window over its right half.
  Term t(group({image_widget("pic", "map", 48.0), placed_window("W", {text("x")}, 24, 0, 48, 48)}),
         o);
  t.be.set_terminal_graphics(sixel_term(16));
  t.be.set_image("map", solid(12, 12, 0, 0, 255));
  t.width = 30;
  t.height = 10;
  t.draw();
  const std::string g = t.be.take_graphics();
  // 48 px = 6 columns; 12 x 12 px at 8 x 16 px cells is round(6 * 8 / 16) = 3 rows. The window's
  // 6 x 3 cells cover columns 3..8, so the bitmap is the left 3 x 3 cells: 24 x 48 px, and 48 is
  // whole bands.
  EXPECT_EQ(count(g, "\x1bP"), 1) << g.substr(0, 200);
  EXPECT_NE(g.find(kSaveCursor + "\x1b[1;1H\x1bP0;1;0q\"1;1;24;48"), std::string::npos)
      << g.substr(0, 80);
}

TEST(AriadneFtxui, AClickOnTheBackgroundIsLeftForTheHost) {
  REQUIRE_FTXUI();
  Term t(group({checkbox("A", "a")}));
  int x = 0, y = 0;
  ASSERT_TRUE(find_cell(t.screen, "[ ] A", x, y));
  EXPECT_FALSE(t.mouse(TermEvent::Mouse::Press, 40, 10)) << "empty background: not the UI's";
  EXPECT_FALSE(t.mouse(TermEvent::Mouse::Release, 40, 10));
  EXPECT_TRUE(t.mouse(TermEvent::Mouse::Press, x, y)) << "a widget's";
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

TEST(AriadneFtxuiGraphics, TerminalImagesDocumentShowsItsPicture) {
  REQUIRE_FTXUI();
  if (!AppRuntime::have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  namespace fs = std::filesystem;
  // A missing document fails: if the examples move again, this test must follow them.
  const fs::path doc = fs::path(CVC_EXAMPLES_DIR) / "ariadne" / "terminal" / "terminal_images.ari";
  ASSERT_TRUE(fs::exists(doc)) << "example not found at " << doc.string();
  FtxuiBackend::Options o;
  o.image_color = ImageColor::Kitty;
  cvc::app app;
  FtxuiBackend be(o);
  be.set_terminal_graphics(kitty_term());
  be.set_image("photo", solid(40, 26, 40, 90, 200)); // what `ari --image photo=FILE` publishes
  AppRuntime art(app, "ui.docs.images");
  art.set_backend(be);
  const std::vector<std::string> warns = art.load(doc.string());
  EXPECT_TRUE(warns.empty()) << warns.front();
  art.drain();
  const std::string s = be.render_to_string(art.runtime(), 100, 30);
  EXPECT_NE(s.find("drag me over the picture"), std::string::npos) << s;
  EXPECT_GT(count(s, kPlaceholder), 0) << s;
  EXPECT_EQ(count(be.take_graphics(), "a=T"), 1);
}

TEST(AriadneFtxui, TerminalTourDocumentRuns) {
  REQUIRE_FTXUI();
  if (!AppRuntime::have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  namespace fs = std::filesystem;
  const fs::path doc = fs::path(CVC_EXAMPLES_DIR) / "ariadne" / "terminal" / "terminal_tour.ari";
  ASSERT_TRUE(fs::exists(doc)) << "example not found at " << doc.string();
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

// ---- §10 tf_editor: the editor the core composes on a terminal --------------------------------

namespace {

// A small model with known nodes: colour at 0 / 0.5 / 1, opacity at 0 / 0.25 / 1, isocontour bone.
std::string tf_sample_model() {
  cvc::tf::model m;
  m.color = {{0.0, 0.f, 0.f, 0.f}, {0.5, 1.f, 0.f, 0.f}, {1.0, 1.f, 1.f, 1.f}};
  m.alpha = {{0.0, 0.f}, {0.25, 0.5f}, {1.0, 1.f}};
  cvc::tf::isocontour bone;
  bone.id = "bone";
  bone.at = 0.6;
  m.isocontours = {bone};
  m.normalize();
  return cvc::tf::to_string(m);
}

cvc::tf::model tf_model(cvc::app &app, const std::string &path) {
  cvc::tf::model m;
  EXPECT_TRUE(cvc::tf::from_string(val(app, path), m)) << val(app, path);
  return m;
}

} // namespace

TEST(AriadneFtxui, TfEditorComposesFromTerminalWidgets) {
  REQUIRE_FTXUI();
  Term t(group({tf_editor("Transfer", "tf")}));
  EXPECT_EQ(val(t.app, "tf"), cvc::tf::to_string(cvc::tf::model::default_model()))
      << "an empty key is seeded on the first walk";
  cvc::state::instance(t.app)("tf").value(tf_sample_model());
  t.height = 60; // a row or two per node: taller than the default screen
  t.draw();
  EXPECT_TRUE(t.shows("Transfer")) << t.screen;
  EXPECT_TRUE(t.shows("domain: 0 .. 1")) << t.screen;
  EXPECT_TRUE(t.shows("colour 1: at 0")) << t.screen;
  EXPECT_TRUE(t.shows("colour 2 ")) << t.screen;
  EXPECT_TRUE(t.shows("alpha 2 ")) << t.screen;
  EXPECT_TRUE(t.shows("iso bone ")) << t.screen;
  EXPECT_TRUE(t.shows("rgb: ")) << t.screen; // the terminal's own colour picker
  EXPECT_TRUE(t.shows("[ x ]")) << t.screen;
  EXPECT_TRUE(t.shows("[ + isocontour ]")) << t.screen;
  EXPECT_TRUE(t.shows("[ reset ]")) << t.screen;
  EXPECT_TRUE(t.shows("[x] histogram")) << t.screen; // the default layers
  EXPECT_TRUE(t.shows("[ ] contour tree")) << t.screen;
  EXPECT_EQ(val(t.app, "tf"), tf_sample_model()) << "drawing alone writes nothing";
}

TEST(AriadneFtxui, TfEditorMouseAndKeyEditsCommit) {
  REQUIRE_FTXUI();
  Term t(group({tf_editor("Transfer", "tf")}));
  cvc::state::instance(t.app)("tf").value(tf_sample_model());
  t.height = 60;
  t.draw();
  // Press and release at the far end of the isocontour's gauge: bone moves to the top of the
  // domain (no data range is published, so the domain is [0, 1]).
  int x = 0, y = 0;
  ASSERT_TRUE(find_cell(t.screen, "iso bone ", x, y)) << t.screen;
  const int gauge = x + 9; // the 20-cell gauge follows "iso bone "
  t.mouse(TermEvent::Mouse::Press, gauge + 19, y);
  t.draw();
  EXPECT_NEAR(tf_model(t.app, "tf").isocontours.at(0).at, 0.6, 1e-6)
      << "a held slider has not committed yet";
  t.mouse(TermEvent::Mouse::Release, gauge + 19, y);
  t.draw();
  cvc::tf::model m = tf_model(t.app, "tf");
  ASSERT_EQ(m.isocontours.size(), 1u);
  EXPECT_EQ(m.isocontours[0].id, "bone");
  const double dragged = m.isocontours[0].at;
  EXPECT_GT(dragged, 0.9) << t.screen;
  // The press focused the slider: Left steps it back by a hundredth of the domain, at once.
  t.key("Left");
  m = tf_model(t.app, "tf");
  ASSERT_EQ(m.isocontours.size(), 1u);
  EXPECT_NEAR(m.isocontours[0].at, dragged - 0.01, 1e-6);
  // A button edits the model too: one more isocontour.
  t.draw();
  ASSERT_TRUE(t.click("[ + isocontour ]")) << t.screen;
  EXPECT_EQ(tf_model(t.app, "tf").isocontours.size(), 2u);
}
