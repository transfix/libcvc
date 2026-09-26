// Tests for the Ariadne Runtime walk (cvc::ariadne::Runtime) via a recording mock
// Backend — headless, no ImGui/FTXUI. Covers the emit walk, the reconcile boundary,
// the deferred action drain, cvc::state read/seed/commit, bind-path resolution, and
// the §3.0.3b grid dispatch.

#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/core/state.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

using namespace cvc::ariadne;

namespace {

// Records the backend calls the walk makes, and returns programmable edits.
struct MockBackend : Backend {
  std::vector<std::string> log;
  bool button_click = false;
  BoolEdit checkbox_ret{};
  IntEdit slider_int_ret{};

  void rec(std::string s) { log.push_back(std::move(s)); }
  bool saw(const std::string &s) const {
    return std::find(log.begin(), log.end(), s) != log.end();
  }
  bool saw_prefix(const std::string &p) const {
    for (const std::string &s : log)
      if (s.rfind(p, 0) == 0)
        return true;
    return false;
  }
  int times(const std::string &s) const { return static_cast<int>(std::count(log.begin(), log.end(), s)); }

  Capabilities capabilities() const override {
    Capabilities c;
    c.windows = true;
    c.menubar = true;
    return c;
  }
  void begin_frame() override { rec("begin_frame"); }
  void end_frame() override { rec("end_frame"); }
  bool begin_main_menu_bar() override {
    rec("begin_menubar");
    return true;
  }
  void end_main_menu_bar() override { rec("end_menubar"); }
  bool begin_menu(const char *l) override {
    rec(std::string("begin_menu:") + l);
    return true;
  }
  void end_menu() override { rec("end_menu"); }
  bool begin_window(const char *t, const char *, const Size &, float) override {
    rec(std::string("begin_window:") + t);
    return true;
  }
  void end_window() override { rec("end_window"); }
  bool begin_grid(const Layout &L, const char *) override {
    rec(std::string("begin_grid:cols=") + std::to_string(L.col_widths.size()));
    return true;
  }
  void grid_next_cell() override { rec("next_cell"); }
  void end_grid() override { rec("end_grid"); }
  void push_id(const char *id) override { rec(std::string("push_id:") + id); }
  void pop_id() override { rec("pop_id"); }
  void text_line(const char *t) override { rec(std::string("text_line:") + t); }
  void text_value(const char *l, const std::string &v) override {
    rec(std::string("text_value:") + l + "=" + v);
  }
  void separator() override { rec("separator"); }
  bool button(const char *l) override {
    rec(std::string("button:") + l);
    return button_click;
  }
  bool menu_item_action(const char *l) override {
    rec(std::string("menu_action:") + l);
    return false;
  }
  BoolEdit menu_item_toggle(const char *l, bool) override {
    rec(std::string("toggle:") + l);
    return {};
  }
  BoolEdit checkbox(const char *l, bool) override {
    rec(std::string("checkbox:") + l);
    return checkbox_ret;
  }
  IntEdit slider_int(const char *l, const char *, int cur, int, int) override {
    rec(std::string("slider_int:") + l + "=" + std::to_string(cur));
    return slider_int_ret;
  }
  DoubleEdit slider_double(const char *l, const char *, double, double, double, const char *) override {
    rec(std::string("slider_double:") + l);
    return {};
  }
  IndexEdit combo(const char *l, int idx, const std::vector<std::string> &) override {
    rec(std::string("combo:") + l + "=" + std::to_string(idx));
    return {};
  }
};

} // namespace

TEST(AriadneRuntime, EmitsWindowAndWidgetsInOrder) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_root(group({window("W", {text("hi"), separator(), button("Go", "go")})}));
  rt.render();
  EXPECT_TRUE(mb.saw("begin_frame"));
  EXPECT_TRUE(mb.saw("begin_window:W"));
  EXPECT_TRUE(mb.saw("text_line:hi"));
  EXPECT_TRUE(mb.saw("separator"));
  EXPECT_TRUE(mb.saw("button:Go"));
  EXPECT_TRUE(mb.saw("end_window"));
  EXPECT_TRUE(mb.saw("end_frame"));
}

TEST(AriadneRuntime, ReconcileSwapAppliedAtRenderBoundary) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_root(group({text("first")}));
  rt.render();
  ASSERT_TRUE(mb.saw("text_line:first"));
  mb.log.clear();
  rt.set_root(group({text("second")})); // queued; not applied until the next render
  rt.render();
  EXPECT_TRUE(mb.saw("text_line:second"));
  EXPECT_FALSE(mb.saw("text_line:first"));
}

TEST(AriadneRuntime, ActionsDrainOffTheWalk) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  int fired = 0;
  rt.on("go", [&] { ++fired; });
  mb.button_click = true; // the button reports a click this frame
  rt.set_root(group({button("Go", "go")}));
  rt.render();
  EXPECT_EQ(fired, 0); // never runs inside render()
  rt.drain();
  EXPECT_EQ(fired, 1); // runs on drain
  rt.drain();
  EXPECT_EQ(fired, 1); // queue cleared — no double-fire
}

TEST(AriadneRuntime, CheckboxCommitWritesState) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  mb.checkbox_ret = BoolEdit{true, true, true}; // changed + committed, value true
  rt.set_root(group({checkbox("Wire", "demo.w", false)}));
  rt.render();
  EXPECT_EQ(cvc::state::instance(app)("demo.w").value(), "1");
}

TEST(AriadneRuntime, UncommittedEditDoesNotWriteState) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  cvc::state::instance(app)("demo.w").value(0);
  mb.checkbox_ret = BoolEdit{true, false, true}; // changed but NOT committed
  rt.set_root(group({checkbox("Wire", "demo.w", false)}));
  rt.render();
  EXPECT_EQ(cvc::state::instance(app)("demo.w").value(), "0"); // unchanged
}

TEST(AriadneRuntime, SliderReadsLiveStateNotDefault) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  cvc::state::instance(app)("demo.n").value(42);
  rt.set_root(group({slider_int("N", "demo.n", 0, 100, 5)}));
  rt.render();
  EXPECT_TRUE(mb.saw("slider_int:N=42")); // the state value, not the widget default 5
}

TEST(AriadneRuntime, PrefixResolvesRelativeBinds) {
  cvc::app app;
  Runtime rt(app, "ui.demo"); // relative binds splice onto this
  MockBackend mb;
  rt.set_backend(&mb);
  mb.checkbox_ret = BoolEdit{true, true, true};
  rt.set_root(group({checkbox("W", "wire", false)}));
  rt.render();
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.wire").value(), "1");
}

TEST(AriadneRuntime, AbsoluteBindEscapesThePrefix) {
  cvc::app app;
  Runtime rt(app, "ui.demo");
  MockBackend mb;
  rt.set_backend(&mb);
  mb.checkbox_ret = BoolEdit{true, true, true};
  rt.set_root(group({checkbox("W", "/global.flag", false)})); // leading '/' = app-root-absolute
  rt.render();
  EXPECT_EQ(cvc::state::instance(app)("global.flag").value(), "1");
}

TEST(AriadneRuntime, GridDispatchesBeginGridAndCells) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  Widget g;
  g.kind = Kind::Group;
  g.layout.kind = LayoutKind::Grid;
  g.layout.col_widths = {{Unit::Px, 10}, {Unit::Px, 10}};
  g.children = {text("a"), text("b")};
  rt.set_root(group({g}));
  rt.render();
  EXPECT_TRUE(mb.saw("begin_grid:cols=2"));
  EXPECT_EQ(mb.times("next_cell"), 2); // one per child
  EXPECT_TRUE(mb.saw("end_grid"));
  EXPECT_TRUE(mb.saw("text_line:a"));
  EXPECT_TRUE(mb.saw("text_line:b"));
}

TEST(AriadneRuntime, VerticalGroupDoesNotOpenAGrid) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_root(group({text("a"), text("b")})); // default vertical group
  rt.render();
  EXPECT_FALSE(mb.saw_prefix("begin_grid"));
  EXPECT_TRUE(mb.saw("text_line:a"));
}

TEST(AriadneRuntime, NoBackendRenderIsSafeNoop) {
  cvc::app app;
  Runtime rt(app, "");
  rt.set_root(group({text("x")}));
  rt.render(); // no backend set — must not crash
  SUCCEED();
}
