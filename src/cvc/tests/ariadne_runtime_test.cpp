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
  CustomEdit custom_ret{}; // programmable return for the escape path
  CustomEdit custom_widget(const char *type, const std::string &current, const Widget &) override {
    rec(std::string("custom_widget:") + type + "=" + current);
    return custom_ret;
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

// --- extensibility: custom widget types (register_widget_type) ---------------

TEST(AriadneRuntime, CustomWidgetComposesAndBinds) {
  cvc::app app;
  Runtime rt(app, ""); // empty prefix -> bind resolution is identity
  MockBackend mock;
  rt.set_backend(&mock);
  bool fired = false;
  rt.on("custom_fired", [&] { fired = true; });

  // A capture-free custom widget: composes built-in primitives, its structure depends
  // on live state, and it raises an event — all through the WidgetEmitContext.
  register_widget_type("vec2", [](const Widget &w, const WidgetEmitContext &ctx) {
    ctx.emit(text(w.label));                          // -> text_line:<label>
    ctx.emit(slider_float("x", w.bind + ".x", 0, 1)); // -> slider_double:x (core binds it)
    if (ctx.read("ui.mode") == "advanced")            // structure depends on state
      ctx.emit(slider_float("y", w.bind + ".y", 0, 1));
    ctx.fire("custom_fired");
  });
  EXPECT_TRUE(has_widget_type("vec2"));

  cvc::state::instance(app)("ui.mode").value(std::string("advanced"));
  Widget c;
  c.kind = Kind::Custom;
  c.custom_type = "vec2";
  c.label = "Pos";
  c.bind = "pos";
  rt.set_root(group({c}));
  rt.render();

  EXPECT_TRUE(mock.saw("text_line:Pos"));    // composed literal caption
  EXPECT_TRUE(mock.saw("slider_double:x"));  // composed bound slider
  EXPECT_TRUE(mock.saw("slider_double:y"));  // state-dependent extra slider (advanced)
  rt.drain();
  EXPECT_TRUE(fired); // ctx.fire enqueued; drain ran the handler off the walk
}

TEST(AriadneRuntime, UnregisteredCustomWidgetDrawsPlaceholder) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mock;
  rt.set_backend(&mock);
  Widget c;
  c.kind = Kind::Custom;
  c.custom_type = "nope";
  rt.set_root(group({c}));
  rt.render();
  // Unregistered: a visible placeholder, not a crash or a silent drop.
  EXPECT_TRUE(mock.saw("text_line:[nope?]"));
}

// --- the init: block runner (run_init, state_exec) ---------------------------

TEST(AriadneInit, RunsScopedToPrefix) {
  if (!have_state_exec())
    GTEST_SKIP() << "libcvc built without state_exec (CVC_STATE_EXEC=OFF)";
  cvc::app app;
  std::vector<std::string> errs;
  // A relative path under the chroot prefix -> writes <prefix>.agents, the same key a
  // widget `bind: agents` (prefix "ui.demo") would resolve to.
  const bool ok = run_init(app, "ui.demo", "(state-set \"agents\" \"128\")", &errs);
  ASSERT_TRUE(ok) << (errs.empty() ? std::string("(no error)") : errs[0]);
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.agents").value(), "128");
}

TEST(AriadneInit, SyntaxErrorReportedNotThrown) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  std::vector<std::string> errs;
  const bool ok = run_init(app, "", "(state-set \"x\" ", &errs); // unbalanced
  EXPECT_FALSE(ok);
  EXPECT_FALSE(errs.empty());
}

TEST(AriadneInit, EmptyScriptIsNoop) {
  cvc::app app;
  EXPECT_TRUE(run_init(app, "ui", "", nullptr)); // no script -> success, nothing written
}

// --- custom widget: the backend novel-primitive escape (Backend::custom_widget) ---

TEST(AriadneRuntime, CustomWidgetBackendEscapeBindsState) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mock;
  rt.set_backend(&mock);
  // No compositional fn for "colorpick" -> emit routes to the backend escape. The mock
  // "handles" it and commits a new value; the core writes it to the bound state key.
  cvc::state::instance(app)("c").value(std::string("red"));
  mock.custom_ret = CustomEdit{/*handled*/ true, /*changed*/ true, /*committed*/ true, "blue"};
  Widget c;
  c.kind = Kind::Custom;
  c.custom_type = "colorpick";
  c.bind = "c";
  rt.set_root(group({c}));
  rt.render();
  EXPECT_TRUE(mock.saw("custom_widget:colorpick=red")); // current value handed in
  EXPECT_EQ(cvc::state::instance(app)("c").value(), "blue"); // committed value written back
}

TEST(AriadneRuntime, CustomWidgetUnhandledByBackendDrawsPlaceholder) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mock;
  rt.set_backend(&mock);
  mock.custom_ret = CustomEdit{}; // handled == false
  Widget c;
  c.kind = Kind::Custom;
  c.custom_type = "unknown_prim";
  rt.set_root(group({c}));
  rt.render();
  EXPECT_TRUE(mock.saw("custom_widget:unknown_prim=")); // backend was asked
  EXPECT_TRUE(mock.saw("text_line:[unknown_prim?]"));   // not handled -> placeholder
}

TEST(AriadneRuntime, CustomWidgetUncommittedEscapeDoesNotWrite) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mock;
  rt.set_backend(&mock);
  cvc::state::instance(app)("c2").value(std::string("keep"));
  mock.custom_ret = CustomEdit{true, true, false, "dropped"}; // handled + changed, NOT committed
  Widget c;
  c.kind = Kind::Custom;
  c.custom_type = "colorpick";
  c.bind = "c2";
  rt.set_root(group({c}));
  rt.render();
  EXPECT_EQ(cvc::state::instance(app)("c2").value(), "keep"); // uncommitted -> no write
}

TEST(AriadneInit, RunawayScriptIsBoundedNotHang) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  std::vector<std::string> errs;
  // A non-terminating init must be bounded (step/time cap) and reported — never hang.
  const bool ok = run_init(app, "", "(while true (+ 1 1))", &errs);
  EXPECT_FALSE(ok);
  EXPECT_FALSE(errs.empty());
}

TEST(AriadneRuntime, CustomWidgetCompositionalWinsOverEscape) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mock;
  rt.set_backend(&mock);
  // One type registered BOTH ways: the compositional fn must win; the backend escape
  // must NOT be consulted.
  register_widget_type("dup", [](const Widget &, const WidgetEmitContext &ctx) {
    ctx.emit(text("composed"));
  });
  mock.custom_ret = CustomEdit{true, true, true, "x"};
  Widget c;
  c.kind = Kind::Custom;
  c.custom_type = "dup";
  rt.set_root(group({c}));
  rt.render();
  EXPECT_TRUE(mock.saw("text_line:composed"));        // compositional ran
  EXPECT_FALSE(mock.saw_prefix("custom_widget:dup"));  // escape not consulted
}

// --- §4 read-lane: reactive visible_when -------------------------------------

TEST(AriadneReactive, EmptyPredicateAlwaysShows) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_root(group({text("always")})); // no visible_when
  rt.render();
  EXPECT_TRUE(mb.saw("text_line:always"));
  EXPECT_TRUE(rt.take_reactive_warnings().empty()); // no predicate -> no engine, no warnings
}

TEST(AriadneReactive, PredicateShowsThenHidesAsStateChanges) {
  if (!have_state_exec())
    GTEST_SKIP() << "libcvc built without state_exec (CVC_STATE_EXEC=OFF)";
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  // Seed n; state-set stores a string, so the predicate coerces with (int ...) — the
  // documented read-lane pattern (a bare (state-get) is a string, not a number).
  ASSERT_TRUE(run_init(app, "", "(state-set \"n\" \"10\")", nullptr));
  Widget w = text("shown");
  w.visible_when = "(> (int (state-get \"n\")) 5)";
  rt.set_root(group({w}));
  rt.render();
  EXPECT_TRUE(mb.saw("text_line:shown")); // 10 > 5 -> visible

  cvc::state::instance(app)("n").value(3); // drop below threshold
  mb.log.clear();
  rt.render();
  EXPECT_FALSE(mb.saw("text_line:shown")); // 3 > 5 false -> hidden, no re-parse hazard
  EXPECT_TRUE(rt.take_reactive_warnings().empty()); // a passing predicate never warns
}

TEST(AriadneReactive, FalsePredicateHidesTheWholeSubtree) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  Widget win = window("W", {text("inner"), button("B", "b")});
  win.visible_when = "(state-exists \"never\")"; // key absent -> false
  rt.set_root(group({win}));
  rt.render();
  EXPECT_FALSE(mb.saw("begin_window:W")); // the window itself is skipped...
  EXPECT_FALSE(mb.saw("text_line:inner")); // ...and everything under it
  EXPECT_FALSE(mb.saw("button:B"));
}

TEST(AriadneReactive, BrokenPredicateHidesFailSafeAndWarnsOnce) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  Widget w = text("shown");
  w.visible_when = "(> (int"; // unbalanced -> parse error
  rt.set_root(group({w}));
  rt.render();
  rt.render();
  rt.render();
  EXPECT_FALSE(mb.saw("text_line:shown")); // fail-safe: a broken predicate HIDES
  std::vector<std::string> warns = rt.take_reactive_warnings();
  ASSERT_EQ(warns.size(), 1u); // one message across three frames (de-duplicated)
  EXPECT_NE(warns[0].find("parse error"), std::string::npos);
  EXPECT_TRUE(rt.take_reactive_warnings().empty()); // drained
}

TEST(AriadneReactive, WriteIntrinsicIsUnavailableInAPredicate) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  cvc::state::instance(app)("x").value(std::string("sentinel"));
  Widget w = text("shown");
  w.visible_when = "(state-set \"x\" 1)"; // a writer is NOT bound in the read-only env
  rt.set_root(group({w}));
  rt.render();
  EXPECT_FALSE(mb.saw("text_line:shown"));                     // unbound symbol -> hidden
  EXPECT_EQ(cvc::state::instance(app)("x").value(), "sentinel"); // read-only: nothing written
  EXPECT_FALSE(rt.take_reactive_warnings().empty());           // and it was reported
}

TEST(AriadneReactive, PredicateReadsArePrefixScoped) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "ui.demo");
  MockBackend mb;
  rt.set_backend(&mb);
  // init scoped to the same prefix writes ui.demo.mode; the predicate's (state-get "mode")
  // must read that SAME key (both chroot'd to ui.demo) — matching widget bind resolution.
  ASSERT_TRUE(run_init(app, "ui.demo", "(state-set \"mode\" \"on\")", nullptr));
  Widget w = text("shown");
  w.visible_when = "(= (state-get \"mode\") \"on\")";
  rt.set_root(group({w}));
  rt.render();
  EXPECT_TRUE(mb.saw("text_line:shown"));
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.mode").value(), "on"); // confirms the key
}

TEST(AriadneReactive, RunawayPredicateIsCappedNotHung) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  Widget w = text("shown");
  w.visible_when = "(while true 1)"; // never terminates; while yields, so the cap fires
  rt.set_root(group({w}));
  rt.render(); // must return (the step/time cap), not hang the walk
  EXPECT_FALSE(mb.saw("text_line:shown")); // capped -> fail-safe hidden
  std::vector<std::string> warns = rt.take_reactive_warnings();
  ASSERT_EQ(warns.size(), 1u);
  EXPECT_NE(warns[0].find("budget"), std::string::npos);
}

TEST(AriadneReactive, StringValueIsTruthyAndUnsetKeyIsCleanlyFalsy) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  ASSERT_TRUE(run_init(app, "", "(state-set \"flag\" \"0\")", nullptr));
  Widget shown = text("shown");
  shown.visible_when = "(state-get \"flag\")"; // a bare string read is non-nil -> TRUTHY (§4.3)
  Widget hidden = text("hidden");
  hidden.visible_when = "(state-get \"never_set\")"; // missing -> nil -> cleanly falsy
  rt.set_root(group({shown, hidden}));
  rt.render();
  EXPECT_TRUE(mb.saw("text_line:shown"));           // string "0" is truthy
  EXPECT_FALSE(mb.saw("text_line:hidden"));          // nil is falsy
  EXPECT_TRUE(rt.take_reactive_warnings().empty());  // nil is a clean falsy, NOT an error
}

TEST(AriadneReactive, DistinctBrokenPredicatesWarnIndependently) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  Widget a = text("a");
  a.visible_when = "(> (int"; // parse error
  Widget b = text("b");
  b.visible_when = "(foobar 1 2)"; // parses, but foobar is unbound -> distinct eval error
  Widget c = text("c");
  c.visible_when = "(> (int"; // IDENTICAL to a -> same message -> collapses
  rt.set_root(group({a, b, c}));
  rt.render();
  std::vector<std::string> warns = rt.take_reactive_warnings();
  EXPECT_EQ(warns.size(), 2u); // two distinct causes; the duplicate does not add a third
}

TEST(AriadneReactive, HiddenMenuIsSkippedEntirely) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  Widget hidden_menu = menu("File", {menu_action("Open", "open")});
  hidden_menu.visible_when = "(state-exists \"never\")"; // false
  rt.set_root(menubar({hidden_menu, menu("Edit", {menu_action("Copy", "copy")})}));
  rt.render();
  EXPECT_FALSE(mb.saw("begin_menu:File")); // hidden -> no begin (and so no end) pairing
  EXPECT_TRUE(mb.saw("begin_menu:Edit"));  // its sibling still renders
}

TEST(AriadneReactive, HiddenGridChildConsumesNoCell) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  Widget g;
  g.kind = Kind::Group;
  g.layout.kind = LayoutKind::Grid;
  g.layout.col_widths = {{Unit::Px, 10}, {Unit::Px, 10}};
  Widget b = text("b");
  b.visible_when = "(state-exists \"never\")"; // hidden middle child
  g.children = {text("a"), b, text("c")};
  rt.set_root(group({g}));
  rt.render();
  EXPECT_TRUE(mb.saw("text_line:a"));
  EXPECT_FALSE(mb.saw("text_line:b"));  // hidden
  EXPECT_TRUE(mb.saw("text_line:c"));
  EXPECT_EQ(mb.times("next_cell"), 2); // only the 2 VISIBLE children take a cell (no shift)
}

TEST(AriadneReactive, PredicateEvalsAreIsolatedPerWidget) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  // widget1 defines a function in its per-eval (throwaway) scope and calls it; widget2 must
  // NOT see it — proving each eval is isolated, which is what keeps the read-only-STATE
  // guarantee intact even though set/defun special forms are always available to a predicate.
  Widget one = text("one");
  one.visible_when = "(begin (defun f () #t) (f))"; // defines + calls -> truthy
  Widget two = text("two");
  two.visible_when = "(f)"; // f is unbound in this fresh eval
  rt.set_root(group({one, two}));
  rt.render();
  EXPECT_TRUE(mb.saw("text_line:one"));  // its own defun is visible within the same eval
  EXPECT_FALSE(mb.saw("text_line:two")); // isolated: f did not leak -> unbound -> hidden
  EXPECT_FALSE(rt.take_reactive_warnings().empty()); // widget2's failure is reported
}
