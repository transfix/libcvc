// Tests for the Ariadne Runtime walk (cvc::ariadne::Runtime) via a recording mock
// Backend — headless, no ImGui/FTXUI. Covers the emit walk, the reconcile boundary,
// the deferred action drain, cvc::state read/seed/commit, bind-path resolution, and
// the §3.0.3b grid dispatch.

#include <algorithm>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/loader.h> // §12 end-to-end load: mount through the real Runtime
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/state_exec/async_scheduler.h> // exec_scheduler().post_message end-to-end test
#include <cvc/core/state_exec/builtins.h>        // host-intrinsic seam test: register_fn
#include <cvc/core/state_exec/types.h>           // value_t
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <span>
#include <string>
#include <thread>
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
  bool saw(const std::string &s) const { return std::find(log.begin(), log.end(), s) != log.end(); }
  bool saw_prefix(const std::string &p) const {
    for (const std::string &s : log)
      if (s.rfind(p, 0) == 0)
        return true;
    return false;
  }
  int times(const std::string &s) const {
    return static_cast<int>(std::count(log.begin(), log.end(), s));
  }

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
  void begin_disabled() override { rec("begin_disabled"); }
  void end_disabled() override { rec("end_disabled"); }
  void set_tooltip(const char *t) override { rec(std::string("tooltip:") + (t ? t : "")); }
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
  DoubleEdit slider_double(const char *l, const char *, double, double, double,
                           const char *) override {
    rec(std::string("slider_double:") + l);
    return {};
  }
  IndexEdit combo_ret{}; // programmable return (a select) for the next combo()
  IndexEdit combo(const char *l, int idx, const std::vector<std::string> &) override {
    rec(std::string("combo:") + l + "=" + std::to_string(idx));
    return combo_ret;
  }
  ColorEdit color_ret{};           // programmable return for the next color()
  float last_color[3] = {0, 0, 0}; // the rgb the core handed color() (read side)
  ColorEdit color(const char *l, const float rgb[3]) override {
    last_color[0] = rgb[0];
    last_color[1] = rgb[1];
    last_color[2] = rgb[2];
    rec(std::string("color:") + l);
    return color_ret;
  }
  bool draw_image_ret =
      false;              // whether the mock "drew" the image (else the core falls back to text)
  std::string last_image; // the image name the core resolved (src or bound key)
  bool draw_image(const char *name, float) override {
    last_image = name ? name : "";
    rec(std::string("image:") + last_image);
    return draw_image_ret;
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

// §4/§7 action lane: an `on:` that is a state_exec PROGRAM (starts with '(', like a computed
// bind:/tooltip:) runs through state_exec at drain — no C++ handler. This is the north-star seam:
// a flag toggle / reset is pure .ari. The program is chrooted to the widget's prefix, so
// (state-set "paused") writes <prefix>.paused, the same key a widget `bind: paused` resolves to.
TEST(AriadneAction, ProgramOnTogglesStateThroughStateExec) {
  if (!have_state_exec())
    GTEST_SKIP() << "libcvc built without state_exec (CVC_STATE_EXEC=OFF)";
  cvc::app app;
  Runtime rt(app, "ui.demo");
  MockBackend mb;
  rt.set_backend(&mb);
  cvc::state::instance(app)("ui.demo.paused").value(std::string("false"));
  Widget b = button("Pause", "(state-set \"paused\" (if (= (state-get \"paused\") \"true\") "
                             "\"false\" \"true\"))");
  rt.set_root(group({b}));
  mb.button_click = true; // the button reports a click this frame
  rt.render();
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.paused").value(), "false"); // not run inside render
  rt.drain();
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.paused").value(), "true"); // program ran, toggled
  rt.drain();
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.paused").value(),
            "true");                                // queue cleared, no re-run
  EXPECT_TRUE(rt.take_reactive_warnings().empty()); // a clean program: no warning
}

// A program `on:` may sequence several state writes in one action (a reset button).
TEST(AriadneAction, ProgramOnRunsMultiStatementReset) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  Widget b = button("Reset", "(begin (state-set \"agents\" \"64\") (state-set \"speed\" \"1.0\"))");
  rt.set_root(group({b}));
  mb.button_click = true;
  rt.render();
  rt.drain();
  EXPECT_EQ(cvc::state::instance(app)("agents").value(), "64");
  EXPECT_EQ(cvc::state::instance(app)("speed").value(), "1.0");
}

// §4.7 end-to-end: an action that parks on (msg-recv …) does NOT block the drain; it suspends
// on the app-wide scheduler and RESUMES on a later drain when a WORKER THREAD delivers the result
// via the thread-safe exec_scheduler().post_message ingress. This is the marquee async story
// (a compute-pool worker waking a parked .ari action) exercised through the real Runtime.
TEST(AriadneAction, ProgramActionParksOnMsgRecvAndResumesWhenWorkerDelivers) {
  if (!have_state_exec())
    GTEST_SKIP() << "libcvc built without state_exec (CVC_STATE_EXEC=OFF)";
  cvc::app app;
  Runtime rt(app, ""); // app-root prefix so the channel/state paths are used as-is
  MockBackend mb;
  rt.set_backend(&mb);
  // The action parks on (msg-recv …), then AFTER it is woken runs a state-set. We assert on the
  // post-resume write (a sentinel), which cleanly proves park → deliver → wake → resume without
  // depending on threading the delivered value into a nested expression (deliver_to_receivers's
  // placeholder patch mis-targets a nested msg-recv — a separate, pre-existing issue).
  Widget b =
      button("Go", "(begin (msg-recv \"async.done\") (state-set \"async.result\" \"resumed\"))");
  rt.set_root(group({b}));
  mb.button_click = true;
  rt.render();             // enqueue the action
  rt.drain();              // submit + pump -> action PARKS on msg-recv
  mb.button_click = false; // don't re-fire on later renders
  EXPECT_NE(cvc::state::instance(app)("async.result").value(), "resumed"); // still parked

  // A worker thread delivers off the UI thread via the thread-safe ingress.
  std::thread worker([&] {
    app.exec_scheduler().post_message("async.done", cvc::state_exec::value_t(std::string("go")));
  });
  worker.join();

  rt.render();
  rt.drain(); // pump drains the ingress -> wakes the parked action -> it resumes and completes
  EXPECT_EQ(cvc::state::instance(app)("async.result").value(), "resumed");
  EXPECT_TRUE(rt.take_reactive_warnings().empty());
}

// §raster viewer: an image widget resolves its image name (a static src, or a bound key that a
// combo can switch) and hands it to the backend's draw_image; a backend that can't draw an image
// (no GL / terminal) returns false and the core shows the name as text.
TEST(AriadneRuntime, ImageWidgetResolvesNameAndFallsBackToText) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  mb.draw_image_ret = true; // the backend draws it
  rt.set_root(group({image_widget("Map", "belief")}));
  rt.render();
  EXPECT_EQ(mb.last_image, "belief"); // the static src name
  // A bound key selects the image (a combo would write it), overriding src.
  cvc::state::instance(app)("raster.layer").value(std::string("risk"));
  Widget im = image_widget("Map", "belief");
  im.bind = "raster.layer";
  rt.set_root(group({im}));
  rt.render();
  EXPECT_EQ(mb.last_image, "risk"); // the bound name wins
  // A backend that cannot draw an image -> the core shows the name as text.
  mb.draw_image_ret = false;
  rt.set_root(group({image_widget("Map", "gone")}));
  rt.render();
  EXPECT_TRUE(mb.saw("text_value:Map=gone"));
}

// The host-intrinsic seam: a host binds a native fn `(host-bump)` into the program lanes, and a
// program on: calls it — so a .ari program can invoke a host capability (e.g. a nav verb) inline.
TEST(AriadneAction, HostIntrinsicCallableFromProgram) {
  if (!have_state_exec())
    GTEST_SKIP();
  namespace se = cvc::state_exec;
  int calls = 0;
  register_action_intrinsics(
      [&calls](std::shared_ptr<se::environment> env, se::intrinsics_context &) {
        se::builtins::register_fn(env, "host-bump", [&calls](std::span<const se::value_t>) {
          ++calls;
          return se::value_t{}; // nil
        });
      });
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  Widget b = button("Go", "(host-bump)"); // a program on: that calls the host intrinsic
  rt.set_root(group({b}));
  mb.button_click = true;
  rt.render();
  rt.drain();
  clear_action_intrinsics(); // process-global — clean up before the next test
  EXPECT_EQ(calls, 1);       // the host fn ran from the .ari program
  EXPECT_TRUE(rt.take_reactive_warnings().empty()); // ran cleanly (the symbol resolved)
}

// A broken program action fails SAFE: it never throws out of drain(), and it surfaces a one-time
// warning (mirroring the read-lane's fail-safe policy) rather than silently doing nothing.
TEST(AriadneAction, ProgramOnBrokenWarnsOnceNoThrow) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  Widget b = button("Bad", "(state-set \"x\""); // unbalanced -> parse error
  rt.set_root(group({b}));
  mb.button_click = true;
  rt.render();
  rt.drain(); // must not throw
  const std::vector<std::string> warns = rt.take_reactive_warnings();
  ASSERT_FALSE(warns.empty());
  EXPECT_NE(warns[0].find("on:"), std::string::npos);
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

  EXPECT_TRUE(mock.saw("text_line:Pos"));   // composed literal caption
  EXPECT_TRUE(mock.saw("slider_double:x")); // composed bound slider
  EXPECT_TRUE(mock.saw("slider_double:y")); // state-dependent extra slider (advanced)
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
  EXPECT_TRUE(mock.saw("custom_widget:colorpick=red"));      // current value handed in
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
  register_widget_type(
      "dup", [](const Widget &, const WidgetEmitContext &ctx) { ctx.emit(text("composed")); });
  mock.custom_ret = CustomEdit{true, true, true, "x"};
  Widget c;
  c.kind = Kind::Custom;
  c.custom_type = "dup";
  rt.set_root(group({c}));
  rt.render();
  EXPECT_TRUE(mock.saw("text_line:composed"));        // compositional ran
  EXPECT_FALSE(mock.saw_prefix("custom_widget:dup")); // escape not consulted
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
  EXPECT_FALSE(mb.saw("text_line:shown"));          // 3 > 5 false -> hidden, no re-parse hazard
  EXPECT_TRUE(rt.take_reactive_warnings().empty()); // a passing predicate never warns
}

// --- §12 module mount: a scoped subtree resolves at its own sub-prefix -------

TEST(AriadneMountScope, BindsResolveAtSubPrefix) {
  cvc::app app;
  Runtime rt(app, "ui.demo");
  MockBackend mb;
  rt.set_backend(&mb);
  mb.checkbox_ret.committed = true;
  mb.checkbox_ret.value = true;
  // A mount wrapper (scope = "includes.rf") holding a checkbox bound to a BARE "on".
  Widget cb = checkbox("Flag", "on", false);
  Widget mount;
  mount.kind = Kind::Group;
  mount.scope = "includes.rf";
  mount.children = {cb};
  rt.set_root(group({mount}));
  rt.render();
  // The bind resolved under the mount's sub-prefix, NOT the document prefix.
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.includes.rf.on").value(), "1");
  EXPECT_NE(cvc::state::instance(app)("ui.demo.on").value(), "1");
}

TEST(AriadneMountScope, ReactiveReadsResolveAtSubPrefix) {
  if (!have_state_exec())
    GTEST_SKIP() << "libcvc built without state_exec (CVC_STATE_EXEC=OFF)";
  cvc::app app;
  Runtime rt(app, "ui.demo");
  MockBackend mb;
  rt.set_backend(&mb);
  // Seed a flag at the mount's SUB-prefix; a child predicate reads it relative to the scope.
  cvc::state::instance(app)("ui.demo.includes.rf.show").value(std::string("1"));
  Widget t = text("hi");
  t.visible_when = "(= (state-get \"show\") \"1\")";
  Widget mount;
  mount.kind = Kind::Group;
  mount.scope = "includes.rf";
  mount.children = {t};
  rt.set_root(group({mount}));
  rt.render();
  EXPECT_TRUE(mb.saw("text_line:hi")); // show==1 at the sub-prefix -> visible
  // Prove it reads the SUB-prefix: flipping the sub-prefix key hides it (a doc-scope "show"
  // would be a different node and could not).
  cvc::state::instance(app)("ui.demo.includes.rf.show").value(std::string("0"));
  mb.log.clear();
  rt.render();
  EXPECT_FALSE(mb.saw("text_line:hi"));
}

TEST(AriadneMountScope, RepeatedMountGetsPerInstanceSubPrefix) {
  if (!have_state_exec())
    GTEST_SKIP() << "libcvc built without state_exec (CVC_STATE_EXEC=OFF)";
  cvc::app app;
  Runtime rt(app, "ui.demo");
  MockBackend mb;
  rt.set_backend(&mb);
  mb.checkbox_ret.committed = true;
  mb.checkbox_ret.value = true;
  // A repeated mount: each instance must land on its OWN sub-prefix (includes.rf.<i>), not share
  // one — otherwise every instance's fragment would bind to the same state.
  Widget cb = checkbox("Flag", "on", false);
  Widget mount;
  mount.kind = Kind::Group;
  mount.scope = "includes.rf";
  mount.repeat = "(int 2)";
  mount.children = {cb};
  rt.set_root(group({mount}));
  rt.render();
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.includes.rf.0.on").value(), "1");
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.includes.rf.1.on").value(), "1");
}

TEST(AriadneMountScope, LinkHoleReadsAndWritesParentScope) {
  if (!have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  namespace fs = std::filesystem;
  const fs::path dir = fs::temp_directory_path() / "ariadne_link_e2e";
  fs::create_directories(dir);
  const fs::path frag = dir / "panel.ari";
  std::ofstream(frag) << "root:\n  - slider_int: Level\n    bind: lvl\n    lo: 0\n    hi: 100\n";
  cvc::app app;
  Runtime rt(app, "ui.demo");
  MockBackend mb;
  rt.set_backend(&mb);
  // Parent-owned value; the module reaches it only through the granted rw hole.
  cvc::state::instance(app)("ui.demo.globals.level").value(7);
  LoadResult lr =
      load_string("windows:\n  - window: W\n    children:\n      - load: " + frag.string() +
                  "\n        as: rf\n        link:\n          lvl: globals.level\n");
  ASSERT_TRUE(lr.ok) << lr.error;
  rt.set_root(std::move(lr.root));
  rt.render();
  EXPECT_TRUE(mb.saw("slider_int:Level=7")); // READ through the hole to the parent value
  // Commit a new value: it must write THROUGH the writable hole to the parent target.
  mb.slider_int_ret.committed = true;
  mb.slider_int_ret.value = 9;
  rt.render();
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.globals.level").value(), "9");
}

TEST(AriadneMountScope, ReadOnlyLinkHoleDoesNotWriteParent) {
  if (!have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  namespace fs = std::filesystem;
  const fs::path dir = fs::temp_directory_path() / "ariadne_link_ro_e2e";
  fs::create_directories(dir);
  const fs::path frag = dir / "panel.ari";
  std::ofstream(frag) << "root:\n  - slider_int: Level\n    bind: lvl\n    lo: 0\n    hi: 100\n";
  cvc::app app;
  Runtime rt(app, "ui.demo");
  MockBackend mb;
  rt.set_backend(&mb);
  cvc::state::instance(app)("ui.demo.globals.level").value(3);
  LoadResult lr = load_string(
      "windows:\n  - window: W\n    children:\n      - load: " + frag.string() +
      "\n        as: rf\n        link:\n          lvl: { to: globals.level, mode: ro }\n");
  ASSERT_TRUE(lr.ok) << lr.error;
  rt.set_root(std::move(lr.root));
  rt.render();
  EXPECT_TRUE(mb.saw("slider_int:Level=3")); // read-through works for a read-only hole too
  // A commit on a READ-ONLY hole must NOT reach the parent — it stays on the hole's own node.
  mb.slider_int_ret.committed = true;
  mb.slider_int_ret.value = 8;
  rt.render();
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.globals.level").value(), "3"); // parent untouched
}

TEST(AriadneMountScope, ReconcileTearsDownStaleHoles) {
  if (!have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  namespace fs = std::filesystem;
  const fs::path dir = fs::temp_directory_path() / "ariadne_stale_hole";
  fs::create_directories(dir);
  const fs::path frag = dir / "panel.ari";
  std::ofstream(frag) << "root:\n  - slider_int: Level\n    bind: lvl\n    lo: 0\n    hi: 100\n";
  cvc::app app;
  Runtime rt(app, "ui.demo");
  MockBackend mb;
  rt.set_backend(&mb);
  cvc::state::instance(app)("ui.demo.secret").value(42);
  // Tree A grants the module a rw hole lvl -> secret.
  LoadResult a =
      load_string("windows:\n  - window: W\n    children:\n      - load: " + frag.string() +
                  "\n        as: rf\n        link:\n          lvl: secret\n");
  ASSERT_TRUE(a.ok) << a.error;
  rt.set_root(std::move(a.root));
  rt.render();
  EXPECT_TRUE(mb.saw("slider_int:Level=42")); // reads secret through the hole
  // Tree B re-mounts the SAME fragment as `rf` but grants NO hole. The stale hole must be torn
  // down, so the module can no longer reach `secret`.
  LoadResult b =
      load_string("windows:\n  - window: W\n    children:\n      - load: " + frag.string() +
                  "\n        as: rf\n");
  ASSERT_TRUE(b.ok) << b.error;
  rt.set_root(std::move(b.root));
  mb.log.clear();
  rt.render();
  EXPECT_TRUE(mb.saw("slider_int:Level=0"));   // reads its own local (seeded) value now
  EXPECT_FALSE(mb.saw("slider_int:Level=42")); // no longer reaches secret
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.secret").value(), "42"); // untouched
}

TEST(AriadneMountScope, RepeatedMountLinkTargetsAreIndexed) {
  if (!have_state_exec())
    GTEST_SKIP() << "libcvc built without state_exec (CVC_STATE_EXEC=OFF)";
  cvc::app app;
  Runtime rt(app, "ui.demo");
  MockBackend mb;
  rt.set_backend(&mb);
  cvc::state::instance(app)("ui.demo.fleet.0").value(10);
  cvc::state::instance(app)("ui.demo.fleet.1").value(20);
  // A repeated mount whose hole target is {i}-parameterized: each instance reaches its own slot.
  Widget sl = slider_int("Slot", "slot", 0, 100, 0);
  Widget mount;
  mount.kind = Kind::Group;
  mount.scope = "includes.rf";
  mount.repeat = "(int 2)";
  mount.links.push_back(LinkHole{"slot", "fleet.{i}", true});
  mount.children = {sl};
  rt.set_root(group({mount}));
  rt.render();
  EXPECT_TRUE(mb.saw("slider_int:Slot=10")); // instance 0 -> fleet.0
  EXPECT_TRUE(mb.saw("slider_int:Slot=20")); // instance 1 -> fleet.1
}

TEST(AriadneMountScope, MountInitSeedsAtSubPrefix) {
  if (!have_state_exec() || !have_yaml())
    GTEST_SKIP() << "needs state_exec + yaml";
  namespace fs = std::filesystem;
  const fs::path dir = fs::temp_directory_path() / "ariadne_mount_init";
  fs::create_directories(dir);
  const fs::path frag = dir / "panel.ari";
  std::ofstream(frag) << "init: (state-set \"lvl\" \"7\")\n"
                         "root:\n  - slider_int: Level\n    bind: lvl\n    lo: 0\n    hi: 100\n";
  cvc::app app;
  Runtime rt(app, "ui.demo");
  MockBackend mb;
  rt.set_backend(&mb);
  LoadResult lr =
      load_string("windows:\n  - window: W\n    children:\n      - load: " + frag.string() +
                  "\n        as: rf\n");
  ASSERT_TRUE(lr.ok) << lr.error;
  rt.set_root(std::move(lr.root));
  rt.render();
  // The fragment's init ran at the mount's sub-prefix (before the slider was emitted), seeding lvl.
  EXPECT_TRUE(mb.saw("slider_int:Level=7"));
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.includes.rf.lvl").value(), "7");
}

TEST(AriadneMountScope, MountInitRunsOnceNotPerFrame) {
  if (!have_state_exec() || !have_yaml())
    GTEST_SKIP() << "needs state_exec + yaml";
  namespace fs = std::filesystem;
  const fs::path dir = fs::temp_directory_path() / "ariadne_mount_init_once";
  fs::create_directories(dir);
  const fs::path frag = dir / "panel.ari";
  std::ofstream(frag) << "init: (state-set \"n\" \"1\")\nroot: [ { text: hi } ]\n";
  cvc::app app;
  Runtime rt(app, "ui.demo");
  MockBackend mb;
  rt.set_backend(&mb);
  LoadResult lr =
      load_string("windows:\n  - window: W\n    children:\n      - load: " + frag.string() +
                  "\n        as: rf\n");
  ASSERT_TRUE(lr.ok) << lr.error;
  rt.set_root(std::move(lr.root));
  rt.render();
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.includes.rf.n").value(), "1");
  // Overwrite the seeded value and render again: init must NOT re-run and clobber it back to "1".
  cvc::state::instance(app)("ui.demo.includes.rf.n").value(std::string("99"));
  rt.render();
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.includes.rf.n").value(), "99");
}

TEST(AriadneMountScope, EndToEndLoadCommitsAtSubPrefix) {
  if (!have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  namespace fs = std::filesystem;
  const fs::path dir = fs::temp_directory_path() / "ariadne_mount_e2e";
  fs::create_directories(dir);
  const fs::path frag = dir / "panel.ari";
  std::ofstream(frag) << "root:\n  - checkbox: Flag\n    bind: on\n";
  ASSERT_TRUE(fs::exists(frag));
  cvc::app app;
  Runtime rt(app, "ui.demo");
  MockBackend mb;
  rt.set_backend(&mb);
  mb.checkbox_ret.committed = true;
  mb.checkbox_ret.value = true;
  // Load a doc that mounts the fragment (absolute path -> file handler), then drive it: the
  // mounted checkbox's BARE bind must commit under the mount's sub-prefix, end to end.
  LoadResult lr =
      load_string("windows:\n  - window: W\n    children:\n      - load: " + frag.string() +
                  "\n        as: rf\n");
  ASSERT_TRUE(lr.ok) << lr.error;
  rt.set_root(std::move(lr.root));
  rt.render();
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.includes.rf.on").value(), "1");
  EXPECT_NE(cvc::state::instance(app)("ui.demo.on").value(), "1"); // not at the doc scope
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
  EXPECT_FALSE(mb.saw("begin_window:W"));  // the window itself is skipped...
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
  EXPECT_FALSE(mb.saw("text_line:shown"));                       // unbound symbol -> hidden
  EXPECT_EQ(cvc::state::instance(app)("x").value(), "sentinel"); // read-only: nothing written
  EXPECT_FALSE(rt.take_reactive_warnings().empty());             // and it was reported
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
  rt.render();                             // must return (the step/time cap), not hang the walk
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
  EXPECT_FALSE(mb.saw("text_line:hidden"));         // nil is falsy
  EXPECT_TRUE(rt.take_reactive_warnings().empty()); // nil is a clean falsy, NOT an error
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
  EXPECT_FALSE(mb.saw("text_line:b")); // hidden
  EXPECT_TRUE(mb.saw("text_line:c"));
  EXPECT_EQ(mb.times("next_cell"), 2); // only the 2 VISIBLE children take a cell (no shift)
}

TEST(AriadneReactive, ReadLaneStillExcludesMaterializersAndSideEffects) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  // Compounds are now allowed, but the SIZE-DOUBLING materializers (no output cap) and the
  // side-effect / invoke builtins are still not bound: a predicate using one is an unbound
  // symbol -> fail-safe hidden. Locks the allowlist against re-admitting an amplifier.
  Widget wc = text("uses_concat");
  wc.visible_when = "(is-string (str-concat \"a\" \"b\"))"; // str-concat OUT (string doubler)
  Widget wp = text("uses_append");
  wp.visible_when = "(is-list (append (list 1) (list 2)))"; // append OUT (list materializer)
  Widget wa = text("uses_apply");
  wa.visible_when = "(apply and (list #t))"; // apply OUT (invoke)
  Widget wpr = text("uses_print");
  wpr.visible_when = "(begin (print \"x\") #t)"; // print OUT (I/O)
  rt.set_root(group({wc, wp, wa, wpr}));
  rt.render();
  EXPECT_FALSE(mb.saw("text_line:uses_concat"));
  EXPECT_FALSE(mb.saw("text_line:uses_append"));
  EXPECT_FALSE(mb.saw("text_line:uses_apply"));
  EXPECT_FALSE(mb.saw("text_line:uses_print"));
  EXPECT_FALSE(rt.take_reactive_warnings().empty()); // each reported an unbound symbol
}

TEST(AriadneReactive, CompoundsAndAllowedFormsNowWork) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  // With memoized values_equal + bounded to_string, compound values and structural equality
  // are safe and usable; let/if/list/length/quote are all allowed.
  Widget eq = text("eq");
  eq.visible_when = "(= (quote (1 2 3)) (quote (1 2 3)))"; // structural equality -> shown
  Widget ne = text("ne");
  ne.visible_when = "(= (quote (1 2)) (quote (9 9)))"; // unequal -> hidden
  Widget lst = text("lst");
  lst.visible_when = "(let ((xs (list 1 2 3))) (> (length xs) 2))"; // 3 > 2 -> shown
  rt.set_root(group({eq, ne, lst}));
  rt.render();
  EXPECT_TRUE(mb.saw("text_line:eq"));
  EXPECT_FALSE(mb.saw("text_line:ne"));
  EXPECT_TRUE(mb.saw("text_line:lst"));
  EXPECT_TRUE(rt.take_reactive_warnings().empty());
}

TEST(AriadneReactive, DeniedSpecialFormsAreRejectedNotRun) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  // The special-form gate denies the code-gen / object-graph forms. In particular the
  // defclass-method nested-eval hang is rejected at the FORM, before the constructor runs —
  // so it is hidden + warned, never the uncapped nested loop. render() must return.
  Widget wdc = text("wdc");
  wdc.visible_when = "(begin (defclass B (init (self) (while 1 1))) (B))"; // defclass denied
  Widget wev = text("wev");
  wev.visible_when = "(eval (quote #t))"; // eval denied
  Widget wdm = text("wdm");
  wdm.visible_when = "(begin (defmacro m () #t) #t)"; // defmacro denied
  rt.set_root(group({wdc, wev, wdm}));
  rt.render(); // must return, not hang
  EXPECT_FALSE(mb.saw("text_line:wdc"));
  EXPECT_FALSE(mb.saw("text_line:wev"));
  EXPECT_FALSE(mb.saw("text_line:wdm"));
  EXPECT_FALSE(rt.take_reactive_warnings().empty());
}

TEST(AriadneReactive, StateDataDagIsCopiedBoundedNotHung) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  // The full-env init lane plants a physically-tiny SHARED DAG (~30 nodes, 2^30 logical) in
  // typed data. A read-lane predicate then reads it via state-data-get -> deep_copy. deep_copy
  // is memoized, so the copy is bounded (physical size); an un-memoized copy would hang/OOM.
  ASSERT_TRUE(run_init(app, "ui.demo",
                       "(begin (set d (list 0 0)) (set i 0) "
                       "(while (< i 30) (begin (set d (list d d)) (set i (+ i 1)))) "
                       "(state-data-set \"bomb\" d))",
                       nullptr));
  Runtime rt(app, "ui.demo");
  MockBackend mb;
  rt.set_backend(&mb);
  Widget w = text("shown");
  w.visible_when = "(is-null (state-data-get \"bomb\"))"; // reads the DAG each frame
  rt.set_root(group({w}));
  rt.render(); // must return promptly (memoized deep_copy), not hang or OOM
  EXPECT_FALSE(mb.saw("text_line:shown"));          // is-null of a non-nil value -> hidden
  EXPECT_TRUE(rt.take_reactive_warnings().empty()); // completed cleanly, no cap/error
}

TEST(AriadneReactive, EnabledWhenGreysOutAndReacts) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  ASSERT_TRUE(run_init(app, "", "(state-set \"n\" \"3\")", nullptr));
  Widget w = button("Go", "go");
  w.enabled_when = "(> (int (state-get \"n\")) 5)"; // 3 > 5 -> false -> disabled
  rt.set_root(group({w}));
  rt.render();
  EXPECT_TRUE(mb.saw("begin_disabled")); // wrapped in a disabled scope
  EXPECT_TRUE(mb.saw("button:Go"));      // still DRAWN (greyed, not skipped)
  EXPECT_TRUE(mb.saw("end_disabled"));

  cvc::state::instance(app)("n").value(10); // now above threshold -> enabled
  mb.log.clear();
  rt.render();
  EXPECT_FALSE(mb.saw("begin_disabled")); // enabled -> no wrap
  EXPECT_TRUE(mb.saw("button:Go"));
}

TEST(AriadneReactive, DisabledWhenDisablesWhenTrue) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  ASSERT_TRUE(run_init(app, "", "(state-set \"lock\" \"1\")", nullptr));
  Widget w = button("Go", "go");
  w.disabled_when = "(state-exists \"lock\")"; // present -> true -> disabled
  rt.set_root(group({w}));
  rt.render();
  EXPECT_TRUE(mb.saw("begin_disabled"));
  EXPECT_TRUE(mb.saw("end_disabled"));
}

TEST(AriadneReactive, EnabledWhenBrokenPredicateDisablesFailSafe) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  Widget w = button("Go", "go");
  w.enabled_when = "(> (int"; // parse error -> fail-safe DISABLED
  rt.set_root(group({w}));
  rt.render();
  EXPECT_TRUE(mb.saw("begin_disabled")); // broken predicate -> disabled, not enabled
  EXPECT_TRUE(mb.saw("button:Go"));
  EXPECT_FALSE(rt.take_reactive_warnings().empty());
}

TEST(AriadneReactive, NoEnableFieldsMeansNoDisabledScope) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_root(group({button("Go", "go")})); // no enabled_when/disabled_when
  rt.render();
  EXPECT_FALSE(mb.saw("begin_disabled")); // no scope opened when neither field is set
  EXPECT_TRUE(mb.saw("button:Go"));
}

TEST(AriadneReactive, ComputedTextValueFromExpression) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  ASSERT_TRUE(run_init(app, "", "(state-set \"n\" \"7\")", nullptr));
  cvc::state::instance(app)("demo.k").value(std::string("hi"));
  Widget computed = text_bound("N1", "(+ (int (state-get \"n\")) 1)"); // bind is an expr: n+1
  Widget raw = text_bound("Raw", "(state-get \"n\")");                 // expr returning a string
  Widget path = text_bound("V", "demo.k"); // a dotted path (not an expr)
  rt.set_root(group({computed, raw, path}));
  rt.render();
  EXPECT_TRUE(mb.saw("text_value:N1=8"));  // computed (7+1) + coerced to a string
  EXPECT_TRUE(mb.saw("text_value:Raw=7")); // a string result shows RAW (no quotes)
  EXPECT_TRUE(mb.saw("text_value:V=hi"));  // a dotted path is still a plain state read
  cvc::state::instance(app)("n").value(9);
  mb.log.clear();
  rt.render();
  EXPECT_TRUE(mb.saw("text_value:N1=10")); // reacts to state each frame
  EXPECT_TRUE(rt.take_reactive_warnings().empty());
}

TEST(AriadneReactive, ComputedTextBrokenExprIsEmptyAndWarns) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  Widget t = text_bound("X", "(+ (int"); // parse error -> fail-safe EMPTY value
  rt.set_root(group({t}));
  rt.render();
  EXPECT_TRUE(mb.saw("text_value:X=")); // empty value, still drawn (not hung/crashed)
  EXPECT_FALSE(rt.take_reactive_warnings().empty());
}

// §4/G6: a combo with a parallel `values:` maps each displayed label to a stored value (e.g. an int
// key). The key holds the VALUE; the combo shows the LABEL at the value's index; a select writes
// the mapped value. This makes an int/enum-keyed control (volren.supersample, …) declarable.
TEST(AriadneRuntime, ComboValueMappingReadsAndWrites) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  // The key holds the VALUE "4"; options [Off,2x,4x] map to values [0,2,4], so 4 -> index 2.
  cvc::state::instance(app)("ss").value(std::string("4"));
  Widget c = combo("Supersample", "ss", {"Off", "2x", "4x"}, "");
  c.values = {"0", "2", "4"};
  rt.set_root(group({c}));
  rt.render();
  EXPECT_TRUE(mb.saw("combo:Supersample=2")); // read: value "4" shows at index 2 (the "4x" label)
  // Now select index 1 ("2x") -> the key must store the MAPPED value "2", not the label "2x".
  mb.combo_ret = IndexEdit{/*changed*/ true, /*committed*/ true, /*index*/ 1};
  rt.render();
  EXPECT_EQ(cvc::state::instance(app)("ss").value(), "2"); // stored the value, not "2x"
}

// §G7: a color widget reads its "r,g,b" key into the backend's picker and writes the edited colour
// back as "r,g,b" on commit. A backend with no colour widget (drawn:false) falls back to text.
TEST(AriadneRuntime, ColorWidgetReadsAndWritesCsv) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  cvc::state::instance(app)("tint").value(std::string("0.2,0.4,0.6"));
  mb.color_ret = ColorEdit{/*drawn*/ true, /*changed*/ false, /*committed*/ false, {0, 0, 0}};
  rt.set_root(group({color_widget("Tint", "tint")}));
  rt.render();
  EXPECT_TRUE(mb.saw("color:Tint"));         // drawn
  EXPECT_NEAR(mb.last_color[0], 0.2f, 1e-4); // read: the key parsed into the picker
  EXPECT_NEAR(mb.last_color[2], 0.6f, 1e-4);
  // Now the picker commits a new colour -> the key stores "r,g,b".
  mb.color_ret = ColorEdit{true, true, true, {0.1f, 0.5f, 0.9f}};
  rt.render();
  EXPECT_EQ(cvc::state::instance(app)("tint").value(), "0.1,0.5,0.9");
}

TEST(AriadneRuntime, ColorWidgetFallsBackToTextWhenBackendCantDraw) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  cvc::state::instance(app)("tint").value(std::string("0.3,0.3,0.3"));
  mb.color_ret = ColorEdit{}; // drawn == false -> the core shows the value as text
  rt.set_root(group({color_widget("Tint", "tint")}));
  rt.render();
  EXPECT_TRUE(mb.saw("text_value:Tint=0.3,0.3,0.3")); // fallback rendered
}

TEST(AriadneReactive, ComputedComboOptionsFromExpression) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  cvc::state::instance(app)("belief").value(std::string("grouped"));
  Widget c = combo("Belief", "belief", {}, "");                 // no static options
  c.options_expr = "(list \"shared\" \"grouped\" \"private\")"; // computed each frame
  rt.set_root(group({c}));
  rt.render();
  EXPECT_TRUE(mb.saw("combo:Belief=1")); // "grouped" is index 1 in the computed list
}

TEST(AriadneReactive, ComputedComboOptionsBrokenExprSkips) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  Widget c = combo("Belief", "belief", {}, "");
  c.options_expr = "(list \"a\""; // parse error -> empty list -> combo not drawn
  rt.set_root(group({c}));
  rt.render();
  EXPECT_FALSE(mb.saw_prefix("combo:"));             // no options -> nothing to show
  EXPECT_FALSE(rt.take_reactive_warnings().empty()); // reported
}

TEST(AriadneReactive, StaticAndComputedTooltips) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  ASSERT_TRUE(run_init(app, "", "(state-set \"hint\" \"dynamic\")", nullptr));
  Widget b1 = button("Go", "go");
  b1.tooltip = "click me"; // literal
  Widget b2 = button("Stop", "stop");
  b2.tooltip = "(state-get \"hint\")"; // computed
  rt.set_root(group({b1, b2}));
  rt.render();
  EXPECT_TRUE(mb.saw("button:Go"));
  EXPECT_TRUE(mb.saw("tooltip:click me")); // literal attached after the item
  EXPECT_TRUE(mb.saw("tooltip:dynamic"));  // computed from state
}

TEST(AriadneReactive, NoTooltipMeansNoSetTooltip) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_root(group({button("Go", "go")})); // no tooltip
  rt.render();
  EXPECT_FALSE(mb.saw_prefix("tooltip:"));
}

TEST(AriadneReactive, RepeatEmitsNInstancesWithIndexSubstituted) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  ASSERT_TRUE(run_init(app, "", "(state-set \"n\" \"3\")", nullptr));
  Widget t = text("Item {i}"); // literal caption with the loop-index token
  t.repeat = "(int (state-get \"n\"))";
  rt.set_root(group({t}));
  rt.render();
  EXPECT_TRUE(mb.saw("text_line:Item 0"));
  EXPECT_TRUE(mb.saw("text_line:Item 1"));
  EXPECT_TRUE(mb.saw("text_line:Item 2"));
  EXPECT_FALSE(mb.saw("text_line:Item 3")); // exactly n instances
  // reacts: raise the count
  cvc::state::instance(app)("n").value(4);
  mb.log.clear();
  rt.render();
  EXPECT_TRUE(mb.saw("text_line:Item 3"));
}

TEST(AriadneReactive, RepeatSubstitutesIndexInBindsForDistinctState) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  ASSERT_TRUE(run_init(app, "", "(state-set \"n\" \"2\")", nullptr));
  Widget c = checkbox("on", "items.{i}.on", false); // each instance addresses its own key
  c.repeat = "(int (state-get \"n\"))";
  mb.checkbox_ret = BoolEdit{true, true, true}; // every instance commits true
  rt.set_root(group({c}));
  rt.render();
  EXPECT_EQ(cvc::state::instance(app)("items.0.on").value(), "1");
  EXPECT_EQ(cvc::state::instance(app)("items.1.on").value(), "1"); // distinct, index-substituted
}

TEST(AriadneReactive, RepeatBrokenCountEmitsNothing) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  Widget t = text("X");
  t.repeat = "(int"; // parse error -> count 0 (fail-safe)
  rt.set_root(group({t}));
  rt.render();
  EXPECT_FALSE(mb.saw("text_line:X"));
  EXPECT_FALSE(rt.take_reactive_warnings().empty());
}

TEST(AriadneReactive, LiteralTooltipStartingWithParenShownVerbatim) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  Widget b = button("Go", "go");
  b.tooltip = "(optional) leave blank to use the default"; // plain text that starts with '('
  rt.set_root(group({b}));
  rt.render();
  EXPECT_TRUE(mb.saw("tooltip:(optional) leave blank to use the default")); // verbatim
  EXPECT_TRUE(rt.take_reactive_warnings().empty()); // NOT misread as a broken expression
}

TEST(AriadneReactive, DisabledWidgetStillEmitsItsTooltip) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  ASSERT_TRUE(run_init(app, "", "(state-set \"lock\" \"1\")", nullptr));
  Widget b = button("Go", "go");
  b.disabled_when = "(state-exists \"lock\")";
  b.tooltip = "why disabled";
  rt.set_root(group({b}));
  rt.render();
  // The tooltip is emitted for the disabled widget (inside the disabled scope), so the backend
  // can surface it (ImGuiBackend uses AllowWhenDisabled). Order: begin_disabled … tooltip … end.
  const auto pos = [&](const std::string &s) {
    return std::find(mb.log.begin(), mb.log.end(), s) - mb.log.begin();
  };
  ASSERT_TRUE(mb.saw("begin_disabled"));
  ASSERT_TRUE(mb.saw("tooltip:why disabled"));
  ASSERT_TRUE(mb.saw("end_disabled"));
  EXPECT_LT(pos("begin_disabled"), pos("tooltip:why disabled"));
  EXPECT_LT(pos("tooltip:why disabled"), pos("end_disabled"));
}

TEST(AriadneReactive, RepeatCountOutOfRangeDoubleIsZero) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  Widget t = text("X");
  t.repeat = "(* 1e19 1e19)"; // ~1e38: out of int64 range -> guarded -> fail-safe 0
  rt.set_root(group({t}));
  rt.render();
  EXPECT_FALSE(mb.saw("text_line:X")); // no instances (not UB / a garbage count)
  EXPECT_FALSE(rt.take_reactive_warnings().empty());
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
