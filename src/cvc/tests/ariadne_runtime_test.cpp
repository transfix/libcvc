// Tests for the Ariadne Runtime walk (cvc::ariadne::Runtime) via a recording mock
// Backend — headless, no ImGui/FTXUI. Covers the emit walk, the reconcile boundary,
// the deferred action drain, cvc::state read/seed/commit, bind-path resolution, and
// the §3.0.3b grid dispatch.

#include <algorithm>
#include <atomic> // item 2: the nav-step worker's arrived-count accumulator
#include <boost/any.hpp>
#include <chrono>
#include <clocale> // §10 tf_editor: the inline seed under a decimal-comma LC_NUMERIC
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cvc/ariadne/app_runtime.h> // AppRuntime: host menus, the scene between documents
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/loader.h>            // §12 end-to-end load: mount through the real Runtime
#include <cvc/ariadne/net_intrinsics.h>    // §13.8 async (http-get-async) intrinsic test
#include <cvc/ariadne/stream_intrinsics.h> // Phase-2 PR6 (stream-open/stream-info/stream-close)
#include <cvc/ariadne/uri.h>               // §13.8 (fetch uri): register a custom scheme handler
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/core/async_task.h> // launch_pool_task generalization test
#include <cvc/core/thread_pool.h> // item 2: full type for app.computePool().parallel_for
#include <cvc/net/http_client.h>  // §13.8: swap the transport for a fake in the http-get test
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_host.h>      // register_async_fn on app.exec_async_host()
#include <cvc/state/state_exec/async_scheduler.h> // exec_scheduler().post_message end-to-end test
#include <cvc/state/state_exec/builtins.h>        // host-intrinsic seam test: register_fn
#include <cvc/state/state_exec/heap.h>            // collect_cycles / tracked_objects
#include <cvc/state/state_exec/intrinsics.h>      // intrinsics_context.root_path (pool-task test)
#include <cvc/state/state_exec/types.h>           // value_t
#include <cvc/tf/model.h>                         // §10 tf_editor: the model the widget edits
#include <cvc/tf/overlays.h>                      // §10 tf_editor: the layers published beside it
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
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

  bool caps_bitmap = false; // report real pixels (the composed tf_editor then shows a preview)
  Capabilities capabilities() const override {
    Capabilities c;
    c.windows = true;
    c.menubar = true;
    c.bitmap = caps_bitmap;
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
  bool close_windows = false; // simulate the user clicking a window's close (X) button
  bool begin_window(const char *t, const char *, const Size &, float, bool *p_open) override {
    rec(std::string("begin_window:") + t + (p_open ? "(closable)" : ""));
    if (p_open && close_windows)
      *p_open = false; // the X was clicked -> the core should persist tree.<id>.open = 0
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
  // Colour-capable overrides (the default impls would drop the colour and call the plain leaves —
  // this records the colour so a test can prove the tint reaches the backend).
  float last_text_rgb[3] = {-1.f, -1.f, -1.f};
  void text_line_colored(const char *t, const float rgb[3]) override {
    last_text_rgb[0] = rgb[0];
    last_text_rgb[1] = rgb[1];
    last_text_rgb[2] = rgb[2];
    rec(std::string("text_line_colored:") + t);
  }
  void text_value_colored(const char *l, const std::string &v, const float rgb[3]) override {
    last_text_rgb[0] = rgb[0];
    last_text_rgb[1] = rgb[1];
    last_text_rgb[2] = rgb[2];
    rec(std::string("text_value_colored:") + l + "=" + v);
  }
  void separator() override { rec("separator"); }
  std::string click_button; // a button LABEL that reports a click (besides button_click)
  bool button(const char *l) override {
    rec(std::string("button:") + l);
    return button_click || (!click_button.empty() && click_button == l);
  }
  bool menu_item_action(const char *l) override {
    rec(std::string("menu_action:") + l);
    return false;
  }
  bool item_clicked_flag = false; // §4.6: report the last item as clicked (widget on_click test)
  bool item_clicked() override { return item_clicked_flag; }
  bool item_hovered_flag = false; // §4.6: report the last item as hovered (widget on_hover test)
  bool item_hovered() override { return item_hovered_flag; }
  bool item_dragged_flag = false; // §4.6: report the last item as dragged (widget on_drag test)
  bool item_dragged() override { return item_dragged_flag; }
  bool item_drag_started_flag = false; // §4.6: on_drag_start edge
  bool item_drag_started() override { return item_drag_started_flag; }
  bool item_drag_ended_flag = false; // §4.6: on_drag_end edge
  bool item_drag_ended() override { return item_drag_ended_flag; }
  // §4.6 event scope: deliver fixed test coords so a program handler can read (get-attr event "x").
  float ptr_x = 0, ptr_y = 0, ptr_dx = 0, ptr_dy = 0;
  int ptr_button = 0;
  bool item_pointer(float &x, float &y, float &dx, float &dy, int &button) override {
    x = ptr_x;
    y = ptr_y;
    dx = ptr_dx;
    dy = ptr_dy;
    button = ptr_button;
    return true;
  }
  // §11.4 geometry persistence: record the seed the core hands us, and report a programmable
  // "current" geometry / track list back so a test can drive the write-back edge.
  WindowGeom seeded_geom{};
  bool seeded_geom_called = false;
  WindowGeom window_geom_ret{};
  void seed_window_geometry(const WindowGeom &g) override {
    seeded_geom = g;
    seeded_geom_called = true;
    rec("seed_window_geometry");
  }
  WindowGeom window_geometry() const override { return window_geom_ret; }
  int layout_resets = 0;
  void reset_layout() override {
    ++layout_resets;
    rec("reset_layout");
  }
  std::vector<float> seeded_tracks;
  bool seeded_tracks_called = false;
  std::vector<float> grid_tracks_ret;
  void seed_grid_tracks(const std::vector<float> &sizes) override {
    seeded_tracks = sizes;
    seeded_tracks_called = true;
    rec("seed_grid_tracks");
  }
  std::vector<float> grid_tracks() const override { return grid_tracks_ret; }
  std::vector<float> seeded_columns;
  bool seeded_columns_called = false;
  std::vector<float> grid_columns_ret;
  void seed_grid_columns(const std::vector<float> &widths) override {
    seeded_columns = widths;
    seeded_columns_called = true;
    rec("seed_grid_columns");
  }
  std::vector<float> grid_columns() const override { return grid_columns_ret; }
  std::string click_toggle; // simulate the user clicking the menu item with this label
  std::map<std::string, bool> toggle_shown; // label -> the tick it was drawn with
  BoolEdit menu_item_toggle(const char *l, bool cur) override {
    rec(std::string("toggle:") + l);
    toggle_shown[l] = cur;
    if (click_toggle == l)
      return {true, true, !cur};
    return {};
  }
  std::map<std::string, BoolEdit> checkbox_by_label; // per-label returns (else checkbox_ret)
  BoolEdit checkbox(const char *l, bool) override {
    rec(std::string("checkbox:") + l);
    const auto it = checkbox_by_label.find(l);
    return it != checkbox_by_label.end() ? it->second : checkbox_ret;
  }
  IntEdit slider_int(const char *l, const char *, int cur, int, int) override {
    rec(std::string("slider_int:") + l + "=" + std::to_string(cur));
    return slider_int_ret;
  }
  std::map<std::string, DoubleEdit> slider_double_ret; // programmable returns, by slider KEY
  DoubleEdit slider_double(const char *l, const char *key, double, double, double,
                           const char *) override {
    rec(std::string("slider_double:") + l);
    const auto it = slider_double_ret.find(key ? key : "");
    return it != slider_double_ret.end() ? it->second : DoubleEdit{};
  }
  IndexEdit combo_ret{}; // programmable return (a select) for the next combo()
  IndexEdit combo(const char *l, int idx, const std::vector<std::string> &) override {
    rec(std::string("combo:") + l + "=" + std::to_string(idx));
    return combo_ret;
  }
  StringEdit string_ret{};     // programmable return for the next input_text()
  std::string last_input_text; // the `current` text the core handed input_text() (read side)
  StringEdit input_text(const char *l, const char *, const std::string &current) override {
    last_input_text = current;
    rec(std::string("input_text:") + l + "=" + current);
    return string_ret;
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
  void set_image(const std::string &name, const cvc::image &) override { rec("set_image:" + name); }
  // §10 tf_editor: a copy of what the core handed the backend (TfEditorInput's pointers do not
  // outlive the call), and a programmable edit. The default {drawn:false} makes the core compose
  // the editor from the leaves above.
  TfEdit tf_ret{};
  int tf_calls = 0;
  std::string last_tf_key;
  TfEditorInput last_tf_in;
  cvc::tf::model last_tf_model;
  cvc::tf::overlays last_tf_overlays;
  TfEdit tf_editor(const char *l, const char *key, const TfEditorInput &in) override {
    ++tf_calls;
    last_tf_key = key ? key : "";
    last_tf_in = in;
    last_tf_in.model = nullptr;
    last_tf_in.overlays = nullptr;
    last_tf_model = in.model ? *in.model : cvc::tf::model();
    last_tf_overlays = in.overlays ? *in.overlays : cvc::tf::overlays();
    rec(std::string("tf_editor:") + l + "@" + last_tf_key);
    return tf_ret;
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

TEST(AriadneRuntime, ColoredTextEmitsTintedLeaf) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  // A tinted literal Text and a tinted bound Text take the *_colored leaves; an untinted Text
  // alongside them still takes the plain leaf (the tint is purely additive).
  rt.set_root(
      group({with_text_color(text("hi"), 0.9f, 0.2f, 0.2f),
             with_text_color(text_bound("PDR", "rf.pdr"), 0.1f, 0.8f, 0.3f), text("plain")}));
  rt.render();
  EXPECT_TRUE(mb.saw("text_line_colored:hi")); // literal tinted -> colored leaf
  EXPECT_FALSE(mb.saw("text_line:hi"));        // ... and NOT the plain leaf
  EXPECT_TRUE(mb.saw("text_line:plain"));      // untinted -> plain leaf (default preserved)
  // The bound tinted Text ran the value-colored leaf; its colour reached the backend (it is the
  // last tinted widget walked).
  EXPECT_FLOAT_EQ(mb.last_text_rgb[0], 0.1f);
  EXPECT_FLOAT_EQ(mb.last_text_rgb[1], 0.8f);
  EXPECT_FLOAT_EQ(mb.last_text_rgb[2], 0.3f);
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

// Phase 0a: Kind::InputText dispatch — the core hands the backend the current text (read lane),
// writes the committed value to the bound key, and fires the optional `on` AFTER the write (so a
// send-on-enter handler sees the new value), queued off the walk like a Button's on.
TEST(AriadneRuntime, InputTextCommitsValueAndFiresOn) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  int sent = 0;
  rt.on("send", [&] { ++sent; });
  rt.set_root(group({input_text("Message", "chat.draft", "send")}));

  // No committed edit -> no write, no fire; the (empty, seeded) current text reaches the backend.
  mb.string_ret = {};
  rt.render();
  rt.drain();
  EXPECT_EQ(mb.last_input_text, "");
  EXPECT_EQ(sent, 0);

  // A committed edit writes the value and queues the on: (which runs on drain, not inside render).
  mb.string_ret = {true, true, std::string("hello")};
  rt.render();
  EXPECT_EQ(cvc::state::instance(app)("chat.draft").value(), "hello");
  EXPECT_EQ(sent, 0);
  rt.drain();
  EXPECT_EQ(sent, 1);
}

// Phase 0a: Kind::InputInt reuses the text primitive — the core seeds + stringifies the int on the
// read lane, and on commit parses it back (fail-safe: garbage keeps the current value) and clamps
// to [lo, hi].
TEST(AriadneRuntime, InputIntCoercesClampsAndIsFailSafe) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_root(group({input_int("Port", "net.port", 1, 65535, 8080)}));

  mb.string_ret = {};
  rt.render();
  EXPECT_EQ(mb.last_input_text, "8080"); // read lane: def seeded, int -> text
  EXPECT_EQ(cvc::state::instance(app)("net.port").value(), "8080"); // seeded into the key

  mb.string_ret = {true, true, std::string("443")}; // in range
  rt.render();
  EXPECT_EQ(cvc::state::instance(app)("net.port").value(), "443");

  mb.string_ret = {true, true, std::string("70000")}; // over range -> clamp to hi
  rt.render();
  EXPECT_EQ(cvc::state::instance(app)("net.port").value(), "65535");

  mb.string_ret = {true, true, std::string("abc")}; // non-numeric -> keep current
  rt.render();
  EXPECT_EQ(cvc::state::instance(app)("net.port").value(), "65535");
}

// §4/§7 action lane: an `on:` that is a state_exec PROGRAM (starts with '(', like a computed
// bind:/tooltip:) runs through state_exec at drain — no C++ handler. This is the north-star seam:
// a flag toggle / reset is pure .ari. The program is chrooted to the widget's prefix, so
// (state-set "paused") writes <prefix>.paused, the same key a widget `bind: paused` resolves to.
TEST(AriadneAction, ProgramOnTogglesStateThroughStateExec) {
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
  cvc::app app;
  Runtime rt(app, ""); // app-root prefix so the channel/state paths are used as-is
  MockBackend mb;
  rt.set_backend(&mb);
  // The action parks on (msg-recv …), then AFTER it is woken runs a state-set. Asserting on the
  // post-resume write (a sentinel) isolates the park → deliver → wake → resume path; the sibling
  // test ProgramActionCapturesDeliveredValueFromMsgRecv covers threading the delivered value into
  // the enclosing expression (which works — deliver_to_receivers patches the parent frame's
  // pending result slot where pop_frame left msg-recv's nil placeholder).
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

// The DELIVERED VALUE threads into the enclosing expression: (state-set k (msg-recv p)) writes
// the value the worker delivered. deliver_to_receivers patches the parent frame's pending result
// slot (where pop_frame pushed msg-recv's nil placeholder), so the resumed state-set applies it.
TEST(AriadneAction, ProgramActionCapturesDeliveredValueFromMsgRecv) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  Widget b = button("Go", "(state-set \"async.value\" (msg-recv \"vchan\"))");
  rt.set_root(group({b}));
  mb.button_click = true;
  rt.render();
  rt.drain(); // parks on msg-recv (value slot pending)
  mb.button_click = false;
  std::thread worker([&] {
    app.exec_scheduler().post_message("vchan", cvc::state_exec::value_t(std::string("delivered")));
  });
  worker.join();
  rt.render();
  rt.drain(); // wakes + resumes; the delivered value flows into state-set
  EXPECT_EQ(cvc::state::instance(app)("async.value").value(), "delivered");
}

// item 2 — the compute-pool async story, headless (no ImGui/GL): a HOST INTRINSIC launches an
// off-thread nav step on app.computePool() and hands the result back through the ONE thread-safe
// seam a worker may touch — exec_scheduler().post_message — waking a parked (msg-recv "nav.done")
// action that threads the value into state. Combines the host-intrinsic seam
// (HostIntrinsicCallableFromProgram) with the worker-delivers park/resume path
// (ProgramActionParksOnMsgRecvAndResumesWhenWorkerDelivers). msg-recv is race-free either way: it
// pops any already-buffered pending message before parking, so worker-posts-early also delivers.
// This is the raw cross-thread primitive (a worker posts, msg-recv wakes). A host verb is better
// written as an async function, which parks at the call and is charged to the action: see
// NavStepAsAnAsyncFunctionReturnsItsResult, the nav_compute demo's verb. Here we hand-roll a
// joinable std::thread so the test can join() deterministically before asserting.
TEST(AriadneAction, IntrinsicRunsNavStepOnComputePoolAndWakesMsgRecv) {
  namespace se = cvc::state_exec;
  cvc::app app;
  app.computePool();    // warm the lazy per-app singletons on THIS thread before a worker touches
  app.exec_scheduler(); // them, so a background worker never races their first construction
  Runtime rt(app, "");  // app-root prefix so the channel/state paths are used as-is
  MockBackend mb;
  rt.set_backend(&mb);

  // The host verb (nav-step-async) kicks off a background job and returns nil immediately — it must
  // NOT block, since it runs inline on the host thread inside drain(). The job fans the nav step
  // out over app.computePool() and, on join, posts the result. The test owns the worker handle so
  // it can join deterministically (no sleep/spin) before the delivering drain.
  std::thread worker;
  register_action_intrinsics([&](std::shared_ptr<se::environment> env, se::intrinsics_context &) {
    se::builtins::register_fn(env, "nav-step-async", [&](std::span<const se::value_t>) {
      worker = std::thread([&app] {
        std::atomic<int> arrived{0};
        const int n = 256;
        // The data-parallel nav step runs on the persistent compute-pool workers.
        app.computePool().parallel_for(n, [&arrived](int i) {
          if ((i % 3) == 0) // stand-in kernel: count the "arrived" agents
            arrived.fetch_add(1, std::memory_order_relaxed);
        });
        // The ONLY thread-safe scheduler contact a worker may make (async_scheduler.h): a
        // string payload matches the existing msg-recv tests and dodges int->state coercion.
        app.exec_scheduler().post_message("nav.done", se::value_t(std::to_string(arrived.load())));
      });
      return se::value_t{}; // nil; the action then parks on (msg-recv "nav.done")
    });
  });

  Widget b =
      button("Go", "(begin (nav-step-async) (state-set \"nav.result\" (msg-recv \"nav.done\")))");
  rt.set_root(group({b}));
  mb.button_click = true;
  rt.render();
  rt.drain(); // intrinsic launches the worker; the action parks on msg-recv (drain never blocks)
  mb.button_click = false;
  ASSERT_TRUE(worker.joinable());
  worker.join(); // make the post deterministic before the delivering drain

  rt.render();
  rt.drain(); // drain_ingress delivers "nav.done" -> wakes the parked action -> state-set runs
  clear_action_intrinsics();
  // i in [0,256), i%3==0 -> {0,3,...,255} = 86 values, so the worker computed arrived == 86.
  EXPECT_EQ(cvc::state::instance(app)("nav.result").value(), "86");
  EXPECT_TRUE(rt.take_reactive_warnings().empty());
}

// §7.1 resident on:tick: set_tick_program submits ONE long-lived process that runs the body once
// per drain (parking between frames), NOT a fresh action each frame. Proven by re-firing: reset a
// flag the body sets, and the next drain sets it again from the SAME resident.
TEST(AriadneResident, TickProgramRunsOncePerDrain) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_tick_program("(state-set \"tick.flag\" \"1\")");
  rt.drain(); // submit the resident + post the frame tick + pump -> body runs once
  EXPECT_EQ(cvc::state::instance(app)("tick.flag").value(), "1");
  cvc::state::instance(app)("tick.flag").value(std::string("0")); // reset
  rt.drain();                                                     // the resident fires AGAIN
  EXPECT_EQ(cvc::state::instance(app)("tick.flag").value(), "1");
  cvc::state::instance(app)("tick.flag").value(std::string("0"));
  rt.drain(); // and again — it is resident, not one-shot
  EXPECT_EQ(cvc::state::instance(app)("tick.flag").value(), "1");
  EXPECT_TRUE(rt.take_reactive_warnings().empty());
}

// The app's exec scheduler reads a submitted script inside its process (parser.step_tokens), so a
// resident whose body doesn't parse is killed by the syntax error once it is read, and the sweep
// reports it with the parse error rather than letting it vanish.
TEST(AriadneResident, ResidentWithASyntaxErrorIsReported) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_tick_program("(state-set \"tick.flag\" \"1\""); // unclosed
  rt.render();
  rt.drain();
  rt.drain();
  const std::vector<std::string> warns = rt.take_reactive_warnings();
  bool reported = false;
  for (const std::string &w : warns)
    reported = reported || (w.find("stopped") != std::string::npos &&
                            w.find("unclosed parenthesis") != std::string::npos);
  EXPECT_TRUE(reported) << "warnings: " << ::testing::PrintToString(warns);
  EXPECT_NE(cvc::state::instance(app)("tick.flag").value(), "1"); // the body never ran
}

// set_tick_program("") clears the resident: after clearing, a drain no longer re-fires the body.
TEST(AriadneResident, ClearingTickProgramStopsIt) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_tick_program("(state-set \"tick.flag\" \"1\")");
  rt.drain();
  EXPECT_EQ(cvc::state::instance(app)("tick.flag").value(), "1");
  rt.set_tick_program(""); // clear -> kills the resident
  cvc::state::instance(app)("tick.flag").value(std::string("0"));
  rt.drain();
  rt.drain();
  EXPECT_EQ(cvc::state::instance(app)("tick.flag").value(), "0"); // no longer firing
}

namespace {
// Another component's process on the app scheduler, as cvc::tf::schedule_overlays submits one: a
// frame-yielding loop that writes how many frames it has resumed from (await #f) to `key`.
struct ForeignAwaitLoop {
  cvc::app &app;
  cvc::state_exec::memory_tracker tracker;
  cvc::state_exec::intrinsics_context ictx;
  cvc::state_exec::environment_ptr env;
  int pid = -1;
  ForeignAwaitLoop(cvc::app &a, const std::string &key) : app(a) {
    namespace se = cvc::state_exec;
    se::async_scheduler &sched = app.exec_scheduler();
    ictx.sched = &sched;
    ictx.root = &cvc::state::instance(app);
    ictx.tracker = &tracker;
    ictx.proc = se::make_process(); // a placeholder: intrinsics reach the running process
    ictx.proc->status = se::process_status::ready;
    env = se::builtins::make_default_environment();
    se::register_intrinsics(env, &ictx);
    se::execute_options opts;
    opts.env = env;
    opts.owner = "component";
    pid =
        sched.execute("(begin (set n 0) (while t (begin (await #f) (set n (+ n 1)) (state-set \"" +
                          key + "\" (str n)))))",
                      opts);
  }
  ~ForeignAwaitLoop() { app.exec_scheduler().kill(pid); } // before ictx goes
  std::optional<cvc::state_exec::process_status> status() const {
    auto info = app.exec_scheduler().get_process_info(pid);
    return info ? std::optional<cvc::state_exec::process_status>(info->status) : std::nullopt;
  }
};

// A cvc::app that can say whether its exec scheduler was built, without building it.
struct SchedulerProbeApp : cvc::app {
  bool scheduler_built() {
    boost::mutex::scoped_lock lock(_execSchedulerMutex);
    return _execScheduler != nullptr;
  }
};
} // namespace

// Every Runtime pumps the app scheduler while a process is live on it, whoever submitted it: a
// document that runs no program, in a host that never called set_scene_capable (ariadne_hello,
// nav_convoy, ...), still advances a scene's overlay tasks. Its teardown leaves them alone (it
// reaps only its own processes).
TEST(AriadneScheduler, ARuntimeThatRanNoProgramAdvancesAnotherComponentsProcess) {
  cvc::app app;
  ForeignAwaitLoop loop(app, "pump.n");
  {
    Runtime rt(app, "ui");
    MockBackend mb;
    rt.set_backend(&mb);
    rt.set_root(group({text("hi")}));
    for (int frame = 0; frame < 4; ++frame) {
      rt.render();
      rt.drain();
    }
    // The first drain runs it to its first (await #f); each later one resumes it once.
    EXPECT_EQ(cvc::state::instance(app)("pump.n").value(), "3");
  }
  ASSERT_TRUE(loop.status().has_value());
  EXPECT_EQ(*loop.status(), cvc::state_exec::process_status::waiting); // parked, not killed
}

// Several Runtimes on one app share one frame boundary: each drains once a frame, in either order,
// and an (await #f) still resumes once a frame, not once per Runtime.
TEST(AriadneScheduler, SeveralRuntimesResumeAnAwaitOncePerFrame) {
  cvc::app app;
  ForeignAwaitLoop loop(app, "pump.n");
  Runtime a(app, "a");
  Runtime b(app, "b");
  MockBackend ma, mb;
  a.set_backend(&ma);
  b.set_backend(&mb);
  a.set_tick_program("(state-set \"ticked\" \"1\")"); // both run programs of their own too
  b.set_tick_program("(state-set \"ticked\" \"1\")");
  for (int frame = 0; frame < 5; ++frame) {
    if (frame < 3) {
      a.drain();
      b.drain();
    } else {
      b.drain();
      a.drain();
    }
  }
  EXPECT_EQ(cvc::state::instance(app)("pump.n").value(), "4");
  EXPECT_EQ(cvc::state::instance(app)("a.ticked").value(), "1");
  EXPECT_EQ(cvc::state::instance(app)("b.ticked").value(), "1");
  // One Runtime that stops drawing holds nothing up: the other alone still resumes it each frame.
  for (int frame = 0; frame < 3; ++frame)
    a.drain();
  EXPECT_EQ(cvc::state::instance(app)("pump.n").value(), "7");
}

// A document that never touches the scheduler, in an app where nothing else did either, pumps
// nothing and never builds it (exec_scheduler() is lazy).
TEST(AriadneScheduler, DrainNeverBuildsAnUnusedScheduler) {
  SchedulerProbeApp app;
  {
    Runtime rt(app, "ui");
    MockBackend mb;
    rt.set_backend(&mb);
    rt.set_root(group({text("hi")}));
    for (int frame = 0; frame < 3; ++frame) {
      rt.render();
      rt.drain();
    }
  }
  EXPECT_FALSE(app.scheduler_built());
}

// §4.6 input seam: set_key_program submits a resident that receives events posted via post_input,
// reading the delivered event dict with (get-attr event ...). post_input BEFORE drain -> delivered
// this frame (feed-before-drain contract). Keyboard events feed the key resident.
TEST(AriadneInput, KeyProgramReceivesPostedEvent) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_key_program("(begin (state-set \"input.last_key\" (get-attr event \"key\")) "
                     "(state-set \"input.kind\" (get-attr event \"kind\")))");
  InputEvent ev;
  ev.kind = InputEvent::Kind::KeyDown;
  ev.key = "Escape";
  rt.post_input(ev); // submit the key resident + queue the event
  rt.drain();        // pump: resident wakes, binds `event`, writes state
  EXPECT_EQ(cvc::state::instance(app)("input.last_key").value(), "Escape");
  EXPECT_EQ(cvc::state::instance(app)("input.kind").value(), "key_down");
}

// A burst of events in one frame all deliver (FIFO — no coalescing on the input channel): three
// keys posted before one drain, and the resident drains all three (last wins in state).
TEST(AriadneInput, KeyBurstAllDeliveredInOrder) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_key_program("(state-set \"input.last_key\" (get-attr event \"key\"))");
  for (const char *k : {"A", "B", "C"}) {
    InputEvent ev;
    ev.kind = InputEvent::Kind::KeyDown;
    ev.key = k;
    rt.post_input(ev);
  }
  rt.drain();
  EXPECT_EQ(cvc::state::instance(app)("input.last_key").value(), "C"); // all three ran, last wins
}

// Mouse events feed the pointer resident (a separate channel/handler); the body reads coords.
TEST(AriadneInput, PointerProgramReceivesMouseEvent) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  // state-set coerces a scalar to string, so the int button stores as "1".
  rt.set_pointer_program("(begin (state-set \"ptr.kind\" (get-attr event \"kind\")) "
                         "(state-set \"ptr.btn\" (get-attr event \"button\")))");
  InputEvent ev;
  ev.kind = InputEvent::Kind::MouseButtonDown;
  ev.button = 1;
  ev.x = 10.0f;
  ev.y = 20.0f;
  rt.post_input(ev);
  rt.drain();
  EXPECT_EQ(cvc::state::instance(app)("ptr.kind").value(), "mouse_button_down");
  EXPECT_EQ(cvc::state::instance(app)("ptr.btn").value(), "1");
}

// on_signal: a signal the host routes through post_signal wakes the document's signal resident with
// {kind, signal, source}. With no program set, post_signal says so (false) and the host applies the
// default; with one, it returns true and the program runs in the next drain, never before.
TEST(AriadneInput, SignalProgramReceivesPostedSignal) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  EXPECT_FALSE(rt.has_signal_program());
  EXPECT_FALSE(rt.post_signal("SIGINT", "terminal")); // nobody handles it: the host's default
  rt.set_signal_program("(begin (state-set \"sig.name\" (get-attr event \"signal\")) "
                        "(state-set \"sig.kind\" (get-attr event \"kind\")) "
                        "(state-set \"sig.source\" (get-attr event \"source\")))");
  EXPECT_TRUE(rt.has_signal_program());
  EXPECT_TRUE(rt.post_signal("SIGINT", "terminal"));
  EXPECT_NE(cvc::state::instance(app)("sig.name").value(), "SIGINT"); // not run yet
  rt.drain();
  EXPECT_EQ(cvc::state::instance(app)("sig.name").value(), "SIGINT");
  EXPECT_EQ(cvc::state::instance(app)("sig.kind").value(), "signal");
  EXPECT_EQ(cvc::state::instance(app)("sig.source").value(), "terminal");
  // Every signal arrives (no coalescing), each its own activation.
  EXPECT_TRUE(rt.post_signal("SIGQUIT"));
  rt.drain();
  EXPECT_EQ(cvc::state::instance(app)("sig.name").value(), "SIGQUIT");
  EXPECT_EQ(cvc::state::instance(app)("sig.source").value(), "host");
  rt.set_signal_program(""); // clearing it hands signals back to the host
  EXPECT_FALSE(rt.post_signal("SIGINT"));
}

// §4.6 widget-level on_click: any widget can carry an on_click program; the walk enqueues it when
// the backend reports the item was clicked (Backend::item_clicked -> ImGui::IsItemClicked natively
// and on wasm). Fired like a Button's on: (queued in render(), run in drain()) — NOT the SDL
// document-level stream. This is the natively-working widget input path.
TEST(AriadneInput, WidgetOnClickFiresWhenBackendReportsClick) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  Widget t = text("Clickable");
  t.on_click = "(state-set \"widget.clicked\" \"yes\")";
  rt.set_root(group({t}));
  // No click reported -> on_click must NOT fire.
  mb.item_clicked_flag = false;
  rt.render();
  rt.drain();
  EXPECT_NE(cvc::state::instance(app)("widget.clicked").value(), "yes");
  // Click reported -> render() enqueues on_click, drain() runs it.
  mb.item_clicked_flag = true;
  rt.render();
  EXPECT_NE(cvc::state::instance(app)("widget.clicked").value(), "yes"); // never inside render()
  rt.drain();
  EXPECT_EQ(cvc::state::instance(app)("widget.clicked").value(), "yes");
}

// §4.6 widget-level on_hover: the CONTINUOUS counterpart of on_click — the walk enqueues on_hover
// every frame the backend reports the item hovered (Backend::item_hovered -> ImGui::IsItemHovered).
// Uses the bare-name host-handler seam (no state_exec needed) and counts invocations to prove it
// fires only while hovered AND re-fires each frame (continuous, not a one-shot latch).
TEST(AriadneInput, WidgetOnHoverFiresContinuouslyWhileHovered) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  int hovers = 0;
  rt.on("hovered", [&] { ++hovers; });
  Widget t = text("Hoverable");
  t.on_hover = "hovered"; // bare name -> host handler
  rt.set_root(group({t}));
  // Not hovered -> on_hover must NOT fire.
  mb.item_hovered_flag = false;
  rt.render();
  rt.drain();
  EXPECT_EQ(hovers, 0);
  // Hovered -> fires; being continuous, each subsequent frame while still hovered fires AGAIN.
  mb.item_hovered_flag = true;
  rt.render();
  rt.drain();
  EXPECT_EQ(hovers, 1);
  rt.render();
  rt.drain();
  EXPECT_EQ(hovers, 2);
  // Leaving the item stops it firing.
  mb.item_hovered_flag = false;
  rt.render();
  rt.drain();
  EXPECT_EQ(hovers, 2);
}

// §4.6 widget-level on_drag: continuous while the item is actively dragged (Backend::item_dragged
// -> ImGui IsItemActive() && IsMouseDragging()). Same enqueue/drain path as on_click/on_hover.
TEST(AriadneInput, WidgetOnDragFiresWhileDragged) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  int drags = 0;
  rt.on("dragging", [&] { ++drags; });
  Widget t = text("Draggable");
  t.on_drag = "dragging"; // bare name -> host handler
  rt.set_root(group({t}));
  // Not dragged -> on_drag must NOT fire.
  mb.item_dragged_flag = false;
  rt.render();
  rt.drain();
  EXPECT_EQ(drags, 0);
  // Dragged -> render() enqueues, drain() runs; continuous while the drag is active.
  mb.item_dragged_flag = true;
  rt.render();
  rt.drain();
  EXPECT_EQ(drags, 1);
  rt.render();
  rt.drain();
  EXPECT_EQ(drags, 2);
}

// §4.6 on_drag_start / on_drag_end: one-shot EDGE handlers (item activate / deactivate). Bare-name
// host-handler seam, so no state_exec needed; each fires exactly on its edge frame.
TEST(AriadneInput, WidgetOnDragStartEndFireOnEdges) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  int starts = 0, ends = 0;
  rt.on("dstart", [&] { ++starts; });
  rt.on("dend", [&] { ++ends; });
  Widget t = text("Draggable");
  t.on_drag_start = "dstart";
  t.on_drag_end = "dend";
  rt.set_root(group({t}));
  rt.render();
  rt.drain();
  EXPECT_EQ(starts, 0);
  EXPECT_EQ(ends, 0);
  // drag-start edge
  mb.item_drag_started_flag = true;
  rt.render();
  rt.drain();
  mb.item_drag_started_flag = false;
  EXPECT_EQ(starts, 1);
  EXPECT_EQ(ends, 0);
  // drag-end edge
  mb.item_drag_ended_flag = true;
  rt.render();
  rt.drain();
  mb.item_drag_ended_flag = false;
  EXPECT_EQ(starts, 1);
  EXPECT_EQ(ends, 1);
}

// §4.6 event scope: a widget POINTER handler that is a PROGRAM runs with an `event` dict bound to
// the item's pointer coords, read with (get-attr event "..."). Assert on button (int -> a stable
// string), like the on_pointer test, to avoid double-format brittleness.
TEST(AriadneInput, WidgetPointerProgramReceivesEventScope) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  mb.ptr_x = 7;
  mb.ptr_y = 9;
  mb.ptr_button = 2;
  Widget t = text("Clickable");
  t.on_click = "(begin (state-set \"ev.btn\" (get-attr event \"button\")) "
               "(state-set \"ev.has_x\" (if (>= (get-attr event \"x\") 0) \"yes\" \"no\")))";
  rt.set_root(group({t}));
  mb.item_clicked_flag = true;
  rt.render();
  rt.drain();
  EXPECT_EQ(cvc::state::instance(app)("ev.btn").value(), "2");     // the event dict was delivered
  EXPECT_EQ(cvc::state::instance(app)("ev.has_x").value(), "yes"); // and x is readable (7 >= 0)
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

// A slider writes its key on every change while it is dragged; with commit: release, only when the
// drag ends.
TEST(AriadneRuntime, SliderWritesWhileDraggedUnlessCommitOnRelease) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  Widget held = slider_int("Held", "demo.held", 0, 100, 5);
  held.commit_on_release = true;
  rt.set_root(group({slider_int("N", "demo.n", 0, 100, 5), held}));
  mb.slider_int_ret = IntEdit{true, false, 37}; // mid-drag: changed, not yet released
  rt.render();
  EXPECT_EQ(cvc::state::instance(app)("demo.n").value(), "37");
  EXPECT_EQ(cvc::state::instance(app)("demo.held").value(), "5"); // the seed, not the drag
  mb.slider_int_ret = IntEdit{false, true, 40};                   // released
  rt.render();
  EXPECT_EQ(cvc::state::instance(app)("demo.held").value(), "40");
}

// Key names compare across spellings: SDL's ("Left Shift", "Space", "PageDown") and VTK's
// ("Shift_L", "space", "Next"); letters fold to lower case.
TEST(AriadneKeys, CanonicalNamesMatchAcrossSpellings) {
  EXPECT_EQ(canonical_key("Left Shift"), canonical_key("Shift_L"));
  EXPECT_EQ(canonical_key("Space"), canonical_key("space"));
  EXPECT_EQ(canonical_key("PageDown"), canonical_key("Next"));
  EXPECT_EQ(canonical_key("PageUp"), canonical_key("Prior"));
  EXPECT_EQ(canonical_key("Left Ctrl"), canonical_key("Control_L"));
  EXPECT_EQ(canonical_key("W"), canonical_key("w"));
  EXPECT_NE(canonical_key("Left"), canonical_key("Right"));
  EXPECT_EQ(canonical_key(","), canonical_key("comma"));
  EXPECT_EQ(canonical_key(","), canonical_key("Comma"));
  EXPECT_EQ(canonical_key("/"), canonical_key("slash"));
  EXPECT_EQ(canonical_key("\\"), canonical_key("backslash"));
  EXPECT_EQ(canonical_key("["), canonical_key("bracketleft"));
  EXPECT_EQ(canonical_key("`"), canonical_key("grave"));
  EXPECT_EQ(canonical_key("Keypad 1"), canonical_key("KP_1"));
  EXPECT_EQ(canonical_key("Keypad +"), canonical_key("KP_Add"));
  EXPECT_EQ(canonical_key("Keypad Enter"), canonical_key("KP_Enter"));
  EXPECT_EQ(canonical_key("Numlock"), canonical_key("Num_Lock"));
  EXPECT_EQ(canonical_key("PrintScreen"), canonical_key("Print"));
  EXPECT_NE(canonical_key("Keypad 1"), canonical_key("1"));
  EXPECT_NE(canonical_key("Keypad -"), canonical_key("-"));
  EXPECT_EQ(canonical_key("_"), canonical_key("underscore"));
  EXPECT_NE(canonical_key("_"), "") << "not a key with no name";
  EXPECT_EQ(canonical_key("+"), canonical_key("plus"));
  EXPECT_EQ(canonical_key("<"), canonical_key("less"));
  EXPECT_EQ(canonical_key("'"), canonical_key("quoteright")) << "VTK's Win32 / Cocoa name";
  EXPECT_EQ(canonical_key("`"), canonical_key("quoteleft"));
  EXPECT_EQ(join_keys({"Right", ","}), "Right, Comma") << "a document's comma default";
  EXPECT_TRUE(key_matches(join_keys({"Right", ","}), ","));
  EXPECT_EQ(stored_key_name(","), "Comma") << "a comma would split the stored keys";
  EXPECT_EQ(stored_key_name("Right"), "Right");
  EXPECT_EQ(stored_key_name(""), "");
  EXPECT_EQ(join_keys({"Right", "PageDown"}), "Right, PageDown");
  EXPECT_EQ(split_keys(" Right ,PageDown,, "), (std::vector<std::string>{"Right", "PageDown"}));
  EXPECT_TRUE(key_matches("Right, PageDown", "Next"));
  EXPECT_FALSE(key_matches("Right, PageDown", "Left"));
  EXPECT_FALSE(key_matches("", "Left"));
}

// A key down that matches a binding runs its action in the next drain; with the Keys window open
// and its capture naming the binding, the next key down becomes its key instead (and runs
// nothing), and Escape keeps the old one. Closing the window drops a capture still waiting.
TEST(AriadneRuntime, KeyBindingsRunTheirActionAndRebind) {
  cvc::app app;
  Runtime rt(app, "ui.k");
  MockBackend mb;
  rt.set_backend(&mb);
  auto &st = cvc::state::instance(app);
  st("ui.k.n").value(0);
  st("ui.k.keys.next").value(std::string("Right, PageDown"));
  rt.set_key_bindings({{"next",
                        "Next",
                        "",
                        {"Right", "PageDown"},
                        "(state-set \"n\" (+ (int (state-get \"n\")) 1))",
                        ""}});
  const auto press = [&](const std::string &key, bool down = true) {
    InputEvent e;
    e.kind = down ? InputEvent::Kind::KeyDown : InputEvent::Kind::KeyUp;
    e.key = key;
    rt.post_input(e);
    rt.drain();
  };
  press("Right");
  EXPECT_EQ(st("ui.k.n").value(), "1");
  press("Next"); // VTK's name for PageDown
  EXPECT_EQ(st("ui.k.n").value(), "2");
  press("Right", /*down=*/false);
  press("Left");
  EXPECT_EQ(st("ui.k.n").value(), "2") << "a key up, and an unbound key, run nothing";

  const std::string capture = "ui.k.tree.ariadne_keys.capture",
                    open = "ui.k.tree.ariadne_keys.open";
  st(open).value(1);
  st(capture).value(std::string("next")); // the Keys window's Change
  press("");
  EXPECT_EQ(st(capture).value(), "next") << "a key with no name: still waiting";
  press("Up");
  EXPECT_EQ(st("ui.k.keys.next").value(), "Up");
  EXPECT_EQ(st(capture).value(), "");
  EXPECT_EQ(st("ui.k.n").value(), "2") << "the captured key is not a press";
  press("Right");
  EXPECT_EQ(st("ui.k.n").value(), "2");
  press("Up");
  EXPECT_EQ(st("ui.k.n").value(), "3");

  st(capture).value(std::string("next"));
  press("Escape");
  EXPECT_EQ(st("ui.k.keys.next").value(), "Up") << "Escape keeps the old key";
  EXPECT_EQ(st(capture).value(), "");

  st(capture).value(std::string("next"));
  press(",");
  EXPECT_EQ(st("ui.k.keys.next").value(), "Comma") << "the comma key, stored so it can't split";
  press(",");
  EXPECT_EQ(st("ui.k.n").value(), "4") << "and it is the binding's key";

  st(capture).value(std::string("next"));
  st(open).value(0); // the user closed the Keys window instead
  press("Space");
  EXPECT_EQ(st("ui.k.keys.next").value(), "Comma") << "a closed window takes no key";
  EXPECT_EQ(st(capture).value(), "") << "and drops the capture";
  st(open).value(1);
  press("Space");
  EXPECT_EQ(st("ui.k.keys.next").value(), "Comma") << "reopened, it waits for a new Change";
}

// A document's key bindings live beside its other state, not among the Keys window's: a binding
// named `capture` is the document's own.
TEST(AriadneRuntime, ABindingNamedCaptureIsTheDocuments) {
  cvc::app app;
  Runtime rt(app, "ui.k");
  MockBackend mb;
  rt.set_backend(&mb);
  auto &st = cvc::state::instance(app);
  st("ui.k.n").value(0);
  st("ui.k.keys.capture").value(std::string("C"));
  rt.set_key_bindings({{"capture", "", "", {"C"}, "(state-set \"n\" 7)", ""}});
  EXPECT_FALSE(rt.key_capture_pending());
  InputEvent e;
  e.kind = InputEvent::Kind::KeyDown;
  e.key = "C";
  rt.post_input(e);
  rt.drain();
  EXPECT_EQ(st("ui.k.n").value(), "7");
}

// A group with an id scopes its children's identities, as a window does.
TEST(AriadneRuntime, GroupIdScopesItsChildren) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  Widget a = group({button("Change", "a")});
  a.id = "row_a";
  rt.set_root(group({a, group({button("Change", "b")})}));
  rt.render();
  const auto at = [&](const std::string &e) {
    return std::find(mb.log.begin(), mb.log.end(), e) - mb.log.begin();
  };
  EXPECT_LT(at("push_id:row_a"), at("button:Change"));
  EXPECT_LT(at("button:Change"), at("pop_id"));
  EXPECT_EQ(std::count(mb.log.begin(), mb.log.end(), "push_id:row_a"), 1);
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
  cvc::app app;
  std::vector<std::string> errs;
  // A relative path under the chroot prefix -> writes <prefix>.agents, the same key a
  // widget `bind: agents` (prefix "ui.demo") would resolve to.
  const bool ok = run_init(app, "ui.demo", "(state-set \"agents\" \"128\")", &errs);
  ASSERT_TRUE(ok) << (errs.empty() ? std::string("(no error)") : errs[0]);
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.agents").value(), "128");
}

TEST(AriadneInit, SyntaxErrorReportedNotThrown) {
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
  if (!have_yaml())
    GTEST_SKIP() << "needs yaml";
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
  if (!have_yaml())
    GTEST_SKIP() << "needs yaml";
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

// PR-C render-pass park safety: a park verb (msg-recv/await/http-get/fetch) in a reactive predicate
// is NOT in the read-lane allowlist, so it is DENIED — the render walk cannot park (its private
// scheduler is never pumped; a park there would silent-nil). render() must return, hidden +
// reported.
TEST(AriadneReactive, ParkVerbInPredicateIsDeniedNotHung) {
  // Both park verbs (msg-recv AND await) must be denied in the reactive read lane — neither is in
  // the read-lane allowlist, so a predicate using one is unbound → fail-safe hidden, render
  // returns.
  for (const char *predicate : {"(msg-recv \"x\")", "(await 1)"}) {
    cvc::app app;
    Runtime rt(app, "");
    MockBackend mb;
    rt.set_backend(&mb);
    Widget w = text("shown");
    w.visible_when = predicate;
    rt.set_root(group({w}));
    rt.render(); // MUST return — the read lane cannot park/block
    EXPECT_FALSE(mb.saw("text_line:shown")) << "predicate: " << predicate; // denied -> hidden
    EXPECT_FALSE(rt.take_reactive_warnings().empty()) << "predicate: " << predicate; // and reported
  }
}

// PR-C end-to-end: a runaway on:tick resident (a body that never parks) is bounded by the
// per-activation budget wired in ensure_resident — drain() returns, and the scheduler recovers so a
// subsequently-installed healthy resident still fires.
TEST(AriadneResident, RunawayTickResidentIsBoundedAndSchedulerRecovers) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_tick_program("(while true (+ 1 1))"); // runaway: the body never parks
  rt.render();
  rt.drain(); // MUST return (per-activation budget aborts the body; the pump caps regardless)
  // Replace with a healthy resident; if the runaway had wedged the scheduler this would never fire.
  rt.set_tick_program("(state-set \"tick.flag\" \"1\")");
  for (int i = 0; i < 4; ++i) {
    rt.render();
    rt.drain();
  }
  EXPECT_EQ(cvc::state::instance(app)("tick.flag").value(), "1");
}

TEST(AriadneReactive, StringValueIsTruthyAndUnsetKeyIsCleanlyFalsy) {
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

// The stdlib is in every program lane's environment, and a predicate gets its pure, bounded
// subset (a scalar, or a result no bigger than its argument).
TEST(AriadneStdlib, PredicateGetsThePureBoundedSubset) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  ASSERT_TRUE(run_init(app, "", "(state-set \"name\" \"caf\xC3\xA9\")", nullptr));
  Widget len = text("len");
  len.visible_when = "(= (string.length (state-get \"name\")) 4)"; // codepoints -> shown
  Widget up = text("up");
  up.visible_when = "(string.starts-with (string.upper (state-get \"name\")) \"CAF\")";
  Widget math = text("math");
  math.visible_when = "(= (math.floor (math.sqrt 17)) 4)";
  Widget bytes = text("bytes");
  bytes.visible_when = "(= (bytes.byte-at b\"\\x01\\xff\" 1) 255)";
  rt.set_root(group({len, up, math, bytes}));
  rt.render();
  EXPECT_TRUE(mb.saw("text_line:len"));
  EXPECT_TRUE(mb.saw("text_line:up"));
  EXPECT_TRUE(mb.saw("text_line:math"));
  EXPECT_TRUE(mb.saw("text_line:bytes"));
  EXPECT_TRUE(rt.take_reactive_warnings().empty());
}

// The materializers run in a predicate as sliced data steps, and collections.map/filter/reduce
// take a lambda there; math.random stays out (a predicate must be deterministic).
TEST(AriadneStdlib, PredicateRunsTheMaterializersButNotRandom) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  ASSERT_TRUE(run_init(app, "", "(state-set \"csv\" \"b,a,c,a\")", nullptr));
  const std::vector<std::pair<const char *, const char *>> shown = {
      {"split", "(= (length (string.split (state-get \"csv\") \",\")) 4)"},
      {"join", "(= (string.join (collections.sort (string.split (state-get \"csv\") \",\")) \"\") "
               "\"aabc\")"},
      {"replace", "(= (string.replace (state-get \"csv\") \",\" \"\") \"baca\")"},
      {"bytes", "(= (bytes.decode (bytes.concat (bytes.slice (string.encode \"xab\") 1) "
                "(string.encode \"!\"))) \"ab!\")"},
      {"map", "(= (collections.map (list 1 2 3) (lambda (x) (* x x))) (list 1 4 9))"},
      {"filter", "(= (length (collections.filter (collections.range 10) (lambda (x) (> x 6)))) 3)"},
      {"reduce", "(= (collections.reduce (collections.range 5) (lambda (a x) (+ a x)) 0) 10)"},
      {"unique", "(= (collections.unique (string.split (state-get \"csv\") \",\")) "
                 "(list \"b\" \"a\" \"c\"))"},
      {"dicts", "(= (collections.dict-keys (collections.dict-merge (dict \"a\" 1) (dict \"b\" 2))) "
                "(list \"a\" \"b\"))"},
      {"shape", "(= (collections.flatten (collections.zip (list 1 2) (collections.reverse (list 3 "
                "4)))) (list 1 4 2 3))"}};
  std::vector<Widget> widgets;
  for (const auto &[name, pred] : shown) {
    widgets.push_back(text(name));
    widgets.back().visible_when = pred;
  }
  Widget rnd = text("rnd");
  rnd.visible_when = "(< (math.random) 2)"; // unbound -> fail-safe hidden
  widgets.push_back(rnd);
  rt.set_root(group(widgets));
  rt.render();
  for (const auto &[name, pred] : shown)
    EXPECT_TRUE(mb.saw(std::string("text_line:") + name)) << pred;
  EXPECT_FALSE(mb.saw("text_line:rnd"));
  const auto warns = rt.take_reactive_warnings();
  ASSERT_EQ(warns.size(), 1u) << (warns.empty() ? "" : warns[0]);
  EXPECT_NE(warns[0].find("math.random"), std::string::npos) << warns[0];
}

// A predicate's deep-data work runs a slice per step, so its step and time caps cut a huge
// materializer instead of the frame waiting for it. data.step_nodes comes from the app tree.
TEST(AriadneStdlib, PredicateMaterializerIsCutByTheEvalBudget) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  cvc::state::instance(app)("state_exec.defaults.data.step_nodes").value("1");
  Widget big = text("big");
  big.visible_when = "(> (length (collections.range 120000)) 0)"; // > 100000 one-node steps
  rt.set_root(group({big}));
  rt.render();
  EXPECT_FALSE(mb.saw("text_line:big"));
  auto warns = rt.take_reactive_warnings();
  ASSERT_FALSE(warns.empty());
  // Cut by the step cap or the time cap, whichever comes first. The time cap reads the same
  // whether it expires between steps or the job's own deadline poll sees it first, as on a slow
  // Debug build: both throw evaluation_timeout.
  EXPECT_NE(warns[0].find("exceeded the eval budget"), std::string::npos) << warns[0];

  // A 0 here still slices, at the host's 1024 nodes a step, and an ordinary call finishes. It is
  // kept small so it fits the 5 ms time cap in a Debug build on a loaded CI runner.
  cvc::state::instance(app)("state_exec.defaults.data.step_nodes").value("0");
  Widget fits = text("fits");
  fits.visible_when = "(= (length (collections.range 1000)) 1000)";
  rt.set_root(group({fits}));
  MockBackend mb2;
  rt.set_backend(&mb2);
  rt.render();
  EXPECT_TRUE(mb2.saw("text_line:fits"));
  EXPECT_TRUE(rt.take_reactive_warnings().empty());
}

// A lambda a predicate passes to collections.map runs on the predicate's own evaluator: the
// special-form gate, the reduced environment and the caps all hold inside it.
TEST(AriadneStdlib, PredicateLambdasStayInsideThePredicateRules) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  Widget macro = text("macro");
  macro.visible_when = "(collections.map (list 1) (lambda (x) (defmacro m () 1)))";
  rt.set_root(group({macro}));
  rt.render();
  EXPECT_FALSE(mb.saw("text_line:macro"));
  auto warns = rt.take_reactive_warnings();
  ASSERT_FALSE(warns.empty());
  EXPECT_NE(warns[0].find("not permitted"), std::string::npos) << warns[0];

  Widget write = text("write");
  write.visible_when = "(collections.map (list 1) (lambda (x) (state-set \"leak\" \"1\")))";
  rt.set_root(group({write}));
  rt.render();
  EXPECT_FALSE(mb.saw("text_line:write"));
  EXPECT_FALSE(rt.take_reactive_warnings().empty());
  EXPECT_NE(cvc::state::instance(app)("leak").value(), "1");

  Widget spin = text("spin");
  spin.visible_when = "(collections.map (list 1) (lambda (x) (while t 1)))";
  rt.set_root(group({spin}));
  rt.render();
  EXPECT_FALSE(mb.saw("text_line:spin"));
  warns = rt.take_reactive_warnings();
  ASSERT_FALSE(warns.empty());
  EXPECT_NE(warns[0].find("exceeded the eval budget"), std::string::npos) << warns[0];
}

TEST(AriadneStdlib, ActionLaneCallsLambdasThroughMapAndApply) {
  cvc::app app;
  Runtime rt(app, "ui");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_root(group({button("Go", "(begin (state-set \"out\" (string.join (collections.map "
                                  "(string.split \"a b c\" \" \") (lambda (w) (string.upper w))) "
                                  "\"-\")) (state-set \"sum\" (apply (lambda (a b) (+ a b)) "
                                  "(list 2 3))))")}));
  mb.button_click = true;
  rt.render();
  rt.drain();
  EXPECT_EQ(cvc::state::instance(app)("ui.out").value(), "A-B-C");
  EXPECT_EQ(cvc::state::instance(app)("ui.sum").value(), "5");
  EXPECT_TRUE(rt.take_reactive_warnings().empty());
}

TEST(AriadneStdlib, InitAndActionLanesReachTheWholeStdlib) {
  cvc::app app;
  std::vector<std::string> errs;
  ASSERT_TRUE(run_init(app, "ui",
                       "(state-set \"words\" (string.join (collections.reverse "
                       "(string.split \"a b c\" \" \")) \",\"))",
                       &errs))
      << (errs.empty() ? std::string("(no error)") : errs[0]);
  EXPECT_EQ(cvc::state::instance(app)("ui.words").value(), "c,b,a");

  Runtime rt(app, "ui");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_root(group({button("Go", "(state-set \"shout\" (string.upper (state-get \"words\")))")}));
  mb.button_click = true;
  rt.render();
  rt.drain();
  EXPECT_EQ(cvc::state::instance(app)("ui.shout").value(), "C,B,A");
  EXPECT_TRUE(rt.take_reactive_warnings().empty());
}

TEST(AriadneStdlib, ActionHitsTheAllocationLimitAndIsReported) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  // The app's exec scheduler reads limits.max_alloc_bytes on every submission.
  cvc::state::instance(app)("state_exec.defaults.limits.max_alloc_bytes").value("1024");
  rt.set_root(group({button("Big", "(state-set \"n\" (length (collections.range 1000)))")}));
  mb.button_click = true;
  rt.render();
  rt.drain();
  EXPECT_FALSE(cvc::state::instance(app)("n").value() == "1000"); // failed before the write
  const auto warns = rt.take_reactive_warnings();
  ASSERT_FALSE(warns.empty());
  // The refused allocation is a MemoryError the action didn't catch: it failed, and the report
  // names the class and message (docs/STATE_EXEC_EXCEPTIONS.md 2.6).
  EXPECT_NE(warns[0].find("action program failed: MemoryError: "), std::string::npos) << warns[0];

  // A limit that fits lets the same action finish.
  cvc::state::instance(app)("state_exec.defaults.limits.max_alloc_bytes").value("1048576");
  mb.button_click = true;
  rt.render();
  rt.drain();
  EXPECT_EQ(cvc::state::instance(app)("n").value(), "1000");
}

// An action's memory budget is the tree setting ariadne.limits.action_max_memory (default
// 4 MiB, 0 = none), read on every submission; it also bounds each native call's allocations.
TEST(AriadneStdlib, ActionMemoryBudgetIsATreeSetting) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  auto &root = cvc::state::instance(app);
  // 10000 elements are well over the old fixed 256 KiB and well under 4 MiB.
  rt.set_root(group({button("Big", "(state-set \"n\" (length (collections.range 10000)))")}));
  const auto click = [&] {
    root("n").value(std::string("-"));
    mb.button_click = true;
    rt.render();
    rt.drain();
    return root("n").value();
  };
  EXPECT_EQ(click(), "10000"); // the 4 MiB default
  EXPECT_TRUE(rt.take_reactive_warnings().empty());

  root("ariadne.limits.action_max_memory").value(std::string("65536"));
  EXPECT_EQ(click(), "-"); // over the budget: failed before the write
  const auto warns = rt.take_reactive_warnings();
  ASSERT_FALSE(warns.empty());
  EXPECT_NE(warns[0].find("action program failed: MemoryError: "), std::string::npos) << warns[0];

  root("ariadne.limits.action_max_memory").value(std::string("0")); // no limit
  EXPECT_EQ(click(), "10000");
  root("ariadne.limits.action_max_memory").value(std::string("lots")); // unparsable: default
  EXPECT_EQ(click(), "10000");
}

// A finished action's process is reaped from the app scheduler, and a function it defined (bound
// in the scope it captured: a reference cycle) is freed by the cycle collector.
TEST(AriadneHeap, FinishedActionsAreReapedAndTheirCyclesFreed) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_root(group({button("Go", "(begin (defun helper (x) x) (state-set \"n\" (helper 7)))")}));
  cvc::state_exec::collect_cycles(); // earlier tests' leftovers
  const std::size_t before = cvc::state_exec::tracked_objects();
  for (int i = 0; i < 20; ++i) {
    mb.button_click = true;
    rt.render();
    rt.drain();
  }
  EXPECT_EQ(cvc::state::instance(app)("n").value(), "7");
  EXPECT_EQ(app.exec_scheduler().process_count(), 0); // every finished action was reaped
  cvc::state_exec::collect_cycles();
  EXPECT_LE(cvc::state_exec::tracked_objects(), before); // the 20 action scopes are gone
}

// An action's live values count against its memory budget, not only its state-tree writes.
TEST(AriadneHeap, AnActionsLiveValuesCountAgainstItsBudget) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  auto &root = cvc::state::instance(app);
  rt.set_root(group({button("Keep", "(begin (set keep (list)) "
                                    "(append keep (collections.range 1000)) "
                                    "(append keep (collections.range 1000)) "
                                    "(append keep (collections.range 1000)) "
                                    "(append keep (collections.range 1000)) "
                                    "(append keep (collections.range 1000)) "
                                    "(append keep (collections.range 1000)) "
                                    "(state-set \"n\" \"kept\"))")}));
  const auto click = [&] {
    root("n").value(std::string("-"));
    mb.button_click = true;
    rt.render();
    rt.drain();
    return root("n").value();
  };
  EXPECT_EQ(click(), "kept"); // well under the 4 MiB default
  root("ariadne.limits.action_max_memory").value(std::string("200000"));
  EXPECT_EQ(click(), "-"); // 6000 live elements are over 200 KB: killed before the write
  EXPECT_FALSE(rt.take_reactive_warnings().empty());
}

TEST(AriadneReactive, DeniedSpecialFormsAreRejectedNotRun) {
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

// ---- §11.4 runtime geometry / track persistence (tree.<id>) ----------------------------------
// The two-way edge: the core seeds a window / split-grid from the persisted tree.<id> state before
// the backend opens it, and writes the post-interaction geometry back after — the state node is the
// authority next frame. Driven headlessly through the MockBackend (no ImGui): the backend records
// the seed it was handed and reports a programmable "current" geometry / track list.

TEST(AriadneRuntime, WindowGeometrySeedsBackendFromPersistedState) {
  cvc::app app;
  Runtime rt(app, "ui.demo");
  MockBackend mb;
  rt.set_backend(&mb);
  // A geometry persisted under the window's id (which defaults to the label "Inspector").
  cvc::state::instance(app)("ui.demo.tree.Inspector.geometry").value(std::string("10,20,300,400"));
  // A settled window reports back what it was seeded to (so the write-back no-ops via the guard).
  mb.window_geom_ret = WindowGeom{true, true, 10, 20, 300, 400};
  rt.set_root(group({window("Inspector", {text("hi")})}));
  rt.render();
  ASSERT_TRUE(mb.seeded_geom_called);
  EXPECT_TRUE(mb.seeded_geom.has_pos);
  EXPECT_TRUE(mb.seeded_geom.has_size);
  EXPECT_FLOAT_EQ(mb.seeded_geom.x, 10.0f);
  EXPECT_FLOAT_EQ(mb.seeded_geom.y, 20.0f);
  EXPECT_FLOAT_EQ(mb.seeded_geom.w, 300.0f);
  EXPECT_FLOAT_EQ(mb.seeded_geom.h, 400.0f);
}

TEST(AriadneRuntime, WindowGeometryWritesPostInteractionBack) {
  cvc::app app;
  Runtime rt(app, "ui.demo");
  MockBackend mb;
  rt.set_backend(&mb);
  // The backend reports a moved/resized window this frame (fractional -> rounded to px on write).
  mb.window_geom_ret = WindowGeom{true, true, 15.4f, 24.6f, 320.2f, 409.8f};
  rt.set_root(group({window("Inspector", {text("hi")})}));
  rt.render();
  EXPECT_FALSE(mb.seeded_geom_called); // nothing persisted yet -> no seed
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.tree.Inspector.geometry").value(), "15,25,320,410");
}

TEST(AriadneRuntime, WindowGeometryRoundTripsAcrossFrames) {
  cvc::app app;
  Runtime rt(app, "ui.demo");
  MockBackend mb;
  rt.set_backend(&mb);
  mb.window_geom_ret = WindowGeom{true, true, 40, 50, 200, 150};
  rt.set_root(group({window("W", {text("x")})}));
  rt.render(); // frame 1: writes 40,50,200,150 to state (no prior state -> no seed)
  EXPECT_FALSE(mb.seeded_geom_called);
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.tree.W.geometry").value(), "40,50,200,150");
  // frame 2: the core reads that back and seeds the backend with it.
  mb.seeded_geom_called = false;
  rt.render();
  ASSERT_TRUE(mb.seeded_geom_called);
  EXPECT_FLOAT_EQ(mb.seeded_geom.x, 40.0f);
  EXPECT_FLOAT_EQ(mb.seeded_geom.h, 150.0f);
}

TEST(AriadneRuntime, CollapsedWindowPreservesPersistedSize) {
  cvc::app app;
  Runtime rt(app, "ui.demo");
  MockBackend mb;
  rt.set_backend(&mb);
  cvc::state::instance(app)("ui.demo.tree.W.geometry").value(std::string("10,20,300,400"));
  // Collapsed: the backend reports a position but no size (has_size == false).
  mb.window_geom_ret = WindowGeom{true, false, 12, 22, 0, 0};
  rt.set_root(group({window("W", {text("x")})}));
  rt.render();
  // Position follows the drag; the size is back-filled from the seed, never clobbered to 0.
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.tree.W.geometry").value(), "12,22,300,400");
}

TEST(AriadneRuntime, SplitGridTracksSeedAndPersist) {
  cvc::app app;
  Runtime rt(app, "ui.demo");
  MockBackend mb;
  rt.set_backend(&mb);
  // A resizable grid with row tracks is a row-split (Layout::is_row_split()).
  Widget g = group({text("top"), text("bottom")});
  g.id = "Split";
  g.layout.kind = LayoutKind::Grid;
  g.layout.resizable = true;
  g.layout.row_heights = {Track{Unit::Px, 80.0f}, Track{Unit::Px, 200.0f}};
  // Persisted pane sizes + a dragged report from the backend.
  cvc::state::instance(app)("ui.demo.tree.Split.tracks").value(std::string("90,210"));
  mb.grid_tracks_ret = {120.0f, 260.0f};
  rt.set_root(group({g}));
  rt.render();
  ASSERT_TRUE(mb.seeded_tracks_called);
  ASSERT_EQ(mb.seeded_tracks.size(), 2u);
  EXPECT_FLOAT_EQ(mb.seeded_tracks[0], 90.0f);
  EXPECT_FLOAT_EQ(mb.seeded_tracks[1], 210.0f);
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.tree.Split.tracks").value(), "120,260");
}

TEST(AriadneRuntime, PlainTableGridDoesNotPersistTracks) {
  cvc::app app;
  Runtime rt(app, "ui.demo");
  MockBackend mb;
  rt.set_backend(&mb);
  // A *resizable* grid with only column tracks and NO row tracks is a plain table, not a row-split
  // (is_row_split() = resizable && !row_heights.empty() — false on the empty-rows half): no track
  // edge.
  Widget g = group({text("a"), text("b")});
  g.id = "Table";
  g.layout.kind = LayoutKind::Grid;
  g.layout.resizable = true; // resizable columns, but no row tracks -> still not a row-split
  g.layout.col_widths = {Track{Unit::Px, 100.0f}, Track{Unit::Px, 100.0f}};
  mb.grid_tracks_ret = {5.0f, 6.0f}; // even if the backend reported sizes, the core ignores them
  rt.set_root(group({g}));
  rt.render();
  EXPECT_FALSE(mb.seeded_tracks_called);
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.tree.Table.tracks").value(), ""); // never written
}

TEST(AriadneRuntime, ResizableTableColumnsSeedAndPersist) {
  cvc::app app;
  Runtime rt(app, "ui.demo");
  MockBackend mb;
  rt.set_backend(&mb);
  // A resizable grid with column tracks and no row tracks is a resizable table: its column widths
  // take the same two-way edge as a row split's panes, under tree.<id>.columns.
  Widget g = group({text("a"), text("b")});
  g.id = "Table";
  g.layout.kind = LayoutKind::Grid;
  g.layout.resizable = true;
  g.layout.col_widths = {Track{Unit::Px, 100.0f}, Track{Unit::Percent, 100.0f}};
  cvc::state::instance(app)("ui.demo.tree.Table.columns").value(std::string("90,210"));
  mb.grid_columns_ret = {120.0f, 0.0f}; // dragged column 1; column 2 not set
  rt.set_root(group({g}));
  rt.render();
  ASSERT_TRUE(mb.seeded_columns_called);
  ASSERT_EQ(mb.seeded_columns.size(), 2u);
  EXPECT_FLOAT_EQ(mb.seeded_columns[0], 90.0f);
  EXPECT_FLOAT_EQ(mb.seeded_columns[1], 210.0f);
  EXPECT_FALSE(mb.seeded_tracks_called) << "a table has no row-split tracks";
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.tree.Table.columns").value(), "120,0");
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.tree.Table.tracks").value(), "");
}

TEST(AriadneRuntime, OnlyAResizableTablePersistsColumns) {
  cvc::app app;
  Runtime rt(app, "ui.demo");
  MockBackend mb;
  rt.set_backend(&mb);
  Widget fixed = group({text("a"), text("b")}); // not resizable: nothing to drag
  fixed.id = "Fixed";
  fixed.layout.kind = LayoutKind::Grid;
  fixed.layout.col_widths = {Track{Unit::Px, 100.0f}, Track{Unit::Px, 100.0f}};
  Widget split = group({text("top"), text("bottom")}); // a row split: tracks, not columns
  split.id = "Split";
  split.layout.kind = LayoutKind::Grid;
  split.layout.resizable = true;
  split.layout.row_heights = {Track{Unit::Px, 80.0f}, Track{Unit::Px, 80.0f}};
  cvc::state::instance(app)("ui.demo.tree.Fixed.columns").value(std::string("1,2"));
  cvc::state::instance(app)("ui.demo.tree.Split.columns").value(std::string("3,4"));
  mb.grid_columns_ret = {5.0f, 6.0f}; // even if the backend reported widths, the core ignores them
  rt.set_root(group({fixed, split}));
  rt.render();
  EXPECT_FALSE(mb.seeded_columns_called);
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.tree.Fixed.columns").value(), "1,2");
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.tree.Split.columns").value(), "3,4");
}

TEST(AriadneRuntime, WindowWithNoGeometryNeitherSeedsNorWrites) {
  cvc::app app;
  Runtime rt(app, "ui.demo");
  MockBackend mb;
  rt.set_backend(&mb);
  // Nothing persisted, and a backend that reports no geometry (the FTXUI/terminal default {}): the
  // core must neither seed nor write a spurious "0,0,0,0" (exercises the has_pos||has_size guard).
  mb.window_geom_ret = WindowGeom{}; // has_pos == has_size == false
  rt.set_root(group({window("W", {text("x")})}));
  rt.render();
  EXPECT_FALSE(mb.seeded_geom_called);
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.tree.W.geometry").value(), ""); // no spurious write
}

TEST(AriadneRuntime, WindowGeometryIsKeyedPerId) {
  cvc::app app;
  Runtime rt(app, "ui.demo");
  // Two windows with distinct ids must each persist to their own tree.<id>.geometry node — the
  // §11.5.3-rule-5 id-keying the roadmap edit claims. One backend can only report one geometry per
  // frame, so use two runtimes/backends (one per window) to give each a distinct reported geometry.
  MockBackend mb_a;
  rt.set_backend(&mb_a);
  mb_a.window_geom_ret = WindowGeom{true, true, 1, 2, 100, 100};
  rt.set_root(group({window("A", {text("a")})}));
  rt.render();
  Runtime rt2(app, "ui.demo");
  MockBackend mb_b;
  rt2.set_backend(&mb_b);
  mb_b.window_geom_ret = WindowGeom{true, true, 9, 8, 700, 600};
  rt2.set_root(group({window("B", {text("b")})}));
  rt2.render();
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.tree.A.geometry").value(), "1,2,100,100");
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.tree.B.geometry").value(), "9,8,700,600");
}

TEST(AriadneRuntime, PositionOnlyPersistedGeometrySeedsPositionNotZeroSize) {
  cvc::app app;
  Runtime rt(app, "ui.demo");
  MockBackend mb;
  rt.set_backend(&mb);
  // A "x,y,0,0" sentinel (a window persisted while collapsed-from-birth) must seed position only —
  // parse_geom sets has_size=false when w/h are non-positive, so no 0x0 size is ever forced.
  cvc::state::instance(app)("ui.demo.tree.W.geometry").value(std::string("10,20,0,0"));
  mb.window_geom_ret = WindowGeom{true, false, 10, 20, 0, 0};
  rt.set_root(group({window("W", {text("x")})}));
  rt.render();
  ASSERT_TRUE(mb.seeded_geom_called);
  EXPECT_TRUE(mb.seeded_geom.has_pos);
  EXPECT_FALSE(mb.seeded_geom.has_size);
}

// ===========================================================================
// §12 channel enforcement — the RUNTIME policy (PR-D), the backstop for dynamic channel names
// ===========================================================================
// set_channel_policy installs the doc's declared/global channels + strict flag. In strict mode a
// program msg-* on an undeclared channel is refused: enforce_channel_policy throws BEFORE
// deliver_to_receivers, so the message is never queued. That delivery outcome
// (pending_message_count on the app scheduler) is the reliable signal — a refused send delivers
// nothing, an allowed send queues one. warn/off pass strict=false (no runtime enforcement — the
// load-time lint handled those). Prefix "" so resolve_channel is identity: a plain name keys to
// itself, "/g" keys to "g".

// Run a one-shot `(msg-send "<ch>" "x")` action under the current policy; return whether it was
// delivered (queued under `key` on the app scheduler). false => the send was refused.
static bool msg_delivered(cvc::app &app, Runtime &rt, MockBackend &mb, const std::string &ch,
                          const std::string &key) {
  rt.set_root(group({button("Go", "(msg-send \"" + ch + "\" \"x\")")}));
  mb.button_click = true;
  rt.render();
  rt.drain();
  return app.exec_scheduler().pending_message_count(key) > 0;
}

TEST(AriadneChannelPolicy, StrictRefusesUndeclaredChannel) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_channel_policy({"nav.done"}, {}, /*strict=*/true, /*quiet=*/false);
  EXPECT_FALSE(
      msg_delivered(app, rt, mb, "other", "other")); // undeclared -> refused, not delivered
}

TEST(AriadneChannelPolicy, AllowsDeclaredChannel) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_channel_policy({"nav.done"}, {}, true, false);
  EXPECT_TRUE(msg_delivered(app, rt, mb, "nav.done", "nav.done")); // declared -> allowed, delivered
}

TEST(AriadneChannelPolicy, PermissiveWhenNotStrict) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_channel_policy({}, {}, /*strict=*/false, false); // warn/off -> no runtime enforcement
  EXPECT_TRUE(msg_delivered(app, rt, mb, "anything", "anything"));
}

TEST(AriadneChannelPolicy, AllowsDeclaredGlobal) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_channel_policy({}, {"app.quit"}, true, false);
  EXPECT_TRUE(
      msg_delivered(app, rt, mb, "/app.quit", "app.quit")); // declared global (key strips /)
}

TEST(AriadneChannelPolicy, RefusesUndeclaredGlobal) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_channel_policy({}, {"app.quit"}, true, false);
  EXPECT_FALSE(
      msg_delivered(app, rt, mb, "/other.g", "other.g")); // not a declared global -> refused
}

TEST(AriadneChannelPolicy, ExemptsHashChannel) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_channel_policy({"nav.done"}, {}, true, false);
  EXPECT_TRUE(msg_delivered(app, rt, mb, "sys#evt", "sys#evt")); // '#'-runtime channel is exempt
}

// ---------------------------------------------------------------------------
// §13.8 async (http-get-async) intrinsic — offline via the cvc::net fake transport.
// ---------------------------------------------------------------------------
namespace {

// A canned transport (PR1's set_http_client seam): returns one fixed response, records the call.
class CannedHttpClient : public cvc::net::HttpClient {
public:
  explicit CannedHttpClient(cvc::net::HttpResponse r) : resp_(std::move(r)) {}
  std::atomic<int> calls{0};
  std::string last_url;
  std::string last_method;
  std::string last_body;
  std::vector<std::string> last_headers;
  cvc::net::HttpResponse send(const cvc::net::HttpRequest &req) override {
    last_url = req.url;
    last_method = req.method;
    last_body = req.body;
    last_headers = req.headers;
    ++calls;
    return resp_;
  }

private:
  cvc::net::HttpResponse resp_;
};

// A transport that blocks in send() until released — proves the verb does NOT block the scheduler.
class BlockingHttpClient : public cvc::net::HttpClient {
public:
  explicit BlockingHttpClient(cvc::net::HttpResponse r) : resp_(std::move(r)) {}
  cvc::net::HttpResponse send(const cvc::net::HttpRequest &) override {
    std::unique_lock<std::mutex> lk(m_);
    cv_.wait(lk, [&] { return release_; });
    return resp_;
  }
  void release() {
    {
      std::lock_guard<std::mutex> lk(m_);
      release_ = true;
    }
    cv_.notify_all();
  }

private:
  std::mutex m_;
  std::condition_variable cv_;
  bool release_ = false;
  cvc::net::HttpResponse resp_;
};

// Restore process-global state so one test never leaks into another.
struct NetIntrinsicsGuard {
  ~NetIntrinsicsGuard() {
    clear_action_intrinsics();
    cvc::net::set_http_client(nullptr);
  }
};

cvc::net::HttpResponse http_ok(long status, std::string body, std::vector<std::string> headers = {},
                               std::string url = "http://ex/x") {
  cvc::net::HttpResponse r;
  r.ok = true;
  r.status = status;
  r.body = std::move(body);
  r.headers = std::move(headers);
  r.canonical_url = std::move(url);
  return r;
}

// Pump render+drain until `pred` holds or the budget elapses (compute_async posts from a background
// worker, so a bounded pump stands in for the nav test's hand-rolled join()).
template <class Pred> bool pump_until(Runtime &rt, Pred pred, int budget_ms = 5000) {
  using namespace std::chrono;
  const auto deadline = steady_clock::now() + milliseconds(budget_ms);
  while (steady_clock::now() < deadline) {
    rt.render();
    rt.drain();
    if (pred())
      return true;
    std::this_thread::sleep_for(milliseconds(2));
  }
  return pred();
}

// The nav_compute demo's verb: (nav-step) is an async function on the app's async host. The action
// parks at the call while the kernel runs on the background task pool and fans out over the compute
// pool, then resumes with the call's value. No channel, no msg-recv.
TEST(AriadneAction, NavStepAsAnAsyncFunctionReturnsItsResult) {
  namespace se = cvc::state_exec;
  cvc::app app;
  app.computePool();
  app.exec_scheduler();
  app.exec_async_host(); // warm them on THIS thread
  register_action_intrinsics(
      [&app](std::shared_ptr<se::environment> env, se::intrinsics_context &) {
        se::register_async_fn(env, app.exec_async_host(), "nav-step",
                              [&app](std::span<const se::value_t>) -> se::async_kernel {
                                return [&app](se::async_call &) -> se::value_t {
                                  std::atomic<int> arrived{0};
                                  app.computePool().parallel_for(256, [&arrived](int i) {
                                    if ((i % 3) == 0)
                                      arrived.fetch_add(1, std::memory_order_relaxed);
                                  });
                                  return se::value_t(std::to_string(arrived.load()));
                                };
                              });
      });
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_root(group({button("Go", "(state-set \"nav.result\" (nav-step))")}));
  mb.button_click = true;
  rt.render();
  rt.drain();
  mb.button_click = false;
  const bool resumed =
      pump_until(rt, [&] { return !cvc::state::instance(app)("nav.result").value().empty(); });
  clear_action_intrinsics();
  ASSERT_TRUE(resumed) << "the parked action never resumed";
  EXPECT_EQ(cvc::state::instance(app)("nav.result").value(), "86");
  EXPECT_TRUE(rt.take_reactive_warnings().empty());
}

std::string node_data_string(cvc::app &app, const char *path) {
  const boost::any d = cvc::state::instance(app)(path).data();
  const std::string *s = boost::any_cast<std::string>(&d);
  return s ? *s : std::string();
}

} // namespace

TEST(AriadneNetIntrinsics, HttpGetAsyncAwaitsResponseDict) {
  if (!cvc::net::have_http_backend())
    GTEST_SKIP() << "libcvc built without an HTTP backend";
  cvc::app app;
  NetIntrinsicsGuard guard;
  auto *fake = new CannedHttpClient(http_ok(200, "hi", {"Content-Type: text/plain"}));
  cvc::net::set_http_client(std::unique_ptr<cvc::net::HttpClient>(fake));
  register_net_intrinsics(app);

  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_root(group({button("Go", "(begin"
                                  "  (set r (msg-recv (http-get-async \"http://ex/x\")))"
                                  "  (state-set \"r.status\" (get-attr r \"status\"))"
                                  "  (state-set \"r.ok\" (get-attr r \"ok\"))"
                                  "  (state-set \"r.error\" (get-attr r \"error\"))"
                                  "  (state-data-set \"r.body\" (get-attr r \"body\")))")}));
  mb.button_click = true;
  rt.render();
  rt.drain(); // http-get-async launches the worker; the action parks on msg-recv (drain never
              // blocks)
  mb.button_click = false;

  ASSERT_TRUE(pump_until(rt, [&] {
    return !cvc::state::instance(app)("r.status").value().empty();
  })) << "the parked action never resumed";
  EXPECT_EQ(cvc::state::instance(app)("r.status").value(), "200");
  EXPECT_EQ(cvc::state::instance(app)("r.ok").value(), "true");
  EXPECT_EQ(cvc::state::instance(app)("r.error").value(), ""); // empty on success
  EXPECT_EQ(node_data_string(app, "r.body"), "hi"); // the bytes body round-trips byte-exact
  EXPECT_EQ(fake->calls.load(), 1);
  EXPECT_EQ(fake->last_url, "http://ex/x");
}

TEST(AriadneNetIntrinsics, HttpGetAsyncErrorPathResumesWithErrorDict) {
  if (!cvc::net::have_http_backend())
    GTEST_SKIP() << "libcvc built without an HTTP backend";
  cvc::app app;
  NetIntrinsicsGuard guard;
  cvc::net::HttpResponse err;
  err.ok = false;
  err.status = 0;
  err.error = "boom";
  cvc::net::set_http_client(std::make_unique<CannedHttpClient>(err));
  register_net_intrinsics(app);

  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_root(group({button("Go", "(begin"
                                  "  (set r (msg-recv (http-get-async \"http://ex/down\")))"
                                  "  (state-set \"r.ok\" (get-attr r \"ok\"))"
                                  "  (state-set \"r.error\" (get-attr r \"error\")))")}));
  mb.button_click = true;
  rt.render();
  rt.drain();
  mb.button_click = false;

  ASSERT_TRUE(pump_until(rt, [&] { return !cvc::state::instance(app)("r.error").value().empty(); }))
      << "the action must resume even on a transport error";
  EXPECT_EQ(cvc::state::instance(app)("r.ok").value(), "false");
  EXPECT_EQ(cvc::state::instance(app)("r.error").value(), "boom");
}

TEST(AriadneNetIntrinsics, HttpGetAsyncDoesNotBlockTheScheduler) {
  if (!cvc::net::have_http_backend())
    GTEST_SKIP() << "libcvc built without an HTTP backend";
  cvc::app app;
  NetIntrinsicsGuard guard;
  auto *fake = new BlockingHttpClient(http_ok(200, "later"));
  cvc::net::set_http_client(std::unique_ptr<cvc::net::HttpClient>(fake));
  register_net_intrinsics(app);

  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_root(group(
      {button("Go", "(state-set \"r.status\" "
                    "(get-attr (msg-recv (http-get-async \"http://ex/slow\")) \"status\"))")}));
  mb.button_click = true;
  rt.render();
  rt.drain(); // the verb returns immediately + the action parks; the fetch is still blocked in
              // send()
  mb.button_click = false;
  // drain() returned though the transport is blocked → the scheduler thread was not blocked.
  EXPECT_TRUE(cvc::state::instance(app)("r.status").value().empty())
      << "the result must not be delivered while the fetch is in flight";

  fake->release(); // let the worker's send() complete
  ASSERT_TRUE(
      pump_until(rt, [&] { return !cvc::state::instance(app)("r.status").value().empty(); }));
  EXPECT_EQ(cvc::state::instance(app)("r.status").value(), "200");
}

// PR-A: the TRANSPARENT verb — (http-get url) self-parks and yields the dict directly, no msg-recv.
TEST(AriadneNetIntrinsics, HttpGetTransparentReturnsDictNoMsgRecv) {
  if (!cvc::net::have_http_backend())
    GTEST_SKIP() << "libcvc built without an HTTP backend";
  cvc::app app;
  NetIntrinsicsGuard guard;
  auto *fake = new CannedHttpClient(http_ok(200, "hi", {"Content-Type: text/plain"}));
  cvc::net::set_http_client(std::unique_ptr<cvc::net::HttpClient>(fake));
  register_net_intrinsics(app);

  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_root(group({button("Go", "(begin"
                                  "  (set r (http-get \"http://ex/x\"))"
                                  "  (state-set \"r.status\" (get-attr r \"status\"))"
                                  "  (state-data-set \"r.body\" (get-attr r \"body\")))")}));
  mb.button_click = true;
  rt.render();
  rt.drain(); // (http-get …) self-parks the action; drain never blocks
  mb.button_click = false;

  ASSERT_TRUE(pump_until(rt, [&] {
    return !cvc::state::instance(app)("r.status").value().empty();
  })) << "the self-parked (http-get) never resumed";
  EXPECT_EQ(cvc::state::instance(app)("r.status").value(), "200");
  EXPECT_EQ(node_data_string(app, "r.body"), "hi"); // the dict flowed into `set r` transparently
  EXPECT_EQ(fake->calls.load(), 1);                 // one transparent fetch, one transfer
}

// PR-A: PROPER await — (await (http-get-async url)) resolves the future returned by the async verb.
TEST(AriadneNetIntrinsics, AwaitResolvesHttpGetAsyncFuture) {
  if (!cvc::net::have_http_backend())
    GTEST_SKIP() << "libcvc built without an HTTP backend";
  cvc::app app;
  NetIntrinsicsGuard guard;
  auto *fake = new CannedHttpClient(http_ok(200, "hi"));
  cvc::net::set_http_client(std::unique_ptr<cvc::net::HttpClient>(fake));
  register_net_intrinsics(app);

  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_root(
      group({button("Go", "(state-set \"r.status\" "
                          "(get-attr (await (http-get-async \"http://ex/x\")) \"status\"))")}));
  mb.button_click = true;
  rt.render();
  rt.drain();
  mb.button_click = false;

  ASSERT_TRUE(pump_until(rt, [&] {
    return !cvc::state::instance(app)("r.status").value().empty();
  })) << "(await <future>) never resolved";
  EXPECT_EQ(cvc::state::instance(app)("r.status").value(), "200");
  EXPECT_EQ(fake->calls.load(), 1);
}

// PR-B: the GENERIC async resolver — (fetch uri) resolves ANY registered scheme off-thread and
// self-parks, returning { ok body(bytes) url error }. No HTTP backend needed (a custom scheme
// here).
TEST(AriadneNetIntrinsics, FetchTransparentResolvesAnyScheme) {
  cvc::app app;
  struct Teardown {
    ~Teardown() {
      clear_action_intrinsics();
      unregister_uri_handler("mem");
    }
  } td;
  // A canned custom scheme; the handler runs on the compute-pool worker, so it must be thread-safe
  // (a pure lambda returning a fixed UriResult is).
  register_uri_handler("mem", [](const Uri &u, const std::string &) {
    return UriResult{true, "hello", u.raw, std::string()};
  });
  register_net_intrinsics(app);

  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_root(group({button("Go", "(begin"
                                  "  (set r (fetch \"mem://x\"))"
                                  "  (state-set \"r.ok\" (get-attr r \"ok\"))"
                                  "  (state-data-set \"r.body\" (get-attr r \"body\")))")}));
  mb.button_click = true;
  rt.render();
  rt.drain();
  mb.button_click = false;

  ASSERT_TRUE(pump_until(rt, [&] { return !cvc::state::instance(app)("r.ok").value().empty(); }))
      << "(fetch …) never resolved";
  EXPECT_EQ(cvc::state::instance(app)("r.ok").value(), "true");
  EXPECT_EQ(node_data_string(app, "r.body"), "hello"); // resolved bytes threaded in transparently
}

// PR-B: (await (fetch-async uri)) resolves the generic resolver's future.
TEST(AriadneNetIntrinsics, FetchAsyncFutureAwaited) {
  cvc::app app;
  struct Teardown {
    ~Teardown() {
      clear_action_intrinsics();
      unregister_uri_handler("mem");
    }
  } td;
  register_uri_handler("mem", [](const Uri &u, const std::string &) {
    return UriResult{true, "world", u.raw, std::string()};
  });
  register_net_intrinsics(app);

  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_root(group({button("Go", "(state-data-set \"r.body\" "
                                  "(get-attr (await (fetch-async \"mem://y\")) \"body\"))")}));
  mb.button_click = true;
  rt.render();
  rt.drain();
  mb.button_click = false;

  ASSERT_TRUE(pump_until(rt, [&] { return !node_data_string(app, "r.body").empty(); }))
      << "(await (fetch-async …)) never resolved";
  EXPECT_EQ(node_data_string(app, "r.body"), "world");
}

namespace {
bool headers_contain(const std::vector<std::string> &h, const char *needle) {
  return std::find(h.begin(), h.end(), needle) != h.end();
}
bool headers_have_prefix(const std::vector<std::string> &h, const std::string &prefix) {
  return std::any_of(h.begin(), h.end(),
                     [&](const std::string &line) { return line.rfind(prefix, 0) == 0; });
}
} // namespace

// The general method verb via the options dict: (http-request "POST" URL (dict "headers" (dict …)
// "form" (dict …))) — a form body is percent-encoded with a default Content-Type, and a bearer
// token rides in as a header. This is the authenticated form-POST the user asked for.
TEST(AriadneNetIntrinsics, HttpRequestPostFormBodyAndBearer) {
  if (!cvc::net::have_http_backend())
    GTEST_SKIP() << "libcvc built without an HTTP backend";
  cvc::app app;
  NetIntrinsicsGuard guard;
  auto *fake = new CannedHttpClient(http_ok(201, "created"));
  cvc::net::set_http_client(std::unique_ptr<cvc::net::HttpClient>(fake));
  register_net_intrinsics(app);

  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_root(
      group({button("Go", "(begin"
                          "  (set r (http-request \"POST\" \"http://ex/api\""
                          "           (dict \"headers\" (dict \"Authorization\" \"Bearer tok-123\")"
                          "                 \"form\"    (dict \"grip\" 1 \"risk\" 2))))"
                          "  (state-set \"r.status\" (get-attr r \"status\")))")}));
  mb.button_click = true;
  rt.render();
  rt.drain();
  mb.button_click = false;

  ASSERT_TRUE(pump_until(rt, [&] {
    return !cvc::state::instance(app)("r.status").value().empty();
  })) << "(http-request …) never resumed";
  EXPECT_EQ(cvc::state::instance(app)("r.status").value(), "201");
  EXPECT_EQ(fake->calls.load(), 1);
  EXPECT_EQ(fake->last_method, "POST");
  EXPECT_EQ(fake->last_url, "http://ex/api");
  EXPECT_EQ(fake->last_body,
            "grip=1&risk=2"); // form dict -> x-www-form-urlencoded, order preserved
  EXPECT_TRUE(headers_contain(fake->last_headers, "Authorization: Bearer tok-123"))
      << "bearer header not forwarded";
  EXPECT_TRUE(
      headers_contain(fake->last_headers, "Content-Type: application/x-www-form-urlencoded"))
      << "form body should default a Content-Type";
}

// (http-get URL (dict "query" (dict …))) percent-encodes the params into the URL's query string —
// the caller never hand-encodes. A space becomes %20 and '&' is escaped so it can't inject a pair.
TEST(AriadneNetIntrinsics, HttpGetQueryParamsEncodedIntoUrl) {
  if (!cvc::net::have_http_backend())
    GTEST_SKIP() << "libcvc built without an HTTP backend";
  cvc::app app;
  NetIntrinsicsGuard guard;
  auto *fake = new CannedHttpClient(http_ok(200, "ok"));
  cvc::net::set_http_client(std::unique_ptr<cvc::net::HttpClient>(fake));
  register_net_intrinsics(app);

  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_root(group(
      {button("Go", "(begin"
                    "  (set r (http-get \"http://ex/x\" (dict \"query\" (dict \"q\" \"hello world\""
                    "                                                         \"n\" 2))))"
                    "  (state-set \"r.status\" (get-attr r \"status\")))")}));
  mb.button_click = true;
  rt.render();
  rt.drain();
  mb.button_click = false;

  ASSERT_TRUE(pump_until(rt, [&] {
    return !cvc::state::instance(app)("r.status").value().empty();
  })) << "(http-get …) never resumed";
  EXPECT_EQ(fake->calls.load(), 1);
  EXPECT_EQ(fake->last_method, "GET");
  EXPECT_EQ(fake->last_url, "http://ex/x?q=hello%20world&n=2");
}

// A raw string "body" option is sent verbatim and does NOT get a defaulted Content-Type (only the
// "form" helper adds one) — the caller controls the content type.
TEST(AriadneNetIntrinsics, HttpRequestRawBodyNoDefaultContentType) {
  if (!cvc::net::have_http_backend())
    GTEST_SKIP() << "libcvc built without an HTTP backend";
  cvc::app app;
  NetIntrinsicsGuard guard;
  auto *fake = new CannedHttpClient(http_ok(200, "ok"));
  cvc::net::set_http_client(std::unique_ptr<cvc::net::HttpClient>(fake));
  register_net_intrinsics(app);

  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_root(group({button(
      "Go", "(begin"
            "  (set r (http-request \"PUT\" \"http://ex/raw\" (dict \"body\" \"raw-payload\")))"
            "  (state-set \"r.status\" (get-attr r \"status\")))")}));
  mb.button_click = true;
  rt.render();
  rt.drain();
  mb.button_click = false;

  ASSERT_TRUE(pump_until(rt, [&] {
    return !cvc::state::instance(app)("r.status").value().empty();
  })) << "(http-request …) never resumed";
  EXPECT_EQ(fake->last_method, "PUT");
  EXPECT_EQ(fake->last_body, "raw-payload");
  EXPECT_FALSE(headers_have_prefix(fake->last_headers, "Content-Type"))
      << "a raw body must not get a defaulted Content-Type";
}

// (http-request-async …) returns a future the program awaits; a BYTES body (here, the octet body a
// prior (http-get) yielded, carried via the "body" option) rides through byte-exact — so a fetched
// blob can be re-uploaded with a PUT without a lossy text round-trip.
TEST(AriadneNetIntrinsics, HttpRequestAsyncPutWithBytesBodyAwaited) {
  if (!cvc::net::have_http_backend())
    GTEST_SKIP() << "libcvc built without an HTTP backend";
  cvc::app app;
  NetIntrinsicsGuard guard;
  auto *fake = new CannedHttpClient(http_ok(200, "payload-bytes"));
  cvc::net::set_http_client(std::unique_ptr<cvc::net::HttpClient>(fake));
  register_net_intrinsics(app);

  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_root(group({button(
      "Go", "(begin"
            "  (set g (await (http-get-async \"http://ex/src\")))"
            "  (set r (await (http-request-async \"PUT\" \"http://ex/dst\""
            "                   (dict \"body\" (get-attr g \"body\")"
            "                         \"headers\" (dict \"Authorization\" \"Bearer tok\")))))"
            "  (state-set \"r.status\" (get-attr r \"status\")))")}));
  mb.button_click = true;
  rt.render();
  rt.drain();
  mb.button_click = false;

  ASSERT_TRUE(pump_until(rt, [&] {
    return !cvc::state::instance(app)("r.status").value().empty();
  })) << "(await (http-request-async …)) never resumed";
  EXPECT_EQ(cvc::state::instance(app)("r.status").value(), "200");
  EXPECT_EQ(fake->calls.load(), 2); // the GET, then the PUT
  EXPECT_EQ(fake->last_method, "PUT");
  EXPECT_EQ(fake->last_url, "http://ex/dst");
  EXPECT_EQ(fake->last_body, "payload-bytes"); // the bytes body threaded through byte-exact
  EXPECT_TRUE(headers_contain(fake->last_headers, "Authorization: Bearer tok"));
}

// launch_pool_task generalizes beyond HTTP: a NON-network "heavy" kernel offloaded to the compute
// pool and awaited by a DSL action — the exact shape a volume/geometry intrinsic follows. Registers
// a transparent (pool-double n) and a future (pool-double-async n) via the SAME action-intrinsics
// seam register_net_intrinsics uses, so both land in the action lane only (never the render lane).
TEST(AriadnePoolTask, OffloadedKernelResumesTransparentAndFuture) {
  namespace se = cvc::state_exec;
  cvc::app app;
  NetIntrinsicsGuard guard; // clears the action intrinsics on teardown
  app.computePool();        // warm the lazy singletons on THIS thread before a worker touches them
  app.exec_scheduler();
  register_action_intrinsics([&app](std::shared_ptr<se::environment> env,
                                    se::intrinsics_context &ictx) {
    const std::string root = ictx.root_path;
    auto arg_int = [](std::span<const se::value_t> args) -> std::int64_t {
      if (!args.empty())
        if (const std::int64_t *v = std::get_if<std::int64_t>(&args[0].v))
          return *v;
      return 0;
    };
    // A pure C++ kernel: doubles its input. Runs on a pool worker; no evaluator/state re-entry.
    auto make_work = [](std::int64_t n) {
      return [n]() { return se::value_t(static_cast<std::int64_t>(n * 2)); };
    };
    auto on_err = [](const std::string &e) { return se::value_t(std::string("err: ") + e); };
    se::builtins::register_fn(
        env, "pool-double",
        [&app, root, arg_int, make_work, on_err](std::span<const se::value_t> args) -> se::value_t {
          return cvc::park_on_pool_task(app, app.exec_scheduler(), root, "test.reply",
                                        make_work(arg_int(args)), on_err);
        });
    se::builtins::register_fn(
        env, "pool-double-async",
        [&app, root, arg_int, make_work, on_err](std::span<const se::value_t> args) -> se::value_t {
          return cvc::future_pool_task(app, app.exec_scheduler(), root, "test.reply",
                                       make_work(arg_int(args)), on_err);
        });
  });

  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_root(group({button("Go", "(begin"
                                  "  (state-set \"r.t\" (str (pool-double 21)))"
                                  "  (state-set \"r.a\" (str (await (pool-double-async 10)))))")}));
  mb.button_click = true;
  rt.render();
  rt.drain();
  mb.button_click = false;

  ASSERT_TRUE(pump_until(rt, [&] { return !cvc::state::instance(app)("r.a").value().empty(); }))
      << "the offloaded pool task never resumed the action";
  EXPECT_EQ(cvc::state::instance(app)("r.t").value(), "42"); // transparent self-park
  EXPECT_EQ(cvc::state::instance(app)("r.a").value(), "20"); // awaited future
}

// An async function on the app's exec async host (docs/STATE_EXEC_EXCEPTIONS.md 3), registered
// through the action seam: an action's call parks and resumes, its kernel's error raises at the
// call, and (AS-17) the reactive lane can't reach it: a predicate naming it is denied, hidden and
// reported, never parked or blocked.
TEST(AriadneAsyncFn, ActionCallsResumeAndTheReactiveLaneCantReachThem) {
  namespace se = cvc::state_exec;
  cvc::app app;
  NetIntrinsicsGuard guard;
  app.exec_async_host(); // warm it on THIS thread
  register_action_intrinsics(
      [&app](std::shared_ptr<se::environment> env, se::intrinsics_context &) {
        se::register_async_fn(env, app.exec_async_host(), "pool-triple",
                              [](std::span<const se::value_t> args) -> se::async_kernel {
                                const std::int64_t n =
                                    args.empty() ? 0 : std::get<std::int64_t>(args[0].v);
                                return [n](se::async_call &) -> se::value_t {
                                  if (n < 0)
                                    throw std::invalid_argument("negative");
                                  return se::value_t(n * 3);
                                };
                              });
      });

  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  Widget predicate = text("never");
  predicate.visible_when = "(pool-triple 1)";
  rt.set_root(group(
      {button("Go", "(state-set \"r.t\" (str (pool-triple 14)))"),
       button("Bad", "(state-set \"r.e\" (try (pool-triple -1) (catch (ValueError e) (get-attr e "
                     "\"origin\"))))"),
       predicate}));
  mb.button_click = true;
  rt.render();
  rt.drain();
  mb.button_click = false;

  ASSERT_TRUE(pump_until(rt, [&] {
    return !cvc::state::instance(app)("r.t").value().empty() &&
           !cvc::state::instance(app)("r.e").value().empty();
  })) << "an async call never resumed its action";
  EXPECT_EQ(cvc::state::instance(app)("r.t").value(), "42");
  EXPECT_EQ(cvc::state::instance(app)("r.e").value(), "async"); // raised at the call, caught
  EXPECT_FALSE(mb.saw("text_line:never"));                      // AS-17: denied, so hidden
  EXPECT_FALSE(rt.take_reactive_warnings().empty());            // and reported
}

// §7.4 an action's step and time caps are tree settings (ariadne.limits.action_max_steps, default
// 5000; ariadne.limits.action_max_time, default 0.1 s), read on every submission. The action's
// async calls are charged to them (docs/STATE_EXEC_EXCEPTIONS.md 3.8), so a CPU-heavy verb kills an
// action at the default caps, and a host whose documents need it raises them.
TEST(AriadneAsyncFn, ActionCapsAreTreeSettingsAndChargeAsyncCalls) {
  namespace se = cvc::state_exec;
  cvc::app app;
  app.exec_async_host();
  // About 20 ms of CPU: some 40,000 steps at the default rate, far over the default 5000.
  register_action_intrinsics(
      [&app](std::shared_ptr<se::environment> env, se::intrinsics_context &) {
        se::register_async_fn(env, app.exec_async_host(), "spin",
                              [](std::span<const se::value_t>) -> se::async_kernel {
                                return [](se::async_call &call) -> se::value_t {
                                  const double until = se::thread_cpu_seconds() + 0.02;
                                  volatile std::uint64_t x = 0;
                                  while (se::thread_cpu_seconds() < until && !call.cancelled())
                                    for (int i = 0; i < 1000; ++i)
                                      x = x + static_cast<std::uint64_t>(i);
                                  return se::value_t(std::string("spun"));
                                };
                              });
      });
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  auto &root = cvc::state::instance(app);
  rt.set_root(group({button("Loop", "(begin (set i 0) (while (< i 200) (set i (+ i 1))) "
                                    "(state-set \"r.loop\" (str i)))"),
                     button("Spin", "(state-set \"r.spin\" (spin))")}));
  const auto click = [&](const char *key) {
    root(key).value(std::string("-"));
    mb.button_click = true;
    rt.render();
    rt.drain();
    mb.button_click = false;
    pump_until(rt, [&] { return root(key).value() != "-"; }, 1000);
    for (int i = 0; i < 3; ++i) { // let the sweep report a kill
      rt.render();
      rt.drain();
    }
    return root(key).value();
  };

  // The defaults: the loop fits in 5000 steps; the spin's CPU doesn't.
  EXPECT_EQ(click("r.loop"), "200");
  EXPECT_EQ(click("r.spin"), "-");
  auto warns = rt.take_reactive_warnings();
  ASSERT_FALSE(warns.empty());
  EXPECT_NE(warns.back().find("max_steps_exceeded"), std::string::npos) << warns.back();

  // A step cap below the loop's needs kills it too.
  root("ariadne.limits.action_max_steps").value(std::string("100"));
  EXPECT_EQ(click("r.loop"), "-");
  // No limit on either: the spin finishes.
  root("ariadne.limits.action_max_steps").value(std::string("0"));
  root("ariadne.limits.action_max_time").value(std::string("0"));
  EXPECT_EQ(click("r.spin"), "spun");
  // Unparsable values keep the defaults.
  root("ariadne.limits.action_max_steps").value(std::string("lots"));
  root("ariadne.limits.action_max_time").value(std::string("soon"));
  EXPECT_EQ(click("r.loop"), "200");
  EXPECT_EQ(click("r.spin"), "-");
  clear_action_intrinsics();
}

// A `closable: true` window renders with a close button, persists tree.<id>.open, is skipped once
// closed, and re-renders when the key is set back (what a Settings menu_toggle does).
TEST(AriadneRuntime, ClosableWindowClosesAndReopens) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  Widget w = window("Panel", {text("hi")});
  w.id = "panel";
  w.closable = true;
  rt.set_root(group({w}));
  const auto has = [&](const std::string &s) {
    return std::find(mb.log.begin(), mb.log.end(), s) != mb.log.end();
  };

  mb.log.clear();
  rt.render(); // open: rendered with a close button; the open state seeds to 1
  EXPECT_TRUE(has("begin_window:Panel(closable)"));
  EXPECT_EQ(cvc::state::instance(app)("tree.panel.open").value(), "1");

  mb.close_windows = true;
  mb.log.clear();
  rt.render(); // the X is clicked -> the core persists open = 0
  mb.close_windows = false;
  EXPECT_EQ(cvc::state::instance(app)("tree.panel.open").value(), "0");

  mb.log.clear();
  rt.render(); // closed -> not rendered at all
  EXPECT_FALSE(has("begin_window:Panel(closable)"));

  cvc::state::instance(app)("tree.panel.open").value(std::string("1")); // the Settings toggle
  mb.log.clear();
  rt.render();
  EXPECT_TRUE(has("begin_window:Panel(closable)"));
}

// ── §5.9 the built-in Windows menu ──────────────────────────────────────────────────────────────
namespace {
std::ptrdiff_t log_index(const MockBackend &mb, const std::string &s) {
  const auto it = std::find(mb.log.begin(), mb.log.end(), s);
  return it == mb.log.end() ? -1 : it - mb.log.begin();
}
Widget window_with_id(const std::string &title, const std::string &id, bool closable) {
  Widget w = window(title, {text("body")});
  w.id = id;
  w.closable = closable;
  return w;
}
} // namespace

TEST(AriadneRuntime, WindowsMenuReopensAClosedWindow) {
  cvc::app app;
  Runtime rt(app, "ui.demo");
  MockBackend mb;
  rt.set_backend(&mb);
  // The document's own menu, a closable window and one without a close button.
  rt.set_root(group({menubar({menu("File", {menu_action("Quit", "quit")})}),
                     window_with_id("Alpha", "a", true), window_with_id("Beta", "b", false)}));
  rt.render();
  // The Windows menu comes after the document's menus, with a ticked item per window.
  ASSERT_GE(log_index(mb, "begin_menu:Windows"), 0);
  EXPECT_LT(log_index(mb, "begin_menu:File"), log_index(mb, "begin_menu:Windows"));
  EXPECT_TRUE(mb.toggle_shown["Alpha"]);
  EXPECT_TRUE(mb.toggle_shown["Beta"]);

  // Alpha's X closes it; its item loses the tick and the window is not drawn.
  mb.close_windows = true;
  rt.render();
  mb.close_windows = false;
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.tree.a.open").value(), "0");
  mb.log.clear();
  rt.render();
  EXPECT_FALSE(mb.saw("begin_window:Alpha(closable)"));
  EXPECT_FALSE(mb.toggle_shown["Alpha"]);

  // Clicking the item opens it again.
  mb.click_toggle = "Alpha";
  rt.render();
  mb.click_toggle.clear();
  EXPECT_EQ(cvc::state::instance(app)("ui.demo.tree.a.open").value(), "1");
  mb.log.clear();
  rt.render();
  EXPECT_TRUE(mb.saw("begin_window:Alpha(closable)"));

  // A window with no close button can be hidden and shown from the menu too.
  mb.click_toggle = "Beta";
  rt.render();
  mb.click_toggle.clear();
  mb.log.clear();
  rt.render();
  EXPECT_FALSE(mb.saw("begin_window:Beta"));
  mb.click_toggle = "Beta";
  rt.render();
  mb.click_toggle.clear();
  mb.log.clear();
  rt.render();
  EXPECT_TRUE(mb.saw("begin_window:Beta"));
}

TEST(AriadneRuntime, WindowsMenuLeavesOutWindowsTheDocumentAlreadyToggles) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  // A Settings menu of the document already opens Alpha (lsystem_forest's pattern).
  rt.set_root(group({menubar({menu("Settings", {menu_toggle("Alpha", "tree.a.open", true)})}),
                     window_with_id("Alpha", "a", true), window_with_id("Beta", "b", true)}));
  rt.render();
  ASSERT_TRUE(mb.saw("begin_menu:Windows"));
  EXPECT_EQ(std::count(mb.log.begin(), mb.log.end(), "toggle:Alpha"), 1) << "not listed twice";
  EXPECT_TRUE(mb.saw("toggle:Beta"));

  // Every window already covered: no Windows menu at all.
  MockBackend mb2;
  Runtime rt2(app, "");
  rt2.set_backend(&mb2);
  rt2.set_root(group({menubar({menu("Settings", {menu_toggle("Alpha", "tree.a.open", true)})}),
                      window_with_id("Alpha", "a", true)}));
  rt2.render();
  EXPECT_FALSE(mb2.saw("begin_menu:Windows"));
}

TEST(AriadneRuntime, WindowsMenuJoinsTheDocumentsOwnMenuOfThatName) {
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  // The document has a "Windows" menu of its own for some of its windows (fem_gallery's panels):
  // the rest join it after a separator, rather than a second "Windows" menu in the bar.
  rt.set_root(group({menubar({menu("File", {menu_action("Quit", "quit")}),
                              menu("Windows", {menu_toggle("Alpha", "tree.a.open", true)})}),
                     window_with_id("Alpha", "a", true), window_with_id("Beta", "b", true)}));
  rt.render();
  EXPECT_EQ(std::count(mb.log.begin(), mb.log.end(), "begin_menu:Windows"), 1);
  EXPECT_EQ(std::count(mb.log.begin(), mb.log.end(), "toggle:Alpha"), 1);
  const std::ptrdiff_t alpha = log_index(mb, "toggle:Alpha"), sep = log_index(mb, "separator"),
                       beta = log_index(mb, "toggle:Beta");
  ASSERT_GE(beta, 0);
  EXPECT_LT(log_index(mb, "begin_menu:Windows"), alpha);
  EXPECT_LT(alpha, sep);
  EXPECT_LT(sep, beta);

  // Its items still open a closed window.
  cvc::state::instance(app)("tree.b.open").value(std::string("0"));
  mb.click_toggle = "Beta";
  rt.render();
  mb.click_toggle.clear();
  EXPECT_EQ(cvc::state::instance(app)("tree.b.open").value(), "1");

  // A submenu of that name is not the bar's: the Windows menu stands in the bar on its own.
  MockBackend mb2;
  Runtime rt2(app, "");
  rt2.set_backend(&mb2);
  rt2.set_root(group({menubar({menu("Document", {menu("Windows", {menu_action("Quit", "quit")})})}),
                      window_with_id("Beta", "b", true)}));
  rt2.render();
  EXPECT_EQ(std::count(mb2.log.begin(), mb2.log.end(), "begin_menu:Windows"), 2);
  EXPECT_FALSE(mb2.saw("separator"));
  EXPECT_LT(log_index(mb2, "menu_action:Quit"), log_index(mb2, "toggle:Beta"));
}

TEST(AriadneRuntime, WindowsMenuCanBePlacedOrTurnedOff) {
  cvc::app app;
  {
    // Placed inside the document's View menu, under its own title, and not appended again.
    MockBackend mb;
    Runtime rt(app, "");
    rt.set_backend(&mb);
    rt.set_root(group(
        {menubar({menu("View", {windows_menu("Panels")})}), window_with_id("Alpha", "a", true)}));
    rt.render();
    const std::ptrdiff_t view = log_index(mb, "begin_menu:View"),
                         panels = log_index(mb, "begin_menu:Panels");
    ASSERT_GE(view, 0);
    ASSERT_GE(panels, 0);
    EXPECT_LT(view, panels);
    EXPECT_FALSE(mb.saw("begin_menu:Windows"));
    EXPECT_TRUE(mb.saw("toggle:Alpha"));
  }
  {
    // windows_menu: false (a setting outside the menubar): no menu.
    MockBackend mb;
    Runtime rt(app, "");
    rt.set_backend(&mb);
    rt.set_root(group({menubar({menu("File", {menu_action("Quit", "quit")})}),
                       window_with_id("Alpha", "a", true), windows_menu("", false)}));
    rt.render();
    EXPECT_FALSE(mb.saw("begin_menu:Windows"));
    EXPECT_FALSE(mb.saw("toggle:Alpha"));
  }
}

TEST(AriadneRuntime, WindowsMenuWithoutADocumentMenubar) {
  cvc::app app;
  {
    // A closable window but no menubar: one is added for the Windows menu, or the closed window
    // could never come back.
    MockBackend mb;
    Runtime rt(app, "");
    rt.set_backend(&mb);
    rt.set_root(group({window_with_id("Alpha", "a", true)}));
    rt.render();
    EXPECT_TRUE(mb.saw("begin_menubar"));
    EXPECT_TRUE(mb.saw("begin_menu:Windows"));
    EXPECT_TRUE(mb.saw("toggle:Alpha"));
  }
  {
    // Nothing can be closed: no menubar is added.
    MockBackend mb;
    Runtime rt(app, "");
    rt.set_backend(&mb);
    Widget repeated = window_with_id("Cam {i}", "cam_{i}", true);
    repeated.repeat = "2"; // made per frame: not listed (the document opens these itself)
    rt.set_root(group({window_with_id("Alpha", "a", false), repeated}));
    rt.render();
    EXPECT_FALSE(mb.saw("begin_menubar"));
    EXPECT_FALSE(mb.saw("begin_menu:Windows"));
  }
}

// ── Phase-2 PR6: cvc::ariadne::stream DSL surface (stream-open/stream-info/stream-close) ──
namespace {
struct StreamActionGuard {
  ~StreamActionGuard() { clear_action_intrinsics(); } // process-global registry
};
} // namespace

TEST(AriadneStreamIntrinsics, OpenInfoCloseRoundTrip) {
  cvc::app app;
  StreamActionGuard guard;
  register_stream_intrinsics(app);

  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_root(
      group({button("Go", "(begin"
                          "  (set h (stream-open (dict \"id\" \"cam0\" \"w\" 4 \"h\" 2)))"
                          "  (state-set \"t.ok\" (get-attr h \"ok\"))"
                          "  (state-set \"t.token\" (get-attr h \"token\"))"
                          "  (state-set \"t.seq\" (get-attr h \"seq_channel\"))"
                          "  (state-set \"t.live\" (get-attr (stream-info h) \"live\"))"
                          "  (state-set \"t.closed\" (get-attr (stream-close h) \"ok\"))"
                          "  (state-set \"t.live2\" (get-attr (stream-info h) \"live\")))")}));
  mb.button_click = true;
  rt.render();
  rt.drain(); // the verbs are synchronous; the action completes in this drain
  mb.button_click = false;

  EXPECT_EQ(cvc::state::instance(app)("t.ok").value(), "true");
  EXPECT_EQ(cvc::state::instance(app)("t.token").value(), "streams.cam0"); // empty root -> identity
  EXPECT_EQ(cvc::state::instance(app)("t.seq").value(), "streams.cam0.seq");
  EXPECT_EQ(cvc::state::instance(app)("t.live").value(), "true");
  EXPECT_EQ(cvc::state::instance(app)("t.closed").value(), "true");
  EXPECT_EQ(cvc::state::instance(app)("t.live2").value(), "false"); // closed -> no longer owned
  EXPECT_EQ(cvc::state::instance(app)("streams.cam0").value(),
            "closed"); // descriptor marked closed
}

TEST(AriadneStreamIntrinsics, OpenIsScopedToTheCallingDocument) {
  cvc::app app;
  StreamActionGuard guard;
  register_stream_intrinsics(app);

  Runtime rt(app, "demo"); // the document's chroot
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_root(group({button("Go", "(begin"
                                  "  (set h (stream-open (dict \"id\" \"cam0\" \"w\" 4 \"h\" 2)))"
                                  "  (state-set \"t.token\" (get-attr h \"token\"))"
                                  "  (state-set \"t.seq\" (get-attr h \"seq_channel\")))")}));
  mb.button_click = true;
  rt.render();
  rt.drain();
  mb.button_click = false;

  // DSL state-set is chroot-relative to "demo"; the stream is scoped under it.
  EXPECT_EQ(cvc::state::instance(app)("demo.t.token").value(), "demo.streams.cam0");
  EXPECT_EQ(cvc::state::instance(app)("demo.t.seq").value(), "demo.channels.streams.cam0.seq");
  EXPECT_EQ(cvc::state::instance(app)("demo.streams.cam0").value(), "live"); // still open
}

// A stream the author opens but never (stream-close)s must be closed when its DOCUMENT tears down,
// while the app is still alive — the lifetime a process-static app*-keyed owning table could NOT
// give (under that model this case HUNG at ~Runtime, the stream's ~close running against a dead
// app at process exit). Here the Runtime owns the stream via its document_scope and clear()s it in
// ~Impl, so the descriptor flips to "closed" the moment the document goes away.
TEST(AriadneStreamIntrinsics, DocumentTeardownClosesAnUnclosedStream) {
  cvc::app app;
  StreamActionGuard guard;
  register_stream_intrinsics(app);

  {
    Runtime rt(app, "demo");
    MockBackend mb;
    rt.set_backend(&mb);
    rt.set_root(group({button("Go", "(begin"
                                    "  (set h (stream-open (dict \"id\" \"cam0\" \"w\" 4 \"h\" 2)))"
                                    "  (state-set \"t.ok\" (get-attr h \"ok\")))")}));
    mb.button_click = true;
    rt.render();
    rt.drain(); // opens the stream; the action never closes it
    mb.button_click = false;

    EXPECT_EQ(cvc::state::instance(app)("demo.t.ok").value(), "true");
    EXPECT_EQ(cvc::state::instance(app)("demo.streams.cam0").value(),
              "live"); // open, document live
  } // ~Runtime -> ~Impl clear()s the document_scope -> ~stream closes the still-open stream

  EXPECT_EQ(cvc::state::instance(app)("demo.streams.cam0").value(), "closed")
      << "a stream left open must be closed when its document tears down (app still alive)";
}

// Out-of-range sizing knobs must fail cleanly (an { ok #f error } dict), not wrap size_t into a
// wild frame_pool allocation or truncate a huge dimension through static_cast<int>.
TEST(AriadneStreamIntrinsics, OutOfRangeSizingFailsCleanly) {
  cvc::app app;
  StreamActionGuard guard;
  register_stream_intrinsics(app);

  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_root(group({button(
      "Go",
      "(begin"
      // negative expected_subscribers would cast to a ~1.8e19 size_t slab count
      "  (set a (stream-open (dict \"id\" \"c0\" \"w\" 4 \"h\" 2 \"expected_subscribers\" -1)))"
      "  (state-set \"t.a\" (get-attr a \"ok\"))"
      "  (set b (stream-open (dict \"id\" \"c1\" \"w\" 999999 \"h\" 2)))" // absurd dimension
      "  (state-set \"t.b\" (get-attr b \"ok\")))")}));
  mb.button_click = true;
  rt.render();
  rt.drain();
  mb.button_click = false;

  EXPECT_EQ(cvc::state::instance(app)("t.a").value(), "false"); // rejected, not an OOM
  EXPECT_EQ(cvc::state::instance(app)("t.b").value(), "false");
  EXPECT_EQ(cvc::state::instance(app)("streams.c0").value(), ""); // nothing opened
  EXPECT_EQ(cvc::state::instance(app)("streams.c1").value(), "");
}

// The load-time init: lane has no document scope to own a stream, so (stream-open) there fails
// cleanly (a stream with no owner is the leak model B exists to prevent) rather than leaking.
TEST(AriadneStreamIntrinsics, OpenOnTheInitLaneFailsWithoutADocumentScope) {
  cvc::app app;
  StreamActionGuard guard;
  register_stream_intrinsics(app);

  std::vector<std::string> errors;
  const bool ok = run_init(app, "demo",
                           "(begin"
                           "  (set h (stream-open (dict \"id\" \"cam0\" \"w\" 4 \"h\" 2)))"
                           "  (state-set \"t.ok\" (get-attr h \"ok\"))"
                           "  (state-set \"t.err\" (get-attr h \"error\")))",
                           &errors);
  EXPECT_TRUE(ok) << "the script itself runs to completion; only (stream-open) returns an error";
  EXPECT_EQ(cvc::state::instance(app)("demo.t.ok").value(), "false");
  EXPECT_NE(cvc::state::instance(app)("demo.t.err").value().find("no document scope"),
            std::string::npos);
  EXPECT_EQ(cvc::state::instance(app)("demo.streams.cam0").value(), ""); // nothing was opened
}

// ---- §10 tf_editor: the core's model / overlay / view edges, through the mock -----------------

namespace {

std::string tf_val(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app)(path).value();
}

void tf_set(cvc::app &app, const std::string &path, const std::string &v) {
  cvc::state::instance(app)(path).value(v);
}

cvc::tf::model tf_parse(const std::string &s) {
  cvc::tf::model m;
  EXPECT_TRUE(cvc::tf::from_string(s, m)) << s;
  return m;
}

// A small model with known nodes: colour at 0 / 0.5 / 1, opacity at 0 / 0.25 / 1, isocontour bone.
cvc::tf::model tf_sample() {
  cvc::tf::model m;
  m.color = {{0.0, 0.f, 0.f, 0.f}, {0.5, 1.f, 0.f, 0.f}, {1.0, 1.f, 1.f, 1.f}};
  m.alpha = {{0.0, 0.f}, {0.25, 0.5f}, {1.0, 1.f}};
  cvc::tf::isocontour bone;
  bone.id = "bone";
  bone.at = 0.6;
  m.isocontours = {bone};
  m.normalize();
  return m;
}

Value tf_scalar(const std::string &s) {
  Value v;
  v.scalar = s;
  return v;
}

Value tf_list(std::initializer_list<const char *> items) {
  Value v;
  v.kind = Value::Kind::Sequence;
  for (const char *i : items)
    v.items.push_back(tf_scalar(i));
  return v;
}

Widget tf_with(Widget w, const std::string &key, Value v) {
  w.props.entries.emplace_back(key, std::move(v));
  return w;
}

int tf_count(const std::vector<std::string> &warns, const std::string &needle) {
  return static_cast<int>(std::count_if(warns.begin(), warns.end(), [&](const std::string &w) {
    return w.find(needle) != std::string::npos;
  }));
}

// A fragment file for the load: mount tests (a fresh path per test).
std::filesystem::path tf_fragment(const std::string &dir, const std::string &content) {
  namespace fs = std::filesystem;
  const fs::path d = fs::temp_directory_path() / dir;
  fs::create_directories(d);
  const fs::path frag = d / "tf_panel.ari";
  std::ofstream(frag) << content;
  return frag;
}

} // namespace

TEST(AriadneTfEditor, SeedsAnEmptyKeyWithTheDefaultModel) {
  cvc::app app;
  Runtime rt(app, "ui");
  MockBackend mb;
  rt.set_backend(&mb);
  mb.tf_ret.drawn = true; // a native editor: nothing composed
  rt.set_root(group({tf_editor("TF", "tf")}));
  rt.render();
  const std::string seeded = cvc::tf::to_string(cvc::tf::model::default_model());
  EXPECT_EQ(tf_val(app, "ui.tf"), seeded);
  EXPECT_EQ(mb.tf_calls, 1);
  EXPECT_EQ(mb.last_tf_key, "ui.tf#TF"); // the resolved bind + the instance id
  EXPECT_EQ(cvc::tf::to_string(mb.last_tf_model), seeded);
  EXPECT_TRUE(mb.last_tf_in.interactive); // interactive_updates defaults on
  EXPECT_EQ(mb.last_tf_in.height, 0.0f);
  EXPECT_DOUBLE_EQ(mb.last_tf_in.lo, 0.0); // an auto domain with no published range: [0, 1]
  EXPECT_DOUBLE_EQ(mb.last_tf_in.hi, 1.0);
  EXPECT_FALSE(mb.saw("text_line:TF")); // drawn natively: no composed rows
  EXPECT_TRUE(rt.take_reactive_warnings().empty());
}

TEST(AriadneTfEditor, SeedsFromADefaultOrAPresetProp) {
  cvc::app app;
  Runtime rt(app, "ui");
  MockBackend mb;
  rt.set_backend(&mb);
  mb.tf_ret.drawn = true;
  const cvc::tf::model custom = tf_sample();
  rt.set_root(group({
      tf_with(tf_editor("A", "a"), "preset", tf_scalar("hot")),
      tf_with(tf_with(tf_editor("B", "b"), "default", tf_scalar(cvc::tf::to_string(custom))),
              "preset", tf_scalar("hot")),
      tf_with(tf_with(tf_editor("C", "c"), "default", tf_scalar("{not a model")), "preset",
              tf_scalar("cool")),
  }));
  rt.render();
  rt.render();
  EXPECT_EQ(tf_val(app, "ui.a"), cvc::tf::to_string(cvc::tf::model::preset("hot")));
  EXPECT_EQ(tf_val(app, "ui.b"), cvc::tf::to_string(custom)); // default: wins over preset:
  EXPECT_EQ(tf_val(app, "ui.c"), cvc::tf::to_string(cvc::tf::model::preset("cool")));
  const std::vector<std::string> warns = rt.take_reactive_warnings();
  EXPECT_EQ(warns.size(), 1u); // the broken default:, once
  EXPECT_EQ(tf_count(warns, "'C'"), 1);
  EXPECT_EQ(tf_count(warns, "default"), 1);
}

// The scene transfer_function: vocabulary on the widget itself: a standalone editor's starting
// model, the lists replacing their part of the preset.
TEST(AriadneTfEditor, SeedsInlineListsOverThePreset) {
  if (!have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  cvc::app app;
  Runtime rt(app, "ui");
  MockBackend mb;
  rt.set_backend(&mb);
  mb.tf_ret.drawn = true;
  LoadResult lr = load_string(R"(
windows:
  - window: W
    children:
      - tf_editor: TF
        bind: tf
        preset: grayscale
        alpha: [[0, 0], [0.5, 0.25], [1, 0.5]]
        isocontours: [{id: bone, at: 0.6, rgb: [1, 0.5, 0]}, 0.3]
        domain: [-1, 3]
        opacity_cubed: true
)");
  ASSERT_TRUE(lr.ok) << lr.error;
  rt.set_root(std::move(lr.root));
  rt.render();
  const cvc::tf::model base = cvc::tf::model::preset("grayscale");
  const cvc::tf::model m = tf_parse(tf_val(app, "ui.tf"));
  ASSERT_EQ(m.color.size(), base.color.size()); // the preset's colour ramp, untouched
  for (std::size_t i = 0; i < m.color.size(); ++i) {
    EXPECT_NEAR(m.color[i].at, base.color[i].at, 1e-6);
    EXPECT_NEAR(m.color[i].r, base.color[i].r, 1e-4);
    EXPECT_NEAR(m.color[i].g, base.color[i].g, 1e-4);
    EXPECT_NEAR(m.color[i].b, base.color[i].b, 1e-4);
  }
  ASSERT_EQ(m.alpha.size(), 3u);
  EXPECT_NEAR(m.alpha[1].at, 0.5, 1e-6);
  EXPECT_NEAR(m.alpha[1].a, 0.25, 1e-4);
  EXPECT_NEAR(m.alpha[2].a, 0.5, 1e-4);
  ASSERT_EQ(m.isocontours.size(), 2u); // sorted: the bare 0.3 first, given a fresh id
  EXPECT_NEAR(m.isocontours[0].at, 0.3, 1e-6);
  EXPECT_FALSE(m.isocontours[0].id.empty());
  EXPECT_EQ(m.isocontours[1].id, "bone");
  EXPECT_TRUE(m.isocontours[1].has_rgb);
  EXPECT_NEAR(m.isocontours[1].g, 0.5f, 1e-4);
  EXPECT_FALSE(m.domain.automatic);
  EXPECT_DOUBLE_EQ(m.domain.min, -1.0);
  EXPECT_DOUBLE_EQ(m.domain.max, 3.0);
  EXPECT_TRUE(m.opacity_cubed);
  EXPECT_TRUE(rt.take_reactive_warnings().empty());
}

TEST(AriadneTfEditor, RespectsAnExistingModel) {
  cvc::app app;
  Runtime rt(app, "ui");
  MockBackend mb;
  rt.set_backend(&mb);
  mb.tf_ret.drawn = true;
  const cvc::tf::model custom = tf_sample();
  tf_set(app, "ui.tf", cvc::tf::to_string(custom));
  rt.set_root(group({tf_with(tf_editor("TF", "tf"), "preset", tf_scalar("hot"))}));
  rt.render();
  EXPECT_EQ(tf_val(app, "ui.tf"), cvc::tf::to_string(custom)); // not re-seeded
  EXPECT_EQ(cvc::tf::to_string(mb.last_tf_model), cvc::tf::to_string(custom));
  // A valid non-canonical spelling (the roadmap's map nodes) is read, and left as written.
  const std::string map_form = R"({"color":[{"at":0,"rgb":[0,0,0]},{"at":1,"rgb":[1,1,1]}],)"
                               R"("alpha":[{"at":0,"a":0},{"at":1,"a":1}]})";
  tf_set(app, "ui.tf", map_form);
  rt.render();
  EXPECT_EQ(tf_val(app, "ui.tf"), map_form);
  ASSERT_EQ(mb.last_tf_model.color.size(), 2u);
  EXPECT_NEAR(mb.last_tf_model.color[1].r, 1.f, 1e-6);
  EXPECT_TRUE(rt.take_reactive_warnings().empty());
}

TEST(AriadneTfEditor, BackendCommitWritesTheNormalizedModelAndFiresOn) {
  cvc::app app;
  Runtime rt(app, "ui");
  MockBackend mb;
  rt.set_backend(&mb);
  mb.tf_ret.drawn = true;
  Widget w = tf_editor("TF", "tf");
  w.on = "tf_changed";
  int fired = 0;
  rt.on("tf_changed", [&] { ++fired; });
  rt.set_root(group({w}));
  rt.render(); // seeds
  rt.drain();
  EXPECT_EQ(fired, 0); // seeding is not an edit
  cvc::tf::model edit; // unsorted, no endpoints: the core normalizes before it writes
  edit.color = {{0.7, 0.f, 0.f, 1.f}, {0.2, 1.f, 0.f, 0.f}};
  edit.alpha = {{0.5, 0.5f}};
  mb.tf_ret.changed = mb.tf_ret.committed = true;
  mb.tf_ret.model = edit;
  rt.render();
  rt.drain();
  cvc::tf::model expect = edit;
  expect.normalize();
  EXPECT_EQ(tf_val(app, "ui.tf"), cvc::tf::to_string(expect));
  EXPECT_EQ(fired, 1);
  // The same commit again changes nothing: no write, no second on:.
  rt.render();
  rt.drain();
  EXPECT_EQ(fired, 1);
  EXPECT_EQ(cvc::tf::to_string(mb.last_tf_model), cvc::tf::to_string(expect)); // what it draws
}

TEST(AriadneTfEditor, UncommittedEditDoesNotWrite) {
  cvc::app app;
  Runtime rt(app, "ui");
  MockBackend mb;
  rt.set_backend(&mb);
  mb.tf_ret.drawn = true;
  rt.set_root(
      group({tf_with(tf_with(tf_editor("TF", "tf"), "interactive_updates", tf_scalar("false")),
                     "height", tf_scalar("140"))}));
  rt.render();
  EXPECT_FALSE(mb.last_tf_in.interactive); // commit on release
  EXPECT_EQ(mb.last_tf_in.height, 140.0f);
  const std::string seeded = tf_val(app, "ui.tf");
  mb.tf_ret.changed = true; // a drag in flight: the backend holds the working model
  mb.tf_ret.model = tf_sample();
  rt.render();
  EXPECT_EQ(tf_val(app, "ui.tf"), seeded);
}

TEST(AriadneTfEditor, ViewIsSeededAndPersistedUnderTreeId) {
  cvc::app app;
  Runtime rt(app, "ui");
  MockBackend mb;
  rt.set_backend(&mb);
  mb.tf_ret.drawn = true;
  Widget w = tf_with(tf_editor("TF", "tf"), "layers", tf_list({"contour_tree"}));
  w.id = "skull_tf";
  rt.set_root(group({w}));
  rt.render();
  const std::string v = "ui.tree.skull_tf.view.";
  EXPECT_EQ(mb.last_tf_key, "ui.tf#skull_tf");
  EXPECT_EQ(tf_val(app, v + "histogram"), "0"); // layers: replaces the default layer set
  EXPECT_EQ(tf_val(app, v + "contour_tree"), "1");
  EXPECT_EQ(tf_val(app, v + "contour_spectrum"), "0");
  EXPECT_EQ(tf_val(app, v + "color"), "1");
  EXPECT_EQ(tf_val(app, v + "opacity"), "1");
  EXPECT_EQ(tf_val(app, v + "zoom"), cvc::tf::range_to_string(0.0, 1.0));
  EXPECT_TRUE(mb.last_tf_in.view.show_contour_tree);
  EXPECT_FALSE(mb.last_tf_in.view.show_histogram);
  // A zoom / layer change the backend reports persists ...
  mb.tf_ret.view_changed = true;
  mb.tf_ret.view = mb.last_tf_in.view;
  mb.tf_ret.view.zoom_lo = 0.25;
  mb.tf_ret.view.zoom_hi = 0.75;
  mb.tf_ret.view.show_histogram = true;
  rt.render();
  EXPECT_EQ(tf_val(app, v + "zoom"), cvc::tf::range_to_string(0.25, 0.75));
  EXPECT_EQ(tf_val(app, v + "histogram"), "1");
  // ... and is what the backend gets next frame: the state, not props, is the authority.
  mb.tf_ret.view_changed = false;
  rt.render();
  EXPECT_NEAR(mb.last_tf_in.view.zoom_lo, 0.25, 1e-9);
  EXPECT_NEAR(mb.last_tf_in.view.zoom_hi, 0.75, 1e-9);
  EXPECT_TRUE(mb.last_tf_in.view.show_histogram);
  // A write from outside the editor (a checkbox bound to the key) reaches it too.
  tf_set(app, v + "histogram", "0");
  rt.render();
  EXPECT_FALSE(mb.last_tf_in.view.show_histogram);
}

TEST(AriadneTfEditor, LazyOverlaysAreRequestedOnceAndTheRequestOnlyGrows) {
  cvc::app app;
  Runtime rt(app, "ui");
  MockBackend mb;
  rt.set_backend(&mb);
  mb.tf_ret.drawn = true;
  rt.set_root(group({tf_with(tf_editor("A", "tf"), "layers", tf_list({"contour_spectrum"})),
                     tf_with(tf_editor("B", "tf"), "layers", tf_list({"histogram"}))}));
  rt.render();
  rt.render();
  const std::string req = "ui.tf.overlay_request";
  // Only the costly layer an editor shows is asked for (the histogram is always published).
  EXPECT_EQ(tf_val(app, req), cvc::tf::overlay_bits_to_csv(cvc::tf::overlay_contour_spectrum));
  // B turns its contour tree on (a checkbox bound to its view key): the request grows.
  tf_set(app, "ui.tree.B.view.contour_tree", "1");
  rt.render();
  const unsigned both = cvc::tf::overlay_contour_spectrum | cvc::tf::overlay_contour_tree;
  EXPECT_EQ(cvc::tf::overlay_bits_from_csv(tf_val(app, req)), both);
  // A hides its spectrum: the request never shrinks (another editor may still show the layer).
  tf_set(app, "ui.tree.A.view.contour_spectrum", "0");
  rt.render();
  EXPECT_EQ(cvc::tf::overlay_bits_from_csv(tf_val(app, req)), both);
  // Once the producer has published the layers nobody asks again (here it cleared the request).
  cvc::tf::spectrum_overlay s;
  s.area = s.min_volume = s.max_volume = s.gradient = {0.0, 1.0, 2.0, 1.0};
  cvc::tf::tree_overlay t;
  t.vertices = {{0.5f, 0.0}, {0.5f, 1.0}};
  t.edges = {{0, 1}};
  tf_set(app, "ui.tf.contour_spectrum", cvc::tf::spectrum_to_string(s));
  tf_set(app, "ui.tf.contour_tree", cvc::tf::tree_to_string(t));
  tf_set(app, req, "");
  tf_set(app, "ui.tree.A.view.contour_spectrum", "1");
  rt.render();
  EXPECT_EQ(tf_val(app, req), "");
  EXPECT_EQ(mb.last_tf_overlays.spectrum.size(), 4u); // and the layers reached the backend
  EXPECT_FALSE(mb.last_tf_overlays.tree.empty());
}

TEST(AriadneTfEditor, MalformedModelKeepsTheLastGoodOneAndWarnsOnce) {
  cvc::app app;
  Runtime rt(app, "ui");
  MockBackend mb;
  rt.set_backend(&mb);
  mb.tf_ret.drawn = true;
  const cvc::tf::model good = tf_sample();
  tf_set(app, "ui.tf", cvc::tf::to_string(good));
  rt.set_root(group({tf_editor("TF", "tf")}));
  rt.render();
  EXPECT_TRUE(rt.take_reactive_warnings().empty());
  const std::string bad = "{\"color\": oops";
  tf_set(app, "ui.tf", bad);
  rt.render();
  rt.render();
  rt.render();
  EXPECT_EQ(cvc::tf::to_string(mb.last_tf_model), cvc::tf::to_string(good));
  EXPECT_EQ(tf_val(app, "ui.tf"), bad); // never clobbered by the widget
  const std::vector<std::string> warns = rt.take_reactive_warnings();
  ASSERT_EQ(warns.size(), 1u);
  EXPECT_NE(warns[0].find("does not parse"), std::string::npos) << warns[0];
  // A good model written later is picked up.
  const cvc::tf::model next = cvc::tf::model::preset("hot");
  tf_set(app, "ui.tf", cvc::tf::to_string(next));
  rt.render();
  EXPECT_EQ(cvc::tf::to_string(mb.last_tf_model), cvc::tf::to_string(next));
}

TEST(AriadneTfEditor, AModelThatNeverParsedShowsTheSeedWithoutWritingIt) {
  cvc::app app;
  Runtime rt(app, "ui");
  MockBackend mb;
  rt.set_backend(&mb);
  mb.tf_ret.drawn = true;
  tf_set(app, "ui.tf", "garbage");
  rt.set_root(group({tf_with(tf_editor("TF", "tf"), "preset", tf_scalar("hot"))}));
  rt.render();
  rt.render();
  EXPECT_EQ(tf_val(app, "ui.tf"), "garbage");
  EXPECT_EQ(cvc::tf::to_string(mb.last_tf_model),
            cvc::tf::to_string(cvc::tf::model::preset("hot")));
  EXPECT_EQ(rt.take_reactive_warnings().size(), 1u);
}

TEST(AriadneTfEditor, ComposedFallbackEmitsARowPerNode) {
  cvc::app app;
  Runtime rt(app, "ui");
  MockBackend mb; // tf_ret.drawn == false: no native editor
  rt.set_backend(&mb);
  const std::string src = cvc::tf::to_string(tf_sample());
  tf_set(app, "ui.tf", src);
  rt.set_root(group({tf_editor("TF", "tf")}));
  rt.render();
  EXPECT_TRUE(mb.saw("tf_editor:TF@ui.tf#TF")); // the backend is asked first ...
  EXPECT_TRUE(mb.saw("text_line:TF"));          // ... then the core composes the editor
  EXPECT_TRUE(mb.saw("text_value:domain=0 .. 1"));
  EXPECT_TRUE(mb.saw("text_value:colour 1=at 0")); // an endpoint is pinned: no position slider
  EXPECT_TRUE(mb.saw("slider_double:colour 2"));
  EXPECT_TRUE(mb.saw("text_value:colour 3=at 1"));
  EXPECT_EQ(mb.times("color:rgb"), 3);
  EXPECT_TRUE(mb.saw_prefix("text_value:rgb=")); // this backend draws no picker: the value as text
  EXPECT_TRUE(mb.saw("slider_double:alpha 1 at 0"));
  EXPECT_TRUE(mb.saw("slider_double:alpha 2"));
  EXPECT_EQ(mb.times("slider_double:opacity"), 1);
  EXPECT_TRUE(mb.saw("slider_double:alpha 3 at 1"));
  EXPECT_TRUE(mb.saw("slider_double:iso bone"));
  EXPECT_EQ(mb.times("button:x"), 3); // colour 2, alpha 2 and bone: endpoints cannot be deleted
  for (const char *b : {"button:+ colour", "button:+ alpha", "button:+ isocontour", "button:reset"})
    EXPECT_TRUE(mb.saw(b)) << b;
  for (const char *c : {"checkbox:opacity cubed", "checkbox:histogram", "checkbox:contour spectrum",
                        "checkbox:contour tree"})
    EXPECT_TRUE(mb.saw(c)) << c;
  EXPECT_FALSE(mb.saw_prefix("set_image:")); // no pixels on this backend: no preview
  EXPECT_EQ(tf_val(app, "ui.tf"), src);      // drawing alone writes nothing
}

TEST(AriadneTfEditor, ComposedSlidersAndButtonsEditTheModel) {
  cvc::app app;
  Runtime rt(app, "ui");
  MockBackend mb;
  rt.set_backend(&mb);
  tf_set(app, "ui.tf", cvc::tf::to_string(tf_sample()));
  Widget w = tf_editor("TF", "tf");
  w.on = "tf_changed";
  int fired = 0;
  rt.on("tf_changed", [&] { ++fired; });
  rt.set_root(group({w}));
  rt.render();
  const std::string k = "ui.tf#TF#"; // a composed row's slider key: <backend key>#<row>
  // Slide the isocontour (no data range is published: data units = the normalized axis).
  mb.slider_double_ret[k + "ibone"] = {true, true, 0.8};
  rt.render();
  rt.drain();
  mb.slider_double_ret.clear();
  cvc::tf::model m = tf_parse(tf_val(app, "ui.tf"));
  ASSERT_EQ(m.isocontours.size(), 1u);
  EXPECT_EQ(m.isocontours[0].id, "bone");
  EXPECT_NEAR(m.isocontours[0].at, 0.8, 1e-6);
  EXPECT_EQ(fired, 1);
  // The second opacity node's opacity, and the middle colour node's position, in one frame.
  mb.slider_double_ret[k + "a1a"] = {true, true, 0.9};
  mb.slider_double_ret[k + "c1"] = {true, true, 0.3};
  rt.render();
  mb.slider_double_ret.clear();
  m = tf_parse(tf_val(app, "ui.tf"));
  ASSERT_EQ(m.alpha.size(), 3u);
  EXPECT_NEAR(m.alpha[1].a, 0.9, 1e-4);
  EXPECT_NEAR(m.alpha[1].at, 0.25, 1e-6); // the opacity slider leaves the position alone
  ASSERT_EQ(m.color.size(), 3u);
  EXPECT_NEAR(m.color[1].at, 0.3, 1e-6);
  // + isocontour adds one (in the widest gap); x deletes a non-endpoint node.
  mb.click_button = "+ isocontour";
  rt.render();
  m = tf_parse(tf_val(app, "ui.tf"));
  ASSERT_EQ(m.isocontours.size(), 2u);
  EXPECT_NE(m.isocontours[0].id, m.isocontours[1].id);
  EXPECT_GT(std::abs(m.isocontours[0].at - m.isocontours[1].at), 0.1);
  mb.click_button = "x"; // every row's x reports a click: one delete per list this frame
  rt.render();
  mb.click_button.clear();
  m = tf_parse(tf_val(app, "ui.tf"));
  ASSERT_EQ(m.color.size(), 2u); // the endpoints survive
  EXPECT_DOUBLE_EQ(m.color[0].at, 0.0);
  EXPECT_DOUBLE_EQ(m.color[1].at, 1.0);
  EXPECT_EQ(m.alpha.size(), 2u);
  EXPECT_EQ(m.isocontours.size(), 1u);
}

TEST(AriadneTfEditor, ComposedSlidersWorkInDataUnits) {
  cvc::app app;
  Runtime rt(app, "ui");
  MockBackend mb;
  rt.set_backend(&mb);
  tf_set(app, "ui.tf", cvc::tf::to_string(tf_sample()));
  tf_set(app, "ui.tf.range", cvc::tf::range_to_string(-10.0, 10.0)); // the auto domain
  rt.set_root(group({tf_editor("TF", "tf")}));
  rt.render();
  EXPECT_TRUE(mb.saw("text_value:domain=-10 .. 10"));
  EXPECT_TRUE(mb.saw("text_value:colour 1=at -10"));
  mb.slider_double_ret["ui.tf#TF#ibone"] = {true, true, 5.0}; // the isovalue, in data units
  rt.render();
  const cvc::tf::model m = tf_parse(tf_val(app, "ui.tf"));
  ASSERT_EQ(m.isocontours.size(), 1u);
  EXPECT_NEAR(m.isocontours[0].at, 0.75, 1e-6); // stored normalized
}

TEST(AriadneTfEditor, ComposedResetKeepsTheDomain) {
  cvc::app app;
  Runtime rt(app, "ui");
  MockBackend mb;
  rt.set_backend(&mb);
  cvc::tf::model start = tf_sample();
  start.domain.automatic = false;
  start.domain.min = -8.0;
  start.domain.max = 8.0;
  start.resolution = 512;
  tf_set(app, "ui.tf", cvc::tf::to_string(start));
  rt.set_root(group({tf_editor("TF", "tf")}));
  rt.render();
  mb.click_button = "reset";
  rt.render();
  mb.click_button.clear();
  cvc::tf::model expect = cvc::tf::model::default_model();
  expect.domain = start.domain;
  expect.resolution = 512;
  EXPECT_EQ(tf_val(app, "ui.tf"), cvc::tf::to_string(expect));
}

TEST(AriadneTfEditor, ComposedTogglesLayersAndOpacityCubed) {
  cvc::app app;
  Runtime rt(app, "ui");
  MockBackend mb;
  rt.set_backend(&mb);
  tf_set(app, "ui.tf", cvc::tf::to_string(tf_sample()));
  rt.set_root(group({tf_editor("TF", "tf")}));
  rt.render();
  EXPECT_EQ(tf_val(app, "ui.tree.TF.view.contour_tree"), "0");
  EXPECT_EQ(tf_val(app, "ui.tf.overlay_request"), "");
  mb.checkbox_by_label["contour tree"] = {true, true, true};
  mb.checkbox_by_label["opacity cubed"] = {true, true, true};
  rt.render();
  mb.checkbox_by_label.clear();
  EXPECT_EQ(tf_val(app, "ui.tree.TF.view.contour_tree"), "1");
  EXPECT_EQ(tf_val(app, "ui.tf.overlay_request"),
            cvc::tf::overlay_bits_to_csv(cvc::tf::overlay_contour_tree));
  EXPECT_TRUE(tf_parse(tf_val(app, "ui.tf")).opacity_cubed);
}

TEST(AriadneTfEditor, ComposedPreviewIsRasterizedOnlyWhenItChanges) {
  cvc::app app;
  Runtime rt(app, "ui");
  MockBackend mb;
  mb.caps_bitmap = true; // a backend that shows pixels gets the preview raster
  rt.set_backend(&mb);
  tf_set(app, "ui.tf", cvc::tf::to_string(tf_sample()));
  rt.set_root(group({tf_editor("TF", "tf")}));
  rt.render();
  rt.render();
  const std::string img = "tf_editor:ui.tf#TF";
  EXPECT_EQ(mb.times("set_image:" + img), 1); // rasterized once ...
  EXPECT_EQ(mb.times("image:" + img), 2);     // ... and drawn every frame
  tf_set(app, "ui.tf", cvc::tf::to_string(cvc::tf::model::preset("hot")));
  rt.render();
  EXPECT_EQ(mb.times("set_image:" + img), 2); // a new model re-rasterizes
  tf_set(app, "ui.tf.histogram", cvc::tf::histogram_to_string({1.0, 2.0, 3.0}));
  rt.render();
  EXPECT_EQ(mb.times("set_image:" + img), 3); // so does a newly published layer
  rt.render();
  EXPECT_EQ(mb.times("set_image:" + img), 3);
}

TEST(AriadneTfEditor, PublishedOverlaysReachTheBackend) {
  cvc::app app;
  Runtime rt(app, "ui");
  MockBackend mb;
  rt.set_backend(&mb);
  mb.tf_ret.drawn = true;
  tf_set(app, "ui.tf", cvc::tf::to_string(tf_sample()));
  tf_set(app, "ui.tf.range", cvc::tf::range_to_string(-2.0, 6.0));
  tf_set(app, "ui.tf.histogram", cvc::tf::histogram_to_string({1.0, 2.0, 3.0, 4.0}));
  tf_set(app, "ui.tf.overlay_status", "computing the contour tree");
  rt.set_root(group({tf_editor("TF", "tf")}));
  rt.render();
  EXPECT_TRUE(mb.last_tf_overlays.has_range);
  EXPECT_NEAR(mb.last_tf_in.lo, -2.0, 1e-9); // the auto domain follows the published range
  EXPECT_NEAR(mb.last_tf_in.hi, 6.0, 1e-9);
  EXPECT_EQ(mb.last_tf_overlays.histogram.size(), 4u);
  EXPECT_EQ(mb.last_tf_overlays.status, "computing the contour tree");
  // A later publish reaches it the next frame.
  tf_set(app, "ui.tf.histogram", cvc::tf::histogram_to_string({5.0, 6.0}));
  rt.render();
  EXPECT_EQ(mb.last_tf_overlays.histogram.size(), 2u);
  // An explicit domain wins over the published range.
  cvc::tf::model fixed = tf_sample();
  fixed.domain.automatic = false;
  fixed.domain.min = 0.0;
  fixed.domain.max = 10.0;
  tf_set(app, "ui.tf", cvc::tf::to_string(fixed));
  rt.render();
  EXPECT_NEAR(mb.last_tf_in.lo, 0.0, 1e-9);
  EXPECT_NEAR(mb.last_tf_in.hi, 10.0, 1e-9);
}

// A mesher's interval (fe_lab publishes <tf>.interval = "<id_a>,<id_b>") reaches the backend as the
// two isocontour ids, only while both are in the model; and the composed preview re-renders for it.
TEST(AriadneTfEditor, TheMeshedIntervalReachesTheBackend) {
  cvc::app app;
  Runtime rt(app, "ui");
  MockBackend mb;
  rt.set_backend(&mb);
  mb.tf_ret.drawn = true;
  cvc::tf::model m = tf_sample(); // bone at 0.6
  cvc::tf::isocontour skin, lung;
  skin.id = "skin";
  skin.at = 0.3;
  lung.id = "lung";
  lung.at = 0.1;
  m.isocontours.push_back(skin);
  m.isocontours.push_back(lung);
  m.normalize();
  tf_set(app, "ui.tf", cvc::tf::to_string(m));
  rt.set_root(group({tf_editor("TF", "tf")}));
  rt.render();
  EXPECT_TRUE(mb.last_tf_in.interval_a.empty()); // nothing published
  EXPECT_TRUE(mb.last_tf_in.interval_b.empty());
  tf_set(app, "ui.tf.interval", "bone,skin");
  rt.render();
  EXPECT_EQ(mb.last_tf_in.interval_a, "bone");
  EXPECT_EQ(mb.last_tf_in.interval_b, "skin");
  tf_set(app, "ui.tf.interval", "skin,bone"); // the order is the mesher's (isovalue, isovalue2)
  rt.render();
  EXPECT_EQ(mb.last_tf_in.interval_a, "skin");
  EXPECT_EQ(mb.last_tf_in.interval_b, "bone");
  // An id the model no longer has (deleted, the mesher not yet caught up): no band.
  tf_set(app, "ui.tf.interval", "skin,gone");
  rt.render();
  EXPECT_TRUE(mb.last_tf_in.interval_a.empty());
  EXPECT_TRUE(mb.last_tf_in.interval_b.empty());
  tf_set(app, "ui.tf.interval", "");
  rt.render();
  EXPECT_TRUE(mb.last_tf_in.interval_a.empty());

  // The composed editor's preview: a published interval re-rasterizes it, an unchanged one not.
  cvc::app app2;
  Runtime rt2(app2, "ui");
  MockBackend mc;
  mc.caps_bitmap = true;
  rt2.set_backend(&mc);
  tf_set(app2, "ui.tf", cvc::tf::to_string(m));
  rt2.set_root(group({tf_editor("TF", "tf")}));
  rt2.render();
  const std::string img = "set_image:tf_editor:ui.tf#TF";
  EXPECT_EQ(mc.times(img), 1);
  tf_set(app2, "ui.tf.interval", "lung,bone");
  rt2.render();
  EXPECT_EQ(mc.times(img), 2);
  rt2.render();
  EXPECT_EQ(mc.times(img), 2);
}

// Through a load: link hole the interval is read beside the hole's TARGET, like the overlays.
TEST(AriadneTfEditor, ALinkHoleReachesTheParentsInterval) {
  if (!have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  const std::filesystem::path frag =
      tf_fragment("ariadne_tf_interval_hole", "root:\n  - tf_editor: TF\n    bind: tf\n");
  cvc::app app;
  Runtime rt(app, "ui.demo");
  MockBackend mb;
  rt.set_backend(&mb);
  mb.tf_ret.drawn = true;
  const std::string parent = "ui.demo.scene.domain.transfer_function";
  cvc::tf::model m = tf_sample();
  cvc::tf::isocontour skin;
  skin.id = "skin";
  skin.at = 0.3;
  m.isocontours.push_back(skin);
  m.normalize();
  tf_set(app, parent, cvc::tf::to_string(m));
  tf_set(app, parent + ".interval", "skin,bone");
  LoadResult lr =
      load_string("windows:\n  - window: W\n    children:\n      - load: " + frag.string() +
                  "\n        as: rf\n        link:\n          tf: "
                  "scene.domain.transfer_function\n");
  ASSERT_TRUE(lr.ok) << lr.error;
  rt.set_root(std::move(lr.root));
  rt.render();
  EXPECT_EQ(mb.last_tf_in.interval_a, "skin");
  EXPECT_EQ(mb.last_tf_in.interval_b, "bone");
}

TEST(AriadneTfEditor, PropsRedirectTheOverlaySources) {
  cvc::app app;
  Runtime rt(app, "ui");
  MockBackend mb;
  rt.set_backend(&mb);
  mb.tf_ret.drawn = true;
  tf_set(app, "ui.tf", cvc::tf::to_string(tf_sample()));
  tf_set(app, "ui.tf.range", cvc::tf::range_to_string(0.0, 1000.0)); // not read: redirected
  tf_set(app, "ui.stats.range", cvc::tf::range_to_string(5.0, 15.0));
  tf_set(app, "ui.stats.hist", cvc::tf::histogram_to_string({1.0, 1.0, 1.0}));
  rt.set_root(group({tf_with(tf_with(tf_editor("TF", "tf"), "range", tf_scalar("stats.range")),
                             "histogram", tf_scalar("stats.hist"))}));
  rt.render();
  EXPECT_NEAR(mb.last_tf_in.lo, 5.0, 1e-9); // the redirect is a bind, resolved in the scope
  EXPECT_NEAR(mb.last_tf_in.hi, 15.0, 1e-9);
  EXPECT_EQ(mb.last_tf_overlays.histogram.size(), 3u);
}

TEST(AriadneTfEditor, TwoEditorsOnOneModelKeepTheirOwnKeyAndView) {
  cvc::app app;
  Runtime rt(app, "ui");
  MockBackend mb;
  rt.set_backend(&mb);
  mb.tf_ret.drawn = true;
  rt.set_root(group({tf_editor("Main", "tf"),
                     tf_with(tf_editor("Inset", "tf"), "layers", tf_list({"contour_tree"}))}));
  rt.render();
  EXPECT_TRUE(mb.saw("tf_editor:Main@ui.tf#Main")); // one model, two gestures
  EXPECT_TRUE(mb.saw("tf_editor:Inset@ui.tf#Inset"));
  EXPECT_EQ(tf_val(app, "ui.tree.Main.view.contour_tree"), "0");
  EXPECT_EQ(tf_val(app, "ui.tree.Inset.view.contour_tree"), "1");
  // An edit through one is what the other draws: both report it, it is written once.
  const cvc::tf::model cool = cvc::tf::model::preset("cool");
  mb.tf_ret.committed = mb.tf_ret.changed = true;
  mb.tf_ret.model = cool;
  rt.render();
  mb.tf_ret.committed = mb.tf_ret.changed = false;
  mb.log.clear();
  rt.render();
  EXPECT_EQ(tf_val(app, "ui.tf"), cvc::tf::to_string(cool));
  EXPECT_EQ(cvc::tf::to_string(mb.last_tf_model), cvc::tf::to_string(cool)); // Inset's frame
}

TEST(AriadneTfEditor, BindsResolveInsideALoadMount) {
  if (!have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  const std::filesystem::path frag =
      tf_fragment("ariadne_tf_mount", "root:\n  - tf_editor: TF\n    bind: tf\n"
                                      "    layers: [contour_spectrum]\n");
  cvc::app app;
  Runtime rt(app, "ui.demo");
  MockBackend mb;
  rt.set_backend(&mb);
  mb.tf_ret.drawn = true;
  LoadResult lr =
      load_string("windows:\n  - window: W\n    children:\n      - load: " + frag.string() +
                  "\n        as: rf\n");
  ASSERT_TRUE(lr.ok) << lr.error;
  rt.set_root(std::move(lr.root));
  rt.render();
  const std::string sub = "ui.demo.includes.rf.";
  EXPECT_EQ(tf_val(app, sub + "tf"), cvc::tf::to_string(cvc::tf::model::default_model()));
  EXPECT_EQ(mb.last_tf_key, sub + "tf#TF");
  EXPECT_EQ(tf_val(app, sub + "tree.TF.view.contour_spectrum"), "1");
  EXPECT_EQ(tf_val(app, sub + "tf.overlay_request"),
            cvc::tf::overlay_bits_to_csv(cvc::tf::overlay_contour_spectrum));
  EXPECT_EQ(tf_val(app, "ui.demo.tf"), ""); // nothing at the document scope
}

// A module given a parent's model through ONE link hole still sees the layers published beside the
// target (keys under a hole are its own children, not the target's) and asks the target's producer.
TEST(AriadneTfEditor, ALinkHoleReachesTheParentsModelAndLayers) {
  if (!have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  const std::filesystem::path frag =
      tf_fragment("ariadne_tf_hole", "root:\n  - tf_editor: TF\n    bind: tf\n"
                                     "    layers: [contour_spectrum]\n");
  cvc::app app;
  Runtime rt(app, "ui.demo");
  MockBackend mb;
  rt.set_backend(&mb);
  mb.tf_ret.drawn = true;
  const std::string parent = "ui.demo.scene.vol.transfer_function";
  tf_set(app, parent, cvc::tf::to_string(tf_sample()));
  tf_set(app, parent + ".range", cvc::tf::range_to_string(-4.0, 4.0));
  tf_set(app, parent + ".histogram", cvc::tf::histogram_to_string({1.0, 2.0}));
  LoadResult lr =
      load_string("windows:\n  - window: W\n    children:\n      - load: " + frag.string() +
                  "\n        as: rf\n        link:\n          tf: scene.vol.transfer_function\n");
  ASSERT_TRUE(lr.ok) << lr.error;
  rt.set_root(std::move(lr.root));
  rt.render();
  EXPECT_EQ(cvc::tf::to_string(mb.last_tf_model), cvc::tf::to_string(tf_sample()));
  EXPECT_NEAR(mb.last_tf_in.lo, -4.0, 1e-9);
  EXPECT_NEAR(mb.last_tf_in.hi, 4.0, 1e-9);
  EXPECT_EQ(mb.last_tf_overlays.histogram.size(), 2u);
  EXPECT_EQ(tf_val(app, parent + ".overlay_request"),
            cvc::tf::overlay_bits_to_csv(cvc::tf::overlay_contour_spectrum));
  // An edit writes through the (read-write) hole to the parent's model.
  const cvc::tf::model hot = cvc::tf::model::preset("hot");
  mb.tf_ret.committed = mb.tf_ret.changed = true;
  mb.tf_ret.model = hot;
  rt.render();
  EXPECT_EQ(tf_val(app, parent), cvc::tf::to_string(hot));
}

// ---- §10 tf_editor: props typing, locale and no-op commits (review C6/C7/C8) -------------------

namespace {

Value tf_seq(std::vector<Value> items) {
  Value v;
  v.kind = Value::Kind::Sequence;
  v.items = std::move(items);
  return v;
}

Value tf_map(std::vector<std::pair<std::string, Value>> entries) {
  Value v;
  v.kind = Value::Kind::Map;
  v.entries = std::move(entries);
  return v;
}

// LC_NUMERIC switched to a decimal-comma locale for one scope (inactive when the host has none).
class DecimalCommaLocale {
public:
  DecimalCommaLocale() {
    if (const char *cur = std::setlocale(LC_NUMERIC, nullptr))
      saved_ = cur;
    for (const char *name : {"de_DE.UTF-8", "de_DE.utf8", "de_DE", "de-DE", "German_Germany.1252",
                             "fr_FR.UTF-8", "fr-FR", "French_France.1252"}) {
      if (!std::setlocale(LC_NUMERIC, name))
        continue;
      char buf[16];
      std::snprintf(buf, sizeof(buf), "%.1f", 0.5);
      if (std::string(buf) == "0,5") {
        active_ = true;
        return;
      }
    }
    restore();
  }
  ~DecimalCommaLocale() { restore(); }
  DecimalCommaLocale(const DecimalCommaLocale &) = delete;
  DecimalCommaLocale &operator=(const DecimalCommaLocale &) = delete;
  bool active() const { return active_; }

private:
  void restore() { std::setlocale(LC_NUMERIC, saved_.empty() ? "C" : saved_.c_str()); }
  std::string saved_;
  bool active_ = false;
};

} // namespace

// The inline seed lists and a YAML-map default: become model JSON in the core. A decimal-comma C
// locale must not reach that JSON ("0,5" splits one number into two and drops the whole seed).
TEST(AriadneTfEditor, InlineSeedIsLocaleIndependent) {
  DecimalCommaLocale comma;
  if (!comma.active())
    GTEST_SKIP() << "no decimal-comma locale on this host";
  cvc::app app;
  Runtime rt(app, "ui");
  MockBackend mb;
  rt.set_backend(&mb);
  mb.tf_ret.drawn = true;
  Widget a = tf_with(tf_editor("A", "a"), "alpha",
                     tf_seq({tf_list({"0", "0"}), tf_list({"0.5", "0.25"}), tf_list({"1", "1"})}));
  a = tf_with(std::move(a), "isocontours",
              tf_seq({tf_map({{"id", tf_scalar("bone")}, {"at", tf_scalar("0.6")}})}));
  a = tf_with(std::move(a), "domain", tf_list({"-1.5", "2.5"}));
  const Value dflt =
      tf_map({{"color", tf_seq({tf_list({"0", "0", "0", "0"}), tf_list({"0.5", "1", "0", "0"}),
                                tf_list({"1", "1", "1", "1"})})},
              {"domain", tf_scalar("auto")}});
  rt.set_root(group({a, tf_with(tf_editor("B", "b"), "default", dflt)}));
  rt.render();
  const cvc::tf::model ma = tf_parse(tf_val(app, "ui.a"));
  ASSERT_EQ(ma.alpha.size(), 3u);
  EXPECT_NEAR(ma.alpha[1].at, 0.5, 1e-9);
  EXPECT_NEAR(ma.alpha[1].a, 0.25, 1e-6);
  ASSERT_EQ(ma.isocontours.size(), 1u);
  EXPECT_EQ(ma.isocontours[0].id, "bone");
  EXPECT_NEAR(ma.isocontours[0].at, 0.6, 1e-9);
  EXPECT_FALSE(ma.domain.automatic);
  EXPECT_DOUBLE_EQ(ma.domain.min, -1.5);
  EXPECT_DOUBLE_EQ(ma.domain.max, 2.5);
  const cvc::tf::model mb_model = tf_parse(tf_val(app, "ui.b"));
  ASSERT_EQ(mb_model.color.size(), 3u);
  EXPECT_NEAR(mb_model.color[1].at, 0.5, 1e-9);
  EXPECT_NEAR(mb_model.color[1].r, 1.f, 1e-6);
  EXPECT_TRUE(mb_model.domain.automatic);
  const std::vector<std::string> warns = rt.take_reactive_warnings();
  EXPECT_TRUE(warns.empty()) << warns.front();
}

// Props keep no YAML type, so the core types each value by the model's schema, not by its text: an
// isocontour id is a string however it reads, a flag takes the loader's flag() spellings (any
// case; an unknown one keeps the default), a number is a number.
TEST(AriadneTfEditor, PropsAreTypedByTheSchemaNotGuessedFromTheText) {
  if (!have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  cvc::app app;
  Runtime rt(app, "ui");
  MockBackend mb;
  rt.set_backend(&mb);
  mb.tf_ret.drawn = true;
  LoadResult lr = load_string(R"(
windows:
  - window: W
    children:
      - tf_editor: TF
        bind: tf
        opacity_cubed: True
        alpha: [[0, 0], [0.5, 0.25], [1, 1]]
        isocontours: [{id: "01", at: 0.25}, {id: 7, at: 0.75}]
        domain: AUTO
        interactive_updates: True
        show_color: YES
        show_opacity: Off
        show_isocontours: maybe
        log_histogram: TRUE
        info: On
)");
  ASSERT_TRUE(lr.ok) << lr.error;
  rt.set_root(std::move(lr.root));
  rt.render();
  const cvc::tf::model m = tf_parse(tf_val(app, "ui.tf"));
  EXPECT_TRUE(m.opacity_cubed);  // `True` is a YAML boolean, as the scene loader reads it
  ASSERT_EQ(m.alpha.size(), 3u); // ... so the rest of the inline seed is not thrown away
  EXPECT_NEAR(m.alpha[1].a, 0.25, 1e-6);
  ASSERT_EQ(m.isocontours.size(), 2u);
  EXPECT_EQ(m.isocontours[0].id, "01"); // a string, not the number 1
  EXPECT_EQ(m.isocontours[1].id, "7");
  EXPECT_TRUE(m.domain.automatic);
  EXPECT_TRUE(mb.last_tf_in.interactive);
  EXPECT_TRUE(mb.last_tf_in.view.show_color);
  EXPECT_FALSE(mb.last_tf_in.view.show_opacity);
  EXPECT_TRUE(mb.last_tf_in.view.show_isocontours); // not a boolean: the default stays
  EXPECT_TRUE(mb.last_tf_in.view.log_histogram);
  EXPECT_TRUE(mb.last_tf_in.view.show_info);
  const std::vector<std::string> warns = rt.take_reactive_warnings();
  EXPECT_TRUE(warns.empty()) << warns.front();
}

// A commit that leaves the model as it is writes nothing and fires nothing, even when the key holds
// a valid spelling that to_string does not reproduce (the roadmap's map nodes, "{}", a script's).
TEST(AriadneTfEditor, ANoOpCommitLeavesANonCanonicalKeyAlone) {
  cvc::app app;
  Runtime rt(app, "ui");
  MockBackend mb;
  rt.set_backend(&mb);
  mb.tf_ret.drawn = true;
  Widget w = tf_editor("TF", "tf");
  w.on = "tf_changed";
  int fired = 0;
  rt.on("tf_changed", [&] { ++fired; });
  const std::string map_form = R"({"color":[{"at":0,"rgb":[0,0,0]},{"at":1,"rgb":[1,1,1]}],)"
                               R"("alpha":[{"at":0,"a":0},{"at":1,"a":1}]})";
  tf_set(app, "ui.tf", map_form);
  rt.set_root(group({w}));
  rt.render();
  // The backend commits exactly what it was shown (ImGui's Reset or Presets > the current one).
  mb.tf_ret.changed = mb.tf_ret.committed = true;
  mb.tf_ret.model = mb.last_tf_model;
  rt.render();
  rt.drain();
  EXPECT_EQ(tf_val(app, "ui.tf"), map_form);
  EXPECT_EQ(fired, 0);
  // A real change still writes the canonical string and fires once.
  const cvc::tf::model hot = cvc::tf::model::preset("hot");
  mb.tf_ret.model = hot;
  rt.render();
  rt.drain();
  EXPECT_EQ(tf_val(app, "ui.tf"), cvc::tf::to_string(hot));
  EXPECT_EQ(fired, 1);
}

TEST(AriadneTfEditor, ComposedResetOfTheDefaultModelWritesNothing) {
  cvc::app app;
  Runtime rt(app, "ui");
  MockBackend mb; // no native editor: the core composes one
  rt.set_backend(&mb);
  Widget w = tf_editor("TF", "tf");
  w.on = "tf_changed";
  int fired = 0;
  rt.on("tf_changed", [&] { ++fired; });
  tf_set(app, "ui.tf", "{}"); // the default model, spelled by a script
  rt.set_root(group({w}));
  rt.render();
  mb.click_button = "reset";
  rt.render();
  mb.click_button.clear();
  rt.drain();
  EXPECT_EQ(tf_val(app, "ui.tf"), "{}");
  EXPECT_EQ(fired, 0);
}

// ── AppRuntime: host menus and the scene between documents ──────────────────────────────────────

namespace {

// Counts the adapter calls AppRuntime::load makes, in order.
struct LogSceneAdapter : SceneAdapter {
  std::vector<std::string> log;
  void realize(const Scene &, const std::string &, std::vector<std::string> *) override {
    log.push_back("realize");
  }
  void clear() override { log.push_back("clear"); }
  void sync_visibility(cvc::app &) override {}
  void tick() override {}
};

std::string write_doc(const std::string &name, const std::string &text) {
  const std::filesystem::path p = std::filesystem::temp_directory_path() / name;
  std::ofstream(p) << text;
  return p.string();
}

// The positions of the begin_menu calls in the walk, by label (-1: never begun).
long menu_at(const MockBackend &mb, const std::string &label) {
  const auto it = std::find(mb.log.begin(), mb.log.end(), "begin_menu:" + label);
  return it == mb.log.end() ? -1 : static_cast<long>(it - mb.log.begin());
}

} // namespace

// A host's menus (a gallery's Demos) go first in every document's menubar, and give a document
// without one a menubar of their own; set_root gets them too.
TEST(AriadneAppRuntime, HostMenusLeadEveryDocumentsMenubar) {
  if (!have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  cvc::app app;
  MockBackend mb;
  AppRuntime art(app, "ui.docs.d");
  art.set_backend(mb);
  art.set_host_menus({menu("Demos", {menu_action("other", "gallery:1")})});

  art.load(write_doc("host_menus_a.ari", "meta: {min_libcvc: 3.4.0}\n"
                                         "menubar:\n"
                                         "  - menu: File\n"
                                         "    items:\n"
                                         "      - menu_item: Quit\n"
                                         "        on: quit\n"));
  art.runtime().render();
  ASSERT_GE(menu_at(mb, "Demos"), 0);
  EXPECT_LT(menu_at(mb, "Demos"), menu_at(mb, "File"));
  EXPECT_EQ(std::count(mb.log.begin(), mb.log.end(), "begin_menubar"), 1); // one bar, not two

  mb.log.clear();
  art.load(write_doc("host_menus_b.ari", "meta: {min_libcvc: 3.4.0}\n"
                                         "root:\n"
                                         "  - text: no menubar here\n"));
  art.runtime().render();
  EXPECT_TRUE(mb.saw("begin_menubar"));
  EXPECT_GE(menu_at(mb, "Demos"), 0);
  EXPECT_TRUE(mb.saw("menu_action:other"));

  mb.log.clear();
  art.set_root(group({text("built in C++")}));
  art.runtime().render();
  EXPECT_GE(menu_at(mb, "Demos"), 0);
}

// With document_menus("Document") among the host's menus the bar is the host's and the same for
// every document: the document's menus are submenus of Document, which is greyed out (not dropped)
// for a document without any.
TEST(AriadneAppRuntime, HostMenuBarStaysTheSameAcrossDocuments) {
  if (!have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  cvc::app app;
  MockBackend mb;
  AppRuntime art(app, "ui.docs.d");
  art.set_backend(mb);
  art.set_host_menus({menu("Demos", {menu_action("other", "gallery:1")}),
                      AppRuntime::document_menus("Document"),
                      menu("Settings", {menu_action("Grid", "grid")})});
  art.load(write_doc("host_bar_a.ari", "meta: {min_libcvc: 3.4.0}\n"
                                       "menubar:\n"
                                       "  - menu: File\n"
                                       "    items:\n"
                                       "      - menu_item: Quit\n"
                                       "        on: quit\n"
                                       "  - menu: Gallery\n"
                                       "    items:\n"
                                       "      - menu_item: Next\n"
                                       "        on: next\n"));
  art.runtime().render();
  const std::vector<std::string> top = {"Demos", "Document", "Settings"};
  ASSERT_GE(menu_at(mb, "Demos"), 0);
  EXPECT_LT(menu_at(mb, "Demos"), menu_at(mb, "Document"));
  EXPECT_LT(menu_at(mb, "Document"), menu_at(mb, "File")); // inside Document, in order
  EXPECT_LT(menu_at(mb, "File"), menu_at(mb, "Gallery"));
  EXPECT_LT(menu_at(mb, "Gallery"), menu_at(mb, "Settings"));
  EXPECT_FALSE(mb.saw("begin_disabled"));

  mb.log.clear();
  art.load(write_doc("host_bar_b.ari", "meta: {min_libcvc: 3.4.0}\n"
                                       "root:\n"
                                       "  - text: no menubar here\n"));
  art.runtime().render();
  for (const std::string &m : top)
    EXPECT_GE(menu_at(mb, m), 0) << m;
  EXPECT_EQ(menu_at(mb, "File"), -1);
  const auto dis = std::find(mb.log.begin(), mb.log.end(), "begin_disabled");
  ASSERT_NE(dis, mb.log.end());
  EXPECT_EQ(*(dis + 1), "begin_menu:Document"); // the empty Document menu, greyed out
}

// With an unlabelled document_menus() the document's menus stand in the bar as written, among the
// host's; a host menu named like one of them adds its items to it after a separator.
TEST(AriadneAppRuntime, HostMenusBesideTheDocumentsOwn) {
  if (!have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  cvc::app app;
  MockBackend mb;
  AppRuntime art(app, "ui.docs.d");
  art.set_backend(mb);
  art.set_host_menus(
      {AppRuntime::document_menus(), menu("Settings", {menu_action("Grid", "grid")})});
  art.load(write_doc("host_beside_a.ari", "meta: {min_libcvc: 3.4.0}\n"
                                          "menubar:\n"
                                          "  - menu: Sim\n"
                                          "    items:\n"
                                          "      - menu_item: Quit\n"
                                          "        on: quit\n"
                                          "  - menu: Settings\n"
                                          "    items:\n"
                                          "      - menu_item: Panel\n"
                                          "        on: panel\n"));
  art.runtime().render();
  EXPECT_LT(menu_at(mb, "Sim"), menu_at(mb, "Settings"));
  EXPECT_EQ(std::count(mb.log.begin(), mb.log.end(), "begin_menu:Settings"), 1);
  const auto at = [&](const std::string &e) {
    return std::find(mb.log.begin(), mb.log.end(), e) - mb.log.begin();
  };
  EXPECT_LT(at("menu_action:Panel"), at("separator"));
  EXPECT_LT(at("separator"), at("menu_action:Grid"));

  mb.log.clear();
  art.load(write_doc("host_beside_b.ari", "meta: {min_libcvc: 3.4.0}\n"
                                          "menubar:\n"
                                          "  - menu: Sim\n"
                                          "    items:\n"
                                          "      - menu_item: Quit\n"
                                          "        on: quit\n"));
  art.runtime().render();
  EXPECT_LT(menu_at(mb, "Sim"), menu_at(mb, "Settings")); // the host's own, after the document's
  EXPECT_FALSE(mb.saw("separator"));
}

// §5.9: a host's menus count as the document's for the Windows menu. arigl's Settings > Keys...
// opens the Keys window, so the Windows menu, after the host's menus, lists only the document's.
TEST(AriadneAppRuntime, TheWindowsMenuLeavesOutWhatTheHostsMenusOpen) {
  if (!have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  cvc::app app;
  MockBackend mb;
  AppRuntime art(app, "ui.docs.d");
  art.set_backend(mb);
  art.set_host_menus({AppRuntime::document_menus(),
                      menu("Settings", {menu_toggle("Keys...", "tree.ariadne_keys.open", false)})});
  art.set_host_keys({});
  art.load(write_doc("host_windows_menu.ari", "meta: {min_libcvc: 3.4.0}\n"
                                              "windows:\n"
                                              "  - window: Panel\n"
                                              "    closable: true\n"
                                              "    children:\n"
                                              "      - text: body\n"));
  art.runtime().render();
  ASSERT_GE(menu_at(mb, "Windows"), 0);
  EXPECT_LT(menu_at(mb, "Settings"), menu_at(mb, "Windows"));
  EXPECT_EQ(std::count(mb.log.begin(), mb.log.end(), "begin_menubar"), 1); // one bar, the host's
  EXPECT_TRUE(mb.saw("toggle:Panel"));
  EXPECT_FALSE(mb.saw("toggle:Keys")) << "Settings > Keys... opens it";

  // Without such a menu the Keys window is listed: the one way to open it.
  cvc::app app2;
  MockBackend mb2;
  AppRuntime plain(app2, "ui.docs.d");
  plain.set_backend(mb2);
  plain.set_host_keys({});
  plain.load(write_doc("host_windows_menu.ari", "meta: {min_libcvc: 3.4.0}\n"
                                                "windows:\n"
                                                "  - window: Panel\n"
                                                "    closable: true\n"
                                                "    children:\n"
                                                "      - text: body\n"));
  plain.runtime().render();
  EXPECT_TRUE(mb2.saw("toggle:Panel"));
  EXPECT_TRUE(mb2.saw("toggle:Keys"));
  EXPECT_FALSE(mb2.saw("begin_window:Keys(closable)")) << "listed, but still closed";
}

// Every document a host mounts after set_host_keys gets a Keys window: closed until
// tree.ariadne_keys.open is set, a row per binding (the document's keys:, then the host's) whose
// Change button names it in tree.ariadne_keys.capture, and each binding's default keys seeded.
// With no keys at all it says so.
TEST(AriadneAppRuntime, KeysWindowListsTheDocumentsAndTheHostsKeys) {
  if (!have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  cvc::app app;
  MockBackend mb;
  AppRuntime art(app, "ui.docs.d");
  art.set_backend(mb);
  art.set_host_keys({{"camera_forward", "Forward", "Camera", {}, "", "/cam.keys.forward"}});
  cvc::state::instance(app)("cam.keys.forward").value(std::string("w"));
  art.load(write_doc("keys_window_a.ari", "meta: {min_libcvc: 3.4.0}\n"
                                          "keys:\n"
                                          "  next:\n"
                                          "    label: Next item\n"
                                          "    key: [Right, PageDown]\n"
                                          "    on: (state-set \"n\" 1)\n"));
  auto &st = cvc::state::instance(app);
  EXPECT_EQ(st("ui.docs.d.keys.next").value(), "Right, PageDown");
  art.runtime().render();
  EXPECT_FALSE(mb.saw("begin_window:Keys(closable)")) << "closed until asked for";

  st("ui.docs.d.tree.ariadne_keys.open").value(1);
  mb.log.clear();
  art.runtime().render();
  ASSERT_TRUE(mb.saw("begin_window:Keys(closable)"));
  EXPECT_TRUE(mb.saw("text_line:Next item"));
  EXPECT_TRUE(mb.saw("text_value:=Right, PageDown"));
  EXPECT_TRUE(mb.saw("text_line:Camera"));
  EXPECT_TRUE(mb.saw("text_line:Forward"));
  EXPECT_TRUE(mb.saw("text_value:=w"));
  EXPECT_TRUE(mb.saw("push_id:key_next"));
  EXPECT_TRUE(mb.saw("push_id:key_camera_forward"));
  EXPECT_EQ(std::count(mb.log.begin(), mb.log.end(), "button:Change"), 2);

  // Change names its binding in the capture -- the key post_input then reads.
  mb.button_click = true; // the mock clicks every button: the last row's wins
  art.runtime().render();
  mb.button_click = false;
  art.drain();
  EXPECT_EQ(st("ui.docs.d.tree.ariadne_keys.capture").value(), "camera_forward");
  EXPECT_TRUE(art.runtime().key_capture_pending());
  art.drain(); // the row shows the wait from the next drain on (a bound text: nothing to evaluate)
  mb.log.clear();
  art.runtime().render();
  EXPECT_TRUE(mb.saw(std::string("text_value:=") + kKeyPrompt));
  EXPECT_TRUE(mb.saw("text_value:=Right, PageDown")) << "the other rows still show their keys";
  InputEvent up;
  up.kind = InputEvent::Kind::KeyDown;
  up.key = "Up";
  art.runtime().post_input(up);
  art.drain();
  EXPECT_EQ(st("cam.keys.forward").value(), "Up") << "the next key down is the binding's key";
  EXPECT_EQ(st("ui.docs.d.tree.ariadne_keys.capture").value(), "");
  mb.log.clear();
  art.runtime().render();
  EXPECT_TRUE(mb.saw("text_value:=Up")) << "and its row shows it";
  EXPECT_FALSE(mb.saw(std::string("text_value:=") + kKeyPrompt));

  art.set_host_keys({});
  mb.log.clear();
  art.load(write_doc("keys_window_b.ari", "meta: {min_libcvc: 3.4.0}\n"
                                          "root:\n"
                                          "  - text: no keys here\n"));
  art.runtime().render();
  EXPECT_TRUE(mb.saw("begin_window:Keys(closable)")) << "still open, and still there";
  EXPECT_TRUE(mb.saw("text_line:No keys to change here."));
}

// Documents a host loads in turn share the prefix, so each new document's key defaults replace an
// earlier one's for the same binding id, where a reload of the same file (under any spelling)
// keeps the keys the user chose. A new document's windows are laid out afresh: the backend is told
// (reset_layout), and each window's first frame takes no geometry persisted by the last document
// -- only its own authored pos, marked as authored.
TEST(AriadneAppRuntime, EachNewDocumentHasItsOwnKeyDefaultsAndLayout) {
  if (!have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  cvc::app app;
  MockBackend mb;
  AppRuntime art(app, "ui.docs.e");
  art.set_backend(mb);
  auto &st = cvc::state::instance(app);
  const std::string a = write_doc("own_keys_a.ari", "meta: {min_libcvc: 3.4.0}\n"
                                                    "keys:\n"
                                                    "  next:\n"
                                                    "    key: Right\n"
                                                    "    on: (state-set \"n\" 1)\n"
                                                    "  toggle:\n"
                                                    "    key: T\n"
                                                    "    on: (state-set \"t\" 1)\n"
                                                    "windows:\n"
                                                    "  - window: Panel\n"
                                                    "    children:\n"
                                                    "      - text: a\n");
  art.load(a);
  EXPECT_EQ(st("ui.docs.e.keys.next").value(), "Right");
  art.runtime().render();
  EXPECT_EQ(mb.layout_resets, 0) << "the first document has nothing to forget";
  st("ui.docs.e.keys.next").value(std::string("Up")); // the user's key

  const std::filesystem::path pa(a);
  art.load((pa.parent_path() / "." / pa.filename()).string()); // a reload, spelled otherwise
  EXPECT_EQ(st("ui.docs.e.keys.next").value(), "Up") << "a reload keeps the user's key";
  art.runtime().render();
  EXPECT_EQ(mb.layout_resets, 0) << "and the windows where the user put them";

  st("ui.docs.e.tree.Panel.geometry").value(std::string("300,400,200,100")); // a's Panel
  st("ui.docs.e.tree.Split.tracks").value(std::string("40,50"));             // a's Split
  art.load(write_doc("own_keys_b.ari", "meta: {min_libcvc: 3.4.0}\n"
                                       "keys:\n"
                                       "  next:\n"
                                       "    key: Space\n"
                                       "    on: (state-set \"n\" 2)\n"
                                       "  toggle:\n"
                                       "    label: Toggle\n"
                                       "    on: (state-set \"t\" 2)\n"
                                       "windows:\n"
                                       "  - window: Panel\n"
                                       "    pos: [5, 6]\n"
                                       "    children:\n"
                                       "      - text: b\n"
                                       "  - window: Split\n"
                                       "    layout:\n"
                                       "      kind: grid\n"
                                       "      row_heights: [90, auto]\n"
                                       "      resizable: true\n"
                                       "    children:\n"
                                       "      - text: top\n"
                                       "      - text: bottom\n"));
  EXPECT_EQ(st("ui.docs.e.keys.next").value(), "Space") << "b's default, not a's key";
  EXPECT_EQ(st("ui.docs.e.keys.toggle").value(), "") << "b's toggle has no key, not a's T";
  mb.seeded_geom_called = false;
  mb.seeded_tracks_called = false;
  art.runtime().render();
  EXPECT_FALSE(mb.seeded_tracks_called) << "a's Split's dragged panes are not b's";
  EXPECT_EQ(mb.layout_resets, 1);
  ASSERT_TRUE(mb.seeded_geom_called);
  EXPECT_TRUE(mb.seeded_geom.authored) << "b's own pos, not a's persisted Panel";
  EXPECT_FLOAT_EQ(mb.seeded_geom.x, 5.0f);
  EXPECT_FLOAT_EQ(mb.seeded_geom.y, 6.0f);
  mb.seeded_geom = WindowGeom{};
  art.runtime().render();
  EXPECT_EQ(mb.layout_resets, 1) << "once";
  EXPECT_FALSE(mb.seeded_geom.authored) << "then what is persisted, as before";
  EXPECT_FLOAT_EQ(mb.seeded_geom.x, 300.0f);
}

// A document's init: runs after its keys: defaults are in, so it can set a key of its own (from
// the command line, say) -- on its first load as on any other.
TEST(AriadneAppRuntime, InitCanSetAKeyOverItsDefault) {
  if (!have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  cvc::app app;
  MockBackend mb;
  AppRuntime art(app, "ui.docs.i");
  art.set_backend(mb);
  art.load(write_doc("init_key.ari", "meta: {min_libcvc: 3.4.0}\n"
                                     "init: (state-set \"keys.next\" \"Space\")\n"
                                     "keys:\n"
                                     "  next:\n"
                                     "    key: Right\n"
                                     "    on: (state-set \"n\" 1)\n"));
  EXPECT_EQ(cvc::state::instance(app)("ui.docs.i.keys.next").value(), "Space");
}

// Every load clears the last document's scene before anything is realized -- a host switching
// documents, or a reload -- and does so even when the new document declares no scene.
TEST(AriadneAppRuntime, EachLoadClearsTheLastScene) {
  if (!have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  cvc::app app;
  MockBackend mb;
  LogSceneAdapter scene;
  AppRuntime art(app, "ui.docs.s", &scene);
  art.set_backend(mb);
  const std::string with_scene =
      write_doc("clear_scene_a.ari", "meta: {min_libcvc: 3.4.0}\n"
                                     "scene:\n"
                                     "  nodes:\n"
                                     "    - node: box\n"
                                     "      source: { file: stanford.bunny }\n");
  art.load(with_scene);
  art.load(with_scene);
  art.load(write_doc("clear_scene_b.ari", "meta: {min_libcvc: 3.4.0}\nroot:\n  - text: x\n"));
  EXPECT_EQ(scene.log, (std::vector<std::string>{"clear", "realize", "clear", "realize", "clear"}));
}
