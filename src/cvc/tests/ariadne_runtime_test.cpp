// Tests for the Ariadne Runtime walk (cvc::ariadne::Runtime) via a recording mock
// Backend — headless, no ImGui/FTXUI. Covers the emit walk, the reconcile boundary,
// the deferred action drain, cvc::state read/seed/commit, bind-path resolution, and
// the §3.0.3b grid dispatch.

#include <algorithm>
#include <atomic> // item 2: the nav-step worker's arrived-count accumulator
#include <boost/any.hpp>
#include <chrono>
#include <condition_variable>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/loader.h>         // §12 end-to-end load: mount through the real Runtime
#include <cvc/ariadne/net_intrinsics.h> // §13.8 async (http-get-async) intrinsic test
#include <cvc/ariadne/uri.h>            // §13.8 (fetch uri): register a custom scheme handler
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/state_exec/async_scheduler.h> // exec_scheduler().post_message end-to-end test
#include <cvc/core/state_exec/builtins.h>        // host-intrinsic seam test: register_fn
#include <cvc/core/state_exec/types.h>           // value_t
#include <cvc/core/thread_pool.h> // item 2: full type for app.computePool().parallel_for
#include <cvc/net/http_client.h>  // §13.8: swap the transport for a fake in the http-get test
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <memory>
#include <mutex>
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
  bool button(const char *l) override {
    rec(std::string("button:") + l);
    return button_click;
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
  std::vector<float> seeded_tracks;
  bool seeded_tracks_called = false;
  std::vector<float> grid_tracks_ret;
  void seed_grid_tracks(const std::vector<float> &sizes) override {
    seeded_tracks = sizes;
    seeded_tracks_called = true;
    rec("seed_grid_tracks");
  }
  std::vector<float> grid_tracks() const override { return grid_tracks_ret; }
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
  if (!have_state_exec())
    GTEST_SKIP() << "libcvc built without state_exec (CVC_STATE_EXEC=OFF)";
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
// NB the nav_compute demo drives this through app.compute_async (which wraps exactly this
// background-thread → parallel_for → post_message); here we hand-roll a joinable std::thread so the
// test can join() deterministically before asserting (compute_async is fire-and-forget by design).
TEST(AriadneAction, IntrinsicRunsNavStepOnComputePoolAndWakesMsgRecv) {
  if (!have_state_exec())
    GTEST_SKIP() << "libcvc built without state_exec (CVC_STATE_EXEC=OFF)";
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
  if (!have_state_exec())
    GTEST_SKIP() << "libcvc built without state_exec (CVC_STATE_EXEC=OFF)";
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

// set_tick_program("") clears the resident: after clearing, a drain no longer re-fires the body.
TEST(AriadneResident, ClearingTickProgramStopsIt) {
  if (!have_state_exec())
    GTEST_SKIP();
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

// §4.6 input seam: set_key_program submits a resident that receives events posted via post_input,
// reading the delivered event dict with (get-attr event ...). post_input BEFORE drain -> delivered
// this frame (feed-before-drain contract). Keyboard events feed the key resident.
TEST(AriadneInput, KeyProgramReceivesPostedEvent) {
  if (!have_state_exec())
    GTEST_SKIP() << "libcvc built without state_exec (CVC_STATE_EXEC=OFF)";
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
  if (!have_state_exec())
    GTEST_SKIP();
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
  if (!have_state_exec())
    GTEST_SKIP();
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

// §4.6 widget-level on_click: any widget can carry an on_click program; the walk enqueues it when
// the backend reports the item was clicked (Backend::item_clicked -> ImGui::IsItemClicked natively
// and on wasm). Fired like a Button's on: (queued in render(), run in drain()) — NOT the SDL
// document-level stream. This is the natively-working widget input path.
TEST(AriadneInput, WidgetOnClickFiresWhenBackendReportsClick) {
  if (!have_state_exec())
    GTEST_SKIP();
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
  if (!have_state_exec())
    GTEST_SKIP();
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

// PR-C render-pass park safety: a park verb (msg-recv/await/http-get/fetch) in a reactive predicate
// is NOT in the read-lane allowlist, so it is DENIED — the render walk cannot park (its private
// scheduler is never pumped; a park there would silent-nil). render() must return, hidden +
// reported.
TEST(AriadneReactive, ParkVerbInPredicateIsDeniedNotHung) {
  if (!have_state_exec())
    GTEST_SKIP();
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
  if (!have_state_exec())
    GTEST_SKIP();
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
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_channel_policy({"nav.done"}, {}, /*strict=*/true, /*quiet=*/false);
  EXPECT_FALSE(
      msg_delivered(app, rt, mb, "other", "other")); // undeclared -> refused, not delivered
}

TEST(AriadneChannelPolicy, AllowsDeclaredChannel) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_channel_policy({"nav.done"}, {}, true, false);
  EXPECT_TRUE(msg_delivered(app, rt, mb, "nav.done", "nav.done")); // declared -> allowed, delivered
}

TEST(AriadneChannelPolicy, PermissiveWhenNotStrict) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_channel_policy({}, {}, /*strict=*/false, false); // warn/off -> no runtime enforcement
  EXPECT_TRUE(msg_delivered(app, rt, mb, "anything", "anything"));
}

TEST(AriadneChannelPolicy, AllowsDeclaredGlobal) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_channel_policy({}, {"app.quit"}, true, false);
  EXPECT_TRUE(
      msg_delivered(app, rt, mb, "/app.quit", "app.quit")); // declared global (key strips /)
}

TEST(AriadneChannelPolicy, RefusesUndeclaredGlobal) {
  if (!have_state_exec())
    GTEST_SKIP();
  cvc::app app;
  Runtime rt(app, "");
  MockBackend mb;
  rt.set_backend(&mb);
  rt.set_channel_policy({}, {"app.quit"}, true, false);
  EXPECT_FALSE(
      msg_delivered(app, rt, mb, "/other.g", "other.g")); // not a declared global -> refused
}

TEST(AriadneChannelPolicy, ExemptsHashChannel) {
  if (!have_state_exec())
    GTEST_SKIP();
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

std::string node_data_string(cvc::app &app, const char *path) {
  const boost::any d = cvc::state::instance(app)(path).data();
  const std::string *s = boost::any_cast<std::string>(&d);
  return s ? *s : std::string();
}

} // namespace

TEST(AriadneNetIntrinsics, HttpGetAsyncAwaitsResponseDict) {
  if (!have_state_exec())
    GTEST_SKIP() << "libcvc built without state_exec";
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
  if (!have_state_exec())
    GTEST_SKIP() << "libcvc built without state_exec";
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
  if (!have_state_exec())
    GTEST_SKIP() << "libcvc built without state_exec";
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
  if (!have_state_exec())
    GTEST_SKIP() << "libcvc built without state_exec";
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
  if (!have_state_exec())
    GTEST_SKIP() << "libcvc built without state_exec";
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
  if (!have_state_exec())
    GTEST_SKIP() << "libcvc built without state_exec";
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
  if (!have_state_exec())
    GTEST_SKIP() << "libcvc built without state_exec";
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
  if (!have_state_exec())
    GTEST_SKIP() << "libcvc built without state_exec";
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
  if (!have_state_exec())
    GTEST_SKIP() << "libcvc built without state_exec";
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
  if (!have_state_exec())
    GTEST_SKIP() << "libcvc built without state_exec";
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
  if (!have_state_exec())
    GTEST_SKIP() << "libcvc built without state_exec";
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
