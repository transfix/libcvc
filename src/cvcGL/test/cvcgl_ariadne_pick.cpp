// cvcgl_ariadne_pick — click-to-world picking in an Ariadne `viewport` (scene_views.h; L0 of
// docs/roadmap/ARIADNE_WORLDS.md): an `input: pick` view's left button picks what the view draws
// instead of turning its camera, writes where it landed under its `pick:` key and raises its
// `on_pick:` with the same fields; SceneViews::pick is the same hit test for hosts. The pick view
// is throttled (`refresh_hz: 2`): it draws its scene less often than the orbit view beside it while
// nothing happens in it, every frame while it is dragged, and its picks test the scene, not its
// last pixels. Offscreen; skips where nothing rasterises unless CVC_REQUIRE_RENDER=1.

#include "streaming_test_util.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/Viewport.h>
#include <cvc/gl/ViewportManager.h>
#include <cvc/gl/ariadne/AriRuntime.h>
#include <cvc/gl/ariadne/scene_views.h>
#include <cvc/state/state.h>
#include <filesystem>
#include <fstream>
#include <string>
#include <vtkRenderer.h>
#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
#endif

namespace fs = std::filesystem;
using cvcgl_test::check;

namespace {

constexpr int kWidth = 960, kHeight = 720;
constexpr double kDt = 1.0 / 30.0;

const char *kDoc = R"(meta:
  name: pick test
  min_libcvc: 3.7.0

windows:
  - window: Pick
    id: pick_window
    pos: [20, 20]
    size: [440, 400]
    children:
      - type: viewport
        view: picker
        input: pick
        pick: hits
        on_pick: (begin (state-set "picked" (get-attr event "node")) (state-set "picked_phase" (get-attr event "phase")))
        refresh_hz: 2
        width: 400
        height: 320
  - window: Orbit
    id: orbit_window
    pos: [500, 20]
    size: [440, 400]
    children:
      - type: viewport
        view: orbit
        width: 400
        height: 320
  - window: Module
    id: module_window
    pos: [20, 430]
    size: [440, 280]
    children:
      - load: panel.ari
        as: panel

init: (state-set "axis" "y")

scene:
  nodes:
    - node: crate
      source: { box: { size: [2, 2, 2] } }
      transform: { position: [-6, 0, 0] }
    - node: post
      source: { box: { size: [1, 1, 1] } }
      transform: { position: [6, 0, 0] }
)";

// A load:ed module's views: their pick:, pointer: and axis: { bind } keys resolve in the module's
// scope (pick.includes.panel), as their bind: would, and the default pick key too; the document's
// `axis` (y) is not the module's (x).
const char *kPanel = R"(init: (state-set "axis" "x")
root:
  - type: viewport
    view: inner
    input: pick
    pick: hits
    pointer: probe
    width: 400
    height: 110
  - type: viewport
    view: inner_slice
    mode: slice
    axis: { bind: axis }
    input: pick
    width: 400
    height: 110
)";

std::string val(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app)(path).value();
}

} // namespace

int main() {
  const bool require = std::getenv("CVC_REQUIRE_RENDER") != nullptr;
#ifndef CVC_ENABLE_IMGUI
  std::printf("SKIP: no ImGui\n");
  return require ? 1 : 0;
#else
  const fs::path dir = fs::temp_directory_path() / "cvcgl_ariadne_pick";
  fs::create_directories(dir);
  const fs::path doc = dir / "pick.ari";
  std::ofstream(doc) << kDoc;
  std::ofstream(dir / "panel.ari") << kPanel;

  cvc::app app;
  app.properties("system.log_verbosity", "0");
  cvc::gl::SceneGraph sg(app, "pick");
  sg.setDiagnosticChromeVisible(false);
  cvc::gl::SceneRenderer view(sg, kWidth, kHeight, /*offscreen=*/true, "main");
  view.setBackground(0.0, 0.0, 0.0);
  cvc::gl::CameraController cam(view);
  cvc::gl::ImGuiOverlay ui(view);
  if (!ui.enabled()) {
    std::printf("SKIP: no ImGui overlay\n");
    return require ? 1 : 0;
  }
  ui.attachCamera(cam);
  ui.setVisible(true);
  ui.setTouchMode(false);
  ui.setUiScale(1.0f);
  ui.setToggleButtonEnabled(false);
  ui.setPanelsOpen(true);
  ui.setFixedDeltaTime(static_cast<float>(kDt));

  cvc::gl::ariadne::AriRuntime rt(view, cam, ui);
  for (const std::string &w : rt.load(doc.string()))
    check(false, "the document loads without warnings", w);
  for (int f = 0; f < 6; ++f)
    rt.step(kDt);
  cvc::gl::ViewportManager &vm = view.viewportManager();
  check(vm.hasViewport("picker") && vm.hasViewport("orbit"), "both viewports exist");
  if (!vm.hasViewport("picker"))
    return cvcgl_test::finish("cvcgl_ariadne_pick");
  cvc::gl::Viewport &picker = vm.viewport("picker");
  cvc::gl::Viewport &orbitView = vm.viewport("orbit");
  check(picker.refreshHz() == 2.0 && orbitView.refreshHz() == 0.0,
        "refresh_hz: 2 throttles the pick view, and only it");
  {
    // Nothing happens in either view: the orbit view draws every frame, the pick view at <= 2 Hz.
    const long p0 = picker.draws(), o0 = orbitView.draws();
    const auto t0 = std::chrono::steady_clock::now();
    for (int f = 0; f < 8; ++f)
      rt.step(kDt);
    const double secs =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    const long pd = picker.draws() - p0, od = orbitView.draws() - o0;
    check(od == 8, "the orbit view draws every frame", std::to_string(od));
    check(pd <= 1 + static_cast<long>(secs * 2.0) && pd < od,
          "the pick view draws no more than 2 Hz allows",
          std::to_string(pd) + " draws in " + std::to_string(secs) + " s");
  }
  double r[4];
  picker.region(r);
  // The view frames both boxes: the crate's centre on screen (VTK display coordinates, and ImGui's
  // with y down), and the view's centre, between the two, where nothing is. The view's renderer
  // draws into its texture, so its display point is a texel, shown in the window at its region.
  vtkRenderer *ren = picker.renderer();
  ren->SetWorldPoint(-6.0, 0.0, 0.0, 1.0);
  ren->WorldToDisplay();
  double d[3], onScreen[2];
  ren->GetDisplayPoint(d);
  picker.rendererToWindow(d[0], d[1], onScreen);
  const double cx = onScreen[0], cy = onScreen[1];
  const float mx = static_cast<float>(cx), my = static_cast<float>(kHeight - cy);
  const double gx = 0.5 * (r[0] + r[2]) * kWidth, gy = 0.5 * (r[1] + r[3]) * kHeight;

  // SceneViews::pick: a hit on the crate, on its surface, naming its node.
  cvc::gl::ariadne::SceneViews::PickHit hit;
  const bool got = rt.views().pick("picker", cx, cy, hit);
  check(got && hit.hit, "a pick on the crate hits it");
  check(hit.node == "crate", "and names its node", hit.node);
  const double reach =
      std::max({std::fabs(hit.world[0] + 6.0), std::fabs(hit.world[1]), std::fabs(hit.world[2])});
  check(got && std::fabs(reach - 1.0) < 0.05, "the point is on the crate's surface",
        std::to_string(hit.world[0]) + " " + std::to_string(hit.world[1]) + " " +
            std::to_string(hit.world[2]));
  cvc::gl::ariadne::SceneViews::PickHit miss;
  const bool between = rt.views().pick("picker", gx, gy, miss);
  check(!between && !miss.hit, "a pick between the boxes misses",
        miss.node + " " + std::to_string(miss.world[0]) + " " + std::to_string(miss.world[1]) +
            " " + std::to_string(miss.world[2]));
  check(!rt.views().pick("nosuch", cx, cy, miss), "no such view: no pick");

  // The left button: down, a drag, up. Each writes the pick key and raises on_pick:, and the
  // camera does not turn.
  ImGui::SetCurrentContext(ui.imguiContext());
  ImGuiIO &io = ImGui::GetIO();
  double eye[3], focal[3], up[3];
  picker.camera().getPose(eye, focal, up);
  io.AddMousePosEvent(mx, my);
  rt.step(kDt);
  io.AddMouseButtonEvent(0, true);
  rt.step(kDt);
  rt.step(kDt);
  const std::string key = "pick.hits";
  check(val(app, key + ".phase") == "down", "a press writes phase down", val(app, key + ".phase"));
  check(val(app, key + ".hit") == "1" && val(app, key + ".node") == "crate", "with the crate hit",
        val(app, key + ".node"));
  check(!val(app, key + ".x").empty() && val(app, key + ".button") == "1",
        "the point and the button");
  check(val(app, "pick.picked") == "crate" && val(app, "pick.picked_phase") == "down",
        "on_pick: ran with the event's fields", val(app, "pick.picked"));
  for (const std::string &w : rt.take_warnings())
    std::printf("  warning: %s\n", w.c_str());
  const std::string seq = val(app, key + ".seq");
  const long dragDraws = picker.draws();
  for (int i = 1; i <= 3; ++i) {
    io.AddMousePosEvent(mx + 6.0f * i, my);
    rt.step(kDt);
  }
  check(picker.draws() - dragDraws == 3, "dragged, the throttled view draws every frame",
        std::to_string(picker.draws() - dragDraws));
  check(val(app, key + ".phase") == "drag" && val(app, key + ".seq") != seq,
        "a drag writes phase drag, a new seq each time");
  io.AddMouseButtonEvent(0, false);
  rt.step(kDt);
  rt.step(kDt);
  check(val(app, key + ".phase") == "up", "the release writes phase up");
  double eye2[3];
  picker.camera().getPose(eye2, focal, up);
  check(std::fabs(eye2[0] - eye[0]) + std::fabs(eye2[1] - eye[1]) + std::fabs(eye2[2] - eye[2]) <
            1e-9,
        "the left button did not turn the camera");

  // A click between the boxes is a miss: hit 0, no point.
  io.AddMousePosEvent(static_cast<float>(gx), static_cast<float>(kHeight - gy));
  rt.step(kDt);
  io.AddMouseButtonEvent(0, true);
  rt.step(kDt);
  io.AddMouseButtonEvent(0, false);
  rt.step(kDt);
  rt.step(kDt);
  check(val(app, key + ".hit") == "0" && val(app, key + ".x").empty() &&
            val(app, key + ".node").empty(),
        "a click between the boxes writes a miss");
  io.AddMousePosEvent(-1.0f, -1.0f);
  rt.step(kDt);

  // A view without input: pick still orbits under the left button, and writes no pick.
  if (vm.hasViewport("orbit")) {
    cvc::gl::Viewport &orbit = vm.viewport("orbit");
    double o[4];
    orbit.region(o);
    const float ox = static_cast<float>(0.5 * (o[0] + o[2]) * kWidth);
    const float oy = static_cast<float>(kHeight - 0.5 * (o[1] + o[3]) * kHeight);
    orbit.camera().getPose(eye, focal, up);
    io.AddMousePosEvent(ox, oy);
    rt.step(kDt);
    io.AddMouseButtonEvent(0, true);
    rt.step(kDt);
    for (int i = 1; i <= 4; ++i) {
      io.AddMousePosEvent(ox + 15.0f * i, oy);
      rt.step(kDt);
    }
    io.AddMouseButtonEvent(0, false);
    rt.step(kDt);
    orbit.camera().getPose(eye2, focal, up);
    check(std::fabs(eye2[0] - eye[0]) + std::fabs(eye2[1] - eye[1]) > 1e-6,
          "a view without input: pick still orbits");
    check(val(app, "pick.viewers.orbit.pick.seq").empty(), "and writes no pick");
    io.AddMousePosEvent(-1.0f, -1.0f);
    rt.step(kDt);
  }

  // A view in a load:ed module writes inside the module's sandbox (pick.includes.panel), where its
  // bind: would resolve, never under the document's keys.
  const std::string mod = "pick.includes.panel";
  check(vm.hasViewport("inner") && vm.hasViewport("inner_slice"), "the module's viewports exist");
  if (vm.hasViewport("inner")) {
    cvc::gl::Viewport &inner = vm.viewport("inner");
    vtkRenderer *ir = inner.renderer();
    ir->SetWorldPoint(-6.0, 0.0, 0.0, 1.0);
    ir->WorldToDisplay();
    ir->GetDisplayPoint(d);
    double onWindow[2]; // the view draws into its texture, which the window shows at its region
    inner.rendererToWindow(d[0], d[1], onWindow);
    const float ix = static_cast<float>(onWindow[0]),
                iy = static_cast<float>(kHeight - onWindow[1]);
    const std::string docSeq = val(app, key + ".seq");
    io.AddMousePosEvent(ix, iy);
    rt.step(kDt);
    rt.step(kDt);
    check(val(app, mod + ".probe.phase") == "hover" && val(app, mod + ".probe.node") == "crate",
          "pointer: probe follows the pointer under the module's prefix",
          val(app, mod + ".probe.phase") + " " + val(app, mod + ".probe.node"));
    check(val(app, "pick.probe.phase").empty(), "not under the document's");
    io.AddMouseButtonEvent(0, true);
    rt.step(kDt);
    rt.step(kDt);
    check(val(app, mod + ".hits.phase") == "down" && val(app, mod + ".hits.node") == "crate",
          "pick: hits lands at includes.panel.hits",
          val(app, mod + ".hits.phase") + " " + val(app, mod + ".hits.node"));
    io.AddMouseButtonEvent(0, false);
    rt.step(kDt);
    rt.step(kDt);
    check(val(app, mod + ".hits.phase") == "up", "and its release there");
    check(val(app, key + ".seq") == docSeq, "the document's hits are not written",
          val(app, key + ".seq"));
    io.AddMousePosEvent(-1.0f, -1.0f);
    rt.step(kDt);
  }
  if (vm.hasViewport("inner_slice")) {
    // axis: { bind: axis } reads the module's axis (x), not the document's (y); the camera is the
    // view's own, under the scene's prefix (pick.viewers.inner_slice.camera), as any view's.
    cvc::gl::Viewport &slice = vm.viewport("inner_slice");
    check(slice.camera().mode() == cvc::gl::CameraController::Mode::Slice &&
              slice.camera().sliceAxis() == 0,
          "axis: { bind: axis } reads includes.panel.axis: down x",
          std::to_string(slice.camera().sliceAxis()));
    check(val(app, "pick.viewers.inner_slice.camera.slice.axis") == "x",
          "its camera keeps its pose under the scene's viewers.inner_slice.camera",
          val(app, "pick.viewers.inner_slice.camera.slice.axis"));
    double s[4];
    slice.region(s);
    const float sx = static_cast<float>(0.5 * (s[0] + s[2]) * kWidth);
    const float sy = static_cast<float>(kHeight - 0.5 * (s[1] + s[3]) * kHeight);
    io.AddMousePosEvent(sx, sy);
    rt.step(kDt);
    io.AddMouseButtonEvent(0, true);
    rt.step(kDt);
    io.AddMouseButtonEvent(0, false);
    rt.step(kDt);
    rt.step(kDt);
    check(val(app, mod + ".viewers.inner_slice.pick.phase") == "up",
          "the default pick key is the module's: includes.panel.viewers.inner_slice.pick",
          val(app, mod + ".viewers.inner_slice.pick.phase"));
    check(val(app, "pick.viewers.inner_slice.pick.seq").empty(), "not the document's");
    io.AddMousePosEvent(-1.0f, -1.0f);
    rt.step(kDt);
  }

  view.close(); // ~AriRuntime runs after the return below, with the view closed
  return cvcgl_test::finish("cvcgl_ariadne_pick");
#endif
}
