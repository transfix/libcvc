// cvcgl_ariadne_pick — click-to-world picking in an Ariadne `viewport` (scene_views.h; L0 of
// docs/roadmap/ARIADNE_WORLDS.md): an `input: pick` view's left button picks what the view draws
// instead of turning its camera, writes where it landed under its `pick:` key and raises its
// `on_pick:` with the same fields; SceneViews::pick is the same hit test for hosts. Offscreen;
// skips where nothing rasterises unless CVC_REQUIRE_RENDER=1.

#include "streaming_test_util.h"

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

scene:
  nodes:
    - node: crate
      source: { box: { size: [2, 2, 2] } }
      transform: { position: [-6, 0, 0] }
    - node: post
      source: { box: { size: [1, 1, 1] } }
      transform: { position: [6, 0, 0] }
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
  double r[4];
  picker.region(r);
  // The view frames both boxes: the crate's centre on screen (VTK display coordinates, and ImGui's
  // with y down), and the view's centre, between the two, where nothing is.
  vtkRenderer *ren = picker.renderer();
  ren->SetWorldPoint(-6.0, 0.0, 0.0, 1.0);
  ren->WorldToDisplay();
  double d[3];
  ren->GetDisplayPoint(d);
  const double cx = d[0], cy = d[1];
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
  for (int i = 1; i <= 3; ++i) {
    io.AddMousePosEvent(mx + 6.0f * i, my);
    rt.step(kDt);
  }
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

  view.close(); // ~AriRuntime runs after the return below, with the view closed
  return cvcgl_test::finish("cvcgl_ariadne_pick");
#endif
}
