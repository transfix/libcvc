// cvcgl_ariadne_viewpoints — authored viewpoints (docs/ARIADNE.md §5.18, Cameras): a scene camera's
// `position:` and `target:`, its lens (`fov:`, `near:`, `far:`) and `orthographic:` with
// `height:`. A `cameras:` entry named `main` sets where the main view starts, a named camera with
// orthographic: true draws a parallel view as tall as its height, and a viewport's own camera takes
// the same keys. Offscreen; skips where nothing rasterises unless CVC_REQUIRE_RENDER=1.

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
#include <filesystem>
#include <fstream>
#include <string>
#include <vtkCamera.h>
#include <vtkRenderer.h>

namespace fs = std::filesystem;
using cvcgl_test::check;

namespace {

constexpr int kWidth = 960, kHeight = 720;
constexpr double kDt = 1.0 / 30.0;

const char *kDoc = R"(meta:
  name: viewpoints test
  min_libcvc: 3.7.0

windows:
  - window: Views
    id: views_window
    pos: [20, 20]
    size: [440, 680]
    children:
      - type: viewport
        view: aerial
        camera: plan
        width: 400
        height: 300
      - type: viewport
        view: street
        position: [0, -30, 2]
        target: [0, 0, 1]
        fov: 60
        width: 400
        height: 300

scene:
  cameras:
    - camera: main
      position: [12, -9, 7]
      target: [1, 1, 0]
      fov: 35
      near: 0.25
      far: 400
    - camera: plan
      orthographic: true
      position: [5, -20, 30]
      target: [0, 0, 0]
      height: 16
  nodes:
    - node: crate
      source: { box: { size: [2, 2, 2] } }
    - node: post
      source: { box: 1 }
      transform: { position: [6, 0, 0] }
)";

bool at3(const double a[3], double x, double y, double z, double tol) {
  return std::fabs(a[0] - x) < tol && std::fabs(a[1] - y) < tol && std::fabs(a[2] - z) < tol;
}
std::string fmt3(const double a[3]) {
  char b[96];
  std::snprintf(b, sizeof b, "(%.6f, %.6f, %.6f)", a[0], a[1], a[2]);
  return b;
}

} // namespace

int main() {
  const bool require = std::getenv("CVC_REQUIRE_RENDER") != nullptr;
#ifndef CVC_ENABLE_IMGUI
  std::printf("SKIP: no ImGui\n");
  return require ? 1 : 0;
#else
  const fs::path dir = fs::temp_directory_path() / "cvcgl_ariadne_viewpoints";
  fs::create_directories(dir);
  const fs::path doc = dir / "viewpoints.ari";
  std::ofstream(doc) << kDoc;

  cvc::app app;
  app.properties("system.log_verbosity", "0");
  cvc::gl::SceneGraph sg(app, "vp");
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

  // The main view starts where `camera: main` says.
  vtkCamera *mc = view.renderer()->GetActiveCamera();
  double p[3], f[3], r[2];
  mc->GetPosition(p);
  mc->GetFocalPoint(f);
  mc->GetClippingRange(r);
  check(at3(p, 12.0, -9.0, 7.0, 1e-6), "the main view's eye is camera main's position", fmt3(p));
  check(at3(f, 1.0, 1.0, 0.0, 1e-6), "looking at its target", fmt3(f));
  check(std::fabs(mc->GetViewAngle() - 35.0) < 1e-9, "with its field of view",
        std::to_string(mc->GetViewAngle()));
  check(std::fabs(r[0] - 0.25) < 1e-9 && std::fabs(r[1] - 400.0) < 1e-9,
        "and its near and far planes", std::to_string(r[0]) + " .. " + std::to_string(r[1]));
  check(!mc->GetParallelProjection(), "in perspective");

  cvc::gl::ViewportManager &vm = view.viewportManager();
  check(vm.hasViewport("aerial") && vm.hasViewport("street"), "both viewports exist");
  if (!vm.hasViewport("aerial") || !vm.hasViewport("street"))
    return cvcgl_test::finish("cvcgl_ariadne_viewpoints");

  // A named orthographic camera: parallel, `height` tall, looking from its position's side.
  vtkCamera *ac = vm.viewport("aerial").renderer()->GetActiveCamera();
  ac->GetPosition(p);
  ac->GetFocalPoint(f);
  check(ac->GetParallelProjection() != 0, "the plan camera is orthographic");
  check(std::fabs(ac->GetParallelScale() - 8.0) < 1e-6, "16 tall (a parallel scale of 8)",
        std::to_string(ac->GetParallelScale()));
  check(at3(f, 0.0, 0.0, 0.0, 1e-6), "looking at its target", fmt3(f));
  const double len = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
  const double want = std::sqrt(5.0 * 5.0 + 20.0 * 20.0 + 30.0 * 30.0);
  check(len > 0.0 && at3(p, 5.0 * len / want, -20.0 * len / want, 30.0 * len / want, 1e-6),
        "from its position's direction", fmt3(p));

  // A viewport's own camera takes the same keys.
  vtkCamera *sc = vm.viewport("street").renderer()->GetActiveCamera();
  sc->GetPosition(p);
  sc->GetFocalPoint(f);
  check(at3(p, 0.0, -30.0, 2.0, 1e-6), "the street view's own camera is at its position", fmt3(p));
  check(at3(f, 0.0, 0.0, 1.0, 1e-6), "looking at its target", fmt3(f));
  check(std::fabs(sc->GetViewAngle() - 60.0) < 1e-9, "with its field of view",
        std::to_string(sc->GetViewAngle()));
  check(!sc->GetParallelProjection(), "in perspective");

  // They hold over frames.
  for (int f2 = 0; f2 < 10; ++f2)
    rt.step(kDt);
  mc->GetPosition(p);
  sc->GetPosition(f);
  check(at3(p, 12.0, -9.0, 7.0, 1e-6) && at3(f, 0.0, -30.0, 2.0, 1e-6), "and they hold over frames",
        fmt3(p) + " " + fmt3(f));

  view.close(); // ~AriRuntime runs after the return below, with the view closed
  return cvcgl_test::finish("cvcgl_ariadne_viewpoints");
#endif
}
