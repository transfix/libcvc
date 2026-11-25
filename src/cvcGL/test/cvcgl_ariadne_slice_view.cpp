// cvcgl_ariadne_slice_view — a `viewport` in `mode: slice` (scene_views.h; roadmap §6 items 2 and
// 3): a parallel view down one axis over the document's volume_slice nodes.
//   - the axis follows `axis: { bind: grid.view }`: xy looks down z, xz down y, each framed on the
//     slices on its axis;
//   - the view leaves out the volume_slice nodes on the other axes (an edge-on red slice draws no
//     pixel in it);
//   - a pick meets the slice's plane exactly and writes i, j, k and value with x, y, z, through
//     the left button and SceneViews::pick, and keeps reporting the plane's point past its edge;
//   - the wheel zooms about the pointer: the world point under it stays put;
//   - in an orbit view a pick that hits a volume_slice node writes its voxel too.
// Offscreen; skips where nothing rasterises unless CVC_REQUIRE_RENDER=1.

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
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_file_io.h>
#include <filesystem>
#include <fstream>
#include <string>
#include <vtkCamera.h>
#include <vtkRenderer.h>
#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
#endif

namespace fs = std::filesystem;
using cvcgl_test::check;

namespace {

constexpr int kWidth = 960, kHeight = 720;
constexpr double kDt = 1.0 / 30.0;

std::string doc(const std::string &file) {
  return R"(meta:
  name: slice view test
  min_libcvc: 3.7.0

windows:
  - window: Slice
    id: slice_window
    pos: [20, 20]
    size: [440, 420]
    children:
      - type: viewport
        view: slicer
        mode: slice
        axis: { bind: grid.view }
        input: pick
        pick: hits
        chrome: false
        clear_color: [0, 0, 0]
        width: 400
        height: 340
  - window: Orbit
    id: orbit_window
    pos: [500, 20]
    size: [440, 420]
    children:
      - type: viewport
        view: orbit
        input: pick
        pick: orbit_hits
        chrome: false
        elevation: 70
        width: 400
        height: 340

scene:
  chrome: false
  nodes:
    - node: s_z
      type: volume_slice
      source: { file: ")" +
         file + R"(" }
      axis: z
      index: 2
    - node: s_y
      type: volume_slice
      source: { file: ")" +
         file + R"(" }
      axis: y
      index: 3
    - node: s_x
      type: volume_slice
      source: { file: ")" +
         file + R"(" }
      axis: x
      index: 5
      transfer_function: { color: [[0, 1, 0, 0], [1, 1, 0, 0]] }
)";
}

std::string val(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app)(path).value();
}

// World point -> the window's VTK display coordinates where view `vp` shows it: its renderer draws
// into its texture, which the window shows at its region.
void to_display(cvc::gl::Viewport &vp, double x, double y, double z, double &dx, double &dy) {
  vtkRenderer *ren = vp.renderer();
  ren->SetWorldPoint(x, y, z, 1.0);
  ren->WorldToDisplay();
  double d[3], w[2];
  ren->GetDisplayPoint(d);
  vp.rendererToWindow(d[0], d[1], w);
  dx = w[0];
  dy = w[1];
}

// Window display -> world on the plane view `vp` looks at (its focal plane: a parallel view).
void to_world(cvc::gl::Viewport &vp, double wx, double wy, double out[3]) {
  vtkRenderer *ren = vp.renderer();
  double d[2];
  vp.windowToRenderer(wx, wy, d);
  ren->SetDisplayPoint(d[0], d[1], 0.5);
  ren->DisplayToWorld();
  double w[4];
  ren->GetWorldPoint(w);
  for (int c = 0; c < 3; ++c)
    out[c] = w[c] / w[3];
}

} // namespace

int main() {
  const bool require = std::getenv("CVC_REQUIRE_RENDER") != nullptr;
#ifndef CVC_ENABLE_IMGUI
  std::printf("SKIP: no ImGui\n");
  return require ? 1 : 0;
#else
  const fs::path dir =
      fs::temp_directory_path() /
      ("cvcgl_slice_view_" +
       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  fs::create_directories(dir);
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  {
    // 8 x 6 x 4, one unit apart, voxel (i, j, k) = i + 10 j + 100 k.
    cvc::volume v(app, cvc::dimension(8, 6, 4), cvc::Float, cvc::bounding_box(0, 0, 0, 7, 5, 3));
    for (int k = 0; k < 4; ++k)
      for (int j = 0; j < 6; ++j)
        for (int i = 0; i < 8; ++i)
          v(i, j, k, i + 10.0 * j + 100.0 * k);
    cvc::writeVolumeFile(app, v, (dir / "coded.rawiv").generic_string());
  }
  const fs::path docfile = dir / "slice_view.ari";
  std::ofstream(docfile) << doc((dir / "coded.rawiv").generic_string());

  cvc::gl::SceneGraph sg(app, "sv");
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
  cvc::state::instance(app)("sv.grid.view").value(std::string("xy"));
  for (const std::string &w : rt.load(docfile.string()))
    check(false, "the document loads without warnings", w);
  for (int f = 0; f < 8; ++f)
    rt.step(kDt);
  cvc::gl::ViewportManager &vm = view.viewportManager();
  check(vm.hasViewport("slicer") && vm.hasViewport("orbit"), "both viewports exist");
  if (!vm.hasViewport("slicer")) {
    fs::remove_all(dir);
    return cvcgl_test::finish("cvcgl_ariadne_slice_view");
  }
  cvc::gl::Viewport &slicer = vm.viewport("slicer");
  vtkRenderer *ren = slicer.renderer();
  cvc::gl::CameraController &sc = slicer.camera();

  // grid.view = xy: down z, parallel, framed on s_z (x 0..7, y 0..5, at z = 2).
  double eye[3], focal[3], up[3];
  sc.getPose(eye, focal, up);
  check(sc.mode() == cvc::gl::CameraController::Mode::Slice && sc.sliceAxis() == 2,
        "mode: slice with grid.view xy looks down z");
  check(std::fabs(focal[0] - 3.5) < 1e-6 && std::fabs(focal[1] - 2.5) < 1e-6 &&
            std::fabs(focal[2] - 2.0) < 1e-6 && eye[2] > focal[2] && std::fabs(up[1] - 1.0) < 1e-9,
        "framed on the z slice, from above, y up",
        std::to_string(focal[0]) + " " + std::to_string(focal[1]) + " " + std::to_string(focal[2]));
  check(ren->GetActiveCamera()->GetParallelProjection(), "parallel projection");

  // The red x slice is edge-on here and left out: no red pixel inside the view.
  {
    double r[4];
    slicer.region(r);
    const cvcgl_test::Frame f = cvcgl_test::grab(view);
    long red = 0, lit = 0;
    for (int y = int(r[1] * kHeight) + 2; y < int(r[3] * kHeight) - 2; ++y)
      for (int x = int(r[0] * kWidth) + 2; x < int(r[2] * kWidth) - 2; ++x) {
        const unsigned char *p = f.at(x, y);
        red += cvcgl_test::isHue(p, 0) ? 1 : 0;
        lit += cvcgl_test::isBg(p) ? 0 : 1;
      }
    check(lit > 1000, "the z slice draws in the view", std::to_string(lit));
    check(red == 0, "the x slice (red, edge-on) is left out", std::to_string(red));
  }

  // SceneViews::pick at voxel (3, 4) of the z slice: the plane, the voxel, its value.
  double dx = 0.0, dy = 0.0;
  to_display(slicer, 3.0, 4.0, 2.0, dx, dy);
  cvc::gl::ariadne::SceneViews::PickHit hit;
  const bool picked = rt.views().pick("slicer", dx, dy, hit);
  check(picked && hit.voxel && hit.node == "s_z" && hit.ijk[0] == 3 && hit.ijk[1] == 4 &&
            hit.ijk[2] == 2 && hit.value == 3 + 40 + 200,
        "a pick in the slice view names voxel (3, 4, 2) and its value",
        hit.node + " " + std::to_string(hit.ijk[0]) + " " + std::to_string(hit.ijk[1]) + " " +
            std::to_string(hit.ijk[2]) + " = " + std::to_string(hit.value));
  check(std::fabs(hit.world[2] - 2.0) < 1e-9, "exactly on the plane");
  double ox = 0.0, oy = 0.0;
  to_display(slicer, 9.5, 2.0, 2.0, ox, oy); // past the volume's edge, still in the view
  cvc::gl::ariadne::SceneViews::PickHit past;
  check(!rt.views().pick("slicer", ox, oy, past) && past.plane && !past.voxel &&
            std::fabs(past.world[0] - 9.5) < 1e-6,
        "past the edge: no hit, but the point on the plane");

  // The left button: the fields under the pick key.
  ImGui::SetCurrentContext(ui.imguiContext());
  ImGuiIO &io = ImGui::GetIO();
  io.AddMousePosEvent(static_cast<float>(dx), static_cast<float>(kHeight - dy));
  rt.step(kDt);
  io.AddMouseButtonEvent(0, true);
  rt.step(kDt);
  io.AddMouseButtonEvent(0, false);
  rt.step(kDt);
  rt.step(kDt);
  const std::string k = "sv.hits";
  check(val(app, k + ".hit") == "1" && val(app, k + ".node") == "s_z" &&
            val(app, k + ".i") == "3" && val(app, k + ".j") == "4" && val(app, k + ".k") == "2" &&
            val(app, k + ".value") == "243",
        "a click writes i, j, k and value",
        val(app, k + ".i") + " " + val(app, k + ".j") + " " + val(app, k + ".k") + " " +
            val(app, k + ".value"));
  check(val(app, k + ".phase") == "up" && !val(app, k + ".x").empty(), "with the point and phase");

  // The wheel zooms about the pointer.
  {
    double before[3], after[3];
    to_world(slicer, dx, dy, before);
    const double s0 = ren->GetActiveCamera()->GetParallelScale();
    io.AddMouseWheelEvent(0.0f, 2.0f);
    for (int f = 0; f < 40; ++f)
      rt.step(kDt); // the glide settles
    to_world(slicer, dx, dy, after);
    const double s1 = ren->GetActiveCamera()->GetParallelScale();
    check(s1 < s0, "the wheel zooms in", std::to_string(s0) + " -> " + std::to_string(s1));
    check(std::fabs(after[0] - before[0]) < 1e-6 && std::fabs(after[1] - before[1]) < 1e-6,
          "about the pointer: the point under it stays",
          std::to_string(before[0]) + "," + std::to_string(before[1]) + " -> " +
              std::to_string(after[0]) + "," + std::to_string(after[1]));
  }

  // grid.view = xz: the same view looks down y, framed on s_y (x 0..7, z 0..3, at y = 3).
  cvc::state::instance(app)("sv.grid.view").value(std::string("xz"));
  for (int f = 0; f < 6; ++f)
    rt.step(kDt);
  sc.getPose(eye, focal, up);
  check(sc.sliceAxis() == 1 && std::fabs(up[2] - 1.0) < 1e-9 && eye[1] < focal[1],
        "grid.view xz: down y, z up");
  check(std::fabs(focal[1] - 3.0) < 1e-6 && std::fabs(focal[0] - 3.5) < 1e-6 &&
            std::fabs(focal[2] - 1.5) < 1e-6,
        "framed on the y slice",
        std::to_string(focal[0]) + " " + std::to_string(focal[1]) + " " + std::to_string(focal[2]));
  to_display(slicer, 6.0, 3.0, 1.0, dx, dy);
  const bool picked_y = rt.views().pick("slicer", dx, dy, hit);
  check(picked_y && hit.node == "s_y" && hit.ijk[0] == 6 && hit.ijk[1] == 3 && hit.ijk[2] == 1 &&
            hit.value == 6 + 30 + 100,
        "a pick now names voxel (6, 3, 1) of the y slice",
        hit.node + " " + std::to_string(hit.ijk[0]) + " " + std::to_string(hit.ijk[1]) + " " +
            std::to_string(hit.ijk[2]));

  // An orbit view: a pick that hits a slice node writes its voxel too.
  if (vm.hasViewport("orbit")) {
    to_display(vm.viewport("orbit"), 2.0, 1.0, 2.0, dx, dy); // on s_z, away from the other slices
    cvc::gl::ariadne::SceneViews::PickHit oh;
    const bool got = rt.views().pick("orbit", dx, dy, oh);
    check(got && oh.node == "s_z" && oh.voxel && oh.ijk[0] == 2 && oh.ijk[1] == 1 && oh.ijk[2] == 2,
          "an orbit view's pick on the z slice names its voxel",
          oh.node + " " + std::to_string(oh.ijk[0]) + " " + std::to_string(oh.ijk[1]) + " " +
              std::to_string(oh.ijk[2]));
  }
  for (const std::string &w : rt.take_warnings())
    std::printf("  warning: %s\n", w.c_str());
  std::error_code ec;
  fs::remove_all(dir, ec);
  return cvcgl_test::finish("cvcgl_ariadne_slice_view");
#endif
}
