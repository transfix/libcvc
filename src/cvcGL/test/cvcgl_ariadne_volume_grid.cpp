// cvcgl_ariadne_volume_grid — the volume_grid component
// (src/cvc/ariadne/components/volume/volume_grid.ari) over two `volume_grid` nodes in one document,
// through AriRuntime, the mouse and the views (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §4,
// P8; T6 and T13):
//   - the document (the component's switch layout for one grid, its quad layout for the other)
//     loads without warnings, and each include makes its own viewports;
//   - the app loads the volume (the document's data: entry) and both grids show that one volume,
//     the dataset's own, not a copy each;
//   - the switch's slice view looks down the axis grid.view names, and follows it;
//   - the pointer over the slice view fills the grid's probe (the viewport's pointer:);
//   - a click in the slice view marks the voxel under it in the current class, its pick carrying
//     the plane's axis; a second click unmarks it; a drag selects with a rubber band;
//   - a click in the other grid's XZ view (quad) marks a point in that grid alone, on its y plane.
// Offscreen; skips where nothing rasterises unless CVC_REQUIRE_RENDER=1.

#include "streaming_test_util.h"

#include <boost/any.hpp>
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
#include <cvc/state/state.h>
#include <cvc/volume/point_classes.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_file_io.h>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
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
  name: volume grid test
  min_libcvc: 4.0.0
import:
  - cvc://components/volume/volume_grid.ari
data:
  coded: ")" +
         file + R"("
windows:
  - include: volume_grid
    args: { node: a, id: ga, title: A, view_size: 200 }
    pos: [0, 0]
    size: [960, 330]
  - include: volume_grid
    args: { node: b, id: gb, title: B, view_size: 150, layout: quad }
    pos: [0, 340]
    size: [960, 380]
scene:
  nodes:
    - node: a
      type: volume_grid
      source: { data: coded }
      view: xy
      classes:
        - { name: seeds, color: [1, 0, 0], radius: 1 }
    - node: b
      type: volume_grid
      source: { data: coded }
      classes:
        - { name: other, color: [0, 1, 0], radius: 1 }
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

std::shared_ptr<const cvc::point_classes> points_of(cvc::app &app, const std::string &path) {
  const boost::any d = cvc::state::instance(app)(path).data();
  if (const auto *p = boost::any_cast<std::shared_ptr<const cvc::point_classes>>(&d))
    return *p;
  return nullptr;
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
      ("cvcgl_ariadne_volume_grid_" +
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
  const fs::path docfile = dir / "volume_grid.ari";
  std::ofstream(docfile) << doc((dir / "coded.rawiv").generic_string());

  cvc::gl::SceneGraph sg(app, "vg");
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
  // The component library from the source tree (an installed one is found by default).
  rt.add_component_path(
      (fs::path(__FILE__).parent_path().parent_path().parent_path() / "cvc" / "ariadne").string());
  for (const std::string &w : rt.load(docfile.string()))
    check(false, "the document loads without warnings", w);
  for (int f = 0; f < 10; ++f)
    rt.step(kDt);
  // The app loads the volume off the frame thread; the grids take it once it is there.
  for (int f = 0; f < 1000 && (val(app, "vg.graphics.root.children.a.grid.info.dims").empty() ||
                               val(app, "vg.graphics.root.children.b.grid.info.dims").empty());
       ++f) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    rt.step(kDt);
  }
  for (int f = 0; f < 5; ++f)
    rt.step(kDt);
  {
    const auto vol = [&](const char *node) {
      const boost::any d = cvc::state::instance(app)(std::string("vg.graphics.root.children.") +
                                                     node + ".grid.volume")
                               .data();
      const auto *p = boost::any_cast<std::shared_ptr<const cvc::volume>>(&d);
      return p ? *p : nullptr;
    };
    check(vol("a") && vol("a") == vol("b") &&
              val(app, "vg.graphics.root.children.a.grid.info.dims") == "8 6 4",
          "the app loads the volume (data: coded) and both grids show that one volume",
          val(app, "vg.graphics.root.children.a.grid.status"));
  }
  cvc::gl::ViewportManager &vm = view.viewportManager();
  check(vm.hasViewport("ga_slice") && vm.hasViewport("ga_3d"),
        "the switch layout makes a slice view and a 3D view");
  check(vm.hasViewport("gb_xy") && vm.hasViewport("gb_xz") && vm.hasViewport("gb_zy") &&
            vm.hasViewport("gb_quad_3d"),
        "the quad layout makes the three slice views and a 3D view");
  check(!vm.hasViewport("gb_slice") && !vm.hasViewport("ga_xy"),
        "each include draws only its own layout's views");
  if (!vm.hasViewport("ga_slice") || !vm.hasViewport("gb_xz")) {
    fs::remove_all(dir);
    return cvcgl_test::finish("cvcgl_ariadne_volume_grid");
  }
  const std::string ka = "vg.graphics.root.children.a.grid";
  const std::string kb = "vg.graphics.root.children.b.grid";
  cvc::gl::Viewport &slice = vm.viewport("ga_slice");
  check(slice.camera().mode() == cvc::gl::CameraController::Mode::Slice &&
            slice.camera().sliceAxis() == 2,
        "grid a's slice view looks down z (grid.view xy)");
  check(val(app, ka + ".slice.z") == "1", "its plane is the middle one, z = 1");

  ImGui::SetCurrentContext(ui.imguiContext());
  ImGuiIO &io = ImGui::GetIO();
  cvc::gl::Viewport *onView = &slice; // the view `at` projects through
  const auto at = [&](double x, double y, double z) {
    double dx = 0.0, dy = 0.0;
    to_display(*onView, x, y, z, dx, dy);
    return ImVec2(static_cast<float>(dx), static_cast<float>(kHeight - dy));
  };

  // The pointer over voxel (5, 2) of the z = 1 plane: the probe.
  const ImVec2 p52 = at(5.0, 2.0, 1.0);
  io.AddMousePosEvent(p52.x, p52.y);
  rt.step(kDt);
  rt.step(kDt);
  check(val(app, ka + ".pointer.i") == "5" && val(app, ka + ".pointer.phase") == "hover",
        "the pointer over the view writes grid.pointer", val(app, ka + ".pointer.i"));
  check(val(app, ka + ".cursor.i") == "5" && val(app, ka + ".cursor.j") == "2" &&
            val(app, ka + ".cursor.k") == "1" && val(app, ka + ".cursor.value") == "125",
        "and the probe follows it",
        val(app, ka + ".cursor.i") + " " + val(app, ka + ".cursor.value"));

  // A click there marks (5, 2, 1) in seeds; another unmarks it.
  const auto click = [&](ImVec2 p) {
    io.AddMousePosEvent(p.x, p.y);
    rt.step(kDt);
    io.AddMouseButtonEvent(0, true);
    rt.step(kDt);
    io.AddMouseButtonEvent(0, false);
    rt.step(kDt);
    rt.step(kDt);
  };
  click(p52);
  auto pa = points_of(app, ka + ".points");
  check(pa && pa->classes.size() == 1 && pa->classes[0].points == std::vector<int>{5, 2, 1},
        "a click in the slice view marks the voxel under it", val(app, ka + ".status"));
  check(val(app, ka + ".pick.axis") == "z", "the pick names the plane's axis");
  click(p52);
  pa = points_of(app, ka + ".points");
  check(pa && pa->point_count() == 0, "a second click unmarks it", val(app, ka + ".status"));

  // Two points, then a drag over the left half selects the one there.
  click(at(1.0, 1.0, 1.0));
  click(at(6.0, 4.0, 1.0));
  io.AddMousePosEvent(at(-0.4, -0.4, 1.0).x, at(-0.4, -0.4, 1.0).y);
  rt.step(kDt);
  io.AddMouseButtonEvent(0, true);
  rt.step(kDt);
  for (double t = 0.25; t <= 1.0; t += 0.25) {
    const ImVec2 p = at(-0.4 + 3.4 * t, -0.4 + 3.4 * t, 1.0);
    io.AddMousePosEvent(p.x, p.y);
    rt.step(kDt);
  }
  io.AddMouseButtonEvent(0, false);
  rt.step(kDt);
  rt.step(kDt);
  check(val(app, ka + ".selection.count") == "1", "a drag selects the points in its band",
        val(app, ka + ".selection.count") + " " + val(app, ka + ".status"));

  // The switch follows grid.view.
  cvc::state::instance(app)(ka + ".view").value(std::string("zy"));
  for (int f = 0; f < 3; ++f)
    rt.step(kDt);
  check(slice.camera().sliceAxis() == 0, "grid.view zy turns the slice view down x");

  // Grid b, quad layout: a click in its XZ view (down y, at its middle plane y = 2) marks a point
  // of b's class alone.
  cvc::gl::Viewport &xz = vm.viewport("gb_xz");
  check(xz.camera().mode() == cvc::gl::CameraController::Mode::Slice &&
            xz.camera().sliceAxis() == 1,
        "grid b's XZ view looks down y");
  onView = &xz;
  click(at(3.0, 2.0, 2.0));
  auto pb = points_of(app, kb + ".points");
  pa = points_of(app, ka + ".points");
  check(pb && pb->classes[0].points == std::vector<int>{3, 2, 2},
        "a click in grid b's XZ view marks (3, 2, 2) in b", val(app, kb + ".status"));
  check(pa && pa->point_count() == 2, "and leaves grid a alone");
  check(val(app, kb + ".pick.view") == "gb_xz" && val(app, ka + ".pick.view") == "ga_slice",
        "each grid's picks come from its own views");

  fs::remove_all(dir);
  return cvcgl_test::finish("cvcgl_ariadne_volume_grid");
#endif
}
