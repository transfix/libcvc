// cvcgl_molecule_pick — picking a `molecule` node (cvc/gl/ariadne/molecule_nodes.h) in an
// `input: pick` viewport: the pick meets its impostor spheres exactly (the cell picker can't),
// names the node, and reports the atom (atom, serial, element, ...) beside x, y, z, in
// SceneViews::pick, in the view's pick key and in the node's own on_pick: event. A pick on its
// sticks names the atom nearest the point. The molecule is a document's dataset (data:, an .xyzr
// file). Offscreen; skips where nothing rasterises unless CVC_REQUIRE_RENDER=1.

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

// Three atoms along x: two far apart, drawn as spheres, and a bonded pair on the right, drawn as
// sticks only. The view writes its picks under `hits`; the node's on_pick: notes the atom.
const char *kDoc = R"(meta:
  name: molecule pick test
  min_libcvc: 4.0.0

customs:
  - node: molecule
    required: true

data:
  atoms: atoms.xyzr

init: (state-set "picked" "")

windows:
  - window: Pick
    id: pick_window
    pos: [20, 20]
    size: [740, 420]
    children:
      - type: viewport
        view: picker
        input: pick
        pick: hits
        width: 700
        height: 340

scene:
  nodes:
    - node: atoms
      type: molecule
      source: { data: atoms }
      style: spacefill
      on_pick: (state-set "picked" (str-concat (state-get "picked") (get-attr event "atom") " "))
    - node: pair
      type: molecule
      source: { data: atoms }
      style: sticks
      bond_radius: 0.4
      transform: { position: [0, -6, 0] }
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
  const fs::path dir = fs::temp_directory_path() / "cvcgl_molecule_pick";
  fs::create_directories(dir);
  const fs::path doc = dir / "molecule_pick.ari";
  std::ofstream(doc) << kDoc;
  // x y z radius: two lone atoms, and a pair 1.4 apart (bonded: no element, so 0.77 + 0.77 + 0.45).
  std::ofstream(dir / "atoms.xyzr") << "-6 0 0 2\n0 0 0 2\n5 0 0 1.5\n6.4 0 0 1.5\n";

  cvc::app app;
  app.properties("system.log_verbosity", "0");
  cvc::gl::SceneGraph sg(app, "mp");
  sg.setDiagnosticChromeVisible(false);
  cvc::gl::SceneRenderer view(sg, kWidth, kHeight, /*offscreen=*/true, "main");
  view.setBackground(0.0, 0.0, 0.0);
  cvc::gl::CameraController cam(view);
  cvc::gl::ImGuiOverlay ui(view);
  if (!ui.enabled()) {
    std::printf("SKIP: no ImGui overlay\n");
    return require ? 1 : 0;
  }
  if (!cvcgl_test::renderAvailable(app))
    return cvcgl_test::finish("cvcgl_molecule_pick");
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
  // Until both nodes have built their views and the view has framed them.
  for (int f = 0; f < 600 && (val(app, "mp.graphics.root.children.atoms.info.drawn") != "4" ||
                              val(app, "mp.graphics.root.children.pair.info.bonds") != "1");
       ++f)
    rt.step(kDt);
  for (int f = 0; f < 6; ++f)
    rt.step(kDt);
  check(val(app, "mp.graphics.root.children.atoms.info.drawn") == "4", "the spheres are drawn",
        val(app, "mp.graphics.root.children.atoms.info.drawn"));
  check(val(app, "mp.graphics.root.children.pair.info.bonds") == "1", "the pair's bond is drawn",
        val(app, "mp.graphics.root.children.pair.info.bonds"));
  cvc::gl::ViewportManager &vm = view.viewportManager();
  check(vm.hasViewport("picker"), "the viewport exists");
  if (!vm.hasViewport("picker"))
    return cvcgl_test::finish("cvcgl_molecule_pick");
  cvc::gl::Viewport &picker = vm.viewport("picker");
  vtkRenderer *ren = picker.renderer();
  struct Spot {
    double x, y;
    float mx, my;
  };
  const auto spot = [&](double wx, double wy, double wz) {
    ren->SetWorldPoint(wx, wy, wz, 1.0);
    ren->WorldToDisplay();
    double d[3], w[2];
    ren->GetDisplayPoint(d);
    picker.rendererToWindow(d[0], d[1], w);
    return Spot{w[0], w[1], static_cast<float>(w[0]), static_cast<float>(kHeight - w[1])};
  };

  using PickHit = cvc::gl::ariadne::SceneViews::PickHit;
  const auto field = [](const PickHit &h, const std::string &k) {
    for (const auto &f : h.fields)
      if (f.first == k)
        return f.second;
    return std::string("-");
  };
  // Each lone atom by the centre of its sphere, and a point on its sphere off the centre.
  for (int a = 0; a < 2; ++a) {
    const double x = a == 0 ? -6.0 : 0.0;
    for (const double off : {0.0, 1.2}) {
      const Spot s = spot(x + off, off, 0.0);
      PickHit h;
      const bool got = rt.views().pick("picker", s.x, s.y, h);
      const std::string what = "atom " + std::to_string(a) + (off ? " off its centre" : "");
      check(got && h.node == "atoms", what + ": the pick hits the node", h.node);
      check(field(h, "atom") == std::to_string(a), what + ": and names the atom", field(h, "atom"));
      check(field(h, "serial") == std::to_string(a + 1), what + ": its serial", field(h, "serial"));
      // On the sphere's surface, nearer the eye than its centre.
      const double dx = h.world[0] - x, dy = h.world[1], dz = h.world[2];
      check(std::abs(std::sqrt(dx * dx + dy * dy + dz * dz) - 2.0) < 1e-3,
            what + ": the point is on its sphere");
    }
  }
  // The sticks: a pick on the pair's bond names an atom of the pair.
  {
    const Spot s = spot(5.7, -6.0, 0.0);
    PickHit h;
    const bool got = rt.views().pick("picker", s.x, s.y, h);
    check(got && h.node == "pair", "a pick on the bond hits the pair", h.node);
    check(field(h, "atom") == "2" || field(h, "atom") == "3", "and names one of its atoms",
          field(h, "atom"));
  }
  // A miss (the view's corner): no fields.
  {
    double rg[4];
    picker.region(rg);
    PickHit h;
    check(!rt.views().pick("picker", rg[0] * kWidth + 4, rg[1] * kHeight + 4, h) &&
              h.fields.empty(),
          "a miss has no atom", h.node);
  }

  // A click: the view's pick key and the node's on_pick: get the atom.
  ImGui::SetCurrentContext(ui.imguiContext());
  ImGuiIO &io = ImGui::GetIO();
  const Spot s = spot(0.0, 0.0, 0.0);
  io.AddMousePosEvent(s.mx, s.my);
  rt.step(kDt);
  io.AddMouseButtonEvent(0, true);
  rt.step(kDt);
  rt.step(kDt);
  io.AddMouseButtonEvent(0, false);
  rt.step(kDt);
  rt.step(kDt);
  for (const std::string &w : rt.take_warnings())
    std::printf("  warning: %s\n", w.c_str());
  check(val(app, "mp.picked") == "1 1 ", "the node's on_pick: got the atom, down and up",
        val(app, "mp.picked"));
  check(val(app, "mp.hits.atom") == "1" && val(app, "mp.hits.node") == "atoms",
        "the view's pick key holds the atom", val(app, "mp.hits.atom"));
  io.AddMousePosEvent(-1.0f, -1.0f);
  rt.step(kDt);

  view.close();
  return cvcgl_test::finish("cvcgl_molecule_pick");
#endif
}
