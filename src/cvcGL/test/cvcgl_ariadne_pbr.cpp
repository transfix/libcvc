// cvcgl_ariadne_pbr — physically based materials in a document (docs/ARIADNE.md §5.4, "Physically
// based materials"): a node's `material: { pbr: { metallic, roughness, emissive, coat } }` and the
// scene's `environment:`.
//
// Pins:
//   A. AriRuntime: each pbr: node's actor is PBR with the document's values, a Phong node is
//      left Phong; the environment lights the main view and the viewport widget (a mirror) with
//      the document's colours; an emissive node glows its colour; it all holds over frames;
//   B. realize_scene alone: a pbr: node with no environment: gets the default environment;
//      environment: false lights pbr: nodes with the lights alone; a scene with neither turns
//      nothing on.
// Offscreen; part A skips where nothing rasterises unless CVC_REQUIRE_RENDER=1.

#include "streaming_test_util.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/loader.h>
#include <cvc/core/app.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/Viewport.h>
#include <cvc/gl/ViewportManager.h>
#include <cvc/gl/ariadne/AriRuntime.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>

namespace fs = std::filesystem;
using cvcgl_test::check;

namespace {

constexpr int kWidth = 960, kHeight = 720;
constexpr double kDt = 1.0 / 30.0;

const char *kDoc = R"(meta:
  name: pbr test
  min_libcvc: 3.7.0

windows:
  - window: Views
    id: views_window
    pos: [20, 20]
    size: [440, 360]
    children:
      - type: viewport
        view: side
        position: [0, -14, 3]
        target: [0, 0, 0]
        width: 400
        height: 300

scene:
  environment: { sky: [0.6, 0.7, 0.9], horizon: [0.5, 0.5, 0.5], ground: [0.15, 0.12, 0.1] }
  cameras:
    - camera: main
      position: [0, 0, 12]
      target: [0, 0, 0]
  lights:
    - light: sun
      azimuth: 0
      elevation: 90
      intensity: 0
  nodes:
    - node: lamp
      source: { box: { size: [2, 2, 2] } }
      material: { color: [0, 0, 0], pbr: { roughness: 1, emissive: [1, 0.5, 0] } }
    - node: paint
      source: { box: { size: [2, 2, 2] } }
      transform: { position: [-4, 0, 0] }
      material: { color: [0.8, 0.1, 0.1], pbr: { metallic: 0.2, roughness: 0.4, coat: 0.8 } }
    - node: chrome
      source: { box: 2 }
      transform: { position: [4, 0, 0] }
      material: { pbr: { metallic: 1, roughness: 0.15 } }
    - node: matte
      source: { box: 2 }
      transform: { position: [0, 4, 0] }
      material: { color: [0.5, 0.5, 0.5] }
)";

vtkProperty *property_of(cvc::gl::SceneGraph &sg, const std::string &name) {
  std::shared_ptr<cvc::gl::GraphicsNode> n = sg.getGraphics(name);
  auto *actor = n ? vtkActor::SafeDownCast(n->prop()) : nullptr;
  return actor ? actor->GetProperty() : nullptr;
}

bool near(double a, double b) { return std::fabs(a - b) < 1e-6; }

// Realize `yaml`'s scene into a fresh scene graph with no renderer, and say whether it lit with
// an environment, and with which sky.
bool realized_ibl(const std::string &yaml, double sky[3]) {
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  cvc::gl::SceneGraph sg(app, "ibl");
  const cvc::ariadne::LoadResult r = cvc::ariadne::load_string(app, yaml);
  if (!r.ok) {
    check(false, "the document loads", r.error);
    return false;
  }
  std::vector<std::string> warnings;
  cvc::gl::ariadne::RealizedScene realized =
      cvc::gl::ariadne::realize_scene(sg, r.scene, "", &warnings);
  double horizon[3], ground[3];
  sg.environment(sky, horizon, ground);
  return sg.imageBasedLighting();
}

void realize_alone() {
  std::printf("B. realize_scene alone\n");
  const std::string head = "meta: { min_libcvc: \"0.0.0\" }\nscene:\n";
  const std::string pbrNode = "  nodes:\n    - node: n\n      source: { box: 1 }\n"
                              "      material: { pbr: { metallic: 1 } }\n";
  const std::string phongNode = "  nodes:\n    - node: n\n      source: { box: 1 }\n"
                                "      material: { color: [1, 1, 1] }\n";
  double sky[3] = {0, 0, 0};
  const bool lit = realized_ibl(head + pbrNode, sky); // before the detail reads sky
  check(lit && near(sky[0], 0.50) && near(sky[2], 0.60),
        "a pbr: node with no environment: gets the default environment",
        std::to_string(sky[0]) + ", " + std::to_string(sky[2]));
  check(!realized_ibl(head + "  environment: false\n" + pbrNode, sky),
        "environment: false lights it with the lights alone");
  check(!realized_ibl(head + phongNode, sky), "no pbr: node and no environment: nothing on");
  check(realized_ibl(head + "  environment: { sky: [0.1, 0.2, 0.3] }\n" + phongNode, sky) &&
            near(sky[1], 0.2f),
        "an environment: alone turns it on, with its colours");
}

} // namespace

int main() {
  const bool require = std::getenv("CVC_REQUIRE_RENDER") != nullptr;
  realize_alone();
#ifndef CVC_ENABLE_IMGUI
  std::printf("SKIP: no ImGui\n");
  return require ? 1 : cvcgl_test::finish("cvcgl_ariadne_pbr");
#else
  const fs::path dir = fs::temp_directory_path() / "cvcgl_ariadne_pbr";
  fs::create_directories(dir);
  const fs::path doc = dir / "pbr.ari";
  std::ofstream(doc) << kDoc;

  cvc::app app;
  app.properties("system.log_verbosity", "0");
  cvc::gl::SceneGraph sg(app, "pbr");
  sg.setDiagnosticChromeVisible(false);
  cvc::gl::SceneRenderer view(sg, kWidth, kHeight, /*offscreen=*/true, "main");
  view.setBackground(0.0, 0.0, 0.0);
  cvc::gl::CameraController cam(view);
  cvc::gl::ImGuiOverlay ui(view);
  if (!ui.enabled()) {
    std::printf("SKIP: no ImGui overlay\n");
    return require ? 1 : cvcgl_test::finish("cvcgl_ariadne_pbr");
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

  std::printf("A. AriRuntime\n");
  vtkProperty *paint = property_of(sg, "paint"), *chrome = property_of(sg, "chrome"),
              *lamp = property_of(sg, "lamp"), *matte = property_of(sg, "matte");
  check(paint && chrome && lamp && matte, "every node has an actor");
  if (!paint || !chrome || !lamp || !matte)
    return cvcgl_test::finish("cvcgl_ariadne_pbr");
  check(paint->GetInterpolation() == VTK_PBR && near(paint->GetMetallic(), 0.2) &&
            near(paint->GetRoughness(), 0.4) && near(paint->GetCoatStrength(), 0.8),
        "paint: PBR, metallic 0.2, roughness 0.4, coat 0.8");
  check(near(paint->GetDiffuse(), 1.0), "its base colour whole (diffuse 1)");
  check(chrome->GetInterpolation() == VTK_PBR && near(chrome->GetMetallic(), 1.0) &&
            near(chrome->GetRoughness(), 0.15),
        "chrome: PBR, metallic 1, roughness 0.15");
  const double *e = lamp->GetEmissiveFactor();
  check(lamp->GetInterpolation() == VTK_PBR && near(e[0], 1.0) && near(e[1], std::pow(0.5, 2.2)) &&
            near(e[2], 0.0),
        "lamp: emissive (1, 0.5, 0), made linear");
  check(matte->GetInterpolation() != VTK_PBR, "matte: Phong, as before");

  vtkRenderer *main = view.renderer();
  double sky[3], horizon[3], ground[3];
  sg.environment(sky, horizon, ground);
  check(sg.imageBasedLighting() && main->GetUseImageBasedLighting() &&
            main->GetEnvironmentTexture() == sg.environmentTexture(),
        "the environment lights the main view");
  check(near(sky[1], 0.7) && near(horizon[0], 0.5) && near(ground[2], 0.1),
        "with the document's colours");
  cvc::gl::ViewportManager &vm = view.viewportManager();
  check(vm.hasViewport("side"), "the viewport exists");
  if (vm.hasViewport("side")) {
    vtkRenderer *side = vm.viewport("side").renderer();
    check(side->GetUseImageBasedLighting() &&
              side->GetEnvironmentTexture() == sg.environmentTexture(),
          "and the viewport's mirror, with the same environment");
    const double *up = side->GetEnvironmentUp();
    check(near(up[2], 1.0), "about +z");
  }

  // The lamp (black, unlit) seen from straight above, at the frame's centre: its emissive
  // orange, plus a little of the environment reflected.
  const std::vector<unsigned char> frame = view.frameRGB();
  const std::size_t i = (static_cast<std::size_t>(kHeight / 2) * kWidth + kWidth / 2) * 3;
  const int r = frame[i], g = frame[i + 1], b = frame[i + 2];
  check(r > 235 && g > 105 && g < 175 && b < 110 && r > g + 60 && g > b + 30,
        "the lamp glows orange",
        std::to_string(r) + ", " + std::to_string(g) + ", " + std::to_string(b));

  for (int f = 0; f < 10; ++f)
    rt.step(kDt);
  check(paint->GetInterpolation() == VTK_PBR && main->GetUseImageBasedLighting(),
        "and it holds over frames");

  view.close(); // ~AriRuntime runs after the return below, with the view closed
  return cvcgl_test::finish("cvcgl_ariadne_pbr");
#endif
}
