// A document's assets, its scenes of their own, and its main view's show: (docs/ARIADNE.md Â§5.18).
//
// Part 1, realize_scene with a shared SceneAssetCache (no window): two scenes built from one asset
// read it once, and their nodes share its geometry's buffers and the VTK arrays built from them,
// until one writes into its points (copy-on-write).
//
// Part 2, a document run as a host runs it (AriRuntime offscreen):
//   - `scenes:`: a scene of its own gets a viewport of its own ("scene.<id>", not shown), and the
//     viewports that name it mirror it: its props and its background, none of the main scene's;
//     two viewports of one scene both draw it; its shadows reach them;
//   - `main_view: { show }`: the main view leaves a tagged group out while a viewport that shows
//     the group draws it, and the main renderer has the group's props back after its draw;
//   - a scene's shadow maps are baked on the first frame a view shows it again, whatever its
//     stride, when something moved while it was unseen;
//   - a new document takes the scenes' viewports down.
// Part 2 skips (rc 0) where nothing rasterises, unless CVC_REQUIRE_RENDER=1.
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
#include <cvc/gl/ariadne/scene_views.h>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include <vtkActor.h>
#include <vtkMapper.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPropCollection.h>
#include <vtkRenderer.h>

namespace fs = std::filesystem;
using cvcgl_test::check;

namespace {

constexpr int kWidth = 960, kHeight = 720;
constexpr double kDt = 1.0 / 30.0;

const char *kAssetsOnly = R"(meta: { min_libcvc: "0.0.0" }
assets:
  crate: { source: { box: { size: [2, 2, 2] } } }
scene:
  nodes:
    - { node: a, type: geometry, source: { asset: crate } }
    - { node: b, type: geometry, source: { asset: crate }, transform: { position: [4, 0, 0] } }
    - { node: lost, type: geometry, source: { asset: nothing } }
scenes:
  - scene: other
    nodes:
      - { node: c, type: geometry, source: { asset: crate } }
)";

const char *kDoc = R"(meta:
  name: scenes test
  min_libcvc: 3.7.0

assets:
  crate: { source: { box: { size: [2, 2, 2] } } }
  ball: { source: { sphere: 1.2 } }

windows:
  - window: Right
    id: right
    pos: [20, 20]
    size: [300, 260]
    children:
      - { type: viewport, view: right_view, show: [right], width: 260, height: 195 }
  - window: Other
    id: other
    pos: [340, 20]
    size: [600, 260]
    children:
      - type: group
        layout: { kind: grid, col_widths: [auto, auto] }
        children:
          - { type: viewport, view: other_a, scene: other, width: 260, height: 195 }
          - { type: viewport, view: other_b, scene: other, azimuth: 90, width: 260, height: 195 }

scene:
  chrome: false
  main_view: { show: [left] }
  nodes:
    - node: left
      type: group
      tags: [left]
      children:
        - { node: red_crate, type: geometry, source: { asset: crate }, transform: { position: [-3, 0, 1] }, material: { color: [0.9, 0.1, 0.1] } }
    - node: right
      type: group
      tags: [right]
      children:
        - { node: blue_ball, type: geometry, source: { asset: ball }, transform: { position: [3, 0, 1.2] }, material: { color: [0.1, 0.1, 0.9] } }

scenes:
  - scene: other
    background: [0.0, 0.6, 0.0]
    shadows: { enabled: true }
    nodes:
      - { node: yellow_crate, type: geometry, source: { asset: crate }, material: { color: [0.9, 0.9, 0.1] } }
      - { node: floor, type: geometry, source: { plane: { size: 10 } }, transform: { position: [0, 0, -1] }, material: { color: [0.5, 0.5, 0.5] } }
    lights:
      - { light: sun, kind: directional, azimuth: 135, elevation: 55 }
)";

const char *kOtherDoc = R"(meta: { name: none, min_libcvc: 3.7.0 }
windows:
  - window: Plain
    id: plain
    children:
      - text: "No scenes here."
)";

bool has_prop(vtkRenderer *r, vtkProp *p) {
  return r && p && r->GetViewProps()->IsItemPresent(p) != 0;
}

// The polydata a geometry node draws (its actor's mapper's input).
vtkPolyData *drawn(const std::shared_ptr<cvc::gl::GeometryNode> &n) {
  auto *a = n ? vtkActor::SafeDownCast(n->prop()) : nullptr;
  return a && a->GetMapper() ? vtkPolyData::SafeDownCast(a->GetMapper()->GetInputDataObject(0, 0))
                             : nullptr;
}

struct Counts {
  int red = 0, blue = 0, yellow = 0, green = 0;
};

Counts count(const std::vector<unsigned char> &rgb, int w, int h, const double reg[4]) {
  Counts c;
  const int x0 = static_cast<int>(reg[0] * w), x1 = static_cast<int>(reg[2] * w);
  const int y0 = static_cast<int>(reg[1] * h), y1 = static_cast<int>(reg[3] * h);
  for (int y = y0; y < y1; ++y)
    for (int x = x0; x < x1; ++x) {
      const unsigned char *p = &rgb[(static_cast<std::size_t>(y) * w + x) * 3];
      const int r = p[0], g = p[1], b = p[2];
      if (r > 60 && r > 2 * g && r > 2 * b)
        ++c.red;
      else if (b > 60 && b > 2 * r && b > 2 * g)
        ++c.blue;
      else if (r > 60 && g > 60 && r > 2 * b && g > 2 * b)
        ++c.yellow;
      else if (g > 100 && g > 2 * r && g > 2 * b)
        ++c.green;
    }
  return c;
}

} // namespace

int main() {
  const bool require = std::getenv("CVC_REQUIRE_RENDER") != nullptr;
  cvc::app app;
  app.properties("system.log_verbosity", "0");

  // â”€â”€ part 1: one read per asset, shared buffers, across two scenes
  // â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
  {
    cvc::ariadne::LoadResult lr = cvc::ariadne::load_string(app, kAssetsOnly);
    check(lr.ok, "the assets document loads", lr.error);
    check(lr.scene.assets.size() == 1 && lr.scenes.size() == 1 && lr.scenes[0].assets.size() == 1,
          "assets: reach the scene and the scenes: entry");
    cvc::gl::SceneGraph sg(app, "assets_main"), other(app, "assets_other");
    cvc::gl::ariadne::SceneAssetCache cache;
    cvc::gl::ariadne::RealizeOptions o;
    o.assets = &cache;
    std::vector<std::string> warns;
    auto rs = cvc::gl::ariadne::realize_scene(sg, lr.scene, "assets_main", &warns, o);
    auto rs2 = cvc::gl::ariadne::realize_scene(other, lr.scenes[0], "assets_main", &warns, o);
    check(cache.reads == 1, "the asset is read once for three nodes in two scenes",
          std::to_string(cache.reads));
    auto a = std::dynamic_pointer_cast<cvc::gl::GeometryNode>(sg.getGraphics("a"));
    auto b = std::dynamic_pointer_cast<cvc::gl::GeometryNode>(sg.getGraphics("b"));
    auto c = std::dynamic_pointer_cast<cvc::gl::GeometryNode>(other.getGraphics("c"));
    sg.processEvents();
    other.processEvents();
    const cvc::geometry *ga = a ? a->getGeometry() : nullptr;
    const cvc::geometry *gb = b ? b->getGeometry() : nullptr;
    const cvc::geometry *gc = c ? c->getGeometry() : nullptr;
    check(ga && gb && gc && ga->num_points() > 0 && ga->num_points() == gc->num_points(),
          "each node on the asset draws its geometry");
    check(ga && gb && gc && ga->points_ptr() == gb->points_ptr() &&
              ga->points_ptr() == gc->points_ptr(),
          "the nodes share the asset's point buffer (no copy)");
    vtkPolyData *pa = drawn(a), *pb = drawn(b), *pc = drawn(c);
    check(pa && pb && pc && pa != pb && pa->GetPoints() && pa->GetPoints() == pb->GetPoints() &&
              pa->GetPoints() == pc->GetPoints() && pa->GetPolys() == pb->GetPolys() &&
              pa->GetPointData()->GetNormals() == pc->GetPointData()->GetNormals(),
          "each node has a polydata of its own over the first node's VTK arrays");
    check(a && b && c && a->sharesArrays() && b->sharesArrays() && c->sharesArrays(),
          "the three nodes report shared arrays");
    if (pa && pb && pa->GetPoints() && a && b) {
      double before[3], after[3];
      pa->GetPoints()->GetPoint(0, before);
      std::vector<double> xyz(static_cast<std::size_t>(pb->GetNumberOfPoints()) * 3);
      for (vtkIdType i = 0; i < pb->GetNumberOfPoints(); ++i) {
        double p[3];
        pb->GetPoint(i, p);
        xyz[3 * i] = p[0] + 10.0;
        xyz[3 * i + 1] = p[1];
        xyz[3 * i + 2] = p[2];
      }
      b->updateVertices(xyz);
      sg.processEvents();
      pa->GetPoints()->GetPoint(0, after);
      double moved[3];
      pb->GetPoints()->GetPoint(0, moved);
      check(pa->GetPoints() != pb->GetPoints() && after[0] == before[0] &&
                std::abs(moved[0] - (before[0] + 10.0)) < 1e-6 && !b->sharesArrays(),
            "a node that writes its points gets points of its own; the others keep the asset's");
      check(pa->GetPoints() == pc->GetPoints(), "the untouched nodes still share theirs");
      a->updateVertices(xyz); // the first node (the donor) writing its own: c keeps the asset's
      sg.processEvents();
      double cp[3];
      pc->GetPoints()->GetPoint(0, cp);
      check(pa->GetPoints() != pc->GetPoints() && cp[0] == before[0],
            "the first node's own writes do not reach the nodes that took its arrays");
    }
    bool warned = false;
    for (const std::string &w : warns)
      warned = warned || w.find("no asset 'nothing'") != std::string::npos;
    check(warned && !sg.getGraphics("lost"),
          "a node on an unknown asset is warned of and left out");
  }

  // â”€â”€ part 2: the document in a host
  // â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
  const fs::path dir = fs::temp_directory_path() / "cvcgl_ariadne_scenes";
  fs::create_directories(dir);
  const fs::path doc = dir / "scenes.ari", other = dir / "other.ari";
  std::ofstream(doc) << kDoc;
  std::ofstream(other) << kOtherDoc;

  cvc::gl::SceneGraph sg(app, "scenes");
  sg.setDiagnosticChromeVisible(false);
  cvc::gl::SceneRenderer view(sg, kWidth, kHeight, /*offscreen=*/true, "main");
  view.setBackground(0.0, 0.0, 0.0);
  cvc::gl::CameraController cam(view);
  cvc::gl::ImGuiOverlay ui(view);
  if (!ui.enabled()) {
    std::printf("SKIP: no ImGui overlay\n");
    return require ? 1 : cvcgl_test::finish("cvcgl_ariadne_scenes");
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
  if (!ui.enabled()) {
    std::printf("SKIP: nothing rasterises\n");
    return require ? 1 : cvcgl_test::finish("cvcgl_ariadne_scenes");
  }

  cvc::gl::ViewportManager &vm = view.viewportManager();
  check(vm.hasViewport("scene.other"), "the other scene has a viewport of its own");
  check(vm.hasViewport("right_view") && vm.hasViewport("other_a") && vm.hasViewport("other_b"),
        "each viewport widget made a view");
  if (!vm.hasViewport("scene.other") || !vm.hasViewport("other_a") || !vm.hasViewport("other_b") ||
      !vm.hasViewport("right_view"))
    return cvcgl_test::finish("cvcgl_ariadne_scenes");
  cvc::gl::Viewport &anchor = vm.viewport("scene.other");
  cvc::gl::Viewport &oa = vm.viewport("other_a"), &ob = vm.viewport("other_b");
  cvc::gl::Viewport &rv = vm.viewport("right_view");

  vtkProp *redCrate = nullptr, *blueBall = nullptr;
  if (auto left = sg.getGraphics("left"))
    if (auto n = left->findChildByName("red_crate"))
      redCrate = n->prop();
  if (auto right = sg.getGraphics("right"))
    if (auto n = right->findChildByName("blue_ball"))
      blueBall = n->prop();
  check(redCrate && blueBall, "the main scene's groups are realized");

  // The other scene's views mirror its own viewport, not the main one.
  const int otherProps = anchor.renderer()->GetViewProps()->GetNumberOfItems();
  check(otherProps >= 2, "the other scene's viewport holds its nodes", std::to_string(otherProps));
  check(oa.isMirror() && oa.mirrorSource() == anchor.renderer() &&
            ob.mirrorSource() == anchor.renderer(),
        "both views of the other scene mirror its viewport");
  check(!has_prop(oa.renderer(), redCrate) && !has_prop(oa.renderer(), blueBall),
        "a view of the other scene draws none of the main scene");
  check(oa.renderer()->GetViewProps()->GetNumberOfItems() == otherProps &&
            ob.renderer()->GetViewProps()->GetNumberOfItems() == otherProps,
        "two views of one scene both draw all of it");
  check(oa.renderer()->GetPass() != nullptr, "the other scene's shadows reach its views");
  double bg[3];
  oa.renderer()->GetBackground(bg);
  check(bg[1] > 0.5 && bg[0] < 0.1, "a view of the other scene clears to that scene's background");

  // main_view: the main view leaves the right group out; the right view draws it.
  check(has_prop(rv.renderer(), blueBall), "the right view draws the right group");
  check(has_prop(view.renderer(), blueBall),
        "the main renderer has the right group back after its draw (the views mirror it)");

  std::vector<unsigned char> rgb = view.frameRGB();
  const int W = view.frameWidth(), H = view.frameHeight();
  if (const char *png = std::getenv("CVC_SCENES_PNG"))
    view.writePNG(png);
  double ra[4], rr[4];
  oa.region(ra);
  rv.region(rr);
  const Counts inOther = count(rgb, W, H, ra);
  const Counts inRight = count(rgb, W, H, rr);
  // The main view: below the windows.
  const double below[4] = {0.0, 0.0, 1.0, 0.55};
  const Counts inMain = count(rgb, W, H, below);
  std::printf("  other view: yellow %d green %d red %d blue %d; right view: blue %d; main: red %d "
              "blue %d green %d\n",
              inOther.yellow, inOther.green, inOther.red, inOther.blue, inRight.blue, inMain.red,
              inMain.blue, inMain.green);
  check(inOther.yellow > 50 && inOther.green > 50 && inOther.red == 0 && inOther.blue == 0,
        "the other scene's view: its crate on its background, nothing of the main scene");
  check(inRight.blue > 50, "the right view draws the blue ball");
  check(inMain.red > 50 && inMain.blue == 0 && inMain.green == 0,
        "the main view: the left group only, and not the other scene");

  // A scene's maps are baked on the first frame a view shows it again, whatever its stride: move
  // its crate while no view shows it (the UI hidden), then show the UI again.
  {
    cvc::gl::ariadne::SceneSlot slot;
    if (rt.views().scene("other", slot) && slot.sg) {
      slot.sg->setShadowUpdateInterval(1000);
      for (int f = 0; f < 3; ++f)
        rt.step(kDt);
      ui.setVisible(false);
      for (int f = 0; f < 3; ++f)
        rt.step(kDt);
      check(!anchor.visible(), "the other scene's viewport stops drawing while no view shows it");
      const std::uint64_t bakes = slot.sg->shadowBakes();
      if (auto crate = slot.sg->getGraphics("yellow_crate"))
        crate->setPosition(1.5, 0.5, 0.0);
      for (int f = 0; f < 2; ++f)
        rt.step(kDt);
      check(slot.sg->shadowBakes() == bakes, "nothing bakes the unseen scene's maps");
      ui.setVisible(true);
      rt.step(kDt);
      rt.step(kDt);
      check(anchor.visible() && slot.sg->shadowBakes() > bakes,
            "shown again, the scene's maps are baked for what moved while it was unseen",
            std::to_string(slot.sg->shadowBakes()) + " bakes, " + std::to_string(bakes) +
                " before");
    } else {
      check(false, "the views find the other scene");
    }
  }

  // A new document takes the scenes down.
  rt.load(other.string());
  for (int f = 0; f < 3; ++f)
    rt.step(kDt);
  check(!vm.hasViewport("scene.other") && !vm.hasViewport("other_a"),
        "a new document removes the last one's scenes and their views");

  view.close();
  return cvcgl_test::finish("cvcgl_ariadne_scenes");
}
