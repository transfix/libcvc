// The Ariadne `viewport` widget and node `tags:` (scene_views.h, docs/ARIADNE.md §5.18), run as a
// host runs a document: AriRuntime + ImGuiBackend over an offscreen SceneRenderer.
//
// The scene is a red box (untagged), a blue box tagged sim with a sphere under it tagged debug,
// and a yellow ball tagged debug. One window's viewport shows [sim], another's hides [sim].
// Checked:
//   - each widget makes a mirror viewport where it is drawn, with its own camera state;
//   - a view draws the untagged nodes and those its show: / hide: allow, and a node's tags hold for
//     its subtree (the sphere under the blue box is in neither view);
//   - the pixels: the [sim] view shows red and blue and no yellow where the main view has yellow;
//   - closing a viewport's window hides its view, and opening it again shows it; hiding the UI
//     hides every view;
//   - a new document removes the last one's views.
// Skips (rc 0) where nothing rasterises, unless CVC_REQUIRE_RENDER=1.
#include "streaming_test_util.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/bind.h> // resolve_bind
#include <cvc/core/app.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/GraphicsNode.h>
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
#include <vector>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCollection.h>
#include <vtkMapper.h>
#include <vtkPropCollection.h>
#include <vtkProperty.h>
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
  name: views test
  min_libcvc: 3.7.0

windows:
  - window: Sim view
    id: sim_view
    pos: [20, 20]
    size: [400, 340]
    children:
      - type: viewport
        view: sim
        show: [sim]
        width: 360
        height: 270
  - window: No sim
    id: no_sim
    pos: [520, 20]
    size: [400, 340]
    children:
      - type: viewport
        view: nosim
        hide: sim
        width: 360
        height: 270
  - window: Restyled
    id: restyled
    pos: [520, 380]
    size: [400, 320]
    children:
      - type: viewport
        view: restyled
        azimuth: 45
        elevation: 20
        override:
          "*": { opacity: 0.5 }
          debug: { color: [0.1, 0.1, 0.9], drawstyle: wireframe }
        shadows: false
        width: 360
        height: 270
  - window: Shared
    id: shared
    pos: [20, 380]
    size: [480, 200]
    children:
      - type: group
        layout: { kind: grid, col_widths: [auto, auto, auto] }
        children:
          - { type: viewport, view: pair_a, camera: pair, width: 140, height: 105 }
          - { type: viewport, view: pair_b, camera: pair, hide: debug, width: 140, height: 105 }
          - { type: viewport, view: follow, camera: main, width: 140, height: 105 }

scene:
  chrome: false
  shadows: { enabled: true }
  cameras:
    - camera: pair
      azimuth: -30
      elevation: 40
  nodes:
    - node: red
      type: geometry
      source: { box: { size: [1.5, 1.5, 1.5] } }
      transform: { position: [-3.0, 0.0, 0.0] }
      material: { color: [0.9, 0.1, 0.1] }
    - node: blue
      type: geometry
      tags: [sim]
      source: { box: { size: [1.5, 1.5, 1.5] } }
      transform: { position: [0.0, 0.0, 0.0] }
      material: { color: [0.1, 0.1, 0.9] }
      children:
        - node: antenna
          type: geometry
          tags: [debug]
          source: { sphere: 0.3 }
          transform: { position: [0.0, 0.0, 1.2] }
          material: { color: [0.1, 0.9, 0.1] }
    - node: yellow
      type: geometry
      tags: debug
      source: { sphere: 0.8 }
      transform: { position: [3.0, 0.0, 0.0] }
      material: { color: [0.9, 0.9, 0.1] }
)";

const char *kOtherDoc = R"(meta:
  name: no views
  min_libcvc: 3.7.0
windows:
  - window: Plain
    id: plain
    children:
      - text: "No viewport here."
)";

bool has_prop(vtkRenderer *r, vtkProp *p) {
  return r && p && r->GetViewProps()->IsItemPresent(p) != 0;
}

struct Counts {
  int red = 0, blue = 0, yellow = 0;
};

// Pixels by colour inside the normalized y-up rect `reg` of an RGB frame (rows bottom-up).
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
    }
  return c;
}

} // namespace

int main() {
  const bool require = std::getenv("CVC_REQUIRE_RENDER") != nullptr;
  const fs::path dir = fs::temp_directory_path() / "cvcgl_ariadne_views";
  fs::create_directories(dir);
  const fs::path doc = dir / "views.ari", other = dir / "other.ari";
  std::ofstream(doc) << kDoc;
  std::ofstream(other) << kOtherDoc;

  cvc::app app;
  app.properties("system.log_verbosity", "0");
  cvc::state &st = cvc::state::instance(app);
  cvc::gl::SceneGraph sg(app, "views");
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
  if (!ui.enabled()) { // ImGui's GL backend could not start: nothing rasterises here
    std::printf("SKIP: nothing rasterises\n");
    return require ? 1 : 0;
  }

  cvc::gl::ViewportManager &vm = view.viewportManager();
  check(rt.views().size() == 6, "each viewport widget made a view",
        std::to_string(rt.views().size()));
  check(vm.hasViewport("sim") && vm.hasViewport("nosim"), "the views are the manager's viewports");
  if (!vm.hasViewport("sim") || !vm.hasViewport("nosim"))
    return cvcgl_test::finish("cvcgl_ariadne_views");
  cvc::gl::Viewport &sim = vm.viewport("sim"), &nosim = vm.viewport("nosim");
  check(sim.isMirror() && sim.visible(), "the sim view is a mirror and shows");
  double reg[4];
  sim.region(reg);
  check(reg[2] > reg[0] && reg[3] > reg[1] && reg[2] < 0.5 && reg[1] > 0.4,
        "the sim view is where its widget is, in the first window",
        std::to_string(reg[0]) + " " + std::to_string(reg[1]) + " " + std::to_string(reg[2]) + " " +
            std::to_string(reg[3]));

  auto prop = [&](const std::string &id) {
    std::shared_ptr<cvc::gl::GraphicsNode> n = sg.getGraphics(id);
    return n ? n->prop() : nullptr;
  };
  std::shared_ptr<cvc::gl::GraphicsNode> blue = sg.getGraphics("blue");
  std::shared_ptr<cvc::gl::GraphicsNode> antenna =
      blue ? blue->findChildByName("antenna") : nullptr;
  check(antenna != nullptr, "the tagged child is realized under its parent");
  vtkProp *antennaProp = antenna ? antenna->prop() : nullptr;
  vtkRenderer *main = view.renderer();
  check(has_prop(main, prop("red")) && has_prop(main, prop("blue")) &&
            has_prop(main, prop("yellow")) && has_prop(main, antennaProp),
        "the main view draws every node");
  check(has_prop(sim.renderer(), prop("red")), "show [sim]: the untagged node is in");
  check(has_prop(sim.renderer(), prop("blue")), "show [sim]: the sim node is in");
  check(!has_prop(sim.renderer(), prop("yellow")), "show [sim]: the debug node is out");
  check(!has_prop(sim.renderer(), antennaProp),
        "show [sim]: the debug child of the sim node is out");
  check(has_prop(nosim.renderer(), prop("red")), "hide sim: the untagged node is in");
  check(has_prop(nosim.renderer(), prop("yellow")), "hide sim: the debug node is in");
  check(!has_prop(nosim.renderer(), prop("blue")), "hide sim: the sim node is out");
  check(!has_prop(nosim.renderer(), antennaProp),
        "hide sim: the sim node's child goes with it, whatever its own tags");

  // Each view's camera is its own, at <prefix>.viewers.<name>.camera.
  const std::string poseKey = cvc::ariadne::resolve_bind("views", "viewers.sim.camera");
  check(st(poseKey).children().size() > 0, "the sim view's camera is in the state tree", poseKey);

  // override: the restyled view draws a twin of each actor (its own property, the SAME mapper, so
  // no geometry is copied); "*" restyles everything and a later tag entry adds to it; the main
  // view's actors keep their own look.
  if (vm.hasViewport("restyled")) {
    cvc::gl::Viewport &restyled = vm.viewport("restyled");
    auto twin_of = [&](vtkProp *orig) -> vtkActor * {
      vtkActor *a = vtkActor::SafeDownCast(orig);
      vtkPropCollection *props = restyled.renderer()->GetViewProps();
      vtkCollectionSimpleIterator it;
      props->InitTraversal(it);
      while (vtkProp *p = props->GetNextProp(it))
        if (vtkActor *b = vtkActor::SafeDownCast(p))
          if (a && b != a && b->GetMapper() == a->GetMapper())
            return b;
      return nullptr;
    };
    vtkActor *redA = vtkActor::SafeDownCast(prop("red"));
    vtkActor *yellowA = vtkActor::SafeDownCast(prop("yellow"));
    vtkActor *redTwin = twin_of(redA), *yellowTwin = twin_of(yellowA);
    check(!has_prop(restyled.renderer(), redA) && !has_prop(restyled.renderer(), yellowA),
          "override: the restyled view draws twins, not the main view's actors");
    check(redTwin && yellowTwin, "override: each twin shares its original's mapper");
    if (redTwin && yellowTwin && redA && yellowA) {
      double c[3];
      redTwin->GetProperty()->GetColor(c);
      check(std::fabs(redTwin->GetProperty()->GetOpacity() - 0.5) < 1e-9 && c[0] > 0.8 &&
                c[2] < 0.2,
            "override \"*\": half opacity, the node's own colour kept");
      yellowTwin->GetProperty()->GetColor(c);
      check(std::fabs(yellowTwin->GetProperty()->GetOpacity() - 0.5) < 1e-9 && c[2] > 0.8 &&
                c[0] < 0.2 && yellowTwin->GetProperty()->GetRepresentation() == VTK_WIREFRAME,
            "override debug: blue wireframe, on top of \"*\"'s opacity");
      yellowA->GetProperty()->GetColor(c);
      check(yellowA->GetProperty()->GetOpacity() == 1.0 && c[0] > 0.8 && c[1] > 0.8 &&
                yellowA->GetProperty()->GetRepresentation() == VTK_SURFACE,
            "the main view's yellow ball keeps its own look");
    }
    // azimuth: / elevation: are where its orbit starts.
    const std::string orbit = cvc::ariadne::resolve_bind("views", "viewers.restyled.camera.orbit");
    double az = 0.0, el = 0.0;
    try {
      az = st(orbit + ".azimuth").value<double>();
      el = st(orbit + ".elevation").value<double>();
    } catch (...) {
    }
    check(std::fabs(az - 45.0) < 1e-6 && std::fabs(el - 20.0) < 1e-6,
          "azimuth / elevation: the orbit starts where the widget says",
          std::to_string(az) + " " + std::to_string(el));
  } else {
    check(false, "the restyled view exists");
  }

  // The pixels: the sim view's rect shows red and blue and no yellow (the main view's yellow ball
  // would be under it otherwise; the window's background is cut away there).
  std::vector<unsigned char> rgb = view.frameRGB();
  const int W = view.frameWidth(), H = view.frameHeight();
  const Counts inSim = count(rgb, W, H, reg);
  const double whole[4] = {0.0, 0.0, 1.0, 1.0};
  const Counts all = count(rgb, W, H, whole);
  std::printf("  sim view: red %d blue %d yellow %d; whole frame: yellow %d\n", inSim.red,
              inSim.blue, inSim.yellow, all.yellow);
  check(inSim.red > 50 && inSim.blue > 50, "the sim view draws the red and blue boxes");
  check(inSim.yellow == 0, "the sim view draws no yellow");
  check(all.yellow > 50, "the main view draws the yellow ball");
  if (const char *png = std::getenv("CVC_VIEWS_PNG")) // the frame, to look at
    view.writePNG(png);

#ifdef CVC_ENABLE_IMGUI
  // A drag in the sim view turns its camera and not the main one; the wheel there zooms it.
  {
    ImGui::SetCurrentContext(ui.imguiContext());
    ImGuiIO &io = ImGui::GetIO();
    const float cx = static_cast<float>(0.5 * (reg[0] + reg[2]) * W);
    const float cy = static_cast<float>((1.0 - 0.5 * (reg[1] + reg[3])) * H);
    double mainEye[3], mainFocal[3], mainUp[3], eye[3], focal[3], up[3];
    cam.getPose(mainEye, mainFocal, mainUp);
    sim.camera().getPose(eye, focal, up);
    const double dist = sim.camera().orbitDistance();
    io.AddMousePosEvent(cx, cy);
    rt.step(kDt);
    io.AddMouseButtonEvent(0, true);
    rt.step(kDt);
    for (int i = 1; i <= 4; ++i) {
      io.AddMousePosEvent(cx + 15.0f * i, cy);
      rt.step(kDt);
    }
    io.AddMouseButtonEvent(0, false);
    rt.step(kDt);
    double mainEye2[3], eye2[3];
    cam.getPose(mainEye2, mainFocal, mainUp);
    sim.camera().getPose(eye2, focal, up);
    auto moved = [](const double a[3], const double b[3]) {
      return std::fabs(a[0] - b[0]) + std::fabs(a[1] - b[1]) + std::fabs(a[2] - b[2]) > 1e-6;
    };
    check(moved(eye, eye2), "a drag in the sim view turns its camera");
    check(!moved(mainEye, mainEye2), "a drag in the sim view leaves the main camera alone");
    io.AddMouseWheelEvent(0.0f, 2.0f);
    rt.step(kDt);
    rt.step(kDt);
    check(sim.camera().orbitDistance() < dist, "the wheel over the sim view zooms it in",
          std::to_string(dist) + " -> " + std::to_string(sim.camera().orbitDistance()));
    io.AddMousePosEvent(-1.0f, -1.0f);
    rt.step(kDt);
  }

  // Named cameras: pair_a and pair_b draw from one camera, so a drag in one moves both and nothing
  // else; `camera: main` draws from the main view's camera, and a drag there moves the main view.
  if (vm.hasViewport("pair_a") && vm.hasViewport("pair_b") && vm.hasViewport("follow")) {
    cvc::gl::Viewport &pa = vm.viewport("pair_a"), &pb = vm.viewport("pair_b");
    cvc::gl::Viewport &follow = vm.viewport("follow");
    vtkCamera *shared = pa.renderer()->GetActiveCamera();
    check(shared == pb.renderer()->GetActiveCamera(), "camera: pair: both views draw from one camera");
    check(shared != view.renderer()->GetActiveCamera() &&
              shared != sim.renderer()->GetActiveCamera(),
          "camera: pair: not the main view's, nor another view's");
    check(follow.renderer()->GetActiveCamera() == view.renderer()->GetActiveCamera(),
          "camera: main: the view draws from the main view's camera");
    const std::string pairAz = cvc::ariadne::resolve_bind("views", "viewers.pair.camera.orbit");
    double az = 0.0;
    try {
      az = st(pairAz + ".azimuth").value<double>();
    } catch (...) {
    }
    check(std::fabs(az - (-30.0)) < 1e-6, "the named camera starts where cameras: says",
          std::to_string(az));
    ImGui::SetCurrentContext(ui.imguiContext());
    ImGuiIO &io = ImGui::GetIO();
    auto drag = [&](cvc::gl::Viewport &vp) {
      double r[4];
      vp.region(r);
      const float x = static_cast<float>(0.5 * (r[0] + r[2]) * W);
      const float y = static_cast<float>((1.0 - 0.5 * (r[1] + r[3])) * H);
      io.AddMousePosEvent(x, y);
      rt.step(kDt);
      io.AddMouseButtonEvent(0, true);
      rt.step(kDt);
      for (int i = 1; i <= 4; ++i) {
        io.AddMousePosEvent(x + 10.0f * i, y);
        rt.step(kDt);
      }
      io.AddMouseButtonEvent(0, false);
      rt.step(kDt);
      io.AddMousePosEvent(-1.0f, -1.0f);
      rt.step(kDt);
    };
    double before[3], after[3], mainBefore[3], mainAfter[3], simBefore[3], simAfter[3];
    shared->GetPosition(before);
    view.renderer()->GetActiveCamera()->GetPosition(mainBefore);
    sim.renderer()->GetActiveCamera()->GetPosition(simBefore);
    drag(pb);
    shared->GetPosition(after);
    view.renderer()->GetActiveCamera()->GetPosition(mainAfter);
    sim.renderer()->GetActiveCamera()->GetPosition(simAfter);
    auto moved = [](const double a[3], const double b[3]) {
      return std::fabs(a[0] - b[0]) + std::fabs(a[1] - b[1]) + std::fabs(a[2] - b[2]) > 1e-6;
    };
    check(moved(before, after), "a drag in pair_b moves the shared camera, so pair_a moves too");
    check(!moved(mainBefore, mainAfter) && !moved(simBefore, simAfter),
          "a drag in pair_b leaves the main view and the sim view alone");
    view.renderer()->GetActiveCamera()->GetPosition(mainBefore);
    drag(follow);
    view.renderer()->GetActiveCamera()->GetPosition(mainAfter);
    check(moved(mainBefore, mainAfter), "a drag in the camera: main view moves the main view");
  } else {
    check(false, "the shared-camera views exist");
  }
#endif

  // Shadows: a view receives the scene's shadows through a pass of its own (reading the main
  // view's maps); `shadows: false` leaves it without.
  check(sim.renderer()->GetPass() != nullptr, "a view draws the scene's shadows (it has a pass)");
  if (vm.hasViewport("restyled"))
    check(vm.viewport("restyled").renderer()->GetPass() == nullptr,
          "shadows: false: that view has no shadow pass");

  // Closing the window hides its view; opening it shows it again.
  const std::string open = cvc::ariadne::resolve_bind("views", "tree.sim_view.open");
  st(open).value(0);
  for (int f = 0; f < 3; ++f)
    rt.step(kDt);
  check(!sim.visible(), "a closed window's view is hidden");
  check(nosim.visible(), "the other window's view still shows");
  st(open).value(1);
  for (int f = 0; f < 3; ++f)
    rt.step(kDt);
  check(sim.visible(), "an opened window's view shows again");

  // The host hiding the whole UI hides the views with it.
  ui.setVisible(false);
  for (int f = 0; f < 3; ++f)
    rt.step(kDt);
  check(!sim.visible() && !nosim.visible(), "hiding the UI hides every view");
  ui.setVisible(true);
  for (int f = 0; f < 3; ++f)
    rt.step(kDt);
  check(sim.visible() && nosim.visible(), "showing the UI again shows them");

  // A new document takes the last one's views with it.
  rt.load(other.string());
  for (int f = 0; f < 3; ++f)
    rt.step(kDt);
  check(rt.views().size() == 0 && !vm.hasViewport("sim") && !vm.hasViewport("nosim"),
        "a new document removes the last one's views");

  // A host that closes the view before the runtime goes (cvcgl_ari_switch, arigl at exit): the
  // views went with the view's viewport manager, and the runtime's teardown must not reach for it.
  rt.load(doc.string());
  for (int f = 0; f < 3; ++f)
    rt.step(kDt);
  check(rt.views().size() == 6, "the views are back with the document");
  view.close(); // ~AriRuntime runs after the return below, with the view closed
  return cvcgl_test::finish("cvcgl_ariadne_views");
}
