// cvcgl_ariadne_node_pick — a scene node's own `on_pick:` (docs/ARIADNE.md §5.18, "A node's own
// on_pick:"). A press in an `input: pick` viewport runs the on_pick: of the node it lands on, or of
// its nearest ancestor that has one; that press's drags and release run the same program wherever
// they land; it runs in the document's scope (its prefix and its functions) although the viewport
// sits in a load:ed module; and the view's own on_pick: runs too, at the module's prefix.
// Offscreen; skips where nothing rasterises unless CVC_REQUIRE_RENDER=1.

#include "streaming_test_util.h"

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

// The document: four boxes along x. The crate and the lid have handlers of their own; the jar has
// none, so its parent's (the shelf's) runs; the post has none and no ancestor with one. Each
// handler calls the document's `note`, which the module holding the viewport is not granted.
const char *kDoc = R"(meta:
  name: node pick test
  min_libcvc: 3.7.0

functions: |
  (defun note (who ev)
    (state-set "log" (str-concat (state-get "log") who ":" (get-attr ev "phase") ":"
                                 (get-attr ev "node") " ")))

init: (begin (state-set "log" "") (state-set "includes.panel.log" ""))

windows:
  - window: Pick
    id: pick_window
    pos: [20, 20]
    size: [740, 400]
    children:
      - load: panel.ari
        as: panel

scene:
  nodes:
    - node: crate
      source: { box: { size: [2, 2, 2] } }
      transform: { position: [-6, 0, 0] }
      on_pick: (note "crate" event)
    - node: shelf
      type: group
      on_pick: (note "shelf" event)
      children:
        - node: jar
          source: { box: 1 }
          transform: { position: [-2, 0, 0] }
        - node: lid
          source: { box: 1 }
          transform: { position: [2, 0, 0] }
          on_pick: (note "lid" event)
    - node: post
      source: { box: 1 }
      transform: { position: [6, 0, 0] }
)";

// The module: the pick view. Its on_pick: runs at the module's prefix.
const char *kPanel = R"(root:
  - type: viewport
    view: picker
    input: pick
    pick: hits
    on_pick: (state-set "log" (str-concat (state-get "log") "view:" (get-attr event "phase") ":" (get-attr event "node") " "))
    width: 700
    height: 320
)";

std::string val(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app)(path).value();
}

bool starts_with(const std::string &s, const std::string &p) { return s.rfind(p, 0) == 0; }
bool ends_with(const std::string &s, const std::string &p) {
  return s.size() >= p.size() && s.compare(s.size() - p.size(), p.size(), p) == 0;
}

} // namespace

int main() {
  const bool require = std::getenv("CVC_REQUIRE_RENDER") != nullptr;
#ifndef CVC_ENABLE_IMGUI
  std::printf("SKIP: no ImGui\n");
  return require ? 1 : 0;
#else
  const fs::path dir = fs::temp_directory_path() / "cvcgl_ariadne_node_pick";
  fs::create_directories(dir);
  const fs::path doc = dir / "node_pick.ari";
  std::ofstream(doc) << kDoc;
  std::ofstream(dir / "panel.ari") << kPanel;

  cvc::app app;
  app.properties("system.log_verbosity", "0");
  cvc::gl::SceneGraph sg(app, "np");
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
  check(vm.hasViewport("picker"), "the module's viewport exists");
  if (!vm.hasViewport("picker"))
    return cvcgl_test::finish("cvcgl_ariadne_node_pick");
  cvc::gl::Viewport &picker = vm.viewport("picker");
  vtkRenderer *ren = picker.renderer();
  // A world point's place on screen: VTK display coordinates (y up), and ImGui's (y down).
  struct Spot {
    double x, y;
    float mx, my;
  };
  const auto spot = [&](double wx) {
    ren->SetWorldPoint(wx, 0.0, 0.0, 1.0);
    ren->WorldToDisplay();
    double d[3];
    ren->GetDisplayPoint(d);
    return Spot{d[0], d[1], static_cast<float>(d[0]), static_cast<float>(kHeight - d[1])};
  };
  const Spot crate = spot(-6.0), jar = spot(-2.0), lid = spot(2.0), post = spot(6.0),
             gap = spot(0.0);
  {
    double rg[4];
    picker.region(rg);
    bool inside = true;
    for (const Spot *s : {&crate, &jar, &lid, &post, &gap})
      inside = inside && s->x > rg[0] * kWidth && s->x < rg[2] * kWidth && s->y > rg[1] * kHeight &&
               s->y < rg[3] * kHeight;
    check(inside, "every box is on the view's screen, where a press reaches it");
  }

  // SceneViews::pick names the handler a press would run.
  using PickHit = cvc::gl::ariadne::SceneViews::PickHit;
  const auto handler = [&](const Spot &s, const std::string &node) {
    PickHit h;
    const bool got = rt.views().pick("picker", s.x, s.y, h);
    check(got && h.node == node, "a pick there hits " + node, h.node);
    return h.on_pick;
  };
  check(handler(crate, "crate") == "(note \"crate\" event)", "the crate runs its own on_pick:");
  check(handler(jar, "jar") == "(note \"shelf\" event)",
        "the jar, which has none, runs its parent's");
  check(handler(lid, "lid") == "(note \"lid\" event)", "the lid runs its own, not its parent's");
  check(handler(post, "post").empty(), "the post, with none above it, runs none");
  PickHit miss;
  check(!rt.views().pick("picker", gap.x, gap.y, miss) && miss.on_pick.empty(), "a miss runs none");

  ImGui::SetCurrentContext(ui.imguiContext());
  ImGuiIO &io = ImGui::GetIO();
  const std::string panel = "np.includes.panel";
  const auto clear_logs = [&] {
    cvc::state::instance(app)("np.log").value(std::string());
    cvc::state::instance(app)(panel + ".log").value(std::string());
  };
  const auto click = [&](const Spot &s) {
    io.AddMousePosEvent(s.mx, s.my);
    rt.step(kDt);
    io.AddMouseButtonEvent(0, true);
    rt.step(kDt);
    rt.step(kDt);
    io.AddMouseButtonEvent(0, false);
    rt.step(kDt);
    rt.step(kDt);
  };

  // A press on the crate, dragged onto the post and let go there: the crate keeps the press.
  clear_logs();
  io.AddMousePosEvent(crate.mx, crate.my);
  rt.step(kDt);
  io.AddMouseButtonEvent(0, true);
  rt.step(kDt);
  rt.step(kDt);
  for (int i = 1; i <= 4; ++i) {
    io.AddMousePosEvent(crate.mx + (post.mx - crate.mx) * i / 4.0f,
                        crate.my + (post.my - crate.my) * i / 4.0f);
    rt.step(kDt);
  }
  io.AddMouseButtonEvent(0, false);
  rt.step(kDt);
  rt.step(kDt);
  for (const std::string &w : rt.take_warnings())
    std::printf("  warning: %s\n", w.c_str());
  const std::string log = val(app, "np.log");
  check(starts_with(log, "crate:down:crate "), "the press on the crate ran the crate's", log);
  check(log.find("crate:drag:") != std::string::npos, "its drags ran the crate's", log);
  check(ends_with(log, "crate:up:post "), "and its release over the post ran the crate's too", log);
  check(log.find("shelf:") == std::string::npos && log.find("lid:") == std::string::npos,
        "the boxes the drag crossed ran nothing", log);
  const std::string viewLog = val(app, panel + ".log");
  check(viewLog.find("crate:") == std::string::npos,
        "the node's handler wrote the document's log, not the module's", viewLog);
  check(starts_with(viewLog, "view:down:crate ") && ends_with(viewLog, "view:up:post "),
        "the view's on_pick: ran too, at the module's prefix", viewLog);

  clear_logs();
  click(jar);
  check(val(app, "np.log") == "shelf:down:jar shelf:up:jar ", "a click on the jar ran the shelf's",
        val(app, "np.log"));
  check(val(app, panel + ".log") == "view:down:jar view:up:jar ", "and the view's each time",
        val(app, panel + ".log"));

  clear_logs();
  click(lid);
  check(val(app, "np.log") == "lid:down:lid lid:up:lid ", "a click on the lid ran the lid's alone",
        val(app, "np.log"));

  clear_logs();
  click(post);
  check(val(app, "np.log").empty(), "a click on the post ran no node's", val(app, "np.log"));
  check(val(app, panel + ".log") == "view:down:post view:up:post ", "but the view's ran",
        val(app, panel + ".log"));

  clear_logs();
  click(gap);
  check(val(app, "np.log").empty(), "a click on nothing ran no node's", val(app, "np.log"));
  io.AddMousePosEvent(-1.0f, -1.0f);
  rt.step(kDt);

  view.close(); // ~AriRuntime runs after the return below, with the view closed
  return cvcgl_test::finish("cvcgl_ariadne_node_pick");
#endif
}
