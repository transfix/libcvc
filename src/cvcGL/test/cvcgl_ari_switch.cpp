// AriRuntime switching documents, offscreen: a host (arigl's gallery) loads one .ari, then another
// in its place. The second must start on an empty stage -- the first's nodes (with any a custom
// type added beside its own), its free light, its lighting rig, its background and its shadows
// gone -- and a reload of one document must not pile its nodes up. The host's menus
// (set_host_menus) ride along into each document it loads.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/bind.h> // resolve_bind
#include <cvc/core/app.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ariadne/AriRuntime.h>
#include <cvc/gl/ariadne/scene_realize.h> // RealizedScene
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vtkRenderer.h>

using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;

static int fails = 0;
static void chk(bool ok, const std::string &what) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok)
    ++fails;
}

static std::string write_doc(const std::string &name, const std::string &text) {
  const std::filesystem::path p = std::filesystem::temp_directory_path() / name;
  std::ofstream(p) << text;
  return p.string();
}

int main() {
  if (!cvc::gl::ariadne::AriRuntime::have_yaml()) {
    std::printf("cvcgl_ari_switch: SKIP (libcvc built without yaml-cpp)\n");
    return 0;
  }
  const std::string a = write_doc("cvcgl_ari_switch_a.ari", R"(meta: {min_libcvc: 3.4.0}
menubar:
  - menu: File
    items:
      - menu_item: Quit
        on: quit
scene:
  background: { top: [0.9, 0.1, 0.1], bottom: [0.1, 0.9, 0.1] }
  shadows: { enabled: true }
  nodes:
    - node: crate
      source: { box: 2 }
    - node: ball
      source: { sphere: 1 }
      transform: { position: [3, 0, 0] }
    - node: grove            # a custom type that adds a second top-level node (grove_needles)
      type: forest_trees
      count: 4
      span: 5
      sea_level: -1          # no ground: planted on a flat plane at 0
      levels: 2
  lights:
    - light: lamp
      kind: directional
      azimuth: 45
      elevation: 60
    - light: studio
      rig: three_point
)");
  const std::string b = write_doc("cvcgl_ari_switch_b.ari", R"(meta: {min_libcvc: 3.4.0}
scene:
  nodes:
    - node: floor
      source: { plane: { size: 10 } }
)");

  cvc::app app;
  SceneGraph sg(app, "switch");
  SceneRenderer view(sg, 160, 120, /*offscreen=*/true, "switch");
  view.setBackground(0.2, 0.3, 0.4);
  cvc::gl::CameraController cam(view);
  cvc::gl::ImGuiOverlay ui(view);
  ui.setVisible(true);
  cvc::gl::ariadne::AriRuntime rt(view, cam, ui);
  rt.app_runtime().set_host_menus(
      {cvc::ariadne::menu("Demos", {cvc::ariadne::menu_action("b", "gallery:1")})});
  const bool shadows_before = sg.shadowsEnabled();

  std::printf("== load a: two nodes, a free light, a rig, a background, shadows ==\n");
  rt.load(a);
  rt.step(1.0 / 30.0);
  chk(sg.getGraphics("crate") && sg.getGraphics("ball"), "a's nodes are in the scene");
  chk(sg.getGraphics("grove") && sg.getGraphics("grove_needles"),
      "a's forest is in the scene, its needles beside it");
  chk(sg.getGraphics("lamp") != nullptr, "a's free light is in the scene");
  chk(rt.realized() && !rt.realized()->rigs.empty(), "a's rig is realized");

  std::printf("== reload a: nothing piles up ==\n");
  const std::size_t before = sg.getAllGraphics().size();
  rt.load(a);
  rt.step(1.0 / 30.0);
  chk(sg.getAllGraphics().size() == before, "the same nodes after a reload as before it");

  std::printf("== load b in its place ==\n");
  rt.load(b);
  rt.step(1.0 / 30.0);
  chk(!sg.getGraphics("crate") && !sg.getGraphics("ball"), "a's nodes are gone");
  chk(!sg.getGraphics("grove") && !sg.getGraphics("grove_needles"),
      "a's forest is gone, needles and all");
  chk(!sg.getGraphics("lamp"), "a's free light is gone");
  chk(sg.getGraphics("floor") != nullptr, "b's node is there");
  chk(rt.realized() && rt.realized()->rigs.empty(), "no rig left from a");
  double bg[3] = {0, 0, 0};
  bool gradient = true;
  if (vtkRenderer *r = view.renderer()) {
    r->GetBackground(bg);
    gradient = r->GetGradientBackground();
  }
  chk(!gradient && std::abs(bg[0] - 0.2) < 1e-6 && std::abs(bg[1] - 0.3) < 1e-6 &&
          std::abs(bg[2] - 0.4) < 1e-6,
      "the host's background is back (no gradient from a)");
  chk(sg.shadowsEnabled() == shadows_before, "shadows are as the host had them");
  chk(ui.phoneHidesPanels(), "a document with a scene: a phone starts with its panels hidden");

  std::printf("== load c: a UI with no scene ==\n");
  rt.load(write_doc("cvcgl_ari_switch_c.ari", "meta: {min_libcvc: 3.0.0}\n"
                                              "windows:\n"
                                              "  - window: Feed\n"
                                              "    children:\n"
                                              "      - text: a post\n"));
  rt.step(1.0 / 30.0);
  chk(!rt.realized(), "c has no scene");
  chk(!ui.phoneHidesPanels(), "so a phone starts with its panels shown");

  // Adaptive quality (scene_quality.h, O21): the loop runs on the frames' wall time, never on the
  // fixed clock (a capture's frames stay as asked for), and publishes quality.* under the prefix.
  std::printf("== adaptive quality: slow frames step the tier down; the fixed clock holds it ==\n");
  rt.load(b);
  auto q = [&](const char *k) -> cvc::state & {
    return cvc::state::instance(app)(cvc::ariadne::resolve_bind(rt.prefix(), k));
  };
  auto slow_frames = [&](double seconds) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < end) {
      rt.step(1.0 / 30.0);
      std::this_thread::sleep_for(std::chrono::milliseconds(150)); // 6.7 fps, far under 30
    }
  };
  const std::string window0 = q("quality.frame_ms").value<std::string>();
  rt.set_fixed_clock(true);
  slow_frames(2.6);
  chk(q("quality.frame_ms").value<std::string>() == window0 &&
          q("quality.tier").value<std::string>() != "1" &&
          q("quality.tier").value<std::string>() != "2" &&
          q("quality.tier").value<std::string>() != "3" &&
          q("quality.tier").value<std::string>() != "4",
      "on the fixed clock the loop does not run: no window closes, no tier drops");
  rt.set_fixed_clock(false);
  slow_frames(3.2); // the start's 2 s cooldown, then a window at 4.5 x the 33 ms target: a panic
  const double tier = q("quality.tier").value<double>(), ms = q("quality.frame_ms").value<double>();
  std::printf("  after 3.2 s at about 150 ms a frame: tier %.0f, scale %.2f, frame %.0f ms\n", tier,
              q("quality.scale").value<double>(), ms);
  chk(tier >= 2.0 && ms > 100.0 && q("quality.target_fps").value<double>() == 30.0,
      "slow frames drop tiers within seconds, and the loop publishes its window's frame time");
  q("quality.adaptive").value(0.0);
  q("quality.tier").value(0.0);
  slow_frames(2.6);
  chk(q("quality.tier").value<double>() == 0.0,
      "quality.adaptive 0 holds the tier where it is set, however slow the frames");

  view.close();
  std::printf("cvcgl_ari_switch: %s\n", fails ? "FAILED" : "PASSED");
  return fails ? 1 : 0;
}
