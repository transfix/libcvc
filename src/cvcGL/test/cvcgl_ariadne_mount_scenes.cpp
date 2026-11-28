// cvcgl_ariadne_mount_scenes — scenes of mounted documents (docs/roadmap/ARIADNE_SHELL.md 4.3, 4.4;
// SH3), run as a host runs a document: AriRuntime + ImGuiBackend over an offscreen SceneRenderer.
// A shell document opens four applications in a `mounts:` slot, each a document with a scene.
// Checked:
//   - each mount's scene is realized apart, as a SceneGraph of its own at its sub-prefix: its nodes
//     in its own graph only, its camera at <sub>.viewers.main.camera, a viewport of its own;
//   - a relative bind in a mounted scene (a node's visible:) resolves under its own sub-prefix, as
//     it did when the document ran alone, and the shell's key of the same name moves nothing;
//   - each application's scene window draws its scene's own viewport into the window's texture,
//     and a `viewport` with `scene: { mount: <as> }` in the shell is a further view of it;
//   - each world steps on a clock of its own: two at their own rates (clock: hz), one without a
//     rate at the shell's;
//   - a world that runs in the background keeps stepping while its window is closed, and one the
//     application holds (background: hold) is not stepped while its window is closed, and goes on
//     when it is opened again;
//   - closing an application takes its scene, its viewports and its clock down.
// Skips (rc 0) where nothing rasterises, unless CVC_REQUIRE_RENDER=1.
#include "streaming_test_util.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/core/scene_clocks.h>
#include <cvc/core/world_clock.h>
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
#include <memory>
#include <string>

namespace fs = std::filesystem;
using cvcgl_test::check;

namespace {

constexpr int kWidth = 960, kHeight = 720;
constexpr double kDt = 1.0 / 30.0;

// The shell: a scene of its own whose world runs at 50 Hz (a scene's clock: hz sets its clock's
// rate when it has a world, here an empty physics one), a window with a further view of the
// application `a`, and the slot its applications open in.
const char *kShellDoc = R"(meta: { name: Shell, min_libcvc: "0.0.0" }
scene:
  chrome: false
  clock: { hz: 50 }
  physics: { backend: "null" }
  nodes:
    - { node: floor, source: { box: { size: [4, 4, 0.1] } }, material: { color: [0.3, 0.3, 0.3] } }
windows:
  - window: Thumbs
    id: thumbs
    pos: [620, 420]
    size: [300, 260]
    children:
      - { type: viewport, view: thumb, scene: { mount: a }, chrome: false, shadows: false, input: false, width: 240, height: 180 }
root:
  - mounts: apps
)";

// An application: one node whose visibility is the document's own `show` key, and a scene clock
// at `hz` (none: no clock: block at all).
std::string app_doc(const std::string &name, const std::string &node, int hz) {
  return "meta: { name: " + name + ", min_libcvc: \"0.0.0\" }\nscene:\n  chrome: false\n" +
         (hz > 0 ? "  clock: { hz: " + std::to_string(hz) + " }\n" : std::string()) +
         "  nodes:\n    - { node: " + node +
         ", source: { box: { size: [2, 2, 2] } }, visible: show, material: { color: [0.9, 0.6, "
         "0.1] } }\n";
}

// An offscreen host, as the other Ariadne view tests make one.
struct Host {
  cvc::app app;
  std::unique_ptr<cvc::gl::SceneGraph> sg;
  std::unique_ptr<cvc::gl::SceneRenderer> view;
  std::unique_ptr<cvc::gl::CameraController> cam;
  std::unique_ptr<cvc::gl::ImGuiOverlay> ui;
  explicit Host(const char *prefix) {
    app.properties("system.log_verbosity", "0");
    sg = std::make_unique<cvc::gl::SceneGraph>(app, prefix);
    sg->setDiagnosticChromeVisible(false);
    view = std::make_unique<cvc::gl::SceneRenderer>(*sg, kWidth, kHeight, true, "main");
    view->setBackground(0.0, 0.0, 0.0);
    cam = std::make_unique<cvc::gl::CameraController>(*view);
    ui = std::make_unique<cvc::gl::ImGuiOverlay>(*view);
    if (!ui->enabled())
      return;
    ui->attachCamera(*cam);
    ui->setVisible(true);
    ui->setTouchMode(false);
    ui->setUiScale(1.0f);
    ui->setToggleButtonEnabled(false);
    ui->setPanelsOpen(true);
    ui->setFixedDeltaTime(static_cast<float>(kDt));
  }
};

std::uint64_t ticks(cvc::app &app, const std::string &sub) {
  return cvc::scene_clock(app, sub).tick();
}

} // namespace

int main() {
  const bool require = std::getenv("CVC_REQUIRE_RENDER") != nullptr;
#ifndef CVC_ENABLE_IMGUI
  std::printf("SKIP: no ImGui\n");
  return require ? 1 : 0;
#else
  const fs::path dir = fs::temp_directory_path() / "cvcgl_ariadne_mount_scenes";
  fs::create_directories(dir);
  std::ofstream(dir / "shell.ari") << kShellDoc;
  std::ofstream(dir / "a.ari") << app_doc("Thirty", "box_a", 30);
  std::ofstream(dir / "b.ari") << app_doc("Sixty", "box_b", 60);
  std::ofstream(dir / "c.ari") << app_doc("Shells", "box_c", 0);
  std::ofstream(dir / "d.ari") << app_doc("Held", "box_d", 40);

  Host h("mounts");
  if (!h.ui->enabled()) {
    std::printf("SKIP: no ImGui overlay\n");
    return require ? 1 : 0;
  }
  cvc::gl::ariadne::AriRuntime rt(*h.view, *h.cam, *h.ui);
  for (const std::string &w : rt.load((dir / "shell.ari").string()))
    check(false, "the shell loads without warnings", w);
  cvc::state &tree = cvc::state::instance(h.app);
  for (const char *as : {"a", "b", "c", "d"})
    tree(std::string("mounts.apps.") + as + ".uri")
        .value((dir / (std::string(as) + ".ari")).generic_string());
  tree("mounts.apps.d.background").value("hold");

  cvc::gl::ViewportManager &vm = h.view->viewportManager();
  const auto sub = [](const char *as) { return std::string("mounts.includes.") + as; };
  bool all = false;
  for (int frame = 0; frame < 20 && !all; ++frame) {
    rt.step(kDt);
    all = true;
    for (const char *as : {"a", "b", "c", "d"})
      all = all && vm.hasViewport("mount:" + sub(as));
  }
  if (!h.ui->enabled()) {
    std::printf("SKIP: nothing rasterises\n");
    return require ? 1 : 0;
  }
  check(all, "each application's scene is realized, with a viewport of its own");
  if (!all)
    return cvcgl_test::finish("cvcgl_ariadne_mount_scenes");
  for (int frame = 0; frame < 4; ++frame)
    rt.step(kDt);
  for (const std::string &w : rt.take_warnings())
    check(false, "no warnings while they open", w);

  // ── apart, each at its own sub-prefix ───────────────────────────────────────────────────────
  cvc::gl::ariadne::SceneViews &views = rt.views();
  for (const char *as : {"a", "b", "c", "d"}) {
    cvc::gl::ariadne::SceneSlot slot;
    const std::string s = sub(as);
    check(views.scene("mount:" + s, slot) && slot.sg, s + ": its scene can be shown");
    if (!slot.sg)
      continue;
    check(slot.sg->getStatePrefix() == s, s + ": its graph is at its sub-prefix",
          slot.sg->getStatePrefix());
    for (const char *other : {"a", "b", "c", "d"})
      check((slot.sg->getGraphics(std::string("box_") + other) != nullptr) ==
                (std::string(other) == as),
            s + ": its graph holds its own node only (box_" + other + ")");
    check(tree.findDescendant(s + ".graphics.root.children.box_" + as) != nullptr,
          s + ": its node's state is under its sub-prefix");
    check(tree.findDescendant(s + ".viewers.main.camera") != nullptr,
          s + ": its camera is <sub>.viewers.main.camera");
    check(tree(s + ".ariadne.scene").value() == "1", s + ": its scene window is shown");
    cvc::gl::Viewport &own = vm.viewport("mount:" + s);
    int ts[2] = {0, 0};
    own.textureSize(ts);
    check(own.visible() && own.textured() && ts[0] > 100 && ts[1] > 100,
          s + ": its scene window draws its scene's own viewport into the window's texture",
          std::to_string(ts[0]) + " x " + std::to_string(ts[1]));
  }
  check(vm.hasViewport("mount:" + sub("a") + "|thumb") &&
            vm.viewport("mount:" + sub("a") + "|thumb").visible(),
        "the shell's viewport with scene: { mount: a } is a further view of a's scene");

  // ── binds resolve under each sub-prefix ──────────────────────────────────────────────────────
  {
    cvc::gl::ariadne::SceneSlot sa, sb;
    views.scene("mount:" + sub("a"), sa);
    views.scene("mount:" + sub("b"), sb);
    const auto shown = [](cvc::gl::ariadne::SceneSlot &s, const char *id) {
      const auto n = s.sg ? s.sg->getGraphics(id) : nullptr;
      return n && n->isVisible();
    };
    check(shown(sa, "box_a") && shown(sb, "box_b"), "both nodes start shown");
    tree("mounts.show").value("0"); // the shell's own key of that name
    rt.step(kDt);
    rt.step(kDt);
    check(shown(sa, "box_a") && shown(sb, "box_b"), "the shell's `show` moves neither");
    tree(sub("a") + ".show").value("0");
    rt.step(kDt);
    rt.step(kDt);
    check(!shown(sa, "box_a") && shown(sb, "box_b"),
          "a's visible: show is a's own key: a's node hides, b's stays");
    tree(sub("a") + ".show").value("1");
    rt.step(kDt);
    check(shown(sa, "box_a"), "and shows again");
  }

  // ── clocks: each world at its own rate, or the shell's ───────────────────────────────────────
  {
    check(std::fabs(cvc::scene_clock(h.app, sub("a")).fixed_dt() - 1.0 / 30.0) < 1e-12,
          "a's clock ticks at its clock: hz (30)");
    check(std::fabs(cvc::scene_clock(h.app, sub("b")).fixed_dt() - 1.0 / 60.0) < 1e-12,
          "b's at its own (60)");
    check(std::fabs(cvc::scene_clock(h.app, sub("c")).fixed_dt() - 1.0 / 50.0) < 1e-12,
          "c's, with no rate of its own, at the shell's (50)",
          std::to_string(cvc::scene_clock(h.app, sub("c")).fixed_dt()));
    check(&cvc::scene_clock(h.app, sub("a")) != &h.app.world_clock() &&
              &cvc::scene_clock(h.app, sub("c")) != &h.app.world_clock(),
          "every mounted world has a clock of its own");
    rt.set_fixed_clock(true);
    constexpr int kFrames = 10;
    constexpr double kStep = 0.1; // one second over the ten frames
    rt.step(kStep);
    const std::uint64_t a0 = ticks(h.app, sub("a")), b0 = ticks(h.app, sub("b")),
                        c0 = ticks(h.app, sub("c")), s0 = h.app.world_clock().tick();
    for (int i = 0; i < kFrames; ++i)
      rt.step(kStep);
    const long long da = static_cast<long long>(ticks(h.app, sub("a")) - a0);
    const long long db = static_cast<long long>(ticks(h.app, sub("b")) - b0);
    const long long dc = static_cast<long long>(ticks(h.app, sub("c")) - c0);
    const long long ds = static_cast<long long>(h.app.world_clock().tick() - s0);
    std::printf("  one second: a %lld, b %lld, c %lld, the shell %lld ticks\n", da, db, dc, ds);
    check(std::llabs(da - 30) <= 1, "a steps 30 times a second", std::to_string(da));
    check(std::llabs(db - 60) <= 1, "b steps 60 times a second, side by side", std::to_string(db));
    check(std::llabs(dc - 50) <= 1, "c steps at the shell's rate", std::to_string(dc));
    check(std::llabs(ds - 50) <= 1, "the shell's own world at its own", std::to_string(ds));
  }

  // ── background: running, and held ───────────────────────────────────────────────────────────
  {
    constexpr double kStep = 0.1;
    tree(sub("a") + ".tree.ariadne_scene.open").value("0"); // runs in the background
    tree(sub("d") + ".tree.ariadne_scene.open").value("0"); // holds in the background
    rt.step(kStep);
    rt.step(kStep);
    check(!vm.viewport("mount:" + sub("d")).visible(), "d's window closed: its view is hidden");
    const std::uint64_t a0 = ticks(h.app, sub("a")), d0 = ticks(h.app, sub("d"));
    for (int i = 0; i < 10; ++i)
      rt.step(kStep);
    const std::uint64_t a1 = ticks(h.app, sub("a")), d1 = ticks(h.app, sub("d"));
    check(a1 > a0 + 20, "a world that runs in the background steps with its window closed",
          std::to_string(a1 - a0));
    check(d1 == d0, "a held world is not stepped while its window is closed",
          std::to_string(d1 - d0));
    tree(sub("d") + ".tree.ariadne_scene.open").value("1");
    for (int i = 0; i < 10; ++i)
      rt.step(kStep);
    check(ticks(h.app, sub("d")) > d1 + 20, "and goes on when its window opens again",
          std::to_string(ticks(h.app, sub("d")) - d1));
  }

  // ── closing takes its scene down ────────────────────────────────────────────────────────────
  {
    tree("mounts.apps").removeChild("b");
    for (int i = 0; i < 6; ++i)
      rt.step(kDt);
    cvc::gl::ariadne::SceneSlot sb;
    check(!vm.hasViewport("mount:" + sub("b")), "closed: its viewport is gone");
    check(!views.scene("mount:" + sub("b"), sb), "and its scene");
    check(!cvc::has_scene_clock(h.app, sub("b")), "and its clock");
    check(tree(sub("b") + ".ariadne.scene").value() == "0", "its scene window is not shown");
    check(vm.hasViewport("mount:" + sub("a")), "the others stay");
  }
  return cvcgl_test::finish("cvcgl_ariadne_mount_scenes");
#endif
}
