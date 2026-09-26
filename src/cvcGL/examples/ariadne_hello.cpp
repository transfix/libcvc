// ariadne_hello — the first Ariadne demo (context-agnostic core, one backend).
//
// Boots the standard cvcGL viewer (SceneGraph + SceneRenderer + ImGuiOverlay) and
// mounts a small Ariadne widget tree built PROGRAMMATICALLY (the P0 slice; the
// YAML .ari loader lands next). It shows the whole P0 loop working end to end:
//   - a main menu bar with a state-bound toggle and a fire-once action,
//   - a floating window of state-bound widgets (slider / checkbox / combo / text),
//   - a Button that raises a named event, drained on the host thread.
// Every bound widget reads/writes a cvc::state path under the scene prefix, so a
// state observer / script / replicated peer sees the same edits.
//
// The tree + Runtime are pure libcvc (cvc::ariadne); the only cvcGL piece is the
// backend — cvc::gl::ImGuiBackend renders the SAME tree as ImGui over VTK
// (roadmap §16.1). Swap the backend and this tree would render on a terminal.

#include <chrono>
#include <cstdio>
#include <thread>

#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/loader.h>
#include <cvc/core/app.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/gl/ariadne/scene_realize.h>

using cvc::gl::CameraController;
using cvc::gl::ImGuiBackend;
using cvc::gl::ImGuiOverlay;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
namespace ari = cvc::ariadne;

int main(int argc, char **argv) {
  cvc::app app;
  SceneGraph sg(app, "hello");
  SceneRenderer view(sg, 1024, 768, /*offscreen=*/false, "main");
  view.setBackground(0.09, 0.10, 0.12);

  CameraController cam(view);
  cam.setMode(CameraController::Mode::Orbit);

  ImGuiOverlay ui(view);
  ui.attachCamera(cam);

  // The Ariadne runtime binds relative widget paths under the scene prefix; the
  // ImGui backend renders it and drives the walk from VTK's render pass.
  ari::Runtime rt(app, sg.getStatePrefix());
  ImGuiBackend backend;
  rt.set_backend(&backend);
  backend.install(rt, ui);

  bool quit = false;
  rt.on("quit", [&] { quit = true; });
  rt.on("reset", [&] { std::printf("[ariadne_hello] reset pressed\n"); });

  using namespace cvc::ariadne; // the builder helpers

  // Build the widget tree: from a .ari file if one is given on the command line
  // (`ariadne_hello hello.ari`), else the built-in programmatic tree below —
  // which the shipped hello.ari mirrors, so the two render identically.
  // The realized scene (§9): geometry/lights built from the document's `scene:`
  // block, plus the visibility bindings we poll each frame (empty for the built-in
  // tree, which declares no scene). Must outlive the render loop.
  cvc::gl::ariadne::RealizedScene realized;

  Widget tree;
  bool loaded = false;
  if (argc > 1) {
    LoadResult lr = load_file(argv[1]);
    if (lr.ok) {
      tree = std::move(lr.root);
      loaded = true;
      std::printf("[ariadne_hello] loaded %s", argv[1]);
      if (!lr.meta.name.empty())
        std::printf(" — \"%s\"", lr.meta.name.c_str());
      std::printf("\n");
      for (const std::string &w : lr.warnings) // §15 semantic validation
        std::printf("[ariadne_hello]   %s\n", w.c_str());
      // Realize the scene under the SAME state prefix the Runtime binds against, so
      // a widget `bind:` and a scene `visible:` on one path share one key.
      if (lr.scene.any()) {
        std::vector<std::string> scene_warnings;
        realized = cvc::gl::ariadne::realize_scene(sg, lr.scene, sg.getStatePrefix(),
                                                   &scene_warnings);
        std::printf("[ariadne_hello] scene: %zu node(s), %zu visibility bind(s)\n",
                    realized.created.size(), realized.visibility.size());
        for (const std::string &w : scene_warnings)
          std::printf("[ariadne_hello]   %s\n", w.c_str());
      }
    } else {
      // A failed load (e.g. the min_libcvc gate) never half-renders.
      std::printf("[ariadne_hello] %s\n[ariadne_hello] falling back to the built-in tree.\n",
                  lr.error.c_str());
    }
  }
  if (!loaded)
    tree = group({
      menubar({
          menu("Sim", {
                          menu_toggle("Paused", "demo.paused", false),
                          menu_action("Reset", "reset"),
                          menu_action("Quit", "quit"),
                      }),
      }),
      window("Controls", {
                             text("Ariadne P0 — a programmatic widget tree."),
                             separator(),
                             slider_int("Agents", "demo.agents", 1, 512, 64),
                             slider_float("Speed", "demo.speed", 0.1, 4.0, 1.0, "%.2f"),
                             checkbox("Wireframe", "demo.wire", false),
                             combo("Belief", "demo.belief", {"shared", "grouped", "private"}, "shared"),
                             separator(),
                             text_bound("belief =", "demo.belief"),
                             checkbox("Show mesh", "demo.show_mesh", true),
                             button("Reset", "reset"),
                         }),
    });
  rt.set_root(std::move(tree));

  std::puts("[ariadne_hello] running — close the window or use Sim > Quit to exit.");
  while (!view.windowClosed() && !quit) {
    view.processUIEvents(); // pump input into the overlay + camera
    rt.drain();             // run queued Ariadne action events on the host thread
    // §9: mirror each bound `visible:` path into its node's `.visible` key (a no-op
    // unless the value changed). On this owner thread the node flips inline.
    ari::sync_scene_visibility(app, realized.visibility);
    // §9: service any volren/volslice nodes (they render nothing without a per-frame
    // tick + a multi-slice depth sort). A no-op for a scene without volume renderers.
    cvc::gl::ariadne::tick_scene(realized, view.renderer());
    view.render();          // draws the scene + the Ariadne overlay
    std::this_thread::sleep_for(std::chrono::milliseconds(8)); // ~120 Hz cap
  }
  return 0;
}
