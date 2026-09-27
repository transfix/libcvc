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
#include <cstdlib>
#include <cstring>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/loader.h>
#include <cvc/core/app.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/StageLighting.h>
#include <cvc/gl/TouchGestures.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/gl/ariadne/scene_panels.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/volume/bounding_box.h>
#include <string>
#include <thread>

using cvc::gl::CameraController;
using cvc::gl::ImGuiBackend;
using cvc::gl::ImGuiOverlay;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
using cvc::gl::TouchGestures;
namespace ari = cvc::ariadne;

int main(int argc, char **argv) {
  // Generic Ariadne runner: load a .ari (widgets + scene), realize it, frame the camera to the
  // scene, and render — a window by default, or a headless capture for verification.
  //   ariadne_hello doc.ari [--offscreen] [--frames N] [--png PATH] [--width W] [--height H]
  std::string docPath, png;
  bool offscreen = false;
  long frames = 0;
  int width = 1024, height = 768;
  for (int i = 1; i < argc; ++i) {
    const char *a = argv[i];
    auto next = [&](const char *dflt) { return (i + 1 < argc) ? argv[++i] : dflt; };
    if (!std::strcmp(a, "--offscreen"))
      offscreen = true;
    else if (!std::strcmp(a, "--png")) {
      png = next("");
      offscreen = true;
    } else if (!std::strcmp(a, "--frames"))
      frames = std::atol(next("0"));
    else if (!std::strcmp(a, "--width"))
      width = std::atoi(next("1024"));
    else if (!std::strcmp(a, "--height"))
      height = std::atoi(next("768"));
    else if (a[0] != '-' && docPath.empty())
      docPath = a;
  }
  const bool capturing = offscreen || !png.empty();
  if (capturing && frames <= 0)
    frames = 1; // a capture defaults to one frame

  cvc::app app;
  SceneGraph sg(app, "hello");
  SceneRenderer view(sg, width, height, offscreen, "main");
  view.setBackground(0.09, 0.10, 0.12);

  CameraController cam(view);
  cam.setMode(CameraController::Mode::Orbit);
  TouchGestures touch(view, cam); // navigation (Tab orbit/fly, drag to look); updated each frame

  ImGuiOverlay ui(view);
  ui.attachCamera(cam);
  ui.setVisible(!capturing); // a captured still has no overlay chrome

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

  // A custom widget type (§ extensibility), registered BEFORE the load so the document
  // finds a handler. It composes built-in widgets — a caption + a read-only value view
  // — so it works on any backend with no Backend change. A .ari uses it as
  //   - type: labeled
  //     title: Belief
  //     bind: demo.belief
  register_widget_type("labeled", [](const Widget &w, const WidgetEmitContext &ctx) {
    ctx.emit(text(w.props.str("title", w.label)));
    ctx.emit(text_bound("  =", w.bind));
  });

  // Build the widget tree: from a .ari file if one is given on the command line
  // (`ariadne_hello hello.ari`), else the built-in programmatic tree below —
  // which the shipped hello.ari mirrors, so the two render identically.
  // The realized scene (§9): geometry/lights built from the document's `scene:`
  // block, plus the visibility bindings we poll each frame (empty for the built-in
  // tree, which declares no scene). Must outlive the render loop.
  cvc::gl::ariadne::RealizedScene realized;

  Widget tree;
  bool loaded = false;
  if (!docPath.empty()) {
    LoadResult lr = load_file(docPath.c_str());
    if (lr.ok) {
      tree = std::move(lr.root);
      loaded = true;
      std::printf("[ariadne_hello] loaded %s", docPath.c_str());
      if (!lr.meta.name.empty())
        std::printf(" — \"%s\"", lr.meta.name.c_str());
      std::printf("\n");
      for (const std::string &w : lr.warnings) // §15 semantic validation + customs warnings
        std::printf("[ariadne_hello]   %s\n", w.c_str());
      // §extensibility: run the init: state_exec script ONCE, scoped to the same prefix
      // the Runtime binds against, BEFORE realize/render — so its seeds win over widget
      // defaults. Errors are logged; init is optional dynamic init, not a hard gate.
      std::vector<std::string> init_errs;
      if (!run_init(app, sg.getStatePrefix(), lr.init_script, &init_errs))
        for (const std::string &e : init_errs)
          std::printf("[ariadne_hello]   %s\n", e.c_str());
      // §extensibility: the loader already fail-fast-checked widget/block customs; now
      // check declared NODE customs (cvcGL registry) before realizing. A missing
      // REQUIRED node custom fails fast (skip the scene); a non-required one just logs.
      std::vector<std::string> custom_errs;
      const bool customs_ok = cvc::gl::ariadne::verify_scene_customs(lr, &custom_errs);
      for (const std::string &e : custom_errs)
        std::printf("[ariadne_hello]   %s\n", e.c_str());
      // Realize the scene under the SAME state prefix the Runtime binds against, so
      // a widget `bind:` and a scene `visible:` on one path share one key.
      if (customs_ok && lr.scene.any()) {
        std::vector<std::string> scene_warnings;
        realized =
            cvc::gl::ariadne::realize_scene(sg, lr.scene, sg.getStatePrefix(), &scene_warnings);
        std::printf("[ariadne_hello] scene: %zu node(s), %zu visibility bind(s)\n",
                    realized.created.size(), realized.visibility.size());
        for (const std::string &w : scene_warnings)
          std::printf("[ariadne_hello]   %s\n", w.c_str());
        // Frame the camera to the realized scene so it fills the view (a scene demo wants this;
        // the built-in tree, with no scene, keeps the default camera).
        const cvc::bounding_box bb = sg.computeGraphicsBounds();
        if (!bb.isNull())
          cam.frameBounds(bb.minx, bb.miny, bb.minz, bb.maxx, bb.maxy, bb.maxz);
        // Escape-hatch fallback: also expose the rich C++ scene composites as custom widgets
        // (scene_panel / stage_lighting_panel / scene_menu), so a document may use EITHER the
        // declarative .ari control components (preferred) OR these. Harmless if unused.
        cvc::gl::StageLighting *rig = realized.rigs.empty() ? nullptr : realized.rigs.front().get();
        cvc::gl::ariadne::register_scene_panels(backend, sg, rig);
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
            menu("Sim",
                 {
                     menu_toggle("Paused", "demo.paused", false),
                     menu_action("Reset", "reset"),
                     menu_action("Quit", "quit"),
                 }),
        }),
        window("Controls",
               {
                   text("Ariadne P0 — a programmatic widget tree."),
                   separator(),
                   slider_int("Agents", "demo.agents", 1, 512, 64),
                   slider_float("Speed", "demo.speed", 0.1, 4.0, 1.0, "%.2f"),
                   checkbox("Wireframe", "demo.wire", false),
                   combo("Belief", "demo.belief", {"shared", "grouped", "private"}, "shared"),
                   separator(),
                   custom("labeled", "Belief", "demo.belief"), // a custom widget type
                   checkbox("Show mesh", "demo.show_mesh", true),
                   button("Reset", "reset"),
               }),
    });
  rt.set_root(std::move(tree));

  // One frame of host work BEFORE the draw (the deferred-intent discipline, §7.2): pump input,
  // advance the camera, drain queued actions, mirror scene visibility, tick volume nodes, surface
  // reactive diagnostics. The actual draw is the caller's (view.render() or view.writePNG()).
  auto frame_body = [&](double dt) {
    view.processUIEvents();
    touch.update();
    cam.update(dt);
    rt.drain(); // run queued Ariadne action events (incl. program on:) on the host thread
    ari::sync_scene_visibility(app, realized.visibility); // §9 bound `visible:` -> node .visible
    cvc::gl::ariadne::tick_scene(realized, view.renderer()); // §9 volren/volslice per-frame service
    for (const std::string &w : rt.take_reactive_warnings())
      std::fprintf(stderr, "%s\n", w.c_str());
  };

  if (capturing) {
    // Headless capture: advance a few frames (so shadows bake and the camera settles), then write
    // the final one — the verification path (no window, no overlay chrome).
    for (long f = 0; f < frames; ++f) {
      frame_body(1.0 / 30.0);
      if (f + 1 == frames && !png.empty())
        view.writePNG(png.c_str()); // renders + writes the final frame
      else
        view.render();
    }
    std::printf("[ariadne_hello] captured %ld frame(s)%s%s\n", frames, png.empty() ? "" : " -> ",
                png.c_str());
  } else {
    std::puts("[ariadne_hello] running — close the window or use Sim > Quit to exit.");
    while (!view.windowClosed() && !quit) {
      frame_body(1.0 / 120.0);
      view.render(); // draws the scene + the Ariadne overlay (runs rt.render())
      std::this_thread::sleep_for(std::chrono::milliseconds(8)); // ~120 Hz cap
    }
  }
  cam.detach();
  return 0;
}
