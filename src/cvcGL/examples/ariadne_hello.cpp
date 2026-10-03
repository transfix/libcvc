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
#ifdef __EMSCRIPTEN__
#include <emscripten.h> // emscripten_sleep — yield the asyncify main loop to the browser
#endif
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/uri.h> // register_cvc_uri_handler — the cvc:// component-library import path
#include <cvc/core/app.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/SdlInput.h> // §4.6 SDL input source -> Runtime::post_input (on_key/on_pointer)
#include <cvc/gl/StageLighting.h>
#include <cvc/gl/TouchGestures.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/gl/ariadne/scene_panels.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/state_publisher.h> // sg.publisher().flush() on the single-thread wasm path
#include <cvc/volume/bounding_box.h>
#include <string>
#include <thread>
#include <vector>
#include <vtkRenderer.h> // a scene background: gradient reaches the renderer directly (§16.1)
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

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
  std::vector<std::string> componentPaths; // extra cvc:// search dirs (dev / bundled components)
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
    else if (!std::strcmp(a, "--component-path")) // add a dir to the cvc:// component search path
      componentPaths.push_back(next(""));
    else if (a[0] != '-' && docPath.empty())
      docPath = a;
  }
  // A wasm demo build bakes in its own .ari (CVC_ARIADNE_WASM_DOC) and embeds the component library
  // in MEMFS (see the examples CMake --embed-file). With no argv in the browser, default to that
  // doc and add the MEMFS root to the search path so `cvc://components/*.ari` resolves at
  // /components.
#ifdef CVC_ARIADNE_WASM_DOC
  if (docPath.empty()) {
    docPath = CVC_ARIADNE_WASM_DOC;
    componentPaths.push_back("/");
  }
#endif
  // The cvc:// component-library import path (import: cvc://components/foo.ari), resolved against
  // CVC_ARIADNE_PATH + any --component-path dirs + the install datadir + cwd. Register before load.
  cvc::ariadne::register_cvc_uri_handler(componentPaths);
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

  // §4.6 input source: SDL feeds keyboard/mouse events to the on_key/on_pointer residents each
  // frame. NB SDL delivers OS input only through an SDL window, so this yields events on the
  // wasm/wasm-mt build (VTK's WebAssembly interactor shares the SDL canvas) — on a native VTK
  // window the SDL queue is empty and input arrives via VTK's interactor -> ImGui (which also
  // drives widget-level on_click/on_hover). Harmless either way; a translation copies the
  // SDL-free cvc::gl::InputEvent to the runtime's cvc::ariadne::InputEvent.
  cvc::gl::SdlInput sdl;
  sdl.init();
  auto to_ari = [](const cvc::gl::InputEvent &e) {
    ari::InputEvent a;
    using GK = cvc::gl::InputEvent::Kind;
    using AK = ari::InputEvent::Kind;
    switch (e.kind) {
    case GK::KeyDown:
      a.kind = AK::KeyDown;
      break;
    case GK::KeyUp:
      a.kind = AK::KeyUp;
      break;
    case GK::MouseMove:
      a.kind = AK::MouseMove;
      break;
    case GK::MouseButtonDown:
      a.kind = AK::MouseButtonDown;
      break;
    case GK::MouseButtonUp:
      a.kind = AK::MouseButtonUp;
      break;
    case GK::MouseWheel:
      a.kind = AK::MouseWheel;
      break;
    }
    a.key = e.key;
    a.mods = e.mods;
    a.repeat = e.repeat;
    a.x = e.x;
    a.y = e.y;
    a.dx = e.dx;
    a.dy = e.dy;
    a.button = e.button;
    a.clicks = e.clicks;
    return a;
  };

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
      // §7.1/§4.6 document-level resident handlers: hand each script to its Runtime setter. Each
      // becomes ONE long-lived resident (parking between activations). on_tick runs every frame;
      // on_key/on_pointer run per input event, fed below via SdlInput -> rt.post_input.
      rt.set_tick_program(lr.on_tick_script);
      rt.set_key_program(lr.on_key_script);
      rt.set_pointer_program(lr.on_pointer_script);
      // §12 channel enforcement: install the document's channel policy so a program that sends/
      // receives on an undeclared channel is refused at run time (the backstop for dynamic channel
      // names; the load-time lint already checked static refs). Only when the doc declared
      // channels: (else fully permissive); strict unless lint.channels relaxed it.
      if (lr.has_channels_block) {
        std::vector<std::string> declared, global;
        for (const ari::ChannelDecl &c : lr.channels) {
          declared.push_back(c.name);
          if (c.global)
            global.push_back(c.name);
        }
        rt.set_channel_policy(std::move(declared), std::move(global),
                              lr.lint.channels == ari::LintConfig::Mode::Strict, lr.lint.quiet);
      }
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
        // Apply the scene's background (a VIEW property): a solid colour, or a top→bottom gradient
        // reached through the renderer (SceneRenderer exposes only a flat setBackground).
        if (lr.scene.has_background) {
          const auto &t = lr.scene.background_top;
          const auto &b = lr.scene.background_bottom;
          if (lr.scene.background_gradient) {
            if (vtkRenderer *r = view.renderer()) {
              r->SetGradientBackground(true);
              r->SetBackground(b[0], b[1], b[2]);  // bottom
              r->SetBackground2(t[0], t[1], t[2]); // top
            }
          } else {
            view.setBackground(t[0], t[1], t[2]);
          }
        }
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
    // Feed this frame's input to the on_key/on_pointer residents BEFORE drain(), so it is delivered
    // this same frame (post_input queues onto the scheduler ingress; drain() drains + pumps).
    for (const cvc::gl::InputEvent &e : sdl.poll())
      rt.post_input(to_ari(e));
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
    // This loop yields to the browser once per frame in the WebAssembly build (emscripten_sleep(0)
    // at its end), so VTK need not yield again inside every render (FrameYield::App; a no-op
    // natively). Only here: the capture branch above renders back to back without yielding, so it
    // keeps VTK's in-render yield.
    view.setFrameYield(SceneRenderer::FrameYield::App);
    while (!view.windowClosed() && !quit) {
      frame_body(1.0 / 120.0);
      view.render(); // draws the scene + the Ariadne overlay (runs rt.render())
#ifdef __EMSCRIPTEN__
#ifndef __EMSCRIPTEN_PTHREADS__
      sg.publisher().flush(); // no worker thread — drain publishes at frame cadence
#endif
      emscripten_sleep(0); // yield to the browser event loop once per frame (Asyncify)
#else
      std::this_thread::sleep_for(std::chrono::milliseconds(8)); // ~120 Hz cap
#endif
    }
  }
  cam.detach();
  return 0;
}
