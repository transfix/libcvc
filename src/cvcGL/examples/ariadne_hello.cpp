// ariadne_hello — the first Ariadne (cvc::gl::ariadne) demo.
//
// Boots the standard cvcGL viewer (SceneGraph + SceneRenderer + ImGuiOverlay) and
// mounts a small Ariadne widget tree built PROGRAMMATICALLY (the P0 slice; the
// YAML .ari loader lands next). It shows the whole P0 loop working end to end:
//   - a main menu bar with a state-bound toggle and a fire-once action,
//   - a floating window of state-bound widgets (slider / checkbox / combo / text),
//   - a Button that raises a named event, drained on the host thread.
// Every bound widget reads/writes a cvc::state path under the scene prefix, so a
// state observer / script / replicated peer sees the same edits.

#include <chrono>
#include <cstdio>
#include <thread>

#include <cvc/core/app.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ariadne/ariadne.h>

using cvc::gl::CameraController;
using cvc::gl::ImGuiOverlay;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
namespace ari = cvc::gl::ariadne;

int main() {
  cvc::app app;
  SceneGraph sg(app, "hello");
  SceneRenderer view(sg, 1024, 768, /*offscreen=*/false, "main");
  view.setBackground(0.09, 0.10, 0.12);

  CameraController cam(view);
  cam.setMode(CameraController::Mode::Orbit);

  ImGuiOverlay ui(view);
  ui.attachCamera(cam);

  // The Ariadne runtime binds relative widget paths under the scene prefix.
  ari::Runtime rt(app, ui, sg.getStatePrefix());

  bool quit = false;
  rt.on("quit", [&] { quit = true; });
  rt.on("reset", [&] { std::printf("[ariadne_hello] reset pressed\n"); });

  using namespace cvc::gl::ariadne; // the builder helpers
  rt.set_root(group({
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
                             button("Reset", "reset"),
                         }),
  }));
  rt.install();

  std::puts("[ariadne_hello] running — close the window or use Sim > Quit to exit.");
  while (!view.windowClosed() && !quit) {
    view.processUIEvents(); // pump input into the overlay + camera
    rt.drain();             // run queued Ariadne action events on the host thread
    view.render();          // draws the scene + the Ariadne overlay
    std::this_thread::sleep_for(std::chrono::milliseconds(8)); // ~120 Hz cap
  }
  return 0;
}
