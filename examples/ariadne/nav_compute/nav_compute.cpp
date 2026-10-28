// nav_compute — the Ariadne cross-thread async demo (roadmap item 2).
//
// Boots the standard cvcGL viewer + an ImGuiBackend and loads nav_compute.ari, whose "Run nav step"
// button is a state_exec PROGRAM on the async lane. It binds ONE host verb, (nav-step), an async
// function on the app's async host (docs/STATE_EXEC_EXCEPTIONS.md 3).
// - The calling action parks at the call while the step's kernel runs on the app's background
//   task pool and fans out over app.computePool().
// - The call returns the arrived count, so the UI never blocks while the step runs.
// - The step's run time and CPU time, the fan-out's included, are charged to the action, and a
//   failed step raises at the call.
//
//   nav_compute nav_compute.ari [--component-path DIR] [--offscreen] [--frames N]
//
// The core mechanism is covered headlessly (no ImGui/GL) by AriadneAction.
// NavStepAsAnAsyncFunctionReturnsItsResult. This binary is the interactive proof: click the button
// and watch status flip running -> done across a real cross-thread gap.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/uri.h> // register_cvc_uri_handler — the cvc:// component-library import path
#include <cvc/core/app.h>
#include <cvc/core/thread_pool.h> // the step fans out with computePool().parallel_for
#include <cvc/gl/CameraController.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/state/state_exec/async_host.h> // register_async_fn — the async host verb
#include <cvc/state/state_exec/types.h>      // value_t
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace ari = cvc::ariadne;
namespace se = cvc::state_exec;

int main(int argc, char **argv) {
  std::string docPath;
  bool offscreen = false;
  long frames = 0;
  int width = 1024, height = 768;
  std::vector<std::string> componentPaths;
  for (int i = 1; i < argc; ++i) {
    const char *a = argv[i];
    auto next = [&](const char *dflt) { return (i + 1 < argc) ? argv[++i] : dflt; };
    if (!std::strcmp(a, "--offscreen"))
      offscreen = true;
    else if (!std::strcmp(a, "--frames"))
      frames = std::atol(next("0"));
    else if (!std::strcmp(a, "--width"))
      width = std::atoi(next("1024"));
    else if (!std::strcmp(a, "--height"))
      height = std::atoi(next("768"));
    else if (!std::strcmp(a, "--component-path"))
      componentPaths.push_back(next(""));
    else if (a[0] != '-' && docPath.empty())
      docPath = a;
  }
  if (docPath.empty()) {
    std::fprintf(stderr, "usage: nav_compute nav_compute.ari [--offscreen] [--frames N]\n");
    return 2;
  }
  ari::register_cvc_uri_handler(componentPaths);
  const bool capturing = offscreen;
  if (capturing && frames <= 0)
    frames = 3;

  cvc::app app;
  cvc::gl::SceneGraph sg(app, "navcompute");
  cvc::gl::SceneRenderer view(sg, width, height, offscreen, "main");
  view.setBackground(0.09, 0.10, 0.12);
  cvc::gl::CameraController cam(view);
  cvc::gl::ImGuiOverlay ui(view);
  ui.attachCamera(cam);
  ui.setVisible(!capturing);

  ari::Runtime rt(app, sg.getStatePrefix());
  cvc::gl::ImGuiBackend backend;
  rt.set_backend(&backend);
  backend.install(rt, ui);

  // Warm the lazily-built per-app singletons ON THE HOST THREAD before any worker touches them, so
  // a background compute-pool job never races their first construction.
  app.computePool();
  app.exec_scheduler();
  app.exec_async_host();

  // The host verb (nav-step): an async function on the app's async host. Its marshal step runs on
  // the scheduler thread in the calling step; its kernel runs on the app's background task pool,
  // where the parallel_for fans out over the compute pool, charged to the calling action. The
  // kernel touches no DSL value or state: its result is a fresh string.
  ari::register_action_intrinsics(
      [&app](std::shared_ptr<se::environment> env, se::intrinsics_context &) {
        se::register_async_fn(env, app.exec_async_host(), "nav-step",
                              [&app](std::span<const se::value_t>) -> se::async_kernel {
                                return [&app](se::async_call &) -> se::value_t {
                                  std::atomic<int> arrived{0};
                                  // stand-in nav kernel: count the "arrived" agents
                                  app.computePool().parallel_for(4096, [&arrived](int i) {
                                    if ((i % 3) == 0)
                                      arrived.fetch_add(1, std::memory_order_relaxed);
                                  });
                                  return se::value_t(std::to_string(arrived.load()));
                                };
                              });
      });

  ari::LoadResult lr = ari::load_file(docPath.c_str());
  if (!lr.ok) {
    std::fprintf(stderr, "[nav_compute] %s\n", lr.error.c_str());
    ari::clear_action_intrinsics();
    return 1;
  }
  std::printf("[nav_compute] loaded %s%s%s\n", docPath.c_str(), lr.meta.name.empty() ? "" : " — ",
              lr.meta.name.c_str());
  for (const std::string &w : lr.warnings)
    std::printf("[nav_compute]   %s\n", w.c_str());
  std::vector<std::string> init_errs;
  if (!ari::run_init(app, sg.getStatePrefix(), lr.init_script, &init_errs))
    for (const std::string &e : init_errs)
      std::printf("[nav_compute]   %s\n", e.c_str());
  rt.set_root(std::move(lr.root));

  // One frame of host work BEFORE the draw (the §7.2 deferred-intent discipline): drain queued
  // Ariadne actions (incl. the async program on:) on the host thread, then surface diagnostics. The
  // draw (view.render()) runs rt.render() via the overlay callback.
  auto frame_body = [&] {
    view.processUIEvents();
    rt.drain();
    for (const std::string &w : rt.take_reactive_warnings())
      std::fprintf(stderr, "%s\n", w.c_str());
  };

  if (capturing) {
    for (long f = 0; f < frames; ++f) {
      frame_body();
      view.render();
    }
    std::printf("[nav_compute] captured %ld frame(s)\n", frames);
  } else {
    std::puts("[nav_compute] running — click \"Run nav step\"; close the window to exit.");
    while (!view.windowClosed()) {
      frame_body();
      view.render();
      std::this_thread::sleep_for(std::chrono::milliseconds(8)); // ~120 Hz cap
    }
  }
  ari::clear_action_intrinsics(); // the provider captured &app; drop it before app dies
  cam.detach();
  return 0;
}
