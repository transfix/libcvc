// nav_compute — the Ariadne cross-thread async demo (roadmap item 2).
//
// Boots the standard cvcGL viewer + an ImGuiBackend and loads nav_compute.ari, whose "Run nav step"
// button is a state_exec PROGRAM on the async lane. It binds ONE host verb, (nav-step-async), that
// runs a nav step on app.computePool() OFF the render thread via app.compute_async and posts the
// arrived count to the "nav.done" channel when the fan-out joins. The .ari action parks on
// (msg-recv "nav.done") and resumes on a later frame — so the UI never blocks while the step runs.
// This is the whole async story end to end: host intrinsic -> compute pool -> post_message ->
// msg-recv wake -> state write (a bound Text shows the result). Message channels are global (not
// chrooted to the document prefix), so the literal "nav.done" matches on both sides.
//
//   nav_compute nav_compute.ari [--component-path DIR] [--offscreen] [--frames N]
//
// The core mechanism is covered headlessly by two gtests (no ImGui/GL): AppTest.ComputeAsync* and
// AriadneAction.IntrinsicRunsNavStepOnComputePoolAndWakesMsgRecv. This binary is the interactive
// proof — click the button and watch status flip running -> done across a real cross-thread gap.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/uri.h> // register_cvc_uri_handler — the cvc:// component-library import path
#include <cvc/core/app.h>
#include <cvc/core/state_exec/async_scheduler.h> // exec_scheduler().post_message
#include <cvc/core/state_exec/builtins.h>   // register_fn — bind the host verb into the lanes
#include <cvc/core/state_exec/intrinsics.h> // resolve_channel_key — scope the completion channel
#include <cvc/core/state_exec/types.h>      // value_t payload
#include <cvc/core/thread_pool.h>           // compute_async drives computePool().parallel_for
#include <cvc/gl/CameraController.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
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

  // The host verb (nav-step-async): kick off a nav step on the compute pool OFF the render thread
  // and post the result when it joins. Runs INLINE on the host thread inside Runtime::drain(), so
  // it MUST submit-and-return, never block — app.compute_async does exactly that (a background pool
  // worker owns the blocking parallel_for; the caller returns immediately). on_done runs on the
  // worker and touches only the one thread-safe seam, exec_scheduler().post_message. Guarded so a
  // CVC_STATE_EXEC=OFF build still links (the program lanes just don't run there).
  if (ari::have_state_exec()) {
    ari::register_action_intrinsics([&app](std::shared_ptr<se::environment> env,
                                           se::intrinsics_context &ictx) {
      // §12 channel scoping: the .ari's (msg-recv "nav.done") resolves against this document's
      // chroot, so the host must post to the SAME scoped key. Resolve it on THIS (scheduler)
      // thread and capture the string — the compute worker must never walk the state tree.
      const std::string done = se::resolve_channel_key(ictx.root_path, "nav.done");
      se::builtins::register_fn(env, "nav-step-async", [&app, done](std::span<const se::value_t>) {
        auto arrived = std::make_shared<std::atomic<int>>(0);
        const int n = 4096;
        app.compute_async(
            n,
            [arrived](int i) { // stand-in nav kernel: count the "arrived" agents
              if ((i % 3) == 0)
                arrived->fetch_add(1, std::memory_order_relaxed);
            },
            [&app, arrived, done] {
              app.exec_scheduler().post_message(done, se::value_t(std::to_string(arrived->load())));
            });
        return se::value_t{}; // nil; the .ari action parks on (msg-recv "nav.done")
      });
    });
  }

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
