// ariadne_conference — the conference host bundle (roadmap §8): the ONLY C++ a conference .ari
// needs. It extends the stock cvcGL Ariadne runner (cf. ariadne_hello / ariadne_stream) with the
// five stream/media registrations, then runs a document that drives the whole media pipeline from
// the DSL program lane — no C++ opens a stream here.
//
//   register_stream_intrinsics   (core)  (stream-open/info/close)
//   register_gl_stream_intrinsics (cvcGL) (gl-bind-stream)
//   register_media_intrinsics    (cvcGL) (stream-capture-camera / -audio / stream-play-audio)
//   register_stream_node_type    (cvcGL) `type: stream`      — video-on-quad scene node
//   register_stream_view_widget  (cvcGL) `type: stream_view` — video-in-an-ImGui-window widget
//
// With these, a pure-.ari document (conference_media.ari) can, from an on_tick/action lane,
// (stream-open) a camera and a mic, attach producers with (stream-capture-camera)/(-audio), render
// the video in a reactive `stream_view` widget, and (stream-play-audio) the mic. Hardware-free by
// default: synthetic video + a 440 Hz tone, so it runs and verifies HEADLESS. A real camera/mic/
// speakers need a build with CVC_ENABLE_SDL (the .ari passes device >= 0).
//
//   ariadne_conference conference_media.ari                                  # navigable window
//   ariadne_conference conference_media.ari --offscreen --png conf.png       # one captured frame
//   ariadne_conference conference_media.ari --frames 32                      # headless, no PNG

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/stream/stream_registry.h> // post-run verification: lookup the opened tokens
#include <cvc/ariadne/stream_intrinsics.h>       // register_stream_intrinsics (stream-open/info/close)
#include <cvc/ariadne/uri.h>
#include <cvc/core/app.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/TouchGestures.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/gl/ariadne/media_verbs.h>  // register_media_intrinsics (capture-camera/-audio, play-audio)
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/stream_node.h>        // register_stream_node_type (type: stream)
#include <cvc/gl/ariadne/stream_verbs.h>       // register_gl_stream_intrinsics (gl-bind-stream)
#include <cvc/gl/ariadne/stream_view_widget.h> // register_stream_view_widget (type: stream_view)
#include <cvc/volume/bounding_box.h>
#include <string>
#include <vector>

using cvc::gl::CameraController;
using cvc::gl::ImGuiBackend;
using cvc::gl::ImGuiOverlay;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
using cvc::gl::TouchGestures;
namespace ari = cvc::ariadne;
namespace sx = cvc::ariadne::stream;

int main(int argc, char **argv) {
  std::string docPath, png;
  bool offscreen = false;
  long frames = 0;
  int width = 1024, height = 768;
  std::vector<std::string> componentPaths;
  for (int i = 1; i < argc; ++i) {
    const char *a = argv[i];
    auto next = [&](const char *d) { return (i + 1 < argc) ? argv[++i] : d; };
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
    else if (!std::strcmp(a, "--component-path"))
      componentPaths.push_back(next(""));
    else if (a[0] != '-' && docPath.empty())
      docPath = a;
  }
  cvc::ariadne::register_cvc_uri_handler(componentPaths);
  const bool capturing = offscreen || !png.empty();
  if (capturing && frames <= 0)
    frames = 16; // let the producers deliver and the texture upload before the capture

  if (docPath.empty()) {
    std::printf("[ariadne_conference] FATAL: give a .ari document (e.g. conference_media.ari)\n");
    return 1;
  }

  cvc::app app;
  SceneGraph sg(app, "conference");
  SceneRenderer view(sg, width, height, offscreen, "main");
  view.setBackground(0.06, 0.07, 0.09);
  CameraController cam(view);
  cam.setMode(CameraController::Mode::Orbit);
  TouchGestures touch(view, cam);
  ImGuiOverlay ui(view);
  ui.attachCamera(cam);
  ui.setVisible(true); // the stream_view window is the point — keep it in captures too

  ari::Runtime rt(app, sg.getStatePrefix());
  ImGuiBackend backend;
  rt.set_backend(&backend);
  backend.install(rt, ui);
  bool quit = false;
  rt.on("quit", [&] { quit = true; });

  // ── The conference host bundle (§8): five registrations, once, before the scene loads ──
  // These are the only C++ a conference .ari needs; everything else is DSL.
  ari::register_stream_intrinsics(app);          // (stream-open/info/close) — core
  cvc::gl::ariadne::register_gl_stream_intrinsics(); // (gl-bind-stream)
  cvc::gl::ariadne::register_media_intrinsics(); // (stream-capture-camera / -audio / play-audio)
  cvc::gl::ariadne::register_stream_node_type(); // type: stream (video-on-quad scene node)
  cvc::gl::ariadne::register_stream_view_widget(backend, app); // type: stream_view (ImGui window)

  // Load the document, realize its scene. NOTE: no stream is opened here — the .ari opens and
  // captures from its on_tick program lane (stream-open needs the document scope, which the init:
  // lane lacks), and a reactive stream_view widget binds to the token each frame as it goes live.
  ari::LoadResult lr = ari::load_file(docPath.c_str());
  if (!lr.ok) {
    std::printf("[ariadne_conference] load failed: %s\n", lr.error.c_str());
    return 1;
  }
  for (const std::string &w : lr.warnings)
    std::printf("[ariadne_conference]   %s\n", w.c_str());
  std::vector<std::string> init_errs;
  ari::run_init(app, sg.getStatePrefix(), lr.init_script, &init_errs);
  for (const std::string &e : init_errs)
    std::printf("[ariadne_conference] init: %s\n", e.c_str());
  rt.set_tick_program(lr.on_tick_script);
  rt.set_root(std::move(lr.root));

  cvc::gl::ariadne::RealizedScene realized;
  if (lr.scene.any()) {
    std::vector<std::string> sw;
    realized = cvc::gl::ariadne::realize_scene(sg, lr.scene, sg.getStatePrefix(), &sw);
    for (const std::string &w : sw)
      std::printf("[ariadne_conference]   %s\n", w.c_str());
    const cvc::bounding_box bb = sg.computeGraphicsBounds();
    if (!bb.isNull())
      cam.frameBounds(bb.minx, bb.miny, bb.minz, bb.maxx, bb.maxy, bb.maxz);
  }

  auto frame_body = [&](double dt) {
    view.processUIEvents();
    touch.update();
    cam.update(dt);
    rt.drain(); // runs the on_tick program lane: (stream-open) + (stream-capture-*) + (stream-play-audio)
    ari::sync_scene_visibility(app, realized.visibility);
    cvc::gl::ariadne::tick_scene(realized, view.renderer(), capturing ? dt : -1.0);
  };

  if (capturing) {
    for (long f = 0; f < frames; ++f) {
      frame_body(1.0 / 30.0);
      if (f + 1 == frames && !png.empty())
        view.writePNG(png.c_str());
      else
        view.render();
    }
    std::printf("[ariadne_conference] captured %ld frame(s)%s%s\n", frames,
                png.empty() ? "" : " -> ", png.c_str());
  } else {
    std::puts("[ariadne_conference] running — close the window or Sim > Quit to exit.");
    while (!view.windowClosed() && !quit) {
      frame_body(1.0 / 120.0);
      view.render();
    }
  }

  // ── Verify the DSL opened the streams (the host opened none) ──
  // The media verbs register channels in the app registry; the reactive stream_view reads the same.
  // The DSL opens streams under <document-root>.streams.<id>; this host's document root is the
  // SceneGraph state prefix ("conference"), so the tokens are conference.streams.<id>.
  auto &reg = sx::stream_registry::for_app(app);
  int opened = 0;
  for (const char *tok : {"conference.streams.selfcam", "conference.streams.selfmic"}) {
    const bool live = reg.lookup(tok) != nullptr;
    std::printf("[ariadne_conference] stream %-16s %s\n", tok, live ? "LIVE" : "(absent)");
    opened += live ? 1 : 0;
  }
  std::printf("[ariadne_conference] %d/2 streams opened from the .ari program lane\n", opened);
  return opened == 2 ? 0 : 2;
}
