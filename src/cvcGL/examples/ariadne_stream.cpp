// ariadne_stream — the Phase-3 streaming "holy grail" demo: one live video stream shown TWO ways at
// once, both driven from a single .ari document:
//   1. as the MAIN SCENE — a `type: stream` scene node (a UV quad textured with the live frames,
//      filling the viewport), and
//   2. in a draggable/resizable ImGui window — a `type: stream_view` Ariadne widget the SAME stream
//      is routed to (it also shows a static image when no stream is live).
//
// The video source is hardware-free by default (a synthetic moving test pattern), so the demo runs
// and verifies headless; pass --camera to feed it from a real SDL3 camera (cvc::gl::capture) when
// one is present. Everything below the stream open + the two registrations is the stock cvcGL
// Ariadne runner (cf. ariadne_hello): load the doc, realize the scene, frame the camera, render —
// a window, or a one-frame offscreen PNG capture for verification.
//
//   ariadne_stream stream.ari                                   # navigable window (synthetic)
//   ariadne_stream stream.ari --camera                          # from a real camera if present
//   ariadne_stream stream.ari --offscreen --png stream.png      # one captured frame (headless)

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/stream/stream.h>
#include <cvc/ariadne/stream/synthetic_source.h>
#include <cvc/ariadne/uri.h>
#include <cvc/core/app.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/TouchGestures.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/stream_node.h>        // register_stream_node_type (the viewport quad)
#include <cvc/gl/ariadne/stream_view_widget.h> // register_stream_view_widget (the ImGui window)
#include <cvc/volume/bounding_box.h>
#include <memory>
#include <string>
#include <vector>

#if CVC_ENABLE_SDL
#include <cvc/gl/capture/camera_source.h> // --camera: a real SDL3 camera as the source
#endif

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
  bool offscreen = false, useCamera = false;
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
    } else if (!std::strcmp(a, "--camera"))
      useCamera = true;
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
  cvc::ariadne::register_cvc_uri_handler(componentPaths);
  const bool capturing = offscreen || !png.empty();
  if (capturing && frames <= 0)
    frames = 8; // a few frames so the producer delivers and the texture uploads before the capture

  cvc::app app;
  SceneGraph sg(app, "stream");
  SceneRenderer view(sg, width, height, offscreen, "main");
  view.setBackground(0.06, 0.07, 0.09);
  CameraController cam(view);
  cam.setMode(CameraController::Mode::Orbit);
  TouchGestures touch(view, cam);
  ImGuiOverlay ui(view);
  ui.attachCamera(cam);
  ui.setVisible(true); // the stream_view WINDOW is the point of this demo — keep it in captures too

  ari::Runtime rt(app, sg.getStatePrefix());
  ImGuiBackend backend;
  rt.set_backend(&backend);
  backend.install(rt, ui);
  bool quit = false;
  rt.on("quit", [&] { quit = true; });

  // The two Phase-3 stream sinks this demo needs, registered once before the scene loads:
  cvc::gl::ariadne::register_stream_node_type(); // the `type: stream` viewport quad
  cvc::gl::ariadne::register_stream_view_widget(backend,
                                                app); // the `type: stream_view` ImGui widget

  // Open ONE video stream at the app root; both sinks resolve it by this token. The source is a
  // synthetic moving test pattern (hardware-free) unless --camera opened a real device.
  const int sw = 640, sh = 480;
  int fw = sw, fh = sh;
  std::unique_ptr<sx::frame_source> source;
  std::string srcName = "synthetic test pattern";
#if CVC_ENABLE_SDL
  if (useCamera) {
    cvc::gl::capture::camera_open camo = cvc::gl::capture::open_camera(0);
    if (camo.source) {
      source = std::move(camo.source);
      fw = camo.width;
      fh = camo.height;
      srcName = "camera: " + camo.name;
    } else {
      std::printf("[ariadne_stream] --camera: no camera opened; using the synthetic source.\n");
    }
  }
#else
  (void)useCamera;
#endif
  if (!source)
    source = std::make_unique<sx::synthetic_source>(fw, fh, 30.0);

  sx::stream_params sp;
  sp.id = "cam0";
  sp.format.kind = sx::frame_kind::video_raw;
  sp.format.codec = "rgba8";
  sp.format.w = fw;
  sp.format.h = fh;
  sp.format.stride = fw * 4;
  sp.expected_subscribers = 3; // the scene-node quad + the stream_view widget (+ headroom): the
                               // pool is fixed-size and refuses rather than under-provisions, so
                               // BOTH sinks must be budgeted here or the second one gets no frames
  std::unique_ptr<sx::stream> stream = sx::stream::open(app, sp);
  if (!stream) {
    std::printf("[ariadne_stream] FATAL: stream::open failed\n");
    return 1;
  }
  stream->start_producer(std::move(source), 30.0);
  std::printf("[ariadne_stream] stream '%s' open: %dx%d rgba8 (%s)\n", stream->token().c_str(), fw,
              fh, srcName.c_str());

  // Load the document, realize its scene, frame the camera to it.
  cvc::gl::ariadne::RealizedScene realized;
  if (docPath.empty()) {
    std::printf("[ariadne_stream] FATAL: give a .ari document (e.g. stream.ari)\n");
    return 1;
  }
  ari::LoadResult lr = ari::load_file(docPath.c_str());
  if (!lr.ok) {
    std::printf("[ariadne_stream] load failed: %s\n", lr.error.c_str());
    return 1;
  }
  for (const std::string &w : lr.warnings)
    std::printf("[ariadne_stream]   %s\n", w.c_str());
  std::vector<std::string> init_errs;
  ari::run_init(app, sg.getStatePrefix(), lr.init_script, &init_errs);
  rt.set_tick_program(lr.on_tick_script);
  rt.set_root(std::move(lr.root));
  if (lr.scene.any()) {
    std::vector<std::string> sw2;
    realized = cvc::gl::ariadne::realize_scene(sg, lr.scene, sg.getStatePrefix(), &sw2);
    for (const std::string &w : sw2)
      std::printf("[ariadne_stream]   %s\n", w.c_str());
    std::printf("[ariadne_stream] scene: %zu node(s)\n", realized.created.size());
    const cvc::bounding_box bb = sg.computeGraphicsBounds();
    if (!bb.isNull())
      cam.frameBounds(bb.minx, bb.miny, bb.minz, bb.maxx, bb.maxy, bb.maxz);
  }

  auto frame_body = [&](double dt) {
    view.processUIEvents();
    touch.update();
    cam.update(dt);
    rt.drain();
    ari::sync_scene_visibility(app, realized.visibility);
    cvc::gl::ariadne::tick_scene(realized, view.renderer()); // services the stream node's texture
  };

  if (capturing) {
    for (long f = 0; f < frames; ++f) {
      frame_body(1.0 / 30.0);
      if (f + 1 == frames && !png.empty())
        view.writePNG(png.c_str());
      else
        view.render();
    }
    std::printf("[ariadne_stream] captured %ld frame(s)%s%s\n", frames, png.empty() ? "" : " -> ",
                png.c_str());
  } else {
    std::puts("[ariadne_stream] running — close the window or Sim > Quit to exit.");
    while (!view.windowClosed() && !quit) {
      frame_body(1.0 / 120.0);
      view.render();
    }
  }

  stream->close(); // join the producer before teardown (render-thread contract)
  return 0;
}
