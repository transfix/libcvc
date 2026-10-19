// ariadne_terminal — run an Ariadne (.ari) document, 3D scene included, in a terminal.
//
// The terminal counterpart of ariadne_hello: the widgets go through a terminal backend (FTXUI, or
// tvision's desktop with --backend tvision) and the document's scene: is rendered by an offscreen
// cvcGL viewer and drawn behind them as half-block cells (cvc::gl::ariadne::TerminalScene). Drag
// on the background to orbit, middle-drag to pan, wheel to zoom. No window or display is needed.
//
//   ariadne_terminal doc.ari [--backend ftxui|tvision] [--fps N] [--component-path DIR]...
//   ariadne_terminal doc.ari --snapshot [--width W] [--height H] [--frames N] [--ansi]
//                              [--png frame.png]
//
// --snapshot renders N frames (default 2) at W x H cells, prints the last as text (--ansi for the
// colours), and with --png also writes the offscreen frame at full resolution.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/ftxui_backend.h>
#include <cvc/ariadne/tvision_backend.h>
#include <cvc/core/app.h>
#include <cvc/gl/ariadne/TerminalScene.h>
#include <cvc/gl/ariadne/extensions.h> // register_cvcgl_extensions
#include <exception>
#include <memory>
#include <string>
#include <vector>

int main(int argc, char **argv) {
  std::string doc, png;
  std::vector<std::string> componentPaths;
  bool snapshot = false, ansi = false, tvision = false;
  int width = 100, height = 30, frames = 2;
  cvc::ariadne::FtxuiBackend::RunOptions run;
  for (int i = 1; i < argc; ++i) {
    const char *a = argv[i];
    auto next = [&]() -> const char * {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "ariadne_terminal: %s needs a value\n", a);
        std::exit(2);
      }
      return argv[++i];
    };
    if (!std::strcmp(a, "--snapshot"))
      snapshot = true;
    else if (!std::strcmp(a, "--ansi"))
      ansi = true;
    else if (!std::strcmp(a, "--png"))
      png = next();
    else if (!std::strcmp(a, "--width"))
      width = std::max(10, std::atoi(next()));
    else if (!std::strcmp(a, "--height"))
      height = std::max(4, std::atoi(next()));
    else if (!std::strcmp(a, "--frames"))
      frames = std::max(1, std::atoi(next()));
    else if (!std::strcmp(a, "--fps"))
      run.fps = std::atoi(next());
    else if (!std::strcmp(a, "--component-path"))
      componentPaths.push_back(next());
    else if (!std::strcmp(a, "--backend"))
      tvision = std::string(next()) == "tvision";
    else if (a[0] != '-' && doc.empty())
      doc = a;
    else {
      std::fprintf(stderr, "ariadne_terminal: unexpected argument '%s'\n", a);
      return 2;
    }
  }
  if (doc.empty()) {
    std::fprintf(stderr, "usage: ariadne_terminal doc.ari [--backend ftxui|tvision] [--snapshot "
                         "[--width W] [--height H] [--frames N] [--ansi] [--png out.png]]\n");
    return 2;
  }
  if (!cvc::ariadne::FtxuiBackend::available() || !cvc::ariadne::AppRuntime::have_yaml()) {
    std::fprintf(stderr, "ariadne_terminal: libcvc lacks FTXUI or yaml-cpp\n");
    return 1;
  }
  tvision = tvision && !snapshot && cvc::ariadne::TvisionBackend::available();

  cvc::app app;
  cvc::gl::ariadne::register_cvcgl_extensions(app); // cvcGL's scene node types (L-systems, ...)
  // Member order is lifetime order: the backend outlives the scene adapter, which outlives the
  // AppRuntime that drives both.
  std::unique_ptr<cvc::ariadne::FtxuiBackend> backend =
      tvision ? std::make_unique<cvc::ariadne::TvisionBackend>()
              : std::make_unique<cvc::ariadne::FtxuiBackend>();
  cvc::gl::ariadne::TerminalScene scene(app, *backend);
  cvc::ariadne::AppRuntime rt(app, "ui.docs.ari", &scene);
  rt.set_backend(*backend);
  for (const std::string &d : componentPaths)
    rt.add_component_path(d);
  rt.on("quit", [&] { rt.request_close(); });
  rt.register_verb("quit", [&] { rt.request_close(); });
  try {
    for (const std::string &w : rt.load(doc))
      std::fprintf(stderr, "ariadne_terminal: warning: %s\n", w.c_str());
  } catch (const std::exception &e) {
    std::fprintf(stderr, "ariadne_terminal: %s\n", e.what());
    return 1;
  }

  if (snapshot) {
    backend->set_screen_size(width, height);
    std::string frame;
    for (int f = 0; f < frames; ++f) {
      rt.drain(); // services and renders the scene, publishing the frame
      frame = backend->render_to_string(rt.runtime(), width, height, ansi);
    }
    std::fputs(frame.c_str(), stdout);
    if (!png.empty())
      scene.write_png(png);
    return 0;
  }

  run.background_input = [&](const cvc::ariadne::TermEvent &e) { return scene.handle_input(e); };
  if (tvision)
    static_cast<cvc::ariadne::TvisionBackend &>(*backend).run(rt, run);
  else
    backend->run(rt, run);
  return 0;
}
