/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// arigl — the GL Ariadne runtime: run a .ari document over cvcGL (VTK + ImGui). `ari` is the
// GL-free core runtime and `arisrv` the headless server; arigl is the one that draws a 3D scene.
//
//   arigl run      [flags] doc.ari [document args...]
//   arigl snapshot [flags] doc.ari [document args...]
//   arigl check    [flags] doc.ari
//
// By default the document runs in a real window with a GL context: its widgets are ImGui over the
// VTK scene. With --terminal it runs in the terminal instead: the widgets go through libcvc's
// terminal backends (FTXUI, or --backend tvision) and the scene is rendered offscreen and drawn
// behind them (cvc::gl::ariadne::TerminalScene).
//
// The flags are the ones every Ariadne runtime takes (host_cli.h) plus --terminal, and they come
// BEFORE the document path. Everything after it belongs to the document (doc_args.h): it reads it
// as argv, or declares an args: block (`arigl run doc.ari --help` then shows its options).
//
// run ends on a "quit" action (a menu item with `on: quit`, or a program calling (quit)), when the
// window closes, or on Ctrl-C in the terminal. OS signals (SIGTERM, SIGHUP, SIGINT, SIGQUIT,
// SIGUSR1/2) go to the document's on_signal: in both modes; one it does not handle ends the run
// with exit status 128 + the signal number.
// snapshot renders N frames on a fixed clock: in a window, offscreen at --width x --height pixels
// (default 1024 x 768), writing the last one to --png FILE; with --terminal, the last frame as
// text (--ansi keeps its colours), plus the rendered scene as --png if given.
// check loads the document as run would, offscreen, and prints its warnings.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/doc_args.h>
#include <cvc/ariadne/ftxui_backend.h>
#include <cvc/ariadne/host_cli.h>
#include <cvc/ariadne/host_signals.h>
#include <cvc/ariadne/tvision_backend.h>
#include <cvc/core/app.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/SdlInput.h>
#include <cvc/gl/StageLighting.h>
#include <cvc/gl/TouchGestures.h>
#include <cvc/gl/ariadne/AriRuntime.h>
#include <cvc/gl/ariadne/TerminalScene.h>
#include <cvc/gl/ariadne/bundles.h>
#include <cvc/gl/ariadne/scene_panels.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/image/image.h>
#include <exception>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {

namespace ari = cvc::ariadne;
namespace gla = cvc::gl::ariadne;

const char *const kDefaultPrefix = "ui.docs.ari"; // the same root ari uses, in both modes

int usage(int code) {
  std::fprintf(code ? stderr : stdout,
               "usage: arigl run|snapshot|check [flags] doc.ari [document args...]\n"
               "\n"
               "flags (before the document):\n"
               "  --terminal             run in the terminal, the scene drawn behind the widgets\n"
               "%s"
               "\n"
               "A window by default (ImGui over the VTK scene). snapshot renders offscreen: write\n"
               "it with --png FILE, or with --terminal print it as text. Everything after the\n"
               "document path is the document's own: it reads it as argv, or declares an args:\n"
               "block (then `arigl run doc.ari --help` shows its options).\n",
               ari::host_flags_help().c_str());
  return code;
}

// Load the document. Returns -1 to carry on, else the exit status: 0 after printing the
// document's --help, 2 for arguments it does not accept, 1 for a failed load.
template <class Runtime> int load_document(Runtime &rt, const ari::HostOptions &o) {
  std::vector<std::string> warnings;
  try {
    warnings = rt.load(o.doc);
  } catch (const ari::document_args_error &e) {
    if (e.help) {
      std::fputs(e.usage.c_str(), stdout);
      return 0;
    }
    std::fprintf(stderr, "arigl: %s: %s\n", o.doc.c_str(), e.what());
    const std::string note = ari::misplaced_flag_note("arigl", o.doc_args);
    if (!note.empty())
      std::fprintf(stderr, "%s\n", note.c_str());
    std::fprintf(stderr, "\n%s", e.usage.c_str());
    return 2;
  } catch (const std::exception &e) {
    std::fprintf(stderr, "arigl: %s\n", e.what());
    return 1;
  }
  for (const std::string &w : warnings)
    std::fprintf(stderr, "arigl: warning: %s\n", w.c_str());
  return -1;
}

void print_warnings(std::vector<std::string> warnings) {
  for (const std::string &w : warnings)
    std::fprintf(stderr, "arigl: warning: %s\n", w.c_str());
}

void note_misplaced(const ari::AppRuntime &rt, const ari::HostOptions &o) {
  if (rt.has_args_block())
    return;
  const std::string note = ari::misplaced_flag_note("arigl", o.doc_args);
  if (!note.empty())
    std::fprintf(stderr, "%s\n", note.c_str());
}

// --graphics MODE -> how a terminal draws images (host_cli has checked MODE).
ari::FtxuiBackend::Options::ImageColor image_mode(const std::string &s) {
  using Mode = ari::FtxuiBackend::Options::ImageColor;
  if (s == "kitty")
    return Mode::Kitty;
  if (s == "sixel")
    return Mode::Sixel;
  if (s == "color")
    return Mode::Color;
  if (s == "mono")
    return Mode::Mono;
  return Mode::Auto;
}

// --image NAME=FILE: publish each picture for the document's image: widgets. False on a bad file.
template <class Backend> bool publish_images(Backend &backend, const ari::HostOptions &o) {
  for (const auto &[name, file] : o.images) {
    try {
      backend.set_image(name, cvc::image::load(file));
    } catch (const std::exception &e) {
      std::fprintf(stderr, "arigl: --image %s: %s\n", file.c_str(), e.what());
      return false;
    }
  }
  return true;
}

// The SDL-free cvc::gl::InputEvent, as the runtime's cvc::ariadne::InputEvent.
ari::InputEvent to_ari(const cvc::gl::InputEvent &e) {
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
}

// ── a real window (ImGui over the VTK scene) ────────────────────────────────────────────────────

int run_window(const ari::HostOptions &o) {
  const bool offscreen = o.command != "run"; // snapshot and check never open a window
  if (o.command == "snapshot" && o.png.empty()) {
    std::fprintf(stderr, "arigl: a window snapshot needs --png FILE (or use --terminal)\n");
    return 2;
  }
  const int width = o.width ? o.width : 1024, height = o.height ? o.height : 768;

  cvc::app app;
  cvc::gl::SceneGraph sg(app, o.prefix.empty() ? kDefaultPrefix : o.prefix);
  cvc::gl::SceneRenderer view(sg, width, height, offscreen, "arigl");
  view.setBackground(0.09, 0.10, 0.12);
  cvc::gl::CameraController cam(view);
  cam.setMode(cvc::gl::CameraController::Mode::Orbit);
  cvc::gl::TouchGestures touch(view, cam); // Tab orbit / fly, drag to look
  cvc::gl::ImGuiOverlay ui(view);
  ui.attachCamera(cam);
  ui.setVisible(true);

  gla::AriRuntime rt(view, cam, ui);
  gla::register_gl_basics(app);
  gla::register_streaming(app);
  gla::register_window_widgets(rt.backend(), app);
  if (!publish_images(rt.backend(), o))
    return 1;
  for (const std::string &d : o.component_paths)
    rt.add_component_path(d);
  rt.on("quit", [&] { rt.request_close(); });
  rt.register_verb("quit", [&] { rt.request_close(); });
  rt.app_runtime().set_document_args(o.doc_args, "arigl " + o.command + " " + o.doc,
                                     /*validate=*/o.command != "check");
  if (const int status = load_document(rt, o); status >= 0)
    return status;
  note_misplaced(rt.app_runtime(), o);
  // The rich scene panels (scene_panel / stage_lighting_panel / scene_menu), for a document that
  // uses them rather than the declarative components. They need the realized lighting rig.
  if (const gla::RealizedScene *realized = rt.realized()) {
    cvc::gl::StageLighting *rig = realized->rigs.empty() ? nullptr : realized->rigs.front().get();
    gla::register_scene_panels(rt.backend(), sg, rig);
  }
  if (o.command == "check")
    return 0;

  if (o.command == "snapshot") {
    rt.set_fixed_clock(true); // reproducible: the scene clock advances 1/30 s a frame
    const int frames = std::max(1, o.frames);
    for (int f = 0; f < frames; ++f) {
      rt.frame(1.0 / 30.0);
      if (f + 1 == frames)
        view.writePNG(o.png.c_str()); // renders and writes the last frame
      else
        rt.render();
    }
    print_warnings(rt.take_warnings());
    return 0;
  }

  // Input the VTK interactor does not carry: touch navigation, SDL events (a wasm canvas), and the
  // OS signals trapped below. Fed before the drain, so the document sees them this frame.
  cvc::gl::SdlInput sdl;
  sdl.init();
  std::string exit_signal;
  rt.set_input_pump([&] {
    touch.update();
    for (const cvc::gl::InputEvent &e : sdl.poll())
      rt.post_input(to_ari(e));
    for (std::string sig = ari::take_trapped_signal(); !sig.empty();
         sig = ari::take_trapped_signal())
      if (!rt.post_signal(sig, "os")) { // no on_signal: the host's default is to stop
        exit_signal = sig;
        rt.request_close();
      }
  });
  ari::install_signal_traps();
  view.setFrameYield(cvc::gl::SceneRenderer::FrameYield::App);
  const double fps = o.fps > 0 ? o.fps : 120.0;
  const auto period = std::chrono::duration<double>(1.0 / fps);
  while (!rt.should_close()) {
    const auto start = std::chrono::steady_clock::now();
    rt.step(1.0 / fps);
    print_warnings(rt.take_warnings());
    std::this_thread::sleep_until(start + period);
  }
  ari::restore_signal_traps();
  cam.detach();
  if (!exit_signal.empty())
    return 128 + ari::signal_number(exit_signal);
  return 0;
}

// ── the terminal (FTXUI / tvision widgets, the scene rendered behind them) ──────────────────────

// TerminalScene plus the scene-node customs check a GL host makes before realizing.
class CheckedTerminalScene : public gla::TerminalScene {
public:
  using gla::TerminalScene::TerminalScene;
  bool verify(const ari::LoadResult &loaded, std::vector<std::string> *errors) override {
    return gla::verify_scene_customs(loaded, errors);
  }
};

int run_terminal(const ari::HostOptions &o) {
  if (!ari::FtxuiBackend::available()) {
    std::fprintf(stderr, "arigl: libcvc was built without FTXUI (no terminal backend)\n");
    return 1;
  }
  // snapshot / check always lay out with FTXUI; tvision only changes the interactive chrome.
  const bool tvision = o.backend == "tvision" && o.command == "run";
  if (tvision && !ari::TvisionBackend::available()) {
    std::fprintf(stderr, "arigl: libcvc was built without tvision (try --backend ftxui)\n");
    return 1;
  }

  cvc::app app;
  // Lifetime order: the backend outlives the scene adapter, which outlives the AppRuntime.
  ari::FtxuiBackend::Options look;
  look.image_color = image_mode(o.graphics);
  std::unique_ptr<ari::FtxuiBackend> backend = tvision ? std::make_unique<ari::TvisionBackend>(look)
                                                       : std::make_unique<ari::FtxuiBackend>(look);
  if (!publish_images(*backend, o))
    return 1;
  CheckedTerminalScene scene(app, *backend);
  ari::AppRuntime rt(app, o.prefix.empty() ? kDefaultPrefix : o.prefix, &scene);
  rt.set_backend(*backend);
  gla::register_gl_basics(app);
  gla::register_streaming(app);
  for (const std::string &d : o.component_paths)
    rt.add_component_path(d);
  rt.on("quit", [&] { rt.request_close(); });
  rt.register_verb("quit", [&] { rt.request_close(); });
  rt.set_document_args(o.doc_args, "arigl " + o.command + " --terminal " + o.doc,
                       /*validate=*/o.command != "check");
  if (const int status = load_document(rt, o); status >= 0)
    return status;
  note_misplaced(rt, o);
  if (o.command == "check")
    return 0;

  if (o.command == "snapshot") {
    const int width = std::max(10, o.width ? o.width : 100);
    const int height = std::max(4, o.height ? o.height : 30);
    using Mode = ari::FtxuiBackend::Options::ImageColor;
    const bool pixels =
        o.ansi && (look.image_color == Mode::Kitty || look.image_color == Mode::Sixel);
    if (pixels) { // the cell size images are drawn at: the terminal's, or --cell-size
      ari::TermGraphics g = ari::gfx::probe_terminal();
      if (o.cell_w > 0) {
        g.cell_w = o.cell_w;
        g.cell_h = o.cell_h;
      }
      backend->set_terminal_graphics(g);
    }
    backend->set_screen_size(width, height);
    std::string frame;
    for (int f = 0; f < std::max(2, o.frames); ++f) { // the first drain realizes; then a frame
      rt.drain(); // services and renders the scene, publishing the frame
      frame = backend->render_to_string(rt.runtime(), width, height, o.ansi);
    }
    if (pixels) { // Sixel draws at screen positions, so the frame starts at the top-left
      const int rows = static_cast<int>(std::count(frame.begin(), frame.end(), '\n')) + 1;
      frame = "\x1b[H\x1b[2J" + frame + backend->take_graphics() + "\x1b[" +
              std::to_string(rows + 1) + ";1H";
    }
    std::fputs(frame.c_str(), stdout);
    if (!o.png.empty())
      scene.write_png(o.png);
    print_warnings(rt.take_warnings());
    return 0;
  }

  ari::FtxuiBackend::RunOptions run;
  if (o.fps > 0)
    run.fps = o.fps;
  run.mouse = o.mouse;
  run.on_start = ari::install_signal_traps;
  run.on_stop = ari::restore_signal_traps;
  run.take_signal = ari::take_trapped_signal;
  run.background_input = [&](const ari::TermEvent &e) { return scene.handle_input(e); };
  ari::FtxuiBackend::RunExit how;
  if (tvision)
    how = static_cast<ari::TvisionBackend &>(*backend).run(rt, run);
  else
    how = backend->run(rt, run);
  print_warnings(rt.take_warnings());
  if (how.from_os) // an OS signal the document didn't handle: report it the way a shell expects
    return 128 + ari::signal_number(how.signal);
  return 0;
}

} // namespace

int main(int argc, char **argv) {
  const ari::HostOptions o = ari::parse_host_args(argc, argv, {"--terminal"});
  if (o.help)
    return usage(0);
  if (!o.error.empty()) {
    std::fprintf(stderr, "arigl: %s\n", o.error.c_str());
    return usage(2);
  }
  if (o.command != "run" && o.command != "snapshot" && o.command != "check") {
    std::fprintf(stderr, "arigl: unknown command '%s'\n", o.command.c_str());
    return usage(2);
  }
  if (!o.backend.empty() && o.backend != "ftxui" && o.backend != "tvision") {
    std::fprintf(stderr, "arigl: --backend is ftxui or tvision, not '%s'\n", o.backend.c_str());
    return 2;
  }
  const bool terminal =
      std::find(o.extra_flags.begin(), o.extra_flags.end(), "--terminal") != o.extra_flags.end();
  if (!terminal && (!o.backend.empty() || !o.graphics.empty() || o.cell_w > 0)) {
    std::fprintf(stderr, "arigl: --backend, --graphics and --cell-size are for a terminal; add "
                         "--terminal\n");
    return 2;
  }
  if (!ari::AppRuntime::have_yaml()) {
    std::fprintf(stderr, "arigl: libcvc was built without yaml-cpp, so it cannot load a .ari\n");
    return 1;
  }
  return terminal ? run_terminal(o) : run_window(o);
}
