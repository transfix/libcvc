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

// ari — run an Ariadne (.ari) document in the terminal, through the FTXUI backend. Headless and
// GL-free: no window, no VTK, no ImGui. A document's scene: block is dropped with a warning (a
// terminal has no scene renderer); its widgets, menus, windows, state and programs all run. arigl
// is the GL runtime (cvcGL), for a real window or a 3D scene behind the terminal widgets.
//
//   ari run      [flags] doc.ari [document args...]
//   ari snapshot [flags] doc.ari [document args...]
//   ari check    [flags] doc.ari
//
// ari's own flags (host_cli.h) come BEFORE the document path; everything after it belongs to the
// document (doc_args.h). The document reads it as the read-only list argv, or declares an args:
// block that ari parses it against: `ari run doc.ari --help` then prints the document's usage,
// and an argument the block doesn't accept exits 2.
//
// run takes over the terminal until a "quit" action (a menu item with `on: quit`, or a program
// calling (quit)) or Ctrl-C. A document with an on_signal: program gets Ctrl-C as SIGINT and
// decides for itself; three Ctrl-Cs within two seconds always quit. Signals sent to the process
// (SIGTERM, SIGHUP, SIGINT, SIGQUIT, SIGUSR1/2) reach on_signal: too, with source "os"; one the
// document doesn't handle ends the run with exit status 128 + the signal number.
// snapshot drains N frames (default 1) and prints the last one as text — for CI, logs, and docs.
// check loads the document and prints its warnings; it exits non-zero when the load fails.
//
// --graphics picks how images draw: auto (the default: Kitty, else Sixel, where the terminal has
// them, else half-blocks), kitty, sixel, color (half-blocks) or mono. --image NAME=FILE publishes
// a picture for the document's `image:` widgets with `src: NAME`. With --ansi and --graphics kitty
// or sixel, snapshot also prints the images, after clearing the screen, at the terminal's cell size
// in pixels (--cell-size WxH where it can't be asked).
//
// The network is off unless a flag turns it on, for run, snapshot and check alike and with either
// backend: --net, --net-allow ORIGIN (repeatable),
// --net-policy FILE and --credential-store NAME|PATH (net_host.h; docs/ARIADNE.md §10.3). Then the
// document's programs get the http verbs, every request checked against the allowed origins, and
// the host keeps any credential they sign in with, out of the document's reach. With it on,
// snapshot waits after its frames until the document's requests have come back (at most 10 s).

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/doc_args.h>
#include <cvc/ariadne/ftxui_backend.h>
#include <cvc/ariadne/host_cli.h>
#include <cvc/ariadne/host_info.h>
#include <cvc/ariadne/host_signals.h>
#include <cvc/ariadne/stream/stream_view.h>
#include <cvc/ariadne/tvision_backend.h>
#include <cvc/ariadne/web_document.h> // the browser page's document, named by URL
#include <cvc/core/app.h>
#include <cvc/image/image.h>
#include <cvc/state/state_exec/async_host.h> // in_flight: snapshot waits for requests
#include <exception>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {

int usage(int code) {
  std::fprintf(code ? stderr : stdout,
               "usage: ari run|snapshot|check [flags] doc.ari [document args...]\n"
               "       ari info [--json] [--functions] [--backend B] [--graphics MODE]\n"
               "\n"
               "flags (before the document):\n"
               "%s"
               "\n"
               "Everything after the document path is the document's own: it reads it as argv, or\n"
               "declares an args: block (then `ari run doc.ari --help` shows its options).\n"
               "\n"
               "info prints what this host provides: the terminal (size, colours, kitty / Sixel\n"
               "graphics), what libcvc was built with, the capabilities a document's requires:\n"
               "and (has? ...) see, and how many functions programs can call (--functions lists\n"
               "them; --json prints it all as JSON).\n"
               "\n"
               "The network is off unless --net, --net-allow, --net-policy or --credential-store\n"
               "turns it on, for run, snapshot and check alike and with either backend. Then the\n"
               "document may reach only the origins allowed, and sign-in tokens stay with ari: a\n"
               "request names one (auth:), it never sees it. With it on, snapshot waits after\n"
               "its frames for the document's requests to come back (at most 10 s).\n",
               cvc::ariadne::host_flags_help().c_str());
  return code;
}

// --graphics MODE -> how images draw (host_cli has checked MODE).
cvc::ariadne::FtxuiBackend::Options::ImageColor image_mode(const std::string &s) {
  using Mode = cvc::ariadne::FtxuiBackend::Options::ImageColor;
  if (s == "kitty")
    return Mode::Kitty;
  if (s == "sixel")
    return Mode::Sixel;
  if (s == "iterm")
    return Mode::Iterm;
  if (s == "color")
    return Mode::Color;
  if (s == "mono")
    return Mode::Mono;
  return Mode::Auto;
}

// snapshot with the network on: drain until no async call (a request) has been in flight for a few
// drains running, so a document that signs in and then fetches is captured with both replies; at
// most kSettleSeconds, for a document that never stops asking.
constexpr double kSettleSeconds = 10.0;

void settle_async_calls(cvc::app &app, cvc::ariadne::AppRuntime &rt) {
  using clock = std::chrono::steady_clock;
  const auto deadline = clock::now() + std::chrono::duration<double>(kSettleSeconds);
  for (int idle = 0; idle < 3 && clock::now() < deadline;) {
    rt.drain();
    idle = app.exec_async_host().in_flight() == 0 ? idle + 1 : 0;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

bool has_flag(const cvc::ariadne::HostOptions &o, const char *flag) {
  return std::find(o.extra_flags.begin(), o.extra_flags.end(), flag) != o.extra_flags.end();
}

// Ask the terminal what it can draw (kitty, Sixel, the cell size) when the image mode could use
// it. Done before the document loads, so its requires: and (has? "term.kitty") see the answer;
// FtxuiBackend::run then does not ask again.
cvc::ariadne::TermGraphics probe_terminal(cvc::ariadne::FtxuiBackend &backend,
                                          const cvc::ariadne::HostOptions &o) {
  using Mode = cvc::ariadne::FtxuiBackend::Options::ImageColor;
  const Mode m = image_mode(o.graphics);
  if (m != Mode::Auto && m != Mode::Kitty && m != Mode::Sixel && m != Mode::Iterm)
    return {};
  cvc::ariadne::TermGraphics g = cvc::ariadne::gfx::probe_terminal();
  if (o.cell_w > 0) {
    g.cell_w = o.cell_w;
    g.cell_h = o.cell_h;
  }
  backend.set_terminal_graphics(g);
  return g;
}

// `ari info`: what this host gives a document, as `ari run` would set it up.
int print_info(const cvc::ariadne::HostOptions &o) {
  namespace ari = cvc::ariadne;
  if (!o.doc.empty()) {
    std::fprintf(stderr,
                 "ari: info takes no document (ari check doc.ari reports what one needs)\n");
    return 2;
  }
  const bool tvision = o.backend == "tvision";
  cvc::app app;
  ari::FtxuiBackend::Options look;
  look.image_color = image_mode(o.graphics);
  std::unique_ptr<ari::FtxuiBackend> backend = tvision ? std::make_unique<ari::TvisionBackend>(look)
                                                       : std::make_unique<ari::FtxuiBackend>(look);
  ari::register_stream_view_widget(*backend, app);
  ari::AppRuntime rt(app, o.prefix.empty() ? "ui.docs.ari" : o.prefix);
  rt.set_backend(*backend);
  if (const std::string error = rt.enable_net(o.net); !error.empty()) {
    std::fprintf(stderr, "ari: %s\n", error.c_str());
    return 2;
  }
  rt.register_verb("quit", [] {}); // every document gets (quit)
  ari::HostInfo term;
  ari::describe_terminal(term);
  ari::TermGraphics graphics;
  const bool probed = term.tty && !tvision; // tvision draws half-blocks only
  if (probed)
    graphics = probe_terminal(*backend, o);
  ari::HostInfo info = ari::gather_host_info(rt.runtime(), "ari");
  info.backend = tvision ? "tvision" : "ftxui";
  if (probed) {
    info.graphics = graphics;
    info.probed = true;
  }
  std::fputs(ari::format_host_info(info, has_flag(o, "--json"), has_flag(o, "--functions")).c_str(),
             stdout);
  return 0;
}

} // namespace

int main(int argc, char **argv) {
  namespace ari = cvc::ariadne;
#ifdef CVC_ARIADNE_WASM_DOC
  // The browser build (ari.html over xterm.js). Opened on ?src=, the page passes a command line
  // naming the document by URL; without one, run the document baked into MEMFS (see the
  // EMSCRIPTEN block in src/ari/CMakeLists.txt).
  const char *baked[] = {"ari", "run", CVC_ARIADNE_WASM_DOC};
  if (argc < 2) {
    argc = static_cast<int>(sizeof baked / sizeof *baked);
    argv = const_cast<char **>(baked);
  }
#endif
  ari::HostOptions o = ari::parse_host_args(argc, argv, {"--json", "--functions"});
#ifdef CVC_ARI_WASM_BACKEND // ari_tvision: its documents run on the tvision desktop
  if (o.backend.empty())
    o.backend = CVC_ARI_WASM_BACKEND;
#endif
#ifdef CVC_ARI_COMPONENT_DIR // the browser build's embedded component library
  o.component_paths.push_back(CVC_ARI_COMPONENT_DIR);
#endif
  if (o.help)
    return usage(0);
  if (!o.error.empty()) {
    std::fprintf(stderr, "ari: %s\n", o.error.c_str());
    return usage(2);
  }
  const std::string &cmd = o.command;
  if (cmd != "run" && cmd != "snapshot" && cmd != "check" && cmd != "info") {
    std::fprintf(stderr, "ari: unknown command '%s'\n", cmd.c_str());
    return usage(2);
  }
  if (!o.backend.empty() && o.backend != "ftxui" && o.backend != "tvision") {
    std::fprintf(stderr, "ari: --backend is ftxui or tvision, not '%s'\n", o.backend.c_str());
    return 2;
  }
  if (cmd == "info")
    return print_info(o);
  if (!o.extra_flags.empty()) {
    std::fprintf(stderr, "ari: %s is for ari info\n", o.extra_flags.front().c_str());
    return usage(2);
  }
  if (!o.png.empty()) {
    std::fprintf(stderr, "ari: --png needs a renderer; arigl can write one\n");
    return 2;
  }
  if (!ari::AppRuntime::have_yaml()) {
    std::fprintf(stderr, "ari: libcvc was built without yaml-cpp, so it cannot load a .ari\n");
    return 1;
  }
  // The browser page names its document by URL (?src=): fetch it into MEMFS first.
  if (ari::is_url_document(o.doc))
    if (const int status = ari::stage_url_document(o, "ari"); status >= 0)
      return status;
  if (cmd != "check" && !ari::FtxuiBackend::available()) {
    std::fprintf(stderr, "ari: libcvc was built without FTXUI (no terminal backend)\n");
    return 1;
  }
  // snapshot / check always lay out with FTXUI; tvision only changes the interactive chrome.
  const bool tvision = o.backend == "tvision" && cmd == "run";
  if (tvision && !ari::TvisionBackend::available()) {
    std::fprintf(stderr, "ari: libcvc was built without tvision (try --backend ftxui)\n");
    return 1;
  }

  cvc::app app;
  // Declared before the AppRuntime, so it outlives it. A TvisionBackend is an FtxuiBackend in
  // hosted mode; its run() is its own (see below).
  ari::FtxuiBackend::Options look;
  look.image_color = image_mode(o.graphics);
  std::unique_ptr<ari::FtxuiBackend> backend = tvision ? std::make_unique<ari::TvisionBackend>(look)
                                                       : std::make_unique<ari::FtxuiBackend>(look);
  for (const auto &[name, file] : o.images) {
    try {
      backend->set_image(name, cvc::image::load(file));
    } catch (const std::exception &e) {
      std::fprintf(stderr, "ari: --image %s: %s\n", file.c_str(), e.what());
      return 1;
    }
  }
  // stream_view: a live stream's frames, or a still image published under its name (--image).
  ari::register_stream_view_widget(*backend, app);
  ari::AppRuntime rt(app, o.prefix.empty() ? "ui.docs.ari" : o.prefix);
  rt.set_backend(*backend);
  if (const std::string error = rt.enable_net(o.net); !error.empty()) {
    std::fprintf(stderr, "ari: %s\n", error.c_str());
    return 2;
  }
  for (const std::string &d : o.component_paths)
    rt.add_component_path(d);
  rt.on("quit", [&] { rt.request_close(); });
  rt.register_verb("quit", [&] { rt.request_close(); }); // (quit), e.g. from an on_signal: program
  rt.set_document_args(o.doc_args, "ari " + cmd + " " + o.doc, /*validate=*/cmd != "check");
  if (cmd == "run" && !tvision)
    probe_terminal(*backend, o); // before the load, so requires: / has? see the terminal

  std::vector<std::string> warnings;
  try {
    warnings = rt.load(o.doc);
  } catch (const ari::document_args_error &e) {
    if (e.help) {
      std::fputs(e.usage.c_str(), stdout);
      return 0;
    }
    std::fprintf(stderr, "ari: %s: %s\n", o.doc.c_str(), e.what());
    const std::string note = ari::misplaced_flag_note("ari", o.doc_args);
    if (!note.empty())
      std::fprintf(stderr, "%s\n", note.c_str());
    std::fprintf(stderr, "\n%s", e.usage.c_str());
    return 2;
  } catch (const std::exception &e) {
    std::fprintf(stderr, "ari: %s\n", e.what());
    return 1;
  }
  for (const std::string &w : warnings)
    std::fprintf(stderr, "ari: warning: %s\n", w.c_str());
  if (!rt.has_args_block()) {
    const std::string note = ari::misplaced_flag_note("ari", o.doc_args);
    if (!note.empty())
      std::fprintf(stderr, "%s\n", note.c_str());
  }
  if (cmd == "check")
    return 0;
  ari::report_running(); // the browser page: the document is loaded

  if (cmd == "snapshot") {
    using Mode = ari::FtxuiBackend::Options::ImageColor;
    const bool pixels =
        o.ansi && (look.image_color == Mode::Kitty || look.image_color == Mode::Sixel ||
                   look.image_color == Mode::Iterm);
    if (pixels) { // the cell size the images are drawn at: the terminal's, or --cell-size
      ari::TermGraphics g = ari::gfx::probe_terminal();
      if (o.cell_w > 0) {
        g.cell_w = o.cell_w;
        g.cell_h = o.cell_h;
      }
      backend->set_terminal_graphics(g);
    }
    std::string frame;
    const int width = std::max(10, o.width ? o.width : 100);
    for (int f = 0; f < std::max(1, o.frames); ++f) {
      rt.drain();
      frame = backend->render_to_string(rt.runtime(), width, o.height, o.ansi);
    }
    if (o.net.enabled) { // the frame shows what the document's requests brought back
      settle_async_calls(app, rt);
      frame = backend->render_to_string(rt.runtime(), width, o.height, o.ansi);
    }
    if (pixels) { // Sixel draws at screen positions, so the frame starts at the top-left
      const int rows = static_cast<int>(std::count(frame.begin(), frame.end(), '\n')) + 1;
      frame = "\x1b[H\x1b[2J" + frame + backend->take_graphics() + "\x1b[" +
              std::to_string(rows + 1) + ";1H";
    }
    std::fputs(frame.c_str(), stdout);
    for (const std::string &w : rt.take_warnings()) // what the programs complained about
      std::fprintf(stderr, "ari: warning: %s\n", w.c_str());
    return 0;
  }

  ari::FtxuiBackend::RunOptions run;
  if (o.fps > 0)
    run.fps = o.fps;
  run.mouse = o.mouse;
  run.on_start = ari::install_signal_traps;
  run.on_stop = ari::restore_signal_traps;
  run.take_signal = ari::take_trapped_signal;
  ari::FtxuiBackend::RunExit how;
  if (tvision) {
    how = static_cast<ari::TvisionBackend &>(*backend).run(rt, run);
  } else {
    how = backend->run(rt, run);
  }
  // The fullscreen screen is gone; anything the read lane complained about is worth seeing now.
  for (const std::string &w : rt.take_warnings())
    std::fprintf(stderr, "ari: warning: %s\n", w.c_str());
  if (how.from_os) // an OS signal the document didn't handle: report it the way a shell expects
    return 128 + ari::signal_number(how.signal);
  return 0;
}
