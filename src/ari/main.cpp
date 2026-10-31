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
//   ari test     [flags] doc.ari...
//
// ari's own flags (host_cli.h) come BEFORE the document path; everything after it belongs to the
// document (doc_args.h). The document reads it as the read-only list argv, or declares an args:
// block that ari parses it against: `ari run doc.ari --help` then prints the document's usage,
// and an argument the block doesn't accept exits 2.
//
// run takes over the terminal until a "quit" action (a menu item with `on: quit`, or a program
// calling (quit [code])) or Ctrl-C. run, snapshot and check exit with the code a program passed
// to (quit code) (0 without one). A document with an on_signal: program gets Ctrl-C as SIGINT and
// decides for itself; three Ctrl-Cs within two seconds always quit. Signals sent to the process
// (SIGTERM, SIGHUP, SIGINT, SIGQUIT, SIGUSR1/2) reach on_signal: too, with source "os"; one the
// document doesn't handle ends the run with exit status 128 + the signal number.
// snapshot drains N frames (default 1) and prints the last one as text — for CI, logs, and docs.
// check loads the document and prints its warnings; it exits non-zero when the load fails.
// test runs each document headless, with no terminal, until it calls (quit code), its tests:
// block has run, --frames N frames, or --timeout (docs/ARIADNE.md "Testing a document"). A
// document fails when it quits non-zero, a program raises an error nothing catches, its load
// fails or warns, or it times out. One line per document (TAP with --tap); exit 1 when any failed.
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
// snapshot waits after its frames until the document is idle, its requests back and its programs
// done (AppRuntime::settle, at most 10 s).

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/doc_args.h>
#include <cvc/ariadne/ftxui_backend.h>
#include <cvc/ariadne/host_cli.h>
#include <cvc/ariadne/host_info.h>
#include <cvc/ariadne/media_player.h> // §5.13 the play policy (--media-play)
#ifdef ARI_MEDIA
#include <cvc/media/player.h> // ari-media: cvcMedia's FFmpeg + SDL3 player
#endif
#ifdef ARI_MEDIA
#include <cvc/media/playback/audio_output.h> // ari-media: sound through SDL3's audio subsystem
#endif
#include <cvc/ariadne/host_signals.h>
#include <cvc/ariadne/stream/stream_view.h>
#include <cvc/ariadne/tvision_backend.h>
#include <cvc/ariadne/web_document.h> // the browser page's document, named by URL
#include <cvc/core/app.h>
#include <cvc/image/image.h>
#include <exception>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

// What `ari info` and the like call this runtime: ari-media is ari with cvcMedia's sound.
#ifdef ARI_MEDIA
constexpr const char *kRuntimeName = "ari-media";
#else
constexpr const char *kRuntimeName = "ari";
#endif

int usage(int code) {
  std::fprintf(code ? stderr : stdout,
               "usage: ari run|snapshot|check [flags] doc.ari [document args...]\n"
               "       ari test [flags] [--tap] [--timeout S] [--frames N] doc.ari...\n"
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
               "test runs each document headless until it calls (quit code), its tests: block\n"
               "has run, --frames N frames, or --timeout S seconds (default 30). A document\n"
               "fails when it quits non-zero, a program raises an error nothing catches (an\n"
               "assert, say), its load fails or warns, or it times out. One line per document,\n"
               "with what failed and in which program; --tap prints TAP. Exits 1 if any failed.\n"
               "run, snapshot and check exit with the code a document passed to (quit code).\n"
               "\n"
               "The network is off unless --net, --net-allow, --net-policy, --credential-store\n"
               "or --credential-env turns it on, for run, snapshot and check alike and with\n"
               "either backend. Then the document may reach only the origins allowed, and\n"
               "sign-in tokens stay with ari: a request names one (auth:, a credential value),\n"
               "it never sees it. With it on, snapshot waits after its frames until the\n"
               "document's requests are back and its programs idle (at most 10 s).\n",
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
  rt.register_quit(); // every document gets (quit [code])
  ari::HostInfo term;
  ari::describe_terminal(term);
  ari::TermGraphics graphics;
  const bool probed = term.tty && !tvision; // tvision draws half-blocks only
  if (probed)
    graphics = probe_terminal(*backend, o);
  ari::HostInfo info = ari::gather_host_info(rt.runtime(), kRuntimeName);
  info.backend = tvision ? "tvision" : "ftxui";
#ifdef ARI_MEDIA
  // ari-media: cvcMedia's SDL3 audio, its subsystem alone (no window, no display).
  info.audio = cvc::media::playback::have_audio_output();
  for (const std::string &dev : cvc::media::playback::list_audio_playback_devices())
    info.audio_info += "device: " + dev + "\n";
#endif
  if (probed) {
    info.graphics = graphics;
    info.probed = true;
  }
  std::fputs(ari::format_host_info(info, has_flag(o, "--json"), has_flag(o, "--functions")).c_str(),
             stdout);
  return 0;
}

// ── ari test ──────────────────────────────────────────────────────────────────────────────────

constexpr double kTestTimeoutSeconds = 30.0;

// How one document's test run went.
struct TestOutcome {
  std::vector<std::string> failures; // each: what failed, and in which program
  std::size_t tests = 0;             // tests: cases that ran to an end
  double seconds = 0;
  bool passed() const { return failures.empty(); }
};

// A warning, without the "ari: " every runtime warning starts with.
std::string trim_ari(std::string w) {
  if (w.rfind("ari: ", 0) == 0)
    w.erase(0, 5);
  return w;
}

// Run one document headless: load it, then drain frames until it quits, its tests: have all run,
// --frames N frames have run, or the timeout. Each frame is laid out (as snapshot does) so its
// bound expressions run, but nothing is drawn.
TestOutcome run_test_document(const cvc::ariadne::HostOptions &o, const std::string &path) {
  namespace ari = cvc::ariadne;
  using clock = std::chrono::steady_clock;
  TestOutcome out;
  const auto start = clock::now();
  const double timeout = o.timeout > 0 ? o.timeout : kTestTimeoutSeconds;
  const auto deadline = start + std::chrono::duration<double>(timeout);
  const auto fail = [&](std::string why) { out.failures.push_back(std::move(why)); };
  const auto finish = [&] {
    out.seconds = std::chrono::duration<double>(clock::now() - start).count();
    return out;
  };

  cvc::app app;
  ari::FtxuiBackend backend{ari::FtxuiBackend::Options{}};
  for (const auto &[name, file] : o.images) {
    try {
      backend.set_image(name, cvc::image::load(file));
    } catch (const std::exception &e) {
      fail("--image " + file + ": " + e.what());
      return finish();
    }
  }
  ari::register_stream_view_widget(backend, app);
  ari::AppRuntime rt(app, o.prefix.empty() ? "ui.docs.ari" : o.prefix);
  rt.set_backend(backend);
  if (const std::string error = rt.enable_net(o.net); !error.empty()) {
    fail(error);
    return finish();
  }
  for (const std::string &d : o.component_paths)
    rt.add_component_path(d);
  rt.register_quit();
  // No command line of its own: declared arguments take their defaults, as in a check.
  rt.set_document_args({}, "ari test " + path, /*validate=*/false);
  try {
    for (const std::string &w : rt.load(path))
      fail("load warning: " + trim_ari(w));
  } catch (const std::exception &e) {
    fail("load failed: " + trim_ari(e.what()));
  }
  if (!out.passed())
    return finish();

  const std::vector<std::pair<std::string, std::string>> &tests = rt.tests();
  std::size_t next = 0; // the next tests: case to start
  bool running = false; // one is running
  bool ended = false;   // ... and has ended, with `how`
  std::string how;      // "" = it finished normally
  const int width = std::max(10, o.width ? o.width : 100);
  const int height = o.height ? o.height : 30;
  bool by_frames = false;
  for (int frame = 0; !rt.should_close(); ++frame) {
    if (o.frames > 0 && frame >= o.frames) {
      by_frames = true;
      break;
    }
    if (clock::now() >= deadline) {
      char after[64];
      std::snprintf(after, sizeof after, "timed out after %g s", timeout);
      std::string what = after;
      if (running)
        what += " in test '" + tests[next - 1].first + "'";
      else if (tests.empty())
        what += " (a document with no tests: block ends with (quit code), or pass --frames N)";
      fail(what);
      break;
    }
    rt.drain();
    backend.render_to_string(rt.runtime(), width, height, false);
    bool warned = false; // a program nothing caught, a bad expression: the document stops
    for (const std::string &w : rt.take_warnings()) {
      fail(trim_ari(w));
      warned = true;
    }
    if (warned)
      break;
    if (ended) {
      if (!how.empty())
        fail("test '" + tests[next - 1].first + "' " + how);
      ++out.tests;
      running = ended = false;
    }
    if (!running && next < tests.size()) { // after the first frame: init: has run, residents too
      running = true;
      rt.runtime().run_program(tests[next++].second, [&](const std::string &h) {
        how = h;
        ended = true;
      });
    } else if (!running && !tests.empty()) {
      break; // every test has run
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  // As snapshot: what the document asked for comes back first, and its pictures.
  if (by_frames && (o.net.enabled || rt.runtime().media_stale())) {
    rt.settle([&] { rt.drain(); });
    for (const std::string &w : rt.take_warnings())
      fail(trim_ari(w));
  }
  if (rt.should_close()) {
    if (rt.exit_code() != 0)
      fail("(quit " + std::to_string(rt.exit_code()) + ")");
    if (out.tests < tests.size())
      fail("quit with " + std::to_string(tests.size() - out.tests) + " of " +
           std::to_string(tests.size()) + " tests not finished");
  }
  return finish();
}

// `ari test`: each document in turn, one line each (or TAP), then a summary. 1 if any failed.
int run_tests(const cvc::ariadne::HostOptions &o) {
  std::vector<std::string> docs{o.doc};
  docs.insert(docs.end(), o.doc_args.begin(), o.doc_args.end());
  for (const std::string &d : docs)
    if (!d.empty() && d[0] == '-') {
      std::fprintf(stderr, "ari: test: '%s' comes after a document; flags go before them\n",
                   d.c_str());
      return 2;
    }
  const bool tap = has_flag(o, "--tap");
  if (tap)
    std::printf("TAP version 13\n1..%zu\n", docs.size());
  std::size_t failed = 0;
  for (std::size_t i = 0; i < docs.size(); ++i) {
    const TestOutcome r = run_test_document(o, docs[i]);
    failed += r.passed() ? 0 : 1;
    char took[64];
    std::snprintf(took, sizeof took, "%.2f s", r.seconds);
    std::string detail = took;
    if (r.tests)
      detail = std::to_string(r.tests) + (r.tests == 1 ? " test, " : " tests, ") + detail;
    if (tap) {
      std::printf("%s %zu - %s # %s\n", r.passed() ? "ok" : "not ok", i + 1, docs[i].c_str(),
                  detail.c_str());
      for (const std::string &f : r.failures)
        std::printf("# %s\n", f.c_str());
    } else {
      std::printf("%s %s (%s)\n", r.passed() ? "ok  " : "FAIL", docs[i].c_str(), detail.c_str());
      for (const std::string &f : r.failures)
        std::printf("     %s\n", f.c_str());
    }
    std::fflush(stdout);
  }
  if (!tap)
    std::printf("ari test: %zu document%s, %zu failed\n", docs.size(), docs.size() == 1 ? "" : "s",
                failed);
  return failed ? 1 : 0;
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
  ari::HostOptions o = ari::parse_host_args(argc, argv, {"--json", "--functions", "--tap"});
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
  // §5.13 video: / audio:: the player (cvcMedia's FFmpeg + SDL3, where this host has it) and where
  // it may play (--media-play; by default not over SSH, where the sound would play remotely).
#ifdef ARI_MEDIA
  cvc::media::register_ariadne_player();
#endif
#ifdef __EMSCRIPTEN__
  cvc::ariadne::media::register_browser_player(); // ari.html: the page's <video> / <audio>
#endif
  if (!o.media_play.empty()) {
    cvc::ariadne::media::PlayPolicy policy = cvc::ariadne::media::PlayPolicy::Auto;
    cvc::ariadne::media::parse_play_policy(o.media_play, policy);
    cvc::ariadne::media::set_play_policy(policy);
  }
  const std::string &cmd = o.command;
  if (cmd != "run" && cmd != "snapshot" && cmd != "check" && cmd != "info" && cmd != "test") {
    std::fprintf(stderr, "ari: unknown command '%s'\n", cmd.c_str());
    return usage(2);
  }
  if (!o.backend.empty() && o.backend != "ftxui" && o.backend != "tvision") {
    std::fprintf(stderr, "ari: --backend is ftxui or tvision, not '%s'\n", o.backend.c_str());
    return 2;
  }
  for (const std::string &f : o.extra_flags)
    if ((f == "--tap") != (cmd == "test")) {
      std::fprintf(stderr, "ari: %s is for ari %s\n", f.c_str(), f == "--tap" ? "test" : "info");
      return usage(2);
    }
  if (o.timeout > 0 && cmd != "test") {
    std::fprintf(stderr, "ari: --timeout is for ari test\n");
    return usage(2);
  }
  if (cmd == "info")
    return print_info(o);
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
  if (cmd == "test")
    return run_tests(o);
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
  rt.register_quit(); // on: quit and (quit [code]), e.g. from an on_signal: program
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
    return rt.exit_code(); // an init: that called (quit code)
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
      if (rt.should_close()) // (quit code): the frame it quit on
        break;
    }
    // The frame shows what the requests brought back, and the pictures it draws.
    if ((o.net.enabled || rt.runtime().media_stale()) && !rt.should_close())
      rt.settle_and_render([&] { rt.drain(); },
                           [&] { frame = backend->render_to_string(rt.runtime(), width, o.height,
                                                                   o.ansi); });
    if (pixels) { // Sixel draws at screen positions, so the frame starts at the top-left
      const int rows = static_cast<int>(std::count(frame.begin(), frame.end(), '\n')) + 1;
      frame = "\x1b[H\x1b[2J" + frame + backend->take_graphics() + "\x1b[" +
              std::to_string(rows + 1) + ";1H";
    }
    std::fputs(frame.c_str(), stdout);
    for (const std::string &w : rt.take_warnings()) // what the programs complained about
      std::fprintf(stderr, "ari: warning: %s\n", w.c_str());
    return rt.exit_code();
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
  return rt.exit_code();
}
