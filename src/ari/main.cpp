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
// terminal has no scene renderer); its widgets, menus, windows, state and programs all run.
//
//   ari run      <doc.ari> [--prefix P] [--fps N] [--no-mouse] [--component-path DIR]...
//   ari snapshot <doc.ari> [--prefix P] [--width W] [--height H] [--frames N] [--ansi]
//   ari check    <doc.ari> [--prefix P]
//
// run takes over the terminal until a "quit" action (a menu item with `on: quit`, or a program
// calling (quit)) or Ctrl-C. A document with an on_signal: program gets Ctrl-C as SIGINT and
// decides for itself; three Ctrl-Cs within two seconds always quit. Signals sent to the process
// (SIGTERM, SIGHUP, SIGINT, SIGQUIT, SIGUSR1/2) reach on_signal: too, with source "os"; one the
// document doesn't handle ends the run with exit status 128 + the signal number.
// snapshot drains N frames (default 1) and prints the last one as text — for CI, logs, and docs.
// check loads the document and prints its warnings; it exits non-zero when the load fails.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/ftxui_backend.h>
#include <cvc/ariadne/tvision_backend.h>
#include <cvc/core/app.h>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#ifndef _WIN32
#include <csignal>
#endif
#ifdef __linux__
#include <climits>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace {

#ifndef _WIN32
// OS signals for the document's on_signal:. A handler only sets a flag (all it may safely do); the
// loop polls them through RunOptions::take_signal. Process-wide state, so it lives here in the
// host program rather than in libcvc.
struct Trap {
  int signo;
  const char *name;
};
constexpr Trap kTraps[] = {{SIGTERM, "SIGTERM"}, {SIGHUP, "SIGHUP"},   {SIGINT, "SIGINT"},
                           {SIGQUIT, "SIGQUIT"}, {SIGUSR1, "SIGUSR1"}, {SIGUSR2, "SIGUSR2"}};
constexpr int kTrapCount = static_cast<int>(sizeof(kTraps) / sizeof(kTraps[0]));
volatile std::sig_atomic_t g_raised[kTrapCount] = {};
struct sigaction g_previous[kTrapCount];

extern "C" void on_os_signal(int signo) {
  for (int i = 0; i < kTrapCount; ++i)
    if (kTraps[i].signo == signo)
      g_raised[i] = 1;
}

void install_traps() {
  struct sigaction sa {};
  sa.sa_handler = on_os_signal;
  sigemptyset(&sa.sa_mask);
  for (int i = 0; i < kTrapCount; ++i)
    sigaction(kTraps[i].signo, &sa, &g_previous[i]);
}

void restore_traps() {
  for (int i = 0; i < kTrapCount; ++i)
    sigaction(kTraps[i].signo, &g_previous[i], nullptr);
}

std::string take_trapped() {
  for (int i = 0; i < kTrapCount; ++i)
    if (g_raised[i]) {
      g_raised[i] = 0;
      return kTraps[i].name;
    }
  return {};
}

int signal_number(const std::string &name) {
  for (const Trap &t : kTraps)
    if (name == t.name)
      return t.signo;
  return 0;
}
#endif

int usage(int code) {
  std::fprintf(code ? stderr : stdout,
               "usage:\n"
               "  ari run      <doc.ari> [--backend ftxui|tvision] [--prefix P] [--fps N] "
               "[--no-mouse] [--component-path DIR]...\n"
               "  ari snapshot <doc.ari> [--prefix P] [--width W] [--height H] [--frames N] "
               "[--ansi]\n"
               "  ari check    <doc.ari> [--prefix P]\n");
  return code;
}

} // namespace

int main(int argc, char **argv) {
#ifdef CVC_ARIADNE_WASM_DOC
  // The browser build (ari.html over xterm.js) has no command line. Run the document baked into
  // MEMFS, with the embedded component library at /components so cvc://components/*.ari resolves
  // (see the EMSCRIPTEN block in src/ari/CMakeLists.txt).
  const char *baked[] = {"ari", "run", CVC_ARIADNE_WASM_DOC, "--component-path", "/"};
  if (argc < 2) {
    argc = static_cast<int>(sizeof baked / sizeof *baked);
    argv = const_cast<char **>(baked);
  }
#endif
  if (argc < 2)
    return usage(2);
  const std::string cmd = argv[1];
  if (cmd == "-h" || cmd == "--help" || cmd == "help")
    return usage(0);
  if (cmd != "run" && cmd != "snapshot" && cmd != "check") {
    std::fprintf(stderr, "ari: unknown command '%s'\n", cmd.c_str());
    return usage(2);
  }

  std::string doc;
  std::string prefix = "ui.docs.ari";
  std::vector<std::string> componentPaths;
  int width = 100, height = 0, frames = 1;
  bool ansi = false;
  bool tvision = false; // run on the tvision desktop instead of the FTXUI screen
  cvc::ariadne::FtxuiBackend::RunOptions run;
  for (int i = 2; i < argc; ++i) {
    const char *a = argv[i];
    auto next = [&]() -> const char * {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "ari: %s needs a value\n", a);
        std::exit(2);
      }
      return argv[++i];
    };
    if (!std::strcmp(a, "--prefix"))
      prefix = next();
    else if (!std::strcmp(a, "--component-path"))
      componentPaths.push_back(next());
    else if (!std::strcmp(a, "--width"))
      width = std::atoi(next());
    else if (!std::strcmp(a, "--height"))
      height = std::atoi(next());
    else if (!std::strcmp(a, "--frames"))
      frames = std::atoi(next());
    else if (!std::strcmp(a, "--fps"))
      run.fps = std::atoi(next());
    else if (!std::strcmp(a, "--no-mouse"))
      run.mouse = false;
    else if (!std::strcmp(a, "--ansi"))
      ansi = true;
    else if (!std::strcmp(a, "--backend")) {
      const std::string b = next();
      if (b != "ftxui" && b != "tvision") {
        std::fprintf(stderr, "ari: --backend is ftxui or tvision, not '%s'\n", b.c_str());
        return 2;
      }
      tvision = b == "tvision";
    } else if (a[0] != '-' && doc.empty())
      doc = a;
    else {
      std::fprintf(stderr, "ari: unexpected argument '%s'\n", a);
      return usage(2);
    }
  }
  if (doc.empty())
    return usage(2);
  if (!cvc::ariadne::AppRuntime::have_yaml()) {
    std::fprintf(stderr, "ari: libcvc was built without yaml-cpp, so it cannot load a .ari\n");
    return 1;
  }
  if (cmd != "check" && !cvc::ariadne::FtxuiBackend::available()) {
    std::fprintf(stderr, "ari: libcvc was built without FTXUI (no terminal backend)\n");
    return 1;
  }
  tvision = tvision && cmd == "run"; // snapshot / check always lay out with FTXUI
  if (tvision && !cvc::ariadne::TvisionBackend::available()) {
    std::fprintf(stderr, "ari: libcvc was built without tvision (try --backend ftxui)\n");
    return 1;
  }

  cvc::app app;
  // Declared before the AppRuntime, so it outlives it. A TvisionBackend is an FtxuiBackend in
  // hosted mode; its run() is its own (see below).
  std::unique_ptr<cvc::ariadne::FtxuiBackend> backend =
      tvision ? std::make_unique<cvc::ariadne::TvisionBackend>()
              : std::make_unique<cvc::ariadne::FtxuiBackend>();
  cvc::ariadne::AppRuntime rt(app, prefix);
  rt.set_backend(*backend);
  for (const std::string &d : componentPaths)
    rt.add_component_path(d);
  rt.on("quit", [&] { rt.request_close(); });
  rt.register_verb("quit", [&] { rt.request_close(); }); // (quit), e.g. from an on_signal: program

  std::vector<std::string> warnings;
  try {
    warnings = rt.load(doc);
  } catch (const std::exception &e) {
    std::fprintf(stderr, "ari: %s\n", e.what());
    return 1;
  }
  for (const std::string &w : warnings)
    std::fprintf(stderr, "ari: warning: %s\n", w.c_str());
  if (cmd == "check")
    return 0;

  if (cmd == "snapshot") {
    std::string frame;
    for (int f = 0; f < std::max(1, frames); ++f) {
      rt.drain();
      frame = backend->render_to_string(rt.runtime(), std::max(10, width), height, ansi);
    }
    std::fputs(frame.c_str(), stdout);
    return 0;
  }

#ifndef _WIN32
  run.on_start = install_traps;
  run.on_stop = restore_traps;
  run.take_signal = take_trapped;
#endif
  cvc::ariadne::FtxuiBackend::RunExit how;
  if (tvision) {
    how = static_cast<cvc::ariadne::TvisionBackend &>(*backend).run(rt, run);
  } else {
    how = backend->run(rt, run);
  }
  // The fullscreen screen is gone; anything the read lane complained about is worth seeing now.
  for (const std::string &w : rt.take_warnings())
    std::fprintf(stderr, "ari: warning: %s\n", w.c_str());
#ifndef _WIN32
  if (how.from_os) // an OS signal the document didn't handle: report it the way a shell expects
    return 128 + signal_number(how.signal);
#endif
  (void)how;
  return 0;
}
