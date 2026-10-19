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
// decides for itself; three Ctrl-Cs within two seconds always quit.
// snapshot drains N frames (default 1) and prints the last one as text — for CI, logs, and docs.
// check loads the document and prints its warnings; it exits non-zero when the load fails.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/ftxui_backend.h>
#include <cvc/core/app.h>
#include <exception>
#include <string>
#include <vector>

namespace {

int usage(int code) {
  std::fprintf(code ? stderr : stdout,
               "usage:\n"
               "  ari run      <doc.ari> [--prefix P] [--fps N] [--no-mouse] "
               "[--component-path DIR]...\n"
               "  ari snapshot <doc.ari> [--prefix P] [--width W] [--height H] [--frames N] "
               "[--ansi]\n"
               "  ari check    <doc.ari> [--prefix P]\n");
  return code;
}

} // namespace

int main(int argc, char **argv) {
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
    else if (a[0] != '-' && doc.empty())
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

  cvc::app app;
  cvc::ariadne::FtxuiBackend backend; // declared before the AppRuntime, so it outlives it
  cvc::ariadne::AppRuntime rt(app, prefix);
  rt.set_backend(backend);
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
      frame = backend.render_to_string(rt.runtime(), std::max(10, width), height, ansi);
    }
    std::fputs(frame.c_str(), stdout);
    return 0;
  }

  backend.run(rt, run);
  // The fullscreen screen is gone; anything the read lane complained about is worth seeing now.
  for (const std::string &w : rt.take_warnings())
    std::fprintf(stderr, "ari: warning: %s\n", w.c_str());
  return 0;
}
