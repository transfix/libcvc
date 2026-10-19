// cvcGL's terminal extension (cvc::gl::ariadne::TerminalScene, roadmap §16.6): a scene rendered
// offscreen reaches the terminal backend as its background, under the widgets, and a background
// drag turns the camera. Runs offscreen like the other cvcGL GL tests; no terminal needed (the
// backend renders to a string).
#include <cstdio>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/ftxui_backend.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/gl/ariadne/TerminalScene.h>
#include <set>
#include <string>
#include <vector>

namespace ari = cvc::ariadne;

namespace {

int failures = 0;

void chk(bool ok, const char *what) {
  std::printf("%s %s\n", ok ? "ok  " : "FAIL", what);
  if (!ok)
    ++failures;
}

// The distinct SGR sequences in an ANSI frame. FTXUI emits colours at whatever depth it detects
// (24- bit on a truecolor terminal, a palette with no TTY), so count sequences, not "38;2;" forms.
std::set<std::string> sgr_sequences(const std::string &ansi) {
  std::set<std::string> out;
  for (std::size_t i = 0; (i = ansi.find("\x1b[", i)) != std::string::npos; i += 2)
    out.insert(ansi.substr(i, ansi.find('m', i) - i));
  return out;
}

} // namespace

int main() {
  if (!ari::FtxuiBackend::available()) {
    std::printf("skip: libcvc built without FTXUI\n");
    return 0;
  }
  cvc::app app;
  ari::FtxuiBackend::Options opts;
  opts.image_color = ari::FtxuiBackend::Options::ImageColor::Color;
  ari::FtxuiBackend backend(opts);
  cvc::gl::ariadne::TerminalScene scene(app, backend);

  ari::Scene spec;
  ari::SceneNode box;
  box.id = "box";
  box.type = "geometry";
  box.source_primitive = "box";
  spec.nodes.push_back(box);
  std::vector<std::string> warns;
  scene.realize(spec, "term", &warns);
  chk(warns.empty(), "the box scene realizes without warnings");

  ari::Runtime rt(app, "");
  rt.set_backend(&backend);
  rt.set_root(ari::group({ari::text("hud")}));
  backend.set_screen_size(40, 12);
  scene.tick(); // render offscreen at 80 x 48 (40 x 12 cells, two pixels a cell, 2x supersample)

  const std::string plain = backend.render_to_string(rt, 40, 12);
  chk(plain.find("hud") != std::string::npos, "the widgets draw over the scene");
  chk(plain.find("\xe2\x96\x80") != std::string::npos, "the scene fills the screen as half-blocks");
  const std::string first = backend.render_to_string(rt, 40, 12, /*ansi=*/true);
  chk(sgr_sequences(first).size() >= 4, "the frame is a rendered image, not a flat fill");

  // A background drag orbits the camera, so the next frame differs.
  chk(scene.handle_input(ari::TermEvent::mouse_event(ari::TermEvent::Mouse::Press, 20, 6)),
      "a background press starts a camera drag");
  chk(scene.handle_input(ari::TermEvent::mouse_event(ari::TermEvent::Mouse::Move, 30, 6)),
      "a drag move turns the camera");
  chk(scene.handle_input(ari::TermEvent::mouse_event(ari::TermEvent::Mouse::Release, 30, 6)),
      "the release ends it");
  chk(!scene.handle_input(ari::TermEvent::key_event("A")), "keys are left for the document");
  scene.tick();
  const std::string second = backend.render_to_string(rt, 40, 12, /*ansi=*/true);
  chk(second != first, "the orbited frame differs");

  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
}
