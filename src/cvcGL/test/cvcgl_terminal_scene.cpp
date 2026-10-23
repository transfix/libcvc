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
#include <cvc/gl/SceneRenderer.h>
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
  cvc::gl::ariadne::TerminalScene scene(app, backend, /*supersample=*/2, "term");

  ari::Scene spec;
  ari::SceneNode box;
  box.id = "box";
  box.type = "geometry";
  box.source_primitive = "box";
  spec.nodes.push_back(box);
  std::vector<std::string> warns;
  scene.realize(spec, "term", &warns);
  chk(warns.empty(), "the box scene realizes without warnings");
  // A document under another state root than the scene graph's is warned about: node state would
  // land where its panels never look.
  {
    ari::FtxuiBackend other_backend(opts); // its own: ~TerminalScene clears its backend's backdrop
    cvc::gl::ariadne::TerminalScene other(app, other_backend, 2, "elsewhere");
    std::vector<std::string> w2;
    other.realize(spec, "term", &w2);
    chk(w2.size() == 1 && w2[0].find("'term'") != std::string::npos,
        "a mismatched state root is warned about");
  }

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

  // Pixel graphics: the scene renders at the cells' real pixels, and Kitty shows it through
  // placeholder cells for an image sent once.
  using Mode = ari::FtxuiBackend::Options::ImageColor;
  ari::TermGraphics kitty;
  kitty.kitty = true;
  kitty.cell_w = 9;
  kitty.cell_h = 18;
  for (const Mode mode : {Mode::Kitty, Mode::Sixel}) {
    const bool k = mode == Mode::Kitty;
    ari::FtxuiBackend::Options po;
    po.image_color = mode;
    ari::FtxuiBackend pbe(po);
    pbe.set_terminal_graphics(kitty);
    cvc::app papp;
    cvc::gl::ariadne::TerminalScene pscene(papp, pbe, /*supersample=*/2, "term");
    pscene.realize(spec, "term", &warns);
    ari::Runtime prt(papp, "");
    prt.set_backend(&pbe);
    prt.set_root(ari::group({ari::text("hud")}));
    pbe.set_screen_size(40, 12);
    pscene.tick();
    chk(pscene.view().frameWidth() == 40 * 9 && pscene.view().frameHeight() == 12 * 18,
        k ? "kitty: the scene renders at the cells' real pixels"
          : "sixel: the scene renders at the cells' real pixels");
    const std::string pf = pbe.render_to_string(prt, 40, 12);
    chk(pf.find("hud") != std::string::npos, "the widgets still draw over it");
    const std::string g = pbe.take_graphics();
    if (k) {
      chk(pf.find("\xf4\x8e\xbb\xae") != std::string::npos, "kitty: placeholder cells");
      chk(g.find("a=T") != std::string::npos &&
              g.find("s=360,v=216,c=40,r=12") != std::string::npos,
          "kitty: the frame is sent at its real size");
      // VTK's second render of a new view can still differ from its first (the clipping range
      // settles), so let one frame go by; after that the same view renders the same pixels.
      int resent = 0;
      for (int f = 0; f < 4; ++f) {
        pscene.tick();
        pbe.render_to_string(prt, 40, 12);
        // A transmit (a=T) is a send; freeing the image's second id once the frame is stable
        // (a=d) is not.
        const bool sent = pbe.take_graphics().find("a=T") != std::string::npos;
        resent += f > 0 && sent ? 1 : 0;
      }
      chk(resent == 0, "kitty: an unchanged frame is not sent again");
    } else {
      chk(g.find("\x1bP0;1;0q") != std::string::npos, "sixel: the frame goes out as a bitmap");
    }
  }

  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
}
