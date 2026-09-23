// SceneRenderer is now a thin FACADE over ViewportManager (a single full-screen
// primary in HostStyle input mode). This pins the two things the facade could
// get wrong that no other test covers:
//
//   * STATE-FOOTPRINT PARITY — the classic single-view renderer wrote NO
//     ".viewers.<name>.camera/.layout" state. A naive facade over the manager's
//     primary would write both (an eager CameraController + a ViewportLayout).
//     The HostStyle "bare" primary must write neither at construction; only a
//     consumer's own CameraController creates camera state. Proven differentially
//     against a Router-mode ViewportManager, which DOES seed both.
//   * The HostStyle host-input contract — a CameraController / FpsHud /
//     ScreenTextHud built on the facade still construct against it and render.
//
// Behaviour (render / setCamera-persists / pickWorld / close) is covered against
// this same facade by cvcgl_renderer.cpp; the render + camera-persistence +
// lifecycle checks here are a fast belt-and-suspenders at the facade seam.
//
// Offscreen throughout, so this runs headless in CI. These tests assert through
// a check() counter (not assert()), so they run under Release too.
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/state_object.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/FpsHud.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ScreenTextHud.h>
#include <cvc/gl/Settings.h>
#include <cvc/gl/Viewport.h>
#include <cvc/gl/ViewportManager.h>
#include <stdexcept>
#include <string>
#include <vector>
#include <vtkCamera.h>
#include <vtkRenderer.h>

using cvc::gl::CameraController;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
using cvc::gl::ViewportLayout;
using cvc::gl::ViewportManager;

static int fails = 0;
static void check(bool ok, const std::string &w) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", w.c_str());
  if (!ok)
    ++fails;
}

// A passive probe rooted at an arbitrary state path. cvc::state auto-creates a
// missing child and reads it back as "", so "was this key seeded?" is exactly
// "is the read non-empty?" — an unmanaged viewport that never wrote it reads "".
class Peer : public cvc::state_object<Peer> {
public:
  Peer(cvc::app &c, const std::string &p) : cvc::state_object<Peer>(c, p) {
    this->setInstanceThreading(false); // a probe must not spawn handler threads
  }
  std::string rd(const std::string &k) { return getState(k).value(); }
};

static void addQuad(SceneGraph &sg, const std::string &name, double h, double z) {
  cvc::geometry g;
  const double xs[4] = {-h, h, h, -h};
  const double ys[4] = {-h, -h, h, h};
  for (int i = 0; i < 4; ++i) {
    cvc::geometry::point_t p;
    p[0] = xs[i];
    p[1] = ys[i];
    p[2] = z;
    g.points().push_back(p);
  }
  const int idx[2][3] = {{0, 1, 2}, {0, 2, 3}};
  for (auto &tri : idx) {
    cvc::geometry::tri_t t;
    t[0] = tri[0];
    t[1] = tri[1];
    t[2] = tri[2];
    g.tris().push_back(t);
  }
  sg.addGraphics(name, g);
}

int main() {
  cvc::app app;
  app.properties("system.log_verbosity", "0");

  std::printf("== A. state-footprint parity (facade writes no camera/layout state) ==\n");
  {
    SceneGraph sg(app, "facade");
    SceneRenderer view(sg, 200, 150, true, "main"); // HostStyle facade
    Peer cam(app, CameraController::viewerStatePath(sg.getStatePrefix(), "main"));
    Peer lay(app, ViewportLayout::viewerStatePath(sg.getStatePrefix(), "main"));
    check(cam.rd("mode").empty(), "no .viewers.main.camera state at construction");
    check(lay.rd("visible").empty(), "no .viewers.main.layout state at construction");

    // The ONLY thing that legitimately creates camera state is a consumer
    // building its own controller — exactly like the classic renderer.
    CameraController cc(view);
    check(!cam.rd("mode").empty(), "a CameraController(view) seeds the camera state");
    check(lay.rd("visible").empty(), "still no layout state after the controller attaches");
  }
  {
    // Differential: the Router primary DOES seed both, so the empties above are a
    // real difference, not a mistyped key path.
    SceneGraph sg2(app, "router");
    ViewportManager vm(sg2, 64, 48, true, "main"); // Router (default)
    (void)vm;
    Peer cam(app, CameraController::viewerStatePath(sg2.getStatePrefix(), "main"));
    Peer lay(app, ViewportLayout::viewerStatePath(sg2.getStatePrefix(), "main"));
    check(!cam.rd("mode").empty(), "Router primary DOES seed camera state (differential)");
    check(!lay.rd("visible").empty(), "Router primary DOES seed layout state (differential)");
  }

  std::printf("== B. render + explicit-camera persistence + pick, through the facade ==\n");
  {
    SceneGraph sg(app, "behav");
    // A small quad far below the camera: from eye z=100 / 30-deg FOV the view
    // spans ~[-36,36] x [-27,27] at z=0, so a [-2,2] quad sits in the middle few
    // pixels — the center pixel hits it and any corner pixel is well clear of it.
    addQuad(sg, "floor", 2.0, 0.0);
    SceneRenderer view(sg, 128, 96, true, "main");
    view.setBackground(0.05, 0.05, 0.05);
    view.setCamera(0, 0, 100, 0, 0, 0, 0, 1, 0);

    std::vector<unsigned char> f = view.frameRGB();
    check(static_cast<int>(f.size()) == 128 * 96 * 3, "frameRGB size = W*H*3");
    bool nonblank = false;
    for (unsigned char b : f)
      if (b > 20) {
        nonblank = true;
        break;
      }
    check(nonblank, "frame is non-blank with a lit quad in view");

    double eyeBefore[3];
    view.renderer()->GetActiveCamera()->GetPosition(eyeBefore);
    view.render();
    view.render();
    double eyeAfter[3];
    view.renderer()->GetActiveCamera()->GetPosition(eyeAfter);
    check(std::fabs(eyeBefore[0] - eyeAfter[0]) < 1e-9 &&
              std::fabs(eyeBefore[1] - eyeAfter[1]) < 1e-9 &&
              std::fabs(eyeBefore[2] - eyeAfter[2]) < 1e-9,
          "explicit camera survives successive renders");

    double world[3];
    check(view.pickWorld(64, 48, world), "pickWorld hits the quad at the center pixel");
    double miss[3];
    check(!view.pickWorld(2, 2, miss), "pickWorld misses over empty space (corner)");
  }

  std::printf("== C. companions construct against the facade and render ==\n");
  {
    SceneGraph sg(app, "chrome");
    addQuad(sg, "floor", 2.0, 0.0);
    SceneRenderer view(sg, 96, 72, true, "main");
    view.setCamera(0, 0, 20, 0, 0, 0, 0, 1, 0);
    CameraController cam(view); // reads scene()/name()/renderer()
    cam.update(0.016);
    cvc::gl::FpsHud fps(view);               // adds an actor to the primary renderer
    cvc::gl::ScreenTextHud txt(view, "cap"); // 2-D overlay on the primary
    txt.setText("facade");
    view.render();
    std::vector<unsigned char> f = view.frameRGB();
    check(static_cast<int>(f.size()) == 96 * 72 * 3,
          "CameraController + FpsHud + ScreenTextHud render at requested size");
  }

  std::printf("== E. viewportManager() exposes the owned manager (grow PiP from a facade) ==\n");
  {
    SceneGraph sg(app, "grow");
    SceneGraph inset(app, "grow_inset");
    addQuad(inset, "inset_floor", 2.0, 0.0);
    SceneRenderer view(sg, 128, 96, true, "main");
    ViewportManager &vm = view.viewportManager();
    // The facade's renderer IS the manager's primary viewport renderer.
    check(vm.primary().renderer() == view.renderer(),
          "viewportManager().primary() is the facade's own renderer");
    check(vm.viewportNames().size() == 1, "just the primary before growing");
    const double region[4] = {0.6, 0.05, 0.97, 0.42};
    vm.addSceneViewport("inset", inset, region, 1);
    vm.addMirrorViewport("mini", "main", region, 2);
    check(vm.viewportNames().size() == 3, "grew a scene inset + a mirror over the same window");
    view.render(); // composites all three through the facade's window
    std::vector<unsigned char> f = view.frameRGB();
    check(static_cast<int>(f.size()) == 128 * 96 * 3,
          "facade frame still whole-window after growing PiP");
  }

  std::printf("== D. lifecycle: idempotent close, safe accessors, post-close throws ==\n");
  {
    SceneGraph sg(app, "life");
    SceneRenderer view(sg, 64, 48, true, "main");
    view.render();
    check(!view.isClosed(), "open before close");
    view.close();
    check(view.isClosed(), "isClosed() flips after close");
    view.close(); // idempotent
    check(view.isClosed(), "close() is idempotent");
    check(view.name() == "main", "name() is valid after close (cached)");
    check(&view.scene() == &sg, "scene() is valid after close (cached)");
    bool threw = false;
    try {
      view.render();
    } catch (const std::exception &) {
      threw = true;
    }
    check(threw, "render() after close throws");
    threw = false;
    try {
      (void)view.renderer();
    } catch (const std::exception &) {
      threw = true;
    }
    check(threw, "renderer() after close throws");
  }

  std::printf("\n%s (%d failure%s)\n", fails ? "FACADE TEST FAILED" : "FACADE TEST PASSED", fails,
              fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
