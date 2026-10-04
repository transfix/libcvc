// Changing the shadow update interval or resolution with shadows on must update
// the live baker in place, and dropping the shadow passes must free their GL
// objects first.
//
// Each setter mirrored its value into cvc::state ("<prefix>.shadows"), and the
// state's change handler called back into setShadowsEnabled(true), which built a
// whole new render-pass chain. The old vtkShadowMapBakerPass went with its FBO
// and shadow-map textures still allocated: its destructor only logs
// "FrameBufferObject/ShadowMaps/LightCameras should have been deleted in
// ReleaseGraphicsResources()" and frees nothing, so every change leaked them, and
// a baker pointer taken before the change was left on a pass nothing renders.
// setShadowsEnabled(false) dropped the chain the same way.
//
// VTK's baker sizes a shadow map only when it creates it, so a resolution change
// on a baker that has already baked must also re-create its maps.
//
// Pins:
//   A. interval and resolution changes keep the renderer's pass and its baker,
//      the baker carries the new values, and the maps are re-made at the new
//      resolution;
//   B. the same through a cvc::state write (the Ariadne / replicated path), and
//      setShadowsEnabled(true) while already on keeps the chain;
//   C. off/on cycles;
//   D. through all of the above: no VTK error, and no growth in live
//      GL textures, framebuffers or renderbuffers (glGen* minus glDelete*,
//      counted at the driver);
//   E. the scene re-opened in a new window: its shadows follow it (no baker is
//      left on the closed window's renderer), the new chain takes resolution
//      changes, and closing the old window logs nothing.
// Renders for real and skips where nothing rasterises (fatal under
// CVC_REQUIRE_RENDER=1). Desktop GL only (the GL counter swaps glad pointers).
#include "gl_call_counter.h"

#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/Settings.h>
#include <memory>
#include <string>
#include <vector>
#include <vtkCameraPass.h>
#include <vtkObjectFactory.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkOutputWindow.h>
#include <vtkRenderPassCollection.h>
#include <vtkRenderer.h>
#include <vtkSequencePass.h>
#include <vtkShadowMapBakerPass.h>
#include <vtkSmartPointer.h>
#include <vtkTextureObject.h>

using cvc::gl::GeometryNode;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;

static int fails = 0;
static void chk(bool ok, const std::string &what) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok)
    ++fails;
}

// VTK errors, counted and echoed. The baker's leak report is a vtkErrorMacro
// from its destructor, so it lands here, not as an exception. Warnings are only
// echoed: window creation warns on some backends (EGL device probing) whatever
// the shadow passes do.
class VtkMessages : public vtkOutputWindow {
public:
  static VtkMessages *New();
  vtkTypeMacro(VtkMessages, vtkOutputWindow);
  static int &count() {
    static int n = 0;
    return n;
  }
  void DisplayErrorText(const char *t) override {
    ++count();
    std::fprintf(stderr, "VTK ERROR: %s\n", t);
  }
  void DisplayWarningText(const char *t) override { std::fprintf(stderr, "VTK WARNING: %s\n", t); }
  void DisplayGenericWarningText(const char *t) override {
    std::fprintf(stderr, "VTK WARNING: %s\n", t);
  }
  void DisplayText(const char *t) override { std::fprintf(stderr, "%s", t); }
};
vtkStandardNewMacro(VtkMessages);

static cvc::geometry box(double x0, double y0, double z0, double x1, double y1, double z1) {
  cvc::geometry g;
  const double v[8][3] = {{x0, y0, z0}, {x1, y0, z0}, {x1, y1, z0}, {x0, y1, z0},
                          {x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1}};
  for (const auto &p : v)
    g.points().push_back({p[0], p[1], p[2]});
  const int f[12][3] = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
                        {1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}};
  for (const auto &t : f)
    g.tris().push_back({static_cast<unsigned int>(t[0]), static_cast<unsigned int>(t[1]),
                        static_cast<unsigned int>(t[2])});
  return g;
}

static vtkShadowMapBakerPass *findBaker(vtkRenderPass *p) {
  if (!p)
    return nullptr;
  if (auto *b = vtkShadowMapBakerPass::SafeDownCast(p))
    return b;
  if (auto *c = vtkCameraPass::SafeDownCast(p))
    return findBaker(c->GetDelegatePass());
  if (auto *s = vtkSequencePass::SafeDownCast(p))
    if (vtkRenderPassCollection *pc = s->GetPasses()) {
      pc->InitTraversal();
      while (vtkRenderPass *child = pc->GetNextRenderPass())
        if (auto *b = findBaker(child))
          return b;
    }
  return nullptr;
}

static bool canRasterise(cvc::app &app) {
  SceneGraph sg(app, "reusecontrol");
  sg.setDiagnosticChromeVisible(false);
  auto g = sg.getGraphicsRoot()->addGraphicsChild<GeometryNode>("control");
  g->setGeometry(box(-1, -1, -1, 1, 1, 1));
  SceneRenderer sr(sg, 32, 32, /*offscreen=*/true);
  sr.setBackground(0.0, 0.0, 0.0);
  sr.setCamera(4, 4, 4, 0, 0, 0, 0, 0, 1, 40.0, 0.5, 100.0);
  for (unsigned char c : sr.frameRGB())
    if (c > 20)
      return true;
  return false;
}

// Live GL objects of the kinds a shadow baker owns: its maps (textures), its FBO
// and the FBO's depth attachment (a renderbuffer). VTK creates and deletes them
// one per call.
struct Live {
  double textures = 0, framebuffers = 0, renderbuffers = 0;
  static Live now() {
    const cvcgl_test::GLCalls c = cvcgl_test::glcallsRead();
    Live l;
    l.textures = c.count("GenTextures") - c.count("DeleteTextures");
    l.framebuffers = c.count("GenFramebuffers") - c.count("DeleteFramebuffers");
    l.renderbuffers = c.count("GenRenderbuffers") - c.count("DeleteRenderbuffers");
    return l;
  }
  std::string str() const {
    return std::to_string(static_cast<long>(textures)) + " textures, " +
           std::to_string(static_cast<long>(framebuffers)) + " framebuffers, " +
           std::to_string(static_cast<long>(renderbuffers)) + " renderbuffers";
  }
  bool operator==(const Live &o) const {
    return textures == o.textures && framebuffers == o.framebuffers &&
           renderbuffers == o.renderbuffers;
  }
};

static std::string glRendererName(vtkRenderWindow *w) {
  auto *gl = vtkOpenGLRenderWindow::SafeDownCast(w);
  const char *caps = gl ? gl->ReportCapabilities() : nullptr;
  const std::string all = caps ? caps : "";
  const std::string key = "renderer string:";
  const std::size_t at = all.find(key);
  if (at == std::string::npos)
    return "unknown";
  std::size_t b = all.find_first_not_of(' ', at + key.size());
  const std::size_t e = all.find('\n', at);
  if (b == std::string::npos || b > e)
    b = e;
  return all.substr(b, e == std::string::npos ? std::string::npos : e - b);
}

// The width of the baker's first shadow map, or 0 if it has none.
static int mapSize(vtkShadowMapBakerPass *b) {
  std::vector<vtkSmartPointer<vtkTextureObject>> *maps = b ? b->GetShadowMaps() : nullptr;
  if (!maps || maps->empty() || !(*maps)[0])
    return 0;
  return static_cast<int>((*maps)[0]->GetWidth());
}

int main() {
  vtkOutputWindow::SetInstance(vtkSmartPointer<VtkMessages>::New());
  cvc::app app;
  app.properties("system.log_verbosity", "0");

  if (!canRasterise(app)) {
    const char *require = std::getenv("CVC_REQUIRE_RENDER");
    if (require && *require && std::string(require) != "0") {
      std::printf("CVC_REQUIRE_RENDER is set, but this build did not rasterise\n");
      return 1;
    }
    std::printf("skipped: this build did not rasterise\n");
    return 0;
  }

  {
    SceneGraph sg(app, "reuse");
    sg.setDiagnosticChromeVisible(false);
    auto ground = sg.getGraphicsRoot()->addGraphicsChild<GeometryNode>("ground");
    ground->setGeometry(box(-20, -20, -1, 20, 20, 0));
    for (int i = 0; i < 3; ++i)
      sg.addGraphics("box" + std::to_string(i), box(-8 + 6 * i, -2, 0, -6 + 6 * i, 2, 4));
    sg.processEvents();

    auto open = [&]() {
      auto v = std::make_unique<SceneRenderer>(sg, 128, 96, /*offscreen=*/true, "main");
      v->setCamera(0, -30, 30, 0, 0, 0, 0, 0, 1, 40.0, 1.0, 200.0);
      return v;
    };
    std::unique_ptr<SceneRenderer> view = open();
    sg.addDirectionalLight(90.0, 45.0);
    if (!sg.setShadowsEnabled(true)) {
      std::printf("skipped: shadows unavailable\n");
      return 0;
    }
    auto frames = [&](int n) {
      for (int i = 0; i < n; ++i) {
        sg.processEvents();
        view->render();
      }
    };
    frames(2);
    cvcgl_test::glcallsInstall(); // the context exists now, so glad is loaded
    frames(2);

    vtkRenderer *ren = view->renderer();
    std::printf("  GL renderer: %s\n", glRendererName(ren->GetRenderWindow()).c_str());
    // Held, so the pointer stays valid to compare even if the chain is rebuilt.
    vtkSmartPointer<vtkRenderPass> pass = ren->GetPass();
    vtkSmartPointer<vtkShadowMapBakerPass> baker = findBaker(pass);
    chk(baker != nullptr, "found the scene's shadow baker");
    if (!baker)
      return 1;
    chk(mapSize(baker) == sg.shadowResolution(),
        "baked at the default resolution (" + std::to_string(mapSize(baker)) + ")");
    const Live base = Live::now();
    const int messages0 = VtkMessages::count();
    std::printf("  baseline: %s\n", base.str().c_str());
    auto sameChain = [&]() { return ren->GetPass() == pass && findBaker(ren->GetPass()) == baker; };

    std::printf("A. interval and resolution, through the setters\n");
    bool kept = true;
    for (int k : {2, 5, 1, 3, 24, 1}) {
      sg.setShadowUpdateInterval(k);
      frames(3);
      kept = kept && sameChain();
    }
    chk(kept, "6 interval changes kept the pass chain and its baker");
    kept = true;
    bool applied = true, resized = true;
    for (int r : {512, 2048, 256, 768, 1024, 640}) {
      sg.setShadowResolution(r);
      frames(2);
      kept = kept && sameChain();
      applied = applied && static_cast<int>(baker->GetResolution()) == r;
      resized = resized && mapSize(baker) == r;
      if (mapSize(baker) != r)
        std::printf("    resolution %d: map is %d\n", r, mapSize(baker));
    }
    chk(kept, "6 resolution changes kept the pass chain and its baker");
    chk(applied, "the baker carries each new resolution");
    chk(resized, "and its shadow map is re-made at that size");

    std::printf("B. through cvc::state, and enabling while on\n");
    const std::string path = cvc::gl::ShadowSettings::sceneStatePath("reuse");
    cvc::state::instance(app)(path + ".resolution").value(1536);
    cvc::state::instance(app)(path + ".interval").value(4);
    frames(2);
    chk(sameChain(), "state writes kept the pass chain and its baker");
    chk(mapSize(baker) == 1536 && sg.shadowUpdateInterval() == 4,
        "and applied (map " + std::to_string(mapSize(baker)) + ", interval " +
            std::to_string(sg.shadowUpdateInterval()) + ")");
    chk(sg.setShadowsEnabled(true) && sameChain(), "setShadowsEnabled(true) while on keeps it");
    cvc::state::instance(app)(path + ".enabled").value(1);
    frames(1);
    chk(sameChain(), "and so does enabled=1 written to state");

    sg.setShadowResolution(1024);
    sg.setShadowUpdateInterval(1);
    frames(2);
    const Live afterA = Live::now();
    chk(afterA == base, "no GL objects leaked by A-B: " + afterA.str());
    chk(VtkMessages::count() == messages0,
        "no VTK errors in A-B (" + std::to_string(VtkMessages::count() - messages0) + ")");

    std::printf("C. off/on cycles\n");
    baker = nullptr; // the next enable builds a new one; don't keep this one alive
    pass = nullptr;
    for (int i = 0; i < 4; ++i) {
      chk(sg.setShadowsEnabled(false) && ren->GetPass() == nullptr, "off");
      frames(2);
      chk(sg.setShadowsEnabled(true) && findBaker(ren->GetPass()) != nullptr, "on");
      frames(2);
    }
    const Live afterC = Live::now();
    chk(afterC == base, "no GL objects leaked by C: " + afterC.str());
    chk(VtkMessages::count() == messages0,
        "no VTK errors in C (" + std::to_string(VtkMessages::count() - messages0) + ")");
    chk(mapSize(findBaker(ren->GetPass())) == 1024, "re-enabled at the set resolution");

    std::printf("E. a new window\n");
    view.reset(); // detaches the scene, then finalizes the window
    chk(sg.shadowsEnabled(), "shadows stay on with no window");
    view = open();
    ren = view->renderer();
    frames(3);
    vtkShadowMapBakerPass *moved = findBaker(ren->GetPass());
    chk(moved != nullptr && mapSize(moved) == 1024, "they follow the scene into the new window");
    chk(sg.setShadowsEnabled(true) && findBaker(ren->GetPass()) == moved,
        "enabling there keeps that chain");
    sg.setShadowResolution(512);
    frames(2);
    chk(findBaker(ren->GetPass()) == moved && mapSize(moved) == 512,
        "and a resolution change re-makes its map (" + std::to_string(mapSize(moved)) + ")");
    chk(VtkMessages::count() == messages0,
        "no VTK errors in E (" + std::to_string(VtkMessages::count() - messages0) + ")");
  }
  chk(VtkMessages::count() == 0,
      "no VTK errors at teardown either (" + std::to_string(VtkMessages::count()) + " in all)");

  std::printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "PASSED", fails, fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
