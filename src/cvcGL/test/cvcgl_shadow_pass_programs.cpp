// LowMemoryPolyDataMapper keeps a shader per render-pass set (its pass programs), so a shadow
// caster no longer rebuilds its shader source twice a frame.
//
// Under shadows every caster is drawn twice a frame: by the shadow baker into the maps and by the
// shadow pass into the view. VTK builds a mapper's shader for the passes on its actor and rebuilds
// it whenever they differ from the last draw's, so each of those draws rebuilt the source (a
// dozen-odd Substitute calls over ~16 KB) and looked the program up again (copy and MD5) -- about
// a fifth of a frame in the browser conference, where only the low-memory mapper draws. The mapper
// now keeps the shader of the pass set it leaves and puts it back when it returns, and VTK's own
// up-to-date test judges the one put back.
//
// Pins (the low-memory mapper forced, as on WebGL):
//   A. a caster moving every frame bakes every frame, and after the first frames no mapper builds
//      a shader: each draw puts back its pass set's; with pass programs off, each caster builds
//      two a frame (the cost this removes);
//   B. the frames are those of pass programs off (every draw building its shader afresh), each
//      compared frame a bake and a view, to two levels a channel;
//   C. the shadowing light changes (one spot stops casting and another starts, the light count
//      the same, which VTK's light mod does not see): the shadow pass's shader is rebuilt
//      (FilteredShadowMapPass's shader-stage MTime), and the frames match pass programs off;
//   D. edge visibility switched on (a property that changes the shader): frames match pass
//      programs off;
//   E. shadows off and on again (new passes, perhaps at old addresses): frames match;
//   F. a bake asks the driver whether its framebuffer is complete only for a light's map it has
//      not asked about: frames that bake issue no more glCheckFramebufferStatus than frames that
//      do not (on WebGL each one waits on the GPU process).
// Renders for real and skips where nothing rasterises (fatal under CVC_REQUIRE_RENDER=1). Desktop
// GL only (the GL counter swaps glad pointers).
#include "gl_call_counter.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/LowMemoryPolyDataMapper.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <vtkActor.h>
#include <vtkCollection.h>
#include <vtkLight.h>
#include <vtkLightCollection.h>
#include <vtkMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>

using cvc::gl::GeometryNode;
using cvc::gl::LowMemoryPolyDataMapper;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;

static int fails = 0;
static void chk(bool ok, const std::string &what) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok)
    ++fails;
}

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

// A flat ground cut into tiles (as cvcgl_shadow_casters has it).
static cvc::geometry tiledGround(double x0, double y0, double x1, double y1, double z,
                                 double step) {
  cvc::geometry g;
  const int nx = static_cast<int>(std::lround((x1 - x0) / step));
  const int ny = static_cast<int>(std::lround((y1 - y0) / step));
  for (int j = 0; j <= ny; ++j)
    for (int i = 0; i <= nx; ++i)
      g.points().push_back({x0 + (x1 - x0) * i / nx, y0 + (y1 - y0) * j / ny, z});
  auto at = [nx](int i, int j) { return static_cast<unsigned int>(j * (nx + 1) + i); };
  for (int j = 0; j < ny; ++j)
    for (int i = 0; i < nx; ++i) {
      g.tris().push_back({at(i, j), at(i + 1, j), at(i + 1, j + 1)});
      g.tris().push_back({at(i, j), at(i + 1, j + 1), at(i, j + 1)});
    }
  return g;
}

// Exposes the node's actor.
class Node : public GeometryNode {
public:
  Node(cvc::app &a, const std::string &path, const std::string &name)
      : GeometryNode(a, path, name) {}
  vtkActor *vtk() { return vtkActor::SafeDownCast(getProp()); }
};

static LowMemoryPolyDataMapper *mapperOf(const std::shared_ptr<Node> &n) {
  vtkActor *a = n->vtk();
  return a ? LowMemoryPolyDataMapper::SafeDownCast(a->GetMapper()) : nullptr;
}

static bool canRasterise(cvc::app &app) {
  SceneGraph sg(app, "ppcontrol");
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

// The largest per-channel difference between two frames, and how many bytes differ.
struct Diff {
  int max = 0;
  long count = 0;
};
static Diff diff(const std::vector<unsigned char> &a, const std::vector<unsigned char> &b) {
  Diff d;
  if (a.size() != b.size()) {
    d.max = 255;
    d.count = -1;
    return d;
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    const int e = std::abs(int(a[i]) - int(b[i]));
    d.max = std::max(d.max, e);
    d.count += e > 0 ? 1 : 0;
  }
  return d;
}

int main() {
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  // Before any node exists: newPolyDataMapper() picks the mapper at construction.
  cvc::gl::setLowMemoryMapperPolicy(app, cvc::gl::LowMemoryMapperPolicy::Force);
  if (!canRasterise(app)) {
    const char *require = std::getenv("CVC_REQUIRE_RENDER");
    if (require && *require && std::string(require) != "0") {
      std::printf("CVC_REQUIRE_RENDER is set, but this build did not rasterise\n");
      return 1;
    }
    std::printf("skipped: this build did not rasterise\n");
    return 0;
  }

  // A tiled ground, two boxes standing on it, lit from either side: the key spot casts, the
  // other light is a fill (cone 90: no shadow) until C swaps them. Seen from above at an angle,
  // so the shadows and the boxes' sides show.
  SceneGraph sg(app, "passprograms");
  sg.setDiagnosticChromeVisible(false);
  auto ground = sg.getGraphicsRoot()->addGraphicsChild<Node>("ground");
  ground->setGeometry(tiledGround(-12, -12, 12, 12, 0.0, 1.0));
  ground->setColor(0.9, 0.9, 0.9);
  auto mover = sg.getGraphicsRoot()->addGraphicsChild<Node>("mover");
  mover->setGeometry(box(-1, -1, 0, 1, 1, 3));
  mover->setColor(0.8, 0.5, 0.3);
  auto still = sg.getGraphicsRoot()->addGraphicsChild<Node>("still");
  still->setGeometry(box(3, 2, 0, 5, 4, 2));
  still->setColor(0.3, 0.6, 0.8);
  sg.processEvents();

  const int W = 128, H = 96;
  SceneRenderer view(sg, W, H, /*offscreen=*/true, "main");
  view.setBackground(0.1, 0.1, 0.3);
  view.setCamera(0, -16, 14, 0, 0, 0, 0, 0, 1, 40.0, 1.0, 100.0);
  sg.addSpotLight(-8, -6, 12, 0, 0, 0, 40.0, 1.0, 1.0, 1.0, 0.9);
  sg.addFillLight(8, -6, 12, 0, 0, 0, 1.0, 1.0, 1.0, 0.5);
  if (!sg.setShadowsEnabled(true)) {
    std::printf("skipped: shadows unavailable\n");
    return 0;
  }
  sg.setShadowResolution(512);
  sg.setShadowUpdateInterval(1);
  vtkLight *lights[2] = {nullptr, nullptr};
  {
    vtkLightCollection *lc = view.renderer()->GetLights();
    vtkCollectionSimpleIterator it;
    lc->InitTraversal(it);
    for (int i = 0; i < 2; ++i)
      lights[i] = lc->GetNextLight(it);
  }
  chk(lights[0] && lights[1] && lights[0]->GetConeAngle() < 90.0 &&
          lights[1]->GetConeAngle() >= 90.0,
      "the renderer's lights: the spot, then the fill");
  if (!lights[0] || !lights[1])
    return 1;

  const std::vector<std::shared_ptr<Node>> casters{ground, mover, still};
  std::vector<LowMemoryPolyDataMapper *> mappers;
  for (const auto &n : casters)
    mappers.push_back(mapperOf(n));
  const bool lowMem = std::all_of(mappers.begin(), mappers.end(), [](auto *m) { return m; });
  chk(lowMem, "every caster draws through LowMemoryPolyDataMapper (forced)");
  if (!lowMem)
    return 1;
  const auto reset = [&]() {
    for (auto *m : mappers)
      m->resetStats();
  };
  const auto sum = [&](std::uint64_t LowMemoryPolyDataMapper::Stats::*field) {
    std::uint64_t s = 0;
    for (auto *m : mappers)
      s += m->stats().*field;
    return s;
  };
  const auto frame = [&]() {
    sg.processEvents();
    view.render();
  };
  const auto capture = [&]() { return view.frameRGB(); };

  // ── A. builds per frame, with and without pass programs ─────────────────────
  std::printf("A. a caster moving every frame: shader builds\n");
  LowMemoryPolyDataMapper::setPassProgramsEnabled(app, true);
  double x = 0.0;
  const auto step = [&]() {
    x += 0.05;
    mover->setPosition(x, 0.0, 0.0);
    frame();
  };
  for (int i = 0; i < 4; ++i) // the first bake, the first view of each pass set
    step();
  reset();
  const std::uint64_t bakes0 = sg.shadowBakes();
  const int kFrames = 12;
  for (int i = 0; i < kFrames; ++i)
    step();
  const std::uint64_t bakes = sg.shadowBakes() - bakes0;
  chk(bakes == static_cast<std::uint64_t>(kFrames),
      std::to_string(bakes) + " bakes in " + std::to_string(kFrames) + " frames (want one each)");
  const std::uint64_t builds = sum(&LowMemoryPolyDataMapper::Stats::shaderBuilds);
  const std::uint64_t restored = sum(&LowMemoryPolyDataMapper::Stats::passProgramsRestored);
  chk(builds == 0, "pass programs on: " + std::to_string(builds) + " shader builds (want 0)");
  chk(restored == 2u * mappers.size() * kFrames,
      "each caster's draw puts back its pass set's shader: " + std::to_string(restored) +
          " (want " + std::to_string(2u * mappers.size() * kFrames) + ")");
  LowMemoryPolyDataMapper::setPassProgramsEnabled(app, false);
  step();
  reset();
  for (int i = 0; i < kFrames; ++i)
    step();
  const std::uint64_t buildsOff = sum(&LowMemoryPolyDataMapper::Stats::shaderBuilds);
  chk(buildsOff == 2u * mappers.size() * kFrames,
      "pass programs off: " + std::to_string(buildsOff) + " shader builds (two a caster a frame)");

  // Each comparison: the same scene state rendered with pass programs on and off, every frame a
  // bake and a view -- the alternation pass programs serve, and with them off each draw builds its
  // shader afresh (the reference). A light touched (its MTime, nothing it shows) makes the bake
  // draw; invalidateShadowBake() alone would not, with nothing changed.
  const auto settled = [&](bool on) {
    LowMemoryPolyDataMapper::setPassProgramsEnabled(app, on);
    const std::uint64_t b0 = sg.shadowBakes();
    for (int i = 0; i < 3; ++i) {
      lights[1]->Modified();
      frame();
    }
    if (sg.shadowBakes() - b0 != 3)
      chk(false, "every compared frame bakes: " + std::to_string(sg.shadowBakes() - b0) + " of 3");
    return capture();
  };
  // Within two levels a channel: byte for byte on NVIDIA and Mesa, but Apple's GL is not always
  // (cvcgl_shadow_casters). A stale shader is far out: C without the shader-stage MTime differs
  // by 145.
  constexpr int kTolerance = 2;
  const auto same = [&](const std::string &what) {
    const std::vector<unsigned char> on = settled(true), off = settled(false);
    const std::vector<unsigned char> on2 = settled(true);
    const Diff d = diff(on, off), d2 = diff(on2, off);
    chk(d.max <= kTolerance && d2.max <= kTolerance,
        what + ": pass programs on == off (max diff " + std::to_string(std::max(d.max, d2.max)) +
            ", " + std::to_string(d.count + d2.count) + " bytes)");
    return on;
  };

  // ── B. the frames ───────────────────────────────────────────────────────────
  std::printf("B. frames\n");
  const std::vector<unsigned char> before = same("the shadowed scene");

  // ── C. the shadowing light changes, the light count does not ────────────────
  std::printf("C. another light casts\n");
  // The same two VTK lights, the spot made a fill and the fill a spot: two lights still, one
  // casting, but the other one -- nothing VTK's light mod counts changes. (SceneGraph would make
  // new lights, and setLightCone clamps 90 to 89.5, which still casts.) Which lights cast is in the
  // shadow pass's shader: the view shaders kept from before must not be put back as they are.
  LowMemoryPolyDataMapper::setPassProgramsEnabled(app, true);
  settled(true);
  lights[0]->SetConeAngle(90.0);
  lights[1]->SetConeAngle(40.0);
  reset();
  frame();
  chk(sum(&LowMemoryPolyDataMapper::Stats::shaderBuilds) >= mappers.size(),
      "the new casting light rebuilds the view shaders");
  const std::vector<unsigned char> swapped = same("the other light casting");
  chk(diff(before, swapped).max > 0, "and the frame is not the old one (the shadows moved)");

  // ── D. a property that changes the shader ───────────────────────────────────
  std::printf("D. a property that changes the shader\n");
  LowMemoryPolyDataMapper::setPassProgramsEnabled(app, true);
  settled(true);
  vtkActor *stillActor = still->vtk();
  stillActor->GetProperty()->SetEdgeVisibility(true);
  stillActor->GetProperty()->SetEdgeColor(1.0, 0.0, 0.0);
  const std::vector<unsigned char> edges = same("edges on the still box");
  chk(diff(swapped, edges).max > 0, "and the edges show");

  // ── E. shadows off and on: new passes ───────────────────────────────────────
  std::printf("E. shadows off and on\n");
  LowMemoryPolyDataMapper::setPassProgramsEnabled(app, true);
  sg.setShadowsEnabled(false);
  frame();
  sg.setShadowsEnabled(true);
  same("after shadows off and on");

  // ── F. the bake's framebuffer status ────────────────────────────────────────
  std::printf("F. the bake's framebuffer\n");
  LowMemoryPolyDataMapper::setPassProgramsEnabled(app, true);
  cvcgl_test::glcallsInstall(); // the context exists, so glad is loaded
  settled(true);
  const auto checks = [&](bool bake, int n) {
    const std::uint64_t b0 = sg.shadowBakes();
    const double c0 = cvcgl_test::glcallsRead().count("CheckFramebufferStatus");
    for (int i = 0; i < n; ++i) {
      if (bake)
        lights[1]->Modified();
      frame();
    }
    return std::make_pair(sg.shadowBakes() - b0,
                          cvcgl_test::glcallsRead().count("CheckFramebufferStatus") - c0);
  };
  const auto baking = checks(true, 8), idle = checks(false, 8);
  chk(baking.first == 8 && idle.first == 0 && baking.second == idle.second,
      "8 frames that bake: " + std::to_string(baking.second) +
          " glCheckFramebufferStatus, 8 that do not: " + std::to_string(idle.second) + " (bakes " +
          std::to_string(baking.first) + ", " + std::to_string(idle.first) + ")");
  cvcgl_test::glcallsUninstall();

  LowMemoryPolyDataMapper::setPassProgramsEnabled(app, true);
  std::printf(fails ? "FAILED (%d)\n" : "OK\n", fails);
  return fails ? 1 : 0;
}
