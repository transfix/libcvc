// Caster-aware shadow baking: GraphicsNode::setCastsShadow(false) takes a node
// out of the shadow maps, and -- the point -- out of the decision to re-bake.
//
// vtkShadowMapBakerPass re-renders every caster into its depth maps whenever ANY
// view prop's MTime moved, hidden or not, casting or not. In the demo3 wasm
// profile that was a 30-37 ms bake on every due frame, triggered by vehicle
// poses, a fog texture repaint and overlay restyles -- none of which change a
// building's shadow. The scene's baker now re-bakes only for a light, or a prop
// that casts, that changed (or joined or left), and draws casters only.
//
// Pins:
//   A. (headless) the flag: default true, inherited by descendants -- including
//      ones added later -- and readable from the bare vtkProp;
//   B. moving / restyling a non-caster never bakes; moving a caster does;
//   C. a node starting or stopping casting bakes; a light change bakes;
//   D. on screen: a non-caster casts no shadow (same pixels as no object at
//      all -- byte for byte, except on Apple's renderer, within 1 luma), a
//      caster does, and only inside its footprint; judged against reference
//      renders of the same run, on frames that show the whole ground;
//   E. the update interval still strides bakes of a moving caster;
//   F. a caster deformed in place (updateVertices / updateNormals /
//      updateColors) bakes, a non-caster does not; a RibbonNode does not cast
//      by default; a casting StreamingGeometryNode's streamed write bakes, and
//      so does a draw-range or relayout change alone; a DrapedLinkNode does not
//      cast by default, and a casting one moved or reshaped by uniforms
//      (endpoints, width, heights) re-bakes while uploading nothing.
// B-F render for real and skip where nothing rasterises (fatal under
// CVC_REQUIRE_RENDER=1).
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/DrapedLinkNode.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/HeightFieldTexture.h>
#include <cvc/gl/NullGraphicNode.h>
#include <cvc/gl/RibbonNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/StreamingGeometryNode.h>
#include <memory>
#include <string>
#include <vector>
#include <vtkActor.h>
#include <vtkCameraPass.h>
#include <vtkMapper.h>
#include <vtkNew.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderPassCollection.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSequencePass.h>
#include <vtkShadowMapBakerPass.h>

using cvc::gl::GeometryNode;
using cvc::gl::GraphicsNode;
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

// A flat, upward-facing ground at height z, cut into step x step tiles. Not two
// big triangles: Apple's software renderer (GitHub's arm64 macOS runners)
// intermittently drops triangles that are clipped far outside the frustum, and
// two triangles spanning a 120-unit ground are exactly that, both in the 16-unit
// view and in the shadow bake, whose light frustum is fitted to the scene bounds
// so the ground's edges sit on it. Lost from the frame, or from the maps (which
// VTK then reads as total shadow), the ground read luma 0.0 in both halves.
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

// A StreamingGeometryNode whose relayout (protected: subclasses own their
// topology) the test can call.
class StreamNode : public cvc::gl::StreamingGeometryNode {
public:
  using Layout = cvc::gl::StreamingLayout;
  StreamNode(cvc::app &a, const std::string &path, const std::string &name, const Layout &l)
      : StreamingGeometryNode(a, path, name, l) {}
  void relayoutTo(const Layout &l) { relayout(l); }
};

// Exposes the node's actor.
class Node : public GeometryNode {
public:
  Node(cvc::app &a, const std::string &path, const std::string &name)
      : GeometryNode(a, path, name) {}
  vtkProp *actor() { return getProp(); }
};

static void flags(cvc::app &app) {
  std::printf("A. the flag\n");
  SceneGraph sg(app, "castflags");
  auto group = sg.getGraphicsRoot()->addGraphicsChild<cvc::gl::NullGraphicNode>("group");
  auto part = group->addGraphicsChild<Node>("part");
  part->setGeometry(box(0, 0, 0, 1, 1, 1));
  chk(part->castsShadow() && GraphicsNode::propCastsShadow(part->actor()), "casts by default");
  group->setCastsShadow(false);
  chk(!part->castsShadow() && !GraphicsNode::propCastsShadow(part->actor()),
      "a group's setCastsShadow(false) reaches its parts, and their props");
  auto late = group->addGraphicsChild<Node>("late");
  late->setGeometry(box(0, 0, 0, 1, 1, 1));
  chk(!late->castsShadow() && !GraphicsNode::propCastsShadow(late->actor()),
      "a part added later inherits it");
  group->setCastsShadow(true);
  chk(part->castsShadow() && GraphicsNode::propCastsShadow(late->actor()), "and back on");
  vtkNew<vtkActor> raw;
  chk(GraphicsNode::propCastsShadow(raw), "a raw prop casts unless marked");
  GraphicsNode::setPropCastsShadow(raw, false);
  chk(!GraphicsNode::propCastsShadow(raw), "setPropCastsShadow marks it");
  GraphicsNode::setPropCastsShadow(raw, true);
  chk(GraphicsNode::propCastsShadow(raw), "and clears it");
  chk(GraphicsNode::propCastsShadow(nullptr), "null is harmless");
}

// Mean luma over columns [x0, x1) of the middle half of the rows; -1 for a
// frame that is not w x h.
static double meanLuma(const std::vector<unsigned char> &rgb, int w, int h, int x0, int x1) {
  if (rgb.size() != static_cast<std::size_t>(w) * h * 3)
    return -1.0;
  double sum = 0.0;
  long n = 0;
  for (int y = h / 4; y < 3 * h / 4; ++y)
    for (int x = x0; x < x1; ++x) {
      const std::size_t o = (static_cast<std::size_t>(y) * w + x) * 3;
      sum += 0.299 * rgb[o] + 0.587 * rgb[o + 1] + 0.114 * rgb[o + 2];
      ++n;
    }
  return n ? sum / n : 0.0;
}

// Whether two w x h frames agree byte for byte over the region meanLuma reads.
static bool sameRegion(const std::vector<unsigned char> &a, const std::vector<unsigned char> &b,
                       int w, int h, int x0, int x1) {
  if (a.size() != static_cast<std::size_t>(w) * h * 3 || b.size() != a.size())
    return false;
  for (int y = h / 4; y < 3 * h / 4; ++y)
    for (int x = x0; x < x1; ++x)
      for (int c = 0; c < 3; ++c) {
        const std::size_t o = (static_cast<std::size_t>(y) * w + x) * 3 + c;
        if (a[o] != b[o])
          return false;
      }
  return true;
}

// Pixels showing the background. It is pure blue; the white ground is grey at
// any light level, black included, so these are pixels where the ground is
// MISSING -- not merely in shadow.
static long backgroundPixels(const std::vector<unsigned char> &rgb) {
  long n = 0;
  for (std::size_t o = 0; o + 2 < rgb.size(); o += 3)
    if (rgb[o + 2] > rgb[o] + 64 && rgb[o + 2] > rgb[o + 1] + 64)
      ++n;
  return n;
}

// The "OpenGL renderer string" line of the window's capability report.
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

static bool canRasterise(cvc::app &app) {
  SceneGraph sg(app, "castcontrol");
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

static void rendered(cvc::app &app) {
  if (!canRasterise(app)) {
    const char *require = std::getenv("CVC_REQUIRE_RENDER");
    if (require && *require && std::string(require) != "0") {
      std::printf("CVC_REQUIRE_RENDER is set, but this build did not rasterise\n");
      chk(false, "rasterise");
    } else {
      std::printf("skipped B-E: this build did not rasterise\n");
    }
    return;
  }

  // A white ground seen from straight above; a slab high up and OUT of view to
  // the +x side; a sun from +x at 45 degrees throws the slab's shadow onto the
  // right half of the view. A building (caster) and a vehicle (non-caster)
  // stand elsewhere, out of view too. The ground is tiled (tiledGround) and the
  // background blue, so a frame that lost the ground cannot pass for shadow.
  SceneGraph sg(app, "casters");
  sg.setDiagnosticChromeVisible(false);
  auto ground = sg.getGraphicsRoot()->addGraphicsChild<GeometryNode>("ground");
  ground->setGeometry(tiledGround(-60, -60, 60, 60, 0.0, 2.0));
  ground->setColor(1.0, 1.0, 1.0);
  auto slab = sg.getGraphicsRoot()->addGraphicsChild<GeometryNode>("slab");
  slab->setGeometry(box(20, -6, 20, 30, 6, 21));
  auto building = sg.addGraphics("building", box(-40, 30, 0, -30, 40, 15));
  auto vehicle = sg.getGraphicsRoot()->addGraphicsChild<GeometryNode>("vehicle");
  vehicle->setGeometry(box(-45, -45, 0, -40, -40, 3));
  vehicle->setCastsShadow(false);
  sg.processEvents();

  const int W = 96, H = 96;
  SceneRenderer view(sg, W, H, /*offscreen=*/true, "main");
  // Red 0, as before: VTK clears the shadow maps to the background, and the
  // maps are single-channel (red).
  view.setBackground(0.0, 0.0, 1.0);
  view.setCamera(0, 0, 30, 0, 0, 0, 0, 1, 0, 30.0, 1.0, 200.0);
  const int sun = sg.addDirectionalLight(90.0, 45.0); // from +x
  if (!sg.setShadowsEnabled(true)) {
    std::printf("skipped B-E: shadows unavailable\n");
    return;
  }
  sg.setShadowResolution(1024);
  sg.setShadowUpdateInterval(1); // let the baker decide every frame
  vtkShadowMapBakerPass *baker = findBaker(view.renderer()->GetPass());
  chk(baker != nullptr, "found the scene's shadow baker");
  if (!baker)
    return;
  auto frame = [&]() {
    sg.processEvents();
    view.render();
    return baker->GetNeedUpdate();
  };
  for (int i = 0; i < 4; ++i) // settle the first bakes
    frame();

  std::printf("B. non-casters never bake; casters do\n");
  int bakes = 0;
  for (int i = 0; i < 10; ++i) {
    vehicle->setPosition(0.5 * i, 0.0, 0.0);
    bakes += frame() ? 1 : 0;
  }
  chk(bakes == 0, "10 frames moving a non-caster: " + std::to_string(bakes) + " bakes (want 0)");
  vehicle->setColor(1.0, 0.2, 0.2);
  vehicle->setOpacity(0.9);
  chk(!frame(), "restyling it: no bake");
  ground->setColor(1.0, 1.0, 0.99); // a caster's property
  chk(frame(), "restyling a caster bakes");
  building->setPosition(0.0, 1.0, 0.0);
  chk(frame(), "moving a caster bakes");
  chk(!frame(), "and the frame after does not");

  std::printf("C. joining / leaving the casters, and lights\n");
  vehicle->setCastsShadow(true);
  chk(frame(), "a node starting to cast bakes");
  vehicle->setPosition(1.0, 1.0, 0.0);
  chk(frame(), "and now its moves bake");
  vehicle->setCastsShadow(false);
  chk(frame(), "stopping casting bakes (it leaves the maps)");
  vehicle->setPosition(2.0, 1.0, 0.0);
  chk(!frame(), "and its moves no longer do");
  sg.setLightDirection(sun, 90.0, 46.0);
  chk(frame(), "a light change bakes");
  sg.setLightDirection(sun, 90.0, 45.0);
  frame();

  // Frames until one does not bake: the maps then match the scene.
  auto settle = [&]() {
    for (int i = 0; i < 6; ++i)
      if (!frame())
        return true;
    return false;
  };

  std::printf("D. on screen (%s)\n", glRendererName(view.renderer()->GetRenderWindow()).c_str());
  // Judged against reference renders of this same run, not absolute levels:
  // the right half lies in the slab's shadow footprint, the left half outside
  // it and is each frame's lit reference. Casting must darken the footprint
  // well below it, not casting must leave the footprint lit like it, `absent`
  // must match `notCasting`, and casting must leave the left half alone.
  //
  // First, every capture must show the ground, whole (see tiledGround). Any
  // renderer but Apple's must manage that on its first draw; on Apple's, a
  // capture that still lost some is redrawn -- same scene, same maps, no bake
  // -- at most kRedraws times, each redraw reported.
#ifdef __APPLE__
  const int kRedraws = 3;
#else
  const int kRedraws = 0;
#endif
  auto capture = [&](const std::string &what) {
    const bool settled = settle();
    std::vector<unsigned char> px = view.frameRGB();
    long missing = backgroundPixels(px);
    for (int i = 0; missing && i < kRedraws; ++i) {
      std::printf("  (%s: %ld pixel(s) without the ground; redraw %d of %d)\n", what.c_str(),
                  missing, i + 1, kRedraws);
      px = view.frameRGB();
      missing = backgroundPixels(px);
    }
    chk(settled && view.frameWidth() == W && view.frameHeight() == H && missing == 0,
        what + ": a settled " + std::to_string(view.frameWidth()) + "x" +
            std::to_string(view.frameHeight()) + " frame, the ground in all of it (" +
            std::to_string(missing) + " background pixels)");
    return px;
  };
  const std::vector<unsigned char> casting = capture("slab casting");
  slab->setCastsShadow(false);
  const std::vector<unsigned char> notCasting = capture("slab not casting");
  slab->setVisible(false);
  const std::vector<unsigned char> absent = capture("slab absent");
  slab->setVisible(true);
  slab->setCastsShadow(true);
  frame();
  const int rx0 = W / 2 + 8, rx1 = W - 4, lx0 = 4, lx1 = W / 2 - 8;
  const double lCast = meanLuma(casting, W, H, rx0, rx1);
  const double lNot = meanLuma(notCasting, W, H, rx0, rx1);
  const double lAbsent = meanLuma(absent, W, H, rx0, rx1);
  const double lLeft = meanLuma(casting, W, H, lx0, lx1);
  const double lLeftNot = meanLuma(notCasting, W, H, lx0, lx1);
  std::printf("  luma, right half: caster %.1f  non-caster %.1f  absent %.1f  (left half %.1f, "
              "%.1f without the caster)\n",
              lCast, lNot, lAbsent, lLeft, lLeftNot);
  // The left half is the lit reference of the same frame.
  chk(lLeft > 20.0, "the ground is lit");
  chk(lCast < 0.5 * lLeft, "a casting slab darkens the ground (to under half its lit level)");
  chk(lNot > 0.9 * lLeftNot, "a non-casting slab does not (its footprint is lit like the rest)");
#ifdef __APPLE__
  // Apple's renderer: within the 1-luma bound the absent / non-casting
  // comparison has always met there, rather than byte for byte.
  chk(std::fabs(lNot - lAbsent) < 1.0 && std::fabs(lLeft - lLeftNot) < 1.0,
      "a non-casting slab leaves the ground as if the slab were absent; the shadow stays in "
      "its footprint (within 1 luma)");
#else
  chk(!notCasting.empty() && notCasting == absent &&
          sameRegion(casting, notCasting, W, H, lx0, lx1),
      "a non-casting slab leaves the ground as if the slab were absent; the shadow stays in "
      "its footprint (byte-exact)");
#endif

  std::printf("F. deformed in place, and streamed\n");
  cvc::geometry tg = box(-50, 40, 0, -45, 45, 6);
  for (std::size_t i = 0; i < tg.points().size(); ++i)
    tg.colors().push_back({0.2 + 0.1 * static_cast<double>(i % 3), 0.5, 0.3});
  auto sway = sg.getGraphicsRoot()->addGraphicsChild<GeometryNode>("sway");
  sway->setUseSingleColor(false);
  sway->setGeometry(tg); // per-vertex colours; normals derived
  chk(settle(), "a new coloured caster settles: no bake once it is in the maps");
  std::vector<double> xyz;
  for (const auto &p : tg.points()) {
    xyz.push_back(p[0] + 0.5);
    xyz.push_back(p[1]);
    xyz.push_back(p[2]);
  }
  sway->updateVertices(xyz);
  chk(frame(), "a caster's updateVertices (sway) bakes");
  chk(!frame(), "and the frame after does not");
  std::vector<double> nrm;
  for (std::size_t i = 0; i < tg.points().size(); ++i) {
    nrm.push_back(0.0);
    nrm.push_back(0.0);
    nrm.push_back(1.0);
  }
  sway->updateNormals(nrm);
  chk(frame(), "updateNormals bakes");
  std::vector<unsigned char> rgb(3 * tg.points().size(), 200);
  sway->updateColors(rgb);
  chk(frame(), "updateColors bakes");
  chk(!frame(), "and then it rests");
  const cvc::geometry vg = box(-45, -45, 0, -40, -40, 3);
  std::vector<double> vxyz;
  for (const auto &p : vg.points()) {
    vxyz.push_back(p[0] + 1.0);
    vxyz.push_back(p[1]);
    vxyz.push_back(p[2]);
  }
  vehicle->updateVertices(vxyz);
  chk(!frame(), "a non-caster's updateVertices does not bake");

  auto ribbon = sg.getGraphicsRoot()->addGraphicsChild<cvc::gl::RibbonNode>(
      "ribbon", 16, 1.0f, cvc::bounding_box(-60, -60, 0, 60, 60, 1));
  chk(!ribbon->castsShadow(), "a RibbonNode does not cast by default");
  settle();
  bakes = 0;
  for (int i = 0; i < 4; ++i) {
    ribbon->append(-40.0f + 5.0f * static_cast<float>(i), -30.0f, 0.2f);
    bakes += frame() ? 1 : 0;
  }
  chk(bakes == 0, "4 frames appending to it: " + std::to_string(bakes) + " bakes (want 0)");

  cvc::gl::StreamingLayout lay;
  lay.capacity_points = 4;
  lay.triangles = {{{0, 1, 2}}, {{0, 2, 3}}};
  lay.reserved_bounds = cvc::bounding_box(-55, -55, 0, -35, -35, 8);
  auto streamed =
      sg.getGraphicsRoot()->addGraphicsChild<cvc::gl::StreamingGeometryNode>("streamed", lay);
  chk(streamed->castsShadow(), "a plain StreamingGeometryNode casts by default");
  const float quad[12] = {-55, -55, 2, -50, -55, 2, -50, -50, 6, -55, -50, 6};
  streamed->writePoints(0, quad, 4);
  chk(settle(), "it settles");
  const float quad2[12] = {-55, -55, 3, -50, -55, 3, -50, -50, 7, -55, -50, 7};
  streamed->writePoints(0, quad2, 4);
  chk(frame(), "a casting streaming node's write bakes");
  chk(!frame(), "once");
  streamed->setCastsShadow(false);
  frame(); // it leaves the maps
  streamed->writePoints(0, quad, 4);
  chk(!frame(), "a non-casting one's write does not");

  // Changes with no point write: a new draw range, a relayout.
  StreamNode::Layout lay2 = lay;
  lay2.triangles.push_back({{1, 2, 3}});
  auto shaped = sg.getGraphicsRoot()->addGraphicsChild<StreamNode>("shaped", lay);
  shaped->writePoints(0, quad, 4);
  chk(settle(), "a second casting streaming node settles");
  shaped->setDrawRange(0, 1);
  chk(frame(), "a casting node's setDrawRange alone bakes");
  chk(!frame(), "once");
  shaped->relayoutTo(lay2);
  chk(frame(), "a casting node's relayout alone bakes");
  chk(settle(), "and settles");
  shaped->setCastsShadow(false);
  frame();
  shaped->setDrawRange(0, 2);
  chk(!frame(), "a non-casting node's setDrawRange does not");
  shaped->relayoutTo(lay);
  chk(!frame(), "nor its relayout");

  // A draped link moved and reshaped by uniforms alone.
  auto heights = std::make_shared<cvc::gl::HeightFieldTexture>(8, 8, -60.0, -60.0, 17.0, 17.0);
  auto link = sg.getGraphicsRoot()->addGraphicsChild<cvc::gl::DrapedLinkNode>("link", heights, 12);
  chk(!link->castsShadow(), "a DrapedLinkNode does not cast by default");
  link->setEndpoints(-50.0f, 20.0f, -20.0f, 50.0f);
  link->setStyle(2.0f, 15.0f, 0.9f, 0.2f, 0.2f, 1.0f); // lifted: it has a shadow to throw
  chk(settle(), "it settles");
  link->setEndpoints(-50.0f, 20.0f, -20.0f, 52.0f);
  chk(!frame(), "a non-casting link's move does not bake");
  link->setCastsShadow(true);
  chk(frame(), "it starts casting: a bake");
  chk(settle(), "and settles");
  const cvc::gl::StreamStats u0 = link->streamStats();
  link->setEndpoints(-48.0f, 20.0f, -20.0f, 54.0f);
  chk(frame(), "a casting link's setEndpoints bakes");
  const cvc::gl::StreamStats u1 = link->streamStats();
  chk(u1.uploads == u0.uploads && u1.fullUploads == u0.fullUploads,
      "and uploads nothing (uniforms only)");
  chk(!frame(), "once");
  link->setStyle(2.0f, 15.0f, 0.2f, 0.9f, 0.2f, 1.0f);
  chk(!frame(), "a colour-only restyle does not bake");
  link->setStyle(3.0f, 15.0f, 0.2f, 0.9f, 0.2f, 1.0f);
  chk(frame(), "a new width does");
  chk(settle(), "settles");
  std::vector<float> row(8, 4.0f);
  heights->updateRows(2, 1, row.data());
  bool baked = frame();
  baked = frame() || baked; // the drape is re-derived as VTK asks for the bounds
  chk(baked, "new heights under a casting link re-bake");

  // The baker remembers each caster mapper's input and looks it up again only
  // when the mapper changes: a new input object must be followed, the old one
  // forgotten.
  auto swap = sg.getGraphicsRoot()->addGraphicsChild<Node>("swap");
  swap->setGeometry(box(40, 40, 0, 45, 45, 6));
  chk(settle(), "a caster to re-wire settles");
  auto *swapActor = vtkActor::SafeDownCast(swap->actor());
  vtkPolyData *oldInput =
      vtkPolyData::SafeDownCast(swapActor->GetMapper()->GetInputDataObject(0, 0));
  vtkNew<vtkPolyData> newInput;
  newInput->DeepCopy(oldInput);
  vtkPolyDataMapper::SafeDownCast(swapActor->GetMapper())->SetInputData(newInput);
  chk(frame(), "a new input object bakes");
  chk(!frame(), "once");
  newInput->GetPoints()->SetPoint(0, 40.0, 40.0, 9.0);
  newInput->GetPoints()->Modified();
  newInput->Modified();
  chk(frame(), "the new input deformed in place bakes");
  oldInput->GetPoints()->SetPoint(0, 40.0, 40.0, 12.0);
  oldInput->Modified();
  chk(!frame(), "the old, replaced input changing does not");

  std::printf("E. the interval still strides a moving caster's bakes\n");
  // A scene of its own, with the interval set BEFORE shadows go on: changing a
  // shadow setting on a live scene rebuilds the whole pass chain.
  SceneGraph sg2(app, "casterstride");
  sg2.setDiagnosticChromeVisible(false);
  auto ground2 = sg2.getGraphicsRoot()->addGraphicsChild<GeometryNode>("ground");
  ground2->setGeometry(box(-60, -60, -1, 60, 60, 0));
  auto mover = sg2.addGraphics("mover", box(-5, -5, 0, 5, 5, 10));
  sg2.processEvents();
  SceneRenderer view2(sg2, 48, 48, /*offscreen=*/true, "main");
  view2.setCamera(0, -40, 40, 0, 0, 0, 0, 0, 1, 40.0, 1.0, 200.0);
  sg2.addDirectionalLight(90.0, 45.0);
  sg2.setShadowUpdateInterval(3);
  sg2.setShadowsEnabled(true);
  vtkShadowMapBakerPass *baker2 = findBaker(view2.renderer()->GetPass());
  chk(baker2 != nullptr, "found its baker");
  if (!baker2)
    return;
  for (int i = 0; i < 4; ++i) {
    sg2.processEvents();
    view2.render();
  }
  bakes = 0;
  for (int i = 0; i < 12; ++i) {
    mover->setPosition(0.0, 0.1 * i, 0.0);
    sg2.processEvents();
    view2.render();
    bakes += baker2->GetNeedUpdate() ? 1 : 0;
  }
  chk(bakes == 4,
      "12 frames moving a caster, interval 3: " + std::to_string(bakes) + " bakes (want 4)");
}

int main() {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  flags(app);
  rendered(app);
  std::printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "PASSED", fails, fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
