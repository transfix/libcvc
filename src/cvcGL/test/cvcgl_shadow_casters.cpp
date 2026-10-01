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
//      all), a caster does;
//   E. the update interval still strides bakes of a moving caster.
// B-E render for real and skip where nothing rasterises (fatal under
// CVC_REQUIRE_RENDER=1).
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/NullGraphicNode.h>
#include <cvc/gl/RibbonNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/StreamingGeometryNode.h>
#include <string>
#include <vector>
#include <vtkActor.h>
#include <vtkCameraPass.h>
#include <vtkNew.h>
#include <vtkRenderPassCollection.h>
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

static double meanLuma(const std::vector<unsigned char> &rgb, int w, int h, int x0, int x1) {
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
  // stand elsewhere, out of view too.
  SceneGraph sg(app, "casters");
  sg.setDiagnosticChromeVisible(false);
  auto ground = sg.getGraphicsRoot()->addGraphicsChild<GeometryNode>("ground");
  ground->setGeometry(box(-60, -60, -1, 60, 60, 0));
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
  view.setBackground(0.0, 0.0, 0.0);
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

  std::printf("D. on screen\n");
  const std::vector<unsigned char> casting = view.frameRGB();
  slab->setCastsShadow(false);
  frame();
  const std::vector<unsigned char> notCasting = view.frameRGB();
  slab->setVisible(false);
  frame();
  const std::vector<unsigned char> absent = view.frameRGB();
  slab->setVisible(true);
  slab->setCastsShadow(true);
  frame();
  const double lCast = meanLuma(casting, W, H, W / 2 + 8, W - 4);
  const double lNot = meanLuma(notCasting, W, H, W / 2 + 8, W - 4);
  const double lAbsent = meanLuma(absent, W, H, W / 2 + 8, W - 4);
  const double lLeft = meanLuma(casting, W, H, 4, W / 2 - 8);
  std::printf("  luma, right half: caster %.1f  non-caster %.1f  absent %.1f  (left half %.1f)\n",
              lCast, lNot, lAbsent, lLeft);
  chk(lLeft > 20.0, "the ground is lit");
  chk(lCast < lNot - 5.0, "a casting slab darkens the ground");
  chk(std::fabs(lNot - lAbsent) < 1.0, "a non-casting slab leaves it as if the slab were absent");

  std::printf("F. deformed in place, and streamed\n");
  auto settle = [&]() {
    for (int i = 0; i < 6; ++i)
      if (!frame())
        return true;
    return false;
  };
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
