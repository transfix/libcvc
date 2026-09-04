// Growing the set of shadow-casting lights must not corrupt the shadow pass.
//
// vtkShadowMapBakerPass sizes its ShadowMaps vector inside Render(), and only
// when NeedUpdate is set. SceneGraph's StridedShadowBaker skips most bakes for
// speed (SetUpToDate() on 2 of every 3 frames by default), which clears
// NeedUpdate WITHOUT resizing. On such a frame the downstream vtkShadowMapPass
// walks the renderer's CURRENT light list while indexing the PREVIOUS bake's
// vector, with no bounds check:
//
//     map = (*GetShadowMaps())[shadowingLightIndex];
//     map->Activate();
//
// so ADDING a caster reads past the end and dereferences garbage, while
// REMOVING one merely ends the loop early with every index valid. That
// asymmetry is exactly what made "increase the wash light count" die while
// decreasing it looked perfectly healthy.
//
// This drives the failing direction: shadows on, a skipping interval, and the
// caster count climbing one light at a time with a render after each. Against
// the unfixed baker this segfaults reproducibly; it must now survive.
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/LightNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <string>

using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;

static int fails = 0;
static void chk(bool ok, const std::string &w) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", w.c_str());
  if (!ok)
    ++fails;
}

// A unit cube, so there is opaque geometry for the depth pass to bake.
static cvc::geometry box(double x, double y, double z, double s) {
  cvc::geometry g;
  const double v[8][3] = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
                          {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
  for (const auto &p : v)
    g.points().push_back({x + s * p[0], y + s * p[1], z + s * p[2]});
  const int f[12][3] = {{0, 1, 2}, {0, 2, 3}, {4, 6, 5}, {4, 7, 6}, {0, 4, 5}, {0, 5, 1},
                        {1, 5, 6}, {1, 6, 2}, {2, 6, 7}, {2, 7, 3}, {3, 7, 4}, {3, 4, 0}};
  for (const auto &p : v)
    g.colors().push_back({0.6, 0.6, 0.62});
  for (const auto &t : f)
    g.tris().push_back({static_cast<unsigned int>(t[0]), static_cast<unsigned int>(t[1]),
                        static_cast<unsigned int>(t[2])});
  return g;
}

int main() {
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  SceneGraph sg(app, "scg");
  SceneRenderer view(sg, 160, 120, /*offscreen=*/true, "main");

  for (int i = 0; i < 6; ++i)
    sg.addGraphics("box" + std::to_string(i), box(i * 2.0, 0.0, 0.0, 1.0));
  sg.processEvents();

  sg.setShadowResolution(512);
  // Interval > 1 is the whole point: with 1 (bake every frame) the vector is
  // resized every time and the bug cannot appear.
  sg.setShadowUpdateInterval(3);
  chk(sg.setShadowsEnabled(true), "shadows enabled");
  view.render();

  // Grow the caster set one Spot at a time. Spot with cone < 90 is what VTK's
  // LightCreatesShadow() accepts, so each of these really does add a shadow map.
  // Several renders per light so a skipped-bake frame is certain to be hit.
  for (int i = 0; i < 6; ++i) {
    auto ln = sg.addLight("caster" + std::to_string(i));
    ln->setKind(cvc::gl::LightNode::Kind::Spot);
    ln->setCone(35.0);
    ln->setPosition(4.0 + 2.0 * i, -8.0, 9.0);
    ln->setTarget(4.0, 0.0, 0.0);
    sg.processEvents();
    for (int f = 0; f < 4; ++f)
      view.render();
  }
  chk(true, "6 casters added one at a time, 4 renders each, no crash");

  // Shrink again — the direction that always worked — so a regression that
  // breaks removal instead of addition is caught too.
  for (int i = 0; i < 6; ++i) {
    sg.removeGraphics("caster" + std::to_string(i));
    sg.processEvents();
    for (int f = 0; f < 4; ++f)
      view.render();
  }
  chk(true, "and removed again, no crash");

  // Hiding and re-showing moves the same count through the same path, which is
  // how the per-light checkboxes and "All on" reach it.
  auto a = sg.addLight("toggle_a");
  a->setKind(cvc::gl::LightNode::Kind::Spot);
  a->setCone(30.0);
  a->setPosition(0.0, -8.0, 8.0);
  sg.processEvents();
  for (int cycle = 0; cycle < 3; ++cycle) {
    a->setVisible(false);
    for (int f = 0; f < 3; ++f)
      view.render();
    a->setVisible(true); // <- the growing edge again
    for (int f = 0; f < 3; ++f)
      view.render();
  }
  chk(true, "hide/show cycles (the per-light checkbox path), no crash");

  std::printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "PASSED", fails, fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
