// Headless checks for the world-units integration on the scene graph:
//   * GraphicsNode::localToWorld / worldToLocal round-trip through a node's
//     transform (including a parent/child compose and the lazily-cached inverse
//     invalidating after a move), and
//   * GraphicsNode::localPointToReal mapping a point in a graphic's OWN
//     coordinate frame to a real-world km/mile coordinate via cvc::world_units.
// Plus a SKIP-guarded smoke check of SceneRenderer::pickWorld's miss contract,
// which needs a GL context (an offscreen renderer) and is skipped where none is
// available, exactly like the other context-dependent cvcGL tests.
//
// The matrix math needs no renderer at all: a SceneGraph with a null renderer
// composes transforms synchronously, which is the whole point of keeping the
// pose math off the rasteriser.

#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/world_units.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/state_publisher.h>
#include <memory>
#include <stdexcept>

using cvc::gl::GraphicsNode;
using cvc::gl::SceneGraph;

// NOT assert(): cvcpkg builds these Release (NDEBUG), where assert() is a no-op
// and a "passing" run proves nothing.
static int g_failures = 0;
static void check_impl(bool ok, const char *expr, int line) {
  if (!ok) {
    std::printf("  FAIL line %d: %s\n", line, expr);
    ++g_failures;
  }
}
#define CHECK(cond) check_impl(static_cast<bool>(cond), #cond, __LINE__)

// Named 'approx', not 'near': <windef.h> (dragged in transitively by VTK on
// Windows) #defines `near` to nothing, which would silently mangle this.
static bool approx(double a, double b, double tol = 1e-9) { return std::fabs(a - b) <= tol; }

static cvc::geometry dot() {
  cvc::geometry g;
  cvc::geometry::point_t p;
  p[0] = p[1] = p[2] = 0;
  g.points().push_back(p);
  return g;
}

// Translation only: local origin lands at the node's position, and worldToLocal
// is its exact inverse.
static void test_translation_roundtrip() {
  cvc::app app;
  SceneGraph sg(app);
  sg.publisher().stop(); // deterministic, no background flush thread
  auto n = sg.addGraphics("t", dot());
  n->setPosition(10.0, 20.0, 30.0);

  double w[3];
  const double origin[3] = {0, 0, 0};
  n->localToWorld(origin, w);
  CHECK(approx(w[0], 10.0) && approx(w[1], 20.0) && approx(w[2], 30.0));

  const double lp[3] = {1.0, 2.0, 3.0};
  n->localToWorld(lp, w);
  CHECK(approx(w[0], 11.0) && approx(w[1], 22.0) && approx(w[2], 33.0));

  double back[3];
  n->worldToLocal(w, back);
  CHECK(approx(back[0], 1.0) && approx(back[1], 2.0) && approx(back[2], 3.0));
  std::printf("  ok: translation localToWorld/worldToLocal round-trip\n");
}

// A combined scale + translate via an explicit row-major matrix, so the math is
// unambiguous, and the inverse recovers the local point.
static void test_scale_translate_inverse() {
  cvc::app app;
  SceneGraph sg(app);
  sg.publisher().stop();
  auto n = sg.addGraphics("st", dot());
  // scale 2 on the diagonal, translation (10,20,30).
  const double m[16] = {2, 0, 0, 10, 0, 2, 0, 20, 0, 0, 2, 30, 0, 0, 0, 1};
  n->setTransform(m);

  double w[3];
  const double lp[3] = {1.0, 1.0, 1.0};
  n->localToWorld(lp, w);
  CHECK(approx(w[0], 12.0) && approx(w[1], 22.0) && approx(w[2], 32.0));

  double back[3];
  n->worldToLocal(w, back);
  CHECK(approx(back[0], 1.0) && approx(back[1], 1.0) && approx(back[2], 1.0));
  std::printf("  ok: scale+translate inverse recovers the local point\n");
}

// A child's local frame composes with its parent's, so localToWorld from the
// child accounts for the whole chain.
static void test_parent_child_compose() {
  cvc::app app;
  SceneGraph sg(app);
  sg.publisher().stop();
  auto group = sg.addGraphics("group");
  auto leaf = group->createChild<cvc::gl::GeometryNode>("leaf", dot());
  group->setPosition(100.0, 0.0, 0.0);
  leaf->setPosition(5.0, 0.0, 0.0);

  double w[3];
  const double origin[3] = {0, 0, 0};
  leaf->localToWorld(origin, w);
  CHECK(approx(w[0], 105.0) && approx(w[1], 0.0) && approx(w[2], 0.0));

  double back[3];
  leaf->worldToLocal(w, back);
  CHECK(approx(back[0], 0.0) && approx(back[1], 0.0) && approx(back[2], 0.0));
  std::printf("  ok: parent/child transforms compose in localToWorld\n");
}

// The cached inverse must invalidate when the node moves: a second pose must be
// reflected by worldToLocal, not the stale inverse of the first.
static void test_lazy_inverse_refreshes_after_move() {
  cvc::app app;
  SceneGraph sg(app);
  sg.publisher().stop();
  auto n = sg.addGraphics("mv", dot());

  n->setPosition(10.0, 0.0, 0.0);
  double back[3];
  const double w10[3] = {10.0, 0.0, 0.0};
  n->worldToLocal(w10, back); // computes + caches the inverse
  CHECK(approx(back[0], 0.0));

  n->setPosition(20.0, 0.0, 0.0); // must dirty the cached inverse
  const double w20[3] = {20.0, 0.0, 0.0};
  n->worldToLocal(w20, back);
  CHECK(approx(back[0], 0.0)); // 0, not 10 (which a stale inverse would give)
  std::printf("  ok: cached world-inverse refreshes after a move\n");
}

// The headline: a point in a graphic's own coordinate frame reported as a real
// world coordinate in the active regime.
static void test_local_point_to_real() {
  cvc::app app;
  SceneGraph sg(app);
  sg.publisher().stop();
  auto n = sg.addGraphics("terrain", dot());
  n->setPosition(3200.0, 500.0, -1000.0); // world units == metres by default

  cvc::world_units si; // SI, 1 m per world unit
  const double origin[3] = {0, 0, 0};
  auto c = n->localPointToReal(origin, si);
  CHECK(c.unit == "km"); // largest axis 3200 m -> km for the whole point
  CHECK(approx(c.x, 3.2) && approx(c.y, 0.5) && approx(c.z, -1.0));

  cvc::world_units::config icfg;
  icfg.regime = cvc::world_units::system::imperial;
  cvc::world_units imp{icfg};
  // 2 miles east of the origin, in the node's own frame.
  auto n2 = sg.addGraphics("terrain2", dot());
  n2->setPosition(2.0 * 1609.344, 0.0, 0.0);
  auto ci = n2->localPointToReal(origin, imp);
  CHECK(ci.unit == "mi");
  CHECK(approx(ci.x, 2.0));
  std::printf("  ok: localPointToReal reports km/miles in the node's own frame\n");
}

// SceneRenderer::pickWorld miss contract: a pick over an empty scene hits
// nothing and returns false. Needs a GL context; SKIP where none is available
// (matching the repo's other context-dependent tests).
static void test_pick_world_miss_contract() {
  cvc::app app;
  SceneGraph sg(app);
  try {
    cvc::gl::SceneRenderer r(sg, 256, 256, /*offscreen=*/true, "pick");
    r.render();
    double w[3] = {123, 123, 123};
    const bool hit = r.pickWorld(128.0, 128.0, w);
    CHECK(!hit);                                      // empty scene -> nothing under the cursor
    CHECK(w[0] == 123 && w[1] == 123 && w[2] == 123); // outWorld left untouched on a miss
    r.close();
    std::printf("  ok: pickWorld returns false (and leaves out untouched) on an empty scene\n");
  } catch (const std::exception &e) {
    std::printf("  SKIP: no offscreen GL context for pickWorld (%s)\n", e.what());
  } catch (...) {
    std::printf("  SKIP: no offscreen GL context for pickWorld\n");
  }
}

int main() {
  test_translation_roundtrip();
  test_scale_translate_inverse();
  test_parent_child_compose();
  test_lazy_inverse_refreshes_after_move();
  test_local_point_to_real();
  test_pick_world_miss_contract();
  if (g_failures) {
    std::printf("cvcGL world units: %d FAILED\n", g_failures);
    return 1;
  }
  std::printf("cvcGL world units: OK\n");
  return 0;
}
