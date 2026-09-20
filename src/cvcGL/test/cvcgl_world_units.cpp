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

// The 8 corners of the unit cube [0,1]^3, so getBoundingBox() == [0,1]^3.
static cvc::geometry unit_cube() {
  cvc::geometry g;
  for (int c = 0; c < 8; ++c) {
    cvc::geometry::point_t p;
    p[0] = (c & 1) ? 1.0 : 0.0;
    p[1] = (c & 2) ? 1.0 : 0.0;
    p[2] = (c & 4) ? 1.0 : 0.0;
    g.points().push_back(p);
  }
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

// A node's world bounding box is its local box pushed through the WHOLE chain of
// local transforms, and its real dimensions come straight off that — no manual
// walk of the parent chain.
static void test_world_bbox_and_dimensions_through_chain() {
  cvc::app app;
  SceneGraph sg(app);
  sg.publisher().stop();

  auto p = sg.addGraphics("p", unit_cube());
  p->setPosition(100.0, 0.0, 0.0); // parent: translate +100 x
  auto c = p->createChild<cvc::gl::GeometryNode>("c", unit_cube());
  // child local: scale 2, translate (10,20,30).
  const double m[16] = {2, 0, 0, 10, 0, 2, 0, 20, 0, 0, 2, 30, 0, 0, 0, 1};
  c->setTransform(m);

  // Child world = translate(100) * (scale2 + translate(10,20,30)); the unit cube
  // [0,1]^3 maps to [110,112] x [20,22] x [30,32].
  auto wb = c->getWorldBoundingBox();
  CHECK(approx(wb.minx, 110.0) && approx(wb.miny, 20.0) && approx(wb.minz, 30.0));
  CHECK(approx(wb.maxx, 112.0) && approx(wb.maxy, 22.0) && approx(wb.maxz, 32.0));

  cvc::world_units si; // SI, 1 m/unit
  auto d = c->realDimensions(si, /*includeChildren=*/false);
  CHECK(d.unit == "m");
  CHECK(approx(d.x, 2.0) && approx(d.y, 2.0) && approx(d.z, 2.0));
  std::printf("  ok: world bbox + dimensions compose through the transform chain\n");
}

// Moving an ancestor must be reflected in a descendant's world bbox/dimensions
// with no explicit refresh — the cached world matrix is kept current top-down.
static void test_dimensions_track_ancestor_moves() {
  cvc::app app;
  SceneGraph sg(app);
  sg.publisher().stop();

  auto p = sg.addGraphics("p", unit_cube());
  p->setPosition(100.0, 0.0, 0.0);
  auto c = p->createChild<cvc::gl::GeometryNode>("c", unit_cube());
  const double m[16] = {2, 0, 0, 10, 0, 2, 0, 20, 0, 0, 2, 30, 0, 0, 0, 1};
  c->setTransform(m);
  CHECK(approx(c->getWorldBoundingBox().minx, 110.0));

  p->setPosition(1000.0, 0.0, 0.0); // move the parent
  auto wb = c->getWorldBoundingBox();
  CHECK(approx(wb.minx, 1010.0)); // followed the ancestor: 1000 + 10
  cvc::world_units si;
  auto d = c->realDimensions(si, false);
  CHECK(approx(d.x, 2.0) && approx(d.y, 2.0) && approx(d.z, 2.0)); // size unchanged
  std::printf("  ok: descendant dimensions follow an ancestor move\n");
}

// realDimensions promotes to km / miles by magnitude, just like a coordinate.
static void test_real_dimensions_promote_units() {
  cvc::app app;
  SceneGraph sg(app);
  sg.publisher().stop();

  auto big = sg.addGraphics("big", unit_cube());
  const double s[16] = {2000, 0, 0, 0, 0, 2000, 0, 0, 0, 0, 2000, 0, 0, 0, 0, 1};
  big->setTransform(s); // 2000 m on each axis
  cvc::world_units si;
  auto dk = big->realDimensions(si, false);
  CHECK(dk.unit == "km" && approx(dk.x, 2.0));

  cvc::world_units::config icfg;
  icfg.regime = cvc::world_units::system::imperial;
  cvc::world_units imp{icfg};
  auto bigmi = sg.addGraphics("bigmi", unit_cube());
  const double sm = 2.0 * 1609.344; // 2 miles per axis
  const double s2[16] = {sm, 0, 0, 0, 0, sm, 0, 0, 0, 0, sm, 0, 0, 0, 0, 1};
  bigmi->setTransform(s2);
  auto dmi = bigmi->realDimensions(imp, false);
  CHECK(dmi.unit == "mi" && approx(dmi.x, 2.0));
  std::printf("  ok: real dimensions promote to km / miles\n");
}

// The combined world box encloses this node AND its descendants, through the
// chain.
static void test_combined_world_bbox_covers_subtree() {
  cvc::app app;
  SceneGraph sg(app);
  sg.publisher().stop();

  auto pp = sg.addGraphics("pp", unit_cube()); // [0,1]^3 at the origin
  auto cc = pp->createChild<cvc::gl::GeometryNode>("cc", unit_cube());
  cc->setPosition(10.0, 0.0, 0.0); // child cube at [10,11] x [0,1] x [0,1]

  auto comb = pp->getCombinedWorldBoundingBox();
  CHECK(approx(comb.minx, 0.0) && approx(comb.maxx, 11.0));
  CHECK(approx(comb.miny, 0.0) && approx(comb.maxy, 1.0));
  CHECK(approx(comb.minz, 0.0) && approx(comb.maxz, 1.0));

  cvc::world_units si;
  auto d = pp->realDimensions(si, /*includeChildren=*/true);
  CHECK(d.unit == "m" && approx(d.x, 11.0) && approx(d.y, 1.0) && approx(d.z, 1.0));
  std::printf("  ok: combined world bbox + dimensions cover the whole subtree\n");
}

// A transform carrying many significant digits (a real-world coordinate, e.g. a
// UTM easting) must survive the round trip through the string-backed state tree
// intact. Before the full-precision serialization fix the value was truncated to
// six significant figures, so dimensions/coordinates taken through the chain were
// silently wrong.
static void test_high_precision_transform_survives_roundtrip() {
  cvc::app app;
  SceneGraph sg(app);
  sg.publisher().stop();
  auto n = sg.addGraphics("hp", dot());

  const double ex = 500123.45678901234; // ~17 sig figs (a UTM-easting-scale value)
  const double ey = 4649987.6543210987;
  const double ez = 12.34567890123456;
  // setTransform writes the value to the state tree and reloads it synchronously
  // -- the exact path that used to truncate to 6 significant figures.
  const double m[16] = {1, 0, 0, ex, 0, 1, 0, ey, 0, 0, 1, ez, 0, 0, 0, 1};
  n->setTransform(m);

  double w[3];
  const double origin[3] = {0, 0, 0};
  n->localToWorld(origin, w);
  CHECK(approx(w[0], ex, 1e-6)); // not degraded to 6 significant digits (~500123)
  CHECK(approx(w[1], ey, 1e-6));
  CHECK(approx(w[2], ez, 1e-9));
  std::printf("  ok: a high-precision transform survives the state round-trip\n");
}

// SceneRenderer::pickWorld smoke test against a live offscreen context. A
// SceneGraph is NOT empty -- it carries default chrome (a world grid at z=0) --
// so whether the centre pixel hits geometry is not this test's concern. What
// must hold is the method's contract regardless of scene content: a reported hit
// carries finite coordinates (never NaN/Inf), and a miss leaves outWorld
// untouched. Needs a GL context; SKIP where none is available (matching the
// repo's other context-dependent tests).
static void test_pick_world_smoke() {
  cvc::app app;
  SceneGraph sg(app);
  try {
    cvc::gl::SceneRenderer r(sg, 256, 256, /*offscreen=*/true, "pick");
    r.render();
    const double sentinel = 123.0;
    double w[3] = {sentinel, sentinel, sentinel};
    const bool hit = r.pickWorld(128.0, 128.0, w);
    if (hit) {
      CHECK(std::isfinite(w[0]) && std::isfinite(w[1]) && std::isfinite(w[2]));
    } else {
      CHECK(w[0] == sentinel && w[1] == sentinel && w[2] == sentinel); // untouched on a miss
    }
    r.close();
    std::printf("  ok: pickWorld runs against a live context and honours its contract (%s)\n",
                hit ? "hit" : "miss");
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
  test_world_bbox_and_dimensions_through_chain();
  test_dimensions_track_ancestor_moves();
  test_real_dimensions_promote_units();
  test_combined_world_bbox_covers_subtree();
  test_high_precision_transform_survives_roundtrip();
  test_pick_world_smoke();
  if (g_failures) {
    std::printf("cvcGL world units: %d FAILED\n", g_failures);
    return 1;
  }
  std::printf("cvcGL world units: OK\n");
  return 0;
}
