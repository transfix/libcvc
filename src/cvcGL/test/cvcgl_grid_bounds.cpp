// The world grid must enclose the geometry that is actually in the scene.
//
// It did not. The grid is sized only by SceneGraph::updateGrid(), and the sole
// caller was onGraphicsBoundsChanged(), which fires on
// GraphicsNode::transformChanged. So the grid tracked geometry that MOVED and
// was blind to geometry that was merely ADDED: a scene that builds itself once
// and never animates drew a grid that did not enclose its own contents, while
// the graphics root's bbox — which auto-syncs to its children — did. Two
// mechanisms, one of them never run, which is why they disagreed on screen.
//
// addGraphics() also did not call registerGraphics(); it inlined the same
// map-assign + trackNodeBounds pair, so four of five registration sites had
// drifted apart. They funnel through one path now.
//
// These checks read GridNode::bounds() — the extent the grid is DRAWN at.
// getBoundingBox() deliberately reports an empty box so the grid never
// contributes to scene bounds, which also made the drawn extent unobservable:
// before bounds() existed there was no way to test this at all.
//
// Animated demos hid the bug by moving a node on the first frame. These checks
// pin the static case, which is the one that was broken: 8 of them fail against
// the unfixed SceneGraph.
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GridNode.h>
#include <cvc/gl/SceneGraph.h>
#include <memory>

using namespace cvc::gl;

// NOT assert(): these tests are built Release (NDEBUG), where assert() compiles
// to nothing and a "passing" run would prove exactly nothing.
static int g_failures = 0;
static void check_impl(bool ok, const char *expr, int line) {
  if (!ok) {
    std::printf("  FAIL line %d: %s\n", line, expr);
    ++g_failures;
  }
}
#define CHECK(cond) check_impl(static_cast<bool>(cond), #cond, __LINE__)

namespace {

// An axis-aligned box of the requested extent, as two triangles per face is
// overkill — a single triangle spanning the corners fixes the bounds, which is
// all these checks read.
cvc::geometry box(double x0, double y0, double z0, double x1, double y1, double z1) {
  cvc::geometry g;
  const double xyz[3][3] = {{x0, y0, z0}, {x1, y0, z0}, {x1, y1, z1}};
  for (auto &v : xyz) {
    cvc::geometry::point_t p;
    p[0] = v[0];
    p[1] = v[1];
    p[2] = v[2];
    g.points().push_back(p);
  }
  cvc::geometry::tri_t t;
  t[0] = 0;
  t[1] = 1;
  t[2] = 2;
  g.tris().push_back(t);
  return g;
}

bool encloses(const cvc::bounding_box &outer, const cvc::bounding_box &inner) {
  for (int i = 0; i < 3; ++i)
    if (outer[i] > inner[i] || outer[i + 3] < inner[i + 3])
      return false;
  return true;
}

bool degenerate(const cvc::bounding_box &b) {
  for (int i = 0; i < 3; ++i)
    if (b[i + 3] > b[i])
      return false;
  return true;
}

// The recompute is marshalled onto the owner thread (updateGrid touches VTK
// actors), so a test must pump the queue before reading the grid back.
cvc::bounding_box grid_bounds(SceneGraph &sg) {
  sg.processEvents();
  auto g = sg.getGridNode();
  // bounds(), NOT getBoundingBox(): the latter deliberately reports an empty box
  // so the grid does not contribute to scene bounds. Reading it here would be
  // asserting against the constant (0,0,0,0,0,0) and could never pass.
  return g ? g->bounds() : cvc::bounding_box();
}

// A scene that is built once and never animated — the case that was broken.
void test_grid_encloses_static_geometry() {
  std::printf("test_grid_encloses_static_geometry\n");
  cvc::app app;
  SceneGraph sg(app);

  sg.addGraphics("subject", box(-50, -38, 0, 50, 38, 99));

  const cvc::bounding_box grid = grid_bounds(sg);
  const cvc::bounding_box scene = sg.computeGraphicsBounds();

  // The regression itself: the grid was left default-constructed.
  CHECK(!degenerate(grid));
  CHECK(encloses(grid, scene));
}

// bunny_shadow's exact shape: a small subject added first, then a much larger
// ground plane. The grid must grow to the plane, not stay at the subject.
void test_grid_grows_when_a_larger_node_is_added() {
  std::printf("test_grid_grows_when_a_larger_node_is_added\n");
  cvc::app app;
  SceneGraph sg(app);

  sg.addGraphics("bunny", box(-50, -38, 0, 50, 38, 99));
  const cvc::bounding_box afterFirst = grid_bounds(sg);
  CHECK(!degenerate(afterFirst));

  sg.addGraphics("ground", box(-220, -220, 0, 220, 220, 0));
  const cvc::bounding_box afterSecond = grid_bounds(sg);

  CHECK(encloses(afterSecond, sg.computeGraphicsBounds()));
  // Specifically: it reaches the plane, not merely the subject.
  CHECK(afterSecond[0] <= -220.0);
  CHECK(afterSecond[3] >= 220.0);
  // ...and still contains what it already had.
  CHECK(encloses(afterSecond, afterFirst));
}

// Order must not matter: adding the big node first and the small one second has
// to land on the same enclosure.
void test_add_order_does_not_matter() {
  std::printf("test_add_order_does_not_matter\n");
  cvc::app app;
  SceneGraph sg(app);

  sg.addGraphics("ground", box(-220, -220, 0, 220, 220, 0));
  sg.addGraphics("bunny", box(-50, -38, 0, 50, 38, 99));

  const cvc::bounding_box grid = grid_bounds(sg);
  CHECK(encloses(grid, sg.computeGraphicsBounds()));
  CHECK(grid[0] <= -220.0);
  CHECK(grid[5] >= 99.0);
}

// Removing geometry must let the grid follow, not strand it at the old extent.
void test_grid_follows_removal() {
  std::printf("test_grid_follows_removal\n");
  cvc::app app;
  SceneGraph sg(app);

  sg.addGraphics("bunny", box(-50, -38, 0, 50, 38, 99));
  sg.addGraphics("ground", box(-220, -220, 0, 220, 220, 0));
  CHECK(grid_bounds(sg)[3] >= 220.0);

  sg.removeGraphics("ground");
  const cvc::bounding_box grid = grid_bounds(sg);

  CHECK(encloses(grid, sg.computeGraphicsBounds()));
  // The 220-wide plane is gone; the grid must not still be claiming it.
  CHECK(grid[3] < 220.0);
}

} // namespace

int main() {
  test_grid_encloses_static_geometry();
  test_grid_grows_when_a_larger_node_is_added();
  test_add_order_does_not_matter();
  test_grid_follows_removal();

  if (g_failures) {
    std::printf("cvcgl_grid_bounds: %d FAILURE(S)\n", g_failures);
    return 1;
  }
  std::printf("cvcgl_grid_bounds: OK\n");
  return 0;
}
