// SceneGraph's scene-wide diagnostic chrome: the setters, and the getters that
// let a UI draw a CHECKBOX instead of a pair of on/off buttons.
//
// The getters are DERIVED ("is any box drawn?"), not a remembered copy of the
// last value passed in, and that distinction is the point of the last check
// here: GraphicsNode observes its own `show_bbox` state key, so a script, a
// config or a replicated peer can switch one on without ever calling
// setBBoxesVisible(). A mirrored member would keep reporting "all off" with a
// box on screen — a tick that lies, which is exactly what this API exists to
// avoid.
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <memory>

using namespace cvc::gl;

// NOT assert(): these run Release (NDEBUG), where assert() compiles to nothing
// and a "passing" run would prove exactly nothing.
static int g_failures = 0;
static void check_impl(bool ok, const char *expr, int line) {
  if (!ok) {
    std::printf("  FAIL line %d: %s\n", line, expr);
    ++g_failures;
  }
}
#define CHECK(cond) check_impl(static_cast<bool>(cond), #cond, __LINE__)

namespace {

cvc::geometry tri(double x1, double y1, double z1) {
  cvc::geometry g;
  const double xyz[3][3] = {{0, 0, 0}, {x1, 0, 0}, {x1, y1, z1}};
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

void test_bboxes_round_trip() {
  std::printf("test_bboxes_round_trip\n");
  cvc::app app;
  SceneGraph sg(app);
  sg.addGraphics("a", tri(10, 10, 10));
  sg.addGraphics("b", tri(20, 20, 20));
  sg.processEvents();

  sg.setBBoxesVisible(false);
  CHECK(!sg.bboxesVisible());
  for (const auto &n : sg.getAllGraphicsOfType<GraphicsNode>())
    CHECK(n && !n->getShowBBox());

  // The regression that made this API necessary: the old call reached only the
  // root, so turning them "on" left children off and vice versa.
  sg.setBBoxesVisible(true);
  CHECK(sg.bboxesVisible());
  for (const auto &n : sg.getAllGraphicsOfType<GraphicsNode>())
    CHECK(n && n->getShowBBox());
}

void test_extent_labels_round_trip() {
  std::printf("test_extent_labels_round_trip\n");
  cvc::app app;
  SceneGraph sg(app);
  sg.addGraphics("a", tri(10, 10, 10));
  sg.processEvents();

  sg.setExtentLabelsVisible(false);
  CHECK(!sg.extentLabelsVisible());
  sg.setExtentLabelsVisible(true);
  CHECK(sg.extentLabelsVisible());
}

// One call has to reach all four, because "hide the chrome" with one of them
// left on is the bug this replaced.
void test_diagnostic_chrome_covers_all_four() {
  std::printf("test_diagnostic_chrome_covers_all_four\n");
  cvc::app app;
  SceneGraph sg(app);
  sg.addGraphics("a", tri(10, 10, 10));
  sg.processEvents();

  sg.setDiagnosticChromeVisible(false);
  CHECK(!sg.gridVisible());
  CHECK(!sg.axisVisible());
  CHECK(!sg.bboxesVisible());
  CHECK(!sg.extentLabelsVisible());
  CHECK(!sg.diagnosticChromeVisible());

  sg.setDiagnosticChromeVisible(true);
  CHECK(sg.gridVisible());
  CHECK(sg.axisVisible());
  CHECK(sg.bboxesVisible());
  CHECK(sg.extentLabelsVisible());
  CHECK(sg.diagnosticChromeVisible());
}

// The reason the getters are derived rather than remembered.
void test_getter_sees_a_node_turned_on_behind_its_back() {
  std::printf("test_getter_sees_a_node_turned_on_behind_its_back\n");
  cvc::app app;
  SceneGraph sg(app);
  sg.addGraphics("a", tri(10, 10, 10));
  sg.processEvents();

  sg.setDiagnosticChromeVisible(false);
  CHECK(!sg.bboxesVisible());
  CHECK(!sg.diagnosticChromeVisible());

  // Straight at the node, never through SceneGraph — what a state write does.
  auto node = sg.getGraphics("a"); // already a GraphicsNode
  CHECK(node != nullptr);
  if (node)
    node->setShowBBox(true);

  // A remembered "I last set them all false" would still say false here.
  CHECK(sg.bboxesVisible());
  CHECK(sg.diagnosticChromeVisible());
}

} // namespace

int main() {
  test_bboxes_round_trip();
  test_extent_labels_round_trip();
  test_diagnostic_chrome_covers_all_four();
  test_getter_sees_a_node_turned_on_behind_its_back();

  if (g_failures) {
    std::printf("cvcgl_scene_chrome: %d FAILURE(S)\n", g_failures);
    return 1;
  }
  std::printf("cvcgl_scene_chrome: OK\n");
  return 0;
}
