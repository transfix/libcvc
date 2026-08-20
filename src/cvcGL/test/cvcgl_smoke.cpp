// Headless functional check for cvcGL: build a scene graph, add a geometry
// node, run the re-pumpable pump (no Qt loop, no VTK renderer), and query.
// Proves the extracted scene graph is usable standalone / SWIG-drivable.
//
// (The headless add + remove path is exercised separately by cvcgl_remove.)
// These tests assert(), and cvcpkg builds them Release -- where NDEBUG makes
// assert() expand to nothing and every check below would pass vacuously.
// Undefine it before <cassert> so the assertions actually run.
#undef NDEBUG
#include <cassert>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/SceneGraph.h>

int main() {
  cvc::app app;
  SceneGraph sg(app);

  cvc::geometry tri;
  cvc::geometry::point_t p;
  for (int i = 0; i < 3; ++i) {
    p[0] = i;
    p[1] = 0;
    p[2] = 0;
    tri.points().push_back(p);
  }
  cvc::geometry::tri_t t;
  t[0] = 0;
  t[1] = 1;
  t[2] = 2;
  tri.tris().push_back(t);

  auto node = sg.addGraphics("tri", tri);
  assert(node);
  sg.processEvents(); // the re-pumpable pump
  assert(sg.getGraphics("tri"));
  assert(sg.getAllGraphics().size() >= 1);

  std::printf("cvcGL smoke: OK (nodes=%zu)\n", sg.getAllGraphics().size());
  return 0;
}
