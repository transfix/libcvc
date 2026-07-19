// Headless functional check for cvcGL: build a scene graph, add a geometry
// node, run the re-pumpable pump (no Qt loop, no VTK renderer), and query.
// Proves the extracted scene graph is usable standalone / SWIG-drivable.
//
// (removeGraphics() is intentionally not exercised: it segfaults in a
// headless no-renderer context — tracked as a follow-up.)
#include <cassert>
#include <cstdio>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/SceneGraph.h>

int main() {
  SceneGraph sg;

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
