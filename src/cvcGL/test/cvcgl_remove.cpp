// Headless functional check for cvcGL graphics *removal*: build a scene graph,
// add a geometry node, pump, then remove it. Exercises the add + remove path
// with no VTK renderer set (m_renderer == nullptr) — the shape the embedded
// Python lab (pycvc.gl) drives when scripting scene nodes headless.
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
  assert(sg.hasGraphics("tri"));

  // The path under test: remove the node headless (no renderer set).
  sg.removeGraphics("tri");
  sg.processEvents();
  assert(!sg.hasGraphics("tri"));

  std::printf("cvcGL remove: OK (nodes=%zu)\n", sg.getAllGraphics().size());
  return 0;
}
