// Teardown-safety check for cvcGL. Hammers the add/remove/destroy paths that
// used to crash (a node destroyed while a main-thread callback that captured it
// was still queued) and asserts the two architectural invariants that make
// teardown race-free:
//
//   1. No background handler threads are ever spawned for scene nodes — state
//      handlers run synchronously. (Asserted via the app's thread map.)
//   2. Destroying a node/scene with callbacks still queued (never pumped) is
//      safe: the callbacks are weak-guarded no-ops.
//
// This is the regression guard for the removeGraphics use-after-free. Before the
// fix a tight add/remove loop faulted within the first iterations; it must now
// run clean regardless of pump timing or destruction order.
// These tests assert(), and cvcpkg builds them Release -- where NDEBUG makes
// assert() expand to nothing and every check below would pass vacuously.
// Undefine it before <cassert> so the assertions actually run.
#undef NDEBUG
#include <cassert>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/SceneGraph.h>
#include <string>
#include <vtkNew.h>
#include <vtkRenderer.h>

static cvc::geometry makeTri() {
  cvc::geometry tri;
  cvc::geometry::point_t p;
  for (int i = 0; i < 3; ++i) {
    p[0] = i;
    p[1] = i * 2;
    p[2] = 1;
    tri.points().push_back(p);
  }
  cvc::geometry::tri_t t;
  t[0] = 0;
  t[1] = 1;
  t[2] = 2;
  tri.tris().push_back(t);
  return tri;
}

// Count live "_stateChanged" handler threads in the scene's app context. The
// hardened design never spawns any; this must stay 0 throughout.
static std::size_t handlerThreadCount(cvc::app &app) {
  std::size_t n = 0;
  for (auto &kv : app.threads()) {
    if (kv.first.find("_stateChanged") != std::string::npos)
      ++n;
  }
  return n;
}

int main() {
  cvc::app app;
  const cvc::geometry tri = makeTri();

  // Case A — the exact original crash repro: add, pump, remove, then destroy
  // without pumping again. The remove's/destroy's still-queued callbacks must
  // not dangle.
  {
    SceneGraph sg(app);
    auto n = sg.addGraphics("tri", tri);
    assert(n);
    sg.processEvents();
    sg.removeGraphics("tri");
    assert(!sg.hasGraphics("tri"));
    // no processEvents() here on purpose; ~SceneGraph tears down with work queued
  }

  // Case B — tight churn, deliberately varying pump timing and never draining
  // between add and remove, across many short-lived scenes. Teardown is
  // deterministic now (no threads), so a few hundred iterations is a firm guard;
  // the original defect faulted within the first iteration.
  for (int iter = 0; iter < 50; ++iter) {
    SceneGraph sg(app);
    for (int k = 0; k < 6; ++k)
      sg.addGraphics("g" + std::to_string(k), tri);
    if (iter % 3 == 0)
      sg.processEvents(); // sometimes pump, sometimes not
    for (int k = 0; k < 6; ++k)
      sg.removeGraphics("g" + std::to_string(k));
    // scene destroyed here with callbacks possibly still queued
    assert(handlerThreadCount(app) == 0 && "scene nodes must not spawn handler threads");
  }

  // Case C — renderer-attached teardown (the shape Scene::show / render_png
  // drive). A bare vtkRenderer needs no window; this exercises the
  // add/removeFromRenderer + setRenderer(nullptr) teardown paths under the
  // guarded pump. No display required.
  {
    SceneGraph sg(app);
    vtkNew<vtkRenderer> renderer;
    sg.setRenderer(renderer);
    sg.addGraphics("a", tri);
    sg.addGraphics("b", tri);
    sg.processEvents();
    sg.removeGraphics("a");
    sg.processEvents();
    sg.setRenderer(nullptr); // detach; nodes leave the renderer
    // ~SceneGraph tears the rest down
  }

  assert(handlerThreadCount(app) == 0);
  std::printf("cvcGL teardown: OK (no handler threads, churn + renderer clean)\n");
  return 0;
}
