// ViewportManager input router: one window, one interactor, N cameras. The
// router's two hard parts — the routing DECISION (display x,y -> which Viewport)
// and the DISPATCH (feed that viewport's CameraController) — are PUBLIC,
// interactor-free methods, so this test drives them directly with synthetic
// bottom-left pixel coordinates and needs NO live interactor (offscreen, headless
// CI). Camera effects are observed through each viewport's own vtkCamera.
//
// What is pinned here:
//   * viewportAt is layer-, visibility- and inputEnabled-aware, and returns null
//     over the gutter (unlike VTK's FindPokedRenderer, which falls back to the
//     primary);
//   * focus-follows-click writes active_viewport to cvc::state;
//   * keyboard goes to the ACTIVE viewport only;
//   * a left drag stays latched to the viewport it began in even when the cursor
//     leaves it, and never leaks into another viewport;
//   * the wheel goes to the viewport under the cursor;
//   * Escape releases pointer capture; a focus change releases the outgoing
//     viewport's held keys (no drift);
//   * the single-view (primary-only) path behaves exactly like before;
//   * offscreen there is no interactor yet every route* still works.
//
// These tests assert(), and cvcpkg builds them Release -- where NDEBUG makes
// assert() expand to nothing and every check below would pass vacuously.
// Undefine it before <cassert> so the assertions actually run.
#undef NDEBUG
#include <cassert>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/Viewport.h>
#include <cvc/gl/ViewportManager.h>
#include <string>
#include <vtkCamera.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>

using cvc::gl::CameraController;
using cvc::gl::SceneGraph;
using cvc::gl::Viewport;
using cvc::gl::ViewportManager;
using MB = ViewportManager::MouseButton;

namespace {

void addQuad(SceneGraph &sg, const std::string &name, double x0, double y0, double x1, double y1,
             double z) {
  cvc::geometry g;
  const double xs[4] = {x0, x1, x1, x0};
  const double ys[4] = {y0, y0, y1, y1};
  for (int i = 0; i < 4; ++i) {
    cvc::geometry::point_t p;
    p[0] = xs[i];
    p[1] = ys[i];
    p[2] = z;
    g.points().push_back(p);
  }
  const int idx[2][3] = {{0, 1, 2}, {0, 2, 3}};
  for (auto &tri : idx) {
    cvc::geometry::tri_t t;
    t[0] = tri[0];
    t[1] = tri[1];
    t[2] = tri[2];
    g.tris().push_back(t);
  }
  sg.addGraphics(name, g);
}

struct Pose {
  double p[3], f[3];
};

Pose pose(Viewport &v) {
  Pose q;
  vtkCamera *c = v.renderer()->GetActiveCamera();
  c->GetPosition(q.p);
  c->GetFocalPoint(q.f);
  return q;
}

double d2(const double a[3], const double b[3]) {
  double s = 0;
  for (int i = 0; i < 3; ++i) {
    const double d = a[i] - b[i];
    s += d * d;
  }
  return s;
}

// Deterministic cameras: an untouched controller reproduces its pose exactly, so
// "same" is a tight bound and "moved" a clearly separated one.
bool moved(const Pose &a, const Pose &b) { return d2(a.p, b.p) > 1e-6 || d2(a.f, b.f) > 1e-6; }
bool same(const Pose &a, const Pose &b) { return d2(a.p, b.p) <= 1e-9 && d2(a.f, b.f) <= 1e-9; }

} // namespace

int main() {
  cvc::app app;
  SceneGraph mainScene(app, "main_scene");
  SceneGraph insetScene(app, "inset_scene");
  addQuad(mainScene, "ground", -50, -50, 50, 50, 0.0);
  addQuad(insetScene, "ground", -50, -50, 50, 50, 0.0);

  const int W = 320, H = 240;
  ViewportManager vm(mainScene, W, H, /*offscreen=*/true);
  // Offscreen invariant: no interactor exists, the router style is never
  // installed, yet every route* below still works.
  assert(vm.renderWindow()->GetInteractor() == nullptr &&
         "offscreen manager must have no interactor");

  const double insetRegion[4] = {0.62, 0.05, 0.97, 0.42}; // px x[198,310] y[12,101]
  Viewport &inset = vm.addSceneViewport("inset", insetScene, insetRegion, 1);
  const double mirrorRegion[4] = {0.62, 0.58, 0.97, 0.95}; // px x[198,310] y[139,228]
  Viewport &mini = vm.addMirrorViewport("mini", "main", mirrorRegion, 2);
  vm.render();
  // Settle every controller to its own param-driven pose first: the FIRST
  // update() applies the controller's params over VTK's ResetCamera framing (a
  // one-time jump), so baselines taken before it would read a pose no input
  // caused. After this, update(0.0) is idempotent and only real input moves a
  // camera.
  vm.updateCameras(0.0);

  const int inPrimary[2] = {40, 120}; // primary-only area
  const int inInset[2] = {254, 56};   // inside inset (layer 1, over primary)
  const int inMini[2] = {254, 187};   // inside mini  (layer 2, over primary)

  // ── viewportAt: layer / visibility / inputEnabled / gutter ─────────────────
  assert(vm.viewportAt(inPrimary[0], inPrimary[1]) == &vm.primary());
  assert(vm.viewportAt(inInset[0], inInset[1]) == &inset); // higher layer wins over primary
  assert(vm.viewportAt(inMini[0], inMini[1]) == &mini);
  assert(vm.viewportAt(400, 400) == nullptr && "gutter must return null, not fall back to primary");

  inset.setVisible(false);
  assert(vm.viewportAt(inInset[0], inInset[1]) == &vm.primary() &&
         "hidden viewport must be skipped");
  inset.setVisible(true);
  assert(vm.viewportAt(inInset[0], inInset[1]) == &inset);

  mini.setInputEnabled(false);
  assert(vm.viewportAt(inMini[0], inMini[1]) == &vm.primary() &&
         "input-disabled viewport must fall through to what is beneath it");
  mini.setInputEnabled(true);
  assert(vm.viewportAt(inMini[0], inMini[1]) == &mini);

  // ── focus-follows-click writes active_viewport to state ────────────────────
  vm.routeMouseButton(MB::Left, true, inInset[0], inInset[1]);
  vm.routeMouseButton(MB::Left, false, inInset[0], inInset[1]);
  assert(vm.activeViewport() == &inset);
  assert(cvc::state::instance(app)("main_scene.active_viewport").value() == std::string("inset") &&
         "focus-follows-click did not write active_viewport state");

  // ── keyboard goes to the ACTIVE viewport only (Fly held-key motion) ─────────
  // Put BOTH viewports in Fly and settle them, so a mis-routed / broadcast key
  // WOULD visibly move the non-active one — otherwise the negative assertion is
  // vacuous (an Orbit camera never moves on a keypress regardless of routing).
  inset.camera().setMode(CameraController::Mode::Fly);
  vm.primary().camera().setMode(CameraController::Mode::Fly);
  vm.updateCameras(0.0);
  vm.routeMouseButton(MB::Left, true, inInset[0], inInset[1]); // focus inset
  vm.routeMouseButton(MB::Left, false, inInset[0], inInset[1]);
  const Pose ki0 = pose(inset), km0 = pose(vm.primary());
  vm.routeKey("w", true);
  for (int k = 0; k < 5; ++k)
    vm.updateCameras(0.05);
  vm.routeKey("w", false);
  assert(moved(ki0, pose(inset)) && "held key did not move the active (inset) camera");
  assert(same(km0, pose(vm.primary())) &&
         "keypress reached a non-active viewport (broadcast) even though it too is in Fly");
  vm.primary().camera().setMode(CameraController::Mode::Orbit); // restore for later tests

  // ── a left drag stays latched to its origin viewport across the boundary ────
  inset.camera().setMode(CameraController::Mode::Orbit);
  vm.updateCameras(0.0);
  const Pose di0 = pose(inset), dm0 = pose(vm.primary());
  vm.routeMouseButton(MB::Left, true, inInset[0], inInset[1]); // latch inset
  vm.routeMouseMove(inPrimary[0], inPrimary[1]);               // cursor leaves into primary area
  vm.routeMouseButton(MB::Left, false, inPrimary[0], inPrimary[1]);
  vm.updateCameras(0.0);
  assert(moved(di0, pose(inset)) && "latched drag did not steer the inset camera");
  assert(same(dm0, pose(vm.primary())) && "the drag leaked into the primary camera");

  // ── the wheel goes to the viewport under the cursor ────────────────────────
  {
    const Pose i0 = pose(inset), m0 = pose(vm.primary());
    vm.routeMouseWheel(inInset[0], inInset[1], 1.0);
    vm.updateCameras(0.0);
    assert(moved(i0, pose(inset)) && "wheel over inset did not zoom the inset");
    assert(same(m0, pose(vm.primary())) && "wheel over inset perturbed the primary");
  }
  {
    const Pose i0 = pose(inset), m0 = pose(vm.primary());
    vm.routeMouseWheel(inPrimary[0], inPrimary[1], 1.0);
    vm.updateCameras(0.0);
    assert(moved(m0, pose(vm.primary())) && "wheel over primary did not zoom the primary");
    assert(same(i0, pose(inset)) && "wheel over primary perturbed the inset");
  }

  // ── middle-drag pans by a SANE amount (render window must be wired) ─────────
  // Frame the inset to the ground so its orbit distance is scene-scaled; then a
  // 10 px middle-pan must shift the camera by a small fraction of the scene. The
  // null-render-window bug makes the pan scale fall back to vh=1 and slews the
  // camera by hundreds of units on the first pixel.
  inset.camera().setMode(CameraController::Mode::Orbit);
  inset.camera().frameBounds(-50, -50, 0, 50, 50, 0);
  vm.updateCameras(0.0);
  {
    const Pose i0 = pose(inset);
    vm.routeMouseButton(MB::Middle, true, inInset[0], inInset[1]);
    vm.routeMouseMove(inInset[0] + 10, inInset[1]);
    vm.routeMouseButton(MB::Middle, false, inInset[0] + 10, inInset[1]);
    vm.updateCameras(0.0);
    const double shift2 = d2(i0.p, pose(inset).p);
    assert(shift2 > 1e-6 && "middle-drag did not pan the inset");
    assert(shift2 < 2500.0 &&
           "middle-drag pan is wildly large — the controller has no render window (vh=1 fallback)");
  }

  // ── an overlapping Left+Middle chord keeps the still-held gesture alive ─────
  // Middle-pan, tap Left (down+up) while Middle stays held: the pan must keep
  // working. Regression: CameraController collapses pan/orbit onto one flag, so
  // a naive endDrag on the Left-up used to clear it and freeze the pan.
  {
    const Pose p0 = pose(inset);
    vm.routeMouseButton(MB::Middle, true, inInset[0], inInset[1]); // begin pan
    vm.routeMouseButton(MB::Left, true, inInset[0], inInset[1]);   // Left down during the pan
    vm.routeMouseButton(MB::Left, false, inInset[0], inInset[1]);  // Left up must NOT freeze it
    vm.routeMouseMove(inInset[0] + 10, inInset[1]);                // Middle still held -> pan
    vm.routeMouseButton(MB::Middle, false, inInset[0] + 10, inInset[1]);
    vm.updateCameras(0.0);
    assert(moved(p0, pose(inset)) &&
           "a Left tap during a Middle pan froze the pan (shared dragging flag cleared)");
  }

  // ── gutter events are safe no-ops ──────────────────────────────────────────
  vm.primary().setVisible(false);
  inset.setVisible(false);
  mini.setVisible(false);
  vm.routeMouseButton(MB::Left, true, inInset[0], inInset[1]); // nothing under cursor
  vm.routeMouseMove(inMini[0], inMini[1]);
  vm.routeMouseWheel(inPrimary[0], inPrimary[1], 1.0);
  vm.routeMouseButton(MB::Left, false, inInset[0], inInset[1]);
  vm.primary().setVisible(true);
  inset.setVisible(true);
  mini.setVisible(true);

  // ── Escape releases pointer capture (and is not a held key) ────────────────
  inset.camera().setMode(CameraController::Mode::Fly);
  vm.routeMouseButton(MB::Left, true, inInset[0], inInset[1]); // focus inset
  vm.routeMouseButton(MB::Left, false, inInset[0], inInset[1]);
  inset.camera().setPointerCapture(true);
  assert(inset.camera().pointerCapture());
  vm.routeKey("Escape", true);
  assert(!inset.camera().pointerCapture() && "Escape did not release pointer capture");
  inset.camera().setMode(CameraController::Mode::Orbit);

  // ── a focus change releases the outgoing viewport's held keys (no drift) ────
  inset.camera().setMode(CameraController::Mode::Fly);
  vm.routeMouseButton(MB::Left, true, inInset[0], inInset[1]); // focus inset
  vm.routeMouseButton(MB::Left, false, inInset[0], inInset[1]);
  vm.updateCameras(0.0);  // settle inset's Fly pose before it holds a key
  vm.routeKey("w", true); // inset now holds "w"
  vm.routeMouseButton(MB::Left, true, inPrimary[0], inPrimary[1]); // refocus primary
  vm.routeMouseButton(MB::Left, false, inPrimary[0], inPrimary[1]);
  assert(vm.activeViewport() == &vm.primary());
  vm.updateCameras(0.0); // one settled tick with no keys held
  const Pose hi0 = pose(inset);
  for (int k = 0; k < 5; ++k)
    vm.updateCameras(0.05);
  assert(same(hi0, pose(inset)) &&
         "inset kept drifting: its held key was not released on focus-out");
  inset.camera().setMode(CameraController::Mode::Orbit);

  // ── single-view (primary only) behaves exactly like before ─────────────────
  {
    SceneGraph solo(app, "solo_scene");
    addQuad(solo, "ground", -50, -50, 50, 50, 0.0);
    ViewportManager svm(solo, 200, 150, /*offscreen=*/true);
    svm.render();
    assert(svm.viewportAt(100, 75) == &svm.primary() && "any in-window point is the primary");
    svm.primary().camera().setMode(CameraController::Mode::Orbit);
    svm.updateCameras(0.0);
    const Pose p0 = pose(svm.primary());
    svm.routeMouseButton(MB::Left, true, 100, 75);
    svm.routeMouseMove(140, 90);
    svm.routeMouseButton(MB::Left, false, 140, 90);
    svm.updateCameras(0.0);
    assert(moved(p0, pose(svm.primary())) && "single-view drag did not steer the primary camera");
  }

  printf("cvcgl_viewport_input: OK\n");
  return 0;
}
