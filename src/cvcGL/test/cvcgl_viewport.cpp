// ViewportManager: N Viewports composited as SetViewport/SetLayer regions in
// ONE vtkRenderWindow (one GL context — the WASM single-canvas constraint), the
// picture-in-picture path the SceneRenderer test documents as unsolvable with
// two SceneRenderers over one scene.
//
// What is pinned here:
//   * construction auto-creates a full-screen PRIMARY viewport over the main
//     scene, so the single-view case is a drop-in for SceneRenderer;
//   * a SECOND viewport over a SECOND scene composites as an inset in the same
//     window — both rects draw, each with its own camera and background;
//   * each distinct scene's geometry reaches its own viewport and no other;
//   * attaching a scene that another viewport already draws is a LOUD error —
//     the exact silent "second setRenderer blanks the first view" trap the
//     SceneRenderer test pins as undiagnosed with two SceneRenderers;
//   * duplicate viewport names and bad sizes are rejected; lookup is diagnosed.
//
// Offscreen throughout, so this runs headless in CI. It renders for real, so it
// needs a working GL driver (CI supplies Mesa llvmpipe).
// These tests assert(), and cvcpkg builds them Release -- where NDEBUG makes
// assert() expand to nothing and every check below would pass vacuously.
// Undefine it before <cassert> so the assertions actually run.
#undef NDEBUG
#include <cassert>
#include <cstddef>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/Viewport.h>
#include <cvc/gl/ViewportManager.h>
#include <stdexcept>
#include <string>
#include <vector>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>

using cvc::gl::SceneGraph;
using cvc::gl::Viewport;
using cvc::gl::ViewportManager;

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

struct Rgb {
  double r = 0, g = 0, b = 0;
};

// Mean colour over a sub-rectangle, in 0..1. frameRGB rows are bottom-up (row 0
// is the bottom of the window), so `py` is measured from the bottom edge —
// matching the y-up normalized regions passed to the viewports.
Rgb meanRect(const std::vector<unsigned char> &f, int W, int H, int px0, int py0, int px1,
             int py1) {
  assert(px0 >= 0 && py0 >= 0 && px1 <= W && py1 <= H && px0 < px1 && py0 < py1);
  double R = 0, G = 0, B = 0;
  long n = 0;
  for (int py = py0; py < py1; ++py)
    for (int px = px0; px < px1; ++px) {
      const size_t i = (static_cast<size_t>(py) * W + px) * 3;
      R += f[i];
      G += f[i + 1];
      B += f[i + 2];
      ++n;
    }
  return {R / n / 255.0, G / n / 255.0, B / n / 255.0};
}

double luma(const Rgb &c) { return 0.2126 * c.r + 0.7152 * c.g + 0.0722 * c.b; }

// Fraction of bytes that differ between two frames within a sub-rectangle.
double rectDelta(const std::vector<unsigned char> &a, const std::vector<unsigned char> &b, int W,
                 int H, int px0, int py0, int px1, int py1) {
  assert(a.size() == b.size());
  (void)H;
  size_t diff = 0, tot = 0;
  for (int py = py0; py < py1; ++py)
    for (int px = px0; px < px1; ++px) {
      const size_t i = (static_cast<size_t>(py) * W + px) * 3;
      for (int c = 0; c < 3; ++c) {
        if (a[i + c] != b[i + c])
          ++diff;
        ++tot;
      }
    }
  return tot ? static_cast<double>(diff) / static_cast<double>(tot) : 0.0;
}

} // namespace

int main() {
  cvc::app app;
  // Distinct state prefixes: the two scenes (and their per-viewport camera
  // state) live in separate subtrees of the one app's state.
  SceneGraph mainScene(app, "main_scene");
  SceneGraph insetScene(app, "inset_scene");

  const int W = 320, H = 240;

  // ── construction: a full-screen primary over the main scene ────────────────
  ViewportManager vm(mainScene, W, H, /*offscreen=*/true);
  assert(vm.frameWidth() == W && vm.frameHeight() == H);
  assert(!vm.windowClosed()); // offscreen has no window to close
  assert(vm.renderWindow() != nullptr);
  assert(vm.hasViewport("main"));
  assert(&vm.primary() == &vm.viewport("main"));
  assert(vm.viewportNames().size() == 1);
  vm.primary().setBackground(0.10, 0.10, 0.45); // opaque blue base, whole window

  // ── an inset over a SECOND scene, bottom-right, drawn on top (layer 1) ──────
  const double insetRegion[4] = {0.62, 0.05, 0.97, 0.42};
  Viewport &inset = vm.addSceneViewport("inset", insetScene, insetRegion, /*layer=*/1);
  inset.setBackground(0.55, 0.28, 0.05); // opaque orange, clears its own rect
  assert(vm.hasViewport("inset"));
  assert(vm.viewportNames().size() == 2);
  assert(!inset.isMirror());

  // ── both viewports composite at their rects, each with its own background ───
  // Sample a rect inside the primary-only area (top-left) and a rect well inside
  // the inset. The base is blue (B>R); the inset is orange (R>B). If the inset
  // failed to composite, its rect would show the blue base and R<B there.
  std::vector<unsigned char> bg = vm.frameRGB();
  assert(bg.size() == static_cast<size_t>(W) * H * 3);
  // Two sample rects, each fully inside one viewport: the primary rect sits
  // left of and above the inset (so it is primary-only) yet spans the window
  // centre, and the inset rect sits well inside the inset. On the empty-scene
  // frame both are pure background (blue / orange) for the hue test below; once
  // a quad is added and both cameras reframe, both rects gain the centred quad
  // for the geometry test.
  const int mPx0 = 90, mPy0 = 110, mPx1 = 180, mPy1 = 160; // primary, left of + above the inset
  const int iPx0 = 224, iPy0 = 36, iPx1 = 288, iPy1 = 84;  // inside the inset
  Rgb m = meanRect(bg, W, H, mPx0, mPy0, mPx1, mPy1);
  Rgb ins = meanRect(bg, W, H, iPx0, iPy0, iPx1, iPy1);
  printf("  primary rect  rgb=(%.3f,%.3f,%.3f) luma=%.3f\n", m.r, m.g, m.b, luma(m));
  printf("  inset   rect  rgb=(%.3f,%.3f,%.3f) luma=%.3f\n", ins.r, ins.g, ins.b, luma(ins));
  assert(luma(m) > 0.02 && "primary viewport drew nothing");
  assert(luma(ins) > 0.02 && "inset viewport drew nothing");
  assert(m.b > m.r && "primary rect is not the blue base — primary did not composite");
  assert(ins.r > ins.b &&
         "inset rect is not the orange inset — it did not composite over the base");

  // ── each distinct scene's geometry reaches ONLY its own viewport ───────────
  // Add a quad to each scene, reframe both cameras, and require both rects to
  // change: geometry added after attach must appear (drained by render()), and
  // it must appear in the viewport whose scene it belongs to.
  addQuad(mainScene, "ground", -50, -50, 50, 50, 0.0);
  addQuad(insetScene, "ground", -50, -50, 50, 50, 0.0);
  vm.render(); // drain the new geometry into both renderers
  vm.primary().renderer()->ResetCamera();
  inset.renderer()->ResetCamera();
  std::vector<unsigned char> geo = vm.frameRGB();
  const double mDelta = rectDelta(bg, geo, W, H, mPx0, mPy0, mPx1, mPy1);
  const double iDelta = rectDelta(bg, geo, W, H, iPx0, iPy0, iPx1, iPy1);
  printf("  primary rect delta after geometry: %.4f\n", mDelta);
  printf("  inset   rect delta after geometry: %.4f\n", iDelta);
  assert(mDelta > 0.0 && "main scene geometry never reached the primary viewport");
  assert(iDelta > 0.0 && "inset scene geometry never reached the inset viewport");

  // ── the double-attach trap is now LOUD ─────────────────────────────────────
  // Re-attaching a scene another viewport already draws is the silent
  // "second setRenderer blanks the first view" bug the SceneRenderer test pins
  // as undiagnosed. Here it must throw rather than blank the primary.
  bool threw = false;
  try {
    const double r[4] = {0.1, 0.1, 0.4, 0.4};
    vm.addSceneViewport("main_again", mainScene, r);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  assert(threw && "re-attaching an already-drawn scene must be a loud error, not a blanked view");
  assert(!vm.hasViewport("main_again"));

  // A duplicate viewport name is rejected (checked before the scene, so use a
  // fresh scene to prove it is the NAME that trips it).
  SceneGraph other(app, "other_scene");
  threw = false;
  try {
    const double r[4] = {0.0, 0.0, 0.3, 0.3};
    vm.addSceneViewport("inset", other, r);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  assert(threw && "a duplicate viewport name must be rejected");

  // Lookup of an unknown viewport is diagnosed, not a null deref.
  assert(!vm.hasViewport("nope"));
  threw = false;
  try {
    (void)vm.viewport("nope");
  } catch (const std::out_of_range &) {
    threw = true;
  }
  assert(threw && "viewport() on an unknown name must throw");

  // Still exactly the two real viewports; a failed add left nothing behind.
  assert(vm.viewportNames().size() == 2);

  // ── bad construction arguments are rejected ────────────────────────────────
  threw = false;
  try {
    ViewportManager bad(mainScene, 0, 10);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  assert(threw && "a zero-width manager must be rejected");

  printf("cvcgl_viewport: OK\n");
  return 0;
}
