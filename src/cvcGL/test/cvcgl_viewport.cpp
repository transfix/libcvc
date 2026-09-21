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
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/Viewport.h>
#include <cvc/gl/ViewportManager.h>
#include <stdexcept>
#include <string>
#include <vector>
#include <vtkActor2D.h>
#include <vtkCollection.h>
#include <vtkLight.h>
#include <vtkLightCollection.h>
#include <vtkProp.h>
#include <vtkPropCollection.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkTextActor.h>

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

  // ── a MIRROR viewport: an alternate camera over an already-drawn scene ──────
  // The minimap case. It draws mainScene (already attached to the primary) from
  // its OWN camera, WITHOUT re-attaching the scene — so it is allowed exactly
  // where addSceneViewport would (rightly) throw.
  const double mirrorRegion[4] = {0.62, 0.58, 0.97, 0.95}; // top-right, above the inset
  Viewport &mini = vm.addMirrorViewport("mini", "main", mirrorRegion, /*layer=*/2);
  assert(mini.isMirror());
  assert(&mini.scene() == &mainScene && "a mirror must echo the source's scene");
  assert(mini.mirrorSource() == vm.primary().renderer());
  assert(vm.viewportNames().size() == 3);
  mini.setBackground(0.05, 0.15, 0.05); // dark green, opaque (luma ~0.12)
  mini.renderer()->ResetCamera();       // frame the shared props with its own camera

  const int gPx0 = 205, gPy0 = 150, gPx1 = 305, gPy1 = 222; // most of the mirror rect
  std::vector<unsigned char> mir0 = vm.frameRGB();
  Rgb miniC = meanRect(mir0, W, H, gPx0, gPy0, gPx1, gPy1);
  printf("  mirror rect   rgb=(%.3f,%.3f,%.3f) luma=%.3f\n", miniC.r, miniC.g, miniC.b,
         luma(miniC));
  assert(luma(miniC) > 0.18 &&
         "mirror shows only its background — the source scene's props never reached it");

  // Prove it is LIVE: a node added to the SOURCE scene must appear in the mirror
  // too, with the mirror camera left untouched (props are re-synced each frame,
  // not reframed). The added quad is larger than the framed ground, so it fills
  // the mirror rect and the delta is unambiguous.
  addQuad(mainScene, "marker", -90, -90, 90, 90, 1.0);
  std::vector<unsigned char> mir1 = vm.frameRGB();
  const double miniDelta = rectDelta(mir0, mir1, W, H, gPx0, gPy0, gPx1, gPy1);
  printf("  mirror rect delta after source node: %.4f\n", miniDelta);
  assert(miniDelta > 0.0 && "mirror did not pick up a node added to the source scene");

  // A 2-D overlay on the SOURCE must NOT be mirrored (a window-space HUD would
  // double-draw onto the inset), but the source's LIGHTS must be, so the minimap
  // is lit like the scene rather than flat.
  {
    // vtkTextActor is a vtkActor2D subclass that renders cleanly with no separate
    // mapper (a bare vtkActor2D logs "No mapper set"); the production filter keys
    // on the vtkActor2D base, so this still exercises it.
    vtkSmartPointer<vtkTextActor> hud = vtkSmartPointer<vtkTextActor>::New();
    hud->SetInput("HUD");
    vm.primary().renderer()->AddActor2D(hud);
    vtkSmartPointer<vtkLight> lamp = vtkSmartPointer<vtkLight>::New();
    lamp->SetPosition(10, 20, 30);
    vm.primary().renderer()->AddLight(lamp);
    vm.render(); // re-syncs the mirror from the source

    bool srcHas2D = false, mirHas2D = false;
    {
      vtkPropCollection *ps = vm.primary().renderer()->GetViewProps();
      vtkCollectionSimpleIterator it;
      ps->InitTraversal(it);
      while (vtkProp *p = ps->GetNextProp(it))
        if (vtkActor2D::SafeDownCast(p))
          srcHas2D = true;
    }
    {
      vtkPropCollection *pm = mini.renderer()->GetViewProps();
      vtkCollectionSimpleIterator it;
      pm->InitTraversal(it);
      while (vtkProp *p = pm->GetNextProp(it))
        if (vtkActor2D::SafeDownCast(p))
          mirHas2D = true;
    }
    assert(srcHas2D && "test setup: the 2-D actor should be on the source renderer");
    assert(!mirHas2D && "a 2-D overlay leaked into the mirror — it will double-draw the HUD");

    bool mirHasLamp = false;
    vtkLightCollection *lc = mini.renderer()->GetLights();
    vtkCollectionSimpleIterator lit;
    lc->InitTraversal(lit);
    while (vtkLight *l = lc->GetNextLight(lit))
      if (l == lamp.Get())
        mirHasLamp = true;
    assert(mirHasLamp && "the source's light did not reach the mirror — the minimap renders flat");

    vm.primary().renderer()->RemoveActor2D(hud); // leave the scene as later tests expect
  }

  // Mirroring a source that does not exist is a loud error.
  bool threw = false;
  try {
    const double r[4] = {0.0, 0.5, 0.3, 0.8};
    vm.addMirrorViewport("ghost", "no_such_viewport", r);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  assert(threw && "mirroring an unknown source viewport must throw");
  assert(!vm.hasViewport("ghost"));

  // ── the double-attach trap is now LOUD ─────────────────────────────────────
  // Re-attaching a scene another viewport already draws is the silent
  // "second setRenderer blanks the first view" bug the SceneRenderer test pins
  // as undiagnosed. Here it must throw rather than blank the primary.
  threw = false;
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

  // Still exactly the three real viewports (main + inset + mini); every failed
  // add left nothing behind.
  assert(vm.viewportNames().size() == 3);

  // ── bad construction arguments are rejected ────────────────────────────────
  threw = false;
  try {
    ViewportManager bad(mainScene, 0, 10);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  assert(threw && "a zero-width manager must be rejected");

  // ── viewport layout persists to / restores from cvc::state ─────────────────
  // Placement round-trips through "<scene prefix>.viewers.<name>.layout.*", so a
  // PiP arrangement can be saved and restored (or driven from a script / peer).
  auto &S = cvc::state::instance(app);
  // object -> state: the setters mirror region/layer/visible.
  vm.primary().setRegion(0.1, 0.2, 0.6, 0.7);
  assert(std::abs(std::stod(S("main_scene.viewers.main.layout.region.x0").value()) - 0.1) < 1e-9 &&
         "setRegion did not write region.x0 to state");
  assert(std::abs(std::stod(S("main_scene.viewers.main.layout.region.y1").value()) - 0.7) < 1e-9);
  inset.setLayer(3);
  assert(S("inset_scene.viewers.inset.layout.layer").value() == std::string("3") &&
         "setLayer did not write layer to state");
  inset.setVisible(false);
  assert(S("inset_scene.viewers.inset.layout.visible").value() == std::string("0") &&
         "setVisible did not write visible to state");

  // state -> object: writing the layout state drives the viewport (a restored or
  // scripted arrangement), including a layer change the window re-syncs at render.
  S("inset_scene.viewers.inset.layout.region.x0").value("0.25");
  S("inset_scene.viewers.inset.layout.region.y0").value("0.30");
  S("inset_scene.viewers.inset.layout.region.x1").value("0.80");
  S("inset_scene.viewers.inset.layout.region.y1").value("0.85");
  S("inset_scene.viewers.inset.layout.visible").value("1");
  S("inset_scene.viewers.inset.layout.layer").value("4");
  double rr[4];
  inset.region(rr);
  assert(std::abs(rr[0] - 0.25) < 1e-9 && std::abs(rr[3] - 0.85) < 1e-9 &&
         "a state write to layout.region did not drive the viewport");
  assert(inset.visible() && inset.layer() == 4 &&
         "a state write to layout.visible/layer did not drive the viewport");
  vm.render(); // must composite cleanly with the state-driven layer
  assert(vm.frameRGB().size() == static_cast<size_t>(W) * H * 3);

  // ── removeViewport: drop a PiP inset at runtime ────────────────────────────
  SceneGraph tmpScene(app, "tmp_scene");
  const double tmpRegion[4] = {0.0, 0.5, 0.3, 0.9}; // top-left; px x[0,96] y[120,216]
  vm.addSceneViewport("tmp", tmpScene, tmpRegion, 1);
  assert(vm.hasViewport("tmp"));
  const size_t nBefore = vm.viewportNames().size();
  vm.removeViewport("tmp");
  assert(!vm.hasViewport("tmp") && "removeViewport did not drop the viewport");
  assert(vm.viewportNames().size() == nBefore - 1);
  // Its scene detached on removal, so it can be drawn again by a new viewport
  // (addSceneViewport would throw the loud double-attach error otherwise).
  vm.addSceneViewport("tmp2", tmpScene, tmpRegion, 1);
  assert(vm.hasViewport("tmp2"));

  // Removing the ACTIVE viewport hands keyboard focus back to the primary.
  vm.routeMouseButton(ViewportManager::MouseButton::Left, true, 48, 168);
  vm.routeMouseButton(ViewportManager::MouseButton::Left, false, 48, 168);
  assert(&vm.viewport("tmp2") == vm.activeViewport());
  vm.removeViewport("tmp2");
  assert(vm.activeViewport() == &vm.primary() &&
         "focus did not fall back to the primary after removing the active viewport");

  // Guards: the primary is not removable, an unknown name throws, and a viewport
  // a mirror still sources is protected until the mirror is removed first.
  bool rthrew = false;
  try {
    vm.removeViewport("main");
  } catch (const std::invalid_argument &) {
    rthrew = true;
  }
  assert(rthrew && "the primary must not be removable");
  rthrew = false;
  try {
    vm.removeViewport("nope");
  } catch (const std::out_of_range &) {
    rthrew = true;
  }
  assert(rthrew && "removing an unknown viewport must throw");

  const double mr[4] = {0.02, 0.02, 0.22, 0.22};
  vm.addMirrorViewport("mini2", "inset", mr, 3);
  rthrew = false;
  try {
    vm.removeViewport("inset");
  } catch (const std::invalid_argument &) {
    rthrew = true;
  }
  assert(rthrew && "a viewport a mirror sources must not be removable");
  vm.removeViewport("mini2"); // remove the mirror first...
  vm.removeViewport("inset"); // ...then the source is removable
  assert(!vm.hasViewport("inset"));

  // activeViewport() falls back to the primary when the state names a viewport
  // that no longer exists (an external edit / a removed name lingering).
  cvc::state::instance(app)("main_scene.active_viewport").value("ghost_gone");
  assert(vm.activeViewport() == &vm.primary() &&
         "activeViewport must fall back to the primary for an unknown active name");

  vm.render(); // still composites cleanly after the removals
  assert(vm.frameRGB().size() == static_cast<size_t>(W) * H * 3);

  printf("cvcgl_viewport: OK\n");
  return 0;
}
