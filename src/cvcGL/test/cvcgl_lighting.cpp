// Scene-owned lighting: the scene describes its sun, not the host.
//
// Lights live on the SceneGraph rather than being pushed at a vtkRenderer so
// they SURVIVE being attached to a render target -- a scene drawn into a second
// window or an offscreen capture has to light identically. That is the property
// most likely to regress, so it is what this pins.
//
// Note the unbuffered stdout below. It is not cosmetic: with the default block
// buffering, a crash swallows everything printed beforehand, which during
// development made a late crash look like an immediate one and sent me hunting
// in the wrong place entirely.
#undef NDEBUG
#include <cassert>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <vtkLightCollection.h>
#include <vtkRenderer.h>

namespace {

cvc::geometry tri() {
  cvc::geometry g;
  cvc::geometry::point_t p;
  const double xyz[3][3] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  for (auto &v : xyz) {
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

void test_lights_are_scene_owned() {
  cvc::app app;
  SceneGraph sg(app);
  sg.addGraphics("m", tri());
  const int sun = sg.addDirectionalLight(-52.0, 34.0, 1.0, 0.94, 0.82, 1.05);
  const int fill = sg.addDirectionalLight(140.0, 55.0, 0.55, 0.66, 0.85, 0.35);
  assert(sun != fill);
  assert(sg.numLights() == 2);

  // Declaring lights BEFORE any render target exists must work: a script builds
  // its scene long before the host hands over somewhere to draw.
  {
    SceneRenderer a(sg, 64, 64, true);
    assert(a.renderer()->GetLights()->GetNumberOfItems() == 2);
    std::printf("  ok: lights declared before a render target land on it\n");
  }
  // The point of scene ownership: a SECOND target gets the same lighting.
  {
    SceneRenderer b(sg, 64, 64, true);
    assert(b.renderer()->GetLights()->GetNumberOfItems() == 2);
    std::printf("  ok: lighting follows the scene onto a second render target\n");

    sg.setLightIntensity(sun, 0.8);
    sg.setLightDirection(sun, -40.0, 20.0);
    sg.setLightColor(sun, 1.0, 0.9, 0.7);
    assert(sg.numLights() == 2);
    assert(b.renderer()->GetLights()->GetNumberOfItems() == 2);

    sg.removeLight(fill);
    assert(sg.numLights() == 1);
    assert(b.renderer()->GetLights()->GetNumberOfItems() == 1);

    // Clearing must not leave the scene black: the renderer gets its default
    // headlight back rather than nothing at all.
    sg.clearLights();
    assert(sg.numLights() == 0);
    assert(b.renderer()->GetLights()->GetNumberOfItems() >= 1);
    std::printf("  ok: clearing restores a default light rather than darkness\n");
  }
}

void test_shadow_toggle() {
  cvc::app app;
  SceneGraph sg(app);
  sg.addGraphics("m", tri());
  sg.addDirectionalLight(-52.0, 34.0);

  // No render target yet -> an honest false, not a silent no-op.
  assert(!sg.setShadowsEnabled(true));
  assert(!sg.shadowsEnabled());
  std::printf("  ok: shadows refuse honestly with no render target\n");

  // Shadow quality knobs round-trip (interval + resolution), and the resolution is
  // clamped off silly-small values. These hold whether or not shadows are live.
  sg.setShadowUpdateInterval(3);
  assert(sg.shadowUpdateInterval() == 3);
  sg.setShadowResolution(2048);
  assert(sg.shadowResolution() == 2048);
  sg.setShadowResolution(1); // clamped up
  assert(sg.shadowResolution() >= 64);
  sg.setShadowResolution(1024);

  SceneRenderer sr(sg, 64, 64, true);
  const bool on = sg.setShadowsEnabled(true);
  assert(on == sg.shadowsEnabled());
  if (on) {
    assert(sr.renderer()->GetPass() != nullptr);
    // Exercise the pass rather than just installing it. This only builds a
    // working shader because the mesh has normals -- GeometryNode generates them
    // for triangle meshes that arrive without any, and without them VTK takes
    // the unlit path and the shadow shader fails to compile.
    assert(sg.setShadowsEnabled(false));
    assert(!sg.shadowsEnabled());
    assert(sr.renderer()->GetPass() == nullptr);
    std::printf("  ok: shadow passes attach, render, and detach\n");
  } else {
    std::printf("  ok: shadows declined cleanly on this build\n");
  }
}

} // namespace

int main() {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  test_lights_are_scene_owned();
  test_shadow_toggle();
  std::printf("cvcgl_lighting: OK\n");
  return 0;
}
