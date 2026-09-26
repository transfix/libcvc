// Prove cvc::gl::ariadne::realize_scene turns a backend-neutral Scene spec into the
// right live cvc::gl::SceneGraph, at the level the loader gtests can't reach (they
// only check parsing — no VTK). Runs offscreen, inspecting the realized vtkLights and
// node world transforms. Covers the §9 realize invariants an adversarial review
// flagged as untested:
//   * the light BATCH bakes a light's moved world position (not the origin),
//   * directional -> non-positional (az/el) vs spot -> positional (pos/target),
//   * a StageLighting rig actually adds lights,
//   * a nested node's transform is LOCAL (world = parent ∘ local).
#include <cmath>
#include <cstdio>

#include <cvc/ariadne/scene.h>
#include <cvc/core/app.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <vtkLight.h>
#include <vtkLightCollection.h>
#include <vtkRenderer.h>

using cvc::ariadne::Scene;
using cvc::ariadne::SceneLight;
using cvc::ariadne::SceneNode;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;

static int fails = 0;
static void chk(bool ok, const std::string &w) {
  printf("  %s  %s\n", ok ? "PASS" : "FAIL", w.c_str());
  if (!ok)
    ++fails;
}
static bool approx(double a, double b) { return std::fabs(a - b) < 1e-3; }

// Walk the renderer's lights, counting positional vs directional and capturing the
// (single) positional light's position.
struct LightSurvey {
  int positional = 0;
  int directional = 0;
  double posLightPos[3] = {0, 0, 0};
};
static LightSurvey survey(vtkRenderer *r) {
  LightSurvey s;
  vtkLightCollection *lc = r->GetLights();
  lc->InitTraversal();
  while (vtkLight *l = lc->GetNextItem()) {
    if (l->GetPositional()) {
      ++s.positional;
      const double *p = l->GetPosition();
      s.posLightPos[0] = p[0];
      s.posLightPos[1] = p[1];
      s.posLightPos[2] = p[2];
    } else {
      ++s.directional;
    }
  }
  return s;
}

int main() {
  cvc::app app;
  app.properties("system.log_verbosity", "0");

  // ── lights: batch position bake + directional-vs-spot routing ───────────────
  {
    SceneGraph sg(app, "lights");
    SceneRenderer view(sg, 128, 128, /*offscreen=*/true, "main");

    Scene scene;
    SceneLight key; // a spot whose trailing setters are all no-ops (defaults), so
    key.id = "key"; // ONLY the light batch can bake its moved position — this is
    key.kind = "spot";                            // exactly the batch-removal regression.
    key.pos[0] = 10; key.pos[1] = 20; key.pos[2] = 30;
    key.target[0] = 0; key.target[1] = 0; key.target[2] = 0;
    key.cone = 30; // == LightNode's internal default (a no-op setCone)
    scene.lights.push_back(key);

    SceneLight sun; // a directional sun — az/el, must come out NON-positional
    sun.id = "sun";
    sun.kind = "directional";
    sun.azimuth = 45;
    sun.elevation = 60;
    scene.lights.push_back(sun);

    cvc::gl::ariadne::realize_scene(sg, scene, "lights");
    sg.processEvents();

    LightSurvey s = survey(view.renderer());
    printf("== lights realize: batch bake + kind routing ==\n");
    printf("     positional=%d directional=%d posLightPos=(%g,%g,%g)\n", s.positional,
           s.directional, s.posLightPos[0], s.posLightPos[1], s.posLightPos[2]);
    chk(s.positional == 1, "spot -> exactly one positional vtkLight");
    chk(s.directional == 1, "directional -> exactly one non-positional vtkLight");
    chk(approx(s.posLightPos[0], 10) && approx(s.posLightPos[1], 20) && approx(s.posLightPos[2], 30),
        "light batch baked the spot's moved position (10,20,30), not the origin");
  }

  // ── nesting: a child's transform is LOCAL (world = parent ∘ local) ──────────
  {
    SceneGraph sg(app, "geo");
    SceneRenderer view(sg, 128, 128, /*offscreen=*/true, "main");

    Scene scene;
    SceneNode convoy; // a transformed group...
    convoy.id = "convoy";
    convoy.type = "group";
    convoy.has_transform = true;
    convoy.position[0] = 100;
    SceneNode truck; // ...with a child at a LOCAL offset (embedded bunny, no file)
    truck.id = "truck";
    truck.type = "geometry";
    truck.source_file = "x.bunny";
    truck.has_transform = true;
    truck.position[0] = 5;
    convoy.children.push_back(truck);
    scene.nodes.push_back(convoy);

    // A rig needs geometry bounds, so add one here too.
    SceneLight rig;
    rig.id = "studio";
    rig.rig = "three_point";
    scene.lights.push_back(rig);

    cvc::gl::ariadne::realize_scene(sg, scene, "geo");
    sg.processEvents();

    printf("== nesting: local transform composes through the parent ==\n");
    auto group = sg.getGraphics("convoy");
    chk(group != nullptr, "top-level group is registered and found");
    std::shared_ptr<cvc::gl::GraphicsNode> child = group ? group->findChildByName("truck") : nullptr;
    chk(child != nullptr, "child 'truck' is nested UNDER the convoy group");
    if (child) {
      const double origin[3] = {0, 0, 0};
      double world[3] = {0, 0, 0};
      child->localToWorld(origin, world);
      printf("     truck world origin = (%g,%g,%g)\n", world[0], world[1], world[2]);
      chk(approx(world[0], 105), "child local [5,0,0] under parent [100,0,0] -> world [105,..]");
    }

    printf("== rig: a StageLighting preset adds lights ==\n");
    chk(survey(view.renderer()).positional + survey(view.renderer()).directional >= 1,
        "rig: three_point realized at least one light");
  }

  printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "PASSED", fails, fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
