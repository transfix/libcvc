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

#include <cstdio>

#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/geometry_file_io.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/VolRenNode.h>
#include <cvc/gl/VolSliceNode.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_file_io.h>
#include <vtkLight.h>
#include <vtkLightCollection.h>
#include <vtkRenderer.h>

using cvc::ariadne::Scene;
using cvc::ariadne::SceneIsosurface;
using cvc::ariadne::SceneLight;
using cvc::ariadne::SceneNode;
using cvc::ariadne::SceneTFPoint;
using cvc::ariadne::Value;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;

// Observables for the custom-node-type test — namespace scope so the registered
// realizer (process-global) captures nothing (no dangling into a test local).
static int g_custom_ticks = 0;
static double g_custom_radius = -1.0;

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

  // ── volren + volslice realize + the per-frame tick seam ─────────────────────
  {
    SceneGraph sg(app, "vol");
    SceneRenderer view(sg, 128, 128, /*offscreen=*/true, "main");

    // The realizer loads volumes from source.file, so write a small density ball to
    // a temp rawiv (the app ctor registered the rawiv handler). n=32 keeps it fast.
    const std::string volPath = "ari_realize_ball.rawiv";
    {
      const unsigned n = 32;
      cvc::volume vol(app, cvc::dimension(n, n, n), cvc::Float,
                      cvc::bounding_box(-1, -1, -1, 1, 1, 1));
      for (unsigned k = 0; k < n; ++k)
        for (unsigned j = 0; j < n; ++j)
          for (unsigned i = 0; i < n; ++i) {
            const double x = -1.0 + 2.0 * i / (n - 1);
            const double y = -1.0 + 2.0 * j / (n - 1);
            const double z = -1.0 + 2.0 * k / (n - 1);
            const double r = std::sqrt(x * x + y * y + z * z);
            vol(i, j, k, r >= 0.9 ? 0.0 : 1.0 - r / 0.9);
          }
      vol.min(0.0);
      vol.max(1.0);
      cvc::writeVolumeFile(app, vol, volPath);
    }

    Scene scene;
    SceneNode vr;
    vr.id = "vr";
    vr.type = "volren";
    vr.source_file = volPath;
    vr.has_volren = true;
    vr.volren.ambient = 0.6f; // lit without needing a light -> not a black silhouette
    vr.volren.steps = 128;
    {
      SceneIsosurface iso; // a shell at density 0.5
      iso.value = 0.5f;
      iso.opacity = 1.0f;
      iso.color[0] = 1.0f; iso.color[1] = 0.3f; iso.color[2] = 0.2f;
      iso.shininess = 32.0f;
      vr.volren.isosurfaces.push_back(iso);
    }
    scene.nodes.push_back(vr);

    SceneNode vs;
    vs.id = "vs";
    vs.type = "volslice";
    vs.source_file = volPath;
    vs.has_volslice = true;
    vs.volslice.quality = 0.5f;
    { SceneTFPoint p; p.value = 0.0f; p.color[0] = 1.0f; vs.volslice.tf.points.push_back(p); }
    { SceneTFPoint p; p.value = 1.0f; p.color[0] = 1.0f; p.color[3] = 0.8f; vs.volslice.tf.points.push_back(p); }
    scene.nodes.push_back(vs);

    auto realized = cvc::gl::ariadne::realize_scene(sg, scene, "vol");
    std::remove(volPath.c_str()); // the volumes are already read into memory

    printf("== volren/volslice realize + tick seam ==\n");
    chk(realized.volren_ticks.size() == 1, "volren node recorded as a per-frame ticker");
    chk(realized.volslice_ticks.size() == 1, "volslice node recorded as a per-frame ticker");
    auto vrn = std::dynamic_pointer_cast<cvc::gl::VolRenNode>(sg.getGraphics("vr"));
    auto vsn = std::dynamic_pointer_cast<cvc::gl::VolSliceNode>(sg.getGraphics("vs"));
    chk(vrn != nullptr, "type: volren -> a VolRenNode in the graph");
    chk(vsn != nullptr, "type: volslice -> a VolSliceNode in the graph");

    // Drive the tick seam. VolRenNode raycasts on a worker thread, so tick+render
    // until it converges (one initial render sets up GL first, per the volren tests).
    view.render();
    bool converged = false;
    for (int i = 0; i < 400 && !converged; ++i) {
      cvc::gl::ariadne::tick_scene(realized, view.renderer());
      view.render();
      if (auto n = realized.volren_ticks[0].lock())
        converged = n->converged();
    }
    chk(converged, "volren tick seam -> the raycast produced a frame (converged)");
    chk(vsn && vsn->planesRendered() > 0, "volslice tick seam -> slice planes were built");
  }

  // ── two top-level volren nodes share an id -> last wins, no leaked orphan ────
  {
    SceneGraph sg(app, "dup");
    const std::string p = "ari_realize_dup.rawiv";
    {
      const unsigned n = 16;
      cvc::volume vol(app, cvc::dimension(n, n, n), cvc::Float,
                      cvc::bounding_box(-1, -1, -1, 1, 1, 1));
      for (unsigned k = 0; k < n; ++k)
        for (unsigned j = 0; j < n; ++j)
          for (unsigned i = 0; i < n; ++i)
            vol(i, j, k, 0.5);
      vol.min(0.0);
      vol.max(1.0);
      cvc::writeVolumeFile(app, vol, p);
    }
    Scene scene;
    for (int i = 0; i < 2; ++i) { // same id "dup" twice
      SceneNode vr;
      vr.id = "dup";
      vr.type = "volren";
      vr.source_file = p;
      vr.has_volren = true;
      SceneIsosurface iso;
      iso.value = 0.5;
      vr.volren.isosurfaces.push_back(iso);
      scene.nodes.push_back(vr);
    }
    auto realized = cvc::gl::ariadne::realize_scene(sg, scene, "dup");
    std::remove(p.c_str());
    printf("== duplicate top-level id: last wins, orphan unlinked ==\n");
    chk(realized.volren_ticks.size() == 2, "both realize attempts recorded a ticker");
    chk(realized.volren_ticks.size() == 2 && realized.volren_ticks[0].expired(),
        "the first same-id node was unlinked/destroyed (its weak_ptr expired)");
    chk(!realized.volren_ticks.empty() && !realized.volren_ticks.back().expired(),
        "the last same-id node survives");
    chk(std::dynamic_pointer_cast<cvc::gl::VolRenNode>(sg.getGraphics("dup")) != nullptr,
        "getGraphics(id) resolves to the surviving VolRenNode");
  }

  // ── custom scene node type via register_scene_node_type ─────────────────────
  {
    // A capture-free realizer (process-global): builds a geometry node from the
    // embedded bunny, records that it read its props bag, and registers a per-frame
    // tick — exercising the whole extension seam (props in, GraphicsNode out, tick).
    cvc::gl::ariadne::register_scene_node_type(
        "beacon",
        [](SceneGraph &sg, const cvc::ariadne::SceneNode &n, cvc::gl::GraphicsNode *parent,
           cvc::gl::ariadne::RealizedScene &out,
           std::vector<std::string> *) -> std::shared_ptr<cvc::gl::GraphicsNode> {
          g_custom_radius = n.props.num("radius", -1.0); // reads the neutral props bag
          cvc::geometry geom = cvc::read_geometry("beacon.bunny"); // embedded, no file
          std::shared_ptr<cvc::gl::GraphicsNode> node;
          if (parent)
            node = parent->createChild<cvc::gl::GeometryNode>(n.id, geom);
          else
            node = sg.addGraphics(n.id, geom);
          out.custom_ticks.push_back([](vtkRenderer *) { ++g_custom_ticks; });
          return node;
        });

    SceneGraph sg(app, "ext");
    SceneRenderer view(sg, 64, 64, /*offscreen=*/true, "main");

    Scene scene;
    SceneNode b;
    b.id = "b1";
    b.type = "beacon";
    b.has_transform = true;
    b.position[0] = 100; // the custom node's own (local) transform
    b.props.kind = Value::Kind::Map; // as the loader would have captured `radius: 2.5`
    {
      Value radius;
      radius.kind = Value::Kind::Scalar;
      radius.scalar = "2.5";
      b.props.entries.emplace_back("radius", radius);
    }
    // A built-in child under the custom node — proves the shared tail (transform +
    // visibility + children) runs on a custom node just like a built-in one.
    SceneNode child;
    child.id = "truck";
    child.type = "geometry";
    child.source_file = "x.bunny";
    child.has_transform = true;
    child.position[0] = 5; // local to the beacon -> world 105
    b.children.push_back(child);
    scene.nodes.push_back(b);

    const int before = g_custom_ticks;
    auto realized = cvc::gl::ariadne::realize_scene(sg, scene, "ext");

    printf("== custom scene node type (register_scene_node_type) ==\n");
    chk(cvc::gl::ariadne::has_scene_node_type("beacon"), "custom type is registered");
    chk(std::dynamic_pointer_cast<cvc::gl::GeometryNode>(sg.getGraphics("b1")) != nullptr,
        "custom realizer built a GraphicsNode addressable in the graph");
    chk(std::fabs(g_custom_radius - 2.5) < 1e-9, "custom realizer read its props bag (radius=2.5)");
    chk(realized.custom_ticks.size() == 1, "custom realizer registered a per-frame tick");
    cvc::gl::ariadne::tick_scene(realized, view.renderer());
    chk(g_custom_ticks == before + 1, "tick_scene runs the custom per-frame tick");
    // The shared tail (transform + children) runs on a custom node like a built-in.
    auto beacon = sg.getGraphics("b1");
    auto truck = beacon ? beacon->findChildByName("truck") : nullptr;
    chk(truck != nullptr, "a built-in child nests UNDER a custom node type");
    if (truck) {
      const double origin[3] = {0, 0, 0};
      double world[3] = {0, 0, 0};
      truck->localToWorld(origin, world);
      chk(approx(world[0], 105),
          "shared tail composes transforms: custom node [100] ∘ child local [5] -> world [105]");
    }
  }

  // ── verify_scene_customs: the customs: gate for NODE types (cvcGL side) ──────
  {
    using cvc::ariadne::CustomRequirement;
    using K = cvc::ariadne::CustomRequirement::Kind;
    printf("== verify_scene_customs: node customs gate ==\n");

    // "beacon" was registered above; an unregistered node type is not.
    cvc::ariadne::LoadResult good;
    good.customs.push_back({K::Node, "beacon", true});
    chk(cvc::gl::ariadne::verify_scene_customs(good, nullptr),
        "required node custom that IS registered -> verify passes");

    cvc::ariadne::LoadResult bad;
    bad.customs.push_back({K::Node, "unregistered_node_zzz", true});
    std::vector<std::string> errs;
    chk(!cvc::gl::ariadne::verify_scene_customs(bad, &errs),
        "required node custom that is MISSING -> verify fails");
    chk(!errs.empty(), "... and appends an error message");

    cvc::ariadne::LoadResult opt;
    opt.customs.push_back({K::Node, "unregistered_optional", false});
    std::vector<std::string> notes;
    chk(cvc::gl::ariadne::verify_scene_customs(opt, &notes),
        "non-required missing node custom -> verify passes (degrade)");
    chk(!notes.empty(), "... with a note logged");

    cvc::ariadne::LoadResult widgetOnly; // widget/block customs are the loader's job
    widgetOnly.customs.push_back({K::Widget, "anything", true});
    chk(cvc::gl::ariadne::verify_scene_customs(widgetOnly, nullptr),
        "verify_scene_customs ignores widget customs (checked at load) -> passes");
  }

  printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "PASSED", fails, fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
