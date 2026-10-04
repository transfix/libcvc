// Prove cvc::gl::ariadne::realize_scene turns a backend-neutral Scene spec into the
// right live cvc::gl::SceneGraph, at the level the loader gtests can't reach (they
// only check parsing — no VTK). Runs offscreen, inspecting the realized vtkLights and
// node world transforms. Covers the §9 realize invariants an adversarial review
// flagged as untested:
//   * the light BATCH bakes a light's moved world position (not the origin),
//   * directional -> non-positional (az/el) vs spot -> positional (pos/target),
//   * a StageLighting rig actually adds lights,
//   * a nested node's transform is LOCAL (world = parent ∘ local).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h>
#include <cvc/core/state.h>       // clock test: read/write the bound sim.* keys
#include <cvc/core/world_clock.h> // clock test: app.world_clock() drive via tick_scene
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/geometry_file_io.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/VolRenNode.h>
#include <cvc/gl/VolSliceNode.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/image/image.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_file_io.h>
#include <vtkLight.h>
#include <vtkLightCollection.h>
#include <vtkRenderer.h>

using cvc::ariadne::Scene;
using cvc::ariadne::SceneHeightColorBand;
using cvc::ariadne::SceneHeightLayer;
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
    SceneLight key;    // a spot whose trailing setters are all no-ops (defaults), so
    key.id = "key";    // ONLY the light batch can bake its moved position — this is
    key.kind = "spot"; // exactly the batch-removal regression.
    key.pos[0] = 10;
    key.pos[1] = 20;
    key.pos[2] = 30;
    key.target[0] = 0;
    key.target[1] = 0;
    key.target[2] = 0;
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
    chk(approx(s.posLightPos[0], 10) && approx(s.posLightPos[1], 20) &&
            approx(s.posLightPos[2], 30),
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
    std::shared_ptr<cvc::gl::GraphicsNode> child =
        group ? group->findChildByName("truck") : nullptr;
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
      iso.color[0] = 1.0f;
      iso.color[1] = 0.3f;
      iso.color[2] = 0.2f;
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
    {
      SceneTFPoint p;
      p.value = 0.0f;
      p.color[0] = 1.0f;
      vs.volslice.tf.points.push_back(p);
    }
    {
      SceneTFPoint p;
      p.value = 1.0f;
      p.color[0] = 1.0f;
      p.color[3] = 0.8f;
      vs.volslice.tf.points.push_back(p);
    }
    scene.nodes.push_back(vs);

    // A `type: volume` node (the VTK GPU mapper) with a transfer function + shading toggle —
    // the `volume:` block, fed to VolumeNode::setTransferFunction/setShading by the realizer.
    SceneNode vn;
    vn.id = "vn";
    vn.type = "volume";
    vn.source_file = volPath;
    vn.has_volume = true;
    vn.volume.has_shaded = true;
    vn.volume.shaded = false; // VolumeNode defaults shading TRUE -> this asserts a real flip
    {
      SceneTFPoint p; // low end: a translucent blue
      p.value = 0.0;
      p.color[0] = 0.00f;
      p.color[1] = 0.42f;
      p.color[2] = 0.78f;
      p.color[3] = 0.0f;
      vn.volume.tf.points.push_back(p);
    }
    {
      SceneTFPoint p; // high end: a darker blue at half opacity
      p.value = 1.0;
      p.color[0] = 0.01f;
      p.color[1] = 0.09f;
      p.color[2] = 0.22f;
      p.color[3] = 0.5f;
      vn.volume.tf.points.push_back(p);
    }
    scene.nodes.push_back(vn);

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

    auto vnn = std::dynamic_pointer_cast<cvc::gl::VolumeNode>(sg.getGraphics("vn"));
    chk(vnn != nullptr, "type: volume -> a VolumeNode in the graph");
    if (vnn) {
      chk(!vnn->getShading(), "volume: { shaded: false } flips VolumeNode's default-true shading");
      // 2 TF points -> color table [s,r,g,b, s,r,g,b]; assert OUR colour reached the mapper
      // (not the default black->white ramp, which would have 0 in the green slot).
      auto col = vnn->getTransferFunctionColorTable();
      chk(col.size() == 8, "volume transfer_function -> 2 colour control points applied");
      chk(col.size() == 8 && std::abs(col[2] - 0.42) < 1e-6,
          "volume transfer_function -> our green channel (0.42) reached the mapper");
      auto op = vnn->getTransferFunctionOpacityTable();
      chk(op.size() == 4 && std::abs(op[3] - 0.5) < 1e-6,
          "volume transfer_function -> our opacity (0.5) reached the mapper");
    }
  }

  // ── a procedural heightfield source -> a displaced grid mesh (verts/uvs/colours) ──
  {
    SceneGraph sg(app, "hf");
    SceneRenderer view(sg, 64, 64, /*offscreen=*/true, "main");
    Scene scene;
    SceneNode t;
    t.id = "terrain";
    t.type = "geometry";
    t.has_heightfield = true;
    t.heightfield.size = 100.0f;
    t.heightfield.resolution = 25; // ODD -> a vertex lands exactly at the centre (0,0)
    {
      SceneHeightLayer d; // a central dome peaking at z = amplitude
      d.kind = "dome";
      d.amplitude = 20.0f;
      d.radius = 30.0f;
      t.heightfield.layers.push_back(d);
    }
    {
      SceneHeightColorBand b; // low band (green)
      b.max_height = 5.0f;
      b.color[1] = 1.0f;
      t.heightfield.colors.push_back(b);
    }
    {
      SceneHeightColorBand b; // high band (red)
      b.max_height = 100.0f;
      b.color[0] = 1.0f;
      t.heightfield.colors.push_back(b);
    }
    scene.nodes.push_back(t);
    cvc::gl::ariadne::realize_scene(sg, scene, "hf");

    printf("== heightfield geometry source ==\n");
    auto gn = std::dynamic_pointer_cast<cvc::gl::GeometryNode>(sg.getGraphics("terrain"));
    chk(gn != nullptr, "type: geometry + heightfield -> a GeometryNode in the graph");
    if (gn && gn->getGeometry()) {
      const cvc::geometry &geom = *gn->getGeometry();
      chk(geom.const_points().size() == 25u * 25u, "heightfield -> res*res vertices");
      chk(geom.const_tris().size() == 2u * 24u * 24u, "heightfield -> 2*(res-1)^2 triangles");
      chk(geom.const_uvs().size() == 25u * 25u, "heightfield -> a uv per vertex");
      chk(geom.const_colors().size() == 25u * 25u, "heightfield colours -> a colour per vertex");
      double maxz = -1e30;
      for (const auto &p : geom.const_points())
        maxz = std::max(maxz, p[2]);
      chk(std::abs(maxz - 20.0) < 1e-6,
          "heightfield dome -> the centre vertex reaches the amplitude");
    }
  }

  // ── material: { texture } -> an image is read + applied through the mesh UVs ──
  {
    SceneGraph sg(app, "tex");
    SceneRenderer view(sg, 64, 64, /*offscreen=*/true, "main");
    const std::string imgPath = "ari_realize_tex.png"; // a tiny RGBA image the realizer reads back
    {
      cvc::image img(4, 4); // 4x4 RGBA u8 (ctor defaults)
      cvc::write_image(img, imgPath);
    }
    Scene scene;
    SceneNode t;
    t.id = "ground";
    t.type = "geometry";
    t.has_heightfield = true; // a heightfield carries UVs, so the texture samples
    t.heightfield.size = 10.0f;
    t.heightfield.resolution = 4;
    t.has_material = true;
    t.material_texture = imgPath;
    scene.nodes.push_back(t);
    std::vector<std::string> warnings;
    cvc::gl::ariadne::realize_scene(sg, scene, "tex", &warnings);
    std::remove(imgPath.c_str());

    printf("== material texture ==\n");
    auto gn = std::dynamic_pointer_cast<cvc::gl::GeometryNode>(sg.getGraphics("ground"));
    chk(gn != nullptr, "textured geometry -> a GeometryNode in the graph");
    bool texWarn = false;
    for (const std::string &w : warnings)
      if (w.find("texture") != std::string::npos)
        texWarn = true;
    chk(!texWarn, "a valid texture URI -> no warning (image read + applied)");
  }

  // ── a bad texture URI warns but still creates the node (graceful, never throws) ──
  {
    SceneGraph sg(app, "texbad");
    SceneRenderer view(sg, 64, 64, /*offscreen=*/true, "main");
    Scene scene;
    SceneNode t;
    t.id = "ground";
    t.type = "geometry";
    t.has_heightfield = true;
    t.heightfield.size = 10.0f;
    t.heightfield.resolution = 4;
    t.has_material = true;
    t.material_texture = "ari_realize_nope.png"; // does not exist
    scene.nodes.push_back(t);
    std::vector<std::string> warnings;
    cvc::gl::ariadne::realize_scene(sg, scene, "texbad", &warnings);
    auto gn = std::dynamic_pointer_cast<cvc::gl::GeometryNode>(sg.getGraphics("ground"));
    chk(gn != nullptr, "bad texture -> the GeometryNode is still created (graceful)");
    bool texWarn = false;
    for (const std::string &w : warnings)
      if (w.find("texture") != std::string::npos)
        texWarn = true;
    chk(texWarn, "a bad texture URI -> a warning (never a throw)");
  }

  // ── §9 shaders: a declarative shader: block applies a preset / warns on an unknown one ──
  {
    cvc::gl::ariadne::register_default_shader_presets();
    printf("== shader DSL (presets + inline) ==\n");
    chk(cvc::gl::ariadne::has_shader_preset("terrain_bump"),
        "default preset terrain_bump registered");
    chk(cvc::gl::ariadne::has_shader_preset("bark"), "default preset bark registered");

    SceneGraph sg(app, "shd");
    SceneRenderer view(sg, 64, 64, /*offscreen=*/true, "main");
    Scene scene;
    SceneNode g; // a known preset applies cleanly
    g.id = "ground";
    g.type = "geometry";
    g.has_heightfield = true;
    g.heightfield.size = 10.0f;
    g.heightfield.resolution = 4;
    g.shader.present = true;
    g.shader.preset = "terrain_bump";
    SceneNode bad; // an unknown preset warns but still realizes
    bad.id = "bad";
    bad.type = "geometry";
    bad.source_primitive = "plane";
    bad.plane_size = 4.0f;
    bad.shader.present = true;
    bad.shader.preset = "does_not_exist";
    scene.nodes.push_back(g);
    scene.nodes.push_back(bad);
    std::vector<std::string> warnings;
    cvc::gl::ariadne::realize_scene(sg, scene, "shd", &warnings);
    chk(sg.getGraphics("ground") != nullptr, "a node with a known shader preset realizes");
    chk(sg.getGraphics("bad") != nullptr, "an unknown shader preset still realizes the node");
    bool known_warned = false, unknown_warned = false;
    for (const std::string &w : warnings) {
      if (w.find("unknown shader preset 'does_not_exist'") != std::string::npos)
        unknown_warned = true;
      if (w.find("terrain_bump") != std::string::npos)
        known_warned = true;
    }
    chk(unknown_warned, "an unknown shader preset -> a warning");
    chk(!known_warned, "a known shader preset -> no warning");
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
          g_custom_radius = n.props.num("radius", -1.0);           // reads the neutral props bag
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
    b.position[0] = 100;             // the custom node's own (local) transform
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

  // ── §9 time: a `clock:` scene drives app.world_clock() through tick_scene ────
  {
    // A clock-only scene (defaults: bind sim.speed / sim.paused, publish sim.time). realize_scene
    // resolves the key paths with the host prefix; tick_scene then steers app.world_clock() from
    // them each frame. We inject a fixed wall_dt (the deterministic-capture path) of one quantum,
    // so the arithmetic is exact: scale 1 → +1 step/quantum, paused → none, scale 2 → +2.
    SceneGraph sg(app, "clk");
    SceneRenderer view(sg, 64, 64, /*offscreen=*/true, "main");
    Scene scene;
    scene.clock.present = true; // defaults carry sim.speed / sim.paused / sim.time
    auto realized = cvc::gl::ariadne::realize_scene(sg, scene, "clk");

    printf("== scene clock drives app.world_clock() ==\n");
    chk(realized.clock.present, "clock declaration carried into the realized scene");
    chk(realized.clock.speed_key == "clk.sim.speed", "clock speed_key resolved with host prefix");
    chk(realized.clock.paused_key == "clk.sim.paused",
        "clock paused_key resolved with host prefix");
    chk(realized.clock.time_key == "clk.sim.time", "clock time_key resolved with host prefix");

    cvc::world_clock &wc = app.world_clock();
    wc.reset(); // deterministic start (shared per-app clock; no other section declares one)
    const double q = wc.fixed_dt();
    for (int i = 0; i < 4; ++i)
      cvc::gl::ariadne::tick_scene(realized, view.renderer(), q); // 4 quanta of wall time, scale 1
    chk(approx(wc.t(), 4.0 * q), "scale 1: four injected quanta advance the clock by 4·fixed_dt");
    chk(approx(cvc::state::instance(app)("clk.sim.time").value<double>(), wc.t()),
        "sim.time publishes the live world_clock t()");

    cvc::state::instance(app)("clk.sim.paused").value(1); // the sim_transport checkbox 'Paused'
    const double tp = wc.t();
    cvc::gl::ariadne::tick_scene(realized, view.renderer(), q);
    chk(approx(wc.t(), tp), "sim.paused freezes the clock (banks no time)");

    cvc::state::instance(app)("clk.sim.paused").value(0);
    cvc::state::instance(app)("clk.sim.speed").value(2.0); // the Speed slider at 2×
    const double t2 = wc.t();
    cvc::gl::ariadne::tick_scene(realized, view.renderer(), q); // q wall × 2 = 2 quanta world
    chk(wc.t() > t2 + 1.5 * q, "sim.speed=2 advances ~two quanta per quantum of wall time");
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

  // ── §9 scene enrichments: plane primitive, fit, specular material, shadow res, rig tuning ──
  {
    using cvc::gl::GeometryNode;
    SceneGraph sg(app, "enrich");
    SceneRenderer view(sg, 64, 64, /*offscreen=*/true, "enrich");
    Scene scene;

    // A procedural `plane` primitive — a geometry node with NO file source.
    SceneNode ground;
    ground.id = "ground";
    ground.type = "geometry";
    ground.source_primitive = "plane";
    ground.plane_size = 20.0f; // spans [-10,10]^2 at z=0
    ground.has_material = true;
    ground.use_single_color = true;
    ground.color[0] = 0.3f;
    ground.color[1] = 0.3f;
    ground.color[2] = 0.35f;
    ground.has_specular = true; // exercises the setSpecular/Power forwarding (no crash / node made)
    ground.specular = 0.25f;
    ground.specular_power = 24.0f;
    scene.nodes.push_back(ground);

    // A `fit`ted mesh: the embedded bunny, Y-up -> Z-up, base on z=0, tallest extent == 100.
    SceneNode bunny;
    bunny.id = "bunny";
    bunny.type = "geometry";
    bunny.source_file = "x.bunny";
    bunny.has_fit = true;
    bunny.fit_up_y = true;
    bunny.fit_height = 100.0f;
    scene.nodes.push_back(bunny);

    // A tuned StageLighting rig (stage + key + fill + ambient) — must still add lights, no crash.
    SceneLight rig;
    rig.rig = "three_point";
    rig.has_stage = true;
    rig.stage_center[2] = 50.0f;
    rig.stage_radius = 62.0f;
    rig.has_key = true;
    rig.key_intensity = 1.9f;
    rig.key_azimuth = -38.0f;
    rig.key_elevation = 52.0f;
    rig.key_cone = 34.0f;
    rig.has_fill = true;
    rig.fill = 0.85f;
    rig.has_rig_ambient = true;
    rig.rig_ambient = 0.4f;
    scene.lights.push_back(rig);

    scene.has_shadows = true;
    scene.shadows_enabled = true;
    scene.has_shadow_resolution = true;
    scene.shadow_resolution = 2048;
    scene.has_shadow_interval = true;
    scene.shadow_interval = 1;

    cvc::gl::ariadne::realize_scene(sg, scene, "enrich");
    printf("== §9 enrichments: plane / fit / specular / shadow-res / rig tuning ==\n");

    auto gnode = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("ground"));
    chk(gnode != nullptr, "plane primitive -> a GeometryNode with no file source");
    const cvc::geometry *pg = gnode ? gnode->getGeometry() : nullptr;
    chk(pg && pg->points().size() == 4 && pg->tris().size() == 2,
        "plane geometry is a 4-vert / 2-tri quad");
    if (pg && pg->points().size() == 4) {
      double lo = 1e30, hi = -1e30;
      for (const auto &p : pg->points()) {
        lo = std::min(lo, p[0]);
        hi = std::max(hi, p[0]);
      }
      chk(approx(lo, -10.0) && approx(hi, 10.0), "plane spans [-size/2, +size/2] in X");
    }

    auto bnode = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("bunny"));
    const cvc::geometry *bg = bnode ? bnode->getGeometry() : nullptr;
    chk(bg != nullptr && bg->points().size() > 100, "fit: the bunny mesh realized");
    if (bg && !bg->points().empty()) {
      double zlo = 1e30, zhi = -1e30, ext[3] = {0, 0, 0};
      double lo3[3] = {1e30, 1e30, 1e30}, hi3[3] = {-1e30, -1e30, -1e30};
      for (const auto &p : bg->points()) {
        for (int k = 0; k < 3; ++k) {
          lo3[k] = std::min(lo3[k], p[k]);
          hi3[k] = std::max(hi3[k], p[k]);
        }
      }
      zlo = lo3[2];
      zhi = hi3[2];
      for (int k = 0; k < 3; ++k)
        ext[k] = hi3[k] - lo3[k];
      const double tallest = std::max({ext[0], ext[1], ext[2]});
      chk(approx(zlo, 0.0), "fit: base sits on z=0");
      chk(approx(tallest, 100.0), "fit: tallest extent scaled to height (100)");
      (void)zhi;
    }

    chk(sg.shadowsEnabled(), "shadows enabled");
    chk(sg.shadowResolution() == 2048, "shadow resolution applied from the DSL");
    chk(sg.shadowUpdateInterval() == 1, "shadow update interval applied from the DSL");
    chk(survey(view.renderer()).positional + survey(view.renderer()).directional >= 1,
        "tuned rig still adds lights");
  }

  printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "PASSED", fails, fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
