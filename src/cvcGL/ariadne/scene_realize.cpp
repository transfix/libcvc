#include <cvc/gl/ariadne/scene_realize.h>

#include <exception>
#include <memory>

#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/scene.h>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/geometry_file_io.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/LightNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/StageLighting.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/volume/bounding_box.h>
#include <cvc/volume/volume.h>

namespace cvc {
namespace gl {
namespace ariadne {

namespace {

void warn(std::vector<std::string> *w, const std::string &m) {
  if (w)
    w->push_back(m);
}

// Apply the shared GraphicsNode transform. Material is GeometryNode-only, handled by
// the caller before this; visibility is handled by apply_visibility below.
void apply_transform(GraphicsNode &node, const cvc::ariadne::SceneNode &n) {
  if (n.has_transform) {
    node.setPosition(n.position[0], n.position[1], n.position[2]);
    node.setRotation(n.rotation[0], n.rotation[1], n.rotation[2]);
    node.setScale(n.scale[0], n.scale[1], n.scale[2]);
  }
}

// Set the node's initial visibility, and — for a bound node — record a binding the
// host will poll. Visibility is driven through the node's OWN `.visible` state key
// (not a direct setVisible), so cvc::state stays authoritative (§9.1) and the node's
// state_object machinery performs the actor flip. `bind_prefix` matches the widget
// Runtime's prefix so a `visible:`/`bind:` pair on the same path share one key.
void apply_visibility(GraphicsNode &node, const cvc::ariadne::SceneNode &n,
                      cvc::app &app, const std::string &bind_prefix,
                      std::vector<cvc::ariadne::SceneVisibilityBinding> &binds) {
  const std::string target = node.stateName("visible");
  if (n.visible_bind.empty()) {
    // Literal `visible: true|false` (or default): write the node key once.
    cvc::ariadne::write<int>(app, target, n.visible_default ? 1 : 0);
    return;
  }
  // Bound `visible: <path>` — resolve, set the node's OWN key from the live source
  // (falling back to the node default while the source is unset), and record the
  // binding for per-frame sync. Read the source WITHOUT seeding it: the scene bind is
  // a follower, so the widget that owns the key (its `def:`) is the sole seeder — the
  // node default never pre-empts it in the shared key.
  const std::string source = cvc::ariadne::resolve_bind(bind_prefix, n.visible_bind);
  const int v0 = cvc::ariadne::read_or<int>(app, source, n.visible_default ? 1 : 0);
  cvc::ariadne::write<int>(app, target, v0);
  binds.push_back({source, target, n.visible_default});
}

void realize_node(SceneGraph &sg, const cvc::ariadne::SceneNode &n,
                  const std::string &bind_prefix, RealizedScene &out,
                  std::vector<std::string> *warnings) {
  std::shared_ptr<GraphicsNode> node;

  if (n.type == "geometry") {
    if (n.source_file.empty()) {
      warn(warnings, "ari: scene node '" + n.id + "' (geometry) has no source.file");
      return;
    }
    cvc::geometry geom;
    try {
      geom = cvc::read_geometry(n.source_file);
    } catch (const std::exception &e) {
      warn(warnings, "ari: scene node '" + n.id + "': " + e.what());
      return;
    }
    node = sg.addGraphics(n.id, geom);
    if (auto g = std::dynamic_pointer_cast<GeometryNode>(node)) {
      if (n.has_material) {
        g->setUseSingleColor(true);
        g->setColor(n.color[0], n.color[1], n.color[2]);
        g->setAmbient(n.ambient);
        g->setDiffuse(n.diffuse);
      }
    }
  } else if (n.type == "volume") {
    if (n.source_file.empty()) {
      warn(warnings, "ari: scene node '" + n.id + "' (volume) has no source.file");
      return;
    }
    std::shared_ptr<VolumeNode> vnode;
    try {
      cvc::volume vol(sg.appContext(), n.source_file); // reads on construct; throws on a bad file
      vnode = sg.addGraphics(n.id, vol); // sets a default grayscale transfer function -> renders
    } catch (const std::exception &e) {
      warn(warnings, "ari: scene node '" + n.id + "': " + e.what());
      return;
    }
    node = vnode;
    if (n.has_material) {
      // A volume's colour comes from a transfer function, not a single actor colour,
      // so `color[3]` has no analog here (a TF spec is a follow-up); only the lighting
      // coefficients map. Apply only when the node is styled, so an unstyled volume
      // keeps VolumeNode's tuned defaults.
      vnode->setAmbient(n.ambient);
      vnode->setDiffuse(n.diffuse);
    }
  } else if (n.type == "group") {
    node = sg.addGraphics(n.id); // empty hierarchy node
  } else {
    // volren/volslice/light realization is a follow-up (§9): volren/volslice need a
    // per-frame tick() host seam + transfer-function/isosurface params the spec does
    // not carry yet; declare a light in the `lights:` array instead. The spec still
    // parses and round-trips so the document stays valid.
    warn(warnings, "ari: scene node '" + n.id + "' type '" + n.type +
                       "' not realized yet (geometry/group/volume; volren/volslice/light are a follow-up)");
    return;
  }

  if (!node)
    return;
  apply_transform(*node, n);
  apply_visibility(*node, n, sg.appContext(), bind_prefix, out.visibility);

  // Children are realized as top-level nodes for now; true parent nesting (a
  // <parent>.children.<child> path) rides on the §11 path binding — a follow-up.
  for (const auto &c : n.children)
    realize_node(sg, c, bind_prefix, out, warnings);
}

LightNode::Kind light_kind(const std::string &k) {
  if (k == "spot")
    return LightNode::Kind::Spot;
  if (k == "fill")
    return LightNode::Kind::Fill;
  return LightNode::Kind::Directional; // default + anything unrecognized
}

// Map the DSL's snake_case rig name to a StageLighting preset. Mapped explicitly —
// StageLighting::presetName() is hyphenated ("three-point"), so it would NOT match a
// `.ari` `rig: three_point`; do not "simplify" this into presetName().
StageLighting::Preset preset_from(const std::string &r) {
  if (r == "overhead")
    return StageLighting::Preset::Overhead;
  if (r == "dramatic")
    return StageLighting::Preset::Dramatic;
  if (r == "flat")
    return StageLighting::Preset::Flat;
  return StageLighting::Preset::ThreePoint; // "three_point" + default
}

void realize_light(SceneGraph &sg, const cvc::ariadne::SceneLight &l, RealizedScene &out,
                   std::vector<std::string> *warnings) {
  if (!l.rig.empty()) {
    // A named StageLighting rig — a whole lighting SETUP, not one light. Frame it to
    // the realized geometry (lights are excluded from the bounds). The rig must
    // outlive the render loop: ~StageLighting removes its lights, so RealizedScene
    // owns it (the host keeps RealizedScene alive).
    auto rig = std::make_unique<StageLighting>(sg);
    const cvc::bounding_box bb = sg.computeGraphicsBounds();
    if (!bb.isNull())
      rig->frameBounds(bb.minx, bb.miny, bb.minz, bb.maxx, bb.maxy, bb.maxz);
    rig->applyPreset(preset_from(l.rig));
    out.rigs.push_back(std::move(rig));
    return;
  }
  auto ln = sg.addLight(l.id);
  if (!ln) {
    warn(warnings, "ari: scene light '" + l.id + "' could not be created");
    return;
  }
  const LightNode::Kind kind = light_kind(l.kind);
  ln->setKind(kind); // FIRST: gates setCone's per-kind clamp
  ln->setColor(l.color[0], l.color[1], l.color[2]);
  ln->setIntensity(l.intensity);
  if (kind == LightNode::Kind::Directional) {
    ln->setDirection(l.azimuth, l.elevation); // a compass sun; pos/target don't apply
  } else {
    ln->setPosition(l.pos[0], l.pos[1], l.pos[2]);
    ln->setTarget(l.target[0], l.target[1], l.target[2]);
    ln->setCone(l.cone);
  }
}

} // namespace

RealizedScene realize_scene(SceneGraph &sg, const cvc::ariadne::Scene &scene,
                            const std::string &bind_prefix,
                            std::vector<std::string> *warnings) {
  RealizedScene out;
  out.created.reserve(scene.nodes.size());
  for (const auto &n : scene.nodes) {
    realize_node(sg, n, bind_prefix, out, warnings);
    out.created.push_back(n.id);
  }
  // Lights run AFTER the nodes so a StageLighting rig can frame the realized geometry.
  // Batch the whole loop: setPosition fires transformChanged, not lightsChanged, and
  // addLight applies the (still-origin) light set before we move it — so one
  // endLightBatch() applyLights() reading each light's now-current world position is
  // both the perf win and the correctness fix (no stale positions baked).
  if (!scene.lights.empty()) {
    sg.beginLightBatch();
    for (const auto &l : scene.lights)
      realize_light(sg, l, out, warnings);
    sg.endLightBatch();
  }
  if (scene.has_shadows) {
    if (!sg.setShadowsEnabled(scene.shadows_enabled))
      warn(warnings, "ari: shadows requested but the renderer has no shadow target yet");
  }
  return out;
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
