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
#include <cvc/gl/VolRenNode.h>
#include <cvc/gl/VolSliceNode.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/volume/bounding_box.h>
#include <cvc/volume/volume.h>
#include <cvc/volren/settings.h>
#include <cvc/volslice/settings.h>

#include <vtkRenderer.h>

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

// Translate the backend-neutral SceneVolRen into cvc::volren settings + apply them to
// a freshly created VolRenNode. Warns (never throws) on configs that render blank.
void configure_volren(VolRenNode &vn, const cvc::ariadne::SceneVolRen &v, const cvc::volume &vol,
                      const std::string &id, std::vector<std::string> *warnings) {
  cvc::volren::volume_settings vs;
  vs.shaded = v.shaded;
  vs.unshaded = v.unshaded;
  vs.distance_field = v.distance_field;
  // The DSL `window` is a TF-DOMAIN statement (as volslice honors it), NOT a density
  // clip. volren's window_min/max is a sample cull, and its only TF-domain knob is
  // tf_auto_domain (data range vs control-point extent) — so DON'T bind `window` to
  // the cull (that would silently delete voxels and diverge from volslice). With
  // has_window the loader sets auto_domain=false, so volren bakes the TF over its
  // control-point extent; author the domain via the control-point values. An explicit
  // arbitrary TF window for volren needs a tf_domain field in cvc::volren — a follow-up.
  vs.tf_auto_domain = v.tf.auto_domain;
  for (const auto &p : v.tf.points)
    vs.tf.add({p.value, p.color[0], p.color[1], p.color[2], p.color[3]});
  for (const auto &s : v.isosurfaces) {
    cvc::volren::isosurface iso;
    iso.value = s.value;
    iso.opacity = s.opacity;
    iso.color = {s.color[0], s.color[1], s.color[2]};
    iso.shininess = s.shininess;
    vs.isosurfaces.push_back(iso);
  }
  // Blank when there is no isosurface AND (no TF, or neither media pass is enabled).
  if (v.isosurfaces.empty() && (v.tf.empty() || !(v.shaded || v.unshaded)))
    warn(warnings, "ari: volren node '" + id +
                       "' has nothing to render (no isosurfaces, and its transfer_function is empty or "
                       "both shaded and unshaded are off); it renders blank");
  else if (v.shaded && v.lights.empty() && v.ambient == 0.0f)
    warn(warnings, "ari: volren node '" + id +
                       "' is shaded with no lights and ambient 0; the surface reads as a black silhouette");
  vn.setResolutionScale(v.resolution_scale);
  vn.addVolume(vol, vs);
  cvc::volren::render_settings rs = vn.renderConfig();
  rs.ambient = v.ambient;
  rs.steps = v.steps;
  for (const auto &l : v.lights) {
    cvc::volren::light lt;
    lt.color = {l.color[0], l.color[1], l.color[2]};
    lt.direction = {l.direction[0], l.direction[1], l.direction[2]};
    rs.lights.push_back(lt);
  }
  vn.setRenderConfig(rs);
  if (v.backend == "cuda")
    vn.setBackend(cvc::volren::backend::cuda);
  else if (v.backend == "automatic")
    vn.setBackend(cvc::volren::backend::automatic);
  else
    vn.setBackend(cvc::volren::backend::cpu);
}

// Translate SceneVolSlice into cvc::volslice settings + apply. Warns on an empty TF.
void configure_volslice(VolSliceNode &vn, const cvc::ariadne::SceneVolSlice &v,
                        const cvc::volume &vol, const std::string &id,
                        std::vector<std::string> *warnings) {
  vn.setVolume(vol);
  cvc::volslice::render_settings s;
  s.slices.quality = v.quality;
  s.slices.max_planes = v.max_planes;
  s.slices.near_plane = v.near_plane;
  s.filter = v.nearest_filter ? cvc::volslice::interpolation::nearest
                              : cvc::volslice::interpolation::linear;
  s.opacity_correction = v.opacity_correction;
  s.tf_auto_domain = v.tf.auto_domain;
  if (v.tf.has_window) {
    s.window_min = v.tf.window_min;
    s.window_max = v.tf.window_max;
  }
  for (const auto &p : v.tf.points)
    s.tf.add({p.value, p.color[0], p.color[1], p.color[2], p.color[3]});
  if (v.tf.empty())
    warn(warnings, "ari: volslice node '" + id +
                       "' has an empty transfer_function; it renders blank");
  vn.setConfig(s);
}

// Realize one node under `parent` (null = a top-level node parented to the graphics
// root). A nested node is created via parent->createChild/addGraphicsChild, which
// gives it the hierarchical state path `{parent}.children.{id}` AND makes its
// `transform:` a LOCAL transform composed with the parent's world transform — so a
// child moves/rotates/scales relative to its parent (§9 local transforms). Top-level
// nodes go through sg.addGraphics (registered in the name map + null-graphic removal
// + the volume-rendering hookup).
void realize_node(SceneGraph &sg, const cvc::ariadne::SceneNode &n,
                  const std::string &bind_prefix, RealizedScene &out,
                  GraphicsNode *parent, std::vector<std::string> *warnings) {
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
    std::shared_ptr<GeometryNode> g =
        parent ? parent->createChild<GeometryNode>(n.id, geom)
               : std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics(n.id, geom));
    if (g && n.has_material) {
      g->setUseSingleColor(true);
      g->setColor(n.color[0], n.color[1], n.color[2]);
      g->setAmbient(n.ambient);
      g->setDiffuse(n.diffuse);
    }
    node = g;
  } else if (n.type == "volume") {
    if (n.source_file.empty()) {
      warn(warnings, "ari: scene node '" + n.id + "' (volume) has no source.file");
      return;
    }
    std::shared_ptr<VolumeNode> vnode;
    try {
      cvc::volume vol(sg.appContext(), n.source_file); // reads on construct; throws on a bad file
      if (parent) {
        // Nested: local transform composes with the parent. createChild -> setVolume
        // sets the default grayscale TF, enough to render a single volume. Multi-volume
        // compositing across a NESTED volume isn't toggled here (SceneGraph's
        // updateVolumeRendering is private) — a documented follow-up; a lone nested
        // volume renders fine.
        vnode = parent->createChild<VolumeNode>(n.id, vol);
      } else {
        vnode = sg.addGraphics(n.id, vol); // sets a default grayscale transfer function -> renders
      }
    } catch (const std::exception &e) {
      warn(warnings, "ari: scene node '" + n.id + "': " + e.what());
      return;
    }
    node = vnode;
    if (vnode && n.has_material) {
      // A volume's colour comes from a transfer function, not a single actor colour,
      // so `color[3]` has no analog here (a TF spec is a follow-up); only the lighting
      // coefficients map. Apply only when the node is styled, so an unstyled volume
      // keeps VolumeNode's tuned defaults.
      vnode->setAmbient(n.ambient);
      vnode->setDiffuse(n.diffuse);
    }
  } else if (n.type == "volren") {
    if (n.source_file.empty()) {
      warn(warnings, "ari: scene node '" + n.id + "' (volren) has no source.file");
      return;
    }
    std::shared_ptr<VolRenNode> vn;
    try {
      cvc::volume vol(sg.appContext(), n.source_file); // reads on construct; throws on a bad file
      // No sg.addGraphics overload for VolRenNode: create under the parent (or root).
      GraphicsNode *pr = parent ? parent : sg.getGraphicsRoot().get();
      // Last-wins parity with sg.addGraphics: registerGraphics only reassigns the name
      // map, so a same-named top-level node must be unlinked first or it leaks + double
      // renders. Done only after the load succeeded, so a bad file can't drop a live node.
      if (!parent && sg.hasGraphics(n.id))
        sg.removeGraphics(n.id);
      vn = pr->addGraphicsChild<VolRenNode>(n.id);
      if (!parent)
        sg.registerGraphics(n.id, vn); // name-map parity + grid enclosure for a top-level node
      configure_volren(*vn, n.volren, vol, n.id, warnings);
    } catch (const std::exception &e) {
      warn(warnings, "ari: scene node '" + n.id + "': " + e.what());
      return;
    }
    out.volren_ticks.push_back(vn); // needs a per-frame tick() (tick_scene)
    node = vn;
  } else if (n.type == "volslice") {
    if (n.source_file.empty()) {
      warn(warnings, "ari: scene node '" + n.id + "' (volslice) has no source.file");
      return;
    }
    std::shared_ptr<VolSliceNode> vn;
    try {
      cvc::volume vol(sg.appContext(), n.source_file);
      GraphicsNode *pr = parent ? parent : sg.getGraphicsRoot().get();
      if (!parent && sg.hasGraphics(n.id)) // last-wins parity with sg.addGraphics (see volren)
        sg.removeGraphics(n.id);
      vn = pr->addGraphicsChild<VolSliceNode>(n.id);
      if (!parent)
        sg.registerGraphics(n.id, vn);
      configure_volslice(*vn, n.volslice, vol, n.id, warnings);
    } catch (const std::exception &e) {
      warn(warnings, "ari: scene node '" + n.id + "': " + e.what());
      return;
    }
    out.volslice_ticks.push_back(vn); // needs a per-frame tick() + depth sort (tick_scene)
    node = vn;
  } else if (n.type == "group") {
    node = parent ? parent->createChild(n.id) // empty nested hierarchy node
                  : sg.addGraphics(n.id);
  } else {
    // light node type is a follow-up (SceneNode lacks the light fields — declare
    // lights in the `lights:` array). The spec still parses/round-trips.
    warn(warnings, "ari: scene node '" + n.id + "' type '" + n.type +
                       "' not realized yet (geometry/group/volume/volren/volslice; a 'light' node is a follow-up)");
    return;
  }

  if (!node)
    return;
  apply_transform(*node, n); // the node's LOCAL transform (composed with the parent's)
  apply_visibility(*node, n, sg.appContext(), bind_prefix, out.visibility);

  // Children nest UNDER this node, so each child's transform is local to it.
  for (const auto &c : n.children)
    realize_node(sg, c, bind_prefix, out, node.get(), warnings);
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
    realize_node(sg, n, bind_prefix, out, /*parent=*/nullptr, warnings);
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

void tick_scene(RealizedScene &realized, vtkRenderer *renderer) {
  for (const std::weak_ptr<VolRenNode> &w : realized.volren_ticks)
    if (std::shared_ptr<VolRenNode> n = w.lock())
      n->tick();

  // Lock the slice nodes once: keep the shared_ptrs alive across the sort, and hand
  // depthSortSliceProps the raw pointers it wants. The sort only matters (and only
  // runs) when ≥2 slice nodes share the renderer.
  std::vector<std::shared_ptr<VolSliceNode>> alive;
  std::vector<VolSliceNode *> raw;
  for (const std::weak_ptr<VolSliceNode> &w : realized.volslice_ticks)
    if (std::shared_ptr<VolSliceNode> n = w.lock()) {
      n->tick();
      raw.push_back(n.get());
      alive.push_back(std::move(n));
    }
  if (renderer && raw.size() >= 2)
    VolSliceNode::depthSortSliceProps(renderer, raw);
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
