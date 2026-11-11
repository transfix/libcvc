#include <algorithm>
#include <array>
#include <boost/any.hpp>
#include <cmath>
#include <cstddef>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/data.h> // source: { data } — a dataset of the document's data: block
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_geometry.h> // a node's mesh from data (core)
#include <cvc/ariadne/scene_world.h>    // the scene's world: physics and bodies (core)
#include <cvc/ariadne/uri.h> // §13.4 resolve a node source: URI to a local path (temp-file bridge)
#include <cvc/core/app.h>
#include <cvc/core/world_clock.h> // §9 time: tick_scene drives app.world_clock() for a clock: scene
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/geometry_file_io.h>
#include <cvc/geometry/height_field.h> // a heightfield source is the world's own cvc::height_field
#include <cvc/geometry/mesh_ops.h>     // mesh_ops_available() — source.sdf.algorithm: igl
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/LightNode.h>
#include <cvc/gl/LodGraphicsNode.h> // RealizeOptions::lod: a mesh node as a LOD ladder
#include <cvc/gl/LodPyramidCache.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/StageLighting.h>
#include <cvc/gl/VolRenNode.h>
#include <cvc/gl/VolSliceNode.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/tf_binding.h>    // every volume node's editable transfer-function model
#include <cvc/gl/ariadne/vehicle_view.h>  // the world's vehicles, drawn on their chassis
#include <cvc/gl/physics_scene_binding.h> // body: the nodes physics moves
#include <cvc/gl/world_geometry_nodes.h>  // a dataset's world_geometry, drawn
#include <cvc/image/image.h>              // cvc::read_image — material: { texture: <uri> }
#include <cvc/physics/world.h>            // physics: the scene's rigid-body world
#include <cvc/physics/world_geometry.h>   // ... and its bodies (a data node's body:)
#include <cvc/state/state.h>
#include <cvc/utility/algorithm.h> // cvc::sdf — source: { sdf: { mesh:, dim: } } volume source
#include <cvc/volren/settings.h>
#include <cvc/volslice/settings.h>
#include <cvc/volume/bounding_box.h>
#include <cvc/volume/volume.h>
#include <cvc/world/generated_world.h> // source.heightfield.generator: a cvc::world terrain
#include <exception>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream> // a frame request
#include <stdexcept>
#include <vtkCamera.h>
#include <vtkMatrix4x4.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

namespace cvc {
namespace gl {
namespace ariadne {

namespace {

void warn(std::vector<std::string> *w, const std::string &m) {
  if (w)
    w->push_back(m);
}

// §13.4: resolve a node's source: URI to a local path a reader (read_geometry / cvc::volume) can
// open — file:// in place, any other scheme spilled to an owned temp file. On failure, warn with
// the node id and return an unset ResolvedFile (ok=false); the caller returns early. The returned
// object must stay alive for the whole read (it owns any temp file).
cvc::ariadne::ResolvedFile resolve_source(const cvc::ariadne::SceneNode &n,
                                          std::vector<std::string> *warnings) {
  cvc::ariadne::ResolvedFile rf = cvc::ariadne::resolve_to_file(n.source_file);
  if (!rf.ok)
    warn(warnings, "ari: scene node '" + n.id + "': " + rf.error);
  return rf;
}

// A body a node declared (`body:`), recorded while the nodes are realized and added once they all
// are (a body on a terrain needs the terrain's field).
struct pending_body {
  const cvc::ariadne::SceneNode *spec = nullptr; // into the Scene realize_scene was given
  std::shared_ptr<GraphicsNode> node;
};

// --- custom scene node type registry (§ extensibility) -----------------------
std::mutex &node_registry_mutex() {
  static std::mutex m;
  return m;
}
std::map<std::string, NodeRealizer> &node_registry() {
  static std::map<std::string, NodeRealizer> r;
  return r;
}
bool is_builtin_scene_type(const std::string &t) {
  return t == "geometry" || t == "volume" || t == "volren" || t == "volslice" || t == "group";
}

// The named GLSL shader-preset registry (the DSL `shader: { preset: <name> }`). Process-global and
// mutex-guarded, mirroring the node-type registry.
std::mutex &shader_preset_mutex() {
  static std::mutex m;
  return m;
}
std::map<std::string, ShaderPreset> &shader_preset_registry() {
  static std::map<std::string, ShaderPreset> r;
  return r;
}

// Apply a node's `shader:` block to a GeometryNode (GL-specific — the loader only carried it as
// data): the coordinate-shift-scale toggle, then a named preset (warn if unknown), then the inline
// vertex/fragment splices. A node with no shader is a no-op.
void apply_shader(GeometryNode &g, const cvc::ariadne::SceneNode &n,
                  std::vector<std::string> *warnings) {
  const cvc::ariadne::SceneShader &sh = n.shader;
  if (!sh.present)
    return;
  if (sh.disable_coord_shift)
    g.disableCoordinateShiftScale();
  if (!sh.preset.empty()) {
    ShaderPreset preset;
    {
      std::lock_guard<std::mutex> lk(shader_preset_mutex());
      const auto it = shader_preset_registry().find(sh.preset);
      if (it != shader_preset_registry().end())
        preset = it->second;
    }
    if (preset)
      preset(g);
    else
      warn(warnings, "ari: scene node '" + n.id + "': unknown shader preset '" + sh.preset + "'");
  }
  for (const cvc::ariadne::SceneShaderStage &st : sh.vertex)
    g.addVertexShaderReplacement(st.at, st.code);
  for (const cvc::ariadne::SceneShaderStage &st : sh.fragment)
    g.addFragmentShaderReplacement(st.at, st.code);
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
void apply_visibility(GraphicsNode &node, const cvc::ariadne::SceneNode &n, cvc::app &app,
                      const std::string &bind_prefix,
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
  const int v0 = cvc::ariadne::read_bool_or(app, source, n.visible_default ? 1 : 0); // visible=bool
  cvc::ariadne::write<int>(app, target, v0);
  binds.push_back({source, target, n.visible_default});
}

// Apply a node's `clip:` block (§9 clip). Every plane is in the node's OWN coordinates (where its
// geometry / volume is defined, before `transform:`): GraphicsNode::setClipPlanes takes them as
// given and keeps them in that frame, so the clip moves, turns and scales with the node. The box's
// six faces come first, then `planes:`. A bound `offset:` is read now -- WITHOUT seeding, a
// follower like `visible:` -- and recorded as a SceneClipBinding that tick_scene re-reads every
// frame. A plane it cannot use (a zero normal, an inverted box) is warned about and dropped.
void apply_clip(GraphicsNode &node, const cvc::ariadne::SceneNode &n, cvc::app &app,
                const std::string &bind_prefix, std::vector<cvc::ariadne::SceneClipBinding> &binds,
                std::vector<std::string> *warnings) {
  const cvc::ariadne::SceneClip &c = n.clip;
  if (!c.present)
    return;
  using BoundPlane = cvc::ariadne::SceneClipBinding::Plane;
  std::vector<BoundPlane> planes;
  if (c.has_box) {
    const double *b = c.box;
    if (b[0] > b[3] || b[1] > b[4] || b[2] > b[5]) {
      warn(warnings, "ari: scene node '" + n.id + "': clip.box has min > max on an axis — ignored");
    } else {
      for (const GraphicsNode::ClipPlane &f :
           GraphicsNode::boxClipPlanes(cvc::bounding_box(b[0], b[1], b[2], b[3], b[4], b[5]))) {
        BoundPlane q;
        for (int k = 0; k < 3; ++k) {
          q.origin[k] = f.origin[k];
          q.normal[k] = f.normal[k];
        }
        planes.push_back(q);
      }
    }
  }
  bool bound = false;
  for (std::size_t i = 0; i < c.planes.size(); ++i) {
    const cvc::ariadne::SceneClipPlane &p = c.planes[i];
    const double len = std::sqrt(p.normal[0] * p.normal[0] + p.normal[1] * p.normal[1] +
                                 p.normal[2] * p.normal[2]);
    if (!(len > 0.0) || !std::isfinite(len)) {
      warn(warnings, "ari: scene node '" + n.id + "': clip.planes[" + std::to_string(i) +
                         "] has a zero normal — ignored");
      continue;
    }
    BoundPlane q;
    for (int k = 0; k < 3; ++k) {
      q.origin[k] = p.origin[k];
      q.normal[k] = p.normal[k] / len;
    }
    q.offset = p.offset;
    if (!p.offset_bind.empty()) {
      q.offset_source = cvc::ariadne::resolve_bind(bind_prefix, p.offset_bind);
      bound = true;
    }
    planes.push_back(q);
  }
  // Apply now, bound offsets at their live value, so the first frame is already right.
  std::vector<GraphicsNode::ClipPlane> local;
  for (const BoundPlane &q : planes) {
    const double d = q.offset_source.empty()
                         ? q.offset
                         : cvc::ariadne::read_or<double>(app, q.offset_source, q.offset);
    GraphicsNode::ClipPlane cp;
    for (int k = 0; k < 3; ++k) {
      cp.origin[k] = q.origin[k] + (std::isfinite(d) ? d : q.offset) * q.normal[k];
      cp.normal[k] = q.normal[k];
    }
    local.push_back(cp);
  }
  node.setClipPlanes(local);
  if (bound)
    binds.push_back({node.stateName("clip_planes"), planes, std::string()});
  if (c.children)
    node.setClipChildren(true);
}

// Say when a node's renderer cannot honour what clips it (its own planes and every ancestor's --
// the ancestors are realized first, so the count is final here). Only for the built-in types that
// draw, plus any node that reports a cap: a group clips nothing of its own.
void warn_clip_capability(GraphicsNode &node, const cvc::ariadne::SceneNode &n,
                          std::vector<std::string> *warnings) {
  const int count = node.clipPlaneCount();
  if (count == 0)
    return;
  const int cap = node.maxClipPlanes();
  const bool draws = n.type == "geometry" || n.type == "volume" || n.type == "volren" ||
                     n.type == "volslice" || cap > 0;
  if (!draws)
    return;
  if (cap == 0)
    warn(warnings, "ari: scene node '" + n.id + "' is clipped (" + std::to_string(count) +
                       " plane(s)) but its renderer has no clip-plane support (VTK's low-memory "
                       "mapper, the WebGL/GLES default) — it draws unclipped");
  else if (count > cap)
    warn(warnings, "ari: scene node '" + n.id + "' is clipped by " + std::to_string(count) +
                       " planes but its renderer honours " + std::to_string(cap) +
                       "; the nearest " + std::to_string(cap) +
                       " apply (its own first, then its parent's, ...)");
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
  if (!v.tf.has_model) // the model form reaches the renderer through its tf_binding instead
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
  const bool no_iso = v.isosurfaces.empty() && !(v.tf.has_model && !v.tf.model.isocontours.empty());
  const bool no_tf = v.tf.has_model ? cvc::tf::transparent(v.tf.model) : v.tf.points.empty();
  if (no_iso && (no_tf || !(v.shaded || v.unshaded)))
    warn(warnings,
         "ari: volren node '" + id +
             "' has nothing to render (no isosurfaces, and its transfer_function is empty or "
             "both shaded and unshaded are off); it renders blank");
  else if (v.shaded && v.lights.empty() && v.ambient == 0.0f)
    warn(warnings,
         "ari: volren node '" + id +
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
  rs.shadows.enabled = v.shadows;
  if (v.shadow_strength >= 0.0f)
    rs.shadows.strength = v.shadow_strength;
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
  if (!v.tf.has_model) // the model form reaches the renderer through its tf_binding instead
    for (const auto &p : v.tf.points)
      s.tf.add({p.value, p.color[0], p.color[1], p.color[2], p.color[3]});
  if (v.tf.has_model ? cvc::tf::transparent(v.tf.model) : v.tf.points.empty())
    warn(warnings,
         "ari: volslice node '" + id + "' has an empty transfer_function; it renders blank");
  vn.setConfig(s);
}

// Feed a SceneTransferFunction to a VolumeNode (the VTK GPU mapper). Point values are ABSOLUTE
// scalar values — the mapper takes the raw domain directly, so auto_domain/window don't apply (see
// SceneVolume). VolumeNode::setTransferFunction wants two flat tables: colour [scalar,r,g,b,...]
// and opacity [scalar,a,...].
void apply_volume_transfer_function(cvc::gl::VolumeNode &vn,
                                    const cvc::ariadne::SceneTransferFunction &tf) {
  std::vector<double> colorTable, opacityTable;
  colorTable.reserve(tf.points.size() * 4);
  opacityTable.reserve(tf.points.size() * 2);
  for (const auto &p : tf.points) {
    colorTable.push_back(p.value);
    colorTable.push_back(p.color[0]);
    colorTable.push_back(p.color[1]);
    colorTable.push_back(p.color[2]);
    opacityTable.push_back(p.value);
    opacityTable.push_back(p.color[3]);
  }
  vn.setTransferFunction(colorTable, opacityTable);
}

// --- the editable transfer-function model (tf_binding) -------------------------------------------
//
// Every volume-type node gets a tf_binding whose model key is <node>.transfer_function, so a
// `tf_editor` bound to graphics.root.children.<id>.transfer_function drives it. The seed is what
// the node already draws: the document's model form (lowered by the binding at attach), or the
// legacy points / isosurfaces / default, which the realizer has already handed the renderer above
// -- those are seed_is_applied, so an existing document renders exactly as before until an edit.

// What a node with no transfer function draws (nothing but its isosurfaces).
cvc::tf::model transparent_model() {
  cvc::tf::model m = cvc::tf::model::default_model();
  m.alpha = {{0.0, 0.0f}, {1.0, 0.0f}};
  m.isocontours.clear();
  m.normalize();
  return m;
}

// The legacy coupled points (raw data values) as a model over [lo, hi], exact inside it: the
// endpoints are the piecewise-linear TF sampled there (held past the outer points, as every
// renderer samples it), so normalize() never invents an endpoint and points outside the span only
// shape it through those samples.
cvc::tf::model model_from_points(const std::vector<cvc::ariadne::SceneTFPoint> &points, double lo,
                                 double hi, bool explicit_domain) {
  std::vector<cvc::ariadne::SceneTFPoint> p(points);
  std::stable_sort(p.begin(), p.end(),
                   [](const cvc::ariadne::SceneTFPoint &a, const cvc::ariadne::SceneTFPoint &b) {
                     return a.value < b.value;
                   });
  const auto sample = [&p](double v, double out[4]) {
    const auto hi_it =
        std::upper_bound(p.begin(), p.end(), v,
                         [](double x, const cvc::ariadne::SceneTFPoint &q) { return x < q.value; });
    if (hi_it == p.begin() || hi_it == p.end()) {
      const cvc::ariadne::SceneTFPoint &q = hi_it == p.begin() ? p.front() : p.back();
      for (int c = 0; c < 4; ++c)
        out[c] = q.color[c];
      return;
    }
    const cvc::ariadne::SceneTFPoint &a = *(hi_it - 1), &b = *hi_it;
    const double t = (v - a.value) / (b.value - a.value);
    for (int c = 0; c < 4; ++c)
      out[c] = a.color[c] + (b.color[c] - a.color[c]) * t;
  };
  std::vector<double> flat;
  const auto push = [&flat](double v, const double c[4]) {
    flat.insert(flat.end(), {v, c[0], c[1], c[2], c[3]});
  };
  double c[4];
  sample(lo, c);
  push(lo, c);
  for (const cvc::ariadne::SceneTFPoint &q : p)
    if (q.value > lo && q.value < hi) {
      const double qc[4] = {q.color[0], q.color[1], q.color[2], q.color[3]};
      push(q.value, qc);
    }
  sample(hi, c);
  push(hi, c);
  cvc::tf::model m = cvc::tf::from_coupled_points(flat, lo, hi);
  m.domain.automatic = !explicit_domain;
  if (explicit_domain) {
    m.domain.min = lo;
    m.domain.max = hi;
  }
  m.normalize();
  return m;
}

void widen(double &lo, double &hi) {
  if (!(hi > lo))
    hi = lo + 1.0;
}

// volren: the document's model, else the legacy points over the domain volren bakes them over --
// the data range (auto_domain, an `auto` model), or the points' own extent (auto_domain: false,
// an explicit model; the TF is held past it) -- so the first unrelated edit lowers the very TF
// volren drew. A legacy `window:` plays no part: configure_volren ignores it (the loader turns it
// into auto_domain: false), so seeding over it would show, and the first edit would draw, a TF
// held at the window's edges that volren never drew. Points at one value with auto_domain: false
// give volren no extent to bake over -- it draws them transparent -- and so seed a transparent
// model. The isosurfaces become isocontours with their colour/opacity as overrides and their
// shininess kept by id.
tf_binding_options volren_tf_options(const cvc::ariadne::SceneVolRen &v, const cvc::volume &vol,
                                     const std::string &id, std::vector<std::string> *warnings) {
  tf_binding_options o;
  double vlo = vol.min(), vhi = vol.max();
  widen(vlo, vhi);
  const cvc::ariadne::SceneTransferFunction &tf = v.tf;
  if (tf.has_model) {
    o.seed = tf.model;
  } else if (!tf.points.empty() && tf.auto_domain) {
    o.seed = model_from_points(tf.points, vlo, vhi, false);
    o.seed_is_applied = true;
  } else if (!tf.points.empty()) {
    const auto mm =
        std::minmax_element(tf.points.begin(), tf.points.end(),
                            [](const cvc::ariadne::SceneTFPoint &a,
                               const cvc::ariadne::SceneTFPoint &b) { return a.value < b.value; });
    o.seed = mm.second->value > mm.first->value
                 ? model_from_points(tf.points, mm.first->value, mm.second->value, true)
                 : transparent_model();
    o.seed_is_applied = true;
  } else {
    o.seed = transparent_model();
    o.seed_is_applied = true;
  }
  double lo = 0.0, hi = 1.0;
  cvc::tf::effective_domain(o.seed, true, vlo, vhi, lo, hi);
  for (const cvc::ariadne::SceneIsosurface &s : v.isosurfaces) {
    cvc::tf::isocontour ic;
    ic.id = o.seed.next_isocontour_id();
    const double t = cvc::tf::to_normalized(s.value, lo, hi);
    if (!(t >= 0.0 && t <= 1.0))
      warn(warnings, "ari: volren node '" + id + "': isosurface " + std::to_string(s.value) +
                         " lies outside its transfer-function domain [" + std::to_string(lo) +
                         ", " + std::to_string(hi) + "]; an editor shows it clamped to the edge");
    ic.at = std::clamp(t, 0.0, 1.0);
    ic.has_rgb = true;
    ic.r = s.color[0];
    ic.g = s.color[1];
    ic.b = s.color[2];
    ic.has_opacity = true;
    ic.opacity = s.opacity;
    o.shininess[ic.id] = s.shininess;
    o.seed.isocontours.push_back(ic);
  }
  o.seed.normalize();
  return o;
}

// volslice: the document's model, else the legacy points over the window (an explicit domain:
// the legacy window spans the LUT, so the TF is held past it) or over the data range (`auto`) --
// VolSliceNode::effectiveWindow's choice. Either way the TF stays fixed in data units, as the
// legacy renderer keeps it: a window written later is only the LUT's span (the binding's range is
// the data range, so a moved window re-lowers nothing), and the binding never writes back over a
// window someone else changed (tf_binding.h).
tf_binding_options volslice_tf_options(const cvc::ariadne::SceneVolSlice &v,
                                       const cvc::volume &vol) {
  tf_binding_options o;
  const cvc::ariadne::SceneTransferFunction &tf = v.tf;
  if (tf.has_model) {
    o.seed = tf.model;
  } else if (!tf.points.empty()) {
    const bool expl = tf.has_window && tf.window_max > tf.window_min;
    double lo = expl ? tf.window_min : vol.min(), hi = expl ? tf.window_max : vol.max();
    widen(lo, hi);
    o.seed = model_from_points(tf.points, lo, hi, expl);
    o.seed_is_applied = true;
  } else {
    o.seed = transparent_model();
    o.seed_is_applied = true;
  }
  return o;
}

// volume (VTK): the document's model, else the legacy points (absolute values, so an explicit
// domain) over the node's data range, else VolumeNode's default ramp -- black -> white, opacity
// 0 -> 1 over the data range, an `auto` domain re-fitted to a reloaded volume like VolumeNode's own
// default. That shape, not the tables VTK holds now: on a degenerate (empty or constant) volume
// VolumeNode makes its ramp invisible (top opacity 0), and a seed copied from it would lower as
// EMPTY tables over the real volume a host loads later (setVolume), blanking the ramp VolumeNode
// re-fits there. The degenerate volume draws nothing either way (opacity 0 at its one value).
tf_binding_options volume_tf_options(const cvc::ariadne::SceneTransferFunction &tf,
                                     VolumeNode &vn) {
  tf_binding_options o;
  double lo = cvc::ariadne::read_or<double>(vn.app(), vn.stateName("data_min"), 0.0);
  double hi = cvc::ariadne::read_or<double>(vn.app(), vn.stateName("data_max"), 1.0);
  widen(lo, hi);
  if (tf.has_model) {
    o.seed = tf.model;
  } else if (!tf.points.empty()) {
    o.seed = model_from_points(tf.points, lo, hi, true);
    o.seed_is_applied = true;
  } else {
    o.seed = cvc::tf::from_tables({lo, 0.0, 0.0, 0.0, hi, 1.0, 1.0, 1.0}, {lo, 0.0, hi, 1.0}, lo,
                                  hi); // VolumeNode::setDefaultTransferFunction's ramp
    o.seed.domain.automatic = true;
    o.seed_is_applied = true;
  }
  return o;
}

// Realize one node under `parent` (null = a top-level node parented to the graphics
// root). A nested node is created via parent->createChild/addGraphicsChild, which
// gives it the hierarchical state path `{parent}.children.{id}` AND makes its
// `transform:` a LOCAL transform composed with the parent's world transform — so a
// child moves/rotates/scales relative to its parent (§9 local transforms). Top-level
// nodes go through sg.addGraphics (registered in the name map + null-graphic removal
// + the volume-rendering hookup).
bool read_node_geometry_and_field(SceneGraph &sg, const cvc::ariadne::SceneNode &n,
                                  cvc::geometry &geom,
                                  std::shared_ptr<const cvc::height_field> &field,
                                  std::vector<std::string> *warnings);

// RealizeOptions::lod: whether geometry node `n`, whose mesh is `geom`, realizes as a LOD ladder.
bool lod_eligible(const cvc::ariadne::SceneNode &n, const cvc::geometry &geom,
                  const RealizeOptions &options) {
  if (!options.lod)
    return false;
  // A mesh asset only. A primitive is a handful of triangles, and a heightfield is the terrain
  // bodies collide with and custom nodes sample (out.height_fields).
  if (n.has_heightfield || !n.source_primitive.empty() || n.source_file.empty())
    return false;
  // Nothing a ladder cannot carry: a texture, shader or clip is set up on one GeometryNode, a body
  // is made from the node's own GeometryNode (realize_physics), and children would nest under a
  // node whose rungs come and go.
  if (!n.material_texture.empty() || n.shader.present || n.clip.present || n.body.present ||
      !n.children.empty())
    return false;
  return geom.num_tris() + 2 * geom.num_quads() >= options.lod_min_tris;
}

// A geometry node realized as a LOD ladder, created where its GeometryNode would have been: under
// `parent`, or at the top level with sg.addGraphics' last-wins registration. Every rung gets the
// node's material; the cache shows `geom` at once and adds the coarser rungs when they are built.
std::shared_ptr<LodGraphicsNode>
realize_lod_geometry(SceneGraph &sg, const cvc::ariadne::SceneNode &n, const cvc::geometry &geom,
                     GraphicsNode *parent, LodPyramidCache &cache) {
  std::shared_ptr<LodGraphicsNode> lod;
  if (parent) {
    lod = parent->addGraphicsChild<LodGraphicsNode>(n.id);
  } else {
    if (sg.hasGraphics(n.id)) // last-wins parity with sg.addGraphics (see volren)
      sg.removeGraphics(n.id);
    lod = sg.getGraphicsRoot()->addGraphicsChild<LodGraphicsNode>(n.id);
    sg.registerGraphics(n.id, lod); // name-map parity + grid enclosure for a top-level node
  }
  // Set before the first rung exists, so no rung ever draws unstyled. The style lives as long as
  // the node and the Scene may not (a room scene is read into a temporary): it holds a copy.
  lod->setRungStyle([spec = n](GeometryNode &g) { apply_node_material(g, spec); });
  cache.attach(lod, geom);
  return lod;
}

// The geometry of a `source: { asset: <id> }` node: the asset's, read and fitted the first time a
// node asks (a copy-on-write copy, so every node on it shares the buffers).
bool asset_geometry(SceneGraph &sg, const cvc::ariadne::SceneNode &n,
                    const std::vector<cvc::ariadne::SceneAsset> &assets, SceneAssetCache &cache,
                    cvc::geometry &geom, std::vector<std::string> *warnings) {
  const std::string &id = n.source_asset;
  auto hit = cache.geometry.find(id);
  if (hit != cache.geometry.end()) {
    geom = hit->second;
    return true;
  }
  if (cache.failed.count(id))
    return false;
  const cvc::ariadne::SceneAsset *asset = nullptr;
  for (const cvc::ariadne::SceneAsset &a : assets)
    if (a.id == id)
      asset = &a;
  if (!asset) {
    warn(warnings,
         "ari: scene node '" + n.id + "': no asset '" + id + "' in the document's assets:");
    cache.failed.insert(id);
    return false;
  }
  cvc::geometry g;
  std::shared_ptr<const cvc::height_field> field;
  if (!read_node_geometry_and_field(sg, asset->source, g, field, warnings)) {
    cache.failed.insert(id);
    return false;
  }
  ++cache.reads;
  cache.geometry[id] = g;
  geom = g;
  return true;
}

void realize_node(SceneGraph &sg, const cvc::ariadne::SceneNode &n, const std::string &bind_prefix,
                  RealizedScene &out, GraphicsNode *parent, std::vector<pending_body> &bodies,
                  const RealizeOptions &options, std::vector<std::string> *warnings,
                  const std::vector<cvc::ariadne::SceneAsset> &assets) {
  std::shared_ptr<GraphicsNode> node;

  if (!n.source_data.empty()) {
    // source: { data: <name> } — an empty group now; sync_scene_data draws the dataset into it
    // once it is loaded, and again each time it changes.
    node = parent ? parent->createChild(n.id) : sg.addGraphics(n.id);
    if (node) {
      SceneDataBinding b;
      b.spec = n;
      b.path = cvc::ariadne::resolve_bind(bind_prefix, n.source_data[0] == '/'
                                                           ? n.source_data
                                                           : std::string(cvc::ariadne::kDataRoot) +
                                                                 "." + n.source_data);
      b.group = node;
      if (!n.frame.empty()) // the main view's camera (CameraController::viewerStatePath)
        b.frame_key = cvc::ariadne::resolve_bind(bind_prefix, "viewers.main.camera.frame");
      cvc::ariadne::DataStore::of(sg.appContext())->touch(b.path); // a lazy dataset loads now
      out.data_nodes.push_back(std::move(b));
    }
  } else if (n.type == "geometry") {
    cvc::geometry geom;
    std::shared_ptr<const cvc::height_field> field;
    if (!n.source_asset.empty()) {
      if (!options.assets || !asset_geometry(sg, n, assets, *options.assets, geom, warnings))
        return;
    } else if (!read_node_geometry_and_field(sg, n, geom, field, warnings)) {
      return;
    }
    if (field)
      out.height_fields[n.id] = field; // the grid this node draws, for bodies and custom nodes
    if (lod_eligible(n, geom, options)) {
      std::shared_ptr<LodGraphicsNode> lod =
          realize_lod_geometry(sg, n, geom, parent, *options.lod);
      out.lod_nodes.push_back(lod);
      node = lod;
    } else {
      std::shared_ptr<GeometryNode> g =
          parent ? parent->createChild<GeometryNode>(n.id, geom)
                 : std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics(n.id, geom));
      if (g)
        apply_node_material(*g, n);
      // An asset's nodes draw from its first node's VTK arrays (copy-on-write).
      if (g && !n.source_asset.empty() && options.assets) {
        std::weak_ptr<GeometryNode> &donor = options.assets->donors[n.source_asset];
        if (std::shared_ptr<GeometryNode> d = donor.lock())
          g->shareArraysFrom(d);
        else
          donor = g;
      }
      // A heightfield's per-vertex band colours are there to be shown: switch the node to
      // per-vertex colour (overriding the material single-colour default) when the field supplied
      // them.
      if (g && n.has_heightfield && !n.heightfield.colors.empty())
        g->setUseSingleColor(false);
      // material: { texture: <uri> } — resolve the URI, read the image, sample it through the
      // mesh's UVs. Warn (never throw) on an unresolved URI or an unreadable image so the rest of
      // the scene still realizes; a mesh with no UVs simply shows the texture's (0,0) texel.
      if (g && !n.material_texture.empty()) {
        cvc::ariadne::ResolvedFile tex = cvc::ariadne::resolve_to_file(n.material_texture);
        if (!tex.ok) {
          warn(warnings, "ari: scene node '" + n.id + "': texture " + tex.error);
        } else {
          try {
            g->setTexture(cvc::read_image(sg.appContext(), tex.path));
          } catch (const std::exception &e) {
            warn(warnings, "ari: scene node '" + n.id + "': texture '" + n.material_texture +
                               "': " + e.what());
          }
        }
        apply_uv_transform(*g, n); // material: { uv: ... } orients the image
      }
      node = g;
    }
  } else if (n.type == "volume") {
    if (n.source_file.empty()) {
      warn(warnings, "ari: scene node '" + n.id + "' (volume) has no source.file");
      return;
    }
    cvc::ariadne::ResolvedFile src = resolve_source(n, warnings);
    if (!src.ok)
      return;
    std::shared_ptr<VolumeNode> vnode;
    try {
      cvc::volume vol(sg.appContext(), src.path); // reads on construct; throws on a bad file
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
    if (vnode) {
      // A volume's colour comes from its transfer function (the `volume:` block), not a single
      // actor colour, so `color[3]` has no analog here. The lighting coefficients come from the
      // node `material:` block. Each is applied only when stated, so an unstyled volume keeps
      // VolumeNode's tuned defaults.
      if (n.has_material) {
        vnode->setAmbient(n.ambient);
        vnode->setDiffuse(n.diffuse);
        if (n.has_specular) {
          vnode->setSpecular(n.specular);
          vnode->setSpecularPower(n.specular_power);
        }
      }
      if (n.has_volume) {
        if (n.volume.has_shaded)
          vnode->setShading(n.volume.shaded);
        if (!n.volume.tf.has_model && !n.volume.tf.points.empty())
          apply_volume_transfer_function(*vnode, n.volume.tf);
      }
      if (const cvc::volume *v = vnode->getVolume())
        attach_transfer_function(out, vnode, tf_target::volume, *v,
                                 volume_tf_options(n.volume.tf, *vnode), warnings);
    }
  } else if (n.type == "volren") {
    if (n.source_file.empty() && n.source_sdf_mesh.empty()) {
      warn(warnings, "ari: scene node '" + n.id + "' (volren) has no source (file or sdf)");
      return;
    }
    std::shared_ptr<VolRenNode> vn;
    try {
      cvc::volume vol = load_node_volume(sg, n, warnings); // file, or an SDF of a mesh
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
      attach_transfer_function(out, vn, tf_target::volren, vol,
                               volren_tf_options(n.volren, vol, n.id, warnings), warnings);
    } catch (const std::exception &e) {
      warn(warnings, "ari: scene node '" + n.id + "': " + e.what());
      return;
    }
    out.volren_ticks.push_back(vn); // needs a per-frame tick() (tick_scene)
    node = vn;
  } else if (n.type == "volslice") {
    if (n.source_file.empty() && n.source_sdf_mesh.empty()) {
      warn(warnings, "ari: scene node '" + n.id + "' (volslice) has no source (file or sdf)");
      return;
    }
    std::shared_ptr<VolSliceNode> vn;
    try {
      cvc::volume vol = load_node_volume(sg, n, warnings);
      GraphicsNode *pr = parent ? parent : sg.getGraphicsRoot().get();
      if (!parent && sg.hasGraphics(n.id)) // last-wins parity with sg.addGraphics (see volren)
        sg.removeGraphics(n.id);
      vn = pr->addGraphicsChild<VolSliceNode>(n.id);
      if (!parent)
        sg.registerGraphics(n.id, vn);
      configure_volslice(*vn, n.volslice, vol, n.id, warnings);
      attach_transfer_function(out, vn, tf_target::volslice, vol,
                               volslice_tf_options(n.volslice, vol), warnings);
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
    // Not a built-in type — a registered CUSTOM realizer (register_scene_node_type)?
    NodeRealizer realizer;
    {
      std::lock_guard<std::mutex> lock(node_registry_mutex());
      auto it = node_registry().find(n.type);
      if (it != node_registry().end())
        realizer = it->second;
    }
    if (realizer) {
      node = realizer(sg, n, parent, out, warnings); // reads n.props; may push out.custom_ticks
      if (!node)
        return; // realizer declined (it warns for itself if useful)
    } else {
      // light node type is a follow-up (SceneNode lacks the light fields — declare
      // lights in the `lights:` array). The spec still parses/round-trips.
      warn(
          warnings,
          "ari: scene node '" + n.id + "' type '" + n.type +
              "' not realized (no built-in or registered realizer; a 'light' node is a follow-up)");
      return;
    }
  }

  if (!node)
    return;
  apply_transform(*node, n); // the node's LOCAL transform (composed with the parent's)
  apply_visibility(*node, n, sg.appContext(), bind_prefix, out.visibility);
  if (n.has_casts_shadow) // receives shadows either way; a non-caster never forces a re-bake
    node->setCastsShadow(n.casts_shadow);
  // shader: { preset | vertex | fragment } — the declarative GLSL surface — and material.opaque.
  // Applied in the shared tail (not just the built-in geometry branch) so a CUSTOM node type whose
  // realizer returns a GeometryNode (e.g. an L-system forest wanting `shader: { preset: bark }`)
  // is shaded too. A non-geometry node (volume/group) silently ignores them.
  if (auto *g = dynamic_cast<GeometryNode *>(node.get()))
    apply_mesh_keys(*g, n, warnings);
  apply_clip(*node, n, sg.appContext(), bind_prefix, out.clip, warnings); // own coordinates
  warn_clip_capability(*node, n, warnings);
  if (n.body.present && n.source_data.empty()) // a data node's come with its dataset
    bodies.push_back({&n, node});              // added once every node exists (realize_physics)
  out.nodes.push_back({n.id, n.tags, node});

  // Children nest UNDER this node, so each child's transform is local to it.
  for (const auto &c : n.children)
    realize_node(sg, c, bind_prefix, out, node.get(), bodies, options, warnings, assets);
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
    // A named StageLighting rig — a whole lighting SETUP, not one light. The rig must outlive the
    // render loop: ~StageLighting removes its lights, so RealizedScene owns it (the host keeps
    // RealizedScene alive).
    auto rig = std::make_unique<StageLighting>(sg);
    // Aim the rig: an explicit stage: { center, radius } (a TIGHT cone on the subject → a crisp
    // shadow map), else frame the realized geometry (lights are excluded from the bounds).
    if (l.has_stage) {
      rig->setStage(l.stage_center[0], l.stage_center[1], l.stage_center[2], l.stage_radius);
    } else {
      const cvc::bounding_box bb = sg.computeGraphicsBounds();
      if (!bb.isNull())
        rig->frameBounds(bb.minx, bb.miny, bb.minz, bb.maxx, bb.maxy, bb.maxz);
    }
    rig->applyPreset(preset_from(l.rig));
    // Optional per-knob tuning ON TOP of the preset (each gated so an unset knob keeps the preset).
    if (l.has_sun)
      rig->setSun(l.sun);
    if (l.has_key)
      rig->setKey(l.key_intensity, l.key_azimuth, l.key_elevation, l.key_cone);
    if (l.has_fill)
      rig->setFill(l.fill);
    if (l.has_back)
      rig->setBack(l.back);
    if (l.has_warmth)
      rig->setWarmth(l.warmth);
    if (l.has_environment)
      rig->setEnvironment(l.environment);
    if (l.has_rig_ambient)
      rig->setAmbient(l.rig_ambient);
    if (l.has_wash)
      rig->setWash(l.wash, l.wash > 0.0f ? l.wash_count : 0, l.wash_height);
    // applyPreset commits its baseline; the setters above need an explicit apply() to take effect.
    if (l.has_key || l.has_fill || l.has_back || l.has_warmth || l.has_environment ||
        l.has_rig_ambient || l.has_wash || l.has_stage || l.has_sun)
      rig->apply();
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

// --- helpers shared with custom node realizers (scene_realize.h) -------------

// `fit:` — bake a "stand this mesh on the ground" normalization into `raw`: optionally rotate a
// canonical Y-up mesh +90° about X to Z-up ((x,y,z)->(x,-z,y), a proper rotation so winding and
// normals stay valid), centre it in XY, sit its base on z=0, and scale its tallest extent to
// `height`. Returns a fresh geometry (points/normals/tris/colours), leaving `raw` untouched. This
// is the general form of bunny_shadow.cpp's stand_bunny — placing an arbitrarily-authored mesh
// predictably on a ground plane.
cvc::geometry fit_to_ground(const cvc::geometry &raw, bool up_y, float height) {
  return fit_to_ground(raw, up_y ? 'y' : 'z', false, height);
}

// The rest are core libcvc's (cvc/ariadne/scene_geometry.h), so a headless host builds the same
// bodies a window draws.
void turn_up_point(char up, bool flip, const double in[3], double out[3]) {
  cvc::ariadne::turn_up_point(up, flip, in, out);
}

char fit_up_axis(const cvc::ariadne::SceneNode &n) { return cvc::ariadne::fit_up_axis(n); }

cvc::geometry turn_up(const cvc::geometry &raw, char up, bool flip) {
  return cvc::ariadne::turn_up(raw, up, flip);
}

cvc::geometry fit_to_ground(const cvc::geometry &raw, char up, bool flip, float height) {
  return cvc::ariadne::fit_to_ground(raw, up, flip, height);
}

// The mesh a `type: geometry` node declares: core libcvc reads it (read_scene_node_geometry).
namespace {

bool read_node_geometry_and_field(SceneGraph &sg, const cvc::ariadne::SceneNode &n,
                                  cvc::geometry &geom,
                                  std::shared_ptr<const cvc::height_field> &field,
                                  std::vector<std::string> *warnings) {
  return cvc::ariadne::read_scene_node_geometry(sg.appContext(), n, geom, field, warnings);
}

} // namespace

bool read_node_geometry(SceneGraph &sg, const cvc::ariadne::SceneNode &n, cvc::geometry &geom,
                        std::vector<std::string> *warnings) {
  std::shared_ptr<const cvc::height_field> field;
  return read_node_geometry_and_field(sg, n, geom, field, warnings);
}

namespace {
// The node's uv block on `uvs`: flips, quarter turns (the image counter-clockwise on the surface,
// so the coordinates clockwise about the centre), then scale and offset.
void transform_uvs(cvc::geometry::uvs_t &uvs, const cvc::ariadne::SceneNode &n) {
  for (auto &uv : uvs) {
    double u = n.uv_flip_u ? 1.0 - uv[0] : uv[0];
    double v = n.uv_flip_v ? 1.0 - uv[1] : uv[1];
    for (int turn = 0; turn < n.uv_rotate / 90; ++turn) {
      const double du = u - 0.5, dv = v - 0.5;
      u = 0.5 + dv;
      v = 0.5 - du;
    }
    uv[0] = u * n.uv_scale[0] + n.uv_offset[0];
    uv[1] = v * n.uv_scale[1] + n.uv_offset[1];
  }
}
} // namespace

void apply_uv_transform(GeometryNode &g, const cvc::ariadne::SceneNode &n) {
  if (!n.has_uv_transform || !g.getGeometry() || g.getGeometry()->const_uvs().empty())
    return;
  cvc::geometry mesh = *g.getGeometry(); // copy-on-write: only its UVs are new
  transform_uvs(mesh.uvs(), n);
  g.setGeometry(mesh);
}

void apply_node_material(GeometryNode &g, const cvc::ariadne::SceneNode &n) {
  if (!n.has_material)
    return;
  g.setUseSingleColor(n.use_single_color);
  g.setColor(n.color[0], n.color[1], n.color[2]);
  g.setAmbient(n.ambient);
  g.setDiffuse(n.diffuse);
  if (n.has_specular) {
    g.setSpecular(n.specular);
    g.setSpecularPower(n.specular_power);
  }
}

// A dim³ signed-distance-field volume of a mesh, framed like the volume demos' bunny_sdf: the grid
// spans the mesh extents padded 10% (cubic). Lets a volren/volslice node declare
// `source: { sdf: { mesh:, dim: } }` instead of a pre-baked volume file.
cvc::volume sdf_volume_of_mesh(cvc::app &app, const cvc::geometry &mesh, int dim,
                               cvc::sdf_algorithm algorithm) {
#if CVC_ENABLE_SDF
  const cvc::bounding_box e = mesh.extents();
  const double ext = std::max({e.maxx - e.minx, e.maxy - e.miny, e.maxz - e.minz});
  const double half = ext * 1.1 * 0.5;
  const double cx = 0.5 * (e.minx + e.maxx), cy = 0.5 * (e.miny + e.maxy),
               cz = 0.5 * (e.minz + e.maxz);
  const cvc::bounding_box bbox(cx - half, cy - half, cz - half, cx + half, cy + half, cz + half);
  const int d = dim > 0 ? dim : 64;
  return cvc::sdf(app, mesh, cvc::dimension(d, d, d), bbox, algorithm);
#else
  (void)app;
  (void)mesh;
  (void)dim;
  (void)algorithm;
  throw std::runtime_error("source: { sdf } needs a CVC_ENABLE_SDF build");
#endif
}

cvc::volume sdf_volume_from_mesh(cvc::app &app, const std::string &mesh_uri, int dim,
                                 bool apply_fit, bool fit_up_y, float fit_height,
                                 cvc::sdf_algorithm algorithm) {
  return sdf_volume_from_mesh(app, mesh_uri, dim, apply_fit, fit_up_y ? 'y' : 'z', false,
                              fit_height, algorithm);
}

cvc::volume sdf_volume_from_mesh(cvc::app &app, const std::string &mesh_uri, int dim,
                                 bool apply_fit, char fit_up, bool fit_flip, float fit_height,
                                 cvc::sdf_algorithm algorithm) {
#if CVC_ENABLE_SDF
  cvc::geometry mesh = cvc::read_geometry(app, mesh_uri);
  if (apply_fit) // stand/centre/scale the mesh before the SDF, so the volume is framed like it
    mesh = fit_to_ground(mesh, fit_up, fit_flip, fit_height);
  return sdf_volume_of_mesh(app, mesh, dim, algorithm);
#else
  (void)app;
  (void)mesh_uri;
  (void)dim;
  (void)apply_fit;
  (void)fit_up;
  (void)fit_flip;
  (void)fit_height;
  (void)algorithm;
  throw std::runtime_error("source: { sdf } needs a CVC_ENABLE_SDF build");
#endif
}

// The volume a volren/volslice (or custom) node draws: loaded from `source: { file }`, or computed
// as the SDF of a mesh (`source: { sdf: { mesh:, dim:, algorithm: } }`). Throws on failure — the
// caller's try/catch warns and skips the node.
cvc::volume load_node_volume(SceneGraph &sg, const cvc::ariadne::SceneNode &n,
                             std::vector<std::string> *warnings) {
  if (!n.source_sdf_mesh.empty()) {
    // algorithm: v1 | v2 (the default) | igl. igl (fast winding-number sign + exact AABB distance)
    // needs a CVC_ENABLE_LIBIGL build; without one it falls back to v2 and says so.
    cvc::sdf_algorithm alg = cvc::SDF_V2;
    if (n.sdf_algorithm == "v1") {
      alg = cvc::SDF_V1;
    } else if (n.sdf_algorithm == "igl") {
      if (cvc::mesh_ops_available())
        alg = cvc::SDF_IGL;
      else
        warn(warnings, "ari: scene node '" + n.id +
                           "': source.sdf.algorithm 'igl' needs a CVC_ENABLE_LIBIGL build — using "
                           "v2");
    } else if (!n.sdf_algorithm.empty() && n.sdf_algorithm != "v2") {
      warn(warnings, "ari: scene node '" + n.id + "': source.sdf.algorithm '" + n.sdf_algorithm +
                         "' is not v1, v2 or igl — using v2");
    }
    return sdf_volume_from_mesh(sg.appContext(), n.source_sdf_mesh, n.sdf_dim, n.has_fit,
                                cvc::ariadne::fit_up_axis(n), n.fit_flip, n.fit_height, alg);
  }
  cvc::ariadne::ResolvedFile src = resolve_source(n, warnings);
  if (!src.ok)
    throw std::runtime_error("unresolved source");
  return cvc::volume(sg.appContext(), src.path);
}

namespace {

// The scene's world, built by core libcvc (cvc::ariadne::SceneWorld) from the nodes that declared a
// body: their specs, their world matrices from the spec (as a headless host composes them, so both
// make the same world), the meshes the nodes already hold, the terrains' grids, the colliders a
// custom node makes (out.colliders: a generated world's) and the data nodes' datasets (their bodies
// come once a dataset is ready, at the world's sync). The moving bodies are bound to their nodes so
// tick_scene draws them where physics has them. Problems warn (naming the node) and leave that body
// out; nothing here throws.
void realize_physics(SceneGraph &sg, const cvc::ariadne::Scene &scene,
                     const std::vector<pending_body> &pending, RealizedScene &out,
                     const std::string &stats_key, std::vector<std::string> *warnings) {
  std::map<const cvc::ariadne::SceneNode *, std::array<double, 16>> matrices;
  cvc::ariadne::scene_world_matrices(scene, matrices);
  std::vector<cvc::ariadne::SceneBodyInput> inputs;
  std::map<std::string, std::shared_ptr<GraphicsNode>> nodes;
  for (const pending_body &p : pending) {
    cvc::ariadne::SceneBodyInput in;
    in.spec = p.spec;
    auto m = matrices.find(p.spec);
    if (m != matrices.end())
      in.world = m->second;
    if (auto c = out.colliders.find(p.spec->id); c != out.colliders.end() && c->second) {
      in.colliders = c->second; // a custom node's own colliders (a generated world's)
    } else {
      auto gnode = std::dynamic_pointer_cast<GeometryNode>(p.node);
      if (gnode && gnode->getGeometry()) // a copy-on-write copy: the buffers are shared
        in.geometry = std::make_shared<const cvc::geometry>(*gnode->getGeometry());
    }
    inputs.push_back(std::move(in));
    nodes[p.spec->id] = p.node;
  }
  for (const SceneDataBinding &b : out.data_nodes) // a dataset's bodies come with the dataset
    if (b.spec.body.present) {
      cvc::ariadne::SceneBodyInput in;
      in.spec = &b.spec;
      for (const auto &m : matrices) // the node's matrix, by id (the binding holds a copy)
        if (m.first->id == b.spec.id)
          in.world = m.second;
      in.data_path = b.path;
      inputs.push_back(std::move(in));
    }
  out.world = cvc::ariadne::SceneWorld::build(sg.appContext(), scene.physics, inputs,
                                              out.height_fields, stats_key, warnings);
  if (!out.world)
    return;
  out.physics = &out.world->world();
  out.physics_binding = std::make_unique<physics_scene_binding>(*out.physics);
  for (const cvc::ariadne::SceneWorldBody &b : out.world->bodies()) {
    out.bodies.emplace_back(b.node, b.id);
    if (b.moves) // a static body never moves: the node stays as realized
      out.physics_binding->bind(b.id, nodes[b.node], b.body_from_visual.data());
  }
  out.body_nodes = std::move(nodes);
}

} // namespace

RealizedScene realize_scene(SceneGraph &sg, const cvc::ariadne::Scene &scene,
                            const std::string &bind_prefix, std::vector<std::string> *warnings) {
  return realize_scene(sg, scene, bind_prefix, warnings, RealizeOptions());
}

RealizedScene realize_scene(SceneGraph &sg, const cvc::ariadne::Scene &scene,
                            const std::string &bind_prefix, std::vector<std::string> *warnings,
                            const RealizeOptions &options) {
  RealizedScene out;
  // §9 time: carry the app + the scene's clock declaration so tick_scene can drive
  // app.world_clock() each frame (a no-op unless the scene declared a `clock:` block).
  out.app = &sg.appContext();
  out.sg = &sg;
  out.bind_prefix = bind_prefix;
  out.clock = scene.clock;
  out.cameras = scene.cameras; // what viewport widgets bind by id (scene_views.cpp)
  out.has_main_view = scene.has_main_view;
  out.main_show = scene.main_show;
  out.main_hide = scene.main_hide;
  // Resolve the clock's key paths to absolute state paths now, with the host bind prefix — the SAME
  // rule widget binds use (resolve_bind) — so the clock and a sim_transport slider on `sim.speed`
  // land on one key. Resolved once here (like the visibility bindings), so tick_scene stays prefix-
  // free. An empty key resolves to empty (stays "not bound").
  out.clock.speed_key = cvc::ariadne::resolve_bind(bind_prefix, scene.clock.speed_key);
  out.clock.paused_key = cvc::ariadne::resolve_bind(bind_prefix, scene.clock.paused_key);
  out.clock.time_key = cvc::ariadne::resolve_bind(bind_prefix, scene.clock.time_key);
  out.clock.tick_key = cvc::ariadne::resolve_bind(bind_prefix, scene.clock.tick_key);
  out.created.reserve(scene.nodes.size());
  // The assets' geometry: the caller's cache (shared across a document's scenes), or one for this
  // scene alone.
  SceneAssetCache own_assets;
  RealizeOptions opts = options;
  if (!opts.assets)
    opts.assets = &own_assets;
  std::set<std::string> present; // the top-level graphics before this scene's
  for (const auto &kv : sg.getAllGraphics())
    present.insert(kv.first);
  std::vector<pending_body> bodies;
  for (const auto &n : scene.nodes) {
    realize_node(sg, n, bind_prefix, out, /*parent=*/nullptr, bodies, opts, warnings, scene.assets);
    out.created.push_back(n.id);
  }
  // A custom node type may add top-level graphics beside its own node (forest_trees' needles).
  // They are this scene's too, so whoever clears it -- a host loading another document -- removes
  // them with the rest.
  for (const auto &kv : sg.getAllGraphics())
    if (!present.count(kv.first) &&
        std::find(out.created.begin(), out.created.end(), kv.first) == out.created.end())
      out.created.push_back(kv.first);
  // physics: and the nodes' body:s, once every node (and every terrain field) exists. A physics
  // scene runs on the scene clock, so it drives app.world_clock() as `clock: {}` would. A scene
  // with worlds (`world:` on its nodes) has one too: it keeps the worlds that are off out, and
  // writes the scene.world keys their nodes' visible: follow.
  if (scene.physics.present || !bodies.empty() || cvc::ariadne::scene_has_world(scene)) {
    realize_physics(sg, scene, bodies, out,
                    opts.physics_key.empty()
                        ? std::string()
                        : cvc::ariadne::resolve_bind(bind_prefix, opts.physics_key),
                    warnings);
    if (out.world) {
      out.world->bind_worlds(scene, cvc::ariadne::resolve_bind(bind_prefix, "scene.world"),
                             warnings);
      out.world->bind_vehicles(scene.vehicles, cvc::ariadne::resolve_bind(bind_prefix, "vehicles"),
                               cvc::ariadne::resolve_bind(bind_prefix, "entities"));
    }
    if (out.physics)
      out.clock.present = true;
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
    const bool applied = sg.setShadowsEnabled(scene.shadows_enabled);
    // setShadowsEnabled returns false ONLY when there is no renderer yet — so warn only when the
    // document actually REQUESTED shadows (enabled: false + no renderer is not an error).
    if (scene.shadows_enabled) {
      if (!applied) {
        warn(warnings, "ari: shadows requested but the renderer has no shadow target yet");
      } else {
        // Bigger map = crisper shadow; update_interval 1 = bake every frame (a static scene has no
        // lag). Only meaningful once shadows are actually on.
        if (scene.has_shadow_resolution)
          sg.setShadowResolution(scene.shadow_resolution);
        if (scene.has_shadow_interval)
          sg.setShadowUpdateInterval(scene.shadow_interval);
        if (scene.has_shadow_softness)
          sg.setShadowSoftness(scene.shadow_softness);
        // A fixed focus is set once; a camera-following one each frame by tick_scene.
        const cvc::ariadne::SceneShadowFocus &f = scene.shadow_focus;
        if (f.present && f.follow_camera)
          out.shadow_focus = f;
        else if (f.present && f.radius > 0.0)
          sg.setShadowFocus(f.center[0], f.center[1], f.center[2], f.radius);
      }
    }
  }
  if (scene.has_chrome) // strip/show the SceneGraph diagnostic chrome (grid/axis/bbox)
    sg.setDiagnosticChromeVisible(scene.chrome_visible);
  return out;
}

cvc::world_clock_keys scene_clock_keys(const cvc::ariadne::SceneClock &clock) {
  return cvc::ariadne::scene_clock_keys(clock);
}

namespace {

// What a data node draws: its source: { item }, else the first of the dataset's world, geometry,
// terrain and volume it has.
boost::any dataset_object(cvc::state &root, const std::string &store, const std::string &item,
                          std::string *found) {
  const std::vector<std::string> keys =
      item.empty() ? std::vector<std::string>{"world", "geometry", "terrain", "volume"}
                   : std::vector<std::string>{item};
  for (const std::string &k : keys)
    if (cvc::state *s = root.findDescendant(store + cvc::state::SEPARATOR + k)) {
      boost::any a = s->data();
      if (!a.empty()) {
        *found = k;
        return a;
      }
    }
  return boost::any();
}

// Drape the dataset's aerial image (a city bundle's satellite.png, decoded at <store>.satellite)
// over `g`, the node drawing the terrain `field`: each vertex's UV is its place in the field's
// extent, the image north up (UV (0, 0), its bottom-left, at the field's south-west corner), then
// turned as the node's material: { uv } says. False when the dataset has no image.
bool drape_satellite(cvc::state &root, const std::string &store, GeometryNode &g,
                     const cvc::height_field &field, const cvc::ariadne::SceneNode &n) {
  cvc::state *s = root.findDescendant(store + cvc::state::SEPARATOR + "satellite");
  std::shared_ptr<const cvc::image> img;
  if (s) {
    try {
      img = boost::any_cast<std::shared_ptr<const cvc::image>>(s->data());
    } catch (const boost::bad_any_cast &) {
    }
  }
  if (!img || img->empty() || !g.getGeometry())
    return false;
  cvc::geometry mesh = *g.getGeometry(); // copy-on-write: only its UVs are new
  const double x0 = field.x0(), y0 = field.y0();
  const double w = field.max_x() - x0, h = field.max_y() - y0;
  if (w <= 0.0 || h <= 0.0)
    return false;
  const auto &p = mesh.const_points();
  cvc::geometry::uvs_t uvs(p.size());
  for (std::size_t i = 0; i < p.size(); ++i) {
    uvs[i][0] = (p[i][0] - x0) / w;
    uvs[i][1] = (p[i][1] - y0) / h; // v = 1: north, the image's top row
  }
  transform_uvs(uvs, n);
  mesh.uvs() = std::move(uvs);
  g.setGeometry(mesh);
  g.setTexture(*img);
  return true;
}

// A mesh's axis-aligned bounds, min then max, into `out`.
void mesh_bounds(const cvc::geometry &g, double out[6]) {
  const auto lo = g.min_point(), hi = g.max_point();
  for (int k = 0; k < 3; ++k) {
    out[k] = lo[k];
    out[3 + k] = hi[k];
  }
}

// The children of a data node's group, from the dataset `store`: replaces what it drew before.
// `bounds` (min then max) gets what it drew, and the return says whether it drew anything with
// bounds (a volume does not frame).
bool draw_dataset(RealizedScene &r, SceneDataBinding &b, const std::string &store,
                  std::vector<std::string> *warnings, double bounds[6]) {
  std::shared_ptr<GraphicsNode> group = b.group.lock();
  if (!group || !r.sg || !r.app)
    return false;
  bool has_bounds = false;
  for (const std::shared_ptr<GraphicsNode> &c : b.drawn)
    group->removeGraphicsChild(c);
  b.drawn.clear();
  r.height_fields.erase(b.spec.id);

  const cvc::ariadne::SceneNode &n = b.spec;
  std::string key;
  const boost::any obj = dataset_object(cvc::state::instance(*r.app), store, n.source_item, &key);
  using world_ptr = std::shared_ptr<const cvc::world::world_geometry>;
  using geometry_ptr = std::shared_ptr<const cvc::geometry>;
  using field_ptr = std::shared_ptr<const cvc::height_field>;
  using volume_ptr = std::shared_ptr<const cvc::volume>;
  if (const world_ptr *w = boost::any_cast<world_ptr>(&obj)) {
    b.drawn = cvc::gl::add_world_geometry(*r.sg, **w, n.id + "_", group);
    const cvc::world::world_bounds wb = (*w)->bounds();
    if (!wb.empty()) {
      for (int k = 0; k < 3; ++k) {
        bounds[k] = wb.min[k];
        bounds[3 + k] = wb.max[k];
      }
      has_bounds = true;
    }
    if (std::shared_ptr<const cvc::height_field> t = (*w)->terrain()) {
      r.height_fields[n.id] = t;
      for (const std::shared_ptr<GraphicsNode> &c : b.drawn) // the terrain wears the aerial image
        if (auto *gn = dynamic_cast<GeometryNode *>(c.get());
            gn && c->getName() == n.id + "_terrain")
          drape_satellite(cvc::state::instance(*r.app), store, *gn, *t, n);
    }
  } else if (const field_ptr *f = boost::any_cast<field_ptr>(&obj)) {
    auto terrain =
        group->createChild<GeometryNode>(n.id + "_" + key, static_cast<const cvc::geometry &>(**f));
    b.drawn.push_back(terrain);
    r.height_fields[n.id] = *f;
    mesh_bounds(**f, bounds);
    has_bounds = true;
    if (terrain)
      drape_satellite(cvc::state::instance(*r.app), store, *terrain, **f, n);
  } else if (const geometry_ptr *g = boost::any_cast<geometry_ptr>(&obj)) {
    b.drawn.push_back(group->createChild<GeometryNode>(n.id + "_" + key, **g));
    mesh_bounds(**g, bounds);
    has_bounds = true;
  } else if (const volume_ptr *v = boost::any_cast<volume_ptr>(&obj)) {
    b.drawn.push_back(group->createChild<VolumeNode>(n.id + "_" + key, **v));
  } else {
    warn(warnings,
         "ari: scene node '" + n.id + "': the dataset " + store + " has " +
             (n.source_item.empty() ? "nothing to draw (a world, geometry, terrain or volume)"
                                    : "no " + n.source_item + " to draw"));
    return false;
  }
  // Its bodies are the scene world's to make (SceneWorld::sync, as a headless host's are).
  if (n.body.present && !r.world)
    warn(warnings, "ari: scene node '" + n.id +
                       "': body: needs the scene's physics: block for a "
                       "dataset's bodies");
  // The node's look, on what it drew: a material when the document gives one (else a world keeps
  // its surfaces' colours), shadows, an opaque pass. The terrain is drawn opaque, its aerial image
  // included: no per-texel alpha scan.
  for (const std::shared_ptr<GraphicsNode> &c : b.drawn) {
    const bool ground = c->getName() == n.id + "_terrain";
    if (n.has_casts_shadow)
      c->setCastsShadow(n.casts_shadow);
    if (auto *gn = dynamic_cast<GeometryNode *>(c.get())) {
      if (n.has_material)
        apply_node_material(*gn, n);
      if (n.material_opaque || ground)
        gn->setForceOpaque(true);
    }
  }
  return has_bounds;
}

} // namespace

int sync_scene_data(RealizedScene &realized, std::vector<std::string> *warnings) {
  if (!realized.app || realized.data_nodes.empty())
    return 0;
  cvc::state &root = cvc::state::instance(*realized.app);
  int rebuilt = 0;
  for (SceneDataBinding &b : realized.data_nodes) {
    cvc::state *node = root.findThroughLinks(b.path);
    if (!node)
      continue;
    const std::string store = node->fullName();
    cvc::state *status = root.findDescendant(store + cvc::state::SEPARATOR + "status");
    if (!status || status->value() != "ready")
      continue;
    cvc::state *rev = root.findDescendant(store + cvc::state::SEPARATOR + "rev");
    const std::string r = rev ? rev->value() : std::string();
    if (store == b.drawn_target && r == b.drawn_rev)
      continue;
    b.drawn_target = store; // first: a dataset that cannot be drawn is not retried every frame
    b.drawn_rev = r;
    double bounds[6];
    const bool drew = draw_dataset(realized, b, store, warnings, bounds);
    ++rebuilt;
    // frame: the main view's camera on what it drew (CameraController's frame request)
    if (drew && !b.frame_key.empty() && (b.spec.frame == "always" || !b.framed)) {
      std::ostringstream req;
      req.precision(17);
      for (int k = 0; k < 6; ++k)
        req << (k ? " " : "") << bounds[k];
      root(b.frame_key).value(req.str());
      b.framed = true;
    }
  }
  return rebuilt;
}

bool restart_scene_world(RealizedScene &realized, std::vector<std::string> *warnings) {
  if (!realized.world)
    return false;
  realized.physics_binding.reset(); // it holds the old bodies' poses
  realized.world->restart(warnings);
  realized.physics = &realized.world->world();
  realized.physics_binding = std::make_unique<physics_scene_binding>(*realized.physics);
  realized.bodies.clear();
  for (const cvc::ariadne::SceneWorldBody &b : realized.world->bodies()) {
    realized.bodies.emplace_back(b.node, b.id);
    auto node = realized.body_nodes.find(b.node);
    if (b.moves && node != realized.body_nodes.end() && node->second) {
      try {
        realized.physics_binding->bind(b.id, node->second, b.body_from_visual.data());
      } catch (const std::exception &e) {
        if (warnings)
          warnings->push_back("ari: scene node '" + b.node + "': " + e.what());
      }
    }
  }
  return true;
}

cvc::world_clock::step_result tick_scene(RealizedScene &realized, vtkRenderer *renderer,
                                         double wall_dt) {
  sync_scene_data(realized); // a data node whose dataset (re)loaded draws it this frame
  cvc::world_clock::step_result clock_step;
  // §9 time — drive the authoritative simulation clock FIRST, so every ticker below (and any
  // custom_tick reading app.world_clock().t()) sees ONE coherent world time for this frame. Opt-in:
  // only when the scene declared a `clock:` block, so a scene without one leaves app.world_clock()
  // entirely untouched (prior behaviour, and no surprise double-advance when several scenes share
  // an app — the one driver is the host that owns the clock: scene).
  if (realized.clock.present && realized.app) {
    if (!realized.clock_driver)
      realized.clock_driver =
          std::make_unique<cvc::ariadne::SceneClockDriver>(*realized.app, realized.clock);
    // Wall delta: a caller-injected fixed dt (deterministic offscreen capture) when non-negative,
    // else real elapsed since the last tick (real-time animation, no crawl under a slow frame
    // rate). Pull, advance, push.
    clock_step = realized.clock_driver->drive(wall_dt);
  }

  // physics: step the world by the quanta the clock just ran, then pose the moving bodies' nodes
  // between the last two states, before the tickers (a custom tick may read a body's node).
  if (realized.physics_binding && realized.world && realized.app) {
    cvc::world_clock &wc = realized.app->world_clock();
    try {
      cvc::gl::physics_scene_binding &view = *realized.physics_binding;
      realized.world->follow(wc, [&view] { view.tracker().capture(); });
      sync_vehicle_markers(realized, nullptr); // spawned, removed or placed again: drawn again
      view.present(wc, clock_step.alpha);
    } catch (const std::exception &e) {
      // A clock whose quantum changed under the world (pose_tracker::follow refuses it): stop
      // stepping rather than throw out of every frame.
      realized.app->log(1, std::string("ari: physics stopped: ") + e.what() + "\n");
      realized.physics_binding.reset();
    }
  }

  // §9 clip: bound `offset:`s -> the nodes' clip_planes keys, before the tickers (a volren node
  // reads its planes in tick()).
  if (realized.app)
    cvc::ariadne::sync_scene_clip(*realized.app, realized.clip);

  // shadows: { focus: camera }: the maps on a sphere `lead` radii ahead of the focal point, the
  // radius growing with the camera's distance from it (tight behind a chase camera, wide over an
  // overview). SceneGraph re-bakes only once the sphere has moved a fiftieth of its radius.
  if (realized.shadow_focus.follow_camera && realized.sg && renderer)
    if (vtkCamera *cam = renderer->GetActiveCamera()) {
      const cvc::ariadne::SceneShadowFocus &f = realized.shadow_focus;
      double e[3], c[3];
      cam->GetPosition(e);
      cam->GetFocalPoint(c);
      const double dx = c[0] - e[0], dy = c[1] - e[1], dz = c[2] - e[2];
      const double r =
          std::clamp(f.min_radius + f.per_distance * std::sqrt(dx * dx + dy * dy + dz * dz),
                     f.min_radius, std::max(f.min_radius, f.max_radius));
      const double h = std::hypot(dx, dy);
      const double cx = c[0] + (h > 1e-9 ? f.lead * r * dx / h : 0.0);
      const double cy = c[1] + (h > 1e-9 ? f.lead * r * dy / h : 0.0);
      double cz = c[2];
      if (const auto it = realized.height_fields.find(f.ground);
          !f.ground.empty() && it != realized.height_fields.end() && it->second) {
        const cvc::height_field &hf = *it->second;
        const double gz =
            hf.height(std::clamp(cx, hf.x0(), hf.max_x()), std::clamp(cy, hf.y0(), hf.max_y()));
        if (!std::isnan(gz))
          cz = gz;
      }
      realized.sg->setShadowFocus(cx, cy, cz + f.lift, r);
    }

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

  // Custom node types that registered a per-frame closure (register_scene_node_type).
  for (const std::function<void(vtkRenderer *)> &t : realized.custom_ticks)
    if (t)
      t(renderer);
  return clock_step;
}

void register_scene_node_type(const std::string &type, NodeRealizer realizer) {
  if (!realizer || is_builtin_scene_type(type)) // built-ins own their dispatch
    return;
  std::lock_guard<std::mutex> lock(node_registry_mutex());
  node_registry()[type] = std::move(realizer);
}

bool has_scene_node_type(const std::string &type) {
  std::lock_guard<std::mutex> lock(node_registry_mutex());
  return node_registry().find(type) != node_registry().end();
}

void apply_mesh_keys(GeometryNode &g, const cvc::ariadne::SceneNode &n,
                     std::vector<std::string> *warnings) {
  if (n.has_casts_shadow)
    g.setCastsShadow(n.casts_shadow);
  if (n.shader.present)
    apply_shader(g, n, warnings);
  if (n.material_opaque) // no per-texel alpha scan to pick the pass
    g.setForceOpaque(true);
}

void register_shader_preset(const std::string &name, ShaderPreset preset) {
  if (name.empty() || !preset)
    return;
  std::lock_guard<std::mutex> lock(shader_preset_mutex());
  shader_preset_registry()[name] = std::move(preset);
}

bool has_shader_preset(const std::string &name) {
  std::lock_guard<std::mutex> lock(shader_preset_mutex());
  return shader_preset_registry().find(name) != shader_preset_registry().end();
}

namespace {
// The low-memory mapper (WebGL2's, or any build's under CVCGL_LOWMEM_MAPPER=force) spells the
// view-space normal differently: its normalVCVSOutput is the varying, and the lighting reads the
// renormalized normalizedNormalVCVSOutput. So the normal-perturbing bump presets target the name
// the node's own mapper uses.
std::string fsNormal(const GeometryNode &node) {
  return node.usesLowMemoryMapper() ? "normalizedNormalVCVSOutput" : "normalVCVSOutput";
}

// Value-noise fBm ground detail; perturbs the fragment normal by the height gradient (Mikkelsen's
// tangent-free method). Needs vertexMC in world space (disableCoordinateShiftScale).
const char *GROUND_GLSL =
    "float ghash(vec2 p){ return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }\n"
    "float gnoise(vec2 p){\n"
    "  vec2 i = floor(p), f = fract(p); f = f*f*(3.0-2.0*f);\n"
    "  float a=ghash(i), b=ghash(i+vec2(1.,0.)), c=ghash(i+vec2(0.,1.)), d=ghash(i+vec2(1.,1.));\n"
    "  return mix(mix(a,b,f.x), mix(c,d,f.x), f.y);\n"
    "}\n"
    "float groundH(vec3 p){\n"
    "  vec2 q = p.xy * 0.35;\n"
    "  float f = 0.0, a = 0.5, fr = 1.0;\n"
    "  for (int i = 0; i < 5; i++){ f += a*gnoise(q*fr); a *= 0.5; fr *= 2.03; }\n"
    "  return f;\n"
    "}\n";
const char *BARK_GLSL =
    "float bhash(vec2 p){ return fract(sin(dot(p, vec2(41.3, 289.1))) * 43758.5); }\n"
    "float bnoise(vec2 p){\n"
    "  vec2 i = floor(p), f = fract(p); f = f*f*(3.0-2.0*f);\n"
    "  return mix(mix(bhash(i), bhash(i+vec2(1,0)), f.x),\n"
    "             mix(bhash(i+vec2(0,1)), bhash(i+vec2(1,1)), f.x), f.y);\n"
    "}\n"
    "float barkH(vec3 nrm, float z){\n"
    "  float ang = atan(nrm.y, nrm.x);\n"
    "  float f = 0.0;\n"
    "  f += 0.6*sin(ang*10.0 + 1.5*sin(z*0.7));\n"
    "  f += 0.3*sin(ang*23.0 + z*0.4);\n"
    "  f += 0.3*bnoise(vec2(ang*4.0, z*1.2));\n"
    "  return f;\n"
    "}\n";
} // namespace

void register_default_shader_presets() {
  // "terrain_bump": value-noise fBm ground detail (the demo's addTerrainBump).
  register_shader_preset("terrain_bump", [](GeometryNode &node) {
    node.disableCoordinateShiftScale(); // vertexMC in the shader becomes world xy
    const std::string FS_NORMAL = fsNormal(node);
    node.addVertexShaderReplacement("//VTK::Normal::Dec", "//VTK::Normal::Dec\nout vec3 gCoord;");
    node.addVertexShaderReplacement("//VTK::PositionVC::Impl",
                                    "//VTK::PositionVC::Impl\n  gCoord = vertexMC.xyz;");
    node.addFragmentShaderReplacement(
        "//VTK::Normal::Dec", std::string("//VTK::Normal::Dec\nin vec3 gCoord;\n") + GROUND_GLSL);
    node.addFragmentShaderReplacement("//VTK::Normal::Impl",
                                      std::string("//VTK::Normal::Impl\n"
                                                  "  {\n"
                                                  "    float h = groundH(gCoord);\n"
                                                  "    vec3 sS = dFdx(vertexVC.xyz);\n"
                                                  "    vec3 sT = dFdy(vertexVC.xyz);\n"
                                                  "    vec3 vn = ") +
                                          FS_NORMAL +
                                          ";\n"
                                          "    vec3 R1 = cross(sT, vn), R2 = cross(vn, sS);\n"
                                          "    float det = dot(sS, R1);\n"
                                          "    vec3 sg = sign(det) * (dFdx(h)*R1 + dFdy(h)*R2);\n"
                                          "    " +
                                          FS_NORMAL + " = normalize(abs(det)*vn - 1.4*sg);\n  }\n");
  });
  // "bark": vertical-furrow tree bark (the demo's addBark).
  register_shader_preset("bark", [](GeometryNode &node) {
    node.disableCoordinateShiftScale();
    const std::string FS_NORMAL = fsNormal(node);
    node.addVertexShaderReplacement("//VTK::Normal::Dec",
                                    "//VTK::Normal::Dec\nout vec3 bNrm;\nout vec3 bPos;");
    node.addVertexShaderReplacement(
        "//VTK::PositionVC::Impl",
        "//VTK::PositionVC::Impl\n  bNrm = normalMC; bPos = vertexMC.xyz;");
    node.addFragmentShaderReplacement(
        "//VTK::Normal::Dec",
        std::string("//VTK::Normal::Dec\nin vec3 bNrm;\nin vec3 bPos;\n") + BARK_GLSL);
    node.addFragmentShaderReplacement(
        "//VTK::Normal::Impl",
        std::string("//VTK::Normal::Impl\n"
                    "  {\n"
                    "    float h = barkH(normalize(bNrm), bPos.z);\n"
                    "    vec3 sS = dFdx(vertexVC.xyz), sT = dFdy(vertexVC.xyz), vn = ") +
            FS_NORMAL +
            ";\n"
            "    vec3 R1 = cross(sT, vn), R2 = cross(vn, sS);\n"
            "    float det = dot(sS, R1);\n"
            "    vec3 sg = sign(det) * (dFdx(h)*R1 + dFdy(h)*R2);\n"
            "    " +
            FS_NORMAL + " = normalize(abs(det)*vn - 1.2*sg);\n  }\n");
  });
}

bool verify_scene_customs(const cvc::ariadne::LoadResult &loaded,
                          std::vector<std::string> *errors) {
  bool ok = true;
  for (const cvc::ariadne::CustomRequirement &req : loaded.customs) {
    if (req.kind != cvc::ariadne::CustomRequirement::Kind::Node)
      continue; // widget/block customs are checked by the loader at load time
    if (has_scene_node_type(req.name))
      continue;
    if (req.required) {
      ok = false;
      if (errors)
        errors->push_back("ari: requires custom node '" + req.name +
                          "' which is not registered on this system");
    } else if (errors) {
      errors->push_back("ari: optional custom node '" + req.name +
                        "' is not registered — it will be skipped");
    }
  }
  return ok;
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
