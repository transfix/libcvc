#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/scene.h>
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
#include <cvc/gl/physics_scene_binding.h> // body: the nodes physics moves
#include <cvc/image/image.h>              // cvc::read_image — material: { texture: <uri> }
#include <cvc/physics/world.h>            // physics: the scene's rigid-body world
#include <cvc/utility/algorithm.h> // cvc::sdf — source: { sdf: { mesh:, dim: } } volume source
#include <cvc/volren/settings.h>
#include <cvc/volslice/settings.h>
#include <cvc/volume/bounding_box.h>
#include <cvc/volume/volume.h>
#include <exception>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
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

// A flat quad with UP normals spanning [-size/2, +size/2]² at z=0 — the built-in `plane` primitive
// (source: { plane: { size } }): a ground / shadow receiver that needs no asset. Colour comes from
// the node's material (single-colour), so no per-vertex colours are baked.
cvc::geometry make_plane(float size) {
  cvc::geometry g;
  const double h = 0.5 * static_cast<double>(size);
  const double v[4][3] = {{-h, -h, 0}, {h, -h, 0}, {h, h, 0}, {-h, h, 0}};
  for (const auto &p : v) {
    g.points().push_back({p[0], p[1], p[2]});
    g.normals().push_back({0, 0, 1});
  }
  g.tris().push_back({0, 1, 2});
  g.tris().push_back({0, 2, 3});
  return g;
}

// A box centred on the origin with edge lengths `size` (source: { box: { size } }): 24 vertices so
// each face keeps its own outward normal, 12 triangles wound counter-clockwise seen from outside.
cvc::geometry make_box(const float size[3]) {
  cvc::geometry g;
  const double h[3] = {0.5 * size[0], 0.5 * size[1], 0.5 * size[2]};
  // Per face: the outward axis, its sign, and two in-face axes u, v with u x v outward, so the
  // corners (-,-), (+,-), (+,+), (-,+) in (u, v) run counter-clockwise seen from outside.
  const int faces[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 2, 0}, {1, 0, 2}, {2, 0, 1}, {2, 1, 0}};
  const double corners[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
  for (int f = 0; f < 6; ++f) {
    const int a = faces[f][0], u = faces[f][1], v = faces[f][2];
    const double sign = (f % 2 == 0) ? 1.0 : -1.0;
    const auto base = static_cast<unsigned>(g.points().size());
    for (const auto &c : corners) {
      double p[3], n[3] = {0, 0, 0};
      p[a] = sign * h[a];
      p[u] = c[0] * h[u];
      p[v] = c[1] * h[v];
      n[a] = sign;
      g.points().push_back({p[0], p[1], p[2]});
      g.normals().push_back({n[0], n[1], n[2]});
    }
    g.tris().push_back({base, base + 1, base + 2});
    g.tris().push_back({base, base + 2, base + 3});
  }
  return g;
}

// A UV sphere of `radius` about the origin (source: { sphere: { radius } }), outward normals,
// counter-clockwise seen from outside, no degenerate triangles at the poles.
cvc::geometry make_sphere(float radius) {
  constexpr double kPi = 3.141592653589793;
  constexpr unsigned kRings = 16, kSegments = 24;
  cvc::geometry g;
  const double r = static_cast<double>(radius);
  for (unsigned i = 0; i <= kRings; ++i) {
    const double th = kPi * i / kRings; // from the north pole
    for (unsigned j = 0; j <= kSegments; ++j) {
      const double ph = 2.0 * kPi * j / kSegments;
      const double n[3] = {std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph), std::cos(th)};
      g.points().push_back({r * n[0], r * n[1], r * n[2]});
      g.normals().push_back({n[0], n[1], n[2]});
    }
  }
  const unsigned row = kSegments + 1;
  for (unsigned i = 0; i < kRings; ++i)
    for (unsigned j = 0; j < kSegments; ++j) {
      const unsigned a = i * row + j, b = a + row; // b: the same longitude, one ring south
      if (i != 0)
        g.tris().push_back({a, b, a + 1});
      if (i + 1 != kRings)
        g.tris().push_back({a + 1, b, b + 1});
    }
  return g;
}

// A procedural heightfield (source: { heightfield: {...} }), the reusable terrain primitive, as
// the world's own cvc::height_field: a res x res grid over [-size/2, size/2]^2 in XY (row 0 at -y),
// displaced in Z by the sum of the dome/wave layers. The field's mesh is what the node draws, so
// the renderer, a body resting on it and a forest planted on it see one surface: each cell split
// south-west to north-east, normals from the grid. World->[0,1]^2 UVs and optional per-vertex
// band colours ride on the mesh. All CPU, no asset. Throws cvc::geometry_error for a size that is
// not > 0.
std::shared_ptr<cvc::height_field> make_height_field(const cvc::ariadne::SceneHeightfield &hf) {
  constexpr double kTwoPi = 6.283185307179586;
  const int res = std::max(2, hf.resolution);
  const double half = 0.5 * static_cast<double>(hf.size);
  const double step = static_cast<double>(hf.size) / (res - 1);

  const auto height_at = [&](double x, double y) {
    double h = 0.0;
    for (const cvc::ariadne::SceneHeightLayer &L : hf.layers) {
      if (L.kind == "dome") {
        const double r = std::sqrt(x * x + y * y);
        const double rr = (L.radius > 0.0f) ? (r / L.radius) : 0.0;
        h += L.amplitude * std::exp(-rr * rr);
      } else if (L.kind == "wave") {
        double dx = L.direction[0], dy = L.direction[1];
        const double dl = std::sqrt(dx * dx + dy * dy);
        if (dl > 1e-9) {
          dx /= dl;
          dy /= dl;
        }
        const double d = x * dx + y * dy;
        const double wl = (L.wavelength != 0.0f) ? L.wavelength : 1.0;
        h += L.amplitude * std::sin(kTwoPi * d / wl + L.phase);
      }
    }
    return h;
  };
  const auto band_color = [&](double z) -> cvc::geometry::color_t {
    const float *col =
        hf.colors.back().color; // bands authored low->high; last = the "above all" cap
    for (const cvc::ariadne::SceneHeightColorBand &b : hf.colors)
      if (z <= b.max_height) {
        col = b.color;
        break;
      }
    return {col[0], col[1], col[2]};
  };

  std::vector<float> heights(static_cast<std::size_t>(res) * res);
  for (int j = 0; j < res; ++j)
    for (int i = 0; i < res; ++i)
      heights[static_cast<std::size_t>(j) * res + i] =
          static_cast<float>(height_at(-half + i * step, -half + j * step));
  auto field =
      std::make_shared<cvc::height_field>(res, res, -half, -half, step, step, std::move(heights));
  // UVs and band colours, one per vertex. The mesh's points and triangles follow the grid; these
  // attributes are the node's to set.
  const double inv = 1.0 / (res - 1);
  auto &uvs = field->uvs();
  uvs.resize(field->num_points());
  for (int j = 0; j < res; ++j)
    for (int i = 0; i < res; ++i)
      uvs[static_cast<std::size_t>(field->vertex_of(j, i))] = {i * inv, j * inv};
  if (!hf.colors.empty()) {
    auto &colors = field->colors();
    colors.resize(field->num_points());
    for (int j = 0; j < res; ++j)
      for (int i = 0; i < res; ++i)
        colors[static_cast<std::size_t>(field->vertex_of(j, i))] =
            band_color(field->height_at(j, i));
  }
  return field;
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

void realize_node(SceneGraph &sg, const cvc::ariadne::SceneNode &n, const std::string &bind_prefix,
                  RealizedScene &out, GraphicsNode *parent, std::vector<pending_body> &bodies,
                  const RealizeOptions &options, std::vector<std::string> *warnings) {
  std::shared_ptr<GraphicsNode> node;

  if (n.type == "geometry") {
    cvc::geometry geom;
    std::shared_ptr<const cvc::height_field> field;
    if (!read_node_geometry_and_field(sg, n, geom, field, warnings))
      return;
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
            g->setTexture(cvc::read_image(tex.path));
          } catch (const std::exception &e) {
            warn(warnings, "ari: scene node '" + n.id + "': texture '" + n.material_texture +
                               "': " + e.what());
          }
        }
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
        if (!n.volume.tf.empty())
          apply_volume_transfer_function(*vnode, n.volume.tf);
      }
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
  // shader: { preset | vertex | fragment } — the declarative GLSL surface. Applied in the shared
  // tail (not just the built-in geometry branch) so a CUSTOM node type whose realizer returns a
  // GeometryNode (e.g. an L-system forest wanting `shader: { preset: bark }`) is shaded too. A
  // non-geometry node (volume/group) silently ignores a shader block.
  if (n.shader.present)
    if (auto *g = dynamic_cast<GeometryNode *>(node.get()))
      apply_shader(*g, n, warnings);
  apply_clip(*node, n, sg.appContext(), bind_prefix, out.clip, warnings); // own coordinates
  warn_clip_capability(*node, n, warnings);
  if (n.body.present)
    bodies.push_back({&n, node}); // added once every node exists (realize_physics)

  // Children nest UNDER this node, so each child's transform is local to it.
  for (const auto &c : n.children)
    realize_node(sg, c, bind_prefix, out, node.get(), bodies, options, warnings);
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
    // applyPreset commits its baseline; the setters above need an explicit apply() to take effect.
    if (l.has_key || l.has_fill || l.has_back || l.has_warmth || l.has_environment ||
        l.has_rig_ambient || l.has_stage)
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
  const auto &P = raw.points();
  const auto &N = raw.normals();
  const auto &C = raw.colors();
  const auto rot = [up_y](double x, double y, double z, double o[3]) {
    if (up_y) {
      o[0] = x;
      o[1] = -z;
      o[2] = y;
    } else {
      o[0] = x;
      o[1] = y;
      o[2] = z;
    }
  };
  double lo[3] = {1e30, 1e30, 1e30}, hi[3] = {-1e30, -1e30, -1e30};
  for (const auto &p : P) {
    double w[3];
    rot(p[0], p[1], p[2], w);
    for (int k = 0; k < 3; ++k) {
      lo[k] = std::min(lo[k], w[k]);
      hi[k] = std::max(hi[k], w[k]);
    }
  }
  const double ext = std::max({hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]});
  const double s = ext > 0 ? static_cast<double>(height) / ext : 1.0;
  const double cx = 0.5 * (lo[0] + hi[0]), cy = 0.5 * (lo[1] + hi[1]);
  cvc::geometry g;
  for (std::size_t i = 0; i < P.size(); ++i) {
    double w[3];
    rot(P[i][0], P[i][1], P[i][2], w);
    g.points().push_back({(w[0] - cx) * s, (w[1] - cy) * s, (w[2] - lo[2]) * s});
    if (i < N.size()) {
      double nn[3];
      rot(N[i][0], N[i][1], N[i][2], nn); // a proper rotation preserves unit length
      g.normals().push_back({nn[0], nn[1], nn[2]});
    }
    if (i < C.size())
      g.colors().push_back(C[i]);
  }
  for (const auto &t : raw.tris())
    g.tris().push_back(t);
  return g;
}

// The mesh a `type: geometry` node declares: a procedural heightfield or plane, or a file/URI
// read and then `fit:`ted. Warns and returns false when there is nothing to show.
namespace {

bool read_node_geometry_and_field(SceneGraph &, const cvc::ariadne::SceneNode &n,
                                  cvc::geometry &geom,
                                  std::shared_ptr<const cvc::height_field> &field,
                                  std::vector<std::string> *warnings) {
  if (n.has_heightfield) {
    // A procedural displaced grid, no asset: the world's own height field, drawn as its mesh.
    try {
      std::shared_ptr<cvc::height_field> f = make_height_field(n.heightfield);
      geom = *f;
      field = f;
    } catch (const std::exception &e) {
      warn(warnings, "ari: scene node '" + n.id + "': heightfield: " + e.what());
      return false;
    }
  } else if (n.source_primitive == "plane") {
    geom = make_plane(n.plane_size); // a procedural ground quad — no asset
  } else if (n.source_primitive == "box") {
    geom = make_box(n.box_size);
  } else if (n.source_primitive == "sphere") {
    geom = make_sphere(n.sphere_radius);
  } else {
    if (n.source_file.empty()) {
      warn(warnings, "ari: scene node '" + n.id + "' (" + n.type +
                         ") has no source (a file/URI or a { plane } primitive)");
      return false;
    }
    cvc::ariadne::ResolvedFile src = resolve_source(n, warnings);
    if (!src.ok)
      return false;
    try {
      geom = cvc::read_geometry(src.path);
    } catch (const std::exception &e) {
      warn(warnings, "ari: scene node '" + n.id + "': " + e.what());
      return false;
    }
  }
  if (n.has_fit) {
    // `fit:` — bake center/ground/scale (and optional Y-up→Z-up) into a LOADED mesh. A built-in
    // primitive is already canonically placed + sized (source.plane.size), and up:y would rotate
    // the ground quad into a vertical wall — so ignore fit on a primitive and say so.
    if (n.has_heightfield)
      warn(warnings, "ari: scene node '" + n.id +
                         "': `fit` is ignored on a heightfield (already centered on z=0; size it "
                         "via source.heightfield.size)");
    else if (n.source_primitive.empty())
      geom = fit_to_ground(geom, n.fit_up_y, n.fit_height);
    else
      warn(warnings, "ari: scene node '" + n.id + "': `fit` is ignored on the built-in '" +
                         n.source_primitive +
                         "' primitive (already placed and sized; size it via source." +
                         n.source_primitive + ")");
  }
  return true;
}

} // namespace

bool read_node_geometry(SceneGraph &sg, const cvc::ariadne::SceneNode &n, cvc::geometry &geom,
                        std::vector<std::string> *warnings) {
  std::shared_ptr<const cvc::height_field> field;
  return read_node_geometry_and_field(sg, n, geom, field, warnings);
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
cvc::volume sdf_volume_from_mesh(cvc::app &app, const std::string &mesh_uri, int dim,
                                 bool apply_fit, bool fit_up_y, float fit_height,
                                 cvc::sdf_algorithm algorithm) {
#if CVC_ENABLE_SDF
  cvc::geometry mesh = cvc::read_geometry(mesh_uri);
  if (apply_fit)
    mesh = fit_to_ground(mesh, fit_up_y, fit_height); // stand/centre/scale the mesh before the SDF,
                                                      // so the volume is framed like the geometry
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
  (void)mesh_uri;
  (void)dim;
  (void)apply_fit;
  (void)fit_up_y;
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
                                n.fit_up_y, n.fit_height, alg);
  }
  cvc::ariadne::ResolvedFile src = resolve_source(n, warnings);
  if (!src.ok)
    throw std::runtime_error("unresolved source");
  return cvc::volume(sg.appContext(), src.path);
}

namespace {

// A rigid pose and per-axis scale from a node's world matrix (row-major 4x4): the translation, the
// rotation with each column's length divided out, and those lengths. `skewed` is set when the
// columns are not orthogonal (a non-uniform scale under a rotated parent), which a rigid body
// cannot follow.
void decompose(const double m[16], cvc::physics::vec3 &t, cvc::physics::quat &q, double scale[3],
               bool &skewed, bool &mirrored) {
  t = {m[3], m[7], m[11]};
  double r[3][3];
  for (int c = 0; c < 3; ++c) {
    const double s = std::sqrt(m[c] * m[c] + m[4 + c] * m[4 + c] + m[8 + c] * m[8 + c]);
    scale[c] = s;
    for (int k = 0; k < 3; ++k)
      r[k][c] = s > 0.0 ? m[4 * k + c] / s : (k == c ? 1.0 : 0.0);
  }
  const auto col_dot = [&](int a, int b) {
    return r[0][a] * r[0][b] + r[1][a] * r[1][b] + r[2][a] * r[2][b];
  };
  skewed = std::fabs(col_dot(0, 1)) > 1e-6 || std::fabs(col_dot(0, 2)) > 1e-6 ||
           std::fabs(col_dot(1, 2)) > 1e-6;
  const double det = r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1]) -
                     r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0]) +
                     r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
  mirrored = det < 0.0;
  // Shepperd's method.
  const double tr = r[0][0] + r[1][1] + r[2][2];
  if (tr > 0.0) {
    const double s = std::sqrt(tr + 1.0) * 2.0;
    q = {0.25 * s, (r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s};
  } else if (r[0][0] > r[1][1] && r[0][0] > r[2][2]) {
    const double s = std::sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0;
    q = {(r[2][1] - r[1][2]) / s, 0.25 * s, (r[0][1] + r[1][0]) / s, (r[0][2] + r[2][0]) / s};
  } else if (r[1][1] > r[2][2]) {
    const double s = std::sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0;
    q = {(r[0][2] - r[2][0]) / s, (r[0][1] + r[1][0]) / s, 0.25 * s, (r[1][2] + r[2][1]) / s};
  } else {
    const double s = std::sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0;
    q = {(r[1][0] - r[0][1]) / s, (r[0][2] + r[2][0]) / s, (r[1][2] + r[2][1]) / s, 0.25 * s};
  }
  const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  q = {q.w / n, q.x / n, q.y / n, q.z / n};
}

cvc::physics::vec3 rotate(const cvc::physics::quat &q, const cvc::physics::vec3 &v) {
  // v' = q v q*
  const double tx = 2 * (q.y * v.z - q.z * v.y), ty = 2 * (q.z * v.x - q.x * v.z),
               tz = 2 * (q.x * v.y - q.y * v.x);
  return {v.x + q.w * tx + (q.y * tz - q.z * ty), v.y + q.w * ty + (q.z * tx - q.x * tz),
          v.z + q.w * tz + (q.x * ty - q.y * tx)};
}

bool near_one(double s) { return std::fabs(s - 1.0) < 1e-6; }

// The physics world a scene declares (`physics:`, or the defaults when only nodes declare `body:`),
// and a body for every recorded node. Each body starts at its node's realized world pose; the
// moving ones are bound to their nodes so tick_scene draws them where physics has them. Problems
// warn (naming the node) and leave that body out; nothing here throws.
void realize_physics(SceneGraph &sg, const cvc::ariadne::ScenePhysics &spec,
                     const std::vector<pending_body> &pending, RealizedScene &out,
                     std::vector<std::string> *warnings) {
  namespace ph = cvc::physics;
  cvc::app &app = sg.appContext();

  ph::world_config cfg;
  cfg.gravity = {spec.gravity[0], spec.gravity[1], spec.gravity[2]};
  const bool have_jolt = ph::backend_available(ph::backend::jolt);
  if (spec.backend == "null") {
    cfg.backend = ph::backend::null;
  } else if (spec.backend == "jolt" || spec.backend == "auto" || spec.backend.empty()) {
    cfg.backend = have_jolt ? ph::backend::jolt : ph::backend::null;
    if (!have_jolt)
      warn(warnings, "ari: physics: this build has no Jolt; bodies are placed but never move "
                     "(the null backend)");
  } else {
    cfg.backend = have_jolt ? ph::backend::jolt : ph::backend::null;
    warn(warnings, "ari: physics: backend '" + spec.backend +
                       "' is not auto, jolt or null; using " + ph::backend_name(cfg.backend));
  }
  // One physics step is a whole number of the clock's quanta (pose_tracker::follow needs that).
  const double quantum = app.world_clock().fixed_dt();
  double steps = 1.0;
  if (spec.fixed_dt > 0.0) {
    steps = std::max(1.0, std::round(spec.fixed_dt / quantum));
    if (std::fabs(steps * quantum - spec.fixed_dt) > 1e-9)
      warn(warnings, "ari: physics: fixed_dt " + std::to_string(spec.fixed_dt) +
                         " s is not a whole number of the clock's " + std::to_string(quantum) +
                         " s quanta; using " + std::to_string(steps * quantum) + " s");
  }
  cfg.fixed_dt = steps * quantum;
  try {
    out.physics = std::make_unique<ph::world>(cfg);
  } catch (const std::exception &e) {
    warn(warnings, std::string("ari: physics: ") + e.what());
    return;
  }
  ph::world &w = *out.physics;
  out.physics_binding = std::make_unique<physics_scene_binding>(w);

  std::map<std::string, ph::material_id> materials;
  for (const cvc::ariadne::ScenePhysicsMaterial &m : spec.materials) {
    try {
      materials[m.name] = w.add_material(
          {m.name, static_cast<float>(m.friction), static_cast<float>(m.restitution)});
    } catch (const std::exception &e) {
      warn(warnings, "ari: physics: material '" + m.name + "': " + e.what());
    }
  }

  for (const pending_body &p : pending) {
    const cvc::ariadne::SceneNode &n = *p.spec;
    const cvc::ariadne::SceneBody &b = n.body;
    const auto bad = [&](const std::string &m) {
      warn(warnings, "ari: scene node '" + n.id + "': body: " + m);
    };
    auto gnode = std::dynamic_pointer_cast<GeometryNode>(p.node);
    if (!gnode || !gnode->getGeometry()) {
      bad("needs a geometry node; skipped");
      continue;
    }
    const cvc::geometry &geom = *gnode->getGeometry();

    // The node's realized world pose (its transform composed with every ancestor's).
    double m[16];
    const vtkSmartPointer<vtkMatrix4x4> wm = p.node->getWorldTransform(); // a copy, held here
    for (int k = 0; k < 16; ++k)
      m[k] = wm ? wm->GetElement(k / 4, k % 4) : (k % 5 == 0 ? 1.0 : 0.0);
    ph::vec3 t;
    ph::quat q;
    double s[3];
    bool skewed = false, mirrored = false;
    decompose(m, t, q, s, skewed, mirrored);
    if (skewed || mirrored)
      bad(std::string("the node's world transform is ") + (skewed ? "skewed" : "mirrored") +
          "; the body follows its rotation and per-axis scale only");

    std::string motion = b.motion.empty() ? (n.has_heightfield ? "static" : "dynamic") : b.motion;
    if (motion != "dynamic" && motion != "kinematic" && motion != "static") {
      bad("motion '" + motion + "' is not dynamic, kinematic or static; using dynamic");
      motion = "dynamic";
    }
    std::string shape = b.shape.empty() ? "auto" : b.shape;
    if (shape == "auto")
      shape = n.has_heightfield                ? "terrain"
              : n.source_primitive == "box"    ? "box"
              : n.source_primitive == "sphere" ? "sphere"
              : n.source_primitive == "plane"  ? "plane"
              : motion == "static"             ? "mesh"
                                               : "hull";

    ph::body_desc d;
    d.motion = motion == "static"      ? ph::motion_type::static_
               : motion == "kinematic" ? ph::motion_type::kinematic
                                       : ph::motion_type::dynamic;
    // The shape's centre in the node's (scaled) frame: the body sits there, and the node is drawn
    // at body pose x translate(-centre) x scale.
    ph::vec3 centre;
    const cvc::bounding_box e = geom.extents();
    const double half[3] = {0.5 * (e.maxx - e.minx) * s[0], 0.5 * (e.maxy - e.miny) * s[1],
                            0.5 * (e.maxz - e.minz) * s[2]};
    const ph::vec3 mid{0.5 * (e.minx + e.maxx) * s[0], 0.5 * (e.miny + e.maxy) * s[1],
                       0.5 * (e.minz + e.maxz) * s[2]};
    bool placed_by_grid = false;
    if (shape == "terrain") {
      auto it = out.height_fields.find(n.id);
      if (it == out.height_fields.end()) {
        bad("shape 'terrain' needs a heightfield source; skipped");
        continue;
      }
      if (d.motion != ph::motion_type::static_)
        bad("a terrain is static; making it static");
      d.motion = ph::motion_type::static_;
      const bool rotated = std::fabs(std::fabs(q.w) - 1.0) > 1e-9;
      if (rotated || !near_one(s[0]) || !near_one(s[1]) || !near_one(s[2]))
        bad("a terrain collides unrotated and unscaled, at the node's position; draw it without "
            "rotation or scale so the ground you see is the ground bodies rest on");
      std::shared_ptr<const cvc::height_field> field = it->second;
      if (t.x != 0.0 || t.y != 0.0 || t.z != 0.0) {
        // The same grid moved to the node's position (the field itself stays the node's own).
        std::vector<float> h = field->heights();
        for (float &z : h)
          z += static_cast<float>(t.z);
        field = std::make_shared<cvc::height_field>(field->rows(), field->cols(), field->x0() + t.x,
                                                    field->y0() + t.y, field->dx(), field->dy(),
                                                    std::move(h), field->cell_materials());
      }
      d.shape = ph::terrain{field};
      placed_by_grid = true;
    } else if (shape == "box") {
      if (n.source_primitive == "box")
        d.shape = ph::box{
            {0.5 * n.box_size[0] * s[0], 0.5 * n.box_size[1] * s[1], 0.5 * n.box_size[2] * s[2]}};
      else {
        d.shape =
            ph::box{{std::max(half[0], 1e-3), std::max(half[1], 1e-3), std::max(half[2], 1e-3)}};
        centre = mid;
      }
    } else if (shape == "sphere") {
      if (!near_one(s[1] / s[0]) || !near_one(s[2] / s[0]))
        bad("a sphere scales uniformly; using the largest scale");
      const double smax = std::max(s[0], std::max(s[1], s[2]));
      if (n.source_primitive == "sphere") {
        d.shape = ph::sphere{n.sphere_radius * smax};
      } else {
        d.shape = ph::sphere{std::max({half[0], half[1], half[2], 1e-3})};
        centre = mid;
      }
    } else if (shape == "plane") {
      // A thin slab under the plane, its top face on the plane.
      const double thick = 0.05;
      d.shape = ph::box{{0.5 * n.plane_size * s[0], 0.5 * n.plane_size * s[1], thick}};
      centre = {0.0, 0.0, -thick};
    } else if (shape == "hull" || shape == "mesh") {
      if (shape == "mesh" && d.motion != ph::motion_type::static_) {
        bad("a mesh body is static; using its convex hull");
        shape = "hull";
      }
      if (shape == "hull") {
        ph::convex_hull h;
        for (const auto &pt : geom.const_points())
          h.points.push_back({pt[0] * s[0], pt[1] * s[1], pt[2] * s[2]});
        d.shape = std::move(h);
      } else {
        auto scaled = std::make_shared<cvc::geometry>(geom);
        if (!near_one(s[0]) || !near_one(s[1]) || !near_one(s[2]))
          for (auto &pt : scaled->points())
            pt = {pt[0] * s[0], pt[1] * s[1], pt[2] * s[2]};
        d.shape = ph::mesh{scaled};
      }
    } else {
      bad("shape '" + shape + "' is not auto, box, sphere, hull, mesh or terrain; skipped");
      continue;
    }

    if (!placed_by_grid) {
      d.position = t + rotate(q, centre);
      d.rotation = q;
    }
    d.mass = b.mass;
    d.density = b.density;
    if (!b.material.empty()) {
      auto mt = materials.find(b.material);
      if (mt != materials.end())
        d.material = mt->second;
      else
        bad("no physics material called '" + b.material + "'; using the default");
    }
    d.linear_velocity = {b.velocity[0], b.velocity[1], b.velocity[2]};
    d.angular_velocity = {b.angular_velocity[0], b.angular_velocity[1], b.angular_velocity[2]};
    if (b.has_linear_damping)
      d.linear_damping = b.linear_damping;
    if (b.has_angular_damping)
      d.angular_damping = b.angular_damping;

    ph::body_id id;
    try {
      id = w.add_body(d);
    } catch (const std::exception &ex) {
      bad(ex.what());
      continue;
    }
    out.bodies.emplace_back(n.id, id);
    if (d.motion == ph::motion_type::static_)
      continue; // never moves: the node stays as realized
    // body_from_visual = translate(-centre) x scale, row-major.
    const double v[16] = {s[0], 0, 0,    -centre.x, 0, s[1], 0, -centre.y,
                          0,    0, s[2], -centre.z, 0, 0,    0, 1};
    out.physics_binding->bind(id, p.node, v);
  }
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
  out.clock = scene.clock;
  // Resolve the clock's key paths to absolute state paths now, with the host bind prefix — the SAME
  // rule widget binds use (resolve_bind) — so the clock and a sim_transport slider on `sim.speed`
  // land on one key. Resolved once here (like the visibility bindings), so tick_scene stays prefix-
  // free. An empty key resolves to empty (stays "not bound").
  out.clock.speed_key = cvc::ariadne::resolve_bind(bind_prefix, scene.clock.speed_key);
  out.clock.paused_key = cvc::ariadne::resolve_bind(bind_prefix, scene.clock.paused_key);
  out.clock.time_key = cvc::ariadne::resolve_bind(bind_prefix, scene.clock.time_key);
  out.clock.tick_key = cvc::ariadne::resolve_bind(bind_prefix, scene.clock.tick_key);
  out.created.reserve(scene.nodes.size());
  std::vector<pending_body> bodies;
  for (const auto &n : scene.nodes) {
    realize_node(sg, n, bind_prefix, out, /*parent=*/nullptr, bodies, options, warnings);
    out.created.push_back(n.id);
  }
  // physics: and the nodes' body:s, once every node (and every terrain field) exists. A physics
  // scene runs on the scene clock, so it drives app.world_clock() as `clock: {}` would.
  if (scene.physics.present || !bodies.empty()) {
    realize_physics(sg, scene.physics, bodies, out, warnings);
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
      }
    }
  }
  if (scene.has_chrome) // strip/show the SceneGraph diagnostic chrome (grid/axis/bbox)
    sg.setDiagnosticChromeVisible(scene.chrome_visible);
  return out;
}

cvc::world_clock_keys scene_clock_keys(const cvc::ariadne::SceneClock &clock) {
  cvc::world_clock_keys k;
  const std::pair<const std::string *, std::string> named[] = {{&clock.speed_key, ".speed"},
                                                               {&clock.paused_key, ".paused"},
                                                               {&clock.time_key, ".time"},
                                                               {&clock.tick_key, ".tick"}};
  for (const auto &n : named) {
    const std::string &key = *n.first;
    const std::string &leaf = n.second;
    if (key.size() > leaf.size() && key.compare(key.size() - leaf.size(), leaf.size(), leaf) == 0) {
      k = cvc::world_clock_keys::under(key.substr(0, key.size() - leaf.size()));
      break;
    }
  }
  k.speed = clock.speed_key;
  k.paused = clock.paused_key;
  k.time = clock.time_key;
  k.tick = clock.tick_key;
  return k;
}

cvc::world_clock::step_result tick_scene(RealizedScene &realized, vtkRenderer *renderer,
                                         double wall_dt) {
  cvc::world_clock::step_result clock_step;
  // §9 time — drive the authoritative simulation clock FIRST, so every ticker below (and any
  // custom_tick reading app.world_clock().t()) sees ONE coherent world time for this frame. Opt-in:
  // only when the scene declared a `clock:` block, so a scene without one leaves app.world_clock()
  // entirely untouched (prior behaviour, and no surprise double-advance when several scenes share
  // an app — the one driver is the host that owns the clock: scene).
  if (realized.clock.present && realized.app) {
    if (!realized.clock_binding) {
      cvc::app &app = *realized.app;
      cvc::world_clock &wc = app.world_clock();
      const cvc::ariadne::SceneClock &c = realized.clock;
      // The declared initial rate and pause go on the clock before the binding exists: the binding
      // seeds every still-unset key from the clock, so the sim_transport widget shows them on frame
      // 0, and they stand for good on a lane the scene left unbound (an empty key). The key paths
      // were resolved at realize time with the host bind prefix, so a sim_transport slider and this
      // clock share one key.
      wc.set_scale(c.scale);
      if (c.paused)
        wc.set_mode(cvc::world_clock::mode::paused);
      else if (wc.current_mode() == cvc::world_clock::mode::paused)
        wc.set_mode(cvc::world_clock::mode::live);
      realized.clock_binding =
          std::make_unique<cvc::world_clock_binding>(app, wc, scene_clock_keys(c));
    }
    // Wall delta: a caller-injected fixed dt (deterministic offscreen capture) when non-negative,
    // else real elapsed since the last tick (real-time animation, no crawl under a slow frame
    // rate).
    double dt = wall_dt;
    if (dt < 0.0) {
      const auto now = std::chrono::steady_clock::now();
      dt = realized.clock_primed ? std::chrono::duration<double>(now - realized.clock_last).count()
                                 : 0.0;
      realized.clock_last = now;
      realized.clock_primed = true;
    }
    // pull (speed / paused / mode / lockstep / step requests), advance, push (time, tick, ...).
    clock_step = realized.clock_binding->drive(dt);
  }

  // physics: step the world by the quanta the clock just ran, then pose the moving bodies' nodes
  // between the last two states, before the tickers (a custom tick may read a body's node).
  if (realized.physics_binding && realized.app) {
    cvc::world_clock &wc = realized.app->world_clock();
    try {
      realized.physics_binding->follow(wc);
      realized.physics_binding->present(wc, clock_step.alpha);
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
// The GLES fragment shader spells the view-space normal differently (the WebGL backend
// renormalizes), so the normal-perturbing bump presets must target the right name per platform.
#ifdef __EMSCRIPTEN__
constexpr const char *FS_NORMAL = "normalizedNormalVCVSOutput";
#else
constexpr const char *FS_NORMAL = "normalVCVSOutput";
#endif

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
