#ifndef CVC_ARIADNE_SCENE_H
#define CVC_ARIADNE_SCENE_H

// Ariadne scene spec (roadmap §9). The backend-neutral DATA the loader parses from
// a .ari `scene:` block — the VTK scene declared in the same document as the
// widgets. It is pure data (no VTK): a GL backend realizes it into a live
// cvc::gl::SceneGraph (cvc::gl::ariadne::realize_scene); a non-GL backend ignores
// it. Every realized node is a cvc::state_object, so a node IS a state subtree and
// each prop a reactive state key (§9.1) — this struct just says what to create.

#include <cvc/ariadne/value.h>
#include <string>
#include <vector>

namespace cvc {
namespace ariadne {

// A transfer-function control point over the RAW value domain (mirrors
// cvc::volren::transfer_point, shared by volren + volslice). `value` is double to
// match the renderer's value domain; color is r,g,b,a in [0,1].
struct SceneTFPoint {
  double value = 0.0;
  float color[4] = {0.0f, 0.0f, 0.0f, 0.0f};
};

// A transfer function for a volume renderer. Empty ⇒ nothing drawn.
struct SceneTransferFunction {
  std::vector<SceneTFPoint> points;
  bool auto_domain = true; // bake over the volume's data range (vs an explicit window)
  bool has_window = false; // explicit value window instead of the data range
  double window_min = 0.0, window_max = 0.0; // raw value domain — double like the renderer
  bool empty() const { return points.empty(); }
};

// One volren isosurface (mirrors cvc::volren::isosurface).
struct SceneIsosurface {
  double value = 0.0; // raw value domain — double like cvc::volren::isosurface::value
  float opacity = 1.0f;
  float color[3] = {1.0f, 1.0f, 1.0f};
  float shininess = 10.0f;
};

// A volren directional light (cvc::volren::render_settings::lights) — the SOFTWARE
// raycaster's own lights, SEPARATE from the scene `lights:` array (which drives VTK /
// StageLighting). `direction` points toward the light.
struct SceneVolRenLight {
  float color[3] = {1.0f, 1.0f, 1.0f};
  float direction[3] = {0.0f, 0.0f, 1.0f};
};

// `type: volren` settings. A non-blank render needs a non-empty `tf` OR a non-empty
// `isosurfaces` list, and (for a shaded surface) a light or ambient > 0.
struct SceneVolRen {
  bool shaded = true;
  bool unshaded = false;
  bool distance_field = false;
  int steps = 512;
  float ambient = 0.0f;
  float resolution_scale = 0.5f;
  std::string backend = "cpu"; // cpu | cuda | automatic
  SceneTransferFunction tf;
  std::vector<SceneIsosurface> isosurfaces;
  std::vector<SceneVolRenLight> lights;
};

// `type: volslice` settings. Always unlit; needs a non-empty `tf` with opacity > 0.
struct SceneVolSlice {
  float quality = 0.5f;
  int max_planes = 1000;
  float near_plane = 0.0f;
  bool nearest_filter = false; // filter: nearest | linear
  bool opacity_correction = false;
  SceneTransferFunction tf;
};

// `type: volume` settings — the VTK GPU volume mapper (cvc::gl::VolumeNode), distinct from the
// SOFTWARE volren/volslice. A volume's colour comes from its transfer function; TF point values are
// ABSOLUTE scalar values (the VTK mapper takes the raw domain directly, so `auto_domain`/`window`
// from the shared TF vocabulary do NOT apply to a volume node). The lighting coefficients come from
// the node `material:` block (ambient/diffuse, plus specular when the material states it). Empty tf
// ⇒ the node keeps VolumeNode's default grayscale TF.
struct SceneVolume {
  bool has_shaded = false; // gate so an unset `shaded` keeps VolumeNode's tuned default
  bool shaded = true;
  SceneTransferFunction tf;
};

// One additive layer of a procedural heightfield (source: { heightfield: { layers: [...] } }).
// The surface height at (x,y) is the SUM of its layers' contributions. Two kinds:
//   dome — a radial Gaussian bump centered on the field: amplitude * exp(-(r/radius)²).
//   wave — a travelling sine along `direction`: amplitude * sin(2π·(d·dir)/wavelength + phase).
// (Mesh-level shape only — fine surface detail belongs in a fragment shader, not the grid.)
struct SceneHeightLayer {
  std::string kind;                  // "dome" | "wave"
  float amplitude = 1.0f;            // dome peak height / wave amplitude
  float radius = 1.0f;               // dome: Gaussian falloff radius (world units)
  float wavelength = 1.0f;           // wave: crest-to-crest distance (world units)
  float direction[2] = {1.0f, 0.0f}; // wave: travel direction in XY (normalized on use)
  float phase = 0.0f;                // wave: phase offset (radians)
};

// A height band for per-vertex heightfield colouring: the first band whose `max_height` is ≥ the
// vertex height wins (bands are tested in declared order, so author them low→high). Empty ⇒ no
// per-vertex colour (the node `material:` single colour applies instead).
struct SceneHeightColorBand {
  float max_height = 0.0f;
  float color[3] = {0.5f, 0.5f, 0.5f};
};

// A procedural heightfield geometry source (source: { heightfield: {...} }) — a flat grid in XY,
// centered on the origin, displaced in Z by the sum of `layers`, with world→[0,1]² UVs and (if
// `colors` is non-empty) per-vertex band colours. Normals are computed from the height gradient.
// The reusable terrain primitive: no asset, authored entirely in the document.
struct SceneHeightfield {
  float size = 1.0f;   // world edge length (the grid spans [-size/2, size/2]² in XY)
  int resolution = 64; // grid vertices per edge (resolution² points, clamped ≥ 2)
  std::vector<SceneHeightLayer> layers;
  std::vector<SceneHeightColorBand> colors;
};

// A GLSL shader surface on a geometry node (§9 — shaders). The DSL face of GeometryNode's shader-
// replacement API (addVertexShaderReplacement / addFragmentShaderReplacement + the coordinate-
// shift-scale toggle): a node can either name a host-registered PRESET (an effect like a terrain
// bump or tree bark, whose GLSL — and any platform gate like the GLES normal-name difference —
// lives in C++ where it belongs) or splice raw GLSL at named VTK injection markers. Both compose
// (preset first, then the inline stages). This lets a demo that needed host C++ to shade its mesh
// be authored entirely in the .ari and run under the generic ariadne_hello host. GL-specific, so
// the realizer (cvcGL) applies it; the loader only carries it as neutral data.
struct SceneShaderStage {
  std::string at;   // the VTK marker to replace (e.g. "//VTK::Normal::Impl")
  std::string code; // the GLSL spliced in place of (and re-including) that marker
};
struct SceneShader {
  bool present = false;             // was a `shader:` block given?
  bool disable_coord_shift = false; // call GeometryNode::disableCoordinateShiftScale() first
                                    // (so vertexMC is world space — the bump/bark shaders need it)
  std::string preset;                     // a host-registered named effect (empty = none)
  std::vector<SceneShaderStage> vertex;   // vertex-shader replacements, applied in order
  std::vector<SceneShaderStage> fragment; // fragment-shader replacements, applied in order
};

// One scene node (§9.2/§9.3). `type` is geometry | volume | volren | volslice |
// group | light. Fields not meaningful to a type are simply unused.
struct SceneNode {
  std::string id;                // node name → <prefix>.graphics.root.children.<id>
  std::string type = "geometry"; // node kind
  std::string source_file;       // source: { file: ... } (geometry/volume load path)

  // A built-in PROCEDURAL primitive as a geometry node's source, instead of source: { file }.
  // Empty = use source_file. "plane" = a flat quad with UP normals, spanning
  // [-plane_size/2, +plane_size/2]² at z=0 — a ground / shadow receiver that needs no asset.
  std::string source_primitive;
  float plane_size = 1.0f; // "plane": full edge length

  // A procedural HEIGHTFIELD as a geometry node's source: source: { heightfield: {...} }. When
  // present (has_heightfield), the realizer generates a displaced grid mesh and source_file /
  // source_primitive are ignored. The reusable terrain primitive (dome/wave layers + band colours).
  bool has_heightfield = false;
  SceneHeightfield heightfield;

  // A VOLUME computed as a signed distance field from a MESH (for volren/volslice), instead of
  // loading a pre-baked volume with source: { file }. `source: { sdf: { mesh: <uri>, dim: N } }` —
  // the realizer loads the mesh and runs cvc::sdf over an N³ grid framed to the mesh extents. Empty
  // = not an SDF source. Needs a CVC_ENABLE_SDF build (the realizer warns and skips the node
  // without it). This is how a demo turns a mesh (e.g. stanford.bunny) into a volume with no asset
  // file.
  std::string source_sdf_mesh;
  int sdf_dim = 64;

  bool has_material = false;
  float color[3] = {0.8f, 0.8f, 0.9f};
  float ambient = 0.2f;
  float diffuse = 0.8f;
  bool use_single_color = true; // material: { single_color: false } to keep per-vertex colors
  bool has_specular = false;    // gate the specular pair so an unset material keeps GeometryNode's
  float specular = 0.25f;       //   defaults; both applied only when the material states specular
  float specular_power = 24.0f;

  // material: { texture: <uri> } — an image sampled through the geometry's UVs (GeometryNode::
  // setTexture). Empty = untextured. Meaningful only for a geometry node whose mesh carries UVs
  // (e.g. a heightfield, or a loaded mesh with texcoords); a mesh with no UVs is unaffected.
  std::string material_texture;

  // fit: bake a "stand this mesh on the ground" normalization INTO the loaded geometry — center it
  // in XY, sit its base on z=0, and scale its tallest extent to fit_height; optionally first rotate
  // a canonical Y-up mesh to Z-up. Geometry-only (a node transform composes on top). For placing a
  // mesh authored at an arbitrary origin/scale (e.g. the Stanford bunny) predictably on a ground.
  bool has_fit = false;
  float fit_height = 1.0f;
  bool fit_up_y = false; // fit: { up: y } — rotate the Y-up mesh +90° about X to Z-up first

  bool has_transform = false;
  float position[3] = {0.0f, 0.0f, 0.0f};
  float rotation[3] = {0.0f, 0.0f, 0.0f}; // Euler degrees
  float scale[3] = {1.0f, 1.0f, 1.0f};

  bool has_volren = false; // volren: {...} present (type: volren)
  SceneVolRen volren;
  bool has_volslice = false; // volslice: {...} present (type: volslice)
  SceneVolSlice volslice;
  bool has_volume = false; // volume: {...} present (type: volume) — TF + shading for the VTK mapper
  SceneVolume volume;

  SceneShader shader; // shader: {...} — GLSL replacements / a named preset (geometry nodes)

  std::string visible_bind;    // visible: <state path> (or, later, an expression)
  bool visible_default = true; // initial visibility when no bind / before first read

  // Every node key the built-in parser did NOT consume, as a neutral map — the
  // config bag a CUSTOM node type (registered via cvc::gl::ariadne::
  // register_scene_node_type) reads. Empty for the built-in types.
  Value props;

  std::vector<SceneNode> children; // nested nodes (…children.<id>.children.<child>)
};

// A scene light (§9.2). Either an explicit light (kind/pos/target/cone/intensity/
// color, or azimuth/elevation for a directional sun) or a named StageLighting preset
// rig. Realized by cvc::gl::ariadne::realize_scene into a cvc::gl::LightNode (or a
// StageLighting rig). A directional light uses azimuth/elevation (a compass sun),
// not pos/target — those drive spot/fill.
struct SceneLight {
  std::string id;
  std::string kind = "directional"; // directional | spot | fill
  float pos[3] = {0.0f, 0.0f, 0.0f};
  float target[3] = {0.0f, 0.0f, 0.0f};
  float cone = 45.0f;      // spot/fill cone angle (degrees)
  float azimuth = 0.0f;    // directional: compass bearing (0 = +Y toward +X)
  float elevation = 45.0f; // directional: degrees above the horizon
  float intensity = 1.0f;
  float color[3] = {1.0f, 1.0f, 1.0f}; // light colour (default white)
  std::string rig; // rig: <preset> (StageLighting), mutually exclusive with the above

  // rig: TUNING (only meaningful when `rig` is set). Each has_* gates one StageLighting setter so
  // an unset knob keeps the preset's value — a faithful rig without re-typing the whole setup.
  bool has_stage = false; // stage: { center: [x,y,z], radius } — the aimed spot's target volume
  float stage_center[3] = {0.0f, 0.0f, 0.0f};
  float stage_radius = 1.0f;
  bool has_key = false; // key: { intensity, azimuth, elevation, cone } — the main light
  float key_intensity = 1.0f, key_azimuth = 0.0f, key_elevation = 45.0f, key_cone = 40.0f;
  bool has_fill = false;
  float fill = 0.5f; // fill: <intensity>
  bool has_back = false;
  float back = 0.5f; // back: <intensity>
  bool has_warmth = false;
  float warmth = 0.0f; // warmth: <0..1>
  bool has_environment = false;
  float environment = 0.0f; // environment: <intensity> (lifts what the cones miss)
  bool has_rig_ambient = false;
  float rig_ambient = 0.2f; // ambient: <0..1> (shadowed sides readable)
};

// A simulation clock for the scene (§9 — time). Declares how the authoritative
// cvc::world_clock (app.world_clock(), inc/cvc/core/world_clock.h — *simulation* time, distinct
// from wall time and render cadence) is driven each frame and which state keys steer it. This is
// the declarative face of libcvc's time discipline: a `clock:` block makes the reusable
// sim_transport.ari (the Paused checkbox + Speed slider, binding sim.paused / sim.speed) actually
// pause and rate-scale the animation with NO host C++, and publishes the live world seconds to
// `time_key` (default sim.time) so widgets, Python or another scene can read one coherent clock.
// The key paths resolve with the SAME rule as widget binds (cvc::ariadne::resolve_bind against the
// host prefix), so a clock's `sim.speed` and a slider's `bind: sim.speed` are one key. Opt-in:
// without a `clock:` block the host leaves app.world_clock() untouched (unchanged behaviour).
struct SceneClock {
  bool present = false; // was a `clock:` block given? (gates driving — opt-in)
  double scale = 1.0;   // initial world-sec per wall-sec (seeds speed_key when it is still unset)
  bool paused = false;  // initial mode (seeds paused_key when it is still unset)
  // State-key bindings, resolved against the host prefix like widget binds. The defaults wire a
  // bare `clock: {}` straight to sim_transport.ari. An EMPTY key disables that lane: no steer from
  // speed_key/paused_key (the initial scale/paused stands), no publish to time_key/tick_key.
  std::string speed_key = "sim.speed";   // read each frame → world_clock scale (0 pauses)
  std::string paused_key = "sim.paused"; // read each frame → paused vs live mode
  std::string time_key = "sim.time";     // published each frame: world seconds (world_clock::t())
  std::string tick_key;                  // optional: published tick count (empty ⇒ not published)
};

struct Scene {
  std::vector<SceneNode> nodes;
  std::vector<SceneLight> lights;
  SceneClock clock; // §9 time: the simulation clock (clock.present gates it) — see SceneClock
  bool has_shadows = false;
  bool shadows_enabled = false;
  bool has_shadow_resolution = false; // shadows: { resolution: <px> } — bigger = crisper map
  int shadow_resolution = 1024;
  bool has_shadow_interval = false; // shadows: { update_interval: <frames> } — 1 = bake every frame
  int shadow_interval = 1;
  bool has_chrome = false;    // chrome: <bool> — the SceneGraph diagnostic grid/axis/bbox chrome
  bool chrome_visible = true; // default on (the SceneGraph default); `chrome: false` strips it
  // background: [r,g,b] (a solid colour) OR { top: [r,g,b], bottom: [r,g,b] } (a vertical
  // gradient). A VIEW property (not the scene graph), applied by the host to the renderer.
  bool has_background = false;
  bool background_gradient = false; // false = solid (background_top); true = top→bottom gradient
  float background_top[3] = {0.09f, 0.10f, 0.12f};
  float background_bottom[3] = {0.02f, 0.02f, 0.03f};
  bool any() const {
    return !nodes.empty() || !lights.empty() || has_shadows || has_chrome || has_background ||
           clock.present;
  }
};

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_SCENE_H
