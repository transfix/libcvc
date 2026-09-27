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

  bool has_material = false;
  float color[3] = {0.8f, 0.8f, 0.9f};
  float ambient = 0.2f;
  float diffuse = 0.8f;
  bool use_single_color = true; // material: { single_color: false } to keep per-vertex colors
  bool has_specular = false;    // gate the specular pair so an unset material keeps GeometryNode's
  float specular = 0.25f;       //   defaults; both applied only when the material states specular
  float specular_power = 24.0f;

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

struct Scene {
  std::vector<SceneNode> nodes;
  std::vector<SceneLight> lights;
  bool has_shadows = false;
  bool shadows_enabled = false;
  bool has_shadow_resolution = false; // shadows: { resolution: <px> } — bigger = crisper map
  int shadow_resolution = 1024;
  bool has_shadow_interval = false; // shadows: { update_interval: <frames> } — 1 = bake every frame
  int shadow_interval = 1;
  bool has_chrome = false;    // chrome: <bool> — the SceneGraph diagnostic grid/axis/bbox chrome
  bool chrome_visible = true; // default on (the SceneGraph default); `chrome: false` strips it
  bool any() const { return !nodes.empty() || !lights.empty() || has_shadows || has_chrome; }
};

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_SCENE_H
