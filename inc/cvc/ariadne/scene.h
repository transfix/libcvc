#ifndef CVC_ARIADNE_SCENE_H
#define CVC_ARIADNE_SCENE_H

// Ariadne scene spec (roadmap §9). The backend-neutral DATA the loader parses from
// a .ari `scene:` block — the VTK scene declared in the same document as the
// widgets. It is pure data (no VTK): a GL backend realizes it into a live
// cvc::gl::SceneGraph (cvc::gl::ariadne::realize_scene); a non-GL backend ignores
// it. Every realized node is a cvc::state_object, so a node IS a state subtree and
// each prop a reactive state key (§9.1) — this struct just says what to create.

#include <string>
#include <vector>

namespace cvc {
namespace ariadne {

// A transfer-function control point over the RAW value domain (mirrors
// cvc::volren::transfer_point, shared by volren + volslice). color is r,g,b,a in [0,1].
struct SceneTFPoint {
  float value = 0.0f;
  float color[4] = {0.0f, 0.0f, 0.0f, 0.0f};
};

// A transfer function for a volume renderer. Empty ⇒ nothing drawn.
struct SceneTransferFunction {
  std::vector<SceneTFPoint> points;
  bool auto_domain = true;   // bake over the volume's data range (vs an explicit window)
  bool has_window = false;   // explicit value window instead of the data range
  float window_min = 0.0f, window_max = 0.0f;
  bool empty() const { return points.empty(); }
};

// One volren isosurface (mirrors cvc::volren::isosurface).
struct SceneIsosurface {
  float value = 0.0f;
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
  bool nearest_filter = false;   // filter: nearest | linear
  bool opacity_correction = false;
  SceneTransferFunction tf;
};

// One scene node (§9.2/§9.3). `type` is geometry | volume | volren | volslice |
// group | light. Fields not meaningful to a type are simply unused.
struct SceneNode {
  std::string id;                 // node name → <prefix>.graphics.root.children.<id>
  std::string type = "geometry";  // node kind
  std::string source_file;        // source: { file: ... } (geometry/volume load path)

  bool has_material = false;
  float color[3] = {0.8f, 0.8f, 0.9f};
  float ambient = 0.2f;
  float diffuse = 0.8f;

  bool has_transform = false;
  float position[3] = {0.0f, 0.0f, 0.0f};
  float rotation[3] = {0.0f, 0.0f, 0.0f}; // Euler degrees
  float scale[3] = {1.0f, 1.0f, 1.0f};

  bool has_volren = false;   // volren: {...} present (type: volren)
  SceneVolRen volren;
  bool has_volslice = false; // volslice: {...} present (type: volslice)
  SceneVolSlice volslice;

  std::string visible_bind;    // visible: <state path> (or, later, an expression)
  bool visible_default = true; // initial visibility when no bind / before first read

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
  float cone = 45.0f;                    // spot/fill cone angle (degrees)
  float azimuth = 0.0f;                  // directional: compass bearing (0 = +Y toward +X)
  float elevation = 45.0f;               // directional: degrees above the horizon
  float intensity = 1.0f;
  float color[3] = {1.0f, 1.0f, 1.0f};   // light colour (default white)
  std::string rig; // rig: <preset> (StageLighting), mutually exclusive with the above
};

struct Scene {
  std::vector<SceneNode> nodes;
  std::vector<SceneLight> lights;
  bool has_shadows = false;
  bool shadows_enabled = false;
  bool any() const { return !nodes.empty() || !lights.empty() || has_shadows; }
};

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_SCENE_H
