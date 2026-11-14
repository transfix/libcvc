#ifndef CVC_GL_ARIADNE_SCENE_REALIZE_H
#define CVC_GL_ARIADNE_SCENE_REALIZE_H

// Realize a parsed Ariadne Scene (roadmap §9) into a live cvc::gl::SceneGraph.
// The backend-neutral Scene spec (cvc::ariadne::Scene, inc/cvc/ariadne/scene.h) is
// GL-agnostic data; this is the cvcGL side that turns it into real GraphicsNodes.
// A geometry node loads its source and gets its transform + single-colour material
// + initial visibility; a group node is an empty hierarchy node. Because every
// SceneGraph node is a cvc::state_object (§9.1), creating a node writes its state
// subtree — so a widget or expression can then drive its props with no extra glue.
//
// §9 increment 2 — dynamic `visible:` sync. When a node's visibility is bound to a
// state path (`visible: demo.show_mesh`), realize resolves that path (with the SAME
// rule the widget Runtime uses, so a checkbox on the same path and the node share
// one key), seeds the node's initial visibility from it, and records a
// cvc::ariadne::SceneVisibilityBinding. The host then calls
// cvc::ariadne::sync_scene_visibility(app, realized.visibility) each frame to mirror
// the live value into the node's own `.visible` key — the node's state_object
// machinery does the setVisible. The binding holds only state-path strings (no node
// pointer), so it can never dangle into a torn-down node.

#include <chrono>                    // RealizedScene clock-drive wall-dt timestamp
#include <cstdint>                   // RealizeOptions::lod_min_tris
#include <cvc/ariadne/bind.h>        // SceneVisibilityBinding
#include <cvc/ariadne/loader.h>      // LoadResult / CustomRequirement (verify_scene_customs)
#include <cvc/ariadne/scene.h>       // SceneClock (RealizedScene holds the resolved clock by value)
#include <cvc/ariadne/scene_world.h> // RealizedScene runs the scene's world (core libcvc)
#include <cvc/core/types.h>          // cvc::sdf_algorithm (sdf_volume_from_mesh)
#include <cvc/core/world_clock.h>    // tick_scene returns the frame's world_clock::step_result
#include <cvc/core/world_clock_binding.h> // RealizedScene drives the clock through a binding
#include <cvc/gl/StageLighting.h>         // RealizedScene owns any StageLighting rigs
#include <cvc/gl/physics_scene_binding.h> // RealizedScene draws its bodies through one
#include <cvc/physics/world.h>            // RealizedScene::physics, the scene's world
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
class app;
class geometry;
class height_field;
class volume;
} // namespace cvc

class vtkRenderer;

namespace cvc {
namespace ariadne {
struct Scene;
struct SceneNode;
} // namespace ariadne
namespace gl {
class SceneGraph;
class GraphicsNode;
class GeometryNode; // ShaderPreset applies to a GeometryNode (forward-decl: used only by reference)
class LodGraphicsNode;
class LodPyramidCache;
class VolRenNode;
class VolSliceNode;

namespace ariadne {

// A `source: { data: <name> }` node (cvc/ariadne/data.h): the node is an empty group at realize,
// and tick_scene (sync_scene_data) rebuilds its children from the dataset whenever the dataset's
// rev, or the store node its link points at, changes: its first load, a reload, a replacement.
// Until then the node is empty, so a download never holds up a frame.
struct SceneDataBinding {
  cvc::ariadne::SceneNode spec;                     // the node as the document declared it
  std::string path;                                 // the dataset: <prefix>.data.<name>
  std::weak_ptr<GraphicsNode> group;                // the node; its children are the drawing
  std::vector<std::shared_ptr<GraphicsNode>> drawn; // those children
  std::string drawn_target;                         // the store node last drawn
  std::string drawn_rev;                            // and its rev then
  std::string frame_key; // frame: the main view camera's frame request (<prefix>.viewers.main...)
  bool framed = false;   // frame: first — done
};

// A node whose material: { opacity: <path> } (or { bind, default }) follows state. tick_scene
// re-reads the key each frame, without seeding it (a follower, as visible: is), clamps it to 0..1,
// and writes it into each mesh the node draws: its own GeometryNode, its LOD ladder's rungs, or
// what its source: { data } drew (a redraw included). Held weakly, like the tickers.
struct SceneOpacityBinding {
  std::string source;               // the resolved bind path
  double fallback = 1.0;            // the opacity while the key is unset (or not a number)
  std::weak_ptr<GraphicsNode> node; // the node
  int data = -1;                    // a source: { data } node: its entry in data_nodes
};

// The result of realizing a Scene: the top-level node ids created (the scene's nodes, and any a
// custom node type added beside its own), the visibility
// bindings the host must poll each frame (empty when no node uses `visible: <path>`),
// any StageLighting rigs, and the volume renderers that need per-frame ticking. The
// rigs are OWNED here because `~StageLighting` removes the rig's lights from the scene
// — so the RealizedScene must outlive the render loop (as the host already keeps it)
// or the lights vanish. The volren/volslice tickers are held as weak_ptr — the
// SceneGraph is the sole owner, so a torn-down node simply drops out of the tick
// rather than dangling (consistent with the string-only visibility bindings).
struct RealizedScene {
  std::vector<std::string> created;
  std::vector<cvc::ariadne::SceneVisibilityBinding> visibility;
  // §9 clip: nodes whose `clip:` has a bound `offset:` — re-read by tick_scene each frame (state
  // paths only, like `visibility`). Empty when no clip offset is bound.
  std::vector<cvc::ariadne::SceneClipBinding> clip;
  // material: { opacity: <path> }: the nodes whose opacity follows state (sync_scene_opacity).
  std::vector<SceneOpacityBinding> opacity;
  std::vector<std::unique_ptr<StageLighting>> rigs;
  std::vector<std::weak_ptr<VolRenNode>> volren_ticks;
  std::vector<std::weak_ptr<VolSliceNode>> volslice_ticks;
  // The geometry nodes realized as LOD ladders (RealizeOptions::lod), held weakly like the
  // tickers. Empty without the option, so a host runs a SceneGraph::selectLOD pass per frame only
  // when this is not.
  std::vector<std::weak_ptr<LodGraphicsNode>> lod_nodes;
  // Every realized node, at any depth, with its view tags (`tags:`, §9.5.1): what a viewport
  // widget leaves out when its show: / hide: rule out a node's tags, and what its override:s name
  // by id or tag. Held weakly, like the tickers.
  struct TaggedNode {
    std::string id;
    std::vector<std::string> tags;
    std::weak_ptr<GraphicsNode> node;
  };
  std::vector<TaggedNode> nodes;
  // The scene's named cameras (`cameras:`), which viewport widgets bind by id.
  std::vector<cvc::ariadne::SceneCamera> cameras;
  // main_view: { show, hide }: the tags the main view draws (SceneViews applies it).
  bool has_main_view = false;
  std::vector<std::string> main_show, main_hide;
  // Per-frame closures a CUSTOM node type registered (register_scene_node_type) that
  // needs servicing — run by tick_scene after the built-in tickers. A closure should
  // capture a weak_ptr to its node (not the node), matching the volren/volslice
  // discipline, so a torn-down node's tick is a no-op.
  std::vector<std::function<void(vtkRenderer *)>> custom_ticks;

  // §9 time. When the scene declared a `clock:` block, tick_scene drives the app's authoritative
  // cvc::world_clock (app.world_clock()) through a cvc::world_clock_binding each frame: it reads
  // sim.speed / sim.paused / sim.mode / sim.lockstep_steps / sim.step (or whatever keys the clock
  // names), advances by the wall delta, and publishes the world time. So a custom_tick reads ONE
  // coherent app.world_clock().t() for the frame, and the reusable sim_transport widget controls
  // it, lockstep and single-step included, with no host C++. `app` is the owning app (set by
  // realize_scene); the binding is made on the first tick; the timestamp/primed pair measures real
  // elapsed between ticks when the caller does not inject a fixed dt. RealizedScene is MOVED,
  // never copied; these members reset on a scene reload (one dt≈0 frame — app.world_clock() itself
  // persists across reloads).
  cvc::ariadne::SceneClock clock; // resolved clock declaration; clock.present gates the drive
  cvc::app *app = nullptr;        // owning app (world_clock() + state) — set by realize_scene
  std::unique_ptr<cvc::ariadne::SceneClockDriver> clock_driver; // made by the first tick_scene

  // shadows: { focus: camera | { follow: camera, ... } }: tick_scene re-aims the shadow maps at
  // what the renderer's camera looks at, every frame, on `sg` (set by realize_scene; the host keeps
  // the SceneGraph alive as long as this, as it does the app). Unused unless follow_camera.
  cvc::ariadne::SceneShadowFocus shadow_focus;
  SceneGraph *sg = nullptr;

  // The heightfield nodes' own cvc::height_field, by node id: the grid the node draws, the terrain
  // its body collides with, and what a custom node standing on it (forest_trees, wave_sea,
  // cloud_sky: their `ground:`) samples. The node's GeometryNode holds a plain copy of the mesh.
  std::map<std::string, std::shared_ptr<const cvc::height_field>> height_fields;

  // What a custom node's body: collides with, by node id, when the node makes it at run time (a
  // generated_world's terrain and buildings): realize_physics hands it to the world, which swaps
  // the node's bodies whenever the node bumps its revision.
  std::map<std::string, std::shared_ptr<cvc::ariadne::SceneColliders>> colliders;

  // The scene's world (`physics:`, or the nodes' `body:`s), null without one: core libcvc's
  // cvc::ariadne::SceneWorld, the world a headless host runs for the same document. Its bodies are
  // placed at their nodes' world poses (composed from the spec); tick_scene steps it on
  // app.world_clock(), which publishes its telemetry under `physics.stats` (RealizeOptions::
  // physics_key), and draws the moving bodies through `physics_binding` (declared after the world,
  // so it goes first). `physics` is the world's physics::world. `bodies` maps node ids to the
  // bodies the world was built with; a node's colliders and a dataset's come and go at run time,
  // and world->bodies_of(id) has them.
  std::unique_ptr<cvc::ariadne::SceneWorld> world;
  cvc::physics::world *physics = nullptr;
  std::unique_ptr<cvc::gl::physics_scene_binding> physics_binding;
  std::vector<std::pair<std::string, cvc::physics::body_id>> bodies;
  // The nodes the moving bodies draw, by node id: restart_scene_world binds them again.
  std::map<std::string, std::shared_ptr<GraphicsNode>> body_nodes;
  // The source: { data } nodes (SceneDataBinding), rebuilt by tick_scene as their datasets load.
  std::vector<SceneDataBinding> data_nodes;
  // The host's bind prefix the scene was realized under (a custom node's keys resolve with it).
  std::string bind_prefix;
  // The world's vehicles as drawn (vehicle_view.h): each entity's node and the chassis it is
  // bound to, as of the vehicles' revision `vehicles_rev`.
  std::vector<std::pair<std::string, cvc::physics::body_id>> vehicle_markers;
  std::uint64_t vehicles_rev = ~std::uint64_t(0);
  // The vehicle models this scene has read and fitted, read once each (vehicle_view.cpp).
  std::shared_ptr<struct vehicle_model_cache> vehicle_models;
  // The spawn points as drawn (their node ids), as of the spawn points' revision `spawns_rev`.
  std::vector<std::string> spawn_markers;
  std::uint64_t spawns_rev = ~std::uint64_t(0);
  // The body of node `id` now (the first, for a node with colliders), or an invalid id.
  cvc::physics::body_id body(const std::string &id) const {
    return world ? world->body(id) : cvc::physics::body_id{};
  }
};

// The binding keys for a scene clock. The scene names speed, paused, time and tick one by one
// (empty = that lane is off); the binding's other keys (mode, lockstep_steps, step, fixed_dt,
// realized_speed, dropped) sit beside them, under the parent of the first named key that keeps its
// standard leaf: clk.sim.speed gives clk.sim.mode, clk.sim.step, ... With no such key those lanes
// are off too.
cvc::world_clock_keys scene_clock_keys(const cvc::ariadne::SceneClock &clock);

// Per-frame servicing for realized volren/volslice nodes (§9): each such node needs
// a tick() every frame or it renders nothing, and multiple volslice nodes need a
// back-to-front depth sort. Geometry/group/volume/light nodes need NO ticking. Call
// this each frame from the host render loop, BEFORE SceneRenderer::render(), on the
// owner/render thread, passing the scene's vtkRenderer (SceneRenderer::renderer()) —
// the renderer is used only for the multi-slice depth sort. A no-op when there are no
// volume renderers.
//
// §9 time: when the realized scene declared a `clock:` block, tick_scene ALSO drives the app's
// authoritative cvc::world_clock here (before the node tickers), so animations read one coherent
// world time and the sim_transport widget's pause/speed/mode/step take effect. `wall_dt` is the
// wall-clock delta to advance by: pass a fixed value (e.g. 1.0/fps) for a DETERMINISTIC offscreen
// capture; leave it negative (the default) and tick_scene measures real elapsed since the last
// call — which keeps animation at real-time speed even when the browser renders well under the
// frame cap (a fixed per-frame dt would make a slow frame rate crawl, the very bug the wall-clock
// ticks fixed). Returns the quanta this frame ran (steps, first_tick, alpha), so a host can step
// its own followers by them. Scenes without a `clock:` block ignore wall_dt, never touch
// app.world_clock(), and return an empty result.
//
// A physics scene (`physics:`, or any node `body:`) drives the clock as if it declared `clock: {}`,
// then steps its world by the quanta due and poses the moving bodies' nodes between the last two
// states (physics_scene_binding::follow + present).
cvc::world_clock::step_result tick_scene(RealizedScene &realized, vtkRenderer *renderer,
                                         double wall_dt = -1.0);

// sim.restart for a realized scene: its world rebuilt as realize_scene made it
// (SceneWorld::restart: the same physics::world, its bodies made again in order), and its moving
// bodies' nodes bound again and drawn where the bodies start. The caller resets the clock (the
// next tick_scene anchors on it). False when the scene has no world. Problems warn.
bool restart_scene_world(RealizedScene &realized, std::vector<std::string> *warnings = nullptr);

// Rebuild each source: { data } node whose dataset is ready and has changed since it last drew
// (SceneDataBinding): tick_scene calls it first every frame; a host that does not tick may call
// it itself. Call it on the scene's owner thread. Returns how many nodes it rebuilt.
int sync_scene_data(RealizedScene &realized, std::vector<std::string> *warnings = nullptr);

// Write each bound material: { opacity: <path> } into the meshes its node draws
// (SceneOpacityBinding): tick_scene calls it every frame, after sync_scene_data; a host that does
// not tick may call it itself. An unchanged value writes nothing. Call it on the owner thread.
void sync_scene_opacity(RealizedScene &realized);

// Create/configure SceneGraph nodes from `scene`. `bind_prefix` is the SAME
// cvc::state prefix the widget Runtime was constructed with (typically
// sg.getStatePrefix()), so a scene `visible:` bind and a widget `bind:` on the same
// relative path resolve to one key. When `warnings` is non-null, non-fatal issues (a
// missing source, an unreadable file, a not-yet-supported node type) are appended
// rather than thrown.
RealizedScene realize_scene(SceneGraph &sg, const cvc::ariadne::Scene &scene,
                            const std::string &bind_prefix,
                            std::vector<std::string> *warnings = nullptr);

// Opt-in realize behaviour. The defaults are the four-argument realize_scene, exactly.
// The geometry of a document's assets (`assets:`), each read and fitted once: what every node whose
// source is { asset: <id> } draws, in every scene that shares the cache. cvc::geometry is
// copy-on-write, so those nodes share its buffers, and they share the VTK arrays built from them
// too: the first node on an asset builds them, the others take them (GeometryNode::shareArraysFrom,
// copy-on-write). Keep one per document, across its scenes (AriRuntime's scene adapter does).
struct SceneAssetCache {
  std::map<std::string, cvc::geometry> geometry;
  std::set<std::string> failed; // read once and failed: not read again, nor warned again
  std::size_t reads = 0;        // how many assets have been read (for tests)
  // Each asset's first node: the one whose VTK arrays the asset's other nodes draw from.
  std::map<std::string, std::weak_ptr<GeometryNode>> donors;
};

struct RealizeOptions {
  // Realize a mesh-file geometry node as a LodGraphicsNode whose ladder comes from this cache
  // (LodPyramidCache::attach: full detail at once, the coarser rungs once their build lands on a
  // later poll()). Null = never. A node qualifies only when its source is a file or URI (not a
  // primitive or a heightfield), it has no material texture, shader, clip, body or children, and
  // the mesh it reads has at least `lod_min_tris` triangles; everything else realizes exactly as
  // without the option. A qualifying node keeps its id, registration (top level) or nesting, its
  // transform and its `visible:`; every rung gets the node's material (apply_node_material). The
  // cache must outlive the realized nodes' builds -- keep it beside the scene, across re-realizes,
  // so a realize of the same meshes again needs no build. LOD is a render proxy only: such a node
  // has no body, and nothing that reads geometry for physics or navigation sees a rung.
  LodPyramidCache *lod = nullptr;
  // Below this a ladder saves little and costs about 1.5x the resident buffers.
  std::uint64_t lod_min_tris = 4096;
  // Where `source: { asset: <id> }` nodes get their geometry. Null: realize_scene keeps a cache of
  // its own for that one scene (each asset still read once there).
  SceneAssetCache *assets = nullptr;
  // Where the scene's physics world publishes its telemetry (`<key>.stats.*`, resolved against the
  // bind prefix like a widget bind). Empty: it publishes none (a document's other scenes).
  std::string physics_key = "physics";
};

// realize_scene with options; the LOD nodes it creates are listed in RealizedScene::lod_nodes.
RealizedScene realize_scene(SceneGraph &sg, const cvc::ariadne::Scene &scene,
                            const std::string &bind_prefix, std::vector<std::string> *warnings,
                            const RealizeOptions &options);

// --- extensibility: custom scene node types ----------------------------------
//
// A realizer for a CUSTOM scene node `type` (a type: the built-ins don't handle).
// It builds and returns a live GraphicsNode from the parsed SceneNode — whose custom
// config is in `node.props` (a neutral cvc::ariadne::Value map). Create the node under
// `parent` when non-null (nested — its transform is then LOCAL to the parent) else
// under sg.getGraphicsRoot() (and call sg.registerGraphics(id, node) for a top-level
// node, matching the built-ins). If it needs per-frame service, push a closure that
// captures a weak_ptr to the node into `out.custom_ticks`. Return the node; the shared
// transform + `visible:` + children handling then runs on it (so a realizer need not
// repeat it). Return nullptr to decline. Runs on the render/owner thread. This is the
// cvcGL side of extensibility — the core never learns the custom type; it only carries
// its `props` as data.
using NodeRealizer = std::function<std::shared_ptr<GraphicsNode>(
    SceneGraph &sg, const cvc::ariadne::SceneNode &node, GraphicsNode *parent, RealizedScene &out,
    std::vector<std::string> *warnings)>;

// Register (or replace) a realizer for a custom scene node `type`. Registering a
// built-in type (geometry/volume/volren/volslice/group) is a no-op — the built-ins own
// their dispatch. The type is `app`'s alone: a scene on another app does not realize it.
// Thread-safe; register before realize_scene.
void register_scene_node_type(cvc::app &app, const std::string &type, NodeRealizer realizer);

// Whether `app` has a realizer for a custom scene node `type` (test/introspection).
bool has_scene_node_type(cvc::app &app, const std::string &type);

// --- the built-in source readers, for custom realizers -----------------------
//
// A custom node type that shows a mesh or a volume reads its `source:` exactly like the built-in
// geometry / volren nodes do, so a document can swap `type: geometry` for a custom type without
// changing anything else. Render/owner thread (they may read files and compute an SDF).

// `fit:` — stand a mesh on the ground: optionally rotate a Y-up mesh to Z-up, centre it in XY, sit
// its base on z = 0 and scale its tallest extent to `height`. Returns a fresh geometry (points,
// normals, colours, tris).
cvc::geometry fit_to_ground(const cvc::geometry &raw, bool up_y, float height);
// The same with the dataset's up axis (`x`, `y` or `z`, which becomes +z) and `flip` (then a half
// turn about x: a scan stored upside down), as `fit: { up, flip }` declares them.
cvc::geometry fit_to_ground(const cvc::geometry &raw, char up, bool flip, float height);
// That rotation alone: points and normals turned, everything else copied.
cvc::geometry turn_up(const cvc::geometry &raw, char up, bool flip);
void turn_up_point(char up, bool flip, const double in[3], double out[3]);
// A node's `fit: { up }` as one of 'x', 'y', 'z' (SceneNode::fit_up_y wins: it is the `y` case).
char fit_up_axis(const cvc::ariadne::SceneNode &node);

// The mesh a node's `source:` declares — a heightfield (its cvc::height_field's mesh), a plane, box
// or sphere primitive, or a file / URI — with its `fit:` applied. Warns (naming the node) and
// returns false when there is no usable source.
bool read_node_geometry(SceneGraph &sg, const cvc::ariadne::SceneNode &node, cvc::geometry &geom,
                        std::vector<std::string> *warnings);

// A geometry node's `material:` block -- single colour, colour, ambient, diffuse, and specular when
// stated -- as the built-in geometry node applies it (and every rung of a LOD node). A node with no
// material block is left as it is.
void apply_node_material(GeometryNode &g, const cvc::ariadne::SceneNode &node);

// The UVs of the mesh `g` draws (UV (0, 0) the image's bottom-left) turned by the node's
// `material: { uv: ... }` block: the flips, then the quarter turns, then scale and offset
// (SceneNode::uv_*), in place. A node with no uv block, or a mesh with no UVs, is left as it is.
void apply_uv_transform(GeometryNode &g, const cvc::ariadne::SceneNode &node);

// A dim³ signed distance field of the mesh at `mesh_uri` (fit applied first when `apply_fit`),
// framed to the mesh extents padded 10% and cubic. Negative inside. Throws on a build without
// CVC_ENABLE_SDF or on an unreadable mesh.
cvc::volume sdf_volume_from_mesh(cvc::app &app, const std::string &mesh_uri, int dim,
                                 bool apply_fit, bool fit_up_y, float fit_height,
                                 cvc::sdf_algorithm algorithm = cvc::SDF_V2);
cvc::volume sdf_volume_from_mesh(cvc::app &app, const std::string &mesh_uri, int dim,
                                 bool apply_fit, char fit_up, bool fit_flip, float fit_height,
                                 cvc::sdf_algorithm algorithm = cvc::SDF_V2);
// The same for a mesh already in hand (any fit applied by the caller).
cvc::volume sdf_volume_of_mesh(cvc::app &app, const cvc::geometry &mesh, int dim,
                               cvc::sdf_algorithm algorithm = cvc::SDF_V2);

// The volume a node's `source:` declares: `{ file }`, or `{ sdf: { mesh, dim, algorithm } }`
// (algorithm v1 | v2 | igl; igl falls back to v2 with a warning when cvc::mesh_ops_available() is
// false). Throws on failure.
cvc::volume load_node_volume(SceneGraph &sg, const cvc::ariadne::SceneNode &node,
                             std::vector<std::string> *warnings);

// --- extensibility: named GLSL shader presets (the DSL `shader: { preset: <name> }`) ---------
//
// A shader preset is a named effect that configures a GeometryNode's shader (the bump/bark/etc.
// GLSL + any platform gate living in C++). realize_scene applies the preset named by a node's
// `shader:` block, then its inline vertex/fragment splices. Registering presets is how cvcGL (and
// a host) contribute GL effects the pure-data DSL can only name. A preset is its app's: a node
// realized on another app does not find it. Thread-safe; register before realize_scene.
// register_default_shader_presets() installs the built-ins ("terrain_bump", "bark"); it is
// idempotent and is what the cvcGL extension bundle calls.
using ShaderPreset = std::function<void(GeometryNode &)>;
void register_shader_preset(cvc::app &app, const std::string &name, ShaderPreset preset);
bool has_shader_preset(cvc::app &app, const std::string &name);
void register_default_shader_presets(cvc::app &app);

// The keys realize_scene applies to a node's mesh once its realizer has returned it: casts_shadow,
// the `shader:` block and material.opaque. A realizer that draws its node's surface with meshes of
// its own as well (a ground cut into LOD chunks) applies them to each of those.
void apply_mesh_keys(GeometryNode &g, const cvc::ariadne::SceneNode &n,
                     std::vector<std::string> *warnings = nullptr);

// Verify the NODE customs a document declared (`loaded.customs`, from the `customs:`
// block) against `app`'s registered scene node types. The loader already fail-fast-checked
// widget/block customs at LOAD; node types register on THIS (cvcGL) side, so the host
// calls this after load and BEFORE realize_scene, to fail fast on a required node
// custom this build lacks. Returns false if any REQUIRED node custom is unregistered
// (appending a message to `errors` when non-null); an OPTIONAL missing one appends a
// note but does not fail (realize_scene warns + skips it). Returns true when every
// required node custom is registered (and when the document declared none).
bool verify_scene_customs(cvc::app &app, const cvc::ariadne::LoadResult &loaded,
                          std::vector<std::string> *errors = nullptr);

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_SCENE_REALIZE_H
