/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// world_nodes.cpp — the generated_world scene node type (see world_nodes.h). A thin cvcGL wrapper:
// cvc::world::generate_world builds the world; this puts its merged meshes, surface texture and
// water into GraphicsNodes, and regenerates them when the world.* keys change. A regeneration or a
// repaint is a sliced task on the app's scheduler (cvc/core/sliced_task.h: world_builder, then
// surface_painter, a few milliseconds a frame), so the page keeps drawing the old world while the
// new one is made; world.progress / world.phase show how far it is, and a request "stop" ends it.
//
// A `source: { data }` node whose dataset is a world (a city bundle's, a saved world's) draws with
// the same renderer (DataWorldView, ARIADNE_WORLDS section 18.8 "one renderer"): a sliced job makes
// the drawn world from the dataset (world::world_from_bundle), and it goes into the scene as a
// generated world does, under the data node's group.

#include <algorithm>
#include <atomic>
#include <boost/any.hpp>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/bind.h> // live node params: read_or_seed / read_or / write / resolve_bind
#include <cvc/ariadne/data.h> // a data node's dataset: its files, data_file
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_generated_world.h> // the node in core: params, keys, stats, dataset
#include <cvc/ariadne/scene_wind.h>            // the scene's one wind, which the trees lean in
#include <cvc/ariadne/shown_clock.h>           // the time the scene is drawn at (OW5q)
#include <cvc/ariadne/uri.h>
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h>
#include <cvc/core/sliced_task.h>
#include <cvc/core/types.h> // CVC_VERSION_STRING
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/height_field.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/LodGraphicsNode.h> // the props' and the ground's ladders, make_view_params
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/WindGLSL.h> // the gusts the trees sway in, the water's and the physics'
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/world_nodes.h>
#include <cvc/gl/world_geometry_nodes.h> // surface_colors: a mesh without colours of its own
#include <cvc/image/image.h>
#include <cvc/model/model_file_io.h> // can_write_model_to_memory
#include <cvc/state/state.h>
#include <cvc/volume/bounding_box.h>
#include <cvc/world/building_archetypes.h>
#include <cvc/world/city_bundle.h>
#include <cvc/world/generated_world.h>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>
#include <vtkCamera.h>
#include <vtkRenderer.h>

namespace cvc {
namespace gl {
namespace ariadne {
namespace {

namespace ari = cvc::ariadne;
namespace world = cvc::world;

// The props' and the ground's LOD budget, as the room view's: the wasm profile coarsens sooner.
#ifdef __EMSCRIPTEN__
constexpr cvc::lod::quality_preset kLodPreset = cvc::lod::quality_preset::aggressive;
#else
constexpr cvc::lod::quality_preset kLodPreset = cvc::lod::quality_preset::balanced;
#endif
const char *const kColorNames[] = {"material", "risk", "occupancy", "height", "water"};

int color_from(const std::string &s, int def) {
  for (int i = 0; i < 5; ++i)
    if (s == kColorNames[i])
      return i;
  return def;
}

// A regeneration (a new world, then its texture) or a repaint (the texture of the shown world).
// Its task writes only this; the tick takes it in once `done` is set.
struct WorldJob {
  bool rebuild = true;
  int color = 0;
  std::atomic<bool> done{false};
  std::atomic<int> progress{0};                  // percent
  std::atomic<const char *> phase{nullptr};      // a static string, or null
  std::shared_ptr<world::generated_world> world; // rebuild: made here; repaint: the shown one
  std::vector<unsigned char> texture;            // the ground's RGB texels, rows south first,
  int texture_w = 0, texture_h = 0;              // this wide and high (a city's may not be square)
  std::vector<unsigned char> water;              // world::paint_water's RGBA texels
  // rebuild: the world as its bundle's files, written here, off the frame thread (empty when no
  // glb can be written, or it failed: bundle_error), for its dataset (publish_generated_world).
  world::city_bundle_files bundle;
  std::string bundle_error;
  std::string error; // why it failed; empty when it did not
};

struct WorldInstall;

// What is leaving the scene: nodes hidden at once (a world swapped out, an install stopped), then
// removed a few a frame, and the worlds they drew, freed a piece at a time after them. (Removed in
// one frame, a 10 km world's nodes took seconds: each removal frees a mesh's VTK objects and its
// state; and freeing the world itself, its meshes and raster, two more.)
struct WorldRetired {
  std::vector<std::pair<std::weak_ptr<GraphicsNode>, std::weak_ptr<GraphicsNode>>>
      nodes; // (parent, node)
  std::vector<std::shared_ptr<world::generated_world>> worlds;
  bool empty() const { return nodes.empty() && worlds.empty(); }
};

// Everything the tick needs, in one shared handle.
struct WorldRT {
  cvc::app *app = nullptr;
  SceneGraph *sg = nullptr; // the scene the world's nodes are in
  std::string id;           // the node's id: the parts beside it are named for it
  // A data node's world (DataWorldView): its terrain node and the parts beside it are children of
  // the data node's group (`root`), not top-level nodes; it publishes no dataset, stats or ground
  // of its own (the data node's dataset is the world).
  bool rooted = false;
  std::weak_ptr<GraphicsNode> root;
  bool publishes = true;
  bool show_underside = true; // the rock under a floating world (put_underside)
  ari::GeneratedWorldKeys k;
  world::generated_world_params base; // the authored props
  std::string base_dir;               // the document's, for a relative terrain_file
  const world::building_library *library = nullptr;
  std::shared_ptr<world::generated_world> w; // the world shown
  // The world shown as the node's dataset (publish_generated_world): a body: on the node is its
  // world, as a data node's is, and the scene's world takes each new one at its next sync.
  std::string data_path;
  std::string sig; // the params the current world (or the one being made) is from
  int preset = 0;  // the preset the counts last came from
  int color = 0;   // the colour the shown texture was painted in
  int texture_size = 2048;
  bool soft = true; // look: soft (demo3's) or classic (terrain_lab's)
  long frame = 0;
  bool shown = true; // the node's visibility, which the parts beside it last followed
  std::shared_ptr<cvc::height_field> field; // published in RealizedScene::height_fields
  std::weak_ptr<GeometryNode> terrain;
  // The buildings and the roads: a top-level node each (<id>_buildings, <id>_roads) that draws
  // nothing itself and holds the shown world's merged mesh as its child, so a new world's mesh is
  // made beside the old one and swapped in.
  std::weak_ptr<GeometryNode> buildings, roads;
  std::weak_ptr<GeometryNode> building_mesh, road_mesh;
  long generation = 0; // the worlds put in so far: the next one's nodes are named for it
  // The props, a LOD ladder per ground cell (world::generated_world::prop_cells) under the groups
  // <id>_props, <id>_foliage and <id>_needles. The wood ladder chooses; the cell's leaves and
  // needles follow it.
  struct PropCell {
    std::weak_ptr<LodGraphicsNode> wood, leaves, needles;
  };
  std::vector<PropCell> cells;
  // The ground, a LOD ladder per chunk (world::generated_world::terrain_chunks) under the terrain
  // node, which holds the texture they all share and draws nothing itself. Each frame every chunk
  // chooses its rung, then is held within one of its neighbours' (what the skirts cover).
  std::vector<std::weak_ptr<LodGraphicsNode>> chunks;
  std::vector<int> chunk_want; // each chunk's own choice last frame: its hysteresis history
  std::shared_ptr<const ari::SceneNode> keys; // the authored node, for its mesh keys
  // The water: a surface per chunk that has any, under the group <id>_water (made the first time),
  // each a ladder on its chunk's rungs that switches with it. Null where a chunk has no water.
  // Their texture (world::paint_water: the depth's colour, transparent over dry ground) is held,
  // once, by an empty mesh under the group that draws nothing, as the terrain node holds the
  // ground's.
  bool show_water = true;
  std::vector<std::weak_ptr<LodGraphicsNode>> water;
  std::weak_ptr<GeometryNode> water_texture;
  // The sea past a shore-edged world (world::outer_sea), under <id>_water too: made the first time
  // a world has one, emptied for a world that has none.
  std::weak_ptr<GeometryNode> outer_sea;
  // The rock under a floating world (world::world_underside: the wall and none edges), under
  // <id>_underside: made the first time a world has one, emptied for a shore-edged world.
  std::weak_ptr<GeometryNode> underside;
  std::shared_ptr<WorldJob> job; // the running job, or null
  int task = -1;                 // its sliced task's pid
  // A made world going into the scene a piece at a time (put_world_step), or null. The job stays
  // the running one until it is in.
  std::unique_ptr<WorldInstall> install;
  WorldRetired retired; // what is on its way out (retire_world_step)
  // The trees in the scene's wind (wire_foliage_wind): the uniforms every prop mesh's sway reads,
  // set each frame, and left out of the shadows' bake, so the trees' shadows stay at rest. The
  // document's state root (the scene wind's keys), world.sway (the sway's strength, 0 holds the
  // trees) and world.lean_dir (the way they lean, published; empty without a wind).
  std::shared_ptr<GeometryNode::ShaderInputs> wind = [] {
    auto in = std::make_shared<GeometryNode::ShaderInputs>();
    in->skip_shadow_bake = true;
    in->f["uFwOn"] = 0.0f;
    return in;
  }();
  std::string prefix, sway_key, lean_key, lean_now;

  ~WorldRT();
};

// A made world going into the scene a piece at a time, a few milliseconds a frame: first its two
// textures, then a ground chunk (and its water) at a time, a prop cell at a time, the buildings and
// the roads, each new node hidden beside the world shown; then, in one frame, the old world's nodes
// go and the new ones show. So the page keeps drawing, and the old world stays whole, until the new
// one is all there. (Made in one frame, a 10 km world's ~250 chunk ladders and ~140 prop cells held
// the page for seconds: each rung is a mesh converted for VTK.)
struct WorldInstall {
  std::shared_ptr<WorldJob> job; // its world and textures
  std::string suffix;            // "_g<generation>": the new nodes' names, apart from the old's
  enum class Stage { textures, chunks, cells, buildings, roads, swap, done };
  Stage stage = Stage::textures;
  std::size_t next = 0;                                 // the next chunk or cell
  std::vector<std::weak_ptr<LodGraphicsNode>> chunks;   // the new ground, hidden
  std::vector<std::weak_ptr<LodGraphicsNode>> water;    // its water, hidden (null where dry)
  std::vector<WorldRT::PropCell> cells;                 // the new props, hidden
  std::weak_ptr<GeometryNode> building_mesh, road_mesh; // the new meshes, hidden
  std::size_t done = 0, total = 1;                      // pieces put in, of
};

WorldRT::~WorldRT() {
  if (task >= 0 && app)
    cvc::cancel_sliced_job(*app, task);
}

// world::paint_surface's rows run south to north (row 0 = min y, the terrain's v = 0); a
// cvc::image's run top down, so its rows are reversed here: setTexture turns them back, and the
// south row lands at v = 0. (Taken as they were, the texture lay north-south mirrored on the
// ground: water painted under dry land and buildings, ground painted on the lakes.)
cvc::image surface_image(int width, int height, const std::vector<unsigned char> &rgb) {
  return cvc::image(width, height, cvc::image::pixel_format::RGB, cvc::image::data_type::u8,
                    rgb.data())
      .flipped_vertical();
}
// world::paint_water's RGBA texels as an image, the same way up.
cvc::image water_image(int size, const std::vector<unsigned char> &rgba) {
  return cvc::image(size, size, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8,
                    rgba.data())
      .flipped_vertical();
}

void publish_stats(WorldRT &rt) {
  if (rt.publishes && rt.w)
    ari::publish_generated_world_stats(*rt.app, rt.k, *rt.w);
}

// A part of the world beside the terrain node, <id><suffix> (the groups and holders of
// kWorldParts): a child of the data node's group for a data node's world, else a top-level node.
// Null when it is not there.
std::shared_ptr<GraphicsNode> find_part(WorldRT &rt, const std::string &suffix) {
  if (!rt.rooted)
    return rt.sg->getGraphics(rt.id + suffix);
  auto r = rt.root.lock();
  return r ? r->findChildByName(rt.id + suffix) : nullptr;
}
// The same, made (an empty group) the first time.
std::shared_ptr<GraphicsNode> make_part(WorldRT &rt, const std::string &suffix) {
  if (std::shared_ptr<GraphicsNode> g = find_part(rt, suffix))
    return g;
  if (!rt.rooted)
    return rt.sg->addGraphics(rt.id + suffix);
  auto r = rt.root.lock();
  return r ? r->createChild(rt.id + suffix) : nullptr;
}

// Style each mesh node the first time it exists. soft = demo3's daylight look (matte ground and
// concrete, a little specular on the buildings); classic = terrain_lab's.
void style_terrain(GeometryNode &t, bool soft) {
  t.setUseSingleColor(true); // the colour is the baked texture
  t.setColor(1.0, 1.0, 1.0);
  t.setAmbient(soft ? 0.4 : 0.5);
  t.setDiffuse(soft ? 0.85 : 0.95);
  t.setSpecular(0.0);
}
void style_solid(GeometryNode &g, bool soft) {
  g.setUseSingleColor(false); // per-vertex material colour
  g.setAmbient(soft ? 0.36 : 0.45);
  g.setDiffuse(soft ? 0.7 : 0.85);
}
void style_buildings(GeometryNode &g, bool soft, bool archetypes) {
  if (soft && archetypes) { // demo3's matte city
    g.setUseSingleColor(true);
    g.setColor(0.74, 0.72, 0.68);
    g.setAmbient(0.3);
    g.setDiffuse(0.7);
    g.setSpecular(0.04);
    return;
  }
  style_solid(g, soft);
}
void style_roads(GeometryNode &g, bool soft) {
  g.setUseSingleColor(false);
  g.setAmbient(soft ? 0.36 : 0.5);
  g.setDiffuse(0.7);
}
void style_foliage(GeometryNode &g, bool soft) {
  g.setUseSingleColor(false);
  g.setAmbient(soft ? 0.45 : 0.6);
  g.setDiffuse(0.7);
}
// The water surface: its colour is the depth's, from world::paint_water's texture (shared from
// `tex`), with a glint of the sun; it casts no shadow onto its own bed, and it is opaque (the depth
// colour stands in for what shows through). The texture is transparent wherever the full terrain
// mesh is dry and VTK discards fully transparent fragments, so a coarse rung's cells end at the
// mesh's own shoreline. Without the texture, the per-vertex depth colours show.
void style_water(GeometryNode &g, const std::weak_ptr<GeometryNode> &tex) {
  if (auto from = tex.lock(); from && from->hasTexture()) {
    g.shareTexture(from);
    g.setUseSingleColor(true); // the colour is the texture's
    g.setColor(1.0, 1.0, 1.0);
  } else {
    g.setUseSingleColor(false);
  }
  g.setAmbient(0.45);
  g.setDiffuse(0.55);
  g.setSpecular(0.6);
  g.setSpecularPower(80.0);
  g.setCastsShadow(false);
  g.setForceOpaque(true);
}
void style_needles(GeometryNode &g) {
  g.setRenderMode(GeometryRenderMode::LINES);
  g.setUseSingleColor(true);
  g.setColor(0.12, 0.34, 0.14);
  g.setAmbient(0.55);
  g.setDiffuse(0.75);
}

// ── The trees in the scene's wind (OCEAN-AND-WEATHER-ROADMAP.md E5) ──────────────────────────
// Every prop mesh (the wood, the leaves and the needles, on every rung) leans downwind and sways
// in the vertex shader, from uniforms alone: no texture, no extra vertex array (the low-memory
// mapper keeps each array in a texture unit, and WebGL2 has 16). A vertex's height above the
// ground under it rides in its normal's length, 1 + h / kLiftPerNormal (with_lift_normals): the
// lighting normalises the normal, so it shades as before, and the shader reads h back. The sway
// grows as h squared, so a trunk's foot stays put, its crown moves most, and a rock (a metre or
// two high) barely stirs. Its phase is the vertex's world position, so neighbouring trees sway
// apart, and the scene's gust field (cvc_wind_gust, the water's and the physics') scales it, so a
// gust is seen crossing the woods. The shadows' bake leaves the uniforms out (uFwOn reads 0
// there): the trees' shadows stay at rest rather than freeze at whatever moment a bake caught.
// And a swaying mesh looks its shadow up where it stands at rest (cvcFwRestVC), so its crown and
// trunk are shaded as the at-rest map says: a point moved behind its own rest-pose surface would
// otherwise read as shadowed (acne along the trunk and the limbs, from about 9 m/s close up).
constexpr const char *kFoliageWind = "cvc.foliage_wind"; // the shared shader inputs' key
constexpr double kLiftPerNormal = 64.0; // metres of height per unit of normal length past 1
constexpr double kLiftMax = 60.0;       // the height encoded at most

// The vertex shader's declarations: the uniforms, the gusts, and the sway of a model-space point
// p with normal n. uFwDir is the way the wind blows (x, y); uFwMotion holds the lean (radians, at
// 10 m up), the sway's amplitude (radians) and the clock (s, reduced to the sway's period);
// uFwEye is the camera, and the sway fades out between 300 and 450 m from it (where it is about a
// pixel at terrain_lab's view), so a far tree skips the gusts' noise.
std::string foliage_wind_dec() {
  return std::string("//VTK::PositionVC::Dec\n"
                     "uniform highp float uFwOn; uniform highp vec3 uFwDir; "
                     "uniform highp vec3 uFwMotion; uniform highp vec3 uFwEye;\n"
                     "out vec4 cvcFwRestVC;\n") +
         cvc::gl::WindGLSL::glsl() +
         "vec4 cvc_foliage_sway(vec4 p, vec3 n){\n"
         "  float h = max(length(n) - 1.0, 0.0) * 64.0;\n" // kLiftPerNormal
         "  float lift = h - 0.3;\n"                       // the foot of a trunk stays put
         "  if (uFwOn <= 0.0 || lift <= 0.0) return p;\n"
         "  float near = 1.0 - smoothstep(300.0, 450.0, distance(p.xyz, uFwEye));\n"
         "  if (near <= 0.0) return p;\n"
         "  float g = cvc_wind_gust(p.xy, 0.0);\n"
         "  float phase = dot(p.xy, vec2(0.071, 0.053));\n"
         "  float a = near * g * (uFwMotion.x + uFwMotion.y * sin(1.3 * uFwMotion.z + phase));\n"
         "  float s = a * lift * lift * 0.1;\n" // the bend: angle a at 10 m up
         "  p.xy += uFwDir.xy * s;\n"
         "  p.z -= 0.5 * s * s / max(h, 1.0);\n" // a leaning crown comes down a little
         "  return p;\n"
         "}\n";
}
// At //VTK::Clip::Impl, the earliest point after the low-memory mapper declares vertexMC and
// normalMC: the rest pose's view-space position goes to the fragments for the shadow lookup, then
// the #define re-points VTK's position code (and the shadow baker's) at the swayed point, in
// either mapper (as DrapedLinkNode drapes its link).
const char *const kFoliageWindClip = "cvcFwRestVC = MCVCMatrix * vertexMC;\n"
                                     "vec4 cvcFoliageMC = cvc_foliage_sway(vertexMC, normalMC);\n"
                                     "#define vertexMC cvcFoliageMC\n"
                                     "  //VTK::Clip::Impl\n";
// The fragments' side: the rest position, and the (first) shadow lookup at it (cvcGL's
// FilteredShadowMapPass writes calcShadow(vertexVC, ...) into the lighting).
const char *const kFoliageWindFsDec = "//VTK::Normal::Dec\nin vec4 cvcFwRestVC;\n";
const char *const kShadowAtFragment = "calcShadow(vertexVC,";
const char *const kShadowAtRest = "calcShadow(cvcFwRestVC,";

// Give a prop mesh the sway, once: its shader code, world-space vertexMC, and the world's inputs.
void wire_foliage_wind(GeometryNode &g,
                       const std::shared_ptr<const GeometryNode::ShaderInputs> &in) {
  if (!in || g.hasSharedShaderInputs(kFoliageWind))
    return;
  g.disableCoordinateShiftScale(); // the gusts and the phase are read at world xy
  g.addVertexShaderReplacement("//VTK::PositionVC::Dec", foliage_wind_dec());
  g.addVertexShaderReplacement("//VTK::Clip::Impl", kFoliageWindClip);
  g.addFragmentShaderReplacement("//VTK::Normal::Dec", kFoliageWindFsDec);
  g.addFragmentShaderPostReplacement(kShadowAtFragment, kShadowAtRest);
  g.setSharedShaderInputs(kFoliageWind, in);
}

// `g` with normals, each lengthened by its vertex's height above `ground` (the sway's weight, read
// back in the shader): a mesh's own smooth normals, or +z for lines.
cvc::geometry with_lift_normals(cvc::geometry g, const cvc::height_field &ground) {
  const std::size_t n = g.num_points();
  if (n == 0)
    return g;
  if (!g.const_tris().empty())
    g.compute_normals();
  else
    g.normals().assign(n, cvc::geometry::vector_t{{0.0, 0.0, 1.0}});
  const auto &P = g.const_points();
  auto &N = g.normals();
  for (std::size_t i = 0; i < n; ++i) {
    cvc::geometry::vector_t &v = N[i];
    const double len = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (!(len > 0.5))
      v = {{0.0, 0.0, 1.0}}; // a degenerate vertex: any direction shades it
    const double h = std::clamp(P[i][2] - ground.height(P[i][0], P[i][1]), 0.0, kLiftMax);
    const double s = (1.0 + h / kLiftPerNormal) / (len > 0.5 ? len : 1.0);
    for (int k = 0; k < 3; ++k)
      v[k] *= s;
  }
  return g;
}

// This frame's wind on the trees: the scene's (none without one, or with world.sway at 0, and the
// trees stand still). The lean grows as the wind's square, the sway as the wind, both to 9 m/s's
// 0.03 and 0.02 rad at 10 m up times world.sway; world.lean_dir publishes the way they lean. The
// camera (`ren`'s) is where the sway fades out from. A storm's gust front (F4, scene_wind_now)
// bends them harder and turns them; the gusts' pattern drifts with the wind as authored.
void tick_foliage_wind(WorldRT &rt, vtkRenderer *ren) {
  cvc::app &app = *rt.app;
  const std::optional<ari::SceneWind> sw = ari::scene_wind_now(app, rt.prefix);
  const double strength = ari::read_or<double>(app, rt.sway_key, 1.0);
  const bool on = sw && sw->mean_speed() > 0.0 && strength > 0.0;
  GeometryNode::ShaderInputs &in = *rt.wind;
  in.f["uFwOn"] = on ? 1.0f : 0.0f;
  if (on) {
    constexpr double kPi = 3.14159265358979323846, kSwayPeriod = 2.0 * kPi / 1.3;
    const double rad = sw->mean_dir() * kPi / 180.0, u = sw->mean_speed() / 9.0,
                 t = ari::shown_time(app, rt.prefix, app.world_clock().t()); // shown_clock.h
    const double lean = std::min(0.03 * u * u * strength, 0.3);
    const double sway = std::min(0.02 * u * strength, 0.2);
    in.f3["uFwDir"] = {float(std::cos(rad)), float(std::sin(rad)), 0.0f};
    in.f3["uFwMotion"] = {float(lean), float(sway), float(std::fmod(t, kSwayPeriod))};
    const cvc::gl::WindGLSL::frame f = cvc::gl::WindGLSL::at(sw->state(), t);
    in.f3["uWindGust"] = f.gust;
    in.f3["uWindDriftL"] = f.driftL;
    in.f3["uWindDriftF"] = f.driftF;
    double eye[3] = {0.0, 0.0, 1.0e6}; // no camera yet: everything is far, and still
    if (ren && ren->IsActiveCameraCreated())
      ren->GetActiveCamera()->GetPosition(eye);
    in.f3["uFwEye"] = {float(eye[0]), float(eye[1]), float(eye[2])};
  }
  if (std::string lean = on ? std::to_string(sw->mean_dir()) : std::string(); lean != rt.lean_now) {
    rt.lean_now = lean;
    ari::write<std::string>(app, rt.lean_key, lean);
  }
}

// The part <id><suffix> that holds a merged mesh as its child (WorldRT::buildings, roads): made
// the first time, drawing nothing itself.
std::weak_ptr<GeometryNode> mesh_holder(WorldRT &rt, const std::string &suffix,
                                        std::weak_ptr<GeometryNode> have) {
  if (auto n = have.lock())
    return n;
  const std::string name = rt.id + suffix;
  if (!rt.rooted)
    return std::dynamic_pointer_cast<GeometryNode>(rt.sg->addGraphics(name, cvc::geometry()));
  auto r = rt.root.lock();
  return r ? r->createChild<GeometryNode>(name, cvc::geometry()) : nullptr;
}

// The props' ladder: rung 0 the full trees, rung 1 from world::prop_rung_radius_m[0] and rung 2
// from [1] -- radii authored against the reference view, so the preset scales them.
cvc::lod::mesh_pyramid prop_ladder(std::vector<cvc::geometry> rungs) {
  const cvc::lod::view_params ref = cvc::lod::preset_view(cvc::lod::quality_preset::balanced);
  cvc::lod::mesh_pyramid pyr;
  pyr.rungs = std::move(rungs);
  pyr.world_error_m = {0.0,
                       cvc::lod::world_error_for_switch_radius(world::prop_rung_radius_m[0], ref),
                       cvc::lod::world_error_for_switch_radius(world::prop_rung_radius_m[1], ref)};
  return pyr;
}

// A LOD ladder under `parent`, hidden until the world it belongs to is swapped in: its style set
// before its rungs exist, so none draws unstyled.
std::shared_ptr<LodGraphicsNode> hidden_ladder(GraphicsNode &parent, const std::string &name,
                                               cvc::lod::mesh_pyramid pyr,
                                               LodGraphicsNode::RungStyle style) {
  auto n = parent.addGraphicsChild<LodGraphicsNode>(name);
  n->setVisible(false);
  n->setRungStyle(std::move(style));
  n->setPyramid(pyr);
  return n;
}

// Prop cell i of w: a ladder per layer under the groups <id>_props, <id>_foliage and
// <id>_needles, hidden. Every rung sways in the scene's wind (`wind`, wire_foliage_wind), its
// normals carrying each vertex's height above w's ground.
WorldRT::PropCell make_prop_cell(WorldRT &rt, const world::generated_world &w, std::size_t i,
                                 const std::string &suffix) {
  const bool soft = rt.soft;
  const std::shared_ptr<const GeometryNode::ShaderInputs> wind = rt.wind;
  std::shared_ptr<GraphicsNode> wood = make_part(rt, "_props"), leaves = make_part(rt, "_foliage"),
                                needles = make_part(rt, "_needles");
  WorldRT::PropCell c;
  if (!wood || !leaves || !needles || !w.terrain)
    return c;
  const world::generated_world::prop_cell &pc = w.prop_cells[i];
  const std::string name = "cell" + std::to_string(i) + suffix;
  const cvc::height_field &ground = *w.terrain;
  auto lifted = [&ground](cvc::geometry g) { return with_lift_normals(std::move(g), ground); };
  c.wood = hidden_ladder(*wood, name,
                         prop_ladder({lifted(world::prop_span_mesh(w.props, pc.props)),
                                      lifted(world::prop_span_mesh(w.props_r1, pc.props_r1)),
                                      lifted(world::prop_span_mesh(w.props_r2, pc.props_r2))}),
                         [soft, wind](GeometryNode &g) {
                           style_solid(g, soft);
                           wire_foliage_wind(g, wind);
                         });
  // The leaves and needles are rung 0's only: the coarser rungs carry them as solids.
  if (!pc.foliage.empty())
    c.leaves = hidden_ladder(*leaves, name,
                             prop_ladder({lifted(world::prop_span_mesh(w.foliage, pc.foliage)),
                                          cvc::geometry(), cvc::geometry()}),
                             [soft, wind](GeometryNode &g) {
                               style_foliage(g, soft);
                               wire_foliage_wind(g, wind);
                             });
  if (!pc.needles.empty())
    c.needles = hidden_ladder(*needles, name,
                              prop_ladder({lifted(world::prop_span_mesh(w.needles, pc.needles)),
                                           cvc::geometry(), cvc::geometry()}),
                              [wind](GeometryNode &g) {
                                style_needles(g);
                                wire_foliage_wind(g, wind);
                              });
  return c;
}

// Give every rung of the ground's chunks the terrain node's texture as it is now.
void share_ground_texture(WorldRT &rt) {
  auto t = rt.terrain.lock();
  if (!t)
    return;
  for (const std::weak_ptr<LodGraphicsNode> &c : rt.chunks)
    if (auto n = c.lock())
      for (int k = 0; k < n->rungCount(); ++k)
        if (auto g = n->rung(k))
          g->shareTexture(t);
}

// A new world's textures: the ground's on the terrain node and the water's on the empty mesh under
// <id>_water that holds it (made the first time). Each setTexture makes a new texture, so the
// shown world's rungs keep drawing theirs, and the new world's rungs share these as they are made.
void put_world_textures(WorldRT &rt, const WorldJob &job) {
  const std::vector<unsigned char> &texture = job.texture, &water = job.water;
  const int tw = job.texture_w > 0 ? job.texture_w : rt.texture_size,
            th = job.texture_h > 0 ? job.texture_h : rt.texture_size;
  if (auto t = rt.terrain.lock()) {
    style_terrain(*t, rt.soft);
    if (texture.size() == std::size_t(tw) * std::size_t(th) * 3)
      t->setTexture(surface_image(tw, th, texture), false);
  }
  std::shared_ptr<GraphicsNode> waterGroup =
      rt.show_water ? make_part(rt, "_water") : find_part(rt, "_water");
  if (!waterGroup)
    return;
  auto tex = rt.water_texture.lock();
  if (!tex) {
    tex = std::dynamic_pointer_cast<GeometryNode>(
        waterGroup->createChild<GeometryNode>("texture", cvc::geometry()));
    rt.water_texture = tex;
  }
  const std::size_t n = std::size_t(rt.texture_size) * std::size_t(rt.texture_size) * 4;
  if (tex && water.size() == n)
    tex->setTexture(water_image(rt.texture_size, water), false); // copied: with mipmaps
}

// Ground chunk i of w: a ladder under the terrain node, each rung styled as the terrain, with the
// node's mesh keys and its texture; and, where the chunk has water, a water ladder under
// <id>_water. Both hidden.
void make_chunk(WorldRT &rt, const world::generated_world &w, std::size_t i,
                const std::string &suffix, WorldInstall &in) {
  auto t = rt.terrain.lock();
  if (!t)
    return;
  const world::terrain_chunk &ch = w.terrain_chunks[i];
  const std::string name = "chunk" + std::to_string(i) + suffix;
  const bool soft = rt.soft;
  const std::weak_ptr<GeometryNode> ground = t; // weak: the node owns its chunks
  const std::shared_ptr<const ari::SceneNode> keys = rt.keys;
  cvc::lod::mesh_pyramid pyr;
  pyr.rungs = ch.rungs;
  pyr.world_error_m = ch.error_m;
  in.chunks.push_back(hidden_ladder(*t, name, pyr, [soft, ground, keys](GeometryNode &g) {
    style_terrain(g, soft);
    if (keys)
      apply_mesh_keys(g, *keys);
    if (auto from = ground.lock())
      g.shareTexture(from);
  }));
  std::shared_ptr<LodGraphicsNode> wn;
  std::shared_ptr<GraphicsNode> waterGroup = find_part(rt, "_water");
  const bool wet = std::any_of(ch.water.begin(), ch.water.end(),
                               [](const cvc::geometry &g) { return !g.const_tris().empty(); });
  if (waterGroup && wet) {
    const std::weak_ptr<GeometryNode> waterTex = rt.water_texture;
    cvc::lod::mesh_pyramid wp;
    wp.rungs = ch.water;
    wp.world_error_m = ch.error_m;
    wn = hidden_ladder(*waterGroup, name, wp,
                       [waterTex](GeometryNode &g) { style_water(g, waterTex); });
  }
  in.water.push_back(wn);
}

// The shown world's sea past its edge (world::outer_sea): a world with a shore edge has one, drawn
// as its water is but in its own colours (no texture); any other edge empties the node.
void put_outer_sea(WorldRT &rt, const world::generated_world *w) {
  const cvc::geometry g = w ? world::outer_sea(*w) : cvc::geometry();
  auto n = rt.outer_sea.lock();
  if (!n) {
    if (g.const_points().empty() || !rt.show_water)
      return;
    std::shared_ptr<GraphicsNode> waterGroup = make_part(rt, "_water");
    if (!waterGroup)
      return;
    n = std::dynamic_pointer_cast<GeometryNode>(
        waterGroup->createChild<GeometryNode>("outer_sea", cvc::geometry()));
    rt.outer_sea = n;
    if (!n)
      return;
  }
  n->setGeometry(g);
  style_water(*n, {}); // after setGeometry, as make_mesh's
}

// The shown world's rock beneath it (world::world_underside): a floating world (the wall and none
// edges) has one, lit as the ground's solids are, in its own vertex colours; it casts no shadow
// (nothing lies below it to take one). A shore-edged world empties the node.
void put_underside(WorldRT &rt, const world::generated_world *w) {
  const cvc::geometry g = w && rt.show_underside ? world::world_underside(*w) : cvc::geometry();
  auto n = rt.underside.lock();
  if (!n) {
    if (g.const_points().empty())
      return;
    std::shared_ptr<GraphicsNode> group = find_part(rt, "_underside");
    if (!group) {
      group = make_part(rt, "_underside");
      if (!group)
        return;
      group->setVisible(rt.shown); // a hidden world's rock stays hidden
    }
    n = std::dynamic_pointer_cast<GeometryNode>(
        group->createChild<GeometryNode>("rock", cvc::geometry()));
    rt.underside = n;
    if (!n)
      return;
  }
  n->setGeometry(g);
  n->setUseSingleColor(false); // after setGeometry, as make_mesh's: its vertex colours
  n->setAmbient(0.5);
  n->setDiffuse(0.7);
  n->setSpecular(0.0);
  n->setCastsShadow(false);
}

// A merged mesh of the new world as a hidden child of its holder; none when the world has none.
std::weak_ptr<GeometryNode> make_mesh(GeometryNode &holder, const std::string &name,
                                      const cvc::geometry &g,
                                      const std::function<void(GeometryNode &)> &style) {
  if (g.const_points().empty())
    return {};
  auto n = holder.addGraphicsChild<GeometryNode>(name);
  n->setVisible(false);
  n->setGeometry(g);
  style(*n); // after setGeometry: a LINES mesh's mode and the colour flags are re-applied
  return n;
}

// Each ground chunk's rung for `view`: its own choice from its error, then refined until no
// neighbour is more than one rung off. Returns how many chunks changed rung.
int select_terrain_lod(WorldRT &rt, const cvc::lod::view_params &view) {
  if (!rt.w) // a data node's, before its first world is in
    return 0;
  const world::generated_world &w = *rt.w;
  const world::terrain_chunk_layout &L = w.terrain_layout;
  const std::size_t n = rt.chunks.size();
  if (n == 0 || n != w.terrain_chunks.size())
    return 0;
  std::vector<int> rung(n, 0);
  for (std::size_t i = 0; i < n; ++i) {
    auto c = rt.chunks[i].lock();
    if (!c)
      continue;
    const cvc::bounding_box bb = c->getWorldBoundingBox();
    const double centre[3] = {0.5 * (bb.minx + bb.maxx), 0.5 * (bb.miny + bb.maxy),
                              0.5 * (bb.minz + bb.maxz)};
    const double dx = bb.maxx - bb.minx, dy = bb.maxy - bb.miny, dz = bb.maxz - bb.minz;
    const double dist =
        cvc::lod::bound_distance_m(centre, 0.5 * std::sqrt(dx * dx + dy * dy + dz * dz), view);
    const std::vector<double> &err = w.terrain_chunks[i].error_m;
    rung[i] = rt.chunk_want[i] =
        cvc::lod::select_rung(dist, err.data(), int(err.size()), rt.chunk_want[i], view);
  }
  world::constrain_terrain_rungs(rung, L.chunk_rows, L.chunk_cols);
  int changes = 0;
  for (std::size_t i = 0; i < n; ++i)
    if (auto c = rt.chunks[i].lock(); c && c->selectedRung() != rung[i]) {
      c->setRung(rung[i]);
      if (i < rt.water.size())
        if (auto wn = rt.water[i].lock()) // the chunk's water on the same rung as its ground
          wn->setRung(rung[i]);
      ++changes;
    }
  return changes;
}

// Each prop cell's rung for `view`: its wood ladder chooses, its leaves and needles follow.
// Returns how many cells changed rung.
int select_prop_lod(WorldRT &rt, const cvc::lod::view_params &view) {
  if (rt.cells.empty())
    return 0;
  int changes = 0;
  for (const WorldRT::PropCell &c : rt.cells) {
    auto wood = c.wood.lock();
    if (!wood)
      continue;
    const int before = wood->activeRung();
    const int k = wood->select(view);
    if (k == before)
      continue;
    ++changes;
    if (auto n = c.leaves.lock())
      n->setRung(k);
    if (auto n = c.needles.lock())
      n->setRung(k);
  }
  return changes;
}

// Start putting job's world into the scene (put_world_step does the work).
void begin_world_install(WorldRT &rt, std::shared_ptr<WorldJob> job) {
  auto in = std::make_unique<WorldInstall>();
  const world::generated_world &w = *job->world;
  in->job = std::move(job);
  in->suffix = "_g" + std::to_string(++rt.generation);
  // The textures, each chunk, each cell, the buildings, the roads and the swap.
  in->total = 1 + w.terrain_chunks.size() + w.prop_cells.size() + 2 + 1;
  rt.install = std::move(in);
}

// The parts of the world beside the terrain node, which follow its visibility.
const char *const kWorldParts[] = {"_props", "_foliage", "_needles",  "_buildings",
                                   "_roads", "_water",   "_underside"};

// A world's nodes on their way out: hidden now, removed later (retire_world_step).
void retire_world_nodes(WorldRT &rt, const std::vector<std::weak_ptr<LodGraphicsNode>> &chunks,
                        const std::vector<std::weak_ptr<LodGraphicsNode>> &water,
                        const std::vector<WorldRT::PropCell> &cells,
                        const std::weak_ptr<GeometryNode> &building_mesh,
                        const std::weak_ptr<GeometryNode> &road_mesh) {
  auto retire = [&rt](const std::shared_ptr<GraphicsNode> &parent, const auto &weak) {
    std::shared_ptr<GraphicsNode> n = weak.lock();
    if (!parent || !n)
      return;
    n->setVisible(false);
    rt.retired.nodes.emplace_back(parent, n);
  };
  const std::shared_ptr<GraphicsNode> t = rt.terrain.lock(), wet = find_part(rt, "_water"),
                                      wood = find_part(rt, "_props"),
                                      leaves = find_part(rt, "_foliage"),
                                      needles = find_part(rt, "_needles");
  for (const auto &c : chunks)
    retire(t, c);
  for (const auto &c : water)
    retire(wet, c);
  for (const WorldRT::PropCell &c : cells) {
    retire(wood, c.wood);
    retire(leaves, c.leaves);
    retire(needles, c.needles);
  }
  retire(rt.buildings.lock(), building_mesh);
  retire(rt.roads.lock(), road_mesh);
}

// One piece of a retired world freed (a ground chunk's rungs, a merged mesh, the raster); false
// once there is nothing left to free a piece at a time.
bool free_world_piece(world::generated_world &w) {
  if (!w.terrain_chunks.empty()) {
    w.terrain_chunks.pop_back();
    return true;
  }
  for (cvc::geometry *g :
       {&w.props, &w.props_r1, &w.props_r2, &w.foliage, &w.needles, &w.buildings, &w.roads})
    if (!g->const_points().empty()) {
      *g = cvc::geometry();
      return true;
    }
  if (!w.raster.klass.empty()) {
    w.raster = world::raster_out();
    return true;
  }
  if (w.model) {
    w.model.reset();
    return true;
  }
  return false;
}

// Take the next retired pieces out, for about budget_ms (at least one): the nodes first, then the
// worlds a piece at a time (one shared elsewhere is only let go).
void retire_world_step(WorldRT &rt, double budget_ms) {
  WorldRetired &r = rt.retired;
  const auto start = std::chrono::steady_clock::now();
  do {
    if (!r.nodes.empty()) {
      auto [parent, node] = r.nodes.back();
      r.nodes.pop_back();
      if (auto p = parent.lock())
        if (auto n = node.lock())
          p->removeGraphicsChild(n);
    } else if (!r.worlds.empty()) {
      std::shared_ptr<world::generated_world> &w = r.worlds.back();
      if (!w || w.use_count() > 1 || !free_world_piece(*w))
        r.worlds.pop_back();
    }
  } while (
      !r.empty() &&
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count() <
          budget_ms);
}

// The install's nodes, made so far, out of the scene (a stopped or superseded install): hidden at
// once, removed by retire_world_step.
void drop_world_install(WorldRT &rt) {
  if (!rt.install)
    return;
  WorldInstall &in = *rt.install;
  retire_world_nodes(rt, in.chunks, in.water, in.cells, in.building_mesh, in.road_mesh);
  rt.install.reset();
}

// Keep the install's nodes hidden: a parent shown again (the node's visibility, followed by the
// parts beside it) shows its children.
void hide_world_install(WorldRT &rt) {
  if (!rt.install)
    return;
  WorldInstall &in = *rt.install;
  auto hide = [](const auto &weak) {
    if (auto n = weak.lock())
      n->setVisible(false);
  };
  for (const auto &c : in.chunks)
    hide(c);
  for (const auto &c : in.water)
    hide(c);
  for (const WorldRT::PropCell &c : in.cells) {
    hide(c.wood);
    hide(c.leaves);
    hide(c.needles);
  }
  hide(in.building_mesh);
  hide(in.road_mesh);
}

// The swap, in one frame: the shown world's nodes hidden (and retired), the new world's shown, and
// it is the world shown (its ground published, its dataset, its stats).
void swap_world(WorldRT &rt) {
  WorldInstall &in = *rt.install;
  SceneGraph &sg = *rt.sg;
  const std::string &id = rt.id;
  retire_world_nodes(rt, rt.chunks, rt.water, rt.cells, rt.building_mesh, rt.road_mesh);
  auto show = [](const auto &weak) {
    if (auto n = weak.lock())
      n->setVisible(true);
  };
  for (const auto &c : in.chunks)
    show(c);
  for (const auto &c : in.water)
    show(c);
  for (const WorldRT::PropCell &c : in.cells) {
    show(c.wood);
    show(c.leaves);
    show(c.needles);
  }
  show(in.building_mesh);
  show(in.road_mesh);
  rt.chunks = std::move(in.chunks);
  rt.water = std::move(in.water);
  rt.cells = std::move(in.cells);
  rt.chunk_want.assign(rt.chunks.size(), -1);
  rt.building_mesh = in.building_mesh;
  rt.road_mesh = in.road_mesh;
  // A hidden world stays hidden: the parts beside the node hide what was just shown under them.
  if (!rt.shown)
    for (const char *part : kWorldParts)
      if (auto g = find_part(rt, part))
        g->setVisible(false);
  if (rt.w && rt.w != in.job->world)
    rt.retired.worlds.push_back(std::move(rt.w)); // freed a piece at a time, after its nodes
  rt.w = in.job->world;
  rt.color = in.job->color;
  if (rt.field)
    *rt.field = *rt.w->terrain; // the published ground follows the world
  put_outer_sea(rt, rt.w.get());
  put_underside(rt, rt.w.get());
  if (rt.publishes) { // the world shown is the node's dataset: its physics, its grid,
    // generated_world(id). The task wrote a rebuild's bundle; the first world's is written here.
    std::vector<std::string> why;
    const bool written = !in.job->bundle.empty() || !in.job->bundle_error.empty();
    if (!in.job->bundle_error.empty())
      why.push_back("ari: generated_world '" + id + "': its world as a bundle: " +
                    in.job->bundle_error + "; its physics is the world as made");
    rt.data_path = ari::publish_generated_world(*rt.app, rt.prefix, *rt.keys, rt.w, &why,
                                                written ? &in.job->bundle : nullptr);
    in.job->bundle.clear(); // the dataset holds the bytes now
    if (!why.empty())
      ari::write<std::string>(*rt.app, rt.k.status, why.front());
  } else { // a data node's world: drawn
    ari::write<std::string>(*rt.app, rt.k.status, std::string());
  }
  publish_stats(rt);
  sg.invalidateShadowBake();
}

// Put the install's next pieces into the scene, for about budget_ms (at least one piece). The swap
// is a frame of its own, after the frame that made the last piece. True once the world is in.
bool put_world_step(WorldRT &rt, double budget_ms) {
  using Stage = WorldInstall::Stage;
  WorldInstall &in = *rt.install;
  if (in.stage == Stage::swap) {
    swap_world(rt);
    in.stage = Stage::done;
    ++in.done;
  }
  if (in.stage == Stage::done)
    return true;
  const world::generated_world &w = *in.job->world;
  const auto start = std::chrono::steady_clock::now();
  auto spent_ms = [&start]() {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
        .count();
  };
  do {
    switch (in.stage) {
    case Stage::textures:
      put_world_textures(rt, *in.job);
      in.stage = Stage::chunks;
      break;
    case Stage::chunks:
      if (in.next < w.terrain_chunks.size()) {
        make_chunk(rt, w, in.next++, in.suffix, in);
        break;
      }
      in.next = 0;
      in.stage = Stage::cells;
      continue; // not a piece
    case Stage::cells:
      if (in.next < w.prop_cells.size()) {
        in.cells.push_back(make_prop_cell(rt, w, in.next++, in.suffix));
        break;
      }
      in.next = 0;
      in.stage = Stage::buildings;
      continue;
    case Stage::buildings:
      rt.buildings = mesh_holder(rt, "_buildings", rt.buildings);
      if (auto h = rt.buildings.lock()) {
        const bool soft = rt.soft, arch = w.archetypes;
        in.building_mesh =
            make_mesh(*h, "mesh" + in.suffix, w.buildings,
                      [soft, arch](GeometryNode &g) { style_buildings(g, soft, arch); });
      }
      in.stage = Stage::roads;
      break;
    case Stage::roads:
      rt.roads = mesh_holder(rt, "_roads", rt.roads);
      if (auto h = rt.roads.lock()) {
        const bool soft = rt.soft;
        in.road_mesh = make_mesh(*h, "mesh" + in.suffix, w.roads,
                                 [soft](GeometryNode &g) { style_roads(g, soft); });
      }
      in.stage = Stage::swap;
      break;
    case Stage::swap: // the next call's
    case Stage::done:
      return false;
    }
    ++in.done;
  } while (in.stage != Stage::swap && spent_ms() < budget_ms);
  return false;
}

// The job's world as its bundle (world::generated_world_bundle), its ground the job's texture when
// that is the bundle's ground (the material colours at world::world_ground_size), else painted
// for it.
world::city_bundle_files generated_world_bundle_of(cvc::app &app, const WorldJob &job, int size) {
  world::world_ground ground;
  if (job.color == int(world::surface_color::material) && size == world::world_ground_size &&
      job.texture.size() == std::size_t(size) * std::size_t(size) * 3) {
    ground.width = ground.height = size;
    ground.rgb = job.texture;
  }
  return world::generated_world_bundle(app, *job.world, CVC_VERSION_STRING,
                                       ground.empty() ? nullptr : &ground);
}

// A WorldJob as a sliced task: a rebuild steps a world_builder, paints the new world's texture,
// then writes the world as its bundle; a repaint only paints. Natively each step runs on the
// background pool, on single-threaded wasm a few milliseconds a frame on the scheduler thread;
// either way it writes only its job (a repaint reads the shown world, which nothing changes while
// it runs), and the tick takes the job in once `done` is set. The builder's own stages name the
// phase.
class WorldTask : public cvc::sliced_job {
public:
  WorldTask(cvc::app &app, std::shared_ptr<WorldJob> job, const world::generated_world_params &p,
            const world::building_library *library, std::shared_ptr<ari::ResolvedFile> file,
            int texture_size)
      : _app(&app), _job(std::move(job)), _file(std::move(file)), _size(texture_size) {
    if (_job->rebuild)
      _builder = std::make_unique<world::world_builder>(app, p, library);
    _bundled = !_job->rebuild;
  }

  bool step(std::size_t budget) override {
    if (done())
      return true;
    try {
      if (_builder) {
        _builder->step(budget);
        set(_builder->phase(), kBuildShare * _builder->progress());
        if (_builder->done()) {
          _job->world = std::make_shared<world::generated_world>(_builder->take());
          _builder.reset();
          _file.reset(); // a fetched DEM's temp file is read
        }
        return done();
      }
      if (!_painted) {
        if (!_painter)
          _painter = std::make_unique<world::surface_painter>(
              *_job->world, world::surface_color(_job->color), _size);
        const bool ready = _painter->step(budget);
        const double from = _job->rebuild ? kBuildShare : 0.0, to = _job->rebuild ? kPaintEnd : 1.0;
        set("paint", from + (to - from) * _painter->progress());
        if (ready) {
          _job->texture = _painter->take();
          _job->texture_w = _job->texture_h = _size;
          _job->water = _painter->take_water();
          _painter.reset();
          _painted = true;
          if (_bundled)
            finish();
        }
        return done();
      }
      // The world as its bundle, here rather than on the frame thread when it goes in (a 1.2 km
      // world's is about 30 MB and 1.6 s to write): its dataset is that bundle read back. Its
      // ground is the texture just painted when that is the bundle's (the material colours at
      // world_ground_size); in another colour or size it is painted again for the bundle.
      set("bundle", kPaintEnd);
      try {
        if (cvc::can_write_model_to_memory(".glb"))
          _job->bundle = generated_world_bundle_of(*_app, *_job, _size);
      } catch (const std::exception &e) {
        _job->bundle_error = e.what();
      }
      _bundled = true;
      finish();
    } catch (const std::exception &e) {
      _job->error = e.what();
      finish();
    } catch (...) {
      _job->error = "unknown exception";
      finish();
    }
    return done();
  }
  bool done() const override { return _finished.load() || _cancelled.load(); }
  void cancel() override { _cancelled = true; }
  bool cancelled() const override { return _cancelled.load(); }
  const char *phase() const override { return _phase.load(); }
  double progress() const override { return _progress.load(); }
  std::string status() const override { return _finished.load() ? "done" : "generating"; }

private:
public:
  // A rebuild's progress: building to kBuildShare, painting to kPaintEnd, and putting the world
  // into the scene (put_world_step, in the tick) the rest.
  static constexpr double kBuildShare = 0.7, kPaintEnd = 0.85;

private:
  void set(const char *phase, double progress) {
    _phase = phase;
    _progress = std::max(_progress.load(), std::min(progress, 0.999));
    _job->phase.store(phase, std::memory_order_relaxed);
    _job->progress = static_cast<int>(100.0 * _progress.load());
  }
  void finish() {
    _progress = 1.0;
    _finished = true;
    _job->done.store(true, std::memory_order_release);
  }

  cvc::app *_app;
  std::shared_ptr<WorldJob> _job;
  std::shared_ptr<ari::ResolvedFile> _file;
  int _size;
  bool _bundled = true, _painted = false;
  std::unique_ptr<world::world_builder> _builder;
  std::unique_ptr<world::surface_painter> _painter;
  std::atomic<const char *> _phase{"terrain"};
  std::atomic<double> _progress{0.0};
  std::atomic<bool> _finished{false}, _cancelled{false};
};

// End the running job, if any: its task stops at its next check, the nodes of a world it was
// putting into the scene go, and the shown world stays.
void stop_world_job(WorldRT &rt) {
  if (rt.task >= 0)
    cvc::cancel_sliced_job(*rt.app, rt.task);
  rt.task = -1;
  if (rt.sg)
    drop_world_install(rt);
  rt.job.reset();
  ari::write<int>(*rt.app, rt.k.busy, 0);
  ari::write<int>(*rt.app, rt.k.progress, 0);
  ari::write<std::string>(*rt.app, rt.k.phase, std::string());
}

// Start a rebuild of p's world (rebuild) or a repaint of the shown one, in `color`, ending the job
// that was running.
void launch_world_job(WorldRT &rt, bool rebuild, const world::generated_world_params &p,
                      int color) {
  cvc::app &a = *rt.app;
  stop_world_job(rt);
  auto job = std::make_shared<WorldJob>();
  job->rebuild = rebuild;
  job->color = color;
  if (!rebuild)
    job->world = rt.w;
  rt.job = job;
  ari::write<int>(a, rt.k.busy, 1);
  ari::write<std::string>(a, rt.k.status, rebuild ? "generating..." : "painting...");
  auto file = std::make_shared<ari::ResolvedFile>();
  world::generated_world_params rp;
  try {
    rp = ari::resolve_generated_world_file(a, p, rt.base_dir, *file);
  } catch (const std::exception &e) {
    job->error = e.what();
    job->done.store(true, std::memory_order_release);
    return;
  }
  cvc::sliced_task_options o;
  o.kind = "generated_world";
  o.subject = std::string(rebuild ? "generate " : "paint ") + rt.k.request;
  o.owner = "generated_world:" + rt.k.request;
  std::string why;
  rt.task = cvc::schedule_sliced_job(
      a, std::make_shared<WorldTask>(a, job, rp, rt.library, std::move(file), rt.texture_size), o,
      &why);
  if (rt.task < 0) {
    job->error = "the job could not start: " + why;
    job->done.store(true, std::memory_order_release);
  }
}

// The job is over: nothing is busy.
void end_world_job(WorldRT &rt, int progress) {
  cvc::app &a = *rt.app;
  rt.job.reset();
  rt.task = -1;
  ari::write<int>(a, rt.k.busy, 0);
  ari::write<std::string>(a, rt.k.phase, std::string());
  ari::write<int>(a, rt.k.progress, progress);
}

// Take a finished job in: a new world starts going into the scene (put_world_step; the job stays
// the running one until it is in), or the shown world takes its new texture. A failed one keeps
// what is shown and says why.
void apply_world_job(WorldRT &rt) {
  cvc::app &a = *rt.app;
  std::shared_ptr<WorldJob> j = rt.job;
  rt.task = -1;
  if (!j->error.empty()) {
    end_world_job(rt, 0);
    ari::write<std::string>(a, rt.k.status, "error: " + j->error);
    return;
  }
  if (j->rebuild) {
    begin_world_install(rt, j);
    ari::write<std::string>(a, rt.k.status, "putting the new world in...");
    return;
  }
  rt.color = j->color;
  if (auto t = rt.terrain.lock())
    t->setTexture(surface_image(j->texture_w, j->texture_h, j->texture), false);
  share_ground_texture(rt);
  publish_stats(rt);
  end_world_job(rt, 100);
}

// How long a frame spends putting a new world into the scene: a piece at least (a ground chunk's
// ladder, a prop cell's), then more while this lasts.
constexpr double kInstallBudgetMs = 6.0;

// A frame of a world's drawing, the generated node's and a data node's alike: the parts beside the
// terrain node follow its visibility, the trees sway, a finished job is taken in, a made world goes
// in a few milliseconds' worth, a retired one goes out, and the LOD is chosen for this frame's
// camera.
void tick_world(WorldRT &rt, vtkRenderer *ren) {
  if (rt.terrain.expired())
    return;
  // The meshes and the water beside the node follow its visibility, so a `visible:` on the node
  // hides (and shows) the whole world, not only the ground it draws itself. Each follows a change
  // of the node's, as a child follows its parent's (SceneNode::propagateVisible).
  if (auto t = rt.terrain.lock()) {
    const bool shown = t->isVisibleInHierarchy();
    if (shown != rt.shown) {
      rt.shown = shown;
      for (const char *part : kWorldParts)
        if (auto g = find_part(rt, part))
          g->setVisible(shown);
      hide_world_install(rt); // a world still going in stays hidden under them
    }
  }
  cvc::app &app = *rt.app;
  rt.frame++;
  tick_foliage_wind(rt, ren); // the trees' sway in the scene's wind, for this frame's draw
  auto show_progress = [&app, &rt](const char *phase, int percent) {
    ari::write<int>(app, rt.k.progress, percent);
    ari::write<std::string>(app, rt.k.phase,
                            phase ? std::string(phase) + " " + std::to_string(percent) + "%"
                                  : std::string());
  };
  if (rt.job && !rt.install) {
    show_progress(rt.job->phase.load(std::memory_order_relaxed), rt.job->progress.load());
    if (rt.job->done.load(std::memory_order_acquire))
      apply_world_job(rt);
  }
  // A made world goes into the scene a few milliseconds a frame (put_world_step), then is swapped
  // in at once; its LOD is chosen below, before the frame draws it.
  // Once no world is going in, the nodes of the one swapped out (or of a stopped install) leave
  // the scene a few a frame, and that world is freed after them (retire_world_step).
  if (!rt.install && !rt.retired.empty())
    retire_world_step(rt, kInstallBudgetMs);
  if (rt.install) {
    const bool in = put_world_step(rt, kInstallBudgetMs);
    const WorldInstall &wi = *rt.install;
    const double share = double(wi.done) / double(std::max<std::size_t>(1, wi.total));
    show_progress("install", int(100.0 * (WorldTask::kPaintEnd +
                                          (1.0 - WorldTask::kPaintEnd) * std::min(1.0, share))));
    if (in) {
      rt.install.reset();
      end_world_job(rt, 100);
    }
  }
  // The props' and the ground's LOD for this frame's camera; a caster that changed rung casts a
  // different shadow.
  if (ren && ren->IsActiveCameraCreated()) {
    const cvc::lod::view_params view = make_view_params(ren, cvc::lod::preset_view(kLodPreset));
    int recast = select_prop_lod(rt, view);
    const int ground = select_terrain_lod(rt, view);
    if (auto t = rt.terrain.lock(); t && t->castsShadow())
      recast += ground;
    if (recast > 0)
      rt.sg->invalidateShadowBake();
  }
}

// type: generated_world — see world_nodes.h.
std::shared_ptr<GraphicsNode> realize_generated_world(SceneGraph &sg, const ari::SceneNode &n,
                                                      GraphicsNode *parent, RealizedScene &out,
                                                      std::vector<std::string> *warnings) {
  auto rt = std::make_shared<WorldRT>();
  rt->app = &sg.appContext();
  rt->sg = &sg;
  rt->id = n.id;
  cvc::app &app = *rt->app;
  rt->base = ari::generated_world_params_from(n);
  rt->base_dir = n.base_dir;
  rt->color = color_from(n.props.str("color", "material"), 0);
  rt->texture_size = std::max(256, std::min(4096, int(n.props.num("texture", 2048.0))));
  rt->soft = n.props.str("look", "soft") != "classic";
  rt->show_water = n.props.num("water", 1.0) != 0.0;
  rt->show_underside = n.props.num("underside", 1.0) != 0.0;
  rt->library = &ari::generated_world_library(app, n, warnings);

  // world.*, or under the node's `keys:` prop (node_key) so two worlds keep separate panels.
  rt->k = ari::generated_world_keys(app, n, sg.getStatePrefix(), rt->base, rt->color);
  // The trees in the scene's wind: world.sway (its strength, from the `sway` prop, 1), and the way
  // they lean published at world.lean_dir.
  rt->prefix = sg.getStatePrefix();
  ari::seed_shown_clock(sg.appContext(), rt->prefix); // its foliage sways at the shown time (OW5q)
  rt->sway_key = ari::resolve_bind(rt->prefix, ari::node_key(n, "world.sway"));
  rt->lean_key = ari::resolve_bind(rt->prefix, ari::node_key(n, "world.lean_dir"));
  ari::read_or_seed<double>(app, rt->sway_key, n.props.num("sway", 1.0));
  world::generated_world_params p0 = ari::read_generated_world_params(app, rt->k, rt->base);
  rt->preset = p0.preset;
  rt->color = int(ari::read_or<double>(app, rt->k.color, rt->color));
  rt->w = ari::first_generated_world(app, rt->k, n, p0, rt->library, warnings);
  rt->sig = ari::generated_world_signature(p0);

  // The node is the terrain; its height_field is what `ground: <id>` and a body stand on. It draws
  // through its chunks (make_chunk): its own mesh stays empty, and it holds their texture.
  rt->field = std::make_shared<cvc::height_field>(*rt->w->terrain);
  out.height_fields[n.id] = rt->field;
  auto keys = std::make_shared<ari::SceneNode>(n);
  keys->children.clear(); // realize_scene realizes those itself
  rt->keys = keys;
  const cvc::geometry none;
  std::shared_ptr<GraphicsNode> gn =
      parent ? parent->createChild<GeometryNode>(n.id, none) : sg.addGraphics(n.id, none);
  auto tnode = std::dynamic_pointer_cast<GeometryNode>(gn);
  if (!tnode) {
    if (warnings)
      warnings->push_back("ari: generated_world '" + n.id + "': could not create the terrain");
    return gn;
  }
  rt->terrain = tnode;
  { // the ground's texture and the water's, in the colour mode (the water colour mode paints both),
    // and the first world into the scene, all of it now
    world::surface_painter painter(*rt->w, world::surface_color(rt->color), rt->texture_size);
    painter.step(0);
    auto first = std::make_shared<WorldJob>();
    first->world = rt->w;
    first->color = rt->color;
    first->texture = painter.take();
    first->texture_w = first->texture_h = rt->texture_size;
    first->water = painter.take_water();
    try { // its dataset: the world as its bundle, its ground this texture when it can be
      if (cvc::can_write_model_to_memory(".glb"))
        first->bundle = generated_world_bundle_of(app, *first, rt->texture_size);
    } catch (const std::exception &e) {
      first->bundle_error = e.what();
    }
    begin_world_install(*rt, first);
    while (!put_world_step(*rt, 1e9)) {
    }
    rt->install.reset();
  }
  // The world went in as the node's dataset (swap_world); a body: on the node is its world.
  out.body_data[n.id] = rt->data_path;

  out.custom_ticks.push_back([rt](vtkRenderer *ren) {
    if (rt->terrain.expired())
      return;
    cvc::app &app = *rt->app;
    // Regenerate, Randomize, Stop and Save, a new preset, and world.auto (poll_generated_world). A
    // new colour alone repaints the shown world at once, once no rebuild is running (a rebuild
    // paints in the colour asked when it started, and a later change is repainted after it). Save
    // writes the shown world's dataset, at once.
    const ari::GeneratedWorldPoll poll =
        ari::poll_generated_world(app, rt->k, rt->base, rt->preset, rt->sig);
    if (poll.save)
      ari::save_generated_world(app, rt->k, rt->data_path, rt->base_dir);
    if (poll.stop && rt->job) {
      stop_world_job(*rt);
      ari::write<std::string>(app, rt->k.status, "stopped: the previous world is shown");
    }
    const int c = int(ari::read_or<double>(app, rt->k.color, rt->color));
    if (poll.rebuild) {
      launch_world_job(*rt, true, poll.params, c);
    } else if (!rt->job ? c != rt->color : !rt->job->rebuild && rt->job->color != c) {
      launch_world_job(*rt, false, poll.params, c);
    }
    tick_world(*rt, ren);
  });
  return gn;
}

// ── A data node's world (DataWorldView) ──────────────────────────────────────────────────────────

// What a world dataset holds, as the city_bundle decoder left it in the store: its world, the
// manifest's fields, its aerial image, and its files by name (the last path component; the first
// of a name wins, as the decoder's own lookup does). Read on the frame thread; the job reads only
// this.
struct WorldDataset {
  std::shared_ptr<const world::world_geometry> geometry;
  int version = 1;
  std::string hash, label;
  world::city_bundle_generator generator;
  std::optional<double> water_level;
  std::shared_ptr<const cvc::image> satellite; // decoded already: satellite.png is left out below
  std::map<std::string, ari::DataBytes> files;
};

// An aerial image (rows north first, any format) as a world_ground (RGB, rows south first).
world::world_ground ground_from_image(const cvc::image &img) {
  world::world_ground g;
  const cvc::image rgb =
      img.converted(cvc::image::pixel_format::RGB, cvc::image::data_type::u8).flipped_vertical();
  if (rgb.empty() || !rgb.data())
    return g;
  g.width = rgb.width();
  g.height = rgb.height();
  const unsigned char *px = rgb.data();
  g.rgb.assign(px, px + std::size_t(g.width) * std::size_t(g.height) * 3);
  return g;
}

// `tint: satellite`: each vertex of `g` coloured by the ground under it, over a grey floor so
// shaded facades still read (0.30 + 0.70 x the photo; 0.32 + 0.68 for blue), as the nav demos
// tinted Austin. The ground spans the world's grid.
void tint_from_ground(cvc::geometry &g, const world::world_ground &ground,
                      const world::grid_spec &grid) {
  const int W = ground.width, H = ground.height;
  const double x0 = grid.min_x, y0 = grid.min_y, w = grid.max_x - x0, h = grid.max_y - y0;
  if (ground.empty() || W <= 0 || H <= 0 || !(w > 0.0) || !(h > 0.0))
    return;
  const auto &p = g.const_points();
  cvc::geometry::colors_t cols(p.size());
  for (std::size_t i = 0; i < p.size(); ++i) {
    const double u = std::clamp((p[i][0] - x0) / w, 0.0, 1.0);
    const double v = std::clamp((p[i][1] - y0) / h, 0.0, 1.0);
    const int ix = std::min(W - 1, int(u * W)), iy = std::min(H - 1, int(v * H)); // row 0: south
    const unsigned char *q = &ground.rgb[(std::size_t(iy) * W + ix) * 3];
    cols[i] = {0.30 + 0.70 * q[0] / 255.0, 0.30 + 0.70 * q[1] / 255.0, 0.32 + 0.68 * q[2] / 255.0};
  }
  g.colors() = std::move(cols);
}

// The drawn world from a world dataset, as a sliced job (WorldTask's for a generated world): the
// world (world::world_from_bundle: the meshes, the cells and streets, the terrain's LOD chunks and
// its water rebuilt), the meshes' colours, then the water's texture. A stage a step: natively on
// the background pool, on single-threaded wasm a stage a frame.
class DataWorldTask : public cvc::sliced_job {
public:
  DataWorldTask(cvc::app &app, std::shared_ptr<WorldJob> job, WorldDataset data, bool tint,
                int water_size)
      : _app(&app), _job(std::move(job)), _data(std::move(data)), _tint(tint),
        _water_size(water_size) {}

  bool step(std::size_t) override {
    if (done())
      return true;
    try {
      switch (_stage) {
      case 0:
        make_world();
        set("colour", 0.4);
        break;
      case 1:
        colour_meshes();
        set("water", 0.6);
        break;
      default:
        paint();
        finish();
        break;
      }
      ++_stage;
    } catch (const std::exception &e) {
      _job->error = e.what();
      finish();
    } catch (...) {
      _job->error = "unknown exception";
      finish();
    }
    return done();
  }
  bool done() const override { return _finished.load() || _cancelled.load(); }
  void cancel() override { _cancelled = true; }
  bool cancelled() const override { return _cancelled.load(); }
  const char *phase() const override { return _phase.load(); }
  double progress() const override { return _progress.load(); }
  std::string status() const override { return _finished.load() ? "done" : "drawing"; }

private:
  // The bundle as the decoder read it, and the drawn world from it. The world keeps the bundle
  // (its surface table is the bundle's), so it lives as long as the world does.
  void make_world() {
    if (!_data.geometry)
      throw std::runtime_error("the dataset has no world");
    auto b = std::make_shared<world::city_bundle>();
    b->geometry = *_data.geometry; // copy-on-write: the meshes are shared
    b->version = _data.version;
    b->hash = _data.hash;
    b->generator = _data.generator;
    b->water_level = _data.water_level;
    const std::map<std::string, ari::DataBytes> &files = _data.files;
    world::bundle_world bw = world::world_from_bundle(
        *_app, *b, [&files](const std::string &name) -> std::shared_ptr<const std::string> {
          const auto f = files.find(name);
          return f == files.end() ? nullptr : f->second;
        });
    if (bw.ground.empty() && _data.satellite)
      bw.ground = ground_from_image(*_data.satellite);
    std::shared_ptr<world::generated_world> keep = std::move(bw.world);
    world::generated_world *raw = keep.get();
    _job->world = std::shared_ptr<world::generated_world>(
        raw, [keep = std::move(keep), b](world::generated_world *) mutable { keep.reset(); });
    _ground = std::move(bw.ground);
    _bundle = std::move(b);
  }

  // A mesh without colours of its own takes its surfaces' (as the data node's plain draw gave
  // them); with tint: satellite, the buildings and the roads take the ground's.
  void colour_meshes() {
    world::generated_world &w = *_job->world;
    auto colour = [this, &w](cvc::geometry &g, const char *item) {
      if (g.const_points().empty() || g.const_colors().size() == g.num_points() || !w.surfaces)
        return;
      const world::geometry_item *it = _bundle->geometry.find(item);
      g.colors() = cvc::gl::surface_colors(g, *w.surfaces, it ? it->surface : 0);
    };
    colour(w.buildings, "buildings");
    colour(w.roads, "roads");
    if (_tint && !_ground.empty()) {
      tint_from_ground(w.buildings, _ground, w.grid);
      tint_from_ground(w.roads, _ground, w.grid);
    }
  }

  // The ground's texture is the dataset's (ground.png, else the aerial image); without one, the
  // terrain's surface colour. The water's is painted when the world has any.
  void paint() {
    world::generated_world &w = *_job->world;
    if (!_ground.empty()) {
      _job->texture_w = _ground.width;
      _job->texture_h = _ground.height;
      _job->texture = std::move(_ground.rgb);
    } else {
      float rgb[3] = {0.55f, 0.53f, 0.48f};
      if (w.surfaces) {
        const world::geometry_item *t = _bundle->geometry.find("terrain");
        const std::uint16_t s = t ? t->surface : 0;
        if (s < w.surfaces->size())
          for (int k = 0; k < 3; ++k)
            rgb[k] = (*w.surfaces)[s].albedo[k];
      }
      _job->texture_w = _job->texture_h = 2;
      _job->texture.resize(2 * 2 * 3);
      for (std::size_t i = 0; i < _job->texture.size(); ++i)
        _job->texture[i] = (unsigned char)std::lround(std::clamp(rgb[i % 3], 0.0f, 1.0f) * 255.0f);
    }
    const bool wet = std::any_of(
        w.terrain_chunks.begin(), w.terrain_chunks.end(), [](const world::terrain_chunk &ch) {
          return std::any_of(ch.water.begin(), ch.water.end(),
                             [](const cvc::geometry &g) { return !g.const_tris().empty(); });
        });
    if (wet)
      _job->water = world::paint_water(w, _water_size);
  }

  void set(const char *phase, double progress) {
    _phase = phase;
    _progress = std::max(_progress.load(), std::min(progress, 0.999));
    _job->phase.store(phase, std::memory_order_relaxed);
    _job->progress = static_cast<int>(100.0 * WorldTask::kPaintEnd * _progress.load());
  }
  void finish() {
    _progress = 1.0;
    _finished = true;
    _job->done.store(true, std::memory_order_release);
  }

  cvc::app *_app;
  std::shared_ptr<WorldJob> _job;
  WorldDataset _data;
  bool _tint;
  int _water_size;
  int _stage = 0;
  std::shared_ptr<world::city_bundle> _bundle;
  world::world_ground _ground;
  std::atomic<const char *> _phase{"world"};
  std::atomic<double> _progress{0.0};
  std::atomic<bool> _finished{false}, _cancelled{false};
};

// The world dataset at store path `store`, read off the state tree (WorldDataset).
WorldDataset read_world_dataset(cvc::app &app, const std::string &store) {
  WorldDataset d;
  cvc::state &root = cvc::state::instance(app);
  auto value = [&root, &store](const std::string &child) {
    cvc::state *s = root.findDescendant(store + cvc::state::SEPARATOR + child);
    return s ? s->value() : std::string();
  };
  if (cvc::state *s = root.findDescendant(store + cvc::state::SEPARATOR + "world")) {
    try {
      d.geometry = boost::any_cast<std::shared_ptr<const world::world_geometry>>(s->data());
    } catch (const boost::bad_any_cast &) {
    }
  }
  if (cvc::state *s = root.findDescendant(store + cvc::state::SEPARATOR + "satellite")) {
    try {
      d.satellite = boost::any_cast<std::shared_ptr<const cvc::image>>(s->data());
    } catch (const boost::bad_any_cast &) {
    }
  }
  try {
    const std::string v = value("version");
    d.version = v.empty() ? 1 : std::stoi(v);
    const std::string level = value("water_level");
    if (!level.empty())
      d.water_level = std::stod(level);
    const std::string seed = value("generator.seed");
    d.generator.seed = seed.empty() ? 0 : std::stoull(seed);
  } catch (const std::exception &) { // a value the decoder did not write: the defaults
  }
  d.hash = value("hash");
  d.generator.name = value("generator");
  d.generator.version = value("generator.version");
  d.generator.params = value("generator.params");
  if (d.generator.params.empty())
    d.generator.params = "{}";
  d.label = value("uri");
  if (d.label.empty())
    d.label = store;
  for (const std::string &path : ari::data_files(app, store)) {
    const std::size_t slash = path.find_last_of('/');
    const std::string name = slash == std::string::npos ? path : path.substr(slash + 1);
    if (d.files.count(name) || (name == "satellite.png" && d.satellite))
      continue;
    if (ari::DataBytes bytes = ari::data_file(app, store, path))
      d.files.emplace(name, std::move(bytes));
  }
  return d;
}

} // namespace

class DataWorldView {
public:
  std::shared_ptr<WorldRT> rt;
  std::shared_ptr<const ari::SceneNode> spec; // the data node, its children left out
  bool tint = false;
};

std::shared_ptr<DataWorldView> make_data_world_view(SceneGraph &sg, const ari::SceneNode &n,
                                                    const std::shared_ptr<GraphicsNode> &group,
                                                    const std::string &bind_prefix) {
  auto v = std::make_shared<DataWorldView>();
  auto rt = std::make_shared<WorldRT>();
  cvc::app &app = sg.appContext();
  rt->app = &app;
  rt->sg = &sg;
  rt->id = n.id;
  rt->rooted = true;
  rt->root = group;
  rt->publishes = false;
  rt->soft = n.props.str("look", "soft") != "classic";
  rt->show_water = n.props.num("water", 1.0) != 0.0;
  rt->show_underside = n.props.num("underside", 0.0) != 0.0;
  rt->texture_size = std::max(256, std::min(4096, int(n.props.num("texture", 2048.0))));
  // Its progress, under <id>.draw.* (busy, progress, phase, status), and the trees' sway.
  auto key = [&bind_prefix, &n](const char *leaf) {
    return ari::resolve_bind(bind_prefix, n.id + ".draw." + leaf);
  };
  rt->k.busy = key("busy");
  rt->k.progress = key("progress");
  rt->k.phase = key("phase");
  rt->k.status = key("status");
  rt->prefix = bind_prefix;
  rt->sway_key = key("sway");
  rt->lean_key = key("lean_dir");
  ari::read_or_seed<double>(app, rt->sway_key, n.props.num("sway", 1.0));
  auto spec = std::make_shared<ari::SceneNode>(n);
  spec->children.clear(); // realize_scene realizes those itself
  rt->keys = spec;
  v->spec = spec;
  v->rt = std::move(rt);
  v->tint = n.props.str("tint") == "satellite";
  return v;
}

void data_world_load(DataWorldView &v, const std::string &store) {
  WorldRT &rt = *v.rt;
  cvc::app &a = *rt.app;
  if (!rt.terrain.lock()) { // the terrain node, made with the first world: it holds the texture
    auto group = rt.root.lock();
    if (!group)
      return;
    rt.terrain = group->createChild<GeometryNode>(rt.id + "_terrain", cvc::geometry());
  }
  stop_world_job(rt);
  auto job = std::make_shared<WorldJob>();
  job->rebuild = true;
  rt.job = job;
  ari::write<int>(a, rt.k.busy, 1);
  ari::write<std::string>(a, rt.k.status, "drawing...");
  cvc::sliced_task_options o;
  o.kind = "data_world";
  o.subject = "draw " + store;
  o.owner = "data_world:" + store;
  std::string why;
  rt.task =
      cvc::schedule_sliced_job(a,
                               std::make_shared<DataWorldTask>(a, job, read_world_dataset(a, store),
                                                               v.tint, rt.texture_size),
                               o, &why);
  if (rt.task < 0) {
    job->error = "the job could not start: " + why;
    job->done.store(true, std::memory_order_release);
  }
}

void data_world_clear(DataWorldView &v) {
  WorldRT &rt = *v.rt;
  if (rt.job)
    stop_world_job(rt);
  retire_world_nodes(rt, rt.chunks, rt.water, rt.cells, rt.building_mesh, rt.road_mesh);
  rt.chunks.clear();
  rt.water.clear();
  rt.cells.clear();
  rt.chunk_want.clear();
  rt.building_mesh.reset();
  rt.road_mesh.reset();
  if (rt.w)
    rt.retired.worlds.push_back(std::move(rt.w));
  rt.w.reset();
  put_outer_sea(rt, nullptr);
  put_underside(rt, nullptr);
  if (auto t = rt.terrain.lock()) // the terrain node goes: nothing of the world is left drawn
    if (auto group = rt.root.lock()) {
      t->setVisible(false);
      rt.retired.nodes.emplace_back(group, t);
    }
  rt.terrain.reset();
}

void data_world_tick(DataWorldView &v, vtkRenderer *ren) {
  WorldRT &rt = *v.rt;
  if (rt.terrain.expired()) { // cleared: only what is on its way out
    if (!rt.retired.empty())
      retire_world_step(rt, kInstallBudgetMs);
    return;
  }
  tick_world(rt, ren);
}

bool data_world_busy(const DataWorldView &v) { return v.rt->job != nullptr || v.rt->install; }

bool data_world_finish(DataWorldView &v) {
  WorldRT &rt = *v.rt;
  if (rt.task >= 0)
    cvc::wait_sliced_job(*rt.app, rt.task, 300.0);
  if (rt.job && !rt.install && rt.job->done.load(std::memory_order_acquire))
    apply_world_job(rt);
  while (rt.install)
    if (put_world_step(rt, 1e9)) {
      rt.install.reset();
      end_world_job(rt, 100);
    }
  while (!rt.retired.empty())
    retire_world_step(rt, 1e9);
  return rt.w != nullptr && !rt.terrain.expired();
}

std::shared_ptr<const world::generated_world> data_world_shown(const DataWorldView &v) {
  return v.rt->w;
}

std::string data_world_status(const DataWorldView &v) {
  return ari::read_or<std::string>(*v.rt->app, v.rt->k.status, std::string());
}

void register_world_node_types(cvc::app &app) {
  register_scene_node_type(app, "generated_world", realize_generated_world);
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
