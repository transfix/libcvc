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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/bind.h> // live node params: read_or_seed / read_or / write / resolve_bind
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_generated_world.h> // the node in core: params, keys, stats, colliders
#include <cvc/ariadne/uri.h>
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h>
#include <cvc/core/sliced_task.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/height_field.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/LodGraphicsNode.h> // the props' and the ground's ladders, make_view_params
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/world_nodes.h>
#include <cvc/image/image.h>
#include <cvc/volume/bounding_box.h>
#include <cvc/world/building_archetypes.h>
#include <cvc/world/generated_world.h>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>
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
  std::vector<unsigned char> texture;
  std::vector<unsigned char> water; // world::paint_water's RGBA texels
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
  ari::GeneratedWorldKeys k;
  world::generated_world_params base; // the authored props
  std::string base_dir;               // the document's, for a relative terrain_file
  const world::building_library *library = nullptr;
  std::shared_ptr<world::generated_world> w; // the world shown
  // What a body: on the node collides with (the terrain and the buildings): the scene's world
  // swaps them when a new world lands.
  std::shared_ptr<ari::SceneColliders> colliders = std::make_shared<ari::SceneColliders>();
  bool bodies = false; // the node declares a body:, so its colliders are made
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
  std::shared_ptr<WorldJob> job; // the running job, or null
  int task = -1;                 // its sliced task's pid
  // A made world going into the scene a piece at a time (put_world_step), or null. The job stays
  // the running one until it is in.
  std::unique_ptr<WorldInstall> install;
  WorldRetired retired; // what is on its way out (retire_world_step)

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
cvc::image surface_image(int size, const std::vector<unsigned char> &rgb) {
  return cvc::image(size, size, cvc::image::pixel_format::RGB, cvc::image::data_type::u8,
                    rgb.data())
      .flipped_vertical();
}
// world::paint_water's RGBA texels as an image, the same way up.
cvc::image water_image(int size, const std::vector<unsigned char> &rgba) {
  return cvc::image(size, size, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8,
                    rgba.data())
      .flipped_vertical();
}

void publish_stats(WorldRT &rt) { ari::publish_generated_world_stats(*rt.app, rt.k, *rt.w); }

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

// The top-level node `name` that holds a merged mesh as its child (WorldRT::buildings, roads):
// made the first time, drawing nothing itself.
std::weak_ptr<GeometryNode> mesh_holder(SceneGraph &sg, const std::string &name,
                                        std::weak_ptr<GeometryNode> have) {
  if (auto n = have.lock())
    return n;
  return std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics(name, cvc::geometry()));
}

// A group node at the top level: found, or made the first time.
std::shared_ptr<GraphicsNode> group_node(SceneGraph &sg, const std::string &name) {
  std::shared_ptr<GraphicsNode> g = sg.getGraphics(name);
  return g ? g : sg.addGraphics(name);
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
// <id>_needles, hidden.
WorldRT::PropCell make_prop_cell(SceneGraph &sg, const std::string &id,
                                 const world::generated_world &w, std::size_t i,
                                 const std::string &suffix, bool soft) {
  std::shared_ptr<GraphicsNode> wood = group_node(sg, id + "_props"),
                                leaves = group_node(sg, id + "_foliage"),
                                needles = group_node(sg, id + "_needles");
  WorldRT::PropCell c;
  if (!wood || !leaves || !needles)
    return c;
  const world::generated_world::prop_cell &pc = w.prop_cells[i];
  const std::string name = "cell" + std::to_string(i) + suffix;
  c.wood = hidden_ladder(*wood, name,
                         prop_ladder({world::prop_span_mesh(w.props, pc.props),
                                      world::prop_span_mesh(w.props_r1, pc.props_r1),
                                      world::prop_span_mesh(w.props_r2, pc.props_r2)}),
                         [soft](GeometryNode &g) { style_solid(g, soft); });
  // The leaves and needles are rung 0's only: the coarser rungs carry them as solids.
  if (!pc.foliage.empty())
    c.leaves = hidden_ladder(*leaves, name,
                             prop_ladder({world::prop_span_mesh(w.foliage, pc.foliage),
                                          cvc::geometry(), cvc::geometry()}),
                             [soft](GeometryNode &g) { style_foliage(g, soft); });
  if (!pc.needles.empty())
    c.needles = hidden_ladder(*needles, name,
                              prop_ladder({world::prop_span_mesh(w.needles, pc.needles),
                                           cvc::geometry(), cvc::geometry()}),
                              [](GeometryNode &g) { style_needles(g); });
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
void put_world_textures(SceneGraph &sg, const std::string &id, WorldRT &rt,
                        const std::vector<unsigned char> &texture,
                        const std::vector<unsigned char> &water) {
  if (auto t = rt.terrain.lock()) {
    style_terrain(*t, rt.soft);
    t->setTexture(surface_image(rt.texture_size, texture), false);
  }
  std::shared_ptr<GraphicsNode> waterGroup = sg.getGraphics(id + "_water");
  if (rt.show_water && !waterGroup)
    waterGroup = sg.addGraphics(id + "_water");
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
void make_chunk(SceneGraph &sg, const std::string &id, WorldRT &rt, const world::generated_world &w,
                std::size_t i, const std::string &suffix, WorldInstall &in) {
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
  std::shared_ptr<GraphicsNode> waterGroup = sg.getGraphics(id + "_water");
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
const char *const kWorldParts[] = {"_props",     "_foliage", "_needles",
                                   "_buildings", "_roads",   "_water"};

// A world's nodes on their way out: hidden now, removed later (retire_world_step).
void retire_world_nodes(SceneGraph &sg, const std::string &id, WorldRT &rt,
                        const std::vector<std::weak_ptr<LodGraphicsNode>> &chunks,
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
  const std::shared_ptr<GraphicsNode> t = rt.terrain.lock(), wet = sg.getGraphics(id + "_water"),
                                      wood = sg.getGraphics(id + "_props"),
                                      leaves = sg.getGraphics(id + "_foliage"),
                                      needles = sg.getGraphics(id + "_needles");
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
void drop_world_install(SceneGraph &sg, const std::string &id, WorldRT &rt) {
  if (!rt.install)
    return;
  WorldInstall &in = *rt.install;
  retire_world_nodes(sg, id, rt, in.chunks, in.water, in.cells, in.building_mesh, in.road_mesh);
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
// it is the world shown (its ground published, its colliders, its stats).
void swap_world(SceneGraph &sg, const std::string &id, WorldRT &rt) {
  WorldInstall &in = *rt.install;
  retire_world_nodes(sg, id, rt, rt.chunks, rt.water, rt.cells, rt.building_mesh, rt.road_mesh);
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
      if (auto g = sg.getGraphics(id + part))
        g->setVisible(false);
  if (rt.w && rt.w != in.job->world)
    rt.retired.worlds.push_back(std::move(rt.w)); // freed a piece at a time, after its nodes
  rt.w = in.job->world;
  rt.color = in.job->color;
  *rt.field = *rt.w->terrain; // the published ground follows the world
  if (rt.bodies) {            // only a node with a body: needs them
    rt.colliders->geometry = ari::generated_world_colliders(*rt.w);
    ++rt.colliders->revision;
  }
  publish_stats(rt);
  sg.invalidateShadowBake();
}

// Put the install's next pieces into the scene, for about budget_ms (at least one piece). The swap
// is a frame of its own, after the frame that made the last piece. True once the world is in.
bool put_world_step(SceneGraph &sg, const std::string &id, WorldRT &rt, double budget_ms) {
  using Stage = WorldInstall::Stage;
  WorldInstall &in = *rt.install;
  if (in.stage == Stage::swap) {
    swap_world(sg, id, rt);
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
      put_world_textures(sg, id, rt, in.job->texture, in.job->water);
      in.stage = Stage::chunks;
      break;
    case Stage::chunks:
      if (in.next < w.terrain_chunks.size()) {
        make_chunk(sg, id, rt, w, in.next++, in.suffix, in);
        break;
      }
      in.next = 0;
      in.stage = Stage::cells;
      continue; // not a piece
    case Stage::cells:
      if (in.next < w.prop_cells.size()) {
        in.cells.push_back(make_prop_cell(sg, id, w, in.next++, in.suffix, rt.soft));
        break;
      }
      in.next = 0;
      in.stage = Stage::buildings;
      continue;
    case Stage::buildings:
      rt.buildings = mesh_holder(sg, id + "_buildings", rt.buildings);
      if (auto h = rt.buildings.lock()) {
        const bool soft = rt.soft, arch = w.archetypes;
        in.building_mesh =
            make_mesh(*h, "mesh" + in.suffix, w.buildings,
                      [soft, arch](GeometryNode &g) { style_buildings(g, soft, arch); });
      }
      in.stage = Stage::roads;
      break;
    case Stage::roads:
      rt.roads = mesh_holder(sg, id + "_roads", rt.roads);
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

// A WorldJob as a sliced task: a rebuild steps a world_builder and then paints the new world's
// texture; a repaint only paints. Natively each step runs on the background pool, on
// single-threaded wasm a few milliseconds a frame on the scheduler thread; either way it writes
// only its job (a repaint reads the shown world, which nothing changes while it runs), and the
// tick takes the job in once `done` is set. The builder's own stages name the phase.
class WorldTask : public cvc::sliced_job {
public:
  WorldTask(std::shared_ptr<WorldJob> job, const world::generated_world_params &p,
            const world::building_library *library, std::shared_ptr<ari::ResolvedFile> file,
            int texture_size)
      : _job(std::move(job)), _file(std::move(file)), _size(texture_size) {
    if (_job->rebuild)
      _builder = std::make_unique<world::world_builder>(p, library);
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
      if (!_painter)
        _painter = std::make_unique<world::surface_painter>(
            *_job->world, world::surface_color(_job->color), _size);
      const bool ready = _painter->step(budget);
      const double from = _job->rebuild ? kBuildShare : 0.0, to = _job->rebuild ? kPaintEnd : 1.0;
      set("paint", from + (to - from) * _painter->progress());
      if (ready) {
        _job->texture = _painter->take();
        _job->water = _painter->take_water();
        finish();
      }
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

  std::shared_ptr<WorldJob> _job;
  std::shared_ptr<ari::ResolvedFile> _file;
  int _size;
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
    drop_world_install(*rt.sg, rt.id, rt);
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
    rp = ari::resolve_generated_world_file(p, rt.base_dir, *file);
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
      a, std::make_shared<WorldTask>(job, rp, rt.library, std::move(file), rt.texture_size), o,
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
    t->setTexture(surface_image(rt.texture_size, j->texture), false);
  share_ground_texture(rt);
  publish_stats(rt);
  end_world_job(rt, 100);
}

// How long a frame spends putting a new world into the scene: a piece at least (a ground chunk's
// ladder, a prop cell's), then more while this lasts.
constexpr double kInstallBudgetMs = 6.0;

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
  rt->bodies = n.body.present;
  rt->base_dir = n.base_dir;
  rt->color = color_from(n.props.str("color", "material"), 0);
  rt->texture_size = std::max(256, std::min(4096, int(n.props.num("texture", 2048.0))));
  rt->soft = n.props.str("look", "soft") != "classic";
  rt->show_water = n.props.num("water", 1.0) != 0.0;
  rt->library = &ari::generated_world_library(app, n, warnings);

  // world.*, or under the node's `keys:` prop (node_key) so two worlds keep separate panels.
  rt->k = ari::generated_world_keys(app, n, sg.getStatePrefix(), rt->base, rt->color);
  world::generated_world_params p0 = ari::read_generated_world_params(app, rt->k, rt->base);
  rt->preset = p0.preset;
  rt->color = int(ari::read_or<double>(app, rt->k.color, rt->color));
  rt->w = ari::first_generated_world(app, rt->k, n, p0, rt->library, warnings);
  rt->sig = ari::generated_world_signature(p0);
  out.colliders[n.id] = rt->colliders; // a body: on the node collides with these

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
    first->water = painter.take_water();
    begin_world_install(*rt, first);
    while (!put_world_step(sg, n.id, *rt, 1e9)) {
    }
    rt->install.reset();
  }

  SceneGraph *sgp = &sg;
  const std::string id = n.id;
  out.custom_ticks.push_back([rt, sgp, id](vtkRenderer *ren) {
    if (rt->terrain.expired())
      return;
    // The meshes and the water beside the node follow its visibility, so a `visible:` on the node
    // hides (and shows) the whole world, not only the ground it draws itself. Each follows a change
    // of the node's, as a child follows its parent's (SceneNode::propagateVisible).
    if (auto t = rt->terrain.lock()) {
      const bool shown = t->isVisibleInHierarchy();
      if (shown != rt->shown) {
        rt->shown = shown;
        for (const char *part : kWorldParts)
          if (auto g = sgp->getGraphics(id + part))
            g->setVisible(shown);
        hide_world_install(*rt); // a world still going in stays hidden under them
      }
    }
    cvc::app &app = *rt->app;
    rt->frame++;
    // Regenerate, Randomize and Stop, a new preset, and world.auto (poll_generated_world). A new
    // colour alone repaints the shown world at once, once no rebuild is running (a rebuild paints
    // in the colour asked when it started, and a later change is repainted after it).
    const ari::GeneratedWorldPoll poll =
        ari::poll_generated_world(app, rt->k, rt->base, rt->preset, rt->sig);
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
    auto show_progress = [&app, &rt](const char *phase, int percent) {
      ari::write<int>(app, rt->k.progress, percent);
      ari::write<std::string>(app, rt->k.phase,
                              phase ? std::string(phase) + " " + std::to_string(percent) + "%"
                                    : std::string());
    };
    if (rt->job && !rt->install) {
      show_progress(rt->job->phase.load(std::memory_order_relaxed), rt->job->progress.load());
      if (rt->job->done.load(std::memory_order_acquire))
        apply_world_job(*rt);
    }
    // A made world goes into the scene a few milliseconds a frame (put_world_step), then is swapped
    // in at once; its LOD is chosen below, before the frame draws it.
    // Once no world is going in, the nodes of the one swapped out (or of a stopped install) leave
    // the scene a few a frame, and that world is freed after them (retire_world_step).
    if (!rt->install && !rt->retired.empty())
      retire_world_step(*rt, kInstallBudgetMs);
    if (rt->install) {
      const bool in = put_world_step(*sgp, id, *rt, kInstallBudgetMs);
      const WorldInstall &wi = *rt->install;
      const double share = double(wi.done) / double(std::max<std::size_t>(1, wi.total));
      show_progress("install", int(100.0 * (WorldTask::kPaintEnd +
                                            (1.0 - WorldTask::kPaintEnd) * std::min(1.0, share))));
      if (in) {
        rt->install.reset();
        end_world_job(*rt, 100);
      }
    }
    // The props' and the ground's LOD for this frame's camera; a caster that changed rung casts a
    // different shadow.
    if (ren && ren->IsActiveCameraCreated()) {
      const cvc::lod::view_params view = make_view_params(ren, cvc::lod::preset_view(kLodPreset));
      int recast = select_prop_lod(*rt, view);
      const int ground = select_terrain_lod(*rt, view);
      if (auto t = rt->terrain.lock(); t && t->castsShadow())
        recast += ground;
      if (recast > 0)
        sgp->invalidateShadowBake();
    }
  });
  return gn;
}

} // namespace

void register_world_node_types() {
  register_scene_node_type("generated_world", realize_generated_world);
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
