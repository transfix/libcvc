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

// cvc::world::generated_world — a whole generated world, ready to draw: terrain_lab's world.
//
//   cvc::world::generated_world_params p;            // seed, size, counts, relief, water level
//   cvc::world::generated_world w = cvc::world::generate_world(p, &library);
//   w.terrain        // the ground as a cvc::height_field (UVs over the extent)
//   paint_surface(w) // its colour: material, roads and plazas baked into an RGB texture
//   w.props / w.foliage / w.needles / w.buildings / w.roads   // merged meshes, one per look
//   w.prop_cells, w.props_r1, w.props_r2                      // the props' render LOD, by cell
//   w.terrain_chunks                                          // the ground's render LOD, by chunk
//   w.water          // the sea, lakes and ponds, and the shore grid (cvc/world/water_bodies.h)
//   water_field(w, t, ...)                                    // the water, as a volume of depth
//   w.geometry()     // the world as cvc::world::world_geometry, for physics and nav
//
// The pieces: the ground from a terrain generator (make_terrain: "fbm", the analytic relief, or
// "dem", an elevation .npy, or one a host registered), world_model::generate (the L-system prop
// scatter on it), the city from a city generator ("districts", below, or "none"), and the props
// tessellated into merged meshes. The default city is roads first:
//
//   * district cores, about one per 600 m of world side, on dry, gentle ground: an urbanity field
//     that is 1 at a core and fades out over a few hundred metres;
//   * a street network grown from them (generate_roads): arterials crossing at each core,
//     collectors a block apart, local streets packed tighter in the cores and thinning outside,
//     steered along the contours to stay under a 35 % grade, kept out of the water, and trimmed of
//     dead-end stubs;
//   * lots along every street (layout_lots), reaching back to the middle of the block, left empty
//     on ground too steep or wet to build, with narrower setbacks and shared walls in the cores;
//   * a building per lot, filled from the cores out (whole street sides at a time) up to
//     `buildings`: the library's archetypes at their real size (the Austin archetypes,
//     load_building_library), or without one procedural massing whose heights rise with urbanity
//     (houses with gable roofs outside the cores, a rare tower in them).
//
// The roads are extruded into a surface and burned into the raster as asphalt, the buildings as
// occupied concrete wall, and trees and rocks keep off the roads and buildings. Water is everything
// below water_level_m, and water_field fills it with a gently travelling wave.
//
// GL-free and deterministic: a pure function of the params and the library. Every mesh is one
// merged cvc::geometry, so a renderer draws the world in a handful of nodes. world_builder and
// surface_painter make the same world and texture a step at a time, for a caller that must not
// block (a big world takes seconds on single-threaded wasm).
#ifndef CVC_WORLD_GENERATED_WORLD_H
#define CVC_WORLD_GENERATED_WORLD_H

#include <cstddef>
#include <cstdint>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/height_field.h>
#include <cvc/world/building_archetypes.h>
#include <cvc/world/grid.h>
#include <cvc/world/raster.h>
#include <cvc/world/roads.h>
#include <cvc/world/terrain_lod.h>
#include <cvc/world/terrain_source.h>
#include <cvc/world/water_bodies.h>
#include <cvc/world/world_geometry.h>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
namespace world {

// The presets and ontologies the lab offers, by index.
enum class world_preset : int { standard = 0, forest, urban, sparse, island };
const char *world_preset_name(int preset);        // "standard", "forest", ... ("standard" past)
const char *world_ontology_name(int ontology);    // "merged_default", ...
int world_preset_from_name(const std::string &n); // its index, or 0

// How the surface texture colours the ground. water is a debug view of generated_world::water
// (water_bodies_debug_rgb): each sea, lake and pond in its kind's hue, dry land in greys, and the
// shore distance in contours.
enum class surface_color : int { material = 0, risk, occupancy, height, water };

struct generated_world_params {
  std::uint64_t seed = 5;
  double half_m = 600.0; // the world spans [-half_m, half_m]^2 metres
  int trees = 500;
  int rocks = 90;
  int buildings = 110; // the most buildings the city builds, filled from the district cores out
  int tree_gen = 5;
  double amp_m = 90.0;
  // The water surface. Sea level is 0 m: every generator lays its ground so that 0 m is the
  // shoreline (world_sea_share of the relief below it, or the island's shelf), whatever the relief.
  // This raises or lowers the sea from there; below it is water (streams, ponds, coast).
  double water_level_m = 0.0;
  int preset = 0;           // world_preset; island also shapes the relief
  int ontology = 0;         // world_ontology_name
  double cell_m = 5.0;      // the terrain mesh's target cell size
  int max_terrain_n = 2049; // its samples per side are capped
  int raster_n = 385;       // the material raster's cells per side (capped by the terrain's)
  // The generators, by name (register_terrain_generator, register_city_generator). Terrain:
  // "fbm" (the analytic relief) or "dem" (an elevation .npy at terrain_file, anchored at 0 and
  // stretched to amp_m). City: "districts" (the roads-first city) or "none" (no streets or
  // buildings).
  std::string terrain = "fbm";
  std::string city = "districts";
  std::string terrain_file;
};

// A lane of the generated city's streets: the widest vehicle driven on them, the Humvee (2.18 m
// overall, cvc::physics::vehicle_desc::humvee), with 0.4 m clear either side. Every street is at
// least two lanes wide, so two Humvees pass.
constexpr double world_vehicle_width_m = 2.18;
constexpr double world_lane_m = world_vehicle_width_m + 2.0 * 0.4;

// How much of a generator's relief lies below sea level (0 m): its floor sits world_sea_share x
// amp_m under the sea, so a lower or higher relief never floods or drains the world.
constexpr double world_sea_share = 34.0 / 90.0;

// The counts and relief a preset starts from (the lab's quick-set): fills trees, rocks, buildings,
// amp_m, and water_level_m (0 m: the sea level every relief is laid against) for p.preset.
void apply_world_preset(generated_world_params &p);

// The world_model params for these (the relief scaled to the world size), the ground from
// make_terrain(p).
world_params to_world_params(const generated_world_params &p);

// ── generators, by name ──────────────────────────────────────────────────────────────────────────
// A terrain generator makes the ground for p over [-p.half_m, p.half_m]^2 with its relief about
// p.amp_m and 0 m its shoreline (its floor world_sea_share x p.amp_m below), so p.water_level_m
// means the same thing whatever made the relief. A generator's own masks (channel, cliff,
// buildable) are respected by the city.
using terrain_generator =
    std::function<std::shared_ptr<const terrain_source>(const generated_world_params &p)>;
// Add a generator, or replace the one of that name. Thread-safe.
void register_terrain_generator(const std::string &name, terrain_generator g);
// The names: the built-ins first ("fbm", "dem"), then in registration order.
std::vector<std::string> terrain_generator_names();
// The ground for p, from p.terrain's generator. Throws std::invalid_argument on an unknown name,
// and passes on the generator's own errors (a missing DEM file, say).
std::shared_ptr<const terrain_source> make_terrain(const generated_world_params &p);
// The analytic relief's params for p (the "fbm" generator), scaled to the world size.
heightfield_params fbm_terrain_params(const generated_world_params &p);

// A city generator lays out the streets; the lots and buildings along them are the same for all.
struct city_streets {
  road_network roads;
  road_density_fn urbanity; // in [0, 1] (1 = a core); empty = 1 everywhere
  std::size_t districts = 0;
};
struct city_context {
  const generated_world_params &params; // half_m already clamped
  const terrain_source &terrain;
  // True where no road may go: the water (and 1.5 m of shore), and the terrain's channel and
  // cliff masks where it has them.
  road_mask_fn blocked;
};
// The context lives only for the call (world_builder lays the city out a stage later): the
// streets it returns, their urbanity included, must not refer to it.
using city_generator = std::function<city_streets(const city_context &c)>;
void register_city_generator(const std::string &name, city_generator g);
// The names: the built-ins first ("districts", "none"), then in registration order.
std::vector<std::string> city_generator_names();

struct generated_world {
  generated_world_params params;
  std::shared_ptr<const world_model> model;
  grid_spec grid;
  raster_out raster; // the material / risk / occupancy / height raster, asphalt included

  std::shared_ptr<cvc::height_field> terrain; // the ground mesh (one sample per cell_m)
  cvc::geometry props;                        // trunks, branches, rocks: per-vertex colour
  cvc::geometry foliage;                      // leaves: per-vertex colour
  cvc::geometry needles;                      // the pines' needles: line segments, one colour

  // Render LOD for the props (prop_cells below): coarser copies of props + foliage + needles, of
  // the same full trees. Rung 1 draws each tree with three-sided tubes, none under 5 cm, and its
  // leaves as solids (cvc::lsys::tree_detail); rung 2 is each tree's trunk and a crown over its
  // full canopy, and a box per rock. Per-vertex colour.
  cvc::geometry props_r1, props_r2;

  // The props are laid down a ground cell at a time (prop_cell_m squares), so each cell is one
  // contiguous run of points and of triangles (lines, for the needles) in every mesh above. A
  // renderer draws a cell as a ladder: rung 0 = its props + foliage + needles, rung 1 = its
  // props_r1, rung 2 = its props_r2 (prop_rung_radius_m says where each starts). LOD is a render
  // proxy only: geometry() and every physics or nav path read rung 0.
  struct prop_span {
    std::size_t p0 = 0, p1 = 0; // points [p0, p1)
    std::size_t c0 = 0, c1 = 0; // triangles (lines in needles) [c0, c1)
    bool empty() const { return p1 == p0; }
  };
  struct prop_cell {
    double min[3] = {0, 0, 0}, max[3] = {0, 0, 0}; // rung 0's bounds
    prop_span props, foliage, needles, props_r1, props_r2;
  };
  std::vector<prop_cell> prop_cells; // the cells with props in them
  double prop_cell_m = 0.0;

  // Render LOD for the ground (cvc/world/terrain_lod.h): the terrain cut into chunks, each a
  // ladder of the same surface at coarser sample strides, skirted where chunks meet. Row-major,
  // terrain_layout.chunk_rows x chunk_cols. A render proxy too: everything else reads terrain.
  terrain_chunk_layout terrain_layout;
  std::vector<terrain_chunk> terrain_chunks;
  cvc::geometry buildings; // per-vertex colour (the archetypes' concrete, or the massing's)
  cvc::geometry roads;     // the extruded street surface
  // The streets' centrelines and widths that `roads` was extruded from. The surface texture paints
  // them at their own width (and the dirt verges along some of them), not from the raster's
  // asphalt cells, which on a large world are tens of metres wide.
  road_network streets;
  std::size_t building_count = 0;
  bool archetypes = false;   // the buildings came from a building library
  double road_length = 0.0;  // the street network's total length (metres)
  std::size_t districts = 0; // how many district cores the city grew from

  // The core lots' paved forecourts (lot centre, half extents along the frontage and depth, yaw),
  // painted into the surface texture.
  struct apron {
    double x = 0, y = 0, z = 0, hx = 0, hy = 0, yaw = 0;
  };
  std::vector<apron> aprons;

  double height_min = 0, height_max = 0; // the relief's range over the raster

  // The ground under each water column (water_resolution(w)^2 samples, row-major, x fastest), so
  // water_field, called every few frames, need not sample the terrain again.
  std::vector<float> water_ground;

  // The bodies of water (cvc/world/water_bodies.h): the sea, the lakes and the ponds below
  // water_level_m, each sample's depth, and the shore grid (distance, normal, slope) that waves and
  // floating bodies read. Built from the terrain mesh's own samples, the roads graded in, every
  // water_bodies_stride()-th one so it is at most 1025 a side: its shore is the mesh's.
  water_bodies water;

  // The ground height at (x, y): the terrain mesh's own surface.
  double height(double x, double y) const;
  // The world for physics and nav: the terrain (collide + navigate), the buildings (collide),
  // and the props and roads (render only).
  world_geometry geometry() const;
};

// One cell's run of a prop mesh (generated_world::prop_cell) as a mesh of its own: its points,
// colours and triangles or lines, the indices rebased.
cvc::geometry prop_span_mesh(const cvc::geometry &g, const generated_world::prop_span &s);

// Where a prop cell's coarser rungs start, for a renderer authoring its ladder: rung 1 at
// prop_rung_radius_m[0] and rung 2 at [1], in metres from the camera, for the reference view of
// cvc::lod::world_error_for_switch_radius (800 px at 2 px error). The vegetation bands of
// LSYSTEM-LABORATORY-ROADMAP section 8.4: the full plant to 260 m, a reduced one to 700 m.
constexpr double prop_rung_radius_m[2] = {260.0, 700.0};

// Generate. `library` may be null or empty: the city's buildings are then procedural massing.
// Throws std::invalid_argument on an unknown generator name. world_builder stepped to its end.
generated_world generate_world(const generated_world_params &p,
                               const building_library *library = nullptr);

// generate_world as a resumable job, for a caller that must not block: the generated_world node
// steps one on the app's scheduler (cvc/core/sliced_task.h), a few milliseconds a frame on
// single-threaded wasm. step(budget) does about `budget` units of work and returns true once the
// world is made; budget 0 runs to the end. A unit is one terrain sample (a raster cell, a terrain
// mesh or water sample, a terrain LOD chunk's sample, those a whole chunk at a time); the stages
// that are one call (the scatter, the streets, the city, the roads, the props, the water bodies)
// take a step each, whatever the budget. phase() names the stage the next step works on
// ("terrain", "scatter", "raster", "mesh", "materials", "streets", "grade", "city", "roads",
// "props", "water", "shore", "lod"), a static
// string. A throw (an unknown generator, a missing DEM) comes out of the step that hit it
// and every later one. The library must outlive the builder.
class world_builder {
public:
  explicit world_builder(const generated_world_params &p,
                         const building_library *library = nullptr);
  ~world_builder();
  world_builder(const world_builder &) = delete;
  world_builder &operator=(const world_builder &) = delete;

  bool step(std::size_t budget);
  bool done() const;
  const char *phase() const;
  double progress() const; // in [0, 1], monotone; 1 once done
  generated_world take();  // the world, once done (moved out; call once)

private:
  struct impl;
  std::unique_ptr<impl> d_;
};

// paint_surface as a resumable job: step(budget) paints about `budget` texels (whole rows) and
// returns true once the texture is done; budget 0 paints the rest. `w` must outlive the painter.
class surface_painter {
public:
  surface_painter(const generated_world &w, surface_color mode = surface_color::material,
                  int size = 2048);
  bool step(std::size_t budget);
  bool done() const { return next_ >= size_ && finished_; }
  double progress() const { return size_ > 0 ? double(next_) / size_ : 1.0; }
  int size() const { return size_; }
  std::vector<unsigned char> take(); // the RGB texels, once done (moved out)
  // The water's texture (paint_water, the same size), painted row by row with the ground's;
  // once done (moved out). In the water colour mode its water takes the water bodies' debug
  // colours too, so the bodies show where the water surfaces cover the ground.
  std::vector<unsigned char> take_water();

private:
  const generated_world &w_;
  surface_color mode_;
  int size_;
  int next_ = 0;
  bool finished_ = false;
  std::vector<unsigned char> img_;
  std::vector<unsigned char> water_;
};

// The ground's colour as a size x size RGB texture over the extent (row 0 = min y, matching the
// terrain's UVs). The raster is far coarser than the texture, so no texel shows its cell's square:
// material mode blends the cells around each texel at a noise-warped position, so classes meet
// along frayed, organic edges; grass is broken up with dirt and dry grass, and the core lots'
// forecourts are paved. The streets are painted from w.streets, not the raster's asphalt cells:
// each road at its own width whatever the world's size, and a verge of light dirt along some
// stretches (street_verge). Risk and height are interpolated; occupancy shows its cells.
std::vector<unsigned char> paint_surface(const generated_world &w,
                                         surface_color mode = surface_color::material,
                                         int size = 2048);

// The water's texture over the extent, size x size RGBA with paint_surface's rows and texels:
// where the terrain mesh lies under the water, water_colour of the column there, opaque; where it
// is dry, transparent (alpha 0), in the shallowest water's colour so a filtered edge fades into
// the shallows. make_water_rungs' surfaces drawn with it end at the mesh's own shoreline whatever
// their rung, since a renderer discards fully transparent fragments; a coarse rung's cell, drawn
// whole, would put lake over dry ground wherever its corners straddle the shore. The texels are
// as fine as paint_surface's (2048 a side: 5 m on a 10 km world, about the mesh's own cell).
std::vector<unsigned char> paint_water(const generated_world &w, int size = 2048);

// The dirt verge along the streets: how much of the light dirt (0..1) the ground at (x, y) takes,
// from_edge_m past a road's pavement, painted over texels texel_m wide. About half the road length
// has one, in stretches a few hundred metres long; its width wanders between 1 m and
// street_verge_max_m, and its edge frays into the grass. 0 past street_verge_max_m + 1 m and half
// a texel (or 0.4 m).
constexpr double street_verge_max_m = 3.5;
double street_verge(double x, double y, double from_edge_m, double texel_m);

// The water as a volume of depth: n x n x nz samples over [-half, half]^2 x [water_floor(w),
// water_top(w)], 1 where the travelling wave's surface is a few metres above both the sample and
// the ground, fading to 0 at them (row-major, x fastest, then y, then z). t in seconds.
std::vector<float> water_field(const generated_world &w, double t, int n, int nz = 22);
double water_floor(const generated_world &w);
double water_top(const generated_world &w);
// The water's samples per side for this world size (80..320, about one per 8 m).
int water_resolution(const generated_world &w);
// generated_world::water takes every this-many of the terrain mesh's n samples a side: the least
// stride that keeps it to 1025 a side.
int water_bodies_stride(int n);
// Its colour and opacity transfer functions at time t (flattened (x, r, g, b) / (x, a) points).
void water_transfer(double t, std::vector<double> &color, std::vector<double> &opacity);

} // namespace world
} // namespace cvc

#endif // CVC_WORLD_GENERATED_WORLD_H
