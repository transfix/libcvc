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
//   water_field(w, t, ...)                                    // the water, as a volume of depth
//   w.geometry()     // the world as cvc::world::world_geometry, for physics and nav
//
// The pieces: world_model::generate (the relief and the L-system prop scatter), the props
// tessellated into merged meshes, and the city. With a building library (the Austin archetypes,
// load_building_library) the city is roads first: a terrain-following street network
// (generate_roads), lots along it (layout_lots), the archetypes fitted into the lots, the roads
// extruded into a surface and burned into the raster as asphalt. Without one it is box massing on
// the scatter's building sites, with streets painted between their plazas. Water is everything
// below water_level_m: roads and lots stop at the shore, and water_field fills it with a gently
// travelling wave.
//
// GL-free and deterministic: a pure function of the params and the library. Every mesh is one
// merged cvc::geometry, so a renderer draws the world in a handful of nodes.
#ifndef CVC_WORLD_GENERATED_WORLD_H
#define CVC_WORLD_GENERATED_WORLD_H

#include <cstdint>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/height_field.h>
#include <cvc/world/building_archetypes.h>
#include <cvc/world/grid.h>
#include <cvc/world/raster.h>
#include <cvc/world/world_geometry.h>
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

// How the surface texture colours the ground.
enum class surface_color : int { material = 0, risk, occupancy, height };

struct generated_world_params {
  std::uint64_t seed = 7;
  double half_m = 600.0; // the world spans [-half_m, half_m]^2 metres
  int trees = 500;
  int rocks = 90;
  int buildings = 110; // building sites for box massing (the roads-first city ignores it)
  int tree_gen = 5;
  double amp_m = 90.0;
  // The water surface. The terrain floor sits at 0 and rises to about amp_m; below this line is
  // water (streams, ponds, coast).
  double water_level_m = 34.0;
  int preset = 0;           // world_preset; island also shapes the relief
  int ontology = 0;         // world_ontology_name
  double cell_m = 5.0;      // the terrain mesh's target cell size
  int max_terrain_n = 2049; // its samples per side are capped
  int raster_n = 385;       // the material raster's cells per side (capped by the terrain's)
};

// The counts and relief a preset starts from (the lab's quick-set): fills trees, rocks, buildings
// and amp_m for p.preset.
void apply_world_preset(generated_world_params &p);

// The world_model params for these (the relief scaled to the world size).
world_params to_world_params(const generated_world_params &p);

struct generated_world {
  generated_world_params params;
  std::shared_ptr<const world_model> model;
  grid_spec grid;
  raster_out raster; // the material / risk / occupancy / height raster, asphalt included

  std::shared_ptr<cvc::height_field> terrain; // the ground mesh (one sample per cell_m)
  cvc::geometry props;                        // trunks, branches, rocks: per-vertex colour
  cvc::geometry foliage;                      // leaves: per-vertex colour
  cvc::geometry needles;                      // the pines' needles: line segments, one colour
  cvc::geometry buildings; // per-vertex colour (the archetypes' concrete, or the massing's)
  cvc::geometry roads;     // the extruded street surface (empty for box massing)
  std::size_t building_count = 0;
  bool archetypes = false; // the city came from a building library (roads first)

  // Box massing's plazas and the streets between them, painted into the surface texture.
  struct apron {
    double x = 0, y = 0, z = 0, hx = 0, hy = 0, yaw = 0;
  };
  struct street {
    double x0, y0, x1, y1;
  };
  std::vector<apron> aprons;
  std::vector<street> streets;

  double height_min = 0, height_max = 0; // the relief's range over the raster

  // The ground height at (x, y): the terrain mesh's own surface.
  double height(double x, double y) const;
  // The world for physics and nav: the terrain (collide + navigate), the buildings (collide),
  // and the props and roads (render only).
  world_geometry geometry() const;
};

// Generate. `library` may be null or empty: the city is then box massing.
generated_world generate_world(const generated_world_params &p,
                               const building_library *library = nullptr);

// The ground's colour as a size x size RGB texture over the extent (row 0 = min y, matching the
// terrain's UVs). The raster is far coarser than the texture, so no texel shows its cell's square:
// material mode blends the cells around each texel at a noise-warped position, so classes meet
// along frayed, organic edges (asphalt excepted, which stays straight); grass is broken up with
// dirt and dry grass, and box massing's streets and plazas are painted on. Risk and height are
// interpolated; occupancy shows its cells.
std::vector<unsigned char> paint_surface(const generated_world &w,
                                         surface_color mode = surface_color::material,
                                         int size = 2048);

// The water as a volume of depth: n x n x nz samples over [-half, half]^2 x [water_floor(w),
// water_top(w)], 1 where the travelling wave's surface is a few metres above both the sample and
// the ground, fading to 0 at them (row-major, x fastest, then y, then z). t in seconds.
std::vector<float> water_field(const generated_world &w, double t, int n, int nz = 22);
double water_floor(const generated_world &w);
double water_top(const generated_world &w);
// The water's samples per side for this world size (80..320, about one per 8 m).
int water_resolution(const generated_world &w);
// Its colour and opacity transfer functions at time t (flattened (x, r, g, b) / (x, a) points).
void water_transfer(double t, std::vector<double> &color, std::vector<double> &opacity);

} // namespace world
} // namespace cvc

#endif // CVC_WORLD_GENERATED_WORLD_H
