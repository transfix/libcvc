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

// water_bodies.h — which body of water each point of a world is in, its still-water level and
// depth, and the shore grid that waves, wakes and floating bodies read
// (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md section 3, W0.1 and W0.2).
//
//   cvc::world::water_bodies_params p;            // the water level, the pond threshold
//   cvc::world::water_bodies wb = cvc::world::build_water_bodies(ground, grid, p);
//   wb.body[i]       // which body sample i is in (an index into wb.bodies), -1 on dry land
//   wb.depth[i]      // its level minus the ground: > 0 under water, <= 0 on dry land
//   wb.shore_m[i]    // signed distance to the shore: + offshore, - inland
//   wb.normal_x/y[i] // the unit shore normal, pointing offshore
//   wb.bodies[k]     // sea, lake or pond; its level, area, bounds, deepest point and fetch
//
// The wet samples are those whose ground lies below the water level, in 8-connected runs. Every
// run that touches the grid's edge is part of the one sea: the water goes on past the world, where
// the runs meet. Each enclosed run is a body of its own, a lake, or a pond when its area is under
// pond_max_area_m2. Bodies are numbered in the order their first sample comes in the grid (row 0 =
// min_y first, x fastest), so there is at most one sea.
//
// One level for every body until hydrology fills each depression to its own spill level (the
// water roadmap's W1). The planes are written for that day: level[] is per sample, and a stream
// kind is reserved for the accumulation network's reaches.
//
// The shore distance is the exact Euclidean distance transform of the wet and the dry samples
// (cvc::algo::edt2_squared_both, the one cvc::nav's clearance fields use), taken half a sample
// short so the shore sits between a wet sample and its dry neighbour: it is within half a sample
// of the true waterline. The grid's cells must be square.
//
// Standard library only, and deterministic: a pure function of the ground, the grid and the params,
// the same at any thread count. Row 0 is min_y, as everywhere in cvc::world.

#ifndef CVC_WORLD_WATER_BODIES_H
#define CVC_WORLD_WATER_BODIES_H

#include <cstddef>
#include <cstdint>
#include <cvc/world/grid.h>
#include <string>
#include <vector>

namespace cvc {
namespace world {

class terrain_source;

enum class water_kind : std::uint8_t {
  sea = 0,   // every run of water touching the grid's edge: open water past the world
  lake = 1,  // enclosed, at least pond_max_area_m2
  pond = 2,  // enclosed and smaller
  stream = 3 // a reach of the drainage network (reserved: nothing makes one before hydrology)
};
const char *water_kind_name(water_kind k); // "sea", "lake", "pond", "stream"

struct water_body {
  water_kind kind = water_kind::sea;
  double level_m = 0.0;                                      // the still-water surface
  std::size_t cells = 0;                                     // wet samples
  double area_m2 = 0.0;                                      // cells x the cell's area
  double min_x = 0.0, min_y = 0.0, max_x = 0.0, max_y = 0.0; // the wet samples' bounds
  double max_depth_m = 0.0;
  // The body's largest open-water extent, for a fetch-limited wind sea: the major axis of the
  // ellipse with the wet samples' second moments (a W x L rectangle gives about 1.15 L), and at
  // least a sample. The sea's means little (it is cut off by the grid, and its runs may lie apart);
  // its waves use the scene's open-ocean fetch.
  double fetch_m = 0.0;
};

struct water_bodies_params {
  double level_m = 0.0;            // the still-water level of every body (until hydrology)
  double pond_max_area_m2 = 1.0e4; // an enclosed body smaller than this is a pond (1 ha: D7)
  // The shore normal is smoothed over this radius, about one swell wavelength, so crests do not
  // twist where the distance field folds (its medial axis). 0: unsmoothed.
  double normal_smooth_m = 0.0;
};

struct water_bodies {
  grid_spec grid; // the samples' frame
  water_bodies_params params;

  // One value per sample, row-major, rows x cols.
  std::vector<float> ground;      // the ground (m); NaN where there is none (counts as dry)
  std::vector<std::int32_t> body; // the body index, -1 on dry land
  std::vector<float> level;       // the still-water level here (dry: the nearest body's)
  std::vector<float> depth;       // level - ground: > 0 under water, <= 0 on dry land, NaN off
  std::vector<float> shore_m;     // signed distance to the shore (m): + offshore, - inland,
                                  // at most the grid's diagonal either way (no shore at all)
  std::vector<float> normal_x;    // the unit shore normal, offshore (0, 0 where it is undefined)
  std::vector<float> normal_y;
  std::vector<float> slope; // the ground's slope, rise over run (tan of its angle)

  std::vector<water_body> bodies;

  std::size_t index(int r, int c) const noexcept { return std::size_t(r) * grid.cols + c; }
  bool empty() const noexcept { return ground.empty(); }

  // The body at (x, y): the nearest sample's, -1 on dry land or off the grid.
  int body_at(double x, double y) const;
  // A plane above, bilinear at (x, y), clamped to the grid; NaN when a corner is.
  double sample(const std::vector<float> &plane, double x, double y) const;
  // How many bodies are of kind k.
  std::size_t count(water_kind k) const;
  // The wet samples, over all bodies.
  std::size_t wet_cells() const;
};

// The bodies of water over `grid` from its ground samples (grid.count() of them, row-major, row 0 =
// min_y). Throws std::invalid_argument when the count is wrong, the grid is under 2 x 2, or its
// cells are not square.
water_bodies build_water_bodies(std::vector<float> ground, const grid_spec &grid,
                                const water_bodies_params &p = {});

// The same, the ground sampled from a terrain source at the grid's samples.
water_bodies build_water_bodies(const terrain_source &t, const grid_spec &grid,
                                const water_bodies_params &p = {});

// The bodies in a line, for a panel: "1 sea, 2 lakes, 5 ponds", or "no water".
std::string water_bodies_summary(const water_bodies &wb);

// A debug colour at (x, y) (RGB, 0..255), interpolated between the samples: dry land in greys by
// height, each body in its kind's hue (sea blue, lake teal, pond green, stream cyan) with its own
// shade, darkening with depth, and a light contour every `band_m` metres of shore distance on
// either side of the waterline.
void water_bodies_debug_rgb(const water_bodies &wb, double x, double y, double band_m,
                            unsigned char rgb[3]);

} // namespace world
} // namespace cvc

#endif // CVC_WORLD_WATER_BODIES_H
