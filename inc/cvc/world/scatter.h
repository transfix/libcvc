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

// scatter.h — parameterized, deterministic placement of L-system props.
//
// Placement is keyed on a stable Morton cell id (rng::cell_id), NOT a loop
// counter, so adding a prop or changing the count cannot move any other one
// (insertion stability — the core repair of the predecessor's single mt19937).
// Each prop is derived once, interpreted to geometry, and reduced to a small set
// of ground FOOTPRINTS (a trunk disc + a canopy disc for a tree; the projected
// box footprints for a building or rock) tagged with a surface class. The raster
// stamps footprints; a building's large footprint therefore becomes many small
// occupied cells -> many small discs downstream, never one bounding circle.

#ifndef CVC_WORLD_SCATTER_H
#define CVC_WORLD_SCATTER_H

#include <cstdint>
#include <cvc/world/heightfield.h>
#include <cvc/world/surface.h>
#include <string>
#include <vector>

namespace cvc {
namespace world {

// A ground-plane footprint: a disc or an oriented box, tagged with a class.
struct footprint {
  enum kind : std::uint8_t { disc, obox } k = disc;
  double cx = 0, cy = 0; // world centre (metres)
  double radius = 0;     // disc: radius
  double ax = 1, ay = 0; // obox: unit axis-0 in XY
  double hx = 0, hy = 0; // obox: half-extents along axis-0 and its perpendicular
  std::uint16_t klass = 0;
  bool occupied = false; // hard -> occupancy raster
};

struct placed_prop {
  double x = 0, y = 0, z = 0; // world position (z = terrain height)
  double scale = 1.0;
  double yaw_deg = 0.0;
  std::string recipe;
  std::vector<footprint> footprints;
};

struct species_weight {
  std::string recipe;
  double weight = 1.0;
};

struct scatter_params {
  std::vector<species_weight> trees;     // kind plant
  std::vector<species_weight> rocks;     // kind rock
  std::vector<species_weight> buildings; // kind building

  int tree_count = 40; // target counts (actual = round(sqrt(count))^2 minus skips)
  int rock_count = 8;
  int building_count = 4;

  double size_min = 0.7, size_max = 1.3; // per-instance scale multiplier
  int tree_gen = 6, rock_gen = 1, building_gen = 1;

  double min_ground_m = 0.3; // do not place where terrain height < this (water/shore)
  double jitter = 0.7;       // placement jitter as a fraction of the sub-grid cell

  // A sensible default mix (used when the vectors are left empty).
  static scatter_params defaults();
};

// Place props over [min_x,max_x] x [min_y,max_y]. Deterministic in `seed`.
std::vector<placed_prop> scatter(double min_x, double min_y, double max_x, double max_y,
                                 const scatter_params &sp, const heightfield &hf,
                                 const surface_registry &reg, std::uint64_t seed);

} // namespace world
} // namespace cvc

#endif // CVC_WORLD_SCATTER_H
