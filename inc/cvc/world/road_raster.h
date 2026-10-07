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

// road_raster.h — burn the road network into the semantic surface raster.
//
// cvc::world::raster() samples a world_model onto a grid (class / risk / hard / occupancy /
// height). A procedural city's roads live in a separate graph (cvc::world::roads), so after
// rasterizing the world this stamps the roads IN: every grid cell whose centre lies on a road
// becomes the asphalt (drivable, low-risk) surface class, which is what a nav / material consumer
// reads — roads become real traversable ground rather than just a texture. The raster_out
// invariants are preserved (risk_raw and hard stay exactly the registry's values for the stamped
// class, hard stays a subset of occupancy), and the stamped cells are marked as grammar-paint
// (layer 1). GL-free.

#ifndef CVC_WORLD_ROAD_RASTER_H
#define CVC_WORLD_ROAD_RASTER_H

#include <cstddef>
#include <cvc/world/roads.h> // road_network
#include <string>

namespace cvc {
namespace world {

struct grid_spec;       // cvc/world/grid.h
struct raster_out;      // cvc/world/raster.h
class surface_registry; // cvc/world/surface.h

// Stamp road cells into `out` (already produced by raster() on the SAME grid `g`). A cell whose
// centre is within a road's half-width + `margin` becomes class `klass_name` (default "asphalt"),
// with its risk_raw / hard / occupancy re-derived from `reg` and its layer_owner set to
// grammar-paint. No-op (returns 0) if the class name is unknown to the registry or the raster size
// != g.count(). Returns the number of cells stamped.
std::size_t rasterize_roads(const road_network &net, const grid_spec &g,
                            const surface_registry &reg, raster_out &out, double margin = 0.0,
                            const std::string &klass_name = "asphalt");

} // namespace world
} // namespace cvc

#endif // CVC_WORLD_ROAD_RASTER_H
