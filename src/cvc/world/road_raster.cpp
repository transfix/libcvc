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

// road_raster.cpp — see road_raster.h. GL-free; no RNG (a pure function of the network + grid).

#include <cstddef>
#include <cstdint>
#include <cvc/world/grid.h>
#include <cvc/world/raster.h>
#include <cvc/world/road_raster.h>
#include <cvc/world/roads.h>
#include <cvc/world/surface.h>
#include <string>

namespace cvc {
namespace world {

std::size_t rasterize_roads(const road_network &net, const grid_spec &g,
                            const surface_registry &reg, raster_out &out, double margin,
                            const std::string &klass_name) {
  std::uint16_t id = 0;
  if (!reg.find(klass_name, id))
    return 0; // unknown class -> leave the raster untouched
  const std::size_t n = g.count();
  if (out.klass.size() != n || out.risk_raw.size() != n || out.hard.size() != n ||
      out.occupancy.size() != n)
    return 0; // raster was not built on this grid

  const surface_class &sc = reg[id];
  const float rho = sc.rho;
  const std::uint8_t hard = sc.hard ? 1 : 0;
  const bool has_layer = out.layer_owner.size() == n;

  std::size_t stamped = 0;
  for (int r = 0; r < g.rows; ++r) {
    const double y = g.world_y(r);
    for (int c = 0; c < g.cols; ++c) {
      if (!net.on_or_near_road(g.world_x(c), y, margin))
        continue;
      const std::size_t i = std::size_t(r) * g.cols + c;
      out.klass[i] = id;
      out.risk_raw[i] = rho;   // invariant: risk_raw == registry[klass].rho
      out.hard[i] = hard;      // invariant: hard == registry[klass].hard
      out.occupancy[i] = hard; // asphalt is drivable; keeps hard ⊆ occupancy
      if (has_layer)
        out.layer_owner[i] = 1; // authored (grammar-paint) layer
      ++stamped;
    }
  }
  return stamped;
}

} // namespace world
} // namespace cvc
