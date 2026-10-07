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

#include <algorithm>
#include <cmath>
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

  const double cw = g.cell_w(), ch = g.cell_h();
  auto set_cell = [&](std::size_t i) {
    out.klass[i] = id;
    out.risk_raw[i] = rho;   // invariant: risk_raw == registry[klass].rho
    out.hard[i] = hard;      // invariant: hard == registry[klass].hard
    out.occupancy[i] = hard; // asphalt is drivable; keeps hard ⊆ occupancy
    if (has_layer)
      out.layer_owner[i] = 1; // authored (grammar-paint) layer
  };

  // Rasterize per SEGMENT into its local cell window rather than testing every cell against every
  // segment (which is O(cells * segments) and dominates on a large grid): for each segment, walk
  // the cells inside its bounding box expanded by the half-width + margin and stamp those within
  // that distance of the segment. O(sum of segment footprints). A cell shared by two segments is
  // counted once (stamped only on the first visit).
  std::size_t stamped = 0;
  for (const road_segment &s : net.segments) {
    if (s.a < 0 || s.b < 0 || s.a >= int(net.nodes.size()) || s.b >= int(net.nodes.size()))
      continue;
    const double ax = net.nodes[s.a].x, ay = net.nodes[s.a].y;
    const double bx = net.nodes[s.b].x, by = net.nodes[s.b].y;
    const double reach = 0.5 * s.width + margin; // half carriageway + margin
    if (reach <= 0.0)
      continue;
    const double minx = std::min(ax, bx) - reach, maxx = std::max(ax, bx) + reach;
    const double miny = std::min(ay, by) - reach, maxy = std::max(ay, by) + reach;
    int c0 = int(std::floor((minx - g.min_x) / cw)), c1 = int(std::ceil((maxx - g.min_x) / cw));
    int r0 = int(std::floor((miny - g.min_y) / ch)), r1 = int(std::ceil((maxy - g.min_y) / ch));
    c0 = std::max(0, c0);
    r0 = std::max(0, r0);
    c1 = std::min(g.cols - 1, c1);
    r1 = std::min(g.rows - 1, r1);
    const double dx = bx - ax, dy = by - ay, len2 = dx * dx + dy * dy;
    const double reach2 = reach * reach;
    for (int r = r0; r <= r1; ++r) {
      const double y = g.world_y(r);
      for (int c = c0; c <= c1; ++c) {
        const double x = g.world_x(c);
        // squared distance from (x,y) to segment a->b
        double t = len2 > 0.0 ? ((x - ax) * dx + (y - ay) * dy) / len2 : 0.0;
        t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
        const double px = ax + t * dx - x, py = ay + t * dy - y;
        if (px * px + py * py > reach2)
          continue;
        const std::size_t i = std::size_t(r) * g.cols + c;
        if (out.klass[i] != id)
          ++stamped; // count each road cell once
        set_cell(i);
      }
    }
  }
  return stamped;
}

} // namespace world
} // namespace cvc
