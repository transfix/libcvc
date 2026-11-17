/*
  Copyright 2026 The University of Texas at Austin

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

// Where the vendored tiler calls cvc::planar (VOLROVER-LIBRARY-PORTS-ROADMAP.md §3): its
// Booleans, offsets, triangulations, convex partitions and natural-neighbour interpolation, in
// place of CGAL's GPL packages. The tiler's polygons are doubles; cvc::planar's are integers on a
// grid. Each call fits a grid to the polygons it is given (planar::grid::fit: a power-of-two unit
// and an origin on a multiple of it). tile_contours() snapped every contour vertex onto a
// power-of-two grid of the whole stack first, and a finer power-of-two grid holds those points
// exactly, so input vertices cross over unchanged; only constructed points (an intersection, an
// offset) round to the local grid, whose unit is about 2^-31 of the polygons' size.

#ifndef CVC_GEOMETRY_TILER_PLANAR_BRIDGE_H
#define CVC_GEOMETRY_TILER_PLANAR_BRIDGE_H

#include <cvc/geometry/planar.h>

#include "common.h"

namespace cvc::tiler {

namespace planar = ::cvc::planar;

// The grid for everything within the box (planar::grid::fit), with room left for offsets.
planar::grid fit_grid(const Bbox_2 &box);
template <class It> planar::grid fit_grid(It first, It last) {
  Bbox_2 b;
  for (It it = first; it != last; ++it)
    b += it->bbox();
  return fit_grid(b);
}

planar::point to_planar(const Point_2 &p, const planar::grid &g);
planar::ring to_planar(const Polygon_2 &p, const planar::grid &g);
planar::polygon to_planar(const Polygon_with_holes_2 &p, const planar::grid &g);

// Back to world coordinates, every point at height z with no id.
Point_2 from_planar(const planar::point &p, const planar::grid &g, Number_type z);
Polygon_2 from_planar(const planar::ring &r, const planar::grid &g, Number_type z);
Polygon_with_holes_2 from_planar(const planar::polygon &p, const planar::grid &g, Number_type z);

} // namespace cvc::tiler

#endif // CVC_GEOMETRY_TILER_PLANAR_BRIDGE_H
