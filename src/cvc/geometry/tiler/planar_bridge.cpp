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

#include "planar_bridge.h"

namespace cvc::tiler {

planar::grid fit_grid(const Bbox_2 &box) {
  if (!(box.xmin() <= box.xmax()) || !(box.ymin() <= box.ymax()))
    return planar::grid::fit(0, 0, 0, 0);
  return planar::grid::fit(box.xmin(), box.ymin(), box.xmax(), box.ymax());
}

planar::point to_planar(const Point_2 &p, const planar::grid &g) { return g.snap(p.x(), p.y()); }

planar::ring to_planar(const Polygon_2 &p, const planar::grid &g) {
  planar::ring r;
  r.reserve(p.size());
  for (const Point_2 &q : p.container())
    r.push_back(to_planar(q, g));
  return r;
}

planar::polygon to_planar(const Polygon_with_holes_2 &p, const planar::grid &g) {
  planar::polygon out(to_planar(p.outer_boundary(), g));
  for (auto it = p.holes_begin(); it != p.holes_end(); ++it)
    out.holes.push_back(to_planar(*it, g));
  return out;
}

Point_2 from_planar(const planar::point &p, const planar::grid &g, Number_type z) {
  const planar::pointd w = g.to_world(p);
  return Point_2(w.x, w.y, z);
}

Polygon_2 from_planar(const planar::ring &r, const planar::grid &g, Number_type z) {
  Polygon_2 out;
  for (const planar::point &p : r)
    out.push_back(from_planar(p, g, z));
  return out;
}

Polygon_with_holes_2 from_planar(const planar::polygon &p, const planar::grid &g, Number_type z) {
  Polygon_with_holes_2 out(from_planar(p.outer, g, z));
  for (const planar::ring &h : p.holes)
    out.add_hole(from_planar(h, g, z));
  return out;
}

} // namespace cvc::tiler
