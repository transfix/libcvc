/*
  Copyright 2008-2012, 2026 The University of Texas at Austin

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

// Vendored from VolumeRover's ContourTiler (John Edwards, UT Austin; transfix/volrover
// src/ContourTiler at 1cf0fbd7) and ported onto libcvc's tiler kernel and cvc::planar
// (VOLROVER-LIBRARY-PORTS-ROADMAP.md §3).

//
// Ported: CGAL's natural_neighbor_coordinates_2 and linear_interpolation on a
// Delaunay_triangulation_2 became cvc::planar::delaunay and sibson_interpolate (P3a's A/B: the
// same weights to 1e-14). The legacy file also set up CGAL's QP solver and Sibson gradient fitting,
// which it never used; they are gone.

#include "interp.h"

#include "planar_bridge.h"
#include "print_utils.h"

#include <iterator>
#include <limits>
#include <list>
#include <vector>

namespace cvc::tiler {

Number_type interpolate(const Polygon_2 &P, const Point_3 &point) { return 0; }

//------------------------------------------------------------------------------
// interpolate
//
/// Gives each point in [points_begin, points_end) a z, in order: the natural-neighbour
/// interpolation of the z values of the polygon's vertices and of the points already given one.
//------------------------------------------------------------------------------
template <typename Poly_iter, typename Iter>
Number_type interpolate(Poly_iter poly_begin, Poly_iter poly_end, Iter points_begin,
                        Iter points_end, Number_type zmid) {
  const Number_type nan = std::numeric_limits<Number_type>::quiet_NaN();
  typedef typename iterator_traits<Iter>::value_type point_type;

  // One grid for every point involved.
  Bbox_2 box;
  for (Poly_iter it = poly_begin; it != poly_end; ++it)
    box += it->bbox();
  for (Iter it = points_begin; it != points_end; ++it)
    box += it->bbox();
  const planar::grid g = fit_grid(box);

  std::vector<planar::point> known;
  std::vector<double> known_z;
  for (Poly_iter it = poly_begin; it != poly_end; ++it) {
    known.push_back(to_planar(*it, g));
    known_z.push_back(it->z());
  }

  for (Iter it = points_begin; it != points_end; ++it) {
    point_type &p = *it;
    p.z() = nan;
  }

  for (Iter it = points_begin; it != points_end; ++it) {
    TILER_TRACE(logger, "interpolating point: " << pp(*it));
    point_type &p = *it;
    std::vector<planar::point> pts = known;
    std::vector<double> values = known_z;
    for (Iter o_it = points_begin; o_it != points_end; ++o_it) {
      if (*o_it != p && o_it->z() == o_it->z()) {
        pts.push_back(to_planar(*o_it, g));
        values.push_back(o_it->z());
      }
    }
    const planar::triangulation T = planar::delaunay(pts);
    p.z() = planar::sibson_interpolate(T, values, g.to_grid(planar::pointd(p.x(), p.y())));
  }
  return 0;
}

template Number_type interpolate(Polygon_2::Vertex_iterator poly_begin,
                                 Polygon_2::Vertex_iterator poly_end,
                                 list<Point_2>::iterator points_begin,
                                 list<Point_2>::iterator points_end, Number_type zmid);
template Number_type interpolate(list<Point_2>::iterator poly_begin,
                                 list<Point_2>::iterator poly_end,
                                 list<Point_2>::iterator points_begin,
                                 list<Point_2>::iterator points_end, Number_type zmid);

} // namespace cvc::tiler
