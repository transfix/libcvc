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

/*! \file symmetric_intersection.cpp
 * Computing the symmetric intersection of two polygons with holes.
 *
 * Ported: CGAL's intersection on Lazy_exact_nt<Gmpz> polygons became cvc::planar::intersect
 * (Clipper2 on int64, planar_bridge.h).
 */

#include "common.h"
#include "planar_bridge.h"
#include "print_utils.h"

#include <list>

namespace cvc::tiler {

template <typename Out_iter>
void polygon_intersection(const Polygon_2 &P, const Polygon_2 &Q, Out_iter out) {
  Number_type z = P[0].z();
  const Polygon_2 both[] = {P, Q};
  const planar::grid g = fit_grid(both, both + 2);
  const planar::polygon_set bso_P{planar::polygon(to_planar(P, g))};
  const planar::polygon_set bso_Q{planar::polygon(to_planar(Q, g))};
  for (const planar::polygon &p : planar::intersect(bso_P, bso_Q))
    *out++ = from_planar(p, g, z);
}

template void polygon_intersection(const Polygon_2 &P, const Polygon_2 &Q,
                                   back_insert_iterator<vector<Polygon_with_holes_2>> out);
template void polygon_intersection(const Polygon_2 &P, const Polygon_2 &Q,
                                   back_insert_iterator<list<Polygon_with_holes_2>> out);

} // namespace cvc::tiler
