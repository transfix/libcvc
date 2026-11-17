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


/*! \file symmetric_difference.cpp
 * Computing the symmetric difference of two polygons with holes.
 *
 * Ported: CGAL::difference on Lazy_exact_nt<Gmpz> polygons became cvc::planar::subtract
 * (Clipper2 on int64, planar_bridge.h). Its output is normalised (outer rings counterclockwise,
 * holes clockwise) without collinear vertices; verify() still splits a ring that touches itself.
 */

#include <list>

#include "common.h"
#include "planar_bridge.h"
#include "print_utils.h"

namespace cvc::tiler {

namespace {

// P minus Q, at height z.
template <typename Out_iter>
void difference(const planar::polygon_set &P, const planar::polygon_set &Q,
                const planar::grid &g, Number_type z, Out_iter out)
{
  for (const planar::polygon &p : planar::subtract(P, Q))
    *out++ = from_planar(p, g, z);
}

} // namespace

// Split polygon at any point that there is a shared point.
template <typename Out_iter>
void split_at_coincident(const Polygon_2& p, Out_iter out)
{
  // Find all coincident points
  const int n = p.size();
  map<Point_2, int> point_set;
  set<int> coincident_set;
  for (int i = 0; i < n; ++i) {
    if (point_set.find(p[i]) != point_set.end()) {
      coincident_set.insert(i);
      coincident_set.insert(point_set[p[i]]);
    }
    point_set[p[i]] = i;
  }

  if (coincident_set.empty()) {
    *out++ = p;
    return;
  }

  // start at a knot and split into two
  int cur = *coincident_set.begin();
  const int beg = cur;
  Polygon_2 p1, p2;
  do {
    p1.push_back(p[cur]);
    cur = (cur+1)%n;
  } while (p[cur] != p[beg]);
  do {
    p2.push_back(p[cur]);
    cur = (cur+1)%n;
  } while (p[cur] != p[beg]);

  // recurse
  split_at_coincident(p1, out);
  split_at_coincident(p2, out);
}

void verify(list<Polygon_with_holes_2>& polygons)
{
  typedef list<Polygon_with_holes_2>::iterator Iter;
  Iter it = polygons.begin();
  while (it != polygons.end()) {
    if (it->outer_boundary().size() == 0) {
      it = polygons.erase(it);
    }
    else {
      // First fix any non-simple holes

      // Erase all original holes (keep all_holes as a copy)
      list<Polygon_2> all_holes(it->holes_begin(), it->holes_end());
      while (it->holes_begin() != it->holes_end()) {
	it->erase_hole(it->holes_begin());
      }

      // Now go through the holes, fix each and re-add result to polygon_with_holes.
      for (list<Polygon_2>::const_iterator hole_it = all_holes.begin(); hole_it != all_holes.end(); ++hole_it) {
	if (!hole_it->is_simple()) {
	  list<Polygon_2> temp;
	  split_at_coincident(*hole_it, back_inserter(temp));
	  for (list<Polygon_2>::const_iterator new_it = temp.begin(); new_it != temp.end(); ++new_it) {
	    it->add_hole(*new_it);
	  }
	}
	else {
	  it->add_hole(*hole_it);
	}
      }

      // Now fix the outer boundary if it is non-simple.  This code currently does not
      // support fixing the outer boundary if there are holes.
      if (!it->outer_boundary().is_simple()) {
	if (it->holes_begin() != it->holes_end()) {
	  throw runtime_error("fixing non-simple outer boundaries not yet supported when there are holes");
	}
	list<Polygon_2> temp;
	split_at_coincident(it->outer_boundary(), back_inserter(temp));
	it = polygons.erase(it);
	for (list<Polygon_2>::const_iterator new_it = temp.begin(); new_it != temp.end(); ++new_it) {
	  polygons.push_back(Polygon_with_holes_2(*new_it));
	}
      }
      else {
	++it;
      }
    }
  }
}

template <typename Out_iter>
void polygon_difference(const Polygon_2& P, const Polygon_2& Q, 
			Out_iter P_out, Out_iter Q_out) {
  TILER_TRACE(logger, "P = " << pp(P));
  TILER_TRACE(logger, "Q = " << pp(Q));

  Number_type z = P[0].z();
  const Polygon_2 both[] = {P, Q};
  const planar::grid g = fit_grid(both, both + 2);
  const planar::polygon_set bso_P{planar::polygon(to_planar(P, g))};
  const planar::polygon_set bso_Q{planar::polygon(to_planar(Q, g))};

  list<Polygon_with_holes_2> new_P, new_Q;
  // Compute the symmetric difference of P and Q.
  difference(bso_P, bso_Q, g, z, back_inserter(new_P));
  difference(bso_Q, bso_P, g, z, back_inserter(new_Q));

  // Split into new polygons if necessary
  verify(new_P);
  verify(new_Q);

  for (list<Polygon_with_holes_2>::const_iterator it = new_P.begin(); it != new_P.end(); ++it) {
    *P_out++ = *it;
    TILER_TRACE(logger, "new_P = " << pp(*it));
  }
  for (list<Polygon_with_holes_2>::const_iterator it = new_Q.begin(); it != new_Q.end(); ++it) {
    *Q_out++ = *it;
    TILER_TRACE(logger, "new_Q = " << pp(*it));
  }
}

template
void polygon_difference(const Polygon_2& P, const Polygon_2& Q, 
			back_insert_iterator<vector<Polygon_with_holes_2> > P_out, back_insert_iterator<vector<Polygon_with_holes_2> > Q_out);
template
void polygon_difference(const Polygon_2& P, const Polygon_2& Q, 
			back_insert_iterator<list<Polygon_with_holes_2> > P_out, back_insert_iterator<list<Polygon_with_holes_2> > Q_out);

template <typename Out_iter>
void polygon_difference(const Polygon_with_holes_2& P, const Polygon_with_holes_2& Q, 
			Out_iter P_out, Out_iter Q_out) {
  Number_type z = P.outer_boundary()[0].z();
  const planar::grid g = fit_grid(P.bbox() + Q.bbox());
  const planar::polygon_set bso_P{to_planar(P, g)};

  list<Polygon_with_holes_2> new_P, new_Q;
  difference(bso_P, planar::polygon_set{to_planar(Q, g)}, g, z, back_inserter(new_P));
  // one-sided: Q itself, at P's height (as the legacy converted it back)
  new_Q.push_back(from_planar(to_planar(Q, g), g, z));
  // This two-sided implementation makes big holes in areas of large overlap

  // Split into new polygons if necessary
  verify(new_P);
  verify(new_Q);

  for (list<Polygon_with_holes_2>::const_iterator it = new_P.begin(); it != new_P.end(); ++it) {
    *P_out++ = *it;
  }
  for (list<Polygon_with_holes_2>::const_iterator it = new_Q.begin(); it != new_Q.end(); ++it) {
    *Q_out++ = *it;
  }
}

template
void polygon_difference(const Polygon_with_holes_2& P, const Polygon_with_holes_2& Q, 
			back_insert_iterator<list<Polygon_with_holes_2> > P_out, back_insert_iterator<list<Polygon_with_holes_2> > Q_out);

} // namespace cvc::tiler
