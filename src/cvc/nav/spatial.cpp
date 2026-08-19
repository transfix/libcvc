/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick.

  VolMagick is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  VolMagick is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// spatial.cpp — the fixed-radius neighbour query declared in grid_nav.h, backed
// by CGAL's Kd_tree (libcvc already links CGAL). Kept in its own translation
// unit so grid_nav.cpp stays dependency-free STL. The inexact
// Simple_cartesian<double> kernel is deliberate: no GMP/MPFR, and the query is
// a plain squared-distance comparison, so the neighbour set matches the naive
// dist^2 <= radius^2 test exactly.

#include <CGAL/Fuzzy_sphere.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Search_traits_2.h>
#include <CGAL/Search_traits_adapter.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/property_map.h>
#include <algorithm>
#include <cvc/nav/grid_nav.h>
#include <utility>
#include <vector>

namespace cvc {
namespace nav {

namespace {
using Kernel = CGAL::Simple_cartesian<double>;
using Point2 = Kernel::Point_2;
using PointIdx = std::pair<Point2, int>;
using TraitsBase = CGAL::Search_traits_2<Kernel>;
using Traits =
    CGAL::Search_traits_adapter<PointIdx, CGAL::First_of_pair_property_map<PointIdx>, TraitsBase>;
using Tree = CGAL::Kd_tree<Traits>;
using FuzzySphere = CGAL::Fuzzy_sphere<Traits>;
} // namespace

neighbor_csr neighbors_within_radius(const double *positions, int n, double radius) {
  neighbor_csr out;
  out.offsets.assign(n + 1, 0);
  if (n <= 0)
    return out;

  Tree tree;
  for (int i = 0; i < n; ++i)
    tree.insert(PointIdx(Point2(positions[2 * i], positions[2 * i + 1]), i));
  tree.build();

  // Query each point's fuzzy sphere with epsilon 0 == an exact radius query
  // (points with squared distance <= radius^2). Self is excluded; the hit
  // indices are sorted so the result is order-independent of the tree layout.
  std::vector<std::vector<int>> per(n);
  std::vector<PointIdx> hits;
  for (int i = 0; i < n; ++i) {
    hits.clear();
    tree.search(std::back_inserter(hits),
                FuzzySphere(Point2(positions[2 * i], positions[2 * i + 1]), radius, 0.0));
    std::vector<int> &v = per[i];
    v.reserve(hits.size());
    for (const PointIdx &h : hits)
      if (h.second != i)
        v.push_back(h.second);
    std::sort(v.begin(), v.end());
  }

  for (int i = 0; i < n; ++i)
    out.offsets[i + 1] = out.offsets[i] + static_cast<int>(per[i].size());
  out.indices.reserve(out.offsets[n]);
  for (int i = 0; i < n; ++i)
    out.indices.insert(out.indices.end(), per[i].begin(), per[i].end());
  return out;
}

} // namespace nav
} // namespace cvc
