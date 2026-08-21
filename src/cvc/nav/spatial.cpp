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

// spatial.cpp — the fixed-radius neighbour query declared in grid_nav.h. Two
// implementations, selected at compile time so the contract (self excluded, indices
// sorted ascending, exact dist^2 <= radius^2) is byte-identical either way:
//
//   * default — CGAL's Kd_tree (O(N log N) build + fast range queries), the fast
//     path used in every normal libcvc build. The inexact Simple_cartesian<double>
//     kernel is deliberate: header-only (no GMP/MPFR to link) and the query is a
//     plain squared-distance comparison, so the neighbour set matches the naive test.
//
//   * DISABLE_CGAL — a CGAL-free STL fallback (naive O(N^2)) with identical output.
//     Without this, DISABLE_CGAL was a misnomer: this TU still #include'd CGAL and
//     only compiled because the build happened to have the headers on its path. The
//     fallback lets a genuinely CGAL-free build (e.g. the lean wasm toolchain, or any
//     -DDISABLE_CGAL build) drop the CGAL headers entirely.

#include <cvc/nav/grid_nav.h>
#include <vector>

#ifndef DISABLE_CGAL

#include <CGAL/Fuzzy_sphere.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Search_traits_2.h>
#include <CGAL/Search_traits_adapter.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/property_map.h>
#include <algorithm>
#include <utility>

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

#else // DISABLE_CGAL — CGAL-free STL fallback (byte-identical output)

namespace cvc {
namespace nav {

neighbor_csr neighbors_within_radius(const double *positions, int n, double radius) {
  neighbor_csr out;
  out.offsets.assign(n + 1, 0);
  if (n <= 0)
    return out;

  const double r2 = radius * radius;
  // Naive fixed-radius search. Scanning j in increasing order leaves each point's
  // neighbours already sorted ascending, and the exact `dx*dx + dy*dy <= r2` test with
  // self excluded matches the Kd_tree path bit-for-bit. O(N^2) — the CGAL Kd_tree is
  // the fast path; this keeps a CGAL-free build correct.
  std::vector<std::vector<int>> per(n);
  for (int i = 0; i < n; ++i) {
    const double xi = positions[2 * i], yi = positions[2 * i + 1];
    std::vector<int> &v = per[i];
    for (int j = 0; j < n; ++j) {
      if (j == i)
        continue;
      const double dx = positions[2 * j] - xi, dy = positions[2 * j + 1] - yi;
      if (dx * dx + dy * dy <= r2)
        v.push_back(j);
    }
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

#endif // DISABLE_CGAL
