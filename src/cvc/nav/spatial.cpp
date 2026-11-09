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

// spatial.cpp — the fixed-radius neighbour query declared in grid_nav.h, on a nanoflann k-d tree
// (BSD-2-Clause, vendored in src/cvc/third_party/nanoflann). It used CGAL's Kd_tree, whose
// Spatial Searching package is GPL-3+, so libcvc no longer uses it; nanoflann builds everywhere
// libcvc does, wasm included, so there is no slower fallback any more.
//
// The contract is unchanged: self excluded, indices ascending, and a neighbour is any other point
// with dx*dx + dy*dy <= radius*radius, computed exactly as the brute-force test computes it.
// nanoflann's radius search keeps only dist < radius, so the query asks for a slightly larger
// radius and every hit is then tested against the exact inclusive bound here.

#include "../third_party/nanoflann/nanoflann.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cvc/nav/grid_nav.h>
#include <vector>

namespace cvc {
namespace nav {

namespace {

// nanoflann's dataset interface over the caller's row-major [n][2] array. It holds a pointer to the
// caller's data and nothing else.
struct point_cloud_2d {
  const double *pos;
  std::size_t n;
  std::size_t kdtree_get_point_count() const { return n; }
  double kdtree_get_pt(std::size_t i, std::size_t dim) const { return pos[2 * i + dim]; }
  template <class BBox> bool kdtree_get_bbox(BBox &) const { return false; }
};

using tree_2d =
    nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, point_cloud_2d>,
                                        point_cloud_2d, 2, std::uint32_t>;

} // namespace

neighbor_csr neighbors_within_radius(const double *positions, int n, double radius) {
  neighbor_csr out;
  out.offsets.assign(n + 1, 0);
  if (n <= 0)
    return out;

  const point_cloud_2d cloud{positions, static_cast<std::size_t>(n)};
  const tree_2d tree(2, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));

  const double r2 = radius * radius;
  // A little above r2, so a point exactly on the boundary is still returned by nanoflann's
  // exclusive test; the exact test below decides.
  const double query_r2 = r2 + std::max(r2, 1.0) * 1e-9;

  std::vector<std::vector<int>> per(n);
  std::vector<nanoflann::ResultItem<std::uint32_t, double>> hits;
  for (int i = 0; i < n; ++i) {
    const double xi = positions[2 * i], yi = positions[2 * i + 1];
    const double query[2] = {xi, yi};
    hits.clear();
    (void)tree.radiusSearch(query, query_r2, hits, nanoflann::SearchParameters(0.0f, false));
    std::vector<int> &v = per[i];
    v.reserve(hits.size());
    for (const auto &h : hits) {
      const int j = static_cast<int>(h.first);
      if (j == i)
        continue;
      const double dx = positions[2 * j] - xi, dy = positions[2 * j + 1] - yi;
      if (dx * dx + dy * dy <= r2)
        v.push_back(j);
    }
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
