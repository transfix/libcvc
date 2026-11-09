/*
  Copyright 2008 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolumeRover.

  VolumeRover is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  VolumeRover is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// project_verts.cpp — cvc::project_points_to_triangles (project_verts.h). The k nearest reference
// vertices come from a nanoflann k-d tree (BSD-2-Clause, vendored in src/cvc/third_party), and each
// candidate triangle's closest point is computed exactly (Ericson, "Real-Time Collision Detection",
// §5.1.5). Everything lives in locals: no state survives a call.

#include "../third_party/nanoflann/nanoflann.hpp"

#include <algorithm>
#include <cmath>
#include <cvc/geometry/project_verts.h>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>

namespace cvc {

namespace {

using vec3 = std::array<double, 3>;

vec3 sub(const vec3 &a, const vec3 &b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
vec3 add_scaled(const vec3 &a, const vec3 &d, double s) {
  return {a[0] + d[0] * s, a[1] + d[1] * s, a[2] + d[2] * s};
}
double dot(const vec3 &a, const vec3 &b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
double dist2(const vec3 &a, const vec3 &b) {
  const vec3 d = sub(a, b);
  return dot(d, d);
}
bool finite(const vec3 &p) {
  return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}

vec3 closest_on_segment(const vec3 &p, const vec3 &a, const vec3 &b) {
  const vec3 ab = sub(b, a);
  const double len2 = dot(ab, ab);
  if (!(len2 > 0.0))
    return a;
  const double t = std::clamp(dot(sub(p, a), ab) / len2, 0.0, 1.0);
  return add_scaled(a, ab, t);
}

vec3 closest_on_degenerate(const vec3 &p, const vec3 &a, const vec3 &b, const vec3 &c) {
  const vec3 q[3] = {closest_on_segment(p, a, b), closest_on_segment(p, b, c),
                     closest_on_segment(p, c, a)};
  int best = 0;
  for (int i = 1; i < 3; ++i)
    if (dist2(p, q[i]) < dist2(p, q[best]))
      best = i;
  return q[best];
}

// The closest point to p on triangle abc, by the Voronoi region p falls in.
vec3 closest_on_triangle(const vec3 &p, const vec3 &a, const vec3 &b, const vec3 &c) {
  const vec3 ab = sub(b, a), ac = sub(c, a);
  const vec3 n = {ab[1] * ac[2] - ab[2] * ac[1], ab[2] * ac[0] - ab[0] * ac[2],
                  ab[0] * ac[1] - ab[1] * ac[0]};
  // Zero area (or not finite): the divisions below would be 0/0, so measure the edges.
  if (!(dot(n, n) > 1e-24 * dot(ab, ab) * dot(ac, ac)))
    return closest_on_degenerate(p, a, b, c);

  const vec3 ap = sub(p, a);
  const double d1 = dot(ab, ap), d2 = dot(ac, ap);
  if (d1 <= 0.0 && d2 <= 0.0)
    return a;
  const vec3 bp = sub(p, b);
  const double d3 = dot(ab, bp), d4 = dot(ac, bp);
  if (d3 >= 0.0 && d4 <= d3)
    return b;
  const double vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
    return add_scaled(a, ab, d1 / (d1 - d3));
  const vec3 cp = sub(p, c);
  const double d5 = dot(ab, cp), d6 = dot(ac, cp);
  if (d6 >= 0.0 && d5 <= d6)
    return c;
  const double vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
    return add_scaled(a, ac, d2 / (d2 - d6));
  const double va = d3 * d6 - d5 * d4;
  if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0)
    return add_scaled(b, sub(c, b), (d4 - d3) / ((d4 - d3) + (d5 - d6)));
  const double denom = 1.0 / (va + vb + vc);
  return add_scaled(add_scaled(a, ab, vb * denom), ac, vc * denom);
}

// nanoflann's dataset interface over the reference vertices.
struct vertex_cloud {
  const std::vector<vec3> &v;
  std::size_t kdtree_get_point_count() const { return v.size(); }
  double kdtree_get_pt(std::size_t i, std::size_t dim) const { return v[i][dim]; }
  template <class BBox> bool kdtree_get_bbox(BBox &) const { return false; }
};

using vertex_tree =
    nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, vertex_cloud>,
                                        vertex_cloud, 3, std::size_t>;

} // namespace

void project_points_to_triangles(std::vector<std::array<double, 3>> &points,
                                 const std::vector<std::array<double, 3>> &ref_verts,
                                 const std::vector<std::array<unsigned int, 3>> &ref_tris,
                                 std::size_t k) {
  if (k == 0)
    throw std::invalid_argument("project_points_to_triangles: k must be at least 1");
  const std::size_t nv = ref_verts.size();
  for (std::size_t t = 0; t < ref_tris.size(); ++t)
    for (unsigned int idx : ref_tris[t])
      if (idx >= nv)
        throw std::invalid_argument("project_points_to_triangles: triangle " + std::to_string(t) +
                                    " uses vertex " + std::to_string(idx) + ", but there are " +
                                    std::to_string(nv) + " reference vertices");
  if (nv == 0 || points.empty() || ref_tris.empty())
    return;

  // Each reference vertex's triangles, in CSR form, in ascending triangle order.
  std::vector<std::size_t> start(nv + 1, 0);
  for (const auto &tri : ref_tris)
    for (unsigned int idx : tri)
      ++start[idx + 1];
  std::partial_sum(start.begin(), start.end(), start.begin());
  std::vector<std::size_t> tris_of(start[nv]);
  std::vector<std::size_t> fill(start.begin(), start.end() - 1);
  for (std::size_t t = 0; t < ref_tris.size(); ++t)
    for (unsigned int idx : ref_tris[t])
      tris_of[fill[idx]++] = t;

  const vertex_cloud cloud{ref_verts};
  const vertex_tree tree(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));

  const std::size_t kk = std::min(k, nv);
  std::vector<std::size_t> nearest(kk);
  std::vector<double> nearest_d2(kk);
  std::vector<std::size_t> candidates;
  for (vec3 &p : points) {
    if (!finite(p))
      continue;
    const std::size_t found = tree.knnSearch(p.data(), kk, nearest.data(), nearest_d2.data());
    candidates.clear();
    for (std::size_t f = 0; f < found; ++f)
      for (std::size_t s = start[nearest[f]]; s < start[nearest[f] + 1]; ++s)
        candidates.push_back(tris_of[s]);
    if (candidates.empty())
      continue; // free vertices only: leave the point where it is
    // Ascending and unique, so ties go to the lowest triangle index whatever order the tree
    // returned the vertices in.
    std::sort(candidates.begin(), candidates.end());
    candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());

    double best = std::numeric_limits<double>::infinity();
    vec3 best_point = p;
    for (std::size_t t : candidates) {
      const auto &tri = ref_tris[t];
      const vec3 q =
          closest_on_triangle(p, ref_verts[tri[0]], ref_verts[tri[1]], ref_verts[tri[2]]);
      const double d = dist2(p, q);
      if (d < best) {
        best = d;
        best_point = q;
      }
    }
    p = best_point;
  }
}

} // namespace cvc
