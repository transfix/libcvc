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

/*
  project_verts.h — move a set of points onto a triangulated surface.

  Joe R., 10/16/2008, for VolumeRover. Since 2026-11 it no longer uses CGAL: its K_neighbor_search
  is in CGAL's Spatial Searching package, which is GPL-3+. The nearest reference vertices now come
  from a nanoflann k-d tree (BSD-2-Clause, private to libcvc), and the closest point on a triangle
  is computed exactly. The old version clamped its edge parameter with max(1, min(0, r)), which is
  always 1, so a point beyond a triangle's edge went to that edge's end vertex.

  cvc::project_points_to_triangles is the function. project_verts::project is the original
  iterator interface, kept as a thin wrapper:

  {
    typedef boost::array<double,3> point_t;
    typedef boost::array<int,3>    triangle_t;
    std::vector<point_t> input_verts, reference_verts;
    std::vector<triangle_t> reference_tris;

    //... fill up containers

    project_verts::project(input_verts.begin(),
                           input_verts.end(),
                           reference_verts.begin(),
                           reference_verts.end(),
                           reference_tris.begin(),
                           reference_tris.end());
  }
*/

#ifndef __PROJECT_VERTS_H__
#define __PROJECT_VERTS_H__

#include <array>
#include <cstddef>
#include <vector>

namespace cvc {

// Moves each point in `points` to the closest point on the triangles of `ref_tris` (indices into
// `ref_verts`) that use one of the point's `k` nearest reference vertices.
//
// - This is the legacy VolumeRover heuristic. It is exact whenever the closest triangle uses one
//   of those vertices, which is always the case when k >= ref_verts.size(); otherwise it is an
//   approximation that is good on a reasonably uniform mesh. geometry::project uses libigl's exact
//   AABB tree instead when libcvc is built with libigl.
// - A point is left where it is when its nearest vertices belong to no triangle (free vertices),
//   when there are no reference vertices, or when one of its coordinates is not finite.
// - Degenerate (zero-area) triangles are measured by their edges.
// - Deterministic: the same input gives the same output. No state is kept between calls.
//
// Throws std::invalid_argument if a triangle index is outside ref_verts, or if k is 0.
void project_points_to_triangles(std::vector<std::array<double, 3>> &points,
                                 const std::vector<std::array<double, 3>> &ref_verts,
                                 const std::vector<std::array<unsigned int, 3>> &ref_tris,
                                 std::size_t k = 10);

} // namespace cvc

namespace project_verts {

// The original interface. Each vertex element provides operator[] for its three components, and
// each triangle element operator[] for its three vertex indices into the reference vertices.
template <class ForwardIterator_vertex, class InputIterator_vertex, class InputIterator_tri>
inline void
project(ForwardIterator_vertex input_verts_begin, ForwardIterator_vertex input_verts_end,
        InputIterator_vertex reference_verts_begin, InputIterator_vertex reference_verts_end,
        InputIterator_tri reference_tris_begin, InputIterator_tri reference_tris_end) {
  std::vector<std::array<double, 3>> points, ref_verts;
  std::vector<std::array<unsigned int, 3>> ref_tris;
  for (ForwardIterator_vertex i = input_verts_begin; i != input_verts_end; ++i)
    points.push_back({double((*i)[0]), double((*i)[1]), double((*i)[2])});
  for (InputIterator_vertex i = reference_verts_begin; i != reference_verts_end; ++i)
    ref_verts.push_back({double((*i)[0]), double((*i)[1]), double((*i)[2])});
  for (InputIterator_tri i = reference_tris_begin; i != reference_tris_end; ++i)
    ref_tris.push_back({(unsigned int)((*i)[0]), (unsigned int)((*i)[1]), (unsigned int)((*i)[2])});

  cvc::project_points_to_triangles(points, ref_verts, ref_tris);

  std::size_t n = 0;
  for (ForwardIterator_vertex i = input_verts_begin; i != input_verts_end; ++i, ++n) {
    (*i)[0] = points[n][0];
    (*i)[1] = points[n][1];
    (*i)[2] = points[n][2];
  }
}

} // namespace project_verts

#endif
