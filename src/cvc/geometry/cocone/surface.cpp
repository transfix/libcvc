/*
  Copyright 2007-2008, 2026 The University of Texas at Austin

        Authors: Samrat Goswami <samrat@ices.utexas.edu> (TightCocone's op.cpp)
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

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

// extract_surface (tight_cocone_stages.h): TightCocone's wt_to_geometry (op.cpp:306-397).

#include "tight_cocone_stages.h"

namespace cvc::cocone {

step_task extract_surface(const Triangulation &triang,
                          const std::vector<std::size_t> &input_of_point, geometry &surface,
                          std::vector<std::size_t> &point_index, surface_counts &counts) {
  const tet_complex &tc = triang.complex();
  const std::size_t n = tc.points().size();
  std::vector<std::array<int, 3>> tris;
  std::vector<int> used(n, -1);

  // The flagged facets, each oriented by its inside cell: (a, b, c) when the inside cell's fourth
  // vertex is on the negative side of (a, b, c), so the triangle faces away from it, else (b, a, c).
  for (FFI fit = triang.finite_facets_begin(); fit != triang.finite_facets_end(); ++fit) {
    SK_UNIT;
    Cell_handle c[2] = {(*fit).first, (*fit).first->neighbor((*fit).second)};
    const int id[2] = {(*fit).second, c[1]->index(c[0])};
    if (!c[0]->cocone_flag(id[0]))
      continue;
    if (c[0]->outside == c[1]->outside)
      ++counts.one_sided;
    Vertex_handle vh[3] = {c[0]->vertex((id[0] + 1) % 4), c[0]->vertex((id[0] + 2) % 4),
                           c[0]->vertex((id[0] + 3) % 4)};
    // The legacy reference: the first cell's fourth vertex when that cell is inside, else the
    // second's.
    const int k = c[0]->outside ? 1 : 0;
    Vertex_handle ref = c[k]->vertex(id[k]);
    bool keep_order;
    if (!triang.is_infinite(ref)) {
      keep_order = cg::orientation(vh[0]->point(), vh[1]->point(), vh[2]->point(), ref->point()) ==
                   cg::NEGATIVE;
    } else {
      // Both cells outside, the second the infinite one (the legacy read the infinite vertex's
      // unset point): face away from infinity, towards the first cell's fourth vertex.
      keep_order = cg::orientation(vh[0]->point(), vh[1]->point(), vh[2]->point(),
                               c[0]->vertex(id[0])->point()) == cg::POSITIVE;
    }
    std::array<int, 3> t = {vh[0]->tc, vh[1]->tc, vh[2]->tc};
    if (!keep_order)
      std::swap(t[0], t[1]);
    for (int v : t)
      used[std::size_t(v)] = 0;
    tris.push_back(t);
  }

  // The points the surface uses, in point order.
  geometry::points_t &pts = surface.points();
  point_index.clear();
  for (std::size_t i = 0; i < n; ++i) {
    if (i % 64 == 63)
      SK_UNIT;
    if (used[i] < 0)
      continue;
    used[i] = int(pts.size());
    const tet_complex::point &p = tc.points()[i];
    pts.push_back(geometry::point_t{{p[0], p[1], p[2]}});
    point_index.push_back(input_of_point.empty() ? i : input_of_point[i]);
  }
  geometry::tris_t &out = surface.tris();
  out.reserve(tris.size());
  for (std::size_t i = 0; i < tris.size(); ++i) {
    if (i % 64 == 63)
      SK_UNIT;
    const std::array<int, 3> &t = tris[i];
    out.push_back(geometry::tri_t{{geometry::index_t(used[std::size_t(t[0])]),
                                   geometry::index_t(used[std::size_t(t[1])]),
                                   geometry::index_t(used[std::size_t(t[2])])}});
  }

  counts.inside_cells = 0;
  for (FCI cit = triang.finite_cells_begin(); cit != triang.finite_cells_end(); ++cit) {
    if (std::size_t(cit->id) % 64 == 63)
      SK_UNIT;
    if (!cit->outside)
      ++counts.inside_cells;
  }
}

} // namespace cvc::cocone
