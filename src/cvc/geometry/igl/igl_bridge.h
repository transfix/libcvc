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

// igl_bridge.h -- INTERNAL, cvc side of the libigl seam (not installed): packs a
// cvc::geometry into the arrays igl_detail.h takes. Included by mesh_ops.cpp
// and fem.cpp only; the igl_*.cpp TUs must never include it (it pulls in
// cvc::geometry, Boost and config.h).

#ifndef CVC_GEOMETRY_IGL_BRIDGE_H
#define CVC_GEOMETRY_IGL_BRIDGE_H

#include "igl_detail.h"

#include <climits>
#include <cstddef>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/mesh_ops.h>
#include <vector>

namespace cvc {
namespace igl_bridge {

// 3 doubles per point, from const_points(). Throws mesh_ops_error past INT_MAX
// points.
std::vector<double> positions(const geometry &g);

// tris(), then each quad (a,b,c,d) as (a,b,c),(a,c,d): 3 ints per triangle.
// Throws mesh_ops_error on an index >= num_points() or past INT_MAX triangles.
std::vector<int> surface_triangles(const geometry &g);

// tets(), 4 ints per tet, checked like surface_triangles().
std::vector<int> tetrahedra(const geometry &g);

// Bounding-box diagonal of packed positions (0 when there are none).
double diagonal(const std::vector<double> &V);

// The largest element count for the operations that assemble finite-element
// matrices: libigl does int arithmetic on multiples of the element count
// (massmatrix sizes 16 entries per tet and 9 per triangle, grad 12 per
// triangle), which overflows long before INT_MAX elements.
const std::size_t kMaxFemElements = std::size_t(INT_MAX) / 16;
// Throws mesh_ops_error past kMaxFemElements.
void check_fem_count(std::size_t elements, const char *what);

// Runs fn, turning a failure inside the libigl code into cvc::mesh_ops_error.
template <class Fn> auto guarded(Fn &&fn) -> decltype(fn()) {
  try {
    return fn();
  } catch (const igl_detail::failure &e) {
    throw mesh_ops_error(e.what());
  }
}

} // namespace igl_bridge
} // namespace cvc

#endif // CVC_GEOMETRY_IGL_BRIDGE_H
