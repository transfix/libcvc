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

// A scalar field on a simplicial mesh: the input of the mesh contour spectrum
// (cvc/volume/contour_spectrum.h) and the mesh contour tree (cvc/volume/contour_tree.h), and of
// cvc::tf's overlays on a mesh (a FEM solution, a curvature or a geodesic distance on a surface).
//
// One value per point of `mesh`, interpolated linearly over each element: the mesh's tetrahedra
// when it has any (a solid; its triangles, a boundary, add nothing), else its triangles (a
// surface). Quads, hexahedra and lines are ignored. Shared and immutable, so a resumable builder
// can hold it across steps while its owner moves on to a new mesh or field.

#ifndef CVC_VOLUME_CONTOUR_MESH_H
#define CVC_VOLUME_CONTOUR_MESH_H

#include <memory>
#include <vector>

namespace cvc {
class geometry;

struct contour_mesh {
  std::shared_ptr<const geometry> mesh;
  std::vector<double> values; // one per mesh point
};

} // namespace cvc

#endif // CVC_VOLUME_CONTOUR_MESH_H
