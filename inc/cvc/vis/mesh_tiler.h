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

// mesh_tiler.h -- split one merged mesh into spatial tiles the culler can drop
// or keep as a unit.
//
// RENDER_PERF phase 1 in one sentence: the Austin buildings arrive as a single
// 978,242-triangle actor, and VTK draws all of it every pass even when the
// camera sees one district. Tiling assigns each triangle to a ground-plane cell
// by its centroid and reports, per cell, the triangle list and the union AABB.
// The renderer then builds one sub-mesh per non-empty tile and culls the tiles
// (cvc::vis::grid_index / the frustum test) instead of the whole city.
//
// This is deliberately geometry-library-agnostic: it takes raw point and index
// arrays (like the cvc::nav kernels), so it is testable with plain vectors and
// carries no cvc::geometry / VTK dependency. The renderer adapter converts a
// cvc::geometry to these arrays and back.
//
// A triangle is placed whole into the cell its CENTROID falls in -- never split
// across a boundary -- so the tiles partition the triangle set exactly (every
// triangle in one tile, none duplicated). A triangle overhanging its cell is
// still culled correctly because the tile's union AABB grows to contain it, the
// same conservative bound grid_index relies on.

#ifndef __CVC_VIS_MESH_TILER_H__
#define __CVC_VIS_MESH_TILER_H__

#include <cstddef>
#include <cstdint>
#include <cvc/vis/types.h>
#include <vector>

namespace cvc {
namespace vis {

// The result of tiling a mesh. `occupied` lists the non-empty tiles in
// row-major (gy*nx + gx) order; for occupied tile k, its triangles are
// tri_ids[start[k] .. start[k+1]) and its world bounds are bounds[k]. Only
// occupied tiles are represented, so a mostly-empty grid costs nothing per empty
// cell.
struct mesh_tiling {
  int nx = 0, ny = 0;
  float cell_size = 0.0f;
  int axis0 = 0, axis1 = 1;

  std::vector<std::uint32_t> occupied; // row-major cell id of each tile
  std::vector<std::uint32_t> start;    // CSR offsets, size occupied.size()+1
  std::vector<std::uint32_t> tri_ids;  // triangle indices, grouped by tile
  std::vector<aabb> bounds;            // per-tile union AABB, size occupied.size()

  std::size_t tile_count() const noexcept { return occupied.size(); }
  std::uint32_t tris_in(std::size_t tile) const noexcept { return start[tile + 1] - start[tile]; }
};

// Tile a mesh. `points` is `npoints` xyz triples (3 doubles each); `tris` is
// `ntris` index triples into those points; `cell_size` is the tile edge in world
// units; `axis0`/`axis1` are the two ground axes the grid spans (default X,Y --
// Austin's buildings stand along +Z). Triangles with an out-of-range vertex
// index are skipped (a malformed mesh loses a triangle, not the whole tiling).
mesh_tiling tile_triangles(const double *points, std::size_t npoints, const std::uint32_t *tris,
                           std::size_t ntris, float cell_size, int axis0 = 0, int axis1 = 1);

} // namespace vis
} // namespace cvc

#endif // __CVC_VIS_MESH_TILER_H__
