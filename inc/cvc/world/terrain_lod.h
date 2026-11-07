/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
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

// cvc::world terrain LOD — a height_field cut into square chunks, each drawn as a ladder of meshes
// of the same surface at coarser sample strides: chunked LOD with skirts
// (LSYSTEM-LABORATORY-ROADMAP section 8.4, after [Ulrich 2002]).
//
//   const terrain_chunk_layout L = plan_terrain_chunks(hf);   // chunk size, rung count
//   for (int gr = 0; gr < L.chunk_rows; ++gr)
//     for (int gc = 0; gc < L.chunk_cols; ++gc)
//       chunks.push_back(make_terrain_chunk(hf, L, gr, gc));
//   // each frame: a rung per chunk from its error_m (cvc::lod::select_rung), then
//   constrain_terrain_rungs(rungs, L.chunk_rows, L.chunk_cols);  // neighbours within one rung
//
// Rung k keeps every 2^k-th sample of the chunk (its last row and column always), so its vertices
// are the full surface's own: the points, normals and UVs of hf's mesh at those samples, and
// rung 0 is hf's mesh exactly. Nothing is regrown or smoothed. A rung's error_m is the largest
// vertical distance from its triangles to the samples it skips. Two chunks meet on the same
// samples, so their rungs differ along a shared edge only by what each leaves out there; a skirt,
// a strip hung straight down from each edge a neighbour shares, deep enough for the neighbour's
// rung either side of this one, covers that gap. The world's own border has none.
//
// LOD is a render proxy only: physics, nav and picking read hf itself.

#ifndef CVC_WORLD_TERRAIN_LOD_H
#define CVC_WORLD_TERRAIN_LOD_H

#include <cvc/geometry/geometry.h>
#include <cvc/geometry/height_field.h>
#include <vector>

namespace cvc {
namespace world {

struct terrain_chunk_layout {
  int cells = 0;                      // cells per chunk side (a power of two)
  int rungs = 0;                      // rungs per chunk: strides 1, 2, ... cells / coarsest cells
  int chunk_rows = 0, chunk_cols = 0; // chunks down and across; the last ones may be narrower
};

// The layout for hf: chunks of min_cells cells a side, doubled until no more than max_per_side of
// them span it, and rungs down to coarsest_cells cells a side. An empty or one-cell layout for an
// empty hf.
terrain_chunk_layout plan_terrain_chunks(const cvc::height_field &hf, int max_per_side = 16,
                                         int min_cells = 64, int coarsest_cells = 4);

struct terrain_chunk {
  int row = 0, col = 0;                          // the chunk's place in the layout
  int r0 = 0, c0 = 0, r1 = 0, c1 = 0;            // its samples [r0, r1] x [c0, c1] of hf
  double min[3] = {0, 0, 0}, max[3] = {0, 0, 0}; // rung 0's bounds, skirts left out
  std::vector<cvc::geometry> rungs; // rung 0 the full surface; points, normals, uvs, tris
  std::vector<double> error_m;      // each rung's vertical error, non-decreasing; [0] == 0
  std::vector<double> skirt_m;      // each rung's skirt depth (0 for a chunk with no neighbour)
  std::vector<cvc::geometry> water; // each rung's water surface (make_water_rungs), or empty
};

// Chunk (row, col) of layout L over hf.
terrain_chunk make_terrain_chunk(const cvc::height_field &hf, const terrain_chunk_layout &L,
                                 int row, int col);

// The water over chunk `ch`, a surface per rung on that rung's samples: level at water_level_m over
// every cell with a corner under it, so the terrain, drawn with it, hides it past the shoreline,
// exactly where the ground rises out of the water whatever the rung. Each vertex is coloured by
// the water column under it (water_colour). All empty when no cell of the chunk is under water.
// The water as a surface rather than a volume: for uniform water a closed surface gives the same
// image (WATER-RENDERING-ROADMAP section 0), at a cost that falls with distance.
std::vector<cvc::geometry> make_water_rungs(const cvc::height_field &hf,
                                            const terrain_chunk_layout &L, const terrain_chunk &ch,
                                            double water_level_m);

// Water `depth_m` deep seen from above, as one colour: the bed's light absorbed per channel along
// the way down and back (Beer-Lambert, red first, then green, blue last) and the water's own
// scattered blue filling in, so the shallows are a sandy bed tint, then green, then deep blue.
cvc::geometry::color_t water_colour(double depth_m);

// Coarsen nothing and refine where needed so that no chunk's rung is more than one off a
// neighbour's (4-connected): what the skirts are deep enough for. `rung` is row-major,
// rows x cols. Each rung only ever goes down.
void constrain_terrain_rungs(std::vector<int> &rung, int rows, int cols);

} // namespace world
} // namespace cvc

#endif
