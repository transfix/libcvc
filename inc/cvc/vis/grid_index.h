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

// grid_index.h -- a flat uniform grid over two world axes, the broad-phase that
// turns the O(N) reference sweep into "skip whole cells the frustum misses."
//
// This is the acceleration structure RENDER_PERF phase 1 asks for by name -- a
// flat uniform grid (128 m tiles on Austin's span), deliberately NOT a tree at
// this N (VISIBILITY-AND-LOD-ROADMAP section 5.4 / 8.8). A quadtree that adds
// level-dependent cell size is a later PR; the flat grid is what the city cull
// needs first.
//
// Correctness is by construction, not by hope. Each cell owns the UNION AABB of
// the proxies binned to it. A cell whose union box is wholly outside the frustum
// cannot contain a visible proxy -- a box that is fully behind some plane keeps
// every sub-box fully behind it too -- so skipping the cell drops nothing. Every
// proxy in a surviving cell is then run through the SAME reference_visible() the
// oracle uses, so distance and layer culling are unchanged. The result is
// therefore EXACTLY the oracle's visible set, only faster. vis_grid_test asserts
// that equivalence over random scenes; the header states it because it is a
// proof, not a hope.

#ifndef __CVC_VIS_GRID_INDEX_H__
#define __CVC_VIS_GRID_INDEX_H__

#include <cstddef>
#include <cstdint>
#include <cvc/vis/reference.h>
#include <cvc/vis/scene_view.h>
#include <cvc/vis/types.h>
#include <cvc/vis/view_params.h>
#include <cvc/vis/visible_set.h>
#include <vector>

namespace cvc {
namespace vis {

class grid_index {
public:
  // Build the grid over the scene. `cell_size` is the tile edge in world units;
  // `axis0`/`axis1` are the two axes the 2-D grid spans (default X and Z, the
  // ground plane in Y-up scenes) -- the third axis rides along inside each
  // cell's union AABB, so the structure is orientation-agnostic. Proxies are
  // binned by AABB centre; a proxy overhanging its cell is still covered because
  // the cell's union AABB grows to contain its whole box.
  //
  // Rebuild is a counting sort: O(N) plus one pass, no per-proxy allocation
  // after the columns are sized. Re-callable every frame the topology changes.
  void build(const scene_view &s, float cell_size, int axis0 = 0, int axis1 = 2);

  // Cull via the grid, filling `out` (cleared first) with the surviving ids in
  // ascending order and their distance/screen payload. Equivalent to
  // reference_cull() on the same scene and view -- same set, same payload.
  void cull(const scene_view &s, const view_params &v, visible_set &out) const;

  // --- introspection (for tests, the debug overlay, and the benchmark) ---
  int nx() const noexcept { return nx_; }
  int ny() const noexcept { return ny_; } // cells along axis1
  std::size_t cell_count() const noexcept { return static_cast<std::size_t>(nx_) * ny_; }
  std::size_t occupied_cells() const noexcept;
  float cell_size() const noexcept { return cell_size_; }

  // The number of cells the last cull() skipped wholesale (frustum-outside).
  // Reset at the start of each cull(). The headline the broad-phase exists for.
  std::size_t last_cells_skipped() const noexcept { return cells_skipped_; }
  std::size_t last_cells_visited() const noexcept { return cells_visited_; }

private:
  int cell_of(const aabb &b, int &gx, int &gy) const noexcept;

  int nx_ = 0, ny_ = 0;
  int axis0_ = 0, axis1_ = 2;
  float cell_size_ = 1.0f;
  float origin_[2] = {0.0f, 0.0f}; // min corner on (axis0, axis1)

  // CSR: proxies[start[c] .. start[c+1]) are the ids in cell c (row-major
  // gy*nx_ + gx). cell_bounds[c] is their union AABB.
  std::vector<std::uint32_t> start_;
  std::vector<proxy_id> proxies_;
  std::vector<aabb> cell_bounds_;

  // Per-cull counters (mutable so cull() can stay const).
  mutable std::size_t cells_skipped_ = 0;
  mutable std::size_t cells_visited_ = 0;
};

} // namespace vis
} // namespace cvc

#endif // __CVC_VIS_GRID_INDEX_H__
