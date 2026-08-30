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

#include <algorithm>
#include <cmath>
#include <cvc/vis/mesh_tiler.h>
#include <limits>
#include <unordered_map>

namespace cvc {
namespace vis {

namespace {

constexpr std::size_t kMaxCells = 4u * 1024u * 1024u;

int clampi(int v, int lo, int hi) noexcept { return v < lo ? lo : (v > hi ? hi : v); }

} // namespace

mesh_tiling tile_triangles(const double *points, std::size_t npoints, const std::uint32_t *tris,
                           std::size_t ntris, float cell_size, int axis0, int axis1) {
  mesh_tiling out;
  out.axis0 = axis0;
  out.axis1 = axis1;
  out.cell_size = cell_size > 0.0f ? cell_size : 1.0f;
  if (points == nullptr || tris == nullptr || npoints == 0 || ntris == 0)
    return out;

  auto pt = [&](std::uint32_t v, int axis) {
    return points[3 * static_cast<std::size_t>(v) + axis];
  };
  auto valid = [&](const std::uint32_t *t) {
    return t[0] < npoints && t[1] < npoints && t[2] < npoints;
  };

  // 1. Centroid extent on the two grid axes (skipping malformed triangles).
  double lo0 = std::numeric_limits<double>::infinity(), hi0 = -lo0;
  double lo1 = lo0, hi1 = hi0;
  for (std::size_t i = 0; i < ntris; ++i) {
    const std::uint32_t *t = tris + 3 * i;
    if (!valid(t))
      continue;
    const double c0 = (pt(t[0], axis0) + pt(t[1], axis0) + pt(t[2], axis0)) / 3.0;
    const double c1 = (pt(t[0], axis1) + pt(t[1], axis1) + pt(t[2], axis1)) / 3.0;
    lo0 = std::min(lo0, c0);
    hi0 = std::max(hi0, c0);
    lo1 = std::min(lo1, c1);
    hi1 = std::max(hi1, c1);
  }
  if (!(lo0 <= hi0))
    return out; // no valid triangle

  // 2. Grid dims, coarsening if the naive dims blow the cell cap.
  const double o0 = lo0, o1 = lo1;
  auto dims = [&](float cs) {
    const int a = std::max(1, static_cast<int>(std::floor((hi0 - lo0) / cs)) + 1);
    const int b = std::max(1, static_cast<int>(std::floor((hi1 - lo1) / cs)) + 1);
    return std::pair<int, int>(a, b);
  };
  auto [ax, ay] = dims(out.cell_size);
  while (static_cast<std::size_t>(ax) * ay > kMaxCells) {
    out.cell_size *= 2.0f;
    std::tie(ax, ay) = dims(out.cell_size);
  }
  out.nx = ax;
  out.ny = ay;

  auto cell_of = [&](const std::uint32_t *t) {
    const double c0 = (pt(t[0], axis0) + pt(t[1], axis0) + pt(t[2], axis0)) / 3.0;
    const double c1 = (pt(t[0], axis1) + pt(t[1], axis1) + pt(t[2], axis1)) / 3.0;
    const int gx = clampi(static_cast<int>(std::floor((c0 - o0) / out.cell_size)), 0, out.nx - 1);
    const int gy = clampi(static_cast<int>(std::floor((c1 - o1) / out.cell_size)), 0, out.ny - 1);
    return static_cast<std::uint32_t>(gy * out.nx + gx);
  };

  // 3. Count triangles per cell (sparse: only touched cells). A dense array over
  // nx*ny cells would be wasteful for a sparse city; a hash keeps it O(occupied).
  std::unordered_map<std::uint32_t, std::uint32_t> counts;
  counts.reserve(ntris / 8 + 1);
  for (std::size_t i = 0; i < ntris; ++i) {
    const std::uint32_t *t = tris + 3 * i;
    if (valid(t))
      ++counts[cell_of(t)];
  }

  // 4. Occupied cells in row-major order, and a cell -> tile-index map.
  out.occupied.reserve(counts.size());
  for (const auto &kv : counts)
    out.occupied.push_back(kv.first);
  std::sort(out.occupied.begin(), out.occupied.end());

  std::unordered_map<std::uint32_t, std::uint32_t> tile_of;
  tile_of.reserve(out.occupied.size() * 2);
  for (std::uint32_t k = 0; k < out.occupied.size(); ++k)
    tile_of[out.occupied[k]] = k;

  // 5. CSR offsets from the counts.
  out.start.assign(out.occupied.size() + 1, 0);
  for (std::uint32_t k = 0; k < out.occupied.size(); ++k)
    out.start[k + 1] = out.start[k] + counts[out.occupied[k]];

  // 6. Scatter triangle ids into their tiles, and grow per-tile bounds.
  const float inf = std::numeric_limits<float>::infinity();
  out.tri_ids.resize(out.start.back());
  out.bounds.assign(out.occupied.size(), aabb{{inf, inf, inf}, {-inf, -inf, -inf}});
  std::vector<std::uint32_t> cursor(out.start.begin(), out.start.end() - 1);
  for (std::size_t i = 0; i < ntris; ++i) {
    const std::uint32_t *t = tris + 3 * i;
    if (!valid(t))
      continue;
    const std::uint32_t tile = tile_of[cell_of(t)];
    out.tri_ids[cursor[tile]++] = static_cast<std::uint32_t>(i);
    aabb &b = out.bounds[tile];
    for (int c = 0; c < 3; ++c) {
      const std::uint32_t v = t[c];
      for (int a = 0; a < 3; ++a) {
        const float p = static_cast<float>(points[3 * static_cast<std::size_t>(v) + a]);
        b.mn[a] = std::min(b.mn[a], p);
        b.mx[a] = std::max(b.mx[a], p);
      }
    }
  }

  return out;
}

} // namespace vis
} // namespace cvc
