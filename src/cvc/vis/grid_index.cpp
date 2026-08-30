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
#include <cvc/vis/grid_index.h>
#include <limits>

namespace cvc {
namespace vis {

namespace {

// Cap the grid so a pathological cell_size (tiny) or extent (huge) cannot ask
// for a billion cells. 4M cells is ~64 MB of CSR headers, well past any real
// city tiling; beyond it, the cell size is silently coarsened by build().
constexpr std::size_t kMaxCells = 4u * 1024u * 1024u;

int clampi(int v, int lo, int hi) noexcept { return v < lo ? lo : (v > hi ? hi : v); }

void expand(aabb &acc, const aabb &b) noexcept {
  for (int k = 0; k < 3; ++k) {
    acc.mn[k] = std::min(acc.mn[k], b.mn[k]);
    acc.mx[k] = std::max(acc.mx[k], b.mx[k]);
  }
}

aabb empty_aabb() noexcept {
  const float inf = std::numeric_limits<float>::infinity();
  return {{inf, inf, inf}, {-inf, -inf, -inf}};
}

} // namespace

int grid_index::cell_of(const aabb &b, int &gx, int &gy) const noexcept {
  const float c0 = 0.5f * (b.mn[axis0_] + b.mx[axis0_]);
  const float c1 = 0.5f * (b.mn[axis1_] + b.mx[axis1_]);
  gx = clampi(static_cast<int>(std::floor((c0 - origin_[0]) / cell_size_)), 0, nx_ - 1);
  gy = clampi(static_cast<int>(std::floor((c1 - origin_[1]) / cell_size_)), 0, ny_ - 1);
  return gy * nx_ + gx;
}

void grid_index::build(const scene_view &s, float cell_size, int axis0, int axis1) {
  axis0_ = axis0;
  axis1_ = axis1;
  cell_size_ = cell_size > 0.0f ? cell_size : 1.0f;

  start_.clear();
  proxies_.clear();
  cell_bounds_.clear();

  if (s.count == 0 || s.bounds == nullptr) {
    nx_ = ny_ = 0;
    return;
  }

  // 1. Extent of the proxy centres on the two grid axes.
  float lo0 = std::numeric_limits<float>::infinity(), hi0 = -lo0;
  float lo1 = lo0, hi1 = hi0;
  for (proxy_id i = 0; i < s.count; ++i) {
    const aabb &b = s.bounds[i];
    const float c0 = 0.5f * (b.mn[axis0_] + b.mx[axis0_]);
    const float c1 = 0.5f * (b.mn[axis1_] + b.mx[axis1_]);
    lo0 = std::min(lo0, c0);
    hi0 = std::max(hi0, c0);
    lo1 = std::min(lo1, c1);
    hi1 = std::max(hi1, c1);
  }
  origin_[0] = lo0;
  origin_[1] = lo1;

  // 2. Grid dimensions, coarsening the cell size if the naive dims blow the cap.
  auto dims = [&](float cs) {
    const int a = std::max(1, static_cast<int>(std::floor((hi0 - lo0) / cs)) + 1);
    const int b = std::max(1, static_cast<int>(std::floor((hi1 - lo1) / cs)) + 1);
    return std::pair<int, int>(a, b);
  };
  auto [ax, ay] = dims(cell_size_);
  while (static_cast<std::size_t>(ax) * ay > kMaxCells) {
    cell_size_ *= 2.0f;
    std::tie(ax, ay) = dims(cell_size_);
  }
  nx_ = ax;
  ny_ = ay;
  const std::size_t ncells = static_cast<std::size_t>(nx_) * ny_;

  // 3. Counting sort into CSR. Pass 1: per-cell counts.
  start_.assign(ncells + 1, 0);
  std::vector<int> gx(s.count), gy(s.count);
  std::vector<std::uint32_t> cell(s.count);
  for (proxy_id i = 0; i < s.count; ++i) {
    cell[i] = static_cast<std::uint32_t>(cell_of(s.bounds[i], gx[i], gy[i]));
    ++start_[cell[i] + 1];
  }
  for (std::size_t c = 0; c < ncells; ++c)
    start_[c + 1] += start_[c];

  // Pass 2: scatter ids into their cells, and accumulate per-cell union AABBs.
  proxies_.resize(s.count);
  cell_bounds_.assign(ncells, empty_aabb());
  std::vector<std::uint32_t> cursor(start_.begin(), start_.end() - 1);
  for (proxy_id i = 0; i < s.count; ++i) {
    const std::uint32_t c = cell[i];
    proxies_[cursor[c]++] = i;
    expand(cell_bounds_[c], s.bounds[i]);
  }
}

std::size_t grid_index::occupied_cells() const noexcept {
  std::size_t n = 0;
  for (std::size_t c = 0; c + 1 < start_.size(); ++c)
    if (start_[c + 1] > start_[c])
      ++n;
  return n;
}

void grid_index::cull(const scene_view &s, const view_params &v, visible_set &out) const {
  out.clear();
  cells_skipped_ = 0;
  cells_visited_ = 0;
  if (start_.empty())
    return;

  const std::size_t ncells = static_cast<std::size_t>(nx_) * ny_;
  out.reserve(s.count / 4 + 1);

  // Collect survivors, then sort by id so the output matches the oracle's
  // ascending order (grid iteration is cell-order). Payload rides along.
  struct hit {
    proxy_id id;
    float dist;
    float px;
  };
  std::vector<hit> hits;
  hits.reserve(s.count / 4 + 1);

  for (std::size_t c = 0; c < ncells; ++c) {
    const std::uint32_t b = start_[c], e = start_[c + 1];
    if (b == e)
      continue; // empty cell -- not even visited

    // Broad phase: a cell whose union box is wholly outside the frustum cannot
    // hold a visible proxy. This is the whole point of the structure.
    if (aabb_vs_frustum(cell_bounds_[c], v.f) == frustum_test::outside) {
      ++cells_skipped_;
      continue;
    }
    ++cells_visited_;

    // Narrow phase: the exact per-proxy test, identical to the oracle's, so
    // distance/layer culling is preserved and the result set is exact.
    for (std::uint32_t k = b; k < e; ++k) {
      const proxy_id id = proxies_[k];
      float dist = 0.0f, px = 0.0f;
      if (reference_visible(s, v, id, &dist, &px))
        hits.push_back({id, dist, px});
    }
  }

  std::sort(hits.begin(), hits.end(), [](const hit &a, const hit &b) { return a.id < b.id; });
  for (const hit &h : hits)
    out.push(h.id, h.dist, h.px);
}

} // namespace vis
} // namespace cvc
