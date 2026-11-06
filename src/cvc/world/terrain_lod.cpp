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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cvc/world/terrain_lod.h>
#include <limits>

namespace cvc {
namespace world {

namespace {

// The samples a rung keeps along [a, b]: every s-th from a, and b.
std::vector<int> stridedSamples(int a, int b, int s) {
  std::vector<int> v;
  for (int i = a; i < b; i += s)
    v.push_back(i);
  v.push_back(b);
  return v;
}

// The interval of `v` holding i: v[k] <= i <= v[k + 1].
int intervalOf(const std::vector<int> &v, int i) {
  const auto it = std::upper_bound(v.begin(), v.end(), i);
  return std::max(0, std::min(int(it - v.begin()) - 1, int(v.size()) - 2));
}

enum side { kSouth, kNorth, kWest, kEast };

} // namespace

terrain_chunk_layout plan_terrain_chunks(const cvc::height_field &hf, int max_per_side,
                                         int min_cells, int coarsest_cells) {
  terrain_chunk_layout L;
  const int cr = hf.cell_rows(), cc = hf.cell_cols();
  if (cr <= 0 || cc <= 0)
    return L;
  int cells = 1;
  while (cells < std::max(1, min_cells))
    cells *= 2;
  const int span = std::max(cr, cc);
  while ((span + cells - 1) / cells > std::max(1, max_per_side))
    cells *= 2;
  L.cells = cells;
  L.rungs = 1;
  for (int s = 2; cells / s >= std::max(1, coarsest_cells); s *= 2)
    ++L.rungs;
  L.chunk_rows = (cr + cells - 1) / cells;
  L.chunk_cols = (cc + cells - 1) / cells;
  return L;
}

terrain_chunk make_terrain_chunk(const cvc::height_field &hf, const terrain_chunk_layout &L,
                                 int row, int col) {
  terrain_chunk ch;
  ch.row = row;
  ch.col = col;
  ch.r0 = row * L.cells;
  ch.c0 = col * L.cells;
  ch.r1 = std::min(hf.cell_rows(), ch.r0 + L.cells);
  ch.c1 = std::min(hf.cell_cols(), ch.c0 + L.cells);
  const std::vector<float> &h = hf.heights();
  const int cols = hf.cols();
  auto H = [&](int r, int c) { return double(h[std::size_t(r) * cols + c]); };
  const bool shares[4] = {row > 0, row + 1 < L.chunk_rows, col > 0, col + 1 < L.chunk_cols};

  // Each rung's samples, and its error: over the whole chunk, and along each edge it shares.
  const int K = std::max(1, L.rungs);
  std::vector<std::vector<int>> R(K), C(K);
  std::vector<double> err(K, 0.0);
  std::vector<double> edge(K, 0.0); // the largest error along any shared edge
  for (int k = 0; k < K; ++k) {
    R[k] = stridedSamples(ch.r0, ch.r1, 1 << k);
    C[k] = stridedSamples(ch.c0, ch.c1, 1 << k);
    if (k == 0)
      continue;
    for (int r = ch.r0; r <= ch.r1; ++r) {
      const int i = intervalOf(R[k], r);
      const int ra = R[k][i], rb = R[k][i + 1];
      const double v = double(r - ra) / (rb - ra);
      for (int c = ch.c0; c <= ch.c1; ++c) {
        const double z = H(r, c);
        const int j = intervalOf(C[k], c);
        const int ca = C[k][j], cb = C[k][j + 1];
        const double sw = H(ra, ca), se = H(ra, cb), ne = H(rb, cb), nw = H(rb, ca);
        if (std::isnan(z) || std::isnan(sw) || std::isnan(se) || std::isnan(ne) || std::isnan(nw))
          continue;
        const double u = double(c - ca) / (cb - ca);
        // The same split as hf's mesh: SW-SE-NE below the diagonal, SW-NE-NW above it.
        const double zi =
            u >= v ? sw + u * (se - sw) + v * (ne - se) : sw + v * (nw - sw) + u * (ne - nw);
        const double e = std::abs(z - zi);
        err[k] = std::max(err[k], e);
        if ((r == ch.r0 && shares[kSouth]) || (r == ch.r1 && shares[kNorth]) ||
            (c == ch.c0 && shares[kWest]) || (c == ch.c1 && shares[kEast]))
          edge[k] = std::max(edge[k], e);
      }
    }
  }
  ch.error_m.resize(K);
  for (int k = 0; k < K; ++k)
    ch.error_m[k] = std::max(err[k], k > 0 ? ch.error_m[k - 1] : 0.0);

  // A skirt is deep enough for a neighbour a rung either side: the gap along a shared edge is at
  // most what the two rungs each leave out of it. The floor hides the pixel cracks a T-junction
  // shows even where the heights agree.
  const bool any_neighbour = shares[0] || shares[1] || shares[2] || shares[3];
  const double floor_m = std::max(0.5, 0.1 * std::min(hf.dx(), hf.dy()));
  ch.skirt_m.assign(K, 0.0);
  if (any_neighbour)
    for (int k = 0; k < K; ++k)
      ch.skirt_m[k] =
          edge[k] + std::max(k > 0 ? edge[k - 1] : 0.0, k + 1 < K ? edge[k + 1] : 0.0) + floor_m;

  const cvc::geometry::points_t &P = hf.points();
  const cvc::geometry::normals_t &N = hf.normals();
  const cvc::geometry::uvs_t &UV = hf.uvs();
  const bool haveN = N.size() == P.size(), haveUV = UV.size() == P.size();
  ch.rungs.resize(K);
  for (int k = 0; k < K; ++k) {
    const std::vector<int> &rs = R[k], &cs = C[k];
    const std::size_t nr = rs.size(), nc = cs.size();
    cvc::geometry::points_t pts;
    cvc::geometry::normals_t nrm;
    cvc::geometry::uvs_t uv;
    cvc::geometry::tris_t tri;
    pts.reserve(nr * nc);
    nrm.reserve(nr * nc);
    uv.reserve(nr * nc);
    tri.reserve((nr - 1) * (nc - 1) * 2);
    constexpr std::uint64_t kNone = std::numeric_limits<std::uint64_t>::max();
    std::vector<std::uint64_t> vert(nr * nc, kNone);
    auto addVertex = [&](std::int64_t src, double dz) {
      cvc::geometry::point_t p = P[std::size_t(src)];
      p[2] -= dz;
      pts.push_back(p);
      if (haveN)
        nrm.push_back(N[std::size_t(src)]);
      if (haveUV)
        uv.push_back(UV[std::size_t(src)]);
      return std::uint64_t(pts.size() - 1);
    };
    for (std::size_t i = 0; i < nr; ++i)
      for (std::size_t j = 0; j < nc; ++j) {
        const std::int64_t src = hf.vertex_of(rs[i], cs[j]);
        if (src >= 0)
          vert[i * nc + j] = addVertex(src, 0.0);
      }
    for (std::size_t i = 0; i + 1 < nr; ++i)
      for (std::size_t j = 0; j + 1 < nc; ++j) {
        const std::uint64_t a = vert[i * nc + j], b = vert[i * nc + j + 1],
                            d = vert[(i + 1) * nc + j + 1], e = vert[(i + 1) * nc + j];
        if (a == kNone || b == kNone || d == kNone || e == kNone)
          continue;
        tri.push_back({{a, b, d}});
        tri.push_back({{a, d, e}});
      }
    if (k == 0 && !pts.empty()) {
      for (int a = 0; a < 3; ++a) {
        ch.min[a] = std::numeric_limits<double>::infinity();
        ch.max[a] = -std::numeric_limits<double>::infinity();
      }
      for (const cvc::geometry::point_t &p : pts)
        for (int a = 0; a < 3; ++a) {
          ch.min[a] = std::min(ch.min[a], p[a]);
          ch.max[a] = std::max(ch.max[a], p[a]);
        }
    }
    // The skirts: each shared edge's samples again, skirt_m lower, joined to the edge by quads.
    if (ch.skirt_m[k] > 0.0) {
      auto skirt = [&](bool along_row, std::size_t fixed) {
        const std::size_t n = along_row ? nc : nr;
        std::uint64_t prevTop = kNone, prevLow = kNone;
        for (std::size_t t = 0; t < n; ++t) {
          const std::size_t i = along_row ? fixed : t, j = along_row ? t : fixed;
          const std::uint64_t top = vert[i * nc + j];
          std::uint64_t low = kNone;
          if (top != kNone)
            low = addVertex(hf.vertex_of(rs[i], cs[j]), ch.skirt_m[k]);
          if (top != kNone && prevTop != kNone) {
            tri.push_back({{prevTop, prevLow, low}});
            tri.push_back({{prevTop, low, top}});
          }
          prevTop = top;
          prevLow = low;
        }
      };
      if (shares[kSouth])
        skirt(true, 0);
      if (shares[kNorth])
        skirt(true, nr - 1);
      if (shares[kWest])
        skirt(false, 0);
      if (shares[kEast])
        skirt(false, nc - 1);
    }
    cvc::geometry g;
    g.points() = std::move(pts);
    if (haveN)
      g.normals() = std::move(nrm);
    if (haveUV)
      g.uvs() = std::move(uv);
    g.tris() = std::move(tri);
    ch.rungs[k] = std::move(g);
  }
  return ch;
}

cvc::geometry::color_t water_colour(double depth_m) {
  // Per-metre absorption (pure water's ratios, a little turbid), the bed (sand), and the water's
  // own scattered colour; the path is down and back up at a typical slant.
  const double sigma[3] = {0.45, 0.065, 0.035};
  const double bed[3] = {0.60, 0.55, 0.40}, deep[3] = {0.02, 0.13, 0.21};
  const double path = 2.6 * std::max(0.0, depth_m);
  cvc::geometry::color_t c;
  for (int k = 0; k < 3; ++k) {
    const double t = std::exp(-sigma[k] * path);
    c[std::size_t(k)] = bed[k] * t + deep[k] * (1.0 - t);
  }
  return c;
}

std::vector<cvc::geometry> make_water_rungs(const cvc::height_field &hf,
                                            const terrain_chunk_layout &L, const terrain_chunk &ch,
                                            double water_level_m) {
  const int K = std::max(1, L.rungs);
  std::vector<cvc::geometry> out(static_cast<std::size_t>(K));
  if (!std::isfinite(water_level_m) || ch.min[2] >= water_level_m)
    return out; // nothing of this chunk is under water
  const std::vector<float> &h = hf.heights();
  const int cols = hf.cols();
  auto H = [&](int r, int c) { return double(h[std::size_t(r) * cols + c]); };
  bool any = false;
  for (int k = 0; k < K; ++k) {
    const std::vector<int> rs = stridedSamples(ch.r0, ch.r1, 1 << k);
    const std::vector<int> cs = stridedSamples(ch.c0, ch.c1, 1 << k);
    const std::size_t nr = rs.size(), nc = cs.size();
    constexpr std::uint64_t kNone = std::numeric_limits<std::uint64_t>::max();
    std::vector<std::uint64_t> vert(nr * nc, kNone);
    cvc::geometry::points_t pts;
    cvc::geometry::normals_t nrm;
    cvc::geometry::colors_t col;
    cvc::geometry::tris_t tri;
    auto vertex = [&](std::size_t i, std::size_t j) {
      std::uint64_t &v = vert[i * nc + j];
      if (v == kNone) {
        const double z = H(rs[i], cs[j]);
        v = pts.size();
        pts.push_back({{hf.x0() + cs[j] * hf.dx(), hf.y0() + rs[i] * hf.dy(), water_level_m}});
        nrm.push_back({{0.0, 0.0, 1.0}});
        col.push_back(water_colour(std::isnan(z) ? 0.0 : water_level_m - z));
      }
      return v;
    };
    for (std::size_t i = 0; i + 1 < nr; ++i)
      for (std::size_t j = 0; j + 1 < nc; ++j) {
        const double lo = std::min({H(rs[i], cs[j]), H(rs[i], cs[j + 1]), H(rs[i + 1], cs[j]),
                                    H(rs[i + 1], cs[j + 1])});
        if (!(lo < water_level_m))
          continue;
        const std::uint64_t a = vertex(i, j), b = vertex(i, j + 1), d = vertex(i + 1, j + 1),
                            e = vertex(i + 1, j);
        tri.push_back({{a, b, d}});
        tri.push_back({{a, d, e}});
      }
    if (tri.empty())
      continue;
    any = true;
    cvc::geometry g;
    g.points() = std::move(pts);
    g.normals() = std::move(nrm);
    g.colors() = std::move(col);
    g.tris() = std::move(tri);
    out[std::size_t(k)] = std::move(g);
  }
  if (!any)
    out.assign(std::size_t(K), cvc::geometry());
  return out;
}

void constrain_terrain_rungs(std::vector<int> &rung, int rows, int cols) {
  if (rows <= 0 || cols <= 0 || rung.size() != std::size_t(rows) * cols)
    return;
  // Lowering a chunk can only lower what its neighbours may be, so sweep until nothing moves.
  for (bool moved = true; moved;) {
    moved = false;
    for (int r = 0; r < rows; ++r)
      for (int c = 0; c < cols; ++c) {
        int &k = rung[std::size_t(r) * cols + c];
        int lo = k;
        if (r > 0)
          lo = std::min(lo, rung[std::size_t(r - 1) * cols + c] + 1);
        if (r + 1 < rows)
          lo = std::min(lo, rung[std::size_t(r + 1) * cols + c] + 1);
        if (c > 0)
          lo = std::min(lo, rung[std::size_t(r) * cols + c - 1] + 1);
        if (c + 1 < cols)
          lo = std::min(lo, rung[std::size_t(r) * cols + c + 1] + 1);
        if (lo < k) {
          k = lo;
          moved = true;
        }
      }
  }
}

} // namespace world
} // namespace cvc
