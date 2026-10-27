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

// cvc::compute_contour_tree against a brute-force oracle: at every level between consecutive
// vertices the number of connected level-set components, glued through the faces of the
// Freudenthal tetrahedra, must equal the number of arcs spanning that level. The grids include
// the 2D images and thin extruded slabs where VolumeRover's original combine step went wrong.
// Then the resumable builder and simplifier: any stepping schedule gives exactly the one-shot
// result, and every phase really is sliced (a phase with U units of work is worked on by at least
// ceil(U / budget) steps).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cvc/core/app.h>
#include <cvc/volume/contour_tree.h>
#include <cvc/volume/voxels.h>
#include <functional>
#include <gtest/gtest.h>
#include <limits>
#include <numeric>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using cvc::contour_tree;
using cvc::contour_tree_options;
using kind = cvc::contour_tree::node_kind;

namespace {

struct field {
  int nx = 1, ny = 1, nz = 1;
  std::vector<float> f;
  int n() const { return nx * ny * nz; }
  int id(int i, int j, int k) const {
    if (i < 0 || j < 0 || k < 0 || i >= nx || j >= ny || k >= nz)
      return -1;
    return i + j * nx + k * nx * ny;
  }
  std::string describe() const {
    std::ostringstream s;
    s << nx << "x" << ny << "x" << nz << " [";
    for (std::size_t i = 0; i < f.size() && i < 64; ++i)
      s << (i ? "," : "") << f[i];
    s << (f.size() > 64 ? ",...]" : "]");
    return s.str();
  }
};

field make_field(int nx, int ny, int nz, float v = 0.f) {
  field g;
  g.nx = nx;
  g.ny = ny;
  g.nz = nz;
  g.f.assign(std::size_t(nx) * ny * nz, v);
  return g;
}

contour_tree tree_of(const field &g, const contour_tree_options &opt = {}) {
  return cvc::compute_contour_tree(g.f.data(), g.nx, g.ny, g.nz, opt);
}

// Vertex ranks in (value, linear index) order: the simulation of simplicity the tree is built on.
std::vector<int> ranks_of(const field &g) {
  std::vector<int> order(g.f.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(),
            [&](int a, int b) { return g.f[a] < g.f[b] || (g.f[a] == g.f[b] && a < b); });
  std::vector<int> rank(order.size());
  for (std::size_t r = 0; r < order.size(); ++r)
    rank[order[r]] = int(r);
  return rank;
}

// ---- the brute-force oracle -----------------------------------------------------------------

// One representative of each +/- pair of VolumeRover's 14-neighbour stencil.
constexpr int kRep[7][3] = {{1, 0, 0},  {0, 1, 0},  {0, 0, 1}, {1, 1, 0},
                            {0, 1, -1}, {1, 0, -1}, {1, 1, -1}};

// The triangles of the triangulation, built independently of the stencil from the simplices of
// the Freudenthal split (paths from a cube corner stepping +x, +y, -z in every order), with their
// edges as slots v*7 + t of the stencil (so a triangle edge outside the stencil is caught).
struct complex {
  int nx, ny, nz, n;
  std::vector<int> ea, eb;               // per edge slot; -1 when the slot is off the grid
  std::vector<std::array<int, 3>> tris;  // edge slots of each triangle
  std::vector<std::uint8_t> slot_in_tri; // every stencil edge must lie on some simplex
  std::vector<int> p;                    // union-find scratch
  bool bad_edge = false;
};

int edge_slot(const field &g, int a, int b) {
  const int ai = a % g.nx, aj = (a / g.nx) % g.ny, ak = a / (g.nx * g.ny);
  const int bi = b % g.nx, bj = (b / g.nx) % g.ny, bk = b / (g.nx * g.ny);
  const int d[3] = {bi - ai, bj - aj, bk - ak};
  for (int t = 0; t < 7; ++t) {
    if (d[0] == kRep[t][0] && d[1] == kRep[t][1] && d[2] == kRep[t][2])
      return a * 7 + t;
    if (d[0] == -kRep[t][0] && d[1] == -kRep[t][1] && d[2] == -kRep[t][2])
      return b * 7 + t;
  }
  return -1;
}

complex build_complex(const field &g) {
  complex c;
  c.nx = g.nx;
  c.ny = g.ny;
  c.nz = g.nz;
  c.n = g.n();
  c.ea.assign(std::size_t(c.n) * 7, -1);
  c.eb.assign(std::size_t(c.n) * 7, -1);
  for (int k = 0; k < g.nz; ++k)
    for (int j = 0; j < g.ny; ++j)
      for (int i = 0; i < g.nx; ++i)
        for (int t = 0; t < 7; ++t) {
          const int w = g.id(i + kRep[t][0], j + kRep[t][1], k + kRep[t][2]);
          if (w >= 0) {
            c.ea[g.id(i, j, k) * 7 + t] = g.id(i, j, k);
            c.eb[g.id(i, j, k) * 7 + t] = w;
          }
        }
  c.slot_in_tri.assign(c.ea.size(), 0);

  const int dims[3] = {g.nx, g.ny, g.nz};
  std::vector<int> active;
  for (int a = 0; a < 3; ++a)
    if (dims[a] > 1)
      active.push_back(a);
  const int step[3] = {1, 1, -1};
  auto mark = [&](int a, int b) {
    const int s = edge_slot(g, a, b);
    if (s < 0 || c.ea[s] < 0) {
      c.bad_edge = true;
      return 0;
    }
    c.slot_in_tri[s] = 1;
    return s;
  };
  auto add_tri = [&](int a, int b, int d) {
    c.tris.push_back({mark(a, b), mark(b, d), mark(a, d)});
  };
  for (int k = 0; k < std::max(g.nz - 1, 1); ++k)
    for (int j = 0; j < std::max(g.ny - 1, 1); ++j)
      for (int i = 0; i < std::max(g.nx - 1, 1); ++i) {
        std::vector<int> perm = active;
        do {
          int p[3] = {i, j, g.nz > 1 ? k + 1 : 0};
          std::vector<int> vs{g.id(p[0], p[1], p[2])};
          for (int a : perm) {
            p[a] += step[a];
            vs.push_back(g.id(p[0], p[1], p[2]));
          }
          if (vs.size() == 2)
            mark(vs[0], vs[1]);
          else if (vs.size() == 3)
            add_tri(vs[0], vs[1], vs[2]);
          else if (vs.size() == 4) {
            add_tri(vs[0], vs[1], vs[2]);
            add_tri(vs[0], vs[1], vs[3]);
            add_tri(vs[0], vs[2], vs[3]);
            add_tri(vs[1], vs[2], vs[3]);
          }
        } while (std::next_permutation(perm.begin(), perm.end()));
      }
  c.p.resize(c.ea.size());
  return c;
}

// Components of the level set separating the vertices with below[v] from the rest: crossing
// edges, glued through every triangle that two of them cross.
int level_set_components(complex &c, const std::vector<std::uint8_t> &below) {
  auto crossing = [&](int s) { return c.ea[s] >= 0 && below[c.ea[s]] != below[c.eb[s]]; };
  auto find = [&](int x) {
    while (c.p[x] != x) {
      c.p[x] = c.p[c.p[x]];
      x = c.p[x];
    }
    return x;
  };
  for (std::size_t s = 0; s < c.p.size(); ++s)
    c.p[s] = int(s);
  for (const auto &t : c.tris) {
    int cs[3], m = 0;
    for (int s : t)
      if (crossing(s))
        cs[m++] = s;
    if (m == 2) {
      const int a = find(cs[0]), b = find(cs[1]);
      if (a != b)
        c.p[a] = b;
    }
  }
  int comps = 0;
  for (std::size_t s = 0; s < c.p.size(); ++s)
    if (crossing(int(s)) && find(int(s)) == int(s))
      ++comps;
  return comps;
}

// ---- tree checks ----------------------------------------------------------------------------

bool above(const contour_tree &t, std::uint32_t a, std::uint32_t b) {
  const auto &p = t.nodes[a], &q = t.nodes[b];
  return p.value > q.value || (p.value == q.value && p.voxel > q.voxel);
}

kind expected_kind(int up, int down) {
  if (down == 0)
    return kind::minimum;
  if (up == 0)
    return kind::maximum;
  if (up >= 2 && down >= 2)
    return kind::join_split;
  return up >= 2 ? kind::join : kind::split;
}

// A tree: arcs == nodes - 1, connected, every arc's hi above its lo, nodes sorted by (value,
// voxel), no regular node, kinds matching the degrees, and (given the field) every node a voxel
// of it carrying that voxel's value.
::testing::AssertionResult valid_tree(const contour_tree &t, const field *g = nullptr) {
  const std::size_t m = t.nodes.size();
  if (m == 0)
    return ::testing::AssertionFailure() << "no nodes";
  if (t.arcs.size() + 1 != m)
    return ::testing::AssertionFailure() << m << " nodes but " << t.arcs.size() << " arcs";
  std::vector<int> up(m, 0), dn(m, 0), p(m);
  std::iota(p.begin(), p.end(), 0);
  auto find = [&](int x) {
    while (p[x] != x)
      x = p[x] = p[p[x]];
    return x;
  };
  int parts = int(m);
  for (const auto &a : t.arcs) {
    if (a.hi >= m || a.lo >= m)
      return ::testing::AssertionFailure() << "arc index out of range";
    if (!above(t, a.hi, a.lo))
      return ::testing::AssertionFailure() << "arc " << a.hi << "-" << a.lo << " hi not above lo";
    ++dn[a.hi];
    ++up[a.lo];
    const int x = find(int(a.hi)), y = find(int(a.lo));
    if (x != y) {
      p[x] = y;
      --parts;
    }
  }
  if (parts != 1)
    return ::testing::AssertionFailure() << "disconnected: " << parts << " parts";
  // With two or more grid axes every vertex link is a circle, sphere or disk, so an extremum's
  // whole (connected) link lies on one side: extrema are leaves. Only lines have 2-arc extrema.
  const bool manifold = g && (g->nx > 1) + (g->ny > 1) + (g->nz > 1) >= 2;
  for (std::size_t i = 0; manifold && m > 1 && i < m; ++i)
    if ((up[i] == 0 || dn[i] == 0) && up[i] + dn[i] != 1)
      return ::testing::AssertionFailure()
             << "extremum " << i << " has " << up[i] + dn[i] << " arcs";
  for (std::size_t i = 0; i < m; ++i) {
    if (i > 0 && !above(t, std::uint32_t(i), std::uint32_t(i - 1)))
      return ::testing::AssertionFailure() << "nodes not sorted at " << i;
    if (m > 1 && up[i] == 1 && dn[i] == 1)
      return ::testing::AssertionFailure() << "regular node " << i;
    if (m > 1 && t.nodes[i].kind != expected_kind(up[i], dn[i]))
      return ::testing::AssertionFailure() << "node " << i << " kind " << int(t.nodes[i].kind)
                                           << " up " << up[i] << " down " << dn[i];
    if (g) {
      if (t.nodes[i].voxel >= cvc::uint64(g->n()))
        return ::testing::AssertionFailure() << "node voxel out of range";
      if (t.nodes[i].value != double(g->f[std::size_t(t.nodes[i].voxel)]))
        return ::testing::AssertionFailure() << "node value differs from the field";
    }
  }
  return ::testing::AssertionSuccess();
}

// The oracle at every level between consecutive ranks (arc spans by rank), and components_at at
// the midpoint between each pair of consecutive distinct values.
::testing::AssertionResult matches_level_sets(const contour_tree &t, const field &g) {
  complex c = build_complex(g);
  if (c.bad_edge)
    return ::testing::AssertionFailure() << "a triangle edge is not a stencil edge";
  const std::vector<int> rank = ranks_of(g);
  const int n = g.n();
  std::vector<std::uint8_t> below(std::size_t(n), 0);
  std::vector<int> order(static_cast<std::size_t>(n));
  for (int v = 0; v < n; ++v)
    order[rank[v]] = v;
  for (int r = 0; r + 1 < n; ++r) {
    below[order[r]] = 1;
    const int comps = level_set_components(c, below);
    int span = 0;
    for (const auto &a : t.arcs)
      if (rank[t.nodes[a.lo].voxel] <= r && r < rank[t.nodes[a.hi].voxel])
        ++span;
    if (comps != span)
      return ::testing::AssertionFailure()
             << "rank " << r << ": " << comps << " components, " << span << " spanning arcs";
  }
  std::vector<float> values = g.f;
  std::sort(values.begin(), values.end());
  values.erase(std::unique(values.begin(), values.end()), values.end());
  for (std::size_t i = 0; i + 1 < values.size(); ++i) {
    const double w = (double(values[i]) + double(values[i + 1])) / 2.0;
    for (int v = 0; v < n; ++v)
      below[v] = double(g.f[v]) < w;
    const int comps = level_set_components(c, below);
    if (std::size_t(comps) != t.components_at(w))
      return ::testing::AssertionFailure() << "value " << w << ": " << comps
                                           << " components, components_at " << t.components_at(w);
    if (t.arcs_at(w).size() != t.components_at(w))
      return ::testing::AssertionFailure() << "arcs_at and components_at disagree";
  }
  return ::testing::AssertionSuccess();
}

// The oracle at a few sampled values only, for grids too large to sweep every rank.
::testing::AssertionResult matches_sampled_level_sets(const contour_tree &t, const field &g,
                                                      int samples) {
  complex c = build_complex(g);
  if (c.bad_edge)
    return ::testing::AssertionFailure() << "a triangle edge is not a stencil edge";
  std::vector<float> values = g.f;
  std::sort(values.begin(), values.end());
  values.erase(std::unique(values.begin(), values.end()), values.end());
  std::vector<std::uint8_t> below(g.f.size(), 0);
  for (int s = 0; s < samples && values.size() > 1; ++s) {
    const std::size_t i = (values.size() - 1) * std::size_t(s) / std::size_t(samples);
    const double w = (double(values[i]) + double(values[i + 1])) / 2.0;
    for (std::size_t v = 0; v < g.f.size(); ++v)
      below[v] = double(g.f[v]) < w;
    const int comps = level_set_components(c, below);
    if (std::size_t(comps) != t.components_at(w))
      return ::testing::AssertionFailure() << "value " << w << ": " << comps
                                           << " components, components_at " << t.components_at(w);
  }
  return ::testing::AssertionSuccess();
}

// ---- fields ---------------------------------------------------------------------------------

std::uint32_t pick(std::mt19937 &rng, std::uint32_t lo, std::uint32_t hi) {
  return lo + std::uint32_t(rng() % (hi - lo + 1));
}

void fill_noise(field &g, std::mt19937 &rng, std::uint32_t levels) {
  for (float &v : g.f)
    v = float(rng() % levels);
}

// A sum of Gaussian bumps and dips; `quantize` truncates to 0..255 like VolumeRover2's uchar remap
// (big plateaus, so many ties).
void fill_smooth(field &g, std::mt19937 &rng, int bumps, bool quantize) {
  struct bump {
    double x, y, z, s, a;
  };
  std::vector<bump> bs;
  for (int b = 0; b < bumps; ++b)
    bs.push_back({double(rng() % 1000) / 1000.0 * g.nx, double(rng() % 1000) / 1000.0 * g.ny,
                  double(rng() % 1000) / 1000.0 * g.nz, 1.0 + double(rng() % 3),
                  rng() % 2 ? 1.0 : -0.6});
  std::vector<double> v(g.f.size());
  for (int k = 0; k < g.nz; ++k)
    for (int j = 0; j < g.ny; ++j)
      for (int i = 0; i < g.nx; ++i) {
        double s = 0;
        for (const bump &b : bs) {
          const double d = (i - b.x) * (i - b.x) + (j - b.y) * (j - b.y) + (k - b.z) * (k - b.z);
          s += b.a * std::exp(-d / (2 * b.s * b.s));
        }
        v[g.id(i, j, k)] = s;
      }
  const auto [lo, hi] = std::minmax_element(v.begin(), v.end());
  const double mn = *lo, mx = *hi;
  for (std::size_t t = 0; t < v.size(); ++t)
    g.f[t] = quantize ? float(std::uint8_t(mx > mn ? 255.0 * (v[t] - mn) / (mx - mn) : 0.0))
                      : float(v[t]);
}

void fill(field &g, std::mt19937 &rng, int mode) {
  switch (mode % 5) {
  case 0:
    fill_noise(g, rng, 256);
    break;
  case 1:
    fill_noise(g, rng, 3);
    break;
  case 2:
    fill_noise(g, rng, 2);
    break;
  case 3:
    fill_smooth(g, rng, 2 + int(rng() % 5), true);
    break;
  default:
    fill_smooth(g, rng, 2 + int(rng() % 5), false);
    break;
  }
}

// A 2D field repeated `copies` times along `axis` (the thin slabs VolumeRover got wrong).
field extrude(const field &slice, int axis, int copies) {
  // slice is nx*ny*1; place its two axes on the other two.
  const int a0 = axis == 0 ? 1 : 0, a1 = axis == 2 ? 1 : 2;
  int dims[3] = {1, 1, 1};
  dims[a0] = slice.nx;
  dims[a1] = slice.ny;
  dims[axis] = copies;
  field g = make_field(dims[0], dims[1], dims[2]);
  for (int k = 0; k < dims[2]; ++k)
    for (int j = 0; j < dims[1]; ++j)
      for (int i = 0; i < dims[0]; ++i) {
        const int p[3] = {i, j, k};
        g.f[g.id(i, j, k)] = slice.f[slice.id(p[a0], p[a1], 0)];
      }
  return g;
}

field random_grid(std::mt19937 &rng, int trial) {
  const int mode = trial / 6;
  int d[3] = {1, 1, 1};
  switch (trial % 6) {
  case 0: { // a line along any axis
    d[rng() % 3] = int(pick(rng, 2, 16));
    break;
  }
  case 1: { // an image in any plane
    const int skip = int(rng() % 3);
    for (int a = 0; a < 3; ++a)
      if (a != skip)
        d[a] = int(pick(rng, 2, 8));
    break;
  }
  case 2: { // a thin slab
    field slice = make_field(int(pick(rng, 2, 7)), int(pick(rng, 2, 7)), 1);
    fill(slice, rng, mode);
    return extrude(slice, int(rng() % 3), int(pick(rng, 2, 4)));
  }
  default: // a box
    for (int &x : d)
      x = int(pick(rng, 2, 6));
    break;
  }
  field g = make_field(d[0], d[1], d[2]);
  fill(g, rng, mode);
  return g;
}

std::set<std::pair<cvc::uint64, cvc::uint64>> arc_voxels(const contour_tree &t) {
  std::set<std::pair<cvc::uint64, cvc::uint64>> s;
  for (const auto &a : t.arcs)
    s.insert({t.nodes[a.hi].voxel, t.nodes[a.lo].voxel});
  return s;
}

std::size_t count_kind(const contour_tree &t, kind k) {
  return std::size_t(std::count_if(t.nodes.begin(), t.nodes.end(),
                                   [k](const contour_tree::node &nd) { return nd.kind == k; }));
}

bool same_tree(const contour_tree &a, const contour_tree &b) {
  if (a.dims != b.dims || a.nodes.size() != b.nodes.size() || a.arcs.size() != b.arcs.size())
    return false;
  for (std::size_t i = 0; i < a.nodes.size(); ++i)
    if (a.nodes[i].voxel != b.nodes[i].voxel || a.nodes[i].value != b.nodes[i].value ||
        a.nodes[i].kind != b.nodes[i].kind)
      return false;
  for (std::size_t i = 0; i < a.arcs.size(); ++i)
    if (a.arcs[i].hi != b.arcs[i].hi || a.arcs[i].lo != b.arcs[i].lo)
      return false;
  return true;
}

template <class T> cvc::voxels typed_voxels(cvc::app &ctx, const field &g, cvc::data_type type) {
  std::vector<T> buf(g.f.size());
  for (std::size_t i = 0; i < buf.size(); ++i)
    buf[i] = static_cast<T>(g.f[i]);
  return cvc::voxels(ctx, buf.data(), cvc::dimension(g.nx, g.ny, g.nz), type);
}

// Two Gaussian peaks of different heights in a 32^3 double field.
field two_blobs() {
  field g = make_field(32, 32, 32);
  for (int k = 0; k < 32; ++k)
    for (int j = 0; j < 32; ++j)
      for (int i = 0; i < 32; ++i) {
        auto bump = [&](double x, double y, double z, double a) {
          const double d = (i - x) * (i - x) + (j - y) * (j - y) + (k - z) * (k - z);
          return a * std::exp(-d / (2 * 4.0 * 4.0));
        };
        g.f[g.id(i, j, k)] = float(bump(9.3, 10.1, 11.2, 1.0) + bump(22.4, 20.6, 19.7, 0.7));
      }
  return g;
}

} // namespace

// ---- hand cases -----------------------------------------------------------------------------

TEST(ContourTree, SingleVoxel) {
  const float v = 4.f;
  const contour_tree t = cvc::compute_contour_tree(&v, 1, 1, 1);
  ASSERT_EQ(t.nodes.size(), 1u);
  EXPECT_TRUE(t.arcs.empty());
  EXPECT_TRUE(t.empty());
  EXPECT_EQ(t.nodes[0].voxel, 0u);
  EXPECT_EQ(t.nodes[0].value, 4.0);
  EXPECT_EQ(t.value_min, 4.0);
  EXPECT_EQ(t.value_max, 4.0);
  EXPECT_EQ(t.components_at(4.0), 0u);
}

TEST(ContourTree, ConstantVolumeIsOneNode) {
  cvc::app ctx;
  cvc::voxels vol(ctx, cvc::dimension(4, 4, 4), cvc::Float);
  vol.fill(3.5);
  const contour_tree t = cvc::compute_contour_tree(vol);
  ASSERT_EQ(t.nodes.size(), 1u);
  EXPECT_TRUE(t.arcs.empty());
  EXPECT_EQ(t.value_min, 3.5);
  EXPECT_EQ(t.value_max, 3.5);
  EXPECT_EQ((t.dims), (std::array<cvc::uint64, 3>{{4, 4, 4}}));
  EXPECT_EQ(t.components_at(3.5), 0u);
  EXPECT_TRUE(t.arcs_at(3.5).empty());
}

TEST(ContourTree, Line) {
  // 0 5 1 4 2: two interior maxima, each with both arcs going down (a 1D maximum's lower link is
  // two points), one interior minimum with two arcs up, and the two ends.
  field g = make_field(5, 1, 1);
  g.f = {0, 5, 1, 4, 2};
  const contour_tree t = tree_of(g);
  ASSERT_TRUE(valid_tree(t, &g));
  ASSERT_EQ(t.nodes.size(), 5u);
  const std::set<std::pair<cvc::uint64, cvc::uint64>> want{{1, 0}, {1, 2}, {3, 2}, {3, 4}};
  EXPECT_EQ(arc_voxels(t), want);
  EXPECT_EQ(t.nodes.front().voxel, 0u); // the global minimum
  EXPECT_EQ(t.nodes.back().voxel, 1u);  // the global maximum
  EXPECT_EQ(t.nodes.back().kind, kind::maximum);
  EXPECT_EQ(t.nodes[1].voxel, 2u);
  EXPECT_EQ(t.nodes[1].kind, kind::minimum);
  EXPECT_EQ(t.components_at(0.5), 1u);
  EXPECT_EQ(t.components_at(1.5), 3u);
  EXPECT_EQ(t.components_at(3.0), 4u);
  EXPECT_EQ(t.components_at(4.5), 2u);
  EXPECT_EQ(t.components_at(-1.0), 0u);
  EXPECT_EQ(t.components_at(6.0), 0u);
  // At a node's own value the rule is half-open, lo <= w < hi: the arcs leaving it upward count.
  EXPECT_EQ(t.components_at(0.0), 1u);
  EXPECT_EQ(t.components_at(1.0), 3u);
  EXPECT_EQ(t.components_at(4.0), 2u);
  const std::vector<std::uint32_t> at0 = t.arcs_at(0.0);
  ASSERT_EQ(at0.size(), 1u);
  EXPECT_EQ(t.nodes[t.arcs[at0[0]].lo].voxel, 0u);
  EXPECT_EQ(t.nodes[t.arcs[at0[0]].hi].voxel, 1u);
  EXPECT_TRUE(matches_level_sets(t, g));
}

TEST(ContourTree, CubeFromComputeCTComment) {
  // computeCT.cpp's commented-out example: udata = {0,1,2,5,3,1,5,6}, dim = {2,2,2}. Every
  // vertex but the global extrema has a connected upper and lower link, so the tree is one arc.
  field g = make_field(2, 2, 2);
  g.f = {0, 1, 2, 5, 3, 1, 5, 6};
  const contour_tree t = tree_of(g);
  ASSERT_TRUE(valid_tree(t, &g));
  ASSERT_EQ(t.nodes.size(), 2u);
  EXPECT_EQ(t.nodes[0].voxel, 0u);
  EXPECT_EQ(t.nodes[0].kind, kind::minimum);
  EXPECT_EQ(t.nodes[1].voxel, 7u);
  EXPECT_EQ(t.nodes[1].kind, kind::maximum);
  ASSERT_EQ(t.arcs.size(), 1u);
  EXPECT_EQ(t.arcs[0].hi, 1u);
  EXPECT_EQ(t.arcs[0].lo, 0u);
  EXPECT_EQ(t.components_at(2.5), 1u);
  EXPECT_EQ(t.components_at(6.0), 1u); // the top: the arc ending there
  EXPECT_TRUE(matches_level_sets(t, g));
}

TEST(ContourTree, PlateauAtTheTop) {
  // A U-shaped plateau at the maximum. Simulation of simplicity orders it by index, so both arm
  // tips are maxima joined inside the plateau by arcs lying entirely at the top value; the level
  // set there is still one U, reached by the one arc coming up from below.
  field g = make_field(5, 3, 1);
  g.f = {9, 9, 9, 9, 9, //
         9, 0, 0, 0, 9, //
         9, 0, 0, 0, 9};
  const contour_tree t = tree_of(g);
  ASSERT_TRUE(valid_tree(t, &g));
  EXPECT_EQ(count_kind(t, kind::maximum), 2u);
  std::size_t inside = 0;
  for (const auto &a : t.arcs)
    inside += t.nodes[a.lo].value == 9.0 ? 1 : 0;
  EXPECT_GE(inside, 2u);
  EXPECT_EQ(t.components_at(9.0), 1u);
  ASSERT_EQ(t.arcs_at(9.0).size(), 1u);
  EXPECT_LT(t.nodes[t.arcs[t.arcs_at(9.0)[0]].lo].value, 9.0);
  EXPECT_EQ(t.components_at(4.5), 1u);
  EXPECT_TRUE(matches_level_sets(t, g));
}

TEST(ContourTree, StencilIsTheFreudenthalSplit) {
  // The triangulation's edges are exactly the 14-neighbour stencil, on every grid shape.
  const int shapes[][3] = {{4, 1, 1}, {1, 4, 1}, {1, 1, 4}, {3, 4, 1},
                           {3, 1, 4}, {1, 3, 4}, {3, 4, 5}, {2, 2, 2}};
  for (const auto &s : shapes) {
    const field g = make_field(s[0], s[1], s[2]);
    const complex c = build_complex(g);
    EXPECT_FALSE(c.bad_edge) << g.describe();
    for (std::size_t e = 0; e < c.ea.size(); ++e)
      if (c.ea[e] >= 0)
        EXPECT_TRUE(c.slot_in_tri[e]) << g.describe() << " stencil edge " << c.ea[e] << "-"
                                      << c.eb[e] << " lies on no simplex";
  }
}

// ---- the oracle -----------------------------------------------------------------------------

TEST(ContourTree, RandomGridsMatchTheLevelSets) {
  std::mt19937 rng(20261025);
  for (int trial = 0; trial < 450; ++trial) {
    const field g = random_grid(rng, trial);
    const contour_tree t = tree_of(g);
    const bool constant =
        std::all_of(g.f.begin(), g.f.end(), [&](float v) { return v == g.f.front(); });
    if (constant) {
      EXPECT_EQ(t.nodes.size(), 1u) << g.describe();
      EXPECT_TRUE(t.arcs.empty()) << g.describe();
      continue;
    }
    ASSERT_TRUE(valid_tree(t, &g)) << "trial " << trial << " " << g.describe();
    ASSERT_TRUE(matches_level_sets(t, g)) << "trial " << trial << " " << g.describe();
  }
}

TEST(ContourTree, ImagesAndThinSlabs) {
  // The shapes VolumeRover's combine step failed most on: 12x12x{2,4} slabs (exhaustive), 64x64
  // noise images and 32x32x{2,4} slabs (sampled levels), each slab also checked against its slice.
  std::mt19937 rng(7);
  for (int trial = 0; trial < 6; ++trial) {
    field slice = make_field(12, 12, 1);
    fill(slice, rng, trial % 5);
    const field g = extrude(slice, 2, trial % 2 ? 4 : 2);
    const contour_tree t = tree_of(g);
    ASSERT_TRUE(valid_tree(t, &g)) << "trial " << trial;
    ASSERT_TRUE(matches_level_sets(t, g)) << "trial " << trial;
  }
  for (int trial = 0; trial < 3; ++trial) {
    field g = make_field(64, 64, 1);
    fill_noise(g, rng, trial == 2 ? 4 : 256);
    const contour_tree t = tree_of(g);
    ASSERT_TRUE(valid_tree(t, &g)) << "image " << trial;
    ASSERT_TRUE(matches_sampled_level_sets(t, g, 24)) << "image " << trial;
  }
  for (int trial = 0; trial < 4; ++trial) {
    field slice = make_field(32, 32, 1);
    fill(slice, rng, trial);
    const int axis = (trial + 2) % 3;
    const field g = extrude(slice, axis, trial % 2 ? 4 : 2);
    const contour_tree t = tree_of(g);
    ASSERT_TRUE(valid_tree(t, &g)) << "slab " << trial;
    ASSERT_TRUE(matches_sampled_level_sets(t, g, 16)) << "slab " << trial;
    // Along z every layer keeps the slice's (1,1,0) diagonal and each tetrahedron projects onto
    // one of its triangles, so the slab's level sets are the slice's, extruded. (The x and y
    // planes use the other diagonal, a different interpolant.)
    if (axis != 2)
      continue;
    const contour_tree ts = tree_of(slice);
    std::vector<float> values = slice.f;
    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end()), values.end());
    for (std::size_t i = 0; i + 1 < values.size(); ++i) {
      const double w = (double(values[i]) + double(values[i + 1])) / 2.0;
      ASSERT_EQ(t.components_at(w), ts.components_at(w)) << "slab " << trial << " at " << w;
    }
  }
}

TEST(ContourTree, SmoothField64) {
  field g = make_field(64, 64, 64);
  std::mt19937 rng(64);
  fill_smooth(g, rng, 40, false);
  const contour_tree t = tree_of(g);
  ASSERT_TRUE(valid_tree(t, &g));
  EXPECT_GE(count_kind(t, kind::maximum), 1u);
  EXPECT_EQ(t.nodes.front().value, t.value_min);
  EXPECT_EQ(t.nodes.back().value, t.value_max);
}

// ---- options and errors ---------------------------------------------------------------------

TEST(ContourTree, RejectsBadInput) {
  field g = make_field(3, 3, 3, 1.f);
  g.f[13] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(tree_of(g), cvc::contour_tree_error);
  const float v = 0.f;
  EXPECT_THROW(cvc::compute_contour_tree(&v, 0, 1, 1), cvc::contour_tree_error);
  EXPECT_THROW(cvc::compute_contour_tree(nullptr, 2, 2, 2), cvc::contour_tree_error);
}

TEST(ContourTree, ProgressAndCancel) {
  field g = make_field(64, 64, 64);
  std::mt19937 rng(3);
  fill_smooth(g, rng, 12, false);
  std::vector<double> seen;
  contour_tree_options opt;
  opt.progress = [&seen](double p) {
    seen.push_back(p);
    return true;
  };
  const contour_tree t = tree_of(g, opt);
  EXPECT_TRUE(valid_tree(t));
  ASSERT_FALSE(seen.empty());
  EXPECT_TRUE(std::is_sorted(seen.begin(), seen.end()));
  EXPECT_GE(seen.front(), 0.0);
  EXPECT_EQ(seen.back(), 1.0);

  opt.progress = [](double) { return false; };
  EXPECT_THROW(tree_of(g, opt), cvc::contour_tree_error);
  int calls = 0;
  opt.progress = [&calls](double) { return ++calls < 6; }; // cancel in the middle of a sweep
  EXPECT_THROW(tree_of(g, opt), cvc::contour_tree_error);
  EXPECT_EQ(calls, 6);
}

TEST(ContourTree, MaxDimResamples) {
  // A ramp in x stays a ramp: two nodes at the first and last voxel of the resampled grid.
  field g = make_field(40, 30, 20);
  for (int k = 0; k < 20; ++k)
    for (int j = 0; j < 30; ++j)
      for (int i = 0; i < 40; ++i)
        g.f[g.id(i, j, k)] = float(i);
  contour_tree_options opt;
  opt.max_dim = 16;
  const contour_tree t = tree_of(g, opt);
  EXPECT_EQ((t.dims), (std::array<cvc::uint64, 3>{{16, 16, 16}}));
  ASSERT_TRUE(valid_tree(t));
  ASSERT_EQ(t.nodes.size(), 2u);
  EXPECT_EQ(t.nodes[0].voxel, 0u);
  EXPECT_EQ(t.nodes[1].voxel, 16u * 16u * 16u - 1u);
  EXPECT_DOUBLE_EQ(t.value_min, 0.0);
  EXPECT_DOUBLE_EQ(t.value_max, 39.0);

  // A size-1 axis stays 1 (voxels::resize would divide 0 by 0 there); an axis already under the
  // limit is kept.
  field img = make_field(50, 12, 1);
  std::mt19937 rng(5);
  fill_smooth(img, rng, 6, false);
  opt.max_dim = 20;
  const contour_tree ti = tree_of(img, opt);
  EXPECT_EQ((ti.dims), (std::array<cvc::uint64, 3>{{20, 12, 1}}));
  EXPECT_TRUE(valid_tree(ti));
  for (const auto &nd : ti.nodes)
    EXPECT_LT(nd.voxel, 20u * 12u);

  // No resampling when every axis is within the limit.
  opt.max_dim = 64;
  EXPECT_TRUE(same_tree(tree_of(img, opt), tree_of(img)));
}

// ---- voxel types ----------------------------------------------------------------------------

class ContourTreeVoxels : public ::testing::Test {
protected:
  cvc::app ctx;
};

TEST_F(ContourTreeVoxels, AllVoxelTypesAgree) {
  std::mt19937 rng(11);
  for (int trial = 0; trial < 4; ++trial) {
    field g = make_field(7, 6, 5);
    if (trial % 2) {
      fill_noise(g, rng, 100);
    } else {
      fill_smooth(g, rng, 5, true);
      for (float &v : g.f)
        v = std::floor(v / 2.56f); // within 0..99, so every type holds it
    }
    const contour_tree ref = tree_of(g);
    ASSERT_TRUE(valid_tree(ref, &g));
    const std::vector<cvc::voxels> vols = {typed_voxels<unsigned char>(ctx, g, cvc::UChar),
                                           typed_voxels<unsigned short>(ctx, g, cvc::UShort),
                                           typed_voxels<unsigned int>(ctx, g, cvc::UInt),
                                           typed_voxels<float>(ctx, g, cvc::Float),
                                           typed_voxels<double>(ctx, g, cvc::Double),
                                           typed_voxels<cvc::uint64>(ctx, g, cvc::UInt64),
                                           typed_voxels<signed char>(ctx, g, cvc::Char),
                                           typed_voxels<int>(ctx, g, cvc::Int),
                                           typed_voxels<cvc::int64>(ctx, g, cvc::Int64)};
    for (const cvc::voxels &v : vols)
      EXPECT_TRUE(same_tree(cvc::compute_contour_tree(v), ref))
          << "trial " << trial << " type " << v.voxelTypeStr();
  }
}

TEST_F(ContourTreeVoxels, SignedTypesAgree) {
  // Negative values exercise the counting sort's sign flip for signed char.
  std::mt19937 rng(12);
  field g = make_field(9, 8, 3);
  for (float &v : g.f)
    v = float(int(rng() % 200) - 100);
  const contour_tree ref = tree_of(g);
  ASSERT_TRUE(valid_tree(ref, &g));
  EXPECT_TRUE(
      same_tree(cvc::compute_contour_tree(typed_voxels<signed char>(ctx, g, cvc::Char)), ref));
  EXPECT_TRUE(same_tree(cvc::compute_contour_tree(typed_voxels<int>(ctx, g, cvc::Int)), ref));
  EXPECT_TRUE(
      same_tree(cvc::compute_contour_tree(typed_voxels<cvc::int64>(ctx, g, cvc::Int64)), ref));
  EXPECT_TRUE(same_tree(cvc::compute_contour_tree(typed_voxels<double>(ctx, g, cvc::Double)), ref));
}

TEST_F(ContourTreeVoxels, NegativeZeroTiesWithZero) {
  // -0.0 == 0.0, so the two must tie (and fall back to index order) in every sort and sweep.
  std::mt19937 rng(14);
  field g = make_field(8, 7, 3);
  for (float &v : g.f)
    v = float(int(rng() % 3) - 1);
  field signed_zero = g;
  for (std::size_t i = 0; i < g.f.size(); ++i)
    if (g.f[i] == 0.f && rng() % 2)
      signed_zero.f[i] = -0.f;
  const contour_tree ref = tree_of(g);
  ASSERT_TRUE(valid_tree(ref, &g));
  EXPECT_TRUE(same_tree(tree_of(signed_zero), ref));
  EXPECT_TRUE(same_tree(
      cvc::compute_contour_tree(typed_voxels<double>(ctx, signed_zero, cvc::Double)), ref));
  EXPECT_TRUE(same_tree(cvc::compute_contour_tree(typed_voxels<int>(ctx, g, cvc::Int)), ref));
}

TEST_F(ContourTreeVoxels, QuantizeU8MatchesVolumeRover2) {
  // ColorTable2 mapped non-uchar volumes to trunc(255 (v - min) / (max - min)), through the
  // volume's own type, and built the tree on that. Node values come back in data units, at the
  // bin's lower edge.
  field g = make_field(12, 11, 10);
  std::mt19937 rng(13);
  fill_smooth(g, rng, 8, false);
  for (float &v : g.f)
    v = 10.f + 90.f * v;
  const auto [lo, hi] = std::minmax_element(g.f.begin(), g.f.end());
  const double vmin = *lo, vmax = *hi;
  field q = g;
  for (float &v : q.f) {
    // A real float store: MSVC folds uint8(float(x)) into uint8(x).
    const volatile float stored = float((double(v) - vmin) / (vmax - vmin) * 255.0);
    v = float(std::uint8_t(stored));
  }
  const contour_tree want =
      cvc::compute_contour_tree(typed_voxels<unsigned char>(ctx, q, cvc::UChar));

  contour_tree_options opt;
  opt.quantize_u8 = true;
  const contour_tree t = cvc::compute_contour_tree(typed_voxels<float>(ctx, g, cvc::Float), opt);
  ASSERT_TRUE(valid_tree(t));
  EXPECT_EQ(arc_voxels(t), arc_voxels(want));
  ASSERT_EQ(t.nodes.size(), want.nodes.size());
  EXPECT_DOUBLE_EQ(t.value_min, vmin);
  EXPECT_DOUBLE_EQ(t.value_max, vmax);
  for (std::size_t i = 0; i < t.nodes.size(); ++i) {
    EXPECT_EQ(t.nodes[i].voxel, want.nodes[i].voxel);
    const double bin = want.nodes[i].value;
    EXPECT_DOUBLE_EQ(t.nodes[i].value, bin == 255 ? vmax : vmin + bin * (vmax - vmin) / 255.0);
  }
  // A uchar volume is used raw, as VolumeRover2 did.
  const cvc::voxels u = typed_voxels<unsigned char>(ctx, q, cvc::UChar);
  EXPECT_TRUE(same_tree(cvc::compute_contour_tree(u, opt), cvc::compute_contour_tree(u)));

  // On [0,3] the minimum at 1.0235294f maps to 86.99999988 in double, which the float round trip
  // turns into bin 87 (a direct truncation would say 86).
  field line = make_field(4, 1, 1);
  line.f = {0.f, 3.f, 1.0235294103622437f, 3.f};
  const contour_tree tl = tree_of(line, opt);
  ASSERT_TRUE(valid_tree(tl));
  bool found = false;
  for (const auto &nd : tl.nodes)
    if (nd.voxel == 2) {
      found = true;
      EXPECT_DOUBLE_EQ(nd.value, 87.0 * 3.0 / 255.0);
    }
  EXPECT_TRUE(found);
}

TEST_F(ContourTreeVoxels, TwoBlobs) {
  const field g = two_blobs();
  const contour_tree t = cvc::compute_contour_tree(typed_voxels<double>(ctx, g, cvc::Double));
  ASSERT_TRUE(valid_tree(t, &g));
  ASSERT_EQ(count_kind(t, kind::maximum), 2u);
  ASSERT_EQ(count_kind(t, kind::join), 1u);
  EXPECT_EQ(count_kind(t, kind::join_split), 0u);
  std::vector<double> peaks;
  double saddle = 0.0, below = -std::numeric_limits<double>::infinity();
  for (const auto &nd : t.nodes)
    if (nd.kind == kind::maximum)
      peaks.push_back(nd.value);
    else if (nd.kind == kind::join)
      saddle = nd.value;
  for (const auto &nd : t.nodes)
    if (nd.value < saddle)
      below = std::max(below, nd.value);
  std::sort(peaks.begin(), peaks.end());
  EXPECT_LT(saddle, peaks[0]);
  EXPECT_EQ(t.components_at((saddle + peaks[0]) / 2), 2u);
  EXPECT_EQ(t.components_at((saddle + below) / 2), 1u);
  EXPECT_EQ(t.components_at((peaks[0] + peaks[1]) / 2), 1u);
  EXPECT_EQ(t.components_at(peaks[1]), 1u); // the top: the one arc ending there
  EXPECT_EQ(t.arcs_at((saddle + peaks[0]) / 2).size(), 2u);
  EXPECT_TRUE(matches_sampled_level_sets(t, g, 8));
}

// ---- simplification -------------------------------------------------------------------------

namespace {
// Leaves that simplify_contour_tree may still prune (Carr's rule: the saddle keeps another arc
// on the leaf's side, and the arc is not the last), with their spans.
std::vector<double> prunable_spans(const contour_tree &t) {
  const std::size_t m = t.nodes.size();
  std::vector<int> up(m, 0), dn(m, 0);
  for (const auto &a : t.arcs) {
    ++dn[a.hi];
    ++up[a.lo];
  }
  std::vector<double> out;
  for (const auto &a : t.arcs) {
    const bool hi_leaf = up[a.hi] + dn[a.hi] == 1, lo_leaf = up[a.lo] + dn[a.lo] == 1;
    if (hi_leaf && lo_leaf)
      continue;
    const double span = t.nodes[a.hi].value - t.nodes[a.lo].value;
    if (hi_leaf && up[a.lo] >= 2)
      out.push_back(span);
    if (lo_leaf && dn[a.hi] >= 2)
      out.push_back(span);
  }
  return out;
}

::testing::AssertionResult subset_of(const contour_tree &s, const contour_tree &t) {
  std::set<std::pair<cvc::uint64, double>> keep;
  for (const auto &nd : t.nodes)
    keep.insert({nd.voxel, nd.value});
  for (const auto &nd : s.nodes)
    if (!keep.count({nd.voxel, nd.value}))
      return ::testing::AssertionFailure() << "node at voxel " << nd.voxel << " is new";
  return ::testing::AssertionSuccess();
}

field noisy_field(std::uint32_t seed, int side) {
  field g = make_field(side, side, side);
  std::mt19937 rng(seed);
  fill_smooth(g, rng, 6, false);
  for (float &v : g.f)
    v += 0.05f * float(rng() % 1000) / 1000.f;
  return g;
}

contour_tree noisy_tree(std::uint32_t seed, int side) { return tree_of(noisy_field(seed, side)); }

// A simplification of a 3D field's tree: still a valid tree of that field (extrema are leaves,
// so each prune cancelled an extremum-saddle pair) made of nodes of the original.
::testing::AssertionResult valid_simplification(const contour_tree &s, const contour_tree &t,
                                                const field &g) {
  ::testing::AssertionResult r = valid_tree(s, &g);
  if (!r)
    return r;
  return subset_of(s, t);
}
} // namespace

TEST(ContourTreeSimplify, ZeroThresholdKeepsTheTree) {
  const contour_tree t = noisy_tree(1, 10);
  EXPECT_TRUE(same_tree(cvc::simplify_contour_tree(t, 0.0), t));
  EXPECT_TRUE(same_tree(cvc::simplify_contour_tree_to(t, t.nodes.size()), t));
}

TEST(ContourTreeSimplify, ThresholdPrunesShortLeaves) {
  const field g = noisy_field(2, 14);
  const contour_tree t = tree_of(g);
  ASSERT_GT(t.nodes.size(), 20u);
  std::size_t last = t.nodes.size();
  for (double p : {0.01, 0.05, 0.2, 0.5, 2.0}) {
    const contour_tree s = cvc::simplify_contour_tree(t, p);
    ASSERT_TRUE(valid_simplification(s, t, g)) << "p " << p;
    EXPECT_LE(s.nodes.size(), last) << "p " << p;
    last = s.nodes.size();
    for (double span : prunable_spans(s))
      EXPECT_GE(span, p * (t.value_max - t.value_min)) << "p " << p;
    EXPECT_EQ(s.value_min, t.value_min);
    EXPECT_EQ(s.value_max, t.value_max);
    EXPECT_EQ(s.dims, t.dims);
  }
  // Past the whole range only the last arc is left.
  EXPECT_EQ(cvc::simplify_contour_tree(t, 2.0).nodes.size(), 2u);
}

TEST(ContourTreeSimplify, NodeBudget) {
  for (std::uint32_t seed = 3; seed < 6; ++seed) {
    const field g = noisy_field(seed, 16);
    const contour_tree t = tree_of(g);
    ASSERT_GT(t.nodes.size(), 60u);
    for (std::size_t budget :
         {std::size_t(50), std::size_t(10), std::size_t(3), std::size_t(2), std::size_t(0)}) {
      const contour_tree s = cvc::simplify_contour_tree_to(t, budget);
      ASSERT_TRUE(valid_simplification(s, t, g)) << "budget " << budget;
      // Each prune removes the leaf and maybe its saddle, so the count lands on the budget or one
      // under it, never under the last arc.
      EXPECT_LE(s.nodes.size(), std::max<std::size_t>(budget, 2)) << "budget " << budget;
      EXPECT_GE(s.nodes.size() + 1, std::max<std::size_t>(budget, 2)) << "budget " << budget;
    }
  }
}

TEST(ContourTreeSimplify, TwoBlobsLoseTheSmallerPeak) {
  const field g = two_blobs();
  const contour_tree t = tree_of(g);
  double saddle = 0.0, low_peak = std::numeric_limits<double>::infinity();
  for (const auto &nd : t.nodes) {
    if (nd.kind == kind::join)
      saddle = nd.value;
    if (nd.kind == kind::maximum)
      low_peak = std::min(low_peak, nd.value);
  }
  const double range = t.value_max - t.value_min, persistence = (low_peak - saddle) / range;
  const contour_tree keep = cvc::simplify_contour_tree(t, persistence * 0.9);
  EXPECT_EQ(count_kind(keep, kind::maximum), 2u);
  const contour_tree lose = cvc::simplify_contour_tree(t, persistence * 1.1);
  ASSERT_TRUE(valid_simplification(lose, t, g));
  EXPECT_EQ(count_kind(lose, kind::maximum), 1u);
  EXPECT_EQ(count_kind(lose, kind::join), 0u);
  EXPECT_EQ(lose.components_at((saddle + low_peak) / 2), 1u);
}

// ---- layout ---------------------------------------------------------------------------------

TEST(ContourTreeLayout, DeterministicAndNormalized) {
  const contour_tree t = noisy_tree(9, 10);
  ASSERT_GT(t.nodes.size(), 4u);
  const std::vector<float> a = cvc::layout_contour_tree(t, 7), b = cvc::layout_contour_tree(t, 7);
  ASSERT_EQ(a.size(), t.nodes.size());
  EXPECT_EQ(a, b);
  for (float x : a) {
    EXPECT_GE(x, 0.f);
    EXPECT_LE(x, 1.f);
  }
  EXPECT_EQ(*std::min_element(a.begin(), a.end()), 0.f);
  EXPECT_EQ(*std::max_element(a.begin(), a.end()), 1.f);
  EXPECT_NE(cvc::layout_contour_tree(t, 8), a);
  EXPECT_EQ(cvc::layout_contour_tree(t), cvc::layout_contour_tree(t, 1));

  contour_tree one;
  one.nodes.emplace_back();
  EXPECT_EQ(cvc::layout_contour_tree(one), std::vector<float>{0.5f});
  EXPECT_TRUE(cvc::layout_contour_tree(contour_tree{}).empty());
}

TEST(ContourTreeLayout, GoldenFan) {
  // The 0 5 1 4 2 line's tree, by hand. Pins the generator (std::minstd_rand, seed 1, raw % 60),
  // the visiting order (up then down neighbours, depth first from the top) and the cot table, so
  // a platform or library change that moved the drawing would show here. Draws: theta 91 for
  // node 0 and 114 for node 1 (both from the root, node 4), 66 for node 3, 97 for node 2.
  contour_tree t;
  const double values[5] = {0, 1, 2, 4, 5};
  const cvc::uint64 voxels[5] = {0, 2, 4, 3, 1};
  for (int i = 0; i < 5; ++i) {
    contour_tree::node nd;
    nd.voxel = voxels[i];
    nd.value = values[i];
    t.nodes.push_back(nd);
  }
  const std::uint32_t arcs[4][2] = {{4, 0}, {4, 1}, {3, 1}, {3, 2}};
  for (const auto &a : arcs) {
    contour_tree::arc arc;
    arc.hi = a[0];
    arc.lo = a[1];
    t.arcs.push_back(arc);
  }
  const std::vector<float> x = cvc::layout_contour_tree(t, 1);
  ASSERT_EQ(x.size(), 5u);
  EXPECT_FLOAT_EQ(x[0], 0.025958035f);
  EXPECT_FLOAT_EQ(x[1], 0.52969205f);
  EXPECT_FLOAT_EQ(x[2], 1.0f);
  EXPECT_FLOAT_EQ(x[3], 0.92696112f);
  EXPECT_FLOAT_EQ(x[4], 0.0f);
}

// ---- resumable building ---------------------------------------------------------------------

namespace {

using cvc::contour_tree_builder;
using cvc::contour_tree_simplifier;

bool identical(const contour_tree &a, const contour_tree &b) {
  return same_tree(a, b) && a.value_min == b.value_min && a.value_max == b.value_max;
}

const std::vector<std::string> kBuilderPhases = {"preparing",  "sorting", "join tree",
                                                 "split tree", "merging", "done"};
const std::vector<std::string> kSimplifierPhases = {"preparing", "pruning", "rebuilding", "done"};

// A fixed budget per step, or (seed != 0) a fresh random one in [1, 5000] every step.
struct schedule {
  std::size_t fixed = 0;
  std::uint32_t seed = 0;
  std::string describe() const {
    return seed ? "random budgets, seed " + std::to_string(seed)
                : "budget " + std::to_string(fixed);
  }
  std::size_t most() const { return seed ? 5000 : fixed; } // the largest budget of any step
};

const std::vector<schedule> &schedules() {
  static const std::vector<schedule> s = {{1, 0}, {7, 0},  {64, 0}, {4096, 0},
                                          {0, 0}, {0, 17}, {0, 99}};
  return s;
}

// What a stepped run saw: per phase, the steps that worked in it (every phase from the one
// reported before the step to the one reported after); `trouble` names the first broken rule.
struct stepped {
  contour_tree tree;
  std::size_t steps = 0;
  std::vector<std::size_t> touched;
  std::string trouble;
};

template <class Machine>
stepped step_through(Machine &m, const std::vector<std::string> &phases, const schedule &s) {
  stepped out;
  out.touched.assign(phases.size(), 0);
  std::mt19937 rng(s.seed);
  auto index = [&phases](const char *p) {
    const auto it = std::find(phases.begin(), phases.end(), std::string(p));
    return it == phases.end() ? -1 : int(it - phases.begin());
  };
  auto complain = [&out](const std::string &what) {
    if (out.trouble.empty())
      out.trouble = "step " + std::to_string(out.steps) + ": " + what;
  };
  int before = index(m.phase());
  double prev = m.progress();
  if (before < 0)
    complain(std::string("unknown phase ") + m.phase());
  if (!(prev >= 0.0 && prev <= 1.0))
    complain("progress " + std::to_string(prev) + " before the first step");
  for (;;) {
    const std::size_t budget = s.seed ? 1 + rng() % 5000 : s.fixed;
    const bool finished = m.step(budget);
    ++out.steps;
    const int after = index(m.phase());
    if (after < 0)
      complain(std::string("unknown phase ") + m.phase());
    else if (after < before)
      complain(std::string("the phase went back to ") + m.phase());
    else
      for (int p = before; p <= after; ++p)
        ++out.touched[std::size_t(p)];
    const double pr = m.progress();
    if (pr < prev || pr > 1.0)
      complain("progress went from " + std::to_string(prev) + " to " + std::to_string(pr));
    if (finished != m.done())
      complain("step() and done() disagree");
    if (!finished && pr == 1.0)
      complain("progress 1 before the end");
    prev = pr;
    before = after;
    if (finished)
      break;
  }
  if (prev != 1.0)
    complain("finished at progress " + std::to_string(prev));
  if (std::string(m.phase()) != "done")
    complain(std::string("finished in phase ") + m.phase());
  if (!m.step(s.fixed))
    complain("a step after the end returned false");
  out.tree = m.take();
  return out;
}

// A step does at most `budget` units, so a phase with units[p] units of work must be worked on
// by at least ceil(units[p] / budget) steps; a phase done in one go (an unsliced pass) fails here.
::testing::AssertionResult sliced(const stepped &r, const std::vector<std::string> &phases,
                                  const std::vector<std::size_t> &units, std::size_t budget) {
  if (!r.trouble.empty())
    return ::testing::AssertionFailure() << r.trouble;
  if (budget == 0)
    return r.steps == 1 ? ::testing::AssertionSuccess()
                        : ::testing::AssertionFailure() << "budget 0 took " << r.steps << " steps";
  for (std::size_t p = 0; p < units.size(); ++p) {
    const std::size_t need = (units[p] + budget - 1) / budget;
    if (r.touched[p] < need)
      return ::testing::AssertionFailure()
             << phases[p] << " was worked on by " << r.touched[p] << " steps for at least "
             << units[p] << " units at budget " << budget << " (expected >= " << need << ")";
  }
  return ::testing::AssertionSuccess();
}

// Lower bounds on the builder's units per phase (preparing, sorting, join, split, merging) for a
// field of n_in voxels built on n: the NaN scan of a floating-point input, the resampling, the
// range and the quantizing visit every voxel once; the sort counts the keys, writes its first
// (new) buffer once in order and moves the keys at least once; the join tree first writes its new
// arrays once in order, then each sweep visits every vertex; the merge flags, peels and reduces
// every vertex.
std::vector<std::size_t> builder_units(std::size_t n_in, const contour_tree &ref, bool float_input,
                                       bool quantize) {
  const std::size_t n = std::size_t(ref.dims[0] * ref.dims[1] * ref.dims[2]);
  const bool resampled = n != n_in, constant = ref.arcs.empty();
  std::vector<std::size_t> u(5, 0);
  u[0] = (float_input ? n_in : 0) + (resampled ? n : 0) + n + (quantize && !constant ? n : 0);
  if (!constant) {
    u[1] = 3 * n;
    u[2] = 2 * n;
    u[3] = n;
    u[4] = 4 * n - 1;
  }
  return u;
}

std::vector<contour_tree_options> stepping_options() {
  std::vector<contour_tree_options> out(4);
  out[1].max_dim = 12;
  out[2].quantize_u8 = true;
  out[3].max_dim = 12;
  out[3].quantize_u8 = true;
  return out;
}

std::string describe(const contour_tree_options &opt) {
  return "max_dim " + std::to_string(opt.max_dim) + (opt.quantize_u8 ? " quantize_u8" : "");
}

std::vector<std::pair<std::string, field>> stepping_fields() {
  std::mt19937 rng(2026);
  std::vector<std::pair<std::string, field>> out;
  field noise = make_field(24, 20, 18);
  fill_noise(noise, rng, 256);
  out.emplace_back("noise", noise);
  field levels = make_field(21, 19, 17);
  fill_noise(levels, rng, 3);
  out.emplace_back("3-level noise", levels);
  field smooth = make_field(26, 22, 15);
  fill_smooth(smooth, rng, 8, false);
  out.emplace_back("smooth", smooth);
  field plateaus = make_field(26, 22, 15);
  fill_smooth(plateaus, rng, 8, true);
  out.emplace_back("smooth with plateaus", plateaus);
  field wide = make_field(17, 16, 15);
  fill_smooth(wide, rng, 5, false);
  for (float &v : wide.f)
    v = v * 1e6f - 3e5f;
  out.emplace_back("wide signed floats", wide);
  field line = make_field(9000, 1, 1);
  fill_noise(line, rng, 1000);
  out.emplace_back("line", line);
  field image = make_field(1, 96, 90);
  fill_smooth(image, rng, 10, false);
  out.emplace_back("image", image);
  field slice = make_field(40, 36, 1);
  fill_noise(slice, rng, 5);
  out.emplace_back("thin slab", extrude(slice, 1, 3));
  out.emplace_back("constant", make_field(20, 20, 20, 2.5f));
  out.emplace_back("one voxel", make_field(1, 1, 1, 7.f));
  field two = make_field(2, 1, 1);
  two.f = {1.f, 0.f};
  out.emplace_back("two voxels", two);
  return out;
}

// A strictly increasing map of level L in [0, 200) into T, spread over most of T's range (so the
// radix sort runs its full complement of passes, skipping the ones a digit does not vary in), or
// a narrow one (one counting pass).
template <class T> T level_value(int L, bool wide) {
  if constexpr (std::is_same_v<T, unsigned char>)
    return T(L);
  else if constexpr (std::is_same_v<T, signed char>)
    return T(L - 100);
  else if constexpr (std::is_same_v<T, unsigned short>)
    return T(wide ? L * 300 : L);
  else if constexpr (std::is_same_v<T, unsigned int>)
    return wide ? T((unsigned(L / 16) << 22) | unsigned(L % 16)) : T(L); // a middle digit is idle
  else if constexpr (std::is_same_v<T, int>)
    return wide ? T((L - 100) * (1 << 23)) : T(L - 100);
  else if constexpr (std::is_same_v<T, float>)
    return wide ? std::ldexp(float(L - 100), 100) : float(L) / 8.f - 12.5f;
  else if constexpr (std::is_same_v<T, double>)
    return wide ? std::ldexp(double(L - 100), 900) : double(L) / 8.0 - 12.5;
  else if constexpr (std::is_same_v<T, cvc::int64>)
    return wide ? T(L - 100) * (cvc::int64(1) << 55) : T(L - 100);
  else
    return wide ? T(L) << 56 : T(L);
}

// `levels` (whole numbers in [0, 200)) mapped into T; zeros of a float field alternate in sign.
template <class T>
cvc::voxels level_voxels(cvc::app &ctx, const field &levels, bool wide, cvc::data_type type) {
  std::vector<T> buf(levels.f.size());
  for (std::size_t i = 0; i < buf.size(); ++i) {
    buf[i] = level_value<T>(int(levels.f[i]), wide);
    if constexpr (std::is_floating_point_v<T>)
      if (buf[i] == T(0) && i % 2)
        buf[i] = -T(0);
  }
  return cvc::voxels(ctx, buf.data(), cvc::dimension(levels.nx, levels.ny, levels.nz), type);
}

std::vector<cvc::voxels> every_type(cvc::app &ctx, const field &levels, bool wide) {
  return {level_voxels<unsigned char>(ctx, levels, wide, cvc::UChar),
          level_voxels<unsigned short>(ctx, levels, wide, cvc::UShort),
          level_voxels<unsigned int>(ctx, levels, wide, cvc::UInt),
          level_voxels<float>(ctx, levels, wide, cvc::Float),
          level_voxels<double>(ctx, levels, wide, cvc::Double),
          level_voxels<cvc::uint64>(ctx, levels, wide, cvc::UInt64),
          level_voxels<signed char>(ctx, levels, wide, cvc::Char),
          level_voxels<int>(ctx, levels, wide, cvc::Int),
          level_voxels<cvc::int64>(ctx, levels, wide, cvc::Int64)};
}

// The tree's shape: node voxels and kinds, and arcs (values differ between level maps).
bool same_shape(const contour_tree &a, const contour_tree &b) {
  if (a.dims != b.dims || a.nodes.size() != b.nodes.size() || a.arcs.size() != b.arcs.size())
    return false;
  for (std::size_t i = 0; i < a.nodes.size(); ++i)
    if (a.nodes[i].voxel != b.nodes[i].voxel || a.nodes[i].kind != b.nodes[i].kind)
      return false;
  for (std::size_t i = 0; i < a.arcs.size(); ++i)
    if (a.arcs[i].hi != b.arcs[i].hi || a.arcs[i].lo != b.arcs[i].lo)
      return false;
  return true;
}

// Whole-number levels in [0, 200): smooth bands plus noise, so there are plateaus and many
// critical points.
field level_field(std::uint32_t seed, int nx, int ny, int nz) {
  std::mt19937 rng(seed);
  field g = make_field(nx, ny, nz);
  fill_smooth(g, rng, 9, false);
  const auto [lo, hi] = std::minmax_element(g.f.begin(), g.f.end());
  const float mn = *lo, mx = *hi;
  for (float &v : g.f)
    v = std::min(199.f, std::floor(150.f * (v - mn) / (mx - mn)) + float(rng() % 50));
  return g;
}

} // namespace

TEST(ContourTreeBuilder, SteppingMatchesOneShot) {
  for (const auto &[name, g] : stepping_fields()) {
    for (const contour_tree_options &base : stepping_options()) {
      const contour_tree ref = tree_of(g, base);
      if (!ref.arcs.empty())
        ASSERT_TRUE(valid_tree(ref)) << name;
      const std::vector<std::size_t> units =
          builder_units(std::size_t(g.n()), ref, true, base.quantize_u8);
      for (const schedule &s : schedules()) {
        contour_tree_options opt = base;
        std::size_t calls = 0;
        opt.progress = [&calls](double) {
          ++calls;
          return true;
        };
        contour_tree_builder b(g.f.data(), g.nx, g.ny, g.nz, opt);
        const stepped r = step_through(b, kBuilderPhases, s);
        const std::string what = name + ", " + describe(base) + ", " + s.describe();
        EXPECT_TRUE(identical(r.tree, ref)) << what;
        EXPECT_TRUE(sliced(r, kBuilderPhases, units, s.most())) << what;
        EXPECT_EQ(calls, r.steps) << what << ": opt.progress is called once per step";
      }
    }
  }
}

TEST_F(ContourTreeVoxels, SteppingMatchesOneShotForEveryType) {
  const field levels = level_field(31, 20, 18, 16);
  for (const bool wide : {false, true}) {
    for (const cvc::voxels &vol : every_type(ctx, levels, wide)) {
      const bool floating = vol.voxelType() == cvc::Float || vol.voxelType() == cvc::Double;
      for (const contour_tree_options &opt : stepping_options()) {
        const contour_tree ref = cvc::compute_contour_tree(vol, opt);
        ASSERT_TRUE(valid_tree(ref)) << vol.voxelTypeStr();
        const std::vector<std::size_t> units =
            builder_units(std::size_t(levels.n()), ref, floating,
                          opt.quantize_u8 && vol.voxelType() != cvc::UChar);
        for (const schedule &s : schedules()) {
          contour_tree_builder b(vol, opt);
          const stepped r = step_through(b, kBuilderPhases, s);
          const std::string what = std::string(vol.voxelTypeStr()) + (wide ? " wide, " : ", ") +
                                   describe(opt) + ", " + s.describe();
          EXPECT_TRUE(identical(r.tree, ref)) << what;
          EXPECT_TRUE(sliced(r, kBuilderPhases, units, s.most())) << what;
        }
      }
    }
  }
}

TEST_F(ContourTreeVoxels, CountingSortBinsAreSliced) {
  // 16-bit data spanning 60001 values sort in one counting pass over 60001 bins: clearing and
  // summing them is 120002 units, far more than the 1000 voxels, and is sliced like the rest.
  field g = make_field(10, 10, 10);
  std::mt19937 rng(6);
  fill_noise(g, rng, 60001);
  g.f[0] = 0.f;
  g.f[1] = 60000.f;
  const cvc::voxels vol = typed_voxels<unsigned short>(ctx, g, cvc::UShort);
  const contour_tree ref = cvc::compute_contour_tree(vol);
  ASSERT_TRUE(valid_tree(ref, &g));
  std::vector<std::size_t> units = builder_units(1000, ref, false, false);
  units[1] += 2 * 60001;
  for (std::size_t budget : {std::size_t(64), std::size_t(4096)}) {
    contour_tree_builder b(vol);
    const stepped r = step_through(b, kBuilderPhases, schedule{budget, 0});
    EXPECT_TRUE(identical(r.tree, ref)) << budget;
    EXPECT_TRUE(sliced(r, kBuilderPhases, units, budget)) << budget;
  }
}

TEST_F(ContourTreeVoxels, WideRangesSortExactly) {
  // The tree depends only on the (value, voxel) order, so every strictly increasing map of the
  // same levels -- narrow or spread over a type's whole range, signed, with -0.0 beside 0.0 --
  // must give the same shape: this pins the radix sort's order for every key width and pass plan.
  for (std::uint32_t seed : {41u, 42u}) {
    const field levels = level_field(seed, 19, 17, 13);
    const contour_tree want = tree_of(levels);
    ASSERT_TRUE(valid_tree(want, &levels));
    for (const bool wide : {false, true})
      for (const cvc::voxels &vol : every_type(ctx, levels, wide))
        EXPECT_TRUE(same_shape(cvc::compute_contour_tree(vol), want))
            << vol.voxelTypeStr() << (wide ? " wide" : " narrow") << " seed " << seed;
  }
}

TEST(ContourTreeBuilder, NothingHappensUntilTheFirstStep) {
  // Construction only stores its arguments: bad input throws from the first step, and a grid of
  // 2^32 voxels is refused before any voxel is read.
  float dummy = 0.f;
  contour_tree_builder huge(&dummy, 65536, 65536, 1);
  EXPECT_FALSE(huge.done());
  EXPECT_EQ(huge.progress(), 0.0);
  EXPECT_STREQ(huge.phase(), "preparing");
  EXPECT_THROW(huge.step(1), cvc::contour_tree_error);
  EXPECT_TRUE(huge.done());
  EXPECT_STREQ(huge.phase(), "failed");
  EXPECT_THROW(huge.step(1), cvc::contour_tree_error);
  EXPECT_THROW(huge.take(), cvc::contour_tree_error);

  contour_tree_builder null(nullptr, 2, 2, 2);
  EXPECT_THROW(null.step(0), cvc::contour_tree_error);
  contour_tree_builder empty(&dummy, 0, 1, 1);
  EXPECT_THROW(empty.step(0), cvc::contour_tree_error);

  // The NaN scan is sliced too: at one unit per step it reaches voxel 13 on the 14th step.
  field g = make_field(3, 3, 3, 1.f);
  g.f[13] = std::numeric_limits<float>::quiet_NaN();
  contour_tree_builder nan(g.f.data(), 3, 3, 3);
  std::size_t steps = 0;
  try {
    while (!nan.step(1))
      ++steps;
    ADD_FAILURE() << "no throw";
  } catch (const cvc::contour_tree_error &) {
  }
  EXPECT_EQ(steps, 13u);
  EXPECT_TRUE(nan.done());
  EXPECT_THROW(nan.step(1), cvc::contour_tree_error);
}

TEST(ContourTreeBuilder, ProgressCallbackCancels) {
  field g = make_field(30, 30, 30);
  std::mt19937 rng(8);
  fill_smooth(g, rng, 10, false);
  for (std::size_t cancel_at : {std::size_t(1), std::size_t(2), std::size_t(9), std::size_t(40)}) {
    contour_tree_options opt;
    std::size_t calls = 0;
    opt.progress = [&calls, cancel_at](double) { return ++calls < cancel_at; };
    contour_tree_builder b(g.f.data(), g.nx, g.ny, g.nz, opt);
    std::size_t steps = 0;
    bool threw = false;
    try {
      while (!b.step(4096))
        ++steps;
    } catch (const cvc::contour_tree_error &) {
      threw = true;
    }
    EXPECT_TRUE(threw) << cancel_at;
    EXPECT_EQ(calls, cancel_at);
    EXPECT_EQ(steps + 1, cancel_at);
    EXPECT_TRUE(b.done());
    EXPECT_STREQ(b.phase(), "failed");
    const std::size_t before = calls;
    EXPECT_THROW(b.step(4096), cvc::contour_tree_error);
    EXPECT_THROW(b.take(), cvc::contour_tree_error);
    EXPECT_EQ(calls, before) << "a failed builder reports nothing more";
  }
  // A cancel on the finishing step still fails the build.
  contour_tree_options opt;
  opt.progress = [](double p) { return p < 1.0; };
  contour_tree_builder b(g.f.data(), g.nx, g.ny, g.nz, opt);
  EXPECT_THROW(b.step(0), cvc::contour_tree_error);
  EXPECT_THROW(b.take(), cvc::contour_tree_error);
  EXPECT_THROW(tree_of(g, opt), cvc::contour_tree_error);
}

TEST_F(ContourTreeVoxels, DestroyingMidwayIsClean) {
  // Stop after a spread of step counts, so in every phase, and destroy; the voxels stay intact and
  // a later build of them is unaffected. (A leak check runs this under a counting allocator
  // outside the suite.)
  const field levels = level_field(5, 18, 17, 16);
  for (const cvc::voxels &vol : every_type(ctx, levels, true)) {
    const contour_tree ref = cvc::compute_contour_tree(vol);
    contour_tree_options opt;
    opt.max_dim = vol.voxelType() == cvc::Float ? 15 : 0;
    opt.quantize_u8 = vol.voxelType() == cvc::Int;
    std::size_t total = 0;
    {
      contour_tree_builder b(vol, opt);
      while (!b.step(500))
        ++total;
    }
    for (std::size_t stop = 0; stop <= total; stop += 1 + total / 23) {
      contour_tree_builder b(vol, opt);
      for (std::size_t s = 0; s < stop; ++s)
        ASSERT_FALSE(b.step(500));
    }
    EXPECT_TRUE(identical(cvc::compute_contour_tree(vol), ref)) << vol.voxelTypeStr();
  }
}

TEST(ContourTreeBuilder, MovesAndInterleaves) {
  field a = make_field(22, 21, 20), c = make_field(30, 1, 40);
  std::mt19937 rng(77);
  fill_smooth(a, rng, 9, false);
  fill_noise(c, rng, 40);
  const contour_tree ra = tree_of(a), rc = tree_of(c);

  contour_tree_builder b(a.f.data(), a.nx, a.ny, a.nz);
  for (int s = 0; s < 20; ++s)
    ASSERT_FALSE(b.step(1000));
  const double p = b.progress();
  contour_tree_builder moved(std::move(b));
  EXPECT_EQ(moved.progress(), p);
  // The moved-from builder is empty and says so.
  EXPECT_TRUE(b.done());
  EXPECT_TRUE(b.step(1));
  EXPECT_EQ(b.progress(), 1.0);
  EXPECT_STREQ(b.phase(), "done");
  EXPECT_THROW(b.take(), cvc::contour_tree_error);

  // Move-assign over a half-built builder (its work is dropped), and into the moved-from one.
  contour_tree_builder other(c.f.data(), c.nx, c.ny, c.nz);
  for (int s = 0; s < 7; ++s)
    ASSERT_FALSE(other.step(333));
  other = std::move(moved);
  b = contour_tree_builder(c.f.data(), c.nx, c.ny, c.nz);
  // Interleave them (and three more in a vector that reallocates): builders share nothing.
  std::vector<contour_tree_builder> more;
  more.emplace_back(a.f.data(), a.nx, a.ny, a.nz);
  more.emplace_back(c.f.data(), c.nx, c.ny, c.nz);
  more.emplace_back(a.f.data(), a.nx, a.ny, a.nz);
  bool all = false;
  while (!all) {
    all = other.step(777);
    all = b.step(555) && all;
    for (contour_tree_builder &m : more)
      all = m.step(4096) && all;
  }
  EXPECT_TRUE(identical(other.take(), ra));
  EXPECT_TRUE(identical(b.take(), rc));
  EXPECT_TRUE(identical(more[0].take(), ra));
  EXPECT_TRUE(identical(more[1].take(), rc));
  EXPECT_TRUE(identical(more[2].take(), ra));
  EXPECT_THROW(more[0].take(), cvc::contour_tree_error); // moved out once
}

namespace {
// Trees to simplify: noisy 3D fields, a 2D noise image (many join_split saddles) and a line
// (extrema with two arcs), plus a copy with an empty stored value range (the simplifier then
// measures the nodes' range).
std::vector<std::pair<std::string, contour_tree>> simplifier_trees() {
  std::vector<std::pair<std::string, contour_tree>> out;
  out.emplace_back("noisy 12^3", noisy_tree(21, 12));
  out.emplace_back("noisy 16^3", noisy_tree(22, 16));
  field image = make_field(48, 40, 1);
  std::mt19937 rng(23);
  fill_noise(image, rng, 64);
  out.emplace_back("noise image", tree_of(image));
  field line = make_field(600, 1, 1);
  fill_noise(line, rng, 100);
  out.emplace_back("line", tree_of(line));
  contour_tree flat = out[0].second;
  flat.value_min = flat.value_max = 0.0;
  out.emplace_back("no stored range", flat);
  return out;
}

// Lower bounds on the simplifier's units per phase: the setup visits every node four times (five
// when it must measure the value range itself) and every arc twice; a prune is one pop and
// removes at most two nodes; the rebuild visits every node and arc once.
std::vector<std::size_t> simplifier_units(const contour_tree &t, const contour_tree &s,
                                          bool measures_range) {
  return {(measures_range ? 5 : 4) * t.nodes.size() + 2 * t.arcs.size(),
          (t.nodes.size() - s.nodes.size()) / 2, t.nodes.size() + t.arcs.size()};
}
} // namespace

TEST(ContourTreeSimplifier, SteppingMatchesOneShot) {
  for (const auto &[name, t] : simplifier_trees()) {
    ASSERT_GT(t.nodes.size(), 30u) << name;
    for (double p : {0.0, 0.01, 0.05, 0.3, 2.0}) {
      const contour_tree ref = cvc::simplify_contour_tree(t, p);
      for (const schedule &s : schedules()) {
        contour_tree_simplifier m = contour_tree_simplifier::by_persistence(t, p);
        const stepped r = step_through(m, kSimplifierPhases, s);
        const std::string what = name + ", persistence " + std::to_string(p) + ", " + s.describe();
        EXPECT_TRUE(identical(r.tree, ref)) << what;
        if (p > 0.0)
          EXPECT_TRUE(sliced(r, kSimplifierPhases,
                             simplifier_units(t, ref, !(t.value_max - t.value_min > 0.0)),
                             s.most()))
              << what;
        else
          EXPECT_EQ(r.steps, 1u) << what << ": nothing to prune";
      }
    }
    for (std::size_t k :
         {std::size_t(0), std::size_t(2), std::size_t(10), std::size_t(25), t.nodes.size()}) {
      const contour_tree ref = cvc::simplify_contour_tree_to(t, k);
      for (const schedule &s : schedules()) {
        contour_tree_simplifier m = contour_tree_simplifier::to_nodes(t, k);
        const stepped r = step_through(m, kSimplifierPhases, s);
        const std::string what = name + ", to " + std::to_string(k) + " nodes, " + s.describe();
        EXPECT_TRUE(identical(r.tree, ref)) << what;
        if (k < t.nodes.size())
          EXPECT_TRUE(sliced(r, kSimplifierPhases, simplifier_units(t, ref, false), s.most()))
              << what;
        else
          EXPECT_EQ(r.steps, 1u) << what << ": nothing to prune";
      }
    }
  }
}

TEST(ContourTreeSimplifier, HighDegreeSaddle) {
  // A minimum, one saddle and 300 maxima on it: the saddle's arc list shrinks by one per prune,
  // so pruning must not rescan it. The most persistent maxima stay.
  contour_tree t;
  auto add = [&t](double v, kind k) {
    contour_tree::node nd;
    nd.voxel = t.nodes.size();
    nd.value = v;
    nd.kind = k;
    t.nodes.push_back(nd);
  };
  add(0.0, kind::minimum);
  add(1.0, kind::join);
  for (int i = 0; i < 300; ++i)
    add(1.0 + 0.001 * (i + 1), kind::maximum);
  t.value_min = 0.0;
  t.value_max = t.nodes.back().value;
  for (std::uint32_t i = 1; i < t.nodes.size(); ++i) {
    contour_tree::arc a;
    a.hi = i;
    a.lo = i == 1 ? 0 : 1;
    t.arcs.push_back(a);
  }
  const contour_tree ten = cvc::simplify_contour_tree_to(t, 10);
  ASSERT_TRUE(valid_tree(ten));
  ASSERT_EQ(ten.nodes.size(), 10u);
  for (std::size_t i = 2; i < 10; ++i)
    EXPECT_EQ(ten.nodes[i].voxel, 301 - 8 + (i - 1));
  const contour_tree two = cvc::simplify_contour_tree_to(t, 2);
  ASSERT_EQ(two.nodes.size(), 2u);
  EXPECT_EQ(two.nodes[0].voxel, 0u);
  EXPECT_EQ(two.nodes[1].voxel, 301u);
  for (const schedule &s : schedules()) {
    contour_tree_simplifier m = contour_tree_simplifier::to_nodes(t, 10);
    EXPECT_TRUE(identical(step_through(m, kSimplifierPhases, s).tree, ten)) << s.describe();
  }
}

TEST(ContourTreeSimplifier, NoWorkMovesAndErrors) {
  const contour_tree t = noisy_tree(24, 10);
  contour_tree_simplifier none = contour_tree_simplifier::by_persistence(t, 0.0);
  EXPECT_TRUE(none.done());
  EXPECT_STREQ(none.phase(), "done");
  EXPECT_EQ(none.progress(), 1.0);
  EXPECT_TRUE(none.step(1));
  EXPECT_TRUE(identical(none.take(), t));
  EXPECT_THROW(none.take(), cvc::contour_tree_error);

  contour_tree_simplifier a = contour_tree_simplifier::to_nodes(t, 5);
  EXPECT_FALSE(a.done());
  EXPECT_THROW(a.take(), cvc::contour_tree_error); // not ready
  for (int s = 0; s < 10; ++s)
    ASSERT_FALSE(a.step(16));
  contour_tree_simplifier b = std::move(a);
  EXPECT_TRUE(a.done());
  EXPECT_TRUE(a.step(1));
  EXPECT_STREQ(a.phase(), "done");
  EXPECT_THROW(a.take(), cvc::contour_tree_error);
  a = std::move(b);
  while (!a.step(16)) {
  }
  EXPECT_TRUE(identical(a.take(), cvc::simplify_contour_tree_to(t, 5)));

  contour_tree bad = t;
  bad.arcs[bad.arcs.size() / 2].lo = std::uint32_t(bad.nodes.size() + 3);
  contour_tree_simplifier m = contour_tree_simplifier::by_persistence(bad, 0.5);
  bool threw = false;
  try {
    while (!m.step(7)) {
    }
  } catch (const cvc::contour_tree_error &) {
    threw = true;
  }
  EXPECT_TRUE(threw);
  EXPECT_TRUE(m.done());
  EXPECT_STREQ(m.phase(), "failed");
  EXPECT_THROW(m.step(7), cvc::contour_tree_error);
  EXPECT_THROW(m.take(), cvc::contour_tree_error);
  EXPECT_THROW(cvc::simplify_contour_tree(bad, 0.5), cvc::contour_tree_error);
}

// ---- infinite values ------------------------------------------------------------------------

namespace {

// The resampling rule contour_tree_options::max_dim documents, rebuilt here: sample i of an axis
// resampled from `from` to `count` voxels sits at i * (from - 1) / (count - 1) of the source, and
// the eight taps are blended along x, then y, then z.
double expected_blend(double p, double q, double w) {
  const bool pinf = std::isinf(p), qinf = std::isinf(q);
  if (!pinf && !qinf) {
    const double r = p + (q - p) * w; // voxels::resize's arithmetic
    if (std::isfinite(r))
      return r;
    return std::clamp(p * (1.0 - w) + q * w, std::min(p, q), std::max(p, q)); // q - p overflowed
  }
  if (w == 0.0)
    return p; // q has no weight
  if (pinf && qinf)
    return p == q || w <= 0.5 ? p : q; // +inf against -inf: the nearer tap, p at the midpoint
  return pinf ? p : q;                 // an infinity outweighs a finite value
}

// `f` (from[0] x from[1] x from[2], some axis longer than max_dim) resampled; `to` gets its dims.
template <class V>
std::vector<V> expected_resample(const std::vector<V> &f, const std::array<int, 3> &from,
                                 int max_dim, std::array<int, 3> &to) {
  struct tap {
    int i0, i1;
    double w;
  };
  std::array<std::vector<tap>, 3> taps;
  for (int a = 0; a < 3; ++a) {
    to[a] = std::min(from[a], max_dim);
    const double s = to[a] > 1 ? double(from[a] - 1) / double(to[a] - 1) : 0.0;
    for (int i = 0; i < to[a]; ++i) {
      const double x = double(i) * s;
      const int i0 = std::min(int(x), from[a] - 1);
      taps[a].push_back({i0, std::min(i0 + 1, from[a] - 1), x - double(i0)});
    }
  }
  auto at = [&](int i, int j, int k) {
    return double(f[std::size_t(i) + std::size_t(j) * std::size_t(from[0]) +
                    std::size_t(k) * std::size_t(from[0]) * std::size_t(from[1])]);
  };
  std::vector<V> out;
  out.reserve(std::size_t(to[0]) * std::size_t(to[1]) * std::size_t(to[2]));
  for (const tap &c : taps[2])
    for (const tap &b : taps[1])
      for (const tap &a : taps[0]) {
        auto plane = [&](int k) {
          return expected_blend(expected_blend(at(a.i0, b.i0, k), at(a.i1, b.i0, k), a.w),
                                expected_blend(at(a.i0, b.i1, k), at(a.i1, b.i1, k), a.w), b.w);
        };
        out.push_back(static_cast<V>(expected_blend(plane(c.i0), plane(c.i1), c.w)));
      }
  return out;
}

field resampled_field(const field &g, int max_dim) {
  std::array<int, 3> to{};
  field r;
  r.f = expected_resample(g.f, {g.nx, g.ny, g.nz}, max_dim, to);
  r.nx = to[0];
  r.ny = to[1];
  r.nz = to[2];
  return r;
}

cvc::voxels double_voxels(cvc::app &ctx, std::vector<double> buf, const std::array<int, 3> &d) {
  return cvc::voxels(ctx, buf.data(), cvc::dimension(d[0], d[1], d[2]), cvc::Double);
}

bool any_nan(const contour_tree &t) {
  if (std::isnan(t.value_min) || std::isnan(t.value_max))
    return true;
  return std::any_of(t.nodes.begin(), t.nodes.end(),
                     [](const contour_tree::node &nd) { return std::isnan(nd.value); });
}

constexpr float kInf = std::numeric_limits<float>::infinity();

} // namespace

TEST(ContourTree, InfinitiesAreExtremes) {
  // Infinite values are accepted: +inf orders above and -inf below every finite value, ties among
  // them broken by voxel index like any plateau, so the tree has the shape of the same field with
  // finite stand-ins beyond its range, and node values of +-inf.
  std::mt19937 rng(91);
  field g = make_field(9, 8, 7);
  fill_noise(g, rng, 100);
  field standin = g;
  for (int v : {13, 14, 200, 377, 401}) {
    g.f[std::size_t(v)] = kInf;
    standin.f[std::size_t(v)] = 1000.f;
  }
  for (int v : {60, 61, 69, 300}) {
    g.f[std::size_t(v)] = -kInf;
    standin.f[std::size_t(v)] = -1000.f;
  }
  const contour_tree t = tree_of(g);
  ASSERT_TRUE(valid_tree(t, &g));
  EXPECT_TRUE(same_shape(t, tree_of(standin)));
  EXPECT_EQ(t.value_max, double(kInf));
  EXPECT_EQ(t.value_min, -double(kInf));
  EXPECT_EQ(t.nodes.back().value, double(kInf));
  EXPECT_EQ(t.nodes.front().value, -double(kInf));
  // quantize_u8 maps the range onto 0..255, so it alone needs a finite one.
  contour_tree_options q;
  q.quantize_u8 = true;
  EXPECT_THROW(tree_of(g, q), cvc::contour_tree_error);
}

TEST_F(ContourTreeVoxels, ResamplingKeepsInfinitiesAndMakesNoNaN) {
  // max_dim resamples after the NaN scan, so the blend must not make NaN of an infinity (inf - inf
  // and inf * 0 are NaN, and every weight-0 tap -- all of an image's z taps -- multiplied one):
  // the builder's samples are exactly the documented rule's (expected_resample).
  std::mt19937 rng(92);
  contour_tree_options opt;
  // The reported case: a 200x200 image with a +inf voxel (a missing-value marker). At voxel 0 it
  // is the resampled grid's voxel 0 too, which came out NaN, and so did the range ("constant").
  field img = make_field(200, 200, 1);
  fill_smooth(img, rng, 6, false);
  img.f[0] = kInf;
  opt.max_dim = 128;
  const contour_tree ti = tree_of(img, opt);
  ASSERT_FALSE(any_nan(ti));
  ASSERT_TRUE(identical(ti, tree_of(resampled_field(img, 128))));
  EXPECT_EQ(ti.value_max, double(kInf));
  EXPECT_EQ(ti.nodes.back().voxel, 0u);

  // Inside a volume: alone, and in +inf / -inf pairs along each axis that a sample straddles
  // (where both carry weight the nearer tap wins), through the float* form and Float and Double
  // voxels.
  field g = make_field(40, 36, 30);
  fill_smooth(g, rng, 8, false);
  g.f[std::size_t(g.id(20, 17, 15))] = kInf;
  g.f[std::size_t(g.id(7, 30, 3))] = -kInf;
  g.f[std::size_t(g.id(10, 5, 22))] = kInf; // x 10.4 between them: the +inf is nearer
  g.f[std::size_t(g.id(11, 5, 22))] = -kInf;
  g.f[std::size_t(g.id(7, 20, 5))] = kInf; // x 7.8: the -inf is nearer
  g.f[std::size_t(g.id(8, 20, 5))] = -kInf;
  g.f[std::size_t(g.id(30, 9, 8))] = -kInf; // y 9.33
  g.f[std::size_t(g.id(30, 10, 8))] = kInf;
  g.f[std::size_t(g.id(25, 25, 11))] = kInf; // z 11.6
  g.f[std::size_t(g.id(25, 25, 12))] = -kInf;
  opt.max_dim = 16;
  const contour_tree want = tree_of(resampled_field(g, 16));
  ASSERT_TRUE(valid_tree(want));
  const contour_tree t = tree_of(g, opt);
  ASSERT_FALSE(any_nan(t));
  EXPECT_TRUE(identical(t, want));
  EXPECT_EQ(t.value_max, double(kInf));
  EXPECT_EQ(t.value_min, -double(kInf));
  EXPECT_TRUE(
      identical(cvc::compute_contour_tree(typed_voxels<float>(ctx, g, cvc::Float), opt), want));
  std::array<int, 3> to{};
  const std::vector<double> gd(g.f.begin(), g.f.end());
  const contour_tree want_d = cvc::compute_contour_tree(
      double_voxels(ctx, expected_resample(gd, {40, 36, 30}, 16, to), to));
  EXPECT_TRUE(
      identical(cvc::compute_contour_tree(typed_voxels<double>(ctx, g, cvc::Double), opt), want_d));

  // Samples exactly midway between a +inf and a -inf (25 -> 17 voxels puts x at 1.5, 4.5, ...)
  // take the lower tap.
  field line = make_field(25, 3, 2);
  fill_noise(line, rng, 50);
  line.f[1] = kInf;
  line.f[2] = -kInf;
  line.f[std::size_t(line.id(4, 1, 1))] = -kInf;
  line.f[std::size_t(line.id(5, 1, 1))] = kInf;
  opt.max_dim = 17;
  const field rl = resampled_field(line, 17);
  EXPECT_EQ(rl.f[1], kInf);
  EXPECT_EQ(rl.f[std::size_t(rl.id(3, 1, 1))], -kInf);
  const contour_tree tl = tree_of(line, opt);
  ASSERT_FALSE(any_nan(tl));
  EXPECT_TRUE(identical(tl, tree_of(rl)));

  // A narrow range (one counting pass over at most 2^16 bins): a NaN's key fell far outside the
  // histogram there.
  field narrow = make_field(150, 140, 1);
  for (float &v : narrow.f)
    v = 1.f + float(rng() % 4000) * std::ldexp(1.f, -23);
  narrow.f[std::size_t(narrow.id(75, 70, 0))] = kInf;
  opt.max_dim = 64;
  const contour_tree tn = tree_of(narrow, opt);
  ASSERT_FALSE(any_nan(tn));
  EXPECT_TRUE(identical(tn, tree_of(resampled_field(narrow, 64))));
}

TEST_F(ContourTreeVoxels, ResamplingHugeDoublesDoesNotOverflow) {
  // Finite doubles of opposite sign more than DBL_MAX apart: q - p overflowed to inf, and a
  // weight-0 tap then made NaN of it (inf * 0). They blend without overflowing and stay finite.
  std::mt19937 rng(93);
  const std::array<int, 3> dims{{30, 28, 3}};
  std::vector<double> f(std::size_t(dims[0] * dims[1] * dims[2]));
  for (double &v : f)
    v = (rng() % 2 ? 1.0 : -1.0) * (0.95e308 + double(rng() % 1000) * 0.8e305);
  contour_tree_options opt;
  opt.max_dim = 12;
  const contour_tree t = cvc::compute_contour_tree(double_voxels(ctx, f, dims), opt);
  ASSERT_FALSE(any_nan(t));
  std::array<int, 3> to{};
  const std::vector<double> r = expected_resample(f, dims, 12, to);
  ASSERT_TRUE(std::all_of(r.begin(), r.end(), [](double v) { return std::isfinite(v); }));
  EXPECT_TRUE(identical(t, cvc::compute_contour_tree(double_voxels(ctx, r, to))));
  EXPECT_TRUE(std::isfinite(t.value_min));
  EXPECT_TRUE(std::isfinite(t.value_max));
}

TEST(ContourTreeBuilder, SteppedResamplingOfInfinitiesMatchesOneShot) {
  std::mt19937 rng(94);
  field g = make_field(26, 22, 15);
  fill_smooth(g, rng, 8, false);
  g.f[std::size_t(g.id(4, 5, 5))] = kInf;
  g.f[std::size_t(g.id(5, 5, 5))] = -kInf;
  g.f[std::size_t(g.id(20, 3, 9))] = kInf;
  g.f[std::size_t(g.id(13, 18, 2))] = -kInf;
  g.f[0] = kInf;
  for (const int max_dim : {0, 12}) {
    contour_tree_options opt;
    opt.max_dim = cvc::uint64(max_dim);
    const contour_tree ref = tree_of(g, opt);
    ASSERT_TRUE(valid_tree(ref)) << max_dim;
    ASSERT_FALSE(any_nan(ref)) << max_dim;
    if (max_dim > 0)
      ASSERT_TRUE(identical(ref, tree_of(resampled_field(g, max_dim))));
    const std::vector<std::size_t> units = builder_units(std::size_t(g.n()), ref, true, false);
    for (const schedule &s : schedules()) {
      contour_tree_builder b(g.f.data(), g.nx, g.ny, g.nz, opt);
      const stepped r = step_through(b, kBuilderPhases, s);
      const std::string what = "max_dim " + std::to_string(max_dim) + ", " + s.describe();
      EXPECT_TRUE(identical(r.tree, ref)) << what;
      EXPECT_TRUE(sliced(r, kBuilderPhases, units, s.most())) << what;
    }
  }
}

TEST(ContourTreeSimplify, NodeBudgetPrunesInfiniteSpans) {
  // simplify_contour_tree_to has no persistence threshold: a leaf of infinite span (an extremum
  // at +-inf) is pruned too, after every finite one, so the budget holds.
  field g = noisy_field(31, 16);
  for (const int v : {g.id(3, 3, 3), g.id(12, 4, 9), g.id(8, 12, 2), g.id(4, 10, 13),
                      g.id(13, 13, 12), g.id(9, 7, 7)})
    g.f[std::size_t(v)] = kInf;
  const contour_tree t = tree_of(g);
  ASSERT_TRUE(valid_tree(t, &g));
  ASSERT_GT(t.nodes.size(), 60u);
  ASSERT_EQ(std::count_if(t.nodes.begin(), t.nodes.end(),
                          [](const contour_tree::node &nd) { return std::isinf(nd.value); }),
            6);
  for (std::size_t budget :
       {std::size_t(50), std::size_t(10), std::size_t(5), std::size_t(3), std::size_t(2)}) {
    const contour_tree s = cvc::simplify_contour_tree_to(t, budget);
    ASSERT_TRUE(valid_simplification(s, t, g)) << "budget " << budget;
    EXPECT_LE(s.nodes.size(), std::max<std::size_t>(budget, 2)) << "budget " << budget;
    EXPECT_GE(s.nodes.size() + 1, std::max<std::size_t>(budget, 2)) << "budget " << budget;
  }

  // By hand: a minimum, a saddle, 300 finite maxima and 4 at +inf on it. The finite maxima go
  // first, then the infinite ones (equal spans in node order).
  contour_tree h;
  auto add = [&h](double v, kind k) {
    contour_tree::node nd;
    nd.voxel = h.nodes.size();
    nd.value = v;
    nd.kind = k;
    h.nodes.push_back(nd);
  };
  add(0.0, kind::minimum);
  add(1.0, kind::join);
  for (int i = 0; i < 300; ++i)
    add(1.0 + 0.001 * (i + 1), kind::maximum);
  for (int i = 0; i < 4; ++i)
    add(double(kInf), kind::maximum);
  h.value_min = 0.0;
  h.value_max = double(kInf);
  for (std::uint32_t i = 1; i < h.nodes.size(); ++i) {
    contour_tree::arc a;
    a.hi = i;
    a.lo = i == 1 ? 0 : 1;
    h.arcs.push_back(a);
  }
  const contour_tree four = cvc::simplify_contour_tree_to(h, 4);
  ASSERT_TRUE(valid_tree(four));
  ASSERT_EQ(four.nodes.size(), 4u);
  EXPECT_EQ(four.nodes[2].voxel, 304u);
  EXPECT_EQ(four.nodes[3].voxel, 305u);
  const contour_tree two = cvc::simplify_contour_tree_to(h, 2);
  ASSERT_TRUE(valid_tree(two));
  ASSERT_EQ(two.nodes.size(), 2u);
  EXPECT_EQ(two.nodes[0].voxel, 0u);
  EXPECT_EQ(two.nodes[1].voxel, 305u);
  for (const schedule &s : schedules()) {
    contour_tree_simplifier m = contour_tree_simplifier::to_nodes(h, 4);
    const stepped r = step_through(m, kSimplifierPhases, s);
    EXPECT_TRUE(identical(r.tree, four)) << s.describe();
    EXPECT_TRUE(sliced(r, kSimplifierPhases, simplifier_units(h, four, false), s.most()))
        << s.describe();
  }
  // By persistence an infinite span is never below the threshold (any positive fraction of an
  // infinite range is infinite): every finite maximum goes, the infinite ones stay.
  const contour_tree p = cvc::simplify_contour_tree(h, 0.5);
  ASSERT_TRUE(valid_tree(p));
  EXPECT_EQ(p.nodes.size(), 6u);
  EXPECT_EQ(count_kind(p, kind::maximum), 4u);

  // A plateau at +inf: its inner arcs join equal values, so their span is 0 as in a finite
  // plateau (inf - inf, a NaN, could never be pruned), and it simplifies as the finite one does.
  field u = make_field(5, 3, 1), u9 = make_field(5, 3, 1);
  u9.f = {9, 9, 9, 9, 9, //
          9, 0, 0, 0, 9, //
          9, 0, 0, 0, 9};
  u.f = u9.f;
  for (float &v : u.f)
    if (v == 9.f)
      v = kInf;
  const contour_tree tu = tree_of(u), t9 = tree_of(u9);
  ASSERT_TRUE(valid_tree(tu, &u));
  ASSERT_TRUE(same_shape(tu, t9));
  ASSERT_GT(tu.nodes.size(), 3u);
  for (std::size_t budget = 2; budget < tu.nodes.size(); ++budget) {
    const contour_tree su = cvc::simplify_contour_tree_to(tu, budget);
    EXPECT_TRUE(same_shape(su, cvc::simplify_contour_tree_to(t9, budget))) << budget;
    EXPECT_LE(su.nodes.size(), budget) << budget;
  }
}

TEST(ContourTreeSimplify, NaNNodeValueIsMalformed) {
  // A NaN value has no order and gives its leaf no span, so it could never be pruned and the node
  // budget would not hold: the simplifiers refuse the tree as malformed.
  contour_tree t;
  const double values[] = {0.0, 1.0, 2.0, 3.0, std::numeric_limits<double>::quiet_NaN()};
  for (const double v : values) {
    contour_tree::node nd;
    nd.voxel = t.nodes.size();
    nd.value = v;
    nd.kind = v == 1.0 ? kind::join : v == 0.0 ? kind::minimum : kind::maximum;
    t.nodes.push_back(nd);
  }
  t.value_max = 3.0;
  for (std::uint32_t i = 1; i < t.nodes.size(); ++i) {
    contour_tree::arc a;
    a.hi = i;
    a.lo = i == 1 ? 0 : 1;
    t.arcs.push_back(a);
  }
  EXPECT_THROW(cvc::simplify_contour_tree_to(t, 2), cvc::contour_tree_error);
  EXPECT_THROW(cvc::simplify_contour_tree(t, 0.5), cvc::contour_tree_error);
  contour_tree_simplifier m = contour_tree_simplifier::to_nodes(t, 2);
  bool threw = false;
  try {
    while (!m.step(1)) {
    }
  } catch (const cvc::contour_tree_error &) {
    threw = true;
  }
  EXPECT_TRUE(threw);
  EXPECT_STREQ(m.phase(), "failed");
  // Nothing to prune is not checked: the tree comes back as it was.
  EXPECT_EQ(cvc::simplify_contour_tree_to(t, 5).nodes.size(), 5u);
}
