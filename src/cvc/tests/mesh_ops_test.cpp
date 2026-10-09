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

// mesh_ops_test -- cvc::mesh_ops (libigl-backed surface and tet operations).
//
// Fixtures are generated here: subdivided icospheres (outward-wound), a unit
// cube of Kuhn tets (6 per cell, positively oriented), a bumpy disk, an
// axis-aligned box, a flat grid with a near-collinear cap triangle. Covers
// outward normals; orient_outward on half-flipped and inside-out spheres;
// repair of a triangle soup with attributes, and welding by true distance;
// curvature of a sphere and its sign under re-winding, sphere-search principal
// curvature past max(F), NaN on boundaries; the three smoothers (denoising,
// area preservation, pinned boundary, step limits); slivers in cotangent
// operators; closest points, rays (scale-free, nearest of close hits), signed
// distance and winding numbers (including a sphere with a hole) and concurrent
// queries; heat geodesics against great-circle distance, across disconnected
// parts and on parts without interior vertices; tet orientation, volumes,
// boundary surface, slices and level sets; colour maps and robust ranges;
// cvc::sdf(SDF_IGL) against the analytic sphere SDF, also on a planar slice;
// and geometry::project() landing on the surface.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/mesh_ops.h>
#include <gtest/gtest.h>
#include <limits>
#include <map>
#include <numeric>
#include <thread>
#include <utility>
#include <vector>

#ifdef CVC_ENABLE_SDF
#include <cvc/core/app.h>
#include <cvc/utility/algorithm.h>
#endif

using cvc::geometry;

namespace {

const double kPi = 3.14159265358979323846;

double norm3(const geometry::point_t &p) {
  return std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
}

// Deterministic value in [-1, 1).
double hash_unit(std::uint64_t i) {
  i = (i ^ (i >> 33)) * 0xff51afd7ed558ccdULL;
  i = (i ^ (i >> 33)) * 0xc4ceb9fe1a85ec53ULL;
  i ^= i >> 33;
  return double(i >> 11) / double(1ULL << 52) - 1.0;
}

// Icosahedron subdivided `levels` times onto the sphere of radius r about c,
// triangles wound counter-clockwise seen from outside.
geometry icosphere(int levels, double r, geometry::point_t c = {{0, 0, 0}}) {
  const double t = (1.0 + std::sqrt(5.0)) / 2.0;
  std::vector<std::array<double, 3>> P = {{-1, t, 0}, {1, t, 0}, {-1, -t, 0}, {1, -t, 0},
                                          {0, -1, t}, {0, 1, t}, {0, -1, -t}, {0, 1, -t},
                                          {t, 0, -1}, {t, 0, 1}, {-t, 0, -1}, {-t, 0, 1}};
  std::vector<std::array<int, 3>> F = {{0, 11, 5}, {0, 5, 1},  {0, 1, 7},   {0, 7, 10}, {0, 10, 11},
                                       {1, 5, 9},  {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
                                       {3, 9, 4},  {3, 4, 2},  {3, 2, 6},   {3, 6, 8},  {3, 8, 9},
                                       {4, 9, 5},  {2, 4, 11}, {6, 2, 10},  {8, 6, 7},  {9, 8, 1}};
  for (int l = 0; l < levels; ++l) {
    std::map<std::pair<int, int>, int> mid;
    const auto midpoint = [&](int a, int b) {
      const std::pair<int, int> key(std::min(a, b), std::max(a, b));
      auto it = mid.find(key);
      if (it != mid.end())
        return it->second;
      P.push_back({(P[a][0] + P[b][0]) / 2, (P[a][1] + P[b][1]) / 2, (P[a][2] + P[b][2]) / 2});
      return mid[key] = int(P.size()) - 1;
    };
    std::vector<std::array<int, 3>> G;
    for (const auto &f : F) {
      const int ab = midpoint(f[0], f[1]), bc = midpoint(f[1], f[2]), ca = midpoint(f[2], f[0]);
      G.push_back({f[0], ab, ca});
      G.push_back({f[1], bc, ab});
      G.push_back({f[2], ca, bc});
      G.push_back({ab, bc, ca});
    }
    F.swap(G);
  }
  geometry g;
  for (const auto &p : P) {
    const double s = r / std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
    g.points().push_back({{c[0] + s * p[0], c[1] + s * p[1], c[2] + s * p[2]}});
  }
  for (const auto &f : F)
    g.tris().push_back({{cvc::index_t(f[0]), cvc::index_t(f[1]), cvc::index_t(f[2])}});
  return g;
}

// n^3 cells of the cube [0,L]^3, six Kuhn tets per cell (conforming), each with
// positive volume.
geometry cube_tets(int n, double L = 1.0) {
  geometry g;
  const auto id = [n](int i, int j, int k) {
    return cvc::index_t(i + (n + 1) * (j + (n + 1) * k));
  };
  for (int k = 0; k <= n; ++k)
    for (int j = 0; j <= n; ++j)
      for (int i = 0; i <= n; ++i)
        g.points().push_back({{L * i / n, L * j / n, L * k / n}});
  const int perms[6][3] = {{0, 1, 2}, {1, 2, 0}, {2, 0, 1}, {0, 2, 1}, {2, 1, 0}, {1, 0, 2}};
  for (int k = 0; k < n; ++k)
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i)
        for (int p = 0; p < 6; ++p) {
          int c[3] = {i, j, k};
          geometry::tet_t t;
          t[0] = id(c[0], c[1], c[2]);
          for (int s = 0; s < 3; ++s) {
            ++c[perms[p][s]];
            t[s + 1] = id(c[0], c[1], c[2]);
          }
          if (p >= 3) // odd permutation
            std::swap(t[2], t[3]);
          g.tets().push_back(t);
        }
  return g;
}

// Axis-aligned box [lo, hi] as 12 outward triangles.
geometry box(geometry::point_t lo, geometry::point_t hi) {
  geometry g;
  for (int k = 0; k < 2; ++k)
    for (int j = 0; j < 2; ++j)
      for (int i = 0; i < 2; ++i)
        g.points().push_back({{i ? hi[0] : lo[0], j ? hi[1] : lo[1], k ? hi[2] : lo[2]}});
  const int q[6][4] = {{0, 2, 3, 1}, {4, 5, 7, 6}, {0, 1, 5, 4},
                       {2, 6, 7, 3}, {0, 4, 6, 2}, {1, 3, 7, 5}};
  for (const auto &f : q) {
    g.tris().push_back({{cvc::index_t(f[0]), cvc::index_t(f[1]), cvc::index_t(f[2])}});
    g.tris().push_back({{cvc::index_t(f[0]), cvc::index_t(f[2]), cvc::index_t(f[3])}});
  }
  return g;
}

// Unit disk in z=0: a centre vertex and `rings` rings of `seg` vertices, wound
// counter-clockwise seen from +z; every vertex but the rim displaced in z.
geometry bumpy_disk(int rings, int seg, double amp) {
  geometry g;
  g.points().push_back({{0, 0, amp * hash_unit(0)}});
  for (int r = 1; r <= rings; ++r)
    for (int s = 0; s < seg; ++s) {
      const double a = 2 * kPi * s / seg, rad = double(r) / rings;
      const double z = r == rings ? 0.0 : amp * hash_unit(std::uint64_t(g.num_points()));
      g.points().push_back({{rad * std::cos(a), rad * std::sin(a), z}});
    }
  const auto v = [seg](int r, int s) { return cvc::index_t(1 + (r - 1) * seg + s % seg); };
  for (int s = 0; s < seg; ++s)
    g.tris().push_back({{0, v(1, s), v(1, s + 1)}});
  for (int r = 1; r < rings; ++r)
    for (int s = 0; s < seg; ++s) {
      g.tris().push_back({{v(r, s), v(r + 1, s), v(r + 1, s + 1)}});
      g.tris().push_back({{v(r, s), v(r + 1, s + 1), v(r, s + 1)}});
    }
  return g;
}

// The 5 x 5 grid [-2, 2]^2 in z = 0 (two triangles per unit cell, wound
// counter-clockwise seen from +z) with a near-collinear "cap" spliced in:
// vertex 25 = (0.5, 1e-9, 0) sits just above the edge (0,0)-(1,0), and that
// cell's triangle (0,0),(1,0),(1,1) becomes (0,0),m,(1,1) + m,(1,0),(1,1) plus
// the cap (0,0),(1,0),m. The cap's edge lengths round to exactly 0.5, 0.5 and
// 1, so its edge-length (Heron) area is 0 while its cross-product area is
// 5e-10: libigl's cotangent weights for it are infinite.
cvc::index_t grid_id(int i, int j) { return cvc::index_t((i + 2) + 5 * (j + 2)); }

geometry capped_grid() {
  geometry g;
  for (int j = -2; j <= 2; ++j)
    for (int i = -2; i <= 2; ++i)
      g.points().push_back({{double(i), double(j), 0.0}});
  g.points().push_back({{0.5, 1e-9, 0.0}});
  const cvc::index_t m = 25;
  for (int j = -2; j < 2; ++j)
    for (int i = -2; i < 2; ++i) {
      const cvc::index_t a = grid_id(i, j), b = grid_id(i + 1, j), c = grid_id(i + 1, j + 1),
                         d = grid_id(i, j + 1);
      if (i == 0 && j == 0) {
        g.tris().push_back({{a, m, c}});
        g.tris().push_back({{m, b, c}});
        g.tris().push_back({{a, b, m}}); // the cap
      } else {
        g.tris().push_back({{a, b, c}});
      }
      g.tris().push_back({{a, c, d}});
    }
  return g;
}

// Signed volume enclosed by a closed triangle surface (positive if outward).
double enclosed_volume(const geometry &g) {
  double v = 0;
  const auto &P = g.const_points();
  for (const auto &t : g.const_tris()) {
    const auto &a = P[t[0]], &b = P[t[1]], &c = P[t[2]];
    v += (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) +
          a[2] * (b[0] * c[1] - b[1] * c[0])) /
         6.0;
  }
  return v;
}

double surface_area(const geometry &g) {
  double area = 0;
  const auto &P = g.const_points();
  for (const auto &t : g.const_tris()) {
    const auto &a = P[t[0]], &b = P[t[1]], &c = P[t[2]];
    const double u[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
    const double w[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
    const double x = u[1] * w[2] - u[2] * w[1], y = u[2] * w[0] - u[0] * w[2],
                 z = u[0] * w[1] - u[1] * w[0];
    area += 0.5 * std::sqrt(x * x + y * y + z * z);
  }
  return area;
}

// Every triangle's normal points away from the origin.
bool all_faces_outward(const geometry &g) {
  const auto &P = g.const_points();
  for (const auto &t : g.const_tris()) {
    const auto &a = P[t[0]], &b = P[t[1]], &c = P[t[2]];
    const double u[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
    const double w[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
    const double n[3] = {u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2],
                         u[0] * w[1] - u[1] * w[0]};
    if (n[0] * (a[0] + b[0] + c[0]) + n[1] * (a[1] + b[1] + c[1]) + n[2] * (a[2] + b[2] + c[2]) <=
        0)
      return false;
  }
  return true;
}

void reverse_tri(geometry::tri_t &t) { std::swap(t[1], t[2]); }

double radial_spread(const geometry &g) {
  double mean = 0, var = 0;
  for (const auto &p : g.const_points())
    mean += norm3(p);
  mean /= double(g.num_points());
  for (const auto &p : g.const_points())
    var += (norm3(p) - mean) * (norm3(p) - mean);
  return std::sqrt(var / double(g.num_points()));
}

} // namespace

TEST(MeshOps, Available) { EXPECT_TRUE(cvc::mesh_ops_available()); }

TEST(MeshOps, IcosphereFixtureIsOutward) {
  const geometry s = icosphere(2, 1.0);
  EXPECT_EQ(s.num_points(), 162u);
  EXPECT_GT(enclosed_volume(s), 0.0);
  EXPECT_TRUE(all_faces_outward(s));
}

// ---- normals and orientation ---------------------------------------------------------

TEST(MeshOps, NormalsPointOutwardOnSphere) {
  for (auto w : {cvc::normal_weighting::UNIFORM, cvc::normal_weighting::AREA,
                 cvc::normal_weighting::ANGLE}) {
    geometry s = icosphere(3, 2.0);
    cvc::compute_vertex_normals(s, w);
    ASSERT_EQ(s.const_normals().size(), s.num_points());
    for (std::size_t i = 0; i < s.num_points(); ++i) {
      const auto &n = s.const_normals()[i], &p = s.const_points()[i];
      EXPECT_NEAR(norm3(n), 1.0, 1e-12);
      EXPECT_GT((n[0] * p[0] + n[1] * p[1] + n[2] * p[2]) / norm3(p), 0.999);
    }
  }
}

TEST(MeshOps, NormalsOfUnreferencedVertexAreZero) {
  geometry s = icosphere(1, 1.0);
  s.points().push_back({{5, 5, 5}});
  cvc::compute_vertex_normals(s);
  const auto &n = s.const_normals().back();
  EXPECT_EQ(n[0], 0.0);
  EXPECT_EQ(n[1], 0.0);
  EXPECT_EQ(n[2], 0.0);
}

TEST(MeshOps, OrientOutwardFixesHalfFlippedSphere) {
  geometry s = icosphere(3, 1.0);
  std::size_t flipped = 0;
  for (std::size_t f = 0; f < s.num_tris(); ++f)
    if (hash_unit(f) > 0) {
      reverse_tri(s.tris()[f]);
      ++flipped;
    }
  ASSERT_GT(flipped, 0u);
  ASSERT_FALSE(all_faces_outward(s));
  EXPECT_EQ(cvc::orient_outward(s), flipped);
  EXPECT_TRUE(all_faces_outward(s));
  EXPECT_EQ(cvc::orient_outward(s), 0u);
}

TEST(MeshOps, OrientOutwardFixesInsideOutSphere) {
  geometry s = icosphere(2, 1.0);
  for (auto &t : s.tris())
    reverse_tri(t);
  // Aligned normals are recomputed from the new winding.
  cvc::compute_vertex_normals(s);
  EXPECT_EQ(cvc::orient_outward(s), s.num_tris());
  EXPECT_TRUE(all_faces_outward(s));
  const auto &n = s.const_normals()[0], &p = s.const_points()[0];
  EXPECT_GT(n[0] * p[0] + n[1] * p[1] + n[2] * p[2], 0.0);
}

TEST(MeshOps, OrientOutwardTriangulatesQuads) {
  geometry g = box({{-1, -1, -1}}, {{1, 1, 1}});
  geometry q;
  q.points() = g.const_points();
  const int quads[6][4] = {{0, 2, 3, 1}, {4, 5, 7, 6}, {0, 1, 5, 4},
                           {2, 6, 7, 3}, {0, 4, 6, 2}, {1, 3, 7, 5}};
  for (int f = 0; f < 6; ++f) {
    geometry::quad_t qd = {{cvc::index_t(quads[f][0]), cvc::index_t(quads[f][1]),
                            cvc::index_t(quads[f][2]), cvc::index_t(quads[f][3])}};
    if (f % 2)
      std::reverse(qd.begin(), qd.end());
    q.quads().push_back(qd);
  }
  EXPECT_EQ(cvc::orient_outward(q), 6u); // the three reversed quads' two triangles each
  EXPECT_TRUE(q.const_quads().empty());
  EXPECT_EQ(q.num_tris(), 12u);
  EXPECT_TRUE(all_faces_outward(q));
}

// ---- repair -----------------------------------------------------------------------------

TEST(MeshOps, RepairWeldsSoupAndRemapsAttributes) {
  const geometry s = icosphere(2, 1.0);
  const auto color_of = [](const geometry::point_t &p) {
    return geometry::color_t{{0.5 + 0.5 * p[0], 0.5 + 0.5 * p[1], 0.5 + 0.5 * p[2]}};
  };
  geometry soup;
  for (const auto &t : s.const_tris()) {
    for (int c = 0; c < 3; ++c) {
      const auto &p = s.const_points()[t[c]];
      soup.points().push_back(p);
      soup.colors().push_back(color_of(p));
      soup.functions().push_back(p[2]);
    }
    const cvc::index_t b = soup.num_points() - 3;
    soup.tris().push_back({{b, b + 1, b + 2}});
  }
  // Two degenerate triangles, a line, and an orphan vertex.
  soup.tris().push_back({{0, 0, 1}});
  soup.tris().push_back({{0, 1, 1}});
  soup.lines().push_back({{4, 7}});
  soup.points().push_back({{9, 9, 9}});
  soup.colors().push_back({{0, 0, 0}});
  soup.functions().push_back(-1.0);
  soup.uvs().resize(3); // not aligned: cleared

  const std::size_t n0 = soup.num_points();
  const cvc::repair_report r = cvc::repair(soup);
  EXPECT_EQ(soup.num_points(), s.num_points());
  EXPECT_EQ(r.vertices_removed, n0 - s.num_points());
  EXPECT_EQ(r.faces_removed, 2u);
  EXPECT_EQ(r.faces_flipped, 0u);
  EXPECT_EQ(soup.num_tris(), s.num_tris());
  ASSERT_EQ(soup.const_colors().size(), soup.num_points());
  ASSERT_EQ(soup.const_functions().size(), soup.num_points());
  EXPECT_TRUE(soup.const_uvs().empty());
  for (std::size_t i = 0; i < soup.num_points(); ++i) {
    const auto &p = soup.const_points()[i];
    const auto want = color_of(p);
    for (int c = 0; c < 3; ++c)
      EXPECT_DOUBLE_EQ(soup.const_colors()[i][c], want[c]);
    EXPECT_DOUBLE_EQ(soup.const_functions()[i], p[2]);
  }
  ASSERT_EQ(soup.num_lines(), 1u);
  EXPECT_LT(soup.const_lines()[0][0], soup.num_points());
  EXPECT_TRUE(all_faces_outward(soup));
  EXPECT_NEAR(enclosed_volume(soup), enclosed_volume(s), 1e-12);
}

TEST(MeshOps, RepairRejectsBadIndex) {
  geometry g = icosphere(0, 1.0);
  g.tris().push_back({{0, 1, 99}});
  EXPECT_THROW(cvc::repair(g), cvc::mesh_ops_error);
}

// weld_epsilon is a distance. A grid that rounds x / eps would keep apart two
// copies 1e-12 apart on either side of a cell boundary (at 2.5 eps), and merge
// two points 1.56 eps apart inside one cell (+-0.45 eps on every axis).
TEST(MeshOps, RepairWeldEpsilonIsADistance) {
  const double eps = 1e-6;
  geometry g;
  // Two triangles meant to share the edge p0-p2; p1 is p0's near-copy.
  g.points().push_back({{2.5 * eps - 5e-13, 0, 0}}); // 0: p0
  g.points().push_back({{2.5 * eps + 5e-13, 0, 0}}); // 1: p1, 1e-12 from p0
  g.points().push_back({{1, 0, 0}});                 // 2
  g.points().push_back({{0, 1, 0}});                 // 3
  g.points().push_back({{0, -1, 0}});                // 4
  g.tris().push_back({{0, 2, 3}});
  g.tris().push_back({{1, 4, 2}});
  // Two separate triangles whose corners q, r are 1.56 eps apart.
  const double h = 0.45 * eps;
  g.points().push_back({{-h, 3 - h, -h}}); // 5: q
  g.points().push_back({{h, 3 + h, h}});   // 6: r
  g.points().push_back({{1, 3, 0}});       // 7
  g.points().push_back({{0, 4, 0}});       // 8
  g.points().push_back({{-1, 3, 0}});      // 9
  g.points().push_back({{0, 2, 0}});       // 10
  g.tris().push_back({{5, 7, 8}});
  g.tris().push_back({{6, 9, 10}});

  cvc::repair_params p;
  p.weld_epsilon = eps;
  p.orient = false;
  const cvc::repair_report rep = cvc::repair(g, p);
  EXPECT_EQ(rep.vertices_removed, 1u);
  ASSERT_EQ(g.num_points(), 10u);
  ASSERT_EQ(g.num_tris(), 4u);
  // The first two triangles now share two corners; the last two none.
  const auto shared = [&g](std::size_t a, std::size_t b) {
    int n = 0;
    for (cvc::index_t u : g.const_tris()[a])
      for (cvc::index_t v : g.const_tris()[b])
        n += u == v ? 1 : 0;
    return n;
  };
  EXPECT_EQ(shared(0, 1), 2);
  EXPECT_EQ(shared(2, 3), 0);
  // The lowest-numbered copy survives, with its own position.
  EXPECT_EQ(g.const_points()[0][0], 2.5 * eps - 5e-13);

  // Transitive: a chain of points each within eps of the next becomes one.
  geometry chain;
  for (int i = 0; i < 4; ++i)
    chain.points().push_back({{0.9 * eps * i, 0, 0}});
  chain.points().push_back({{1, 0, 0}});
  chain.points().push_back({{0, 1, 0}});
  for (cvc::index_t i = 0; i < 4; ++i)
    chain.tris().push_back({{i, 4, 5}});
  cvc::repair(chain, p);
  EXPECT_EQ(chain.num_points(), 3u);
}

// ---- curvature ----------------------------------------------------------------------------

TEST(MeshOps, CurvatureOfSphere) {
  const double r = 2.0;
  geometry s = icosphere(4, r);
  const std::vector<double> H = cvc::vertex_curvature(s, cvc::curvature_kind::MEAN);
  const std::vector<double> K = cvc::vertex_curvature(s, cvc::curvature_kind::GAUSSIAN);
  cvc::compute_curvature(s);
  ASSERT_EQ(s.const_curvatures().size(), s.num_points());
  for (std::size_t i = 0; i < s.num_points(); ++i) {
    EXPECT_NEAR(H[i], 1 / r, 0.05 / r);
    EXPECT_NEAR(K[i], 1 / (r * r), 0.05 / (r * r));
    const auto &k = s.const_curvatures()[i];
    EXPECT_GE(k[0], k[1]);
    EXPECT_NEAR(k[0], 1 / r, 0.05 / r);
    EXPECT_NEAR(k[1], 1 / r, 0.05 / r);
  }
}

TEST(MeshOps, CurvatureSignFollowsWinding) {
  const double r = 0.5;
  geometry s = icosphere(4, r);
  for (auto &t : s.tris())
    reverse_tri(t);
  const std::vector<double> H = cvc::vertex_curvature(s, cvc::curvature_kind::MEAN);
  const std::vector<double> K = cvc::vertex_curvature(s, cvc::curvature_kind::GAUSSIAN);
  const std::vector<double> k1 = cvc::vertex_curvature(s, cvc::curvature_kind::MAX_PRINCIPAL);
  const std::vector<double> k2 = cvc::vertex_curvature(s, cvc::curvature_kind::MIN_PRINCIPAL);
  for (std::size_t i = 0; i < s.num_points(); ++i) {
    EXPECT_NEAR(H[i], -1 / r, 0.05 / r);
    EXPECT_NEAR(K[i], 1 / (r * r), 0.05 / (r * r)); // k1 * k2 keeps its sign
    EXPECT_NEAR(k1[i], -1 / r, 0.05 / r);
    EXPECT_NEAR(k2[i], -1 / r, 0.05 / r);
    EXPECT_GE(k1[i], k2[i]);
  }
}

TEST(MeshOps, CurvatureOfUnreferencedVertexIsZero) {
  geometry s = icosphere(2, 1.0);
  s.points().push_back({{3, 0, 0}});
  for (auto kind : {cvc::curvature_kind::MEAN, cvc::curvature_kind::GAUSSIAN,
                    cvc::curvature_kind::MAX_PRINCIPAL})
    EXPECT_EQ(cvc::vertex_curvature(s, kind).back(), 0.0);
}

// Sphere-search principal curvature (use_kring = false) visits every vertex,
// including ones above the largest index any usable triangle has: a trailing
// unreferenced point, or a vertex whose only triangle has zero area. libigl's
// own sphere search read past the end of its adjacency list there.
TEST(MeshOps, PrincipalCurvatureSphereSearch) {
  cvc::curvature_params p;
  p.use_kring = false;
  p.radius = 2;
  const double r = 2.0;
  geometry s = icosphere(4, r);
  const std::size_t n = s.num_points();
  s.points().push_back({{3 * r, 0, 0}}); // unreferenced, highest index
  const std::vector<double> k1 = cvc::vertex_curvature(s, cvc::curvature_kind::MAX_PRINCIPAL, p);
  const std::vector<double> k2 = cvc::vertex_curvature(s, cvc::curvature_kind::MIN_PRINCIPAL, p);
  ASSERT_EQ(k1.size(), n + 1);
  for (std::size_t i = 0; i < n; ++i) {
    EXPECT_NEAR(k1[i], 1 / r, 0.05 / r) << i;
    EXPECT_NEAR(k2[i], 1 / r, 0.05 / r) << i;
    EXPECT_GE(k1[i], k2[i]);
  }
  EXPECT_EQ(k1[n], 0.0);
  EXPECT_EQ(k2[n], 0.0);
  cvc::compute_curvature(s, p);
  ASSERT_EQ(s.const_curvatures().size(), n + 1);
  EXPECT_EQ(s.const_curvatures()[n][0], 0.0);

  // Three collinear points: their triangle has no area, so no usable triangle
  // reaches the highest-index vertex. It is referenced, so it reads NaN.
  geometry t = icosphere(2, 1.0);
  const cvc::index_t a = t.num_points();
  t.points().push_back({{3, 0, 0}});
  t.points().push_back({{4, 0, 0}});
  t.points().push_back({{5, 0, 0}});
  t.tris().push_back({{a, a + 1, a + 2}});
  const std::vector<double> kt = cvc::vertex_curvature(t, cvc::curvature_kind::MAX_PRINCIPAL, p);
  ASSERT_EQ(kt.size(), t.num_points());
  EXPECT_TRUE(std::isnan(kt.back()));
  EXPECT_NEAR(kt[0], 1.0, 0.25); // a coarse sphere, but a real fit
}

// On an open surface MEAN and GAUSSIAN are not curvature at the rim (the angle
// defect there is the boundary's turning): NaN, while the interior of a flat
// disk reads 0.
TEST(MeshOps, CurvatureIsNaNOnTheBoundary) {
  const geometry disk = bumpy_disk(6, 16, 0.0);
  const std::vector<double> H = cvc::vertex_curvature(disk, cvc::curvature_kind::MEAN);
  const std::vector<double> K = cvc::vertex_curvature(disk, cvc::curvature_kind::GAUSSIAN);
  ASSERT_EQ(H.size(), disk.num_points());
  std::size_t rim = 0;
  for (std::size_t i = 0; i < disk.num_points(); ++i) {
    const auto &q = disk.const_points()[i];
    if (std::hypot(q[0], q[1]) > 0.999) {
      ++rim;
      EXPECT_TRUE(std::isnan(H[i])) << i;
      EXPECT_TRUE(std::isnan(K[i])) << i;
    } else {
      EXPECT_NEAR(H[i], 0.0, 1e-9) << i;
      EXPECT_NEAR(K[i], 0.0, 1e-9) << i;
    }
  }
  EXPECT_EQ(rim, 16u);
  // The principal fit still works at the rim (one-sided neighbourhood).
  for (double k : cvc::vertex_curvature(disk, cvc::curvature_kind::MAX_PRINCIPAL))
    EXPECT_NEAR(k, 0.0, 1e-9);
}

// ---- smoothing -----------------------------------------------------------------------------

TEST(MeshOps, SmoothDenoisesSphere) {
  const geometry clean = icosphere(3, 1.0);
  geometry noisy = clean;
  for (std::size_t i = 0; i < noisy.num_points(); ++i) {
    const double s = 1.0 + 0.04 * hash_unit(i + 7);
    for (int c = 0; c < 3; ++c)
      noisy.points()[i][c] *= s;
  }
  const double spread0 = radial_spread(noisy);
  const double area0 = surface_area(noisy);

  geometry cot = noisy;
  cvc::smooth_params p;
  p.iterations = 3;
  cvc::smooth(cot, p);
  EXPECT_LT(radial_spread(cot), 0.5 * spread0);
  EXPECT_NEAR(surface_area(cot) / area0, 1.0, 1e-9); // preserve_area

  geometry tau = noisy;
  p.method = cvc::smooth_params::TAUBIN;
  p.lambda = 0.5;
  p.iterations = 10;
  cvc::smooth(tau, p);
  EXPECT_LT(radial_spread(tau), 0.5 * spread0);

  geometry uni = noisy;
  p.method = cvc::smooth_params::UNIFORM_LAPLACIAN;
  cvc::smooth(uni, p);
  EXPECT_LT(radial_spread(uni), 0.5 * spread0);
  // Taubin's mu step undoes the shrinking of plain umbrella smoothing.
  EXPECT_GT(enclosed_volume(tau), enclosed_volume(uni));
  EXPECT_NEAR(enclosed_volume(tau) / enclosed_volume(clean), 1.0, 0.05);
}

TEST(MeshOps, SmoothFixBoundaryPinsTheDiskRim) {
  const geometry disk = bumpy_disk(8, 24, 0.1);
  for (auto method : {cvc::smooth_params::COTAN_IMPLICIT, cvc::smooth_params::UNIFORM_LAPLACIAN,
                      cvc::smooth_params::TAUBIN}) {
    geometry g = disk;
    cvc::smooth_params p;
    p.method = method;
    p.fix_boundary = true;
    p.iterations = 5;
    p.lambda = method == cvc::smooth_params::COTAN_IMPLICIT ? 1e-2 : 0.5;
    cvc::smooth(g, p);
    double moved = 0;
    for (std::size_t i = 0; i < g.num_points(); ++i) {
      const auto &a = disk.const_points()[i], &b = g.const_points()[i];
      const double d = std::abs(a[0] - b[0]) + std::abs(a[1] - b[1]) + std::abs(a[2] - b[2]);
      if (std::hypot(a[0], a[1]) > 0.999) { // the rim
        EXPECT_EQ(d, 0.0);
      } else {
        moved += d;
      }
    }
    EXPECT_GT(moved, 0.0);
  }
}

// The explicit steps diverge above lambda = 1 (UNIFORM_LAPLACIAN) or 0.7
// (TAUBIN); such a step is rejected rather than run.
TEST(MeshOps, SmoothRejectsUnstableExplicitSteps) {
  const geometry s = icosphere(2, 1.0);
  cvc::smooth_params p;
  p.iterations = 3;
  p.method = cvc::smooth_params::UNIFORM_LAPLACIAN;
  p.lambda = 1.5;
  geometry g = s;
  EXPECT_THROW(cvc::smooth(g, p), cvc::mesh_ops_error);
  EXPECT_EQ(g.const_points()[0], s.const_points()[0]); // untouched
  p.lambda = 1.0;
  EXPECT_NO_THROW(cvc::smooth(g, p));
  p.method = cvc::smooth_params::TAUBIN;
  p.lambda = 1.0;
  geometry t = s;
  EXPECT_THROW(cvc::smooth(t, p), cvc::mesh_ops_error);
  p.lambda = 0.7;
  EXPECT_NO_THROW(cvc::smooth(t, p));
  for (const auto &q : t.const_points())
    EXPECT_NEAR(norm3(q), 1.0, 0.1);
  p.lambda = 0.0;
  EXPECT_THROW(cvc::smooth(t, p), cvc::mesh_ops_error);
}

// A near-collinear cap (see capped_grid) gets infinite cotangent weights from
// libigl; it is left out of the operator instead of breaking the whole solve.
TEST(MeshOps, SliverTriangleDoesNotBreakCotangentOperators) {
  const geometry grid = capped_grid();
  geometry g = grid;
  ASSERT_NO_THROW(cvc::smooth(g)); // COTAN_IMPLICIT
  for (const auto &q : g.const_points()) {
    EXPECT_TRUE(std::isfinite(q[0]) && std::isfinite(q[1]));
    EXPECT_EQ(q[2], 0.0);
  }

  const std::vector<double> d = cvc::geodesic_distance(grid, {grid_id(-2, -2)});
  ASSERT_EQ(d.size(), grid.num_points());
  for (std::size_t i = 0; i < d.size(); ++i)
    EXPECT_TRUE(std::isfinite(d[i])) << i;
  EXPECT_LT(d[grid_id(-1, -1)], d[grid_id(2, 2)]);

  // Flat interior: zero curvature, except around the cap, whose removal leaves
  // its corners on a (slit) boundary.
  const std::vector<double> H = cvc::vertex_curvature(grid, cvc::curvature_kind::MEAN);
  const std::vector<double> K = cvc::vertex_curvature(grid, cvc::curvature_kind::GAUSSIAN);
  for (int j = -1; j <= 1; ++j)
    for (int i = -1; i <= 1; ++i) {
      if (j == 0 && (i == 0 || i == 1))
        continue;
      EXPECT_NEAR(H[grid_id(i, j)], 0.0, 1e-9) << i << "," << j;
      EXPECT_NEAR(K[grid_id(i, j)], 0.0, 1e-9) << i << "," << j;
    }
}

// ---- spatial queries -----------------------------------------------------------------------

TEST(MeshOps, LocatorClosestPointsOnBox) {
  const geometry b = box({{0, 0, 0}}, {{1, 1, 1}});
  const cvc::mesh_locator loc(b);
  EXPECT_FALSE(loc.empty());
  EXPECT_EQ(loc.num_faces(), 12u);
  const geometry::points_t q = {{{0.5, 0.25, 2.0}}, {{0.5, 0.5, 0.6}}, {{2, 2, 2}}};
  const cvc::closest_point_result r = loc.closest_points(q);
  ASSERT_EQ(r.points.size(), 3u);
  EXPECT_NEAR(r.sq_distance[0], 1.0, 1e-12);
  EXPECT_NEAR(r.points[0][0], 0.5, 1e-12);
  EXPECT_NEAR(r.points[0][1], 0.25, 1e-12);
  EXPECT_NEAR(r.points[0][2], 1.0, 1e-12);
  EXPECT_NEAR(r.sq_distance[1], 0.16, 1e-12);
  EXPECT_NEAR(r.sq_distance[2], 3.0, 1e-12);
  for (std::int64_t f : r.face)
    EXPECT_TRUE(f >= 0 && f < 12);

  const std::vector<double> sd = loc.signed_distance(q);
  EXPECT_NEAR(sd[0], 1.0, 1e-12);
  EXPECT_NEAR(sd[1], -0.4, 1e-12);
  EXPECT_NEAR(sd[2], std::sqrt(3.0), 1e-12);
  const std::vector<double> w = loc.winding_number(q);
  EXPECT_NEAR(w[0], 0.0, 1e-3);
  EXPECT_NEAR(w[1], 1.0, 1e-3);
}

TEST(MeshOps, LocatorRayCasting) {
  const geometry b = box({{0, 0, 0}}, {{1, 1, 1}});
  const cvc::mesh_locator loc(b);
  cvc::ray_hit hit;
  ASSERT_TRUE(loc.intersect_ray({{0.3, 0.6, -1}}, {{0, 0, 2}}, hit));
  EXPECT_NEAR(hit.t, 0.5, 1e-14);
  EXPECT_NEAR(hit.point[2], 0.0, 1e-14);
  EXPECT_NEAR(hit.point[0], 0.3, 1e-14);
  const auto &P = b.const_points();
  const auto &T = b.const_tris()[std::size_t(hit.face)];
  for (int c = 0; c < 3; ++c)
    EXPECT_NEAR((1 - hit.u - hit.v) * P[T[0]][c] + hit.u * P[T[1]][c] + hit.v * P[T[2]][c],
                hit.point[c], 1e-14);
  // The far side, then nothing inside the window.
  ASSERT_TRUE(loc.intersect_ray({{0.3, 0.6, -1}}, {{0, 0, 2}}, hit, 0.75));
  EXPECT_NEAR(hit.t, 1.0, 1e-14);
  cvc::ray_hit untouched;
  untouched.t = 42;
  EXPECT_FALSE(loc.intersect_ray({{0.3, 0.6, -1}}, {{0, 0, 2}}, untouched, 0.0, 0.4));
  EXPECT_FALSE(loc.intersect_ray({{0.3, 0.6, -1}}, {{0, 0, -1}}, untouched));
  EXPECT_FALSE(loc.intersect_ray({{3, 3, 3}}, {{1, 0, 0}}, untouched));
  EXPECT_EQ(untouched.t, 42.0);
  // A negative min_t looks behind the origin.
  ASSERT_TRUE(loc.intersect_ray({{0.3, 0.6, 2}}, {{0, 0, 1}}, hit, -5.0));
  EXPECT_NEAR(hit.t, -2.0, 1e-14);
}

// Neither the mesh's unit nor |dir| decides what a ray hits. (libigl's leaf
// test rejects |e1 . (dir x e2)| <= 1e-6 -- twice the triangle's area times
// |dir| -- so a 0.1 mm box in metres, or a short dir, was never hit.)
TEST(MeshOps, LocatorRaysAreScaleFree) {
  const double s = 1e-4;
  const geometry tiny_box = box({{0, 0, 0}}, {{s, s, s}});
  const cvc::mesh_locator loc(tiny_box);
  cvc::ray_hit hit;
  ASSERT_TRUE(loc.intersect_ray({{0.3 * s, 0.6 * s, -1}}, {{0, 0, 1}}, hit));
  EXPECT_NEAR(hit.t, 1.0, 1e-15);
  EXPECT_NEAR(hit.point[0], 0.3 * s, 1e-18);
  EXPECT_NEAR(hit.point[1], 0.6 * s, 1e-18);
  EXPECT_NEAR(hit.point[2], 0.0, 1e-15);
  ASSERT_TRUE(loc.intersect_ray({{0.3 * s, 0.6 * s, -1}}, {{0, 0, 1}}, hit, 1.0 + 0.5 * s));
  EXPECT_NEAR(hit.t, 1.0 + s, 1e-14);

  const cvc::mesh_locator unit(box({{0, 0, 0}}, {{1, 1, 1}}));
  ASSERT_TRUE(unit.intersect_ray({{0.3, 0.6, -1}}, {{0, 0, 1e-7}}, hit));
  EXPECT_NEAR(hit.t, 1e7, 1e-6);
  EXPECT_NEAR(hit.point[2], 0.0, 1e-12);
  // A ray in a face's plane slides along it: the other faces are hit.
  ASSERT_TRUE(unit.intersect_ray({{-1, 0.5, 0}}, {{1, 0, 0}}, hit));
  EXPECT_NEAR(hit.t, 1.0, 1e-15);
  EXPECT_NEAR(hit.point[0], 0.0, 1e-15);
}

// Of two hits closer together than float precision the nearer one wins, in
// whichever order the triangles come (libigl keeps t in float).
TEST(MeshOps, LocatorRayPicksTheNearerOfCloseHits) {
  for (int order = 0; order < 2; ++order) {
    geometry g;
    for (int k = 0; k < 2; ++k) {
      const double z = (k == order) ? 1.0 : 1.0 + 1e-9;
      const cvc::index_t b = g.num_points();
      g.points().push_back({{0, 0, z}});
      g.points().push_back({{1, 0, z}});
      g.points().push_back({{0, 1, z}});
      g.tris().push_back({{b, b + 1, b + 2}});
    }
    const cvc::mesh_locator loc(g);
    cvc::ray_hit hit;
    ASSERT_TRUE(loc.intersect_ray({{0.25, 0.25, 0}}, {{0, 0, 1}}, hit)) << order;
    EXPECT_EQ(hit.face, order) << order;
    EXPECT_EQ(hit.t, 1.0) << order;
    EXPECT_NEAR(hit.u, 0.25, 1e-15);
    EXPECT_NEAR(hit.v, 0.25, 1e-15);
  }
}

TEST(MeshOps, SignedDistanceOfSphereWithHole) {
  geometry s = icosphere(3, 1.0);
  // Cut a cap off the top: the surface is no longer closed.
  geometry::tris_t kept;
  for (const auto &t : s.const_tris())
    if (s.const_points()[t[0]][2] < 0.8 || s.const_points()[t[1]][2] < 0.8 ||
        s.const_points()[t[2]][2] < 0.8)
      kept.push_back(t);
  ASSERT_LT(kept.size(), s.num_tris());
  s.tris() = kept;
  const cvc::mesh_locator loc(s);
  const geometry::points_t q = {{{0, 0, 0}},   {{0.3, -0.2, 0.1}}, {{0, 0, -0.5}},
                                {{1.5, 0, 0}}, {{0, 0, -2}},       {{0.7, 0.7, 0.7}}};
  const std::vector<double> sd = loc.signed_distance(q);
  const std::vector<double> w = loc.winding_number(q);
  for (int i = 0; i < 3; ++i) {
    EXPECT_LT(sd[i], 0.0) << i;
    EXPECT_GT(w[i], 0.5) << i;
  }
  for (int i = 3; i < 6; ++i) {
    EXPECT_GT(sd[i], 0.0) << i;
    EXPECT_LT(w[i], 0.5) << i;
  }
  EXPECT_NEAR(sd[0], -1.0, 0.02);
  EXPECT_NEAR(sd[3], 0.5, 1e-9);
}

TEST(MeshOps, LocatorIsSharedAndThreadSafe) {
  const geometry s = icosphere(3, 1.0);
  const cvc::mesh_locator loc(s);
  const cvc::mesh_locator copy = loc;
  geometry::points_t q;
  for (int i = 0; i < 2000; ++i)
    q.push_back({{1.5 * hash_unit(3 * i), 1.5 * hash_unit(3 * i + 1), 1.5 * hash_unit(3 * i + 2)}});
  const std::vector<double> ref = loc.signed_distance(q);
  std::vector<std::vector<double>> out(4);
  std::vector<std::thread> pool;
  for (int t = 0; t < 4; ++t)
    pool.emplace_back([&, t] { out[std::size_t(t)] = (t % 2 ? copy : loc).signed_distance(q); });
  for (auto &th : pool)
    th.join();
  for (const auto &o : out)
    EXPECT_EQ(o, ref);
  for (std::size_t i = 0; i < q.size(); ++i)
    EXPECT_NEAR(ref[i], norm3(q[i]) - 1.0, 0.02);
}

TEST(MeshOps, EmptyLocator) {
  const cvc::mesh_locator loc;
  EXPECT_TRUE(loc.empty());
  const cvc::closest_point_result r = loc.closest_points({{{0, 0, 0}}});
  EXPECT_EQ(r.face[0], -1);
  EXPECT_TRUE(std::isinf(r.sq_distance[0]));
  cvc::ray_hit hit;
  EXPECT_FALSE(loc.intersect_ray({{0, 0, 0}}, {{1, 0, 0}}, hit));
}

// ---- geodesics -------------------------------------------------------------------------------

TEST(MeshOps, GeodesicMatchesGreatCircle) {
  const geometry s = icosphere(4, 1.0);
  cvc::index_t src = 0;
  for (std::size_t i = 0; i < s.num_points(); ++i)
    if (s.const_points()[i][2] > s.const_points()[src][2])
      src = i;
  const cvc::geodesic_solver solver(s);
  EXPECT_FALSE(solver.empty());
  const std::vector<double> d = solver.distance({src});
  ASSERT_EQ(d.size(), s.num_points());
  EXPECT_EQ(d[src], 0.0);
  const auto &ps = s.const_points()[src];
  double worst = 0;
  for (std::size_t i = 0; i < s.num_points(); ++i) {
    const auto &p = s.const_points()[i];
    const double arc = std::acos(std::clamp(p[0] * ps[0] + p[1] * ps[1] + p[2] * ps[2], -1.0, 1.0));
    if (arc > 0.25)
      worst = std::max(worst, std::abs(d[i] - arc) / arc);
  }
  EXPECT_LT(worst, 0.10);
}

TEST(MeshOps, GeodesicUnreachableIsInfinite) {
  geometry two = icosphere(2, 1.0);
  const geometry other = icosphere(2, 1.0, {{5, 0, 0}});
  const cvc::index_t off = two.num_points();
  for (const auto &p : other.const_points())
    two.points().push_back(p);
  for (const auto &t : other.const_tris())
    two.tris().push_back({{t[0] + off, t[1] + off, t[2] + off}});
  const std::vector<double> d = cvc::geodesic_distance(two, {0});
  for (std::size_t i = 0; i < two.num_points(); ++i) {
    if (i < off) {
      EXPECT_TRUE(std::isfinite(d[i]) && d[i] >= 0 && d[i] < 3.5) << i;
    } else {
      EXPECT_TRUE(std::isinf(d[i])) << i;
    }
  }
  EXPECT_THROW(cvc::geodesic_distance(two, {two.num_points()}), cvc::mesh_ops_error);
}

// A connected part with no interior vertex leaves the heat method's Dirichlet
// solve without an unknown (a failed libigl assertion in Debug builds); it gets
// exact shortest paths along its edges.
TEST(MeshOps, GeodesicOnPartsWithoutInteriorVertices) {
  // A sphere plus a disjoint isolated triangle.
  geometry g = icosphere(2, 1.0);
  const cvc::index_t a = g.num_points();
  g.points().push_back({{3, 0, 0}});
  g.points().push_back({{6, 0, 0}});
  g.points().push_back({{3, 4, 0}});
  g.tris().push_back({{a, a + 1, a + 2}});
  const cvc::geodesic_solver solver(g);
  const std::vector<double> d = solver.distance({a});
  EXPECT_EQ(d[a], 0.0);
  EXPECT_NEAR(d[a + 1], 3.0, 1e-12);
  EXPECT_NEAR(d[a + 2], 4.0, 1e-12);
  for (cvc::index_t i = 0; i < a; ++i)
    EXPECT_TRUE(std::isinf(d[i])) << i;
  // The sphere still uses the heat method.
  const std::vector<double> ds = solver.distance({0});
  EXPECT_TRUE(std::isinf(ds[a]));
  for (cvc::index_t i = 1; i < a; ++i)
    EXPECT_TRUE(std::isfinite(ds[i]) && ds[i] > 0 && ds[i] < 3.5) << i;

  // A one-triangle-wide strip, two unit cells long: every vertex is on its
  // boundary.
  geometry strip;
  for (int j = 0; j < 2; ++j)
    for (int i = 0; i < 3; ++i)
      strip.points().push_back({{double(i), double(j), 0}});
  strip.tris().push_back({{0, 1, 4}});
  strip.tris().push_back({{0, 4, 3}});
  strip.tris().push_back({{1, 2, 5}});
  strip.tris().push_back({{1, 5, 4}});
  const std::vector<double> e = cvc::geodesic_distance(strip, {0});
  ASSERT_EQ(e.size(), 6u);
  EXPECT_NEAR(e[1], 1.0, 1e-12);
  EXPECT_NEAR(e[2], 2.0, 1e-12);
  EXPECT_NEAR(e[3], 1.0, 1e-12);
  EXPECT_NEAR(e[4], std::sqrt(2.0), 1e-12);
  EXPECT_NEAR(e[5], 1.0 + std::sqrt(2.0), 1e-12);
}

// ---- tetrahedral meshes ----------------------------------------------------------------------

TEST(MeshOps, OrientTetsAndVolumes) {
  const int n = 3;
  geometry cube = cube_tets(n);
  std::size_t flipped = 0;
  for (std::size_t i = 0; i < cube.num_tets(); ++i)
    if (hash_unit(i + 1000) < -0.2) {
      std::swap(cube.tets()[i][0], cube.tets()[i][1]);
      ++flipped;
    }
  ASSERT_GT(flipped, 0u);
  double total = 0;
  for (double v : cvc::tet_volumes(cube))
    total += std::abs(v);
  EXPECT_NEAR(total, 1.0, 1e-12);
  EXPECT_EQ(cvc::orient_tets(cube), flipped);
  const std::vector<double> vol = cvc::tet_volumes(cube);
  ASSERT_EQ(vol.size(), std::size_t(6 * n * n * n));
  for (double v : vol)
    EXPECT_NEAR(v, 1.0 / (6 * n * n * n), 1e-15);
  EXPECT_EQ(cvc::orient_tets(cube), 0u);
}

TEST(MeshOps, TetBoundarySurfaceIsOutward) {
  const int n = 4;
  geometry cube = cube_tets(n);
  for (std::size_t i = 0; i < cube.num_tets(); i += 3)
    std::swap(cube.tets()[i][2], cube.tets()[i][3]); // orientation must not matter
  for (const auto &p : cube.const_points())
    cube.functions().push_back(p[0] + p[1]);
  const geometry b = cvc::tet_boundary_surface(cube);
  EXPECT_EQ(b.get_geometry_type(), geometry::SURFACE_TRI);
  EXPECT_EQ(b.num_tris(), std::size_t(12 * n * n));
  EXPECT_EQ(b.num_points(), cube.num_points());
  EXPECT_EQ(b.num_tets(), 0u);
  ASSERT_EQ(b.const_functions().size(), b.num_points());
  EXPECT_NEAR(enclosed_volume(b), 1.0, 1e-12);
  EXPECT_NEAR(surface_area(b), 6.0, 1e-12);
  const auto &P = b.const_points();
  for (const auto &t : b.const_tris()) {
    const auto &a = P[t[0]], &c1 = P[t[1]], &c2 = P[t[2]];
    const double u[3] = {c1[0] - a[0], c1[1] - a[1], c1[2] - a[2]};
    const double w[3] = {c2[0] - a[0], c2[1] - a[1], c2[2] - a[2]};
    const double nrm[3] = {u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2],
                           u[0] * w[1] - u[1] * w[0]};
    double dot = 0;
    for (int c = 0; c < 3; ++c)
      dot += nrm[c] * ((a[c] + c1[c] + c2[c]) / 3 - 0.5);
    EXPECT_GT(dot, 0.0);
  }
}

TEST(MeshOps, SliceTetsCrossSection) {
  geometry cube = cube_tets(4);
  for (const auto &p : cube.const_points())
    cube.colors().push_back({{p[0], p[1], p[2]}});
  const geometry s = cvc::slice_tets(cube, {{0, 0, 2}}, 0.74); // z = 0.37
  ASSERT_GT(s.num_tris(), 0u);
  EXPECT_NEAR(surface_area(s), 1.0, 1e-12);
  ASSERT_EQ(s.const_colors().size(), s.num_points());
  for (std::size_t i = 0; i < s.num_points(); ++i) {
    EXPECT_NEAR(s.const_points()[i][2], 0.37, 1e-12);
    for (int c = 0; c < 3; ++c)
      EXPECT_NEAR(s.const_colors()[i][c], s.const_points()[i][c], 1e-12);
  }
  // Faces point along the plane normal.
  const auto &P = s.const_points();
  for (const auto &t : s.const_tris()) {
    const double nz = (P[t[1]][0] - P[t[0]][0]) * (P[t[2]][1] - P[t[0]][1]) -
                      (P[t[1]][1] - P[t[0]][1]) * (P[t[2]][0] - P[t[0]][0]);
    EXPECT_GT(nz, 0.0);
  }
  EXPECT_THROW(cvc::slice_tets(cube, {{0, 0, 0}}, 0.0), cvc::mesh_ops_error);
}

TEST(MeshOps, TetIsosurfaceOfLinearFieldIsPlanar) {
  geometry cube = cube_tets(5);
  std::vector<double> f;
  for (const auto &p : cube.const_points())
    f.push_back(p[0] + 2 * p[1] + 3 * p[2]);
  cube.functions() = f;
  const geometry iso = cvc::tet_isosurface(cube, f, 2.15);
  ASSERT_GT(iso.num_tris(), 0u);
  ASSERT_EQ(iso.const_functions().size(), iso.num_points());
  for (std::size_t i = 0; i < iso.num_points(); ++i) {
    const auto &p = iso.const_points()[i];
    EXPECT_NEAR(p[0] + 2 * p[1] + 3 * p[2], 2.15, 1e-12);
    EXPECT_NEAR(iso.const_functions()[i], 2.15, 1e-12);
  }
  EXPECT_THROW(cvc::tet_isosurface(cube, std::vector<double>(3, 0.0), 0.0), cvc::mesh_ops_error);
  // A level the field never reaches gives an empty surface.
  EXPECT_EQ(cvc::tet_isosurface(cube, f, 100.0).num_tris(), 0u);
}

// ---- colour maps -------------------------------------------------------------------------------

TEST(MeshOps, ColormapEndpointsAndNaN) {
  const std::vector<double> v = {0.0, 1.0, std::numeric_limits<double>::quiet_NaN(), 0.5};
  const std::vector<unsigned char> c = cvc::colormap_rgb(v, cvc::colormap_kind::VIRIDIS);
  ASSERT_EQ(c.size(), 12u);
  EXPECT_NEAR(c[0], 68, 1);
  EXPECT_NEAR(c[1], 1, 1);
  EXPECT_NEAR(c[2], 84, 1);
  EXPECT_NEAR(c[3], 253, 1);
  EXPECT_NEAR(c[4], 231, 1);
  EXPECT_NEAR(c[5], 37, 1);
  EXPECT_EQ(c[6], 128);
  EXPECT_EQ(c[7], 128);
  EXPECT_EQ(c[8], 128);

  const std::vector<unsigned char> g =
      cvc::colormap_rgb({-5.0, 0.0, 5.0, 10.0}, cvc::colormap_kind::GRAY, 0.0, 10.0);
  EXPECT_EQ(g[0], 0);   // clamped below lo
  EXPECT_EQ(g[3], 0);   // lo
  EXPECT_EQ(g[6], 128); // midway
  EXPECT_EQ(g[9], 255); // hi
  const std::vector<unsigned char> jet = cvc::colormap_rgb({0.0, 1.0}, cvc::colormap_kind::JET);
  EXPECT_EQ(jet[0], 0);
  EXPECT_EQ(jet[2], 128);
  EXPECT_EQ(jet[3], 128);
  EXPECT_EQ(jet[5], 0);
  // A constant field maps to the middle of the map instead of dividing by zero.
  const std::vector<unsigned char> flat = cvc::colormap_rgb({3.0, 3.0}, cvc::colormap_kind::GRAY);
  EXPECT_EQ(flat[0], 128);
}

TEST(MeshOps, ColormapNames) {
  cvc::colormap_kind k = cvc::colormap_kind::GRAY;
  EXPECT_TRUE(cvc::colormap_from_string("Turbo", k));
  EXPECT_EQ(k, cvc::colormap_kind::TURBO);
  EXPECT_FALSE(cvc::colormap_from_string("rainbow", k));
  EXPECT_EQ(k, cvc::colormap_kind::TURBO);
  for (auto kind :
       {cvc::colormap_kind::VIRIDIS, cvc::colormap_kind::MAGMA, cvc::colormap_kind::PLASMA,
        cvc::colormap_kind::INFERNO, cvc::colormap_kind::TURBO, cvc::colormap_kind::JET,
        cvc::colormap_kind::PARULA, cvc::colormap_kind::GRAY}) {
    cvc::colormap_kind back;
    ASSERT_TRUE(cvc::colormap_from_string(cvc::to_string(kind), back));
    EXPECT_EQ(back, kind);
  }
}

TEST(MeshOps, RobustRange) {
  std::vector<double> v;
  for (int i = 0; i <= 100; ++i)
    v.push_back(i);
  v.push_back(1e9);
  v.push_back(-1e9);
  v.push_back(std::numeric_limits<double>::infinity());
  v.push_back(std::numeric_limits<double>::quiet_NaN());
  const auto r = cvc::robust_range(v, 0.05, 0.95);
  EXPECT_NEAR(r.first, 5.0, 1.0);
  EXPECT_NEAR(r.second, 95.0, 1.0);
  const auto all = cvc::robust_range(v, 0.0, 1.0);
  EXPECT_EQ(all.first, -1e9);
  EXPECT_EQ(all.second, 1e9);
  const auto none = cvc::robust_range({std::numeric_limits<double>::quiet_NaN()});
  EXPECT_TRUE(std::isnan(none.first) && std::isnan(none.second));
}

// ---- integration with cvc::sdf and geometry::project -----------------------------------------

#ifdef CVC_ENABLE_SDF
TEST(MeshOps, SdfIglMatchesAnalyticSphere) {
  cvc::app ctx;
  const double r = 0.6;
  const geometry s = icosphere(4, r, {{0.1, -0.05, 0.0}});
  const cvc::dimension dim(17, 12, 9);
  const cvc::bounding_box bbox(-1.0, -0.9, -0.8, 1.0, 0.9, 0.8);
  const cvc::volume vol = cvc::sdf(ctx, s, dim, bbox, cvc::SDF_IGL);
  ASSERT_EQ(vol.XDim(), 17u);
  ASSERT_EQ(vol.ZDim(), 9u);
  const double span = std::min({vol.XSpan(), vol.YSpan(), vol.ZSpan()});
  for (cvc::uint64 k = 0; k < dim.zdim; ++k)
    for (cvc::uint64 j = 0; j < dim.ydim; ++j)
      for (cvc::uint64 i = 0; i < dim.xdim; ++i) {
        const double x = vol.XMin() + double(i) * vol.XSpan() - 0.1;
        const double y = vol.YMin() + double(j) * vol.YSpan() + 0.05;
        const double z = vol.ZMin() + double(k) * vol.ZSpan();
        const double exact = std::sqrt(x * x + y * y + z * z) - r;
        const double got = vol(i, j, k);
        EXPECT_NEAR(got, exact, 0.05 * span) << i << "," << j << "," << k;
        if (std::abs(exact) > 0.01) {
          EXPECT_EQ(got < 0, exact < 0);
        }
      }
  const cvc::volume flipped = cvc::sdf(ctx, s, dim, bbox, cvc::SDF_IGL, true);
  EXPECT_EQ(flipped(8, 6, 4), -vol(8, 6, 4));
}

// A planar (z0 == z1) box is a 2-D slice, not "no box": only the
// default-constructed, all-zero box means the geometry's extents.
TEST(MeshOps, SdfIglPlanarSliceKeepsItsBox) {
  cvc::app ctx;
  const double r = 0.6;
  const geometry s = icosphere(4, r);
  const cvc::dimension dim(9, 7, 1);
  const cvc::bounding_box bbox(-0.5, -0.4, 0.2, 0.7, 0.5, 0.2);
  const cvc::volume vol = cvc::sdf(ctx, s, dim, bbox, cvc::SDF_IGL);
  ASSERT_EQ(vol.XDim(), 9u);
  ASSERT_EQ(vol.YDim(), 7u);
  ASSERT_EQ(vol.ZDim(), 1u);
  EXPECT_EQ(vol.XMin(), -0.5);
  EXPECT_EQ(vol.XMax(), 0.7);
  EXPECT_EQ(vol.YMin(), -0.4);
  EXPECT_EQ(vol.YMax(), 0.5);
  EXPECT_EQ(vol.ZMin(), 0.2);
  EXPECT_EQ(vol.ZMax(), 0.2);
  for (cvc::uint64 j = 0; j < 7; ++j)
    for (cvc::uint64 i = 0; i < 9; ++i) {
      const double x = -0.5 + 1.2 * double(i) / 8, y = -0.4 + 0.9 * double(j) / 6, z = 0.2;
      EXPECT_NEAR(vol(i, j, 0), std::sqrt(x * x + y * y + z * z) - r, 2e-3) << i << "," << j;
    }
  // The default box still means the geometry's extents ([-r, r]^3 here).
  const cvc::volume full =
      cvc::sdf(ctx, s, cvc::dimension(5, 5, 5), cvc::bounding_box(), cvc::SDF_IGL);
  EXPECT_NEAR(full.XMin(), -r, 1e-12);
  EXPECT_NEAR(full.ZMax(), r, 1e-12);
}
#endif

TEST(MeshOps, ProjectLandsOnTheSurface) {
  const geometry s = icosphere(3, 1.0);
  geometry pts;
  for (int i = 0; i < 300; ++i) {
    geometry::point_t p = {{hash_unit(3 * i + 5), hash_unit(3 * i + 6), hash_unit(3 * i + 7)}};
    const double l = norm3(p);
    const double rad = 0.5 + 1.5 * (0.5 + 0.5 * hash_unit(i + 99));
    pts.points().push_back({{rad * p[0] / l, rad * p[1] / l, rad * p[2] / l}});
  }
  pts.project(s);
  const cvc::mesh_locator loc(s);
  const cvc::closest_point_result r = loc.closest_points(pts.const_points());
  for (std::size_t i = 0; i < pts.num_points(); ++i) {
    EXPECT_LT(r.sq_distance[i], 1e-24);
    EXPECT_NEAR(norm3(pts.const_points()[i]), 1.0, 0.01);
  }
}
