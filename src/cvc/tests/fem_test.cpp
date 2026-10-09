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

// fem_test -- cvc::fem, linear (P1) finite elements on tet meshes and triangle
// surfaces.
//
// Fixtures are generated here: the unit cube cut into Kuhn tets (6 per cell)
// and subdivided icospheres. Covers the assembled operators (symmetry, zero row
// sums, total mass), boundary vertices, gradients of a linear field, exact P1
// reproduction of a linear harmonic function, O(h^2) convergence of Poisson,
// heat diffusion (mass conservation without boundary conditions, convergence
// to the harmonic solution with them, the progress callback), the Laplace-
// Beltrami spectrum of a sphere on both eigensolver paths, zero modes of a
// mesh in several parts, degenerate elements, slivers too flat for cotangent
// weights, unreferenced vertices, and argument checking.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cvc/geometry/fem.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/mesh_ops.h>
#include <gtest/gtest.h>
#include <map>
#include <utility>
#include <vector>

using cvc::geometry;
namespace fem = cvc::fem;

namespace {

double hash_unit(std::uint64_t i) {
  i = (i ^ (i >> 33)) * 0xff51afd7ed558ccdULL;
  i = (i ^ (i >> 33)) * 0xc4ceb9fe1a85ec53ULL;
  i ^= i >> 33;
  return double(i >> 11) / double(1ULL << 52) - 1.0;
}

// n^3 cells of the cube [0,L]^3 (corner at `o`), six Kuhn tets per cell.
geometry cube_tets(int n, double L = 1.0, geometry::point_t o = {{0, 0, 0}}) {
  geometry g;
  const auto id = [n](int i, int j, int k) {
    return cvc::index_t(i + (n + 1) * (j + (n + 1) * k));
  };
  for (int k = 0; k <= n; ++k)
    for (int j = 0; j <= n; ++j)
      for (int i = 0; i <= n; ++i)
        g.points().push_back({{o[0] + L * i / n, o[1] + L * j / n, o[2] + L * k / n}});
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
          if (p >= 3)
            std::swap(t[2], t[3]);
          g.tets().push_back(t);
        }
  return g;
}

geometry icosphere(int levels, double r) {
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
    g.points().push_back({{s * p[0], s * p[1], s * p[2]}});
  }
  for (const auto &f : F)
    g.tris().push_back({{cvc::index_t(f[0]), cvc::index_t(f[1]), cvc::index_t(f[2])}});
  return g;
}

// The 5 x 5 grid [-2, 2]^2 in z = 0 with a near-collinear cap spliced in
// (as in mesh_ops_test): vertex 25 = (0.5, 1e-9, 0) sits just above the edge
// (0,0)-(1,0), and the triangle (0,0),(1,0),25 has edge lengths that round to
// exactly 0.5, 0.5 and 1 -- a zero edge-length (Heron) area, so infinite
// cotangent weights in libigl, though its cross-product area is 5e-10.
geometry capped_grid() {
  geometry g;
  const auto id = [](int i, int j) { return cvc::index_t((i + 2) + 5 * (j + 2)); };
  for (int j = -2; j <= 2; ++j)
    for (int i = -2; i <= 2; ++i)
      g.points().push_back({{double(i), double(j), 0.0}});
  g.points().push_back({{0.5, 1e-9, 0.0}});
  const cvc::index_t m = 25;
  for (int j = -2; j < 2; ++j)
    for (int i = -2; i < 2; ++i) {
      const cvc::index_t a = id(i, j), b = id(i + 1, j), c = id(i + 1, j + 1), d = id(i, j + 1);
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

// Append b's points and elements to a.
void append(geometry &a, const geometry &b) {
  const cvc::index_t off = a.num_points();
  for (const auto &p : b.const_points())
    a.points().push_back(p);
  for (const auto &t : b.const_tris())
    a.tris().push_back({{t[0] + off, t[1] + off, t[2] + off}});
  for (const auto &t : b.const_tets())
    a.tets().push_back({{t[0] + off, t[1] + off, t[2] + off, t[3] + off}});
}

std::map<std::pair<std::size_t, std::size_t>, double> entries(const fem::sparse_matrix &A) {
  std::map<std::pair<std::size_t, std::size_t>, double> m;
  for (std::size_t k = 0; k < A.value.size(); ++k)
    m[{A.row[k], A.col[k]}] += A.value[k];
  return m;
}

std::vector<double> diagonal(const fem::sparse_matrix &A) {
  std::vector<double> d(A.rows, 0.0);
  for (std::size_t k = 0; k < A.value.size(); ++k)
    if (A.row[k] == A.col[k])
      d[A.row[k]] += A.value[k];
  return d;
}

double weighted_sum(const std::vector<double> &m, const std::vector<double> &u) {
  double s = 0;
  for (std::size_t i = 0; i < u.size(); ++i)
    s += m[i] * u[i];
  return s;
}

fem::dirichlet boundary_condition(const geometry &g, double (*f)(const geometry::point_t &)) {
  fem::dirichlet bc;
  bc.vertices = fem::boundary_vertices(g);
  for (cvc::index_t v : bc.vertices)
    bc.values.push_back(f(g.const_points()[v]));
  return bc;
}

double linear(const geometry::point_t &p) { return 1.0 + 2.0 * p[0] - p[1] + 0.5 * p[2]; }
double quadratic(const geometry::point_t &p) { return p[0] * p[0] + p[1] * p[1] + p[2] * p[2]; }
double smooth_field(const geometry::point_t &p) {
  return std::exp(p[0] + 0.5 * p[1]) * std::cos(p[2]);
}

} // namespace

TEST(Fem, OperatorsOnCube) {
  const geometry cube = cube_tets(3);
  const std::size_t n = cube.num_points();
  const fem::sparse_matrix K = fem::stiffness_matrix(cube);
  ASSERT_EQ(K.rows, n);
  ASSERT_EQ(K.cols, n);
  const auto k = entries(K);
  std::vector<double> rowsum(n, 0.0);
  for (const auto &e : k) {
    rowsum[e.first.first] += e.second;
    const auto t = k.find({e.first.second, e.first.first});
    ASSERT_NE(t, k.end());
    EXPECT_NEAR(t->second, e.second, 1e-12);
    if (e.first.first == e.first.second) {
      EXPECT_GT(e.second, 0.0);
    }
  }
  for (double s : rowsum)
    EXPECT_NEAR(s, 0.0, 1e-12);

  const fem::sparse_matrix M = fem::mass_matrix(cube);
  double total = 0;
  for (std::size_t i = 0; i < M.value.size(); ++i) {
    EXPECT_EQ(M.row[i], M.col[i]); // lumped: diagonal
    EXPECT_GT(M.value[i], 0.0);
    total += M.value[i];
  }
  EXPECT_NEAR(total, 1.0, 1e-12);
  const fem::sparse_matrix Mc = fem::mass_matrix(cube, fem::domain::AUTO, false);
  double ctotal = 0;
  for (double v : Mc.value)
    ctotal += v;
  EXPECT_NEAR(ctotal, 1.0, 1e-12);
  EXPECT_GT(Mc.value.size(), M.value.size());
}

TEST(Fem, SurfaceMassIsArea) {
  const geometry s = icosphere(2, 1.0);
  double area = 0;
  for (const auto &t : s.const_tris()) {
    const auto &a = s.const_points()[t[0]], &b = s.const_points()[t[1]],
               &c = s.const_points()[t[2]];
    const double u[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
    const double w[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
    area += 0.5 * std::sqrt(std::pow(u[1] * w[2] - u[2] * w[1], 2) +
                            std::pow(u[2] * w[0] - u[0] * w[2], 2) +
                            std::pow(u[0] * w[1] - u[1] * w[0], 2));
  }
  double total = 0;
  for (double v : fem::mass_matrix(s).value)
    total += v;
  EXPECT_NEAR(total, area, 1e-12);
  EXPECT_TRUE(fem::boundary_vertices(s).empty()); // closed surface
}

TEST(Fem, BoundaryVerticesOfCube) {
  const int n = 4;
  const geometry cube = cube_tets(n);
  const std::vector<cvc::index_t> b = fem::boundary_vertices(cube);
  EXPECT_EQ(b.size(), std::size_t((n + 1) * (n + 1) * (n + 1) - (n - 1) * (n - 1) * (n - 1)));
  EXPECT_TRUE(std::is_sorted(b.begin(), b.end()));
  for (cvc::index_t v : b) {
    const auto &p = cube.const_points()[v];
    bool on = false;
    for (int c = 0; c < 3; ++c)
      on = on || p[c] == 0.0 || p[c] == 1.0;
    EXPECT_TRUE(on);
  }
}

TEST(Fem, GradientOfLinearFieldIsExact) {
  geometry cube = cube_tets(3);
  for (std::size_t i = 0; i < cube.num_tets(); i += 2)
    std::swap(cube.tets()[i][2], cube.tets()[i][3]); // orientation must not matter
  std::vector<double> u;
  for (const auto &p : cube.const_points())
    u.push_back(linear(p));
  const std::vector<double> g = fem::gradient(cube, u);
  ASSERT_EQ(g.size(), 3 * cube.num_tets());
  for (std::size_t e = 0; e < cube.num_tets(); ++e) {
    EXPECT_NEAR(g[3 * e], 2.0, 1e-10);
    EXPECT_NEAR(g[3 * e + 1], -1.0, 1e-10);
    EXPECT_NEAR(g[3 * e + 2], 0.5, 1e-10);
  }
}

TEST(Fem, LaplaceReproducesLinearFunction) {
  geometry cube = cube_tets(4);
  for (std::size_t i = 0; i < cube.num_tets(); i += 3)
    std::swap(cube.tets()[i][0], cube.tets()[i][1]);
  const std::vector<double> u = fem::solve_laplace(cube, boundary_condition(cube, linear));
  ASSERT_EQ(u.size(), cube.num_points());
  for (std::size_t i = 0; i < u.size(); ++i)
    EXPECT_NEAR(u[i], linear(cube.const_points()[i]), 1e-10);
}

TEST(Fem, PoissonReproducesQuadraticOnKuhnMesh) {
  // u = x^2 + y^2 + z^2, -lap(u) = -6: nodally exact on this structured mesh.
  const geometry cube = cube_tets(4);
  const std::vector<double> u =
      fem::solve_poisson(cube, {-6.0}, boundary_condition(cube, quadratic));
  for (std::size_t i = 0; i < u.size(); ++i)
    EXPECT_NEAR(u[i], quadratic(cube.const_points()[i]), 1e-10);
}

TEST(Fem, PoissonConvergesQuadratically) {
  // u = exp(x + y/2) cos(z): lap(u) = u / 4, so f = -u / 4.
  double err[2];
  for (int r = 0; r < 2; ++r) {
    const geometry cube = cube_tets(r == 0 ? 4 : 8);
    std::vector<double> f;
    for (const auto &p : cube.const_points())
      f.push_back(-0.25 * smooth_field(p));
    const std::vector<double> u =
        fem::solve_poisson(cube, f, boundary_condition(cube, smooth_field));
    err[r] = 0;
    for (std::size_t i = 0; i < u.size(); ++i)
      err[r] = std::max(err[r], std::abs(u[i] - smooth_field(cube.const_points()[i])));
  }
  EXPECT_LT(err[0], 0.01);
  EXPECT_GT(err[1], 0.0);
  EXPECT_LT(err[1], 0.4 * err[0]);
}

TEST(Fem, HeatConservesMassWithoutBoundaryConditions) {
  const geometry cube = cube_tets(3);
  std::vector<double> u0;
  for (std::size_t i = 0; i < cube.num_points(); ++i)
    u0.push_back(1.0 + hash_unit(i));
  const std::vector<double> m = diagonal(fem::mass_matrix(cube));
  fem::heat_params p;
  p.steps = 5;
  p.dt = 0.01;
  const std::vector<double> u = fem::solve_heat(cube, u0, p);
  EXPECT_NEAR(weighted_sum(m, u), weighted_sum(m, u0), 1e-12);
  const auto spread = [](const std::vector<double> &v) {
    return *std::max_element(v.begin(), v.end()) - *std::min_element(v.begin(), v.end());
  };
  EXPECT_LT(spread(u), 0.5 * spread(u0));

  // The callback sees every step and can stop early.
  std::vector<int> seen;
  const std::vector<double> u3 = fem::solve_heat(cube, u0, p, fem::dirichlet(), fem::domain::AUTO,
                                                 [&seen](int step, const std::vector<double> &) {
                                                   seen.push_back(step);
                                                   return step < 3;
                                                 });
  EXPECT_EQ(seen, (std::vector<int>{1, 2, 3}));
  p.steps = 3;
  const std::vector<double> direct = fem::solve_heat(cube, u0, p);
  for (std::size_t i = 0; i < u3.size(); ++i)
    EXPECT_NEAR(u3[i], direct[i], 1e-14);
}

TEST(Fem, HeatWithBoundaryConditionsTendsToLaplace) {
  const geometry cube = cube_tets(3);
  const fem::dirichlet bc = boundary_condition(cube, linear);
  fem::heat_params p;
  p.kappa = 2.0;
  p.dt = 1.0;
  p.steps = 40;
  const std::vector<double> u =
      fem::solve_heat(cube, std::vector<double>(cube.num_points(), 0.0), p, bc);
  for (std::size_t i = 0; i < u.size(); ++i)
    EXPECT_NEAR(u[i], linear(cube.const_points()[i]), 1e-6);
}

TEST(Fem, EigenmodesOfSphere) {
  const double r = 1.5;
  // 162 vertices: the dense eigensolver. 2562: the iterative one.
  for (int levels : {2, 4}) {
    const geometry s = icosphere(levels, r);
    const fem::eigen_result e = fem::laplacian_eigenmodes(s, 5);
    ASSERT_EQ(e.values.size(), 5u);
    ASSERT_EQ(e.vectors.size(), 5u);
    EXPECT_NEAR(e.values[0], 0.0, 1e-8);
    for (int i = 1; i < 4; ++i)
      EXPECT_NEAR(e.values[i], 2.0 / (r * r), 0.05 * 2.0 / (r * r)) << levels << ":" << i;
    EXPECT_NEAR(e.values[4], 6.0 / (r * r), 0.05 * 6.0 / (r * r)) << levels;
    EXPECT_TRUE(std::is_sorted(e.values.begin(), e.values.end()));
    const std::vector<double> m = diagonal(fem::mass_matrix(s));
    for (int i = 0; i < 5; ++i)
      for (int j = 0; j < 5; ++j) {
        double dot = 0;
        for (std::size_t v = 0; v < m.size(); ++v)
          dot += m[v] * e.vectors[i][v] * e.vectors[j][v];
        EXPECT_NEAR(dot, i == j ? 1.0 : 0.0, 1e-6) << levels << ":" << i << "," << j;
      }
  }
}

// The zero modes (one per connected part) only settle to round-off, so a
// purely relative stopping test never accepted them: with k <= #parts the
// iterative solver (above 400 vertices) used to run all 500 iterations.
TEST(Fem, EigenmodesZeroModesOfSeveralParts) {
  geometry two = icosphere(3, 1.0); // 642 vertices each
  geometry other = icosphere(3, 1.0);
  for (auto &p : other.points())
    p[0] += 5.0;
  append(two, other);
  const std::size_t half = two.num_points() / 2;
  const std::vector<double> m = diagonal(fem::mass_matrix(two));
  for (int k : {1, 2, 3}) {
    const fem::eigen_result e = fem::laplacian_eigenmodes(two, k);
    ASSERT_EQ(e.values.size(), std::size_t(k));
    for (int i = 0; i < k; ++i) {
      if (i < 2) {
        EXPECT_NEAR(e.values[i], 0.0, 1e-8) << k << ":" << i;
        // Constant on each sphere.
        for (std::size_t part = 0; part < 2; ++part) {
          const auto lo = e.vectors[i].begin() + std::ptrdiff_t(part * half);
          const auto mm = std::minmax_element(lo, lo + std::ptrdiff_t(half));
          EXPECT_NEAR(*mm.first, *mm.second, 1e-6) << k << ":" << i << ":" << part;
        }
      } else {
        EXPECT_NEAR(e.values[i], 2.0, 0.05 * 2.0) << k; // l = 1 on the unit sphere
      }
      double norm = 0;
      for (std::size_t v = 0; v < m.size(); ++v)
        norm += m[v] * e.vectors[i][v] * e.vectors[i][v];
      EXPECT_NEAR(norm, 1.0, 1e-8) << k << ":" << i;
    }
  }
}

// Elements too flat for libigl's edge-length cotangent formulas are dropped
// like degenerate ones, instead of putting inf/NaN into the operators.
TEST(Fem, SliverElementsAreDropped) {
  const geometry grid = capped_grid();
  const fem::sparse_matrix K = fem::stiffness_matrix(grid);
  std::vector<double> rowsum(grid.num_points(), 0.0);
  for (std::size_t i = 0; i < K.value.size(); ++i) {
    ASSERT_TRUE(std::isfinite(K.value[i])) << i;
    rowsum[K.row[i]] += K.value[i];
  }
  for (double s : rowsum)
    EXPECT_NEAR(s, 0.0, 1e-12);
  for (double v : fem::mass_matrix(grid).value)
    EXPECT_TRUE(std::isfinite(v));
  const std::vector<double> u = fem::solve_laplace(grid, boundary_condition(grid, linear));
  for (double v : u)
    EXPECT_TRUE(std::isfinite(v));

  // A sliver tet (|6 V| / l^3 ~ 4e-10, but |V| above the 1e-12 extent
  // tolerance) as a separate part of a cube: no entry of K touches its
  // vertices, and its gradient reads 0 like any dropped element's.
  geometry cube = cube_tets(2);
  const cvc::index_t o = cube.num_points();
  cube.points().push_back({{3, 0, 0}});
  cube.points().push_back({{4, 0, 0}});
  cube.points().push_back({{3, 1, 0}});
  cube.points().push_back({{3.25, 0.25, 1e-9}});
  cube.tets().push_back({{o, o + 1, o + 2, o + 3}});
  const fem::sparse_matrix Kt = fem::stiffness_matrix(cube);
  for (std::size_t i = 0; i < Kt.value.size(); ++i) {
    EXPECT_TRUE(std::isfinite(Kt.value[i])) << i;
    EXPECT_LT(Kt.row[i], std::size_t(o));
  }
  const std::vector<double> g = fem::gradient(cube, std::vector<double>(cube.num_points(), 1.0));
  for (int c = 0; c < 3; ++c)
    EXPECT_EQ(g[3 * (cube.num_tets() - 1) + std::size_t(c)], 0.0);
}

TEST(Fem, VolumeModesOfCubeAreNeumann) {
  // Neumann Laplacian on the unit cube: 0, then pi^2 three times.
  const geometry cube = cube_tets(8);
  const fem::eigen_result e = fem::laplacian_eigenmodes(cube, 4);
  ASSERT_EQ(e.values.size(), 4u);
  EXPECT_NEAR(e.values[0], 0.0, 1e-8);
  const double pi2 = 9.8696044010893586;
  for (int i = 1; i < 4; ++i)
    EXPECT_NEAR(e.values[i], pi2, 0.05 * pi2) << i;
}

TEST(Fem, DegenerateElementsAndOrphanVertices) {
  geometry cube = cube_tets(2);
  const std::size_t tets = cube.num_tets();
  // A flat tet on the z = 0 face, and a vertex nothing uses.
  cube.tets().push_back({{0, 1, 3, 4}});
  cube.points().push_back({{7, 7, 7}});
  const std::size_t orphan = cube.num_points() - 1;

  const fem::sparse_matrix K = fem::stiffness_matrix(cube);
  for (double v : K.value)
    EXPECT_TRUE(std::isfinite(v));
  std::vector<double> u0;
  for (const auto &p : cube.const_points())
    u0.push_back(linear(p));
  const std::vector<double> g = fem::gradient(cube, u0);
  ASSERT_EQ(g.size(), 3 * (tets + 1));
  EXPECT_EQ(g[3 * tets], 0.0);
  EXPECT_NEAR(g[0], 2.0, 1e-10);

  fem::dirichlet bc = boundary_condition(cube, linear);
  const std::vector<double> u = fem::solve_laplace(cube, bc);
  EXPECT_EQ(u[orphan], 0.0);
  for (std::size_t i = 0; i < orphan; ++i)
    EXPECT_NEAR(u[i], linear(cube.const_points()[i]), 1e-10);
  // Held at its Dirichlet value when it has one.
  bc.vertices.push_back(orphan);
  bc.values.push_back(5.0);
  EXPECT_EQ(fem::solve_laplace(cube, bc)[orphan], 5.0);
}

TEST(Fem, DomainSelection) {
  geometry both = cube_tets(2);
  both.tris().push_back({{0, 1, 3}});
  EXPECT_EQ(fem::gradient(both, std::vector<double>(both.num_points(), 1.0)).size(),
            3 * both.num_tets());
  EXPECT_EQ(
      fem::gradient(both, std::vector<double>(both.num_points(), 1.0), fem::domain::SURFACE).size(),
      3u);
  const geometry sphere = icosphere(1, 1.0);
  EXPECT_THROW(fem::stiffness_matrix(sphere, fem::domain::VOLUME), cvc::mesh_ops_error);
}

TEST(Fem, ArgumentChecks) {
  const geometry cube = cube_tets(2);
  const fem::dirichlet bc = boundary_condition(cube, linear);
  EXPECT_THROW(fem::solve_poisson(cube, {1.0}, fem::dirichlet()), cvc::mesh_ops_error);
  EXPECT_THROW(fem::solve_laplace(cube, fem::dirichlet()), cvc::mesh_ops_error);
  EXPECT_THROW(fem::solve_poisson(cube, {1.0, 2.0}, bc), cvc::mesh_ops_error);
  fem::dirichlet bad = bc;
  bad.vertices.push_back(cube.num_points());
  bad.values.push_back(0.0);
  EXPECT_THROW(fem::solve_laplace(cube, bad), cvc::mesh_ops_error);
  fem::dirichlet uneven = bc;
  uneven.values.pop_back();
  EXPECT_THROW(fem::solve_laplace(cube, uneven), cvc::mesh_ops_error);
  // One value is broadcast.
  fem::dirichlet one;
  one.vertices = bc.vertices;
  one.values = {3.0};
  for (double v : fem::solve_laplace(cube, one))
    EXPECT_NEAR(v, 3.0, 1e-10);
  fem::heat_params hp;
  hp.dt = 0.0;
  EXPECT_THROW(fem::solve_heat(cube, std::vector<double>(cube.num_points(), 0.0), hp),
               cvc::mesh_ops_error);
  EXPECT_THROW(fem::gradient(cube, {1.0}), cvc::mesh_ops_error);
  EXPECT_THROW(fem::stiffness_matrix(geometry()), cvc::mesh_ops_error);

  // A second, disjoint cube with no Dirichlet vertex makes K singular.
  geometry two = cube_tets(1);
  const geometry other = cube_tets(1, 1.0, {{3, 0, 0}});
  const cvc::index_t off = two.num_points();
  for (const auto &p : other.const_points())
    two.points().push_back(p);
  for (const auto &t : other.const_tets())
    two.tets().push_back({{t[0] + off, t[1] + off, t[2] + off, t[3] + off}});
  fem::dirichlet half;
  half.vertices = {0};
  half.values = {1.0};
  EXPECT_THROW(fem::solve_laplace(two, half), cvc::mesh_ops_error);
}
