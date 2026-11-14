/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::tet_complex (cvc/geometry/tet_complex.h), the skeletonization port's Delaunay
// tetrahedralization (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §2, P1's first PR):
//   - T2: the complex is valid and exactly Delaunay (every circumsphere empty by the exact,
//     perturbed in-sphere test) on clouds in general position and on cospherical ones (a cube
//     and its centre, a lattice, points rounded onto a sphere), and its traversals agree with
//     brute force;
//   - the incremental build against the batch build: in general position it equals Qhull's
//     `d Qbb Qt Qz` triangulation tetrahedron for tetrahedron (with Qhull in the build);
//   - T4: the triangulation does not depend on the input order or the seed, even on degenerate
//     input (the perturbation makes it unique);
//   - T5: too few points, duplicates, collinear and coplanar points, NaN;
//   - T7: the builder through the resumable harness.
// Runs natively and under node (the ports lane).

#include "resumable_harness.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cvc/geometry/tet_complex.h>
#include <gtest/gtest.h>
#include <limits>
#include <map>
#include <numeric>
#include <random>
#include <set>
#include <string>
#include <vector>

#ifdef CVC_WITH_QHULL
extern "C" {
#include "libqhull_r/qhull_ra.h"
}
#endif

namespace {

using cvc::tet_complex;
using pts_t = std::vector<std::array<double, 3>>;
using tets_t = std::vector<std::array<int, 4>>;

pts_t uniform(int n, std::uint32_t seed) {
  std::mt19937 r(seed);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  pts_t p(static_cast<std::size_t>(n));
  for (auto &q : p)
    q = {u(r), u(r), u(r)};
  return p;
}

pts_t clump(int n, std::uint32_t seed) {
  std::mt19937 r(seed);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  pts_t p(static_cast<std::size_t>(n));
  for (auto &q : p)
    for (double &x : q) {
      double s = 0;
      for (int k = 0; k < 6; ++k)
        s += u(r);
      x = s;
    }
  return p;
}

pts_t lattice(int m) {
  pts_t p;
  for (int z = 0; z < m; ++z)
    for (int y = 0; y < m; ++y)
      for (int x = 0; x < m; ++x)
        p.push_back({double(x), double(y), double(z)});
  return p;
}

pts_t cube_and_centre() {
  pts_t p;
  for (int i = 0; i < 8; ++i)
    p.push_back({double(i & 1), double((i >> 1) & 1), double((i >> 2) & 1)});
  p.push_back({0.5, 0.5, 0.5});
  return p;
}

pts_t sphere(int n, std::uint32_t seed) { // rounded onto a unit sphere: nearly cospherical
  std::mt19937 r(seed);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  const double pi = 3.14159265358979323846;
  pts_t p;
  for (int i = 0; i < n; ++i) {
    const double th = std::acos(u(r)), ph = (u(r) + 1.0) * pi;
    p.push_back({std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph), std::cos(th)});
  }
  return p;
}

// The finite tetrahedra relabelled through `map` (new index -> old index), canonical again.
tets_t relabel(const tets_t &ts, const std::vector<int> &map) {
  tets_t out;
  for (auto t : ts) {
    for (int &v : t)
      v = map[std::size_t(v)];
    std::sort(t.begin(), t.end());
    out.push_back(t);
  }
  std::sort(out.begin(), out.end());
  return out;
}

std::string problems(const tet_complex &t, bool delaunay) {
  std::string s;
  for (const std::string &p : t.validate(delaunay))
    s += p + "\n";
  return s;
}

// Bit for bit: points, cells, duplicates, vertex cells and circumcentres.
bool same_complex(const tet_complex &a, const tet_complex &b) {
  if (a.num_points() != b.num_points() || a.num_cells() != b.num_cells() ||
      a.num_finite_cells() != b.num_finite_cells() || a.num_vertices() != b.num_vertices())
    return false;
  for (int c = 0; c < a.num_cells(); ++c) {
    if (a.cells()[std::size_t(c)].v != b.cells()[std::size_t(c)].v ||
        a.cells()[std::size_t(c)].n != b.cells()[std::size_t(c)].n)
      return false;
    if (a.has_circumcentres() != b.has_circumcentres())
      return false;
    if (a.has_circumcentres()) {
      const auto &p = a.circumcentre(c), &q = b.circumcentre(c);
      for (int k = 0; k < 3; ++k)
        if (!(p[std::size_t(k)] == q[std::size_t(k)] ||
              (std::isnan(p[std::size_t(k)]) && std::isnan(q[std::size_t(k)]))))
          return false;
    }
  }
  for (int v = 0; v <= a.num_points(); ++v) {
    if (a.incident_cell(v) != b.incident_cell(v))
      return false;
    if (v < a.num_points() && a.duplicate_of(v) != b.duplicate_of(v))
      return false;
  }
  return true;
}

#ifdef CVC_WITH_QHULL
// Qhull's batch Delaunay, as the ports plan runs it (`d Qbb Qt Qz`), as canonical tetrahedra.
tets_t qhull_batch(const pts_t &p) {
  qhT qh_storage;
  qhT *qh = &qh_storage;
  qh_zero(qh, stderr);
  std::vector<double> xyz;
  for (const auto &q : p)
    xyz.insert(xyz.end(), q.begin(), q.end());
  char cmd[] = "qhull d Qbb Qt Qz";
  const int n = int(p.size());
  tets_t out;
  const int exitcode = qh_new_qhull(qh, 3, n, xyz.data(), False, cmd, nullptr, stderr);
  EXPECT_EQ(exitcode, 0);
  if (exitcode == 0) {
    facetT *facet;
    vertexT *vertex, **vertexp;
    FORALLfacets {
      if (facet->upperdelaunay)
        continue;
      std::array<int, 4> t{};
      int k = 0;
      FOREACHvertex_(facet->vertices) {
        if (k < 4)
          t[std::size_t(k)] = qh_pointid(qh, vertex->point);
        ++k;
      }
      EXPECT_EQ(k, 4);
      std::sort(t.begin(), t.end());
      out.push_back(t);
    }
  }
  qh_freeqhull(qh, !qh_ALL);
  int curlong = 0, totlong = 0;
  qh_memfreeshort(qh, &curlong, &totlong);
  std::sort(out.begin(), out.end());
  return out;
}
#endif

class TetComplex : public ::testing::Test {
protected:
  void SetUp() override {
    if (!cvc::tet_complex_available())
      GTEST_SKIP() << "libcvc built without Shewchuk's predicates (CVC_WITH_SHEWCHUK)";
  }
};

// --- T2: small known complexes -------------------------------------------------------------

TEST_F(TetComplex, OneTetrahedron) {
  const tet_complex t = cvc::delaunay_tet_complex({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}});
  EXPECT_EQ(problems(t, true), "");
  EXPECT_EQ(t.num_vertices(), 4);
  EXPECT_EQ(t.num_finite_cells(), 1);
  EXPECT_EQ(t.num_cells(), 5); // and four infinite cells, one per hull facet
  EXPECT_FALSE(t.is_infinite_cell(0)) << "finite cells come first";
  for (int c = 1; c < 5; ++c)
    EXPECT_TRUE(t.is_infinite_cell(c));
  for (int v = 0; v < 4; ++v)
    EXPECT_TRUE(t.on_convex_hull(v));
  // The circumcentre of the corner tetrahedron is (1/2, 1/2, 1/2).
  ASSERT_TRUE(t.has_circumcentres());
  for (int k = 0; k < 3; ++k)
    EXPECT_DOUBLE_EQ(t.circumcentre(0)[std::size_t(k)], 0.5);
  EXPECT_TRUE(std::isnan(t.circumcentre(1)[0]));
}

TEST_F(TetComplex, CubeAndCentreGivesTwelveTetrahedra) {
  // Eight cospherical corners and the centre: twelve tetrahedra, two per face, each with the
  // centre (CGAL's answer, and the only Delaunay one: the centre is inside every corner sphere).
  const tet_complex t = cvc::delaunay_tet_complex(cube_and_centre());
  EXPECT_EQ(problems(t, true), "");
  EXPECT_EQ(t.num_finite_cells(), 12);
  for (const auto &q : t.finite_tetrahedra())
    EXPECT_EQ(q[3], 8) << "every tetrahedron has the centre";
  EXPECT_FALSE(t.on_convex_hull(8));
}

// --- T2 on clouds, with the incremental build against Qhull's batch build ------------------

void check_cloud(const std::string &name, const pts_t &p, bool delaunay_check,
                 bool general_position) {
  SCOPED_TRACE(name);
  cvc::tet_complex_builder b(p);
  ASSERT_TRUE(b.step(0));
  const cvc::tet_complex_stats st = b.stats();
  const tet_complex t = b.take();
  EXPECT_EQ(problems(t, delaunay_check), "");
  EXPECT_EQ(t.num_vertices(), int(p.size()));
  EXPECT_EQ(st.inserted, p.size());
  std::cout << name << ": " << t.num_finite_cells() << " finite cells, " << st.walk_steps
            << " walk steps, " << st.conflict_cells << " conflict cells (max " << st.max_conflict
            << "), " << st.perturbed << " ties broken by the perturbation\n";
#ifdef CVC_WITH_QHULL
  if (general_position) // one Delaunay triangulation: Qhull's batch build is it too
    EXPECT_EQ(t.finite_tetrahedra(), qhull_batch(p)) << "the incremental build equals the batch";
#else
  (void)general_position;
#endif
}

TEST_F(TetComplex, UniformCloudsAreDelaunayAndEqualTheBatchBuild) {
  check_cloud("uniform 300", uniform(300, 1), true, true);
  check_cloud("uniform 5000", uniform(5000, 2), false, true);
  check_cloud("clump 5000", clump(5000, 3), false, true);
}

TEST_F(TetComplex, CosphericalInputIsExactlyDelaunayWithNoFlatCells) {
  // Where Qhull's Qt leaves flat tetrahedra (the lattice) or merges nearly cospherical points
  // into a fan that is not Delaunay (the sphere), the exact build has neither: validate() checks
  // every cell positively oriented and every circumsphere empty, exactly.
  check_cloud("lattice 6^3", lattice(6), true, false);
  check_cloud("sphere 400", sphere(400, 4), true, false);
  {
    // A 4^3 lattice has 27 unit cubes, each with eight cospherical corners: the perturbation
    // splits each into five or six tetrahedra, all of volume 1/6 or 1/3, filling the 27.
    const tet_complex t = cvc::delaunay_tet_complex(lattice(4));
    EXPECT_EQ(problems(t, true), "");
    double vol = 0;
    for (const auto &q : t.finite_tetrahedra()) {
      const auto &a = t.point_of(q[0]), &b = t.point_of(q[1]), &c = t.point_of(q[2]),
                 &d = t.point_of(q[3]);
      const double u[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
      const double v[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
      const double w[3] = {d[0] - a[0], d[1] - a[1], d[2] - a[2]};
      const double det = u[0] * (v[1] * w[2] - v[2] * w[1]) - u[1] * (v[0] * w[2] - v[2] * w[0]) +
                         u[2] * (v[0] * w[1] - v[1] * w[0]);
      EXPECT_GT(std::fabs(det), 0.5) << "no flat or sliver cell on the lattice";
      vol += std::fabs(det); // an integer, exactly
    }
    EXPECT_EQ(vol, 27.0 * 6.0);
  }
}

// --- T2: traversals against brute force ----------------------------------------------------

TEST_F(TetComplex, TraversalsAgreeWithBruteForce) {
  const tet_complex t = cvc::delaunay_tet_complex(uniform(200, 5));
  ASSERT_EQ(problems(t, false), "");
  std::vector<int> got;
  for (int v = 0; v <= t.num_points(); ++v) {
    t.incident_cells(v, got);
    std::vector<int> want;
    for (int c = 0; c < t.num_cells(); ++c)
      if (t.has_vertex(c, v))
        want.push_back(c);
    std::sort(got.begin(), got.end());
    EXPECT_EQ(got, want) << "vertex " << v;
  }
  for (int c = 0; c < t.num_cells(); ++c)
    for (int i = 0; i < 4; ++i)
      for (int j = i + 1; j < 4; ++j) {
        t.cells_around_edge(c, i, j, got);
        const int s = t.vertex(c, i), u = t.vertex(c, j);
        std::vector<int> want;
        for (int d = 0; d < t.num_cells(); ++d)
          if (t.has_vertex(d, s) && t.has_vertex(d, u))
            want.push_back(d);
        ASSERT_FALSE(got.empty());
        EXPECT_EQ(got.front(), c);
        for (std::size_t k = 0; k < got.size(); ++k) // consecutive cells share a facet
          EXPECT_GE(t.neighbor_index(got[k], got[(k + 1) % got.size()]), 0);
        std::sort(got.begin(), got.end());
        EXPECT_EQ(got, want);
      }
  for (int c = 0; c < t.num_cells(); ++c)
    for (int i = 0; i < 4; ++i) {
      const int n = t.neighbor(c, i), m = t.mirror_index(c, i);
      EXPECT_EQ(t.neighbor(n, m), c);
      EXPECT_EQ(t.mirror_vertex(c, i), t.vertex(n, m));
      EXPECT_EQ(t.is_infinite_facet(c, i), t.is_infinite_facet(n, m));
    }
  // Circumcentres: equidistant from the four vertices.
  for (int c = 0; c < t.num_finite_cells(); ++c) {
    double r2 = 0;
    const auto o = t.compute_circumcentre(c, &r2);
    EXPECT_EQ(o, t.circumcentre(c));
    for (int i = 0; i < 4; ++i) {
      const auto &p = t.point_of(t.vertex(c, i));
      const double d2 = (p[0] - o[0]) * (p[0] - o[0]) + (p[1] - o[1]) * (p[1] - o[1]) +
                        (p[2] - o[2]) * (p[2] - o[2]);
      EXPECT_NEAR(d2, r2, 1e-9 * r2);
    }
  }
}

// --- T4: the order of the input and the seed do not matter ---------------------------------

void check_order_invariance(const std::string &name, const pts_t &p) {
  SCOPED_TRACE(name);
  const tets_t ref = cvc::delaunay_tet_complex(p).finite_tetrahedra();
  std::mt19937 r(7);
  for (int trial = 0; trial < 3; ++trial) {
    std::vector<int> perm(p.size());
    std::iota(perm.begin(), perm.end(), 0);
    std::shuffle(perm.begin(), perm.end(), r);
    pts_t q(p.size());
    for (std::size_t i = 0; i < p.size(); ++i)
      q[i] = p[std::size_t(perm[i])];
    cvc::tet_complex_options opt;
    opt.seed = 1000 + std::uint64_t(trial);
    const tet_complex t = cvc::delaunay_tet_complex(q, opt);
    EXPECT_EQ(relabel(t.finite_tetrahedra(), perm), ref) << "trial " << trial;
  }
}

TEST_F(TetComplex, TriangulationIsIndependentOfInputOrderAndSeed) {
  check_order_invariance("uniform 1000", uniform(1000, 8));
  check_order_invariance("lattice 5^3", lattice(5));
  check_order_invariance("sphere 300", sphere(300, 9));
  check_order_invariance("cube and centre", cube_and_centre());
}

// --- T5: bad input -------------------------------------------------------------------------

std::string build_error(const pts_t &p) {
  try {
    cvc::delaunay_tet_complex(p);
  } catch (const cvc::tet_complex_error &e) {
    return e.what();
  }
  return "";
}

TEST_F(TetComplex, BadInputThrowsWithAReason) {
  EXPECT_NE(build_error({}).find("0 points"), std::string::npos);
  EXPECT_NE(build_error({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}).find("3 points"), std::string::npos);
  EXPECT_NE(build_error(pts_t(10, {1, 2, 3})).find("the same point"), std::string::npos);
  pts_t line, plane;
  for (int i = 0; i < 20; ++i) {
    line.push_back({double(i), 2.0 * i, -1.0 * i});
    plane.push_back({double(i % 5), double(i / 5), 7.0});
  }
  EXPECT_NE(build_error(line).find("one line"), std::string::npos);
  EXPECT_NE(build_error(plane).find("one plane"), std::string::npos);
  pts_t bad = uniform(10, 10);
  bad[6][1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_NE(build_error(bad).find("point 6"), std::string::npos);
  bad[6][1] = std::numeric_limits<double>::infinity();
  EXPECT_NE(build_error(bad).find("not finite"), std::string::npos);
}

TEST_F(TetComplex, DuplicatesMapToTheirFirstPoint) {
  pts_t p = uniform(300, 11);
  const tets_t ref = cvc::delaunay_tet_complex(p).finite_tetrahedra();
  // Repeat some points, earlier and later than their originals.
  pts_t q;
  std::vector<int> original; // index in q -> index in p
  for (std::size_t i = 0; i < p.size(); ++i) {
    if (i % 7 == 3) {
      q.push_back(p[i]);
      original.push_back(int(i));
    }
    q.push_back(p[i]);
    original.push_back(int(i));
    if (i % 11 == 5) {
      q.push_back(p[i]);
      original.push_back(int(i));
      q.push_back(p[i]);
      original.push_back(int(i));
    }
  }
  cvc::tet_complex_builder b(q);
  b.step(0);
  EXPECT_GT(b.stats().duplicates, 0u);
  const tet_complex t = b.take();
  EXPECT_EQ(problems(t, true), "");
  EXPECT_EQ(t.num_vertices(), int(p.size()));
  for (int i = 0; i < t.num_points(); ++i) {
    // The vertex is the first of equal points.
    int first = i;
    while (first > 0 && original[std::size_t(first - 1)] == original[std::size_t(i)])
      --first;
    EXPECT_EQ(t.duplicate_of(i), first) << "point " << i;
    EXPECT_EQ(t.is_vertex(i), first == i);
  }
  EXPECT_EQ(relabel(t.finite_tetrahedra(), original), ref);
}

// --- T7: resumable -------------------------------------------------------------------------

TEST_F(TetComplex, BuilderIsResumable) {
  const pts_t p = clump(3000, 12);
  cvc::test::t7_subject<cvc::tet_complex_builder, tet_complex> s;
  s.name = "tet_complex clump 3000";
  s.make = [p] { return std::make_unique<cvc::tet_complex_builder>(p); };
  s.take = [](cvc::tet_complex_builder &b) { return b.take(); };
  s.equal = same_complex;
  s.blocking = [p] { return cvc::delaunay_tet_complex(p); };
  const cvc::test::t7_report r = cvc::test::run_t7(s);
  const std::vector<std::string> want{"checking", "ordering", "inserting", "compacting",
                                      "circumcentres"};
  EXPECT_EQ(r.phases, want);
  // A step of 4096 units inserts a few hundred points at most: well under a second anywhere.
  EXPECT_LT(r.slowest_step_ms, 1000.0);
}

TEST_F(TetComplex, CancelEndsTheBuildWithNoResult) {
  cvc::tet_complex_builder b(uniform(2000, 13));
  for (int i = 0; i < 40; ++i)
    b.step(64);
  EXPECT_FALSE(b.done());
  b.cancel();
  EXPECT_TRUE(b.step(64));
  EXPECT_TRUE(b.done());
  EXPECT_STREQ(b.phase(), "failed");
  EXPECT_NE(b.status().find("cancelled"), std::string::npos);
  EXPECT_THROW(b.take(), cvc::tet_complex_error);
}

} // namespace
