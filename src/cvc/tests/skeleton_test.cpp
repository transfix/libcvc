/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::skeletonize (cvc/geometry/skeleton.h), the VolumeRover Skeletonization port
// (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §2, P1): known shapes (T3), invariants (T4),
// bad input (T5), no singletons (T6), resumable (T7) and the parameter table (T8). Runs natively
// and under node (the ports lane).

#include "../geometry/skeleton/datastruct.h"
#include "resumable_harness.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cvc/geometry/skeleton.h>
#include <cvc/geometry/skeleton_params.h>
#include <cvc/geometry/tet_complex.h>
#include <fstream>
#include <gtest/gtest.h>
#include <map>
#include <numeric>
#include <random>
#include <set>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

namespace {

using pts_t = std::vector<std::array<double, 3>>;
constexpr double pi = 3.14159265358979323846;

// Samples of closed surfaces, deterministic (a seeded generator for the jitter).
pts_t torus(int nu, int nv, double R, double r, std::uint32_t seed) {
  std::mt19937 g(seed);
  std::uniform_real_distribution<double> j(-0.15, 0.15);
  pts_t p;
  for (int i = 0; i < nu; ++i)
    for (int k = 0; k < nv; ++k) {
      const double u = 2 * pi * (i + j(g)) / nu, v = 2 * pi * (k + j(g)) / nv;
      p.push_back({(R + r * std::cos(v)) * std::cos(u), (R + r * std::cos(v)) * std::sin(u),
                   r * std::sin(v)});
    }
  return p;
}

// A closed box of size a x b x c, sampled on its faces with spacing h.
pts_t box(double a, double b, double c, double h, std::uint32_t seed) {
  std::mt19937 g(seed);
  std::uniform_real_distribution<double> j(-0.1 * h, 0.1 * h);
  pts_t p;
  std::set<std::array<long, 3>> seen;
  const long na = std::lround(a / h), nb = std::lround(b / h), nc = std::lround(c / h);
  for (long i = 0; i <= na; ++i)
    for (long k = 0; k <= nb; ++k)
      for (long l = 0; l <= nc; ++l) {
        if (i != 0 && i != na && k != 0 && k != nb && l != 0 && l != nc)
          continue; // interior
        auto jit = [&](long q, long n) { return (q == 0 || q == n) ? 0.0 : j(g); };
        p.push_back({i * h + jit(i, na), k * h + jit(k, nb), l * h + jit(l, nc)});
      }
  return p;
}

// A capsule: a cylinder of radius r and length L along x, with hemispherical caps.
pts_t capsule(double L, double r, int nring, int nalong, std::uint32_t seed) {
  std::mt19937 g(seed);
  std::uniform_real_distribution<double> j(-0.15, 0.15);
  pts_t p;
  for (int i = 0; i <= nalong; ++i)
    for (int k = 0; k < nring; ++k) {
      const double x = L * (i + (i > 0 && i < nalong ? j(g) : 0.0)) / nalong;
      const double a = 2 * pi * (k + j(g)) / nring;
      p.push_back({x, r * std::cos(a), r * std::sin(a)});
    }
  const int ncap = nring / 4;
  for (int side = 0; side < 2; ++side)
    for (int i = 1; i <= ncap; ++i) {
      const double t = (pi / 2) * (i + 0.1 * j(g)) / (ncap + 0.5);
      const int nr = std::max(1, int(std::lround(nring * std::cos(t))));
      for (int k = 0; k < nr; ++k) {
        const double a = 2 * pi * (k + j(g)) / nr;
        const double x = side == 0 ? -r * std::sin(t) : L + r * std::sin(t);
        p.push_back({x, r * std::cos(t) * std::cos(a), r * std::cos(t) * std::sin(a)});
      }
    }
  return p;
}

std::string describe(const cvc::skeleton &s) {
  std::string d = std::to_string(s.polylines) + " polylines, " +
                  std::to_string(s.strands.const_lines().size()) + " strand lines, " +
                  std::to_string(s.clusters) + " clusters (" + std::to_string(s.sheet_clusters) +
                  " sheets, " + std::to_string(s.sheets.const_tris().size()) + " triangles)";
  for (const std::string &w : s.warnings)
    d += "; " + w;
  return d;
}

class Skeleton : public ::testing::Test {
protected:
  void SetUp() override {
    if (!cvc::tet_complex_available())
      GTEST_SKIP() << "libcvc built without Shewchuk's predicates (CVC_WITH_SHEWCHUK)";
  }
};

// ---- T1: legacy parity (tools/volrover-goldens) ---------------------------------------------

#ifndef CVC_REPO_DIR
#define CVC_REPO_DIR "."
#endif

struct golden {
  std::vector<std::array<double, 6>> polyline_segments, stars;
  std::vector<std::vector<std::array<double, 3>>> polygons;
  std::vector<std::array<double, 4>> helix; // maximum flag, x, y, z
};

golden read_golden(const std::string &path) {
  golden g;
  std::ifstream in(path);
  std::string kind;
  while (in >> kind) {
    if (kind == "L" || kind == "S") {
      std::array<double, 6> s{};
      for (double &x : s)
        in >> x;
      (kind == "L" ? g.polyline_segments : g.stars).push_back(s);
    } else if (kind == "P") {
      std::size_t n = 0;
      in >> n;
      std::vector<std::array<double, 3>> poly(n);
      for (auto &p : poly)
        in >> p[0] >> p[1] >> p[2];
      g.polygons.push_back(poly);
    } else if (kind == "H") {
      std::size_t n = 0;
      in >> n;
      for (std::size_t i = 0; i < n; ++i) {
        std::array<double, 4> h{};
        in >> h[0] >> h[1] >> h[2] >> h[3];
        g.helix.push_back(h);
      }
    }
  }
  return g;
}

pts_t read_points(const std::string &path) {
  pts_t p;
  std::ifstream in(path);
  std::array<double, 3> q{};
  while (in >> q[0] >> q[1] >> q[2])
    p.push_back(q);
  return p;
}

// How many of `want` have a match in `got` (each used once) within `tol`, comparing with
// `dist`: a greedy match over candidates sorted by their first coordinate.
template <class T, class D>
std::size_t matched(const std::vector<T> &want, const std::vector<T> &got, double tol, D dist,
                    double (*key)(const T &)) {
  std::vector<std::pair<double, std::size_t>> idx;
  for (std::size_t i = 0; i < got.size(); ++i)
    idx.push_back({key(got[i]), i});
  std::sort(idx.begin(), idx.end());
  std::vector<char> used(got.size(), 0);
  std::size_t n = 0;
  for (const T &w : want) {
    const double k = key(w);
    auto it = std::lower_bound(idx.begin(), idx.end(), std::make_pair(k - tol, std::size_t(0)));
    for (; it != idx.end() && it->first <= k + tol; ++it)
      if (!used[it->second] && dist(w, got[it->second]) <= tol) {
        used[it->second] = 1;
        ++n;
        break;
      }
  }
  return n;
}

// A segment as (lower end, upper end) in lexicographic order, so its direction does not matter.
std::array<double, 6> canonical(std::array<double, 6> s) {
  if (std::make_tuple(s[3], s[4], s[5]) < std::make_tuple(s[0], s[1], s[2])) {
    std::swap(s[0], s[3]);
    std::swap(s[1], s[4]);
    std::swap(s[2], s[5]);
  }
  return s;
}
double seg_key(const std::array<double, 6> &s) { return s[0]; }
double seg_dist(const std::array<double, 6> &a, const std::array<double, 6> &b) {
  double d = 0;
  for (int k = 0; k < 6; ++k)
    d = std::max(d, std::fabs(a[std::size_t(k)] - b[std::size_t(k)]));
  return d;
}
double pt_key(const std::array<double, 3> &p) { return p[0]; }
double pt_dist(const std::array<double, 3> &a, const std::array<double, 3> &b) {
  return std::max({std::fabs(a[0] - b[0]), std::fabs(a[1] - b[1]), std::fabs(a[2] - b[2])});
}

// The segments of strands lines whose strand_of_line passes `which`.
template <class F> std::vector<std::array<double, 6>> segments(const cvc::skeleton &s, F which) {
  std::vector<std::array<double, 6>> out;
  const auto &pts = s.strands.const_points();
  const auto &lines = s.strands.const_lines();
  for (std::size_t i = 0; i < lines.size(); ++i) {
    if (!which(s.strand_of_line[i]))
      continue;
    const auto &a = pts[std::size_t(lines[i][0])], &b = pts[std::size_t(lines[i][1])];
    out.push_back(canonical({a[0], a[1], a[2], b[0], b[1], b[2]}));
  }
  return out;
}

double polygon_area(const std::vector<std::array<double, 3>> &poly) { // fanned, as the port does
  double a = 0;
  for (std::size_t k = 1; k + 1 < poly.size(); ++k) {
    const auto &p = poly[0], &q = poly[k], &r = poly[k + 1];
    const double u[3] = {q[0] - p[0], q[1] - p[1], q[2] - p[2]};
    const double v[3] = {r[0] - p[0], r[1] - p[1], r[2] - p[2]};
    const double c[3] = {u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
                         u[0] * v[1] - u[1] * v[0]};
    a += 0.5 * std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
  }
  return a;
}

struct parity {
  std::size_t want = 0, got = 0, matched = 0;
  bool exact() const { return want == got && matched == want; }
};

// exact: the port's output is the legacy output (to 1e-9). superset: every legacy element is in
// the port's output, and the port has at most `extra` more (U2's order effect, and the closed
// loops the legacy code dropped; see MatchesTheLegacyLibrary).
void check_parity(const std::string &name, bool exact, std::size_t extra = 0) {
  SCOPED_TRACE(name);
  const std::string inputs = std::string(CVC_REPO_DIR) + "/tools/volrover-goldens/inputs/skeleton/";
  const std::string goldens =
      std::string(CVC_REPO_DIR) + "/src/cvc/tests/data/volrover-goldens/skeleton/";
  const pts_t pts = read_points(inputs + name + ".pts");
  ASSERT_FALSE(pts.empty()) << "no input at " << inputs + name + ".pts";
  const golden g = read_golden(goldens + name + ".skel.txt");
  cvc::skeleton_params params;
  params.helix_window = true; // the legacy output always has the helix strips
  const cvc::skeleton s = cvc::skeletonize(pts, params);
  // The closed polylines (loops): the legacy code dropped them, so they are not compared here.
  std::vector<char> closed(std::size_t(s.polylines), 0);
  {
    std::vector<std::size_t> first(std::size_t(s.polylines), SIZE_MAX), last(closed.size(), 0);
    const auto &lines = s.strands.const_lines();
    for (std::size_t i = 0; i < lines.size(); ++i)
      if (s.strand_of_line[i] >= 0) {
        const auto k = std::size_t(s.strand_of_line[i]);
        if (first[k] == SIZE_MAX)
          first[k] = i;
        last[k] = i;
      }
    for (std::size_t k = 0; k < closed.size(); ++k)
      closed[k] = first[k] != SIZE_MAX && lines[first[k]][0] == lines[last[k]][1];
  }

  // Coordinates are compared to 1e-9 of the input's extent: the Voronoi vertices round
  // differently when a cell lists its vertices in another order.
  double extent = 0;
  for (const auto &p : pts)
    for (double x : p)
      extent = std::max(extent, std::fabs(x));
  const double tol = 1e-9 * std::max(extent, 1.0);

  auto compare_segments = [&](const char *what, std::vector<std::array<double, 6>> want,
                              const std::vector<std::array<double, 6>> &got) {
    for (auto &w : want)
      w = canonical(w);
    // The legacy output is a std::set: equal segments appear once.
    std::sort(want.begin(), want.end());
    want.erase(std::unique(want.begin(), want.end()), want.end());
    std::vector<std::array<double, 6>> g2 = got;
    std::sort(g2.begin(), g2.end());
    g2.erase(std::unique(g2.begin(), g2.end()), g2.end());
    parity p{want.size(), g2.size(), matched(want, g2, tol, seg_dist, seg_key)};
    std::cout << name << " " << what << ": legacy " << p.want << ", port " << p.got << ", "
              << p.matched << " matched\n";
    if (exact) {
      EXPECT_TRUE(p.exact()) << what;
    } else {
      EXPECT_EQ(p.matched, p.want) << what << ": every legacy segment is in the port's output";
      EXPECT_LE(p.got, p.want + extra) << what;
    }
    return p;
  };
  compare_segments("polyline segments", g.polyline_segments,
                   segments(s, [&](int k) { return k >= 0 && !closed[std::size_t(k)]; }));
  compare_segments("star segments", g.stars, segments(s, [](int k) { return k < 0; }));

  // Sheets: the polygons' points and total area against the port's triangles.
  std::vector<std::array<double, 3>> want_pts, got_pts;
  double want_area = 0;
  for (const auto &poly : g.polygons) {
    want_area += polygon_area(poly);
    want_pts.insert(want_pts.end(), poly.begin(), poly.end());
  }
  std::sort(want_pts.begin(), want_pts.end());
  want_pts.erase(std::unique(want_pts.begin(), want_pts.end()), want_pts.end());
  for (const auto &p : s.sheets.const_points())
    got_pts.push_back({p[0], p[1], p[2]});
  std::sort(got_pts.begin(), got_pts.end());
  got_pts.erase(std::unique(got_pts.begin(), got_pts.end()), got_pts.end());
  double got_area = 0;
  for (const auto &t : s.sheets.const_tris())
    got_area += polygon_area({{s.sheets.const_points()[std::size_t(t[0])][0],
                               s.sheets.const_points()[std::size_t(t[0])][1],
                               s.sheets.const_points()[std::size_t(t[0])][2]},
                              {s.sheets.const_points()[std::size_t(t[1])][0],
                               s.sheets.const_points()[std::size_t(t[1])][1],
                               s.sheets.const_points()[std::size_t(t[1])][2]},
                              {s.sheets.const_points()[std::size_t(t[2])][0],
                               s.sheets.const_points()[std::size_t(t[2])][1],
                               s.sheets.const_points()[std::size_t(t[2])][2]}});
  const std::size_t sheet_matched = matched(want_pts, got_pts, tol, pt_dist, pt_key);
  std::cout << name << " sheets: legacy " << g.polygons.size() << " polygons, " << want_pts.size()
            << " points, area " << want_area << "; port " << s.sheets.const_tris().size()
            << " triangles, " << got_pts.size() << " points, area " << got_area << "; "
            << sheet_matched << " points matched\n";
  if (exact) {
    EXPECT_EQ(want_pts.size(), got_pts.size());
    EXPECT_EQ(sheet_matched, want_pts.size());
    EXPECT_NEAR(got_area, want_area, 1e-9 * std::max(1.0, want_area));
  }

  // The helix strips' vertices.
  std::vector<std::array<double, 3>> want_h, got_h;
  for (const auto &h : g.helix)
    want_h.push_back({h[1], h[2], h[3]});
  for (const auto &p : s.helix.const_points())
    got_h.push_back({p[0], p[1], p[2]});
  std::sort(want_h.begin(), want_h.end());
  want_h.erase(std::unique(want_h.begin(), want_h.end()), want_h.end());
  const std::size_t helix_matched = matched(want_h, got_h, tol, pt_dist, pt_key);
  std::cout << name << " helix: legacy " << want_h.size() << " vertices, port " << got_h.size()
            << ", " << helix_matched << " matched\n";
  if (exact) {
    EXPECT_EQ(want_h.size(), got_h.size());
    EXPECT_EQ(helix_matched, want_h.size());
  }
  std::cout << name << ": " << describe(s) << "\n";
}

TEST_F(Skeleton, MatchesTheLegacyLibrary) {
  // The capsule, the torus and the cospherical box: the port's skeleton is the legacy library's,
  // segment for segment, sheet point for sheet point (the triangulation is CGAL's, cell for
  // cell). Closed loops are left out of the comparison: the legacy code dropped them (it walks
  // only from joints and leaves), so its torus is empty, where the port has the core circle
  // (KnownShapes checks it). Where U2's chains merge, which pieces are kept depends on the order
  // the saddles are traced in: the legacy code took its cells' memory order (CGAL's allocation
  // history), the port a geometric order. On the dumbbell that leaves the port two more segments
  // than the legacy output, which the port contains whole.
  check_parity("capsule", true);
  check_parity("box_grid", true);
  check_parity("torus", true);
  check_parity("dumbbell", false, 4);
}

// ---- T2: the kernel's exact pieces --------------------------------------------------------

TEST_F(Skeleton, KernelPredicates) {
  using cvc::cocone::Point;
  using cvc::cocone::Segment;
  using cvc::cocone::Triangle_3;
  namespace cg = cvc::cocone::cg;
  const Triangle_3 t(Point(0, 0, 0), Point(4, 0, 0), Point(0, 4, 0));
  EXPECT_TRUE(cg::do_intersect(t, Segment(Point(1, 1, -1), Point(1, 1, 1))));    // through
  EXPECT_FALSE(cg::do_intersect(t, Segment(Point(3, 3, -1), Point(3, 3, 1))));   // beside
  EXPECT_FALSE(cg::do_intersect(t, Segment(Point(1, 1, 0.5), Point(1, 1, 1))));  // above
  EXPECT_TRUE(cg::do_intersect(t, Segment(Point(1, 1, 0), Point(1, 1, 1))));     // an end on it
  EXPECT_TRUE(cg::do_intersect(t, Segment(Point(2, 0, -1), Point(2, 0, 1))));    // via an edge
  EXPECT_TRUE(cg::do_intersect(t, Segment(Point(-1, 1, 0), Point(5, 1, 0))));    // coplanar
  EXPECT_FALSE(cg::do_intersect(t, Segment(Point(-1, -1, 0), Point(-1, 5, 0)))); // coplanar, off
  // Degenerate triangles are the point or segment they span.
  const Triangle_3 dot(Point(1, 1, 1), Point(1, 1, 1), Point(1, 1, 1));
  EXPECT_TRUE(cg::do_intersect(dot, Segment(Point(0, 0, 0), Point(2, 2, 2))));
  EXPECT_FALSE(cg::do_intersect(dot, Segment(Point(0, 0, 0), Point(2, 2, 2.5))));
  const Triangle_3 rod(Point(0, 0, 0), Point(2, 0, 0), Point(1, 0, 0));
  EXPECT_TRUE(cg::do_intersect(rod, Segment(Point(1.5, -1, 0), Point(1.5, 1, 0))));
  EXPECT_FALSE(cg::do_intersect(rod, Segment(Point(2.5, -1, 0), Point(2.5, 1, 0))));
  EXPECT_FALSE(cg::do_intersect(rod, Segment(Point(1, -1, 1), Point(1, 1, 1))));
  // Circumcentres, by CGAL's formulas.
  const Point c4 = cg::circumcenter(Point(0, 0, 0), Point(2, 0, 0), Point(0, 2, 0), Point(0, 0, 2));
  EXPECT_DOUBLE_EQ(c4.x(), 1.0);
  EXPECT_DOUBLE_EQ(c4.y(), 1.0);
  EXPECT_DOUBLE_EQ(c4.z(), 1.0);
  const Point c3 = cg::circumcenter(Point(0, 0, 0), Point(2, 0, 0), Point(0, 2, 0));
  EXPECT_DOUBLE_EQ(c3.x(), 1.0);
  EXPECT_DOUBLE_EQ(c3.y(), 1.0);
  EXPECT_DOUBLE_EQ(c3.z(), 0.0);
  EXPECT_EQ(cg::orientation(Point(0, 0, 0), Point(1, 0, 0), Point(0, 1, 0), Point(0, 0, 1)),
            cg::POSITIVE);
  EXPECT_TRUE(cg::collinear(Point(0, 0, 0), Point(1, 1, 1), Point(3, 3, 3)));
  // A ray against a segment in the plane.
  cg::Point_2<int> p;
  EXPECT_TRUE(cg::assign(
      p, cg::intersection(cg::Ray_2<int>({0, -1}, {0, 1}), cg::Segment_2<int>({-1, 0}, {1, 0}))));
  EXPECT_DOUBLE_EQ(p.x(), 0.0);
  EXPECT_FALSE(cg::assign(
      p, cg::intersection(cg::Ray_2<int>({0, 1}, {0, 2}), cg::Segment_2<int>({-1, 0}, {1, 0}))));
}

// ---- T3: known shapes ---------------------------------------------------------------------

// The strands' polylines as point sequences (by strand_of_line).
std::vector<std::vector<std::array<double, 3>>> polylines_of(const cvc::skeleton &s) {
  std::vector<std::vector<std::array<double, 3>>> out(std::size_t(s.polylines));
  const auto &pts = s.strands.const_points();
  const auto &lines = s.strands.const_lines();
  for (std::size_t i = 0; i < lines.size(); ++i) {
    const int k = s.strand_of_line[i];
    if (k < 0)
      continue;
    auto &pl = out[std::size_t(k)];
    const auto &a = pts[std::size_t(lines[i][0])], &b = pts[std::size_t(lines[i][1])];
    if (pl.empty())
      pl.push_back({a[0], a[1], a[2]});
    pl.push_back({b[0], b[1], b[2]});
  }
  return out;
}

std::string input_path(const char *name) {
  return std::string(CVC_REPO_DIR) + "/tools/volrover-goldens/inputs/skeleton/" + name + ".pts";
}

TEST_F(Skeleton, KnownShapes) {
  {
    // A torus: its core circle, one closed strand near radius 4.
    const cvc::skeleton s = cvc::skeletonize(read_points(input_path("torus")));
    std::cout << "torus: " << describe(s) << "\n";
    const auto pls = polylines_of(s);
    ASSERT_EQ(pls.size(), 1u) << describe(s);
    EXPECT_EQ(pls[0].front(), pls[0].back()) << "closed";
    EXPECT_GT(pls[0].size(), 20u);
    for (const auto &p : pls[0])
      EXPECT_NEAR(std::hypot(p[0], p[1]), 4.0, 0.5);
    EXPECT_EQ(s.clusters, 0);
  }
  {
    // A capsule along x from 0 to 8: one strand along its axis.
    const cvc::skeleton s = cvc::skeletonize(capsule(8.0, 1.0, 20, 40, 3));
    std::cout << "capsule: " << describe(s) << "\n";
    const auto pls = polylines_of(s);
    ASSERT_EQ(pls.size(), 1u) << describe(s);
    for (const auto &p : pls[0])
      EXPECT_LT(std::hypot(p[1], p[2]), 0.2) << "on the axis";
    EXPECT_EQ(s.sheets.const_tris().size(), 0u);
  }
  {
    // A thin slab: a sheet near its middle plane.
    const cvc::skeleton s = cvc::skeletonize(box(6.0, 6.0, 0.6, 0.15, 2));
    std::cout << "slab: " << describe(s) << "\n";
    ASSERT_GE(s.sheet_clusters, 1);
    double zlo = 1e300, zhi = -1e300;
    for (const auto &p : s.sheets.const_points()) {
      zlo = std::min(zlo, p[2]);
      zhi = std::max(zhi, p[2]);
    }
    EXPECT_GT(s.sheets.const_tris().size(), 100u);
    EXPECT_NEAR(0.5 * (zlo + zhi), 0.3, 0.1) << "the sheet lies near the middle plane";
  }
}

// ---- T4: invariants -----------------------------------------------------------------------

// The skeleton's segments and sheet points, sorted, after `f` maps them to the input's frame.
struct summary {
  std::vector<std::array<double, 6>> strands;
  std::vector<std::array<double, 3>> sheet_points;
};
template <class F> summary summarize(const cvc::skeleton &s, F f) {
  summary out;
  const auto &pts = s.strands.const_points();
  for (const auto &l : s.strands.const_lines()) {
    const auto a = f(pts[std::size_t(l[0])]), b = f(pts[std::size_t(l[1])]);
    out.strands.push_back(canonical({a[0], a[1], a[2], b[0], b[1], b[2]}));
  }
  for (const auto &p : s.sheets.const_points())
    out.sheet_points.push_back(f(p));
  std::sort(out.strands.begin(), out.strands.end());
  std::sort(out.sheet_points.begin(), out.sheet_points.end());
  return out;
}

TEST_F(Skeleton, UnchangedByPointOrderAndRigidMotion) {
  auto id = [](const auto &p) { return std::array<double, 3>{p[0], p[1], p[2]}; };
  for (const char *name : {"box_grid", "dumbbell", "capsule"}) {
    SCOPED_TRACE(name);
    const pts_t input = read_points(input_path(name));
    const summary ref = summarize(cvc::skeletonize(input), id);
    // Shuffled.
    pts_t q = input;
    std::mt19937 g(5);
    std::shuffle(q.begin(), q.end(), g);
    const summary shuffled = summarize(cvc::skeletonize(q), id);
    EXPECT_EQ(shuffled.strands.size(), ref.strands.size());
    EXPECT_EQ(matched(ref.strands, shuffled.strands, 1e-9, seg_dist, seg_key), ref.strands.size());
    EXPECT_EQ(shuffled.sheet_points.size(), ref.sheet_points.size());
    EXPECT_EQ(matched(ref.sheet_points, shuffled.sheet_points, 1e-9, pt_dist, pt_key),
              ref.sheet_points.size());
    // Moved by an exact offset and turned a quarter turn about z: (x, y, z) -> (-y, x, z) + t,
    // exact in doubles for these inputs, so the skeleton maps back to within roundoff. Only in
    // general position: on cospherical input the perturbation breaks ties by the points'
    // lexicographic order, which a rotation changes (as CGAL's does), so the triangulation and
    // the skeleton may differ there.
    if (std::string(name) == "box_grid")
      continue;
    pts_t m;
    for (const auto &p : input)
      m.push_back({-p[1] + 16.0, p[0] - 8.0, p[2] + 4.0});
    auto back = [](const auto &p) {
      return std::array<double, 3>{p[1] + 8.0, -(p[0] - 16.0), p[2] - 4.0};
    };
    const summary moved = summarize(cvc::skeletonize(m), back);
    EXPECT_EQ(moved.strands.size(), ref.strands.size());
    EXPECT_EQ(matched(ref.strands, moved.strands, 1e-6, seg_dist, seg_key), ref.strands.size());
    EXPECT_EQ(moved.sheet_points.size(), ref.sheet_points.size());
    EXPECT_EQ(matched(ref.sheet_points, moved.sheet_points, 1e-6, pt_dist, pt_key),
              ref.sheet_points.size());
  }
}

// ---- T5: bad input ------------------------------------------------------------------------

std::string error_of(const pts_t &p, const cvc::skeleton_params &params = {}) {
  try {
    cvc::skeletonize(p, params);
  } catch (const cvc::skeleton_error &e) {
    return e.what();
  }
  return "";
}

TEST_F(Skeleton, BadInputThrowsWithAReason) {
  EXPECT_NE(error_of({{0, 0, 0}, {1, 0, 0}}).find("2 points"), std::string::npos);
  pts_t plane, line;
  for (int i = 0; i < 30; ++i) {
    plane.push_back({double(i % 6), double(i / 6), 1.0});
    line.push_back({double(i), double(i), double(i)});
  }
  EXPECT_NE(error_of(plane).find("one plane"), std::string::npos);
  EXPECT_NE(error_of(line).find("one line"), std::string::npos);
  pts_t nan = capsule(4.0, 1.0, 10, 10, 1);
  nan[3][0] = std::nan("");
  EXPECT_NE(error_of(nan).find("not finite"), std::string::npos);
  cvc::skeleton_params bad;
  bad.threshold = 2.0;
  bad.sheet_count = -1;
  const std::string e = error_of(capsule(4.0, 1.0, 10, 10, 1), bad);
  EXPECT_NE(e.find("threshold"), std::string::npos) << e;
  EXPECT_NE(e.find("sheet_count"), std::string::npos) << e;
}

TEST_F(Skeleton, DegenerateInputRunsToTheEnd) {
  // Points on a sphere (nearly cospherical), duplicates, and four points: a result, not a crash.
  pts_t sphere;
  std::mt19937 g(9);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  for (int i = 0; i < 400; ++i) {
    const double th = std::acos(u(g)), ph = pi * (u(g) + 1.0);
    sphere.push_back({std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph), std::cos(th)});
  }
  const cvc::skeleton s = cvc::skeletonize(sphere);
  std::cout << "sphere: " << describe(s) << "\n";
  pts_t dup = capsule(6.0, 1.0, 16, 24, 4);
  const std::size_t n = dup.size();
  for (std::size_t i = 0; i < n; i += 3)
    dup.push_back(dup[i]);
  const cvc::skeleton d = cvc::skeletonize(dup);
  ASSERT_FALSE(d.warnings.empty());
  EXPECT_NE(d.warnings.front().find("duplicate"), std::string::npos);
  EXPECT_NO_THROW(cvc::skeletonize(pts_t{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}}));
}

// ---- T6: no singletons --------------------------------------------------------------------

bool same(const cvc::skeleton &a, const cvc::skeleton &b) {
  return a.strands.const_points() == b.strands.const_points() &&
         a.strands.const_lines() == b.strands.const_lines() &&
         a.sheets.const_points() == b.sheets.const_points() &&
         a.sheets.const_tris() == b.sheets.const_tris() && a.strand_of_line == b.strand_of_line &&
         a.sheet_of_tri == b.sheet_of_tri && a.helix.const_points() == b.helix.const_points() &&
         a.helix.const_lines() == b.helix.const_lines() && a.warnings == b.warnings;
}

TEST_F(Skeleton, CallsAreIndependent) {
  const pts_t a = box(6.0, 6.0, 0.6, 0.2, 7), b = capsule(8.0, 1.0, 20, 40, 8);
  cvc::skeleton_params pa, pb;
  pb.helix_window = true;
  pb.sheet_count = 1;
  const cvc::skeleton ra = cvc::skeletonize(a, pa), rb = cvc::skeletonize(b, pb);
  // B after A, and A after B, equal each alone (the legacy bounding box was filled by the first
  // call and reused by every later one).
  EXPECT_TRUE(same(cvc::skeletonize(b, pb), rb));
  EXPECT_TRUE(same(cvc::skeletonize(a, pa), ra));
  // Two at once on two threads, with different parameters.
  cvc::skeleton ta, tb;
  std::thread t1([&] { ta = cvc::skeletonize(a, pa); });
  std::thread t2([&] { tb = cvc::skeletonize(b, pb); });
  t1.join();
  t2.join();
  EXPECT_TRUE(same(ta, ra));
  EXPECT_TRUE(same(tb, rb));
}

// ---- T7: resumable ------------------------------------------------------------------------

TEST_F(Skeleton, StepperIsResumable) {
  const pts_t p = capsule(6.0, 1.0, 16, 24, 6);
  cvc::test::t7_subject<cvc::skeleton_stepper, cvc::skeleton> s;
  s.name = "skeleton capsule";
  s.make = [p] { return std::make_unique<cvc::skeleton_stepper>(p); };
  s.take = [](cvc::skeleton_stepper &st) { return st.take(); };
  s.equal = same;
  s.blocking = [p] { return cvc::skeletonize(p); };
  const cvc::test::t7_report r = cvc::test::run_t7(s);
  for (const char *phase :
       {"checking", "ordering", "inserting", "compacting", "circumcentres", "building", "voronoi",
        "tight cocone", "medial axis", "sheets", "strands", "clean-up", "output"})
    EXPECT_NE(std::find(r.phases.begin(), r.phases.end(), phase), r.phases.end()) << phase;
  EXPECT_LT(r.slowest_step_ms, 1000.0);
}

TEST_F(Skeleton, RobustAndHelixOptionsRun) {
  cvc::skeleton_params p;
  p.robust = true;
  p.helix_window = true;
  const cvc::skeleton s = cvc::skeletonize(capsule(8.0, 1.0, 20, 40, 3), p);
  std::cout << "robust capsule: " << describe(s) << "\n";
  EXPECT_EQ(s.polylines, 1);
  // With `robust`, the robust cocone runs between two triangulations.
  cvc::skeleton_stepper st(capsule(4.0, 1.0, 12, 12, 1), p);
  std::set<std::string> phases;
  while (!st.step(32))
    phases.insert(st.phase());
  EXPECT_TRUE(phases.count("robust cocone"));
}

// ---- T8: the parameter table --------------------------------------------------------------

TEST_F(Skeleton, ParameterTable) {
  cvc::skeleton_params p;
  p.robust = true;
  p.cocone_phi = 0.3;
  p.sheet_count = 5;
  p.helix_window = true;
  EXPECT_TRUE(cvc::params_validate(p).empty());
  std::size_t fields = 0;
  cvc::for_each_param<cvc::skeleton_params>([&](const auto &) { ++fields; });
  EXPECT_EQ(fields, 14u);
  cvc::skeleton_params q = p;
  q.medial_theta = -1.0;
  EXPECT_FALSE(cvc::params_validate(q).empty());
}

} // namespace
