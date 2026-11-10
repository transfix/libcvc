/*
  Copyright 2026 The University of Texas at Austin

        Authors: Joe Rivera <transfix@gmail.com>
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

// Smoke test for the permissive geometry libraries that replace CGAL's GPL packages
// (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.2, phase P0c):
//   - reentrant Qhull (libqhull_r): 3D Delaunay, run as `d Qbb Qt Qz`;
//   - Clipper2: polygon Booleans, offsets and Minkowski sums;
//   - CDT: 2D constrained Delaunay;
//   - Shewchuk's predicates: exact orientation and in-circle / in-sphere signs.
// All four are cvcpkg packages (qhull, clipper2, cdt, shewchuk-predicates); each section runs
// when libcvc was built with its package (CVC_WITH_*) and prints SKIP otherwise.
// A plain main(), not gtest, so the same program runs natively under ctest and as wasm-mt under
// node (ci.yml's permissive-wasm job), where there is no googletest. Prints one PASS or FAIL
// line per check and exits non-zero on any failure.
//
// Each library is checked on known answers, and the libraries check each other: the predicates
// decide whether Qhull's tetrahedra and CDT's triangles have empty circumspheres / circumcircles.
// Qhull and the predicates also run on two threads at once, which must change nothing (no
// process-wide state, §1.3).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#ifdef CVC_WITH_SHEWCHUK
#include <shewchuk/predicates.hpp>
#endif

#ifdef CVC_WITH_CDT
#include <CDT/CDT.h>
#endif

#ifdef CVC_WITH_CLIPPER2
#include <clipper2/clipper.h>
#endif

#ifdef CVC_WITH_QHULL
extern "C" {
#include "libqhull_r/qhull_ra.h"
}
#endif

namespace {

int g_checks = 0;
int g_failures = 0;

void check(bool ok, const std::string &name, const std::string &detail) {
  ++g_checks;
  if (!ok) {
    ++g_failures;
  }
  std::printf("%s %s: %s\n", ok ? "PASS" : "FAIL", name.c_str(), detail.c_str());
  std::fflush(stdout);
}

template <typename... Args> std::string fmt(const char *f, Args... args) {
  char buf[512];
  std::snprintf(buf, sizeof buf, f, args...);
  return buf;
}

int sgn(double v) { return (v > 0.0) - (v < 0.0); }
int sgn(std::int64_t v) { return (v > 0) - (v < 0); }

// xorshift32, so every platform makes the same clouds.
std::vector<double> cloud(unsigned seed, int n) {
  std::vector<double> p;
  p.reserve(3 * n);
  unsigned x = seed;
  for (int i = 0; i < 3 * n; ++i) {
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    p.push_back((x % 100000) / 100000.0);
  }
  return p;
}

using point3 = std::array<double, 3>;

// ---------------------------------------------------------------------------------------------
// Shewchuk's predicates

#ifdef CVC_WITH_SHEWCHUK
constexpr double two_m53 = 1.0 / 9007199254740992.0; // 2^-53: the spacing of doubles in [0.5, 1)
constexpr double two_m27 = 1.0 / 134217728.0;        // 2^-27

// The exact sign of x^2 + y^2 - 1 for x = i 2^-27, y = -(1 - k 2^-53): it is
// 2^-54 (i^2 - 4k) + k^2 2^-106.
int unit_circle_side(int i, int k) {
  const std::int64_t lead = std::int64_t(i) * i - 4 * std::int64_t(k);
  if (lead != 0) {
    return sgn(lead);
  }
  return k != 0 ? 1 : 0;
}

// The exact 2D orientation, in-circle, 3D orientation and in-sphere determinants of small
// integer points, in int64 (coordinates below 2^8 keep every term below 2^52).
std::int64_t exact_orient2d(const std::array<std::int64_t, 2> &a,
                            const std::array<std::int64_t, 2> &b,
                            const std::array<std::int64_t, 2> &c) {
  return (a[0] - c[0]) * (b[1] - c[1]) - (a[1] - c[1]) * (b[0] - c[0]);
}

std::int64_t det3(std::int64_t a, std::int64_t b, std::int64_t c, std::int64_t d, std::int64_t e,
                  std::int64_t f, std::int64_t g, std::int64_t h, std::int64_t i) {
  return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
}

std::int64_t exact_incircle(const std::array<std::int64_t, 2> &a,
                            const std::array<std::int64_t, 2> &b,
                            const std::array<std::int64_t, 2> &c,
                            const std::array<std::int64_t, 2> &d) {
  const std::int64_t ax = a[0] - d[0], ay = a[1] - d[1], bx = b[0] - d[0], by = b[1] - d[1];
  const std::int64_t cx = c[0] - d[0], cy = c[1] - d[1];
  return det3(ax, ay, ax * ax + ay * ay, bx, by, bx * bx + by * by, cx, cy, cx * cx + cy * cy);
}

// Shewchuk's orient3d: det of the rows a - d, b - d, c - d.
std::int64_t exact_orient3d(const std::array<std::int64_t, 3> &a,
                            const std::array<std::int64_t, 3> &b,
                            const std::array<std::int64_t, 3> &c,
                            const std::array<std::int64_t, 3> &d) {
  return det3(a[0] - d[0], a[1] - d[1], a[2] - d[2], b[0] - d[0], b[1] - d[1], b[2] - d[2],
              c[0] - d[0], c[1] - d[1], c[2] - d[2]);
}

// Shewchuk's insphere: the 4x4 determinant of the rows (p - e, |p - e|^2) for p = a, b, c, d.
std::int64_t exact_insphere(const std::array<std::int64_t, 3> &a,
                            const std::array<std::int64_t, 3> &b,
                            const std::array<std::int64_t, 3> &c,
                            const std::array<std::int64_t, 3> &d,
                            const std::array<std::int64_t, 3> &e) {
  std::int64_t m[4][4];
  const std::array<std::int64_t, 3> *p[4] = {&a, &b, &c, &d};
  for (int r = 0; r < 4; ++r) {
    const std::int64_t x = (*p[r])[0] - e[0], y = (*p[r])[1] - e[1], z = (*p[r])[2] - e[2];
    m[r][0] = x;
    m[r][1] = y;
    m[r][2] = z;
    m[r][3] = x * x + y * y + z * z;
  }
  // Laplace expansion along the last column.
  std::int64_t det = 0;
  for (int r = 0; r < 4; ++r) {
    std::int64_t minor[9];
    int n = 0;
    for (int rr = 0; rr < 4; ++rr) {
      if (rr == r) {
        continue;
      }
      minor[n++] = m[rr][0];
      minor[n++] = m[rr][1];
      minor[n++] = m[rr][2];
    }
    const std::int64_t cof = det3(minor[0], minor[1], minor[2], minor[3], minor[4], minor[5],
                                  minor[6], minor[7], minor[8]);
    det += ((r + 3) % 2 == 0 ? 1 : -1) * m[r][3] * cof;
  }
  return det;
}

// Near-degenerate grids whose exact signs are known in closed form. Returns the number of
// cases whose sign is wrong, and counts the cases.
struct grid_result {
  long cases = 0;
  long wrong = 0;
  long zeros = 0;
};

grid_result predicate_grids() {
  grid_result r;
  auto tally = [&r](int got, int want) {
    ++r.cases;
    r.wrong += got != want;
    r.zeros += want == 0;
  };
  // orient2d: a = (0.5 + i u, 0.5 + j u) against the line through (12, 12) and (24, 24).
  // orient2d(a, b, c) = 12 (a_y - a_x), so its sign is sign(j - i).
  const shewchuk::point2 b2 = {12, 12}, c2 = {24, 24};
  for (int i = 0; i < 256; ++i) {
    for (int j = 0; j < 256; ++j) {
      const shewchuk::point2 a = {0.5 + i * two_m53, 0.5 + j * two_m53};
      tally(sgn(shewchuk::orient2d(a, b2, c2)), (j > i) - (j < i));
    }
  }
  // orient3d: b, c, d on the plane z = x; orient3d(a, b, c, d) = 409 (a_z - a_x).
  const shewchuk::point3 b3 = {12, 0, 12}, c3 = {24, 7, 24}, d3 = {5, 30, 5};
  for (int i = 0; i < 128; ++i) {
    for (int j = 0; j < 128; ++j) {
      const shewchuk::point3 a = {0.5 + i * two_m53, 0.75, 0.5 + j * two_m53};
      tally(sgn(shewchuk::orient3d(a, b3, c3, d3)), (j > i) - (j < i));
    }
  }
  // incircle: the unit circle through (1, 0), (0, 1), (-1, 0), counterclockwise, against
  // d = (i 2^-27, -(1 - k 2^-53)), just inside, on or just outside it near (0, -1).
  const shewchuk::point2 ca = {1, 0}, cb = {0, 1}, cc = {-1, 0};
  for (int i = -24; i <= 24; ++i) {
    for (int k = 0; k < 256; ++k) {
      const shewchuk::point2 d = {i * two_m27, -(1.0 - k * two_m53)};
      tally(sgn(shewchuk::incircle(ca, cb, cc, d)), -unit_circle_side(i, k));
    }
  }
  // insphere: the unit sphere through (1, 0, 0), (0, 1, 0), (-1, 0, 0), (0, 0, 1), against
  // e = (i 2^-27, 0, -(1 - k 2^-53)). orient3d(a, b, c, d) < 0 here (d is above the
  // counterclockwise a, b, c), so insphere's sign is the opposite of "inside".
  const shewchuk::point3 sa = {1, 0, 0}, sb = {0, 1, 0}, sc = {-1, 0, 0}, sd = {0, 0, 1};
  const int orientation = sgn(shewchuk::orient3d(sa, sb, sc, sd));
  tally(orientation, -1);
  for (int i = -24; i <= 24; ++i) {
    for (int k = 0; k < 256; ++k) {
      const shewchuk::point3 e = {i * two_m27, 0.0, -(1.0 - k * two_m53)};
      tally(sgn(shewchuk::insphere(sa, sb, sc, sd, e)), -unit_circle_side(i, k) * orientation);
    }
  }
  return r;
}

// Random small-integer points, many of them exactly collinear, cocircular, coplanar or
// cospherical, against the exact int64 determinants.
grid_result predicate_integers(unsigned seed) {
  grid_result r;
  unsigned x = seed;
  auto next = [&x](int range) {
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return std::int64_t(x % unsigned(range)) - range / 2;
  };
  auto tally = [&r](int got, int want) {
    ++r.cases;
    r.wrong += got != want;
    r.zeros += want == 0;
  };
  for (int n = 0; n < 20000; ++n) {
    const int range = (n % 2) ? 5 : 200; // small ranges are mostly degenerate
    std::array<std::int64_t, 3> q[5];
    for (auto &p : q) {
      p = {next(range), next(range), next(range)};
    }
    auto d2 = [&](int k) { return shewchuk::point2{double(q[k][0]), double(q[k][1])}; };
    auto d3 = [&](int k) {
      return shewchuk::point3{double(q[k][0]), double(q[k][1]), double(q[k][2])};
    };
    auto i2 = [&](int k) { return std::array<std::int64_t, 2>{q[k][0], q[k][1]}; };
    tally(sgn(shewchuk::orient2d(d2(0), d2(1), d2(2))), sgn(exact_orient2d(i2(0), i2(1), i2(2))));
    tally(sgn(shewchuk::incircle(d2(0), d2(1), d2(2), d2(3))),
          sgn(exact_incircle(i2(0), i2(1), i2(2), i2(3))));
    tally(sgn(shewchuk::orient3d(d3(0), d3(1), d3(2), d3(3))),
          sgn(exact_orient3d(q[0], q[1], q[2], q[3])));
    tally(sgn(shewchuk::insphere(d3(0), d3(1), d3(2), d3(3), d3(4))),
          sgn(exact_insphere(q[0], q[1], q[2], q[3], q[4])));
  }
  return r;
}

void test_predicates() {
  // Exactly degenerate inputs give exactly zero.
  check(shewchuk::orient2d(shewchuk::point2{0, 0}, {1, 1}, {2, 2}) == 0.0, "predicates.collinear",
        "orient2d of three collinear points is 0");
  check(shewchuk::orient2d(shewchuk::point2{0, 0}, {1, 0}, {0, 1}) > 0.0 &&
            shewchuk::orient2d(shewchuk::point2{0, 0}, {0, 1}, {1, 0}) < 0.0,
        "predicates.orient2d", "counterclockwise > 0, clockwise < 0");
  check(shewchuk::orient3d(shewchuk::point3{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, -1}) > 0.0,
        "predicates.orient3d",
        "a point below a counterclockwise triangle is > 0 (Shewchuk's sign)");
  check(shewchuk::incircle(shewchuk::point2{1, 0}, {0, 1}, {-1, 0}, {0, -1}) == 0.0 &&
            shewchuk::incircle(shewchuk::point2{1, 0}, {0, 1}, {-1, 0}, {0, 0}) > 0.0,
        "predicates.incircle", "on the circle 0, at its centre > 0");
  check(shewchuk::insphere(shewchuk::point3{1, 0, 0}, {0, 1, 0}, {-1, 0, 0}, {0, 0, 1},
                           {0, 0, -1}) == 0.0,
        "predicates.insphere", "a fifth point on the sphere is 0");

  const grid_result g = predicate_grids();
  check(g.wrong == 0, "predicates.near_degenerate",
        fmt("%ld of %ld signs wrong (%ld exactly degenerate) on grids spaced 2^-53", g.wrong,
            g.cases, g.zeros));
  const grid_result n = predicate_integers(7u);
  check(n.wrong == 0 && n.zeros > 1000, "predicates.integers",
        fmt("%ld of %ld signs wrong against exact int64 determinants (%ld exactly degenerate)",
            n.wrong, n.cases, n.zeros));

  // Reentrant: the same grids on two threads at once.
  grid_result t1r, t2r;
  std::thread t1([&] { t1r = predicate_grids(); });
  std::thread t2([&] { t2r = predicate_integers(7u); });
  t1.join();
  t2.join();
  check(t1r.wrong == 0 && t2r.wrong == 0 && t1r.cases == g.cases && t2r.zeros == n.zeros,
        "predicates.threads", "the same answers on two threads at once");
}
#endif // CVC_WITH_SHEWCHUK

// ---------------------------------------------------------------------------------------------
// CDT

#ifdef CVC_WITH_CDT
void test_cdt() {
  // A unit square with its centre and one constrained diagonal: 4 triangles. The centre lies on
  // the diagonal, so CDT splits that constraint there: 6 fixed edges, the four sides and the
  // diagonal's two halves.
  CDT::Triangulation<double> cdt;
  const std::vector<CDT::V2d<double>> v = {{0, 0}, {1, 0}, {1, 1}, {0, 1}, {0.5, 0.5}};
  const std::vector<CDT::Edge> e = {CDT::Edge(0, 1), CDT::Edge(1, 2), CDT::Edge(2, 3),
                                    CDT::Edge(3, 0), CDT::Edge(0, 2)};
  cdt.insertVertices(v);
  cdt.insertEdges(e);
  cdt.eraseSuperTriangle();
  const bool halves =
      cdt.fixedEdges.count(CDT::Edge(0, 4)) && cdt.fixedEdges.count(CDT::Edge(4, 2));
  check(cdt.triangles.size() == 4 && cdt.fixedEdges.size() == 6 && halves, "cdt.constrained_square",
        fmt("%zu triangles (4), %zu fixed edges (6), diagonal split at the centre: %s",
            cdt.triangles.size(), cdt.fixedEdges.size(), halves ? "yes" : "no"));
  // Without the centre, the square and its diagonal: 2 triangles, the diagonal an edge.
  CDT::Triangulation<double> sq;
  sq.insertVertices(std::vector<CDT::V2d<double>>(v.begin(), v.begin() + 4));
  sq.insertEdges(e);
  sq.eraseSuperTriangle();
  bool diagonal = false;
  for (const auto &t : sq.triangles) {
    for (int k = 0; k < 3; ++k) {
      const auto a = t.vertices[k], b = t.vertices[(k + 1) % 3];
      diagonal |= (a == 0 && b == 2) || (a == 2 && b == 0);
    }
  }
  check(sq.triangles.size() == 2 && diagonal, "cdt.constrained_diagonal",
        fmt("%zu triangles (2), the constrained diagonal an edge: %s", sq.triangles.size(),
            diagonal ? "yes" : "no"));

  // Delaunay: no vertex strictly inside any triangle's circumcircle (Shewchuk's incircle).
#ifdef CVC_WITH_SHEWCHUK
  const std::vector<double> c = cloud(3u, 300);
  std::vector<CDT::V2d<double>> pts;
  for (int i = 0; i < 300; ++i) {
    pts.push_back({c[3 * i], c[3 * i + 1]});
  }
  CDT::Triangulation<double> dt;
  dt.insertVertices(pts);
  dt.eraseSuperTriangle();
  long bad = 0, flat = 0;
  for (const auto &t : dt.triangles) {
    const shewchuk::point2 a = {pts[t.vertices[0]].x, pts[t.vertices[0]].y};
    const shewchuk::point2 b = {pts[t.vertices[1]].x, pts[t.vertices[1]].y};
    const shewchuk::point2 cc = {pts[t.vertices[2]].x, pts[t.vertices[2]].y};
    const int o = sgn(shewchuk::orient2d(a, b, cc));
    flat += o == 0;
    for (std::size_t k = 0; k < pts.size(); ++k) {
      bad += o * sgn(shewchuk::incircle(a, b, cc, {pts[k].x, pts[k].y})) > 0;
    }
  }
  check(bad == 0 && flat == 0 && dt.triangles.size() > 500, "cdt.delaunay",
        fmt("%zu triangles of 300 points, %ld flat, %ld vertices inside a circumcircle",
            dt.triangles.size(), flat, bad));
#else
  std::printf("SKIP cdt.delaunay: needs Shewchuk's predicates\n");
#endif
}
#endif // CVC_WITH_CDT

// ---------------------------------------------------------------------------------------------
// Clipper2

#ifdef CVC_WITH_CLIPPER2
void test_clipper2() {
  using namespace Clipper2Lib;
  // Two 2 x 2 squares overlapping in a 1 x 1 square, at 1000 units per unit.
  const Paths64 s1 = {MakePath({0, 0, 2000, 0, 2000, 2000, 0, 2000})};
  const Paths64 s2 = {MakePath({1000, 1000, 3000, 1000, 3000, 3000, 1000, 3000})};
  const double uni = Area(Union(s1, s2, FillRule::NonZero)) / 1e6;
  const double inter = Area(Intersect(s1, s2, FillRule::NonZero)) / 1e6;
  const double diff = Area(Difference(s1, s2, FillRule::NonZero)) / 1e6;
  check(uni == 7 && inter == 1 && diff == 3, "clipper2.booleans",
        fmt("union %g (7), intersection %g (1), difference %g (3)", uni, inter, diff));
  const double mitre = Area(InflatePaths(s1, 500, JoinType::Miter, EndType::Polygon)) / 1e6;
  check(mitre == 9, "clipper2.offset_miter",
        fmt("2 x 2 square grown 0.5, mitred: area %g (9)", mitre));
  // Round joins: the corners are polygons within arc_tolerance (0.05 units) of the true arcs,
  // so the area falls short by at most the arc length times that, 2 pi 500 x 0.05 = 157 units^2.
  const double round =
      Area(InflatePaths(s1, 500, JoinType::Round, EndType::Polygon, 2.0, 0.05)) / 1e6;
  const double pi = 3.14159265358979323846;
  const double round_want = 4 + 4 * 2 * 0.5 + pi * 0.25;
  check(std::fabs(round - round_want) < 1e-3, "clipper2.offset_round",
        fmt("rounded: area %.6f (%.6f, arcs approximated)", round, round_want));
  const double shrunk = Area(InflatePaths(s1, -500, JoinType::Miter, EndType::Polygon)) / 1e6;
  check(shrunk == 1, "clipper2.offset_inward", fmt("shrunk 0.5: area %g (1)", shrunk));
  const Path64 dot = MakePath({-100, -100, 100, -100, 100, 100, -100, 100});
  Paths64 swept = MinkowskiSum(dot, s1[0], true);
  swept.push_back(s1[0]);
  const double mink = std::fabs(Area(Union(swept, FillRule::NonZero))) / 1e6;
  check(mink == 4.84, "clipper2.minkowski",
        fmt("2 x 2 square plus a 0.2 x 0.2 square: area %g (4.84)", mink));
}
#endif

// ---------------------------------------------------------------------------------------------
// Qhull

#ifdef CVC_WITH_QHULL
struct delaunay_result {
  int exitcode = -1;
  std::vector<std::array<int, 4>> tets; // input indices, each sorted, the list sorted
};

// Delaunay tetrahedra through the reentrant API: every call owns its qhT, so calls on different
// threads share nothing.
delaunay_result qhull_delaunay(const std::vector<double> &xyz, const char *options) {
  delaunay_result r;
  const std::unique_ptr<qhT> holder(new qhT); // large, and a wasm thread's stack is small
  qhT *qh = holder.get();
  qh_zero(qh, stderr);
  std::vector<double> pts = xyz; // qh_new_qhull keeps pointers into its input
  std::string cmd = std::string("qhull ") + options;
  r.exitcode =
      qh_new_qhull(qh, 3, int(pts.size() / 3), pts.data(), False, &cmd[0], nullptr, stderr);
  if (r.exitcode == 0) {
    facetT *facet;
    vertexT *vertex, **vertexp;
    FORALLfacets {
      if (facet->upperdelaunay) {
        continue;
      }
      std::array<int, 4> t{};
      int k = 0;
      FOREACHvertex_(facet->vertices) {
        if (k < 4) {
          t[k] = qh_pointid(qh, vertex->point);
        }
        ++k;
      }
      if (k == 4) {
        std::sort(t.begin(), t.end());
        r.tets.push_back(t);
      } else {
        r.exitcode = -2; // a non-simplicial facet: Qt should have triangulated it
      }
    }
    std::sort(r.tets.begin(), r.tets.end());
  }
  qh_freeqhull(qh, !qh_ALL);
  int curlong = 0, totlong = 0;
  qh_memfreeshort(qh, &curlong, &totlong);
  return r;
}

point3 pt(const std::vector<double> &xyz, int i) {
  return {xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]};
}

// Six times a tetrahedron's signed volume, in plain doubles: exact for the cube's coordinates
// (0, 0.5 and 1), so it needs no exact predicate.
double volume6(const point3 &a, const point3 &b, const point3 &c, const point3 &d) {
  const double u[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
  const double v[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
  const double w[3] = {d[0] - a[0], d[1] - a[1], d[2] - a[2]};
  return u[0] * (v[1] * w[2] - v[2] * w[1]) - u[1] * (v[0] * w[2] - v[2] * w[0]) +
         u[2] * (v[0] * w[1] - v[1] * w[0]);
}

void test_qhull() {
  const char *opts = "d Qbb Qt Qz";
  // A cube's corners plus its centre: exactly 12 tetrahedra (two per face), none flat. The
  // corners are cospherical; Qz keeps Qt from adding flat slivers there.
  std::vector<double> cube;
  for (int i = 0; i < 8; ++i) {
    cube.push_back(i & 1);
    cube.push_back((i >> 1) & 1);
    cube.push_back((i >> 2) & 1);
  }
  cube.insert(cube.end(), {0.5, 0.5, 0.5});
  const delaunay_result cr = qhull_delaunay(cube, opts);
  int flat = 0, with_centre = 0;
  for (const auto &t : cr.tets) {
    flat += volume6(pt(cube, t[0]), pt(cube, t[1]), pt(cube, t[2]), pt(cube, t[3])) == 0.0;
    with_centre += t[3] == 8;
  }
  check(cr.exitcode == 0 && cr.tets.size() == 12 && flat == 0 && with_centre == 12,
        "qhull.cube_and_centre",
        fmt("exit %d, %zu tetrahedra (12), %d flat (0), %d at the centre (12)", cr.exitcode,
            cr.tets.size(), flat, with_centre));

  // Empty circumspheres (Shewchuk's insphere) on a 300-point cloud.
#ifdef CVC_WITH_SHEWCHUK
  const std::vector<double> small = cloud(4u, 300);
  const delaunay_result sr = qhull_delaunay(small, opts);
  long bad = 0, flat_s = 0;
  for (const auto &t : sr.tets) {
    const auto a = pt(small, t[0]), b = pt(small, t[1]), c = pt(small, t[2]), d = pt(small, t[3]);
    const int o = sgn(shewchuk::orient3d(a, b, c, d));
    flat_s += o == 0;
    for (int k = 0; k < 300; ++k) {
      bad += o * sgn(shewchuk::insphere(a, b, c, d, pt(small, k))) > 0;
    }
  }
  check(sr.exitcode == 0 && bad == 0 && flat_s == 0 && sr.tets.size() > 1500, "qhull.delaunay",
        fmt("%zu tetrahedra of 300 points, %ld flat, %ld points inside a circumsphere",
            sr.tets.size(), flat_s, bad));
#else
  std::printf("SKIP qhull.delaunay: needs Shewchuk's predicates\n");
#endif

  // Reentrant: two triangulations on two threads equal the sequential ones, tetrahedron for
  // tetrahedron.
  const std::vector<double> a = cloud(1u, 2000), b = cloud(2u, 3000);
  const delaunay_result as = qhull_delaunay(a, opts), bs = qhull_delaunay(b, opts);
  delaunay_result at, bt;
  std::thread t1([&] { at = qhull_delaunay(a, opts); });
  std::thread t2([&] { bt = qhull_delaunay(b, opts); });
  t1.join();
  t2.join();
  check(as.exitcode == 0 && bs.exitcode == 0 && at.tets == as.tets && bt.tets == bs.tets &&
            as.tets.size() > 10000,
        "qhull.threads",
        fmt("2000 and 3000 points: %zu and %zu tetrahedra on two threads, %zu and %zu alone",
            at.tets.size(), bt.tets.size(), as.tets.size(), bs.tets.size()));
}
#endif

} // namespace

int main() {
#ifdef CVC_WITH_SHEWCHUK
  test_predicates();
#else
  std::printf("SKIP predicates: libcvc was built without Shewchuk's predicates\n");
#endif
#ifdef CVC_WITH_CDT
  test_cdt();
#else
  std::printf("SKIP cdt: libcvc was built without CDT\n");
#endif
#ifdef CVC_WITH_CLIPPER2
  test_clipper2();
#else
  std::printf("SKIP clipper2: libcvc was built without Clipper2\n");
#endif
#ifdef CVC_WITH_QHULL
  test_qhull();
#else
  std::printf("SKIP qhull: libcvc was built without Qhull\n");
#endif
  std::printf("permissive_libs_test: %d checks, %d failed\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
