// cgal_ab_permissive.cpp -- row T16 of the VolumeRover ports plan (its §1.2 and §1.7): each
// permissive library against the CGAL package it replaces, on the same inputs, for results and
// time. See README.md.
//
//   A: CGAL (GPL-3+)                              B: permissive (libcvc)
//   orientation / side_of_oriented_circle /       Shewchuk's predicates (public domain,
//     side_of_oriented_sphere, Epick (exact)        cvcpkg shewchuk-predicates)
//   Delaunay_triangulation_3                      reentrant Qhull, `d Qbb Qt Qz` (and the P1
//                                                   spike: Qhull by incremental insertion)
//   Constrained_Delaunay_triangulation_2          CDT 2.0.1 (MPL-2.0, cvcpkg cdt)
//   Polygon_set_2 Booleans, Epeck                 Clipper2 Booleans (BSL-1.0)
//   straight-skeleton offsets (mitred)            Clipper2 InflatePaths, JoinType::Miter
//   minkowski_sum_2                               Clipper2 MinkowskiSum
//
// Exact where the two must agree: predicate signs; triangulations of points in general
// position; Booleans and Minkowski sums whose vertices all land on the integer grid. By
// tolerance where they differ by design: Clipper2 rounds new vertices to its int64 grid (area
// within the perimeter times one unit), and degenerate (cospherical, cocircular) point sets have
// many Delaunay triangulations (checked for validity instead: empty spheres, a manifold tiling
// of the same volume, the same count where it is fixed).

#include "cgal_ab.h"

#include <CDT/CDT.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_face_base_2.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Triangulation_data_structure_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <CGAL/create_offset_polygons_from_polygon_with_holes_2.h>
#include <CGAL/minkowski_sum_2.h>
#include <algorithm>
#include <array>
#include <clipper2/clipper.h>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iterator>
#include <map>
#include <shewchuk/predicates.hpp>
#include <string>
#include <utility>
#include <vector>

namespace cgal_ab {
namespace {

using Epick = CGAL::Exact_predicates_inexact_constructions_kernel;
using Epeck = CGAL::Exact_predicates_exact_constructions_kernel;

template <typename... Args> std::string fmt(const char *f, Args... args) {
  char buf[512];
  std::snprintf(buf, sizeof buf, f, args...);
  return buf;
}

int sgn(double v) { return (v > 0.0) - (v < 0.0); }

// =============================================================================================
// Predicates: Shewchuk's signs against CGAL's exact ones.

struct quad2 {
  shewchuk::point2 p[4];
};
struct quint3 {
  shewchuk::point3 p[5];
};

// Random, nearly degenerate (points rounded onto a line / circle / plane / sphere) and exactly
// degenerate (small integers) inputs, a third of each.
void make_predicate_inputs(rng &r, int n, std::vector<quad2> &q2, std::vector<quint3> &q3) {
  const double pi = 3.14159265358979323846;
  for (int i = 0; i < n; ++i) {
    quad2 a{};
    quint3 b{};
    switch (i % 3) {
    case 0:
      for (auto &p : a.p)
        p = {r.uniform(-1, 1), r.uniform(-1, 1)};
      for (auto &p : b.p)
        p = {r.uniform(-1, 1), r.uniform(-1, 1), r.uniform(-1, 1)};
      break;
    case 1: {
      // 2D: three points on a line (rounded), the fourth on the circle through three others.
      const shewchuk::point2 o = {r.uniform(-10, 10), r.uniform(-10, 10)};
      const double ang = r.uniform(0, 2 * pi), rad = r.uniform(0.1, 100);
      if (i % 2) {
        for (int k = 0; k < 4; ++k) {
          const double t = r.uniform(-100, 100);
          a.p[k] = {o[0] + t * std::cos(ang), o[1] + t * std::sin(ang)};
        }
      } else {
        for (int k = 0; k < 4; ++k) {
          const double t = r.uniform(0, 2 * pi);
          a.p[k] = {o[0] + rad * std::cos(t), o[1] + rad * std::sin(t)};
        }
      }
      // 3D: four points on a plane (rounded), or five on a sphere.
      const shewchuk::point3 c = {r.uniform(-10, 10), r.uniform(-10, 10), r.uniform(-10, 10)};
      if (i % 2) {
        const shewchuk::point3 u = {r.uniform(-1, 1), r.uniform(-1, 1), r.uniform(-1, 1)};
        const shewchuk::point3 v = {r.uniform(-1, 1), r.uniform(-1, 1), r.uniform(-1, 1)};
        for (int k = 0; k < 5; ++k) {
          const double s = r.uniform(-50, 50), t = r.uniform(-50, 50);
          b.p[k] = {c[0] + s * u[0] + t * v[0], c[1] + s * u[1] + t * v[1],
                    c[2] + s * u[2] + t * v[2]};
        }
      } else {
        for (int k = 0; k < 5; ++k) {
          const double th = std::acos(r.uniform(-1, 1)), ph = r.uniform(0, 2 * pi);
          b.p[k] = {c[0] + rad * std::sin(th) * std::cos(ph),
                    c[1] + rad * std::sin(th) * std::sin(ph), c[2] + rad * std::cos(th)};
        }
      }
      break;
    }
    default:
      for (auto &p : a.p)
        p = {double(int(r.next() % 7) - 3), double(int(r.next() % 7) - 3)};
      for (auto &p : b.p)
        p = {double(int(r.next() % 5) - 2), double(int(r.next() % 5) - 2),
             double(int(r.next() % 5) - 2)};
      break;
    }
    q2.push_back(a);
    q3.push_back(b);
  }
}

void ab_predicates(rng &r) {
  std::vector<quad2> q2;
  std::vector<quint3> q3;
  make_predicate_inputs(r, 300000, q2, q3);
  auto p2 = [](const shewchuk::point2 &p) { return Epick::Point_2(p[0], p[1]); };
  auto p3 = [](const shewchuk::point3 &p) { return Epick::Point_3(p[0], p[1], p[2]); };

  // Sign conventions (cgal_ab README): orient2d and incircle agree with CGAL's orientation and
  // side_of_oriented_circle; orient3d and insphere have the opposite sign to CGAL's.
  struct tally {
    long n = 0, wrong = 0, zeros = 0;
    long sum_a = 0, sum_b = 0; // keeps the timed loops from being optimised away
  };
  auto run = [&](const char *name, auto cgal_sign, auto ours_sign, std::size_t count) {
    std::vector<signed char> a(count), b(count);
    tally t;
    const double ta = ms_of([&] {
      for (std::size_t i = 0; i < count; ++i)
        a[i] = static_cast<signed char>(cgal_sign(i));
    });
    const double tb = ms_of([&] {
      for (std::size_t i = 0; i < count; ++i)
        b[i] = static_cast<signed char>(ours_sign(i));
    });
    for (std::size_t i = 0; i < count; ++i) {
      ++t.n;
      t.wrong += a[i] != b[i];
      t.zeros += a[i] == 0;
    }
    report(t.wrong == 0, std::string("predicates ") + name,
           fmt("%ld of %ld signs differ from CGAL's exact ones (%ld exactly degenerate)", t.wrong,
               t.n, t.zeros));
    report_time(std::string("predicates ") + name, "cgal", ta, "shewchuk", tb);
  };
  run(
      "orient2d",
      [&](std::size_t i) {
        return int(CGAL::orientation(p2(q2[i].p[0]), p2(q2[i].p[1]), p2(q2[i].p[2])));
      },
      [&](std::size_t i) { return sgn(shewchuk::orient2d(q2[i].p[0], q2[i].p[1], q2[i].p[2])); },
      q2.size());
  run(
      "incircle",
      [&](std::size_t i) {
        return int(CGAL::side_of_oriented_circle(p2(q2[i].p[0]), p2(q2[i].p[1]), p2(q2[i].p[2]),
                                                 p2(q2[i].p[3])));
      },
      [&](std::size_t i) {
        return sgn(shewchuk::incircle(q2[i].p[0], q2[i].p[1], q2[i].p[2], q2[i].p[3]));
      },
      q2.size());
  run(
      "orient3d",
      [&](std::size_t i) {
        return -int(
            CGAL::orientation(p3(q3[i].p[0]), p3(q3[i].p[1]), p3(q3[i].p[2]), p3(q3[i].p[3])));
      },
      [&](std::size_t i) {
        return sgn(shewchuk::orient3d(q3[i].p[0], q3[i].p[1], q3[i].p[2], q3[i].p[3]));
      },
      q3.size());
  run(
      "insphere",
      [&](std::size_t i) {
        return -int(CGAL::side_of_oriented_sphere(p3(q3[i].p[0]), p3(q3[i].p[1]), p3(q3[i].p[2]),
                                                  p3(q3[i].p[3]), p3(q3[i].p[4])));
      },
      [&](std::size_t i) {
        return sgn(shewchuk::insphere(q3[i].p[0], q3[i].p[1], q3[i].p[2], q3[i].p[3], q3[i].p[4]));
      },
      q3.size());
}

// =============================================================================================
// 3D Delaunay: Qhull (and the incremental spike) against Delaunay_triangulation_3.

using tets = std::vector<std::array<int, 4>>;
using Vb3 = CGAL::Triangulation_vertex_base_with_info_3<int, Epick>;
using Tds3 = CGAL::Triangulation_data_structure_3<Vb3>;
using DT3 = CGAL::Delaunay_triangulation_3<Epick, Tds3>;

tets cgal_dt3(const std::vector<double> &xyz) {
  std::vector<std::pair<DT3::Point, int>> pts;
  for (int i = 0; i < int(xyz.size() / 3); ++i)
    pts.emplace_back(DT3::Point(xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]), i);
  const DT3 dt(pts.begin(), pts.end());
  tets out;
  out.reserve(dt.number_of_finite_cells());
  for (auto c = dt.finite_cells_begin(); c != dt.finite_cells_end(); ++c) {
    std::array<int, 4> t = {c->vertex(0)->info(), c->vertex(1)->info(), c->vertex(2)->info(),
                            c->vertex(3)->info()};
    std::sort(t.begin(), t.end());
    out.push_back(t);
  }
  std::sort(out.begin(), out.end());
  return out;
}

std::size_t count_common(const tets &a, const tets &b) {
  std::size_t n = 0;
  for (std::size_t i = 0, j = 0; i < a.size() && j < b.size();) {
    if (a[i] < b[j])
      ++i;
    else if (b[j] < a[i])
      ++j;
    else {
      ++n;
      ++i;
      ++j;
    }
  }
  return n;
}

shewchuk::point3 at(const std::vector<double> &xyz, int i) {
  return {xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]};
}

// Is a set of tetrahedra a valid tiling? None flat, no triangle in more than two of them, and
// the total volume (to roundoff) of CGAL's triangulation, which is the hull's.
struct tiling {
  long flat = 0, overfull_faces = 0;
  double volume = 0;
  // Non-flat tetrahedra with a point strictly inside their circumsphere: exactly (Shewchuk's
  // insphere), and by more than a relative 1e-9 of the radius. -1 when not checked.
  long not_empty = -1, not_empty_tol = -1;
};

// The circumcentre of a non-flat tetrahedron, and its squared radius.
void circumsphere(const shewchuk::point3 &a, const shewchuk::point3 &b, const shewchuk::point3 &c,
                  const shewchuk::point3 &d, double centre[3], double &r2) {
  const double ba[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
  const double ca[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
  const double da[3] = {d[0] - a[0], d[1] - a[1], d[2] - a[2]};
  auto cross = [](const double u[3], const double v[3], double w[3]) {
    w[0] = u[1] * v[2] - u[2] * v[1];
    w[1] = u[2] * v[0] - u[0] * v[2];
    w[2] = u[0] * v[1] - u[1] * v[0];
  };
  double cd[3], db[3], bc[3];
  cross(ca, da, cd);
  cross(da, ba, db);
  cross(ba, ca, bc);
  const double l_b = ba[0] * ba[0] + ba[1] * ba[1] + ba[2] * ba[2];
  const double l_c = ca[0] * ca[0] + ca[1] * ca[1] + ca[2] * ca[2];
  const double l_d = da[0] * da[0] + da[1] * da[1] + da[2] * da[2];
  const double den = 2 * (ba[0] * cd[0] + ba[1] * cd[1] + ba[2] * cd[2]);
  double off[3];
  for (int k = 0; k < 3; ++k) {
    off[k] = (l_b * cd[k] + l_c * db[k] + l_d * bc[k]) / den;
    centre[k] = a[k] + off[k];
  }
  r2 = off[0] * off[0] + off[1] * off[1] + off[2] * off[2];
}

tiling check_tiling(const std::vector<double> &xyz, const tets &ts, bool check_empty) {
  tiling t;
  std::map<std::array<int, 3>, int> faces;
  for (const auto &q : ts) {
    const double o = shewchuk::orient3d(at(xyz, q[0]), at(xyz, q[1]), at(xyz, q[2]), at(xyz, q[3]));
    t.flat += o == 0.0;
    t.volume += std::fabs(o) / 6.0;
    for (int skip = 0; skip < 4; ++skip) {
      std::array<int, 3> f{};
      for (int k = 0, m = 0; k < 4; ++k)
        if (k != skip)
          f[m++] = q[k];
      if (++faces[f] > 2)
        ++t.overfull_faces;
    }
  }
  if (check_empty) {
    t.not_empty = t.not_empty_tol = 0;
    const int n = int(xyz.size() / 3);
    for (const auto &q : ts) {
      const auto a = at(xyz, q[0]), b = at(xyz, q[1]), c = at(xyz, q[2]), d = at(xyz, q[3]);
      const int o = sgn(shewchuk::orient3d(a, b, c, d));
      if (o == 0)
        continue; // flat: counted above, and it has no circumsphere
      double centre[3], r2;
      circumsphere(a, b, c, d, centre, r2);
      bool exact_hit = false, tol_hit = false;
      for (int k = 0; k < n && !(exact_hit && tol_hit); ++k) {
        const auto p = at(xyz, k);
        exact_hit |= o * sgn(shewchuk::insphere(a, b, c, d, p)) > 0;
        const double dx = p[0] - centre[0], dy = p[1] - centre[1], dz = p[2] - centre[2];
        tol_hit |= dx * dx + dy * dy + dz * dz < r2 * (1 - 2e-9);
      }
      t.not_empty += exact_hit;
      t.not_empty_tol += tol_hit;
    }
  }
  return t;
}

void ab_delaunay3(const std::string &name, const std::vector<double> &xyz, bool general_position,
                  bool spike) {
  tets a;
  qhull_tets b;
  const double ta = ms_of([&] { a = cgal_dt3(xyz); });
  const double tb = ms_of([&] { b = qhull_batch(xyz); });
  const int n = int(xyz.size() / 3);
  const bool small = n <= 3000;
  if (general_position) {
    // A unique triangulation: the two must be identical, tetrahedron for tetrahedron.
    const bool same = b.exitcode == 0 && a == b.tets;
    report(same, "delaunay3 " + name,
           fmt("cgal %zu tetrahedra, qhull %zu (exit %d), %zu in common", a.size(), b.tets.size(),
               b.exitcode, count_common(a, b.tets)));
  } else {
    // Cospherical subsets have many Delaunay triangulations, so CGAL (symbolic perturbation)
    // and Qhull (Qt) pick different ones. Qhull's must still tile the hull (no face in three
    // tetrahedra, CGAL's volume) with empty circumspheres to a relative 1e-9: Qhull's precision
    // model treats points within roundoff of a sphere as on it, so on nearly cospherical input
    // its tetrahedra need not be empty by exact predicates (reported, not asserted). Qt also
    // leaves flat tetrahedra where exactly cospherical points merged (reported, not asserted;
    // P1 must drop or avoid them).
    const tiling ca = check_tiling(xyz, a, false);
    const tiling cb = check_tiling(xyz, b.tets, small);
    const double rel = std::fabs(cb.volume - ca.volume) / ca.volume;
    const bool ok =
        b.exitcode == 0 && cb.overfull_faces == 0 && rel < 1e-9 && cb.not_empty_tol <= 0;
    report(ok, "delaunay3 " + name,
           fmt("cgal %zu tetrahedra (%ld flat), qhull %zu (exit %d, %zu in common); qhull's: %ld "
               "flat, %ld faces in 3+ tetrahedra, volume off by %.1e, non-empty circumspheres "
               "%ld to 1e-9 and %ld exactly%s",
               a.size(), ca.flat, b.tets.size(), b.exitcode, count_common(a, b.tets), cb.flat,
               cb.overfull_faces, rel, cb.not_empty_tol, cb.not_empty,
               small ? "" : " (not checked: large)"));
  }
  report_time("delaunay3 " + name, "cgal", ta, "qhull", tb);
  if (!spike)
    return;

  // The P1 spike: Qhull by incremental insertion, against the batch build. Reported, not
  // asserted (SPIKE lines): P1's first PR turns what holds into tests.
  struct variant {
    const char *label;
    insert_order order;
    bool warm;
  };
  const variant variants[] = {{"morton, warm start", insert_order::morton, true},
                              {"morton, qh_findbestfacet", insert_order::morton, false},
                              {"input order, qh_findbestfacet", insert_order::input, false}};
  for (const variant &v : variants) {
    if (!v.warm && n > 50000)
      continue; // a cold start walks from the oldest facet every time: minutes at 100k points
    qhull_tets c;
    const double tc = ms_of([&] { c = qhull_incremental(xyz, v.order, v.warm); });
    const std::size_t common = count_common(b.tets, c.tets);
    const bool same = c.exitcode == 0 && c.tets == b.tets;
    std::string valid;
    if (!same && c.exitcode == 0) {
      const tiling cc = check_tiling(xyz, c.tets, small);
      valid = fmt("; incremental: %ld flat, %ld faces in 3+, volume %.6g vs %.6g, non-empty "
                  "circumspheres %ld to 1e-9 and %ld exactly",
                  cc.flat, cc.overfull_faces, cc.volume, check_tiling(xyz, a, false).volume,
                  cc.not_empty_tol, cc.not_empty);
    }
    std::printf("SPIKE qhull incremental (%s) %s: %s batch, %zu vs %zu tetrahedra, %zu in common, "
                "exit %d, %d points not added, %s CGAL%s; %.2f ms vs batch %.2f ms (%.2fx)\n",
                v.label, name.c_str(), same ? "IDENTICAL to" : "DIFFERS from", c.tets.size(),
                b.tets.size(), common, c.exitcode, c.not_added,
                c.tets == a ? "identical to" : "differs from", valid.c_str(), tc, tb,
                tb > 0 ? tc / tb : 0.0);
    std::fflush(stdout);
  }
}

std::vector<double> uniform_cloud(rng &r, int n) {
  std::vector<double> p(3 * n);
  for (double &x : p)
    x = r.uniform(0, 1);
  return p;
}

void ab_delaunay3_all(rng &r) {
  ab_delaunay3("uniform n=2000", uniform_cloud(r, 2000), true, true);
  ab_delaunay3("uniform n=20000", uniform_cloud(r, 20000), true, true);
  ab_delaunay3("uniform n=100000", uniform_cloud(r, 100000), true, true);
  {
    // A Gaussian clump: dense in the middle, sparse outside.
    std::vector<double> p;
    for (int i = 0; i < 3 * 20000; ++i) {
      double s = 0;
      for (int k = 0; k < 6; ++k)
        s += r.uniform(-1, 1);
      p.push_back(s);
    }
    ab_delaunay3("clump n=20000", p, true, true);
  }
  {
    // Points on a sphere, rounded: nearly cospherical everywhere.
    std::vector<double> p;
    const double pi = 3.14159265358979323846;
    for (int i = 0; i < 2000; ++i) {
      const double th = std::acos(r.uniform(-1, 1)), ph = r.uniform(0, 2 * pi);
      p.insert(p.end(), {std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph), std::cos(th)});
    }
    ab_delaunay3("sphere n=2000 (nearly cospherical)", p, false, true);
  }
  {
    // A cube's corners plus its centre, and a 10^3 lattice: exactly cospherical subsets.
    std::vector<double> cube;
    for (int i = 0; i < 8; ++i)
      cube.insert(cube.end(), {double(i & 1), double((i >> 1) & 1), double((i >> 2) & 1)});
    cube.insert(cube.end(), {0.5, 0.5, 0.5});
    ab_delaunay3("cube and centre (cospherical)", cube, false, false);
    std::vector<double> lattice;
    for (int z = 0; z < 10; ++z)
      for (int y = 0; y < 10; ++y)
        for (int x = 0; x < 10; ++x)
          lattice.insert(lattice.end(), {double(x), double(y), double(z)});
    ab_delaunay3("lattice 10^3 (cospherical)", lattice, false, true);
  }
}

// =============================================================================================
// 2D constrained Delaunay: CDT against Constrained_Delaunay_triangulation_2.

using tris = std::vector<std::array<int, 3>>;
using Vb2 = CGAL::Triangulation_vertex_base_with_info_2<int, Epick>;
using Fb2 = CGAL::Constrained_triangulation_face_base_2<Epick>;
using Tds2 = CGAL::Triangulation_data_structure_2<Vb2, Fb2>;
using CDT2 = CGAL::Constrained_Delaunay_triangulation_2<Epick, Tds2, CGAL::Exact_predicates_tag>;

tris cgal_cdt(const std::vector<std::array<double, 2>> &pts,
              const std::vector<std::pair<int, int>> &edges) {
  std::vector<std::pair<CDT2::Point, int>> in;
  for (int i = 0; i < int(pts.size()); ++i)
    in.emplace_back(CDT2::Point(pts[i][0], pts[i][1]), i);
  CDT2 cdt;
  cdt.insert(in.begin(), in.end());
  std::vector<CDT2::Vertex_handle> vh(pts.size());
  for (auto v = cdt.finite_vertices_begin(); v != cdt.finite_vertices_end(); ++v)
    vh[v->info()] = v;
  for (const auto &e : edges)
    cdt.insert_constraint(vh[e.first], vh[e.second]);
  tris out;
  for (auto f = cdt.finite_faces_begin(); f != cdt.finite_faces_end(); ++f) {
    std::array<int, 3> t = {f->vertex(0)->info(), f->vertex(1)->info(), f->vertex(2)->info()};
    std::sort(t.begin(), t.end());
    out.push_back(t);
  }
  std::sort(out.begin(), out.end());
  return out;
}

tris our_cdt(const std::vector<std::array<double, 2>> &pts,
             const std::vector<std::pair<int, int>> &edges) {
  CDT::Triangulation<double> cdt;
  std::vector<CDT::V2d<double>> v;
  v.reserve(pts.size());
  for (const auto &p : pts)
    v.push_back({p[0], p[1]});
  cdt.insertVertices(v);
  if (!edges.empty()) {
    std::vector<CDT::Edge> e;
    for (const auto &ed : edges)
      e.emplace_back(CDT::VertInd(ed.first), CDT::VertInd(ed.second));
    cdt.insertEdges(e);
  }
  cdt.eraseSuperTriangle();
  tris out;
  out.reserve(cdt.triangles.size());
  for (const auto &t : cdt.triangles) {
    std::array<int, 3> a = {int(t.vertices[0]), int(t.vertices[1]), int(t.vertices[2])};
    std::sort(a.begin(), a.end());
    out.push_back(a);
  }
  std::sort(out.begin(), out.end());
  return out;
}

void ab_cdt(const std::string &name, const std::vector<std::array<double, 2>> &pts,
            const std::vector<std::pair<int, int>> &edges, bool general_position) {
  tris a, b;
  const double ta = ms_of([&] { a = cgal_cdt(pts, edges); });
  const double tb = ms_of([&] { b = our_cdt(pts, edges); });
  if (general_position) {
    report(a == b, "cdt " + name,
           fmt("cgal %zu triangles, CDT %zu, %s", a.size(), b.size(),
               a == b ? "identical" : "different"));
  } else {
    // Cocircular points: any Delaunay triangulation will do, and every triangulation of the same
    // points has the same number of triangles. Check CDT's circumcircles are empty.
    long not_empty = 0;
    for (const auto &t : b) {
      const shewchuk::point2 p = {pts[t[0]][0], pts[t[0]][1]}, q = {pts[t[1]][0], pts[t[1]][1]},
                             s = {pts[t[2]][0], pts[t[2]][1]};
      const int o = sgn(shewchuk::orient2d(p, q, s));
      for (const auto &v : pts)
        if (o * sgn(shewchuk::incircle(p, q, s, {v[0], v[1]})) > 0) {
          ++not_empty;
          break;
        }
    }
    report(a.size() == b.size() && not_empty == 0, "cdt " + name,
           fmt("cgal %zu triangles, CDT %zu; CDT's with a vertex inside the circumcircle: %ld",
               a.size(), b.size(), not_empty));
  }
  report_time("cdt " + name, "cgal", ta, "CDT", tb);
}

void ab_cdt_all(rng &r) {
  const double pi = 3.14159265358979323846;
  auto uniform = [&](int n) {
    std::vector<std::array<double, 2>> p(n);
    for (auto &q : p)
      q = {r.uniform(0, 1), r.uniform(0, 1)};
    return p;
  };
  ab_cdt("uniform n=2000", uniform(2000), {}, true);
  ab_cdt("uniform n=100000", uniform(100000), {}, true);
  {
    // A star-shaped polygon's 300 edges as constraints, among 5000 free points.
    std::vector<std::array<double, 2>> p = uniform(5000);
    std::vector<std::pair<int, int>> e;
    const int first = int(p.size()), k = 300;
    for (int i = 0; i < k; ++i) {
      const double t = 2 * pi * (i + r.uniform(0, 0.8)) / k, rad = r.uniform(0.2, 0.45);
      p.push_back({0.5 + rad * std::cos(t), 0.5 + rad * std::sin(t)});
      e.emplace_back(first + i, first + (i + 1) % k);
    }
    ab_cdt("star polygon constraints, n=5300", p, e, true);
  }
  {
    // A 60 x 60 grid: every unit square's corners are cocircular.
    std::vector<std::array<double, 2>> p;
    for (int y = 0; y < 60; ++y)
      for (int x = 0; x < 60; ++x)
        p.push_back({double(x), double(y)});
    ab_cdt("grid 60x60 (cocircular)", p, {}, false);
  }
}

// =============================================================================================
// Polygons: Clipper2 against CGAL's Booleans (Epeck, exact), straight-skeleton offsets and
// Minkowski sums, on integer coordinates.

using ipath = std::vector<std::array<std::int64_t, 2>>;
using Poly = CGAL::Polygon_2<Epeck>;
using Pwh = CGAL::Polygon_with_holes_2<Epeck>;

Poly to_cgal(const ipath &p) {
  Poly out;
  for (const auto &v : p)
    out.push_back(Epeck::Point_2(double(v[0]), double(v[1])));
  return out;
}

CGAL::Polygon_2<Epick> to_epick(const ipath &p) {
  CGAL::Polygon_2<Epick> out;
  for (const auto &v : p)
    out.push_back(Epick::Point_2(double(v[0]), double(v[1])));
  return out;
}

Clipper2Lib::Path64 to_clip(const ipath &p) {
  Clipper2Lib::Path64 out;
  for (const auto &v : p)
    out.emplace_back(v[0], v[1]);
  return out;
}

// A region's area, its outer boundaries and holes, and its perimeter.
struct region {
  double area = 0, perimeter = 0;
  int outers = 0, holes = 0;
};

double perimeter_of(const Poly &p) {
  double s = 0;
  for (auto e = p.edges_begin(); e != p.edges_end(); ++e)
    s += std::sqrt(CGAL::to_double(e->squared_length()));
  return s;
}

region of_cgal(const std::vector<Pwh> &ps) {
  region r;
  Epeck::FT area = 0;
  for (const Pwh &p : ps) {
    ++r.outers;
    area += p.outer_boundary().area();
    r.perimeter += perimeter_of(p.outer_boundary());
    for (auto h = p.holes_begin(); h != p.holes_end(); ++h) {
      ++r.holes;
      area += h->area(); // holes run clockwise: negative
      r.perimeter += perimeter_of(*h);
    }
  }
  r.area = CGAL::to_double(area);
  return r;
}

region of_clip(const Clipper2Lib::Paths64 &ps) {
  region r;
  for (const auto &p : ps) {
    const double a = Clipper2Lib::Area(p);
    r.area += a;
    (a > 0 ? r.outers : r.holes) += 1;
  }
  return r;
}

// Star-shaped polygon (counterclockwise): k vertices at increasing angles, random radii.
ipath star(rng &r, double cx, double cy, double rad, int k) {
  const double pi = 3.14159265358979323846;
  ipath p;
  for (int i = 0; i < k; ++i) {
    const double t = 2 * pi * (i + r.uniform(0, 0.8)) / k, q = r.uniform(0.35, 1.0) * rad;
    p.push_back({std::llround(cx + q * std::cos(t)), std::llround(cy + q * std::sin(t))});
  }
  return p;
}

// Convex polygon (counterclockwise): k points on a circle.
ipath convex(rng &r, double cx, double cy, double rad, int k) {
  const double pi = 3.14159265358979323846;
  ipath p;
  for (int i = 0; i < k; ++i) {
    const double t = 2 * pi * (i + r.uniform(0, 0.8)) / k;
    p.push_back({std::llround(cx + rad * std::cos(t)), std::llround(cy + rad * std::sin(t))});
  }
  return p;
}

// Orthogonal "skyline" polygon (counterclockwise): every edge axis-parallel, so every
// intersection with another such polygon is on the integer grid.
ipath skyline(rng &r, std::int64_t x0, std::int64_t width, std::int64_t y0, std::int64_t hmax,
              int n, bool transpose) {
  std::vector<std::int64_t> xs = {x0, x0 + width};
  while (int(xs.size()) < n + 1) {
    const std::int64_t x = x0 + 1 + std::int64_t(r.next() % std::uint64_t(width - 1));
    if (std::find(xs.begin(), xs.end(), x) == xs.end())
      xs.push_back(x);
  }
  std::sort(xs.begin(), xs.end());
  std::vector<std::int64_t> h(n);
  for (int i = 0; i < n; ++i)
    do
      h[i] = hmax / 5 + std::int64_t(r.next() % std::uint64_t(hmax - hmax / 5));
    while (i > 0 && h[i] == h[i - 1]);
  ipath p = {{xs[0], y0}, {xs[n], y0}};
  for (int i = n - 1; i >= 0; --i) {
    p.push_back({xs[i + 1], y0 + h[i]});
    p.push_back({xs[i], y0 + h[i]});
  }
  // The last vertex, (x0, y0 + h[0]), closes back to (x0, y0) along x = x0.
  if (transpose) { // swap x and y (a reflection), then reverse to stay counterclockwise
    for (auto &v : p)
      std::swap(v[0], v[1]);
    std::reverse(p.begin(), p.end());
  }
  return p;
}

enum class bool_op { join, intersect, difference, xor_ };
const char *op_name(bool_op o) {
  switch (o) {
  case bool_op::join:
    return "union";
  case bool_op::intersect:
    return "intersection";
  case bool_op::difference:
    return "difference";
  default:
    return "symmetric difference";
  }
}

void ab_boolean(const std::string &name, const ipath &p, const ipath &q, bool_op op, bool exact,
                double &ta_sum, double &tb_sum) {
  std::vector<Pwh> a;
  Clipper2Lib::Paths64 b;
  ta_sum += ms_of([&] {
    CGAL::Polygon_set_2<Epeck> s(to_cgal(p));
    const Poly cq = to_cgal(q);
    switch (op) {
    case bool_op::join:
      s.join(cq);
      break;
    case bool_op::intersect:
      s.intersection(cq);
      break;
    case bool_op::difference:
      s.difference(cq);
      break;
    default:
      s.symmetric_difference(cq);
      break;
    }
    s.polygons_with_holes(std::back_inserter(a));
  });
  tb_sum += ms_of([&] {
    using namespace Clipper2Lib;
    const Paths64 cp = {to_clip(p)}, cq = {to_clip(q)};
    switch (op) {
    case bool_op::join:
      b = Union(cp, cq, FillRule::NonZero);
      break;
    case bool_op::intersect:
      b = Intersect(cp, cq, FillRule::NonZero);
      break;
    case bool_op::difference:
      b = Difference(cp, cq, FillRule::NonZero);
      break;
    default:
      b = Xor(cp, cq, FillRule::NonZero);
      break;
    }
  });
  const region ra = of_cgal(a), rb = of_clip(b);
  const double tol = exact ? 0.0 : ra.perimeter; // a unit per unit of boundary
  // The pieces of a symmetric difference touch where the two boundaries cross. CGAL's
  // arrangement keeps pieces that touch at a point as one polygon with holes; Clipper2 returns
  // them as separate polygons. So there the area is compared, not the count.
  const bool same_topology = ra.outers == rb.outers && ra.holes == rb.holes;
  const bool ok = std::fabs(ra.area - rb.area) <= tol && (same_topology || op == bool_op::xor_);
  report(
      ok, std::string("clipper2 ") + op_name(op) + " " + name,
      fmt("area cgal %.1f, clipper2 %.1f (diff %.3g, %s %.3g); %d/%d outer, %d/%d holes%s", ra.area,
          rb.area, rb.area - ra.area, exact ? "exact, tol" : "tol", tol, ra.outers, rb.outers,
          ra.holes, rb.holes,
          op == bool_op::xor_ && !same_topology ? " (pieces touching at points, by design)" : ""));
}

// The straight skeleton's offset is the wavefront: when an edge shrinks to nothing, its neighbours
// meet along their own bisector. Clipper2's mitred offset joins each vertex's two edges and
// unions away the loops a collapsed edge leaves, which is a different polygon from then on. So
// the two agree (to rounding) until the first edge event, and on convex polygons, whose
// offsets are intersections of half-planes either way; past an event they differ by design.
// `asserted` says which: deep offsets of non-convex polygons are reported (INFO), not asserted.
void ab_offset(const std::string &name, const ipath &p, double d, bool asserted, double &ta_sum,
               double &tb_sum) {
  std::vector<Pwh> a;
  Clipper2Lib::Paths64 b;
  ta_sum += ms_of([&] {
    const CGAL::Polygon_2<Epick> pk = to_epick(p);
    const auto out =
        d > 0 ? CGAL::create_exterior_skeleton_and_offset_polygons_with_holes_2(d, pk, Epeck())
              : CGAL::create_interior_skeleton_and_offset_polygons_with_holes_2(-d, pk, Epeck());
    for (const auto &q : out)
      a.push_back(*q);
  });
  tb_sum += ms_of([&] {
    using namespace Clipper2Lib;
    // The straight skeleton's offset is mitred without limit; a large miter limit matches it.
    b = InflatePaths({to_clip(p)}, d, JoinType::Miter, EndType::Polygon, 1e6);
  });
  const region ra = of_cgal(a), rb = of_clip(b);
  const double tol = ra.perimeter;
  const bool ok =
      std::fabs(ra.area - rb.area) <= tol && ra.outers == rb.outers && ra.holes == rb.holes;
  const std::string detail =
      fmt("d=%g: area cgal %.1f, clipper2 %.1f (diff %.3g, tol %.3g); %d/%d outer, %d/%d holes", d,
          ra.area, rb.area, rb.area - ra.area, tol, ra.outers, rb.outers, ra.holes, rb.holes);
  if (asserted) {
    report(ok, "clipper2 offset " + name, detail);
  } else {
    std::printf("INFO clipper2 offset %s: %s, %s\n", name.c_str(), detail.c_str(),
                ok ? "agree" : "differ (past an edge event, by design)");
    std::fflush(stdout);
  }
}

void ab_minkowski(const std::string &name, const ipath &p, const ipath &q, bool exact,
                  double &ta_sum, double &tb_sum) {
  Pwh a;
  Clipper2Lib::Paths64 b;
  ta_sum += ms_of([&] { a = CGAL::minkowski_sum_2(to_cgal(p), to_cgal(q)); });
  tb_sum += ms_of([&] {
    using namespace Clipper2Lib;
    // MinkowskiSum sweeps p along q's boundary; adding q moved by a vertex of p (and p by one of
    // q) fills the inside, so the union is the whole sum.
    Paths64 parts = MinkowskiSum(to_clip(p), to_clip(q), true);
    Path64 qs = to_clip(q), ps = to_clip(p);
    for (auto &v : qs)
      v = Point64(v.x + p[0][0], v.y + p[0][1]);
    for (auto &v : ps)
      v = Point64(v.x + q[0][0], v.y + q[0][1]);
    parts.push_back(qs);
    parts.push_back(ps);
    b = Union(parts, FillRule::NonZero);
  });
  const region ra = of_cgal({a}), rb = of_clip(b);
  const double tol = exact ? 0.0 : ra.perimeter;
  const bool ok =
      std::fabs(ra.area - rb.area) <= tol && ra.outers == rb.outers && ra.holes == rb.holes;
  report(ok, "clipper2 minkowski " + name,
         fmt("area cgal %.1f, clipper2 %.1f (diff %.3g, %s %.3g); %d/%d outer, %d/%d holes",
             ra.area, rb.area, rb.area - ra.area, exact ? "exact, tol" : "tol", tol, ra.outers,
             rb.outers, ra.holes, rb.holes));
}

void ab_polygons(rng &r) {
  double ta = 0, tb = 0;
  const bool_op ops[] = {bool_op::join, bool_op::intersect, bool_op::difference, bool_op::xor_};
  for (int i = 0; i < 4; ++i) {
    // Orthogonal polygons: every new vertex is a grid point, so the areas must be equal.
    const ipath p = skyline(r, 0, 1000000, 0, 800000, 12 + 4 * i, false);
    const ipath q = skyline(r, 100000, 900000, 50000, 900000, 10 + 3 * i, true);
    for (bool_op op : ops)
      ab_boolean(fmt("skylines #%d", i), p, q, op, true, ta, tb);
  }
  for (int i = 0; i < 4; ++i) {
    // Star polygons: new vertices round onto Clipper2's grid.
    const ipath p = star(r, 0, 0, 1e6, 24 + 8 * i);
    const ipath q = star(r, 3e5, 2e5, 9e5, 20 + 8 * i);
    for (bool_op op : ops)
      ab_boolean(fmt("stars #%d", i), p, q, op, false, ta, tb);
  }
  report_time("clipper2 booleans (32 cases)", "cgal", ta, "clipper2", tb);

  ta = tb = 0;
  for (int i = 0; i < 4; ++i) {
    const ipath p = star(r, 0, 0, 1e6, 16 + 8 * i);
    const ipath c = convex(r, 0, 0, 1e6, 12 + 4 * i);
    // 5% of the radius comes before any edge event of these stars; 20% is past some.
    for (double d : {5e4, 2e5}) {
      const bool shallow = d < 1e5;
      ab_offset(fmt("star #%d outward", i), p, d, shallow, ta, tb);
      ab_offset(fmt("star #%d inward", i), p, -d, shallow, ta, tb);
      ab_offset(fmt("convex #%d outward", i), c, d, true, ta, tb);
      ab_offset(fmt("convex #%d inward", i), c, -d, true, ta, tb);
    }
  }
  report_time("clipper2 offsets (32 cases)", "cgal", ta, "clipper2", tb);

  ta = tb = 0;
  for (int i = 0; i < 4; ++i) {
    const ipath c1 = convex(r, 0, 0, 1e5, 8 + 4 * i), c2 = convex(r, 0, 0, 1e6, 10 + 2 * i);
    ab_minkowski(fmt("convex+convex #%d", i), c1, c2, true, ta, tb);
    const ipath s = star(r, 0, 0, 1e6, 24 + 8 * i);
    ab_minkowski(fmt("convex+star #%d", i), c1, s, false, ta, tb);
  }
  report_time("clipper2 minkowski (8 cases)", "cgal", ta, "clipper2", tb);
}

} // namespace

void ab_permissive(rng &r) {
  ab_predicates(r);
  ab_delaunay3_all(r);
  ab_cdt_all(r);
  ab_polygons(r);
}

} // namespace cgal_ab
