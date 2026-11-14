// cgal_ab.cpp -- the A/B check of libcvc's permissive geometry against the CGAL code it replaced.
//
// libcvc does not use CGAL: its algorithm packages are GPL-3+ (transfix/libcvc#1080, and the
// VolumeRover ports plan, docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md). This program runs each
// replacement beside the CGAL code it replaced, on the same inputs, and compares results and
// times. It links CGAL, so as built it is a GPL-3+ combined work: it is never installed or
// packaged, and it builds only with -DCVC_BUILD_CGAL_AB=ON (ci.yml's cgal-ab job). See README.md.
//
//   A: CGAL                                     B: libcvc
//   Kd_tree + Fuzzy_sphere (epsilon 0)          nav::neighbors_within_radius (nanoflann)
//   AABB_tree::closest_point (exact)            project_points_to_triangles (nanoflann + exact)
//   the VolumeRover ports' rows                 cgal_ab_permissive.cpp (Qhull, Clipper2, CDT,
//                                               Shewchuk's predicates)
//
// Prints one PASS/FAIL line per check, TIME lines for both sides, and exits non-zero on any FAIL.

#include "cgal_ab.h"

#include <CGAL/AABB_traits_3.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_triangle_primitive_3.h>
#include <CGAL/Fuzzy_sphere.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Search_traits_2.h>
#include <CGAL/Search_traits_adapter.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/property_map.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cvc/geometry/project_verts.h>
#include <cvc/nav/grid_nav.h>
#include <functional>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

namespace cgal_ab {

namespace {
int g_failures = 0;
}

void report(bool ok, const std::string &name, const std::string &detail) {
  std::printf("%s %s%s%s\n", ok ? "PASS" : "FAIL", name.c_str(), detail.empty() ? "" : ": ",
              detail.c_str());
  std::fflush(stdout);
  if (!ok)
    ++g_failures;
}

double ms_of(const std::function<void()> &f) {
  const auto t0 = std::chrono::steady_clock::now();
  f();
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

void report_time(const std::string &name, const char *a, double ta, const char *b, double tb) {
  std::printf("TIME %s: %s %.2f ms, %s %.2f ms (%.2fx)\n", name.c_str(), a, ta, b, tb,
              tb > 0 ? ta / tb : 0.0);
  std::fflush(stdout);
}

} // namespace cgal_ab

namespace {

using cgal_ab::ms_of;
using cgal_ab::report;
using cgal_ab::rng;

// --- A: CGAL's fixed-radius search, exactly as nav/spatial.cpp had it before #1080 -------------

using K = CGAL::Simple_cartesian<double>;
using point_idx = std::pair<K::Point_2, int>;
using search_traits =
    CGAL::Search_traits_adapter<point_idx, CGAL::First_of_pair_property_map<point_idx>,
                                CGAL::Search_traits_2<K>>;

cvc::nav::neighbor_csr cgal_neighbors(const std::vector<double> &pos, double radius) {
  const int n = static_cast<int>(pos.size() / 2);
  cvc::nav::neighbor_csr out;
  out.offsets.assign(n + 1, 0);
  if (n == 0)
    return out;
  CGAL::Kd_tree<search_traits> tree;
  for (int i = 0; i < n; ++i)
    tree.insert(point_idx(K::Point_2(pos[2 * i], pos[2 * i + 1]), i));
  tree.build();
  std::vector<std::vector<int>> per(n);
  std::vector<point_idx> hits;
  for (int i = 0; i < n; ++i) {
    hits.clear();
    tree.search(std::back_inserter(hits), CGAL::Fuzzy_sphere<search_traits>(
                                              K::Point_2(pos[2 * i], pos[2 * i + 1]), radius, 0.0));
    for (const point_idx &h : hits)
      if (h.second != i)
        per[i].push_back(h.second);
    std::sort(per[i].begin(), per[i].end());
  }
  for (int i = 0; i < n; ++i)
    out.offsets[i + 1] = out.offsets[i] + static_cast<int>(per[i].size());
  for (int i = 0; i < n; ++i)
    out.indices.insert(out.indices.end(), per[i].begin(), per[i].end());
  return out;
}

void ab_neighbors(const std::string &name, const std::vector<double> &pos, double radius) {
  cvc::nav::neighbor_csr a, b;
  const double ta = ms_of([&] { a = cgal_neighbors(pos, radius); });
  const double tb = ms_of(
      [&] { b = cvc::nav::neighbors_within_radius(pos.data(), int(pos.size() / 2), radius); });
  const bool same = a.offsets == b.offsets && a.indices == b.indices;
  std::size_t first_diff = 0;
  if (!same) {
    const int n = int(pos.size() / 2);
    for (int i = 0; i < n; ++i) {
      const std::vector<int> ai(a.indices.begin() + a.offsets[i],
                                a.indices.begin() + a.offsets[i + 1]);
      const std::vector<int> bi(b.indices.begin() + b.offsets[i],
                                b.indices.begin() + b.offsets[i + 1]);
      if (ai != bi) {
        first_diff = std::size_t(i);
        break;
      }
    }
  }
  report(same, "neighbors " + name,
         same ? std::to_string(a.indices.size()) + " pairs identical"
              : "first difference at point " + std::to_string(first_diff));
  std::printf("TIME neighbors %s: cgal %.2f ms, nanoflann %.2f ms (%.2fx)\n", name.c_str(), ta, tb,
              tb > 0 ? ta / tb : 0.0);
}

// --- A: CGAL's exact closest point on a triangle soup ------------------------------------------

using tri_vec = std::vector<K::Triangle_3>;
using primitive = CGAL::AABB_triangle_primitive_3<K, tri_vec::const_iterator>;
using aabb_tree = CGAL::AABB_tree<CGAL::AABB_traits_3<K, primitive>>;

using v3 = std::array<double, 3>;
using t3 = std::array<unsigned int, 3>;

double d2(const v3 &a, const v3 &b) {
  const double x = a[0] - b[0], y = a[1] - b[1], z = a[2] - b[2];
  return x * x + y * y + z * z;
}

// A UV sphere of radius 1: nearly uniform triangles away from the poles.
void uv_sphere(int rings, int sectors, std::vector<v3> &v, std::vector<t3> &t) {
  const double pi = 3.14159265358979323846;
  v.clear();
  t.clear();
  v.push_back({0, 0, 1});
  for (int r = 1; r < rings; ++r)
    for (int s = 0; s < sectors; ++s) {
      const double th = pi * r / rings, ph = 2 * pi * s / sectors;
      v.push_back({std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph), std::cos(th)});
    }
  v.push_back({0, 0, -1});
  const unsigned south = unsigned(v.size() - 1);
  auto ring = [&](int r, int s) { return unsigned(1 + (r - 1) * sectors + (s % sectors)); };
  for (int s = 0; s < sectors; ++s)
    t.push_back({0, ring(1, s), ring(1, s + 1)});
  for (int r = 1; r < rings - 1; ++r)
    for (int s = 0; s < sectors; ++s) {
      t.push_back({ring(r, s), ring(r + 1, s), ring(r + 1, s + 1)});
      t.push_back({ring(r, s), ring(r + 1, s + 1), ring(r, s + 1)});
    }
  for (int s = 0; s < sectors; ++s)
    t.push_back({ring(rings - 1, s), south, ring(rings - 1, s + 1)});
}

// A strip of long, thin triangles: where the nearest-vertex heuristic is weakest.
void sliver_strip(int n, std::vector<v3> &v, std::vector<t3> &t) {
  v.clear();
  t.clear();
  for (int i = 0; i <= n; ++i) {
    v.push_back({double(i), 0, 0});
    v.push_back({double(i), 0.05, 0});
  }
  for (int i = 0; i < n; ++i) {
    const unsigned a = 2 * i, b = 2 * i + 1, c = 2 * i + 2, d = 2 * i + 3;
    t.push_back({a, c, d});
    t.push_back({a, d, b});
  }
}

void ab_projection(const std::string &name, const std::vector<v3> &verts,
                   const std::vector<t3> &tris, const std::vector<v3> &queries, std::size_t k,
                   bool must_be_exact) {
  tri_vec soup;
  for (const t3 &tr : tris)
    soup.emplace_back(K::Point_3(verts[tr[0]][0], verts[tr[0]][1], verts[tr[0]][2]),
                      K::Point_3(verts[tr[1]][0], verts[tr[1]][1], verts[tr[1]][2]),
                      K::Point_3(verts[tr[2]][0], verts[tr[2]][1], verts[tr[2]][2]));
  std::vector<v3> exact(queries.size());
  const double ta = ms_of([&] {
    aabb_tree tree(soup.cbegin(), soup.cend());
    tree.accelerate_distance_queries();
    for (std::size_t i = 0; i < queries.size(); ++i) {
      const K::Point_3 c =
          tree.closest_point(K::Point_3(queries[i][0], queries[i][1], queries[i][2]));
      exact[i] = {c.x(), c.y(), c.z()};
    }
  });
  std::vector<v3> ours = queries;
  const double tb = ms_of([&] { cvc::project_points_to_triangles(ours, verts, tris, k); });

  // Distances, not points: a query equidistant from two triangles may go to either.
  double max_excess = 0, sum_excess = 0;
  std::size_t exact_count = 0, closer = 0;
  for (std::size_t i = 0; i < queries.size(); ++i) {
    const double de = std::sqrt(d2(queries[i], exact[i]));
    const double dc = std::sqrt(d2(queries[i], ours[i]));
    const double excess = dc - de;
    if (excess < -1e-9)
      ++closer; // nothing may beat the exact closest point
    if (excess <= 1e-9)
      ++exact_count;
    max_excess = std::max(max_excess, excess);
    sum_excess += std::max(excess, 0.0);
  }
  char detail[256];
  std::snprintf(detail, sizeof detail,
                "k=%zu, %zu/%zu exact, max excess %.3g, mean excess %.3g, %zu closer than exact", k,
                exact_count, queries.size(), max_excess, sum_excess / queries.size(), closer);
  const bool ok = closer == 0 && (!must_be_exact || exact_count == queries.size());
  report(ok, "projection " + name, detail);
  std::printf("TIME projection %s: cgal %.2f ms, cvc %.2f ms (%.2fx)\n", name.c_str(), ta, tb,
              tb > 0 ? ta / tb : 0.0);
}

} // namespace

int main() {
  rng r{12345u};

  // Fixed-radius neighbours: identical results required.
  for (int n : {1000, 20000}) {
    std::vector<double> pos(2 * n);
    for (double &x : pos)
      x = r.uniform(0, 100);
    for (double radius : {0.5, 2.0, 5.0})
      ab_neighbors("uniform n=" + std::to_string(n) + " r=" + std::to_string(radius), pos, radius);
  }
  {
    std::vector<double> lattice; // exact boundary distances (3-4-5, axis, diagonal)
    for (int y = 0; y <= 40; ++y)
      for (int x = 0; x <= 40; ++x) {
        lattice.push_back(x);
        lattice.push_back(y);
      }
    for (double radius : {1.0, std::sqrt(2.0), 5.0})
      ab_neighbors("lattice r=" + std::to_string(radius), lattice, radius);
  }
  {
    std::vector<double> cluster; // a dense clump with duplicates
    for (int i = 0; i < 3000; ++i) {
      cluster.push_back(i % 7 == 0 ? 1.0 : r.uniform(0.9, 1.1));
      cluster.push_back(i % 7 == 0 ? 1.0 : r.uniform(0.9, 1.1));
    }
    for (double radius : {0.0, 0.01, 0.1})
      ab_neighbors("cluster r=" + std::to_string(radius), cluster, radius);
  }

  // Projection onto triangles: exact when every triangle is a candidate; otherwise report how far
  // the nearest-vertex heuristic is from CGAL's exact closest point.
  std::vector<v3> sv;
  std::vector<t3> st;
  uv_sphere(40, 80, sv, st);
  std::vector<v3> q(3000);
  for (v3 &p : q)
    p = {r.uniform(-1.5, 1.5), r.uniform(-1.5, 1.5), r.uniform(-1.5, 1.5)};
  ab_projection("sphere, every triangle a candidate", sv, st, q, sv.size(), true);
  ab_projection("sphere, k=10", sv, st, q, 10, false);
  std::vector<v3> near(3000);
  for (std::size_t i = 0; i < near.size(); ++i) { // within 5% of the surface
    const v3 &p = sv[i % sv.size()];
    const double s = r.uniform(0.95, 1.05);
    near[i] = {p[0] * s + r.uniform(-0.01, 0.01), p[1] * s + r.uniform(-0.01, 0.01),
               p[2] * s + r.uniform(-0.01, 0.01)};
  }
  ab_projection("sphere, near-surface points, k=10", sv, st, near, 10, false);
  std::vector<v3> lv;
  std::vector<t3> lt;
  sliver_strip(200, lv, lt);
  std::vector<v3> lq(2000);
  for (v3 &p : lq)
    p = {r.uniform(-1, 201), r.uniform(-0.5, 0.55), r.uniform(-0.5, 0.5)};
  ab_projection("sliver strip, k=10", lv, lt, lq, 10, false);

  // The VolumeRover ports' permissive libraries (row T16 of their plan's test matrix).
  cgal_ab::ab_permissive(r);
  // cvc::planar, the ContourTiler port's planar geometry, against ContourTiler's CGAL calls.
  cgal_ab::ab_planar(r);

  std::printf("%s: %d failure(s)\n", cgal_ab::g_failures ? "FAILED" : "ok", cgal_ab::g_failures);
  return cgal_ab::g_failures ? 1 : 0;
}
