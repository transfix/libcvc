/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The contour tiler port (cvc::tile_contours, inc/cvc/geometry/tiler.h;
// VOLROVER-LIBRARY-PORTS-ROADMAP.md §3, §1.7): legacy parity (T1, against
// tools/volrover-goldens' runs of the legacy ContourTiler), known shapes (T3), invariants (T4),
// bad input (T5), no singletons (T6), resumable (T7) and the parameter table (T8). The legacy
// library's own unit tests, converted, are tiler_legacy_test.cpp (T2).
//
// T1 is the settled parity (§9 item 2, 2026-11-17): away from untiled regions the surfaces equal
// the legacy's, triangle for triangle; untiled regions (filled from a different convex partition)
// and contours that curation rebuilt (Clipper2's Booleans in place of CGAL's exact ones) are
// compared by topology and Hausdorff distance under a tolerance.

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/geometry/tiler.h>
#include <cvc/geometry/tiler_params.h>
#include <fstream>
#include <gtest/gtest.h>
#include <map>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "resumable_harness.h"

#ifndef CVC_REPO_DIR
#define CVC_REPO_DIR "."
#endif

namespace {

constexpr double pi = 3.14159265358979323846;

#ifndef CVC_WITH_PLANAR
#define SKIP_WITHOUT_PLANAR() GTEST_SKIP() << "libcvc was built without cvc::planar"
#else
#define SKIP_WITHOUT_PLANAR()                                                                      \
  do {                                                                                             \
  } while (0)
#endif

// A coordinate on a 2^-8 grid: exact in decimal, and unchanged by the tiler's snapping.
double q8(double v) { return std::round(v * 256) / 256; }

// A wobbly ellipse, counterclockwise, as tools/volrover-goldens/make_inputs.py writes them.
std::vector<double> blob(double cx, double cy, double a, double b, double wobble, double phase,
                         int n) {
  std::vector<double> xy;
  for (int i = 0; i < n; ++i) {
    const double t = 2 * pi * i / n;
    const double r = 1 + wobble * std::cos(3 * t + phase);
    const double x = q8(cx + a * r * std::cos(t)), y = q8(cy + b * r * std::sin(t));
    if (!xy.empty() && xy[xy.size() - 2] == x && xy.back() == y)
      continue;
    xy.push_back(x);
    xy.push_back(y);
  }
  return xy;
}

// ---- meshes ---------------------------------------------------------------------------------

using P3 = std::array<double, 3>;
using Tri = std::array<P3, 3>;

struct mesh {
  std::vector<P3> v;
  std::vector<std::array<std::size_t, 3>> t;
};

mesh read_tiles(const std::string &path) {
  mesh m;
  std::ifstream in(path);
  std::size_t nv = 0, nt = 0;
  in >> nv >> nt;
  m.v.resize(nv);
  for (auto &p : m.v)
    in >> p[0] >> p[1] >> p[2];
  m.t.resize(nt);
  for (auto &t : m.t)
    in >> t[0] >> t[1] >> t[2];
  return m;
}

mesh from_geometry(const cvc::geometry &g) {
  mesh m;
  for (const auto &p : g.points())
    m.v.push_back({p[0], p[1], p[2]});
  for (const auto &t : g.tris())
    m.t.push_back({std::size_t(t[0]), std::size_t(t[1]), std::size_t(t[2])});
  return m;
}

bool on_slice(const P3 &p) { return p[2] == std::floor(p[2]); }

// Triangles as their sorted corners: those of contour vertices only (on slice planes), or all.
std::multiset<Tri> triangle_set(const mesh &m, bool contour_only) {
  std::multiset<Tri> out;
  for (const auto &t : m.t) {
    Tri tr{m.v[t[0]], m.v[t[1]], m.v[t[2]]};
    if (contour_only && (!on_slice(tr[0]) || !on_slice(tr[1]) || !on_slice(tr[2])))
      continue;
    std::sort(tr.begin(), tr.end());
    out.insert(tr);
  }
  return out;
}

struct topo {
  long euler = 0;
  std::size_t boundary_edges = 0, boundary_loops = 0, nonmanifold_edges = 0, components = 0;
  bool operator==(const topo &o) const {
    return euler == o.euler && boundary_loops == o.boundary_loops &&
           nonmanifold_edges == o.nonmanifold_edges && components == o.components;
  }
};

std::ostream &operator<<(std::ostream &os, const topo &t) {
  return os << "{euler " << t.euler << ", boundary loops " << t.boundary_loops << " ("
            << t.boundary_edges << " edges), non-manifold edges " << t.nonmanifold_edges
            << ", components " << t.components << "}";
}

// The surface's topology, its vertices merged by position.
topo topology(const mesh &m) {
  std::map<P3, std::size_t> id;
  std::vector<std::size_t> vid(m.v.size());
  for (std::size_t i = 0; i < m.v.size(); ++i)
    vid[i] = id.emplace(m.v[i], id.size()).first->second;
  std::vector<std::size_t> parent(id.size());
  for (std::size_t i = 0; i < parent.size(); ++i)
    parent[i] = i;
  auto find = [&](std::size_t a) {
    while (parent[a] != a)
      a = parent[a] = parent[parent[a]];
    return a;
  };
  std::map<std::pair<std::size_t, std::size_t>, int> edges;
  for (const auto &t : m.t)
    for (int i = 0; i < 3; ++i) {
      const std::size_t a = vid[t[i]], b = vid[t[(i + 1) % 3]];
      parent[find(a)] = find(b);
      edges[{std::min(a, b), std::max(a, b)}]++;
    }
  topo r;
  r.euler = long(id.size()) - long(edges.size()) + long(m.t.size());
  std::vector<std::size_t> bparent(id.size());
  for (std::size_t i = 0; i < bparent.size(); ++i)
    bparent[i] = i;
  auto bfind = [&](std::size_t a) {
    while (bparent[a] != a)
      a = bparent[a] = bparent[bparent[a]];
    return a;
  };
  std::set<std::size_t> bverts;
  for (const auto &e : edges) {
    if (e.second == 1) {
      ++r.boundary_edges;
      bparent[bfind(e.first.first)] = bfind(e.first.second);
      bverts.insert(e.first.first);
      bverts.insert(e.first.second);
    }
    if (e.second > 2)
      ++r.nonmanifold_edges;
  }
  std::set<std::size_t> broots, roots;
  for (std::size_t v : bverts)
    broots.insert(bfind(v));
  r.boundary_loops = broots.size();
  for (const auto &t : m.t)
    roots.insert(find(vid[t[0]]));
  r.components = roots.size();
  return r;
}

P3 sub(const P3 &u, const P3 &v) { return {u[0] - v[0], u[1] - v[1], u[2] - v[2]}; }
double dot(const P3 &u, const P3 &v) { return u[0] * v[0] + u[1] * v[1] + u[2] * v[2]; }
P3 cross(const P3 &u, const P3 &v) {
  return {u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]};
}

// The distance from p to the triangle abc (Ericson's closest point).
double point_triangle(const P3 &p, const P3 &a, const P3 &b, const P3 &c) {
  auto at = [](const P3 &o, const P3 &u, double s, const P3 &v, double t) {
    return P3{o[0] + s * u[0] + t * v[0], o[1] + s * u[1] + t * v[1], o[2] + s * u[2] + t * v[2]};
  };
  const P3 ab = sub(b, a), ac = sub(c, a), ap = sub(p, a);
  const double d1 = dot(ab, ap), d2 = dot(ac, ap);
  P3 q;
  if (d1 <= 0 && d2 <= 0) {
    q = a;
  } else {
    const P3 bp = sub(p, b), cp = sub(p, c);
    const double d3 = dot(ab, bp), d4 = dot(ac, bp), d5 = dot(ab, cp), d6 = dot(ac, cp);
    const double vc = d1 * d4 - d3 * d2, vb = d5 * d2 - d1 * d6, va = d3 * d6 - d5 * d4;
    if (d3 >= 0 && d4 <= d3)
      q = b;
    else if (vc <= 0 && d1 >= 0 && d3 <= 0)
      q = at(a, ab, d1 / (d1 - d3), ac, 0);
    else if (d6 >= 0 && d5 <= d6)
      q = c;
    else if (vb <= 0 && d2 >= 0 && d6 <= 0)
      q = at(a, ab, 0, ac, d2 / (d2 - d6));
    else if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
      q = at(b, sub(c, b), (d4 - d3) / ((d4 - d3) + (d5 - d6)), ac, 0);
    else {
      const double den = 1 / (va + vb + vc);
      q = at(a, ab, vb * den, ac, vc * den);
    }
  }
  const P3 d = sub(p, q);
  return std::sqrt(dot(d, d));
}

// The largest distance from a's vertices and triangle centroids to b.
double one_sided(const mesh &a, const mesh &b) {
  std::vector<P3> samples = a.v;
  for (const auto &t : a.t)
    samples.push_back({(a.v[t[0]][0] + a.v[t[1]][0] + a.v[t[2]][0]) / 3,
                       (a.v[t[0]][1] + a.v[t[1]][1] + a.v[t[2]][1]) / 3,
                       (a.v[t[0]][2] + a.v[t[1]][2] + a.v[t[2]][2]) / 3});
  double worst = 0;
  for (const P3 &p : samples) {
    double best = HUGE_VAL;
    for (const auto &t : b.t)
      best = std::min(best, point_triangle(p, b.v[t[0]], b.v[t[1]], b.v[t[2]]));
    worst = std::max(worst, best);
  }
  return worst;
}

double hausdorff(const mesh &a, const mesh &b) { return std::max(one_sided(a, b), one_sided(b, a)); }

// The signed volume a closed, consistently oriented surface encloses.
double volume(const mesh &m) {
  double v = 0;
  for (const auto &t : m.t)
    v += dot(m.v[t[0]], cross(m.v[t[1]], m.v[t[2]])) / 6;
  return v;
}

// Do the triangles share an interior point? For triangles with no common vertex: an edge of one
// crosses the other's interior (Moller's test without the coplanar case, which tiling never makes
// between components).
bool triangles_cross(const Tri &a, const Tri &b) {
  auto edge_hits = [](const P3 &p, const P3 &q, const Tri &t) {
    const P3 n = cross(sub(t[1], t[0]), sub(t[2], t[0]));
    const double dp = dot(n, sub(p, t[0])), dq = dot(n, sub(q, t[0]));
    if ((dp > 0 && dq > 0) || (dp < 0 && dq < 0) || dp == dq)
      return false;
    const double s = dp / (dp - dq);
    const P3 x{p[0] + s * (q[0] - p[0]), p[1] + s * (q[1] - p[1]), p[2] + s * (q[2] - p[2])};
    for (int i = 0; i < 3; ++i)
      if (dot(n, cross(sub(t[(i + 1) % 3], t[i]), sub(x, t[i]))) <= 0)
        return false;
    return true;
  };
  for (int i = 0; i < 3; ++i)
    if (edge_hits(a[i], a[(i + 1) % 3], b) || edge_hits(b[i], b[(i + 1) % 3], a))
      return true;
  return false;
}

// Every pair of triangles of `m` that share no vertex but cross: none, for a surface that does not
// intersect itself.
std::size_t self_crossings(const mesh &m) {
  std::size_t n = 0;
  for (std::size_t i = 0; i < m.t.size(); ++i)
    for (std::size_t j = i + 1; j < m.t.size(); ++j) {
      bool shares = false;
      for (int a = 0; a < 3 && !shares; ++a)
        for (int b = 0; b < 3 && !shares; ++b)
          shares = m.v[m.t[i][a]] == m.v[m.t[j][b]];
      if (shares)
        continue;
      const Tri ta{m.v[m.t[i][0]], m.v[m.t[i][1]], m.v[m.t[i][2]]};
      const Tri tb{m.v[m.t[j][0]], m.v[m.t[j][1]], m.v[m.t[j][2]]};
      n += triangles_cross(ta, tb);
    }
  return n;
}

bool same_geometry(const cvc::geometry &a, const cvc::geometry &b) {
  return a.points() == b.points() && a.tris() == b.tris() && a.normals() == b.normals() &&
         a.colors() == b.colors();
}

bool same(const cvc::tile_result &a, const cvc::tile_result &b) {
  if (a.components.size() != b.components.size() || a.warnings != b.warnings)
    return false;
  for (const auto &c : a.components) {
    auto it = b.components.find(c.first);
    if (it == b.components.end() || !same_geometry(c.second, it->second))
      return false;
  }
  return true;
}

// ---- the legacy runs ------------------------------------------------------------------------

struct legacy_case {
  std::string name;
  int first, last;
  cvc::tile_params params;
};

// A case as tools/volrover-goldens ran it: its sections and parameters (inputs/tiler/<case>.json),
// without caps (the legacy closed a component only where the section range went past it).
legacy_case load_case(const std::string &name, int first, int last, double delta, bool ri) {
  legacy_case c{name, first, last, {}};
  c.params.curation_delta = delta;
  c.params.collinear_epsilon = 1e-15;
  c.params.remove_intersections = ri;
  c.params.z_first = first;
  c.params.z_last = last;
  c.params.cap_ends = false;
  return c;
}

cvc::contour_stack read_case(cvc::app &app, const legacy_case &c) {
  cvc::contour_read_params rp;
  rp.first = c.first;
  rp.last = c.last;
  return cvc::read_contours(app,
                            std::string(CVC_REPO_DIR) + "/tools/volrover-goldens/inputs/tiler/" +
                                c.name + "/" + c.name + ".ser",
                            rp);
}

mesh golden(const std::string &c, const std::string &component) {
  return read_tiles(std::string(CVC_REPO_DIR) + "/src/cvc/tests/data/volrover-goldens/tiler/" + c +
                    "." + component + ".tiles.txt");
}

legacy_case tube_case() { return load_case("tube", 1, 6, 0.01, false); }
legacy_case branch_case() { return load_case("branch", 0, 7, 0.01, false); }
legacy_case touching_case() { return load_case("touching", 1, 5, 0.05, true); }

} // namespace

// ---- T1: legacy parity ----------------------------------------------------------------------

// A tube tiled between contour vertices only: the legacy's surface, triangle for triangle.
TEST(TilerLegacy, TheTubeIsTheLegacySurface) {
  SKIP_WITHOUT_PLANAR();
  cvc::app app;
  const legacy_case c = tube_case();
  const cvc::tile_result r = cvc::tile_contours(read_case(app, c), c.params);
  EXPECT_TRUE(r.warnings.empty());
  ASSERT_EQ(r.components.count("a001"), 1u);
  const mesh legacy = golden(c.name, "a001"), port = from_geometry(r.components.at("a001"));
  EXPECT_EQ(port.v.size(), legacy.v.size());
  EXPECT_EQ(port.t.size(), legacy.t.size());
  EXPECT_TRUE(triangle_set(port, false) == triangle_set(legacy, false));
  EXPECT_EQ(topology(port), topology(legacy));
}

// A branching component and an annulus, closed by caps: every tile between contour vertices is the
// legacy's; the untiled regions (caps, the branch) are filled from a different convex partition,
// so they are compared by topology and distance. The tolerance is a quarter of the slice spacing
// (measured: 0.13 of it, from the fill vertices' heights).
TEST(TilerLegacy, BranchingMatchesAwayFromUntiledRegions) {
  SKIP_WITHOUT_PLANAR();
  cvc::app app;
  const legacy_case c = branch_case();
  const cvc::tile_result r = cvc::tile_contours(read_case(app, c), c.params);
  for (const char *comp : {"d001", "r001"}) {
    SCOPED_TRACE(comp);
    ASSERT_EQ(r.components.count(comp), 1u);
    const mesh legacy = golden(c.name, comp), port = from_geometry(r.components.at(comp));
    EXPECT_TRUE(triangle_set(port, true) == triangle_set(legacy, true));
    EXPECT_EQ(topology(port), topology(legacy));
    EXPECT_LT(hausdorff(port, legacy), 0.25);
  }
  // The legacy's own failure on the annulus (an untiled region it cannot trace) is the port's.
  ASSERT_FALSE(r.warnings.empty());
  for (const std::string &w : r.warnings)
    EXPECT_NE(w.find("No chord was legal"), std::string::npos) << w;
}

// Two components whose traces overlap: curation pushes them apart (dilate, subtract, erode) with
// Clipper2 where the legacy used CGAL's exact Booleans, so the curated contours differ near the
// overlap. Compared by topology and distance: within twice the curation distance (measured: 1.03
// times it).
TEST(TilerLegacy, CurationMatchesWithinItsDistance) {
  SKIP_WITHOUT_PLANAR();
  cvc::app app;
  const legacy_case c = touching_case();
  const cvc::tile_result r = cvc::tile_contours(read_case(app, c), c.params);
  for (const char *comp : {"a002", "a003"}) {
    SCOPED_TRACE(comp);
    ASSERT_EQ(r.components.count(comp), 1u);
    const mesh legacy = golden(c.name, comp), port = from_geometry(r.components.at(comp));
    EXPECT_EQ(topology(port), topology(legacy));
    EXPECT_LT(hausdorff(port, legacy), 2 * c.params.curation_delta);
  }
}

// ---- T3: known shapes -----------------------------------------------------------------------

namespace {

cvc::contour_stack cylinder(int slices) {
  cvc::contour_stack s;
  for (int k = 0; k < slices; ++k)
    s.add(k, "tube", blob(8 + 0.125 * k, 8, 3 + 0.125 * (k % 2), 2.5, 0.05, 0.4 * k, 24));
  return s;
}

cvc::contour_stack branching_y() {
  cvc::contour_stack s;
  for (int k = 0; k < 3; ++k)
    s.add(k, "y", blob(6, 6, 3.5 - 0.25 * k, 2, 0.05, 0.4 * k, 28));
  for (int k = 3; k < 6; ++k) {
    s.add(k, "y", blob(4.25, 6, 1.5, 1.5, 0.06, 0.5 * k, 20));
    s.add(k, "y", blob(7.875, 6.25, 1.5, 1.25, 0.06, 0.7 * k, 20));
  }
  return s;
}

} // namespace

TEST(Tiler, ACylinderIsAClosedGenusZeroTube) {
  SKIP_WITHOUT_PLANAR();
  const cvc::tile_result r = cvc::tile_contours(cylinder(5));
  EXPECT_TRUE(r.warnings.empty());
  ASSERT_EQ(r.components.size(), 1u);
  const topo t = topology(from_geometry(r.components.at("tube")));
  EXPECT_EQ(t.euler, 2) << t; // a sphere: closed, genus 0
  EXPECT_EQ(t.boundary_edges, 0u) << t;
  EXPECT_EQ(t.nonmanifold_edges, 0u) << t;
  EXPECT_EQ(t.components, 1u) << t;
  // Without caps it is an open tube: genus 0 with two boundary loops.
  cvc::tile_params p;
  p.cap_ends = false;
  const topo open = topology(from_geometry(cvc::tile_contours(cylinder(5), p).components.at("tube")));
  EXPECT_EQ(open.euler, 0) << open;
  EXPECT_EQ(open.boundary_loops, 2u) << open;
}

TEST(Tiler, ABranchingYIsOneSurface) {
  SKIP_WITHOUT_PLANAR();
  const cvc::tile_result r = cvc::tile_contours(branching_y());
  ASSERT_EQ(r.components.size(), 1u);
  const topo t = topology(from_geometry(r.components.at("y")));
  EXPECT_EQ(t.components, 1u) << t;
  EXPECT_EQ(t.boundary_edges, 0u) << t;
  EXPECT_EQ(t.nonmanifold_edges, 0u) << t;
  EXPECT_EQ(t.euler, 2) << t; // a Y with closed ends is still a sphere
}

TEST(Tiler, TouchingComponentsGiveDisjointSurfaces) {
  SKIP_WITHOUT_PLANAR();
  cvc::contour_stack s;
  for (int k = 0; k < 5; ++k) {
    s.add(k, "a", blob(6, 6, 2.5, 2.25, 0.05, 0.3 * k, 26));
    s.add(k, "b", blob(9.75 - 0.25 * k, 6.5, 2, 2.25, 0.05, 0.6 * k, 26));
  }
  cvc::tile_params p;
  p.curation_delta = 0.05;
  p.remove_intersections = true;
  const cvc::tile_result r = cvc::tile_contours(s, p);
  ASSERT_EQ(r.components.size(), 2u);
  const mesh a = from_geometry(r.components.at("a")), b = from_geometry(r.components.at("b"));
  std::size_t crossings = 0;
  for (const auto &ta : a.t)
    for (const auto &tb : b.t)
      crossings += triangles_cross({a.v[ta[0]], a.v[ta[1]], a.v[ta[2]]},
                                   {b.v[tb[0]], b.v[tb[1]], b.v[tb[2]]});
  EXPECT_EQ(crossings, 0u);
  EXPECT_EQ(topology(a).components, 1u);
  EXPECT_EQ(topology(b).components, 1u);
}

// ---- T4: invariants -------------------------------------------------------------------------

namespace {

// A seeded stack: one component, a blob per slice drifting and changing shape.
cvc::contour_stack random_stack(unsigned seed, int slices, double spacing) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> u(0, 1);
  cvc::contour_stack s;
  s.z_spacing = spacing;
  double cx = 8, cy = 8;
  for (int k = 0; k < slices; ++k) {
    cx += 0.4 * (u(rng) - 0.5);
    cy += 0.4 * (u(rng) - 0.5);
    s.add(k, "c",
          blob(cx, cy, 2.5 + u(rng), 2 + u(rng), 0.08 * u(rng), 2 * pi * u(rng),
               16 + int(16 * u(rng))));
  }
  return s;
}

} // namespace

TEST(Tiler, SurfacesAreClosedManifoldAndOnTheirPlanes) {
  SKIP_WITHOUT_PLANAR();
  for (unsigned seed = 1; seed <= 6; ++seed) {
    SCOPED_TRACE(seed);
    const double spacing = 0.5 + 0.25 * seed;
    const cvc::contour_stack s = random_stack(seed, 4 + int(seed % 3), spacing);
    const cvc::tile_result r = cvc::tile_contours(s);
    ASSERT_EQ(r.components.size(), 1u);
    const mesh m = from_geometry(r.components.at("c"));
    const topo t = topology(m);
    EXPECT_EQ(t.boundary_edges, 0u) << t;
    EXPECT_EQ(t.nonmanifold_edges, 0u) << t;
    EXPECT_EQ(t.components, 1u) << t;
    EXPECT_EQ(t.euler, 2) << t;
    EXPECT_EQ(self_crossings(m), 0u);
    // Every vertex is on a slice's plane, or between two (an untiled region's fill); the caps
    // reach half a spacing past the end slices.
    const double zlo = s.slice_z(s.slices.begin()->first) - spacing;
    const double zhi = s.slice_z(s.slices.rbegin()->first) + spacing;
    for (const P3 &v : m.v) {
      EXPECT_GE(v[2], zlo);
      EXPECT_LE(v[2], zhi);
    }
    // Enclosed volume: positive (outward triangles) and, the caps' share apart, the prisms'.
    EXPECT_GT(volume(m), 0);
  }
}

TEST(Tiler, AnOpenTubeEnclosesThePrismVolume) {
  SKIP_WITHOUT_PLANAR();
  for (unsigned seed = 1; seed <= 4; ++seed) {
    SCOPED_TRACE(seed);
    const cvc::contour_stack s = random_stack(seed, 6, 1.0);
    cvc::tile_params p;
    p.cap_ends = false;
    mesh m = from_geometry(cvc::tile_contours(s, p).components.at("c"));
    // Close the tube with its end contours (fans), and compare with the prisms between slices.
    double prisms = 0;
    for (auto it = s.slices.begin(); std::next(it) != s.slices.end(); ++it) {
      const double a0 = std::fabs(it->second.components.at("c")[0].signed_area());
      const double a1 = std::fabs(std::next(it)->second.components.at("c")[0].signed_area());
      prisms += (a0 + a1) / 2 * (s.slice_z(std::next(it)->first) - s.slice_z(it->first));
    }
    const double tube_volume = volume(m);
    for (const auto &end : {*s.slices.begin(), *s.slices.rbegin()}) {
      const auto &xy = end.second.components.at("c")[0].xy;
      const double z = s.slice_z(end.first);
      const bool top = end.first == s.slices.rbegin()->first;
      const std::size_t n = xy.size() / 2;
      const std::size_t base = m.v.size();
      for (std::size_t i = 0; i < n; ++i)
        m.v.push_back({xy[2 * i], xy[2 * i + 1], z});
      for (std::size_t i = 1; i + 1 < n; ++i)
        m.t.push_back(top ? std::array<std::size_t, 3>{base, base + i, base + i + 1}
                          : std::array<std::size_t, 3>{base, base + i + 1, base + i});
    }
    const double closed = volume(m);
    EXPECT_NEAR(std::fabs(closed), prisms, 0.02 * prisms)
        << "tube " << tube_volume << " closed " << closed << " prisms " << prisms;
  }
}

// ---- T5: bad input --------------------------------------------------------------------------

TEST(Tiler, EmptyAndSingleSlices) {
  SKIP_WITHOUT_PLANAR();
  EXPECT_TRUE(cvc::tile_contours(cvc::contour_stack{}).components.empty());
  // One slice: capped above and below, a closed lens.
  cvc::contour_stack one;
  one.add(3, "c", blob(4, 4, 2, 1.5, 0.05, 0.2, 20));
  const cvc::tile_result r = cvc::tile_contours(one);
  ASSERT_EQ(r.components.size(), 1u);
  const topo t = topology(from_geometry(r.components.at("c")));
  EXPECT_EQ(t.boundary_edges, 0u) << t;
  EXPECT_EQ(t.euler, 2) << t;
  // Without caps there is nothing to tile.
  cvc::tile_params p;
  p.cap_ends = false;
  EXPECT_TRUE(cvc::tile_contours(one, p).components.empty());
  // A slice range with no slices in it.
  p = {};
  p.z_first = 10;
  p.z_last = 12;
  EXPECT_TRUE(cvc::tile_contours(one, p).components.empty());
}

TEST(Tiler, AGapInTheSlicesClosesBothSides) {
  SKIP_WITHOUT_PLANAR();
  cvc::contour_stack s;
  for (int k : {0, 1, 4, 5})
    s.add(k, "c", blob(5, 5, 2, 1.75, 0.05, 0.3 * k, 22));
  const topo t = topology(from_geometry(cvc::tile_contours(s).components.at("c")));
  EXPECT_EQ(t.components, 2u) << t;
  EXPECT_EQ(t.boundary_edges, 0u) << t;
  EXPECT_EQ(t.euler, 4) << t; // two spheres
}

TEST(Tiler, ASelfIntersectingContourIsSkippedWithAWarning) {
  SKIP_WITHOUT_PLANAR();
  cvc::contour_stack s = cylinder(3);
  s.add(1, "tube", {0, 0, 2, 2, 2, 0, 0, 2}); // a bow tie
  const cvc::tile_result r = cvc::tile_contours(s);
  ASSERT_FALSE(r.warnings.empty());
  bool found = false;
  for (const std::string &w : r.warnings)
    found = found || w.find("non-simple contour: slice = 1") != std::string::npos;
  EXPECT_TRUE(found);
  EXPECT_EQ(r.components.size(), 1u);
}

TEST(Tiler, OrientationDoesNotMatter) {
  SKIP_WITHOUT_PLANAR();
  const cvc::contour_stack ccw = cylinder(4);
  cvc::contour_stack cw = ccw;
  for (auto &slice : cw.slices)
    for (auto &poly : slice.second.components.at("tube")) {
      std::vector<double> rev;
      for (std::size_t i = poly.size(); i-- > 0;) {
        rev.push_back(poly.xy[2 * i]);
        rev.push_back(poly.xy[2 * i + 1]);
      }
      // Start at the same vertex, so the reversed ring is the same polygon.
      std::rotate(rev.begin(), rev.end() - 2, rev.end());
      poly.xy = rev;
    }
  EXPECT_TRUE(same(cvc::tile_contours(ccw), cvc::tile_contours(cw)));
}

TEST(Tiler, InvalidInputIsATilerError) {
  SKIP_WITHOUT_PLANAR();
  cvc::contour_stack bad = cylinder(3);
  bad.add(1, "tube", {0, 0, 1, 1}); // two vertices
  EXPECT_THROW(cvc::tile_contours(bad), cvc::tiler_error);
  cvc::contour_stack nan = cylinder(3);
  nan.slices.at(1).components.at("tube")[0].xy[0] = std::nan("");
  EXPECT_THROW(cvc::tile_contours(nan), cvc::tiler_error);
  cvc::tile_params p;
  p.curation_delta = 2; // the slices are 1 apart
  EXPECT_THROW(cvc::tile_contours(cylinder(3), p), cvc::tiler_error);
  p = {};
  p.include = {"("};
  EXPECT_THROW(cvc::tile_contours(cylinder(3), p), cvc::tiler_error);
  p = {};
  p.color = {1, 0};
  EXPECT_THROW(cvc::tile_contours(cylinder(3), p), cvc::tiler_error);
  p = {};
  p.remove_intersections = true;
  p.curation_delta = 0;
  EXPECT_THROW(cvc::tile_contours(cylinder(3), p), cvc::tiler_error);
  p = {};
  p.z_first = 5;
  p.z_last = 2;
  EXPECT_THROW(cvc::tile_contours(cylinder(3), p), cvc::tiler_error);
  try {
    cvc::tile_contours(cylinder(3), p);
  } catch (const cvc::tiler_error &e) {
    EXPECT_NE(std::string(e.what()).find("z_first"), std::string::npos) << e.what();
  }
}

// The snapping grid fits any extent, but the legacy algorithm keeps absolute tolerances (1e-5 to
// 1e-11 world units, as tiler.h says), so the scales tested are those it was written for, from a
// hundredth to a thousand times the slice spacing, near the origin and far from it.
TEST(Tiler, CoordinatesFarFromTheOriginOrSmall) {
  SKIP_WITHOUT_PLANAR();
  for (const double scale : {1e-2, 1.0, 1e3}) {
    for (const double offset : {0.0, 1e4}) {
      SCOPED_TRACE(std::to_string(scale) + " " + std::to_string(offset));
      cvc::contour_stack s;
      s.z_spacing = scale;
      for (int k = 0; k < 4; ++k) {
        std::vector<double> xy = blob(0, 0, 2 + 0.1 * k, 1.5, 0.05, 0.3 * k, 20);
        for (double &v : xy)
          v = offset + scale * v;
        s.add(k, "c", xy);
      }
      cvc::tile_params p;
      p.curation_delta = 1e-3 * scale;
      const cvc::tile_result r = cvc::tile_contours(s, p);
      ASSERT_EQ(r.components.size(), 1u);
      const topo t = topology(from_geometry(r.components.at("c")));
      EXPECT_EQ(t.boundary_edges, 0u) << t;
      EXPECT_EQ(t.euler, 2) << t;
    }
  }
}

// ---- T6: no singletons ----------------------------------------------------------------------

TEST(Tiler, ThreadsAndOrderDoNotMatter) {
  SKIP_WITHOUT_PLANAR();
  cvc::app a1, a2;
  const legacy_case c1 = branch_case(), c2 = touching_case();
  const cvc::contour_stack s1 = read_case(a1, c1), s2 = read_case(a2, c2);
  const cvc::tile_result alone1 = cvc::tile_contours(s1, c1.params);
  const cvc::tile_result alone2 = cvc::tile_contours(s2, c2.params);
  // B then A gives what A then B gave.
  const cvc::tile_result again2 = cvc::tile_contours(s2, c2.params);
  const cvc::tile_result again1 = cvc::tile_contours(s1, c1.params);
  EXPECT_TRUE(same(alone1, again1));
  EXPECT_TRUE(same(alone2, again2));
  // At once, on two threads, each with its own app's stack.
  std::optional<cvc::tile_result> r1, r2;
  std::thread t1([&] { r1.emplace(cvc::tile_contours(read_case(a1, c1), c1.params)); });
  std::thread t2([&] { r2.emplace(cvc::tile_contours(read_case(a2, c2), c2.params)); });
  t1.join();
  t2.join();
  ASSERT_TRUE(r1 && r2);
  EXPECT_TRUE(same(*r1, alone1));
  EXPECT_TRUE(same(*r2, alone2));
}

// ---- T7: resumable --------------------------------------------------------------------------

TEST(Tiler, StepperIsResumable) {
  SKIP_WITHOUT_PLANAR();
  cvc::app app;
  const legacy_case c = touching_case(); // curation, augmenting, tiling and intersection removal
  const cvc::contour_stack s = read_case(app, c);
  cvc::test::t7_subject<cvc::tile_stepper, cvc::tile_result> t;
  t.name = "tiler touching";
  t.make = [&] { return std::make_unique<cvc::tile_stepper>(s, c.params); };
  t.take = [](cvc::tile_stepper &st) { return st.take(); };
  t.equal = [](const cvc::tile_result &a, const cvc::tile_result &b) { return same(a, b); };
  t.blocking = [&] { return cvc::tile_contours(s, c.params); };
  const cvc::test::t7_report r = cvc::test::run_t7(t);
  for (const char *phase : {"snapping", "curating", "augmenting", "tiling", "assembling"})
    EXPECT_NE(std::find(r.phases.begin(), r.phases.end(), phase), r.phases.end()) << phase;
}

TEST(Tiler, TheCapsAndBranchesStepToo) {
  SKIP_WITHOUT_PLANAR();
  cvc::app app;
  const legacy_case c = branch_case();
  const cvc::contour_stack s = read_case(app, c);
  cvc::test::t7_subject<cvc::tile_stepper, cvc::tile_result> t;
  t.name = "tiler branch";
  t.make = [&] { return std::make_unique<cvc::tile_stepper>(s, c.params); };
  t.take = [](cvc::tile_stepper &st) { return st.take(); };
  t.equal = [](const cvc::tile_result &a, const cvc::tile_result &b) { return same(a, b); };
  t.blocking = [&] { return cvc::tile_contours(s, c.params); };
  cvc::test::run_t7(t);
}

TEST(Tiler, TheLabelNamesTheSlicesBeingTiled) {
  SKIP_WITHOUT_PLANAR();
  cvc::tile_stepper st(cylinder(4));
  std::set<std::string> labels;
  while (!st.step(1))
    if (std::string(st.phase()) == "tiling")
      labels.insert(st.label());
  EXPECT_TRUE(labels.count("slices 1 - 2")) << labels.size();
  EXPECT_TRUE(st.label().empty());
  EXPECT_NE(st.status().find("done: 1 surfaces"), std::string::npos) << st.status();
}

TEST(Tiler, ACancelledJobGivesNothing) {
  SKIP_WITHOUT_PLANAR();
  auto job = cvc::tile_contours_job(cylinder(5), {});
  job->step(10);
  EXPECT_FALSE(job->done());
  job->cancel();
  EXPECT_TRUE(job->step(10));
  EXPECT_TRUE(job->cancelled());
  EXPECT_FALSE(job->has_result());
  cvc::tile_params p;
  p.curation_delta = -1;
  auto bad = cvc::tile_contours_job(cylinder(3), p);
  EXPECT_TRUE(bad->step(0));
  EXPECT_TRUE(bad->failed());
  EXPECT_NE(bad->error().find("curation_delta"), std::string::npos) << bad->error();
  EXPECT_THROW((void)bad->result(), cvc::tiler_error);
}

// ---- T8: the parameter table ----------------------------------------------------------------

TEST(Tiler, ParameterTable) {
  cvc::app a;
  cvc::tile_params p;
  p.collinear_epsilon = 1e-15;
  p.curation_delta = 0.01;
  p.remove_intersections = true;
  p.correspondence_overlap = 0.5;
  p.include = {"a0.*", "d001"};
  p.exclude = {"a002"};
  p.z_first = 3;
  p.z_last = 9;
  p.cap_ends = false;
  p.color = {1, 0.5, 0};
  EXPECT_TRUE(cvc::params_validate(p).empty());
  std::size_t fields = 0;
  cvc::for_each_param<cvc::tile_params>([&](const auto &) { ++fields; });
  EXPECT_EQ(fields, 10u);
  cvc::state &node = a.root()("doc.lab.tiler");
  cvc::params_to_state(p, node);
  EXPECT_EQ(node("curation_delta").value(), "0.01");
  EXPECT_EQ(node("curation_delta").property("unit"), "world");
  EXPECT_EQ(node("z_first").property("min"), "-1");
  const auto back = cvc::params_from_state<cvc::tile_params>(node);
  ASSERT_TRUE(back.ok()) << back.message();
  EXPECT_EQ(back.value.collinear_epsilon, p.collinear_epsilon);
  EXPECT_EQ(back.value.curation_delta, p.curation_delta);
  EXPECT_EQ(back.value.remove_intersections, true);
  EXPECT_EQ(back.value.correspondence_overlap, 0.5);
  EXPECT_EQ(back.value.include, p.include);
  EXPECT_EQ(back.value.exclude, p.exclude);
  EXPECT_EQ(back.value.z_first, 3);
  EXPECT_EQ(back.value.z_last, 9);
  EXPECT_EQ(back.value.cap_ends, false);
  EXPECT_EQ(back.value.color, p.color);
  // Defaults seeded, bad values reported.
  cvc::state &fresh = a.root()("doc.lab2.tiler");
  const auto seeded = cvc::params_from_state<cvc::tile_params>(fresh);
  ASSERT_TRUE(seeded.ok()) << seeded.message();
  EXPECT_EQ(fresh("cap_ends").value(), "1");
  fresh("curation_delta").value("-1");
  fresh("include").value("(");
  const auto bad = cvc::params_from_state<cvc::tile_params>(fresh);
  EXPECT_FALSE(bad.ok());
  EXPECT_NE(bad.message().find("curation_delta"), std::string::npos) << bad.message();
  EXPECT_NE(bad.message().find("include"), std::string::npos) << bad.message();
}
