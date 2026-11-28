/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::tight_cocone (cvc/geometry/tight_cocone.h), the VolumeRover TightCocone port
// (docs/roadmap/VOLROVER-TEXMOL-APPS-ROADMAP.md §2.4, V1): legacy parity (T1,
// tools/volrover-goldens), known shapes (T3), invariants (T4), bad input (T5), no singletons (T6),
// resumable (T7), and the parameter table and its keys on the state tree (T8). Runs natively and
// under node (the ports lane).

#include "resumable_harness.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/geometry/skeleton.h>
#include <cvc/geometry/skeleton_params.h>
#include <cvc/geometry/tet_complex.h>
#include <cvc/geometry/tight_cocone.h>
#include <cvc/geometry/tight_cocone_params.h>
#include <cvc/state/state.h>
#include <fstream>
#include <gtest/gtest.h>
#include <map>
#include <numeric>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

namespace {

using pts_t = std::vector<std::array<double, 3>>;
constexpr double pi = 3.14159265358979323846;

// ---- samples of closed surfaces, deterministic ----------------------------------------------

// n points on the unit sphere (a Fibonacci spiral), each jittered along it by up to j of the
// spacing.
pts_t sphere(int n, double j, std::uint32_t seed, double radius = 1.0) {
  std::mt19937 g(seed);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  const double golden = pi * (3.0 - std::sqrt(5.0)), step = std::sqrt(4.0 * pi / n);
  pts_t p;
  for (int i = 0; i < n; ++i) {
    const double z = 1.0 - 2.0 * (i + 0.5) / n, r = std::sqrt(1.0 - z * z);
    const double a = golden * i + j * step * u(g) / std::max(r, 0.05);
    const double zz = std::clamp(z + j * step * u(g) * r, -1.0, 1.0), rr = std::sqrt(1 - zz * zz);
    p.push_back({radius * rr * std::cos(a), radius * rr * std::sin(a), radius * zz});
  }
  return p;
}

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

// ---- what a surface is ----------------------------------------------------------------------

struct topology {
  std::size_t vertices = 0, edges = 0, faces = 0;
  std::size_t open_edges = 0;         // edges with one triangle
  std::size_t non_manifold_edges = 0; // edges with three or more
  std::size_t misoriented_edges = 0;  // edges both of whose triangles run the same way
  double volume = 0.0;                // signed: positive when the triangles face out
  long euler() const { return long(vertices) - long(edges) + long(faces); }
  bool closed_manifold() const {
    return open_edges == 0 && non_manifold_edges == 0 && misoriented_edges == 0;
  }
};

topology topology_of(const cvc::geometry &g) {
  topology t;
  t.vertices = g.const_points().size();
  t.faces = g.const_tris().size();
  std::map<std::pair<std::size_t, std::size_t>, std::array<int, 2>> e; // (lo, hi): +, - runs
  const auto &P = g.const_points();
  for (const auto &tri : g.const_tris()) {
    for (int k = 0; k < 3; ++k) {
      const std::size_t a = tri[std::size_t(k)], b = tri[std::size_t((k + 1) % 3)];
      auto &c = e[{std::min(a, b), std::max(a, b)}];
      ++c[a < b ? 0 : 1];
    }
    const auto &a = P[tri[0]], &b = P[tri[1]], &c = P[tri[2]];
    t.volume += (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) +
                 a[2] * (b[0] * c[1] - b[1] * c[0])) /
                6.0;
  }
  t.edges = e.size();
  for (const auto &[k, c] : e) {
    const int n = c[0] + c[1];
    if (n == 1)
      ++t.open_edges;
    else if (n > 2)
      ++t.non_manifold_edges;
    else if (c[0] != 1)
      ++t.misoriented_edges;
  }
  return t;
}

std::string describe(const cvc::tight_cocone_result &r) {
  const topology t = topology_of(r.surface);
  std::ostringstream o;
  o << r.surface.const_tris().size() << " triangles over " << r.point_index.size() << " of "
    << r.points << " points, chi " << t.euler() << ", " << t.open_edges << " open, "
    << t.non_manifold_edges << " non-manifold, " << t.misoriented_edges << " misoriented edges, "
    << "volume " << t.volume << ", " << r.inside_cells << " of " << r.cells << " cells inside";
  for (const std::string &w : r.warnings)
    o << "; " << w;
  return o.str();
}

// The triangles by the input indices of their corners, each rotated to start at its smallest:
// the surface independent of how its points are numbered.
std::set<std::array<std::size_t, 3>> triangles_by_input(const cvc::tight_cocone_result &r) {
  std::set<std::array<std::size_t, 3>> s;
  for (const auto &t : r.surface.const_tris()) {
    std::array<std::size_t, 3> q = {r.point_index[t[0]], r.point_index[t[1]], r.point_index[t[2]]};
    std::rotate(q.begin(), std::min_element(q.begin(), q.end()), q.end());
    s.insert(q);
  }
  return s;
}

class TightCocone : public ::testing::Test {
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

using corner = std::array<double, 3>;
using tri3 = std::array<corner, 3>;

// A triangle by its corners, rotated to start at the smallest: orientation kept.
tri3 canonical(tri3 t) {
  std::rotate(t.begin(), std::min_element(t.begin(), t.end()), t.end());
  return t;
}
tri3 flipped(const tri3 &t) { return canonical(tri3{t[0], t[2], t[1]}); }

struct legacy_surface {
  std::size_t points = 0, vertices = 0;
  std::set<tri3> tris;
};

// tightcocone_driver.cpp's output.
legacy_surface read_tc(const std::string &path) {
  legacy_surface g;
  std::ifstream in(path);
  std::string line;
  std::vector<corner> verts;
  while (std::getline(in, line)) {
    std::istringstream l(line);
    std::string key;
    l >> key;
    if (key == "points") {
      l >> g.points;
    } else if (key == "vertices") {
      l >> g.vertices;
    } else if (key == "v") {
      corner c;
      l >> c[0] >> c[1] >> c[2];
      verts.push_back(c);
    } else if (key == "t") {
      std::size_t i, j, k;
      l >> i >> j >> k;
      g.tris.insert(canonical(tri3{verts.at(i), verts.at(j), verts.at(k)}));
    }
  }
  return g;
}

pts_t read_points(const std::string &path) {
  pts_t p;
  std::ifstream in(path);
  double x, y, z;
  while (in >> x >> y >> z)
    p.push_back({x, y, z});
  return p;
}

std::set<tri3> triangles_by_corner(const cvc::tight_cocone_result &r) {
  std::set<tri3> s;
  const auto &P = r.surface.const_points();
  for (const auto &t : r.surface.const_tris())
    s.insert(canonical(tri3{corner{P[t[0]][0], P[t[0]][1], P[t[0]][2]},
                            corner{P[t[1]][0], P[t[1]][1], P[t[1]][2]},
                            corner{P[t[2]][0], P[t[2]][1], P[t[2]][2]}}));
  return s;
}

// A case's parameters: the legacy names in its inputs/tightcocone/<case>.json, as the driver
// takes them. The json is flat and small, so a scan for "name": value is enough here.
cvc::tight_cocone_params case_params(const std::string &json) {
  cvc::tight_cocone_params p;
  auto num = [&](const char *name, double &out) {
    const std::string k = std::string("\"") + name + "\":";
    const std::size_t at = json.find(k);
    if (at != std::string::npos)
      out = std::strtod(json.c_str() + at + k.size(), nullptr);
  };
  double robust = 0.0;
  num("robust", robust);
  p.robust = robust != 0.0;
  num("bb_ratio", p.big_ball_ratio);
  num("theta_ff", p.theta_ff);
  num("theta_if", p.theta_if);
  num("flatness_ratio", p.flatness_ratio);
  num("cocone_phi", p.cocone_phi);
  num("flat_phi", p.flat_phi);
  return p;
}

struct parity {
  std::size_t legacy = 0, port = 0, shared = 0, flipped = 0;
  topology legacy_topology, port_topology;
};

// The legacy triangles as a geometry, their corners merged by coordinates.
cvc::geometry as_geometry(const std::set<tri3> &tris) {
  cvc::geometry g;
  std::map<corner, cvc::geometry::index_t> ids;
  for (const tri3 &t : tris) {
    cvc::geometry::tri_t ijk;
    for (int k = 0; k < 3; ++k) {
      const corner &c = t[std::size_t(k)];
      auto [it, fresh] = ids.try_emplace(c, cvc::geometry::index_t(g.points().size()));
      if (fresh)
        g.points().push_back(cvc::geometry::point_t{{c[0], c[1], c[2]}});
      ijk[std::size_t(k)] = it->second;
    }
    g.tris().push_back(ijk);
  }
  return g;
}

parity compare_with_legacy(const std::string &name, cvc::tight_cocone_result *out = nullptr) {
  const std::string root = std::string(CVC_REPO_DIR);
  const std::string gdir = root + "/src/cvc/tests/data/volrover-goldens/tightcocone/";
  std::ifstream jf(root + "/tools/volrover-goldens/inputs/tightcocone/" + name + ".json");
  std::string json((std::istreambuf_iterator<char>(jf)), std::istreambuf_iterator<char>());
  json.erase(std::remove_if(json.begin(), json.end(), [](char c) { return std::isspace(c); }),
             json.end());
  const std::string key = "\"input\":\"";
  const std::size_t at = json.find(key);
  EXPECT_NE(at, std::string::npos) << name;
  const std::string input =
      json.substr(at + key.size(), json.find('"', at + key.size()) - at - key.size());
  const pts_t p = read_points(root + "/tools/volrover-goldens/inputs/" + input);
  const legacy_surface want = read_tc(gdir + name + ".tc.txt");
  EXPECT_EQ(want.points, p.size()) << name;
  const cvc::tight_cocone_result r = cvc::tight_cocone(p, case_params(json));
  const std::set<tri3> got = triangles_by_corner(r);
  parity q;
  q.legacy = want.tris.size();
  q.port = got.size();
  for (const tri3 &t : got) {
    q.shared += want.tris.count(t);
    q.flipped += want.tris.count(flipped(t));
  }
  q.legacy_topology = topology_of(as_geometry(want.tris));
  q.port_topology = topology_of(r.surface);
  std::cout << "[ T1 ] " << name << ": legacy " << q.legacy << " triangles over " << want.vertices
            << " points, port " << q.port << " over " << r.point_index.size() << " (robust left "
            << r.robust_dropped << " out); " << q.shared << " the same, " << q.flipped
            << " turned the other way; legacy / port: chi " << q.legacy_topology.euler() << " / "
            << q.port_topology.euler() << ", open edges " << q.legacy_topology.open_edges << " / "
            << q.port_topology.open_edges << ", non-manifold "
            << q.legacy_topology.non_manifold_edges << " / " << q.port_topology.non_manifold_edges
            << "\n";
  if (out)
    *out = r;
  return q;
}

TEST_F(TightCocone, MatchesTheLegacyLibrary) {
  const std::string gdir =
      std::string(CVC_REPO_DIR) + "/src/cvc/tests/data/volrover-goldens/tightcocone/";
  if (!std::ifstream(gdir + "capsule.tc.txt"))
    GTEST_SKIP() << "goldens not found in " << gdir;
  // The triangulation is CGAL's cell for cell, and the port's Tight Cocone is the legacy code.
  // Where its peeling meets close choices, it takes them in the cells' order: CGAL's memory order
  // in the legacy, tet_complex's insertion order in the port (PocketTunnel's A/B found the same).
  // So on most cases the surface is the legacy's, triangle for triangle and facing the same way; on
  // the others both are surfaces of the same topology and nearly the same size that share most of
  // their triangles.
  // The robust cases also differ from one legacy run to the next on Windows, whose heap orders the
  // cells differently each run (five runs on noisy_sphere gave two surfaces 162 triangles apart).
  // `share` is the least fraction of the legacy triangles the port must have (1: all of them).
  const struct {
    const char *name;
    double share;
  } cases[] = {{"capsule", 1.0},           {"dumbbell", 1.0},
               {"box_grid", 1.0},          {"pocket", 1.0},
               {"helices_and_sheet", 1.0}, {"dumbbell_narrow", 1.0},
               {"capsule_robust", 1.0},    {"torus", 0.95},
               {"outlier_sphere", 0.90},   {"1grl", 0.97},
               {"noisy_sphere", 0.90}};
  for (const auto &c : cases) {
    SCOPED_TRACE(c.name);
    const parity q = compare_with_legacy(c.name);
    ASSERT_GT(q.legacy, 0u);
    EXPECT_EQ(q.flipped, 0u) << "every facet faces the same way as the legacy's";
    if (c.share >= 1.0) {
      EXPECT_EQ(q.port, q.legacy);
      EXPECT_EQ(q.shared, q.legacy);
    } else {
      // As closed as the legacy's (1GRL's protein surface at a 3 A map's resolution is not, in
      // either: a few edges have four triangles), and of the same Euler characteristic.
      EXPECT_GE(double(q.shared), c.share * double(q.legacy));
      EXPECT_EQ(q.port_topology.closed_manifold(), q.legacy_topology.closed_manifold());
      EXPECT_EQ(q.port_topology.open_edges, q.legacy_topology.open_edges);
      EXPECT_EQ(q.port_topology.euler(), q.legacy_topology.euler());
    }
  }
}

// ---- T3: known shapes -----------------------------------------------------------------------

TEST_F(TightCocone, KnownShapes) {
  // A sphere: one closed surface of genus 0 through every point, facing out.
  {
    const pts_t p = sphere(600, 0.25, 1);
    const cvc::tight_cocone_result r = cvc::tight_cocone(p);
    std::cout << "sphere: " << describe(r) << "\n";
    const topology t = topology_of(r.surface);
    EXPECT_TRUE(t.closed_manifold());
    EXPECT_EQ(t.euler(), 2);
    EXPECT_EQ(r.point_index.size(), p.size());
    EXPECT_GT(t.volume, 0.9 * 4.0 / 3.0 * pi); // facing out, close to the ball's volume
    EXPECT_LT(t.volume, 4.0 / 3.0 * pi);
    EXPECT_GT(r.inside_cells, 0u);
    EXPECT_TRUE(r.warnings.empty()) << describe(r);
  }
  // A torus: closed, genus 1.
  {
    const cvc::tight_cocone_result r = cvc::tight_cocone(torus(48, 16, 3.0, 1.0, 2));
    std::cout << "torus: " << describe(r) << "\n";
    const topology t = topology_of(r.surface);
    EXPECT_TRUE(t.closed_manifold());
    EXPECT_EQ(t.euler(), 0);
    EXPECT_GT(t.volume, 0.0);
  }
  // A capsule: closed, genus 0.
  {
    const cvc::tight_cocone_result r = cvc::tight_cocone(capsule(6.0, 1.0, 20, 30, 3));
    std::cout << "capsule: " << describe(r) << "\n";
    const topology t = topology_of(r.surface);
    EXPECT_TRUE(t.closed_manifold());
    EXPECT_EQ(t.euler(), 2);
    EXPECT_GT(t.volume, 0.0);
  }
  // A mesh's vertices: only the points are read, and the surface's points are the mesh's.
  {
    cvc::geometry g;
    for (const auto &q : sphere(300, 0.2, 4))
      g.points().push_back(cvc::geometry::point_t{{q[0], q[1], q[2]}});
    g.tris().push_back(cvc::geometry::tri_t{{0, 1, 2}}); // ignored
    const cvc::tight_cocone_result r = cvc::tight_cocone(g);
    EXPECT_TRUE(topology_of(r.surface).closed_manifold());
    for (std::size_t i = 0; i < r.point_index.size(); ++i)
      EXPECT_EQ(r.surface.const_points()[i], g.const_points()[r.point_index[i]]);
  }
}

// ---- T4: invariants -------------------------------------------------------------------------

TEST_F(TightCocone, CloseUnderReorderingAndExactMotions) {
  const pts_t p = capsule(5.0, 1.0, 16, 24, 5);
  const cvc::tight_cocone_result rp = cvc::tight_cocone(p);
  const auto ref = triangles_by_input(rp);
  // The points shuffled: the triangulation is the same (exact, perturbed by the points' order
  // alone, as CGAL's), but Tight Cocone's peeling is greedy in the cells' order, which follows the
  // insertion order: where its choices are close it takes other facets (PocketTunnel's A/B found
  // the same of the legacy library, in the cells' memory order). A closed surface again, with
  // nearly every triangle the same.
  std::vector<std::size_t> perm(p.size());
  std::iota(perm.begin(), perm.end(), 0);
  std::shuffle(perm.begin(), perm.end(), std::mt19937(11));
  pts_t q(p.size());
  for (std::size_t i = 0; i < p.size(); ++i)
    q[i] = p[perm[i]];
  const cvc::tight_cocone_result rq = cvc::tight_cocone(q);
  std::set<std::array<std::size_t, 3>> back;
  for (const auto &t : triangles_by_input(rq)) {
    std::array<std::size_t, 3> u = {perm[t[0]], perm[t[1]], perm[t[2]]};
    std::rotate(u.begin(), std::min_element(u.begin(), u.end()), u.end());
    back.insert(u);
  }
  const topology tp = topology_of(rp.surface), tq = topology_of(rq.surface);
  EXPECT_TRUE(tq.closed_manifold()) << describe(rq);
  EXPECT_EQ(tq.euler(), tp.euler());
  std::size_t shared = 0;
  for (const auto &t : back)
    shared += ref.count(t);
  std::cout << "reordered capsule: " << shared << " of " << ref.size() << " triangles the same, "
            << back.size() << " in all\n";
  EXPECT_GE(double(shared), 0.95 * double(ref.size()));
  // A quarter turn about z and a scale by two are exact in doubles, so the triangulation is the
  // same; but tet_complex inserts the points in a spatial order, which the turn changes, so the
  // peeling may again take a few other facets (on MSVC's sample none, on libc++'s a few). A closed
  // surface of the same genus, nearly every triangle the same.
  pts_t m = p;
  for (auto &v : m)
    v = {-2.0 * v[1], 2.0 * v[0], 2.0 * v[2]};
  const cvc::tight_cocone_result rm = cvc::tight_cocone(m);
  const topology tm = topology_of(rm.surface);
  EXPECT_TRUE(tm.closed_manifold()) << describe(rm);
  EXPECT_EQ(tm.euler(), tp.euler());
  EXPECT_GT(tm.volume, 0.0);
  std::size_t turned = 0;
  for (const auto &t : triangles_by_input(rm))
    turned += ref.count(t);
  std::cout << "turned capsule: " << turned << " of " << ref.size() << " triangles the same\n";
  EXPECT_GE(double(turned), 0.95 * double(ref.size()));
}

// The cocone core is one: the skeleton's Tight Cocone fields are tight_cocone's, with the same
// defaults.
TEST(TightCoconeParams, MatchTheSkeletonsTightCoconeFields) {
  const cvc::tight_cocone_params t;
  const cvc::skeleton_params s;
  EXPECT_EQ(t.robust, s.robust);
  EXPECT_EQ(t.big_ball_ratio, s.big_ball_ratio);
  EXPECT_EQ(t.theta_if, s.theta_if);
  EXPECT_EQ(t.theta_ff, s.theta_ff);
  EXPECT_EQ(t.cocone_phi, s.cocone_phi);
  EXPECT_EQ(t.sharp_phi, s.sharp_phi);
  EXPECT_EQ(t.flat_phi, s.flat_phi);
  EXPECT_EQ(t.flatness_ratio, s.flatness_ratio);
  // The legacy TightCocone::Parameters' defaults (tight_cocone.h).
  EXPECT_FALSE(t.robust);
  EXPECT_DOUBLE_EQ(t.big_ball_ratio, 0.25 * 0.25);
  EXPECT_DOUBLE_EQ(t.theta_if, pi / 180.0 * 5.0);
  EXPECT_DOUBLE_EQ(t.theta_ff, pi / 180.0 * 10.0);
  EXPECT_DOUBLE_EQ(t.cocone_phi, pi / 8.0);
  EXPECT_DOUBLE_EQ(t.sharp_phi, 2 * pi / 3.0);
  EXPECT_DOUBLE_EQ(t.flat_phi, pi / 3.0);
  EXPECT_DOUBLE_EQ(t.flatness_ratio, 1.2 * 1.2);
  std::vector<std::string> tn, sn;
  cvc::for_each_param<cvc::tight_cocone_params>([&](const auto &f) { tn.push_back(f.name); });
  cvc::for_each_param<cvc::skeleton_params>([&](const auto &f) { sn.push_back(f.name); });
  for (const std::string &n : tn)
    EXPECT_NE(std::find(sn.begin(), sn.end(), n), sn.end()) << n;
}

// ---- Robust Cocone ---------------------------------------------------------------------------

TEST_F(TightCocone, RobustCoconeFiltersANoisySample) {
  // A sphere with points scattered inside it: Robust Cocone leaves out those off the surface.
  pts_t p = sphere(800, 0.2, 6, 10.0);
  std::mt19937 g(7);
  std::uniform_real_distribution<double> u(-5.0, 5.0);
  for (int i = 0; i < 40; ++i)
    p.push_back({u(g), u(g), u(g)});
  cvc::tight_cocone_params params;
  params.robust = true;
  const cvc::tight_cocone_result r = cvc::tight_cocone(p, params);
  std::cout << "robust sphere: " << describe(r) << "\n";
  EXPECT_GT(r.robust_dropped, 0u);
  const topology t = topology_of(r.surface);
  EXPECT_GT(t.volume, 0.0);
  EXPECT_GT(r.surface.const_tris().size(), 0u);
  for (std::size_t i = 0; i < r.point_index.size(); ++i) {
    EXPECT_EQ(r.surface.const_points()[i],
              (cvc::geometry::point_t{
                  {p[r.point_index[i]][0], p[r.point_index[i]][1], p[r.point_index[i]][2]}}));
    if (i > 0)
      EXPECT_LT(r.point_index[i - 1], r.point_index[i]) << "the surface's points in input order";
  }
  // With `robust`, Robust Cocone runs between two triangulations.
  cvc::tight_cocone_stepper st(p, params);
  std::set<std::string> phases;
  while (!st.step(32))
    phases.insert(st.phase());
  EXPECT_TRUE(phases.count("robust cocone"));
}

// ---- T5: bad input --------------------------------------------------------------------------

std::string error_of(const pts_t &p, const cvc::tight_cocone_params &params = {}) {
  try {
    cvc::tight_cocone(p, params);
  } catch (const cvc::tight_cocone_error &e) {
    return e.what();
  }
  return "";
}

TEST_F(TightCocone, BadInputThrowsWithAReason) {
  EXPECT_NE(error_of({{0, 0, 0}, {1, 0, 0}}).find("2 points"), std::string::npos);
  EXPECT_NE(error_of({}).find("tight_cocone"), std::string::npos);
  pts_t plane, line;
  for (int i = 0; i < 30; ++i) {
    plane.push_back({double(i % 6), double(i / 6), 1.0});
    line.push_back({double(i), double(i), double(i)});
  }
  EXPECT_NE(error_of(plane).find("one plane"), std::string::npos) << error_of(plane);
  EXPECT_NE(error_of(line).find("one line"), std::string::npos) << error_of(line);
  pts_t nan = sphere(100, 0.2, 1);
  nan[3][0] = std::nan("");
  EXPECT_NE(error_of(nan).find("not finite"), std::string::npos);
  cvc::tight_cocone_params bad;
  bad.cocone_phi = 3.0;
  bad.big_ball_ratio = -1.0;
  const std::string e = error_of(sphere(100, 0.2, 1), bad);
  EXPECT_NE(e.find("cocone_phi"), std::string::npos) << e;
  EXPECT_NE(e.find("big_ball_ratio"), std::string::npos) << e;
  EXPECT_EQ(e.rfind("tight_cocone: invalid parameters", 0), 0u) << e;
}

TEST_F(TightCocone, DegenerateInputRunsToTheEnd) {
  // Duplicates are merged, with a warning; their first copy is the one the surface names.
  pts_t dup = sphere(300, 0.2, 8);
  const std::size_t n = dup.size();
  for (std::size_t i = 0; i < n; i += 3)
    dup.push_back(dup[i]);
  const cvc::tight_cocone_result d = cvc::tight_cocone(dup);
  EXPECT_EQ(d.duplicates, 100u);
  ASSERT_FALSE(d.warnings.empty());
  EXPECT_NE(d.warnings.front().find("100 duplicate"), std::string::npos) << describe(d);
  for (std::size_t i : d.point_index)
    EXPECT_LT(i, n);
  EXPECT_TRUE(topology_of(d.surface).closed_manifold()) << describe(d);
  // Exactly cospherical points (a cube's corners and face centres on a lattice), and the fewest
  // points there can be: a result, not a crash.
  pts_t lattice;
  for (int i = -2; i <= 2; ++i)
    for (int k = -2; k <= 2; ++k)
      for (int l = -2; l <= 2; ++l)
        if (std::abs(i) == 2 || std::abs(k) == 2 || std::abs(l) == 2)
          lattice.push_back({double(i), double(k), double(l)});
  std::cout << "lattice box: " << describe(cvc::tight_cocone(lattice)) << "\n";
  EXPECT_NO_THROW(cvc::tight_cocone(pts_t{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}}));
}

// ---- T6: no singletons ----------------------------------------------------------------------

bool same(const cvc::tight_cocone_result &a, const cvc::tight_cocone_result &b) {
  return a.surface.const_points() == b.surface.const_points() &&
         a.surface.const_tris() == b.surface.const_tris() && a.point_index == b.point_index &&
         a.points == b.points && a.duplicates == b.duplicates &&
         a.robust_dropped == b.robust_dropped && a.cells == b.cells &&
         a.inside_cells == b.inside_cells && a.warnings == b.warnings;
}

TEST_F(TightCocone, CallsAreIndependent) {
  const pts_t a = sphere(500, 0.2, 9, 4.0), b = capsule(8.0, 1.0, 20, 40, 8);
  cvc::tight_cocone_params pa, pb;
  pb.robust = true;
  pb.cocone_phi = 0.35;
  const cvc::tight_cocone_result ra = cvc::tight_cocone(a, pa), rb = cvc::tight_cocone(b, pb);
  // B after A, and A after B, equal each alone (the legacy bounding box was a global filled by
  // the first call).
  EXPECT_TRUE(same(cvc::tight_cocone(b, pb), rb));
  EXPECT_TRUE(same(cvc::tight_cocone(a, pa), ra));
  // Two at once on two threads, with different parameters.
  cvc::tight_cocone_result ta, tb;
  std::thread t1([&] { ta = cvc::tight_cocone(a, pa); });
  std::thread t2([&] { tb = cvc::tight_cocone(b, pb); });
  t1.join();
  t2.join();
  EXPECT_TRUE(same(ta, ra));
  EXPECT_TRUE(same(tb, rb));
}

// ---- T7: resumable --------------------------------------------------------------------------

TEST_F(TightCocone, StepperIsResumable) {
  const pts_t p = capsule(6.0, 1.0, 16, 24, 6);
  cvc::test::t7_subject<cvc::tight_cocone_stepper, cvc::tight_cocone_result> s;
  s.name = "tight cocone capsule";
  s.make = [p] { return std::make_unique<cvc::tight_cocone_stepper>(p); };
  s.take = [](cvc::tight_cocone_stepper &st) { return st.take(); };
  s.equal = same;
  s.blocking = [p] { return cvc::tight_cocone(p); };
  const cvc::test::t7_report r = cvc::test::run_t7(s);
  for (const char *phase : {"checking", "ordering", "inserting", "compacting", "circumcentres",
                            "building", "voronoi", "tight cocone", "surface"})
    EXPECT_NE(std::find(r.phases.begin(), r.phases.end(), phase), r.phases.end()) << phase;
  EXPECT_LT(r.slowest_step_ms, 1000.0);
  std::cout << "[ T7 ] tight_cocone: slowest step " << r.slowest_step_ms << " ms, budget 0 "
            << r.budget0_ms << " ms, stepped " << r.stepped_ms << " ms\n";
}

TEST_F(TightCocone, TheJobHandsOutTheResult) {
  const pts_t p = sphere(200, 0.2, 10);
  auto job = cvc::tight_cocone_job(p);
  while (!job->done())
    job->step(256);
  EXPECT_TRUE(same(job->take_result(), cvc::tight_cocone(p)));
}

// ---- T8: the parameter table ----------------------------------------------------------------

TEST(TightCoconeParams, Table) {
  cvc::tight_cocone_params p;
  p.robust = true;
  p.cocone_phi = 0.3;
  EXPECT_TRUE(cvc::params_validate(p).empty());
  std::size_t fields = 0;
  cvc::for_each_param<cvc::tight_cocone_params>([&](const auto &) { ++fields; });
  EXPECT_EQ(fields, 8u);
  cvc::tight_cocone_params q = p;
  q.flat_phi = -1.0;
  EXPECT_FALSE(cvc::params_validate(q).empty());
}

// The parameters on the state tree (§1.5): every field goes struct -> keys -> struct bit for bit;
// a missing key is seeded (with its properties); a value that does not parse is reported and its
// key left as it was. Needs no exact predicates, so it runs in every build.
TEST(TightCoconeParams, OnTheStateTree) {
  cvc::app app;
  cvc::state &node = cvc::state::instance(app)("doc.cocone.params");
  cvc::tight_cocone_params p;
  p.robust = true;
  p.big_ball_ratio = 1.0 / 3.0;
  p.theta_if = 0.1;
  p.theta_ff = 0.2;
  p.cocone_phi = 0.3;
  p.sharp_phi = 2.0;
  p.flat_phi = 0.9;
  p.flatness_ratio = 1.7;
  ASSERT_TRUE(cvc::params_validate(p).empty());
  cvc::params_to_state(p, node);
  const auto back = cvc::params_from_state<cvc::tight_cocone_params>(node);
  ASSERT_TRUE(back.ok()) << back.message();
  std::size_t fields = 0;
  cvc::for_each_param<cvc::tight_cocone_params>([&](const auto &f) {
    EXPECT_EQ(back.value.*(f.member), p.*(f.member)) << f.name;
    EXPECT_FALSE(node(f.name).property("doc").empty()) << f.name;
    ++fields;
  });
  EXPECT_EQ(fields, 8u);

  cvc::state &fresh = cvc::state::instance(app)("doc.other.params");
  const auto seeded = cvc::params_from_state<cvc::tight_cocone_params>(fresh);
  EXPECT_TRUE(seeded.ok()) << seeded.message();
  EXPECT_EQ(fresh("robust").value(), "0");
  EXPECT_EQ(fresh("cocone_phi").property("unit"), "rad");
  EXPECT_EQ(fresh("robust").property("type"), "bool");
  fresh("flat_phi").value("steep");
  const auto bad = cvc::params_from_state<cvc::tight_cocone_params>(fresh);
  EXPECT_FALSE(bad.ok());
  EXPECT_NE(bad.message().find("flat_phi"), std::string::npos) << bad.message();
  EXPECT_EQ(fresh("flat_phi").value(), "steep");
}

} // namespace
