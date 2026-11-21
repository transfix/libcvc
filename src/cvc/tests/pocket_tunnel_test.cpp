/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::pocket_tunnel (cvc/geometry/pocket_tunnel.h), the PocketTunnel port
// (docs/roadmap/TEXMOL-F2DOCK-ARISRV-ROADMAP.md §2.2, T5): legacy parity (T1, against
// tools/volrover-goldens), known shapes (T3), invariants of the output (T4), bad input (T5), no
// singletons, two apps at once (T6), resumable (T7), the parameter table and its keys on the state
// tree (T8). Runs natively and under node (the ports lane).

#include "../geometry/pocket_tunnel/pocket_tunnel_marked.h"
#include "resumable_harness.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/core/job_keys.h>
#include <cvc/core/sliced_task.h>
#include <cvc/geometry/pocket_tunnel.h>
#include <cvc/geometry/pocket_tunnel_params.h>
#include <cvc/geometry/tet_complex.h>
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

#ifndef CVC_REPO_DIR
#define CVC_REPO_DIR "."
#endif

namespace {

using pts_t = std::vector<std::array<double, 3>>;
constexpr double pi = 3.14159265358979323846;

// The committed inputs (tools/volrover-goldens/inputs/pockettunnel): balls with chambers and
// necks cut out (make_inputs.py says how), float-exact.
pts_t read_case(const std::string &name) {
  std::ifstream in(std::string(CVC_REPO_DIR) + "/tools/volrover-goldens/inputs/pockettunnel/" +
                   name + ".pts");
  pts_t p;
  double x, y, z;
  while (in >> x >> y >> z)
    p.push_back({x, y, z});
  return p;
}

std::string describe(const cvc::pocket_tunnels &r) {
  std::ostringstream d;
  d << r.points << " points: " << r.pockets << " pockets, " << r.tunnels << " tunnels, " << r.voids
    << " voids; " << r.surface.const_tris().size() << " triangles shown";
  for (std::size_t k = 0; k < std::min<std::size_t>(r.features.size(), 6); ++k) {
    const cvc::pocket_feature &f = r.features[k];
    d << "\n  " << k << ": " << cvc::to_string(f.kind) << ", " << f.mouths << " mouths, volume "
      << f.volume << ", " << f.cells << " cells, " << f.maxima << " maxima, shown " << f.shown;
  }
  for (const std::string &w : r.warnings)
    d << "\n  warning: " << w;
  return d.str();
}

class PocketTunnel : public ::testing::Test {
protected:
  void SetUp() override {
    if (!cvc::tet_complex_available())
      GTEST_SKIP() << "libcvc built without Shewchuk's predicates (CVC_WITH_SHEWCHUK)";
  }
};

// ---- T4: the output's invariants ------------------------------------------------------------

double det3(const cvc::geometry::point_t &a, const cvc::geometry::point_t &b,
            const cvc::geometry::point_t &c) {
  return a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) +
         a[2] * (b[0] * c[1] - b[1] * c[0]);
}

// Every shown feature's triangles bound it: each directed edge is matched by its reverse (a closed,
// consistently oriented surface), and the signed volume they enclose is the feature's volume
// (they face away from it). The surface holds every input point; a point on no triangle is white.
void check_output(const pts_t &in, const cvc::pocket_tunnels &r) {
  const cvc::geometry &g = r.surface;
  ASSERT_EQ(g.const_points().size(), in.size());
  ASSERT_EQ(g.const_colors().size(), in.size());
  for (std::size_t i = 0; i < in.size(); ++i)
    for (int k = 0; k < 3; ++k)
      ASSERT_EQ(g.const_points()[i][std::size_t(k)], in[i][std::size_t(k)]);
  const auto &tris = g.const_tris();
  ASSERT_EQ(r.feature_of_tri.size(), tris.size());
  ASSERT_EQ(r.tri_on_surface.size(), tris.size());
  int shown = 0, pockets = 0, tunnels = 0, voids = 0;
  for (std::size_t k = 0; k < r.features.size(); ++k) {
    const cvc::pocket_feature &f = r.features[k];
    if (k)
      EXPECT_LE(f.volume, r.features[k - 1].volume) << "largest first";
    EXPECT_EQ(f.kind, f.mouths == 0   ? cvc::pocket_kind::void_
                      : f.mouths == 1 ? cvc::pocket_kind::pocket
                                      : cvc::pocket_kind::tunnel);
    EXPECT_GT(f.volume, 0.0);
    EXPECT_GE(f.cells, 1u);
    EXPECT_GE(f.maxima, 1u);
    shown += f.shown >= 0;
    (f.kind == cvc::pocket_kind::pocket   ? pockets
     : f.kind == cvc::pocket_kind::tunnel ? tunnels
                                          : voids)++;
  }
  EXPECT_EQ(pockets, r.pockets);
  EXPECT_EQ(tunnels, r.tunnels);
  EXPECT_EQ(voids, r.voids);

  std::map<int, std::map<std::pair<int, int>, int>> edges; // feature -> directed edge -> count
  std::map<int, double> volume;
  std::vector<bool> on_tri(in.size(), false);
  // A duplicate point is never a corner: its first occurrence is.
  std::vector<std::size_t> first(in.size());
  {
    std::map<std::array<double, 3>, std::size_t> seen;
    for (std::size_t i = 0; i < in.size(); ++i)
      first[i] = seen.try_emplace(in[i], i).first->second;
  }
  for (std::size_t t = 0; t < tris.size(); ++t) {
    const int f = r.feature_of_tri[t];
    ASSERT_GE(f, 0);
    ASSERT_LT(std::size_t(f), r.features.size());
    EXPECT_GE(r.features[std::size_t(f)].shown, 0);
    const auto &tri = tris[t];
    for (int k = 0; k < 3; ++k) {
      ASSERT_LT(tri[std::size_t(k)], in.size());
      EXPECT_EQ(first[tri[std::size_t(k)]], tri[std::size_t(k)]) << "a corner is no duplicate";
      on_tri[tri[std::size_t(k)]] = true;
      edges[f][{int(tri[std::size_t(k)]), int(tri[std::size_t((k + 1) % 3)])}]++;
    }
    volume[f] +=
        det3(g.const_points()[tri[0]], g.const_points()[tri[1]], g.const_points()[tri[2]]) / 6.0;
  }
  EXPECT_EQ(int(edges.size()), shown) << "every shown feature has triangles";
  for (const auto &[f, e] : edges) {
    for (const auto &[uv, n] : e) {
      auto back = e.find({uv.second, uv.first});
      ASSERT_TRUE(back != e.end() && back->second == n)
          << "feature " << f << ": edge " << uv.first << "-" << uv.second << " is not matched";
    }
    const double want = r.features[std::size_t(f)].volume;
    EXPECT_NEAR(volume[f], want, 1e-9 * std::max(1.0, want)) << "feature " << f;
  }
  for (std::size_t i = 0; i < in.size(); ++i) {
    if (!on_tri[first[i]])
      EXPECT_EQ(g.const_colors()[i], (cvc::geometry::color_t{{1.0, 1.0, 1.0}})) << i;
    EXPECT_EQ(g.const_colors()[i], g.const_colors()[first[i]]) << i;
  }
}

// ---- T3: known shapes -------------------------------------------------------------------------

TEST_F(PocketTunnel, FindsTheChambers) {
  // A chamber with one neck is a pocket, with two a tunnel; each is the largest feature, about the
  // chamber's volume (the necks' cells flow to its centre too, or out of the ball).
  const pts_t pocket = read_case("pocket"), tunnel = read_case("tunnel");
  ASSERT_GT(pocket.size(), 1000u);
  const double chamber = 4.0 / 3.0 * pi * 1.4 * 1.4 * 1.4;

  const cvc::pocket_tunnels p = cvc::pocket_tunnel(pocket);
  std::cout << "pocket: " << describe(p) << "\n";
  check_output(pocket, p);
  ASSERT_FALSE(p.features.empty());
  EXPECT_EQ(p.features[0].kind, cvc::pocket_kind::pocket);
  EXPECT_GT(p.features[0].volume, 0.5 * chamber);
  EXPECT_LT(p.features[0].volume, 1.5 * chamber);

  const cvc::pocket_tunnels t = cvc::pocket_tunnel(tunnel);
  std::cout << "tunnel: " << describe(t) << "\n";
  check_output(tunnel, t);
  ASSERT_FALSE(t.features.empty());
  EXPECT_EQ(t.features[0].kind, cvc::pocket_kind::tunnel);
  EXPECT_EQ(t.features[0].mouths, 2);
  EXPECT_GT(t.features[0].volume, 0.5 * chamber);
  EXPECT_LT(t.features[0].volume, 1.5 * chamber);
}

TEST_F(PocketTunnel, ShowsTheLargestOfEachKind) {
  // three_chambers: a tunnel and two pockets of different sizes, among the small features a
  // sample makes. `tunnels` and `pockets` choose how many are shown, largest first, tunnels and
  // voids before pockets.
  const pts_t in = read_case("three_chambers");
  cvc::pocket_tunnel_params params;
  params.pockets = 2;
  params.tunnels = 1;
  const cvc::pocket_tunnels r = cvc::pocket_tunnel(in, params);
  std::cout << "three_chambers: " << describe(r) << "\n";
  check_output(in, r);
  std::vector<int> shown;
  for (std::size_t k = 0; k < r.features.size(); ++k)
    if (r.features[k].shown >= 0)
      shown.push_back(int(k));
  ASSERT_EQ(shown.size(), 3u);
  std::sort(shown.begin(), shown.end(), [&](int a, int b) {
    return r.features[std::size_t(a)].shown < r.features[std::size_t(b)].shown;
  });
  EXPECT_NE(r.features[std::size_t(shown[0])].kind, cvc::pocket_kind::pocket);
  EXPECT_EQ(r.features[std::size_t(shown[1])].kind, cvc::pocket_kind::pocket);
  EXPECT_EQ(r.features[std::size_t(shown[2])].kind, cvc::pocket_kind::pocket);
  EXPECT_LT(shown[1], shown[2]) << "the larger pocket first";
  // Nothing larger of its kind is left out.
  for (std::size_t k = 0; k < std::size_t(shown[2]); ++k)
    if (r.features[k].kind == cvc::pocket_kind::pocket)
      EXPECT_GE(r.features[k].shown, 0) << k;

  // None shown: no triangles, every point white; the features are the same.
  params.pockets = params.tunnels = 0;
  const cvc::pocket_tunnels none = cvc::pocket_tunnel(in, params);
  EXPECT_TRUE(none.surface.const_tris().empty());
  ASSERT_EQ(none.features.size(), r.features.size());
  for (std::size_t k = 0; k < r.features.size(); ++k) {
    EXPECT_EQ(none.features[k].volume, r.features[k].volume);
    EXPECT_EQ(none.features[k].shown, -1);
  }
  check_output(in, none);
}

TEST_F(PocketTunnel, ColoursAsTheLegacyLibrary) {
  // A pocket's facets on the surface are green and its mouth purple; a tunnel's yellow and red.
  // One feature shown, so no other's colours overwrite its points'.
  const pts_t pocket = read_case("pocket"), tunnel = read_case("tunnel");
  for (const pts_t *in : {&pocket, &tunnel}) {
    cvc::pocket_tunnel_params one;
    one.pockets = in == &pocket ? 1 : 0;
    one.tunnels = in == &pocket ? 0 : 1;
    const cvc::pocket_tunnels r = cvc::pocket_tunnel(*in, one);
    ASSERT_FALSE(r.surface.const_tris().empty());
    std::set<std::array<double, 3>> wall, mouth;
    const bool is_pocket = in == &pocket;
    const cvc::geometry::color_t wall_c = is_pocket ? cvc::geometry::color_t{{0.0, 1.0, 0.0}}
                                                    : cvc::geometry::color_t{{1.0, 1.0, 0.0}},
                                 mouth_c = is_pocket ? cvc::geometry::color_t{{1.0, 0.0, 1.0}}
                                                     : cvc::geometry::color_t{{1.0, 0.0, 0.0}};
    std::size_t walls = 0, mouths = 0;
    for (std::size_t t = 0; t < r.surface.const_tris().size(); ++t) {
      const bool on = r.tri_on_surface[t];
      (on ? walls : mouths)++;
      for (int k = 0; k < 3; ++k) {
        const auto &c = r.surface.const_colors()[r.surface.const_tris()[t][std::size_t(k)]];
        if (!on)
          EXPECT_EQ(c, mouth_c) << "a mouth's points keep the mouth's colour";
        else
          EXPECT_TRUE(c == wall_c || c == mouth_c);
      }
    }
    EXPECT_GT(walls, 0u);
    EXPECT_GT(mouths, 0u);
  }
}

TEST_F(PocketTunnel, AConvexSurfaceHasNoLargeFeature) {
  // A sphere's sample: its outside is the convex hull's, so what features there are lie in the
  // dents between sample points, tiny beside the ball.
  pts_t sphere;
  std::mt19937 g(5);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  for (int i = 0; i < 600; ++i) {
    const double th = std::acos(u(g)), ph = pi * (u(g) + 1.0);
    sphere.push_back(
        {2 * std::sin(th) * std::cos(ph), 2 * std::sin(th) * std::sin(ph), 2 * std::cos(th)});
  }
  const cvc::pocket_tunnels r = cvc::pocket_tunnel(sphere);
  std::cout << "sphere: " << describe(r) << "\n";
  check_output(sphere, r);
  double total = 0;
  for (const cvc::pocket_feature &f : r.features)
    total += f.volume;
  EXPECT_LT(total, 0.01 * 4.0 / 3.0 * pi * 8.0);
}

// ---- T5: bad and degenerate input --------------------------------------------------------------

std::string error_of(const pts_t &p, const cvc::pocket_tunnel_params &params = {}) {
  try {
    cvc::pocket_tunnel(p, params);
  } catch (const cvc::pocket_tunnel_error &e) {
    return e.what();
  }
  return "";
}

TEST_F(PocketTunnel, BadInputThrowsWithAReason) {
  EXPECT_NE(error_of({}).find("0 points"), std::string::npos) << error_of({});
  EXPECT_NE(error_of({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}).find("3 points"), std::string::npos);
  pts_t plane, line, same;
  for (int i = 0; i < 30; ++i) {
    plane.push_back({double(i % 6), double(i / 6), 1.0});
    line.push_back({double(i), double(i), double(i)});
    same.push_back({0.5, 0.5, 0.5});
  }
  EXPECT_NE(error_of(plane).find("one plane"), std::string::npos) << error_of(plane);
  EXPECT_NE(error_of(line).find("one line"), std::string::npos) << error_of(line);
  EXPECT_NE(error_of(same).find("same point"), std::string::npos) << error_of(same);
  pts_t nan = read_case("pocket");
  nan[3][1] = std::nan("");
  EXPECT_NE(error_of(nan).find("not finite"), std::string::npos) << error_of(nan);
  pts_t inf = read_case("pocket");
  inf[7][2] = HUGE_VAL;
  EXPECT_NE(error_of(inf).find("not finite"), std::string::npos) << error_of(inf);
  for (const std::string &e : {error_of({}), error_of(plane)})
    EXPECT_EQ(e.rfind("pocket_tunnel: ", 0), 0u) << e;

  cvc::pocket_tunnel_params bad;
  bad.pockets = -1;
  bad.cocone_phi = 4.0;
  const std::string e = error_of(read_case("pocket"), bad);
  EXPECT_NE(e.find("pockets"), std::string::npos) << e;
  EXPECT_NE(e.find("cocone_phi"), std::string::npos) << e;

  // An empty geometry, through the geometry entry and the job.
  EXPECT_THROW(cvc::pocket_tunnel(cvc::geometry{}), cvc::pocket_tunnel_error);
  auto job = cvc::pocket_tunnel_job(cvc::geometry{});
  job->step(0);
  EXPECT_TRUE(job->failed());
  EXPECT_NE(job->error().find("0 points"), std::string::npos) << job->error();
}

TEST_F(PocketTunnel, DegenerateInputRunsToTheEnd) {
  // Four points, five, a lattice (exactly cospherical everywhere), a box's surface on a lattice,
  // nearly cospherical points, and duplicates: a result, never a crash.
  for (const pts_t &p : {pts_t{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
                         pts_t{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 1}}}) {
    const cvc::pocket_tunnels r = cvc::pocket_tunnel(p);
    check_output(p, r);
  }
  pts_t lattice, box;
  for (int i = 0; i <= 6; ++i)
    for (int j = 0; j <= 6; ++j)
      for (int k = 0; k <= 6; ++k) {
        lattice.push_back({double(i), double(j), double(k)});
        if (i == 0 || i == 6 || j == 0 || j == 6 || k == 0 || k == 6)
          box.push_back({0.5 * i, 0.5 * j, 0.5 * k});
      }
  for (const pts_t *p : {&lattice, &box}) {
    const cvc::pocket_tunnels r = cvc::pocket_tunnel(*p);
    std::cout << (p == &lattice ? "lattice: " : "box: ") << describe(r) << "\n";
    check_output(*p, r);
  }
  pts_t sphere;
  for (int i = 0; i < 300; ++i) {
    const double z = 1.0 - 2.0 * (i + 0.5) / 300, s = std::sqrt(1.0 - z * z),
                 ph = i * pi * (3.0 - std::sqrt(5.0));
    sphere.push_back({s * std::cos(ph), s * std::sin(ph), z});
  }
  check_output(sphere, cvc::pocket_tunnel(sphere));

  pts_t dup = read_case("pocket");
  const std::size_t n = dup.size();
  for (std::size_t i = 0; i < n; i += 5)
    dup.push_back(dup[i]);
  const cvc::pocket_tunnels d = cvc::pocket_tunnel(dup);
  check_output(dup, d);
  ASSERT_FALSE(d.warnings.empty());
  EXPECT_NE(d.warnings.front().find("duplicate"), std::string::npos) << d.warnings.front();
  EXPECT_EQ(std::size_t(d.points), n);
  // A duplicate is coloured as the point it repeats, and the features are the input's without
  // duplicates.
  for (std::size_t i = n; i < dup.size(); ++i)
    EXPECT_EQ(d.surface.const_colors()[i], d.surface.const_colors()[(i - n) * 5]);
  const cvc::pocket_tunnels plain = cvc::pocket_tunnel(read_case("pocket"));
  ASSERT_EQ(d.features.size(), plain.features.size());
  for (std::size_t k = 0; k < plain.features.size(); ++k) {
    EXPECT_EQ(d.features[k].mouths, plain.features[k].mouths);
    EXPECT_EQ(d.features[k].cells, plain.features[k].cells);
  }
}

// ---- T4: point order ----------------------------------------------------------------------

using tri_key = std::array<double, 9>;
// A triangle by its corners' coordinates, rotated to start at its least corner (orientation kept).
tri_key key_of(const cvc::geometry &g, const cvc::geometry::tri_t &t) {
  std::array<std::array<double, 3>, 3> c;
  for (int k = 0; k < 3; ++k)
    for (int a = 0; a < 3; ++a)
      c[std::size_t(k)][std::size_t(a)] = g.const_points()[t[std::size_t(k)]][std::size_t(a)];
  const std::size_t m = std::size_t(std::min_element(c.begin(), c.end()) - c.begin());
  tri_key key;
  for (std::size_t k = 0; k < 3; ++k)
    for (std::size_t a = 0; a < 3; ++a)
      key[3 * k + a] = c[(m + k) % 3][a];
  return key;
}

// A permutation that is the same on every platform (std::shuffle's algorithm is the library's).
template <class T> void portable_shuffle(std::vector<T> &v, std::uint32_t seed) {
  std::mt19937 g(seed);
  for (std::size_t i = v.size(); i > 1; --i)
    std::swap(v[i - 1], v[std::size_t(g() % i)]);
}

TEST_F(PocketTunnel, PointOrderMovesOnlyAFewCellsByTheSurface) {
  // The triangulation is unique for a point set, but tet_complex numbers its cells in insertion
  // order, and Tight Cocone's greedy peeling walks in that order (as the legacy library's walks in
  // its cells' addresses; pocket_tunnel_marked.h). So a permutation of the input can mark a few
  // cells by the surface differently: the clusters of more than a hundred cells keep their kind
  // and mouths, and their cells and volume to 1%; the small ones may come and go.
  const pts_t in = read_case("three_chambers");
  const cvc::pocket_tunnels a = cvc::pocket_tunnel(in);
  for (const std::uint32_t seed : {17u, 18u, 19u}) {
    SCOPED_TRACE(seed);
    pts_t shuffled = in;
    portable_shuffle(shuffled, seed);
    const cvc::pocket_tunnels b = cvc::pocket_tunnel(shuffled);
    std::vector<const cvc::pocket_feature *> big_a, big_b;
    for (const cvc::pocket_feature &f : a.features)
      if (f.cells > 100)
        big_a.push_back(&f);
    for (const cvc::pocket_feature &f : b.features)
      if (f.cells > 100)
        big_b.push_back(&f);
    ASSERT_EQ(big_a.size(), big_b.size());
    std::cout << "seed " << seed << ": " << b.features.size() << " clusters (" << a.features.size()
              << " in input order)";
    for (std::size_t k = 0; k < big_a.size(); ++k) {
      std::cout << "; " << big_b[k]->cells << " cells (" << big_a[k]->cells << ")";
      EXPECT_EQ(big_a[k]->kind, big_b[k]->kind) << k;
      EXPECT_EQ(big_a[k]->mouths, big_b[k]->mouths) << k;
      EXPECT_NEAR(double(big_b[k]->cells), double(big_a[k]->cells), 0.01 * big_a[k]->cells) << k;
      EXPECT_NEAR(big_b[k]->volume, big_a[k]->volume, 0.01 * big_a[k]->volume) << k;
    }
    std::cout << "\n";
    // Run again in the same order, the result is the same, bit for bit.
    const cvc::pocket_tunnels again = cvc::pocket_tunnel(shuffled);
    ASSERT_EQ(again.features.size(), b.features.size());
    for (std::size_t k = 0; k < b.features.size(); ++k)
      EXPECT_EQ(again.features[k].volume, b.features[k].volume) << k;
    EXPECT_TRUE(again.surface.const_tris() == b.surface.const_tris());
  }
}

// ---- T6: no singletons -----------------------------------------------------------------------

bool same(const cvc::pocket_tunnels &a, const cvc::pocket_tunnels &b) {
  if (a.features.size() != b.features.size())
    return false;
  for (std::size_t k = 0; k < a.features.size(); ++k) {
    const cvc::pocket_feature &x = a.features[k], &y = b.features[k];
    if (x.kind != y.kind || x.mouths != y.mouths || x.volume != y.volume || x.cells != y.cells ||
        x.maxima != y.maxima || x.shown != y.shown)
      return false;
  }
  return a.surface.const_points() == b.surface.const_points() &&
         a.surface.const_colors() == b.surface.const_colors() &&
         a.surface.const_tris() == b.surface.const_tris() && a.feature_of_tri == b.feature_of_tri &&
         a.tri_on_surface == b.tri_on_surface && a.pockets == b.pockets && a.tunnels == b.tunnels &&
         a.voids == b.voids && a.points == b.points && a.warnings == b.warnings;
}

TEST_F(PocketTunnel, CallsAreIndependent) {
  const pts_t a = read_case("pocket"), b = read_case("tunnel");
  cvc::pocket_tunnel_params pa, pb;
  pb.pockets = 1;
  pb.tunnels = 5;
  pb.flatness_ratio = 1.3;
  const cvc::pocket_tunnels ra = cvc::pocket_tunnel(a, pa), rb = cvc::pocket_tunnel(b, pb);
  EXPECT_TRUE(same(cvc::pocket_tunnel(b, pb), rb));
  EXPECT_TRUE(same(cvc::pocket_tunnel(a, pa), ra));
  cvc::pocket_tunnels ta, tb;
  std::thread t1([&] { ta = cvc::pocket_tunnel(a, pa); });
  std::thread t2([&] { tb = cvc::pocket_tunnel(b, pb); });
  t1.join();
  t2.join();
  EXPECT_TRUE(same(ta, ra));
  EXPECT_TRUE(same(tb, rb));
}

TEST_F(PocketTunnel, TwoAppsAtOnce) {
  // Two apps in one process, each with its own parameters on its own state tree and its own job
  // on its own scheduler, both running at once: each gets the blocking call's result for its own
  // parameters, publishes to its own keys, and neither sees the other's.
  cvc::app a1, a2;
  const pts_t in = read_case("three_chambers");
  cvc::pocket_tunnel_params p1, p2;
  p1.pockets = 1;
  p1.tunnels = 0;
  p2.pockets = 3;
  p2.tunnels = 2;
  cvc::params_to_state(p1, cvc::state::instance(a1)("doc.ball.pocket_tunnel"));
  cvc::params_to_state(p2, cvc::state::instance(a2)("doc.ball.pocket_tunnel"));

  struct run {
    cvc::app *app;
    cvc::pocket_tunnel_params params;
    cvc::pocket_tunnels result;
    int pid = -1;
  } runs[2] = {{&a1, {}, {}}, {&a2, {}, {}}};
  for (run &r : runs) {
    const auto read = cvc::params_from_state<cvc::pocket_tunnel_params>(
        cvc::state::instance(*r.app)("doc.ball.pocket_tunnel"));
    ASSERT_TRUE(read.ok()) << read.message();
    r.params = read.value;
    std::shared_ptr<cvc::result_job<cvc::pocket_tunnels>> job(cvc::pocket_tunnel_job(in, r.params));
    auto keys = std::make_shared<cvc::job_keys>(r.app->root(), "doc.ball.job");
    cvc::sliced_task_options o;
    o.kind = "pocket_tunnel";
    o.threading = cvc::sliced_threading::cooperative;
    r.pid = cvc::schedule_published_job(
        *r.app, job, o, keys, "finding pockets...", [&r](cvc::sliced_job &j) {
          r.result = static_cast<cvc::result_job<cvc::pocket_tunnels> &>(j).take_result();
        });
    ASSERT_GE(r.pid, 0);
  }
  // Pump both schedulers in turn, so both jobs are part-way at once.
  bool saw_both_running = false;
  for (int frame = 0; frame < 1000000; ++frame) {
    const bool r1 = cvc::sliced_job_running(a1, runs[0].pid),
               r2 = cvc::sliced_job_running(a2, runs[1].pid);
    saw_both_running |= r1 && r2;
    if (!r1 && !r2)
      break;
    if (r1)
      cvc::test::detail::pump_frame(a1);
    if (r2)
      cvc::test::detail::pump_frame(a2);
  }
  EXPECT_TRUE(saw_both_running);
  EXPECT_EQ(runs[0].params.pockets, 1);
  EXPECT_EQ(runs[1].params.pockets, 3);
  EXPECT_TRUE(same(runs[0].result, cvc::pocket_tunnel(in, p1)));
  EXPECT_TRUE(same(runs[1].result, cvc::pocket_tunnel(in, p2)));
  for (cvc::app *a : {&a1, &a2})
    EXPECT_EQ(a->root()("doc.ball.job")("status").value().rfind("done", 0), 0u)
        << a->root()("doc.ball.job")("status").value();
  EXPECT_EQ(cvc::state::instance(a1)("doc.ball.pocket_tunnel")("pockets").value(), "1");
  EXPECT_EQ(cvc::state::instance(a2)("doc.ball.pocket_tunnel")("pockets").value(), "3");
}

// ---- T7: resumable -------------------------------------------------------------------------

TEST_F(PocketTunnel, StepperIsResumable) {
  const pts_t p = read_case("pocket");
  cvc::test::t7_subject<cvc::pocket_tunnel_stepper, cvc::pocket_tunnels> s;
  s.name = "pocket_tunnel pocket";
  s.make = [p] { return std::make_unique<cvc::pocket_tunnel_stepper>(p); };
  s.take = [](cvc::pocket_tunnel_stepper &st) { return st.take(); };
  s.equal = same;
  s.blocking = [p] { return cvc::pocket_tunnel(p); };
  const cvc::test::t7_report r = cvc::test::run_t7(s);
  for (const char *phase :
       {"checking", "ordering", "inserting", "compacting", "circumcentres", "building", "voronoi",
        "tight cocone", "maxima", "merging", "volumes", "sorting", "mouths", "output"})
    EXPECT_NE(std::find(r.phases.begin(), r.phases.end(), phase), r.phases.end()) << phase;
  EXPECT_LT(r.slowest_step_ms, 1000.0);
}

TEST_F(PocketTunnel, ACancelledJobGivesNothing) {
  auto job = cvc::pocket_tunnel_job(read_case("tunnel"));
  for (int i = 0; i < 40; ++i)
    job->step(64);
  job->cancel();
  job->step(64);
  EXPECT_TRUE(job->done());
  EXPECT_TRUE(job->cancelled());
  EXPECT_FALSE(job->has_result());
}

// ---- T8: the parameter table -------------------------------------------------------------------

TEST(PocketTunnelParams, OnTheStateTree) {
  cvc::app app;
  cvc::state &node = cvc::state::instance(app)("doc.mesh.pocket_tunnel");
  cvc::pocket_tunnel_params p;
  p.pockets = 7;
  p.tunnels = 0;
  p.cocone_phi = 1.0 / 3.0;
  p.sharp_phi = 2.0;
  p.flat_phi = 0.9;
  p.flatness_ratio = 1.7;
  ASSERT_TRUE(cvc::params_validate(p).empty());
  cvc::params_to_state(p, node);
  const auto back = cvc::params_from_state<cvc::pocket_tunnel_params>(node);
  ASSERT_TRUE(back.ok()) << back.message();
  std::size_t fields = 0;
  cvc::for_each_param<cvc::pocket_tunnel_params>([&](const auto &f) {
    EXPECT_EQ(back.value.*(f.member), p.*(f.member)) << f.name;
    EXPECT_FALSE(node(f.name).property("doc").empty()) << f.name;
    ++fields;
  });
  EXPECT_EQ(fields, 6u);

  cvc::state &fresh = cvc::state::instance(app)("doc.other.pocket_tunnel");
  const auto seeded = cvc::params_from_state<cvc::pocket_tunnel_params>(fresh);
  EXPECT_TRUE(seeded.ok()) << seeded.message();
  EXPECT_EQ(fresh("pockets").value(), "3");
  EXPECT_EQ(fresh("tunnels").value(), "3");
  EXPECT_EQ(fresh("pockets").property("default"), "3");
  EXPECT_EQ(fresh("cocone_phi").property("unit"), "rad");
  EXPECT_EQ(fresh("pockets").property("min"), "0");

  fresh("tunnels").value("several");
  const auto bad = cvc::params_from_state<cvc::pocket_tunnel_params>(fresh);
  EXPECT_FALSE(bad.ok());
  EXPECT_NE(bad.message().find("tunnels"), std::string::npos) << bad.message();
  fresh("tunnels").value("-2");
  const auto out = cvc::params_from_state<cvc::pocket_tunnel_params>(fresh);
  EXPECT_FALSE(out.ok());
  EXPECT_NE(out.message().find("tunnels"), std::string::npos) << out.message();
}

// ---- T1: legacy parity (tools/volrover-goldens) ---------------------------------------------

// A golden: what pockettunnel_driver.cpp wrote for the legacy library's run on a committed input.
struct golden {
  struct feature {
    char kind;
    int mouths;
    double volume;
    long cells, maxima;
  };
  std::vector<feature> features;
  pts_t points; // the legacy vertices, in CGAL's order
  std::vector<cvc::geometry::color_t> colours;
  std::vector<std::array<int, 5>> tris; // a, b, c, feature, surface
  std::vector<std::array<int, 4>> outside;
  std::vector<std::array<int, 3>> cocone;
};

golden read_golden(const std::string &name) {
  std::ifstream in(std::string(CVC_REPO_DIR) +
                   "/src/cvc/tests/data/volrover-goldens/pockettunnel/" + name + ".ptv.txt");
  golden g;
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream s(line);
    char tag = 0;
    s >> tag;
    if (tag == 'F') {
      golden::feature f{};
      s >> f.kind >> f.mouths >> f.volume >> f.cells >> f.maxima;
      g.features.push_back(f);
    } else if (tag == 'V') {
      std::array<double, 3> p;
      cvc::geometry::color_t c;
      s >> p[0] >> p[1] >> p[2] >> c[0] >> c[1] >> c[2];
      g.points.push_back(p);
      g.colours.push_back(c);
    } else if (tag == 'T') {
      std::array<int, 5> t;
      s >> t[0] >> t[1] >> t[2] >> t[3] >> t[4];
      g.tris.push_back(t);
    } else if (tag == 'O') {
      std::array<int, 4> c;
      s >> c[0] >> c[1] >> c[2] >> c[3];
      g.outside.push_back(c);
    } else if (tag == 'K') {
      std::array<int, 3> f;
      s >> f[0] >> f[1] >> f[2];
      g.cocone.push_back(f);
    }
  }
  return g;
}

// Each legacy vertex's index in the input (its first occurrence): the legacy library read the
// input as floats, and the committed inputs are float-exact, so the coordinates are the same.
std::vector<int> input_index(const golden &g, const pts_t &in) {
  std::map<std::array<double, 3>, int> first;
  for (std::size_t i = 0; i < in.size(); ++i)
    first.try_emplace(in[i], int(i));
  std::vector<int> idx;
  for (const auto &p : g.points) {
    auto it = first.find(p);
    idx.push_back(it == first.end() ? -1 : it->second);
  }
  return idx;
}

cvc::pocket_tunnel_detail::marking marking_of(const golden &g, const std::vector<int> &idx) {
  cvc::pocket_tunnel_detail::marking m;
  for (const auto &c : g.outside) {
    std::array<int, 4> k = {idx[std::size_t(c[0])], idx[std::size_t(c[1])], idx[std::size_t(c[2])],
                            idx[std::size_t(c[3])]};
    std::sort(k.begin(), k.end());
    m.outside.insert(k);
  }
  for (const auto &f : g.cocone) {
    std::array<int, 3> k = {idx[std::size_t(f[0])], idx[std::size_t(f[1])], idx[std::size_t(f[2])]};
    std::sort(k.begin(), k.end());
    m.cocone.insert(k);
  }
  return m;
}

// The shown triangles by their corners' coordinates (rotated to start at the least, orientation
// kept), with the feature each bounds and its class.
using shown_tri = std::tuple<std::array<double, 9>, int, bool>;
std::multiset<shown_tri> shown_of(const golden &g) {
  std::multiset<shown_tri> s;
  cvc::geometry pts;
  for (const auto &p : g.points)
    pts.points().push_back(cvc::geometry::point_t{{p[0], p[1], p[2]}});
  for (const auto &t : g.tris)
    s.insert({key_of(pts, cvc::geometry::tri_t{{cvc::geometry::index_t(t[0]),
                                                cvc::geometry::index_t(t[1]),
                                                cvc::geometry::index_t(t[2])}}),
              t[3], t[4] != 0});
  return s;
}
std::multiset<shown_tri> shown_of(const cvc::pocket_tunnels &r) {
  std::multiset<shown_tri> s;
  for (std::size_t t = 0; t < r.surface.const_tris().size(); ++t)
    s.insert({key_of(r.surface, r.surface.const_tris()[t]), r.feature_of_tri[t],
              bool(r.tri_on_surface[t])});
  return s;
}

char kind_char(cvc::pocket_kind k) {
  return k == cvc::pocket_kind::void_ ? 'V' : k == cvc::pocket_kind::pocket ? 'P' : 'T';
}

// The port's stages after Tight Cocone, on the legacy library's marking: every cluster (kind,
// mouths, cells, maxima, volume to 1e-12 of it: summed in another order), every shown triangle
// with its cluster and class, and every point's colour.
void check_stages(const std::string &name) {
  SCOPED_TRACE(name);
  const pts_t in = read_case(name);
  const golden g = read_golden(name);
  ASSERT_FALSE(g.points.empty()) << "no golden for " << name;
  const std::vector<int> idx = input_index(g, in);
  ASSERT_TRUE(std::find(idx.begin(), idx.end(), -1) == idx.end());
  const cvc::pocket_tunnels r =
      cvc::pocket_tunnel_detail::pocket_tunnel_marked(in, {}, marking_of(g, idx));
  check_output(in, r);
  ASSERT_EQ(r.features.size(), g.features.size());
  for (std::size_t k = 0; k < g.features.size(); ++k) {
    const cvc::pocket_feature &f = r.features[k];
    const golden::feature &w = g.features[k];
    EXPECT_EQ(kind_char(f.kind), w.kind) << k;
    EXPECT_EQ(f.mouths, w.mouths) << k;
    EXPECT_EQ(long(f.cells), w.cells) << k;
    EXPECT_EQ(long(f.maxima), w.maxima) << k;
    EXPECT_NEAR(f.volume, w.volume, 1e-12 * w.volume) << k;
  }
  EXPECT_TRUE(shown_of(r) == shown_of(g)) << "the shown triangles differ";
  std::size_t colours = 0;
  for (std::size_t v = 0; v < g.points.size(); ++v) {
    EXPECT_EQ(r.surface.const_colors()[std::size_t(idx[v])], g.colours[v]) << v;
    colours += g.colours[v] != cvc::geometry::color_t{{1.0, 1.0, 1.0}};
  }
  std::cout << name << ": the legacy marking gives " << g.features.size() << " clusters, "
            << g.tris.size() << " triangles, " << colours << " coloured points, all equal\n";
}

TEST_F(PocketTunnel, MatchesTheLegacyLibraryAfterTightCocone) {
  // PocketTunnel's own stages (S_max, the mouths, the output), from the legacy library's Tight
  // Cocone marking: the same clusters, triangles and colours as the legacy library's.
  for (const char *name : {"pocket", "tunnel", "three_chambers", "cavity"})
    check_stages(name);
}

TEST_F(PocketTunnel, MatchesTheLegacyLibrary) {
  // The whole pipeline. Tight Cocone's peeling is greedy and walks in the cells' order: the
  // legacy library's is their addresses, the port's tet_complex's spatial order
  // (pocket_tunnel_marked.h). Where the two orders mark a few cells by the surface differently,
  // the clusters of a cell or two those cells make differ, and a large cluster gains or loses a
  // few cells. On the pocket, the three chambers and the cavity the marking is the same, and so is
  // the whole result; on the tunnel the port's tunnel has 2 more of its 3,083 cells, and the port
  // finds 5 pockets of one or two cells the legacy run did not (one legacy run in five, its cells
  // elsewhere in the heap, finds five such pockets too). Every cluster of more than a
  // hundred cells is the legacy library's, to 1% of its cells and volume.
  for (const char *name : {"pocket", "tunnel", "three_chambers", "cavity"}) {
    SCOPED_TRACE(name);
    const pts_t in = read_case(name);
    const golden g = read_golden(name);
    ASSERT_FALSE(g.points.empty()) << "no golden for " << name;
    const std::vector<int> idx = input_index(g, in);
    const cvc::pocket_tunnels r = cvc::pocket_tunnel(in);
    if (std::string(name) != "tunnel") {
      ASSERT_EQ(r.features.size(), g.features.size());
      for (std::size_t k = 0; k < g.features.size(); ++k) {
        EXPECT_EQ(long(r.features[k].cells), g.features[k].cells) << k;
        EXPECT_EQ(long(r.features[k].maxima), g.features[k].maxima) << k;
        EXPECT_NEAR(r.features[k].volume, g.features[k].volume, 1e-12 * g.features[k].volume);
      }
      EXPECT_TRUE(shown_of(r) == shown_of(g)) << "the shown triangles differ";
      for (std::size_t v = 0; v < g.points.size(); ++v)
        EXPECT_EQ(r.surface.const_colors()[std::size_t(idx[v])], g.colours[v]) << v;
    }
    std::vector<golden::feature> big_g, big_r;
    for (const golden::feature &f : g.features)
      if (f.cells > 100)
        big_g.push_back(f);
    for (const cvc::pocket_feature &f : r.features)
      if (f.cells > 100)
        big_r.push_back({kind_char(f.kind), f.mouths, f.volume, long(f.cells), long(f.maxima)});
    std::cout << name << ": legacy " << g.features.size() << " clusters (" << big_g.size()
              << " of more than 100 cells), port " << r.features.size() << " (" << big_r.size()
              << ")\n";
    ASSERT_EQ(big_r.size(), big_g.size());
    for (std::size_t k = 0; k < big_g.size(); ++k) {
      std::cout << "  " << k << ": legacy " << big_g[k].kind << " " << big_g[k].mouths << " "
                << big_g[k].cells << " cells " << big_g[k].volume << "; port " << big_r[k].kind
                << " " << big_r[k].mouths << " " << big_r[k].cells << " cells " << big_r[k].volume
                << "\n";
      EXPECT_EQ(big_r[k].kind, big_g[k].kind) << k;
      EXPECT_EQ(big_r[k].mouths, big_g[k].mouths) << k;
      EXPECT_NEAR(double(big_r[k].cells), double(big_g[k].cells), 0.01 * big_g[k].cells) << k;
      EXPECT_NEAR(big_r[k].volume, big_g[k].volume, 0.01 * big_g[k].volume) << k;
    }
  }
}

} // namespace
