/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::find_secondary_structures (cvc/geometry/secondary_structures.h), the SecondaryStructures
// port (docs/roadmap/TEXMOL-F2DOCK-ARISRV-ROADMAP.md T5): the legacy library's output on two
// committed surfaces (T1, tools/volrover-goldens), PDB 1GRL's helices, known shapes (T3),
// invariants (T4), bad input (T5), no singletons and two apps at once (T6), resumable (T7), and the
// parameter table on the state tree (T8). Runs natively and under node (the ports lane).

#include "resumable_harness.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/geometry/secondary_structures.h>
#include <cvc/geometry/secondary_structures_params.h>
#include <cvc/geometry/tet_complex.h>
#include <cvc/mol/blur.h>
#include <cvc/mol/molecule.h>
#include <cvc/state/state.h>
#include <fstream>
#include <gtest/gtest.h>
#include <iostream>
#include <limits>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>
#ifdef CVC_ENABLE_MESHER
#include <cvc/utility/algorithm.h>
#endif

namespace {

using pts_t = std::vector<std::array<double, 3>>;
constexpr double pi = 3.14159265358979323846;

cvc::geometry cloud(cvc::app &a, const pts_t &p) {
  cvc::geometry g(a);
  for (const auto &q : p)
    g.points().push_back(cvc::geometry::point_t{{q[0], q[1], q[2]}});
  return g;
}

// A capsule of radius r and length L from `o` along the unit direction d, sampled about every h.
void capsule(pts_t &p, std::array<double, 3> o, std::array<double, 3> d, double L, double r,
             double h, std::mt19937 &g) {
  std::uniform_real_distribution<double> j(-0.15, 0.15);
  // an orthonormal frame around d
  std::array<double, 3> u =
      std::fabs(d[0]) < 0.9 ? std::array<double, 3>{1, 0, 0} : std::array<double, 3>{0, 1, 0};
  const double dd = u[0] * d[0] + u[1] * d[1] + u[2] * d[2];
  for (int k = 0; k < 3; ++k)
    u[std::size_t(k)] -= dd * d[std::size_t(k)];
  const double ul = std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
  for (double &x : u)
    x /= ul;
  const std::array<double, 3> v{d[1] * u[2] - d[2] * u[1], d[2] * u[0] - d[0] * u[2],
                                d[0] * u[1] - d[1] * u[0]};
  auto put = [&](double t, double a, double rr) {
    p.push_back({o[0] + t * d[0] + rr * (std::cos(a) * u[0] + std::sin(a) * v[0]),
                 o[1] + t * d[1] + rr * (std::cos(a) * u[1] + std::sin(a) * v[1]),
                 o[2] + t * d[2] + rr * (std::cos(a) * u[2] + std::sin(a) * v[2])});
  };
  const int nring = std::max(8, int(std::lround(2 * pi * r / h)));
  const int nalong = std::max(2, int(std::lround(L / h)));
  for (int i = 0; i <= nalong; ++i)
    for (int k = 0; k < nring; ++k)
      put(L * (i + (i > 0 && i < nalong ? j(g) : 0.0)) / nalong, 2 * pi * (k + j(g)) / nring, r);
  const int ncap = std::max(2, int(std::lround(pi * r / 2 / h)));
  for (int side = 0; side < 2; ++side)
    for (int i = 1; i <= ncap; ++i) {
      const double t = (pi / 2) * (i + 0.1 * j(g)) / (ncap + 0.5);
      const int nr = std::max(1, int(std::lround(nring * std::cos(t))));
      for (int k = 0; k < nr; ++k)
        put(side == 0 ? -r * std::sin(t) : L + r * std::sin(t), 2 * pi * (k + j(g)) / nr,
            r * std::cos(t));
    }
}

// A closed box a x b x c at `o`, its faces sampled with spacing h.
void box(pts_t &p, std::array<double, 3> o, double a, double b, double c, double h,
         std::mt19937 &g) {
  std::uniform_real_distribution<double> j(-0.1 * h, 0.1 * h);
  const long na = std::lround(a / h), nb = std::lround(b / h), nc = std::lround(c / h);
  for (long i = 0; i <= na; ++i)
    for (long k = 0; k <= nb; ++k)
      for (long l = 0; l <= nc; ++l) {
        if (i != 0 && i != na && k != 0 && k != nb && l != 0 && l != nc)
          continue;
        auto jit = [&](long q, long n) { return (q == 0 || q == n) ? 0.0 : j(g); };
        p.push_back({o[0] + a * i / na + jit(i, na), o[1] + b * k / nb + jit(k, nb),
                     o[2] + c * l / nc + jit(l, nc)});
      }
}

std::string describe(const cvc::secondary_structures &s) {
  std::string d = std::to_string(s.helices.size()) + " helices (" +
                  std::to_string(s.helix_candidates) + " candidates), " +
                  std::to_string(s.sheets.size()) + " sheets (" +
                  std::to_string(s.sheet_candidates) + " candidates); skeleton " +
                  std::to_string(s.skeleton.points.size()) + " points, " +
                  std::to_string(s.skeleton.edges.size()) + " edges, " +
                  std::to_string(s.skeleton.faces.size()) + " polygons, " +
                  std::to_string(s.skeleton.component_area.size()) + " components";
  for (const auto &h : s.helices)
    d += "\n  helix length " + std::to_string(h.length) + " from (" + std::to_string(h.start[0]) +
         ", " + std::to_string(h.start[1]) + ", " + std::to_string(h.start[2]) + ") to (" +
         std::to_string(h.end[0]) + ", " + std::to_string(h.end[1]) + ", " +
         std::to_string(h.end[2]) + "), " + std::to_string(h.points.size()) + " points";
  for (const auto &sh : s.sheets)
    d += "\n  sheet area " + std::to_string(sh.area) + ", " + std::to_string(sh.polygons) +
         " polygons drawn";
  for (const std::string &w : s.warnings)
    d += "\n  warning: " + w;
  return d;
}

class SecondaryStructures : public ::testing::Test {
protected:
  void SetUp() override {
    if (!cvc::tet_complex_available())
      GTEST_SKIP() << "libcvc built without Shewchuk's predicates (CVC_WITH_SHEWCHUK)";
  }
};

// ---- the legacy library's output (tools/volrover-goldens) ----------------------------------

#ifndef CVC_REPO_DIR
#define CVC_REPO_DIR "."
#endif

struct legacy {
  int nv = 0, ne = 0, nf = 0, components = 0, helices = 0, drawn = 0, sheets = 0, candidates = 0;
  std::vector<double> areas, helix_widths, sheet_widths;
  std::vector<std::vector<std::array<double, 3>>> runs;
  std::vector<std::array<double, 6>> axes;
  std::vector<std::array<double, 5>> polygons; // n, centre, area
};

legacy read_legacy(const std::string &path) {
  legacy g;
  std::ifstream in(path);
  std::string kind;
  while (in >> kind) {
    if (kind == "N") {
      in >> g.nv >> g.ne >> g.nf >> g.components >> g.helices >> g.drawn >> g.sheets >>
          g.candidates;
    } else if (kind == "A" || kind == "W" || kind == "F") {
      double x = 0;
      in >> x;
      (kind == "A" ? g.areas : kind == "W" ? g.helix_widths : g.sheet_widths).push_back(x);
    } else if (kind == "R") {
      std::size_t n = 0;
      in >> n;
      std::vector<std::array<double, 3>> run(n);
      for (auto &p : run)
        in >> p[0] >> p[1] >> p[2];
      g.runs.push_back(run);
    } else if (kind == "H") {
      std::array<double, 6> a{};
      for (double &x : a)
        in >> x;
      g.axes.push_back(a);
    } else if (kind == "P") {
      std::array<double, 5> p{};
      for (double &x : p)
        in >> x;
      g.polygons.push_back(p);
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

// The sheet geometry's polygons as the legacy driver writes them: each a run of triangles fanned
// from a shared centre (its third vertex).
std::vector<std::array<double, 5>> sheet_polygons(const cvc::geometry &g) {
  std::vector<std::array<double, 5>> out;
  const auto &sp = g.const_points();
  const auto &st = g.const_tris();
  for (std::size_t i = 0; i < st.size();) {
    const auto mid = st[i][2];
    std::size_t j = i;
    double area = 0;
    while (j < st.size() && st[j][2] == mid) {
      const auto &p = sp[std::size_t(st[j][0])], &q = sp[std::size_t(st[j][1])],
                 &r = sp[std::size_t(st[j][2])];
      const double u[3] = {q[0] - p[0], q[1] - p[1], q[2] - p[2]};
      const double v[3] = {r[0] - p[0], r[1] - p[1], r[2] - p[2]};
      const double c[3] = {u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
                           u[0] * v[1] - u[1] * v[0]};
      area += 0.5 * std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
      ++j;
    }
    out.push_back({double(j - i), sp[std::size_t(mid)][0], sp[std::size_t(mid)][1],
                   sp[std::size_t(mid)][2], area});
    i = j;
  }
  return out;
}

// How many of the sorted `want` have a value in the sorted `got` within tol (each used once).
std::size_t matched_values(std::vector<double> want, std::vector<double> got, double tol) {
  std::sort(want.begin(), want.end());
  std::sort(got.begin(), got.end());
  std::size_t n = 0, j = 0;
  for (double w : want) {
    while (j < got.size() && got[j] < w - tol)
      ++j;
    if (j < got.size() && std::fabs(got[j] - w) <= tol) {
      ++n;
      ++j;
    }
  }
  return n;
}

double pdist(const std::array<double, 3> &a, const std::array<double, 3> &b) {
  return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) +
                   (a[2] - b[2]) * (a[2] - b[2]));
}

// The larger of the distances between two axes' ends, either way round.
double axis_dist(const std::array<double, 6> &a, const std::array<double, 3> &s,
                 const std::array<double, 3> &e) {
  const std::array<double, 3> a0{a[0], a[1], a[2]}, a1{a[3], a[4], a[5]};
  return std::min(std::max(pdist(a0, s), pdist(a1, e)), std::max(pdist(a0, e), pdist(a1, s)));
}

// Whether every point of `run` is among `pts` (within tol), and every point of `pts` among `run`:
// the same coordinates, whatever the multiplicities.
bool same_points(const std::vector<std::array<double, 3>> &run,
                 const std::vector<std::array<double, 3>> &pts, double tol) {
  auto covered = [tol](const std::vector<std::array<double, 3>> &x,
                       const std::vector<std::array<double, 3>> &y) {
    for (const auto &p : x) {
      bool found = false;
      for (const auto &q : y)
        if (pdist(p, q) < tol) {
          found = true;
          break;
        }
      if (!found)
        return false;
    }
    return true;
  };
  return covered(run, pts) && covered(pts, run);
}

// The port against the legacy library's golden for one committed input. exact: the same
// skeleton, histograms, helices (axes to 1e-6, the same points) and sheet polygons. Otherwise
// (a real surface, where Tight Cocone and U2, the core's, keep the legacy's results only up to the
// cells' order and rounding; see MatchesTheLegacyLibrary), shares of them, printed and bounded.
void compare_with_legacy(const std::string &name, bool exact) {
  SCOPED_TRACE(name);
  const std::string inputs =
      std::string(CVC_REPO_DIR) + "/tools/volrover-goldens/inputs/secondary/" + name + ".pts";
  const std::string golden = std::string(CVC_REPO_DIR) +
                             "/src/cvc/tests/data/volrover-goldens/secondary/" + name + ".ss.txt";
  const pts_t pts = read_points(inputs);
  ASSERT_FALSE(pts.empty()) << "no input at " << inputs;
  const legacy g = read_legacy(golden);
  ASSERT_GT(g.nf, 0) << "no golden at " << golden;
  cvc::app a;
  const cvc::secondary_structures s = cvc::find_secondary_structures(a, cloud(a, pts));
  std::cout << name << ": " << describe(s) << "\n";

  // Coordinates to 1e-9 of the input's extent (Voronoi vertices round differently when a cell
  // lists its vertices in another order); the legacy sheet geometry holds floats.
  double extent = 1.0;
  for (const auto &p : pts)
    for (double x : p)
      extent = std::max(extent, std::fabs(x));
  const double tol = 1e-9 * extent, ftol = 1e-5 * extent;

  // The skeleton graph's components and their areas, and the two histograms.
  const std::size_t am = matched_values(g.areas, s.skeleton.component_area, 1e-9 * extent * extent);
  const std::size_t hw = matched_values(g.helix_widths, s.helix_widths, tol);
  const std::size_t sw = matched_values(g.sheet_widths, s.sheet_widths, tol);
  std::cout << name << " skeleton: legacy " << g.nf << " polygons, " << g.ne << " edges, "
            << g.components << " components; port " << s.skeleton.faces.size() << ", "
            << s.skeleton.edges.size() << ", " << s.skeleton.component_area.size() << "; " << am
            << " component areas matched\n"
            << name << " helix widths: legacy " << g.helix_widths.size() << ", port "
            << s.helix_widths.size() << ", " << hw << " matched; sheet widths: legacy "
            << g.sheet_widths.size() << ", port " << s.sheet_widths.size() << ", " << sw
            << " matched\n";

  // The helices drawn: each legacy axis's nearest port axis, either way round.
  std::size_t helices_near = 0, helices_3 = 0, same_runs = 0;
  double worst = 0;
  for (int i = 0; i < g.drawn && i < int(g.axes.size()); ++i) {
    double best = 1e300;
    std::size_t at = 0;
    for (std::size_t k = 0; k < s.helices.size(); ++k) {
      const double d = axis_dist(g.axes[std::size_t(i)], s.helices[k].start, s.helices[k].end);
      if (d < best) {
        best = d;
        at = k;
      }
    }
    if (exact) {
      EXPECT_EQ(at, std::size_t(i)) << "helix " << i << " is the port's " << at;
      worst = std::max(worst, best);
    }
    helices_near += best < 1.0;
    helices_3 += best < 3.0;
    // The legacy's runs, in its order, are not the helices' order; the run with this axis's
    // points is the one whose points the port's helix has.
    for (const auto &run : g.runs)
      if (same_points(run, s.helices[at].points, 1e-6 * extent)) {
        ++same_runs;
        break;
      }
  }
  std::cout << name << " helices: legacy " << g.helices << " (" << g.drawn << " drawn), port "
            << s.helix_candidates << " (" << s.helices.size() << " drawn); " << helices_near
            << " legacy axes have a port axis within 1 at both ends (" << helices_3
            << " within 3), " << same_runs << " from the same points\n";

  // The sheets drawn: their polygons by centre, size and area.
  const auto got = sheet_polygons(s.sheet_geometry);
  std::size_t pm = 0;
  {
    std::vector<char> used(got.size(), 0);
    for (const auto &w : g.polygons)
      for (std::size_t k = 0; k < got.size(); ++k)
        if (!used[k] && got[k][0] == w[0] &&
            pdist({got[k][1], got[k][2], got[k][3]}, {w[1], w[2], w[3]}) < ftol &&
            std::fabs(got[k][4] - w[4]) < ftol * std::max(1.0, w[4])) {
          used[k] = 1;
          ++pm;
          break;
        }
  }
  std::cout << name << " sheets: legacy " << g.sheets << " of " << g.candidates << " ("
            << g.polygons.size() << " polygons), port " << s.sheets.size() << " of "
            << s.sheet_candidates << " (" << got.size() << " polygons); " << pm
            << " polygons matched\n";

  const auto share = [](std::size_t n, std::size_t of) {
    return of ? double(n) / double(of) : 1.0;
  };
  if (exact) {
    EXPECT_EQ(int(s.skeleton.faces.size()), g.nf);
    EXPECT_EQ(int(s.skeleton.edges.size()), g.ne);
    EXPECT_EQ(int(s.skeleton.component_area.size()), g.components);
    EXPECT_EQ(am, g.areas.size());
    EXPECT_EQ(hw, g.helix_widths.size());
    EXPECT_EQ(s.helix_widths.size(), g.helix_widths.size());
    EXPECT_EQ(sw, g.sheet_widths.size());
    EXPECT_EQ(s.sheet_widths.size(), g.sheet_widths.size());
    EXPECT_EQ(s.helix_candidates, g.helices);
    EXPECT_EQ(int(s.helices.size()), g.drawn);
    EXPECT_LT(worst, 1e-6);
    EXPECT_EQ(same_runs, std::size_t(g.drawn));
    EXPECT_EQ(int(s.sheets.size()), g.sheets);
    EXPECT_EQ(s.sheet_candidates, g.candidates);
    EXPECT_EQ(pm, g.polygons.size());
    EXPECT_EQ(got.size(), g.polygons.size());
  } else {
    // What 1GRL measured: 99% of the polygon widths, every helix width, 65 of the 69 component
    // areas, 15 of the 20 helix axes within 1 at both ends and 17 within 3, every sheet polygon.
    // The goldens are Linux's; a Windows build's legacy output gives 13 and 17. The bounds leave
    // room for another platform's.
    EXPECT_GE(share(sw, g.sheet_widths.size()), 0.95);
    EXPECT_GE(share(hw, g.helix_widths.size()), 0.95);
    EXPECT_GE(share(am, g.areas.size()), 0.85);
    EXPECT_GE(share(helices_near, std::size_t(g.drawn)), 0.5);
    EXPECT_GE(share(helices_3, std::size_t(g.drawn)), 0.75);
    EXPECT_GE(share(pm, g.polygons.size()), 0.8);
    EXPECT_EQ(int(s.sheets.size()), g.sheets);
  }
}

TEST_F(SecondaryStructures, MatchesTheLegacyLibrary) {
  compare_with_legacy("helices_and_sheet", true);
  compare_with_legacy("1grl", false);
}

// ---- PDB 1GRL: the helices found lie on its HELIX records ---------------------------------

// The committed 1GRL surface (inputs/secondary/1grl.pts, its blur map's isosurface) against the
// PDB file's own secondary structure: most helix axes found run through residues the file lists as
// helices, and most of its helices have an axis through them. A helix axis runs inside the ring of
// C-alphas (2.3 Å from them), so an axis point is on a helix when its nearest C-alpha is one, and a
// PDB helix is found when an axis passes within 3 Å of its C-alphas' centroid.
TEST_F(SecondaryStructures, FindsTheHelicesOf1GRL) {
  const std::string base = std::string(CVC_REPO_DIR);
  const pts_t pts = read_points(base + "/tools/volrover-goldens/inputs/secondary/1grl.pts");
  ASSERT_FALSE(pts.empty());
  cvc::app a;
  const cvc::mol::molecule m =
      cvc::mol::read_molecule(a, base + "/examples/data/molecules/1grl.pdb");
  const cvc::secondary_structures s = cvc::find_secondary_structures(a, cloud(a, pts));
  ASSERT_FALSE(s.helices.empty());
  struct ca_t {
    std::array<double, 3> p;
    bool helix;
  };
  std::vector<ca_t> ca;
  for (const cvc::mol::atom &at : m.atoms)
    if (at.name == "CA") {
      const int k = m.structure_of(at);
      ca.push_back({{at.x, at.y, at.z},
                    k >= 0 && m.structures[std::size_t(k)].type ==
                                  cvc::mol::secondary_structure::kind::helix});
    }
  int on_helices = 0;
  for (const cvc::secondary_helix &h : s.helices) {
    int in = 0, samples = 0;
    const int n = std::max(2, int(h.length / 0.5));
    for (int i = 0; i <= n; ++i, ++samples) {
      const double t = double(i) / n;
      const std::array<double, 3> p{h.start[0] + t * (h.end[0] - h.start[0]),
                                    h.start[1] + t * (h.end[1] - h.start[1]),
                                    h.start[2] + t * (h.end[2] - h.start[2])};
      double best = 1e300;
      bool helix = false;
      for (const ca_t &c : ca)
        if (pdist(p, c.p) < best) {
          best = pdist(p, c.p);
          helix = c.helix;
        }
      in += best < 4.0 && helix;
    }
    on_helices += 2 * in > samples;
  }
  int helices = 0, found = 0;
  for (std::size_t k = 0; k < m.structures.size(); ++k) {
    if (m.structures[k].type != cvc::mol::secondary_structure::kind::helix)
      continue;
    ++helices;
    std::array<double, 3> c{0, 0, 0};
    int n = 0;
    for (const cvc::mol::atom &at : m.atoms)
      if (at.name == "CA" && m.structure_of(at) == int(k)) {
        c = {c[0] + at.x, c[1] + at.y, c[2] + at.z};
        ++n;
      }
    if (n == 0)
      continue;
    c = {c[0] / n, c[1] / n, c[2] / n};
    for (const cvc::secondary_helix &h : s.helices) {
      const std::array<double, 3> d{h.end[0] - h.start[0], h.end[1] - h.start[1],
                                    h.end[2] - h.start[2]};
      double t =
          ((c[0] - h.start[0]) * d[0] + (c[1] - h.start[1]) * d[1] + (c[2] - h.start[2]) * d[2]) /
          (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
      t = std::min(1.0, std::max(0.0, t));
      if (pdist({h.start[0] + t * d[0], h.start[1] + t * d[1], h.start[2] + t * d[2]}, c) < 3.0) {
        ++found;
        break;
      }
    }
  }
  std::cout << "1GRL: " << on_helices << " of " << s.helices.size()
            << " helix axes lie on PDB helices; " << found << " of the PDB's " << helices
            << " helices have an axis through them\n";
  EXPECT_EQ(helices, 19);
  // Measured: 18 of 20 axes, 15 of 19 helices.
  EXPECT_GE(on_helices, int(0.8 * double(s.helices.size())));
  EXPECT_GE(found, 12);
}

// The committed 1GRL surface is what 1grl.json says it is: the isosurface at 12 of the heavy atoms'
// blur map (cvc::mol::blur, blobbiness -0.3, a 2 Å grid; cvc::iso), rounded to 1/64 Å. Native
// builds only: the wasm ports lane has no isosurfacing.
TEST_F(SecondaryStructures, The1GRLInputIsItsBlurMapsIsosurface) {
#ifndef CVC_ENABLE_MESHER
  GTEST_SKIP() << "libcvc was built without CVC_ENABLE_MESHER (cvc::iso)";
#else
  const std::string base = std::string(CVC_REPO_DIR);
  cvc::app a;
  cvc::mol::molecule_read_params rp;
  rp.hydrogens = false;
  const cvc::mol::molecule m =
      cvc::mol::read_molecule(a, base + "/examples/data/molecules/1grl.pdb", rp);
  cvc::mol::blur_params bp;
  bp.blobbiness = -0.3;
  bp.resolution = 2.0;
  const cvc::geometry g = cvc::iso(cvc::mol::blur(a, m, bp), 12.0);
  pts_t made;
  for (const auto &p : g.const_points())
    made.push_back(
        {std::round(p[0] * 64) / 64, std::round(p[1] * 64) / 64, std::round(p[2] * 64) / 64});
  pts_t kept = read_points(base + "/tools/volrover-goldens/inputs/secondary/1grl.pts");
  std::sort(made.begin(), made.end());
  std::sort(kept.begin(), kept.end());
  std::size_t same_points = 0;
  for (std::size_t i = 0, j = 0; i < kept.size() && j < made.size();) {
    if (kept[i] == made[j]) {
      ++same_points;
      ++i;
      ++j;
    } else if (kept[i] < made[j]) {
      ++i;
    } else {
      ++j;
    }
  }
  std::cout << "1GRL surface: " << made.size() << " points made, " << kept.size() << " committed, "
            << same_points << " the same\n";
  // The same mesh; another compiler's last bits may move a vertex across a 1/64 Å step.
  EXPECT_EQ(made.size(), kept.size());
  EXPECT_GE(double(same_points), 0.99 * double(kept.size()));
#endif
}

// ---- T3: known shapes ----------------------------------------------------------------------

TEST_F(SecondaryStructures, KnownShapes) {
  cvc::app a;
  std::mt19937 g(5);
  // A tube of radius 2.5 (a helix's density) along x: one helix on its axis.
  pts_t tube;
  capsule(tube, {0, 0, 0}, {1, 0, 0}, 20.0, 2.5, 0.8, g);
  const cvc::secondary_structures t = cvc::find_secondary_structures(a, cloud(a, tube));
  std::cout << "tube: " << describe(t) << "\n";
  ASSERT_EQ(t.helices.size(), 1u);
  const cvc::secondary_helix &h = t.helices[0];
  EXPECT_GT(h.length, 15.0);
  EXPECT_LT(h.length, 22.0);
  for (const auto &e : {h.start, h.end}) {
    EXPECT_LT(std::hypot(e[1], e[2]), 0.1);
    EXPECT_GT(e[0], -1.0);
    EXPECT_LT(e[0], 21.0);
  }
  EXPECT_EQ(h.radius, 1.8);
  EXPECT_TRUE(t.sheets.empty());
  // Its cylinder: 202 points and 400 triangles, the rims 1.8 from the axis.
  EXPECT_EQ(t.helix_geometry.const_points().size(), 202u);
  EXPECT_EQ(t.helix_geometry.const_tris().size(), 400u);
  EXPECT_EQ(t.helix_of_tri.size(), 400u);
  for (std::size_t i = 0; i < 200; ++i) {
    const auto &p = t.helix_geometry.const_points()[i];
    EXPECT_NEAR(std::hypot(p[1] - h.start[1], p[2] - h.start[2]), 1.8, 1e-3);
  }
  // The skeleton geometry: every edge of the graph.
  EXPECT_EQ(t.skeleton_geometry.const_lines().size(), t.skeleton.edges.size());

  // A slab 2.5 thick (a sheet's density): one sheet, about its area, and no helix.
  pts_t slab;
  box(slab, {0, 0, 0}, 20.0, 15.0, 2.5, 0.8, g);
  const cvc::secondary_structures s = cvc::find_secondary_structures(a, cloud(a, slab));
  std::cout << "slab: " << describe(s) << "\n";
  ASSERT_EQ(s.sheets.size(), 1u);
  EXPECT_GT(s.sheets[0].area, 0.8 * 20 * 15);
  EXPECT_LT(s.sheets[0].area, 1.2 * 20 * 15);
  EXPECT_GT(s.sheets[0].polygons, 0u);
  EXPECT_FALSE(s.sheet_geometry.const_tris().empty());
  EXPECT_EQ(s.sheet_of_tri.size(), s.sheet_geometry.const_tris().size());
  for (const auto &p : s.sheet_geometry.const_points())
    EXPECT_NEAR(p[2], 1.25, 0.5); // the sheet is the slab's mid-plane
  EXPECT_TRUE(s.helices.empty());

  // A tube of radius 4 is too wide for the helix window; a slab 6 thick is too thick for the sheet
  // window but at its rim, where the medial axis turns to the edges and corners.
  pts_t wide, thick;
  capsule(wide, {0, 0, 0}, {0, 1, 0}, 20.0, 4.0, 0.8, g);
  box(thick, {0, 0, 0}, 20.0, 15.0, 6.0, 0.8, g);
  const cvc::secondary_structures w = cvc::find_secondary_structures(a, cloud(a, wide));
  const cvc::secondary_structures k = cvc::find_secondary_structures(a, cloud(a, thick));
  std::cout << "thick slab: " << describe(k) << "\n";
  EXPECT_TRUE(w.helices.empty()) << describe(w);
  std::size_t inner = 0;
  for (const auto &p : k.sheet_geometry.const_points())
    inner += p[0] > 4 && p[0] < 16 && p[1] > 4 && p[1] < 11;
  EXPECT_EQ(inner, 0u) << "sheet points over the thick slab's mid-plane, away from its rim";
  // ... unless the windows say otherwise.
  cvc::secondary_params pw;
  pw.helix_min_width = 3.5;
  pw.helix_max_width = 4.5;
  EXPECT_EQ(cvc::find_secondary_structures(a, cloud(a, wide), pw).helices.size(), 1u);
}

// ---- T4: invariants ------------------------------------------------------------------------

bool same(const cvc::geometry &a, const cvc::geometry &b) {
  return a.const_points() == b.const_points() && a.const_colors() == b.const_colors() &&
         a.const_tris() == b.const_tris() && a.const_lines() == b.const_lines();
}

bool same(const cvc::secondary_structures &a, const cvc::secondary_structures &b) {
  if (a.helices.size() != b.helices.size() || a.sheets.size() != b.sheets.size())
    return false;
  for (std::size_t i = 0; i < a.helices.size(); ++i)
    if (a.helices[i].start != b.helices[i].start || a.helices[i].end != b.helices[i].end ||
        a.helices[i].length != b.helices[i].length || a.helices[i].radius != b.helices[i].radius ||
        a.helices[i].points != b.helices[i].points)
      return false;
  for (std::size_t i = 0; i < a.sheets.size(); ++i)
    if (a.sheets[i].component != b.sheets[i].component || a.sheets[i].area != b.sheets[i].area ||
        a.sheets[i].polygons != b.sheets[i].polygons)
      return false;
  const auto &sa = a.skeleton, &sb = b.skeleton;
  if (sa.points != sb.points || sa.component_area != sb.component_area ||
      sa.edges.size() != sb.edges.size() || sa.faces.size() != sb.faces.size())
    return false;
  for (std::size_t i = 0; i < sa.edges.size(); ++i)
    if (sa.edges[i].a != sb.edges[i].a || sa.edges[i].b != sb.edges[i].b ||
        sa.edges[i].width != sb.edges[i].width || sa.edges[i].faces != sb.edges[i].faces ||
        sa.edges[i].on_u1 != sb.edges[i].on_u1 || sa.edges[i].on_u2 != sb.edges[i].on_u2)
      return false;
  for (std::size_t i = 0; i < sa.faces.size(); ++i)
    if (sa.faces[i].points != sb.faces[i].points || sa.faces[i].width != sb.faces[i].width ||
        sa.faces[i].component != sb.faces[i].component)
      return false;
  return same(a.helix_geometry, b.helix_geometry) && same(a.sheet_geometry, b.sheet_geometry) &&
         same(a.skeleton_geometry, b.skeleton_geometry) && a.helix_of_tri == b.helix_of_tri &&
         a.sheet_of_tri == b.sheet_of_tri && a.helix_widths == b.helix_widths &&
         a.sheet_widths == b.sheet_widths && a.helix_candidates == b.helix_candidates &&
         a.sheet_candidates == b.sheet_candidates && a.warnings == b.warnings;
}

pts_t helices_and_sheet() {
  return read_points(std::string(CVC_REPO_DIR) +
                     "/tools/volrover-goldens/inputs/secondary/helices_and_sheet.pts");
}

TEST_F(SecondaryStructures, UnchangedByPointOrderAndMotion) {
  cvc::app a;
  const pts_t p = helices_and_sheet();
  ASSERT_FALSE(p.empty());
  const cvc::secondary_structures r = cvc::find_secondary_structures(a, cloud(a, p));
  // Any order of the points: the same result, bit for bit (they are sorted first).
  pts_t q = p;
  std::mt19937 g(17);
  std::shuffle(q.begin(), q.end(), g);
  EXPECT_TRUE(same(cvc::find_secondary_structures(a, cloud(a, q)), r));
  // Turned (x, y, z to y, z, x) and moved by an exact offset: the same helices and sheets, moved,
  // to rounding.
  const std::array<double, 3> off{10.25, -3.5, 7.125};
  pts_t m;
  for (const auto &x : p)
    m.push_back({x[1] + off[0], x[2] + off[1], x[0] + off[2]});
  const cvc::secondary_structures t = cvc::find_secondary_structures(a, cloud(a, m));
  ASSERT_EQ(t.helices.size(), r.helices.size()) << describe(t);
  ASSERT_EQ(t.sheets.size(), r.sheets.size());
  auto back = [&](const std::array<double, 3> &x) {
    return std::array<double, 3>{x[2] - off[2], x[0] - off[0], x[1] - off[1]};
  };
  for (std::size_t i = 0; i < r.helices.size(); ++i) {
    const std::array<double, 6> want{r.helices[i].start[0], r.helices[i].start[1],
                                     r.helices[i].start[2], r.helices[i].end[0],
                                     r.helices[i].end[1],   r.helices[i].end[2]};
    EXPECT_LT(axis_dist(want, back(t.helices[i].start), back(t.helices[i].end)), 1e-6) << i;
  }
  for (std::size_t i = 0; i < r.sheets.size(); ++i)
    EXPECT_NEAR(t.sheets[i].area, r.sheets[i].area, 1e-6 * r.sheets[i].area);
}

// A mesh is read by its faces: the points no face uses are left out.
TEST_F(SecondaryStructures, ASurfaceIsReadByItsFaces) {
  cvc::app a;
  std::mt19937 g(3);
  pts_t tube;
  capsule(tube, {0, 0, 0}, {1, 0, 0}, 20.0, 2.5, 0.8, g);
  const cvc::secondary_structures r = cvc::find_secondary_structures(a, cloud(a, tube));
  // The same points as faces' corners (any triangles: only which points they use counts), and far
  // points no face uses.
  cvc::geometry mesh(a);
  for (const auto &q : tube)
    mesh.points().push_back(cvc::geometry::point_t{{q[0], q[1], q[2]}});
  for (int k = 0; k < 20; ++k)
    mesh.points().push_back(cvc::geometry::point_t{{100.0 + k, -50.0, 3.0 * k}});
  for (std::size_t i = 0; i + 2 < tube.size(); i += 3)
    mesh.tris().push_back(cvc::geometry::tri_t{{i, i + 1, i + 2}});
  for (std::size_t i = tube.size() - tube.size() % 3; i < tube.size(); ++i)
    mesh.tris().push_back(cvc::geometry::tri_t{{i, 0, 1}});
  EXPECT_TRUE(same(cvc::find_secondary_structures(a, mesh), r));
}

// ---- T5: bad input -------------------------------------------------------------------------

std::string error_of(cvc::app &a, const cvc::geometry &g, const cvc::secondary_params &p = {}) {
  try {
    cvc::find_secondary_structures(a, g, p);
  } catch (const cvc::secondary_error &e) {
    return e.what();
  }
  return "";
}

TEST_F(SecondaryStructures, BadInputThrowsWithAReason) {
  cvc::app a;
  const std::string none = error_of(a, cloud(a, {}));
  EXPECT_NE(none.find("0 points"), std::string::npos) << none;
  const std::string three = error_of(a, cloud(a, {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}));
  EXPECT_NE(three.find("3 points"), std::string::npos) << three;
  pts_t plane, line;
  for (int i = 0; i < 30; ++i) {
    plane.push_back({double(i % 6), double(i / 6), 1.0});
    line.push_back({double(i), double(i), double(i)});
  }
  EXPECT_NE(error_of(a, cloud(a, plane)).find("one plane"), std::string::npos);
  EXPECT_NE(error_of(a, cloud(a, line)).find("one line"), std::string::npos);
  std::mt19937 g(1);
  pts_t tube;
  capsule(tube, {0, 0, 0}, {1, 0, 0}, 8.0, 2.5, 0.8, g);
  pts_t nan = tube;
  nan[5][1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_NE(error_of(a, cloud(a, nan)).find("not finite"), std::string::npos);
  cvc::geometry faces = cloud(a, tube);
  faces.tris().push_back(cvc::geometry::tri_t{{0, 1, tube.size() + 7}});
  EXPECT_NE(error_of(a, faces).find("past the surface's points"), std::string::npos);
  cvc::secondary_params bad;
  bad.helix_min_width = 3.5; // more than helix_max_width
  bad.sheet_count = -1;
  bad.medial_theta = std::numeric_limits<double>::quiet_NaN();
  const std::string e = error_of(a, cloud(a, tube), bad);
  EXPECT_NE(e.find("helix_min_width"), std::string::npos) << e;
  EXPECT_NE(e.find("sheet_count"), std::string::npos) << e;
  EXPECT_NE(e.find("medial_theta"), std::string::npos) << e;

  // select_secondary_structures checks the skeleton and the parameters it is given.
  const cvc::secondary_structures r = cvc::find_secondary_structures(a, cloud(a, tube));
  auto select_error = [&](cvc::secondary_skeleton s, const cvc::secondary_params &q) {
    try {
      cvc::select_secondary_structures(a, std::move(s), q);
    } catch (const cvc::secondary_error &x) {
      return std::string(x.what());
    }
    return std::string();
  };
  cvc::secondary_skeleton s1 = r.skeleton;
  ASSERT_FALSE(s1.edges.empty());
  s1.edges[0].b = std::uint32_t(s1.points.size());
  EXPECT_NE(select_error(s1, {}).find("edge 0"), std::string::npos);
  cvc::secondary_skeleton s2 = r.skeleton;
  s2.faces.push_back({});
  EXPECT_NE(select_error(s2, {}).find("no points"), std::string::npos);
  cvc::secondary_skeleton s3 = r.skeleton;
  s3.faces.push_back({{0, 1, 2}, 2.5, 99});
  EXPECT_NE(select_error(s3, {}).find("component 99"), std::string::npos);
  EXPECT_NE(select_error(r.skeleton, bad).find("helix_min_width"), std::string::npos);
}

TEST_F(SecondaryStructures, DegenerateInputRunsToTheEnd) {
  cvc::app a;
  // A box's surface on a lattice (exactly cospherical everywhere), with duplicates; four points.
  pts_t lattice;
  for (int i = 0; i <= 16; ++i)
    for (int j = 0; j <= 12; ++j)
      for (int k = 0; k <= 4; ++k)
        if (i == 0 || i == 16 || j == 0 || j == 12 || k == 0 || k == 4)
          lattice.push_back({0.75 * i, 0.75 * j, 0.75 * k});
  const cvc::secondary_structures l = cvc::find_secondary_structures(a, cloud(a, lattice));
  std::cout << "lattice: " << describe(l) << "\n";
  pts_t dup = lattice;
  for (std::size_t i = 0; i < lattice.size(); i += 4)
    dup.push_back(lattice[i]);
  const cvc::secondary_structures d = cvc::find_secondary_structures(a, cloud(a, dup));
  ASSERT_FALSE(d.warnings.empty());
  EXPECT_NE(d.warnings.front().find("duplicate"), std::string::npos);
  EXPECT_EQ(d.helices.size(), l.helices.size());
  EXPECT_NO_THROW(
      cvc::find_secondary_structures(a, cloud(a, {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}})));
}

// ---- T6: no singletons; two apps at once ---------------------------------------------------

TEST_F(SecondaryStructures, CallsAndAppsAreIndependent) {
  cvc::app a1, a2;
  const pts_t pa = helices_and_sheet();
  std::mt19937 g(8);
  pts_t pb;
  capsule(pb, {0, 0, 0}, {0, 0.6, 0.8}, 16.0, 2.5, 0.8, g);
  cvc::secondary_params qb;
  qb.helix_count = 1;
  qb.sheet_count = 0;
  const cvc::secondary_structures ra = cvc::find_secondary_structures(a1, cloud(a1, pa));
  const cvc::secondary_structures rb = cvc::find_secondary_structures(a2, cloud(a2, pb), qb);
  // B after A and A after B equal each alone.
  EXPECT_TRUE(same(cvc::find_secondary_structures(a2, cloud(a2, pb), qb), rb));
  EXPECT_TRUE(same(cvc::find_secondary_structures(a1, cloud(a1, pa)), ra));
  // Two apps at once, each stepping its own job on its own thread: the same results, each in its
  // own app's geometries.
  cvc::secondary_structures ta, tb;
  auto run = [](cvc::app &app, const pts_t &p, const cvc::secondary_params &q,
                cvc::secondary_structures &out) {
    auto job = cvc::find_secondary_structures_job(app, cloud(app, p), q);
    while (!job->step(512)) {
    }
    out = job->take_result();
  };
  std::thread t1([&] { run(a1, pa, cvc::secondary_params{}, ta); });
  std::thread t2([&] { run(a2, pb, qb, tb); });
  t1.join();
  t2.join();
  EXPECT_TRUE(same(ta, ra));
  EXPECT_TRUE(same(tb, rb));
  EXPECT_EQ(&ta.helix_geometry.ctx(), &a1);
  EXPECT_EQ(&ta.sheet_geometry.ctx(), &a1);
  EXPECT_EQ(&ta.skeleton_geometry.ctx(), &a1);
  EXPECT_EQ(&tb.helix_geometry.ctx(), &a2);
  EXPECT_EQ(&tb.skeleton_geometry.ctx(), &a2);
}

// ---- T7: resumable -------------------------------------------------------------------------

TEST_F(SecondaryStructures, StepperIsResumable) {
  cvc::app a;
  std::mt19937 g(6);
  pts_t p;
  capsule(p, {0, 0, 0}, {1, 0, 0}, 12.0, 2.5, 0.9, g);
  box(p, {-2, 8, -1}, 10.0, 8.0, 2.5, 0.9, g);
  const cvc::geometry surface = cloud(a, p);
  cvc::test::t7_subject<cvc::secondary_stepper, cvc::secondary_structures> s;
  s.name = "secondary structures: a tube and a slab";
  s.make = [&] { return std::make_unique<cvc::secondary_stepper>(a, surface); };
  s.take = [](cvc::secondary_stepper &st) { return st.take(); };
  s.equal = [](const cvc::secondary_structures &x, const cvc::secondary_structures &y) {
    return same(x, y);
  };
  s.blocking = [&] { return cvc::find_secondary_structures(a, surface); };
  const cvc::test::t7_report r = cvc::test::run_t7(s);
  for (const char *phase :
       {"checking", "ordering", "inserting", "compacting", "circumcentres", "building", "voronoi",
        "tight cocone", "medial axis", "u1", "skeleton", "u2", "sheets", "helices", "output"})
    EXPECT_NE(std::find(r.phases.begin(), r.phases.end(), phase), r.phases.end()) << phase;
  EXPECT_LT(r.slowest_step_ms, 1000.0);
}

// ---- the legacy GUI's sliders: the selection again, on a kept skeleton ----------------------

TEST_F(SecondaryStructures, SelectingAgainIsARunWithThoseParameters) {
  cvc::app a;
  const pts_t p = helices_and_sheet();
  const cvc::geometry surface = cloud(a, p);
  const cvc::secondary_structures r = cvc::find_secondary_structures(a, surface);
  cvc::secondary_params q;
  q.helix_count = 1;
  q.helix_min_length = 16.0;
  q.helix_radius = 2.2;
  q.sheet_min_width = 2.2;
  q.sheet_max_width = 2.8;
  q.helix_join_distance = 3.5;
  const cvc::secondary_structures again = cvc::select_secondary_structures(a, r.skeleton, q);
  EXPECT_TRUE(same(again, cvc::find_secondary_structures(a, surface, q)));
  ASSERT_EQ(again.helices.size(), 1u);
  EXPECT_EQ(again.helices[0].radius, 2.2);
  EXPECT_LT(again.sheet_geometry.const_tris().size(), r.sheet_geometry.const_tris().size());
  // No sheets at all, then the defaults back.
  cvc::secondary_params none;
  none.sheet_count = 0;
  EXPECT_TRUE(cvc::select_secondary_structures(a, r.skeleton, none).sheets.empty());
  EXPECT_TRUE(same(cvc::select_secondary_structures(a, r.skeleton), r));
}

// ---- T8: the parameter table ---------------------------------------------------------------

TEST(SecondaryStructuresParams, OnTheStateTree) {
  cvc::app app;
  cvc::state &node = cvc::state::instance(app)("doc.mol.secondary");
  cvc::secondary_params p;
  p.cocone_phi = 0.3;
  p.sharp_phi = 2.0;
  p.flat_phi = 0.9;
  p.flatness_ratio = 1.7;
  p.medial_theta = 0.35;
  p.medial_ratio = 50.0;
  p.helix_min_width = 1.0 / 3.0; // shortest round-trip text, not a rounded one
  p.helix_max_width = 3.5;
  p.helix_join_distance = 4.5;
  p.helix_min_length = 2.5;
  p.helix_count = 7;
  p.helix_radius = 2.0;
  p.sheet_min_width = 1.5;
  p.sheet_max_width = 3.25;
  p.sheet_min_area = 12.0;
  p.sheet_count = 3;
  ASSERT_TRUE(cvc::params_validate(p).empty());
  cvc::params_to_state(p, node);
  const auto back = cvc::params_from_state<cvc::secondary_params>(node);
  ASSERT_TRUE(back.ok()) << back.message();
  std::size_t fields = 0;
  cvc::for_each_param<cvc::secondary_params>([&](const auto &f) {
    EXPECT_EQ(back.value.*(f.member), p.*(f.member)) << f.name;
    EXPECT_FALSE(node(f.name).property("doc").empty()) << f.name;
    ++fields;
  });
  EXPECT_EQ(fields, 16u);
  // The windows' check across fields.
  cvc::secondary_params q;
  q.sheet_min_width = 4.0;
  const std::vector<std::string> problems = cvc::params_validate(q);
  ASSERT_EQ(problems.size(), 1u);
  EXPECT_NE(problems[0].find("sheet_min_width"), std::string::npos);
}

} // namespace
