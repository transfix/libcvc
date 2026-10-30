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

// Tests for the contour tree and contour spectrum of a field on a mesh (cvc/volume/contour_mesh.h)
// and for cvc::tf's overlays on one:
//   - on a grid's own tetrahedra (or triangles, in 2D) the mesh contour tree is exactly the grid
//     builder's, random data with plateaus included, however it is stepped;
//   - a linear field's spectrum is known in closed form: on the unit cube's tetrahedra, f = x has
//     area 1, volumes w and 1 - w and gradient 1 at every isovalue; on the unit square's triangles
//     the contour length is 1 and the areas w and 1 - w;
//   - stepping the spectrum builder never changes a bit of it;
//   - a ring (not simply connected) still gives a tree, two separate pieces give one each;
//   - bad input throws, a NaN point is left out, a constant field gives one node;
//   - an overlay_job and an overlay task on a mesh give the one-shot layers.

#include <algorithm>
#include <cmath>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/state/state.h>
#include <cvc/tf/overlay_tasks.h>
#include <cvc/tf/overlays.h>
#include <cvc/volume/contour_mesh.h>
#include <cvc/volume/contour_spectrum.h>
#include <cvc/volume/contour_tree.h>
#include <gtest/gtest.h>
#include <limits>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace {

using cvc::contour_mesh;
using cvc::contour_tree;
using cvc::geometry;

// A grid's points, numbered as voxels (i + j*nx + k*nx*ny) at spacing h, and its simplices: each
// cube split into the 6 tetrahedra of the (+1,+1,-1) diagonal -- the paths from corner (0,0,1) to
// (1,1,0) by +x, +y and -z in every order, the grid contour tree's own stencil -- or, with nz == 1,
// each square into the 2 triangles of its (1,1) diagonal.
std::shared_ptr<geometry> grid_mesh(int nx, int ny, int nz, double h = 1.0) {
  auto g = std::make_shared<geometry>();
  const auto id = [&](int i, int j, int k) { return geometry::index_t(i + j * nx + k * nx * ny); };
  for (int k = 0; k < nz; ++k)
    for (int j = 0; j < ny; ++j)
      for (int i = 0; i < nx; ++i)
        g->points().push_back(geometry::point_t{{i * h, j * h, k * h}});
  if (nz == 1) {
    for (int j = 0; j + 1 < ny; ++j)
      for (int i = 0; i + 1 < nx; ++i) {
        g->tris().push_back(geometry::tri_t{{id(i, j, 0), id(i + 1, j, 0), id(i + 1, j + 1, 0)}});
        g->tris().push_back(geometry::tri_t{{id(i, j, 0), id(i, j + 1, 0), id(i + 1, j + 1, 0)}});
      }
    return g;
  }
  const int steps[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, -1}};
  int perm[3] = {0, 1, 2};
  for (int k = 0; k + 1 < nz; ++k)
    for (int j = 0; j + 1 < ny; ++j)
      for (int i = 0; i + 1 < nx; ++i) {
        std::sort(perm, perm + 3);
        do {
          int p[3] = {i, j, k + 1};
          geometry::tet_t t;
          t[0] = id(p[0], p[1], p[2]);
          for (int s = 0; s < 3; ++s) {
            for (int c = 0; c < 3; ++c)
              p[c] += steps[perm[s]][c];
            t[std::size_t(s) + 1] = id(p[0], p[1], p[2]);
          }
          g->tets().push_back(t);
        } while (std::next_permutation(perm, perm + 3));
      }
  return g;
}

std::shared_ptr<contour_mesh> field(std::shared_ptr<const geometry> g, std::vector<double> v) {
  auto m = std::make_shared<contour_mesh>();
  m->mesh = std::move(g);
  m->values = std::move(v);
  return m;
}

// A tree's nodes as (point, value, kind) in order, and its arcs as a set of (hi, lo).
using node_row = std::tuple<cvc::uint64, double, int>;
std::vector<node_row> nodes_of(const contour_tree &t) {
  std::vector<node_row> out;
  for (const auto &n : t.nodes)
    out.emplace_back(n.voxel, n.value, int(n.kind));
  return out;
}
std::set<std::pair<std::uint32_t, std::uint32_t>> arcs_of(const contour_tree &t) {
  std::set<std::pair<std::uint32_t, std::uint32_t>> out;
  for (const auto &a : t.arcs)
    out.emplace(a.hi, a.lo);
  return out;
}

void expect_same_tree(const contour_tree &a, const contour_tree &b, const std::string &what) {
  EXPECT_EQ(nodes_of(a), nodes_of(b)) << what;
  EXPECT_EQ(arcs_of(a), arcs_of(b)) << what;
  EXPECT_EQ(a.arcs.size(), b.arcs.size()) << what;
}

// The grid builder on the same values, as floats (what it reads), and the mesh on them.
void check_against_grid(int nx, int ny, int nz, unsigned seed, int levels) {
  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> q(0, levels - 1);
  std::uniform_real_distribution<float> r(-1.f, 1.f);
  const std::size_t n = std::size_t(nx) * ny * nz;
  std::vector<float> f(n);
  for (float &v : f)
    v = levels > 0 ? float(q(rng)) : r(rng);
  const contour_tree grid = cvc::compute_contour_tree(f.data(), nx, ny, nz);
  const auto mesh = field(grid_mesh(nx, ny, nz), std::vector<double>(f.begin(), f.end()));
  const contour_tree whole = cvc::compute_contour_tree(*mesh);
  const std::string what = std::to_string(nx) + "x" + std::to_string(ny) + "x" +
                           std::to_string(nz) + " seed " + std::to_string(seed) + " levels " +
                           std::to_string(levels);
  expect_same_tree(whole, grid, what);
  EXPECT_EQ(whole.dims[0], n);
  // Stepped in tiny budgets: the same tree.
  cvc::mesh_contour_tree_builder b(mesh);
  int steps = 0;
  while (!b.step(7))
    ++steps;
  EXPECT_GT(steps, 10) << what;
  expect_same_tree(b.take(), grid, what + " (stepped)");
}

TEST(MeshContourTree, MatchesTheGridBuilderOnTheGridsOwnTetrahedra) {
  for (unsigned seed = 1; seed <= 6; ++seed) {
    check_against_grid(7, 6, 5, seed, 0);
    check_against_grid(6, 5, 4, seed, 4); // plateaus: ties broken by point index
  }
}

TEST(MeshContourTree, MatchesTheGridBuilderOnTheGridsOwnTriangles) {
  for (unsigned seed = 1; seed <= 6; ++seed) {
    check_against_grid(11, 9, 1, seed, 0);
    check_against_grid(8, 8, 1, seed, 3);
  }
}

TEST(MeshContourTree, TwoBlobsSplitBelowTheirSaddle) {
  const int n = 21;
  std::vector<double> v;
  for (int k = 0; k < n; ++k)
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i) {
        const auto g = [&](double cx) {
          const double dx = i - cx, dy = j - 10.0, dz = k - 10.0;
          return std::exp(-(dx * dx + dy * dy + dz * dz) / 18.0);
        };
        v.push_back(g(5.0) + g(15.0));
      }
  const contour_tree t = cvc::compute_contour_tree(*field(grid_mesh(n, n, n), v));
  int maxima = 0, joins = 0;
  for (const auto &nd : t.nodes) {
    maxima += nd.kind == contour_tree::node_kind::maximum;
    joins += nd.kind == contour_tree::node_kind::join;
  }
  EXPECT_EQ(maxima, 2);
  EXPECT_EQ(joins, 1);
  EXPECT_EQ(t.components_at(0.9), 2u);
  EXPECT_EQ(t.components_at(0.05), 1u);
}

TEST(MeshContourTree, ARingStillGivesATree) {
  // An annulus of triangles around the origin, f = y: the level sets between the bottom and the
  // top are two arcs, a loop no tree holds. The forced leaf keeps it a tree.
  auto g = std::make_shared<geometry>();
  const int seg = 24;
  std::vector<double> v;
  for (int s = 0; s < seg; ++s)
    for (const double r : {1.0, 2.0}) {
      const double a = 2.0 * std::acos(-1.0) * s / seg + 0.01;
      g->points().push_back(geometry::point_t{{r * std::cos(a), r * std::sin(a), 0.0}});
      v.push_back(r * std::sin(a));
    }
  for (int s = 0; s < seg; ++s) {
    const auto in = geometry::index_t(2 * s), out = in + 1;
    const auto in2 = geometry::index_t(2 * ((s + 1) % seg)), out2 = in2 + 1;
    g->tris().push_back(geometry::tri_t{{in, out, out2}});
    g->tris().push_back(geometry::tri_t{{in, out2, in2}});
  }
  const contour_tree t = cvc::compute_contour_tree(*field(g, v));
  ASSERT_GE(t.nodes.size(), 2u);
  EXPECT_EQ(t.arcs.size() + 1, t.nodes.size()) << "one piece: a tree";
  EXPECT_GE(t.components_at(0.0), 1u);
  EXPECT_EQ(t.nodes.front().value, *std::min_element(v.begin(), v.end()));
  EXPECT_EQ(t.nodes.back().value, *std::max_element(v.begin(), v.end()));
}

TEST(MeshContourTree, RandomFieldsOnLoopyDomainsStillGiveTrees) {
  // A torus (a u x v grid wrapping both ways) and an annulus (wrapping one way), random values:
  // level sets that loop in every way a surface allows. Each must come out one tree, from the
  // global minimum to the global maximum, however it is stepped.
  const auto wrapped = [](int nu, int nv, bool wrap_v) {
    auto g = std::make_shared<geometry>();
    for (int j = 0; j < nv; ++j)
      for (int i = 0; i < nu; ++i)
        g->points().push_back(geometry::point_t{{double(i), double(j), 0.0}});
    const auto id = [&](int i, int j) { return geometry::index_t((j % nv) * nu + (i % nu)); };
    for (int j = 0; j < (wrap_v ? nv : nv - 1); ++j)
      for (int i = 0; i < nu; ++i) {
        g->tris().push_back(geometry::tri_t{{id(i, j), id(i + 1, j), id(i + 1, j + 1)}});
        g->tris().push_back(geometry::tri_t{{id(i, j), id(i, j + 1), id(i + 1, j + 1)}});
      }
    return g;
  };
  for (const bool torus : {true, false}) {
    const auto g = wrapped(9, 7, torus);
    for (unsigned seed = 1; seed <= 40; ++seed) {
      std::mt19937 rng(seed);
      std::uniform_int_distribution<int> q(0, seed % 2 ? 5 : 1000);
      std::vector<double> v;
      for (std::size_t i = 0; i < g->const_points().size(); ++i)
        v.push_back(double(q(rng)));
      const auto mesh = field(g, v);
      const std::string what =
          std::string(torus ? "torus" : "annulus") + " seed " + std::to_string(seed);
      contour_tree t;
      ASSERT_NO_THROW(t = cvc::compute_contour_tree(*mesh)) << what;
      ASSERT_GE(t.nodes.size(), 2u) << what;
      EXPECT_EQ(t.arcs.size() + 1, t.nodes.size()) << what << ": one piece, one tree";
      EXPECT_EQ(t.nodes.front().value, *std::min_element(v.begin(), v.end())) << what;
      EXPECT_EQ(t.nodes.back().value, *std::max_element(v.begin(), v.end())) << what;
      cvc::mesh_contour_tree_builder b(mesh);
      while (!b.step(5)) {
      }
      expect_same_tree(b.take(), t, what + " (stepped)");
    }
  }
}

TEST(MeshContourTree, SeparatePiecesGetATreeEach) {
  auto g = std::make_shared<geometry>();
  std::vector<double> v;
  for (int piece = 0; piece < 2; ++piece) {
    const auto b = geometry::index_t(g->points().size());
    for (int c = 0; c < 4; ++c) {
      g->points().push_back(geometry::point_t{{double(c & 1) + 5.0 * piece, double(c >> 1), 0.0}});
      v.push_back(double(c) + 0.1 * piece);
    }
    g->tris().push_back(geometry::tri_t{{b, b + 1, b + 3}});
    g->tris().push_back(geometry::tri_t{{b, b + 2, b + 3}});
  }
  const contour_tree t = cvc::compute_contour_tree(*field(g, v));
  EXPECT_EQ(t.arcs.size() + 2, t.nodes.size()) << "two pieces: two trees";
  EXPECT_EQ(t.components_at(1.5), 2u);
}

TEST(MeshContourTree, BadInputThrowsNaNPointsAreLeftOutAndAConstantFieldIsOneNode) {
  const auto g = grid_mesh(3, 3, 3);
  EXPECT_THROW(cvc::compute_contour_tree(*field(g, std::vector<double>(5, 0.0))),
               cvc::contour_tree_error);
  // A NaN point is left out with its edges (its neighbours still form a piece); all NaN throws.
  std::vector<double> v(27);
  for (std::size_t i = 0; i < v.size(); ++i)
    v[i] = double((i * 7) % 11);
  v[13] = std::nan("");
  contour_tree holed;
  ASSERT_NO_THROW(holed = cvc::compute_contour_tree(*field(g, v)));
  ASSERT_GE(holed.nodes.size(), 2u);
  for (const auto &nd : holed.nodes)
    EXPECT_NE(nd.voxel, 13u) << "the NaN point is no node";
  EXPECT_EQ(holed.arcs.size() + 1, holed.nodes.size());
  EXPECT_THROW(cvc::compute_contour_tree(*field(g, std::vector<double>(27, std::nan("")))),
               cvc::contour_tree_error);
  auto bad = std::make_shared<geometry>(*g);
  bad->tets().push_back(geometry::tet_t{{0, 1, 2, 99}});
  EXPECT_THROW(cvc::compute_contour_tree(*field(bad, std::vector<double>(27, 0.0))),
               cvc::contour_tree_error);
  auto empty = std::make_shared<geometry>();
  empty->points().push_back(geometry::point_t{{0.0, 0.0, 0.0}});
  EXPECT_THROW(cvc::compute_contour_tree(*field(empty, {0.0})), cvc::contour_tree_error);

  const contour_tree flat = cvc::compute_contour_tree(*field(g, std::vector<double>(27, 2.5)));
  ASSERT_EQ(flat.nodes.size(), 1u);
  EXPECT_TRUE(flat.arcs.empty());
  EXPECT_EQ(flat.nodes[0].value, 2.5);
}

// --- the spectrum -------------------------------------------------------------------------------

// f = a.x + b.y + c.z at every point.
std::vector<double> linear(const geometry &g, double a, double b, double c) {
  std::vector<double> v;
  for (const auto &p : g.const_points())
    v.push_back(a * p[0] + b * p[1] + c * p[2]);
  return v;
}

TEST(MeshContourSpectrum, ALinearFieldOnTheUnitCube) {
  // 3 cells a side, so the samples (tenths) never fall on a grid plane.
  const auto g = grid_mesh(4, 4, 4, 1.0 / 3.0);
  cvc::contour_spectrum_options opt;
  opt.samples = 11;
  const cvc::contour_spectrum s =
      cvc::compute_contour_spectrum(*field(g, linear(*g, 1, 0, 0)), opt);
  ASSERT_EQ(s.size(), 11u);
  EXPECT_DOUBLE_EQ(s.domain_min, 0.0);
  EXPECT_DOUBLE_EQ(s.domain_max, 1.0);
  EXPECT_NEAR(s.total_volume, 1.0, 1e-12);
  for (std::size_t b = 1; b + 1 < s.size(); ++b) {
    const double w = s.isovalue[b];
    // sort_verts separates equal corner values by (range / 4000) on the 255 scale: ~1e-3 here.
    EXPECT_NEAR(s.surface_area[b], 1.0, 2e-3) << w;
    EXPECT_NEAR(s.min_volume[b], w, 2e-3) << w;
    EXPECT_NEAR(s.max_volume[b], 1.0 - w, 2e-3) << w;
    EXPECT_NEAR(s.gradient[b], 1.0, 2e-3) << w;
  }
  // A slanted plane: the gradient is |grad f|^2 = 14 times the area, at every isovalue.
  const cvc::contour_spectrum t =
      cvc::compute_contour_spectrum(*field(g, linear(*g, 1, 2, 3)), opt);
  for (std::size_t b = 1; b + 1 < t.size(); ++b) {
    EXPECT_NEAR(t.gradient[b], 14.0 * t.surface_area[b], 1e-9 * t.gradient[b]) << b;
    EXPECT_LE(t.min_volume[b - 1], t.min_volume[b]);
  }
  EXPECT_NEAR(t.min_volume.back(), 1.0, 2e-3);
}

TEST(MeshContourSpectrum, ALinearFieldOnTheUnitSquareIsExact) {
  const auto g = grid_mesh(4, 4, 1, 1.0 / 3.0);
  cvc::contour_spectrum_options opt;
  opt.samples = 11;
  const cvc::contour_spectrum s =
      cvc::compute_contour_spectrum(*field(g, linear(*g, 1, 0, 0)), opt);
  EXPECT_NEAR(s.total_volume, 1.0, 1e-12) << "a surface's total is its area";
  for (std::size_t b = 1; b + 1 < s.size(); ++b) {
    const double w = s.isovalue[b];
    EXPECT_NEAR(s.surface_area[b], 1.0, 1e-12) << "the contour's length at " << w;
    EXPECT_NEAR(s.min_volume[b], w, 1e-12) << w;
    EXPECT_NEAR(s.max_volume[b], 1.0 - w, 1e-12) << w;
    EXPECT_NEAR(s.gradient[b], 1.0, 1e-12) << w;
  }
  // On a diagonal field the contours are the square's diagonals' parallels: length and area in
  // closed form (f = x + y on [0,1]^2, w in (0,1): length w*sqrt(2), area below w^2/2).
  const cvc::contour_spectrum d =
      cvc::compute_contour_spectrum(*field(g, linear(*g, 1, 1, 0)), opt);
  for (std::size_t b = 1; b + 1 < d.size(); ++b) {
    const double w = d.isovalue[b];
    if (w >= 1.0)
      break;
    EXPECT_NEAR(d.surface_area[b], w * std::sqrt(2.0), 1e-12) << w;
    EXPECT_NEAR(d.min_volume[b], 0.5 * w * w, 1e-12) << w;
    EXPECT_NEAR(d.gradient[b], 2.0 * w * std::sqrt(2.0), 1e-12) << w;
  }
}

TEST(MeshContourSpectrum, SteppingChangesNothing) {
  const auto g = grid_mesh(6, 5, 4, 0.5);
  std::vector<double> v;
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> r(-3.0, 5.0);
  for (std::size_t i = 0; i < g->const_points().size(); ++i)
    v.push_back(r(rng));
  const auto mesh = field(g, v);
  cvc::contour_spectrum_options opt;
  opt.samples = 64;
  const cvc::contour_spectrum once = cvc::compute_contour_spectrum(*mesh, opt);
  cvc::mesh_contour_spectrum_builder b(mesh, opt);
  int steps = 0;
  double last = 0.0;
  while (!b.step(13)) {
    ++steps;
    EXPECT_GE(b.progress(), last);
    last = b.progress();
  }
  EXPECT_GT(steps, 20);
  EXPECT_EQ(b.progress(), 1.0);
  const cvc::contour_spectrum stepped = b.take();
  EXPECT_EQ(stepped.surface_area, once.surface_area);
  EXPECT_EQ(stepped.min_volume, once.min_volume);
  EXPECT_EQ(stepped.max_volume, once.max_volume);
  EXPECT_EQ(stepped.gradient, once.gradient);
  EXPECT_THROW(b.take(), cvc::contour_spectrum_error) << "taken once";
}

TEST(MeshContourSpectrum, BadInputThrowsAndADegenerateDomainIsZero) {
  const auto g = grid_mesh(3, 3, 3);
  EXPECT_THROW(cvc::compute_contour_spectrum(*field(g, std::vector<double>(4, 0.0))),
               cvc::contour_spectrum_error);
  EXPECT_THROW(cvc::compute_contour_spectrum(
                   *field(g, std::vector<double>(27, std::numeric_limits<double>::infinity()))),
               cvc::contour_spectrum_error)
      << "no finite value";
  const cvc::contour_spectrum flat =
      cvc::compute_contour_spectrum(*field(g, std::vector<double>(27, 1.0)));
  EXPECT_NEAR(flat.total_volume, 8.0, 1e-12);
  for (double a : flat.surface_area)
    EXPECT_EQ(a, 0.0);
}

// --- overlays -----------------------------------------------------------------------------------

TEST(MeshOverlays, AJobMatchesTheOneShotLayersHoweverStepped) {
  const auto g = grid_mesh(9, 8, 7, 0.25);
  std::vector<double> v;
  for (const auto &p : g->const_points())
    v.push_back(std::sin(3.0 * p[0]) * std::cos(2.0 * p[1]) + p[2]);
  const auto mesh = field(g, v);
  cvc::tf::overlay_options opt;
  opt.histogram_bins = 32;
  opt.spectrum_samples = 32;
  const cvc::tf::overlays ref = cvc::tf::compute_overlays(*mesh, cvc::tf::overlay_all, opt);
  EXPECT_EQ(ref.status, "");
  ASSERT_TRUE(ref.has_range);
  EXPECT_EQ(ref.range_lo, *std::min_element(v.begin(), v.end()));
  EXPECT_EQ(ref.range_hi, *std::max_element(v.begin(), v.end()));
  ASSERT_EQ(ref.histogram.size(), 32u);
  double total = 0.0;
  for (double b : ref.histogram)
    total += b;
  EXPECT_EQ(total, double(v.size()));
  EXPECT_EQ(ref.spectrum.size(), 32u);
  EXPECT_FALSE(ref.tree.empty());

  cvc::tf::overlay_job job(mesh, cvc::tf::overlay_all, opt);
  std::set<std::string> phases;
  while (!job.step(50))
    phases.insert(job.phase());
  EXPECT_TRUE(phases.count("contour spectrum: integrating")) << "the mesh builder's phase names";
  EXPECT_TRUE(phases.count("contour tree: join tree"));
  const cvc::tf::overlays got = job.take();
  EXPECT_EQ(got.histogram, ref.histogram);
  EXPECT_EQ(got.spectrum.area, ref.spectrum.area);
  EXPECT_EQ(got.spectrum.gradient, ref.spectrum.gradient);
  ASSERT_EQ(got.tree.vertices.size(), ref.tree.vertices.size());
  EXPECT_EQ(got.tree.edges, ref.tree.edges);

  // A given range: the spectrum is sampled over it, and the range comes back as given.
  opt.has_range = true;
  opt.range_lo = 1.0;
  opt.range_hi = -0.5;
  const cvc::tf::overlays given =
      cvc::tf::compute_overlays(*mesh, cvc::tf::overlay_contour_spectrum, opt);
  EXPECT_EQ(given.range_lo, -0.5);
  EXPECT_EQ(given.range_hi, 1.0);
  EXPECT_EQ(given.spectrum.size(), 32u);
  EXPECT_TRUE(given.histogram.empty());
}

TEST(MeshOverlays, ATaskPublishesThemFromTheScheduler) {
  cvc::app app;
  const auto g = grid_mesh(8, 8, 8, 0.5);
  std::vector<double> v;
  for (const auto &p : g->const_points())
    v.push_back(p[0] * p[0] - p[1] + 0.5 * p[2]);
  const auto mesh = field(g, v);
  cvc::tf::overlay_task_options opt;
  opt.threading = cvc::tf::overlay_threading::cooperative;
  const unsigned bits = cvc::tf::overlay_contour_spectrum | cvc::tf::overlay_contour_tree;
  const cvc::tf::overlays ref = cvc::tf::compute_overlays(*mesh, bits, opt.overlays);
  const std::string tf = "mesh_test.field_colors";
  const int pid = cvc::tf::schedule_overlays(app, tf, mesh, bits, opt);
  ASSERT_GT(pid, 0);
  ASSERT_TRUE(cvc::tf::wait_overlays(app, pid, 60.0));
  const cvc::tf::overlays got = cvc::tf::read_overlays(cvc::state::instance(app)(tf));
  EXPECT_EQ(got.spectrum.area.size(), ref.spectrum.area.size());
  EXPECT_EQ(got.tree.edges, ref.tree.edges);
  EXPECT_EQ(cvc::state::instance(app)(tf)("overlay_status").value(), "");
}

} // namespace
