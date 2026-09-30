// lod_pyramid_test -- the per-asset LOD ladder builders (cvc::lod::build_*_pyramid).
//
// Covers: a mesh ladder that strictly coarsens with a monotone world-error ladder
// and a verbatim rung 0; its single progressive pass reproduces an independent
// cvc::simplify per rung bit-for-bit; its error is metric (a 10x mesh has a 10x
// ladder); a flat-shaded (split-face) mesh coarsens like the welded sheet unless
// welding is turned off; a planar-projected texture stays exactly on its
// positions, with no orphaned vertices, in every rung of a flat-shaded sheet
// and a triangle soup; a rung the collapse cannot reach, or one a single
// collapse jumped past, is left out, not repeated, and the ladder goes on past
// it; a snapshot 5% coarser than the rung before is kept, one ~3% coarser is
// left out; a faceted curved mesh (per-face normals) cannot coarsen with its normals
// carried -- the rung that only erodes its border is left out -- and builds a
// full ladder with recompute_normals; pooled == serial; a volume ladder that
// halves each axis with a
// growing voxel-size error; and an image mip ladder whose box filter actually
// area-averages (a 2x2 checker becomes mid-grey) rather than point-samples.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/core/thread_pool.h>
#include <cvc/core/types.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/simplify.h>
#include <cvc/image/image.h>
#include <cvc/lod/pyramid.h>
#include <cvc/volume/volume.h>
#include <gtest/gtest.h>

using cvc::geometry;
using cvc::image;
using cvc::volume;

namespace {

geometry bumpy_grid(cvc::app &ctx, int n) {
  geometry g(ctx);
  geometry::points_t &P = g.points();
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j)
      P.push_back({double(i), double(j), 1.4 * std::sin(0.6 * i) * std::cos(0.8 * j)});
  geometry::tris_t &T = g.tris();
  auto id = [n](int i, int j) { return std::uint64_t(i * n + j); };
  for (int i = 0; i < n - 1; ++i)
    for (int j = 0; j < n - 1; ++j) {
      T.push_back({id(i, j), id(i + 1, j), id(i, j + 1)});
      T.push_back({id(i + 1, j), id(i + 1, j + 1), id(i, j + 1)});
    }
  g.set_geometry_type(geometry::SURFACE_TRI);
  return g;
}

template <class V> bool bitwise_equal(const V &a, const V &b) {
  if (a.size() != b.size())
    return false;
  return a.empty() || std::memcmp(a.data(), b.data(), a.size() * sizeof(a[0])) == 0;
}

bool same_mesh(const geometry &a, const geometry &b) {
  return bitwise_equal(a.const_points(), b.const_points()) &&
         bitwise_equal(a.const_tris(), b.const_tris()) &&
         bitwise_equal(a.const_normals(), b.const_normals());
}

// `g` with its vertices split per quad (make_grid order: triangle pairs), the
// way a flat-shaded export stores hard edges.
geometry flat_shaded(const geometry &g) {
  geometry s(g.ctx());
  const geometry::tris_t &T = g.const_tris();
  for (std::size_t q = 0; q + 1 < T.size(); q += 2) {
    const std::uint64_t o = s.points().size();
    const std::uint64_t id[4] = {T[q][0], T[q][1], T[q][2], T[q + 1][1]};
    for (std::uint64_t v : id)
      s.points().push_back(g.const_points()[v]);
    s.tris().push_back({o, o + 1, o + 2});
    s.tris().push_back({o + 1, o + 3, o + 2});
  }
  s.set_geometry_type(geometry::SURFACE_TRI);
  return s;
}

// `g` as a triangle soup: every triangle owns copies of its vertices.
geometry soup(const geometry &g) {
  geometry s(g.ctx());
  for (const auto &t : g.const_tris()) {
    const std::uint64_t o = s.points().size();
    for (int k = 0; k < 3; ++k)
      s.points().push_back(g.const_points()[t[k]]);
    s.tris().push_back({o, o + 1, o + 2});
  }
  s.set_geometry_type(geometry::SURFACE_TRI);
  return s;
}

// uv = (x, y) of each position: a planar-projected texture.
geometry with_planar_uv(geometry g) {
  g.uvs().clear();
  for (const auto &p : g.const_points())
    g.uvs().push_back({p[0], p[1]});
  return g;
}

// A ladder's error column must be monotone non-decreasing and start at 0.
void expect_monotone(const std::vector<double> &e) {
  ASSERT_FALSE(e.empty());
  EXPECT_EQ(e.front(), 0.0);
  for (std::size_t k = 1; k < e.size(); ++k)
    EXPECT_GE(e[k], e[k - 1]) << "rung " << k;
}

} // namespace

TEST(LodPyramid, MeshLadderCoarsens) {
  cvc::app ctx;
  geometry src = bumpy_grid(ctx, 30);
  cvc::lod::pyramid_params pp;
  pp.max_rungs = 4;
  cvc::lod::mesh_pyramid pyr = cvc::lod::build_mesh_pyramid(src, pp);

  ASSERT_GE(pyr.rungs.size(), 2u);
  EXPECT_EQ(pyr.rungs.size(), pyr.world_error_m.size());
  EXPECT_EQ(pyr.rungs[0].num_tris(), src.num_tris()); // rung 0 is the source, verbatim
  expect_monotone(pyr.world_error_m);
  // Triangle count strictly decreases down the ladder.
  for (std::size_t k = 1; k < pyr.rungs.size(); ++k)
    EXPECT_LT(pyr.rungs[k].num_tris(), pyr.rungs[k - 1].num_tris()) << "rung " << k;
}

TEST(LodPyramid, MeshLadderPoolMatchesSerial) {
  cvc::app ctx;
  geometry src = bumpy_grid(ctx, 44);
  cvc::lod::pyramid_params pp;
  pp.max_rungs = 4;
  cvc::lod::mesh_pyramid s = cvc::lod::build_mesh_pyramid(src, pp, nullptr);
  cvc::lod::mesh_pyramid p = cvc::lod::build_mesh_pyramid(src, pp, &ctx.computePool());
  ASSERT_EQ(s.rungs.size(), p.rungs.size());
  for (std::size_t k = 0; k < s.rungs.size(); ++k) {
    EXPECT_EQ(s.rungs[k].num_tris(), p.rungs[k].num_tris()) << "rung " << k;
    EXPECT_TRUE(same_mesh(s.rungs[k], p.rungs[k])) << "rung " << k;
    EXPECT_EQ(s.world_error_m[k], p.world_error_m[k]) << "rung " << k;
  }
}

// The ladder is ONE progressive pass, yet every rung is exactly what an
// independent simplify() of the source to that rung's target produces, and its
// error is that run's world_error (under the running max).
TEST(LodPyramid, MeshLadderMatchesIndependentSimplify) {
  cvc::app ctx;
  geometry src = bumpy_grid(ctx, 40);
  src.compute_normals(); // carried through every rung
  cvc::lod::pyramid_params pp;
  pp.max_rungs = 4;
  pp.mesh_ratio = 0.4;
  cvc::lod::mesh_pyramid pyr = cvc::lod::build_mesh_pyramid(src, pp);
  ASSERT_EQ(pyr.rungs.size(), 5u);
  double ratio = 1.0, prev = 0.0;
  for (std::size_t k = 1; k < pyr.rungs.size(); ++k) {
    ratio *= pp.mesh_ratio;
    cvc::simplify_params sp;
    sp.target_tris = std::uint64_t(std::llround(double(src.num_tris()) * ratio));
    sp.preserve_boundary = pp.preserve_boundary;
    cvc::simplify_result r;
    const geometry one = cvc::simplify(src, sp, &r);
    EXPECT_TRUE(same_mesh(pyr.rungs[k], one)) << "rung " << k;
    prev = std::max(prev, r.world_error);
    EXPECT_EQ(pyr.world_error_m[k], prev) << "rung " << k;
  }
}

// A flat-shaded mesh (every quad its own part, touching its neighbours only
// through coincident vertices) builds a full ladder that coarsens like the
// welded sheet; with pyramid_params::weld_seams off each quad decimates on its
// own, tearing the sheet for a larger error.
TEST(LodPyramid, MeshLadderCoarsensSplitFaces) {
  cvc::app ctx;
  const geometry src = flat_shaded(bumpy_grid(ctx, 60)); // 6962 tris: targets 2437 .. 104
  cvc::lod::pyramid_params pp;
  pp.max_rungs = 4;
  const cvc::lod::mesh_pyramid welded = cvc::lod::build_mesh_pyramid(src, pp);
  ASSERT_EQ(welded.rungs.size(), 5u);
  expect_monotone(welded.world_error_m);
  for (std::size_t k = 1; k < welded.rungs.size(); ++k)
    EXPECT_LE(welded.rungs[k].num_tris(),
              std::uint64_t(std::llround(double(src.num_tris()) * std::pow(pp.mesh_ratio, k))))
        << "rung " << k;
  pp.weld_seams = false;
  const cvc::lod::mesh_pyramid torn = cvc::lod::build_mesh_pyramid(src, pp);
  ASSERT_GE(torn.rungs.size(), 2u);
  EXPECT_GT(torn.world_error_m[1], welded.world_error_m[1]);
}

// Every rung of a flat-shaded sheet and of a triangle soup, both carrying a
// planar-projected texture (uv = xy, continuous across every split), keeps each
// corner's uv exactly on its position -- no uv is re-attached to another
// position, so no rung shears the texture -- and holds only vertices its
// triangles use.
TEST(LodPyramid, MeshLadderKeepsPlanarUvOnItsPositions) {
  cvc::app ctx;
  const geometry sheet = bumpy_grid(ctx, 40);
  for (int kind = 0; kind < 2; ++kind) {
    const geometry src = with_planar_uv(kind == 0 ? flat_shaded(sheet) : soup(sheet));
    cvc::lod::pyramid_params pp;
    pp.max_rungs = 4;
    const cvc::lod::mesh_pyramid pyr = cvc::lod::build_mesh_pyramid(src, pp, &ctx.computePool());
    ASSERT_EQ(pyr.rungs.size(), 4u) << (kind == 0 ? "flat-shaded" : "soup"); // targets 1065 .. 130
    for (std::size_t k = 0; k < pyr.rungs.size(); ++k) {
      SCOPED_TRACE(testing::Message() << (kind == 0 ? "flat-shaded" : "soup") << " rung " << k);
      const geometry &r = pyr.rungs[k];
      ASSERT_EQ(r.const_uvs().size(), r.num_points());
      std::vector<char> used(r.num_points(), 0);
      int sheared = 0;
      for (const auto &t : r.const_tris())
        for (int c = 0; c < 3; ++c) {
          used[t[c]] = 1;
          const auto &p = r.const_points()[t[c]];
          const auto &uv = r.const_uvs()[t[c]];
          sheared += uv[0] != p[0] || uv[1] != p[1] ? 1 : 0;
        }
      EXPECT_EQ(sheared, 0);
      EXPECT_EQ(std::count(used.begin(), used.end(), 0), 0) << "points no triangle uses";
      if (k > 0)
        EXPECT_LT(r.num_tris(), pyr.rungs[k - 1].num_tris());
    }
  }
}

// A rung whose target the collapse cannot reach (every remaining collapse is
// guarded) would repeat the rung before it: it is left out. A tetrahedron
// collapses once, to a two-sided triangle, and then stops -- the last triangle
// never goes -- so of the targets 2, 1, 1 only the first yields a rung.
TEST(LodPyramid, MeshLadderLeavesOutStalledRungs) {
  cvc::app ctx;
  geometry tet(ctx);
  tet.points() = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  tet.tris() = {{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}};
  tet.set_geometry_type(geometry::SURFACE_TRI);
  cvc::lod::pyramid_params pp;
  pp.max_rungs = 3;
  pp.mesh_ratio = 0.6;
  pp.mesh_min_tris = 1;
  const cvc::lod::mesh_pyramid pyr = cvc::lod::build_mesh_pyramid(tet, pp);
  ASSERT_EQ(pyr.rungs.size(), 2u);
  EXPECT_EQ(pyr.rungs[1].num_tris(), 2u);
  EXPECT_GT(pyr.world_error_m[1], 0.0);
  EXPECT_TRUE(std::isfinite(pyr.world_error_m[1]));
}

// Close targets can fall within one collapse: on this sheet a single collapse
// takes the count from 96 to 94, past the targets 95 and 94, so those two
// snapshots are equal. The repeat is left out, yet the ladder goes on to the
// targets beyond it, each rung matching an independent simplify() to its own
// target.
TEST(LodPyramid, MeshLadderSkipsARepeatAndGoesOn) {
  cvc::app ctx;
  const geometry src = bumpy_grid(ctx, 8); // 98 tris: targets 97 96 95 94 93 92
  cvc::lod::pyramid_params pp;
  pp.max_rungs = 6;
  pp.mesh_ratio = 0.99;
  pp.mesh_min_tris = 1;
  const cvc::lod::mesh_pyramid pyr = cvc::lod::build_mesh_pyramid(src, pp);
  std::vector<std::uint64_t> counts;
  for (const geometry &r : pyr.rungs)
    counts.push_back(r.num_tris());
  ASSERT_EQ(counts, (std::vector<std::uint64_t>{98, 97, 96, 94, 93, 92}));
  expect_monotone(pyr.world_error_m);
  for (std::size_t k = 1; k < pyr.rungs.size(); ++k) {
    cvc::simplify_params sp;
    sp.target_tris = counts[k];
    EXPECT_TRUE(same_mesh(pyr.rungs[k], cvc::simplify(src, sp))) << "rung " << k;
  }
}

// A faceted curved mesh -- a bumpy sheet exported with per-face normals, so
// its attributes jump at every vertex -- can only give up its open border
// while its normals are carried: that barely coarser rung is not worth a
// switch and is left out, so the ladder is rung 0 alone. With
// recompute_normals the input normals no longer split its vertices: it builds
// a full ladder, every rung at least 5% coarser than the one before and equal to
// an independent simplify() with the same switch.
TEST(LodPyramid, MeshLadderRecomputesNormalsOfAFacetedMesh) {
  cvc::app ctx;
  const geometry sheet = soup(bumpy_grid(ctx, 40));
  geometry src = sheet;
  for (std::size_t f = 0; f < src.num_tris(); ++f) {
    const auto &t = src.const_tris()[f];
    const auto &a = src.const_points()[t[0]], &b = src.const_points()[t[1]],
               &c = src.const_points()[t[2]];
    const double e[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
    const double g[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
    const double n[3] = {e[1] * g[2] - e[2] * g[1], e[2] * g[0] - e[0] * g[2],
                         e[0] * g[1] - e[1] * g[0]};
    const double l = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    src.normals().insert(src.normals().end(), 3, {n[0] / l, n[1] / l, n[2] / l});
  }
  cvc::lod::pyramid_params pp;
  pp.max_rungs = 4;
  const cvc::lod::mesh_pyramid carried = cvc::lod::build_mesh_pyramid(src, pp);
  EXPECT_EQ(carried.rungs.size(), 1u);
  cvc::simplify_params sp;
  sp.target_tris = std::uint64_t(std::llround(0.35 * double(src.num_tris())));
  cvc::simplify_result r;
  cvc::simplify(src, sp, &r);
  EXPECT_GT(r.out_tris, src.num_tris() * 97 / 100) << "only the border can go";

  pp.recompute_normals = true;
  const cvc::lod::mesh_pyramid fresh = cvc::lod::build_mesh_pyramid(src, pp);
  ASSERT_EQ(fresh.rungs.size(), 4u); // targets 1065 .. 130
  expect_monotone(fresh.world_error_m);
  for (std::size_t k = 1; k < fresh.rungs.size(); ++k) {
    EXPECT_LE(fresh.rungs[k].num_tris() * 100, fresh.rungs[k - 1].num_tris() * 95) << "rung " << k;
    sp.target_tris = fresh.rungs[k].num_tris();
    sp.recompute_normals = true;
    EXPECT_TRUE(same_mesh(fresh.rungs[k], cvc::simplify(src, sp))) << "rung " << k;
  }
}

// A snapshot 5% or more coarser than the rung before it is worth a switch and
// is kept, even where the pass then stalls, and one less coarse is not: a flat
// 7 x 7 sheet collapses to two triangles beside closed tetrahedra colored per
// face, which cannot give up a triangle. Beside 232 of them the first snapshot
// is ~7% coarser than the source and is kept; beside 500, ~3%, and it is left
// out. The targets past it repeat it and are left out.
TEST(LodPyramid, MeshLadderKeepsOnlyRungsFivePercentCoarser) {
  cvc::app ctx;
  for (int tets : {232, 500}) {
    SCOPED_TRACE(testing::Message() << tets << " tetrahedra");
    geometry src = bumpy_grid(ctx, 7);
    for (auto &p : src.points())
      p[2] = 0.0;
    src.colors().assign(src.num_points(), {0.5, 0.5, 0.5});
    for (int k = 0; k < tets; ++k) {
      const double x = 10.0 + 2.0 * (k % 16), y = 2.0 * (k / 16);
      const geometry::point_t c[4] = {{x, y, 0}, {x + 1, y, 0}, {x, y + 1, 0}, {x, y, 1}};
      const int face[4][3] = {{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}};
      for (int f = 0; f < 4; ++f) {
        const std::uint64_t o = src.num_points();
        for (int v : face[f]) {
          src.points().push_back(c[v]);
          src.colors().push_back({double(f), double(k), 0.0});
        }
        src.tris().push_back({o, o + 1, o + 2});
      }
    }
    const std::uint64_t n = src.num_tris();
    cvc::simplify_params sp;
    sp.target_tris = std::uint64_t(std::llround(double(n) * 0.35));
    sp.preserve_boundary = true;
    const geometry first = cvc::simplify(src, sp);
    const double removed = 1.0 - double(first.num_tris()) / double(n);
    ASSERT_GT(removed, tets == 232 ? 0.05 : 0.02);
    ASSERT_LT(removed, tets == 232 ? 0.10 : 0.05);
    cvc::lod::pyramid_params pp; // mesh_ratio 0.35
    const cvc::lod::mesh_pyramid pyr = cvc::lod::build_mesh_pyramid(src, pp);
    ASSERT_EQ(pyr.rungs.size(), tets == 232 ? 2u : 1u);
    if (tets == 232)
      EXPECT_TRUE(same_mesh(pyr.rungs[1], first));
  }
}

// world_error_m is a length: the same terrain at 10x scale has a 10x ladder.
TEST(LodPyramid, MeshLadderErrorScalesWithTheMesh) {
  cvc::app ctx;
  geometry src = bumpy_grid(ctx, 30), big = bumpy_grid(ctx, 30);
  for (auto &p : big.points())
    for (int k = 0; k < 3; ++k)
      p[k] *= 10.0;
  cvc::lod::pyramid_params pp;
  pp.max_rungs = 3;
  cvc::lod::mesh_pyramid a = cvc::lod::build_mesh_pyramid(src, pp);
  cvc::lod::mesh_pyramid b = cvc::lod::build_mesh_pyramid(big, pp);
  ASSERT_EQ(a.rungs.size(), b.rungs.size());
  ASSERT_GE(a.rungs.size(), 3u);
  for (std::size_t k = 1; k < a.rungs.size(); ++k) {
    ASSERT_GT(a.world_error_m[k], 1e-6) << "rung " << k;
    EXPECT_NEAR(b.world_error_m[k] / (10.0 * a.world_error_m[k]), 1.0, 0.05) << "rung " << k;
  }
}

TEST(LodPyramid, VolumeLadderHalves) {
  cvc::app ctx;
  volume src(ctx, cvc::dimension(64, 64, 64), cvc::Float, cvc::bounding_box(0, 0, 0, 1, 1, 1));
  cvc::lod::pyramid_params pp;
  pp.max_rungs = 3;
  pp.vol_min_dim = 4;
  cvc::lod::volume_pyramid pyr = cvc::lod::build_volume_pyramid(src, pp);

  ASSERT_GE(pyr.rungs.size(), 2u);
  EXPECT_EQ(pyr.rungs[0].XDim(), 64u); // rung 0 is the source
  expect_monotone(pyr.world_error_m);
  for (std::size_t k = 1; k < pyr.rungs.size(); ++k) {
    EXPECT_LT(pyr.rungs[k].XDim(), pyr.rungs[k - 1].XDim()) << "rung " << k;
    EXPECT_GT(pyr.world_error_m[k], 0.0);
  }
}

TEST(LodPyramid, ImageMipBoxAverages) {
  cvc::app ctx;
  // 4x4 GRAY checker of 0/255 in 2x2 super-cells; a box downscale to 2x2 must
  // average each 2x2 block to ~128 (nearest would keep 0/255).
  image src(4, 4, image::pixel_format::GRAY, image::data_type::u8);
  unsigned char *d = src.data();
  for (int y = 0; y < 4; ++y)
    for (int x = 0; x < 4; ++x)
      d[y * 4 + x] = ((x / 1 + y / 1) % 2) ? 255 : 0; // fine checker

  cvc::lod::pyramid_params pp;
  pp.max_rungs = 2;
  pp.img_min_dim = 1;
  cvc::lod::image_pyramid pyr = cvc::lod::build_image_pyramid(src, pp);

  ASSERT_GE(pyr.rungs.size(), 2u);
  EXPECT_EQ(pyr.rungs[0].width(), 4); // rung 0 = source
  expect_monotone(pyr.world_error_m);
  // rung 1 is 2x2; each pixel is a 2x2 box average of the checker => ~128.
  const image &r1 = pyr.rungs[1];
  ASSERT_EQ(r1.width(), 2);
  ASSERT_EQ(r1.height(), 2);
  const unsigned char *p = r1.data();
  for (int i = 0; i < 4; ++i)
    EXPECT_NEAR(int(p[i]), 128, 2) << "box average at " << i;
  for (std::size_t k = 1; k < pyr.rungs.size(); ++k)
    EXPECT_LT(pyr.rungs[k].width(), pyr.rungs[k - 1].width());
}
