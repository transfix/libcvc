// geometry_simplify_test -- the QEM half-edge decimator (cvc::simplify).
//
// Covers: a flat sheet collapses to near-zero triangles at ~zero world error; a
// bumpy sheet reduces toward the triangle target without NaNs, degenerate faces,
// or triangle-count growth; the boundary and the bounding box are preserved; the
// pooled setup is bit-identical to the serial setup; and a no-op target returns
// the input untouched.

#include <cmath>
#include <cstdint>
#include <cvc/core/app.h>
#include <cvc/core/thread_pool.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/simplify.h>
#include <gtest/gtest.h>
#include <set>
#include <vector>

using cvc::geometry;

namespace {

// n x n vertex grid in the z=0 plane, triangulated (2*(n-1)^2 tris). If `amp`>0,
// displace z by amp*sin*cos so the sheet is genuinely non-planar.
geometry make_grid(cvc::app &ctx, int n, double amp) {
  geometry g(ctx);
  geometry::points_t &P = g.points();
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) {
      double z = amp * std::sin(0.7 * i) * std::cos(0.9 * j);
      P.push_back({double(i), double(j), z});
    }
  geometry::tris_t &T = g.tris();
  auto idx = [n](int i, int j) { return std::uint64_t(i * n + j); };
  for (int i = 0; i < n - 1; ++i)
    for (int j = 0; j < n - 1; ++j) {
      T.push_back({idx(i, j), idx(i + 1, j), idx(i, j + 1)});
      T.push_back({idx(i + 1, j), idx(i + 1, j + 1), idx(i, j + 1)});
    }
  g.set_geometry_type(geometry::SURFACE_TRI);
  return g;
}

bool finite_pt(const geometry::point_t &p) {
  return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}

// A mesh is well-formed if every position is finite and no triangle repeats an
// index or points out of range.
void expect_wellformed(const geometry &g) {
  for (const auto &p : g.const_points())
    ASSERT_TRUE(finite_pt(p));
  const auto np = g.const_points().size();
  for (const auto &t : g.const_tris()) {
    EXPECT_LT(t[0], np);
    EXPECT_LT(t[1], np);
    EXPECT_LT(t[2], np);
    EXPECT_TRUE(t[0] != t[1] && t[1] != t[2] && t[0] != t[2]) << "degenerate triangle";
  }
}

} // namespace

TEST(GeometrySimplify, FlatSheetCollapsesAtZeroError) {
  cvc::app ctx;
  geometry g = make_grid(ctx, 12, 0.0); // planar
  cvc::simplify_params p;
  p.target_ratio = 0.05;
  cvc::simplify_result r;
  geometry s = cvc::simplify(g, p, &r);
  expect_wellformed(s);
  EXPECT_LT(s.num_tris(), g.num_tris());
  // Every collapse on a plane is free: the error must be ~0.
  EXPECT_NEAR(r.world_error, 0.0, 1e-6);
  // A plane can decimate hard (down toward a couple of triangles).
  EXPECT_LE(s.num_tris(), g.num_tris() / 4);
}

TEST(GeometrySimplify, BumpySheetReducesCleanly) {
  cvc::app ctx;
  geometry g = make_grid(ctx, 24, 1.5); // non-planar
  const std::uint64_t in = g.num_tris();
  cvc::simplify_params p;
  p.target_ratio = 0.25;
  cvc::simplify_result r;
  geometry s = cvc::simplify(g, p, &r);
  expect_wellformed(s);
  EXPECT_EQ(r.in_tris, in);
  EXPECT_EQ(r.out_tris, s.num_tris());
  EXPECT_LT(s.num_tris(), in);      // it actually reduced
  EXPECT_GT(s.num_tris(), in / 20); // but did not annihilate the surface
  EXPECT_GT(r.world_error, 0.0);    // a curved sheet pays real error
  EXPECT_GT(r.collapses, 0u);
}

TEST(GeometrySimplify, PreservesBoundingBox) {
  cvc::app ctx;
  geometry g = make_grid(ctx, 20, 1.0);
  cvc::bounding_box b0 = g.extents();
  cvc::simplify_params p;
  p.target_ratio = 0.1;
  p.preserve_boundary = true;
  geometry s = cvc::simplify(g, p);
  cvc::bounding_box b1 = s.extents();
  // Boundary preservation keeps the border vertices, so the XY extent is intact.
  EXPECT_NEAR(b0.minx, b1.minx, 1e-9);
  EXPECT_NEAR(b0.miny, b1.miny, 1e-9);
  EXPECT_NEAR(b0.maxx, b1.maxx, 1e-9);
  EXPECT_NEAR(b0.maxy, b1.maxy, 1e-9);
}

TEST(GeometrySimplify, PoolMatchesSerialBitForBit) {
  cvc::app ctx;
  geometry g = make_grid(ctx, 40, 2.0); // ~3000 tris -> the pool actually fans out
  cvc::simplify_params p;
  p.target_ratio = 0.3;
  cvc::simplify_result rs, rp;
  geometry serial = cvc::simplify(g, p, &rs, nullptr);
  geometry pooled = cvc::simplify(g, p, &rp, &ctx.computePool());

  ASSERT_EQ(serial.num_points(), pooled.num_points());
  ASSERT_EQ(serial.num_tris(), pooled.num_tris());
  EXPECT_EQ(rs.out_tris, rp.out_tris);
  EXPECT_DOUBLE_EQ(rs.world_error, rp.world_error);
  for (std::size_t i = 0; i < serial.const_points().size(); ++i)
    for (int k = 0; k < 3; ++k)
      EXPECT_DOUBLE_EQ(serial.const_points()[i][k], pooled.const_points()[i][k]);
  for (std::size_t i = 0; i < serial.const_tris().size(); ++i)
    for (int k = 0; k < 3; ++k)
      EXPECT_EQ(serial.const_tris()[i][k], pooled.const_tris()[i][k]);
}

TEST(GeometrySimplify, NoOpWhenTargetAboveInput) {
  cvc::app ctx;
  geometry g = make_grid(ctx, 10, 1.0);
  cvc::simplify_params p;
  p.target_ratio = 1.0; // ask for everything
  cvc::simplify_result r;
  geometry s = cvc::simplify(g, p, &r);
  EXPECT_EQ(s.num_tris(), g.num_tris());
  EXPECT_EQ(r.collapses, 0u);
  EXPECT_EQ(r.world_error, 0.0);

  cvc::simplify_params p2;
  p2.target_tris = g.num_tris() + 100; // explicit over-target
  geometry s2 = cvc::simplify(g, p2);
  EXPECT_EQ(s2.num_tris(), g.num_tris());
}

TEST(GeometrySimplify, EmptyMeshIsSafe) {
  cvc::app ctx;
  geometry g(ctx);
  cvc::simplify_result r;
  geometry s = cvc::simplify(g, cvc::simplify_params(), &r);
  EXPECT_EQ(s.num_tris(), 0u);
  EXPECT_EQ(r.in_tris, 0u);
}

TEST(GeometrySimplify, ErrorBudgetStopsEarly) {
  cvc::app ctx;
  geometry g = make_grid(ctx, 24, 1.5);
  cvc::simplify_params p;
  p.target_tris = 2;  // ask to go all the way down...
  p.max_error = 0.05; // ...but cap the world error
  cvc::simplify_result r;
  geometry s = cvc::simplify(g, p, &r);
  expect_wellformed(s);
  EXPECT_TRUE(r.hit_error_limit);
  EXPECT_GT(s.num_tris(), 2u); // the error cap kept it above the tri target
  EXPECT_LE(r.world_error, 0.05 + 1e-9);
}
