/*
  Copyright 2026 The University of Texas at Austin

  Unit tests for cvc/vis/types.h -- the frustum construction and the box/sphere
  intersection tests every cvc::vis culler is built on.

  The load-bearing properties:
    * from_vtk_planes REORDERS vtk's L,R,B,T,FAR,NEAR into L,R,B,T,NEAR,FAR --
      the near/far swap that is silent if botched;
    * from_view_proj (Gribb-Hartmann) yields a frustum that classifies points
      identically to a hand-built box, independent of plane order;
    * aabb_vs_frustum reports INSIDE distinctly (the subtree short-circuit) and
      clears satisfied planes from the mask (the descent optimization);
    * the standard conservatism holds: never OUTSIDE for a box the frustum
      actually touches.
*/

#include <cmath>
#include <cstdint>
#include <cvc/vis/types.h>
#include <gtest/gtest.h>
#include <random>
#include <vector>

using namespace cvc::vis;

namespace {

// A frustum that bounds an axis-aligned box, six inward planes. Plane ORDER is
// irrelevant to the box/sphere tests (they sweep all six), so this is a clean
// oracle for them.
frustum box_frustum(float minx, float maxx, float miny, float maxy, float minz, float maxz) {
  frustum f;
  f.p[0] = {{1, 0, 0}, -minx}; // x >= minx
  f.p[1] = {{-1, 0, 0}, maxx}; // x <= maxx
  f.p[2] = {{0, 1, 0}, -miny}; // y >= miny
  f.p[3] = {{0, -1, 0}, maxy}; // y <= maxy
  f.p[4] = {{0, 0, 1}, -minz}; // z >= minz
  f.p[5] = {{0, 0, -1}, maxz}; // z <= maxz
  return f;
}

bool inside_all(const frustum &f, float x, float y, float z) {
  const float p[3] = {x, y, z};
  for (int i = 0; i < 6; ++i)
    if (plane_distance(f.p[i], p) < 0.0f)
      return false;
  return true;
}

aabb box(float x0, float y0, float z0, float x1, float y1, float z1) {
  return {{x0, y0, z0}, {x1, y1, z1}};
}

} // namespace

// --- Frustum construction --------------------------------------------------

TEST(VisFrustum, FromVtkPlanesSwapsNearAndFar) {
  // Six already-unit planes so normalization is identity and `d` is checkable.
  // vtk order is L, R, B, T, FAR, NEAR -- tag each `d` with its vtk index.
  double p24[24] = {
      1, 0, 0, 10, // 0 L
      0, 1, 0, 11, // 1 R
      0, 0, 1, 12, // 2 B
      1, 0, 0, 13, // 3 T
      0, 1, 0, 14, // 4 FAR
      0, 0, 1, 15, // 5 NEAR
  };
  const frustum f = frustum::from_vtk_planes(p24);
  EXPECT_FLOAT_EQ(f.p[0].d, 10.0f); // L unchanged
  EXPECT_FLOAT_EQ(f.p[1].d, 11.0f); // R
  EXPECT_FLOAT_EQ(f.p[2].d, 12.0f); // B
  EXPECT_FLOAT_EQ(f.p[3].d, 13.0f); // T
  EXPECT_FLOAT_EQ(f.p[4].d, 15.0f); // our NEAR <- vtk index 5
  EXPECT_FLOAT_EQ(f.p[5].d, 14.0f); // our FAR  <- vtk index 4
}

TEST(VisFrustum, FromVtkPlanesNormalizes) {
  double p24[24] = {0};
  p24[0] = 0.0;
  p24[1] = 0.0;
  p24[2] = 5.0; // a length-5 normal along z
  p24[3] = 20.0;
  const frustum f = frustum::from_vtk_planes(p24);
  EXPECT_NEAR(
      std::sqrt(f.p[0].n[0] * f.p[0].n[0] + f.p[0].n[1] * f.p[0].n[1] + f.p[0].n[2] * f.p[0].n[2]),
      1.0f, 1e-6f);
  EXPECT_NEAR(f.p[0].n[2], 1.0f, 1e-6f);
  EXPECT_NEAR(f.p[0].d, 4.0f, 1e-6f); // 20 / 5
}

TEST(VisFrustum, FromViewProjMatchesHandBuiltBox) {
  // Symmetric orthographic projection, row-major M = P (V = identity). It maps
  // the world box x in [-r,r], y in [-t,t], z in [-f,-n] to clip. See the test
  // comment; only the resulting VOLUME matters here, not the plane labels.
  const double r = 3.0, t = 2.0, n = 1.0, fz = 20.0;
  const double M[16] = {
      1.0 / r,
      0,
      0,
      0, // row 0
      0,
      1.0 / t,
      0,
      0, // row 1
      0,
      0,
      -2.0 / (fz - n),
      -(fz + n) / (fz - n), // row 2
      0,
      0,
      0,
      1, // row 3
  };
  const frustum gh = frustum::from_view_proj(M);
  const frustum bx = box_frustum(-3, 3, -2, 2, -20, -1);

  std::mt19937 rng(20260829);
  std::uniform_real_distribution<float> U(-25.0f, 25.0f);
  int agree = 0;
  for (int i = 0; i < 20000; ++i) {
    const float x = U(rng), y = U(rng), z = U(rng);
    // Nudge off the exact faces so float noise on the boundary is not counted.
    ASSERT_EQ(inside_all(gh, x, y, z), inside_all(bx, x, y, z))
        << "disagreement at (" << x << ", " << y << ", " << z << ")";
    ++agree;
  }
  EXPECT_EQ(agree, 20000);
  // And the frustum is not vacuous / not everything.
  EXPECT_TRUE(inside_all(gh, 0, 0, -5));
  EXPECT_FALSE(inside_all(gh, 100, 0, -5));
}

// --- AABB vs frustum -------------------------------------------------------

TEST(VisAabb, InsideIntersectOutside) {
  const frustum f = box_frustum(-10, 10, -10, 10, -10, 10);

  // Fully inside.
  std::uint8_t mask = 0x3f;
  EXPECT_EQ(aabb_vs_frustum(box(-1, -1, -1, 1, 1, 1), f, mask), frustum_test::inside);
  EXPECT_EQ(mask, 0u) << "a fully-inside box must clear every plane from the mask";

  // Straddling the +x face.
  mask = 0x3f;
  EXPECT_EQ(aabb_vs_frustum(box(9, -1, -1, 11, 1, 1), f, mask), frustum_test::intersect);
  EXPECT_NE(mask, 0u) << "a straddled plane stays in the mask";

  // Wholly beyond +x.
  EXPECT_EQ(aabb_vs_frustum(box(20, -1, -1, 22, 1, 1), f), frustum_test::outside);
  // Wholly behind -z.
  EXPECT_EQ(aabb_vs_frustum(box(-1, -1, -22, 1, 1, -20), f), frustum_test::outside);
}

TEST(VisAabb, PlaneMaskingPropagates) {
  const frustum f = box_frustum(-10, 10, -10, 10, -10, 10);
  // A child box fully clear of the left/right/near/far planes but straddling
  // top: only the y planes should remain live after the test.
  std::uint8_t mask = 0x3f;
  const frustum_test r = aabb_vs_frustum(box(-1, 9, -1, 1, 11, 1), f, mask);
  EXPECT_EQ(r, frustum_test::intersect);
  // Planes 0,1 (x), 4,5 (z) satisfied and cleared; plane 3 (top, y<=10) straddled.
  EXPECT_EQ(mask & (1u << 0), 0u);
  EXPECT_EQ(mask & (1u << 1), 0u);
  EXPECT_EQ(mask & (1u << 3), (1u << 3));
}

TEST(VisAabb, PreClearedMaskSkipsPlanes) {
  const frustum f = box_frustum(-10, 10, -10, 10, -10, 10);
  // Box is outside on +x, but if the caller says +x is already satisfied (bit 1
  // cleared) the test must skip it and not report OUTSIDE on that plane.
  std::uint8_t mask = 0x3f & ~(1u << 1);
  EXPECT_NE(aabb_vs_frustum(box(20, -1, -1, 22, 1, 1), f, mask), frustum_test::outside);
}

TEST(VisAabb, AxisAlignedIsExact) {
  // Box-vs-box is the one case the p/n-vertex test is EXACT for (no corner false
  // positives), so aabb_vs_frustum must agree with a ground-truth AABB overlap
  // on every box. This is the strong metamorphic check on the plane arithmetic.
  const float FL = -10, FH = 10;
  const frustum f = box_frustum(FL, FH, FL, FH, FL, FH);
  auto overlaps = [&](const aabb &b) {
    return b.mx[0] >= FL && b.mn[0] <= FH && b.mx[1] >= FL && b.mn[1] <= FH && b.mx[2] >= FL &&
           b.mn[2] <= FH;
  };
  std::mt19937 rng(7);
  std::uniform_real_distribution<float> C(-20.0f, 20.0f);
  std::uniform_real_distribution<float> S(0.1f, 6.0f);
  for (int i = 0; i < 20000; ++i) {
    const float cx = C(rng), cy = C(rng), cz = C(rng);
    const float hx = S(rng), hy = S(rng), hz = S(rng);
    const aabb b = box(cx - hx, cy - hy, cz - hz, cx + hx, cy + hy, cz + hz);
    const bool touch = aabb_vs_frustum(b, f) != frustum_test::outside;
    ASSERT_EQ(touch, overlaps(b)) << "at (" << cx << "," << cy << "," << cz << ")";
  }
}

TEST(VisAabb, ConservativeAtCornersOfAnAngledFrustum) {
  // A 90-degree wedge opening toward +x (corner at the origin, inside iff
  // x >= |y|), with the other four planes trivially satisfied. A box entirely in
  // x < 0 does NOT intersect the wedge, but its farthest corners clear both
  // angled planes individually -- the classic corner false positive. The test
  // reports it not-outside, which is the documented, harmless conservatism.
  const float s = 0.70710678f; // 1/sqrt(2)
  frustum f;
  f.p[0] = {{s, s, 0}, 0};    // x + y >= 0
  f.p[1] = {{s, -s, 0}, 0};   // x - y >= 0
  f.p[2] = {{0, 1, 0}, 1e6f}; // trivially satisfied
  f.p[3] = {{0, -1, 0}, 1e6f};
  f.p[4] = {{0, 0, 1}, 1e6f};
  f.p[5] = {{0, 0, -1}, 1e6f};

  const aabb outside_the_wedge = box(-0.5f, -2.0f, -0.1f, -0.1f, 2.0f, 0.1f);
  // Ground truth: every corner has x < 0, so none satisfies x >= |y| >= 0.
  EXPECT_NE(aabb_vs_frustum(outside_the_wedge, f), frustum_test::outside)
      << "corner false positive is expected (conservative), not a bug";

  // A box that really is behind one angled plane is still correctly OUTSIDE.
  EXPECT_EQ(aabb_vs_frustum(box(-5, 1, -0.1f, -3, 3, 0.1f), f), frustum_test::outside);
}

// --- Sphere vs frustum -----------------------------------------------------

TEST(VisSphere, InsideIntersectOutside) {
  const frustum f = box_frustum(-10, 10, -10, 10, -10, 10);
  EXPECT_EQ(sphere_vs_frustum(sphere{{0, 0, 0}, 1}, f), frustum_test::inside);
  EXPECT_EQ(sphere_vs_frustum(sphere{{10, 0, 0}, 2}, f), frustum_test::intersect);
  EXPECT_EQ(sphere_vs_frustum(sphere{{20, 0, 0}, 2}, f), frustum_test::outside);
}

TEST(VisSphere, BoundingSphereOfBox) {
  const sphere s = bounding_sphere(box(0, 0, 0, 2, 2, 2));
  EXPECT_FLOAT_EQ(s.c[0], 1.0f);
  EXPECT_FLOAT_EQ(s.c[1], 1.0f);
  EXPECT_FLOAT_EQ(s.c[2], 1.0f);
  EXPECT_NEAR(s.r, std::sqrt(3.0f), 1e-6f);
}
