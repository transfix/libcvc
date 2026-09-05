// Unit tests for the cvc::volslice slicing engine.
//
// Everything here is checked against values derived INDEPENDENTLY of the
// implementation: the arand slice-count formula evaluated by hand, the
// geometry of axis-aligned and body-diagonal cuts of a cube, and the
// invariants a plane/box intersection must satisfy regardless of tables
// (fan planarity, vertex count 3..6, back-to-front ordering).
#include <cmath>
#include <cvc/volslice/slicer.h>
#include <gtest/gtest.h>

using cvc::volslice::box3d;
using cvc::volslice::compute_slices;
using cvc::volslice::mat4;
using cvc::volslice::slice_geometry;
using cvc::volslice::slice_params;
using cvc::volslice::vec3d;
using cvc::volslice::view_plane_normal;

namespace {

// Centroid of one fan polygon.
vec3d centroid(const slice_geometry &g, std::size_t poly) {
  vec3d c{0, 0, 0};
  const auto off = g.fan_offset[poly], n = g.fan_count[poly];
  for (std::uint32_t v = off; v < off + n; ++v) {
    c.x += g.positions[v * 3 + 0];
    c.y += g.positions[v * 3 + 1];
    c.z += g.positions[v * 3 + 2];
  }
  c.x /= n;
  c.y /= n;
  c.z /= n;
  return c;
}

// A pure rotation whose third ROW is `dir` (unit): the view-plane normal of
// the resulting "camera" matrix is dir (see view_plane_normal: with identity
// projection folded in, n = row2 + row3, and row3 of a rotation is 0).
mat4 rotation_with_view_normal(vec3d dir) {
  // build an orthonormal basis {u, v, dir}
  vec3d up = std::fabs(dir.z) < 0.9 ? vec3d{0, 0, 1} : vec3d{1, 0, 0};
  vec3d u{up.y * dir.z - up.z * dir.y, up.z * dir.x - up.x * dir.z, up.x * dir.y - up.y * dir.x};
  const double ul = std::sqrt(u.x * u.x + u.y * u.y + u.z * u.z);
  u = {u.x / ul, u.y / ul, u.z / ul};
  vec3d v{dir.y * u.z - dir.z * u.y, dir.z * u.x - dir.x * u.z, dir.x * u.y - dir.y * u.x};
  mat4 m;
  m.m = {u.x, u.y, u.z, 0, v.x, v.y, v.z, 0, dir.x, dir.y, dir.z, 0, 0, 0, 0, 1};
  return m;
}

// The plane count the arand formula predicts for a unit cube seen head-on:
// interval = min_ratio/N with N = 2*(10 + max_planes*q^3); planes that hit
// the cube are those with sweep offset in [-0.5, 0.5] of the view axis.
double expected_headon_planes(double q, int mp) {
  const double N = 2.0 * (10.0 + mp * q * q * q);
  return N; // sweep hits |z| <= 0.5, interval = 1/N -> ~N planes
}

} // namespace

TEST(ViewPlaneNormal, IdentityLooksAlongMinusZ) {
  const vec3d n = view_plane_normal(mat4::identity());
  EXPECT_NEAR(n.x, 0.0, 1e-12);
  EXPECT_NEAR(n.y, 0.0, 1e-12);
  EXPECT_NEAR(n.z, 1.0, 1e-12);
}

TEST(ViewPlaneNormal, DegenerateMatrixThrows) {
  mat4 zero;
  zero.m.fill(0.0);
  EXPECT_THROW(view_plane_normal(zero), cvc::volslice_error);
}

TEST(Slicer, HeadOnViewProducesAxisAlignedQuads) {
  slice_geometry g = compute_slices(mat4::identity(), box3d{}, slice_params{});
  ASSERT_FALSE(g.empty());

  // Every slice of an axis-aligned view is a quad at constant z.
  for (std::size_t p = 0; p < g.planes(); ++p) {
    ASSERT_EQ(4u, g.fan_count[p]) << "polygon " << p;
    const auto off = g.fan_offset[p];
    const float z0 = g.positions[off * 3 + 2];
    for (std::uint32_t v = off; v < off + 4; ++v)
      EXPECT_NEAR(z0, g.positions[v * 3 + 2], 1e-6f);
  }

  // Count matches the arand formula: N = 2*(10 + 1000*0.5^3) = 270.
  const double expect = expected_headon_planes(0.5, 1000);
  EXPECT_NEAR(double(g.planes()), expect, 3.0);
}

TEST(Slicer, SlicesAreOrderedBackToFront) {
  // Identity clip: the viewer looks along -z, so back-to-front means the
  // first polygon has the MOST NEGATIVE z and centroids increase from there.
  slice_geometry g = compute_slices(mat4::identity(), box3d{}, slice_params{});
  ASSERT_GT(g.planes(), 10u);
  double prev = centroid(g, 0).z;
  EXPECT_LT(prev, 0.0); // farthest first
  for (std::size_t p = 1; p < g.planes(); ++p) {
    const double z = centroid(g, p).z;
    EXPECT_GT(z, prev) << "polygon " << p << " not in back-to-front order";
    prev = z;
  }
}

TEST(Slicer, TexcoordsTrackPositionsOnTheUnitCube) {
  // For the default box (unit cube at origin) with the full sub-cube,
  // texcoord == position + 0.5 exactly, on every axis of every vertex.
  slice_geometry g = compute_slices(mat4::identity(), box3d{}, slice_params{});
  ASSERT_FALSE(g.empty());
  for (std::size_t i = 0; i < g.positions.size(); ++i)
    EXPECT_NEAR(g.texcoords[i], g.positions[i] + 0.5f, 1e-5f) << "component " << i;
}

TEST(Slicer, BodyDiagonalViewYieldsAHexagon) {
  const double s = 1.0 / std::sqrt(3.0);
  slice_geometry g = compute_slices(rotation_with_view_normal({s, s, s}), box3d{}, slice_params{});
  ASSERT_FALSE(g.empty());
  std::uint32_t maxv = 0, minv = 7;
  for (std::size_t p = 0; p < g.planes(); ++p) {
    maxv = std::max(maxv, g.fan_count[p]);
    minv = std::min(minv, g.fan_count[p]);
    ASSERT_GE(g.fan_count[p], 3u);
    ASSERT_LE(g.fan_count[p], 6u);
  }
  // The mid-diagonal cut of a cube is the classic hexagon; near the corners
  // the cut is a triangle.
  EXPECT_EQ(6u, maxv);
  EXPECT_EQ(3u, minv);
}

TEST(Slicer, EveryPolygonIsPlanarAlongTheViewNormal) {
  const double s = 1.0 / std::sqrt(3.0);
  const mat4 m = rotation_with_view_normal({s, 2 * s / 2, s}); // == (s,s,s)
  slice_geometry g = compute_slices(m, box3d{}, slice_params{});
  const vec3d n = view_plane_normal(m);
  for (std::size_t p = 0; p < g.planes(); ++p) {
    const auto off = g.fan_offset[p], cnt = g.fan_count[p];
    const double d0 = n.x * g.positions[off * 3] + n.y * g.positions[off * 3 + 1] +
                      n.z * g.positions[off * 3 + 2];
    for (std::uint32_t v = off; v < off + cnt; ++v) {
      const double d =
          n.x * g.positions[v * 3] + n.y * g.positions[v * 3 + 1] + n.z * g.positions[v * 3 + 2];
      EXPECT_NEAR(d0, d, 1e-5) << "polygon " << p;
    }
  }
}

TEST(Slicer, NearPlaneCutsTheViewerSideOfTheSweep) {
  slice_params half;
  half.near_plane = 0.5;
  slice_geometry full = compute_slices(mat4::identity(), box3d{}, slice_params{});
  slice_geometry cut = compute_slices(mat4::identity(), box3d{}, half);
  ASSERT_FALSE(cut.empty());
  // Half the diagonal is cut from the NEAR side: every remaining slice sits
  // in the far half (z <= 0 for the -z viewer), and the count drops.
  EXPECT_LT(cut.planes(), full.planes());
  for (std::size_t p = 0; p < cut.planes(); ++p)
    EXPECT_LE(centroid(cut, p).z, 1e-6);
}

TEST(Slicer, MaxPlanesCapStretchesTheInterval) {
  // q=1, max_planes=5: uncapped sweep = sqrt(3)*2*(10+5) = ~52 planes, cap =
  // 10*5 = 50, so the cap engages and the total swept count is <= 50.
  slice_params p;
  p.quality = 1.0;
  p.max_planes = 5;
  slice_geometry g = compute_slices(mat4::identity(), box3d{}, p);
  ASSERT_FALSE(g.empty());
  EXPECT_LE(g.planes(), 50u);
  // The cap stretched the interval past min_ratio/N.
  const double uncapped = 1.0 / (2.0 * (10.0 + 5.0));
  EXPECT_GT(g.plane_spacing, uncapped);
}

TEST(Slicer, SubCubeRemapsTexcoordsOnly) {
  slice_params p;
  p.tex_min = {0.25, 0.25, 0.25};
  p.tex_max = {0.75, 0.75, 0.75};
  slice_geometry full = compute_slices(mat4::identity(), box3d{}, slice_params{});
  slice_geometry sub = compute_slices(mat4::identity(), box3d{}, p);
  ASSERT_EQ(full.vertices(), sub.vertices());
  for (std::size_t i = 0; i < sub.texcoords.size(); ++i) {
    EXPECT_GE(sub.texcoords[i], 0.25f - 1e-6f);
    EXPECT_LE(sub.texcoords[i], 0.75f + 1e-6f);
    // Positions are untouched by the sub-cube.
    EXPECT_EQ(full.positions[i], sub.positions[i]);
  }
}

TEST(Slicer, NonCubicBoxStaysInsideItsBounds) {
  box3d box;
  box.min = {10.0, -2.0, 3.0};
  box.max = {12.0, -1.0, 7.0}; // 2 x 1 x 4
  slice_geometry g = compute_slices(mat4::identity(), box, slice_params{});
  ASSERT_FALSE(g.empty());
  for (std::size_t i = 0; i < g.positions.size(); i += 3) {
    EXPECT_GE(g.positions[i + 0], float(box.min.x) - 1e-4f);
    EXPECT_LE(g.positions[i + 0], float(box.max.x) + 1e-4f);
    EXPECT_GE(g.positions[i + 1], float(box.min.y) - 1e-4f);
    EXPECT_LE(g.positions[i + 1], float(box.max.y) + 1e-4f);
    EXPECT_GE(g.positions[i + 2], float(box.min.z) - 1e-4f);
    EXPECT_LE(g.positions[i + 2], float(box.max.z) + 1e-4f);
  }
  // Spacing is reported in LOCAL units: ratio-space interval times the
  // longest side (4).  min_ratio = 1/4, N = 270 -> spacing = (0.25/270)*4.
  EXPECT_NEAR(g.plane_spacing, 0.25 / 270.0 * 4.0, 1e-9);
}

TEST(Slicer, DegenerateBoxIsEmptyNotFatal) {
  box3d flat;
  flat.min = {0, 0, 0};
  flat.max = {0, 0, 0};
  slice_geometry g = compute_slices(mat4::identity(), flat, slice_params{});
  EXPECT_TRUE(g.empty());
}

TEST(Slicer, QualityZeroStillRenders) {
  // q=0 collapses to N=20 slices minimum -- the legacy "interactive" floor.
  slice_params p;
  p.quality = 0.0;
  slice_geometry g = compute_slices(mat4::identity(), box3d{}, p);
  EXPECT_NEAR(double(g.planes()), 20.0, 2.0);
}

TEST(Slicer, OutOfRangeParamsAreClamped) {
  slice_params p;
  p.quality = 7.0;     // -> 1.0
  p.max_planes = -3;   // -> limits::min_max_planes
  p.near_plane = -0.5; // -> 0.0
  slice_geometry g = compute_slices(mat4::identity(), box3d{}, p);
  ASSERT_FALSE(g.empty());
  EXPECT_LE(g.planes(), 10u * 1u); // capped at 10*max_planes with mp=1
}
