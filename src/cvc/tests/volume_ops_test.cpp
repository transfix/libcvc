/*
  Unit tests for volume_ops (volume arithmetic and utility operations)
*/

#include <cmath>
#include <cvc/app.h>
#include <cvc/volume_ops.h>
#include <gtest/gtest.h>

using namespace CVC_NAMESPACE;

// Helper: create a small Float volume with a constant value
static volume make_const_vol(app &ctx, uint64 dim, double val,
                             const bounding_box &bb = bounding_box(0, 0, 0, 1, 1, 1)) {
  volume v(ctx, dimension(dim, dim, dim), Float, bb);
  for (uint64 k = 0; k < dim; k++)
    for (uint64 j = 0; j < dim; j++)
      for (uint64 i = 0; i < dim; i++)
        v(i, j, k, val);
  return v;
}

// Helper: create a volume with linearly increasing values
static volume make_ramp_vol(app &ctx, uint64 dim) {
  volume v(ctx, dimension(dim, dim, dim), Float, bounding_box(0, 0, 0, 1, 1, 1));
  double n = 0.0;
  for (uint64 k = 0; k < dim; k++)
    for (uint64 j = 0; j < dim; j++)
      for (uint64 i = 0; i < dim; i++)
        v(i, j, k, n++);
  return v;
}

// ============================================================================
// Statistics Tests
// ============================================================================

class VolumeOpsTest : public ::testing::Test {
protected:
  cvc::app ctx;
};

TEST_F(VolumeOpsTest, StatsConstant) {
  auto vol = make_const_vol(ctx, 4, 5.0);
  auto s = compute_stats(vol);
  EXPECT_DOUBLE_EQ(s.min, 5.0);
  EXPECT_DOUBLE_EQ(s.max, 5.0);
  EXPECT_DOUBLE_EQ(s.mean, 5.0);
  EXPECT_DOUBLE_EQ(s.std_dev, 0.0);
  EXPECT_EQ(s.num_voxels, 64u);
}

TEST_F(VolumeOpsTest, StatsRamp) {
  auto vol = make_ramp_vol(ctx, 4);
  auto s = compute_stats(vol);
  EXPECT_DOUBLE_EQ(s.min, 0.0);
  EXPECT_DOUBLE_EQ(s.max, 63.0);
  EXPECT_NEAR(s.mean, 31.5, 1e-10);
  EXPECT_GT(s.std_dev, 0.0);
  EXPECT_EQ(s.num_voxels, 64u);
}

TEST_F(VolumeOpsTest, StatsBoundedRegion) {
  auto vol = make_const_vol(ctx, 4, 10.0, bounding_box(0, 0, 0, 3, 3, 3));
  // Region covering roughly the first quadrant
  bounding_box region(0, 0, 0, 1, 1, 1);
  auto s = compute_stats(vol, region);
  EXPECT_GT(s.num_voxels, 0u);
  EXPECT_DOUBLE_EQ(s.min, 10.0);
  EXPECT_DOUBLE_EQ(s.max, 10.0);
}

TEST_F(VolumeOpsTest, StatsEmptyRegion) {
  auto vol = make_const_vol(ctx, 4, 10.0, bounding_box(0, 0, 0, 1, 1, 1));
  // Region outside the volume
  bounding_box region(5, 5, 5, 6, 6, 6);
  auto s = compute_stats(vol, region);
  EXPECT_EQ(s.num_voxels, 0u);
}

// ============================================================================
// Arithmetic Tests
// ============================================================================

TEST_F(VolumeOpsTest, Add) {
  auto a = make_const_vol(ctx, 4, 3.0);
  auto b = make_const_vol(ctx, 4, 7.0);
  auto c = vol_add(a, b);
  EXPECT_EQ(c.XDim(), 4u);
  EXPECT_DOUBLE_EQ(c(0, 0, 0), 10.0);
  EXPECT_DOUBLE_EQ(c(3, 3, 3), 10.0);
}

TEST_F(VolumeOpsTest, Subtract) {
  auto a = make_const_vol(ctx, 4, 10.0);
  auto b = make_const_vol(ctx, 4, 3.0);
  auto c = vol_subtract(a, b);
  EXPECT_DOUBLE_EQ(c(0, 0, 0), 7.0);
}

TEST_F(VolumeOpsTest, Difference) {
  auto a = make_const_vol(ctx, 4, 3.0);
  auto b = make_const_vol(ctx, 4, 10.0);
  auto c = vol_difference(a, b);
  EXPECT_DOUBLE_EQ(c(0, 0, 0), 7.0);
}

TEST_F(VolumeOpsTest, Average) {
  auto a = make_const_vol(ctx, 4, 2.0);
  auto b = make_const_vol(ctx, 4, 8.0);
  auto c = vol_average(a, b);
  EXPECT_DOUBLE_EQ(c(0, 0, 0), 5.0);
}

TEST_F(VolumeOpsTest, DimensionMismatchThrows) {
  volume a(ctx, dimension(4, 4, 4), Float);
  volume b(ctx, dimension(8, 8, 8), Float);
  EXPECT_THROW(vol_add(a, b), dimension_mismatch);
  EXPECT_THROW(vol_subtract(a, b), dimension_mismatch);
  EXPECT_THROW(vol_difference(a, b), dimension_mismatch);
  EXPECT_THROW(vol_average(a, b), dimension_mismatch);
}

// ============================================================================
// Scalar Operations Tests
// ============================================================================

TEST_F(VolumeOpsTest, Scale) {
  auto vol = make_const_vol(ctx, 4, 5.0);
  auto out = vol_scale(vol, 3.0);
  EXPECT_DOUBLE_EQ(out(0, 0, 0), 15.0);
}

TEST_F(VolumeOpsTest, Normalize) {
  auto vol = make_ramp_vol(ctx, 4); // 0..63
  auto out = vol_normalize(vol, 0.0, 1.0);
  EXPECT_NEAR(out(0, 0, 0), 0.0, 1e-10);
  // Last voxel: index (3,3,3) = 63
  EXPECT_NEAR(out(3, 3, 3), 1.0, 1e-10);
}

TEST_F(VolumeOpsTest, NormalizeConstant) {
  auto vol = make_const_vol(ctx, 4, 5.0);
  // All same value → normalized to new_min
  auto out = vol_normalize(vol, 10.0, 20.0);
  EXPECT_DOUBLE_EQ(out(0, 0, 0), 10.0);
}

TEST_F(VolumeOpsTest, Clip) {
  auto vol = make_ramp_vol(ctx, 4);
  auto out = vol_clip(vol, 32.0);
  EXPECT_DOUBLE_EQ(out(0, 0, 0), 0.0); // 0 < 32 → kept
  // Voxel with value 32 should be zeroed (>= threshold)
  // value 32 is at index 32 → (0, 0, 2) since 32 = 0 + 0*4 + 2*16
  EXPECT_DOUBLE_EQ(out(0, 0, 2), 0.0);
}

TEST_F(VolumeOpsTest, ClampMin) {
  auto vol = make_ramp_vol(ctx, 4);
  auto out = vol_clamp_min(vol, 30.0);
  EXPECT_DOUBLE_EQ(out(0, 0, 0), 30.0); // 0 < 30 → clamped to 30
  EXPECT_DOUBLE_EQ(out(3, 3, 3), 63.0); // 63 > 30 → unchanged
}

TEST_F(VolumeOpsTest, Negate) {
  auto vol = make_const_vol(ctx, 4, 5.0);
  auto out = vol_negate(vol);
  EXPECT_DOUBLE_EQ(out(0, 0, 0), -5.0);
}

// ============================================================================
// Masking Tests
// ============================================================================

TEST_F(VolumeOpsTest, Mask) {
  auto intensity = make_const_vol(ctx, 4, 10.0);
  auto mask = make_const_vol(ctx, 4, 0.0);
  // Set some mask voxels to nonzero
  mask(0, 0, 0, 1.0);
  mask(1, 1, 1, 1.0);

  auto out = vol_mask(intensity, mask);
  EXPECT_DOUBLE_EQ(out(0, 0, 0), 0.0);  // masked out
  EXPECT_DOUBLE_EQ(out(1, 1, 1), 0.0);  // masked out
  EXPECT_DOUBLE_EQ(out(2, 2, 2), 10.0); // not masked
}

TEST_F(VolumeOpsTest, InverseMask) {
  auto intensity = make_const_vol(ctx, 4, 10.0);
  auto mask = make_const_vol(ctx, 4, 0.0);
  mask(0, 0, 0, 1.0);

  auto out = vol_inverse_mask(intensity, mask);
  EXPECT_DOUBLE_EQ(out(0, 0, 0), 10.0); // mask is nonzero → kept
  EXPECT_DOUBLE_EQ(out(2, 2, 2), 0.0);  // mask is zero → zeroed
}

TEST_F(VolumeOpsTest, MaskDimensionMismatch) {
  volume a(ctx, dimension(4, 4, 4), Float);
  volume b(ctx, dimension(8, 8, 8), Float);
  EXPECT_THROW(vol_mask(a, b), dimension_mismatch);
}

// ============================================================================
// Spatial Tests
// ============================================================================

TEST_F(VolumeOpsTest, Downsample) {
  auto vol = make_ramp_vol(ctx, 8);
  auto out = vol_downsample(vol, 2, 2, 2);
  EXPECT_EQ(out.XDim(), 4u);
  EXPECT_EQ(out.YDim(), 4u);
  EXPECT_EQ(out.ZDim(), 4u);
  // First voxel: stride 0 → value at (0,0,0) = 0
  EXPECT_DOUBLE_EQ(out(0, 0, 0), 0.0);
  // Second voxel in x: stride 2 → value at (2,0,0) = 2
  EXPECT_DOUBLE_EQ(out(1, 0, 0), 2.0);
}

TEST_F(VolumeOpsTest, DownsampleZeroFactor) {
  auto vol = make_const_vol(ctx, 4, 1.0);
  EXPECT_THROW(vol_downsample(vol, 0, 1, 1), std::invalid_argument);
}

TEST_F(VolumeOpsTest, DownsamplePreservesOrigin) {
  bounding_box bb(1.0, 2.0, 3.0, 5.0, 6.0, 7.0);
  auto vol = make_const_vol(ctx, 8, 1.0, bb);
  auto out = vol_downsample(vol, 2, 2, 2);
  EXPECT_DOUBLE_EQ(out.XMin(), 1.0);
  EXPECT_DOUBLE_EQ(out.YMin(), 2.0);
  EXPECT_DOUBLE_EQ(out.ZMin(), 3.0);
}

// ============================================================================
// Output Properties Tests
// ============================================================================

TEST_F(VolumeOpsTest, AddPreservesBoundingBox) {
  bounding_box bb(1, 2, 3, 4, 5, 6);
  auto a = make_const_vol(ctx, 4, 1.0, bb);
  auto b = make_const_vol(ctx, 4, 2.0, bb);
  auto c = vol_add(a, b);
  EXPECT_DOUBLE_EQ(c.XMin(), 1.0);
  EXPECT_DOUBLE_EQ(c.YMax(), 5.0);
}

TEST_F(VolumeOpsTest, OutputIsFloat) {
  // Even if input is UChar, outputs should be Float for full precision
  volume a(ctx, dimension(4, 4, 4), UChar);
  volume b(ctx, dimension(4, 4, 4), UChar);
  auto c = vol_add(a, b);
  EXPECT_EQ(c.voxelType(), Float);
}

// ============================================================================
// Rotation Tests
// ============================================================================

TEST_F(VolumeOpsTest, RotateZeroAngle) {
  auto vol = make_ramp_vol(ctx, 8);
  auto rot = vol_rotate_z(vol, 0.0);
  EXPECT_EQ(rot.XDim(), vol.XDim());
  EXPECT_EQ(rot.YDim(), vol.YDim());
  EXPECT_EQ(rot.ZDim(), vol.ZDim());
  // Zero-angle rotation should preserve values (up to interpolation)
  for (uint64 k = 1; k < vol.ZDim() - 1; k++)
    for (uint64 j = 1; j < vol.YDim() - 1; j++)
      for (uint64 i = 1; i < vol.XDim() - 1; i++)
        EXPECT_NEAR(rot(i, j, k), vol(i, j, k), 1e-6);
}

TEST_F(VolumeOpsTest, RotatePreservesDimensions) {
  auto vol = make_const_vol(ctx, 8, 1.0);
  auto rot = vol_rotate_z(vol, M_PI / 4.0);
  EXPECT_EQ(rot.XDim(), vol.XDim());
  EXPECT_EQ(rot.YDim(), vol.YDim());
  EXPECT_EQ(rot.ZDim(), vol.ZDim());
}

TEST_F(VolumeOpsTest, Rotate180Symmetry) {
  auto vol = make_const_vol(ctx, 8, 42.0);
  auto rot = vol_rotate_z(vol, M_PI);
  // For a constant volume, 180-degree rotation should yield same values
  // (center voxels at least)
  EXPECT_NEAR(rot(4, 4, 4), 42.0, 1e-6);
}

// ============================================================================
// SSIM Tests
// ============================================================================

TEST_F(VolumeOpsTest, SSIMIdentical) {
  auto vol = make_ramp_vol(ctx, 8);
  auto result = vol_ssim(vol, vol, 3, 1.0);
  EXPECT_NEAR(result.mean_ssim, 1.0, 1e-10);
}

TEST_F(VolumeOpsTest, SSIMDifferent) {
  auto a = make_const_vol(ctx, 8, 0.0);
  auto b = make_const_vol(ctx, 8, 255.0);
  auto result = vol_ssim(a, b, 3, 1.0);
  EXPECT_LT(result.mean_ssim, 0.1);
}

TEST_F(VolumeOpsTest, SSIMMapDimensions) {
  auto a = make_ramp_vol(ctx, 8);
  auto b = make_ramp_vol(ctx, 8);
  auto result = vol_ssim(a, b, 3, 1.0);
  EXPECT_EQ(result.ssim_map.XDim(), a.XDim());
  EXPECT_EQ(result.ssim_map.YDim(), a.YDim());
  EXPECT_EQ(result.ssim_map.ZDim(), a.ZDim());
}

TEST_F(VolumeOpsTest, SSIMBadWindowSize) {
  auto vol = make_ramp_vol(ctx, 4);
  EXPECT_THROW(vol_ssim(vol, vol, 4, 1.0), std::invalid_argument);
  EXPECT_THROW(vol_ssim(vol, vol, 0, 1.0), std::invalid_argument);
}

TEST_F(VolumeOpsTest, SSIMDimensionMismatch) {
  auto a = make_const_vol(ctx, 4, 1.0);
  volume b(ctx, dimension(8, 8, 8), Float, bounding_box(0, 0, 0, 1, 1, 1));
  EXPECT_THROW(vol_ssim(a, b), dimension_mismatch);
}

// ============================================================================
// Projection Tests
// ============================================================================

TEST_F(VolumeOpsTest, ProjectSingleAngle) {
  auto vol = make_const_vol(ctx, 8, 1.0);
  std::vector<double> angles = {0.0};
  auto proj = vol_project(vol, angles, 0.5);
  EXPECT_EQ(proj.XDim(), vol.XDim());
  EXPECT_EQ(proj.YDim(), vol.YDim());
  EXPECT_EQ(proj.ZDim(), (uint64)1);
}

TEST_F(VolumeOpsTest, ProjectMultipleAngles) {
  auto vol = make_const_vol(ctx, 8, 1.0);
  std::vector<double> angles = {0.0, M_PI / 4.0, M_PI / 2.0};
  auto proj = vol_project(vol, angles, 0.5);
  EXPECT_EQ(proj.ZDim(), (uint64)3);
}

TEST_F(VolumeOpsTest, ProjectEmptyAngles) {
  auto vol = make_const_vol(ctx, 4, 1.0);
  std::vector<double> empty;
  EXPECT_THROW(vol_project(vol, empty), std::invalid_argument);
}

TEST_F(VolumeOpsTest, ProjectNonzeroIntegrals) {
  auto vol = make_const_vol(ctx, 8, 1.0);
  std::vector<double> angles = {0.0};
  auto proj = vol_project(vol, angles, 0.1);
  // A constant volume should produce positive projection values
  bool has_nonzero = false;
  for (uint64 j = 0; j < proj.YDim(); j++)
    for (uint64 i = 0; i < proj.XDim(); i++)
      if (proj(i, j, 0) > 0)
        has_nonzero = true;
  EXPECT_TRUE(has_nonzero);
}

// ============================================================================
// Back-projection Tests
// ============================================================================

TEST_F(VolumeOpsTest, BackProjectDimensions) {
  // Create a small "projection" volume: 8 x 8 x 2 (2 angles)
  volume proj(ctx, dimension(8, 8, 2), Float, bounding_box(0, 0, 0, 7, 7, 1));
  for (uint64 k = 0; k < 2; k++)
    for (uint64 j = 0; j < 8; j++)
      for (uint64 i = 0; i < 8; i++)
        proj(i, j, k, 1.0);
  std::vector<double> angles = {0.0, M_PI / 2.0};
  auto recon = vol_back_project(proj, angles, 8, true);
  EXPECT_EQ(recon.XDim(), (uint64)8);
  EXPECT_EQ(recon.YDim(), (uint64)8);
  EXPECT_EQ(recon.ZDim(), (uint64)8);
}

TEST_F(VolumeOpsTest, BackProjectNoFilter) {
  volume proj(ctx, dimension(4, 4, 1), Float, bounding_box(0, 0, 0, 3, 3, 0));
  for (uint64 j = 0; j < 4; j++)
    for (uint64 i = 0; i < 4; i++)
      proj(i, j, 0, 1.0);
  std::vector<double> angles = {0.0};
  auto recon = vol_back_project(proj, angles, 4, false);
  EXPECT_EQ(recon.XDim(), (uint64)4);
}

TEST_F(VolumeOpsTest, BackProjectEmptyAngles) {
  volume proj(ctx, dimension(4, 4, 1), Float, bounding_box(0, 0, 0, 3, 3, 0));
  std::vector<double> empty;
  EXPECT_THROW(vol_back_project(proj, empty, 4), std::invalid_argument);
}

// ============================================================================
// RGBA Merge / Split Tests
// ============================================================================

TEST_F(VolumeOpsTest, RGBAMergeDimensions) {
  auto r = make_const_vol(ctx, 4, 1.0);
  auto g = make_const_vol(ctx, 4, 2.0);
  auto b = make_const_vol(ctx, 4, 3.0);
  auto a = make_const_vol(ctx, 4, 4.0);
  auto merged = vol_rgba_merge(r, g, b, a);
  EXPECT_EQ(merged.XDim(), 4u * 4);
  EXPECT_EQ(merged.YDim(), (uint64)4);
  EXPECT_EQ(merged.ZDim(), (uint64)4);
}

TEST_F(VolumeOpsTest, RGBAMergeValues) {
  auto r = make_const_vol(ctx, 4, 10.0);
  auto g = make_const_vol(ctx, 4, 20.0);
  auto b = make_const_vol(ctx, 4, 30.0);
  auto a = make_const_vol(ctx, 4, 40.0);
  auto merged = vol_rgba_merge(r, g, b, a);
  // Check interleaved pattern for voxel (1,1,1)
  EXPECT_DOUBLE_EQ(merged(1 * 4 + 0, 1, 1), 10.0);
  EXPECT_DOUBLE_EQ(merged(1 * 4 + 1, 1, 1), 20.0);
  EXPECT_DOUBLE_EQ(merged(1 * 4 + 2, 1, 1), 30.0);
  EXPECT_DOUBLE_EQ(merged(1 * 4 + 3, 1, 1), 40.0);
}

TEST_F(VolumeOpsTest, RGBAMergeMismatch) {
  auto r = make_const_vol(ctx, 4, 1.0);
  auto g = make_const_vol(ctx, 4, 1.0);
  auto b = make_const_vol(ctx, 8, 1.0);
  auto a = make_const_vol(ctx, 4, 1.0);
  EXPECT_THROW(vol_rgba_merge(r, g, b, a), dimension_mismatch);
}

TEST_F(VolumeOpsTest, SplitVarsSingle) {
  auto vol = make_ramp_vol(ctx, 4);
  auto parts = vol_split_vars(vol);
  EXPECT_EQ(parts.size(), 1u);
  EXPECT_EQ(parts[0].XDim(), vol.XDim());
}
