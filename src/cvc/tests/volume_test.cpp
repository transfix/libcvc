/*
  Copyright 2007-2011 The University of Texas at Austin

  Unit tests for the volume class (extends voxels with spatial coordinate system)
*/

#include <cmath>
#include <cvc/core/app.h>
#include <cvc/volume/volume.h>
#include <gtest/gtest.h>

using namespace cvc;

// ============================================================================
// Construction and Basic Properties Tests
// ============================================================================

class VolumeTest : public ::testing::Test {
protected:
  cvc::app ctx;
};

TEST_F(VolumeTest, DefaultConstruction) {
  volume vol(ctx);

  // Default dimension is 4x4x4
  EXPECT_EQ(vol.XDim(), 4u);
  EXPECT_EQ(vol.YDim(), 4u);
  EXPECT_EQ(vol.ZDim(), 4u);

  // Default type is UChar
  EXPECT_EQ(vol.voxelType(), UChar);

  // Default bounding box is [-0.5, 0.5]³
  EXPECT_DOUBLE_EQ(vol.XMin(), -0.5);
  EXPECT_DOUBLE_EQ(vol.XMax(), 0.5);
  EXPECT_DOUBLE_EQ(vol.YMin(), -0.5);
  EXPECT_DOUBLE_EQ(vol.YMax(), 0.5);
  EXPECT_DOUBLE_EQ(vol.ZMin(), -0.5);
  EXPECT_DOUBLE_EQ(vol.ZMax(), 0.5);
}

TEST_F(VolumeTest, CustomConstruction) {
  dimension dim(10, 20, 30);
  bounding_box bbox(0.0, 0.0, 0.0, 10.0, 20.0, 30.0);

  volume vol(ctx, dim, Float, bbox);

  EXPECT_EQ(vol.XDim(), 10u);
  EXPECT_EQ(vol.YDim(), 20u);
  EXPECT_EQ(vol.ZDim(), 30u);
  EXPECT_EQ(vol.voxelType(), Float);

  EXPECT_DOUBLE_EQ(vol.XMin(), 0.0);
  EXPECT_DOUBLE_EQ(vol.XMax(), 10.0);
  EXPECT_DOUBLE_EQ(vol.YMin(), 0.0);
  EXPECT_DOUBLE_EQ(vol.YMax(), 20.0);
  EXPECT_DOUBLE_EQ(vol.ZMin(), 0.0);
  EXPECT_DOUBLE_EQ(vol.ZMax(), 30.0);
}

TEST_F(VolumeTest, CopyConstruction) {
  dimension dim(8, 8, 8);
  bounding_box bbox(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
  volume vol1(ctx, dim, Float, bbox);
  vol1.desc("Test Volume");
  vol1.fill(42.0);

  volume vol2(vol1);

  EXPECT_EQ(vol2.XDim(), vol1.XDim());
  EXPECT_EQ(vol2.YDim(), vol1.YDim());
  EXPECT_EQ(vol2.ZDim(), vol1.ZDim());
  EXPECT_EQ(vol2.voxelType(), vol1.voxelType());
  EXPECT_EQ(vol2.boundingBox(), vol1.boundingBox());
  EXPECT_EQ(vol2.desc(), vol1.desc());
  EXPECT_DOUBLE_EQ(vol2(4, 4, 4), 42.0);
}

TEST_F(VolumeTest, Assignment) {
  volume vol1(ctx, dimension(10, 10, 10), Float, bounding_box(0.0, 0.0, 0.0, 9.0, 9.0, 9.0));
  vol1.desc("Volume One");
  vol1.fill(100.0);

  volume vol2(ctx);
  vol2 = vol1;

  EXPECT_EQ(vol2.XDim(), 10u);
  EXPECT_EQ(vol2.desc(), "Volume One");
  EXPECT_DOUBLE_EQ(vol2(5, 5, 5), 100.0);
  EXPECT_EQ(vol2.boundingBox(), vol1.boundingBox());
}

// ============================================================================
// Span Calculation Tests
// ============================================================================

TEST_F(VolumeTest, SpanCalculation) {
  // 10x10x10 volume in [0,9]³ space
  // Span should be (9-0)/(10-1) = 1.0
  volume vol(ctx, dimension(10, 10, 10), Float, bounding_box(0.0, 0.0, 0.0, 9.0, 9.0, 9.0));

  EXPECT_DOUBLE_EQ(vol.XSpan(), 1.0);
  EXPECT_DOUBLE_EQ(vol.YSpan(), 1.0);
  EXPECT_DOUBLE_EQ(vol.ZSpan(), 1.0);
}

TEST_F(VolumeTest, SpanWithNonUniformBoundingBox) {
  // 5x10x20 volume in [0,4]x[0,18]x[0,38] space
  volume vol(ctx, dimension(5, 10, 20), Float, bounding_box(0.0, 0.0, 0.0, 4.0, 18.0, 38.0));

  EXPECT_DOUBLE_EQ(vol.XSpan(), 1.0); // 4/(5-1) = 1.0
  EXPECT_DOUBLE_EQ(vol.YSpan(), 2.0); // 18/(10-1) = 2.0
  EXPECT_DOUBLE_EQ(vol.ZSpan(), 2.0); // 38/(20-1) = 2.0
}

TEST_F(VolumeTest, SpanWithSingleVoxelDimension) {
  // 1x10x10 volume - X span should be 0
  volume vol(ctx, dimension(1, 10, 10), Float, bounding_box(0.0, 0.0, 0.0, 5.0, 9.0, 9.0));

  EXPECT_DOUBLE_EQ(vol.XSpan(), 0.0);
  EXPECT_DOUBLE_EQ(vol.YSpan(), 1.0);
  EXPECT_DOUBLE_EQ(vol.ZSpan(), 1.0);
}

// ============================================================================
// Interpolation Tests
// ============================================================================

TEST_F(VolumeTest, InterpolationCornerValues) {
  // Simple 2x2x2 volume with known values at corners
  volume vol(ctx, dimension(2, 2, 2), Float, bounding_box(0.0, 0.0, 0.0, 1.0, 1.0, 1.0));

  vol(0, 0, 0, 0.0);
  vol(1, 0, 0, 1.0);
  vol(0, 1, 0, 2.0);
  vol(1, 1, 0, 3.0);
  vol(0, 0, 1, 4.0);
  vol(1, 0, 1, 5.0);
  vol(0, 1, 1, 6.0);
  vol(1, 1, 1, 7.0);

  // Test corner values (should be exact)
  EXPECT_DOUBLE_EQ(vol.interpolate(0.0, 0.0, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(vol.interpolate(1.0, 0.0, 0.0), 1.0);
  EXPECT_DOUBLE_EQ(vol.interpolate(0.0, 1.0, 0.0), 2.0);
  EXPECT_DOUBLE_EQ(vol.interpolate(1.0, 1.0, 1.0), 7.0);
}

TEST_F(VolumeTest, InterpolationMidpoint) {
  // 2x2x2 volume with linear gradient
  volume vol(ctx, dimension(2, 2, 2), Float, bounding_box(0.0, 0.0, 0.0, 1.0, 1.0, 1.0));

  vol(0, 0, 0, 0.0);
  vol(1, 0, 0, 2.0);
  vol(0, 1, 0, 0.0);
  vol(1, 1, 0, 2.0);
  vol(0, 0, 1, 0.0);
  vol(1, 0, 1, 2.0);
  vol(0, 1, 1, 0.0);
  vol(1, 1, 1, 2.0);

  // Midpoint in X should be 1.0 (average of 0 and 2)
  double mid = vol.interpolate(0.5, 0.0, 0.0);
  EXPECT_NEAR(mid, 1.0, 0.01);
}

TEST_F(VolumeTest, InterpolationOutOfBounds) {
  volume vol(ctx, dimension(5, 5, 5), Float, bounding_box(0.0, 0.0, 0.0, 4.0, 4.0, 4.0));
  vol.fill(1.0);

  // Outside bounding box should throw
  EXPECT_THROW(vol.interpolate(-1.0, 2.0, 2.0), index_out_of_bounds);
  EXPECT_THROW(vol.interpolate(2.0, 5.0, 2.0), index_out_of_bounds);
  EXPECT_THROW(vol.interpolate(2.0, 2.0, -0.5), index_out_of_bounds);
}

TEST_F(VolumeTest, InterpolationLinearGradient) {
  // Volume with linear gradient along X
  volume vol(ctx, dimension(11, 5, 5), Float, bounding_box(0.0, 0.0, 0.0, 10.0, 4.0, 4.0));

  for (uint64 i = 0; i < 11; ++i) {
    for (uint64 j = 0; j < 5; ++j) {
      for (uint64 k = 0; k < 5; ++k) {
        vol(i, j, k, double(i) * 10.0); // Value = x * 10
      }
    }
  }

  // Interpolate at various X positions
  EXPECT_NEAR(vol.interpolate(0.0, 2.0, 2.0), 0.0, 0.1);
  EXPECT_NEAR(vol.interpolate(5.0, 2.0, 2.0), 50.0, 0.1);
  EXPECT_NEAR(vol.interpolate(7.5, 2.0, 2.0), 75.0, 0.1);
  EXPECT_NEAR(vol.interpolate(10.0, 2.0, 2.0), 100.0, 0.1);
}

// ============================================================================
// Subvolume Tests
// ============================================================================

TEST_F(VolumeTest, SubvolumeByOffset) {
  volume vol(ctx, dimension(10, 10, 10), Float, bounding_box(0.0, 0.0, 0.0, 9.0, 9.0, 9.0));

  // Fill with position-dependent values
  for (uint64 k = 0; k < 10; ++k) {
    for (uint64 j = 0; j < 10; ++j) {
      for (uint64 i = 0; i < 10; ++i) {
        vol(i, j, k, double(i + j * 10 + k * 100));
      }
    }
  }

  // Extract 3x3x3 subvolume starting at (2,3,4)
  vol.sub(2, 3, 4, dimension(3, 3, 3));

  EXPECT_EQ(vol.XDim(), 3u);
  EXPECT_EQ(vol.YDim(), 3u);
  EXPECT_EQ(vol.ZDim(), 3u);

  // Bounding box is updated using the ORIGINAL spans (before dimension change)
  // Original XSpan = 9/(10-1) = 1.0, so XMin = 0 + 1.0*2 = 2.0
  EXPECT_NEAR(vol.XMin(), 2.0, 0.01);
  EXPECT_NEAR(vol.YMin(), 3.0, 0.01);
  EXPECT_NEAR(vol.ZMin(), 4.0, 0.01);

  // XMax should be XMin + original_span * (new_dim - 1)
  // XMax = 2.0 + 1.0 * (3-1) = 4.0
  EXPECT_NEAR(vol.XMax(), 4.0, 0.01);
  EXPECT_NEAR(vol.YMax(), 5.0, 0.01);
  EXPECT_NEAR(vol.ZMax(), 6.0, 0.01);

  // Original (2,3,4) should now be at (0,0,0)
  EXPECT_DOUBLE_EQ(vol(0, 0, 0), 432.0); // 2 + 3*10 + 4*100
}

TEST_F(VolumeTest, SubvolumeByBoundingBox) {
  volume vol(ctx, dimension(10, 10, 10), Float, bounding_box(0.0, 0.0, 0.0, 9.0, 9.0, 9.0));
  vol.fill(5.0);

  // Extract central region
  bounding_box subbox(2.0, 2.0, 2.0, 7.0, 7.0, 7.0);
  vol.sub(subbox);

  // Should be approximately 6x6x6 voxels
  EXPECT_GE(vol.XDim(), 5u);
  EXPECT_LE(vol.XDim(), 7u);

  // Bounding box should match requested box
  EXPECT_NEAR(vol.XMin(), 2.0, 0.1);
  EXPECT_NEAR(vol.XMax(), 7.0, 0.1);
  EXPECT_NEAR(vol.YMin(), 2.0, 0.1);
  EXPECT_NEAR(vol.YMax(), 7.0, 0.1);
}

TEST_F(VolumeTest, SubvolumeWithDifferentResolution) {
  volume vol(ctx, dimension(20, 20, 20), Float, bounding_box(0.0, 0.0, 0.0, 19.0, 19.0, 19.0));

  // Fill with gradient
  for (uint64 k = 0; k < 20; ++k) {
    for (uint64 j = 0; j < 20; ++j) {
      for (uint64 i = 0; i < 20; ++i) {
        vol(i, j, k, double(i));
      }
    }
  }

  // Extract center region with different resolution
  bounding_box subbox(5.0, 5.0, 5.0, 14.0, 14.0, 14.0);
  dimension subdim(5, 5, 5); // Lower resolution

  vol.sub(subbox, subdim);

  EXPECT_EQ(vol.XDim(), 5u);
  EXPECT_EQ(vol.YDim(), 5u);
  EXPECT_EQ(vol.ZDim(), 5u);

  // Bounding box should match
  EXPECT_NEAR(vol.XMin(), 5.0, 0.1);
  EXPECT_NEAR(vol.XMax(), 14.0, 0.1);

  // Values should be interpolated
  EXPECT_GT(vol(0, 0, 0), 4.0);
  EXPECT_LT(vol(0, 0, 0), 6.0);
}

TEST_F(VolumeTest, SubvolumeOutOfBounds) {
  volume vol(ctx, dimension(10, 10, 10), Float, bounding_box(0.0, 0.0, 0.0, 9.0, 9.0, 9.0));

  // Subvolume extends outside original bounds
  bounding_box bad_box(-1.0, 0.0, 0.0, 5.0, 5.0, 5.0);

  EXPECT_THROW(vol.sub(bad_box), sub_volume_out_of_bounds);
}

// ============================================================================
// Combine Volumes Tests
// ============================================================================

TEST_F(VolumeTest, CombineWithNonOverlapping) {
  // Two non-overlapping volumes
  volume vol1(ctx, dimension(5, 5, 5), Float, bounding_box(0.0, 0.0, 0.0, 4.0, 4.0, 4.0));
  vol1.fill(10.0);

  volume vol2(ctx, dimension(5, 5, 5), Float, bounding_box(5.0, 5.0, 5.0, 9.0, 9.0, 9.0));
  vol2.fill(20.0);

  vol1.combineWith(vol2);

  // Combined bounding box should contain both
  EXPECT_NEAR(vol1.XMin(), 0.0, 0.01);
  EXPECT_NEAR(vol1.XMax(), 9.0, 0.01);
  EXPECT_NEAR(vol1.YMin(), 0.0, 0.01);
  EXPECT_NEAR(vol1.YMax(), 9.0, 0.01);
  EXPECT_NEAR(vol1.ZMin(), 0.0, 0.01);
  EXPECT_NEAR(vol1.ZMax(), 9.0, 0.01);

  // Dimension stays the same as vol1's original dimension (combineWith uses voxel_dimensions())
  EXPECT_EQ(vol1.XDim(), 5u);
}

TEST_F(VolumeTest, CombineWithOverlapping) {
  // Two overlapping volumes
  volume vol1(ctx, dimension(6, 6, 6), Float, bounding_box(0.0, 0.0, 0.0, 5.0, 5.0, 5.0));
  vol1.fill(100.0);

  volume vol2(ctx, dimension(6, 6, 6), Float, bounding_box(3.0, 3.0, 3.0, 8.0, 8.0, 8.0));
  vol2.fill(200.0);

  vol1.combineWith(vol2);

  // Combined bounding box spans both
  EXPECT_NEAR(vol1.XMin(), 0.0, 0.01);
  EXPECT_NEAR(vol1.XMax(), 8.0, 0.01);

  // Should have interpolated values from both volumes
  // Vol2 values should dominate in its region
  double val_in_vol2_region = vol1.interpolate(7.0, 7.0, 7.0);
  EXPECT_GT(val_in_vol2_region, 150.0);
}

TEST_F(VolumeTest, CombineWithCustomDimension) {
  volume vol1(ctx, dimension(5, 5, 5), Float, bounding_box(0.0, 0.0, 0.0, 4.0, 4.0, 4.0));
  vol1.fill(50.0);

  volume vol2(ctx, dimension(5, 5, 5), Float, bounding_box(5.0, 5.0, 5.0, 9.0, 9.0, 9.0));
  vol2.fill(50.0);

  dimension target_dim(20, 20, 20);
  vol1.combineWith(vol2, target_dim);

  EXPECT_EQ(vol1.XDim(), 20u);
  EXPECT_EQ(vol1.YDim(), 20u);
  EXPECT_EQ(vol1.ZDim(), 20u);
}

// ============================================================================
// Equality and Comparison Tests
// ============================================================================

TEST_F(VolumeTest, Equality) {
  volume vol1(ctx, dimension(5, 5, 5), Float, bounding_box(0.0, 0.0, 0.0, 4.0, 4.0, 4.0));
  vol1.fill(42.0);

  volume vol2(vol1);

  EXPECT_TRUE(vol1 == vol2);
  EXPECT_FALSE(vol1 != vol2);
}

TEST_F(VolumeTest, InequalityDifferentBoundingBox) {
  volume vol1(ctx, dimension(5, 5, 5), Float, bounding_box(0.0, 0.0, 0.0, 4.0, 4.0, 4.0));
  vol1.fill(42.0);

  volume vol2(ctx, dimension(5, 5, 5), Float, bounding_box(1.0, 1.0, 1.0, 5.0, 5.0, 5.0));
  vol2.fill(42.0);

  EXPECT_TRUE(vol1 != vol2);
  EXPECT_FALSE(vol1 == vol2);
}

TEST_F(VolumeTest, InequalityDifferentData) {
  // Create two volumes with same bounding box but different data
  // Use UChar for reliable comparison (Float might have precision issues)
  volume vol1(ctx, dimension(5, 5, 5), UChar, bounding_box(0.0, 0.0, 0.0, 4.0, 4.0, 4.0));
  vol1.fill(42.0);

  volume vol2(ctx, dimension(5, 5, 5), UChar, bounding_box(0.0, 0.0, 0.0, 4.0, 4.0, 4.0));
  vol2.fill(100.0);

  // Verify data is different
  EXPECT_DOUBLE_EQ(vol1(0, 0, 0), 42.0);
  EXPECT_DOUBLE_EQ(vol2(0, 0, 0), 100.0);

  // With fixed operator==, different data makes them unequal
  EXPECT_TRUE(vol1 != vol2);
}

// ============================================================================
// Description Tests
// ============================================================================

TEST_F(VolumeTest, Description) {
  volume vol(ctx);

  EXPECT_EQ(vol.desc(), "No Name");

  vol.desc("Test Volume");
  EXPECT_EQ(vol.desc(), "Test Volume");

  vol.desc("Modified Description");
  EXPECT_EQ(vol.desc(), "Modified Description");
}

TEST_F(VolumeTest, DescriptionPersistsThroughCopy) {
  volume vol1(ctx);
  vol1.desc("Original");

  volume vol2(vol1);
  EXPECT_EQ(vol2.desc(), "Original");

  volume vol3(ctx);
  vol3 = vol1;
  EXPECT_EQ(vol3.desc(), "Original");
}

// ============================================================================
// Edge Cases and Boundary Tests
// ============================================================================

TEST_F(VolumeTest, InterpolationAtBoundaryEdges) {
  volume vol(ctx, dimension(3, 3, 3), Float, bounding_box(0.0, 0.0, 0.0, 2.0, 2.0, 2.0));

  for (uint64 i = 0; i < 27; ++i) {
    vol(i, double(i));
  }

  // Test interpolation at exact boundary positions
  EXPECT_NO_THROW(vol.interpolate(0.0, 0.0, 0.0));
  EXPECT_NO_THROW(vol.interpolate(2.0, 2.0, 2.0));
  EXPECT_NO_THROW(vol.interpolate(0.0, 1.0, 2.0));
}

TEST_F(VolumeTest, VerySmallBoundingBox) {
  volume vol(ctx, dimension(10, 10, 10), Float, bounding_box(0.0, 0.0, 0.0, 0.001, 0.001, 0.001));

  vol.fill(1.0);

  // Spans should be very small
  EXPECT_LT(vol.XSpan(), 0.001);
  EXPECT_LT(vol.YSpan(), 0.001);
  EXPECT_LT(vol.ZSpan(), 0.001);

  // Interpolation should still work
  double val = vol.interpolate(0.0005, 0.0005, 0.0005);
  EXPECT_NEAR(val, 1.0, 0.1);
}

TEST_F(VolumeTest, NegativeBoundingBox) {
  volume vol(ctx, dimension(5, 5, 5), Float, bounding_box(-10.0, -10.0, -10.0, -6.0, -6.0, -6.0));
  vol.fill(3.14);

  EXPECT_DOUBLE_EQ(vol.XMin(), -10.0);
  EXPECT_DOUBLE_EQ(vol.XMax(), -6.0);

  // Interpolation in negative space
  double val = vol.interpolate(-8.0, -8.0, -8.0);
  EXPECT_NEAR(val, 3.14, 0.1);
}

TEST_F(VolumeTest, LargeBoundingBox) {
  volume vol(ctx, dimension(5, 5, 5), Float, bounding_box(0.0, 0.0, 0.0, 1000.0, 1000.0, 1000.0));
  vol.fill(99.0);

  // Large spans
  EXPECT_DOUBLE_EQ(vol.XSpan(), 250.0); // 1000/4

  // Interpolation should work
  double val = vol.interpolate(500.0, 500.0, 500.0);
  EXPECT_NEAR(val, 99.0, 0.1);
}

// ============================================================================
// Subvolume Preservation Tests
// ============================================================================

TEST_F(VolumeTest, SubvolumePreservesValues) {
  volume vol(ctx, dimension(10, 10, 10), Float, bounding_box(0.0, 0.0, 0.0, 9.0, 9.0, 9.0));

  // Set specific value at center
  vol.fill(0.0);
  vol(5, 5, 5, 123.456);

  // Extract subvolume containing that voxel
  vol.sub(4, 4, 4, dimension(3, 3, 3));

  // Value should be at new (1,1,1) position
  // Use NEAR because Float storage has precision limits
  EXPECT_NEAR(vol(1, 1, 1), 123.456, 0.001);
}

TEST_F(VolumeTest, SubvolumeInterpolationAccuracy) {
  // Create volume with known interpolatable pattern
  volume vol(ctx, dimension(11, 11, 11), Float, bounding_box(0.0, 0.0, 0.0, 10.0, 10.0, 10.0));

  // Linear gradient
  for (uint64 k = 0; k < 11; ++k) {
    for (uint64 j = 0; j < 11; ++j) {
      for (uint64 i = 0; i < 11; ++i) {
        vol(i, j, k, double(i + j + k));
      }
    }
  }

  // Extract with different resolution requiring interpolation
  bounding_box subbox(2.5, 2.5, 2.5, 7.5, 7.5, 7.5);
  dimension subdim(3, 3, 3);

  vol.sub(subbox, subdim);

  EXPECT_EQ(vol.XDim(), 3u);

  // Corner value should be interpolated correctly
  // At (2.5, 2.5, 2.5) the linear value should be 7.5
  EXPECT_NEAR(vol(0, 0, 0), 7.5, 0.5);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
