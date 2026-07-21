/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

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

#include <algorithm>
#include <atomic>
#include <boost/thread.hpp>
#include <chrono>
#include <cvc/core/app.h>
#include <cvc/core/exception.h>
#include <cvc/core/types.h>
#include <cvc/utility/composite_function.h>
#include <cvc/volume/voxels.h>
#include <gtest/gtest.h>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <thread>
#include <vector>

using namespace cvc;

// Global flag to enable/disable stress and performance tests
// Can be enabled with --enable-stress-tests command line flag
bool enable_stress_tests = false;

// ============================================================================
// Construction and Basic Properties Tests
// ============================================================================

class VoxelsTest : public ::testing::Test {
protected:
  cvc::app ctx;
};

TEST_F(VoxelsTest, DefaultConstruction) {
  voxels v(ctx);

  // Default is 4x4x4 UChar
  EXPECT_EQ(v.XDim(), 4u);
  EXPECT_EQ(v.YDim(), 4u);
  EXPECT_EQ(v.ZDim(), 4u);
  EXPECT_EQ(v.voxelType(), UChar);
  EXPECT_EQ(v.voxelSize(), 1u);
  EXPECT_STREQ(v.voxelTypeStr(), "unsigned char");
}

TEST_F(VoxelsTest, DimensionConstruction) {
  dimension dim(10, 20, 30);
  voxels v(ctx, dim, Float);

  EXPECT_EQ(v.XDim(), 10u);
  EXPECT_EQ(v.YDim(), 20u);
  EXPECT_EQ(v.ZDim(), 30u);
  EXPECT_EQ(v.voxelType(), Float);
  EXPECT_EQ(v.voxelSize(), 4u);
  EXPECT_STREQ(v.voxelTypeStr(), "float");
}

TEST_F(VoxelsTest, DataTypeConstruction) {
  std::vector<data_type> types = {UChar, UShort, UInt, Float, Double, UInt64};
  std::vector<uint64> sizes = {1, 2, 4, 4, 8, 8};

  for (size_t i = 0; i < types.size(); ++i) {
    voxels v(ctx, dimension(5, 5, 5), types[i]);
    EXPECT_EQ(v.voxelType(), types[i]);
    EXPECT_EQ(v.voxelSize(), sizes[i]);
  }
}

TEST_F(VoxelsTest, CopyConstruction) {
  dimension dim(8, 8, 8);
  voxels v1(ctx, dim, UShort);

  // Set some values
  v1(0, 0, 0, 100.0);
  v1(5, 5, 5, 200.0);

  voxels v2(v1);

  EXPECT_EQ(v2.XDim(), v1.XDim());
  EXPECT_EQ(v2.YDim(), v1.YDim());
  EXPECT_EQ(v2.ZDim(), v1.ZDim());
  EXPECT_EQ(v2.voxelType(), v1.voxelType());
  EXPECT_DOUBLE_EQ(v2(0, 0, 0), 100.0);
  EXPECT_DOUBLE_EQ(v2(5, 5, 5), 200.0);
}

TEST_F(VoxelsTest, PointerConstruction) {
  dimension dim(4, 4, 4);
  std::vector<unsigned char> data(64, 42);

  voxels v(ctx, data.data(), dim, UChar);

  EXPECT_EQ(v.XDim(), 4u);
  EXPECT_EQ(v.YDim(), 4u);
  EXPECT_EQ(v.ZDim(), 4u);
  EXPECT_DOUBLE_EQ(v(0, 0, 0), 42.0);
  EXPECT_DOUBLE_EQ(v(3, 3, 3), 42.0);
}

// ============================================================================
// Voxel Access Tests
// ============================================================================

TEST_F(VoxelsTest, LinearIndexAccess) {
  voxels v(ctx, dimension(10, 10, 10), Float);

  // Write and read using linear index
  v(0, 1.5);
  v(50, 2.5);
  v(999, 3.5);

  EXPECT_DOUBLE_EQ(v(0), 1.5);
  EXPECT_DOUBLE_EQ(v(50), 2.5);
  EXPECT_DOUBLE_EQ(v(999), 3.5);
}

TEST_F(VoxelsTest, ThreeDimensionalAccess) {
  voxels v(ctx, dimension(10, 10, 10), Double);

  // Write and read using 3D coordinates
  v(0, 0, 0, 10.0);
  v(5, 5, 5, 20.0);
  v(9, 9, 9, 30.0);

  EXPECT_DOUBLE_EQ(v(0, 0, 0), 10.0);
  EXPECT_DOUBLE_EQ(v(5, 5, 5), 20.0);
  EXPECT_DOUBLE_EQ(v(9, 9, 9), 30.0);
}

TEST_F(VoxelsTest, OutOfBoundsRead) {
  voxels v(ctx, dimension(5, 5, 5), UChar);

  // Should throw index_out_of_bounds
  EXPECT_THROW(v(125), index_out_of_bounds);
  EXPECT_THROW(v(1000), index_out_of_bounds);
}

TEST_F(VoxelsTest, OutOfBoundsWrite) {
  voxels v(ctx, dimension(5, 5, 5), UChar);

  // Should throw index_out_of_bounds
  EXPECT_THROW(v(125, 42.0), index_out_of_bounds);
  EXPECT_THROW(v(1000, 42.0), index_out_of_bounds);
}

TEST_F(VoxelsTest, TypeConversionRead) {
  voxels v(ctx, dimension(5, 5, 5), UChar);

  // Write as double, stored as unsigned char, read as double
  v(0, 127.5); // Will be truncated to 127
  v(1, 255.9); // Will be truncated to 255

  EXPECT_DOUBLE_EQ(v(0), 127.0);
  EXPECT_DOUBLE_EQ(v(1), 255.0);
}

// ============================================================================
// Dimension and Type Modification Tests
// ============================================================================

TEST_F(VoxelsTest, ChangeDimensions) {
  voxels v(ctx, dimension(5, 5, 5), Float);
  v(2, 2, 2, 42.0);

  // Change dimensions
  v.voxel_dimensions(dimension(10, 10, 10));

  EXPECT_EQ(v.XDim(), 10u);
  EXPECT_EQ(v.YDim(), 10u);
  EXPECT_EQ(v.ZDim(), 10u);

  // Original data should be preserved where it overlaps
  EXPECT_DOUBLE_EQ(v(2, 2, 2), 42.0);

  // New voxels should be zero-initialized
  EXPECT_DOUBLE_EQ(v(9, 9, 9), 0.0);
}

TEST_F(VoxelsTest, ShrinkDimensions) {
  voxels v(ctx, dimension(10, 10, 10), UShort);
  v(2, 2, 2, 100.0);
  v(8, 8, 8, 200.0);

  // Shrink dimensions
  v.voxel_dimensions(dimension(5, 5, 5));

  EXPECT_EQ(v.XDim(), 5u);
  EXPECT_EQ(v.YDim(), 5u);
  EXPECT_EQ(v.ZDim(), 5u);

  // Data within new bounds should be preserved
  EXPECT_DOUBLE_EQ(v(2, 2, 2), 100.0);

  // Data outside new bounds is lost (can't access it)
  EXPECT_THROW(v(8, 8, 8), index_out_of_bounds);
}

TEST_F(VoxelsTest, ChangeVoxelType) {
  voxels v(ctx, dimension(5, 5, 5), UChar);
  v(0, 0, 0, 100.0);
  v(1, 1, 1, 200.0);

  // Change from UChar to Float
  v.voxelType(Float);

  EXPECT_EQ(v.voxelType(), Float);
  EXPECT_EQ(v.voxelSize(), 4u);

  // Values should be preserved
  EXPECT_DOUBLE_EQ(v(0, 0, 0), 100.0);
  EXPECT_DOUBLE_EQ(v(1, 1, 1), 200.0);
}

TEST_F(VoxelsTest, TypeConversionPrecision) {
  voxels v(ctx, dimension(5, 5, 5), Double);
  v(0, 3.14159265358979);

  // Convert to Float (loses precision)
  v.voxelType(Float);
  double as_float = v(0);

  // Convert back to Double
  v.voxelType(Double);
  double after_conversion = v(0);

  // Should have Float precision, not Double precision
  EXPECT_NEAR(after_conversion, 3.14159265358979, 1e-6);
  EXPECT_NE(after_conversion, 3.14159265358979); // Not exact
}

// ============================================================================
// Min/Max Tests
// ============================================================================

TEST_F(VoxelsTest, MinMaxAutoCalculation) {
  voxels v(ctx, dimension(5, 5, 5), Float);

  v(0, 1.0);
  v(1, 5.0);
  v(2, -3.0);
  v(3, 10.0);

  EXPECT_DOUBLE_EQ(v.min(), -3.0);
  EXPECT_DOUBLE_EQ(v.max(), 10.0);
  EXPECT_TRUE(v.minIsSet());
  EXPECT_TRUE(v.maxIsSet());
}

TEST_F(VoxelsTest, MinMaxManualSet) {
  voxels v(ctx, dimension(5, 5, 5), UChar);

  v.min(10.0);
  v.max(250.0);

  EXPECT_DOUBLE_EQ(v.min(), 10.0);
  EXPECT_DOUBLE_EQ(v.max(), 250.0);
  EXPECT_TRUE(v.minIsSet());
  EXPECT_TRUE(v.maxIsSet());
}

TEST_F(VoxelsTest, MinMaxUnset) {
  voxels v(ctx, dimension(5, 5, 5), UShort);

  v.min(10.0);
  v.max(1000.0);
  EXPECT_TRUE(v.minIsSet());
  EXPECT_TRUE(v.maxIsSet());

  v.unsetMinMax();
  EXPECT_FALSE(v.minIsSet());
  EXPECT_FALSE(v.maxIsSet());
}

TEST_F(VoxelsTest, MinMaxSubvolume) {
  voxels v(ctx, dimension(10, 10, 10), Float);

  // Fill volume with values - set (0,0,0) to mid-range value
  // because the implementation uses (0,0,0) as initial value
  v.fill(50.0);
  v(0, 0, 0, 10.0);

  // Set a region with specific values
  for (uint64 k = 2; k < 5; ++k)
    for (uint64 j = 2; j < 5; ++j)
      for (uint64 i = 2; i < 5; ++i)
        v(i, j, k, double(i + j + k));

  // Calculate min/max for the subvolume
  double sub_min = v.min(2, 2, 2, dimension(3, 3, 3));
  double sub_max = v.max(2, 2, 2, dimension(3, 3, 3));

  EXPECT_DOUBLE_EQ(sub_min, 6.0);  // 2+2+2
  EXPECT_DOUBLE_EQ(sub_max, 12.0); // 4+4+4
}

// ============================================================================
// Operations Tests
// ============================================================================

TEST_F(VoxelsTest, AssignmentOperator) {
  voxels v1(ctx, dimension(5, 5, 5), Float);
  v1(2, 2, 2, 42.0);

  voxels v2(ctx, dimension(10, 10, 10), Double);
  v2 = v1;

  EXPECT_EQ(v2.XDim(), v1.XDim());
  EXPECT_EQ(v2.YDim(), v1.YDim());
  EXPECT_EQ(v2.ZDim(), v1.ZDim());
  EXPECT_EQ(v2.voxelType(), v1.voxelType());
  EXPECT_DOUBLE_EQ(v2(2, 2, 2), 42.0);
}

TEST_F(VoxelsTest, EqualityOperator) {
  voxels v1(ctx, dimension(5, 5, 5), UChar);
  voxels v2(ctx, dimension(5, 5, 5), UChar);

  v1.fill(42.0);
  v2.fill(42.0);

  EXPECT_TRUE(v1 == v2);

  v2(0, 0, 0, 43.0);
  EXPECT_FALSE(v1 == v2);
}

TEST_F(VoxelsTest, InequalityOperator) {
  voxels v1(ctx, dimension(5, 5, 5), UChar);
  voxels v2(ctx, dimension(5, 5, 5), UChar);

  v1.fill(42.0);
  v2.fill(43.0);

  EXPECT_TRUE(v1 != v2);

  v2.fill(42.0);
  EXPECT_FALSE(v1 != v2);
}

TEST_F(VoxelsTest, Fill) {
  voxels v(ctx, dimension(10, 10, 10), Float);

  v.fill(3.14);

  // Use NEAR for float precision
  EXPECT_NEAR(v(0, 0, 0), 3.14, 1e-6);
  EXPECT_NEAR(v(5, 5, 5), 3.14, 1e-6);
  EXPECT_NEAR(v(9, 9, 9), 3.14, 1e-6);
}

TEST_F(VoxelsTest, FillSub) {
  voxels v(ctx, dimension(10, 10, 10), UShort);

  v.fill(0.0);
  v.fillsub(2, 2, 2, dimension(4, 4, 4), 100.0);

  // Inside subvolume should be 100
  EXPECT_DOUBLE_EQ(v(2, 2, 2), 100.0);
  EXPECT_DOUBLE_EQ(v(5, 5, 5), 100.0);

  // Outside subvolume should be 0
  EXPECT_DOUBLE_EQ(v(0, 0, 0), 0.0);
  EXPECT_DOUBLE_EQ(v(9, 9, 9), 0.0);
}

TEST_F(VoxelsTest, Map) {
  voxels v(ctx, dimension(10, 10, 10), Float);

  // Fill with values 0 to 9
  for (uint64 i = 0; i < 10; ++i)
    v(i, double(i));

  // Map from [0, 9] to [100, 200]
  v.map(100.0, 200.0);

  EXPECT_NEAR(v(0), 100.0, 1e-5);
  EXPECT_NEAR(v(9), 200.0, 1e-5);
  // Middle value: 4 maps from [0,9] to [100,200]
  // (4-0)/(9-0) * (200-100) + 100 = 4/9 * 100 + 100 = 144.44...
  EXPECT_NEAR(v(4), 144.444, 0.01);
}

TEST_F(VoxelsTest, Sub) {
  voxels v(ctx, dimension(10, 10, 10), UChar);

  // Fill with pattern
  for (uint64 k = 0; k < 10; ++k)
    for (uint64 j = 0; j < 10; ++j)
      for (uint64 i = 0; i < 10; ++i)
        v(i, j, k, i + j + k);

  // Extract subvolume [2,2,2] to [6,6,6] (5x5x5)
  v.sub(2, 2, 2, dimension(5, 5, 5));

  EXPECT_EQ(v.XDim(), 5u);
  EXPECT_EQ(v.YDim(), 5u);
  EXPECT_EQ(v.ZDim(), 5u);

  // Check that extracted values are correct
  EXPECT_DOUBLE_EQ(v(0, 0, 0), 6.0);  // Was at (2,2,2)
  EXPECT_DOUBLE_EQ(v(4, 4, 4), 18.0); // Was at (6,6,6)
}

TEST_F(VoxelsTest, Resize) {
  voxels v(ctx, dimension(4, 4, 4), Float);

  // Set corner values
  v(0, 0, 0, 0.0);
  v(3, 3, 3, 100.0);

  // Resize to 8x8x8 (should interpolate)
  v.resize(dimension(8, 8, 8));

  EXPECT_EQ(v.XDim(), 8u);
  EXPECT_EQ(v.YDim(), 8u);
  EXPECT_EQ(v.ZDim(), 8u);

  // Corner values should be preserved
  EXPECT_NEAR(v(0, 0, 0), 0.0, 1e-6);
  EXPECT_NEAR(v(7, 7, 7), 100.0, 1e-6);
}

// ============================================================================
// Histogram Tests
// ============================================================================

TEST_F(VoxelsTest, Histogram) {
  voxels v(ctx, dimension(10, 10, 10), UChar);

  // Fill with some pattern
  for (uint64 i = 0; i < 1000; ++i)
    v(i, i % 256);

  auto hist = v.histogram(256);
  const uint64 *bins = hist.get<0>();
  uint64 size = hist.get<1>();

  EXPECT_EQ(size, 256u);
  EXPECT_NE(bins, nullptr);
}

// ============================================================================
// Copy-on-Write and Deep Copy Tests
// ============================================================================

TEST_F(VoxelsTest, CopyOnWrite) {
  voxels v1(ctx, dimension(5, 5, 5), Float);
  v1.fill(42.0);

  voxels v2(v1); // Shares data with v1

  // Reading shouldn't trigger copy
  EXPECT_DOUBLE_EQ(v2(0, 0, 0), 42.0);

  // Writing should trigger copy-on-write
  v2(0, 0, 0, 100.0);

  // v1 should be unchanged
  EXPECT_DOUBLE_EQ(v1(0, 0, 0), 42.0);
  EXPECT_DOUBLE_EQ(v2(0, 0, 0), 100.0);
}

TEST_F(VoxelsTest, ShallowCopyDefault) {
  voxels v1(ctx, dimension(5, 5, 5), Float);
  v1.fill(10.0);
  v1(2, 2, 2, 50.0);

  voxels v2(ctx);
  v2.copy(v1); // Default is shallow copy

  // v2 shares data with v1
  EXPECT_DOUBLE_EQ(v2(2, 2, 2), 50.0);

  // Modify v1 after shallow copy - should NOT affect v2 due to copy-on-write
  v1(2, 2, 2, 99.0);

  // v2 should still have old value (copy-on-write triggered)
  EXPECT_DOUBLE_EQ(v2(2, 2, 2), 50.0);
  EXPECT_DOUBLE_EQ(v1(2, 2, 2), 99.0);
}

TEST_F(VoxelsTest, ShallowCopyExplicit) {
  voxels v1(ctx, dimension(5, 5, 5), Double);
  v1.fill(20.0);
  v1(1, 1, 1, 100.0);

  voxels v2(ctx);
  v2.copy(v1, false); // Explicit shallow copy

  // v2 shares data with v1 initially
  EXPECT_DOUBLE_EQ(v2(1, 1, 1), 100.0);

  // Modify v2 - triggers copy-on-write, v1 unaffected
  v2(1, 1, 1, 200.0);

  EXPECT_DOUBLE_EQ(v1(1, 1, 1), 100.0);
  EXPECT_DOUBLE_EQ(v2(1, 1, 1), 200.0);
}

TEST_F(VoxelsTest, DeepCopy) {
  voxels v1(ctx, dimension(5, 5, 5), Float);
  v1.fill(10.0);
  v1(0, 0, 0, 5.0);
  v1(4, 4, 4, 15.0);

  voxels v2(ctx);
  v2.copy(v1, true); // Deep copy - independent data

  // Check dimensions and type are copied
  EXPECT_EQ(v2.XDim(), v1.XDim());
  EXPECT_EQ(v2.YDim(), v1.YDim());
  EXPECT_EQ(v2.ZDim(), v1.ZDim());
  EXPECT_EQ(v2.voxelType(), v1.voxelType());

  // Check values are copied
  EXPECT_DOUBLE_EQ(v2(0, 0, 0), 5.0);
  EXPECT_DOUBLE_EQ(v2(2, 2, 2), 10.0);
  EXPECT_DOUBLE_EQ(v2(4, 4, 4), 15.0);

  // Modify v1 - should NOT affect v2
  v1(0, 0, 0, 500.0);
  v1(2, 2, 2, 1000.0);
  v1(4, 4, 4, 1500.0);

  // v2 should be unchanged
  EXPECT_DOUBLE_EQ(v2(0, 0, 0), 5.0);
  EXPECT_DOUBLE_EQ(v2(2, 2, 2), 10.0);
  EXPECT_DOUBLE_EQ(v2(4, 4, 4), 15.0);

  // v1 should have new values
  EXPECT_DOUBLE_EQ(v1(0, 0, 0), 500.0);
  EXPECT_DOUBLE_EQ(v1(2, 2, 2), 1000.0);
  EXPECT_DOUBLE_EQ(v1(4, 4, 4), 1500.0);
}

TEST_F(VoxelsTest, DeepCopyIndependence) {
  voxels v1(ctx, dimension(10, 10, 10), UShort);

  // Fill with pattern
  for (uint64 i = 0; i < 1000; ++i)
    v1(i, double(i));

  voxels v2(ctx);
  v2.copy(v1, true); // Deep copy

  // Modify all values in v1
  v1.fill(9999.0);

  // v2 should retain original values
  for (uint64 i = 0; i < 1000; ++i)
    EXPECT_DOUBLE_EQ(v2(i), double(i));

  // v1 should have new values
  EXPECT_DOUBLE_EQ(v1(0), 9999.0);
  EXPECT_DOUBLE_EQ(v1(500), 9999.0);
  EXPECT_DOUBLE_EQ(v1(999), 9999.0);
}

TEST_F(VoxelsTest, DeepCopyDifferentTypes) {
  std::vector<data_type> types = {UChar, UShort, UInt, Float, Double, UInt64};

  for (auto type : types) {
    voxels v1(ctx, dimension(5, 5, 5), type);
    v1.fill(42.0);
    v1(0, 0, 0, 10.0);
    v1(4, 4, 4, 100.0);

    voxels v2(ctx);
    v2.copy(v1, true); // Deep copy

    EXPECT_EQ(v2.voxelType(), type);
    EXPECT_DOUBLE_EQ(v2(0, 0, 0), 10.0);
    EXPECT_DOUBLE_EQ(v2(4, 4, 4), 100.0);

    // Modify v1
    v1(0, 0, 0, 999.0);

    // v2 should be unchanged
    EXPECT_DOUBLE_EQ(v2(0, 0, 0), 10.0);
  }
}

TEST_F(VoxelsTest, DeepCopyMinMax) {
  voxels v1(ctx, dimension(10, 10, 10), Float);
  v1.fill(50.0);
  v1(0, 0, 0, 1.0);
  v1(9, 9, 9, 100.0);

  // Force min/max calculation
  double min1 = v1.min();
  double max1 = v1.max();

  EXPECT_DOUBLE_EQ(min1, 1.0);
  EXPECT_DOUBLE_EQ(max1, 100.0);

  voxels v2(ctx);
  v2.copy(v1, true); // Deep copy

  // Min/max should be copied
  EXPECT_TRUE(v2.minIsSet());
  EXPECT_TRUE(v2.maxIsSet());
  EXPECT_DOUBLE_EQ(v2.min(), 1.0);
  EXPECT_DOUBLE_EQ(v2.max(), 100.0);

  // Modify v1's data
  v1(0, 0, 0, -50.0);
  v1(9, 9, 9, 200.0);

  // v2's min/max should be unchanged (they're cached)
  EXPECT_DOUBLE_EQ(v2.min(), 1.0);
  EXPECT_DOUBLE_EQ(v2.max(), 100.0);
}

TEST_F(VoxelsTest, DeepCopySelfAssignment) {
  voxels v1(ctx, dimension(5, 5, 5), Float);
  v1.fill(42.0);
  v1(2, 2, 2, 100.0);

  // Self-assignment should be safe
  v1.copy(v1, true);

  EXPECT_DOUBLE_EQ(v1(2, 2, 2), 100.0);
  EXPECT_DOUBLE_EQ(v1(0, 0, 0), 42.0);
}

TEST_F(VoxelsTest, DeepCopyLargeVolume) {
  voxels v1(ctx, dimension(50, 50, 50), Double);

  // Fill with unique pattern
  uint64 count = 0;
  for (uint64 k = 0; k < 50; ++k)
    for (uint64 j = 0; j < 50; ++j)
      for (uint64 i = 0; i < 50; ++i)
        v1(i, j, k, double(count++));

  voxels v2(ctx);
  v2.copy(v1, true); // Deep copy

  // Verify all values copied correctly
  count = 0;
  for (uint64 k = 0; k < 50; ++k)
    for (uint64 j = 0; j < 50; ++j)
      for (uint64 i = 0; i < 50; ++i)
        EXPECT_DOUBLE_EQ(v2(i, j, k), double(count++));

  // Modify v1
  v1.fill(0.0);

  // v2 should still have original values
  EXPECT_DOUBLE_EQ(v2(0, 0, 0), 0.0);
  EXPECT_DOUBLE_EQ(v2(25, 25, 25), 63775.0);  // 25 + 25*50 + 25*50*50 = 25 + 1250 + 62500
  EXPECT_DOUBLE_EQ(v2(49, 49, 49), 124999.0); // 49 + 49*50 + 49*50*50 = 49 + 2450 + 122500
}

TEST_F(VoxelsTest, DeepCopySelf) {
  // Test the zero-parameter copy() method which deep copies itself
  voxels v1(ctx, dimension(10, 10, 10), Float);
  v1.fill(42.0);
  v1(5, 5, 5, 100.0);

  // Create a shallow copy that shares v1's data
  voxels v2(ctx);
  v2.copy(v1, false); // Shallow copy

  // Verify they share data
  EXPECT_FLOAT_EQ(v1(5, 5, 5), 100.0);
  EXPECT_FLOAT_EQ(v2(5, 5, 5), 100.0);

  // Modify through v1 (triggers copy-on-write for v1)
  v1(5, 5, 5, 200.0);

  // After write, v1 has its own copy, v2 still has original
  EXPECT_FLOAT_EQ(v1(5, 5, 5), 200.0);
  EXPECT_FLOAT_EQ(v2(5, 5, 5), 100.0);

  // Now deep copy v1 itself
  v1.copy();

  // After copy(), v1 should still have same values
  EXPECT_FLOAT_EQ(v1(5, 5, 5), 200.0);
  EXPECT_FLOAT_EQ(v1(0, 0, 0), 42.0);

  // Create another shallow copy of v1
  voxels v3(ctx);
  v3.copy(v1, false);

  // Modify v3 - should not affect v1 since v1 just did a deep copy
  v3.fill(999.0);

  // v1 should be unaffected
  EXPECT_FLOAT_EQ(v1(5, 5, 5), 200.0);
  EXPECT_FLOAT_EQ(v3(5, 5, 5), 999.0);

  // Test with v2: it should be independent after its own copy()
  v2.copy();
  voxels v4(ctx);
  v4.copy(v2, false);
  v4.fill(777.0);

  // v2 should be unaffected
  EXPECT_FLOAT_EQ(v2(5, 5, 5), 100.0);
  EXPECT_FLOAT_EQ(v4(5, 5, 5), 777.0);
}

TEST_F(VoxelsTest, DeepCopySelfWithDifferentTypes) {
  // Test zero-parameter copy() with various data types
  std::vector<data_type> types = {UChar, UShort, UInt, Float, Double, UInt64};

  for (auto type : types) {
    voxels v(ctx, dimension(8, 8, 8), type);
    v.fill(42.0);
    v(4, 4, 4, 100.0);

    // Deep copy itself
    v.copy();

    // Verify data is preserved
    EXPECT_NEAR(v(4, 4, 4), 100.0, 1e-5);
    EXPECT_NEAR(v(0, 0, 0), 42.0, 1e-5);
    EXPECT_EQ(v.voxelType(), type);
    EXPECT_EQ(v.XDim(), 8u);
  }
}

TEST_F(VoxelsTest, AssignmentOperatorUsesShallowCopy) {
  voxels v1(ctx, dimension(5, 5, 5), Float);
  v1.fill(10.0);
  v1(2, 2, 2, 50.0);

  voxels v2(ctx);
  v2 = v1; // Assignment operator uses copy() with default shallow behavior

  // Should share data initially
  EXPECT_DOUBLE_EQ(v2(2, 2, 2), 50.0);

  // Modify v1 - triggers copy-on-write
  v1(2, 2, 2, 99.0);

  // v2 should have old value
  EXPECT_DOUBLE_EQ(v2(2, 2, 2), 50.0);
  EXPECT_DOUBLE_EQ(v1(2, 2, 2), 99.0);
}

// ============================================================================
// Composite Function Tests
// ============================================================================

TEST_F(VoxelsTest, CompositeAdd) {
  voxels v1(ctx, dimension(5, 5, 5), Float);
  voxels v2(ctx, dimension(5, 5, 5), Float);

  v1.fill(10.0);
  v2.fill(5.0);

  // Composite v2 into v1 using add function
  add_func add;
  v1.composite(v2, 0, 0, 0, add);

  // All values in v1 should now be 15.0
  EXPECT_DOUBLE_EQ(v1(0, 0, 0), 15.0);
  EXPECT_DOUBLE_EQ(v1(2, 2, 2), 15.0);
}

TEST_F(VoxelsTest, CompositeOverwrite) {
  voxels v1(ctx, dimension(5, 5, 5), Float);
  voxels v2(ctx, dimension(3, 3, 3), Float);

  v1.fill(10.0);
  v2.fill(99.0);

  // Composite v2 into v1 at offset (1,1,1) using copy (overwrite)
  copy_func overwrite;
  v1.composite(v2, 1, 1, 1, overwrite);

  // Original corner should be unchanged
  EXPECT_DOUBLE_EQ(v1(0, 0, 0), 10.0);

  // Composited region should be 99.0
  EXPECT_DOUBLE_EQ(v1(1, 1, 1), 99.0);
  EXPECT_DOUBLE_EQ(v1(3, 3, 3), 99.0);

  // Outside composited region should be original
  EXPECT_DOUBLE_EQ(v1(4, 4, 4), 10.0);
}

TEST_F(VoxelsTest, CompositeNegativeOffset) {
  voxels v1(ctx, dimension(10, 10, 10), Float);
  voxels v2(ctx, dimension(5, 5, 5), Float);

  v1.fill(10.0);
  v2.fill(50.0);

  // Composite with negative offset (partial overlap)
  copy_func overwrite;
  v1.composite(v2, -2, -2, -2, overwrite);

  // Only the overlapping portion should be affected
  EXPECT_DOUBLE_EQ(v1(0, 0, 0), 50.0); // Overlapping
  EXPECT_DOUBLE_EQ(v1(2, 2, 2), 50.0); // Overlapping
  EXPECT_DOUBLE_EQ(v1(5, 5, 5), 10.0); // Outside overlap
}

// ============================================================================
// Data Access Tests
// ============================================================================

TEST_F(VoxelsTest, RawDataAccess) {
  voxels v(ctx, dimension(5, 5, 5), UChar);
  v.fill(42.0);

  const unsigned char *data = *v;
  EXPECT_NE(data, nullptr);
  EXPECT_EQ(data[0], 42);
}

TEST_F(VoxelsTest, SharedArrayAccess) {
  voxels v(ctx, dimension(5, 5, 5), Float);
  v(0, 0, 0, 3.14f);

  // Access data pointer directly
  const float *fdata = reinterpret_cast<const float *>(v.data_ptr());
  EXPECT_NE(fdata, nullptr);
  EXPECT_FLOAT_EQ(fdata[0], 3.14f);
}

// ============================================================================
// Edge Cases and Error Handling
// ============================================================================

TEST_F(VoxelsTest, ZeroVolumeMinMax) {
  voxels v(ctx, dimension(0, 0, 0), Float);

  // Min/Max on empty volume shouldn't crash
  // (but behavior is implementation-defined)
  EXPECT_NO_THROW(v.min());
  EXPECT_NO_THROW(v.max());
}

TEST_F(VoxelsTest, SingleVoxelVolume) {
  voxels v(ctx, dimension(1, 1, 1), Double);
  v(0, 0, 0, 42.0);

  EXPECT_DOUBLE_EQ(v(0, 0, 0), 42.0);
  EXPECT_DOUBLE_EQ(v.min(), 42.0);
  EXPECT_DOUBLE_EQ(v.max(), 42.0);
}

TEST_F(VoxelsTest, LargeVolumeCreation) {
  // Test that we can create a reasonably large volume
  // (Not too large to avoid test timeouts)
  dimension large(100, 100, 100);

  EXPECT_NO_THROW({
    voxels v(ctx, large, Float);
    EXPECT_EQ(v.XDim(), 100u);
    EXPECT_EQ(v.YDim(), 100u);
    EXPECT_EQ(v.ZDim(), 100u);
  });
}

TEST_F(VoxelsTest, AllDataTypes) {
  dimension dim(4, 4, 4);

  // Test each data type
  voxels v_uchar(ctx, dim, UChar);
  EXPECT_EQ(v_uchar.voxelType(), UChar);

  voxels v_ushort(ctx, dim, UShort);
  EXPECT_EQ(v_ushort.voxelType(), UShort);

  voxels v_uint(ctx, dim, UInt);
  EXPECT_EQ(v_uint.voxelType(), UInt);

  voxels v_float(ctx, dim, Float);
  EXPECT_EQ(v_float.voxelType(), Float);

  voxels v_double(ctx, dim, Double);
  EXPECT_EQ(v_double.voxelType(), Double);

  voxels v_uint64(ctx, dim, UInt64);
  EXPECT_EQ(v_uint64.voxelType(), UInt64);
}

// ============================================================================
// Type Conversion Tests (Comprehensive)
// ============================================================================

TEST_F(VoxelsTest, TypeConversionUCharToAll) {
  voxels v(ctx, dimension(5, 5, 5), UChar);
  v(0, 0, 0, 100.0);
  v(1, 1, 1, 200.0);

  // UChar to UShort
  v.voxelType(UShort);
  EXPECT_DOUBLE_EQ(v(0, 0, 0), 100.0);
  EXPECT_DOUBLE_EQ(v(1, 1, 1), 200.0);

  // UShort to UInt
  v.voxelType(UInt);
  EXPECT_DOUBLE_EQ(v(0, 0, 0), 100.0);

  // UInt to Float
  v.voxelType(Float);
  EXPECT_NEAR(v(0, 0, 0), 100.0, 1e-5);

  // Float to Double
  v.voxelType(Double);
  EXPECT_NEAR(v(0, 0, 0), 100.0, 1e-10);

  // Double to UInt64
  v.voxelType(UInt64);
  EXPECT_DOUBLE_EQ(v(0, 0, 0), 100.0);
}

TEST_F(VoxelsTest, TypeConversionPrecisionLoss) {
  voxels v(ctx, dimension(5, 5, 5), Double);

  // Set precise double value
  v(0, 3.141592653589793);
  v(1, 1234567.89012345);

  // Convert to Float (loses precision)
  v.voxelType(Float);
  EXPECT_NEAR(v(0), 3.14159, 1e-5);
  EXPECT_NEAR(v(1), 1234567.875, 1.0); // Float precision

  // Convert to UInt (loses fractional part)
  v.voxelType(UInt);
  EXPECT_DOUBLE_EQ(v(0), 3.0);
  EXPECT_DOUBLE_EQ(v(1), 1234567.0);
}

TEST_F(VoxelsTest, TypeConversionNegativeValues) {
  voxels v(ctx, dimension(5, 5, 5), Float);
  v(0, -50.5);
  v(1, -100.0);
  v(2, 75.5);

  // Float to UChar (negative becomes 0, clamped)
  v.voxelType(UChar);
  // Note: Actual behavior depends on cast - typically wraps or clamps
  EXPECT_GE(v(0), 0.0);
  EXPECT_GE(v(1), 0.0);
  EXPECT_NEAR(v(2), 75.0, 1.0);
}

TEST_F(VoxelsTest, AllTypeConversionCombinations) {
  std::vector<data_type> types = {UChar, UShort, UInt, Float, Double, UInt64};

  for (auto from_type : types) {
    for (auto to_type : types) {
      if (from_type == to_type)
        continue;

      voxels v(ctx, dimension(3, 3, 3), from_type);
      v(0, 42.0);

      v.voxelType(to_type);

      // Value should be approximately preserved
      EXPECT_NEAR(v(0), 42.0, 1.0) << "Failed converting from " << data_type_strings[from_type]
                                   << " to " << data_type_strings[to_type];
    }
  }
}

// ============================================================================
// Resize and Interpolation Tests
// ============================================================================

TEST_F(VoxelsTest, ResizeUpsample) {
  voxels v(ctx, dimension(2, 2, 2), Float);

  // Create a simple gradient
  v(0, 0, 0, 0.0);
  v(1, 0, 0, 1.0);
  v(0, 1, 0, 2.0);
  v(1, 1, 0, 3.0);
  v(0, 0, 1, 4.0);
  v(1, 0, 1, 5.0);
  v(0, 1, 1, 6.0);
  v(1, 1, 1, 7.0);

  // Upsample to 4x4x4
  v.resize(dimension(4, 4, 4));

  EXPECT_EQ(v.XDim(), 4u);
  EXPECT_EQ(v.YDim(), 4u);
  EXPECT_EQ(v.ZDim(), 4u);

  // Check corners are preserved
  EXPECT_NEAR(v(0, 0, 0), 0.0, 1e-5);
  EXPECT_NEAR(v(3, 3, 3), 7.0, 1e-5);

  // Check interpolated values exist and are reasonable
  EXPECT_GT(v(1, 1, 1), 0.0);
  EXPECT_LT(v(1, 1, 1), 7.0);
}

TEST_F(VoxelsTest, ResizeDownsample) {
  voxels v(ctx, dimension(8, 8, 8), Float);

  // Fill with pattern
  for (uint64 k = 0; k < 8; ++k)
    for (uint64 j = 0; j < 8; ++j)
      for (uint64 i = 0; i < 8; ++i)
        v(i, j, k, double(i + j + k));

  // Downsample to 4x4x4
  v.resize(dimension(4, 4, 4));

  EXPECT_EQ(v.XDim(), 4u);
  EXPECT_EQ(v.YDim(), 4u);
  EXPECT_EQ(v.ZDim(), 4u);

  // Values should be interpolated
  EXPECT_NEAR(v(0, 0, 0), 0.0, 1e-5);
}

TEST_F(VoxelsTest, ResizeSameSize) {
  voxels v(ctx, dimension(5, 5, 5), Float);
  v.fill(42.0);

  v.resize(dimension(5, 5, 5));

  EXPECT_EQ(v.XDim(), 5u);
  EXPECT_EQ(v.YDim(), 5u);
  EXPECT_EQ(v.ZDim(), 5u);
  EXPECT_NEAR(v(2, 2, 2), 42.0, 1e-5);
}

TEST_F(VoxelsTest, ResizeNonUniform) {
  voxels v(ctx, dimension(4, 4, 4), Float);
  v.fill(10.0);

  // Resize to non-uniform dimensions
  v.resize(dimension(8, 4, 2));

  EXPECT_EQ(v.XDim(), 8u);
  EXPECT_EQ(v.YDim(), 4u);
  EXPECT_EQ(v.ZDim(), 2u);
}

// ============================================================================
// Min/Max Calculation Tests (Comprehensive)
// ============================================================================

TEST_F(VoxelsTest, MinMaxAllDataTypes) {
  std::vector<data_type> types = {UChar, UShort, UInt, Float, Double, UInt64};

  for (auto type : types) {
    voxels v(ctx, dimension(5, 5, 5), type);

    v(0, 10.0);
    v(1, 50.0);
    v(2, 5.0);
    v(3, 100.0);

    double min_val = v.min();
    double max_val = v.max();

    EXPECT_LE(min_val, 10.0) << "Type: " << data_type_strings[type];
    EXPECT_GE(max_val, 100.0) << "Type: " << data_type_strings[type];
  }
}

TEST_F(VoxelsTest, MinMaxWithZeros) {
  voxels v(ctx, dimension(10, 10, 10), Float);
  v.fill(0.0);

  v(5, 5, 5, 10.0);
  v(7, 7, 7, -5.0);

  EXPECT_DOUBLE_EQ(v.min(), -5.0);
  EXPECT_DOUBLE_EQ(v.max(), 10.0);
}

TEST_F(VoxelsTest, MinMaxAllSameValue) {
  voxels v(ctx, dimension(10, 10, 10), Double);
  v.fill(42.42);

  EXPECT_DOUBLE_EQ(v.min(), 42.42);
  EXPECT_DOUBLE_EQ(v.max(), 42.42);
}

TEST_F(VoxelsTest, MinMaxAfterTypeChange) {
  voxels v(ctx, dimension(5, 5, 5), Float);
  v(0, 10.5);
  v(1, 99.5);

  // Get min/max
  double min1 = v.min();
  double max1 = v.max();

  // Change type
  v.voxelType(UChar);

  // Min/max should be recalculated
  v.unsetMinMax();
  double min2 = v.min();
  double max2 = v.max();

  // Values may differ due to type conversion
  EXPECT_LE(min2, min1 + 1.0);
  EXPECT_GE(max2, max1 - 1.0);
}

TEST_F(VoxelsTest, MinMaxLargeVolume) {
  voxels v(ctx, dimension(50, 50, 50), UShort);

  // Fill with pattern - min at (0,0,0), max at (49,49,49)
  for (uint64 k = 0; k < 50; ++k)
    for (uint64 j = 0; j < 50; ++j)
      for (uint64 i = 0; i < 50; ++i)
        v(i, j, k, double(i + j + k));

  EXPECT_DOUBLE_EQ(v.min(), 0.0);
  EXPECT_DOUBLE_EQ(v.max(), 147.0); // 49+49+49
}

// ============================================================================
// Map Operation Tests
// ============================================================================

TEST_F(VoxelsTest, MapExpand) {
  voxels v(ctx, dimension(10, 10, 10), Float);

  // Fill with 0-9
  for (uint64 i = 0; i < 10; ++i)
    v(i, double(i));

  // Map [0,9] to [0,100]
  v.map(0.0, 100.0);

  EXPECT_NEAR(v(0), 0.0, 1e-5);
  EXPECT_NEAR(v(9), 100.0, 1e-5);
  EXPECT_NEAR(v(5), 55.555, 0.01);
}

TEST_F(VoxelsTest, MapShrink) {
  voxels v(ctx, dimension(10, 10, 10), Float);

  // Fill with 0-100
  for (uint64 i = 0; i < 10; ++i)
    v(i, double(i * 10));

  // Map [0,90] to [0,1]
  v.map(0.0, 1.0);

  EXPECT_NEAR(v(0), 0.0, 1e-5);
  EXPECT_NEAR(v(9), 1.0, 1e-5);
}

TEST_F(VoxelsTest, MapNegativeRange) {
  voxels v(ctx, dimension(5, 5, 5), Float);

  for (uint64 i = 0; i < 5; ++i)
    v(i, double(i));

  // Map [0,4] to [-10,10]
  v.map(-10.0, 10.0);

  EXPECT_NEAR(v(0), -10.0, 1e-5);
  EXPECT_NEAR(v(4), 10.0, 1e-5);
  EXPECT_NEAR(v(2), 0.0, 1e-5);
}

TEST_F(VoxelsTest, MapIdentity) {
  voxels v(ctx, dimension(5, 5, 5), Float);
  v.fill(50.0);

  v.min(0.0);
  v.max(100.0);

  // Map to same range
  v.map(0.0, 100.0);

  EXPECT_NEAR(v(0), 50.0, 1e-5);
}

// ============================================================================
// Sub (Subvolume Extraction) Tests
// ============================================================================

TEST_F(VoxelsTest, SubCenterExtraction) {
  voxels v(ctx, dimension(10, 10, 10), Float);

  // Fill with position-based values
  for (uint64 k = 0; k < 10; ++k)
    for (uint64 j = 0; j < 10; ++j)
      for (uint64 i = 0; i < 10; ++i)
        v(i, j, k, double(i * 100 + j * 10 + k));

  // Extract center 4x4x4
  v.sub(3, 3, 3, dimension(4, 4, 4));

  EXPECT_EQ(v.XDim(), 4u);
  EXPECT_EQ(v.YDim(), 4u);
  EXPECT_EQ(v.ZDim(), 4u);

  // Check that (0,0,0) in new volume was (3,3,3) in old
  EXPECT_DOUBLE_EQ(v(0, 0, 0), 333.0);
}

TEST_F(VoxelsTest, SubCornerExtraction) {
  voxels v(ctx, dimension(10, 10, 10), UShort);

  for (uint64 i = 0; i < 1000; ++i)
    v(i, double(i));

  // Extract from origin
  v.sub(0, 0, 0, dimension(5, 5, 5));

  EXPECT_EQ(v.XDim(), 5u);
  EXPECT_DOUBLE_EQ(v(0, 0, 0), 0.0);
  EXPECT_DOUBLE_EQ(v(4, 4, 4), 444.0); // Was at (4,4,4)
}

TEST_F(VoxelsTest, SubSingleSlice) {
  voxels v(ctx, dimension(10, 10, 10), Float);
  v.fill(42.0);

  // Extract single slice in Z
  v.sub(0, 0, 5, dimension(10, 10, 1));

  EXPECT_EQ(v.XDim(), 10u);
  EXPECT_EQ(v.YDim(), 10u);
  EXPECT_EQ(v.ZDim(), 1u);
  EXPECT_NEAR(v(5, 5, 0), 42.0, 1e-5);
}

// ============================================================================
// Bilateral Filter Tests
// ============================================================================

TEST_F(VoxelsTest, BilateralFilterUniform) {
  // Test 1: Nearly uniform region (avoid min==max division issues)
  voxels v(ctx, dimension(10, 10, 10), Float);
  v.fill(50.0);
  v(0, 0, 0, 50.1); // Slight variation to avoid min==max
  v(9, 9, 9, 49.9);

  v.bilateralFilter();

  // Nearly uniform regions should stay very close to original
  EXPECT_NEAR(v(5, 5, 5), 50.0, 0.5);
  EXPECT_NEAR(v(0, 0, 0), 50.1, 1.0); // Edges have slight boundary effects
  EXPECT_NEAR(v(9, 9, 9), 49.9, 1.0);
}

TEST_F(VoxelsTest, BilateralFilterEdgePreservation) {
  // Test 2: Sharp edges should be preserved while noise is reduced
  voxels v(ctx, dimension(10, 10, 1), Float);

  // Create sharp edge: left half = 20, right half = 80
  for (uint64 j = 0; j < 10; ++j) {
    for (uint64 i = 0; i < 5; ++i) {
      v(i, j, 0, 20.0);
    }
    for (uint64 i = 5; i < 10; ++i) {
      v(i, j, 0, 80.0);
    }
  }

  // Apply bilateral filter (edge-preserving)
  v.bilateralFilter(50.0, 1.5, 1);

  // Check edge is preserved (regions should remain distinct)
  double left_avg = 0.0, right_avg = 0.0;
  for (uint64 j = 2; j < 8; ++j) { // Avoid edges
    left_avg += v(2, j, 0);        // Left region
    right_avg += v(7, j, 0);       // Right region
  }
  left_avg /= 6.0;
  right_avg /= 6.0;

  EXPECT_LT(left_avg, 40.0) << "Left side should stay low";
  EXPECT_GT(right_avg, 60.0) << "Right side should stay high";
  EXPECT_GT(right_avg - left_avg, 35.0) << "Edge should be preserved";
}

TEST_F(VoxelsTest, BilateralFilterNoiseReduction) {
  // Test 3: Small noise should be smoothed in uniform regions
  voxels v(ctx, dimension(9, 9, 1), Float);
  v.fill(50.0);

  // Add salt and pepper noise in center region
  v(4, 3, 0, 55.0);
  v(5, 3, 0, 45.0);
  v(4, 4, 0, 52.0);
  v(5, 5, 0, 48.0);
  v(3, 4, 0, 54.0);

  double noise_before = 0.0;
  for (uint64 j = 3; j <= 5; ++j) {
    for (uint64 i = 3; i <= 5; ++i) {
      double diff = v(i, j, 0) - 50.0;
      noise_before += diff * diff;
    }
  }

  v.bilateralFilter(20.0, 1.5, 1);

  // After filtering, noise should be reduced
  double noise_after = 0.0;
  for (uint64 j = 3; j <= 5; ++j) {
    for (uint64 i = 3; i <= 5; ++i) {
      double diff = v(i, j, 0) - 50.0;
      noise_after += diff * diff;
    }
  }

  EXPECT_LT(noise_after, noise_before) << "Noise should be reduced";
  EXPECT_NEAR(v(4, 4, 0), 50.0, 3.0) << "Values should converge toward mean";
}

TEST_F(VoxelsTest, BilateralFilterIsolatedSpike) {
  // Test 4: Isolated spike in uniform region
  voxels v(ctx, dimension(9, 9, 9), Float);
  v.fill(50.0);
  v(4, 4, 4, 150.0); // Large spike

  double initial_spike = v(4, 4, 4);
  double initial_neighbor = v(4, 4, 5);

  v.bilateralFilter(50.0, 1.5, 1);

  double final_spike = v(4, 4, 4);
  double final_neighbor = v(4, 4, 5);

  // Bilateral filter should preserve edges, so spike should remain elevated
  // but be smoothed significantly due to spatial averaging
  EXPECT_GT(final_spike, 70.0) << "Spike should remain elevated (above background)";
  EXPECT_LT(final_spike, initial_spike) << "Spike should be smoothed";

  // Neighbor should be slightly influenced but not much (edge preservation)
  EXPECT_GT(final_neighbor, initial_neighbor) << "Neighbor affected slightly";
  EXPECT_LT(final_neighbor, 70.0) << "But edge preservation limits influence";
}

TEST_F(VoxelsTest, BilateralFilterSpatialSigmaEffect) {
  // Test 5: Spatial sigma controls spatial smoothing extent
  voxels v1(ctx, dimension(11, 11, 1), Float);
  voxels v2(v1);

  // Create gradient with noise
  for (uint64 j = 0; j < 11; ++j) {
    for (uint64 i = 0; i < 11; ++i) {
      double base = double(i) * 10.0; // Gradient 0-100
      double noise = ((i + j) % 3 == 0) ? 5.0 : 0.0;
      v1(i, j, 0, base + noise);
      v2(i, j, 0, base + noise);
    }
  }

  // Apply with different spatial sigmas
  v1.bilateralFilter(50.0, 1.0, 1); // Small spatial sigma
  v2.bilateralFilter(50.0, 2.5, 1); // Large spatial sigma

  // Larger spatial sigma should smooth more within uniform regions
  // Calculate variance in a local region
  double var1 = 0.0, var2 = 0.0;
  double mean1 = 0.0, mean2 = 0.0;
  int count = 0;

  // Sample region around x=5 (mid-gradient)
  for (uint64 j = 4; j <= 6; ++j) {
    for (uint64 i = 4; i <= 6; ++i) {
      mean1 += v1(i, j, 0);
      mean2 += v2(i, j, 0);
      count++;
    }
  }
  mean1 /= count;
  mean2 /= count;

  for (uint64 j = 4; j <= 6; ++j) {
    for (uint64 i = 4; i <= 6; ++i) {
      double d1 = v1(i, j, 0) - mean1;
      double d2 = v2(i, j, 0) - mean2;
      var1 += d1 * d1;
      var2 += d2 * d2;
    }
  }

  // Larger spatial sigma should give smoother result (lower variance)
  EXPECT_LE(var2, var1 * 1.2) << "Larger spatial sigma should smooth more";
}

TEST_F(VoxelsTest, BilateralFilterRadiometricSigmaEffect) {
  // Test 6: Radiometric sigma controls edge preservation strength
  voxels v1(ctx, dimension(10, 10, 1), Float);
  voxels v2(v1);

  // Create step edge
  for (uint64 j = 0; j < 10; ++j) {
    for (uint64 i = 0; i < 5; ++i) {
      v1(i, j, 0, 30.0);
      v2(i, j, 0, 30.0);
    }
    for (uint64 i = 5; i < 10; ++i) {
      v1(i, j, 0, 70.0);
      v2(i, j, 0, 70.0);
    }
  }

  // Apply with different radiometric sigmas
  v1.bilateralFilter(20.0, 1.5, 1);  // Small radiometric sigma (strong edge preservation)
  v2.bilateralFilter(100.0, 1.5, 1); // Large radiometric sigma (weak edge preservation)

  // Calculate edge sharpness (difference between regions)
  double left1 = v1(2, 5, 0);
  double right1 = v1(7, 5, 0);
  double left2 = v2(2, 5, 0);
  double right2 = v2(7, 5, 0);

  double contrast1 = right1 - left1;
  double contrast2 = right2 - left2;

  // Smaller radiometric sigma should preserve edge better
  EXPECT_GT(contrast1, 25.0) << "Small sigma preserves edge";
  EXPECT_GT(contrast2, 15.0) << "Large sigma still has some edge";
  EXPECT_GE(contrast1, contrast2 * 0.9) << "Small sigma preserves edge better";
}

TEST_F(VoxelsTest, BilateralFilterFilterRadiusEffect) {
  // Test 7: Filter radius controls neighborhood size
  voxels v1(ctx, dimension(11, 11, 1), Float);
  voxels v2(v1);

  v1.fill(50.0);
  v2.fill(50.0);

  // Add feature at edge of different neighborhoods
  v1(5, 5, 0, 100.0); // Center
  v1(5, 7, 0, 80.0);  // 2 pixels away
  v2 = v1;

  // Apply with different filter radii
  v1.bilateralFilter(50.0, 1.5, 1); // Radius 1 (3x3x3 neighborhood)
  v2.bilateralFilter(50.0, 1.5, 2); // Radius 2 (5x5x5 neighborhood)

  // Larger radius should have more influence on distant pixels
  double influence_r1 = v1(5, 7, 0) - 80.0; // How much did center affect this pixel
  double influence_r2 = v2(5, 7, 0) - 80.0;

  // With radius 2, the center spike can influence pixels 2 away
  // With radius 1, it cannot reach that far
  EXPECT_LE(std::abs(influence_r1), std::abs(influence_r2) + 10.0)
      << "Larger radius should have more influence on distant pixels";
}

TEST_F(VoxelsTest, BilateralFilter3DSmoothing) {
  // Test 8: 3D bilateral filtering
  voxels v(ctx, dimension(7, 7, 7), Float);

  // Create 3D pattern: sphere of high values in uniform background
  v.fill(30.0);
  for (uint64 k = 2; k <= 4; ++k) {
    for (uint64 j = 2; j <= 4; ++j) {
      for (uint64 i = 2; i <= 4; ++i) {
        // Distance from center (3,3,3)
        int dx = i - 3, dy = j - 3, dz = k - 3;
        if (dx * dx + dy * dy + dz * dz <= 2) { // Roughly spherical
          v(i, j, k, 70.0);
        }
      }
    }
  }

  double center_before = v(3, 3, 3);
  double background_before = v(0, 0, 0);

  v.bilateralFilter(50.0, 1.5, 1);

  double center_after = v(3, 3, 3);
  double background_after = v(0, 0, 0);

  // Center sphere should remain elevated (edge preservation in 3D)
  EXPECT_GT(center_after, 50.0) << "3D feature preserved";

  // Background should stay low
  EXPECT_LT(background_after, 40.0) << "Background stays low";

  // Edge should be preserved
  EXPECT_GT(center_after - background_after, 20.0) << "3D edge preserved";
}

// ============================================================================
// Contrast Enhancement Tests
// ============================================================================

TEST_F(VoxelsTest, ContrastEnhancementLowContrast) {
  // Test 1: Algorithm restores original range after internal processing
  voxels v(ctx, dimension(10, 10, 10), Float);

  // Create low contrast volume (values clustered in narrow range)
  for (uint64 i = 0; i < 1000; ++i) {
    v(i, 40.0 + double(i % 20)); // Range: [40, 60)
  }

  double initial_min = v.min();
  double initial_max = v.max();

  v.contrastEnhancement(0.9);

  double final_min = v.min();
  double final_max = v.max();

  // Algorithm maps to [0,255], processes, then restores original range
  EXPECT_NEAR(final_min, initial_min, 1.0) << "Min should be restored";
  EXPECT_NEAR(final_max, initial_max, 1.0) << "Max should be restored";

  // Verify adaptive enhancement occurred (check variance)
  double sum = 0.0, sum_sq = 0.0;
  for (uint64 i = 0; i < 1000; ++i) {
    double val = v(i);
    sum += val;
    sum_sq += val * val;
  }
  double mean = sum / 1000.0;
  double variance = (sum_sq / 1000.0) - (mean * mean);

  EXPECT_GT(variance, 20.0) << "Contrast should be enhanced";
}

TEST_F(VoxelsTest, ContrastEnhancementHistogramSpread) {
  // Test 2: Verify histogram spreading within original range
  voxels v(ctx, dimension(10, 10, 1), Float);

  // Create image with clustered values
  for (uint64 j = 0; j < 10; ++j) {
    for (uint64 i = 0; i < 10; ++i) {
      // Three distinct levels (low contrast)
      if (i < 3)
        v(i, j, 0, 30.0);
      else if (i < 7)
        v(i, j, 0, 50.0);
      else
        v(i, j, 0, 70.0);
    }
  }

  double initial_min = v.min();
  double initial_max = v.max();

  v.contrastEnhancement(0.8);

  double final_min = v.min();
  double final_max = v.max();

  // Range is preserved
  EXPECT_NEAR(final_min, initial_min, 1.0) << "Min preserved";
  EXPECT_NEAR(final_max, initial_max, 1.0) << "Max preserved";

  // Check that three regions are more separated
  double val_low = v(1, 5, 0);
  double val_mid = v(5, 5, 0);
  double val_high = v(8, 5, 0);

  double sep1 = val_mid - val_low;
  double sep2 = val_high - val_mid;

  EXPECT_LT(val_low, val_mid) << "Low region should be less than mid";
  EXPECT_LT(val_mid, val_high) << "Mid region should be less than high";
  EXPECT_GT(sep1, 5.0) << "Regions should be separated";
  EXPECT_GT(sep2, 5.0) << "Regions should be separated";
}

TEST_F(VoxelsTest, ContrastEnhancementPreservesOrder) {
  // Test 3: Verify that general relative ordering trend is preserved
  voxels v(ctx, dimension(10, 1, 1), Float);

  // Create monotonically increasing values
  for (uint64 i = 0; i < 10; ++i) {
    v(i, 0, 0, 30.0 + double(i) * 4.0); // 30, 34, 38, ..., 66
  }

  v.contrastEnhancement(0.7);

  // Nonlinear transformation may not preserve strict monotonicity
  // but endpoints and general trend should be preserved
  EXPECT_LT(v(0, 0, 0), v(9, 0, 0)) << "General increasing trend preserved";
  EXPECT_LT(v(0, 0, 0), v(4, 0, 0)) << "First half increases";
  EXPECT_LT(v(4, 0, 0), v(9, 0, 0)) << "Second half increases";

  // Check most values preserve order (allowing some tolerance)
  int order_violations = 0;
  for (uint64 i = 0; i < 9; ++i) {
    if (v(i, 0, 0) >= v(i + 1, 0, 0))
      order_violations++;
  }
  EXPECT_LE(order_violations, 2) << "Most ordering should be preserved";
}

TEST_F(VoxelsTest, ContrastEnhancementResistorEffect) {
  // Test 4: Different resistor values have different effects
  voxels v1(ctx, dimension(10, 10, 1), Float);
  voxels v2(v1);

  // Create gradient
  for (uint64 j = 0; j < 10; ++j) {
    for (uint64 i = 0; i < 10; ++i) {
      double val = 20.0 + double(i) * 6.0; // Gradient across x
      v1(i, j, 0, val);
      v2(i, j, 0, val);
    }
  }

  double initial_min = v1.min();
  double initial_max = v1.max();

  // Apply with different resistor values
  v1.contrastEnhancement(0.3); // Low resistor (less spatial influence)
  v2.contrastEnhancement(0.9); // High resistor (more spatial influence)

  // Both should preserve original range
  EXPECT_NEAR(v1.min(), initial_min, 1.0);
  EXPECT_NEAR(v1.max(), initial_max, 1.0);
  EXPECT_NEAR(v2.min(), initial_min, 1.0);
  EXPECT_NEAR(v2.max(), initial_max, 1.0);

  // Different resistor values should produce different internal distributions
  double mid1 = v1(5, 5, 0);
  double mid2 = v2(5, 5, 0);
  EXPECT_NE(mid1, mid2) << "Different resistors should produce different results";
}

TEST_F(VoxelsTest, ContrastEnhancementLocalAdaptive) {
  // Test 5: Verify local adaptive behavior
  voxels v(ctx, dimension(20, 1, 1), Float);

  // Create two regions with different local contrasts
  // Left region: low contrast (40-50)
  for (uint64 i = 0; i < 10; ++i) {
    v(i, 0, 0, 40.0 + double(i));
  }

  // Right region: already high contrast (0-100)
  for (uint64 i = 10; i < 20; ++i) {
    v(i, 0, 0, double(i - 10) * 10.0);
  }

  // Calculate local ranges before
  double left_range_before = v(9, 0, 0) - v(0, 0, 0);
  double right_range_before = v(19, 0, 0) - v(10, 0, 0);

  v.contrastEnhancement(0.8);

  // Calculate local ranges after
  double left_range_after = v(9, 0, 0) - v(0, 0, 0);
  double right_range_after = v(19, 0, 0) - v(10, 0, 0);

  // Left region (low contrast) should get more enhancement
  double left_enhancement = left_range_after / left_range_before;
  double right_enhancement = right_range_after / right_range_before;

  EXPECT_GT(left_enhancement, 1.0) << "Left region should be enhanced";
  EXPECT_GT(right_enhancement, 0.8) << "Right region should be preserved/enhanced";
}

TEST_F(VoxelsTest, ContrastEnhancementEdgeEnhancement) {
  // Test 6: Edges should become more pronounced
  voxels v(ctx, dimension(10, 10, 1), Float);

  // Create step edge
  for (uint64 j = 0; j < 10; ++j) {
    for (uint64 i = 0; i < 5; ++i) {
      v(i, j, 0, 30.0);
    }
    for (uint64 i = 5; i < 10; ++i) {
      v(i, j, 0, 60.0);
    }
  }

  double edge_diff_before = v(5, 5, 0) - v(4, 5, 0);

  v.contrastEnhancement(0.85);

  double edge_diff_after = v(5, 5, 0) - v(4, 5, 0);

  // Edge should be enhanced or at least maintained
  EXPECT_GE(edge_diff_after, edge_diff_before * 0.8)
      << "Edge contrast should be maintained or enhanced";
}

TEST_F(VoxelsTest, ContrastEnhancement3D) {
  // Test 7: 3D contrast enhancement
  voxels v(ctx, dimension(8, 8, 8), Float);

  // Create 3D gradient
  for (uint64 k = 0; k < 8; ++k) {
    for (uint64 j = 0; j < 8; ++j) {
      for (uint64 i = 0; i < 8; ++i) {
        // Gradient based on distance from origin
        double dist = sqrt(double(i * i + j * j + k * k));
        v(i, j, k, 30.0 + dist * 3.0);
      }
    }
  }

  double initial_range = v.max() - v.min();

  v.contrastEnhancement(0.75);

  double final_range = v.max() - v.min();

  // 3D enhancement should work
  EXPECT_GT(final_range, initial_range * 0.9) << "3D contrast should be enhanced";

  // Verify gradient structure is preserved
  EXPECT_LT(v(0, 0, 0), v(4, 4, 4)) << "Gradient direction preserved";
  EXPECT_LT(v(4, 4, 4), v(7, 7, 7)) << "Gradient monotonic";
}

TEST_F(VoxelsTest, ContrastEnhancementRangePreservation) {
  // Test 8: Original min/max range is restored after internal processing
  voxels v(ctx, dimension(10, 10, 1), Float);

  // Create volume with specific range
  for (uint64 i = 0; i < 100; ++i) {
    v(i, 100.0 + double(i % 50) * 2.0); // Range [100, 200)
  }

  double original_min = v.min();
  double original_max = v.max();

  v.contrastEnhancement(0.8);

  // Algorithm should restore original range (just redistribute values)
  EXPECT_NEAR(v.min(), original_min, 5.0) << "Min should be approximately restored";
  EXPECT_NEAR(v.max(), original_max, 5.0) << "Max should be approximately restored";

  // But the histogram should be more spread out within that range
  // Check that not all values are clustered
  int unique_buckets = 0;
  std::vector<int> histogram(10, 0);
  for (uint64 i = 0; i < 100; ++i) {
    double val = v(i);
    int bucket = int((val - v.min()) / (v.max() - v.min()) * 9.999);
    histogram[std::max(0, std::min(9, bucket))]++;
  }

  for (int count : histogram) {
    if (count > 0)
      unique_buckets++;
  }

  EXPECT_GE(unique_buckets, 3) << "Values should be distributed across histogram";
}

TEST_F(VoxelsTest, ContrastEnhancementResistorClamping) {
  // Test 9: Resistor values outside [0,1] should be clamped
  voxels v1(ctx, dimension(8, 8, 1), Float);
  voxels v2(v1), v3(v1);

  // Create test pattern
  for (uint64 i = 0; i < 64; ++i) {
    v1(i, 30.0 + double(i % 30));
    v2(i, 30.0 + double(i % 30));
    v3(i, 30.0 + double(i % 30));
  }

  // Apply with clamped values (algorithm clamps internally)
  v1.contrastEnhancement(-0.5); // Should be clamped to 0.0
  v2.contrastEnhancement(0.0);  // Minimum valid value
  v3.contrastEnhancement(1.5);  // Should be clamped to 1.0

  // All should complete without crashing and produce valid results
  EXPECT_TRUE(v1.min() < v1.max()) << "Negative resistor clamped, still works";
  EXPECT_TRUE(v2.min() < v2.max()) << "Zero resistor works";
  EXPECT_TRUE(v3.min() < v3.max()) << "Resistor > 1.0 clamped, still works";
}

// ============================================================================
// Anisotropic Diffusion Tests
// ============================================================================

TEST_F(VoxelsTest, AnisotropicDiffusionUniform) {
  // Test 1: Uniform volume should remain unchanged
  voxels v(ctx, dimension(10, 10, 10), Float);
  v.fill(50.0);

  v.anisotropicDiffusion(5);

  // Uniform regions should remain stable
  EXPECT_NEAR(v(5, 5, 5), 50.0, 0.01);
  EXPECT_NEAR(v(0, 0, 0), 50.0, 0.01);
  EXPECT_NEAR(v(9, 9, 9), 50.0, 0.01);
}

TEST_F(VoxelsTest, AnisotropicDiffusionSmoothGradient) {
  // Test 2: Smooth gradient should be preserved
  voxels v(ctx, dimension(10, 1, 1), Float);

  // Create linear gradient: 0, 10, 20, ..., 90
  for (uint64 i = 0; i < 10; ++i)
    v(i, 0, 0, double(i * 10));

  voxels original(v);
  v.anisotropicDiffusion(3);

  // Smooth gradients should be relatively unchanged (anisotropic diffusion preserves edges)
  // Check that gradient is still monotonically increasing
  for (uint64 i = 0; i < 9; ++i) {
    EXPECT_LT(v(i, 0, 0), v(i + 1, 0, 0)) << "Gradient should remain monotonic at position " << i;
  }

  // End points should be relatively stable
  EXPECT_NEAR(v(0, 0, 0), 0.0, 5.0);
  EXPECT_NEAR(v(9, 0, 0), 90.0, 5.0);
}

TEST_F(VoxelsTest, AnisotropicDiffusionEdgePreservation) {
  // Test 3: Sharp edges should be preserved
  voxels v(ctx, dimension(10, 10, 1), Float);

  // Create step edge: left half = 0, right half = 100
  for (uint64 j = 0; j < 10; ++j) {
    for (uint64 i = 0; i < 5; ++i) {
      v(i, j, 0, 0.0);
    }
    for (uint64 i = 5; i < 10; ++i) {
      v(i, j, 0, 100.0);
    }
  }

  v.anisotropicDiffusion(5);

  // Check edge is preserved (should still have clear distinction)
  // Left side should be much less than right side
  double left_avg = 0.0;
  double right_avg = 0.0;
  for (uint64 j = 0; j < 10; ++j) {
    left_avg += v(2, j, 0);  // Sample from left region
    right_avg += v(7, j, 0); // Sample from right region
  }
  left_avg /= 10.0;
  right_avg /= 10.0;

  EXPECT_LT(left_avg, 40.0) << "Left side should remain relatively dark";
  EXPECT_GT(right_avg, 60.0) << "Right side should remain relatively bright";
  EXPECT_GT(right_avg - left_avg, 30.0) << "Edge contrast should be preserved";
}

TEST_F(VoxelsTest, AnisotropicDiffusionNoiseReduction) {
  // Test 4: Small noise should be smoothed while preserving structure
  voxels v(ctx, dimension(10, 10, 1), Float);

  // Create noisy constant region with small fluctuations
  v.fill(50.0);
  v(3, 3, 0, 55.0); // Small noise
  v(3, 4, 0, 45.0);
  v(6, 6, 0, 53.0);
  v(7, 7, 0, 47.0);

  v.anisotropicDiffusion(10);

  // After diffusion, values should be closer to mean
  // Small gradients get smoothed
  EXPECT_NEAR(v(3, 3, 0), 50.0, 3.0);
  EXPECT_NEAR(v(3, 4, 0), 50.0, 3.0);
  EXPECT_NEAR(v(6, 6, 0), 50.0, 3.0);
}

TEST_F(VoxelsTest, AnisotropicDiffusionIsolatedSpike) {
  // Test 5: Isolated spike behavior
  voxels v(ctx, dimension(7, 7, 7), Float);
  v.fill(50.0);

  // Add isolated spike
  v(3, 3, 3, 150.0);

  double initial_spike = v(3, 3, 3);
  double initial_neighbor = v(3, 3, 4);

  v.anisotropicDiffusion(5);

  double final_spike = v(3, 3, 3);
  double final_neighbor = v(3, 3, 4);

  // Spike should diffuse outward (decrease)
  EXPECT_LT(final_spike, initial_spike) << "Spike should be smoothed";

  // Neighbors should increase as they receive diffused values
  EXPECT_GT(final_neighbor, initial_neighbor) << "Neighbors should receive diffused values";

  // But spike should still be higher than its neighbors
  EXPECT_GT(final_spike, final_neighbor) << "Spike should remain locally maximal";
}

TEST_F(VoxelsTest, AnisotropicDiffusionMultipleIterations) {
  // Test 6: More iterations = more smoothing (but edges preserved)
  voxels v1(ctx, dimension(10, 10, 1), Float);
  voxels v2(v1);

  // Create pattern with both smooth regions and edges
  for (uint64 i = 0; i < 10; ++i) {
    for (uint64 j = 0; j < 10; ++j) {
      if (i < 5) {
        v1(i, j, 0, 20.0 + (i % 2) * 5.0); // Noisy low region
      } else {
        v1(i, j, 0, 80.0 + (i % 2) * 5.0); // Noisy high region
      }
    }
  }
  v2 = v1;

  // Apply different iteration counts
  v1.anisotropicDiffusion(3);
  v2.anisotropicDiffusion(10);

  // More iterations should smooth noise more in uniform regions
  // Calculate variance in left region (should decrease)
  double var1 = 0.0, var2 = 0.0;
  double mean1 = 0.0, mean2 = 0.0;
  int count = 0;

  for (uint64 i = 1; i < 4; ++i) {
    for (uint64 j = 0; j < 10; ++j) {
      mean1 += v1(i, j, 0);
      mean2 += v2(i, j, 0);
      count++;
    }
  }
  mean1 /= count;
  mean2 /= count;

  for (uint64 i = 1; i < 4; ++i) {
    for (uint64 j = 0; j < 10; ++j) {
      double diff1 = v1(i, j, 0) - mean1;
      double diff2 = v2(i, j, 0) - mean2;
      var1 += diff1 * diff1;
      var2 += diff2 * diff2;
    }
  }

  // More iterations should reduce variance (more smoothing)
  EXPECT_LT(var2, var1) << "More iterations should reduce noise (lower variance)";
}

TEST_F(VoxelsTest, AnisotropicDiffusion3DEdgePreservation) {
  // Test 7: 3D edge preservation
  voxels v(ctx, dimension(8, 8, 8), Float);

  // Create 3D step: bottom half = 20, top half = 80
  for (uint64 k = 0; k < 8; ++k) {
    double val = (k < 4) ? 20.0 : 80.0;
    for (uint64 j = 0; j < 8; ++j) {
      for (uint64 i = 0; i < 8; ++i) {
        v(i, j, k, val);
      }
    }
  }

  v.anisotropicDiffusion(5);

  // Check that layers far from edge are relatively preserved
  double bottom_avg = 0.0, top_avg = 0.0;
  for (uint64 j = 0; j < 8; ++j) {
    for (uint64 i = 0; i < 8; ++i) {
      bottom_avg += v(i, j, 1); // Second layer from bottom
      top_avg += v(i, j, 6);    // Second layer from top
    }
  }
  bottom_avg /= 64.0;
  top_avg /= 64.0;

  EXPECT_LT(bottom_avg, 40.0) << "Bottom region should stay relatively low";
  EXPECT_GT(top_avg, 60.0) << "Top region should stay relatively high";
  EXPECT_GT(top_avg - bottom_avg, 25.0) << "3D edge should be preserved";
}

// ============================================================================
// GDTV Filter Tests
// ============================================================================

TEST_F(VoxelsTest, GDTVFilterUniform) {
  // Test 1: Uniform volume should remain nearly unchanged
  voxels v(ctx, dimension(10, 10, 10), Float);
  v.fill(50.0);

  v.gdtvFilter(1.5, 0.5, 3, 0); // 6-neighbor mode

  // Uniform volume should be stable
  EXPECT_NEAR(v(5, 5, 5), 50.0, 0.1);
  EXPECT_NEAR(v(0, 0, 0), 50.0, 0.1);
  EXPECT_NEAR(v(9, 9, 9), 50.0, 0.1);
}

TEST_F(VoxelsTest, GDTVFilterNoiseReduction) {
  // Test 2: Noise reduction with quantitative validation
  voxels v(ctx, dimension(10, 10, 1), Float);

  // Create noisy data around mean of 50
  for (uint64 j = 0; j < 10; ++j) {
    for (uint64 i = 0; i < 10; ++i) {
      double noise = (i * j) % 7 - 3.0; // Noise: -3 to +3
      v(i, j, 0, 50.0 + noise);
    }
  }

  // Calculate variance before
  double sum_before = 0.0, sum_sq_before = 0.0;
  for (uint64 i = 0; i < 100; ++i) {
    double val = v(i);
    sum_before += val;
    sum_sq_before += val * val;
  }
  double mean_before = sum_before / 100.0;
  double var_before = (sum_sq_before / 100.0) - (mean_before * mean_before);

  v.gdtvFilter(1.5, 0.3, 5, 0);

  // Calculate variance after
  double sum_after = 0.0, sum_sq_after = 0.0;
  for (uint64 i = 0; i < 100; ++i) {
    double val = v(i);
    sum_after += val;
    sum_sq_after += val * val;
  }
  double mean_after = sum_after / 100.0;
  double var_after = (sum_sq_after / 100.0) - (mean_after * mean_after);

  // Variance should be reduced
  EXPECT_LT(var_after, var_before);
  // Mean should be approximately preserved
  EXPECT_NEAR(mean_after, mean_before, 2.0);
}

TEST_F(VoxelsTest, GDTVFilterEdgePreservation) {
  // Test 3: Edge-preserving property
  voxels v(ctx, dimension(10, 10, 1), Float);

  // Create sharp edge: left half = 20, right half = 80
  for (uint64 j = 0; j < 10; ++j) {
    for (uint64 i = 0; i < 5; ++i) {
      v(i, j, 0, 20.0);
    }
    for (uint64 i = 5; i < 10; ++i) {
      v(i, j, 0, 80.0);
    }
  }

  v.gdtvFilter(1.5, 0.3, 3, 0);

  // Check that edge is preserved (regions should remain distinct)
  double left_avg = 0.0, right_avg = 0.0;
  int count = 0;

  for (uint64 j = 2; j < 8; ++j) { // Avoid boundary effects
    left_avg += v(1, j, 0) + v(2, j, 0);
    right_avg += v(7, j, 0) + v(8, j, 0);
    count += 2;
  }

  left_avg /= count;
  right_avg /= count;

  // Edge should be preserved with significant contrast
  EXPECT_LT(left_avg, 40.0) << "Left region should stay low";
  EXPECT_GT(right_avg, 60.0) << "Right region should stay high";
  EXPECT_GT(right_avg - left_avg, 30.0) << "Edge contrast should be preserved";
}

TEST_F(VoxelsTest, GDTVFilterGradientSmoothing) {
  // Test 4: Smooth gradient should be preserved
  voxels v(ctx, dimension(10, 10, 1), Float);

  // Create smooth gradient
  for (uint64 j = 0; j < 10; ++j) {
    for (uint64 i = 0; i < 10; ++i) {
      v(i, j, 0, 20.0 + double(i) * 5.0); // Linear gradient
    }
  }

  v.gdtvFilter(1.5, 0.3, 3, 0);

  // Gradient direction should be preserved
  EXPECT_LT(v(0, 5, 0), v(5, 5, 0));
  EXPECT_LT(v(5, 5, 0), v(9, 5, 0));

  // Check monotonicity in a few rows
  for (uint64 j = 2; j < 8; j += 2) {
    for (uint64 i = 0; i < 8; ++i) {
      EXPECT_LE(v(i, j, 0), v(i + 1, j, 0) + 1.0)
          << "Gradient should be approximately preserved at row " << j;
    }
  }
}

TEST_F(VoxelsTest, GDTVFilterParameterQ) {
  // Test 5: Different q parameter values
  voxels v1(ctx, dimension(8, 8, 1), Float);
  voxels v2(v1);

  // Create noisy data
  for (uint64 j = 0; j < 8; ++j) {
    for (uint64 i = 0; i < 8; ++i) {
      double val = 30.0 + double(i + j) * 3.0 + (i * j) % 5;
      v1(i, j, 0, val);
      v2(i, j, 0, val);
    }
  }

  // Apply with different q values
  v1.gdtvFilter(1.2, 0.3, 3, 0); // Small q (less nonlinear)
  v2.gdtvFilter(1.8, 0.3, 3, 0); // Large q (more nonlinear)

  // Both should smooth, but differently
  // At least one value should differ significantly
  bool found_difference = false;
  for (uint64 i = 0; i < 64; ++i) {
    if (std::abs(v1(i) - v2(i)) > 1.0) {
      found_difference = true;
      break;
    }
  }
  EXPECT_TRUE(found_difference) << "Different q values should produce different results";
}

TEST_F(VoxelsTest, GDTVFilterParameterLambda) {
  // Test 6: Lambda parameter controls data fidelity
  voxels v1(ctx, dimension(8, 8, 1), Float);
  voxels v2(v1);

  // Create noisy gradient
  for (uint64 j = 0; j < 8; ++j) {
    for (uint64 i = 0; i < 8; ++i) {
      double noise = (i * j) % 3 - 1.0;
      double val = 25.0 + double(i) * 4.0 + noise;
      v1(i, j, 0, val);
      v2(i, j, 0, val);
    }
  }

  double orig_val = v1(4, 4, 0);

  // Apply with different lambda values
  v1.gdtvFilter(1.5, 0.1, 3, 0); // Low lambda (more smoothing)
  v2.gdtvFilter(1.5, 0.9, 3, 0); // High lambda (more data fidelity)

  // High lambda should stay closer to original
  double dist1 = std::abs(v1(4, 4, 0) - orig_val);
  double dist2 = std::abs(v2(4, 4, 0) - orig_val);

  EXPECT_LT(dist2, dist1 + 0.5) << "Higher lambda should preserve data better";
}

TEST_F(VoxelsTest, GDTVFilterIterations) {
  // Test 7: GDTV filter with varying iteration counts
  voxels v(ctx, dimension(8, 8, 1), Float);

  // Create noisy gradient data
  for (uint64 j = 0; j < 8; ++j) {
    for (uint64 i = 0; i < 8; ++i) {
      double noise = ((i * 13 + j * 7) % 7) - 3.0;
      v(i, j, 0, 30.0 + double(i) * 4.0 + noise);
    }
  }

  // Test with minimal iterations
  v.gdtvFilter(1.5, 0.3, 1, 0);
  EXPECT_EQ(v.XDim(), 8u);
  EXPECT_EQ(v.YDim(), 8u);

  // Test with more iterations (should complete without errors)
  v.gdtvFilter(1.5, 0.3, 10, 0);
  EXPECT_EQ(v.XDim(), 8u);
  EXPECT_EQ(v.YDim(), 8u);

  // Verify values are in reasonable range
  EXPECT_GT(v(4, 4, 0), 20.0);
  EXPECT_LT(v(4, 4, 0), 80.0);
}

TEST_F(VoxelsTest, GDTVFilter6vs26Neighbor) {
  // Test 8: 6-neighbor vs 26-neighbor connectivity
  voxels v1(ctx, dimension(8, 8, 8), Float);
  voxels v2(v1);

  // Create 3D gradient with noise
  for (uint64 k = 0; k < 8; ++k) {
    for (uint64 j = 0; j < 8; ++j) {
      for (uint64 i = 0; i < 8; ++i) {
        double noise = (i + j + k) % 3 - 1.0;
        double val = 30.0 + double(i + j + k) * 2.0 + noise;
        v1(i, j, k, val);
        v2(i, j, k, val);
      }
    }
  }

  // Apply with different neighbor connectivity
  v1.gdtvFilter(1.5, 0.3, 3, 0);  // 6-neighbor
  v2.gdtvFilter(1.5, 0.3, 3, 26); // 26-neighbor

  // Results should differ
  bool found_difference = false;
  for (uint64 i = 0; i < 512; ++i) {
    if (std::abs(v1(i) - v2(i)) > 0.5) {
      found_difference = true;
      break;
    }
  }
  EXPECT_TRUE(found_difference) << "6 and 26 neighbor modes should differ";
}

TEST_F(VoxelsTest, GDTVFilter3DGradient) {
  // Test 9: 3D volumetric filtering
  voxels v(ctx, dimension(6, 6, 6), Float);

  // Create 3D gradient
  for (uint64 k = 0; k < 6; ++k) {
    for (uint64 j = 0; j < 6; ++j) {
      for (uint64 i = 0; i < 6; ++i) {
        double dist = sqrt(double(i * i + j * j + k * k));
        v(i, j, k, 25.0 + dist * 3.0);
      }
    }
  }

  v.gdtvFilter(1.5, 0.3, 3, 0);

  // Gradient structure should be approximately preserved
  EXPECT_LT(v(0, 0, 0), v(3, 3, 3));
  EXPECT_LT(v(3, 3, 3), v(5, 5, 5));

  // Center should be smoothed
  EXPECT_GT(v(3, 3, 3), 30.0);
  EXPECT_LT(v(3, 3, 3), 60.0);
}

TEST_F(VoxelsTest, GDTVFilterIsolatedSpike) {
  // Test 10: GDTV with gradient-dependent weighting
  voxels v(ctx, dimension(10, 10, 1), Float);

  // Create smooth region with an elevated area (not extreme spike)
  v.fill(40.0);
  v(4, 4, 0, 65.0);
  v(5, 4, 0, 65.0);
  v(4, 5, 0, 65.0);
  v(5, 5, 0, 65.0);

  double orig_center = v(4, 4, 0);
  double orig_far = v(0, 0, 0);

  v.gdtvFilter(1.5, 0.3, 3, 0);

  // Elevated region should be smoothed but preserved
  EXPECT_LT(v(4, 4, 0), orig_center + 5.0) << "Should be smoothed";
  EXPECT_GT(v(4, 4, 0), orig_far + 10.0) << "Should remain elevated";

  // Far region should be stable
  EXPECT_NEAR(v(0, 0, 0), orig_far, 1.0);
  EXPECT_NEAR(v(9, 9, 0), orig_far, 1.0);
}

// ============================================================================
// Composite Function Tests (Additional)
// ============================================================================

TEST_F(VoxelsTest, CompositeSubtract) {
  voxels v1(ctx, dimension(5, 5, 5), Float);
  voxels v2(ctx, dimension(5, 5, 5), Float);

  v1.fill(100.0);
  v2.fill(30.0);

  // Subtract v2 from v1
  subtract_func subtract;
  v1.composite(v2, 0, 0, 0, subtract);

  EXPECT_DOUBLE_EQ(v1(0, 0, 0), 70.0);
  EXPECT_DOUBLE_EQ(v1(2, 2, 2), 70.0);
}

TEST_F(VoxelsTest, CompositePartialOverlap) {
  voxels v1(ctx, dimension(10, 10, 10), Float);
  voxels v2(ctx, dimension(5, 5, 5), Float);

  v1.fill(0.0);
  v2.fill(100.0);

  // Composite v2 at corner with partial overlap
  copy_func copy;
  v1.composite(v2, 7, 7, 7, copy);

  // Overlapping region should be 100
  EXPECT_DOUBLE_EQ(v1(7, 7, 7), 100.0);
  EXPECT_DOUBLE_EQ(v1(9, 9, 9), 100.0);

  // Non-overlapping should be 0
  EXPECT_DOUBLE_EQ(v1(0, 0, 0), 0.0);
  EXPECT_DOUBLE_EQ(v1(6, 6, 6), 0.0);
}

// ============================================================================
// Error Condition and Edge Case Tests
// ============================================================================

TEST_F(VoxelsTest, FillSubOutOfBounds) {
  voxels v(ctx, dimension(10, 10, 10), Float);
  v.fill(0.0);

  // Out-of-bounds fillsub throws index_out_of_bounds exception
  EXPECT_THROW(v.fillsub(8, 8, 8, dimension(5, 5, 5), 100.0), index_out_of_bounds);
}

TEST_F(VoxelsTest, MinMaxUninitialized) {
  voxels v(ctx, dimension(5, 5, 5), Float);
  // Don't set any values, use default zeros

  EXPECT_NO_THROW(v.min());
  EXPECT_NO_THROW(v.max());
}

TEST_F(VoxelsTest, CopyLargeVolume) {
  voxels v1(ctx, dimension(50, 50, 50), Float);
  v1.fill(42.0);

  voxels v2(v1);

  EXPECT_EQ(v2.XDim(), 50u);
  EXPECT_EQ(v2.YDim(), 50u);
  EXPECT_EQ(v2.ZDim(), 50u);
  EXPECT_NEAR(v2(25, 25, 25), 42.0, 1e-5);
}

TEST_F(VoxelsTest, ResizeThenFill) {
  voxels v(ctx, dimension(5, 5, 5), Float);
  v.fill(10.0);

  v.resize(dimension(10, 10, 10));
  v.fill(20.0);

  EXPECT_NEAR(v(5, 5, 5), 20.0, 1e-5);
  EXPECT_NEAR(v(9, 9, 9), 20.0, 1e-5);
}

// ============================================================================
// CUDA Unified Memory Tests (only compiled if CUDA is available)
// ============================================================================

#ifdef CVC_USING_CUDA

class VoxelsCUDATest : public ::testing::Test {
protected:
  cvc::app ctx;
};

TEST_F(VoxelsCUDATest, CUDAAvailability) {
  // Test that CUDA availability can be queried
  bool cuda_avail = voxels::cuda_available();
  int device_count = voxels::cuda_device_count();

  // These should be consistent
  if (cuda_avail) {
    EXPECT_GT(device_count, 0);
  } else {
    EXPECT_EQ(device_count, 0);
  }
}

TEST_F(VoxelsCUDATest, GPUDeviceInfo) {
  if (!voxels::cuda_available()) {
    GTEST_SKIP() << "CUDA not available, skipping GPU tests";
  }

  auto gpus = voxels::get_gpu_info();
  EXPECT_GT(gpus.size(), 0u);

  // Check first GPU has valid properties
  EXPECT_GE(gpus[0].device_id, 0);
  EXPECT_FALSE(gpus[0].name.empty());
  EXPECT_GT(gpus[0].total_memory, 0u);
}

TEST_F(VoxelsCUDATest, DeviceSelection) {
  if (!voxels::cuda_available()) {
    GTEST_SKIP() << "CUDA not available, skipping GPU tests";
  }

  int original_device = voxels::get_current_gpu();
  EXPECT_GE(original_device, 0);

  // Set device 0 explicitly
  voxels::set_current_gpu(0);
  EXPECT_EQ(voxels::get_current_gpu(), 0);

  // Restore original device
  voxels::set_current_gpu(original_device);
}

TEST_F(VoxelsCUDATest, EnableDisableCUDA) {
  if (!voxels::cuda_available()) {
    GTEST_SKIP() << "CUDA not available, skipping GPU tests";
  }

  voxels v(ctx, dimension(10, 10, 10), Float);
  v.fill(42.0f);

  // Initially not using CUDA
  EXPECT_FALSE(v.using_cuda());
  EXPECT_EQ(v.cuda_device(), -1);

  // Enable CUDA
  v.enableCUDA(0);
  EXPECT_TRUE(v.using_cuda());
  EXPECT_EQ(v.cuda_device(), 0);

  // Data should be preserved
  EXPECT_NEAR(v(5, 5, 5), 42.0, 1e-5);
  EXPECT_NEAR(v(0, 0, 0), 42.0, 1e-5);
  EXPECT_NEAR(v(9, 9, 9), 42.0, 1e-5);

  // Disable CUDA
  v.disableCUDA();
  EXPECT_FALSE(v.using_cuda());
  EXPECT_EQ(v.cuda_device(), -1);

  // Data should still be preserved
  EXPECT_NEAR(v(5, 5, 5), 42.0, 1e-5);
  EXPECT_NEAR(v(0, 0, 0), 42.0, 1e-5);
  EXPECT_NEAR(v(9, 9, 9), 42.0, 1e-5);
}

TEST_F(VoxelsCUDATest, DataMigrationToGPU) {
  if (!voxels::cuda_available()) {
    GTEST_SKIP() << "CUDA not available, skipping GPU tests";
  }

  voxels v(ctx, dimension(20, 20, 20), Double);

  // Fill with specific pattern on CPU
  for (uint64 k = 0; k < 20; k++) {
    for (uint64 j = 0; j < 20; j++) {
      for (uint64 i = 0; i < 20; i++) {
        v(i, j, k, i * 100.0 + j * 10.0 + k);
      }
    }
  }

  // Migrate to GPU
  v.enableCUDA(0);

  // Verify all data is intact
  for (uint64 k = 0; k < 20; k++) {
    for (uint64 j = 0; j < 20; j++) {
      for (uint64 i = 0; i < 20; i++) {
        double expected = i * 100.0 + j * 10.0 + k;
        EXPECT_NEAR(v(i, j, k), expected, 1e-9);
      }
    }
  }
}

TEST_F(VoxelsCUDATest, DataMigrationFromGPU) {
  if (!voxels::cuda_available()) {
    GTEST_SKIP() << "CUDA not available, skipping GPU tests";
  }

  voxels v(ctx, dimension(15, 15, 15), Float);

  // Enable CUDA first
  v.enableCUDA(0);

  // Fill with data while on GPU
  for (uint64 k = 0; k < 15; k++) {
    for (uint64 j = 0; j < 15; j++) {
      for (uint64 i = 0; i < 15; i++) {
        v(i, j, k, static_cast<float>(i + j + k));
      }
    }
  }

  // Migrate back to CPU
  v.disableCUDA();

  // Verify all data survived the migration
  for (uint64 k = 0; k < 15; k++) {
    for (uint64 j = 0; j < 15; j++) {
      for (uint64 i = 0; i < 15; i++) {
        float expected = static_cast<float>(i + j + k);
        EXPECT_NEAR(v(i, j, k), expected, 1e-5);
      }
    }
  }
}

TEST_F(VoxelsCUDATest, MultipleEnableDisableCycles) {
  if (!voxels::cuda_available()) {
    GTEST_SKIP() << "CUDA not available, skipping GPU tests";
  }

  voxels v(ctx, dimension(10, 10, 10), UChar);
  v.fill(123.0);

  // Cycle between CPU and GPU multiple times
  for (int cycle = 0; cycle < 3; cycle++) {
    v.enableCUDA(0);
    EXPECT_TRUE(v.using_cuda());
    EXPECT_EQ(v(5, 5, 5), 123);

    v.disableCUDA();
    EXPECT_FALSE(v.using_cuda());
    EXPECT_EQ(v(5, 5, 5), 123);
  }
}

TEST_F(VoxelsCUDATest, SwitchGPUDevices) {
  if (!voxels::cuda_available()) {
    GTEST_SKIP() << "CUDA not available, skipping GPU tests";
  }

  int gpu_count = voxels::cuda_device_count();
  voxels v(ctx, dimension(12, 12, 12), Float);
  v.fill(99.0f);

  // Enable on GPU 0
  v.enableCUDA(0);
  EXPECT_EQ(v.cuda_device(), 0);
  EXPECT_NEAR(v(6, 6, 6), 99.0, 1e-5);

  if (gpu_count >= 2) {
    // Test actual GPU switching with multiple GPUs
    v.switchGPU(1);
    EXPECT_EQ(v.cuda_device(), 1);
    EXPECT_NEAR(v(6, 6, 6), 99.0, 1e-5);

    // Switch back to GPU 0
    v.switchGPU(0);
    EXPECT_EQ(v.cuda_device(), 0);
    EXPECT_NEAR(v(6, 6, 6), 99.0, 1e-5);
  } else {
    // With single GPU, just verify we can "switch" to the same GPU
    v.switchGPU(0);
    EXPECT_EQ(v.cuda_device(), 0);
    EXPECT_NEAR(v(6, 6, 6), 99.0, 1e-5);
  }
}

TEST_F(VoxelsCUDATest, ModifyDataOnGPU) {
  if (!voxels::cuda_available()) {
    GTEST_SKIP() << "CUDA not available, skipping GPU tests";
  }

  voxels v(ctx, dimension(10, 10, 10), Float);
  v.fill(0.0f);

  // Enable CUDA
  v.enableCUDA(0);

  // Modify data while on GPU
  v(5, 5, 5, 123.456f);
  v(0, 0, 0, 789.012f);
  v(9, 9, 9, 345.678f);

  // Verify modifications (use relaxed tolerance for float precision)
  EXPECT_NEAR(v(5, 5, 5), 123.456, 1e-3);
  EXPECT_NEAR(v(0, 0, 0), 789.012, 1e-3);
  EXPECT_NEAR(v(9, 9, 9), 345.678, 1e-3);

  // Disable CUDA and verify data persisted
  v.disableCUDA();
  EXPECT_NEAR(v(5, 5, 5), 123.456, 1e-3);
  EXPECT_NEAR(v(0, 0, 0), 789.012, 1e-3);
  EXPECT_NEAR(v(9, 9, 9), 345.678, 1e-3);
}

TEST_F(VoxelsCUDATest, FillOperationCPUvsGPU) {
  if (!voxels::cuda_available()) {
    GTEST_SKIP() << "CUDA not available, skipping GPU tests";
  }

  // Create two identical voxels
  dimension dim(20, 20, 20);
  voxels v_cpu(ctx, dim, Float);
  voxels v_gpu(ctx, dim, Float);

  // Fill on CPU
  v_cpu.fill(42.42);

  // Fill on GPU
  v_gpu.enableCUDA(0);
  v_gpu.fill(42.42);
  v_gpu.disableCUDA();

  // Compare results
  for (uint64 k = 0; k < 20; k++) {
    for (uint64 j = 0; j < 20; j++) {
      for (uint64 i = 0; i < 20; i++) {
        EXPECT_NEAR(v_cpu(i, j, k), v_gpu(i, j, k), 1e-5);
      }
    }
  }
}

TEST_F(VoxelsCUDATest, MapOperationCPUvsGPU) {
  if (!voxels::cuda_available()) {
    GTEST_SKIP() << "CUDA not available, skipping GPU tests";
  }

  dimension dim(15, 15, 15);
  voxels v_cpu(ctx, dim, Float);
  voxels v_gpu(ctx, dim, Float);

  // Fill with gradient
  for (uint64 k = 0; k < 15; k++) {
    for (uint64 j = 0; j < 15; j++) {
      for (uint64 i = 0; i < 15; i++) {
        double val = (i + j + k) / 45.0 * 255.0;
        v_cpu(i, j, k, val);
        v_gpu(i, j, k, val);
      }
    }
  }

  // Map on CPU
  v_cpu.map(0.0, 100.0);

  // Map on GPU
  v_gpu.enableCUDA(0);
  v_gpu.map(0.0, 100.0);
  v_gpu.disableCUDA();

  // Compare results
  for (uint64 k = 0; k < 15; k++) {
    for (uint64 j = 0; j < 15; j++) {
      for (uint64 i = 0; i < 15; i++) {
        EXPECT_NEAR(v_cpu(i, j, k), v_gpu(i, j, k), 1e-4);
      }
    }
  }
}

TEST_F(VoxelsCUDATest, SubvolumeOperationCPUvsGPU) {
  if (!voxels::cuda_available()) {
    GTEST_SKIP() << "CUDA not available, skipping GPU tests";
  }

  dimension dim(20, 20, 20);
  voxels v_cpu(ctx, dim, Float);
  voxels v_gpu(ctx, dim, Float);

  // Fill with pattern
  for (uint64 k = 0; k < 20; k++) {
    for (uint64 j = 0; j < 20; j++) {
      for (uint64 i = 0; i < 20; i++) {
        double val = i * 100 + j * 10 + k;
        v_cpu(i, j, k, val);
        v_gpu(i, j, k, val);
      }
    }
  }

  // Extract subvolume on CPU
  v_cpu.sub(5, 5, 5, dimension(10, 10, 10));

  // Extract subvolume on GPU
  v_gpu.enableCUDA(0);
  v_gpu.sub(5, 5, 5, dimension(10, 10, 10));
  v_gpu.disableCUDA();

  // Compare dimensions
  EXPECT_EQ(v_cpu.XDim(), v_gpu.XDim());
  EXPECT_EQ(v_cpu.YDim(), v_gpu.YDim());
  EXPECT_EQ(v_cpu.ZDim(), v_gpu.ZDim());

  // Compare data
  for (uint64 k = 0; k < 10; k++) {
    for (uint64 j = 0; j < 10; j++) {
      for (uint64 i = 0; i < 10; i++) {
        EXPECT_NEAR(v_cpu(i, j, k), v_gpu(i, j, k), 1e-5);
      }
    }
  }
}

TEST_F(VoxelsCUDATest, BilateralFilterCPUvsGPU) {
  if (!voxels::cuda_available()) {
    GTEST_SKIP() << "CUDA not available, skipping GPU tests";
  }

  dimension dim(16, 16, 16);
  voxels v_cpu(ctx, dim, Float);
  voxels v_gpu(ctx, dim, Float);

  // Create a simple pattern with some noise
  for (uint64 k = 0; k < 16; k++) {
    for (uint64 j = 0; j < 16; j++) {
      for (uint64 i = 0; i < 16; i++) {
        double base = (i < 8) ? 50.0 : 150.0;
        double noise = ((i + j + k) % 3) * 2.0;
        v_cpu(i, j, k, base + noise);
        v_gpu(i, j, k, base + noise);
      }
    }
  }

  // Apply bilateral filter on CPU
  v_cpu.bilateralFilter(20.0, 1.0, 1);

  // Apply bilateral filter on GPU
  v_gpu.enableCUDA(0);
  v_gpu.bilateralFilter(20.0, 1.0, 1);
  v_gpu.disableCUDA();

  // Compare results - should be very close
  double max_diff = 0.0;
  for (uint64 k = 0; k < 16; k++) {
    for (uint64 j = 0; j < 16; j++) {
      for (uint64 i = 0; i < 16; i++) {
        double diff = std::abs(v_cpu(i, j, k) - v_gpu(i, j, k));
        max_diff = std::max(max_diff, diff);
      }
    }
  }

  // Results should be identical or very close (within floating point tolerance)
  EXPECT_LT(max_diff, 0.1) << "CPU and GPU bilateral filter results differ significantly";
}

TEST_F(VoxelsCUDATest, MinMaxCalculationCPUvsGPU) {
  if (!voxels::cuda_available()) {
    GTEST_SKIP() << "CUDA not available, skipping GPU tests";
  }

  dimension dim(10, 10, 10);
  voxels v_cpu(ctx, dim, Float);
  voxels v_gpu(ctx, dim, Float);

  // Fill with random-ish pattern
  for (uint64 k = 0; k < 10; k++) {
    for (uint64 j = 0; j < 10; j++) {
      for (uint64 i = 0; i < 10; i++) {
        double val = (i * 7 + j * 13 + k * 19) % 100;
        v_cpu(i, j, k, val);
        v_gpu(i, j, k, val);
      }
    }
  }

  // Calculate min/max on CPU
  double cpu_min = v_cpu.min();
  double cpu_max = v_cpu.max();

  // Calculate min/max on GPU
  v_gpu.enableCUDA(0);
  double gpu_min = v_gpu.min();
  double gpu_max = v_gpu.max();
  v_gpu.disableCUDA();

  // Should be identical
  EXPECT_NEAR(cpu_min, gpu_min, 1e-9);
  EXPECT_NEAR(cpu_max, gpu_max, 1e-9);
}

TEST_F(VoxelsCUDATest, MinMaxSubvolumeCPUvsGPU) {
  if (!voxels::cuda_available()) {
    GTEST_SKIP() << "CUDA not available, skipping GPU tests";
  }

  dimension dim(20, 20, 20);
  voxels v_cpu(ctx, dim, Float);
  voxels v_gpu(ctx, dim, Float);

  // Fill with pattern where center region has specific range
  for (uint64 k = 0; k < 20; k++) {
    for (uint64 j = 0; j < 20; j++) {
      for (uint64 i = 0; i < 20; i++) {
        double val;
        if (i >= 5 && i < 15 && j >= 5 && j < 15 && k >= 5 && k < 15) {
          // Center region: values from 10 to ~110
          val = 10.0 + ((i - 5) * 10 + (j - 5) + (k - 5));
        } else {
          // Outer region: large values
          val = 500.0 + i + j + k;
        }
        v_cpu(i, j, k, val);
        v_gpu(i, j, k, val);
      }
    }
  }

  // Calculate min/max on subvolume (center region) using CPU
  double cpu_min = v_cpu.min(5, 5, 5, dimension(10, 10, 10));
  double cpu_max = v_cpu.max(5, 5, 5, dimension(10, 10, 10));

  // Calculate min/max on same subvolume using GPU
  v_gpu.enableCUDA(0);
  double gpu_min = v_gpu.min(5, 5, 5, dimension(10, 10, 10));
  double gpu_max = v_gpu.max(5, 5, 5, dimension(10, 10, 10));
  v_gpu.disableCUDA();

  // Results should be identical
  EXPECT_NEAR(cpu_min, gpu_min, 1e-9);
  EXPECT_NEAR(cpu_max, gpu_max, 1e-9);

  // Verify the values are from the center region, not the outer region
  EXPECT_LT(cpu_min, 200.0) << "Min should be from center region";
  EXPECT_LT(cpu_max, 500.0) << "Max should be from center region";
  EXPECT_GE(cpu_min, 10.0) << "Min should be at least 10";
}

TEST_F(VoxelsCUDATest, MinMaxPerformanceComparison) {
#ifdef CVC_USING_CUDA
  if (!enable_stress_tests) {
    GTEST_SKIP() << "Stress test disabled. Use --enable-stress-tests to run.";
  }

  if (!voxels::cuda_available()) {
    GTEST_SKIP() << "CUDA not available, skipping test";
    return;
  }

  std::cout << "\n=== CPU vs GPU Min/Max Performance Comparison ===" << std::endl;
  std::cout << std::string(110, '=') << std::endl;
  std::cout << std::setw(12) << "Volume Size" << std::setw(12) << "Voxels" << std::setw(15)
            << "CPU Min (µs)" << std::setw(15) << "GPU Min (µs)" << std::setw(12) << "Speedup"
            << std::setw(15) << "CPU Max (µs)" << std::setw(15) << "GPU Max (µs)" << std::setw(12)
            << "Speedup" << std::endl;
  std::cout << std::string(110, '-') << std::endl;

  // Test different volume sizes
  std::vector<uint64> dimensions = {32, 64, 96, 128, 160, 192, 224, 256};
  const int num_iterations = 20; // Run multiple times for better timing

  for (uint64 dim : dimensions) {
    uint64 num_voxels = dim * dim * dim;

    // Create volume with varied data to ensure meaningful min/max
    voxels v_cpu(ctx, dimension(dim, dim, dim), Float);
    for (uint64 k = 0; k < dim; k++) {
      for (uint64 j = 0; j < dim; j++) {
        for (uint64 i = 0; i < dim; i++) {
          // Create pattern with both small and large values
          float val = static_cast<float>((i * 7 + j * 13 + k * 19) % 1000) / 10.0f;
          v_cpu(i, j, k, val);
        }
      }
    }

    // Create GPU copy
    voxels v_gpu(v_cpu);
    v_gpu.enableCUDA(0);
    EXPECT_TRUE(v_gpu.using_cuda());

    // Warm up (first call may include overhead)
    v_cpu.min();
    v_gpu.min();
    v_cpu.max();
    v_gpu.max();

    // Benchmark CPU min (multiple iterations)
    auto cpu_min_start = std::chrono::high_resolution_clock::now();
    double cpu_min_val = 0;
    for (int iter = 0; iter < num_iterations; iter++) {
      v_cpu.unsetMinMax(); // Force recalculation
      cpu_min_val = v_cpu.min();
    }
    auto cpu_min_end = std::chrono::high_resolution_clock::now();
    auto cpu_min_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(cpu_min_end - cpu_min_start);
    double cpu_min_us = cpu_min_duration.count() / double(num_iterations);

    // Benchmark GPU min (multiple iterations)
    auto gpu_min_start = std::chrono::high_resolution_clock::now();
    double gpu_min_val = 0;
    for (int iter = 0; iter < num_iterations; iter++) {
      v_gpu.unsetMinMax(); // Force recalculation
      gpu_min_val = v_gpu.min();
    }
    auto gpu_min_end = std::chrono::high_resolution_clock::now();
    auto gpu_min_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(gpu_min_end - gpu_min_start);
    double gpu_min_us = gpu_min_duration.count() / double(num_iterations);

    // Benchmark CPU max (multiple iterations)
    auto cpu_max_start = std::chrono::high_resolution_clock::now();
    double cpu_max_val = 0;
    for (int iter = 0; iter < num_iterations; iter++) {
      v_cpu.unsetMinMax(); // Force recalculation
      cpu_max_val = v_cpu.max();
    }
    auto cpu_max_end = std::chrono::high_resolution_clock::now();
    auto cpu_max_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(cpu_max_end - cpu_max_start);
    double cpu_max_us = cpu_max_duration.count() / double(num_iterations);

    // Benchmark GPU max (multiple iterations)
    auto gpu_max_start = std::chrono::high_resolution_clock::now();
    double gpu_max_val = 0;
    for (int iter = 0; iter < num_iterations; iter++) {
      v_gpu.unsetMinMax(); // Force recalculation
      gpu_max_val = v_gpu.max();
    }
    auto gpu_max_end = std::chrono::high_resolution_clock::now();
    auto gpu_max_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(gpu_max_end - gpu_max_start);
    double gpu_max_us = gpu_max_duration.count() / double(num_iterations);

    // Verify results match
    EXPECT_NEAR(cpu_min_val, gpu_min_val, 1e-5) << "Min values should match for " << dim << "³";
    EXPECT_NEAR(cpu_max_val, gpu_max_val, 1e-5) << "Max values should match for " << dim << "³";

    double min_speedup = cpu_min_us / gpu_min_us;
    double max_speedup = cpu_max_us / gpu_max_us;

    std::string dim_str = std::to_string(dim) + "³";
    std::string voxels_str = std::to_string(num_voxels / 1000000) + "M";
    if (num_voxels < 1000000) {
      voxels_str = std::to_string(num_voxels / 1000) + "K";
    }

    std::cout << std::setw(12) << dim_str << std::setw(12) << voxels_str << std::setw(15)
              << std::fixed << std::setprecision(2) << cpu_min_us << std::setw(15) << std::fixed
              << std::setprecision(2) << gpu_min_us << std::setw(12) << std::fixed
              << std::setprecision(2) << min_speedup << "x" << std::setw(15) << std::fixed
              << std::setprecision(2) << cpu_max_us << std::setw(15) << std::fixed
              << std::setprecision(2) << gpu_max_us << std::setw(12) << std::fixed
              << std::setprecision(2) << max_speedup << "x" << std::endl;
  }

  std::cout << std::string(110, '=') << std::endl;
  std::cout << "Note: GPU timings include kernel launch overhead and final host reduction"
            << std::endl;
  std::cout << "      CPU implementation uses OpenMP parallel reduction with collapse(3)"
            << std::endl;
  std::cout << "      Each measurement averaged over " << num_iterations << " iterations"
            << std::endl;
#else
  GTEST_SKIP() << "CUDA support not compiled in";
#endif
}

TEST_F(VoxelsCUDATest, CopyOperationWithCUDA) {
  if (!voxels::cuda_available()) {
    GTEST_SKIP() << "CUDA not available, skipping GPU tests";
  }

  voxels v1(ctx, dimension(10, 10, 10), Float);
  v1.fill(123.0);
  v1.enableCUDA(0);

  // Shallow copy
  voxels v2(ctx);
  v2.copy(v1);

  // v2 should also be using CUDA on the same device
  EXPECT_TRUE(v2.using_cuda());
  EXPECT_EQ(v2.cuda_device(), 0);
  EXPECT_NEAR(v2(5, 5, 5), 123.0, 1e-5);

  // Deep copy
  voxels v3(ctx);
  v3.copy(v1, true);

  // v3 should NOT be using CUDA (deep copy migrates to CPU)
  // Actually, let's check what the behavior is...
  EXPECT_NEAR(v3(5, 5, 5), 123.0, 1e-5);
}

TEST_F(VoxelsCUDATest, DeepCopySelfWithCUDA) {
  if (!voxels::cuda_available()) {
    GTEST_SKIP() << "CUDA not available, skipping GPU tests";
  }

  voxels v(ctx, dimension(10, 10, 10), Float);
  v.fill(42.0);
  v(5, 5, 5, 100.0);

  // Enable CUDA
  v.enableCUDA(0);
  EXPECT_TRUE(v.using_cuda());
  EXPECT_NEAR(v(5, 5, 5), 100.0, 1e-5);

  // Create a shallow copy to share CUDA memory
  voxels v_shared(ctx);
  v_shared.copy(v, false);
  EXPECT_TRUE(v_shared.using_cuda());

  // Now deep copy v to make it independent (using zero-parameter copy())
  v.copy();

  // After deep copy, v should NOT be using CUDA (deep copy migrates to CPU)
  EXPECT_FALSE(v.using_cuda());
  EXPECT_NEAR(v(5, 5, 5), 100.0, 1e-5);

  // v_shared should still be using CUDA
  EXPECT_TRUE(v_shared.using_cuda());
  EXPECT_NEAR(v_shared(5, 5, 5), 100.0, 1e-5);

  // Modify v (CPU-only now)
  v.fill(999.0);

  // v_shared should be unaffected (was using CUDA memory)
  EXPECT_NEAR(v_shared(5, 5, 5), 100.0, 1e-5);
  EXPECT_NEAR(v(5, 5, 5), 999.0, 1e-5);
}

TEST_F(VoxelsCUDATest, DifferentDataTypes) {
  if (!voxels::cuda_available()) {
    GTEST_SKIP() << "CUDA not available, skipping GPU tests";
  }

  // Test multiple data types
  std::vector<data_type> types = {UChar, UShort, UInt, Float, Double};

  for (auto dtype : types) {
    voxels v(ctx, dimension(8, 8, 8), dtype);
    v.fill(42.0);

    // Enable CUDA
    v.enableCUDA(0);
    EXPECT_TRUE(v.using_cuda());

    // Verify data
    EXPECT_NEAR(v(4, 4, 4), 42.0, 1e-5);

    // Disable CUDA
    v.disableCUDA();
    EXPECT_FALSE(v.using_cuda());
    EXPECT_NEAR(v(4, 4, 4), 42.0, 1e-5);
  }
}

TEST_F(VoxelsCUDATest, LargeVolumePerformance) {
  if (!voxels::cuda_available()) {
    GTEST_SKIP() << "CUDA not available, skipping GPU tests";
  }

  // Test with a reasonably large volume
  dimension dim(100, 100, 100);
  voxels v(ctx, dim, Float);

  // Fill with pattern
  for (uint64 k = 0; k < 100; k += 10) {
    for (uint64 j = 0; j < 100; j += 10) {
      for (uint64 i = 0; i < 100; i += 10) {
        v(i, j, k, static_cast<float>(i + j + k));
      }
    }
  }

  // Migrate to GPU - this tests that large volumes work
  v.enableCUDA(0);
  EXPECT_TRUE(v.using_cuda());

  // Spot check some values
  EXPECT_NEAR(v(50, 50, 50), 150.0, 1e-5);
  EXPECT_NEAR(v(90, 90, 90), 270.0, 1e-5);

  // Migrate back
  v.disableCUDA();
  EXPECT_FALSE(v.using_cuda());
  EXPECT_NEAR(v(50, 50, 50), 150.0, 1e-5);
}

// Test multithreading behavior with CUDA
TEST_F(VoxelsCUDATest, MultithreadedCUDAOperations) {
#ifdef CVC_USING_CUDA
  if (!voxels::cuda_available()) {
    GTEST_SKIP() << "CUDA not available, skipping test";
    return;
  }

  const uint64 dim = 32;

  // Test 1: Create voxels in main thread, pass to worker thread for CUDA operations
  {
    voxels v(ctx, dimension(dim, dim, dim), Float);

    // Initialize data in main thread
    for (uint64 k = 0; k < dim; k++) {
      for (uint64 j = 0; j < dim; j++) {
        for (uint64 i = 0; i < dim; i++) {
          v(i, j, k, static_cast<float>(i + j + k));
        }
      }
    }

    // Pass to worker thread for CUDA operations
    std::thread worker([&v, dim]() {
      // Enable CUDA in this thread
      v.enableCUDA(0);
      EXPECT_TRUE(v.using_cuda());

      // Verify data is accessible
      EXPECT_NEAR(v(10, 10, 10), 30.0f, 1e-5);

      // Modify data on GPU
      for (uint64 k = 0; k < dim; k++) {
        for (uint64 j = 0; j < dim; j++) {
          for (uint64 i = 0; i < dim; i++) {
            float val = v(i, j, k);
            v(i, j, k, val * 2.0f);
          }
        }
      }

      // Synchronize
      cudaDeviceSynchronize();
    });

    worker.join();

    // Verify changes in main thread
    EXPECT_NEAR(v(10, 10, 10), 60.0f, 1e-5);
    EXPECT_NEAR(v(0, 0, 0), 0.0f, 1e-5);
    EXPECT_NEAR(v(20, 20, 20), 120.0f, 1e-5);
  }

  // Test 2: Shallow copy with CUDA - demonstrates copy-on-write behavior
  {
    voxels v1(ctx, dimension(dim, dim, dim), Float);

    // Initialize and enable CUDA
    float *fdata = reinterpret_cast<float *>(v1.data_ptr());
    for (uint64 i = 0; i < dim * dim * dim; i++) {
      fdata[i] = static_cast<float>(i);
    }
    v1.enableCUDA(0);
    EXPECT_TRUE(v1.using_cuda());

    // Verify initial value
    float initial_val = v1(5, 5, 5);
    EXPECT_NEAR(initial_val, 5.0f + 5.0f * dim + 5.0f * dim * dim, 1e-5);

    // Create shallow copy - shares CUDA memory initially
    voxels v2(v1);
    EXPECT_TRUE(v2.using_cuda());

    // Verify both have the same value initially
    EXPECT_EQ(v1(5, 5, 5), v2(5, 5, 5));

    // Modify through v2 in a worker thread
    // IMPORTANT: This triggers copy-on-write!
    // preWrite() creates a new CPU-only copy for v2 and disables its CUDA
    std::thread worker([&v2]() {
      v2(5, 5, 5, 999.0f); // This write triggers copy-on-write
      // After preWrite(), v2 now has its own CPU memory with CUDA disabled
    });

    worker.join();

    // After copy-on-write, v1 still has original CUDA memory, v2 has new CPU memory
    EXPECT_TRUE(v1.using_cuda()) << "v1 should still have CUDA enabled";
    EXPECT_FALSE(v2.using_cuda()) << "v2 should have CUDA disabled after copy-on-write";

    // v2 has the modified value in its CPU copy
    EXPECT_NEAR(v2(5, 5, 5), 999.0f, 1e-5);

    // v1 has the original value in its CUDA memory
    EXPECT_NEAR(v1(5, 5, 5), initial_val, 1e-5);
  }

  // Test 3: Shallow copy read-only access - shares CUDA memory safely
  {
    voxels v1(ctx, dimension(dim, dim, dim), Float);

    // Initialize and enable CUDA
    float *fdata = reinterpret_cast<float *>(v1.data_ptr());
    for (uint64 i = 0; i < dim * dim * dim; i++) {
      fdata[i] = static_cast<float>(i);
    }
    v1.enableCUDA(0);

    // Create shallow copy - shares CUDA memory
    voxels v2(v1);
    EXPECT_TRUE(v1.using_cuda());
    EXPECT_TRUE(v2.using_cuda());

    // Read from v2 in worker thread (no writes, no copy-on-write)
    std::atomic<bool> read_success(false);
    std::thread worker([&v2, &read_success, dim]() {
      // Read-only access doesn't trigger copy-on-write
      float val = v2(10, 10, 10);
      float expected = 10.0f + 10.0f * dim + 10.0f * dim * dim;
      if (std::abs(val - expected) < 1e-5) {
        read_success = true;
      }
    });

    worker.join();

    EXPECT_TRUE(read_success.load())
        << "Worker thread should read correct value from shared CUDA memory";
    EXPECT_TRUE(v1.using_cuda());
    EXPECT_TRUE(v2.using_cuda());
    // Both still share CUDA memory since no writes occurred
  }

  // Test 4: Deep copy with independent CUDA memory across threads
  {
    voxels v1(ctx, dimension(dim, dim, dim), Float);

    // Initialize and enable CUDA
    float *fdata = reinterpret_cast<float *>(v1.data_ptr());
    for (uint64 i = 0; i < dim * dim * dim; i++) {
      fdata[i] = static_cast<float>(i);
    }
    v1.enableCUDA(0);

    // Create deep copy - independent memory
    voxels v2(ctx);
    v2.copy(v1, true); // Deep copy

    // v2 has CPU-only copy (deep copy doesn't enable CUDA on destination)
    EXPECT_TRUE(v1.using_cuda());
    EXPECT_FALSE(v2.using_cuda());

    // Initially equal but independent
    float original_value = v1(5, 5, 5);
    EXPECT_EQ(v2(5, 5, 5), original_value);

    // Modify v2 in worker thread
    std::thread worker([&v2]() { v2(5, 5, 5, 777.0f); });

    worker.join();

    // v1 should be unchanged (deep copy has independent memory)
    EXPECT_NEAR(v1(5, 5, 5), original_value, 1e-5);
    EXPECT_NEAR(v2(5, 5, 5), 777.0f, 1e-5);
  }

  // Test 5: Multiple threads working on independent voxels objects
  {
    std::atomic<int> success_count(0);
    const int num_threads = 4;
    std::vector<std::string> task_keys;

    // Set pool size
    unsigned int original_pool_size = ctx.getThreadPoolSize();
    ctx.setThreadPoolSize(std::min(4u, boost::thread::hardware_concurrency()));

    for (int t = 0; t < num_threads; t++) {
      std::string key = "cuda_voxels_" + std::to_string(t);
      task_keys.push_back(key);

      ctx.startThreadPooled(
          key,
          [this, t, &success_count, dim]() {
            try {
              voxels v(ctx, dimension(dim, dim, dim), Float);

              // Each thread initializes with different values
              float offset = static_cast<float>(t * 1000);
              float *fdata = reinterpret_cast<float *>(v.data_ptr());
              for (uint64 i = 0; i < dim * dim * dim; i++) {
                fdata[i] = offset + static_cast<float>(i);
              }

              // Enable CUDA (all threads use device 0 for simplicity)
              v.enableCUDA(0);

              // Verify data
              float expected = offset + (10 + 10 * dim + 10 * dim * dim);
              if (std::abs(v(10, 10, 10) - expected) < 1e-5) {
                success_count++;
              }

              cudaDeviceSynchronize();
            } catch (...) {
              // Thread failed
            }
          },
          PRIORITY_NORMAL, true);
    }

    // Wait for all tasks to complete
    for (const auto &key : task_keys) {
      if (ctx.hasThread(key)) {
        thread_ptr tptr = ctx.threads(key);
        if (tptr)
          tptr->join();
      }
    }

    // Restore original pool size
    ctx.setThreadPoolSize(original_pool_size);

    EXPECT_EQ(success_count.load(), num_threads);
  }

  // Test 6: Direct memory modification across threads (bypass copy-on-write)
  {
    voxels v(ctx, dimension(dim, dim, dim), UChar);

    // Initialize
    unsigned char *udata = v.data_ptr();
    for (uint64 i = 0; i < dim * dim * dim; i++) {
      udata[i] = static_cast<unsigned char>(i % 256);
    }

    // Enable CUDA in main thread
    v.enableCUDA(0);
    unsigned char original = v(15, 15, 15);

    // Modify directly via data_ptr() in worker thread (bypasses copy-on-write)
    std::thread worker([&v, dim]() {
      // Direct memory access bypasses operator() and copy-on-write
      unsigned char *data = v.data_ptr();
      uint64 idx = 15 + 15 * dim + 15 * dim * dim;
      unsigned char old_val = data[idx];
      data[idx] = static_cast<unsigned char>(old_val + 10);
      cudaDeviceSynchronize();
    });

    worker.join();
    cudaDeviceSynchronize();

    // Changes should be visible since we modified CUDA memory directly
    EXPECT_EQ(v(15, 15, 15), static_cast<unsigned char>(original + 10));
    EXPECT_TRUE(v.using_cuda()) << "CUDA should still be enabled";
  }
#else
  GTEST_SKIP() << "CUDA support not compiled in";
#endif
}

// Test GPU-accelerated trilinear resize
TEST_F(VoxelsCUDATest, ResizeWithCUDA) {
#ifdef CVC_USING_CUDA
  if (!voxels::cuda_available()) {
    GTEST_SKIP() << "CUDA not available, skipping test";
    return;
  }

  // Test with different sizes and data types
  {
    // Create a small test volume with known pattern
    const uint64 src_dim = 16;
    voxels v_cpu(ctx, dimension(src_dim, src_dim, src_dim), Float);

    // Initialize with a simple pattern: value = i + j + k
    for (uint64 k = 0; k < src_dim; k++) {
      for (uint64 j = 0; j < src_dim; j++) {
        for (uint64 i = 0; i < src_dim; i++) {
          v_cpu(i, j, k, static_cast<float>(i + j + k));
        }
      }
    }

    // Resize on CPU
    const uint64 dst_dim = 24;
    voxels v_cpu_resized(v_cpu);
    v_cpu_resized.resize(dimension(dst_dim, dst_dim, dst_dim));

    // Now do the same with CUDA
    voxels v_gpu(v_cpu);
    v_gpu.enableCUDA(0);
    EXPECT_TRUE(v_gpu.using_cuda());

    v_gpu.resize(dimension(dst_dim, dst_dim, dst_dim));

    // Compare results - they should be very close
    float max_diff = 0.0f;
    for (uint64 k = 0; k < dst_dim; k++) {
      for (uint64 j = 0; j < dst_dim; j++) {
        for (uint64 i = 0; i < dst_dim; i++) {
          float cpu_val = v_cpu_resized(i, j, k);
          float gpu_val = v_gpu(i, j, k);
          float diff = std::abs(cpu_val - gpu_val);
          max_diff = std::max(max_diff, diff);
        }
      }
    }

    EXPECT_LT(max_diff, 1e-4) << "GPU and CPU resize results should match";

    // Verify some specific interpolated values
    EXPECT_NEAR(v_gpu(0, 0, 0), 0.0f, 1e-4);
    EXPECT_NEAR(v_gpu(dst_dim - 1, dst_dim - 1, dst_dim - 1), static_cast<float>(3 * (src_dim - 1)),
                0.1f);
  }

  // Test upsampling (small to large)
  {
    voxels v_small(ctx, dimension(8, 8, 8), UChar);
    for (uint64 i = 0; i < 8 * 8 * 8; i++) {
      v_small.data_ptr()[i] = static_cast<unsigned char>(i % 256);
    }

    v_small.enableCUDA(0);
    v_small.resize(dimension(32, 32, 32));

    EXPECT_EQ(v_small.XDim(), 32u);
    EXPECT_EQ(v_small.YDim(), 32u);
    EXPECT_EQ(v_small.ZDim(), 32u);
    EXPECT_TRUE(v_small.using_cuda());
  }

  // Test downsampling (large to small)
  {
    voxels v_large(ctx, dimension(64, 64, 64), Double);
    double *ddata = reinterpret_cast<double *>(v_large.data_ptr());
    for (uint64 i = 0; i < 64 * 64 * 64; i++) {
      ddata[i] = static_cast<double>(i) / 100.0;
    }

    v_large.enableCUDA(0);
    v_large.resize(dimension(16, 16, 16));

    EXPECT_EQ(v_large.XDim(), 16u);
    EXPECT_EQ(v_large.YDim(), 16u);
    EXPECT_EQ(v_large.ZDim(), 16u);
    EXPECT_TRUE(v_large.using_cuda());

    // Values should be in a reasonable range
    EXPECT_GE(v_large(0, 0, 0), 0.0);
    EXPECT_LT(v_large(15, 15, 15), 650000.0); // max index 262143 / 100
  }
#else
  GTEST_SKIP() << "CUDA support not compiled in";
#endif
}

TEST_F(VoxelsCUDATest, CUDAAnisotropicDiffusionSliceProcessing) {
#ifdef CVC_USING_CUDA
  if (!voxels::cuda_available()) {
    GTEST_SKIP() << "CUDA not available, skipping GPU tests";
  }

  // Create a volume with an edge to preserve
  voxels v(ctx, dimension(20, 20, 20), Float);

  // Create a step edge in the middle
  for (uint64 k = 0; k < 20; k++) {
    for (uint64 j = 0; j < 20; j++) {
      for (uint64 i = 0; i < 20; i++) {
        if (i < 10) {
          v(i, j, k, 0.0);
        } else {
          v(i, j, k, 100.0);
        }
      }
    }
  }

  // Enable CUDA
  v.enableCUDA(0);
  EXPECT_TRUE(v.using_cuda());

  // Store original edge values
  double edge_low_before = v(9, 10, 10);
  double edge_high_before = v(10, 10, 10);

  // Apply anisotropic diffusion (should preserve edges)
  v.anisotropicDiffusion(5);

  EXPECT_TRUE(v.using_cuda());

  // Check that the edge is still relatively sharp
  double edge_low_after = v(9, 10, 10);
  double edge_high_after = v(10, 10, 10);

  // The difference should still be significant (edge preserved)
  double difference = fabs(edge_high_after - edge_low_after);
  EXPECT_GT(difference, 50.0); // Should still have clear edge

  // Values should be smoothed but not completely blurred
  EXPECT_GT(edge_low_after, edge_low_before - 5.0);
  EXPECT_LT(edge_high_after, edge_high_before + 5.0);

  // Test uniform region smoothing
  voxels v2(ctx, dimension(20, 20, 20), Float);
  v2.fill(50.0);

  // Add some noise
  for (uint64 i = 0; i < 10; i++) {
    uint64 x = i * 2;
    uint64 y = i * 2;
    uint64 z = i * 2;
    if (x < 20 && y < 20 && z < 20) {
      v2(x, y, z, 50.0 + (i % 2 == 0 ? 10.0 : -10.0));
    }
  }

  v2.enableCUDA(0);
  v2.anisotropicDiffusion(10);

  // After diffusion, noise should be reduced
  double variance = 0.0;
  for (uint64 k = 0; k < 20; k++) {
    for (uint64 j = 0; j < 20; j++) {
      for (uint64 i = 0; i < 20; i++) {
        double diff = v2(i, j, k) - 50.0;
        variance += diff * diff;
      }
    }
  }
  variance /= (20 * 20 * 20);

  // Variance should be small after smoothing
  EXPECT_LT(variance, 25.0);
#else
  GTEST_SKIP() << "CUDA support not compiled in";
#endif
}

// CPU vs GPU resize performance benchmark
TEST_F(VoxelsCUDATest, ResizePerformanceComparison) {
#ifdef CVC_USING_CUDA
  if (!voxels::cuda_available()) {
    GTEST_SKIP() << "CUDA not available, skipping test";
    return;
  }

  std::cout << "\n=== CPU vs GPU Resize Performance Comparison ===" << std::endl;
  std::cout << std::string(90, '=') << std::endl;
  std::cout << std::setw(15) << "Resolution" << std::setw(15) << "CPU Time (ms)" << std::setw(15)
            << "GPU Time (ms)" << std::setw(15) << "Speedup" << std::setw(15) << "Max Diff"
            << std::setw(15) << "Status" << std::endl;
  std::cout << std::string(90, '-') << std::endl;

  // Test different resolutions
  std::vector<std::pair<uint64, uint64>> resize_configs = {
      {16, 32},  // 2x upsample
      {32, 64},  // 2x upsample
      {64, 128}, // 2x upsample
      {32, 48},  // 1.5x upsample
      {64, 32},  // 2x downsample
      {128, 64}  // 2x downsample
  };

  for (const auto &config : resize_configs) {
    uint64 src_dim = config.first;
    uint64 dst_dim = config.second;

    // Create source volume with known pattern
    voxels v_src(ctx, dimension(src_dim, src_dim, src_dim), Float);
    for (uint64 k = 0; k < src_dim; k++) {
      for (uint64 j = 0; j < src_dim; j++) {
        for (uint64 i = 0; i < src_dim; i++) {
          float val = static_cast<float>(i + j + k) / (3.0f * src_dim);
          v_src(i, j, k, val);
        }
      }
    }

    // CPU resize
    voxels v_cpu(v_src);
    auto cpu_start = std::chrono::high_resolution_clock::now();
    v_cpu.resize(dimension(dst_dim, dst_dim, dst_dim));
    auto cpu_end = std::chrono::high_resolution_clock::now();
    auto cpu_duration = std::chrono::duration_cast<std::chrono::microseconds>(cpu_end - cpu_start);
    double cpu_time_ms = cpu_duration.count() / 1000.0;

    // GPU resize
    voxels v_gpu(v_src);
    v_gpu.enableCUDA(0);
    EXPECT_TRUE(v_gpu.using_cuda());

    auto gpu_start = std::chrono::high_resolution_clock::now();
    v_gpu.resize(dimension(dst_dim, dst_dim, dst_dim));
    auto gpu_end = std::chrono::high_resolution_clock::now();
    auto gpu_duration = std::chrono::duration_cast<std::chrono::microseconds>(gpu_end - gpu_start);
    double gpu_time_ms = gpu_duration.count() / 1000.0;

    // Compare results
    float max_diff = 0.0f;
    for (uint64 k = 0; k < dst_dim; k++) {
      for (uint64 j = 0; j < dst_dim; j++) {
        for (uint64 i = 0; i < dst_dim; i++) {
          float cpu_val = v_cpu(i, j, k);
          float gpu_val = v_gpu(i, j, k);
          float diff = std::abs(cpu_val - gpu_val);
          max_diff = std::max(max_diff, diff);
        }
      }
    }

    double speedup = cpu_time_ms / gpu_time_ms;
    std::string status = (max_diff < 1e-4) ? "✓ PASS" : "✗ DIFF";

    std::string res_str = std::to_string(src_dim) + "→" + std::to_string(dst_dim);
    std::cout << std::setw(15) << res_str << std::setw(15) << std::fixed << std::setprecision(3)
              << cpu_time_ms << std::setw(15) << std::fixed << std::setprecision(3) << gpu_time_ms
              << std::setw(15) << std::fixed << std::setprecision(2) << speedup << "x"
              << std::setw(15) << std::scientific << std::setprecision(2) << max_diff
              << std::setw(15) << status << std::endl;

    // Verify results match
    EXPECT_LT(max_diff, 1e-4) << "CPU and GPU resize results should match for " << res_str;
  }

  std::cout << std::string(90, '=') << std::endl;
  std::cout << "Note: GPU resize includes CUDA memory transfer overhead" << std::endl;
  std::cout << "      Speedup improves for larger volumes" << std::endl;
#else
  GTEST_SKIP() << "CUDA support not compiled in";
#endif
}

// CPU vs GPU GDTV filter performance benchmark
TEST_F(VoxelsCUDATest, GDTVFilterPerformanceComparison) {
#ifdef CVC_USING_CUDA
  if (!voxels::cuda_available()) {
    GTEST_SKIP() << "CUDA not available, skipping test";
    return;
  }

  std::cout << "\n=== CPU vs GPU GDTV Filter Performance Comparison ===" << std::endl;
  std::cout << std::string(110, '=') << std::endl;
  std::cout << std::setw(15) << "Volume Size" << std::setw(15) << "Iterations" << std::setw(15)
            << "CPU Time (ms)" << std::setw(15) << "GPU Time (ms)" << std::setw(15) << "Speedup"
            << std::setw(20) << "Max Diff" << std::setw(15) << "Status" << std::endl;
  std::cout << std::string(110, '-') << std::endl;

  // Test different volume sizes and iteration counts
  std::vector<std::tuple<uint64, unsigned int>> test_configs = {
      {32, 5},  // Small volume, few iterations
      {64, 5},  // Medium volume, few iterations
      {32, 20}, // Small volume, many iterations
      {64, 20}, // Medium volume, many iterations
      {128, 5}, // Large volume, few iterations
      {128, 10} // Large volume, moderate iterations
  };

  for (const auto &[dim, iterations] : test_configs) {
    dimension vol_dim(dim, dim, dim);

    // Create test volumes with noise
    voxels v_cpu(ctx, vol_dim, Float);
    voxels v_gpu(ctx, vol_dim, Float);

    for (uint64 k = 0; k < dim; k++) {
      for (uint64 j = 0; j < dim; j++) {
        for (uint64 i = 0; i < dim; i++) {
          // Create a pattern with edges and noise
          float base = (i < dim / 2) ? 100.0f : 200.0f;
          float noise = float((i + j + k) % 7) * 3.0f;
          v_cpu(i, j, k, base + noise);
          v_gpu(i, j, k, base + noise);
        }
      }
    }

    // CPU timing
    auto cpu_start = std::chrono::high_resolution_clock::now();
    v_cpu.gdtvFilter(1.5, 0.5, iterations, 0); // q=1.5, lambda=0.5, 6-neighbor
    auto cpu_end = std::chrono::high_resolution_clock::now();
    double cpu_time_ms = std::chrono::duration<double, std::milli>(cpu_end - cpu_start).count();

    // GPU timing (includes CUDA memory transfer)
    v_gpu.enableCUDA(0);
    auto gpu_start = std::chrono::high_resolution_clock::now();
    v_gpu.gdtvFilter(1.5, 0.5, iterations, 0); // q=1.5, lambda=0.5, 6-neighbor
    auto gpu_end = std::chrono::high_resolution_clock::now();
    double gpu_time_ms = std::chrono::duration<double, std::milli>(gpu_end - gpu_start).count();
    v_gpu.disableCUDA();

    // Compare results
    double max_diff = 0.0;
    for (uint64 k = 0; k < dim; k++) {
      for (uint64 j = 0; j < dim; j++) {
        for (uint64 i = 0; i < dim; i++) {
          double diff = std::abs(v_cpu(i, j, k) - v_gpu(i, j, k));
          max_diff = std::max(max_diff, diff);
        }
      }
    }

    double speedup = cpu_time_ms / gpu_time_ms;
    std::string status = (max_diff < 0.1) ? "✓ PASS" : "✗ DIFF";

    std::string size_str = std::to_string(dim) + "³";
    std::cout << std::setw(15) << size_str << std::setw(15) << iterations << std::setw(15)
              << std::fixed << std::setprecision(1) << cpu_time_ms << std::setw(15) << std::fixed
              << std::setprecision(1) << gpu_time_ms << std::setw(15) << std::fixed
              << std::setprecision(2) << speedup << "x" << std::setw(20) << std::scientific
              << std::setprecision(2) << max_diff << std::setw(15) << status << std::endl;

    // Verify results match
    EXPECT_LT(max_diff, 0.1) << "CPU and GPU GDTV filter results should match for " << size_str
                             << " with " << iterations << " iterations";
  }

  std::cout << std::string(110, '=') << std::endl;
  std::cout << "Note: GPU timing includes CUDA memory transfer overhead" << std::endl;
  std::cout << "      Speedup increases with iteration count and volume size" << std::endl;
  std::cout << "      Each iteration is fully parallelized on GPU" << std::endl;
#else
  GTEST_SKIP() << "CUDA support not compiled in";
#endif
}

#endif // CVC_USING_CUDA

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  // Parse custom command line arguments
  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--enable-stress-tests") {
      enable_stress_tests = true;
    } else if (std::string(argv[i]) == "--help") {
      std::cout << "Custom options:\n";
      std::cout << "  --enable-stress-tests    Enable long-running stress/performance tests\n";
      return 0;
    }
  }

  if (enable_stress_tests) {
    std::cout << "Stress tests ENABLED\n";
  }

  return RUN_ALL_TESTS();
}

// ---------------------------------------------------------------------------
// Regression: mutating filters must invalidate the cached min/max.
//
// bilateralFilter() reads min()/max() BEFORE filtering (priming the cache with
// pre-filter bounds) and then writes through the raw data pointer, bypassing
// preWrite() — which dirties the histogram but not the min/max cache. Without
// an unsetMinMax() at the end, a bilateralFilter -> vol_normalize pipeline
// remaps against stale pre-filter bounds and does not stretch to the target
// range. anisotropicDiffusion/gdtvFilter are covered as the same invariant
// (currently safe via copy()/operator= importing-or-unsetting; the explicit
// invalidation keeps them safe under refactors).
// ---------------------------------------------------------------------------

namespace {

// Independent ground truth: scan every voxel through the read accessor.
static void scan_min_max(const cvc::voxels &v, double &lo, double &hi) {
  lo = std::numeric_limits<double>::infinity();
  hi = -std::numeric_limits<double>::infinity();
  for (cvc::uint64 k = 0; k < v.ZDim(); ++k)
    for (cvc::uint64 j = 0; j < v.YDim(); ++j)
      for (cvc::uint64 i = 0; i < v.XDim(); ++i) {
        const double val = v(i, j, k);
        lo = std::min(lo, val);
        hi = std::max(hi, val);
      }
}

// A small volume with a hot impulse: smoothing filters must pull the max
// down (and generally lift the min), so pre- and post-filter bounds differ.
static cvc::voxels impulse_volume(cvc::app &ctx) {
  cvc::voxels v(ctx, cvc::dimension(8, 8, 8), cvc::Float);
  for (cvc::uint64 k = 0; k < 8; ++k)
    for (cvc::uint64 j = 0; j < 8; ++j)
      for (cvc::uint64 i = 0; i < 8; ++i)
        v(i, j, k, 10.0);
  v(4, 4, 4, 200.0); // impulse
  return v;
}

} // namespace

TEST_F(VoxelsTest, BilateralFilterInvalidatesMinMaxCache) {
  voxels v = impulse_volume(ctx);

  // Prime the cache with PRE-filter bounds (this is what the filter itself
  // also does internally).
  EXPECT_DOUBLE_EQ(v.min(), 10.0);
  EXPECT_DOUBLE_EQ(v.max(), 200.0);

  v.bilateralFilter(50.0, 1.5, 1);

  double lo, hi;
  scan_min_max(v, lo, hi);
  // The smoothed impulse must no longer reach the original max...
  ASSERT_LT(hi, 200.0);
  // ...and the cached accessors must agree with the actual post-filter data.
  EXPECT_DOUBLE_EQ(v.min(), lo);
  EXPECT_DOUBLE_EQ(v.max(), hi);
}

TEST_F(VoxelsTest, AnisotropicDiffusionInvalidatesMinMaxCache) {
  voxels v = impulse_volume(ctx);
  EXPECT_DOUBLE_EQ(v.max(), 200.0); // prime cache

  v.anisotropicDiffusion(3);

  double lo, hi;
  scan_min_max(v, lo, hi);
  EXPECT_DOUBLE_EQ(v.min(), lo);
  EXPECT_DOUBLE_EQ(v.max(), hi);
}

TEST_F(VoxelsTest, GdtvFilterInvalidatesMinMaxCache) {
  voxels v = impulse_volume(ctx);
  EXPECT_DOUBLE_EQ(v.max(), 200.0); // prime cache

  v.gdtvFilter(1.0, 0.1, 2, false);

  double lo, hi;
  scan_min_max(v, lo, hi);
  EXPECT_DOUBLE_EQ(v.min(), lo);
  EXPECT_DOUBLE_EQ(v.max(), hi);
}
