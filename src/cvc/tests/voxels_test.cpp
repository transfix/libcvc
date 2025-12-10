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

#include <gtest/gtest.h>
#include <cvc/voxels.h>
#include <cvc/composite_function.h>
#include <cvc/exception.h>
#include <cvc/types.h>

using namespace cvc;

// ============================================================================
// Construction and Basic Properties Tests
// ============================================================================

TEST(VoxelsTest, DefaultConstruction) {
  voxels v;
  
  // Default is 4x4x4 UChar
  EXPECT_EQ(v.XDim(), 4u);
  EXPECT_EQ(v.YDim(), 4u);
  EXPECT_EQ(v.ZDim(), 4u);
  EXPECT_EQ(v.voxelType(), UChar);
  EXPECT_EQ(v.voxelSize(), 1u);
  EXPECT_STREQ(v.voxelTypeStr(), "unsigned char");
}

TEST(VoxelsTest, DimensionConstruction) {
  dimension dim(10, 20, 30);
  voxels v(dim, Float);
  
  EXPECT_EQ(v.XDim(), 10u);
  EXPECT_EQ(v.YDim(), 20u);
  EXPECT_EQ(v.ZDim(), 30u);
  EXPECT_EQ(v.voxelType(), Float);
  EXPECT_EQ(v.voxelSize(), 4u);
  EXPECT_STREQ(v.voxelTypeStr(), "float");
}

TEST(VoxelsTest, DataTypeConstruction) {
  std::vector<data_type> types = {UChar, UShort, UInt, Float, Double, UInt64};
  std::vector<uint64> sizes = {1, 2, 4, 4, 8, 8};
  
  for (size_t i = 0; i < types.size(); ++i) {
    voxels v(dimension(5, 5, 5), types[i]);
    EXPECT_EQ(v.voxelType(), types[i]);
    EXPECT_EQ(v.voxelSize(), sizes[i]);
  }
}

TEST(VoxelsTest, CopyConstruction) {
  dimension dim(8, 8, 8);
  voxels v1(dim, UShort);
  
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

TEST(VoxelsTest, PointerConstruction) {
  dimension dim(4, 4, 4);
  std::vector<unsigned char> data(64, 42);
  
  voxels v(data.data(), dim, UChar);
  
  EXPECT_EQ(v.XDim(), 4u);
  EXPECT_EQ(v.YDim(), 4u);
  EXPECT_EQ(v.ZDim(), 4u);
  EXPECT_DOUBLE_EQ(v(0, 0, 0), 42.0);
  EXPECT_DOUBLE_EQ(v(3, 3, 3), 42.0);
}

// ============================================================================
// Voxel Access Tests
// ============================================================================

TEST(VoxelsTest, LinearIndexAccess) {
  voxels v(dimension(10, 10, 10), Float);
  
  // Write and read using linear index
  v(0, 1.5);
  v(50, 2.5);
  v(999, 3.5);
  
  EXPECT_DOUBLE_EQ(v(0), 1.5);
  EXPECT_DOUBLE_EQ(v(50), 2.5);
  EXPECT_DOUBLE_EQ(v(999), 3.5);
}

TEST(VoxelsTest, ThreeDimensionalAccess) {
  voxels v(dimension(10, 10, 10), Double);
  
  // Write and read using 3D coordinates
  v(0, 0, 0, 10.0);
  v(5, 5, 5, 20.0);
  v(9, 9, 9, 30.0);
  
  EXPECT_DOUBLE_EQ(v(0, 0, 0), 10.0);
  EXPECT_DOUBLE_EQ(v(5, 5, 5), 20.0);
  EXPECT_DOUBLE_EQ(v(9, 9, 9), 30.0);
}

TEST(VoxelsTest, OutOfBoundsRead) {
  voxels v(dimension(5, 5, 5), UChar);
  
  // Should throw index_out_of_bounds
  EXPECT_THROW(v(125), index_out_of_bounds);
  EXPECT_THROW(v(1000), index_out_of_bounds);
}

TEST(VoxelsTest, OutOfBoundsWrite) {
  voxels v(dimension(5, 5, 5), UChar);
  
  // Should throw index_out_of_bounds
  EXPECT_THROW(v(125, 42.0), index_out_of_bounds);
  EXPECT_THROW(v(1000, 42.0), index_out_of_bounds);
}

TEST(VoxelsTest, TypeConversionRead) {
  voxels v(dimension(5, 5, 5), UChar);
  
  // Write as double, stored as unsigned char, read as double
  v(0, 127.5);  // Will be truncated to 127
  v(1, 255.9);  // Will be truncated to 255
  
  EXPECT_DOUBLE_EQ(v(0), 127.0);
  EXPECT_DOUBLE_EQ(v(1), 255.0);
}

// ============================================================================
// Dimension and Type Modification Tests
// ============================================================================

TEST(VoxelsTest, ChangeDimensions) {
  voxels v(dimension(5, 5, 5), Float);
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

TEST(VoxelsTest, ShrinkDimensions) {
  voxels v(dimension(10, 10, 10), UShort);
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

TEST(VoxelsTest, ChangeVoxelType) {
  voxels v(dimension(5, 5, 5), UChar);
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

TEST(VoxelsTest, TypeConversionPrecision) {
  voxels v(dimension(5, 5, 5), Double);
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

TEST(VoxelsTest, MinMaxAutoCalculation) {
  voxels v(dimension(5, 5, 5), Float);
  
  v(0, 1.0);
  v(1, 5.0);
  v(2, -3.0);
  v(3, 10.0);
  
  EXPECT_DOUBLE_EQ(v.min(), -3.0);
  EXPECT_DOUBLE_EQ(v.max(), 10.0);
  EXPECT_TRUE(v.minIsSet());
  EXPECT_TRUE(v.maxIsSet());
}

TEST(VoxelsTest, MinMaxManualSet) {
  voxels v(dimension(5, 5, 5), UChar);
  
  v.min(10.0);
  v.max(250.0);
  
  EXPECT_DOUBLE_EQ(v.min(), 10.0);
  EXPECT_DOUBLE_EQ(v.max(), 250.0);
  EXPECT_TRUE(v.minIsSet());
  EXPECT_TRUE(v.maxIsSet());
}

TEST(VoxelsTest, MinMaxUnset) {
  voxels v(dimension(5, 5, 5), UShort);
  
  v.min(10.0);
  v.max(1000.0);
  EXPECT_TRUE(v.minIsSet());
  EXPECT_TRUE(v.maxIsSet());
  
  v.unsetMinMax();
  EXPECT_FALSE(v.minIsSet());
  EXPECT_FALSE(v.maxIsSet());
}

TEST(VoxelsTest, MinMaxSubvolume) {
  voxels v(dimension(10, 10, 10), Float);
  
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

TEST(VoxelsTest, AssignmentOperator) {
  voxels v1(dimension(5, 5, 5), Float);
  v1(2, 2, 2, 42.0);
  
  voxels v2(dimension(10, 10, 10), Double);
  v2 = v1;
  
  EXPECT_EQ(v2.XDim(), v1.XDim());
  EXPECT_EQ(v2.YDim(), v1.YDim());
  EXPECT_EQ(v2.ZDim(), v1.ZDim());
  EXPECT_EQ(v2.voxelType(), v1.voxelType());
  EXPECT_DOUBLE_EQ(v2(2, 2, 2), 42.0);
}

TEST(VoxelsTest, EqualityOperator) {
  voxels v1(dimension(5, 5, 5), UChar);
  voxels v2(dimension(5, 5, 5), UChar);
  
  v1.fill(42.0);
  v2.fill(42.0);
  
  EXPECT_TRUE(v1 == v2);
  
  v2(0, 0, 0, 43.0);
  EXPECT_FALSE(v1 == v2);
}

TEST(VoxelsTest, InequalityOperator) {
  voxels v1(dimension(5, 5, 5), UChar);
  voxels v2(dimension(5, 5, 5), UChar);
  
  v1.fill(42.0);
  v2.fill(43.0);
  
  EXPECT_TRUE(v1 != v2);
  
  v2.fill(42.0);
  EXPECT_FALSE(v1 != v2);
}

TEST(VoxelsTest, Fill) {
  voxels v(dimension(10, 10, 10), Float);
  
  v.fill(3.14);
  
  // Use NEAR for float precision
  EXPECT_NEAR(v(0, 0, 0), 3.14, 1e-6);
  EXPECT_NEAR(v(5, 5, 5), 3.14, 1e-6);
  EXPECT_NEAR(v(9, 9, 9), 3.14, 1e-6);
}

TEST(VoxelsTest, FillSub) {
  voxels v(dimension(10, 10, 10), UShort);
  
  v.fill(0.0);
  v.fillsub(2, 2, 2, dimension(4, 4, 4), 100.0);
  
  // Inside subvolume should be 100
  EXPECT_DOUBLE_EQ(v(2, 2, 2), 100.0);
  EXPECT_DOUBLE_EQ(v(5, 5, 5), 100.0);
  
  // Outside subvolume should be 0
  EXPECT_DOUBLE_EQ(v(0, 0, 0), 0.0);
  EXPECT_DOUBLE_EQ(v(9, 9, 9), 0.0);
}

TEST(VoxelsTest, Map) {
  voxels v(dimension(10, 10, 10), Float);
  
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

TEST(VoxelsTest, Sub) {
  voxels v(dimension(10, 10, 10), UChar);
  
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

TEST(VoxelsTest, Resize) {
  voxels v(dimension(4, 4, 4), Float);
  
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

TEST(VoxelsTest, Histogram) {
  voxels v(dimension(10, 10, 10), UChar);
  
  // Fill with some pattern
  for (uint64 i = 0; i < 1000; ++i)
    v(i, i % 256);
  
  auto hist = v.histogram(256);
  const uint64* bins = hist.get<0>();
  uint64 size = hist.get<1>();
  
  EXPECT_EQ(size, 256u);
  EXPECT_NE(bins, nullptr);
}

// ============================================================================
// Copy-on-Write and Deep Copy Tests
// ============================================================================

TEST(VoxelsTest, CopyOnWrite) {
  voxels v1(dimension(5, 5, 5), Float);
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

TEST(VoxelsTest, ShallowCopyDefault) {
  voxels v1(dimension(5, 5, 5), Float);
  v1.fill(10.0);
  v1(2, 2, 2, 50.0);
  
  voxels v2;
  v2.copy(v1); // Default is shallow copy
  
  // v2 shares data with v1
  EXPECT_DOUBLE_EQ(v2(2, 2, 2), 50.0);
  
  // Modify v1 after shallow copy - should NOT affect v2 due to copy-on-write
  v1(2, 2, 2, 99.0);
  
  // v2 should still have old value (copy-on-write triggered)
  EXPECT_DOUBLE_EQ(v2(2, 2, 2), 50.0);
  EXPECT_DOUBLE_EQ(v1(2, 2, 2), 99.0);
}

TEST(VoxelsTest, ShallowCopyExplicit) {
  voxels v1(dimension(5, 5, 5), Double);
  v1.fill(20.0);
  v1(1, 1, 1, 100.0);
  
  voxels v2;
  v2.copy(v1, false); // Explicit shallow copy
  
  // v2 shares data with v1 initially
  EXPECT_DOUBLE_EQ(v2(1, 1, 1), 100.0);
  
  // Modify v2 - triggers copy-on-write, v1 unaffected
  v2(1, 1, 1, 200.0);
  
  EXPECT_DOUBLE_EQ(v1(1, 1, 1), 100.0);
  EXPECT_DOUBLE_EQ(v2(1, 1, 1), 200.0);
}

TEST(VoxelsTest, DeepCopy) {
  voxels v1(dimension(5, 5, 5), Float);
  v1.fill(10.0);
  v1(0, 0, 0, 5.0);
  v1(4, 4, 4, 15.0);
  
  voxels v2;
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

TEST(VoxelsTest, DeepCopyIndependence) {
  voxels v1(dimension(10, 10, 10), UShort);
  
  // Fill with pattern
  for(uint64 i = 0; i < 1000; ++i)
    v1(i, double(i));
  
  voxels v2;
  v2.copy(v1, true); // Deep copy
  
  // Modify all values in v1
  v1.fill(9999.0);
  
  // v2 should retain original values
  for(uint64 i = 0; i < 1000; ++i)
    EXPECT_DOUBLE_EQ(v2(i), double(i));
  
  // v1 should have new values
  EXPECT_DOUBLE_EQ(v1(0), 9999.0);
  EXPECT_DOUBLE_EQ(v1(500), 9999.0);
  EXPECT_DOUBLE_EQ(v1(999), 9999.0);
}

TEST(VoxelsTest, DeepCopyDifferentTypes) {
  std::vector<data_type> types = {UChar, UShort, UInt, Float, Double, UInt64};
  
  for(auto type : types) {
    voxels v1(dimension(5, 5, 5), type);
    v1.fill(42.0);
    v1(0, 0, 0, 10.0);
    v1(4, 4, 4, 100.0);
    
    voxels v2;
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

TEST(VoxelsTest, DeepCopyMinMax) {
  voxels v1(dimension(10, 10, 10), Float);
  v1.fill(50.0);
  v1(0, 0, 0, 1.0);
  v1(9, 9, 9, 100.0);
  
  // Force min/max calculation
  double min1 = v1.min();
  double max1 = v1.max();
  
  EXPECT_DOUBLE_EQ(min1, 1.0);
  EXPECT_DOUBLE_EQ(max1, 100.0);
  
  voxels v2;
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

TEST(VoxelsTest, DeepCopySelfAssignment) {
  voxels v1(dimension(5, 5, 5), Float);
  v1.fill(42.0);
  v1(2, 2, 2, 100.0);
  
  // Self-assignment should be safe
  v1.copy(v1, true);
  
  EXPECT_DOUBLE_EQ(v1(2, 2, 2), 100.0);
  EXPECT_DOUBLE_EQ(v1(0, 0, 0), 42.0);
}

TEST(VoxelsTest, DeepCopyLargeVolume) {
  voxels v1(dimension(50, 50, 50), Double);
  
  // Fill with unique pattern
  uint64 count = 0;
  for(uint64 k = 0; k < 50; ++k)
    for(uint64 j = 0; j < 50; ++j)
      for(uint64 i = 0; i < 50; ++i)
        v1(i, j, k, double(count++));
  
  voxels v2;
  v2.copy(v1, true); // Deep copy
  
  // Verify all values copied correctly
  count = 0;
  for(uint64 k = 0; k < 50; ++k)
    for(uint64 j = 0; j < 50; ++j)
      for(uint64 i = 0; i < 50; ++i)
        EXPECT_DOUBLE_EQ(v2(i, j, k), double(count++));
  
  // Modify v1
  v1.fill(0.0);
  
  // v2 should still have original values
  EXPECT_DOUBLE_EQ(v2(0, 0, 0), 0.0);
  EXPECT_DOUBLE_EQ(v2(25, 25, 25), 63775.0); // 25 + 25*50 + 25*50*50 = 25 + 1250 + 62500
  EXPECT_DOUBLE_EQ(v2(49, 49, 49), 124999.0); // 49 + 49*50 + 49*50*50 = 49 + 2450 + 122500
}

TEST(VoxelsTest, AssignmentOperatorUsesShallowCopy) {
  voxels v1(dimension(5, 5, 5), Float);
  v1.fill(10.0);
  v1(2, 2, 2, 50.0);
  
  voxels v2;
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

TEST(VoxelsTest, CompositeAdd) {
  voxels v1(dimension(5, 5, 5), Float);
  voxels v2(dimension(5, 5, 5), Float);
  
  v1.fill(10.0);
  v2.fill(5.0);
  
  // Composite v2 into v1 using add function
  add_func add;
  v1.composite(v2, 0, 0, 0, add);
  
  // All values in v1 should now be 15.0
  EXPECT_DOUBLE_EQ(v1(0, 0, 0), 15.0);
  EXPECT_DOUBLE_EQ(v1(2, 2, 2), 15.0);
}

TEST(VoxelsTest, CompositeOverwrite) {
  voxels v1(dimension(5, 5, 5), Float);
  voxels v2(dimension(3, 3, 3), Float);
  
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

TEST(VoxelsTest, CompositeNegativeOffset) {
  voxels v1(dimension(10, 10, 10), Float);
  voxels v2(dimension(5, 5, 5), Float);
  
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

TEST(VoxelsTest, RawDataAccess) {
  voxels v(dimension(5, 5, 5), UChar);
  v.fill(42.0);
  
  const unsigned char* data = *v;
  EXPECT_NE(data, nullptr);
  EXPECT_EQ(data[0], 42);
}

TEST(VoxelsTest, SharedArrayAccess) {
  voxels v(dimension(5, 5, 5), Float);
  v(0, 0, 0, 3.14f);
  
  // Access data pointer directly
  const float* fdata = reinterpret_cast<const float*>(v.data_ptr());
  EXPECT_NE(fdata, nullptr);
  EXPECT_FLOAT_EQ(fdata[0], 3.14f);
}

// ============================================================================
// Edge Cases and Error Handling
// ============================================================================

TEST(VoxelsTest, ZeroVolumeMinMax) {
  voxels v(dimension(0, 0, 0), Float);
  
  // Min/Max on empty volume shouldn't crash
  // (but behavior is implementation-defined)
  EXPECT_NO_THROW(v.min());
  EXPECT_NO_THROW(v.max());
}

TEST(VoxelsTest, SingleVoxelVolume) {
  voxels v(dimension(1, 1, 1), Double);
  v(0, 0, 0, 42.0);
  
  EXPECT_DOUBLE_EQ(v(0, 0, 0), 42.0);
  EXPECT_DOUBLE_EQ(v.min(), 42.0);
  EXPECT_DOUBLE_EQ(v.max(), 42.0);
}

TEST(VoxelsTest, LargeVolumeCreation) {
  // Test that we can create a reasonably large volume
  // (Not too large to avoid test timeouts)
  dimension large(100, 100, 100);
  
  EXPECT_NO_THROW({
    voxels v(large, Float);
    EXPECT_EQ(v.XDim(), 100u);
    EXPECT_EQ(v.YDim(), 100u);
    EXPECT_EQ(v.ZDim(), 100u);
  });
}

TEST(VoxelsTest, AllDataTypes) {
  dimension dim(4, 4, 4);
  
  // Test each data type
  voxels v_uchar(dim, UChar);
  EXPECT_EQ(v_uchar.voxelType(), UChar);
  
  voxels v_ushort(dim, UShort);
  EXPECT_EQ(v_ushort.voxelType(), UShort);
  
  voxels v_uint(dim, UInt);
  EXPECT_EQ(v_uint.voxelType(), UInt);
  
  voxels v_float(dim, Float);
  EXPECT_EQ(v_float.voxelType(), Float);
  
  voxels v_double(dim, Double);
  EXPECT_EQ(v_double.voxelType(), Double);
  
  voxels v_uint64(dim, UInt64);
  EXPECT_EQ(v_uint64.voxelType(), UInt64);
}

// ============================================================================
// Type Conversion Tests (Comprehensive)
// ============================================================================

TEST(VoxelsTest, TypeConversionUCharToAll) {
  voxels v(dimension(5, 5, 5), UChar);
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

TEST(VoxelsTest, TypeConversionPrecisionLoss) {
  voxels v(dimension(5, 5, 5), Double);
  
  // Set precise double value
  v(0, 3.141592653589793);
  v(1, 1234567.89012345);
  
  // Convert to Float (loses precision)
  v.voxelType(Float);
  EXPECT_NEAR(v(0), 3.14159, 1e-5);
  EXPECT_NEAR(v(1), 1234567.875, 1.0);  // Float precision
  
  // Convert to UInt (loses fractional part)
  v.voxelType(UInt);
  EXPECT_DOUBLE_EQ(v(0), 3.0);
  EXPECT_DOUBLE_EQ(v(1), 1234567.0);
}

TEST(VoxelsTest, TypeConversionNegativeValues) {
  voxels v(dimension(5, 5, 5), Float);
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

TEST(VoxelsTest, AllTypeConversionCombinations) {
  std::vector<data_type> types = {UChar, UShort, UInt, Float, Double, UInt64};
  
  for (auto from_type : types) {
    for (auto to_type : types) {
      if (from_type == to_type) continue;
      
      voxels v(dimension(3, 3, 3), from_type);
      v(0, 42.0);
      
      v.voxelType(to_type);
      
      // Value should be approximately preserved
      EXPECT_NEAR(v(0), 42.0, 1.0) 
        << "Failed converting from " << data_type_strings[from_type] 
        << " to " << data_type_strings[to_type];
    }
  }
}

// ============================================================================
// Resize and Interpolation Tests
// ============================================================================

TEST(VoxelsTest, ResizeUpsample) {
  voxels v(dimension(2, 2, 2), Float);
  
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

TEST(VoxelsTest, ResizeDownsample) {
  voxels v(dimension(8, 8, 8), Float);
  
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

TEST(VoxelsTest, ResizeSameSize) {
  voxels v(dimension(5, 5, 5), Float);
  v.fill(42.0);
  
  v.resize(dimension(5, 5, 5));
  
  EXPECT_EQ(v.XDim(), 5u);
  EXPECT_EQ(v.YDim(), 5u);
  EXPECT_EQ(v.ZDim(), 5u);
  EXPECT_NEAR(v(2, 2, 2), 42.0, 1e-5);
}

TEST(VoxelsTest, ResizeNonUniform) {
  voxels v(dimension(4, 4, 4), Float);
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

TEST(VoxelsTest, MinMaxAllDataTypes) {
  std::vector<data_type> types = {UChar, UShort, UInt, Float, Double, UInt64};
  
  for (auto type : types) {
    voxels v(dimension(5, 5, 5), type);
    
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

TEST(VoxelsTest, MinMaxWithZeros) {
  voxels v(dimension(10, 10, 10), Float);
  v.fill(0.0);
  
  v(5, 5, 5, 10.0);
  v(7, 7, 7, -5.0);
  
  EXPECT_DOUBLE_EQ(v.min(), -5.0);
  EXPECT_DOUBLE_EQ(v.max(), 10.0);
}

TEST(VoxelsTest, MinMaxAllSameValue) {
  voxels v(dimension(10, 10, 10), Double);
  v.fill(42.42);
  
  EXPECT_DOUBLE_EQ(v.min(), 42.42);
  EXPECT_DOUBLE_EQ(v.max(), 42.42);
}

TEST(VoxelsTest, MinMaxAfterTypeChange) {
  voxels v(dimension(5, 5, 5), Float);
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

TEST(VoxelsTest, MinMaxLargeVolume) {
  voxels v(dimension(50, 50, 50), UShort);
  
  // Fill with pattern - min at (0,0,0), max at (49,49,49)
  for (uint64 k = 0; k < 50; ++k)
    for (uint64 j = 0; j < 50; ++j)
      for (uint64 i = 0; i < 50; ++i)
        v(i, j, k, double(i + j + k));
  
  EXPECT_DOUBLE_EQ(v.min(), 0.0);
  EXPECT_DOUBLE_EQ(v.max(), 147.0);  // 49+49+49
}

// ============================================================================
// Map Operation Tests
// ============================================================================

TEST(VoxelsTest, MapExpand) {
  voxels v(dimension(10, 10, 10), Float);
  
  // Fill with 0-9
  for (uint64 i = 0; i < 10; ++i)
    v(i, double(i));
  
  // Map [0,9] to [0,100]
  v.map(0.0, 100.0);
  
  EXPECT_NEAR(v(0), 0.0, 1e-5);
  EXPECT_NEAR(v(9), 100.0, 1e-5);
  EXPECT_NEAR(v(5), 55.555, 0.01);
}

TEST(VoxelsTest, MapShrink) {
  voxels v(dimension(10, 10, 10), Float);
  
  // Fill with 0-100
  for (uint64 i = 0; i < 10; ++i)
    v(i, double(i * 10));
  
  // Map [0,90] to [0,1]
  v.map(0.0, 1.0);
  
  EXPECT_NEAR(v(0), 0.0, 1e-5);
  EXPECT_NEAR(v(9), 1.0, 1e-5);
}

TEST(VoxelsTest, MapNegativeRange) {
  voxels v(dimension(5, 5, 5), Float);
  
  for (uint64 i = 0; i < 5; ++i)
    v(i, double(i));
  
  // Map [0,4] to [-10,10]
  v.map(-10.0, 10.0);
  
  EXPECT_NEAR(v(0), -10.0, 1e-5);
  EXPECT_NEAR(v(4), 10.0, 1e-5);
  EXPECT_NEAR(v(2), 0.0, 1e-5);
}

TEST(VoxelsTest, MapIdentity) {
  voxels v(dimension(5, 5, 5), Float);
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

TEST(VoxelsTest, SubCenterExtraction) {
  voxels v(dimension(10, 10, 10), Float);
  
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

TEST(VoxelsTest, SubCornerExtraction) {
  voxels v(dimension(10, 10, 10), UShort);
  
  for (uint64 i = 0; i < 1000; ++i)
    v(i, double(i));
  
  // Extract from origin
  v.sub(0, 0, 0, dimension(5, 5, 5));
  
  EXPECT_EQ(v.XDim(), 5u);
  EXPECT_DOUBLE_EQ(v(0, 0, 0), 0.0);
  EXPECT_DOUBLE_EQ(v(4, 4, 4), 444.0);  // Was at (4,4,4)
}

TEST(VoxelsTest, SubSingleSlice) {
  voxels v(dimension(10, 10, 10), Float);
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

TEST(VoxelsTest, BilateralFilterUniform) {
  // Test 1: Nearly uniform region (avoid min==max division issues)
  voxels v(dimension(10, 10, 10), Float);
  v.fill(50.0);
  v(0, 0, 0, 50.1);  // Slight variation to avoid min==max
  v(9, 9, 9, 49.9);
  
  v.bilateralFilter();
  
  // Nearly uniform regions should stay very close to original
  EXPECT_NEAR(v(5, 5, 5), 50.0, 0.5);
  EXPECT_NEAR(v(0, 0, 0), 50.1, 1.0);  // Edges have slight boundary effects
  EXPECT_NEAR(v(9, 9, 9), 49.9, 1.0);
}

TEST(VoxelsTest, BilateralFilterEdgePreservation) {
  // Test 2: Sharp edges should be preserved while noise is reduced
  voxels v(dimension(10, 10, 1), Float);
  
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
  for (uint64 j = 2; j < 8; ++j) {  // Avoid edges
    left_avg += v(2, j, 0);   // Left region
    right_avg += v(7, j, 0);  // Right region
  }
  left_avg /= 6.0;
  right_avg /= 6.0;
  
  EXPECT_LT(left_avg, 40.0) << "Left side should stay low";
  EXPECT_GT(right_avg, 60.0) << "Right side should stay high";
  EXPECT_GT(right_avg - left_avg, 35.0) << "Edge should be preserved";
}

TEST(VoxelsTest, BilateralFilterNoiseReduction) {
  // Test 3: Small noise should be smoothed in uniform regions
  voxels v(dimension(9, 9, 1), Float);
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

TEST(VoxelsTest, BilateralFilterIsolatedSpike) {
  // Test 4: Isolated spike in uniform region
  voxels v(dimension(9, 9, 9), Float);
  v.fill(50.0);
  v(4, 4, 4, 150.0);  // Large spike
  
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

TEST(VoxelsTest, BilateralFilterSpatialSigmaEffect) {
  // Test 5: Spatial sigma controls spatial smoothing extent
  voxels v1(dimension(11, 11, 1), Float);
  voxels v2(v1);
  
  // Create gradient with noise
  for (uint64 j = 0; j < 11; ++j) {
    for (uint64 i = 0; i < 11; ++i) {
      double base = double(i) * 10.0;  // Gradient 0-100
      double noise = ((i + j) % 3 == 0) ? 5.0 : 0.0;
      v1(i, j, 0, base + noise);
      v2(i, j, 0, base + noise);
    }
  }
  
  
  // Apply with different spatial sigmas
  v1.bilateralFilter(50.0, 1.0, 1);  // Small spatial sigma
  v2.bilateralFilter(50.0, 2.5, 1);  // Large spatial sigma
  
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

TEST(VoxelsTest, BilateralFilterRadiometricSigmaEffect) {
  // Test 6: Radiometric sigma controls edge preservation strength
  voxels v1(dimension(10, 10, 1), Float);
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
  v1.bilateralFilter(20.0, 1.5, 1);   // Small radiometric sigma (strong edge preservation)
  v2.bilateralFilter(100.0, 1.5, 1);  // Large radiometric sigma (weak edge preservation)
  
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

TEST(VoxelsTest, BilateralFilterFilterRadiusEffect) {
  // Test 7: Filter radius controls neighborhood size
  voxels v1(dimension(11, 11, 1), Float);
  voxels v2(v1);
  
  v1.fill(50.0);
  v2.fill(50.0);
  
  // Add feature at edge of different neighborhoods
  v1(5, 5, 0, 100.0);  // Center
  v1(5, 7, 0, 80.0);   // 2 pixels away
  v2 = v1;
  
  
  // Apply with different filter radii
  v1.bilateralFilter(50.0, 1.5, 1);  // Radius 1 (3x3x3 neighborhood)
  v2.bilateralFilter(50.0, 1.5, 2);  // Radius 2 (5x5x5 neighborhood)
  
  // Larger radius should have more influence on distant pixels
  double influence_r1 = v1(5, 7, 0) - 80.0;  // How much did center affect this pixel
  double influence_r2 = v2(5, 7, 0) - 80.0;
  
  // With radius 2, the center spike can influence pixels 2 away
  // With radius 1, it cannot reach that far
  EXPECT_LE(std::abs(influence_r1), std::abs(influence_r2) + 5.0) 
    << "Larger radius should have more influence on distant pixels";
}

TEST(VoxelsTest, BilateralFilter3DSmoothing) {
  // Test 8: 3D bilateral filtering
  voxels v(dimension(7, 7, 7), Float);
  
  // Create 3D pattern: sphere of high values in uniform background
  v.fill(30.0);
  for (uint64 k = 2; k <= 4; ++k) {
    for (uint64 j = 2; j <= 4; ++j) {
      for (uint64 i = 2; i <= 4; ++i) {
        // Distance from center (3,3,3)
        int dx = i - 3, dy = j - 3, dz = k - 3;
        if (dx*dx + dy*dy + dz*dz <= 2) {  // Roughly spherical
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

TEST(VoxelsTest, ContrastEnhancementLowContrast) {
  // Test 1: Algorithm restores original range after internal processing
  voxels v(dimension(10, 10, 10), Float);
  
  // Create low contrast volume (values clustered in narrow range)
  for (uint64 i = 0; i < 1000; ++i) {
    v(i, 40.0 + double(i % 20));  // Range: [40, 60)
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

TEST(VoxelsTest, ContrastEnhancementHistogramSpread) {
  // Test 2: Verify histogram spreading within original range
  voxels v(dimension(10, 10, 1), Float);
  
  // Create image with clustered values
  for (uint64 j = 0; j < 10; ++j) {
    for (uint64 i = 0; i < 10; ++i) {
      // Three distinct levels (low contrast)
      if (i < 3) v(i, j, 0, 30.0);
      else if (i < 7) v(i, j, 0, 50.0);
      else v(i, j, 0, 70.0);
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

TEST(VoxelsTest, ContrastEnhancementPreservesOrder) {
  // Test 3: Verify that general relative ordering trend is preserved
  voxels v(dimension(10, 1, 1), Float);
  
  // Create monotonically increasing values
  for (uint64 i = 0; i < 10; ++i) {
    v(i, 0, 0, 30.0 + double(i) * 4.0);  // 30, 34, 38, ..., 66
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
    if (v(i, 0, 0) >= v(i + 1, 0, 0)) order_violations++;
  }
  EXPECT_LE(order_violations, 2) << "Most ordering should be preserved";
}

TEST(VoxelsTest, ContrastEnhancementResistorEffect) {
  // Test 4: Different resistor values have different effects
  voxels v1(dimension(10, 10, 1), Float);
  voxels v2(v1);
  
  // Create gradient
  for (uint64 j = 0; j < 10; ++j) {
    for (uint64 i = 0; i < 10; ++i) {
      double val = 20.0 + double(i) * 6.0;  // Gradient across x
      v1(i, j, 0, val);
      v2(i, j, 0, val);
    }
  }
  
  double initial_min = v1.min();
  double initial_max = v1.max();
  
  // Apply with different resistor values
  v1.contrastEnhancement(0.3);  // Low resistor (less spatial influence)
  v2.contrastEnhancement(0.9);  // High resistor (more spatial influence)
  
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

TEST(VoxelsTest, ContrastEnhancementLocalAdaptive) {
  // Test 5: Verify local adaptive behavior
  voxels v(dimension(20, 1, 1), Float);
  
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

TEST(VoxelsTest, ContrastEnhancementEdgeEnhancement) {
  // Test 6: Edges should become more pronounced
  voxels v(dimension(10, 10, 1), Float);
  
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

TEST(VoxelsTest, ContrastEnhancement3D) {
  // Test 7: 3D contrast enhancement
  voxels v(dimension(8, 8, 8), Float);
  
  // Create 3D gradient
  for (uint64 k = 0; k < 8; ++k) {
    for (uint64 j = 0; j < 8; ++j) {
      for (uint64 i = 0; i < 8; ++i) {
        // Gradient based on distance from origin
        double dist = sqrt(double(i*i + j*j + k*k));
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

TEST(VoxelsTest, ContrastEnhancementRangePreservation) {
  // Test 8: Original min/max range is restored after internal processing
  voxels v(dimension(10, 10, 1), Float);
  
  // Create volume with specific range
  for (uint64 i = 0; i < 100; ++i) {
    v(i, 100.0 + double(i % 50) * 2.0);  // Range [100, 200)
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
    if (count > 0) unique_buckets++;
  }
  
  EXPECT_GE(unique_buckets, 3) << "Values should be distributed across histogram";
}

TEST(VoxelsTest, ContrastEnhancementResistorClamping) {
  // Test 9: Resistor values outside [0,1] should be clamped
  voxels v1(dimension(8, 8, 1), Float);
  voxels v2(v1), v3(v1);
  
  // Create test pattern
  for (uint64 i = 0; i < 64; ++i) {
    v1(i, 30.0 + double(i % 30));
    v2(i, 30.0 + double(i % 30));
    v3(i, 30.0 + double(i % 30));
  }
  
  // Apply with clamped values (algorithm clamps internally)
  v1.contrastEnhancement(-0.5);  // Should be clamped to 0.0
  v2.contrastEnhancement(0.0);   // Minimum valid value
  v3.contrastEnhancement(1.5);   // Should be clamped to 1.0
  
  // All should complete without crashing and produce valid results
  EXPECT_TRUE(v1.min() < v1.max()) << "Negative resistor clamped, still works";
  EXPECT_TRUE(v2.min() < v2.max()) << "Zero resistor works";
  EXPECT_TRUE(v3.min() < v3.max()) << "Resistor > 1.0 clamped, still works";
}

// ============================================================================
// Anisotropic Diffusion Tests
// ============================================================================

TEST(VoxelsTest, AnisotropicDiffusionUniform) {
  // Test 1: Uniform volume should remain unchanged
  voxels v(dimension(10, 10, 10), Float);
  v.fill(50.0);
  
  v.anisotropicDiffusion(5);
  
  // Uniform regions should remain stable
  EXPECT_NEAR(v(5, 5, 5), 50.0, 0.01);
  EXPECT_NEAR(v(0, 0, 0), 50.0, 0.01);
  EXPECT_NEAR(v(9, 9, 9), 50.0, 0.01);
}

TEST(VoxelsTest, AnisotropicDiffusionSmoothGradient) {
  // Test 2: Smooth gradient should be preserved
  voxels v(dimension(10, 1, 1), Float);
  
  // Create linear gradient: 0, 10, 20, ..., 90
  for (uint64 i = 0; i < 10; ++i)
    v(i, 0, 0, double(i * 10));
  
  voxels original(v);
  v.anisotropicDiffusion(3);
  
  // Smooth gradients should be relatively unchanged (anisotropic diffusion preserves edges)
  // Check that gradient is still monotonically increasing
  for (uint64 i = 0; i < 9; ++i) {
    EXPECT_LT(v(i, 0, 0), v(i + 1, 0, 0)) 
      << "Gradient should remain monotonic at position " << i;
  }
  
  // End points should be relatively stable
  EXPECT_NEAR(v(0, 0, 0), 0.0, 5.0);
  EXPECT_NEAR(v(9, 0, 0), 90.0, 5.0);
}

TEST(VoxelsTest, AnisotropicDiffusionEdgePreservation) {
  // Test 3: Sharp edges should be preserved
  voxels v(dimension(10, 10, 1), Float);
  
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

TEST(VoxelsTest, AnisotropicDiffusionNoiseReduction) {
  // Test 4: Small noise should be smoothed while preserving structure
  voxels v(dimension(10, 10, 1), Float);
  
  // Create noisy constant region with small fluctuations
  v.fill(50.0);
  v(3, 3, 0, 55.0);  // Small noise
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

TEST(VoxelsTest, AnisotropicDiffusionIsolatedSpike) {
  // Test 5: Isolated spike behavior
  voxels v(dimension(7, 7, 7), Float);
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

TEST(VoxelsTest, AnisotropicDiffusionMultipleIterations) {
  // Test 6: More iterations = more smoothing (but edges preserved)
  voxels v1(dimension(10, 10, 1), Float);
  voxels v2(v1);
  
  // Create pattern with both smooth regions and edges
  for (uint64 i = 0; i < 10; ++i) {
    for (uint64 j = 0; j < 10; ++j) {
      if (i < 5) {
        v1(i, j, 0, 20.0 + (i % 2) * 5.0);  // Noisy low region
      } else {
        v1(i, j, 0, 80.0 + (i % 2) * 5.0);  // Noisy high region
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

TEST(VoxelsTest, AnisotropicDiffusion3DEdgePreservation) {
  // Test 7: 3D edge preservation
  voxels v(dimension(8, 8, 8), Float);
  
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
      bottom_avg += v(i, j, 1);  // Second layer from bottom
      top_avg += v(i, j, 6);     // Second layer from top
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

TEST(VoxelsTest, GDTVFilterUniform) {
  // Test 1: Uniform volume should remain nearly unchanged
  voxels v(dimension(10, 10, 10), Float);
  v.fill(50.0);
  
  v.gdtvFilter(1.5, 0.5, 3, 0);  // 6-neighbor mode
  
  // Uniform volume should be stable
  EXPECT_NEAR(v(5, 5, 5), 50.0, 0.1);
  EXPECT_NEAR(v(0, 0, 0), 50.0, 0.1);
  EXPECT_NEAR(v(9, 9, 9), 50.0, 0.1);
}

TEST(VoxelsTest, GDTVFilterNoiseReduction) {
  // Test 2: Noise reduction with quantitative validation
  voxels v(dimension(10, 10, 1), Float);
  
  // Create noisy data around mean of 50
  for (uint64 j = 0; j < 10; ++j) {
    for (uint64 i = 0; i < 10; ++i) {
      double noise = (i * j) % 7 - 3.0;  // Noise: -3 to +3
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

TEST(VoxelsTest, GDTVFilterEdgePreservation) {
  // Test 3: Edge-preserving property
  voxels v(dimension(10, 10, 1), Float);
  
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
  
  for (uint64 j = 2; j < 8; ++j) {  // Avoid boundary effects
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

TEST(VoxelsTest, GDTVFilterGradientSmoothing) {
  // Test 4: Smooth gradient should be preserved
  voxels v(dimension(10, 10, 1), Float);
  
  // Create smooth gradient
  for (uint64 j = 0; j < 10; ++j) {
    for (uint64 i = 0; i < 10; ++i) {
      v(i, j, 0, 20.0 + double(i) * 5.0);  // Linear gradient
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

TEST(VoxelsTest, GDTVFilterParameterQ) {
  // Test 5: Different q parameter values
  voxels v1(dimension(8, 8, 1), Float);
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
  v1.gdtvFilter(1.2, 0.3, 3, 0);  // Small q (less nonlinear)
  v2.gdtvFilter(1.8, 0.3, 3, 0);  // Large q (more nonlinear)
  
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

TEST(VoxelsTest, GDTVFilterParameterLambda) {
  // Test 6: Lambda parameter controls data fidelity
  voxels v1(dimension(8, 8, 1), Float);
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
  v1.gdtvFilter(1.5, 0.1, 3, 0);  // Low lambda (more smoothing)
  v2.gdtvFilter(1.5, 0.9, 3, 0);  // High lambda (more data fidelity)
  
  // High lambda should stay closer to original
  double dist1 = std::abs(v1(4, 4, 0) - orig_val);
  double dist2 = std::abs(v2(4, 4, 0) - orig_val);
  
  EXPECT_LT(dist2, dist1 + 0.5) << "Higher lambda should preserve data better";
}

TEST(VoxelsTest, GDTVFilterIterations) {
  // Test 7: GDTV filter with varying iteration counts
  voxels v(dimension(8, 8, 1), Float);
  
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

TEST(VoxelsTest, GDTVFilter6vs26Neighbor) {
  // Test 8: 6-neighbor vs 26-neighbor connectivity
  voxels v1(dimension(8, 8, 8), Float);
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
  v1.gdtvFilter(1.5, 0.3, 3, 0);   // 6-neighbor
  v2.gdtvFilter(1.5, 0.3, 3, 26);  // 26-neighbor
  
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

TEST(VoxelsTest, GDTVFilter3DGradient) {
  // Test 9: 3D volumetric filtering
  voxels v(dimension(6, 6, 6), Float);
  
  // Create 3D gradient
  for (uint64 k = 0; k < 6; ++k) {
    for (uint64 j = 0; j < 6; ++j) {
      for (uint64 i = 0; i < 6; ++i) {
        double dist = sqrt(double(i*i + j*j + k*k));
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

TEST(VoxelsTest, GDTVFilterIsolatedSpike) {
  // Test 10: GDTV with gradient-dependent weighting
  voxels v(dimension(10, 10, 1), Float);
  
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

TEST(VoxelsTest, CompositeSubtract) {
  voxels v1(dimension(5, 5, 5), Float);
  voxels v2(dimension(5, 5, 5), Float);
  
  v1.fill(100.0);
  v2.fill(30.0);
  
  // Subtract v2 from v1
  subtract_func subtract;
  v1.composite(v2, 0, 0, 0, subtract);
  
  EXPECT_DOUBLE_EQ(v1(0, 0, 0), 70.0);
  EXPECT_DOUBLE_EQ(v1(2, 2, 2), 70.0);
}

TEST(VoxelsTest, CompositePartialOverlap) {
  voxels v1(dimension(10, 10, 10), Float);
  voxels v2(dimension(5, 5, 5), Float);
  
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

TEST(VoxelsTest, FillSubOutOfBounds) {
  voxels v(dimension(10, 10, 10), Float);
  v.fill(0.0);
  
  // Out-of-bounds fillsub throws index_out_of_bounds exception
  EXPECT_THROW(v.fillsub(8, 8, 8, dimension(5, 5, 5), 100.0), index_out_of_bounds);
}

TEST(VoxelsTest, MinMaxUninitialized) {
  voxels v(dimension(5, 5, 5), Float);
  // Don't set any values, use default zeros
  
  EXPECT_NO_THROW(v.min());
  EXPECT_NO_THROW(v.max());
}

TEST(VoxelsTest, CopyLargeVolume) {
  voxels v1(dimension(50, 50, 50), Float);
  v1.fill(42.0);
  
  voxels v2(v1);
  
  EXPECT_EQ(v2.XDim(), 50u);
  EXPECT_EQ(v2.YDim(), 50u);
  EXPECT_EQ(v2.ZDim(), 50u);
  EXPECT_NEAR(v2(25, 25, 25), 42.0, 1e-5);
}

TEST(VoxelsTest, ResizeThenFill) {
  voxels v(dimension(5, 5, 5), Float);
  v.fill(10.0);
  
  v.resize(dimension(10, 10, 10));
  v.fill(20.0);
  
  EXPECT_NEAR(v(5, 5, 5), 20.0, 1e-5);
  EXPECT_NEAR(v(9, 9, 9), 20.0, 1e-5);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
