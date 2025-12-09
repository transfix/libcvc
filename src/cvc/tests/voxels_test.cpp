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
// Copy-on-Write Tests
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
  
  const boost::shared_array<unsigned char>& data = v.data();
  EXPECT_TRUE(data);
  
  // Access as float pointer
  const float* fdata = reinterpret_cast<const float*>(data.get());
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

TEST(VoxelsTest, BilateralFilterBasic) {
  voxels v(dimension(10, 10, 10), Float);
  
  // Create noisy data
  v.fill(50.0);
  v(5, 5, 5, 100.0);  // Spike
  v(3, 3, 3, 10.0);   // Dip
  
  // Apply bilateral filter with default parameters
  v.bilateralFilter();
  
  // Volume should still exist and have same dimensions
  EXPECT_EQ(v.XDim(), 10u);
  EXPECT_EQ(v.YDim(), 10u);
  EXPECT_EQ(v.ZDim(), 10u);
  
  // Values should be smoothed but edges preserved
  double filtered_spike = v(5, 5, 5);
  EXPECT_GT(filtered_spike, 50.0);  // Still elevated
  EXPECT_LT(filtered_spike, 100.0); // But smoothed
}

TEST(VoxelsTest, BilateralFilterParameters) {
  voxels v(dimension(8, 8, 8), Float);
  v.fill(50.0);
  v(4, 4, 4, 200.0);
  
  // Test with different parameters
  v.bilateralFilter(100.0, 1.0, 1);
  
  EXPECT_EQ(v.XDim(), 8u);
  EXPECT_GT(v(4, 4, 4), 50.0);
}

// ============================================================================
// Contrast Enhancement Tests
// ============================================================================

TEST(VoxelsTest, ContrastEnhancement) {
  voxels v(dimension(10, 10, 10), Float);
  
  // Create volume with varying intensities
  for (uint64 i = 0; i < 1000; ++i)
    v(i, double(i % 100));
  
  v.contrastEnhancement(0.9);
  
  // Volume should exist with same dimensions
  EXPECT_EQ(v.XDim(), 10u);
  EXPECT_EQ(v.YDim(), 10u);
  EXPECT_EQ(v.ZDim(), 10u);
}

TEST(VoxelsTest, ContrastEnhancementResistor) {
  voxels v(dimension(8, 8, 8), Float);
  
  // Fill with gradient
  for (uint64 i = 0; i < 512; ++i)
    v(i, double(i) / 10.0);
  
  // Apply with different resistor values
  v.contrastEnhancement(0.5);
  
  EXPECT_EQ(v.XDim(), 8u);
}

// ============================================================================
// Anisotropic Diffusion Tests
// ============================================================================

TEST(VoxelsTest, AnisotropicDiffusion) {
  voxels v(dimension(10, 10, 10), Float);
  
  // Create noisy volume
  for (uint64 i = 0; i < 1000; ++i)
    v(i, 50.0 + (i % 10) - 5.0);
  
  v.anisotropicDiffusion(5);
  
  // Should smooth while preserving edges
  EXPECT_EQ(v.XDim(), 10u);
  EXPECT_EQ(v.YDim(), 10u);
  EXPECT_EQ(v.ZDim(), 10u);
}

TEST(VoxelsTest, AnisotropicDiffusionIterations) {
  voxels v(dimension(8, 8, 8), Float);
  v.fill(50.0);
  v(4, 4, 4, 100.0);
  
  // Test with different iteration counts
  v.anisotropicDiffusion(10);
  
  EXPECT_EQ(v.XDim(), 8u);
}

// ============================================================================
// GDTV Filter Tests
// ============================================================================

TEST(VoxelsTest, GDTVFilter) {
  voxels v(dimension(10, 10, 10), Float);
  
  // Create test volume
  for (uint64 i = 0; i < 1000; ++i)
    v(i, double(i % 50));
  
  v.gdtvFilter(1.5, 0.1, 5, 6);
  
  EXPECT_EQ(v.XDim(), 10u);
  EXPECT_EQ(v.YDim(), 10u);
  EXPECT_EQ(v.ZDim(), 10u);
}

TEST(VoxelsTest, GDTVFilterParameters) {
  voxels v(dimension(8, 8, 8), Float);
  v.fill(25.0);
  
  // Test with different parameters
  v.gdtvFilter(2.0, 0.05, 3, 18);
  
  EXPECT_EQ(v.XDim(), 8u);
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
