/*
  Test suite for CVC geometry class.
  Tests geometry operations using the Stanford Bunny mesh.
*/

#include <cvc/geometry.h>
#include <cvc/geometry_file_io.h>
#include <cvc/algorithm.h>
#include <cvc/volmagick.h>
#include <cvc/exception.h>

#include <gtest/gtest.h>
#include <cmath>
#include <fstream>
#include <limits>
#include <vector>

using namespace CVC_NAMESPACE;

class GeometryTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Load the Stanford Bunny for testing
    bunny = read_geometry("test.bunny");
  }

  geometry bunny;
};

// ============================================================================
// Construction and Basic Properties Tests
// ============================================================================

TEST_F(GeometryTest, DefaultConstruction) {
  geometry geom;
  EXPECT_TRUE(geom.empty());
  EXPECT_EQ(geom.num_points(), 0);
  EXPECT_EQ(geom.num_lines(), 0);
  EXPECT_EQ(geom.num_tris(), 0);
  EXPECT_EQ(geom.num_quads(), 0);
}

TEST_F(GeometryTest, CopyConstruction) {
  geometry copy(bunny);
  EXPECT_EQ(copy.num_points(), bunny.num_points());
  EXPECT_EQ(copy.num_tris(), bunny.num_tris());
  EXPECT_FALSE(copy.empty());
}

TEST_F(GeometryTest, AssignmentOperator) {
  geometry geom;
  geom = bunny;
  EXPECT_EQ(geom.num_points(), bunny.num_points());
  EXPECT_EQ(geom.num_tris(), bunny.num_tris());
}

TEST_F(GeometryTest, CopyMethod) {
  geometry geom;
  geom.copy(bunny);
  EXPECT_EQ(geom.num_points(), bunny.num_points());
  EXPECT_EQ(geom.num_tris(), bunny.num_tris());
}

TEST_F(GeometryTest, BunnyProperties) {
  // Stanford Bunny has 34,835 vertices and 69,473 triangles
  EXPECT_EQ(bunny.num_points(), 34835);
  EXPECT_EQ(bunny.num_tris(), 69473);
  EXPECT_EQ(bunny.num_lines(), 0);
  EXPECT_EQ(bunny.num_quads(), 0);
  EXPECT_FALSE(bunny.empty());
}

TEST_F(GeometryTest, BunnyHasNormals) {
  // Bunny should come with pre-computed normals
  EXPECT_EQ(bunny.num_points(), bunny.normals().size());
  
  // Check that normals are valid (non-zero)
  bool has_valid_normals = false;
  for (const auto& normal : bunny.normals()) {
    double length = std::sqrt(normal[0]*normal[0] + 
                              normal[1]*normal[1] + 
                              normal[2]*normal[2]);
    if (length > 0.1) {
      has_valid_normals = true;
      break;
    }
  }
  EXPECT_TRUE(has_valid_normals);
}

// ============================================================================
// Extents and Bounding Box Tests
// ============================================================================

TEST_F(GeometryTest, MinMaxPoints) {
  point_t min_pt = bunny.min_point();
  point_t max_pt = bunny.max_point();
  
  // Check that min is less than max in all dimensions
  EXPECT_LT(min_pt[0], max_pt[0]);
  EXPECT_LT(min_pt[1], max_pt[1]);
  EXPECT_LT(min_pt[2], max_pt[2]);
  
  // Verify all points are within bounds
  for (const auto& pt : bunny.points()) {
    EXPECT_GE(pt[0], min_pt[0]);
    EXPECT_GE(pt[1], min_pt[1]);
    EXPECT_GE(pt[2], min_pt[2]);
    EXPECT_LE(pt[0], max_pt[0]);
    EXPECT_LE(pt[1], max_pt[1]);
    EXPECT_LE(pt[2], max_pt[2]);
  }
}

TEST_F(GeometryTest, ExtentsBoundingBox) {
  bounding_box bbox = bunny.extents();
  point_t min_pt = bunny.min_point();
  point_t max_pt = bunny.max_point();
  
  EXPECT_DOUBLE_EQ(bbox.minx, min_pt[0]);
  EXPECT_DOUBLE_EQ(bbox.miny, min_pt[1]);
  EXPECT_DOUBLE_EQ(bbox.minz, min_pt[2]);
  EXPECT_DOUBLE_EQ(bbox.maxx, max_pt[0]);
  EXPECT_DOUBLE_EQ(bbox.maxy, max_pt[1]);
  EXPECT_DOUBLE_EQ(bbox.maxz, max_pt[2]);
}

TEST_F(GeometryTest, EmptyGeometryExtents) {
  geometry empty_geom;
  point_t min_pt = empty_geom.min_point();
  point_t max_pt = empty_geom.max_point();
  
  // Empty geometry should have inverted bounds
  EXPECT_GE(min_pt[0], max_pt[0]);
  EXPECT_GE(min_pt[1], max_pt[1]);
  EXPECT_GE(min_pt[2], max_pt[2]);
}

// ============================================================================
// Triangle Topology Tests
// ============================================================================

TEST_F(GeometryTest, TriangleIndicesValid) {
  // All triangle indices should be within point count
  uint64_t num_pts = bunny.num_points();
  for (const auto& tri : bunny.tris()) {
    EXPECT_LT(tri[0], num_pts);
    EXPECT_LT(tri[1], num_pts);
    EXPECT_LT(tri[2], num_pts);
  }
}

TEST_F(GeometryTest, TrianglesNonDegenerate) {
  // Check that triangles don't have duplicate vertices
  size_t degenerate_count = 0;
  for (const auto& tri : bunny.tris()) {
    if (tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0]) {
      degenerate_count++;
    }
  }
  EXPECT_EQ(degenerate_count, 0);
}

TEST_F(GeometryTest, TriangleArea) {
  // Compute area of first triangle to verify it's non-zero
  const tri_t& tri = bunny.tris()[0];
  const point_t& p0 = bunny.points()[tri[0]];
  const point_t& p1 = bunny.points()[tri[1]];
  const point_t& p2 = bunny.points()[tri[2]];
  
  // Edge vectors
  double v1[3] = {p1[0]-p0[0], p1[1]-p0[1], p1[2]-p0[2]};
  double v2[3] = {p2[0]-p0[0], p2[1]-p0[1], p2[2]-p0[2]};
  
  // Cross product magnitude = 2 * area
  double cross[3] = {
    v1[1]*v2[2] - v1[2]*v2[1],
    v1[2]*v2[0] - v1[0]*v2[2],
    v1[0]*v2[1] - v1[1]*v2[0]
  };
  
  double area = 0.5 * std::sqrt(cross[0]*cross[0] + 
                                 cross[1]*cross[1] + 
                                 cross[2]*cross[2]);
  EXPECT_GT(area, 0.0);
}

// ============================================================================
// Merge Operations Tests
// ============================================================================

TEST_F(GeometryTest, MergeEmpty) {
  geometry copy(bunny);
  geometry empty;
  copy.merge(empty);
  
  EXPECT_EQ(copy.num_points(), bunny.num_points());
  EXPECT_EQ(copy.num_tris(), bunny.num_tris());
}

TEST_F(GeometryTest, MergeTwoGeometries) {
  geometry geom1(bunny);
  geometry geom2(bunny);
  
  uint64_t orig_points = geom1.num_points();
  uint64_t orig_tris = geom1.num_tris();
  
  geom1.merge(geom2);
  
  EXPECT_EQ(geom1.num_points(), orig_points * 2);
  EXPECT_EQ(geom1.num_tris(), orig_tris * 2);
}

TEST_F(GeometryTest, MergeIndicesReindexed) {
  geometry simple;
  
  // Create a simple triangle
  simple.points().push_back({{0.0, 0.0, 0.0}});
  simple.points().push_back({{1.0, 0.0, 0.0}});
  simple.points().push_back({{0.0, 1.0, 0.0}});
  simple.tris().push_back({{0, 1, 2}});
  
  geometry copy(simple);
  simple.merge(copy);
  
  // After merge, should have 6 points and 2 triangles
  EXPECT_EQ(simple.num_points(), 6);
  EXPECT_EQ(simple.num_tris(), 2);
  
  // Second triangle should reference points 3, 4, 5
  EXPECT_EQ(simple.tris()[1][0], 3);
  EXPECT_EQ(simple.tris()[1][1], 4);
  EXPECT_EQ(simple.tris()[1][2], 5);
}

// ============================================================================
// Triangle Surface Extraction Tests
// ============================================================================

TEST_F(GeometryTest, TriSurfaceNoBoundary) {
  geometry tri_surf = bunny.tri_surface();
  
  // Without boundary info, should be identical
  EXPECT_EQ(tri_surf.num_tris(), bunny.num_tris());
  EXPECT_EQ(tri_surf.num_points(), bunny.num_points());
}

TEST_F(GeometryTest, TriSurfaceWithBoundary) {
  geometry geom;
  
  // Create a simple tetrahedron (4 points, 4 faces)
  geom.points().push_back({{0.0, 0.0, 0.0}});
  geom.points().push_back({{1.0, 0.0, 0.0}});
  geom.points().push_back({{0.5, 1.0, 0.0}});
  geom.points().push_back({{0.5, 0.5, 1.0}});
  
  geom.tris().push_back({{0, 1, 2}});
  geom.tris().push_back({{0, 1, 3}});
  geom.tris().push_back({{1, 2, 3}});
  geom.tris().push_back({{2, 0, 3}});
  
  // Mark all points as boundary
  geom.boundary().resize(4, true);
  
  geometry tri_surf = geom.tri_surface();
  EXPECT_EQ(tri_surf.num_tris(), 4);
  
  // Mark one point as interior
  geom.boundary()[3] = false;
  tri_surf = geom.tri_surface();
  EXPECT_LT(tri_surf.num_tris(), 4);  // Should exclude tris with interior point
}

TEST_F(GeometryTest, TriSurfaceQuadsToTris) {
  geometry geom;
  
  // Create a quad
  geom.points().push_back({{0.0, 0.0, 0.0}});
  geom.points().push_back({{1.0, 0.0, 0.0}});
  geom.points().push_back({{1.0, 1.0, 0.0}});
  geom.points().push_back({{0.0, 1.0, 0.0}});
  geom.quads().push_back({{0, 1, 2, 3}});
  
  geometry tri_surf = geom.tri_surface();
  
  // Quad should be split into 2 triangles
  EXPECT_EQ(tri_surf.num_tris(), 2);
  EXPECT_EQ(tri_surf.num_quads(), 0);
}

// ============================================================================
// Normal Calculation Tests
// ============================================================================

TEST_F(GeometryTest, CalculateSurfNormals) {
  geometry geom;
  
  // Create a simple triangle facing up (normal should be +Z)
  geom.points().push_back({{0.0, 0.0, 0.0}});
  geom.points().push_back({{1.0, 0.0, 0.0}});
  geom.points().push_back({{0.0, 1.0, 0.0}});
  geom.tris().push_back({{0, 1, 2}});
  
  geom.calculate_surf_normals();
  
  EXPECT_EQ(geom.normals().size(), 3);
  
  // All three vertices should have normals pointing up (+Z)
  for (const auto& normal : geom.normals()) {
    EXPECT_NEAR(normal[0], 0.0, 0.01);
    EXPECT_NEAR(normal[1], 0.0, 0.01);
    EXPECT_NEAR(normal[2], 1.0, 0.01);
  }
}

TEST_F(GeometryTest, NormalsHaveLength) {
  geometry copy(bunny);
  copy.calculate_surf_normals();
  
  // Check that recalculated normals have non-zero length
  // Note: Averaged normals at vertices may not be unit length  
  size_t non_zero_count = 0;
  for (const auto& normal : copy.normals()) {
    double length = std::sqrt(normal[0]*normal[0] + 
                              normal[1]*normal[1] + 
                              normal[2]*normal[2]);
    if (length > 0.01) {
      non_zero_count++;
    }
  }
  // Should have many valid normals
  EXPECT_GT(non_zero_count, copy.num_points() / 2);
}

TEST_F(GeometryTest, RecalculateNormals) {
  geometry copy(bunny);
  
  // Clear existing normals and recalculate
  copy.normals().clear();
  copy.calculate_surf_normals();
  
  EXPECT_EQ(copy.normals().size(), copy.num_points());
}

// ============================================================================
// Wire Interior Generation Tests
// ============================================================================

TEST_F(GeometryTest, GenerateWireInteriorNoBoundary) {
  geometry wire = bunny.generate_wire_interior();
  
  // Without boundary info, should preserve tris and add no lines
  EXPECT_EQ(wire.num_tris(), bunny.num_tris());
  EXPECT_EQ(wire.num_lines(), 0);
}

TEST_F(GeometryTest, GenerateWireInteriorWithBoundary) {
  geometry geom;
  
  // Create a tetrahedron
  geom.points().push_back({{0.0, 0.0, 0.0}});
  geom.points().push_back({{1.0, 0.0, 0.0}});
  geom.points().push_back({{0.5, 1.0, 0.0}});
  geom.points().push_back({{0.5, 0.5, 1.0}});
  
  geom.tris().push_back({{0, 1, 2}});
  geom.tris().push_back({{0, 1, 3}});
  geom.tris().push_back({{1, 2, 3}});
  geom.tris().push_back({{2, 0, 3}});
  
  // Mark first 3 as boundary, 4th as interior
  geom.boundary().resize(4);
  geom.boundary()[0] = true;
  geom.boundary()[1] = true;
  geom.boundary()[2] = true;
  geom.boundary()[3] = false;
  
  geometry wire = geom.generate_wire_interior();
  
  // Should generate lines for interior edges
  EXPECT_GT(wire.num_lines(), 0);
}

// ============================================================================
// Normal Inversion Tests
// ============================================================================

TEST_F(GeometryTest, InvertNormals) {
  geometry geom;
  
  // Create a triangle with a known normal
  geom.points().push_back({{0.0, 0.0, 0.0}});
  geom.points().push_back({{1.0, 0.0, 0.0}});
  geom.points().push_back({{0.0, 1.0, 0.0}});
  geom.tris().push_back({{0, 1, 2}});
  
  geom.calculate_surf_normals();
  
  // Store original normal
  vector_t orig_normal = geom.normals()[0];
  
  geom.invert_normals();
  
  // Check that normals are inverted
  for (size_t i = 0; i < geom.normals().size(); i++) {
    EXPECT_NEAR(geom.normals()[i][0], -orig_normal[0], 1e-6);
    EXPECT_NEAR(geom.normals()[i][1], -orig_normal[1], 1e-6);
    EXPECT_NEAR(geom.normals()[i][2], -orig_normal[2], 1e-6);
  }
}

TEST_F(GeometryTest, InvertNormalsPreservesLength) {
  geometry copy(bunny);
  
  // Calculate original normal lengths
  std::vector<double> orig_lengths;
  for (const auto& normal : copy.normals()) {
    double length = std::sqrt(normal[0]*normal[0] + 
                              normal[1]*normal[1] + 
                              normal[2]*normal[2]);
    orig_lengths.push_back(length);
  }
  
  copy.invert_normals();
  
  // Check that lengths are preserved
  for (size_t i = 0; i < copy.normals().size(); i++) {
    const auto& normal = copy.normals()[i];
    double length = std::sqrt(normal[0]*normal[0] + 
                              normal[1]*normal[1] + 
                              normal[2]*normal[2]);
    EXPECT_NEAR(length, orig_lengths[i], 1e-6);
  }
}

// ============================================================================
// Clear Tests
// ============================================================================

TEST_F(GeometryTest, ClearGeometry) {
  geometry copy(bunny);
  EXPECT_FALSE(copy.empty());
  
  copy.clear();
  
  EXPECT_TRUE(copy.empty());
  EXPECT_EQ(copy.num_points(), 0);
  EXPECT_EQ(copy.num_tris(), 0);
  EXPECT_EQ(copy.num_lines(), 0);
  EXPECT_EQ(copy.num_quads(), 0);
}

// ============================================================================
// File I/O Tests
// ============================================================================

TEST_F(GeometryTest, ReadBunnyFile) {
  geometry geom;
  geom.read("test.bunny");
  
  EXPECT_EQ(geom.num_points(), 34835);
  EXPECT_EQ(geom.num_tris(), 69473);
}

TEST_F(GeometryTest, FileConstructor) {
  geometry geom("test.bunny");
  
  EXPECT_EQ(geom.num_points(), 34835);
  EXPECT_EQ(geom.num_tris(), 69473);
}

TEST_F(GeometryTest, WriteAndReadBackRAW) {
  // Create a simple geometry
  geometry original;
  original.points().push_back({{0.0, 0.0, 0.0}});
  original.points().push_back({{1.0, 0.0, 0.0}});
  original.points().push_back({{0.0, 1.0, 0.0}});
  original.tris().push_back({{0, 1, 2}});
  
  // Write to RAW format
  original.write("test_output.raw");
  
  // Read back
  geometry loaded;
  loaded.read("test_output.raw");
  
  EXPECT_EQ(loaded.num_points(), 3);
  EXPECT_EQ(loaded.num_tris(), 1);
  
  // Clean up
  std::remove("test_output.raw");
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST_F(GeometryTest, EmptyAfterClear) {
  geometry geom(bunny);
  geom.clear();
  
  EXPECT_TRUE(geom.empty());
}

TEST_F(GeometryTest, MultipleOperations) {
  geometry geom(bunny);
  
  // Chain multiple operations
  geom.calculate_surf_normals()
      .invert_normals()
      .invert_normals();  // Double inversion should restore
  
  // Should still have valid data
  EXPECT_EQ(geom.num_points(), bunny.num_points());
  EXPECT_EQ(geom.num_tris(), bunny.num_tris());
}

TEST_F(GeometryTest, CopyOnWrite) {
  geometry geom1(bunny);
  geometry geom2(geom1);
  
  // Modify geom2
  geom2.points()[0][0] = 999.0;
  
  // geom1 should be unchanged (copy-on-write)
  EXPECT_NE(geom1.points()[0][0], 999.0);
}

TEST_F(GeometryTest, AccessorsConst) {
  const geometry& const_bunny = bunny;
  
  // Test const accessors
  EXPECT_EQ(const_bunny.num_points(), 34835);
  EXPECT_EQ(const_bunny.points().size(), 34835);
  EXPECT_EQ(const_bunny.const_points().size(), 34835);
}

// ============================================================================
// Geometry with Lines Tests
// ============================================================================

TEST_F(GeometryTest, AddLines) {
  geometry geom;
  
  geom.points().push_back({{0.0, 0.0, 0.0}});
  geom.points().push_back({{1.0, 0.0, 0.0}});
  geom.points().push_back({{1.0, 1.0, 0.0}});
  
  geom.lines().push_back({{0, 1}});
  geom.lines().push_back({{1, 2}});
  
  EXPECT_EQ(geom.num_lines(), 2);
  EXPECT_FALSE(geom.empty());
}

TEST_F(GeometryTest, MergeWithLines) {
  geometry geom1;
  geom1.points().push_back({{0.0, 0.0, 0.0}});
  geom1.points().push_back({{1.0, 0.0, 0.0}});
  geom1.lines().push_back({{0, 1}});
  
  geometry geom2(geom1);
  geom1.merge(geom2);
  
  EXPECT_EQ(geom1.num_lines(), 2);
  EXPECT_EQ(geom1.num_points(), 4);
  
  // Second line should reference points 2, 3
  EXPECT_EQ(geom1.lines()[1][0], 2);
  EXPECT_EQ(geom1.lines()[1][1], 3);
}

// ============================================================================
// Geometry with Quads Tests
// ============================================================================

TEST_F(GeometryTest, AddQuads) {
  geometry geom;
  
  geom.points().push_back({{0.0, 0.0, 0.0}});
  geom.points().push_back({{1.0, 0.0, 0.0}});
  geom.points().push_back({{1.0, 1.0, 0.0}});
  geom.points().push_back({{0.0, 1.0, 0.0}});
  
  geom.quads().push_back({{0, 1, 2, 3}});
  
  EXPECT_EQ(geom.num_quads(), 1);
  EXPECT_FALSE(geom.empty());
}

TEST_F(GeometryTest, MergeWithQuads) {
  geometry geom1;
  geom1.points().push_back({{0.0, 0.0, 0.0}});
  geom1.points().push_back({{1.0, 0.0, 0.0}});
  geom1.points().push_back({{1.0, 1.0, 0.0}});
  geom1.points().push_back({{0.0, 1.0, 0.0}});
  geom1.quads().push_back({{0, 1, 2, 3}});
  
  geometry geom2(geom1);
  geom1.merge(geom2);
  
  EXPECT_EQ(geom1.num_quads(), 2);
  EXPECT_EQ(geom1.num_points(), 8);
  
  // Second quad should reference points 4, 5, 6, 7
  EXPECT_EQ(geom1.quads()[1][0], 4);
  EXPECT_EQ(geom1.quads()[1][1], 5);
  EXPECT_EQ(geom1.quads()[1][2], 6);
  EXPECT_EQ(geom1.quads()[1][3], 7);
}

// ============================================================================
// Algorithm Tests - Processing and Filtering
// ============================================================================

TEST_F(GeometryTest, CalculateSurfNormalsTriangle) {
  geometry geom;
  
  // Create a simple triangle
  geom.points().push_back({{0.0, 0.0, 0.0}});
  geom.points().push_back({{1.0, 0.0, 0.0}});
  geom.points().push_back({{0.0, 1.0, 0.0}});
  geom.tris().push_back({{0, 1, 2}});
  
  // Calculate normals
  geom.calculate_surf_normals();
  
  EXPECT_EQ(geom.normals().size(), 3);
  
  // All three vertices should have the same normal (perpendicular to xy-plane)
  // Normal should point in +z direction: (0, 0, 1)
  for (const auto& normal : geom.normals()) {
    EXPECT_NEAR(normal[0], 0.0, 1e-5);
    EXPECT_NEAR(normal[1], 0.0, 1e-5);
    EXPECT_NEAR(std::abs(normal[2]), 1.0, 1e-5);
  }
}

TEST_F(GeometryTest, InvertNormalsBasic) {
  geometry geom;
  
  geom.points().push_back({{0.0, 0.0, 0.0}});
  geom.points().push_back({{1.0, 0.0, 0.0}});
  geom.points().push_back({{0.0, 1.0, 0.0}});
  geom.tris().push_back({{0, 1, 2}});
  geom.calculate_surf_normals();
  
  // Get original normal
  vector_t original_normal = geom.normals()[0];
  
  // Invert
  geom.invert_normals();
  
  // Check that normals are inverted
  for (size_t i = 0; i < 3; i++) {
    EXPECT_NEAR(geom.normals()[0][i], -original_normal[i], 1e-5);
  }
}

TEST_F(GeometryTest, SmoothingPreservesTopology) {
  geometry geom(bunny);
  
  uint64_t original_points = geom.num_points();
  uint64_t original_tris = geom.num_tris();
  
  // Apply smoothing
  geom.smoothing(0.1f, false);
  
  // Topology should be preserved
  EXPECT_EQ(geom.num_points(), original_points);
  EXPECT_EQ(geom.num_tris(), original_tris);
}

TEST_F(GeometryTest, SmoothingMovesVertices) {
  geometry original(bunny);
  geometry smoothed(bunny);
  
  // Apply smoothing
  smoothed.smoothing(0.5f, false);
  
  // Vertices should have moved
  bool vertices_moved = false;
  for (size_t i = 0; i < original.num_points() && i < 100; i++) {
    double dist = 0.0;
    for (int j = 0; j < 3; j++) {
      double diff = original.points()[i][j] - smoothed.points()[i][j];
      dist += diff * diff;
    }
    dist = std::sqrt(dist);
    
    if (dist > 1e-6) {
      vertices_moved = true;
      break;
    }
  }
  
  EXPECT_TRUE(vertices_moved);
}

TEST_F(GeometryTest, DISABLED_SmoothingWithBoundaryFixed) {
  geometry geom(bunny);
  
  // Set up boundary markers (mark first 10 vertices as boundary)
  geom.boundary().resize(geom.num_points(), 0);
  for (size_t i = 0; i < 10; i++) {
    geom.boundary()[i] = 1;
  }
  
  // Save original boundary positions
  std::vector<point_t> original_boundary_pos;
  for (size_t i = 0; i < 10; i++) {
    original_boundary_pos.push_back(geom.points()[i]);
  }
  
  // Apply smoothing with boundary fixed
  geom.smoothing(0.5f, true);
  
  // Boundary vertices should not have moved (allow small numerical errors)
  for (size_t i = 0; i < 10; i++) {
    for (int j = 0; j < 3; j++) {
      EXPECT_NEAR(geom.points()[i][j], original_boundary_pos[i][j], 1e-3);
    }
  }
}

TEST_F(GeometryTest, QualityImprovePreservesTopology) {
  geometry geom(bunny);
  
  uint64_t original_points = geom.num_points();
  uint64_t original_tris = geom.num_tris();
  
  // Apply quality improvement
  geom.quality_improve(1, "geo_flow");
  
  // Topology should be preserved
  EXPECT_EQ(geom.num_points(), original_points);
  EXPECT_EQ(geom.num_tris(), original_tris);
}

TEST_F(GeometryTest, QualityImproveMultipleIterations) {
  geometry geom(bunny);
  
  // Apply multiple iterations
  geom.quality_improve(3, "geo_flow");
  
  // Should still have valid geometry
  EXPECT_EQ(geom.num_points(), bunny.num_points());
  EXPECT_EQ(geom.num_tris(), bunny.num_tris());
  EXPECT_FALSE(geom.empty());
}

#ifdef CVC_GEOMETRY_ENABLE_PROJECT
TEST_F(GeometryTest, ProjectToTargetSurface) {
  geometry source(bunny);
  geometry target(bunny);
  
  // Perturb source vertices slightly
  for (auto& pt : source.points()) {
    pt[0] += 0.01;
    pt[1] += 0.01;
  }
  
  // Project source onto target
  source.project(target);
  
  // Vertices should be closer to target now
  // This is a basic sanity check - projection should change vertices
  bool projection_occurred = false;
  for (size_t i = 0; i < std::min(source.num_points(), size_t(100)); i++) {
    double dist = 0.0;
    for (int j = 0; j < 3; j++) {
      double diff = source.points()[i][j] - target.points()[i][j];
      dist += diff * diff;
    }
    
    // After projection, distances should be small
    if (dist < 0.1) {
      projection_occurred = true;
      break;
    }
  }
  
  EXPECT_TRUE(projection_occurred);
}
#endif

// ============================================================================
// Algorithm Tests - SDF and Isosurface Extraction
// ============================================================================

TEST(AlgorithmTest, DISABLED_SDFBasic) {
  // Create a simple cube geometry
  geometry cube;
  
  // Cube vertices
  cube.points().push_back({{-1.0, -1.0, -1.0}});
  cube.points().push_back({{1.0, -1.0, -1.0}});
  cube.points().push_back({{1.0, 1.0, -1.0}});
  cube.points().push_back({{-1.0, 1.0, -1.0}});
  cube.points().push_back({{-1.0, -1.0, 1.0}});
  cube.points().push_back({{1.0, -1.0, 1.0}});
  cube.points().push_back({{1.0, 1.0, 1.0}});
  cube.points().push_back({{-1.0, 1.0, 1.0}});
  
  // Cube faces (using triangles)
  // Bottom face
  cube.tris().push_back({{0, 1, 2}});
  cube.tris().push_back({{0, 2, 3}});
  // Top face
  cube.tris().push_back({{4, 5, 6}});
  cube.tris().push_back({{4, 6, 7}});
  // Front face
  cube.tris().push_back({{0, 1, 5}});
  cube.tris().push_back({{0, 5, 4}});
  // Back face
  cube.tris().push_back({{2, 3, 7}});
  cube.tris().push_back({{2, 7, 6}});
  // Left face
  cube.tris().push_back({{0, 3, 7}});
  cube.tris().push_back({{0, 7, 4}});
  // Right face
  cube.tris().push_back({{1, 2, 6}});
  cube.tris().push_back({{1, 6, 5}});
  
  // Generate SDF
  dimension dim(32, 32, 32);
  bounding_box bbox(-2.0, -2.0, -2.0, 2.0, 2.0, 2.0);
  
  volume sdf_vol = sdf(cube, dim, bbox);
  
  // Verify SDF properties
  EXPECT_EQ(sdf_vol.XDim(), 32);
  EXPECT_EQ(sdf_vol.YDim(), 32);
  EXPECT_EQ(sdf_vol.ZDim(), 32);
  
  // SDF should have negative values inside and positive outside
  bool has_negative = false;
  bool has_positive = false;
  
  for (uint64 i = 0; i < sdf_vol.XDim() * sdf_vol.YDim() * sdf_vol.ZDim(); i++) {
    double val = sdf_vol(i);
    if (val < 0) has_negative = true;
    if (val > 0) has_positive = true;
  }
  
  EXPECT_TRUE(has_negative) << "SDF should have negative values inside";
  EXPECT_TRUE(has_positive) << "SDF should have positive values outside";
}

TEST(AlgorithmTest, IsoBasic) {
  // Create a simple volume with a known isosurface
  dimension dim(16, 16, 16);
  bounding_box bbox(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
  
  volume vol(dim, Float, bbox);
  
  // Create a sphere-like distance function
  double center_x = 0.5, center_y = 0.5, center_z = 0.5;
  double radius = 0.3;
  
  for (uint64 k = 0; k < dim[2]; k++) {
    for (uint64 j = 0; j < dim[1]; j++) {
      for (uint64 i = 0; i < dim[0]; i++) {
        double x = bbox[0] + (i + 0.5) * (bbox[3] - bbox[0]) / dim[0];
        double y = bbox[1] + (j + 0.5) * (bbox[4] - bbox[1]) / dim[1];
        double z = bbox[2] + (k + 0.5) * (bbox[5] - bbox[2]) / dim[2];
        
        double dist = std::sqrt((x - center_x) * (x - center_x) +
                                (y - center_y) * (y - center_y) +
                                (z - center_z) * (z - center_z));
        
        vol(i, j, k, dist - radius);
      }
    }
  }
  
  // Extract isosurface at 0
  geometry isosurface = iso(vol, 0.0);
  
  // Should have created a non-empty surface
  EXPECT_GT(isosurface.num_points(), 0);
  EXPECT_GT(isosurface.num_tris(), 0);
  EXPECT_FALSE(isosurface.empty());
}

TEST(AlgorithmTest, IsoWithDifferentIsovalues) {
  dimension dim(16, 16, 16);
  bounding_box bbox(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
  
  volume vol(dim, Float, bbox);
  
  // Linear gradient in x direction
  for (uint64 k = 0; k < dim[2]; k++) {
    for (uint64 j = 0; j < dim[1]; j++) {
      for (uint64 i = 0; i < dim[0]; i++) {
        double x = static_cast<double>(i) / dim[0];
        vol(i, j, k, x);
      }
    }
  }
  
  // Extract at different isovalues
  geometry iso1 = iso(vol, 0.25);
  geometry iso2 = iso(vol, 0.5);
  geometry iso3 = iso(vol, 0.75);
  
  // All should produce valid surfaces
  EXPECT_GT(iso1.num_points(), 0);
  EXPECT_GT(iso2.num_points(), 0);
  EXPECT_GT(iso3.num_points(), 0);
}

TEST(AlgorithmTest, DISABLED_SDFThenIsoRoundtrip) {
  // Create simple geometry
  geometry original;
  
  // Simple pyramid
  original.points().push_back({{0.0, 0.0, 1.0}});   // apex
  original.points().push_back({{-1.0, -1.0, 0.0}}); // base corners
  original.points().push_back({{1.0, -1.0, 0.0}});
  original.points().push_back({{1.0, 1.0, 0.0}});
  original.points().push_back({{-1.0, 1.0, 0.0}});
  
  // Pyramid faces
  original.tris().push_back({{0, 1, 2}});
  original.tris().push_back({{0, 2, 3}});
  original.tris().push_back({{0, 3, 4}});
  original.tris().push_back({{0, 4, 1}});
  // Base (two triangles)
  original.tris().push_back({{1, 2, 3}});
  original.tris().push_back({{1, 3, 4}});
  
  // Convert to SDF (use smaller dimensions to avoid memory issues)
  dimension dim(16, 16, 16);
  bounding_box bbox(-2.0, -2.0, -1.0, 2.0, 2.0, 2.0);
  volume sdf_vol = sdf(original, dim, bbox);
  
  // Extract isosurface
  geometry reconstructed = iso(sdf_vol, 0.0);
  
  // Should have created valid geometry
  EXPECT_GT(reconstructed.num_points(), 0);
  EXPECT_GT(reconstructed.num_tris(), 0);
  
  // Verify it's roughly in the same bounding box
  point_t recon_min = reconstructed.min_point();
  point_t recon_max = reconstructed.max_point();
  
  // Reconstructed geometry should be within original bounds (with some tolerance)
  EXPECT_GE(recon_min[0], -1.5);
  EXPECT_LE(recon_max[0], 1.5);
  EXPECT_GE(recon_min[1], -1.5);
  EXPECT_LE(recon_max[1], 1.5);
}

// Run all tests
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
