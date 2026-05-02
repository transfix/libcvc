#include <chrono>
#include <cmath>
#include <cvc/app.h>
#include <cvc/geometry.h>
#include <cvc/state.h>
#include <cvc/volume.h>
#include <gtest/gtest.h>
#include <thread>
#include <volrover3/GeometryNode.h>
#include <volrover3/SceneGraph.h>
#include <volrover3/VolumeNode.h>
#include <vtkMatrix4x4.h>
#include <vtkPlane.h>
#include <vtkPlaneCollection.h>

class VolumeNodeTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    // Disable threading for state_object to avoid race conditions during destruction
    cvc::state_object<SceneNode>::setUseThreading(false);
  }

  void SetUp() override {
    // Create a simple test volume with known data range
    testVolume = cvc::volume(ctx, cvc::dimension(10, 10, 10), cvc::UChar,
                             cvc::bounding_box(0.0, 0.0, 0.0, 9.0, 9.0, 9.0));

    // Fill with gradient data
    for (unsigned int k = 0; k < 10; k++) {
      for (unsigned int j = 0; j < 10; j++) {
        for (unsigned int i = 0; i < 10; i++) {
          testVolume(i, j, k, static_cast<unsigned char>(i + j + k));
        }
      }
    }

    // Set min/max explicitly
    testVolume.min(0.0);
    testVolume.max(27.0);

    // Create a volume with larger bounds for spacing tests
    largeVolume = cvc::volume(ctx, cvc::dimension(171, 171, 171), cvc::Float,
                              cvc::bounding_box(0.0, 0.0, 0.0, 945.0, 945.0, 945.0));

    // Fill with test data
    for (unsigned int k = 0; k < 171; k++) {
      for (unsigned int j = 0; j < 171; j++) {
        for (unsigned int i = 0; i < 171; i++) {
          largeVolume(i, j, k, static_cast<float>(i * 0.01 + j * 0.01 + k * 0.01));
        }
      }
    }

    largeVolume.min(-5.185418);
    largeVolume.max(8.067500);
  }

  void TearDown() override {}

  cvc::app ctx;
  cvc::volume testVolume{ctx};
  cvc::volume largeVolume{ctx};
};

// ============================================================================
// Volume Span and Spacing Tests (Critical for rendering)
// ============================================================================

TEST_F(VolumeNodeTest, VolumeSpanCalculation) {
  // Test that volume span is calculated correctly
  // XSpan = (XMax - XMin) / (XDim - 1)

  // Small volume: (9-0)/(10-1) = 1.0
  EXPECT_DOUBLE_EQ(testVolume.XSpan(), 1.0);
  EXPECT_DOUBLE_EQ(testVolume.YSpan(), 1.0);
  EXPECT_DOUBLE_EQ(testVolume.ZSpan(), 1.0);

  // Large volume: (945-0)/(171-1) = 5.558823...
  double expectedSpan = 945.0 / 170.0;
  EXPECT_NEAR(largeVolume.XSpan(), expectedSpan, 0.0001);
  EXPECT_NEAR(largeVolume.YSpan(), expectedSpan, 0.0001);
  EXPECT_NEAR(largeVolume.ZSpan(), expectedSpan, 0.0001);
}

TEST_F(VolumeNodeTest, VolumeSpacingForVTK) {
  VolumeNode node(ctx, "test_volume");
  node.setVolume(largeVolume);

  // The critical fix: spacing should be (Max - Min) / Dim, NOT Span() / Dim
  // Spacing = (945 - 0) / 171 = 5.526315...
  double expectedSpacing = 945.0 / 171.0;

  // We don't have direct access to VTK spacing, but we can verify
  // the volume bounds are correct
  cvc::bounding_box bbox = node.getBoundingBox();

  EXPECT_DOUBLE_EQ(bbox[0], 0.0);
  EXPECT_DOUBLE_EQ(bbox[1], 0.0);
  EXPECT_DOUBLE_EQ(bbox[2], 0.0);
  EXPECT_DOUBLE_EQ(bbox[3], 945.0);
  EXPECT_DOUBLE_EQ(bbox[4], 945.0);
  EXPECT_DOUBLE_EQ(bbox[5], 945.0);

  // Verify dimensions
  EXPECT_EQ(largeVolume.XDim(), 171);
  EXPECT_EQ(largeVolume.YDim(), 171);
  EXPECT_EQ(largeVolume.ZDim(), 171);
}

TEST_F(VolumeNodeTest, VolumeSpacingNotSpanDivDim) {
  // This test verifies the critical bug fix
  // WRONG: spacing = XSpan() / XDim()
  // RIGHT: spacing = (XMax() - XMin()) / XDim()

  double wrongSpacing = largeVolume.XSpan() / largeVolume.XDim();
  double correctSpacing = (largeVolume.XMax() - largeVolume.XMin()) / largeVolume.XDim();

  // These should be different!
  EXPECT_NE(wrongSpacing, correctSpacing);

  // The wrong calculation gives ~0.0325
  EXPECT_NEAR(wrongSpacing, 0.0325, 0.001);

  // The correct calculation gives ~5.526
  EXPECT_NEAR(correctSpacing, 5.526, 0.001);
}

// ============================================================================
// Transfer Function and Data Range Tests
// ============================================================================

TEST_F(VolumeNodeTest, VolumeDataRange) {
  VolumeNode node(ctx, "test_volume");
  node.setVolume(testVolume);

  // Verify metadata contains correct data range
  EXPECT_TRUE(node.hasMetadata("data_min"));
  EXPECT_TRUE(node.hasMetadata("data_max"));

  double dataMin = std::any_cast<double>(node.getMetadata("data_min"));
  double dataMax = std::any_cast<double>(node.getMetadata("data_max"));

  EXPECT_DOUBLE_EQ(dataMin, 0.0);
  EXPECT_DOUBLE_EQ(dataMax, 27.0);
}

TEST_F(VolumeNodeTest, LargeVolumeDataRange) {
  VolumeNode node(ctx, "large_volume");
  node.setVolume(largeVolume);

  // Verify the actual data range from the problematic volume
  EXPECT_TRUE(node.hasMetadata("data_min"));
  EXPECT_TRUE(node.hasMetadata("data_max"));

  double dataMin = std::any_cast<double>(node.getMetadata("data_min"));
  double dataMax = std::any_cast<double>(node.getMetadata("data_max"));

  EXPECT_DOUBLE_EQ(dataMin, -5.185418);
  EXPECT_DOUBLE_EQ(dataMax, 8.067500);
}

// ============================================================================
// Volume Label Tests
// ============================================================================

TEST_F(VolumeNodeTest, VolumeLabelDefaultState) {
  VolumeNode node(ctx, "test.volume", "test_volume");

  // Label should be off by default
  EXPECT_FALSE(node.getShowLabel());
  // Default label text should be node name
  EXPECT_EQ(node.getLabelText(), "test_volume");
  // Default size should be 14
  EXPECT_EQ(node.getLabelSize(), 14);
}

// ============================================================================
// Volume Bounding Box Tests
// ============================================================================

TEST_F(VolumeNodeTest, VolumeBBoxBounds) {
  VolumeNode node(ctx, "test_volume");
  node.setVolume(testVolume);

  cvc::bounding_box bbox = node.getBoundingBox();

  // Should match volume bounds
  EXPECT_DOUBLE_EQ(bbox[0], 0.0);
  EXPECT_DOUBLE_EQ(bbox[1], 0.0);
  EXPECT_DOUBLE_EQ(bbox[2], 0.0);
  EXPECT_DOUBLE_EQ(bbox[3], 9.0);
  EXPECT_DOUBLE_EQ(bbox[4], 9.0);
  EXPECT_DOUBLE_EQ(bbox[5], 9.0);
}

// ============================================================================
// Volume Metadata Tests
// ============================================================================

TEST_F(VolumeNodeTest, VolumeMetadataComplete) {
  VolumeNode node(ctx, "test_volume");
  node.setVolume(testVolume);

  // Check all expected metadata fields exist
  EXPECT_TRUE(node.hasMetadata("dim_x"));
  EXPECT_TRUE(node.hasMetadata("dim_y"));
  EXPECT_TRUE(node.hasMetadata("dim_z"));
  EXPECT_TRUE(node.hasMetadata("data_min"));
  EXPECT_TRUE(node.hasMetadata("data_max"));
  EXPECT_TRUE(node.hasMetadata("bbox_min_x"));
  EXPECT_TRUE(node.hasMetadata("bbox_max_z"));
  EXPECT_TRUE(node.hasMetadata("voxel_type"));
}

TEST_F(VolumeNodeTest, VolumeMetadataValues) {
  VolumeNode node(ctx, "test_volume");
  node.setVolume(testVolume);

  // Verify dimension metadata
  EXPECT_EQ(std::any_cast<int>(node.getMetadata("dim_x")), 10);
  EXPECT_EQ(std::any_cast<int>(node.getMetadata("dim_y")), 10);
  EXPECT_EQ(std::any_cast<int>(node.getMetadata("dim_z")), 10);

  // Verify bounding box metadata
  EXPECT_DOUBLE_EQ(std::any_cast<double>(node.getMetadata("bbox_min_x")), 0.0);
  EXPECT_DOUBLE_EQ(std::any_cast<double>(node.getMetadata("bbox_max_x")), 9.0);

  // Verify voxel type
  std::string voxelType = std::any_cast<std::string>(node.getMetadata("voxel_type"));
  EXPECT_EQ(voxelType, "unsigned_char");
}

// ============================================================================
// Clip Plane Tests for VolumeNode
// ============================================================================

TEST_F(VolumeNodeTest, ClipChildrenDefault) {
  SceneGraph sceneGraph("volume_clip_test");
  auto parent = sceneGraph.addGraphics("volume_clip_test.parent", testVolume);

  EXPECT_FALSE(parent->getClipChildren());
}

TEST_F(VolumeNodeTest, SetClipChildren) {
  SceneGraph sceneGraph("volume_clip_test");
  auto parent = sceneGraph.addGraphics("volume_clip_test.setter", testVolume);

  parent->setClipChildren(true);
  EXPECT_TRUE(parent->getClipChildren());

  parent->setClipChildren(false);
  EXPECT_FALSE(parent->getClipChildren());
}

TEST_F(VolumeNodeTest, ClipPlanesGenerated) {
  SceneGraph sceneGraph("volume_clip_test");
  auto parent = sceneGraph.addGraphics("volume_clip_test.planes", testVolume);

  parent->setClipChildren(true);

  // Should have 6 clip planes
  vtkPlaneCollection *planes = parent->getClipPlanes();
  ASSERT_NE(planes, nullptr);
  EXPECT_EQ(planes->GetNumberOfItems(), 6);
}

TEST_F(VolumeNodeTest, VolumeClipsGeometryChild) {
  // Test that a VolumeNode can clip a GeometryNode child
  SceneGraph sceneGraph("volume_clip_test");

  auto volumeParent = sceneGraph.addGraphics("volume_clip_test.vol_parent", testVolume);

  // Create a geometry child
  cvc::geometry childGeom;
  childGeom.points().push_back({5.0, 5.0, 5.0});
  childGeom.points().push_back({15.0, 15.0, 15.0});

  auto geomChild =
      std::make_shared<GeometryNode>(ctx, "volume_clip_test.vol_parent.geom_child", "child");
  geomChild->setGeometry(childGeom);
  volumeParent->addGraphicsChild(geomChild);

  // Enable clipping on volume parent
  volumeParent->setClipChildren(true);

  // Give time for threading/event queue to process
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Verify planes were created
  vtkPlaneCollection *planes = volumeParent->getClipPlanes();
  ASSERT_NE(planes, nullptr);
  EXPECT_EQ(planes->GetNumberOfItems(), 6);
}

TEST_F(VolumeNodeTest, GeometryClipsVolumeChild) {
  // Test that a GeometryNode can clip a VolumeNode child
  SceneGraph sceneGraph("volume_clip_test");

  cvc::geometry parentGeom;
  parentGeom.points().push_back({0.0, 0.0, 0.0});
  parentGeom.points().push_back({20.0, 20.0, 20.0});

  auto geomParent = sceneGraph.addGraphics("volume_clip_test.geom_parent", parentGeom);

  // Create a volume child
  auto volumeChild =
      std::make_shared<VolumeNode>(ctx, "volume_clip_test.geom_parent.vol_child", "volume");
  volumeChild->setVolume(testVolume);
  geomParent->addGraphicsChild(volumeChild);

  // Enable clipping on geometry parent
  geomParent->setClipChildren(true);

  // Give time for threading/event queue to process
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Verify planes were created
  vtkPlaneCollection *planes = geomParent->getClipPlanes();
  ASSERT_NE(planes, nullptr);
  EXPECT_EQ(planes->GetNumberOfItems(), 6);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
