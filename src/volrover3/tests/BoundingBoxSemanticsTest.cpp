#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/core/state.h>
#include <cvc/volume/volume.h>
#include <gtest/gtest.h>
#include <volrover3/GeometryNode.h>
#include <volrover3/VolumeNode.h>

class BoundingBoxSemanticsTest : public ::testing::Test {
protected:
  cvc::app ctx;
  void SetUp() override { m_statePrefix = "test_bbox_semantics_" + std::to_string(testCounter++); }

  // Helper to create geometry with specific bounds
  cvc::geometry createGeometry(double minX, double minY, double minZ, double maxX, double maxY,
                               double maxZ) {
    cvc::geometry geom;
    geom.points().resize(2);
    geom.points()[0][0] = minX;
    geom.points()[0][1] = minY;
    geom.points()[0][2] = minZ;
    geom.points()[1][0] = maxX;
    geom.points()[1][1] = maxY;
    geom.points()[1][2] = maxZ;
    return geom;
  }

  std::string m_statePrefix;
  static int testCounter;
};

int BoundingBoxSemanticsTest::testCounter = 0;

// Test parent node's own bounding box
TEST_F(BoundingBoxSemanticsTest, ParentOwnBoundingBox) {
  auto parent = std::make_shared<GeometryNode>(ctx, "parent");
  auto geom = createGeometry(0, 0, 0, 10, 10, 10);
  parent->setGeometry(geom);

  // Parent's own bounding box should be its geometry bounds
  auto bbox = parent->getBoundingBox();
  EXPECT_NEAR(bbox[0], 0.0, 1e-6);
  EXPECT_NEAR(bbox[1], 0.0, 1e-6);
  EXPECT_NEAR(bbox[2], 0.0, 1e-6);
  EXPECT_NEAR(bbox[3], 10.0, 1e-6);
  EXPECT_NEAR(bbox[4], 10.0, 1e-6);
  EXPECT_NEAR(bbox[5], 10.0, 1e-6);
}

// Test combined bounding box with no children equals own bounding box
TEST_F(BoundingBoxSemanticsTest, CombinedBBoxNoChildren) {
  auto parent = std::make_shared<GeometryNode>(ctx, "parent");
  auto geom = createGeometry(5, 5, 5, 15, 15, 15);
  parent->setGeometry(geom);

  auto ownBBox = parent->getBoundingBox();
  auto combinedBBox = parent->getCombinedBoundingBox();

  // Should be identical when no children
  EXPECT_NEAR(ownBBox[0], combinedBBox[0], 1e-6);
  EXPECT_NEAR(ownBBox[1], combinedBBox[1], 1e-6);
  EXPECT_NEAR(ownBBox[2], combinedBBox[2], 1e-6);
  EXPECT_NEAR(ownBBox[3], combinedBBox[3], 1e-6);
  EXPECT_NEAR(ownBBox[4], combinedBBox[4], 1e-6);
  EXPECT_NEAR(ownBBox[5], combinedBBox[5], 1e-6);
}

// Test combined bounding box includes child
TEST_F(BoundingBoxSemanticsTest, CombinedBBoxIncludesChild) {
  auto parent = std::make_shared<GeometryNode>(ctx, "parent");
  auto parentGeom = createGeometry(0, 0, 0, 10, 10, 10);
  parent->setGeometry(parentGeom);

  auto child = std::make_shared<GeometryNode>(ctx, "child");
  auto childGeom = createGeometry(15, 15, 15, 25, 25, 25);
  child->setGeometry(childGeom);

  parent->addGraphicsChild(child);

  // Parent's own bbox should still be its geometry
  auto ownBBox = parent->getBoundingBox();
  EXPECT_NEAR(ownBBox[0], 0.0, 1e-6);
  EXPECT_NEAR(ownBBox[3], 10.0, 1e-6);

  // Combined bbox should include both parent and child
  auto combinedBBox = parent->getCombinedBoundingBox();
  EXPECT_NEAR(combinedBBox[0], 0.0, 1e-6); // min from parent
  EXPECT_NEAR(combinedBBox[1], 0.0, 1e-6);
  EXPECT_NEAR(combinedBBox[2], 0.0, 1e-6);
  EXPECT_NEAR(combinedBBox[3], 25.0, 1e-6); // max from child
  EXPECT_NEAR(combinedBBox[4], 25.0, 1e-6);
  EXPECT_NEAR(combinedBBox[5], 25.0, 1e-6);
}

// Test combined bounding box with multiple children
TEST_F(BoundingBoxSemanticsTest, CombinedBBoxMultipleChildren) {
  auto parent = std::make_shared<GeometryNode>(ctx, "parent");
  auto parentGeom = createGeometry(10, 10, 10, 20, 20, 20);
  parent->setGeometry(parentGeom);

  auto child1 = std::make_shared<GeometryNode>(ctx, "child1");
  auto child1Geom = createGeometry(0, 0, 0, 5, 5, 5);
  child1->setGeometry(child1Geom);
  parent->addGraphicsChild(child1);

  auto child2 = std::make_shared<GeometryNode>(ctx, "child2");
  auto child2Geom = createGeometry(30, 30, 30, 40, 40, 40);
  child2->setGeometry(child2Geom);
  parent->addGraphicsChild(child2);

  // Parent's own bbox unchanged
  auto ownBBox = parent->getBoundingBox();
  EXPECT_NEAR(ownBBox[0], 10.0, 1e-6);
  EXPECT_NEAR(ownBBox[3], 20.0, 1e-6);

  // Combined bbox should span all three
  auto combinedBBox = parent->getCombinedBoundingBox();
  EXPECT_NEAR(combinedBBox[0], 0.0, 1e-6); // min from child1
  EXPECT_NEAR(combinedBBox[1], 0.0, 1e-6);
  EXPECT_NEAR(combinedBBox[2], 0.0, 1e-6);
  EXPECT_NEAR(combinedBBox[3], 40.0, 1e-6); // max from child2
  EXPECT_NEAR(combinedBBox[4], 40.0, 1e-6);
  EXPECT_NEAR(combinedBBox[5], 40.0, 1e-6);
}

// Test combined bounding box with nested children (grandchildren)
TEST_F(BoundingBoxSemanticsTest, CombinedBBoxNestedChildren) {
  auto grandparent = std::make_shared<GeometryNode>(ctx, "grandparent");
  auto grandparentGeom = createGeometry(50, 50, 50, 60, 60, 60);
  grandparent->setGeometry(grandparentGeom);

  auto parent = std::make_shared<GeometryNode>(ctx, "parent");
  auto parentGeom = createGeometry(0, 0, 0, 10, 10, 10);
  parent->setGeometry(parentGeom);
  grandparent->addGraphicsChild(parent);

  auto child = std::make_shared<GeometryNode>(ctx, "child");
  auto childGeom = createGeometry(100, 100, 100, 110, 110, 110);
  child->setGeometry(childGeom);
  parent->addGraphicsChild(child);

  // Grandparent's own bbox is just its geometry
  auto grandparentOwnBBox = grandparent->getBoundingBox();
  EXPECT_NEAR(grandparentOwnBBox[0], 50.0, 1e-6);
  EXPECT_NEAR(grandparentOwnBBox[3], 60.0, 1e-6);

  // Parent's combined bbox includes parent + child
  auto parentCombinedBBox = parent->getCombinedBoundingBox();
  EXPECT_NEAR(parentCombinedBBox[0], 0.0, 1e-6);
  EXPECT_NEAR(parentCombinedBBox[3], 110.0, 1e-6);

  // Grandparent's combined bbox includes all three levels
  auto grandparentCombinedBBox = grandparent->getCombinedBoundingBox();
  EXPECT_NEAR(grandparentCombinedBBox[0], 0.0, 1e-6);   // min from parent
  EXPECT_NEAR(grandparentCombinedBBox[3], 110.0, 1e-6); // max from child
}

// Test parent with no geometry but has children
TEST_F(BoundingBoxSemanticsTest, ParentNoGeometryHasChildren) {
  auto parent = std::make_shared<GeometryNode>(ctx, "parent");
  // No geometry set on parent

  auto child = std::make_shared<GeometryNode>(ctx, "child");
  auto childGeom = createGeometry(10, 20, 30, 40, 50, 60);
  child->setGeometry(childGeom);
  parent->addGraphicsChild(child);

  // Parent's own bbox should be empty/default (0,0,0 to 0,0,0)
  auto ownBBox = parent->getBoundingBox();
  EXPECT_DOUBLE_EQ(ownBBox[0], 0.0);
  EXPECT_DOUBLE_EQ(ownBBox[3], 0.0);

  // Combined bbox starts with parent's bbox (0,0,0) and expands to include child
  // So combined is min(0, 10) to max(0, 40) = 0 to 40 (includes origin)
  auto combinedBBox = parent->getCombinedBoundingBox();
  EXPECT_NEAR(combinedBBox[0], 0.0, 1e-6);  // min(0, 10)
  EXPECT_NEAR(combinedBBox[1], 0.0, 1e-6);  // min(0, 20)
  EXPECT_NEAR(combinedBBox[2], 0.0, 1e-6);  // min(0, 30)
  EXPECT_NEAR(combinedBBox[3], 40.0, 1e-6); // max(0, 40)
  EXPECT_NEAR(combinedBBox[4], 50.0, 1e-6); // max(0, 50)
  EXPECT_NEAR(combinedBBox[5], 60.0, 1e-6); // max(0, 60)
}

// Test volume node bounding box semantics

// Test combined bbox with volume and geometry children

// Test child removed updates combined bbox
TEST_F(BoundingBoxSemanticsTest, ChildRemovedUpdatesCombinedBBox) {
  auto parent = std::make_shared<GeometryNode>(ctx, "parent");
  auto parentGeom = createGeometry(10, 10, 10, 20, 20, 20);
  parent->setGeometry(parentGeom);

  auto child = std::make_shared<GeometryNode>(ctx, "child");
  auto childGeom = createGeometry(50, 50, 50, 100, 100, 100);
  child->setGeometry(childGeom);
  parent->addGraphicsChild(child);

  // Combined bbox with child
  auto combinedBBoxWithChild = parent->getCombinedBoundingBox();
  EXPECT_NEAR(combinedBBoxWithChild[0], 10.0, 1e-6);
  EXPECT_NEAR(combinedBBoxWithChild[3], 100.0, 1e-6);

  // Remove child
  parent->removeGraphicsChild(child);

  // Combined bbox should now be same as own bbox
  auto combinedBBoxNoChild = parent->getCombinedBoundingBox();
  auto ownBBox = parent->getBoundingBox();
  EXPECT_NEAR(combinedBBoxNoChild[0], ownBBox[0], 1e-6);
  EXPECT_NEAR(combinedBBoxNoChild[3], ownBBox[3], 1e-6);
}

// Test non-overlapping children
TEST_F(BoundingBoxSemanticsTest, NonOverlappingChildren) {
  auto parent = std::make_shared<GeometryNode>(ctx, "parent");
  auto parentGeom = createGeometry(0, 0, 0, 1, 1, 1);
  parent->setGeometry(parentGeom);

  auto child1 = std::make_shared<GeometryNode>(ctx, "child1");
  auto child1Geom = createGeometry(-100, -100, -100, -90, -90, -90);
  child1->setGeometry(child1Geom);
  parent->addGraphicsChild(child1);

  auto child2 = std::make_shared<GeometryNode>(ctx, "child2");
  auto child2Geom = createGeometry(200, 200, 200, 210, 210, 210);
  child2->setGeometry(child2Geom);
  parent->addGraphicsChild(child2);

  // Combined bbox should span entire range
  auto combinedBBox = parent->getCombinedBoundingBox();
  EXPECT_NEAR(combinedBBox[0], -100.0, 1e-6);
  EXPECT_NEAR(combinedBBox[1], -100.0, 1e-6);
  EXPECT_NEAR(combinedBBox[2], -100.0, 1e-6);
  EXPECT_NEAR(combinedBBox[3], 210.0, 1e-6);
  EXPECT_NEAR(combinedBBox[4], 210.0, 1e-6);
  EXPECT_NEAR(combinedBBox[5], 210.0, 1e-6);
}

// Test negative coordinates
TEST_F(BoundingBoxSemanticsTest, NegativeCoordinates) {
  auto parent = std::make_shared<GeometryNode>(ctx, "parent");
  auto parentGeom = createGeometry(-50, -60, -70, -10, -20, -30);
  parent->setGeometry(parentGeom);

  auto child = std::make_shared<GeometryNode>(ctx, "child");
  auto childGeom = createGeometry(-200, -150, -100, -180, -130, -80);
  child->setGeometry(childGeom);
  parent->addGraphicsChild(child);

  // Parent's own bbox
  auto ownBBox = parent->getBoundingBox();
  EXPECT_NEAR(ownBBox[0], -50.0, 1e-6);
  EXPECT_NEAR(ownBBox[3], -10.0, 1e-6);

  // Combined bbox with negative values
  auto combinedBBox = parent->getCombinedBoundingBox();
  EXPECT_NEAR(combinedBBox[0], -200.0, 1e-6);
  EXPECT_NEAR(combinedBBox[1], -150.0, 1e-6);
  EXPECT_NEAR(combinedBBox[2], -100.0, 1e-6);
  EXPECT_NEAR(combinedBBox[3], -10.0, 1e-6);
  EXPECT_NEAR(combinedBBox[4], -20.0, 1e-6);
  EXPECT_NEAR(combinedBBox[5], -30.0, 1e-6);
}

// Test empty parent with empty child
TEST_F(BoundingBoxSemanticsTest, EmptyParentEmptyChild) {
  auto parent = std::make_shared<GeometryNode>(ctx, "parent");
  auto child = std::make_shared<GeometryNode>(ctx, "child");
  parent->addGraphicsChild(child);

  // Both should have default/empty bboxes
  auto ownBBox = parent->getBoundingBox();
  auto combinedBBox = parent->getCombinedBoundingBox();

  // Combined should equal own when both empty
  EXPECT_DOUBLE_EQ(ownBBox[0], combinedBBox[0]);
  EXPECT_DOUBLE_EQ(ownBBox[3], combinedBBox[3]);
}

// Test single point geometry
TEST_F(BoundingBoxSemanticsTest, SinglePointGeometry) {
  auto parent = std::make_shared<GeometryNode>(ctx, "parent");
  cvc::geometry geom;
  geom.points().resize(1);
  geom.points()[0][0] = 5.5;
  geom.points()[0][1] = 6.6;
  geom.points()[0][2] = 7.7;
  parent->setGeometry(geom);

  auto bbox = parent->getBoundingBox();
  // Single point creates a bbox with min == max
  EXPECT_NEAR(bbox[0], 5.5, 1e-6);
  EXPECT_NEAR(bbox[1], 6.6, 1e-6);
  EXPECT_NEAR(bbox[2], 7.7, 1e-6);
  EXPECT_NEAR(bbox[3], 5.5, 1e-6);
  EXPECT_NEAR(bbox[4], 6.6, 1e-6);
  EXPECT_NEAR(bbox[5], 7.7, 1e-6);

  // Combined should be same
  auto combinedBBox = parent->getCombinedBoundingBox();
  EXPECT_NEAR(combinedBBox[0], 5.5, 1e-6);
  EXPECT_NEAR(combinedBBox[3], 5.5, 1e-6);
}
