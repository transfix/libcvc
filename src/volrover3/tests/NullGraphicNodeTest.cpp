#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/state_object.h>
#include <cvc/geometry/geometry.h>
#include <gtest/gtest.h>
#include <volrover3/GeometryNode.h>
#include <volrover3/NullGraphicNode.h>
#include <volrover3/SceneGraph.h>

class NullGraphicNodeTest : public ::testing::Test {
protected:
  cvc::app ctx;
  void SetUp() override {
    // Disable threading for state_object to avoid destruction race conditions
    cvc::state_object<SceneNode>::setUseThreading(false);

    m_statePrefix = "test_null_graphic_" + std::to_string(testCounter++);
  }

  void TearDown() override {
    // disconnectState() in SceneNode destructor prevents callbacks during destruction
  }

  std::string m_statePrefix;
  static int testCounter;
};

int NullGraphicNodeTest::testCounter = 0;

// Test NullGraphicNode default construction
TEST_F(NullGraphicNodeTest, DefaultConstruction) {
  auto nullNode = std::make_shared<NullGraphicNode>(ctx, "test.null", "test_null");

  ASSERT_NE(nullNode, nullptr);
  EXPECT_EQ(nullNode->getName(), "test_null");

  // Check default bounding box (1x1x1 centered at origin)
  auto bbox = nullNode->getBoundingBox();
  EXPECT_DOUBLE_EQ(bbox[0], -0.5); // min x
  EXPECT_DOUBLE_EQ(bbox[1], -0.5); // min y
  EXPECT_DOUBLE_EQ(bbox[2], -0.5); // min z
  EXPECT_DOUBLE_EQ(bbox[3], 0.5);  // max x
  EXPECT_DOUBLE_EQ(bbox[4], 0.5);  // max y
  EXPECT_DOUBLE_EQ(bbox[5], 0.5);  // max z
}

// Test NullGraphicNode bounds can be modified
TEST_F(NullGraphicNodeTest, SetBoundsArray) {
  auto nullNode = std::make_shared<NullGraphicNode>(ctx, "test_null");

  cvc::bounding_box newBounds(-50, -25, -10, 50, 25, 10);
  nullNode->setBounds(newBounds);

  auto bbox = nullNode->getBoundingBox();
  EXPECT_DOUBLE_EQ(bbox[0], -50.0);
  EXPECT_DOUBLE_EQ(bbox[1], -25.0);
  EXPECT_DOUBLE_EQ(bbox[2], -10.0);
  EXPECT_DOUBLE_EQ(bbox[3], 50.0);
  EXPECT_DOUBLE_EQ(bbox[4], 25.0);
  EXPECT_DOUBLE_EQ(bbox[5], 10.0);
}

// Test NullGraphicNode bounds can be set with individual values
TEST_F(NullGraphicNodeTest, SetBoundsIndividual) {
  auto nullNode = std::make_shared<NullGraphicNode>(ctx, "test_null");

  nullNode->setBounds(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

  auto bbox = nullNode->getBoundingBox();
  EXPECT_DOUBLE_EQ(bbox[0], 1.0);
  EXPECT_DOUBLE_EQ(bbox[1], 2.0);
  EXPECT_DOUBLE_EQ(bbox[2], 3.0);
  EXPECT_DOUBLE_EQ(bbox[3], 4.0);
  EXPECT_DOUBLE_EQ(bbox[4], 5.0);
  EXPECT_DOUBLE_EQ(bbox[5], 6.0);
}

// Test SceneGraph creates null graphic as graphics root
TEST_F(NullGraphicNodeTest, SceneGraphInitialNullGraphic) {
  SceneGraph sceneGraph(m_statePrefix);

  // Graphics root IS the null graphic node
  auto nullNode = std::dynamic_pointer_cast<NullGraphicNode>(sceneGraph.getGraphicsRoot());
  ASSERT_NE(nullNode, nullptr);
  EXPECT_EQ(nullNode->getName(), "root");

  // Should have bbox visible by default
  EXPECT_TRUE(nullNode->getShowBBox());

  // Should have grid and axis as initial children
  auto children = nullNode->getGraphicsChildren();
  EXPECT_EQ(children.size(), 2);
}

// Test geometry added as child of null graphic root
TEST_F(NullGraphicNodeTest, NullGraphicRemovedOnGeometryAdd) {
  SceneGraph sceneGraph(m_statePrefix);

  // Graphics root is the null graphic, starts with grid and axis
  auto nullNode = std::dynamic_pointer_cast<NullGraphicNode>(sceneGraph.getGraphicsRoot());
  ASSERT_NE(nullNode, nullptr);
  auto children = nullNode->getGraphicsChildren();
  ASSERT_EQ(children.size(), 2);

  // Add geometry
  cvc::geometry geom;
  geom.points().resize(3);
  geom.points()[0][0] = 0;
  geom.points()[0][1] = 0;
  geom.points()[0][2] = 0;
  geom.points()[1][0] = 1;
  geom.points()[1][1] = 0;
  geom.points()[1][2] = 0;
  geom.points()[2][0] = 0;
  geom.points()[2][1] = 1;
  geom.points()[2][2] = 0;

  sceneGraph.addGraphics("test_geom", geom);

  // Null graphic is still the root, now has grid + axis + test_geom = 3 children
  children = nullNode->getGraphicsChildren();
  ASSERT_EQ(children.size(), 3);

  // children[0] is grid, children[1] is axis, children[2] should be the geometry we added
  auto geomNode = std::dynamic_pointer_cast<GeometryNode>(children[2]);
  ASSERT_NE(geomNode, nullptr);
  EXPECT_EQ(geomNode->getName(), "test_geom");
}

// Test null graphic root remains when graphics removed
TEST_F(NullGraphicNodeTest, NullGraphicRestoredOnRemove) {
  SceneGraph sceneGraph(m_statePrefix);

  // Add geometry
  cvc::geometry geom;
  geom.points().resize(3);
  geom.points()[0][0] = 0;
  geom.points()[0][1] = 0;
  geom.points()[0][2] = 0;
  geom.points()[1][0] = 1;
  geom.points()[1][1] = 0;
  geom.points()[1][2] = 0;
  geom.points()[2][0] = 0;
  geom.points()[2][1] = 1;
  geom.points()[2][2] = 0;
  sceneGraph.addGraphics("test_geom", geom);

  // Graphics root (null graphic) has grid + axis + test_geom = 3 children
  auto nullNode = std::dynamic_pointer_cast<NullGraphicNode>(sceneGraph.getGraphicsRoot());
  ASSERT_NE(nullNode, nullptr);
  auto children = nullNode->getGraphicsChildren();
  ASSERT_EQ(children.size(), 3);

  // Remove the geometry
  sceneGraph.removeGraphics("test_geom");

  // Graphics root (null graphic) is back to grid and axis only
  children = nullNode->getGraphicsChildren();
  EXPECT_EQ(children.size(), 2);
}

// Test null graphic not counted as real graphic
TEST_F(NullGraphicNodeTest, NullGraphicNotInGraphicsMap) {
  SceneGraph sceneGraph(m_statePrefix);

  // Graphics root IS the null graphic, starts with grid and axis children
  auto nullNode = std::dynamic_pointer_cast<NullGraphicNode>(sceneGraph.getGraphicsRoot());
  ASSERT_NE(nullNode, nullptr);
  auto children = nullNode->getGraphicsChildren();
  ASSERT_EQ(children.size(), 2);

  // Verify null graphic is not accessible via getGraphics (it's the root, not a child)
  auto nullFromMap = sceneGraph.getGraphics("root");
  EXPECT_EQ(nullFromMap, nullptr);

  // Add real graphic
  cvc::geometry geom;
  geom.points().resize(3);
  geom.points()[0][0] = 0;
  geom.points()[0][1] = 0;
  geom.points()[0][2] = 0;
  geom.points()[1][0] = 1;
  geom.points()[1][1] = 0;
  geom.points()[1][2] = 0;
  geom.points()[2][0] = 0;
  geom.points()[2][1] = 1;
  geom.points()[2][2] = 0;
  sceneGraph.addGraphics("test_geom", geom);

  // Now should be able to get the real graphic
  auto realGraphic = sceneGraph.getGraphics("test_geom");
  ASSERT_NE(realGraphic, nullptr);
  EXPECT_EQ(realGraphic->getName(), "test_geom");
}

// Test large custom bounds
TEST_F(NullGraphicNodeTest, LargeCustomBounds) {
  auto nullNode = std::make_shared<NullGraphicNode>(ctx, "test_null");

  nullNode->setBounds(-1000.0, -2000.0, -3000.0, 1000.0, 2000.0, 3000.0);

  auto bbox = nullNode->getBoundingBox();
  EXPECT_DOUBLE_EQ(bbox[0], -1000.0);
  EXPECT_DOUBLE_EQ(bbox[1], -2000.0);
  EXPECT_DOUBLE_EQ(bbox[2], -3000.0);
  EXPECT_DOUBLE_EQ(bbox[3], 1000.0);
  EXPECT_DOUBLE_EQ(bbox[4], 2000.0);
  EXPECT_DOUBLE_EQ(bbox[5], 3000.0);
}

// Test asymmetric bounds
TEST_F(NullGraphicNodeTest, AsymmetricBounds) {
  auto nullNode = std::make_shared<NullGraphicNode>(ctx, "test_null");

  nullNode->setBounds(-500.0, 100.0, -200.0, 300.0, 1000.0, 500.0);

  auto bbox = nullNode->getBoundingBox();
  EXPECT_DOUBLE_EQ(bbox[0], -500.0);
  EXPECT_DOUBLE_EQ(bbox[1], 100.0);
  EXPECT_DOUBLE_EQ(bbox[2], -200.0);
  EXPECT_DOUBLE_EQ(bbox[3], 300.0);
  EXPECT_DOUBLE_EQ(bbox[4], 1000.0);
  EXPECT_DOUBLE_EQ(bbox[5], 500.0);
}

// Test includeOwnBounds flag - default should be false
TEST_F(NullGraphicNodeTest, IncludeOwnBoundsDefault) {
  auto nullNode = std::make_shared<NullGraphicNode>(ctx, "test_null");

  // Default should be false (typical for root nodes)
  EXPECT_FALSE(nullNode->getIncludeOwnBounds());
}

// Test setting includeOwnBounds flag
TEST_F(NullGraphicNodeTest, SetIncludeOwnBounds) {
  auto nullNode = std::make_shared<NullGraphicNode>(ctx, "test_null");

  nullNode->setIncludeOwnBounds(true);
  EXPECT_TRUE(nullNode->getIncludeOwnBounds());

  nullNode->setIncludeOwnBounds(false);
  EXPECT_FALSE(nullNode->getIncludeOwnBounds());
}

// Test includeOwnBounds state tree synchronization
TEST_F(NullGraphicNodeTest, IncludeOwnBoundsStateSync) {
  auto nullNode = std::make_shared<NullGraphicNode>(ctx, "test.null");

  // Set via method
  nullNode->setIncludeOwnBounds(true);

  // Verify state tree was updated
  int stateValue = nullNode->getState("include_own_bounds").value<int>();
  EXPECT_EQ(stateValue, 1);

  // Change via state tree
  nullNode->getState("include_own_bounds").value(0);

  // Should trigger handleStateChanged and update member
  EXPECT_FALSE(nullNode->getIncludeOwnBounds());
}

// Test syncBoundsWithChildren flag - default should be true
TEST_F(NullGraphicNodeTest, SyncBoundsWithChildrenDefault) {
  auto nullNode = std::make_shared<NullGraphicNode>(ctx, "test_null");

  // Default should be true (auto-sync enabled)
  EXPECT_TRUE(nullNode->getSyncBoundsWithChildren());
}

// Test setting syncBoundsWithChildren flag
TEST_F(NullGraphicNodeTest, SetSyncBoundsWithChildren) {
  auto nullNode = std::make_shared<NullGraphicNode>(ctx, "test_null");

  nullNode->setSyncBoundsWithChildren(false);
  EXPECT_FALSE(nullNode->getSyncBoundsWithChildren());

  nullNode->setSyncBoundsWithChildren(true);
  EXPECT_TRUE(nullNode->getSyncBoundsWithChildren());
}

// Test syncBoundsWithChildren state tree synchronization
TEST_F(NullGraphicNodeTest, SyncBoundsWithChildrenStateSync) {
  auto nullNode = std::make_shared<NullGraphicNode>(ctx, "test.null");

  // Set via method
  nullNode->setSyncBoundsWithChildren(false);

  // Verify state tree was updated
  int stateValue = nullNode->getState("sync_bounds_with_children").value<int>();
  EXPECT_EQ(stateValue, 0);

  // Change via state tree
  nullNode->getState("sync_bounds_with_children").value(1);

  // Should trigger handleStateChanged and update member
  EXPECT_TRUE(nullNode->getSyncBoundsWithChildren());
}

// Test syncBoundsToChildren with single child
TEST_F(NullGraphicNodeTest, SyncBoundsToChildrenSingleChild) {
  SceneGraph sceneGraph(m_statePrefix);
  auto nullNode = std::dynamic_pointer_cast<NullGraphicNode>(sceneGraph.getGraphicsRoot());
  ASSERT_NE(nullNode, nullptr);

  // Set initial bounds
  nullNode->setBounds(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);

  // Add geometry with known bounds
  cvc::geometry geom;
  geom.points().resize(8);
  geom.points()[0][0] = 0;
  geom.points()[0][1] = 0;
  geom.points()[0][2] = 0;
  geom.points()[1][0] = 10;
  geom.points()[1][1] = 0;
  geom.points()[1][2] = 0;
  geom.points()[2][0] = 10;
  geom.points()[2][1] = 10;
  geom.points()[2][2] = 0;
  geom.points()[3][0] = 0;
  geom.points()[3][1] = 10;
  geom.points()[3][2] = 0;
  geom.points()[4][0] = 0;
  geom.points()[4][1] = 0;
  geom.points()[4][2] = 10;
  geom.points()[5][0] = 10;
  geom.points()[5][1] = 0;
  geom.points()[5][2] = 10;
  geom.points()[6][0] = 10;
  geom.points()[6][1] = 10;
  geom.points()[6][2] = 10;
  geom.points()[7][0] = 0;
  geom.points()[7][1] = 10;
  geom.points()[7][2] = 10;

  sceneGraph.addGraphics("test_box", geom);

  // Enable sync (which triggers immediate sync)
  nullNode->setSyncBoundsWithChildren(true);
  nullNode->syncBoundsToChildren();

  // Bounds should now encompass the geometry (grid and axis are excluded)
  auto bbox = nullNode->getBoundingBox();
  EXPECT_NEAR(bbox[0], 0.0, 0.01);
  EXPECT_NEAR(bbox[1], 0.0, 0.01);
  EXPECT_NEAR(bbox[2], 0.0, 0.01);
  EXPECT_NEAR(bbox[3], 10.0, 0.01);
  EXPECT_NEAR(bbox[4], 10.0, 0.01);
  EXPECT_NEAR(bbox[5], 10.0, 0.01);
}

// Test syncBoundsToChildren with multiple children
TEST_F(NullGraphicNodeTest, SyncBoundsToChildrenMultipleChildren) {
  SceneGraph sceneGraph(m_statePrefix);
  auto nullNode = std::dynamic_pointer_cast<NullGraphicNode>(sceneGraph.getGraphicsRoot());
  ASSERT_NE(nullNode, nullptr);

  // Add first geometry
  cvc::geometry geom1;
  geom1.points().resize(2);
  geom1.points()[0][0] = -5;
  geom1.points()[0][1] = -5;
  geom1.points()[0][2] = -5;
  geom1.points()[1][0] = 0;
  geom1.points()[1][1] = 0;
  geom1.points()[1][2] = 0;
  sceneGraph.addGraphics("geom1", geom1);

  // Add second geometry
  cvc::geometry geom2;
  geom2.points().resize(2);
  geom2.points()[0][0] = 0;
  geom2.points()[0][1] = 0;
  geom2.points()[0][2] = 0;
  geom2.points()[1][0] = 15;
  geom2.points()[1][1] = 20;
  geom2.points()[1][2] = 25;
  sceneGraph.addGraphics("geom2", geom2);

  // Sync to children
  nullNode->setSyncBoundsWithChildren(true);
  nullNode->syncBoundsToChildren();

  // Bounds should encompass both geometries
  auto bbox = nullNode->getBoundingBox();
  EXPECT_NEAR(bbox[0], -5.0, 0.01);
  EXPECT_NEAR(bbox[1], -5.0, 0.01);
  EXPECT_NEAR(bbox[2], -5.0, 0.01);
  EXPECT_NEAR(bbox[3], 15.0, 0.01);
  EXPECT_NEAR(bbox[4], 20.0, 0.01);
  EXPECT_NEAR(bbox[5], 25.0, 0.01);
}

// Test that syncBoundsToChildren does nothing when sync is disabled
TEST_F(NullGraphicNodeTest, SyncBoundsToChildrenDisabled) {
  SceneGraph sceneGraph(m_statePrefix);
  auto nullNode = std::dynamic_pointer_cast<NullGraphicNode>(sceneGraph.getGraphicsRoot());
  ASSERT_NE(nullNode, nullptr);

  // Disable sync BEFORE setting custom bounds and adding geometry
  nullNode->setSyncBoundsWithChildren(false);

  // Set custom bounds
  nullNode->setBounds(-100.0, -100.0, -100.0, 100.0, 100.0, 100.0);

  // Add geometry
  cvc::geometry geom;
  geom.points().resize(2);
  geom.points()[0][0] = 0;
  geom.points()[0][1] = 0;
  geom.points()[0][2] = 0;
  geom.points()[1][0] = 10;
  geom.points()[1][1] = 10;
  geom.points()[1][2] = 10;
  sceneGraph.addGraphics("test_geom", geom);

  // Call syncBoundsToChildren - should have no effect
  nullNode->syncBoundsToChildren();

  // Bounds should remain unchanged
  auto bbox = nullNode->getBoundingBox();
  EXPECT_DOUBLE_EQ(bbox[0], -100.0);
  EXPECT_DOUBLE_EQ(bbox[1], -100.0);
  EXPECT_DOUBLE_EQ(bbox[2], -100.0);
  EXPECT_DOUBLE_EQ(bbox[3], 100.0);
  EXPECT_DOUBLE_EQ(bbox[4], 100.0);
  EXPECT_DOUBLE_EQ(bbox[5], 100.0);
}

// Test bounds state tree synchronization
TEST_F(NullGraphicNodeTest, BoundsStateTreeSync) {
  auto nullNode = std::make_shared<NullGraphicNode>(ctx, "test.null");

  // Set bounds via method
  nullNode->setBounds(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

  // Check state tree was updated
  std::string boundsStr = nullNode->getState("bounds").value<std::string>();
  EXPECT_EQ(boundsStr, "1,2,3,4,5,6");

  // Update via state tree
  nullNode->getState("bounds").value(std::string("-10,-20,-30,40,50,60"));

  // Should trigger handleStateChanged and update bounds
  auto bbox = nullNode->getBoundingBox();
  EXPECT_DOUBLE_EQ(bbox[0], -10.0);
  EXPECT_DOUBLE_EQ(bbox[1], -20.0);
  EXPECT_DOUBLE_EQ(bbox[2], -30.0);
  EXPECT_DOUBLE_EQ(bbox[3], 40.0);
  EXPECT_DOUBLE_EQ(bbox[4], 50.0);
  EXPECT_DOUBLE_EQ(bbox[5], 60.0);
}

// Test NullGraphicNode transform propagates to children
TEST_F(NullGraphicNodeTest, TransformPropagationToChildren) {
  SceneGraph sceneGraph("test.scenegraph");
  auto nullNode = std::dynamic_pointer_cast<NullGraphicNode>(sceneGraph.getGraphicsRoot());
  ASSERT_NE(nullNode, nullptr);

  // Set transform on null node (translate by 10 in X)
  nullNode->setPosition(10.0, 0.0, 0.0);

  // Add geometry as child (at origin, size 2x2x2)
  cvc::geometry geom;
  geom.points().resize(2);
  geom.points()[0][0] = 0;
  geom.points()[0][1] = 0;
  geom.points()[0][2] = 0;
  geom.points()[1][0] = 2;
  geom.points()[1][1] = 2;
  geom.points()[1][2] = 2;
  sceneGraph.addGraphics("geom1", geom);

  // Get the geometry child (SceneGraph may have axis/grid children too)
  auto child = sceneGraph.getGraphics("geom1");
  ASSERT_NE(child, nullptr);

  auto worldTransform = child->getWorldTransform();

  // Child's world position should be (10, 0, 0) from parent
  EXPECT_DOUBLE_EQ(worldTransform->GetElement(0, 3), 10.0);
  EXPECT_DOUBLE_EQ(worldTransform->GetElement(1, 3), 0.0);
  EXPECT_DOUBLE_EQ(worldTransform->GetElement(2, 3), 0.0);
}

// Test NullGraphicNode transform affects combined bounding box calculation
TEST_F(NullGraphicNodeTest, TransformAffectsCombinedBounds) {
  SceneGraph sceneGraph("test.scenegraph2");
  auto nullNode = std::dynamic_pointer_cast<NullGraphicNode>(sceneGraph.getGraphicsRoot());
  ASSERT_NE(nullNode, nullptr);

  // Disable includeOwnBounds so root's default bounds don't affect combined bbox
  nullNode->setIncludeOwnBounds(false);

  // Add geometry at origin (size 1x1x1 from 0 to 1)
  cvc::geometry geom;
  geom.points().resize(2);
  geom.points()[0][0] = 0;
  geom.points()[0][1] = 0;
  geom.points()[0][2] = 0;
  geom.points()[1][0] = 1;
  geom.points()[1][1] = 1;
  geom.points()[1][2] = 1;
  sceneGraph.addGraphics("geom1", geom);

  // Get combined bbox (should be 0-1 in local space)
  auto localBBox = nullNode->getCombinedBoundingBox();
  EXPECT_NEAR(localBBox[0], 0.0, 0.01);
  EXPECT_NEAR(localBBox[3], 1.0, 0.01);

  // Now translate the null node by (5, 0, 0)
  nullNode->setPosition(5.0, 0.0, 0.0);

  // Get combined bbox again - should still be 0-1 in null node's local space
  // because getCombinedBoundingBox returns bounds in this node's local space
  auto localBBox2 = nullNode->getCombinedBoundingBox();
  EXPECT_NEAR(localBBox2[0], 0.0, 0.01);
  EXPECT_NEAR(localBBox2[3], 1.0, 0.01);

  // But the world-space bounds should be 5-6
  // To get world bounds, we need to transform the local bbox by the node's world transform
  auto worldTransform = nullNode->getWorldTransform();
  double corner_in[4] = {localBBox2[3], localBBox2[4], localBBox2[5], 1.0}; // max corner
  double corner_out[4];
  worldTransform->MultiplyPoint(corner_in, corner_out);

  EXPECT_NEAR(corner_out[0], 6.0, 0.01); // 1.0 + 5.0 translation
  EXPECT_NEAR(corner_out[1], 1.0, 0.01);
  EXPECT_NEAR(corner_out[2], 1.0, 0.01);
}

// Test nested transforms with NullGraphicNode
TEST_F(NullGraphicNodeTest, NestedTransforms) {
  SceneGraph sceneGraph("test.scenegraph3");
  auto rootNull = std::dynamic_pointer_cast<NullGraphicNode>(sceneGraph.getGraphicsRoot());
  ASSERT_NE(rootNull, nullptr);

  // Create child null node
  auto childNull = std::make_shared<NullGraphicNode>(ctx, "test.child_null", "child");
  rootNull->addGraphicsChild(childNull);

  // Set transforms
  rootNull->setPosition(10.0, 0.0, 0.0);
  childNull->setPosition(5.0, 0.0, 0.0);

  // Add geometry to child null
  cvc::geometry geom;
  geom.points().resize(2);
  geom.points()[0][0] = 0;
  geom.points()[0][1] = 0;
  geom.points()[0][2] = 0;
  geom.points()[1][0] = 1;
  geom.points()[1][1] = 1;
  geom.points()[1][2] = 1;

  auto geomNode = childNull->addGraphicsChild<GeometryNode>("geom");
  geomNode->setGeometry(geom);
  geomNode->setPosition(2.0, 0.0, 0.0);

  // Geometry node's world transform should accumulate all parent transforms
  // Root: +10, Child: +5, Geom: +2 = +17 total
  auto worldTransform = geomNode->getWorldTransform();
  EXPECT_NEAR(worldTransform->GetElement(0, 3), 17.0, 0.01);
  EXPECT_NEAR(worldTransform->GetElement(1, 3), 0.0, 0.01);
  EXPECT_NEAR(worldTransform->GetElement(2, 3), 0.0, 0.01);
}

// Test syncBoundsToChildren respects child transforms
TEST_F(NullGraphicNodeTest, SyncBoundsRespectsChildTransforms) {
  SceneGraph sceneGraph("test.scenegraph4");
  auto nullNode = std::dynamic_pointer_cast<NullGraphicNode>(sceneGraph.getGraphicsRoot());
  ASSERT_NE(nullNode, nullptr);

  // Add two geometries at different positions
  cvc::geometry geom1;
  geom1.points().resize(2);
  geom1.points()[0][0] = 0;
  geom1.points()[0][1] = 0;
  geom1.points()[0][2] = 0;
  geom1.points()[1][0] = 1;
  geom1.points()[1][1] = 1;
  geom1.points()[1][2] = 1;

  auto geomNode1 = sceneGraph.addGraphics("geom1", geom1);
  geomNode1->setPosition(0.0, 0.0, 0.0);

  cvc::geometry geom2;
  geom2.points().resize(2);
  geom2.points()[0][0] = 0;
  geom2.points()[0][1] = 0;
  geom2.points()[0][2] = 0;
  geom2.points()[1][0] = 1;
  geom2.points()[1][1] = 1;
  geom2.points()[1][2] = 1;

  auto geomNode2 = sceneGraph.addGraphics("geom2", geom2);
  geomNode2->setPosition(10.0, 0.0, 0.0); // Offset by 10 in X

  // Sync bounds
  nullNode->setSyncBoundsWithChildren(true);
  nullNode->syncBoundsToChildren();

  // Bounds should encompass both geometries with their transforms
  // geom1: 0-1, geom2: 10-11, so combined should be 0-11
  auto bbox = nullNode->getBoundingBox();
  EXPECT_NEAR(bbox[0], 0.0, 0.01);
  EXPECT_NEAR(bbox[3], 11.0, 0.01);
}
