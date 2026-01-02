#include <gtest/gtest.h>
#include <volrover3/NullGraphicNode.h>
#include <volrover3/SceneGraph.h>
#include <volrover3/GeometryNode.h>
#include <cvc/geometry.h>
#include <cvc/state.h>

class NullGraphicNodeTest : public ::testing::Test {
protected:
    void SetUp() override {
        m_statePrefix = "test_null_graphic_" + std::to_string(testCounter++);
    }
    
    std::string m_statePrefix;
    static int testCounter;
};

int NullGraphicNodeTest::testCounter = 0;

// Test NullGraphicNode default construction
TEST_F(NullGraphicNodeTest, DefaultConstruction) {
    auto nullNode = std::make_shared<NullGraphicNode>("test_null");
    
    ASSERT_NE(nullNode, nullptr);
    EXPECT_EQ(nullNode->getName(), "test_null");
    
    // Check default bounding box
    auto bbox = nullNode->getBoundingBox();
    EXPECT_DOUBLE_EQ(bbox[0], -100.0);  // min x
    EXPECT_DOUBLE_EQ(bbox[1], -100.0);  // min y
    EXPECT_DOUBLE_EQ(bbox[2], -100.0);  // min z
    EXPECT_DOUBLE_EQ(bbox[3], 100.0);   // max x
    EXPECT_DOUBLE_EQ(bbox[4], 100.0);   // max y
    EXPECT_DOUBLE_EQ(bbox[5], 100.0);   // max z
}

// Test NullGraphicNode bounds can be modified
TEST_F(NullGraphicNodeTest, SetBoundsArray) {
    auto nullNode = std::make_shared<NullGraphicNode>("test_null");
    
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
    auto nullNode = std::make_shared<NullGraphicNode>("test_null");
    
    nullNode->setBounds(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
    
    auto bbox = nullNode->getBoundingBox();
    EXPECT_DOUBLE_EQ(bbox[0], 1.0);
    EXPECT_DOUBLE_EQ(bbox[1], 2.0);
    EXPECT_DOUBLE_EQ(bbox[2], 3.0);
    EXPECT_DOUBLE_EQ(bbox[3], 4.0);
    EXPECT_DOUBLE_EQ(bbox[4], 5.0);
    EXPECT_DOUBLE_EQ(bbox[5], 6.0);
}

// Test NullGraphicNode state synchronization
TEST_F(NullGraphicNodeTest, StateSynchronization) {
    auto nullNode = std::make_shared<NullGraphicNode>("test_null");
    
    // Set custom bounds
    nullNode->setBounds(-200, -150, -100, 200, 150, 100);
    nullNode->setShowBBox(true);
    
    // Sync to state
    cvc::state& testState = cvc::state::instance()(m_statePrefix);
    nullNode->syncToState(testState);
    
    // Verify state was written
    EXPECT_TRUE(testState("test_null").initialized());
    EXPECT_TRUE(testState("test_null")("bounds").initialized());
    EXPECT_TRUE(testState("test_null")("bounds")("min_x").initialized());
    
    // Create new node and sync from state
    auto nullNode2 = std::make_shared<NullGraphicNode>("test_null");
    nullNode2->syncFromState(testState);
    
    // Verify bounds were loaded
    auto bbox = nullNode2->getBoundingBox();
    EXPECT_DOUBLE_EQ(bbox[0], -200.0);
    EXPECT_DOUBLE_EQ(bbox[1], -150.0);
    EXPECT_DOUBLE_EQ(bbox[2], -100.0);
    EXPECT_DOUBLE_EQ(bbox[3], 200.0);
    EXPECT_DOUBLE_EQ(bbox[4], 150.0);
    EXPECT_DOUBLE_EQ(bbox[5], 100.0);
    EXPECT_TRUE(nullNode2->getShowBBox());
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
    
    // Should have no children initially
    auto children = nullNode->getGraphicsChildren();
    EXPECT_EQ(children.size(), 0);
}

// Test geometry added as child of null graphic root
TEST_F(NullGraphicNodeTest, NullGraphicRemovedOnGeometryAdd) {
    SceneGraph sceneGraph(m_statePrefix);
    
    // Graphics root is the null graphic, starts empty
    auto nullNode = std::dynamic_pointer_cast<NullGraphicNode>(sceneGraph.getGraphicsRoot());
    ASSERT_NE(nullNode, nullptr);
    auto children = nullNode->getGraphicsChildren();
    ASSERT_EQ(children.size(), 0);
    
    // Add geometry
    cvc::geometry geom;
    geom.points().resize(3);
    geom.points()[0][0] = 0; geom.points()[0][1] = 0; geom.points()[0][2] = 0;
    geom.points()[1][0] = 1; geom.points()[1][1] = 0; geom.points()[1][2] = 0;
    geom.points()[2][0] = 0; geom.points()[2][1] = 1; geom.points()[2][2] = 0;
    
    sceneGraph.addGraphics("test_geom", geom);
    
    // Null graphic is still the root, but now has one child
    children = nullNode->getGraphicsChildren();
    ASSERT_EQ(children.size(), 1);
    
    // Should be the geometry we added (not the null graphic)
    auto geomNode = std::dynamic_pointer_cast<GeometryNode>(children[0]);
    ASSERT_NE(geomNode, nullptr);
    EXPECT_EQ(geomNode->getName(), "test_geom");
}

// Test null graphic root remains when graphics removed
TEST_F(NullGraphicNodeTest, NullGraphicRestoredOnRemove) {
    SceneGraph sceneGraph(m_statePrefix);
    
    // Add geometry
    cvc::geometry geom;
    geom.points().resize(3);
    geom.points()[0][0] = 0; geom.points()[0][1] = 0; geom.points()[0][2] = 0;
    geom.points()[1][0] = 1; geom.points()[1][1] = 0; geom.points()[1][2] = 0;
    geom.points()[2][0] = 0; geom.points()[2][1] = 1; geom.points()[2][2] = 0;
    sceneGraph.addGraphics("test_geom", geom);
    
    // Graphics root (null graphic) has one child
    auto nullNode = std::dynamic_pointer_cast<NullGraphicNode>(sceneGraph.getGraphicsRoot());
    ASSERT_NE(nullNode, nullptr);
    auto children = nullNode->getGraphicsChildren();
    ASSERT_EQ(children.size(), 1);
    
    // Remove the geometry
    sceneGraph.removeGraphics("test_geom");
    
    // Graphics root (null graphic) is back to empty
    children = nullNode->getGraphicsChildren();
    EXPECT_EQ(children.size(), 0);
}

// Test null graphic not counted as real graphic
TEST_F(NullGraphicNodeTest, NullGraphicNotInGraphicsMap) {
    SceneGraph sceneGraph(m_statePrefix);
    
    // Graphics root IS the null graphic, starts with no children
    auto nullNode = std::dynamic_pointer_cast<NullGraphicNode>(sceneGraph.getGraphicsRoot());
    ASSERT_NE(nullNode, nullptr);
    auto children = nullNode->getGraphicsChildren();
    ASSERT_EQ(children.size(), 0);
    
    // Verify null graphic is not accessible via getGraphics (it's the root, not a child)
    auto nullFromMap = sceneGraph.getGraphics("root");
    EXPECT_EQ(nullFromMap, nullptr);
    
    // Add real graphic
    cvc::geometry geom;
    geom.points().resize(3);
    geom.points()[0][0] = 0; geom.points()[0][1] = 0; geom.points()[0][2] = 0;
    geom.points()[1][0] = 1; geom.points()[1][1] = 0; geom.points()[1][2] = 0;
    geom.points()[2][0] = 0; geom.points()[2][1] = 1; geom.points()[2][2] = 0;
    sceneGraph.addGraphics("test_geom", geom);
    
    // Now should be able to get the real graphic
    auto realGraphic = sceneGraph.getGraphics("test_geom");
    ASSERT_NE(realGraphic, nullptr);
    EXPECT_EQ(realGraphic->getName(), "test_geom");
}

// Test large custom bounds
TEST_F(NullGraphicNodeTest, LargeCustomBounds) {
    auto nullNode = std::make_shared<NullGraphicNode>("test_null");
    
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
    auto nullNode = std::make_shared<NullGraphicNode>("test_null");
    
    nullNode->setBounds(-500.0, 100.0, -200.0, 300.0, 1000.0, 500.0);
    
    auto bbox = nullNode->getBoundingBox();
    EXPECT_DOUBLE_EQ(bbox[0], -500.0);
    EXPECT_DOUBLE_EQ(bbox[1], 100.0);
    EXPECT_DOUBLE_EQ(bbox[2], -200.0);
    EXPECT_DOUBLE_EQ(bbox[3], 300.0);
    EXPECT_DOUBLE_EQ(bbox[4], 1000.0);
    EXPECT_DOUBLE_EQ(bbox[5], 500.0);
}
