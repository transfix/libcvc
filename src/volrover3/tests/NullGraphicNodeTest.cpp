#include <gtest/gtest.h>
#include <volrover3/NullGraphicNode.h>
#include <volrover3/SceneGraph.h>
#include <volrover3/GeometryNode.h>
#include <cvc/geometry.h>
#include <cvc/state.h>
#include <cvc/state_object.h>

class NullGraphicNodeTest : public ::testing::Test {
protected:
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
    auto nullNode = std::make_shared<NullGraphicNode>("test.null", "test_null");
    
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
    geom.points()[0][0] = 0; geom.points()[0][1] = 0; geom.points()[0][2] = 0;
    geom.points()[1][0] = 1; geom.points()[1][1] = 0; geom.points()[1][2] = 0;
    geom.points()[2][0] = 0; geom.points()[2][1] = 1; geom.points()[2][2] = 0;
    
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
    geom.points()[0][0] = 0; geom.points()[0][1] = 0; geom.points()[0][2] = 0;
    geom.points()[1][0] = 1; geom.points()[1][1] = 0; geom.points()[1][2] = 0;
    geom.points()[2][0] = 0; geom.points()[2][1] = 1; geom.points()[2][2] = 0;
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
