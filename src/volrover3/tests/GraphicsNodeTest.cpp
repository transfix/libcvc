#include <gtest/gtest.h>
#include <volrover3/GraphicsNode.h>
#include <volrover3/SceneGraph.h>
#include <cvc/geometry.h>
#include <cvc/state.h>
#include <vtkMatrix4x4.h>
#include <cmath>

class GraphicsNodeTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a simple test geometry (triangle)
        testGeom.points().push_back({0.0, 0.0, 0.0});
        testGeom.points().push_back({1.0, 0.0, 0.0});
        testGeom.points().push_back({0.0, 1.0, 0.0});
        
        testGeom.tris().push_back({0, 1, 2});
    }
    
    void TearDown() override {
    }
    
    cvc::geometry testGeom;
};

// Test basic GraphicsNode creation
TEST_F(GraphicsNodeTest, Creation) {
    GraphicsNode node("test_node");
    EXPECT_EQ(node.getName(), "test_node");
    EXPECT_FALSE(node.hasGeometry());
}

// Test setting geometry
TEST_F(GraphicsNodeTest, SetGeometry) {
    GraphicsNode node("test_node");
    node.setGeometry(testGeom);
    
    EXPECT_TRUE(node.hasGeometry());
    ASSERT_NE(node.getGeometry(), nullptr);
    EXPECT_EQ(node.getGeometry()->num_points(), 3);
    EXPECT_EQ(node.getGeometry()->num_tris(), 1);
}

// Test geometry storage in node
TEST_F(GraphicsNodeTest, GeometryRetrieval) {
    GraphicsNode node("test_node");
    node.setGeometry(testGeom);
    
    const cvc::geometry* geom = node.getGeometry();
    ASSERT_NE(geom, nullptr);
    
    // Verify geometry data is correct
    EXPECT_EQ(geom->points().size(), 3);
    EXPECT_EQ(geom->tris().size(), 1);
    EXPECT_DOUBLE_EQ(geom->points()[0][0], 0.0);
    EXPECT_DOUBLE_EQ(geom->points()[1][0], 1.0);
}

// Test default transform is identity
TEST_F(GraphicsNodeTest, DefaultTransformIsIdentity) {
    GraphicsNode node("test_node");
    vtkMatrix4x4* transform = node.getTransform();
    
    ASSERT_NE(transform, nullptr);
    
    // Check identity matrix
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (i == j) {
                EXPECT_DOUBLE_EQ(transform->GetElement(i, j), 1.0);
            } else {
                EXPECT_DOUBLE_EQ(transform->GetElement(i, j), 0.0);
            }
        }
    }
}

// Test setting position
TEST_F(GraphicsNodeTest, SetPosition) {
    GraphicsNode node("test_node");
    node.setPosition(1.0, 2.0, 3.0);
    
    vtkMatrix4x4* transform = node.getTransform();
    EXPECT_DOUBLE_EQ(transform->GetElement(0, 3), 1.0);
    EXPECT_DOUBLE_EQ(transform->GetElement(1, 3), 2.0);
    EXPECT_DOUBLE_EQ(transform->GetElement(2, 3), 3.0);
}

// Test setting scale
TEST_F(GraphicsNodeTest, SetScale) {
    GraphicsNode node("test_node");
    node.setScale(2.0, 3.0, 4.0);
    
    vtkMatrix4x4* transform = node.getTransform();
    EXPECT_DOUBLE_EQ(transform->GetElement(0, 0), 2.0);
    EXPECT_DOUBLE_EQ(transform->GetElement(1, 1), 3.0);
    EXPECT_DOUBLE_EQ(transform->GetElement(2, 2), 4.0);
}

// Test reset transform
TEST_F(GraphicsNodeTest, ResetTransform) {
    GraphicsNode node("test_node");
    node.setPosition(1.0, 2.0, 3.0);
    node.resetTransform();
    
    vtkMatrix4x4* transform = node.getTransform();
    
    // Should be identity again
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (i == j) {
                EXPECT_DOUBLE_EQ(transform->GetElement(i, j), 1.0);
            } else {
                EXPECT_DOUBLE_EQ(transform->GetElement(i, j), 0.0);
            }
        }
    }
}

// Test metadata storage and retrieval
TEST_F(GraphicsNodeTest, MetadataStorage) {
    GraphicsNode node("test_node");
    
    node.setMetadata("test_string", std::string("hello"));
    node.setMetadata("test_int", 42);
    node.setMetadata("test_double", 3.14);
    node.setMetadata("test_bool", true);
    
    EXPECT_TRUE(node.hasMetadata("test_string"));
    EXPECT_TRUE(node.hasMetadata("test_int"));
    EXPECT_TRUE(node.hasMetadata("test_double"));
    EXPECT_TRUE(node.hasMetadata("test_bool"));
    EXPECT_FALSE(node.hasMetadata("nonexistent"));
    
    // Retrieve and verify
    EXPECT_EQ(std::any_cast<std::string>(node.getMetadata("test_string")), "hello");
    EXPECT_EQ(std::any_cast<int>(node.getMetadata("test_int")), 42);
    EXPECT_DOUBLE_EQ(std::any_cast<double>(node.getMetadata("test_double")), 3.14);
    EXPECT_EQ(std::any_cast<bool>(node.getMetadata("test_bool")), true);
}

// Test default visible metadata
TEST_F(GraphicsNodeTest, DefaultVisibleMetadata) {
    GraphicsNode node("test_node");
    
    EXPECT_TRUE(node.hasMetadata("visible"));
    EXPECT_EQ(std::any_cast<bool>(node.getMetadata("visible")), true);
}

// Test setVisible updates metadata
TEST_F(GraphicsNodeTest, SetVisibleUpdatesMetadata) {
    GraphicsNode node("test_node");
    
    node.setVisible(false);
    EXPECT_EQ(std::any_cast<bool>(node.getMetadata("visible")), false);
    
    node.setVisible(true);
    EXPECT_EQ(std::any_cast<bool>(node.getMetadata("visible")), true);
}

// Test type metadata for geometry
TEST_F(GraphicsNodeTest, TypeMetadata) {
    GraphicsNode node("test_node");
    node.setMetadata("type", std::string("geometry"));
    
    EXPECT_TRUE(node.hasMetadata("type"));
    EXPECT_EQ(std::any_cast<std::string>(node.getMetadata("type")), "geometry");
}

// Test hierarchical structure - adding children
TEST_F(GraphicsNodeTest, AddChild) {
    auto parent = std::make_shared<GraphicsNode>("parent");
    auto child1 = std::make_shared<GraphicsNode>("child1");
    auto child2 = std::make_shared<GraphicsNode>("child2");
    
    parent->addGraphicsChild(child1);
    parent->addGraphicsChild(child2);
    
    EXPECT_EQ(parent->getGraphicsChildren().size(), 2);
}

// Test finding child by name
TEST_F(GraphicsNodeTest, FindChildByName) {
    auto parent = std::make_shared<GraphicsNode>("parent");
    auto child1 = std::make_shared<GraphicsNode>("child1");
    auto child2 = std::make_shared<GraphicsNode>("child2");
    
    parent->addGraphicsChild(child1);
    parent->addGraphicsChild(child2);
    
    auto found = parent->findChildByName("child1");
    ASSERT_NE(found, nullptr);
    EXPECT_EQ(found->getName(), "child1");
    
    auto notFound = parent->findChildByName("nonexistent");
    EXPECT_EQ(notFound, nullptr);
}

// Test removing children
TEST_F(GraphicsNodeTest, RemoveChild) {
    auto parent = std::make_shared<GraphicsNode>("parent");
    auto child1 = std::make_shared<GraphicsNode>("child1");
    auto child2 = std::make_shared<GraphicsNode>("child2");
    
    parent->addGraphicsChild(child1);
    parent->addGraphicsChild(child2);
    
    parent->removeGraphicsChild(child1);
    EXPECT_EQ(parent->getGraphicsChildren().size(), 1);
    
    auto found = parent->findChildByName("child1");
    EXPECT_EQ(found, nullptr);
}

// Test state synchronization - syncToState
TEST_F(GraphicsNodeTest, SyncToState) {
    GraphicsNode node("test_node");
    node.setGeometry(testGeom);
    node.setPosition(1.0, 2.0, 3.0);
    node.setMetadata("filename", std::string("test.obj"));
    node.setMetadata("num_vertices", 3);
    node.setMetadata("type", std::string("geometry"));
    
    cvc::state& testState = cvc::state::instance()("graphics_test");
    node.syncToState(testState);
    
    // Verify state was created
    EXPECT_TRUE(testState("test_node").initialized());
    EXPECT_TRUE(testState("test_node")("transform").initialized());
    EXPECT_TRUE(testState("test_node")("metadata").initialized());
    EXPECT_TRUE(testState("test_node")("metadata")("filename").initialized());
    EXPECT_TRUE(testState("test_node")("metadata")("num_vertices").initialized());
    
    // Verify metadata values
    EXPECT_EQ(testState("test_node")("metadata")("filename").value(), "test.obj");
    EXPECT_EQ(testState("test_node")("metadata")("num_vertices").value(), "3");
    
    // Clean up
    testState.reset();
}

// Test state synchronization - syncFromState
TEST_F(GraphicsNodeTest, SyncFromState) {
    // Create a state with data
    cvc::state& testState = cvc::state::instance()("graphics_test2");
    testState("test_node")("metadata")("filename").value("loaded.obj");
    testState("test_node")("metadata")("num_vertices").value("5");
    testState("test_node")("metadata")("visible").value("false");
    testState("test_node")("metadata")("type").value("geometry");
    
    // Create node and sync from state
    GraphicsNode node("test_node");
    node.syncFromState(testState);
    
    // Verify metadata was loaded
    EXPECT_TRUE(node.hasMetadata("filename"));
    EXPECT_EQ(std::any_cast<std::string>(node.getMetadata("filename")), "loaded.obj");
    EXPECT_TRUE(node.hasMetadata("num_vertices"));
    EXPECT_EQ(std::any_cast<std::string>(node.getMetadata("num_vertices")), "5");
    EXPECT_TRUE(node.hasMetadata("visible"));
    EXPECT_EQ(std::any_cast<bool>(node.getMetadata("visible")), false);
    
    // Clean up
    testState.reset();
}

// Test read-only metadata flag is set
TEST_F(GraphicsNodeTest, ReadOnlyMetadataFlag) {
    GraphicsNode node("test_node");
    node.setMetadata("filename", std::string("test.obj"));
    node.setMetadata("num_vertices", 100);
    node.setMetadata("num_triangles", 50);
    node.setMetadata("type", std::string("geometry"));
    node.setMetadata("custom", std::string("editable"));
    
    cvc::state& testState = cvc::state::instance()("graphics_test3");
    node.syncToState(testState);
    
    // Check read-only flags are set correctly
    EXPECT_TRUE(testState("test_node")("metadata")("filename").readOnly());
    EXPECT_TRUE(testState("test_node")("metadata")("num_vertices").readOnly());
    EXPECT_TRUE(testState("test_node")("metadata")("num_triangles").readOnly());
    EXPECT_TRUE(testState("test_node")("metadata")("type").readOnly());
    EXPECT_FALSE(testState("test_node")("metadata")("custom").readOnly());
    
    // Clean up
    testState.reset();
}

// Test that modifying read-only state throws exception
TEST_F(GraphicsNodeTest, ReadOnlyStateThrowsException) {
    GraphicsNode node("test_node");
    node.setMetadata("filename", std::string("test.obj"));
    
    cvc::state& testState = cvc::state::instance()("graphics_test_readonly");
    node.syncToState(testState);
    
    // Verify filename is read-only
    EXPECT_TRUE(testState("test_node")("metadata")("filename").readOnly());
    
    // Attempting to modify should throw
    EXPECT_THROW({
        testState("test_node")("metadata")("filename").value("modified.obj");
    }, cvc::read_only_error);
    
    // Original value should be unchanged
    EXPECT_EQ(testState("test_node")("metadata")("filename").value(), "test.obj");
    
    // Clean up
    testState.reset();
}

// Test geometry stored in state data field
TEST_F(GraphicsNodeTest, GeometryInStateData) {
    GraphicsNode node("test_node");
    node.setGeometry(testGeom);
    
    cvc::state& testState = cvc::state::instance()("graphics_test4");
    node.syncToState(testState);
    
    // Verify geometry is in state data
    EXPECT_TRUE(testState("test_node").isData<cvc::geometry>());
    
    // Try to retrieve geometry from state data
    try {
        const cvc::geometry& storedGeom = testState("test_node").data<cvc::geometry>();
        EXPECT_EQ(storedGeom.num_points(), 3);
        EXPECT_EQ(storedGeom.num_tris(), 1);
    } catch (...) {
        FAIL() << "Failed to retrieve geometry from state data";
    }
    
    // Clean up
    testState.reset();
}

// Test hierarchical state sync with children
TEST_F(GraphicsNodeTest, HierarchicalStateSyncToState) {
    auto parent = std::make_shared<GraphicsNode>("parent");
    auto child = std::make_shared<GraphicsNode>("child");
    
    parent->addGraphicsChild(child);
    parent->setMetadata("type", std::string("geometry"));
    child->setMetadata("type", std::string("geometry"));
    
    cvc::state& testState = cvc::state::instance()("graphics_test5");
    parent->syncToState(testState);
    
    // Verify parent and child states exist
    EXPECT_TRUE(testState("parent").initialized());
    EXPECT_TRUE(testState("parent")("children").initialized());
    EXPECT_TRUE(testState("parent")("children")("child").initialized());
    
    // Clean up
    testState.reset();
}

// Test SceneGraph graphics management
TEST_F(GraphicsNodeTest, SceneGraphAddGraphics) {
    SceneGraph sceneGraph;
    
    auto node = sceneGraph.addGraphics("test_graphics", testGeom);
    
    ASSERT_NE(node, nullptr);
    EXPECT_EQ(node->getName(), "test_graphics");
    EXPECT_TRUE(node->hasGeometry());
    
    // Verify we can retrieve it
    auto retrieved = sceneGraph.getGraphics("test_graphics");
    EXPECT_EQ(retrieved, node);
}

// Test SceneGraph empty graphics node
TEST_F(GraphicsNodeTest, SceneGraphAddEmptyGraphics) {
    SceneGraph sceneGraph;
    
    auto node = sceneGraph.addGraphics("empty_node");
    
    ASSERT_NE(node, nullptr);
    EXPECT_EQ(node->getName(), "empty_node");
    EXPECT_FALSE(node->hasGeometry());
}

// Test SceneGraph remove graphics
TEST_F(GraphicsNodeTest, SceneGraphRemoveGraphics) {
    SceneGraph sceneGraph;
    
    sceneGraph.addGraphics("test_graphics", testGeom);
    sceneGraph.removeGraphics("test_graphics");
    
    auto retrieved = sceneGraph.getGraphics("test_graphics");
    EXPECT_EQ(retrieved, nullptr);
}

// Test SceneGraph graphics root
TEST_F(GraphicsNodeTest, SceneGraphGraphicsRoot) {
    SceneGraph sceneGraph;
    
    auto root = sceneGraph.getGraphicsRoot();
    ASSERT_NE(root, nullptr);
    EXPECT_EQ(root->getName(), "graphics_root");
}

// Test SceneGraph register graphics
TEST_F(GraphicsNodeTest, SceneGraphRegisterGraphics) {
    SceneGraph sceneGraph;
    
    auto customNode = std::make_shared<GraphicsNode>("custom");
    sceneGraph.registerGraphics("custom", customNode);
    
    auto retrieved = sceneGraph.getGraphics("custom");
    EXPECT_EQ(retrieved, customNode);
}

// Test SceneGraph sync to state
TEST_F(GraphicsNodeTest, SceneGraphSyncGraphicsToState) {
    SceneGraph sceneGraph;
    
    sceneGraph.addGraphics("node1", testGeom);
    sceneGraph.addGraphics("node2", testGeom);
    
    sceneGraph.syncGraphicsToState();
    
    // Verify state tree was updated
    cvc::state& volroverState = cvc::state::instance()("volrover3");
    EXPECT_TRUE(volroverState("graphics").initialized());
    
    // Clean up
    volroverState.reset();
}

// Test SceneGraph compute bounds
TEST_F(GraphicsNodeTest, SceneGraphComputeBounds) {
    SceneGraph sceneGraph;
    
    // Create geometry with known bounds
    cvc::geometry geom1;
    geom1.points().push_back({0.0, 0.0, 0.0});
    geom1.points().push_back({1.0, 1.0, 1.0});
    
    cvc::geometry geom2;
    geom2.points().push_back({-1.0, -1.0, -1.0});
    geom2.points().push_back({2.0, 2.0, 2.0});
    
    sceneGraph.addGraphics("geom1", geom1);
    sceneGraph.addGraphics("geom2", geom2);
    
    cvc::bounding_box bounds = sceneGraph.computeGraphicsBounds();
    
    // Bounds should encompass both geometries
    EXPECT_LE(bounds[0], -1.0);
    EXPECT_LE(bounds[1], -1.0);
    EXPECT_LE(bounds[2], -1.0);
    EXPECT_GE(bounds[3], 2.0);
    EXPECT_GE(bounds[4], 2.0);
    EXPECT_GE(bounds[5], 2.0);
}

// Test SceneGraph compute bounds with no graphics
TEST_F(GraphicsNodeTest, SceneGraphComputeBoundsEmpty) {
    SceneGraph sceneGraph;
    
    cvc::bounding_box bounds = sceneGraph.computeGraphicsBounds();
    
    // Should return invalid/empty bounds
    EXPECT_TRUE(bounds[0] > bounds[3] || bounds[0] == 0.0);
}

// Test world transform calculation for nested nodes
TEST_F(GraphicsNodeTest, WorldTransformHierarchy) {
    auto parent = std::make_shared<GraphicsNode>("parent");
    auto child = std::make_shared<GraphicsNode>("child");
    
    // Set parent position
    parent->setPosition(10.0, 0.0, 0.0);
    
    // Add child and set its position
    parent->addGraphicsChild(child);
    child->setPosition(5.0, 0.0, 0.0);
    
    // Get child's world transform
    vtkSmartPointer<vtkMatrix4x4> worldTransform = child->getWorldTransform();
    
    // Child's world position should be parent + child = 15.0, 0.0, 0.0
    EXPECT_DOUBLE_EQ(worldTransform->GetElement(0, 3), 15.0);
    EXPECT_DOUBLE_EQ(worldTransform->GetElement(1, 3), 0.0);
    EXPECT_DOUBLE_EQ(worldTransform->GetElement(2, 3), 0.0);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
