#include <gtest/gtest.h>
#include <volrover3/GeometryNode.h>
#include <volrover3/SceneGraph.h>
#include <cvc/geometry.h>
#include <cvc/state.h>
#include <cvc/state_object.h>
#include <vtkMatrix4x4.h>
#include <cmath>

class GraphicsNodeTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Disable threading for state_object to avoid destruction race conditions
        cvc::state_object<SceneNode>::setUseThreading(false);
        
        // Each test uses its own state subtree
        m_statePrefix = "graphics_test_" + std::to_string(testCounter++);
        
        // Create a simple test geometry (triangle)
        testGeom.points().push_back({0.0, 0.0, 0.0});
        testGeom.points().push_back({1.0, 0.0, 0.0});
        testGeom.points().push_back({0.0, 1.0, 0.0});
        
        testGeom.tris().push_back({0, 1, 2});
    }
    
    void TearDown() override {
        // disconnectState() in SceneNode destructor prevents callbacks during destruction
    }
    
    static int testCounter;
    std::string m_statePrefix;
    cvc::geometry testGeom;
};

int GraphicsNodeTest::testCounter = 0;

// Test basic GeometryNode creation (GraphicsNode is abstract)
TEST_F(GraphicsNodeTest, Creation) {
    GeometryNode node("test", "test_node");
    EXPECT_EQ(node.getName(), "test_node");
    EXPECT_FALSE(node.hasGeometry());
}

// Test setting geometry
TEST_F(GraphicsNodeTest, SetGeometry) {
    GeometryNode node("test", "test_node");
    node.setGeometry(testGeom);
    
    EXPECT_TRUE(node.hasGeometry());
    ASSERT_NE(node.getGeometry(), nullptr);
    EXPECT_EQ(node.getGeometry()->num_points(), 3);
    EXPECT_EQ(node.getGeometry()->num_tris(), 1);
}

// Test geometry storage in node
TEST_F(GraphicsNodeTest, GeometryRetrieval) {
    GeometryNode node("test", "test_node");
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
    GeometryNode node("test", "test_node");
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
    GeometryNode node("test", "test_node");
    node.setPosition(1.0, 2.0, 3.0);
    
    vtkMatrix4x4* transform = node.getTransform();
    EXPECT_DOUBLE_EQ(transform->GetElement(0, 3), 1.0);
    EXPECT_DOUBLE_EQ(transform->GetElement(1, 3), 2.0);
    EXPECT_DOUBLE_EQ(transform->GetElement(2, 3), 3.0);
}

// Test setting scale
TEST_F(GraphicsNodeTest, SetScale) {
    GeometryNode node("test", "test_node");
    node.setScale(2.0, 3.0, 4.0);
    
    vtkMatrix4x4* transform = node.getTransform();
    EXPECT_DOUBLE_EQ(transform->GetElement(0, 0), 2.0);
    EXPECT_DOUBLE_EQ(transform->GetElement(1, 1), 3.0);
    EXPECT_DOUBLE_EQ(transform->GetElement(2, 2), 4.0);
}

// Test reset transform
TEST_F(GraphicsNodeTest, ResetTransform) {
    GeometryNode node("test", "test_node");
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
    GeometryNode node("test", "test_node");
    
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
    GeometryNode node("test", "test_node");
    
    EXPECT_TRUE(node.isVisible());
}

// Test setVisible updates metadata
TEST_F(GraphicsNodeTest, SetVisibleUpdatesMetadata) {
    GeometryNode node("test", "test_node");
    
    node.setVisible(false);
    EXPECT_FALSE(node.isVisible());
    
    node.setVisible(true);
    EXPECT_TRUE(node.isVisible());
}

// Test type metadata for geometry
TEST_F(GraphicsNodeTest, TypeMetadata) {
    GeometryNode node("test", "test_node");
    node.setMetadata("type", std::string("geometry"));
    
    EXPECT_TRUE(node.hasMetadata("type"));
    EXPECT_EQ(std::any_cast<std::string>(node.getMetadata("type")), "geometry");
}

// Test hierarchical structure - adding children
TEST_F(GraphicsNodeTest, AddChild) {
    auto parent = std::make_shared<GeometryNode>("test", "parent");
    auto child1 = std::make_shared<GeometryNode>("test", "child1");
    auto child2 = std::make_shared<GeometryNode>("test", "child2");
    
    parent->addGraphicsChild(child1);
    parent->addGraphicsChild(child2);
    
    EXPECT_EQ(parent->getGraphicsChildren().size(), 2);
}

// Test finding child by name
TEST_F(GraphicsNodeTest, FindChildByName) {
    auto parent = std::make_shared<GeometryNode>("test", "parent");
    auto child1 = std::make_shared<GeometryNode>("test", "child1");
    auto child2 = std::make_shared<GeometryNode>("test", "child2");
    
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
    auto parent = std::make_shared<GeometryNode>("test", "parent");
    auto child1 = std::make_shared<GeometryNode>("test", "child1");
    auto child2 = std::make_shared<GeometryNode>("test", "child2");
    
    parent->addGraphicsChild(child1);
    parent->addGraphicsChild(child2);
    
    parent->removeGraphicsChild(child1);
    EXPECT_EQ(parent->getGraphicsChildren().size(), 1);
    
    auto found = parent->findChildByName("child1");
    EXPECT_EQ(found, nullptr);
}

// Test SceneGraph graphics management
TEST_F(GraphicsNodeTest, SceneGraphAddGraphics) {
    SceneGraph sceneGraph(m_statePrefix);
    
    auto node = sceneGraph.addGraphics("test_graphics", testGeom);
    
    ASSERT_NE(node, nullptr);
    EXPECT_EQ(node->getName(), "test_graphics");
    
    // Cast to GeometryNode to check geometry
    auto geomNode = std::dynamic_pointer_cast<GeometryNode>(node);
    ASSERT_NE(geomNode, nullptr);
    EXPECT_TRUE(geomNode->hasGeometry());
    
    // Verify we can retrieve it
    auto retrieved = sceneGraph.getGraphics("test_graphics");
    EXPECT_EQ(retrieved, node);
}

// Test SceneGraph empty graphics node
TEST_F(GraphicsNodeTest, SceneGraphAddEmptyGraphics) {
    SceneGraph sceneGraph(m_statePrefix);
    
    auto node = sceneGraph.addGraphics("empty_node");
    
    ASSERT_NE(node, nullptr);
    EXPECT_EQ(node->getName(), "empty_node");
    
    // Cast to GeometryNode to check geometry
    auto geomNode = std::dynamic_pointer_cast<GeometryNode>(node);
    ASSERT_NE(geomNode, nullptr);
    EXPECT_FALSE(geomNode->hasGeometry());
}

// Test SceneGraph remove graphics
TEST_F(GraphicsNodeTest, SceneGraphRemoveGraphics) {
    SceneGraph sceneGraph(m_statePrefix);
    
    sceneGraph.addGraphics("test_graphics", testGeom);
    sceneGraph.removeGraphics("test_graphics");
    
    auto retrieved = sceneGraph.getGraphics("test_graphics");
    EXPECT_EQ(retrieved, nullptr);
}

// Test SceneGraph graphics root
TEST_F(GraphicsNodeTest, SceneGraphGraphicsRoot) {
    SceneGraph sceneGraph(m_statePrefix);
    
    auto root = sceneGraph.getGraphicsRoot();
    ASSERT_NE(root, nullptr);
    // Graphics root is now the NullGraphicNode named "root"
    EXPECT_EQ(root->getName(), "root");
}

// Test SceneGraph register graphics
TEST_F(GraphicsNodeTest, SceneGraphRegisterGraphics) {
    SceneGraph sceneGraph(m_statePrefix);
    
    auto customNode = std::make_shared<GeometryNode>("test", "custom");
    sceneGraph.registerGraphics("custom", customNode);
    
    auto retrieved = sceneGraph.getGraphics("custom");
    EXPECT_EQ(retrieved, customNode);
}
// Test SceneGraph compute bounds
TEST_F(GraphicsNodeTest, SceneGraphComputeBounds) {
    SceneGraph sceneGraph(m_statePrefix);
    
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
    SceneGraph sceneGraph(m_statePrefix);
    
    cvc::bounding_box bounds = sceneGraph.computeGraphicsBounds();
    
    // Should return invalid/empty bounds
    EXPECT_TRUE(bounds[0] > bounds[3] || bounds[0] == 0.0);
}

// Test world transform calculation for nested nodes
TEST_F(GraphicsNodeTest, WorldTransformHierarchy) {
    auto parent = std::make_shared<GeometryNode>("test.parent", "parent");
    auto child = std::make_shared<GeometryNode>("test.child", "child");
    
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

// Test that metadata is computed from geometry
TEST_F(GraphicsNodeTest, MetadataFromGeometry) {
    GeometryNode node("test", "test_node");
    node.setGeometry(testGeom);
    
    // Check that basic stats are computed
    EXPECT_TRUE(node.hasMetadata("num_vertices"));
    EXPECT_TRUE(node.hasMetadata("num_triangles"));
    EXPECT_TRUE(node.hasMetadata("type"));
    
    // Verify values
    int numVerts = std::any_cast<int>(node.getMetadata("num_vertices"));
    int numTris = std::any_cast<int>(node.getMetadata("num_triangles"));
    
    EXPECT_EQ(numVerts, 3);
    EXPECT_EQ(numTris, 1);
    
    std::string type = std::any_cast<std::string>(node.getMetadata("type"));
    EXPECT_EQ(type, "triangle_mesh");
}

// Test that bounding box metadata is computed
TEST_F(GraphicsNodeTest, BoundingBoxMetadata) {
    GeometryNode node("test", "test_node");
    node.setGeometry(testGeom);
    
    // Check bounding box metadata exists
    EXPECT_TRUE(node.hasMetadata("bbox_min_x"));
    EXPECT_TRUE(node.hasMetadata("bbox_min_y"));
    EXPECT_TRUE(node.hasMetadata("bbox_min_z"));
    EXPECT_TRUE(node.hasMetadata("bbox_max_x"));
    EXPECT_TRUE(node.hasMetadata("bbox_max_y"));
    EXPECT_TRUE(node.hasMetadata("bbox_max_z"));
    
    // Verify bounding box values
    double minX = std::any_cast<double>(node.getMetadata("bbox_min_x"));
    double minY = std::any_cast<double>(node.getMetadata("bbox_min_y"));
    double maxX = std::any_cast<double>(node.getMetadata("bbox_max_x"));
    double maxY = std::any_cast<double>(node.getMetadata("bbox_max_y"));
    
    EXPECT_DOUBLE_EQ(minX, 0.0);
    EXPECT_DOUBLE_EQ(minY, 0.0);
    EXPECT_DOUBLE_EQ(maxX, 1.0);
    EXPECT_DOUBLE_EQ(maxY, 1.0);
}

// Test that extent metadata is computed
TEST_F(GraphicsNodeTest, ExtentMetadata) {
    GeometryNode node("test", "test_node");
    node.setGeometry(testGeom);
    
    // Check extent metadata exists
    EXPECT_TRUE(node.hasMetadata("extent_x"));
    EXPECT_TRUE(node.hasMetadata("extent_y"));
    EXPECT_TRUE(node.hasMetadata("extent_z"));
    
    // Verify extent values
    double extentX = std::any_cast<double>(node.getMetadata("extent_x"));
    double extentY = std::any_cast<double>(node.getMetadata("extent_y"));
    double extentZ = std::any_cast<double>(node.getMetadata("extent_z"));
    
    EXPECT_DOUBLE_EQ(extentX, 1.0);
    EXPECT_DOUBLE_EQ(extentY, 1.0);
    EXPECT_DOUBLE_EQ(extentZ, 0.0);
}

// Test that center metadata is computed
TEST_F(GraphicsNodeTest, CenterMetadata) {
    GeometryNode node("test", "test_node");
    node.setGeometry(testGeom);
    
    // Check center metadata exists
    EXPECT_TRUE(node.hasMetadata("center_x"));
    EXPECT_TRUE(node.hasMetadata("center_y"));
    EXPECT_TRUE(node.hasMetadata("center_z"));
    
    // Verify center values
    double centerX = std::any_cast<double>(node.getMetadata("center_x"));
    double centerY = std::any_cast<double>(node.getMetadata("center_y"));
    double centerZ = std::any_cast<double>(node.getMetadata("center_z"));
    
    EXPECT_DOUBLE_EQ(centerX, 0.5);
    EXPECT_DOUBLE_EQ(centerY, 0.5);
    EXPECT_DOUBLE_EQ(centerZ, 0.0);
}
// Test geometry type detection
TEST_F(GraphicsNodeTest, GeometryTypeDetection) {
    // Test triangle mesh
    {
        GeometryNode node("test", "tri_mesh");
        cvc::geometry triGeom;
        triGeom.points().push_back({0.0, 0.0, 0.0});
        triGeom.points().push_back({1.0, 0.0, 0.0});
        triGeom.points().push_back({0.0, 1.0, 0.0});
        triGeom.tris().push_back({0, 1, 2});
        
        node.setGeometry(triGeom);
        std::string type = std::any_cast<std::string>(node.getMetadata("type"));
        EXPECT_EQ(type, "triangle_mesh");
    }
    
    // Test quad mesh
    {
        GeometryNode node("test", "quad_mesh");
        cvc::geometry quadGeom;
        quadGeom.points().push_back({0.0, 0.0, 0.0});
        quadGeom.points().push_back({1.0, 0.0, 0.0});
        quadGeom.points().push_back({1.0, 1.0, 0.0});
        quadGeom.points().push_back({0.0, 1.0, 0.0});
        quadGeom.quads().push_back({0, 1, 2, 3});
        
        node.setGeometry(quadGeom);
        std::string type = std::any_cast<std::string>(node.getMetadata("type"));
        EXPECT_EQ(type, "quad_mesh");
    }
    
    // Test mixed mesh
    {
        GeometryNode node("test", "mixed_mesh");
        cvc::geometry mixedGeom;
        mixedGeom.points().push_back({0.0, 0.0, 0.0});
        mixedGeom.points().push_back({1.0, 0.0, 0.0});
        mixedGeom.points().push_back({1.0, 1.0, 0.0});
        mixedGeom.points().push_back({0.0, 1.0, 0.0});
        mixedGeom.tris().push_back({0, 1, 2});
        mixedGeom.quads().push_back({0, 1, 2, 3});
        
        node.setGeometry(mixedGeom);
        std::string type = std::any_cast<std::string>(node.getMetadata("type"));
        EXPECT_EQ(type, "mixed_mesh");
    }
}

// Test metadata updates when geometry changes
TEST_F(GraphicsNodeTest, MetadataUpdatesOnGeometryChange) {
    GeometryNode node("test", "test_node");
    
    // Set initial geometry
    node.setGeometry(testGeom);
    int initialVerts = std::any_cast<int>(node.getMetadata("num_vertices"));
    EXPECT_EQ(initialVerts, 3);
    
    // Change geometry
    cvc::geometry newGeom;
    for (int i = 0; i < 10; ++i) {
        newGeom.points().push_back({static_cast<double>(i), 0.0, 0.0});
    }
    newGeom.tris().push_back({0, 1, 2});
    newGeom.tris().push_back({3, 4, 5});
    
    node.setGeometry(newGeom);
    
    // Verify metadata was updated
    int newVerts = std::any_cast<int>(node.getMetadata("num_vertices"));
    int newTris = std::any_cast<int>(node.getMetadata("num_triangles"));
    
    EXPECT_EQ(newVerts, 10);
    EXPECT_EQ(newTris, 2);
}

// Test that empty geometry produces zero metadata
TEST_F(GraphicsNodeTest, EmptyGeometryMetadata) {
    GeometryNode node("test", "empty_node");
    
    cvc::geometry emptyGeom;
    node.setGeometry(emptyGeom);
    
    // Verify metadata for empty geometry
    EXPECT_TRUE(node.hasMetadata("num_vertices"));
    EXPECT_TRUE(node.hasMetadata("num_triangles"));
    
    int numVerts = std::any_cast<int>(node.getMetadata("num_vertices"));
    int numTris = std::any_cast<int>(node.getMetadata("num_triangles"));
    
    EXPECT_EQ(numVerts, 0);
    EXPECT_EQ(numTris, 0);
}

// Test geometry with normals and colors preserves metadata
TEST_F(GraphicsNodeTest, GeometryWithNormalsAndColors) {
    GeometryNode node("test", "colored_node");
    
    cvc::geometry coloredGeom;
    coloredGeom.points().push_back({0.0, 0.0, 0.0});
    coloredGeom.points().push_back({1.0, 0.0, 0.0});
    coloredGeom.points().push_back({0.0, 1.0, 0.0});
    coloredGeom.tris().push_back({0, 1, 2});
    
    // Add normals
    coloredGeom.normals().push_back({0.0, 0.0, 1.0});
    coloredGeom.normals().push_back({0.0, 0.0, 1.0});
    coloredGeom.normals().push_back({0.0, 0.0, 1.0});
    
    // Add colors
    coloredGeom.colors().push_back({1.0, 0.0, 0.0});
    coloredGeom.colors().push_back({0.0, 1.0, 0.0});
    coloredGeom.colors().push_back({0.0, 0.0, 1.0});
    
    node.setGeometry(coloredGeom);
    
    // Metadata should still be computed correctly
    EXPECT_TRUE(node.hasMetadata("num_vertices"));
    int numVerts = std::any_cast<int>(node.getMetadata("num_vertices"));
    EXPECT_EQ(numVerts, 3);
}
// Label Tests
// ============================================================================

TEST_F(GraphicsNodeTest, LabelDefaultState) {
    GeometryNode node("test", "test_node");
    
    // Label should be off by default
    EXPECT_FALSE(node.getShowLabel());
    // Default label text should be node name
    EXPECT_EQ(node.getLabelText(), "test_node");
    // Default size should be 14
    EXPECT_EQ(node.getLabelSize(), 14);
    // Default color should be white
    double r, g, b;
    node.getLabelColor(r, g, b);
    EXPECT_DOUBLE_EQ(r, 1.0);
    EXPECT_DOUBLE_EQ(g, 1.0);
    EXPECT_DOUBLE_EQ(b, 1.0);
}

TEST_F(GraphicsNodeTest, SetLabelText) {
    GeometryNode node("test", "test_node");
    
    node.setLabelText("Custom Label");
    EXPECT_EQ(node.getLabelText(), "Custom Label");
    
    node.setLabelText("Another Label");
    EXPECT_EQ(node.getLabelText(), "Another Label");
}

TEST_F(GraphicsNodeTest, SetLabelSize) {
    GeometryNode node("test", "test_node");
    
    node.setLabelSize(20);
    EXPECT_EQ(node.getLabelSize(), 20);
    
    // Should clamp to minimum 1
    node.setLabelSize(0);
    EXPECT_EQ(node.getLabelSize(), 1);
    
    node.setLabelSize(-5);
    EXPECT_EQ(node.getLabelSize(), 1);
}

TEST_F(GraphicsNodeTest, SetLabelColor) {
    GeometryNode node("test", "test_node");
    
    node.setLabelColor(0.5, 0.75, 1.0);
    
    double r, g, b;
    node.getLabelColor(r, g, b);
    EXPECT_DOUBLE_EQ(r, 0.5);
    EXPECT_DOUBLE_EQ(g, 0.75);
    EXPECT_DOUBLE_EQ(b, 1.0);
}

TEST_F(GraphicsNodeTest, SetShowLabel) {
    GeometryNode node("test", "test_node");
    
    EXPECT_FALSE(node.getShowLabel());
    
    node.setShowLabel(true);
    EXPECT_TRUE(node.getShowLabel());
    
    node.setShowLabel(false);
    EXPECT_FALSE(node.getShowLabel());
}
// ============================================================================
// Bounding Box Tests
// ============================================================================

TEST_F(GraphicsNodeTest, BBoxDefaultState) {
    GeometryNode node("test", "test_node");
    
    // BBox should be off by default for regular nodes
    EXPECT_FALSE(node.getShowBBox());
}

TEST_F(GraphicsNodeTest, SetShowBBox) {
    GeometryNode node("test", "test_node");
    
    node.setShowBBox(true);
    EXPECT_TRUE(node.getShowBBox());
    
    node.setShowBBox(false);
    EXPECT_FALSE(node.getShowBBox());
}
TEST_F(GraphicsNodeTest, BBoxColor) {
    GeometryNode node("test", "test_node");
    
    // Set bbox color
    node.setBBoxColor(1.0, 0.0, 0.0);
    
    // Verify color was set correctly
    double r, g, b;
    node.getBBoxColor(r, g, b);
    EXPECT_DOUBLE_EQ(r, 1.0);
    EXPECT_DOUBLE_EQ(g, 0.0);
    EXPECT_DOUBLE_EQ(b, 0.0);
}

TEST_F(GraphicsNodeTest, BBoxBounds) {
    GeometryNode node("test", "test_node");
    node.setGeometry(testGeom);
    
    cvc::bounding_box bbox = node.getBoundingBox();
    
    // Verify bounding box encompasses all points
    EXPECT_LE(bbox[0], 0.0);  // min x
    EXPECT_LE(bbox[1], 0.0);  // min y
    EXPECT_LE(bbox[2], 0.0);  // min z
    EXPECT_GE(bbox[3], 1.0);  // max x
    EXPECT_GE(bbox[4], 1.0);  // max y
    EXPECT_GE(bbox[5], 0.0);  // max z
}

// Test SceneGraph computeGraphicsBounds with translation transform
TEST_F(GraphicsNodeTest, SceneGraphComputeBoundsWithTranslation) {
    SceneGraph sceneGraph(m_statePrefix);
    
    // Create geometry with unit cube: [0,0,0] to [1,1,1]
    cvc::geometry geom;
    geom.points().push_back({0.0, 0.0, 0.0});
    geom.points().push_back({1.0, 0.0, 0.0});
    geom.points().push_back({0.0, 1.0, 0.0});
    geom.points().push_back({1.0, 1.0, 0.0});
    geom.points().push_back({0.0, 0.0, 1.0});
    geom.points().push_back({1.0, 0.0, 1.0});
    geom.points().push_back({0.0, 1.0, 1.0});
    geom.points().push_back({1.0, 1.0, 1.0});
    
    auto node = sceneGraph.addGraphics("translated_cube", geom);
    
    // Apply translation: move by (10, 20, 30)
    node->setPosition(10.0, 20.0, 30.0);
    
    cvc::bounding_box bounds = sceneGraph.computeGraphicsBounds();
    
    // Bounds should be [10,20,30] to [11,21,31]
    EXPECT_NEAR(bounds[0], 10.0, 1e-6);
    EXPECT_NEAR(bounds[1], 20.0, 1e-6);
    EXPECT_NEAR(bounds[2], 30.0, 1e-6);
    EXPECT_NEAR(bounds[3], 11.0, 1e-6);
    EXPECT_NEAR(bounds[4], 21.0, 1e-6);
    EXPECT_NEAR(bounds[5], 31.0, 1e-6);
}

// Test SceneGraph computeGraphicsBounds with scale transform
TEST_F(GraphicsNodeTest, SceneGraphComputeBoundsWithScale) {
    SceneGraph sceneGraph(m_statePrefix);
    
    // Create geometry with unit cube: [0,0,0] to [1,1,1]
    cvc::geometry geom;
    geom.points().push_back({0.0, 0.0, 0.0});
    geom.points().push_back({1.0, 0.0, 0.0});
    geom.points().push_back({0.0, 1.0, 0.0});
    geom.points().push_back({1.0, 1.0, 0.0});
    geom.points().push_back({0.0, 0.0, 1.0});
    geom.points().push_back({1.0, 0.0, 1.0});
    geom.points().push_back({0.0, 1.0, 1.0});
    geom.points().push_back({1.0, 1.0, 1.0});
    
    auto node = sceneGraph.addGraphics("scaled_cube", geom);
    
    // Apply scale: 2x in X, 3x in Y, 4x in Z
    node->setScale(2.0, 3.0, 4.0);
    
    cvc::bounding_box bounds = sceneGraph.computeGraphicsBounds();
    
    // Bounds should be [0,0,0] to [2,3,4]
    EXPECT_NEAR(bounds[0], 0.0, 1e-6);
    EXPECT_NEAR(bounds[1], 0.0, 1e-6);
    EXPECT_NEAR(bounds[2], 0.0, 1e-6);
    EXPECT_NEAR(bounds[3], 2.0, 1e-6);
    EXPECT_NEAR(bounds[4], 3.0, 1e-6);
    EXPECT_NEAR(bounds[5], 4.0, 1e-6);
}

// Test SceneGraph computeGraphicsBounds with rotation transform
TEST_F(GraphicsNodeTest, SceneGraphComputeBoundsWithRotation) {
    SceneGraph sceneGraph(m_statePrefix);
    
    // Create geometry with square in XY plane: [-1,-1,0] to [1,1,0]
    cvc::geometry geom;
    geom.points().push_back({-1.0, -1.0, 0.0});
    geom.points().push_back({ 1.0, -1.0, 0.0});
    geom.points().push_back({-1.0,  1.0, 0.0});
    geom.points().push_back({ 1.0,  1.0, 0.0});
    
    auto node = sceneGraph.addGraphics("rotated_square", geom);
    
    // Rotate 45 degrees around Z axis
    node->setRotation(0.0, 0.0, 45.0);
    
    cvc::bounding_box bounds = sceneGraph.computeGraphicsBounds();
    
    // After 45 degree rotation, the diagonal becomes axis-aligned
    // Expected bbox: approximately [-sqrt(2), -sqrt(2), 0] to [sqrt(2), sqrt(2), 0]
    double expected = std::sqrt(2.0);
    EXPECT_NEAR(bounds[0], -expected, 1e-4);
    EXPECT_NEAR(bounds[1], -expected, 1e-4);
    EXPECT_NEAR(bounds[2], 0.0, 1e-6);
    EXPECT_NEAR(bounds[3], expected, 1e-4);
    EXPECT_NEAR(bounds[4], expected, 1e-4);
    EXPECT_NEAR(bounds[5], 0.0, 1e-6);
}

// Test SceneGraph computeGraphicsBounds with combined transforms
TEST_F(GraphicsNodeTest, SceneGraphComputeBoundsWithCombinedTransforms) {
    SceneGraph sceneGraph(m_statePrefix);
    
    // Create geometry with unit cube centered at origin: [-0.5,-0.5,-0.5] to [0.5,0.5,0.5]
    cvc::geometry geom;
    geom.points().push_back({-0.5, -0.5, -0.5});
    geom.points().push_back({ 0.5, -0.5, -0.5});
    geom.points().push_back({-0.5,  0.5, -0.5});
    geom.points().push_back({ 0.5,  0.5, -0.5});
    geom.points().push_back({-0.5, -0.5,  0.5});
    geom.points().push_back({ 0.5, -0.5,  0.5});
    geom.points().push_back({-0.5,  0.5,  0.5});
    geom.points().push_back({ 0.5,  0.5,  0.5});
    
    auto node = sceneGraph.addGraphics("combined_cube", geom);
    
    // Apply scale then translate
    node->setScale(2.0, 2.0, 2.0);  // Scale to [-1,-1,-1] to [1,1,1]
    node->setPosition(5.0, 10.0, 15.0);  // Then translate
    
    cvc::bounding_box bounds = sceneGraph.computeGraphicsBounds();
    
    // After scale: [-1,-1,-1] to [1,1,1]
    // After translate: [4,9,14] to [6,11,16]
    EXPECT_NEAR(bounds[0], 4.0, 1e-6);
    EXPECT_NEAR(bounds[1], 9.0, 1e-6);
    EXPECT_NEAR(bounds[2], 14.0, 1e-6);
    EXPECT_NEAR(bounds[3], 6.0, 1e-6);
    EXPECT_NEAR(bounds[4], 11.0, 1e-6);
    EXPECT_NEAR(bounds[5], 16.0, 1e-6);
}

// Test SceneGraph computeGraphicsBounds with multiple transformed objects
TEST_F(GraphicsNodeTest, SceneGraphComputeBoundsMultipleTransformed) {
    SceneGraph sceneGraph(m_statePrefix);
    
    // Create first geometry at [0,0,0] to [1,1,1]
    cvc::geometry geom1;
    geom1.points().push_back({0.0, 0.0, 0.0});
    geom1.points().push_back({1.0, 1.0, 1.0});
    
    // Create second geometry at [0,0,0] to [1,1,1]
    cvc::geometry geom2;
    geom2.points().push_back({0.0, 0.0, 0.0});
    geom2.points().push_back({1.0, 1.0, 1.0});
    
    auto node1 = sceneGraph.addGraphics("obj1", geom1);
    auto node2 = sceneGraph.addGraphics("obj2", geom2);
    
    // Translate first to [10,10,10] to [11,11,11]
    node1->setPosition(10.0, 10.0, 10.0);
    
    // Translate second to [-5,-5,-5] to [-4,-4,-4]
    node2->setPosition(-5.0, -5.0, -5.0);
    
    cvc::bounding_box bounds = sceneGraph.computeGraphicsBounds();
    
    // Combined bounds should be [-5,-5,-5] to [11,11,11]
    EXPECT_NEAR(bounds[0], -5.0, 1e-6);
    EXPECT_NEAR(bounds[1], -5.0, 1e-6);
    EXPECT_NEAR(bounds[2], -5.0, 1e-6);
    EXPECT_NEAR(bounds[3], 11.0, 1e-6);
    EXPECT_NEAR(bounds[4], 11.0, 1e-6);
    EXPECT_NEAR(bounds[5], 11.0, 1e-6);
}

// Test SceneGraph computeGraphicsBounds with hierarchical transforms
TEST_F(GraphicsNodeTest, SceneGraphComputeBoundsHierarchical) {
    SceneGraph sceneGraph(m_statePrefix);
    
    // Create parent geometry at [0,0,0] to [1,1,1]
    cvc::geometry geom1;
    geom1.points().push_back({0.0, 0.0, 0.0});
    geom1.points().push_back({1.0, 1.0, 1.0});
    
    // Create child geometry at [0,0,0] to [1,1,1]
    cvc::geometry geom2;
    geom2.points().push_back({0.0, 0.0, 0.0});
    geom2.points().push_back({1.0, 1.0, 1.0});
    
    auto parent = sceneGraph.addGraphics("parent", geom1);
    auto child = std::make_shared<GeometryNode>("test", "child");
    child->setGeometry(geom2);
    
    // Parent at [10,0,0]
    parent->setPosition(10.0, 0.0, 0.0);
    
    // Child at [5,0,0] relative to parent
    parent->addGraphicsChild(child);
    child->setPosition(5.0, 0.0, 0.0);
    
    cvc::bounding_box bounds = sceneGraph.computeGraphicsBounds();
    
    // Parent bounds: [10,0,0] to [11,1,1]
    // Child world position: [15,0,0] to [16,1,1]
    // Combined: [10,0,0] to [16,1,1]
    EXPECT_NEAR(bounds[0], 10.0, 1e-6);
    EXPECT_NEAR(bounds[1], 0.0, 1e-6);
    EXPECT_NEAR(bounds[2], 0.0, 1e-6);
    EXPECT_NEAR(bounds[3], 16.0, 1e-6);
    EXPECT_NEAR(bounds[4], 1.0, 1e-6);
    EXPECT_NEAR(bounds[5], 1.0, 1e-6);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
