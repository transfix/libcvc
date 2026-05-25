#include <chrono>
#include <cmath>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/state_object.h>
#include <cvc/geometry/geometry.h>
#include <cvc/volume/volmagick.h>
#include <gtest/gtest.h>
#include <thread>
#include <volrover3/GeometryNode.h>
#include <volrover3/NullGraphicNode.h>
#include <volrover3/SceneGraph.h>
#include <volrover3/VolumeNode.h>
#include <vtkActor.h>
#include <vtkMatrix4x4.h>
#include <vtkPlane.h>
#include <vtkPlaneCollection.h>
#include <vtkVolume.h>

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
  cvc::app ctx;
  cvc::geometry testGeom;
};

int GraphicsNodeTest::testCounter = 0;

// Test basic GeometryNode creation (GraphicsNode is abstract)
TEST_F(GraphicsNodeTest, Creation) {
  GeometryNode node(ctx, "test", "test_node");
  EXPECT_EQ(node.getName(), "test_node");
  EXPECT_FALSE(node.hasGeometry());
}

// Test setting geometry
TEST_F(GraphicsNodeTest, SetGeometry) {
  GeometryNode node(ctx, "test", "test_node");
  node.setGeometry(testGeom);

  EXPECT_TRUE(node.hasGeometry());
  ASSERT_NE(node.getGeometry(), nullptr);
  EXPECT_EQ(node.getGeometry()->num_points(), 3);
  EXPECT_EQ(node.getGeometry()->num_tris(), 1);
}

// Test geometry storage in node
TEST_F(GraphicsNodeTest, GeometryRetrieval) {
  GeometryNode node(ctx, "test", "test_node");
  node.setGeometry(testGeom);

  const cvc::geometry *geom = node.getGeometry();
  ASSERT_NE(geom, nullptr);

  // Verify geometry data is correct
  EXPECT_EQ(geom->points().size(), 3);
  EXPECT_EQ(geom->tris().size(), 1);
  EXPECT_DOUBLE_EQ(geom->points()[0][0], 0.0);
  EXPECT_DOUBLE_EQ(geom->points()[1][0], 1.0);
}

// Test default transform is identity
TEST_F(GraphicsNodeTest, DefaultTransformIsIdentity) {
  GeometryNode node(ctx, "test", "test_node");
  vtkMatrix4x4 *transform = node.getTransform();

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
  GeometryNode node(ctx, "test", "test_node");
  node.setPosition(1.0, 2.0, 3.0);

  vtkMatrix4x4 *transform = node.getTransform();
  EXPECT_DOUBLE_EQ(transform->GetElement(0, 3), 1.0);
  EXPECT_DOUBLE_EQ(transform->GetElement(1, 3), 2.0);
  EXPECT_DOUBLE_EQ(transform->GetElement(2, 3), 3.0);
}

// Test setting scale
TEST_F(GraphicsNodeTest, SetScale) {
  GeometryNode node(ctx, "test", "test_node");
  node.setScale(2.0, 3.0, 4.0);

  vtkMatrix4x4 *transform = node.getTransform();
  EXPECT_DOUBLE_EQ(transform->GetElement(0, 0), 2.0);
  EXPECT_DOUBLE_EQ(transform->GetElement(1, 1), 3.0);
  EXPECT_DOUBLE_EQ(transform->GetElement(2, 2), 4.0);
}

// Test reset transform
TEST_F(GraphicsNodeTest, ResetTransform) {
  GeometryNode node(ctx, "test", "test_node");
  node.setPosition(1.0, 2.0, 3.0);
  node.resetTransform();

  vtkMatrix4x4 *transform = node.getTransform();

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
  GeometryNode node(ctx, "test", "test_node");

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
  GeometryNode node(ctx, "test", "test_node");

  EXPECT_TRUE(node.isVisible());
}

// Test setVisible updates metadata
TEST_F(GraphicsNodeTest, SetVisibleUpdatesMetadata) {
  GeometryNode node(ctx, "test", "test_node");

  node.setVisible(false);
  EXPECT_FALSE(node.isVisible());

  node.setVisible(true);
  EXPECT_TRUE(node.isVisible());
}

// Test type metadata for geometry
TEST_F(GraphicsNodeTest, TypeMetadata) {
  GeometryNode node(ctx, "test", "test_node");
  node.setMetadata("type", std::string("geometry"));

  EXPECT_TRUE(node.hasMetadata("type"));
  EXPECT_EQ(std::any_cast<std::string>(node.getMetadata("type")), "geometry");
}

// Test hierarchical structure - adding children
TEST_F(GraphicsNodeTest, AddChild) {
  auto parent = std::make_shared<GeometryNode>(ctx, "test", "parent");
  auto child1 = std::make_shared<GeometryNode>(ctx, "test", "child1");
  auto child2 = std::make_shared<GeometryNode>(ctx, "test", "child2");

  parent->addGraphicsChild(child1);
  parent->addGraphicsChild(child2);

  EXPECT_EQ(parent->getGraphicsChildren().size(), 2);
}

// Test finding child by name
TEST_F(GraphicsNodeTest, FindChildByName) {
  auto parent = std::make_shared<GeometryNode>(ctx, "test", "parent");
  auto child1 = std::make_shared<GeometryNode>(ctx, "test", "child1");
  auto child2 = std::make_shared<GeometryNode>(ctx, "test", "child2");

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
  auto parent = std::make_shared<GeometryNode>(ctx, "test", "parent");
  auto child1 = std::make_shared<GeometryNode>(ctx, "test", "child1");
  auto child2 = std::make_shared<GeometryNode>(ctx, "test", "child2");

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

  auto customNode = std::make_shared<GeometryNode>(ctx, "test", "custom");
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
  auto parent = std::make_shared<GeometryNode>(ctx, "test.parent", "parent");
  auto child = std::make_shared<GeometryNode>(ctx, "test.child", "child");

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
  GeometryNode node(ctx, "test", "test_node");
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
  GeometryNode node(ctx, "test", "test_node");
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
  GeometryNode node(ctx, "test", "test_node");
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
  GeometryNode node(ctx, "test", "test_node");
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
    GeometryNode node(ctx, "test", "tri_mesh");
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
    GeometryNode node(ctx, "test", "quad_mesh");
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
    GeometryNode node(ctx, "test", "mixed_mesh");
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
  GeometryNode node(ctx, "test", "test_node");

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
  GeometryNode node(ctx, "test", "empty_node");

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
  GeometryNode node(ctx, "test", "colored_node");

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
  GeometryNode node(ctx, "test", "test_node");

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
  GeometryNode node(ctx, "test", "test_node");

  node.setLabelText("Custom Label");
  EXPECT_EQ(node.getLabelText(), "Custom Label");

  node.setLabelText("Another Label");
  EXPECT_EQ(node.getLabelText(), "Another Label");
}

TEST_F(GraphicsNodeTest, SetLabelSize) {
  GeometryNode node(ctx, "test", "test_node");

  node.setLabelSize(20);
  EXPECT_EQ(node.getLabelSize(), 20);

  // Should clamp to minimum 1
  node.setLabelSize(0);
  EXPECT_EQ(node.getLabelSize(), 1);

  node.setLabelSize(-5);
  EXPECT_EQ(node.getLabelSize(), 1);
}

TEST_F(GraphicsNodeTest, SetLabelColor) {
  GeometryNode node(ctx, "test", "test_node");

  node.setLabelColor(0.5, 0.75, 1.0);

  double r, g, b;
  node.getLabelColor(r, g, b);
  EXPECT_DOUBLE_EQ(r, 0.5);
  EXPECT_DOUBLE_EQ(g, 0.75);
  EXPECT_DOUBLE_EQ(b, 1.0);
}

TEST_F(GraphicsNodeTest, SetShowLabel) {
  GeometryNode node(ctx, "test", "test_node");

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
  GeometryNode node(ctx, "test", "test_node");

  // BBox should be off by default for regular nodes
  EXPECT_FALSE(node.getShowBBox());
}

TEST_F(GraphicsNodeTest, SetShowBBox) {
  GeometryNode node(ctx, "test", "test_node");

  node.setShowBBox(true);
  EXPECT_TRUE(node.getShowBBox());

  node.setShowBBox(false);
  EXPECT_FALSE(node.getShowBBox());
}
TEST_F(GraphicsNodeTest, BBoxColor) {
  GeometryNode node(ctx, "test", "test_node");

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
  GeometryNode node(ctx, "test", "test_node");
  node.setGeometry(testGeom);

  cvc::bounding_box bbox = node.getBoundingBox();

  // Verify bounding box encompasses all points
  EXPECT_LE(bbox[0], 0.0); // min x
  EXPECT_LE(bbox[1], 0.0); // min y
  EXPECT_LE(bbox[2], 0.0); // min z
  EXPECT_GE(bbox[3], 1.0); // max x
  EXPECT_GE(bbox[4], 1.0); // max y
  EXPECT_GE(bbox[5], 0.0); // max z
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
  geom.points().push_back({1.0, -1.0, 0.0});
  geom.points().push_back({-1.0, 1.0, 0.0});
  geom.points().push_back({1.0, 1.0, 0.0});

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
  geom.points().push_back({0.5, -0.5, -0.5});
  geom.points().push_back({-0.5, 0.5, -0.5});
  geom.points().push_back({0.5, 0.5, -0.5});
  geom.points().push_back({-0.5, -0.5, 0.5});
  geom.points().push_back({0.5, -0.5, 0.5});
  geom.points().push_back({-0.5, 0.5, 0.5});
  geom.points().push_back({0.5, 0.5, 0.5});

  auto node = sceneGraph.addGraphics("combined_cube", geom);

  // Apply scale then translate
  node->setScale(2.0, 2.0, 2.0);      // Scale to [-1,-1,-1] to [1,1,1]
  node->setPosition(5.0, 10.0, 15.0); // Then translate

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
  auto child = std::make_shared<GeometryNode>(ctx, "test", "child");
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

// ============================================================================
// Bounding Box Transform Tests
// ============================================================================

TEST_F(GraphicsNodeTest, BBoxShowsLocalSpace) {
  // Verify that bounding box geometry stays in local space
  // and transform is applied to the bbox actor
  SceneGraph sceneGraph(m_statePrefix);

  cvc::geometry geom;
  geom.points().push_back({0.0, 0.0, 0.0});
  geom.points().push_back({1.0, 1.0, 1.0});

  auto node = sceneGraph.addGraphics("bbox_test", geom);

  // Enable bbox
  node->setShowBBox(true);

  // Apply a transform (rotation + translation)
  node->setPosition(10.0, 0.0, 0.0);
  node->setRotation(0.0, 0.0, 45.0); // 45 degree rotation around Z

  // The node's getBoundingBox should return local space coords
  cvc::bounding_box localBBox = node->getBoundingBox();
  EXPECT_NEAR(localBBox[0], 0.0, 1e-6);
  EXPECT_NEAR(localBBox[3], 1.0, 1e-6);

  // The bbox visualization will use the transform to render correctly
  // This is applied in updateBoundingBoxNode() via setTransform()
}

// ============================================================================
// Clip Plane Tests
// ============================================================================

TEST_F(GraphicsNodeTest, ClipChildrenDefault) {
  // Default clipChildren should be false
  SceneGraph sceneGraph(m_statePrefix);
  cvc::geometry geom;
  geom.points().push_back({0.0, 0.0, 0.0});
  auto parent = sceneGraph.addGraphics("cliptest.parent", geom);

  EXPECT_FALSE(parent->getClipChildren());
}

TEST_F(GraphicsNodeTest, SetClipChildren) {
  SceneGraph sceneGraph(m_statePrefix);
  cvc::geometry geom;
  geom.points().push_back({0.0, 0.0, 0.0});
  auto parent = sceneGraph.addGraphics("cliptest.setter", geom);

  parent->setClipChildren(true);
  EXPECT_TRUE(parent->getClipChildren());

  parent->setClipChildren(false);
  EXPECT_FALSE(parent->getClipChildren());
}

TEST_F(GraphicsNodeTest, ClipChildrenStateSync) {
  SceneGraph sceneGraph(m_statePrefix);
  cvc::geometry geom;
  geom.points().push_back({0.0, 0.0, 0.0});
  auto parent = sceneGraph.addGraphics("cliptest.statesync", geom);

  // Test setter -> state tree
  parent->setClipChildren(true);
  int stateValue = parent->getState("clip_children").template value<int>();
  EXPECT_EQ(stateValue, 1);

  parent->setClipChildren(false);
  stateValue = parent->getState("clip_children").template value<int>();
  EXPECT_EQ(stateValue, 0);

  // Test state tree -> getter
  parent->getState("clip_children").value(1);
  // Give time for state change to propagate
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_TRUE(parent->getClipChildren());
}

TEST_F(GraphicsNodeTest, ClipPlanesGenerated) {
  // Create a parent with a known bounding box
  SceneGraph sceneGraph(m_statePrefix);
  cvc::geometry geom;
  geom.points().push_back({0.0, 0.0, 0.0});
  geom.points().push_back({2.0, 3.0, 4.0});

  auto parent = sceneGraph.addGraphics("cliptest.planes", geom);
  parent->setClipChildren(true);

  // Should have 6 clip planes
  vtkPlaneCollection *planes = parent->getClipPlanes();
  ASSERT_NE(planes, nullptr);
  EXPECT_EQ(planes->GetNumberOfItems(), 6);
}

TEST_F(GraphicsNodeTest, ClipPlanesTransform) {
  // Create a parent with bounding box and transform
  SceneGraph sceneGraph(m_statePrefix);
  cvc::geometry geom;
  geom.points().push_back({0.0, 0.0, 0.0});
  geom.points().push_back({1.0, 1.0, 1.0});

  auto parent = sceneGraph.addGraphics("cliptest.transform", geom);
  parent->setPosition(10.0, 20.0, 30.0);
  parent->setClipChildren(true);

  // Planes should be transformed with the parent's transform
  vtkPlaneCollection *planes = parent->getClipPlanes();
  ASSERT_NE(planes, nullptr);
  EXPECT_EQ(planes->GetNumberOfItems(), 6);

  // Get first plane and check it's been transformed
  // The exact values depend on implementation, but planes should exist
  planes->InitTraversal();
  vtkPlane *plane = planes->GetNextItem();
  ASSERT_NE(plane, nullptr);
}

TEST_F(GraphicsNodeTest, ChildrenClipped) {
  // Create parent and child geometry nodes
  SceneGraph sceneGraph(m_statePrefix);
  cvc::geometry parentGeom;
  parentGeom.points().push_back({0.0, 0.0, 0.0});
  parentGeom.points().push_back({10.0, 10.0, 10.0});

  cvc::geometry childGeom;
  childGeom.points().push_back({5.0, 5.0, 5.0});
  childGeom.points().push_back({15.0, 15.0, 15.0});

  auto parent = sceneGraph.addGraphics("cliptest.children", parentGeom);
  auto child = std::make_shared<GeometryNode>(ctx, "cliptest.children.child", "child");
  child->setGeometry(childGeom);
  parent->addGraphicsChild(child);

  // Enable clipping on parent
  parent->setClipChildren(true);

  // Give time for threading/event queue to process
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Child should have received clip planes
  // We can't easily test the mapper directly without VTK rendering context,
  // but we can verify the planes were created
  vtkPlaneCollection *planes = parent->getClipPlanes();
  ASSERT_NE(planes, nullptr);
  EXPECT_EQ(planes->GetNumberOfItems(), 6);
}

TEST_F(GraphicsNodeTest, ClippingDisabled) {
  SceneGraph sceneGraph(m_statePrefix);
  cvc::geometry parentGeom;
  parentGeom.points().push_back({0.0, 0.0, 0.0});
  parentGeom.points().push_back({10.0, 10.0, 10.0});

  cvc::geometry childGeom;
  childGeom.points().push_back({5.0, 5.0, 5.0});

  auto parent = sceneGraph.addGraphics("cliptest.disabled", parentGeom);
  auto child = std::make_shared<GeometryNode>(ctx, "cliptest.disabled.child", "child");
  child->setGeometry(childGeom);
  parent->addGraphicsChild(child);

  // Enable then disable clipping
  parent->setClipChildren(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  parent->setClipChildren(false);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Clipping should be disabled
  EXPECT_FALSE(parent->getClipChildren());
}

// Test createChild with GeometryNode and geometry data
TEST_F(GraphicsNodeTest, CreateChildGeometry) {
  SceneGraph sceneGraph(m_statePrefix);

  // Create parent geometry
  cvc::geometry parentGeom;
  parentGeom.points().push_back({0.0, 0.0, 0.0});
  parentGeom.points().push_back({10.0, 0.0, 0.0});
  parentGeom.points().push_back({0.0, 10.0, 0.0});
  parentGeom.tris().push_back({0, 1, 2});

  auto parent = sceneGraph.addGraphics("parent", parentGeom);

  // Create child geometry using createChild
  cvc::geometry childGeom;
  childGeom.points().push_back({1.0, 1.0, 1.0});
  childGeom.points().push_back({2.0, 1.0, 1.0});
  childGeom.points().push_back({1.0, 2.0, 1.0});
  childGeom.tris().push_back({0, 1, 2});

  auto child = parent->createChild<GeometryNode>("child", childGeom);

  // Verify child was created correctly
  ASSERT_NE(child, nullptr);
  EXPECT_TRUE(child->hasGeometry());
  EXPECT_EQ(child->getName(), "child");

  // Verify parent-child relationship
  const auto &children = parent->getGraphicsChildren();
  EXPECT_EQ(children.size(), 1);
  EXPECT_EQ(children[0], child);

  // Verify geometry was set correctly
  const cvc::geometry *geom = child->getGeometry();
  ASSERT_NE(geom, nullptr);
  EXPECT_EQ(geom->points().size(), 3);
  EXPECT_EQ(geom->tris().size(), 1);
}

// Test createChild with VolumeNode and volume data
TEST_F(GraphicsNodeTest, CreateChildVolume) {
  SceneGraph sceneGraph(m_statePrefix);

  // Create parent geometry
  auto parent = sceneGraph.addGraphics("parent", testGeom);

  // Create child volume using createChild
  cvc::volume vol(ctx, cvc::dimension(10, 10, 10), cvc::UChar,
                  cvc::bounding_box(0.0, 0.0, 0.0, 10.0, 10.0, 10.0));
  auto child = parent->createChild<VolumeNode>("volume_child", vol);

  // Verify child was created correctly
  ASSERT_NE(child, nullptr);
  EXPECT_TRUE(child->hasVolume());
  EXPECT_EQ(child->getName(), "volume_child");

  // Verify parent-child relationship
  const auto &children = parent->getGraphicsChildren();
  EXPECT_EQ(children.size(), 1);
  EXPECT_EQ(children[0], child);

  // Verify volume was set correctly
  const cvc::volume *v = child->getVolume();
  ASSERT_NE(v, nullptr);
  EXPECT_EQ(v->XDim(), 10);
  EXPECT_EQ(v->YDim(), 10);
  EXPECT_EQ(v->ZDim(), 10);
}

// Test createChild without data (should create NullGraphicNode)
TEST_F(GraphicsNodeTest, CreateChildNoData) {
  SceneGraph sceneGraph(m_statePrefix);

  auto parent = sceneGraph.addGraphics("parent", testGeom);

  // Create child without data
  auto child = parent->createChild("null_child");

  // Verify child was created
  ASSERT_NE(child, nullptr);
  EXPECT_EQ(child->getName(), "null_child");

  // Verify parent-child relationship
  const auto &children = parent->getGraphicsChildren();
  EXPECT_EQ(children.size(), 1);
  EXPECT_EQ(children[0], child);

  // Verify it's a NullGraphicNode (default bounds are -0.5 to 0.5)
  auto bbox = child->getBoundingBox();
  EXPECT_EQ(bbox.minx, -0.5);
  EXPECT_EQ(bbox.maxx, 0.5);
  EXPECT_EQ(bbox.miny, -0.5);
  EXPECT_EQ(bbox.maxy, 0.5);
  EXPECT_EQ(bbox.minz, -0.5);
  EXPECT_EQ(bbox.maxz, 0.5);
}

// Test multiple createChild calls
TEST_F(GraphicsNodeTest, CreateMultipleChildren) {
  SceneGraph sceneGraph(m_statePrefix);

  auto parent = sceneGraph.addGraphics("parent", testGeom);

  // Create multiple children of different types
  cvc::geometry geom1;
  geom1.points().push_back({1.0, 0.0, 0.0});
  auto child1 = parent->createChild<GeometryNode>("geom1", geom1);

  cvc::volume vol1(ctx, cvc::dimension(5, 5, 5), cvc::UChar,
                   cvc::bounding_box(0.0, 0.0, 0.0, 5.0, 5.0, 5.0));
  auto child2 = parent->createChild<VolumeNode>("vol1", vol1);

  cvc::geometry geom2;
  geom2.points().push_back({2.0, 0.0, 0.0});
  auto child3 = parent->createChild<GeometryNode>("geom2", geom2);

  // Verify all children were created
  ASSERT_NE(child1, nullptr);
  ASSERT_NE(child2, nullptr);
  ASSERT_NE(child3, nullptr);

  // Verify parent has all children
  const auto &children = parent->getGraphicsChildren();
  EXPECT_EQ(children.size(), 3);

  // Verify correct types
  auto geomChild1 = std::dynamic_pointer_cast<GeometryNode>(child1);
  auto volChild = std::dynamic_pointer_cast<VolumeNode>(child2);
  auto geomChild2 = std::dynamic_pointer_cast<GeometryNode>(child3);

  EXPECT_NE(geomChild1, nullptr);
  EXPECT_NE(volChild, nullptr);
  EXPECT_NE(geomChild2, nullptr);
}

// Test createChild state tree path construction
TEST_F(GraphicsNodeTest, CreateChildStatePath) {
  SceneGraph sceneGraph(m_statePrefix);

  auto parent = sceneGraph.addGraphics("parent", testGeom);
  auto child = parent->createChild<GeometryNode>("child", testGeom);

  // Verify state path is properly constructed
  std::string parentPath = parent->getState().fullName();
  std::string childPath = child->getState().fullName();

  // Child path should be parent.children.child
  std::string expectedPath = parentPath + ".children.child";
  EXPECT_EQ(childPath, expectedPath);
}

// Test nested createChild calls
TEST_F(GraphicsNodeTest, CreateNestedChildren) {
  SceneGraph sceneGraph(m_statePrefix);

  auto root = sceneGraph.addGraphics("root", testGeom);
  auto level1 = root->createChild<GeometryNode>("level1", testGeom);
  auto level2 = level1->createChild<GeometryNode>("level2", testGeom);
  auto level3 = level2->createChild<GeometryNode>("level3", testGeom);

  // Verify hierarchy
  ASSERT_NE(level1, nullptr);
  ASSERT_NE(level2, nullptr);
  ASSERT_NE(level3, nullptr);

  EXPECT_EQ(root->getGraphicsChildren().size(), 1);
  EXPECT_EQ(level1->getGraphicsChildren().size(), 1);
  EXPECT_EQ(level2->getGraphicsChildren().size(), 1);
  EXPECT_EQ(level3->getGraphicsChildren().size(), 0);

  // Verify state paths are properly nested
  std::string rootPath = root->getState().fullName();
  std::string level1Path = level1->getState().fullName();
  std::string level2Path = level2->getState().fullName();
  std::string level3Path = level3->getState().fullName();

  EXPECT_EQ(level1Path, rootPath + ".children.level1");
  EXPECT_EQ(level2Path, level1Path + ".children.level2");
  EXPECT_EQ(level3Path, level2Path + ".children.level3");
}

// Test that child volumes are found by SceneGraph::getAllVolumeGraphics()
TEST_F(GraphicsNodeTest, CreateChildVolumeDiscovery) {
  SceneGraph sceneGraph(m_statePrefix);

  // Create a parent geometry
  auto parent = sceneGraph.addGraphics("parent", testGeom);

  // Initial check - no volumes
  EXPECT_EQ(sceneGraph.getAllVolumeGraphics().size(), 0);
  EXPECT_EQ(sceneGraph.getVolumeGraphicsCount(), 0);

  // Create child volumes
  cvc::volume vol1(ctx, cvc::dimension(5, 5, 5), cvc::UChar, cvc::bounding_box(0, 0, 0, 4, 4, 4));
  auto childVol1 = parent->createChild<VolumeNode>("vol1", vol1);

  // Should now find the child volume
  auto allVolumes = sceneGraph.getAllVolumeGraphics();
  EXPECT_EQ(allVolumes.size(), 1);
  EXPECT_EQ(sceneGraph.getVolumeGraphicsCount(), 1);
  EXPECT_EQ(allVolumes[0], childVol1);

  // Add another child volume at different level
  cvc::volume vol2(ctx, cvc::dimension(3, 3, 3), cvc::UChar, cvc::bounding_box(0, 0, 0, 2, 2, 2));
  auto childVol2 = childVol1->createChild<VolumeNode>("vol2", vol2);

  // Should find both volumes
  allVolumes = sceneGraph.getAllVolumeGraphics();
  EXPECT_EQ(allVolumes.size(), 2);
  EXPECT_EQ(sceneGraph.getVolumeGraphicsCount(), 2);

  // Verify both volumes are in the list
  bool foundVol1 = false, foundVol2 = false;
  for (const auto &vol : allVolumes) {
    if (vol == childVol1)
      foundVol1 = true;
    if (vol == childVol2)
      foundVol2 = true;
  }
  EXPECT_TRUE(foundVol1);
  EXPECT_TRUE(foundVol2);
}

// Test that child geometries are found by SceneGraph::getAllGeometryGraphics()
TEST_F(GraphicsNodeTest, CreateChildGeometryDiscovery) {
  SceneGraph sceneGraph(m_statePrefix);

  // Create a parent volume
  cvc::volume vol(ctx, cvc::dimension(5, 5, 5), cvc::UChar, cvc::bounding_box(0, 0, 0, 4, 4, 4));
  auto parent = sceneGraph.addGraphics("parent_vol", vol);

  // Initial check - no geometries
  EXPECT_EQ(sceneGraph.getAllGeometryGraphics().size(), 0);
  EXPECT_EQ(sceneGraph.getGeometryGraphicsCount(), 0);

  // Create child geometries
  cvc::geometry geom1;
  geom1.points().push_back({1.0, 0.0, 0.0});
  geom1.points().push_back({0.0, 1.0, 0.0});
  geom1.points().push_back({0.0, 0.0, 1.0});
  geom1.tris().push_back({0, 1, 2});
  auto childGeom1 = parent->createChild<GeometryNode>("geom1", geom1);

  // Should now find the child geometry
  auto allGeometries = sceneGraph.getAllGeometryGraphics();
  EXPECT_EQ(allGeometries.size(), 1);
  EXPECT_EQ(sceneGraph.getGeometryGraphicsCount(), 1);
  EXPECT_EQ(allGeometries[0], childGeom1);

  // Add another child geometry at different level
  cvc::geometry geom2;
  geom2.points().push_back({2.0, 0.0, 0.0});
  auto childGeom2 = childGeom1->createChild<GeometryNode>("geom2", geom2);

  // Should find both geometries
  allGeometries = sceneGraph.getAllGeometryGraphics();
  EXPECT_EQ(allGeometries.size(), 2);
  EXPECT_EQ(sceneGraph.getGeometryGraphicsCount(), 2);

  // Verify both geometries are in the list
  bool foundGeom1 = false, foundGeom2 = false;
  for (const auto &geom : allGeometries) {
    if (geom == childGeom1)
      foundGeom1 = true;
    if (geom == childGeom2)
      foundGeom2 = true;
  }
  EXPECT_TRUE(foundGeom1);
  EXPECT_TRUE(foundGeom2);
}

// ============================================================================
// Transform Hierarchy Tests - World Transform Application
// ============================================================================

TEST_F(GraphicsNodeTest, ChildVolumeInheritsParentTransform) {
  // Create scene graph with parent geometry and child volume
  SceneGraph sceneGraph;
  auto rootNode = sceneGraph.getGraphicsRoot();

  // Create parent geometry with scale transform
  cvc::geometry parentGeom;
  parentGeom.points().push_back({0.0, 0.0, 0.0});
  parentGeom.points().push_back({1.0, 0.0, 0.0});
  parentGeom.points().push_back({0.0, 1.0, 0.0});
  parentGeom.tris().push_back({0, 1, 2});

  auto parentNode = rootNode->createChild<GeometryNode>("parent", parentGeom);

  // Apply scale to parent
  parentNode->setScale(2.0, 2.0, 2.0);

  // Create child volume (SDF from parent geometry)
  cvc::volume childVol(ctx, cvc::dimension(10, 10, 10), cvc::UChar,
                       cvc::bounding_box(0.0, 0.0, 0.0, 1.0, 1.0, 1.0));

  auto childNode = parentNode->createChild<VolumeNode>("sdf", childVol);

  // Get the world transform of the child
  auto childWorldTransform = childNode->getWorldTransform();

  // The world transform should include parent's scale (2x)
  // Extract scale from the transform matrix
  double scaleX =
      std::sqrt(childWorldTransform->GetElement(0, 0) * childWorldTransform->GetElement(0, 0) +
                childWorldTransform->GetElement(1, 0) * childWorldTransform->GetElement(1, 0) +
                childWorldTransform->GetElement(2, 0) * childWorldTransform->GetElement(2, 0));

  EXPECT_NEAR(scaleX, 2.0, 1e-6);
}

TEST_F(GraphicsNodeTest, ChildGeometryInheritsParentTransform) {
  // Create scene graph with parent volume and child geometry
  SceneGraph sceneGraph;
  auto rootNode = sceneGraph.getGraphicsRoot();

  // Create parent volume
  cvc::volume parentVol(ctx, cvc::dimension(10, 10, 10), cvc::UChar,
                        cvc::bounding_box(0.0, 0.0, 0.0, 1.0, 1.0, 1.0));

  auto parentNode = rootNode->createChild<VolumeNode>("parent", parentVol);

  // Apply scale to parent
  parentNode->setScale(3.0, 3.0, 3.0);

  // Create child geometry (isosurface from parent volume)
  cvc::geometry childGeom;
  childGeom.points().push_back({0.0, 0.0, 0.0});
  childGeom.points().push_back({1.0, 0.0, 0.0});
  childGeom.points().push_back({0.0, 1.0, 0.0});
  childGeom.tris().push_back({0, 1, 2});

  auto childNode = parentNode->createChild<GeometryNode>("isosurface", childGeom);

  // Get the world transform of the child
  auto childWorldTransform = childNode->getWorldTransform();

  // The world transform should include parent's scale (3x)
  double scaleX =
      std::sqrt(childWorldTransform->GetElement(0, 0) * childWorldTransform->GetElement(0, 0) +
                childWorldTransform->GetElement(1, 0) * childWorldTransform->GetElement(1, 0) +
                childWorldTransform->GetElement(2, 0) * childWorldTransform->GetElement(2, 0));

  EXPECT_NEAR(scaleX, 3.0, 1e-6);
}

TEST_F(GraphicsNodeTest, DeepTransformHierarchy) {
  // Test a 3-level hierarchy: root -> parent (scaled) -> child (rotated) -> grandchild (translated)
  SceneGraph sceneGraph;
  auto rootNode = sceneGraph.getGraphicsRoot();

  // Create parent with scale
  cvc::geometry parentGeom;
  parentGeom.points().push_back({0.0, 0.0, 0.0});
  auto parentNode = rootNode->createChild<GeometryNode>("parent", parentGeom);
  parentNode->setScale(2.0, 2.0, 2.0);

  // Create child with rotation (90 degrees around Z)
  cvc::volume childVol(ctx, cvc::dimension(5, 5, 5), cvc::UChar,
                       cvc::bounding_box(0.0, 0.0, 0.0, 1.0, 1.0, 1.0));
  auto childNode = parentNode->createChild<VolumeNode>("child", childVol);
  childNode->setRotation(0.0, 0.0, 90.0);

  // Create grandchild with translation
  cvc::geometry grandchildGeom;
  grandchildGeom.points().push_back({1.0, 0.0, 0.0});
  auto grandchildNode = childNode->createChild<GeometryNode>("grandchild", grandchildGeom);
  grandchildNode->setPosition(5.0, 0.0, 0.0);

  // Verify grandchild's world transform includes all transformations
  auto worldTransform = grandchildNode->getWorldTransform();

  // The grandchild should be scaled by parent
  double scaleX = std::sqrt(worldTransform->GetElement(0, 0) * worldTransform->GetElement(0, 0) +
                            worldTransform->GetElement(1, 0) * worldTransform->GetElement(1, 0) +
                            worldTransform->GetElement(2, 0) * worldTransform->GetElement(2, 0));
  EXPECT_NEAR(scaleX, 2.0, 0.1); // Allow some tolerance due to rotation
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
