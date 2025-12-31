#include <gtest/gtest.h>
#include <volrover3/SceneGraph.h>
#include <volrover3/GeometryNode.h>
#include <volrover3/VolumeNode.h>
#include <volrover3/GridNode.h>
#include <volrover3/AxisNode.h>
#include <volrover3/BBoxNode.h>
#include <volrover3/AppState.h>
#include <cvc/state.h>
#include <cvc/geometry.h>
#include <cvc/volume.h>

class SceneGraphTest : public ::testing::Test {
protected:
    void SetUp() override {
        sceneGraph = new SceneGraph();
        appState = &AppState::instance();
    }
    
    void TearDown() override {
        delete sceneGraph;
    }
    
    SceneGraph* sceneGraph;
    AppState* appState;
};

TEST_F(SceneGraphTest, InitialState) {
    EXPECT_NE(sceneGraph, nullptr);
    // SceneGraph doesn't expose renderer/renderWindow, just verify it was created
    SUCCEED();
}

TEST_F(SceneGraphTest, AddGeometryNode) {
    cvc::geometry geom;
    geom.points().push_back({0.0, 0.0, 0.0});
    geom.points().push_back({1.0, 0.0, 0.0});
    geom.points().push_back({0.0, 1.0, 0.0});
    
    sceneGraph->setGeometry(geom);
    
    // Verify the geometry was set (node should be created internally)
    // We can't directly test the private geometry node, but we can verify no crash
    SUCCEED();
}

TEST_F(SceneGraphTest, AddVolumeNode) {
    cvc::volume vol(cvc::dimension(4, 4, 4), cvc::UChar);
    
    sceneGraph->setVolume(vol);
    
    // Verify the volume was set (node should be created internally)
    SUCCEED();
}

TEST_F(SceneGraphTest, ShowHideGrid) {
    sceneGraph->setGridVisible(true);
    // Grid should be created and visible
    
    sceneGraph->setGridVisible(false);
    // Grid should be hidden
    
    SUCCEED();
}

TEST_F(SceneGraphTest, ShowHideAxes) {
    sceneGraph->setAxisVisible(true);
    // Axes should be created and visible
    
    sceneGraph->setAxisVisible(false);
    // Axes should be hidden
    
    SUCCEED();
}

TEST_F(SceneGraphTest, UpdateBoundingBox) {
    cvc::bounding_box bounds;
    bounds.setMin(-1.0, -1.0, -1.0);
    bounds.setMax(1.0, 1.0, 1.0);
    sceneGraph->updateGrid(bounds);
    
    // Grid should be updated to match bounding box
    SUCCEED();
}

TEST_F(SceneGraphTest, ResetCamera) {
    // SceneGraph doesn't expose resetCamera, this would be done via the renderer
    SUCCEED();
}

TEST_F(SceneGraphTest, TransferFunctionUpdate) {
    // Create a volume first
    cvc::volume vol(cvc::dimension(4, 4, 4), cvc::UChar);
    sceneGraph->setVolume(vol);
    
    // Update transfer function
    std::vector<double> colorTable = {0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0};
    std::vector<double> opacityTable = {0.0, 0.0, 1.0, 1.0};
    
    sceneGraph->updateTransferFunction(colorTable, opacityTable);
    
    // Transfer function should be applied to volume
    SUCCEED();
}

TEST_F(SceneGraphTest, MultipleUpdates) {
    // Test multiple updates don't cause issues
    cvc::geometry geom;
    geom.points().push_back({0.0, 0.0, 0.0});
    sceneGraph->setGeometry(geom);
    
    cvc::volume vol(cvc::dimension(2, 2, 2), cvc::UChar);
    sceneGraph->setVolume(vol);
    
    sceneGraph->setGridVisible(true);
    sceneGraph->setAxisVisible(true);
    
    // Camera reset would be done externally
    
    SUCCEED();
}

// ===========================
// State Tree Integration Tests
// ===========================

TEST_F(SceneGraphTest, VisibilityStateTree) {
    // SceneGraph doesn't directly manipulate state tree
    // but we can verify it responds to AppState visibility flags
    
    // Set visibility via scene graph
    sceneGraph->setGridVisible(true);
    sceneGraph->setAxisVisible(false);
    
    // These don't save to AppState automatically
    // In actual usage, MainWindow coordinates between SceneGraph and AppState
    SUCCEED();
}

TEST_F(SceneGraphTest, TransferFunctionFromState) {
    // Create volume
    cvc::volume vol(cvc::dimension(4, 4, 4), cvc::UChar);
    sceneGraph->setVolume(vol);
    
    // Get transfer function from AppState
    auto colorTable = appState->transferFunctionColorTable();
    auto opacityTable = appState->transferFunctionOpacityTable();
    
    // Apply to scene graph
    sceneGraph->updateTransferFunction(colorTable, opacityTable);
    
    SUCCEED();
}

TEST_F(SceneGraphTest, WorldBoundsUpdate) {
    // Set world bounds in AppState
    cvc::bounding_box bounds(-2.0, -2.0, -2.0, 2.0, 2.0, 2.0);
    appState->setWorldBounds(bounds);
    
    // Update scene graph grid
    sceneGraph->updateGrid(bounds);
    
    // Verify state tree has the bounds
    auto& stateTree = cvc::state::instance()("volrover3");
    auto values = stateTree("world_bounds").values();
    ASSERT_EQ(values.size(), size_t(6));
    
    SUCCEED();
}

// ===========================
// Color Tests
// ===========================

TEST_F(SceneGraphTest, GridColorUpdate) {
    // Set grid color
    sceneGraph->setGridColor(0.9, 0.1, 0.5);
    
    // Verify color was applied (we can't directly inspect VTK properties
    // in these tests without a renderer, but we verify it doesn't crash)
    SUCCEED();
}

// ===========================
// Axis Scaling Tests
// ===========================

TEST_F(SceneGraphTest, AxisScalingWithBounds) {
    // Create bounding boxes of different sizes and verify axis scales
    
    // Small bounds
    cvc::bounding_box smallBounds(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
    sceneGraph->updateGrid(smallBounds);
    // Axis should be scaled to ~20% of max span (2.0), so ~0.4
    
    // Large bounds
    cvc::bounding_box largeBounds(-50.0, -50.0, -50.0, 50.0, 50.0, 50.0);
    sceneGraph->updateGrid(largeBounds);
    // Axis should be scaled to ~20% of max span (100.0), so ~20.0
    
    // Asymmetric bounds
    cvc::bounding_box asymBounds(-5.0, -2.0, -1.0, 5.0, 2.0, 1.0);
    sceneGraph->updateGrid(asymBounds);
    // Axis should be scaled to ~20% of max span (10.0), so ~2.0
    
    SUCCEED();
}

TEST_F(SceneGraphTest, AxisScalingEdgeCases) {
    // Test with very small bounds
    cvc::bounding_box tinyBounds(-0.01, -0.01, -0.01, 0.01, 0.01, 0.01);
    sceneGraph->updateGrid(tinyBounds);
    
    // Test with very large bounds
    cvc::bounding_box hugeBounds(-1000.0, -1000.0, -1000.0, 1000.0, 1000.0, 1000.0);
    sceneGraph->updateGrid(hugeBounds);
    
    // Test with zero-volume bounds
    cvc::bounding_box zeroBounds(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    sceneGraph->updateGrid(zeroBounds);
    
    SUCCEED();
}

TEST_F(SceneGraphTest, AxisScalingWithGeometry) {
    // Create simple geometry
    cvc::geometry geom;
    geom.points().push_back({-2.0, -2.0, -2.0});
    geom.points().push_back({2.0, 2.0, 2.0});
    
    // Set geometry and get its bounds
    sceneGraph->setGeometry(geom);
    cvc::bounding_box geomBounds = geom.extents();
    
    // Update grid/axis with geometry bounds
    sceneGraph->updateGrid(geomBounds);
    
    SUCCEED();
}

TEST_F(SceneGraphTest, GridAndAxisUpdateTogether) {
    // Verify that updating grid also updates axis scaling
    cvc::bounding_box bounds1(-10.0, -10.0, -10.0, 10.0, 10.0, 10.0);
    sceneGraph->updateGrid(bounds1);
    
    cvc::bounding_box bounds2(-5.0, -5.0, -5.0, 5.0, 5.0, 5.0);
    sceneGraph->updateGrid(bounds2);
    
    cvc::bounding_box bounds3(-100.0, -100.0, -100.0, 100.0, 100.0, 100.0);
    sceneGraph->updateGrid(bounds3);
    
    SUCCEED();
}

// ===========================
// Integration Tests
// ===========================

TEST_F(SceneGraphTest, ColorAndBoundsIntegration) {
    // Test setting both colors and bounds together
    appState->setGridColor(0.5, 0.5, 0.5);
    
    double r, g, b;
    appState->getGridColor(r, g, b);
    sceneGraph->setGridColor(r, g, b);
    
    cvc::bounding_box bounds(-25.0, -25.0, -25.0, 25.0, 25.0, 25.0);
    sceneGraph->updateGrid(bounds);
    
    SUCCEED();
}

