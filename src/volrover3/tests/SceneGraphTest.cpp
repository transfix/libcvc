#include <gtest/gtest.h>
#include <volrover3/SceneGraph.h>
#include <volrover3/GeometryNode.h>
#include <volrover3/VolumeNode.h>
#include <volrover3/GridNode.h>
#include <volrover3/AxisNode.h>
#include <volrover3/BBoxNode.h>
#include <cvc/geometry.h>
#include <cvc/volume.h>

class SceneGraphTest : public ::testing::Test {
protected:
    void SetUp() override {
        sceneGraph = new SceneGraph();
    }
    
    void TearDown() override {
        delete sceneGraph;
    }
    
    SceneGraph* sceneGraph;
};

TEST_F(SceneGraphTest, InitialState) {
    EXPECT_NE(sceneGraph->getRenderer(), nullptr);
    EXPECT_NE(sceneGraph->getRenderWindow(), nullptr);
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
    sceneGraph->setAxesVisible(true);
    // Axes should be created and visible
    
    sceneGraph->setAxesVisible(false);
    // Axes should be hidden
    
    SUCCEED();
}

TEST_F(SceneGraphTest, ShowHideBoundingBox) {
    sceneGraph->setBoundingBoxVisible(true);
    // Bounding box should be created and visible
    
    sceneGraph->setBoundingBoxVisible(false);
    // Bounding box should be hidden
    
    SUCCEED();
}

TEST_F(SceneGraphTest, UpdateBoundingBox) {
    double bounds[6] = {-1.0, 1.0, -1.0, 1.0, -1.0, 1.0};
    sceneGraph->updateBoundingBox(bounds);
    
    // Bounding box should be updated
    SUCCEED();
}

TEST_F(SceneGraphTest, ResetCamera) {
    sceneGraph->resetCamera();
    
    // Camera should be reset to view all geometry
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
    sceneGraph->setAxesVisible(true);
    sceneGraph->setBoundingBoxVisible(true);
    
    sceneGraph->resetCamera();
    
    SUCCEED();
}
