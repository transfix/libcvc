#include <gtest/gtest.h>
#include <QApplication>
#include <volrover3/AppState.h>
#include <cvc/geometry.h>
#include <cvc/volume.h>
#include <cvc/state.h>

// Need QApplication for Qt types
class AppStateTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!QApplication::instance()) {
            int argc = 0;
            char** argv = nullptr;
            app = new QApplication(argc, argv);
        }
    }
    
    void SetUp() override {
        // Create AppState with custom prefix for test isolation
        // Each test class gets its own state subtree
        state = std::make_unique<AppState>("appstate_test");
    }
    
    void TearDown() override {
        // Clean up test-specific state tree to prevent pollution between tests
        if (state) {
            state->getRootState().reset();
        }
        // Clean up test-specific state instance
        state.reset();
    }
    
    static QApplication* app;
    std::unique_ptr<AppState> state;
};

QApplication* AppStateTest::app = nullptr;

TEST_F(AppStateTest, SingletonInstance) {
    // Verify that the singleton instance is different from our test instance
    // (they use different state prefixes)
    AppState& singleton = AppState::instance();
    EXPECT_NE(state->getStatePrefix(), singleton.getStatePrefix());
    EXPECT_EQ(singleton.getStatePrefix(), "volrover3");
    EXPECT_EQ(state->getStatePrefix(), "appstate_test");
}

TEST_F(AppStateTest, CameraPosition) {
    state->setCameraPosition(1.0, 2.0, 3.0);
    
    double x, y, z;
    state->getCameraPosition(x, y, z);
    EXPECT_DOUBLE_EQ(x, 1.0);
    EXPECT_DOUBLE_EQ(y, 2.0);
    EXPECT_DOUBLE_EQ(z, 3.0);
}

TEST_F(AppStateTest, CameraSensitivity) {
    state->setCameraSensitivity(0.5);
    EXPECT_DOUBLE_EQ(state->cameraSensitivity(), 0.5);
    
    state->setCameraSensitivity(1.5);
    EXPECT_DOUBLE_EQ(state->cameraSensitivity(), 1.5);
}

TEST_F(AppStateTest, CameraSpeed) {
    state->setCameraSpeed(2.0);
    EXPECT_DOUBLE_EQ(state->cameraSpeed(), 2.0);
    
    state->setCameraSpeed(5.0);
    EXPECT_DOUBLE_EQ(state->cameraSpeed(), 5.0);
}

TEST_F(AppStateTest, KeyBindings) {
    state->setCameraKeyForward(Qt::Key_W);
    EXPECT_EQ(state->cameraKeyForward(), Qt::Key_W);
    
    state->setCameraKeyBackward(Qt::Key_S);
    EXPECT_EQ(state->cameraKeyBackward(), Qt::Key_S);
    
    state->setCameraKeyLeft(Qt::Key_A);
    EXPECT_EQ(state->cameraKeyLeft(), Qt::Key_A);
    
    state->setCameraKeyRight(Qt::Key_D);
    EXPECT_EQ(state->cameraKeyRight(), Qt::Key_D);
    
    state->setCameraKeyUp(Qt::Key_E);
    EXPECT_EQ(state->cameraKeyUp(), Qt::Key_E);
    
    state->setCameraKeyDown(Qt::Key_Q);
    EXPECT_EQ(state->cameraKeyDown(), Qt::Key_Q);
}

TEST_F(AppStateTest, TransferFunctionColorTable) {
    std::vector<double> colorTable = {0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0};
    state->setTransferFunctionColorTable(colorTable);
    
    auto retrieved = state->transferFunctionColorTable();
    ASSERT_EQ(retrieved.size(), colorTable.size());
    for (size_t i = 0; i < colorTable.size(); ++i) {
        EXPECT_DOUBLE_EQ(retrieved[i], colorTable[i]);
    }
}

TEST_F(AppStateTest, TransferFunctionOpacityTable) {
    std::vector<double> opacityTable = {0.0, 0.0, 0.5, 0.5, 1.0, 1.0};
    state->setTransferFunctionOpacityTable(opacityTable);
    
    auto retrieved = state->transferFunctionOpacityTable();
    ASSERT_EQ(retrieved.size(), opacityTable.size());
    for (size_t i = 0; i < opacityTable.size(); ++i) {
        EXPECT_DOUBLE_EQ(retrieved[i], opacityTable[i]);
    }
}

TEST_F(AppStateTest, GeometryStorage) {
    // Create a simple geometry
    cvc::geometry geom;
    geom.points().push_back({0.0, 0.0, 0.0});
    geom.points().push_back({1.0, 0.0, 0.0});
    geom.points().push_back({0.0, 1.0, 0.0});
    
    state->setGeometry(geom);
    
    auto retrieved = state->geometry();
    EXPECT_EQ(retrieved.points().size(), size_t(3));
}

TEST_F(AppStateTest, VolumeStorage) {
    // Create a simple volume
    cvc::volume vol(cvc::dimension(2, 2, 2), cvc::UChar);
    
    state->setVolume(vol);
    
    auto retrieved = state->volume();
    EXPECT_EQ(retrieved.XDim(), size_t(2));
    EXPECT_EQ(retrieved.YDim(), size_t(2));
    EXPECT_EQ(retrieved.ZDim(), size_t(2));
}

// ===========================
// State Tree Tests
// ===========================

TEST_F(AppStateTest, StateTreeCameraPosition) {
    // Set camera position via AppState
    state->setCameraPosition(10.0, 20.0, 30.0);
    
    // Verify values are stored in state tree
    auto& stateTree = state->getRootState();
    EXPECT_DOUBLE_EQ(stateTree("camera_position_x").value<double>(), 10.0);
    EXPECT_DOUBLE_EQ(stateTree("camera_position_y").value<double>(), 20.0);
    EXPECT_DOUBLE_EQ(stateTree("camera_position_z").value<double>(), 30.0);
    
    // Verify getters match state tree values
    double x, y, z;
    state->getCameraPosition(x, y, z);
    EXPECT_DOUBLE_EQ(x, 10.0);
    EXPECT_DOUBLE_EQ(y, 20.0);
    EXPECT_DOUBLE_EQ(z, 30.0);
}

TEST_F(AppStateTest, StateTreeCameraViewDirection) {
    state->setCameraViewDirection(1.0, 0.0, 0.0);
    
    auto& stateTree = state->getRootState();
    EXPECT_DOUBLE_EQ(stateTree("camera_view_dir_x").value<double>(), 1.0);
    EXPECT_DOUBLE_EQ(stateTree("camera_view_dir_y").value<double>(), 0.0);
    EXPECT_DOUBLE_EQ(stateTree("camera_view_dir_z").value<double>(), 0.0);
    
    double x, y, z;
    state->getCameraViewDirection(x, y, z);
    EXPECT_DOUBLE_EQ(x, 1.0);
    EXPECT_DOUBLE_EQ(y, 0.0);
    EXPECT_DOUBLE_EQ(z, 0.0);
}

TEST_F(AppStateTest, StateTreeCameraUpVector) {
    state->setCameraUpVector(0.0, 1.0, 0.0);
    
    auto& stateTree = state->getRootState();
    EXPECT_DOUBLE_EQ(stateTree("camera_up_x").value<double>(), 0.0);
    EXPECT_DOUBLE_EQ(stateTree("camera_up_y").value<double>(), 1.0);
    EXPECT_DOUBLE_EQ(stateTree("camera_up_z").value<double>(), 0.0);
    
    double x, y, z;
    state->getCameraUpVector(x, y, z);
    EXPECT_DOUBLE_EQ(x, 0.0);
    EXPECT_DOUBLE_EQ(y, 1.0);
    EXPECT_DOUBLE_EQ(z, 0.0);
}

TEST_F(AppStateTest, StateTreeCameraFOV) {
    state->setCameraFieldOfView(60.0);
    
    auto& stateTree = state->getRootState();
    EXPECT_DOUBLE_EQ(stateTree("camera_fov").value<double>(), 60.0);
    EXPECT_DOUBLE_EQ(state->cameraFieldOfView(), 60.0);
}

TEST_F(AppStateTest, StateTreeCameraSpeed) {
    state->setCameraSpeed(3.5);
    
    auto& stateTree = state->getRootState();
    EXPECT_DOUBLE_EQ(stateTree("camera_speed").value<double>(), 3.5);
    EXPECT_DOUBLE_EQ(state->cameraSpeed(), 3.5);
}

TEST_F(AppStateTest, StateTreeCameraSensitivity) {
    state->setCameraSensitivity(0.75);
    
    auto& stateTree = state->getRootState();
    EXPECT_DOUBLE_EQ(stateTree("camera_sensitivity").value<double>(), 0.75);
    EXPECT_DOUBLE_EQ(state->cameraSensitivity(), 0.75);
}

TEST_F(AppStateTest, StateTreeWorldBounds) {
    cvc::bounding_box bounds(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
    state->setWorldBounds(bounds);
    
    auto& stateTree = state->getRootState();
    std::vector<std::string> values = stateTree("world_bounds").values();
    ASSERT_EQ(values.size(), size_t(6));
    
    auto retrieved = state->worldBounds();
    EXPECT_DOUBLE_EQ(retrieved[0], 1.0);
    EXPECT_DOUBLE_EQ(retrieved[1], 2.0);
    EXPECT_DOUBLE_EQ(retrieved[2], 3.0);
    EXPECT_DOUBLE_EQ(retrieved[3], 4.0);
    EXPECT_DOUBLE_EQ(retrieved[4], 5.0);
    EXPECT_DOUBLE_EQ(retrieved[5], 6.0);
}

TEST_F(AppStateTest, StateTreeGridVisible) {
    state->setGridVisible(true);
    
    auto& stateTree = state->getRootState();
    EXPECT_TRUE(stateTree("grid_visible").value<bool>());
    EXPECT_TRUE(state->gridVisible());
    
    state->setGridVisible(false);
    EXPECT_FALSE(stateTree("grid_visible").value<bool>());
    EXPECT_FALSE(state->gridVisible());
}

TEST_F(AppStateTest, StateTreeAxisVisible) {
    state->setAxisVisible(true);
    
    auto& stateTree = state->getRootState();
    EXPECT_TRUE(stateTree("axis_visible").value<bool>());
    EXPECT_TRUE(state->axisVisible());
    
    state->setAxisVisible(false);
    EXPECT_FALSE(stateTree("axis_visible").value<bool>());
    EXPECT_FALSE(state->axisVisible());
}

TEST_F(AppStateTest, StateTreeKeyBindings) {
    state->setCameraKeyForward(Qt::Key_W);
    state->setCameraKeyBackward(Qt::Key_S);
    state->setCameraKeyLeft(Qt::Key_A);
    state->setCameraKeyRight(Qt::Key_D);
    state->setCameraKeyUp(Qt::Key_E);
    state->setCameraKeyDown(Qt::Key_Q);
    
    auto& stateTree = state->getRootState();
    EXPECT_EQ(stateTree("camera_key_forward").value<int>(), Qt::Key_W);
    EXPECT_EQ(stateTree("camera_key_backward").value<int>(), Qt::Key_S);
    EXPECT_EQ(stateTree("camera_key_left").value<int>(), Qt::Key_A);
    EXPECT_EQ(stateTree("camera_key_right").value<int>(), Qt::Key_D);
    EXPECT_EQ(stateTree("camera_key_up").value<int>(), Qt::Key_E);
    EXPECT_EQ(stateTree("camera_key_down").value<int>(), Qt::Key_Q);
}

TEST_F(AppStateTest, StateTreeDirectUpdate) {
    // Set values directly in state tree (simulating external update)
    auto& stateTree = state->getRootState();
    stateTree("camera_position_x").value(100.0);
    stateTree("camera_position_y").value(200.0);
    stateTree("camera_position_z").value(300.0);
    
    // Verify AppState reads from state tree
    double x, y, z;
    state->getCameraPosition(x, y, z);
    EXPECT_DOUBLE_EQ(x, 100.0);
    EXPECT_DOUBLE_EQ(y, 200.0);
    EXPECT_DOUBLE_EQ(z, 300.0);
}

// ===========================
// Callback Tests
// ===========================

TEST_F(AppStateTest, CameraChangedCallback) {
    int callback_count = 0;
    
    auto connection = state->onCameraChanged([&callback_count]() {
        callback_count++;
    });
    
    // Clear camera_changed flag first
    auto& stateTree = state->getRootState();
    stateTree("camera_changed").value(false);
    
    // Trigger camera changes
    state->setCameraPosition(1.0, 2.0, 3.0);
    EXPECT_GT(callback_count, 0);
    
    // Reset for next change
    stateTree("camera_changed").value(false);
    int prev_count = callback_count;
    state->setCameraViewDirection(0.0, 0.0, 1.0);
    EXPECT_GT(callback_count, prev_count);
    
    stateTree("camera_changed").value(false);
    prev_count = callback_count;
    state->setCameraUpVector(0.0, 1.0, 0.0);
    EXPECT_GT(callback_count, prev_count);
    
    stateTree("camera_changed").value(false);
    prev_count = callback_count;
    state->setCameraFieldOfView(45.0);
    EXPECT_GT(callback_count, prev_count);
    
    // Disconnect and verify no more callbacks
    connection.disconnect();
    stateTree("camera_changed").value(false);
    prev_count = callback_count;
    state->setCameraPosition(99.0, 99.0, 99.0);
    EXPECT_EQ(callback_count, prev_count);  // Should not have incremented
}

TEST_F(AppStateTest, WorldBoundsChangedCallback) {
    int callback_count = 0;
    
    auto connection = state->onWorldBoundsChanged([&callback_count]() {
        callback_count++;
    });
    
    cvc::bounding_box bounds(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
    state->setWorldBounds(bounds);
    
    EXPECT_GT(callback_count, 0);
    
    connection.disconnect();
}

TEST_F(AppStateTest, GridVisibilityChangedCallback) {
    int callback_count = 0;
    
    // Ensure we know the current state
    state->setGridVisible(true);
    
    auto connection = state->onGridVisibilityChanged([&callback_count]() {
        callback_count++;
    });
    
    // Now change to false (should trigger)
    state->setGridVisible(false);
    EXPECT_GT(callback_count, 0);
    
    int prev_count = callback_count;
    state->setGridVisible(true);
    EXPECT_GT(callback_count, prev_count);
    
    connection.disconnect();
}

TEST_F(AppStateTest, AxisVisibilityChangedCallback) {
    int callback_count = 0;
    
    // Ensure we know the current state
    state->setAxisVisible(true);
    
    auto connection = state->onAxisVisibilityChanged([&callback_count]() {
        callback_count++;
    });
    
    // Now change to false (should trigger)
    state->setAxisVisible(false);
    EXPECT_GT(callback_count, 0);
    
    int prev_count = callback_count;
    state->setAxisVisible(true);
    EXPECT_GT(callback_count, prev_count);
    
    connection.disconnect();
}

TEST_F(AppStateTest, GeometryChangedCallback) {
    int callback_count = 0;
    
    auto connection = state->onGeometryChanged([&callback_count]() {
        callback_count++;
    });
    
    // Clear geometry_changed flag first
    auto& stateTree = state->getRootState();
    stateTree("geometry_changed").value(false);
    
    cvc::geometry geom;
    geom.points().push_back({0.0, 0.0, 0.0});
    state->setGeometry(geom);
    
    EXPECT_GT(callback_count, 0);
    
    connection.disconnect();
}

TEST_F(AppStateTest, VolumeChangedCallback) {
    int callback_count = 0;
    
    auto connection = state->onVolumeChanged([&callback_count]() {
        callback_count++;
    });
    
    // Clear volume_changed flag first
    auto& stateTree = state->getRootState();
    stateTree("volume_changed").value(false);
    
    cvc::volume vol(cvc::dimension(2, 2, 2), cvc::UChar);
    state->setVolume(vol);
    
    EXPECT_GT(callback_count, 0);
    
    connection.disconnect();
}

TEST_F(AppStateTest, MultipleCallbacksForSameState) {
    int callback1_count = 0;
    int callback2_count = 0;
    int callback3_count = 0;
    
    auto conn1 = state->onCameraChanged([&callback1_count]() { callback1_count++; });
    auto conn2 = state->onCameraChanged([&callback2_count]() { callback2_count++; });
    auto conn3 = state->onCameraChanged([&callback3_count]() { callback3_count++; });
    
    // Clear camera_changed flag first
    auto& stateTree = state->getRootState();
    stateTree("camera_changed").value(false);
    
    state->setCameraPosition(5.0, 5.0, 5.0);
    
    EXPECT_GT(callback1_count, 0);
    EXPECT_GT(callback2_count, 0);
    EXPECT_GT(callback3_count, 0);
    
    conn1.disconnect();
    conn2.disconnect();
    conn3.disconnect();
}

TEST_F(AppStateTest, CallbackReceivesCorrectValue) {
    double captured_x = 0.0;
    double captured_y = 0.0;
    double captured_z = 0.0;
    
    auto connection = state->onCameraChanged([this, &captured_x, &captured_y, &captured_z]() {
        state->getCameraPosition(captured_x, captured_y, captured_z);
    });
    
    // Clear camera_changed flag first
    auto& stateTree = state->getRootState();
    stateTree("camera_changed").value(false);
    
    state->setCameraPosition(7.0, 8.0, 9.0);
    
    EXPECT_DOUBLE_EQ(captured_x, 7.0);
    EXPECT_DOUBLE_EQ(captured_y, 8.0);
    EXPECT_DOUBLE_EQ(captured_z, 9.0);
    
    connection.disconnect();
}

TEST_F(AppStateTest, StateTreeTriggerCallback) {
    // This test demonstrates that callbacks can be triggered by directly
    // setting the state tree value.
    int callback_count = 0;
    
    auto connection = state->onCameraChanged([&callback_count]() {
        callback_count++;
    });
    
    // Clear camera_changed flag first
    auto& stateTree = state->getRootState();
    int before_count = callback_count;
    stateTree("camera_changed").value(false);
    
    // Trigger callback by setting state tree directly
    stateTree("camera_changed").value(true);
    
    EXPECT_GT(callback_count, before_count);
    
    connection.disconnect();
}

TEST_F(AppStateTest, CallbackDisconnection) {
    int callback_count = 0;
    
    auto connection = state->onGridVisibilityChanged([&callback_count]() {
        callback_count++;
    });
    
    // Trigger callback
    state->setGridVisible(false);
    EXPECT_EQ(callback_count, 1);
    
    // Disconnect
    connection.disconnect();
    
    // Trigger again - should not fire
    state->setGridVisible(true);
    EXPECT_EQ(callback_count, 1);  // Should still be 1
}

// ===========================
// State Persistence Tests
// ===========================

TEST_F(AppStateTest, StateTreeInitialized) {
    auto& stateTree = state->getRootState();
    
    // Verify default state values are initialized
    EXPECT_TRUE(stateTree("camera_position_x").initialized());
    EXPECT_TRUE(stateTree("camera_position_y").initialized());
    EXPECT_TRUE(stateTree("camera_position_z").initialized());
    EXPECT_TRUE(stateTree("camera_speed").initialized());
    EXPECT_TRUE(stateTree("camera_sensitivity").initialized());
    EXPECT_TRUE(stateTree("camera_fov").initialized());
}

TEST_F(AppStateTest, StateTreePersistence) {
    // Set values
    state->setCameraPosition(11.0, 22.0, 33.0);
    state->setCameraSpeed(5.5);
    state->setGridVisible(true);
    
    // Verify values persist in state tree
    auto& stateTree = state->getRootState();
    EXPECT_DOUBLE_EQ(stateTree("camera_position_x").value<double>(), 11.0);
    EXPECT_DOUBLE_EQ(stateTree("camera_position_y").value<double>(), 22.0);
    EXPECT_DOUBLE_EQ(stateTree("camera_position_z").value<double>(), 33.0);
    EXPECT_DOUBLE_EQ(stateTree("camera_speed").value<double>(), 5.5);
    EXPECT_TRUE(stateTree("grid_visible").value<bool>());
    
    // Values should persist across multiple reads
    EXPECT_DOUBLE_EQ(stateTree("camera_position_x").value<double>(), 11.0);
    EXPECT_DOUBLE_EQ(stateTree("camera_speed").value<double>(), 5.5);
}

// ===========================
// Color State Tests
// ===========================

TEST_F(AppStateTest, GridColor) {
    // Set grid color
    state->setGridColor(0.8, 0.6, 0.4);
    
    // Read back
    double r, g, b;
    state->getGridColor(r, g, b);
    EXPECT_DOUBLE_EQ(r, 0.8);
    EXPECT_DOUBLE_EQ(g, 0.6);
    EXPECT_DOUBLE_EQ(b, 0.4);
}

TEST_F(AppStateTest, GeometryBBoxColor) {
    // Set geometry bbox color
    state->setGeometryBBoxColor(1.0, 0.0, 0.5);
    
    // Read back
    double r, g, b;
    state->getGeometryBBoxColor(r, g, b);
    EXPECT_DOUBLE_EQ(r, 1.0);
    EXPECT_DOUBLE_EQ(g, 0.0);
    EXPECT_DOUBLE_EQ(b, 0.5);
}

TEST_F(AppStateTest, VolumeBBoxColor) {
    // Set volume bbox color
    state->setVolumeBBoxColor(0.2, 0.9, 0.7);
    
    // Read back
    double r, g, b;
    state->getVolumeBBoxColor(r, g, b);
    EXPECT_DOUBLE_EQ(r, 0.2);
    EXPECT_DOUBLE_EQ(g, 0.9);
    EXPECT_DOUBLE_EQ(b, 0.7);
}

TEST_F(AppStateTest, StateTreeGridColor) {
    // Set via state tree
    auto& stateTree = state->getRootState();
    stateTree("grid_color").value("0.1,0.2,0.3");
    
    // Read via AppState
    double r, g, b;
    state->getGridColor(r, g, b);
    EXPECT_DOUBLE_EQ(r, 0.1);
    EXPECT_DOUBLE_EQ(g, 0.2);
    EXPECT_DOUBLE_EQ(b, 0.3);
}

TEST_F(AppStateTest, StateTreeGeometryBBoxColor) {
    // Set via AppState
    state->setGeometryBBoxColor(0.7, 0.8, 0.9);
    
    // Verify in state tree
    auto& stateTree = state->getRootState();
    std::string colorStr = stateTree("geometry_bbox_color").value<std::string>();
    // Check individual components due to floating point representation
    EXPECT_TRUE(colorStr.find("0.7") != std::string::npos || 
                colorStr.find("0.69999") != std::string::npos);
}

TEST_F(AppStateTest, StateTreeVolumeBBoxColor) {
    // Set via AppState
    state->setVolumeBBoxColor(0.25, 0.5, 0.75);
    
    // Verify in state tree
    auto& stateTree = state->getRootState();
    std::string colorStr = stateTree("volume_bbox_color").value<std::string>();
    EXPECT_EQ(colorStr, "0.25,0.5,0.75");
}

TEST_F(AppStateTest, ColorDefaultValues) {
    // Reset to default values first
    auto& stateTree = state->getRootState();
    stateTree("grid_color").value("0.5,0.5,0.5");
    stateTree("geometry_bbox_color").value("0.0,1.0,0.0");
    stateTree("volume_bbox_color").value("1.0,0.0,1.0");
    
    // Test default values from fresh state
    double r, g, b;
    
    // Grid should default to gray
    state->getGridColor(r, g, b);
    EXPECT_DOUBLE_EQ(r, 0.5);
    EXPECT_DOUBLE_EQ(g, 0.5);
    EXPECT_DOUBLE_EQ(b, 0.5);
    
    // Geometry bbox should default to green
    state->getGeometryBBoxColor(r, g, b);
    EXPECT_DOUBLE_EQ(r, 0.0);
    EXPECT_DOUBLE_EQ(g, 1.0);
    EXPECT_DOUBLE_EQ(b, 0.0);
    
    // Volume bbox should default to magenta
    state->getVolumeBBoxColor(r, g, b);
    EXPECT_DOUBLE_EQ(r, 1.0);
    EXPECT_DOUBLE_EQ(g, 0.0);
    EXPECT_DOUBLE_EQ(b, 1.0);
}

TEST_F(AppStateTest, GridColorChangedCallback) {
    int callback_count = 0;
    
    auto connection = state->onGridColorChanged([&callback_count]() {
        callback_count++;
    });
    
    // Trigger color change
    state->setGridColor(1.0, 0.5, 0.0);
    EXPECT_GT(callback_count, 0);
    
    int prev_count = callback_count;
    state->setGridColor(0.0, 0.5, 1.0);
    EXPECT_GT(callback_count, prev_count);
    
    // Cleanup
    connection.disconnect();
}

TEST_F(AppStateTest, GeometryBBoxColorChangedCallback) {
    int callback_count = 0;
    
    auto connection = state->onGeometryBBoxColorChanged([&callback_count]() {
        callback_count++;
    });
    
    // Trigger color change
    state->setGeometryBBoxColor(0.3, 0.6, 0.9);
    EXPECT_GT(callback_count, 0);
    
    // Cleanup
    connection.disconnect();
}

TEST_F(AppStateTest, VolumeBBoxColorChangedCallback) {
    int callback_count = 0;
    
    auto connection = state->onVolumeBBoxColorChanged([&callback_count]() {
        callback_count++;
    });
    
    // Trigger color change
    state->setVolumeBBoxColor(0.9, 0.6, 0.3);
    EXPECT_GT(callback_count, 0);
    
    // Cleanup
    connection.disconnect();
}

TEST_F(AppStateTest, ColorBoundaryValues) {
    // Test with boundary values (0.0 and 1.0)
    state->setGridColor(0.0, 1.0, 0.0);
    
    double r, g, b;
    state->getGridColor(r, g, b);
    EXPECT_DOUBLE_EQ(r, 0.0);
    EXPECT_DOUBLE_EQ(g, 1.0);
    EXPECT_DOUBLE_EQ(b, 0.0);
    
    // Test all zeros
    state->setVolumeBBoxColor(0.0, 0.0, 0.0);
    state->getVolumeBBoxColor(r, g, b);
    EXPECT_DOUBLE_EQ(r, 0.0);
    EXPECT_DOUBLE_EQ(g, 0.0);
    EXPECT_DOUBLE_EQ(b, 0.0);
    
    // Test all ones
    state->setGeometryBBoxColor(1.0, 1.0, 1.0);
    state->getGeometryBBoxColor(r, g, b);
    EXPECT_DOUBLE_EQ(r, 1.0);
    EXPECT_DOUBLE_EQ(g, 1.0);
    EXPECT_DOUBLE_EQ(b, 1.0);
}

// ===========================
// Grid-Specific Tests
// ===========================

TEST_F(AppStateTest, GridPlaneVisibility) {
    state->setGridYZPlaneVisible(true);
    EXPECT_TRUE(state->gridYZPlaneVisible());
    
    state->setGridXZPlaneVisible(false);
    EXPECT_FALSE(state->gridXZPlaneVisible());
    
    state->setGridXYPlaneVisible(true);
    EXPECT_TRUE(state->gridXYPlaneVisible());
}

TEST_F(AppStateTest, GridDivisions) {
    state->setGridDivisions(16, 32, 64);
    
    int x, y, z;
    state->getGridDivisions(x, y, z);
    EXPECT_EQ(x, 16);
    EXPECT_EQ(y, 32);
    EXPECT_EQ(z, 64);
}

TEST_F(AppStateTest, GridTickIntervals) {
    state->setGridTickIntervals(4, 8, 16);
    
    int x, y, z;
    state->getGridTickIntervals(x, y, z);
    EXPECT_EQ(x, 4);
    EXPECT_EQ(y, 8);
    EXPECT_EQ(z, 16);
}

TEST_F(AppStateTest, GridTicksVisible) {
    // Reset to test default value
    state->getRootState()("grid_ticks_visible").reset();
    
    // Default should be true
    EXPECT_TRUE(state->gridTicksVisible());
    
    state->setGridTicksVisible(false);
    EXPECT_FALSE(state->gridTicksVisible());
    
    state->setGridTicksVisible(true);
    EXPECT_TRUE(state->gridTicksVisible());
}

TEST_F(AppStateTest, GridPlaneColors) {
    state->setGridYZPlaneColor(1.0, 0.0, 0.0);
    state->setGridXZPlaneColor(0.0, 1.0, 0.0);
    state->setGridXYPlaneColor(0.0, 0.0, 1.0);
    
    double r, g, b;
    state->getGridYZPlaneColor(r, g, b);
    EXPECT_DOUBLE_EQ(r, 1.0);
    EXPECT_DOUBLE_EQ(g, 0.0);
    EXPECT_DOUBLE_EQ(b, 0.0);
    
    state->getGridXZPlaneColor(r, g, b);
    EXPECT_DOUBLE_EQ(r, 0.0);
    EXPECT_DOUBLE_EQ(g, 1.0);
    EXPECT_DOUBLE_EQ(b, 0.0);
    
    state->getGridXYPlaneColor(r, g, b);
    EXPECT_DOUBLE_EQ(r, 0.0);
    EXPECT_DOUBLE_EQ(g, 0.0);
    EXPECT_DOUBLE_EQ(b, 1.0);
}

TEST_F(AppStateTest, GridTickLabelProperties) {
    state->setGridTickLabelColor(0.8, 0.6, 0.4);
    state->setGridTickLabelFontSize(20);
    
    double r, g, b;
    state->getGridTickLabelColor(r, g, b);
    EXPECT_DOUBLE_EQ(r, 0.8);
    EXPECT_DOUBLE_EQ(g, 0.6);
    EXPECT_DOUBLE_EQ(b, 0.4);
    
    EXPECT_EQ(state->gridTickLabelFontSize(), 20);
}

TEST_F(AppStateTest, GridPlaneVisibilityCallbacks) {
    int callback_count = 0;
    
    auto connection = state->onGridPlaneVisibilityChanged([&callback_count]() {
        callback_count++;
    });
    
    // Toggle from current state to trigger callbacks
    bool currentYZ = state->gridYZPlaneVisible();
    state->setGridYZPlaneVisible(!currentYZ);
    EXPECT_GT(callback_count, 0);
    
    int prev_count = callback_count;
    bool currentXZ = state->gridXZPlaneVisible();
    state->setGridXZPlaneVisible(!currentXZ);
    EXPECT_GT(callback_count, prev_count);
    
    prev_count = callback_count;
    bool currentXY = state->gridXYPlaneVisible();
    state->setGridXYPlaneVisible(!currentXY);
    EXPECT_GT(callback_count, prev_count);
    
    connection.disconnect();
}

TEST_F(AppStateTest, GridDivisionsCallbacks) {
    int callback_count = 0;
    
    auto connection = state->onGridDivisionsChanged([&callback_count]() {
        callback_count++;
    });
    
    state->setGridDivisions(10, 20, 30);
    EXPECT_GT(callback_count, 0);
    
    connection.disconnect();
}

TEST_F(AppStateTest, GridTickIntervalsCallbacks) {
    int callback_count = 0;
    
    auto connection = state->onGridTickIntervalsChanged([&callback_count]() {
        callback_count++;
    });
    
    state->setGridTickIntervals(5, 10, 15);
    EXPECT_GT(callback_count, 0);
    
    connection.disconnect();
}

TEST_F(AppStateTest, GridTicksVisibleCallbacks) {
    int callback_count = 0;
    
    // Set initial value to true so we can detect change to false
    state->setGridTicksVisible(true);
    
    auto connection = state->onGridTicksVisibleChanged([&callback_count]() {
        callback_count++;
    });
    
    state->setGridTicksVisible(false);
    EXPECT_GT(callback_count, 0);
    
    int prev_count = callback_count;
    state->setGridTicksVisible(true);
    EXPECT_GT(callback_count, prev_count);
    
    connection.disconnect();
}

TEST_F(AppStateTest, GridPlaneColorsCallbacks) {
    int callback_count = 0;
    
    auto connection = state->onGridPlaneColorsChanged([&callback_count]() {
        callback_count++;
    });
    
    state->setGridYZPlaneColor(1.0, 0.5, 0.0);
    EXPECT_GT(callback_count, 0);
    
    connection.disconnect();
}

TEST_F(AppStateTest, GridTickLabelPropertiesCallbacks) {
    int callback_count = 0;
    
    
    auto connection = state->onGridTickLabelPropertiesChanged([&callback_count]() {
        callback_count++;
    });
    
    state->setGridTickLabelColor(0.5, 0.5, 0.5);
    EXPECT_GT(callback_count, 0);
    
    int prev_count = callback_count;
    state->setGridTickLabelFontSize(18);
    EXPECT_GT(callback_count, prev_count);
    
    connection.disconnect();
}

// Test computeGraphicsBounds with no graphics
TEST_F(AppStateTest, ComputeGraphicsBounds_Empty) {
    // Get graphics state through root
    cvc::state& rootState = state->getRootState();
    cvc::state& graphicsState = rootState("graphics");
    
    // Clear children by creating a fresh graphics state
    // (The state system will handle this)
    
    // Should return an invalid/empty bounding box
    cvc::bounding_box result = state->computeGraphicsBounds();
    // No assertion since we just verify it doesn't crash
}

// Test computeGraphicsBounds with single untransformed object
TEST_F(AppStateTest, ComputeGraphicsBounds_SingleObject) {
    // Get graphics state through root
    cvc::state& rootState = state->getRootState();
    cvc::state& graphicsState = rootState("graphics");
    cvc::state& childrenState = graphicsState("children");
    
    // Add a single graphics object with a known bounding box in metadata
    cvc::state& obj1 = childrenState("test_object1");
    obj1("metadata")("bounding_box").value("-1,-2,-3,1,2,3");
    
    cvc::bounding_box result = state->computeGraphicsBounds();
    EXPECT_DOUBLE_EQ(result.minx, -1.0);
    EXPECT_DOUBLE_EQ(result.miny, -2.0);
    EXPECT_DOUBLE_EQ(result.minz, -3.0);
    EXPECT_DOUBLE_EQ(result.maxx, 1.0);
    EXPECT_DOUBLE_EQ(result.maxy, 2.0);
    EXPECT_DOUBLE_EQ(result.maxz, 3.0);
}

// Test computeGraphicsBounds with multiple untransformed objects
TEST_F(AppStateTest, ComputeGraphicsBounds_MultipleObjects) {
    // Get graphics state through root
    cvc::state& rootState = state->getRootState();
    cvc::state& graphicsState = rootState("graphics");
    cvc::state& childrenState = graphicsState("children");
    
    // Add multiple graphics objects
    cvc::state& obj1 = childrenState("test_multi_object1");
    obj1("metadata")("bounding_box").value("0,0,0,1,1,1");
    
    cvc::state& obj2 = childrenState("test_multi_object2");
    obj2("metadata")("bounding_box").value("-2,-2,-2,-1,-1,-1");
    
    cvc::state& obj3 = childrenState("test_multi_object3");
    obj3("metadata")("bounding_box").value("2,2,2,3,3,3");
    
    cvc::bounding_box result = state->computeGraphicsBounds();
    // Should be union of all three: [-2,-2,-2] to [3,3,3]
    EXPECT_DOUBLE_EQ(result.minx, -2.0);
    EXPECT_DOUBLE_EQ(result.miny, -2.0);
    EXPECT_DOUBLE_EQ(result.minz, -2.0);
    EXPECT_DOUBLE_EQ(result.maxx, 3.0);
    EXPECT_DOUBLE_EQ(result.maxy, 3.0);
    EXPECT_DOUBLE_EQ(result.maxz, 3.0);
}

// Test computeGraphicsBounds with translation transform
TEST_F(AppStateTest, ComputeGraphicsBounds_Translation) {
    cvc::state& rootState = state->getRootState();
    cvc::state& graphicsState = rootState("graphics");
    cvc::state& childrenState = graphicsState("children");
    
    // Add object with unit cube bbox centered at origin: [-0.5, -0.5, -0.5] to [0.5, 0.5, 0.5]
    cvc::state& obj1 = childrenState("test_trans_obj1");
    obj1("metadata")("bounding_box").value("-0.5,-0.5,-0.5,0.5,0.5,0.5");
    
    // Translation matrix: translate by (10, 20, 30)
    // Identity matrix with translation in last column
    std::string transform = "1,0,0,10,"  // row 0
                            "0,1,0,20,"  // row 1
                            "0,0,1,30,"  // row 2
                            "0,0,0,1";   // row 3
    obj1("transform").value(transform);
    
    cvc::bounding_box result = state->computeGraphicsBounds();
    // Cube should be translated to [9.5, 19.5, 29.5] to [10.5, 20.5, 30.5]
    EXPECT_NEAR(result.minx, 9.5, 1e-6);
    EXPECT_NEAR(result.miny, 19.5, 1e-6);
    EXPECT_NEAR(result.minz, 29.5, 1e-6);
    EXPECT_NEAR(result.maxx, 10.5, 1e-6);
    EXPECT_NEAR(result.maxy, 20.5, 1e-6);
    EXPECT_NEAR(result.maxz, 30.5, 1e-6);
}

// Test computeGraphicsBounds with scale transform
TEST_F(AppStateTest, ComputeGraphicsBounds_Scale) {
    cvc::state& rootState = state->getRootState();
    cvc::state& graphicsState = rootState("graphics");
    cvc::state& childrenState = graphicsState("children");
    
    // Add object with unit cube bbox: [0, 0, 0] to [1, 1, 1]
    cvc::state& obj1 = childrenState("test_scale_obj");
    obj1("metadata")("bounding_box").value("0,0,0,1,1,1");
    
    // Scale matrix: scale by (2, 3, 4)
    std::string transform = "2,0,0,0,"  // row 0
                            "0,3,0,0,"  // row 1
                            "0,0,4,0,"  // row 2
                            "0,0,0,1";  // row 3
    obj1("transform").value(transform);
    
    cvc::bounding_box result = state->computeGraphicsBounds();
    // Cube should be scaled to [0, 0, 0] to [2, 3, 4]
    EXPECT_NEAR(result.minx, 0.0, 1e-6);
    EXPECT_NEAR(result.miny, 0.0, 1e-6);
    EXPECT_NEAR(result.minz, 0.0, 1e-6);
    EXPECT_NEAR(result.maxx, 2.0, 1e-6);
    EXPECT_NEAR(result.maxy, 3.0, 1e-6);
    EXPECT_NEAR(result.maxz, 4.0, 1e-6);
}

// Test computeGraphicsBounds with rotation transform (45 degrees around Z)
TEST_F(AppStateTest, ComputeGraphicsBounds_Rotation) {
    cvc::state& rootState = state->getRootState();
    cvc::state& graphicsState = rootState("graphics");
    cvc::state& childrenState = graphicsState("children");
    
    // Add object with bbox: [-1, -1, 0] to [1, 1, 0] (square in XY plane)
    cvc::state& obj1 = childrenState("test_rotation_obj");
    obj1("metadata")("bounding_box").value("-1,-1,0,1,1,0");
    
    // 45 degree rotation around Z axis
    // cos(45) = sin(45) = sqrt(2)/2 ≈ 0.707107
    double c = 0.707107;
    double s = 0.707107;
    std::stringstream ss;
    ss << c << "," << -s << ",0,0,"  // row 0
       << s << "," << c << ",0,0,"   // row 1
       << "0,0,1,0,"                  // row 2
       << "0,0,0,1";                  // row 3
    obj1("transform").value(ss.str());
    
    cvc::bounding_box result = state->computeGraphicsBounds();
    // After 45 degree rotation, diagonal of square becomes axis-aligned
    // Expected bbox: approximately [-sqrt(2), -sqrt(2), 0] to [sqrt(2), sqrt(2), 0]
    double expected = 1.414214; // sqrt(2)
    EXPECT_NEAR(result.minx, -expected, 1e-4);
    EXPECT_NEAR(result.miny, -expected, 1e-4);
    EXPECT_NEAR(result.minz, 0.0, 1e-6);
    EXPECT_NEAR(result.maxx, expected, 1e-4);
    EXPECT_NEAR(result.maxy, expected, 1e-4);
    EXPECT_NEAR(result.maxz, 0.0, 1e-6);
}

// Test computeGraphicsBounds with combined transform (scale + translate)
TEST_F(AppStateTest, ComputeGraphicsBounds_Combined) {
    cvc::state& rootState = state->getRootState();
    cvc::state& graphicsState = rootState("graphics");
    cvc::state& childrenState = graphicsState("children");
    
    // Add object with unit cube: [0, 0, 0] to [1, 1, 1]
    cvc::state& obj1 = childrenState("test_combined_obj");
    obj1("metadata")("bounding_box").value("0,0,0,1,1,1");
    
    // Combined transform: scale by 2 and translate by (5, 10, 15)
    std::string transform = "2,0,0,5,"   // row 0
                            "0,2,0,10,"  // row 1
                            "0,0,2,15,"  // row 2
                            "0,0,0,1";   // row 3
    obj1("transform").value(transform);
    
    cvc::bounding_box result = state->computeGraphicsBounds();
    // Cube should be scaled to [0,0,0]->[2,2,2] then translated to [5,10,15]->[7,12,17]
    EXPECT_NEAR(result.minx, 5.0, 1e-6);
    EXPECT_NEAR(result.miny, 10.0, 1e-6);
    EXPECT_NEAR(result.minz, 15.0, 1e-6);
    EXPECT_NEAR(result.maxx, 7.0, 1e-6);
    EXPECT_NEAR(result.maxy, 12.0, 1e-6);
    EXPECT_NEAR(result.maxz, 17.0, 1e-6);
}

// Test computeGraphicsBounds with multiple transformed objects
TEST_F(AppStateTest, ComputeGraphicsBounds_MultipleTransformed) {
    cvc::state& rootState = state->getRootState();
    cvc::state& graphicsState = rootState("graphics");
    cvc::state& childrenState = graphicsState("children");
    
    // Object 1: unit cube at origin, translated to (0,0,0)->(1,1,1)
    cvc::state& obj1 = childrenState("test_trans_obj1");
    obj1("metadata")("bounding_box").value("0,0,0,1,1,1");
    obj1("transform").value("1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1");
    
    // Object 2: unit cube at origin, translated to (10,10,10)->(11,11,11)
    cvc::state& obj2 = childrenState("test_scale_obj2");
    obj2("metadata")("bounding_box").value("0,0,0,1,1,1");
    obj2("transform").value("1,0,0,10,0,1,0,10,0,0,1,10,0,0,0,1");
    
    cvc::bounding_box result = state->computeGraphicsBounds();
    // Union should be [0,0,0] to [11,11,11]
    EXPECT_NEAR(result.minx, 0.0, 1e-6);
    EXPECT_NEAR(result.miny, 0.0, 1e-6);
    EXPECT_NEAR(result.minz, 0.0, 1e-6);
    EXPECT_NEAR(result.maxx, 11.0, 1e-6);
    EXPECT_NEAR(result.maxy, 11.0, 1e-6);
    EXPECT_NEAR(result.maxz, 11.0, 1e-6);
}








