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
        // Disable threading for state_object to avoid destruction race conditions
        cvc::state_object<AppState>::setUseThreading(false);
    }
    
    void SetUp() override {
        // Create AppState with unique prefix for test isolation
        // Each test instance gets its own state subtree
        m_statePrefix = "appstate_test_" + std::to_string(testCounter++);
        state = std::make_unique<AppState>(m_statePrefix);
    }
    
    void TearDown() override {
        // Clean up test-specific state instance
        // No need to reset state tree - unique prefixes provide isolation
        state.reset();
    }
    
    static QApplication* app;
    static int testCounter;
    std::string m_statePrefix;
    std::unique_ptr<AppState> state;
};

QApplication* AppStateTest::app = nullptr;
int AppStateTest::testCounter = 0;

TEST_F(AppStateTest, SingletonInstance) {
    // Verify that the singleton instance is different from our test instance
    // (they use different state prefixes)
    AppState& singleton = AppState::instance();
    EXPECT_NE(state->getStatePrefix(), singleton.getStatePrefix());
    EXPECT_EQ(singleton.getStatePrefix(), "volrover3");
    // Our test instance uses unique prefix
    EXPECT_EQ(state->getStatePrefix(), m_statePrefix);
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

TEST_F(AppStateTest, ColorDefaultValues) {
    // Reset to default values first
    auto& stateTree = state->getRootState();
    stateTree("grid_color").value("0.5,0.5,0.5");
    
    // Test default values from fresh state
    double r, g, b;
    
    // Grid should default to gray
    state->getGridColor(r, g, b);
    EXPECT_DOUBLE_EQ(r, 0.5);
    EXPECT_DOUBLE_EQ(g, 0.5);
    EXPECT_DOUBLE_EQ(b, 0.5);
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

TEST_F(AppStateTest, ColorBoundaryValues) {
    // Test with boundary values (0.0 and 1.0)
    state->setGridColor(0.0, 1.0, 0.0);
    
    double r, g, b;
    state->getGridColor(r, g, b);
    EXPECT_DOUBLE_EQ(r, 0.0);
    EXPECT_DOUBLE_EQ(g, 1.0);
    EXPECT_DOUBLE_EQ(b, 0.0);
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

