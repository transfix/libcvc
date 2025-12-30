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
        // AppState is a singleton, get instance
        state = &AppState::instance();
    }
    
    static QApplication* app;
    AppState* state;
};

QApplication* AppStateTest::app = nullptr;

TEST_F(AppStateTest, SingletonInstance) {
    AppState* state2 = &AppState::instance();
    EXPECT_EQ(state, state2);
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
    auto& stateTree = cvc::state::instance()("volrover3");
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
    
    auto& stateTree = cvc::state::instance()("volrover3");
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
    
    auto& stateTree = cvc::state::instance()("volrover3");
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
    
    auto& stateTree = cvc::state::instance()("volrover3");
    EXPECT_DOUBLE_EQ(stateTree("camera_fov").value<double>(), 60.0);
    EXPECT_DOUBLE_EQ(state->cameraFieldOfView(), 60.0);
}

TEST_F(AppStateTest, StateTreeCameraSpeed) {
    state->setCameraSpeed(3.5);
    
    auto& stateTree = cvc::state::instance()("volrover3");
    EXPECT_DOUBLE_EQ(stateTree("camera_speed").value<double>(), 3.5);
    EXPECT_DOUBLE_EQ(state->cameraSpeed(), 3.5);
}

TEST_F(AppStateTest, StateTreeCameraSensitivity) {
    state->setCameraSensitivity(0.75);
    
    auto& stateTree = cvc::state::instance()("volrover3");
    EXPECT_DOUBLE_EQ(stateTree("camera_sensitivity").value<double>(), 0.75);
    EXPECT_DOUBLE_EQ(state->cameraSensitivity(), 0.75);
}

TEST_F(AppStateTest, StateTreeWorldBounds) {
    cvc::bounding_box bounds(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
    state->setWorldBounds(bounds);
    
    auto& stateTree = cvc::state::instance()("volrover3");
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
    
    auto& stateTree = cvc::state::instance()("volrover3");
    EXPECT_TRUE(stateTree("grid_visible").value<bool>());
    EXPECT_TRUE(state->gridVisible());
    
    state->setGridVisible(false);
    EXPECT_FALSE(stateTree("grid_visible").value<bool>());
    EXPECT_FALSE(state->gridVisible());
}

TEST_F(AppStateTest, StateTreeAxisVisible) {
    state->setAxisVisible(true);
    
    auto& stateTree = cvc::state::instance()("volrover3");
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
    
    auto& stateTree = cvc::state::instance()("volrover3");
    EXPECT_EQ(stateTree("camera_key_forward").value<int>(), Qt::Key_W);
    EXPECT_EQ(stateTree("camera_key_backward").value<int>(), Qt::Key_S);
    EXPECT_EQ(stateTree("camera_key_left").value<int>(), Qt::Key_A);
    EXPECT_EQ(stateTree("camera_key_right").value<int>(), Qt::Key_D);
    EXPECT_EQ(stateTree("camera_key_up").value<int>(), Qt::Key_E);
    EXPECT_EQ(stateTree("camera_key_down").value<int>(), Qt::Key_Q);
}

TEST_F(AppStateTest, StateTreeDirectUpdate) {
    // Set values directly in state tree (simulating external update)
    auto& stateTree = cvc::state::instance()("volrover3");
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
    auto& stateTree = cvc::state::instance()("volrover3");
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
    auto& stateTree = cvc::state::instance()("volrover3");
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
    auto& stateTree = cvc::state::instance()("volrover3");
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
    auto& stateTree = cvc::state::instance()("volrover3");
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
    auto& stateTree = cvc::state::instance()("volrover3");
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
    auto& stateTree = cvc::state::instance()("volrover3");
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
    auto& stateTree = cvc::state::instance()("volrover3");
    
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
    auto& stateTree = cvc::state::instance()("volrover3");
    EXPECT_DOUBLE_EQ(stateTree("camera_position_x").value<double>(), 11.0);
    EXPECT_DOUBLE_EQ(stateTree("camera_position_y").value<double>(), 22.0);
    EXPECT_DOUBLE_EQ(stateTree("camera_position_z").value<double>(), 33.0);
    EXPECT_DOUBLE_EQ(stateTree("camera_speed").value<double>(), 5.5);
    EXPECT_TRUE(stateTree("grid_visible").value<bool>());
    
    // Values should persist across multiple reads
    EXPECT_DOUBLE_EQ(stateTree("camera_position_x").value<double>(), 11.0);
    EXPECT_DOUBLE_EQ(stateTree("camera_speed").value<double>(), 5.5);
}

