#include <QApplication>
#include <cvc/core/state.h>
#include <cvc/core/state_object.h>
#include <cvc/geometry/geometry.h>
#include <cvc/volume/volume.h>
#include <gtest/gtest.h>
#include <volrover3/AppState.h>
#include <volrover3/CameraController.h>

// Need QApplication for Qt types
class AppStateTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!QApplication::instance()) {
      int argc = 0;
      char **argv = nullptr;
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

  static QApplication *app;
  static int testCounter;
  std::string m_statePrefix;
  std::unique_ptr<AppState> state;
};

QApplication *AppStateTest::app = nullptr;
int AppStateTest::testCounter = 0;

TEST_F(AppStateTest, SingletonInstance) {
  // Verify that the singleton instance is different from our test instance
  // (they use different state prefixes)
  AppState &singleton = AppState::instance();
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

// NOTE: Transfer function storage moved to per-volume state in VolumeNode
// These tests are commented out as AppState no longer has global transfer function storage
/*
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
*/

// ===========================
// State Tree Tests
// ===========================

TEST_F(AppStateTest, StateTreeCameraPosition) {
  // Set camera position via AppState
  state->setCameraPosition(10.0, 20.0, 30.0);

  // Verify values are stored in state tree
  auto &stateTree = state->getRootState();
  EXPECT_DOUBLE_EQ(stateTree("camera.position.x").value<double>(), 10.0);
  EXPECT_DOUBLE_EQ(stateTree("camera.position.y").value<double>(), 20.0);
  EXPECT_DOUBLE_EQ(stateTree("camera.position.z").value<double>(), 30.0);

  // Verify getters match state tree values
  double x, y, z;
  state->getCameraPosition(x, y, z);
  EXPECT_DOUBLE_EQ(x, 10.0);
  EXPECT_DOUBLE_EQ(y, 20.0);
  EXPECT_DOUBLE_EQ(z, 30.0);
}

TEST_F(AppStateTest, StateTreeCameraViewDirection) {
  state->setCameraViewDirection(1.0, 0.0, 0.0);

  auto &stateTree = state->getRootState();
  EXPECT_DOUBLE_EQ(stateTree("camera.view_direction.x").value<double>(), 1.0);
  EXPECT_DOUBLE_EQ(stateTree("camera.view_direction.y").value<double>(), 0.0);
  EXPECT_DOUBLE_EQ(stateTree("camera.view_direction.z").value<double>(), 0.0);

  double x, y, z;
  state->getCameraViewDirection(x, y, z);
  EXPECT_DOUBLE_EQ(x, 1.0);
  EXPECT_DOUBLE_EQ(y, 0.0);
  EXPECT_DOUBLE_EQ(z, 0.0);
}

TEST_F(AppStateTest, StateTreeCameraUpVector) {
  state->setCameraUpVector(0.0, 1.0, 0.0);

  auto &stateTree = state->getRootState();
  EXPECT_DOUBLE_EQ(stateTree("camera.up_vector.x").value<double>(), 0.0);
  EXPECT_DOUBLE_EQ(stateTree("camera.up_vector.y").value<double>(), 1.0);
  EXPECT_DOUBLE_EQ(stateTree("camera.up_vector.z").value<double>(), 0.0);

  double x, y, z;
  state->getCameraUpVector(x, y, z);
  EXPECT_DOUBLE_EQ(x, 0.0);
  EXPECT_DOUBLE_EQ(y, 1.0);
  EXPECT_DOUBLE_EQ(z, 0.0);
}

TEST_F(AppStateTest, StateTreeCameraFOV) {
  state->setCameraFieldOfView(60.0);

  auto &stateTree = state->getRootState();
  EXPECT_DOUBLE_EQ(stateTree("camera.fov").value<double>(), 60.0);
  EXPECT_DOUBLE_EQ(state->cameraFieldOfView(), 60.0);
}

TEST_F(AppStateTest, StateTreeCameraSpeed) {
  state->setCameraSpeed(3.5);

  auto &stateTree = state->getRootState();
  EXPECT_DOUBLE_EQ(stateTree("camera.speed").value<double>(), 3.5);
  EXPECT_DOUBLE_EQ(state->cameraSpeed(), 3.5);
}

TEST_F(AppStateTest, StateTreeCameraSensitivity) {
  state->setCameraSensitivity(0.75);

  auto &stateTree = state->getRootState();
  EXPECT_DOUBLE_EQ(stateTree("camera.sensitivity").value<double>(), 0.75);
  EXPECT_DOUBLE_EQ(state->cameraSensitivity(), 0.75);
}

TEST_F(AppStateTest, StateTreeWorldBounds) {
  cvc::bounding_box bounds(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
  state->setWorldBounds(bounds);

  auto &stateTree = state->getRootState();
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

// Grid and axis visibility now managed by GraphicsNode state tree
// See GridNodeTest.cpp for grid visibility tests

TEST_F(AppStateTest, StateTreeKeyBindings) {
  state->setCameraKeyForward(Qt::Key_W);
  state->setCameraKeyBackward(Qt::Key_S);
  state->setCameraKeyLeft(Qt::Key_A);
  state->setCameraKeyRight(Qt::Key_D);
  state->setCameraKeyUp(Qt::Key_E);
  state->setCameraKeyDown(Qt::Key_Q);

  auto &stateTree = state->getRootState();
  EXPECT_EQ(stateTree("camera.key_forward").value<int>(), Qt::Key_W);
  EXPECT_EQ(stateTree("camera.key_backward").value<int>(), Qt::Key_S);
  EXPECT_EQ(stateTree("camera.key_left").value<int>(), Qt::Key_A);
  EXPECT_EQ(stateTree("camera.key_right").value<int>(), Qt::Key_D);
  EXPECT_EQ(stateTree("camera.key_up").value<int>(), Qt::Key_E);
  EXPECT_EQ(stateTree("camera.key_down").value<int>(), Qt::Key_Q);
}

TEST_F(AppStateTest, StateTreeDirectUpdate) {
  // Set values directly in state tree (simulating external update)
  auto &stateTree = state->getRootState();
  stateTree("camera.position.x").value(100.0);
  stateTree("camera.position.y").value(200.0);
  stateTree("camera.position.z").value(300.0);

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

  auto connection = state->onCameraChanged([&callback_count]() { callback_count++; });

  // Trigger camera changes
  state->setCameraPosition(1.0, 2.0, 3.0);
  EXPECT_GT(callback_count, 0);

  int prev_count = callback_count;
  state->setCameraViewDirection(0.0, 0.0, 1.0);
  EXPECT_GT(callback_count, prev_count);

  prev_count = callback_count;
  state->setCameraUpVector(0.0, 1.0, 0.0);
  EXPECT_GT(callback_count, prev_count);

  prev_count = callback_count;
  state->setCameraFieldOfView(45.0);
  EXPECT_GT(callback_count, prev_count);

  // Disconnect and verify no more callbacks
  connection.disconnect();
  prev_count = callback_count;
  state->setCameraPosition(99.0, 99.0, 99.0);
  EXPECT_EQ(callback_count, prev_count); // Should not have incremented
}

TEST_F(AppStateTest, WorldBoundsChangedCallback) {
  int callback_count = 0;

  auto connection = state->onWorldBoundsChanged([&callback_count]() { callback_count++; });

  cvc::bounding_box bounds(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
  state->setWorldBounds(bounds);

  EXPECT_GT(callback_count, 0);

  connection.disconnect();
}

// Grid and axis visibility callback tests removed - GraphicsNode manages its own state
// See GridNodeTest.cpp for grid-related tests

TEST_F(AppStateTest, MultipleCallbacksForSameState) {
  int callback1_count = 0;
  int callback2_count = 0;
  int callback3_count = 0;

  auto conn1 = state->onCameraChanged([&callback1_count]() { callback1_count++; });
  auto conn2 = state->onCameraChanged([&callback2_count]() { callback2_count++; });
  auto conn3 = state->onCameraChanged([&callback3_count]() { callback3_count++; });

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

  auto connection = state->onCameraChanged([&callback_count]() { callback_count++; });

  auto &stateTree = state->getRootState();
  int before_count = callback_count;

  // Trigger callback by setting camera position in state tree directly
  stateTree("camera.position.x").value(123.0);

  EXPECT_GT(callback_count, before_count);

  connection.disconnect();
}

TEST_F(AppStateTest, CallbackDisconnection) {
  int callback_count = 0;

  auto connection = state->onCameraModeChanged([&callback_count]() { callback_count++; });

  // Trigger callback (default is ORBIT_MODE, so change to FLY_MODE)
  state->setCameraMode(FLY_MODE);
  EXPECT_EQ(callback_count, 1);

  // Disconnect
  connection.disconnect();

  // Trigger again - should not fire
  state->setCameraMode(ORBIT_MODE);
  EXPECT_EQ(callback_count, 1); // Should still be 1
}

// ===========================
// State Persistence Tests
// ===========================

TEST_F(AppStateTest, StateTreeInitialized) {
  auto &stateTree = state->getRootState();

  // Verify default state values are initialized
  EXPECT_TRUE(stateTree("camera.position.x").initialized());
  EXPECT_TRUE(stateTree("camera.position.y").initialized());
  EXPECT_TRUE(stateTree("camera.position.z").initialized());
  EXPECT_TRUE(stateTree("camera.speed").initialized());
  EXPECT_TRUE(stateTree("camera.sensitivity").initialized());
  EXPECT_TRUE(stateTree("camera.fov").initialized());
}

TEST_F(AppStateTest, StateTreePersistence) {
  // Set values
  state->setCameraPosition(11.0, 22.0, 33.0);
  state->setCameraSpeed(5.5);

  // Verify values persist in state tree
  auto &stateTree = state->getRootState();
  EXPECT_DOUBLE_EQ(stateTree("camera.position.x").value<double>(), 11.0);
  EXPECT_DOUBLE_EQ(stateTree("camera.position.y").value<double>(), 22.0);
  EXPECT_DOUBLE_EQ(stateTree("camera.position.z").value<double>(), 33.0);
  EXPECT_DOUBLE_EQ(stateTree("camera.speed").value<double>(), 5.5);

  // Values should persist across multiple reads
  EXPECT_DOUBLE_EQ(stateTree("camera.position.x").value<double>(), 11.0);
  EXPECT_DOUBLE_EQ(stateTree("camera.speed").value<double>(), 5.5);
}

// ===========================
// Note: Grid-specific tests removed
// ===========================
// Grid visibility and properties are now managed by GridNode at
// volrover3.graphics.root.children.grid See GridNodeTest.cpp for grid-related tests
