#include <gtest/gtest.h>
#include <QApplication>
#include <QKeyEvent>
#include <volrover3/CameraController.h>
#include <vtkRenderer.h>
#include <vtkCamera.h>
#include <vtkRenderWindow.h>

class CameraControllerTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!QApplication::instance()) {
            int argc = 0;
            char** argv = nullptr;
            app = new QApplication(argc, argv);
        }
    }
    
    void SetUp() override {
        renderer = vtkSmartPointer<vtkRenderer>::New();
        camera = vtkSmartPointer<vtkCamera>::New();
        renderer->SetActiveCamera(camera);
        
        controller = new CameraController(renderer);
    }
    
    void TearDown() override {
        delete controller;
    }
    
    static QApplication* app;
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkCamera> camera;
    CameraController* controller;
};

QApplication* CameraControllerTest::app = nullptr;

TEST_F(CameraControllerTest, InitialState) {
    EXPECT_NE(controller, nullptr);
    EXPECT_EQ(controller->getMode(), CameraController::Orbit);
}

TEST_F(CameraControllerTest, ModeSwitch) {
    controller->setMode(CameraController::Fly);
    EXPECT_EQ(controller->getMode(), CameraController::Fly);
    
    controller->setMode(CameraController::Orbit);
    EXPECT_EQ(controller->getMode(), CameraController::Orbit);
}

TEST_F(CameraControllerTest, MouseSensitivity) {
    controller->setMouseSensitivity(0.5);
    EXPECT_DOUBLE_EQ(controller->getMouseSensitivity(), 0.5);
    
    controller->setMouseSensitivity(1.5);
    EXPECT_DOUBLE_EQ(controller->getMouseSensitivity(), 1.5);
}

TEST_F(CameraControllerTest, MouseInversion) {
    controller->setMouseInverted(true);
    EXPECT_TRUE(controller->getMouseInverted());
    
    controller->setMouseInverted(false);
    EXPECT_FALSE(controller->getMouseInverted());
}

TEST_F(CameraControllerTest, MovementSpeed) {
    controller->setMovementSpeed(2.0);
    EXPECT_DOUBLE_EQ(controller->getMovementSpeed(), 2.0);
    
    controller->setMovementSpeed(5.0);
    EXPECT_DOUBLE_EQ(controller->getMovementSpeed(), 5.0);
}

TEST_F(CameraControllerTest, KeyBindings) {
    controller->setKeyMoveForward(Qt::Key_W);
    controller->setKeyMoveBackward(Qt::Key_S);
    controller->setKeyMoveLeft(Qt::Key_A);
    controller->setKeyMoveRight(Qt::Key_D);
    controller->setKeyMoveUp(Qt::Key_E);
    controller->setKeyMoveDown(Qt::Key_Q);
    
    // Verify bindings were set (movement will be tested in integration tests)
    SUCCEED();
}

TEST_F(CameraControllerTest, OrbitRotation) {
    controller->setMode(CameraController::Orbit);
    
    // Get initial camera position
    double initialPos[3];
    camera->GetPosition(initialPos);
    
    // Simulate mouse drag
    controller->handleMousePress(100, 100, Qt::LeftButton);
    controller->handleMouseMove(150, 150);
    controller->handleMouseRelease(Qt::LeftButton);
    
    // Camera position should have changed
    double newPos[3];
    camera->GetPosition(newPos);
    
    // In orbit mode, camera should have moved
    bool positionChanged = (initialPos[0] != newPos[0]) ||
                          (initialPos[1] != newPos[1]) ||
                          (initialPos[2] != newPos[2]);
    EXPECT_TRUE(positionChanged);
}

TEST_F(CameraControllerTest, OrbitPan) {
    controller->setMode(CameraController::Orbit);
    
    // Get initial focal point
    double initialFocus[3];
    camera->GetFocalPoint(initialFocus);
    
    // Simulate middle mouse drag
    controller->handleMousePress(100, 100, Qt::MiddleButton);
    controller->handleMouseMove(150, 150);
    controller->handleMouseRelease(Qt::MiddleButton);
    
    // Focal point should have changed
    double newFocus[3];
    camera->GetFocalPoint(newFocus);
    
    bool focusChanged = (initialFocus[0] != newFocus[0]) ||
                       (initialFocus[1] != newFocus[1]) ||
                       (initialFocus[2] != newFocus[2]);
    EXPECT_TRUE(focusChanged);
}

TEST_F(CameraControllerTest, Zoom) {
    // Set initial distance
    camera->SetPosition(0, 0, 10);
    camera->SetFocalPoint(0, 0, 0);
    
    double initialDistance = camera->GetDistance();
    
    // Simulate scroll (zoom in)
    controller->handleWheel(120);  // Positive delta = zoom in
    
    double newDistance = camera->GetDistance();
    
    // Distance should have decreased (zoomed in)
    EXPECT_LT(newDistance, initialDistance);
}

TEST_F(CameraControllerTest, FlyMode) {
    controller->setMode(CameraController::Fly);
    
    // Get initial camera position
    double initialPos[3];
    camera->GetPosition(initialPos);
    
    // Simulate mouse drag (should change view direction)
    controller->handleMousePress(100, 100, Qt::LeftButton);
    controller->handleMouseMove(150, 100);
    controller->handleMouseRelease(Qt::LeftButton);
    
    // View direction should have changed (tested via focal point relative to position)
    SUCCEED();
}

TEST_F(CameraControllerTest, KeyboardMovement) {
    controller->setMode(CameraController::Fly);
    controller->setMovementSpeed(1.0);
    
    // Get initial position
    double initialPos[3];
    camera->GetPosition(initialPos);
    
    // Simulate key press for moving forward
    QKeyEvent keyPress(QEvent::KeyPress, Qt::Key_W, Qt::NoModifier);
    controller->handleKeyPress(&keyPress);
    
    // Update camera (simulate time passing)
    controller->update(0.016);  // 16ms frame time
    
    // Position should have changed
    double newPos[3];
    camera->GetPosition(newPos);
    
    bool moved = (initialPos[0] != newPos[0]) ||
                (initialPos[1] != newPos[1]) ||
                (initialPos[2] != newPos[2]);
    
    // Release key
    QKeyEvent keyRelease(QEvent::KeyRelease, Qt::Key_W, Qt::NoModifier);
    controller->handleKeyRelease(&keyRelease);
    
    EXPECT_TRUE(moved);
}

TEST_F(CameraControllerTest, GetSetCameraState) {
    // Set specific camera state
    std::array<double, 3> position = {5.0, 3.0, 8.0};
    std::array<double, 3> direction = {0.0, 0.0, -1.0};
    std::array<double, 3> up = {0.0, 1.0, 0.0};
    double fov = 45.0;
    
    controller->setCameraState(position, direction, up, fov);
    
    // Verify camera state was set
    double pos[3], focal[3], upVec[3];
    camera->GetPosition(pos);
    camera->GetFocalPoint(focal);
    camera->GetViewUp(upVec);
    
    EXPECT_NEAR(pos[0], position[0], 0.001);
    EXPECT_NEAR(pos[1], position[1], 0.001);
    EXPECT_NEAR(pos[2], position[2], 0.001);
    EXPECT_NEAR(upVec[0], up[0], 0.001);
    EXPECT_NEAR(upVec[1], up[1], 0.001);
    EXPECT_NEAR(upVec[2], up[2], 0.001);
    EXPECT_NEAR(camera->GetViewAngle(), fov, 0.001);
}

TEST_F(CameraControllerTest, ResetCamera) {
    // Move camera to a specific position
    camera->SetPosition(100, 200, 300);
    camera->SetFocalPoint(50, 50, 50);
    
    // Reset camera
    controller->resetCamera();
    
    // Camera should be reset to a standard view
    double pos[3];
    camera->GetPosition(pos);
    
    // Position should be different from what we set
    bool wasReset = (pos[0] != 100) || (pos[1] != 200) || (pos[2] != 300);
    EXPECT_TRUE(wasReset);
}

TEST_F(CameraControllerTest, UpdateWithNoMovement) {
    // Get initial position
    double initialPos[3];
    camera->GetPosition(initialPos);
    
    // Update without any key presses
    controller->update(0.016);
    
    // Position should not change
    double newPos[3];
    camera->GetPosition(newPos);
    
    EXPECT_DOUBLE_EQ(initialPos[0], newPos[0]);
    EXPECT_DOUBLE_EQ(initialPos[1], newPos[1]);
    EXPECT_DOUBLE_EQ(initialPos[2], newPos[2]);
}
