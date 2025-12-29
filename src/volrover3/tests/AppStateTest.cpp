#include <gtest/gtest.h>
#include <QApplication>
#include <volrover3/AppState.h>
#include <cvc/geometry.h>
#include <cvc/volume.h>

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
    std::array<double, 3> pos = {1.0, 2.0, 3.0};
    state->setCameraPosition(pos);
    
    auto retrieved = state->cameraPosition();
    EXPECT_DOUBLE_EQ(retrieved[0], 1.0);
    EXPECT_DOUBLE_EQ(retrieved[1], 2.0);
    EXPECT_DOUBLE_EQ(retrieved[2], 3.0);
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
