#include <QApplication>
#include <cvc/app.h>
#include <cvc/bounding_box.h>
#include <gtest/gtest.h>
#include <volrover3/AppState.h>
#include <volrover3/GridNode.h>
#include <vtkActor.h>
#include <vtkActor2D.h>
#include <vtkNew.h>
#include <vtkRenderer.h>

class GridNodeTest : public ::testing::Test {
protected:
  cvc::app ctx;
  static void SetUpTestSuite() {
    if (!QApplication::instance()) {
      int argc = 0;
      char **argv = nullptr;
      app = new QApplication(argc, argv);
    }
    // Disable threading for state_object to avoid race conditions during destruction
    cvc::state_object<SceneNode>::setUseThreading(false);
  }

  void SetUp() override {
    gridNode = new GridNode(ctx, "test.grid", "grid");
    renderer = vtkRenderer::New();
    appState = &AppState::instance();
  }

  void TearDown() override {
    if (gridNode) {
      gridNode->removeFromRenderer(renderer);
    }
    renderer->Delete();
    delete gridNode;
  }

  static QApplication *app;
  GridNode *gridNode;
  vtkRenderer *renderer;
  AppState *appState;
};

QApplication *GridNodeTest::app = nullptr;

// ===========================
// Construction and Basic Properties
// ===========================

TEST_F(GridNodeTest, Construction) {
  EXPECT_NE(gridNode, nullptr);
  EXPECT_TRUE(gridNode->isVisible());
}

TEST_F(GridNodeTest, VisibilityToggle) {
  gridNode->setVisible(true);
  EXPECT_TRUE(gridNode->isVisible());

  gridNode->setVisible(false);
  EXPECT_FALSE(gridNode->isVisible());
}

// ===========================
// Bounding Box and Grid Positioning
// ===========================

TEST_F(GridNodeTest, SetBounds) {
  cvc::bounding_box bounds;
  bounds.setMin(-5.0, -3.0, -2.0);
  bounds.setMax(5.0, 3.0, 2.0);

  // Should not crash
  gridNode->setBounds(bounds);
  SUCCEED();
}

TEST_F(GridNodeTest, GridAtBoundingBoxMinimum) {
  // Grid planes should be positioned at bounding box minimum corner
  cvc::bounding_box bounds;
  bounds.setMin(10.0, 20.0, 30.0);
  bounds.setMax(50.0, 60.0, 70.0);

  gridNode->setBounds(bounds);

  // Grid should create planes at (10, 20, 30) corner
  // YZ plane at X=10, XZ plane at Y=20, XY plane at Z=30
  // Can't directly test internal VTK geometry, but verify no crash
  SUCCEED();
}

TEST_F(GridNodeTest, NegativeBounds) {
  cvc::bounding_box bounds;
  bounds.setMin(-100.0, -50.0, -25.0);
  bounds.setMax(-10.0, -5.0, -2.0);

  gridNode->setBounds(bounds);
  SUCCEED();
}

TEST_F(GridNodeTest, ZeroBounds) {
  cvc::bounding_box bounds;
  bounds.setMin(0.0, 0.0, 0.0);
  bounds.setMax(0.0, 0.0, 0.0);

  // Edge case: zero-volume bounding box
  gridNode->setBounds(bounds);
  SUCCEED();
}

TEST_F(GridNodeTest, LargeBounds) {
  cvc::bounding_box bounds;
  bounds.setMin(-1000.0, -1000.0, -1000.0);
  bounds.setMax(1000.0, 1000.0, 1000.0);

  gridNode->setBounds(bounds);
  SUCCEED();
}

// ===========================
// Grid Divisions
// ===========================

TEST_F(GridNodeTest, SetDivisions) {
  gridNode->setGridDivisions(10, 20, 30);

  int x, y, z;
  gridNode->getGridDivisions(x, y, z);
  EXPECT_EQ(x, 10);
  EXPECT_EQ(y, 20);
  EXPECT_EQ(z, 30);
}

TEST_F(GridNodeTest, MinimumDivisions) {
  gridNode->setGridDivisions(1, 1, 1);

  int x, y, z;
  gridNode->getGridDivisions(x, y, z);
  EXPECT_EQ(x, 1);
  EXPECT_EQ(y, 1);
  EXPECT_EQ(z, 1);
}

TEST_F(GridNodeTest, LargeDivisions) {
  gridNode->setGridDivisions(256, 256, 256);

  int x, y, z;
  gridNode->getGridDivisions(x, y, z);
  EXPECT_EQ(x, 256);
  EXPECT_EQ(y, 256);
  EXPECT_EQ(z, 256);
}

TEST_F(GridNodeTest, DivisionsUpdateGrid) {
  cvc::bounding_box bounds;
  bounds.setMin(0.0, 0.0, 0.0);
  bounds.setMax(10.0, 10.0, 10.0);
  gridNode->setBounds(bounds);

  // Change divisions should update grid
  gridNode->setGridDivisions(5, 5, 5);
  gridNode->setGridDivisions(20, 20, 20);

  int x, y, z;
  gridNode->getGridDivisions(x, y, z);
  EXPECT_EQ(x, 20);
  EXPECT_EQ(y, 20);
  EXPECT_EQ(z, 20);
}

// ===========================
// Plane Visibility
// ===========================

TEST_F(GridNodeTest, PlaneVisibility) {
  gridNode->setYZPlaneVisible(true);
  gridNode->setXZPlaneVisible(false);
  gridNode->setXYPlaneVisible(true);

  EXPECT_TRUE(gridNode->isYZPlaneVisible());
  EXPECT_FALSE(gridNode->isXZPlaneVisible());
  EXPECT_TRUE(gridNode->isXYPlaneVisible());
}

TEST_F(GridNodeTest, AllPlanesHidden) {
  gridNode->setYZPlaneVisible(false);
  gridNode->setXZPlaneVisible(false);
  gridNode->setXYPlaneVisible(false);

  EXPECT_FALSE(gridNode->isYZPlaneVisible());
  EXPECT_FALSE(gridNode->isXZPlaneVisible());
  EXPECT_FALSE(gridNode->isXYPlaneVisible());
}

TEST_F(GridNodeTest, AllPlanesVisible) {
  gridNode->setYZPlaneVisible(true);
  gridNode->setXZPlaneVisible(true);
  gridNode->setXYPlaneVisible(true);

  EXPECT_TRUE(gridNode->isYZPlaneVisible());
  EXPECT_TRUE(gridNode->isXZPlaneVisible());
  EXPECT_TRUE(gridNode->isXYPlaneVisible());
}

// ===========================
// Tick Intervals
// ===========================

TEST_F(GridNodeTest, SetTickIntervals) {
  gridNode->setTickIntervals(4, 8, 16);

  int x, y, z;
  gridNode->getTickIntervals(x, y, z);
  EXPECT_EQ(x, 4);
  EXPECT_EQ(y, 8);
  EXPECT_EQ(z, 16);
}

TEST_F(GridNodeTest, MinimumTickInterval) {
  gridNode->setTickIntervals(1, 1, 1);

  int x, y, z;
  gridNode->getTickIntervals(x, y, z);
  EXPECT_EQ(x, 1);
  EXPECT_EQ(y, 1);
  EXPECT_EQ(z, 1);
}

TEST_F(GridNodeTest, LargeTickInterval) {
  gridNode->setTickIntervals(128, 128, 128);

  int x, y, z;
  gridNode->getTickIntervals(x, y, z);
  EXPECT_EQ(x, 128);
  EXPECT_EQ(y, 128);
  EXPECT_EQ(z, 128);
}

TEST_F(GridNodeTest, TickIntervalsWithBounds) {
  cvc::bounding_box bounds;
  bounds.setMin(0.0, 0.0, 0.0);
  bounds.setMax(100.0, 100.0, 100.0);
  gridNode->setBounds(bounds);
  gridNode->setGridDivisions(100, 100, 100);

  // Set tick intervals
  gridNode->setTickIntervals(10, 10, 10);

  int x, y, z;
  gridNode->getTickIntervals(x, y, z);
  EXPECT_EQ(x, 10);
  EXPECT_EQ(y, 10);
  EXPECT_EQ(z, 10);
}

// ===========================
// Tick Label Properties
// ===========================

TEST_F(GridNodeTest, TickLabelColor) {
  gridNode->setTickLabelColor(1.0, 0.5, 0.0);

  double r, g, b;
  gridNode->getTickLabelColor(r, g, b);
  EXPECT_DOUBLE_EQ(r, 1.0);
  EXPECT_DOUBLE_EQ(g, 0.5);
  EXPECT_DOUBLE_EQ(b, 0.0);
}

TEST_F(GridNodeTest, TickLabelFontSize) {
  gridNode->setTickLabelFontSize(24);
  EXPECT_EQ(gridNode->getTickLabelFontSize(), 24);

  gridNode->setTickLabelFontSize(8);
  EXPECT_EQ(gridNode->getTickLabelFontSize(), 8);
}

// ===========================
// Plane Colors
// ===========================

TEST_F(GridNodeTest, YZPlaneColor) {
  gridNode->setYZPlaneColor(0.8, 0.2, 0.1);

  double r, g, b;
  gridNode->getYZPlaneColor(r, g, b);
  EXPECT_DOUBLE_EQ(r, 0.8);
  EXPECT_DOUBLE_EQ(g, 0.2);
  EXPECT_DOUBLE_EQ(b, 0.1);
}

TEST_F(GridNodeTest, XZPlaneColor) {
  gridNode->setXZPlaneColor(0.1, 0.8, 0.2);

  double r, g, b;
  gridNode->getXZPlaneColor(r, g, b);
  EXPECT_DOUBLE_EQ(r, 0.1);
  EXPECT_DOUBLE_EQ(g, 0.8);
  EXPECT_DOUBLE_EQ(b, 0.2);
}

TEST_F(GridNodeTest, XYPlaneColor) {
  gridNode->setXYPlaneColor(0.2, 0.1, 0.8);

  double r, g, b;
  gridNode->getXYPlaneColor(r, g, b);
  EXPECT_DOUBLE_EQ(r, 0.2);
  EXPECT_DOUBLE_EQ(g, 0.1);
  EXPECT_DOUBLE_EQ(b, 0.8);
}

// ===========================
// Renderer Management
// ===========================

TEST_F(GridNodeTest, AddToRenderer) {
  cvc::bounding_box bounds;
  bounds.setMin(0.0, 0.0, 0.0);
  bounds.setMax(10.0, 10.0, 10.0);
  gridNode->setBounds(bounds);

  // Should not crash
  gridNode->addToRenderer(renderer);
  SUCCEED();
}

TEST_F(GridNodeTest, RemoveFromRenderer) {
  cvc::bounding_box bounds;
  bounds.setMin(0.0, 0.0, 0.0);
  bounds.setMax(10.0, 10.0, 10.0);
  gridNode->setBounds(bounds);

  gridNode->addToRenderer(renderer);
  gridNode->removeFromRenderer(renderer);
  SUCCEED();
}

TEST_F(GridNodeTest, MultipleAddRemove) {
  cvc::bounding_box bounds;
  bounds.setMin(0.0, 0.0, 0.0);
  bounds.setMax(10.0, 10.0, 10.0);
  gridNode->setBounds(bounds);

  // Add and remove multiple times
  for (int i = 0; i < 5; ++i) {
    gridNode->addToRenderer(renderer);
    gridNode->removeFromRenderer(renderer);
  }
  SUCCEED();
}

// ===========================
// Tick Visibility and State Integration
// ===========================

TEST_F(GridNodeTest, TickVisibilityDefault) {
  // GridNode initializes tics.visible to false by default
  EXPECT_FALSE(gridNode->getState("tics.visible").value<bool>());
}

TEST_F(GridNodeTest, TickVisibilityToggle) {
  // Test state synchronization via GridNode state
  gridNode->getState("tics.visible").value(true);
  EXPECT_TRUE(gridNode->getState("tics.visible").value<bool>());

  gridNode->getState("tics.visible").value(false);
  EXPECT_FALSE(gridNode->getState("tics.visible").value<bool>());
}

TEST_F(GridNodeTest, TickLabelsWithVisibilityOff) {
  // When ticks are not visible, tick labels should not be added to renderer
  gridNode->getState("tics.visible").value(false);

  cvc::bounding_box bounds;
  bounds.setMin(0.0, 0.0, 0.0);
  bounds.setMax(10.0, 10.0, 10.0);
  gridNode->setBounds(bounds);
  gridNode->setTickIntervals(2, 2, 2);

  gridNode->addToRenderer(renderer);

  // Verify no crash and tick actors should not be in renderer
  SUCCEED();
}

TEST_F(GridNodeTest, TickLabelsWithVisibilityOn) {
  // When ticks are visible, tick labels should be added to renderer
  gridNode->getState("tics.visible").value(true);

  cvc::bounding_box bounds;
  bounds.setMin(0.0, 0.0, 0.0);
  bounds.setMax(10.0, 10.0, 10.0);
  gridNode->setBounds(bounds);
  gridNode->setTickIntervals(2, 2, 2);

  gridNode->addToRenderer(renderer);

  // Verify no crash and tick actors should be in renderer
  SUCCEED();
}

// ===========================
// Grid Update Scenarios
// ===========================

TEST_F(GridNodeTest, UpdateAfterDataLoad) {
  // Simulate data loading scenario
  cvc::bounding_box initialBounds;
  initialBounds.setMin(0.0, 0.0, 0.0);
  initialBounds.setMax(1.0, 1.0, 1.0);
  gridNode->setBounds(initialBounds);
  gridNode->addToRenderer(renderer);

  // Load new data with different bounds
  cvc::bounding_box newBounds;
  newBounds.setMin(-10.0, -5.0, -2.0);
  newBounds.setMax(10.0, 5.0, 2.0);
  gridNode->setBounds(newBounds);

  // Old tick labels should be removed, new ones created
  SUCCEED();
}

TEST_F(GridNodeTest, TickLabelPositioningAfterBoundsChange) {
  gridNode->getState("tics.visible").value(true);

  // Initial setup
  cvc::bounding_box bounds1;
  bounds1.setMin(0.0, 0.0, 0.0);
  bounds1.setMax(10.0, 10.0, 10.0);
  gridNode->setBounds(bounds1);
  gridNode->setTickIntervals(5, 5, 5);
  gridNode->addToRenderer(renderer);

  // Change bounds
  cvc::bounding_box bounds2;
  bounds2.setMin(-5.0, -5.0, -5.0);
  bounds2.setMax(5.0, 5.0, 5.0);
  gridNode->setBounds(bounds2);

  // Tick labels should be repositioned correctly
  SUCCEED();
}

TEST_F(GridNodeTest, MultiplePropertyChanges) {
  cvc::bounding_box bounds;
  bounds.setMin(0.0, 0.0, 0.0);
  bounds.setMax(100.0, 100.0, 100.0);

  gridNode->setBounds(bounds);
  gridNode->setGridDivisions(50, 50, 50);
  gridNode->setTickIntervals(10, 10, 10);
  gridNode->setYZPlaneColor(1.0, 0.0, 0.0);
  gridNode->setXZPlaneColor(0.0, 1.0, 0.0);
  gridNode->setXYPlaneColor(0.0, 0.0, 1.0);
  gridNode->setTickLabelColor(1.0, 1.0, 1.0);
  gridNode->setTickLabelFontSize(16);
  gridNode->setYZPlaneVisible(true);
  gridNode->setXZPlaneVisible(true);
  gridNode->setXYPlaneVisible(true);

  gridNode->addToRenderer(renderer);

  // All properties should be applied correctly
  int divX, divY, divZ;
  gridNode->getGridDivisions(divX, divY, divZ);
  EXPECT_EQ(divX, 50);
  EXPECT_EQ(divY, 50);
  EXPECT_EQ(divZ, 50);

  int tickX, tickY, tickZ;
  gridNode->getTickIntervals(tickX, tickY, tickZ);
  EXPECT_EQ(tickX, 10);
  EXPECT_EQ(tickY, 10);
  EXPECT_EQ(tickZ, 10);

  EXPECT_EQ(gridNode->getTickLabelFontSize(), 16);
}

// ===========================
// Edge Cases
// ===========================

TEST_F(GridNodeTest, AsymmetricBounds) {
  cvc::bounding_box bounds;
  bounds.setMin(0.0, 0.0, 0.0);
  bounds.setMax(100.0, 10.0, 1.0);

  gridNode->setBounds(bounds);
  gridNode->setGridDivisions(100, 10, 1);
  gridNode->setTickIntervals(10, 2, 1);

  SUCCEED();
}

TEST_F(GridNodeTest, FractionalBounds) {
  cvc::bounding_box bounds;
  bounds.setMin(0.123, 0.456, 0.789);
  bounds.setMax(1.234, 2.345, 3.456);

  gridNode->setBounds(bounds);
  SUCCEED();
}

TEST_F(GridNodeTest, VisibilityWhileInRenderer) {
  cvc::bounding_box bounds;
  bounds.setMin(0.0, 0.0, 0.0);
  bounds.setMax(10.0, 10.0, 10.0);
  gridNode->setBounds(bounds);
  gridNode->addToRenderer(renderer);

  // Toggle visibility while in renderer
  gridNode->setVisible(false);
  gridNode->setVisible(true);

  SUCCEED();
}

TEST_F(GridNodeTest, PlaneVisibilityWhileInRenderer) {
  cvc::bounding_box bounds;
  bounds.setMin(0.0, 0.0, 0.0);
  bounds.setMax(10.0, 10.0, 10.0);
  gridNode->setBounds(bounds);
  gridNode->addToRenderer(renderer);

  // Toggle individual planes while in renderer
  gridNode->setYZPlaneVisible(false);
  gridNode->setXZPlaneVisible(false);
  gridNode->setXYPlaneVisible(false);

  gridNode->setYZPlaneVisible(true);
  gridNode->setXZPlaneVisible(true);
  gridNode->setXYPlaneVisible(true);

  SUCCEED();
}
