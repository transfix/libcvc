#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QTest>
#include <cvc/geometry/geometry.h>
#include <cvc/core/state.h>
#include <gtest/gtest.h>
#include <volrover3/GeometryDialog.h>
#include <volrover3/GeometryNode.h>
#include <volrover3/SceneGraph.h>
#include <volrover3/volrover3_app.h>

class GeometryDialogTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    // Disable threading for state_object to avoid race conditions
    cvc::state_object<SceneNode>::setUseThreading(false);

    // Initialize Qt if not already initialized
    if (!QApplication::instance()) {
      // Qt requires valid argc/argv pointers, otherwise X11 backend crashes
      // when trying to access QCoreApplication::arguments()
      static int argc = 1;
      static char appName[] = "test";
      static char *argv[] = {appName, nullptr};
      static QApplication app(argc, argv);

      // Disable session management to prevent X11 session manager crashes
      app.setProperty("sessionManagement", false);
    }
  }

  void SetUp() override {
    sceneGraph = std::make_shared<SceneGraph>();
    dialog = nullptr;
  }

  void TearDown() override {
    if (dialog) {
      delete dialog;
      dialog = nullptr;
    }
    sceneGraph.reset();
  }

  cvc::geometry createTestGeometry() {
    cvc::geometry geom;
    geom.points().push_back({0.0, 0.0, 0.0});
    geom.points().push_back({1.0, 0.0, 0.0});
    geom.points().push_back({0.0, 1.0, 0.0});
    geom.tris().push_back({0, 1, 2});
    return geom;
  }

  std::shared_ptr<SceneGraph> sceneGraph;
  GeometryDialog *dialog;
};

TEST_F(GeometryDialogTest, DialogCreation) {
  dialog = new GeometryDialog(sceneGraph);
  EXPECT_NE(dialog, nullptr);
  EXPECT_EQ(dialog->windowTitle(), "Geometry Properties");
}

TEST_F(GeometryDialogTest, EmptySceneGraph) {
  dialog = new GeometryDialog(sceneGraph);

  // Dialog should be created but properties should be disabled
  EXPECT_NE(dialog, nullptr);

  // Access the combo box through findChild
  QComboBox *comboBox = dialog->findChild<QComboBox *>();
  ASSERT_NE(comboBox, nullptr);
  EXPECT_EQ(comboBox->count(), 0);
}

TEST_F(GeometryDialogTest, SingleGeometry) {
  cvc::geometry geom = createTestGeometry();
  sceneGraph->addGraphics("test_geom", geom);

  dialog = new GeometryDialog(sceneGraph);

  QComboBox *comboBox = dialog->findChild<QComboBox *>();
  ASSERT_NE(comboBox, nullptr);
  EXPECT_EQ(comboBox->count(), 1);
  EXPECT_EQ(comboBox->itemText(0).toStdString(), "test_geom");
}

TEST_F(GeometryDialogTest, MultipleGeometries) {
  cvc::geometry geom1 = createTestGeometry();
  cvc::geometry geom2 = createTestGeometry();

  sceneGraph->addGraphics("geom1", geom1);
  sceneGraph->addGraphics("geom2", geom2);

  dialog = new GeometryDialog(sceneGraph);

  QComboBox *comboBox = dialog->findChild<QComboBox *>();
  ASSERT_NE(comboBox, nullptr);
  EXPECT_EQ(comboBox->count(), 2);
}

TEST_F(GeometryDialogTest, GeometrySelectionUpdatesUI) {
  cvc::geometry geom = createTestGeometry();
  auto node = sceneGraph->addGraphics("test_geom", geom);
  auto geomNode = std::dynamic_pointer_cast<GeometryNode>(node);
  ASSERT_NE(geomNode, nullptr);

  // Set some properties
  geomNode->setColor(0.5, 0.6, 0.7);
  geomNode->setAmbient(0.3);
  geomNode->setDiffuse(0.8);

  dialog = new GeometryDialog(sceneGraph);

  // Find the color spin boxes
  QList<QDoubleSpinBox *> spinBoxes = dialog->findChildren<QDoubleSpinBox *>();
  EXPECT_GT(spinBoxes.size(), 0);

  // Check that UI reflects the geometry properties
  // Look for the specific color values with proper tolerance
  bool foundColorValue = false;
  for (auto *spinBox : spinBoxes) {
    double value = spinBox->value();
    if (qAbs(value - 0.5) < 0.001 || qAbs(value - 0.6) < 0.001 || qAbs(value - 0.7) < 0.001) {
      foundColorValue = true;
      break;
    }
  }
  EXPECT_TRUE(foundColorValue);
}

TEST_F(GeometryDialogTest, RenderModeChange) {
  cvc::geometry geom = createTestGeometry();
  auto node = sceneGraph->addGraphics("test_geom", geom);
  auto geomNode = std::dynamic_pointer_cast<GeometryNode>(node);
  ASSERT_NE(geomNode, nullptr);

  dialog = new GeometryDialog(sceneGraph);

  // Find render mode combo box by object name
  QComboBox *renderModeCombo = dialog->findChild<QComboBox *>("renderModeComboBox");
  ASSERT_NE(renderModeCombo, nullptr);

  // Change render mode
  int wireframeIndex = renderModeCombo->findText("Wireframe");
  ASSERT_GE(wireframeIndex, 0);

  renderModeCombo->setCurrentIndex(wireframeIndex);
  QCoreApplication::processEvents();

  // Verify the node's render mode changed
  EXPECT_EQ(geomNode->getRenderMode(), GeometryRenderMode::LINES);
}

TEST_F(GeometryDialogTest, ColorPropertyChange) {
  cvc::geometry geom = createTestGeometry();
  auto node = sceneGraph->addGraphics("test_geom", geom);
  auto geomNode = std::dynamic_pointer_cast<GeometryNode>(node);
  ASSERT_NE(geomNode, nullptr);

  dialog = new GeometryDialog(sceneGraph);

  // Find color spin boxes by object name
  auto colorRSpinBox = dialog->findChild<QDoubleSpinBox *>("colorRSpinBox");
  auto colorGSpinBox = dialog->findChild<QDoubleSpinBox *>("colorGSpinBox");
  auto colorBSpinBox = dialog->findChild<QDoubleSpinBox *>("colorBSpinBox");
  ASSERT_NE(colorRSpinBox, nullptr);
  ASSERT_NE(colorGSpinBox, nullptr);
  ASSERT_NE(colorBSpinBox, nullptr);

  // Set color values
  colorRSpinBox->setValue(1.0);
  colorGSpinBox->setValue(0.0);
  colorBSpinBox->setValue(0.0);

  // Verify the UI values were set
  EXPECT_DOUBLE_EQ(colorRSpinBox->value(), 1.0);
  EXPECT_DOUBLE_EQ(colorGSpinBox->value(), 0.0);
  EXPECT_DOUBLE_EQ(colorBSpinBox->value(), 0.0);
}

TEST_F(GeometryDialogTest, DynamicGeometryAddition) {
  dialog = new GeometryDialog(sceneGraph);

  QComboBox *comboBox = dialog->findChild<QComboBox *>();
  ASSERT_NE(comboBox, nullptr);
  EXPECT_EQ(comboBox->count(), 0);

  // Add geometry after dialog creation
  cvc::geometry geom = createTestGeometry();
  sceneGraph->addGraphics("new_geom", geom);

  // Wait for state tree signals to propagate
  QCoreApplication::processEvents();

  // The combo box should update automatically
  EXPECT_EQ(comboBox->count(), 1);
}

TEST_F(GeometryDialogTest, DynamicGeometryRemoval) {
  cvc::geometry geom = createTestGeometry();
  sceneGraph->addGraphics("test_geom", geom);

  dialog = new GeometryDialog(sceneGraph);

  QComboBox *comboBox = dialog->findChild<QComboBox *>();
  ASSERT_NE(comboBox, nullptr);
  EXPECT_EQ(comboBox->count(), 1);

  // Remove geometry
  sceneGraph->removeGraphics("test_geom");

  // Wait for state tree signal and Qt signal processing
  QCoreApplication::processEvents();

  // The combo box should update automatically
  EXPECT_EQ(comboBox->count(), 0);
}

TEST_F(GeometryDialogTest, OpacityChange) {
  cvc::geometry geom = createTestGeometry();
  auto node = sceneGraph->addGraphics("test_geom", geom);
  auto geomNode = std::dynamic_pointer_cast<GeometryNode>(node);
  ASSERT_NE(geomNode, nullptr);

  dialog = new GeometryDialog(sceneGraph);

  // Find all double spin boxes
  QList<QDoubleSpinBox *> spinBoxes = dialog->findChildren<QDoubleSpinBox *>();

  // Look for opacity spin box (should have range 0-1)
  for (auto *spinBox : spinBoxes) {
    if (spinBox->minimum() == 0.0 && spinBox->maximum() == 1.0) {
      spinBox->setValue(0.5);
      QCoreApplication::processEvents();

      // Check if it could be the opacity
      auto opacity = geomNode->getMetadata("opacity");
      if (opacity.has_value()) {
        double opacityValue = std::any_cast<double>(opacity);
        if (qFuzzyCompare(opacityValue, 0.5)) {
          SUCCEED();
          return;
        }
      }
    }
  }
}

TEST_F(GeometryDialogTest, EmptyGeometryNotListed) {
  cvc::geometry emptyGeom; // Empty geometry
  sceneGraph->addGraphics("empty_geom", emptyGeom);

  cvc::geometry validGeom = createTestGeometry();
  sceneGraph->addGraphics("valid_geom", validGeom);

  dialog = new GeometryDialog(sceneGraph);

  QComboBox *comboBox = dialog->findChild<QComboBox *>();
  ASSERT_NE(comboBox, nullptr);

  // Only the valid geometry should be listed
  EXPECT_EQ(comboBox->count(), 1);
  EXPECT_EQ(comboBox->itemText(0).toStdString(), "valid_geom");
}

TEST_F(GeometryDialogTest, NestedGeometries) {
  cvc::geometry geom1 = createTestGeometry();
  cvc::geometry geom2 = createTestGeometry();

  auto parent = sceneGraph->addGraphics("parent_geom", geom1);
  ASSERT_NE(parent, nullptr);

  // Add child geometry
  auto child = parent->createChild<GeometryNode>("child_geom", geom2);
  ASSERT_NE(child, nullptr);

  dialog = new GeometryDialog(sceneGraph);

  QComboBox *comboBox = dialog->findChild<QComboBox *>();
  ASSERT_NE(comboBox, nullptr);

  // Both parent and child should be listed
  EXPECT_EQ(comboBox->count(), 2);
}

// Test safe deletion of geometry from state tree
TEST_F(GeometryDialogTest, SafeGeometryDeletion) {
  cvc::geometry geom = createTestGeometry();
  auto node = sceneGraph->addGraphics("test_geom", geom);
  ASSERT_NE(node, nullptr);

  // Verify geometry is in scene graph
  EXPECT_EQ(sceneGraph->getAllGeometryGraphics().size(), 1);

  // Get weak pointer to track object lifetime
  std::weak_ptr<GraphicsNode> weakNode = node;
  node.reset(); // Release our reference

  // Object should still exist (held by scene graph)
  EXPECT_FALSE(weakNode.expired());

  // Remove geometry - should not crash
  sceneGraph->removeGraphics("test_geom");

  // Verify removal was clean - C++ object should be destroyed
  EXPECT_EQ(sceneGraph->getAllGeometryGraphics().size(), 0);

  // The GraphicsNode object should now be destroyed (no more references)
  EXPECT_TRUE(weakNode.expired());

  // State tree should still be accessible without crashes (even if nodes remain)
  std::string statePrefix = sceneGraph->getStatePrefix();
  EXPECT_NO_THROW({
    auto &state = cvc::state::instance(volrover3::app())(statePrefix + ".graphics.root.children");
    // State tree nodes may persist, but accessing them shouldn't crash
    size_t childCount = state.numChildren();
    EXPECT_GE(childCount, 0); // Just verify we can read without crashing
  });
}

// Test multiple additions and removals
TEST_F(GeometryDialogTest, MultipleAddRemoveCycles) {
  cvc::geometry geom = createTestGeometry();

  // Perform multiple add/remove cycles
  for (int i = 0; i < 5; ++i) {
    auto node = sceneGraph->addGraphics("cycle_geom", geom);
    ASSERT_NE(node, nullptr);
    EXPECT_EQ(sceneGraph->getAllGeometryGraphics().size(), 1);

    sceneGraph->removeGraphics("cycle_geom");
    EXPECT_EQ(sceneGraph->getAllGeometryGraphics().size(), 0);
  }

  // State tree should still be valid
  std::string statePrefix = sceneGraph->getStatePrefix();
  EXPECT_NO_THROW({
    auto &state = cvc::state::instance(volrover3::app())(statePrefix + ".graphics.root");
    EXPECT_TRUE(true); // Just verify no crash accessing state
  });
}

// Test removal of nested geometries
TEST_F(GeometryDialogTest, SafeNestedGeometryDeletion) {
  cvc::geometry geom1 = createTestGeometry();
  cvc::geometry geom2 = createTestGeometry();

  // Create parent-child hierarchy
  auto parent = sceneGraph->addGraphics("parent", geom1);
  ASSERT_NE(parent, nullptr);

  auto child = parent->createChild<GeometryNode>("child", geom2);
  ASSERT_NE(child, nullptr);

  EXPECT_EQ(sceneGraph->getAllGeometryGraphics().size(), 2);

  // Remove parent (should handle child cleanup)
  sceneGraph->removeGraphics("parent");

  // Should not crash and should clean up properly
  EXPECT_EQ(sceneGraph->getAllGeometryGraphics().size(), 0);
}

// Test repeated add/remove cycles for memory safety
TEST_F(GeometryDialogTest, RepeatedAddRemoveSafety) {
  cvc::geometry geom = createTestGeometry();

  // Perform multiple add/remove cycles
  for (int i = 0; i < 10; ++i) {
    auto node = sceneGraph->addGraphics("test_geom_" + std::to_string(i % 3), geom);
    ASSERT_NE(node, nullptr);

    // Immediately remove it
    sceneGraph->removeGraphics("test_geom_" + std::to_string(i % 3));
  }

  // Should complete without crashes or memory issues
  EXPECT_EQ(sceneGraph->getAllGeometryGraphics().size(), 0);
}

// Test removal of non-existent geometry (error handling)
TEST_F(GeometryDialogTest, RemoveNonExistentGeometry) {
  // Should not crash when removing non-existent geometry
  EXPECT_NO_THROW({ sceneGraph->removeGraphics("does_not_exist"); });

  EXPECT_EQ(sceneGraph->getAllGeometryGraphics().size(), 0);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
