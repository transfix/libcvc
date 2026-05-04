#include <QApplication>
#include <cvc/app.h>
#include <cvc/state.h>
#include <gtest/gtest.h>
#include <volrover3/AppState.h>
#include <volrover3/TransferFunctionWidget.h>
#include <volrover3/VolumeNode.h>
#include <volrover3/volrover3_app.h>

// Need QApplication for Qt widgets
class TransferFunctionTest : public ::testing::Test {
protected:
  cvc::app ctx;
  static void SetUpTestSuite() {
    if (!QApplication::instance()) {
      int argc = 0;
      char **argv = nullptr;
      app = new QApplication(argc, argv);
    }
  }

  void SetUp() override {
    widget = new TransferFunctionWidget();
    appState = &AppState::instance();
  }

  void TearDown() override { delete widget; }

  static QApplication *app;
  TransferFunctionWidget *widget;
  AppState *appState;
};

QApplication *TransferFunctionTest::app = nullptr;

TEST_F(TransferFunctionTest, WidgetCreation) { EXPECT_NE(widget, nullptr); }

TEST_F(TransferFunctionTest, DataRangeInitialization) {
  // Default data range should be 0.0 to 1.0
  widget->setDataRange(0.0, 1.0);

  // Should not crash
  SUCCEED();
}

TEST_F(TransferFunctionTest, ColorTableGeneration) {
  widget->setDataRange(-5.0, 10.0);

  auto colorTable = widget->getColorTable();

  // Should have at least 2 color points (scalar, r, g, b)
  EXPECT_GE(colorTable.size(), 8); // At least 2 points * 4 values

  // Check that color table has groups of 4 values
  EXPECT_EQ(colorTable.size() % 4, 0);

  // Scalar values should be within data range
  for (size_t i = 0; i < colorTable.size() / 4; ++i) {
    double scalar = colorTable[i * 4];
    EXPECT_GE(scalar, -5.0);
    EXPECT_LE(scalar, 10.0);

    // RGB values should be in [0, 1]
    EXPECT_GE(colorTable[i * 4 + 1], 0.0);
    EXPECT_LE(colorTable[i * 4 + 1], 1.0);
    EXPECT_GE(colorTable[i * 4 + 2], 0.0);
    EXPECT_LE(colorTable[i * 4 + 2], 1.0);
    EXPECT_GE(colorTable[i * 4 + 3], 0.0);
    EXPECT_LE(colorTable[i * 4 + 3], 1.0);
  }
}

TEST_F(TransferFunctionTest, OpacityTableGeneration) {
  widget->setDataRange(-5.0, 10.0);

  auto opacityTable = widget->getOpacityTable();

  // Opacity table may be empty until user adds points or a preset is applied
  // This is expected behavior - check that it's valid when present
  if (opacityTable.size() > 0) {
    // Check that opacity table has groups of 2 values
    EXPECT_EQ(opacityTable.size() % 2, 0);

    // Scalar values should be within data range
    for (size_t i = 0; i < opacityTable.size() / 2; ++i) {
      double scalar = opacityTable[i * 2];
      double opacity = opacityTable[i * 2 + 1];

      EXPECT_GE(scalar, -5.0);
      EXPECT_LE(scalar, 10.0);

      // Opacity should be in [0, 1]
      EXPECT_GE(opacity, 0.0);
      EXPECT_LE(opacity, 1.0);
    }
  }

  // Test passes - opacity table is either empty or properly formatted
  SUCCEED();
}

TEST_F(TransferFunctionTest, OpacityIndependentOfColor) {
  widget->setDataRange(0.0, 100.0);

  // Get initial opacity table
  auto opacityBefore = widget->getOpacityTable();

  // Apply a color preset (this should NOT reset opacity)
  widget->applyPreset("Rainbow");

  // Get opacity table after color preset
  auto opacityAfter = widget->getOpacityTable();

  // Opacity table should be unchanged (unless it was empty initially)
  if (opacityBefore.size() > 0) {
    EXPECT_EQ(opacityBefore.size(), opacityAfter.size());
  }
}

TEST_F(TransferFunctionTest, DataRangeMapping) {
  // Set a specific data range
  widget->setDataRange(10.0, 20.0);

  auto colorTable = widget->getColorTable();
  auto opacityTable = widget->getOpacityTable();

  // First scalar should be near minimum
  if (colorTable.size() >= 4) {
    EXPECT_NEAR(colorTable[0], 10.0, 0.1);
  }

  // Last scalar should be near maximum
  if (colorTable.size() >= 8) {
    size_t lastIdx = (colorTable.size() / 4 - 1) * 4;
    EXPECT_NEAR(colorTable[lastIdx], 20.0, 0.1);
  }

  // Same for opacity
  if (opacityTable.size() >= 4) {
    EXPECT_NEAR(opacityTable[0], 10.0, 0.1);

    size_t lastIdx = (opacityTable.size() / 2 - 1) * 2;
    EXPECT_NEAR(opacityTable[lastIdx], 20.0, 0.1);
  }
}

TEST_F(TransferFunctionTest, PresetApplication) {
  widget->setDataRange(0.0, 1.0);

  // Test different presets
  widget->applyPreset("Grayscale");
  auto grayscale = widget->getColorTable();
  EXPECT_GT(grayscale.size(), 0);

  widget->applyPreset("Rainbow");
  auto rainbow = widget->getColorTable();
  EXPECT_GT(rainbow.size(), 0);

  widget->applyPreset("Hot");
  auto hot = widget->getColorTable();
  EXPECT_GT(hot.size(), 0);

  widget->applyPreset("Cool");
  auto cool = widget->getColorTable();
  EXPECT_GT(cool.size(), 0);

  widget->applyPreset("X-Ray");
  auto xray = widget->getColorTable();
  EXPECT_GT(xray.size(), 0);
}

TEST_F(TransferFunctionTest, SignalEmission) {
  bool signalReceived = false;

  QObject::connect(widget, &TransferFunctionWidget::transferFunctionChanged,
                   [&signalReceived]() { signalReceived = true; });

  // Applying a preset should emit signal
  widget->applyPreset("Grayscale");

  // Process events to ensure signal is delivered
  QApplication::processEvents();

  EXPECT_TRUE(signalReceived);
}

// ===========================
// Feedback Loop Prevention Tests
// ===========================

TEST_F(TransferFunctionTest, NoFeedbackLoopOnStateUpdate) {
  // This test verifies that the counter mechanism prevents feedback loops
  // by checking that repeated setTransferFunction calls don't cause exponential growth

  auto volume = std::make_shared<VolumeNode>(ctx, "test_volume");

  widget->setDataRange(0.0, 100.0);
  widget->applyPreset("Rainbow");

  auto initialColor = widget->getColorTable();
  size_t initialSize = initialColor.size();

  // Simulate rapid updates (this would cause feedback loops in the old implementation)
  for (int i = 0; i < 5; ++i) {
    auto colorTable = widget->getColorTable();
    auto opacityTable = widget->getOpacityTable();

    // This should NOT trigger a reload that increases the table size
    volume->setTransferFunction(colorTable, opacityTable);

    // Verify size hasn't grown
    auto currentSize = widget->getColorTable().size();
    EXPECT_EQ(currentSize, initialSize)
        << "Iteration " << i << ": size changed from " << initialSize << " to " << currentSize;
  }
}

TEST_F(TransferFunctionTest, StateRoundTripPreservesData) {
  // Test that data survives round-trip through state tree without corruption

  auto volume = std::make_shared<VolumeNode>(ctx, "test_volume");

  widget->setDataRange(0.0, 255.0);
  widget->applyPreset("Rainbow");

  auto originalColor = widget->getColorTable();
  auto originalOpacity = widget->getOpacityTable();

  // Save to volume state
  volume->setTransferFunction(originalColor, originalOpacity);

  // Retrieve from volume state
  auto retrievedColor = volume->getTransferFunctionColorTable();
  auto retrievedOpacity = volume->getTransferFunctionOpacityTable();

  // Should have same number of values
  EXPECT_EQ(originalColor.size(), retrievedColor.size());
  EXPECT_EQ(originalOpacity.size(), retrievedOpacity.size());

  // Values should be very close (allowing for floating point precision with 6 decimal places)
  for (size_t i = 0; i < std::min(originalColor.size(), retrievedColor.size()); ++i) {
    EXPECT_NEAR(originalColor[i], retrievedColor[i], 1e-5) << "Color mismatch at index " << i;
  }

  for (size_t i = 0; i < std::min(originalOpacity.size(), retrievedOpacity.size()); ++i) {
    EXPECT_NEAR(originalOpacity[i], retrievedOpacity[i], 1e-5) << "Opacity mismatch at index " << i;
  }
}

TEST_F(TransferFunctionTest, PerVolumeStateSeparation) {
  // Test that each volume has independent transfer function state

  auto volume1 = std::make_shared<VolumeNode>(ctx, "volume_1");
  auto volume2 = std::make_shared<VolumeNode>(ctx, "volume_2");

  // Set different transfer functions for each volume
  widget->setDataRange(0.0, 100.0);
  widget->applyPreset("Rainbow");
  volume1->setTransferFunction(widget->getColorTable(), widget->getOpacityTable());
  auto volume1Color = volume1->getTransferFunctionColorTable();

  widget->applyPreset("Grayscale");
  volume2->setTransferFunction(widget->getColorTable(), widget->getOpacityTable());
  auto volume2Color = volume2->getTransferFunctionColorTable();

  // They should be different sizes (Rainbow has 5 points, Grayscale has 2)
  EXPECT_NE(volume1Color.size(), volume2Color.size())
      << "Volumes should have independent transfer functions";

  // Verify each volume retained its own TF
  EXPECT_EQ(volume1Color.size(), 5 * 4); // 5 color points * 4 values each
  EXPECT_EQ(volume2Color.size(), 2 * 4); // 2 color points * 4 values each

  // Verify they're actually different
  EXPECT_NE(volume1Color, volume2Color);
}

TEST_F(TransferFunctionTest, DefaultTransferFunctionSet) {
  // Test that VolumeNode gets a default transfer function when created

  auto volume = std::make_shared<VolumeNode>(ctx, "test_volume");

  // Set default TF
  volume->setDefaultTransferFunction();

  // Should have a valid transfer function in state
  auto colorTable = volume->getTransferFunctionColorTable();
  auto opacityTable = volume->getTransferFunctionOpacityTable();

  EXPECT_GT(colorTable.size(), 0) << "Default color table should not be empty";
  EXPECT_GT(opacityTable.size(), 0) << "Default opacity table should not be empty";

  // Should be properly formatted
  EXPECT_EQ(colorTable.size() % 4, 0);
  EXPECT_EQ(opacityTable.size() % 2, 0);
}

// ===========================
// State Tree Integration Tests
// ===========================

// NOTE: Transfer function storage moved to per-volume state in VolumeNode
// These tests are commented out as they tested the old global AppState TF storage
/*
TEST_F(TransferFunctionTest, TransferFunctionStateStorage) {
    // In actual usage, MainWindow saves transfer function to AppState
    // when widget emits transferFunctionChanged signal

    widget->setDataRange(0.0, 100.0);
    widget->applyPreset("Rainbow");

    auto colorTable = widget->getColorTable();
    auto opacityTable = widget->getOpacityTable();

    // Simulate what MainWindow does
    appState->setTransferFunctionColorTable(colorTable);
    appState->setTransferFunctionOpacityTable(opacityTable);

    // Verify state tree has the data
    auto& stateTree = cvc::state::instance(volrover3::app())("volrover3");
    EXPECT_TRUE(stateTree("transfer_function_color").initialized());
    EXPECT_TRUE(stateTree("transfer_function_opacity").initialized());

    // Retrieve from AppState
    auto retrievedColor = appState->transferFunctionColorTable();
    auto retrievedOpacity = appState->transferFunctionOpacityTable();

    EXPECT_EQ(colorTable.size(), retrievedColor.size());
    EXPECT_EQ(opacityTable.size(), retrievedOpacity.size());
}

TEST_F(TransferFunctionTest, TransferFunctionCallback) {
    int callback_count = 0;

    // Register callback for transfer function changes
    auto connection = appState->onTransferFunctionChanged([&callback_count]() {
        callback_count++;
    });

    // Clear transfer_function_changed flag
    auto& stateTree = cvc::state::instance(volrover3::app())("volrover3");
    stateTree("transfer_function_changed").value(false);

    // Change transfer function via AppState (simulating MainWindow)
    widget->applyPreset("Hot");
    auto colorTable = widget->getColorTable();
    auto opacityTable = widget->getOpacityTable();

    appState->setTransferFunctionColorTable(colorTable);
    appState->setTransferFunctionOpacityTable(opacityTable);

    // Callback should have been triggered
    EXPECT_GT(callback_count, 0);

    connection.disconnect();
}

TEST_F(TransferFunctionTest, TransferFunctionPersistence) {
    // Set transfer function
    widget->setDataRange(-10.0, 10.0);
    widget->applyPreset("X-Ray");

    auto originalColor = widget->getColorTable();
    auto originalOpacity = widget->getOpacityTable();

    appState->setTransferFunctionColorTable(originalColor);
    appState->setTransferFunctionOpacityTable(originalOpacity);

    // Retrieve from state tree multiple times
    auto retrieved1 = appState->transferFunctionColorTable();
    auto retrieved2 = appState->transferFunctionColorTable();

    // Should be consistent
    EXPECT_EQ(retrieved1.size(), retrieved2.size());
    for (size_t i = 0; i < retrieved1.size(); ++i) {
        EXPECT_DOUBLE_EQ(retrieved1[i], retrieved2[i]);
    }
}

TEST_F(TransferFunctionTest, SignalAndStateIntegration) {
    // Test that Qt signals and AppState callbacks work together
    // In practice: Widget emits signal → MainWindow saves to AppState → callbacks fire

    int callback_count = 0;

    // Register AppState callback
    auto connection = appState->onTransferFunctionChanged([&callback_count]() {
        callback_count++;
    });

    // Manually trigger state change (simulating MainWindow's save operation)
    std::vector<double> testColorTable = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    std::vector<double> testOpacityTable = {0.0, 0.5, 1.0};

    appState->setTransferFunctionColorTable(testColorTable);
    appState->setTransferFunctionOpacityTable(testOpacityTable);

    // Both setters toggle transfer_function_changed, so callback fires twice
    EXPECT_EQ(callback_count, 2);

    // Verify we can retrieve the data
    auto retrievedColor = appState->transferFunctionColorTable();
    auto retrievedOpacity = appState->transferFunctionOpacityTable();

    EXPECT_EQ(testColorTable.size(), retrievedColor.size());
    EXPECT_EQ(testOpacityTable.size(), retrievedOpacity.size());

    connection.disconnect();
}
*/
