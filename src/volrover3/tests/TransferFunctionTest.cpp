#include <gtest/gtest.h>
#include <QApplication>
#include <volrover3/TransferFunctionWidget.h>

// Need QApplication for Qt widgets
class TransferFunctionTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!QApplication::instance()) {
            int argc = 0;
            char** argv = nullptr;
            app = new QApplication(argc, argv);
        }
    }
    
    void SetUp() override {
        widget = new TransferFunctionWidget();
    }
    
    void TearDown() override {
        delete widget;
    }
    
    static QApplication* app;
    TransferFunctionWidget* widget;
};

QApplication* TransferFunctionTest::app = nullptr;

TEST_F(TransferFunctionTest, WidgetCreation) {
    EXPECT_NE(widget, nullptr);
}

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
    EXPECT_GE(colorTable.size(), 8);  // At least 2 points * 4 values
    
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
    
    // Should have at least 2 opacity points (scalar, opacity)
    EXPECT_GE(opacityTable.size(), 4);  // At least 2 points * 2 values
    
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
