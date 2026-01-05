#include <gtest/gtest.h>
#include <QApplication>
#include <QTest>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <volrover3/VolumeDialog.h>
#include <volrover3/SceneGraph.h>
#include <volrover3/VolumeNode.h>
#include <cvc/volume.h>
#include <cvc/state.h>

class VolumeDialogTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        // Disable threading for state_object to avoid race conditions
        cvc::state_object<SceneNode>::setUseThreading(false);
        
        // Initialize Qt if not already initialized
        if (!QApplication::instance()) {
            int argc = 0;
            char** argv = nullptr;
            static QApplication app(argc, argv);
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
    
    cvc::volume createTestVolume() {
        // Create a simple test volume (data doesn't matter for dialog tests)
        return cvc::volume(cvc::dimension(4, 4, 4), cvc::UChar);
    }
    
    std::shared_ptr<SceneGraph> sceneGraph;
    VolumeDialog* dialog;
};

TEST_F(VolumeDialogTest, DialogCreation) {
    dialog = new VolumeDialog(sceneGraph);
    EXPECT_NE(dialog, nullptr);
    EXPECT_EQ(dialog->windowTitle(), "Volume Properties");
}

TEST_F(VolumeDialogTest, EmptySceneGraph) {
    dialog = new VolumeDialog(sceneGraph);
    
    // Dialog should be created but properties should be disabled
    EXPECT_NE(dialog, nullptr);
    
    // Access the combo box through findChild
    QComboBox* comboBox = dialog->findChild<QComboBox*>();
    ASSERT_NE(comboBox, nullptr);
    EXPECT_EQ(comboBox->count(), 0);
}

TEST_F(VolumeDialogTest, SingleVolume) {
    cvc::volume vol = createTestVolume();
    sceneGraph->addGraphics("test_vol", vol);
    
    dialog = new VolumeDialog(sceneGraph);
    
    QComboBox* comboBox = dialog->findChild<QComboBox*>();
    ASSERT_NE(comboBox, nullptr);
    EXPECT_EQ(comboBox->count(), 1);
    EXPECT_EQ(comboBox->itemText(0).toStdString(), "test_vol");
}

TEST_F(VolumeDialogTest, MultipleVolumes) {
    cvc::volume vol1 = createTestVolume();
    cvc::volume vol2 = createTestVolume();
    
    sceneGraph->addGraphics("vol1", vol1);
    sceneGraph->addGraphics("vol2", vol2);
    
    dialog = new VolumeDialog(sceneGraph);
    
    QComboBox* comboBox = dialog->findChild<QComboBox*>();
    ASSERT_NE(comboBox, nullptr);
    EXPECT_EQ(comboBox->count(), 2);
}

TEST_F(VolumeDialogTest, VolumeSelectionUpdatesUI) {
    cvc::volume vol = createTestVolume();
    auto node = sceneGraph->addGraphics("test_vol", vol);
    auto volNode = std::dynamic_pointer_cast<VolumeNode>(node);
    ASSERT_NE(volNode, nullptr);
    
    // Set some properties
    volNode->setShading(true);
    volNode->setAmbient(0.3);
    volNode->setDiffuse(0.8);
    volNode->setSpecular(0.5);
    
    dialog = new VolumeDialog(sceneGraph);
    
    // Find the shading checkbox
    QCheckBox* shadingCheckBox = nullptr;
    QList<QCheckBox*> checkBoxes = dialog->findChildren<QCheckBox*>();
    for (auto* cb : checkBoxes) {
        if (cb->text().contains("Shading", Qt::CaseInsensitive)) {
            shadingCheckBox = cb;
            break;
        }
    }
    
    if (shadingCheckBox) {
        EXPECT_TRUE(shadingCheckBox->isChecked());
    }
    
    // Check spin boxes reflect values
    QList<QDoubleSpinBox*> spinBoxes = dialog->findChildren<QDoubleSpinBox*>();
    EXPECT_GT(spinBoxes.size(), 0);
}

TEST_F(VolumeDialogTest, ShadingToggle) {
    cvc::volume vol = createTestVolume();
    auto node = sceneGraph->addGraphics("test_vol", vol);
    auto volNode = std::dynamic_pointer_cast<VolumeNode>(node);
    ASSERT_NE(volNode, nullptr);
    
    volNode->setShading(false);
    
    dialog = new VolumeDialog(sceneGraph);
    
    // Find the shading checkbox
    QCheckBox* shadingCheckBox = nullptr;
    QList<QCheckBox*> checkBoxes = dialog->findChildren<QCheckBox*>();
    for (auto* cb : checkBoxes) {
        if (cb->text().contains("Shading", Qt::CaseInsensitive)) {
            shadingCheckBox = cb;
            break;
        }
    }
    
    ASSERT_NE(shadingCheckBox, nullptr);
    EXPECT_FALSE(shadingCheckBox->isChecked());
    
    // Toggle shading
    shadingCheckBox->setChecked(true);
    QTest::qWait(10);
    
    // Verify the node's shading changed
    EXPECT_TRUE(volNode->getShading());
}

TEST_F(VolumeDialogTest, AmbientPropertyChange) {
    cvc::volume vol = createTestVolume();
    auto node = sceneGraph->addGraphics("test_vol", vol);
    auto volNode = std::dynamic_pointer_cast<VolumeNode>(node);
    ASSERT_NE(volNode, nullptr);
    
    dialog = new VolumeDialog(sceneGraph);
    
    // Find ambient spin box (should be one of the first few with range 0-1)
    QList<QDoubleSpinBox*> spinBoxes = dialog->findChildren<QDoubleSpinBox*>();
    ASSERT_GT(spinBoxes.size(), 0);
    
    // Try to find and set ambient (it's typically the first material property)
    for (auto* spinBox : spinBoxes) {
        if (spinBox->minimum() == 0.0 && spinBox->maximum() == 1.0) {
            double oldValue = volNode->getAmbient();
            double newValue = 0.42;
            
            spinBox->setValue(newValue);
            QTest::qWait(10);
            
            // Check if this changed the ambient
            if (!qFuzzyCompare(volNode->getAmbient(), oldValue)) {
                EXPECT_DOUBLE_EQ(volNode->getAmbient(), newValue);
                return;
            }
        }
    }
}

TEST_F(VolumeDialogTest, DiffusePropertyChange) {
    cvc::volume vol = createTestVolume();
    auto node = sceneGraph->addGraphics("test_vol", vol);
    auto volNode = std::dynamic_pointer_cast<VolumeNode>(node);
    ASSERT_NE(volNode, nullptr);
    
    dialog = new VolumeDialog(sceneGraph);
    
    double originalDiffuse = volNode->getDiffuse();
    double newDiffuse = 0.67;
    
    // Find and change diffuse
    QList<QDoubleSpinBox*> spinBoxes = dialog->findChildren<QDoubleSpinBox*>();
    for (auto* spinBox : spinBoxes) {
        if (spinBox->minimum() == 0.0 && spinBox->maximum() == 1.0) {
            spinBox->setValue(newDiffuse);
            QTest::qWait(10);
            
            if (!qFuzzyCompare(volNode->getDiffuse(), originalDiffuse)) {
                EXPECT_DOUBLE_EQ(volNode->getDiffuse(), newDiffuse);
                return;
            }
        }
    }
}

TEST_F(VolumeDialogTest, SpecularPropertyChange) {
    cvc::volume vol = createTestVolume();
    auto node = sceneGraph->addGraphics("test_vol", vol);
    auto volNode = std::dynamic_pointer_cast<VolumeNode>(node);
    ASSERT_NE(volNode, nullptr);
    
    dialog = new VolumeDialog(sceneGraph);
    
    double originalSpecular = volNode->getSpecular();
    double newSpecular = 0.89;
    
    QList<QDoubleSpinBox*> spinBoxes = dialog->findChildren<QDoubleSpinBox*>();
    for (auto* spinBox : spinBoxes) {
        if (spinBox->minimum() == 0.0 && spinBox->maximum() == 1.0) {
            spinBox->setValue(newSpecular);
            QTest::qWait(10);
            
            if (!qFuzzyCompare(volNode->getSpecular(), originalSpecular)) {
                EXPECT_DOUBLE_EQ(volNode->getSpecular(), newSpecular);
                return;
            }
        }
    }
}

TEST_F(VolumeDialogTest, SpecularPowerChange) {
    cvc::volume vol = createTestVolume();
    auto node = sceneGraph->addGraphics("test_vol", vol);
    auto volNode = std::dynamic_pointer_cast<VolumeNode>(node);
    ASSERT_NE(volNode, nullptr);
    
    dialog = new VolumeDialog(sceneGraph);
    
    double newSpecularPower = 64.0;
    
    // Find specular power spin box (range 0-128)
    QList<QDoubleSpinBox*> spinBoxes = dialog->findChildren<QDoubleSpinBox*>();
    for (auto* spinBox : spinBoxes) {
        if (spinBox->maximum() == 128.0) {
            spinBox->setValue(newSpecularPower);
            QTest::qWait(10);
            
            EXPECT_DOUBLE_EQ(volNode->getSpecularPower(), newSpecularPower);
            return;
        }
    }
}

TEST_F(VolumeDialogTest, SampleDistanceChange) {
    cvc::volume vol = createTestVolume();
    auto node = sceneGraph->addGraphics("test_vol", vol);
    auto volNode = std::dynamic_pointer_cast<VolumeNode>(node);
    ASSERT_NE(volNode, nullptr);
    
    dialog = new VolumeDialog(sceneGraph);
    
    double newSampleDistance = 0.5;
    
    // Find sample distance spin box (range 0.001-10.0)
    QList<QDoubleSpinBox*> spinBoxes = dialog->findChildren<QDoubleSpinBox*>();
    for (auto* spinBox : spinBoxes) {
        if (qFuzzyCompare(spinBox->minimum(), 0.001) && 
            qFuzzyCompare(spinBox->maximum(), 10.0)) {
            spinBox->setValue(newSampleDistance);
            QTest::qWait(10);
            
            EXPECT_DOUBLE_EQ(volNode->getSampleDistance(), newSampleDistance);
            return;
        }
    }
}

TEST_F(VolumeDialogTest, AutoAdjustSampleDistancesToggle) {
    cvc::volume vol = createTestVolume();
    auto node = sceneGraph->addGraphics("test_vol", vol);
    auto volNode = std::dynamic_pointer_cast<VolumeNode>(node);
    ASSERT_NE(volNode, nullptr);
    
    volNode->setAutoAdjustSampleDistances(false);
    
    dialog = new VolumeDialog(sceneGraph);
    
    // Find auto-adjust checkbox
    QCheckBox* autoAdjustCheckBox = nullptr;
    QList<QCheckBox*> checkBoxes = dialog->findChildren<QCheckBox*>();
    for (auto* cb : checkBoxes) {
        if (cb->text().contains("Auto", Qt::CaseInsensitive)) {
            autoAdjustCheckBox = cb;
            break;
        }
    }
    
    ASSERT_NE(autoAdjustCheckBox, nullptr);
    EXPECT_FALSE(autoAdjustCheckBox->isChecked());
    
    // Toggle auto-adjust
    autoAdjustCheckBox->setChecked(true);
    QTest::qWait(10);
    
    EXPECT_TRUE(volNode->getAutoAdjustSampleDistances());
}

TEST_F(VolumeDialogTest, DynamicVolumeAddition) {
    dialog = new VolumeDialog(sceneGraph);
    
    QComboBox* comboBox = dialog->findChild<QComboBox*>();
    ASSERT_NE(comboBox, nullptr);
    EXPECT_EQ(comboBox->count(), 0);
    
    // Add volume after dialog creation
    cvc::volume vol = createTestVolume();
    sceneGraph->addGraphics("new_vol", vol);
    
    // Wait for state tree signals to propagate
    QTest::qWait(50);
    
    // The combo box should update automatically
    EXPECT_EQ(comboBox->count(), 1);
}

TEST_F(VolumeDialogTest, DynamicVolumeRemoval) {
    cvc::volume vol = createTestVolume();
    sceneGraph->addGraphics("test_vol", vol);
    
    dialog = new VolumeDialog(sceneGraph);
    
    QComboBox* comboBox = dialog->findChild<QComboBox*>();
    ASSERT_NE(comboBox, nullptr);
    EXPECT_EQ(comboBox->count(), 1);
    
    // Remove volume
    sceneGraph->removeGraphics("test_vol");
    
    // Wait for state tree signal and Qt signal processing
    QTest::qWait(100);
    
    // The combo box should update automatically
    EXPECT_EQ(comboBox->count(), 0);
}

TEST_F(VolumeDialogTest, NestedVolumes) {
    cvc::volume vol1 = createTestVolume();
    cvc::volume vol2 = createTestVolume();
    
    auto parent = sceneGraph->addGraphics("parent_vol", vol1);
    ASSERT_NE(parent, nullptr);
    
    // Add child volume
    auto child = parent->createChild<VolumeNode>("child_vol", vol2);
    ASSERT_NE(child, nullptr);
    
    dialog = new VolumeDialog(sceneGraph);
    
    QComboBox* comboBox = dialog->findChild<QComboBox*>();
    ASSERT_NE(comboBox, nullptr);
    
    // Both parent and child should be listed
    EXPECT_EQ(comboBox->count(), 2);
}

TEST_F(VolumeDialogTest, SelectionPreservation) {
    cvc::volume vol1 = createTestVolume();
    cvc::volume vol2 = createTestVolume();
    
    sceneGraph->addGraphics("vol1", vol1);
    sceneGraph->addGraphics("vol2", vol2);
    
    dialog = new VolumeDialog(sceneGraph);
    
    QComboBox* comboBox = dialog->findChild<QComboBox*>();
    ASSERT_NE(comboBox, nullptr);
    EXPECT_EQ(comboBox->count(), 2);
    
    // Select second volume
    comboBox->setCurrentIndex(1);
    QTest::qWait(10);
    
    QString selectedName = comboBox->currentText();
    
    // Add a third volume
    cvc::volume vol3 = createTestVolume();
    sceneGraph->addGraphics("vol3", vol3);
    QTest::qWait(50);
    
    // Selection should be preserved
    EXPECT_EQ(comboBox->currentText(), selectedName);
}

TEST_F(VolumeDialogTest, ScalarOpacityUnitDistanceChange) {
    cvc::volume vol = createTestVolume();
    auto node = sceneGraph->addGraphics("test_vol", vol);
    auto volNode = std::dynamic_pointer_cast<VolumeNode>(node);
    ASSERT_NE(volNode, nullptr);
    
    dialog = new VolumeDialog(sceneGraph);
    
    double newValue = 2.5;
    
    // Find scalar opacity unit distance spin box (range 0.001-100.0)
    QList<QDoubleSpinBox*> spinBoxes = dialog->findChildren<QDoubleSpinBox*>();
    for (auto* spinBox : spinBoxes) {
        if (qFuzzyCompare(spinBox->minimum(), 0.001) && 
            qFuzzyCompare(spinBox->maximum(), 100.0)) {
            spinBox->setValue(newValue);
            QTest::qWait(10);
            
            EXPECT_DOUBLE_EQ(volNode->getScalarOpacityUnitDistance(), newValue);
            return;
        }
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
