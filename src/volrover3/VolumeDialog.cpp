#include <volrover3/VolumeDialog.h>
#include <volrover3/SceneGraph.h>
#include <volrover3/GraphicsNode.h>
#include <volrover3/VolumeNode.h>
#include <cvc/state.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QLabel>
#include <QMetaObject>

VolumeDialog::VolumeDialog(std::shared_ptr<SceneGraph> sceneGraph, QWidget *parent)
    : QDialog(parent)
    , m_sceneGraph(sceneGraph)
    , m_volumeComboBox(nullptr)
    , m_shadingCheckBox(nullptr)
    , m_ambientSpinBox(nullptr)
    , m_diffuseSpinBox(nullptr)
    , m_specularSpinBox(nullptr)
    , m_specularPowerSpinBox(nullptr)
    , m_scalarOpacityUnitDistanceSpinBox(nullptr)
    , m_sampleDistanceSpinBox(nullptr)
    , m_autoAdjustSampleDistancesCheckBox(nullptr)
    , m_updating(false)
{
    setWindowTitle(tr("Volume Properties"));
    setMinimumWidth(400);
    setupUI();
    connectSignals();
    populateVolumeList();
    
    // Connect to state tree to monitor for new/removed volumes
    if (m_sceneGraph) {
        std::string statePrefix = m_sceneGraph->getStatePrefix();
        std::string graphicsChildrenPath = statePrefix + ".graphics.root.children";
        
        try {
            auto& childrenState = cvc::state::instance()(graphicsChildrenPath);
            
            // Listen to dataChanged which fires when touch() is called
            // (i.e., when the collection structure changes)
            m_graphicsChildrenConnection = childrenState.dataChanged.connect(
                [this]() {
                    QMetaObject::invokeMethod(this, "onGraphicsChildrenChanged", Qt::QueuedConnection);
                }
            );
        } catch (...) {
            // State might not exist yet
        }
    }
}

void VolumeDialog::setupUI()
{
    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    
    // Volume Selection Group
    QGroupBox *selectionGroup = new QGroupBox(tr("Volume Selection"), this);
    QFormLayout *selectionLayout = new QFormLayout(selectionGroup);
    
    m_volumeComboBox = new QComboBox(this);
    selectionLayout->addRow(tr("Volume:"), m_volumeComboBox);
    
    mainLayout->addWidget(selectionGroup);
    
    // Rendering Properties Group
    QGroupBox *renderGroup = new QGroupBox(tr("Rendering Properties"), this);
    QFormLayout *renderLayout = new QFormLayout(renderGroup);
    
    m_shadingCheckBox = new QCheckBox(tr("Enable Shading"), this);
    renderLayout->addRow(m_shadingCheckBox);
    
    m_ambientSpinBox = new QDoubleSpinBox(this);
    m_ambientSpinBox->setRange(0.0, 1.0);
    m_ambientSpinBox->setSingleStep(0.01);
    m_ambientSpinBox->setDecimals(3);
    renderLayout->addRow(tr("Ambient:"), m_ambientSpinBox);
    
    m_diffuseSpinBox = new QDoubleSpinBox(this);
    m_diffuseSpinBox->setRange(0.0, 1.0);
    m_diffuseSpinBox->setSingleStep(0.01);
    m_diffuseSpinBox->setDecimals(3);
    renderLayout->addRow(tr("Diffuse:"), m_diffuseSpinBox);
    
    m_specularSpinBox = new QDoubleSpinBox(this);
    m_specularSpinBox->setRange(0.0, 1.0);
    m_specularSpinBox->setSingleStep(0.01);
    m_specularSpinBox->setDecimals(3);
    renderLayout->addRow(tr("Specular:"), m_specularSpinBox);
    
    m_specularPowerSpinBox = new QDoubleSpinBox(this);
    m_specularPowerSpinBox->setRange(0.0, 128.0);
    m_specularPowerSpinBox->setSingleStep(1.0);
    m_specularPowerSpinBox->setDecimals(1);
    renderLayout->addRow(tr("Specular Power:"), m_specularPowerSpinBox);
    
    mainLayout->addWidget(renderGroup);
    
    // Advanced Properties Group
    QGroupBox *advancedGroup = new QGroupBox(tr("Advanced Properties"), this);
    QFormLayout *advancedLayout = new QFormLayout(advancedGroup);
    
    m_scalarOpacityUnitDistanceSpinBox = new QDoubleSpinBox(this);
    m_scalarOpacityUnitDistanceSpinBox->setRange(0.001, 100.0);
    m_scalarOpacityUnitDistanceSpinBox->setSingleStep(0.1);
    m_scalarOpacityUnitDistanceSpinBox->setDecimals(3);
    m_scalarOpacityUnitDistanceSpinBox->setToolTip(tr("Distance over which opacity is evaluated"));
    advancedLayout->addRow(tr("Scalar Opacity Unit Distance:"), m_scalarOpacityUnitDistanceSpinBox);
    
    m_sampleDistanceSpinBox = new QDoubleSpinBox(this);
    m_sampleDistanceSpinBox->setRange(0.001, 10.0);
    m_sampleDistanceSpinBox->setSingleStep(0.01);
    m_sampleDistanceSpinBox->setDecimals(3);
    m_sampleDistanceSpinBox->setToolTip(tr("Distance between samples during ray casting"));
    advancedLayout->addRow(tr("Sample Distance:"), m_sampleDistanceSpinBox);
    
    m_autoAdjustSampleDistancesCheckBox = new QCheckBox(tr("Auto-adjust Sample Distances"), this);
    m_autoAdjustSampleDistancesCheckBox->setToolTip(tr("Automatically adjust sample distances based on volume size"));
    advancedLayout->addRow(m_autoAdjustSampleDistancesCheckBox);
    
    mainLayout->addWidget(advancedGroup);
    
    setLayout(mainLayout);
}

void VolumeDialog::connectSignals()
{
    connect(m_volumeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &VolumeDialog::onVolumeSelected);
    
    // Rendering property signals
    connect(m_shadingCheckBox, &QCheckBox::toggled,
            this, &VolumeDialog::onRenderingPropertyChanged);
    connect(m_ambientSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &VolumeDialog::onRenderingPropertyChanged);
    connect(m_diffuseSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &VolumeDialog::onRenderingPropertyChanged);
    connect(m_specularSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &VolumeDialog::onRenderingPropertyChanged);
    connect(m_specularPowerSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &VolumeDialog::onRenderingPropertyChanged);
    connect(m_scalarOpacityUnitDistanceSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &VolumeDialog::onRenderingPropertyChanged);
    connect(m_sampleDistanceSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &VolumeDialog::onRenderingPropertyChanged);
    connect(m_autoAdjustSampleDistancesCheckBox, &QCheckBox::toggled,
            this, &VolumeDialog::onRenderingPropertyChanged);
}

void VolumeDialog::populateVolumeList()
{
    m_volumeComboBox->clear();
    m_volumePaths.clear();
    
    if (!m_sceneGraph) return;
    
    // Get all volume nodes recursively
    auto allVolumes = m_sceneGraph->getAllVolumeGraphics();
    
    for (const auto& volumeNode : allVolumes) {
        if (volumeNode) {
            // Use full state tree path for uniqueness
            std::string fullPath = volumeNode->getState().fullName();
            m_volumePaths.push_back(fullPath);
            
            // Display the node name in the combo box
            m_volumeComboBox->addItem(QString::fromStdString(volumeNode->getName()));
        }
    }
    
    if (m_volumeComboBox->count() == 0) {
        setPropertiesEnabled(false);
    } else {
        setPropertiesEnabled(true);
        onVolumeSelected(0);
    }
}

void VolumeDialog::onGraphicsChildrenChanged()
{
    if (!m_sceneGraph) return;
    
    // Get current volume count
    size_t currentCount = m_volumePaths.size();
    
    // Count volume nodes in scene graph recursively
    size_t sceneVolumeCount = m_sceneGraph->getVolumeGraphicsCount();
    
    // If counts differ, refresh the list
    if (sceneVolumeCount != currentCount) {
        // Save current selection (by path)
        QString currentSelection;
        int currentIndex = m_volumeComboBox->currentIndex();
        if (currentIndex >= 0 && currentIndex < static_cast<int>(m_volumePaths.size())) {
            currentSelection = QString::fromStdString(m_volumePaths[currentIndex]);
        }
        
        // Refresh the list
        populateVolumeList();
        
        // Try to restore the previous selection by matching path
        if (!currentSelection.isEmpty()) {
            for (int i = 0; i < static_cast<int>(m_volumePaths.size()); ++i) {
                if (QString::fromStdString(m_volumePaths[i]) == currentSelection) {
                    m_volumeComboBox->setCurrentIndex(i);
                    break;
                }
            }
        }
    }
}

void VolumeDialog::onVolumeSelected(int index)
{
    if (m_updating) return;
    
    if (index < 0 || index >= static_cast<int>(m_volumePaths.size())) {
        setPropertiesEnabled(false);
        return;
    }
    
    setPropertiesEnabled(true);
    updatePropertiesFromNode();
}

void VolumeDialog::updatePropertiesFromNode()
{
    if (m_updating) return;
    
    int index = m_volumeComboBox->currentIndex();
    if (index < 0 || index >= static_cast<int>(m_volumePaths.size())) return;
    
    const std::string& volumePath = m_volumePaths[index];
    
    // Get the volume node
    auto allVolumes = m_sceneGraph->getAllVolumeGraphics();
    for (const auto& volumeNode : allVolumes) {
        if (volumeNode && volumeNode->getState().fullName() == volumePath) {
            m_updating = true;
            
            // Update rendering properties
            m_shadingCheckBox->setChecked(volumeNode->getShading());
            m_ambientSpinBox->setValue(volumeNode->getAmbient());
            m_diffuseSpinBox->setValue(volumeNode->getDiffuse());
            m_specularSpinBox->setValue(volumeNode->getSpecular());
            m_specularPowerSpinBox->setValue(volumeNode->getSpecularPower());
            m_scalarOpacityUnitDistanceSpinBox->setValue(volumeNode->getScalarOpacityUnitDistance());
            m_sampleDistanceSpinBox->setValue(volumeNode->getSampleDistance());
            m_autoAdjustSampleDistancesCheckBox->setChecked(volumeNode->getAutoAdjustSampleDistances());
            
            m_updating = false;
            break;
        }
    }
}

void VolumeDialog::onRenderingPropertyChanged()
{
    if (m_updating) return;
    
    int index = m_volumeComboBox->currentIndex();
    if (index < 0 || index >= static_cast<int>(m_volumePaths.size())) return;
    
    const std::string& volumePath = m_volumePaths[index];
    
    // Get the volume node
    auto allVolumes = m_sceneGraph->getAllVolumeGraphics();
    for (const auto& volumeNode : allVolumes) {
        if (volumeNode && volumeNode->getState().fullName() == volumePath) {
            // Determine which property changed and update it
            QObject* sender = QObject::sender();
            
            if (sender == m_shadingCheckBox) {
                volumeNode->setShading(m_shadingCheckBox->isChecked());
            } else if (sender == m_ambientSpinBox) {
                volumeNode->setAmbient(m_ambientSpinBox->value());
            } else if (sender == m_diffuseSpinBox) {
                volumeNode->setDiffuse(m_diffuseSpinBox->value());
            } else if (sender == m_specularSpinBox) {
                volumeNode->setSpecular(m_specularSpinBox->value());
            } else if (sender == m_specularPowerSpinBox) {
                volumeNode->setSpecularPower(m_specularPowerSpinBox->value());
            } else if (sender == m_scalarOpacityUnitDistanceSpinBox) {
                volumeNode->setScalarOpacityUnitDistance(m_scalarOpacityUnitDistanceSpinBox->value());
            } else if (sender == m_sampleDistanceSpinBox) {
                volumeNode->setSampleDistance(m_sampleDistanceSpinBox->value());
            } else if (sender == m_autoAdjustSampleDistancesCheckBox) {
                volumeNode->setAutoAdjustSampleDistances(m_autoAdjustSampleDistancesCheckBox->isChecked());
            }
            
            break;
        }
    }
}

void VolumeDialog::setPropertiesEnabled(bool enabled)
{
    m_shadingCheckBox->setEnabled(enabled);
    m_ambientSpinBox->setEnabled(enabled);
    m_diffuseSpinBox->setEnabled(enabled);
    m_specularSpinBox->setEnabled(enabled);
    m_specularPowerSpinBox->setEnabled(enabled);
    m_scalarOpacityUnitDistanceSpinBox->setEnabled(enabled);
    m_sampleDistanceSpinBox->setEnabled(enabled);
    m_autoAdjustSampleDistancesCheckBox->setEnabled(enabled);
}
