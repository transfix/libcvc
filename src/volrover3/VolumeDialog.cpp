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
#include <QPushButton>
#include <QMessageBox>
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
    
    // Connect to SceneGraph signal to monitor for new/removed volumes
    if (m_sceneGraph) {
        m_graphicsChangedConnection = m_sceneGraph->graphicsChanged.connect(
            [this]() {
                QMetaObject::invokeMethod(this, "onGraphicsChildrenChanged", Qt::QueuedConnection);
            }
        );
    }
}

void VolumeDialog::setupUI()
{
    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    
    // Volume Selection Group
    QGroupBox *selectionGroup = new QGroupBox(tr("Volume Selection"), this);
    QVBoxLayout *selectionVLayout = new QVBoxLayout(selectionGroup);
    
    // Combo box and delete button in horizontal layout
    QHBoxLayout *comboLayout = new QHBoxLayout();
    m_volumeComboBox = new QComboBox(this);
    m_deleteButton = new QPushButton(tr("Delete"), this);
    m_deleteButton->setToolTip(tr("Remove selected volume from scene"));
    comboLayout->addWidget(new QLabel(tr("Volume:"), this));
    comboLayout->addWidget(m_volumeComboBox, 1);
    comboLayout->addWidget(m_deleteButton);
    selectionVLayout->addLayout(comboLayout);
    
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
    connect(m_deleteButton, &QPushButton::clicked,
            this, &VolumeDialog::onDeleteButtonClicked);
    
    // Rendering property signals
    connect(m_shadingCheckBox, &QCheckBox::toggled,
            this, &VolumeDialog::onMaterialPropertyChanged);
    connect(m_ambientSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &VolumeDialog::onMaterialPropertyChanged);
    connect(m_diffuseSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &VolumeDialog::onMaterialPropertyChanged);
    connect(m_specularSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &VolumeDialog::onMaterialPropertyChanged);
    connect(m_specularPowerSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &VolumeDialog::onMaterialPropertyChanged);
    connect(m_scalarOpacityUnitDistanceSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &VolumeDialog::onMaterialPropertyChanged);
    connect(m_sampleDistanceSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &VolumeDialog::onMaterialPropertyChanged);
    connect(m_autoAdjustSampleDistancesCheckBox, &QCheckBox::toggled,
            this, &VolumeDialog::onMaterialPropertyChanged);
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
    
    // Save current selection (by path)
    QString currentSelection;
    int currentIndex = m_volumeComboBox->currentIndex();
    if (currentIndex >= 0 && currentIndex < static_cast<int>(m_volumePaths.size())) {
        currentSelection = QString::fromStdString(m_volumePaths[currentIndex]);
    }
    
    // Refresh the list
    populateVolumeList();
    
    // Try to restore the previous selection by matching path
    bool selectionRestored = false;
    if (!currentSelection.isEmpty()) {
        for (int i = 0; i < static_cast<int>(m_volumePaths.size()); ++i) {
            if (QString::fromStdString(m_volumePaths[i]) == currentSelection) {
                m_volumeComboBox->setCurrentIndex(i);
                selectionRestored = true;
                break;
            }
        }
    }
    
    // If selection couldn't be restored (volume was deleted), disable controls if no volumes
    if (!selectionRestored && m_volumeComboBox->count() == 0) {
        setPropertiesEnabled(false);
    }
}

void VolumeDialog::onVolumeSelected(int index)
{
    if (m_updating) return;
    
    // Disconnect from previous node's state changes
    m_nodeStateConnection.disconnect();
    
    if (index < 0 || index >= static_cast<int>(m_volumePaths.size())) {
        setPropertiesEnabled(false);
        return;
    }
    
    // Connect to selected node's state changes
    const std::string& volumePath = m_volumePaths[index];
    auto allVolumes = m_sceneGraph->getAllVolumeGraphics();
    for (const auto& volumeNode : allVolumes) {
        if (volumeNode && volumeNode->getState().fullName() == volumePath) {
            // Connect to the node's childChanged signal (fires when any child state changes)
            m_nodeStateConnection = volumeNode->getState().childChanged.connect(
                [this](const std::string&) {
                    // Use Qt's queued connection for thread-safe UI updates
                    QMetaObject::invokeMethod(this, "onNodeStateChanged", Qt::QueuedConnection);
                }
            );
            break;
        }
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

void VolumeDialog::onNodeStateChanged()
{
    // Update UI from state tree when node state changes
    updatePropertiesFromNode();
}

void VolumeDialog::onMaterialPropertyChanged()
{
    if (m_updating) return;
    
    int index = m_volumeComboBox->currentIndex();
    if (index < 0 || index >= static_cast<int>(m_volumePaths.size())) {
        setPropertiesEnabled(false);
        return;
    }
    
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

void VolumeDialog::onDeleteButtonClicked()
{
    if (!m_sceneGraph) return;
    
    int currentIndex = m_volumeComboBox->currentIndex();
    if (currentIndex < 0 || currentIndex >= static_cast<int>(m_volumePaths.size())) {
        return;
    }
    
    // Extract the volume name from the full path (last component after the last dot)
    std::string volumePath = m_volumePaths[currentIndex];
    size_t lastDot = volumePath.find_last_of('.');
    std::string volumeName = (lastDot != std::string::npos) ? 
                             volumePath.substr(lastDot + 1) : volumePath;
    
    // Confirm deletion
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, tr("Delete Volume"),
                                   tr("Are you sure you want to delete '%1'?")
                                       .arg(QString::fromStdString(volumeName)),
                                   QMessageBox::Yes | QMessageBox::No);
    
    if (reply == QMessageBox::Yes) {
        m_sceneGraph->removeGraphics(volumeName);
        // The combo box will update automatically via the state tree signal
    }
}

void VolumeDialog::setPropertiesEnabled(bool enabled)
{
    m_deleteButton->setEnabled(enabled);
    m_shadingCheckBox->setEnabled(enabled);
    m_ambientSpinBox->setEnabled(enabled);
    m_diffuseSpinBox->setEnabled(enabled);
    m_specularSpinBox->setEnabled(enabled);
    m_specularPowerSpinBox->setEnabled(enabled);
    m_scalarOpacityUnitDistanceSpinBox->setEnabled(enabled);
    m_sampleDistanceSpinBox->setEnabled(enabled);
    m_autoAdjustSampleDistancesCheckBox->setEnabled(enabled);
}
