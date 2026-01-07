#include <volrover3/GeometryDialog.h>
#include <volrover3/SceneGraph.h>
#include <volrover3/GraphicsNode.h>
#include <volrover3/GeometryNode.h>
#include <cvc/geometry.h>
#include <cvc/state.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QMessageBox>
#include <QTabWidget>
#include <QMetaObject>

GeometryDialog::GeometryDialog(std::shared_ptr<SceneGraph> sceneGraph, QWidget *parent)
    : QDialog(parent)
    , m_sceneGraph(sceneGraph)
    , m_geometryComboBox(nullptr)
    , m_renderModeComboBox(nullptr)
    , m_colorRSpinBox(nullptr)
    , m_colorGSpinBox(nullptr)
    , m_colorBSpinBox(nullptr)
    , m_ambientSpinBox(nullptr)
    , m_diffuseSpinBox(nullptr)
    , m_specularSpinBox(nullptr)
    , m_specularPowerSpinBox(nullptr)
    , m_opacitySpinBox(nullptr)
    , m_pointSizeSpinBox(nullptr)
    , m_lineWidthSpinBox(nullptr)
    , m_updating(false)
{
    setWindowTitle(tr("Geometry Properties"));
    setMinimumWidth(400);
    setupUI();
    connectSignals();
    populateGeometryList();
    
    // Connect to state tree to monitor for new/removed geometry
    if (m_sceneGraph) {
        std::string statePrefix = m_sceneGraph->getStatePrefix();
        std::string graphicsChildrenPath = statePrefix + ".graphics.root.children";
        
        try {
            // Access the state to ensure it exists (this will create it if needed)
            auto& childrenState = cvc::state::instance()(graphicsChildrenPath);
            
            // Touch it once to ensure the state tree entry is fully initialized
            // This is important for the dataChanged signal to work properly
            childrenState.touch();
            
            // Listen to dataChanged which fires when touch() is called
            // (i.e., when the collection structure changes via add/remove)
            // Use AutoConnection so Qt determines the best way to invoke (direct or queued)
            m_graphicsChildrenConnection = childrenState.dataChanged.connect(
                [this]() {
                    QMetaObject::invokeMethod(this, "onGraphicsChildrenChanged", Qt::AutoConnection);
                }
            );
        } catch (const std::exception& e) {
            cvcapp.log(0, std::string("GeometryDialog: Failed to connect to state tree: ") + e.what());
        } catch (...) {
            cvcapp.log(0, "GeometryDialog: Failed to connect to state tree (unknown error)");
        }
    }
}

void GeometryDialog::setupUI()
{
    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    
    // Geometry Selection Group (always visible at top)
    QGroupBox *selectionGroup = new QGroupBox(tr("Geometry Selection"), this);
    QVBoxLayout *selectionVLayout = new QVBoxLayout(selectionGroup);
    
    // Combo box and delete button in horizontal layout
    QHBoxLayout *comboLayout = new QHBoxLayout();
    m_geometryComboBox = new QComboBox(this);
    m_geometryComboBox->setObjectName("geometryComboBox");
    m_deleteButton = new QPushButton(tr("Delete"), this);
    m_deleteButton->setToolTip(tr("Remove selected geometry from scene"));
    comboLayout->addWidget(new QLabel(tr("Geometry:"), this));
    comboLayout->addWidget(m_geometryComboBox, 1);
    comboLayout->addWidget(m_deleteButton);
    selectionVLayout->addLayout(comboLayout);
    
    mainLayout->addWidget(selectionGroup);
    
    // Create tab widget for geometry properties
    QTabWidget *tabWidget = new QTabWidget(this);
    
    // === Appearance Tab ===
    QWidget *appearanceTab = new QWidget();
    QVBoxLayout *appearanceLayout = new QVBoxLayout(appearanceTab);
    
    // Render Mode Group
    QGroupBox *renderGroup = new QGroupBox(tr("Render Mode"), appearanceTab);
    QFormLayout *renderLayout = new QFormLayout(renderGroup);
    
    m_renderModeComboBox = new QComboBox(this);
    m_renderModeComboBox->setObjectName("renderModeComboBox");
    m_renderModeComboBox->addItem(tr("Surface (Triangles)"), static_cast<int>(GeometryRenderMode::TRIS));
    m_renderModeComboBox->addItem(tr("Surface (Quads)"), static_cast<int>(GeometryRenderMode::QUADS));
    m_renderModeComboBox->addItem(tr("Wireframe"), static_cast<int>(GeometryRenderMode::LINES));
    m_renderModeComboBox->addItem(tr("Points"), static_cast<int>(GeometryRenderMode::POINTS));
    renderLayout->addRow(tr("Mode:"), m_renderModeComboBox);
    
    appearanceLayout->addWidget(renderGroup);
    
    // Color Group
    QGroupBox *colorGroup = new QGroupBox(tr("Color"), appearanceTab);
    QFormLayout *colorLayout = new QFormLayout(colorGroup);
    
    m_colorRSpinBox = new QDoubleSpinBox(this);
    m_colorRSpinBox->setObjectName("colorRSpinBox");
    m_colorRSpinBox->setRange(0.0, 1.0);
    m_colorRSpinBox->setSingleStep(0.01);
    m_colorRSpinBox->setDecimals(3);
    colorLayout->addRow(tr("Red:"), m_colorRSpinBox);
    
    m_colorGSpinBox = new QDoubleSpinBox(this);
    m_colorGSpinBox->setObjectName("colorGSpinBox");
    m_colorGSpinBox->setRange(0.0, 1.0);
    m_colorGSpinBox->setSingleStep(0.01);
    m_colorGSpinBox->setDecimals(3);
    colorLayout->addRow(tr("Green:"), m_colorGSpinBox);
    
    m_colorBSpinBox = new QDoubleSpinBox(this);
    m_colorBSpinBox->setObjectName("colorBSpinBox");
    m_colorBSpinBox->setRange(0.0, 1.0);
    m_colorBSpinBox->setSingleStep(0.01);
    m_colorBSpinBox->setDecimals(3);
    colorLayout->addRow(tr("Blue:"), m_colorBSpinBox);
    
    appearanceLayout->addWidget(colorGroup);
    
    // Opacity in appearance tab
    QGroupBox *opacityGroup = new QGroupBox(tr("Transparency"), appearanceTab);
    QFormLayout *opacityLayout = new QFormLayout(opacityGroup);
    
    m_opacitySpinBox = new QDoubleSpinBox(this);
    m_opacitySpinBox->setRange(0.0, 1.0);
    m_opacitySpinBox->setSingleStep(0.01);
    m_opacitySpinBox->setDecimals(3);
    opacityLayout->addRow(tr("Opacity:"), m_opacitySpinBox);
    
    appearanceLayout->addWidget(opacityGroup);
    appearanceLayout->addStretch();
    
    // === Material Tab ===
    QWidget *materialTab = new QWidget();
    QVBoxLayout *materialLayout = new QVBoxLayout(materialTab);
    
    QGroupBox *materialGroup = new QGroupBox(tr("Material Properties"), materialTab);
    QFormLayout *matLayout = new QFormLayout(materialGroup);
    
    m_ambientSpinBox = new QDoubleSpinBox(this);
    m_ambientSpinBox->setRange(0.0, 1.0);
    m_ambientSpinBox->setSingleStep(0.01);
    m_ambientSpinBox->setDecimals(3);
    matLayout->addRow(tr("Ambient:"), m_ambientSpinBox);
    
    m_diffuseSpinBox = new QDoubleSpinBox(this);
    m_diffuseSpinBox->setRange(0.0, 1.0);
    m_diffuseSpinBox->setSingleStep(0.01);
    m_diffuseSpinBox->setDecimals(3);
    matLayout->addRow(tr("Diffuse:"), m_diffuseSpinBox);
    
    m_specularSpinBox = new QDoubleSpinBox(this);
    m_specularSpinBox->setRange(0.0, 1.0);
    m_specularSpinBox->setSingleStep(0.01);
    m_specularSpinBox->setDecimals(3);
    matLayout->addRow(tr("Specular:"), m_specularSpinBox);
    
    m_specularPowerSpinBox = new QDoubleSpinBox(this);
    m_specularPowerSpinBox->setRange(0.0, 128.0);
    m_specularPowerSpinBox->setSingleStep(1.0);
    m_specularPowerSpinBox->setDecimals(1);
    matLayout->addRow(tr("Specular Power:"), m_specularPowerSpinBox);
    
    materialLayout->addWidget(materialGroup);
    materialLayout->addStretch();
    
    // === Rendering Tab ===
    QWidget *renderingTab = new QWidget();
    QVBoxLayout *renderingLayout = new QVBoxLayout(renderingTab);
    
    QGroupBox *sizeGroup = new QGroupBox(tr("Point and Line Properties"), renderingTab);
    QFormLayout *sizeLayout = new QFormLayout(sizeGroup);
    
    m_pointSizeSpinBox = new QDoubleSpinBox(this);
    m_pointSizeSpinBox->setRange(0.1, 50.0);
    m_pointSizeSpinBox->setSingleStep(0.5);
    m_pointSizeSpinBox->setDecimals(1);
    sizeLayout->addRow(tr("Point Size:"), m_pointSizeSpinBox);
    
    m_lineWidthSpinBox = new QDoubleSpinBox(this);
    m_lineWidthSpinBox->setRange(0.1, 50.0);
    m_lineWidthSpinBox->setSingleStep(0.5);
    m_lineWidthSpinBox->setDecimals(1);
    sizeLayout->addRow(tr("Line Width:"), m_lineWidthSpinBox);
    
    renderingLayout->addWidget(sizeGroup);
    renderingLayout->addStretch();
    
    // Add tabs to tab widget
    tabWidget->addTab(appearanceTab, tr("Appearance"));
    tabWidget->addTab(materialTab, tr("Material"));
    tabWidget->addTab(renderingTab, tr("Rendering"));
    
    mainLayout->addWidget(tabWidget);
    
    setLayout(mainLayout);
}

void GeometryDialog::connectSignals()
{
    connect(m_geometryComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &GeometryDialog::onGeometrySelected);
    connect(m_deleteButton, &QPushButton::clicked,
            this, &GeometryDialog::onDeleteButtonClicked);
    connect(m_renderModeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &GeometryDialog::onRenderModeChanged);
    
    // Color signals
    connect(m_colorRSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &GeometryDialog::onColorChanged);
    connect(m_colorGSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &GeometryDialog::onColorChanged);
    connect(m_colorBSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &GeometryDialog::onColorChanged);
    
    // Material property signals
    connect(m_ambientSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &GeometryDialog::onMaterialPropertyChanged);
    connect(m_diffuseSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &GeometryDialog::onMaterialPropertyChanged);
    connect(m_specularSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &GeometryDialog::onMaterialPropertyChanged);
    connect(m_specularPowerSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &GeometryDialog::onMaterialPropertyChanged);
    connect(m_opacitySpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &GeometryDialog::onMaterialPropertyChanged);
    connect(m_pointSizeSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &GeometryDialog::onMaterialPropertyChanged);
    connect(m_lineWidthSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &GeometryDialog::onMaterialPropertyChanged);
}

void GeometryDialog::populateGeometryList()
{
    m_geometryComboBox->clear();
    m_geometryNames.clear();
    
    if (!m_sceneGraph) return;
    
    // Get all geometry nodes recursively
    auto allGeometries = m_sceneGraph->getAllGeometryGraphics();
    
    for (const auto& geomNode : allGeometries) {
        if (geomNode && geomNode->getGeometry() && !geomNode->getGeometry()->empty()) {
            std::string name = geomNode->getName();
            m_geometryNames.push_back(name);
            m_geometryComboBox->addItem(QString::fromStdString(name));
        }
    }
    
    if (m_geometryComboBox->count() == 0) {
        setPropertiesEnabled(false);
    } else {
        setPropertiesEnabled(true);
        onGeometrySelected(0);
    }
}

void GeometryDialog::onGraphicsChildrenChanged()
{
    if (!m_sceneGraph) return;
    
    // Get current geometry count
    size_t currentCount = m_geometryNames.size();
    
    // Count geometry nodes in scene graph recursively
    size_t sceneGeomCount = 0;
    auto allGeometries = m_sceneGraph->getAllGeometryGraphics();
    for (const auto& geomNode : allGeometries) {
        if (geomNode && geomNode->getGeometry() && !geomNode->getGeometry()->empty()) {
            sceneGeomCount++;
        }
    }
    
    // If counts differ, refresh the list
    if (sceneGeomCount != currentCount) {
        // Save current selection
        QString currentSelection;
        int currentIndex = m_geometryComboBox->currentIndex();
        if (currentIndex >= 0 && currentIndex < static_cast<int>(m_geometryNames.size())) {
            currentSelection = QString::fromStdString(m_geometryNames[currentIndex]);
        }
        
        // Refresh the list
        populateGeometryList();
        
        // Try to restore the previous selection
        if (!currentSelection.isEmpty()) {
            int newIndex = m_geometryComboBox->findText(currentSelection);
            if (newIndex >= 0) {
                m_geometryComboBox->setCurrentIndex(newIndex);
            }
        }
    }
}

void GeometryDialog::onGeometrySelected(int index)
{
    if (m_updating) return;
    
    // Disconnect from previous node's state changes
    m_nodeStateConnection.disconnect();
    
    if (index < 0 || index >= static_cast<int>(m_geometryNames.size())) {
        setPropertiesEnabled(false);
        return;
    }
    
    // Connect to selected node's state changes
    const std::string& geomName = m_geometryNames[index];
    auto graphicsNode = m_sceneGraph->getGraphics(geomName);
    auto geomNode = std::dynamic_pointer_cast<GeometryNode>(graphicsNode);
    
    if (geomNode) {
        // Connect to the node's childChanged signal (fires when any child state changes)
        // Use AutoConnection so Qt determines the best way to invoke (direct or queued)
        m_nodeStateConnection = geomNode->getState().childChanged.connect(
            [this](const std::string&) {
                QMetaObject::invokeMethod(this, "onNodeStateChanged", Qt::AutoConnection);
            }
        );
    }
    
    setPropertiesEnabled(true);
    updatePropertiesFromNode();
}

void GeometryDialog::updatePropertiesFromNode()
{
    if (m_updating) return;
    
    int index = m_geometryComboBox->currentIndex();
    if (index < 0 || index >= static_cast<int>(m_geometryNames.size())) return;
    
    const std::string& geomName = m_geometryNames[index];
    auto graphicsNode = m_sceneGraph->getGraphics(geomName);
    auto geomNode = std::dynamic_pointer_cast<GeometryNode>(graphicsNode);
    
    if (!geomNode) return;
    
    m_updating = true;
    
    // Update render mode
    GeometryRenderMode mode = geomNode->getRenderMode();
    int modeIndex = m_renderModeComboBox->findData(static_cast<int>(mode));
    if (modeIndex >= 0) {
        m_renderModeComboBox->setCurrentIndex(modeIndex);
    }
    
    // Update color from state tree directly
    try {
        m_colorRSpinBox->setValue(geomNode->getState("color_r").value<double>());
        m_colorGSpinBox->setValue(geomNode->getState("color_g").value<double>());
        m_colorBSpinBox->setValue(geomNode->getState("color_b").value<double>());
    } catch (const std::exception&) {
        // Use defaults if state not available
    } catch (...) {
        // Catch all other exceptions
    }
    
    // Update material properties from state tree
    try { m_ambientSpinBox->setValue(geomNode->getState("ambient").value<double>()); } catch (const std::exception&) {} catch (...) {}
    try { m_diffuseSpinBox->setValue(geomNode->getState("diffuse").value<double>()); } catch (const std::exception&) {} catch (...) {}
    try { m_specularSpinBox->setValue(geomNode->getState("specular").value<double>()); } catch (const std::exception&) {} catch (...) {}
    try { m_specularPowerSpinBox->setValue(geomNode->getState("specular_power").value<double>()); } catch (const std::exception&) {} catch (...) {}
    try { m_opacitySpinBox->setValue(geomNode->getState("opacity").value<double>()); } catch (const std::exception&) {} catch (...) {}
    try { m_pointSizeSpinBox->setValue(geomNode->getState("point_size").value<double>()); } catch (const std::exception&) {} catch (...) {}
    try { m_lineWidthSpinBox->setValue(geomNode->getState("line_width").value<double>()); } catch (const std::exception&) {} catch (...) {}
    
    m_updating = false;
}

void GeometryDialog::onNodeStateChanged()
{
    // Update UI from state tree when node state changes
    updatePropertiesFromNode();
}

void GeometryDialog::onRenderModeChanged(int index)
{
    if (m_updating) return;
    
    int geomIndex = m_geometryComboBox->currentIndex();
    if (geomIndex < 0 || geomIndex >= static_cast<int>(m_geometryNames.size())) return;
    
    const std::string& geomName = m_geometryNames[geomIndex];
    auto graphicsNode = m_sceneGraph->getGraphics(geomName);
    auto geomNode = std::dynamic_pointer_cast<GeometryNode>(graphicsNode);
    
    if (!geomNode) return;
    
    GeometryRenderMode mode = static_cast<GeometryRenderMode>(
        m_renderModeComboBox->currentData().toInt());
    geomNode->setRenderMode(mode);
}

void GeometryDialog::onColorChanged()
{
    if (m_updating) return;
    
    int index = m_geometryComboBox->currentIndex();
    if (index < 0 || index >= static_cast<int>(m_geometryNames.size())) return;
    
    const std::string& geomName = m_geometryNames[index];
    auto graphicsNode = m_sceneGraph->getGraphics(geomName);
    auto geomNode = std::dynamic_pointer_cast<GeometryNode>(graphicsNode);
    
    if (!geomNode) return;
    
    geomNode->setColor(m_colorRSpinBox->value(),
                      m_colorGSpinBox->value(),
                      m_colorBSpinBox->value());
}

void GeometryDialog::onMaterialPropertyChanged()
{
    if (m_updating) return;
    
    int index = m_geometryComboBox->currentIndex();
    if (index < 0 || index >= static_cast<int>(m_geometryNames.size())) return;
    
    const std::string& geomName = m_geometryNames[index];
    auto graphicsNode = m_sceneGraph->getGraphics(geomName);
    auto geomNode = std::dynamic_pointer_cast<GeometryNode>(graphicsNode);
    
    if (!geomNode) return;
    
    // Determine which property changed and update it
    QObject* sender = QObject::sender();
    
    if (sender == m_ambientSpinBox) {
        geomNode->setAmbient(m_ambientSpinBox->value());
    } else if (sender == m_diffuseSpinBox) {
        geomNode->setDiffuse(m_diffuseSpinBox->value());
    } else if (sender == m_specularSpinBox) {
        geomNode->setSpecular(m_specularSpinBox->value());
    } else if (sender == m_specularPowerSpinBox) {
        geomNode->setSpecularPower(m_specularPowerSpinBox->value());
    } else if (sender == m_opacitySpinBox) {
        geomNode->setOpacity(m_opacitySpinBox->value());
    } else if (sender == m_pointSizeSpinBox) {
        geomNode->setPointSize(m_pointSizeSpinBox->value());
    } else if (sender == m_lineWidthSpinBox) {
        geomNode->setLineWidth(m_lineWidthSpinBox->value());
    }
}

void GeometryDialog::onDeleteButtonClicked()
{
    if (!m_sceneGraph) return;
    
    int currentIndex = m_geometryComboBox->currentIndex();
    if (currentIndex < 0 || currentIndex >= static_cast<int>(m_geometryNames.size())) {
        return;
    }
    
    std::string geometryName = m_geometryNames[currentIndex];
    
    // Confirm deletion
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, tr("Delete Geometry"),
                                   tr("Are you sure you want to delete '%1'?")
                                       .arg(QString::fromStdString(geometryName)),
                                   QMessageBox::Yes | QMessageBox::No);
    
    if (reply == QMessageBox::Yes) {
        m_sceneGraph->removeGraphics(geometryName);
        // The combo box will update automatically via the state tree signal
    }
}

void GeometryDialog::setPropertiesEnabled(bool enabled)
{
    m_deleteButton->setEnabled(enabled);
    m_renderModeComboBox->setEnabled(enabled);
    m_colorRSpinBox->setEnabled(enabled);
    m_colorGSpinBox->setEnabled(enabled);
    m_colorBSpinBox->setEnabled(enabled);
    m_ambientSpinBox->setEnabled(enabled);
    m_diffuseSpinBox->setEnabled(enabled);
    m_specularSpinBox->setEnabled(enabled);
    m_specularPowerSpinBox->setEnabled(enabled);
    m_opacitySpinBox->setEnabled(enabled);
    m_pointSizeSpinBox->setEnabled(enabled);
    m_lineWidthSpinBox->setEnabled(enabled);
}
