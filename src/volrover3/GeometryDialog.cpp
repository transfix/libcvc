#include <volrover3/GeometryDialog.h>
#include <volrover3/SceneGraph.h>
#include <volrover3/GraphicsNode.h>
#include <volrover3/GeometryNode.h>
#include <cvc/geometry.h>
#include <cvc/algorithm.h>
#include <cvc/state.h>
#include <cvc/app.h>
#include <cvc/types.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QCheckBox>
#include <QLabel>
#include <QPushButton>
#include <QMessageBox>
#include <QTabWidget>
#include <QTableWidget>
#include <QHeaderView>
#include <QMetaObject>
#include <QColorDialog>

GeometryDialog::GeometryDialog(std::shared_ptr<SceneGraph> sceneGraph, QWidget *parent)
    : QDialog(parent)
    , m_sceneGraph(sceneGraph)
    , m_geometryComboBox(nullptr)
    , m_renderModeComboBox(nullptr)
    , m_singleColorCheckBox(nullptr)
    , m_colorRSpinBox(nullptr)
    , m_colorGSpinBox(nullptr)
    , m_colorBSpinBox(nullptr)
    , m_visibilityCheckBox(nullptr)
    , m_showBBoxCheckBox(nullptr)
    , m_bboxColorButton(nullptr)
    , m_invertNormalsButton(nullptr)
    , m_reorientButton(nullptr)
    , m_projectButton(nullptr)
    , m_projectTargetComboBox(nullptr)
    , m_smoothingButton(nullptr)
    , m_smoothingDeltaSpinBox(nullptr)
    , m_smoothingFixBoundaryCheckBox(nullptr)
    , m_smoothingPerturb1CheckBox(nullptr)
    , m_smoothingGeoFlowCheckBox(nullptr)
    , m_smoothingEnabledCheckBox(nullptr)
    , m_smoothingPerturb2CheckBox(nullptr)
    , m_qualityImproveButton(nullptr)
    , m_qualityIterationsSpinBox(nullptr)
    , m_qualityMethodComboBox(nullptr)
    , m_ambientSpinBox(nullptr)
    , m_diffuseSpinBox(nullptr)
    , m_specularSpinBox(nullptr)
    , m_specularPowerSpinBox(nullptr)
    , m_opacitySpinBox(nullptr)
    , m_pointSizeSpinBox(nullptr)
    , m_lineWidthSpinBox(nullptr)
    , m_infoTable(nullptr)
    , m_updating(false)
{
    m_bboxColor[0] = m_bboxColor[1] = m_bboxColor[2] = 1.0;  // Default white
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
    QVBoxLayout *colorVLayout = new QVBoxLayout(colorGroup);
    
    // Single color checkbox
    m_singleColorCheckBox = new QCheckBox(tr("Use single color (override vertex colors)"), this);
    m_singleColorCheckBox->setObjectName("singleColorCheckBox");
    m_singleColorCheckBox->setChecked(false);
    m_singleColorCheckBox->setToolTip(tr("When enabled, all vertices use the color specified below.\nWhen disabled, per-vertex colors from the geometry data are used if available."));
    colorVLayout->addWidget(m_singleColorCheckBox);
    
    // Color controls in form layout
    QFormLayout *colorLayout = new QFormLayout();
    
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
    
    colorVLayout->addLayout(colorLayout);
    
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
    
    // Visibility Group
    QGroupBox *visibilityGroup = new QGroupBox(tr("Visibility"), renderingTab);
    QVBoxLayout *visibilityLayout = new QVBoxLayout(visibilityGroup);
    
    m_visibilityCheckBox = new QCheckBox(tr("Visible"), this);
    m_visibilityCheckBox->setObjectName("visibilityCheckBox");
    m_visibilityCheckBox->setChecked(true);
    m_visibilityCheckBox->setToolTip(tr("Show or hide this geometry in the scene"));
    visibilityLayout->addWidget(m_visibilityCheckBox);
    
    renderingLayout->addWidget(visibilityGroup);
    
    // Bounding Box Group
    QGroupBox *bboxGroup = new QGroupBox(tr("Bounding Box"), renderingTab);
    QVBoxLayout *bboxLayout = new QVBoxLayout(bboxGroup);
    
    m_showBBoxCheckBox = new QCheckBox(tr("Show Bounding Box"), this);
    m_showBBoxCheckBox->setObjectName("showBBoxCheckBox");
    m_showBBoxCheckBox->setChecked(false);
    m_showBBoxCheckBox->setToolTip(tr("Display the bounding box of this geometry"));
    bboxLayout->addWidget(m_showBBoxCheckBox);
    
    QHBoxLayout *bboxColorLayout = new QHBoxLayout();
    bboxColorLayout->addWidget(new QLabel(tr("Color:"), this));
    m_bboxColorButton = new QPushButton(this);
    m_bboxColorButton->setFixedSize(50, 25);
    m_bboxColorButton->setToolTip(tr("Click to change bounding box color"));
    updateBBoxColorButton();
    bboxColorLayout->addWidget(m_bboxColorButton);
    bboxColorLayout->addStretch();
    bboxLayout->addLayout(bboxColorLayout);
    
    // Extent Labels
    m_showExtentLabelsCheckBox = new QCheckBox(tr("Show Extent Labels"), this);
    m_showExtentLabelsCheckBox->setObjectName("showExtentLabelsCheckBox");
    m_showExtentLabelsCheckBox->setChecked(false);
    m_showExtentLabelsCheckBox->setToolTip(tr("Display min/max coordinate labels on the bounding box"));
    bboxLayout->addWidget(m_showExtentLabelsCheckBox);
    
    QHBoxLayout *extentLabelColorLayout = new QHBoxLayout();
    extentLabelColorLayout->addWidget(new QLabel(tr("Label Color:"), this));
    m_extentLabelColorButton = new QPushButton(this);
    m_extentLabelColorButton->setFixedSize(50, 25);
    m_extentLabelColorButton->setToolTip(tr("Click to change extent label color"));
    // Initialize extent label color to white
    m_extentLabelColor[0] = m_extentLabelColor[1] = m_extentLabelColor[2] = 1.0;
    updateExtentLabelColorButton();
    extentLabelColorLayout->addWidget(m_extentLabelColorButton);
    extentLabelColorLayout->addStretch();
    bboxLayout->addLayout(extentLabelColorLayout);
    
    renderingLayout->addWidget(bboxGroup);
    renderingLayout->addStretch();
    
    // === Operations Tab ===
    QWidget *operationsTab = new QWidget();
    QVBoxLayout *operationsTabLayout = new QVBoxLayout(operationsTab);
    
    // Normals Group
    QGroupBox *normalsGroup = new QGroupBox(tr("Normals"), operationsTab);
    QVBoxLayout *normalsLayout = new QVBoxLayout(normalsGroup);
    
    // Invert Normals button
    m_invertNormalsButton = new QPushButton(tr("Invert Normals"), this);
    m_invertNormalsButton->setObjectName("invertNormalsButton");
    m_invertNormalsButton->setToolTip(tr("Invert all vertex and face normals of this geometry"));
    normalsLayout->addWidget(m_invertNormalsButton);
    
    // Reorient button
    m_reorientButton = new QPushButton(tr("Reorient"), this);
    m_reorientButton->setObjectName("reorientButton");
    m_reorientButton->setToolTip(tr("Make normals consistent across the mesh"));
    normalsLayout->addWidget(m_reorientButton);
    
    operationsTabLayout->addWidget(normalsGroup);
    
    // Project Group
    QGroupBox *projectGroup = new QGroupBox(tr("Projection"), operationsTab);
    QVBoxLayout *projectGroupLayout = new QVBoxLayout(projectGroup);
    
    QHBoxLayout *projectLayout = new QHBoxLayout();
    m_projectButton = new QPushButton(tr("Project"), this);
    m_projectButton->setObjectName("projectButton");
    m_projectButton->setToolTip(tr("Project boundary vertices to target geometry"));
    projectLayout->addWidget(m_projectButton);
    projectLayout->addWidget(new QLabel(tr("Target:"), this));
    m_projectTargetComboBox = new QComboBox(this);
    m_projectTargetComboBox->setObjectName("projectTargetComboBox");
    m_projectTargetComboBox->setToolTip(tr("Select target geometry for projection"));
    projectLayout->addWidget(m_projectTargetComboBox, 1);
    projectGroupLayout->addLayout(projectLayout);
    
    operationsTabLayout->addWidget(projectGroup);
    
    // Smoothing Group
    QGroupBox *smoothingGroup = new QGroupBox(tr("Smoothing"), operationsTab);
    QVBoxLayout *smoothingGroupLayout = new QVBoxLayout(smoothingGroup);
    
    // Smoothing section - first row with button and delta
    QHBoxLayout *smoothingLayout = new QHBoxLayout();
    m_smoothingButton = new QPushButton(tr("Smooth"), this);
    m_smoothingButton->setObjectName("smoothingButton");
    m_smoothingButton->setToolTip(tr("Apply smoothing to the mesh"));
    smoothingLayout->addWidget(m_smoothingButton);
    smoothingLayout->addWidget(new QLabel(tr("Delta:"), this));
    m_smoothingDeltaSpinBox = new QDoubleSpinBox(this);
    m_smoothingDeltaSpinBox->setObjectName("smoothingDeltaSpinBox");
    m_smoothingDeltaSpinBox->setRange(0.001, 1.0);
    m_smoothingDeltaSpinBox->setSingleStep(0.01);
    m_smoothingDeltaSpinBox->setValue(0.1);
    m_smoothingDeltaSpinBox->setDecimals(3);
    m_smoothingDeltaSpinBox->setToolTip(tr("Smoothing delta parameter (default 0.1)"));
    smoothingLayout->addWidget(m_smoothingDeltaSpinBox);
    smoothingLayout->addStretch();
    smoothingGroupLayout->addLayout(smoothingLayout);
    
    // Smoothing options - second row with checkboxes
    QHBoxLayout *smoothingOptionsLayout = new QHBoxLayout();
    m_smoothingFixBoundaryCheckBox = new QCheckBox(tr("Fix Boundary"), this);
    m_smoothingFixBoundaryCheckBox->setObjectName("smoothingFixBoundaryCheckBox");
    m_smoothingFixBoundaryCheckBox->setToolTip(tr("Keep boundary vertices fixed during smoothing"));
    smoothingOptionsLayout->addWidget(m_smoothingFixBoundaryCheckBox);
    m_smoothingPerturb1CheckBox = new QCheckBox(tr("Perturb 1"), this);
    m_smoothingPerturb1CheckBox->setObjectName("smoothingPerturb1CheckBox");
    m_smoothingPerturb1CheckBox->setToolTip(tr("Apply initial perturbation before smoothing"));
    smoothingOptionsLayout->addWidget(m_smoothingPerturb1CheckBox);
    m_smoothingGeoFlowCheckBox = new QCheckBox(tr("Geo Flow"), this);
    m_smoothingGeoFlowCheckBox->setObjectName("smoothingGeoFlowCheckBox");
    m_smoothingGeoFlowCheckBox->setChecked(true);  // Default enabled
    m_smoothingGeoFlowCheckBox->setToolTip(tr("Apply geometric flow smoothing"));
    smoothingOptionsLayout->addWidget(m_smoothingGeoFlowCheckBox);
    m_smoothingEnabledCheckBox = new QCheckBox(tr("Smooth"), this);
    m_smoothingEnabledCheckBox->setObjectName("smoothingEnabledCheckBox");
    m_smoothingEnabledCheckBox->setChecked(true);  // Default enabled
    m_smoothingEnabledCheckBox->setToolTip(tr("Apply smoothing pass"));
    smoothingOptionsLayout->addWidget(m_smoothingEnabledCheckBox);
    m_smoothingPerturb2CheckBox = new QCheckBox(tr("Perturb 2"), this);
    m_smoothingPerturb2CheckBox->setObjectName("smoothingPerturb2CheckBox");
    m_smoothingPerturb2CheckBox->setToolTip(tr("Apply final perturbation after smoothing"));
    smoothingOptionsLayout->addWidget(m_smoothingPerturb2CheckBox);
    smoothingOptionsLayout->addStretch();
    smoothingGroupLayout->addLayout(smoothingOptionsLayout);
    
    operationsTabLayout->addWidget(smoothingGroup);
    
    // Quality Improve Group
    QGroupBox *qualityGroup = new QGroupBox(tr("Quality Improvement"), operationsTab);
    QVBoxLayout *qualityGroupLayout = new QVBoxLayout(qualityGroup);
    
    QHBoxLayout *qualityLayout = new QHBoxLayout();
    m_qualityImproveButton = new QPushButton(tr("Quality Improve"), this);
    m_qualityImproveButton->setObjectName("qualityImproveButton");
    m_qualityImproveButton->setToolTip(tr("Improve mesh quality"));
    qualityLayout->addWidget(m_qualityImproveButton);
    qualityLayout->addWidget(new QLabel(tr("Iters:"), this));
    m_qualityIterationsSpinBox = new QSpinBox(this);
    m_qualityIterationsSpinBox->setObjectName("qualityIterationsSpinBox");
    m_qualityIterationsSpinBox->setRange(1, 100);
    m_qualityIterationsSpinBox->setValue(1);
    m_qualityIterationsSpinBox->setToolTip(tr("Number of improvement iterations"));
    qualityLayout->addWidget(m_qualityIterationsSpinBox);
    qualityLayout->addWidget(new QLabel(tr("Method:"), this));
    m_qualityMethodComboBox = new QComboBox(this);
    m_qualityMethodComboBox->setObjectName("qualityMethodComboBox");
    m_qualityMethodComboBox->addItem(tr("No Improve"), 0);
    m_qualityMethodComboBox->addItem(tr("Geo Flow"), 1);
    m_qualityMethodComboBox->addItem(tr("Edge Contract"), 2);
    m_qualityMethodComboBox->addItem(tr("Joe Liu"), 3);
    m_qualityMethodComboBox->addItem(tr("Minimal Vol"), 4);
    m_qualityMethodComboBox->addItem(tr("Optimization"), 5);
    m_qualityMethodComboBox->setCurrentIndex(1);  // Default to Geo Flow
    m_qualityMethodComboBox->setToolTip(tr("Select mesh improvement method"));
    qualityLayout->addWidget(m_qualityMethodComboBox);
    qualityLayout->addStretch();
    qualityGroupLayout->addLayout(qualityLayout);
    
    operationsTabLayout->addWidget(qualityGroup);
    operationsTabLayout->addStretch();
    
    // === Info Tab ===
    QWidget *infoTab = new QWidget();
    QVBoxLayout *infoLayout = new QVBoxLayout(infoTab);
    
    QLabel *infoLabel = new QLabel(tr("Geometry node metadata:"), infoTab);
    infoLayout->addWidget(infoLabel);
    
    m_infoTable = new QTableWidget(this);
    m_infoTable->setObjectName("infoTable");
    m_infoTable->setColumnCount(2);
    m_infoTable->setHorizontalHeaderLabels({tr("Property"), tr("Value")});
    m_infoTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    m_infoTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    m_infoTable->verticalHeader()->setVisible(false);
    m_infoTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_infoTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_infoTable->setAlternatingRowColors(true);
    infoLayout->addWidget(m_infoTable);
    
    // Add tabs to tab widget
    tabWidget->addTab(appearanceTab, tr("Appearance"));
    tabWidget->addTab(materialTab, tr("Material"));
    tabWidget->addTab(renderingTab, tr("Rendering"));
    tabWidget->addTab(operationsTab, tr("Operations"));
    tabWidget->addTab(infoTab, tr("Info"));
    
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
    connect(m_singleColorCheckBox, &QCheckBox::toggled,
            this, &GeometryDialog::onSingleColorChanged);
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
    
    // Visibility signals
    connect(m_visibilityCheckBox, &QCheckBox::toggled,
            this, &GeometryDialog::onVisibilityChanged);
    
    // Bounding box signals
    connect(m_showBBoxCheckBox, &QCheckBox::toggled,
            this, &GeometryDialog::onShowBBoxChanged);
    connect(m_bboxColorButton, &QPushButton::clicked,
            this, &GeometryDialog::onBBoxColorChanged);
    connect(m_showExtentLabelsCheckBox, &QCheckBox::toggled,
            this, &GeometryDialog::onShowExtentLabelsChanged);
    connect(m_extentLabelColorButton, &QPushButton::clicked,
            this, &GeometryDialog::onExtentLabelColorChanged);
    
    // Geometry operations signals
    connect(m_invertNormalsButton, &QPushButton::clicked,
            this, &GeometryDialog::onInvertNormalsClicked);
    connect(m_reorientButton, &QPushButton::clicked,
            this, &GeometryDialog::onReorientClicked);
    connect(m_projectButton, &QPushButton::clicked,
            this, &GeometryDialog::onProjectClicked);
    connect(m_smoothingButton, &QPushButton::clicked,
            this, &GeometryDialog::onSmoothingClicked);
    connect(m_qualityImproveButton, &QPushButton::clicked,
            this, &GeometryDialog::onQualityImproveClicked);
}

void GeometryDialog::populateGeometryList()
{
    m_geometryComboBox->clear();
    m_projectTargetComboBox->clear();
    m_geometryNames.clear();
    
    if (!m_sceneGraph) return;
    
    // Get all geometry nodes recursively
    auto allGeometries = m_sceneGraph->getAllGeometryGraphics();
    
    for (const auto& geomNode : allGeometries) {
        if (geomNode && geomNode->getGeometry() && !geomNode->getGeometry()->empty()) {
            std::string name = geomNode->getName();
            m_geometryNames.push_back(name);
            m_geometryComboBox->addItem(QString::fromStdString(name));
            m_projectTargetComboBox->addItem(QString::fromStdString(name));
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
    
    // Update single color checkbox
    try { m_singleColorCheckBox->setChecked(geomNode->getState("use_single_color").value<bool>()); } catch (const std::exception&) { m_singleColorCheckBox->setChecked(false); } catch (...) { m_singleColorCheckBox->setChecked(false); }
    
    // Update visibility checkbox
    try { 
        int visible = geomNode->getState("visible").value<int>(); 
        m_visibilityCheckBox->setChecked(visible != 0); 
    } catch (const std::exception&) { 
        m_visibilityCheckBox->setChecked(true); 
    } catch (...) { 
        m_visibilityCheckBox->setChecked(true); 
    }
    
    // Update bounding box controls
    try { 
        int showBBox = geomNode->getState("show_bbox").value<int>(); 
        m_showBBoxCheckBox->setChecked(showBBox != 0); 
    } catch (const std::exception&) { 
        m_showBBoxCheckBox->setChecked(false); 
    } catch (...) { 
        m_showBBoxCheckBox->setChecked(false); 
    }
    
    // Update bounding box color
    geomNode->getBBoxColor(m_bboxColor[0], m_bboxColor[1], m_bboxColor[2]);
    updateBBoxColorButton();
    
    // Update extent label controls
    try { 
        int showExtentLabels = geomNode->getState("show_extent_labels").value<int>(); 
        m_showExtentLabelsCheckBox->setChecked(showExtentLabels != 0); 
    } catch (const std::exception&) { 
        m_showExtentLabelsCheckBox->setChecked(false); 
    } catch (...) { 
        m_showExtentLabelsCheckBox->setChecked(false); 
    }
    
    // Update extent label color
    try { 
        m_extentLabelColor[0] = geomNode->getState("extent_label_color_r").value<double>();
        m_extentLabelColor[1] = geomNode->getState("extent_label_color_g").value<double>();
        m_extentLabelColor[2] = geomNode->getState("extent_label_color_b").value<double>();
    } catch (const std::exception&) { 
        m_extentLabelColor[0] = m_extentLabelColor[1] = m_extentLabelColor[2] = 1.0; // Default to white
    } catch (...) { 
        m_extentLabelColor[0] = m_extentLabelColor[1] = m_extentLabelColor[2] = 1.0; // Default to white
    }
    updateExtentLabelColorButton();
    
    // Update info table with metadata
    m_infoTable->setRowCount(0);
    const auto& metadata = geomNode->getAllMetadata();
    for (const auto& kv : metadata) {
        int row = m_infoTable->rowCount();
        m_infoTable->insertRow(row);
        
        m_infoTable->setItem(row, 0, new QTableWidgetItem(QString::fromStdString(kv.first)));
        
        // Convert std::any to string for display
        QString valueStr;
        try {
            if (kv.second.type() == typeid(int)) {
                valueStr = QString::number(std::any_cast<int>(kv.second));
            } else if (kv.second.type() == typeid(double)) {
                valueStr = QString::number(std::any_cast<double>(kv.second), 'g', 6);
            } else if (kv.second.type() == typeid(float)) {
                valueStr = QString::number(std::any_cast<float>(kv.second), 'g', 6);
            } else if (kv.second.type() == typeid(std::string)) {
                valueStr = QString::fromStdString(std::any_cast<std::string>(kv.second));
            } else if (kv.second.type() == typeid(bool)) {
                valueStr = std::any_cast<bool>(kv.second) ? tr("true") : tr("false");
            } else {
                valueStr = tr("<unknown type>");
            }
        } catch (...) {
            valueStr = tr("<error>");
        }
        
        m_infoTable->setItem(row, 1, new QTableWidgetItem(valueStr));
    }
    
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

void GeometryDialog::onSingleColorChanged(bool checked)
{
    if (m_updating) return;
    
    int index = m_geometryComboBox->currentIndex();
    if (index < 0 || index >= static_cast<int>(m_geometryNames.size())) return;
    
    const std::string& geomName = m_geometryNames[index];
    auto graphicsNode = m_sceneGraph->getGraphics(geomName);
    auto geomNode = std::dynamic_pointer_cast<GeometryNode>(graphicsNode);
    
    if (!geomNode) return;
    
    geomNode->setUseSingleColor(checked);
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
    m_singleColorCheckBox->setEnabled(enabled);
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
    m_visibilityCheckBox->setEnabled(enabled);
    m_showBBoxCheckBox->setEnabled(enabled);
    m_bboxColorButton->setEnabled(enabled);
    setOperationButtonsEnabled(enabled);
}

void GeometryDialog::setOperationButtonsEnabled(bool enabled)
{
    m_invertNormalsButton->setEnabled(enabled);
    m_reorientButton->setEnabled(enabled);
    m_projectButton->setEnabled(enabled);
    m_projectTargetComboBox->setEnabled(enabled);
    m_smoothingButton->setEnabled(enabled);
    m_smoothingDeltaSpinBox->setEnabled(enabled);
    m_smoothingFixBoundaryCheckBox->setEnabled(enabled);
    m_smoothingPerturb1CheckBox->setEnabled(enabled);
    m_smoothingGeoFlowCheckBox->setEnabled(enabled);
    m_smoothingEnabledCheckBox->setEnabled(enabled);
    m_smoothingPerturb2CheckBox->setEnabled(enabled);
    m_qualityImproveButton->setEnabled(enabled);
    m_qualityIterationsSpinBox->setEnabled(enabled);
    m_qualityMethodComboBox->setEnabled(enabled);
}

void GeometryDialog::onVisibilityChanged(bool checked)
{
    if (m_updating) return;
    
    int index = m_geometryComboBox->currentIndex();
    if (index < 0 || index >= static_cast<int>(m_geometryNames.size())) return;
    
    const std::string& geomName = m_geometryNames[index];
    auto graphicsNode = m_sceneGraph->getGraphics(geomName);
    
    if (!graphicsNode) return;
    
    graphicsNode->setVisible(checked);
}

void GeometryDialog::onShowBBoxChanged(bool checked)
{
    if (m_updating) return;
    
    int index = m_geometryComboBox->currentIndex();
    if (index < 0 || index >= static_cast<int>(m_geometryNames.size())) return;
    
    const std::string& geomName = m_geometryNames[index];
    auto graphicsNode = m_sceneGraph->getGraphics(geomName);
    
    if (!graphicsNode) return;
    
    graphicsNode->setShowBBox(checked);
}

void GeometryDialog::onBBoxColorChanged()
{
    if (m_updating) return;
    
    int index = m_geometryComboBox->currentIndex();
    if (index < 0 || index >= static_cast<int>(m_geometryNames.size())) return;
    
    QColor currentColor = QColor::fromRgbF(m_bboxColor[0], m_bboxColor[1], m_bboxColor[2]);
    QColor color = QColorDialog::getColor(currentColor, this, tr("Select Bounding Box Color"));
    
    if (color.isValid()) {
        m_bboxColor[0] = color.redF();
        m_bboxColor[1] = color.greenF();
        m_bboxColor[2] = color.blueF();
        
        const std::string& geomName = m_geometryNames[index];
        auto graphicsNode = m_sceneGraph->getGraphics(geomName);
        
        if (graphicsNode) {
            graphicsNode->setBBoxColor(m_bboxColor[0], m_bboxColor[1], m_bboxColor[2]);
        }
        
        updateBBoxColorButton();
    }
}

void GeometryDialog::updateBBoxColorButton()
{
    int r = static_cast<int>(m_bboxColor[0] * 255);
    int g = static_cast<int>(m_bboxColor[1] * 255);
    int b = static_cast<int>(m_bboxColor[2] * 255);
    QString style = QString("background-color: rgb(%1, %2, %3);").arg(r).arg(g).arg(b);
    m_bboxColorButton->setStyleSheet(style);
}

void GeometryDialog::onShowExtentLabelsChanged(bool checked)
{
    if (m_updating) return;
    
    int index = m_geometryComboBox->currentIndex();
    if (index < 0 || index >= static_cast<int>(m_geometryNames.size())) return;
    
    const std::string& geomName = m_geometryNames[index];
    auto graphicsNode = m_sceneGraph->getGraphics(geomName);
    
    if (!graphicsNode) return;
    
    graphicsNode->setShowExtentLabels(checked);
}

void GeometryDialog::onExtentLabelColorChanged()
{
    if (m_updating) return;
    
    int index = m_geometryComboBox->currentIndex();
    if (index < 0 || index >= static_cast<int>(m_geometryNames.size())) return;
    
    QColor currentColor = QColor::fromRgbF(m_extentLabelColor[0], m_extentLabelColor[1], m_extentLabelColor[2]);
    QColor color = QColorDialog::getColor(currentColor, this, tr("Select Extent Label Color"));
    
    if (color.isValid()) {
        m_extentLabelColor[0] = color.redF();
        m_extentLabelColor[1] = color.greenF();
        m_extentLabelColor[2] = color.blueF();
        
        const std::string& geomName = m_geometryNames[index];
        auto graphicsNode = m_sceneGraph->getGraphics(geomName);
        
        if (graphicsNode) {
            graphicsNode->setExtentLabelColor(m_extentLabelColor[0], m_extentLabelColor[1], m_extentLabelColor[2]);
        }
        
        updateExtentLabelColorButton();
    }
}

void GeometryDialog::updateExtentLabelColorButton()
{
    int r = static_cast<int>(m_extentLabelColor[0] * 255);
    int g = static_cast<int>(m_extentLabelColor[1] * 255);
    int b = static_cast<int>(m_extentLabelColor[2] * 255);
    QString style = QString("background-color: rgb(%1, %2, %3);").arg(r).arg(g).arg(b);
    m_extentLabelColorButton->setStyleSheet(style);
}

void GeometryDialog::onInvertNormalsClicked()
{
    if (m_updating) return;
    
    int index = m_geometryComboBox->currentIndex();
    if (index < 0 || index >= static_cast<int>(m_geometryNames.size())) return;
    
    const std::string& geomName = m_geometryNames[index];
    auto graphicsNode = m_sceneGraph->getGraphics(geomName);
    auto geomNode = std::dynamic_pointer_cast<GeometryNode>(graphicsNode);
    
    if (!geomNode || !geomNode->getGeometry()) return;
    
    // Get a copy of the geometry for thread-safe operation
    cvc::geometry geom = *geomNode->getGeometry();
    
    // Create unique thread key
    std::string threadKey = "invert_normals_" + geomName;
    
    // Disable button while processing
    m_invertNormalsButton->setEnabled(false);
    
    // Start the operation in a background thread
    cvc::app::instance().startThread(
        threadKey,
        [this, geom, geomName, threadKey]() mutable {
            // Use thread_feedback for proper progress tracking
            cvc::app::thread_feedback feedback(threadKey);
            
            try {
                cvc::app::instance().threadProgress(threadKey, 0.1);
                cvc::app::instance().threadInfo(threadKey, "Inverting normals...");
                
                // Perform the normal inversion
                geom.invert_normals();
                
                cvc::app::instance().threadProgress(threadKey, 0.9);
                cvc::app::instance().threadInfo(threadKey, "Updating scene...");
                
                // Post scene update to main thread
                m_sceneGraph->postEvent([this, geom, geomName, threadKey]() {
                    auto graphicsNode = m_sceneGraph->getGraphics(geomName);
                    auto geomNode = std::dynamic_pointer_cast<GeometryNode>(graphicsNode);
                    
                    if (geomNode) {
                        geomNode->setGeometry(geom);
                    }
                    
                    cvc::app::instance().finishThreadProgress(threadKey);
                    
                    // Re-enable button on Qt thread
                    QMetaObject::invokeMethod(this, [this]() {
                        m_invertNormalsButton->setEnabled(true);
                    }, Qt::QueuedConnection);
                });
                
            } catch (const boost::thread_interrupted&) {
                QMetaObject::invokeMethod(this, [this]() {
                    m_invertNormalsButton->setEnabled(true);
                }, Qt::QueuedConnection);
            } catch (const std::exception& e) {
                std::string errorMsg = std::string("Error inverting normals: ") + e.what();
                QMetaObject::invokeMethod(this, [this, errorMsg]() {
                    m_invertNormalsButton->setEnabled(true);
                    QMessageBox::warning(this, tr("Error"), QString::fromStdString(errorMsg));
                }, Qt::QueuedConnection);
            }
        },
        false  // Don't wait for existing thread
    );
}

void GeometryDialog::onReorientClicked()
{
    if (m_updating) return;
    
    int index = m_geometryComboBox->currentIndex();
    if (index < 0 || index >= static_cast<int>(m_geometryNames.size())) return;
    
    const std::string& geomName = m_geometryNames[index];
    auto graphicsNode = m_sceneGraph->getGraphics(geomName);
    auto geomNode = std::dynamic_pointer_cast<GeometryNode>(graphicsNode);
    
    if (!geomNode || !geomNode->getGeometry()) return;
    
    // Get a copy of the geometry for thread-safe operation
    cvc::geometry geom = *geomNode->getGeometry();
    
    // Create unique thread key
    std::string threadKey = "reorient_" + geomName;
    
    // Disable button while processing
    m_reorientButton->setEnabled(false);
    
    // Start the operation in a background thread
    cvc::app::instance().startThread(
        threadKey,
        [this, geom, geomName, threadKey]() mutable {
            cvc::app::thread_feedback feedback(threadKey);
            
            try {
                cvc::app::instance().threadProgress(threadKey, 0.1);
                cvc::app::instance().threadInfo(threadKey, "Reorienting mesh...");
                
                // Perform the reorient operation
                geom.reorient();
                
                cvc::app::instance().threadProgress(threadKey, 0.9);
                cvc::app::instance().threadInfo(threadKey, "Updating scene...");
                
                // Post scene update to main thread
                m_sceneGraph->postEvent([this, geom, geomName, threadKey]() {
                    auto graphicsNode = m_sceneGraph->getGraphics(geomName);
                    auto geomNode = std::dynamic_pointer_cast<GeometryNode>(graphicsNode);
                    
                    if (geomNode) {
                        geomNode->setGeometry(geom);
                    }
                    
                    cvc::app::instance().finishThreadProgress(threadKey);
                    
                    QMetaObject::invokeMethod(this, [this]() {
                        m_reorientButton->setEnabled(true);
                    }, Qt::QueuedConnection);
                });
                
            } catch (const boost::thread_interrupted&) {
                QMetaObject::invokeMethod(this, [this]() {
                    m_reorientButton->setEnabled(true);
                }, Qt::QueuedConnection);
            } catch (const std::exception& e) {
                std::string errorMsg = std::string("Error reorienting: ") + e.what();
                QMetaObject::invokeMethod(this, [this, errorMsg]() {
                    m_reorientButton->setEnabled(true);
                    QMessageBox::warning(this, tr("Error"), QString::fromStdString(errorMsg));
                }, Qt::QueuedConnection);
            }
        },
        false
    );
}

void GeometryDialog::onProjectClicked()
{
    if (m_updating) return;
    
    int index = m_geometryComboBox->currentIndex();
    if (index < 0 || index >= static_cast<int>(m_geometryNames.size())) return;
    
    int targetIndex = m_projectTargetComboBox->currentIndex();
    if (targetIndex < 0 || targetIndex >= static_cast<int>(m_geometryNames.size())) return;
    
    // Don't project onto self
    if (index == targetIndex) {
        QMessageBox::warning(this, tr("Warning"), tr("Cannot project geometry onto itself"));
        return;
    }
    
    const std::string& geomName = m_geometryNames[index];
    const std::string& targetName = m_geometryNames[targetIndex];
    
    auto graphicsNode = m_sceneGraph->getGraphics(geomName);
    auto geomNode = std::dynamic_pointer_cast<GeometryNode>(graphicsNode);
    
    auto targetGraphicsNode = m_sceneGraph->getGraphics(targetName);
    auto targetGeomNode = std::dynamic_pointer_cast<GeometryNode>(targetGraphicsNode);
    
    if (!geomNode || !geomNode->getGeometry() || !targetGeomNode || !targetGeomNode->getGeometry()) return;
    
    // Get copies of the geometries
    cvc::geometry geom = *geomNode->getGeometry();
    cvc::geometry targetGeom = *targetGeomNode->getGeometry();
    
    std::string threadKey = "project_" + geomName;
    
    m_projectButton->setEnabled(false);
    
    cvc::app::instance().startThread(
        threadKey,
        [this, geom, targetGeom, geomName, threadKey]() mutable {
            cvc::app::thread_feedback feedback(threadKey);
            
            try {
                cvc::app::instance().threadProgress(threadKey, 0.1);
                cvc::app::instance().threadInfo(threadKey, "Projecting to target geometry...");
                
                // Perform the projection
                geom.project(targetGeom);
                
                cvc::app::instance().threadProgress(threadKey, 0.9);
                cvc::app::instance().threadInfo(threadKey, "Updating scene...");
                
                m_sceneGraph->postEvent([this, geom, geomName, threadKey]() {
                    auto graphicsNode = m_sceneGraph->getGraphics(geomName);
                    auto geomNode = std::dynamic_pointer_cast<GeometryNode>(graphicsNode);
                    
                    if (geomNode) {
                        geomNode->setGeometry(geom);
                    }
                    
                    cvc::app::instance().finishThreadProgress(threadKey);
                    
                    QMetaObject::invokeMethod(this, [this]() {
                        m_projectButton->setEnabled(true);
                    }, Qt::QueuedConnection);
                });
                
            } catch (const boost::thread_interrupted&) {
                QMetaObject::invokeMethod(this, [this]() {
                    m_projectButton->setEnabled(true);
                }, Qt::QueuedConnection);
            } catch (const std::exception& e) {
                std::string errorMsg = std::string("Error projecting: ") + e.what();
                QMetaObject::invokeMethod(this, [this, errorMsg]() {
                    m_projectButton->setEnabled(true);
                    QMessageBox::warning(this, tr("Error"), QString::fromStdString(errorMsg));
                }, Qt::QueuedConnection);
            }
        },
        false
    );
}

void GeometryDialog::onSmoothingClicked()
{
    if (m_updating) return;
    
    int index = m_geometryComboBox->currentIndex();
    if (index < 0 || index >= static_cast<int>(m_geometryNames.size())) return;
    
    const std::string& geomName = m_geometryNames[index];
    auto graphicsNode = m_sceneGraph->getGraphics(geomName);
    auto geomNode = std::dynamic_pointer_cast<GeometryNode>(graphicsNode);
    
    if (!geomNode || !geomNode->getGeometry()) return;
    
    // Get parameters from UI
    float delta = static_cast<float>(m_smoothingDeltaSpinBox->value());
    bool fixBoundary = m_smoothingFixBoundaryCheckBox->isChecked();
    bool perturb1 = m_smoothingPerturb1CheckBox->isChecked();
    bool geoFlow = m_smoothingGeoFlowCheckBox->isChecked();
    bool smoothingEnabled = m_smoothingEnabledCheckBox->isChecked();
    bool perturb2 = m_smoothingPerturb2CheckBox->isChecked();
    
    cvc::geometry geom = *geomNode->getGeometry();
    
    std::string threadKey = "smoothing_" + geomName;
    
    m_smoothingButton->setEnabled(false);
    
    cvc::app::instance().startThread(
        threadKey,
        [this, geom, delta, fixBoundary, perturb1, geoFlow, smoothingEnabled, perturb2, geomName, threadKey]() mutable {
            cvc::app::thread_feedback feedback(threadKey);
            
            try {
                cvc::app::instance().threadProgress(threadKey, 0.1);
                cvc::app::instance().threadInfo(threadKey, "Smoothing mesh...");
                
                // Perform the smoothing operation with all parameters
                geom.smoothing(delta, fixBoundary, perturb1, geoFlow, smoothingEnabled, perturb2);
                
                cvc::app::instance().threadProgress(threadKey, 0.9);
                cvc::app::instance().threadInfo(threadKey, "Updating scene...");
                
                m_sceneGraph->postEvent([this, geom, geomName, threadKey]() {
                    auto graphicsNode = m_sceneGraph->getGraphics(geomName);
                    auto geomNode = std::dynamic_pointer_cast<GeometryNode>(graphicsNode);
                    
                    if (geomNode) {
                        geomNode->setGeometry(geom);
                    }
                    
                    cvc::app::instance().finishThreadProgress(threadKey);
                    
                    QMetaObject::invokeMethod(this, [this]() {
                        m_smoothingButton->setEnabled(true);
                    }, Qt::QueuedConnection);
                });
                
            } catch (const boost::thread_interrupted&) {
                QMetaObject::invokeMethod(this, [this]() {
                    m_smoothingButton->setEnabled(true);
                }, Qt::QueuedConnection);
            } catch (const std::exception& e) {
                std::string errorMsg = std::string("Error smoothing: ") + e.what();
                QMetaObject::invokeMethod(this, [this, errorMsg]() {
                    m_smoothingButton->setEnabled(true);
                    QMessageBox::warning(this, tr("Error"), QString::fromStdString(errorMsg));
                }, Qt::QueuedConnection);
            }
        },
        false
    );
}

void GeometryDialog::onQualityImproveClicked()
{
    if (m_updating) return;
    
    int index = m_geometryComboBox->currentIndex();
    if (index < 0 || index >= static_cast<int>(m_geometryNames.size())) return;
    
    const std::string& geomName = m_geometryNames[index];
    auto graphicsNode = m_sceneGraph->getGraphics(geomName);
    auto geomNode = std::dynamic_pointer_cast<GeometryNode>(graphicsNode);
    
    if (!geomNode || !geomNode->getGeometry()) return;
    
    // Get parameters from UI
    int iterations = m_qualityIterationsSpinBox->value();
    int methodInt = m_qualityMethodComboBox->currentData().toInt();
    cvc::improvement_method method = static_cast<cvc::improvement_method>(methodInt);
    
    cvc::geometry geom = *geomNode->getGeometry();
    
    std::string threadKey = "quality_improve_" + geomName;
    
    m_qualityImproveButton->setEnabled(false);
    
    cvc::app::instance().startThread(
        threadKey,
        [this, geom, iterations, method, geomName, threadKey]() mutable {
            cvc::app::thread_feedback feedback(threadKey);
            
            try {
                cvc::app::instance().threadProgress(threadKey, 0.1);
                cvc::app::instance().threadInfo(threadKey, "Improving mesh quality...");
                
                // Perform the quality improvement
                geom.quality_improve(iterations, method);
                
                cvc::app::instance().threadProgress(threadKey, 0.9);
                cvc::app::instance().threadInfo(threadKey, "Updating scene...");
                
                m_sceneGraph->postEvent([this, geom, geomName, threadKey]() {
                    auto graphicsNode = m_sceneGraph->getGraphics(geomName);
                    auto geomNode = std::dynamic_pointer_cast<GeometryNode>(graphicsNode);
                    
                    if (geomNode) {
                        geomNode->setGeometry(geom);
                    }
                    
                    cvc::app::instance().finishThreadProgress(threadKey);
                    
                    QMetaObject::invokeMethod(this, [this]() {
                        m_qualityImproveButton->setEnabled(true);
                    }, Qt::QueuedConnection);
                });
                
            } catch (const boost::thread_interrupted&) {
                QMetaObject::invokeMethod(this, [this]() {
                    m_qualityImproveButton->setEnabled(true);
                }, Qt::QueuedConnection);
            } catch (const std::exception& e) {
                std::string errorMsg = std::string("Error improving quality: ") + e.what();
                QMetaObject::invokeMethod(this, [this, errorMsg]() {
                    m_qualityImproveButton->setEnabled(true);
                    QMessageBox::warning(this, tr("Error"), QString::fromStdString(errorMsg));
                }, Qt::QueuedConnection);
            }
        },
        false
    );
}
