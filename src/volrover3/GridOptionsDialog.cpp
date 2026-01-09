#include <volrover3/GridOptionsDialog.h>
#include <volrover3/GridNode.h>
#include <cvc/state.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QCheckBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QSlider>
#include <QLabel>
#include <QPushButton>
#include <QColorDialog>
#include <QShowEvent>
#include <QCloseEvent>
#include <QTabWidget>

GridOptionsDialog::GridOptionsDialog(std::shared_ptr<GridNode> gridNode, QWidget *parent)
    : QWidget(parent)
    , m_gridNode(gridNode)
    , m_yzPlaneCheckBox(nullptr)
    , m_xzPlaneCheckBox(nullptr)
    , m_xyPlaneCheckBox(nullptr)
    , m_xDivisionsSpinBox(nullptr)
    , m_yDivisionsSpinBox(nullptr)
    , m_zDivisionsSpinBox(nullptr)
    , m_xTickIntervalSpinBox(nullptr)
    , m_yTickIntervalSpinBox(nullptr)
    , m_zTickIntervalSpinBox(nullptr)
    , m_yzPlaneColorButton(nullptr)
    , m_xzPlaneColorButton(nullptr)
    , m_xyPlaneColorButton(nullptr)
    , m_tickLabelColorButton(nullptr)
    , m_tickLabelFontSizeSpinBox(nullptr)
    , m_yzLineWidthSpinBox(nullptr)
    , m_xzLineWidthSpinBox(nullptr)
    , m_xyLineWidthSpinBox(nullptr)
    , m_yzOpacitySlider(nullptr)
    , m_xzOpacitySlider(nullptr)
    , m_xyOpacitySlider(nullptr)
    , m_updatingFromState(false)
{
    setWindowTitle(tr("Grid Options"));
    setMinimumWidth(400);
    setAttribute(Qt::WA_DeleteOnClose);
    setupUI();
    connectSignals();
    connectStateMonitoring();
    loadFromState();
}

GridOptionsDialog::~GridOptionsDialog()
{
    disconnectStateMonitoring();
}

void GridOptionsDialog::showEvent(QShowEvent *event)
{
    QWidget::showEvent(event);
    // Reload state from GridNode when dialog is shown to ensure sync
    loadFromState();
}

void GridOptionsDialog::closeEvent(QCloseEvent *event)
{
    disconnectStateMonitoring();
    QWidget::closeEvent(event);
}

void GridOptionsDialog::setupUI()
{
    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    
    // Create tab widget
    QTabWidget *tabWidget = new QTabWidget(this);
    
    // === Visibility Tab ===
    QWidget *visibilityTab = new QWidget();
    QVBoxLayout *visibilityLayout = new QVBoxLayout(visibilityTab);
    
    QGroupBox *visibilityGroup = new QGroupBox(tr("Grid Plane Visibility"), visibilityTab);
    QVBoxLayout *visLayout = new QVBoxLayout(visibilityGroup);
    
    m_yzPlaneCheckBox = new QCheckBox(tr("YZ Plane (X = 0)"), visibilityTab);
    m_xzPlaneCheckBox = new QCheckBox(tr("XZ Plane (Y = 0)"), visibilityTab);
    m_xyPlaneCheckBox = new QCheckBox(tr("XY Plane (Z = 0)"), visibilityTab);
    
    visLayout->addWidget(m_yzPlaneCheckBox);
    visLayout->addWidget(m_xzPlaneCheckBox);
    visLayout->addWidget(m_xyPlaneCheckBox);
    
    visibilityLayout->addWidget(visibilityGroup);
    visibilityLayout->addStretch();
    
    // === Divisions Tab ===
    QWidget *divisionsTab = new QWidget();
    QVBoxLayout *divisionsLayout = new QVBoxLayout(divisionsTab);
    
    QGroupBox *divisionsGroup = new QGroupBox(tr("Grid Divisions"), divisionsTab);
    QVBoxLayout *divLayout = new QVBoxLayout(divisionsGroup);
    
    QHBoxLayout *xDivLayout = new QHBoxLayout();
    xDivLayout->addWidget(new QLabel(tr("X Divisions:"), divisionsTab));
    m_xDivisionsSpinBox = new QSpinBox(divisionsTab);
    m_xDivisionsSpinBox->setRange(1, 512);
    m_xDivisionsSpinBox->setValue(64);
    xDivLayout->addWidget(m_xDivisionsSpinBox);
    xDivLayout->addStretch();
    divLayout->addLayout(xDivLayout);
    
    QHBoxLayout *yDivLayout = new QHBoxLayout();
    yDivLayout->addWidget(new QLabel(tr("Y Divisions:"), divisionsTab));
    m_yDivisionsSpinBox = new QSpinBox(divisionsTab);
    m_yDivisionsSpinBox->setRange(1, 512);
    m_yDivisionsSpinBox->setValue(64);
    yDivLayout->addWidget(m_yDivisionsSpinBox);
    yDivLayout->addStretch();
    divLayout->addLayout(yDivLayout);
    
    QHBoxLayout *zDivLayout = new QHBoxLayout();
    zDivLayout->addWidget(new QLabel(tr("Z Divisions:"), divisionsTab));
    m_zDivisionsSpinBox = new QSpinBox(divisionsTab);
    m_zDivisionsSpinBox->setRange(1, 512);
    m_zDivisionsSpinBox->setValue(64);
    zDivLayout->addWidget(m_zDivisionsSpinBox);
    zDivLayout->addStretch();
    divLayout->addLayout(zDivLayout);
    
    divisionsLayout->addWidget(divisionsGroup);
    divisionsLayout->addStretch();
    
    // === Ticks Tab ===
    QWidget *ticksTab = new QWidget();
    QVBoxLayout *ticksLayout = new QVBoxLayout(ticksTab);
    
    QGroupBox *tickGroup = new QGroupBox(tr("Tick Intervals"), ticksTab);
    QVBoxLayout *tickLayout = new QVBoxLayout(tickGroup);
    
    // Show ticks checkbox
    QHBoxLayout *showTicksLayout = new QHBoxLayout();
    showTicksLayout->addWidget(new QLabel(tr("Show Ticks:"), ticksTab));
    m_showTicksCheckBox = new QCheckBox(ticksTab);
    showTicksLayout->addWidget(m_showTicksCheckBox);
    showTicksLayout->addStretch();
    tickLayout->addLayout(showTicksLayout);
    
    QHBoxLayout *xTickLayout = new QHBoxLayout();
    xTickLayout->addWidget(new QLabel(tr("X Tick Interval:"), ticksTab));
    m_xTickIntervalSpinBox = new QSpinBox(ticksTab);
    m_xTickIntervalSpinBox->setRange(1, 256);
    m_xTickIntervalSpinBox->setValue(8);
    xTickLayout->addWidget(m_xTickIntervalSpinBox);
    xTickLayout->addStretch();
    tickLayout->addLayout(xTickLayout);
    
    QHBoxLayout *yTickLayout = new QHBoxLayout();
    yTickLayout->addWidget(new QLabel(tr("Y Tick Interval:"), ticksTab));
    m_yTickIntervalSpinBox = new QSpinBox(ticksTab);
    m_yTickIntervalSpinBox->setRange(1, 256);
    m_yTickIntervalSpinBox->setValue(8);
    yTickLayout->addWidget(m_yTickIntervalSpinBox);
    yTickLayout->addStretch();
    tickLayout->addLayout(yTickLayout);
    
    QHBoxLayout *zTickLayout = new QHBoxLayout();
    zTickLayout->addWidget(new QLabel(tr("Z Tick Interval:"), ticksTab));
    m_zTickIntervalSpinBox = new QSpinBox(ticksTab);
    m_zTickIntervalSpinBox->setRange(1, 256);
    m_zTickIntervalSpinBox->setValue(8);
    zTickLayout->addWidget(m_zTickIntervalSpinBox);
    zTickLayout->addStretch();
    tickLayout->addLayout(zTickLayout);
    
    ticksLayout->addWidget(tickGroup);
    
    // Tick Label Properties
    QGroupBox *labelGroup = new QGroupBox(tr("Tick Label Properties"), ticksTab);
    QVBoxLayout *labelLayout = new QVBoxLayout(labelGroup);
    
    QHBoxLayout *labelColorLayout = new QHBoxLayout();
    labelColorLayout->addWidget(new QLabel(tr("Label Color:"), ticksTab));
    m_tickLabelColorButton = new QPushButton(tr("Choose..."), ticksTab);
    m_tickLabelColorButton->setMinimumWidth(100);
    labelColorLayout->addWidget(m_tickLabelColorButton);
    labelColorLayout->addStretch();
    labelLayout->addLayout(labelColorLayout);
    
    QHBoxLayout *fontSizeLayout = new QHBoxLayout();
    fontSizeLayout->addWidget(new QLabel(tr("Font Size:"), ticksTab));
    m_tickLabelFontSizeSpinBox = new QSpinBox(ticksTab);
    m_tickLabelFontSizeSpinBox->setRange(6, 72);
    m_tickLabelFontSizeSpinBox->setValue(12);
    fontSizeLayout->addWidget(m_tickLabelFontSizeSpinBox);
    fontSizeLayout->addStretch();
    labelLayout->addLayout(fontSizeLayout);
    
    ticksLayout->addWidget(labelGroup);
    ticksLayout->addStretch();
    
    // === Plane Appearance Tab ===
    QWidget *appearanceTab = new QWidget();
    QVBoxLayout *appearanceLayout = new QVBoxLayout(appearanceTab);
    
    QGroupBox *colorsGroup = new QGroupBox(tr("Plane Appearance"), appearanceTab);
    QVBoxLayout *colorsLayout = new QVBoxLayout(colorsGroup);
    
    // YZ Plane
    colorsLayout->addWidget(new QLabel(tr("<b>YZ Plane (X = 0)</b>"), appearanceTab));
    QHBoxLayout *yzColorLayout = new QHBoxLayout();
    yzColorLayout->addWidget(new QLabel(tr("Color:"), appearanceTab));
    m_yzPlaneColorButton = new QPushButton(tr("Choose..."), appearanceTab);
    m_yzPlaneColorButton->setMinimumWidth(100);
    yzColorLayout->addWidget(m_yzPlaneColorButton);
    yzColorLayout->addStretch();
    colorsLayout->addLayout(yzColorLayout);
    
    QHBoxLayout *yzLineWidthLayout = new QHBoxLayout();
    yzLineWidthLayout->addWidget(new QLabel(tr("Line Width:"), appearanceTab));
    m_yzLineWidthSpinBox = new QDoubleSpinBox(appearanceTab);
    m_yzLineWidthSpinBox->setRange(0.1, 10.0);
    m_yzLineWidthSpinBox->setSingleStep(0.1);
    m_yzLineWidthSpinBox->setValue(1.0);
    m_yzLineWidthSpinBox->setMinimumWidth(100);
    yzLineWidthLayout->addWidget(m_yzLineWidthSpinBox);
    yzLineWidthLayout->addStretch();
    colorsLayout->addLayout(yzLineWidthLayout);
    
    QHBoxLayout *yzOpacityLayout = new QHBoxLayout();
    yzOpacityLayout->addWidget(new QLabel(tr("Opacity:"), appearanceTab));
    m_yzOpacitySlider = new QSlider(Qt::Horizontal, appearanceTab);
    m_yzOpacitySlider->setRange(0, 100);
    m_yzOpacitySlider->setValue(50);
    m_yzOpacitySlider->setMinimumWidth(100);
    yzOpacityLayout->addWidget(m_yzOpacitySlider);
    yzOpacityLayout->addStretch();
    colorsLayout->addLayout(yzOpacityLayout);
    
    // XZ Plane
    colorsLayout->addWidget(new QLabel(tr("<b>XZ Plane (Y = 0)</b>"), appearanceTab));
    QHBoxLayout *xzColorLayout = new QHBoxLayout();
    xzColorLayout->addWidget(new QLabel(tr("Color:"), appearanceTab));
    m_xzPlaneColorButton = new QPushButton(tr("Choose..."), appearanceTab);
    m_xzPlaneColorButton->setMinimumWidth(100);
    xzColorLayout->addWidget(m_xzPlaneColorButton);
    xzColorLayout->addStretch();
    colorsLayout->addLayout(xzColorLayout);
    
    QHBoxLayout *xzLineWidthLayout = new QHBoxLayout();
    xzLineWidthLayout->addWidget(new QLabel(tr("Line Width:"), appearanceTab));
    m_xzLineWidthSpinBox = new QDoubleSpinBox(appearanceTab);
    m_xzLineWidthSpinBox->setRange(0.1, 10.0);
    m_xzLineWidthSpinBox->setSingleStep(0.1);
    m_xzLineWidthSpinBox->setValue(1.0);
    m_xzLineWidthSpinBox->setMinimumWidth(100);
    xzLineWidthLayout->addWidget(m_xzLineWidthSpinBox);
    xzLineWidthLayout->addStretch();
    colorsLayout->addLayout(xzLineWidthLayout);
    
    QHBoxLayout *xzOpacityLayout = new QHBoxLayout();
    xzOpacityLayout->addWidget(new QLabel(tr("Opacity:"), appearanceTab));
    m_xzOpacitySlider = new QSlider(Qt::Horizontal, appearanceTab);
    m_xzOpacitySlider->setRange(0, 100);
    m_xzOpacitySlider->setValue(50);
    m_xzOpacitySlider->setMinimumWidth(100);
    xzOpacityLayout->addWidget(m_xzOpacitySlider);
    xzOpacityLayout->addStretch();
    colorsLayout->addLayout(xzOpacityLayout);
    
    // XY Plane
    colorsLayout->addWidget(new QLabel(tr("<b>XY Plane (Z = 0)</b>"), appearanceTab));
    QHBoxLayout *xyColorLayout = new QHBoxLayout();
    xyColorLayout->addWidget(new QLabel(tr("Color:"), appearanceTab));
    m_xyPlaneColorButton = new QPushButton(tr("Choose..."), appearanceTab);
    m_xyPlaneColorButton->setMinimumWidth(100);
    xyColorLayout->addWidget(m_xyPlaneColorButton);
    xyColorLayout->addStretch();
    colorsLayout->addLayout(xyColorLayout);
    
    QHBoxLayout *xyLineWidthLayout = new QHBoxLayout();
    xyLineWidthLayout->addWidget(new QLabel(tr("Line Width:"), appearanceTab));
    m_xyLineWidthSpinBox = new QDoubleSpinBox(appearanceTab);
    m_xyLineWidthSpinBox->setRange(0.1, 10.0);
    m_xyLineWidthSpinBox->setSingleStep(0.1);
    m_xyLineWidthSpinBox->setValue(1.0);
    m_xyLineWidthSpinBox->setMinimumWidth(100);
    xyLineWidthLayout->addWidget(m_xyLineWidthSpinBox);
    xyLineWidthLayout->addStretch();
    colorsLayout->addLayout(xyLineWidthLayout);
    
    QHBoxLayout *xyOpacityLayout = new QHBoxLayout();
    xyOpacityLayout->addWidget(new QLabel(tr("Opacity:"), appearanceTab));
    m_xyOpacitySlider = new QSlider(Qt::Horizontal, appearanceTab);
    m_xyOpacitySlider->setRange(0, 100);
    m_xyOpacitySlider->setValue(50);
    m_xyOpacitySlider->setMinimumWidth(100);
    xyOpacityLayout->addWidget(m_xyOpacitySlider);
    xyOpacityLayout->addStretch();
    colorsLayout->addLayout(xyOpacityLayout);
    
    appearanceLayout->addWidget(colorsGroup);
    appearanceLayout->addStretch();
    
    // Add tabs to tab widget
    tabWidget->addTab(visibilityTab, tr("Visibility"));
    tabWidget->addTab(divisionsTab, tr("Divisions"));
    tabWidget->addTab(ticksTab, tr("Ticks"));
    tabWidget->addTab(appearanceTab, tr("Appearance"));
    
    mainLayout->addWidget(tabWidget);
    
    setLayout(mainLayout);
}

void GridOptionsDialog::connectSignals()
{
    // Apply changes immediately when checkboxes change
    connect(m_yzPlaneCheckBox, &QCheckBox::toggled, this, &GridOptionsDialog::applyChanges);
    connect(m_xzPlaneCheckBox, &QCheckBox::toggled, this, &GridOptionsDialog::applyChanges);
    connect(m_xyPlaneCheckBox, &QCheckBox::toggled, this, &GridOptionsDialog::applyChanges);
    connect(m_showTicksCheckBox, &QCheckBox::toggled, this, &GridOptionsDialog::applyChanges);
    
    // Apply changes when spin boxes change
    connect(m_xDivisionsSpinBox, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &GridOptionsDialog::applyChanges);
    connect(m_yDivisionsSpinBox, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &GridOptionsDialog::applyChanges);
    connect(m_zDivisionsSpinBox, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &GridOptionsDialog::applyChanges);
    
    connect(m_xTickIntervalSpinBox, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &GridOptionsDialog::applyChanges);
    connect(m_yTickIntervalSpinBox, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &GridOptionsDialog::applyChanges);
    connect(m_zTickIntervalSpinBox, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &GridOptionsDialog::applyChanges);
    
    connect(m_tickLabelFontSizeSpinBox, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &GridOptionsDialog::applyChanges);
    
    // Line width and opacity spin boxes
    connect(m_yzLineWidthSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &GridOptionsDialog::applyChanges);
    connect(m_xzLineWidthSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &GridOptionsDialog::applyChanges);
    connect(m_xyLineWidthSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &GridOptionsDialog::applyChanges);
    connect(m_yzOpacitySlider, &QSlider::valueChanged,
            this, &GridOptionsDialog::applyChanges);
    connect(m_xzOpacitySlider, &QSlider::valueChanged,
            this, &GridOptionsDialog::applyChanges);
    connect(m_xyOpacitySlider, &QSlider::valueChanged,
            this, &GridOptionsDialog::applyChanges);
    
    // Color picker buttons
    connect(m_yzPlaneColorButton, &QPushButton::clicked, this, &GridOptionsDialog::chooseYZPlaneColor);
    connect(m_xzPlaneColorButton, &QPushButton::clicked, this, &GridOptionsDialog::chooseXZPlaneColor);
    connect(m_xyPlaneColorButton, &QPushButton::clicked, this, &GridOptionsDialog::chooseXYPlaneColor);
    connect(m_tickLabelColorButton, &QPushButton::clicked, this, &GridOptionsDialog::chooseTickLabelColor);
}

void GridOptionsDialog::loadFromState()
{
    if (!m_gridNode) return;
    
    m_updatingFromState = true;
    
    // Block signals while loading to avoid triggering apply
    m_yzPlaneCheckBox->blockSignals(true);
    m_xzPlaneCheckBox->blockSignals(true);
    m_xyPlaneCheckBox->blockSignals(true);
    m_showTicksCheckBox->blockSignals(true);
    m_xDivisionsSpinBox->blockSignals(true);
    m_yDivisionsSpinBox->blockSignals(true);
    m_zDivisionsSpinBox->blockSignals(true);
    m_xTickIntervalSpinBox->blockSignals(true);
    m_yTickIntervalSpinBox->blockSignals(true);
    m_zTickIntervalSpinBox->blockSignals(true);
    m_tickLabelFontSizeSpinBox->blockSignals(true);
    m_yzLineWidthSpinBox->blockSignals(true);
    m_xzLineWidthSpinBox->blockSignals(true);
    m_xyLineWidthSpinBox->blockSignals(true);
    m_yzOpacitySlider->blockSignals(true);
    m_xzOpacitySlider->blockSignals(true);
    m_xyOpacitySlider->blockSignals(true);
    
    // Load visibility from GridNode state
    m_yzPlaneCheckBox->setChecked(m_gridNode->getState("yz_plane.visible").value<bool>());
    m_xzPlaneCheckBox->setChecked(m_gridNode->getState("xz_plane.visible").value<bool>());
    m_xyPlaneCheckBox->setChecked(m_gridNode->getState("xy_plane.visible").value<bool>());
    
    // Load divisions from GridNode state
    int x = m_gridNode->getState("divisions_x").value<int>();
    int y = m_gridNode->getState("divisions_y").value<int>();
    int z = m_gridNode->getState("divisions_z").value<int>();
    m_xDivisionsSpinBox->setValue(x);
    m_yDivisionsSpinBox->setValue(y);
    m_zDivisionsSpinBox->setValue(z);
    
    // Load tick intervals from GridNode state
    m_showTicksCheckBox->setChecked(m_gridNode->getState("tics.visible").value<bool>());
    x = m_gridNode->getState("tics.interval_x").value<int>();
    y = m_gridNode->getState("tics.interval_y").value<int>();
    z = m_gridNode->getState("tics.interval_z").value<int>();
    m_xTickIntervalSpinBox->setValue(x);
    m_yTickIntervalSpinBox->setValue(y);
    m_zTickIntervalSpinBox->setValue(z);
    
    // Load colors from GridNode state
    m_yzPlaneColor[0] = m_gridNode->getState("yz_plane.color_r").value<double>();
    m_yzPlaneColor[1] = m_gridNode->getState("yz_plane.color_g").value<double>();
    m_yzPlaneColor[2] = m_gridNode->getState("yz_plane.color_b").value<double>();
    
    m_xzPlaneColor[0] = m_gridNode->getState("xz_plane.color_r").value<double>();
    m_xzPlaneColor[1] = m_gridNode->getState("xz_plane.color_g").value<double>();
    m_xzPlaneColor[2] = m_gridNode->getState("xz_plane.color_b").value<double>();
    
    m_xyPlaneColor[0] = m_gridNode->getState("xy_plane.color_r").value<double>();
    m_xyPlaneColor[1] = m_gridNode->getState("xy_plane.color_g").value<double>();
    m_xyPlaneColor[2] = m_gridNode->getState("xy_plane.color_b").value<double>();
    
    m_tickLabelColor[0] = m_gridNode->getState("tics.label_color_r").value<double>();
    m_tickLabelColor[1] = m_gridNode->getState("tics.label_color_g").value<double>();
    m_tickLabelColor[2] = m_gridNode->getState("tics.label_color_b").value<double>();
    
    updateColorButton(m_yzPlaneColorButton, m_yzPlaneColor[0], m_yzPlaneColor[1], m_yzPlaneColor[2]);
    updateColorButton(m_xzPlaneColorButton, m_xzPlaneColor[0], m_xzPlaneColor[1], m_xzPlaneColor[2]);
    updateColorButton(m_xyPlaneColorButton, m_xyPlaneColor[0], m_xyPlaneColor[1], m_xyPlaneColor[2]);
    updateColorButton(m_tickLabelColorButton, m_tickLabelColor[0], m_tickLabelColor[1], m_tickLabelColor[2]);
    
    // Load line width from GridNode state
    m_yzLineWidthSpinBox->setValue(m_gridNode->getState("yz_plane.line_width").value<double>());
    m_xzLineWidthSpinBox->setValue(m_gridNode->getState("xz_plane.line_width").value<double>());
    m_xyLineWidthSpinBox->setValue(m_gridNode->getState("xy_plane.line_width").value<double>());
    
    // Load opacity from GridNode state
    m_yzOpacitySlider->setValue(static_cast<int>(m_gridNode->getState("yz_plane.opacity").value<double>() * 100));
    m_xzOpacitySlider->setValue(static_cast<int>(m_gridNode->getState("xz_plane.opacity").value<double>() * 100));
    m_xyOpacitySlider->setValue(static_cast<int>(m_gridNode->getState("xy_plane.opacity").value<double>() * 100));
    
    // Load font size from GridNode state
    m_tickLabelFontSizeSpinBox->setValue(m_gridNode->getState("tics.label_font_size").value<int>());
    
    // Unblock signals
    m_yzPlaneCheckBox->blockSignals(false);
    m_xzPlaneCheckBox->blockSignals(false);
    m_xyPlaneCheckBox->blockSignals(false);
    m_showTicksCheckBox->blockSignals(false);
    m_xDivisionsSpinBox->blockSignals(false);
    m_yDivisionsSpinBox->blockSignals(false);
    m_zDivisionsSpinBox->blockSignals(false);
    m_xTickIntervalSpinBox->blockSignals(false);
    m_yTickIntervalSpinBox->blockSignals(false);
    m_zTickIntervalSpinBox->blockSignals(false);
    m_tickLabelFontSizeSpinBox->blockSignals(false);
    m_yzLineWidthSpinBox->blockSignals(false);
    m_xzLineWidthSpinBox->blockSignals(false);
    m_xyLineWidthSpinBox->blockSignals(false);
    m_yzOpacitySlider->blockSignals(false);
    m_xzOpacitySlider->blockSignals(false);
    m_xyOpacitySlider->blockSignals(false);
    
    m_updatingFromState = false;
}

void GridOptionsDialog::applyChanges()
{
    if (!m_gridNode || m_updatingFromState) return;
    
    // Apply visibility changes to GridNode state
    m_gridNode->getState("yz_plane.visible").value(m_yzPlaneCheckBox->isChecked());
    m_gridNode->getState("xz_plane.visible").value(m_xzPlaneCheckBox->isChecked());
    m_gridNode->getState("xy_plane.visible").value(m_xyPlaneCheckBox->isChecked());
    
    // Apply division changes to GridNode state
    m_gridNode->getState("divisions_x").value(m_xDivisionsSpinBox->value());
    m_gridNode->getState("divisions_y").value(m_yDivisionsSpinBox->value());
    m_gridNode->getState("divisions_z").value(m_zDivisionsSpinBox->value());
    
    // Apply tick interval changes to GridNode state
    m_gridNode->getState("tics.visible").value(m_showTicksCheckBox->isChecked());
    m_gridNode->getState("tics.interval_x").value(m_xTickIntervalSpinBox->value());
    m_gridNode->getState("tics.interval_y").value(m_yTickIntervalSpinBox->value());
    m_gridNode->getState("tics.interval_z").value(m_zTickIntervalSpinBox->value());
    
    // Apply color changes to GridNode state
    m_gridNode->getState("yz_plane.color_r").value(m_yzPlaneColor[0]);
    m_gridNode->getState("yz_plane.color_g").value(m_yzPlaneColor[1]);
    m_gridNode->getState("yz_plane.color_b").value(m_yzPlaneColor[2]);
    
    m_gridNode->getState("xz_plane.color_r").value(m_xzPlaneColor[0]);
    m_gridNode->getState("xz_plane.color_g").value(m_xzPlaneColor[1]);
    m_gridNode->getState("xz_plane.color_b").value(m_xzPlaneColor[2]);
    
    m_gridNode->getState("xy_plane.color_r").value(m_xyPlaneColor[0]);
    m_gridNode->getState("xy_plane.color_g").value(m_xyPlaneColor[1]);
    m_gridNode->getState("xy_plane.color_b").value(m_xyPlaneColor[2]);
    
    m_gridNode->getState("tics.label_color_r").value(m_tickLabelColor[0]);
    m_gridNode->getState("tics.label_color_g").value(m_tickLabelColor[1]);
    m_gridNode->getState("tics.label_color_b").value(m_tickLabelColor[2]);
    
    // Apply line width to GridNode state
    m_gridNode->getState("yz_plane.line_width").value(m_yzLineWidthSpinBox->value());
    m_gridNode->getState("xz_plane.line_width").value(m_xzLineWidthSpinBox->value());
    m_gridNode->getState("xy_plane.line_width").value(m_xyLineWidthSpinBox->value());
    
    // Apply opacity to GridNode state
    m_gridNode->getState("yz_plane.opacity").value(m_yzOpacitySlider->value() / 100.0);
    m_gridNode->getState("xz_plane.opacity").value(m_xzOpacitySlider->value() / 100.0);
    m_gridNode->getState("xy_plane.opacity").value(m_xyOpacitySlider->value() / 100.0);
    
    // Apply font size to GridNode state
    m_gridNode->getState("tics.label_font_size").value(m_tickLabelFontSizeSpinBox->value());
}

void GridOptionsDialog::chooseYZPlaneColor()
{
    QColor current(static_cast<int>(m_yzPlaneColor[0] * 255),
                   static_cast<int>(m_yzPlaneColor[1] * 255),
                   static_cast<int>(m_yzPlaneColor[2] * 255));
    QColor color = QColorDialog::getColor(current, this, tr("Choose YZ Plane Color"));
    if (color.isValid()) {
        m_yzPlaneColor[0] = color.redF();
        m_yzPlaneColor[1] = color.greenF();
        m_yzPlaneColor[2] = color.blueF();
        updateColorButton(m_yzPlaneColorButton, m_yzPlaneColor[0], m_yzPlaneColor[1], m_yzPlaneColor[2]);
        applyChanges();
    }
}

void GridOptionsDialog::chooseXZPlaneColor()
{
    QColor current(static_cast<int>(m_xzPlaneColor[0] * 255),
                   static_cast<int>(m_xzPlaneColor[1] * 255),
                   static_cast<int>(m_xzPlaneColor[2] * 255));
    QColor color = QColorDialog::getColor(current, this, tr("Choose XZ Plane Color"));
    if (color.isValid()) {
        m_xzPlaneColor[0] = color.redF();
        m_xzPlaneColor[1] = color.greenF();
        m_xzPlaneColor[2] = color.blueF();
        updateColorButton(m_xzPlaneColorButton, m_xzPlaneColor[0], m_xzPlaneColor[1], m_xzPlaneColor[2]);
        applyChanges();
    }
}

void GridOptionsDialog::chooseXYPlaneColor()
{
    QColor current(static_cast<int>(m_xyPlaneColor[0] * 255),
                   static_cast<int>(m_xyPlaneColor[1] * 255),
                   static_cast<int>(m_xyPlaneColor[2] * 255));
    QColor color = QColorDialog::getColor(current, this, tr("Choose XY Plane Color"));
    if (color.isValid()) {
        m_xyPlaneColor[0] = color.redF();
        m_xyPlaneColor[1] = color.greenF();
        m_xyPlaneColor[2] = color.blueF();
        updateColorButton(m_xyPlaneColorButton, m_xyPlaneColor[0], m_xyPlaneColor[1], m_xyPlaneColor[2]);
        applyChanges();
    }
}

void GridOptionsDialog::chooseTickLabelColor()
{
    QColor current(static_cast<int>(m_tickLabelColor[0] * 255),
                   static_cast<int>(m_tickLabelColor[1] * 255),
                   static_cast<int>(m_tickLabelColor[2] * 255));
    QColor color = QColorDialog::getColor(current, this, tr("Choose Tick Label Color"));
    if (color.isValid()) {
        m_tickLabelColor[0] = color.redF();
        m_tickLabelColor[1] = color.greenF();
        m_tickLabelColor[2] = color.blueF();
        updateColorButton(m_tickLabelColorButton, m_tickLabelColor[0], m_tickLabelColor[1], m_tickLabelColor[2]);
        applyChanges();
    }
}

void GridOptionsDialog::updateColorButton(QPushButton* button, double r, double g, double b)
{
    int red = static_cast<int>(r * 255);
    int green = static_cast<int>(g * 255);
    int blue = static_cast<int>(b * 255);
    
    QString styleSheet = QString("QPushButton { background-color: rgb(%1, %2, %3); }")
        .arg(red).arg(green).arg(blue);
    button->setStyleSheet(styleSheet);
}

void GridOptionsDialog::connectStateMonitoring()
{
    if (!m_gridNode) return;
    
    // Monitor all state changes that affect the UI
    std::vector<std::string> statePaths = {
        "yz_plane.visible", "yz_plane.color_r", "yz_plane.color_g", "yz_plane.color_b",
        "yz_plane.line_width", "yz_plane.opacity",
        "xz_plane.visible", "xz_plane.color_r", "xz_plane.color_g", "xz_plane.color_b",
        "xz_plane.line_width", "xz_plane.opacity",
        "xy_plane.visible", "xy_plane.color_r", "xy_plane.color_g", "xy_plane.color_b",
        "xy_plane.line_width", "xy_plane.opacity",
        "divisions_x", "divisions_y", "divisions_z",
        "tics.visible", "tics.interval_x", "tics.interval_y", "tics.interval_z",
        "tics.label_color_r", "tics.label_color_g", "tics.label_color_b",
        "tics.label_font_size"
    };
    
    for (const auto& path : statePaths) {
        auto connection = m_gridNode->getState(path).valueChanged.connect(
            [this]() { onStateChanged(); }
        );
        m_stateConnections.push_back(connection);
    }
}

void GridOptionsDialog::disconnectStateMonitoring()
{
    m_stateConnections.clear();
}

void GridOptionsDialog::onStateChanged()
{
    // Reload UI from state when external changes occur
    loadFromState();
}
