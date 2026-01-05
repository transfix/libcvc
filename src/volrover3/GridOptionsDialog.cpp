#include <volrover3/GridOptionsDialog.h>
#include <volrover3/GridNode.h>
#include <cvc/state.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QCheckBox>
#include <QSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QDialogButtonBox>
#include <QColorDialog>
#include <QShowEvent>

GridOptionsDialog::GridOptionsDialog(std::shared_ptr<GridNode> gridNode, QWidget *parent)
    : QDialog(parent)
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
{
    setWindowTitle(tr("Grid Options"));
    setMinimumWidth(400);
    setupUI();
    connectSignals();
    loadFromState();
}

void GridOptionsDialog::showEvent(QShowEvent *event)
{
    QDialog::showEvent(event);
    // Reload state from GridNode when dialog is shown to ensure sync
    loadFromState();
}

void GridOptionsDialog::setupUI()
{
    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    
    // Grid Plane Visibility Group
    QGroupBox *visibilityGroup = new QGroupBox(tr("Grid Plane Visibility"), this);
    QVBoxLayout *visLayout = new QVBoxLayout(visibilityGroup);
    
    m_yzPlaneCheckBox = new QCheckBox(tr("YZ Plane (X = 0)"), this);
    m_xzPlaneCheckBox = new QCheckBox(tr("XZ Plane (Y = 0)"), this);
    m_xyPlaneCheckBox = new QCheckBox(tr("XY Plane (Z = 0)"), this);
    
    visLayout->addWidget(m_yzPlaneCheckBox);
    visLayout->addWidget(m_xzPlaneCheckBox);
    visLayout->addWidget(m_xyPlaneCheckBox);
    
    mainLayout->addWidget(visibilityGroup);
    
    // Grid Divisions Group
    QGroupBox *divisionsGroup = new QGroupBox(tr("Grid Divisions"), this);
    QVBoxLayout *divLayout = new QVBoxLayout(divisionsGroup);
    
    QHBoxLayout *xDivLayout = new QHBoxLayout();
    xDivLayout->addWidget(new QLabel(tr("X Divisions:"), this));
    m_xDivisionsSpinBox = new QSpinBox(this);
    m_xDivisionsSpinBox->setRange(1, 512);
    m_xDivisionsSpinBox->setValue(64);
    xDivLayout->addWidget(m_xDivisionsSpinBox);
    xDivLayout->addStretch();
    divLayout->addLayout(xDivLayout);
    
    QHBoxLayout *yDivLayout = new QHBoxLayout();
    yDivLayout->addWidget(new QLabel(tr("Y Divisions:"), this));
    m_yDivisionsSpinBox = new QSpinBox(this);
    m_yDivisionsSpinBox->setRange(1, 512);
    m_yDivisionsSpinBox->setValue(64);
    yDivLayout->addWidget(m_yDivisionsSpinBox);
    yDivLayout->addStretch();
    divLayout->addLayout(yDivLayout);
    
    QHBoxLayout *zDivLayout = new QHBoxLayout();
    zDivLayout->addWidget(new QLabel(tr("Z Divisions:"), this));
    m_zDivisionsSpinBox = new QSpinBox(this);
    m_zDivisionsSpinBox->setRange(1, 512);
    m_zDivisionsSpinBox->setValue(64);
    zDivLayout->addWidget(m_zDivisionsSpinBox);
    zDivLayout->addStretch();
    divLayout->addLayout(zDivLayout);
    
    mainLayout->addWidget(divisionsGroup);
    
    // Tick Intervals Group
    QGroupBox *tickGroup = new QGroupBox(tr("Tick Intervals"), this);
    QVBoxLayout *tickLayout = new QVBoxLayout(tickGroup);
    
    // Show ticks checkbox
    QHBoxLayout *showTicksLayout = new QHBoxLayout();
    showTicksLayout->addWidget(new QLabel(tr("Show Ticks:"), this));
    m_showTicksCheckBox = new QCheckBox(this);
    showTicksLayout->addWidget(m_showTicksCheckBox);
    showTicksLayout->addStretch();
    tickLayout->addLayout(showTicksLayout);
    
    QHBoxLayout *xTickLayout = new QHBoxLayout();
    xTickLayout->addWidget(new QLabel(tr("X Tick Interval:"), this));
    m_xTickIntervalSpinBox = new QSpinBox(this);
    m_xTickIntervalSpinBox->setRange(1, 256);
    m_xTickIntervalSpinBox->setValue(8);
    xTickLayout->addWidget(m_xTickIntervalSpinBox);
    xTickLayout->addStretch();
    tickLayout->addLayout(xTickLayout);
    
    QHBoxLayout *yTickLayout = new QHBoxLayout();
    yTickLayout->addWidget(new QLabel(tr("Y Tick Interval:"), this));
    m_yTickIntervalSpinBox = new QSpinBox(this);
    m_yTickIntervalSpinBox->setRange(1, 256);
    m_yTickIntervalSpinBox->setValue(8);
    yTickLayout->addWidget(m_yTickIntervalSpinBox);
    yTickLayout->addStretch();
    tickLayout->addLayout(yTickLayout);
    
    QHBoxLayout *zTickLayout = new QHBoxLayout();
    zTickLayout->addWidget(new QLabel(tr("Z Tick Interval:"), this));
    m_zTickIntervalSpinBox = new QSpinBox(this);
    m_zTickIntervalSpinBox->setRange(1, 256);
    m_zTickIntervalSpinBox->setValue(8);
    zTickLayout->addWidget(m_zTickIntervalSpinBox);
    zTickLayout->addStretch();
    tickLayout->addLayout(zTickLayout);
    
    mainLayout->addWidget(tickGroup);
    
    // Plane Colors Group
    QGroupBox *colorsGroup = new QGroupBox(tr("Plane Colors"), this);
    QVBoxLayout *colorsLayout = new QVBoxLayout(colorsGroup);
    
    QHBoxLayout *yzColorLayout = new QHBoxLayout();
    yzColorLayout->addWidget(new QLabel(tr("YZ Plane Color:"), this));
    m_yzPlaneColorButton = new QPushButton(tr("Choose..."), this);
    m_yzPlaneColorButton->setMinimumWidth(100);
    yzColorLayout->addWidget(m_yzPlaneColorButton);
    yzColorLayout->addStretch();
    colorsLayout->addLayout(yzColorLayout);
    
    QHBoxLayout *xzColorLayout = new QHBoxLayout();
    xzColorLayout->addWidget(new QLabel(tr("XZ Plane Color:"), this));
    m_xzPlaneColorButton = new QPushButton(tr("Choose..."), this);
    m_xzPlaneColorButton->setMinimumWidth(100);
    xzColorLayout->addWidget(m_xzPlaneColorButton);
    xzColorLayout->addStretch();
    colorsLayout->addLayout(xzColorLayout);
    
    QHBoxLayout *xyColorLayout = new QHBoxLayout();
    xyColorLayout->addWidget(new QLabel(tr("XY Plane Color:"), this));
    m_xyPlaneColorButton = new QPushButton(tr("Choose..."), this);
    m_xyPlaneColorButton->setMinimumWidth(100);
    xyColorLayout->addWidget(m_xyPlaneColorButton);
    xyColorLayout->addStretch();
    colorsLayout->addLayout(xyColorLayout);
    
    mainLayout->addWidget(colorsGroup);
    
    // Tick Label Properties Group
    QGroupBox *labelGroup = new QGroupBox(tr("Tick Label Properties"), this);
    QVBoxLayout *labelLayout = new QVBoxLayout(labelGroup);
    
    QHBoxLayout *labelColorLayout = new QHBoxLayout();
    labelColorLayout->addWidget(new QLabel(tr("Label Color:"), this));
    m_tickLabelColorButton = new QPushButton(tr("Choose..."), this);
    m_tickLabelColorButton->setMinimumWidth(100);
    labelColorLayout->addWidget(m_tickLabelColorButton);
    labelColorLayout->addStretch();
    labelLayout->addLayout(labelColorLayout);
    
    QHBoxLayout *fontSizeLayout = new QHBoxLayout();
    fontSizeLayout->addWidget(new QLabel(tr("Font Size:"), this));
    m_tickLabelFontSizeSpinBox = new QSpinBox(this);
    m_tickLabelFontSizeSpinBox->setRange(6, 72);
    m_tickLabelFontSizeSpinBox->setValue(12);
    fontSizeLayout->addWidget(m_tickLabelFontSizeSpinBox);
    fontSizeLayout->addStretch();
    labelLayout->addLayout(fontSizeLayout);
    
    mainLayout->addWidget(labelGroup);
    
    // Button box
    QDialogButtonBox *buttonBox = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel | QDialogButtonBox::Apply,
        this);
    mainLayout->addWidget(buttonBox);
    
    connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(buttonBox->button(QDialogButtonBox::Apply), &QPushButton::clicked,
            this, &GridOptionsDialog::applyChanges);
    
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
    
    // Color picker buttons
    connect(m_yzPlaneColorButton, &QPushButton::clicked, this, &GridOptionsDialog::chooseYZPlaneColor);
    connect(m_xzPlaneColorButton, &QPushButton::clicked, this, &GridOptionsDialog::chooseXZPlaneColor);
    connect(m_xyPlaneColorButton, &QPushButton::clicked, this, &GridOptionsDialog::chooseXYPlaneColor);
    connect(m_tickLabelColorButton, &QPushButton::clicked, this, &GridOptionsDialog::chooseTickLabelColor);
}

void GridOptionsDialog::loadFromState()
{
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
    
    if (!m_gridNode) return;
    
    // Load visibility from GridNode state
    m_yzPlaneCheckBox->setChecked(m_gridNode->getState("yz_plane_visible").value<bool>());
    m_xzPlaneCheckBox->setChecked(m_gridNode->getState("xz_plane_visible").value<bool>());
    m_xyPlaneCheckBox->setChecked(m_gridNode->getState("xy_plane_visible").value<bool>());
    
    // Load divisions from GridNode state
    int x = m_gridNode->getState("divisions_x").value<int>();
    int y = m_gridNode->getState("divisions_y").value<int>();
    int z = m_gridNode->getState("divisions_z").value<int>();
    m_xDivisionsSpinBox->setValue(x);
    m_yDivisionsSpinBox->setValue(y);
    m_zDivisionsSpinBox->setValue(z);
    
    // Load tick intervals from GridNode state
    m_showTicksCheckBox->setChecked(m_gridNode->getState("ticks_visible").value<bool>());
    x = m_gridNode->getState("tick_interval_x").value<int>();
    y = m_gridNode->getState("tick_interval_y").value<int>();
    z = m_gridNode->getState("tick_interval_z").value<int>();
    m_xTickIntervalSpinBox->setValue(x);
    m_yTickIntervalSpinBox->setValue(y);
    m_zTickIntervalSpinBox->setValue(z);
    
    // Load colors from GridNode state
    m_yzPlaneColor[0] = m_gridNode->getState("yz_plane_color_r").value<double>();
    m_yzPlaneColor[1] = m_gridNode->getState("yz_plane_color_g").value<double>();
    m_yzPlaneColor[2] = m_gridNode->getState("yz_plane_color_b").value<double>();
    
    m_xzPlaneColor[0] = m_gridNode->getState("xz_plane_color_r").value<double>();
    m_xzPlaneColor[1] = m_gridNode->getState("xz_plane_color_g").value<double>();
    m_xzPlaneColor[2] = m_gridNode->getState("xz_plane_color_b").value<double>();
    
    m_xyPlaneColor[0] = m_gridNode->getState("xy_plane_color_r").value<double>();
    m_xyPlaneColor[1] = m_gridNode->getState("xy_plane_color_g").value<double>();
    m_xyPlaneColor[2] = m_gridNode->getState("xy_plane_color_b").value<double>();
    
    m_tickLabelColor[0] = m_gridNode->getState("tick_label_color_r").value<double>();
    m_tickLabelColor[1] = m_gridNode->getState("tick_label_color_g").value<double>();
    m_tickLabelColor[2] = m_gridNode->getState("tick_label_color_b").value<double>();
    
    updateColorButton(m_yzPlaneColorButton, m_yzPlaneColor[0], m_yzPlaneColor[1], m_yzPlaneColor[2]);
    updateColorButton(m_xzPlaneColorButton, m_xzPlaneColor[0], m_xzPlaneColor[1], m_xzPlaneColor[2]);
    updateColorButton(m_xyPlaneColorButton, m_xyPlaneColor[0], m_xyPlaneColor[1], m_xyPlaneColor[2]);
    updateColorButton(m_tickLabelColorButton, m_tickLabelColor[0], m_tickLabelColor[1], m_tickLabelColor[2]);
    
    // Load font size from GridNode state
    m_tickLabelFontSizeSpinBox->setValue(m_gridNode->getState("tick_label_font_size").value<int>());
    
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
}

void GridOptionsDialog::applyChanges()
{
    if (!m_gridNode) return;
    
    // Apply visibility changes to GridNode state
    m_gridNode->getState("yz_plane_visible").value(m_yzPlaneCheckBox->isChecked());
    m_gridNode->getState("xz_plane_visible").value(m_xzPlaneCheckBox->isChecked());
    m_gridNode->getState("xy_plane_visible").value(m_xyPlaneCheckBox->isChecked());
    
    // Apply division changes to GridNode state
    m_gridNode->getState("divisions_x").value(m_xDivisionsSpinBox->value());
    m_gridNode->getState("divisions_y").value(m_yDivisionsSpinBox->value());
    m_gridNode->getState("divisions_z").value(m_zDivisionsSpinBox->value());
    
    // Apply tick interval changes to GridNode state
    m_gridNode->getState("ticks_visible").value(m_showTicksCheckBox->isChecked());
    m_gridNode->getState("tick_interval_x").value(m_xTickIntervalSpinBox->value());
    m_gridNode->getState("tick_interval_y").value(m_yTickIntervalSpinBox->value());
    m_gridNode->getState("tick_interval_z").value(m_zTickIntervalSpinBox->value());
    
    // Apply color changes to GridNode state
    m_gridNode->getState("yz_plane_color_r").value(m_yzPlaneColor[0]);
    m_gridNode->getState("yz_plane_color_g").value(m_yzPlaneColor[1]);
    m_gridNode->getState("yz_plane_color_b").value(m_yzPlaneColor[2]);
    
    m_gridNode->getState("xz_plane_color_r").value(m_xzPlaneColor[0]);
    m_gridNode->getState("xz_plane_color_g").value(m_xzPlaneColor[1]);
    m_gridNode->getState("xz_plane_color_b").value(m_xzPlaneColor[2]);
    
    m_gridNode->getState("xy_plane_color_r").value(m_xyPlaneColor[0]);
    m_gridNode->getState("xy_plane_color_g").value(m_xyPlaneColor[1]);
    m_gridNode->getState("xy_plane_color_b").value(m_xyPlaneColor[2]);
    
    m_gridNode->getState("tick_label_color_r").value(m_tickLabelColor[0]);
    m_gridNode->getState("tick_label_color_g").value(m_tickLabelColor[1]);
    m_gridNode->getState("tick_label_color_b").value(m_tickLabelColor[2]);
    
    // Apply font size to GridNode state
    m_gridNode->getState("tick_label_font_size").value(m_tickLabelFontSizeSpinBox->value());
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
