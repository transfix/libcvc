#include <volrover3/BoundingBoxDialog.h>
#include <volrover3/AppState.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QDialogButtonBox>
#include <QLabel>
#include <QDoubleValidator>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QColorDialog>

BoundingBoxDialog::BoundingBoxDialog(const cvc::bounding_box& bounds, QWidget *parent)
    : QDialog(parent)
{
    setWindowTitle(tr("World Bounding Box"));
    m_tickLabelColor[0] = m_tickLabelColor[1] = m_tickLabelColor[2] = 1.0;
    setupUI();
    loadTickSettings();
    
    // Set initial values
    m_minXEdit->setText(QString::number(bounds[0]));
    m_minYEdit->setText(QString::number(bounds[1]));
    m_minZEdit->setText(QString::number(bounds[2]));
    m_maxXEdit->setText(QString::number(bounds[3]));
    m_maxYEdit->setText(QString::number(bounds[4]));
    m_maxZEdit->setText(QString::number(bounds[5]));
}

void BoundingBoxDialog::setupUI()
{
    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    
    // Info label
    QLabel *infoLabel = new QLabel(
        tr("The world bounding box defines the coordinate system and grid extents. "
           "It automatically expands to contain all loaded geometry and volumes."));
    infoLabel->setWordWrap(true);
    mainLayout->addWidget(infoLabel);
    
    // Bounds group
    QGroupBox *boundsGroup = new QGroupBox(tr("Bounds"));
    QFormLayout *formLayout = new QFormLayout(boundsGroup);
    
    m_minXEdit = new QLineEdit();
    m_minYEdit = new QLineEdit();
    m_minZEdit = new QLineEdit();
    m_maxXEdit = new QLineEdit();
    m_maxYEdit = new QLineEdit();
    m_maxZEdit = new QLineEdit();
    
    QDoubleValidator *validator = new QDoubleValidator(this);
    m_minXEdit->setValidator(validator);
    m_minYEdit->setValidator(validator);
    m_minZEdit->setValidator(validator);
    m_maxXEdit->setValidator(validator);
    m_maxYEdit->setValidator(validator);
    m_maxZEdit->setValidator(validator);
    
    formLayout->addRow(tr("Min X:"), m_minXEdit);
    formLayout->addRow(tr("Min Y:"), m_minYEdit);
    formLayout->addRow(tr("Min Z:"), m_minZEdit);
    formLayout->addRow(tr("Max X:"), m_maxXEdit);
    formLayout->addRow(tr("Max Y:"), m_maxYEdit);
    formLayout->addRow(tr("Max Z:"), m_maxZEdit);
    
    mainLayout->addWidget(boundsGroup);
    
    // Quick set button
    QHBoxLayout *quickSetLayout = new QHBoxLayout();
    QPushButton *resetGraphicsBtn = new QPushButton(tr("Fit to Graphics"));
    
    connect(resetGraphicsBtn, &QPushButton::clicked, this, &BoundingBoxDialog::onResetToGraphics);
    
    quickSetLayout->addWidget(resetGraphicsBtn);
    quickSetLayout->addStretch();
    mainLayout->addLayout(quickSetLayout);
    
    // World bbox visibility group
    QGroupBox *visibilityGroup = new QGroupBox(tr("Visibility"));
    QFormLayout *visibilityLayout = new QFormLayout(visibilityGroup);
    
    m_worldBBoxVisibleCheckbox = new QCheckBox();
    visibilityLayout->addRow(tr("Show World Bounding Box:"), m_worldBBoxVisibleCheckbox);
    
    mainLayout->addWidget(visibilityGroup);
    
    // Coordinate settings group
    QGroupBox *tickGroup = new QGroupBox(tr("World Bounding Box Coordinates"));
    QFormLayout *tickLayout = new QFormLayout(tickGroup);
    
    m_showTicksCheckbox = new QCheckBox();
    tickLayout->addRow(tr("Show Coordinates:"), m_showTicksCheckbox);
    
    m_tickLabelColorButton = new QPushButton();
    m_tickLabelColorButton->setFixedSize(50, 25);
    connect(m_tickLabelColorButton, &QPushButton::clicked, this, &BoundingBoxDialog::chooseTickLabelColor);
    tickLayout->addRow(tr("Coordinate Color:"), m_tickLabelColorButton);
    
    m_tickLabelFontSizeSpinBox = new QSpinBox();
    m_tickLabelFontSizeSpinBox->setRange(6, 72);
    tickLayout->addRow(tr("Font Size:"), m_tickLabelFontSizeSpinBox);
    
    mainLayout->addWidget(tickGroup);
    
    // Dialog buttons
    QDialogButtonBox *buttonBox = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttonBox, &QDialogButtonBox::accepted, this, [this]() {
        saveTickSettings();
        accept();
    });
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    mainLayout->addWidget(buttonBox);
}

cvc::bounding_box BoundingBoxDialog::getBoundingBox() const
{
    return cvc::bounding_box(
        m_minXEdit->text().toDouble(),
        m_minYEdit->text().toDouble(),
        m_minZEdit->text().toDouble(),
        m_maxXEdit->text().toDouble(),
        m_maxYEdit->text().toDouble(),
        m_maxZEdit->text().toDouble()
    );
}

void BoundingBoxDialog::onResetToGraphics()
{
    // Get transformed bounds of all graphics objects
    cvc::bounding_box bounds = AppState::instance().computeGraphicsBounds();
    
    // Check if bounds are valid (max > min for all dimensions)
    if (bounds[3] > bounds[0] && bounds[4] > bounds[1] && bounds[5] > bounds[2]) {
        m_minXEdit->setText(QString::number(bounds[0]));
        m_minYEdit->setText(QString::number(bounds[1]));
        m_minZEdit->setText(QString::number(bounds[2]));
        m_maxXEdit->setText(QString::number(bounds[3]));
        m_maxYEdit->setText(QString::number(bounds[4]));
        m_maxZEdit->setText(QString::number(bounds[5]));
    }
}

void BoundingBoxDialog::loadTickSettings()
{
    m_worldBBoxVisibleCheckbox->setChecked(AppState::instance().worldBBoxVisible());
    m_showTicksCheckbox->setChecked(AppState::instance().worldBBoxCoordinatesVisible());
    m_tickLabelFontSizeSpinBox->setValue(AppState::instance().worldBBoxCoordinateFontSize());
    
    AppState::instance().getWorldBBoxCoordinateColor(
        m_tickLabelColor[0], m_tickLabelColor[1], m_tickLabelColor[2]);
    updateColorButton();
}

void BoundingBoxDialog::saveTickSettings()
{
    AppState::instance().setWorldBBoxVisible(m_worldBBoxVisibleCheckbox->isChecked());
    AppState::instance().setWorldBBoxCoordinatesVisible(m_showTicksCheckbox->isChecked());
    AppState::instance().setWorldBBoxCoordinateFontSize(m_tickLabelFontSizeSpinBox->value());
    AppState::instance().setWorldBBoxCoordinateColor(
        m_tickLabelColor[0], m_tickLabelColor[1], m_tickLabelColor[2]);
}

void BoundingBoxDialog::chooseTickLabelColor()
{
    QColor currentColor = QColor::fromRgbF(m_tickLabelColor[0], m_tickLabelColor[1], m_tickLabelColor[2]);
    QColor color = QColorDialog::getColor(currentColor, this, tr("Choose Coordinate Label Color"));
    
    if (color.isValid()) {
        m_tickLabelColor[0] = color.redF();
        m_tickLabelColor[1] = color.greenF();
        m_tickLabelColor[2] = color.blueF();
        updateColorButton();
    }
}

void BoundingBoxDialog::updateColorButton()
{
    int r = static_cast<int>(m_tickLabelColor[0] * 255);
    int g = static_cast<int>(m_tickLabelColor[1] * 255);
    int b = static_cast<int>(m_tickLabelColor[2] * 255);
    
    QString styleSheet = QString("background-color: rgb(%1, %2, %3);").arg(r).arg(g).arg(b);
    m_tickLabelColorButton->setStyleSheet(styleSheet);
}
