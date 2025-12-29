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

BoundingBoxDialog::BoundingBoxDialog(const cvc::bounding_box& bounds, QWidget *parent)
    : QDialog(parent)
{
    setWindowTitle(tr("World Bounding Box"));
    setupUI();
    
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
    
    // Quick set buttons
    QHBoxLayout *quickSetLayout = new QHBoxLayout();
    QPushButton *resetGeomBtn = new QPushButton(tr("Fit to Geometry"));
    QPushButton *resetVolBtn = new QPushButton(tr("Fit to Volume"));
    
    connect(resetGeomBtn, &QPushButton::clicked, this, &BoundingBoxDialog::onResetToGeometry);
    connect(resetVolBtn, &QPushButton::clicked, this, &BoundingBoxDialog::onResetToVolume);
    
    quickSetLayout->addWidget(resetGeomBtn);
    quickSetLayout->addWidget(resetVolBtn);
    mainLayout->addLayout(quickSetLayout);
    
    // Dialog buttons
    QDialogButtonBox *buttonBox = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
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

void BoundingBoxDialog::onResetToGeometry()
{
    cvc::geometry geom = AppState::instance().geometry();
    if (geom.num_points() > 0) {
        cvc::bounding_box bounds = geom.extents();
        m_minXEdit->setText(QString::number(bounds[0]));
        m_minYEdit->setText(QString::number(bounds[1]));
        m_minZEdit->setText(QString::number(bounds[2]));
        m_maxXEdit->setText(QString::number(bounds[3]));
        m_maxYEdit->setText(QString::number(bounds[4]));
        m_maxZEdit->setText(QString::number(bounds[5]));
    }
}

void BoundingBoxDialog::onResetToVolume()
{
    cvc::volume vol = AppState::instance().volume();
    if (vol.XDim() > 0) {
        cvc::bounding_box bounds = vol.boundingBox();
        m_minXEdit->setText(QString::number(bounds[0]));
        m_minYEdit->setText(QString::number(bounds[1]));
        m_minZEdit->setText(QString::number(bounds[2]));
        m_maxXEdit->setText(QString::number(bounds[3]));
        m_maxYEdit->setText(QString::number(bounds[4]));
        m_maxZEdit->setText(QString::number(bounds[5]));
    }
}
