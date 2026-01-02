#include <volrover3/BoundingBoxDialog.h>
#include <volrover3/SceneGraph.h>
#include <volrover3/GraphicsNode.h>
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

BoundingBoxDialog::BoundingBoxDialog(std::shared_ptr<SceneGraph> sceneGraph, QWidget *parent)
    : QDialog(parent)
    , m_sceneGraph(sceneGraph)
    , m_currentGraphics(nullptr)
{
    setWindowTitle(tr("Bounding Box Settings"));
    m_bboxColor[0] = m_bboxColor[1] = m_bboxColor[2] = 1.0;
    setupUI();
    populateGraphicsComboBox();
    
    // Select first graphics if available
    if (m_graphicsComboBox->count() > 0) {
        m_graphicsComboBox->setCurrentIndex(0);
        onGraphicsSelectionChanged(0);
    }
}

void BoundingBoxDialog::setupUI()
{
    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    
    // Graphics selector
    QGroupBox *selectorGroup = new QGroupBox(tr("Graphic"));
    QFormLayout *selectorLayout = new QFormLayout(selectorGroup);
    
    m_graphicsComboBox = new QComboBox();
    connect(m_graphicsComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &BoundingBoxDialog::onGraphicsSelectionChanged);
    selectorLayout->addRow(tr("Select Graphic:"), m_graphicsComboBox);
    
    mainLayout->addWidget(selectorGroup);
    
    // Bounds group (read-only display of computed bounds)
    QGroupBox *boundsGroup = new QGroupBox(tr("Computed Bounds"));
    QFormLayout *formLayout = new QFormLayout(boundsGroup);
    
    m_minXEdit = new QLineEdit();
    m_minYEdit = new QLineEdit();
    m_minZEdit = new QLineEdit();
    m_maxXEdit = new QLineEdit();
    m_maxYEdit = new QLineEdit();
    m_maxZEdit = new QLineEdit();
    
    // Make read-only - bounds are computed from geometry/volume data
    m_minXEdit->setReadOnly(true);
    m_minYEdit->setReadOnly(true);
    m_minZEdit->setReadOnly(true);
    m_maxXEdit->setReadOnly(true);
    m_maxYEdit->setReadOnly(true);
    m_maxZEdit->setReadOnly(true);
    
    formLayout->addRow(tr("Min X:"), m_minXEdit);
    formLayout->addRow(tr("Min Y:"), m_minYEdit);
    formLayout->addRow(tr("Min Z:"), m_minZEdit);
    formLayout->addRow(tr("Max X:"), m_maxXEdit);
    formLayout->addRow(tr("Max Y:"), m_maxYEdit);
    formLayout->addRow(tr("Max Z:"), m_maxZEdit);
    
    mainLayout->addWidget(boundsGroup);
    
    // Bounding box rendering group
    QGroupBox *renderGroup = new QGroupBox(tr("Bounding Box Rendering"));
    QFormLayout *renderLayout = new QFormLayout(renderGroup);
    
    m_bboxVisibleCheckbox = new QCheckBox();
    connect(m_bboxVisibleCheckbox, &QCheckBox::toggled,
            this, &BoundingBoxDialog::onBBoxVisibilityChanged);
    renderLayout->addRow(tr("Show Bounding Box:"), m_bboxVisibleCheckbox);
    
    QHBoxLayout *colorLayout = new QHBoxLayout();
    m_bboxColorButton = new QPushButton();
    m_bboxColorButton->setFixedSize(50, 25);
    connect(m_bboxColorButton, &QPushButton::clicked,
            this, &BoundingBoxDialog::onBBoxColorChanged);
    colorLayout->addWidget(m_bboxColorButton);
    colorLayout->addStretch();
    renderLayout->addRow(tr("Color:"), colorLayout);
    
    mainLayout->addWidget(renderGroup);
    
    // Dialog buttons
    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Close);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    mainLayout->addWidget(buttonBox);
    
    updateColorButton();
}

void BoundingBoxDialog::populateGraphicsComboBox()
{
    m_graphicsComboBox->clear();
    m_graphicsList.clear();
    
    if (!m_sceneGraph) return;
    
    // Add root graphic first
    auto root = m_sceneGraph->getGraphicsRoot();
    if (root) {
        m_graphicsComboBox->addItem(tr("(Root - All Graphics)"));
        m_graphicsList.push_back(root);
    }
    
    // Add all other graphics
    const auto& allGraphics = m_sceneGraph->getAllGraphics();
    for (const auto& pair : allGraphics) {
        m_graphicsComboBox->addItem(QString::fromStdString(pair.first));
        m_graphicsList.push_back(pair.second);
    }
}

void BoundingBoxDialog::onGraphicsSelectionChanged(int index)
{
    if (index < 0 || index >= static_cast<int>(m_graphicsList.size())) {
        m_currentGraphics = nullptr;
        return;
    }
    
    m_currentGraphics = m_graphicsList[index];
    loadGraphicsSettings();
}

void BoundingBoxDialog::loadGraphicsSettings()
{
    if (!m_currentGraphics) {
        return;
    }
    
    // Load bounding box
    cvc::bounding_box bounds = m_currentGraphics->getBoundingBox();
    m_minXEdit->setText(QString::number(bounds[0]));
    m_minYEdit->setText(QString::number(bounds[1]));
    m_minZEdit->setText(QString::number(bounds[2]));
    m_maxXEdit->setText(QString::number(bounds[3]));
    m_maxYEdit->setText(QString::number(bounds[4]));
    m_maxZEdit->setText(QString::number(bounds[5]));
    
    // Load bbox visibility
    m_bboxVisibleCheckbox->setChecked(m_currentGraphics->getShowBBox());
    
    // Load bbox color
    m_currentGraphics->getBBoxColor(m_bboxColor[0], m_bboxColor[1], m_bboxColor[2]);
    updateColorButton();
}

void BoundingBoxDialog::onResetToGraphics()
{
    // For now, bounds are computed automatically, so this is a no-op
    // Could potentially re-compute or reload
    if (m_currentGraphics) {
        loadGraphicsSettings();
    }
}

void BoundingBoxDialog::onBBoxVisibilityChanged(bool visible)
{
    if (m_currentGraphics) {
        m_currentGraphics->setShowBBox(visible);
        onApplyChanges();
    }
}

void BoundingBoxDialog::onBBoxColorChanged()
{
    if (!m_currentGraphics) return;
    
    QColor currentColor = QColor::fromRgbF(m_bboxColor[0], m_bboxColor[1], m_bboxColor[2]);
    QColor color = QColorDialog::getColor(currentColor, this, tr("Choose Bounding Box Color"));
    
    if (color.isValid()) {
        m_bboxColor[0] = color.redF();
        m_bboxColor[1] = color.greenF();
        m_bboxColor[2] = color.blueF();
        updateColorButton();
        
        m_currentGraphics->setBBoxColor(m_bboxColor[0], m_bboxColor[1], m_bboxColor[2]);
        onApplyChanges();
    }
}

void BoundingBoxDialog::onApplyChanges()
{
    // Trigger a render update
    if (m_sceneGraph) {
        m_sceneGraph->update();
    }
}

void BoundingBoxDialog::updateColorButton()
{
    int r = static_cast<int>(m_bboxColor[0] * 255);
    int g = static_cast<int>(m_bboxColor[1] * 255);
    int b = static_cast<int>(m_bboxColor[2] * 255);
    
    QString style = QString("background-color: rgb(%1, %2, %3);").arg(r).arg(g).arg(b);
    m_bboxColorButton->setStyleSheet(style);
}
