#include <volrover3/GraphicsParentDialog.h>
#include <volrover3/GraphicsNode.h>
#include <volrover3/SceneGraph.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QComboBox>
#include <QPushButton>

GraphicsParentDialog::GraphicsParentDialog(std::shared_ptr<SceneGraph> sceneGraph, QWidget *parent)
    : QDialog(parent)
    , m_sceneGraph(sceneGraph)
    , m_parentComboBox(new QComboBox(this))
    , m_okButton(new QPushButton(tr("OK"), this))
    , m_cancelButton(new QPushButton(tr("Cancel"), this))
{
    setWindowTitle(tr("Select Parent Graphics Node"));
    setModal(true);
    
    // Create layout
    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    
    // Add description label
    QLabel *descLabel = new QLabel(
        tr("Select the parent node for the new geometry.\n"
           "The new geometry will be placed under the selected node."),
        this);
    descLabel->setWordWrap(true);
    mainLayout->addWidget(descLabel);
    
    // Add combo box
    QLabel *comboLabel = new QLabel(tr("Parent Node:"), this);
    mainLayout->addWidget(comboLabel);
    mainLayout->addWidget(m_parentComboBox);
    
    // Populate the list
    populateParentList();
    
    // Add buttons
    QHBoxLayout *buttonLayout = new QHBoxLayout();
    buttonLayout->addStretch();
    buttonLayout->addWidget(m_okButton);
    buttonLayout->addWidget(m_cancelButton);
    mainLayout->addLayout(buttonLayout);
    
    // Connect signals
    connect(m_okButton, &QPushButton::clicked, this, &QDialog::accept);
    connect(m_cancelButton, &QPushButton::clicked, this, &QDialog::reject);
    
    resize(400, 200);
}

GraphicsParentDialog::~GraphicsParentDialog()
{
}

void GraphicsParentDialog::populateParentList()
{
    m_parentComboBox->clear();
    
    // Add root option (empty parent)
    m_parentComboBox->addItem(tr("(Root - No Parent)"), QVariant(QString("")));
    
    // Add all graphics nodes hierarchically
    auto graphicsRoot = m_sceneGraph->getGraphicsRoot();
    if (graphicsRoot) {
        for (const auto& child : graphicsRoot->getGraphicsChildren()) {
            addNodeToList(child, 0);
        }
    }
    
    // Select root by default
    m_parentComboBox->setCurrentIndex(0);
}

void GraphicsParentDialog::addNodeToList(std::shared_ptr<GraphicsNode> node, int depth)
{
    if (!node) return;
    
    // Create indented display name
    QString indent(depth * 2, ' ');
    QString displayName = indent + QString::fromStdString(node->getName());
    
    // Add to combo box with node name as data
    m_parentComboBox->addItem(displayName, QVariant(QString::fromStdString(node->getName())));
    
    // Recursively add children
    for (const auto& child : node->getGraphicsChildren()) {
        addNodeToList(child, depth + 1);
    }
}

std::string GraphicsParentDialog::getSelectedParentName() const
{
    QString name = m_parentComboBox->currentData().toString();
    return name.toStdString();
}

std::shared_ptr<GraphicsNode> GraphicsParentDialog::getSelectedParent() const
{
    std::string parentName = getSelectedParentName();
    if (parentName.empty()) {
        return nullptr; // Root
    }
    return m_sceneGraph->getGraphics(parentName);
}
