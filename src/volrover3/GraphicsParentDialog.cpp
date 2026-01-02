#include <volrover3/GraphicsParentDialog.h>
#include <volrover3/GraphicsNode.h>
#include <volrover3/VolumeNode.h>
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
        tr("Select the parent node for the new graphics object.\n"
           "The new object will be placed under the selected node."),
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
    
    // Add all graphics nodes hierarchically (includes both geometry and volumes)
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
    
    // Determine if this is a volume or geometry node
    bool isVolume = (std::dynamic_pointer_cast<VolumeNode>(node) != nullptr);
    
    // Create indented display name with icon
    QString indent(depth * 2, ' ');
    QString icon = isVolume ? "🔲" : "📦";
    QString type = isVolume ? "Volume" : "Geometry";
    QString displayName = indent + icon + " " + type + ": " + QString::fromStdString(node->getName());
    
    // Add to combo box with node name as data (prefixed with type)
    QString prefix = isVolume ? "vol:" : "geom:";
    m_parentComboBox->addItem(displayName, QVariant(prefix + QString::fromStdString(node->getName())));
    
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
    
    // Check if it's a geometry node (prefixed with "geom:")
    if (parentName.substr(0, 5) == "geom:") {
        return m_sceneGraph->getGraphics(parentName.substr(5));
    }
    
    // Check if it's a volume node (prefixed with "vol:")
    if (parentName.substr(0, 4) == "vol:") {
        // Return the volume node as a GraphicsNode (volumes can parent both geometry and volumes)
        return std::dynamic_pointer_cast<VolumeNode>(m_sceneGraph->getGraphics(parentName.substr(4)));
    }
    
    return nullptr;
}

std::shared_ptr<VolumeNode> GraphicsParentDialog::getSelectedVolumeParent() const
{
    std::string parentName = getSelectedParentName();
    if (parentName.empty()) {
        return nullptr; // Root
    }
    
    // Check if it's a volume node (prefixed with "vol:")
    if (parentName.substr(0, 4) == "vol:") {
        return std::dynamic_pointer_cast<VolumeNode>(m_sceneGraph->getGraphics(parentName.substr(4)));
    }
    
    // Check if it's a geometry node (prefixed with "geom:") - volumes can be children of geometry
    if (parentName.substr(0, 5) == "geom:") {
        auto geomNode = m_sceneGraph->getGraphics(parentName.substr(5));
        // Try to cast to VolumeNode (in case geometry node is actually a volume)
        return std::dynamic_pointer_cast<VolumeNode>(geomNode);
    }
    
    return nullptr;
}
