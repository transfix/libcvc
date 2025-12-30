#ifndef GRAPHICSPARENTDIALOG_H
#define GRAPHICSPARENTDIALOG_H

#include <QDialog>
#include <memory>
#include <string>

class QComboBox;
class QPushButton;
class GraphicsNode;
class SceneGraph;

/**
 * @brief Dialog for selecting a parent graphics node for new geometry
 * 
 * Allows the user to select which graphics node should be the parent
 * for newly loaded geometry files. Shows a hierarchical list of existing
 * graphics nodes, with the root as the default option.
 */
class GraphicsParentDialog : public QDialog
{
    Q_OBJECT

public:
    explicit GraphicsParentDialog(std::shared_ptr<SceneGraph> sceneGraph, QWidget *parent = nullptr);
    ~GraphicsParentDialog() override;

    // Get the selected parent node name (empty string = root)
    std::string getSelectedParentName() const;
    
    // Get the selected parent node (nullptr = root)
    std::shared_ptr<GraphicsNode> getSelectedParent() const;

private:
    void populateParentList();
    void addNodeToList(std::shared_ptr<GraphicsNode> node, int depth = 0);

    std::shared_ptr<SceneGraph> m_sceneGraph;
    QComboBox *m_parentComboBox;
    QPushButton *m_okButton;
    QPushButton *m_cancelButton;
};

#endif // GRAPHICSPARENTDIALOG_H
