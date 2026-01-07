#ifndef GEOMETRYDIALOG_H
#define GEOMETRYDIALOG_H

#include <QDialog>
#include <memory>
#include <vector>
#include <string>
#include <boost/signals2.hpp>

class QComboBox;
class QDoubleSpinBox;
class QCheckBox;
class QGroupBox;
class QTableWidget;
class QPushButton;
class SceneGraph;

class GeometryDialog : public QDialog
{
    Q_OBJECT

public:
    explicit GeometryDialog(std::shared_ptr<SceneGraph> sceneGraph, QWidget *parent = nullptr);
    ~GeometryDialog() = default;

private slots:
    void onGeometrySelected(int index);
    void onGraphicsChildrenChanged();
    void onRenderModeChanged(int index);
    void onColorChanged();
    void onSingleColorChanged(bool checked);
    void onMaterialPropertyChanged();
    void onDeleteButtonClicked();
    void onNodeStateChanged();
    void onVisibilityChanged(bool checked);
    void onShowBBoxChanged(bool checked);
    void onBBoxColorChanged();
    void onInvertNormalsClicked();

private:
    void setupUI();
    void connectSignals();
    void populateGeometryList();
    void updatePropertiesFromNode();
    void setPropertiesEnabled(bool enabled);
    void updateBBoxColorButton();

    std::shared_ptr<SceneGraph> m_sceneGraph;
    
    // UI elements
    QComboBox *m_geometryComboBox;
    QPushButton *m_deleteButton;
    QComboBox *m_renderModeComboBox;
    
    // Color controls
    QCheckBox *m_singleColorCheckBox;
    QDoubleSpinBox *m_colorRSpinBox;
    QDoubleSpinBox *m_colorGSpinBox;
    QDoubleSpinBox *m_colorBSpinBox;
    
    // Visibility controls
    QCheckBox *m_visibilityCheckBox;
    
    // Bounding box controls
    QCheckBox *m_showBBoxCheckBox;
    QPushButton *m_bboxColorButton;
    double m_bboxColor[3];
    
    // Invert normals button
    QPushButton *m_invertNormalsButton;
    
    // Info tab
    QTableWidget *m_infoTable;
    
    // Material properties
    QDoubleSpinBox *m_ambientSpinBox;
    QDoubleSpinBox *m_diffuseSpinBox;
    QDoubleSpinBox *m_specularSpinBox;
    QDoubleSpinBox *m_specularPowerSpinBox;
    QDoubleSpinBox *m_opacitySpinBox;
    QDoubleSpinBox *m_pointSizeSpinBox;
    QDoubleSpinBox *m_lineWidthSpinBox;
    
    // Geometry tracking
    std::vector<std::string> m_geometryNames;
    
    // State tree connection
    boost::signals2::scoped_connection m_graphicsChildrenConnection;
    boost::signals2::scoped_connection m_nodeStateConnection;
    
    // Flag to prevent recursive updates
    bool m_updating;
};

#endif // GEOMETRYDIALOG_H
