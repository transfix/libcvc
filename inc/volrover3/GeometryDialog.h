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
    void onMaterialPropertyChanged();

private:
    void setupUI();
    void connectSignals();
    void populateGeometryList();
    void updatePropertiesFromNode();
    void setPropertiesEnabled(bool enabled);

    std::shared_ptr<SceneGraph> m_sceneGraph;
    
    // UI elements
    QComboBox *m_geometryComboBox;
    QComboBox *m_renderModeComboBox;
    
    // Color controls
    QDoubleSpinBox *m_colorRSpinBox;
    QDoubleSpinBox *m_colorGSpinBox;
    QDoubleSpinBox *m_colorBSpinBox;
    
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
    
    // Flag to prevent recursive updates
    bool m_updating;
};

#endif // GEOMETRYDIALOG_H
