#ifndef GEOMETRYDIALOG_H
#define GEOMETRYDIALOG_H

#include <QDialog>
#include <memory>
#include <vector>
#include <string>
#include <boost/signals2.hpp>

class QComboBox;
class QDoubleSpinBox;
class QSpinBox;
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
    void onShowExtentLabelsChanged(bool checked);
    void onExtentLabelColorChanged();
    void onExtentLabelFontSizeChanged(int size);
    void onInvertNormalsClicked();
    void onReorientClicked();
    void onProjectClicked();
    void onSmoothingClicked();
    void onQualityImproveClicked();

private:
    void setupUI();
    void connectSignals();
    void populateGeometryList();
    void updatePropertiesFromNode();
    void setPropertiesEnabled(bool enabled);
    void updateBBoxColorButton();
    void updateExtentLabelColorButton();
    void setOperationButtonsEnabled(bool enabled);

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
    
    // Extent label controls
    QCheckBox *m_showExtentLabelsCheckBox;
    QPushButton *m_extentLabelColorButton;
    QSpinBox *m_extentLabelFontSizeSpinBox;
    double m_extentLabelColor[3];
    
    // Geometry operations buttons
    QPushButton *m_invertNormalsButton;
    QPushButton *m_reorientButton;
    QPushButton *m_projectButton;
    QComboBox *m_projectTargetComboBox;
    QPushButton *m_smoothingButton;
    QDoubleSpinBox *m_smoothingDeltaSpinBox;
    QCheckBox *m_smoothingFixBoundaryCheckBox;
    QCheckBox *m_smoothingPerturb1CheckBox;
    QCheckBox *m_smoothingGeoFlowCheckBox;
    QCheckBox *m_smoothingEnabledCheckBox;
    QCheckBox *m_smoothingPerturb2CheckBox;
    QPushButton *m_qualityImproveButton;
    QSpinBox *m_qualityIterationsSpinBox;
    QComboBox *m_qualityMethodComboBox;
    
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
    
    // Signal connections
    boost::signals2::scoped_connection m_graphicsChangedConnection;
    boost::signals2::scoped_connection m_nodeStateConnection;
    
    // Flag to prevent recursive updates
    bool m_updating;
};

#endif // GEOMETRYDIALOG_H
