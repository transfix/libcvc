#ifndef VOLUMEDIALOG_H
#define VOLUMEDIALOG_H

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

class VolumeDialog : public QDialog
{
    Q_OBJECT

public:
    explicit VolumeDialog(std::shared_ptr<SceneGraph> sceneGraph, QWidget *parent = nullptr);
    ~VolumeDialog() = default;

private slots:
    void onVolumeSelected(int index);
    void onGraphicsChildrenChanged();
    void onRenderingPropertyChanged();

private:
    void setupUI();
    void connectSignals();
    void populateVolumeList();
    void updatePropertiesFromNode();
    void setPropertiesEnabled(bool enabled);

    std::shared_ptr<SceneGraph> m_sceneGraph;
    
    // UI elements
    QComboBox *m_volumeComboBox;
    
    // Rendering properties
    QCheckBox *m_shadingCheckBox;
    QDoubleSpinBox *m_ambientSpinBox;
    QDoubleSpinBox *m_diffuseSpinBox;
    QDoubleSpinBox *m_specularSpinBox;
    QDoubleSpinBox *m_specularPowerSpinBox;
    QDoubleSpinBox *m_scalarOpacityUnitDistanceSpinBox;
    QDoubleSpinBox *m_sampleDistanceSpinBox;
    QCheckBox *m_autoAdjustSampleDistancesCheckBox;
    
    // Volume tracking
    std::vector<std::string> m_volumePaths;  // Full state tree paths
    
    // State tree connection
    boost::signals2::scoped_connection m_graphicsChildrenConnection;
    
    // Flag to prevent recursive updates
    bool m_updating;
};

#endif // VOLUMEDIALOG_H
