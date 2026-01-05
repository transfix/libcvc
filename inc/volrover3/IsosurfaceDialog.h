#ifndef ISOSURFACEDIALOG_H
#define ISOSURFACEDIALOG_H

#include <QDialog>
#include <memory>
#include <vector>
#include <string>
#include <boost/signals2.hpp>

class QComboBox;
class QDoubleSpinBox;
class QSpinBox;
class QCheckBox;
class QPushButton;
class QProgressBar;
class QLabel;
class SceneGraph;

class IsosurfaceDialog : public QDialog
{
    Q_OBJECT

public:
    explicit IsosurfaceDialog(std::shared_ptr<SceneGraph> sceneGraph, QWidget *parent = nullptr);
    ~IsosurfaceDialog() = default;

private slots:
    void onVolumeSelected(int index);
    void onComputeClicked();
    void onGraphicsChildrenChanged();

private:
    void setupUI();
    void connectSignals();
    void populateVolumeList();
    void updateProgress(int value);
    void onComputeFinished(bool success, const std::string& message);
    void setControlsEnabled(bool enabled);

    std::shared_ptr<SceneGraph> m_sceneGraph;
    
    // UI elements
    QComboBox *m_volumeComboBox;
    QDoubleSpinBox *m_isovalueSpinBox;
    QComboBox *m_methodComboBox;
    QSpinBox *m_improveIterationsSpinBox;
    QComboBox *m_normalTypeComboBox;
    QPushButton *m_computeButton;
    QPushButton *m_cancelButton;
    QProgressBar *m_progressBar;
    QLabel *m_statusLabel;
    
    // Volume tracking
    std::vector<std::string> m_volumePaths;  // Full state tree paths
    
    // Computation state
    bool m_computing;
    std::string m_activeThreadKey;
    
    // State tree connection
    boost::signals2::scoped_connection m_graphicsChildrenConnection;
};

#endif // ISOSURFACEDIALOG_H
