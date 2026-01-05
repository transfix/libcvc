#ifndef TRANSFERFUNCTIONWIDGET_H
#define TRANSFERFUNCTIONWIDGET_H

#include <QWidget>
#include <vector>
#include <QColor>
#include <memory>
#include <boost/signals2.hpp>

class QCustomPlot;
class QCPGraph;
class QCPColorMap;
class QComboBox;
class SceneGraph;
class VolumeNode;

class TransferFunctionWidget : public QWidget
{
    Q_OBJECT

public:
    // Color control points (value, r, g, b)
    struct ColorPoint {
        double value;
        QColor color;
    };

    // Opacity control points (value, opacity)
    struct OpacityPoint {
        double value;
        double opacity;
    };

    explicit TransferFunctionWidget(QWidget *parent = nullptr);
    ~TransferFunctionWidget();

    void setDataRange(double min, double max);
    
    std::vector<double> getColorTable() const;
    std::vector<double> getOpacityTable() const;
    
    void applyPreset(const QString &presetName);
    
    // Volume selection
    void setSceneGraph(SceneGraph* sceneGraph);
    void refreshVolumeList();
    std::shared_ptr<VolumeNode> getSelectedVolume() const;

signals:
    void transferFunctionChanged();
    void selectedVolumeChanged(std::shared_ptr<VolumeNode> volume);

private slots:
    void onPresetChanged(int index);
    void onColorMapClicked(double x, double y);
    void onOpacityGraphChanged();
    void onVolumeSelected(int index);
    void onGraphicsChildrenChanged();
    void onVolumeTransferFunctionChanged();

private:
    void setupUI();
    void createDefaultTransferFunction();
    void updateColorBar();
    void loadTransferFunctionFromVolume(std::shared_ptr<VolumeNode> volume);
    void connectToVolumeState(std::shared_ptr<VolumeNode> volume);
    void disconnectFromVolumeState();

    QComboBox *m_presetCombo;
    QComboBox *m_volumeCombo;
    QWidget *m_colorBarWidget;
    QWidget *m_opacityWidget;

    double m_dataMin;
    double m_dataMax;

    std::vector<ColorPoint> m_colorPoints;
    std::vector<OpacityPoint> m_opacityPoints;
    
    SceneGraph* m_sceneGraph;
    std::vector<std::shared_ptr<VolumeNode>> m_volumes;
    
    // State tree connections
    boost::signals2::scoped_connection m_graphicsChildrenConnection;
    boost::signals2::scoped_connection m_colorTFConnection;
    boost::signals2::scoped_connection m_opacityTFConnection;
    
    // Track if we're updating from state to prevent feedback loops
    // Use counter instead of bool to handle nested/queued updates
    int m_updatingFromState = 0;
};

#endif // TRANSFERFUNCTIONWIDGET_H
