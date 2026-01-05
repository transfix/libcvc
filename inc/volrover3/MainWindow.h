#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDockWidget>
#include <QLabel>
#include <QProgressBar>
#include <memory>
#include <vector>
#include <boost/signals2.hpp>

class VTKRenderWidget;
class TransferFunctionWidget;
class SceneGraph;
class ThreadMonitorWidget;
class StateTreeWidget;
class GridOptionsDialog;
class SDFDialog;
class IsosurfaceDialog;
class GeometryDialog;
class VolumeDialog;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void openFile();
    void toggleGrid();
    void toggleAxis();
    void editBoundingBox();
    void editCameraSettings();
    void showGridOptions();
    void showThreadMonitor();
    void showStateTree();
    void showSDF();
    void showIsosurface();
    void showGeometry();
    void showVolume();
    void aboutVolRover();
    void updateThreadStatus();

protected:
    void closeEvent(QCloseEvent *event) override;

private:
    void createMenus();
    void createDockWidgets();
    void setupConnections();
    void initializeCameraFromState();
    void setupStatusBar();

    VTKRenderWidget *m_renderWidget;
    TransferFunctionWidget *m_transferFunctionWidget;
    std::shared_ptr<SceneGraph> m_sceneGraph;
    ThreadMonitorWidget *m_threadMonitor;
    StateTreeWidget *m_stateTreeWidget;
    GridOptionsDialog *m_gridOptionsDialog;
    SDFDialog *m_sdfDialog;
    IsosurfaceDialog *m_isosurfaceDialog;
    GeometryDialog *m_geometryDialog;
    VolumeDialog *m_volumeDialog;

    // Status bar widgets for thread monitoring
    QLabel *m_threadNameLabel;
    QLabel *m_threadInfoLabel;
    QProgressBar *m_threadProgressBar;
    
    std::vector<boost::signals2::connection> m_connections;

    bool m_gridVisible;
    bool m_axisVisible;
};

#endif // MAINWINDOW_H
