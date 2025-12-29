#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDockWidget>
#include <memory>

class VTKRenderWidget;
class TransferFunctionWidget;
class SceneGraph;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void openGeometry();
    void openVolume();
    void toggleGrid();
    void toggleAxis();
    void toggleGeometryBBox();
    void toggleVolumeBBox();
    void editBoundingBox();
    void editCameraSettings();
    void aboutVolRover();

private:
    void createMenus();
    void createDockWidgets();
    void setupConnections();
    void initializeCameraFromState();

    VTKRenderWidget *m_renderWidget;
    TransferFunctionWidget *m_transferFunctionWidget;
    std::shared_ptr<SceneGraph> m_sceneGraph;

    bool m_gridVisible;
    bool m_axisVisible;
};

#endif // MAINWINDOW_H
