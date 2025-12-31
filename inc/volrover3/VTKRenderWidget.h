#ifndef VTKRENDERWIDGET_H
#define VTKRENDERWIDGET_H

#include <QWidget>
#include <vtkSmartPointer.h>
#include <memory>

// Qt5/Qt6 compatibility
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
  #include <QVTKOpenGLNativeWidget.h>
  #define QVTK_WIDGET_BASE QVTKOpenGLNativeWidget
#else
  #include <QVTKOpenGLWidget.h>
  #define QVTK_WIDGET_BASE QVTKOpenGLWidget
#endif

class vtkRenderer;
class vtkRenderWindow;
class vtkGenericOpenGLRenderWindow;
class SceneGraph;
class CameraController;

class VTKRenderWidget : public QVTK_WIDGET_BASE
{
    Q_OBJECT

public:
    explicit VTKRenderWidget(QWidget *parent = nullptr);
    ~VTKRenderWidget();

    void setSceneGraph(std::shared_ptr<SceneGraph> sceneGraph);
    void resetCamera();
    void render();  // Force an immediate render
    
    CameraController* getCameraController() { return m_cameraController.get(); }

protected:
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

private:
    void initializeVTK();
    void updateCamera();

    vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renderWindow;
    vtkSmartPointer<vtkRenderer> m_renderer;
    std::shared_ptr<SceneGraph> m_sceneGraph;
    std::unique_ptr<CameraController> m_cameraController;
    
    QPoint m_lastMousePos;
};

#endif // VTKRENDERWIDGET_H
