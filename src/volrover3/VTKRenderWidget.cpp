#include <volrover3/VTKRenderWidget.h>
#include <volrover3/SceneGraph.h>
#include <volrover3/CameraController.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkCamera.h>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QWheelEvent>

VTKRenderWidget::VTKRenderWidget(QWidget *parent)
    : QVTK_WIDGET_BASE(parent)
    , m_renderWindow(vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New())
    , m_renderer(vtkSmartPointer<vtkRenderer>::New())
    , m_cameraController(std::make_unique<CameraController>())
{
    initializeVTK();
    
    // Set up timer to process SceneGraph events on main thread
    connect(&m_eventTimer, &QTimer::timeout, this, &VTKRenderWidget::processSceneGraphEvents);
    m_eventTimer.start(16);  // ~60fps event processing
}

VTKRenderWidget::~VTKRenderWidget()
{
}

void VTKRenderWidget::initializeVTK()
{
    // Set up render window
    setRenderWindow(m_renderWindow);
    m_renderWindow->AddRenderer(m_renderer);

    // Set background color (dark gray)
    m_renderer->SetBackground(0.2, 0.2, 0.2);

    // Set up camera
    vtkCamera *camera = m_renderer->GetActiveCamera();
    camera->SetPosition(0, 0, 10);
    camera->SetFocalPoint(0, 0, 0);
    camera->SetViewUp(0, 1, 0);

    // Initialize camera controller
    m_cameraController->setCamera(camera);

    // Enable focus for keyboard input
    setFocusPolicy(Qt::StrongFocus);
}

void VTKRenderWidget::setSceneGraph(std::shared_ptr<SceneGraph> sceneGraph)
{
    m_sceneGraph = sceneGraph;
    if (m_sceneGraph) {
        m_sceneGraph->setRenderer(m_renderer);
    }
}

void VTKRenderWidget::keyPressEvent(QKeyEvent *event)
{
    if (m_cameraController) {
        m_cameraController->handleKeyPress(event->key());
        updateCamera();
        renderWindow()->Render();
    }
    QVTK_WIDGET_BASE::keyPressEvent(event);
}

void VTKRenderWidget::keyReleaseEvent(QKeyEvent *event)
{
    if (m_cameraController) {
        m_cameraController->handleKeyRelease(event->key());
    }
    QVTK_WIDGET_BASE::keyReleaseEvent(event);
}

void VTKRenderWidget::mousePressEvent(QMouseEvent *event)
{
    m_lastMousePos = event->pos();
    if (m_cameraController) {
        m_cameraController->handleMousePress(event->button());
    }
    QVTK_WIDGET_BASE::mousePressEvent(event);
}

void VTKRenderWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (m_cameraController) {
        m_cameraController->handleMouseRelease(event->button());
    }
    QVTK_WIDGET_BASE::mouseReleaseEvent(event);
}

void VTKRenderWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (m_cameraController) {
        QPoint delta = event->pos() - m_lastMousePos;
        m_cameraController->handleMouseMove(delta.x(), delta.y());
        updateCamera();
        renderWindow()->Render();
    }
    m_lastMousePos = event->pos();
    QVTK_WIDGET_BASE::mouseMoveEvent(event);
}

void VTKRenderWidget::wheelEvent(QWheelEvent *event)
{
    if (m_cameraController) {
        m_cameraController->handleMouseWheel(event->angleDelta().y());
        updateCamera();
        renderWindow()->Render();
    }
    QVTK_WIDGET_BASE::wheelEvent(event);
}

void VTKRenderWidget::updateCamera()
{
    if (m_cameraController) {
        m_cameraController->update();
    }
}

void VTKRenderWidget::resetCamera()
{
    if (m_renderer) {
        m_renderer->ResetCamera();
        renderWindow()->Render();
    }
}

void VTKRenderWidget::render()
{
    if (m_renderWindow) {
        m_renderWindow->Render();
    }
}

void VTKRenderWidget::processSceneGraphEvents()
{
    if (m_sceneGraph) {
        m_sceneGraph->processEvents();
    }
}
