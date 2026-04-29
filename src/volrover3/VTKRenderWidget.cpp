#include <volrover3/VTKRenderWidget.h>
#include <volrover3/SceneGraph.h>
#include <volrover3/CameraController.h>
#include <volrover3/AppState.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkCamera.h>
#include <vtkCornerAnnotation.h>
#include <vtkTextProperty.h>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QWheelEvent>

VTKRenderWidget::VTKRenderWidget(QWidget *parent)
    : QVTK_WIDGET_BASE(parent)
    , m_renderWindow(vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New())
    , m_renderer(vtkSmartPointer<vtkRenderer>::New())
    , m_cameraController(std::make_unique<CameraController>())
    , m_fpsAnnotation(vtkSmartPointer<vtkCornerAnnotation>::New())
    , m_showFPS(false)
{
    initializeVTK();
    
    // Set up timer to process SceneGraph events on main thread
    connect(&m_eventTimer, &QTimer::timeout, this, &VTKRenderWidget::processSceneGraphEvents);
    m_eventTimer.start(16);  // ~60fps event processing
    
    // Set up timer to update FPS display (every 500ms)
    connect(&m_fpsTimer, &QTimer::timeout, this, &VTKRenderWidget::updateFPSDisplay);
    
    // Load FPS display setting from state
    m_showFPS = AppState::instance().showFPS();
    if (m_showFPS) {
        m_fpsAnnotation->SetVisibility(true);
        m_fpsTimer.start(500);
    }
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

    // Set up FPS annotation in top-left corner
    m_fpsAnnotation->SetText(2, "FPS: --");  // Position 2 = upper left
    m_fpsAnnotation->GetTextProperty()->SetColor(1.0, 1.0, 0.0);  // Yellow
    m_fpsAnnotation->GetTextProperty()->SetFontSize(14);
    m_fpsAnnotation->SetVisibility(false);  // Hidden by default
    m_renderer->AddViewProp(m_fpsAnnotation);

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
    // Don't pass middle mouse to VTK to avoid conflicts with our CameraController
    if (event->button() != Qt::MiddleButton) {
        QVTK_WIDGET_BASE::mousePressEvent(event);
    }
}

void VTKRenderWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (m_cameraController) {
        m_cameraController->handleMouseRelease(event->button());
    }
    // Don't pass middle mouse to VTK to avoid conflicts with our CameraController
    if (event->button() != Qt::MiddleButton) {
        QVTK_WIDGET_BASE::mouseReleaseEvent(event);
    }
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
    // Don't pass middle mouse moves to VTK when middle button is pressed
    bool isMiddlePressed = (event->buttons() & Qt::MiddleButton);
    if (!isMiddlePressed) {
        QVTK_WIDGET_BASE::mouseMoveEvent(event);
    }
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
        // Trigger render if any events modified the scene
        if (m_sceneGraph->checkAndResetRenderNeeded()) {
            render();
        }
    }
}

void VTKRenderWidget::setShowFPS(bool show)
{
    m_showFPS = show;
    
    if (m_fpsAnnotation) {
        m_fpsAnnotation->SetVisibility(show ? 1 : 0);
    }
    
    if (show) {
        // Start timer to update FPS display
        m_fpsTimer.start(500);  // Update every 500ms
        updateFPSDisplay();
    } else {
        m_fpsTimer.stop();
    }
    
    // Trigger a render to show/hide the annotation
    if (m_renderWindow) {
        m_renderWindow->Render();
    }
}

void VTKRenderWidget::updateFPSDisplay()
{
    if (!m_showFPS || !m_renderer || !m_fpsAnnotation) {
        return;
    }
    
    // Get the last render time from the renderer
    double renderTime = m_renderer->GetLastRenderTimeInSeconds();
    
    QString fpsText;
    if (renderTime > 0.0) {
        double fps = 1.0 / renderTime;
        fpsText = QString("FPS: %1").arg(fps, 0, 'f', 1);
    } else {
        fpsText = "FPS: --";
    }
    
    m_fpsAnnotation->SetText(2, fpsText.toStdString().c_str());
    
    // Trigger render to update the display
    if (m_renderWindow) {
        m_renderWindow->Render();
    }
}
