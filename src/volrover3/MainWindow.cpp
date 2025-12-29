#include <volrover3/MainWindow.h>
#include <volrover3/VTKRenderWidget.h>
#include <volrover3/TransferFunctionWidget.h>
#include <volrover3/SceneGraph.h>
#include <volrover3/AppState.h>
#include <volrover3/BoundingBoxDialog.h>
#include <volrover3/CameraSettingsDialog.h>
#include <volrover3/CameraController.h>
#include <QMenuBar>
#include <QMenu>
#include <QAction>
#include <QFileDialog>
#include <QMessageBox>
#include <QStatusBar>
#include <QDockWidget>
#include <QVBoxLayout>
#include <cvc/geometry_file_io.h>
#include <cvc/volume_file_io.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , m_renderWidget(nullptr)
    , m_transferFunctionWidget(nullptr)
    , m_sceneGraph(std::make_shared<SceneGraph>())
    , m_gridVisible(true)
    , m_axisVisible(true)
{
    setWindowTitle("VolRover3 - Volume Rover Version 3");
    resize(1280, 720);

    // Create central render widget
    m_renderWidget = new VTKRenderWidget(this);
    m_renderWidget->setSceneGraph(m_sceneGraph);
    setCentralWidget(m_renderWidget);

    createDockWidgets();
    createMenus();
    setupConnections();

    // Initialize from AppState
    m_gridVisible = AppState::instance().gridVisible();
    m_axisVisible = AppState::instance().axisVisible();
    m_sceneGraph->setGridVisible(m_gridVisible);
    m_sceneGraph->setAxisVisible(m_axisVisible);
    
    // Connect to state changes
    AppState::instance().onGeometryChanged([this]() {
        m_sceneGraph->setGeometry(AppState::instance().geometry());
        m_sceneGraph->updateGrid(AppState::instance().worldBounds());
        m_renderWidget->update();
    });
    
    AppState::instance().onVolumeChanged([this]() {
        cvc::volume vol = AppState::instance().volume();
        m_sceneGraph->setVolume(vol);
        m_transferFunctionWidget->setDataRange(vol.min(), vol.max());
        m_sceneGraph->updateGrid(AppState::instance().worldBounds());
        m_renderWidget->update();
    });
    
    AppState::instance().onWorldBoundsChanged([this]() {
        // Update grid to match new world bounds
        m_sceneGraph->updateGrid(AppState::instance().worldBounds());
        m_renderWidget->update();
    });
    
    AppState::instance().onGridVisibilityChanged([this]() {
        m_gridVisible = AppState::instance().gridVisible();
        m_sceneGraph->setGridVisible(m_gridVisible);
        m_renderWidget->update();
    });
    
    AppState::instance().onAxisVisibilityChanged([this]() {
        m_axisVisible = AppState::instance().axisVisible();
        m_sceneGraph->setAxisVisible(m_axisVisible);
        m_renderWidget->update();
    });
    
    AppState::instance().onGeometryBBoxVisibilityChanged([this]() {
        m_sceneGraph->setGeometryBBoxVisible(AppState::instance().geometryBBoxVisible());
        m_renderWidget->update();
    });
    
    AppState::instance().onVolumeBBoxVisibilityChanged([this]() {
        m_sceneGraph->setVolumeBBoxVisible(AppState::instance().volumeBBoxVisible());
        m_renderWidget->update();
    });
    
    // Connect camera state changes to update rendering
    AppState::instance().onCameraChanged([this]() {
        CameraController* camCtrl = m_renderWidget->getCameraController();
        if (camCtrl) {
            double pos[3], dir[3], up[3], fov;
            AppState::instance().getCameraPosition(pos[0], pos[1], pos[2]);
            AppState::instance().getCameraViewDirection(dir[0], dir[1], dir[2]);
            AppState::instance().getCameraUpVector(up[0], up[1], up[2]);
            fov = AppState::instance().cameraFieldOfView();
            camCtrl->setCameraState(pos, dir, up, fov);
            m_renderWidget->update();
        }
    });
    
    // Connect transfer function state changes to update rendering
    AppState::instance().onTransferFunctionChanged([this]() {
        m_sceneGraph->updateTransferFunction(
            AppState::instance().transferFunctionColorTable(),
            AppState::instance().transferFunctionOpacityTable());
        m_renderWidget->update();
    });
    
    // Initialize camera settings from AppState
    initializeCameraFromState();
    
    // Initialize transfer function from AppState
    m_sceneGraph->updateTransferFunction(
        AppState::instance().transferFunctionColorTable(),
        AppState::instance().transferFunctionOpacityTable());
}

MainWindow::~MainWindow()
{
}

void MainWindow::createMenus()
{
    // File menu
    QMenu *fileMenu = menuBar()->addMenu(tr("&File"));
    
    QAction *openGeomAction = new QAction(tr("Open &Geometry..."), this);
    openGeomAction->setShortcut(QKeySequence::Open);
    connect(openGeomAction, &QAction::triggered, this, &MainWindow::openGeometry);
    fileMenu->addAction(openGeomAction);

    QAction *openVolAction = new QAction(tr("Open &Volume..."), this);
    openVolAction->setShortcut(tr("Ctrl+V"));
    connect(openVolAction, &QAction::triggered, this, &MainWindow::openVolume);
    fileMenu->addAction(openVolAction);

    fileMenu->addSeparator();

    QAction *exitAction = new QAction(tr("E&xit"), this);
    exitAction->setShortcut(QKeySequence::Quit);
    connect(exitAction, &QAction::triggered, this, &QMainWindow::close);
    fileMenu->addAction(exitAction);

    // View menu
    QMenu *viewMenu = menuBar()->addMenu(tr("&View"));

    QAction *toggleGridAction = new QAction(tr("Show &Grid"), this);
    toggleGridAction->setCheckable(true);
    toggleGridAction->setChecked(m_gridVisible);
    connect(toggleGridAction, &QAction::triggered, this, &MainWindow::toggleGrid);
    viewMenu->addAction(toggleGridAction);

    QAction *toggleAxisAction = new QAction(tr("Show &Axis"), this);
    toggleAxisAction->setCheckable(true);
    toggleAxisAction->setChecked(m_axisVisible);
    connect(toggleAxisAction, &QAction::triggered, this, &MainWindow::toggleAxis);
    viewMenu->addAction(toggleAxisAction);
    
    QAction *toggleGeomBBoxAction = new QAction(tr("Show Geometry Bounding Bo&x"), this);
    toggleGeomBBoxAction->setCheckable(true);
    toggleGeomBBoxAction->setChecked(false);
    connect(toggleGeomBBoxAction, &QAction::triggered, this, &MainWindow::toggleGeometryBBox);
    viewMenu->addAction(toggleGeomBBoxAction);
    
    QAction *toggleVolBBoxAction = new QAction(tr("Show &Volume Bounding Box"), this);
    toggleVolBBoxAction->setCheckable(true);
    toggleVolBBoxAction->setChecked(false);
    connect(toggleVolBBoxAction, &QAction::triggered, this, &MainWindow::toggleVolumeBBox);
    viewMenu->addAction(toggleVolBBoxAction);
    
    viewMenu->addSeparator();
    
    QAction *editBoundsAction = new QAction(tr("Edit World &Bounding Box..."), this);
    editBoundsAction->setShortcut(tr("Ctrl+B"));
    connect(editBoundsAction, &QAction::triggered, this, &MainWindow::editBoundingBox);
    viewMenu->addAction(editBoundsAction);
    
    QAction *editCameraAction = new QAction(tr("&Camera Settings..."), this);
    editCameraAction->setShortcut(tr("Ctrl+K"));
    connect(editCameraAction, &QAction::triggered, this, &MainWindow::editCameraSettings);
    viewMenu->addAction(editCameraAction);

    // Help menu
    QMenu *helpMenu = menuBar()->addMenu(tr("&Help"));

    QAction *aboutAction = new QAction(tr("&About VolRover3"), this);
    connect(aboutAction, &QAction::triggered, this, &MainWindow::aboutVolRover);
    helpMenu->addAction(aboutAction);
}

void MainWindow::createDockWidgets()
{
    // Transfer function dock widget
    QDockWidget *tfDock = new QDockWidget(tr("Transfer Function"), this);
    tfDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    
    m_transferFunctionWidget = new TransferFunctionWidget(tfDock);
    tfDock->setWidget(m_transferFunctionWidget);
    
    addDockWidget(Qt::RightDockWidgetArea, tfDock);
}

void MainWindow::setupConnections()
{
    // Connect transfer function changes to AppState
    connect(m_transferFunctionWidget, &TransferFunctionWidget::transferFunctionChanged,
            [this]() {
                // Save to AppState
                AppState::instance().setTransferFunctionColorTable(
                    m_transferFunctionWidget->getColorTable());
                AppState::instance().setTransferFunctionOpacityTable(
                    m_transferFunctionWidget->getOpacityTable());
            });
}

void MainWindow::openGeometry()
{
    QString fileName = QFileDialog::getOpenFileName(
        this,
        tr("Open Geometry File"),
        QString(),
        tr("Geometry Files (*.off *.raw *.rawn *.rawc *.rawnc *.obj);;All Files (*)"));

    if (fileName.isEmpty())
        return;

    try {
        // Load geometry using CVC library
        cvc::geometry geom = cvc::read_geometry(fileName.toStdString());
        
        // Update app state (will trigger scene graph update via callback)
        AppState::instance().setGeometry(geom);

        statusBar()->showMessage(tr("Loaded geometry: %1 vertices, %2 triangles")
            .arg(geom.num_points())
            .arg(geom.num_tris()), 3000);
    }
    catch (const std::exception &e) {
        QMessageBox::critical(this, tr("Error Loading Geometry"),
            tr("Failed to load geometry file:\n%1").arg(e.what()));
    }
}

void MainWindow::openVolume()
{
    QString fileName = QFileDialog::getOpenFileName(
        this,
        tr("Open Volume File"),
        QString(),
        tr("Volume Files (*.rawiv *.mrc *.ccp4);;All Files (*)"));

    if (fileName.isEmpty())
        return;

    try {
        // Load volume using CVC library
        cvc::volume vol(fileName.toStdString());
        
        // Update app state (will trigger scene graph update via callback)
        AppState::instance().setVolume(vol);

        statusBar()->showMessage(tr("Loaded volume: %1x%2x%3")
            .arg(vol.XDim())
            .arg(vol.YDim())
            .arg(vol.ZDim()), 3000);
    }
    catch (const std::exception &e) {
        QMessageBox::critical(this, tr("Error Loading Volume"),
            tr("Failed to load volume file:\n%1").arg(e.what()));
    }
}

void MainWindow::toggleGrid()
{
    m_gridVisible = !m_gridVisible;
    AppState::instance().setGridVisible(m_gridVisible);
}

void MainWindow::toggleAxis()
{
    m_axisVisible = !m_axisVisible;
    AppState::instance().setAxisVisible(m_axisVisible);
}

void MainWindow::toggleGeometryBBox()
{
    bool visible = !AppState::instance().geometryBBoxVisible();
    AppState::instance().setGeometryBBoxVisible(visible);
}

void MainWindow::toggleVolumeBBox()
{
    bool visible = !AppState::instance().volumeBBoxVisible();
    AppState::instance().setVolumeBBoxVisible(visible);
}

void MainWindow::editBoundingBox()
{
    BoundingBoxDialog dialog(AppState::instance().worldBounds(), this);
    if (dialog.exec() == QDialog::Accepted) {
        AppState::instance().setWorldBounds(dialog.getBoundingBox());
    }
}

void MainWindow::editCameraSettings()
{
    CameraController *camCtrl = m_renderWidget->getCameraController();
    if (!camCtrl) return;
    
    // Get current settings from AppState
    CameraSettingsDialog::CameraSettings settings;
    settings.mode = AppState::instance().cameraMode();
    settings.flySpeed = AppState::instance().cameraSpeed();
    settings.mouseSensitivity = AppState::instance().cameraSensitivity();
    settings.invertMouse = AppState::instance().cameraInvertMouse();
    settings.keyForward = AppState::instance().cameraKeyForward();
    settings.keyBackward = AppState::instance().cameraKeyBackward();
    settings.keyStrafeLeft = AppState::instance().cameraKeyLeft();
    settings.keyStrafeRight = AppState::instance().cameraKeyRight();
    settings.keyUp = AppState::instance().cameraKeyUp();
    settings.keyDown = AppState::instance().cameraKeyDown();
    
    CameraSettingsDialog dialog(settings, this);
    if (dialog.exec() == QDialog::Accepted) {
        CameraSettingsDialog::CameraSettings newSettings = dialog.getSettings();
        
        // Save settings to AppState
        AppState::instance().setCameraMode(newSettings.mode);
        AppState::instance().setCameraSpeed(newSettings.flySpeed);
        AppState::instance().setCameraSensitivity(newSettings.mouseSensitivity);
        AppState::instance().setCameraInvertMouse(newSettings.invertMouse);
        AppState::instance().setCameraKeyForward(newSettings.keyForward);
        AppState::instance().setCameraKeyBackward(newSettings.keyBackward);
        AppState::instance().setCameraKeyLeft(newSettings.keyStrafeLeft);
        AppState::instance().setCameraKeyRight(newSettings.keyStrafeRight);
        AppState::instance().setCameraKeyUp(newSettings.keyUp);
        AppState::instance().setCameraKeyDown(newSettings.keyDown);
        
        // Apply settings to controller
        camCtrl->setMode(static_cast<CameraMode>(newSettings.mode));
        camCtrl->setMovementSpeed(newSettings.flySpeed);
        camCtrl->setMouseSensitivity(newSettings.mouseSensitivity);
        camCtrl->setInvertMouse(newSettings.invertMouse);
        camCtrl->setKeyBindings(
            newSettings.keyForward,
            newSettings.keyBackward,
            newSettings.keyStrafeLeft,
            newSettings.keyStrafeRight,
            newSettings.keyUp,
            newSettings.keyDown
        );
        
        // Update orbit center to world bounds center when switching to orbit mode
        if (newSettings.mode == 0) {
            cvc::bounding_box bounds = AppState::instance().worldBounds();
            double cx = (bounds[0] + bounds[3]) / 2.0;
            double cy = (bounds[1] + bounds[4]) / 2.0;
            double cz = (bounds[2] + bounds[5]) / 2.0;
            camCtrl->setOrbitCenter(cx, cy, cz);
        }
    }
}

void MainWindow::initializeCameraFromState()
{
    CameraController *camCtrl = m_renderWidget->getCameraController();
    if (!camCtrl) return;
    
    // Load all camera settings from AppState
    camCtrl->setMode(static_cast<CameraMode>(AppState::instance().cameraMode()));
    camCtrl->setMovementSpeed(AppState::instance().cameraSpeed());
    camCtrl->setMouseSensitivity(AppState::instance().cameraSensitivity());
    camCtrl->setInvertMouse(AppState::instance().cameraInvertMouse());
    camCtrl->setKeyBindings(
        AppState::instance().cameraKeyForward(),
        AppState::instance().cameraKeyBackward(),
        AppState::instance().cameraKeyLeft(),
        AppState::instance().cameraKeyRight(),
        AppState::instance().cameraKeyUp(),
        AppState::instance().cameraKeyDown()
    );
    
    // Load camera position, direction, up vector, and FOV
    double pos[3], dir[3], up[3], fov;
    AppState::instance().getCameraPosition(pos[0], pos[1], pos[2]);
    AppState::instance().getCameraViewDirection(dir[0], dir[1], dir[2]);
    AppState::instance().getCameraUpVector(up[0], up[1], up[2]);
    fov = AppState::instance().cameraFieldOfView();
    camCtrl->setCameraState(pos, dir, up, fov);
    
    // Set orbit center to world bounds center
    cvc::bounding_box bounds = AppState::instance().worldBounds();
    double cx = (bounds[0] + bounds[3]) / 2.0;
    double cy = (bounds[1] + bounds[4]) / 2.0;
    double cz = (bounds[2] + bounds[5]) / 2.0;
    camCtrl->setOrbitCenter(cx, cy, cz);
}

void MainWindow::aboutVolRover()
{
    QMessageBox::about(this, tr("About VolRover3"),
        tr("<h2>VolRover3</h2>"
           "<p>Volume Rover Version 3.0</p>"
           "<p>A prototype visualization application built on libcvc</p>"
           "<p>Features:</p>"
           "<ul>"
           "<li>Volume rendering with transfer functions</li>"
           "<li>Surface and volumetric mesh visualization</li>"
           "<li>Isosurface extraction and rendering</li>"
           "<li>Quake-style camera controls</li>"
           "</ul>"
           "<p>Copyright © 2025 CVC</p>"));
}
