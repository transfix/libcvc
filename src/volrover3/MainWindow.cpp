#include <volrover3/MainWindow.h>
#include <volrover3/VTKRenderWidget.h>
#include <volrover3/TransferFunctionWidget.h>
#include <volrover3/SceneGraph.h>
#include <volrover3/GraphicsNode.h>
#include <volrover3/AppState.h>
#include <volrover3/BoundingBoxDialog.h>
#include <volrover3/CameraSettingsDialog.h>
#include <volrover3/GridOptionsDialog.h>
#include <volrover3/GraphicsParentDialog.h>
#include <volrover3/CameraController.h>
#include <volrover3/ThreadMonitorWidget.h>
#include <volrover3/StateTreeWidget.h>
#include <QMenuBar>
#include <QMenu>
#include <QAction>
#include <QFileDialog>
#include <QMessageBox>
#include <QStatusBar>
#include <QDockWidget>
#include <QVBoxLayout>
#include <QLabel>
#include <QProgressBar>
#include <cvc/geometry_file_io.h>
#include <cvc/volume_file_io.h>
#include <cvc/app.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , m_renderWidget(nullptr)
    , m_transferFunctionWidget(nullptr)
    , m_sceneGraph(std::make_shared<SceneGraph>())
    , m_threadMonitor(nullptr)
    , m_stateTreeWidget(nullptr)
    , m_threadNameLabel(nullptr)
    , m_threadInfoLabel(nullptr)
    , m_threadProgressBar(nullptr)
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
    setupStatusBar();
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
        
        // Set grid divisions to match volume resolution
        AppState::instance().setGridDivisions(vol.XDim(), vol.YDim(), vol.ZDim());
        
        m_sceneGraph->updateGrid(AppState::instance().worldBounds());
        m_renderWidget->update();
    });
    
    AppState::instance().onWorldBoundsChanged([this]() {
        // Update grid to match new world bounds
        m_sceneGraph->updateGrid(AppState::instance().worldBounds());
        
        // Update camera orbit center to match new bounds center
        CameraController* camCtrl = m_renderWidget->getCameraController();
        if (camCtrl) {
            cvc::bounding_box bounds = AppState::instance().worldBounds();
            camCtrl->updateOrbitCenterFromBounds(
                bounds.minx, bounds.miny, bounds.minz,
                bounds.maxx, bounds.maxy, bounds.maxz
            );
        }
        
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
    
    // Connect color state changes to update rendering
    AppState::instance().onGridColorChanged([this]() {
        double r, g, b;
        AppState::instance().getGridColor(r, g, b);
        m_sceneGraph->setGridColor(r, g, b);
        m_renderWidget->update();
    });
    
    AppState::instance().onGeometryBBoxColorChanged([this]() {
        double r, g, b;
        AppState::instance().getGeometryBBoxColor(r, g, b);
        m_sceneGraph->setGeometryBBoxColor(r, g, b);
        m_renderWidget->update();
    });
    
    AppState::instance().onVolumeBBoxColorChanged([this]() {
        double r, g, b;
        AppState::instance().getVolumeBBoxColor(r, g, b);
        m_sceneGraph->setVolumeBBoxColor(r, g, b);
        m_renderWidget->update();
    });
    
    AppState::instance().onVolumeBBoxTicksChanged([this]() {
        double r, g, b;
        AppState::instance().getVolumeBBoxTickLabelColor(r, g, b);
        m_sceneGraph->setVolumeBBoxTicks(
            AppState::instance().volumeBBoxTicksVisible(),
            AppState::instance().volumeBBoxTickInterval(),
            r, g, b,
            AppState::instance().volumeBBoxTickLabelFontSize()
        );
        m_renderWidget->update();
    });
    
    // Connect grid plane visibility and divisions changes
    AppState::instance().onGridPlaneVisibilityChanged([this]() {
        bool yz = AppState::instance().gridYZPlaneVisible();
        bool xz = AppState::instance().gridXZPlaneVisible();
        bool xy = AppState::instance().gridXYPlaneVisible();
        m_sceneGraph->setGridPlaneVisibility(yz, xz, xy);
        m_renderWidget->update();
    });
    
    AppState::instance().onGridDivisionsChanged([this]() {
        int x, y, z;
        AppState::instance().getGridDivisions(x, y, z);
        m_sceneGraph->setGridDivisions(x, y, z);
        m_renderWidget->update();
    });
    
    AppState::instance().onGridTickIntervalsChanged([this]() {
        int x, y, z;
        AppState::instance().getGridTickIntervals(x, y, z);
        m_sceneGraph->setGridTickIntervals(x, y, z);
        m_renderWidget->update();
    });
    
    // Add callback for grid ticks visibility changes
    AppState::instance().onGridTicksVisibleChanged([this]() {
        m_sceneGraph->updateGrid(AppState::instance().worldBounds());
        m_renderWidget->update();
    });
    
    AppState::instance().onGridPlaneColorsChanged([this]() {
        double yzR, yzG, yzB, xzR, xzG, xzB, xyR, xyG, xyB;
        AppState::instance().getGridYZPlaneColor(yzR, yzG, yzB);
        AppState::instance().getGridXZPlaneColor(xzR, xzG, xzB);
        AppState::instance().getGridXYPlaneColor(xyR, xyG, xyB);
        m_sceneGraph->setGridPlaneColors(yzR, yzG, yzB, xzR, xzG, xzB, xyR, xyG, xyB);
        m_renderWidget->update();
    });
    
    AppState::instance().onGridTickLabelPropertiesChanged([this]() {
        double r, g, b;
        AppState::instance().getGridTickLabelColor(r, g, b);
        int fontSize = AppState::instance().gridTickLabelFontSize();
        m_sceneGraph->setGridTickLabelProperties(r, g, b, fontSize);
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
    // Disconnect all callbacks
    for (auto& conn : m_connections) {
        conn.disconnect();
    }
    m_connections.clear();
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
    
    QAction *gridOptionsAction = new QAction(tr("&Grid Options..."), this);
    gridOptionsAction->setShortcut(tr("Ctrl+G"));
    connect(gridOptionsAction, &QAction::triggered, this, &MainWindow::showGridOptions);
    viewMenu->addAction(gridOptionsAction);
    
    viewMenu->addSeparator();
    
    QAction *threadMonitorAction = new QAction(tr("&Thread Monitor..."), this);
    threadMonitorAction->setShortcut(tr("Ctrl+T"));
    connect(threadMonitorAction, &QAction::triggered, this, &MainWindow::showThreadMonitor);
    viewMenu->addAction(threadMonitorAction);
    
    QAction *stateTreeAction = new QAction(tr("&State Tree..."), this);
    stateTreeAction->setShortcut(tr("Ctrl+Shift+S"));
    connect(stateTreeAction, &QAction::triggered, this, &MainWindow::showStateTree);
    viewMenu->addAction(stateTreeAction);

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
    cvc::thread_info ti(BOOST_CURRENT_FUNCTION);
    
    // First, show parent selection dialog
    GraphicsParentDialog parentDialog(m_sceneGraph, this);
    if (parentDialog.exec() != QDialog::Accepted) {
        return; // User cancelled
    }
    
    std::string parentName = parentDialog.getSelectedParentName();
    auto parentNode = parentDialog.getSelectedParent();
    
    // Now show file selection dialog (allow multiple files)
    QStringList fileNames = QFileDialog::getOpenFileNames(
        this,
        tr("Open Geometry File(s)"),
        QString(),
        tr("Geometry Files (*.off *.raw *.rawn *.rawc *.rawnc *.obj);;All Files (*)"));

    if (fileNames.isEmpty())
        return;

    int successCount = 0;
    int totalVertices = 0;
    int totalTriangles = 0;
    
    for (const QString& fileName : fileNames) {
        try {
            // Load geometry using CVC library
            cvc::geometry geom = cvc::read_geometry(fileName.toStdString());
            
            // Extract filename without path for naming
            QFileInfo fileInfo(fileName);
            std::string baseName = fileInfo.baseName().toStdString();
            
            // Sanitize the base name to conform to C identifier rules
            std::string sanitizedName = cvc::state::sanitizeStateName(baseName);
            
            // Create unique name if needed
            std::string graphicsName = sanitizedName;
            int counter = 1;
            while (m_sceneGraph->getGraphics(graphicsName)) {
                graphicsName = sanitizedName + "_" + std::to_string(counter++);
            }
            
            // Create graphics node
            auto graphicsNode = std::make_shared<GraphicsNode>(graphicsName);
            graphicsNode->setGeometry(geom);
            
            // Store metadata
            graphicsNode->setMetadata("type", std::string("geometry"));
            graphicsNode->setMetadata("filename", fileName.toStdString());
            graphicsNode->setMetadata("num_vertices", static_cast<int>(geom.num_points()));
            graphicsNode->setMetadata("num_triangles", static_cast<int>(geom.num_tris()));
            
            // Add to parent or root
            if (parentNode) {
                parentNode->addGraphicsChild(graphicsNode);
                m_sceneGraph->registerGraphics(graphicsName, graphicsNode);
            } else {
                m_sceneGraph->getGraphicsRoot()->addGraphicsChild(graphicsNode);
                m_sceneGraph->registerGraphics(graphicsName, graphicsNode);
            }
            
            totalVertices += geom.num_points();
            totalTriangles += geom.num_tris();
            successCount++;
            
        } catch (const std::exception &e) {
            QMessageBox::warning(this, tr("Error Loading Geometry"),
                tr("Failed to load %1:\n%2").arg(fileName).arg(e.what()));
        }
    }
    
    // Sync to state tree
    m_sceneGraph->syncGraphicsToState();
    
    // Update world bounding box to include all graphics
    cvc::bounding_box graphicsBounds = m_sceneGraph->computeGraphicsBounds();
    if (graphicsBounds[0] <= graphicsBounds[3]) { // Valid bounds
        AppState::instance().setWorldBounds(graphicsBounds);
        m_sceneGraph->updateGrid(graphicsBounds);
    }
    
    // Update render
    m_renderWidget->update();
    
    // Show status message
    if (successCount > 0) {
        QString parentMsg = parentName.empty() ? tr("root") : QString::fromStdString(parentName);
        statusBar()->showMessage(
            tr("Loaded %1 geometry file(s) under '%2': %3 vertices, %4 triangles")
                .arg(successCount)
                .arg(parentMsg)
                .arg(totalVertices)
                .arg(totalTriangles),
            5000);
    }
}

void MainWindow::openVolume()
{
    cvc::thread_info ti(BOOST_CURRENT_FUNCTION);
    
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
    cvc::thread_info ti(BOOST_CURRENT_FUNCTION);
    
    BoundingBoxDialog dialog(AppState::instance().worldBounds(), this);
    if (dialog.exec() == QDialog::Accepted) {
        AppState::instance().setWorldBounds(dialog.getBoundingBox());
    }
}

void MainWindow::editCameraSettings()
{
    cvc::thread_info ti(BOOST_CURRENT_FUNCTION);
    
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
    
    // Connect reset view signal
    connect(&dialog, &CameraSettingsDialog::resetViewRequested, [this, camCtrl]() {
        cvc::bounding_box bounds = AppState::instance().worldBounds();
        camCtrl->resetView(
            bounds.minx, bounds.miny, bounds.minz,
            bounds.maxx, bounds.maxy, bounds.maxz
        );
        m_renderWidget->update();
    });
    
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
            double cx = (bounds[0] + bounds[3]) * 0.5;
            double cy = (bounds[1] + bounds[4]) * 0.5;
            double cz = (bounds[2] + bounds[5]) * 0.5;
            camCtrl->setOrbitCenter(cx, cy, cz);
        }
        
        m_renderWidget->update();
    }
}

void MainWindow::showGridOptions()
{
    cvc::thread_info ti(BOOST_CURRENT_FUNCTION);
    
    GridOptionsDialog dialog(this);
    dialog.exec();
}

void MainWindow::showThreadMonitor()
{
    // Create thread monitor widget as a separate window if not already created
    if (!m_threadMonitor) {
        m_threadMonitor = new ThreadMonitorWidget();
        m_threadMonitor->setWindowTitle(tr("Thread Monitor - VolRover3"));
        m_threadMonitor->setAttribute(Qt::WA_DeleteOnClose);
        
        // Clean up pointer when window is closed
        connect(m_threadMonitor, &QObject::destroyed, [this]() {
            m_threadMonitor = nullptr;
        });
    }
    
    // Show and raise the window
    m_threadMonitor->show();
    m_threadMonitor->raise();
    m_threadMonitor->activateWindow();
}

void MainWindow::showStateTree()
{
    // Create state tree widget as a separate window if not already created
    if (!m_stateTreeWidget) {
        m_stateTreeWidget = new StateTreeWidget();
        m_stateTreeWidget->setWindowTitle(tr("State Tree - VolRover3"));
        m_stateTreeWidget->setAttribute(Qt::WA_DeleteOnClose);
        m_stateTreeWidget->resize(600, 500);
        
        // Set root state to the global state singleton
        m_stateTreeWidget->setRootState(&cvc::state::instance());
        
        // Clean up pointer when window is closed
        connect(m_stateTreeWidget, &QObject::destroyed, [this]() {
            m_stateTreeWidget = nullptr;
        });
        
        // Connect state tree refresh to trigger graphics updates
        connect(m_stateTreeWidget, &StateTreeWidget::stateChanged, this, [this]() {
            // Sync graphics from state tree
            m_sceneGraph->syncGraphicsFromState();
            // Update all graphics nodes
            m_sceneGraph->update();
            // Refresh render
            m_renderWidget->update();
        });
    }
    
    // Refresh to show current state
    m_stateTreeWidget->refresh();
    
    // Show and raise the window
    m_stateTreeWidget->show();
    m_stateTreeWidget->raise();
    m_stateTreeWidget->activateWindow();
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

void MainWindow::setupStatusBar()
{
    // Create status bar widgets for thread monitoring
    m_threadNameLabel = new QLabel(this);
    m_threadNameLabel->setMinimumWidth(150);
    m_threadNameLabel->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    
    m_threadInfoLabel = new QLabel(this);
    m_threadInfoLabel->setMinimumWidth(200);
    m_threadInfoLabel->setFrameStyle(QFrame::Panel | QFrame::Sunken);
    
    m_threadProgressBar = new QProgressBar(this);
    m_threadProgressBar->setMinimumWidth(150);
    m_threadProgressBar->setMaximumWidth(200);
    m_threadProgressBar->setTextVisible(true);
    m_threadProgressBar->setRange(0, 100);
    m_threadProgressBar->setValue(0);
    
    // Add widgets to status bar
    statusBar()->addPermanentWidget(m_threadNameLabel);
    statusBar()->addPermanentWidget(m_threadInfoLabel);
    statusBar()->addPermanentWidget(m_threadProgressBar);
    
    // Initially hidden
    m_threadNameLabel->hide();
    m_threadInfoLabel->hide();
    m_threadProgressBar->hide();
    
    // Register callback for thread changes
    m_connections.push_back(
        cvc::app::instance().threadsChanged.connect(
            [this](const std::string&) {
                updateThreadStatus();
            }
        )
    );
    
    // Do initial update
    updateThreadStatus();
}

void MainWindow::updateThreadStatus()
{
    // Get all threads
    auto threads = cvc::app::instance().threads();
    
    if (threads.empty()) {
        // No threads active - hide widgets
        m_threadNameLabel->hide();
        m_threadInfoLabel->hide();
        m_threadProgressBar->hide();
        statusBar()->clearMessage();
    } else {
        // Find the most recently updated thread (last in the map)
        auto lastThread = threads.rbegin();
        std::string threadKey = lastThread->first;
        auto threadPtr = lastThread->second;
        
        // Get thread info
        std::string info = cvc::app::instance().threadInfo(threadKey);
        double progress = cvc::app::instance().threadProgress(threadKey);
        
        // Update status bar widgets
        m_threadNameLabel->setText(QString::fromStdString(threadKey));
        m_threadInfoLabel->setText(QString::fromStdString(info));
        m_threadProgressBar->setValue(static_cast<int>(progress * 100.0));
        
        // Show widgets
        m_threadNameLabel->show();
        m_threadInfoLabel->show();
        m_threadProgressBar->show();
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
