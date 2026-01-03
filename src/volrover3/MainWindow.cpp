#include <volrover3/MainWindow.h>
#include <volrover3/VTKRenderWidget.h>
#include <volrover3/TransferFunctionWidget.h>
#include <volrover3/SceneGraph.h>
#include <volrover3/SceneNode.h>
#include <volrover3/GeometryNode.h>
#include <volrover3/GraphicsNode.h>
#include <volrover3/VolumeNode.h>
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
#include <QCloseEvent>
#include <QMetaObject>
#include <QThread>
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
    
    // Set up thread-safe state change callback for SceneNode hierarchy
    // This allows state changes from worker threads to be marshaled to the Qt main thread
    SceneNode::setMainThreadCallback([this](std::function<void()> func) {
        // Check if we're already on the main thread
        if (QThread::currentThread() == this->thread()) {
            // We're on the main thread, execute immediately
            func();
        } else {
            // We're on a worker thread, marshal to main thread
            QMetaObject::invokeMethod(this, [func]() { func(); }, Qt::QueuedConnection);
        }
    });

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
    
    // Initialize grid divisions from AppState (override GridNode's default 64x64x64)
    int x, y, z;
    AppState::instance().getGridDivisions(x, y, z);
    m_sceneGraph->setGridDivisions(x, y, z);
    
    // Initialize grid with default world bounds
    m_sceneGraph->updateGrid(AppState::instance().worldBounds());
    
    // Connect to state changes
    AppState::instance().onWorldBoundsChanged([this]() {
        cvc::bounding_box bounds = AppState::instance().worldBounds();
        
        // Always update grid to match new world bounds
        m_sceneGraph->updateGrid(bounds);
        
        // Update camera orbit center to match new bounds center
        CameraController* camCtrl = m_renderWidget->getCameraController();
        if (camCtrl) {
            cvc::bounding_box bounds = AppState::instance().worldBounds();
            camCtrl->updateOrbitCenterFromBounds(
                bounds.minx, bounds.miny, bounds.minz,
                bounds.maxx, bounds.maxy, bounds.maxz
            );
        }
        
        m_renderWidget->render();
    });
    
    AppState::instance().onGridVisibilityChanged([this]() {
        m_gridVisible = AppState::instance().gridVisible();
        m_sceneGraph->setGridVisible(m_gridVisible);
        m_renderWidget->render();
    });
    
    AppState::instance().onAxisVisibilityChanged([this]() {
        m_axisVisible = AppState::instance().axisVisible();
        m_sceneGraph->setAxisVisible(m_axisVisible);
        m_renderWidget->render();
    });
    
    // Connect color state changes to update rendering
    AppState::instance().onGridColorChanged([this]() {
        double r, g, b;
        AppState::instance().getGridColor(r, g, b);
        m_sceneGraph->setGridColor(r, g, b);
        m_renderWidget->render();
    });
    
    // Connect grid plane visibility and divisions changes
    AppState::instance().onGridPlaneVisibilityChanged([this]() {
        bool yz = AppState::instance().gridYZPlaneVisible();
        bool xz = AppState::instance().gridXZPlaneVisible();
        bool xy = AppState::instance().gridXYPlaneVisible();
        m_sceneGraph->setGridPlaneVisibility(yz, xz, xy);
        m_renderWidget->render();
    });
    
    AppState::instance().onGridDivisionsChanged([this]() {
        int x, y, z;
        AppState::instance().getGridDivisions(x, y, z);
        m_sceneGraph->setGridDivisions(x, y, z);
        m_renderWidget->render();
    });
    
    AppState::instance().onGridTickIntervalsChanged([this]() {
        int x, y, z;
        AppState::instance().getGridTickIntervals(x, y, z);
        m_sceneGraph->setGridTickIntervals(x, y, z);
        m_renderWidget->render();
    });
    
    // Add callback for grid ticks visibility changes
    AppState::instance().onGridTicksVisibleChanged([this]() {
        m_sceneGraph->updateGrid(AppState::instance().worldBounds());
        m_renderWidget->render();
    });
    
    AppState::instance().onGridPlaneColorsChanged([this]() {
        double yzR, yzG, yzB, xzR, xzG, xzB, xyR, xyG, xyB;
        AppState::instance().getGridYZPlaneColor(yzR, yzG, yzB);
        AppState::instance().getGridXZPlaneColor(xzR, xzG, xzB);
        AppState::instance().getGridXYPlaneColor(xyR, xyG, xyB);
        m_sceneGraph->setGridPlaneColors(yzR, yzG, yzB, xzR, xzG, xzB, xyR, xyG, xyB);
        m_renderWidget->render();
    });
    
    AppState::instance().onGridTickLabelPropertiesChanged([this]() {
        double r, g, b;
        AppState::instance().getGridTickLabelColor(r, g, b);
        int fontSize = AppState::instance().gridTickLabelFontSize();
        m_sceneGraph->setGridTickLabelProperties(r, g, b, fontSize);
        m_renderWidget->render();
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
            
            // Set flag to prevent feedback loop: we're updating FROM AppState, so don't save back
            camCtrl->setUpdatingFromAppState(true);
            camCtrl->setCameraState(pos, dir, up, fov);
            camCtrl->setUpdatingFromAppState(false);
            
            m_renderWidget->render();
        }
    });
    
    // Connect transfer function state changes to update rendering
    AppState::instance().onTransferFunctionChanged([this]() {
        m_sceneGraph->updateTransferFunction(
            AppState::instance().transferFunctionColorTable(),
            AppState::instance().transferFunctionOpacityTable());
        m_renderWidget->render();
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

void MainWindow::closeEvent(QCloseEvent *event)
{
    // Close all child windows when main window is closing
    if (m_threadMonitor) {
        m_threadMonitor->close();
    }
    if (m_stateTreeWidget) {
        m_stateTreeWidget->close();
    }
    
    // Accept the close event
    QMainWindow::closeEvent(event);
}

void MainWindow::createMenus()
{
    // File menu
    QMenu *fileMenu = menuBar()->addMenu(tr("&File"));
    
    QAction *openFileAction = new QAction(tr("&Open File..."), this);
    openFileAction->setShortcut(QKeySequence::Open);
    connect(openFileAction, &QAction::triggered, this, &MainWindow::openFile);
    fileMenu->addAction(openFileAction);

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
    
    viewMenu->addSeparator();
    
    QAction *editBoundsAction = new QAction(tr("Edit &Bounding Box..."), this);
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
    m_transferFunctionWidget->setSceneGraph(m_sceneGraph.get());
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

void MainWindow::openFile()
{
    cvc::thread_info ti(BOOST_CURRENT_FUNCTION);
    
    // First, show parent selection dialog
    GraphicsParentDialog parentDialog(m_sceneGraph, this);
    if (parentDialog.exec() != QDialog::Accepted) {
        return; // User cancelled
    }
    
    std::string parentName = parentDialog.getSelectedParentName();
    auto parentNode = parentDialog.getSelectedParent();
    
    // Get supported extensions from I/O classes
    std::vector<std::string> geomExts = cvc::geometry_file_io::get_extensions();
    std::vector<std::string> volExts = cvc::volume_file_io::getExtensions();
    
    // Build filter strings (extensions may or may not have leading dots)
    QString geomFilter;
    for (const auto& ext : geomExts) {
        if (!geomFilter.isEmpty()) geomFilter += " ";
        QString extStr = QString::fromStdString(ext);
        // Handle extensions with or without leading dot
        if (extStr.startsWith('.')) {
            geomFilter += "*" + extStr;
        } else {
            geomFilter += "*." + extStr;
        }
    }
    
    QString volFilter;
    for (const auto& ext : volExts) {
        if (!volFilter.isEmpty()) volFilter += " ";
        QString extStr = QString::fromStdString(ext);
        // Handle extensions with or without leading dot
        if (extStr.startsWith('.')) {
            volFilter += "*" + extStr;
        } else {
            volFilter += "*." + extStr;
        }
    }
    
    QString allFilter = geomFilter;
    if (!geomFilter.isEmpty() && !volFilter.isEmpty()) {
        allFilter += " ";
    }
    allFilter += volFilter;
    
    QString filters = tr("All Graphics Files (%1);;"
                        "Geometry Files (%2);;"
                        "Volume Files (%3);;"
                        "All Files (*)")
        .arg(allFilter)
        .arg(geomFilter)
        .arg(volFilter);
    
    // Show file selection dialog with both geometry and volume extensions
    QStringList fileNames = QFileDialog::getOpenFileNames(
        this,
        tr("Open Graphics File(s)"),
        QString(),
        filters);

    if (fileNames.isEmpty())
        return;

    int geomCount = 0, volCount = 0;
    int totalVertices = 0, totalTriangles = 0;
    cvc::uint64 totalVoxels = 0;
    
    for (const QString& fileName : fileNames) {
        QFileInfo fileInfo(fileName);
        
        try {
            // Extract filename without path for naming
            std::string baseName = fileInfo.baseName().toStdString();
            std::string sanitizedName = cvc::state::sanitizeStateName(baseName);
            
            // Create unique name
            std::string graphicsName = sanitizedName;
            int counter = 1;
            while (m_sceneGraph->getGraphics(graphicsName)) {
                graphicsName = sanitizedName + "_" + std::to_string(counter++);
            }
            
            // Try loading as volume first, then geometry if that fails
            bool loadedAsVolume = false;
            bool loadedAsGeometry = false;
            std::string lastError;
            
            try {
                // Try loading as volume
                cvc::volume vol(fileName.toStdString());
                auto volumeNode = m_sceneGraph->addGraphics(graphicsName, vol);
                volumeNode->setMetadata("type", std::string("volume"));
                volumeNode->setMetadata("filename", fileName.toStdString());
                
                // Set parent if requested
                if (parentNode) {
                    m_sceneGraph->getGraphicsRoot()->removeGraphicsChild(volumeNode);
                    parentNode->addGraphicsChild(volumeNode);
                }
                
                totalVoxels += vol.XDim() * vol.YDim() * vol.ZDim();
                volCount++;
                loadedAsVolume = true;
            } catch (const cvc::unsupported_volume_file_type& e) {
                // Not a volume file, try geometry
                lastError = e.what();
                try {
                    cvc::geometry geom = cvc::read_geometry(fileName.toStdString());
                    
                    // Create geometry node using parent's factory method (or root if no parent)
                    // This automatically creates the correct state path
                    std::shared_ptr<GeometryNode> graphicsNode;
                    if (parentNode) {
                        graphicsNode = parentNode->addGraphicsChild<GeometryNode>(graphicsName);
                        m_sceneGraph->registerGraphics(graphicsName, graphicsNode);
                    } else {
                        graphicsNode = m_sceneGraph->getGraphicsRoot()->addGraphicsChild<GeometryNode>(graphicsName);
                        m_sceneGraph->registerGraphics(graphicsName, graphicsNode);
                    }
                    
                    // Set geometry and metadata
                    graphicsNode->setGeometry(geom);
                    graphicsNode->setMetadata("type", std::string("geometry"));
                    graphicsNode->setMetadata("filename", fileName.toStdString());
                    graphicsNode->setMetadata("num_vertices", static_cast<int>(geom.num_points()));
                    graphicsNode->setMetadata("num_triangles", static_cast<int>(geom.num_tris()));
                    
                    // Node automatically connected to state tree via state_object constructor
                    // (no manual sync needed)
                    
                    totalVertices += geom.num_points();
                    totalTriangles += geom.num_tris();
                    geomCount++;
                    loadedAsGeometry = true;
                } catch (const cvc::unsupported_geometry_file_type&) {
                    // Neither volume nor geometry - throw a clear error
                    throw std::runtime_error("File format not supported by any loader (not a recognized volume or geometry file)");
                } catch (const std::exception& e) {
                    // Other geometry loading error
                    throw std::runtime_error(std::string("Failed to load as geometry: ") + e.what());
                }
            } catch (const std::exception& e) {
                // Other volume loading error - still try geometry
                lastError = e.what();
                try {
                    cvc::geometry geom = cvc::read_geometry(fileName.toStdString());
                    
                    // Create geometry node using parent's factory method (or root if no parent)
                    // This automatically creates the correct state path
                    std::shared_ptr<GeometryNode> graphicsNode;
                    if (parentNode) {
                        graphicsNode = parentNode->addGraphicsChild<GeometryNode>(graphicsName);
                        m_sceneGraph->registerGraphics(graphicsName, graphicsNode);
                    } else {
                        graphicsNode = m_sceneGraph->getGraphicsRoot()->addGraphicsChild<GeometryNode>(graphicsName);
                        m_sceneGraph->registerGraphics(graphicsName, graphicsNode);
                    }
                    
                    // Set geometry and metadata
                    graphicsNode->setGeometry(geom);
                    graphicsNode->setMetadata("type", std::string("geometry"));
                    graphicsNode->setMetadata("filename", fileName.toStdString());
                    graphicsNode->setMetadata("num_vertices", static_cast<int>(geom.num_points()));
                    graphicsNode->setMetadata("num_triangles", static_cast<int>(geom.num_tris()));
                    
                    // Node automatically connected to state tree via state_object constructor
                    // (no manual sync needed)
                    
                    totalVertices += geom.num_points();
                    totalTriangles += geom.num_tris();
                    geomCount++;
                    loadedAsGeometry = true;
                } catch (const cvc::unsupported_geometry_file_type& e) {
                    // Failed both ways - report the original volume error
                    throw std::runtime_error(lastError);
                }
            }
            
        } catch (const std::exception &e) {
            QMessageBox::warning(this, tr("Error Loading File"),
                tr("Failed to load %1:\n%2").arg(fileName).arg(e.what()));
        }
    }
    
    // Note: No manual sync needed - nodes auto-sync via state_object
    
    // Update world bounding box to include all graphics
    cvc::bounding_box graphicsBounds = m_sceneGraph->computeGraphicsBounds();
    if (graphicsBounds[0] <= graphicsBounds[3]) { // Valid bounds
        AppState::instance().setWorldBounds(graphicsBounds);
        // Grid will update automatically via onWorldBoundsChanged callback
    }
    
    // Update render
    m_renderWidget->render();
    
    // Refresh transfer function widget if volumes were loaded
    if (volCount > 0) {
        m_transferFunctionWidget->refreshVolumeList();
    }
    
    // Show status message
    if (geomCount > 0 || volCount > 0) {
        QString parentMsg = parentName.empty() ? tr("root") : QString::fromStdString(parentName);
        QString msg;
        if (geomCount > 0 && volCount > 0) {
            msg = tr("Loaded %1 geometry file(s) (%2 vertices, %3 triangles) and %4 volume file(s) (%5 voxels) under '%6'")
                .arg(geomCount).arg(totalVertices).arg(totalTriangles)
                .arg(volCount).arg(totalVoxels).arg(parentMsg);
        } else if (geomCount > 0) {
            msg = tr("Loaded %1 geometry file(s) under '%2': %3 vertices, %4 triangles")
                .arg(geomCount).arg(parentMsg).arg(totalVertices).arg(totalTriangles);
        } else {
            msg = tr("Loaded %1 volume file(s) under '%2': %3 voxels")
                .arg(volCount).arg(parentMsg).arg(totalVoxels);
        }
        statusBar()->showMessage(msg, 5000);
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

void MainWindow::editBoundingBox()
{
    cvc::thread_info ti(BOOST_CURRENT_FUNCTION);
    
    BoundingBoxDialog dialog(m_sceneGraph, this);
    dialog.exec();
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
        m_renderWidget->render();
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
        
        m_renderWidget->render();
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
            // Update all graphics nodes
            m_sceneGraph->update();
            // Force immediate render
            m_renderWidget->render();
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
