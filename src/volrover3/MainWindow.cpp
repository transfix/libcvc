#include <QAction>
#include <QCloseEvent>
#include <QDockWidget>
#include <QFileDialog>
#include <QLabel>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QMetaObject>
#include <QProgressBar>
#include <QStatusBar>
#include <QThread>
#include <QVBoxLayout>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry_file_io.h>
#include <cvc/volume/volume_file_io.h>
#include <volrover3/AppState.h>
#include <volrover3/BoundingBoxDialog.h>
#include <volrover3/CameraController.h>
#include <volrover3/CameraSettingsDialog.h>
#include <volrover3/GeometryDialog.h>
#include <volrover3/GeometryNode.h>
#include <volrover3/GraphicsNode.h>
#include <volrover3/GraphicsParentDialog.h>
#include <volrover3/GridNode.h>
#include <volrover3/GridOptionsDialog.h>
#include <volrover3/IsosurfaceDialog.h>
#include <volrover3/MainWindow.h>
#include <volrover3/NullGraphicNode.h>
#include <volrover3/ProceduralGeometryDialog.h>
#include <volrover3/SDFDialog.h>
#include <volrover3/SceneGraph.h>
#include <volrover3/SceneNode.h>
#include <volrover3/StateTreeWidget.h>
#include <volrover3/ThreadMonitorWidget.h>
#include <volrover3/TransferFunctionWidget.h>
#include <volrover3/VTKRenderWidget.h>
#include <volrover3/ViewerOptionsDialog.h>
#include <volrover3/VolumeDialog.h>
#include <volrover3/VolumeNode.h>
#include <volrover3/volrover3_app.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), m_renderWidget(nullptr), m_transferFunctionWidget(nullptr),
      m_sceneGraph(nullptr) // Will be created after callback is set
      ,
      m_threadMonitor(nullptr), m_stateTreeWidget(nullptr), m_gridOptionsDialog(nullptr),
      m_sdfDialog(nullptr), m_isosurfaceDialog(nullptr), m_geometryDialog(nullptr),
      m_volumeDialog(nullptr), m_viewerOptionsDialog(nullptr), m_cameraDialog(nullptr),
      m_mainToolBar(nullptr), m_threadNameLabel(nullptr), m_threadInfoLabel(nullptr),
      m_threadProgressBar(nullptr), m_gridVisible(true), m_axisVisible(true) {
  setWindowTitle("VolRover3 - Volume Rover Version 3");
  resize(1280, 720);

  // Set up thread-safe state change callback for SceneNode hierarchy FIRST
  // This MUST be done before creating SceneGraph to avoid VTK calls on background threads
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

  // NOW create the scene graph - state changes will be properly marshaled
  m_sceneGraph = std::make_shared<SceneGraph>();

  // Create central render widget
  m_renderWidget = new VTKRenderWidget(this);
  m_renderWidget->setSceneGraph(m_sceneGraph);
  setCentralWidget(m_renderWidget);

  createDockWidgets();
  createMenus();
  createToolBar();
  setupStatusBar();
  setupConnections();

  // GridNode and AxisNode initialize their own visibility state
  // Get initial values from their state
  m_gridVisible = m_sceneGraph->getGridNode()->isVisible();
  m_axisVisible = true; // AxisNode default

  // Initialize grid with default world bounds
  m_sceneGraph->updateGrid(AppState::instance().worldBounds());

  // Connect to state changes
  AppState::instance().onWorldBoundsChanged([this]() {
    cvc::bounding_box bounds = AppState::instance().worldBounds();

    std::cout << "[DEBUG] MainWindow - World bounds changed: [" << bounds[0] << "," << bounds[1]
              << "," << bounds[2] << "] to [" << bounds[3] << "," << bounds[4] << "," << bounds[5]
              << "]" << std::endl;

    // Always update grid to match new world bounds
    m_sceneGraph->updateGrid(bounds);

    // Update camera orbit center to match new bounds center
    CameraController *camCtrl = m_renderWidget->getCameraController();
    if (camCtrl) {
      cvc::bounding_box bounds = AppState::instance().worldBounds();
      camCtrl->updateOrbitCenterFromBounds(bounds.minx, bounds.miny, bounds.minz, bounds.maxx,
                                           bounds.maxy, bounds.maxz);
    }

    m_renderWidget->render();
  });

  // GridNode and AxisNode handle their own state changes internally via handleStateChanged()
  // and updates VTK actors automatically. No MainWindow callbacks needed.
  // Grid state is at: volrover3.graphics.root.children.grid.*

  // Initialize camera settings from state tree
  initializeCameraFromState();
}

MainWindow::~MainWindow() {
  // Disconnect all callbacks
  for (auto &conn : m_connections) {
    conn.disconnect();
  }
  m_connections.clear();
}

void MainWindow::closeEvent(QCloseEvent *event) {
  // Close all child windows when main window is closing
  if (m_threadMonitor) {
    m_threadMonitor->close();
  }
  if (m_stateTreeWidget) {
    m_stateTreeWidget->close();
  }
  if (m_gridOptionsDialog) {
    m_gridOptionsDialog->close();
  }
  if (m_sdfDialog) {
    m_sdfDialog->close();
  }
  if (m_isosurfaceDialog) {
    m_isosurfaceDialog->close();
  }
  if (m_viewerOptionsDialog) {
    m_viewerOptionsDialog->close();
  }

  // Accept the close event
  QMainWindow::closeEvent(event);
}

void MainWindow::createMenus() {
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

  QAction *viewerOptionsAction = new QAction(tr("&Viewer Options..."), this);
  viewerOptionsAction->setShortcut(tr("Ctrl+Shift+V"));
  connect(viewerOptionsAction, &QAction::triggered, this, &MainWindow::showViewerOptions);
  viewMenu->addAction(viewerOptionsAction);

  viewMenu->addSeparator();

  QAction *threadMonitorAction = new QAction(tr("&Thread Monitor..."), this);
  threadMonitorAction->setShortcut(tr("Ctrl+T"));
  connect(threadMonitorAction, &QAction::triggered, this, &MainWindow::showThreadMonitor);
  viewMenu->addAction(threadMonitorAction);

  QAction *stateTreeAction = new QAction(tr("&State Tree..."), this);
  stateTreeAction->setShortcut(tr("Ctrl+Shift+S"));
  connect(stateTreeAction, &QAction::triggered, this, &MainWindow::showStateTree);
  viewMenu->addAction(stateTreeAction);

  viewMenu->addSeparator();

  QAction *geometryAction = new QAction(tr("Geo&metry Properties..."), this);
  geometryAction->setShortcut(tr("Ctrl+M"));
  connect(geometryAction, &QAction::triggered, this, &MainWindow::showGeometry);
  viewMenu->addAction(geometryAction);

  QAction *volumeAction = new QAction(tr("&Volume Properties..."), this);
  volumeAction->setShortcut(tr("Ctrl+V"));
  connect(volumeAction, &QAction::triggered, this, &MainWindow::showVolume);
  viewMenu->addAction(volumeAction);

  // Tools menu
  QMenu *toolsMenu = menuBar()->addMenu(tr("&Tools"));

  QAction *sdfAction = new QAction(tr("&Signed Distance Function..."), this);
  sdfAction->setShortcut(tr("Ctrl+D"));
  connect(sdfAction, &QAction::triggered, this, &MainWindow::showSDF);
  toolsMenu->addAction(sdfAction);

  QAction *isoAction = new QAction(tr("&Isosurface Extraction..."), this);
  isoAction->setShortcut(tr("Ctrl+I"));
  connect(isoAction, &QAction::triggered, this, &MainWindow::showIsosurface);
  toolsMenu->addAction(isoAction);

  toolsMenu->addSeparator();

  // Generate submenu
  QMenu *generateMenu = toolsMenu->addMenu(tr("&Generate"));

  // Geometry submenu under Generate
  QMenu *generateGeometryMenu = generateMenu->addMenu(tr("&Geometry"));

  QAction *bunnyAction = new QAction(tr("Stanford &Bunny"), this);
  bunnyAction->setToolTip(tr("Generate the Stanford Bunny test geometry"));
  connect(bunnyAction, &QAction::triggered, this, &MainWindow::generateStanfordBunny);
  generateGeometryMenu->addAction(bunnyAction);

  generateGeometryMenu->addSeparator();

  QAction *sphereAction = new QAction(tr("&Sphere..."), this);
  sphereAction->setToolTip(tr("Generate a parametric sphere"));
  connect(sphereAction, &QAction::triggered, this, &MainWindow::generateSphere);
  generateGeometryMenu->addAction(sphereAction);

  QAction *cubeAction = new QAction(tr("&Cube..."), this);
  cubeAction->setToolTip(tr("Generate a parametric cube"));
  connect(cubeAction, &QAction::triggered, this, &MainWindow::generateCube);
  generateGeometryMenu->addAction(cubeAction);

  QAction *torusAction = new QAction(tr("&Torus..."), this);
  torusAction->setToolTip(tr("Generate a parametric torus"));
  connect(torusAction, &QAction::triggered, this, &MainWindow::generateTorus);
  generateGeometryMenu->addAction(torusAction);

  QAction *coneAction = new QAction(tr("C&one..."), this);
  coneAction->setToolTip(tr("Generate a parametric cone"));
  connect(coneAction, &QAction::triggered, this, &MainWindow::generateCone);
  generateGeometryMenu->addAction(coneAction);

  // Help menu
  QMenu *helpMenu = menuBar()->addMenu(tr("&Help"));

  QAction *aboutAction = new QAction(tr("&About VolRover3"), this);
  connect(aboutAction, &QAction::triggered, this, &MainWindow::aboutVolRover);
  helpMenu->addAction(aboutAction);
}

void MainWindow::createToolBar() {
  m_mainToolBar = addToolBar(tr("Main Toolbar"));
  m_mainToolBar->setObjectName("MainToolBar");
  m_mainToolBar->setMovable(true);

  // Reset Camera button
  QAction *resetCameraAction =
      new QAction(QIcon::fromTheme("view-refresh"), tr("Reset Camera"), this);
  resetCameraAction->setToolTip(tr("Reset camera to view all content"));
  resetCameraAction->setShortcut(tr("Ctrl+R"));
  connect(resetCameraAction, &QAction::triggered, this, &MainWindow::resetCamera);
  m_mainToolBar->addAction(resetCameraAction);

  m_mainToolBar->addSeparator();

  // Axis toggle
  QAction *axisAction = new QAction(QIcon::fromTheme("show-axis"), tr("Toggle Axis"), this);
  axisAction->setToolTip(tr("Show/hide coordinate axis"));
  axisAction->setCheckable(true);
  axisAction->setChecked(m_axisVisible);
  connect(axisAction, &QAction::triggered, this, &MainWindow::toggleAxis);
  m_mainToolBar->addAction(axisAction);
}

void MainWindow::createDockWidgets() {
  // Transfer function dock widget
  QDockWidget *tfDock = new QDockWidget(tr("Transfer Function"), this);
  tfDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

  m_transferFunctionWidget = new TransferFunctionWidget(tfDock);
  m_transferFunctionWidget->setSceneGraph(m_sceneGraph.get());
  tfDock->setWidget(m_transferFunctionWidget);

  addDockWidget(Qt::RightDockWidgetArea, tfDock);
}

void MainWindow::setupConnections() {
  // Connect transfer function changes to update only the selected volume
  connect(m_transferFunctionWidget, &TransferFunctionWidget::transferFunctionChanged, [this]() {
    // Apply transfer function to the selected volume only
    auto selectedVolume = m_transferFunctionWidget->getSelectedVolume();
    if (selectedVolume) {
      selectedVolume->setTransferFunction(m_transferFunctionWidget->getColorTable(),
                                          m_transferFunctionWidget->getOpacityTable());
      m_renderWidget->render();
    }
  });
}

void MainWindow::openFile() {
  cvc::thread_info ti(volrover3::app(), BOOST_CURRENT_FUNCTION);

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
  for (const auto &ext : geomExts) {
    if (!geomFilter.isEmpty())
      geomFilter += " ";
    QString extStr = QString::fromStdString(ext);
    // Handle extensions with or without leading dot
    if (extStr.startsWith('.')) {
      geomFilter += "*" + extStr;
    } else {
      geomFilter += "*." + extStr;
    }
  }

  QString volFilter;
  for (const auto &ext : volExts) {
    if (!volFilter.isEmpty())
      volFilter += " ";
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
  QStringList fileNames =
      QFileDialog::getOpenFileNames(this, tr("Open Graphics File(s)"), QString(), filters);

  if (fileNames.isEmpty())
    return;

  int geomCount = 0, volCount = 0;
  int totalVertices = 0, totalTriangles = 0;
  cvc::uint64 totalVoxels = 0;

  for (const QString &fileName : fileNames) {
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
        cvc::volume vol(volrover3::app(), fileName.toStdString());
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
      } catch (const cvc::unsupported_volume_file_type &e) {
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
            graphicsNode =
                m_sceneGraph->getGraphicsRoot()->addGraphicsChild<GeometryNode>(graphicsName);
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
        } catch (const cvc::unsupported_geometry_file_type &) {
          // Neither volume nor geometry - throw a clear error
          throw std::runtime_error(
              "File format not supported by any loader (not a recognized volume or geometry file)");
        } catch (const std::exception &e) {
          // Other geometry loading error
          throw std::runtime_error(std::string("Failed to load as geometry: ") + e.what());
        }
      } catch (const std::exception &e) {
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
            graphicsNode =
                m_sceneGraph->getGraphicsRoot()->addGraphicsChild<GeometryNode>(graphicsName);
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
        } catch (const cvc::unsupported_geometry_file_type &e) {
          // Failed both ways - report the original volume error
          throw std::runtime_error(lastError);
        }
      }

    } catch (const std::exception &e) {
      QMessageBox::warning(this, tr("Error Loading File"),
                           tr("Failed to load %1:\n%2").arg(fileName).arg(e.what()));
    }
  }

  // Note: World bounds are automatically updated when root NullGraphicNode
  // syncs its bounds to children (happens in GeometryNode::setGeometry/VolumeNode::setVolume)
  // Grid will update automatically via onWorldBoundsChanged callback

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
      msg = tr("Loaded %1 geometry file(s) (%2 vertices, %3 triangles) and %4 volume file(s) (%5 "
               "voxels) under '%6'")
                .arg(geomCount)
                .arg(totalVertices)
                .arg(totalTriangles)
                .arg(volCount)
                .arg(totalVoxels)
                .arg(parentMsg);
    } else if (geomCount > 0) {
      msg = tr("Loaded %1 geometry file(s) under '%2': %3 vertices, %4 triangles")
                .arg(geomCount)
                .arg(parentMsg)
                .arg(totalVertices)
                .arg(totalTriangles);
    } else {
      msg = tr("Loaded %1 volume file(s) under '%2': %3 voxels")
                .arg(volCount)
                .arg(parentMsg)
                .arg(totalVoxels);
    }
    statusBar()->showMessage(msg, 5000);
  }
}

void MainWindow::toggleGrid() {
  m_gridVisible = !m_gridVisible;
  m_sceneGraph->setGridVisible(m_gridVisible);
}

void MainWindow::toggleAxis() {
  m_axisVisible = !m_axisVisible;
  m_sceneGraph->setAxisVisible(m_axisVisible);
}

void MainWindow::editBoundingBox() {
  cvc::thread_info ti(volrover3::app(), BOOST_CURRENT_FUNCTION);

  BoundingBoxDialog dialog(m_sceneGraph, this);
  dialog.exec();
}

void MainWindow::editCameraSettings() {
  cvc::thread_info ti(volrover3::app(), BOOST_CURRENT_FUNCTION);

  CameraController *camCtrl = m_renderWidget->getCameraController();
  if (!camCtrl)
    return;

  if (!m_cameraDialog) {
    // Get current settings from AppState for initial setup
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

    // Pass camera state tree for live state display (subscribes to childChanged signal)
    cvc::state &cameraState = camCtrl->getState();

    m_cameraDialog = new CameraSettingsDialog(settings, &cameraState, this);
    m_cameraDialog->setAttribute(Qt::WA_DeleteOnClose);

    // Connect destroyed signal to reset pointer
    connect(m_cameraDialog, &QObject::destroyed, [this]() { m_cameraDialog = nullptr; });

    // Connect reset view signal
    connect(m_cameraDialog, &CameraSettingsDialog::resetViewRequested, [this, camCtrl]() {
      cvc::bounding_box bounds = AppState::instance().worldBounds();
      camCtrl->resetView(bounds.minx, bounds.miny, bounds.minz, bounds.maxx, bounds.maxy,
                         bounds.maxz);
      m_renderWidget->render();
    });

    // Connect settings changed signal for real-time application
    connect(m_cameraDialog, &CameraSettingsDialog::settingsChanged,
            [this, camCtrl](const CameraSettingsDialog::CameraSettings &newSettings) {
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
              camCtrl->setKeyBindings(newSettings.keyForward, newSettings.keyBackward,
                                      newSettings.keyStrafeLeft, newSettings.keyStrafeRight,
                                      newSettings.keyUp, newSettings.keyDown);

              // Update orbit center to world bounds center when switching to orbit mode
              if (newSettings.mode == 0) {
                cvc::bounding_box bounds = AppState::instance().worldBounds();
                double cx = (bounds[0] + bounds[3]) * 0.5;
                double cy = (bounds[1] + bounds[4]) * 0.5;
                double cz = (bounds[2] + bounds[5]) * 0.5;
                camCtrl->setOrbitCenter(cx, cy, cz);
              }

              m_renderWidget->render();
            });
  }

  m_cameraDialog->show();
  m_cameraDialog->raise();
  m_cameraDialog->activateWindow();
}

void MainWindow::showGridOptions() {
  cvc::thread_info ti(volrover3::app(), BOOST_CURRENT_FUNCTION);

  if (!m_gridOptionsDialog) {
    m_gridOptionsDialog = new GridOptionsDialog(m_sceneGraph->getGridNode());
    m_gridOptionsDialog->setWindowTitle(tr("Grid Options - VolRover3"));
    m_gridOptionsDialog->setAttribute(Qt::WA_DeleteOnClose);
    m_gridOptionsDialog->resize(450, 600);

    // Reset pointer when dialog is closed
    connect(m_gridOptionsDialog, &QObject::destroyed, [this]() { m_gridOptionsDialog = nullptr; });
  }

  m_gridOptionsDialog->show();
  m_gridOptionsDialog->raise();
  m_gridOptionsDialog->activateWindow();
}

void MainWindow::showViewerOptions() {
  cvc::thread_info ti(volrover3::app(), BOOST_CURRENT_FUNCTION);

  if (!m_viewerOptionsDialog) {
    m_viewerOptionsDialog = new ViewerOptionsDialog(m_renderWidget, m_sceneGraph);
    m_viewerOptionsDialog->setWindowTitle(tr("Viewer Options - VolRover3"));
    m_viewerOptionsDialog->setAttribute(Qt::WA_DeleteOnClose);

    // Reset pointer when dialog is closed
    connect(m_viewerOptionsDialog, &QObject::destroyed,
            [this]() { m_viewerOptionsDialog = nullptr; });
  }

  m_viewerOptionsDialog->show();
  m_viewerOptionsDialog->raise();
  m_viewerOptionsDialog->activateWindow();
}

void MainWindow::showThreadMonitor() {
  // Create thread monitor widget as a separate window if not already created
  if (!m_threadMonitor) {
    m_threadMonitor = new ThreadMonitorWidget();
    m_threadMonitor->setWindowTitle(tr("Thread Monitor - VolRover3"));
    m_threadMonitor->setAttribute(Qt::WA_DeleteOnClose);

    // Clean up pointer when window is closed
    connect(m_threadMonitor, &QObject::destroyed, [this]() { m_threadMonitor = nullptr; });

    // Connect to thread completion signal for status bar updates
    connect(m_threadMonitor, &ThreadMonitorWidget::threadCompleted,
            [this](const QString &threadName, const QString &threadInfo) {
              statusBar()->showMessage(
                  tr("Thread '%1' completed: %2").arg(threadName).arg(threadInfo),
                  10000); // Show for 10 seconds
            });
  }

  // Show and raise the window
  m_threadMonitor->show();
  m_threadMonitor->raise();
  m_threadMonitor->activateWindow();
}

void MainWindow::showStateTree() {
  // Create state tree widget as a separate window if not already created
  if (!m_stateTreeWidget) {
    m_stateTreeWidget = new StateTreeWidget();
    m_stateTreeWidget->setWindowTitle(tr("State Tree - VolRover3"));
    m_stateTreeWidget->setAttribute(Qt::WA_DeleteOnClose);
    m_stateTreeWidget->resize(600, 500);

    // Set root state to the global state singleton
    m_stateTreeWidget->setRootState(&cvc::state::instance(volrover3::app()));

    // Clean up pointer when window is closed
    connect(m_stateTreeWidget, &QObject::destroyed, [this]() { m_stateTreeWidget = nullptr; });

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

void MainWindow::showSDF() {
  cvc::thread_info ti(volrover3::app(), BOOST_CURRENT_FUNCTION);

  // Create SDF dialog as a separate window if not already created
  if (!m_sdfDialog) {
    m_sdfDialog = new SDFDialog(m_sceneGraph);
    m_sdfDialog->setWindowTitle(tr("Signed Distance Function - VolRover3"));
    m_sdfDialog->setAttribute(Qt::WA_DeleteOnClose);

    // Clean up pointer when window is closed
    connect(m_sdfDialog, &QObject::destroyed, [this]() { m_sdfDialog = nullptr; });
  }

  // Show and raise the window
  m_sdfDialog->show();
  m_sdfDialog->raise();
  m_sdfDialog->activateWindow();
}

void MainWindow::showIsosurface() {
  cvc::thread_info ti(volrover3::app(), BOOST_CURRENT_FUNCTION);

  // Create Isosurface dialog as a separate window if not already created
  if (!m_isosurfaceDialog) {
    m_isosurfaceDialog = new IsosurfaceDialog(m_sceneGraph);
    m_isosurfaceDialog->setWindowTitle(tr("Isosurface Extraction - VolRover3"));
    m_isosurfaceDialog->setAttribute(Qt::WA_DeleteOnClose);

    // Clean up pointer when window is closed
    connect(m_isosurfaceDialog, &QObject::destroyed, [this]() { m_isosurfaceDialog = nullptr; });
  }

  // Show and raise the window
  m_isosurfaceDialog->show();
  m_isosurfaceDialog->raise();
  m_isosurfaceDialog->activateWindow();
}

void MainWindow::showGeometry() {
  cvc::thread_info ti(volrover3::app(), BOOST_CURRENT_FUNCTION);

  // Create Geometry dialog as a separate window if not already created
  if (!m_geometryDialog) {
    m_geometryDialog = new GeometryDialog(m_sceneGraph, this);
    m_geometryDialog->setWindowTitle(tr("Geometry Properties - VolRover3"));
    m_geometryDialog->setAttribute(Qt::WA_DeleteOnClose);

    // Clean up pointer when window is closed
    connect(m_geometryDialog, &QObject::destroyed, [this]() { m_geometryDialog = nullptr; });
  }

  // Show and raise the window
  m_geometryDialog->show();
  m_geometryDialog->raise();
  m_geometryDialog->activateWindow();
}

void MainWindow::showVolume() {
  cvc::thread_info ti(volrover3::app(), BOOST_CURRENT_FUNCTION);

  // Create Volume dialog as a separate window if not already created
  if (!m_volumeDialog) {
    m_volumeDialog = new VolumeDialog(m_sceneGraph, this);
    m_volumeDialog->setWindowTitle(tr("Volume Properties - VolRover3"));
    m_volumeDialog->setAttribute(Qt::WA_DeleteOnClose);

    // Clean up pointer when window is closed
    connect(m_volumeDialog, &QObject::destroyed, [this]() { m_volumeDialog = nullptr; });
  }

  // Show and raise the window
  m_volumeDialog->show();
  m_volumeDialog->raise();
  m_volumeDialog->activateWindow();
}

void MainWindow::aboutVolRover() {
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

void MainWindow::resetCamera() {
  CameraController *camCtrl = m_renderWidget->getCameraController();
  if (camCtrl) {
    // Use CameraController's resetView to properly update state tree
    cvc::bounding_box bounds = AppState::instance().worldBounds();
    camCtrl->resetView(bounds.minx, bounds.miny, bounds.minz, bounds.maxx, bounds.maxy,
                       bounds.maxz);
    m_renderWidget->render();
  }
}

void MainWindow::generateStanfordBunny() {
  cvc::thread_info ti(volrover3::app(), BOOST_CURRENT_FUNCTION);

  try {
    // Load the built-in Stanford Bunny using the .bunny extension
    // The bunny_io class handles this special extension and returns the embedded mesh
    cvc::geometry geom = cvc::read_geometry("stanford.bunny");

    // Create unique name for the geometry
    std::string baseName = "StanfordBunny";
    std::string graphicsName = baseName;
    int counter = 1;
    while (m_sceneGraph->getGraphics(graphicsName)) {
      graphicsName = baseName + "_" + std::to_string(counter++);
    }

    // Create geometry node under root
    auto graphicsNode =
        m_sceneGraph->getGraphicsRoot()->addGraphicsChild<GeometryNode>(graphicsName);
    m_sceneGraph->registerGraphics(graphicsName, graphicsNode);

    // Set geometry and metadata
    graphicsNode->setGeometry(geom);
    graphicsNode->setMetadata("type", std::string("geometry"));
    graphicsNode->setMetadata("filename", std::string("stanford.bunny"));
    graphicsNode->setMetadata("num_vertices", static_cast<int>(geom.num_points()));
    graphicsNode->setMetadata("num_triangles", static_cast<int>(geom.num_tris()));
    graphicsNode->setMetadata("source", std::string("built-in"));

    // Update world bounds and reset camera to show new geometry
    AppState::instance().setWorldBounds(geom.extents());
    resetCamera();

    statusBar()->showMessage(tr("Generated Stanford Bunny: %1 vertices, %2 triangles")
                                 .arg(geom.num_points())
                                 .arg(geom.num_tris()),
                             5000);

  } catch (const std::exception &e) {
    QMessageBox::critical(this, tr("Generation Error"),
                          tr("Failed to generate Stanford Bunny:\n%1").arg(e.what()));
  }
}

void MainWindow::generateSphere() {
  ProceduralGeometryDialog dialog(ProceduralGeometryType::Sphere, m_sceneGraph, this);
  if (dialog.exec() == QDialog::Accepted) {
    resetCamera();
    statusBar()->showMessage(tr("Generated Sphere"), 3000);
  }
}

void MainWindow::generateCube() {
  ProceduralGeometryDialog dialog(ProceduralGeometryType::Cube, m_sceneGraph, this);
  if (dialog.exec() == QDialog::Accepted) {
    resetCamera();
    statusBar()->showMessage(tr("Generated Cube"), 3000);
  }
}

void MainWindow::generateTorus() {
  ProceduralGeometryDialog dialog(ProceduralGeometryType::Torus, m_sceneGraph, this);
  if (dialog.exec() == QDialog::Accepted) {
    resetCamera();
    statusBar()->showMessage(tr("Generated Torus"), 3000);
  }
}

void MainWindow::generateCone() {
  ProceduralGeometryDialog dialog(ProceduralGeometryType::Cone, m_sceneGraph, this);
  if (dialog.exec() == QDialog::Accepted) {
    resetCamera();
    statusBar()->showMessage(tr("Generated Cone"), 3000);
  }
}

void MainWindow::setupStatusBar() {
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
  // Use QMetaObject::invokeMethod to ensure UI updates happen on the main thread
  m_connections.push_back(volrover3::app().threadsChanged.connect([this](const std::string &) {
    QMetaObject::invokeMethod(this, "updateThreadStatus", Qt::QueuedConnection);
  }));

  // Do initial update
  updateThreadStatus();
}

void MainWindow::updateThreadStatus() {
  // Get all threads
  auto threads = volrover3::app().threads();

  if (threads.empty()) {
    // No threads active - hide widgets
    m_threadNameLabel->hide();
    m_threadInfoLabel->hide();
    m_threadProgressBar->hide();
    // Don't clear message - let completion messages persist
  } else {
    // Find the best thread to display:
    // 1. Prefer running threads (progress < 1.0)
    // 2. Otherwise show the most recent thread
    std::string displayThreadKey;
    double displayProgress = -1.0;
    bool hasRunningThread = false;

    for (const auto &entry : threads) {
      const std::string &threadKey = entry.first;
      const cvc::thread_ptr &threadPtr = entry.second;

      if (!threadPtr)
        continue;

      double progress = volrover3::app().threadProgress(threadKey);
      bool isComplete = (progress >= 1.0);

      if (!isComplete) {
        // Found a running thread - prefer this
        if (!hasRunningThread || progress > displayProgress) {
          displayThreadKey = threadKey;
          displayProgress = progress;
          hasRunningThread = true;
        }
      } else if (!hasRunningThread) {
        // No running threads yet, track completed ones
        displayThreadKey = threadKey;
        displayProgress = progress;
      }
    }

    if (displayThreadKey.empty()) {
      // Fallback to first thread
      displayThreadKey = threads.begin()->first;
      displayProgress = volrover3::app().threadProgress(displayThreadKey);
    }

    // Get thread info
    std::string info = volrover3::app().threadInfo(displayThreadKey);

    // Add status indicator
    bool isComplete = (displayProgress >= 1.0);
    if (isComplete) {
      if (info.empty())
        info = "completed";
      else
        info += " (completed)";
    } else if (info.empty()) {
      info = "running...";
    }

    // Update status bar widgets
    m_threadNameLabel->setText(QString::fromStdString(displayThreadKey));
    m_threadInfoLabel->setText(QString::fromStdString(info));
    m_threadProgressBar->setValue(static_cast<int>(displayProgress * 100.0));

    // Color the progress bar based on status
    if (isComplete) {
      m_threadProgressBar->setStyleSheet("QProgressBar::chunk { background-color: #4CAF50; }");
    } else {
      m_threadProgressBar->setStyleSheet(""); // Default color
    }

    // Show widgets
    m_threadNameLabel->show();
    m_threadInfoLabel->show();
    m_threadProgressBar->show();
  }
}

void MainWindow::initializeCameraFromState() {
  CameraController *camCtrl = m_renderWidget->getCameraController();
  if (!camCtrl)
    return;

  // Load all camera settings from AppState
  camCtrl->setMode(static_cast<CameraMode>(AppState::instance().cameraMode()));
  camCtrl->setMovementSpeed(AppState::instance().cameraSpeed());
  camCtrl->setMouseSensitivity(AppState::instance().cameraSensitivity());
  camCtrl->setInvertMouse(AppState::instance().cameraInvertMouse());
  camCtrl->setKeyBindings(
      AppState::instance().cameraKeyForward(), AppState::instance().cameraKeyBackward(),
      AppState::instance().cameraKeyLeft(), AppState::instance().cameraKeyRight(),
      AppState::instance().cameraKeyUp(), AppState::instance().cameraKeyDown());

  // Camera state is now managed entirely through CameraController's state tree
  // No need to load from AppState

  // Set orbit center to world bounds center
  cvc::bounding_box bounds = AppState::instance().worldBounds();
  double cx = (bounds[0] + bounds[3]) / 2.0;
  double cy = (bounds[1] + bounds[4]) / 2.0;
  double cz = (bounds[2] + bounds[5]) / 2.0;
  camCtrl->setOrbitCenter(cx, cy, cz);
}
