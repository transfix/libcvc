#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QShowEvent>
#include <QVBoxLayout>
#include <volrover3/AppState.h>
#include <volrover3/SceneGraph.h>
#include <volrover3/VTKRenderWidget.h>
#include <volrover3/ViewerOptionsDialog.h>

ViewerOptionsDialog::ViewerOptionsDialog(VTKRenderWidget *renderWidget,
                                         std::shared_ptr<SceneGraph> sceneGraph, QWidget *parent)
    : QWidget(parent, Qt::Window), m_renderWidget(renderWidget), m_sceneGraph(sceneGraph),
      m_showFPSCheckBox(nullptr), m_graphicsRootComboBox(nullptr), m_refreshRootsButton(nullptr),
      m_cameraComboBox(nullptr), m_refreshCamerasButton(nullptr) {
  setupUI();
  connectSignals();
  loadFromState();
}

ViewerOptionsDialog::~ViewerOptionsDialog() {
  for (auto &conn : m_connections) {
    conn.disconnect();
  }
}

void ViewerOptionsDialog::setupUI() {
  QVBoxLayout *mainLayout = new QVBoxLayout(this);

  // Display Options Group
  QGroupBox *displayGroup = new QGroupBox(tr("Display Options"));
  QFormLayout *displayLayout = new QFormLayout(displayGroup);

  m_showFPSCheckBox = new QCheckBox(tr("Show FPS Counter"));
  m_showFPSCheckBox->setToolTip(
      tr("Display frames per second in the top-left corner of the viewer"));
  displayLayout->addRow(m_showFPSCheckBox);

  mainLayout->addWidget(displayGroup);

  // Scene Selection Group
  QGroupBox *sceneGroup = new QGroupBox(tr("Scene Selection"));
  QFormLayout *sceneLayout = new QFormLayout(sceneGroup);

  QHBoxLayout *rootsLayout = new QHBoxLayout();
  m_graphicsRootComboBox = new QComboBox();
  m_graphicsRootComboBox->setToolTip(
      tr("Select which graphics scene to render (for future multi-scene support)"));
  m_graphicsRootComboBox->setMinimumWidth(200);
  rootsLayout->addWidget(m_graphicsRootComboBox);
  m_refreshRootsButton = new QPushButton(tr("↻"));
  m_refreshRootsButton->setToolTip(tr("Refresh the list of available graphics roots"));
  m_refreshRootsButton->setMaximumWidth(30);
  rootsLayout->addWidget(m_refreshRootsButton);
  sceneLayout->addRow(tr("Graphics Root:"), rootsLayout);

  QHBoxLayout *cameraLayout = new QHBoxLayout();
  m_cameraComboBox = new QComboBox();
  m_cameraComboBox->setToolTip(tr("Select which camera to use (for future multi-camera support)"));
  m_cameraComboBox->setMinimumWidth(200);
  cameraLayout->addWidget(m_cameraComboBox);
  m_refreshCamerasButton = new QPushButton(tr("↻"));
  m_refreshCamerasButton->setToolTip(tr("Refresh the list of available cameras"));
  m_refreshCamerasButton->setMaximumWidth(30);
  cameraLayout->addWidget(m_refreshCamerasButton);
  sceneLayout->addRow(tr("Camera:"), cameraLayout);

  mainLayout->addWidget(sceneGroup);

  // Add stretch to push everything to the top
  mainLayout->addStretch();

  // Set window properties
  setWindowTitle(tr("Viewer Options"));
  resize(350, 200);
}

void ViewerOptionsDialog::connectSignals() {
  connect(m_showFPSCheckBox, &QCheckBox::toggled, this, &ViewerOptionsDialog::onShowFPSChanged);

  connect(m_graphicsRootComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &ViewerOptionsDialog::onGraphicsRootChanged);

  connect(m_cameraComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &ViewerOptionsDialog::onCameraChanged);

  connect(m_refreshRootsButton, &QPushButton::clicked, this,
          &ViewerOptionsDialog::refreshGraphicsRoots);

  connect(m_refreshCamerasButton, &QPushButton::clicked, this,
          &ViewerOptionsDialog::refreshCameras);
}

void ViewerOptionsDialog::loadFromState() {
  // Load FPS display setting from AppState
  bool showFPS = AppState::instance().showFPS();
  m_showFPSCheckBox->setChecked(showFPS);

  // Populate combo boxes
  refreshGraphicsRoots();
  refreshCameras();
}

void ViewerOptionsDialog::showEvent(QShowEvent *event) {
  loadFromState();
  QWidget::showEvent(event);
}

void ViewerOptionsDialog::closeEvent(QCloseEvent *event) { QWidget::closeEvent(event); }

void ViewerOptionsDialog::onShowFPSChanged(bool checked) {
  AppState::instance().setShowFPS(checked);

  // Update the render widget to show/hide FPS display
  if (m_renderWidget) {
    m_renderWidget->setShowFPS(checked);
  }
}

void ViewerOptionsDialog::onGraphicsRootChanged(int index) {
  Q_UNUSED(index);
  // Currently only one graphics root is supported
  // This is a placeholder for future multi-scene support
}

void ViewerOptionsDialog::onCameraChanged(int index) {
  Q_UNUSED(index);
  // Currently only one camera is supported
  // This is a placeholder for future multi-camera support
}

void ViewerOptionsDialog::refreshGraphicsRoots() {
  m_graphicsRootComboBox->clear();

  if (m_sceneGraph) {
    std::string statePrefix = m_sceneGraph->getStatePrefix();
    // Currently only one graphics root is supported
    m_graphicsRootComboBox->addItem(QString::fromStdString(statePrefix + ".graphics.root"));
  }

  // Disable combo box if only one option
  m_graphicsRootComboBox->setEnabled(m_graphicsRootComboBox->count() > 1);
}

void ViewerOptionsDialog::refreshCameras() {
  m_cameraComboBox->clear();

  if (m_sceneGraph) {
    std::string statePrefix = m_sceneGraph->getStatePrefix();
    // Currently only one camera is supported
    m_cameraComboBox->addItem(QString::fromStdString(statePrefix + ".camera"));
  }

  // Disable combo box if only one option
  m_cameraComboBox->setEnabled(m_cameraComboBox->count() > 1);
}
