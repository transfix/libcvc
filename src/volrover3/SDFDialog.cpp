#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QMetaObject>
#include <QProgressBar>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>
#include <cvc/algorithm.h>
#include <cvc/app.h>
#include <cvc/geometry.h>
#include <cvc/state.h>
#include <cvc/volmagick.h>
#include <volrover3/GeometryNode.h>
#include <volrover3/GraphicsNode.h>
#include <volrover3/SDFDialog.h>
#include <volrover3/SceneGraph.h>
#include <volrover3/VolumeNode.h>
#include <volrover3/volrover3_app.h>

SDFDialog::SDFDialog(std::shared_ptr<SceneGraph> sceneGraph, QWidget *parent)
    : QDialog(parent), m_sceneGraph(sceneGraph), m_geometryComboBox(nullptr),
      m_dimXSpinBox(nullptr), m_dimYSpinBox(nullptr), m_dimZSpinBox(nullptr),
      m_algorithmComboBox(nullptr), m_flipNormalsCheckBox(nullptr), m_useBoundsCheckBox(nullptr),
      m_minXSpinBox(nullptr), m_minYSpinBox(nullptr), m_minZSpinBox(nullptr),
      m_maxXSpinBox(nullptr), m_maxYSpinBox(nullptr), m_maxZSpinBox(nullptr),
      m_computeButton(nullptr), m_cancelButton(nullptr), m_progressBar(nullptr),
      m_statusLabel(nullptr), m_computing(false) {
  setWindowTitle(tr("Signed Distance Function"));
  setMinimumWidth(400);
  setupUI();
  connectSignals();
  populateGeometryList();

  // Connect to state tree to monitor for new geometry
  // Listen to graphics root's children changes
  if (m_sceneGraph) {
    std::string statePrefix = m_sceneGraph->getStatePrefix();
    std::string graphicsRootPath = statePrefix + ".graphics.root.children";

    m_graphicsChildrenConnection =
        cvc::state::instance()(graphicsRootPath).childChanged.connect([this](const std::string &) {
          // Post to Qt event loop to ensure thread safety
          QMetaObject::invokeMethod(this, "onGraphicsChildrenChanged", Qt::QueuedConnection);
        });
  }
}

void SDFDialog::setupUI() {
  QVBoxLayout *mainLayout = new QVBoxLayout(this);

  // Geometry Selection Group
  QGroupBox *geomGroup = new QGroupBox(tr("Input Geometry"), this);
  QFormLayout *geomLayout = new QFormLayout(geomGroup);

  m_geometryComboBox = new QComboBox(this);
  geomLayout->addRow(tr("Geometry:"), m_geometryComboBox);

  mainLayout->addWidget(geomGroup);

  // Grid Dimensions Group
  QGroupBox *dimGroup = new QGroupBox(tr("Grid Dimensions"), this);
  QFormLayout *dimLayout = new QFormLayout(dimGroup);

  m_dimXSpinBox = new QSpinBox(this);
  m_dimXSpinBox->setRange(8, 1024);
  m_dimXSpinBox->setValue(128);
  dimLayout->addRow(tr("X Dimension:"), m_dimXSpinBox);

  m_dimYSpinBox = new QSpinBox(this);
  m_dimYSpinBox->setRange(8, 1024);
  m_dimYSpinBox->setValue(128);
  dimLayout->addRow(tr("Y Dimension:"), m_dimYSpinBox);

  m_dimZSpinBox = new QSpinBox(this);
  m_dimZSpinBox->setRange(8, 1024);
  m_dimZSpinBox->setValue(128);
  dimLayout->addRow(tr("Z Dimension:"), m_dimZSpinBox);

  mainLayout->addWidget(dimGroup);

  // Algorithm Options Group
  QGroupBox *algoGroup = new QGroupBox(tr("Algorithm Options"), this);
  QFormLayout *algoLayout = new QFormLayout(algoGroup);

  m_algorithmComboBox = new QComboBox(this);
  m_algorithmComboBox->addItem(tr("SDF v1 (Default)"), static_cast<int>(cvc::SDF_V1));
  m_algorithmComboBox->addItem(tr("SDF v2 (Faster)"), static_cast<int>(cvc::SDF_V2));
  algoLayout->addRow(tr("Algorithm:"), m_algorithmComboBox);

  m_flipNormalsCheckBox = new QCheckBox(tr("Flip normals (invert inside/outside)"), this);
  algoLayout->addRow(m_flipNormalsCheckBox);

  mainLayout->addWidget(algoGroup);

  // Bounding Box Group
  QGroupBox *bboxGroup = new QGroupBox(tr("Bounding Box"), this);
  QVBoxLayout *bboxLayout = new QVBoxLayout(bboxGroup);

  m_useBoundsCheckBox =
      new QCheckBox(tr("Use custom bounding box (unchecked = use geometry extents)"), this);
  bboxLayout->addWidget(m_useBoundsCheckBox);

  QFormLayout *boundsLayout = new QFormLayout();

  QHBoxLayout *minLayout = new QHBoxLayout();
  m_minXSpinBox = new QDoubleSpinBox(this);
  m_minXSpinBox->setRange(-10000.0, 10000.0);
  m_minXSpinBox->setValue(0.0);
  m_minXSpinBox->setEnabled(false);
  minLayout->addWidget(new QLabel(tr("X:"), this));
  minLayout->addWidget(m_minXSpinBox);

  m_minYSpinBox = new QDoubleSpinBox(this);
  m_minYSpinBox->setRange(-10000.0, 10000.0);
  m_minYSpinBox->setValue(0.0);
  m_minYSpinBox->setEnabled(false);
  minLayout->addWidget(new QLabel(tr("Y:"), this));
  minLayout->addWidget(m_minYSpinBox);

  m_minZSpinBox = new QDoubleSpinBox(this);
  m_minZSpinBox->setRange(-10000.0, 10000.0);
  m_minZSpinBox->setValue(0.0);
  m_minZSpinBox->setEnabled(false);
  minLayout->addWidget(new QLabel(tr("Z:"), this));
  minLayout->addWidget(m_minZSpinBox);

  boundsLayout->addRow(tr("Min:"), minLayout);

  QHBoxLayout *maxLayout = new QHBoxLayout();
  m_maxXSpinBox = new QDoubleSpinBox(this);
  m_maxXSpinBox->setRange(-10000.0, 10000.0);
  m_maxXSpinBox->setValue(1.0);
  m_maxXSpinBox->setEnabled(false);
  maxLayout->addWidget(new QLabel(tr("X:"), this));
  maxLayout->addWidget(m_maxXSpinBox);

  m_maxYSpinBox = new QDoubleSpinBox(this);
  m_maxYSpinBox->setRange(-10000.0, 10000.0);
  m_maxYSpinBox->setValue(1.0);
  m_maxYSpinBox->setEnabled(false);
  maxLayout->addWidget(new QLabel(tr("Y:"), this));
  maxLayout->addWidget(m_maxYSpinBox);

  m_maxZSpinBox = new QDoubleSpinBox(this);
  m_maxZSpinBox->setRange(-10000.0, 10000.0);
  m_maxZSpinBox->setValue(1.0);
  m_maxZSpinBox->setEnabled(false);
  maxLayout->addWidget(new QLabel(tr("Z:"), this));
  maxLayout->addWidget(m_maxZSpinBox);

  boundsLayout->addRow(tr("Max:"), maxLayout);

  bboxLayout->addLayout(boundsLayout);
  mainLayout->addWidget(bboxGroup);

  // Progress Group
  QGroupBox *progressGroup = new QGroupBox(tr("Progress"), this);
  QVBoxLayout *progressLayout = new QVBoxLayout(progressGroup);

  m_statusLabel = new QLabel(tr("Ready"), this);
  progressLayout->addWidget(m_statusLabel);

  m_progressBar = new QProgressBar(this);
  m_progressBar->setRange(0, 100);
  m_progressBar->setValue(0);
  progressLayout->addWidget(m_progressBar);

  mainLayout->addWidget(progressGroup);

  // Button box
  QHBoxLayout *buttonLayout = new QHBoxLayout();
  buttonLayout->addStretch();

  m_computeButton = new QPushButton(tr("Compute SDF"), this);
  m_computeButton->setDefault(true);
  buttonLayout->addWidget(m_computeButton);

  m_cancelButton = new QPushButton(tr("Close"), this);
  buttonLayout->addWidget(m_cancelButton);

  mainLayout->addLayout(buttonLayout);

  setLayout(mainLayout);
}

void SDFDialog::connectSignals() {
  connect(m_geometryComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &SDFDialog::onGeometrySelected);
  connect(m_computeButton, &QPushButton::clicked, this, &SDFDialog::onComputeClicked);
  connect(m_cancelButton, &QPushButton::clicked, this, &QDialog::reject);

  connect(m_useBoundsCheckBox, &QCheckBox::toggled, [this](bool checked) {
    m_minXSpinBox->setEnabled(checked);
    m_minYSpinBox->setEnabled(checked);
    m_minZSpinBox->setEnabled(checked);
    m_maxXSpinBox->setEnabled(checked);
    m_maxYSpinBox->setEnabled(checked);
    m_maxZSpinBox->setEnabled(checked);
  });
}

void SDFDialog::populateGeometryList() {
  m_geometryComboBox->clear();
  m_geometryNames.clear();

  if (!m_sceneGraph)
    return;

  // Get all geometry nodes recursively
  auto allGeometries = m_sceneGraph->getAllGeometryGraphics();

  for (const auto &geomNode : allGeometries) {
    if (geomNode && geomNode->getGeometry() && !geomNode->getGeometry()->empty()) {
      std::string name = geomNode->getName();
      m_geometryNames.push_back(name);
      m_geometryComboBox->addItem(QString::fromStdString(name));
    }
  }

  if (m_geometryComboBox->count() == 0) {
    m_computeButton->setEnabled(false);
    m_statusLabel->setText(tr("No geometry available"));
  } else {
    onGeometrySelected(0);
  }
}

void SDFDialog::onGraphicsChildrenChanged() {
  if (!m_sceneGraph)
    return;

  // Get current geometry count
  size_t currentCount = m_geometryNames.size();

  // Count geometry nodes in scene graph recursively
  size_t sceneGeomCount = 0;
  auto allGeometries = m_sceneGraph->getAllGeometryGraphics();
  for (const auto &geomNode : allGeometries) {
    if (geomNode && geomNode->getGeometry() && !geomNode->getGeometry()->empty()) {
      sceneGeomCount++;
    }
  }

  // If counts differ, refresh the list
  if (sceneGeomCount != currentCount) {
    // Save current selection
    QString currentSelection;
    int currentIndex = m_geometryComboBox->currentIndex();
    if (currentIndex >= 0 && currentIndex < static_cast<int>(m_geometryNames.size())) {
      currentSelection = QString::fromStdString(m_geometryNames[currentIndex]);
    }

    // Refresh the list
    populateGeometryList();

    // Try to restore the previous selection
    if (!currentSelection.isEmpty()) {
      int newIndex = m_geometryComboBox->findText(currentSelection);
      if (newIndex >= 0) {
        m_geometryComboBox->setCurrentIndex(newIndex);
      }
    }

    // Update status if geometry is now available
    if (m_geometryComboBox->count() > 0 && !m_computing) {
      m_computeButton->setEnabled(true);
      if (m_statusLabel->text() == tr("No geometry available")) {
        m_statusLabel->setText(tr("Ready"));
      }
    }
  }
}

void SDFDialog::onGeometrySelected(int index) {
  if (index < 0 || index >= static_cast<int>(m_geometryNames.size()))
    return;

  const std::string &geomName = m_geometryNames[index];
  auto graphicsNode = m_sceneGraph->getGraphics(geomName);
  auto geomNode = std::dynamic_pointer_cast<GeometryNode>(graphicsNode);

  if (!geomNode)
    return;

  // Update bounding box fields with geometry extents
  const cvc::geometry *geom = geomNode->getGeometry();
  if (!geom)
    return;

  cvc::bounding_box bbox = geom->extents();

  m_minXSpinBox->setValue(bbox.minx);
  m_minYSpinBox->setValue(bbox.miny);
  m_minZSpinBox->setValue(bbox.minz);
  m_maxXSpinBox->setValue(bbox.maxx);
  m_maxYSpinBox->setValue(bbox.maxy);
  m_maxZSpinBox->setValue(bbox.maxz);
}

void SDFDialog::onComputeClicked() {
  cvc::thread_info ti(volrover3::app(), BOOST_CURRENT_FUNCTION);

  if (m_computing) {
    // Cancel ongoing computation
    if (!m_activeThreadKey.empty()) {
      volrover3::app().threads(m_activeThreadKey)->interrupt();
    }
    setControlsEnabled(true);
    m_statusLabel->setText(tr("Cancelled"));
    m_progressBar->setValue(0);
    m_computing = false;
    m_computeButton->setText(tr("Compute SDF"));
    return;
  }

  if (m_geometryComboBox->currentIndex() < 0) {
    QMessageBox::warning(this, tr("No Geometry"), tr("Please select a geometry to compute SDF."));
    return;
  }

  const std::string &geomName = m_geometryNames[m_geometryComboBox->currentIndex()];
  auto graphicsNode = m_sceneGraph->getGraphics(geomName);
  auto geomNode = std::dynamic_pointer_cast<GeometryNode>(graphicsNode);

  if (!geomNode) {
    QMessageBox::critical(this, tr("Error"), tr("Failed to get geometry node."));
    return;
  }

  // Get parameters
  cvc::dimension dim(m_dimXSpinBox->value(), m_dimYSpinBox->value(), m_dimZSpinBox->value());

  cvc::bounding_box bbox;
  if (m_useBoundsCheckBox->isChecked()) {
    bbox =
        cvc::bounding_box(m_minXSpinBox->value(), m_minYSpinBox->value(), m_minZSpinBox->value(),
                          m_maxXSpinBox->value(), m_maxYSpinBox->value(), m_maxZSpinBox->value());
  } else {
    const cvc::geometry *geomPtr = geomNode->getGeometry();
    if (!geomPtr) {
      QMessageBox::critical(this, tr("Error"), tr("Failed to get geometry."));
      return;
    }
    bbox = geomPtr->extents();
  }

  cvc::sdf_algorithm algorithm =
      static_cast<cvc::sdf_algorithm>(m_algorithmComboBox->currentData().toInt());

  bool flipNormals = m_flipNormalsCheckBox->isChecked();

  // Copy geometry for thread safety
  const cvc::geometry *geomPtr = geomNode->getGeometry();
  if (!geomPtr) {
    QMessageBox::critical(this, tr("Error"), tr("Failed to get geometry."));
    return;
  }
  cvc::geometry geom = *geomPtr;

  // Update UI
  setControlsEnabled(false);
  m_computing = true;
  m_computeButton->setText(tr("Cancel"));
  m_statusLabel->setText(tr("Computing SDF..."));
  m_progressBar->setValue(0);

  // Create unique thread key
  m_activeThreadKey = "sdf_computation_" + geomName;

  // Start computation in background thread
  volrover3::app().startThread(
      m_activeThreadKey,
      [this, geom, dim, bbox, algorithm, flipNormals, geomName, activeKey = m_activeThreadKey]() {
        // Use thread_feedback for proper progress tracking (must be at thread entry point)
        cvc::app::thread_feedback feedback(volrover3::app(), activeKey);

        try {
          // Update progress to indicate we've started
          volrover3::app().threadProgress(activeKey, 0.1);
          volrover3::app().threadInfo(activeKey, "Computing SDF...");

          // Compute SDF (this is safe to do in background thread)
          cvc::volume sdfVol = cvc::sdf(volrover3::app(), geom, dim, bbox, algorithm, flipNormals);

          // Update progress
          volrover3::app().threadProgress(activeKey, 0.9);
          volrover3::app().threadInfo(activeKey, "Adding volume to scene...");

          // SDF computation complete, now adding to scene
          QMetaObject::invokeMethod(
              this, [this]() { m_statusLabel->setText(tr("Adding SDF volume to scene...")); },
              Qt::QueuedConnection);

          // Post all SceneGraph/VTK operations to main thread via SceneGraph event queue
          // Capture activeKey for finish call
          m_sceneGraph->postEvent([this, sdfVol, geomName, activeKey]() {
            try {
              // Sanitize the name to ensure it's a valid C identifier
              std::string rawName = geomName + "_sdf";
              std::string sdfName = cvc::state::sanitizeStateName(rawName);

              // Check if SDF volume already exists
              auto existingNode = m_sceneGraph->getGraphics(sdfName);
              if (existingNode) {
                // Remove existing SDF volume
                m_sceneGraph->removeGraphics(sdfName);
              }

              // Get the parent geometry node first
              auto geomNode = m_sceneGraph->getGraphics(geomName);
              if (!geomNode) {
                throw std::runtime_error("Geometry node not found");
              }

              // Add SDF volume as child of geometry using the template createChild method
              auto sdfNode = geomNode->createChild<VolumeNode>(sdfName, sdfVol);

              if (!sdfNode) {
                throw std::runtime_error("Failed to create SDF volume node");
              }

              // Mark thread as finished
              volrover3::app().finishThreadProgress(activeKey);

              // Update UI on Qt thread
              QMetaObject::invokeMethod(
                  this, [this]() { onComputeFinished(true, "SDF computed successfully"); },
                  Qt::QueuedConnection);
            } catch (const std::exception &e) {
              std::string errorMsg = std::string("Failed to create volume node: ") + e.what();
              volrover3::app().finishThreadProgress(activeKey);
              QMetaObject::invokeMethod(
                  this, [this, errorMsg]() { onComputeFinished(false, errorMsg); },
                  Qt::QueuedConnection);
            }
          });

        } catch (const boost::thread_interrupted &) {
          QMetaObject::invokeMethod(
              this, [this]() { onComputeFinished(false, "Computation cancelled"); },
              Qt::QueuedConnection);
        } catch (const std::exception &e) {
          std::string errorMsg = std::string("Error: ") + e.what();
          QMetaObject::invokeMethod(
              this, [this, errorMsg]() { onComputeFinished(false, errorMsg); },
              Qt::QueuedConnection);
        }
      },
      false // Don't wait for existing thread
  );
}

void SDFDialog::updateProgress(int value) { m_progressBar->setValue(value); }

void SDFDialog::onComputeFinished(bool success, const std::string &message) {
  setControlsEnabled(true);
  m_computing = false;
  m_computeButton->setText(tr("Compute SDF"));
  m_progressBar->setValue(success ? 100 : 0);
  m_statusLabel->setText(QString::fromStdString(message));

  if (success) {
    QMessageBox::information(this, tr("Success"), QString::fromStdString(message));
  } else {
    QMessageBox::warning(this, tr("Error"), QString::fromStdString(message));
  }
}

void SDFDialog::setControlsEnabled(bool enabled) {
  m_geometryComboBox->setEnabled(enabled);
  m_dimXSpinBox->setEnabled(enabled);
  m_dimYSpinBox->setEnabled(enabled);
  m_dimZSpinBox->setEnabled(enabled);
  m_algorithmComboBox->setEnabled(enabled);
  m_flipNormalsCheckBox->setEnabled(enabled);
  m_useBoundsCheckBox->setEnabled(enabled);

  bool boundsEnabled = enabled && m_useBoundsCheckBox->isChecked();
  m_minXSpinBox->setEnabled(boundsEnabled);
  m_minYSpinBox->setEnabled(boundsEnabled);
  m_minZSpinBox->setEnabled(boundsEnabled);
  m_maxXSpinBox->setEnabled(boundsEnabled);
  m_maxYSpinBox->setEnabled(boundsEnabled);
  m_maxZSpinBox->setEnabled(boundsEnabled);
}
