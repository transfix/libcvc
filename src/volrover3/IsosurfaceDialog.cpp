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
#include <cvc/state.h>
#include <cvc/volmagick.h>
#include <volrover3/GeometryNode.h>
#include <volrover3/GraphicsNode.h>
#include <volrover3/IsosurfaceDialog.h>
#include <volrover3/SceneGraph.h>
#include <volrover3/VolumeNode.h>
#include <volrover3/volrover3_app.h>

IsosurfaceDialog::IsosurfaceDialog(std::shared_ptr<SceneGraph> sceneGraph, QWidget *parent)
    : QDialog(parent), m_sceneGraph(sceneGraph), m_volumeComboBox(nullptr),
      m_isovalueSpinBox(nullptr), m_methodComboBox(nullptr), m_improveIterationsSpinBox(nullptr),
      m_normalTypeComboBox(nullptr), m_computeButton(nullptr), m_cancelButton(nullptr),
      m_progressBar(nullptr), m_statusLabel(nullptr), m_computing(false) {
  setWindowTitle(tr("Isosurface Extraction"));
  setMinimumWidth(400);
  setupUI();
  connectSignals();
  populateVolumeList();

  // Connect to SceneGraph signal to monitor for new volumes
  if (m_sceneGraph) {
    m_graphicsChangedConnection = m_sceneGraph->graphicsChanged.connect([this]() {
      QMetaObject::invokeMethod(this, "onGraphicsChildrenChanged", Qt::QueuedConnection);
    });
  }
}

void IsosurfaceDialog::setupUI() {
  QVBoxLayout *mainLayout = new QVBoxLayout(this);

  // Volume Selection Group
  QGroupBox *volumeGroup = new QGroupBox(tr("Input Volume"), this);
  QFormLayout *volumeLayout = new QFormLayout(volumeGroup);

  m_volumeComboBox = new QComboBox(this);
  volumeLayout->addRow(tr("Volume:"), m_volumeComboBox);

  mainLayout->addWidget(volumeGroup);

  // Isosurface Parameters Group
  QGroupBox *paramGroup = new QGroupBox(tr("Extraction Parameters"), this);
  QFormLayout *paramLayout = new QFormLayout(paramGroup);

  m_isovalueSpinBox = new QDoubleSpinBox(this);
  m_isovalueSpinBox->setRange(-1e10, 1e10);
  m_isovalueSpinBox->setDecimals(6);
  m_isovalueSpinBox->setValue(0.0);
  paramLayout->addRow(tr("Isovalue:"), m_isovalueSpinBox);

  m_methodComboBox = new QComboBox(this);
  m_methodComboBox->addItem(tr("DualLib (Default)"), static_cast<int>(cvc::DUALLIB));
  m_methodComboBox->addItem(tr("Fast Contouring"), static_cast<int>(cvc::FASTCONTOURING));
  m_methodComboBox->addItem(tr("Lib IsoContour"), static_cast<int>(cvc::LIBISOCONTOUR));
  paramLayout->addRow(tr("Method:"), m_methodComboBox);

  m_improveIterationsSpinBox = new QSpinBox(this);
  m_improveIterationsSpinBox->setRange(0, 100);
  m_improveIterationsSpinBox->setValue(0);
  m_improveIterationsSpinBox->setToolTip(
      tr("Number of mesh improvement iterations (0 = no improvement)"));
  paramLayout->addRow(tr("Improve Iterations:"), m_improveIterationsSpinBox);

  m_normalTypeComboBox = new QComboBox(this);
  m_normalTypeComboBox->addItem(tr("B-Spline Convolution"),
                                static_cast<int>(cvc::BSPLINE_CONVOLUTION));
  m_normalTypeComboBox->addItem(tr("Central Difference"),
                                static_cast<int>(cvc::CENTRAL_DIFFERENCE));
  m_normalTypeComboBox->addItem(tr("B-Spline Interpolation"),
                                static_cast<int>(cvc::BSPLINE_INTERPOLATION));
  paramLayout->addRow(tr("Normal Type:"), m_normalTypeComboBox);

  mainLayout->addWidget(paramGroup);

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

  m_computeButton = new QPushButton(tr("Extract Isosurface"), this);
  m_computeButton->setDefault(true);
  buttonLayout->addWidget(m_computeButton);

  m_cancelButton = new QPushButton(tr("Close"), this);
  buttonLayout->addWidget(m_cancelButton);

  mainLayout->addLayout(buttonLayout);

  setLayout(mainLayout);
}

void IsosurfaceDialog::connectSignals() {
  connect(m_volumeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &IsosurfaceDialog::onVolumeSelected);
  connect(m_computeButton, &QPushButton::clicked, this, &IsosurfaceDialog::onComputeClicked);
  connect(m_cancelButton, &QPushButton::clicked, this, &QDialog::reject);
}

void IsosurfaceDialog::populateVolumeList() {
  m_volumeComboBox->clear();
  m_volumePaths.clear();

  if (!m_sceneGraph)
    return;

  // Get all volume nodes recursively
  auto allVolumes = m_sceneGraph->getAllVolumeGraphics();

  for (const auto &volumeNode : allVolumes) {
    if (volumeNode) {
      // Use full state tree path for uniqueness
      std::string fullPath = volumeNode->getState().fullName();
      m_volumePaths.push_back(fullPath);

      // Display the node name in the combo box
      m_volumeComboBox->addItem(QString::fromStdString(volumeNode->getName()));
    }
  }

  if (m_volumeComboBox->count() == 0) {
    m_computeButton->setEnabled(false);
    m_statusLabel->setText(tr("No volumes available"));
  } else {
    onVolumeSelected(0);
  }
}

void IsosurfaceDialog::onGraphicsChildrenChanged() {
  if (!m_sceneGraph)
    return;

  // Save current selection (by path)
  QString currentSelection;
  int currentIndex = m_volumeComboBox->currentIndex();
  if (currentIndex >= 0 && currentIndex < static_cast<int>(m_volumePaths.size())) {
    currentSelection = QString::fromStdString(m_volumePaths[currentIndex]);
  }

  // Refresh the list
  populateVolumeList();

  // Try to restore the previous selection by matching path
  bool selectionRestored = false;
  if (!currentSelection.isEmpty()) {
    for (int i = 0; i < static_cast<int>(m_volumePaths.size()); ++i) {
      if (QString::fromStdString(m_volumePaths[i]) == currentSelection) {
        m_volumeComboBox->setCurrentIndex(i);
        selectionRestored = true;
        break;
      }
    }
  }

  // Update UI state based on volume availability
  if (m_volumeComboBox->count() > 0 && !m_computing) {
    m_computeButton->setEnabled(true);
    if (m_statusLabel->text() == tr("No volumes available")) {
      m_statusLabel->setText(tr("Ready"));
    }
  } else if (m_volumeComboBox->count() == 0) {
    m_computeButton->setEnabled(false);
    if (!m_computing) {
      m_statusLabel->setText(tr("No volumes available"));
    }
  }
}

void IsosurfaceDialog::onVolumeSelected(int index) {
  if (index < 0 || index >= static_cast<int>(m_volumePaths.size()))
    return;

  // Could update isovalue based on volume's data range
  // For now, just ensure it's a valid selection
  const std::string &volumePath = m_volumePaths[index];

  // Get the volume node to access its data range
  auto allVolumes = m_sceneGraph->getAllVolumeGraphics();
  for (const auto &volumeNode : allVolumes) {
    if (volumeNode && volumeNode->getState().fullName() == volumePath) {
      // Try to get data range from metadata
      auto minVal = volumeNode->getMetadata("data_min");
      auto maxVal = volumeNode->getMetadata("data_max");

      if (minVal.has_value() && maxVal.has_value()) {
        try {
          double dataMin = 0.0, dataMax = 1.0;

          if (minVal.type() == typeid(double)) {
            dataMin = std::any_cast<double>(minVal);
            dataMax = std::any_cast<double>(maxVal);
          } else {
            dataMin = std::stod(std::any_cast<std::string>(minVal));
            dataMax = std::stod(std::any_cast<std::string>(maxVal));
          }

          // Set isovalue to middle of range
          m_isovalueSpinBox->setValue((dataMin + dataMax) / 2.0);
          m_isovalueSpinBox->setRange(dataMin - (dataMax - dataMin), dataMax + (dataMax - dataMin));
        } catch (...) {
          // Ignore conversion errors, keep default range
        }
      }
      break;
    }
  }
}

void IsosurfaceDialog::onComputeClicked() {
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
    m_computeButton->setText(tr("Extract Isosurface"));
    return;
  }

  int currentIndex = m_volumeComboBox->currentIndex();
  if (currentIndex < 0 || currentIndex >= static_cast<int>(m_volumePaths.size())) {
    QMessageBox::warning(this, tr("No Volume"),
                         tr("Please select a volume to extract isosurface."));
    return;
  }

  const std::string &volumePath = m_volumePaths[currentIndex];

  // Find the volume node
  std::shared_ptr<VolumeNode> volumeNode;
  std::string volumeName;
  auto allVolumes = m_sceneGraph->getAllVolumeGraphics();
  for (const auto &volNode : allVolumes) {
    if (volNode && volNode->getState().fullName() == volumePath) {
      volumeNode = volNode;
      volumeName = volNode->getName();
      break;
    }
  }

  if (!volumeNode) {
    QMessageBox::critical(this, tr("Error"), tr("Failed to get volume node."));
    return;
  }

  // Get the volume data
  const cvc::volume *volPtr = volumeNode->getVolume();
  if (!volPtr) {
    QMessageBox::critical(this, tr("Error"), tr("Volume has no data loaded."));
    return;
  }

  // Get parameters
  double isovalue = m_isovalueSpinBox->value();
  cvc::extraction_method method =
      static_cast<cvc::extraction_method>(m_methodComboBox->currentData().toInt());
  int improveIterations = m_improveIterationsSpinBox->value();
  cvc::normal_type normalType =
      static_cast<cvc::normal_type>(m_normalTypeComboBox->currentData().toInt());

  // Copy volume for thread safety
  cvc::volume vol = *volPtr;

  // Update UI
  setControlsEnabled(false);
  m_computing = true;
  m_computeButton->setText(tr("Cancel"));
  m_statusLabel->setText(tr("Extracting isosurface..."));
  m_progressBar->setValue(0);

  // Create unique thread key
  m_activeThreadKey = "iso_extraction_" + volumeName;

  // Start computation in background thread
  volrover3::app().startThread(
      m_activeThreadKey,
      [this, vol, isovalue, method, improveIterations, normalType, volumeName, volumeNode]() {
        cvc::thread_info ti(volrover3::app(), "Isosurface Extraction");

        try {
          // Extract isosurface (thread-safe)
          cvc::geometry isoGeom = cvc::iso(vol, isovalue, method, improveIterations, normalType);

          // Extraction complete, now adding to scene
          QMetaObject::invokeMethod(
              this, [this]() { m_statusLabel->setText(tr("Adding isosurface to scene...")); },
              Qt::QueuedConnection);

          // Post all SceneGraph/VTK operations to main thread via SceneGraph event queue
          m_sceneGraph->postEvent([this, isoGeom, volumeName, isovalue, volumeNode]() {
            cvc::thread_info ti(volrover3::app(), "Add Isosurface");

            try {
              // Sanitize the name to ensure it's a valid C identifier
              std::string rawName = volumeName + "_iso_" + std::to_string(isovalue);
              std::string isoName = cvc::state::sanitizeStateName(rawName);

              // Check if isosurface already exists
              auto existingNode = m_sceneGraph->getGraphics(isoName);
              if (existingNode) {
                // Remove existing isosurface
                m_sceneGraph->removeGraphics(isoName);
              }

              // Add isosurface as child of volume using the template createChild method
              auto isoNode = volumeNode->createChild<GeometryNode>(isoName, isoGeom);

              if (!isoNode) {
                throw std::runtime_error("Failed to create isosurface node");
              }

              // Update UI on Qt thread
              QMetaObject::invokeMethod(
                  this, [this]() { onComputeFinished(true, "Isosurface extracted successfully"); },
                  Qt::QueuedConnection);
            } catch (const std::exception &e) {
              std::string errorMsg = std::string("Failed to create geometry node: ") + e.what();
              QMetaObject::invokeMethod(
                  this, [this, errorMsg]() { onComputeFinished(false, errorMsg); },
                  Qt::QueuedConnection);
            }
          });

        } catch (const boost::thread_interrupted &) {
          QMetaObject::invokeMethod(
              this, [this]() { onComputeFinished(false, "Extraction cancelled"); },
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

void IsosurfaceDialog::updateProgress(int value) { m_progressBar->setValue(value); }

void IsosurfaceDialog::onComputeFinished(bool success, const std::string &message) {
  setControlsEnabled(true);
  m_computing = false;
  m_computeButton->setText(tr("Extract Isosurface"));
  m_progressBar->setValue(success ? 100 : 0);
  m_statusLabel->setText(QString::fromStdString(message));

  if (success) {
    QMessageBox::information(this, tr("Success"), QString::fromStdString(message));
  } else {
    QMessageBox::warning(this, tr("Error"), QString::fromStdString(message));
  }
}

void IsosurfaceDialog::setControlsEnabled(bool enabled) {
  m_volumeComboBox->setEnabled(enabled);
  m_isovalueSpinBox->setEnabled(enabled);
  m_methodComboBox->setEnabled(enabled);
  m_improveIterationsSpinBox->setEnabled(enabled);
  m_normalTypeComboBox->setEnabled(enabled);
}
