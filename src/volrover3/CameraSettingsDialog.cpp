#include <QCheckBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QKeyEvent>
#include <QLabel>
#include <QTabWidget>
#include <QTableWidget>
#include <QVBoxLayout>
#include <Qt>
#include <cvc/state.h>
#include <volrover3/CameraSettingsDialog.h>

// KeyBindButton implementation
KeyBindButton::KeyBindButton(int initialKey, QWidget *parent)
    : QPushButton(parent), m_key(initialKey), m_waitingForKey(false) {
  updateText();
  connect(this, &QPushButton::clicked, this, &KeyBindButton::startCapture);
}

void KeyBindButton::setKey(int key) {
  m_key = key;
  m_waitingForKey = false;
  updateText();
}

void KeyBindButton::keyPressEvent(QKeyEvent *event) {
  if (m_waitingForKey) {
    m_key = event->key();
    m_waitingForKey = false;
    updateText();
    emit keyChanged(m_key);
    clearFocus();
  } else {
    QPushButton::keyPressEvent(event);
  }
}

void KeyBindButton::focusOutEvent(QFocusEvent *event) {
  if (m_waitingForKey) {
    m_waitingForKey = false;
    updateText();
  }
  QPushButton::focusOutEvent(event);
}

void KeyBindButton::startCapture() {
  m_waitingForKey = true;
  updateText();
  setFocus();
}

void KeyBindButton::updateText() {
  if (m_waitingForKey) {
    setText(tr("Press a key..."));
    setStyleSheet("QPushButton { background-color: #4CAF50; color: white; }");
  } else {
    setText(QKeySequence(m_key).toString());
    setStyleSheet("");
  }
}

// CameraSettingsDialog implementation

CameraSettingsDialog::CameraSettingsDialog(const CameraSettings &settings, cvc::state *cameraState,
                                           QWidget *parent)
    : QDialog(parent), m_cameraState(cameraState), m_stateTable(nullptr) {
  setWindowTitle(tr("Camera Settings"));
  setupUI(settings);

  // Connect UI controls to emit settingsChanged signal
  connect(m_modeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &CameraSettingsDialog::emitSettingsChanged);
  connect(m_flySpeedSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &CameraSettingsDialog::emitSettingsChanged);
  connect(m_mouseSensitivitySpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &CameraSettingsDialog::emitSettingsChanged);
  connect(m_invertMouseCheck, &QCheckBox::toggled, this,
          &CameraSettingsDialog::emitSettingsChanged);
  connect(m_keyForwardButton, &KeyBindButton::keyChanged, this,
          &CameraSettingsDialog::emitSettingsChanged);
  connect(m_keyBackwardButton, &KeyBindButton::keyChanged, this,
          &CameraSettingsDialog::emitSettingsChanged);
  connect(m_keyStrafeLeftButton, &KeyBindButton::keyChanged, this,
          &CameraSettingsDialog::emitSettingsChanged);
  connect(m_keyStrafeRightButton, &KeyBindButton::keyChanged, this,
          &CameraSettingsDialog::emitSettingsChanged);
  connect(m_keyUpButton, &KeyBindButton::keyChanged, this,
          &CameraSettingsDialog::emitSettingsChanged);
  connect(m_keyDownButton, &KeyBindButton::keyChanged, this,
          &CameraSettingsDialog::emitSettingsChanged);

  // Subscribe to camera state changes if provided
  if (m_cameraState) {
    m_stateConnection = m_cameraState->childChanged.connect([this](const std::string &) {
      // Update UI on main thread using Qt's queued connection mechanism
      QMetaObject::invokeMethod(this, "updateStateDisplay", Qt::QueuedConnection);
    });
    updateStateDisplay(); // Initial update
  }
}

CameraSettingsDialog::~CameraSettingsDialog() {
  // Disconnect from state changes
  m_stateConnection.disconnect();
}

void CameraSettingsDialog::setupUI(const CameraSettings &settings) {
  QVBoxLayout *mainLayout = new QVBoxLayout(this);

  // Create tab widget
  QTabWidget *tabWidget = new QTabWidget(this);

  // === Mode Tab ===
  QWidget *modeTab = new QWidget();
  QVBoxLayout *modeTabLayout = new QVBoxLayout(modeTab);

  // Camera mode selection
  QGroupBox *modeGroup = new QGroupBox(tr("Camera Mode"));
  QFormLayout *modeLayout = new QFormLayout(modeGroup);

  m_modeCombo = new QComboBox();
  m_modeCombo->addItem(tr("Orbit (Trackball)"), 0);
  m_modeCombo->addItem(tr("Fly (FPS)"), 1);
  m_modeCombo->setCurrentIndex(settings.mode);
  modeLayout->addRow(tr("Mode:"), m_modeCombo);

  modeTabLayout->addWidget(modeGroup);

  // Movement settings
  QGroupBox *movementGroup = new QGroupBox(tr("Movement Settings"));
  QFormLayout *movementLayout = new QFormLayout(movementGroup);

  m_flySpeedSpin = new QDoubleSpinBox();
  m_flySpeedSpin->setRange(0.1, 100.0);
  m_flySpeedSpin->setSingleStep(0.5);
  m_flySpeedSpin->setValue(settings.flySpeed);
  m_flySpeedSpin->setSuffix(tr(" units/sec"));
  movementLayout->addRow(tr("Fly Speed:"), m_flySpeedSpin);

  m_mouseSensitivitySpin = new QDoubleSpinBox();
  m_mouseSensitivitySpin->setRange(0.1, 10.0);
  m_mouseSensitivitySpin->setSingleStep(0.1);
  m_mouseSensitivitySpin->setValue(settings.mouseSensitivity);
  movementLayout->addRow(tr("Mouse Sensitivity:"), m_mouseSensitivitySpin);

  m_invertMouseCheck = new QCheckBox(tr("Invert mouse Y-axis"));
  m_invertMouseCheck->setChecked(settings.invertMouse);
  movementLayout->addRow("", m_invertMouseCheck);

  modeTabLayout->addWidget(movementGroup);
  modeTabLayout->addStretch();

  // === Key Bindings Tab ===
  QWidget *keysTab = new QWidget();
  QVBoxLayout *keysTabLayout = new QVBoxLayout(keysTab);

  QGroupBox *keysGroup = new QGroupBox(tr("Key Bindings (Click to rebind)"));
  QFormLayout *keysLayout = new QFormLayout(keysGroup);

  m_keyForwardButton = new KeyBindButton(settings.keyForward);
  keysLayout->addRow(tr("Forward:"), m_keyForwardButton);

  m_keyBackwardButton = new KeyBindButton(settings.keyBackward);
  keysLayout->addRow(tr("Backward:"), m_keyBackwardButton);

  m_keyStrafeLeftButton = new KeyBindButton(settings.keyStrafeLeft);
  keysLayout->addRow(tr("Strafe Left:"), m_keyStrafeLeftButton);

  m_keyStrafeRightButton = new KeyBindButton(settings.keyStrafeRight);
  keysLayout->addRow(tr("Strafe Right:"), m_keyStrafeRightButton);

  m_keyUpButton = new KeyBindButton(settings.keyUp);
  keysLayout->addRow(tr("Up:"), m_keyUpButton);

  m_keyDownButton = new KeyBindButton(settings.keyDown);
  keysLayout->addRow(tr("Down:"), m_keyDownButton);

  keysTabLayout->addWidget(keysGroup);
  keysTabLayout->addStretch();

  // === State Tab ===
  QWidget *stateTab = new QWidget();
  QVBoxLayout *stateTabLayout = new QVBoxLayout(stateTab);

  QLabel *stateLabel = new QLabel(tr("Camera state from state tree (read-only):"), stateTab);
  stateTabLayout->addWidget(stateLabel);

  m_stateTable = new QTableWidget(this);
  m_stateTable->setObjectName("cameraStateTable");
  m_stateTable->setColumnCount(2);
  m_stateTable->setHorizontalHeaderLabels({tr("Property"), tr("Value")});
  m_stateTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  m_stateTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
  m_stateTable->verticalHeader()->setVisible(false);
  m_stateTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
  m_stateTable->setSelectionBehavior(QAbstractItemView::SelectRows);
  m_stateTable->setAlternatingRowColors(true);
  stateTabLayout->addWidget(m_stateTable);

  // Add tabs
  tabWidget->addTab(modeTab, tr("Mode"));
  tabWidget->addTab(keysTab, tr("Key Bindings"));
  tabWidget->addTab(stateTab, tr("State"));

  mainLayout->addWidget(tabWidget);

  // Reset to defaults and reset view buttons
  QHBoxLayout *resetLayout = new QHBoxLayout();
  QPushButton *resetButton = new QPushButton(tr("Reset to Defaults"));
  connect(resetButton, &QPushButton::clicked, this, &CameraSettingsDialog::onResetDefaults);
  resetLayout->addWidget(resetButton);

  QPushButton *resetViewButton = new QPushButton(tr("Reset View"));
  resetViewButton->setToolTip(tr("Position camera to view the entire scene"));
  connect(resetViewButton, &QPushButton::clicked, this, &CameraSettingsDialog::onResetView);
  resetLayout->addWidget(resetViewButton);
  mainLayout->addLayout(resetLayout);
}

void CameraSettingsDialog::updateStateDisplay() {
  if (!m_cameraState || !m_stateTable)
    return;

  CameraState state = readCameraState();

  // Define the properties to display
  struct Property {
    QString name;
    QString value;
  };

  QList<Property> properties = {
      {tr("Mode"), state.mode == 0 ? tr("Orbit") : tr("Fly")},
      {tr("Position X"), QString::number(state.positionX, 'f', 4)},
      {tr("Position Y"), QString::number(state.positionY, 'f', 4)},
      {tr("Position Z"), QString::number(state.positionZ, 'f', 4)},
      {tr("View Direction X"), QString::number(state.viewDirX, 'f', 4)},
      {tr("View Direction Y"), QString::number(state.viewDirY, 'f', 4)},
      {tr("View Direction Z"), QString::number(state.viewDirZ, 'f', 4)},
      {tr("Up Vector X"), QString::number(state.upX, 'f', 4)},
      {tr("Up Vector Y"), QString::number(state.upY, 'f', 4)},
      {tr("Up Vector Z"), QString::number(state.upZ, 'f', 4)},
      {tr("Field of View"), QString::number(state.fov, 'f', 2) + QString::fromUtf8("°")},
      {tr("---Orbit Mode---"), ""},
      {tr("Center X"), QString::number(state.orbitCenterX, 'f', 4)},
      {tr("Center Y"), QString::number(state.orbitCenterY, 'f', 4)},
      {tr("Center Z"), QString::number(state.orbitCenterZ, 'f', 4)},
      {tr("Distance"), QString::number(state.orbitDistance, 'f', 4)},
      {tr("Azimuth"), QString::number(state.orbitAzimuth, 'f', 2) + QString::fromUtf8("°")},
      {tr("Elevation"), QString::number(state.orbitElevation, 'f', 2) + QString::fromUtf8("°")},
      {tr("---Fly Mode---"), ""},
      {tr("Yaw"), QString::number(state.flyYaw, 'f', 2) + QString::fromUtf8("°")},
      {tr("Pitch"), QString::number(state.flyPitch, 'f', 2) + QString::fromUtf8("°")},
      {tr("Focal Point X"), QString::number(state.flyFocalX, 'f', 4)},
      {tr("Focal Point Y"), QString::number(state.flyFocalY, 'f', 4)},
      {tr("Focal Point Z"), QString::number(state.flyFocalZ, 'f', 4)},
  };

  m_stateTable->setRowCount(properties.size());
  for (int i = 0; i < properties.size(); ++i) {
    m_stateTable->setItem(i, 0, new QTableWidgetItem(properties[i].name));
    m_stateTable->setItem(i, 1, new QTableWidgetItem(properties[i].value));

    // Style section headers
    if (properties[i].name.startsWith("---")) {
      QFont boldFont;
      boldFont.setBold(true);
      m_stateTable->item(i, 0)->setFont(boldFont);
      m_stateTable->item(i, 0)->setBackground(QColor(220, 220, 220));
      m_stateTable->item(i, 1)->setBackground(QColor(220, 220, 220));
    }
  }
}

CameraSettingsDialog::CameraState CameraSettingsDialog::readCameraState() const {
  CameraState state = {};
  if (!m_cameraState)
    return state;

  // Read from state tree children using operator()
  state.mode = (*m_cameraState)("mode").value<int>();
  state.positionX = (*m_cameraState)("position.x").value<double>();
  state.positionY = (*m_cameraState)("position.y").value<double>();
  state.positionZ = (*m_cameraState)("position.z").value<double>();
  state.viewDirX = (*m_cameraState)("view_direction.x").value<double>();
  state.viewDirY = (*m_cameraState)("view_direction.y").value<double>();
  state.viewDirZ = (*m_cameraState)("view_direction.z").value<double>();
  state.upX = (*m_cameraState)("up_vector.x").value<double>();
  state.upY = (*m_cameraState)("up_vector.y").value<double>();
  state.upZ = (*m_cameraState)("up_vector.z").value<double>();
  state.fov = (*m_cameraState)("fov").value<double>();
  state.orbitCenterX = (*m_cameraState)("orbit.center.x").value<double>();
  state.orbitCenterY = (*m_cameraState)("orbit.center.y").value<double>();
  state.orbitCenterZ = (*m_cameraState)("orbit.center.z").value<double>();
  state.orbitDistance = (*m_cameraState)("orbit.distance").value<double>();
  state.orbitAzimuth = (*m_cameraState)("orbit.azimuth").value<double>();
  state.orbitElevation = (*m_cameraState)("orbit.elevation").value<double>();
  state.flyYaw = (*m_cameraState)("fly.yaw").value<double>();
  state.flyPitch = (*m_cameraState)("fly.pitch").value<double>();
  state.flyFocalX = (*m_cameraState)("fly.focal_point.x").value<double>();
  state.flyFocalY = (*m_cameraState)("fly.focal_point.y").value<double>();
  state.flyFocalZ = (*m_cameraState)("fly.focal_point.z").value<double>();

  return state;
}

CameraSettingsDialog::CameraSettings CameraSettingsDialog::getSettings() const {
  CameraSettings settings;
  settings.mode = m_modeCombo->currentData().toInt();
  settings.flySpeed = m_flySpeedSpin->value();
  settings.mouseSensitivity = m_mouseSensitivitySpin->value();
  settings.invertMouse = m_invertMouseCheck->isChecked();
  settings.keyForward = m_keyForwardButton->key();
  settings.keyBackward = m_keyBackwardButton->key();
  settings.keyStrafeLeft = m_keyStrafeLeftButton->key();
  settings.keyStrafeRight = m_keyStrafeRightButton->key();
  settings.keyUp = m_keyUpButton->key();
  settings.keyDown = m_keyDownButton->key();
  return settings;
}

CameraSettingsDialog::CameraSettings CameraSettingsDialog::getDefaultSettings() const {
  CameraSettings settings;
  settings.mode = 0; // Orbit mode
  settings.flySpeed = 5.0;
  settings.mouseSensitivity = 1.0;
  settings.invertMouse = false;
  settings.keyForward = Qt::Key_W;
  settings.keyBackward = Qt::Key_S;
  settings.keyStrafeLeft = Qt::Key_A;
  settings.keyStrafeRight = Qt::Key_D;
  settings.keyUp = Qt::Key_Space;
  settings.keyDown = Qt::Key_Control;
  return settings;
}

void CameraSettingsDialog::onResetDefaults() {
  CameraSettings defaults = getDefaultSettings();
  m_modeCombo->setCurrentIndex(defaults.mode);
  m_flySpeedSpin->setValue(defaults.flySpeed);
  m_mouseSensitivitySpin->setValue(defaults.mouseSensitivity);
  m_invertMouseCheck->setChecked(defaults.invertMouse);
  m_keyForwardButton->setKey(defaults.keyForward);
  m_keyBackwardButton->setKey(defaults.keyBackward);
  m_keyStrafeLeftButton->setKey(defaults.keyStrafeLeft);
  m_keyStrafeRightButton->setKey(defaults.keyStrafeRight);
  m_keyUpButton->setKey(defaults.keyUp);
  m_keyDownButton->setKey(defaults.keyDown);

  // Emit settings changed for real-time application
  emitSettingsChanged();
}

void CameraSettingsDialog::onResetView() {
  // Signal to parent to reset the camera view
  // This will be handled by MainWindow
  emit resetViewRequested();
}

void CameraSettingsDialog::emitSettingsChanged() {
  // Get current settings and emit signal for real-time application
  emit settingsChanged(getSettings());
}
