#include <volrover3/CameraSettingsDialog.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QLabel>
#include <QGroupBox>
#include <QDialogButtonBox>
#include <QCheckBox>
#include <QKeyEvent>
#include <Qt>

// KeyBindButton implementation
KeyBindButton::KeyBindButton(int initialKey, QWidget *parent)
    : QPushButton(parent)
    , m_key(initialKey)
    , m_waitingForKey(false)
{
    updateText();
    connect(this, &QPushButton::clicked, this, &KeyBindButton::startCapture);
}

void KeyBindButton::setKey(int key)
{
    m_key = key;
    m_waitingForKey = false;
    updateText();
}

void KeyBindButton::keyPressEvent(QKeyEvent *event)
{
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

void KeyBindButton::focusOutEvent(QFocusEvent *event)
{
    if (m_waitingForKey) {
        m_waitingForKey = false;
        updateText();
    }
    QPushButton::focusOutEvent(event);
}

void KeyBindButton::startCapture()
{
    m_waitingForKey = true;
    updateText();
    setFocus();
}

void KeyBindButton::updateText()
{
    if (m_waitingForKey) {
        setText(tr("Press a key..."));
        setStyleSheet("QPushButton { background-color: #4CAF50; color: white; }");
    } else {
        setText(QKeySequence(m_key).toString());
        setStyleSheet("");
    }
}

// CameraSettingsDialog implementation

CameraSettingsDialog::CameraSettingsDialog(const CameraSettings& settings, QWidget *parent)
    : QDialog(parent)
{
    setWindowTitle(tr("Camera Settings"));
    setupUI(settings);
}

void CameraSettingsDialog::setupUI(const CameraSettings& settings)
{
    QVBoxLayout *mainLayout = new QVBoxLayout(this);

    // Camera mode selection
    QGroupBox *modeGroup = new QGroupBox(tr("Camera Mode"));
    QFormLayout *modeLayout = new QFormLayout(modeGroup);
    
    m_modeCombo = new QComboBox();
    m_modeCombo->addItem(tr("Orbit (Trackball)"), 0);
    m_modeCombo->addItem(tr("Fly (FPS)"), 1);
    m_modeCombo->setCurrentIndex(settings.mode);
    modeLayout->addRow(tr("Mode:"), m_modeCombo);
    
    mainLayout->addWidget(modeGroup);

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
    
    mainLayout->addWidget(movementGroup);

    // Key bindings
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
    
    mainLayout->addWidget(keysGroup);

    // Reset to defaults button
    QPushButton *resetButton = new QPushButton(tr("Reset to Defaults"));
    connect(resetButton, &QPushButton::clicked, this, &CameraSettingsDialog::onResetDefaults);
    mainLayout->addWidget(resetButton);

    // Dialog buttons
    QDialogButtonBox *buttonBox = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    mainLayout->addWidget(buttonBox);
}

CameraSettingsDialog::CameraSettings CameraSettingsDialog::getSettings() const
{
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

CameraSettingsDialog::CameraSettings CameraSettingsDialog::getDefaultSettings() const
{
    CameraSettings settings;
    settings.mode = 0;  // Orbit mode
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

void CameraSettingsDialog::onResetDefaults()
{
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
}
