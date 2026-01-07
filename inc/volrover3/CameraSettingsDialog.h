#ifndef CAMERASETTINGSDIALOG_H
#define CAMERASETTINGSDIALOG_H

#include <QDialog>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QPushButton>
#include <boost/signals2/connection.hpp>

namespace cvc { class state; }

class QKeySequenceEdit;
class QTableWidget;

// Custom button that captures key presses for binding
class KeyBindButton : public QPushButton
{
    Q_OBJECT
public:
    KeyBindButton(int initialKey, QWidget *parent = nullptr);
    
    int key() const { return m_key; }
    void setKey(int key);

signals:
    void keyChanged(int key);

protected:
    void keyPressEvent(QKeyEvent *event) override;
    void focusOutEvent(QFocusEvent *event) override;

private slots:
    void startCapture();

private:
    void updateText();
    
    int m_key;
    bool m_waitingForKey;
};

class CameraSettingsDialog : public QDialog
{
    Q_OBJECT

public:
    struct CameraSettings {
        int mode;              // 0 = orbit, 1 = fly
        double flySpeed;
        double mouseSensitivity;
        bool invertMouse;
        int keyForward;
        int keyBackward;
        int keyStrafeLeft;
        int keyStrafeRight;
        int keyUp;
        int keyDown;
    };
    
    // Camera state from state tree (for display purposes)
    struct CameraState {
        int mode;
        double positionX, positionY, positionZ;
        double viewDirX, viewDirY, viewDirZ;
        double upX, upY, upZ;
        double fov;
        // Orbit mode
        double orbitCenterX, orbitCenterY, orbitCenterZ;
        double orbitDistance;
        double orbitAzimuth, orbitElevation;
        // Fly mode
        double flyYaw, flyPitch;
        double flyFocalX, flyFocalY, flyFocalZ;
    };

    // Constructor with optional camera state for live state display
    explicit CameraSettingsDialog(const CameraSettings& settings, 
                                  cvc::state* cameraState = nullptr,
                                  QWidget *parent = nullptr);
    ~CameraSettingsDialog();
    
    CameraSettings getSettings() const;

signals:
    void resetViewRequested();
    void settingsChanged(const CameraSettings& settings);

private slots:
    void onResetDefaults();
    void onResetView();
    void updateStateDisplay();
    void emitSettingsChanged();

private:
    void setupUI(const CameraSettings& settings);
    CameraSettings getDefaultSettings() const;
    CameraState readCameraState() const;

    cvc::state* m_cameraState;
    boost::signals2::connection m_stateConnection;
    
    QComboBox *m_modeCombo;
    QDoubleSpinBox *m_flySpeedSpin;
    QDoubleSpinBox *m_mouseSensitivitySpin;
    QCheckBox *m_invertMouseCheck;
    KeyBindButton *m_keyForwardButton;
    KeyBindButton *m_keyBackwardButton;
    KeyBindButton *m_keyStrafeLeftButton;
    KeyBindButton *m_keyStrafeRightButton;
    KeyBindButton *m_keyUpButton;
    KeyBindButton *m_keyDownButton;
    
    // State display
    QTableWidget *m_stateTable;
};

#endif // CAMERASETTINGSDIALOG_H
