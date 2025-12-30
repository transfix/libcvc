#ifndef CAMERASETTINGSDIALOG_H
#define CAMERASETTINGSDIALOG_H

#include <QDialog>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QPushButton>

class QKeySequenceEdit;

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

    explicit CameraSettingsDialog(const CameraSettings& settings, QWidget *parent = nullptr);
    
    CameraSettings getSettings() const;

signals:
    void resetViewRequested();

private slots:
    void onResetDefaults();
    void onResetView();

private:
    void setupUI(const CameraSettings& settings);
    CameraSettings getDefaultSettings() const;

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
};

#endif // CAMERASETTINGSDIALOG_H
