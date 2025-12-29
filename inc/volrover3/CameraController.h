#ifndef CAMERACONTROLLER_H
#define CAMERACONTROLLER_H

#include <vtkCamera.h>
#include <vtkSmartPointer.h>
#include <set>

enum CameraMode {
    ORBIT_MODE = 0,
    FLY_MODE = 1
};

class CameraController
{
public:
    CameraController();
    ~CameraController();

    void setCamera(vtkCamera *camera);
    
    void handleKeyPress(int key);
    void handleKeyRelease(int key);
    void handleMousePress(int button);
    void handleMouseRelease(int button);
    void handleMouseMove(int dx, int dy);
    void handleMouseWheel(int delta);

    void update();

    void setMovementSpeed(double speed) { m_movementSpeed = speed; }
    void setMouseSensitivity(double sensitivity) { m_mouseSensitivity = sensitivity; }
    void setInvertMouse(bool invert) { m_invertMouse = invert; }
    
    void setMode(CameraMode mode) { m_mode = mode; }
    CameraMode getMode() const { return m_mode; }
    
    void setOrbitCenter(double x, double y, double z);
    
    void setKeyBindings(int forward, int backward, int left, int right, int up, int down);
    
    // Camera state synchronization
    void getCameraState(double pos[3], double dir[3], double up[3], double& fov);
    void setCameraState(const double pos[3], const double dir[3], const double up[3], double fov);
    void applyCameraToVTK();

private:
    void updateOrientation();
    void move(double forward, double right, double up);
    void orbitCamera(int dx, int dy);
    void saveCameraStateToAppState();

    vtkSmartPointer<vtkCamera> m_camera;
    
    // Camera mode
    CameraMode m_mode;
    
    // Orbit mode state
    double m_orbitCenter[3];
    double m_orbitDistance;
    double m_orbitAzimuth;
    double m_orbitElevation;
    
    // Fly mode state
    double m_position[3];
    double m_yaw;
    double m_pitch;
    
    // Input state
    std::set<int> m_keysPressed;
    bool m_mouseLeftPressed;
    bool m_mouseRightPressed;
    
    // Settings
    double m_movementSpeed;
    double m_mouseSensitivity;
    bool m_invertMouse;
    
    // Key bindings
    int m_keyForward;
    int m_keyBackward;
    int m_keyStrafeLeft;
    int m_keyStrafeRight;
    int m_keyUp;
    int m_keyDown;
};

#endif // CAMERACONTROLLER_H
