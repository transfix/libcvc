#include <volrover3/CameraController.h>
#include <volrover3/AppState.h>
#include <Qt>
#include <cmath>

CameraController::CameraController()
    : m_camera(nullptr)
    , m_mode(ORBIT_MODE)
    , m_orbitDistance(10.0)
    , m_orbitAzimuth(0.0)
    , m_orbitElevation(30.0)
    , m_yaw(0.0)
    , m_pitch(0.0)
    , m_mouseLeftPressed(false)
    , m_mouseRightPressed(false)
    , m_movementSpeed(5.0)
    , m_mouseSensitivity(1.0)
    , m_invertMouse(false)
    , m_keyForward(Qt::Key_W)
    , m_keyBackward(Qt::Key_S)
    , m_keyStrafeLeft(Qt::Key_A)
    , m_keyStrafeRight(Qt::Key_D)
    , m_keyUp(Qt::Key_Space)
    , m_keyDown(Qt::Key_Control)
{
    m_position[0] = 0.0;
    m_position[1] = 0.0;
    m_position[2] = 10.0;
    
    m_orbitCenter[0] = 0.0;
    m_orbitCenter[1] = 0.0;
    m_orbitCenter[2] = 0.0;
}

CameraController::~CameraController()
{
}

void CameraController::setCamera(vtkCamera *camera)
{
    m_camera = camera;
    if (m_camera) {
        double *pos = m_camera->GetPosition();
        m_position[0] = pos[0];
        m_position[1] = pos[1];
        m_position[2] = pos[2];
        
        // Initialize orbit parameters from current camera
        double *focal = m_camera->GetFocalPoint();
        m_orbitCenter[0] = focal[0];
        m_orbitCenter[1] = focal[1];
        m_orbitCenter[2] = focal[2];
        
        double dx = pos[0] - focal[0];
        double dy = pos[1] - focal[1];
        double dz = pos[2] - focal[2];
        m_orbitDistance = std::sqrt(dx*dx + dy*dy + dz*dz);
    }
}

void CameraController::setOrbitCenter(double x, double y, double z)
{
    m_orbitCenter[0] = x;
    m_orbitCenter[1] = y;
    m_orbitCenter[2] = z;
}

void CameraController::setKeyBindings(int forward, int backward, int left, int right, int up, int down)
{
    m_keyForward = forward;
    m_keyBackward = backward;
    m_keyStrafeLeft = left;
    m_keyStrafeRight = right;
    m_keyUp = up;
    m_keyDown = down;
}

void CameraController::getCameraState(double pos[3], double dir[3], double up[3], double& fov)
{
    if (!m_camera) return;
    
    double* camPos = m_camera->GetPosition();
    pos[0] = camPos[0];
    pos[1] = camPos[1];
    pos[2] = camPos[2];
    
    // Get view direction from focal point
    double* focal = m_camera->GetFocalPoint();
    double dx = focal[0] - camPos[0];
    double dy = focal[1] - camPos[1];
    double dz = focal[2] - camPos[2];
    double len = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (len > 0.0001) {
        dir[0] = dx / len;
        dir[1] = dy / len;
        dir[2] = dz / len;
    } else {
        dir[0] = 0.0;
        dir[1] = 1.0;
        dir[2] = 0.0;
    }
    
    double* camUp = m_camera->GetViewUp();
    up[0] = camUp[0];
    up[1] = camUp[1];
    up[2] = camUp[2];
    
    fov = m_camera->GetViewAngle();
}

void CameraController::setCameraState(const double pos[3], const double dir[3], const double up[3], double fov)
{
    if (!m_camera) return;
    
    // Update internal position state
    m_position[0] = pos[0];
    m_position[1] = pos[1];
    m_position[2] = pos[2];
    
    // Set camera position
    m_camera->SetPosition(pos[0], pos[1], pos[2]);
    
    // Set focal point based on view direction
    // Place focal point 1 unit in front of camera
    m_camera->SetFocalPoint(
        pos[0] + dir[0],
        pos[1] + dir[1],
        pos[2] + dir[2]
    );
    
    // Set up vector
    m_camera->SetViewUp(up[0], up[1], up[2]);
    
    // Set field of view
    m_camera->SetViewAngle(fov);
}

void CameraController::applyCameraToVTK()
{
    if (!m_camera) return;
    
    if (m_mode == ORBIT_MODE) {
        // In orbit mode, use orbit parameters
        updateOrientation();
    } else {
        // In fly mode, use fly parameters  
        updateOrientation();
    }
}

void CameraController::handleKeyPress(int key)
{
    m_keysPressed.insert(key);
}

void CameraController::handleKeyRelease(int key)
{
    m_keysPressed.erase(key);
}

void CameraController::handleMousePress(int button)
{
    if (button == Qt::LeftButton) {
        m_mouseLeftPressed = true;
    } else if (button == Qt::RightButton) {
        m_mouseRightPressed = true;
    }
}

void CameraController::handleMouseRelease(int button)
{
    if (button == Qt::LeftButton) {
        m_mouseLeftPressed = false;
    } else if (button == Qt::RightButton) {
        m_mouseRightPressed = false;
    }
}

void CameraController::handleMouseMove(int dx, int dy)
{
    if (m_mouseLeftPressed || m_mouseRightPressed) {
        if (m_mode == ORBIT_MODE) {
            orbitCamera(dx, dy);
        } else {
            // Fly mode - update yaw and pitch
            double yawDelta = dx * m_mouseSensitivity * 0.2;
            double pitchDelta = dy * m_mouseSensitivity * 0.2;
            
            if (m_invertMouse) {
                pitchDelta = -pitchDelta;
            }
            
            m_yaw += yawDelta;
            m_pitch += pitchDelta;

            // Clamp pitch to avoid gimbal lock
            const double maxPitch = 89.0;
            if (m_pitch > maxPitch) m_pitch = maxPitch;
            if (m_pitch < -maxPitch) m_pitch = -maxPitch;

            updateOrientation();
        }
    }
}

void CameraController::handleMouseWheel(int delta)
{
    if (m_mode == ORBIT_MODE) {
        // Zoom by changing orbit distance
        double zoomFactor = (delta > 0 ? 0.9 : 1.1);
        m_orbitDistance *= zoomFactor;
        if (m_orbitDistance < 0.1) m_orbitDistance = 0.1;
        orbitCamera(0, 0);
    } else {
        // Fly mode - move forward/backward
        double amount = (delta > 0 ? 1.0 : -1.0) * m_movementSpeed * 0.1;
        move(amount, 0.0, 0.0);
    }
}

void CameraController::update()
{
    if (!m_camera) return;

    // Only handle keyboard movement in fly mode
    if (m_mode == FLY_MODE) {
        double forward = 0.0;
        double right = 0.0;
        double up = 0.0;
        
        double frameSpeed = m_movementSpeed * 0.016; // Assume ~60fps

        if (m_keysPressed.count(m_keyForward)) forward += frameSpeed;
        if (m_keysPressed.count(m_keyBackward)) forward -= frameSpeed;
        if (m_keysPressed.count(m_keyStrafeRight)) right += frameSpeed;
        if (m_keysPressed.count(m_keyStrafeLeft)) right -= frameSpeed;
        if (m_keysPressed.count(m_keyUp)) up += frameSpeed;
        if (m_keysPressed.count(m_keyDown)) up -= frameSpeed;

        if (forward != 0.0 || right != 0.0 || up != 0.0) {
            move(forward, right, up);
        }
    }
}

void CameraController::updateOrientation()
{
    if (!m_camera) return;

    // Convert yaw and pitch to radians
    double yawRad = m_yaw * M_PI / 180.0;
    double pitchRad = m_pitch * M_PI / 180.0;

    // Calculate forward vector
    double forward[3];
    forward[0] = cos(pitchRad) * sin(yawRad);
    forward[1] = sin(pitchRad);
    forward[2] = -cos(pitchRad) * cos(yawRad);

    // Calculate focal point
    double focalPoint[3];
    focalPoint[0] = m_position[0] + forward[0];
    focalPoint[1] = m_position[1] + forward[1];
    focalPoint[2] = m_position[2] + forward[2];

    // Update camera
    m_camera->SetPosition(m_position);
    m_camera->SetFocalPoint(focalPoint);
    m_camera->SetViewUp(0, 1, 0);
    
    saveCameraStateToAppState();
}

void CameraController::saveCameraStateToAppState()
{
    if (!m_camera) return;
    
    double pos[3], dir[3], up[3], fov;
    getCameraState(pos, dir, up, fov);
    
    AppState::instance().setCameraPosition(pos[0], pos[1], pos[2]);
    AppState::instance().setCameraViewDirection(dir[0], dir[1], dir[2]);
    AppState::instance().setCameraUpVector(up[0], up[1], up[2]);
    AppState::instance().setCameraFieldOfView(fov);
}

void CameraController::move(double forward, double right, double up)
{
    if (!m_camera) return;

    // Convert yaw to radians
    double yawRad = m_yaw * M_PI / 180.0;

    // Calculate forward and right vectors
    double forwardVec[3];
    forwardVec[0] = sin(yawRad);
    forwardVec[1] = 0.0;  // Keep movement on horizontal plane
    forwardVec[2] = -cos(yawRad);

    double rightVec[3];
    rightVec[0] = cos(yawRad);
    rightVec[1] = 0.0;
    rightVec[2] = sin(yawRad);

    // Update position
    m_position[0] += forward * forwardVec[0] + right * rightVec[0];
    m_position[1] += up;  // Vertical movement
    m_position[2] += forward * forwardVec[2] + right * rightVec[2];

    updateOrientation();
}

void CameraController::orbitCamera(int dx, int dy)
{
    if (!m_camera) return;
    
    // Update orbit angles
    m_orbitAzimuth += dx * m_mouseSensitivity * 0.5;
    m_orbitElevation -= dy * m_mouseSensitivity * 0.5;
    
    // Clamp elevation
    if (m_orbitElevation > 89.0) m_orbitElevation = 89.0;
    if (m_orbitElevation < -89.0) m_orbitElevation = -89.0;
    
    // Convert to radians
    double azimuthRad = m_orbitAzimuth * M_PI / 180.0;
    double elevationRad = m_orbitElevation * M_PI / 180.0;
    
    // Calculate camera position on sphere around orbit center
    double x = m_orbitCenter[0] + m_orbitDistance * cos(elevationRad) * sin(azimuthRad);
    double y = m_orbitCenter[1] + m_orbitDistance * sin(elevationRad);
    double z = m_orbitCenter[2] + m_orbitDistance * cos(elevationRad) * cos(azimuthRad);
    
    // Update camera
    m_camera->SetPosition(x, y, z);
    m_camera->SetFocalPoint(m_orbitCenter);
    m_camera->SetViewUp(0, 1, 0);
    
    saveCameraStateToAppState();
}
