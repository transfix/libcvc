#include <volrover3/CameraController.h>
#include <cvc/state.h>
#include <Qt>
#include <cmath>
#include <sstream>

CameraController::CameraController(const std::string& statePath)
    : SceneNode(statePath)
    , m_camera(nullptr)
    , m_mode(ORBIT_MODE)
    , m_orbitDistance(10.0)
    , m_orbitAzimuth(0.0)
    , m_orbitElevation(30.0)
    , m_yaw(0.0)
    , m_pitch(0.0)
    , m_mouseLeftPressed(false)
    , m_mouseRightPressed(false)
    , m_mouseMiddlePressed(false)
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
    m_position[2] = 5.0;
    m_focalPoint[0] = 0.0;
    m_focalPoint[1] = 0.0;
    m_focalPoint[2] = 0.0;
    
    m_orbitCenter[0] = 0.0;
    m_orbitCenter[1] = 0.0;
    m_orbitCenter[2] = 0.0;
    
    initializeState();
}

CameraController::~CameraController()
{
}

void CameraController::initializeState()
{
    // Camera mode
    getState("mode").value(static_cast<int>(m_mode));
    
    // Current camera state (computed from VTK camera)
    getState("position.x").value(m_position[0]);
    getState("position.y").value(m_position[1]);
    getState("position.z").value(m_position[2]);
    getState("view_direction.x").value(0.0);
    getState("view_direction.y").value(0.0);
    getState("view_direction.z").value(-1.0);
    getState("up_vector.x").value(0.0);
    getState("up_vector.y").value(1.0);
    getState("up_vector.z").value(0.0);
    getState("fov").value(30.0);
    
    // Orbit mode state
    getState("orbit.center.x").value(m_orbitCenter[0]);
    getState("orbit.center.y").value(m_orbitCenter[1]);
    getState("orbit.center.z").value(m_orbitCenter[2]);
    getState("orbit.distance").value(m_orbitDistance);
    getState("orbit.azimuth").value(m_orbitAzimuth);
    getState("orbit.elevation").value(m_orbitElevation);
    
    // Fly mode state
    getState("fly.position.x").value(m_position[0]);
    getState("fly.position.y").value(m_position[1]);
    getState("fly.position.z").value(m_position[2]);
    getState("fly.focal_point.x").value(m_focalPoint[0]);
    getState("fly.focal_point.y").value(m_focalPoint[1]);
    getState("fly.focal_point.z").value(m_focalPoint[2]);
    getState("fly.yaw").value(m_yaw);
    getState("fly.pitch").value(m_pitch);
    
    // Settings
    getState("settings.movement_speed").value(m_movementSpeed);
    getState("settings.mouse_sensitivity").value(m_mouseSensitivity);
    getState("settings.invert_mouse").value(m_invertMouse);
    
    // Key bindings
    getState("keys.forward").value(m_keyForward);
    getState("keys.backward").value(m_keyBackward);
    getState("keys.strafe_left").value(m_keyStrafeLeft);
    getState("keys.strafe_right").value(m_keyStrafeRight);
    getState("keys.up").value(m_keyUp);
    getState("keys.down").value(m_keyDown);
    
    // Input state (read-only)
    getState("input.mouse_left_pressed").value(m_mouseLeftPressed);
    getState("input.mouse_left_pressed").readOnly(true);
    getState("input.mouse_right_pressed").value(m_mouseRightPressed);
    getState("input.mouse_right_pressed").readOnly(true);
    
    // Keys pressed - store as count for simplicity (read-only)
    getState("input.keys_pressed_count").value(static_cast<int>(m_keysPressed.size()));
    getState("input.keys_pressed_count").readOnly(true);
}

void CameraController::handleStateChanged(const std::string& childState)
{
    if (!m_camera) return;
    
    bool needsOrbitUpdate = false;
    bool needsFlyUpdate = false;
    
    // Camera mode
    if (childState == "mode") {
        m_mode = static_cast<CameraMode>(getState("mode").value<int>());
        // Mode change should trigger appropriate camera update
        if (m_mode == ORBIT_MODE) {
            needsOrbitUpdate = true;
        } else {
            needsFlyUpdate = true;
        }
    }
    // Orbit mode state
    else if (childState == "orbit.center.x") {
        m_orbitCenter[0] = getState("orbit.center.x").value<double>();
        needsOrbitUpdate = (m_mode == ORBIT_MODE);
    }
    else if (childState == "orbit.center.y") {
        m_orbitCenter[1] = getState("orbit.center.y").value<double>();
        needsOrbitUpdate = (m_mode == ORBIT_MODE);
    }
    else if (childState == "orbit.center.z") {
        m_orbitCenter[2] = getState("orbit.center.z").value<double>();
        needsOrbitUpdate = (m_mode == ORBIT_MODE);
    }
    else if (childState == "orbit.distance") {
        m_orbitDistance = getState("orbit.distance").value<double>();
        needsOrbitUpdate = (m_mode == ORBIT_MODE);
    }
    else if (childState == "orbit.azimuth") {
        m_orbitAzimuth = getState("orbit.azimuth").value<double>();
        needsOrbitUpdate = (m_mode == ORBIT_MODE);
    }
    else if (childState == "orbit.elevation") {
        m_orbitElevation = getState("orbit.elevation").value<double>();
        needsOrbitUpdate = (m_mode == ORBIT_MODE);
    }
    // Fly mode state
    else if (childState == "fly.position.x") {
        m_position[0] = getState("fly.position.x").value<double>();
        needsFlyUpdate = (m_mode == FLY_MODE);
    }
    else if (childState == "fly.position.y") {
        m_position[1] = getState("fly.position.y").value<double>();
        needsFlyUpdate = (m_mode == FLY_MODE);
    }
    else if (childState == "fly.position.z") {
        m_position[2] = getState("fly.position.z").value<double>();
        needsFlyUpdate = (m_mode == FLY_MODE);
    }
    else if (childState == "fly.focal_point.x") {
        m_focalPoint[0] = getState("fly.focal_point.x").value<double>();
        needsFlyUpdate = (m_mode == FLY_MODE);
    }
    else if (childState == "fly.focal_point.y") {
        m_focalPoint[1] = getState("fly.focal_point.y").value<double>();
        needsFlyUpdate = (m_mode == FLY_MODE);
    }
    else if (childState == "fly.focal_point.z") {
        m_focalPoint[2] = getState("fly.focal_point.z").value<double>();
        needsFlyUpdate = (m_mode == FLY_MODE);
    }
    else if (childState == "fly.yaw") {
        m_yaw = getState("fly.yaw").value<double>();
        needsFlyUpdate = (m_mode == FLY_MODE);
    }
    else if (childState == "fly.pitch") {
        m_pitch = getState("fly.pitch").value<double>();
        needsFlyUpdate = (m_mode == FLY_MODE);
    }
    // Direct camera state changes (position, view_direction, up_vector, fov)
    // These bypass the mode-specific parameters and directly set the camera
    else if (childState.find("position.") == 0 || 
             childState.find("view_direction.") == 0 ||
             childState.find("up_vector.") == 0 ||
             childState == "fov") {
        // Direct camera manipulation - apply immediately
        double pos[3], dir[3], up[3], fov;
        pos[0] = getState("position.x").value<double>();
        pos[1] = getState("position.y").value<double>();
        pos[2] = getState("position.z").value<double>();
        dir[0] = getState("view_direction.x").value<double>();
        dir[1] = getState("view_direction.y").value<double>();
        dir[2] = getState("view_direction.z").value<double>();
        up[0] = getState("up_vector.x").value<double>();
        up[1] = getState("up_vector.y").value<double>();
        up[2] = getState("up_vector.z").value<double>();
        fov = getState("fov").value<double>();
        
        runOnMainThread([this, pos, dir, up, fov]() {
            m_camera->SetPosition(pos[0], pos[1], pos[2]);
            m_camera->SetFocalPoint(
                pos[0] + dir[0],
                pos[1] + dir[1],
                pos[2] + dir[2]
            );
            m_camera->SetViewUp(up[0], up[1], up[2]);
            m_camera->SetViewAngle(fov);
        });
    }
    // Settings
    else if (childState == "settings.movement_speed") {
        m_movementSpeed = getState("settings.movement_speed").value<double>();
    }
    else if (childState == "settings.mouse_sensitivity") {
        m_mouseSensitivity = getState("settings.mouse_sensitivity").value<double>();
    }
    else if (childState == "settings.invert_mouse") {
        m_invertMouse = getState("settings.invert_mouse").value<bool>();
    }
    // Key bindings
    else if (childState == "keys.forward") {
        m_keyForward = getState("keys.forward").value<int>();
    }
    else if (childState == "keys.backward") {
        m_keyBackward = getState("keys.backward").value<int>();
    }
    else if (childState == "keys.strafe_left") {
        m_keyStrafeLeft = getState("keys.strafe_left").value<int>();
    }
    else if (childState == "keys.strafe_right") {
        m_keyStrafeRight = getState("keys.strafe_right").value<int>();
    }
    else if (childState == "keys.up") {
        m_keyUp = getState("keys.up").value<int>();
    }
    else if (childState == "keys.down") {
        m_keyDown = getState("keys.down").value<int>();
    }
    
    // Apply camera updates if needed
    if (needsOrbitUpdate) {
        orbitCamera(0, 0); // Reposition camera using current orbit parameters
    } else if (needsFlyUpdate) {
        updateOrientation(); // Update camera using current fly parameters
    }
}

void CameraController::setCamera(vtkCamera *camera)
{
    m_camera = camera;
    if (m_camera) {
        runOnMainThread([this]() {
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
        });
        
        // Sync initial camera state to state tree
        syncCameraToState();
        
        // Also update orbit parameters in state
        getState("orbit.center.x").value(m_orbitCenter[0]);
        getState("orbit.center.y").value(m_orbitCenter[1]);
        getState("orbit.center.z").value(m_orbitCenter[2]);
        getState("orbit.distance").value(m_orbitDistance);
    }
}

void CameraController::setOrbitCenter(double x, double y, double z)
{
    m_orbitCenter[0] = x;
    m_orbitCenter[1] = y;
    m_orbitCenter[2] = z;
    
    // Update state
    getState("orbit.center.x").value(x);
    getState("orbit.center.y").value(y);
    getState("orbit.center.z").value(z);
}

void CameraController::setKeyBindings(int forward, int backward, int left, int right, int up, int down)
{
    m_keyForward = forward;
    m_keyBackward = backward;
    m_keyStrafeLeft = left;
    m_keyStrafeRight = right;
    m_keyUp = up;
    m_keyDown = down;
    
    // Update state
    getState("keys.forward").value(forward);
    getState("keys.backward").value(backward);
    getState("keys.strafe_left").value(left);
    getState("keys.strafe_right").value(right);
    getState("keys.up").value(up);
    getState("keys.down").value(down);
}

void CameraController::getCameraState(double pos[3], double dir[3], double up[3], double& fov)
{
    if (!m_camera) return;
    
    runOnMainThread([this, pos, dir, up, &fov]() {
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
    });
}

void CameraController::setCameraState(const double pos[3], const double dir[3], const double up[3], double fov)
{
    if (!m_camera) return;
    
    // Update internal position state
    m_position[0] = pos[0];
    m_position[1] = pos[1];
    m_position[2] = pos[2];
    
    runOnMainThread([this, pos, dir, up, fov]() {
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
    });
    
    // Sync camera state to state tree
    syncCameraToState();
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
    
    // Update read-only input state
    getState("input.keys_pressed_count").readOnly(false);
    getState("input.keys_pressed_count").value(static_cast<int>(m_keysPressed.size()));
    getState("input.keys_pressed_count").readOnly(true);
}

void CameraController::handleKeyRelease(int key)
{
    m_keysPressed.erase(key);
    
    // Update read-only input state
    getState("input.keys_pressed_count").readOnly(false);
    getState("input.keys_pressed_count").value(static_cast<int>(m_keysPressed.size()));
    getState("input.keys_pressed_count").readOnly(true);
}

void CameraController::handleMousePress(int button)
{
    if (button == Qt::LeftButton) {
        m_mouseLeftPressed = true;
        getState("input.mouse_left_pressed").readOnly(false);
        getState("input.mouse_left_pressed").value(true);
        getState("input.mouse_left_pressed").readOnly(true);
    } else if (button == Qt::RightButton) {
        m_mouseRightPressed = true;
        getState("input.mouse_right_pressed").readOnly(false);
        getState("input.mouse_right_pressed").value(true);
        getState("input.mouse_right_pressed").readOnly(true);
    } else if (button == Qt::MiddleButton) {
        m_mouseMiddlePressed = true;
        getState("input.mouse_middle_pressed").readOnly(false);
        getState("input.mouse_middle_pressed").value(true);
        getState("input.mouse_middle_pressed").readOnly(true);
    }
}

void CameraController::handleMouseRelease(int button)
{
    if (button == Qt::LeftButton) {
        m_mouseLeftPressed = false;
        getState("input.mouse_left_pressed").readOnly(false);
        getState("input.mouse_left_pressed").value(false);
        getState("input.mouse_left_pressed").readOnly(true);
    } else if (button == Qt::RightButton) {
        m_mouseRightPressed = false;
        getState("input.mouse_right_pressed").readOnly(false);
        getState("input.mouse_right_pressed").value(false);
        getState("input.mouse_right_pressed").readOnly(true);
    } else if (button == Qt::MiddleButton) {
        m_mouseMiddlePressed = false;
        getState("input.mouse_middle_pressed").readOnly(false);
        getState("input.mouse_middle_pressed").value(false);
        getState("input.mouse_middle_pressed").readOnly(true);
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
    } else if (m_mouseMiddlePressed) {
        // Middle mouse button - pan camera
        panCamera(dx, dy);
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

    // Update focal point based on new orientation
    m_focalPoint[0] = m_position[0] + forward[0];
    m_focalPoint[1] = m_position[1] + forward[1];
    m_focalPoint[2] = m_position[2] + forward[2];

    runOnMainThread([this]() {
        // Update camera
        m_camera->SetPosition(m_position);
        m_camera->SetFocalPoint(m_focalPoint);
        m_camera->SetViewUp(0, 1, 0);
    });
    
    // Update state
    getState("fly.position.x").value(m_position[0]);
    getState("fly.position.y").value(m_position[1]);
    getState("fly.position.z").value(m_position[2]);
    getState("fly.focal_point.x").value(m_focalPoint[0]);
    getState("fly.focal_point.y").value(m_focalPoint[1]);
    getState("fly.focal_point.z").value(m_focalPoint[2]);
    getState("fly.yaw").value(m_yaw);
    getState("fly.pitch").value(m_pitch);
    
    syncCameraToState();
}

void CameraController::syncCameraToState()
{
    if (!m_camera) return;
    
    double pos[3], dir[3], up[3], fov;
    getCameraState(pos, dir, up, fov);
    
    // Update position in state
    getState("position.x").value(pos[0]);
    getState("position.y").value(pos[1]);
    getState("position.z").value(pos[2]);
    
    // Update view direction in state
    getState("view_direction.x").value(dir[0]);
    getState("view_direction.y").value(dir[1]);
    getState("view_direction.z").value(dir[2]);
    
    // Update up vector in state
    getState("up_vector.x").value(up[0]);
    getState("up_vector.y").value(up[1]);
    getState("up_vector.z").value(up[2]);
    
    // Update field of view
    getState("fov").value(fov);
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
    
    // Update focal point to maintain current view direction
    runOnMainThread([this, forward, right, up, forwardVec, rightVec]() {
        double* focal = m_camera->GetFocalPoint();
        m_focalPoint[0] = focal[0] + forward * forwardVec[0] + right * rightVec[0];
        m_focalPoint[1] = focal[1] + up;
        m_focalPoint[2] = focal[2] + forward * forwardVec[2] + right * rightVec[2];
    });

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
    
    // Update state
    getState("orbit.azimuth").value(m_orbitAzimuth);
    getState("orbit.elevation").value(m_orbitElevation);
    
    // Convert to radians
    double azimuthRad = m_orbitAzimuth * M_PI / 180.0;
    double elevationRad = m_orbitElevation * M_PI / 180.0;
    
    // Calculate camera position on sphere around orbit center
    double x = m_orbitCenter[0] + m_orbitDistance * cos(elevationRad) * sin(azimuthRad);
    double y = m_orbitCenter[1] + m_orbitDistance * sin(elevationRad);
    double z = m_orbitCenter[2] + m_orbitDistance * cos(elevationRad) * cos(azimuthRad);
    
    runOnMainThread([this, x, y, z]() {
        // Update camera
        m_camera->SetPosition(x, y, z);
        m_camera->SetFocalPoint(m_orbitCenter);
        m_camera->SetViewUp(0, 1, 0);
    });
    
    syncCameraToState();
}

void CameraController::panCamera(int dx, int dy)
{
    if (!m_camera) return;
    
    // Pan speed factor
    double panSpeed = 0.001 * m_mouseSensitivity;
    
    if (m_mode == ORBIT_MODE) {
        // In orbit mode, pan by moving the orbit center
        // Get camera right and up vectors
        double pos[3], focal[3], up[3];
        m_camera->GetPosition(pos);
        m_camera->GetFocalPoint(focal);
        m_camera->GetViewUp(up);
        
        // Calculate right vector (cross product of view direction and up)
        double viewDir[3] = {focal[0] - pos[0], focal[1] - pos[1], focal[2] - pos[2]};
        double right[3];
        right[0] = viewDir[1] * up[2] - viewDir[2] * up[1];
        right[1] = viewDir[2] * up[0] - viewDir[0] * up[2];
        right[2] = viewDir[0] * up[1] - viewDir[1] * up[0];
        
        // Normalize vectors
        double rightLen = sqrt(right[0]*right[0] + right[1]*right[1] + right[2]*right[2]);
        double upLen = sqrt(up[0]*up[0] + up[1]*up[1] + up[2]*up[2]);
        if (rightLen > 1e-9 && upLen > 1e-9) {
            for (int i = 0; i < 3; i++) {
                right[i] /= rightLen;
                up[i] /= upLen;
            }
            
            // Pan is proportional to distance from center
            double panFactor = m_orbitDistance * panSpeed;
            
            // Update orbit center
            m_orbitCenter[0] += (-dx * right[0] + dy * up[0]) * panFactor;
            m_orbitCenter[1] += (-dx * right[1] + dy * up[1]) * panFactor;
            m_orbitCenter[2] += (-dx * right[2] + dy * up[2]) * panFactor;
            
            // Update state
            getState("orbit.center.x").value(m_orbitCenter[0]);
            getState("orbit.center.y").value(m_orbitCenter[1]);
            getState("orbit.center.z").value(m_orbitCenter[2]);
            
            // Reposition camera around new orbit center
            orbitCamera(0, 0);
        }
    } else {
        // Fly mode - pan by moving both position and focal point
        // Get camera right and up vectors
        double pos[3], focal[3], up[3];
        m_camera->GetPosition(pos);
        m_camera->GetFocalPoint(focal);
        m_camera->GetViewUp(up);
        
        // Calculate right vector
        double viewDir[3] = {focal[0] - pos[0], focal[1] - pos[1], focal[2] - pos[2]};
        double right[3];
        right[0] = viewDir[1] * up[2] - viewDir[2] * up[1];
        right[1] = viewDir[2] * up[0] - viewDir[0] * up[2];
        right[2] = viewDir[0] * up[1] - viewDir[1] * up[0];
        
        // Normalize vectors
        double rightLen = sqrt(right[0]*right[0] + right[1]*right[1] + right[2]*right[2]);
        double upLen = sqrt(up[0]*up[0] + up[1]*up[1] + up[2]*up[2]);
        if (rightLen > 1e-9 && upLen > 1e-9) {
            for (int i = 0; i < 3; i++) {
                right[i] /= rightLen;
                up[i] /= upLen;
            }
            
            // Pan factor for fly mode
            double panFactor = 0.1 * panSpeed * m_movementSpeed;
            
            // Pan delta
            double deltaX = (-dx * right[0] + dy * up[0]) * panFactor;
            double deltaY = (-dx * right[1] + dy * up[1]) * panFactor;
            double deltaZ = (-dx * right[2] + dy * up[2]) * panFactor;
            
            // Update position and focal point
            m_position[0] += deltaX;
            m_position[1] += deltaY;
            m_position[2] += deltaZ;
            m_focalPoint[0] += deltaX;
            m_focalPoint[1] += deltaY;
            m_focalPoint[2] += deltaZ;
            
            // Apply to camera
            runOnMainThread([this]() {
                m_camera->SetPosition(m_position);
                m_camera->SetFocalPoint(m_focalPoint);
            });
            
            syncCameraToState();
        }
    }
}

void CameraController::updateOrbitCenterFromBounds(double minX, double minY, double minZ, double maxX, double maxY, double maxZ)
{
    // Set orbit center to center of bounding box
    m_orbitCenter[0] = (minX + maxX) * 0.5;
    m_orbitCenter[1] = (minY + maxY) * 0.5;
    m_orbitCenter[2] = (minZ + maxZ) * 0.5;
    
    // Update camera focal point to maintain orbit
    if (m_camera && m_mode == ORBIT_MODE) {
        runOnMainThread([this]() {
            m_camera->SetFocalPoint(m_orbitCenter);
        });
    }
}

void CameraController::resetView(double minX, double minY, double minZ, double maxX, double maxY, double maxZ)
{
    if (!m_camera) return;
    
    // Calculate bounding box center and size
    double centerX = (minX + maxX) * 0.5;
    double centerY = (minY + maxY) * 0.5;
    double centerZ = (minZ + maxZ) * 0.5;
    
    double sizeX = maxX - minX;
    double sizeY = maxY - minY;
    double sizeZ = maxZ - minZ;
    double maxSize = std::max({sizeX, sizeY, sizeZ});
    
    // Position camera to view entire scene
    double distance = maxSize * 2.0; // Distance to see entire bounding box
    
    if (m_mode == ORBIT_MODE) {
        // Update orbit center to bounding box center
        m_orbitCenter[0] = centerX;
        m_orbitCenter[1] = centerY;
        m_orbitCenter[2] = centerZ;
        m_orbitDistance = distance;
        m_orbitAzimuth = 45.0;
        m_orbitElevation = 30.0;
        
        // Update state
        getState("orbit.center.x").value(m_orbitCenter[0]);
        getState("orbit.center.y").value(m_orbitCenter[1]);
        getState("orbit.center.z").value(m_orbitCenter[2]);
        getState("orbit.distance").value(m_orbitDistance);
        getState("orbit.azimuth").value(m_orbitAzimuth);
        getState("orbit.elevation").value(m_orbitElevation);
        
        // Position camera
        orbitCamera(0, 0);
    } else {
        // Fly mode - position camera back from center
        m_position[0] = centerX;
        m_position[1] = centerY + maxSize * 0.5;
        m_position[2] = centerZ + distance;
        
        m_focalPoint[0] = centerX;
        m_focalPoint[1] = centerY;
        m_focalPoint[2] = centerZ;
        
        // Calculate yaw and pitch to look at center
        m_yaw = 0.0;
        m_pitch = -20.0;
        
        updateOrientation();
    }
}
