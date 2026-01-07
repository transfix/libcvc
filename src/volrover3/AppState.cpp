#include <volrover3/AppState.h>
#include <cvc/geometry_file_io.h>
#include <cvc/volume_file_io.h>
#include <boost/lexical_cast.hpp>
#include <QtCore/Qt>
#include <sstream>
#include <algorithm>
#include <cmath>

AppState& AppState::instance()
{
    static AppState instance; // Uses default parameter "volrover3"
    return instance;
}

AppState::AppState(const std::string& statePrefix)
    : m_statePrefix(statePrefix)
{
    initializeDefaults();
}

void AppState::initializeDefaults()
{
    // Initialize default world bounds to match null graphic default
    cvc::bounding_box defaultBounds(-0.5, -0.5, -0.5, 0.5, 0.5, 0.5);
    std::string boundsStr = 
        boost::lexical_cast<std::string>(defaultBounds[0]) + "," +
        boost::lexical_cast<std::string>(defaultBounds[1]) + "," +
        boost::lexical_cast<std::string>(defaultBounds[2]) + "," +
        boost::lexical_cast<std::string>(defaultBounds[3]) + "," +
        boost::lexical_cast<std::string>(defaultBounds[4]) + "," +
        boost::lexical_cast<std::string>(defaultBounds[5]);
    getState("world_bounds").value(boundsStr);
    getState("world_bounds").comment("Computed from graphics bounds - read only");
    getState("world_bounds").readOnly(true);
    
    // Grid and axis visibility now managed by GridNode/AxisNode state trees
    
    // Initialize camera settings
    getState("camera.mode").value(0);  // 0 = orbit, 1 = fly
    getState("camera.speed").value(5.0);
    getState("camera.sensitivity").value(1.0);
    getState("camera.invert_mouse").value(false);
    
    // Initialize camera key bindings (Qt::Key enum values)
    getState("camera.key_forward").value(static_cast<int>(Qt::Key_W));
    getState("camera.key_backward").value(static_cast<int>(Qt::Key_S));
    getState("camera.key_left").value(static_cast<int>(Qt::Key_A));
    getState("camera.key_right").value(static_cast<int>(Qt::Key_D));
    getState("camera.key_up").value(static_cast<int>(Qt::Key_Space));
    getState("camera.key_down").value(static_cast<int>(Qt::Key_Control));
    
    // Initialize camera position (looking at origin from distance)
    getState("camera.position.x").value(0.0);
    getState("camera.position.y").value(-10.0);
    getState("camera.position.z").value(5.0);
    
    // Initialize camera view direction (looking at origin)
    getState("camera.view_direction.x").value(0.0);
    getState("camera.view_direction.y").value(1.0);
    getState("camera.view_direction.z").value(-0.5);
    
    // Initialize camera up vector (standard Z-up)
    getState("camera.up_vector.x").value(0.0);
    getState("camera.up_vector.y").value(0.0);
    getState("camera.up_vector.z").value(1.0);
    
    // Initialize field of view (degrees)
    getState("camera.fov").value(60.0);
}

cvc::state& AppState::getState(const std::string& path)
{
    return cvc::state::instance()(m_statePrefix)(path);
}

cvc::state& AppState::getRootState()
{
    return cvc::state::instance()(m_statePrefix);
}

cvc::bounding_box AppState::worldBounds()
{
    std::string boundsStr = getState("world_bounds").value();
    std::vector<std::string> values = getState("world_bounds").values();
    
    if (values.size() == 6) {
        return cvc::bounding_box(
            boost::lexical_cast<double>(values[0]),
            boost::lexical_cast<double>(values[1]),
            boost::lexical_cast<double>(values[2]),
            boost::lexical_cast<double>(values[3]),
            boost::lexical_cast<double>(values[4]),
            boost::lexical_cast<double>(values[5])
        );
    }
    
    return cvc::bounding_box(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
}

void AppState::setWorldBounds(const cvc::bounding_box& bounds)
{
    std::string boundsStr = 
        boost::lexical_cast<std::string>(bounds[0]) + "," +
        boost::lexical_cast<std::string>(bounds[1]) + "," +
        boost::lexical_cast<std::string>(bounds[2]) + "," +
        boost::lexical_cast<std::string>(bounds[3]) + "," +
        boost::lexical_cast<std::string>(bounds[4]) + "," +
        boost::lexical_cast<std::string>(bounds[5]);
    // Temporarily allow write to update computed bounds
    getState("world_bounds").readOnly(false);
    getState("world_bounds").value(boundsStr);
    getState("world_bounds").readOnly(true);
}

// ===========================
// Camera Methods
// ===========================

int AppState::cameraMode()
{
    return getState("camera.mode").value<int>();
}

void AppState::setCameraMode(int mode)
{
    getState("camera.mode").value(mode);
}

boost::signals2::connection AppState::onWorldBoundsChanged(const boost::function<void()>& callback)
{
    return getState("world_bounds").valueChanged.connect(callback);
}

boost::signals2::connection AppState::onCameraModeChanged(const boost::function<void()>& callback)
{
    return getState("camera.mode").valueChanged.connect(callback);
}

// Camera settings
double AppState::cameraSpeed()
{
    return getState("camera.speed").value<double>();
}

void AppState::setCameraSpeed(double speed)
{
    getState("camera.speed").value(speed);
}

double AppState::cameraSensitivity()
{
    return getState("camera.sensitivity").value<double>();
}

void AppState::setCameraSensitivity(double sensitivity)
{
    getState("camera.sensitivity").value(sensitivity);
}

bool AppState::cameraInvertMouse()
{
    return getState("camera.invert_mouse").value<bool>();
}

void AppState::setCameraInvertMouse(bool invert)
{
    getState("camera.invert_mouse").value(invert);
}

int AppState::cameraKeyForward()
{
    return getState("camera.key_forward").value<int>();
}

void AppState::setCameraKeyForward(int key)
{
    getState("camera.key_forward").value(key);
}

int AppState::cameraKeyBackward()
{
    return getState("camera.key_backward").value<int>();
}

void AppState::setCameraKeyBackward(int key)
{
    getState("camera.key_backward").value(key);
}

int AppState::cameraKeyLeft()
{
    return getState("camera.key_left").value<int>();
}

void AppState::setCameraKeyLeft(int key)
{
    getState("camera.key_left").value(key);
}

int AppState::cameraKeyRight()
{
    return getState("camera.key_right").value<int>();
}

void AppState::setCameraKeyRight(int key)
{
    getState("camera.key_right").value(key);
}

int AppState::cameraKeyUp()
{
    return getState("camera.key_up").value<int>();
}

void AppState::setCameraKeyUp(int key)
{
    getState("camera.key_up").value(key);
}

int AppState::cameraKeyDown()
{
    return getState("camera.key_down").value<int>();
}

void AppState::setCameraKeyDown(int key)
{
    getState("camera.key_down").value(key);
}

void AppState::getCameraPosition(double& x, double& y, double& z)
{
    x = getState("camera.position.x").value<double>();
    y = getState("camera.position.y").value<double>();
    z = getState("camera.position.z").value<double>();
}

void AppState::setCameraPosition(double x, double y, double z)
{
    getState("camera.position.x").value(x);
    getState("camera.position.y").value(y);
    getState("camera.position.z").value(z);
}

void AppState::getCameraViewDirection(double& x, double& y, double& z)
{
    x = getState("camera.view_direction.x").value<double>();
    y = getState("camera.view_direction.y").value<double>();
    z = getState("camera.view_direction.z").value<double>();
}

void AppState::setCameraViewDirection(double x, double y, double z)
{
    getState("camera.view_direction.x").value(x);
    getState("camera.view_direction.y").value(y);
    getState("camera.view_direction.z").value(z);
}

void AppState::getCameraUpVector(double& x, double& y, double& z)
{
    x = getState("camera.up_vector.x").value<double>();
    y = getState("camera.up_vector.y").value<double>();
    z = getState("camera.up_vector.z").value<double>();
}

void AppState::setCameraUpVector(double x, double y, double z)
{
    getState("camera.up_vector.x").value(x);
    getState("camera.up_vector.y").value(y);
    getState("camera.up_vector.z").value(z);
}

double AppState::cameraFieldOfView()
{
    return getState("camera.fov").value<double>();
}

void AppState::setCameraFieldOfView(double fov)
{
    getState("camera.fov").value(fov);
}

boost::signals2::connection AppState::onCameraChanged(const boost::function<void()>& callback)
{
    // Connect to the "camera" parent node's childChanged signal
    // Any child value change (camera.position.x, camera.fov, etc.) will trigger this
    return getState("camera").childChanged.connect([callback](const std::string&) {
        callback();
    });
}
