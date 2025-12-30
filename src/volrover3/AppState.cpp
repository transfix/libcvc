#include <volrover3/AppState.h>
#include <cvc/geometry_file_io.h>
#include <cvc/volume_file_io.h>
#include <boost/lexical_cast.hpp>
#include <QtCore/Qt>

AppState& AppState::instance()
{
    static AppState instance;
    return instance;
}

AppState::AppState()
{
    // Initialize default world bounds
    cvc::bounding_box defaultBounds(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
    std::string boundsStr = 
        boost::lexical_cast<std::string>(defaultBounds[0]) + "," +
        boost::lexical_cast<std::string>(defaultBounds[1]) + "," +
        boost::lexical_cast<std::string>(defaultBounds[2]) + "," +
        boost::lexical_cast<std::string>(defaultBounds[3]) + "," +
        boost::lexical_cast<std::string>(defaultBounds[4]) + "," +
        boost::lexical_cast<std::string>(defaultBounds[5]);
    getState("world_bounds").value(boundsStr);
    
    // Initialize visibility states
    getState("grid_visible").value(true);
    getState("axis_visible").value(true);
    getState("geometry_bbox_visible").value(false);
    getState("volume_bbox_visible").value(false);
    
    // Initialize camera settings
    getState("camera_mode").value(0);  // 0 = orbit, 1 = fly
    getState("camera_speed").value(5.0);
    getState("camera_sensitivity").value(1.0);
    getState("camera_invert_mouse").value(false);
    
    // Initialize camera key bindings (Qt::Key enum values)
    getState("camera_key_forward").value(static_cast<int>(Qt::Key_W));
    getState("camera_key_backward").value(static_cast<int>(Qt::Key_S));
    getState("camera_key_left").value(static_cast<int>(Qt::Key_A));
    getState("camera_key_right").value(static_cast<int>(Qt::Key_D));
    getState("camera_key_up").value(static_cast<int>(Qt::Key_Space));
    getState("camera_key_down").value(static_cast<int>(Qt::Key_Control));
    
    // Initialize camera position (looking at origin from distance)
    getState("camera_position_x").value(0.0);
    getState("camera_position_y").value(-10.0);
    getState("camera_position_z").value(5.0);
    
    // Initialize camera view direction (looking at origin)
    getState("camera_view_dir_x").value(0.0);
    getState("camera_view_dir_y").value(1.0);
    getState("camera_view_dir_z").value(-0.5);
    
    // Initialize camera up vector (standard Z-up)
    getState("camera_up_x").value(0.0);
    getState("camera_up_y").value(0.0);
    getState("camera_up_z").value(1.0);
    
    // Initialize field of view (degrees)
    getState("camera_fov").value(60.0);
    
    // Initialize default grayscale transfer function
    // Color table: scalar, r, g, b (grayscale 0-1)
    std::string defaultColorTable = "0.0,0.0,0.0,0.0,1.0,1.0,1.0,1.0";
    getState("transfer_function_color").value(defaultColorTable);
    
    // Opacity table: scalar, opacity (linear ramp)
    std::string defaultOpacityTable = "0.0,0.0,0.5,0.5,1.0,1.0";
    getState("transfer_function_opacity").value(defaultOpacityTable);
    
    // Initialize change tracker
    getState("transfer_function_changed").value(false);
}

cvc::state& AppState::getState(const std::string& path)
{
    return cvc::state::instance()("volrover3")(path);
}

cvc::geometry AppState::geometry()
{
    try {
        auto geomPtr = getState("geometry_data").data<std::shared_ptr<cvc::geometry>>();
        if (geomPtr) {
            return *geomPtr;
        }
    } catch (...) {
        // No geometry stored yet
    }
    return cvc::geometry();
}

void AppState::setGeometry(const cvc::geometry& geom)
{
    auto geomPtr = std::make_shared<cvc::geometry>(geom);
    getState("geometry_data").data(boost::any(geomPtr));
    
    // Update world bounds to contain geometry if needed
    if (geom.num_points() > 0) {
        cvc::bounding_box geomBounds = geom.extents();
        cvc::bounding_box currentBounds = worldBounds();
        
        // Expand world bounds if geometry is outside
        cvc::bounding_box newBounds(
            std::min(currentBounds[0], geomBounds[0]),
            std::min(currentBounds[1], geomBounds[1]),
            std::min(currentBounds[2], geomBounds[2]),
            std::max(currentBounds[3], geomBounds[3]),
            std::max(currentBounds[4], geomBounds[4]),
            std::max(currentBounds[5], geomBounds[5])
        );
        
        setWorldBounds(newBounds);
    }
    
    // Trigger geometry changed notification
    getState("geometry_changed").value(true);
}

cvc::volume AppState::volume()
{
    try {
        auto volPtr = getState("volume_data").data<std::shared_ptr<cvc::volume>>();
        if (volPtr) {
            return *volPtr;
        }
    } catch (...) {
        // No volume stored yet
    }
    return cvc::volume();
}

void AppState::setVolume(const cvc::volume& vol)
{
    auto volPtr = std::make_shared<cvc::volume>(vol);
    getState("volume_data").data(boost::any(volPtr));
    
    // Update world bounds to contain volume
    cvc::bounding_box volBounds = vol.boundingBox();
    cvc::bounding_box currentBounds = worldBounds();
    
    // Expand world bounds if volume is outside
    cvc::bounding_box newBounds(
        std::min(currentBounds[0], volBounds[0]),
        std::min(currentBounds[1], volBounds[1]),
        std::min(currentBounds[2], volBounds[2]),
        std::max(currentBounds[3], volBounds[3]),
        std::max(currentBounds[4], volBounds[4]),
        std::max(currentBounds[5], volBounds[5])
    );
    
    setWorldBounds(newBounds);
    
    // Trigger volume changed notification
    getState("volume_changed").value(true);
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
    getState("world_bounds").value(boundsStr);
}

bool AppState::gridVisible()
{
    return getState("grid_visible").value<bool>();
}

void AppState::setGridVisible(bool visible)
{
    getState("grid_visible").value(visible);
}

bool AppState::axisVisible()
{
    return getState("axis_visible").value<bool>();
}

void AppState::setAxisVisible(bool visible)
{
    getState("axis_visible").value(visible);
}

boost::signals2::connection AppState::onGeometryChanged(const boost::function<void()>& callback)
{
    return getState("geometry_changed").valueChanged.connect(callback);
}

boost::signals2::connection AppState::onVolumeChanged(const boost::function<void()>& callback)
{
    return getState("volume_changed").valueChanged.connect(callback);
}

boost::signals2::connection AppState::onWorldBoundsChanged(const boost::function<void()>& callback)
{
    return getState("world_bounds").valueChanged.connect(callback);
}

boost::signals2::connection AppState::onGridVisibilityChanged(const boost::function<void()>& callback)
{
    return getState("grid_visible").valueChanged.connect(callback);
}

boost::signals2::connection AppState::onAxisVisibilityChanged(const boost::function<void()>& callback)
{
    return getState("axis_visible").valueChanged.connect(callback);
}

bool AppState::geometryBBoxVisible()
{
    return getState("geometry_bbox_visible").value<bool>();
}

void AppState::setGeometryBBoxVisible(bool visible)
{
    getState("geometry_bbox_visible").value(visible);
}

bool AppState::volumeBBoxVisible()
{
    return getState("volume_bbox_visible").value<bool>();
}

void AppState::setVolumeBBoxVisible(bool visible)
{
    getState("volume_bbox_visible").value(visible);
}

int AppState::cameraMode()
{
    return getState("camera_mode").value<int>();
}

void AppState::setCameraMode(int mode)
{
    getState("camera_mode").value(mode);
}

std::shared_ptr<cvc::geometry> AppState::geometryPtr()
{
    try {
        return getState("geometry_data").data<std::shared_ptr<cvc::geometry>>();
    } catch (...) {
        return nullptr;
    }
}

std::shared_ptr<cvc::volume> AppState::volumePtr()
{
    try {
        return getState("volume_data").data<std::shared_ptr<cvc::volume>>();
    } catch (...) {
        return nullptr;
    }
}

boost::signals2::connection AppState::onGeometryBBoxVisibilityChanged(const boost::function<void()>& callback)
{
    return getState("geometry_bbox_visible").valueChanged.connect(callback);
}

boost::signals2::connection AppState::onVolumeBBoxVisibilityChanged(const boost::function<void()>& callback)
{
    return getState("volume_bbox_visible").valueChanged.connect(callback);
}

boost::signals2::connection AppState::onCameraModeChanged(const boost::function<void()>& callback)
{
    return getState("camera_mode").valueChanged.connect(callback);
}

// Camera settings
double AppState::cameraSpeed()
{
    return getState("camera_speed").value<double>();
}

void AppState::setCameraSpeed(double speed)
{
    getState("camera_speed").value(speed);
}

double AppState::cameraSensitivity()
{
    return getState("camera_sensitivity").value<double>();
}

void AppState::setCameraSensitivity(double sensitivity)
{
    getState("camera_sensitivity").value(sensitivity);
}

bool AppState::cameraInvertMouse()
{
    return getState("camera_invert_mouse").value<bool>();
}

void AppState::setCameraInvertMouse(bool invert)
{
    getState("camera_invert_mouse").value(invert);
}

int AppState::cameraKeyForward()
{
    return getState("camera_key_forward").value<int>();
}

void AppState::setCameraKeyForward(int key)
{
    getState("camera_key_forward").value(key);
}

int AppState::cameraKeyBackward()
{
    return getState("camera_key_backward").value<int>();
}

void AppState::setCameraKeyBackward(int key)
{
    getState("camera_key_backward").value(key);
}

int AppState::cameraKeyLeft()
{
    return getState("camera_key_left").value<int>();
}

void AppState::setCameraKeyLeft(int key)
{
    getState("camera_key_left").value(key);
}

int AppState::cameraKeyRight()
{
    return getState("camera_key_right").value<int>();
}

void AppState::setCameraKeyRight(int key)
{
    getState("camera_key_right").value(key);
}

int AppState::cameraKeyUp()
{
    return getState("camera_key_up").value<int>();
}

void AppState::setCameraKeyUp(int key)
{
    getState("camera_key_up").value(key);
}

int AppState::cameraKeyDown()
{
    return getState("camera_key_down").value<int>();
}

void AppState::setCameraKeyDown(int key)
{
    getState("camera_key_down").value(key);
}

void AppState::getCameraPosition(double& x, double& y, double& z)
{
    x = getState("camera_position_x").value<double>();
    y = getState("camera_position_y").value<double>();
    z = getState("camera_position_z").value<double>();
}

void AppState::setCameraPosition(double x, double y, double z)
{
    getState("camera_position_x").value(x);
    getState("camera_position_y").value(y);
    getState("camera_position_z").value(z);
    getState("camera_changed").value(true);
}

void AppState::getCameraViewDirection(double& x, double& y, double& z)
{
    x = getState("camera_view_dir_x").value<double>();
    y = getState("camera_view_dir_y").value<double>();
    z = getState("camera_view_dir_z").value<double>();
}

void AppState::setCameraViewDirection(double x, double y, double z)
{
    getState("camera_view_dir_x").value(x);
    getState("camera_view_dir_y").value(y);
    getState("camera_view_dir_z").value(z);
    getState("camera_changed").value(true);
}

void AppState::getCameraUpVector(double& x, double& y, double& z)
{
    x = getState("camera_up_x").value<double>();
    y = getState("camera_up_y").value<double>();
    z = getState("camera_up_z").value<double>();
}

void AppState::setCameraUpVector(double x, double y, double z)
{
    getState("camera_up_x").value(x);
    getState("camera_up_y").value(y);
    getState("camera_up_z").value(z);
    getState("camera_changed").value(true);
}

double AppState::cameraFieldOfView()
{
    return getState("camera_fov").value<double>();
}

void AppState::setCameraFieldOfView(double fov)
{
    getState("camera_fov").value(fov);
    getState("camera_changed").value(true);
}

boost::signals2::connection AppState::onCameraChanged(const boost::function<void()>& callback)
{
    return getState("camera_changed").valueChanged.connect(callback);
}

std::vector<double> AppState::transferFunctionColorTable()
{
    std::string tableStr = getState("transfer_function_color").value();
    std::vector<std::string> values = getState("transfer_function_color").values();
    
    std::vector<double> table;
    for (const auto& val : values) {
        try {
            table.push_back(boost::lexical_cast<double>(val));
        } catch (...) {
            // Skip invalid values
        }
    }
    
    return table;
}

void AppState::setTransferFunctionColorTable(const std::vector<double>& table)
{
    std::string tableStr;
    for (size_t i = 0; i < table.size(); ++i) {
        if (i > 0) tableStr += ",";
        tableStr += boost::lexical_cast<std::string>(table[i]);
    }
    getState("transfer_function_color").value(tableStr);
    // Toggle to ensure change notification fires
    bool current = getState("transfer_function_changed").value<bool>();
    getState("transfer_function_changed").value(!current);
}

std::vector<double> AppState::transferFunctionOpacityTable()
{
    std::string tableStr = getState("transfer_function_opacity").value();
    std::vector<std::string> values = getState("transfer_function_opacity").values();
    
    std::vector<double> table;
    for (const auto& val : values) {
        try {
            table.push_back(boost::lexical_cast<double>(val));
        } catch (...) {
            // Skip invalid values
        }
    }
    
    return table;
}

void AppState::setTransferFunctionOpacityTable(const std::vector<double>& table)
{
    std::string tableStr;
    for (size_t i = 0; i < table.size(); ++i) {
        if (i > 0) tableStr += ",";
        tableStr += boost::lexical_cast<std::string>(table[i]);
    }
    getState("transfer_function_opacity").value(tableStr);
    // Toggle to ensure change notification fires
    bool current = getState("transfer_function_changed").value<bool>();
    getState("transfer_function_changed").value(!current);
}

boost::signals2::connection AppState::onTransferFunctionChanged(const boost::function<void()>& callback)
{
    return getState("transfer_function_changed").valueChanged.connect(callback);
}
