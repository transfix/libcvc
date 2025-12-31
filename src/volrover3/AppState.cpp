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
    
    // Initialize default world bounds (-0.5 to 0.5 on each axis)
    getState("world_bounds").value("-0.5,-0.5,-0.5,0.5,0.5,0.5");
    
    // Initialize grid plane visibility (all visible by default)
    getState("grid_yz_plane_visible").value(true);
    getState("grid_xz_plane_visible").value(true);
    getState("grid_xy_plane_visible").value(true);
    
    // Initialize grid divisions (32 per axis by default)
    getState("grid_divisions_x").value(32);
    getState("grid_divisions_y").value(32);
    getState("grid_divisions_z").value(32);
    
    // Initialize grid tick intervals (8 per axis by default)
    getState("grid_tick_interval_x").value(8);
    getState("grid_tick_interval_y").value(8);
    getState("grid_tick_interval_z").value(8);
    getState("grid_ticks_visible").value(false);
    
    // Initialize colors (RGB triplets)
    getState("grid_color").value("0.5,0.5,0.5");  // Gray
    getState("grid_yz_plane_color").value("0.5,0.5,0.5");  // Gray
    getState("grid_xz_plane_color").value("0.5,0.5,0.5");  // Gray
    getState("grid_xy_plane_color").value("0.5,0.5,0.5");  // Gray
    getState("grid_tick_label_color").value("1.0,1.0,1.0");  // White
    getState("grid_tick_label_font_size").value(12);
    
    // Initialize world bounding box coordinate settings (no interval - shows at vertices)
    getState("world_bbox_coordinates_visible").value(true);  // Visible by default
    getState("world_bbox_coordinate_color").value("1.0,1.0,1.0");  // White
    getState("world_bbox_coordinate_font_size").value(12);
    
    // Initialize world bounding box visibility (on by default to show coordinates)
    getState("world_bbox_visible").value(true);
    
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
    getState("world_bounds").value(boundsStr);
}

cvc::bounding_box AppState::computeGraphicsBounds()
{
    // Iterate through all graphics objects in the state tree,
    // apply their transforms, and compute the union of all transformed bounding boxes.
    
    cvc::state& graphicsState = getState("graphics");
    
    // Initialize with invalid bounds
    cvc::bounding_box result;
    bool foundAny = false;
    
    // Graphics children are stored under a "children" container
    // Check if children container exists
    if (!graphicsState("children").initialized()) {
        return result; // No children, return empty bounds
    }
    
    cvc::state& childrenState = graphicsState("children");
    
    // Get list of all child graphics objects
    std::vector<std::string> allDescendants = childrenState.children();
    
    // Filter to only direct children (not nested descendants)
    std::vector<std::string> childNames;
    std::string childrenPath = childrenState.fullName();
    int expectedDepth = std::count(childrenPath.begin(), childrenPath.end(), '.') + 1;
    
    for (const auto& descendant : allDescendants) {
        int depth = std::count(descendant.begin(), descendant.end(), '.');
        if (depth == expectedDepth) {
            // Extract just the child name (last component after final '.')
            size_t lastDot = descendant.find_last_of('.');
            std::string childName = (lastDot != std::string::npos) ? descendant.substr(lastDot + 1) : descendant;
            childNames.push_back(childName);
        }
    }
    
    for (const auto& childName : childNames) {
        cvc::state& childState = childrenState(childName);
        
        // Check if this child has a bounding box in metadata
        try {
            std::string bboxStr = childState("metadata")("bounding_box").value();
            if (bboxStr.empty()) continue;
            
            cvc::bounding_box childBBox(bboxStr);
            
            // Get transform if it exists and apply it to the bounding box
            try {
                std::string transformStr = childState("transform").value();
                if (!transformStr.empty()) {
                    // Parse the 4x4 transform matrix (row-major order, comma-separated)
                    std::vector<double> matrixValues;
                    std::stringstream ss(transformStr);
                    std::string token;
                    while (std::getline(ss, token, ',')) {
                        matrixValues.push_back(std::stod(token));
                    }
                    
                    if (matrixValues.size() == 16) {
                        // Get the 8 vertices of the bounding box
                        double vertices[8][3] = {
                            {childBBox.minx, childBBox.miny, childBBox.minz},
                            {childBBox.maxx, childBBox.miny, childBBox.minz},
                            {childBBox.minx, childBBox.maxy, childBBox.minz},
                            {childBBox.maxx, childBBox.maxy, childBBox.minz},
                            {childBBox.minx, childBBox.miny, childBBox.maxz},
                            {childBBox.maxx, childBBox.miny, childBBox.maxz},
                            {childBBox.minx, childBBox.maxy, childBBox.maxz},
                            {childBBox.maxx, childBBox.maxy, childBBox.maxz}
                        };
                        
                        // Transform all 8 vertices
                        double transformedVertices[8][3];
                        for (int i = 0; i < 8; ++i) {
                            double x = vertices[i][0];
                            double y = vertices[i][1];
                            double z = vertices[i][2];
                            
                            // Apply 4x4 matrix transform: [x' y' z' w'] = [x y z 1] * M
                            transformedVertices[i][0] = matrixValues[0]*x + matrixValues[1]*y + matrixValues[2]*z + matrixValues[3];
                            transformedVertices[i][1] = matrixValues[4]*x + matrixValues[5]*y + matrixValues[6]*z + matrixValues[7];
                            transformedVertices[i][2] = matrixValues[8]*x + matrixValues[9]*y + matrixValues[10]*z + matrixValues[11];
                            // w component: matrixValues[12]*x + matrixValues[13]*y + matrixValues[14]*z + matrixValues[15]
                            // For affine transforms, w should be 1, so we don't need to divide
                        }
                        
                        // Compute axis-aligned bounding box of transformed vertices
                        double minX = transformedVertices[0][0];
                        double maxX = transformedVertices[0][0];
                        double minY = transformedVertices[0][1];
                        double maxY = transformedVertices[0][1];
                        double minZ = transformedVertices[0][2];
                        double maxZ = transformedVertices[0][2];
                        
                        for (int i = 1; i < 8; ++i) {
                            minX = std::min(minX, transformedVertices[i][0]);
                            maxX = std::max(maxX, transformedVertices[i][0]);
                            minY = std::min(minY, transformedVertices[i][1]);
                            maxY = std::max(maxY, transformedVertices[i][1]);
                            minZ = std::min(minZ, transformedVertices[i][2]);
                            maxZ = std::max(maxZ, transformedVertices[i][2]);
                        }
                        
                        // Update childBBox with transformed bounds
                        childBBox.setMin(minX, minY, minZ);
                        childBBox.setMax(maxX, maxY, maxZ);
                    }
                }
            } catch (...) {
                // No transform or parsing failed, use untransformed bbox
            }
            
            // Union with result
            if (!foundAny) {
                result = childBBox;
                foundAny = true;
            } else {
                // Compute union: min of mins, max of maxes
                result.setMin(
                    std::min(result.minx, childBBox.minx),
                    std::min(result.miny, childBBox.miny),
                    std::min(result.minz, childBBox.minz)
                );
                result.setMax(
                    std::max(result.maxx, childBBox.maxx),
                    std::max(result.maxy, childBBox.maxy),
                    std::max(result.maxz, childBBox.maxz)
                );
            }
        } catch (...) {
            // Child doesn't have a bounding box, skip it
            continue;
        }
    }
    
    return result;
}

bool AppState::worldBBoxVisible()
{
    return getState("world_bbox_visible").value<bool>();
}

void AppState::setWorldBBoxVisible(bool visible)
{
    getState("world_bbox_visible").value(visible);
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

boost::signals2::connection AppState::onWorldBoundsChanged(const boost::function<void()>& callback)
{
    return getState("world_bounds").valueChanged.connect(callback);
}

boost::signals2::connection AppState::onWorldBBoxVisibilityChanged(const boost::function<void()>& callback)
{
    return getState("world_bbox_visible").valueChanged.connect(callback);
}

boost::signals2::connection AppState::onGridVisibilityChanged(const boost::function<void()>& callback)
{
    return getState("grid_visible").valueChanged.connect(callback);
}

boost::signals2::connection AppState::onAxisVisibilityChanged(const boost::function<void()>& callback)
{
    return getState("axis_visible").valueChanged.connect(callback);
}

bool AppState::gridYZPlaneVisible()
{
    return getState("grid_yz_plane_visible").value<bool>();
}

void AppState::setGridYZPlaneVisible(bool visible)
{
    getState("grid_yz_plane_visible").value(visible);
}

bool AppState::gridXZPlaneVisible()
{
    return getState("grid_xz_plane_visible").value<bool>();
}

void AppState::setGridXZPlaneVisible(bool visible)
{
    getState("grid_xz_plane_visible").value(visible);
}

bool AppState::gridXYPlaneVisible()
{
    return getState("grid_xy_plane_visible").value<bool>();
}

void AppState::setGridXYPlaneVisible(bool visible)
{
    getState("grid_xy_plane_visible").value(visible);
}

void AppState::getGridDivisions(int& x, int& y, int& z)
{
    x = getState("grid_divisions_x").value<int>();
    y = getState("grid_divisions_y").value<int>();
    z = getState("grid_divisions_z").value<int>();
}

void AppState::setGridDivisions(int x, int y, int z)
{
    getState("grid_divisions_x").value(x);
    getState("grid_divisions_y").value(y);
    getState("grid_divisions_z").value(z);
}

void AppState::getGridTickIntervals(int& x, int& y, int& z)
{
    x = getState("grid_tick_interval_x").value<int>();
    y = getState("grid_tick_interval_y").value<int>();
    z = getState("grid_tick_interval_z").value<int>();
}

void AppState::setGridTickIntervals(int x, int y, int z)
{
    getState("grid_tick_interval_x").value(x);
    getState("grid_tick_interval_y").value(y);
    getState("grid_tick_interval_z").value(z);
}

bool AppState::gridTicksVisible()
{
    cvc::state& tickState = getState("grid_ticks_visible");
    if (!tickState.initialized()) {
        // Default to true if not yet set
        tickState.value(true);
    }
    return tickState.value<bool>();
}

void AppState::setGridTicksVisible(bool visible)
{
    getState("grid_ticks_visible").value(visible);
}

void AppState::getGridColor(double& r, double& g, double& b)
{
    std::string colorStr = getState("grid_color").value<std::string>();
    std::vector<std::string> parts;
    boost::split(parts, colorStr, boost::is_any_of(","));
    if (parts.size() >= 3) {
        r = boost::lexical_cast<double>(parts[0]);
        g = boost::lexical_cast<double>(parts[1]);
        b = boost::lexical_cast<double>(parts[2]);
    } else {
        r = g = b = 0.5; // Default gray
    }
}

void AppState::setGridColor(double r, double g, double b)
{
    std::string colorStr = 
        boost::lexical_cast<std::string>(r) + "," +
        boost::lexical_cast<std::string>(g) + "," +
        boost::lexical_cast<std::string>(b);
    getState("grid_color").value(colorStr);
}

void AppState::getGridYZPlaneColor(double& r, double& g, double& b)
{
    std::string colorStr = getState("grid_yz_plane_color").value<std::string>();
    std::vector<std::string> parts;
    boost::split(parts, colorStr, boost::is_any_of(","));
    if (parts.size() >= 3) {
        r = boost::lexical_cast<double>(parts[0]);
        g = boost::lexical_cast<double>(parts[1]);
        b = boost::lexical_cast<double>(parts[2]);
    } else {
        r = g = b = 0.5;
    }
}

void AppState::setGridYZPlaneColor(double r, double g, double b)
{
    std::string colorStr = 
        boost::lexical_cast<std::string>(r) + "," +
        boost::lexical_cast<std::string>(g) + "," +
        boost::lexical_cast<std::string>(b);
    getState("grid_yz_plane_color").value(colorStr);
}

void AppState::getGridXZPlaneColor(double& r, double& g, double& b)
{
    std::string colorStr = getState("grid_xz_plane_color").value<std::string>();
    std::vector<std::string> parts;
    boost::split(parts, colorStr, boost::is_any_of(","));
    if (parts.size() >= 3) {
        r = boost::lexical_cast<double>(parts[0]);
        g = boost::lexical_cast<double>(parts[1]);
        b = boost::lexical_cast<double>(parts[2]);
    } else {
        r = g = b = 0.5;
    }
}

void AppState::setGridXZPlaneColor(double r, double g, double b)
{
    std::string colorStr = 
        boost::lexical_cast<std::string>(r) + "," +
        boost::lexical_cast<std::string>(g) + "," +
        boost::lexical_cast<std::string>(b);
    getState("grid_xz_plane_color").value(colorStr);
}

void AppState::getGridXYPlaneColor(double& r, double& g, double& b)
{
    std::string colorStr = getState("grid_xy_plane_color").value<std::string>();
    std::vector<std::string> parts;
    boost::split(parts, colorStr, boost::is_any_of(","));
    if (parts.size() >= 3) {
        r = boost::lexical_cast<double>(parts[0]);
        g = boost::lexical_cast<double>(parts[1]);
        b = boost::lexical_cast<double>(parts[2]);
    } else {
        r = g = b = 0.5;
    }
}

void AppState::setGridXYPlaneColor(double r, double g, double b)
{
    std::string colorStr = 
        boost::lexical_cast<std::string>(r) + "," +
        boost::lexical_cast<std::string>(g) + "," +
        boost::lexical_cast<std::string>(b);
    getState("grid_xy_plane_color").value(colorStr);
}

void AppState::getGridTickLabelColor(double& r, double& g, double& b)
{
    std::string colorStr = getState("grid_tick_label_color").value<std::string>();
    std::vector<std::string> parts;
    boost::split(parts, colorStr, boost::is_any_of(","));
    if (parts.size() >= 3) {
        r = boost::lexical_cast<double>(parts[0]);
        g = boost::lexical_cast<double>(parts[1]);
        b = boost::lexical_cast<double>(parts[2]);
    } else {
        r = g = b = 1.0; // Default white
    }
}

void AppState::setGridTickLabelColor(double r, double g, double b)
{
    std::string colorStr = 
        boost::lexical_cast<std::string>(r) + "," +
        boost::lexical_cast<std::string>(g) + "," +
        boost::lexical_cast<std::string>(b);
    getState("grid_tick_label_color").value(colorStr);
}

int AppState::gridTickLabelFontSize()
{
    return getState("grid_tick_label_font_size").value<int>();
}

void AppState::setGridTickLabelFontSize(int size)
{
    getState("grid_tick_label_font_size").value(size);
}

boost::signals2::connection AppState::onGridColorChanged(const boost::function<void()>& callback)
{
    return getState("grid_color").valueChanged.connect(callback);
}

int AppState::cameraMode()
{
    return getState("camera_mode").value<int>();
}

void AppState::setCameraMode(int mode)
{
    getState("camera_mode").value(mode);
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
}

double AppState::cameraFieldOfView()
{
    return getState("camera_fov").value<double>();
}

void AppState::setCameraFieldOfView(double fov)
{
    getState("camera_fov").value(fov);
}

boost::signals2::connection AppState::onCameraChanged(const boost::function<void()>& callback)
{
    // Connect to all camera state changes
    auto conn = getState("camera_position_x").valueChanged.connect(callback);
    getState("camera_position_y").valueChanged.connect(callback);
    getState("camera_position_z").valueChanged.connect(callback);
    getState("camera_view_dir_x").valueChanged.connect(callback);
    getState("camera_view_dir_y").valueChanged.connect(callback);
    getState("camera_view_dir_z").valueChanged.connect(callback);
    getState("camera_up_x").valueChanged.connect(callback);
    getState("camera_up_y").valueChanged.connect(callback);
    getState("camera_up_z").valueChanged.connect(callback);
    getState("camera_fov").valueChanged.connect(callback);
    return conn;
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
}

boost::signals2::connection AppState::onTransferFunctionChanged(const boost::function<void()>& callback)
{
    auto conn = getState("transfer_function_color").valueChanged.connect(callback);
    getState("transfer_function_opacity").valueChanged.connect(callback);
    return conn;
}

boost::signals2::connection AppState::onGridPlaneVisibilityChanged(const boost::function<void()>& callback)
{
    // Connect to all three plane visibility states
    auto conn1 = getState("grid_yz_plane_visible").valueChanged.connect(callback);
    getState("grid_xz_plane_visible").valueChanged.connect(callback);
    getState("grid_xy_plane_visible").valueChanged.connect(callback);
    return conn1; // Return first connection (caller can disconnect individually if needed)
}

boost::signals2::connection AppState::onGridDivisionsChanged(const boost::function<void()>& callback)
{
    // Connect to all three division states
    auto conn1 = getState("grid_divisions_x").valueChanged.connect(callback);
    getState("grid_divisions_y").valueChanged.connect(callback);
    getState("grid_divisions_z").valueChanged.connect(callback);
    return conn1; // Return first connection
}

boost::signals2::connection AppState::onGridTickIntervalsChanged(const boost::function<void()>& callback)
{
    auto conn1 = getState("grid_tick_interval_x").valueChanged.connect(callback);
    getState("grid_tick_interval_y").valueChanged.connect(callback);
    getState("grid_tick_interval_z").valueChanged.connect(callback);
    return conn1;
}

boost::signals2::connection AppState::onGridTicksVisibleChanged(const boost::function<void()>& callback)
{
    return getState("grid_ticks_visible").valueChanged.connect(callback);
}

boost::signals2::connection AppState::onGridPlaneColorsChanged(const boost::function<void()>& callback)
{
    auto conn1 = getState("grid_yz_plane_color").valueChanged.connect(callback);
    getState("grid_xz_plane_color").valueChanged.connect(callback);
    getState("grid_xy_plane_color").valueChanged.connect(callback);
    return conn1;
}

boost::signals2::connection AppState::onGridTickLabelPropertiesChanged(const boost::function<void()>& callback)
{
    auto conn1 = getState("grid_tick_label_color").valueChanged.connect(callback);
    getState("grid_tick_label_font_size").valueChanged.connect(callback);
    return conn1;
}


// World BBox coordinate methods (no interval - displays at vertices only)
bool AppState::worldBBoxCoordinatesVisible()
{
    return getState("world_bbox_coordinates_visible").value<bool>();
}

void AppState::setWorldBBoxCoordinatesVisible(bool visible)
{
    getState("world_bbox_coordinates_visible").value(visible);
}

void AppState::getWorldBBoxCoordinateColor(double& r, double& g, double& b)
{
    std::string colorStr = getState("world_bbox_coordinate_color").value<std::string>();
    
    size_t pos1 = colorStr.find(',');
    size_t pos2 = colorStr.find(',', pos1 + 1);
    
    if (pos1 != std::string::npos && pos2 != std::string::npos) {
        r = boost::lexical_cast<double>(colorStr.substr(0, pos1));
        g = boost::lexical_cast<double>(colorStr.substr(pos1 + 1, pos2 - pos1 - 1));
        b = boost::lexical_cast<double>(colorStr.substr(pos2 + 1));
    } else {
        r = g = b = 1.0;
    }
}

void AppState::setWorldBBoxCoordinateColor(double r, double g, double b)
{
    std::string colorStr = 
        boost::lexical_cast<std::string>(r) + "," +
        boost::lexical_cast<std::string>(g) + "," +
        boost::lexical_cast<std::string>(b);
    getState("world_bbox_coordinate_color").value(colorStr);
}

int AppState::worldBBoxCoordinateFontSize()
{
    return getState("world_bbox_coordinate_font_size").value<int>();
}

void AppState::setWorldBBoxCoordinateFontSize(int size)
{
    getState("world_bbox_coordinate_font_size").value(size);
}

boost::signals2::connection AppState::onWorldBBoxCoordinatesChanged(const boost::function<void()>& callback)
{
    auto conn1 = getState("world_bbox_coordinates_visible").valueChanged.connect(callback);
    getState("world_bbox_coordinate_color").valueChanged.connect(callback);
    getState("world_bbox_coordinate_font_size").valueChanged.connect(callback);
    return conn1;
}
