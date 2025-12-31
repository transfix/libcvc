#ifndef APPSTATE_H
#define APPSTATE_H

#include <cvc/state.h>
#include <cvc/geometry.h>
#include <cvc/volume.h>
#include <cvc/bounding_box.h>
#include <boost/signals2/connection.hpp>
#include <memory>

// Application state manager using cvc::state for reactive updates
class AppState
{
public:
    // Get default singleton instance (uses "volrover3" prefix)
    static AppState& instance();
    
    // Create instance with custom state prefix (for multiple viewers or testing)
    explicit AppState(const std::string& statePrefix = "volrover3");
    
    // Get the state prefix for this instance
    std::string getStatePrefix() const { return m_statePrefix; }
    
    // State accessors with change notification
    cvc::geometry geometry();
    void setGeometry(const cvc::geometry& geom);
    
    cvc::volume volume();
    void setVolume(const cvc::volume& vol);
    
    cvc::bounding_box worldBounds();
    void setWorldBounds(const cvc::bounding_box& bounds);
    
    // Compute bounding box that contains all graphics objects with their transforms applied
    cvc::bounding_box computeGraphicsBounds();
    
    // World bounding box visibility
    bool worldBBoxVisible();
    void setWorldBBoxVisible(bool visible);
    
    bool gridVisible();
    void setGridVisible(bool visible);
    
    bool axisVisible();
    void setAxisVisible(bool visible);
    
    bool geometryBBoxVisible();
    void setGeometryBBoxVisible(bool visible);
    
    bool volumeBBoxVisible();
    void setVolumeBBoxVisible(bool visible);
    
    // Grid plane visibility (individual planes)
    bool gridYZPlaneVisible();
    void setGridYZPlaneVisible(bool visible);
    
    bool gridXZPlaneVisible();
    void setGridXZPlaneVisible(bool visible);
    
    bool gridXYPlaneVisible();
    void setGridXYPlaneVisible(bool visible);
    
    // Grid divisions per axis
    void getGridDivisions(int& x, int& y, int& z);
    void setGridDivisions(int x, int y, int z);
    
    // Grid tick intervals
    void getGridTickIntervals(int& x, int& y, int& z);
    void setGridTickIntervals(int x, int y, int z);
    
    // Grid ticks visibility
    bool gridTicksVisible();
    void setGridTicksVisible(bool visible);
    
    // Grid and bbox colors (RGB triplets)
    void getGridColor(double& r, double& g, double& b);
    void setGridColor(double r, double g, double b);
    
    // Per-plane grid colors
    void getGridYZPlaneColor(double& r, double& g, double& b);
    void setGridYZPlaneColor(double r, double g, double b);
    
    void getGridXZPlaneColor(double& r, double& g, double& b);
    void setGridXZPlaneColor(double r, double g, double b);
    
    void getGridXYPlaneColor(double& r, double& g, double& b);
    void setGridXYPlaneColor(double r, double g, double b);
    
    // Grid tick label properties
    void getGridTickLabelColor(double& r, double& g, double& b);
    void setGridTickLabelColor(double r, double g, double b);
    
    int gridTickLabelFontSize();
    void setGridTickLabelFontSize(int size);
    
    void getGeometryBBoxColor(double& r, double& g, double& b);
    void setGeometryBBoxColor(double r, double g, double b);
    
    void getVolumeBBoxColor(double& r, double& g, double& b);
    void setVolumeBBoxColor(double r, double g, double b);
    
    // Volume BBox tick controls
    bool volumeBBoxTicksVisible();
    void setVolumeBBoxTicksVisible(bool visible);
    
    double volumeBBoxTickInterval();
    void setVolumeBBoxTickInterval(double interval);
    
    void getVolumeBBoxTickLabelColor(double& r, double& g, double& b);
    void setVolumeBBoxTickLabelColor(double r, double g, double b);
    
    int volumeBBoxTickLabelFontSize();
    void setVolumeBBoxTickLabelFontSize(int size);
    
    // World BBox coordinate controls (no interval - shows at vertices only)
    bool worldBBoxCoordinatesVisible();
    void setWorldBBoxCoordinatesVisible(bool visible);
    
    void getWorldBBoxCoordinateColor(double& r, double& g, double& b);
    void setWorldBBoxCoordinateColor(double r, double g, double b);
    
    int worldBBoxCoordinateFontSize();
    void setWorldBBoxCoordinateFontSize(int size);
    
    // Camera control mode (0 = orbit, 1 = fly)
    int cameraMode();
    void setCameraMode(int mode);
    
    // Camera settings
    double cameraSpeed();
    void setCameraSpeed(double speed);
    
    double cameraSensitivity();
    void setCameraSensitivity(double sensitivity);
    
    bool cameraInvertMouse();
    void setCameraInvertMouse(bool invert);
    
    // Camera key bindings
    int cameraKeyForward();
    void setCameraKeyForward(int key);
    
    int cameraKeyBackward();
    void setCameraKeyBackward(int key);
    
    int cameraKeyLeft();
    void setCameraKeyLeft(int key);
    
    int cameraKeyRight();
    void setCameraKeyRight(int key);
    
    int cameraKeyUp();
    void setCameraKeyUp(int key);
    
    int cameraKeyDown();
    void setCameraKeyDown(int key);
    
    // Camera position and orientation
    void getCameraPosition(double& x, double& y, double& z);
    void setCameraPosition(double x, double y, double z);
    
    void getCameraViewDirection(double& x, double& y, double& z);
    void setCameraViewDirection(double x, double y, double z);
    
    void getCameraUpVector(double& x, double& y, double& z);
    void setCameraUpVector(double x, double y, double z);
    
    double cameraFieldOfView();
    void setCameraFieldOfView(double fov);
    
    // Transfer function (color and opacity tables)
    std::vector<double> transferFunctionColorTable();
    void setTransferFunctionColorTable(const std::vector<double>& table);
    
    std::vector<double> transferFunctionOpacityTable();
    void setTransferFunctionOpacityTable(const std::vector<double>& table);
    
    // Get shared pointers to data (for direct access)
    std::shared_ptr<cvc::geometry> geometryPtr();
    std::shared_ptr<cvc::volume> volumePtr();
    
    // Register callbacks for state changes
    // Returns a connection object that can be used to disconnect the callback
    boost::signals2::connection onGeometryChanged(const boost::function<void()>& callback);
    boost::signals2::connection onVolumeChanged(const boost::function<void()>& callback);
    boost::signals2::connection onWorldBoundsChanged(const boost::function<void()>& callback);
    boost::signals2::connection onWorldBBoxVisibilityChanged(const boost::function<void()>& callback);
    boost::signals2::connection onGridVisibilityChanged(const boost::function<void()>& callback);
    boost::signals2::connection onAxisVisibilityChanged(const boost::function<void()>& callback);
    boost::signals2::connection onGeometryBBoxVisibilityChanged(const boost::function<void()>& callback);
    boost::signals2::connection onVolumeBBoxVisibilityChanged(const boost::function<void()>& callback);
    boost::signals2::connection onGridColorChanged(const boost::function<void()>& callback);
    boost::signals2::connection onGeometryBBoxColorChanged(const boost::function<void()>& callback);
    boost::signals2::connection onVolumeBBoxColorChanged(const boost::function<void()>& callback);
    boost::signals2::connection onVolumeBBoxTicksChanged(const boost::function<void()>& callback);
    boost::signals2::connection onWorldBBoxCoordinatesChanged(const boost::function<void()>& callback);
    boost::signals2::connection onGridPlaneVisibilityChanged(const boost::function<void()>& callback);
    boost::signals2::connection onGridDivisionsChanged(const boost::function<void()>& callback);
    boost::signals2::connection onGridTickIntervalsChanged(const boost::function<void()>& callback);
    boost::signals2::connection onGridTicksVisibleChanged(const boost::function<void()>& callback);
    boost::signals2::connection onGridPlaneColorsChanged(const boost::function<void()>& callback);
    boost::signals2::connection onGridTickLabelPropertiesChanged(const boost::function<void()>& callback);
    boost::signals2::connection onCameraModeChanged(const boost::function<void()>& callback);
    boost::signals2::connection onCameraChanged(const boost::function<void()>& callback);
    boost::signals2::connection onTransferFunctionChanged(const boost::function<void()>& callback);
    
    // State tree access for debugging/inspection
    cvc::state& getRootState();
    
public:
    ~AppState() = default;
    
private:
    AppState(const AppState&) = delete;
    AppState& operator=(const AppState&) = delete;
    
    cvc::state& getState(const std::string& path);
    void initializeDefaults();
    
    std::string m_statePrefix;
};

#endif // APPSTATE_H
