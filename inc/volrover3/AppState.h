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
    static AppState& instance();
    
    // State accessors with change notification
    cvc::geometry geometry();
    void setGeometry(const cvc::geometry& geom);
    
    cvc::volume volume();
    void setVolume(const cvc::volume& vol);
    
    cvc::bounding_box worldBounds();
    void setWorldBounds(const cvc::bounding_box& bounds);
    
    bool gridVisible();
    void setGridVisible(bool visible);
    
    bool axisVisible();
    void setAxisVisible(bool visible);
    
    bool geometryBBoxVisible();
    void setGeometryBBoxVisible(bool visible);
    
    bool volumeBBoxVisible();
    void setVolumeBBoxVisible(bool visible);
    
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
    boost::signals2::connection onGridVisibilityChanged(const boost::function<void()>& callback);
    boost::signals2::connection onAxisVisibilityChanged(const boost::function<void()>& callback);
    boost::signals2::connection onGeometryBBoxVisibilityChanged(const boost::function<void()>& callback);
    boost::signals2::connection onVolumeBBoxVisibilityChanged(const boost::function<void()>& callback);
    boost::signals2::connection onCameraModeChanged(const boost::function<void()>& callback);
    boost::signals2::connection onCameraChanged(const boost::function<void()>& callback);
    boost::signals2::connection onTransferFunctionChanged(const boost::function<void()>& callback);
    
private:
    AppState();
    ~AppState() = default;
    AppState(const AppState&) = delete;
    AppState& operator=(const AppState&) = delete;
    
    cvc::state& getState(const std::string& path);
};

#endif // APPSTATE_H
