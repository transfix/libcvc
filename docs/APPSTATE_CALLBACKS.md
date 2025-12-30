# AppState Callback System

## Overview

The `AppState` class provides a reactive state management system with callback notifications. All callback registration methods now return `boost::signals2::connection` objects that can be used to disconnect callbacks when they are no longer needed.

## Basic Usage

### Registering a Callback

```cpp
#include <volrover3/AppState.h>

// Get the singleton instance
AppState& state = AppState::instance();

// Register a callback that fires when camera changes
auto connection = state.onCameraChanged([]() {
    std::cout << "Camera changed!" << std::endl;
});

// Make a change that triggers the callback
state.setCameraPosition(1.0, 2.0, 3.0);
```

### Disconnecting a Callback

```cpp
// Disconnect when no longer needed
connection.disconnect();

// Further changes won't trigger the callback
state.setCameraPosition(4.0, 5.0, 6.0);  // No output
```

## Available Callbacks

All callback registration methods follow the same pattern: they accept a `boost::function<void()>` callback and return a `boost::signals2::connection`.

### Geometry and Volume

- `onGeometryChanged()` - Fires when geometry is updated
- `onVolumeChanged()` - Fires when volume data is updated
- `onWorldBoundsChanged()` - Fires when world bounding box changes

### Visibility States

- `onGridVisibilityChanged()` - Fires when grid visibility toggles
- `onAxisVisibilityChanged()` - Fires when axis visibility toggles
- `onGeometryBBoxVisibilityChanged()` - Fires when geometry bbox visibility changes
- `onVolumeBBoxVisibilityChanged()` - Fires when volume bbox visibility changes

### Camera

- `onCameraChanged()` - Fires when camera position, direction, up vector, or FOV changes
- `onCameraModeChanged()` - Fires when camera mode switches (orbit/fly)

### Transfer Function

- `onTransferFunctionChanged()` - Fires when transfer function is modified

## Object Lifecycle Management

The connection object follows RAII principles and should be stored as a member variable in classes that need to manage callback lifetime:

```cpp
class MyRenderer {
public:
    MyRenderer() {
        AppState& state = AppState::instance();
        
        // Register callbacks
        cameraConnection_ = state.onCameraChanged([this]() {
            updateCameraMatrices();
        });
        
        volumeConnection_ = state.onVolumeChanged([this]() {
            reloadVolumeData();
        });
    }
    
    ~MyRenderer() {
        // Connections are automatically disconnected when destroyed
        // But can also disconnect explicitly if needed
        cameraConnection_.disconnect();
        volumeConnection_.disconnect();
    }
    
private:
    boost::signals2::connection cameraConnection_;
    boost::signals2::connection volumeConnection_;
    
    void updateCameraMatrices() { /* ... */ }
    void reloadVolumeData() { /* ... */ }
};
```

## Multiple Callbacks

Multiple callbacks can be registered for the same state change:

```cpp
auto conn1 = state.onCameraChanged([]() {
    std::cout << "Callback 1" << std::endl;
});

auto conn2 = state.onCameraChanged([]() {
    std::cout << "Callback 2" << std::endl;
});

// Both callbacks will fire
state.setCameraPosition(1.0, 2.0, 3.0);

// Disconnect only one
conn1.disconnect();

// Only callback 2 will fire now
state.setCameraPosition(4.0, 5.0, 6.0);
```

## Accessing State Values in Callbacks

Callbacks can access the updated state values:

```cpp
auto connection = state.onCameraChanged([&state]() {
    double x, y, z;
    state.getCameraPosition(x, y, z);
    std::cout << "New position: " << x << ", " << y << ", " << z << std::endl;
});
```

## Thread Safety

The underlying `cvc::state` system is thread-safe. Callbacks are triggered synchronously on the thread that modifies the state.

## Implementation Notes

- Callbacks are implemented using Boost.Signals2
- State changes only trigger callbacks when values actually change (setting the same value twice won't fire the callback)
- The connection object can be safely copied; all copies refer to the same connection
- Disconnecting is idempotent - calling `disconnect()` multiple times is safe
