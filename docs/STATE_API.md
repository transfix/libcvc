# State API Reference

## Table of Contents

- [Overview](#overview)
- [Core Concepts](#core-concepts)
- [State Tree Structure](#state-tree-structure)
- [Basic Operations](#basic-operations)
  - [Creating and Accessing State Nodes](#creating-and-accessing-state-nodes)
  - [Value Operations](#value-operations)
  - [Data Operations](#data-operations)
  - [Property Management](#property-management)
- [Advanced Features](#advanced-features)
  - [Futures API](#futures-api)
  - [Signals and Callbacks](#signals-and-callbacks)
  - [Thread Safety](#thread-safety)
  - [State Serialization](#state-serialization)
  - [State Object Pattern](#state-object-pattern)
- [Complete API Reference](#complete-api-reference)
  - [Construction and Access](#construction-and-access)
  - [Value Methods](#value-methods)
  - [Data Methods](#data-methods)
  - [Property Methods](#property-methods)
  - [Tree Navigation](#tree-navigation)
  - [Futures Methods](#futures-methods)
  - [Serialization Methods](#serialization-methods)
- [Usage Examples](#usage-examples)
- [Exception Handling](#exception-handling)
- [Performance Considerations](#performance-considerations)
- [Testing](#testing)

## Overview

The `cvc::state` class provides a thread-safe, hierarchical key-value store with signal/slot notifications and futures-based async operations. It serves as the backbone for application state management in trans-cvc, supporting:

- **Hierarchical organization**: Dot-notation paths create tree structures
- **Type-safe storage**: Template methods with automatic conversion
- **Async notifications**: Signal/slot mechanism for value changes
- **Futures API**: Producer-consumer patterns and blocking waits
- **Property system**: Metadata and attributes per state node
- **Thread safety**: All operations protected by mutex locks
- **Serialization**: JSON import/export for persistence

## Core Concepts

### State Tree

State nodes are organized in a hierarchical tree structure using dot-notation paths:

```
root
├── app
│   ├── window
│   │   ├── width (value: 1920)
│   │   └── height (value: 1080)
│   └── theme (value: "dark")
└── data
    ├── volume.filename (value: "scan.rawiv")
    └── geometry.mesh (data: geometry object)
```

### Values vs Data

- **Values**: Convertible types (int, double, string) - stored internally as strings
- **Data**: Arbitrary C++ objects stored as `boost::any` - no automatic conversion

### Signals

State nodes emit signals when values/data change, enabling reactive programming:

```cpp
cvcstate("sensor.temperature").valueChanged.connect([](std::string val) {
    std::cout << "Temperature: " << val << std::endl;
});
```

## State Tree Structure

### Path Syntax

```cpp
// Dot notation creates parent nodes automatically
cvcstate("level1.level2.level3.key");

// Equivalent to:
//   cvcstate("level1")
//     .child("level2")
//     .child("level3")
//     .child("key")
```

### Automatic Node Creation

```cpp
// Setting a deep path creates all intermediate nodes
cvcstate("new.path.to.value").value(42);

// Creates hierarchy:
//   root -> new -> path -> to -> value
```

## Basic Operations

### Creating and Accessing State Nodes

```cpp
#include <cvc/state.h>

// Global convenience function creates/accesses nodes
state s1 = cvcstate("app.window.width");

// Direct construction
state s2("app.window.height");

// Access via parent
state parent = cvcstate("app.window");
state child = parent.child("title");
```

### Value Operations

```cpp
// Set value (converted to string internally)
cvcstate("count").value(42);
cvcstate("ratio").value(3.14159);
cvcstate("name").value(std::string("trans-cvc"));

// Get value with type conversion
int count = cvcstate("count").value<int>();
double ratio = cvcstate("ratio").value<double>();
std::string name = cvcstate("name").value<std::string>();

// Check if value exists
if (cvcstate("optional.key").initialized()) {
    int val = cvcstate("optional.key").value<int>();
}

// Get raw string value
std::string raw = cvcstate("count").value();  // "42"
```

### Data Operations

```cpp
// Store arbitrary C++ objects
struct Config {
    int width, height;
    std::string theme;
};

Config cfg{1920, 1080, "dark"};
cvcstate("app.config").data(cfg);

// Retrieve with exact type match
try {
    Config loaded = cvcstate("app.config").data<Config>();
} catch (const cvc::type_conversion_error& e) {
    std::cerr << "Type mismatch: " << e.what() << std::endl;
}

// Check if data exists
if (cvcstate("app.config").has_data()) {
    Config cfg = cvcstate("app.config").data<Config>();
}
```

### Property Management

```cpp
// Set property (metadata for state node)
cvcstate("value").property("units", "meters");
cvcstate("value").property("min", "0.0");
cvcstate("value").property("max", "100.0");

// Get property
std::string units = cvcstate("value").property("units");

// Check if property exists
if (cvcstate("value").has_property("units")) {
    // Property exists
}

// Get all property keys
std::vector<std::string> keys = cvcstate("value").property_keys();
```

## Advanced Features

### Futures API

The futures API enables async/await-style programming with state values, providing blocking waits, timeout support, callbacks, and future objects for advanced control. This enables producer-consumer patterns and async message passing through the state tree.

**Key Features:**
- **Blocking waits**: Wait indefinitely for a value to be set
- **Timeout support**: Wait with configurable timeout
- **Callbacks**: Register callbacks that fire when values change
- **Type-safe**: Fully templated with type conversion
- **Thread-safe**: All operations use proper synchronization

#### Blocking Waits

```cpp
// Wait indefinitely for a value to be set
int result = cvcstate("computation.result").wait_for_value<int>();

// Wait with timeout (throws timeout_error on timeout)
double value = cvcstate("sensor.reading")
    .wait_for_value<double>(boost::chrono::seconds(5));

// Wait for data object
MyStruct data = cvcstate("queue.item").wait_for_data<MyStruct>();
```

#### Value Callbacks

```cpp
// Register callback that fires when value changes
int current = cvcstate("counter").value<int>([](int newValue) {
    std::cout << "Counter changed to: " << newValue << std::endl;
});

// Callback fires asynchronously whenever counter is updated
cvcstate("counter").value(current + 1);  // Triggers callback
```

#### Future Objects

The `value_future<T>()` method returns a `state_future<T>` object for advanced control:

```cpp
// Get a future object for advanced control
auto future = cvcstate("async.result").value_future<std::string>();

// Non-blocking check
if (future.is_ready()) {
    std::string val = future.get();
}

// Wait with timeout
if (future.wait_for(boost::chrono::milliseconds(500))) {
    std::string val = future.get();
} else {
    std::cout << "Still waiting..." << std::endl;
}

// Get with timeout (throws on timeout)
std::string val = future.get_for(boost::chrono::seconds(10));

// Block indefinitely
std::string val = future.get();
```

#### Producer-Consumer Pattern

```cpp
// Consumer thread waits for producer
boost::thread consumer([]() {
    // Blocks until producer sets the value
    int result = cvcstate("work.result").wait_for_value<int>();
    std::cout << "Got result: " << result << std::endl;
});

// Producer thread does work and sets result
boost::thread producer([]() {
    int result = do_expensive_computation();
    cvcstate("work.result").value(result);
});

consumer.join();
producer.join();
```

#### Multiple Consumers

```cpp
// Multiple threads can wait on the same state value
std::vector<boost::thread> consumers;

for (int i = 0; i < 5; ++i) {
    consumers.emplace_back([i]() {
        // All threads will be notified when value is set
        int value = cvcstate("broadcast.value").wait_for_value<int>();
        std::cout << "Consumer " << i << " got: " << value << std::endl;
    });
}

// Producer sets value once - all consumers wake up
boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
cvcstate("broadcast.value").value(42);

for (auto& t : consumers) {
    t.join();
}
```

#### Comparison with Traditional Approaches

**Before (inefficient polling):**
```cpp
// Inefficient busy-wait polling
while (!cvcstate("key").initialized()) {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
}
int val = cvcstate("key").value<int>();
```

**After (efficient futures):**
```cpp
// Efficient blocking wait with condition variables
int val = cvcstate("key").wait_for_value<int>();
```

#### Futures Implementation Details

**Synchronization:**
- Uses `boost::condition_variable` for blocking waits
- Uses `boost::signals2` for callbacks
- All operations are protected by mutex locks
- Supports multiple waiters on the same state node

**Exception Safety:**
- Timeout operations throw `cvc::timeout_error` with descriptive messages
- `timeout_error` inherits from `cvc::exception` (which inherits from `boost::exception`)
- Type conversion errors in callbacks are silently caught
- Move semantics for `state_future` prevent accidental copies

**Performance:**
- Callbacks execute in the thread that modifies the state
- Futures wake all waiting threads when value changes
- Minimal overhead when features are not used
- Condition variables efficiently wait without busy-spinning

### Signals and Callbacks

State nodes emit signals when values/data/properties change:

```cpp
// Connect to value change signal
cvcstate("config.theme").valueChanged.connect([](std::string newTheme) {
    applyTheme(newTheme);
});

// Connect to data change signal
cvcstate("mesh.data").dataChanged.connect([]() {
    // Data changed - reload mesh
    cvc::geometry mesh = cvcstate("mesh.data").data<cvc::geometry>();
    renderMesh(mesh);
});

// Connect to property change signal
cvcstate("slider").propertyChanged.connect([](std::string key, std::string value) {
    std::cout << "Property '" << key << "' = " << value << std::endl;
});

// Signals use boost::signals2 - thread-safe and multi-subscriber
```

### Thread Safety

All `cvc::state` operations are thread-safe via mutex protection:

```cpp
// Multiple threads can safely access the same state
boost::thread t1([](){ cvcstate("shared").value(1); });
boost::thread t2([](){ cvcstate("shared").value(2); });
boost::thread t3([](){ int v = cvcstate("shared").value<int>(); });

t1.join(); t2.join(); t3.join();

// Futures enable producer-consumer patterns
boost::thread producer([](){ 
    cvcstate("result").value(compute()); 
});
boost::thread consumer([](){ 
    int result = cvcstate("result").wait_for_value<int>(); 
});
```

### State Serialization

```cpp
// Export state tree to JSON string
std::string json = cvcstate.dump();

// Import from JSON (merges with existing state)
cvcstate.read(json_string);

// Serialization preserves:
// - Value data (as strings)
// - Properties (key-value pairs)
// - Tree hierarchy

// Note: Data objects (boost::any) are NOT serialized
```

### State Object Pattern

The `cvc::state_object<T>` template class provides a convenient base class for objects that want to store their member data in the state tree. This enables automatic state monitoring, easy persistence, and external control of object state.

**Key Features:**
- **Automatic state registration**: Each instance gets a unique state path based on class name and memory address
- **Change notifications**: `handleStateChanged()` callback fires when any child state changes
- **Thread-safe updates**: State changes trigger async handlers in separate threads
- **Easy access**: Convenient `getState()` and `stateName()` helper methods
- **Type registration**: Automatically registers the class type with `cvcapp`

#### Basic Usage

```cpp
#include <cvc/state_object.h>

// Inherit from state_object<YourClass>
class Configuration : public state_object<Configuration> {
public:
    Configuration() {
        // Initialize default values
        getState("width").value(1920);
        getState("height").value(1080);
        getState("fullscreen").value(false);
    }
    
protected:
    // Override to handle state changes
    virtual void handleStateChanged(const std::string& childState) override {
        if (childState == "width" || childState == "height") {
            resizeWindow(
                getState("width").value<int>(),
                getState("height").value<int>()
            );
        } else if (childState == "fullscreen") {
            setFullscreen(getState("fullscreen").value<bool>());
        }
    }
    
private:
    void resizeWindow(int w, int h) { /* ... */ }
    void setFullscreen(bool fs) { /* ... */ }
};
```

#### External State Access

Other parts of your application can access and modify object state using the generated state path:

```cpp
Configuration* config = new Configuration();

// State path format: <TypeName>/<InstanceAddress>/<ChildState>
// Example: "Configuration/0x7f8a1c000a10/width"

// Access via the object
config->getState("width").value(2560);  // Triggers handleStateChanged()

// Or access globally if you know the path
std::string path = config->stateName("width");
cvcstate(path).value(2560);  // Same effect
```

#### Monitoring Object State

The state tree integration makes it easy to monitor and log state changes:

```cpp
class DataProcessor : public state_object<DataProcessor> {
public:
    DataProcessor() {
        // Set up properties
        getState("status").value("idle");
        getState("progress").value(0.0);
        getState("error_count").value(0);
    }
    
    void processData(const std::vector<double>& data) {
        getState("status").value("processing");
        
        for (size_t i = 0; i < data.size(); ++i) {
            try {
                processItem(data[i]);
                getState("progress").value(
                    static_cast<double>(i) / data.size()
                );
            } catch (const std::exception& e) {
                int count = getState("error_count").value<int>();
                getState("error_count").value(count + 1);
            }
        }
        
        getState("status").value("complete");
        getState("progress").value(1.0);
    }
    
protected:
    virtual void handleStateChanged(const std::string& childState) override {
        // Log all state changes
        cvcapp.log(2, str(boost::format("DataProcessor: %s = %s") 
            % childState 
            % getState(childState).value()));
            
        // React to specific changes
        if (childState == "error_count") {
            int errors = getState("error_count").value<int>();
            if (errors > 10) {
                cvcapp.log(0, "Too many errors, halting processing");
                // Take corrective action
            }
        }
    }
    
private:
    void processItem(double value) { /* ... */ }
};
```

#### Multi-threaded State Updates

State changes automatically spawn threads for `handleStateChanged()`, making the pattern safe for concurrent updates:

```cpp
class Renderer : public state_object<Renderer> {
public:
    Renderer() {
        getState("camera.position").value("0,0,10");
        getState("camera.target").value("0,0,0");
        getState("render_mode").value("solid");
    }
    
protected:
    virtual void handleStateChanged(const std::string& childState) override {
        // This runs in a separate thread for each state change
        // Safe to perform expensive operations here
        
        if (childState.find("camera.") == 0) {
            updateCameraFromState();
            requestRedraw();
        } else if (childState == "render_mode") {
            std::string mode = getState("render_mode").value();
            setRenderMode(mode);
            requestRedraw();
        }
    }
    
private:
    void updateCameraFromState() {
        // Parse and update camera
        std::string pos = getState("camera.position").value();
        std::string target = getState("camera.target").value();
        // ... apply to camera
    }
    
    void setRenderMode(const std::string& mode) { /* ... */ }
    void requestRedraw() { /* ... */ }
};

// Multiple threads can safely update the renderer's state
boost::thread t1([&renderer]() {
    renderer.getState("camera.position").value("5,5,5");
});

boost::thread t2([&renderer]() {
    renderer.getState("render_mode").value("wireframe");
});

t1.join();
t2.join();
// Both state changes trigger handleStateChanged() in separate threads
```

#### Persistence and Serialization

Since state_object integrates with the state tree, persistence is automatic:

```cpp
class AppSettings : public state_object<AppSettings> {
public:
    void loadDefaults() {
        getState("window.width").value(1920);
        getState("window.height").value(1080);
        getState("theme").value("dark");
        getState("language").value("en");
    }
    
    void save(const std::string& filename) {
        // Export this object's state subtree to JSON
        std::string json = getState().dump();
        std::ofstream file(filename);
        file << json;
    }
    
    void load(const std::string& filename) {
        std::ifstream file(filename);
        std::string json((std::istreambuf_iterator<char>(file)),
                         std::istreambuf_iterator<char>());
        getState().read(json);
        // handleStateChanged() will be called for each restored value
    }
    
protected:
    virtual void handleStateChanged(const std::string& childState) override {
        // React to loaded settings
        if (childState == "theme") {
            applyTheme(getState("theme").value());
        }
    }
};
```

#### State Object API

The `state_object<T>` template provides these methods:

**Constructor:**
- Registers the type with `cvcapp`
- Sets up automatic state change monitoring
- Each instance gets a unique state path: `<TypeName>/<InstanceAddress>`

**Protected Methods:**
```cpp
// Override to handle state changes
virtual void handleStateChanged(const std::string& childState);
```

**Public Methods:**
```cpp
// Get state path for this instance (with optional child)
std::string stateName(const std::string& childState = std::string()) const;

// Access state tree for this instance
state& getState(const std::string& s = std::string()) const;
```

**Destructor:**
- Disconnects state change monitoring
- State tree data persists until explicitly cleared

#### Implementation Notes

**Thread Safety:**
- Each `handleStateChanged()` call runs in its own thread
- Thread name format: `<statePath>_stateChanged`
- Non-blocking: state modifications return immediately
- Multiple simultaneous state changes spawn multiple handler threads

**State Path Format:**
```
<TypeName>/<InstanceAddress>/<ChildState>
Example: Configuration/0x7f8a1c000a10/window.width
```

**Performance Considerations:**
- Keep `handleStateChanged()` implementations lightweight
- Heavy operations are fine (they run in separate threads)
- Avoid modifying the same state from within `handleStateChanged()` (potential recursion)

**Best Practices:**
1. Initialize default state values in constructor
2. Use `handleStateChanged()` to react to external modifications
3. Use child state paths for logical grouping (e.g., "window.width", "window.height")
4. Add logging in `handleStateChanged()` for debugging
5. Use state properties for metadata (units, ranges, descriptions)

## Complete API Reference

### Construction and Access

```cpp
// Global access function
state cvcstate(const std::string& name);

// Constructor
state(const std::string& name = std::string());

// Copy semantics (shallow copy - same node)
state(const state& s);
state& operator=(const state& s);
```

### Value Methods

```cpp
// Get raw string value
std::string value() const;

// Get typed value
template<class T>
T value() const;

// Get value with callback
template<class T>
T value(const boost::function<void(T)>& callback);

// Set value
template<class T>
void value(const T& val);

// Wait for value (futures API)
template<class T>
T wait_for_value(boost::chrono::milliseconds timeout = boost::chrono::milliseconds(0));

// Get value future
template<class T>
state_future<T> value_future();

// Check if initialized
bool initialized() const;
```

### Data Methods

```cpp
// Get data (internal boost::any)
boost::any data() const;

// Get typed data (throws type_conversion_error on mismatch)
template<class T>
T data();

// Get data with callback
template<class T>
T data(const boost::function<void(T)>& callback);

// Set data
void data(const boost::any& d);

// Wait for data (futures API)
template<class T>
T wait_for_data(boost::chrono::milliseconds timeout = boost::chrono::milliseconds(0));

// Check if data exists
bool has_data() const;
```

### Property Methods

```cpp
// Get property value
std::string property(const std::string& key) const;

// Set property
void property(const std::string& key, const std::string& val);

// Check if property exists
bool has_property(const std::string& key) const;

// Get all property keys
std::vector<std::string> property_keys() const;
```

### Tree Navigation

```cpp
// Get child node
state child(const std::string& name);

// Get parent node
state parent();

// Get node name
std::string name() const;

// Get full path
std::string path() const;

// Get all children
std::vector<state> children();
```

### Futures Methods

The futures API provides async/await-style functionality:

- `wait_for_value<T>(timeout)` - Block until value is set (with optional timeout)
- `wait_for_data<T>(timeout)` - Block until data is set (with optional timeout)
- `value_future<T>()` - Get future object for advanced control with `is_ready()`, `wait_for()`, `get()`, `get_for()`
- `value<T>(callback)` - Get value and register callback for changes
- `data<T>(callback)` - Get data and register callback for changes

See the [Futures API](#futures-api) section above for detailed examples and patterns.

### Serialization Methods

```cpp
// Export to JSON string
std::string dump() const;

// Import from JSON string
void read(const std::string& json);
```

## Usage Examples

### Configuration Management

```cpp
void loadConfig() {
    // Set default configuration
    cvcstate("app.window.width").value(1920);
    cvcstate("app.window.height").value(1080);
    cvcstate("app.theme").value("dark");
    
    // Add metadata
    cvcstate("app.window.width").property("min", "640");
    cvcstate("app.window.width").property("max", "3840");
    
    // Monitor changes
    cvcstate("app.theme").valueChanged.connect([](std::string theme) {
        applyTheme(theme);
    });
}

void saveConfig() {
    std::string json = cvcstate("app").dump();
    writeFile("config.json", json);
}

void restoreConfig() {
    std::string json = readFile("config.json");
    cvcstate("app").read(json);
}
```

### Data Pipeline

```cpp
// Stage 1: Load data
void loadStage() {
    cvc::volume vol("input.rawiv");
    cvcstate("pipeline.input").data(vol);
}

// Stage 2: Process (waits for stage 1)
void processStage() {
    cvc::volume input = cvcstate("pipeline.input")
        .wait_for_data<cvc::volume>(boost::chrono::seconds(30));
    
    input.bilateral_filter(5.0, 0.1);
    cvcstate("pipeline.filtered").data(input);
}

// Stage 3: Output (waits for stage 2)
void outputStage() {
    cvc::volume result = cvcstate("pipeline.filtered")
        .wait_for_data<cvc::volume>(boost::chrono::seconds(30));
    
    result.write("output.rawiv");
    cvcstate("pipeline.complete").value(true);
}
```

### Real-time Monitoring

```cpp
class TemperatureMonitor {
    std::atomic<bool> running_{true};
    boost::thread monitor_thread_;
    
public:
    TemperatureMonitor() {
        // Set up monitoring with callback
        cvcstate("sensor.temperature").value<double>([this](double temp) {
            if (temp > 100.0) {
                std::cerr << "ALERT: High temperature: " << temp << std::endl;
                cvcstate("alarm.triggered").value(true);
            }
        });
        
        // Monitoring thread
        monitor_thread_ = boost::thread([this]() {
            while (running_) {
                try {
                    // Wait for temperature reading
                    double temp = cvcstate("sensor.reading")
                        .wait_for_value<double>(boost::chrono::seconds(1));
                    
                    // Update display
                    cvcstate("sensor.temperature").value(temp);
                    
                } catch (const cvc::timeout_error&) {
                    // No reading yet, keep waiting
                }
            }
        });
    }
    
    ~TemperatureMonitor() {
        running_ = false;
        monitor_thread_.join();
    }
};
```

### Request-Response Pattern

```cpp
class AsyncServer {
    std::atomic<bool> running_{true};
    boost::thread server_thread_;
    
public:
    AsyncServer() {
        server_thread_ = boost::thread([this]() {
            while (running_) {
                try {
                    // Wait for request
                    std::string request = cvcstate("server.request")
                        .wait_for_value<std::string>(boost::chrono::seconds(1));
                    
                    // Process and respond
                    std::string response = processRequest(request);
                    cvcstate("server.response").value(response);
                    
                } catch (const cvc::timeout_error&) {
                    // No request, keep waiting
                }
            }
        });
    }
    
    std::string makeRequest(const std::string& req) {
        cvcstate("server.request").value(req);
        return cvcstate("server.response")
            .wait_for_value<std::string>(boost::chrono::seconds(10));
    }
};
```

## Exception Handling

### Type Conversion Errors

```cpp
// Setting an int value
cvcstate("number").value(42);

// Attempting wrong type conversion throws type_conversion_error
try {
    // This will throw because "42" cannot be cast to Config
    Config cfg = cvcstate("number").data<Config>();
} catch (const cvc::type_conversion_error& e) {
    std::cerr << "Conversion failed: " << e.what() << std::endl;
    // Output: "Conversion failed: cvc::type_conversion_error exception: 
    //          Failed to cast data to requested type: ..."
}
```

### Timeout Errors

```cpp
// Wait with timeout
try {
    int value = cvcstate("slow.computation")
        .wait_for_value<int>(boost::chrono::milliseconds(100));
} catch (const cvc::timeout_error& e) {
    std::cerr << "Timeout: " << e.what() << std::endl;
    // Output: "Timeout: cvc::timeout_error exception: 
    //          Timeout waiting for value at path 'slow.computation'"
}
```

### Exception Hierarchy

```cpp
cvc::exception (inherits from boost::exception)
├── cvc::type_conversion_error   // Data/value type mismatches
├── cvc::timeout_error           // Futures timeout exceeded
├── cvc::read_error              // File I/O errors
└── ... (other CVC exceptions)
```

## Performance Considerations

### Value vs Data

- **Values**: Stored as strings, involve conversion overhead
- **Data**: Stored as `boost::any`, zero conversion for exact type matches
- Use **values** for simple types (int, double, string)
- Use **data** for complex objects (geometry, volumes, custom structs)

### Callback Performance

- Callbacks execute in the thread that modifies the state
- Keep callbacks short and non-blocking
- For expensive operations, trigger async work instead:

```cpp
cvcstate("trigger").valueChanged.connect([](std::string val) {
    // Launch async processing instead of blocking
    boost::thread([val]() {
        expensiveOperation(val);
    }).detach();
});
```

### Locking Strategy

- All operations use mutex locks (no lock-free operations)
- Futures use condition variables for efficient blocking
- Avoid holding state references across long operations

### Memory Considerations

- State nodes use `boost::shared_ptr` - automatic cleanup
- Data objects stored in `boost::any` - copy semantics
- Large objects should use shared_ptr internally:

```cpp
// Efficient: Shares data via shared_ptr
struct VolumeRef {
    boost::shared_ptr<cvc::volume> vol;
};
cvcstate("volume").data(VolumeRef{vol_ptr});
```

## Testing

The state API includes comprehensive test coverage:

### Test Suites

- **128 State Tests** (100% passing)
  - Basic value get/set operations
  - Data get/set with type conversion
  - Property management
  - Tree navigation and hierarchy
  - Signals and callbacks
  - Futures API (11 dedicated tests)
  - Thread safety and concurrent access
  - Serialization (JSON dump/read)

### Coverage Metrics

From `TESTING_COVERAGE.md`:
- **state.cpp**: 92.75% line coverage (243/262 lines)
- **state.h**: 94.7% function coverage (355/375 functions)

### Key Test Scenarios

```cpp
// Thread safety test
TEST(StateTest, ConcurrentAccess) {
    std::vector<boost::thread> threads;
    for (int i = 0; i < 10; i++) {
        threads.emplace_back([i]() {
            cvcstate("concurrent.value").value(i);
            int val = cvcstate("concurrent.value").value<int>();
        });
    }
    for (auto& t : threads) t.join();
}

// Futures test
TEST(StateTest, FuturesBlocking) {
    boost::thread producer([]() {
        boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
        cvcstate("future.value").value(42);
    });
    
    int result = cvcstate("future.value").wait_for_value<int>();
    EXPECT_EQ(42, result);
    producer.join();
}

// Type conversion error test
TEST(StateTest, TypeConversionError) {
    cvcstate("test").value(42);
    EXPECT_THROW(
        cvcstate("test").data<std::vector<int>>(),
        cvc::type_conversion_error
    );
}
```

### Running Tests

```bash
# Run all state tests
./build/bin/state_test

# Run specific test
./build/bin/state_test --gtest_filter=StateTest.TypeConversionError

# Run futures tests
./build/bin/state_test --gtest_filter=*Futures*
```

## See Also

- **[../README.md](../README.md)** - Project overview and quick start
- **[TESTING_COVERAGE.md](TESTING_COVERAGE.md)** - Test coverage metrics
- **[PROJECT_REPORT.md](PROJECT_REPORT.md)** - Complete project documentation
