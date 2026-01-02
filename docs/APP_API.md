# Application Framework API (cvc::app)

*Complete reference for the trans-cvc application singleton*

## Table of Contents

- [Overview](#overview)
- [Quick Start](#quick-start)
- [Singleton Access](#singleton-access)
- [Data Management](#data-management)
  - [Basic Data Operations](#basic-data-operations)
  - [Type-Safe Data Access](#type-safe-data-access)
  - [Querying Data by Type](#querying-data-by-type)
  - [Batch Data Operations](#batch-data-operations)
  - [List-Based Data Access](#list-based-data-access)
- [Property Management](#property-management)
  - [Basic Property Operations](#basic-property-operations)
  - [Type Conversion](#type-conversion)
  - [List Properties](#list-properties)
  - [Property-to-Data Bridge](#property-to-data-bridge)
  - [Persistence](#persistence)
- [Thread Management](#thread-management)
  - [Starting Threads](#starting-threads)
  - [Thread Progress Tracking](#thread-progress-tracking)
  - [Thread Information](#thread-information)
  - [Thread Lifecycle Helpers](#thread-lifecycle-helpers)
  - [Waiting for Threads](#waiting-for-threads)
- [Mutex Management](#mutex-management)
  - [Named Mutexes](#named-mutexes)
  - [Scoped Locks](#scoped-locks)
  - [Mutex Debugging](#mutex-debugging)
- [Type Registration](#type-registration)
  - [Registering Type Names](#registering-type-names)
  - [Registering Type Enums](#registering-type-enums)
- [Signals and Observers](#signals-and-observers)
- [Utilities](#utilities)
- [Design Patterns](#design-patterns)
- [Thread Safety](#thread-safety)
- [Complete Examples](#complete-examples)
- [Best Practices](#best-practices)

## Overview

The `cvc::app` class is a **singleton** that serves as the central hub for managing:

- **Data objects** - Type-safe storage for application data using `boost::any`
- **Properties** - String-based key-value configuration storage
- **Threads** - Thread lifecycle management with progress tracking
- **Mutexes** - Named mutex management for resource synchronization
- **Signals** - Observer pattern for monitoring state changes

**Key Characteristics:**
- Thread-safe with fine-grained locking
- Type-agnostic data storage via `boost::any`
- Automatic type name resolution
- Built-in change notification via Boost.Signals2
- Progress tracking for long-running operations
- Named mutex system for resource locks

**Accessed via global macro:**
```cpp
cvcapp  // Expands to CVC_NAMESPACE::app::instance()
```

## Quick Start

```cpp
#include <cvc/app.h>

// Store data
cvcapp.data("volume", myVolume);
cvcapp.data("geometry", myGeometry);

// Retrieve data
auto vol = cvcapp.data<cvc::volume>("volume");
auto geom = cvcapp.data<cvc::geometry>("geometry");

// Set properties
cvcapp.properties("window.width", 1920);
cvcapp.properties("window.height", 1080);
cvcapp.properties("render.quality", "high");

// Get properties
int width = cvcapp.properties<int>("window.width");
std::string quality = cvcapp.properties("render.quality");

// Start a background thread
cvcapp.startThread("loader", [&]() {
    thread_feedback feedback("Loading data...");
    // Long-running operation
    cvcapp.threadProgress(0.5);
    // More work...
    cvcapp.threadProgress(1.0);
});

// Named mutex for file access
{
    scoped_lock lock("output.dat", "Writing results");
    // Exclusive access to output.dat
    writeFile("output.dat", results);
}
```

## Singleton Access

### instance()

Returns a reference to the singleton application object.

```cpp
static app& instance();
```

**Usage:**
```cpp
// Direct access
cvc::app& myApp = cvc::app::instance();

// Via global macro (preferred)
cvcapp.data("key", value);
```

**Thread Safety:** ✅ Thread-safe singleton initialization

## Data Management

The data map stores arbitrary typed objects using `boost::any`, providing type-safe storage and retrieval.

### Basic Data Operations

#### data() - Get All Data

```cpp
data_map data();
```

Returns a copy of the entire data map.

```cpp
data_map allData = cvcapp.data();
for (const auto& pair : allData) {
    std::cout << pair.first << std::endl;
}
```

#### data(key) - Get Single Value

```cpp
boost::any data(const std::string& key);
```

Returns the raw `boost::any` value for the given key.

```cpp
boost::any rawValue = cvcapp.data("volume");
```

#### data(key, value) - Set Single Value

```cpp
void data(const std::string& key, const boost::any& value);
```

Stores a value in the data map. Fires `dataChanged` signal.

```cpp
cvcapp.data("volume", myVolume);
cvcapp.data("count", 42);
cvcapp.data("name", std::string("MyApp"));
```

#### data(map) - Batch Set

```cpp
void data(const data_map& map);
```

Merges all entries from the input map into the data map.

```cpp
data_map batch;
batch["volume1"] = vol1;
batch["volume2"] = vol2;
cvcapp.data(batch);
```

### Type-Safe Data Access

#### data\<T\>(key) - Get Typed Value

```cpp
template<class T>
T data(const std::string& key);
```

Returns the value cast to type `T`. Throws `boost::bad_any_cast` if type doesn't match.

```cpp
auto vol = cvcapp.data<cvc::volume>("myVolume");
int count = cvcapp.data<int>("iteration");
std::string name = cvcapp.data<std::string>("projectName");
```

#### isData\<T\>(key) - Type Check

```cpp
template<class T>
bool isData(const std::string& key);
```

Returns `true` if the key exists and can be cast to type `T`.

```cpp
if (cvcapp.isData<cvc::volume>("volume")) {
    auto vol = cvcapp.data<cvc::volume>("volume");
    processVolume(vol);
}
```

### Querying Data by Type

#### data\<T\>() - Get All Keys of Type

```cpp
template<class T>
std::vector<std::string> data();
```

Returns all keys that contain data of type `T`.

```cpp
// Find all volumes in the data map
std::vector<std::string> volumeKeys = cvcapp.data<cvc::volume>();

for (const auto& key : volumeKeys) {
    auto vol = cvcapp.data<cvc::volume>(key);
    std::cout << "Volume: " << key << " - " 
              << vol.XDim() << "x" << vol.YDim() << "x" << vol.ZDim() 
              << std::endl;
}
```

#### data\<T\>(keys) - Get Multiple Objects

```cpp
template<class T>
std::vector<T> data(const std::vector<std::string>& keys);
```

Returns a vector of objects corresponding to the given keys.

```cpp
std::vector<std::string> keys = {"vol1", "vol2", "vol3"};
std::vector<cvc::volume> volumes = cvcapp.data<cvc::volume>(keys);
```

### Batch Data Operations

#### data(keys, objects) - Set Multiple

```cpp
template<class Object>
void data(const std::vector<std::string>& keys, 
          const std::vector<Object>& v);
```

Sets multiple data entries from parallel arrays.

```cpp
std::vector<std::string> keys = {"result1", "result2", "result3"};
std::vector<cvc::volume> results = computeResults();
cvcapp.data(keys, results);
```

#### data(keys, value) - Duplicate Value

```cpp
template<class T>
void data(const std::vector<std::string>& keys, const T& value);
```

Sets the same value for all specified keys.

```cpp
std::vector<std::string> layers = {"layer1", "layer2", "layer3"};
cvcapp.data(layers, defaultVolume);
```

### List-Based Data Access

#### listData\<T\>(keylist) - Parse Comma-Separated List

```cpp
template<class T>
std::vector<T> listData(const std::string& keylist);
```

Parses a comma-separated list of keys and returns corresponding objects.

```cpp
// Property contains: "volume1, volume2, volume3"
cvcapp.properties("input.volumes", "volume1, volume2, volume3");

// Retrieve all volumes in one call
auto volumes = cvcapp.listData<cvc::volume>(
    cvcapp.properties("input.volumes")
);
```

**List Separators:** Configured via `system.list_separators` property (default: `,`)

## Property Management

Properties are string key-value pairs for configuration and settings.

### Basic Property Operations

#### properties() - Get All

```cpp
property_map properties();
```

Returns a copy of all properties.

```cpp
property_map props = cvcapp.properties();
for (const auto& pair : props) {
    std::cout << pair.first << " = " << pair.second << std::endl;
}
```

#### properties(key) - Get Value

```cpp
std::string properties(const std::string& key);
```

Returns the property value as a string.

```cpp
std::string quality = cvcapp.properties("render.quality");
std::string outputPath = cvcapp.properties("output.path");
```

#### properties(key, val) - Set Value (String)

```cpp
void properties(const std::string& key, const std::string& val);
```

Sets a property value. Fires `propertiesChanged` signal.

```cpp
cvcapp.properties("render.quality", "high");
cvcapp.properties("output.format", "rawiv");
```

#### properties(map) - Set Multiple

```cpp
void properties(const property_map& map);
```

Replaces all properties with the given map.

```cpp
property_map config;
config["window.width"] = "1920";
config["window.height"] = "1080";
cvcapp.properties(config);
```

#### addProperties(map) - Merge Properties

```cpp
void addProperties(const property_map& map);
```

Merges properties without clearing existing ones.

```cpp
property_map additional;
additional["plugin.path"] = "/usr/local/plugins";
cvcapp.addProperties(additional);
```

#### hasProperty(key) - Check Existence

```cpp
bool hasProperty(const std::string& key);
```

Returns `true` if the property exists.

```cpp
if (cvcapp.hasProperty("output.path")) {
    std::string path = cvcapp.properties("output.path");
}
```

### Type Conversion

#### properties\<T\>(key, val) - Set with Conversion

```cpp
template<class T>
void properties(const std::string& key, const T& val);
```

Converts value to string using `boost::lexical_cast` before storing.

```cpp
cvcapp.properties("window.width", 1920);
cvcapp.properties("threshold", 0.5);
cvcapp.properties("iterations", 100);
cvcapp.properties("enabled", true);
```

#### properties\<T\>(key) - Get with Conversion

```cpp
template<class T>
T properties(const std::string& key);
```

Converts string property to type `T` using `boost::lexical_cast`. Returns default-constructed `T()` if property doesn't exist.

```cpp
int width = cvcapp.properties<int>("window.width");
double threshold = cvcapp.properties<double>("threshold");
bool enabled = cvcapp.properties<bool>("enabled");
```

### List Properties

#### listProperty(key, unique) - Parse to Strings

```cpp
std::vector<std::string> listProperty(const std::string& key, 
                                      bool uniqueElements = false);
```

Parses a comma-separated property value into a vector of strings.

```cpp
// Property: "input.files" = "data1.rawiv, data2.rawiv, data3.rawiv"
auto files = cvcapp.listProperty("input.files");
// files = {"data1.rawiv", "data2.rawiv", "data3.rawiv"}

// With unique elements
cvcapp.properties("layers", "layer1, layer2, layer1, layer3");
auto uniqueLayers = cvcapp.listProperty("layers", true);
// uniqueLayers = {"layer1", "layer2", "layer3"}
```

#### listProperty\<T\>(key, unique) - Parse with Type Conversion

```cpp
template<class T>
std::vector<T> listProperty(const std::string& key,
                            bool uniqueElements = false);
```

Parses and converts each element to type `T`.

```cpp
// Property: "thresholds" = "0.1, 0.5, 0.9"
auto thresholds = cvcapp.listProperty<double>("thresholds");
// thresholds = {0.1, 0.5, 0.9}

// Property: "resolutions" = "64, 128, 256, 512"
auto sizes = cvcapp.listProperty<int>("resolutions");
// sizes = {64, 128, 256, 512}
```

#### listPropertyAppend(key, val) - Add to List

```cpp
void listPropertyAppend(const std::string& key, const std::string& val);
```

Appends a value to a comma-separated list property.

```cpp
cvcapp.properties("recent.files", "file1.dat");
cvcapp.listPropertyAppend("recent.files", "file2.dat");
cvcapp.listPropertyAppend("recent.files", "file3.dat");
// "recent.files" = "file1.dat, file2.dat, file3.dat"
```

#### listPropertyRemove(key, val) - Remove from List

```cpp
void listPropertyRemove(const std::string& key, const std::string& val);
```

Removes a value from a comma-separated list property.

```cpp
cvcapp.listPropertyRemove("recent.files", "file2.dat");
// "recent.files" = "file1.dat, file3.dat"
```

### Property-to-Data Bridge

#### propertyData\<T\>(propKey, unique) - Property → Data Lookup

```cpp
template<class T>
std::vector<T> propertyData(const std::string& propKey,
                            bool uniqueElements = false);
```

Reads a comma-separated list from a property, treats each item as a data key, and returns the corresponding objects.

```cpp
// Store volumes in data map
cvcapp.data("input1", volume1);
cvcapp.data("input2", volume2);
cvcapp.data("input3", volume3);

// Configure which volumes to process via property
cvcapp.properties("pipeline.inputs", "input1, input2, input3");

// Retrieve all input volumes in one call
auto inputs = cvcapp.propertyData<cvc::volume>("pipeline.inputs");
// inputs = {volume1, volume2, volume3}

// Process them
for (auto& vol : inputs) {
    processVolume(vol);
}
```

### Persistence

#### readPropertyMap(path) - Load Properties

```cpp
void readPropertyMap(const std::string& path);
```

Loads properties from a file (INI or JSON format via Boost.PropertyTree).

```cpp
cvcapp.readPropertyMap("config.ini");
cvcapp.readPropertyMap("settings.json");
```

#### writePropertyMap(path) - Save Properties

```cpp
void writePropertyMap(const std::string& path);
```

Saves all properties to a file.

```cpp
cvcapp.writePropertyMap("config.ini");
cvcapp.writePropertyMap("settings.json");
```

## Thread Management

The app class provides built-in thread management with progress tracking and lifecycle helpers.

### Starting Threads

#### startThread(key, functor, wait)

```cpp
template<class T>
void startThread(const std::string& key, const T& t, bool wait = true);
```

Starts a new thread running the given functor.

**Parameters:**
- `key` - Unique identifier for this thread
- `t` - Functor with `operator()` (lambda, function object, etc.)
- `wait` - If `true` and a thread with this key exists, wait for it to finish before starting new one

```cpp
// Simple thread
cvcapp.startThread("worker", []() {
    std::cout << "Working..." << std::endl;
});

// Thread with captured data
auto processData = [&volume]() {
    thread_feedback feedback("Processing volume...");
    // Long operation
    cvcapp.threadProgress(0.5);
    // More work
    cvcapp.threadProgress(1.0);
};
cvcapp.startThread("processor", processData);

// Don't wait for existing thread (generates unique key)
cvcapp.startThread("task", task, false); // Creates "task.1", "task.2", etc.
```

### Thread Progress Tracking

#### threadProgress() - Get Current Progress

```cpp
double threadProgress(const std::string& key = std::string());
```

Returns progress (0.0 to 1.0) for the specified thread, or current thread if key is empty.

```cpp
double progress = cvcapp.threadProgress("loader");
std::cout << "Loading: " << (progress * 100) << "%" << std::endl;
```

#### threadProgress(progress) - Set Progress

```cpp
void threadProgress(double progress); // 0.0 - 1.0
void threadProgress(const std::string& key, double progress);
```

Sets progress for current thread or specified thread.

```cpp
// In a thread
for (int i = 0; i < 100; i++) {
    processItem(i);
    cvcapp.threadProgress(i / 100.0);
}
```

#### finishThreadProgress(key) - Mark Complete

```cpp
void finishThreadProgress(const std::string& key = std::string());
```

Sets progress to 1.0 for the thread.

```cpp
cvcapp.finishThreadProgress(); // Current thread
```

### Thread Information

#### threadInfo(key, info) - Set Status String

```cpp
void threadInfo(const std::string& key, const std::string& infostr);
std::string threadInfo(const std::string& key = std::string());
```

Associates a status string with a thread for monitoring.

```cpp
cvcapp.threadInfo("loader", "Loading file 1 of 10");
// Later
cvcapp.threadInfo("loader", "Parsing data...");

// Query status
std::string status = cvcapp.threadInfo("loader");
```

#### thisThreadInfo() - Convenience Wrappers

```cpp
void thisThreadInfo(const std::string& infostr);
std::string thisThreadInfo();
```

Shorthand for current thread.

```cpp
cvcapp.thisThreadInfo("Initializing...");
```

### Thread Lifecycle Helpers

#### thread_feedback - RAII Progress Management

```cpp
class thread_feedback {
public:
    thread_feedback(const std::string& info = "running");
    ~thread_feedback();
};
```

Automatically manages thread lifecycle:
- Constructor: Sets progress to 0.0
- Destructor: Sets progress to 1.0, removes thread from map

```cpp
void workerThread() {
    thread_feedback feedback("Processing data");
    
    // Work happens here
    for (int i = 0; i < 100; i++) {
        processItem(i);
        cvcapp.threadProgress(i / 100.0);
        cvcapp.thisThreadInfo("Processing item " + std::to_string(i));
    }
    
    // Automatic cleanup on scope exit
}

cvcapp.startThread("worker", workerThread);
```

#### thread_info - RAII Info Stack

```cpp
class thread_info {
public:
    thread_info(const std::string& info = "running");
    ~thread_info();
};
```

Saves and restores thread info/progress when entering/exiting scopes.

```cpp
void outerFunction() {
    thread_info info("Outer function");
    cvcapp.threadProgress(0.2);
    
    {
        thread_info innerInfo("Inner function");
        cvcapp.threadProgress(0.5);
        // Do work
    } // Progress and info restored to 0.2, "Outer function"
    
    cvcapp.threadProgress(0.8);
}
```

### Waiting for Threads

#### wait() / wait_for_threads()

```cpp
void wait(); // Non-static convenience wrapper
static void wait_for_threads(); // Static version
```

Blocks until all threads complete. Useful for cleanup or shutdown.

```cpp
// Start multiple threads
cvcapp.startThread("worker1", task1);
cvcapp.startThread("worker2", task2);
cvcapp.startThread("worker3", task3);

// Wait for all to complete
cvcapp.wait();

std::cout << "All workers finished" << std::endl;
```

### Thread Utilities

#### threads() - Get All Threads

```cpp
thread_map threads();
```

Returns map of all active threads.

```cpp
thread_map active = cvcapp.threads();
std::cout << "Active threads: " << active.size() << std::endl;
```

#### hasThread(key) - Check Existence

```cpp
bool hasThread(const std::string& key);
```

Returns `true` if thread exists.

```cpp
if (cvcapp.hasThread("loader")) {
    std::cout << "Loader is running" << std::endl;
}
```

#### removeThread(key) - Manual Removal

```cpp
void removeThread(const std::string& key);
```

Removes thread from tracking (doesn't stop the thread).

#### uniqueThreadKey(hint) - Generate Unique Key

```cpp
std::string uniqueThreadKey(const std::string& hint = std::string());
```

Generates a unique thread key based on a hint.

```cpp
std::string key = cvcapp.uniqueThreadKey("worker");
// Returns "worker.1", "worker.2", etc.
```

## Mutex Management

Named mutex system for coordinating access to resources.

### Named Mutexes

#### mutex(name) - Get Mutex

```cpp
mutex_ptr mutex(const std::string& name);
```

Returns a shared pointer to the named mutex, creating it if necessary.

```cpp
auto fileMutex = cvcapp.mutex("output.dat");
boost::mutex::scoped_lock lock(*fileMutex);
// Exclusive access
writeToFile("output.dat", data);
```

### Scoped Locks

#### scoped_lock - RAII Lock with Info

```cpp
class scoped_lock {
public:
    scoped_lock(const std::string& name,
                const std::string& info = std::string());
    ~scoped_lock();
};
```

Acquires named mutex on construction, releases on destruction. Automatically adds thread key to info string.

```cpp
// Simple usage
{
    scoped_lock lock("database");
    // Exclusive database access
    updateDatabase();
} // Lock released

// With description for debugging
{
    scoped_lock lock("output.vti", "Writing volume");
    volume.write("output.vti");
}
```

**Typedef:**
```cpp
typedef cvc::app::scoped_lock scoped_lock;
```

### Mutex Debugging

#### mutexInfo(name, info) - Set Debug Info

```cpp
void mutexInfo(const std::string& name, const std::string& in);
std::string mutexInfo(const std::string& name);
```

Associates debug information with a mutex to track who holds it.

```cpp
// Set manually
cvcapp.mutexInfo("file.dat", "Thread 5: Writing results");

// Query (useful for debugging deadlocks)
std::string holder = cvcapp.mutexInfo("file.dat");
std::cout << "Lock held by: " << holder << std::endl;
```

**Note:** `scoped_lock` automatically sets this to `"<threadKey>: <info>"`

## Type Registration

Register human-readable names and enums for C++ types.

### Registering Type Names

#### registerDataType\<T\>(name) - Register Name

```cpp
template<class T>
void registerDataType(const std::string& datatypename);
```

Associates a friendly name with a C++ type.

```cpp
cvcapp.registerDataType<cvc::volume>("Volume");
cvcapp.registerDataType<cvc::geometry>("Geometry");
cvcapp.registerDataType<std::vector<double>>("DoubleVector");
```

**Macro Shorthand:**
```cpp
#define registerDataType(type) registerDataType<type>(#type)

// Usage
cvcapp.registerDataType(volume);      // Registers as "volume"
cvcapp.registerDataType(geometry);    // Registers as "geometry"
```

#### dataTypeName(key) - Get Name from Key

```cpp
std::string dataTypeName(const std::string& key);
```

Returns the registered name for the type of data at the key.

```cpp
cvcapp.data("myVol", volume);
std::string type = cvcapp.dataTypeName("myVol");
// type = "Volume" (if registered)
```

#### dataTypeName\<T\>() - Get Name from Type

```cpp
template<class T>
std::string dataTypeName();
```

Returns the registered name for type `T`.

```cpp
std::string volTypeName = cvcapp.dataTypeName<cvc::volume>();
// volTypeName = "Volume"
```

#### dataTypeName(any) - Get Name from boost::any

```cpp
std::string dataTypeName(const boost::any& d);
```

Returns the registered name for a `boost::any` value.

```cpp
boost::any data = myVolume;
std::string name = cvcapp.dataTypeName(data);
```

### Registering Type Enums

#### registerDataType\<T\>(enum) - Register Enum

```cpp
template<class T>
void registerDataType(data_type dt);
```

Associates a `data_type` enum value with a C++ type.

```cpp
cvcapp.registerDataType<cvc::volume>(cvc::VolumeData);
cvcapp.registerDataType<cvc::geometry>(cvc::GeometryData);
```

#### dataType(key) - Get Enum from Key

```cpp
data_type dataType(const std::string& key);
```

Returns the enum for the type of data at the key.

```cpp
data_type type = cvcapp.dataType("myVolume");
if (type == cvc::VolumeData) {
    // Handle volume
}
```

#### dataType\<T\>() - Get Enum from Type

```cpp
template<class T>
data_type dataType();
```

Returns the registered enum for type `T`.

```cpp
data_type volType = cvcapp.dataType<cvc::volume>();
// volType = cvc::VolumeData
```

## Signals and Observers

Monitor changes to the app state using Boost.Signals2.

### Available Signals

```cpp
map_change_signal dataChanged;       // Fired when data map changes
map_change_signal propertiesChanged; // Fired when properties change
map_change_signal threadsChanged;    // Fired when thread map changes
map_change_signal mutexesChanged;    // Fired when mutex map changes
```

### Connecting to Signals

```cpp
// Monitor data changes
cvcapp.dataChanged.connect([](const std::string& key) {
    std::cout << "Data changed: " << key << std::endl;
});

// Monitor property changes
cvcapp.propertiesChanged.connect([](const std::string& key) {
    std::cout << "Property changed: " << key << std::endl;
});

// Monitor thread lifecycle
cvcapp.threadsChanged.connect([](const std::string& key) {
    if (cvcapp.hasThread(key)) {
        double progress = cvcapp.threadProgress(key);
        std::cout << key << ": " << (progress * 100) << "%" << std::endl;
    }
});
```

## Utilities

### listify - String ↔ Vector Conversion

#### listify(string) - Parse List

```cpp
std::vector<std::string> listify(const std::string& keylist);
```

Parses a comma-separated string into a vector.

```cpp
auto items = cvcapp.listify("item1, item2, item3");
// items = {"item1", "item2", "item3"}
```

#### listify(vector) - Join List

```cpp
std::string listify(const std::vector<std::string>& keys);
```

Joins a vector into a comma-separated string.

```cpp
std::vector<std::string> items = {"a", "b", "c"};
std::string list = cvcapp.listify(items);
// list = "a, b, c"
```

### sleep - Cross-Platform Delay

```cpp
void sleep(double ms);
```

Sleeps for the specified milliseconds.

```cpp
cvcapp.sleep(100); // Sleep 100ms
```

### log - Logging Output

```cpp
void log(unsigned int level, const std::string& buf);
```

Outputs a log message at the specified level.

```cpp
cvcapp.log(0, "Info: Application started");
cvcapp.log(1, "Warning: Low memory");
cvcapp.log(2, "Error: Failed to load file");
```

## Design Patterns

### Configuration Management Pattern

```cpp
// Load configuration
cvcapp.readPropertyMap("config.ini");

// Access throughout application
int width = cvcapp.properties<int>("window.width");
std::string theme = cvcapp.properties("ui.theme");

// Save modified configuration
cvcapp.writePropertyMap("config.ini");
```

### Data Pipeline Pattern

```cpp
// Stage 1: Load data
cvcapp.data("input", loadVolume("data.rawiv"));

// Stage 2: Process
auto input = cvcapp.data<cvc::volume>("input");
auto processed = applyFilter(input);
cvcapp.data("filtered", processed);

// Stage 3: Save results
auto result = cvcapp.data<cvc::volume>("filtered");
result.write("output.rawiv");
```

### Progress Monitoring Pattern

```cpp
// GUI thread
void updateProgressBar() {
    if (cvcapp.hasThread("processor")) {
        double progress = cvcapp.threadProgress("processor");
        std::string info = cvcapp.threadInfo("processor");
        
        progressBar->setValue(progress * 100);
        statusLabel->setText(info);
    }
}

// Worker thread
cvcapp.startThread("processor", []() {
    thread_feedback feedback("Processing...");
    
    for (int i = 0; i < 100; i++) {
        cvcapp.thisThreadInfo("Processing item " + std::to_string(i));
        cvcapp.threadProgress(i / 100.0);
        processItem(i);
    }
});
```

### Resource Lock Pattern

```cpp
// Multiple threads accessing same file
void writeResults(const std::string& filename, const Results& r) {
    scoped_lock lock(filename, "Writing results");
    std::ofstream out(filename, std::ios::app);
    out << r.toString() << std::endl;
}

// Called from multiple threads safely
cvcapp.startThread("worker1", [&]() {
    writeResults("output.txt", results1);
});
cvcapp.startThread("worker2", [&]() {
    writeResults("output.txt", results2);
});
```

### Observable Data Pattern

```cpp
// Setup observer
cvcapp.dataChanged.connect([](const std::string& key) {
    if (key == "volume") {
        auto vol = cvcapp.data<cvc::volume>(key);
        updateVisualization(vol);
    }
});

// Any code that modifies data triggers update
cvcapp.data("volume", newVolume);  // Observer fires automatically
```

## Thread Safety

### Thread-Safe Operations

✅ **All public methods are thread-safe** with fine-grained locking:

```cpp
// Safe from multiple threads
cvcapp.data("key1", value1);  // Thread 1
cvcapp.data("key2", value2);  // Thread 2
cvcapp.properties("prop", val); // Thread 3
```

### Interruption Points

All operations check for thread interruption:

```cpp
cvcapp.startThread("worker", []() {
    try {
        for (int i = 0; i < 1000; i++) {
            boost::this_thread::interruption_point();
            // Work
        }
    } catch (boost::thread_interrupted&) {
        // Clean shutdown
    }
});

// Later, from another thread
auto thread = cvcapp.threads("worker");
thread->interrupt(); // Cooperative cancellation
```

### Signal Deadlock Prevention

Signals are fired **outside of lock scopes** to prevent deadlocks when observers access the app.

```cpp
// Safe: Observer can access app without deadlock
cvcapp.dataChanged.connect([](const std::string& key) {
    // Can safely call cvcapp methods here
    auto val = cvcapp.data(key);
    cvcapp.properties("last.modified", key);
});
```

## Complete Examples

### Example 1: Volume Processing Pipeline

```cpp
#include <cvc/app.h>
#include <cvc/volume.h>

void processingPipeline() {
    // Register types
    cvcapp.registerDataType<cvc::volume>("Volume");
    
    // Load configuration
    cvcapp.readPropertyMap("pipeline.ini");
    
    // Get input files from config
    auto inputFiles = cvcapp.listProperty("pipeline.inputs");
    
    // Load all volumes
    for (size_t i = 0; i < inputFiles.size(); i++) {
        auto key = "input" + std::to_string(i);
        cvcapp.data(key, cvc::volume(inputFiles[i]));
    }
    
    // Process each volume in parallel
    auto volumeKeys = cvcapp.data<cvc::volume>();
    for (const auto& key : volumeKeys) {
        cvcapp.startThread("process_" + key, [key]() {
            thread_feedback feedback("Processing " + key);
            
            auto vol = cvcapp.data<cvc::volume>(key);
            
            cvcapp.thisThreadInfo("Applying bilateral filter...");
            vol.bilateralFilter(2.0, 0.1);
            cvcapp.threadProgress(0.5);
            
            cvcapp.thisThreadInfo("Saving result...");
            vol.write("processed_" + key + ".rawiv");
            cvcapp.threadProgress(1.0);
        });
    }
    
    // Wait for all processing to complete
    cvcapp.wait();
    
    std::cout << "Pipeline complete" << std::endl;
}
```

### Example 2: Multi-threaded Data Processing

```cpp
void parallelProcessing() {
    // Prepare data
    std::vector<cvc::volume> inputs = loadInputs();
    
    // Store in app
    for (size_t i = 0; i < inputs.size(); i++) {
        cvcapp.data("input" + std::to_string(i), inputs[i]);
    }
    
    // Process in parallel
    for (size_t i = 0; i < inputs.size(); i++) {
        std::string key = "input" + std::to_string(i);
        cvcapp.startThread("worker" + std::to_string(i), [key, i]() {
            thread_feedback feedback;
            
            auto vol = cvcapp.data<cvc::volume>(key);
            
            for (int step = 0; step < 10; step++) {
                cvcapp.thisThreadInfo("Step " + std::to_string(step));
                cvcapp.threadProgress(step / 10.0);
                
                processStep(vol, step);
                
                cvcapp.sleep(100); // Simulate work
            }
            
            cvcapp.data("output" + std::to_string(i), vol);
        });
    }
    
    // Monitor progress
    while (cvcapp.threads().size() > 0) {
        std::cout << "Active threads: " << cvcapp.threads().size() << std::endl;
        cvcapp.sleep(500);
    }
    
    // Collect results
    auto outputs = cvcapp.data<cvc::volume>();
    std::cout << "Processed " << outputs.size() << " volumes" << std::endl;
}
```

### Example 3: Configuration-Driven Application

```cpp
class MyApplication {
public:
    void initialize() {
        // Register types
        cvcapp.registerDataType<cvc::volume>("Volume");
        cvcapp.registerDataType<cvc::geometry>("Geometry");
        
        // Load configuration
        cvcapp.readPropertyMap("myapp.ini");
        
        // Setup from configuration
        int width = cvcapp.properties<int>("window.width");
        int height = cvcapp.properties<int>("window.height");
        std::string theme = cvcapp.properties("ui.theme");
        
        createWindow(width, height, theme);
        
        // Load recent files
        auto recentFiles = cvcapp.listProperty("recent.files");
        populateRecentMenu(recentFiles);
        
        // Monitor configuration changes
        cvcapp.propertiesChanged.connect([this](const std::string& key) {
            if (key == "ui.theme") {
                applyTheme(cvcapp.properties("ui.theme"));
            }
        });
    }
    
    void loadFile(const std::string& path) {
        cvcapp.startThread("loader", [this, path]() {
            thread_feedback feedback("Loading file");
            
            cvcapp.thisThreadInfo("Reading file: " + path);
            auto vol = cvc::volume(path);
            cvcapp.threadProgress(0.5);
            
            cvcapp.thisThreadInfo("Updating display");
            cvcapp.data("current.volume", vol);
            cvcapp.threadProgress(1.0);
            
            // Update recent files
            cvcapp.listPropertyAppend("recent.files", path);
        });
    }
    
    void shutdown() {
        // Wait for pending operations
        cvcapp.wait();
        
        // Save configuration
        cvcapp.writePropertyMap("myapp.ini");
    }
};
```

### Example 4: Thread-Safe File Access

```cpp
void multiThreadedFileWriter() {
    std::vector<std::string> workers = {"A", "B", "C", "D"};
    
    for (const auto& name : workers) {
        cvcapp.startThread("writer_" + name, [name]() {
            thread_feedback feedback("Writer " + name);
            
            for (int i = 0; i < 10; i++) {
                // Acquire lock before writing
                {
                    scoped_lock lock("output.log", 
                                   "Writer " + name + " iteration " + std::to_string(i));
                    
                    std::ofstream out("output.log", std::ios::app);
                    out << "Writer " << name << " - Iteration " << i << std::endl;
                }
                
                cvcapp.threadProgress(i / 10.0);
                cvcapp.sleep(100);
            }
        });
    }
    
    cvcapp.wait();
    std::cout << "All writes complete" << std::endl;
}
```

## Best Practices

### 1. Always Use thread_feedback for Long Operations

```cpp
// Good
cvcapp.startThread("worker", []() {
    thread_feedback feedback("Working");
    // Automatic progress initialization and cleanup
});

// Avoid
cvcapp.startThread("worker", []() {
    // Manual progress management (error-prone)
});
```

### 2. Use Descriptive Keys

```cpp
// Good
cvcapp.data("user.preferences.volume1", vol);
cvcapp.properties("render.quality.high", "true");

// Avoid
cvcapp.data("v1", vol);
cvcapp.properties("q", "true");
```

### 3. Leverage Type Registration

```cpp
// Register early in main()
cvcapp.registerDataType<cvc::volume>("Volume");
cvcapp.registerDataType<cvc::geometry>("Geometry");

// Now get nice names
std::string type = cvcapp.dataTypeName("myVolume");
// type = "Volume" instead of mangled C++ name
```

### 4. Use Properties for Configuration

```cpp
// Good - easy to modify without recompiling
cvcapp.properties("iterations", 100);
cvcapp.writePropertyMap("config.ini");

// Avoid - hardcoded
const int ITERATIONS = 100;
```

### 5. Monitor Thread Progress

```cpp
// Good
cvcapp.startThread("processor", []() {
    thread_feedback feedback("Processing");
    for (int i = 0; i < 100; i++) {
        cvcapp.threadProgress(i / 100.0);
        cvcapp.thisThreadInfo("Processing item " + std::to_string(i));
        // work
    }
});

// Avoid - no progress feedback
cvcapp.startThread("processor", []() {
    // User has no idea what's happening
});
```

### 6. Use scoped_lock for Resource Access

```cpp
// Good - automatic lock management
{
    scoped_lock lock("resource", "Operation X");
    accessResource();
}

// Avoid - manual lock management
auto m = cvcapp.mutex("resource");
m->lock();
accessResource();
m->unlock(); // Easy to forget!
```

### 7. Handle Thread Interruption

```cpp
// Good
cvcapp.startThread("worker", []() {
    try {
        for (int i = 0; i < 1000; i++) {
            boost::this_thread::interruption_point();
            // work
        }
    } catch (boost::thread_interrupted&) {
        cleanup();
    }
});

// Can safely interrupt
cvcapp.threads("worker")->interrupt();
```

### 8. Use Signals for Loose Coupling

```cpp
// Good - Observer pattern
cvcapp.dataChanged.connect([](const std::string& key) {
    if (key.find("volume.") == 0) {
        updateVisualization();
    }
});

// Avoid - Tight coupling
void setVolume(const cvc::volume& vol) {
    cvcapp.data("volume", vol);
    updateVisualization(); // Hard-coded dependency
}
```

---

**Full Documentation:**
- [State API (cvc::state)](STATE_API.md)
- [Testing Guide](TESTING.md)
- [Project Overview](PROJECT_REPORT.md)
- [SDF Library](SDF_LIBRARY.md)
