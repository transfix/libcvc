# Application Framework API (cvc::app)

*Reference for the `cvc::app` context object — the data, properties,
threads, and named-mutex hub used throughout libcvc.*

## Table of Contents

- [Overview](#overview)
- [Quick Start](#quick-start)
- [Constructing an App](#constructing-an-app)
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

`cvc::app` is a **constructible context object** that bundles the
state libcvc components share among themselves:

- **Data objects** — type-safe storage via `boost::any`
- **Properties** — string-keyed configuration values with `lexical_cast` shortcuts
- **Threads** — lifecycle management with progress reporting and an optional priority pool
- **Named mutexes** — process-wide resource locks identified by string name
- **Signals** — `boost::signals2` change notifications on the four maps above

Construct as many `cvc::app` instances as you need — typically one
per logical application or per test fixture — and pass references
into the components that should share state.

```cpp
cvc::app app;          // default-constructed: registers libcvc's built-in types
// ... or, for a stripped-down instance (e.g. tests):
cvc::app bare(cvc::no_init_t{});
```

> **History.** Earlier versions of libcvc exposed `cvc::app` as a
> process-wide singleton accessed via `app::instance()` and the
> `cvcapp` macro. Both have been removed; `cvc::app` is now
> exclusively constructed by callers and passed as `cvc::app& ctx`.
> The RAII helpers `thread_info`, `thread_feedback`, and
> `scoped_lock` all take a leading `app& ctx` parameter for this
> reason.

**Key characteristics:**
- Thread-safe with fine-grained locking
- Type-agnostic data storage via `boost::any`
- Optional human-readable type name registration
- Change notification via `boost::signals2`
- Progress tracking and an optional priority thread pool
- Named-mutex system for cross-thread resource locks

## Quick Start

```cpp
#include <cvc/core/app.h>

cvc::app app;   // your application's context object

// Store data
app.data("volume", myVolume);
app.data("geometry", myGeometry);

// Retrieve data
auto vol  = app.data<cvc::volume>("volume");
auto geom = app.data<cvc::geometry>("geometry");

// Set properties (lexical_cast shortcut for non-string types)
app.properties("window.width", 1920);
app.properties("window.height", 1080);
app.properties("render.quality", "high");

// Get properties
int width            = app.properties<int>("window.width");
std::string quality  = app.properties("render.quality");

// Start a background thread; pass `app` to the RAII helper so it
// reports into this app's thread map.
app.startThread("loader", [&app]() {
    cvc::thread_feedback feedback(app, "Loading data...");
    app.threadProgress(0.5);
    // ... more work ...
    app.threadProgress(1.0);
});

// Named mutex for file access
{
    cvc::scoped_lock lock(app, "output.dat", "Writing results");
    writeFile("output.dat", results);
}
```

Throughout the rest of this document, `app` denotes a `cvc::app`
instance you have constructed (as in the snippet above).

## Constructing an App

```cpp
namespace cvc {
  struct no_init_t {};            // tag type for the bare overload

  class app {
  public:
    app();                        // registers libcvc's built-in data types
    explicit app(no_init_t);      // skips type registration
    ~app();
    // ... (rest of the API documented below)
  };
}
```

### Default construction

```cpp
cvc::app app;
```

Registers libcvc's built-in C++ → human-readable type-name
mappings (`cvc::volume`, `cvc::geometry`, the scalar types, etc.)
and installs the default file-format handlers.

### Bare construction (tests, lightweight callers)

```cpp
cvc::app bare(cvc::no_init_t{});
```

Skips type registration and default-handler installation. Useful in
unit tests that exercise just the data map or property store and
don't want the global IO handlers fired up.

### Lifetime

`cvc::app` is move-only-ish (the copy constructor is private).
Construct one at the scope that owns it (`main`, a test fixture, a
cluster-node `cvcsrv` instance) and pass references downward.

## Data Management

The data map stores arbitrary typed objects using `boost::any`, providing type-safe storage and retrieval.

### Basic Data Operations

#### data() - Get All Data

```cpp
data_map data();
```

Returns a copy of the entire data map.

```cpp
data_map allData = app.data();
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
boost::any rawValue = app.data("volume");
```

#### data(key, value) - Set Single Value

```cpp
void data(const std::string& key, const boost::any& value);
```

Stores a value in the data map. Fires `dataChanged` signal.

```cpp
app.data("volume", myVolume);
app.data("count", 42);
app.data("name", std::string("MyApp"));
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
app.data(batch);
```

### Type-Safe Data Access

#### data\<T\>(key) - Get Typed Value

```cpp
template<class T>
T data(const std::string& key);
```

Returns the value cast to type `T`. Throws `boost::bad_any_cast` if type doesn't match.

```cpp
auto vol = app.data<cvc::volume>("myVolume");
int count = app.data<int>("iteration");
std::string name = app.data<std::string>("projectName");
```

#### isData\<T\>(key) - Type Check

```cpp
template<class T>
bool isData(const std::string& key);
```

Returns `true` if the key exists and can be cast to type `T`.

```cpp
if (app.isData<cvc::volume>("volume")) {
    auto vol = app.data<cvc::volume>("volume");
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
std::vector<std::string> volumeKeys = app.data<cvc::volume>();

for (const auto& key : volumeKeys) {
    auto vol = app.data<cvc::volume>(key);
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
std::vector<cvc::volume> volumes = app.data<cvc::volume>(keys);
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
app.data(keys, results);
```

#### data(keys, value) - Duplicate Value

```cpp
template<class T>
void data(const std::vector<std::string>& keys, const T& value);
```

Sets the same value for all specified keys.

```cpp
std::vector<std::string> layers = {"layer1", "layer2", "layer3"};
app.data(layers, defaultVolume);
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
app.properties("input.volumes", "volume1, volume2, volume3");

// Retrieve all volumes in one call
auto volumes = app.listData<cvc::volume>(
    app.properties("input.volumes")
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
property_map props = app.properties();
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
std::string quality = app.properties("render.quality");
std::string outputPath = app.properties("output.path");
```

#### properties(key, val) - Set Value (String)

```cpp
void properties(const std::string& key, const std::string& val);
```

Sets a property value. Fires `propertiesChanged` signal.

```cpp
app.properties("render.quality", "high");
app.properties("output.format", "rawiv");
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
app.properties(config);
```

#### addProperties(map) - Merge Properties

```cpp
void addProperties(const property_map& map);
```

Merges properties without clearing existing ones.

```cpp
property_map additional;
additional["plugin.path"] = "/usr/local/plugins";
app.addProperties(additional);
```

#### hasProperty(key) - Check Existence

```cpp
bool hasProperty(const std::string& key);
```

Returns `true` if the property exists.

```cpp
if (app.hasProperty("output.path")) {
    std::string path = app.properties("output.path");
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
app.properties("window.width", 1920);
app.properties("threshold", 0.5);
app.properties("iterations", 100);
app.properties("enabled", true);
```

#### properties\<T\>(key) - Get with Conversion

```cpp
template<class T>
T properties(const std::string& key);
```

Converts string property to type `T` using `boost::lexical_cast`. Returns default-constructed `T()` if property doesn't exist.

```cpp
int width = app.properties<int>("window.width");
double threshold = app.properties<double>("threshold");
bool enabled = app.properties<bool>("enabled");
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
auto files = app.listProperty("input.files");
// files = {"data1.rawiv", "data2.rawiv", "data3.rawiv"}

// With unique elements
app.properties("layers", "layer1, layer2, layer1, layer3");
auto uniqueLayers = app.listProperty("layers", true);
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
auto thresholds = app.listProperty<double>("thresholds");
// thresholds = {0.1, 0.5, 0.9}

// Property: "resolutions" = "64, 128, 256, 512"
auto sizes = app.listProperty<int>("resolutions");
// sizes = {64, 128, 256, 512}
```

#### listPropertyAppend(key, val) - Add to List

```cpp
void listPropertyAppend(const std::string& key, const std::string& val);
```

Appends a value to a comma-separated list property.

```cpp
app.properties("recent.files", "file1.dat");
app.listPropertyAppend("recent.files", "file2.dat");
app.listPropertyAppend("recent.files", "file3.dat");
// "recent.files" = "file1.dat, file2.dat, file3.dat"
```

#### listPropertyRemove(key, val) - Remove from List

```cpp
void listPropertyRemove(const std::string& key, const std::string& val);
```

Removes a value from a comma-separated list property.

```cpp
app.listPropertyRemove("recent.files", "file2.dat");
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
app.data("input1", volume1);
app.data("input2", volume2);
app.data("input3", volume3);

// Configure which volumes to process via property
app.properties("pipeline.inputs", "input1, input2, input3");

// Retrieve all input volumes in one call
auto inputs = app.propertyData<cvc::volume>("pipeline.inputs");
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
app.readPropertyMap("config.ini");
app.readPropertyMap("settings.json");
```

#### writePropertyMap(path) - Save Properties

```cpp
void writePropertyMap(const std::string& path);
```

Saves all properties to a file.

```cpp
app.writePropertyMap("config.ini");
app.writePropertyMap("settings.json");
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
app.startThread("worker", []() {
    std::cout << "Working..." << std::endl;
});

// Thread with captured data
auto processData = [&volume]() {
    thread_feedback feedback("Processing volume...");
    // Long operation
    app.threadProgress(0.5);
    // More work
    app.threadProgress(1.0);
};
app.startThread("processor", processData);

// Don't wait for existing thread (generates unique key)
app.startThread("task", task, false); // Creates "task.1", "task.2", etc.
```

### Thread Progress Tracking

#### threadProgress() - Get Current Progress

```cpp
double threadProgress(const std::string& key = std::string());
```

Returns progress (0.0 to 1.0) for the specified thread, or current thread if key is empty.

```cpp
double progress = app.threadProgress("loader");
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
    app.threadProgress(i / 100.0);
}
```

#### finishThreadProgress(key) - Mark Complete

```cpp
void finishThreadProgress(const std::string& key = std::string());
```

Sets progress to 1.0 for the thread.

```cpp
app.finishThreadProgress(); // Current thread
```

### Thread Information

#### threadInfo(key, info) - Set Status String

```cpp
void threadInfo(const std::string& key, const std::string& infostr);
std::string threadInfo(const std::string& key = std::string());
```

Associates a status string with a thread for monitoring.

```cpp
app.threadInfo("loader", "Loading file 1 of 10");
// Later
app.threadInfo("loader", "Parsing data...");

// Query status
std::string status = app.threadInfo("loader");
```

#### thisThreadInfo() - Convenience Wrappers

```cpp
void thisThreadInfo(const std::string& infostr);
std::string thisThreadInfo();
```

Shorthand for current thread.

```cpp
app.thisThreadInfo("Initializing...");
```

### Thread Lifecycle Helpers

#### thread_feedback - RAII Progress Management

```cpp
class thread_feedback {
public:
    thread_feedback(app& ctx, const std::string& key = "");
    ~thread_feedback();
};
```

Automatically manages thread lifecycle:
- Constructor: sets progress to 0.0 in the bound app's thread map.
- Destructor: marks the thread "completed" and finishes its progress entry.

```cpp
void workerThread(cvc::app& app) {
    cvc::thread_feedback feedback(app, "Processing data");

    for (int i = 0; i < 100; i++) {
        processItem(i);
        app.threadProgress(i / 100.0);
        app.thisThreadInfo("Processing item " + std::to_string(i));
    }
    // Automatic cleanup on scope exit.
}

app.startThread("worker", [&app]{ workerThread(app); });
```

#### thread_info - RAII Info Stack

```cpp
class thread_info {
public:
    thread_info(app& ctx, const std::string& info = "running");
    ~thread_info();
};
```

Saves and restores thread info/progress when entering/exiting scopes.

```cpp
void outerFunction(cvc::app& app) {
    cvc::thread_info info(app, "Outer function");
    app.threadProgress(0.2);

    {
        cvc::thread_info innerInfo(app, "Inner function");
        app.threadProgress(0.5);
        // Do work.
    } // Progress and info restored to 0.2, "Outer function"
    
    app.threadProgress(0.8);
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
app.startThread("worker1", task1);
app.startThread("worker2", task2);
app.startThread("worker3", task3);

// Wait for all to complete
app.wait();

std::cout << "All workers finished" << std::endl;
```

### Thread Utilities

#### threads() - Get All Threads

```cpp
thread_map threads();
```

Returns map of all active threads.

```cpp
thread_map active = app.threads();
std::cout << "Active threads: " << active.size() << std::endl;
```

#### hasThread(key) - Check Existence

```cpp
bool hasThread(const std::string& key);
```

Returns `true` if thread exists.

```cpp
if (app.hasThread("loader")) {
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
std::string key = app.uniqueThreadKey("worker");
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
auto fileMutex = app.mutex("output.dat");
boost::mutex::scoped_lock lock(*fileMutex);
// Exclusive access
writeToFile("output.dat", data);
```

### Scoped Locks

#### scoped_lock - RAII Lock with Info

```cpp
class scoped_lock {
public:
    scoped_lock(app& ctx,
                const std::string& name,
                const std::string& info = std::string());
    ~scoped_lock();
};
```

Acquires the named mutex from `ctx.mutex(name)` on construction and
releases it on destruction. The current thread key is automatically
prepended to the `info` string registered via `mutexInfo`, so the
mutex's debug record identifies the holder.

```cpp
// Simple usage
{
    cvc::scoped_lock lock(app, "database");
    updateDatabase();   // exclusive access
} // lock released

// With description for debugging
{
    cvc::scoped_lock lock(app, "output.vti", "Writing volume");
    volume.write("output.vti");
}
```

**Typedef** (in `namespace cvc`):
```cpp
typedef app::scoped_lock scoped_lock;
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
app.mutexInfo("file.dat", "Thread 5: Writing results");

// Query (useful for debugging deadlocks)
std::string holder = app.mutexInfo("file.dat");
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
app.registerDataType<cvc::volume>("Volume");
app.registerDataType<cvc::geometry>("Geometry");
app.registerDataType<std::vector<double>>("DoubleVector");
```

**Macro Shorthand:**
```cpp
#define registerDataType(type) registerDataType<type>(#type)

// Usage
app.registerDataType(volume);      // Registers as "volume"
app.registerDataType(geometry);    // Registers as "geometry"
```

#### dataTypeName(key) - Get Name from Key

```cpp
std::string dataTypeName(const std::string& key);
```

Returns the registered name for the type of data at the key.

```cpp
app.data("myVol", volume);
std::string type = app.dataTypeName("myVol");
// type = "Volume" (if registered)
```

#### dataTypeName\<T\>() - Get Name from Type

```cpp
template<class T>
std::string dataTypeName();
```

Returns the registered name for type `T`.

```cpp
std::string volTypeName = app.dataTypeName<cvc::volume>();
// volTypeName = "Volume"
```

#### dataTypeName(any) - Get Name from boost::any

```cpp
std::string dataTypeName(const boost::any& d);
```

Returns the registered name for a `boost::any` value.

```cpp
boost::any data = myVolume;
std::string name = app.dataTypeName(data);
```

### Registering Type Enums

#### registerDataType\<T\>(enum) - Register Enum

```cpp
template<class T>
void registerDataType(data_type dt);
```

Associates a `data_type` enum value with a C++ type.

```cpp
app.registerDataType<cvc::volume>(cvc::VolumeData);
app.registerDataType<cvc::geometry>(cvc::GeometryData);
```

#### dataType(key) - Get Enum from Key

```cpp
data_type dataType(const std::string& key);
```

Returns the enum for the type of data at the key.

```cpp
data_type type = app.dataType("myVolume");
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
data_type volType = app.dataType<cvc::volume>();
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
app.dataChanged.connect([](const std::string& key) {
    std::cout << "Data changed: " << key << std::endl;
});

// Monitor property changes
app.propertiesChanged.connect([](const std::string& key) {
    std::cout << "Property changed: " << key << std::endl;
});

// Monitor thread lifecycle
app.threadsChanged.connect([](const std::string& key) {
    if (app.hasThread(key)) {
        double progress = app.threadProgress(key);
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
auto items = app.listify("item1, item2, item3");
// items = {"item1", "item2", "item3"}
```

#### listify(vector) - Join List

```cpp
std::string listify(const std::vector<std::string>& keys);
```

Joins a vector into a comma-separated string.

```cpp
std::vector<std::string> items = {"a", "b", "c"};
std::string list = app.listify(items);
// list = "a, b, c"
```

### sleep - Cross-Platform Delay

```cpp
void sleep(double ms);
```

Sleeps for the specified milliseconds.

```cpp
app.sleep(100); // Sleep 100ms
```

### log - Logging Output

```cpp
void log(unsigned int level, const std::string& buf);
```

Outputs a log message at the specified level.

```cpp
app.log(0, "Info: Application started");
app.log(1, "Warning: Low memory");
app.log(2, "Error: Failed to load file");
```

## Design Patterns

### Configuration Management Pattern

```cpp
// Load configuration
app.readPropertyMap("config.ini");

// Access throughout application
int width = app.properties<int>("window.width");
std::string theme = app.properties("ui.theme");

// Save modified configuration
app.writePropertyMap("config.ini");
```

### Data Pipeline Pattern

```cpp
// Stage 1: Load data
app.data("input", loadVolume("data.rawiv"));

// Stage 2: Process
auto input = app.data<cvc::volume>("input");
auto processed = applyFilter(input);
app.data("filtered", processed);

// Stage 3: Save results
auto result = app.data<cvc::volume>("filtered");
result.write("output.rawiv");
```

### Progress Monitoring Pattern

```cpp
// GUI thread
void updateProgressBar() {
    if (app.hasThread("processor")) {
        double progress = app.threadProgress("processor");
        std::string info = app.threadInfo("processor");
        
        progressBar->setValue(progress * 100);
        statusLabel->setText(info);
    }
}

// Worker thread
app.startThread("processor", []() {
    thread_feedback feedback("Processing...");
    
    for (int i = 0; i < 100; i++) {
        app.thisThreadInfo("Processing item " + std::to_string(i));
        app.threadProgress(i / 100.0);
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
app.startThread("worker1", [&]() {
    writeResults("output.txt", results1);
});
app.startThread("worker2", [&]() {
    writeResults("output.txt", results2);
});
```

### Observable Data Pattern

```cpp
// Setup observer
app.dataChanged.connect([](const std::string& key) {
    if (key == "volume") {
        auto vol = app.data<cvc::volume>(key);
        updateVisualization(vol);
    }
});

// Any code that modifies data triggers update
app.data("volume", newVolume);  // Observer fires automatically
```

## Thread Safety

### Thread-Safe Operations

✅ **All public methods are thread-safe** with fine-grained locking:

```cpp
// Safe from multiple threads
app.data("key1", value1);  // Thread 1
app.data("key2", value2);  // Thread 2
app.properties("prop", val); // Thread 3
```

### Interruption Points

All operations check for thread interruption:

```cpp
app.startThread("worker", []() {
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
auto thread = app.threads("worker");
thread->interrupt(); // Cooperative cancellation
```

### Signal Deadlock Prevention

Signals are fired **outside of lock scopes** to prevent deadlocks when observers access the app.

```cpp
// Safe: Observer can access app without deadlock
app.dataChanged.connect([](const std::string& key) {
    // Can safely call app methods here
    auto val = app.data(key);
    app.properties("last.modified", key);
});
```

## Complete Examples

The examples below assume an `app` of type `cvc::app&` is reachable
from the surrounding scope (a function parameter, a class member,
or a local variable created with `cvc::app app;`). Any thread
function or lambda that uses it should capture it by reference
(e.g. `[&app]`).

### Example 1: Volume Processing Pipeline

```cpp
#include <cvc/core/app.h>
#include <cvc/volume/volume.h>

void processingPipeline() {
    // Register types
    app.registerDataType<cvc::volume>("Volume");
    
    // Load configuration
    app.readPropertyMap("pipeline.ini");
    
    // Get input files from config
    auto inputFiles = app.listProperty("pipeline.inputs");
    
    // Load all volumes
    for (size_t i = 0; i < inputFiles.size(); i++) {
        auto key = "input" + std::to_string(i);
        app.data(key, cvc::volume(inputFiles[i]));
    }
    
    // Process each volume in parallel
    auto volumeKeys = app.data<cvc::volume>();
    for (const auto& key : volumeKeys) {
        app.startThread("process_" + key, [key]() {
            thread_feedback feedback("Processing " + key);
            
            auto vol = app.data<cvc::volume>(key);
            
            app.thisThreadInfo("Applying bilateral filter...");
            vol.bilateralFilter(2.0, 0.1);
            app.threadProgress(0.5);
            
            app.thisThreadInfo("Saving result...");
            vol.write("processed_" + key + ".rawiv");
            app.threadProgress(1.0);
        });
    }
    
    // Wait for all processing to complete
    app.wait();
    
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
        app.data("input" + std::to_string(i), inputs[i]);
    }
    
    // Process in parallel
    for (size_t i = 0; i < inputs.size(); i++) {
        std::string key = "input" + std::to_string(i);
        app.startThread("worker" + std::to_string(i), [key, i]() {
            thread_feedback feedback;
            
            auto vol = app.data<cvc::volume>(key);
            
            for (int step = 0; step < 10; step++) {
                app.thisThreadInfo("Step " + std::to_string(step));
                app.threadProgress(step / 10.0);
                
                processStep(vol, step);
                
                app.sleep(100); // Simulate work
            }
            
            app.data("output" + std::to_string(i), vol);
        });
    }
    
    // Monitor progress
    while (app.threads().size() > 0) {
        std::cout << "Active threads: " << app.threads().size() << std::endl;
        app.sleep(500);
    }
    
    // Collect results
    auto outputs = app.data<cvc::volume>();
    std::cout << "Processed " << outputs.size() << " volumes" << std::endl;
}
```

### Example 3: Configuration-Driven Application

```cpp
class MyApplication {
public:
    void initialize() {
        // Register types
        app.registerDataType<cvc::volume>("Volume");
        app.registerDataType<cvc::geometry>("Geometry");
        
        // Load configuration
        app.readPropertyMap("myapp.ini");
        
        // Setup from configuration
        int width = app.properties<int>("window.width");
        int height = app.properties<int>("window.height");
        std::string theme = app.properties("ui.theme");
        
        createWindow(width, height, theme);
        
        // Load recent files
        auto recentFiles = app.listProperty("recent.files");
        populateRecentMenu(recentFiles);
        
        // Monitor configuration changes
        app.propertiesChanged.connect([this](const std::string& key) {
            if (key == "ui.theme") {
                applyTheme(app.properties("ui.theme"));
            }
        });
    }
    
    void loadFile(const std::string& path) {
        app.startThread("loader", [this, path]() {
            thread_feedback feedback("Loading file");
            
            app.thisThreadInfo("Reading file: " + path);
            auto vol = cvc::volume(path);
            app.threadProgress(0.5);
            
            app.thisThreadInfo("Updating display");
            app.data("current.volume", vol);
            app.threadProgress(1.0);
            
            // Update recent files
            app.listPropertyAppend("recent.files", path);
        });
    }
    
    void shutdown() {
        // Wait for pending operations
        app.wait();
        
        // Save configuration
        app.writePropertyMap("myapp.ini");
    }
};
```

### Example 4: Thread-Safe File Access

```cpp
void multiThreadedFileWriter() {
    std::vector<std::string> workers = {"A", "B", "C", "D"};
    
    for (const auto& name : workers) {
        app.startThread("writer_" + name, [name]() {
            thread_feedback feedback("Writer " + name);
            
            for (int i = 0; i < 10; i++) {
                // Acquire lock before writing
                {
                    scoped_lock lock("output.log", 
                                   "Writer " + name + " iteration " + std::to_string(i));
                    
                    std::ofstream out("output.log", std::ios::app);
                    out << "Writer " << name << " - Iteration " << i << std::endl;
                }
                
                app.threadProgress(i / 10.0);
                app.sleep(100);
            }
        });
    }
    
    app.wait();
    std::cout << "All writes complete" << std::endl;
}
```

## Best Practices

### 1. Always Use thread_feedback for Long Operations

```cpp
// Good
app.startThread("worker", []() {
    thread_feedback feedback("Working");
    // Automatic progress initialization and cleanup
});

// Avoid
app.startThread("worker", []() {
    // Manual progress management (error-prone)
});
```

### 2. Use Descriptive Keys

```cpp
// Good
app.data("user.preferences.volume1", vol);
app.properties("render.quality.high", "true");

// Avoid
app.data("v1", vol);
app.properties("q", "true");
```

### 3. Leverage Type Registration

```cpp
// Register early in main()
app.registerDataType<cvc::volume>("Volume");
app.registerDataType<cvc::geometry>("Geometry");

// Now get nice names
std::string type = app.dataTypeName("myVolume");
// type = "Volume" instead of mangled C++ name
```

### 4. Use Properties for Configuration

```cpp
// Good - easy to modify without recompiling
app.properties("iterations", 100);
app.writePropertyMap("config.ini");

// Avoid - hardcoded
const int ITERATIONS = 100;
```

### 5. Monitor Thread Progress

```cpp
// Good
app.startThread("processor", []() {
    thread_feedback feedback("Processing");
    for (int i = 0; i < 100; i++) {
        app.threadProgress(i / 100.0);
        app.thisThreadInfo("Processing item " + std::to_string(i));
        // work
    }
});

// Avoid - no progress feedback
app.startThread("processor", []() {
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
auto m = app.mutex("resource");
m->lock();
accessResource();
m->unlock(); // Easy to forget!
```

### 7. Handle Thread Interruption

```cpp
// Good
app.startThread("worker", []() {
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
app.threads("worker")->interrupt();
```

### 8. Use Signals for Loose Coupling

```cpp
// Good - Observer pattern
app.dataChanged.connect([](const std::string& key) {
    if (key.find("volume.") == 0) {
        updateVisualization();
    }
});

// Avoid - Tight coupling
void setVolume(const cvc::volume& vol) {
    app.data("volume", vol);
    updateVisualization(); // Hard-coded dependency
}
```

---

**Full Documentation:**
- [State API (cvc::state)](STATE_API.md)
- [Testing Guide](TESTING.md)
- [SDF Library](SDF_LIBRARY.md)
