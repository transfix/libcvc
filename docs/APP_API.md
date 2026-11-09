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
- [Program Execution (state_exec)](#program-execution-state_exec)
  - [Ariadne hosts and the frame pump](#ariadne-hosts-and-the-frame-pump)
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

## Program Execution (state_exec)

These members connect an app to state_exec programs. Each is built at its first call.
[STATE_EXEC_DEVELOPER_GUIDE.md](STATE_EXEC_DEVELOPER_GUIDE.md#which-scheduler-runs-a-program)
says which host and lane runs on which scheduler, and
[STATE_EXEC_THREADED.md](STATE_EXEC_THREADED.md) 4.12 designs the pool and its groups.

- `exec_scheduler()`: the app's one `async_scheduler`, group 0. Ariadne's actions and residents,
  `AppRuntime`'s verbs and the app's posting hosts (sliced tasks, net and stream intrinsics) run
  on it, one timeline for every document, except an isolated document's, which run in its own
  group ([below](#ariadne-hosts-and-the-frame-pump)). Building it reads `state_exec.policy` and
  `state_exec.process_class`, and makes the lock manager below if nothing has yet. It has its own
  cycle registry and `math.random` stream, which it binds whichever thread drives it, and draws its
  pids from `exec_pid_source()`.
- `exec_async_host()`: the async host for `exec_scheduler()`, which it builds first. An async
  function registered with it parks its calling process while the kernel runs on the app's
  background pool.
- `lock_manager()`: the `with-lock` table that `exec_scheduler()`, Ariadne's `init:` lane,
  pycvc's `Exec`s and every `exec_group` built on the app share. Made at the first call to it, to
  `exec_scheduler()`, to `exec_pool()` asking for threads, or by building an `exec_group` on the
  app, which fixes its mode from `state_exec.locks.threading`.
- `exec_pool()` and `exec_pool(const pool_options &)` (`<cvc/core/exec_pool.h>`): the app's one
  pool of execution groups, which pumps `exec_group`s fork-join on persistent workers and the
  calling thread (Ariadne's frame groups from Stage 1b, pycvc's `ExecPool`, a host's own groups).
  It is built at the first call, under its own mutex: from the `pool_options` that call gives, else
  from the tree's `state_exec.threads` (a whole number or `auto`; the `CVC_EXEC_THREADS`
  environment variable overrides it), `state_exec.cpu_set` (`performance` or `all`) and
  `state_exec.deterministic`. Options apply only when the call builds the pool: once it is built, a
  call keeps it, and one giving different options logs that they were not applied. An
  `exec_pool(opts)` asking for threads above 0 before anything has made `lock_manager()` makes the
  manager shared, for this app only. Workers pump only over a shared manager: if the manager is
  already single (building an `exec_group` on the app makes it, in a C++ host without
  `state_exec.threads`, and logs so once), the pool is built with 0 threads and a log line says
  why; and every pool pumps a group on a single manager on its calling thread only. With 0
  threads, or in a build without threads, every pump runs on the calling thread. The pool's clamps
  (`hardware_concurrency() - 1`, the affinity mask) and any worker that failed to start are logged
  once.
- `exec_pool_probe()`: the pool if it has been built, else null. It never builds it.
- `exec_pump_frame()`: one frame's pump of the app's groups, on the host thread, outside a pump:
  a single `exec_pool().pump_all` with a 0.1 s deadline from its entry and Ariadne's budget (200000
  steps and 0.1 s per group) over every registered frame group, with group 0 (`exec_scheduler()`,
  once it is built and holds a live process; never built here) pumped first on the calling thread
  as a host-affinity group: its token taken, `drain_ingress`, `wake_awaiting` and
  `sync_run_steps`. A frame group the deadline skipped last frame starts before group 0 whenever
  the calling thread pumps it, so a busy group 0 delays it by one frame at most. In deterministic
  mode (the pool's, or with no pool the tree's `state_exec.deterministic` and
  `CVC_EXEC_DETERMINISTIC`) group 0 is pumped with steps only and no deadline, and its
  `math.random` stream is seeded from the name "group0". With no frame group registered it builds
  no pool and pumps group 0 alone. Then `exec_pump_generation()` advances: the count of
  `exec_pump_frame()` calls that have finished, which Ariadne's `drain_end()` reads (Stage 1b) to
  skip its own pump in a frame the host already pumped.
- `cvc::pump_exec_frame(sched, lanes, max_steps, max_time)` (`<cvc/core/async_lane.h>`) is a
  different thing: one frame of one scheduler (`run_deferred()` on the lanes given, then
  `wake_awaiting`, one unconditional `sync_step` and `sync_run`), with a render-loop budget of 20000
  steps and 3 ms by default. The cvcGL tests drive `exec_scheduler()` with it. It pumps no group,
  goes through no pool and does not advance `exec_pump_generation()`; the scheduler binds its own
  registry and stream all the same.
- `register_frame_group(group)`: have every `exec_pump_frame()` pump `group` until the returned
  `frame_group_handle` is reset or destroyed. The handle is movable, not copyable. The group must
  outlive it: destroy the handle first, on the host thread, outside a pump.
- `exec_pid_source()`: the pids `exec_scheduler()` and every `exec_group` built with this app draw
  from, so none repeats app-wide. It is made with group 0's state, counting from 1, so the app
  scheduler's first pid is 1, as before.

A value posted to a group's scheduler (`group.sched().post_message(channel, value)`) is moved in:
the host hands it over and keeps no reference to it, and never posts one compound value (a list or
dict) to two groups. The handle is copied, not the list, so a list the host keeps and changes, or
posts again, is shared with a group a worker may be pumping: a data race in a release build, which
nothing checks in this stage. Post a fresh value each time, or a string. `exec_group::execute` of a
`value_t` copies the value's lists and dicts itself.

Settings are read once, so a host sets them before the first call to any of these:

```cpp
cvc::app app;
app.root()("state_exec.policy").value("priority_rr");
app.root()("state_exec.locks.threading").value("shared");
cvc::state_exec::async_scheduler &sched = app.exec_scheduler(); // reads both now
```

A host that pumps its own groups on the app's pool gives its pool options first, so the lock
manager is made shared:

```cpp
cvc::app app;
cvc::pool_options po;
po.threads = cvc::pool_options::auto_threads;  // the performance cores less one
cvc::exec_pool &pool = app.exec_pool(po);      // makes the lock manager shared, for this app

cvc::state_exec::exec_group::options o;
o.name = "sim";
o.app = &app;
cvc::state_exec::exec_group sim(std::move(o)); // its scheduler, host, registry and stream
cvc::frame_group_handle h = app.register_frame_group(sim);
for (;;) {
  // ... events, posts to sim.sched() ...
  app.exec_pump_frame();                       // group 0 and `sim`, one pump_all, 0.1 s at most
  // ... read results, render ...
}
```

### Ariadne hosts and the frame pump

An Ariadne document's programs run on `exec_scheduler()`, or on an execution group of the
document's own when the app tree's `ariadne.exec.isolate_documents` isolates it
([ARIADNE.md](ARIADNE.md) 2.5). Its Runtime (`<cvc/ariadne/ariadne.h>`) and `AppRuntime`
(`<cvc/ariadne/app_runtime.h>`) meet the frame pump here:

- `Runtime::drain()` is `drain_begin()` then `drain_end()`. `drain_begin()` is the frame's host
  work (the queued events and key bindings, the actions and residents submitted, the frame's tick
  posted) and notes `exec_pump_generation()`. `drain_end()` pumps the document's scheduler (group
  0 a bounded slice, when the document used it or anything lives on it; the document's group, if
  it has one, through `exec_pool().pump_all`, 200000 steps and a 0.1 s deadline) unless
  `exec_pump_generation()` moved since this Runtime's `drain_begin()`, that is, unless the host
  called `exec_pump_frame()` in between; then it sweeps what finished. `drain_end()` alone runs
  `drain_begin()` first.
- `AppRuntime::drain_begin()` and `drain_end()` are the Runtime's, with the scene work, the
  warnings and the frame hooks after `drain_end()`; `AppRuntime::drain()` is both.
- **A host with several documents** calls `drain_begin()` on each, `exec_pump_frame()` once, then
  `drain_end()` on each: group 0 is pumped once a frame (its budget 200000 steps a frame, not one
  per document, and an await crosses one frame), every isolated document's group, which its
  Runtime registered as a frame group, is pumped in the same `pump_all`, and each `drain_end()`
  only sweeps. A host does that or calls `drain()` on each, not both: `drain()` beside
  `exec_pump_frame()` pumps a document's group twice a frame. `cvc::pump_exec_frame` is no
  substitute: it does not advance `exec_pump_generation()`, so every `drain_end()` would pump
  again. A host with one document keeps calling `drain()`.
- **Workers.** For an isolated document to run on a worker, the pool needs workers and a shared
  lock manager: give the pool options before anything calls `exec_scheduler()` or
  `lock_manager()` (an `AppRuntime`'s `load()`, its verb registrations and `enable_net` build the
  scheduler), or set `state_exec.threads` in the tree.
- `AppRuntime::declare_thread_affine_prefix(prefix)` declares that a subscriber the host connected
  at or under `prefix` must run on the host thread; an isolated document whose prefix is at, above
  or under a declared one is then pumped on the host thread (`Runtime::declare_thread_affine_prefix`
  underneath). Call it before `load()`; a later call has the placement decided again.
- `AppRuntime::use_script_engine(engine)` keeps an isolated document on the host thread
  (`Runtime::require_host_thread`): an engine's activations run its language inline in their
  steps. Call it before `load()`, as a scripting host's `attach` does.
- `AppRuntime::wait_idle()` waits, without cancelling anything, until no async kernel of the
  document is queued or running: on `exec_async_host()`, and on its group's host when the document
  is isolated. It reads the group the document is in now (`Runtime::current_group()`) and decides
  nothing, so it never builds, moves or destroys the document's group (it may build group 0, as
  `exec_async_host()` does, though `load()` has built it already). `AppRuntime::idle()` counts the
  group's calls and runnable processes the same way. Both also count the kernels still running in
  a group a move retired (`Runtime::retired_calls_in_flight()`, `wait_retired_calls()`): `idle()`
  is false while one runs and `wait_idle()` waits for it, so a host that waits before letting go
  of what its async functions use waits for those too; a later `drain_begin` destroys the retired
  group once they end. Host thread, outside a frame.
- Every one of these, like `load()`, `set_root()`, the `set_*_program` calls, `post_input`,
  `post_signal` and the destructors, is a host-thread call made outside a pump: never from a
  native, a callback or a host verb that the app's pool runs (`run_program` only queues, so it may
  be called from inside one). Tearing a Runtime down, or moving its document out of its group,
  while a `pump_all` of the app's pool holds that group is refused in every build, from any thread
  (printed and logged, then the process aborts); a document in group 0, or one whose group no pump
  holds, is not refused.
- **Registrations.** An `AppRuntime`'s verbs and functions, and providers
  (`register_action_intrinsics`), are registered before `load()` or before the first frame. One
  registered once the document's programs run moves an isolated document only the first time
  nothing of it runs (no live action, resident, mount tick or spawned child), at the latest at its
  next restart (a load, `set_root` or a lane's program); until then a late `host_only` name is a
  `NameError` in its `any` group (logged once).

```cpp
cvc::app app;
cvc::pool_options po;
po.threads = cvc::pool_options::auto_threads;
app.exec_pool(po);                                        // first: the lock manager is shared
app.root()("ariadne.exec.isolate_documents").value("1");
std::vector<std::unique_ptr<cvc::ariadne::AppRuntime>> docs;
for (int i = 0; i < 4; ++i) {
  docs.push_back(std::make_unique<cvc::ariadne::AppRuntime>(app, "docs.d" + std::to_string(i)));
  docs.back()->load(path);                                // decides where the document runs
}
for (;;) {
  for (auto &d : docs) d->drain_begin();
  app.exec_pump_frame();                                  // group 0 once, and each document's group
  for (auto &d : docs) d->drain_end();
  // ... render each document ...
}
```

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
