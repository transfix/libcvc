# Thread Pool API

## Overview

The `cvc::app` class includes a priority-based thread pool to manage concurrent task execution. This allows you to limit the number of simultaneously running heavy computations and prioritize critical tasks.

The examples below assume `cvc::app app;` is in scope and use it as
the explicit context (a fixture member, `main()`-scoped object, etc.).
The previously documented `cvcapp` macro has been removed; every
caller now passes its own `cvc::app&`.

## Basic Usage

### Starting a Pooled Thread

```cpp
#include <cvc/core/app.h>

// Define your task
class MyTask {
public:
  cvc::app& app;
  explicit MyTask(cvc::app& a) : app(a) {}
  void operator()() {
    // Your computation here
    cvc::thread_feedback feedback(app, "Processing data...");

    for (int i = 0; i < 100; i++) {
      boost::this_thread::interruption_point();
      app.threadProgress(i / 100.0);
      // Do work...
    }
  }
};

// Submit to thread pool with normal priority
app.startThreadPooled("myTask", MyTask(app), PRIORITY_NORMAL);
```

### Priority Levels

```cpp
PRIORITY_LOW       = 0  // Background tasks
PRIORITY_NORMAL    = 1  // Default priority
PRIORITY_HIGH      = 2  // Important tasks
PRIORITY_CRITICAL  = 3  // Urgent tasks
```

Higher priority tasks are scheduled before lower priority ones when the pool is at capacity.

### Configuring Pool Size

```cpp
// Set maximum concurrent threads (default: hardware_concurrency or 4)
app.setThreadPoolSize(8);

// Get current pool size
unsigned int poolSize = app.getThreadPoolSize();

// Check active and pending threads
unsigned int active = app.getActiveThreadCount();
unsigned int pending = app.getPendingThreadCount();
```

## Comparison: startThread vs startThreadPooled

### `startThread(key, task, wait=true)`
- Creates thread immediately
- No limit on concurrent threads
- Can flood system with threads
- Use for: One-off tasks, quick operations

### `startThreadPooled(key, task, priority=PRIORITY_NORMAL, wait=true)`
- Queued if pool is at capacity
- Limited concurrent threads
- Priority-based scheduling
- Use for: Heavy computations, many parallel tasks

## Example: Batch Processing

```cpp
// Configure pool for 4 concurrent threads
app.setThreadPoolSize(4);

// Submit 100 tasks - only 4 run concurrently
for (int i = 0; i < 100; i++) {
  std::string key = "batch_" + std::to_string(i);
  
  // Critical tasks processed first
  thread_priority priority = (i < 10) ? PRIORITY_CRITICAL : PRIORITY_NORMAL;
  
  app.startThreadPooled(key, [i]() {
    thread_feedback feedback("Processing batch " + std::to_string(i));
    // Do work...
  }, priority, false); // false = don't wait, use unique key
}
```

## Thread Pool Behavior

1. **Task Submission**: Tasks are added to a priority queue
2. **Worker Allocation**: Workers start when pool has capacity
3. **Execution**: Highest priority task is executed next
4. **Completion**: Worker processes next task or terminates if queue is empty
5. **Thread Tracking**: Running tasks appear in `app.threads()` map with their keys

## Notes

- The pool uses detached worker threads for management
- Tasks still appear in the thread map and can be interrupted
- `waitForHandlers()` in `state_object` waits for all handler threads
- Thread pool workers are internal and don't appear in the thread map
- Pool size can be adjusted at runtime

## Disabled Tests

The test `GeometryTest.SDFSignAmbiguityThreshold` is now disabled by default as it takes 10+ minutes to run. Enable it with:

```bash
ctest -R SDFSignAmbiguityThreshold --gtest_also_run_disabled_tests
```

This allows faster iteration during development while preserving the ability to run comprehensive SDF tests when needed.
