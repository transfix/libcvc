# Thread Pool API

## Overview

The CVC application class now includes a priority-based thread pool to manage concurrent task execution. This allows you to limit the number of simultaneously running heavy computations and prioritize critical tasks.

## Basic Usage

### Starting a Pooled Thread

```cpp
#include <cvc/app.h>

// Define your task
class MyTask {
public:
  void operator()() {
    // Your computation here
    thread_feedback feedback("Processing data...");
    
    for (int i = 0; i < 100; i++) {
      boost::this_thread::interruption_point();
      cvcapp.threadProgress(i / 100.0);
      // Do work...
    }
  }
};

// Submit to thread pool with normal priority
cvcapp.startThreadPooled("myTask", MyTask(), PRIORITY_NORMAL);
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
cvcapp.setThreadPoolSize(8);

// Get current pool size
unsigned int poolSize = cvcapp.getThreadPoolSize();

// Check active and pending threads
unsigned int active = cvcapp.getActiveThreadCount();
unsigned int pending = cvcapp.getPendingThreadCount();
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
cvcapp.setThreadPoolSize(4);

// Submit 100 tasks - only 4 run concurrently
for (int i = 0; i < 100; i++) {
  std::string key = "batch_" + std::to_string(i);
  
  // Critical tasks processed first
  thread_priority priority = (i < 10) ? PRIORITY_CRITICAL : PRIORITY_NORMAL;
  
  cvcapp.startThreadPooled(key, [i]() {
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
5. **Thread Tracking**: Running tasks appear in `cvcapp.threads()` map with their keys

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
