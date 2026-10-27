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

1. **Task Submission**: Tasks are added to one queue, ordered by priority and,
   within a priority, by submission (FIFO)
2. **Admission**: At most `getThreadPoolSize()` tasks run at once
   (`getActiveThreadCount()`); the rest wait (`getPendingThreadCount()`)
3. **Anonymous tasks** (`wait=false`, `submitPooled`): run on persistent worker
   threads, started on demand up to the pool size and kept until `~app`. A
   finished task is destroyed on its worker before it counts as done, so it
   keeps nothing alive. They have no key and never appear in `app.threads()`.
4. **Keyed tasks** (`wait=true`): get a thread of their own, registered in
   `app.threads()` under the key before the task runs. When the pool has room
   at submission for it and for every task already queued (anonymous ones
   included), it starts on the submitting thread and is registered before
   `startThreadPooled` returns; otherwise it is registered when it leaves the
   queue. The entry stays after the task ends, like `startThread`'s.
5. **Shutdown**: `wait()` interrupts the running tasks and waits (up to 5 s)
   for the pool to drain, so it is not a completion barrier: wait on the
   tasks' own completion for that. `~app` interrupts the running tasks
   (every task a worker has taken, however recently), drops the queued ones
   and joins the workers

## Notes

- `submitPooled(std::function<void()>, priority)` is `startThreadPooled` with
  `wait=false` without the key argument
- Keyed tasks appear in the thread map and can be interrupted through it
- `waitForHandlers()` in `state_object` waits for all handler threads
- Thread pool workers are internal and don't appear in the thread map
- Pool size can be adjusted at runtime

## Disabled Tests

The test `GeometryTest.SDFSignAmbiguityThreshold` is now disabled by default as it takes 10+ minutes to run. Enable it with:

```bash
ctest -R SDFSignAmbiguityThreshold --gtest_also_run_disabled_tests
```

This allows faster iteration during development while preserving the ability to run comprehensive SDF tests when needed.
