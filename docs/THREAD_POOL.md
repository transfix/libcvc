# Thread Pool

A priority-based thread pool built into the libcvc application class
(`cvc::app`). Use it to run many concurrent tasks without flooding the
system, prioritize urgent work, and integrate with libcvc's standard
thread-tracking and progress-reporting utilities.

For the full per-method reference, see
[THREAD_POOL_API.md](THREAD_POOL_API.md).

## Capabilities

- **Bounded concurrency.** Configure how many tasks run at once; extra
  submissions queue automatically.
- **Four priority levels.** `PRIORITY_LOW`, `PRIORITY_NORMAL`,
  `PRIORITY_HIGH`, `PRIORITY_CRITICAL`. Higher-priority tasks run
  before lower-priority ones when the pool is at capacity.
- **Tracked threads.** Each pooled task is registered in the app's
  thread map under a user-supplied key, so the same APIs that work for
  `startThread` (`hasThread`, `threads(key)`, progress, interruption,
  `thread_feedback`) also work for pooled tasks.
- **Cooperative interruption.** Tasks honor
  `boost::this_thread::interruption_point()` and shut down cleanly when
  interrupted or when the application exits.
- **Exception-safe.** Exceptions thrown by a task are logged and do not
  take down the pool or other workers.
- **Dynamic resizing.** `setThreadPoolSize()` may be called while tasks
  are running.

## Quick Start

```cpp
#include <cvc/app.h>

// Submit work to the pool. Returns immediately; the task runs when a
// worker slot is available.
app.startThreadPooled("compute_sdf",
                         []() { compute_sdf(); },
                         cvc::PRIORITY_NORMAL,
                         /*wait=*/true);
```

## Examples

### Parallel batch with bounded concurrency

```cpp
unsigned int saved = app.getThreadPoolSize();
app.setThreadPoolSize(4);            // at most 4 concurrent

std::vector<std::string> keys;
for (int i = 0; i < 100; ++i) {
  std::string key = "batch_" + std::to_string(i);
  keys.push_back(key);
  app.startThreadPooled(key,
                           [i]() { process_item(i); },
                           cvc::PRIORITY_NORMAL,
                           true);
}

// Wait for completion via the standard thread map.
for (const auto& key : keys) {
  if (app.hasThread(key)) {
    if (auto t = app.threads(key)) t->join();
  }
}

app.setThreadPoolSize(saved);
```

### Mixing priorities

```cpp
// A long-running background scan.
app.startThreadPooled("background_scan",
                         []() { scan_dataset(); },
                         cvc::PRIORITY_LOW, true);

// User-initiated work jumps the queue.
app.startThreadPooled("user_request",
                         []() { handle_user_request(); },
                         cvc::PRIORITY_HIGH, true);
```

### Reporting progress and respecting interruption

```cpp
app.startThreadPooled("import", [&app]() {
  cvc::thread_feedback feedback(app, "Importing volume...");
  for (int i = 0; i < 100; ++i) {
    boost::this_thread::interruption_point();
    app.threadProgress(i / 100.0);
    do_step(i);
  }
}, cvc::PRIORITY_NORMAL, true);
```

### Inspecting the pool

```cpp
unsigned int max_concurrent = app.getThreadPoolSize();
unsigned int running        = app.getActiveThreadCount();
unsigned int queued         = app.getPendingThreadCount();
```

## When to Use the Pool vs. `startThread`

| Use the thread pool                              | Use `startThread`                          |
| ------------------------------------------------ | ------------------------------------------ |
| Many tasks (more than core count)                | One-off or fire-and-forget work            |
| Heavy computations you want to throttle          | Lightweight, short-lived threads           |
| Mixed priorities matter                          | All work is equal priority                 |
| You want queuing instead of unbounded threads    | You want the thread to start immediately   |

`startThreadPooled` and `startThread` share the same key namespace, so
keys must be unique across both.

## Tips

- **Unique keys.** Each concurrently-running task needs a unique key.
  A common pattern is `prefix_<index>` or `prefix_<uuid>`.
- **Don't block the calling thread holding pool resources.** If task A
  waits on task B, make sure the pool has room for both, or submit B
  first.
- **Pool size defaults** to `std::thread::hardware_concurrency()`,
  falling back to 4 if that returns 0.
- **CUDA + threading.** The pool is exercised in libcvc's tests with
  concurrent CUDA operations and concurrent SDF computations; see the
  `voxels` and `geometry` test binaries for live examples.

## See Also

- [THREAD_POOL_API.md](THREAD_POOL_API.md) - complete API reference
- [APP_API.md](APP_API.md) - the surrounding `cvc::app` API
  (`startThread`, `threads`, `hasThread`, `threadProgress`, ...)
