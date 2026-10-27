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
  before lower-priority ones when the pool is at capacity, and tasks of
  one priority start in the order they were submitted.
- **Keyed tasks are tracked threads.** A keyed task (`wait=true`) gets a
  thread of its own, registered in the app's thread map under its key
  before it runs, so the same APIs that work for `startThread`
  (`hasThread`, `threads(key)` to join or interrupt it, keyed progress
  and info, `thread_feedback`) also work for it. When the pool has room
  for it and for every task already queued, it is registered before
  `startThreadPooled` returns, so `if (app.hasThread(k))
  app.threads(k)->join();` waits for it.
- **Anonymous tasks run on persistent workers.** An anonymous task
  (`wait=false`, or `submitPooled`) runs on one of at most
  `getThreadPoolSize()` worker threads the pool starts on demand (a
  burst of N tasks starts min(N, pool size) of them at once) and keeps
  until the app goes. It has no key and no thread-map entry, and
  the pool destroys it, with everything it captured, as soon as it
  returns. This is the path for many small tasks (state_exec's async
  calls, `compute_async`): no thread is created per task.
- **Cooperative interruption.** Tasks honor
  `boost::this_thread::interruption_point()` and shut down cleanly when
  interrupted (`threads(key)->interrupt()` for a keyed one; `wait()` and
  `~app` interrupt every running task). `~app` drops the tasks still
  queued without running them.
- **Exception-safe.** Exceptions thrown by a task are logged and do not
  take down the pool or other workers.
- **Dynamic resizing.** `setThreadPoolSize()` may be called while tasks
  are running.

## Quick Start

```cpp
#include <cvc/core/app.h>

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

### Many small tasks

```cpp
// No key, no thread per task: runs on the pool's persistent workers.
const int n = 10000;
std::atomic<int> left(n);
std::promise<void> all_done;
for (int i = 0; i < n; ++i)
  app.submitPooled([i, &left, &all_done]() {
    process_item(i);
    if (--left == 0)
      all_done.set_value();
  }, cvc::PRIORITY_NORMAL);
all_done.get_future().wait(); // the tasks' own completion
```

Wait on the tasks' own completion, as here. `app.wait()` is a shutdown
step, not a barrier: it first interrupts every running task (one that
reaches an interruption point ends early, without an error) and then
waits at most 5 s for the pool to drain, with a warning if it doesn't.

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

Keyed `startThreadPooled` tasks and `startThread` share the same key
namespace, so keys must be unique across both.

## Tips

- **Unique keys.** Each concurrently-running keyed task needs a unique
  key. A common pattern is `prefix_<index>` or `prefix_<uuid>`. If you
  don't need to find the task by key, submit it anonymously instead.
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
