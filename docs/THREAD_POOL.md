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
  app.threads(k)->join();` waits for it; if its thread can't be created
  then, `startThreadPooled` throws `boost::thread_resource_error`.
- **Anonymous tasks run on persistent workers.** An anonymous task
  (`wait=false`, or `submitPooled`) runs on one of at most
  `getThreadPoolSize()` worker threads the pool starts on demand (a
  burst of N tasks starts min(N, pool size) of them at once) and keeps
  until the app goes. It has no key and no thread-map entry, and
  the pool destroys it, with everything it captured, as soon as it
  returns. This is the path for many small tasks (state_exec's async
  calls, `compute_async`): no thread is created per task, and however
  many are submitted, `threads()` does not grow and no two of them
  share a key or cancel each other.
- **Cooperative interruption.** Tasks honor
  `boost::this_thread::interruption_point()` and shut down cleanly when
  interrupted (`threads(key)->interrupt()` for a keyed one; `wait()` and
  `~app` interrupt every running task, also one a worker started a
  moment ago). `thread_feedback` and `thread_info` hold interruption off
  while they keep their books, so an interrupt that arrives in them
  stays pending for the task's next interruption point instead of being
  swallowed. `wait()`, `~app` and the destructors of dropped tasks do
  the same, and `~app` holds interruption off until its last member is
  destroyed. `~app` closes the pool before anything else but its async
  host's shutdown (and before that too when the pool has no worker,
  which could never run a kernel the host waits for): no queued task
  starts after that, the queued ones are dropped without running, and
  from then on `submitPooled` drops a task at once and returns `false`.
  A dropped task is destroyed, so it can report the drop from its
  destructor (the async host fails the call with `RuntimeError`,
  `launch_pool_task` posts `on_error`'s value), saying why with
  `app::poolDropReason()`.
- **Thread creation failures.** If a new worker's thread can't be
  created, the queued tasks wait for a running worker. When the pool has
  none at all (thread resources were exhausted before its first worker),
  `trySubmitPooled` hands the task back unrun, once any other worker
  creation under way has succeeded or failed too (or when a keyed task
  queued ahead of it, which it starts on the calling thread, can't get a
  thread, nor can the worker asked for as that frees its slot):
  `compute_async` then throws `boost::thread_resource_error` and runs
  nothing, so its caller can do the work itself (a threaded tf overlay
  task carries on cooperatively), an async call raises `RuntimeError`
  ("not run: no background worker could be started"), and
  `launch_pool_task` posts `on_error`'s value. While no worker runs yet,
  `trySubmitPooled` (and so each of those) waits for the worker
  creations under way to succeed or fail, up to 5 s, before it returns.
  A task it queued can still be left with no worker: keyed threads held
  every slot of a pool with none, and the worker asked for as one of
  them ends can't be created either. Once a worker creation fails and
  leaves the pool with no worker running or being created (also while
  other keyed threads still hold their slots), such a task is dropped
  unrun: its async call raises the same `RuntimeError`,
  `launch_pool_task` posts `on_error`'s value, and `compute_async`'s
  `on_error` form calls `on_error` with "not run: no background worker
  could be started" (a threaded tf overlay task then carries on
  cooperatively). The drop runs on the thread whose worker creation
  failed or, if a `trySubmitPooled` call was still deciding about its
  own task then (waiting on that creation, or starting a keyed task),
  on that call's thread before it returns, so another submitter's
  `on_error` or post can run inside a `compute_async` call, an async
  call or `launch_pool_task`. `compute_async`'s other form has no way
  to report a drop, so its job, like a `submitPooled` task, stays
  queued until a later submission or `setThreadPoolSize()` starts a
  worker. A pool that never got a worker can't run a queued async
  kernel, so `~app` drops those (a kernel queued behind keyed threads
  still running, say) before it shuts down the async host, which would
  otherwise wait for them for good. A keyed task whose own thread can't be created as
  `startThreadPooled` starts it makes that call throw
  `boost::thread_resource_error`; one that waited in the queue is
  dropped unrun when its thread can't be created, and the drop logged
  before its slot is freed.
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
An interrupt sent to the thread calling it once it has begun doesn't
cut it short: it stays pending until `wait()` returns. One already
pending when `wait()` is called is thrown at once, before it does
anything.

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

## Thread Classes

`cvc/core/thread_class.h` asks the OS for less of the machine, for a thread
or for the whole process (docs/STATE_EXEC_LOCKS.md 7.6). It is for hosts
only: no DSL intrinsic reaches it. It never goes above normal and never
uses a realtime class. Each call returns whether the OS took it; it never
throws.

```cpp
#include <cvc/core/thread_class.h>

cvc::set_current_thread_class(cvc::thread_class::background); // this thread
cvc::set_process_class(cvc::thread_class::utility);           // the process, from any thread
```

| Platform   | `background` | `utility` | `normal` |
|------------|--------------|-----------|----------|
| Windows    | `THREAD_MODE_BACKGROUND_BEGIN`; process: `PROCESS_MODE_BACKGROUND_BEGIN` | `THREAD_PRIORITY_BELOW_NORMAL`; process: `BELOW_NORMAL_PRIORITY_CLASS` | background mode ended, `THREAD_PRIORITY_NORMAL`; process: `NORMAL_PRIORITY_CLASS` |
| Linux      | nice 10 | nice 5 | nice 0 |
| macOS      | `QOS_CLASS_BACKGROUND`; process: `PRIO_DARWIN_BG` (also throttles I/O) | `QOS_CLASS_UTILITY`; process: none (false) | `QOS_CLASS_DEFAULT`; process: `PRIO_DARWIN_BG` cleared |
| Emscripten | false | false | false |

- **Linux classes are per thread.** A thread's nice value is its own, and an
  unprivileged thread can lower it but not raise it back (that needs
  `CAP_SYS_NICE` or an `RLIMIT_NICE` that allows it), so going from
  `background` back to `utility` or `normal` returns false. `set_process_class`
  stores the class, sets every thread that exists now (the main thread
  included, through `/proc/self/task`), and the library's worker loops apply
  it as each worker starts: `cvc::thread_pool`'s workers, `cvc::async_lane`'s,
  and the app pool's. Any other new thread inherits the nice value of the
  thread that creates it, so a thread a host makes from a thread that was
  never demoted keeps its own.
- **macOS** `utility` and `background` also steer threads to the efficiency
  cores.
- **`state_exec.process_class`** (`background`, `utility` or `normal`) on the
  app tree sets the process class when `app::exec_scheduler()` is first built,
  on whatever thread builds it; the result, or an unknown value, is logged.
  pycvc has `pycvc.set_thread_class(pycvc.thread_class.background)` for the
  calling thread.

## See Also

- [THREAD_POOL_API.md](THREAD_POOL_API.md) - complete API reference
- [APP_API.md](APP_API.md) - the surrounding `cvc::app` API
  (`startThread`, `threads`, `hasThread`, `threadProgress`, ...)
