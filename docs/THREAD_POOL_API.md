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
// Configure pool for 4 concurrent tasks
app.setThreadPoolSize(4);

// Count finished batches yourself: an anonymous task has no key and no entry in app.threads(),
// so keyed threadProgress/threadInfo don't see it.
auto done = std::make_shared<std::atomic<int>>(0);

// Submit 100 tasks - only 4 run concurrently
for (int i = 0; i < 100; i++) {
  // Critical tasks processed first
  cvc::thread_priority priority = (i < 10) ? cvc::PRIORITY_CRITICAL : cvc::PRIORITY_NORMAL;

  app.startThreadPooled("batch", [i, done]() {
    // Do work on batch i...
    ++*done;
  }, priority, false); // false = anonymous, as app.submitPooled(task, priority): "batch" is unused
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
   keeps nothing alive. They have no key and never appear in `app.threads()`,
   so submitting many (`compute_async` a few times a second, say) does not
   grow it, and none needs reaping once it ends.
4. **Keyed tasks** (`wait=true`): get a thread of their own, registered in
   `app.threads()` under the key before the task runs. When the pool has room
   at submission for it and for every task already queued (anonymous ones
   included), it starts on the submitting thread and is registered before
   `startThreadPooled` returns; otherwise it is registered when it leaves the
   queue. The entry stays after the task ends, like `startThread`'s. If its
   thread can't be created as it starts on the submitting thread,
   `startThreadPooled` throws `boost::thread_resource_error` and nothing new is
   registered; one leaving the queue is dropped unrun then, and the drop logged
   before its slot is freed (so once `wait()` returns, or the pool's counts
   are 0, it is in the log).
5. **Shutdown**: `wait()` interrupts the running tasks and waits (up to 5 s)
   for the pool to drain, so it is not a completion barrier: wait on the
   tasks' own completion for that. `~app` first shuts down its own exec
   async host (`exec_async_host()`, if it was made): it cancels the
   host's calls and waits for their kernels, which run on the pool, so
   the pool is still open then. A pool with no worker, running or being
   created, can't run a kernel still queued (one queued behind keyed
   threads that hold every slot, say), and the host would wait for it
   for good: `~app` then closes the pool before the host's shutdown,
   which drops the kernel and ends its call. Otherwise it closes the
   pool right after the host. From then on no queued task starts. It
   interrupts the running tasks (every task a worker has taken, however
   recently: a new worker takes no task until it is among the workers
   these interrupts go to) and drops the queued ones; then it interrupts
   every thread in `app.threads()`, keyed pool tasks included, and only
   then joins the workers and those threads, so a pool task waiting on
   one of them isn't left waiting out a join. Once it has joined the
   workers it sweeps the map again, also when the first sweep found it
   empty: a worker it was joining may have registered a keyed thread
   (one it took before the pool closed, or one its task started). It
   sweeps for threads registered meanwhile a bounded number of times.
   `wait()` and a dropped task's destruction hold interruption off, so
   an interrupt sent to the thread doing them stays pending; `~app`
   holds it off from its first step until its last member is destroyed,
   so a member's destructor that reaches an interruption point doesn't
   throw, and an interrupt pending on the thread destroying the app is
   still pending once the app is gone. A dropped task is destroyed
   without running, and so is one submitted after that point
   (`submitPooled` then returns `false`); a task whose submitter must
   hear of it reports it from its destructor, as the async host,
   `launch_pool_task` and `compute_async`'s `on_error` form do, with
   `app::poolDropReason()` (`app::kPoolStopped` here). Their callables
   hold that state by value, so each task is one heap block: a
   `shared_ptr` capture would add a block per task under libstdc++,
   whose `std::function` keeps only a trivially copyable callable in its
   own buffer
6. **Thread creation failures**: a queued task waits for a running
   worker when another one's thread can't be created. When the pool has
   no worker at all (thread resources were exhausted before its first),
   `trySubmitPooled` hands the task back unrun (`PoolSubmit::no_worker`)
   once the worker creations it relied on have failed, or when a keyed
   task queued ahead of it that it starts (on the calling thread) can't
   get a thread, nor can the worker asked for as that frees its slot;
   while no worker runs yet, it waits for those creations (up to 5 s)
   before it returns. A `trySubmitPooled` task queued behind keyed
   threads that hold every slot of a pool with no worker is dropped
   later, once a worker creation fails and leaves the pool with no
   worker running or being created: the one asked for as one of those
   threads ends, say, even while the others still hold their slots. It
   reports the drop with `app::kPoolNoWorker` ("not run: no background
   worker could be started"). The drop runs on the thread whose worker
   creation failed or, if a `trySubmitPooled` call was still deciding
   about its own task then (waiting on that creation, or starting a
   keyed task), on that call's thread before it returns, so another
   submitter's report of a drop (an async call's `RuntimeError`,
   `launch_pool_task`'s post, `compute_async`'s `on_error`) can run
   inside a `compute_async` call, an async call or `launch_pool_task`. A
   `submitPooled` task, and a job of `compute_async`'s form without
   `on_error` (it has no way to report a drop), waits in the queue
   instead, until a later submission or `setThreadPoolSize()` starts a
   worker

## Notes

- `submitPooled(std::function<void()>, priority)` is `startThreadPooled` with
  `wait=false` without the key argument; it returns `false` only when the
  app is being destroyed and the task was dropped unrun
- `trySubmitPooled(std::function<void()> &task, priority)` is `submitPooled`
  for a submitter that must hear of a task no worker will run. It returns a
  `PoolSubmit`: `queued` (the pool has it; `task` is left empty), `stopped`
  (the app is being destroyed: the task was dropped unrun, as by
  `submitPooled`) or `no_worker` (no worker could be started and the pool
  has none: the task is handed back in `task`, unrun). A task it queued is
  dropped later if the pool is left with no worker while it waits (item 6).
  `compute_async`, the exec async host and `launch_pool_task` submit
  through it (`compute_async`'s form without `on_error` has its job handed
  back, but never dropped later: item 6)
- `app::poolDropReason()` is why the pool is destroying a task unrun on the
  calling thread (`app::kPoolStopped` or `app::kPoolNoWorker`), for a
  task that reports its drop from its destructor
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
