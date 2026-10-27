# state_exec: exceptions and long-running native calls

Status: P0 specification (2026-10-18); P1 (#706), P2 (#800) and P3 are implemented. The decisions below
are the recommended defaults, agreed before this was written. Section 7 records how the five questions
P0 left open were settled, and one decision added since: async calls are charged to the process
(2026-10-19).
Tracked in the `state-exec-exceptions` workstream.

Two related pieces of the state_exec DSL:

1. **Exceptions.** A program can raise an exception, define its own exception classes by
   deriving from the built-in ones, and catch them by class. Errors from native C++ functions,
   Python functions and asynchronous work arrive as exceptions too, with their origin, so the
   program can handle them. An exception that isn't caught leaves the program as a typed value
   that C++, Python and Ariadne can inspect and catch by class.
2. **Long-running native calls.** A native function (C++ or Python) that does slow work, such as
   a volume filter, a mesh build or a network request, runs on a host thread pool. The call
   site waits the way a `msg-recv` waits: the calling process parks, every other process keeps
   running, and the call resumes with the function's value or raises its error.

Async failures should arrive as exceptions, so exceptions come first.

## 1. What exists today

**Errors.**
- The DSL has no `raise`, `try`, `catch` or `finally`, and no builtin that raises.
- Every failure is a C++ exception that unwinds the whole process:
  - the async scheduler kills the process and keeps the message as `exit_error`, a string;
  - the sync scheduler throws it out of `run()` and leaves the process `running`;
  - `pycvc.Exec.run` raises `RuntimeError(message)`; only a syntax error has a type of its own,
    `pycvc.parse_error`;
  - Ariadne logs the message.
- Type information is lost at every boundary. A Python exception in a registered function
  becomes the text `python DSL fn 'f': division by zero`, without the Python type. A C++
  `std::out_of_range` becomes its `what()`.
- Asynchronous work reports failure as a value, `{"__async_error__": message}`, which the
  program has to check by hand.

**Classes.**
- `(defclass Name [Parent] (method (self args...) body...)...)` binds `Name` to a constructor
  and keeps the class dict at `Name.__cls_dict__`. The class dict has the methods, the
  inherited `init` and `finalize`, and `__super__` (the parent's dict) for single inheritance.
- An instance is a dict whose `__class__` is the class dict.
- Methods are closures taking `self`, called with `send`; `super` calls the parent's.
- A class dict doesn't record the class's name, and there's no `isinstance`.

**Long-running calls.** `cvc/core/async_task.h` has the substrate:
- `launch_pool_task`, `park_on_pool_task` and `future_pool_task` run a kernel on the app's
  compute pool and post its value to a reply channel the calling process is parked on;
- `cvc/core/async_lane.h` does the same on a dedicated thread;
- futures, with `await` and `msg-recv` on them;
- pycvc's `Exec.register_async_fn` and Ariadne's `http-*-async` verbs are built on it.

What it lacks:
- **Registration:** there's no one-line way to make a C++ function async; every host writes
  the plumbing.
- **Where it parks:** only `async_scheduler` can park. A call that is the whole program can't,
  because the park returns a nil placeholder that delivery writes into the enclosing frame.
- **Errors** come back as the `__async_error__` dict.
- **No cancellation or timeout:** killing a parked process doesn't cancel its kernel, which
  runs to completion while its result is dropped, and a call can't have a timeout.
- **Executors:** a kernel runs on the app's background task pool (`compute_async` starts it
  there), with no choice of priority or lane through the common helpers.

## 2. Exceptions

### 2.1 The forms

**`(raise exc)`** raises `exc`, which must be an exception (an instance of `Exception` or a
class derived from it). Anything else raises `TypeError`. `(raise)` with no argument, inside a
`catch` handler, re-raises the exception being handled; elsewhere it raises `RuntimeError`.

**`(error message [data])`** is `(raise (RuntimeError message data))`.

**`try`:**

```lisp
(try
  body...
  (catch (Type var) handler...)   ; zero or more, tried in order
  (catch (Other) handler...)      ; var is optional
  (catch (var) handler...)        ; no type: catches any Exception
  (finally cleanup...))           ; optional, last
```

- `body...` runs as a `begin`. If it finishes, the `try`'s value is the body's.
- If it raises, the catch clauses are tried in order. A clause matches when the exception is an
  instance of `Type` or of a class derived from it. The first match runs its handler, with
  `var` bound to the exception in a new scope, and the `try`'s value is the handler's.
- With no match, the exception keeps unwinding.
- `Type` is looked up when a catch is tried, not when the `try` starts. An undefined `Type` is
  an error raised at that point (`NameError`), which replaces the exception being matched.
- In `(catch (X) ...)` the lone symbol is a class when `X` names one where the `try` is, and
  otherwise the variable of a catch-all.
- The handler's scope is its own: a `set` in it binds there, as in a `let` body.
- At least one `catch` or a `finally` is required.

`finally`'s `cleanup...` runs whenever control leaves the `try`:

- when the body or a handler finishes;
- when an exception leaves the `try`, whether no clause matched or a handler raised;
- when a `return` or `break` unwinds through it.

Its value is discarded. If `cleanup` itself raises, or runs `return` or `break`, that replaces
whatever was leaving the `try` (Python's rule). `finally` doesn't run when the process is killed
(2.5).

**`(isinstance x Type)`** is `#t` when `x` is an instance of `Type` or of a class derived from
it. It works for any class, not only exceptions.

### 2.2 Exception values

An exception is an ordinary instance, a dict whose `__class__` is its class dict, with these
fields:

| Field | Meaning |
|---|---|
| `message` | a string |
| `data` | any value the raiser attached (nil by default) |
| `cause` | the exception being handled when this one was raised (nil otherwise), set automatically |
| `origin` | where it came from: `"dsl"`, `"builtin"`, `"native"`, `"python"` or `"async"` |
| `traceback` | a list of strings, innermost last: the expression of each frame being evaluated at the raise (a call's starts with the function's name), up to 64, each at most 120 characters |

Origin-specific fields:
- `native_type`: the C++ exception's type, demangled, for `origin` `native`;
- `python_type`: the Python exception's qualified class name, for `origin` `python`;
- `function`: the name of the native or async function that failed.

`(Exception message [data])` constructs one; so do the derived classes. A user class derives
from any of them and can add fields and methods in its `init`:

```lisp
(defclass VolumeError RuntimeError
  (init (self message volume)
    (super self "init" message)
    (set-attr self "volume" volume)
    self))

(try (load-volume path)
  (catch (VolumeError e) (log (get-attr e "volume")))
  (catch (PythonError e) (log (get-attr e "python_type"))))
```

`defclass` adds `__name__`, the class's name, to every class dict, so an exception's class can
be named in messages and across boundaries.

### 2.3 The built-in classes

All are bound in every default environment.

| Class | Parent | Raised by |
|---|---|---|
| `Exception` | | the root; `catch` without a type catches it |
| `RuntimeError` | `Exception` | `error`; a native failure with no better class |
| `TypeError` | `Exception` | a value of the wrong type; calling a non-function; the wrong number of arguments; a special form used with the wrong shape |
| `ValueError` | `Exception` | a well-typed but invalid value (`(int "abc")`), `std::invalid_argument` |
| `NameError` | `Exception` | an undefined symbol |
| `KeyError` | `Exception` | a missing dict key or attribute, where the operation requires one |
| `IndexError` | `Exception` | an index out of range, `std::out_of_range` |
| `ArithmeticError` | `Exception` | division by zero, overflow |
| `MemoryError` | `Exception` | a native call refused by the per-call allocation limit (`eval_alloc_check`) |
| `ParseError` | `Exception` | `eval` or `parse` of bad source; adds `line` and `col` |
| `TimeoutError` | `Exception` | an async call or `await` that ran past its timeout |
| `NativeError` | `RuntimeError` | a C++ exception of any other type; `native_type` names it |
| `PythonError` | `RuntimeError` | a Python exception from a registered Python function; `python_type` names it |

Mapping a C++ exception thrown by a native function:
- `cvc::state_exec::dsl_exception` carries a class already;
- `std::invalid_argument`, `std::out_of_range` and `std::bad_alloc` become `ValueError`,
  `IndexError` and `MemoryError`;
- `cvc::state_exec::parse_error` becomes `ParseError`;
- everything else becomes `NativeError`.

The evaluator's and the builtins' own errors (an undefined symbol, a wrong argument count, a
type mismatch, an index or key error, division by zero) are thrown with their class (P1 changes
the throw sites). Their messages stay as they are.

### 2.4 Where an exception is caught

A raise unwinds the process's frame stack to the nearest `try` frame. That frame runs its
catches and `finally`, and if it doesn't handle the exception, the unwinding goes on. These
rules apply on the way:

- **Function calls:** a `try` in a caller catches what a callee raises. A `return` unwinding
  through a `try` with a `finally` runs it.
- **Native boundaries (`call_function`):** a DSL function that a native function (or a
  deep-data job) calls runs as frames above the native call. An exception it doesn't catch
  leaves the native call too: the job is abandoned and the native call raises the exception
  at its own call site, as if it had thrown it. A `try` below the native call catches it.
- **Generators:** an exception raised in a generator body ends the generator and raises at the
  `next`, `for` or job that advanced it.
- **Signal and watch handlers** run as if called at the point they interrupted. An exception a
  handler doesn't catch is raised in the interrupted program at that point, so the program's
  own `try` can catch it.
- **`eval` and macro expansion:** an exception raised while expanding or evaluating propagates
  like any other. `eval` of unparsable source raises `ParseError`.
- **Kills don't unwind** (2.5): no handler or `finally` runs.

### 2.5 What can't be caught

A process ended by its host or its budget is killed. It is not raised to:

- `max_steps`, `max_time`, `max_step_time`, `max_memory`, the message limits, including what
  the process's async calls are charged against them (3.8);
- `kill()`, `kill_owner()`, `SIGKILL`;
- `interrupt()` and an `evaluate()` timeout.

Nothing in the program runs after a kill, and its status is `killed` with `exit_error` naming
the reason, as today. A program must not be able to catch its own budget or its host's
decision to stop it. The per-call allocation limit is different: it refuses one call, so it is
the catchable `MemoryError`.

### 2.6 Leaving the program

An exception no `try` catches ends the process, with a new status, **`failed`**. `killed` stays
for 2.5. The process keeps:

- `exit_exception`: the exception value;
- `exit_error`: `"Class: message"`, as now for anything that ends a process.

What each host sees:

- **C++:**
  - `process_info` gains `exit_exception`, and `get_exit_exception(pid)` returns it;
  - a new `cvc::state_exec::dsl_exception` (derived from `std::runtime_error`, `what()` is
    `"Class: message"`) carries the value and its class chain;
  - `stackless_evaluator::run`/`evaluate` and `evaluator::evaluate` throw it to their caller;
  - both schedulers end the process `failed`, and neither throws for a program's exception.
    Behaviour change: the sync scheduler's `run()`/`step()` used to throw and leave the process
    `running`.
- **Python (pycvc):**
  - `pycvc.dsl_error` (a `RuntimeError` subclass) is raised by `Exec.run`, the evaluators'
    `evaluate`/`run`/`step`, and `get_exit_exception`'s raise helper;
  - it has `.value` (the exception), `.dsl_class` (its class name), `.dsl_classes` (the chain,
    most derived first), `.origin` and `.traceback`;
  - `pycvc.dsl_class("Name")` returns the Python class for a DSL class name;
  - a raised exception's Python type derives from the class for every name in its chain, so
    `except pycvc.dsl_class("ValueError")` catches a DSL `(defclass MyError ValueError)`;
  - a `PythonError` that crosses back to Python unhandled re-raises the original Python
    exception, with the `pycvc.dsl_error` as its `__cause__`. That lets Python code catch its
    own exception types through a DSL program.
- **Ariadne:** a failed action or resident is reported with its class, message and traceback.

**As built in P2:**
- **Not covered:** a syntax error found while a process reads its script (parser.step_tokens)
  isn't an exception the program raised. It stays as before: the sync scheduler throws its
  `parse_error` from `run()`, and the async scheduler kills the process with the message.
- **The schedulers:**
  - both have `get_exit_error(pid)` (the sync scheduler is new), `get_exit_exception(pid)` and
    `raise_exit_exception(pid)`;
  - `raise_exit_exception(pid)` throws a failed process's `dsl_exception`, with the C++
    exception it was made from, if any, as its `original()`, and returns for any other process;
  - `process_info` has `exit_error` and `exit_exception`, and `scheduler_stats` has `failed`.
- **The DSL:** `(inspect pid)` adds `exit_error` and `exception` once a process has ended that
  way, and `(scheduler-stats)` counts `failed`.
- **Hosts:** `dsl_error` takes an `origin` and fields of the exception's own, so a host can raise
  a class with its own origin. pycvc's error for a Python function's exception derives from it
  as a `PythonError`:
  - its origin is `"python"`;
  - `python_type` is the type's `module.QualName`, or the bare name of a builtin;
  - its message is `python DSL fn 'name': <str(exception)>`, as the error text was before.
- **Unchanged:** an exception a Python `on_complete` raises is not the program's. It is still
  thrown from the scheduler call that finished the process (PY-6).

### 2.7 Snapshots

A `try` frame and an exception value encode like any frame or value (`state_value_codec`).
The new frame phases are added by name, so older snapshots still decode. A process snapshotted
mid-unwind resumes the unwinding.

The codec writes a dict once for each value that holds it, so after a restore an exception's
`__class__` is a copy of the class dict the program names. `catch` and `isinstance` match such
a copy by its chain of class names (P2). Without that, an exception made before the snapshot,
re-raised after it, wouldn't be caught by its own class.

### 2.8 The recursive evaluator

`evaluator` (the reference evaluator) implements the same forms with C++ exceptions
(`dsl_exception` for a raise), so the two evaluators agree on every case in section 6 that
uses only forms it has. It records where an exception was raised in the first frame the
exception unwinds, rather than catching and rethrowing at each level: on MSVC a rethrow from a
handler runs on top of the throwing frame's stack, so that grew the stack with every level.

### 2.9 `return`

P1 also makes `return` leave only the function it is in on the stackless evaluator, as on the
recursive one. Before, `(+ (f) 1)` with `f` returning 5 was 5 there: the unwinding didn't stop
at the function's body and took the pending `+` with it. A function's body frame is marked
(`eval_frame::function_body`, kept in snapshots), and a `return` unwinds to it.

## 3. Long-running native calls

### 3.1 The model

An **async function** is a native function whose work runs on a host executor (a thread
pool), not in the evaluator step that called it:

1. **Before the work:** the call marshals its arguments on the scheduler's thread.
2. **While it runs:** the kernel runs on an executor thread. The calling process parks, and the
   scheduler goes on stepping every other process.
3. **After:** the kernel's value, or its exception, is posted back through the scheduler's
   thread-safe ingress. The process resumes at the call site with the value, or the exception
   raises there.

From the program's side, `(volume-blur v 3)` is an ordinary call that takes a while.

### 3.2 Registering

C++ (sketch; the names are settled in P3):

```cpp
// The kernel gets a handle to check for cancellation and post progress; it returns the value
// or throws.
using async_kernel = std::function<value_t(async_call &call)>;
// Runs on the scheduler thread with the call's arguments; returns the kernel to run.
using async_marshal = std::function<async_kernel(std::span<const value_t> args)>;

struct async_options {
  executor where = executor::background;     // the app's background task pool, or a named lane
  thread_priority priority = PRIORITY_NORMAL; // its place in the background pool's queue
  std::optional<double> timeout;             // seconds; past it the call raises TimeoutError
  bool future_variant = true;                // also register "<name>-async", which returns a future
};

void register_async_fn(environment_ptr env, async_host &host, const std::string &name,
                       async_marshal marshal, async_options opts = {});
```

`async_host` is what a host provides:
- the executors;
- the scheduler that drives the processes;
- in-flight tracking, so teardown waits for running kernels.

`app` builds one for its exec scheduler (Ariadne), and pycvc's `Exec` builds one for its
private scheduler.

Python: `pycvc.builtins.register_async_fn(env, host, name, callable, executor="compute",
timeout=None)`, with `Exec.async_host()`. `Exec.register_async_fn` keeps working, built on this.
The callable runs on an executor thread and takes the GIL for its call, as now.

### 3.3 Calling

- **Transparent:** `(name args...)` parks the calling process and resumes with the value. It
  works at any depth, including as the whole program (3.9 removes the top-level restriction),
  inside a `try`, and inside a function a native function called.
- **Futures:** `(name-async args...)` starts the work and returns a future at once.
  - `(await f)` parks until it finishes, then returns its value or raises its exception.
  - `(await f seconds)` raises `TimeoutError` if it hasn't finished by then; the work goes on,
    and the future can be awaited again.
  - `(await-all (list f1 f2 ...))` waits for every one and returns their values in order. If any
    failed, it raises the first failure in list order once all have finished.
  - A future that is never awaited still finishes; its result is dropped.
- **Where it can't park:** a context that forbids parking raises `RuntimeError` naming the
  function. Ariadne's reactive and render lane is one; registering through the action seam
  only, as today, keeps async functions out of it.

### 3.4 Errors

A kernel that throws resumes the call by raising. The exception is mapped as in 2.3, with
`origin` `"async"` and `function` set:

- a C++ exception maps by its type;
- a Python exception becomes `PythonError` (keeping the Python exception);
- a timeout raises `TimeoutError`.

The `{"__async_error__": ...}` value is gone: P3 removes it from `Exec`, and the Ariadne
documentation is updated. A function whose result describes a failure, such as an HTTP reply
with `ok: #f`, still returns that as a value; only a thrown error is an exception.

### 3.5 Where the work runs

The app has two pools, and an async kernel uses the first:

- **The background task pool** (`app::startThreadPooled`, which `compute_async` uses today):
  - a queue of whole tasks, with priorities (FIFO within one), run by at most
    `setThreadPoolSize` persistent worker threads (the core count by default); an async kernel
    is an anonymous task (`app::submitPooled`), so a call creates no thread and its task is
    destroyed on the worker once it ends;
  - every async kernel runs here, CPU-bound or blocking (`executor::background`, the default);
  - a blocking request holds one of its threads while it waits, and nothing else.
- **The fork-join compute pool** (`app::computePool()`):
  - render, simulation and nav fan-outs share it through `parallel_for`, and fan-outs from
    several threads (several kernels) run side by side on its workers;
  - a CPU-heavy kernel fans out through it the same way (a volume filter splitting its
    slabs), so a kernel's parallelism comes from the same workers as everyone else's;
  - a kernel never runs on its workers as a whole task, so a slow kernel can't stall a frame's
    `parallel_for`.

So blocking I/O doesn't get a pool of its own:
- the compute pool never sees it;
- in the background pool it only takes a thread out of the rotation.

The cost is that enough slow requests in flight at once can delay other async kernels until a
thread frees. If that shows up in practice, the fix is a cap on how many blocking kernels run at
once, in the same pool (a registration option), or a larger pool. A second pool isn't needed.

Other places a kernel can run:
- **A named `async_lane`:** for work that must stay on one thread.
- **No executor** (the sync scheduler, a bare evaluator, a unit test's inline executor): the
  kernel runs inline in the step. The value or exception is the same, and the step blocks.

### 3.6 Cancellation and lifetime

`async_call::cancelled()` turns true when:

- the process is killed or reaped, including by running out of `max_time` or `max_steps` while
  the call runs (3.8);
- the call times out;
- its owner group is reaped (`kill_owner`);
- the host is torn down.

A kernel checks it at convenient points and may stop early by throwing. A cancelled call's
result is dropped. Host teardown (an `Exec`, an Ariadne document, the app's scheduler) cancels
every in-flight kernel and waits for them before the scheduler goes, as `Exec` does today.

### 3.7 Data crossing threads

- **Arguments:** the marshal step copies what the kernel needs. DSL lists and dicts are copied,
  because the kernel must never touch a value the program can still change. A large copy runs
  as a deep-data job, so it counts against the process's budget like other deep-data work.
- **Large data:** a volume or mesh crosses as a `data_object` holding a `shared_ptr` the kernel
  treats as read-only. A kernel that modifies data works on its own copy.
- **The kernel must not touch:** the state tree, the evaluator, or any DSL value. The evaluators
  and the schedulers check it on every build (3.11). Its result is a fresh value.

### 3.8 Budgets

An async call is charged to the process that made it, as if the process had done the kernel's
work itself. A call can't be a way around a process's budget: a host that limits a process
limits what that process can make the app do for it.

**What a call is charged.**
- **Run time:** the kernel's wall time, from when it starts on its executor thread to when it
  returns or throws. Time waiting in the pool's queue for a thread isn't charged: that's the
  host's congestion, as a descheduled step's stall is.
- **CPU time:** the CPU seconds of the kernel's thread, plus the work it fans out through
  `computePool().parallel_for`. The compute pool carries the call's charge account into the
  chunks it runs for the kernel (a thread-local), and each chunk adds its thread's CPU time.
- **Steps:** the CPU time converted at `limits.async_steps_per_cpu_ms` steps per CPU
  millisecond, at least one step per call.
  - The rate is a state-tree setting, read live: the host reads it at every charge, so a
    change applies to the next charge, including calls already running, with no
    `load_settings()` or resubmission. The first of these that is set wins:
    1. the process's own override, `execute_options::async_steps_per_cpu_ms` (unset by
       default);
    2. `state_exec.schedulers.<id>.limits.async_steps_per_cpu_ms`;
    3. `state_exec.defaults.limits.async_steps_per_cpu_ms`;
    4. the built-in default, 2000. P3 measured what a step costs and set it from that: about
       0.4 to 0.45 CPU microseconds on the async scheduler (Ariadne's and `Exec`'s) and 0.26
       to 0.3 on the sync one, on a loop of arithmetic, DSL function calls and list and dict
       work (MSVC Release, x64; `StepCostSetsTheDefaultRate` in
       `state_exec_async_calls_test`, which, in an optimized build, fails if the default
       drifts beyond a factor of four of either; a debug build's steps cost 20 to 40 times
       more). 2000 rounds the async scheduler's 2,200 to 2,500 steps per CPU
       millisecond down. (The first draft's 1000 was provisional.)
  - The value is a non-negative decimal number; a fraction is allowed, `0.5` being a step every
    2 CPU ms. A node that isn't one falls through to the next level.
  - The scheduler publishes the effective value under its own subtree, as it does its other
    settings.
  - `0` converts no CPU time to steps; each call still counts its one step.
  - The steps are a new `step_kind::async`: they count in `step_count`, and so against
    `max_steps`, next to the eval, read and data steps.

**Measuring it.**
- **The thread CPU clock:** CPU time comes from `thread_cpu_seconds()`.
  - On Linux and macOS it reads `CLOCK_THREAD_CPUTIME_ID`.
  - On Windows it falls back to the steady clock today, so it reads wall time. A kernel that
    sleeps would then be charged its sleep as CPU time, and its steps.
  - P3 gives it a Windows implementation:
    - `QueryThreadCycleTime` for the thread's cycles, converted to seconds at the cycle
      counter's rate, measured once at startup;
    - `GetThreadTimes` (100 ns units, updated at the clock tick, about 15.6 ms) where the cycle
      counter can't be used.
  - That also corrects the CPU half of `max_time` for ordinary steps on Windows, which charges
    wall time today (section 5).
- **Python kernels:** the time a Python kernel waits to take the GIL isn't charged as run time.
  - It's the host's congestion, like time in the queue.
  - The host times the acquire and takes the wait off the call's run time.
  - CPU time needs no adjustment: a thread waiting for the GIL uses none.

**How it meets each budget.**
- **`max_time`:** it is charged like a time budget, as now: running time for the first half,
  then CPU time for the second.
  - A call's run time adds to the first half (`accumulated_time`), and its CPU time to the
    second (`cpu_time_charged`). A call that crosses the halfway point charges the part of its
    run time past it as the same fraction of its CPU time.
  - A call's CPU time is charged in full. The per-step cap (`max_time / 32`) keeps a stalled
    step from running a process out of time, and CPU time doesn't count a stall.
- **`max_steps`:** its steps, as above.
- **`max_memory`:** the kernel runs under the process's allocation limit.
  - The host arms an `eval_alloc_guard` on the executor thread with the limit the calling step
    had: `max_alloc_bytes`, tightened to what is left of `max_memory`. The compute pool carries
    it into the kernel's fan-out chunks, as it carries the charge account.
  - An allocation the kernel reports through `eval_alloc_check`, or through the natives it
    uses, is refused past that limit. The call then raises `MemoryError` at its call site. That
    is a per-call refusal, which a program can catch (2.5), as for a step.
  - What the kernel reports allocating is added to the process's `heap_alloc_bytes`, so its
    next heap measurement comes due sooner. The measurement counts the result once it's back.
  - An allocation the kernel doesn't report (a plain `std::vector`) isn't seen.
    `register_async_fn`'s documentation tells kernel authors to report large allocations with
    `eval_alloc_check`.
  - A `data_object` passed in (3.7) is shared, not copied, so it isn't charged again.
- **`max_step_time`** doesn't apply. It protects the host's drain from a step that never ends,
  and an async call doesn't hold a step. A kernel's own limit is its `timeout` option.
- **Several calls in flight** (futures): each is charged its own run time, CPU time and steps,
  so a process can be charged more run time than the wall clock shows. It used that many
  threads' worth.
- **Inline** (no executor: the sync scheduler, a bare evaluator): the kernel is charged as an
  async call, its full CPU time and steps, and not as part of its step's capped charge.

**When it's charged, and what happens at the limit.**
- **While the call runs**, the host charges as it goes:
  - it advances the call's run time on the scheduler's clock;
  - `async_call::cancelled()` doubles as a meter: each call to it, on the kernel's thread,
    records the thread's CPU time so far;
  - fan-out chunks add their CPU time as they finish.
- **At a limit:** when the charges reach `max_time` or `max_steps`, the process is killed with
  `time_limit_exceeded` or `max_steps_exceeded`, as for a step. It's a kill (2.5): no handler or
  `finally` runs. The kernel sees `cancelled()`, and its result is dropped when it arrives.
- **A kernel that never checks `cancelled()`** is still stopped by its run time, which the host
  measures without it. Its CPU time is known when it ends, and is charged then.
- **When the call finishes**, the rest is charged before the process resumes, so a call that
  used up the budget kills the process instead of resuming it.

**Reporting.** `process_info` gains:
- `async_calls`: calls made;
- `async_time`: their run time;
- `async_cpu_time`: their CPU time;
- `async_steps`: the steps charged for them.

`elapsed_time` includes the run time.

**Snapshots.** The counters are saved and restored with the process, along with
`accumulated_time` and `cpu_time_charged`.
- A call in flight at a snapshot keeps what the host had charged it so far.
- After the restore, the call can't resume (3.9), so nothing more is charged for it.

Time a process spends parked for other reasons (`sleep`, `msg-recv`, awaiting a frame) is still
not charged.

### 3.9 The call's frame

The call's frame stays on the stack in a new `await_call` phase, holding the reply channel,
instead of returning a nil placeholder for delivery to write into the enclosing frame. Delivery
completes the frame, with the value or by raising.

That:
- makes a top-level call parkable;
- gives an exception a precise site;
- lets a snapshot record a pending call. A restored process whose call can't resume raises
  `RuntimeError`.

### 3.10 Progress

`async_call::post(value)` sends a progress value to the channel `<name>.progress` under the
process's root, which the program reads with `msg-recv` or a resident watches. Nothing waits on
it; a full queue drops progress values first.

### 3.11 As built in P3

The tests are `src/cvc/tests/state_exec_async_calls_test.cpp` (AS-1..33 in both evaluators and
on both schedulers where they apply), `bindings/pycvc/test_pycvc_state_exec_async.py` (the same
from Python, with AS-4, AS-16 and AS-32), and `AriadneAsyncFn` in `ariadne_runtime_test.cpp`
(AS-17 through a document). The gtest file also passes under MSVC's AddressSanitizer
(`/fsanitize=address`), so the teardown in AS-9 and AS-10 frees nothing a kernel still uses.

**The names (3.2).**
- `cvc/state/state_exec/async_call.h`: `async_kernel`, `async_marshal`, `async_call` (for kernels:
  `cancelled()`, `post(value)`, `add_wait(seconds)`), `executor` (`background()`,
  `on_lane(name)`, `inline_call()`), `async_options` (`where`, `priority`, `timeout`,
  `future_variant`).
- `cvc/state/state_exec/async_host.h`: `async_host` (`async_host(scheduler_base *)` runs kernels
  inline; `async_host(async_scheduler &, app *)` parks; `add_lane`, `shutdown`, `wait_idle`,
  `in_flight`; `begin_external`/`finish` for a call whose outcome the host supplies),
  `register_async_fn(env, host, name, marshal, opts)`, `make_async_future`.
- `app::exec_async_host()` is the app's host for `exec_scheduler()`; `~app` shuts it down first.
- pycvc: `pycvc.async_host()` (inline, no scheduler), `pycvc.async_host(scheduler)`,
  `pycvc.async_host(async_scheduler, app)`, `Exec.async_host()`, and
  `pycvc.builtins.register_async_fn(env, host, name, fn, executor="background", timeout=None,
  future_variant=True, with_call=False)`. pycvc takes no keyword arguments, so these are
  positional. `executor` is `"background"` (`"compute"`, the sketch's name, means the same),
  `"inline"` or `"lane:<name>"`, a `pycvc.async_lane` given to `host.add_lane(name, lane)`.
  With `with_call` (also `Exec.register_async_fn(name, fn, with_call)`), `fn` gets the call's
  `pycvc.async_call` (`function_name()`, `cancelled()`, `post(value)`, `add_wait(seconds)`,
  `finished()`) before the arguments. `pycvc.is_async_future(v)` too. `begin_external`/`finish`
  and `make_async_future` aren't bound: an external call's outcome comes from C++ host code
  (`register_async_coro` is pycvc's), and a future is made by `(name-async ...)`.

**Calling (3.3, 3.9).**
- A native the stackless evaluator applies at its own level, in the outermost evaluator step on
  the thread, may park (`can_await()`, `request_await()`); its frame takes the `await_call`
  phase. Elsewhere (a generator's body, a native function's own call, the recursive evaluator, a
  host with no parking scheduler) the kernel runs inline. A job's call of an async function parks
  too: its frame goes on top, as a DSL function the job called would.
- A future of an async call is `{"__future__": channel, "__async_call__": handle}`. `await` reads
  the call directly, so a future can be awaited more than once, and a completion is never queued
  as a message. `(msg-recv f)` on such a future awaits it. `await` on a channel future (a lane's,
  `future_pool_task`'s) parks in the `await_call` phase too, so it takes a timeout.
- `await-all` takes these futures, channel futures and plain values, and waits for one at a time.
  A channel future's message (one already queued on its channel, or the next) takes the future's
  place in the call's list, as `{"__awaited__": message}`, and the call is applied again; so a
  snapshot keeps the messages it has, and a restored process waits again for the rest. A message
  that is itself a future is the value, not waited for in turn. A channel future where it can't
  park raises `RuntimeError`.
- An `await_call` frame waiting on a channel that is stepped before its message comes (a process
  restored from a snapshot) applies its call again, and it waits again. One waiting on an async
  call raises `RuntimeError`, since the call is gone.
- The arguments are copied before the marshal step runs (`make_copy_job`), as deep-data work.

**Data crossing threads (3.7).** The host marks the threads running a kernel: its executor thread,
its fan-out's chunks, and the step an inline kernel runs in (`in_async_kernel()`). There
`evaluator::evaluate`, `stackless_evaluator::step`, both schedulers' `execute`, `receive_message`,
`deliver_to_receivers` and `kill`, `scheduler::step` and `async_scheduler::drain_ingress` throw
`std::logic_error`, which raises at the call. The check is on in every build: the mesher defines
`NDEBUG` for every build type, so a debug-only assert would never fire. An inline kernel's `post()`
delivers as the host, not the kernel. The state tree isn't checked: Ariadne's `fetch` of a
`state://` URL reads it on its kernel, by design, pinning the nodes it reads.

**Charging (3.8).**
- While a kernel runs, the run time since it last recorded its CPU time (`cancelled()`) counts as
  CPU time, so a kernel that never checks is stopped by its run time; its true CPU time replaces
  the estimate when it ends. Steps come from recorded CPU time only.
- An inline call's run time is inside its step's slice of `accumulated_time`; its CPU time and
  steps are charged as the call's, and the step's own (capped) CPU charge and time by kind leave
  them out. A call that runs out of the budget drops its outcome, and the scheduler kills the
  process when the step is over.
- `async_scheduler::drain_ingress()` takes completions, charges the calls still running, applies
  registration timeouts, kills at the limits, and wakes due sleepers and timed waits, so a driver
  that only drains while nothing is runnable (pycvc's `Exec`) keeps budgets and timeouts current.
- The effective rate is published at `state_exec.schedulers.<id>.effective.async_steps_per_cpu_ms`,
  not at the scheduler's `limits.` key: that one is an input with precedence over the defaults,
  so writing the effective value there would pin it and break the live reading. The async
  scheduler gains `set_id()` for this.
- `process_info` also gains `heap_alloc_bytes` (what natives and kernels reported allocating
  since the last heap measurement), and `(inspect pid)` and `(budget-usage)` add the four
  `async_*` counters.
- A Python kernel registered without `with_call` has no handle to the call, so it can't check
  `cancelled()` or `post()`; its run time stops it, as for any kernel that doesn't check. A
  coroutine (`register_async_coro`) is an external call: its run time isn't charged, only its
  one step.
- The Windows thread CPU clock calibrates the time-stamp counter's rate against the performance
  counter once, over 5 ms, where the processor reports an invariant TSC.

**Snapshots.** `encode_process`/`decode_process` (the codec) snapshot a process with its budget
so far; `snapshot(pid, node)` and `restore(node, natives)` on both schedulers wrap them, and
`rebind_natives` binds the native functions again (natives aren't serialized). A handle to an
async call decodes empty, so a restored call or future raises `RuntimeError`.

**Ariadne.** `http-get`, `http-request` and `fetch` (with their `-async` forms) are async functions
on `app::exec_async_host()`. A failed send is still a reply with `ok` `#f`, as they document. The
reactive lane arms `park_forbidden_scope`. `launch_pool_task` and the lane helpers stay for the
hosts that use them. After P3:
- `AppRuntime::register_async_verb` (and `AriRuntime`'s, and pycvc_gl's) registers an async
  function on the app's host, so a host verb parks at the call, raises there and is charged to the
  action; the done-channel form also posts its result. The nav_compute example's verb is
  `(nav-step)`, an async function.
- An action's step and time caps are the tree settings `ariadne.limits.action_max_steps` (5000)
  and `ariadne.limits.action_max_time` (0.1 s), read on every submission, `0` = no limit.

## 4. Phases

| Phase | Content | Done when |
|---|---|---|
| P0 | This spec and the test list (section 6) | merged |
| P1 | The forms (`raise`, `error`, `try`/`catch`/`finally`, `isinstance`), `__name__` on classes, the built-in classes, the evaluator's and builtins' errors thrown with their class, C++ exception mapping, `cause`/`origin`/`traceback`, all of 2.4 in the stackless evaluator, the same in the recursive evaluator, codec support | EX and UN cases pass in both evaluators |
| P2 | Leaving the program: `failed`, `exit_exception`, `dsl_exception`, both schedulers, Ariadne reporting; `PythonError` from registered Python functions, pycvc `dsl_error` and `dsl_class`, the round trip | CX and PY cases pass |
| P3 | Async: `async_host`, `register_async_fn` (C++ and pycvc), the `await_call` frame, futures with timeouts and `await-all`, errors as exceptions, executors (the background pool with priorities, lanes, inline), cancellation, progress, charging calls to the process's budgets (3.8), `Exec` and Ariadne moved onto it | AS cases pass (done: 3.11) |
| P4 | pycvc parity for the new API, `docs/STATE_EXEC_DEVELOPER_GUIDE.md` and `ARIADNE.md` | the parity doc lists every new item |

## 5. Compatibility

These are the behaviour changes, each covered by a case in section 6:

1. The sync scheduler no longer throws a program's error out of `run()`/`step()`. The process
   ends `failed` with `exit_exception`. Code that relied on the throw reads `get_exit_exception`
   or `process_info`.
2. A limit or an interrupt is a kill, not an exception, on both schedulers. A C++ test that
   expects `run()` to throw for a time limit is updated.
3. The `{"__async_error__": ...}` dict is replaced by a raise at the call site.
4. A process that fails ends `failed`, not `killed`. Hosts that check `killed` to mean "ended
   badly" check both.
5. `process_status` gains `failed`. The pycvc `IntEnum` and Ariadne's reporting follow.
6. **Async calls are charged to the process (3.8).** Today a parked process uses none of its
   budget.
   - A process whose calls block, such as a resident making slow HTTP requests, now uses
     `max_time` while it waits for them.
   - A process whose calls compute now uses `max_time` and `max_steps` for the work.
   - Hosts that set `max_time` or `max_steps` on such processes may need larger budgets.
7. **On Windows, the CPU half of `max_time` charges real CPU time** (3.8, the thread CPU clock),
   not wall time. A step that was descheduled is no longer charged for the stall there.

P1 also changed two things:
- On the sync scheduler, a process whose error nothing caught was killed, with the message as its
  `exit_error`, and `run()` still threw the error. Before, the process stayed `running`, and the
  next `run()` stepped it again. P2 replaced this with item 1.
- On the stackless evaluator, `return` leaves only the function it is in (2.9).

## 6. The test list

Each case runs in both evaluators and on both schedulers unless it says otherwise. P1 to P3 turn
them into gtest and pytest cases with these IDs.

### 6.1 Raising and catching (EX)

| ID | Program | Result |
|---|---|---|
| EX-1 | `(try (error "boom") (catch (e) (get-attr e "message")))` | `"boom"` |
| EX-2 | `(try (raise (ValueError "v")) (catch (TypeError e) 1) (catch (ValueError e) 2))` | `2` |
| EX-3 | `(begin (defclass MyError ValueError) (try (raise (MyError "m")) (catch (ValueError e) (get-attr (get-attr e "__class__") "__name__"))))` | `"MyError"` |
| EX-4 | `(try (raise (ValueError "v")) (catch (TypeError e) 1))` | uncaught `ValueError` |
| EX-5 | `(try 7 (catch (e) 1))` | `7` (no raise: the body's value) |
| EX-6 | `(begin (set log (list)) (try (error "x") (catch (e) (append log "c")) (finally (append log "f"))) log)` | `("c" "f")` (`append` adds to the list in place) |
| EX-7 | `(begin (set n 0) (try (try (error "in") (finally (set n 1))) (catch (e) n)))` | `1` (inner finally ran before the outer catch) |
| EX-8 | `(try (try (error "a") (catch (e) (raise))) (catch (e) (get-attr e "message")))` | `"a"` (bare raise re-raises) |
| EX-9 | `(try (try (error "a") (catch (e) (error "b"))) (catch (e) (get-attr (get-attr e "cause") "message")))` | `"a"` (cause chained) |
| EX-10 | `(try (error "a") (finally (error "b")))` | uncaught `RuntimeError "b"` (finally replaces) |
| EX-11 | `(begin (set box (dict "g" 0)) (defun f () (try (return 1) (finally (set-attr box "g" 2)))) (list (f) (get-attr box "g")))` | `(1 2)` (return runs finally; a `set` in `f` would bind in `f`'s scope) |
| EX-12 | `(begin (set g 0) (while #t (try (break 5) (finally (set g 9)))) g)` | `9` (not on the recursive evaluator, which has no `break`) |
| EX-13 | `(raise 5)` | uncaught `TypeError` |
| EX-14 | `(try (raise (Exception "m" (dict "k" 1))) (catch (e) (get-attr (get-attr e "data") "k")))` | `1` |
| EX-15 | `(isinstance (ValueError "v") Exception)`, `(isinstance 5 Exception)` | `#t`, `#f` |
| EX-16 | `(try (error "x") (catch (NoSuchClass e) 1))` | uncaught `NameError` (an undefined catch type replaces the exception) |
| EX-17 | `(try (error "x"))` | uncaught `TypeError` at the form: a `try` needs a `catch` or `finally` |
| EX-18 | `(raise)` outside a handler | uncaught `RuntimeError` |
| EX-19 | A `try` whose handler binds `e`, after the `try` | `e` is unbound (the handler's scope) |
| EX-20 | `(try (error "x") (catch (e) (get-attr e "traceback")))` | a list of strings, innermost last, at most 64 |

### 6.2 Errors from the evaluator, builtins and natives (UN)

| ID | Program | Caught as | Checks |
|---|---|---|---|
| UN-1 | `undefined-name` | `NameError` | `origin` `"builtin"` |
| UN-2 | `(begin (set f (lambda (x) x)) (f))`; `((lambda (x) x))` | `TypeError` | the arity message as today; a list in the head position is an invalid head |
| UN-3 | `(5 1)` | `TypeError` | not callable |
| UN-4 | `(int "abc")` | `ValueError` | |
| UN-5 | `(nth (list 1) 5)` | `IndexError` | |
| UN-6 | `(/ 1 0)` | `ArithmeticError` | |
| UN-7 | `(eval "(+ 1")` | `ParseError` | `line`, `col` |
| UN-8 | a C++ native throwing `std::invalid_argument` | `ValueError` | `origin` `"native"`, `function` |
| UN-9 | a C++ native throwing a custom `std::exception` subclass | `NativeError` | `native_type` names it |
| UN-10 | `(collections.range 1000000000)` | `MemoryError` | the allocation limit, catchable |
| UN-11 | a lambda, called through `apply`, that raises | caught by a `try` around the `apply` | the native boundary (2.4); not on the recursive evaluator, whose natives can't call a lambda |
| UN-12 | the same through `collections.map` with deep-data slicing on | caught; the job abandoned | |
| UN-13 | a generator body that raises, advanced by `next` | caught at the `next`; the generator then finished | |
| UN-14 | a method raising inside `send`, and inside `super` | caught by the caller | |
| UN-15 | a signal handler raising | caught by the program's `try` around the interrupted point | |
| UN-16 | a macro expansion that raises | caught | |

### 6.3 Not catchable (KL)

| ID | Situation | Result |
|---|---|---|
| KL-1 | `(try (while #t 1) (catch (e) 1))` with `max_steps` 100 | `killed`, `exit_error` names the limit, the catch didn't run |
| KL-2 | the same with `max_time` | `killed` |
| KL-3 | `kill(pid)` while inside a `try` with a `finally` | `killed`; the finally didn't run |
| KL-4 | `max_memory` exceeded inside a `try` | `killed` |
| KL-5 | `evaluate(..., timeout)` and `interrupt()` on a bare evaluator | `evaluation_timeout` / `evaluation_interrupted` to the C++ caller; no DSL handler runs |

### 6.4 Leaving the program (CX)

| ID | Situation | Result |
|---|---|---|
| CX-1 | uncaught `ValueError` on the async scheduler | status `failed`, `get_exit_exception` is the value, `exit_error` is `"ValueError: ..."` |
| CX-2 | the same on the sync scheduler | `run()` returns; the same status and values (the compatibility change) |
| CX-3 | uncaught on a bare `stackless_evaluator::evaluate` | throws `dsl_exception` with the value and the class chain |
| CX-4 | a user class `MyError` derived from `ValueError` | the chain is `MyError, ValueError, Exception` |
| CX-5 | snapshot taken inside a `try`, restored, then the body raises | caught by the restored `try` |
| CX-6 | Ariadne action raising | warning names the class and message |
| CX-7 | `failed` vs `killed` in `get_stats()` and `list_processes()` | counted apart |

### 6.5 Python (PY)

| ID | Situation | Result |
|---|---|---|
| PY-1 | a registered Python function raising `ZeroDivisionError`, caught in the DSL | `PythonError`, `python_type` `"ZeroDivisionError"`, `origin` `"python"` |
| PY-2 | the same, uncaught, through `Exec.run` | the original `ZeroDivisionError` re-raised in Python, `__cause__` a `pycvc.dsl_error` |
| PY-3 | a Python exception class defined by the caller, raised in a registered function, uncaught | `except` on that class catches it from `Exec.run` |
| PY-4 | DSL `(raise (MyError "m"))` with `MyError` derived from `ValueError`, through `Exec.run` | `except pycvc.dsl_class("ValueError")` catches it; `.dsl_class` is `"MyError"` |
| PY-5 | `pycvc.dsl_error` fields | `.value`, `.dsl_classes`, `.origin`, `.traceback` |
| PY-6 | an exception raised in a Python `on_complete` | unchanged: raised from the step, as today |

### 6.6 Async calls (AS)

| ID | Situation | Result |
|---|---|---|
| AS-1 | `(slow-op 2)` registered with a 50 ms kernel, two processes | both finish; the other process steps while one is parked |
| AS-2 | the call as the whole program | parks and returns the value (no top-level restriction) |
| AS-3 | a kernel throwing `std::invalid_argument`, call inside a `try` | caught as `ValueError`, `origin` `"async"`, `function` set |
| AS-4 | a Python async function raising | `PythonError`, `python_type` set |
| AS-5 | `(await (slow-op-async 2))` | the value |
| AS-6 | `(await (slow-op-async 2) 0.01)` on a 200 ms kernel | `TimeoutError`; awaiting again later returns the value |
| AS-7 | `(await-all (list (a-async) (b-async)))`, b failing | raises b's exception after both finish |
| AS-8 | registration timeout 0.05 s on a 1 s kernel | `TimeoutError` at the call site; the kernel sees `cancelled()` |
| AS-9 | the process killed while parked | `killed`; the kernel sees `cancelled()`; its result is dropped |
| AS-10 | host teardown with a kernel in flight | teardown waits; no use after free (run under the sanitizers) |
| AS-11 | a kernel blocked on a sleep while another thread runs `computePool().parallel_for` | the fan-out isn't delayed (kernels never occupy compute workers) |
| AS-12 | on the sync scheduler and a bare evaluator | the kernel runs inline; the same value and exceptions |
| AS-13 | progress posted by the kernel | `msg-recv` on `<name>.progress` receives it |
| AS-14 | a list argument changed by the program after the call starts | the kernel saw the copy |
| AS-15 | a snapshot of a process parked in a call, restored | raises `RuntimeError` at the call (the call can't resume) |
| AS-16 | `Exec.register_async_fn` raising | raises at the call site, no `__async_error__` dict (the compatibility change) |
| AS-17 | an async function called from Ariadne's reactive lane | not reachable (action seam only); if called, `RuntimeError` |
| AS-18 | a 200 ms CPU-bound kernel | `async_calls` 1, `async_time` and `async_cpu_time` about 0.2 s, `async_steps` about 400,000 at the default rate; `elapsed_time` includes the 0.2 s |
| AS-19 | `max_time` 0.1 s, a 1 s CPU-bound kernel that checks `cancelled()`, the call inside a `try` | `killed`, `time_limit_exceeded`, well before 1 s; the catch didn't run; the kernel sees `cancelled()` |
| AS-20 | `max_steps` 1000, a 50 ms CPU-bound kernel | `killed`, `max_steps_exceeded`; with `limits.async_steps_per_cpu_ms` 0, it finishes |
| AS-21 | a kernel that sleeps 200 ms (blocking, almost no CPU) | `async_time` about 0.2 s, charged to `max_time`; `async_cpu_time` near 0; a step or a few |
| AS-22 | a kernel fanning out over 4 compute workers, about 100 ms each | `async_cpu_time` at least 0.3 s, more than its run time |
| AS-23 | a background pool of one thread; a call waits 200 ms for it behind another process's call | its process's `async_time` is its own kernel's run time; the wait isn't charged |
| AS-24 | two futures in flight from one process, 100 ms each | `async_time` about 0.2 s, though about 0.1 s passed |
| AS-25 | a kernel that never checks `cancelled()`, past `max_time` | the process is killed when its run time reaches the limit; the late result is dropped |
| AS-26 | the AS-18 kernel inline (sync scheduler) | the same charges as AS-18, not the step's capped charge |
| AS-27 | two 50 ms kernels in a row; between them `state_exec.defaults.limits.async_steps_per_cpu_ms` is set to `0`, with no `load_settings()` | the first call is charged about 100,000 steps and the second one |
| AS-28 | the rate set to 10 in `state_exec.defaults.limits` and to 100 in `state_exec.schedulers.<id>.limits` | a 50 ms kernel is charged about 5,000 steps (the scheduler's node wins); with `execute_options::async_steps_per_cpu_ms` 1, about 50 |
| AS-29 | AS-21's sleeping kernel, on Windows | `async_cpu_time` near 0 and a step or a few, as on Linux and macOS (the thread CPU clock) |
| AS-30 | `max_memory` 1 MiB, a kernel that reports allocating 4 MiB through `eval_alloc_check`, the call inside a `try` | `MemoryError` at the call site, caught; `heap_alloc_bytes` grew by what the kernel reported before the refusal |
| AS-31 | a kernel fanning out whose chunk reports an allocation past the limit | the same `MemoryError` (the guard reaches the chunks) |
| AS-32 | a Python kernel that waits 200 ms for the GIL, held by another thread, then runs 50 ms | `async_time` about 0.05 s; the wait isn't charged |
| AS-33 | a process snapshotted after a 100 ms call, then restored | `async_calls`, `async_time`, `async_cpu_time` and `async_steps` are as before the snapshot |

## 7. Settled questions

P0 opened with five open questions. They were settled on 2026-10-18 (1 to 5), and one decision
was added on 2026-10-19 (6):

1. **`failed` is a new `process_status`** (2.6).
   - Alternative: `killed` with an exception attached.
   - Why: an error in the program and a stop by the host or a budget are different outcomes.
     A host restarts or reports the first and accepts the second, and `get_stats` and
     `list_processes` should count them apart. One enum value is cheaper than every host
     checking whether `exit_exception` is set.
2. **A signal handler's uncaught exception is raised in the interrupted program** (2.4), at
   the point it was interrupted, as Python does with signals.
   - Alternative: end the process.
   - Why: a handler runs as if called at that point, so its errors belong there. It lets a
     program turn a signal into a catchable exception (an interrupt it cleans up after) instead
     of losing the process. Watch handlers follow the same rule.
3. **The sync scheduler no longer throws a program's error out of `run()`/`step()`** (5.1).
   The process ends `failed` with `exit_exception`, as on the async scheduler.
   - Why: today it throws and leaves the process `running`, so the next `run()` throws again.
     The two schedulers should agree.
   - C++ that wants an exception can call `evaluate`, which throws `dsl_exception`, or read
     `get_exit_exception`.
4. **There's no separate I/O pool** (3.5).
   - Async kernels run on the app's background task pool, which is already separate from the
     fork-join compute pool frames use, bounded, and prioritized.
   - A blocking request only takes a background thread out of the rotation; the compute pool
     never sees it.
   - If slow requests are found to starve other kernels, a cap on concurrent blocking kernels
     in that pool comes first.
5. **`await-all` waits for every future, then raises the first failure in list order** (3.3).
   It doesn't cancel the rest or raise at the first failure in time.
   - Why: the error reported is then the same however the timing falls, which keeps programs
     and their tests deterministic, and no work is left running unseen.
   - A program that wants to stop early awaits the futures one by one.
6. **An async call is charged to the process that made it** (3.8; settled 2026-10-19). Its
   kernel's run time and CPU time count against `max_time`, and its CPU time, converted to
   steps, counts against `max_steps`.
   - This replaces the first draft, under which the executor's time was the host's and charged
     to no process.
   - Why: a call can't be a way around a process's budget. The kernel does work the program
     asked for, in a process the host bounded.
   - Also settled the same day:
     - the process's allocation limit follows a kernel onto its threads (`max_memory`);
     - a Python kernel's wait for the GIL isn't charged;
     - the charges survive a snapshot;
     - Windows gets a real thread CPU clock.
