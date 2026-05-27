# state_exec Developer Guide

## Overview

`state_exec` is a sandboxed S-expression DSL engine built on top of
`cvc::state`.  Programs execute as lightweight processes managed by a
scheduler, with full support for process control, inter-process messaging
via the shared state tree, distributed execution across nodes, and process
migration.

This guide walks through every major subsystem with working C++ examples
you can drop directly into a test or application.

---

## Table of Contents

1. [Quick Start](#1-quick-start)
2. [The DSL Language](#2-the-dsl-language)
3. [Running Programs](#3-running-programs)
4. [Process Management](#4-process-management)
5. [The State Tree](#5-the-state-tree)
6. [Inter-Process Messaging](#6-inter-process-messaging)
7. [Resource Limits & Policies](#7-resource-limits--policies)
8. [Distributed Execution](#8-distributed-execution)
9. [Process Migration](#9-process-migration)
10. [Access Control Model](#10-access-control-model)
11. [Standard Library](#11-standard-library)
12. [Architecture Reference](#12-architecture-reference)

---

## 1. Quick Start

### Minimal "Hello World"

```cpp
#include <cvc/core/state_exec/scheduler.h>
#include <iostream>

using namespace cvc::state_exec;

int main() {
    scheduler sched;

    // Submit a program — returns a process ID
    int pid = sched.execute(std::string("(+ 40 2)"));

    // Run all processes to completion
    auto results = sched.run();

    // Retrieve the result
    auto& val = results[pid];
    std::cout << std::get<int64_t>(val.v) << "\n";   // prints 42
}
```

### Fibonacci with Named Process

```cpp
scheduler sched;

execute_options opts;
opts.name = "fibonacci";
opts.uid  = "alice";

int pid = sched.execute(std::string(R"(
    (begin
      (defun fib (n)
        (if (<= n 1) n
          (+ (fib (- n 1)) (fib (- n 2)))))
      (fib 10))
)"), opts);

auto results = sched.run();
// results[pid] == 55
```

---

## 2. The DSL Language

### Types

| Type | Literal examples | C++ variant tag |
|------|-----------------|-----------------|
| Integer | `42`, `-7` | `int64_t` |
| Float | `3.14`, `1e-5` | `double` |
| String | `"hello"` | `std::string` |
| Boolean | `true`, `false` | `bool` |
| Symbol | `x`, `my-func` | `symbol` |
| Nil | `nil` | `std::monostate` |
| List | `(1 2 3)` | `list_ptr` |

### Special Forms

| Form | Syntax | Description |
|------|--------|-------------|
| `set` | `(set x 42)` | Bind a variable |
| `defun` | `(defun name (params...) body...)` | Define a named function |
| `lambda` | `(lambda (x y) (+ x y))` | Anonymous function |
| `if` | `(if cond then [else])` | Conditional |
| `begin` | `(begin e1 e2 ... en)` | Sequence; returns last value |
| `while` | `(while cond body)` | Loop while condition is true |
| `for` | `(for x collection body)` | Iterate over a list or generator |
| `let` | `(let ((x 1) (y 2)) (+ x y))` | Local bindings |
| `quote` | `(quote (1 2 3))` | Return expression unevaluated |
| `return` | `(return expr)` | Early return from a function |
| `break` | `(break)` or `(break expr)` | Exit nearest `while`/`for` loop |
| `yield` | `(yield expr)` | Produce a value from a generator |
| `defmacro` | `(defmacro name (params) template)` | Macro definition |
| `eval` | `(eval expr)` | Evaluate a quoted expression |
| `defclass` | `(defclass Name ...)` | OOP class definition |
| `root` | `(root)` | Returns the root expression |
| `super` | `(super self method args...)` | Call parent class method |

### Built-in Functions

**Arithmetic:** `+`, `-`, `*`, `/`, `%`
**Comparison:** `<`, `>`, `<=`, `>=`, `=`, `!=`
**Logic:** `not`, `and`, `or`
**String:** `str` (convert to string), `str-concat`
**List:** `list`, `car`, `cdr`, `cons`, `nth`, `set-nth`, `length`, `append`, `slice`, `del-nth`
**Dict:** `dict`, `get-attr`, `set-attr`, `del-attr`
**Type:** `null?`, `list?`, `type-of`, `generator?`, `is-int`, `is-float`, `is-string`
**Conversion:** `int` (to integer), `float` (to float)
**I/O:** `print`
**Generator:** `generator`, `next`, `range`, `collect`, `gen-done?`
**OOP:** `send` (method dispatch), `apply`

#### Type Conversion

```lisp
;; Convert to integer — truncates floats, parses strings
(int 3.7)       ;; => 3
(int "123")     ;; => 123
(int true)      ;; => 1

;; Convert to float — widens integers, parses strings
(float 5)       ;; => 5.0
(float "3.14")  ;; => 3.14
(float true)    ;; => 1.0

;; Type predicates
(is-int 42)       ;; => true
(is-int 3.14)     ;; => false
(is-float 3.14)   ;; => true
(is-float 42)     ;; => false
(is-string "hi")  ;; => true
(is-string 42)    ;; => false
```

Conversion errors throw a runtime error (e.g., `(int "abc")` fails).

### Example Programs

```lisp
;; Factorial
(begin
  (defun fact (n)
    (if (<= n 1) 1
      (* n (fact (- n 1)))))
  (fact 10))
;; => 3628800

;; Map over a list using for-loop
(begin
  (set result (list))
  (for x (list 1 2 3 4 5)
    (set result (append result (list (* x x)))))
  result)
;; => (1 4 9 16 25)

;; Producer-consumer via state tree
;; Process 1 writes, Process 2 reads
(state-set "shared.counter" "0")
```

### Generators and Lazy Sequences

Generators produce values lazily — one at a time, on demand.  They are first-class
values and can be consumed with `(next gen)`, iterated with `(for x gen body)`,
or materialised with `(collect gen)`.

#### Creating Generators

**From a closure with `yield`:**

```lisp
;; A generator that yields 1, 2, 3
(set g (generator (lambda ()
  (yield 1)
  (yield 2)
  (yield 3))))
```

**From `range`:**

```lisp
(range 5)         ;; yields 0, 1, 2, 3, 4
(range 2 8)       ;; yields 2, 3, 4, 5, 6, 7
(range 0 10 3)    ;; yields 0, 3, 6, 9
```

#### Consuming Generators

```lisp
;; Pull one value at a time
(set g (range 3))
(next g)           ;; => 0
(next g)           ;; => 1
(next g)           ;; => 2
(next g)           ;; => nil (exhausted)

;; Check if exhausted
(gen-done? g)      ;; => true

;; Iterate with for
(for x (range 5)
  (print x))       ;; prints 0 1 2 3 4

;; Materialise all values into a list
(collect (range 5))  ;; => (0 1 2 3 4)
```

#### Generator Patterns

**Filter — yield only values matching a predicate:**

```lisp
(set evens (generator (lambda ()
  (for x (range 10)
    (if (= (% x 2) 0)
      (yield x) nil)))))
(collect evens)  ;; => (0 2 4 6 8)
```

**Map — transform each value:**

```lisp
(set squares (generator (lambda ()
  (for x (range 5)
    (yield (* x x))))))
(collect squares)  ;; => (0 1 4 9 16)
```

**Take-N with `break` — consume only the first N values:**

```lisp
(set taken (list))
(for x (range 1000)
  (if (>= (length taken) 4)
    (break nil)
    (append taken x)))
taken  ;; => (0 1 2 3)
```

**Accumulator — running sum:**

```lisp
(set running-sum (generator (lambda ()
  (let ((total 0))
    (for x (range 1 6)
      (begin
        (set total (+ total x))
        (yield total)))))))
(collect running-sum)  ;; => (1 3 6 10 15)
```

**Infinite-style with while + yield:**

```lisp
(set counter (generator (lambda ()
  (let ((n 0))
    (while (< n 1000)
      (begin (yield n)
             (set n (+ n 1))))))))
;; Consume lazily with break
(for x counter
  (if (= x 5) (break x) nil))  ;; => 5
```

### The `break` Statement

`break` exits the nearest enclosing `while` or `for` loop.

```lisp
;; Exit with no value (loop evaluates to nil)
(while t (break))

;; Exit with a value
(while t (break 42))  ;; => 42

;; Find first match
(for x (list 10 20 30 40)
  (if (> x 25) (break x) nil))  ;; => 30

;; Nested loops — break exits only the innermost
(set sum 0)
(for i (list 1 2 3)
  (for j (list 10 20 30)
    (if (= j 20) (break nil)      ;; exits inner for
      (set sum (+ sum j)))))
sum  ;; => 30  (10 + 10 + 10)
```

`break` unwinds through `begin`, `let`, `if`, and any other intermediate
forms — it always targets the nearest loop boundary.

---

## 3. Running Programs

### The Scheduler

The `scheduler` is the primary entry point.  It manages a collection of
processes, runs them cooperatively via time-slicing, and provides querying,
pausing, and resource-limit enforcement.

```cpp
#include <cvc/core/state_exec/scheduler.h>
using namespace cvc::state_exec;

// Choose a scheduling policy
scheduler sched(scheduling_policy::round_robin);   // default
scheduler sched2(scheduling_policy::priority);     // -20..19 nice values
scheduler sched3(scheduling_policy::priority_rr);  // priority + RR within tier
```

### Submitting Programs

```cpp
// From a string script
int pid = sched.execute(std::string("(+ 1 2)"));

// With options
execute_options opts;
opts.name        = "worker-1";
opts.uid         = "alice";
opts.gid         = "engineers";
opts.priority    = -5;      // higher priority (Unix nice style)
opts.max_steps   = 10000;   // kill after 10k evaluator steps
opts.max_time    = 5.0;     // kill after 5 seconds wall time
opts.max_memory  = 1048576; // 1 MB memory limit
opts.max_messages = 100;    // max 100 outbound messages
opts.on_complete = [](value_t result) {
    std::cout << "Done: " << to_string(result) << "\n";
};
int pid = sched.execute(std::string("(fib 20)"), opts);
```

### Running & Stepping

```cpp
// Run all processes to completion
auto results = sched.run();
// results is std::unordered_map<int, value_t>

// Run with a global step budget (partial execution)
auto results = sched.run(1000);        // at most 1000 total steps
auto results = sched.run(std::nullopt, 2.0);  // at most 2 seconds

// Single-step (one evaluator step for one process)
while (sched.has_runnable()) {
    sched.step();
}
```

### Checking Progress

```cpp
// Overall scheduler statistics
scheduler_stats stats = sched.get_stats();
// stats.total_processes, stats.running, stats.ready,
// stats.paused, stats.terminated, stats.killed, stats.total_steps

// List all processes
std::vector<process_info> procs = sched.list_processes();
for (auto& p : procs) {
    std::cout << p.pid << " " << p.name << " "
              << to_string(p.status) << " steps=" << p.step_count
              << " uid=" << p.uid << "\n";
}

// Query one process
auto info = sched.get_process_info(pid);
if (info) {
    std::cout << "Status: " << to_string(info->status) << "\n";
}

// Get result of a terminated process
auto result = sched.get_result(pid);
if (result) {
    std::cout << "Result: " << to_string(*result) << "\n";
}
```

---

## 4. Process Management

### Pause, Resume, Kill

```cpp
sched.pause(pid);    // suspends the process
sched.resume(pid);   // resumes it
sched.kill(pid);     // immediately terminates

// Adjust limits at runtime
sched.set_max_steps(pid, 5000);
sched.set_max_time(pid, 10.0);
sched.set_max_memory(pid, 2 * 1024 * 1024);
sched.set_max_messages(pid, 200);
sched.set_priority(pid, -10);
```

### Forking

```cpp
int parent = sched.execute(std::string("(begin 1 2 3)"), opts);
sched.step(); // advance parent at least one step

int child = sched.fork(parent);
// child inherits: priority, uid, gid, max_steps, max_time, evaluator state
// child.parent_pid == parent

auto results = sched.run();
// Both parent and child complete independently
```

### Signals

```cpp
execute_options opts;
opts.signal_handlers["SIGUSR1"] = parse("(begin (state-set \"got-signal\" \"yes\"))");
int pid = sched.execute(std::string("(begin 1 2 3 4 5)"), opts);

sched.step();
sched.send_signal(pid, "SIGUSR1");  // handler runs before next step
sched.send_signal(pid, "SIGKILL");  // immediate termination
```

### Process Lifecycle (from DSL)

Programs can manage other processes using DSL intrinsics:

```lisp
;; Spawn a child process
(set child-pid (spawn "(begin (state-set \"child.done\" \"yes\") 42)" "child-worker"))

;; Fork the current process
(set cpid (fork))

;; Query own identity
(self-pid)      ;; => 1
(self-uid)      ;; => "alice"

;; List all processes
(ps)            ;; => list of dicts [{pid: 1, name: "...", status: "running", ...}, ...]

;; Inspect a specific process
(inspect 2)     ;; => dict with full process_info

;; Control other processes
(pause 2)
(resume 2)
(kill 2)

;; Query resource usage
(memory-usage)   (memory-limit)
(time-elapsed)   (time-limit)
(step-count)
(message-count)  (message-limit)

;; System identity
(cluster-id)    ;; => "cluster-1"
(node-id)       ;; => "node-A"
(scheduler-stats)  ;; => {total: N, running: N, ...}
```

---

## 5. The State Tree

Programs interact with a shared `cvc::state` tree via intrinsics.
The state tree is a hierarchical key-value store where any node can
hold a string value and have named children.

### Connecting Programs to the State Tree

To use state intrinsics, processes need an `intrinsics_context` wired
to the scheduler and state root.  When using `exec_coordinator`,
this is automatic.  For local-only usage:

```cpp
#include <cvc/core/state_exec/scheduler.h>
#include <cvc/core/state_exec/intrinsics.h>
#include <cvc/core/state_exec/builtins.h>
#include <cvc/core/state_exec/memory_tracker.h>
#include <cvc/core/app.h>
#include <cvc/core/state.h>

using namespace cvc::state_exec;

cvc::app app_ctx;
scheduler sched;
memory_tracker tracker;

// Create an environment with builtins + intrinsics for a process
process_ptr proc = make_process();
proc->pid = 1;
proc->status = process_status::ready;

intrinsics_context ictx;
ictx.sched      = &sched;
ictx.root        = &cvc::state::instance(app_ctx);
ictx.tracker     = &tracker;
ictx.proc        = proc;
ictx.pid         = 1;
ictx.uid         = "alice";
ictx.cluster_id  = "cluster-1";
ictx.node_id     = "node-A";

environment_ptr env = builtins::make_default_environment();
register_intrinsics(env, &ictx);
```

### State Tree Operations (DSL)

```lisp
;; Write a value
(state-set "myapp.config.timeout" "30")

;; Read it back
(state-get "myapp.config.timeout")   ;; => "30"

;; Check existence
(state-exists "myapp.config.timeout")  ;; => true

;; List children of a node
(state-children "myapp.config")  ;; => ("timeout")

;; Delete a node
(state-delete "myapp.old-data")

;; Store/retrieve binary data objects
(state-data-set "myapp.blob" some-value)
(state-data-get "myapp.blob")
```

### State Expiry

```lisp
;; Expire a node 60 seconds from now
(state-expire "myapp.session.abc" 60)

;; Expire at an absolute ISO 8601 time
(state-expire-at "myapp.session.abc" "2026-01-01T00:00:00Z")

;; Query expiry status
(state-has-expiry "myapp.session.abc")   ;; => true
(state-is-expired "myapp.session.abc")   ;; => false (not yet)

;; Clear expiry
(state-clear-expiry "myapp.session.abc")

;; Sweep all expired nodes under a path (returns count removed)
(state-sweep-expired "myapp.session")
```

---

## 6. Inter-Process Messaging

Processes communicate through the shared state tree using `msg-send`.
A process sends a message to a state tree path, and the state tree's
message bus delivers it to any subscribers on that path.

### Sending Messages (DSL)

```lisp
;; Send a plain-text message to a state tree path
(msg-send "myapp.events.updates" "hello from process 1")

;; Send with an explicit content type
(msg-send "myapp.events.data" "{\"key\": \"value\"}" "application/json")

;; msg-send returns a dict: {status: "delivered", path: "myapp.events.updates"}
```

### Producer-Consumer Pattern

**Producer process:**
```lisp
(begin
  (set i 0)
  (while (< i 10)
    (begin
      (state-set (str-concat "queue.item." (str i)) (str (* i i)))
      (msg-send "queue.notify" (str i))
      (set i (+ i 1))))
  "producer-done")
```

**Consumer process:**
```lisp
(begin
  ;; Poll for items (cooperative check)
  (set count 0)
  (while (< count 10)
    (if (state-exists (str-concat "queue.item." (str count)))
      (begin
        (set val (state-get (str-concat "queue.item." (str count))))
        (state-set (str-concat "consumed." (str count)) val)
        (set count (+ count 1)))
      nil))
  count)
```

### Running Multiple Communicating Processes (C++)

```cpp
scheduler sched;
cvc::app app_ctx;
auto& root = cvc::state::instance(app_ctx);

// Submit producer and consumer
int producer = sched.execute(std::string(R"(
    (begin
      (state-set "shared.data" "42")
      "wrote")
)"));

int consumer = sched.execute(std::string(R"(
    (begin
      (state-get "shared.data"))
)"));

// Both run cooperatively in the same scheduler
auto results = sched.run();
```

Note: For processes to share state, they must be connected to the
same `cvc::state` root and use `register_intrinsics()` with a context
pointing to that root.

### Host Receiving DSL Output (Print Pattern)

A C++ host can capture DSL program output by subscribing to a path prefix
on the message bus. This is the idiomatic way to implement "print"
statements in the DSL — the program sends messages and the host collects
them.

**C++ host setup:**
```cpp
#include <cvc/core/state_cluster_shard.h>
#include <cvc/core/state_exec/scheduler.h>

cvc::app app_ctx;
auto& root = cvc::state::instance(app_ctx);

// Create and attach a shard (enables msg-send routing)
cvc::state_cluster_shard shard(app_ctx, "my-cluster", "my-node");
shard.attach();

// Subscribe to all "console.*" messages
std::vector<std::string> output;
auto sub = shard.message_bus().subscribe("console",
    [&](const cvc::state_message& m) {
        output.push_back(m.string_value);
    });

// Run a DSL program that "prints"
scheduler sched;
sched.execute(std::string(R"(
    (begin
      (msg-send "console.stdout" "Hello from DSL!")
      (msg-send "console.stdout" "Computing...")
      (msg-send "console.stdout" (str-concat "Result: " (str (* 6 7))))
      (msg-send "console.stderr" "Warning: example only"))
)"));
sched.run();

// output now contains: ["Hello from DSL!", "Computing...",
//                       "Result: 42", "Warning: example only"]

shard.message_bus().unsubscribe(sub);
shard.detach();
```

Key points:
- `subscribe(prefix, callback)` matches any path starting with that prefix
  (dot-segment boundary matching, so "console" matches "console.stdout"
  and "console.stderr" but not "consolex")
- Each `msg-send` from DSL generates a unique message ID automatically
- The callback fires synchronously inside `admit()` — keep it fast
- Unsubscribe when done to avoid dangling references

### Receiving Messages (DSL)

`msg-recv` suspends the calling process until a message arrives on the
specified path.  If a message is already queued, it returns immediately.

```lisp
;; Block until a message arrives on "events.data"
(set msg (msg-recv "events.data"))

;; msg is a dict — extract the payload
(get-attr msg "payload")
```

Resolution order when `msg-recv` is called:
1. **Inbox** — if the process has buffered messages, pops the front one
2. **Pending queue** — if the scheduler has a pre-delivered message for the path, returns it
3. **Suspend** — otherwise the process enters `waiting` status until a message is delivered

### Checking for Pending Messages

`msg-pending` returns the number of queued messages for a path without
blocking:

```lisp
;; Non-blocking check
(if (> (msg-pending "events.data") 0)
    (set msg (msg-recv "events.data"))
    (print "no messages yet"))
```

### Sleeping

`sleep` suspends the current process for a given number of seconds.
The scheduler automatically wakes it when the duration expires.

```lisp
;; Sleep for half a second
(sleep 0.5)

;; Polling loop with sleep
(while true
  (begin
    (if (> (msg-pending "work.queue") 0)
        (begin
          (set job (msg-recv "work.queue"))
          (process-job job))
        (sleep 0.1))))
```

`sleep` accepts integer or floating-point seconds (must be non-negative).
It returns `true` on success.

### Message-Driven Process Example

A complete receiver that waits for messages and accumulates results:

```lisp
(begin
  (set results (list))
  (set done false)
  (while (not done)
    (begin
      (set m (msg-recv "tasks.input"))
      (set payload (get-attr m "payload"))
      (if (= payload "stop")
          (set done true)
          (set results (append results (list payload))))))
  results)
```

---

## 7. Resource Limits & Policies

### Per-Process Limits

Set via `execute_options` at submission time or adjusted at runtime:

```cpp
execute_options opts;
opts.max_steps         = 10000;     // evaluator steps
opts.max_time          = 5.0;       // wall time in seconds
opts.max_memory        = 1048576;   // bytes (tracked by memory_tracker)
opts.max_messages      = 100;       // outbound msg-send calls
opts.max_message_bytes = 65536;     // total outbound payload bytes
```

When a limit is exceeded, the process is killed (status becomes `killed`).

### Cluster-Level Resource Policy

A `resource_policy` defines cluster-wide constraints that apply to all
processes submitted through an `exec_coordinator`:

```cpp
#include <cvc/core/state_exec/resource_policy.h>

resource_policy policy;
policy.max_processes     = 50;       // cluster-wide process cap
policy.max_total_memory  = 100 * 1024 * 1024;  // 100 MB total

// Per-process bounds (enforced at submission time)
policy.max_time_min     = 0.1;   policy.max_time_max     = 60.0;
policy.max_memory_min   = 1024;  policy.max_memory_max   = 10485760;
policy.max_steps_min    = 100;   policy.max_steps_max    = 1000000;
policy.max_messages_min = 1;     policy.max_messages_max = 1000;

// Defaults applied when the submitter doesn't specify
policy.max_time_default    = 30.0;
policy.max_memory_default  = 1048576;
policy.max_steps_default   = 100000;
policy.max_messages_default = 500;

// Enforcement mode
policy.enforce = resource_policy::mode::clamp;   // clamp values to range
// policy.enforce = resource_policy::mode::strict;  // reject out-of-range
// policy.enforce = resource_policy::mode::warn;     // warn and allow

coordinator.set_resource_policy(policy);
```

### Validating Limits

```cpp
process_limits requested{
    .max_steps = 999999,
    .max_time  = 120.0,  // exceeds max
    .max_memory = 512,
    .max_messages = 50
};
auto validated = validate_limits(policy, requested);
// validated.max_time is clamped to 60.0
```

### Scheduler Settings via the State Tree

Scheduler configuration can be loaded from the state tree, allowing
runtime adjustment without recompilation.  Settings are resolved in
order: per-scheduler override → global default → hardcoded fallback.

```cpp
scheduler sched;
sched.set_watch_root(&root);   // point to the state tree root
sched.set_id("worker-1");      // scheduler identity
sched.load_settings();         // read and apply settings
```

**State tree paths:**

| Setting | Per-scheduler path | Global path | Default |
|---------|-------------------|-------------|---------|
| `max_pending_messages` | `state_exec.schedulers.<id>.max_pending_messages` | `state_exec.defaults.max_pending_messages` | `1024` |

```cpp
// Set a global default for all schedulers
root("state_exec.defaults.max_pending_messages").value("512");
sched.load_settings();
// sched.max_pending_messages == 512

// Override for a specific scheduler
root("state_exec.schedulers.worker-1.max_pending_messages").value("2048");
sched.load_settings();
// sched.max_pending_messages == 2048

// "0" means unlimited; invalid values fall back to 1024
```

After `load_settings()`, the scheduler publishes its effective
configuration back to the state tree:

- `state_exec.schedulers.<id>.max_pending_messages` — the resolved cap
- `state_exec.schedulers.<id>.policy` — `"round_robin"`, `"priority"`, or `"priority_rr"`

---

## 8. Distributed Execution

### The exec_coordinator

The `exec_coordinator` manages distributed process execution across
a cluster of nodes.  It handles leader election, process submission
routing, migration, cross-cluster observation, and admin controls.

```cpp
#include <cvc/core/state_exec/exec_coordinator.h>
#include <cvc/core/state_message_bus.h>

using namespace cvc::state_exec;

// Each node creates its own coordinator
scheduler sched;
state_message_bus bus;
exec_coordinator coord;

coord.set_node_id("node-A");
coord.set_cluster_id("cluster-1");
coord.attach_scheduler(&sched);
coord.attach_message_bus(&bus);

// Optional: attach a cluster shard for state tree replication
// coord.attach_shard(&shard);

// Optional: set resource policy
resource_policy policy;
policy.max_processes = 100;
coord.set_resource_policy(policy);

// Start the coordinator (begins leader election)
coord.start();
```

### Multi-Node Setup

In a multi-node cluster, each node has its own `exec_coordinator`,
`scheduler`, and they share messages through the `state_message_bus`:

```cpp
// Node A
scheduler sched_a;
state_message_bus shared_bus;  // in production, transport layer bridges these
exec_coordinator coord_a;
coord_a.set_node_id("node-A");
coord_a.set_cluster_id("cluster-1");
coord_a.attach_scheduler(&sched_a);
coord_a.attach_message_bus(&shared_bus);
coord_a.start();

// Node B
scheduler sched_b;
exec_coordinator coord_b;
coord_b.set_node_id("node-B");
coord_b.set_cluster_id("cluster-1");
coord_b.attach_scheduler(&sched_b);
coord_b.attach_message_bus(&shared_bus);
coord_b.start();
```

### Submitting to a Cluster

```cpp
// submit() routes through the leader
auto result = coord.submit("(+ 1 2)");
if (result.accepted) {
    std::cout << "PID: " << result.pid
              << " on node: " << result.node_id << "\n";
}

// With options
execute_options opts;
opts.name = "cluster-job";
opts.uid  = "alice";
auto result = coord.submit("(fib 20)", opts);
```

### Cross-Cluster Observation

```cpp
// List all processes across all nodes
auto all_procs = coord.ps_all();
for (auto& cp : all_procs) {
    std::cout << cp.node_id << ": pid=" << cp.info.pid
              << " name=" << cp.info.name << "\n";
}

// Cluster-wide statistics
auto cstats = coord.cluster_stats();
std::cout << "Total processes: " << cstats.total_processes
          << " running: " << cstats.total_running << "\n";
```

### Admin Controls

```cpp
// Control processes on specific nodes (leader only)
coord.admin_pause(pid, "node-B");
coord.admin_resume(pid, "node-B");
coord.admin_kill(pid, "node-B");

// Push a resource policy to the entire cluster
coord.admin_set_policy(policy);

// Hand off leadership to another node
coord.admin_handoff("node-B");
```

### Leader Election

The coordinator uses a **Bully Algorithm** for leader election.  Exactly one
node per cluster is elected *leader* (the scheduling node).  All `submit()`
calls are routed through the leader, which decides where processes run.

#### What triggers an election

| Trigger | Description |
|---------|-------------|
| Startup (no leader known) | `start()` sees `leader_node_id_` is empty |
| Leader death/eviction | Membership event with `kind >= 2` for the current leader |
| Explicit request | Calling `request_election()` programmatically |
| Receiving `election-start` from a lower-priority node | Responds with `election-alive` and starts own election |

#### Election message format

All messages are JSON, sent to path `__state_exec.<cluster_id>.election`
with MIME type `application/x-state-exec-election`.

**`election-start`** — candidacy announcement:
```json
{
  "type": "election-start",
  "node_id": "node-A",
  "priority": 10,
  "timestamp": 1716588123456789
}
```

**`election-alive`** — bully response ("I outrank you, stand down"):
```json
{
  "type": "election-alive",
  "node_id": "node-B",
  "priority": 20
}
```

**`election-victory`** — leader declaration:
```json
{
  "type": "election-victory",
  "node_id": "node-B"
}
```

#### Winner determination

The bully comparison is:
1. **Higher `election_priority`** wins outright.
2. **Equal priority** → lexicographically greater `node_id` wins.

```cpp
bool we_win = (our_priority > sender_priority) ||
              (our_priority == sender_priority && our_node_id > sender_node_id);
```

#### Protocol flow

```
Node A starts → no leader known → broadcast "election-start"
  │
  ├─ Nodes with HIGHER priority (or same + higher node_id):
  │    → Reply with "election-alive"
  │    → Start their own election
  │    → Node A sees "election-alive" from superior → stands down
  │
  └─ If no "election-alive" arrives within election_timeout:
       → Node A calls declare_victory()
       → Broadcasts "election-victory"
       → All other nodes accept Node A as leader
```

The timeout is checked opportunistically on heartbeat reception.  If no
superior node responds within `election_timeout` milliseconds, the
candidate self-declares.

#### State transitions

| State | `is_leader` | `election_in_progress` | `leader_node_id` |
|-------|:-----------:|:----------------------:|-------------------|
| Startup | false | false | `""` |
| Candidate | false | true | `""` (or stale) |
| Follower | false | false | `"<other_node>"` |
| Leader | true | false | `"<self>"` |

- `start()` → **Candidate** (starts election if no leader known)
- Election timeout expires → **Leader** (via `declare_victory()`)
- Receive `election-alive` from superior → stand down (stay Follower)
- Receive `election-victory` → **Follower** (accept new leader)
- Leader dies → **Candidate** (re-election triggered)

#### Heartbeats

The leader periodically broadcasts heartbeat messages to
`__state_exec.<cluster_id>.heartbeat`:

```json
{
  "node_id": "node-B",
  "is_leader": true,
  "stats": "<serialized scheduler_stats>"
}
```

Heartbeats serve three purposes:
1. Prove liveness of the leader to followers.
2. Propagate cluster-wide statistics for observation.
3. Trigger election-timeout convergence (checked on receipt).

#### Configuration

```cpp
exec_coordinator::config cfg;
cfg.heartbeat_interval = std::chrono::milliseconds(2000);  // send heartbeat every 2s
cfg.election_timeout   = std::chrono::milliseconds(3000);  // wait 3s before declaring victory
cfg.election_priority  = 10;  // higher = more likely to win
coord.set_config(cfg);
```

#### Voluntary leadership transfer

A leader can hand off leadership without a full re-election:

```cpp
// Current leader voluntarily transfers to node-B
coord.admin_handoff("node-B");
// Broadcasts election-victory with node-B's ID
// All nodes (including self) accept node-B as new leader
```

#### Single-node clusters

When only one node exists in the cluster, the election completes
immediately: no `election-alive` arrives within the timeout, so the
node declares itself leader.  In tests, the `make_leader()` helper
short-circuits this by directly injecting an `election-victory` message.

---

## 9. Process Migration

Processes can be migrated from one node to another.  The migration
protocol is: pause → serialize → send → resume on target → kill on source.

### Initiating Migration (C++)

```cpp
// Migrate process with given PID to node-B
auto result = coord.migrate(pid, "node-B");
if (result.success) {
    std::cout << "Migrated to " << result.target_node << "\n";
} else {
    std::cerr << "Migration failed: " << result.error << "\n";
}
```

### Self-Migration (from DSL)

A process can request its own migration by writing a request to the
state tree that an external coordinator monitors:

```lisp
;; A process writing a migration request for itself
(begin
  (set my-pid (self-pid))
  (state-set
    (str-concat "__migrate_request." (str my-pid))
    "target-node-B")
  ;; The coordinator or a management process can poll for
  ;; __migrate_request.* entries and call coord.migrate()
  my-pid)
```

### What Migrates

- Process name, priority, UID, GID
- Evaluator state (stack, variables, program counter)
- Resource limits (max_steps, max_time, max_memory, max_messages)
- Accumulated statistics (step_count, elapsed_time)

### What Does Not Migrate

- Open file handles or OS resources
- In-flight signals
- The `on_complete` callback
- Memory tracker ownership (re-registered on target)

---

## 10. Access Control Model

### UID/GID Inheritance

Every process has a `uid` and `gid` set at creation time.  When a
process spawns a child via the `spawn` intrinsic, the child inherits
the parent's UID:

```cpp
execute_options opts;
opts.uid = "alice";
opts.gid = "engineers";
int pid = sched.execute(std::string(R"(
    (begin
      ;; self-uid returns "alice"
      (set parent-uid (self-uid))
      ;; Spawn a child — it inherits our UID
      (set child (spawn "(self-uid)" "child-proc"))
      parent-uid)
)"), opts);
```

### Fork Inherits ACL

When a process forks, the child inherits all parent properties
including UID, GID, priority, and resource limits:

```cpp
execute_options opts;
opts.uid = "alice";
opts.gid = "engineers";
int parent = sched.execute(std::string("(begin 1 2 3)"), opts);
sched.step();
int child = sched.fork(parent);

auto child_info = sched.get_process_info(child);
assert(child_info->uid == "alice");
assert(child_info->gid == "engineers");
```

### Local Sovereignty Model

`state_exec` uses a **local sovereignty** model: the local scheduler
does not enforce access control on state tree operations.  Any process
on a node can read/write any state path on that node.

Access control is enforced at the **cluster consensus boundary** — when
a state mutation propagates through `state_cluster_shard`, the shard's
write policy, authority map, and delegation system decide whether the
write is accepted or rejected by other nodes.

This means:
- Processes are trusted within their local node
- The UID/GID on a process primarily serves as *identity* for auditing,
  cross-node permission checks, and cluster-level policy enforcement
- Cross-node admin commands (kill, pause, resume) go through the
  `exec_coordinator` which is leader-gated

### Resource Policy as ACL

The `resource_policy` acts as a form of access control by limiting
what processes can do:

```cpp
resource_policy policy;
policy.max_processes     = 10;    // cap concurrency
policy.max_memory_max    = 1024 * 1024;  // 1 MB per process
policy.max_steps_max     = 50000; // prevent runaway computation
policy.enforce = resource_policy::mode::strict;  // reject violations

coord.set_resource_policy(policy);
// Now any submit() that requests more than these limits is rejected
```

---

## 11. Standard Library

The `stdlib_registry` provides additional functions organized into modules.
Import a module into a process's environment to make its functions available:

```cpp
#include <cvc/core/state_exec/stdlib.h>

stdlib_registry stdlib;

// Import all functions from a module
stdlib.import_module("string", env);
stdlib.import_module("math", env);
stdlib.import_module("collections", env);

// Import specific functions
stdlib.import_module("math", env, {"math.sqrt", "math.abs"});
```

### Module: `string` (12 functions)

```lisp
(string.split "a,b,c" ",")         ;; => ("a" "b" "c")
(string.join (list "a" "b" "c") ",") ;; => "a,b,c"
(string.replace "hello" "l" "r")    ;; => "herro"
(string.trim "  hello  ")           ;; => "hello"
(string.upper "hello")              ;; => "HELLO"
(string.lower "HELLO")              ;; => "hello"
(string.starts-with "hello" "he")   ;; => true
(string.ends-with "hello" "lo")     ;; => true
(string.contains "hello" "ell")     ;; => true
(string.substring "hello" 1 3)      ;; => "el"
(string.char-at "hello" 0)          ;; => "h"
(string.length "hello")             ;; => 5
```

### Module: `math` (16 functions)

```lisp
(math.sqrt 16)       ;; => 4.0
(math.abs -5)        ;; => 5
(math.floor 3.7)     ;; => 3.0
(math.ceil 3.2)      ;; => 4.0
(math.round 3.5)     ;; => 4.0
(math.pow 2 10)      ;; => 1024.0
(math.log 2.718)     ;; => ~1.0
(math.sin 0)         ;; => 0.0
(math.cos 0)         ;; => 1.0
(math.min 3 7 1 5)   ;; => 1
(math.max 3 7 1 5)   ;; => 7
(math.clamp 15 0 10) ;; => 10
(math.pi)            ;; => 3.14159...
(math.e)             ;; => 2.71828...
(math.random)        ;; => random double in [0,1)
```

### Module: `collections` (12 functions)

```lisp
(collections.map (lambda (x) (* x x)) (list 1 2 3))     ;; => (1 4 9)
(collections.filter (lambda (x) (> x 2)) (list 1 2 3 4)) ;; => (3 4)
(collections.reduce (lambda (a b) (+ a b)) (list 1 2 3) 0) ;; => 6
(collections.zip (list 1 2) (list "a" "b"))               ;; => ((1 "a") (2 "b"))
(collections.flatten (list (list 1 2) (list 3 4)))         ;; => (1 2 3 4)
(collections.sort (list 3 1 2))                            ;; => (1 2 3)
(collections.reverse (list 1 2 3))                         ;; => (3 2 1)
(collections.range 1 5)                                    ;; => (1 2 3 4)
(collections.unique (list 1 2 2 3 3))                      ;; => (1 2 3)
(collections.dict-keys (dict "a" 1 "b" 2))                 ;; => ("a" "b")
(collections.dict-values (dict "a" 1 "b" 2))               ;; => (1 2)
(collections.dict-merge (dict "a" 1) (dict "b" 2))         ;; => {a: 1, b: 2}
```

---

## 12. Architecture Reference

```
┌─────────────────────────────────────────────────────────────────┐
│                        DSL Programs                             │
│  (defun fib (n) (if (<= n 1) n (+ (fib (- n 1)) (fib ...))))  │
├─────────────────────────────────────────────────────────────────┤
│  parser.h        → value_t AST                                 │
│  builtins.h      → +,-,*,/,list,dict,str,...                   │
│  stdlib.h        → string.*,math.*,collections.*               │
├─────────────────────────────────────────────────────────────────┤
│  evaluator.h              → sync recursive (local only)        │
│  stackless_evaluator.h    → sync stackless (serializable)      │
│  async_evaluator.h        → async recursive (local only)       │
│  async_stackless_evaluator.h → async stackless (cluster-ready) │
├─────────────────────────────────────────────────────────────────┤
│  process.h         → pid, uid/gid, status, limits, state       │
│  scheduler.h       → execute, run, step, pause, fork, signals  │
│  async_scheduler.h → async variant of scheduler                │
│  memory_tracker.h  → per-process byte tracking                 │
├─────────────────────────────────────────────────────────────────┤
│  intrinsics.h      → state-get/set, spawn, fork, ps, msg-send,  │
│                       msg-recv, msg-pending, sleep               │
│  resource_policy.h → cluster-wide resource constraints         │
├─────────────────────────────────────────────────────────────────┤
│  exec_coordinator.h → leader election, submit, migrate, admin  │
├─────────────────────────────────────────────────────────────────┤
│  state_value_codec.h   → value_t ↔ JSON serialization         │
│  task.h                → C++20 coroutine support               │
├─────────────────────────────────────────────────────────────────┤
│  cvc::state            → hierarchical state tree               │
│  state_message_bus     → pub/sub message routing               │
│  state_cluster_shard   → cluster replication + consensus       │
└─────────────────────────────────────────────────────────────────┘
```

### Component Relationships

- **Parser** produces `value_t` ASTs consumed by all evaluators
- **Evaluators** execute ASTs in `environment` scopes; the stackless
  variants expose `evaluator_state` for serialization and migration
- **Scheduler** wraps `stackless_evaluator` and manages process lifecycle
- **Intrinsics** bridge DSL code to C++ APIs (state tree, scheduler, etc.)
- **exec_coordinator** sits above the scheduler and provides distributed
  services via `state_message_bus`
- **state_value_codec** enables migration by serializing evaluator state

### Header Dependency Graph

```
types.h ← parser.h ← builtins.h ← evaluator.h
                                  ← stackless_evaluator.h
                                  ← state_value_codec.h
                   ← process.h   ← scheduler.h ← async_scheduler.h
                                  ← intrinsics.h
                                  ← exec_coordinator.h
              task.h ← async_evaluator.h
                     ← async_stackless_evaluator.h
         resource_policy.h ← exec_coordinator.h
              stdlib.h (standalone)
         memory_tracker.h ← scheduler.h
```
