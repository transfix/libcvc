# state_exec Developer Guide

## Overview

`state_exec` is a sandboxed S-expression DSL engine built on top of
`cvc::state`.  Programs execute as lightweight processes managed by a
scheduler, with full support for process control, inter-process messaging
via the shared state tree, distributed execution across nodes, and process
migration.

This guide walks through every major subsystem with working C++ examples
you can drop directly into a test or application.

Every scheduler steps its processes on one thread today. The design for
running programs on several threads is in
[STATE_EXEC_THREADED.md](STATE_EXEC_THREADED.md) (adopted, not yet built):
execution groups, each one scheduler stepped by one thread at a time, run in
parallel on a shared worker pool, added in stages.

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
12. [Gotchas & Pitfalls](#12-gotchas--pitfalls)
13. [Architecture Reference](#13-architecture-reference)

---

## 1. Quick Start

### Minimal "Hello World"

```cpp
#include <cvc/state/state_exec/scheduler.h>
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

The full syntax (literals, escapes, Unicode rules) is specified in
[STATE_EXEC_GRAMMAR.md](STATE_EXEC_GRAMMAR.md).

### Types

| Type | Literal examples | C++ variant tag |
|------|-----------------|-----------------|
| Integer | `42`, `-7` | `int64_t` |
| Float | `3.14`, `1e-5` | `double` |
| String | `"hello"` | `std::string` |
| Bytes | `b"\x00\xff"` | `bytes_value` |
| Boolean | `true`, `#t` (`false` and `#f` read as nil) | `bool` |
| Symbol | `x`, `my-func` | `symbol` |
| Nil | `nil`, `false`, `#f` | `std::monostate` |
| List | `(1 2 3)` | `list_ptr` |

### Special Forms

| Form | Syntax | Description |
|------|--------|-------------|
| `set` | `(set x 42)` | Bind a variable |
| `defun` | `(defun name (params...) body...)` | Define a named function |
| `lambda` | `(lambda (x y) (+ x y))` | Anonymous function |
| `if` | `(if cond then [else])` | Conditional; more than one else form is an error |
| `begin` | `(begin e1 e2 ... en)` | Sequence; returns last value |
| `and` | `(and a b ...)` | Evaluates left to right and stops at the first falsy value: `#f` there, else the last value (`#t` with none). The rest aren't evaluated. Passed as a value (`(apply and lst)`), `and` is a function of arguments already evaluated |
| `or` | `(or a b ...)` | Evaluates left to right and stops at the first truthy value, which it returns; `#f` when none is. As a value, a function like `and` |
| `while` | `(while cond body...)` | Loop while condition is true; the body forms run in order, as a `begin`; the loop's value is nil (or `break`'s) |
| `for` | `(for x collection body...)` | Iterate over a list or generator; the body forms run in order, as a `begin` |
| `let` | `(let ((x 1) (y 2)) (+ x y))` | Local bindings |
| `quote` | `(quote (1 2 3))` | Return expression unevaluated |
| `return` | `(return expr)` | Early return from the function it is in |
| `break` | `(break)` or `(break expr)` | Exit nearest `while`/`for` loop |
| `try` | `(try body... (catch (Type e) handler...)... (finally cleanup...))` | Catch exceptions; see [Exceptions](#exceptions) |
| `raise` | `(raise exc)` or `(raise)` | Raise an exception; bare, re-raise the one a handler handles |
| `yield` | `(yield expr)` | Produce a value from a generator |
| `defmacro` | `(defmacro name (params) template)` | Macro definition |
| `eval` | `(eval expr)` | Evaluate a quoted expression |
| `defclass` | `(defclass Name ...)` | OOP class definition |
| `root` | `(root)` | Returns the root expression |

`with-lock` is not a special form: it is the lock form the intrinsics bind, recognized by what
its name is bound to ([Locks](#locks-with-lock)).

Extra arguments are an error, not ignored: `(if c a b d)`, `(return a b)`, `(yield a b)` and
`(break a b)` throw, naming the form. Until 2026-10 `while` and `for` ran only their first body
form and `if` dropped forms after its else branch, silently; scripts that wrapped a loop body in
`(begin ...)` behave exactly as before.
| `super` | `(super self method args...)` | Call parent class method |

### Built-in Functions

**Arithmetic:** `+`, `-`, `*`, `/`, `%`
**Comparison:** `<`, `>`, `<=`, `>=`, `=`, `!=`
**Logic:** `not`, `and`, `or`
**String:** `str` (convert to string), `str-concat`, `url-encode`, `url-decode`
**JSON:** `json-parse`, `json-encode` ([below](#json))
**List:** `list`, `car`, `cdr`, `cons`, `nth`, `set-nth`, `length`, `append`, `slice`, `del-nth`
**Dict:** `dict`, `get-attr`, `set-attr`, `del-attr`
**Type:** `null?`, `list?`, `type-of`, `generator?`, `is-int`, `is-float`, `is-string`
**Conversion:** `int` (to integer), `float` (to float)
**I/O:** `print`
**Generator:** `generator`, `next`, `range`, `collect`, `gen-done?`
**OOP:** `send` (method dispatch), `apply`
**Errors:** `error`, `assert` ([Exceptions](#exceptions))
**Stdlib:** `string.*`, `bytes.*`, `math.*`, `collections.*`, always loaded ([§11](#11-standard-library))

`apply`, `send` and `collections.map`/`filter`/`reduce` call a native function or a DSL
function alike. A DSL function runs as the program's own frames on the same evaluator
(`call_function()` in `data_job.h`; see §11, "Sliced work"):

```lisp
(apply (lambda (a b) (* a b)) (list 6 7))        ;; => 42
(set o (dict "__class__" (dict "inc" (lambda (self x) (+ x 1)))))
(send o "inc" 41)                                ;; => 42
```

#### Surprises

- `and` and `or` short-circuit: `(and (net-auth-has? "txwtf") (load-home))` runs `load-home` only
  with a credential. Until 2026-10 they were plain functions and evaluated every argument.
- `false` and `#f` read as nil, and `(not true)` is the bool false. Both are falsy.
- `str` prints its argument as `print` would, so a string comes back quoted: `(str "abc")` is
  `"\"abc\""`. To join text, pass the strings to `str-concat` as they are:
  `(str-concat "id=" s)`.
- `/` always gives a float: `(/ 7 2)` is `3.5`, and `(/ 6 3)` is the float 2 (`is-float` is true;
  it prints as `2`). For a whole quotient use `(math.floor (/ 7 2))` (`3`, an int) or
  `(int (/ 7 2))`, which truncates toward zero; `%` is the remainder.
- `collections.map`, `filter` and `reduce` take the list first, then the function:
  `(collections.map (list 1 2 3) (lambda (x) (* x x)))`.

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

#### JSON

`json-parse` reads JSON text (a string, or bytes such as an HTTP reply's body) into values, and
`json-encode` writes values as JSON text. Both run as data jobs (see
[Deep data on the executor](STATE_EXEC_GRAMMAR.md#deep-data-on-the-executor)), so a large
document is read or written a slice per step.

```lisp
(json-parse "{\"name\": \"ada\", \"tags\": [1, 2.5, true, null]}")
;; => {"name": "ada", "tags": (1 2.5 #t nil)}
(json-encode (dict "name" "ada" "n" 3))      ;; => "{\"name\":\"ada\",\"n\":3}"
(json-encode (list 1 (dict)) true)          ;; => "[\n  1,\n  {}\n]"
(json-encode (list 1) 4)                    ;; four spaces a level
```

| JSON | value | back to JSON |
|---|---|---|
| object | dict, keys in document order | an object, keys in the dict's order |
| array | list | an array |
| string | string | a string: `"`, `\` and control characters escaped, other text as UTF-8 |
| number without a fraction or exponent | int, when it fits in 64 bits; otherwise a float | digits |
| any other number | float | its shortest round-trip form, with `.0` when it has neither a point nor an exponent |
| `true`, `false` | the bools | `true`, `false` |
| `null` | nil | `null` |

- **A repeated key** keeps its first place and takes its last value, as JavaScript's
  `JSON.parse` and Python's `json.loads` do: `{"a": 1, "b": 2, "a": 3}` is `{"a": 3, "b": 2}`.
- **Big numbers.** An integer outside 64 bits reads as a float, as JavaScript reads it, so it is
  exact only up to 2^53. A number too large for a float (`1e999`) is an error.
- **Text.** A byte that is not part of valid UTF-8, or a `\u` escape of a lone surrogate (a
  truncated emoji), becomes U+FFFD, so one bad character doesn't lose a whole reply.
  `json-encode` refuses a string that isn't valid UTF-8.
- **Errors.** Malformed text is a `ValueError` whose message names the line and column (in code
  points), with `line`, `col` and `offset` (in bytes) as fields:
  `(try (json-parse text) (catch (ValueError e) (get-attr e "line")))`. A raw control
  character in a string is malformed too.
- **Depth.** Lists and dicts nest at most 512 levels (`kJsonMaxDepth`), in both directions.
  Deeper input is a `ValueError`, found without recursion.
- **What JSON can't hold.** `json-encode` raises a `TypeError` for a symbol, function, bytes,
  generator or data object, and a `ValueError` for `nan` or an infinity, or a list or dict that
  contains itself. A value held twice is written twice. The indent is `true` (2 spaces), `nil`
  or 0 (compact), or 1 to 10 spaces.
- **`false` reads as nil** in source, and nil encodes as `null`. The bool false is `(not true)`.
- **Limits.** What `json-parse` builds and what `json-encode` writes are held to the per-call
  allocation limit (`limits.max_alloc_bytes`, a `MemoryError` past it).

The C++ jobs are `make_json_parse_job` and `make_json_encode_job` (`data_job.h`).

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

### Exceptions

The full design is in [STATE_EXEC_EXCEPTIONS.md](STATE_EXEC_EXCEPTIONS.md). An exception is an
instance of `Exception` or a class derived from it. `raise` raises one, `error` raises a
`RuntimeError`, and `try` catches them:

```lisp
(defclass VolumeError RuntimeError
  (init (self message volume)
    (super self init message)          ;; Exception's init sets message and data
    (set-attr self "volume" volume)
    self))

(try (raise (VolumeError "unreadable" "v1"))
  (catch (VolumeError e) (get-attr e "volume"))   ;; => "v1"
  (catch (e) (get-attr e "message"))              ;; any other exception
  (finally (print "done")))                       ;; however the try is left
```

- Catch clauses are tried in order; a clause matches an instance of its class or of a class
  derived from it. `(catch (Type) ...)` binds no variable, and `(catch (e) ...)` catches
  anything. A lone symbol is a class when it names one, otherwise the variable.
- The handler's variable is bound in a scope of its own: `set` in a handler binds there, so
  record effects in a list or dict (`append`, `set-attr`) when they should outlive it.
- `finally` runs when the body or a handler finishes, when an exception leaves the try, and when
  a `return` or `break` passes through. An exception, `return` or `break` in the cleanup replaces
  what was leaving.
- `(error message [data])` is `(raise (RuntimeError message data))`. `(isinstance x Class)`
  works for any class.
- `(assert cond [message [data]])` returns nil when `cond` is truthy. Otherwise it raises an
  `AssertionError` (derived from `Exception`, not `RuntimeError`) whose message is
  `assertion failed`, or `assertion failed: <message>` (a non-string message is printed), with
  `data` as its data. Its traceback ends at the `(assert ...)` form, so a host that reports an
  uncaught one (Ariadne's `failure_report`) shows the condition that failed:

  ```lisp
  (try (assert (= (+ 1 1) 3) "one and one")
    (catch (AssertionError e) (get-attr e "message")))  ;; => "assertion failed: one and one"
  (assert (> n 0) "n is positive")                       ;; uncaught:
  ;; AssertionError: assertion failed: n is positive (at (assert (> n 0) "n is positive"))
  ```

  Ariadne's `ari test` runs documents whose programs assert (docs/ARIADNE.md §13).

Every exception has `message`, `data`, `cause` (the exception being handled when it was raised),
`origin` (`"dsl"`, `"builtin"` or `"native"`) and `traceback` (the expressions being evaluated,
innermost last, at most 64). One from a native function also has `function`, and a C++ exception
of no mapped type is a `NativeError` with `native_type`.

The evaluator's and the builtins' errors are exceptions of these classes, so a program catches
them too: `NameError` (an undefined symbol), `TypeError` (a wrong type, a wrong number of
arguments, a malformed special form, calling a non-function), `ValueError`, `KeyError`,
`IndexError`, `ArithmeticError` (division by zero), `MemoryError` (a call over the allocation
limit), `ParseError`. A program can't catch a kill: a step, time or memory limit, `kill`, or an
evaluator's timeout or `interrupt()` ends it with no handler or `finally` run.

A signal or watch handler's uncaught exception is raised in the program where the handler
interrupted it, so the program's own `try` can catch it.

**Raising from a native function.** Include `<cvc/state/state_exec/exceptions.h>`.
- `dsl_error(class_name, message)` raises a built-in class.
- `dsl_exception(value)` raises an exception value made with `make_exception`.
- Other C++ exceptions are mapped:

  | C++ exception | DSL class |
  |---|---|
  | `std::invalid_argument` | `ValueError` |
  | `std::out_of_range` | `IndexError` |
  | `std::bad_alloc` | `MemoryError` |
  | `parse_error` | `ParseError` |
  | a plain `std::runtime_error` | `RuntimeError` |
  | anything else | `NativeError` |

**When nothing catches it,** the process ends **`failed`**, which is not `killed`.
- **On a scheduler:**
  - `run()` doesn't throw it;
  - `get_exit_exception(pid)` returns the exception, and `get_exit_error(pid)` its
    `"Class: message"`;
  - `raise_exit_exception(pid)` throws it as a `dsl_exception`;
  - `process_info` carries both `exit_exception` and `exit_error`, and `scheduler_stats`
    counts `failed`.
- **On a bare evaluator:** `step()`, `run()` and `evaluate()` throw the `dsl_exception`.
- **The `dsl_exception`:** `what()` is `"Class: message"`, `value()` is the exception, and
  `class_chain()` names its classes. For an exception made from a C++ exception, `original()`
  is that exception.

**Raising into a state:** `stackless_evaluator::raise(state, exc)` raises an exception in a
state where it is evaluating.

**From Python:**
- A DSL exception that reaches Python raises `pycvc.dsl_error` (a `RuntimeError`). It has
  `.value`, `.dsl_class`, `.dsl_classes`, `.origin` and `.traceback`.
- Its type derives from `pycvc.dsl_class(name)` for each class in its chain, so
  `except pycvc.dsl_class("ValueError")` catches a `(defclass MyError ValueError)`.
- A registered Python function's exception is a `PythonError` in the DSL. Uncaught, it comes
  back to Python as the original exception, with the `dsl_error` as its `__cause__`.

---

## 3. Running Programs

### The Scheduler

The `scheduler` is the primary entry point.  It manages a collection of
processes, runs them cooperatively via time-slicing, and provides querying,
pausing, and resource-limit enforcement.

```cpp
#include <cvc/state/state_exec/scheduler.h>
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

### How the Schedulers Pick, Wait and Forget

Both schedulers keep their processes in a process table (`process_table.h`) with the indexes a
step needs, so no step scans, rebuilds or sorts the processes. (A step still gets dearer as the
processes grow from one to thousands, two to three times over, from cache misses on the process
it picks.)

- **The pick.** `round_robin` steps the runnable (ready or running) processes in pid order: the
  scheduler keeps an index, and each step takes the process at that index (modulo how many are
  runnable) and moves the index on. `priority` steps the lowest pid among the best (lowest)
  priority, however many are tied there (before the run queue the tie went to whatever an
  unstable `std::sort` left first, which was the lowest pid only while the runnable set fit the
  sort's insertion pass: 32 processes on MSVC, 16 on libstdc++); `priority_rr` goes round robin
  among the best priority's. A process that becomes
  runnable takes its place by pid, so the order in which a set of processes interleaves is fixed
  by their pids, their statuses and their priorities, step by step. The run queue finds the pick
  without scanning (order statistics over the processes in pid order).
- **Sleeps and timed waits** wake from a timer heap, when their deadline comes; a message wakes
  the processes waiting on its channel from an index of waiters by channel.
- **The sync `run()` does not spin.** `max_steps` counts the run loop's iterations, the idle ones
  (nothing ready) included, and `max_time` the run's own time. While nothing is ready and
  processes sleep (or wait for a message with a time limit), `run()` used to spin through idle
  iterations until a deadline came or a budget ran out. It waits for the earliest deadline instead
  and charges the wait to both budgets as the spin spent them: to `max_time` as run time
  (`time_budget::charge_wait`), to `max_steps` at 100 ns an idle iteration, about what a spinning
  one took. So a host that pumps `run(N)` a frame gets control back at once when its N steps
  would run out before the next deadline (as the spin did, a few microseconds later), a bounded
  run to completion with steps to spare (Ariadne's init scripts: 10M steps, 5 s) waits for the
  sleeper, as the spin did, and a run whose `max_time` runs out first returns then. When only
  processes waiting for a message with no time limit are left, nothing in a synchronous run can
  wake them: `run()` returns at once when `max_steps` bounds it, else waits out `max_time`, else
  waits until `stop()`. An infinite (or huge) `max_time` or sleep is "never" (the time arithmetic
  saturates). `run_steps()` is `run()` without the map of results it returns: a host that reads
  only the results it needs (`get_result`) saves a map entry per finished process not yet reaped
  on every call (`async_scheduler::sync_run_steps()` likewise).
- **An idle host can wait for work.** `async_scheduler::wait_for_work(deadline)` blocks the
  scheduler's thread until a `post_message()` or a settled async call arrives (from any thread), a
  sleeper or timed wait is due, an async call's registration timeout passes, the next charge of a
  call whose process has a step or time limit is due (every millisecond while one runs: a kernel
  that never checks `cancelled()` is charged its run time as CPU time, and only a drain finds the
  limit reached, `docs/STATE_EXEC_EXCEPTIONS.md` 3.8), or the deadline passes; it returns at
  once when a process is ready. Only the calls of processes still running count: a call whose
  process ended without awaiting it (an abandoned future) runs on, but no drain times it out or
  charges it, so it doesn't end the wait either. Nor does a timeout that never passes (NaN or
  infinite). Processes parked until the next frame (`(await ...)`, `awaiting_frame_count()`) are
  the host's to wake with `wake_awaiting()`. pycvc's `Exec` waits this way instead of polling;
  while a coroutine is pending, its asyncio slice is the wait where asyncio's clock is fine (the
  slice blocks in the selector until its 5 ms stop timer), and a 5 ms beat follows a slice that
  returned at once (Windows before Python 3.13, whose ~15.6 ms clock runs that timer at once). A
  process with no limits is charged for a call at the drains the driver makes anyway (each
  settled call's final charge included), so a change to `limits.async_steps_per_cpu_ms` applies
  to the next of those.

  ```cpp
  for (;;) {
    sched.drain_ingress();
    sched.wake_awaiting();          // once per frame, if the host has frames
    if (sched.has_runnable())
      sched.sync_run(200000, 0.05);
    else
      sched.wait_for_work(std::chrono::steady_clock::now() + std::chrono::seconds(1));
  }
  ```
- **Waits and the CPU.** A deadline of the timer heap, a sleeper's or a timed wait's (an
  `(await ...)` or a message wait with a time limit), is kept to well under a millisecond. On
  Windows, whose high-resolution timer fires on a ~0.5 ms grid, that costs up to ~0.6 ms of
  spinning at the end of each such wait (`detail::wakeup`): a run of millisecond sleeps still uses
  about half a core, against all of one while `run()` spun. Such a wait raises a thread at normal
  priority to above normal, so on a machine whose cores are all busy the thread gets one back
  when its timer fires rather than a scheduler quantum later; even so a blocked wait there can
  wake a few milliseconds late now and then, where the spin, which never gave its core up, did
  not. The thread is back at normal after the wait, unless its priority was changed meanwhile
  (that change stands), and a thread the host gave another priority keeps it: one lowered to keep
  the driver below the host's other threads isn't raised. Every other wait (a deadline of the
  caller's, a call's timeout or charge, a budget's end) may end up to ~0.5 ms late and burns no
  CPU. No wait ends before its time on the steady clock: when an OS timer fires early, the wait
  reads the clock and waits the rest, and a sleeper or a timed wait wakes only once the clock has
  reached its deadline. A call's timeout is waited for until the first clock tick at which the
  call counts as timed out (`async_call::timeout_point()`).
- **Finished processes** leave every index at once but stay in the table, so `get_result()`,
  `get_process_info()` and `get_results()` still report them, until `reap(pid)` erases one (both
  schedulers; a host that reaps from `on_complete` is fine: the process goes once its last step is
  over).
- **State watches** are read only when something under the watch root changed since the last
  poll: the scheduler counts the root's change signals while processes have watches, so a quiet
  tree costs a step nothing however many watches there are. This relies on writes firing the
  tree's signals, as `value()` writes do. Two kinds don't: `state::reset(..., fireCallbacks =
  false)`, and a node `operator()` creates. `state_list`'s `pop_back`, `erase`, `insert` and
  `clear` reset elements that way, but each then writes the list's size, which fires, so a watch
  on an element still sees it change (`WatchSeesAStateListElementPoppedOff`). A host that resets
  silently and writes nothing after it is seen at its next write that fires; the old every-step
  poll would have seen it at once. Likewise the async scheduler reads
  `limits.async_steps_per_cpu_ms` from the tree for its calls in flight only when the tree
  changed (and `set_id()` makes it read again), and a change still applies to the next charge.
  A drain reads it once, before it walks the calls in flight: a change of the rate is published
  (`effective.async_steps_per_cpu_ms`), a tree write whose slots run there and then and may kill
  or reap, so none runs while the calls are walked.
- **Settings.** What a submission reads from the tree (the async scheduler's
  `limits.max_alloc_bytes`, `limits.heap_check_steps`, `parser.*` and `data.*`) and the sync
  scheduler's inline-call rate are read again only when something under the tree's `state_exec`
  node changed (a count of that node's change signals, `detail::settings_changes`), so a
  submission doesn't walk the tree four times when nothing did, and writes elsewhere in the tree
  pay nothing for it. A change still applies to the next submission.
- **Time.** A step reads the steady clock once: within one `run()`, a step starts at the reading
  the step before it ended at, unless something other than picking it ran in between (a handler
  set up, a process finished, a watch poll, a cycle collection, an async-call charge, a sleeper's
  or timed wait's wake, connecting or disconnecting the watch count), and every time budget of
  the step uses that reading.

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

From the DSL, `(fork)` forks the process running it: it returns the child's pid in the parent and
0 in the child, and the child has the parent's owner, so a host reaping the owner's processes
(`kill_owner`) reaps it too. Inside `with-lock` it raises `LockError` (the child would run in a
section it doesn't hold).

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

**What a spawned child sees.** A child always keeps its parent's `owner`, so a host that reaps
a document's processes with `kill_owner` reaps its children too. Its environment depends on the
host (docs/STATE_EXEC_THREADED.md 4.15):

- With no spawn context, the child gets the bare builtins: no intrinsics, so `(state-get ...)`
  in the child is a `NameError`. This is what a context built by hand gets unless it sets one.
- With a spawn context (`intrinsics_context::spawn_context`), the callback gives the child its
  environment and a context of its own. The child's process keeps that context alive until the
  process is reaped. Ariadne's actions, residents and init scripts, pycvc's `Exec` and
  `make_spawn_context` all install one.

A C++ host that registers the plain intrinsics installs the ready-made callback:

```cpp
intrinsics_context ictx;  // sched, root (apply_chroot), uid, channels, lock fields, ...
ictx.spawn_context = make_spawn_context();  // or make_spawn_context(base, after_register)
register_intrinsics(env, &ictx);
```

Each child then gets a context copying the parent's:

- scheduler, root and chroot, and tracker;
- identity, channel policy and document;
- lock fields (`lock_wait_allowed`, `lock_default_timeout`);
- the callback itself, so grandchildren get one too.

The child's environment is `environment::extend(base)` with the intrinsics registered in it,
then `after_register(env, ctx)`, the host's own bindings. A null `base` is one default
environment, shared by the callback's children.

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
#include <cvc/state/state_exec/scheduler.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/memory_tracker.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>

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

;; List every node under a node, by its full path: descendants too, in pre-order
(state-children "myapp.config")   ;; => ("myapp.config.timeout")
(state-set "myapp.users.ann.role" "admin")
(state-children "myapp.users")    ;; => ("myapp.users.ann" "myapp.users.ann.role")

;; Delete a node
(state-delete "myapp.old-data")

;; Store/retrieve binary data objects
(state-data-set "myapp.blob" some-value)
(state-data-get "myapp.blob")

;; A subtree's values as JSON, and back: the document cvc::state::json() writes, keyed by paths
;; relative to the node ("" for the node itself). Both run as sliced data steps.
(state-json "myapp.config")   ;; => "{\n    \"timeout\": \"30\"\n}\n"
(state-from-json "myapp.copy" (state-json "myapp.config"))

;; A value of dicts, lists and scalars (what json-parse makes) as a subtree
(state-set-tree "feed" (json-parse "{\"items\": [{\"text\": \"hi\"}, {\"text\": \"yo\"}]}"))
(state-get "feed.items")          ;; => "2", the list's length
(state-get "feed.items.1.text")   ;; => "yo"
```

A node holds text, so `state-set` turns any other value into text. A string is stored as it is.
An int is stored as its digits (`10` as `"10"`). A float is stored as the shortest text that
reads back as the same number: `1234567.5` as `"1234567.5"`, `0.1` as `"0.1"`, `1e21` as
`"1e+21"`, `(+ 0.1 0.2)` as `"0.30000000000000004"`. Of the fixed and scientific forms the
shorter is stored, so `100000.0` is `"1e+05"`. `true` and `false` are stored as `"true"` and
`"false"`, and `nil` as `""`. A list or dict is stored as it prints; a structured value
belongs on the data channel (`state-data-set`). `str` and `str-concat` write a float the same
way, so `(state-set "x" (str 1234567.5))` also stores `"1234567.5"`.

`state-json` returns nil for a missing node and reads through a transparent link, as
`state-get` does. `state-from-json` writes through a writable transparent link, as `state-set`
does. It sets every non-empty string, number, `true`, `false` and `null` at its key, and returns
nil. It checks the whole document before it sets anything, so a malformed document changes
nothing. Its errors name the line, as in `state-from-json: line 3: expected ':'`.

`state-set-tree` writes a value as the subtree at its path, so Ariadne's binds and `repeat:` can
show it (docs/ARIADNE.md §10.4):

- A dict's keys and a list's indices (`0`, `1`, ...) name child nodes. A scalar is a node's
  value as `state-set` writes it: `true`/`false`, digits, a float in its shortest round-trip
  form (`0.30000000000000004`), the string itself, and the empty string for nil.
- A list's node holds its length, so `repeat: (state-get "feed.items")` repeats a widget per
  element. A dict's node holds the empty string.
- It replaces the subtree: a child the value doesn't name is removed with its subtree (the
  expiry `state-delete` uses), and a node whose text doesn't change isn't written, so its
  watchers don't fire.
- It checks the whole value before it writes anything, so a value it can't write changes
  nothing, not even the node at its path. A key must name one node: not empty, no `.`, and no
  space at either end (paths are trimmed). Anything but nil, a bool, a number, a string, a list
  or a dict is a `TypeError`; a bad key, a list or dict that contains itself, or nesting deeper
  than 512 levels is a `ValueError`. So is an empty path: replacing the root's subtree would
  remove everything else the program can reach.
- It writes through a writable transparent link, as `state-set` does, and returns nil. Both
  passes run as sliced data steps; a reader between two slices of the write sees part of it.

#### Atomic single-node updates

Three natives read and write one node in one step, under one hold of the node's mutex. No other
writer can land between the read and the write: not another process, not another thread, not
the host.

```lisp
(state-incr "stats.hits")              ;; => 1, then 2, ...: the new number
(state-incr "stats.load" 0.5)          ;; a float delta makes a float
(state-cas "jobs.j7.owner" nil "me")   ;; => #t when it was absent or "", and is now "me"
(state-cas "jobs.j7.owner" "me" nil)   ;; => #t: released, so it can be claimed again
(state-swap "config.mode" "fast")      ;; => the value it replaced (nil when absent or "")
```

- **Paths and links.** Each resolves its path under the program's chroot, and through a
  transparent link exactly as `state-get` reads it. A writable transparent link writes its
  target. A transparent link that isn't writable raises the read-only error (a `NativeError`
  whose `native_type` names `read_only_error`) instead of writing the link node as `state-set`
  would. So a compare and its write always see the same node. A read-only node raises the same
  error.
- **Absent nodes.** An absent node is made when the native writes: `state-swap` and `state-incr`
  always, and `state-cas` only when its `expected` is `nil`.
- **`nil`.** A `nil` `expected` matches a node that is absent or `""`. A `nil` new value stores
  `""`. Other scalars are compared and stored as `state-set` stores them (`10` as `"10"`). A list
  or dict is a `TypeError`.
- **`state-incr`.** It reads the node's text as a DSL number (the parser's rules: the whole text
  an integer, else a float). An absent or empty node is 0. Anything else is a `ValueError`, and
  the node is left as it was. An int plus an int (the default delta is 1) stays an int, and an
  overflow is an `ArithmeticError`. Otherwise the sum is a float, stored in its shortest
  round-trip form: `1000000` plus `0.5` stores `"1000000.5"`, not six digits.
- **Signals.** A write that changes the value fires watches and `valueChanged` once. A
  compare-and-set or swap whose new value equals the old one succeeds without writing, and fires
  nothing.

**Which tool.** One node: these three. Several nodes that must change together, or a section
that spans a park: `with-lock` (docs/STATE_EXEC_LOCKS.md). A queue between programs: channels.

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
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_exec/scheduler.h>

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

### Results From a Worker Lane (`cvc::async_lane`)

Slow native work must not run inside a process step: the step deadline is
cooperative, so a blocking native function freezes the thread that pumps the
scheduler. For a native function a program calls and waits on, prefer an
async function ([Async Functions](#async-functions-register_async_fn), next):
it parks at the call, raises its errors there, and is charged to the process.
The helpers here post to a channel instead.

`cvc::async_lane` (`cvc/core/async_lane.h`) is a persistent named
worker thread with a FIFO job queue. `launch_lane_task`, `park_on_lane_task`,
`future_lane_task` and `post_lane_task` run a kernel on a lane and post its
value (or `on_error(message)` if it throws) to a channel that a process,
chrooted or not, receives with `msg-recv`. They mirror the compute-pool
helpers in `cvc/core/async_task.h`, but reuse one thread instead of starting
a thread per call, which suits per-frame work such as a simulation tick.

```cpp
#include <cvc/core/async_lane.h>

cvc::async_lane sim_lane("sim");  // declare AFTER what its jobs capture
se::builtins::register_fn(env, "sim-launch", [&](std::span<const se::value_t>) {
  // Runs on the pump thread: returns the unique '#' reply channel "demo#sim.<n>".
  return se::value_t(cvc::launch_lane_task(sim_lane, sched, ictx.root_path, "demo",
                                           [&] { return run_ticks(); }, on_err));
});
sched.execute("(while t (apply-sim (msg-recv (sim-launch))))", opts);

// Every frame, on the thread that owns the scheduler:
cvc::pump_exec_frame(sched, {&sim_lane});
```

- Post to `#` channels. A `#` channel is the same key inside and outside a
  chroot, and channel policy never refuses it.
- Every task posts exactly one value, even when the kernel throws, the
  `on_error` builder throws, or the lane has been stopped.
- `stop()` (also called by the destructor) runs every queued job, then joins.
- On Emscripten without `-pthread`, lanes run their jobs on the pump thread
  inside `pump_exec_frame` (`lane_mode::deferred`).

### Async Functions (`register_async_fn`)

An **async function** is a native function whose work runs on a host
executor, not in the step that called it
(`docs/STATE_EXEC_EXCEPTIONS.md` section 3). From the program's side,
`(volume-blur v 3)` is an ordinary call that takes a while:

1. The call's **marshal step** runs on the scheduler's thread, in the calling
   step, with the arguments, and returns the **kernel**.
2. The kernel runs on an executor thread. The calling process parks at the
   call, and the scheduler goes on stepping every other process.
3. The process resumes at the call with the kernel's value, or the kernel's
   exception raises there.

#### Registering (C++)

`register_async_fn` (`cvc/state/state_exec/async_host.h`) registers `name`, and
`name-async`, which returns a future. The host is the app's
(`app.exec_async_host()`, for `exec_scheduler()`) or one you build:

```cpp
#include <cvc/state/state_exec/async_host.h>

namespace se = cvc::state_exec;

// (blur v radius): the kernel runs on the app's background task pool.
se::register_async_fn(
    env, app.exec_async_host(), "blur",
    [](std::span<const se::value_t> args) -> se::async_kernel {
      // The scheduler's thread: read the arguments (already private copies).
      se::value_t volume = args[0];
      const int64_t radius = std::get<int64_t>(args[1].v);
      return [volume, radius](se::async_call &call) -> se::value_t {
        // An executor thread: do the work; check for cancellation now and then.
        for (int64_t slab = 0; slab < 64; ++slab) {
          if (call.cancelled())
            throw std::runtime_error("cancelled"); // the result is dropped anyway
          blur_slab(volume, slab, radius);
          call.post(se::value_t(slab)); // progress, to "blur.progress"
        }
        return volume;
      };
    });
```

`async_options` (the last argument) sets:
- `where`: `executor::background()` (the default), `executor::on_lane(name)`, or
  `executor::inline_call()`;
- `priority`: the kernel's place in the background pool's queue;
- `timeout`: seconds, past which the call raises `TimeoutError`;
- `future_variant`: whether to register `<name>-async` too (the default).

A host without a parking scheduler (`async_host(&sched)` on the sync scheduler,
`async_host(nullptr)` for a bare evaluator) runs every kernel inline, in the
step. The value or exception is the same, and the step blocks.

#### Registering (Python)

```python
host = pycvc.async_host(sched, app)  # or Exec.async_host()

def blur(call, volume, radius):  # with_call: the handle comes first
    for slab in range(64):
        if call.cancelled():
            raise RuntimeError("cancelled")
        blur_slab(volume, slab, radius)
        call.post(slab)
    return volume

# env, host, name, fn, executor, timeout, future_variant, with_call (positional)
pycvc.builtins.register_async_fn(env, host, "blur", blur, "background", None, True, True)
exe.register_async_fn("blur", blur, True)  # an Exec: its own host, background pool
```

The callable runs on an executor thread and takes the GIL for its call; the
wait for the GIL isn't charged. Without `with_call` it gets only the arguments.
Call the handle only from the kernel, on its thread. `executor` is
`"background"`, `"inline"` or `"lane:<name>"`, a lane given to
`host.add_lane(name, pycvc.async_lane(name))`; the host keeps the lane alive.

#### Calling

- `(name args...)` parks and returns the value. It works at any depth: as the
  whole program, inside a `try`, inside a function a native called.
- `(name-async args...)` returns a future at once:
  - `(await f)` parks until it finishes and returns its value or raises;
  - `(await f seconds)` raises `TimeoutError` if it hasn't finished by then,
    and the future can be awaited again;
  - `(await-all (list f1 f2))` returns every value in order, or raises the
    first failure in list order once all have finished;
  - a future never awaited still finishes, and its result is dropped.
- `is_async_future(v)` (C++ and pycvc) says whether a value is such a future.
- Where a process can't park (Ariadne's reactive lane), the call raises
  `RuntimeError` naming the function.

#### Errors

A kernel that throws raises at the call, with `origin` `"async"` and
`function` set: a C++ exception maps by its type, a Python one becomes a
`PythonError` that keeps it, and a timeout is a `TimeoutError`.

```lisp
(try (blur v 3)
  (catch (MemoryError e) "too big")
  (catch (e) (str (get-attr e "function") ": " (get-attr e "message"))))
```

A result that describes a failure (an HTTP reply with `ok` `#f`) is a value,
not an exception.

#### What a kernel may touch

- **Arguments:** private copies. DSL lists and dicts are copied before the
  marshal step (as deep-data work, charged); a `data_object` (a volume, a mesh)
  is shared and read-only.
- **The result:** a fresh value.
- **Never:** the state tree, the evaluator, or a DSL value the program holds.
  Resolve tree paths and channels in the marshal step.

#### Cancellation, progress, teardown

- `call.cancelled()` turns true when the process is killed or reaped
  (including by its budget running out while the call runs), the call times
  out, its owner group is reaped, or the host shuts down. A cancelled call's
  result is dropped.
- `call.post(value)` sends `value` to `<name>.progress` under the process's
  root, for `msg-recv` or a resident. Nothing waits for it; a full queue drops
  it.
- `host.shutdown()` (also `~app` and `~Exec`) cancels every call in flight and
  waits for its kernel.
- A snapshot records a pending call, but it can't resume: the restored process
  raises `RuntimeError` at the call.

#### Budgets

A call is charged to the process that made it, as if the process had done the
work (`docs/STATE_EXEC_EXCEPTIONS.md` 3.8):

- **Run time** (the kernel's wall time, not its time queued) counts toward the
  first half of `max_time`, and its **CPU time** (its thread's, plus its
  `computePool().parallel_for` chunks') toward the second.
- **Steps:** the CPU time at `limits.async_steps_per_cpu_ms` steps per CPU
  millisecond (default 8000; at least one step per call), against `max_steps`.
  The rate is read live; the first of these that is set wins:
  `execute_options::async_steps_per_cpu_ms`,
  `state_exec.schedulers.<id>.limits.async_steps_per_cpu_ms`,
  `state_exec.defaults.limits.async_steps_per_cpu_ms`.
- **Memory:** the kernel runs under the process's allocation limit. Report
  large allocations with `eval_alloc_check(bytes, who)`; past the limit the call
  raises `MemoryError`.
- At `max_time` or `max_steps` the process is killed, as for a step, and the
  kernel sees `cancelled()`. Each `cancelled()` call also records the thread's
  CPU time; a kernel that never checks is stopped by its run time.
- `process_info` reports `async_calls`, `async_time`, `async_cpu_time` and
  `async_steps`.

A short budget limits what a call can do: at the default rate, 5000 steps are
0.625 ms of CPU. Ariadne's actions run with `ariadne.limits.action_max_steps`
(5000) and `ariadne.limits.action_max_time` (0.1 s), app-wide settings a
document can't change, so a host whose documents call heavy async verbs
raises them (`docs/ARIADNE.md` 2.2).

#### Ariadne verbs

`AppRuntime::register_async_verb(name, work)` makes a host verb an async
function on the app's host: `(name)` parks the action while `work` runs on the
background task pool, and returns its string or raises its exception. The
overload with a done channel also posts the result there, for documents that
wait with `msg-recv`. A verb that needs arguments, progress or cancellation
registers with `register_async_fn` through `register_action_intrinsics`, as
the nav_compute example does.

---

### Locks (`with-lock`)

`with-lock` runs a section of a program holding a named lock on part of the state tree, so that
several programs that read and write the same nodes don't interleave. The design is
[`STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md); the exact protocol is
[`STATE_EXEC_LOCKS_PROTOCOL.md`](STATE_EXEC_LOCKS_PROTOCOL.md). Locks are **advisory**: they keep
out other `with-lock` sections, not plain `state-set`s. One node changed alone needs no lock:
`state-incr`, `state-cas` and `state-swap` are atomic ([State Tree Operations](#state-tree-operations-dsl)).

```lisp
(with-lock "inventory"                       ; X on <chroot>.inventory and everything below it
  (state-set "inventory.apples" (- (int (state-get "inventory.apples")) 1)))

(with-lock "inventory" :shared (report))     ; readers share
(with-lock (list "accounts.a" "accounts.b") :timeout 0.5 :as g (transfer))  ; both, or neither
(with-lock "cache" :if-available :else (state-get "cache.v") (refresh))      ; never waits
```

**The form.** `(with-lock KEY option... body...)`, the value of the last body form (nil with no
body). Options come right after `KEY`, in any order:

| Option | Meaning |
|---|---|
| `:exclusive` (the default) / `:shared` | the mode: exclusive excludes every other section on an overlapping key; shared sections share, and exclude only exclusive ones |
| `:timeout SECONDS` | give up after this long (`TimeoutError`); 0 never waits; without it, the context's default (none: wait as long as it takes) |
| `:if-available` | never wait: if the lock is busy, run `:else` instead (nil without one), holding nothing |
| `:else FORM` | with `:if-available` only |
| `:as NAME` | binds NAME in the body to `{key mode generation abandoned}` (nil in `:else`) |

An unknown option, an option given twice (the mode counts once), `:else` without `:if-available`,
an option with no form after it, and `:as` without a symbol are `ValueError`s, raised before
anything is evaluated or locked. `with-lock` is a soft form: it is whatever the name is bound to,
so a program's own `defun`, `defmacro` or `set` of the name replaces it.

**Keys.** A key is a state-tree path, relative to the program's chroot like `state-get`'s:
segments are split on `.` and trimmed, and empty ones are dropped (`" a..b "` is `a.b`). An
empty key, or one with `#`, is a `ValueError`. A key with a leading `/` is app-global, allowed
exactly where a `/` channel is: when the program's channel policy enforces, only a key equal to
or under a declared global, else `PermissionError`. `KEY` may be a list: every key at once, or
none (keys under one another collapse to the outer one). Two keys conflict on segment boundaries:
`a.b` overlaps `a` and `a.b.c`, not `a.bc`.

**Waiting.** Requests are granted in arrival order among those that conflict; non-conflicting
ones pass freely. A section nested in one the program already holds is granted at once when the
outer one covers it (same key or an ancestor, and a mode that allows it), counted, not locked
again. A nested request that would need an upgrade (exclusive under the program's own shared
hold) raises `LockError` with `reason` `"upgrade"`. A program waits only where its process can
park: in a nested evaluator, a generator, or under Ariadne's reactive lane a busy request raises
`LockError` (`"cannot wait here"`, `"generator"`); a context that forbids lock waits (Ariadne's
`init:` lane) treats every busy request as `:timeout 0`. Signal and watch handlers may not take a
lock the program they interrupted holds, or one that conflicts with it (`"handler"`).

**Release.** A section's lock is released when the body ends, however it ends: a value, a
`return`, a `break`, or an exception (before an outer `finally` runs). A kill, a budget running
out, or an uncaught exception ending the process release its locks too; the next exclusive
section on such a key sees `abandoned: #t` through `:as`, once.

**Deadlocks** are detected exactly: the request that would close a cycle of waiting programs
raises `DeadlockError` at once, its `cycle` a list of `{pid key}` (a pid only for a process of
the same scheduler and owner). A `try` around the `with-lock` can catch it and retry.

**Reads.** `(locks-held)` lists the program's own holds, outermost first:
`{key mode count generation}`. `(lock-query [prefix])` lists every lock under the prefix (the
program's chroot by default): `{held: [{key mode pids count generation abandoned}], waiting:
[{key mode pid}]}`, showing pids only for processes of the same scheduler and owner.
`(inspect pid)` adds `locks_held` and `waiting_on` (`"lock k"`, `"msg channel"`, `"sleep"`).

**Fork and snapshots.** `(fork)` inside a section raises `LockError` (`"fork"`). A snapshot of a
process that holds a lock is refused, naming its keys; a waiter can be snapshotted, and restored
it asks again, at the back of the queue, with what was left of its timeout. A restore whose
environment has no `with-lock` lock form raises `LockError` (`"restore"`) in the restored
program.

**Hosts.** Each scheduler has its own lock manager, made when a program first locks, unless the
host gives it one with `set_lock_manager`. The app's `exec_scheduler()` uses the app's manager
(`app::lock_manager()`). Ariadne's actions give up after `ariadne.exec.lock_timeout` seconds (1.0)
when they name no `:timeout`. pycvc's `register_intrinsics` keeps the lock form's context alive,
and `intrinsics_context.lock_wait_allowed` / `lock_default_timeout` set the context's rules.

## 7. Resource Limits & Policies

### Per-Process Limits

Set via `execute_options` at submission time or adjusted at runtime:

```cpp
execute_options opts;
opts.max_steps         = 10000;     // evaluator steps
opts.max_time          = 5.0;       // wall time in seconds
opts.max_memory        = 1048576;   // bytes: state-tree writes + the measured heap share
opts.max_messages      = 100;       // outbound msg-send calls
opts.max_message_bytes = 65536;     // total outbound payload bytes
```

When a limit is exceeded, the process is killed (status becomes `killed`). The async scheduler
keeps the reason as its exit error: `max_steps_exceeded`, `time_limit_exceeded`,
`memory_limit_exceeded`, and so on.

`max_time` is checked between steps, and inside a step by the step's deadline. A long loop
inside one native call polls that deadline with `eval_deadline_check()`, which throws
`evaluation_timeout`. Examples are an unsliced data job, a generator body that never yields, and
a stdlib materializer. Either way the process is killed with `time_limit_exceeded`. On the sync
scheduler, a deadline armed outside it (its caller's) propagates out of `run()` instead.

Outside a scheduler, every evaluator's timeout arms the same deadline. A loop inside a step
throws `evaluation_timeout` (in pycvc, `pycvc.evaluation_timeout`), as the evaluator's own
check between steps does. An Ariadne predicate reports either one as the eval budget's time cap.

### Memory: state-tree writes and the measured heap

`max_memory` covers two things:
- what the process wrote to the state tree, counted by the scheduler's `memory_tracker`;
- its share of the values it holds in memory (lists, dicts, strings, closures, environments),
  measured by walking what it can reach (`heap.h`).

A value can be shared between processes. A broadcast message is one dict held by every
receiver, and a forked child shares its parent's lists. So a process pays its **share** of each
node it reaches: the node's size × (its references ÷ the node's owner count), and a node passes
its share on to the nodes it holds. Summed over all holders, a node's shares come to its size: a
1 MB message held by 50 residents costs each about 20 KB, and as holders drop it, those left pay
more at their next measurement.

A measurement is the process's own deep-data work, run a slice per step like other data work
(`data.step_nodes`) and counted against its steps and time. One starts after a step when any of
these holds:

- natives have reported allocating 64 KiB, or half the last measurement if that is more, since
  the last one (the functions with an allocation check, §11, plus copies like `cons` and
  `slice`, and the jobs of calls left waiting on the evaluator);
- what they reported could cross the remaining budget;
- `limits.heap_check_steps` steps have passed (default 4096; `0` = only the two above).

A measurement can start while the process waits on other deep-data work, such as a call the
evaluator makes for a native function: it runs first, and the work goes on after it. It counts
that work too, with what the work's jobs hold (a waiting call's function and arguments, the
values `collect` has gathered). So a process that never stops waiting, like a class whose `init`
constructs again, is still held to `max_memory`.

The kill decision always uses an actual measurement, never the reported bytes, so temporaries a
program drops don't count. Only a process with `max_memory > 0` is measured.
`process_info::heap_memory` and `heap_measurements` report the last measurement. The per-step
allocation limit (§11) takes what is left after state-tree writes and the measured heap.

### Reference cycles

Values are reference counted, and reference counting can't free a cycle. These all leaked their
whole scope, with everything in it, before the collector existed:
- a closure bound in the scope it captured: every `defun`, `(set f (lambda …))`, a `let`-local
  recursive lambda;
- a class, whose methods capture the scope the class is bound in;
- a container that holds itself, as after `(append l l)`.

`collect_cycles()` (`heap.h`) finds the cycles nothing outside them holds, the way CPython does:
- It tracks the environments that closures, generators and class methods capture, and the
  containers changed in place to hold another container (`append`, `set-nth`, `set-attr`). Only
  those can close a cycle.
- It counts the references these objects (and everything they reach) make to each other. Anything
  with more owners than that is held from outside: by a live process, a host (pycvc's `Exec`, an
  Ariadne action context), or anything else. Such an object stays, along with everything it
  reaches.
- What is left is garbage. Clearing it lets it free.

Both schedulers collect between steps once a thread has tracked as many new objects as survived
the last collection, and at least an eighth as many as it walked (at least 256). That keeps a
collection's cost proportional to what was tracked since the last one. They also collect at most
a second after a process ended. A finished
process also releases its evaluation state right away (`process::release_state`; result, error
and statistics stay), so what it leaked can be collected before the process is reaped. Tracking
is per thread. A host that runs evaluators without a scheduler calls `collect_cycles()` itself,
between steps; inside a step it does nothing.

### Cluster-Level Resource Policy

A `resource_policy` defines cluster-wide constraints that apply to all
processes submitted through an `exec_coordinator`:

```cpp
#include <cvc/state/state_exec/resource_policy.h>

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
| `limits.max_alloc_bytes` | `state_exec.schedulers.<id>.limits.max_alloc_bytes` | `state_exec.defaults.limits.max_alloc_bytes` | `16777216` ([§11](#allocation-limit)) |
| `limits.heap_check_steps` | `state_exec.schedulers.<id>.limits.heap_check_steps` | `state_exec.defaults.limits.heap_check_steps` | `4096` ([heap](#memory-state-tree-writes-and-the-measured-heap)) |

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
- `state_exec.schedulers.<id>.limits.max_alloc_bytes` — the resolved per-call allocation limit
- `state_exec.schedulers.<id>.limits.heap_check_steps` — the resolved heap re-measure interval
- `state_exec.schedulers.<id>.policy` — `"round_robin"`, `"priority"`, or `"priority_rr"`

---

## 8. Distributed Execution

### The exec_coordinator

The `exec_coordinator` manages distributed process execution across
a cluster of nodes.  It handles leader election, process submission
routing, migration, cross-cluster observation, and admin controls.

```cpp
#include <cvc/state/state_exec/exec_coordinator.h>
#include <cvc/state/state_message_bus.h>

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

Four modules of native functions sit beside the builtins: `string`, `bytes`, `math` and
`collections`. Every environment made by `builtins::make_default_environment()` reaches them,
so every host does without an import step:

- the sync and async schedulers' default environment (`cvc exec`, `app::exec_scheduler()`);
- Ariadne's `init:`, action (`on:`) and resident lanes;
- pycvc's `Exec`.

```lisp
(string.length "café")        ;; => 4
(math.sqrt 16)                ;; => 4
(collections.reverse (list 1 2 3)) ;; => (3 2 1)
```

Every name carries its module prefix, so none clashes with a builtin: `length`, `range` and
`slice` are still the builtins, and `string.length` or `collections.range` are the stdlib.

### How the modules are loaded

`stdlib_registry::shared_scope()` builds one scope holding every function of
`stdlib_registry::standard()`, once per process. `make_default_environment()` links it as its
outer scope, so a lookup that misses the builtins falls through to it. Nothing is copied into
each environment.

The shared scope is frozen. `set`, `defun` and `environment::set_existing()` never write into
it: a program that rebinds a stdlib name shadows it in its own scope, and other programs keep
the real function.

```lisp
(set string.length 42)
string.length                 ;; => 42 in this program only
```

Downstream C++ code can build its own `stdlib_registry`, add modules with
`register_function()`, and import them into an environment it owns:

```cpp
#include <cvc/state/state_exec/stdlib.h>

stdlib_registry mine;
mine.register_function("geo", "geo.area", area_fn);
mine.import_module("geo", env);                       // every function of the module
mine.import_module("geo", env, {"geo.area"});         // or a subset
```

### Allocation limit

The per-process `max_memory` check runs between steps, and the heap it counts is measured after
the fact (§7), so neither can stop `(collections.range 1000000000)` from allocating. Each
function whose result can be larger than all of its arguments therefore works out the result's
size and fails before building it when it is over the limit. A sliced function (next section)
checks as soon as it has counted, before it builds anything. The checked functions are:

- `string.split`, `string.join`, `string.replace`, `bytes.concat`;
- `collections.range`, `collections.zip`, `collections.flatten`, `collections.dict-merge`;
- the builtins `str-concat` and `collect`.

The limit for a step is the tighter of two values:

1. the tree setting `limits.max_alloc_bytes` (table below; `0` means no limit of its own);
2. what is left of the process's `max_memory`, when it has one: after its state-tree writes and
   its last measured heap share (§7).

| Setting | Per-scheduler path | Global path | Default |
|---------|-------------------|-------------|---------|
| `limits.max_alloc_bytes` | `state_exec.schedulers.<id>.limits.max_alloc_bytes` | `state_exec.defaults.limits.max_alloc_bytes` | `16777216` (16 MiB) |

The sync scheduler resolves it in `load_settings()` and publishes the effective value under
`state_exec.schedulers.<id>.limits.max_alloc_bytes`. The async scheduler (the app's exec
scheduler, and the one inside pycvc's `Exec`) reads the global path on every submission. A
bare evaluator, or a native called directly from C++, gets the 16 MiB default.

A call over the limit is a runtime error naming the function. The async scheduler kills the
process with it as the exit error; the sync scheduler's `run()` throws it:

```lisp
(collections.range 1000000000)
;; error: collections.range: the result needs <n> bytes, over the allocation limit of
;;        16777216 bytes (limits.max_alloc_bytes, or what is left of max_memory)
```

### Sliced work

These functions do their work as a data job (`data_job.h`; "Deep data on the executor" in
`docs/STATE_EXEC_GRAMMAR.md`): `string.split`, `string.join`, `string.replace`,
`string.encode`, `bytes.slice`, `bytes.concat`, `bytes.decode`, every `collections.*`
function, and the intrinsics `state-json` and `state-from-json`.

`state-json` and `state-from-json` run `cvc::state`'s own writer and reader
(`cvc/state/state_json.h`), the code behind `json()`, `json(j)`, `save()` and `restore()`, so
they produce and accept the same documents. The writer's unit is a node visited. The reader's
is a value scanned, and it scans each value twice: once to check the document, once to set it.
Both also count one unit per 256 bytes. The writer holds the nodes it has yet to visit, so a
node another process removes between slices is still written, as it was when its parent was
read. The document is held to the allocation limit as it grows.

- **Slices.** A slice is `data.step_nodes` nodes: one per element visited, and one per 256
  bytes (`kDataNodeBytes`) scanned or copied. With `data.step_nodes` = N the call does one
  slice in its own step. The rest runs N nodes per **data step** while the call waits. Data
  steps count against `max_steps` and `max_time`, and other processes run between them. A call
  that fits in one slice takes no extra step.
- **Defaults.** The app's exec scheduler, pycvc's `Exec` and Ariadne predicates slice 1024
  nodes a step (`async_scheduler::host_data_defaults()`). With 0, the sync scheduler's default,
  the work runs whole in the call's step.
- **Calls.** `collections.map`, `filter` and `reduce` hand each call of their function to the
  evaluator running the program, and so do the builtins `apply` and `send`. A native function
  is called right away when the native function the step applies asks for it, and by the
  evaluator, in a data step, when a native function deeper than that asks (so native calls nest
  at most one level; `docs/STATE_EXEC_GRAMMAR.md`, "Native call nesting"). A DSL function runs
  as the process's own frames, so its body's steps are eval steps. The job goes on in the data
  step after it returns, even with `data.step_nodes` = 0. Another native function can do the
  same with `call_function(fn, args, who)`.
- **Cost.** `collections.sort` is a stable merge sort, n log n nodes.
  `collections.unique` costs a node per comparison, and compares lists and dicts with a sliced
  equality walk. `collections.dict-merge` keeps a key index, so it is linear. Every job polls the
  evaluation deadline (`max_time`) about every 64 nodes, and throws `evaluation_timeout` once it
  has passed.
- **While a call runs.** Signals and watch events wait until the call is done, and `fork`
  returns -1. A snapshot taken during it restores to the call's start: the call runs again, and
  calls its function again from the first element.
- **Measured** (`state_exec_bench_test` `StdlibJobThroughput`, 1024-node steps, MSVC Release):
  - about 5-20 ns a node for split, join, replace and sort;
  - about 200-370 ns a node for a 256-byte copy such as `string.encode`, including first-touch
    page faults;
  - about 60 ns an element for `collections.map` with a native function, and about 400 ns with
    `(lambda (x) x)`, whose body's eval steps are included.

### In Ariadne predicates

A per-frame predicate (`visible_when`, `bind`, `fmt`, …) runs in a reduced environment. It
gets every stdlib function except `math.random`, which would make a predicate
nondeterministic.

The materializers are safe there because their work is sliced. The step and time caps are
checked between steps, and a predicate always slices: it uses the app tree's
`state_exec.defaults.data.step_nodes`, with 1024 for 0 or unset. A huge call is cut by the caps
(fail-safe, with a warning) instead of stalling the frame, and each call stays under the 16 MiB
allocation limit.

A function a predicate passes to `collections.map`, `filter` or `reduce` runs on the
predicate's own evaluator. The special-form gate, the reduced environment and the caps cover
its body too. The `init:` and action lanes get every function.

### Module: `string` (14 functions)

Indices and lengths count code points, not bytes (`UNICODE_SUPPORT_ROADMAP.md` SE-1).

```lisp
(string.split "a,b,c" ",")           ;; => ("a" "b" "c")
(string.split "né" "")               ;; => ("n" "é")   one element per code point
(string.join (list "a" "b" "c") ",") ;; => "a,b,c"
(string.replace "hello" "l" "r")     ;; => "herro"
(string.trim "  hello  ")            ;; => "hello"
(string.upper "hello")               ;; => "HELLO"   ASCII letters only
(string.lower "HELLO")               ;; => "hello"
(string.starts-with "hello" "he")    ;; => #t
(string.ends-with "hello" "lo")      ;; => #t
(string.contains "hello" "ell")      ;; => #t
(string.substring "hello" 1 3)       ;; => "ell"   (start, length)
(string.char-at "café" 3)            ;; => "é"
(string.length "café")               ;; => 4
(string.byte-length "café")          ;; => 5
(string.encode "café")               ;; => b"caf\xc3\xa9"   utf-8 only
```

### Module: `bytes` (5 functions)

Byte semantics throughout; `string.encode` and `bytes.decode` are the only crossings between
text and bytes.

```lisp
(bytes.length b"\x01\x02")           ;; => 2
(bytes.byte-at b"\x01\xff" 1)        ;; => 255
(bytes.slice b"abcd" 1 2)            ;; => b"bc"   (start, length)
(bytes.concat b"ab" b"cd")           ;; => b"abcd"
(bytes.decode b"caf\xc3\xa9")        ;; => "café"  invalid UTF-8 is an error
```

### Module: `math` (16 functions)

```lisp
(math.sqrt 16)       ;; => 4
(math.abs -5)        ;; => 5
(math.floor 3.7)     ;; => 3       an integer
(math.ceil 3.2)      ;; => 4       an integer
(math.round 3.5)     ;; => 4       an integer
(math.pow 2 10)      ;; => 1024
(math.log 1)         ;; => 0
(math.sin 0)         ;; => 0
(math.cos 0)         ;; => 1
(math.tan 0)         ;; => 0
(math.min 3 7 1 5)   ;; => 1       a float
(math.max 3 7 1 5)   ;; => 7       a float
(math.clamp 15 0 10) ;; => 10      a float
(math.pi)            ;; => 3.14159
(math.e)             ;; => 2.71828
(math.random)        ;; => a float in [0, 1)
```

### Module: `collections` (12 functions)

`collections.map`, `filter` and `reduce` take the list first, then a function. It can be a
builtin, a stdlib function, a host function (pycvc's `register_fn`) or a DSL function
(`lambda`, `defun`). A DSL function runs as the program's own steps (see Sliced work), and
`return` in it returns from it. C++ that calls the registry's function directly, with no
running program, can pass only a native function.

```lisp
(collections.map (list 1 4 9) math.sqrt)          ;; => (1 2 3)
(collections.map (list 1 2 3) (lambda (x) (* x x))) ;; => (1 4 9)
(collections.filter (list 1 "a" 2) is-int)        ;; => (1 2)
(collections.filter (list 1 2 3 4) (lambda (x) (= (% x 2) 0))) ;; => (2 4)
(collections.reduce (list 1 2 3) + 0)             ;; => 6
(collections.reduce (list 1 2 3) (lambda (acc x) (+ acc (* x x))) 0) ;; => 14
(collections.zip (list 1 2) (list "a" "b"))       ;; => ((1 "a") (2 "b"))
(collections.flatten (list (list 1 2) 3 (list 4))) ;; => (1 2 3 4)   one level
(collections.sort (list 3 1 2))                   ;; => (1 2 3)   numbers by value, strings by bytes; stable
(collections.reverse (list 1 2 3))                ;; => (3 2 1)
(collections.range 1 5)                           ;; => (1 2 3 4)
(collections.range 5 0 -2)                        ;; => (5 3 1)
(collections.unique (list 1 2 2 3 3))             ;; => (1 2 3)
(collections.dict-keys (dict "a" 1 "b" 2))        ;; => ("a" "b")
(collections.dict-values (dict "a" 1 "b" 2))      ;; => (1 2)
(collections.dict-merge (dict "a" 1) (dict "b" 2)) ;; => {"a": 1, "b": 2}
```

---

## 12. Gotchas & Pitfalls

This section collects non-obvious behaviours that can bite you when
writing DSL programs, setting up state replication, or integrating
the state_exec system with C++ hosts.

### 12.1 First Write to a New Path Is Not Replicated

The `state_sync_adapter` attaches observers **lazily**.  When a node is
created (e.g. via `state-set` or `cvc::state::operator()(path).value(v)`),
the internal sequence is:

1. `valueChanged()` fires on the new node — **no observer yet**.
2. `childChanged()` fires on the parent and on every ancestor above it,
   the root's slots first (`state::emitChildChanged()`).
3. The adapter's `childChanged` handler attaches an observer on the
   new node.

Because step 1 happens before step 3, the **very first value written
to a brand-new path** is never captured by the journal and therefore
never replicated to peers.

**Workaround — seed paths before real writes:**

```cpp
// Create the node (triggers observer attachment) with a throwaway value.
root("myapp.status").value(std::string("seed"));

// Now the adapter has an observer — this write IS replicated.
root("myapp.status").value(std::string("running"));
```

In DSL code the same principle applies: call `state-set` with a
dummy value before the real write when the path has never been
accessed before, or seed the path from C++ before the program starts.

### 12.2 Duplicate-Value Writes Are Silently Dropped

`state::value(v)` guards with `if (value() == v) return *this;` and
skips the write entirely — no `valueChanged()` signal, no journal
entry, no replication:

```cpp
root("k").value(std::string("hello"));
root("k").value(std::string("hello"));  // no-op: no signal, no journal entry
```

If you need to "re-publish" the same value (e.g. to signal a peer),
either use `state::touch()` or write a distinct sentinel first.

### 12.3 Pumping Before a Peer Connects Loses Mutations

`state_cluster_shard::drain_local()` returns pending mutations **and
permanently advances the publish cursor**.  If no peer is connected
when `pump_all()` is called, the mutations are drained but never sent
— and they will **not** be re-sent on subsequent pumps.

```cpp
// BAD — peer may not be connected yet
shard.attach();
transport.start(sock, "A", "C");
root("data").value(std::string("important"));
transport.pump_all();   // drains cursor; if no peer, data is lost
```

**Fix — wait for a peer before pumping:**

```cpp
shard.attach();
transport.start(sock, "A", "C");

auto deadline = steady_clock::now() + 5s;
while (transport.connection_count() == 0 && steady_clock::now() < deadline)
    std::this_thread::sleep_for(5ms);

root("data").value(std::string("important"));
transport.pump_all();   // peer is connected — data is sent
```

### 12.4 `let` Creates a New Scope — Variables Don't Escape

`let` introduces **local bindings** that are not visible outside the
block.  To carry state across iterations, use `set`:

```lisp
;; WRONG — i is scoped to the let body; the outer i never changes
(set i 0)
(while (< i 5)
  (let ((i (+ i 1)))
    (print i)))  ;; infinite loop!

;; RIGHT — use set for loop-carried state
(set i 0)
(while (< i 5)
  (begin
    (print i)
    (set i (+ i 1))))
```

### 12.5 All State Values Are Strings

The state tree stores everything as `std::string`.  `state-set`
turns a non-string argument into text (§5), and `state-get` always
returns a string.  Convert explicitly for numeric work:

```lisp
(state-set "counter" (str 42))
(set raw (state-get "counter"))   ;; raw is "42" (string)
(set n (int raw))                  ;; n is 42 (integer)
(if (> n 40) "big" "small")       ;; => "big"
```

### 12.6 Generators Are Single-Use

A generator can be iterated **exactly once**.  After exhaustion,
`next` returns `nil` and `for` produces no values:

```lisp
(set g (range 3))
(collect g)        ;; => (0 1 2)
(collect g)        ;; => ()  — already exhausted

;; To reuse, wrap in a factory
(defun make-range () (range 3))
(collect (make-range))   ;; => (0 1 2)
(collect (make-range))   ;; => (0 1 2)
```

### 12.7 `msg-recv` Suspends the Process

`msg-recv` is a **blocking** intrinsic.  If no message is available,
the calling process enters `waiting` status and yields to the
scheduler.  In a single-process program with no external sender this
creates a deadlock:

```lisp
;; DEADLOCK — nothing else is running to send a message
(msg-recv "work.queue")
```

Always pair with a producer (another spawned process or the host)
or check `msg-pending` first.

### 12.8 `sleep` Granularity Depends on the Scheduler Tick

`sleep` records a wake-up time and suspends the process.  The process
resumes on the **next `step()` after** the duration expires.  If the
host calls `step()` infrequently (e.g. a pump loop with a 50 ms
sleep), the effective granularity is limited by that polling interval,
not by the requested duration.

A host that pumps with `sync_run()` alone never wakes a sleeper when nothing
else is runnable: `sync_run()` only steps while `has_runnable()` is true, and
sleepers are woken inside `step()`. A per-frame pump must therefore call
`sync_step()` unconditionally, after `drain_ingress()` (which delivers
cross-thread `post_message` values) and `wake_awaiting()`, and before
`sync_run()`. `cvc::pump_exec_frame()` (`cvc/core/async_lane.h`) does this
in the right order.

### 12.9 `break` Exits Only the Innermost Loop

`break` targets the nearest enclosing `while` or `for`.  There is no
labelled-break or "break N".  To exit an outer loop from an inner
one, set a flag and check it in the outer condition:

```lisp
(set done false)
(for i (range 10)
  (if done nil
    (for j (range 10)
      (if (= j 3)
        (begin (set done true) (break nil))
        nil))))
```

### 12.10 Fork-Based IPC Tests and Socket Cleanup

Tests using `fork()` with `state_transport_ipc` create Unix domain
socket files in `/tmp`.  If a test is killed before cleanup, stale
socket files cause "address already in use" on the next run.  Use
unique socket paths (incorporating PID or a UUID) and
`std::filesystem::remove()` in both setup and teardown.

---

## 13. Architecture Reference

```
┌─────────────────────────────────────────────────────────────────┐
│                        DSL Programs                             │
│  (defun fib (n) (if (<= n 1) n (+ (fib (- n 1)) (fib ...))))  │
├─────────────────────────────────────────────────────────────────┤
│  parser.h        → value_t AST                                 │
│  builtins.h      → +,-,*,/,list,dict,str,...                   │
│  stdlib.h        → string.*,bytes.*,math.*,collections.*       │
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
              stdlib.h ← builtins.cpp (the shared stdlib scope)
         memory_tracker.h ← scheduler.h
```
