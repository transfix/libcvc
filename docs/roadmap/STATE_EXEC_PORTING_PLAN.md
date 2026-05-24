# state_exec: Porting Plan

## Porting txwtf.cfos S-Expression DSL to cvc::state (C++20)

### 1. Overview

**state_exec** is a general-purpose, lisp-like S-expression DSL engine built on
top of `cvc::state`.  It replaces the Python `txwtf.cfos` module with a C++20
implementation whose code *and* runtime state live inside the `cvc::state` tree,
enabling distributed execution across a cluster of `cvc::state` peers.

Key design tenets:

- Programs (code + data), evaluator state (stack frames, environments), and
  scheduler state (process table, scheduling metadata) are all stored as
  `cvc::state` subtrees.
- Execution of a program can be suspended on one node, the relevant subtree
  replicated to a peer, and resumed there with zero out-of-band state.
- Cluster admins can observe, pause, migrate, or kill any program running in
  their cluster from any node.
- The DSL exposes intrinsics for manipulating the `cvc::state` tree, querying
  the scheduler, and controlling execution — programs are first-class citizens
  of the runtime.
- **Evaluator tiers**: Four evaluator implementations exist at two tiers.
  The **`async_stackless_evaluator`** is the default and only evaluator used
  by the scheduler for cluster-managed processes — it is non-blocking (async)
  and serializable (stackless), which are both required for migration and
  distributed execution.  The other three evaluators (`evaluator`,
  `async_evaluator`, `stackless_evaluator`) are available for **local
  evaluation contexts** — embedded scripting, one-shot expressions, testing
  — but programs run through them are *not* first-class scheduler citizens
  and cannot be migrated, paused-and-resumed across nodes, or observed via
  cluster-wide `ps-all`.
- A **standard library** of reusable routines lives in the global state tree
  (`__state_exec__.__global__.__stdlib__`), readable by all programs.
  **Access control is enforced at the cluster consensus level, not locally**:
  a process can write to any path in its local tree, but the cluster's
  replication layer can reject writes that violate per-subtree ACL policies.
  Rejected writes propagate an error to the process but do not crash it —
  the user may choose to ignore the error and keep their local patch.
  Downstream C++ libraries can register additional stdlib modules, making
  the DSL extensible without recompilation.
- Programs can read and write **data objects** (`boost::any` via
  `state::data()`) — not just string values — on state nodes and local
  variables.  The `data_object` variant bridges the DSL with the typed C++
  ecosystem (geometry, volumes, scene graphs via `state_object<T>`).
- **Batched state changes** are exposed to DSL programs via `(state-batch)`
  and `(state-lock)`, leveraging `state_object<T>`'s `beginBatch()`/
  `endBatch()` and `state_lock_scope` to coalesce change notifications and
  ensure consistency across multi-field updates.

---

### 2. Source Material (txwtf.cfos)

| Python file | Lines | Role |
|---|---|---|
| `txwtf/cfos/script.py` | 3 975 | S-expression parser, 4 evaluator variants (sync recursive, async recursive, sync stackless, async stackless), built-in functions, serializable `EvaluatorState`/`EvalFrame`/`Closure`, macro system, OOP (`defclass`/`send`/`super`), `RestrictedPython` sandboxed builtins |
| `txwtf/cfos/scheduler.py` | 1 873 | `CFOSScheduler` (sync) and `AsyncCFOSScheduler` — process abstraction with PID/UID/GID, scheduling policies (round-robin, priority, priority-RR), signal handling, serialization, parallel stepping via `ThreadPoolExecutor` / `asyncio.gather` |
| `tests/test_cfos_script.py` | 3 214 | ~120 tests covering parsing, built-ins, special forms, closures, macros, OOP, stackless serialization, pause/resume, async variants, stats, callbacks |
| `tests/test_cfos_scheduler.py` | 3 553 | ~130 tests covering process lifecycle, all scheduling policies, signal handling, concurrency, serialization/cross-machine resume, UID/GID, max-steps, timeouts |

All references to "CFOS", "Crypto Financial Operating System", and any
finance-specific terminology will be removed.  The new namespace is
`cvc::state_exec`.

---

### 3. Architecture

```
cvc::state tree
└── __state_exec__                        (registry root)
    ├── __clusters__
    │   └── <cluster_id>
    │       ├── __scheduler__             (scheduler metadata)
    │       │   ├── policy                (round_robin | priority | priority_rr)
    │       │   ├── next_pid
    │       │   ├── rr_index
    │       │   └── total_steps
    │       ├── __processes__
    │       │   └── <pid>
    │       │       ├── __meta__
    │       │       │   ├── name
    │       │       │   ├── status        (ready|running|paused|waiting|terminated|killed)
    │       │       │   ├── priority
    │       │       │   ├── uid           (client/node identity)
    │       │       │   ├── gid
    │       │       │   ├── max_steps
    │       │       │   ├── timeout
    │       │       │   ├── max_time       (wall-clock time limit, 0 = unlimited)
    │       │       │   ├── max_messages   (outbound message limit, 0 = unlimited)
    │       │       │   ├── create_time
    │       │       │   ├── exit_code
    │       │       │   ├── parent_pid     (PID of parent if forked, -1 if root)
    │       │       │   └── owner_node    (node_id that launched)
    │       │       ├── __program__        (the S-expr AST stored as ordered list)
    │       │       ├── __evaluator__
    │       │       │   ├── __stack__      (ordered list of frames)
    │       │       │   │   └── <index>
    │       │       │   │       ├── expr   (serialized expression)
    │       │       │   │       ├── phase
    │       │       │   │       ├── index
    │       │       │   │       ├── results (ordered list)
    │       │       │   │       └── env    (environment chain)
    │       │       │   ├── result
    │       │       │   ├── done
    │       │       │   ├── root_expr
    │       │       │   └── global_env
    │       │       ├── __stats__
    │       │       │   ├── step_count
    │       │       │   ├── start_time
    │       │       │   ├── end_time
    │       │       │   └── message_count  (outbound messages sent)
    │       │       ├── __memory__
    │       │       │   ├── max_bytes      (memory limit, 0 = unlimited)
    │       │       │   ├── current_bytes  (sum of owned state tree objects)
    │       │       │   └── peak_bytes     (high-water mark)
    │       │       ├── __signals__
    │       │       │   ├── pending       (ordered list)
    │       │       │   └── handlers      (signal_name → handler expr)
    │       │       ├── __subscriptions__   (active message bus subscriptions)
    │       │       │   └── <sub_id>
    │       │       │       ├── path_prefix   (subscribed path)
    │       │       │       └── handler       (serialized closure)
    │       │       ├── __watches__          (active state-watch registrations)
    │       │       │   └── <watch_id>
    │       │       │       ├── path          (watched state path)
    │       │       │       └── handler       (serialized closure)
    │       │       └── __expiry__         (cvc::state expiry metadata)
    │       │           ├── mode           (none|after|at)
    │       │           ├── duration        (boost::posix_time::time_duration, if mode=after)
    │       │           └── deadline        (boost::posix_time::ptime, if mode=at)
    │       ├── __policy__                 (cluster resource policy, §4.15)
    │       │   ├── max_time_min
    │       │   ├── max_time_max
    │       │   ├── max_time_default
    │       │   ├── max_memory_min
    │       │   ├── max_memory_max
    │       │   ├── max_memory_default
    │       │   ├── max_messages_min
    │       │   ├── max_messages_max
    │       │   ├── max_messages_default
    │       │   ├── max_steps_min
    │       ├── __acl__                    (cluster-level ACL, §4.19)
    │       │   ├── admin                 (list of admin UIDs)
    │       │   └── delegates             (subtree path → delegated admin UID)
    │       └── __scheduling_node__       (node_id in charge of scheduling)
    └── __global__
        ├── __macros__                    (user-defined macros)
        └── __stdlib__                    (standard library, §4.17)
            ├── __acl__                   (access control: read=all, write=admin)
            │   ├── read                  "all"
            │   └── write                 "admin"
    │       └── __scheduling_node__       (node_id in charge of scheduling)
    └── __global__
        ├── __macros__                    (user-defined macros)
        └── __stdlib__                    (standard library, §4.17)
            ├── __acl__                   (access control: read=all, write=admin)
            ├── string                    (string manipulation routines)
            │   ├── split                 (defun)
            │   ├── join                  (defun)
            │   ├── replace               (defun)
            │   └── ...                   (other string routines)
            ├── math                      (math routines)
            ├── io                        (I/O: url-fetch, etc.)
            ├── collections               (data structure utilities)
            └── <app>                     (downstream-registered modules)
```

#### 3.1 Ordered Child Lists in cvc::state

`cvc::state` stores children in a `std::map<std::string, state_ptr>` (sorted by
name).  S-expression lists require strict positional ordering.  The approach:

- **Positional key encoding**: child names are zero-padded decimal indices
  (`"0000"`, `"0001"`, …).  A sibling `__len__` node stores the list length.
  The fixed-width padding ensures lexicographic map order equals positional
  order.
- A thin **`state_list`** C++ wrapper provides `push_back`, `at(i)`,
  `pop_back`, `size`, iterator, and `insert`/`erase` over this convention.
- The wrapper will be part of the public `cvc::state` API (new header
  `cvc/state_list.h`) so other subsystems can use ordered lists in the state
  tree.
- Padding width is configurable (default 6 digits → up to 999 999 elements),
  but the wrapper validates and can re-index if needed.

#### 3.2 Value Representation in the State Tree

S-expression values map to `cvc::state` nodes as follows:

| S-expr type | state representation |
|---|---|
| Integer | `state::value<int64_t>()` |
| Float | `state::value<double>()` |
| String | `state::value<std::string>()` |
| Boolean | `state::value<std::string>("t"` / `"nil")` |
| nil | `state::value<std::string>("nil")` |
| Symbol | node with `__type__` = `"symbol"`, `__value__` = name |
| List | node with `__type__` = `"list"`, children = ordered elements via state_list |
| Closure | node with `__type__` = `"closure"`, children for params/body/env_snapshot |
| Dict (DSL object) | node with `__type__` = `"dict"`, children = key-value entries |
| Data object | node with `__type__` = `"data_object"`, `__data_type__` = type name; actual `boost::any` payload stored via `state::data()` |

A `state_value_codec` utility converts between in-memory C++ variant types
(`state_exec::value_t` — see §4.1) and the state tree encoding, and back.

**Data objects and local variables.**  A DSL program can read a state node's
`state::data()` (`boost::any`) into a local variable as a `data_object` value.
This captures both the payload and its type name.  The `data_object` can then
be passed to functions, stored in lists/dicts, compared by type, and written
back to a state node via `(state-data-set)`.  This bridges the gap between
the typed C++ object world (state_object subclasses, geometry data, volumes)
and the DSL's string-based value layer.

When a `data_object` is stored in the state tree (via `state_value_codec`),
the codec writes the `boost::any` payload to `state::data()` and records the
type name in a `__data_type__` child.  When decoded, the codec reads
`state::data()` back into a `data_object` wrapper.  Local variables hold
`data_object_ptr` directly — no state tree round-trip is needed for
variable-to-variable operations.

#### 3.3 cvc::state Memory Management (LRU Eviction)

As state trees grow (especially with large programs, environments, and blob
data loaded by state_exec), memory consumption can become a problem.  A new
general-purpose **`state_memory_manager`** component adds memory-pressure
awareness to `cvc::state` itself — this is a core cvc::state feature, not
specific to state_exec.

##### Design

- Every `cvc::state` node has an estimated memory footprint (value string +
  `boost::any` data + overhead).  The memory manager tracks the total resident
  memory across all nodes in a tree.
- A configurable **memory budget** (e.g., 512 MB) defines the cap.  When
  resident memory exceeds the budget, the manager evicts node payloads
  according to a pluggable **eviction policy**.
- Eviction does **not** delete the node from the tree — the node remains
  visible in the child map with its name, metadata, and children intact.  Only
  the `_value` string and `_data` (`boost::any`) are cleared, and a
  **residency marker** records how to repopulate them.
- When a caller reads `value()` or `data()` on an evicted node, the memory
  manager intercepts and attempts **on-demand repopulation** from the backing
  store before returning.  If repopulation requires I/O (blob store, file,
  remote peer), the call blocks or returns a future, depending on the API
  used.  Repopulation may trigger further evictions of other nodes to stay
  within budget.

##### Eviction Policies

| Policy | Description |
|---|---|
| `lru` (default) | Least-recently-used: evicts the node whose value/data was accessed longest ago.  Every `value()` / `data()` read updates an access timestamp. |
| `lfu` | Least-frequently-used: evicts the node with the fewest accesses. |
| `size_weighted_lru` | LRU but weighted by payload size — prefer evicting large payloads first. |
| `ttl` | Time-to-live: evicts nodes whose payload has been resident longer than a configured duration. |
| `manual` | No automatic eviction; the caller explicitly calls `evict(path)`. |

##### Backing Stores for Evicted Payloads

The memory manager delegates storage of evicted payloads to a
**`state_eviction_store`** interface:

```cpp
namespace cvc {

class state_eviction_store {
public:
    virtual ~state_eviction_store() = default;

    // Store evicted payload. Returns a token for later retrieval.
    virtual std::string store(const std::string& path,
                              const std::string& value,
                              const boost::any& data) = 0;

    // Retrieve a previously evicted payload. Returns false if not found.
    virtual bool retrieve(const std::string& token,
                          std::string& value_out,
                          boost::any& data_out) = 0;

    // Discard a stored payload (node deleted from tree).
    virtual bool discard(const std::string& token) = 0;
};

}
```

Built-in implementations:

| Store | Description |
|---|---|
| `memory_eviction_store` | Moves payloads to a separate memory pool (useful for soft caps — data is still in RAM but outside the tracked budget). |
| `blob_eviction_store` | Writes payloads to a `state_blob_store` (in-memory or file-backed).  Leverages the existing `file_state_blob_store` for disk-backed eviction. |
| `null_eviction_store` | Discards evicted payloads entirely (data is lost; reads after eviction return empty).  Useful for caches. |

##### Integration with cvc::state

```cpp
namespace cvc {

class state_memory_manager {
public:
    enum class eviction_policy { lru, lfu, size_weighted_lru, ttl, manual };

    state_memory_manager(state& root,
                         std::size_t budget_bytes,
                         eviction_policy policy = eviction_policy::lru,
                         std::unique_ptr<state_eviction_store> store = nullptr);

    // Budget management
    void set_budget(std::size_t bytes);
    std::size_t budget() const;
    std::size_t resident_bytes() const;
    std::size_t evicted_count() const;

    // Manual eviction/repopulation
    bool evict(const std::string& path);
    bool repopulate(const std::string& path);
    bool is_evicted(const std::string& path) const;

    // Eviction callbacks
    using eviction_callback = std::function<void(const std::string& path)>;
    void on_eviction(eviction_callback cb);
    void on_repopulation(eviction_callback cb);

    // Policy tuning
    void set_ttl(std::chrono::seconds ttl);  // for ttl policy
    void set_high_watermark(double ratio);   // trigger eviction at ratio*budget
    void set_low_watermark(double ratio);    // stop eviction at ratio*budget
};

}
```

The `state` class gains a few additions:

- `std::size_t estimated_memory() const` — returns this node's payload size.
- An internal `_access_timestamp` updated on every `value()`/`data()` read.
- An internal `_eviction_token` (empty when resident, set when evicted).
- `value()` and `data()` check `_eviction_token` and call the memory
  manager's repopulation path before returning.
- `bool is_resident() const` — true if payload is in memory.

This integrates with the existing `state_data_hydrator` (which already handles
lazy blob hydration from remote peers) by serving as a local-first eviction
layer beneath it.

---

### 4. Component Breakdown

#### 4.1 Core Types (`cvc/state_exec/types.h`)

```cpp
namespace cvc::state_exec {

// The DSL value variant
using value_t = std::variant<
    std::monostate,          // nil
    bool,                    // t / nil
    int64_t,                 // integers
    double,                  // floats
    std::string,             // strings
    symbol,                  // symbol names
    list_ptr,                // shared_ptr<std::vector<value_t>>
    closure_ptr,             // shared_ptr<closure>
    dict_ptr,                // shared_ptr<ordered_map<std::string, value_t>>
    native_fn,               // std::function<value_t(std::span<value_t>)>
    data_object_ptr          // shared_ptr<data_object> — typed data from state::data()
>;

// Wraps boost::any for DSL-level access to state data objects.
// Programs can store data_object values in local variables, pass them
// to functions, and write them back via (state-data-set).
struct data_object {
    boost::any    payload;         // the actual data
    std::string   type_name;       // cvc::state::dataTypeName() at capture time
    bool is_type(const std::string& t) const { return type_name == t; }
};
using data_object_ptr = std::shared_ptr<data_object>;

struct symbol {
    std::string name;
    auto operator<=>(const symbol&) const = default;
};

struct closure {
    std::vector<symbol> params;
    bool variadic = false;   // &rest
    std::vector<value_t> body;
    environment_ptr env_snapshot;
};

// Environment chain (lexical scoping)
struct environment {
    std::unordered_map<std::string, value_t> bindings;
    std::shared_ptr<environment> outer;
};

}
```

#### 4.2 Parser (`cvc/state_exec/parser.h`)

Port the `ParserMixin.parse()` logic.  Instead of depending on Python
`sexpdata`, implement a self-contained S-expression parser in C++.  This is
straightforward — the grammar is:

```
expr   ::= atom | '(' expr* ')' | quote
atom   ::= number | string | symbol
quote  ::= '\'' expr
number ::= int_lit | float_lit
string ::= '"' ... '"'
```

The parser produces `value_t` trees (in-memory).  A separate
`store_to_state(state&, value_t)` function persists a parsed program into the
state tree.

#### 4.3 Built-in Functions (`cvc/state_exec/builtins.h`)

All 30+ built-in functions from `BUILT_IN_SOURCES` in `script.py`, directly
implemented as C++ functions (no sandboxed Python compilation needed):

| Category | Functions |
|---|---|
| Arithmetic | `+`, `-`, `*`, `/`, `%` |
| Comparison | `<`, `>`, `<=`, `>=`, `=`, `!=` |
| String | `str-concat`, `str` |
| List | `list`, `car`, `cdr`, `cons`, `nth`, `set-nth`, `length`, `append`, `slice`, `del-nth` |
| Dict | `dict`, `get-attr`, `set-attr`, `del-attr` |
| OOP | `send` |
| Higher-order | `apply` |

Additionally, new **state_exec-specific intrinsics** for state tree and runtime
manipulation (see §4.8).

#### 4.4 Evaluators

Four evaluator classes, organized into two tiers:

**Tier 1 — Scheduler / Distributed** (the only evaluator used by the
scheduler for cluster-managed processes):

| Class | Sync | Stackless | Notes |
|---|---|---|---|
| `async_stackless_evaluator` | async | yes | **Default for all scheduled processes.** Serializable + non-blocking.  Required for migration, pause/resume, distributed observation.  The only evaluator that produces first-class scheduler citizens. |

**Tier 2 — Local evaluation contexts** (developer convenience; *not*
schedulable, *not* migratable, *not* visible to cluster-wide `ps-all`):

| Class | Sync | Stackless | Notes |
|---|---|---|---|
| `evaluator` | sync | no (recursive) | Simplest.  Blocks the calling thread.  Good for one-shot expressions, REPL, testing. |
| `async_evaluator` | async (co_await) | no (recursive) | Non-blocking but not serializable — cannot migrate.  Good for local async scripting. |
| `stackless_evaluator` | sync | yes | Serializable but blocks — cannot yield to the async scheduler loop.  Useful for step-debugging or synchronous batch evaluation. |

All four share common logic via CRTP or a mixin base to avoid the code
duplication present in the Python version.

> **Why `async_stackless_evaluator` is mandatory for scheduling:**
> Sync evaluators block the scheduler's event loop, preventing other
> processes from stepping.  Non-stackless evaluators use the C++ call
> stack, which cannot be serialized — making migration, pause-and-resume
> across nodes, and state-tree persistence impossible.  Only
> `async_stackless_evaluator` satisfies both constraints.

##### 4.4.1 `evaluator` (sync recursive)

- `value_t evaluate(const value_t& expr, environment_ptr env)`
- Special forms: `if`, `begin`, `while`, `for`, `set`, `quote`, `lambda`,
  `return`, `let`, `super`, `defun`, `defclass`, `defmacro`, `eval`, `root`
- Interrupt/pause via `std::atomic<bool>` flags + `std::condition_variable`
- Thread-safe `evaluate()` with `std::mutex`
- Timeout via `std::jthread` + interrupt
- Statistics tracking (`evaluation_stats`)
- `on_complete` callback

##### 4.4.2 `async_evaluator` (async recursive)

- Uses C++20 coroutines (`co_await`)
- `task<value_t> evaluate(const value_t& expr, environment_ptr env)`
- Supports async callbacks for I/O operations
- Additional `await` special form
- `asyncio.sleep(0)` equivalent → `co_await std::suspend_always{}`

##### 4.4.3 `stackless_evaluator` (sync stackless) — **Tier 2: local contexts only**

- Explicit stack of `eval_frame` objects
- `evaluator_state` dataclass: stack, result, done, root_expr, global_env,
  user_macros, stats
- `create_state(script_or_expr, env) → evaluator_state`
- `step(evaluator_state&) → bool` — single step
- `run(evaluator_state&, max_steps, timeout, on_complete) → value_t`
- Full serialization to/from `cvc::state` tree nodes
- Phase-driven dispatch: `init`, `eval_args`, `apply`, `if_cond`,
  `if_branch`, `begin_next`, `while_cond`, `while_body`, `for_body`,
  `set_value`, `let_bindings`, `let_body`, `lambda_body`, `eval_inner`,
  `return_value`, `defun_body`, `defclass_methods`, `defmacro`,
  `super_args`

##### 4.4.4 `async_stackless_evaluator` (async stackless) — **Tier 1: default for all scheduled processes**

- Same as stackless but `step()` is `task<bool>` (coroutine)
- Supports async callbacks
- Additional `await_result` phase
- **The scheduler creates an `async_stackless_evaluator` for every process
  submitted via `execute()` or `fork()`.  There is no option to schedule a
  process with a different evaluator.**  Programs evaluated through any
  other evaluator class are local-only and not managed by the scheduler.

#### 4.5 Scheduler (`cvc/state_exec/scheduler.h`)

The scheduler exclusively uses **`async_stackless_evaluator`** for all managed
processes.  This is not configurable — every process submitted via `execute()`
or `fork()` is backed by an `async_stackless_evaluator` instance, ensuring
non-blocking stepping, state-tree serialization, and cross-node migration.
Programs that a developer runs through other evaluator classes (e.g., the sync
`evaluator` for a local REPL) are not scheduler-managed and do not appear in
`ps`, `ps-all`, or cluster-wide observation.

Two scheduler classes:

##### `scheduler` (sync)

Port of `CFOSScheduler`:

- Process abstraction (`process` struct) with PID, name, status, priority
  (nice -20..19), UID/GID (client identity / node_id), max_steps, timeout,
  max_memory, max_time, max_messages, signal handlers, create_time,
  exit_code/error, on_complete callback, per-process message inbox
- Process lifecycle: READY → RUNNING → TERMINATED/KILLED; PAUSED, WAITING
- `execute(script, name, priority, uid, gid, max_memory, max_time, max_messages, ...) → pid`
- `fork(pid) → child_pid` — clone a running process (§4.16)
- `step(concurrency, block) → int`
- Scheduling policies: `round_robin`, `priority`, `priority_rr`
- Signal handling: `send_signal(pid, signal)`, SIGKILL uncatchable
- Memory enforcement: check `current_bytes > max_memory` at step boundary
- Time enforcement: check `elapsed_time > max_time` at step boundary
- Message enforcement: check `message_count > max_messages` at step boundary
- Expired process sweep: call `sweepExpired()` on process subtrees each
  scheduler tick; expired process nodes are treated as killed
- Scheduler loop: `run(delay, max_steps, max_time, concurrency)`
- Process control: `pause(pid)`, `resume(pid)`, `kill(pid)`,
  `set_priority(pid, n)`, `set_max_steps(pid, n)`, `set_timeout(pid, t)`,
  `set_max_memory(pid, bytes)`, `set_max_time(pid, duration)`,
  `set_max_messages(pid, count)`
- Info: `list_processes()`, `get_process(pid)`, `get_process_info(pid)`,
  `get_result(pid)`, `get_results()`, `get_scheduler_stats()`
- Parallel stepping via `std::jthread` pool
- Full serialization to/from `cvc::state` tree + JSON

##### `async_scheduler` (async)

Port of `AsyncCFOSScheduler`:

- Same interface but async methods using C++20 coroutines
- Parallel stepping via `co_await` gather equivalent

#### 4.6 State Tree Integration

##### 4.6.1 `state_list` (`cvc/state_list.h`)

New addition to the `cvc::state` public API:

```cpp
namespace cvc {

class state_list {
public:
    explicit state_list(state& node);

    // List operations
    state& push_back();                  // appends new child, returns it
    state& at(std::size_t index);        // access by position
    const state& at(std::size_t index) const;
    void pop_back();
    void erase(std::size_t index);       // erase + reindex
    void insert(std::size_t index);      // insert + reindex
    std::size_t size() const;
    bool empty() const;
    void clear();

    // Iteration
    class iterator;
    iterator begin();
    iterator end();

private:
    state& _node;
    std::size_t _pad_width;
    static constexpr const char* len_key = "__len__";
    void set_size(std::size_t n);
    static std::string index_key(std::size_t i);  // "000000"
};

}
```

##### 4.6.2 `state_value_codec` (`cvc/state_exec/state_value_codec.h`)

Converts between in-memory `value_t` and state tree encoding:

```cpp
namespace cvc::state_exec {

// Write a value_t into a state node subtree
void encode_value(state& node, const value_t& val);

// Read a value_t from a state node subtree
value_t decode_value(const state& node);

// Write an evaluator_state into a state node subtree
void encode_evaluator_state(state& node, const evaluator_state& es);

// Read an evaluator_state from a state node subtree
evaluator_state decode_evaluator_state(const state& node);

}
```

##### 4.6.3 Scheduler ↔ State Tree

The scheduler persists its full state into the `cvc::state` tree under
`__state_exec__.__clusters__.<cluster_id>`.  This means:

- Process table changes propagate through `cvc::state` signals
  (`valueChanged`, `childChanged`)
- The `state_sync_adapter` + `state_cluster_shard` replicate scheduler state
  to peers
- Any peer can observe all processes via the state tree
- A peer can take over scheduling by updating `__scheduling_node__`

#### 4.7 Distributed Execution

##### 4.7.1 Scheduling Coordination via Distributed State Messaging

Cluster execution coordination is built on top of the existing
`state_message_bus` / `state_message` / `state_cluster_shard` infrastructure
rather than inventing a separate protocol.  All coordination messages are
`state_message` instances routed through the shard's message bus and transport.

A single node per cluster is the **scheduling node** (recorded in
`__state_exec__.__clusters__.<cluster_id>.__scheduling_node__`).  This node
runs the scheduler loop.

**Message types** (distinguished by `content_type`):

| content_type | Direction | Purpose |
|---|---|---|
| `application/x-state-exec-election` | broadcast | Election proposal: `{node_id, priority, timestamp}` |
| `application/x-state-exec-heartbeat` | scheduling→peers | Periodic liveness from scheduling node |
| `application/x-state-exec-handoff` | old→new | Scheduling handoff notification |
| `application/x-state-exec-submit` | client→scheduler | Submit a program for execution |
| `application/x-state-exec-control` | admin→scheduler | Pause/resume/kill/migrate commands |
| `application/x-state-exec-status` | scheduler→peers | Process status updates (batched) |
| `application/x-state-exec-signal` | any→scheduler | Signal delivery to a running process |
| `application/x-state-exec-migrate` | scheduler→target | Process migration payload |
| `application/x-state-exec-user` | program→program | User-defined inter-process messages |
| `application/x-state-exec-policy` | admin→scheduler | Resource policy update for the cluster |

The scheduling node subscribes to the `__state_exec__` path prefix on the
local `state_message_bus` and dispatches incoming messages.  Non-scheduling
nodes use `state::sendMessage()` (which routes through the shard and transport)
to reach the scheduling node regardless of which physical node they are on.

**Election protocol** (via messages, not ad-hoc):

1. When a cluster starts, or when heartbeats from the scheduling node stop
   arriving (TTL configurable, default 5 s), any node may broadcast an
   `election` message.
2. Nodes compare `(priority, node_id)` — lowest wins.  The winner sends a
   `handoff` message and writes its ID to `__scheduling_node__`.
3. The scheduling node emits `heartbeat` messages at a configurable interval
   (default 1 s) via the message bus.  Peers track these through
   `state_peer_registry::note_seen()`.
4. If a peer detects a stale heartbeat beyond the TTL, it initiates
   re-election by broadcasting an `election` message.

##### 4.7.2 Process Migration

A running process can be migrated to another node (or another cluster).
Migration transfers **all** execution responsibility to the target node,
including event handlers (signal handlers, message subscriptions, and
state-watch callbacks).

**Migration protocol:**

1. **Pause** — The scheduling node pauses the process.

2. **Serialize evaluator state** — The `evaluator_state` (stack frames,
   environments, result register) is written into the process's state tree
   node under `__evaluator__`.

3. **Serialize handler registrations** — All live event handler registrations
   are serialized into the process subtree:
   - **Signal handlers** are already in `__signals__.handlers` (signal_name
     → handler closure).  No extra work needed.
   - **Message subscriptions** (`msg-subscribe`) are written to
     `__subscriptions__.<sub_id>` with `path_prefix` and the serialized
     handler closure.  Each subscription the process registered via
     `(msg-subscribe)` gets an entry.
   - **State watches** (`state-watch`) are written to
     `__watches__.<watch_id>` with the watched `path` and the serialized
     handler closure.
   - **IPC inbox** — any undelivered messages in the process's message inbox
     queue are serialized into `__signals__.pending` (interleaved with
     pending signals, tagged by type).

4. **Tear down origin-side registrations** — The origin node:
   - Unsubscribes all `state_message_bus` subscriptions for this process
     (calls `unsubscribe()` for each `sub_id`).
   - Disconnects all `cvc::state` signal connections for `state-watch`
     callbacks.
   - Removes the process from the origin scheduler's local process table.
   - Does **not** delete the process state tree node (it replicates to the
     target via `state_cluster_shard`).

5. **Send migrate message** — The origin sends an
   `application/x-state-exec-migrate` message to the target node with the
   process PID and cluster ID.

6. **Target-side reconstruction** — The target node's scheduler:
   - Reads the replicated process subtree from the state tree.
   - Reconstructs the `evaluator_state` from `__evaluator__`.
   - **Re-registers message subscriptions**: walks `__subscriptions__`,
     deserializes each handler closure, and calls `subscribe()` on the
     target node's local `state_message_bus` with the same `path_prefix`.
     New local `sub_id`s are assigned; the `__subscriptions__` entries are
     updated to reflect the target-side IDs.
   - **Re-registers state watches**: walks `__watches__`, deserializes each
     handler closure, and connects to the target node's local `cvc::state`
     signals (`valueChanged` / `childChanged`) at the watched path.  New
     connection handles are stored internally.
   - **Restores IPC inbox**: deserializes pending messages from
     `__signals__.pending` back into the process's message inbox queue.
   - Adds the process to the target scheduler's local process table.
   - Updates `__meta__.owner_node` to the target's `node_id`.

7. **Resume** — The target scheduler resumes the process.  From the
   program's perspective, execution continues exactly where it left off.
   The next `(ipc-recv)`, message callback, or `state-watch` callback will
   fire on the target node.

**Handler semantics during migration:**

- Between steps 4 (origin teardown) and 6 (target reconstruction), there is
  a brief window where the process has no active subscriptions on any node.
  Messages and state changes arriving during this window are **not lost** —
  they are either queued by the message bus (messages) or will be observed
  on the next state tree read (state changes are persistent).  However,
  the handler closures will not fire for events that occurred during the
  gap.  This is a documented trade-off: migration is not instantaneous.

- If the process was watching a state path that only exists on the origin
  node (a non-replicated local path), the watch re-registration on the
  target will find no matching node and the watch becomes a no-op.  The
  process can detect this via `(state-exists)` and re-subscribe if needed.

- Message subscriptions are path-prefix-based and work identically on any
  node — the `state_message_bus` is per-node but messages route through
  the cluster transport, so the target node receives the same messages.

Since the evaluator state, handler closures, and subscription metadata are
all fully serializable *and* stored in the replicated state tree, migration
is a natural consequence of the architecture.

##### 4.7.3 Client Identity & Process Ownership

Each process records:
- `uid`: the `node_id` of the client that launched it
- `gid`: optional group identifier
- `owner_node`: the `node_id` of the node responsible for scheduling it

| `(state-data-get path)` | Read `state::data()` (`boost::any`) from a state node, returning a `data_object` value (nil if empty) |
| `(state-data-set path data-obj)` | Write a `data_object` value's payload to `state::data()` on a state node |
| `(state-data-type path)` | Return the type name string of the data object at a state node (nil if no data) |
| `(data-type obj)` | Return the type name of a `data_object` local variable (nil if not a data_object) |
| `(data? obj)` | Return `t` if `obj` is a `data_object`, `nil` otherwise |
| `(state-batch body...)` | Evaluate `body` forms while batching state changes; handler dispatch is deferred until all forms complete (wraps `state_change_batch_scope`) |
| `(state-lock path body...)` | Evaluate `body` forms while holding an exclusive lock on the state_object at `path` (wraps `state_lock_scope`); ensures atomic multi-field updates |
This enables:
- Access control: only the owner (or admin) can kill/pause a process
- Accounting: track resource usage per client
- Migration audit trail

#### 4.8 DSL Intrinsics for Runtime Interaction

New built-in functions available to state_exec programs (not present in
txwtf.cfos):

| Intrinsic | Description |
|---|---|
| `(state-get path)` | Read a value from the `cvc::state` tree |
| `(state-set path value)` | Write a value to the `cvc::state` tree |
| `(state-children path)` | List child names at a state path |
| `(state-exists path)` | Check if a state path exists |
| `(state-delete path)` | Delete a state subtree |
| `(state-watch path callback)` | Register a callback on state changes |
| `(ps)` | List all processes in the local cluster |
| `(ps-all)` | List all processes across all accessible clusters |
| `(kill pid)` | Kill a process by PID |
| `(pause pid)` | Pause process |
| `(resume pid)` | Resume process |
| `(spawn script [name] [priority])` | Launch a new process, returns PID |
| `(fork)` | Fork the current process; returns child PID to parent, 0 to child (§4.16) |
| `(self-pid)` | Current process PID |
| `(self-uid)` | Current process UID (client identity) |
| `(cluster-id)` | Current cluster ID |
| `(node-id)` | Current node ID |
| `(peers)` | List connected peers |
| `(migrate pid target-node)` | Migrate a process to a target node |
| `(scheduler-stats)` | Get scheduler statistics |
| `(sleep seconds)` | Cooperative sleep (yields to scheduler) |
| `(signal pid signal-name)` | Send a signal to a process |
| `(msg-send path payload [content-type])` | Send a `state_message` to a state tree path |
| `(msg-subscribe path-prefix callback)` | Subscribe to messages on a path prefix; callback receives message |
| `(msg-unsubscribe id)` | Unsubscribe from messages (id returned by `msg-subscribe`) |
| `(msg-broadcast payload [content-type])` | Broadcast a message to all peers in the cluster |
| `(ipc-send pid payload)` | Send a user message to a specific process by PID |
| `(ipc-recv [timeout])` | Receive the next pending user message (blocks or returns nil on timeout) |
| `(memory-usage)` | Current memory usage of this process (bytes) |
| `(memory-limit)` | Max memory allowed for this process (0 = unlimited) |
| `(time-elapsed)` | Wall-clock seconds since this process started |
| `(time-limit)` | Max wall-clock time allowed for this process (0 = unlimited) |
| `(message-count)` | Number of outbound messages this process has sent |
| `(message-limit)` | Max outbound messages allowed for this process (0 = unlimited) |
| `(state-expire path duration)` | Set a `cvc::state` node to expire after `duration` seconds (`expireAfter`) |
| `(state-expire-at path timestamp)` | Set a `cvc::state` node to expire at an absolute time (`expireAt`) |
| `(state-has-expiry path)` | Check if a state node has an expiry set |
| `(state-expiry-time path)` | Return the absolute expiry time of a state node (nil if none) |
| `(state-clear-expiry path)` | Remove expiry from a state node |
| `(state-is-expired path)` | Check if a state node is past its expiry time |
| `(state-sweep-expired path)` | Sweep all expired descendants of a state node, removing them |

#### 4.9 Process Memory Limits

Each state_exec process can have a **maximum memory budget** (`max_memory`
field in process metadata).  Memory is defined as the **total estimated size
of state tree objects the process has written**, tracked with per-node
ownership.

##### Ownership Model

Every `cvc::state` node written by a state_exec process records the PID of
the writer (stored in a lightweight side-table in the `memory_tracker`, not
in the state node itself — the ownership map is `std::unordered_map<state*,
pid_owner_entry>` where `pid_owner_entry = { int pid; std::size_t bytes; }`).

Rules:

1. **Write** (`state-set`, `push_back`, `set-attr`, closure capture, env
   frame, etc.): The estimated byte cost of the written object is added to
   the process's `current_bytes`.  The node is tagged with the writing
   process's PID in the ownership map.

2. **Overwrite by same process**: If the process overwrites a node it already
   owns, the *old* object's bytes are subtracted and the *new* object's bytes
   are added.  Net effect is the delta.

3. **Overwrite by different process**: The *new* writer's PID replaces the
   old owner in the ownership map.  The *new* object's bytes are charged to
   the new writer.  The *old* object's bytes are credited back to the
   original owner (their `current_bytes` decreases).  This prevents the
   original owner from being permanently penalized for data that no longer
   exists.

4. **Delete by owner** (`state-delete`, `pop_back`, environment frame pop,
   etc.): The deleted object's bytes are subtracted from the owning process's
   `current_bytes`.  The ownership entry is removed.

5. **Delete by non-owner**: If process A deletes a node owned by process B,
   process A gets **no credit** — its `current_bytes` does not change.
   Process B's `current_bytes` is decremented (B's object is gone).  This
   prevents a malicious process from artificially shrinking its own budget by
   deleting other processes' data.

6. **Subtree operations**: `state-delete` on a subtree walks all descendant
   nodes; each node's bytes are credited back to whichever PID owns it.

7. **Process termination cleanup**: When a process terminates, all its owned
   nodes in the ownership map remain (the data persists in the tree).  The
   ownership entries are reassigned to PID 0 (unowned / system).  Future
   deletes of those nodes credit PID 0 (no-op).

##### What Counts as Size

| Operation | Cost charged to writer |
|---|---|
| Integer/float/bool/nil via `state-set` | `sizeof(value_t)` (~64 bytes, fixed) |
| String via `state-set` | `sizeof(std::string) + str.capacity()` |
| List element (`push_back` / `cons` to state list) | `sizeof(value_t)` per element + state node overhead |
| Dict entry (`set-attr`) | `sizeof(std::string) + sizeof(value_t)` per entry |
| Closure written to state | params + body deep size + env snapshot size |
| State node creation (child added) | `sizeof(state)` node overhead (~256 bytes) + value payload |
| Environment frame (internal, not tree) | Not tracked here — see below |

**In-evaluator temporaries** (local variables in an environment frame,
intermediate computation results) are **not** counted toward the process
memory limit.  Only objects that land in the `cvc::state` tree are tracked.
This is deliberate: the memory limit governs the process's *footprint in the
shared state tree*, not its transient working set.  Transient evaluator
memory is bounded by `max_steps` (a terminating program's stack is finite)
and by the overall `state_memory_manager` budget (§3.3).

The scheduler checks `current_bytes` against `max_memory` at each step
boundary (same checkpoint as max_steps/max_time/max_messages).  If exceeded,
the process is **terminated** with status `killed` and `exit_error` set to
`memory_limit_exceeded`.

##### C++ API

```cpp
// On process submission
int pid = scheduler.execute(script, {
    .name = "my-program",
    .max_memory = 64 * 1024 * 1024,  // 64 MB
});

// Adjust at runtime
scheduler.set_max_memory(pid, 128 * 1024 * 1024);

// Query
auto info = scheduler.get_process_info(pid);
info.current_memory;  // live sum of owned objects in tree
info.peak_memory;     // high-water mark
info.max_memory;      // configured limit
```

The DSL intrinsic `(memory-usage)` returns `current_bytes` so programs can
self-monitor and take action (e.g., delete cached state nodes, reduce working
set) before hitting the hard limit.

##### `memory_tracker` Implementation

```cpp
namespace cvc::state_exec {

class memory_tracker {
public:
    // Record a write: process `pid` wrote `bytes` to `node`.
    // If node was already owned, adjusts old owner's counter.
    void record_write(int pid, cvc::state* node, std::size_t bytes);

    // Record a delete: credits bytes back to the owning process.
    // If `deleter_pid != owner_pid`, deleter gets no credit.
    void record_delete(int deleter_pid, cvc::state* node);

    // Record a subtree delete: walks ownership map for all descendants.
    void record_subtree_delete(int deleter_pid, cvc::state* node);

    // Query
    std::size_t current_bytes(int pid) const;
    std::size_t peak_bytes(int pid) const;

    // Process terminated: reassign all owned nodes to PID 0.
    void release_ownership(int pid);

    // Fork: copy all ownership entries from parent to child for
    // nodes under the child's new process subtree.  The child starts
    // with current_bytes equal to the size of its cloned subtree.
    void fork_ownership(int parent_pid, int child_pid,
                        cvc::state* child_root);

private:
    struct pid_owner_entry {
        int pid;
        std::size_t bytes;
    };
    std::unordered_map<cvc::state*, pid_owner_entry> _ownership;
    std::unordered_map<int, std::size_t> _current;  // pid → total bytes
    std::unordered_map<int, std::size_t> _peak;     // pid → high-water
};

}
```

#### 4.10 Messaging Integration for Programs

state_exec programs interact with the `cvc::state_message_bus` through the
messaging intrinsics listed in §4.8.  This enables:

- **Inter-process communication**: `(ipc-send pid payload)` sends a
  `state_message` with `content_type = "application/x-state-exec-user"` and
  the target PID encoded in the path.  The scheduler delivers it to the
  target process's message inbox (a bounded queue in the process struct).  The
  target reads it with `(ipc-recv)`.

- **State tree event handling**: `(msg-subscribe "sensors.temperature" handler)`
  registers a `state_message_bus` subscription.  When a message arrives on
  that path prefix, the handler (a closure) is invoked on the process's next
  step — messages are **not** delivered mid-evaluation but queued and
  dispatched at step boundaries, similar to signal handling.

- **Broadcast**: `(msg-broadcast payload)` sends a message to all peers via
  `state::sendMessage()`, which routes through the shard's transport.  Remote
  peers' message buses deliver it to any subscribed processes.

- **Cross-cluster messaging**: Messages are scoped by `cluster_id` in
  `state_message`.  A process can subscribe to messages from its own cluster;
  cross-cluster messaging requires explicit routing through a gateway node
  that participates in multiple clusters.

The scheduler's own coordination messages (§4.7.1) use the same bus.  User
messages are distinguished by `content_type` prefix
(`application/x-state-exec-user`).

#### 4.11 Process Observation ("ps" / "top")

Two levels of visibility:

1. **Cluster-local**: `list_processes()` on the scheduler, or `(ps)` from a
   DSL program, returns info for all processes in the current cluster.
   Equivalent to `ps aux`.

2. **Cross-cluster**: `(ps-all)` queries the state trees of all clusters the
   current node has access to (via `state_cluster_shard` peers).  Returns
   aggregated process lists from every reachable cluster.

The C++ API also provides:

```cpp
struct process_info {
    int pid;
    std::string name;
    process_status status;
    int priority;
    std::string uid;       // client identity / node_id
    std::string gid;
    std::string cluster_id;
    std::string owner_node;
    uint64_t step_count;
    double elapsed_time;
    std::string current_phase;
    int stack_depth;
    uint64_t current_memory;  // sum of owned state tree objects (bytes)
    uint64_t peak_memory;     // high-water mark (bytes)
    uint64_t max_memory;      // configured limit (0 = unlimited)
    double max_time;           // wall-clock time limit in seconds (0 = unlimited)
    uint64_t message_count;   // outbound messages sent so far
    uint64_t max_messages;    // outbound message limit (0 = unlimited)
    int parent_pid;            // PID of parent if forked, -1 if root
    bool has_expiry;           // whether the process node has a cvc::state expiry set
    std::optional<boost::posix_time::ptime> expiry_time; // absolute expiry, if set
};

// Cluster-local
std::vector<process_info> list_processes(const std::string& cluster_id);

// Cross-cluster
std::vector<process_info> list_all_processes();

// Summary stats (like "top")
struct cluster_exec_stats {
    std::string cluster_id;
    int total_processes;
    int running;
    int paused;
    int terminated;
    uint64_t total_steps;
    std::string scheduling_node;
};
std::vector<cluster_exec_stats> cluster_stats();
```

---

#### 4.12 State Node Expiry Integration

`cvc::state` already provides a full expiry API (`expireAt`, `expireAfter`,
`clearExpiry`, `hasExpiry`, `expiryTime`, `isExpired`, `sweepExpired`, and
the `expiring` signal).  state_exec integrates with this at three levels:

##### 4.12.1 Process Node Expiry

A process's root state node (`__state_exec__.__clusters__.<cid>.__processes__.<pid>`)
can itself have a `cvc::state` expiry set.  This is the mechanism for
**auto-cleanup of terminated processes**: after a process terminates, the
scheduler calls `expireAfter(process_node, retention_duration)` so the node
is automatically swept after a configurable retention period (default: 5 min).

When a process node expires:
1. The scheduler's `expiring` signal handler fires *before* removal.
2. The handler archives the process result/exit code if configured (e.g., to
   blob store via `state_data_hydrator`).
3. The node and its entire subtree (program, evaluator state, stats, memory,
   signals) are removed by `sweepExpired()`.

A running process can also have an expiry set externally (e.g., by cluster
policy — see §4.15).  If the scheduler's sweep encounters a running process
whose node has expired, it kills the process with `exit_error =
"node_expired"` before the node is removed.

##### 4.12.2 Program-Accessible Expiry

DSL programs can set, query, and clear expiry on arbitrary state nodes via
the intrinsics in §4.8:

```lisp
;; Expire a cache entry after 60 seconds
(state-set "sensors.cache.temperature" (read-sensor))
(state-expire "sensors.cache.temperature" 60)

;; Check before reading
(if (state-is-expired "sensors.cache.temperature")
    (begin
        (state-sweep-expired "sensors.cache")
        (refresh-cache))
    (state-get "sensors.cache.temperature"))

;; Set absolute deadline
(state-expire-at "jobs.batch-42" "2025-06-15T00:00:00Z")
```

Access control: programs can only set expiry on state nodes they own (under
their process subtree) or on general-purpose paths if the process's UID
matches the node's owner.  Processes cannot set expiry on other processes'
nodes or on scheduler infrastructure nodes.

##### 4.12.3 Scheduler Sweep Integration

The scheduler calls `sweepExpired()` on the cluster root node once per
scheduler tick (same loop as step execution).  This is lightweight — expiry
is checked via `isExpired()` on each node's cached `_expiry_time` field,
which is an O(n) walk over children with early exit on non-expiring subtrees.

For nodes that are about to expire, the `expiring` signal fires before
removal.  The scheduler connects to this signal on process nodes to perform
cleanup (archive results, update counters, notify waiting processes).

##### 4.12.4 Expiry and Distributed Replication

When a node's expiry is set, the expiry metadata (`expireAt`/`expireAfter`
parameters) is part of the serialized state and replicates through
`state_sync_adapter` + `state_cluster_shard`.  This means:

- An expiry set on one node is enforced on all replicas.
- The scheduling node is responsible for sweeping; peer replicas do *not*
  independently sweep (to avoid race conditions with stale clocks).
- If the scheduling node changes (failover), the new scheduling node picks
  up sweep responsibility immediately since expiry times are absolute
  (`ptime`).

#### 4.13 Process Time Limits

Each state_exec process can have a **maximum wall-clock time** (`max_time`
field in process metadata, in seconds).  A value of `0` means unlimited
(the default).

The scheduler checks `elapsed_time > max_time` at each step boundary
(same checkpoint as max_steps, max_memory, and interrupt/pause).  If
exceeded, the process is **terminated** with status `killed` and `exit_error`
set to `time_limit_exceeded`.

```cpp
// On process submission
int pid = scheduler.execute(script, {
    .name = "batch-job",
    .max_time = 300.0,   // 5 minutes wall-clock
});

// Adjust at runtime
scheduler.set_max_time(pid, 600.0);  // extend to 10 minutes

// Query
auto info = scheduler.get_process_info(pid);
info.elapsed_time;  // seconds since start
info.max_time;      // configured limit (0 = unlimited)
```

The DSL intrinsics `(time-elapsed)` and `(time-limit)` let programs
self-monitor.  A program can request a time extension by calling
`(state-set)` on its own `__meta__.max_time` node — but this is subject
to cluster resource policy validation (§4.15) and will be rejected if it
exceeds the cluster's maximum allowed time.

Time accounting uses `boost::posix_time::microsec_clock::universal_time()`
and compares against `start_time` stored in the process's `__stats__` node.
Time spent while the process is paused does **not** count — `elapsed_time`
only accumulates while `status == running`.

#### 4.14 Process Message Limits

Each state_exec process can have a **maximum outbound message count**
(`max_messages` field in process metadata).  A value of `0` means unlimited
(the default).

Every outbound message operation — `(msg-send)`, `(msg-broadcast)`,
`(ipc-send)`, and any internal message the process triggers — increments a
per-process `message_count` counter in `__stats__`.  The scheduler checks
`message_count > max_messages` at each step boundary.  If exceeded, the
process is **terminated** with status `killed` and `exit_error` set to
`message_limit_exceeded`.

```cpp
// On process submission
int pid = scheduler.execute(script, {
    .name = "chatty-service",
    .max_messages = 10000,  // limit outbound messages
});

// Adjust at runtime
scheduler.set_max_messages(pid, 50000);

// Query
auto info = scheduler.get_process_info(pid);
info.message_count;   // messages sent so far
info.max_messages;    // configured limit (0 = unlimited)
```

This limit prevents:
- Malicious programs from flooding the message bus and overwhelming peers
- Runaway processes generating infinite broadcast storms
- Resource exhaustion on the network/transport layer

The DSL intrinsics `(message-count)` and `(message-limit)` let programs
monitor their own budget.  The count is checked *after* the message is sent
(fire-then-check at next step boundary), not before, so the last message
always goes through — this matches the behavior of max_steps and max_memory.

#### 4.15 Cluster Resource Policies

Each cluster can define a **resource policy** that constrains the limits
programs may request.  This prevents individual programs from requesting
unlimited resources on a shared cluster.

The policy is stored in the state tree at:

```
__state_exec__.__clusters__.<cluster_id>.__policy__
    ├── max_time_min          (minimum max_time any process must have, 0 = no floor)
    ├── max_time_max          (maximum max_time any process may have, 0 = no ceiling)
    ├── max_time_default      (default max_time for new processes, 0 = unlimited)
    ├── max_memory_min
    ├── max_memory_max
    ├── max_memory_default
    ├── max_messages_min
    ├── max_messages_max
    ├── max_messages_default
    ├── max_steps_min
    ├── max_steps_max
    ├── max_steps_default
    ├── max_processes          (max simultaneous processes in this cluster, 0 = no limit)
    ├── max_total_memory       (aggregate memory budget across all processes, 0 = no limit)
    └── enforce               (strict | clamp | warn)
```

##### Enforcement Modes

| Mode | Behavior |
|---|---|
| `strict` | Reject `execute()` if any requested limit falls outside `[min, max]` range. Return error. |
| `clamp` | Silently clamp requested limits to the `[min, max]` range. Log the adjustment. |
| `warn` | Accept any limits but log a warning if they fall outside the recommended range. |

##### Policy Application

1. **On process submission** (`execute()`):
   - If the caller does not specify a limit (value = 0), apply the
     `*_default` from the policy.
   - If the caller specifies a limit, validate it against `*_min`/`*_max`.
   - Check `max_processes` — reject if the cluster already has that many
     non-terminated processes.
   - Check `max_total_memory` — reject if the sum of all processes'
     `max_memory` (including the new one) would exceed the aggregate budget.

2. **On runtime adjustment** (`set_max_time()`, `set_max_memory()`, etc.):
   - The new value is validated against the policy in the same way.
   - In `strict` mode, the call fails with an error.
   - In `clamp` mode, the value is silently adjusted.

3. **On policy change**: If the cluster admin tightens a policy, already-
   running processes are **not** retroactively killed (they were valid when
   submitted).  New submissions are subject to the updated policy.  An admin
   can choose to manually kill processes that no longer comply.

##### C++ API

```cpp
namespace cvc::state_exec {

struct resource_policy {
    double max_time_min = 0;
    double max_time_max = 0;
    double max_time_default = 0;

    uint64_t max_memory_min = 0;
    uint64_t max_memory_max = 0;
    uint64_t max_memory_default = 0;

    uint64_t max_messages_min = 0;
    uint64_t max_messages_max = 0;
    uint64_t max_messages_default = 0;

    uint64_t max_steps_min = 0;
    uint64_t max_steps_max = 0;
    uint64_t max_steps_default = 0;

    int max_processes = 0;             // 0 = unlimited
    uint64_t max_total_memory = 0;     // 0 = unlimited

    enum class mode { strict, clamp, warn };
    mode enforce = mode::clamp;
};

// Set the cluster's resource policy
void set_resource_policy(const std::string& cluster_id,
                         const resource_policy& policy);

// Get the current resource policy
resource_policy get_resource_policy(const std::string& cluster_id);

// Validate a set of process limits against the policy
// Returns adjusted limits (in clamp mode) or throws (in strict mode)
struct process_limits {
    double max_time = 0;
    uint64_t max_memory = 0;
    uint64_t max_messages = 0;
    uint64_t max_steps = 0;
};
process_limits validate_limits(const std::string& cluster_id,
                               const process_limits& requested);

}
```

##### Policy and the DSL
#### 4.16 Process Forking

A running state_exec process can **fork** itself, producing a child process
that is a complete clone of the parent at the current point of execution.
This follows Unix `fork()` semantics adapted to the state tree model.

##### Semantics

1. **`(fork)`** pauses the calling process for the remainder of the current
   step.
2. The scheduler deep-copies the parent's entire process subtree
   (`__program__`, `__evaluator__`, `__stats__`, `__memory__`, `__signals__`)
   into a new process node with a fresh PID.
3. The child's `__meta__` inherits the parent's `uid`, `gid`, `owner_node`,
   and resource limits (`max_steps`, `max_time`, `max_memory`,
   `max_messages`), subject to cluster resource policy validation (§4.15).
   `parent_pid` is set to the parent's PID.
4. The child's `__stats__` are **reset**: `step_count = 0`,
   `start_time = now`, `message_count = 0`.  The child gets a full budget
   of its own — it does not inherit the parent's consumed steps/time/messages.
5. Both processes resume on the next scheduler step.  The parent's evaluator
   result register is set to the child's PID (integer).  The child's result
   register is set to `0`.  This is how the program distinguishes parent
   from child:

```lisp
(let ((pid (fork)))
  (if (= pid 0)
      (begin
        ;; child: do work
        (state-set "results.child" (compute)))
      (begin
        ;; parent: pid is the child's PID
        (state-set "results.parent" (str "spawned child " pid)))))
```

##### Memory Ownership After Fork

The deep-copy creates **new state nodes** under the child's process subtree.
The `memory_tracker`'s `fork_ownership()` method walks the child's cloned
subtree and creates ownership entries for the child PID, sized to match
each node's payload.  The child starts with `current_bytes` equal to the
total size of its cloned subtree.

The parent's `current_bytes` is **unchanged** — the parent still owns its
original nodes.  The child owns its copies.  After the fork, parent and
child are fully independent: writes by one do not affect the other's memory
accounting.

If the fork would cause `max_total_memory` (cluster aggregate) or
`max_processes` (cluster cap) to be exceeded, the fork fails: the parent
receives `-1` and no child is created.  The parent can check for this:

```lisp
(let ((pid (fork)))
  (if (= pid -1)
      (print "fork failed: resource limit")
      ...))
```

##### State Outside the Process Subtree

The fork only clones the process subtree under `__processes__.<pid>`.  State
nodes that the parent has written elsewhere in the tree (via `(state-set)`)
are **not** cloned — they remain owned by the parent.  The child can read
them, but writing to the same paths creates new ownership entries for the
child (per the normal ownership rules in §4.9).

Signal handlers and message subscriptions are **cloned** into the child's
process-local state.  However, `state_message_bus` subscriptions registered
via `(msg-subscribe)` are duplicated for the child — the child gets its own
subscription IDs.

##### Scheduler API

```cpp
// Fork a running process. Returns child PID, or -1 on failure.
int fork(int parent_pid);

// Query lineage
std::optional<int> get_parent_pid(int pid) const;
std::vector<int> get_children(int pid) const;
```

##### Fork and Distributed Execution

Forking creates the child on the **same node** as the parent.  To fork onto
a remote node, the program should `(fork)` locally and then
`(migrate child_pid target_node)`.  Remote forking is not atomic — it is
two distinct operations.

#### 4.17 Standard Library

state_exec programs have access to a **standard library** of reusable
routines stored in the global state tree at
`__state_exec__.__global__.__stdlib__`.  The stdlib provides higher-level
operations (string manipulation, math, I/O, data structures) that are too
complex or verbose to implement as single built-in C++ functions, and serves
as the primary extensibility mechanism for downstream C++ libraries.

##### 4.17.1 Location and Access Control

The stdlib lives at a fixed path in the state tree:

```
__state_exec__.__global__.__stdlib__
    ├── __acl__
    │   ├── read       "all"        (every process can read)
    │   └── write      "admin"      (only cluster admins can modify)
    ├── string
    ├── math
    ├── io
    ├── collections
    └── <downstream_module>
```

**Access rules:**
 (local)**: A process can write to any path in its local state tree
  — including `__stdlib__` — without local blocking.  The evaluator and
  intrinsics layer do **not** enforce ACL checks on `state-set`.
- **Write (cluster consensus)**: When a local write to a protected subtree
  (e.g., `__stdlib__`) replicates to the cluster, the cluster's consensus
  layer checks the `__acl__` node of the target subtree.  If the writing
  process's UID is not in the admin list (or a delegate for that subtree),
  the cluster **rejects** the write.  An `acl_denied` error is propagated
  back to the originating process via the replication error channel.
- **Error handling**: The process receives the `acl_denied` error
  asynchronously (after replication attempt).  It may handle it, log it, or
  ignore it entirely — the local tree retains whatever the process wrote.
  The cluster's authoritative copy does **not** include the rejected write.
- **Replication**: The `__global__.__stdlib__` subtree replicates across all
  nodes via `state_cluster_shard`, so every node has a local read copy.
  Admin-authored writes propagate through normal state replication; non-admin
  writes are rejected during replication and do not propagate.

##### 4.17.2 DSL Interface

Programs access stdlib functions via a `(require)` intrinsic or direct
qualified calls:

```lisp
;; Import a module into the local environment
(require "string")
(split "hello world" " ")           ;; → ("hello" "world")

;; Or call with fully qualified path
(stdlib.string.split "hello world" " ")

;; Import specific symbols
(require "math" '(sqrt abs floor ceil))
(sqrt 144)                           ;; → 12

;; List available modules
(stdlib-modules)                     ;; → ("string" "math" "io" "collections")

;; List functions in a module
(stdlib-list "string")               ;; → ("split" "join" "replace" ...)
```

New intrinsics:

| Intrinsic | Description |
|---|---|
| `(require module [symbols])` | Import a stdlib module (or specific symbols) into the calling environment |
| `(stdlib-modules)` | List all available stdlib module names |
| `(stdlib-list module)` | List all function/object names in a stdlib module |
| `(stdlib.module.func args...)` | Direct qualified call to a stdlib function |

##### 4.17.3 Built-in Standard Library Modules

The following modules ship with state_exec:

**`string`** — String manipulation

| Function | Signature | Description |
|---|---|---|
| `split` | `(split s delim)` | Split string by delimiter, returns list |
| `join` | `(join lst delim)` | Join list elements with delimiter |
| `replace` | `(replace s old new)` | Replace all occurrences of `old` with `new` |
| `trim` | `(trim s)` | Strip leading/trailing whitespace |
| `upper` | `(upper s)` | Convert to uppercase |
| `lower` | `(lower s)` | Convert to lowercase |
| `starts-with` | `(starts-with s prefix)` | Test if string starts with prefix |
| `ends-with` | `(ends-with s suffix)` | Test if string ends with suffix |
| `contains` | `(contains s substr)` | Test if string contains substring |
| `substring` | `(substring s start [end])` | Extract substring by index |
| `char-at` | `(char-at s index)` | Character at position (as string) |
| `format` | `(format template args...)` | `printf`-style string formatting |
| `regex-match` | `(regex-match pattern s)` | Regex match, returns list of groups or nil |
| `regex-replace` | `(regex-replace pattern s replacement)` | Regex-based replacement |

**`math`** — Mathematical functions

| Function | Signature | Description |
|---|---|---|
| `sqrt` | `(sqrt x)` | Square root |
| `abs` | `(abs x)` | Absolute value |
| `floor` | `(floor x)` | Floor to integer |
| `ceil` | `(ceil x)` | Ceiling to integer |
| `round` | `(round x [places])` | Round to N decimal places |
| `pow` | `(pow base exp)` | Exponentiation |
| `log` | `(log x [base])` | Logarithm (default natural) |
| `sin` / `cos` / `tan` | `(sin x)` | Trigonometric functions (radians) |
| `min` / `max` | `(min a b ...)` | Minimum / maximum of arguments |
| `clamp` | `(clamp x lo hi)` | Clamp value to range |
| `pi` / `e` | constants | Mathematical constants (accessed as values) |
| `random` | `(random [lo] [hi])` | Random number (int or float depending on args) |

**`io`** — Input/output and network

| Function | Signature | Description |
|---|---|---|
| `url-fetch` | `(url-fetch url [opts])` | HTTP GET, returns response body as string. `opts` is a dict with optional `:headers`, `:timeout`, `:method`. |
| `url-post` | `(url-post url body [opts])` | HTTP POST with body string |
| `json-parse` | `(json-parse s)` | Parse JSON string into state_exec value (dict/list/string/number) |
| `json-encode` | `(json-encode val)` | Encode a value as JSON string |
| `base64-encode` | `(base64-encode s)` | Base64 encode |
| `base64-decode` | `(base64-decode s)` | Base64 decode |
| `sha256` | `(sha256 s)` | SHA-256 hash, returns hex string |
| `timestamp` | `(timestamp)` | Current UTC timestamp as ISO 8601 string |
| `timestamp-unix` | `(timestamp-unix)` | Current UTC timestamp as Unix epoch seconds |

**`collections`** — Data structure utilities

| Function | Signature | Description |
|---|---|---|
| `map` | `(map func lst)` | Apply function to each element, return new list |
| `filter` | `(filter pred lst)` | Return elements where predicate is true |
| `reduce` | `(reduce func init lst)` | Left fold |
| `zip` | `(zip lst1 lst2)` | Pair elements from two lists |
| `flatten` | `(flatten lst)` | Flatten nested lists one level |
| `sort` | `(sort lst [cmp])` | Sort list (optional comparator) |
| `reverse` | `(reverse lst)` | Reverse a list |
| `range` | `(range start end [step])` | Generate list of numbers |
| `assoc` | `(assoc alist key)` | Association list lookup |
| `unique` | `(unique lst)` | Remove duplicates (preserving order) |
| `group-by` | `(group-by func lst)` | Group elements by function result |
| `dict-keys` | `(dict-keys d)` | List of keys from a dict |
| `dict-values` | `(dict-values d)` | List of values from a dict |
| `dict-merge` | `(dict-merge d1 d2)` | Merge two dicts (d2 wins on conflict) |

##### 4.17.4 Implementation: DSL vs. Native

Stdlib functions can be implemented in two ways:

1. **DSL definitions**: Written as `(defun ...)` expressions stored in the
   state tree.  These are evaluated in the calling process's context (like
   a library `require`).  Advantages: portable, inspectable, modifiable by
   admins at runtime.

2. **Native (C++) bridges**: A C++ function registered as a built-in that
   is exposed through the stdlib namespace.  The function runs as native
   code but appears to the DSL program as a normal callable.  Used for
   operations that need C++ performance or access to system resources
   (e.g., `url-fetch` wraps libcurl/Boost.Beast, `sha256` wraps OpenSSL,
   regex operations wrap `std::regex`).

The stdlib module metadata records which implementation each function uses:

```
__stdlib__.<module>.<func>
    ├── __type__     "dsl" | "native"
    ├── __doc__      (optional docstring)
    ├── __params__   (parameter names as ordered list)
    └── __body__     (if type=dsl: the defun body as stored S-expression)
                     (if type=native: the registered C++ function key)
```

##### 4.17.5 C++ Registration API

Downstream C++ libraries register stdlib modules and functions at runtime.
This is the primary extensibility mechanism — a library like libcvc itself,
or any application built on libcvc, can add domain-specific operations that
state_exec programs can call.

```cpp
namespace cvc::state_exec {

class stdlib_registry {
public:
    // Get the singleton registry (one per state tree root)
    static stdlib_registry& instance(cvc::state& root);

    // Register an entire module of DSL-defined functions.
    // `dsl_source` is a string of (defun ...) forms.
    void register_module_dsl(const std::string& module_name,
                             const std::string& dsl_source);

    // Register a single native (C++) function in a module.
    // `func` takes (span<const value_t> args, environment& env) → value_t.
    using native_fn = std::function<value_t(std::span<const value_t>,
                                            environment&)>;
    void register_native(const std::string& module_name,
                         const std::string& func_name,
                         native_fn func,
                         std::vector<std::string> param_names = {},
                         std::string docstring = "");

    // Register a batch of native functions for a module.
    struct native_entry {
        std::string name;
        native_fn func;
        std::vector<std::string> params;
        std::string doc;
    };
    void register_module_native(const std::string& module_name,
                                std::vector<native_entry> entries);

    // Remove a module (admin only).
    void unregister_module(const std::string& module_name);

    // Query
    std::vector<std::string> list_modules() const;
    std::vector<std::string> list_functions(const std::string& module) const;
    bool has_module(const std::string& module) const;
    bool has_function(const std::string& module,
                      const std::string& func) const;

    // Populate __stdlib__ subtree in the state tree from all
    // registered modules.  Called once at startup and after
    // register_module_* calls.
    void materialize(cvc::state& stdlib_root);

private:
    std::unordered_map<std::string,
                       std::vector<native_entry>> _native_modules;
    std::unordered_map<std::string, std::string> _dsl_modules;
};

}
```

**Usage example — downstream library registering domain functions:**

```cpp
// In libcvc's initialization code (or any downstream app):
auto& reg = cvc::state_exec::stdlib_registry::instance(root_state);

// Register a "geometry" module with native C++ functions
reg.register_module_native("geometry", {
    {
        .name = "distance",
        .func = [](std::span<const value_t> args, environment&) -> value_t {
            auto x1 = std::get<double>(args[0]);
            auto y1 = std::get<double>(args[1]);
            auto x2 = std::get<double>(args[2]);
            auto y2 = std::get<double>(args[3]);
            return std::sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
        },
        .params = {"x1", "y1", "x2", "y2"},
        .doc = "Euclidean distance between two 2D points"
    },
    {
        .name = "area-circle",
        .func = [](std::span<const value_t> args, environment&) -> value_t {
            auto r = std::get<double>(args[0]);
            return M_PI * r * r;
        },
        .params = {"radius"},
        .doc = "Area of a circle"
    }
});

// Register a module with DSL-defined utility functions
reg.register_module_dsl("util", R"(
    (defun identity (x) x)
    (defun compose (f g) (lambda (x) (f (g x))))
    (defun repeatedly (n f) (for i 0 n (f i)))
)");

// Materialize into the state tree
auto& stdlib_node = root_state.child("__state_exec__")
                               .child("__global__")
                               .child("__stdlib__");
reg.materialize(stdlib_node);
```

After `materialize()`, DSL programs can:

```lisp
(require "geometry")
(distance 0 0 3 4)         ;; → 5.0

(require "util")
(repeatedly 5 (lambda (i) (print i)))  ;; prints 0-4
```

##### 4.17.6 Security Considerations

- **Network access** (`url-fetch`, `url-post`): These are powerful operations.
  The evaluator gates them through a configurable **URL allowlist** stored at
  `__stdlib__.io.__url_allowlist__` (list of allowed URL prefixes, or `"*"`
  for unrestricted).  Requests to disallowed URLs return an error, not an
  exception (the process is not killed — it can handle the error).
  The allowlist is admin-writable (cluster-consensus-enforced, §4.19).

- **Regex denial of service**: `regex-match` and `regex-replace` use
  `std::regex` which can exhibit catastrophic backtracking on pathological
  patterns.  Mitigation: the evaluator wraps regex execution in a
  `max_time`-aware timeout check — if the regex call takes longer than a
  configurable threshold (default 100 ms), it is aborted and returns an error.

- **Crypto / hashing**: `sha256` and `base64-*` are read-only transforms
  with bounded output.  No security concern beyond the usual input
  validation.

- **Native function safety**: Registered native functions run in the same
  process as the evaluator.  Downstream libraries are responsible for
  ensuring their native functions do not crash, leak, or block indefinitely.
  The `stdlib_registry` documents this contract: native functions must be
  pure or have bounded side effects, must not throw exceptions (return
  error values instead), and must complete in bounded time.

#### 4.18 Data Objects, Local Variable Data, and Batched State Changes

##### 4.18.1 Data Objects in the DSL

`cvc::state` nodes carry two independent data channels:

1. **`value()`** — a `std::string` interpreted via `boost::lexical_cast`.
   This is what the existing `(state-get)` / `(state-set)` intrinsics read
   and write.
2. **`data()`** — a `boost::any` holding an arbitrary C++ object (geometry
   meshes, volume data, matrices, etc.).  This is how `state_object<T>`
   subclasses and the broader `cvc` ecosystem store typed domain data.

Prior plan sections only expose `value()` to DSL programs.  This section
adds first-class DSL support for the `data()` channel.

A new `data_object` variant in `value_t` (see §4.1) wraps `boost::any` with
a type-name tag.  DSL programs can:

- **Read** data from a state node into a local variable:
  ```lisp
  (let ((mesh (state-data-get "/scene/model/geometry")))
    (if (data? mesh)
        (begin
          (print (data-type mesh))    ;; e.g. "CVC::Geometry"
          (state-data-set "/output/result" mesh))))
  ```

- **Store** data objects in local variables, pass them to functions, put
  them in lists/dicts, and return them from functions — they are first-class
  `value_t` values.

- **Write** data objects back to a state node:
  ```lisp
  (state-data-set "/scene/model/transformed" transformed-mesh)
  ```

- **Inspect** the type at runtime:
  ```lisp
  (data-type mesh)          ;; → "CVC::Geometry" or nil
  (data? mesh)              ;; → t
  (data? 42)                ;; → nil
  (state-data-type "/path") ;; → type name of data() on that node
  ```

##### 4.18.2 Local Variable Data Objects

Environment bindings (`environment::bindings` — see §4.1) map names to
`value_t`.  Since `data_object_ptr` is a `value_t` variant, data objects
participate in all normal variable operations:

| Operation | Behavior |
|---|---|
| `(set x (state-data-get path))` | Binds `x` to a `data_object_ptr` captured from the state tree |
| `(set y x)` | Copies the shared_ptr — `y` and `x` alias the same `boost::any` payload |
| `(list x y z)` | Data objects can be list elements |
| `(dict-set d "mesh" x)` | Data objects can be dict values |
| `(defun process (obj) ...)` | Data objects can be function arguments |
| `(if (data? x) ...)` | Type predicate works on any `value_t` |
| `(eq x y)` | True if same `data_object_ptr` (identity comparison) |
| `(state-data-set path x)` | Writes the `boost::any` payload back to the tree |

**Serialization caveat**: When an `evaluator_state` is serialized to the
state tree (for migration or persistence), `data_object` values in local
variables are encoded by writing their payload to a temporary state node
via `state::data()`.  This works for types that survive `boost::any` copy
semantics, but types that hold non-copyable resources (file handles, GPU
buffers) will fail at serialization time.  The codec detects this and
stores a sentinel `__type__` = `"data_object_expired"` with an error
message, so the migrated process sees `nil` for that binding.

##### 4.18.3 Data Objects and `state_object<T>`

The `state_object<T>` template (`cvc/state_object.h`) is the primary way
C++ classes expose member data through the state tree.  DSL programs
interact with state_object instances naturally:

- **Reading fields**: `(state-get "/TypeName/0x.../width")` reads the string
  value; `(state-data-get "/TypeName/0x.../geometry")` reads the typed
  data object.
- **Writing fields**: `(state-set "/TypeName/0x.../width" 1920)` sets the
  string value and triggers `handleStateChanged("width")` on the C++ object.
  `(state-data-set "/TypeName/0x.../geometry" mesh)` sets the data object
  and triggers `dataChanged` on the node.
- **Batch writes** (§4.18.4): Multiple field writes can be batched so
  `handleStateChanged` fires once per field, after all writes complete.

##### 4.18.4 Batched State Changes

`state_object<T>` provides batch semantics via `beginBatch()`/`endBatch()`
and the RAII wrapper `state_change_batch_scope<T>`.  During a batch:
- `childChanged` signals are queued in a `std::set<std::string>` (deduplicating
  repeated writes to the same child).
- `handleStateChanged()` is deferred until the batch ends.
- Nested batches are reference-counted (`_batchDepth`); only the outermost
  `endBatch()` triggers the flush.

The DSL exposes this via two intrinsics:

**`(state-batch body...)`** — Evaluates all `body` forms while state change
handlers are batched.  Implementation:

1. The intrinsic looks up the nearest `state_object` ancestor for the
   process’s state subtree.
2. Calls `beginBatch()` before evaluating `body`.
3. Evaluates each form sequentially.
4. Calls `endBatch()` in a scope guard (even if a form signals an error).
5. Returns the value of the last form.

Example:
```lisp
;; Without batching: handleStateChanged fires 3 times, possibly spawning
;; 3 handler threads.
(state-set "/scene/camera/x" 10)
(state-set "/scene/camera/y" 20)
(state-set "/scene/camera/z" 30)

;; With batching: handleStateChanged fires once per unique child, after
;; all three writes complete.  If the camera object batches internally,
;; this collapses to a single flush.
(state-batch
  (state-set "/scene/camera/x" 10)
  (state-set "/scene/camera/y" 20)
  (state-set "/scene/camera/z" 30))
```

**`(state-lock path body...)`** — Acquires the `state_lock_scope` on the
`state_object` at `path` for the duration of `body`.  Other threads
(including handler threads from other DSL processes) that attempt to lock
the same object will block until `body` completes.  This provides atomicity
when multiple fields must be read or written consistently.

Example:
```lisp
(state-lock "/scene/camera"
  (let ((x (state-get "/scene/camera/x"))
        (y (state-get "/scene/camera/y")))
    ;; x and y are consistent — no other writer can interleave
    (state-set "/scene/camera/distance"
               (sqrt (+ (* x x) (* y y))))))
```

**Batch + Lock composition**: `(state-lock)` implicitly begins a batch
for the locked object’s scope.  This means writes inside a lock are always
batched.  Programs can nest `(state-batch)` inside `(state-lock)` (the
batch depth increments; no conflict).

##### 4.18.5 Scheduler-Level Batching

The scheduler itself uses batching internally when writing process metadata
to the state tree.  Each scheduler tick may update `step_count`,
`message_count`, `current_bytes`, and `status` for one or more processes.
These writes are wrapped in a batch scope on the cluster’s state subtree
so that observation tools (and DSL programs watching process nodes) see a
consistent snapshot per tick rather than a partial update.

Similarly, process forking (§4.16) deep-copies an entire subtree.  The
fork operation wraps all child-node creation in a batch scope, so the
parent’s `handleStateChanged` (if any) fires once for the new child process
node rather than once per cloned descendant.

##### 4.18.6 Memory Accounting for Data Objects

Data objects stored in local variables (`data_object_ptr` in environment
bindings) are not tracked by the `memory_tracker` (§4.9) because they are
in-memory shared_ptrs, not state tree writes.  Memory tracking applies when
a program writes a data object to the state tree via `(state-data-set)`:
the size estimate uses `sizeof(boost::any)` plus the stored type's size
if determinable, otherwise a configurable default (e.g., 256 bytes).  The
normal ownership rules (§4.9) apply to the state node written.

#### 4.19 State Tree Access Control

Access control in state_exec follows a **local sovereignty / cluster consensus**
model.  The core principle: a process can write to *any* path in its local
`cvc::state` tree without local blocking.  The local evaluator and intrinsics
layer perform **no ACL checks** on `state-set` or `state-data-set`.  Access
control is enforced by the cluster's replication/consensus layer, which can
reject writes that violate per-subtree ACL policies.

##### 4.19.1 Local Write Behavior

All writes succeed locally.  When a process calls `(state-set "/some/path" v)`,
the intrinsics layer writes to the local `cvc::state` tree immediately.  There
is no path-based blocking, no admin-check gate, and no pre-write ACL lookup.
The process observes the write in subsequent reads on the same node.

##### 4.19.2 Cluster Consensus Enforcement

When a local write is replicated to the cluster via `state_cluster_shard`, the
receiving nodes (or the scheduling/consensus node) check the `__acl__` metadata
of the target subtree:

1. **ACL lookup**: Walk from the target node toward the root, looking for the
   nearest ancestor that has an `__acl__` child node.  The first `__acl__`
   found defines the access policy for the subtree.
2. **Admin check**: The `__acl__` node contains an `admin` child listing
   authorized UIDs and a `delegates` child mapping sub-paths to delegated
   admin UIDs.  If the writing process's UID matches the admin list or is a
   delegate for the target sub-path, the write is accepted.
3. **Rejection**: If the UID is not authorized, the cluster rejects the write.
   The write is **not** applied to the cluster's authoritative state.  An
   `acl_denied` error is sent back to the originating node via the replication
   error channel.

##### 4.19.3 Error Propagation

When the cluster rejects a write, the originating node's
`state_cluster_shard` receives an `acl_denied` response.  This is delivered
to the process as a non-fatal error:

- If the process has a signal handler for `acl_denied`, the handler is invoked.
- If no handler is registered, the error is logged and silently discarded.
- The local tree **retains** the written value — the user's local state is
  their own.  The cluster simply does not accept the write into the shared
  authoritative copy.

A process that patches its own local tree in defiance of cluster ACL is
"off-consensus" for that subtree.  Other nodes will not see the write, and
subsequent replication of the authoritative version may overwrite the local
patch.

##### 4.19.4 Per-Subtree ACL Nodes

`__acl__` nodes can appear at any level of the state tree.  Each `__acl__` node
governs the subtree rooted at its parent:

```
<cluster_id>
├── __acl__                            (cluster-level ACL)
│   ├── admin          "uid1,uid2"     (cluster admins)
│   └── delegates                      (subtree delegation)
│       ├── __processes__.42   "uid3"  (uid3 admins process 42's subtree)
│       └── custom_data        "uid4"  (uid4 admins custom_data subtree)
├── __processes__
│   └── ...
└── custom_data
    ├── __acl__                        (overrides cluster-level ACL for this subtree)
    │   ├── admin      "uid4,uid5"
    │   └── read       "all"
    └── ...
```

Sensitive or secure data subtrees **should** have an `__acl__` node.  It is the
cluster administrator's responsibility to set up ACL nodes for subtrees that
require protection.  Subtrees without `__acl__` nodes inherit the nearest
ancestor's policy; if no ancestor has an `__acl__` node, writes are
unrestricted.

##### 4.19.5 Admin Delegation

Cluster admins can delegate write authority for specific subtrees to non-admin
UIDs by adding entries under `__acl__.delegates`.  Delegation is tree-scoped:
a delegate for path `X` can write to `X` and all descendants of `X`, but not
to sibling subtrees.

Delegation is itself admin-only — only a UID listed in the parent `__acl__.admin`
can modify the `__acl__` node.

##### 4.19.6 Integration with stdlib

The standard library subtree (`__global__.__stdlib__`) has a default `__acl__`
with `write = "admin"`.  This means:

- Any process can read stdlib entries (used during `(require)` and qualified
  calls).
- Only cluster admins can modify stdlib entries in the cluster's authoritative
  copy.
- A non-admin process that writes to `__stdlib__` locally will see its write
  rejected on replication.  The process can choose to handle or ignore the
  `acl_denied` error.

##### 4.19.7 No Local Enforcement by Design

The deliberate absence of local ACL enforcement reflects a sovereignty model:
the local `cvc::state` tree belongs to the node operator.  They can patch it
however they like.  The cluster consensus layer protects the shared state from
unauthorized modifications, but does not prevent a node from diverging locally.
This design avoids a class of failure modes where a misconfigured ACL blocks
a node from operating, and aligns with the principle that `cvc::state` itself
has no built-in ACL.

---

### 5. File Layout

All new files go under `/home/joe/src/libcvc/` (the working copy, not cvc/libcvc):

```
inc/cvc/
    state_list.h                        # Ordered list abstraction over state
    state_memory_manager.h              # LRU/eviction memory management for state
    state_eviction_store.h              # Abstract + built-in eviction store impls
    state_exec/
        types.h                         # value_t, symbol, closure, environment, data_object
        parser.h                        # S-expression parser
        builtins.h                      # Built-in function registry
        evaluator.h                     # Sync recursive evaluator
        async_evaluator.h               # Async recursive evaluator (coroutines)
        stackless_evaluator.h           # Sync stackless evaluator + evaluator_state
        async_stackless_evaluator.h     # Async stackless evaluator
        scheduler.h                     # Sync scheduler
        async_scheduler.h              # Async scheduler
        process.h                       # Process struct and status enum
        memory_tracker.h                # Per-process memory accounting
        state_value_codec.h             # value_t <-> state tree encoding (incl. data_object)
        intrinsics.h                    # DSL intrinsics for state/scheduler interaction
        messaging.h                     # Message bus integration for programs
        resource_policy.h               # Cluster resource policy definition + validation
        distributed.h                   # Scheduling coordination protocol, migration
        stdlib_registry.h              # Standard library registration and resolution
        state_batch_guard.h            # RAII batch scope for state_object-unaware code paths

src/cvc/
    state_list.cpp
    state_memory_manager.cpp
    state_eviction_store.cpp
    state_exec/
        parser.cpp
        builtins.cpp
        evaluator.cpp
        async_evaluator.cpp
        stackless_evaluator.cpp
        async_stackless_evaluator.cpp
        scheduler.cpp
        async_scheduler.cpp
        process.cpp
        memory_tracker.cpp
        state_value_codec.cpp
        intrinsics.cpp
        messaging.cpp
        resource_policy.cpp
        distributed.cpp
        stdlib_registry.cpp
        state_acl.cpp                   # ACL node lookup and consensus-side check helpers
        stdlib_builtins.cpp             # Built-in stdlib modules (string, math, io, collections)

src/cvc/tests/
    state_list_test.cpp
    state_memory_manager_test.cpp        # LRU eviction, repopulation, budget enforcement
    state_exec_parser_test.cpp
    state_exec_builtins_test.cpp
    state_exec_evaluator_test.cpp
    state_exec_async_evaluator_test.cpp
    state_exec_stackless_evaluator_test.cpp
    state_exec_async_stackless_evaluator_test.cpp
    state_exec_scheduler_test.cpp
    state_exec_async_scheduler_test.cpp
    state_exec_memory_tracker_test.cpp   # Per-process memory limits
    state_exec_state_codec_test.cpp
    state_exec_intrinsics_test.cpp
    state_exec_messaging_test.cpp        # Message bus integration + IPC
    state_exec_fork_test.cpp             # Process forking, memory ownership cloning
    state_exec_resource_policy_test.cpp  # Cluster policy enforcement, clamp/strict/warn
    state_exec_stdlib_test.cpp           # Standard library: require, module resolution, ACL, native/DSL
    state_exec_acl_test.cpp              # ACL enforcement: local sovereignty, cluster consensus, delegation
    state_exec_data_object_test.cpp      # Data objects, local variable data, batched state changes
```

### 6. Test Porting Strategy

The existing Python tests (~250 tests across 2 files, ~6 700 lines) need to be
ported to C++ (Google Test, matching the existing libcvc test infrastructure).

#### 6.1 Script Evaluator Tests (~120 tests)

Map per category:

| Python 
| Parsing (basic, nested, numbers, booleans, strings, errors) | `state_exec_parser_test.cpp` | 6 |
| Built-in functions (arithmetic, list ops, dict, etc.) | `state_exec_builtins_test.cpp` | 4 |
| Special forms (if, begin, while, for, set, quote, lambda, return, let) | `state_exec_evaluator_test.cpp` | ~15 |
| Functions (defun, closures, scoping) | `state_exec_evaluator_test.cpp` | ~10 |
| OOP (defclass, inheritance, send, super, wrap_to_py) | `state_exec_evaluator_test.cpp` | ~10 |
| Macros (defmacro, eval, template substitution) | `state_exec_evaluator_test.cpp` | ~10 |
| Async evaluator (basic, defun, control flow, lambda, defclass, macros, async callbacks, concurrent) | `state_exec_async_evaluator_test.cpp` | ~15 |
| Stackless evaluator (basic, step-by-step, serialization, pause/resume, interrupt, timeout, defclass, equivalence, thread safety) | `state_exec_stackless_evaluator_test.cpp` | ~25 |
| Async stackless (basic, step-by-step, serialization, async callbacks, interrupt, concurrent) | `state_exec_async_stackless_evaluator_test.cpp` | ~10 |
| Callbacks (on_complete for all evaluator types) | split across evaluator test files | ~15 |
| Statistics (step count, timestamps, serialization, thread safety) | split across evaluator test files | ~15 |

#### 6.2 Scheduler Tests (~130 tests)

| Python test category | C++ test file | Count |
|---|---|---|
| Scheduler creation and policy | `state_exec_scheduler_test.cpp` | 3 |
| Process submission and execution | `state_exec_scheduler_test.cpp` | ~10 |
| Scheduling policies (round-robin, priority, priority-RR fairness) | `state_exec_scheduler_test.cpp` | ~20 |
| Blocking step | `state_exec_scheduler_test.cpp` | ~5 |
| Process control (pause, resume, kill, set_priority, max_steps, timeout) | `state_exec_scheduler_test.cpp` | ~20 |
| Signal handling | `state_exec_scheduler_test.cpp` | ~5 |
| Scheduler loop (run with limits, stop, callbacks) | `state_exec_scheduler_test.cpp` | ~5 |
| Process info and stats | `state_exec_scheduler_test.cpp` | ~10 |
| Concurrency (thread safety, background execution) | `state_exec_scheduler_test.cpp` | ~5 |
| Serialization (JSON round-trip, cross-machine simulation, paused processes) | `state_exec_scheduler_test.cpp` | ~15 |
| UID/GID | `state_exec_scheduler_test.cpp` | ~10 |
| Async scheduler (all of the above, async variants) | `state_exec_async_scheduler_test.cpp` | ~25 |

#### 6.3 New Tests (not from Python)

| Category | C++ test file | Description |
|---|---|---|
| state_list | `state_list_test.cpp` | Ordered list operations, indexing, iteration, edge cases |
| State memory manager | `state_memory_manager_test.cpp` | LRU/LFU eviction, budget enforcement, on-demand repopulation, eviction stores (memory, blob, null), watermarks, evicted-node reads, concurrent access |
| State codec | `state_exec_state_codec_test.cpp` | Round-trip encoding/decoding of all value types, evaluator state, environments |
| DSL intrinsics | `state_exec_intrinsics_test.cpp` | `state-get`, `state-set`, `ps`, `spawn`, `kill`, etc. |
| Process memory | `state_exec_memory_tracker_test.cpp` | Per-process memory accounting, limit enforcement, `(memory-usage)` intrinsic, OOM termination, `fork_ownership` cloning |
| Time limits | `state_exec_time_limit_test.cpp` | Wall-clock time enforcement, paused-time exclusion, `(time-elapsed)` / `(time-limit)` intrinsics, `time_limit_exceeded` termination, interaction with max_steps |
| Message limits | `state_exec_message_limit_test.cpp` | `message_count` tracking across `msg-send`/`ipc-send`/`msg-broadcast`, limit enforcement, `message_limit_exceeded` termination, `(message-count)` / `(message-limit)` intrinsics |
| State expiry | `state_exec_expiry_test.cpp` | `state-expire`, `state-expire-at`, `state-clear-expiry`, `state-is-expired`, `state-sweep-expired` intrinsics, scheduler sweep integration, terminated process auto-cleanup via expiry, running-process-node-expired handling, expiry + distributed replication |
| Resource policy | `state_exec_resource_policy_test.cpp` | `strict`/`clamp`/`warn` enforcement modes, default application, `max_processes` cap, `max_total_memory` aggregate budget, runtime adjustment validation, `(cluster-policy)` intrinsic |
| Standard library | `state_exec_stdlib_test.cpp` | `(require)` module import, qualified `stdlib.module.func` calls, `(stdlib-modules)` / `(stdlib-list)`, ACL enforcement (cluster-consensus rejection of non-admin writes), native function registration via `stdlib_registry`, DSL module registration, `materialize()` round-trip, URL allowlist enforcement, downstream module registration from C++ |
| Data objects & batching | `state_exec_data_object_test.cpp` | `(state-data-get)` / `(state-data-set)` round-trip, `(data-type)` / `(data?)` predicates, data_object in local variables / lists / dicts / function args, `(state-batch)` deduplication and deferred handler dispatch, `(state-lock)` exclusivity, batch+lock composition, `state_batch_guard` in codec, data_object serialization/migration (including `data_object_expired` sentinel), memory accounting for data writes |
| ACL | `state_exec_acl_test.cpp` | `__acl__` node lookup (nearest ancestor), admin check, delegate check, local write succeeds without ACL gate, cluster-side rejection of unauthorized writes, `acl_denied` error propagation, local patch retained after rejection, delegation CRUD, stdlib ACL integration, ACL inheritance (no `__acl__` → unrestricted) |
| Distributed migration | `state_exec_distributed_test.cpp` | Serialize process, send migrate message, re-register handlers on target node, resume on peer, test handler continuity across nodes |

### 7. Implementation Phases

#### Phase 1: Foundation (no distributed features)

1. **`state_list`** ✅ — ordered child list wrapper for `cvc::state`.  Add to
   `cvc::state` API.  Write tests.  *Completed: header, source, 20 passing
   tests, integrated into CMakeLists.*
2. **`state_memory_manager`** ✅ — LRU eviction, eviction stores, budget
   enforcement, on-demand repopulation.  General `cvc::state` feature.
   *Completed: state_eviction_store interface + memory/null impls,
   state_memory_manager with LRU/LFU/size-weighted/TTL/manual policies,
   watermarks, callbacks, 20 passing tests.*
3. **Core types** ✅ — `value_t`, `symbol`, `closure`, `environment`.
   *Completed: types.h/cpp with 11-variant value_t, environment scope
   chain, to_string, values_equal, 26 passing tests.*
4. **Parser** ✅ — S-expression parser producing `value_t`.  Port all 6 parser
   tests.  *Completed: parser.h/cpp with atoms, lists, strings, quotes,
   comments, error locations, 30 passing tests.*

#### Phase 2: Evaluators + Built-ins

5. ✅ **`builtins`** — Built-in function registry (`define`, `lambda`, `if`,
   `cond`, `let`, `begin`, `quote`, `defclass`, `new`, arithmetic,
   comparison, list operations, type predicates).  Port all built-in tests.
   *(38 tests passing)*
6. ✅ **`evaluator`** (sync recursive) — Full S-expression evaluator with OOP
   support (`defclass`/`new`/method dispatch), closures, tail-call
   optimization.  Port ~50 evaluator tests.
   *(39 tests passing)*
7. ✅ **`stackless_evaluator`** (sync stackless) — Explicit-stack evaluator
   with step-by-step execution.  Port ~50 stackless evaluator tests + OOP
   tests.
   *(31 tests passing)*
8. **`environment`** — Scope chains, closures, environment serialization.
   Shared by all evaluator variants.
9. **Evaluator state serialization** — To/from `value_t` and state tree
   (frame stack, environments, results).  Port serialization tests.
10. **Evaluator integration tests** — Cross-evaluator consistency checks;
    verify all four evaluator variants produce identical results on the
    same programs.

#### Phase 3: Async Evaluators

11. **`async_evaluator`** — C++20 coroutine-based recursive evaluator.  Port
    ~15 async tests.
12. **`async_stackless_evaluator`** — Coroutine stackless evaluator.  Port ~10
    async stackless tests.

#### Phase 4: Scheduler + Resource Limits

13. **`process`** struct and `process_status` enum.
14. **`memory_tracker`** — Per-process memory accounting.  Tracks allocations
    in `value_t` operations, checks against `max_memory` at step boundaries.
15. **Time limit enforcement** — `max_time` field on process, elapsed-time
    tracking (paused time excluded), `time_limit_exceeded` kill at step
    boundary.  `set_max_time()` API.
16. **Message limit enforcement** — `max_messages` field on process,
    `message_count` incrementing on every outbound message operation,
    `message_limit_exceeded` kill at step boundary.  `set_max_messages()` API.
17. **`scheduler`** (sync) — Process management, scheduling policies, signal
    handling, stepping, memory/time/message limit enforcement, process
    forking.  Port ~80 scheduler tests + new limit + fork tests.
18. **`async_scheduler`** — Async variant.  Port ~25 async scheduler tests.
19. **Scheduler serialization** — To/from state tree and JSON.

#### Phase 5: State Integration + Intrinsics + Messaging + Expiry + Stdlib + Data Objects

20. **DSL intrinsics** — `state-get`, `state-set`, `ps`, `spawn`, `kill`,
    `memory-usage`, `memory-limit`, `time-elapsed`, `time-limit`,
    `message-count`, `message-limit`, etc.
21. **Expiry intrinsics** — `state-expire`, `state-expire-at`,
    `state-has-expiry`, `state-expiry-time`, `state-clear-expiry`,
    `state-is-expired`, `state-sweep-expired`.  Scheduler sweep integration:
    configurable tick interval, `pending_expiry_count()`.
22. **Messaging intrinsics** — `(msg-send)`, `(msg-subscribe)`,
    `(msg-unsubscribe)`, `(ipc-send)`, `(ipc-recv)`, broadcasting.
    Port messaging tests.
23. **Observation intrinsics** — `(ps)`, `(ps-all)`, `(inspect)`,
    `(state-watch)`.  Port observation tests.
24. **`resource_policy`** — `validate()` and runtime `set_*` calls.
    `(cluster-policy)` DSL intrinsic.  Policy enforcement integration.
25. **Standard library** — `stdlib_registry` singleton, built-in modules
    (`string`, `math`, `io`, `collections`), `materialize()` to populate
    `__stdlib__` subtree, `(require)` intrinsic, qualified
    `stdlib.module.func` call syntax, ACL enforcement (cluster-consensus),
    URL allowlist for `io` module, downstream C++ registration API.
26. **Data objects and batching** — `data_object` variant in `value_t`,
    `(state-data-get)` / `(state-data-set)` / `(data-type)` / `(data?)`
    intrinsics, `(state-batch)` and `(state-lock)` intrinsics wrapping
    `state_change_batch_scope` / `state_lock_scope`, `state_batch_guard`
    RAII helper in `state_value_codec`, data_object serialization for
    migration.  Write data object and batching tests.
27. **State tree ACL** — `state_acl` helpers for `__acl__` node lookup
    (nearest ancestor walk), admin/delegate UID checks, `acl_denied` error
    type.  Integration with `state_cluster_shard` replication hook for
    consensus-side write rejection.  Write ACL tests.

#### Phase 6: Distributed Execution (message-based coordination)

28. **Scheduling coordination messages** — Election, heartbeat, handoff via
    `state_message_bus` and `state_message` with typed `content_type`.
29. **Process submission via messages** — Clients submit programs to the
    scheduling node via `state::sendMessage()`.  Resource policy validation
    applied on the scheduling node before accepting.
30. **Process migration** — Pause, serialize to state tree, send `migrate`
    message to target node, resume on peer.  Full handler migration protocol:
    pause, serialize evaluator state + subscription/watch/signal handler
    registrations + IPC inbox into state tree, tear down origin-side
    registrations, send `migrate` message to target, target re-registers
    all handlers on local `state_message_bus` and `cvc::state` signals,
    resume on peer.  Test handler continuity across nodes.
31. **Cross-cluster observation** — `ps-all`, `cluster_stats()`, aggregated
    views.
32. **Admin controls** — Cluster admin can control execution on their cluster
    and hand off to peers, all through the message bus.  Includes
    `application/x-state-exec-policy` messages for remote policy updates.
33. **Integration tests** — End-to-end distributed execution scenarios
    including messaging, all resource limits, expiry sweep, policy
    enforcement, forking, ACL, and eviction under load.

#### Phase 7: CMake Integration + Polish

34. **CMakeLists.txt** updates — new source files, test targets, feature flags
    (`CVC_STATE_EXEC`, `CVC_STATE_MEMORY_MANAGER`).
35. **Documentation** — Header-level Doxygen comments.
36. **Benchmark** — Performance comparison with Python implementation;
    memory-pressure stress tests; message-flood stress tests.

---

### 8. Key Design Decisions

#### 8.1 C++20 Features Used

| Feature | Usage |
|---|---|
| `std::variant` | `value_t` — the universal DSL value type |
| `std::span` | Passing argument lists to built-in functions |
| Concepts | Constraining template parameters (e.g., `Evaluatable`, `Steppable`) |
| Coroutines (`co_await`/`co_return`) | `async_evaluator`, `async_stackless_evaluator`, `async_scheduler` |
| `std::jthread` | Managed threads for timeout, parallel stepping |
| `std::stop_token` | Cooperative cancellation for evaluator timeout |
| `std::format` | Error message formatting |
| Designated initializers | Struct initialization for `process`, `eval_frame`, etc. |
| `std::ranges` | Filtering/transforming process lists in scheduler |
| Three-way comparison (`<=>`) | `symbol` ordering |
| `constexpr` / `consteval` | Compile-time builtin registration |
| Structured bindings | Throughout |
| `[[nodiscard]]`, `[[likely]]`/`[[unlikely]]` | API safety + branch hints in hot eval loop |

#### 8.2 No RestrictedPython Equivalent — Local Sovereignty Model

The Python implementation uses `RestrictedPython` to sandbox built-in function
source code.  In C++, built-ins are compiled directly — there is no runtime
code injection.  Security comes from:

- Built-ins are a closed set at compile time
- The DSL cannot call arbitrary C++ functions
- State tree access is mediated through the intrinsics API but **not locally
  restricted** — the evaluator does not block writes to any path
- Process UID/GID enable access control policies **at the cluster consensus
  level**, not at the local evaluator level
- Standard library modules use cluster-consensus ACL enforcement (§4.19);
  native functions registered via `stdlib_registry` run in-process under the
  documented safety contract (§4.17, §8.9)

Locally, a process can write to any path in the state tree.  The cluster's
replication layer (not the evaluator) enforces ACL policies.  Rejected writes
propagate an `acl_denied` error back to the process, but the process may
ignore it — its local tree remains patched.  This is by design: the local
state tree is sovereign to the node operator.  See §4.19 for the full model.

#### 8.3 State Tree vs. In-Memory Representation

During active evaluation, the evaluator works with the fast in-memory `value_t`
representation.  The state tree encoding is used for:

- Persistence (saving/restoring programs)
- Replication (distributed execution via `state_cluster_shard`)
- Observation (external tools reading process state)
- Migration (moving a process to another node)

The `state_value_codec` bridges these two worlds.  A "dirty flag" pattern
avoids unnecessary re-encoding during tight evaluation loops — the evaluator
writes back to the state tree periodically or on pause/migration.

#### 8.4 Scheduling Node vs. Fully Decentralized

A single scheduling node per cluster simplifies the design:

- No consensus needed for each step
- Deterministic scheduling policy behavior
- Simple implementation of priority and round-robin
- Migration and signal delivery have a clear target

The tradeoff is a single point of failure for scheduling, mitigated by the
message-based election protocol, heartbeat monitoring, and fast handoff (§4.7.1).
All coordination traffic uses the existing `state_message_bus` / transport
infrastructure — no separate protocol layer is needed.

#### 8.5 Two-Tier Memory Management

Memory management operates at two levels:

- **Tree-level (`state_memory_manager`, §3.3)**: LRU/LFU eviction of state
  node payloads to keep the overall `cvc::state` tree within a configurable
  memory budget.  Evicted payloads are written to a backing store and
  repopulated on demand.  This is transparent to DSL programs.
- **Process-level (`memory_tracker`, §4.9)**: Per-process byte accounting.
  Each process owns the state-tree nodes it writes.  A
  `state::valueChanged` / `state::dataChanged` callback on the process's
  nodes updates the per-PID byte counter.  Ownership transfers on write:
  if process B overwrites process A's node, ownership (and the byte charge)
  transfers to B while crediting A.  Enforced at step boundaries — if a
  process exceeds its `max_memory` limit it is killed with
  `memory_limit_exceeded`.

The two tiers are complementary: the state memory manager keeps the overall
tree footprint under control, while per-process limits prevent any single
program from monopolizing the shared state tree.  In-evaluator temporaries
(local variables, intermediate results) are *not* counted toward the process
memory limit — they are bounded by `max_steps` and by the overall
`state_memory_manager` budget.

#### 8.6 Messaging as the Universal Coordination Primitive

All distributed coordination (election, handoff, process submission, control
commands, status broadcast, signal delivery, migration, inter-process
communication) is built on `state_message` / `state_message_bus`.  This is a
deliberate design choice:

- **Single mechanism**: No ad-hoc RPC or custom protocol channels.  Everything
  flows through the same path-prefix-matched, dedup-protected, transport-agnostic
  message bus.
- **Transport reuse**: Messages piggyback on whatever `state_transport` is
  configured (gRPC, IPC, etc.) — no additional network setup.
- **Observability**: All coordination traffic is visible via the bus's
  counters (`total_admitted`, `total_dispatched`, `total_dropped`).
- **Extensibility**: New message types are just new `content_type` strings;
  no schema changes to the bus or transport.

#### 8.7 Unified Resource Limit Enforcement

All per-process resource limits (`max_steps`, `max_memory`, `max_time`,
`max_messages`) follow the same enforcement pattern:

1. Checked at **step boundaries** (not mid-evaluation) — this ensures the
   evaluator is always in a consistent state when a limit fires.
2. A value of **`0` means unlimited** — no enforcement for that dimension.
3. Termination sets `status = killed` and `exit_error` to a descriptive
   string (e.g., `memory_limit_exceeded`, `time_limit_exceeded`,
   `message_limit_exceeded`, `step_limit_exceeded`).
4. The limit can be **adjusted at runtime** via `set_max_*(pid, value)`.
5. The DSL exposes **read-only intrinsics** for the current value and the
   configured limit, enabling programs to self-monitor and take action.
6. All limits are subject to **cluster resource policy** validation (§4.15).

This uniform pattern makes the enforcement code a single checkpoint function
called once per step, keeping the hot evaluation loop simple.

#### 8.8 Expiry as Infrastructure, Not Logic

`cvc::state` expiry is an infrastructure-level mechanism — nodes expire based
on wall-clock time, and the `sweepExpired()` walk removes them.  state_exec
leverages this for:

- **Automatic cleanup**: terminated process nodes expire after a retention
  period, preventing unbounded state tree growth.
- **Program-controlled caching**: DSL programs can set expiry on data nodes
  to implement TTL-based caches without custom timers.
- **Cluster policy enforcement**: a cluster admin can set expiry on a process
  node as a hard deadline (the process is killed when the node expires).

Expiry is *not* used as a general-purpose timer or scheduling mechanism.
For time-based program logic, programs should use `(sleep)` or condition
variables; expiry is for resource lifecycle management.

#### 8.9 Standard Library as Primary Extensibility Mechanism

The standard library (`__global__.__stdlib__`) is the primary extensibility
mechanism for state_exec.  Rather than requiring DSL programs to call raw
C++ FFI or recompile libcvc, downstream libraries register stdlib modules
via `stdlib_registry::register_native_function()` at startup.  This design
choice has several consequences:

- **Discoverability**: programs can call `(stdlib-modules)` and
  `(stdlib-list mod)` to introspect available operations at runtime.
- **ACL uniformity**: all stdlib functions — whether built-in or
  downstream — share the same read-all / admin-write access model.
  ACL enforcement happens at the cluster consensus level during replication
  (§4.19) but does not authenticate callers — identity verification and
  credential management are separate concerns.
- **Replication**: the entire `__stdlib__` subtree is replicated across
  clusters, so registered functions are available everywhere without
  per-node configuration.
- **Dual implementation**: functions can be written as native C++ (for
  performance) or pure DSL (for portability).  The stdlib metadata records
  which implementation each function uses; the evaluator dispatches
  accordingly.
- **Safety boundary**: native functions run in-process and can crash the
  evaluator.  The `stdlib_registry` contract requires bounded execution
  time, no exceptions (return error values), and no unbounded memory
  allocation.  Downstream authors accept this responsibility.

#### 8.10 Data Objects as First-Class Values with Batched Writes

`cvc::state` has two independent data channels per node: `value()` (string)
and `data()` (`boost::any`).  The original plan exposed only `value()` to
DSL programs.  §4.18 adds `data()` access because the broader cvc ecosystem
(geometry processing, volume rendering, scene graphs via `state_object<T>`)
stores typed domain data via `boost::any`, not strings.

Key design choices:

- **Wrapping, not unwrapping**: DSL programs see `data_object` as an opaque
  handle.  They can inspect the type name and pass the object around, but
  cannot reach into the `boost::any` to extract fields.  This is deliberate
  — the DSL is not a C++ REPL; domain logic lives in native C++ code.
- **Shared ownership**: `data_object_ptr` is a `shared_ptr`.  When a
  program binds a data object to a local variable and then passes it to
  another function, both references share the same `boost::any` payload.
  This avoids deep-copy overhead for large objects (meshes, volumes).
- **Batching via `state_object` semantics**: The DSL's `(state-batch)`
  intrinsic wraps `state_change_batch_scope<T>` (§4.18.4).  This bridges
  the gap between DSL programs that make many small writes and C++ objects
  that want coalesced change notifications.  The scheduler itself uses
  batching internally for process metadata updates (§4.18.5).
- **Lock composition**: `(state-lock)` wraps `state_lock_scope<T>` and
  implicitly begins a batch.  This ensures that multi-field reads/writes
  are both consistent and coalesced.

#### 8.11 Local Sovereignty and Cluster-Consensus ACL

Access control in state_exec follows a "local sovereignty" model:

- **The local tree is sovereign**: A process can write to any path in its
  local `cvc::state` tree.  The evaluator and intrinsics layer perform no
  ACL checks — `state-set` always succeeds locally.
- **Cluster consensus is the enforcement boundary**: When writes replicate
  to the cluster, the consensus layer checks per-subtree `__acl__` nodes
  and rejects unauthorized writes.  The cluster's authoritative copy is
  protected; individual nodes' local copies are not.
- **Errors are advisory, not fatal**: An `acl_denied` error from the
  cluster is delivered to the process as a signal.  The process may handle
  it, log it, or ignore it.  The local patch persists until the next
  replication sync overwrites it.
- **Sensitive data requires explicit ACL**: There is no "secure by default"
  — subtrees are unprotected unless a cluster admin places an `__acl__` node.
  Sensitive or security-critical data should always have an `__acl__` node
  with an explicit admin list.
- **Delegation keeps admin lightweight**: Cluster admins can delegate
  subtree control to non-admin UIDs without granting full cluster admin
  rights (§4.19.5).

This design reflects the reality that a node operator controls their own
machine.  We cannot prevent them from patching their local state tree, and
attempting to do so would create fragile failure modes.  Instead, we protect
the *shared authoritative state* at the replication boundary, where the
cluster has actual enforcement power.

#### 8.12 Evaluator Tier Model — Async Stackless as the Only First-Class Evaluator

state_exec provides four evaluator implementations, but they are **not**
interchangeable in a cluster context:

| Evaluator | Async | Stackless | Schedulable | Migratable | Tier |
|---|---|---|---|---|---|
| `evaluator` | no | no | no | no | 2 (local) |
| `async_evaluator` | yes | no | no | no | 2 (local) |
| `stackless_evaluator` | no | yes | no | no | 2 (local) |
| `async_stackless_evaluator` | yes | yes | **yes** | **yes** | **1 (system)** |

**Only `async_stackless_evaluator` is a first-class citizen of the system.**
The scheduler uses it exclusively for all managed processes.  The reasons are
fundamental, not preference:

- **Async is required for scheduling**: Sync evaluators block the event loop.
  A single long-running sync eval step would starve all other processes in
  the scheduler's run queue.  The async `step()` (returning `task<bool>`)
  yields control cooperatively, allowing the scheduler to interleave steps
  across many processes.
- **Stackless is required for migration**: The C++ call stack of a recursive
  evaluator cannot be captured, serialized, or replicated.  The stackless
  evaluator's explicit `evaluator_state` (frame stack + environments +
  macros + stats) lives in ordinary data that can be written to the state
  tree via `state_value_codec`, replicated by `state_cluster_shard`, and
  resumed on a different node with zero loss of execution context.
- **Stackless is required for observation**: External tools and DSL
  intrinsics (`ps`, `inspect`) read process state from the state tree.
  Only the stackless evaluator's state is materialized there.

The three Tier 2 evaluators are deliberately retained because they serve
legitimate local use cases:

- **`evaluator`** (sync recursive): Simplest possible evaluation path.
  Ideal for one-shot `evaluate("(+ 1 2)")` calls in C++ application code,
  REPL loops, test harnesses, and any context where blocking is acceptable
  and migration is irrelevant.
- **`async_evaluator`** (async recursive): Non-blocking local evaluation
  for embedding in an `io_context` or coroutine-driven application without
  needing the overhead of an explicit frame stack.
- **`stackless_evaluator`** (sync stackless): Step-debuggable and
  serializable, but blocks the caller.  Useful for batch evaluation,
  deterministic testing, and offline state-tree inspection.

Developers are free to instantiate any evaluator directly for local purposes.
However, the scheduler's `execute()` and `fork()` always create an
`async_stackless_evaluator`.  There is no API to override this — it is a
deliberate architectural constraint, not a missing feature.

---

### 9. Dependencies

| Dependency | Purpose | Status |
|---|---|---|
| Boost (property_tree, signals2, thread, chrono) | Existing cvc::state dependency | Already present |
| Google Test | Test framework | Already present in libcvc |
| nlohmann/json (or Boost.JSON) | JSON serialization for cross-machine compat | Evaluate; may use existing ptree JSON |
| C++20 coroutines | async evaluators/scheduler | Compiler support required (GCC 11+, Clang 14+) |

No new external dependencies anticipated beyond what libcvc already uses,
except for optional stdlib modules:

| Dependency | Purpose | Status |
|---|---|---|
| Boost.Beast or libcurl | `io` stdlib module: `url-fetch` | Optional; CMake feature flag `CVC_STATE_EXEC_STDLIB_IO` |
| OpenSSL (libcrypto) | `io` stdlib module: `sha256`, `base64` | Optional; often already present via Boost.Asio |

C++20 coroutine support may require the `<coroutine>` header and a simple
`task<T>` type (can be implemented in ~50 lines or use an existing utility).

---

### 10. Naming Conventions

All Python names are translated following libcvc conventions:

| Python | C++ |
|---|---|
| `CFOSScriptEvaluator` | `cvc::state_exec::evaluator` |
| `AsyncCFOSScriptEvaluator` | `cvc::state_exec::async_evaluator` |
| `StacklessCFOSScriptEvaluator` | `cvc::state_exec::stackless_evaluator` |
| `AsyncStacklessCFOSScriptEvaluator` | `cvc::state_exec::async_stackless_evaluator` |
| `CFOSScheduler` | `cvc::state_exec::scheduler` |
| `AsyncCFOSScheduler` | `cvc::state_exec::async_scheduler` |
| `CFOSProcess` | `cvc::state_exec::process` |
| `EvaluatorState` | `cvc::state_exec::evaluator_state` |
| `EvalFrame` | `cvc::state_exec::eval_frame` |
| `EvaluationStats` | `cvc::state_exec::evaluation_stats` |
| `ProcessStatus` | `cvc::state_exec::process_status` |
| `SchedulingPolicy` | `cvc::state_exec::scheduling_policy` |
| `Symbol` | `cvc::state_exec::symbol` |
| `Closure` | `cvc::state_exec::closure` |

---

### 11. Risk Assessment

| Risk | Mitigation |
|---|---|
| C++20 coroutine complexity | Start with sync evaluators (Phase 1-2), defer coroutines to Phase 3. Provide a minimal `task<T>` implementation. |
| State tree encoding overhead | Use dirty-flag pattern; batch writes. Profile early in Phase 6. |
| Distributed scheduling correctness | Rely on existing `state_cluster_shard` replication and `state_message_bus`. Phase 6 integration tests exercise failure scenarios including split-brain and message loss. |
| `state_list` performance with large lists | Zero-padded keys keep lexicographic = positional order. For very large ASTs, consider chunking. Profile in Phase 7. |
| OOP (`defclass`) complexity in stackless mode | This is the most complex part of the Python port. Allocate extra attention in Phase 2. The Python code uses internal `run()` calls for method closures — model this as sub-state evaluation. |
| Python test parity | Maintain a tracking spreadsheet mapping each Python test to its C++ counterpart. Mark untranslatable tests (e.g., Python interop via `wrap_to_py`). |
| Memory accounting accuracy | Memory tracking counts objects written to the state tree (not `malloc`-level). The ownership side-table adds a per-node `{pid, bytes}` entry (~24 bytes overhead). For programs that create millions of tiny nodes, the side-table itself consumes non-trivial memory — mitigate with a configurable headroom multiplier (e.g., set limit to 80% of real budget). Ownership transfer on overwrite requires an ownership-map lookup per write — profile in Phase 4. |
| LRU eviction thrashing | If budget is too tight and access patterns cycle, nodes may be repeatedly evicted and repopulated. Mitigate with high/low watermarks and `size_weighted_lru` policy. Monitor eviction counters. |
| Message bus ordering | `state_message_bus` provides at-most-once delivery with no ordering guarantees. Election protocol uses timestamps + node_id tiebreakers to tolerate reordering. |
| Eviction-during-evaluation | A state_exec program reading a state node may trigger repopulation I/O. For sync evaluators this blocks the step; for async evaluators it `co_await`s. Document latency implications. |
| Time limit clock skew | `max_time` uses wall-clock time (`boost::posix_time`). On machines with unstable clocks (NTP jumps, VM migration), elapsed time may be inaccurate. Mitigation: use monotonic clock where available; document that `max_time` is best-effort, not real-time guarantee. |
| Message limit evasion | A malicious program could attempt to send messages through side channels (e.g., writing to a state node that triggers a signal which indirectly sends a message). Mitigation: all outbound message paths in the evaluator increment the counter; audit intrinsic implementations. |
| Expiry sweep latency | `sweepExpired()` runs once per scheduler tick. If the tick interval is large, expired nodes may persist slightly beyond their deadline. Mitigation: configurable tick interval; document that expiry is not real-time. |
| Migration handler gap | Between origin teardown and target re-registration, event handlers are not active on any node. Messages arriving during this window will not trigger handler closures. Mitigation: window is brief (one replication round-trip); messages are persistent in the bus and state changes are persistent in the tree; document that handler callbacks may miss events during migration. For critical subscriptions, programs can re-subscribe explicitly after detecting migration via `(node-id)` change. |
| Cluster policy consistency | In a distributed cluster, policy changes propagate through state replication which has latency. A process submitted to a non-scheduling peer during propagation may see a stale policy. Mitigation: policy validation happens on the scheduling node (which has the authoritative copy); submission messages are forwarded there. |
| Aggregate memory budget race | `max_total_memory` is checked on the scheduling node. Concurrent submissions may briefly exceed the budget before the first submission's allocation is recorded. Mitigation: serialize `execute()` calls on the scheduling node (already single-threaded per cluster). |
| Fork memory doubling | Forking a large process deep-copies its entire subtree, temporarily doubling memory usage. If the cluster is near its `max_total_memory` cap, the fork may fail or push other processes toward eviction. Mitigation: fork checks aggregate budget *before* cloning; the copy is atomic from the scheduler's perspective (no partial clone on failure). |
| Fork bomb | A malicious program could fork in a loop, exhausting `max_processes`. Mitigation: `max_processes` cluster policy cap applies to forks; each fork is a new process counted against the cap. A per-process `max_forks` field could be added later if needed (out of scope for now). |

---

### 12. Out of Scope (for this plan)

- GUI/TUI for process observation (the API and DSL intrinsics are provided; a
  UI is a separate project)
- Authentication/authorization framework (UID/GID are recorded; cluster
  resource policies enforce limits but do not authenticate callers — identity
  verification is a separate concern)
- Network transport implementation (uses existing `state_transport` /
  `state_transport_grpc` / `state_transport_ipc`)
- Python bindings for the C++ implementation
- JIT compilation or optimization of DSL programs
- OS-level cgroup / seccomp sandboxing of state_exec processes
- Distributed garbage collection of orphaned blob store entries
- Persistent message queues (messages are transient; durable messaging is a
  separate concern)
