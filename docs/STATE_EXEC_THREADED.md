# state_exec: running programs on several threads

Status: design (2026-10-26). Nothing here is built yet. Tracked in the `state-exec-perf`
workstream, as its third track. The first two are the benchmark suite (transfix/libcvc#834) and the
single-thread optimizations (branches `perf/sched`, `perf/async` and `perf/eval`).

Today every state_exec scheduler steps its processes on one thread. This document designs how
programs run in parallel when several native threads are available. The user asked for two families
of models to be explored: one executor scheduling across N native threads, and one executor per
native thread with handoff between them. Five proposals were written up. Three independent judges
scored all five against the code. Section 3 records the scores and the decision. The rest of the
document is the chosen design, with the strongest ideas of the other four folded in.

This revision replaces a first draft that compared only three of the five proposals, because one
input had been truncated. With all five judged, the decision changed (section 3.3).

**Sources.**

- **File:line references** are to master at 09fa876c. For the runtime sources (`src/cvc/state`,
  `inc/cvc/state`, `src/cvc/ariadne`, `src/cvc/core`, `bindings/pycvc`) this is identical to the
  `feat/state-exec-bench` branch the audit read.
  - The `perf/sched` and `perf/async` branches rewrite parts of `async_scheduler.cpp`,
    `scheduler.cpp`, `process.*`, `app.cpp` and `async_host.cpp`. References into those files are
    to master before them.
  - Section 2.4 says what those branches already change.
- **The benchmark suite** and its documentation are `src/cvc/tests/state_exec_bench.cpp` and
  `docs/STATE_EXEC_BENCH.md` on `feat/state-exec-bench` (#834), not yet on master.
- **The measured numbers** are the first full run of that suite on the development machine:
  - The machine is a Core Ultra 9 185H with 6 P-cores, 8 E-cores and 2 low-power E-cores (22 logical
    CPUs), running Windows. The build is MSVC 14.44 Release.
  - Each point is one rep (`full5.jsonl`; `full6.jsonl` where marked). One-thread repeats read
    1.17-1.20 of themselves (a cold reference run), so treat about +-15% as noise.

## 1. Goal and non-goals

### 1.1 Goal

1. **Parallel groups.** Processes that share no mutable data run at the same time on different
   native threads.
   - The unit is an *execution group*: one `async_scheduler`, with its own async host, cycle
     registry and random stream, stepped by one thread at a time.
   - Groups run in parallel on a shared pool of worker threads.
2. **Inside a group nothing observable changes.** Its round robin, step counts, budgets and message
   order are today's. A group's results on the pool must equal its results run alone (section 8,
   GR-1).
3. **Opt-in and staged.** With the default `state_exec.threads = 0` the runtime behaves as today,
   apart from the fixes of Stage 0 (section 5). Each stage ships on its own and is verified by the
   benchmark suite and by a race detector.
4. **No cost on the step path.** A group's step costs what a standalone scheduler's step costs. The
   pool adds work per pump (a few thread-local stores), never per step.
5. **Hosts first, then programs.** Hosts with independent work gain first, in Stage 1: several
   pycvc `Exec`s, `ExecPool` batches, several Ariadne documents, `exec_coordinator` jobs. DSL
   programs ask for parallelism explicitly with `spawn-isolated`, in Stage 3.
6. **Each host has a defined way in.** Ariadne, pycvc's `Exec` and `exec_coordinator` each have one.

### 1.2 Non-goals

- **Parallelism inside one process.** A process is a sequence of steps, so one long `fib` gets one
  core.
- **Shared mutable values between groups.** There are no per-list, per-dict or per-environment
  locks (section 3.1, option F). Processes that share mutable values keep sharing them, inside one
  group.
- **Several threads inside one group.** This is the audit's model (a). It is Stage 4, designed in
  section 4.16 and built only if Stages 2 and 3 show single groups that are DSL-bound and cannot be
  split.
- **The recursive evaluator and `async_evaluator`.** No scheduler uses them (`scheduler.h:251`,
  `async_scheduler.h:270`), and they cannot change threads. Their state lives on the C++ stack or in
  coroutine frames, and `evaluate()` holds a recursive_mutex for the whole call
  (`evaluator.cpp:28`, `async_evaluator.cpp:67`). They stay one instance per thread.
- **The sync `scheduler`.** It stays the CLI's scheduler and the deterministic reference. It gets
  only Stage 0's control queue and atomic stop flag.
- **Cross-node distribution.** `exec_coordinator` keeps its own design. Section 4.12 says what it
  gains.
- **A concurrent state tree.** This design makes the DSL's use of the tree safe (pinned lookups, no
  tree-wide sweeps). It does not redesign the tree's locking.
- **The single-thread optimization tracks.** This design assumes `perf/sched` (run queue, timer
  heap, one drain and no coroutine frames a step, `wait_for_work`), `perf/async` (persistent
  background pool, concurrent fan-outs) and `perf/eval` (cheaper steps) land first (section 2.4).

## 2. Today

### 2.1 What runs where

- **Schedulers.** Two schedulers step a `stackless_evaluator`, one step at a time:
  - the sync `scheduler` (the CLI, tests, `exec_coordinator`);
  - the `async_scheduler`. Ariadne uses the app-wide one (`app::exec_scheduler()`,
    `app.cpp:1024-1038`). pycvc's `Exec` builds its own (`pycvc_exec.cpp:410-423`).
- **The step loop.** Each step selects a process, sets the scheduler's current process, arms the
  deadline and allocation guards, steps, and accounts (`async_scheduler.cpp:152-300`,
  `310-322`).
- **Async calls.** An async call marshals on the scheduler thread, then runs its kernel on the
  app's background pool or a lane. Its completion comes back through the scheduler's ingress, the
  only thread-safe entry (`async_scheduler.cpp:516-556`; `docs/STATE_EXEC_EXCEPTIONS.md` 3.1).
- **Ariadne.** It pumps the app scheduler from every `Runtime::drain` (`ariadne.cpp:2021-2023` ->
  `1688-1706`). The sequence is `drain_ingress`, then `wake_awaiting`, then
  `sync_run(200000, 0.1)`, then the sweep. Every document shares that one scheduler, one pid space
  and one async host (`app.cpp:1041-1047`).
- **pycvc.** `Exec.run` pumps the `Exec`'s private scheduler.
  - It releases the GIL around each slice (`pycvc_exec.cpp:594`).
  - When its program is parked it sleeps 1 ms (`pycvc_exec.cpp:648`), which is about 15.6 ms on
    Windows.
  - Several `Exec`s on several Python threads therefore already step DSL in parallel today. The
    docs recommend exactly that ("a separate `Exec` per thread", `docs/ARIADNE.md:1151-1152`).
    That usage is exposed to the process-global hazards below (B7, B10, B14, B23).

### 2.2 What already makes threads possible

- **Every step boundary is a safe point.** All per-step thread-locals are set and cleared by RAII
  objects inside one step:
  - the deadline, allocation, release-queue, backlog and kernel guards (`types.cpp:171`, `260`,
    `263`, `317`, `353`);
  - the data step scope and native nesting depth (`data_job.cpp:560`, `621`);
  - the step depth, the park-forbidden flag and the await slot (`stackless_evaluator.cpp:378`,
    `379`, `402`).
- **No coroutine spans a step.** The async evaluator suspends only after `inner_.step()` returns
  (`async_stackless_evaluator.cpp:40-44`), and `sync_wait` consumes the suspension on the same
  thread (`async_scheduler.cpp:229`). `perf/sched` removes even that frame.
  - So between steps a process holds no pointer tied to a thread, and neither does a whole
    scheduler between pumps. A scheduler can be pumped by thread A in one frame and by thread B
    in the next, provided the handoff has a release/acquire edge.
- **Refcounts are already atomic.** Every heap value is a `std::shared_ptr`
  (`types.h:34-35, 50-51, 59-60`). Copying or dropping a value on any thread is memory-safe, and
  `drop_owner`'s `use_count()==1` test is a heuristic that tolerates races (`types.h:88-96`).
- **The stdlib scope is frozen and shared** (`stdlib.cpp:1427-1439`). `set` throws on a frozen
  scope (`types.cpp:431-433`), and the cycle collector skips frozen scopes (`heap.cpp:35-38`).
- **Isolation already exists in embryo.** An `Exec` owns a private scheduler, env, intrinsics
  context and async host (`pycvc_exec.cpp:369-424`). Ariadne builds a fresh default env and
  intrinsics context per action (`ariadne.cpp:1646-1676`). An execution group is that pattern made
  first-class.

### 2.3 The blockers

These come from the thread-safety audit of the code map, plus three the judges found (B30-B32).
Each has an ID; section 4.18 says where the design resolves each one. The last column says when the
chosen design needs the fix. "Stage 4 only" means a group is stepped by one thread at a time, so the
blocker cannot occur until several threads step one group.

| ID | Blocker | Where | Needed at |
|---|---|---|---|
| B1 | One current process per scheduler, read by intrinsics and `async_host` | `scheduler.h:248-249`; `async_scheduler.h:263-265`; set at `async_scheduler.cpp:159-168`; read at `intrinsics.cpp:443, 479, 511-521, 583, 769, 979-980, 1035-1040` and `async_host.cpp:399, 412, 550-551` | Stage 4 only |
| B2 | Unsynchronized scheduler maps: `processes_`, `pending_messages_`, `next_pid_`, `rr_index_`, `async_calls_` | `async_scheduler.h:257-288` | Stage 4 only (hosts touch them between pumps) |
| B3 | Intrinsics write into other processes: kill, pause and resume; delivery writes into each waiter's frame; `release_if_idle` frees any process but "the current one" | `async_scheduler.cpp:858-929`, `465-489`, `875-880`; `scheduler.cpp:736-741` | Stage 4 only |
| B4 | `msg-recv` checks the pending queue, then parks: harmless on one thread, a lost message on two | `intrinsics.cpp:1167-1191` | Stage 4 only |
| B5 | The cycle collector's registry is `thread_local`; a registry is lost when its thread exits | `heap.cpp:408-411`, `564-646`, `437-467` | Stage 1 (a group changes threads) |
| B6 | Heap-share measurement walks nodes shared with other processes, between slices | `heap.cpp:263-380`, `516-554` | never (invariant G, 4.3) |
| B7 | The 13 built-in exception class dicts are process-global and mutable through `set-attr`/`del-attr` | `exceptions.cpp:55-79`, `125-131`, `158`; `builtins.cpp:585-613` | Stage 0 (races today) |
| B8 | `fork` shares the parent's frame environments, the global env's values included | `async_scheduler.cpp:957-990`; `scheduler.cpp:950-959` | Stage 4 only |
| B9 | One envelope dict is given to every receiver a `msg-send` wakes | `intrinsics.cpp:1005-1011`; `async_scheduler.cpp:468-477` | Stage 4 only |
| B10 | `state-data-set` stores the live value and refuses a callable only at top level. `make_copy_job` copies lists and dicts and shares everything else, so a closure, native, generator or future inside a list crosses through the tree | `intrinsics.cpp:231-249`; `data_job.h:113-115` | Stage 0 (races today) |
| B11 | A host's `opts.env` becomes the process's global env without `extend()`; the `intrinsics_context` behind it holds a mutable `watches` map and `proc` | `stackless_evaluator.cpp:52-53, 66`; `intrinsics.cpp:379, 396`; `intrinsics.h:139-159` | Stage 4 only (Stage 3 checks it before a move) |
| B12 | Quoted literals alias the program's AST; generators are shared values that own an evaluator | `stackless_evaluator.cpp:975-979`; `generator.h:30-45` | Stage 1 (an AST handed to another group) |
| B13 | `state-watch` points the whole scheduler's `watch_root_` at the caller's chroot, from inside a step; settings, charging and watch polling read it | `intrinsics.cpp:384`; `async_scheduler.cpp:44-57, 641-647` | Stage 0 (a bug today) |
| B14 | The state tree hands out raw node pointers; `state-delete` sweeps the whole root | `state.cpp:1334-1354`; `intrinsics.cpp:183-194`; `state.h:153-159` | Stage 0 (races today) |
| B15 | signals2 subscribers run on the writing thread | `state.h:308-331` | Stage 1 (a group off the host thread) |
| B16 | The scheduler's base builtins env is unfrozen | `async_scheduler.cpp:17-18`; `builtins.cpp:855` | Stage 1 (lightweight groups share it) |
| B17 | Non-atomic stop flags; `exec_coordinator` calls `pause`/`resume`/`kill`/`send_signal` from bus threads | `async_scheduler.h:260`; `scheduler.cpp:462, 475`; `exec_coordinator.cpp:1045-1071` | Stage 0 (races today) |
| B18 | `async_call::outcome()` changes the scheduler, so two awaiters on two threads race | `async_host.cpp:109-113` -> `async_scheduler.cpp:569-588` | Stage 0 (a future cannot leave its group) |
| B19 | Completions wake no one; hosts poll | `async_scheduler.cpp:516-556`; `pycvc_exec.cpp:648` | fixed by `perf/sched`; Stage 2 extends it |
| B20 | `park_forbidden_scope` is per thread, so it would not follow a step onto a worker | `stackless_evaluator.cpp:379, 463-467` | Stage 4 only |
| B21 | A step can block its thread: an inline kernel, or `call->wait` where the process can't park | `intrinsics.cpp:537-542`; `async_host.cpp:432-480` | Stage 2 (a blocked worker) |
| B22 | Host natives that expect the host thread: Ariadne's action intrinsic providers, cvcGL media and stream verbs | `ariadne.cpp:1660-1662`; `src/cvcGL/ariadne/media_verbs.cpp`, `stream_verbs.cpp` | Stage 1 |
| B23 | The app background pool starts a thread per task, probes keys in O(K), never erases entries, and stalls then hangs past its size | `app.cpp:594-623`, `742-753`, `780-788`, `1075-1095` | fixed by `perf/async` |
| B24 | `computePool().parallel_for` runs one fan-out at a time | `thread_pool.cpp:170` | fixed by `perf/async` |
| B25 | Refcount traffic on shared read-only objects (a shared AST, the global class dicts) bounces cache lines | `stackless_evaluator.cpp:912`; `exceptions.cpp:158` | Stage 1 (measured) |
| B26 | `math.random` is `thread_local`; `print` lines from several threads interleave | `stdlib.cpp:1279`; `builtins.cpp:688-707` | Stage 1 |
| B27 | Spawned children get the bare base env, with no intrinsics, no chroot and no owner, so they cannot `msg-send`, touch the tree or report a result | `intrinsics.cpp:420-431` -> `stackless_evaluator.cpp:53, 66`; `builtins.cpp:855-938`; intrinsics come only from `register_intrinsics` (`intrinsics.cpp:1062`) | Stage 1 |
| B28 | `(fork)` forks `ctx->pid`, which hosts leave at -1 or 0, not the running process; a forked child does not inherit `owner` | `intrinsics.cpp:437`; `ariadne.cpp:1645-1660`; `pycvc_exec.cpp:416`; `async_scheduler.cpp:957-976` | Stage 0 (a bug today) |
| B29 | With several documents Ariadne pumps the shared scheduler once per document per frame, so `wake_awaiting` runs D times a frame and "an await crosses exactly one frame" does not hold | `ariadne.cpp:2021-2023`, `1694-1697` | Stage 1 |
| B30 | Host posting and async-function sites are bound to the app scheduler and host, not to the scheduler that runs the program | `app_runtime.cpp:96-101`; `net_intrinsics.cpp:268-276`; `stream/stream.cpp:132-148, 238`; `ariadne.cpp:1843, 1967` | Stage 1 |
| B31 | Envs capture a concrete scheduler and host: `ictx->sched` is a raw pointer (`intrinsics.h:140`), and `register_async_fn`'s natives capture `&host` (`async_host.cpp:589-591`). A process moved to another scheduler would call the old one from another thread | as listed | Stage 3 (moving a process) |
| B32 | Pids are per scheduler (`next_pid_`, `async_scheduler.h:257`), so several schedulers reuse pids; `restore(process_ptr)` re-pids a process and clears its receive state | `async_scheduler.cpp:652-664` | Stage 1 (pids), Stage 3 (moves) |

B27 to B29 are not threading bugs, but every model needs them fixed. Without B27, a DSL program
cannot start useful parallel work in any model, because a spawned child can neither message nor
report. Today the only parallelism a host can get is from programs it submits itself.

The table shows why groups are cheaper than one scheduler on N threads. Eight of the 32 blockers
(B1-B4, B8, B9, B11, B20) are "Stage 4 only": they cannot occur while one thread at a time steps a
scheduler. They are also the ones whose fixes would put new concurrent code in the step path.

### 2.4 What the single-thread tracks already change

These branches are in progress. The stages below build on them and do not repeat their work.

| Branch | What it does | Blockers it settles |
|---|---|---|
| `perf/sched` | A process table with a run queue, a timer heap and waiter indexes, in both schedulers. One clock reading a step. One ingress drain a step, an atomic flag when empty. The evaluator step called directly, with no coroutine frames. `wait_for_work(deadline)`, woken by posts, completions and timers. Watch polling only when the tree changed (an atomic counter fed by the root's change signals). pycvc's `Exec` waits for work instead of sleeping 1 ms | B19; the O(P log P) selection; the per-step ingress mutex |
| `perf/async` | Persistent background-pool workers, with anonymous tasks and no thread per task. A fix for the stall and hang. Concurrent `parallel_for` fan-outs side by side. Less per-call work in `async_host` | B23, B24 |
| `perf/eval` | Cheaper steps: special forms by switch, a closure body built once, call scopes recycled per thread, one hash per lookup | none; a faster step makes per-pump costs relatively larger |

`perf/eval` recycles call scopes in a per-thread cache. A scope is returned to the cache of the
thread that drops it, and is reused only by that thread. Under groups, a scope is dropped only by
the thread pumping its group, so a group that changes threads just fills a different thread's cache.
Nothing is shared.

### 2.5 Measured costs

| Measure | Value | Scenario |
|---|---|---|
| A bare stackless step | 86 ns (`arith_loop`), 104 ns (`fib`) | `eval.*` stackless |
| A scheduled step, sync | 249 ns (`arith_loop`), 263 ns (`fib`) | `sched.single.*` |
| A scheduled step, async | 456 ns (`arith_loop`), 470 ns (`fib`) | `sched.single.*` |
| Step cost against processes held | sync 234 ns at p=1, 21.8 us at p=1000; async 420 ns, 24.0 us | `sched.scaling` |
| Message round trip, no bystanders | sync 85 us, async 95 us (`full5`); 23-64 us (`full6`) | `msg.pingpong` |
| Message round trip, 1000 parked bystanders | sync 1.94 ms, async 1.99 ms | `msg.pingpong` |
| Thread-safe ingress | about 46k messages/s from 1 posting thread, 61k from 4 | `msg.ingress` |
| An unrelated step with 8 calls in flight | about 25 us with the tree read of the rate, about 1 us without | `async.inflight_tax` |

- **Profile.** The first sampled profile of a sync step shows about 20% of the time in clock reads
  and about 21% in `malloc`.
- **Scheduler overhead.** The scheduler costs 2.5x (sync) to 4.5x (async) a bare step. `perf/sched`
  removes most of the async excess: the per-step selection, the per-step ingress mutex, the two
  coroutine frames and two of the clock reads. Its own measurements will replace these anchors.

**Independent schedulers, one per thread** (`mt.independent`). Each thread has one stackless
evaluator and default env, and nothing is shared. This is the shape of the chosen design, and the
ceiling for any design on this machine.

| Threads | fib, AST per thread | fib, one shared AST | raise/catch, per thread | raise/catch, shared |
|---|---|---|---|---|
| 2 | 1.39x | 1.42x | 1.91x | 1.80x |
| 4 | 2.64x | 2.63x | 3.16x | 2.67x |
| 8 | 4.45x | 3.76x | 4.46x | 5.13x |

- **Per-thread slowdown.** At 8 threads one fib thread ran at 3857 ns/iter against 2379 alone, 1.62x
  slower. The causes are E-cores (two of the eight threads ended on a lower efficiency class), the
  lower all-core clock, and the allocator (16.5 allocations per fib iteration).
- **A shared AST costs about 15%** at 8 threads, because its control blocks' refcounts ping-pong
  between cores. That is why groups parse per process.
- **The raise/catch rows** show no class-dict contention signal: shared is 5.13x against private
  4.46x. But the single-thread reference for that program ranged from 16.9 to 48.0 us/iter within
  the run, so one rep cannot rule contention in or out.

So about 4.5x at 8 threads is this machine's ceiling, whatever the scheduler does. A homogeneous
many-core server should do better.

## 3. The options and the decision

### 3.1 The options

Five proposals were written, each from one angle. Two more options frame them.

**A. One scheduler, N workers, over heap domains** (`proposal-shared-scheduler.md`).
- **Structure:**
  - One `async_scheduler` with N worker threads; the host thread is worker 0.
  - The unit of scheduling is a *heap domain*: the processes allowed to share mutable values (a
    fork family, the processes of one host env). By default each spawned or submitted process is
    its own domain.
  - A worker claims a domain with a CAS and runs a slice of it. Cross-domain kill, pause, signal and
    delivery become requests on the target domain's MPSC inbox. Pid, channel and timer tables are
    sharded.
  - The collector runs per domain at the owner's slice boundary, so there is no stop-the-world.
- **For:**
  - It has one pid space, one channel table and one host API, and balances dynamically.
  - Its race handling is the most explicit of the five: seq-numbered waiters with a re-send rule, an
    atomic pop-or-register receive, a kill flag checked every step, release only by the owner.
  - It is the only proposal that noticed spawned children have no intrinsics (B27).
- **Against:**
  - About 2-3k lines of new concurrent engine sit at the core, with a high-risk phase; MSVC has no
    TSan.
  - Its release path ("CAS running -> idle, or -> queued if the inbox is non-empty") can lose a
    wakeup without a post-release re-check.
  - By default it gives Ariadne and pycvc 1.0x, and two scheduler code paths live side by side.
- **Judges' corrections:**
  - `state-data-set` refuses only a top-level callable (B10), so generators do not "stay in a
    domain".
  - Its `msg.pingpong` figures came from `full6`, not `full5`.
  - The root-mutex claim holds only for unchrooted contexts: `apply_chroot` points Ariadne walks
    at the chroot node (`intrinsics.cpp:1134`).

**B. Sharded executors: one `async_scheduler` per thread, with handoff**
(`proposal-per-thread-schedulers.md`).
- **Structure:**
  - N unchanged schedulers, one per pinned thread.
  - Groups (a document, a fork family, an env family, a spawned process) are placed on a shard.
  - Every cross-shard effect is a command through the ingress.
  - A `shard_router : scheduler_base` resolves the current shard through a thread-local, so envs do
    not capture a concrete scheduler.
  - Pids encode their origin shard. Channels have home shards, with a subscribe, deliver and
    requeue protocol.
  - Group migration and stealing of ready singleton processes come late.
- **For:**
  - N=1 is exactly today, and the step path takes no lock.
  - It found two real defects: with several documents every `Runtime::drain` pumps the shared
    scheduler (B29), and `state-watch` redirects the scheduler's watch root (B13).
  - It is the only proposal that spells out thread-local resolution of the scheduler and host
    (B31).
- **Against:**
  - The whole cross-shard protocol must land before any host benefits: 5-7 weeks at high risk.
  - Shards are pinned until a late phase.
  - Its idle loop publishes idle and then waits without re-checking, which can lose a wakeup.
  - A migrated parked process keeps its remote subscription at the old shard, so it hangs until its
    timeout.
- **Judges' corrections:** spawned children "safe to place anywhere" cannot message (B27), and
  `make_copy_job` does not refuse nested closures (B10).

**C. BEAM-style SMP: per-worker run queues, isolated per-process heaps** (`proposal-beam-style.md`).
- **Structure:**
  - One logical scheduler with per-worker run queues, stealing and wake-on-waker.
  - Slices of K steps play the part of BEAM's reductions.
  - Every mutable object is reachable from one process only. fork deep-copies the reachable heap,
    and `state-data-set` copies closures with their environments.
  - Cross-process operations are signals, and each process has its own cycle registry.
- **For:**
  - It has the finest balancing and the largest single-thread win: slices with one clock read per
    16 steps.
  - It contributes good mechanisms: adaptive helper wake, separate dirty CPU and I/O pools, a
    long-schedule monitor, and `on_complete` on the host thread.
  - It is the only proposal whose copy walk closes the nested-callable hole (B10).
- **Against:**
  - Its isolation changes are always on, even at one worker: fork becomes an O(heap) deep copy, and
    shared host envs are refused at submit.
  - That buys little, because fork already fails in both real hosts (B28).
  - Forked children made by `clone_state` still share one `intrinsics_context`, whose `watches` map
    and `next_watch_id` they mutate (`intrinsics.cpp:379, 396`), so the isolation is incomplete.
  - Its receive is not specified as atomic, and nothing prevents a process being enqueued twice.
  - It has the largest new core of the five.
- **Judges' corrections:** "workers=1 is exactly today" contradicts its own always-on changes, and
  intrinsic natives are not immutable (`msg-send` updates `ctx->proc->message_count`,
  `intrinsics.cpp:986-989`).

**D. Share-nothing isolates ("places")** (`proposal-isolates.md`).
- **Structure:**
  - Several complete runtimes, one per worker. Each isolate has a scheduler, env, async host,
    registry, RNG and control ingress.
  - Isolate 0 is today's scheduler, driven as today.
  - Work moves to a worker isolate only when asked: `spawn-isolate`, `execute_options.isolate`, or
    an auto-spawn setting.
  - Values crossing isolates are structured-cloned: closures, natives and generators are refused.
  - A group facade gives global pids and routes control and channels: a home isolate per channel,
    remote subscriptions, a bounce when a waiter is gone.
  - M:N isolates with stealing come late.
- **For:**
  - It has the strongest default compatibility: nothing moves unless asked.
  - It is the only proposal that states the wake protocol (set the flag, signal, re-check before
    waiting) and bounds the ingress with a sender-visible error.
  - Host natives are default-deny on workers (a NameError, never a race).
  - `kill_owner` is broadcast and acknowledged before Ariadne frees the contexts the intrinsics
    point at.
  - A deterministic mode runs every isolate on one thread with seeded RNGs.
- **Against:**
  - Nothing parallelizes by default.
  - Balancing is per isolate only; a process moves only by a codec transfer, and the codec omits the
    inbox, watches and pending signals (`state_value_codec.h:66-71`).
  - The cross-isolate channel protocol is new distributed code.
- **Judges' corrections:**
  - Pids `(isolate << 22) | local` collide: `next_pid_` is unbounded, so isolate 0 reaches isolate
    1's range after 4,194,303 processes, about 19 hours at one Ariadne action per frame at 60 fps.
  - Plain `spawn` is "the natural unit to place" only once spawned children have intrinsics (B27).
  - Its 1-2 us in-isolate message estimate contradicts the measured 23-95 us round trip, so a
    cross-isolate hop is a small fraction of today's cost, not a multiple of it.

**E. Groups first: isolated schedulers on a shared worker pool, grown in stages**
(`proposal-incremental.md`).
- **Structure:**
  - The unit is an *execution group*: one `async_scheduler` with its own async host, env and cycle
    registry, stepped by one thread at a time under a pump token. It belongs to one document, one
    `Exec`, one coordinator job or one isolated spawn.
  - Groups run on a shared pool in stages:
    - Stage 0 fixes the shared hazards, which also makes today's multi-`Exec` use safe.
    - Stage 1 is a fork-join pool pumping groups once per frame or batch, with the cycle registry
      moved from the thread to the scheduler.
    - Stage 2 is continuous pumping with wakeups, plus a router for cross-group messages.
    - Stage 3 adds isolated spawns and process moves between groups.
    - Stage 4, maybe never, is option A inside one group.
- **For:**
  - It adds the fewest new concurrent protocols. The current-process, map, kill, deliver and
    `release_if_idle` blockers are avoided by construction rather than rewritten.
  - It has the strongest test oracle: each group must match its solo run bit for bit.
  - Its first stage fixes races that exist today.
  - Its scaling estimate is the best supported, because `mt.independent` measured exactly this
    shape.
  - Every stage ships alone, with default behaviour unchanged.
- **Against:**
  - The user's goal of programs scheduled in one DSL scheduler running in parallel arrives late
    (Stage 3).
  - Its Stage 3 move keeps envs whose intrinsics and async natives point at the old group (B31),
    and `restore` re-pids the process (B32).
  - Its router forwards only when there is no local receiver, which weakens broadcast. A channel
    with no registered owner stays local, so a send to an app-global channel can miss its receiver.
  - Programs lose cross-document `ps` and `kill`.
- **Judges' corrections:**
  - Its isolated spawn "talks to the parent by channel", but a default env has no `msg-send` (B27).
  - Its `state-data-set` copy does not enforce its own invariant G (B10).

**F. Shared mutable values with per-object locks** (CPython's PEP 703 route). Not written up as a
proposal.
- Every list, dict and environment would get a lock or a lock-free read path, and the collector a
  stop-the-world safepoint.
- PEP 703 measured 5-6% single-thread overhead after years of engineering. Here a step is
  86-104 ns and touches environments on every symbol lookup, so the overhead would be larger. That
  fails goal 4.
- It also keeps every existing race between programs.

**G. Independent schedulers only** (the status quo plus fixes).
- Hosts that want parallelism run one scheduler per thread, as pycvc already allows.
- This is `mt.independent`, and it needs only Stage 0.
- It gives Ariadne documents and DSL programs nothing.
- Under the chosen design it stays available, and Stage 0 makes it safe.

### 3.2 Scores

Each judge scored all five proposals 1-10 on five axes and checked their claims against the source.
The axes are correctness and safety, performance and scaling, cost and risk (higher is cheaper and
safer), compatibility, and testability. Each cell gives the total, then the five axis scores in
that order.

| Proposal | Judge 1 | Judge 2 | Judge 3 | Sum | Ranked first by |
|---|---|---|---|---|---|
| E. groups first (incremental) | 39 (8/6/8/8/9) | 39 (8/6/8/8/9) | 41 (8/6/9/9/9) | **119** | all three |
| D. isolates | 35 (7/6/6/8/8) | 35 (8/5/6/8/8) | 37 (8/6/7/8/8) | 107 | none |
| B. sharded per-thread schedulers | 32 (6/7/5/6/8) | 35 (7/7/5/8/8) | 33 (7/7/5/6/8) | 100 | none |
| A. shared scheduler over heap domains | 33 (7/8/4/7/7) | 32 (6/8/4/7/7) | 30 (6/7/4/6/7) | 95 | none |
| C. BEAM-style isolation | 28 (5/9/3/4/7) | 27 (5/8/3/4/7) | 28 (6/8/3/4/7) | 83 | none |

Judge 3's list of ideas to graft was cut short in the copy this synthesis received, after its first
item. Its scores, strengths, weaknesses and factual errors for all five arrived complete. Its first
graft agrees with judges 1 and 2: a structured clone at every crossing.

### 3.3 The decision: E, groups first

The verdict is unanimous: all three judges ranked E first. For every judge it scored highest, or
tied for highest, on every axis except performance.

**Why E.**
1. **It avoids the hardest blockers instead of rewriting them.** A group is today's scheduler,
   stepped by one thread at a time. B1-B4, B8, B9, B11 and B20 cannot occur. The only new
   concurrent code before Stage 4 is:
   - a pump token and a group state word (section 4.4);
   - a fork-join pool and, later, a ready queue of groups;
   - a router that posts into the existing thread-safe ingress.
2. **Its scaling claim rests on a measurement, not a model.** `mt.independent` is groups with
   nothing between them, and it reached 4.45x at 8 threads.
3. **The realistic gains are host-level anyway.**
   - Spawned children cannot message or report in any model until B27 is fixed.
   - An `Exec`'s programs share `impl_->env` (`pycvc_exec.cpp:565`), so one `Exec` is one domain
     in every model.
   - Every model defaults Ariadne to host affinity.
   - What parallelizes soonest is independent host-submitted work: several `Exec`s, batches,
     several documents, coordinator jobs. E delivers that in Stage 1, after about 5-6 weeks
     including Stage 0. Option A needs about 8-11 weeks of high-risk engine work first.
4. **It has the best oracle.** Each group must match its solo single-thread run (results, step
   counts, exit status, tracked objects), at any worker count and under forced worker changes. No
   other model can state its correctness that simply.
5. **The finer models stay reachable.** The proposals converge on one structure: a domain of
   processes that share data, never touched by two threads at once.
   - E's Stage 2 pool of groups is A's slice-claimed domains, with the domain promoted to a full
     scheduler.
   - B in deterministic mode is E with fixed placement.
   - D's M:N phase is E's Stage 2 with explicit placement.
   - The real differences are whether a domain is a full scheduler or a lightweight sub-scheduler,
     and how messages cross. E answers "a full scheduler, through the existing ingress" first, and
     keeps A's design in reserve as Stage 4 (section 4.16) for single groups that turn out to be
     DSL-bound.

**What E gives up, and how this design narrows it.**
- **DSL programs get parallelism in Stage 3.** It comes through `spawn-isolated`, not through plain
  `spawn` or `fork` inside one scheduler. Stage 1 already fixes B27, so in-group spawned children
  are useful, and Stage 3 places them on other groups.
- **One document or one `Exec` program gets 1.0x** until it uses `spawn-isolated`, or Ariadne's
  opt-in parallel actions (Stage 3), or Stage 4. The same is true of every option by default.
- **Cross-group messages pay an ingress hop.** It costs about 0.5-1 us plus a wake when the target
  is idle, against today's measured 23-95 us in-scheduler round trip. Chatty processes belong in
  one group.

**Why the first draft chose A, and why that no longer holds.** The first draft compared A, B and C
only, where A and B tied. With D and E judged, every judge placed A fourth or lower, below two
models that avoid A's new engine.
- The single-thread work in `perf/sched` also removes much of what made A attractive. A's per-step
  savings were no coroutine frames, no per-step ingress lock, no per-step scans and fewer clock
  reads, and `perf/sched` delivers them to every scheduler.
- What A uniquely adds is parallelism inside one group, at the cost of a new concurrent core. That
  is Stage 4.

### 3.4 Grafts from the other proposals

| From | Idea | Where |
|---|---|---|
| A (and all judges) | Decide the spawn environment now: a spawned child gets an intrinsics context and the parent's chroot, owner and channel policy, through a host callback | Stage 1, 4.15 |
| C (all judges) | An isolating copy at every crossing between groups. Lists and dicts are copied; closures, natives, generators, environments and futures are refused at any depth (D's structured clone) rather than deep-copied (C's choice; open question Q3) | Stage 0 for `state-data-set`; Stage 1-3 for posts and arguments; 4.3 |
| B | The current group resolved through a thread-local, not through captured pointers: `intrinsics_context::sched` may point at a group-relative forwarder, and async natives resolve their host the same way | Stage 3, 4.15 |
| B | One pump per frame across every Runtime: `drain_begin`, one app pump, `drain_end` | Stage 1, 4.12 |
| B | A per-watch root instead of `set_watch_root` from a step | Stage 0, 4.8 |
| B | Lightweight groups: one process-wide frozen builtins scope that each group extends, instead of a default env of about 150 allocations per group | Stage 1, 4.2 |
| D | Default-deny host natives in groups pumped off the host thread: a NameError, never a silent race | Stage 1, 4.9 |
| D | The wake protocol stated explicitly (set the flag, signal, re-check before waiting); bounded cross-group ingress with a sender-visible error | Stage 2, 4.4, 4.6 |
| D | `kill_owner` acknowledged by every group before Ariadne frees the contexts its intrinsics point at | Stage 2, 4.5 |
| D | Deterministic mode: every group on one thread in a fixed rotation, with seeded random streams per group | Stage 1, 4.4 |
| D | A pinned handle for the chroot node in the intrinsics context | Stage 0, 4.8 |
| A | Seq-numbered remote waiters, and a re-send to the channel's home when a delivery finds its waiter gone | Stage 2, 4.6 |
| A | `finish_async_call` idempotent (an atomic `closed`); `exec_coordinator`'s control through a queue | Stage 0, 4.9 |
| A | Stage 4's design, should it ever be built: step context, domain inboxes, kill flag, per-domain registries collected at the owner's slice boundary with no stop-the-world, and the notified bit on release that fixes A's lost wakeup | 4.16 |
| C | Slices: guards, thread-local setup and a timestamp once per run of K steps, a clock read every 16 steps. Built single-threaded, as its own stage, after `perf/sched` | Stage S, 4.4 |
| C | Adaptive helper wake: small frames pump inline on the host thread and never pay a cross-thread wake | Stage 1, 4.4 |
| C | Separate CPU and blocking kernel pools, sized together with the group workers; a long-step monitor | Stage 2, 4.9 |
| C | `on_complete` callbacks of groups pumped off the host thread run on the host thread after the join | Stage 1, 4.9 |
| C | An explicit 8 MiB stack for worker threads; line-atomic `print` | Stage 1, 4.1, 4.11 |
| new | One pid source per pool, so pids stay unique across groups and a moved process keeps its pid (fixes D's pid collision and E's remap table) | Stage 1, 4.5 |
| new | A per-frame wall deadline across all groups, so a frame stays bounded when there are more groups than threads | Stage 1, 4.10 |

**Considered and not taken.**
- **C's always-on deep-copying fork and env-owner refusal.** Under groups, fork families and shared
  envs stay inside one group and need no change. `(fork)` is fixed as a bug instead (B28).
- **D's pid encoding.** It collides (section 3.1); one pid source per pool replaces it.
- **B's hash-homed app-global channels.** Hashing makes every app-global channel a remote one for
  most groups. This design homes them at the pool's group 0 instead (section 4.6).
- **C's deferred notification sink**, which queues signals2 emissions from workers and replays them
  on the host thread:
  - It delays a group's own watch events by a pump, which breaks the per-group oracle.
  - It changes `cvc::state` core.
  - It reorders notifications against host writes.
  - Host affinity plus an audit of subscribers is used instead (4.8), and the question stays open
    (Q7).
- **Per-receiver envelopes and fork deep copies always on** (A, C, D). Within a group today's
  sharing is legal, so these wait for Stage 4, where several threads step one group.

## 4. The design

### 4.1 Threading model

```
host thread    pumps group 0 (the app scheduler, or an Exec's) and every host-affinity group;
               takes part in each fork-join pump as worker 0
worker 1..N    persistent threads of app::exec_pool(); each pumps one group at a time
kernel pools   the app's background pool and lanes (perf/async), shared by every group
```

**An execution group** (`exec_group`, Stage 1) is:
- one `async_scheduler`;
- its own async host, built on first use;
- its own cycle registry and `math.random` stream;
- its options: a name, an owner tag, an affinity (`any` or `host`), a watch root.

**The pump token.** A group is pumped by at most one thread at a time.
- `pump()` is exactly today's driver sequence, as Ariadne (`ariadne.cpp:1688-1706`) and `Exec`
  (`pycvc_exec.cpp:587-594`) run it: `drain_ingress`, `wake_awaiting`, `sync_run(steps, seconds)`.
- The token is an atomic owner field in the group's state word (section 4.4), taken with an acquire
  and released with a release. That handoff is the release/acquire edge a scheduler needs when it
  changes threads (the audit's model (c)).
- Debug builds assert the token in the scheduler's public mutators.

**The pump scope.** For the duration of a pump the pumping thread binds three thread-locals:
- `exec_group::current()`;
- the group's cycle registry (section 4.7);
- the group's random stream (section 4.10).

That is three stores and three restores per pump, and nothing per step.

**The pool** (`exec_pool`, owned by `app::exec_pool()` and built lazily like `exec_scheduler()`):
- **Size.** It has `state_exec.threads` persistent `std::thread` workers.
  - 0 (the default) means every pump runs inline on the calling thread, which is today's
    behaviour and the only mode in single-threaded wasm.
  - `auto` means `hardware_concurrency() - 1`, minus the low-power E-cores where the OS reports
    them.
- **Stack.** Workers are created with an explicit 8 MiB stack. The native-stack guard computes its
  bound per thread (`native_stack.cpp:64-71`), and deep natives need the room `kNativeStackReserve`
  assumes (`data_job.cpp:834-840`).
- **Placement.** On hybrid parts workers go on performance cores first: CPU sets on Windows,
  affinity on Linux, as the bench pins its driver.
- **Lifetime.** Workers never start or stop per task.

**Two pool modes.**
- **Fork-join** (Stage 1; Ariadne's mode and `ExecPool`'s):
  - `pump_all(groups, budget)` pumps each group once, in parallel. The caller pumps the
    host-affinity groups itself and helps with the rest. It returns when all are done.
  - Outside a `pump_all` no worker touches any group. Host code (sweeps, `get_process_info`,
    `reap`, `kill_owner`) runs as today, without the token.
  - Render never overlaps a pump, so it stays a pure reader (`docs/ARIADNE.md` section 3).
- **Continuous** (Stage 2; pycvc servers, `exec_coordinator`, headless hosts):
  - Attached groups are pumped whenever they have work, a slice at a time (section 4.4).
  - A host call that touches a group's scheduler takes the token (`with_group`), or goes through a
    thread-safe post (`post_message`, `post_control`).

### 4.2 Groups

**Who makes a group.**

| Group | Made by | Affinity |
|---|---|---|
| Group 0 | `app::exec_scheduler()` itself; for pycvc, each `Exec`'s private scheduler | host |
| A document group | an Ariadne `Runtime` with `ariadne.exec.isolate_documents` on (Stage 1) | `any`, or `host` if the document's env holds a host-only native (4.9) |
| An `ExecPool` group | `pycvc.ExecPool` (Stage 1): one per pooled `Exec` | `any` |
| A job group | `exec_coordinator::submit_local` with a pool attached (Stage 2) | `any` |
| A spawn group | the pool, for `spawn-isolated` children (Stage 3); up to `threads` per owner, made lazily | `any` |
| An action group | an Ariadne document with `ariadne.exec.parallel_actions: k` (Stage 3) | `any` |

**What a group shares with other groups.** Only frozen or immutable data:
- the frozen stdlib scope;
- a process-wide frozen builtins scope (below);
- the 13 built-in exception class dicts, made read-only in Stage 0;
- strings, numbers and bytes, which the variant holds by value (`types.h:154-155`);
- read-only `data_object` payloads (`docs/STATE_EXEC_EXCEPTIONS.md` 3.7);
- the state tree, through its own locks (section 4.8);
- the kernel pools.

**Lightweight groups.** A group's scheduler would normally build its own `make_default_environment()`,
about 150 allocations (`builtins.cpp:855-938`; `flood.small env=own`).
- Stage 1 builds the builtins scope once per process and freezes it (B16).
- Each group's base env is `extend()` of it. The natives in it are stateless free functions
  (`builtins.cpp:851-912`).
- `async_scheduler` gains a constructor option for that base env. Hosts keep registering their own
  natives into their own per-process or per-group envs, as Ariadne and pycvc already do.

**Teardown.** Destroying a group:
1. takes the token, waiting for a pump in progress;
2. kills its processes;
3. shuts its async host down, waiting for kernels in flight, as `~Exec` does
   (`pycvc_exec.cpp:425-449`);
4. collects and frees its registry.

### 4.3 Ownership and values

**Invariant G.** A list, dict, unfrozen environment, closure, generator, async-call future or
deep-data job's held value is reachable from the processes of one group only.
- Anything reachable from two groups is frozen, immutable by type, or a read-only `data_object`.
- Inside a group, sharing is exactly today's: fork's shared frames, the aliased broadcast
  envelope, a host env used as the global env, generators. One thread at a time steps the group,
  so this is safe.

**The isolating copy.** `make_copy_job` is not an isolation boundary: it copies lists and dicts and
shares everything else (`data_job.h:113-115`). Stage 0 adds an isolating mode,
`make_copy_job(v, copy_mode::isolate)`. It is D's structured clone:
- strings, bytes, numbers, symbols and nil are values, so they are copied as values;
- lists and dicts are deep-copied, keeping their sharing and cycles, as today's copy does;
- a `data_object` is shared, under its read-only contract;
- a closure, `native_fn`, generator, environment or async-call future anywhere in the value raises
  `runtime_error`, naming the crossing and the kind (for example "state-data-set: cannot store a
  closure (nested in a list)"). This extends the message `state-data-set` uses for a top-level
  callable today (`intrinsics.cpp:236-239`).
- The copy runs as a sliced data job, charged to the process that makes it, as async arguments are
  today (3.7).

**Where values cross a group boundary, and what crosses.**

| Crossing | What happens | Stage |
|---|---|---|
| `state-data-set` | stores an isolating copy; `state-data-get` keeps its copy on the way out (`intrinsics.cpp:212-213`) | 0 |
| A host `post_message` to another group | the host hands the value over (moved in); the router makes an isolating copy for each extra receiving group | 1-2 |
| A cross-group `msg-send` | the payload is a string or bytes (`as_string_or_bytes`, `intrinsics.cpp:966`); each receiving group gets a fresh 4-entry envelope | 2 |
| `execute(value_t)` into a group | the AST is copied when `use_count() > 1`, because quoted literals alias it (`stackless_evaluator.cpp:975-979`); scripts are parsed inside their process already | 1 |
| `ExecPool.run_all` inputs and results | strings in; results copied out through Python | 1 |
| `spawn-isolated` arguments | isolating copy | 3 |
| Async arguments and results | already copied (3.7) | today |
| A process moved between groups | allowed only when an isolation walk proves it shares nothing (4.15) | 3 |

**Why refusal and not copying closures.** C copies a closure with its environment. That copy has to
decide what to do with an environment chain that reaches the group's global env or a host env, and
with the natives inside, which capture an `intrinsics_context` (B11, B31).
- The existing refusal's reasoning applies at every depth: a stored callable can outlive what it
  captures, and it is an invocation channel around an allowlist (`intrinsics.cpp:231-235`).
- A nested closure in stored data was a hole, not a feature.
- Copying closures stays open (Q3).

**Debug enforcement** (`CVC_EXEC_VERIFY=1`):
- every compound value posted across a boundary is tagged with its source group, and a second
  posting of the same object asserts;
- a move (Stage 3) runs an isolation walk first: the collector's census restricted to the process's
  roots, where every non-frozen node's `use_count()` must equal its references from inside the
  process plus its roots;
- in TSan CI, the walk runs for every group every 1000th pump.

### 4.4 Scheduling and balancing

**Inside a group.** Today's policy, on `perf/sched`'s run queue. The round-robin and priority
orders are unchanged.

**Stage S: slices.** Optional, single-threaded, after `perf/sched`.
- A *run* is up to `slice_steps` consecutive steps of one process. These are BEAM's reductions.
- **Once per run:**
  - the guards and the step's thread-locals;
  - one clock read;
  - one `eval_deadline_guard`, at the earlier of the process's time left and the pump deadline;
  - one `eval_alloc_guard`;
  - one `thread_cpu_seconds()` read if the process charges CPU time.
- **Once per step:** `step()`, `++steps` with an exact `max_steps` compare, a status check, and a
  clock read every 16th step.
- A process with `max_step_time` (Ariadne residents, `ariadne.cpp:1792`) keeps its guard per step.
- `slice_steps = 1` is today's round robin exactly, and stays the default until the user decides
  (Q10).
- Slices are a single-thread gain for every group, independent of the pool. After `perf/sched` they
  remove what is left of the per-step overhead: guard arming, thread-local setup and most clock
  reads.

**Stage 1: fork-join across groups.**
- **Order.** `pump_all` orders groups longest-last-pump first (LPT), which approximates the
  shortest makespan.
- **Execution.** Workers and the caller take groups from a mutex-guarded list. The caller takes
  host-affinity groups first. The list mutex is the handoff edge.
- **Each group keeps its own pump budget** (section 4.10). The frame's DSL time becomes the maximum
  over workers instead of the sum.
- **Adaptive helper wake** (C's graft). Helpers are woken only when they would pay for themselves:
  - Waking a parked thread costs about 5-20 us on Windows.
  - If the groups' last pumps took less than `inline_below` in total (default 100 us), the caller
    pumps them all inline and no helper is woken.
  - Otherwise the caller wakes one helper per group beyond the first, up to the pool size. Each
    woken helper wakes the next only if work remains.
  - A typical Ariadne frame (one or two short actions and a few parked residents) therefore never
    wakes a helper.

**Stage 2: continuous pumping.** Groups are `attach`ed to the pool.
- **The group state word** follows the Akka and CAF mailbox pattern:
  - `SCHEDULED`: in the ready queue or being pumped;
  - `NOTIFIED`: work arrived since the pump began;
  - `PUMPING`: holds the owner thread id.
- **`wake(g)`**, called after any post to `g`'s ingress, a completion, or a due timer:
  - `old = g.state.fetch_or(SCHEDULED | NOTIFIED)`.
  - If `old` lacked `SCHEDULED`, push `g` onto the ready queue and notify one parked worker.
  - Exactly one caller sees that transition, so a group is never queued twice.
- **A worker that pops `g`:**
  1. clears `NOTIFIED`;
  2. takes the token;
  3. pumps one slice (default 2 ms or 20000 steps);
  4. releases the token;
  5. if `g.wants_pump()` is true, pushes `g` back. Otherwise it CASes `SCHEDULED -> 0`, expecting no
     `NOTIFIED`. If the CAS fails because a post set `NOTIFIED` meanwhile, it pushes `g` back.
  - A post that races the release is therefore never lost. This is the re-check that A's release
    and B's idle loop lacked.
- **`wants_pump()`** is true when the group has a runnable process, its ingress is non-empty (the
  atomic flag from `perf/sched`), or its earliest timer is due.
- **The ready queue.** One mutex-guarded queue, ordered by each group's virtual time: pump time
  consumed, weighted by an optional share. A CPU-heavy group then cannot starve a latency-sensitive
  one.
  - The lock is taken about once per slice per worker (2 ms), so per-worker deques with stealing
    are not needed at group granularity. They are a later option if `mt.groups.flood` shows
    contention.
- **Locality.** A group woken by a cross-group message from worker W goes into W's one-entry
  `next` slot, Go's `runnext`. A request-and-reply pair of groups therefore stays on one core when
  W is about to go idle. The slot is bypassed after 3 consecutive uses (Tokio's fairness valve).
- **Parking.** Idle workers spin about 20 us, then park on the pool's eventcount
  (`WaitOnAddress` on Windows, a futex on Linux, a condition variable elsewhere). The parking
  timeout is the earliest timer among attached groups.
  - Wakes use the eventcount's prepare, re-check, commit protocol: a worker announces it is about to
    sleep, re-checks the queue, then sleeps. A wake between its last look and its sleep is never
    lost (D's graft).
- **Timers.** Each group keeps `perf/sched`'s timer heap. The pool keeps a small heap of
  (deadline, group) for the earliest timer of each idle group. A worker that finds one due wakes
  that group.
- **Host-affinity groups** are pumped only by the host thread, in `pump_host()` or while it waits
  in `wait_for_work`.
- **Group moves.** A group pumped by worker 1 now and by worker 3 next is a move. It costs only the
  cache misses of its state, and nothing has to be re-registered, because nothing in a group is tied
  to a thread (section 2.2). Groups move freely in Stage 2; processes move between groups only in
  Stage 3.

**Deterministic mode** (`state_exec.deterministic`, or `CVC_EXEC_DETERMINISTIC=1`):
- every group is pumped on one thread in a fixed rotation by group creation order;
- slices are fixed in steps, not time;
- each group's random stream is seeded from the group's name;
- cross-group posts are applied in post order.

The whole multi-group schedule is then reproducible, with the same cross-group semantics as the
threaded modes. The parallel test suites run in this mode first (section 8).

**Hybrid CPUs.** Workers prefer P-cores.
- In fork-join mode the frame waits for its slowest worker, so the heaviest groups by last pump
  time go first, to P-core workers.
- Whether E-cores take part at all is a setting, `state_exec.cpu_set`: `performance` or `all`
  (Q8).

### 4.5 Cross-process operations

**Inside a group:** unchanged. Kill, pause, resume, signal, delivery, `ps`, `inspect` and fork
work as today (`async_scheduler.cpp:858-929, 945-998`).

**Pids.** The pool owns one `std::atomic<int>` pid source. Every group's scheduler takes its pids
from it (`async_scheduler::set_pid_source`, Stage 1).
- Pids stay unique across the whole app, as they are today with one app scheduler. `ps` output and
  host logs keep meaning what they mean.
- A process moved in Stage 3 keeps its pid (B32).
- The cost is one atomic increment per process created.
- A directory maps a pid to its group: 64 shards, each a mutex and a map. It is written at create,
  move and reap, and read by host and cross-group calls. Ariadne creates about one process per
  action, so the writes are negligible.

**From a host or another thread.** `pool.post_control(pid, cmd)` covers `kill`, `pause`, `resume`
and `signal`.
- The command goes into the target group's ingress as a new entry kind, applied in
  `drain_ingress` between steps.
- It returns true when the pid was live in the directory.
- It never blocks.
- `exec_coordinator`'s bus-thread control uses it (Stage 0 adds the same for the sync scheduler).

**From a program to another group** (Stage 2):
- `kill`, `pause`, `resume` and `send-signal` on a pid in another group are posted as above. The
  call returns `#t` for "requested", and the effect lands at the target's next step boundary.
- This is allowed only when the target's group has the caller's owner: the same document or the
  same `Exec`. Otherwise the pid is treated as unknown, as today for a dead pid.
- The same rule governs `ps` and `inspect`. A program sees its own group exactly, and other groups
  of its owner through each group's published table, at most one slice stale.
- So documents stop seeing and killing each other's processes. Today they can, through the shared
  app scheduler. Within one document nothing is lost (Q5).

**Published info.** At the end of each pump a group publishes a snapshot of its processes' info
(status, steps, times, memory) under a mutex. Cross-group `ps`, `pool.info(pid)` and the per-group
statistics read it.

**`kill_owner`** (D's graft). `pool.kill_owner(owner)` posts a kill to every group that has a
process of that owner, then waits until each group has applied it.
- In fork-join mode the wait pumps those groups' ingress inline.
- Ariadne calls it before freeing the action contexts that intrinsics point at (`ictx.document`,
  `ictx.channels`; `ariadne.cpp:1656, 1768`). Without the acknowledgement, a child in another group
  could still run against a freed context.
- A document's own group is simply destroyed (section 4.2).

### 4.6 Messaging between groups

**Inside a group:** unchanged. Delivery is immediate, broadcast reaches every waiter with one
shared envelope, and the pending FIFO keeps its cap of 1024 (`async_scheduler.h:105`).

**Stage 1: no router.** A document opts into its own group only if it uses no app-global channels.
- Its channels then all resolve under its chroot (`resolve_channel_key`, `intrinsics.cpp:1138-1146`).
- Its channel policy says whether it uses global keys (`app_runtime.cpp:244`).
- The host posts it receives (tick, key, pointer, signal, stream and async results) go to its group
  through `ictx.sched` and `ictx.host` (B30, section 4.12).

**Stage 2: channel homes.**
- **Each resolved channel key has one home group.** The home is the group that registered the
  longest prefix covering the key.
  - Groups register prefixes when they are made: a document group its document prefix, a spawn
    group nothing.
  - A key with no registered prefix (an app-global or `/`-escaped key, or a raw key in an
    unchrooted context) is homed at group 0. Group 0 is the app scheduler, or the `Exec` that owns
    the pool.
  - `#` keys (runtime-internal: tick, key, pointer and future channels) are always local to the
    group that resolves them.
- **The home owns the channel's waiters and pending FIFO.** Each group caches key-to-home lookups,
  invalidated by a directory epoch. When the pool has one group the router is never consulted.

**`msg-send` in group S to a key homed at H.** The state-tree half (`sendMessage`) is unchanged,
on pinned lookups. The scheduler half:
1. **H == S:** today's path. Every current waiter is delivered, local or remote. A remote waiter
   gets a `deliver{pid, seq, envelope}` post to its group, with a fresh envelope per receiving
   group. With no waiter, the message joins the pending FIFO.
2. **H != S:** S posts `send{key, envelope}` to H's ingress, and H performs step 1 when it drains,
   which is at its next step if it is running.

**`msg-recv` (or `await` on a channel) in group R on a key homed at H != R:**
1. R parks the process as today, with a fresh `seq`. It posts `subscribe{key, R, pid, seq,
   deadline}` to H.
2. H pops its pending FIFO and replies with `deliver`, or records a remote waiter.
3. When R applies `deliver{pid, seq, envelope}`, it patches the process's frame as today, if the
   process still waits with that `seq`.
4. **The re-send rule** (A's graft). If the process no longer waits with that `seq` (it timed out,
   was killed or was reaped), R posts `requeue{key, envelope}` to H. H delivers it to another waiter
   or puts it at the front of the pending FIFO. No message is lost.
5. **On a timeout or kill,** R posts `unsubscribe{key, pid, seq}` to H. If a delivery is already
   in flight, step 4 returns it.

**`msg-pending`** on a remote-homed key reads the home's published count, which is approximate.

**What is kept and what is not across groups:**
- **Kept:**
  - FIFO per (sender group, channel), because each ingress is one queue applied in order;
  - a broadcast reaches every process waiting at the home when the send is applied there, local
    and remote alike;
  - no loss to an expiry, kill or reap race;
  - the pending cap, applied at the home.
- **Not kept:**
  - order between messages from different groups;
  - the order of a requeued message relative to later sends on the channel (documented);
  - immediacy: a cross-group message arrives at the home's next drain.
- **Fork-join mode.** In fork-join mode (Stage 1 hosts that also enable the router) "next drain"
  means the receiving group's next pump. For Ariadne that is the next frame, as for host posts
  today.

**Bounded ingress** (D's graft). Cross-group entries into one group's ingress are capped
(`state_exec.groups.ingress_cap`, default 65536).
- Past the cap a cross-group `msg-send` returns its existing failure shape, `{status: "error"}`
  (`intrinsics.cpp:990-992`), with `reason: "ingress full"`.
- `pool.post_message` returns false.
- Control posts are never dropped.

**Cost.** One cross-group message costs:
- a mutex push into the target ingress, about 50-100 ns;
- a 4-entry envelope, about 5 allocations, plus the payload string copy;
- a wake of 5-20 us when the target group is idle and parked (0 when it is running).

Today's in-scheduler round trip is 23-95 us, mostly the tree-side `sendMessage`, so a cross-group
hop is a small fraction of it (judge 2's correction of D). It will matter more once messaging
itself gets faster, so placement keeps chatty pairs in one group.

### 4.7 Cycle collector and heap measurement

- **A registry per group** (B5, Stage 1):
  - `cycle_registry` becomes an opaque type in `heap.h`.
  - `registry()` (`heap.cpp:408-411`) returns the registry bound by the pump scope, and the
    thread-local one otherwise: bare evaluators, the reactive lane, host-thread setup.
  - `track()` and `note_process_exit()` follow it, as does the every-64th-step due check
    (`async_scheduler.cpp:155-156`).
- **No safepoint.** Collection runs inside the pump, at a step boundary, as today
  (`heap.cpp:565-566` still holds). Under invariant G every node the walk reaches is owned by the
  group or frozen.
  - Frozen environments are skipped (`heap.cpp:35-38`).
  - The 13 class dicts and the stdlib scope are held from outside the group, so the census sees
    `owners > internal` and keeps them live (`heap.cpp:612-618`).
  - No other thread mutates or refcounts anything in the walked graph.
- **Host references.** A host holding a group's value is an external owner, which keeps it live.
  Hosts read and drop such values only between pumps (fork-join) or under the token (continuous).
- **Lifetime.** A group's registry dies with the group, after one final collection. This also fixes
  a leak that exists today: an `Exec` driven from short-lived Python threads loses the objects
  tracked on each thread when it exits.
- **Moves** (Stage 3). The isolation walk that admits a move also re-tracks the process's tracked
  objects into the target group's registry.
- **Heap measurement** (`heap_share_job`, `heap.cpp:263-380`) runs as the process's own data steps.
  Under invariant G it reaches only group-owned and frozen nodes. For frozen ones it reads
  `use_count()` racily, but only for an estimate, so it needs no change (B6).

### 4.8 State tree

The tree is the one mutable structure all groups share. Stage 0 makes the DSL's use of it safe.

- **Pinned lookups** (B14). The intrinsics use `findDescendantShared` and `sharedChild`
  (`state.h:698-699`) instead of `findDescendant` and `operator()` (`intrinsics.cpp:133-251,
  960-990`).
  - `msg-send`'s node lookup is one of them.
  - `poll_watches` holds pins across use.
  - `sendMessage`'s link path already reads the pinned walk's path (`state.cpp:1583-1587`). The
    remaining unpinned `target->fullName()` (`state.cpp:1606-1607`) gets the pinned walk's path
    too.
- **A pinned chroot** (D's graft). `apply_chroot` keeps a `state_ptr` to the chroot node in the
  intrinsics context, alongside the raw `ictx.root` (`intrinsics.cpp:1134`). The node then cannot
  be freed while a context uses it.
- **`state-delete`** erases its one node from its parent. It no longer expires it and calls
  `root->sweepExpired()` (`intrinsics.cpp:187-191`).
  - One group can then no longer free nodes another group holds.
  - For unchrooted contexts it also removes a whole-tree walk. Ariadne's contexts sweep only their
    chroot today (`intrinsics.cpp:190, 1134`).
- **A per-watch root** (B13). `state-watch` records its chrooted root with the watch and no longer
  calls `set_watch_root(ctx->root)` (`intrinsics.cpp:384`). The scheduler's watch root stays the
  host's, for settings and charging.
- **Contention.**
  - A lookup takes each path segment's `boost::mutex` from where it starts (`state.cpp:1334-1354`),
    and a write emits up the parent chain.
  - Ariadne's lookups start at the chroot node, so documents writing disjoint subtrees share only
    the locks of the nodes above their prefixes, through write emission.
  - pycvc `Exec`s are unchrooted (`pycvc_exec.cpp:414`), so all their lookups start at the app
    root. `ExecPool` therefore gives each pooled `Exec` its own prefix by default.
  - `perf/sched` connects a counter slot to the watch root's change signals while a scheduler has
    watches or calls in flight. The slot's counter is atomic. With G groups watching the app root,
    every write calls G slots. Groups attach the slot to their own chroot where they have one.
- **signals2 subscribers** (B15) run on the writing thread (`state.h:308-331`).
  - **Who is affected.** A group pumped by a worker moves its writes' subscribers onto that worker.
    The known subscribers on DSL-written subtrees are `room_net` and `holon_sync` replication,
    `uri_http_cache`, and cvcGL's `state_object` handlers.
  - **Rule.** A document group gets `affinity::any` only when its subtree's subscribers are
    declared thread-safe. Otherwise it is `affinity::host`: still its own group, but pumped by the
    host thread.
  - **The audit** of those subscribers is part of Stage 1, and its outcome decides how many
    documents can leave the host thread (Q7).

### 4.9 Async host, ingress and host-thread natives

- **One async host per group**, bound to the group's scheduler as today (`async_host.h:52`). Hosts
  are built on first use, because most `ExecPool` groups never make an async call.
  - Kernels run on the shared kernel pools and lanes (`perf/async`).
  - A completion is posted to `call->async_sched_`, the creating group, so it always returns to the
    right group.
- **`intrinsics_context` gains `async_host *host`** (Stage 1, B30). Ariadne's providers register
  their async functions on `ictx.host` and post to `ictx.sched`, instead of `app.exec_async_host()`
  and `app.exec_scheduler()`:
  - `app_runtime.cpp:96-101`;
  - `net_intrinsics.cpp:268-276`;
  - `stream_intrinsics.cpp:138`;
  - the cvcGL verbs.

  Streams capture `ictx.sched` when they open (`stream/stream.cpp:132-148, 238`). The Runtime posts
  tick, key, pointer and signal messages to its group (`ariadne.cpp:1810-1811, 1843, 1967`). For a
  document that stays in group 0 these are the same objects as today.
- **Futures cannot leave their group.**
  - Messages carry strings and bytes only.
  - The isolating copy refuses futures at any depth (4.3).
  - `async_call::outcome()` (`async_host.cpp:109-113`) can therefore be reached from one group only.
  - `finish_async_call` still becomes idempotent (an atomic `closed` flag) as a cheap backstop, and
    awaiting a future whose `async_sched_` is not the current group raises `RuntimeError`.
- **The step rate** for charging is read again only when the tree changed (`perf/sched`).
- **Host-thread natives** (B22). A host registers each intrinsic provider with an affinity. The
  provider classification is Stage 1 work, and each provider's owner decides its kind.

| Kind | On the host thread | In a group pumped by a worker |
|---|---|---|
| `any` | inline | inline; the provider promises thread safety |
| `host_proxied` | inline | the process parks through `begin_external` (`async_host.cpp:541-579`). The call goes onto the pool's host-call queue, and the host thread runs it. The result comes back as a completion, at the group's next pump |
| `host_only` (the default for host-registered providers) | inline | not installed: a call is a NameError, never a race. A document whose env needs one is `affinity::host` |

- **The host-call queue.**
  - At a point that cannot park (a generator body; `request_await` throws there,
    `stackless_evaluator.cpp:457-460`), a `host_proxied` call blocks its worker until the host
    thread has run it.
  - **Rule: the host thread never blocks without servicing the host-call queue.** That covers the
    fork-join join, `kill_owner`, `with_group` waits and `wait_for_work`. A blocked worker
    therefore always makes progress.
  - libcvc's own builtins and stdlib are `any`. pycvc's Python natives are `any`, because
    `call_python` takes the GIL itself (`pycvc_exec.cpp:133-192`), so they serialize on the GIL.
- **`on_complete` callbacks** of processes in worker-pumped groups are queued and run on the host
  thread after the join (fork-join) or in `pump_host` (continuous), in completion order per group.
  Host-pumped groups run them inline, as today.
- **Kernel pools** (C's graft, Stage 2). `perf/async` gives one persistent background pool.
  - Once group workers compete with kernels for cores, the pool is split in two:
    - `background_cpu`, sized to the cores minus the group workers;
    - `background_io`, larger, for blocking kernels.
  - `executor::background` keeps meaning the first, and a registration option picks the second.
  - This reverses settled question 4 of `docs/STATE_EXEC_EXCEPTIONS.md` ("There's no separate I/O
    pool", line 840; section 3.5), so it is the user's call (Q9).
- **Blocking steps** (B21). An inline kernel or a `call->wait` at a point that cannot park
  (`intrinsics.cpp:537-542`) blocks only the worker pumping that group.
  - Other workers keep pumping other groups, which is better than today, where it stalls everything.
  - A long-step monitor logs any step over 1 ms with the native's name, as BEAM's `long_schedule`
    does.
  - In continuous mode, a worker blocked over 10 ms is excluded from the active count, and the
    pool may start one compensating worker (Go's syscall handoff). Stage 2 makes this optional.

### 4.10 Budgets and time accounting

- **Per process: unchanged.** `max_steps`, `max_time`, `max_memory`, `max_alloc_bytes`,
  `max_step_time`, the message limits and async charges (`docs/STATE_EXEC_EXCEPTIONS.md` 3.8) all
  have one writer, the group's pumping thread.
  - A step runs wholly on one thread, so `thread_cpu_seconds()` deltas stay valid
    (`process.h:182-186`; `types.h:322-324`).
  - With Stage S slices, a run is on one thread too. A group changes threads only between pumps.
- **Per pump.** Each group gets its own pump budget: Ariadne's 200000 steps and 0.1 s
  (`ariadne.cpp:1704-1705`), or `Exec`'s slice (`pycvc_exec.cpp:574-575`). `time_budget` charges
  thread CPU past its half-way point, so a pump descheduled by oversubscription still completes its
  quick work (`types.cpp:94-110`; `docs/ARIADNE.md` 2.2).
- **Per frame** (new). `pump_all(groups, budget)` also takes a wall deadline: Ariadne passes 0.1 s.
  - A group starts only if the deadline has not passed, and its pump's time budget is the smaller
    of its own and the time left.
  - Groups that did not start are put first in the next frame's order.
  - With more groups than threads, the frame stays bounded by about 0.1 s plus one pump, not by
    ceil(G / threads) x 0.1 s.
- **Ariadne's budget changes meaning only for documents that opt in.** Today 200000 steps and 0.1 s
  cover every document together, once per `Runtime::drain`. An isolated document gets its own
  200000 and 0.1 s per frame (section 5).
- **Hybrid CPUs.** A step on an E-core is about 1.6x slower, so `max_time` and Ariadne's 0.1 s
  action cap burn faster there. Step budgets are unaffected. P-core placement first (4.4).
- **`math.random`** (B26). Its generator moves from `thread_local` (`stdlib.cpp:1279`) into the
  group, bound by the pump scope, seeded from `std::random_device` on first use, or from the group
  name in deterministic mode. A group's stream then does not change when its pumping thread does.
  Bare evaluators keep the thread-local fallback.

### 4.11 Ordering guarantees

**Kept, at any thread count:**
1. **Inside a group, everything:** round robin, step counts, message order, broadcast and frame
   rules. A group's run equals its solo run, given the same inputs at the same pump boundaries.
   This is the oracle of section 8 (GR-1).
2. A process's steps never run concurrently, and keep program order.
3. FIFO per (sender group, channel) across groups.
4. A broadcast reaches every waiter registered at the channel's home when the send is applied.
5. No message is lost to an expiry, kill or reap race, inside or across groups.
6. An `await` crosses exactly one frame. `wake_awaiting` runs once per group per pump, and a
   multi-document host pumps once per frame (B29).
7. A quick Ariadne action completes within its drain, and render never runs during a fork-join
   pump.
8. State-tree writes are atomic per node, as today.
9. `print` writes whole lines: it formats the line, then makes one locked write
   (`builtins.cpp:688-707`).

**Not kept between different groups:**
1. Interleaving and relative progress. Deterministic mode reproduces them.
2. The order of messages from different groups, and the position of a requeued message.
3. The immediacy of cross-group kill, pause, resume and signal. They land at the target's next
   step boundary, and cross-group `ps` can lag one slice.
4. Strict priority across groups: each group's policy holds inside it.
5. That a reader in one group sees all of another group's writes from one step at once, across
   nodes.

### 4.12 Host APIs and how the hosts use them

**`exec_group`** (new: `inc/cvc/state/state_exec/exec_group.h`, `src/cvc/state/state_exec/exec_group.cpp`):

```cpp
enum class group_affinity { any, host };
class exec_group {
 public:
  struct options {
    std::string name, owner;                // stats under state_exec.groups.<name>.*
    group_affinity affinity = group_affinity::any;
    cvc::app *app = nullptr;
    cvc::state *watch_root = nullptr;
    environment_ptr base;                   // frozen scope to extend; null = the shared builtins
    std::vector<std::string> channel_prefixes; // Stage 2: the channels this group is home to
  };
  explicit exec_group(options);
  async_scheduler &sched();
  async_host &host();                       // built on first use
  pump_result pump(const pump_budget &);    // drain_ingress, wake_awaiting, sync_run; token held
  bool wants_pump() const;                  // runnable, ingress pending or a timer due
  template <class F> auto with_group(F &&); // run F holding the token (continuous mode)
  static exec_group *current();             // the group this thread is pumping, or null
};
```

**`exec_pool`** (new: `inc/cvc/core/exec_pool.h`, `src/cvc/core/exec_pool.cpp`; `app::exec_pool()`):

```cpp
struct pool_options {
  unsigned threads = 0;                     // 0 = every pump inline on the caller
  bool deterministic = false;
  double inline_below = 100e-6;             // adaptive helper wake (4.4)
  cpu_set placement = cpu_set::performance;
  double slice_seconds = 2e-3; uint64_t slice_steps = 20000; // continuous mode
};
class exec_pool {
 public:
  // Stage 1: fork-join
  void pump_all(std::span<exec_group *const>, const pump_budget &, clock::time_point deadline);
  // Stage 2: continuous, routing, control
  void attach(exec_group &); void detach(exec_group &);
  bool pump_host(double max_seconds);       // host-affinity groups and the host-call queue
  bool wait_for_work(double seconds);       // services the host-call queue while it waits
  bool post_message(const std::string &channel, value_t); // to the home group; false if full
  bool post_control(int pid, control_cmd);
  void kill_owner(const std::string &owner); // acknowledged by every group
  std::optional<process_info> info(int pid); // the published snapshot
  // Stage 3
  int spawn_isolated(const std::string &script, execute_options, value_t args);
  bool move(int pid, exec_group &to);       // false when the process is not movable
};
```

`async_scheduler` gains:
- `set_pid_source(std::shared_ptr<std::atomic<int>>)`;
- a base-env option;
- an ingress entry kind for control and router messages;
- `adopt(process_ptr)` (Stage 3: like `restore`, keeping the pid and the process's state).

The sync `scheduler` gains `post_control` (a mutex-guarded queue applied between steps) and atomic
stop flags (Stage 0).

**Ariadne.**
- **One pump per frame** (Stage 1, B29). `Runtime::drain()` splits in two, and `drain()` stays
  `drain_begin()` followed by `drain_end()`:
  - `drain_begin()` takes queued events, submits actions, residents and ticks, and makes the
    frame's posts. It does not pump.
  - `drain_end()` pumps the runtime's groups unless `app.exec_pump_frame()` has run since this
    Runtime's `drain_begin()` (the app keeps a pump generation counter), then sweeps
    (`ariadne.cpp:1709-1757`).
- **Hosts with several Runtimes.** They call `drain_begin` on each, then `app.exec_pump_frame()`
  once, then `drain_end` on each. That one call pumps group 0 once (`drain_ingress`,
  `wake_awaiting`, `sync_run`) and every isolated document group, through `pump_all` with the
  0.1 s frame deadline.
  - With several documents in group 0 this makes "an await crosses exactly one frame" true.
  - The shipped hosts drive one Runtime each (`src/arigl/main.cpp:1660`, `src/ari/main.cpp:292`).
    For them `drain()` is exactly today's.
- **Isolated documents** (`ariadne.exec.isolate_documents`, opt-in). The Runtime owns an
  `exec_group` instead of using `app.exec_scheduler()` (`ariadne.cpp:1644, 1690, 1757, 1810`).
  - `submit_action` and `ensure_resident` use its scheduler and set `ictx.sched` and `ictx.host`.
  - Teardown destroys the group (4.2), after `pool.kill_owner` when spawn groups hold the document's
    children (Stage 3).
  - **Conditions:**
    - no app-global channels until Stage 2;
    - its providers classified (4.9);
    - its subtree's subscribers declared thread-safe, or the group is `affinity::host`.
- **The reactive lane** runs its own evaluator on the render thread under `park_forbidden_scope`
  (`ariadne.cpp:670-690`). It never touches a group and is unchanged.
- **Parallel actions** (Stage 3, `ariadne.exec.parallel_actions: k`).
  - A document gets k action groups beside its resident group. Fire-once actions are placed
    least-loaded across them.
  - Actions already get their own env and context (`ariadne.cpp:1646-1676`), so an action placed in
    another group needs no copy.
  - The cost is that actions from one frame may run at the same time, so their state writes within
    a frame are in no fixed order.
  - `doc_scope_` verbs must be `any` or `host_proxied`.
  - This is the first speedup for a single document.

**pycvc.**
- **`Exec`** keeps its private scheduler, which becomes its group 0.
  - Stage 1 adds only the pump scope (registry and random stream per `Exec`).
  - Several `Exec`s on several Python threads run in parallel safely after Stage 0, with or without
    a pool.
  - `run()` already waits with `wait_for_work` instead of sleeping (`perf/sched`).
- **`pycvc.ExecPool(app, threads, size=threads)`** (Stage 1):
  - It holds `size` pooled `Exec`s, each its own group with its own prefix.
  - `run_all(sources) -> list[result]` distributes the programs, then pumps the groups on the pool
    in fork-join rounds with the GIL released for the whole batch.
  - Top-level definitions persist per pooled `Exec`, as in `Exec` (`pycvc_exec.cpp:421, 563`). A
    `fresh_env=True` option gives each program a fresh env (Q12).
  - Python natives retake the GIL, so pure-DSL programs scale and Python-calling programs serialize
    on it.
- **Stage 3.** An `Exec` program's `spawn-isolated` children run in the app pool's spawn groups.
  `Exec.run` keeps pumping its own group and wakes on their messages.

**exec_coordinator.** Its `sched_` is the sync `scheduler` (`exec_coordinator.h:236`).
- **Stage 0.** `handle_control` (`exec_coordinator.cpp:1045-1071`) and `migrate`'s pause and kill
  (`712-753`) go through `scheduler::post_control`. This fixes a race that exists today.
- **Stage 2.**
  - `attach_pool(exec_pool &)` generalizes `attach_scheduler` (`exec_coordinator.cpp:223`).
  - `submit_local` (`exec_coordinator.cpp:586-615`) places a job in its own job group, or in the
    least-loaded of k job groups. This is the in-process analogue of its leader forwarding
    (`539-569`).
  - The sync-scheduler path stays for the CLI (Q13).
- **Cross-node `migrate`.** Today it sends metadata and the target starts `nil`
  (`exec_coordinator.cpp:809-866`). It can send `encode_process` of a process paused under its
  group's token, and the target restores it and calls `rebind_natives` (`state_value_codec.h:72-82`).
  The codec omits the inbox, watches and pending signals (`66-71`), so this is a separate fix. The
  in-process move of Stage 3 does not use the codec.

### 4.13 Configuration and opting in

From highest to lowest precedence, following the settings rules of
`docs/STATE_EXEC_EXCEPTIONS.md` 3.8:
1. the `pool_options` a host passes;
2. the app tree's `state_exec.threads`, `state_exec.cpu_set` and `state_exec.deterministic`;
3. the built-in defaults: threads 0, P-cores, not deterministic.

The pool reads them when it is built. `threads` cannot change while groups are attached.

| Host | How it opts in | Stage |
|---|---|---|
| C++ embedder | make `exec_group`s; `app.exec_pool().pump_all(...)` or `attach` | 1, 2 |
| Ariadne, several documents | `state_exec.threads: auto`, `ariadne.exec.isolate_documents: true` | 1 |
| Ariadne, one document | `ariadne.exec.parallel_actions: k`; programs use `spawn-isolated` | 3 |
| pycvc | `pycvc.ExecPool(app, threads=8).run_all(...)`; several `Exec`s on several threads | 1 (0) |
| `exec_coordinator` | `attach_pool` | 2 |
| CLI | none (sync scheduler) | - |

**A program opts in** with `(spawn-isolated script [name] [priority] [args])` (Stage 3), which
returns the child's pid. Plain `spawn` stays in the caller's group. Placing plain spawns on spawn
groups automatically (D's `auto_spawn`) is open (Q4).

**Debug knobs** (environment variables, read once):
- `CVC_EXEC_DETERMINISTIC=1`: deterministic mode (4.4).
- `CVC_EXEC_FORCE_MOVE=1`: every group is pumped by a different worker on each pump; in Stage 3,
  every movable process moves every K pumps.
- `CVC_EXEC_VERIFY=1`: boundary tagging and isolation walks (4.3).
- `CVC_EXEC_FUZZ=<seed>`: yields and short sleeps at the protocol points (the state word, ingress
  push, eventcount, router posts).

### 4.14 When only one thread is available

- `threads` is clamped to `std::thread::hardware_concurrency() - 1` and to the process's affinity
  mask where the OS reports one. A clamp to 0 is logged once.
- With 0 threads every `pump_all` pumps its groups inline on the caller, in LPT order, and
  continuous mode pumps attached groups only from `pump_host` and `wait_for_work`. Results are
  those of deterministic mode.
- The cost over today is the pump scope (three thread-local stores per pump) and, with isolated
  documents, one pump per group instead of one shared one.
- Deterministic tests and the CLI keep 0 threads.
- Emscripten builds without threads compile the workers out. `threads > 0` is clamped to 0 with a
  log line.

### 4.15 Spawned and isolated processes, and moving a process (Stage 1 and Stage 3)

**The spawn context** (B27, Stage 1). `intrinsics_context` gains
`std::function<spawn_setup(const intrinsics_context &parent)> spawn_context`, set by the host.
- `spawn_setup` returns the child's env and an owned context object, held by the child process.
- **The default** (`make_spawn_context`, for hosts that call plain `register_intrinsics`) gives the
  child a new heap-owned `intrinsics_context`:
  - it copies the parent's `sched`, `host`, `root` (with its pin), `root_path`, `uid` and
    `channels`;
  - it builds `extend(base)` with the intrinsics registered against that context;
  - the child gets the parent's `owner`.
- **Ariadne's callback** also re-applies the action intrinsic providers and sets `document`.
- **pycvc's callback** gives the child `extend(natives)`.
  - `natives` is a new scope holding the `Exec`'s registered natives.
  - `impl_->env` becomes `extend(natives)`, so the REPL's top-level definitions still persist
    across runs.
- **With no callback** a child keeps today's bare env.
- **The child's result.** A spawned child reports by message. The pool also records each
  `spawn-isolated` child's exit status for `inspect`.

**`(fork)`** (B28, Stage 0) forks the running process, `current_pid()`, not `ctx->pid`, which hosts
leave at -1 (Ariadne, `ariadne.cpp:1645-1660`) or 0 (pycvc, `pycvc_exec.cpp:416`). The child copies
`owner` (`async_scheduler.cpp:957-976` copies everything else), so `kill_owner` reaches it.

**`spawn-isolated`** (Stage 3):
1. The pool picks a spawn group of the caller's owner: the least-loaded one, or a new one while the
   owner has fewer than `threads`.
2. It runs the owner's spawn context against that group, with `sched` and `host` set to the target
   group's.
3. The context's `document` is null unless the host declares its `document_scope` thread-safe.
   Verbs that need it are `host_proxied`.
4. Optional `args` cross by isolating copy and are bound as `args` in the child's env.
5. The child talks to its parent by channel through the router (4.6), since both resolve channels
   under the same chroot.

**Moving a process between groups** (Stage 3b; for balancing, by `pool.move(pid, group)` or the
pool's rebalancer).
- **Eligibility.** The process must be:
  - `ready`, not parked: a parked process's waiter registration and timers live in its group;
  - not in a data job (`waits_on_data_work`, `async_scheduler.cpp:955`);
  - free of async calls in flight, state watches and fork family;
  - free of any object shared with another process, which the isolation walk proves: no shared
    env, generator or compound value.

  Isolated spawns usually qualify; a process that does not is simply not moved.
- **Group-relative resolution** (B31; B's graft). Processes made by `spawn-isolated` are created
  movable:
  - their context's `sched` points at a forwarder (`group_scheduler_ref : scheduler_base`), which
    forwards every call to `exec_group::current()->sched()`;
  - their async natives resolve their host through `exec_group::current()->host()`;
  - their context's `tracker` is resolved the same way.

  A moved process's intrinsics therefore follow it, and nothing calls the old group from a new
  thread. The cost is one thread-local load and a virtual call per intrinsic call, only for movable
  processes.
- **The move:**
  1. The source group, holding its token, removes the process from its table and run queue.
  2. It re-tracks the process's objects into the target's registry, during the isolation walk.
  3. It posts `adopt{process_ptr}` to the target's ingress.
  4. The target inserts the process with its pid (`adopt`, B32), and the directory is updated.
- **It is a pointer move, not a codec round trip,** so the inbox, signal handlers and pending
  signals move with it. D's transfer through `encode_process` loses them.

### 4.16 Stage 4 in reserve: several threads inside one group

Built only if Stages 2 and 3 show single groups that are DSL-bound and that `spawn-isolated` and
parallel actions cannot split. The design is option A applied to one group flagged `parallel`, with
the judges' corrections. It is recorded here so the earlier stages do not close it off.

- **Domains.** The group's processes are partitioned into domains of processes that may share
  mutable values: a fork family, the processes of one env, one Ariadne document. A domain is
  claimed by one worker at a time, and runs slices.
- **The step context.** A `thread_local step_context {group, domain, process, pid, park_forbidden}`
  stands behind `current_pid()` and `current_process()` (B1, B20).
  - `release_if_idle` releases unless `t_step->proc == &proc` (B3).
  - Only a domain's owner releases its processes.
- **Requests.**
  - Cross-domain kill, pause, resume, signal, delivery, completion and frame wakes are requests on
    the target domain's MPSC inbox, applied by its owner between steps.
  - Kill also sets an atomic `kill_requested` flag, checked before every step.
  - Processes publish their info, seqlocked, at the end of each run.
- **The domain state word** has `SCHEDULED` and `NOTIFIED` bits. Release CASes `SCHEDULED -> 0`
  only when `NOTIFIED` is clear, and otherwise re-applies the inbox. That closes the lost wakeup in
  A's release (judge 1).
- **Channels.** A sharded channel table replaces `pending_messages_`.
  - `msg-recv`'s pop-or-register is one operation under the shard lock (B4).
  - Waiters carry a `seq`, and a delivery that finds its waiter gone is re-sent.
- **Values.** Each receiver gets its own envelope (B9), a shared AST is copied at submit (B12), and
  the group's base env is frozen (B16).
- **The collector.** One registry per domain, collected at the owner's slice prologue, with no
  stop-the-world. A stop-the-world was E's own proposal, and the judges rejected it (B5).
- **Host access.** `with_domain` claims a domain, and host-proxied natives go through the
  host-call queue as in 4.9.
- **Its tests** reuse the GR oracle (8.2), the stress cases (8.8) and the sanitizer jobs (8.7),
  plus an exhaustive small-state model of the domain word and inbox.
- **Cost:** 8-12 engineer-weeks at high risk.

### 4.17 Failure modes

| Failure | Handling |
|---|---|
| A program error in a step | As today: the process ends `failed` or `killed` (`async_scheduler.cpp:228-268`) |
| A C++ exception escaping a pump | The pool catches it, marks the group failed, and kills its processes with "internal error". The host learns of it at the join (fork-join), or from `pool.failed_groups()`; other groups continue. No exception escapes a worker thread |
| A native that loops inside one step | Bounded by its deadline guard (`max_step_time`, the pump's time budget). It holds one worker; other groups keep running, which is better than today. A fork-join join waits for it, bounded by the frame deadline plus that step |
| A worker blocked in an inline kernel or `call->wait` | Holds its group only; long-step monitor; optional compensating worker (4.9) |
| A host-only native called in a worker-pumped group | Impossible by construction: not installed, so a NameError. Debug builds also assert that `host_proxied` bodies run on the host thread |
| A host posts one compound value to two groups (invariant G broken) | `CVC_EXEC_VERIFY` asserts; TSan in CI. Release builds have a data race; the host API documents a posted value as moved in |
| A subscriber assumed the main thread but its document was declared `any` | A data race in the host's code. Mitigated by the audit and by `host_only` as the default |
| Cross-group ingress full | `msg-send` returns `{status: "error", reason: "ingress full"}`; `post_message` returns false; counted per group |
| A remote waiter vanishes with a delivery in flight | Requeued at the home (4.6); counted |
| A group destroyed while pumping | Destruction takes the token and waits; kernels in flight are waited for (4.2) |
| A worker that fails to start | The pool runs with the workers that started and logs; the caller always pumps |
| Teardown of the app | `~exec_pool` stops workers after the current pumps, then groups are destroyed in reverse creation order, then the kernel pools |

### 4.18 Every blocker, resolved

| ID | Resolution | Section, stage |
|---|---|---|
| B1 | A group is stepped by one thread at a time; Stage 4: thread-local step context | 4.1; 4.16 |
| B2 | Owned by the pumping thread; hosts touch them between pumps or under the token; cross-thread input only through posts | 4.1, 4.5; Stage 1-2 |
| B3 | Same; across groups, `post_control` applied between the target's steps; Stage 4: requests and owner-only release | 4.5; 4.16 |
| B4 | Same group: one thread. Across groups: subscription at the home with seq and re-send; Stage 4: atomic pop-or-register | 4.6; 4.16 |
| B5 | Registry per group, bound by the pump scope; freed with the group; re-tracked on a move | 4.7; Stage 1 |
| B6 | Unchanged: under invariant G the walk reaches only owned and frozen nodes | 4.7 |
| B7 | The 13 class dicts refuse mutation (`TypeError`) | 4.3, 5; Stage 0 |
| B8 | Fork families stay in one group; `(fork)` fixed (B28); Stage 4: one domain | 4.15; 4.16 |
| B9 | Inside a group, unchanged; across groups a fresh envelope per receiving group; Stage 4: per receiver | 4.6; 4.16 |
| B10 | `state-data-set` stores an isolating copy; callables, generators, environments and futures refused at any depth | 4.3; Stage 0 |
| B11 | Processes sharing an env or context stay in one group; a move requires the isolation walk | 4.3, 4.15 |
| B12 | `execute(value_t)` into a group copies a shared AST; generators cannot cross (isolating copy) | 4.3; Stage 1 |
| B13 | Per-watch root | 4.8; Stage 0 |
| B14 | Pinned lookups and a pinned chroot; `state-delete` erases one node | 4.8; Stage 0 |
| B15 | `affinity::host` unless the subtree's subscribers are declared thread-safe; audit | 4.8; Stage 1 |
| B16 | One frozen builtins scope, extended per group | 4.2; Stage 1 |
| B17 | Atomic stop flags; `post_control` on both schedulers; `exec_coordinator` uses it | 4.5, 4.12; Stage 0 |
| B18 | Futures cannot leave their group (isolating copy, messages are strings); `finish_async_call` idempotent | 4.9; Stage 0 |
| B19 | `perf/sched`'s `wait_for_work`; Stage 2's group state word and pool eventcount | 4.4; Stage 2 |
| B20 | Only the reactive lane arms it, on the host thread; Stage 4: in the step context | 4.12; 4.16 |
| B21 | Blocks one worker only; long-step monitor; optional compensating worker | 4.9; Stage 2 |
| B22 | Provider kinds `any`, `host_proxied`, `host_only`; host-call queue; `affinity::host` groups | 4.9; Stage 1 |
| B23 | `perf/async` | 2.4 |
| B24 | `perf/async` | 2.4 |
| B25 | Per-process parsing; a shared AST copied at submit; class dicts frozen and measured (`mt.groups` raise/catch); per-group copies if contention shows (Q2) | 4.3, 6; Stage 1 |
| B26 | Random stream per group; line-atomic `print` | 4.10, 4.11; Stage 1 |
| B27 | Spawn context with a default; Ariadne's and pycvc's callbacks | 4.15; Stage 1 |
| B28 | `(fork)` forks `current_pid()`; the child keeps `owner` | 4.15; Stage 0 |
| B29 | `drain_begin` / `exec_pump_frame` / `drain_end` | 4.12; Stage 1 |
| B30 | `ictx.host` and `ictx.sched` at every posting and async-function site | 4.9; Stage 1 |
| B31 | Movable processes resolve their scheduler, host and tracker through `exec_group::current()` | 4.15; Stage 3 |
| B32 | One pid source per pool; `adopt` keeps the pid | 4.5, 4.15; Stage 1, 3 |

## 5. Semantics changes and compatibility

Each item says when it applies. "Always" items land whether or not a host enables threads.

1. **Always (Stage 0): built-in exception classes are read-only.**
   - `set-attr`, `del-attr` and the C++ `set_field` raise `TypeError` ("built-in class is
     read-only") on one of the 13 built-in exception class dicts. That covers `Exception.__cls_dict__`
     and the other 12, and a caught exception's `__class__` when it is built in.
   - User classes, including ones derived from the built-ins, are unaffected.
   - Today one program can redefine `init` for every program in the OS process.
2. **Always (Stage 0): `state-data-set` stores an isolating copy.**
   - A list changed after it was stored no longer changes what `state-data-get` returns.
   - The copy is deep-data work charged to the writer, O(size) in steps.
   - A closure, native, generator, environment or future anywhere inside the value is refused with
     the existing `runtime_error`; today only a top-level callable is.
3. **Always (Stage 0): `state-delete` deletes only its node.** It no longer sweeps every other
   expired node under the context's root as a side effect.
4. **Always (Stage 0): `state-watch` from a chrooted process** no longer changes where the scheduler
   reads its settings.
5. **Always (Stage 0): `(fork)` forks the running process.** Under Ariadne and pycvc it used to fork
   pid -1 or 0, which fails. The child keeps `owner`, so `kill_owner` reaps it.
6. **Always (Stage 1): spawned children get an intrinsics context** and the parent's chroot, owner
   and channel policy, where the host installed a spawn context: Ariadne, pycvc, and hosts that use
   `make_spawn_context`. Today they get the bare builtins.
7. **Always (Stage 1): `math.random` draws from a stream per scheduler**, not per OS thread. This is
   visible only to programs in several schedulers on one thread.
8. **Always (Stage 1): cycle-collector statistics** (`tracked_objects()`, `cycle_stats`) are per
   scheduler, not per thread. This is visible only to tests and diagnostics.
9. **Always (Stage 1): `on_complete` callbacks** of processes in groups pumped off the host thread
   run on the host thread after the join. Groups pumped on the host thread run them inline, as
   today.
10. **Opt-in (several Runtimes and `exec_pump_frame`): the shared scheduler is pumped once per
    frame.** Today it is pumped once per document. With several documents an `await` now crosses
    exactly one frame; before, it could resume within the same frame.
11. **Opt-in (`isolate_documents`): an isolated document runs in its own scheduler.**
    - Its pump budget is 200000 steps and 0.1 s per frame, of its own.
    - Its `ps`, `kill` and `inspect` no longer see other documents' processes.
    - Its natives and subscribers may run on a worker thread, if declared `any`.
    - Pids stay unique app-wide.
12. **Opt-in (Stage 2 router): cross-group messages.**
    - A message to a channel homed in another group arrives at that group's next drain.
    - FIFO holds per sender group and channel, with no order across groups.
    - A message bounced by a vanished waiter may be reordered.
    - A full ingress makes `msg-send` return `{status: "error", reason: "ingress full"}`.
13. **Opt-in (Stage 2): cross-group control.** `kill`, `pause`, `resume` and `send-signal` on a
    same-owner process in another group return "requested" and land at its next step boundary.
    Cross-group `ps` and `inspect` read a snapshot at most one slice old.
14. **Opt-in (Stage 3): `spawn-isolated`.** The child runs in another group with copied arguments.
    It cannot share values with its parent, only messages and the tree.
15. **Opt-in (Stage 3): `parallel_actions`.** A document's actions from one frame may run at the
    same time, and their state writes are in no fixed order.
16. **Opt-in (Stage S, `slice_steps > 1`): slices.**
    - Processes in one group take turns in runs of up to `slice_steps` steps.
    - Watch events are seen per run.
    - `max_time` is checked every 16 steps, so it overshoots by at most 16 steps.
    - `slice_steps = 1` is exact.
17. **Opt-in (Stage 2, Q9): the split kernel pools.** CPU-bound and blocking kernels run on separate
    pools.
18. **Unchanged, deliberately:**
    - fork's shared environments;
    - the shared broadcast envelope inside a group;
    - a host env used as the global env;
    - strict priority within a group;
    - every host API that exists today.

**Compatibility checks before Stage 0 merges.** Scan the corpus, the tests and the shipped
documents for each of these, and list any hits in the PR:
- `set-attr` or `del-attr` on built-in classes;
- `state-data-set` of a list that is mutated afterwards, or that holds a callable;
- reliance on `state-delete`'s sweep;
- `(fork)` under Ariadne or pycvc.

## 6. Expected performance

### 6.1 The data points

Section 2.5 has the measured anchors:
- the bare step: 86-104 ns;
- the scheduled step: 249-263 ns sync, 456-470 ns async, before `perf/sched`;
- `mt.independent`: 4.45x at 8 threads for fib, 3.76x with a shared AST;
- `msg.pingpong`: 23-95 us a round trip;
- `msg.ingress`: 46-61k messages/s.

The expectations below are estimates derived from them. Each names the scenario that will confirm
or refute it.

### 6.2 What to expect

| Workload | Expected | Why | Scenario |
|---|---|---|---|
| Independent CPU-bound groups (several `Exec`s, an `ExecPool` batch, coordinator jobs) | 90-97% of the `mt.independent` curve: about 2.4-2.6x at 4 threads, 4.0-4.3x at 8 on this machine | Nothing added per step; a pump adds three thread-local stores and a token CAS per 2 ms or per frame; private ASTs | `mt.groups`, `mt.pycvc_threads` |
| The same past 8 threads | perhaps 5.5-7x at 14-16, flattening as E-cores join (unmeasured) | E-cores at about 0.5-0.6 of a P-core | `mt.independent` and `mt.groups` at t = 12, 16, 20 |
| One group (one document, one `Exec` program, a fork family) | 1.0x from threads | One thread steps a group | `mt.groups` G=1 |
| One group with Stage S slices | a further cut in ns/step after `perf/sched`; size unknown until `perf/sched` is measured | Guards, thread-local setup and clock reads once per run | `sched.slice` |
| Floods of tiny programs | Near the independent row when pooled `Exec`s are reused; a fresh group per program pays its setup (a scheduler and an extended frozen builtins scope, tens of us, against about 150 allocations without lightweight groups) | `ExecPool` reuses its `size` groups | `mt.groups.flood` |
| Ariadne, one document | 1.0x | Group 0 | `ariadne.frames` D=1 |
| Ariadne, D isolated documents with heavy residents | up to min(D, threads) x about 0.6 per-thread efficiency on the DSL share of the frame, unless documents are host-affine | One group per document, fork-join per frame | `mt.ariadne_docs` |
| Ariadne, a typical small frame | no change; no helper woken | Adaptive helper wake | `mt.ariadne_docs` small, `extra.helper_wakes` = 0 |
| Message-heavy inside one group | 1.0x | Unchanged | `msg.*` |
| Request and reply across two groups | per message about 0.5-1 us of post and copy, plus a 5-20 us wake when the target is parked; small against today's 23-95 us round trip | Ingress hop | `mt.cross_group_msg` pingpong |
| Fan-out from one dispatcher to many isolated workers (Stage 3) | Like independent groups once each worker's work per message exceeds about 30 us | Receivers' work is parallel | `mt.cross_group_msg` fanout, `mt.isolated_spawn` |
| State-tree loops in disjoint chrooted subtrees | about 2-3x at 8 groups (estimate) | Per-segment locks below each chroot; writes emit up the parent chain | `mt.state_tree` disjoint |
| State-tree loops on a shared subtree, or unchrooted | about 1x, contended | Locks on the same nodes | `mt.state_tree` shared |
| Async-heavy | The DSL side (marshal, copy, park, charge) scales with groups; kernel throughput with the pools; completion latency loses its polling floor (`perf/sched`) | Each group has its own ingress and drain | `mt.groups.async`, `async.flood.*` |
| Stage 4, if built | Up to the independent row for separate domains inside one group | Domains claimed per slice | `mt.sched.independent` |

**What limits all of it on this machine.**
- The 1.6x per-thread inflation at 8 threads: E-cores, the all-core clock, and the allocator.
- A scalable allocator (mimalloc from cvcpkg, or the Windows segment heap) is the most likely next
  lever, since `malloc` is about 21% of a single-thread step. It is measured against
  `mt.independent` raise/catch, whose per-thread time doubles at 8 threads, before worker defaults
  are chosen (Q8).

### 6.3 How the benchmark suite verifies it

**Existing scenarios** that must not regress:
- `sched.single.*`, `flood.*` and `limits.*`: at `threads = 0`, within 3% of the base, since the
  pump scope must cost under 1 ns a step;
- `msg.*`, `async.*`: unchanged at `threads = 0`;
- `mt.independent`: the ceiling, re-measured in the same rep as every new `mt.*` point and extended
  to t = 12, 16, 20.

**New scenarios** (in `make_scenarios()` of `state_exec_bench.cpp`, marked `spawns_threads`):

| Scenario | Stage | What it runs | Reports |
|---|---|---|---|
| `mt.multi_exec` | 0 | T threads, each with a full `async_scheduler`, app-tree intrinsics and async host, running fib and raise/catch; the TSan shape | `extra.speedup`, against `mt.independent` |
| `mt.groups` | 1 | G groups of fib, raise/catch or cycle-making closures, on the pool at threads 1, 2, 4, 8, 16; fork-join and (Stage 2) continuous | `extra.speedup`, `extra.of_ceiling` against `mt.independent` in the same rep, pump and wake counts |
| `mt.groups.flood` | 1 | n tiny programs through `ExecPool` with reused and with fresh groups | programs/s, group setup ns |
| `mt.pycvc_threads` | 0 | T Python threads, one `Exec` each (pytest-benchmark driven) | speedup |
| `mt.ariadne_docs` | 1 | D Runtimes with heavy residents, and D small documents, through `drain_begin` / `exec_pump_frame` / `drain_end`, threads 0-8 | frame DSL time, `extra.helper_wakes` |
| `mt.cross_group_msg` | 2 | pingpong within one group and across two; fan-out from one dispatcher to k groups; broadcast to waiters in k groups | ns per message, wake latency percentiles |
| `mt.state_tree` | 1 | `state-get`/`state-set` loops from G groups on disjoint and on shared subtrees, chrooted and not | ns per op against G |
| `mt.groups.async` | 2 | async kernels from G groups, `wait_for_work` drivers | completion latency, calls/s |
| `mt.groups.move` | 2, 3 | `CVC_EXEC_FORCE_MOVE` on and off; Stage 3 process moves | cost per group move and per process move |
| `mt.isolated_spawn` | 3 | one parent spawning k `spawn-isolated` fib children that report by message | speedup against `mt.groups` |
| `sched.slice` | S | the `sched.single` workloads at `slice_steps` 1, 16, 64, 256, 1024 | ns/step, `extra.fairness` |

**Gates.** Base and candidate are interleaved twice with 5 reps, as `docs/STATE_EXEC_BENCH.md`
("Getting stable numbers") prescribes.
- **Stage 0:**
  - `mt.multi_exec` TSan-clean;
  - `sched.single.*` within 3%;
  - `mt.multi_exec` at 8 threads at least 0.9x `mt.independent`.
- **Stage 1:**
  - `mt.groups` fib at 8 threads at least 0.9x `mt.independent`'s speedup in the same rep;
  - `mt.ariadne_docs` small frames show 0 helper wakes;
  - `sched.single.*` at `threads = 0` within 3%.
- **Stage 2:**
  - `mt.groups` continuous at 8 threads at least 0.9x;
  - `mt.cross_group_msg` wake latency p50 under 50 us on Windows;
  - `mt.groups.move` group-move cost under 5 us.
- **Stage 3:** `mt.isolated_spawn` with children of at least 1 ms of work reaches at least 0.8x
  `mt.groups` at 8.
- **Stage S:**
  - `slice_steps = 1` bit-identical (SL-1);
  - `slice_steps = 256` at least 20% fewer ns/step than `perf/sched`'s async `sched.single`, or the
    stage is dropped.

## 7. Phases

Each stage ships on its own, after the previous one, with its tests and its benchmark gate. Stage S
is independent of the others.

| Stage | Content | Builds on | Done when |
|---|---|---|---|
| 0 | Process-global hazard fixes; no threads added | `perf/async` (pool), `perf/sched` (ingress flag) | HZ cases pass; TSan clean on `mt.multi_exec` |
| 1 | `exec_group`, a fork-join `exec_pool`, registry and random stream per group, lightweight groups, pid source, spawn context, host natives classified, Ariadne one pump per frame and isolated documents, `ExecPool` | Stage 0 | GR, IS-1, IS-2 and AR-1 to AR-4 pass; Stage 1 gate |
| 2 | Continuous pool with wakeups, router with channel homes, cross-group control, published info, bounded ingress, split kernel pools, `exec_coordinator` on the pool | Stage 1; `perf/sched` (timer heap, `wait_for_work`) | PL, MS and AR-7 pass; Stage 2 gate |
| 3 | `spawn-isolated`, spawn groups, movable processes and moves, Ariadne parallel actions | Stage 2 | IS-3 to IS-10, AR-5 and AR-6 pass; Stage 3 gate |
| S | Slices in today's `async_scheduler`, single-threaded | `perf/sched` (direct step, one clock reading) | SL pass; Stage S gate |
| 4 | Several threads inside one group (4.16) | Stage 3, and data showing DSL-bound single groups | its own design review |

**Stage 0: hazards** (1.5-2 engineer-weeks).
- **Files:**
  - `exceptions.cpp` (`is_builtin_class_dict`) and `builtins.cpp:585-613`: read-only class dicts;
  - `data_job.h`/`.cpp`: the isolating copy mode;
  - `intrinsics.cpp`/`intrinsics.h`:
    - the isolating copy in `state-data-set`;
    - pinned lookups and the pinned chroot;
    - a single-node `state-delete`;
    - the per-watch root;
    - `(fork)` of `current_pid()`;
  - `async_scheduler.cpp`: fork copies `owner`; an idempotent `finish_async_call`; atomic stop;
    `post_control`;
  - `scheduler.h`/`.cpp`: `post_control` (a mutex-guarded queue applied between steps), atomic stop
    flags;
  - `exec_coordinator.cpp`: control through `post_control`;
  - `state.cpp:1606-1607`: the pinned path in `sendMessage`;
  - `builtins.cpp:688-707`: line-atomic `print`;
  - docs: `STATE_EXEC_DEVELOPER_GUIDE.md`, `STATE_EXEC_EXCEPTIONS.md` (the class and data-channel
    rules).
- **CI:** a Linux clang TSan job (ubuntu-24.04, `-fsanitize=thread`, RelWithDebInfo) running the
  state_exec tests, `ariadne_runtime_test`, `state_exec_coordinator_test` and `mt.multi_exec`. Its
  suppressions file covers boost.thread and signals2 internals only.
- **Tests:** HZ-1 to HZ-14.
- **Exit:** every state_exec, Ariadne and pycvc suite passes; the TSan job is clean; the
  compatibility scan of section 5 is in the PR.
- **Risk:** low. The behaviour changes are items 1-5 of section 5.

**Stage 1: groups and a fork-join pool** (3-4 weeks).
- **Files:**
  - new `exec_group.h`/`.cpp`, `exec_pool.h`/`.cpp`;
  - `app.h`/`app.cpp`: `exec_pool()`, `exec_pump_frame()`, the settings;
  - `heap.h`/`heap.cpp:392-411`: an opaque registry and the pump-scope binding;
  - `stdlib.cpp:1279`: the random stream per group;
  - `async_scheduler.h`/`.cpp`: the base-env option, the pid source;
  - `builtins.cpp`: the shared frozen builtins scope;
  - `intrinsics.h`/`.cpp`: `ictx.host`, `spawn_context`, `make_spawn_context`;
  - `app_runtime.cpp`, `net_intrinsics.cpp`, `stream*.cpp`, `src/cvcGL/ariadne/*_verbs.cpp`:
    `ictx.sched`/`ictx.host`, provider kinds;
  - `ariadne.cpp`: `drain_begin`/`drain_end`, isolated documents, its spawn context;
  - `bindings/pycvc`: `ExecPool`, the natives scope and spawn context.
- **Content:** besides the above:
  - LPT fork-join with adaptive helper wake and the frame deadline;
  - deterministic mode;
  - `on_complete` on the host thread;
  - the subscriber audit (`room_net`, `holon_sync`, `uri_http_cache`, cvcGL `state_object`), with
    each subscriber declared thread-safe or not.
- **Tests:** GR-1 to GR-12, IS-1, IS-2, AR-1 to AR-4, DT-1, DT-2, TS-1 and TS-2 on the new tests.
- **Exit:** the Stage 1 gate; Ariadne and pycvc unchanged with defaults.
- **Risk:** medium-low. The surprises are host natives and subscribers that assume the main thread.
  Any doubtful one defaults to `host_only` or not thread-safe.

**Stage 2: continuous pool, router and control** (3-4 weeks).
- **Files:**
  - `exec_pool.cpp`: the state word, ready queue, eventcount, `next` slot, timers, `pump_host`,
    `with_group`, the pid directory, published info, `kill_owner` with acknowledgement;
  - new `exec_router.h`/`.cpp`: homes, the subscription protocol, requeue, caps;
  - `intrinsics.cpp:960-1015, 1167-1191`: `msg-send` and `msg-recv` through the router when the pool
    has more than one group;
  - `async_scheduler.cpp`: the ingress entry kinds;
  - `app.cpp`/`async_host.cpp`: the split kernel pools (if Q9 says yes) and the long-step monitor;
  - `exec_coordinator.cpp`: `attach_pool`, job groups.
- **Tests:** PL-1 to PL-10, MS-1 to MS-12, AR-7, DT-3, ST-1 to ST-3.
- **Exit:** the Stage 2 gate; the channel semantics matrix (MS-12) matches single-group outcomes.
- **Risk:** medium. The router's corner cases are timed waits on remote homes, kills between
  subscribe and deliver, and caps at the home.

**Stage 3: isolated spawns and moves** (3 weeks, plus 1-2 for parallel actions).
- **Files:**
  - `intrinsics.cpp`: `spawn-isolated`;
  - `exec_pool.cpp`: spawn groups, `move`, the rebalancer;
  - new `group_scheduler_ref`;
  - `async_host.cpp`: host resolution through the current group for movable processes;
  - `async_scheduler.cpp`: `adopt`, detach;
  - `heap.cpp`: the isolation walk and re-tracking;
  - `ariadne.cpp`: `parallel_actions`.
- **Tests:** IS-3 to IS-10, AR-5, AR-6, ST-4.
- **Exit:** the Stage 3 gate; forced moves (`CVC_EXEC_FORCE_MOVE`) leave every suite's results
  unchanged.
- **Risk:** medium. The isolation walk must refuse whenever it is unsure. Parallel actions make
  write order within a frame nondeterministic (opt-in only).

**Stage S: slices** (1-2 weeks, after `perf/sched`).
- **Files:** `async_scheduler.cpp`/`.h`, `process.cpp` (per-run accounting), `scheduler_base.h`
  (`slice_steps`, `slice_seconds`).
- **Tests:** SL-1 to SL-6; the whole suite at `slice_steps` 1 and 256.
- **Exit:** the Stage S gate.
- **Risk:** low.

**Stage 4: several threads inside one group** (8-12 weeks, high risk). Only on evidence: Stage 2-3
benchmarks or user reports showing a single group that is DSL-bound and cannot be split. It gets
its own design review, starting from section 4.16.

## 8. The test list

Unless a case says otherwise, cases run on Windows (MSVC) and Linux (gcc). The TS cases also run on
Linux clang. Each stage turns its cases into gtest and pytest cases with these IDs.

### 8.1 Hazards (HZ, Stage 0)

| ID | Situation | Result |
|---|---|---|
| HZ-1 | `(set-attr ValueError.__cls_dict__ "init" (lambda (self m) self))` | `TypeError`, "built-in class is read-only"; a later `(ValueError "v")` is unaffected |
| HZ-2 | the same on a user class derived from `ValueError` | works, as today |
| HZ-3 | `(begin (set l (list 1 2)) (state-data-set "k" l) (set-nth l 0 9) (state-data-get "k"))` | `(1 2)` |
| HZ-4 | `(state-data-set "k" (list 1 (lambda (x) x)))`; the same with a generator, a native and a future nested in a dict | `runtime_error` naming the kind for each; the node is unchanged |
| HZ-5 | `state-delete` of a node while a sibling has expired but not been swept | only the node is gone |
| HZ-6 | one thread loops `state-delete` and re-create of a node while another `state-get`s its siblings (ASan, TSan) | no use after free, no race |
| HZ-7 | a chrooted process calls `state-watch`; then a process is submitted | its settings are read from the host's watch root |
| HZ-8 | `(fork)` in a pycvc program and in an Ariadne action | the running process forks: both continue, with 0 and the child's pid; the child has the parent's `owner`; `kill_owner` reaps it |
| HZ-9 | `exec_coordinator` control messages on a bus thread while the scheduler runs (TSan) | no race; the commands apply between steps |
| HZ-10 | `mt.multi_exec` at 8 threads under TSan | clean |
| HZ-11 | two `pycvc.Exec`s on two Python threads, raising and catching built-in exceptions and using `state-data-set` (Linux TSan) | correct results, clean |
| HZ-12 | `print` from 8 schedulers on 8 threads, 1000 lines each | every line whole |
| HZ-13 | a future awaited twice (`finish_async_call` reached twice) | one charge, one result |
| HZ-14 | `stop()` from another thread during `run()` on both schedulers (TSan) | clean; run returns |

### 8.2 Groups and the fork-join pool (GR, Stage 1)

| ID | Situation | Result |
|---|---|---|
| GR-1 | **The oracle.** G groups (2-32) x M processes mixing fib, exception loops, cycle-making closures, `set-attr` on own dicts, `state-set`/`get`/`delete` under the group's prefix, in-group `msg-send`/`msg-recv` with broadcast, fork, spawn, and `await` on inline-executor kernels. Each group run alone on one thread, then all on the pool at threads 1, 2, 4, 8, 16 | every group's per-process results, step counts, exit statuses and `tracked_objects()` after a final collection equal its solo run (time limits excluded) |
| GR-2 | GR-1 with `CVC_EXEC_FORCE_MOVE` and random pump budgets | the same |
| GR-3 | GR-1 with host-affinity groups mixed in | the same; host-affinity groups pumped only on the caller's thread (thread id) |
| GR-4 | a cycle made in group A while B collects | A's collection frees it; B's registry never sees it |
| GR-5 | a group destroyed after its last process; an `Exec` driven from 100 short-lived Python threads | its registry freed; nothing lost with the threads |
| GR-6 | `math.random` with a fixed seed in a group pumped by a different worker every pump | the stream equals a one-thread run |
| GR-7 | pids from 8 groups on one pool | unique and increasing app-wide |
| GR-8 | `pump_all` of 16 groups each with 0.1 s of work, threads 4, frame deadline 0.1 s | returns within about 0.1 s plus one pump; the skipped groups go first next frame |
| GR-9 | `pump_all` of groups whose last pumps total under 100 us | no helper woken (`extra.helper_wakes` 0) |
| GR-10 | a `host_only` native called in a worker-pumped group | NameError; a `host_proxied` native runs on the host thread and the process resumes at the group's next pump |
| GR-11 | a `host_proxied` native called inside a generator body on a worker | blocks that worker until the host thread runs it during the join; no deadlock |
| GR-12 | an exception escaping a pump on a worker | that group marked failed and its processes killed with "internal error"; other groups' results correct |

### 8.3 Continuous pool and wakeups (PL, Stage 2)

| ID | Situation | Result |
|---|---|---|
| PL-1 | the state word model: a post during release, a release during a post, two posts at once, in all orders (exhaustive, 2-3 actors) | no lost wake, no double queueing |
| PL-2 | the eventcount: a post between a worker's last look and its sleep, fuzzed 10^6 times | the worker wakes every time |
| PL-3 | everything parked; a host thread posts a message | the receiver runs within 1 ms |
| PL-4 | a group with a 5 ms sleeper and nothing else | it is pumped within 1 ms of the deadline; no worker spins meanwhile |
| PL-5 | one CPU-heavy group and one latency-sensitive group on one thread | the light group's wake-to-run latency stays under one slice (virtual time) |
| PL-6 | `post_control` kill from a host thread while the target's group pumps | the target takes at most one more step; `(kill p)` from a same-owner program in another group returns `#t` and the same holds |
| PL-7 | a program kills a pid in another owner's group | `#f`, nothing happens |
| PL-8 | `kill_owner` with the owner's processes in 4 groups, then the host frees the contexts (ASan) | every group acknowledged first; no use after free |
| PL-9 | a worker blocked 100 ms in an inline kernel | other groups keep running; the long-step monitor logs it |
| PL-10 | `~exec_pool` with 8 busy workers (ASan) | joins; no leak, no use after free |

### 8.4 Messaging across groups (MS, Stage 2)

| ID | Situation | Result |
|---|---|---|
| MS-1 | a send from group S to a channel homed at H with a waiter in H | delivered once at H's next drain |
| MS-2 | a send to H with no waiter, then a receive in H | the receive takes it from H's pending FIFO |
| MS-3 | a receive in R on a channel homed at H, then a send from S | delivered to R once |
| MS-4 | a broadcast at H with waiters in H, R1 and R2 | all three get it; each envelope is its own |
| MS-5 | a delivery to R in flight when R's timed wait expires | R raises or returns its timeout; the message is requeued at H; the next receiver gets it |
| MS-6 | the same when R's process is killed, or reaped | requeued, not lost |
| MS-7 | 8 groups x 1000 messages each to one channel | FIFO per sender group; none lost; none duplicated |
| MS-8 | an app-global key sent from a document group | homed at group 0; a receiver there gets it |
| MS-9 | a `#` tick channel in a document group | local; the host's posts reach the group |
| MS-10 | H's ingress at its cap | the sender's `msg-send` returns `{status: "error", reason: "ingress full"}`; `post_message` returns false |
| MS-11 | `msg-pending` on a remote-homed key | the home's count, at most one slice stale |
| MS-12 | **Semantics matrix**: for {same group, home remote, receiver remote, both remote} x {waiter present or absent, broadcast to M waiters, timed receive expiring with a message in flight, waiter killed, pending cap, chroot-resolved key, future channel}; deterministic mode | every outcome equals the single-group outcome, except the documented requeue order |

### 8.5 Isolated spawns and moves (IS, Stage 3)

| ID | Situation | Result |
|---|---|---|
| IS-1 | an Ariadne action `spawn`s a child that `msg-send`s on a document channel and `state-set`s (Stage 1) | the message reaches a resident; the write lands under the document's prefix; `kill_owner` reaps the child |
| IS-2 | a pycvc program `spawn`s a child that calls a registered Python function (Stage 1) | the child calls it; the parent's top-level definitions persist across `run()` |
| IS-3 | `(spawn-isolated "fib" ... (list 1 2))` | runs in a spawn group on another thread; `args` is a copy; it reports by message |
| IS-4 | `spawn-isolated` with an argument holding a closure | `runtime_error` naming the closure; no child |
| IS-5 | 32 isolated children of one owner, threads 8 | at most 8 spawn groups for that owner; children spread across them |
| IS-6 | moving a ready isolated process every 10 pumps (`CVC_EXEC_FORCE_MOVE`) | its results, step count, pid, inbox, signal handlers and budgets equal an unmoved run |
| IS-7 | a move of a parked process, one with a call in flight, one with a watch, one sharing an env | refused (`move` returns false); the process is unaffected |
| IS-8 | a moved process's intrinsic and async calls (thread ids, TSan) | served by the target group only |
| IS-9 | the isolation walk on a process holding a list another process also holds | refused |
| IS-10 | the owner's document is torn down with children in 4 spawn groups | `kill_owner` acknowledged; children gone before the contexts are freed |

### 8.6 Hosts (AR)

| ID | Stage | Situation | Result |
|---|---|---|---|
| AR-1 | 1 | the Ariadne host and runtime suites with `state_exec.threads: auto` and no document opted in | pass; every action and resident steps on the frame thread |
| AR-2 | 1 | 3 documents in group 0 via `drain_begin` / `exec_pump_frame` / `drain_end`; an action `(await (slow-op-async))` | resumes exactly one frame later; group 0 pumped once a frame |
| AR-3 | 1 | 4 isolated documents, each with a busy resident, threads 4 | residents step on different threads; render runs only outside pumps (a debug latch); a quick action completes in its drain |
| AR-4 | 1 | `pycvc.ExecPool(app, threads=8).run_all` of 64 fib programs; the same with Python natives | correct results; scales for pure DSL; serialized on the GIL with Python natives |
| AR-5 | 3 | a document with `parallel_actions: 4`, 8 actions in one frame | actions ran on several threads; each completed in its drain; `doc_scope_` verbs ran on the host thread |
| AR-6 | 3 | a pycvc `Exec` program spawning 8 isolated children that report by message | correct; children on several threads; `Exec.run` wakes on their messages |
| AR-7 | 2 | `exec_coordinator` with a pool attached: 16 local jobs | each in a job group; results correct; control from a bus thread is TSan-clean |

### 8.7 Sanitizers (TS)

| ID | Situation | Result |
|---|---|---|
| TS-1 | Linux clang `-fsanitize=thread`: state_exec tests, GR, PL, MS, IS and ST at threads 4, the Ariadne runtime tests at threads 4, the TSan pycvc set | clean |
| TS-2 | ASan and UBSan jobs on the same set; MSVC `/fsanitize=address` on GR and IS | clean |
| TS-3 | Windows debug build with `CVC_EXEC_VERIFY` on GR, MS and IS (MSVC has no TSan) | no ownership assertion |

### 8.8 Stress (ST)

| ID | Situation | Result |
|---|---|---|
| ST-1 | A random program generator over 16 groups, threads 8, 10 minutes nightly, under TSan and in Release. It mixes spawn trees, fork, cross-group sends and receives with timeouts, kill and pause storms within and across groups, sleeps, `state-get`/`set`/`delete` on disjoint and shared subtrees, async calls with random latency and timeouts, and `set-attr` on received envelopes | Every message delivered once, queued, or dropped by a cap (counted). Every kill lands within one step of being applied. No budget exceeded by more than one step (16 with slices). The collector never clears a value a host still reads. Memory flat after warmup |
| ST-2 | ST-1 with `CVC_EXEC_FUZZ` over 1000 seeds | the same invariants |
| ST-3 | group churn: 10k groups each running one program, destroyed while kernels are in flight (ASan) | thread and handle counts flat; nothing leaked |
| ST-4 | a one-hour soak: Ariadne-shaped frames with 4 isolated documents, document churn, isolated children and moves | registries, ingress queues, the pid directory and spawn groups return to their baseline |

### 8.9 Determinism (DT)

| ID | Situation | Result |
|---|---|---|
| DT-1 | deterministic mode, the same program set twice, threads 4 | identical per-group step traces and identical cross-group message order |
| DT-2 | `threads = 0` against today's scheduler on the whole suite | identical results and traces |
| DT-3 | deterministic mode against threaded mode on MS-12 | the same outcomes |

### 8.10 Slices (SL, Stage S)

| ID | Situation | Result |
|---|---|---|
| SL-1 | the whole scheduler suite at `slice_steps` 1 | passes; a recorded per-step trace of a 3-process program is identical to the legacy loop's |
| SL-2 | `max_steps` 1000 on an endless loop, `slice_steps` 256 | killed at exactly 1000 steps |
| SL-3 | `max_time` 0.05 s on an endless loop | killed; overshoot at most 16 steps |
| SL-4 | a resident with `max_step_time` 0.05 s whose native loops | killed by its per-step deadline |
| SL-5 | process A kills B in one group while B is mid-run | B takes no further step |
| SL-6 | a watched node changes during a run | the handler fires at the watcher's next run |

## 9. Risks and open questions

### 9.1 Risks

1. **Coarse grain.** One document, one `Exec` program or one fork family gets 1.0x until Stage 3
   (`spawn-isolated`, parallel actions) or Stage 4. The shipped Ariadne hosts drive one document, so
   Stages 1-2 give them no speedup. Real DSL-level parallelism needs programs written around
   `spawn-isolated` and messages.
2. **Invariant G is a convention.** It is enforced by the isolating copy, debug tagging and TSan,
   not by the type system. A host that hands one compound value to two groups, a native that
   captures per-group state, or a non-thread-safe `data_object` becomes a data race in release
   builds. TSan runs on Linux only, while development is on Windows.
3. **Host code that assumes the main thread.** If most providers and subscribers turn out
   `host_only` or not thread-safe, isolated documents stay on the host thread and gain nothing. The
   audit is real work, and a missed case is a race in replication code outside state_exec.
4. **The state tree** stays one shared structure with per-segment locks and emission up the parent
   chain. State-heavy workloads may scale only 2-3x, and unchrooted ones about 1x.
5. **The router is the one new protocol before Stage 4.** Its corner cases (timed waits on remote
   homes, kills between subscribe and deliver, caps) are where bugs will be. The semantics matrix
   and deterministic mode are the mitigation.
6. **Per-group overhead.** Each group costs a scheduler, a lazily built async host and an extended
   builtins scope. Floods of tiny groups lose to reused groups, and `ExecPool` reuses by default.
7. **The behaviour changes of Stage 0** (frozen classes, data-channel copies and refusals,
   single-node delete, fork) are unconditional. Programs or tests that relied on them break.
8. **Thread oversubscription.** Group workers, the kernel pools and `computePool` together can
   exceed the core count. They must be sized together, and on hybrid parts an E-core worker sets the
   frame time in fork-join mode.
9. **Measurement.** The scaling data is one rep on a hybrid CPU with 1.6x per-thread inflation at 8
   threads. Defaults past 8 threads, and whether the allocator is the main cause, need P-core-only
   and allocator runs first.

### 9.2 Questions for the user

1. **Q1. The model.** All three judges ranked groups-first (E) first, by 119 points against 107 for
   isolates. This document adopts it and keeps option A as Stage 4, to be built only on evidence.
   Confirm, or say if parallelism inside one scheduler is needed sooner than Stage 3's
   `spawn-isolated` would give it.
2. **Q2. Built-in exception classes.** Freeze them (proposed; a behaviour change), or give each
   group its own copies? Copies keep mutation, and stay catch-compatible because `is_instance`
   matches by name chain (`exceptions.cpp:183-199`).
3. **Q3. Callables in crossing values.** Refuse closures, natives, generators, environments and
   futures at any depth (proposed, D's structured clone), or copy closures with their environments
   (C)?
4. **Q4. Spawn.** What should a spawned child inherit?
   - The proposal is the parent's intrinsics, chroot, owner and channel policy, through a host
     callback.
   - Should a pycvc child also inherit the `Exec`'s Python natives (proposed: yes)?
   - Should plain `spawn` ever be placed on a spawn group automatically (D's `auto_spawn`), or only
     `spawn-isolated` (proposed)?
5. **Q5. Cross-group control from programs.** Allow `kill`, `pause`, `ps` and `inspect` across
   groups of the same owner only (proposed), not at all, or across all groups as today?
6. **Q6. The router.** Should app-global channels be homed at group 0 (proposed), or by the first
   receiver (D)? And should a remote receive be allowed at all (proposed), or should a receive be
   confined to channels the group is home to (simpler, no subscription protocol)?
7. **Q7. Ariadne.**
   - Does any shipped host (tvision desktop, conference room, holon participants) run several
     Runtimes in one process? That decides whether Stage 1's Ariadne half is worth building before
     Stage 3.
   - Which providers and subscribers must stay on the host thread? Each needs its owner's answer.
   - If too many must, should the deferred-notification sink (rejected in 3.4) be built after all?
8. **Q8. Defaults.** `state_exec.threads` 0 (proposed) or `auto`? Workers on P-cores only
   (proposed), or all cores?
9. **Q9. Kernel pools.** Split `background_cpu` and `background_io` once group workers compete for
   cores (proposed for Stage 2)? That reverses settled question 4 of `docs/STATE_EXEC_EXCEPTIONS.md`.
   The alternative is one pool with a cap on blocking kernels.
10. **Q10. Slices.** Should Stage S's default `slice_steps` become 256 once the suites pass, or stay
    1 with hosts opting in?
11. **Q11. Pids.** One pid source per pool (proposed) keeps pids unique app-wide and stable across
    moves. Should pids also widen to 64 bits now? Today's `int` wraps after 2^31 processes, about a
    year at 60 actions a second.
12. **Q12. `ExecPool`.** Reuse pooled `Exec`s, so top-level definitions persist per pooled `Exec`
    (proposed default), or give each program a fresh env?
13. **Q13. `exec_coordinator`.** Move its local path from the sync scheduler to async groups
    entirely? The sync scheduler has no ingress, and it busy-waits on parked processes before
    `perf/sched`.
14. **Q14. Setting names.** Confirm:
    - `state_exec.threads`, `state_exec.cpu_set`, `state_exec.deterministic`,
      `state_exec.groups.ingress_cap`;
    - `ariadne.exec.isolate_documents`, `ariadne.exec.parallel_actions`;
    - `spawn-isolated`, `pycvc.ExecPool`.

## 10. Prior art and lessons

The survey is `prior-art.md` in the design scratchpad. These are its lessons that shaped this
design.

- **CPython PEP 684 and PEP 734.**
  - A GIL per interpreter took years of moving global state into the interpreter. Shared built-in
    singletons were made immortal (PEP 683).
  - PEP 734 copies values between interpreters, and `InterpreterPoolExecutor` is a thread pool with
    one interpreter per worker.
  - Lessons: groups are PEP 684 for state_exec, with a short global-state list (Stage 0); the
    exception classes are the PEP 683 case; `ExecPool` is the executor. PEP 703's 5-6%
    single-thread cost for per-object locks is why option F was rejected.
- **V8 isolates and `Locker`.**
  - An isolate is single-threaded but not tied to a thread. Any thread may enter it under `Locker`.
  - Lesson: that is the pump token, and a group moving between workers.
  - `postMessage` transfer (move and detach) is attractive, but `use_count()==1` is only a
    heuristic here (`types.h:86-96`), so values are copied.
- **Racket.**
  - Places came first: isolated instances with copied messages. Shared-memory parallel threads came
    years later (Racket 9.0, 5.2-5.4x on `fib 40` at 8 threads on a 4P+4E part, close to
    `mt.independent`).
  - Futures run until a blocking operation, then wait for the touching thread.
  - Lessons: the stage order; `host_proxied`; and that shared subsystems (file I/O locks capped some
    workloads at about 1.6x) set the ceiling, as the state tree may here.
- **Lua Lanes and effil.**
  - Separate states per thread, copying channels, and "deep userdata" for shared native objects
    (`data_object` here). It is the cheapest route to parallelism in an embedded VM, and state_exec
    already has all three parts.
  - effil's second collector for shared tables is the warning: once mutable containers are shared,
    a cross-thread collector follows. That is why invariant G has no exceptions.
- **Erlang/BEAM.**
  - It has per-scheduler run queues, reductions as preemption, a heap and GC per process, and
    asynchronous signals handled by the receiver.
  - Separate dirty CPU and I/O pools take native work, and a long-schedule monitor reports offenders.
  - Lessons: Stage S slices, the split kernel pools, the long-step monitor, a collector per
    isolation unit with no global pause, and Stage 4's receiver-applied requests.
- **Go and Tokio.**
  - Per-worker queues, an injection queue, `runnext` (a LIFO slot disabled after 3 uses), wakes
    throttled to one searching thread, and a P handed off from a blocked thread.
  - Lessons: the `next` slot, the eventcount wake protocol, the adaptive helper wake and the
    compensating worker.
  - state_exec already has what Go 1.14 had to add, a safe point at every step.
- **Akka and CAF mailboxes.** An actor is scheduled by a CAS on a status word, and a "notified" bit
  makes a message arriving during release reschedule the actor. Lesson: the group state word of
  4.4, and Stage 4's domain word.
- **Ruby Ractors and Pony.**
  - Their vocabulary: unshareable values are copied, shareable ones deeply frozen, `move` detaches.
  - Class objects were Ractor's loophole, as the exception class dicts are here.
- **Guile Fibers.** A scheduler per thread with stealing, which breaks on state tied to a kernel
  thread. Here that is the random stream and `t_park_forbidden` (B20, B26), both handled per group.

Sources: the survey's links. Erlang: `erl_vm.h`, `erl_nif`, ProcessManagementOptimizations. Go:
`proc.go`, Vyukov's scheduler design, proposal 24543. Tokio: "Making the Tokio scheduler 10x
faster". Python: PEPs 683, 684, 703, 734. Racket: the parallelism guide and "Parallel threads in
Racket 9.0". Lua: Lanes, effil. V8: the embedder's guide, `v8-locker.h`. Guile: the Fibers manual.
Ruby: the Ractor docs. Pony: the ORCA paper.
