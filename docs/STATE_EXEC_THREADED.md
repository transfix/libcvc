# state_exec: running programs on several threads

Status: design, adopted 2026-10-26 with the decisions in section 9.2. Stage 0, the process-global
hazard fixes (section 7), is built. Stage 1 is built, in two parts (S1-Q12): Stage 1a, the
execution groups, the fork-join pool and pycvc's `ExecPool`; and Stage 1b, Ariadne's half (one
pump per frame, isolated documents, providers and posts on the caller's group, the init lane's
own host, the teardown reap, the subscriber audit). The later stages are not built. Tracked in the
`state-exec-perf` workstream, as its third track. The first two are the benchmark suite
(transfix/libcvc#834) and the single-thread optimizations (branches `perf/sched`, `perf/async` and
`perf/eval`).

Locks on the state tree and process priority changes are designed in
[`docs/STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md). Section 4.19 says how they behave across groups,
and section 4.20 adds Stage M, moving programs between hosts, scheduled last.

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
- **Stage 0's text** (4.3, 4.5, 4.8, 4.9, 4.11, 4.12, 5 items 1-5, 7 and 8.1) describes the code
  as built, and names functions rather than lines. Its plan, with the six decisions the user
  approved on 2026-11-01 (S0-Q1 to S0-Q6, listed in 9.2), is `threaded-impl/S0.md` in the
  `state-exec-perf` workstream.
- **Stage 1a's text** (4.1, 4.2, 4.4's Stage 1 and deterministic mode, 4.5, 4.7, 4.9's async
  host, foreign-future check, host-thread natives, host-call queue and `on_complete`, 4.10's
  budgets and `math.random`, 4.12's `exec_group`, `exec_pool`, app and pycvc parts, 4.13, 4.14,
  4.17, 5 items 7-9, 6.3, 7's Stage 1 entry and 8's Stage 1 cases) also describes the code as
  built, naming functions. Its plan, with the twelve decisions the user approved on 2026-11-07
  (S1-Q1 to S1-Q12, listed in 9.2), is `threaded-impl/S1.md`. Stage 1b's Ariadne text (4.6's
  Stage 1, 4.8, 4.9's providers and posts, 4.11 item 7, 4.12's Ariadne part, 4.15's spawn
  context in Ariadne, 4.17's Runtime teardown, 5 items 10 and 11, 7's Stage 1b entry and 8's
  Stage 1b cases) describes the code as built too, naming functions.
- **The benchmark suite** and its documentation are `src/cvc/tests/state_exec_bench.cpp` and
  `docs/STATE_EXEC_BENCH.md`, on master since #834 (bc4761bf); this design was first written
  against the `feat/state-exec-bench` branch.
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
7. **Locks and priorities hold across groups.** A DSL lock (`with-lock`,
   [`docs/STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md)) excludes the same way on one thread, across
   `Exec`s and across groups. A process's priority acts inside its group and weights its group in
   the pool (4.19).
8. **Programs can move between hosts, last.** Stage M (4.20) moves a running program to another
   host, a server or a client. It is scheduled last, and needs only Stage 0 and the locks track's
   single-machine phases (L1-L3), hardened.

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
  Stage 0's control queue and atomic stop flag, and the locks track's lock waits, lock posts,
  wakeup-based `idle_wait` and priority overrides ([`docs/STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md)
  8.7). It gets no groups or workers.
- **Cross-node distribution.** `exec_coordinator` keeps its own design. Section 4.12 says what it
  gains, and Stage M (4.20) what moving one program between hosts needs. Cross-host locks are
  the locks design's phase L7.
- **A concurrent state tree.** This design makes the DSL's use of the tree safe (pinned lookups, no
  tree-wide sweeps). It does not redesign the tree's locking.
- **The single-thread optimization tracks.** This design assumes `perf/sched` (run queue, timer
  heap, one drain and no coroutine frames a step, `wait_for_work`), `perf/async` (persistent
  background pool, concurrent fan-outs) and `perf/eval` (cheaper steps) land first (section 2.4).

## 2. Today

### 2.1 What runs where

- **Schedulers.** Two schedulers step a `stackless_evaluator`, one step at a time:
  - the sync `scheduler` (the CLI, tests, `exec_coordinator`, and Ariadne's `init:` lane, which
    runs each script to completion on a private one);
  - the `async_scheduler`. Ariadne uses the app-wide one (`app::exec_scheduler()`,
    `app.cpp:1024-1038`). pycvc's `Exec` builds its own (`pycvc_exec.cpp:410-423`).
- **The step loop.** Each step selects a process, sets the scheduler's current process, arms the
  deadline and allocation guards, steps, and accounts (`async_scheduler.cpp:152-300`,
  `310-322`).
- **Async calls.** An async call marshals on the scheduler thread, then runs its kernel on the
  app's background pool or a lane. Its completion comes back through the scheduler's ingress, the
  only thread-safe entry (`async_scheduler.cpp:516-556`;
  [`docs/STATE_EXEC_EXCEPTIONS.md`](STATE_EXEC_EXCEPTIONS.md) 3.1).
- **Ariadne.** It pumps the app scheduler from every `Runtime::drain` (`ariadne.cpp:2021-2023` ->
  `1688-1706`). The sequence is `drain_ingress`, then `wake_awaiting`, then
  `sync_run(200000, 0.1)`, then the sweep. Every document shares that one scheduler, one pid space
  and one async host (`app.cpp:1041-1047`).
- **pycvc.** `Exec.run` pumps the `Exec`'s private scheduler.
  - It releases the GIL around each slice (`pycvc_exec.cpp:594`).
  - When its program is parked it sleeps 1 ms (`pycvc_exec.cpp:648`), which is about 15.6 ms on
    Windows. (Since the scheduler track, #842, it waits on the scheduler's wakeup instead,
    `wait_for_work`, which an ingress post or the next timer ends; since the locks track's L3, a
    lock post too.)
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
blocker cannot occur until several threads step one group. Stage 0 has fixed B7, B10, B13, B14,
B17, B18 and B26's `print` half (section 7); their rows describe the code before it.

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
| B27 | Spawned children get the bare base env, with no intrinsics, no chroot and no owner, so they cannot `msg-send`, touch the tree or report a result | `intrinsics.cpp:420-431` -> `stackless_evaluator.cpp:53, 66`; `builtins.cpp:855-938`; intrinsics come only from `register_intrinsics` (`intrinsics.cpp:1062`) | Stage 1; its single-scheduler part landed with the locks track's L1b |
| B28 | `(fork)` forks `ctx->pid`, which hosts leave at -1 or 0, not the running process; a forked child does not inherit `owner` | `intrinsics.cpp:437`; `ariadne.cpp:1645-1660`; `pycvc_exec.cpp:416`; `async_scheduler.cpp:957-976` | Stage 0, or the locks track's L1 if that lands first (a bug today) |
| B29 | With several documents Ariadne pumps the shared scheduler once per document per frame, so `wake_awaiting` runs D times a frame and "an await crosses exactly one frame" does not hold | `ariadne.cpp:2021-2023`, `1694-1697` | Stage 1. The `wake_awaiting` half is fixed: it now runs once per app frame however many documents pump (`Runtime::Impl::starts_frame`) |
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

These branches, and the locks track, are in progress. The stages below build on them and do not
repeat their work.

| Branch | What it does | Blockers it settles |
|---|---|---|
| `perf/sched` | A process table with a run queue, a timer heap and waiter indexes, in both schedulers. One clock reading a step. One ingress drain a step, an atomic flag when empty. The evaluator step called directly, with no coroutine frames. `wait_for_work(deadline)`, woken by posts, completions and timers. Watch polling only when the tree changed (an atomic counter fed by the root's change signals). pycvc's `Exec` waits for work instead of sleeping 1 ms | B19; the O(P log P) selection; the per-step ingress mutex |
| `perf/async` | Persistent background-pool workers, with anonymous tasks and no thread per task. A fix for the stall and hang. Concurrent `parallel_for` fan-outs side by side. Less per-call work in `async_host` | B23, B24 |
| `perf/eval` | Cheaper steps: special forms by switch, a closure body built once, call scopes recycled per thread, one hash per lookup | none; a faster step makes per-pump costs relatively larger |
| locks track ([`docs/STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md) L0-L3, built, on `perf/sched`, `perf/eval` and `perf/async`) | Atomic single-node natives. `with-lock` on tree paths, a lock home per scheduler, lock waits as a process-table wait kind. The spawn context on one scheduler (B27's first part, L1b). `set-priority`, `get-priority`, the policy from `state_exec.policy`, priority inheritance. One app lock manager, under one mutex in shared mode (`state_exec.locks.threading`), shared by every scheduler and thread (L3): several homes, each with a lock port whose posts end its scheduler's `wait_for_work` or `idle_wait`; pycvc's `Exec`s and Ariadne's `init:` lane on the app's manager | B28 (L1); B27's single-scheduler part (L1b). Stage 1 extends L1b's spawn context to groups and gives each group a lock home over the app's manager (4.19) |

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
| C (all judges) | An isolating copy at every crossing between groups. Lists and dicts are copied; closures, natives, generators, environments and futures are refused at any depth (D's structured clone) rather than deep-copied (C's choice; Q3) | Stage 0 for `state-data-set`; Stage 1-3 for posts and arguments; 4.3 |
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
  - Host affinity plus an audit of subscribers is used instead (4.8). The sink is reconsidered only
    if the audit leaves most documents on the host thread (Q7).
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

**An execution group** (`exec_group`, `exec_group.h`; Stage 1a) is:
- one `async_scheduler`, whose global env extends a base env (4.2);
- its own async host, built on first use (`exec_group::host()`);
- its own cycle registry and `math.random` stream, which its scheduler binds (4.7, 4.10);
- its options (`exec_group::options`): a name, an owner tag, an affinity (`any` or `host`), the
  app, a watch root, the base env and an optional `math.random` seed.

**The pump token.** A group is pumped by at most one thread at a time.
- `exec_group::pump(budget)` is the driver sequence `Exec` runs: `drain_ingress`, `wake_awaiting`,
  then `sync_run_steps(budget.steps, budget.seconds)`. A `pump_budget` holds the steps and the
  seconds (infinity: no time limit). The `pump_result` gives the steps the pump took, the processes
  `wake_awaiting` woke, and whether a C++ exception escaped it (the pool then fails the group,
  4.17).
- The token is the group's scheduler's pump owner: an atomic thread id on the scheduler, held
  through `async_scheduler::pump_owner_scope` (S1-Q4). `pump()` takes it by a compare-and-swap from
  unset, with acquire, and releases it with a release store. That handoff is the release/acquire
  edge a scheduler needs when it changes threads (the audit's model (c)). A pump that finds the
  token held by another thread is a bug of the pool or the host: a debug build asserts, and the
  pump throws the pump-owner error below.
- The owner's rule is "no owner, or me". A `pump_owner_scope` takes an unset owner, nests inside a
  scope of the same thread, and restores what it found. Against another thread's owner its
  constructor throws `std::runtime_error` "scheduler: another thread is pumping it"
  (`kPumpedElsewhereError`, `scheduler.h`). The sync `scheduler` has the same owner. Each driver
  holds one while it drives: a group's pump (the pool takes the token per pump), pycvc's
  `Exec::run` until it returns, and pycvc's raw pump calls, per call (4.12). `pumped_elsewhere()`
  tells any thread, in one relaxed load, whether another thread holds it.
- Debug builds, and release builds run with `CVC_EXEC_VERIFY=1`, check the rule in both
  schedulers' public mutators (`execute`, `kill`, the async `kill_owner`, `pause`, `resume`,
  `send_signal`, `fork`, `set_priority`, `set_priority_checked`, `set_priority_floor`, `reap`,
  `restore`, `register_watch_handler`, `set_watch_root`): a call while another thread holds the
  owner prints the error and aborts, since it is a host's race. `post_message`, `post_control`,
  `stop` and `is_running` are thread-safe and unchecked.

**The pump scope.** A pump binds the group as `exec_group::current()` on the pumping thread. The
group's cycle registry (4.7) and random stream (4.10) are bound by its scheduler itself: a
scheduler given a registry or a stream (`set_cycle_registry`, `set_random_stream`) binds what it
was given on every entry that can run DSL or track (`step`; `run_loop`, so `run`, `sync_run` and
`sync_run_steps`; `sync_step`, `drain_ingress`, `wake_awaiting` and both `restore`s), one store and
one restore per call and nothing per step. So whoever drives a group's scheduler tracks into its
registry: the pool, `cvc::pump_exec_frame`, `wait_sliced_job`, `Exec::run` or pycvc's raw calls. A
scheduler given neither binds nothing, as before.

**The pool** (`exec_pool`, `exec_pool.h`; one per app, `app::exec_pool()`, built at its first call
like `exec_scheduler()`, 4.12):
- **Size.** `pool_options::threads` persistent workers besides the calling thread.
  - 0 (the default) means every pump runs inline on the calling thread, which is today's
    behaviour and the only mode in single-threaded wasm.
  - `auto` (`pool_options::auto_threads`) means the logical CPUs in the highest efficiency class
    minus one with `cpu_set::performance` (Q8), and `hardware_concurrency() - 1` with
    `cpu_set::all`. Every count is clamped (4.14), and `threads()` and `stats()` report the
    workers running.
  - Workers pump only over a shared lock manager (S1-Q1, 4.13).
- **Workers** are `boost::thread`s created with an explicit 8 MiB stack
  (`boost::thread::attributes::set_stack_size`; a `std::thread` cannot be given one). The
  native-stack guard computes its bound per thread (`native_stack.cpp`), and deep natives need the
  room `kNativeStackReserve` assumes (`data_job.h`). Each worker applies the process's thread class
  at its start (`detail::apply_process_class_here`, as the background pool's workers do) and opts
  in to the collection of its own cycle registry at its exit (4.7). That registry should stay
  empty, since a group's objects go to the group's registry: a debug build asserts it at the
  worker's exit.
- **Placement** (`cpu_placement.h`, shared with the bench's `--pin`). Worker i goes on CPU i of
  `placement_cpus(placement)`, and a worker beyond the list floats: CPU sets on Windows (resolved
  at run time; processor group 0), `sched_setaffinity` on Linux (efficiency classes from
  `cpu_capacity`, or the hybrid `cpu_core` list), nothing on macOS or Emscripten. The list puts the
  faster class first; within each class, the first logical CPU of every physical core, then their
  SMT siblings, so the first workers never share a core, with the core logical CPU 0 is on (where
  most interrupts land) after the others; last of all comes the driver's CPU, a performance core
  off CPU 0's, which `pin_current_thread(-1)` (the bench's `--pin auto`) takes, so a pool with
  fewer workers than CPUs never uses it. `cpu_set::performance` takes the highest efficiency class
  only; with `cpu_set::all`, P-core workers take groups from the front of the LPT list and E-core
  workers from the back (4.4).
- **Thread class** (declared in `pool_options::worker_class` and `exec_group::options` for the
  locks track's L4 and Stage 2; not applied in Stage 1). `pool_options::worker_class`
  (`background`, `utility` or `normal`; default `normal`, never above) sets each worker's OS
  priority through `set_current_thread_class`
  ([`docs/STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md) 7.6). It is best effort, and every result is
  logged. A group's `thread_class` applies while that group is pumped; a worker switches only when
  the class changes. On Linux an unprivileged thread cannot raise its nice value back, so a shared
  worker is demoted for a group only when raising back is known to work (`CAP_SYS_NICE`, or
  `RLIMIT_NICE` allowing 0); otherwise per-group classes apply only to dedicated workers, and groups
  with a lower class are pinned to workers already at that class. On macOS, `utility` and
  `background` steer workers to E-cores, against Q8. No DSL intrinsic reaches it.
- **Lifetime.** Workers never start or stop per task. `~exec_pool` stops them after their current
  pumps; the pool owns no group (4.17).

**Two pool modes.**
- **Fork-join** (Stage 1; Ariadne's mode and `ExecPool`'s):
  - `pump_all(groups, budget, deadline)` pumps each group once, in parallel. The caller pumps the
    host-affinity groups itself and helps with the rest. It returns when all are done (4.4).
  - Outside a `pump_all` no worker touches any group, and no token is held. Host code (sweeps,
    `get_process_info`, `reap`, `kill_owner`) runs as today, between pumps.
  - Render never overlaps a pump, so it stays a pure reader ([`docs/ARIADNE.md`](ARIADNE.md)
    section 3).
- **Continuous** (Stage 2; pycvc servers, `exec_coordinator`, headless hosts):
  - Attached groups are pumped whenever they have work, a slice at a time (section 4.4).
  - A host call that touches a group's scheduler takes the token (`with_group`), or goes through a
    thread-safe post (`post_message`, `post_control`).

### 4.2 Groups

**Who makes a group.**

| Group | Made by | Affinity |
|---|---|---|
| Group 0 | `app::exec_scheduler()` itself; for pycvc, each plain `Exec`'s private scheduler. Neither is an `exec_group`: each has its own registry and random stream, which its scheduler binds (the app's in `app::exec_group0_state`, declared before the scheduler and its async host so it goes after both; an `Exec`'s in its `ExecImpl`, before its scheduler and envs), and `app::exec_pump_frame()` pumps the app's as a host-affinity group (4.12) | host |
| A host's group | a C++ host: `exec_group(options)`, pumped by `app.exec_pool().pump_all(...)`, or registered as a frame group (`app::register_frame_group`) for `exec_pump_frame` (Stage 1a) | as its options say |
| An `ExecPool` group | `pycvc.ExecPool` (Stage 1a): one per pooled `Exec`, owned by it | `any` |
| A document group | an Ariadne `Runtime` whose document `ariadne.exec.isolate_documents` isolates (Stage 1b, 4.12), registered as a frame group | `host` when the document's env holds a `host_only` native or its host declared a thread-affine prefix at, above or under its prefix (4.8); else `any` |
| A job group | `exec_coordinator::submit_local` with a pool attached (Stage 2) | `any` |
| A spawn group | the pool, for `spawn-isolated` children (Stage 3); up to `threads` per owner, made lazily | `any` |
| An action group | an Ariadne document with `ariadne.exec.parallel_actions: k` (Stage 3) | `any` |

**What a group shares with other groups.** Only frozen or immutable data:
- the frozen stdlib scope;
- the process-wide frozen builtins scope (below);
- the 19 built-in exception class dicts, read-only since Stage 0 (5 item 1);
- strings, numbers and bytes, which the variant holds by value (`types.h:154-155`);
- read-only `data_object` payloads ([`docs/STATE_EXEC_EXCEPTIONS.md`](STATE_EXEC_EXCEPTIONS.md)
  3.7);
- the state tree, through its own locks (section 4.8);
- the app's lock manager (`app::lock_manager()`), synchronized internally by one mutex in shared
  mode (4.19);
- the app's pid source (4.5);
- the kernel pools.

**Lightweight groups** (B16, built in Stage 1a). A scheduler built the old way makes its own
`make_default_environment()`, about 150 allocations (`flood.small env=own`).
- `frozen_builtins(app)` (`builtins.h`) builds that default environment once per app, in an app
  slot, and freezes it (`environment::frozen`, so a `set` on it throws). It keeps a stdlib scope of
  its own as its outer. Its natives are safe on any thread: `print` takes stdout's own lock,
  `generator`, `next` and `collect` reach the bound registry and the step's data scope,
  `math.random` the bound stream (4.10), and `random-token` the OS generator.
- A group's scheduler is built with `async_scheduler(policy, base)`, whose global env is
  `environment::extend(base)`: `options::base` when the host gives one, else
  `frozen_builtins(app)` (a group with no app freezes a default environment of its own).
  A program's `set` binds in its process's own run env, so nothing a program writes reaches the
  base. A base several groups share must be frozen, as `frozen_builtins(app)` is: a `set` of a name
  bound in an unfrozen base writes it there, from whichever thread pumps the group. An unfrozen
  base that one group owns is allowed (the bench's `driver=group base=own` passes one), so nothing
  refuses it; the rule is documented, not checked.
- Hosts keep registering their own natives into envs of their own over the base, as pycvc's
  pooled `Exec`s do (`natives` is `extend(frozen_builtins(app))`).
- Only `exec_group`s and pooled `Exec`s use the frozen base. The app scheduler, a plain `Exec`,
  Ariadne's action envs and every bare scheduler keep their own default environments, so no
  existing host changes.

**Teardown.** `~exec_group`, in this order, each step catching and logging what it throws (the
destructor never throws):
1. takes the token, waiting for a pump in progress (no pool holds a group's token outside a
   `pump_all`), then closes its lock port (`async_scheduler::close_lock_port`, which closes the
   home's port only if it has one), so no lock post lands from here on, and none calls `wake(g)`
   on a dying group in Stage 2 (4.19; [`docs/STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md) 9.2). A
   test hook runs here (`options::teardown_hook_for_test`, GL-2);
2. kills every process that has not finished, which settles its async calls as cancelled;
3. shuts its async host down, if it built one, waiting for kernels in flight, as `~Exec` does;
4. purges the group's entries from the host-call queue of the `pump_all` it belongs to, unrun,
   waiting for one its caller is running (4.9);
5. reaps every process.

Then its members go in reverse order: the async host, the scheduler and its envs, and last the
registry, whose deleter collects it under its own binding and frees it (4.7). A group is not
destroyed inside a `pump_all` that holds it, nor from a `host_proxied` body or a deferred
`on_complete` callback (debug assertions; a body destroying its own group stays undefined in a
release build). Destroying a group from inside its own pump, from a native or an inline callback
of its own process (`exec_group::current()` is the group), aborts in every build, since the
teardown would free the scheduler under its run loop (`ExecGroupDeathTest`).
An app's groups are destroyed by their owners before the app: `~app` asserts that none is left
(`detail::live_groups`), then stops the pool, before its async host shuts down and its background
pool, on which the groups' kernels ran, closes (4.17).

### 4.3 Ownership and values

**Invariant G.** A list, dict, unfrozen environment, closure, generator, async-call future or
deep-data job's held value is reachable from the processes of one group only.
- Anything reachable from two groups is frozen, immutable by type, or a read-only `data_object`.
- Inside a group, sharing is exactly today's: fork's shared frames, the aliased broadcast
  envelope, a host env used as the global env, generators. One thread at a time steps the group,
  so this is safe.

**The isolating copy.** `make_copy_job`'s default mode (`copy_mode::share`) is not an isolation
boundary: it copies lists and dicts and shares everything else. Stage 0 added an isolating mode,
`make_copy_job(v, copy_mode::isolate, who)` (`data_job.cpp`, `copy_walk`). It is D's structured
clone:
- strings, bytes, numbers, symbols and nil are values, so they are copied as values;
- lists and dicts are deep-copied, keeping their sharing and cycles, as the share mode's copy
  does;
- an ordinary `data_object` is shared, under its read-only contract;
- a built-in exception class dict is shared, in both modes: it is read-only (5 item 1). So a
  built-in exception's instance is copied and keeps its class. A restored exception's built-in
  class is the built-in dict again (the codec relinks it), so it is shared too;
- anything that runs code or holds a process's state raises `runtime_error`, at any depth,
  naming the crossing and the kind: "`who`: cannot store a `kind`" ("an" before a vowel), plus
  " (nested in a list)" or " (nested in a dict)" below the root, for example "state-data-set:
  cannot store a closure (nested in a list)". The kinds are `closure`, `native function`,
  `generator`, `future` (a future's dict or its `async_call` handle), `lock form` (`with-lock`'s
  form, which holds its context's raw pointer), `lock wait` (a lock wait's handle), and `object
  instance` (S0-Q4). An object instance is known by its shape, the one `defclass` and
  `make_exception` build: a dict whose first key is `__class__`, holding a dict whose first key
  is `__name__` and which is not a built-in class. Any other dict is data, a `__class__` key or
  not; plain data of exactly that shape (parsed JSON `{"__class__": {"__name__": "P"}}`) is
  refused too. An environment is reachable only through a closure or generator, so it needs no
  kind of its own. This replaced `state-data-set`'s check of a top-level callable.
- The copy runs as a sliced data job, charged to the process that makes it, as async arguments
  are (3.7). A refusal in any slice throws before anything is written. The memo of the lists and
  dicts already copied holds each source until the copy ends, in both modes, so a list another
  process drops between two slices cannot be taken for a new one allocated at its address.

**Where values cross a group boundary, and what crosses.**

| Crossing | What happens | Stage |
|---|---|---|
| `state-data-set` | stores an isolating copy; `state-data-get` keeps its copy on the way out (`intrinsic_state_data_get`, a share-mode copy) | 0 (built) |
| A host `post_message` to another group | the host hands the value over (moved in); the router makes an isolating copy for each extra receiving group | 1-2 |
| A cross-group `msg-send` | the payload is a string or bytes (`as_string_or_bytes`, `intrinsics.cpp:966`); each receiving group gets a fresh 4-entry envelope | 2 |
| `execute(value_t)` into a group | `exec_group::execute` copies the AST's lists and dicts, keeping their sharing, since `quote` returns the AST's own node; it copies always, as the caller's reference is always there (4.12). `async_scheduler::execute` is unchanged. Scripts are read inside their process already | 1a (built) |
| `ExecPool.run_all` inputs and results | strings in; results rendered as strings, as `Exec.run` renders them | 1a (built) |
| `spawn-isolated` arguments | isolating copy | 3 |
| Async arguments and results | already copied (3.7) | today |
| A process moved between groups | allowed only when an isolation walk proves it shares nothing (4.15) | 3 |

**Locks are not values.** A lock is named by a path string, and its holds live in the app's lock
manager, so no lock object crosses a boundary and invariant G is unaffected. The grant a
`with-lock` binds with `:as` is a plain dict in the holder's group.

**Why refusal and not copying closures.** C copies a closure with its environment. That copy has to
decide what to do with an environment chain that reaches the group's global env or a host env, and
with the natives inside, which capture an `intrinsics_context` (B11, B31).
- The old top-level refusal's reasoning applies at every depth: a stored callable can outlive what
  it captures, and it is an invocation channel around an allowlist (`intrinsic_state_data_set`'s
  comment).
- A nested closure in stored data was a hole, not a feature.
- Closures are refused, not copied (Q3).
- **Object instances** are refused for the same reason (S0-Q4): a user class's methods are
  closures over the environment `defclass` ran in, so an instance carries the writer's live
  environment inside its class dict. With two pycvc `Exec`s on one app, B could read A's stored
  instance and call its methods against A's environment, with no state write, emit or watch.
  Sharing a class dict needs a read-only dict whose methods hold nothing mutable, as the built-in
  classes are (5 item 1); a user class is neither. Rebuilding an instance by its class's name on
  read is a roadmap item (`docs/roadmap/STATE_EXEC_ROADMAP.md`, "Planned").

**Debug enforcement** (`CVC_EXEC_VERIFY=1`; Stage 1 builds only the pump owner's check, 4.1):
- every compound value posted across a boundary is tagged with its source group, and a second
  posting of the same object asserts;
- a move (Stage 3) runs an isolation walk first: the collector's census restricted to the process's
  roots, where every non-frozen node's `use_count()` must equal its references from inside the
  process plus its roots;
- in TSan CI, the walk runs for every group every 1000th pump.

### 4.4 Scheduling and balancing

**Inside a group.** Today's policy, on `perf/sched`'s run queue. The round-robin and priority
orders are unchanged. The policy is chosen when the scheduler is built (`state_exec.policy`), the
run queue keys on a process's effective priority (its base, or a lock-inheritance boost), and
`set-priority` re-levels at once ([`docs/STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md) 6.2, 7.3).

**Stage S: slices** (built in `perf/threaded-ss`, `async_scheduler::set_slice`; the plan is
`threaded-impl/S2.md`, S2-Q12 and S2-Q17). Optional and single-threaded.
- A *run* is up to `slice_steps` consecutive steps of one process, BEAM's reductions: one pick
  (`step_once`'s drain, watch poll and pick), then that process's steps on one step's guards,
  clock reading and accounting.
- **Settings**, each winning over the next:
  - `async_scheduler::set_slice(steps, seconds)`, or either half alone, `set_slice_steps(steps)`
    and `set_slice_seconds(seconds)`, each winning over its tree setting from then on;
    `exec_group::options::slice_steps` and `slice_seconds` call the half they give, so the tree's
    other setting still applies; `slice_steps()` and `slice_seconds()` read the values in force;
  - the tree's `state_exec.slice_steps` and `state_exec.slice_seconds`, under the scheduler's watch
    root, read when a run loop starts if a setting under `state_exec` may have changed
    (`settings_changes`), each unless it was set on the scheduler. A value that is not a whole
    number above 0 (steps), or not a number read strictly (seconds: `"2ms"` is refused), leaves the
    one in force; seconds of 0 or less mean 1 ms; a key missing, deleted or empty gives the
    default below;
  - `CVC_EXEC_SLICE_STEPS`, read whenever a scheduler is made, with no process-wide copy, so a whole
    suite runs at 256 unchanged;
  - else 1. `slice_seconds` defaults to 1 ms, since a data step costs 0.05-0.4 ms; set infinite,
    runs end on steps alone.
  - They live on `async_scheduler`, not `scheduler_base`. The sync scheduler, which ignores
    `max_step_time` already, is unchanged.
- **`slice_steps = 1`**, the default (Q10), is today's loop exactly: `run_loop` takes `step_once`,
  untouched. Above 1 it takes `step_run`, whose run lives in `execute_process_step` behind
  `max_run > 1`, so the step body is shared. A run counts charged steps, `run_loop`'s rule since
  #1117 (an engine's `ScriptSlice` or an inline async call charges more than its one step), clipped
  to what is left of the run loop's step budget. `step()` and `sync_step()` stay one step.
- **What ends a run:**
  - at every step: a park or any other status change (the end, a self or slot kill, a pause);
    `max_steps` and the message limits, which stay exact; a post from another thread (the ingress flag, or the lock
    port's), so a posted kill lands within one step; a stop; a reap; a process made ready at a
    strictly better priority level, or the runner re-levelled to a worse one
    (`process_table::begin_run` and `preempted`, raised in the enqueue and in `update`), so under
    a priority policy the best level is run within one step (the lowest-pid tie-break within a
    level is not kept: a runner goes on while a lower pid at its own level becomes ready); a
    pending signal or watch event;
  - at one clock reading after the run's steps 2, 4, 8 and 16, then every 16th step (so a run of
    costly data steps is read early; "the 16-step readings" below): past `slice_seconds`, only
    when the run loop has a time budget, so a run on steps alone (deterministic mode, a steps-only
    pump) stays deterministic; the run loop's time budget, which holds the pump deadline, each
    reading charging it as the steps since its last check (`time_budget::spent(now, checks)`), as
    one step a pick would; the process's time; a due timer; a due charge of an async call in flight
    (`next_call_charge` from the run's pick: the call's timeout, or the 1 ms charge beat of a
    process with a step or time limit), not a call merely in flight.
- **Limits and accounting.**
  - The deadline guard is armed once per run, with the process's time left, and the alloc guard
    once per run. The guard is checked only inside a native's long loop (`eval_deadline_check`):
    `step_now` checks no deadline between steps, so the guard does not bound a run of plain steps.
    `max_time` ends a run at the 16-step readings instead, both halves of it: a process whose first
    half ran out since its run began ends the run (its CPU is charged from the next pick), and one
    charging CPU ends it once the run's CPU so far, capped as at its end, would reach the second
    half. The pump deadline is not in the guard either, since a guard that fires kills; the run
    loop's time budget is checked at the readings.
  - A process with `max_step_time` (Ariadne residents, `ariadne.cpp:1792`) keeps one step a pick,
    so its guard stays per step. So does one with `max_memory` (Ariadne actions): its heap is
    measured and charged step by step (`heap_after_step`), and a run would let a short action
    allocate past its budget before any measurement began
    (`AriadneHeap.AnActionsLiveValuesCountAgainstItsBudget`). A heap accounting per run, so
    actions gain too, is a follow-up.
  - CPU time is charged at the run's end, each step's cap (`max_time / 32`) scaled by the steps
    run.
  - The run's seconds are split over its steps' kinds (eval, read, data) by count.
  - Its steps count in the scheduler's total and toward the 64-step cycle check.
- A contended `with-lock` parks, which ends the run. An uncontended one does not, so a lock holder
  may run a whole run.
- **Against this design's first text** (S2.md section 0, rows 1-6):
  - The guards were already armed only when limits need them, and the registry and stream were
    bound per entry, not per step. So a run saves the clock read, `add_step_time`'s atomic, the
    drain, watch poll and pick, and bookkeeping, not guard arming or thread-local setup.
  - The pump deadline is not folded into the deadline guard, but checked at the 16-step readings.
  - CPU is charged at the run's end with a scaled cap, and read at the 16-step readings while the
    process charges it.
  - A run counts charged steps, not picks.
  - Runs also end on posts and on a better priority level.
  - The setting lives on `async_scheduler`, not `scheduler_base`, and the sync scheduler is
    unchanged.
- **The gain.** The spike (SS.0) measured 46% fewer ns/step at 256 than at 1 on `sched.single`: a
  geomean over 17 points, ratios 0.37-0.82, on a loaded machine (S2.md section 11). It is a
  single-thread gain for every group, independent of the pool.
- **Semantics** at `slice_steps > 1` are section 5's item 16: the interleaving changes, and each
  process's step count does not. `Groups.OC01` pins `slice_steps` 1, since it asserts round robin's
  completion order.

**Stage 1: fork-join across groups** (built in Stage 1a, `exec_pool::pump_all`).
- **Order.** `pump_all` orders groups longest-last-pump first (LPT), which approximates the
  shortest makespan; groups skipped by the last deadline go first, and ties go by creation order.
  From Stage 2, which publishes each group's `best_runnable` (4.5), ties go to the group with the
  better one. At threads 0, and in deterministic mode, every group is pumped on the caller in
  creation order (4.14).
- **Execution.** Workers and the caller take groups from one mutex-guarded list per `pump_all`
  call. The caller pumps the host-affinity groups first, in creation order, and the groups on a
  single-mode lock manager with them (4.13), then helps. The pool's mutex is the handoff edge. A
  group belongs to one `pump_all` at a time (asserted); calls from several threads share the
  workers, each with its own list, host-call queue and accounting (S1-Q11), a helper woken for a
  call serves that call, and each caller services only its own queue (4.9).
- **Skipped groups first.** A group the last deadline skipped goes first. One the caller pumps
  starts even before `host_first`, `exec_pump_frame`'s group 0 (4.12): at threads 0, a
  host-affinity one, and in the threaded path when no helper was woken; otherwise the helpers take
  it beside group 0. So a busy group 0 delays a frame group by one frame at most (4.10).
- **Each group keeps its own pump budget** (section 4.10). The frame's DSL time becomes the maximum
  over workers instead of the sum.
- **Adaptive helper wake** (C's graft). Helpers are woken only when they would pay for themselves:
  - Waking a parked thread costs about 5-20 us on Windows.
  - If the groups' last pumps took less than `inline_below` in total (default 100 us), the caller
    pumps them all inline and no helper is woken. A group never pumped counts as 0 s, so the first
    frame of a fresh set of groups wakes none.
  - Otherwise the caller wakes one helper per group beyond the one it takes, up to the pool size.
    Each woken helper wakes the next only if work remains. `stats().helper_wakes` counts a wake
    when it is made.
  - A typical Ariadne frame (one or two short actions and a few parked residents) therefore never
    wakes a helper.
- **`CVC_EXEC_FORCE_MOVE=1`** (read when a pool is built) gives each group to a thread other than
  the one that pumped it last, every pump, waking a helper per group up to the pool's size. A
  caller left with only groups it pumped last hands them to the workers and waits for them no
  longer than the call's deadline (in waits of at most a second, so a far deadline is never handed
  to the condition variable whole), then skips those no worker started; a worker that reaches the
  call past its deadline skips its groups and tells the caller. Before Stage 1b's fix that worker
  told no one, and under load a frame (`exec_pump_frame`, an isolated document's `drain()`) could
  wait for ever (`ExecPool.AForceMovedCallEndsAtItsDeadline`).

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
  consumed, divided by the group's weight. A CPU-heavy group then cannot starve a latency-sensitive
  one.
  - **Weight from priority.** At the end of each pump a group publishes `best_runnable`, the best
    effective priority among its runnable processes, lock boosts included. Under a priority policy
    its weight is that priority's CFS weight (1024 at 0, about 1.25x per step). A group of nice-0
    processes is unweighted; a group holding a -10 process, or a lock holder boosted to -10, gets
    about 9x the pump share of a nice-0 group, and none starves. Round-robin groups keep 1024.
  - `state_exec.groups.priority_weights` (default on) turns it off. It ships on only once
    `prio.groups_weighted` passes (6.3).
  - The queue's mutex is taken about once per slice per worker (2 ms), so per-worker deques with
    stealing are not needed at group granularity. They are a later option if `mt.groups.flood`
    shows contention.
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

**Deterministic mode** (`pool_options::deterministic`, `state_exec.deterministic`, or
`CVC_EXEC_DETERMINISTIC=1`):
- every group is pumped on one thread in a fixed rotation by group creation order (as built:
  `pump_all` pumps them all on its caller, in creation order);
- slices are fixed in steps, not time: the frame deadline and the seconds budgets are ignored;
- each group's random stream is seeded from the group's name (a hash), unless it has a seed or has
  drawn already; `exec_pump_frame` pumps group 0 with steps only and no deadline too, and seeds its
  stream from the name "group0";
- cross-group posts are applied in post order, lock wakes and priority relevels included;
- lock grants follow arrival order among conflicting requests, and a deadlock is found when the
  request that closes it blocks, so its victim is fixed by the arrival order
  ([`docs/STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md) 5.4, 6.1).

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

**Pids** (B32; built in Stage 1a). Each app has one pid source, a shared `std::atomic<int>`
(`app::exec_pid_source()`), made with group 0's state and counting from 1. The app scheduler and
every `exec_group` built with that app draw their pids from it from their first pid
(`async_scheduler::set_pid_source`), so the app scheduler's first pid is still 1 when no group
exists.
- Pids stay unique across the whole app, as they are today with one app scheduler, and increase
  within each scheduler in creation order (a group sees only its own). `ps` output and host logs
  keep meaning what they mean.
- Every pid a scheduler issues comes from its source when it has one: `execute`, `fork`, and
  `restore`, which gives the restored process a new pid as before.
- `set_pid_source` refuses a source whose next pid is not above every pid the scheduler has
  issued (`std::invalid_argument`), the rule `process_table::insert` needs (pids increase), and a
  debug build asserts that an insert never replaces a live pid.
- A plain pycvc `Exec`, a bare scheduler and Ariadne's init lane keep counters of their own, so
  their pids still repeat across schedulers.
- The cost is one atomic increment per process created.
- Pids stay `int` (Q11). Every group of an app now draws from one counter, so a long-lived host
  with heavy process churn reaches the wrap after 2^31 processes sooner than one scheduler did;
  that stays a known limit.
- A process moved in Stage 3 keeps its pid.
- Stage 2 adds a directory that maps a pid to its group: 64 shards, each a mutex and a map. It is
  written at create, move and reap, and read by host and cross-group calls. Ariadne creates about
  one process per action, so the writes are negligible.

**From a host or another thread.** `pool.post_control(pid, cmd)` covers `kill`, `pause`, `resume`,
`signal` and `set_priority`.
- The command goes into the target group's ingress as a new entry kind, applied in
  `drain_ingress` between steps.
- It returns true when the pid was live in the directory.
- It never blocks.
- Stage 0 built the scheduler half: `post_control` with these kinds on both schedulers
  (`control_cmd` in `scheduler_base.h`: `kill`, `pause`, `resume`, `signal`, `set_priority` and
  `query`, with an optional `done(bool)`), which `exec_coordinator`'s bus threads use (4.12). The
  pool's version routes to the target's group.

**From a program to another group** (Stage 2):
- `kill`, `pause`, `resume`, `send-signal` and `set-priority` on a pid in another group are posted
  as above. The call returns `#t` for "requested", and the effect lands at the target's next step
  boundary. A `set-priority` is checked against the target's published owner, uid, parent pid and
  floor (the rule of [`docs/STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md) 7.2, the descent walked over
  the published parent pids), carries the caller's pid, owner, uid and floor, and is re-checked in
  full at the target, which holds the authoritative values
  ([`docs/STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md) 7.5).
- This is allowed only when the target's group has the caller's owner: the same document or the
  same `Exec`. Otherwise the pid is treated as unknown, as today for a dead pid.
- The same rule governs `ps` and `inspect`. A program sees its own group exactly, and other groups
  of its owner through each group's published table, at most one slice stale.
- So documents stop seeing and killing each other's processes. Today they can, through the shared
  app scheduler. Within one document nothing is lost (Q5).

**Published info.** At the end of each pump a group publishes a snapshot of its processes' info
(status, steps, times, memory, owner, uid, parent pid, base and effective priority, floor, locks
held, what each waits on) and its `best_runnable` priority, under a mutex. Cross-group `ps`,
`pool.info(pid)`, cross-group `set-priority`'s check, the ready queue's weights and the
per-group statistics read it.

**`kill_owner`** (D's graft). Stage 1a built the scheduler's half, which group teardowns and
Ariadne's rely on: `async_scheduler::kill_owner(owner)` takes the owner's live processes first and
then kills each by its pid, so a kill whose tree slots reap or submit processes cannot disturb the
loop (one submitted meanwhile is not killed). Every kill is tried, and the first that threw is
rethrown after the rest (KO-1, KO-2). `kill(pid, reason)` kills with a reason of the caller's as
the exit error (the pool's "internal error", 4.17). Stage 2's `pool.kill_owner(owner)` posts a
kill to every group that has a process of that owner, then waits until each group has applied it.
- In fork-join mode the wait pumps those groups' ingress inline.
- Ariadne calls it before freeing the action contexts that intrinsics point at (`ictx.document`,
  `ictx.channels`, which `Runtime::Impl::make_action_context` points at the Runtime's document
  scope and channel policy). Without the acknowledgement, a child in another group could still run
  against a freed context.
- A document's own group is simply destroyed (section 4.2), after the Runtime's own kills and
  reaps (4.17, "Teardown of an Ariadne Runtime").

### 4.6 Messaging between groups

**Inside a group:** unchanged. Delivery is immediate, broadcast reaches every waiter with one
shared envelope, and the pending FIFO keeps its cap of 1024 (`async_scheduler.h:105`).

**Stage 1: no router** (built in Stage 1b). A document gets a group of its own only if it uses no
app-global channel (S1-Q6; the rules are in 4.12):
- Only a `channels:` block enforced in strict lint mode proves that, so it needs one, with no
  `global: true` entry, and no program of it may name a `/` channel in a literal. A channel key it
  computes at run time is checked by the channel policy (`Runtime::set_channel_policy`'s
  `enforce`), which refuses an undeclared one, a `/` key included.
- Its channels then all resolve under its prefix (`resolve_channel_key`), as its state keys do.
- A `#` key is never scoped, so oidc's `"oidc#NAME"`, posted to group 0, would never reach the
  document's group: a document that uses oidc stays in group 0.
- The host posts it receives go to its group: the Runtime's tick, key, pointer and signal posts
  (`post_tick`, `deliver_input`, `post_signal`) go to the scheduler its programs run on, and its
  providers post to the calling program's group: async verbs' done channels, streams' events and
  net completions (4.9).

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

**Locks do not use the router.** A lock is not a channel, so it has no home group, and Q6 does not
apply to it.
- All lock state (holds, waiting requests, per-process records) lives in the app's lock manager
  under one mutex. An uncontended acquire from any group costs the same as a local one, with no
  round trip, and every grant, cancellation and deadlock check is decided there, on one
  consistent state.
- Only two kinds of post cross groups, both appended to the target group's lock port and
  followed by `wake(g)`: `wake{pid, seq}`, which readies a waiter still parked on that request,
  and `relevel{pid}`, which makes the holder's group re-read its inherited priority. Neither
  carries an outcome: the woken group reads the request's outcome from the manager, and a stale
  wake (the waiter timed out and adopted the grant, ended, or made a newer request) or a late
  relevel changes nothing. The port has its own mutex and queue, outside the ingress cap, and
  shares its scheduler's wakeup, whose notify is `wake(g)`, so a post never touches a scheduler
  member. A lock post is dropped only once the port is closed. A group's teardown closes its lock
  port right after it takes the group's token, before it kills the group's processes (4.2), and a
  post calls `wake(g)` inside the port's mutex only while the port is open, so no post reaches a
  dying group, and none is dropped while the group is pumped
  ([`docs/STATE_EXEC_LOCKS_PROTOCOL.md`](STATE_EXEC_LOCKS_PROTOCOL.md) 2.3, 5.7;
  [`docs/STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md) 9.2).
- **Nothing is passed on.** A grant is recorded in the manager as a hold of the waiter's process
  when it is made, so a waiter killed, reaped or ended with its wake in flight releases it through
  its end, like any other hold, and the next waiter is granted. A waiter whose timeout fires after
  the grant takes the grant instead. No grant is lost or released twice
  ([`docs/STATE_EXEC_LOCKS_PROTOCOL.md`](STATE_EXEC_LOCKS_PROTOCOL.md) I6, I10).
- A lock wait is not a `recv_path`, and no `#` channel is involved, so no `msg-send` can wake a lock
  waiter ([`docs/STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md) 3.4, 8.5).

**Cost.** One cross-group message costs:
- a mutex push into the target ingress, about 50-100 ns;
- a 4-entry envelope, about 5 allocations, plus the payload string copy;
- a wake of 5-20 us when the target group is idle and parked (0 when it is running).

Today's in-scheduler round trip is 23-95 us, mostly the tree-side `sendMessage`, so a cross-group
hop is a small fraction of it (judge 2's correction of D). It will matter more once messaging
itself gets faster, so placement keeps chatty pairs in one group.

### 4.7 Cycle collector and heap measurement

- **A registry per group** (B5; built in Stage 1a):
  - `cycle_registry` is an opaque type in `heap.h`. `make_cycle_registry()` makes one, with an id
    of its own that no later registry reuses. `cycle_registry_scope` binds one on the calling
    thread and restores the binding it found.
  - Tracking (`track_captured_env`, `track_mutated`) and the no-argument `collect_cycles`,
    `cycle_collection_due`, `note_process_exit` and `tracked_objects` use the bound registry,
    else the thread's own: bare evaluators, bare schedulers, the reactive lane, host-thread setup.
    Each has a form taking a registry too.
  - A scheduler given a registry (`set_cycle_registry`) binds it on every entry that can run DSL or
    track (4.1), and names it, not the bound one, for its every-64th-step due check
    (`execute_process_step`) and for its exit note (`release_if_idle`). So a host's `kill(pid)`
    between pumps notes the group's registry, never the host thread's.
  - Who owns one: each `exec_group` (its first member, so it goes last), the app's group 0
    (`app::exec_group0_state`, declared before `exec_scheduler()` and its async host), a plain
    pycvc `Exec` (before its scheduler and envs) and a pooled `Exec` (its group's). Each owner
    frees its registry after its scheduler and environments are gone. `Exec.collect_cycles()`
    collects an `Exec`'s registry on demand, as its pump owner; `pycvc.collect_cycles()` stays the
    calling thread's own.
  - The closure-body cache (`detail::shared_body_expr`, a per-thread cache matching a lambda form
    by address) tags each slot with `bound_cycle_registry_id()` and reuses a slot only under the
    same registry, so two groups taking turns on one worker never share a body list.
  - A debug build asserts that no async kernel tracks: a kernel's thread's own registry is never
    collected.
- **No safepoint.** Collection runs inside the pump, at a step boundary, as today: `collect_cycles`
  still does nothing inside an evaluator step. Under invariant G every node the walk reaches is
  owned by the group or frozen.
  - Frozen environments are skipped.
  - The built-in class dicts and the stdlib scope are held from outside the group, so the census
    sees more owners than internal references and keeps them live.
  - No other thread mutates or refcounts anything in the walked graph.
- **Host references.** A host holding a group's value is an external owner, which keeps it live.
  Hosts read and drop such values only between pumps (fork-join) or under the token (continuous).
  A queued `host_proxied` body and a deferred `on_complete` callback (4.9) run on the caller
  while no thread pumps their group, with the group's registry and random stream bound, so what
  they track goes to the group, not to the caller's thread.
- **Lifetime.** A group's registry dies with its owner, after one final collection: the deleter
  (`cycle_registry_deleter`) binds the registry, collects it (even inside another group's step,
  where `collect_cycles` refuses, since no step of its own group can run any more), hands what a
  host still holds to the caller's registry (the bound one, else the thread's own while it
  exists), and frees it. This also fixes a leak that existed before Stage 1: an `Exec` driven from
  short-lived Python threads lost the objects tracked on each thread when it ended (GR-5).
- **Idle groups.** A group's registry is collected only by its own steps: every 64th step, once
  enough was tracked (as many new objects as survived the last collection, at least an eighth of
  what it walked, at least 256), or once a process ended and something was tracked since, at most
  once a second. A thread's registry used to be collected by whichever scheduler stepped on that
  thread. So a run (`run_loop`) that leaves nothing runnable collects the scheduler's registry when
  that rule says one is due: since the pool pumps an idle group every frame, its garbage goes
  within about a second of a process ending (5 item 8).
- **Threads' own registries** (S1-Q3). A thread's own registry used to be dropped at thread exit
  with nothing collected. Now a thread may opt in, once, to a collection at its exit
  (`collect_cycles_at_thread_exit()`): the pool's workers do (their registries should stay empty),
  and so do test threads (RX-1). Python threads and the main thread never do: in a `python`
  process their thread-locals go after `Py_Finalize`, where pycvc's callable deleter cannot take
  the GIL, and on Windows thread-local destructors run under the loader lock.
- **Moves** (Stage 3). The isolation walk that admits a move also re-tracks the process's tracked
  objects into the target group's registry.
- **Heap measurement** (`heap_share_job`, `heap.cpp:263-380`) runs as the process's own data steps.
  Under invariant G it reaches only group-owned and frozen nodes. For frozen ones it reads
  `use_count()` racily, but only for an estimate, so it needs no change (B6).

### 4.8 State tree

The tree is the one mutable structure all groups share. Stage 0 made the DSL's use of it safe.

- **Pinned lookups** (B14). The intrinsics walk the tree holding what they walk (`intrinsics.cpp`:
  `read_node`, `write_node`, `find_node`, `make_node`), so another thread's delete cannot free a
  node under them.
  - The read view walks through links with `state::findThroughLinksShared` and holds the node it
    returns. The write view walks with `childThroughLinksShared` and holds the chain below the
    root to the end of the call. On a link hop both add the link's `target_pins`.
  - The physical walks (`state-delete`, the expiry natives, `state-watch`, `msg-send` and the
    channel lookups) use `findDescendantShared(self, path, pins)` and `sharedChild(self, path,
    pins)`: overloads for a caller that already holds the start node, which take no reference on
    it, so a lookup does not touch the app root's or a chroot's count.
  - The sliced writers differ. `state-from-json` and `state-set-tree` hold the root's chain, their
    target's and any link's for their whole life, and write through what they hold (the JSON
    reader, `state_json.cpp`, walks each write from its held target and keeps that chain until
    the write's signals are done). `state-set` of a list and `state-data-set`'s copy hold only the
    root's chain while their data job runs, and walk to the target, holding its chain, when they
    write.
  - Chains are released leaf first, so a node held in one dies while its ancestors live (a
    dying node's `destroyed()` slot may walk `_parent`). The context's `root_pins`, the jobs'
    chains and a link resolution's `target_pins` are `cvc::pin_vector`s (`state.h`), which drop
    their pins from the back when they go, are cleared or are assigned over. Stage 1b made two
    more so (S1.9, S1.11; PIN-1): a stream's descriptor pins, and `uri_state.cpp`'s chains
    (`chain` in both lookups, and `effective::pins`, declared before `effective::node`, so the
    node goes first and then its chain, leaf first; a followed link's node and chain are released
    explicitly in that order). They are reachable without groups: `state_resolve` may run on a
    compute-pool worker. A hold on a single node does not keep that rule: a `state-get`'s
    read, `state-json`'s node or a watch entry's root can outlive an ancestor that another thread
    unlinked meanwhile.
  - `msg-send` holds its node and that node's chain across `sendMessage`, whose link path reads
    the pinned walk's path and whose non-link path reads `fullName()` up the held chain.
    `sendMessage` itself is unchanged.
  - `poll_watches` holds each watched node while it reads it, through the start-pinned lookup.
  - The schedulers' settings reads and publication hold what they walk: every `*_from_state`
    reader goes through `detail::setting_value`, the publication (`load_settings`, the published
    rate) through `detail::publish_setting`, and `settings_changes` holds the `state_exec` node it
    counts until it connects elsewhere (`clear()` lets go of it).
  - pycvc's tree accessors (`pycvc_state.cpp`) walk pinned too, so `Exec`s on several threads and
    Python threads reading the tree are safe together (4.12). A `state_observer` dropped while
    another thread's write is firing its slot is never called: the slot checks a liveness flag
    once it has the GIL. The pinned walks and `removeChild` themselves are not exposed to Python
    (`pycvc.i` ignores them): a returned pin would leak, and `removeChild`'s caller must hold the
    parent's chain.
  - Host code outside `intrinsics.cpp` still walks raw: Ariadne's settings reads and holes
    (`ariadne.cpp`), `doc_args.cpp`, `app.cpp`'s settings read, `state_value_codec.cpp`'s
    snapshot walks, the conference natives (`room_intrinsics.cpp`, `room_media_intrinsics.cpp`),
    `holon` replication and `oidc_host.cpp`. As built in Stage 1b, none of it runs on a worker:
    Ariadne reads its settings and plants its holes on the host thread (`drain_begin`, the
    submissions, render's reconcile), and the conference, oidc and `data-*` natives are
    `host_proxied`, so in an `any` group they run on the `pump_all`'s caller (4.9). A provider
    body a worker may run (a spawned child's, 4.15) walks pinned or not at all: AppRuntime's
    `data-*` provider checks its program's chroot through `findDescendantShared`. A stream's
    descriptor chain is a `cvc::pin_vector` (`stream::descriptor_pins_`), released leaf first.
- **A pinned chroot** (D's graft). The context holds its root and every ancestor up to the tree's
  root (`intrinsics_context::root_pins`, `back()` is `root`). `apply_chroot` pins them once; a
  host that assigns `ictx.root` directly is pinned on the context's first state intrinsic
  (`pinned_root`: one compare per call). Spawned children copy the pins. A chroot cannot be freed
  while a context uses it, and the held chain makes `fullName()` and a write's `childChanged` up
  the parents safe. A context keeps its chain for its life: a deleted chroot lives on, detached,
  until its contexts go.
- **`state-delete`** unlinks its one node from its parent (`state::removeChild`): `childChanged`
  once, never `expiring`. It no longer expires the node and sweeps the context's root.
  - One group can then no longer free nodes another group holds, beyond the one it deletes, which
    lives on until the last holder lets go.
  - For unchrooted contexts it also removes a whole-tree walk.
  - An empty path does nothing. Before, `state-delete ""` in a chroot expired the chroot node,
    which its own sweep could not remove, so a later sweep of an ancestor deleted the document.
  - pycvc's `state_remove` and `state-set-tree`'s stale children use `removeChild` too.
  - An expired node stays until a sweep (`state-sweep-expired`, the host's `sweepExpired()`).
- **A per-watch root** (B13). `state-watch` registers its context's root with the watch
  (`watch_handler_entry::root`, through the 5-argument `register_watch_handler`). The watch
  resolves under that root, and is polled only after a write under it: per-root gating, one change
  counter per distinct watch root (`detail::watch_roots`), shared by the entries with that root.
  - A chrooted context never calls `set_watch_root`. The scheduler's watch root stays the host's,
    for settings, the published settings and the async charge rate.
  - One case is kept: an unchrooted context on a scheduler that has no watch root
    (`scheduler_base::watch_root()` null) sets it, as before, and the scheduler then reads and
    publishes its settings there and gates its watches. A bare scheduler's programs rely on it.
    That call comes from a step, so it passes a pin: `set_watch_root(root, pin)`, where the pin
    holds the context's whole root chain. No host keeps such a root alive, so the scheduler keeps
    the pin for as long as that root is its watch root, and a later `set_watch_root`, of either
    form, replaces or drops it.
  - Otherwise `set_watch_root` is a host call, made between steps: the app
    (`app::exec_scheduler()`), pycvc's `Exec` and the CLI make it, with the one-argument form,
    and keep the root alive.
  - A `scheduler_base` subclass outside the tree gets defaults that ignore what it doesn't keep:
    the 5-argument `register_watch_handler` drops the watch's root (so a chrooted watch's path
    resolves under that scheduler's watch root), `watch_root()` returns null (so every unchrooted
    `state-watch` sets the root again), and the 2-argument `set_watch_root` drops the pin. Such a
    scheduler overrides all three. Both in-tree schedulers do.
  - `watch_polls()` counts the watch entries polled, one each time a watch is looked up (whether
    or not its node exists), so a test can see the gating.
- **Contention.**
  - A lookup takes each path segment's `boost::mutex` from where it starts (`state.cpp:1334-1354`),
    and a write emits up the parent chain.
  - Ariadne's lookups start at the chroot node, so documents writing disjoint subtrees share only
    the locks of the nodes above their prefixes, through write emission.
  - pycvc `Exec`s are unchrooted (`pycvc_exec.cpp:414`), so all their lookups start at the app
    root. `ExecPool` therefore gives each pooled `Exec` its own prefix by default.
  - `perf/sched` connects a counter slot to the watch root's change signals while a scheduler has
    watches or calls in flight. The slot's counter is atomic. With G groups watching the app root,
    every write calls G slots. Since Stage 0 each distinct watch root gets its own slot, so a
    chrooted watch's slot sits on its chroot, and a write calls only the slots on its ancestors.
- **signals2 subscribers** (B15) run on the writing thread (`state.h`'s signal members).
  - **How one runs.** A write emits on the thread that writes: `valueChanged`, then `childChanged`
    on the parent and every ancestor above it, with no node mutex held while the slots run. A
    `cvc::state_object<T>` connects its own node's `childChanged` in its constructor, so the
    thread that builds one connects it, and with instance threading off (every subscriber below
    that is a `state_object` turns it off) its `handleStateChanged` runs inline on the writing
    thread. So a group a worker pumps runs, on that worker, every subscriber connected at, under
    or above a node its programs write. In arigl the scene prefix is the document's prefix, so
    the camera, touch, lighting, HUD and settings subscribers sit under the document.
  - **Rule** (as built in Stage 1b, S1-Q6). An isolated document's group is `affinity::host`,
    pumped on the host thread, when its host declared a thread-affine subscriber at, above or
    under the document's prefix (a subscriber above it is called for each of its writes; R1b-mid
    S-M2), or when its env holds a `host_only` native (4.9). Otherwise it is `affinity::any`.
  - **Declarations.** `AppRuntime::declare_thread_affine_prefix(prefix)` (it forwards to
    `Runtime::declare_thread_affine_prefix`) records a prefix under which the host connected a
    subscriber that must run on the host thread; the Runtime reads its declarations whenever it
    decides where the document runs (4.12), so a host declares before `load()`, and a later
    declaration has the placement decided again. An empty prefix (the root) covers every document.
    `AriRuntime` declares its scene prefix before any load, so every arigl document's group is
    `host` (S1-Q6), until the subscribers below move out of the document's prefix or marshal.
  - **The audit** (S1-Q7, S1.11), every subscriber on a subtree a document group's programs can
    write:

| Subscriber | Connects | Kind (S1-Q7) | Verdict (S1.11) | Why |
|---|---|---|---|---|
| `state_sync_adapter` (replication) | each attached node, all seven signals | not thread-safe | confirmed not thread-safe; its header's "Thread-safe" is qualified, no code change | its slots re-walk from the root with raw, creating lookups (`state::instance(ctx)(path)`; so do `attach`, `attach_node` and `apply_remote`), and read `_attached`, which is written under `_mutex`, unlocked. In Stage 1 only host-thread writers reach the subtree it observes (`conf.*`: the room natives, which are `host_proxied`, and host code); arigl applies remote mutations in its frame's transport pump, on the host thread |
| cvcGL `SceneNode` family (`SceneNode`, `GeometryNode`, `GridNode`, `AxisNode`, `LightNode`, ...) | `state_object`, instance threading off | thread-safe (marshals) | thread-safe | each key's work goes through `runOnMainThread` (posted through the scene's locked event sink, or held for the node's own thread when it has no scene); `LightNode` marshals all of it |
| `GraphicsNode::handleStateChanged` | as above | reviewed in S1.11 | thread-safe after one fix | its echo checks run on the writing thread before it marshals; one was a race, now fixed (below) |
| `CameraController` | `state_object`, threading off | not thread-safe | not thread-safe; debug home-thread assert | `readAllFromState` and `applyToCamera` on a `vtkCamera`, into unlocked `Impl` fields the render loop's `update()` reads; its atomic self-write flag would drop a foreign write that lands during a self-write as an echo |
| `TouchGestures` | the same | not thread-safe | not thread-safe; assert | `readAllFromState` into unlocked `Impl` fields the event handlers read |
| `StageLighting` | the same | not thread-safe | not thread-safe; assert | `apply()` rebuilds light nodes; its re-entry flag is a plain `bool` |
| `FpsHud`, `ScreenTextHud` | the same | not thread-safe | not thread-safe; assert | `apply()` / `applyToActor` on a `vtkTextActor` |
| `ShadowSettings` (built by `SceneGraph::syncShadowState`) | the same | not thread-safe until reviewed | not thread-safe; assert | it writes its value, unlocked, on the writing thread while `get()`/`set()` use it on the owner thread (`SceneGraph`'s apply itself marshals); the "replicated peer, script thread" path its comment names now fails the debug assert, as S1-Q7 decided |
| `UiSettings` (built by `ImGuiOverlay`), `ViewportLayout` (built by `Viewport`) | the same | not thread-safe until reviewed | not thread-safe; assert | a plain re-entry `bool`; their apply callbacks write the overlay's fields, or call `vtkRenderer::SetViewport`/`SetLayer`/`SetDraw`, on the writing thread |
| volren `state_settings` (`VolRenNode`) | the same | not thread-safe until reviewed | kept "until reviewed"; nearly thread-safe; no assert (not in S1.11's files) | a snapshot under its own mutex, then `VolRenNode`'s apply takes its config mutex but calls `ownBoundsChanged()`, which reads two plain `bool`s (`m_showBBox`, `m_clipChildren`) before it marshals |
| volslice `state_settings` (`VolSliceNode`) | the same | not thread-safe until reviewed | not thread-safe; no assert | `VolSliceNode`'s apply increments a plain `m_settingsVersion` the render path reads |
| physics `world_binding` | the prefix node's `childChanged` | thread-safe | confirmed | a thread-local check of its own writes, then a store to an atomic flag held by `shared_ptr` |
| pycvc `state_observer` | the root's `childChanged` | thread-safe, with the GIL rule | confirmed | it takes the GIL on the writing thread and checks a liveness flag; so a Python host releases the GIL across `exec_pump_frame` and `run_all` (4.12) |

  - **Also checked** (S1.md section 0, item 17): `room_net.cpp`, `holon_sync.cpp`, `holon.cpp` and
    `uri_http_cache.cpp` connect no state signal. The library's other state-signal slots are the
    watch roots' change counters (atomic), xmlrpc's monitor on `__system.xmlrpc` (not a DSL
    subtree), `data.cpp`'s slot on the root's `destroyed`, and the generic `state::value(callback)`
    and `state_future`. No `state_object` subclass exists outside cvcGL and volren/volslice.
  - **The debug home-thread asserts** (S1.11). Each not-thread-safe cvcGL subscriber above
    (`CameraController`, `TouchGestures`, `StageLighting`, `FpsHud`, `ScreenTextHud`,
    `ShadowSettings`, `UiSettings`, `ViewportLayout`) records the thread that built it, so the
    thread that connected it, and asserts it on the first line of `handleStateChanged`, before any
    echo check: "`<Class>`: a state change off its render thread (STATE_EXEC_THREADED.md 4.8)". The
    member is unconditional (in the `Impl` of the five pimpl classes, in `Settings.h` for the
    three settings classes), so a Debug app and a Release library agree on the layout; only the
    assert depends on `NDEBUG`. Release builds compile them out; CI's Debug cvcGL jobs
    (`package-linux`, `package-macos`) check them. No existing test writes one of these subtrees
    from another thread.
  - **`GraphicsNode::handleStateChanged`** runs on the writing thread and, before it marshals,
    checks three things:
    - its own publisher's echo of `position`/`matrix`: `state_publisher::flushing()` is
      thread-local, so on a worker, which is in no flush, it is null and nothing more runs; in a
      flush, the scene outlives it (`~SceneGraph` joins its publisher's worker first). Safe;
    - its own scene-less pose write: a plain `bool` member, set by the node's thread around its
      write in `publishPoseMatrix` and read on whatever thread wrote, was a data race, and a
      foreign `matrix` write that read it `true` was dropped as the node's echo, leaving node and
      tree apart. Fixed: the mark names the writing thread (`std::atomic<std::thread::id>
      m_poseWriter`, claimed by compare-exchange in `publishPoseMatrix` and cleared after the
      write, checked as `m_poseWriter == this_thread::get_id()`), on the node rather than in a
      `thread_local` (no process-wide state, VOLROVER-LIBRARY-PORTS-ROADMAP.md 1.3); another
      thread's write goes to `runOnMainThread`. A second thread writing the same node's pose at
      once does not claim it, so its echo is heard as a change and redraws the same pose. Test:
      `cvcgl_unattached_node_thread`'s
      `test_another_threads_matrix_during_the_nodes_own_pose_write_is_not_its_echo`, which failed
      before;
    - its metadata mirror, which reads only its argument. Safe.
  - **Follow-ups** (recorded, not built): `state_sync_adapter` walks pinned in its slots and
    `apply_remote`, with an atomic `_attached`, before a worker-pumped group may write the subtree
    it observes; the settings classes and the five render-thread subscribers move out of the
    document's prefix (or marshal like the `SceneNode` family) before an arigl document can be
    `any`; the racy reads in volren's and volslice's settings above are fixed before those are
    called thread-safe; and `state_object`'s teardown against a slot running on another
    thread (signals2's disconnect does not wait for it), which also covers a narrow pre-existing
    read in `GraphicsNode`'s publisher check when two scenes mirror one prefix and one is
    destroyed on its owner thread while the other's publisher flushes a path the node mirrors.

### 4.9 Async host, ingress and host-thread natives

- **One async host per group** (built in Stage 1a): `exec_group::host()` builds
  `async_host(sched(), app)` on first use, under a `std::once_flag`, because most `ExecPool` groups
  never make an async call. The constructor binds the group's scheduler for the host's life.
  - Kernels run on the shared kernel pools and lanes (`perf/async`).
  - A completion is posted to `call->async_sched_`, the creating group, so it always returns to the
    right group.
- **`intrinsics_context` gains `async_host *host`** (B30). Stage 1a appended the field (the
  context stays an aggregate), `make_spawn_context` copies it to a child with `sched`, and pycvc's
  `Exec` sets it to its own host or its group's. Stage 1b sets it in every Ariadne context, so a
  provider registers its async functions on `*ictx.host`, never on `app.exec_async_host()`:
  - an action's and a resident's (`Runtime::Impl::make_action_context`): the document's group's
    host (`exec_group::host()`) when the document is isolated (4.12), else the app's; a child they
    spawn copies its parent's (`spawn_child`, 4.15);
  - the init lane's (`run_scoped_program`): an inline host of its own, `async_host
    init_host(&sched)` over the lane's private sync scheduler, declared after it so it goes first
    (S1-Q2). A provider's async functions run inline there, as before, but are now charged to
    init's own process, where nobody was charged before (FF-3's init case). The lane's
    scheduler also has a cycle registry of its own, which the run collects and frees when it ends
    (`AriadneInit.ItsCyclesAreItsOwnAndFreedWhenItEnds`);
  - the env the name probes build (`Runtime::Impl::probe_env`, for `has?`, `requires:` and the
    placement's `host_only` check): an inline host, never called.
- **Posts go to the calling program's group** (B30, S1.9). A provider posts where the calling
  program's host parks its calls, `ictx.host->parking_scheduler()`, read when the env is built: an
  isolated document's group, else group 0. An inline host parks nowhere (the init lane's), and
  then the post goes to group 0, `app.exec_scheduler()`. Nothing posts to `app.exec_scheduler()`
  on a program's behalf otherwise:
  - AppRuntime's done channels (`register_async_verb(name, done_channel, work)`,
    `register_async_function(name, marshal, done_channel)`): the kernel posts its value to the
    channel, resolved under the program's root when the env is built, on the calling program's
    group; from `init:`, on group 0, where a program waiting on it hears it
    (`AriadneHostVerbs.AnInitLanesAsyncVerbPostsItsDoneChannelToGroupZero`).
  - Streams: `stream-open` gives the stream its opener's group (`stream_params::scheduler`; null
    means group 0, `stream::open`), and the stream posts its seq and lifecycle events there, from
    its producer thread too, under a per-stream mutex. `room-subscribe`'s proxy stream opens on the
    caller's group; the host-side bridge and relay streams, and the public `room_subscribe`, stay
    on group 0. Every stream opened with a document's scheduler is owned by the document's stream
    table (`stream-open`'s, and the proxy through `own_document_stream`), so a document that moves
    takes the streams it owns along, whatever scope each was opened under, and no other owner's
    (`retarget_document_streams`, 4.12).
  - Net: a request's completion goes to its call's own scheduler, its host's; a started request
    (a transport that answers by itself) keeps its host's finisher, not the host (below).
  - The Runtime's own tick, key, pointer and signal posts go to the scheduler its programs run on
    (`post_tick`, `deliver_input`, `post_signal`).
  - oidc's flow end, `"oidc#NAME"`, still goes to group 0 (`Session::set_status`): a document that
    uses oidc is never isolated (S1-Q6).

  For a document in group 0 these are the same host and scheduler as before Stage 1b.
- **Futures cannot leave their group.**
  - Messages carry strings and bytes only.
  - The isolating copy refuses futures at any depth (4.3).
  - `async_call::outcome()` can therefore be reached from one group only.
  - Stage 0 made finishing a call idempotent, as a cheap backstop: the first `finish_async_call`
    or cancel claims it (`async_call::closed_`, an atomic exchange), so a call awaited twice, or
    by a fork child beside its parent, is charged once and gives one result. Every process
    waiting on the call is woken, including on the paths that used to wake nobody: a call no
    longer listed, and the calls `cancel_async_calls` drops. So a fork child awaiting its
    parent's future is woken when the parent is killed. `cancel_async_calls` settles each call
    it claims with `RuntimeError` "`name`: cancelled: the process that made the call ended",
    before it wakes the waiters, so every way the call is awaited raises it (`await`,
    `await-all`, an `await` made after the cancel, an await in a generator body), and the
    kernel's own late settling is ignored. Nothing waits for such a call for ever.
    `async_call::outcome()`'s check of the cancel stays as a backstop, for a call cancelled some
    other way and read before it settles.
  - A finish that throws still wakes the call's waiters: the final charge reads the step rate,
    which may publish it and run the tree's slots, and a kill at a limit can throw; the catch
    wakes and rethrows. A completion `requeue_ingress` cannot queue again (no memory) wakes its
    waiters instead, whose `outcome()` then finishes the call.
  - **The foreign-future check** (S0-Q6, S1-Q2; built in Stage 1a). A threaded future is finished
    by the scheduler its completion is posted to, its owner (`detail::awaitable::owner()`, which
    `async_call` answers with its `async_sched_`). `await` (and `msg-recv` on a future, which goes
    through it) and `await-all` compare a future's owner with the awaiting context's scheduler
    before they read, park on or wait for the call. Another owner raises `RuntimeError` "await: the
    future belongs to another scheduler" (`kForeignFutureError`, `stackless_evaluator.h`), origin
    `"async"`. `await-all` checks every future in its list before it waits on any, so a foreign
    future raises at once rather than after an earlier entry finishes.
    - The owners are compared by serial, not by address: every scheduler has a serial
      (`scheduler_base::serial()`), unique in the OS process and never reused, and a call keeps its
      owner's (`awaitable::owner_serial()`, set when the scheduler tracks the call). A host can keep
      a future after its scheduler is gone, and a later scheduler can sit at the freed address;
      that one's future is still foreign.
    - An inline future has no owner: it was settled before it returned (an inline executor's, or
      a call made where its host had no current process, which runs at once), so it is a plain
      value any scheduler may read (FF-2).
    - Ariadne's `init:` lane awaits the app host's futures on its private sync scheduler. During
      init the app scheduler has no current process, so each of those calls runs inline and comes
      back settled with no owner: the check passes it, and HZ-13 (whose provider registers on the
      app's host) keeps passing. Stage 1b gave the lane an inline host of its own as `ictx.host`,
      so a provider that registers on `*ictx.host` runs its calls inline there, charged to init's
      process (S1-Q2; FF-3's init case).
    - It closes a race reachable before Stage 1: a future carried through Python from one pycvc
      `Exec` to another and awaited there ran the first `Exec`'s `finish_async_call` on the
      second's thread, or waited to the idle limit. Now it raises at once (FF-1, FF-3).
- **The step rate** for charging is read again only when the tree changed (`perf/sched`).
- **Host-thread natives** (B22). A host registers each intrinsic provider with a kind,
  `provider_affinity` (`intrinsics.h`): `any`, `host_proxied` or `host_only`. Each provider's owner
  decides its kind; S1-Q7 records the decisions for Ariadne's providers, which Stage 1b registers
  (below). Stage 1a built the mechanism: `exec_group::install(env, name, fn, kind)` binds a native
  into one of the group's envs by its kind.

| Kind | In group 0 or a host-affinity group | In an `any` group |
|---|---|---|
| `any` | installed as it is | installed as it is; it runs on whatever thread pumps the group, so it touches only its own context, thread-safe tables and the tree through pinned lookups |
| `host_proxied` | installed as it is | installed as a wrapper. On the `pump_all`'s caller, at threads 0, or with no `pump_all` active, the body runs inline, as today. On a worker the process parks through `begin_external`, and the body goes onto the call's host-call queue with copies of its arguments (a share-mode copy, made as the process's deep-data work when one is compound); the caller runs it once the group's pump has ended, and the result comes back as a completion at the group's next pump |
| `host_only` (`register_action_intrinsics`'s kind when given none) | installed as it is | not installed: a call is a NameError at any thread count, never a race, and `has?` does not see it. A document whose env would hold one is `affinity::host` (4.12) |

- **Ariadne's providers** (built in Stage 1b, S1-Q7, S1.9). `register_action_intrinsics(app,
  provider, affinity)` (`ariadne.h`) adds a provider of the program lanes' natives with its kind
  to the app's own list; the forms without a kind register it `host_only`. Where an env is built,
  `install_providers` (`ariadne.cpp`) runs the registered providers in registration order:
  - in group 0, in a host-affinity group, in the init lane and in the name probes, each runs as it
    is, into the env;
  - in an `any` group, an `any` provider runs as it is and a `host_only` one does not run; a
    `host_proxied` one runs into a scratch scope over the env (`environment::extend`, so it reads
    the env's bindings as it would), and what it bound there is moved into the env: each native
    through `exec_group::install(..., host_proxied)`, anything else (a constant) as it is.

  As built, by provider:

| Provider | Natives | Kind | Why |
|---|---|---|---|
| AppRuntime (`AppRuntime::ensure_intrinsics`; seven providers, registered in the order the one provider bound them before, so a name two of them bind resolves as it did, the later one winning: `AriadneHostVerbs.ANameTwoTablesBindResolvesInTheirOrder`) | 1. the host's verbs (`register_verb`) | `host_only` | arbitrary host code, run in the calling step |
| | 2. `(quit [code])` (`register_quit`) | `host_proxied` | it writes the AppRuntime's close request and exit code |
| | 3. the host's async verbs (`register_async_verb`) | `any` | libcvc's own marshal on `*ictx.host`; the work runs on the app's background pool; the done channel goes to the calling program's group |
| | 4. `data-load`, `data-reload`, `data-status`, `data-files`, `data-file` | `host_proxied` | they walk the tree raw (`data.cpp`); the body binds them only for a program of its own app, which it checks through a pinned walk |
| | 5. the host's functions (`register_function`) | `host_only` | the host's code, run in the calling step; pycvc's take the GIL |
| | 6. the host's async functions (`register_async_function`) | `host_only` | the host's marshal runs in the calling step (pycvc's takes the GIL; a C++ host's is unaudited), on `*ictx.host` |
| | 7. the host's async work functions (`register_async_work_function`; a script lane's `export_activation`, master's #1093) | `host_only` | the work's start runs in the calling step (an exported call starts its activation there, `Runtime::start_process`), on `*ictx.host`; it finishes the call through the host it was made on (`async_call::starting_host`); where that host can't park (init:'s, inline), the blocking form instead (master's #1094); where it parks, `(name ...)` is the sliced form, the calling program's own work (master's #1105) |
| net (`register_net_intrinsics`) | `http-get`, `http-request`, `fetch` (and their `-async` forms), `ws-*`, `net-allowed?`, `net-auth-*` | `any` | the kernels run on the app's background pool and complete on their call's scheduler; the WebSocket table and each connection's sends lock; the policy is immutable once the host made it; and the credential store locks: every `CredentialStore` member that reads or writes its entries, bindings or renewers holds its mutex, so `net-allowed?` and `net-auth-*` need no host thread (`net-auth-forget` and `-clear` do blocking file I/O on whatever thread pumps: a cost, not a race) |
| | `open-url` | `host_proxied` | it opens a tab in the user's browser |
| streams (`register_stream_intrinsics`) | `stream-open`, `stream-info`, `stream-close` | `any` | mutex-guarded tables; descriptors written through pinned walks; a stream posts to the group it was opened in |
| room (`register_room_intrinsics`) | `room-*` and `chat-*` | `host_proxied` | they write the app-global room tree raw, and use a room's chat client outside the room table's lock |
| room-media (`register_room_media_intrinsics`) | `room-publish`, `room-subscribe` | `host_proxied` | they write the room tree raw and drive the bridge; a subscribed proxy posts to the caller's group |
| oidc (`enable_oidc`) | `oidc-*` | `host_proxied` | process-global sessions, stepped by `pump_oidc` on the host thread; a document that uses oidc stays in group 0 (S1-Q6) |
| `nav_compute` (the example) | `nav-step` | `any` | its marshal touches nothing, and its kernel runs on the background pool |
| The backend's verbs (the Runtime's own, `Runtime::Impl::install_backend_verbs`) | `clipboard-set` (where the backend has a clipboard), `fullscreen` | `host_proxied` | they call the backend, whose window and clipboard are the host thread's; `has?` sees them in an `any` group too (`AriadneIsolatedDocs.TheBackendsVerbsRunOnTheFramesThread`) |
| cvcGL media and stream verbs (`media_verbs.cpp`, `stream_verbs.cpp`) | `stream-capture-camera`, `stream-capture-audio`, `stream-play-audio`, `stream-set-muted`, `gl-bind-stream` | `host_only` (one-argument form) | SDL's camera and audio on their owner thread, GL on the render thread |

- **Provider rules.**
  - A provider's body runs where the env is built: on the host thread for an action, a resident,
    the init lane and the probes, but on the pumping thread for a spawned child (4.15), which is a
    worker in an `any` group. So a provider an `any` group runs (`any` or `host_proxied`) has a
    thread-safe body: every body above reads only its context, what it captured (immutable, atomic,
    weak or locked state) and AppRuntime's verb table, under that table's lock (4.12), and binds
    natives into the new env.
  - A `host_proxied` provider binds synchronous natives only: an async one, wrapped, would run its
    kernel inline on the caller, where its host has no running process, uncharged.
  - What a provider binds that is not a native goes into an `any` group's env as it is, so it is
    made fresh for each env, or is immutable: two groups' programs must never share a mutable value
    (invariant G, 4.3). No shipped provider binds one.
  - `has?` in an `any` group, which a worker may pump, answers from what `drain_begin` snapshotted
    on the host thread (the host's facts, and the names the providers bind there by kind), read
    under the Runtime's names lock; elsewhere it answers live.
  - A host has no way to give AppRuntime's own verbs and functions a kind other than the table's:
    one that wants an `any` verb registers its own provider with `register_action_intrinsics(...,
    any)`, as `nav_compute` does (a follow-up, 7).
  - A provider, or an AppRuntime verb or function, registered after a document's programs have
    started is bound in an isolated `any` group, if it is `host_only`, only once the placement is
    decided again: the first time nothing of the document runs, at the latest at its next restart
    (4.12, "Late registrations"); an `any` or `host_proxied` one binds in the next env built, since
    `install_providers` reads the registry each time.
- **A started call's finisher.** Work that answers by itself (an `async_work` whose `start` hands
  the call to a transport, which settles it from its own thread: net's started requests, a
  started `fetch`) keeps its host's finisher (`async_host::make_finisher()`), never the host.
  `finisher::finish` posts a settled call's outcome as `async_host::finish` does, but under the
  host's state lock and only while the host has not begun to shut down; the shutdown sets its
  stopped flag under that lock. So a transport answering as an isolated document's group goes
  touches neither the group's host nor its scheduler, and the shutdown has cancelled the call.
  The lock order is the host's state lock, then the scheduler's ingress lock. A work checks
  `cancel_requested()` before it settles, so a cancelled call keeps the outcome its cancel gave it
  (`AsyncCalls.AStartedCallsFinisherPostsOnlyWhileItsHostIsUp`).
  - The shutdown (`async_host::shutdown`) sets `stopped` and takes the live calls under that lock,
    releases it, cancels them, so their `on_cancel` hooks run outside it, then waits under the lock
    for the running kernels (R1b skeptic m6). A hook may therefore wait for a thread that is
    finishing the call through a finisher (an abort that joins its transport's thread): that thread
    gets the lock, sees `stopped` and posts nothing
    (`AsyncCalls.AShutdownsCancelHookMayWaitForAThreadInTheFinisher`).
- **A cancelled request stops its transfer** (R1b skeptic m1). `cvc::net::HttpRequest::cancelled`
  is a predicate asked before a request is made and about once a second while it is in flight
  (the curl backend's progress callback); once it returns true the transfer is aborted and the
  reply is a transport failure, "cvc::net: cancelled". Code that builds requests for its caller
  takes the predicate as an argument: `resolve(uri, base, cancelled)` hands it to the scheme's
  `UriHandler` (the http(s) handlers, cached or not, set it on their request), and
  `CredentialRenewer::renew(rejected, cancelled)` sets it on the refresh. Net's blocking
  kernels, `http_kernel` and `uri_fetch_kernel`, pass their call's `cancel_requested()`. (It
  began as `HttpCancelScope`, a `thread_local` over the kernel; that carried context past the
  functions' arguments, which the no-singletons rule forbids, so it went.) A kill
  (`cancel_async_calls`) and a host's shutdown both cancel the call, so either ends the transfer
  in about a second, not at the server's reply or the request's timeout (30 s by default): a
  teardown that waits for its group's kernels waits that long at most for a request. The
  browser's fetch ignores the predicate (`AriadneNetHost.ACancelledRequestStopsItsTransfer`,
  `NetCurlBackend.ACancelledRequestStopsItsTransfer`).
- **Whole WebSocket frames.** `ws-send` is `any`, so two documents on two workers may send on one
  connection at once (a handle is app-global). A connection sends one frame at a time, whole: its
  send lock is held across the frame (a close's too), while its handle lock is still taken per
  chunk so the reader goes on receiving between them; libcurl continues a partly sent frame with
  the next send's bytes, so messages used to interleave
  (`NetWsClient.TwoThreadsSendingOnOneConnectionSendWholeMessages`).

- **The host-call queue** (`exec_pool::run_on_host`):
  - A body queued on a worker goes onto the queue of the `pump_all` call pumping the worker's
    group. That call's caller runs the queue while it waits at the join, and `pump_all` returns
    only with its queue empty.
  - The caller runs a parked process's body once the group's pump has ended (the process resumes
    at the group's next pump anyway), so no thread pumps the group meanwhile, and the body runs
    with the group's registry and random stream bound.
  - At a point that cannot park (`can_await()` false: a generator body), a `host_proxied` call
    blocks its worker until the caller has run it (`detail::run_on_host_waiting`): the caller runs
    such a body as soon as it services its queue, while the worker waits.
  - When the calling thread is that `pump_all`'s caller, or no `pump_all` pumps its group, the body
    runs inline, so a caller never waits for itself, at threads 0 included.
  - A group's teardown purges the group's entries unrun (its kills have settled their calls as
    cancelled), waiting for one its caller is running now.
  - A body, like a deferred `on_complete` callback, must not call `pump_all` or destroy a group
    (debug assertions).
  - **Rule: the host thread never blocks without servicing the host-call queue.** In Stage 1 that
    is the fork-join join; Stage 2 adds `kill_owner`, `with_group` waits and `wait_for_work`. A
    blocked worker therefore always makes progress.
  - Each `pump_all` call has its own queue, which only its caller services (S1-Q11), so a host
    thread never waits behind another caller's batch. So with two callers at once (an Ariadne
    frame beside an `ExecPool.run_all`, or two host threads), one provider's `host_proxied` bodies
    can run on two threads at once: "the host thread" is the caller of the body's `pump_all`, not
    one fixed thread, and a provider whose state only one thread may touch serializes its bodies
    itself.
  - libcvc's own builtins and stdlib are `any`. pycvc's Python natives are `any`, because
    `call_python` takes the GIL itself, so they serialize on the GIL.
- **`on_complete` callbacks** (5 item 9; built in Stage 1a). In a threaded `pump_all` (workers,
  not deterministic) a callback given through `exec_group::execute` is queued, whichever thread
  pumps its group, a worker or the caller, and the `pump_all`'s caller runs it after the join:
  group by group in the order their pumps ended, each group's in completion order, with the
  group's registry and random stream bound. So whether a callback is deferred does not depend on
  which thread took its group, and none runs while a worker pumps a group of the same call. If one
  throws, the rest still run, and `pump_all` rethrows the first. Before, a throw left the pump at
  once; the group's other processes now finish their pump first. The values a callback receives
  are group-owned, so a host drops them between pumps. At threads 0, in deterministic mode, and
  outside a `pump_all`, callbacks run inline, as today, and a throw ends the pump (rethrown from
  `pump_all` after the join); so do `host_first`'s. A callback given to the group's scheduler
  directly (`sched().execute`, which also copies no AST and defaults no owner) runs where the group
  is pumped, inside the pump, on a worker included. In continuous mode (Stage 2) they run in
  `pump_host`.
- **Kernel pools** (C's graft, Stage 2). `perf/async` gives one persistent background pool.
  - Once group workers compete with kernels for cores, the pool is split in two:
    - `background_cpu`, sized to the cores minus the group workers;
    - `background_io`, larger, for blocking kernels.
  - `executor::background` keeps meaning the first, and a registration option picks the second.
  - This reverses settled question 4 of
    [`docs/STATE_EXEC_EXCEPTIONS.md`](STATE_EXEC_EXCEPTIONS.md) ("There's no separate I/O pool",
    line 840; section 3.5) from Stage 2 on, as the user decided (Q9).
- **Blocking steps** (B21). An inline kernel or a `call->wait` at a point that cannot park
  (`intrinsics.cpp:537-542`) blocks only the worker pumping that group.
  - Other workers keep pumping other groups, which is better than today, where it stalls everything.
  - A long-step monitor logs any step over 1 ms with the native's name, as BEAM's `long_schedule`
    does.
  - In continuous mode, a worker blocked over 10 ms is excluded from the active count, and the
    pool may start one compensating worker (Go's syscall handoff). Stage 2 makes this optional.

### 4.10 Budgets and time accounting

- **Per process: unchanged.** `max_steps`, `max_time`, `max_memory`, `max_alloc_bytes`,
  `max_step_time`, the message limits and async charges
  ([`docs/STATE_EXEC_EXCEPTIONS.md`](STATE_EXEC_EXCEPTIONS.md) 3.8) all have one writer, the
  group's pumping thread.
  - A step runs wholly on one thread, so `thread_cpu_seconds()` deltas stay valid
    (`process.h:182-186`; `types.h:322-324`).
  - With Stage S's runs (4.4), a run is on one thread too, and a group changes threads only
    between pumps. In a run, `max_steps` and the message limits are checked at every step, so they
    stay exact, and the
    process's time at the 16-step clock readings, both halves of `max_time` (the run's CPU so far
    is read there while it charges CPU). The deadline guard, armed once per run with the time left,
    cuts only a native's long loop: no deadline is checked between a run's plain steps, so it does
    not bound the run. CPU time is charged at the run's end, each step's cap (`max_time / 32`)
    scaled by the run's steps. A process with `max_step_time` or `max_memory` keeps one step a
    pick.
- **Lock waits** are parks and are not charged, like `msg-recv`. Ariadne actions, and the children
  they spawn, get a default lock timeout (`ariadne.exec.lock_timeout`, 1.0 s), since their
  budgets never end a park. Init scripts never wait: they run synchronously on the host thread,
  where no resident can release, so their context forbids lock waits (`lock_wait_allowed`
  false), as does the context of every child they spawn, and a contended init request fails at
  once, whatever its `:timeout`. Uncontended `with-lock` adds no step, so step budgets are
  unchanged ([`docs/STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md) 5.5, 8.8).
- **Per pump.** Each group gets its own pump budget (`pump_budget`): Ariadne's 200000 steps and
  0.1 s, which `app::exec_pump_frame()` gives too, or `Exec`'s slice (2000000 steps and 0.05 s,
  which `ExecPool.run_all` gives each pooled `Exec` per round). `time_budget` charges thread CPU
  past its half-way point, so a pump descheduled by oversubscription still completes its quick
  work (`types.cpp:94-110`; [`docs/ARIADNE.md`](ARIADNE.md) 2.2). With Stage S's runs, a run counts
  the steps it charged against the pump's step budget, clipped to what is left of it, and the
  pump's time budget is checked at the run's clock readings (steps 2, 4, 8, 16, then every 16th),
  each charged as the steps since the last check (`time_budget::spent(now, checks)`), so a run of
  costly steps stretches the budget no more than one step a pick does.
- **Per frame** (built in Stage 1a). `pump_all(groups, budget, deadline)` also takes a wall
  deadline: `exec_pump_frame` passes 0.1 s from its entry, `ExecPool.run_all` none.
  - A group starts only if the deadline has not passed, and its pump's time budget is the smaller
    of its own and the time left. A group not started counts in `stats().skipped`.
  - Groups that did not start are put first in the next frame's order.
  - With more groups than threads, the frame stays bounded by about 0.1 s plus one pump, not by
    ceil(G / threads) x 0.1 s.
  - Group 0's pump in `exec_pump_frame` (4.12) does not starve the frame groups: a frame group
    skipped in one frame starts first in the next, before group 0 whenever the caller pumps it
    (4.4), so a busy group 0 delays a frame group by one frame at most, at threads 0 and for
    host-affinity groups as well.
- **Ariadne's budget changes meaning only for hosts that opt in** (built in Stage 1b). With plain
  `drain()`, 200000 steps and 0.1 s cover every document in group 0 together, once per
  `Runtime::drain`; a host that calls `exec_pump_frame` gives group 0 one such pump a frame (5
  item 10); and an isolated document gets its own 200000 and 0.1 s per frame (5 item 11).
- **Hybrid CPUs.** A step on an E-core is about 1.6x slower, so `max_time` and Ariadne's 0.1 s
  action cap burn faster there. Step budgets are unaffected. P-core placement first (4.4).
- **`math.random`** (B26; built in Stage 1a). It draws from the `random_stream` (`stdlib.h`) bound
  on the calling thread, else from the thread's own engine, which bare evaluators and bare
  schedulers keep. A group's stream, like the app scheduler's and each pycvc `Exec`'s, is set on
  its scheduler (`set_random_stream`), which binds it with the registry (4.1), so it does not
  change when the pumping thread does. It is seeded on first use: from `exec_group::options::seed`
  when given, else from a hash of the group's name in deterministic mode, else from eight
  `std::random_device` words (the thread's engine takes one, which allows only 2^32 streams).
  `uniform_real_distribution` differs between standard libraries, so a seeded stream repeats on
  one platform, not across them.

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
   pump. That holds for a group the frame starts: with more groups than pumping threads and
   pumps that outlast the frame's deadline, a group not started by then is skipped (4.10), starts
   first next frame, and its action completes then. As built in Stage 1b, one more exception: in
   an isolated `any` document whose group a worker pumps, an action that calls a `host_proxied`
   verb (`quit`, the `data-*` verbs, `open-url`, the room and chat verbs; oidc's never run there,
   S1-Q6) parks at the call, and the frame's caller runs the verb after the group's pump, so the
   action resumes at the group's next pump: it completes a frame later. When the frame's thread
   pumps that group (a lone isolated document under `drain()`, usually, 4.12), the verb runs
   inline, as in group 0.
8. State-tree writes are atomic per node, as today.
9. `print` writes whole lines: it formats the line outside any lock (a list argument's in its
   data job), then writes it with its newline in one `std::cout.write` under one process-wide
   mutex, never held across a data job or DSL code (`builtin_print`, `write_print_line`; built in
   Stage 0). Its text and charging are unchanged.
10. Conflicting lock requests are granted in order of arrival, whichever groups their processes
    are in, and a grant is never lost or released twice in an expiry, kill, reap or move race: the
    waiter takes it, or its end releases it.

**Not kept between different groups:**
1. Interleaving and relative progress. Deterministic mode reproduces them.
2. The order of messages from different groups, and the position of a requeued message.
3. The immediacy of cross-group kill, pause, resume, signal and `set-priority`, and of a lock
   grant to a waiter in another group. They land at the target's next step boundary (a grant at
   its group's next drain), and cross-group `ps` can lag one slice.
4. Strict priority across groups: each group's policy holds inside it. Between groups, Stage 2
   weights a group's pump share by its best runnable priority (4.4), and a lock boost raises a
   holder inside its own group.
5. That a reader in one group sees all of another group's writes from one step at once, across
   nodes.

### 4.12 Host APIs and how the hosts use them

**`exec_group`** (`inc/cvc/state/state_exec/exec_group.h`,
`src/cvc/state/state_exec/exec_group.cpp`; built in Stage 1a):

```cpp
enum class group_affinity { any, host };
struct pump_budget { std::uint64_t steps; double seconds; };  // seconds: infinity = no limit
struct pump_result { std::uint64_t steps = 0; int woken = 0; bool failed = false; };
class exec_group {
 public:
  struct options {
    std::string name;                       // logs; the random seed in deterministic mode
    std::string owner;                      // the owner tag of what it runs
    group_affinity affinity = group_affinity::any;
    cvc::app *app = nullptr;                // settings, lock manager, pid source, kernel pool
    cvc::state *watch_root = nullptr;       // null: the app's root
    environment_ptr base;                   // the scope its global env extends; null: the builtins
    std::optional<std::uint64_t> seed;      // math.random's seed; unset: random_device words
    cvc::thread_class thread_class = cvc::thread_class::normal; // L4 and Stage 2; ignored
    std::vector<std::string> channel_prefixes;                  // Stage 2; ignored
    std::function<void(exec_group &)> pump_hook_for_test;       // GR-12
    std::function<void(exec_group &)> teardown_hook_for_test;   // GL-2
  };
  explicit exec_group(options);
  ~exec_group();                            // the teardown of 4.2
  async_scheduler &sched();
  async_host &host();                       // built on first use
  pump_result pump(const pump_budget &);    // drain_ingress, wake_awaiting, sync_run_steps
  bool wants_pump() const;                  // runnable, a frame-waiter, ingress, a timer due
  int execute(const std::string &script, const execute_options & = {}); // the group's owner
  int execute(const value_t &expr, const execute_options & = {}); // copies the AST (B12)
  void install(const environment_ptr &env, const std::string &name, native_fn,
               provider_affinity);          // a native by its kind (4.9)
  bool failed() const;                      // a pump failed it (4.17)
  cycle_registry *registry() const;
  static exec_group *current();             // the group this thread is pumping, or null
  const options &opts() const;
};
```

- **Construction** builds the group's scheduler as `app::exec_scheduler()` builds the app's: the
  policy from `state_exec.policy`, the watch root (`options::watch_root`, else the app's root), the
  host parser and data defaults, the base env (4.2), the app's pid source (4.5), its own registry
  and random stream (4.7, 4.10), and the app's lock manager in the manager's own mode (S1-Q1,
  4.13). With no app, it has no pid source and no lock manager.
- **`execute(value_t)`** copies the AST's lists and dicts, keeping their sharing and cycles, and
  shares its atoms (B12). It copies always, not only when the value is shared, since the caller's
  reference is always there: so two groups, and the host's AST, never share a quoted literal or the
  closure-body cache's keys. A script is read inside its process already. Both forms submit with
  the group's `options::owner` when the `execute_options` give none, so a host's
  `kill_owner(group.opts().owner)` reaches what the group runs. A host that submits through
  `sched()` directly gets none of this: no copy, no owner, no deferred `on_complete` (4.9).
- **`wants_pump()`** is true when a pump has work: a runnable process, a process awaiting the next
  frame (`awaiting_frame_count()`, which the pump's `wake_awaiting` readies), or something
  `wait_for_work` would wake for, already due (pending ingress, a lock post, a timer due, a call to
  charge).
- A value a host posts to a group's scheduler (`post_message`) is handed over, moved in: the host
  keeps no reference to a compound value it posted, and never posts one value to two groups
  (invariant G, 4.3). Nothing checks it in Stage 1 (`CVC_EXEC_VERIFY`'s tagging is not built), so
  a value shared that way is a data race in a release build.
- td's `with_group` (continuous mode) is Stage 2's, and not declared.

**`exec_pool`** (`inc/cvc/core/exec_pool.h`, `src/cvc/core/exec_pool.cpp`; Stage 1a's fork-join
half; `app::exec_pool()`):

```cpp
struct pool_options {
  static constexpr unsigned auto_threads = unsigned(-1);
  unsigned threads = 0;                     // workers besides the caller; 0 = every pump inline
  bool deterministic = false;
  double inline_below = 100e-6;             // adaptive helper wake (4.4)
  cpu_set placement = cpu_set::performance;
  cvc::thread_class worker_class = cvc::thread_class::normal; // L4; not applied in Stage 1
};
struct pool_stats { std::uint64_t pumps, helper_wakes, skipped, worker_pumps;
                    unsigned effective_threads; };
class exec_pool {
 public:
  explicit exec_pool(const pool_options &);
  ~exec_pool();                             // stops the workers after their current pumps
  // Stage 1: fork-join
  void pump_all(std::span<exec_group *const>, const pump_budget &, clock::time_point deadline);
  void pump_all(std::span<exec_group *const>, const pump_budget &, clock::time_point deadline,
                const std::function<void()> &host_first); // exec_pump_frame's group 0
  unsigned threads() const;                 // the workers running, after the clamps
  pool_stats stats() const;
  std::vector<exec_group *> failed_groups() const; // this thread's last pump_all on this pool
  void run_on_host(std::function<void()>);  // the host-call queue (4.9)
  bool in_pump_all() const;                 // as a call's caller, or a worker pumping its group
  const pool_options &options() const;      // as given, before the clamps
  bool deterministic() const;               // its flag, or CVC_EXEC_DETERMINISTIC=1 at build
  std::vector<std::string> take_notes();    // the clamps and failed workers, for the owner to log
  void set_last_pump_for_test(exec_group &, double seconds); // GR-9
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
// (A pool's worker pumps are its stats().worker_pumps; no process-wide total is kept.)
```

Stage 2's `pool_options` add `slice_seconds` and `slice_steps` (2 ms, 20000) and
`priority_weights` (4.4). The pool owns no group: a group's owner destroys it before the pool goes
(4.17). A group whose lock home attached to a single-mode manager is marked caller-only
(`group_pool_link::caller_only`), and every pool pumps it on the caller of `pump_all` only, as a
host-affinity group, logging it once per pool (a debug build asserts when a pool with workers is
given one), so a pool built directly, or one app's pool pumping another app's groups, never steps
a single manager's homes on two threads (S1-Q1).

**The app** (`inc/cvc/core/app.h`; Stage 1a; [`docs/APP_API.md`](APP_API.md)):
- `exec_pool()` builds the app's one pool at its first call, under its own mutex, from the tree
  (4.13); `exec_pool(const pool_options &)` builds it from the options given, which apply only to
  the call that builds it: a later call keeps the built pool and logs that its options were not
  applied. Asking for threads above 0 before anything has made the lock manager makes that manager
  shared, for this app only (S1-Q1). `exec_pool_probe()` returns the pool if it is built, and
  never builds it.
- `exec_pump_frame()` is one frame's pump of the app's groups: one `pump_all`, with a 0.1 s
  deadline from its entry and Ariadne's budget (200000 steps, 0.1 s), over every registered frame
  group, with group 0, `exec_scheduler()`, pumped first on the calling thread as a host-affinity
  group (`host_first`: its token taken, `drain_ingress`, `wake_awaiting`, `sync_run_steps`) once it
  is built and holds a live process; a frame group skipped last frame starts before it when the
  caller pumps it (4.4). In deterministic mode (the pool's, or with no pool the tree's and
  `CVC_EXEC_DETERMINISTIC`) group 0 is pumped with steps only and its stream seeded from "group0".
  It builds no pool when no frame group is registered, and never builds group 0. Then
  `exec_pump_generation()` advances, which Stage 1b's `drain_end` reads.
  Host thread, outside a pump. `cvc::pump_exec_frame(sched, lanes)` (`async_lane.h`) is another
  driver of one scheduler, `exec_scheduler()` included, which pumps no group and does not advance
  the generation.
- `register_frame_group(group)` returns a `frame_group_handle` (movable, RAII): every
  `exec_pump_frame()` pumps the group until the handle is reset or destroyed, which the host does
  before the group goes, on the host thread, outside a pump.
- `exec_pid_source()` is the pid source group 0 and every `exec_group` of the app share (4.5).

`async_scheduler` gains:
- `async_scheduler(scheduling_policy, environment_ptr base)`, the base-env constructor (4.2);
- `set_pid_source(std::shared_ptr<std::atomic<int>>)` (4.5);
- `set_cycle_registry`, `registry()` and `set_random_stream` (4.7, 4.10), which the sync
  `scheduler` has too;
- `pump_owner_scope` and `pumped_elsewhere()` (4.1), on both schedulers;
- `close_lock_port()` (4.2), on both schedulers;
- `kill(pid, reason)` (4.5);
- (Stage 2) an ingress entry kind for control and router messages;
- (Stage 3) `adopt(process_ptr)`: like `restore`, keeping the pid and the process's state, its lock
  holds included;
- a lock port for `wake` and `relevel` posts, with its own mutex, queue and pending flag and
  sharing the scheduler's wakeup, applied in `drain` (4.19; built by the locks track's L3).

The sync `scheduler` has `post_control` (a mutex-guarded queue applied between steps) and atomic
stop flags since Stage 0. The same queue carries `set_priority`. As built, on both schedulers:
- **`stop()`** may come from any thread: `running_` and `stop_requested_` are lock-free
  `std::atomic<bool>`s. A run clears `stop_requested_` before it publishes `running_`, so a
  `stop()` made once `is_running()` reads true is never lost, and one made with no run active is
  discarded by the next run. The sync `stop()` also notifies the idle wait, so it is no longer
  async-signal-safe (the notify takes a lock on POSIX). It takes its own reference to that
  wakeup before it stores the flag, so the scheduler may be destroyed as soon as `run()` returns,
  while `stop()` is still notifying. The run reads the flag with acquire, pairing with `stop()`'s
  release store, so that store happens before the destruction (a relaxed read left the two
  unordered, and clang-tsan reported the race). The async run loop never waits, so its `stop()` notifies
  nothing.
- **`post_control`'s kinds** are `kill` (reason `killed_by_user`), `pause`, `resume`, `signal`
  (`"SIGKILL"` kills) and `set_priority`, each the direct call's host path, unchecked, and
  `query`. An unknown pid is found, and refused, on the scheduler's thread.
- **`done(bool)`**, optional, is called once on the scheduler's thread with what the direct call
  returned, or with false from the scheduler's destructor for a command never applied. A query
  has none. It must not throw, block or take the GIL. It is moved out of the command before it is
  called, so a `done` that throws anyway is not called again. Each destructor refuses what is
  queued until nothing is left, so a command a `done(false)` posts during the destruction is
  refused too (`done(false)`), never dropped.
- **Order.** First in, first out, applied at the next step boundary: the sync scheduler at the
  top of `step()` and of each `run()` iteration, before its `has_runnable()` test (so a resume of
  the only process, or a kill of the last, takes effect in a run with nothing else to do); the
  async scheduler in its next drain, after the drain's messages and completions, whatever the
  order they were all posted in (Stage 2 unifies them if it needs a total order). Commands are
  applied one at a time: one that throws (a kill releases locks and cancels calls, which can
  write the tree) gets `done(false)`, and the rest go back to the front of the queue, in order,
  before the exception propagates. The async drain puts back its unapplied messages and
  completions the same way.
- **Wakes.** A post ends the sync scheduler's idle `run()` wait (it notifies the idle wakeup, now
  made with the scheduler), as a post wakes the async scheduler's `wait_for_work()`.
  `has_pending_control()` is true from a post until the batch that took the command has been
  applied, refused or put back, and then for as long as anything is still queued: both
  schedulers set it from what is queued only once a batch is done with. It lets a loop that
  steps only for runnable processes step for a command too: the CLI's serve loop steps on
  `has_runnable() || has_pending_control()`. A step there that throws (a posted command that
  throws) is logged on stderr as `exec: <what>`, the server goes on serving, and the commands
  after it stay queued.

On the sync scheduler, lock wakes and relevels go to its lock port, which shares its wakeup
(made when the scheduler is built, no longer lazily in `idle_wait`): `idle_wait` also ends as
soon as a lock post is queued and returns so `run()` applies it. Otherwise a `run()` left with
only lock waiters behaves as with only message waiters
(`perf/sched`): one bounded by steps returns at once when no timer is set or its steps would run
out first, and otherwise it waits for a timer, a lock post or the end of its time budget, so a
`run(N)` frame pump still returns on time ([`docs/STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md) 6.1,
9.2).

**Ariadne** (built in Stage 1b; `ariadne.h`, `app_runtime.h`; [`docs/ARIADNE.md`](ARIADNE.md) 2.5
and 9.1).
- **The drain in two halves** (B29, S1.10). `Runtime::drain()` is `drain_begin()` followed by
  `drain_end()`:
  - `drain_begin()` is the frame's host work. It notes the app's pump generation
    (`app.exec_pump_generation()`), decides where the document runs if that is open (below),
    takes a step of a sign-in in progress (`pump_oidc`), runs the key bindings and the queued
    events (a program action is submitted, a bare event name runs the host's C++ handler), starts
    the tick resident and posts it the frame's tick, starts the input residents, and, for an `any`
    group, snapshots what `has?` answers from (4.9). It does not pump.
  - `drain_end()` is the pump half, then the sweep. It pumps (`Runtime::Impl::pump_frame`) unless
    `app.exec_pump_generation()` moved since this Runtime's `drain_begin()`, that is, unless the
    host called `app.exec_pump_frame()` in between; then it sweeps the finished actions, residents
    and children where the document's processes run. `drain_end()` alone runs `drain_begin()`
    first.
  - `drain()`'s pump is what it was, plus the document's group: group 0 a bounded slice
    (`drain_ingress`; `wake_awaiting` once per app frame however many Runtimes drain, through
    `starts_frame()` and the app's pump clock, which stay for this path; `sync_run(200000, 0.1)`)
    when this Runtime used group 0 or any process lives on it, never building it otherwise; then
    the document's group, if it has one, through `app.exec_pool().pump_all` with the same budget
    and a 0.1 s deadline. So a host with one Runtime that opts in keeps calling `drain()`.
- **Hosts with several Runtimes** call `drain_begin()` on each, then `app.exec_pump_frame()` once,
  then `drain_end()` on each (`AppRuntime::drain_begin` and `drain_end` too, which do the scene
  work, the warnings and the frame hooks after `drain_end`). That one call is the frame's only
  pump: group 0 once, on the calling thread (its token, `drain_ingress`, `wake_awaiting`,
  `sync_run_steps` with 200000 steps and what is left of the 0.1 s), and every isolated document's
  group, which its Runtime registered as a frame group, in one `pump_all` ("The app", above).
  - Group 0 gets one `sync_run` a frame instead of one per Runtime: its budget is 200000 steps a
    frame, not N x 200000, and with several documents in it an await crosses exactly one frame
    (AR-2; 5 item 10).
  - A host does one or the other: `drain()` on one Runtime beside the split on another pumps a
    registered group twice a frame.
  - The shipped hosts (`ari`, `arigl`, `AriRuntime`) drive one Runtime each with `drain()`, which
    is exactly what it was for them.
- **Host-thread calls** (S1.10). What a Runtime does to its document's scheduler runs on the host
  thread, between pumps, never inside a `pump_all` of the app's pool (from a native, an
  `on_complete` callback or a `host_proxied` body): `drain_begin` and the sweep; render, which
  starts mount ticks, runs mount `init:`s and, in its reconcile, kills residents; `set_root` and
  `AppRuntime::load`; `set_tick/key/pointer/signal_program`; `post_input` and `post_signal`, which
  may start their residents, then post; the placement and its moves; and the destructors. The
  headers say so of each. `post_input` and `post_signal` used to be called thread-safe: a host that
  catches an OS signal on another thread hands it to the host thread. `run_program` is a host-thread
  call too, but it only queues the program for the next `drain_begin`, so it is safe inside a pump.
  - Debug builds assert it (`assert_outside_pump`: no `pump_all` of the app's pool on this thread,
    read through `exec_pool_probe()`, which never builds the pool) in render, `post_input`,
    `post_signal` and `set_*_program`, the calls that start or kill. Render's is AR-3's latch.
  - Tearing a Runtime down, and moving a document out of its group, are refused while a `pump_all`
    of the app's pool holds that group, from any thread, in every build (`refuse_if_held`: the
    group's `pool_link().call` is set from the `pump_all`'s entry until it returns; the refusal is
    printed on stderr and logged, then the process aborts, before anything is touched): their kills
    would race a worker on the group, and the group's destruction would abort anyway. A host verb
    that closes or reloads an isolated document, run in group 0 while the same `exec_pump_frame`
    pumps that document's group, is the case
    (`AriadneIsolatedDocsDeathTest.ATeardownInsideAPumpIsRefused`), and so is a teardown on another
    thread while the frame's pump holds the group
    (`ATeardownFromAnotherThreadWhileAPumpHoldsItsGroupIsRefused`). A group-0 document's teardown or
    move is never refused, nor one whose group no pump holds: a host group's native may tear down a
    group-0 document that never ran (`TearingDownAnUnpumpedDocumentInsideAPumpIsAllowed`).
- **Isolated documents** (`ariadne.exec.isolate_documents`, opt-in, S1-Q6). The setting is read
  from the app's tree (on for `1` or `true`, in any case) each time the placement is decided. With
  it on, the Runtime gives its document an `exec_group` of its own (name and owner the Runtime's
  owner tag, built on the app) and registers it as a frame group, when the document passes every
  rule, checked in this order (`Runtime::Impl::isolation_verdict`):
  1. a non-empty prefix;
  2. an enforced channel policy: a `channels:` block in strict lint mode (the default for a
     document with one; `AppRuntime::load` passes the mode to `set_channel_policy`);
  3. no `global: true` channel;
  4. no static `/` reference: no program of the document names a `/` channel as the string-literal
     first argument of a `msg-send`, `msg-recv` or `msg-pending`. The programs read are its lanes,
     its key bindings and functions, every widget's `on:`, `on_click`, `on_hover`, `on_drag`,
     `on_drag_start`, `on_drag_end` and `on_top`, each mount's `init:`, `on_tick:` and functions,
     and the `init:` and `tests:` that `AppRuntime::load` runs elsewhere and hands over
     (`set_document_programs`): more lanes than the loader's channel lint reads;
  5. no oidc: no literal `"oidc#..."` channel and no free `oidc-*` call (`free_heads`) in those
     programs, since oidc posts its flow's end to group 0;
  6. no writable `link:` hole whose target, resolved against its mount's parent scope, leaves the
     prefix (a read-only hole, or one inside, is fine).

  A program that does not parse is skipped there (its error shows when it runs). A document that
  fails a rule stays in group 0, and the first rule it failed is its reason:
  `Runtime::isolation_reason()` returns it (`""` when isolated), and with the setting on the
  app's log shows it when it changes (so once while it stays the same, however many frames the
  document runs). A provider that throws in the probe keeps the document in group 0 too, and so
  does a group that could not be made.
- **The group's affinity.** `host`, so pumped on the frame's thread (the `pump_all`'s caller), when
  its host declared a thread-affine prefix at, under or above the document's prefix, the root's
  empty prefix included (4.8), when its programs run host code inline
  (`Runtime::require_host_thread`, below), or when a `host_only` provider binds anything in an
  action's env (`host_only_bindings`, which runs the providers in a probe env: an AppRuntime with a
  host verb or function, for one). Otherwise `any`, which the pool's workers may pump. In arigl
  every document is `host`: `AriRuntime` declares its scene prefix, the document's own.
- **Scripting engines** (master's script-engine interface, #1091, merged into Stage 1b). An
  engine's activation is one of the document's processes (`Runtime::start_process`), which runs in
  the document's group like its actions. Its natives (`LaneProcess::extend`) run a slice of the
  engine's language inline in the calling step: they charge the slice to `current_process()`, look
  up the document's functions in the process's env, and share a per-language frame budget across
  documents. So they stay on the host thread. `AppRuntime::use_script_engine` declares
  `Runtime::require_host_thread`, and the document gets a `host` group. `start_process` declares it
  too for a process with natives of its own. It refuses one (it throws, and a hook's delivery
  warns) while the document still runs in an `any` group, which happens only when the declaration
  came after its programs started. The declaration follows the late-registration rule below.
  `ScriptLane::stop` and `running` reach the document's group (`current_group()`), not group 0.
  - **Exported calls.** An exported call (`ScriptLane::export_activation`, #1093) is a host async
    work function: AppRuntime's seventh provider, `host_only`, on the calling program's own host.
  - **Its start runs in the calling step,** on the host thread. It starts the activation there, so
    `start_process` may run from inside a pump of the document's group 0 or host group, and has no
    `assert_outside_pump`.
  - **It finishes on the host it was made on** (`async_call::starting_host()`), through that host's
    finisher, which posts nothing once the host has shut down. Finishing through another host
    would forget the call by a slot of the wrong host's list.
  - **Covered by** `ScriptEngineTest.AnIsolatedDocumentsExportedCallFinishesOnItsGroup` and
    `AsyncCalls.AStartSeesTheHostItWasMadeOn`.
  - **Where the program parks, `(name ...)` is the sliced form** (master's #1105): the activation
    runs as the calling program's own data job, a slice per data step of its process. The parked
    form above is then `(name-async ...)`'s. The seventh provider binds the sliced form where
    `host.parking_scheduler()` is non-null, so for group 0's programs and an isolated document's.
    The sliced form finds the calling process on the group the thread is pumping
    (`exec_group::current()`), else on group 0. Master looked only on group 0, where an isolated
    document's call found no process and raised RuntimeError. The same test covers both forms.
  - **An activation's lock** (master's #1126, `ActivationOptions::lock`) is a `with-lock` key that
    the activation's process holds, and that a sliced call takes in the calling process. The app's
    one lock manager serves every group (4.19), so the lock excludes across group 0 and isolated
    groups alike. Nothing here changes for it.
  - **A call where the program can't park** (master's #1094) gets the function's blocking form.
    That is `init:`, whose host runs its calls inline (S1-Q2). The seventh provider binds the
    blocking form where `host.parking_scheduler()` is null. It doesn't compare `ictx.sched` with
    group 0's scheduler, because an isolated document's actions and residents park on their
    group's host.
  - **The blocking form runs the activation to its end in the call** (`ScriptLane::run`), as a
    guest's `ari_init` does at load. `Runtime::pump` is `drain_end`'s pump and sweep: group 0 as
    `scheduler_to_pump` says, and the document's group through the app's pool. `run` refuses
    (RuntimeError) inside a step of either scheduler, or inside a frame's `pump_all`, which it
    would re-enter.
  - **When it runs:** at a first load, `init:` and `ari_init` run before the document is placed,
    so in group 0. At a reload they run in the document's group.
    `ScriptEngineTest.AnIsolatedDocumentsInitCallRunsInItsGroup` covers both.
  `ScriptEngineTest.AnIsolatedDocumentThatUsesAnEngineRunsOnTheHostThread` and
  `ScriptEngineTest.AProcessWithNativesIsRefusedInAWorkersGroup` (`ariadne_script_engine_test`)
  cover it.
- **What goes to the group.** Every process the Runtime starts runs in its group, and every post
  and kill goes there: its actions and residents (`make_action_context` gives them the group's
  scheduler and host), the children they spawn (4.15), its tick, key, pointer and signal posts,
  `stop_resident`, and the teardown's kills. Its settings reads stay on the host thread. Its
  providers are installed by kind (4.9), so a `host_proxied` verb on a worker completes a frame
  later (4.11 item 7). Its programs see only the group: `ps`, `inspect` and `kill` reach only the
  document's own processes (5 item 11), and pids stay unique app-wide.
- **When it is decided** (`ensure_placement`). At load (`AppRuntime::load` asks, through
  `isolated_group()`, once the document's programs, channel policy and tree, and its host's
  providers and declarations, are set), and again, before the next program starts, the next frame
  is drawn or the next `drain_begin` runs, after any of: `set_root`, a lane's program
  (`set_*_program`), `set_document_programs`, `set_channel_policy`, `set_key_bindings` or
  `declare_thread_affine_prefix`. A Runtime given no document decides when its first program
  starts. `isolated_group()` and `isolation_reason()` decide if that is open; `current_group()`
  returns what was last decided and decides nothing, and `AppRuntime::idle()` and `wait_idle()` use
  it, so a query never builds, moves or destroys the document's group.
- **Late registrations** (R1b skeptic m3). A host verb or provider registered after the placement
  makes it stale, by any of three paths: AppRuntime's `register_*` (through `refresh_program_names`),
  `register_action_intrinsics` (the providers' registry moves past the generation the placement
  read), or `require_host_thread`. A stale placement is decided again, at the same points (each `drain_begin`, render and
  submission tries), the first time nothing of the document runs where it runs
  (`nothing_running`: no live action, started resident or mount tick, nor spawned child), so a
  move then kills nothing; at the latest it is decided at the document's next restart (a load,
  `set_root` or `set_*_program`, which kill and restart anyway). A registration between `load()`
  and the first frame, after an `init:` too, therefore still moves the document (to a `host`
  group, for a host verb) and binds the name (`AHostVerbRegisteredBeforeTheFirstFrameIsBound`), and
  so does one into a document with no resident, child or action left, at its next idle frame (a
  page of buttons, say). While something of it runs, a late `host_only` name is a NameError in the
  document's `any` group, and the app's log says once per placement that the document "started its
  programs before a host function or provider was registered: it binds once nothing of the
  document runs, or at its next load (until then a host_only one is a NameError in its isolated
  group)" (`AHostVerbRegisteredWhileItRunsBindsAtTheNextLoad`, whose document has a tick
  resident). In group 0 and in a `host` group every kind is installed as it is, so a late name
  binds in the next env built there.
- **A move** (a placement change). When the answer changes (group 0 against a group, or the group's
  affinity) the document moves, on the host thread (refused while a pump holds its group, above):
  1. its actions, the children they spawned or forked, its residents and its mount ticks are
     killed and reaped where they ran, and a `run_program` caller whose program had not ended is
     told "did not complete: the document moved to another group";
  2. the old group stops being a frame group, and a new group is made and registered if the
     document is eligible;
  3. the streams the document owns (its stream table) are re-pointed at the new scheduler
     (`retarget_document_streams`, under the table's lock, through each stream's
     `stream::retarget(from, to)`, under the stream's own lock, so no post to the old scheduler is
     in flight once it returns), whatever scope each was opened under, and no other owner's
     stream: a group-0 document nested under the moving one keeps its own
     (`AMoveLeavesANestedDocumentsStreamAlone`,
     `AStreamTheDocumentOwnsFollowsItsMoveWhateverItsScope`).
     If re-pointing throws, the document's streams are closed (`close_document_streams`, and if
     that throws too, its resources are dropped, `doc_scope_.clear()`): none is left naming the old
     place;
  4. the old group is retired, not destroyed: unregistered and unpumped, it keeps the contexts
     taken from it, and a later `drain_begin` destroys it once its host has no kernel in flight
     (`host().in_flight()` 0), so a move never waits for the old group's calls
     (`AMoveDoesNotWaitForTheOldGroupsCalls`). Only its own kernels' completions can post to it
     then: its processes are gone, no stream names it, the Runtime posts to the new place, and its
     host-call queue is empty outside a pump. `AppRuntime::idle()` and `wait_idle()` count its
     kernels while they run (`Runtime::retired_calls_in_flight()` and `wait_retired_calls()`, which
     decide and destroy nothing; `IdleAndWaitIdleCountARetiredGroupsCalls`). `~Impl` destroys any
     retired group left, which may wait for its kernels, as a teardown does (4.17);
  5. the residents and mount ticks start again in the new place at the next drain.

  The contexts taken from group 0 stay with the Runtime until it goes, for a process a failed kill
  left there. In group 0 an action and its children survive a reload; a move kills them.
- **AppRuntime's registrations** (`register_verb`, `register_async_verb`, `register_function`,
  `register_async_function`, `register_quit`) are made on the host thread, outside a frame, and
  before `load()` or before the first frame: a `host_only` one (a verb or function) made once the
  document's programs run binds in an isolated `any` document the first time nothing of it runs,
  at the latest at its next load ("Late registrations", above), where in group 0 it binds at once.
  AppRuntime's verb table is read by its providers' bodies, which a spawned child in an `any`
  group runs on a worker, and written by the registrations, `load()` and the destructor, all under
  one process-wide lock; a body copies what it binds under the lock and binds unlocked, and no
  callable a host gave is destroyed under it (pycvc's take the GIL as they go).
- **One isolated document.** A host with one Runtime that opts in may keep calling `drain()`: its
  `drain_end` pumps the document's group through `pump_all` itself. A lone group with nothing else
  in the call is pumped by the caller (no helper is woken for it, 4.4), so one isolated document
  runs on the frame's thread as before, and gains a worker only beside other work in an
  `exec_pump_frame`; several isolated documents spread over the workers. `CVC_EXEC_FORCE_MOVE`
  puts even a lone group on a worker (IS-1b, AR-3b).
- **Teardown** destroys the group after the Runtime's own kills and reaps, then any group a move
  retired (4.17), and, in Stage 3,
  after `pool.kill_owner` when spawn groups hold the document's children.
- **The reactive lane** runs its own evaluator on the render thread under `park_forbidden_scope`
  (`ReactiveEngine`). It never touches a group and is unchanged.
- **Parallel actions** (Stage 3, `ariadne.exec.parallel_actions: k`).
  - A document gets k action groups beside its resident group. Fire-once actions are placed
    least-loaded across them.
  - Actions already get their own env and context (`make_action_context`), so an action placed in
    another group needs no copy.
  - The cost is that actions from one frame may run at the same time, so their state writes within
    a frame are in no fixed order.
  - `doc_scope_` verbs must be `any` or `host_proxied`.
  - This is the first speedup for a single document.

**pycvc.**
- **`Exec`** keeps its private scheduler, which is its group 0.
  - Stage 1a gives it its own cycle registry and random stream, which its scheduler binds whoever
    drives it (4.7, 4.10), sets `ictx.host` to its async host, and enforces the pump-thread rule.
    `Exec.tracked_objects()` reads its registry, and `Exec.collect_cycles()` collects it (with the
    GIL released, as its pump owner); `pycvc.collect_cycles()` collects the calling thread's own
    registry, which an `Exec`'s programs never use.
  - Several `Exec`s on several Python threads run in parallel safely after Stage 0, with or without
    a pool.
  - **The pump owner** (S0-Q5, S1-Q4). The raw schedulers' calls (`Exec.scheduler()`'s, and
    `pycvc.scheduler`'s and `pycvc.async_scheduler`'s `execute`, `kill`, `pause`, `resume`,
    `send_signal`, `fork`, `list_processes`, `get_process_info` and the rest) are the pump
    thread's, and so are `Exec.set_parser_defaults`, `set_data_defaults`, `register_fn`,
    `register_async_fn` and `register_async_coro`, which change the scheduler or the natives its
    programs share, and `Exec.tracked_objects()` and `Exec.collect_cycles()`. Stage 1a enforces it
    through the scheduler's pump owner (4.1):
    - `Exec.run` holds the owner from its re-entrancy guard until it returns, through its quiesce
      (its program's last step and any kill), its wait for its kernels and its reap, and releases
      it last, under the `Exec`'s control mutex. So a raw call or a pump-thread `Exec` call from
      another thread raises during the quiesce as during the run.
    - The Python pump wrappers (`run`, `step`, `sync_run`, `sync_step`, `drain_ingress`,
      `wake_awaiting`, `wait_for_work`) hold it for their call; those that release the GIL hold
      it while the GIL is released.
    - The priority calls' direct path holds the control mutex. With no `run()` active it takes the
      owner too (and raises while another thread pumps the scheduler through a raw call); during a
      `run()`'s quiesce, which still holds the owner and touches the scheduler only under the
      mutex, it acts under the mutex alone; from `run()`'s own thread it acts directly.
    - Every other raw call checks it (a SWIG `%exception`), except `stop`, `is_running` and
      `post_message`, which any thread may call. While another thread holds the owner, a raw call,
      a pump call, `Exec.run` and the `Exec` calls above raise `RuntimeError` "scheduler: another
      thread is pumping it" instead of racing it; from the owner's own thread (a registered
      function) they work. `Exec.kill`, `pause`, `resume` and `send_signal`, posted the way
      `set_priority` is, wait for a request.
  - `run()` already waits with `wait_for_work` instead of sleeping (`perf/sched`).
  - `Exec(app, policy=None, idle_timeout=60.0)` (keyword arguments; `policy` `None` for
    `state_exec.policy`, a `scheduling_policy` member or its name), `Exec.set_priority`,
    `Exec.get_priority` and `Exec.lock_query` exist. Another Python thread may call them while
    `run()` pumps, so while one is active they go through `post_control` (a query waits for its
    answer with the GIL released, and never re-takes the GIL while it holds the `Exec`'s control
    mutex); with none active, or from the thread running `run()` (a registered function or
    callback), they act directly, and from a thread running another `Exec`'s `run()` a call that
    would wait raises `RuntimeError` instead. `lock_query` reads the app's lock manager directly and
    never waits. On every exit path, before it waits for its kernels (`wait_idle`), `run()` stops
    accepting queries (a new flag, not the re-entrancy guard) and answers or fails every pending
    query, so no caller, a kernel included, waits for ever. The re-entrancy guard stays set until
    `run()` returns, so a second `run()` of the same `Exec`, from any thread or a kernel, still
    raises `RuntimeError`. Every `Exec` uses the app's lock manager, so `Exec`s on several threads
    share one lock namespace. `run()` counts a lock wait as pending work for its idle limit (now the
    `Exec` option `idle_timeout`, 60 s by default) only while a process it ultimately waits for is
    on another `Exec` that has made progress within that limit (stepped, or awaited an async call).
    A wait behind an `Exec` nobody pumps or that has made no progress within the limit, or behind a
    holder on the same `Exec` with nothing else pending there, is killed at the limit, naming the
    key. The rule is per `Exec`: a holder parked for ever on another `Exec` that keeps stepping
    keeps the waiter alive, as a busy sibling does today.
- **`pycvc.ExecPool(app, threads=None, size=None, prefix="exec_pool", fresh_env=False,
  idle_timeout=60.0)`** (S1-Q5; built in Stage 1a; `bindings/pycvc/README.md`):
  - The app has one pool (`app::exec_pool()`). The first caller that builds it sets its size:
    `threads`, else the tree's `state_exec.threads`. A later different request keeps the built size
    and logs a warning, and `ExecPool.threads` reports the real count.
  - It holds `size` pooled `Exec`s (by default the pool's real thread count, which a request the
    pool did not apply does not change; at least 1). Each is an `Exec` in group mode: it owns an
    `exec_group` (its
    scheduler, async host, registry and random stream), its natives are
    `extend(frozen_builtins())`, it draws its pids from the app's source, and its context is
    chrooted at `<prefix>.<i>` (4.8: unchrooted `Exec`s all lock from the app root), so its
    programs' state keys, channels and lock keys resolve under it. The pooled `Exec`s are not
    exposed to Python, so no Python thread reaches a pooled scheduler.
  - `run_all(programs) -> list[str]` puts program i on pooled `Exec` i % `size`, each pooled
    `Exec`'s programs one at a time and in order, and pumps the groups together in fork-join rounds
    (one `pump_all` each, `Exec`'s slice per group, no deadline) with the GIL released for the
    whole batch, until every program has ended. With nothing runnable it waits on one parked
    group's `wait_for_work` for at most 1 ms at a time, rotating, so a completion to any group is
    seen within about 1 ms (Stage 2's eventcount replaces this). The results come back in program
    order, rendered as `Exec.run` renders them; a failing program does not stop the others, and
    once all have ended `run_all` raises the first failure in program order.
  - A program parked for `idle_timeout` with nothing pending is killed, as in `Exec.run`. Each
    round first drains the ingress and wakes the frame-waiters of every active group, from the
    caller, and only then tests `has_runnable()`, as `Exec.run` orders them, so a process that
    drain or wake readies counts as pending work: a program that yields frames
    (`(await <value>)`) keeps running under `ExecPool` as it does under `Exec.run`.
  - Top-level definitions persist per pooled `Exec`, as in `Exec`. `fresh_env=True` gives each
    program a fresh scope over the natives (Q12).
  - `register_fn` and `register_async_fn` register on every pooled `Exec`. Python natives retake
    the GIL, so pure-DSL programs scale and Python-calling programs serialize on it.
  - One call at a time: `run_all` or a registration while another is in progress raises
    `RuntimeError`. `close()` kills what a batch on another thread still runs, waits for it with
    the GIL released, and destroys the pooled `Exec`s; from inside its own batch it raises.
  - Without threads (the Emscripten pycvc) or at `threads=0`, every group is pumped on the calling
    thread, in creation order.
- **`ExecPool` or several `Exec`s.** Several `Exec`s, each `run()` on a Python thread of its own,
  already run in parallel (Stage 0): each is its own group 0, with its own pid counter and an
  unchrooted context. `ExecPool` is for a batch of independent programs: one call, the GIL
  released once for the batch, the app pool's workers instead of Python threads, and chrooted,
  reused pooled `Exec`s.
- **The GIL and `state_observer`** (S1-Q7). A `pycvc.state_observer`'s slot takes the GIL on the
  writing thread, which can be a pool worker. A Python host releases the GIL across
  `exec_pump_frame` and `run_all` (`run_all` does, and so does `AriApp.frame`), or the first worker
  write under an observer deadlocks.
- **Stage 3.** An `Exec` program's `spawn-isolated` children run in the app pool's spawn groups.
  `Exec.run` keeps pumping its own group and wakes on their messages.

**exec_coordinator.** Its `sched_` is the sync `scheduler` (`exec_coordinator.h:236`).
- **Stage 0 (built).** Every path by which a bus thread reached the scheduler posts instead
  (`scheduler::post_control`), which fixed a race the coordinator had (S0-Q2, S0-Q3):
  - `handle_control`'s pause, resume, kill, send_signal and set_priority
    ([`docs/STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md) 7.4), fire and forget;
  - a submission from another node (`handle_submit`): a posted `query` entry runs `submit_local`
    and answers the requesting node from there. One that doesn't parse is answered
    `accepted: false` and counted received and rejected (before, the bus swallowed the throw:
    no answer, no count); a local `submit()` of such a script still throws and counts nothing;
  - the migrate ingest (`handle_migrate`), in a posted entry. The coordinator's own
    migrate-request comes back to it on the bus (it subscribes to the whole prefix), so that
    ingest runs on the scheduler's thread too;
  - `migrate()`, now asynchronous: one posted entry pauses the process, serializes it, sends it
    and kills it on the scheduler's thread, and `migrate()` returns `{true, -1, target, ""}`,
    meaning "requested". The no-scheduler and self checks stay synchronous; any other failure
    shows in the stats after a step. A pid that is gone, or neither ready nor running, counts in
    `migrations_failed`; so does a serialize or send that fails, after which the process is
    resumed and runs on here, counted once. `migrations_initiated` counts only a migration sent.
    A kill of the source that throws after the send is tried once more; a process still alive
    then is left paused, never resumed, rather than run on both nodes, and counted in
    `migrations_kill_failed` (a field appended to `coordinator_stats`), for the host to
    `admin_kill` or resume;
  - the local `admin_pause`, `admin_resume` and `admin_kill`, which return true once posted (an
    unknown pid included), as `admin_set_priority` does.

  `policy_` is read under `remote_mu_`, since a bus thread may replace it. `submit()`, `ps_all()`,
  `cluster_stats()`, `emit_heartbeat()` and `emit_status_broadcast()` stay scheduler-thread calls,
  not posted: they are synchronous, and tests call `ps_all()` and `cluster_stats()` with no pump
  thread. Posted entries hold the coordinator through a weak token it resets in its destructor,
  never a bare `this`, so an entry still queued when the coordinator goes (applied later, or dropped
  by `~scheduler`) does nothing. The token does not wait for an entry already running on another
  thread, so a host destroys the coordinator on its scheduler's thread, or while nothing pumps the
  scheduler. Until Stage M, `migrate`'s kill of the source releases its locks like any kill; Stage M
  adds the refusal of lock holders (4.20).
- **Stage 2.**
  - `attach_pool(exec_pool &)` generalizes `attach_scheduler` (`exec_coordinator.cpp:223`).
  - `submit_local` (`exec_coordinator.cpp:586-615`) places a job in its own job group, or in the
    least-loaded of k job groups. This is the in-process analogue of its leader forwarding
    (`539-569`).
  - The sync-scheduler path stays for the CLI (Q13).
- **Cross-node `migrate`.** Today it sends metadata and the target starts `nil`
  (`exec_coordinator.cpp:809-866`). It can send `encode_process` of a process paused under its
  group's token, and the target restores it and calls `rebind_natives` (`state_value_codec.h:72-82`).
  The codec omits the inbox, watches and pending signals (`66-71`); Stage M (4.20) closes those
  gaps and builds the cross-node move. The in-process move of Stage 3 does not use the codec.

### 4.13 Configuration and opting in

From highest to lowest precedence, following the settings rules of
[`docs/STATE_EXEC_EXCEPTIONS.md`](STATE_EXEC_EXCEPTIONS.md) 3.8:
1. the `pool_options` a host passes (`app::exec_pool(opts)`);
2. the app tree's `state_exec.threads` (a whole number or `auto`; `CVC_EXEC_THREADS` overrides
   it), `state_exec.cpu_set` (`performance` or `all`) and `state_exec.deterministic` (`true`,
   `1`, `on` or `yes`; `false`, `0`, `off` or `no`), read through `detail::setting_value`;
3. the built-in defaults: threads 0, P-cores, not deterministic.

The pool reads them once, when it is built (`app::exec_pool()`, at its first call). An unknown
value is logged and the default used. Options given once the pool is built are not applied: the
built pool is kept, and a different request is logged. `threads` cannot change while groups are
attached (Stage 2).

**Lock and priority settings** ([`docs/STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md)) follow the same
precedence: `state_exec.policy` (`round_robin` by default), `state_exec.locks.threading` (`auto`,
`single`, `shared`), `state_exec.groups.priority_weights` (on), `state_exec.worker_class`
(`normal`, `pool_options::worker_class`), `state_exec.process_class`, and
`ariadne.exec.lock_timeout` (1.0 s). `auto` makes the app's lock manager `shared` when the build
has threads and `state_exec.threads` is above 0 (`CVC_EXEC_THREADS` counts here too), or pycvc is
running the app (`detail::note_threaded_host(app)`, for each app pycvc makes or runs an `Exec`
on), or this app's `exec_pool(opts)` asked for threads above 0 before anything had made the
manager (S1-Q1). Both are app-local hints. Their names await approval as Q14's did
([`docs/STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md) 14.2).

**One manager, one mode** (S1-Q1). Every group's scheduler attaches to `app::lock_manager()` in
the manager's own mode (`lock_attach{shared}`), as the app scheduler does: `with-lock` in a group
excludes the same keys app-wide, and at threads 0 the groups share a single-mode manager on one
thread. The manager's mode is fixed when it is made, and building a group makes it (the group's
attach calls `lock_manager()`). So workers pump only when the app's manager is shared: when it is
single, `app::exec_pool` clamps the pool to 0 threads and logs why, and every group is pumped on
the caller, as at threads 0. A host that wants workers gives its pool options before its first
call of `exec_scheduler()`, `lock_manager()` or an `exec_group` constructor, or sets
`state_exec.threads` in the tree. A group built while the manager is single, before any pool,
logs that once per app. And every pool keeps a group on a single manager on its caller (4.12),
whichever pool pumps it.

| Host | How it opts in | Stage |
|---|---|---|
| C++ embedder | make `exec_group`s; `app.exec_pool(opts).pump_all(...)`, or `register_frame_group` and `exec_pump_frame`; `attach` in Stage 2 | 1a, 2 |
| Ariadne, several documents | `state_exec.threads: auto` (or `app.exec_pool(opts)` first), `ariadne.exec.isolate_documents: true`, and each frame `drain_begin` / `app.exec_pump_frame()` / `drain_end` (4.12) | 1b |
| Ariadne, one document | `ariadne.exec.parallel_actions: k`; programs use `spawn-isolated` | 3 |
| pycvc | `pycvc.ExecPool(app, threads=8).run_all(...)`; several `Exec`s on several threads | 1a (0) |
| `exec_coordinator` | `attach_pool` | 2 |
| CLI | none (sync scheduler) | - |

**A program opts in** with `(spawn-isolated script [name] [priority] [args])` (Stage 3), which
returns the child's pid. Plain `spawn` stays in the caller's group, and is never placed on a spawn
group automatically (D's `auto_spawn` is not taken; Q4).

**Debug knobs** (environment variables):
- `CVC_EXEC_THREADS`: overrides the tree's `state_exec.threads`, for the pool's size and for the
  lock mode's `auto` alike, so a whole suite runs threaded unchanged (AR-1). Read once per
  process. It does not override a host's `pool_options` (`ExecPool(threads=0)` stays 0).
- `CVC_EXEC_DETERMINISTIC=1`: deterministic mode (4.4). Read when a pool is built.
- `CVC_EXEC_FORCE_MOVE=1`: every group is pumped by a different thread on each pump (4.4), a lone
  isolated Ariadne document's included; in Stage 3, every movable process moves every K pumps.
  Read when a pool is built, so a test sets it for the pool it builds.
- `CVC_EXEC_VERIFY=1`: in Stage 1, the pump owner's check in the schedulers' public mutators in a
  release build (4.1; debug builds always check). Read as each scheduler is made. Boundary
  tagging and isolation walks (4.3) are not built yet.
- `CVC_EXEC_FUZZ=<seed>`: yields and short pauses (0-50 us) at the protocol points, read as each
  lock manager is made (the seed is the manager's). The locks track's L3 built it for the lock manager's operations and the lock
  ports' posts, drains and close (`lock_fuzz_point`, LT-4); Stage 2 adds the state word, ingress
  push, eventcount and router posts.

### 4.14 When only one thread is available

- Every worker count is clamped (`exec_pool`'s constructor), each clamp noted once for the pool's
  owner to log (`take_notes`; `app::exec_pool` logs them):
  - to `std::thread::hardware_concurrency() - 1`;
  - to the size of the process's affinity mask, less the caller, where the OS reports one;
  - to 0 in a build without threads (`threads_compiled_in` false: Emscripten without pthreads);
  - to 0 by `app::exec_pool` when the app's lock manager is single (4.13).

  `threads()` and `stats().effective_threads` report the workers running, which the tests record.
- With 0 threads every `pump_all` pumps its groups inline on the caller, in creation order (with
  the groups skipped by the last deadline first, before `host_first`), host-affinity ones included.
  Results are those of deterministic mode. Continuous mode (Stage 2) pumps attached groups only
  from `pump_host` and `wait_for_work`.
- The cost over today is the pump scope (the scheduler's two thread-local stores and restores per
  entry, and `current()`'s per pump) and, with isolated documents, one pump per group instead of
  one shared one.
- Deterministic tests and the CLI keep 0 threads.
- The lock manager runs in `single` mode where no two threads use it at once (a scheduler's
  private manager, builds without threads, an app at 0 threads without pycvc), with no mutex or
  atomic on its path. Groups pumped inline still exchange lock wakes through their lock ports,
  whose own mutexes are then uncontended, and deadlock detection is the same exact check at
  block time ([`docs/STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md) 6.1, 9.5).
- Emscripten builds without threads compile no worker: `threads > 0` is clamped to 0 with a log
  line, and `exec_pool` pumps inline (`state_exec_wasm_smoke`; pycvc's wasm smoke runs
  `ExecPool(app, threads=0)`).

### 4.15 Spawned and isolated processes, and moving a process (Stage 1 and Stage 3)

**The spawn context** (B27, Stage 1). `intrinsics_context` gains
`std::function<spawn_setup(const intrinsics_context &parent)> spawn_context`, set by the host.
The single-scheduler part lands first, in the locks track's phase L1b
([`docs/STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md) 12): the hook, the default and both callbacks,
with `sched` and `host` the parent's, and section 5's spawn compatibility scan in its PR. It needs
no groups. Stage 1 then only extends it to groups and adds what follows from them.
- `spawn_setup` returns the child's env and an owned context object, held by the child process.
- **The default** (`make_spawn_context`, for hosts that call plain `register_intrinsics`) gives the
  child a new heap-owned `intrinsics_context`:
  - it copies the parent's `sched`, `host`, `root` (with its pin), `root_path`, `uid` and
    `channels`;
  - it builds `extend(base)` with the intrinsics registered against that context;
  - the child gets the parent's `owner`;
  - the child's intrinsics include `with-lock` and `set-priority`, so its lock keys resolve under
    the parent's chroot. The default also copies the parent context's `lock_default_timeout`
    and `lock_wait_allowed`, so a child an Ariadne init script spawns, stepped on the host
    thread, never waits for a lock either. The child records the parent's pid and inherits its
    priority floor ([`docs/STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md) 5.5, 5.8, 7.2).
- **Ariadne's callback** (`Runtime::Impl::spawn_child`; as built in Stage 1b, S1.10) builds the
  child's context from its parent's, by value and nothing by path: its `sched` and `host` (so the
  child runs in its parent's group, and its async calls go to that group's host), its `root`,
  `root_pins` and `root_path`, `uid`, cluster and node ids, `channels` (the document's policy),
  `document` (its resource scope), both lock fields and `spawn_context` (grandchildren get one
  too). It walks no tree, so it is safe on the pumping thread, which is a worker in an `any`
  group. It then builds the child's env as an action's (`finish_context`: the intrinsics, the
  providers by kind for the parent's group, `has?`), and records the child so the drain reaps it
  once it ends; the child keeps the document's owner, so the teardown's `kill_owner` reaches it.
  So a child of a deleted chroot keeps writing its parent's detached node, as the library's
  default does, and makes no node at the old path (SP-1). Before, it re-chrooted by path, through
  `make_action_context`, which made a new node there.
- **pycvc's callback** gives the child `extend(natives)`.
  - `natives` is a new scope holding the `Exec`'s registered natives.
  - `impl_->env` becomes `extend(natives)`, so the REPL's top-level definitions still persist
    across runs.
- **With no callback** a child keeps today's bare env.
- **The child's result.** A spawned child reports by message. The pool also records each
  `spawn-isolated` child's exit status for `inspect`.

**`(fork)`** (B28, landed with the locks track's L1, #862) forks the running process,
`current_pid()`, not `ctx->pid`, which hosts leave at -1 (Ariadne's `make_action_context`) or 0
(pycvc, `pycvc_exec.cpp:416`). The child copies `owner` (`async_scheduler.cpp:957-976` copies
everything else), so `kill_owner` reaches it.

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

  Isolated spawns usually qualify; a process that does not is simply not moved. A lock holder is
  eligible: its holds name its pid and travel with it (4.19); a lock waiter is parked, so it is not.
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
  5. Every moved process arrives with no inherited priority: `adopt` resets it, so a boost is never
     carried to a round-robin group, nor kept stale. For a lock holder, `adopt`, on the target's
     thread, also renames its record in the app's lock manager from the source group's lock home to
     the target's, under the manager's mutex (the manager's `move`, which the target calls), and the
     target re-reads its inherited priority at once when its group uses a priority policy. A moved
     process has no waiting request, so a wake or relevel posted to the source for its pid finds
     nothing to do. The lock protocol does not cover moves yet; the locks track's L4 adds this
     operation (`move`, [`docs/STATE_EXEC_LOCKS_PROTOCOL.md`](STATE_EXEC_LOCKS_PROTOCOL.md) 9), with
     its proof and model check, before Stage 3's moves of holders merge.
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
- **Locks.** The app's lock manager is already safe for several threads. A group's lock home
  (key cache and mirrors) becomes one per domain, and a wake to a waiter in another domain is a
  request on that domain's inbox. If the manager's mutex contends, it can be sharded by subtree;
  if handoff convoys appear, Go's 1 ms starvation rule can be added to the grant rule. The DSL
  does not change.
- **Its tests** reuse the GR oracle (8.2), the stress cases (8.8) and the sanitizer jobs (8.7),
  plus an exhaustive small-state model of the domain word and inbox.
- **Cost:** 8-12 engineer-weeks at high risk.

### 4.17 Failure modes

| Failure | Handling |
|---|---|
| A program error in a step | As today: the process ends `failed` or `killed` (`async_scheduler.cpp:228-268`) |
| A C++ exception escaping a pump | `exec_group::pump` catches it, marks the group failed (`failed()`) and returns `pump_result::failed`; the pool then kills the group's processes with "internal error" (`kill(pid, reason)`), on the pumping thread with the token taken again, and lets the other groups finish. At the join the caller logs the failure through the group's app, and `failed_groups()` returns the groups the calling thread's last `pump_all` on that pool marked failed (raw pointers, to compare with; the thread's next `pump_all` replaces them). No exception escapes a worker thread (GR-12). An `on_complete` callback run inline that throws is the host's exception, as today: it leaves the pump and is rethrown from `pump_all` after the join, as `host_first`'s is |
| A native that loops inside one step | Bounded by its deadline guard (`max_step_time`, the pump's time budget). It holds one worker; other groups keep running, which is better than today. A fork-join join waits for it, bounded by the frame deadline plus that step |
| A worker blocked in an inline kernel or `call->wait` | Holds its group only; long-step monitor; optional compensating worker (4.9) |
| A host-only native called in a worker-pumped group | Impossible by construction: not installed, so a NameError. A `host_proxied` body runs on the `pump_all`'s caller by construction (`run_on_host`) |
| A host posts one compound value to two groups (invariant G broken) | A data race in a release build; TSan in CI. The host API documents a posted value as moved in (4.12). Stage 1 has no `CVC_EXEC_VERIFY` tagging, and `exec_group::execute(value_t)` copies the AST's lists and dicts itself |
| A subscriber assumed the main thread but its document was declared `any` | A data race in the host's code. Mitigated by the audit, the host's thread-affine declarations, `host_only` as the default, and the debug home-thread asserts of cvcGL's render-thread subscribers (4.8) |
| Cross-group ingress full | `msg-send` returns `{status: "error", reason: "ingress full"}`; `post_message` returns false; counted per group |
| A remote waiter vanishes with a delivery in flight | Requeued at the home (4.6); counted |
| A lock holder killed or failed, alone or with its group | Its locks are released when the kill is applied, before `release_if_idle`; an X holder whose section had started is marked abandoned for the next exclusive holder ([`docs/STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md) 5.6) |
| A lock waiter killed while its grant's wake is in flight | The grant is already a hold of the waiter in the lock manager, so the kill releases it and the next waiter is granted; the late wake is ignored. A waiter whose timeout fires after the grant takes the grant |
| A lock deadlock across groups | `DeadlockError` in the request that closes the cycle, when it blocks: the check runs under the lock manager's mutex over every group's holds and waiters, so it is exact |
| A group destroyed while pumping | Destruction takes the token, waiting for the pump; kernels in flight are waited for. Each teardown step catches and logs what it throws, so a kill or a lock release that throws (a Stage 0 follow-up) never terminates the process (4.2). Destroying a group inside a `pump_all` that holds it, or from a body or callback the pool runs, is a debug assertion; from inside its own pump (a native or inline callback of its own process) it aborts in every build |
| A worker that fails to start | The pool runs with the workers that started, and notes each failure for its owner to log (`take_notes`; `app::exec_pool` logs them); the caller always pumps |
| Under `CVC_EXEC_FORCE_MOVE`, no worker reaches a call's groups before its deadline | The caller, which may not take a group it pumped last, waits for the workers no longer than the deadline, then skips those groups itself, and a worker that skips them tells it (4.4): the frame ends on time, and the skipped groups go first next frame |
| Teardown of an Ariadne Runtime (`~Runtime`; built in Stage 1b, S1.10, S1-Q8) | On the host thread, outside a pump: while a `pump_all` of the app's pool holds the document's group it is refused in every build, from any thread, printed and logged and then aborted before anything is touched (`refuse_if_held`, 4.12); a group-0 document's, or one whose group no pump holds, goes ahead. In this order, each step catching and logging what it throws: (1) the document's resources close (`doc_scope_.clear()`: a stream marks its descriptor closed and posts its last seq and "closed" to the scheduler it names); (2) on the scheduler its processes run on (its group's, else group 0's if it ever used it), `kill_owner` of the Runtime's owner tag, then a reap of every process the Runtime knows of, its actions' own processes included: the actions and the children they forked, the spawned children, the residents and the mount ticks (S1-Q8: a killed record left in group 0 was a leak whose context was already gone; HZ-8); (3) its frame registration, then its group, whose own teardown (4.2) kills, shuts its host down and reaps whatever is left, and then any group a move retired, each waiting for its kernels (a cancelled http request ends its transfer within about a second, 4.9). The group goes first of the Runtime's members, before the channel policy and the document scope its processes' contexts point at. Why this order: an open stream holds the group's scheduler and posts to it, from its producer thread too, so the streams close while the group lives; a started net call holds the group's host's finisher, and the host's shutdown, after the kills, cancels it before the scheduler goes (4.9). A resident a reload or a mount reconcile stops is killed and reaped then (`stop_resident`), so nothing of the Runtime's owner is left in group 0 (`AriadneHazards.AResidentAReloadStopsIsReaped`) |
| A document moved (its placement changed) while its programs run | Its processes are killed and reaped where they ran, a `run_program` caller whose program had not ended is told "did not complete: the document moved to another group", its residents start again in the new place, the streams it owns are re-pointed there (closed if that fails), and the old group is retired: unpumped at once, destroyed at a later `drain_begin` once its kernels have ended, so the move does not wait for them (4.12) |
| Teardown of the app | The groups' owners destroy them first; the pool owns none, and `~app` asserts in debug builds that none is left. Then `~app` destroys the pool, whose workers stop after their current pumps, before the app's async host shuts down and its background pool, which ran the groups' kernels, closes |

### 4.18 Every blocker, resolved

| ID | Resolution | Section, stage |
|---|---|---|
| B1 | A group is stepped by one thread at a time; Stage 4: thread-local step context | 4.1; 4.16 |
| B2 | Owned by the pumping thread; hosts touch them between pumps or under the token; cross-thread input only through posts | 4.1, 4.5; Stage 1-2 |
| B3 | Same; across groups, `post_control` applied between the target's steps; Stage 4: requests and owner-only release | 4.5; 4.16 |
| B4 | Same group: one thread. Across groups: subscription at the home with seq and re-send; Stage 4: atomic pop-or-register | 4.6; 4.16 |
| B5 | Registry per group, bound by its scheduler on every entry; collected and freed with its owner; threads opt in to a collection at exit; re-tracked on a move | 4.7; Stage 1a |
| B6 | Unchanged: under invariant G the walk reaches only owned and frozen nodes | 4.7 |
| B7 | The built-in class dicts refuse mutation (`TypeError`) | 4.3, 5; Stage 0 |
| B8 | Fork families stay in one group; `(fork)` fixed (B28); Stage 4: one domain | 4.15; 4.16 |
| B9 | Inside a group, unchanged; across groups a fresh envelope per receiving group; Stage 4: per receiver | 4.6; 4.16 |
| B10 | `state-data-set` stores an isolating copy; closures, natives, generators, futures, lock forms, lock waits and user-class instances refused at any depth | 4.3; Stage 0 |
| B11 | Processes sharing an env or context stay in one group; a move requires the isolation walk | 4.3, 4.15 |
| B12 | `exec_group::execute(value_t)` copies the AST's lists and dicts; generators cannot cross (isolating copy) | 4.3, 4.12; Stage 1a |
| B13 | Per-watch root | 4.8; Stage 0 |
| B14 | Pinned lookups and a pinned chroot; `state-delete` erases one node | 4.8; Stage 0 |
| B15 | `affinity::host` when the host declared a thread-affine prefix at, above or under the document (`AriRuntime` declares its scene), or its env holds a `host_only` native; the audit, with debug home-thread asserts in cvcGL's render-thread subscribers | 4.8; Stage 1b |
| B16 | One frozen builtins scope (`frozen_builtins()`), extended per group | 4.2; Stage 1a |
| B17 | Atomic stop flags; `post_control` on both schedulers; `exec_coordinator` uses it | 4.5, 4.12; Stage 0 |
| B18 | Futures cannot leave their group (isolating copy, messages are strings); `finish_async_call` idempotent, every waiter woken; the foreign-future check raises on another scheduler's threaded future | 4.9; Stage 0 (the check: Stage 1a) |
| B19 | `perf/sched`'s `wait_for_work`; Stage 2's group state word and pool eventcount | 4.4; Stage 2 |
| B20 | Only the reactive lane arms it, on the host thread; Stage 4: in the step context | 4.12; 4.16 |
| B21 | Blocks one worker only; long-step monitor; optional compensating worker | 4.9; Stage 2 |
| B22 | Provider kinds `any`, `host_proxied`, `host_only` (`exec_group::install`); host-call queue; `affinity::host` groups; Ariadne's providers registered by kind and installed by it (`register_action_intrinsics(app, provider, affinity)`) | 4.9; Stage 1a (Ariadne's providers: Stage 1b) |
| B23 | `perf/async` | 2.4 |
| B24 | `perf/async` | 2.4 |
| B25 | Per-process parsing; a shared AST copied at submit; class dicts frozen and measured (`mt.groups` raise/catch); per-group copies if contention shows (Q2) | 4.3, 6; Stage 1 |
| B26 | Random stream per group, bound by its scheduler; line-atomic `print` | 4.10, 4.11; Stage 1a (`print`: Stage 0) |
| B27 | Spawn context with a default; Ariadne's and pycvc's callbacks; Ariadne's builds the child from its parent's context | 4.15; Stage 1 (Ariadne's in groups: Stage 1b; its single-scheduler part landed with the locks track's L1b, which passes IS-1 and IS-2 on one scheduler; Stage 1 re-runs them in groups) |
| B28 | `(fork)` forks `current_pid()`; the child keeps `owner` | 4.15; landed with the locks track's L1 |
| B29 | `drain_begin` / `exec_pump_frame` / `drain_end` | 4.12; Stage 1b |
| B30 | `ictx.host` in every Ariadne context and at every async-function site; posts to `ictx.host->parking_scheduler()`, else group 0 | 4.9; Stage 1a (the field), Stage 1b (Ariadne) |
| B31 | Movable processes resolve their scheduler, host and tracker through `exec_group::current()` | 4.15; Stage 3 |
| B32 | One pid source per app, for group 0 and every group; `adopt` keeps the pid | 4.5, 4.15; Stage 1a, 3 |

### 4.19 Locks and process priority across groups

[`docs/STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md) designs DSL locks and priority changes. This
section says how they meet the groups.

- **One manager.** Every group's scheduler is a *lock home* (key cache, the mirrors of its
  processes' holds, a port) over the one `app::lock_manager()` (in shared mode when several threads
  use it; `single` at 0 threads without pycvc, 4.14). All lock state is under the manager's one
  mutex: holds on absolute tree paths, each owned by (home, pid), the waiting requests in arrival
  order, and a record per process. A request is granted all at once or waits holding nothing new.
- **No domains.** Groups on disjoint prefixes share the mutex but no hold; a lock on a common
  ancestor (an unchrooted `docs`) excludes every document below it. If the mutex contends at high
  thread counts, the locks design's fallback is sharding the manager by subtree (its 3.3), measured
  by `lock.cross_thread` and `mt.groups.locks`.
- **Stage 1 (fork-join).** Uncontended locks are immediate from any group. A grant to a waiter in
  another group is decided in the manager and announced by a wake post, applied at that group's
  next pump: the next frame for Ariadne. The spawn context gives children the lock intrinsics, as
  it already does on one scheduler from the locks track's L1b. At 0 threads the groups share a
  `single`-mode manager on one thread and still exchange wakes by post. As built in Stage 1a
  (S1-Q1): each group's scheduler attaches to the app's manager in the manager's own mode, and
  workers pump only when that manager is shared (4.13); a group's teardown closes its lock port
  before its kills (4.2). GL-1 and GL-2 test both; the LG cases stay the locks track's L4.
- **Stage 2 (continuous).** Lock posts are followed by `wake(g)`, so a cross-group grant costs a
  post plus a wake when the group is parked (5-20 us). A post calls `wake(g)` inside its port's
  mutex while the port is open, and a group's teardown closes its lock port right after it takes the
  group's token, before it kills its processes (4.2), so no post reaches a dying group (4.6).
  `pool.kill_owner`'s acknowledgement covers locks. Cross-group `set-priority` goes through
  `post_control` (Q5). The ready queue weights groups by `best_runnable` (4.4).
- **Stage 3 (moves).** Holders may move; waiters never do (4.15). The lock manager's `move`
  operation, which renames a holder's record, is specified and model-checked before it lands.
- **Deterministic mode.** Conflicting requests are granted in arrival order; cross-group wakes are
  posts applied in post order; the deadlock victim is the requester that closes the cycle, found
  when it blocks.
- **Slices** (Stage S, 4.4). A contended acquire parks, which ends the run; an uncontended one does
  not, so a holder may run a whole run. A lock post ends the run, as any post does, and so does a
  release that readies a waiter at a better level than the runner's, or that drops the runner's
  inherited boost, so the best level is run within one step.
- **Priority.** Strict priority holds inside a group. A lock holder in a group under a priority
  policy inherits the best base priority of the processes it blocks, directly or through up to 8
  edges (a round-robin waiter lends nothing but passes on what it inherits), computed in the manager
  over paths, not from the direct waiters' effective priorities; across groups a `relevel` post
  makes the holder's group re-read it, so a late post changes nothing. Inheritance is not user
  control; this design reads Q5 as not gating it (9.2).
- **OS priority.** Workers take `worker_class` or their group's `thread_class` (4.1). It is
  host-only.
- **Tests:** LG-1 to LG-7 (8.11).

### 4.20 Moving programs between hosts (Stage M)

A DSL program can run on a server or a client, and may move between them. This is scheduled last,
after the single-machine work. It needs only Stage 0 (`post_control`) and the locks track's L1-L3,
hardened: nothing here uses groups, and a cross-host restore is not an in-process move.

- **The move.** `exec_coordinator::migrate` pauses the process through `post_control` (under its
  group's token once Stage 2 runs the coordinator's jobs in groups), ships `encode_process`, and
  the target restores it. Today it ships metadata and the target starts `nil` (4.12).
- **The codec gaps close here.** The inbox, watches and pending signals, which `encode_process`
  omits (`state_value_codec.h:66-71`), travel with the process.
- **Eligibility:** ready, no async call in flight, no fork family (a subset of Stage 3's rules:
  watches travel, and a codec copy needs no isolation walk). A lock waiter is never moved, since
  `pause` fails on a waiting process.
- **Pids are not portable across hosts.** The target assigns a new pid. The source answers control
  messages for the old pid with "moved to <node>, pid <new>" until it forgets the process.
- **Never on two hosts.** The source kills its copy only after the target acknowledges the restore;
  until then the target keeps the process paused. A failure in between leaves exactly one copy (the
  source's, resumed) and is reported.
- **Locks** follow [`docs/STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md) section 10 (its phase L7, built
  with this stage). Until now `migrate` killed the source, which released its locks like any kill.
  From here a holder is refused unless the caller asks to release its locks; the refusal is checked
  inside the paused control entry, on the pumping thread. The release is applied by the same entry,
  marks the locks abandoned, and marks the source process `locks_lost`: if a failure resumes the
  source, `LockError` ("lost") is raised into it before its next step, so it never continues its
  section without the lock. Cross-host locks are leases with fencing tokens, renewed in the
  coordinator heartbeat and rebound to the new host and pid before the process resumes.
- **Priority.** Base priority and floor travel; boosts never do. The target applies its own floor.
- **Deterministic mode** is per host. A move is applied at a step boundary on both sides.
- **Tests:** MG-1 to MG-6 (8.12).

## 5. Semantics changes and compatibility

Each item says when it applies. "Always" items land whether or not a host enables threads.

1. **Always (Stage 0, built): built-in exception classes are read-only.**
   - All 19 built-in class dicts: the 13 this design started from, the locks track's
     `LockError`, `DeadlockError` and `PermissionError`, `AssertionError`, and the wasm guests'
     `WasmError` and `RecursionError`. Each is one per OS
     process, shared by every environment (`exceptions.cpp`, `classes()`).
   - `set-attr` and `del-attr` on one raise `TypeError` "set-attr: built-in class is read-only:
     ValueError" ("del-attr: ..." likewise), and the C++ `set_field` raises `TypeError` "built-in
     class is read-only: ValueError". That covers `Exception.__cls_dict__` and the others, a
     caught exception's `__class__` and its `__super__` when built in, and a class's `init` called
     on the class dict itself. A shape test (2-3 entries, `__name__` first) keeps the lookup off
     the hot paths (`builtin_class.h`).
   - User classes, including ones derived from the built-ins, are unaffected: each has its own
     dict.
   - `(raise ValueError.__cls_dict__)` is still the `TypeError` "raise: expected an exception, got
     dict": `raise` refuses a non-exception before anything annotates it. The evaluators'
     `annotate` (and `async_call::outcome()`) leave a built-in class dict as it is for another
     case: a native function that throws a `dsl_exception` holding one, which is then raised
     unchanged rather than turned into the read-only `TypeError`.
   - Before, one program could redefine `init` for every program in the OS process.
2. **Always (Stage 0, built): `state-data-set` stores an isolating copy** (4.3).
   - A list changed after it was stored no longer changes what `state-data-get` returns.
   - Nil, booleans, numbers, strings and symbols are stored as they are, and `bytes` as raw
     octets; everything else is copied (`data_object`s included, so the internal ones are
     refused). An ordinary `data_object` is shared.
   - Refused at any depth, with `runtime_error` (the DSL's `RuntimeError`) "state-data-set: cannot
     store a `kind`" ("an" before a vowel), plus " (nested in a list)" or " (nested in a dict)":
     a closure, native function, generator, future, lock form or lock wait, and an object
     instance, an instance of a user class (S0-Q4: its methods hold the writer's live
     environment). An instance is known by its shape: a dict whose first key is `__class__`,
     holding a dict whose first key is `__name__` and which is not a built-in class. Before, only
     a top-level callable was refused. A refused value leaves the node as it was.
   - Any other dict is data, `__class__` key or not. Plain data of exactly an instance's shape is
     refused, though: `(json-parse "{\"__class__\": {\"__name__\": \"P\"}}")` cannot be
     stored.
   - A built-in exception's instance is copied, its class dict shared (item 1), so an exception
     read back with `state-data-get` can be stored again. So can one restored from a snapshot and
     one that passed through Python (a Python function's return value, alone or in a list, and
     `pycvc.make_exception`'s, by class name or class dict): the codec and pycvc's converter
     relink a copy of exactly a built-in class's shape to the class itself, by one rule
     (`builtin_class_of_copy`, `exceptions.h`).
     `state-data-get`'s share-mode copy, and the copies of async arguments, share the built-in
     class dicts too: a read-back exception's `__class__` is the shared read-only class, so
     `set-attr` on it raises `TypeError` (it changed a private copy before), and the copy costs
     fewer data nodes.
   - A built-in exception that holds an instance is refused too: one whose `cause`, set
     automatically, is a user-class exception gives "state-data-set: cannot store an object
     instance (nested in a dict)". A program stores the fields it needs instead.
   - The copy is deep-data work charged to the writer: `data_nodes` grows by one per element, with
     a deadline check per element. With `data.step_nodes` 0 (the bare schedulers' default) it adds
     no step. On the hosts (`data.step_nodes` 1024: the app's scheduler, pycvc's `Exec`, Ariadne)
     the first slice runs in the call's step within what the step has left, and the rest takes
     about one data step per 1024 nodes visited, a container's end counting as one: none for a
     small value, but a deeply nested one (600 nested one-element lists) can take one. With
     `data.step_nodes` above 0, freeing the node's previous value goes through the release backlog
     and is charged to the writer too; a bare scheduler frees it unmetered.
3. **Always (Stage 0, built): `state-delete` deletes only its node** (4.8). It no longer sweeps
   every other expired node under the context's root as a side effect: an expired node stays until
   a sweep. It unlinks the node from its parent (`childChanged` once, `expiring` never), and an
   empty path does nothing. pycvc's `state_remove` does the same: before Stage 0 any
   `state_remove`, and `state_remove(app, "")`, swept the whole tree's expired nodes.
   `state-set-tree` removes its stale children the same way, and charges a unit for each, not
   what a sweep removed.
4. **Always (Stage 0, built): `state-watch` from a chrooted process no longer moves the
   scheduler's watch root** (B13, 4.8). It did more than change where the scheduler reads its
   settings: `set_watch_root(ctx->root)` from a step moved the shared scheduler's whole watch root
   to the chroot, so the submit limits, the async charge rate (where a 0 removed the
   per-millisecond step charge) and the published settings (`effective.async_steps_per_cpu_ms`,
   and `load_settings`'s values) all moved into the document, and every other process's watches
   resolved under the new root. Now each watch resolves under its own context's root, gated per
   root, and the scheduler's root stays the host's. Kept: an unchrooted context on a scheduler
   with no watch root still sets it (`ProducerConsumerTest` relies on it), and the scheduler
   holds that root while it is its watch root (`set_watch_root(root, pin)`). A host's own
   `scheduler_base` subclass must override the 5-argument `register_watch_handler` (or a
   chrooted watch resolves under its watch root), `watch_root()` (or every unchrooted
   `state-watch` sets its root again) and the 2-argument `set_watch_root` (or a root a step gave
   it is not kept alive).
5. **Always (landed with the locks track's L1, #862): `(fork)` forks the running process.** Under
   Ariadne and pycvc it used to fork pid -1 or 0, which fails. The child keeps `owner`, so
   `kill_owner` reaps it. Stage 0 added HZ-8's host tests (8.1).
6. **Always (the locks track's L1b, or Stage 1 if it lands first): spawned children get an
   intrinsics context** and the parent's chroot, owner and channel policy, where the host
   installed a spawn context: Ariadne, pycvc, and hosts that use `make_spawn_context`. Today they
   get the bare builtins. IS-1 and IS-2 (8.5) verify it in whichever of the two merges first, on
   one scheduler; Stage 1 re-runs them in groups.
7. **Always (Stage 1a, built): `math.random` draws from a stream per scheduler** where the
   scheduler has one: each `exec_group`, the app scheduler and each pycvc `Exec` (4.10). Bare
   schedulers and evaluators still draw from the thread's engine. This is visible only to programs
   in several such schedulers on one thread, whose sequences no longer interleave.
8. **Always (Stage 1a, built): cycle-collector statistics** (`tracked_objects()`, `cycle_stats`)
   and collections are per scheduler, not per thread, for the schedulers with a registry: each
   `exec_group`, the app scheduler and each pycvc `Exec` (4.7). Read through the thread's own
   registry (`tracked_objects()` with no argument, `pycvc.collect_cycles()`), they no longer see
   those schedulers' objects: `tracked_objects(*sched.registry())`, `Exec.tracked_objects()` and
   `Exec.collect_cycles()` do. An idle group keeps its garbage longer than a thread did, until its
   own run collects it (4.7). This is visible only to tests and diagnostics.
9. **Always (Stage 1a, built): `on_complete` callbacks** given through `exec_group::execute` run
   on the `pump_all`'s caller after the join in a threaded `pump_all`, whichever thread pumped
   their group, and a throw among them is rethrown once all have run (4.9). At threads 0 and in
   deterministic mode they run inline, as today; so do the app scheduler's and pycvc's `Exec`'s.
10. **Opt-in (several Runtimes and `exec_pump_frame`; built in Stage 1b): the shared scheduler is
    pumped once per frame.** Its `wake_awaiting` half was already true before Stage 1: with
    plain `drain()`, `starts_frame()` and the app's pump clock wake the awaiting processes once per
    app frame however many Runtimes drain, so an `await` already crossed exactly one frame
    (`SeveralRuntimesResumeAnAwaitOncePerFrame`), and that path is unchanged. What
    `drain_begin` / `exec_pump_frame` / `drain_end` changes is the step budget: group 0 gets one
    `sync_run` a frame, 200000 steps, instead of one per Runtime, N x 200000 (AR-2).
11. **Opt-in (`isolate_documents`; built in Stage 1b): an isolated document runs in its own
    scheduler** (4.12).
    - Its pump budget is 200000 steps and 0.1 s per frame, of its own.
    - Its `ps`, `kill` and `inspect` see only its own group's processes, not other documents'
      (AR-3).
    - Its natives and subscribers may run on a worker thread, if its group is `any`.
    - In an `any` group a worker pumps, an action that calls a `host_proxied` verb completes a
      frame later (4.11 item 7), and `has?` answers from the frame's snapshot (4.9).
    - A change of its placement (a reload that makes it eligible or not, a program, the channel
      policy, the key bindings or a declaration) kills and reaps what it runs, tells a
      `run_program` caller, and restarts its residents in the new place; in group 0 an action
      survives a reload.
    - A host verb, function or provider registered once its programs run moves it only when
      nothing of it runs, so the move kills nothing. Until then, at the latest until its next
      load, a `host_only` one is a NameError in its `any` group (logged once), where in group 0 it
      binds at once.
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
16. **Opt-in (Stage S, `slice_steps > 1`; built in `perf/threaded-ss`): runs** (4.4).
    - Processes in one scheduler take turns in runs of up to `slice_steps` steps, so the
      interleaving changes. Each process's step count does not.
    - Watch events coalesce over a run. Watches are polled once per pick, so a watched node changed
      several times in one run gives one event, with its value at the next poll, and a change
      undone within the run gives none.
    - Per-kind step time (`evaluation_stats::step_time`) is apportioned: a run's seconds are split
      over eval, read and data by its steps of each kind.
    - `max_time` is checked there too, and CPU time is charged at the run's end, so it overshoots by
      up to 16 steps. The per-step CPU cap (`max_time / 32`) applies to the run's total, so one
      long native step among cheap ones is charged in full where at slice 1 its charge is capped:
      a run can end a process for time that a pick of one step would not.
    - A process with an async call of its own in flight is charged for it at the 1 ms charge beat
      (and its timeout), not at every step, so its `max_steps` and `max_time` can overshoot by up
      to a run while the call runs.
    - A sleeper at a better level waits up to 16 steps: due timers are checked at those readings. A
      process made ready at a better level by a step or a post, or the runner's own level getting
      worse, ends the run within one step, so the best level is run within one step. The
      `priority` policy's lowest-pid tie-break within a level is not kept: a runner goes on while
      a lower pid at its own level becomes ready.
    - Kept: `max_steps` and the message limits (`max_messages`, `max_message_bytes`) are exact,
      checked before every step of a run (but for a process's own call in flight, above), a posted
      kill lands within one step, and a process with `max_step_time` or `max_memory` keeps one step
      a pick.
    - `slice_steps = 1`, the default, is today's loop exactly.
17. **Opt-in (Stage 2, Q9): the split kernel pools.** CPU-bound and blocking kernels run on separate
    pools.
18. **Opt-in (Stages 1-2): locks and priority across groups**
    ([`docs/STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md)).
    - A contended lock handed to a waiter in another group lands at that group's next drain (the
      next frame in fork-join mode).
    - A `set-priority` on a same-owner process in another group returns `#t` for "requested".
    - Under a priority policy a group's pump share follows its best runnable priority.
    - Inside a group, locks and priority behave exactly as on one scheduler.
19. **Unchanged, deliberately:**
    - fork's shared environments;
    - the shared broadcast envelope inside a group;
    - a host env used as the global env;
    - strict priority within a group;
    - every host API that exists today, apart from `exec_coordinator`'s asynchronous contract
      and Stage 1a's and Stage 1b's always-on changes (all below).

**Hosts (Stage 0, built): `exec_coordinator`'s asynchronous contract** (4.12; S0-Q2, S0-Q3).
`migrate()` returns `{true, -1, target, ""}` once it has posted the migration, and a failure shows
in `stats().migrations_failed` after the scheduler's next step instead of in the result; a sent
migration whose source survived its kill, in the new `migrations_kill_failed`, with the source left
paused for the host to decide. The local `admin_pause`, `admin_resume` and `admin_kill` return true
once posted, an unknown pid included, and act at the next step. A submission or migration from
another node, and a control message, are applied at the next step too. A submission from another
node whose script doesn't parse is answered `accepted: false` and counted in `submissions_received`
and `submissions_rejected`; before, the throw was lost on the bus thread, with no answer and no
count. A local `submit()` of such a script still throws and counts nothing. A host that steps only
while something is runnable steps for a pending command as well (`has_pending_control()`).

**Step counts (Stage 0).** Every existing program takes the steps it took before, except:
- `state-data-set` of a list or dict on the hosts, whose copy can add data steps (item 2);
- the share-mode copies (`state-data-get`, async arguments) of a value holding built-in
  exceptions: they now share the class dicts, so they count fewer data nodes, and on the hosts
  can take a data step fewer at a 1024-node boundary;
- `state-set-tree`, whose stale children cost one unit each, where the sweep counted what it
  removed (item 3);
- the coordinator's tests, which now step before they check what a posted command did (4.12).

**Compatibility checks before Stage 0 merges.** Scan the corpus, the tests and the shipped
documents for each of these, and list any hits in the PR:
- `set-attr` or `del-attr` on built-in classes;
- `state-data-set` of a list that is mutated afterwards, or that holds a callable, a future, a
  lock form or an object instance;
- reliance on `state-delete`'s sweep;
- a chrooted `state-watch` relied on to move the scheduler's settings;
- `(fork)` under Ariadne or pycvc, in the PR of Stage 0 or the locks track's L1, whichever merges
  first (L1's, #862).

Before the locks track's L1b (or Stage 1) merges, scan the same for `spawn` under Ariadne and
pycvc whose child relied on the bare builtins (item 6), and list any hits in its PR.

**Always (Stage 1a, built), besides items 7-9** (S1.md section 1):
- **Another scheduler's threaded future raises** (S1-Q2): `await`, `await-all` and `msg-recv` on
  a threaded future whose owner is not the awaiting process's scheduler raise `RuntimeError`
  "await: the future belongs to another scheduler", origin `"async"` (4.9). An inline future is a
  value, as before. In this repo only a future carried between two pycvc `Exec`s through Python
  reached the check, and it raced before.
- **pycvc's pump-thread rule is enforced** (S1-Q4): while another thread pumps a scheduler (an
  `Exec.run`, a raw pump call), its raw calls, its pump calls, `Exec.run` and the `Exec` calls that
  change its scheduler or natives raise `RuntimeError` "scheduler: another thread is pumping it"
  instead of racing it (4.12). Only `stop()`, `is_running()` and `post_message()` stay callable
  from any thread, and `Exec.set_priority`, `get_priority` and `lock_query` keep their rules. A
  C++ host's debug build aborts on the same race in the schedulers' public mutators (4.1).
- **`kill_owner` tries every kill** (S1.4): it kills the owner's processes from a snapshot, and
  rethrows the first throw after the rest, where one throw used to end the loop (4.5).

**Step counts (Stage 1a).** Every existing program takes the steps it took before: the
registry's and stream's binding, the pid source and the idle collection change no step, and
`state_exec_bench_gates` pins `sched.single.*`'s `steps_total` through a group too. The reworked
tests change none: `AriadneHeap.FinishedActionsAreReapedAndTheirCyclesFreed` (AH-1) reads group
0's registry (reworked in Stage 1a, where the old reads went vacuous), and the deep-data test
threads opt in to the thread-exit collection (RX-1).

**Compatibility scan (Stage 1a)**, at master 5878932d (S1.md section 8; the PR repeats it at its
head):
- `math.random` sequences relied on across schedulers on one thread (item 7): none.
- `tracked_objects()`, `cycle_stats` or `pycvc.collect_cycles()` read after pumping the app
  scheduler or an `Exec` (item 8): only AH-1. The pycvc data test uses a bare evaluator, and the
  heap tests bare schedulers, which keep the thread's registry.
- `on_complete` on a group pumped off the host thread (item 9): none. pycvc's raw
  `execute_options` and the bench's tracker use it on schedulers no worker pumps; `mt.groups`
  reads completion after the join.
- A threaded future awaited from another scheduler: none in the repo.
- A raw pycvc scheduler call from another thread while one pumps: none but `stop()` and
  `is_running()`. Priority calls across the quiesce stay legal.
- `cvc::pump_exec_frame` users (cvcGL tests on `app.exec_scheduler()`): covered by the scheduler's
  own binding (RX-3).
- Trees that set `state_exec.threads`: only the `AppLockManager` tests.
- `kill_owner` callers: none throws from a kill.

**Always (Stage 1b, built)** (S1.md section 1). With `ariadne.exec.isolate_documents` off and no
host calling `exec_pump_frame`, Ariadne runs every document in group 0 as before, with these
changes:
- **The init lane charges its async calls to init's process** (S1-Q2): a provider that registers
  on `*ictx.host` runs its async functions inline on the lane's own host, charged to init's
  process, where nobody was charged before. Their done channels still post to group 0, and the
  lane's cycles are collected and freed when the run ends (4.9).
- **Teardown reaps every process of the Runtime's owner** (S1-Q8): the actions' own processes too,
  which used to stay in group 0's table, killed (HZ-8), and a resident a reload or a mount
  reconcile stops is reaped when it is stopped (4.17).
- **A spawned child takes its context from its parent's** (S1.10): a child of a deleted chroot
  writes its parent's detached node instead of making a node at the old path (4.15, SP-1).
- **`post_input` and `post_signal` are host-thread calls** (S1.10): their header called them
  thread-safe, and a debug build now asserts they run outside a pump (4.12). Every shipped caller
  calls them from its frame thread.
- **A cancelled request stops its transfer** (R1b skeptic m1): an `http-get`, `http-request`, or
  `fetch` of an http(s) URI, whose program is killed, or whose host shuts down, has its transfer
  aborted within about a second (`HttpRequest::cancelled`, 4.9), where it used to run on to the
  reply or its timeout, with its result dropped. The browser's fetch is unchanged.

AppRuntime's names resolve as before (its seven providers keep the one provider's binding order,
4.9), error texts and warnings are unchanged, and the new log lines come only with the setting on
or from a teardown step that throws.

**Step counts (Stage 1b).** Every existing program takes the steps it took before, except an
`init:` script's async calls, now charged to init's process (no shipped document has one). The
reworked test is HZ-8, whose Ariadne case now expects the action's own process reaped too.

**Compatibility scan (Stage 1b)**, at master 5878932d (S1.md section 8; the PR repeats it at its
head):
- `init:` scripts calling async verbs: none. The five shipped documents that call async verbs
  (`oidc_sign_in.ari`, `nav_compute.ari`, `l4_frames.ari`, `net_probe.ari`, `oidc_probe.ari`) call
  them from actions.
- Hosts that drive several Runtimes with `drain()`: none shipped;
  `SeveralRuntimesResumeAnAwaitOncePerFrame` keeps its behaviour.
- Tests or hosts relying on a torn-down Runtime's killed actions staying in the table: HZ-8's
  Ariadne case, reworked.
- A spawned child of a deleted chroot relied on to write at the old path: none.
- Trees that set `ariadne.exec.isolate_documents`: none ship.

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
| One group with Stage S slices | measured by the spike (SS.0, a loaded box): 46% fewer ns/step at `slice_steps` 256 than at 1, a geomean over 17 `sched.single` points (ratios 0.37-0.82) | The clock read, the step-time atomic, drain, watch poll, pick and bookkeeping once per run (the guards were already armed only for limits) | `sched.single.*` `slice_steps` |
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
  `mt.independent` raise/catch, whose per-thread time doubles at 8 threads, before the worker count
  for `auto` (4.1) is settled (Q8).

### 6.3 How the benchmark suite verifies it

**Existing scenarios** that must not regress:
- `sched.single.*`, `flood.*` and `limits.*`: at `threads = 0`, within 3% of the base, since the
  pump scope must cost under 1 ns a step;
- `msg.*`, `async.*`: unchanged at `threads = 0`;
- `mt.independent`: the ceiling, re-measured in the same rep as each `mt.*` point that reports
  `extra.of_ceiling` (`mt.multi_exec` and `mt.groups`; `mt.groups.flood` and `mt.state_tree`
  measure no ceiling); it already runs at t = 12, 16 and 20.

**New scenarios** (in `make_scenarios()` of `state_exec_bench.cpp`, marked `spawns_threads`):

| Scenario | Stage | What it runs | Reports |
|---|---|---|---|
| `mt.multi_exec` | 0 | T threads, each with a full `async_scheduler`, app-tree intrinsics and async host, running fib and raise/catch; the TSan shape. As built, a third program, `prog=tree` (recorded, never gated), runs Stage 0's code on every thread: chrooted state natives, `state-data-set`, a `state-watch`, and controls posted across threads | `extra.speedup`, and `extra.of_ceiling` against `mt.independent` in the same rep |
| `mt.groups` | 1a | `groups` (8, 16) `exec_group`s over the frozen builtins on the app's pool, fork-join, `t` (1, 2, 4, 8, 16) pumping threads (`t - 1` workers and the caller); each group runs `prog` (fib, exceptions, or `cycle`: closure cycles) `runs` (5) times, completion read from process info after each round, not from `on_complete`. As built (Stage 2 adds continuous mode) | `extra.speedup` against one group alone through the pool, `extra.of_ceiling` against `mt.independent` in the same rep with the pool's placement, the pool's `pumps`, `helper_wakes`, `worker_pumps`, `skipped` and `effective_threads` |
| `mt.groups.flood` | 1a | `n` (2000) tiny programs on the app's pool at `t` 1 and 4, through `t` reused groups (`mode=reuse`) or a group per program built and destroyed in the region (`mode=fresh`). The `ExecPool` twin, a pycvc script, is not written yet | `programs_per_s`, `extra.group_setup_ns`, `extra.group_teardown_ns` |
| `mt.pycvc_threads` | 0 | T Python threads, one `Exec` each. As built, a standard-library script (`tools/bench/mt_pycvc_threads.py`) writing the bench's records, not pytest-benchmark; recorded, not gated | speedup |
| `mt.ariadne_docs` | 1b | `docs` (1, 2, 4, 8) `AppRuntime`s on one app, each at its own prefix, loading one eligible document with `ariadne.exec.isolate_documents` on, through `drain_begin` / `exec_pump_frame` / `drain_end`, at `t` (0, 1, 2, 4, 8) pool workers besides the caller; `frame=heavy` (each tick about 8000 steps) and `frame=small` (one `state-incr`). As built ([`STATE_EXEC_BENCH.md`](STATE_EXEC_BENCH.md)) | ns per frame, the pool's `helper_wakes`, `pumps` and `skipped`, `worker_pumps`, and `extra.isolated` (documents in a group of their own), which guards the small-frame gate from passing vacuously |
| `mt.cross_group_msg` | 2 | pingpong within one group and across two; fan-out from one dispatcher to k groups; broadcast to waiters in k groups | ns per message, wake latency percentiles |
| `mt.state_tree` | 1a | `state-get`/`state-set` loops from G (1, 2, 4, 8) groups, a pumping thread each, on disjoint and on shared subtrees, chrooted and not, at depths 1 and 4 | ns per op against G (`extra.ns_per_op`, `extra.group_ns_per_op`) |
| `mt.groups.async` | 2 | async kernels from G groups, `wait_for_work` drivers | completion latency, calls/s |
| `mt.groups.move` | 2, 3 | `CVC_EXEC_FORCE_MOVE` on and off; Stage 3 process moves | cost per group move and per process move |
| `mt.isolated_spawn` | 3 | one parent spawning k `spawn-isolated` fib children that report by message | speedup against `mt.groups` |
| `sched.single.*` `slice_steps` (built for `sched.slice`) | S | the `sched.single` workloads at `slice_steps` 1, 16, 64, 256, 1024, paired in one binary | ns/step; fairness in the bench's Stage S points |
| `mt.groups.locks` | 2 | G groups (one per document or `ExecPool` `Exec`) whose processes take `with-lock` on keys under their own prefix, then on one shared `/` key; the same workload without `with-lock` alternated in the same rep | throughput, cross-group handoff latency |
| `mt.groups.locks.isolated` | 3 | `spawn-isolated` children locking their parent document's keys; disjoint keys, then one hot key | throughput, handoff latency |
| `prio.groups_weighted` | 2 | 2 busy groups at best priorities 0 and 10 at threads 1; 4 busy groups at 0, 0, 10, 10 on 2 workers | `extra.share_ratio` against the CFS ratio expected for the configuration |
| `mt.migrate` | M | a program moved between two hosts on loopback, with and without held leases | move latency, rebind round trips |

**New keys on existing scenarios** (Stage 1a). `sched.single.*`'s async points take `driver`
`group` beside today's `spin`: the same program pumped through an `exec_group` at threads 0 by the
app's `exec_pool`, with the same slices, so `driver=group` against `driver=spin` is the pump
scope's cost, paired within one binary. For `driver=group`, `base` is `own` (the default, the
world's default environment) or `frozen` (the frozen builtins with the natives in a scope over
them, a pooled `Exec`'s shape), whose difference is the frozen base's extra lookup hop. A base
binary's records match the candidate's default points (`driver=spin`, `base=own`).

**Gates.** Base and candidate are interleaved twice with 5 reps, as `docs/STATE_EXEC_BENCH.md`
("Getting stable numbers") prescribes.
- **Stage 0:**
  - `mt.multi_exec` TSan-clean;
  - `sched.single.*` within 3%;
  - `mt.multi_exec` at 8 threads at least 0.9x `mt.independent` (`extra.of_ceiling`, for `prog`
    fib and exceptions).
- **Stage 1** (S1.md section 6; `docs/STATE_EXEC_BENCH.md`, "Threaded execution's Stage 1
  gates"; recorded, as Stage 0's were, when the machine gives no verdict):
  - `steps_total` unchanged on `eval.*`, `sched.single.*`, `flood.small` sync, `native.*`, `data.*`
    and `limits.*`;
  - `sched.single.*`, `flood.*` and `limits.*` at `threads = 0` within 3%, and `msg.*` and
    `async.*` unchanged;
  - the pump scope's cost: `sched.single.*` `driver=group` within 3% of `driver=spin`, paired in
    one binary over runs in both orders, and `base=frozen` against `base=own` recorded;
  - `mt.groups` fib at 8 threads at least 0.9x `mt.independent`'s speedup in the same rep
    (`extra.of_ceiling`), on Windows;
  - `mt.groups` clean under gcc's and clang's TSan (CI);
  - Stage 1b: `mt.ariadne_docs` small frames show 0 helper wakes (`extra.helper_wakes`).
- **Stage 2:**
  - `mt.groups` continuous at 8 threads at least 0.9x;
  - `mt.cross_group_msg` wake latency p50 under 50 us on Windows;
  - `mt.groups.move` group-move cost under 5 us;
  - `mt.groups.locks` with disjoint keys within 5% of the same workload without `with-lock` in
    the same rep;
  - `prio.groups_weighted` within 20% of the CFS ratio expected for each configuration (about 9.3
    between a 0 and a 10 group), or the weights ship off. Weights matter only while runnable
    groups outnumber workers, so the scenario fixes the thread count.
- **Stage 3:** `mt.isolated_spawn` with children of at least 1 ms of work reaches at least 0.8x
  `mt.groups` at 8; `mt.groups.locks.isolated` recorded.
- **Stage S** (`threaded-impl/S2.md` section 6). `sched.slice` is built as a `slice_steps` key on
  `sched.single.*` (1 by default), so both sides of the pair run in one binary:
  - `slice_steps = 1` identical: SL-1, and `steps_total` unchanged against master;
  - the default path: `sched.single.*`, `flood.*` and `limits.*` at `slice_steps` 1 within 3% of
    master (`compare --fail-on-worse --max-noise 3 --same-pin`), so the shared step body costs it
    nothing;
  - `slice_steps = 256` at least 20% fewer ns/step on `sched.single.*`, as a geomean over the
    matched points: `compare POOLED POOLED --pair slice_steps=1:256 --geomean-better 20`, in one
    binary, over runs in reversed order, or the stage is dropped. The spike measured 46% (4.4).

## 7. Phases

Each stage ships on its own, after the previous one, with its tests and its benchmark gate. Stage S
is independent of the others. The locks track (L) needs the perf branches; it brings B28 forward
(its L1) and B27's single-scheduler part (its L1b), and adds `post_control` for `set_priority` if
Stage 0 has not landed (its L2). Stage M comes last.

| Stage | Content | Builds on | Done when |
|---|---|---|---|
| 0 | Process-global hazard fixes; no threads added | `perf/async` (pool), `perf/sched` (ingress flag) | HZ cases pass; TSan clean on `mt.multi_exec` |
| 1 | 1a (built): `exec_group`, a fork-join `exec_pool`, registry and random stream per group, lightweight groups, pid source, pump owner, the foreign-future check, provider kinds, `ExecPool`. 1b (built): host natives classified, Ariadne one pump per frame and isolated documents, the spawn context in Ariadne's groups | Stage 0 | GR, IS-1, IS-2 and AR-1 to AR-4 pass; Stage 1 gate |
| 2 | Continuous pool with wakeups, router with channel homes, cross-group control, published info, bounded ingress, split kernel pools, `exec_coordinator` on the pool | Stage 1; `perf/sched` (timer heap, `wait_for_work`) | PL, MS and AR-7 pass; Stage 2 gate |
| 3 | `spawn-isolated`, spawn groups, movable processes and moves, Ariadne parallel actions | Stage 2 | IS-3 to IS-10, AR-5 and AR-6 pass; Stage 3 gate |
| S | Slices (runs) in today's `async_scheduler`, single-threaded; built in `perf/threaded-ss` | `perf/sched` (direct step, one clock reading); cut from master | SL-1 to SL-10 pass; Stage S gate |
| L | Locks and priority on one machine, one thread and several `Exec`s ([`docs/STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md) L0-L3, with the spawn context on one scheduler in L1b); its L4 rides Stages 1-3 | `perf/sched`, `perf/eval`, `perf/async`; it lands B28 (L1) and B27's single-scheduler part (L1b) itself, and Stage 0's `post_control` for `set_priority` if Stage 0 has not landed (L2) | that document's phase exits, with section 5's compatibility scans; its L1b also passes IS-1 and IS-2 on one scheduler (they move to whichever of L1b or Stage 1 merges first, and Stage 1 re-runs them in groups) |
| 4 | Several threads inside one group (4.16) | Stage 3, and data showing DSL-bound single groups | its own design review |
| M | Moving programs between hosts (4.20); cross-host lock leases (locks L7) | Stage 0 and the locks track's L1-L3, hardened; scheduled last; not Stages 1-4 | MG pass; `mt.migrate` recorded |

**Stage 0: hazards** (built). This design estimated 1.5-2 engineer-weeks; the plan, from reading
the code, 2.5-3: pinned walks through links, chain pins for the writers that span several slices,
the coordinator's other bus-thread paths, and the sync scheduler's control wakeups.
- **Files:**
  - `exceptions.h`/`.cpp` (`is_builtin_class_dict`, and `builtin_class_of_copy`, the exact-shape
    rule that relinks a copy of a built-in class dict), `builtin_class.h` (the shape prefilter),
    `builtins.cpp` (`set-attr`, `del-attr`), and the evaluators' `annotate`
    (`stackless_evaluator.cpp`, `evaluator.cpp`; `async_evaluator.cpp` has no raise path of its
    own), which leaves a built-in class dict a native function throws as it is: read-only class
    dicts;
  - `data_job.h`/`.cpp`: `copy_mode::isolate`, its refusals, built-in class dicts shared in both
    modes, and a memo that holds what it memoized until the copy ends;
  - `state.h`/`state.cpp`: `findThroughLinksShared`, `childThroughLinksShared`, the start-pinned
    `findDescendantShared(self, ...)` and `sharedChild(self, ...)`, and `removeChild`;
    `cvc::pin_vector`, a vector of pins that drops them from the back when it goes, is cleared or
    is assigned over, which `link_resolution::target_pins` and the pinned chains use;
  - `state_json.h`/`state_json.cpp`: the JSON reader writes through held nodes (`state-from-json`,
    `state::json(j)`, `restore()`), and the subtree cursor releases its deepest level first;
  - `intrinsics.h`/`intrinsics.cpp`:
    - the isolating copy in `state-data-set`;
    - pinned lookups, `root_pins` (a `cvc::pin_vector`) and `pinned_root`, and the pinned
      chroot;
    - chain pins for the sliced writers (`state-set` of a list, `state-data-set`,
      `state-from-json`, `state-set-tree`, whose stale children `removeChild` removes);
    - a single-node `state-delete`;
    - the per-watch root, and the pinned root a `state-watch` gives a scheduler with none;
  - `process.h` (`watch_handler_entry::root`) and `scheduler_base.h` (`control_cmd`'s kinds,
    `signal` and `done`; the 5-argument `register_watch_handler`; `watch_root()`; the 2-argument
    `set_watch_root(root, pin)`);
  - `process_table.h`, `process.cpp`: `settings_changes` holds the node it counts, and `clear()`
    lets go of it;
  - `scheduler.h`/`.cpp`: atomic stop flags and a `stop()` that ends the idle wait, holding the
    wakeup it notifies; the idle wakeup made with the scheduler; `post_control`'s kinds, `done`,
    order and wakeups, `has_pending_control()`, and a destructor that refuses until nothing is
    queued; per-root watch gating (`detail::watch_roots`, `watch_polls()`); the watch root's pin;
    settings read and published through `detail::setting_value` and `detail::publish_setting`,
    which `data_job.cpp`'s, `heap.cpp`'s and `parser.cpp`'s settings readers use too;
  - `async_scheduler.h`/`.cpp`: the same atomics, `post_control`, destructor and gating; a drain
    that puts back what a throw left unapplied; an idempotent `finish_async_call` that wakes the
    call's waiters even when it throws; `cancel_async_calls` settling each call it claims with
    the cancelled-call `RuntimeError` and waking every waiter;
  - `async_call.h`, `async_host.cpp`: the `closed_` claim; `outcome()`'s backstop check of a
    cancel, and a built-in class dict left as it is;
  - `exec_coordinator.h`/`.cpp`: control, submissions, `migrate()` and the migrate ingest through
    `post_control`; `admin_pause/resume/kill` posted; `policy_` read under its lock; the lifetime
    token; a failed migration resumes its process, and a source that survives its kill is left
    paused (`coordinator_stats::migrations_kill_failed`);
  - `src/cvc-cli/main.cpp`: the serve loop steps for a pending command, and logs a step that
    throws (`exec: <what>`) and goes on serving;
  - `state_value_codec.cpp`: a restore relinks a decoded dict of exactly a built-in class's shape
    to that class, so a restored exception keeps its read-only class (and can be stored);
  - `bindings/pycvc`: `pycvc_state.cpp`'s pinned accessors, `state_remove` through `removeChild`,
    and a `state_observer` that is never called once dropped; `pycvc.i`'s converter relinking a
    built-in class (`builtin_class_of_copy`), the pinned walks and `removeChild` kept out of
    Python, and which calls are thread-safe; `pycvc_exec.cpp`'s async-argument copy label and
    its held read of `state_exec.policy`;
  - `builtins.cpp`: line-atomic `print` (`write_print_line`, whose lock outlives static
    destruction);
  - not changed: `sendMessage` (`msg-send` holds its node's chain across it instead), and `(fork)`
    of `current_pid()` with the child's `owner`, which the locks track's L1 landed;
  - the bench (`state_exec_bench.cpp`: `mt.multi_exec`, `chroot` sizes for `native.state_get`,
    `native.state_set` and `sched.watch_poll`, `native.state_delete`), `tools/bench`
    (`mt_pycvc_threads.py`) and [`STATE_EXEC_BENCH.md`](STATE_EXEC_BENCH.md);
  - docs: [`STATE_EXEC_DEVELOPER_GUIDE.md`](STATE_EXEC_DEVELOPER_GUIDE.md),
    [`STATE_EXEC_EXCEPTIONS.md`](STATE_EXEC_EXCEPTIONS.md) (the class and data-channel rules),
    `docs/roadmap/STATE_LIFETIME_AND_ATOMICITY.md` 3.4, `docs/roadmap/STATE_EXEC_ROADMAP.md`
    ("Planned"), `bindings/pycvc/README.md` (the pump-thread rule), and this document.
- **CI:** C0's `sanitizers` job, extended (S0-Q1): gcc 13 on ubuntu-24.04, legs `tsan` and
  `asan-ubsan`, pycvc included. It runs Stage 0's binaries (`state_exec_hazards_test`,
  `state_lifetime_stress_test`, `state_exec_intrinsics_test`, `state_exec_exceptions_test`,
  `state_exec_async_calls_test`, `state_exec_data_steps_test`) beside the locks track's, adds
  `test_pycvc_state.py`, and runs `mt.multi_exec` at t = 8 (`.github/sanitizers/run.sh`'s
  `SAN_BENCH`). Every HZ case's line, and each of `mt.multi_exec`'s three points, is required in
  both legs. The suppressions files are empty. The state_exec binaries no sanitizer ran before
  join after a baseline run. A Linux clang TSan job stayed Stage 1's (TS-1), and Stage 1a added it.
- **Tests:** HZ-1 to HZ-14, and the `post_control` cases (`AsyncPostControl`, `SyncPostControl`).
- **Exit:** HZ-1 to HZ-14 pass; every state_exec, Ariadne and pycvc suite passes on Windows,
  Linux and macOS, with step counts unchanged except as section 5's "Step counts" lists; both
  sanitizer legs are clean with every required line found; the compatibility scan of section 5
  is in the PR; no thread is added.
- **Risk:** low. The behaviour changes are items 1-4 of section 5 and the coordinator's
  asynchronous contract (section 5, "Hosts"); item 5 landed with the locks track's L1.

**Stage 1: groups and a fork-join pool.** This design estimated 3-4 engineer-weeks; the plan
(`threaded-impl/S1.md`), from reading the code, 6-7 weeks plus a 2-day baseline for the clang leg.
What it added: the cycle registry and random stream bound by the schedulers themselves (several
drivers step group 0 with no pump scope) with final collections at every owner; a pump owner on
both schedulers and its check in pycvc (S1-Q4); the lock attach and the manager's mode (S1-Q1);
worker stacks and CPU placement; the init lane's host and the foreign-future check's owner accessor
(S1-Q2); more B30 sites and two providers to split by kind (S1-Q7); isolation eligibility and a
subscriber audit (S1-Q6); the host-call queue's rules and concurrent `pump_all` callers (S1-Q11);
four bench scenarios and a clang TSan leg. It ships as two PRs (S1-Q12): **Stage 1a** (groups, the
pool, pycvc; about 4 weeks), then **Stage 1b** (Ariadne's half; 2-3 weeks), each with its own
reviews and A/B.

**Stage 1a** (built).
- **Files:**
  - new `exec_group.h`/`exec_group.cpp` (`inc/cvc/state/state_exec`, `src/cvc/state/state_exec`):
    the group, its pump and token, its teardown, `execute`'s AST copy, `install` by kind and the
    `host_proxied` wrapper;
  - new `exec_pool.h`/`exec_pool.cpp` and `cpu_placement.h`/`cpu_placement.cpp` (`inc/cvc/core`,
    `src/cvc/core`): the fork-join pool (LPT, the adaptive helper wake, the frame deadline, the
    host-call queue, deferred `on_complete`, deterministic mode, failures, several callers) and the
    CPU placement the pool and the bench share;
  - `app.h`/`app.cpp`: `exec_pool()`, `exec_pool(pool_options)`, `exec_pool_probe()`,
    `exec_pump_frame()`, `exec_pump_generation()`, `register_frame_group` and
    `frame_group_handle`, `exec_pid_source()`; group 0's registry, stream and pids
    (`exec_group0_state`); the pool's settings, `CVC_EXEC_THREADS` and the lock-mode hint; `~app`
    destroying the pool first;
  - `heap.h`/`heap.cpp`: the opaque `cycle_registry`, `cycle_registry_scope`, the deleter, the
    explicit-registry overloads, registry ids, the thread-exit opt-in;
  - `stdlib.h`/`stdlib.cpp`: `random_stream` and `random_stream_scope`, `math.random` from the
    bound stream; `builtins.h`/`builtins.cpp`: `frozen_builtins()`;
  - `async_scheduler.h`/`.cpp` and `scheduler.h`/`.cpp`: the registry and stream bound on every
    entry and named for the due check and the exit note, the idle collection, the pump owner and
    its debug check, `close_lock_port()`; on the async one also the base-env constructor, the pid
    source, `kill(pid, reason)` and `kill_owner` over a snapshot; `process.cpp`: the debug check
    that an insert never replaces a live pid;
  - `stackless_evaluator.h`/`.cpp`: `awaitable::owner()`, `owner_serial()` and
    `kForeignFutureError`, the closure-body cache tagged by registry; `async_call.h`: `owner()` and
    `owner_serial()`; `scheduler_base.h`: `serial()`;
  - `intrinsics.h`/`intrinsics.cpp`: `ictx.host`, `provider_affinity`, `make_spawn_context` copying
    `host`, the foreign-future check in `await` and `await-all`;
  - `bindings/pycvc`: `pycvc_exec.cpp`/`.h` (each `Exec`'s registry and stream, `ictx.host`, the
    pump owner in `run()` and the priority calls, the pooled `Exec` in group mode and `run_batch`),
    new `pycvc_exec_pool.cpp`/`.h` (`ExecPool`), `pycvc.i` (the pump-owner check on the raw calls,
    `ExecPool`, `Exec.tracked_objects()` and `Exec.collect_cycles()`), `README.md`, and the wasm
    smoke line;
  - the bench (`state_exec_bench.cpp`: `mt.groups`, `mt.groups.flood`, `mt.state_tree`,
    `sched.single.*`'s `driver=group` and `base`; the placement through `cpu_placement.h`),
    `state_exec_bench_gates.cmake`, `tools/bench`, and [`STATE_EXEC_BENCH.md`](STATE_EXEC_BENCH.md);
  - CI: the sanitizer jobs run `state_exec_groups_test`, `test_pycvc_exec_pool.py` and `mt.groups`
    at `t=8 groups=8`, and a third leg, `clang-tsan`, builds with the runner's clang (S1-Q9; its
    TSan runtime from `libclang-rt-<major>-dev` when the image lacks it), where
    `.github/sanitizers/run.sh` preloads clang's shared TSan runtime for Python, since clang links
    its runtime into each executable; `state_exec_wasm_smoke` pumps an `exec_pool` at threads 0;
    [`TESTING.md`](TESTING.md) lists the legs;
  - docs: this document, [`STATE_EXEC_DEVELOPER_GUIDE.md`](STATE_EXEC_DEVELOPER_GUIDE.md),
    [`APP_API.md`](APP_API.md), [`STATE_EXEC_EXCEPTIONS.md`](STATE_EXEC_EXCEPTIONS.md),
    [`STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md) (9.2, 12) and [`STATE_API.md`](STATE_API.md) (the
    state APIs Stage 0 added).
- **Tests:** `state_exec_groups_test` (the `Groups` suite: GR-1 to GR-12, RX-1 to RX-3, PO-2, FF-1,
  FF-2, GL-1, GL-2, RB-1, AS-1, BF-1, OC-1, PA-1, DT-1, DT-2), `exec_pool_test` (the pool, the
  group and the app's pool, by their implementer), KO-1 and KO-2 in `state_exec_async_test`, the
  unit cases in the heap, scheduler, async-calls and intrinsics suites, and pycvc's
  `test_pycvc_exec_pool.py` (AR-4, IS-2 in a pooled `Exec`, GR-5's pycvc half, FF-3's pycvc half,
  PO-1) with cases in `test_pycvc_exec.py` and `test_pycvc_state_exec_locks.py` (section 8).
  AH-1 is reworked in Stage 1a, since group 0's registry made the old reads vacuous.
- **Exit:** these pass; every `state_exec_*`, Ariadne, cvcGL and pycvc suite passes on Windows,
  Linux and macOS with step counts unchanged (5, "Step counts (Stage 1a)"); the gcc TSan, clang
  TSan and ASan legs are clean with every required line found; TS-2's MSVC ASan run and TS-3's
  `CVC_EXEC_VERIFY` run are recorded in the PR; the compatibility scan is repeated at the PR's
  head; the Stage 1 gates pass or are recorded as no verdict with the owner's agreement; with
  `state_exec.threads` unset and no host opting in, nothing runs on a worker.
- **Risk:** medium-low. The host-call queue is the one new blocking protocol (a worker waits for the
  caller), kept deadlock-free by its rules (4.9); a host that relied on pycvc's raw cross-thread
  race now gets a `RuntimeError`; a host whose app made a single lock manager before its pool gets
  an inline pool and a log line.

**Stage 1b** (built): Ariadne's providers on `ictx.host`, registered by kind (S1-Q7), with posts
on the caller's group; the init lane on an inline host of its own (S1-Q2); `drain_begin` /
`drain_end` and one pump per frame; isolated documents with S1-Q6's eligibility, their placement
and its moves; `spawn_child` from its parent's context; the teardown reap (S1-Q8); the subscriber
audit and the thread-affine declarations; `uri_state`'s and streams' pin vectors; `mt.ariadne_docs`.
- **Files:**
  - `ariadne.h`/`ariadne.cpp`: `Runtime::drain_begin`, `drain_end` (`pump_frame`,
    `sweep_actions`); `register_action_intrinsics(app, provider, affinity)`, the app's list keeping each
    provider's kind, and `install_providers`; the init lane's inline host and cycle registry
    (`run_scoped_program`); `ictx.host` in every context; isolated documents
    (`ensure_placement`, `isolation_verdict`, `host_only_bindings`, `probe_env`, `move_out`,
    `kill_and_reap`), `isolated_group`, `isolation_reason`, `current_group`,
    `declare_thread_affine_prefix`, `set_document_programs`; `has?`'s snapshot for an `any` group;
    `spawn_child` from the parent's context; the teardown's order and reap, and `stop_resident`'s
    reap; retired groups (`destroy_idle_retired_groups`) and late registrations
    (`nothing_running`); the host-thread asserts (`assert_outside_pump`) and the refusal by a held
    group (`refuse_if_held`);
  - `app_runtime.h`/`app_runtime.cpp`: seven providers by kind, in the old binding order; done
    channels on the calling program's group; `drain_begin`, `drain_end`,
    `declare_thread_affine_prefix`, `wait_idle`; `idle()` counting the document's group; `load()`
    handing over `init:` and `tests:` and deciding the placement; the verb table's lock;
  - `intrinsics.h` (the provider rules), `async_scheduler.h`/`.cpp` (`ingress_posts_for_test`, for
    AR-3b), `async_host.h`/`.cpp` (`finisher`, `make_finisher`; a shutdown that runs its calls'
    cancel hooks outside its lock);
  - `net_intrinsics.h`/`.cpp`: two providers by kind on `*ictx.host`, a started request keeping its
    host's finisher, the kernels' cancel passed to the request, `resolve` and `renew`;
    `http_client.h`/`.cpp`, `http_client_curl.cpp`: `HttpRequest::cancelled`; `uri.h`:
    `UriCancelled`; `credentials.h`: `renew`'s `cancelled`; `ws_client.cpp`: a frame sent whole;
  - `stream.h`/`stream.cpp`: `stream_params::scheduler`, posts under a per-stream lock, the member
    `stream::retarget(from, to)`, the descriptor pins a `cvc::pin_vector`;
    `stream_intrinsics.h`/`.cpp`: `any`, posting to the opener's group;
    `retarget_document_streams` and `close_document_streams` over a document's stream table;
  - `room_intrinsics.cpp`, `room_media_intrinsics.cpp` (a subscribed proxy on the caller's group),
    `oidc_host.cpp`: `host_proxied`;
  - `uri_state.cpp`: its chains are `cvc::pin_vector`s;
  - `exec_pool.cpp`: a force-moved call ends at its deadline (4.4);
  - `state_sync_adapter.h`/`.cpp`: what is not thread-safe, in comments;
  - cvcGL: `AriRuntime.h`/`.cpp` (the scene prefix declared, `wait_idle` through
    `AppRuntime::wait_idle`); the debug home-thread asserts (`CameraController.cpp`,
    `TouchGestures.cpp`, `StageLighting.cpp`, `FpsHud.cpp`, `ScreenTextHud.cpp`, `Settings.h`,
    `Settings.cpp`); `GraphicsNode.h`/`.cpp` (the pose mark, thread-local);
  - `examples/ariadne/nav_compute/nav_compute.cpp`: `any`, on `*ictx.host`;
  - `bindings/pycvc/pycvc_ariadne.i`: `AriApp.wait_idle` through `AppRuntime::wait_idle`;
  - the bench (`state_exec_bench.cpp`'s `mt.ariadne_docs`, `tools/bench`'s gate guard
    `GATE_GUARDS`) and [`STATE_EXEC_BENCH.md`](STATE_EXEC_BENCH.md);
  - CI: the CTest entries `ariadne_runtime_test_threads_auto` and `ariadne_host_test_threads_auto`
    (AR-1: both suites again with `CVC_EXEC_THREADS=auto`, `AppLockManager.*` filtered out); the
    sanitizer jobs' `SAN_REQUIRED_S1B`, required in every leg; the clang leg runs
    `state_lifetime_stress_test` and `ariadne_runtime_test`, the latter at `CVC_EXEC_THREADS=4`
    with `AppLockManager.*` filtered out (its `cxx_runs`, `run.sh`'s `SAN_CXX_RUNS`);
    [`TESTING.md`](TESTING.md);
  - docs: this document, [`ARIADNE.md`](ARIADNE.md),
    [`STATE_EXEC_DEVELOPER_GUIDE.md`](STATE_EXEC_DEVELOPER_GUIDE.md), [`APP_API.md`](APP_API.md),
    and the pycvc README's `AriApp` passage.
- **Tests:** IS-1b, AR-1, AR-2, AR-3, AR-3b, SP-1, EL-1, FF-3's init case, HZ-8's rework and
  PIN-1 (section 8), and the implementers' own cases beside the code's existing tests (8.6).
- **Exit:** these pass; every Ariadne, cvcGL and pycvc suite passes on Windows, Linux and macOS
  with step counts unchanged (5, "Step counts (Stage 1b)"); the three sanitizer legs are clean
  with every Stage 1b line found; with `ariadne.exec.isolate_documents` off and no host calling
  `exec_pump_frame`, nothing runs on a worker (AR-1); `mt.ariadne_docs`' small frames wake no
  helper, or the gate is recorded as no verdict with the owner's agreement.
- **Risk:** the surprises are host natives and subscribers that assume the main thread. Any
  doubtful one defaults to `host_only` or not thread-safe, and arigl's documents stay `host`.
- **Follow-ups** (recorded, not built):
  - every AppRuntime's verbs bind into every Runtime's env (the process-global registry; only the
    `data-*` provider checks its program is its own), so a host with several AppRuntimes, one of
    which registered a host verb or function, makes every isolated document `host`: affinity counts
    other hosts' verbs (pre-existing cross-binding; conservative, R1b-mid s-m5);
  - no host API declares a host verb's or function's kind other than `host_only`: a host that
    wants an `any` verb registers a provider of its own (4.9);
  - the subscriber audit's (4.8): `state_sync_adapter`'s pinned walks and atomic `_attached`;
    the settings classes and render-thread subscribers moving out of arigl's document prefix;
    volren's and volslice's settings; `state_object`'s teardown against a slot on another thread,
    which covers the two-scenes read in `GraphicsNode`.

**Stage 2: continuous pool, router and control** (3-4 weeks).
- **Files:**
  - `exec_pool.cpp`: the state word, ready queue, eventcount, `next` slot, timers, `pump_host`,
    `with_group`, the pid directory, published info, `kill_owner` with acknowledgement;
  - new `exec_router.h`/`.cpp`: homes, the subscription protocol, requeue, caps;
  - `intrinsics.cpp:960-1015, 1167-1191`: `msg-send` and `msg-recv` through the router when the pool
    has more than one group;
  - `async_scheduler.cpp`: the ingress entry kinds;
  - `app.cpp`/`async_host.cpp`: the split kernel pools (Q9) and the long-step monitor;
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

**Locks across Stages 1-3** ([`docs/STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md) L4, about 1 week per
stage).
- Stage 1: a lock home per group over the app's lock manager, the spawn context (on one scheduler
  since the locks track's L1b) in groups.
- Stage 2: `wake(g)` after lock posts, cross-group `set-priority`, `best_runnable` and the pool
  weights, `worker_class`.
- Stage 3: `adopt` renames a holder's lock record, through the lock protocol's `move`, which is
  specified, proved and model-checked first; `group_scheduler_ref` forwards `locks()` and the
  priority virtuals (`set_priority`, `get_priority`, `set_priority_checked`, `get_priority_as`,
  `set_priority_floor`, `policy`), so a movable process's `set-priority` and `get-priority`
  reach its group's scheduler.
- **Tests:** LG-1 to LG-7. **Exit:** each stage's gate, plus `mt.groups.locks` and
  `prio.groups_weighted` in Stage 2, and `mt.groups.locks.isolated` recorded in Stage 3.

**Stage S: slices** (built in `perf/threaded-ss`). This design estimated 1-2 engineer-weeks; the
plan (`threaded-impl/S2.md`, SS), from reading the code, 11-14 engineer-days. A spike (SS.0) first
showed the gain: 46% fewer ns/step at 256 (4.4).
- **Files:**
  - `async_scheduler.h`/`.cpp`: `set_slice` (`set_slice_steps`, `set_slice_seconds`),
    `slice_steps()` and `slice_seconds()`; the tree's
    settings (`refresh_slice`) and `CVC_EXEC_SLICE_STEPS`; `run_loop` taking `step_once` at 1 and
    `step_run` above it; the run in `execute_process_step` behind `max_run > 1`, its limits in
    `continue_run`; the run's CPU charge, and its seconds split by kind;
  - `process.h`: `charge_cpu_time(seconds, steps)`, the scaled cap; `evaluator.h`:
    `add_step_time(kind, seconds)`;
  - `process_table.h`, `process.cpp`: `begin_run`, `end_run` and `preempted`, raised in the
    enqueue;
  - `exec_group.h`/`.cpp`: `options::slice_steps` and `slice_seconds`;
  - not changed: `scheduler_base.h` and the sync scheduler;
  - `state_exec_groups_test.cpp`: `Groups.OC01` pins `slice_steps` 1, since it asserts round
    robin's completion order;
  - the bench: `sched.single.*`'s `slice_steps` key (`state_exec_bench.cpp`; 1 by default, smoke 1
    and 256);
  - docs: this document and [`STATE_EXEC_DEVELOPER_GUIDE.md`](STATE_EXEC_DEVELOPER_GUIDE.md).
- **Tests:** SL-1 to SL-10 (8.10); the async suite at `slice_steps` 1, and at
  `CVC_EXEC_SLICE_STEPS=256` with the cases that assume per-step interleaving tagged.
- **Exit:** with `slice_steps` unset (1), behaviour and step counts are today's exactly (SL-1); the
  Stage S gate (6.3).
- **Risk:** low.

**Stage 4: several threads inside one group** (8-12 weeks, high risk). Only on evidence: Stage 2-3
benchmarks or user reports showing a single group that is DSL-bound and cannot be split. It gets
its own design review, starting from section 4.16.

**Stage M: moving programs between hosts** (3-4 weeks, scheduled last; it needs Stage 0 and the
locks track's L1-L3, hardened, and no groups).
- **Files:**
  - `exec_coordinator.*`: `migrate` ships `encode_process`, waits for the target's acknowledgement,
    then kills its copy; it refuses a lock holder unless asked to release its locks, checked and
    released in the paused control entry (locks L7); control messages for a moved pid answer with
    its new place;
  - `state_value_codec.*`: the inbox, watches and pending signals; `priority_floor`; held lock
    leases (locks L7);
  - `state_change_journal.h`, `state_cluster_shard.cpp`: lock fencing (locks L7).
- **Tests:** MG-1 to MG-6.
- **Exit:** a program moved mid-run between two hosts on loopback ends as an unmoved run does, with
  its inbox, watches, signal handlers and pending signals.
- **Risk:** medium. Pids and leases across hosts, and a host failing mid-move.

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
| HZ-8 | `(fork)` in a pycvc program and in an Ariadne action | the running process forks: both continue, with 0 and the child's pid; the child has the parent's `owner`; `kill_owner` reaps it. `kill_owner` exists on the async scheduler only, so the reaping half is checked there, under pycvc's `Exec` and Ariadne's app scheduler (both schedulers' fork is covered by `LocksReview.ForkInAStepReturnsZeroInTheChild` and `Locks.LW06_Fork`). As built in Stage 0, the Ariadne case's teardown reaped the forked child and left the action's own process in the table, killed with `owner_reaped`; S1-Q8 decided that teardown reaps it too, and Stage 1b reworked the case (`AriadneHazards.HZ08_ForkedChildKeepsOwnerAndTeardownReapsIt`): after teardown the forked child and the action's own process are both gone |
| HZ-9 | `exec_coordinator` control messages on a bus thread while the scheduler runs (TSan) | no race; the commands apply between steps |
| HZ-10 | `mt.multi_exec` at 8 threads under TSan | clean |
| HZ-11 | two `pycvc.Exec`s on two Python threads, raising and catching built-in exceptions and using `state-data-set` (Linux TSan) | correct results, clean |
| HZ-12 | `print` from 8 schedulers on 8 threads, 1000 lines each | every line whole |
| HZ-13 | a future awaited twice (`finish_async_call` reached twice) | one charge, one result |
| HZ-14 | `stop()` from another thread during `run()` on both schedulers (TSan) | clean; run returns |

### 8.2 Groups and the fork-join pool (GR, Stage 1)

As built in Stage 1a: the gtest cases are `Groups.<ID>_<Name>` in `state_exec_groups_test.cpp`
(GR-5's pycvc half, PO-1 and FF-3's pycvc half are in `test_pycvc_exec_pool.py`, KO-1 and KO-2 in
`state_exec_async_test.cpp`). Oracles that compare step counts run with
`state_exec.async_steps_per_cpu_ms` at 0, so an async call is charged a fixed step and no CPU
time. Timing assertions are one-sided, with generous ceilings; no name says "Stress".

| ID | Situation | Result |
|---|---|---|
| GR-1 | **The oracle** (`GR01_EveryGroupEqualsItsSoloRun`). G groups (2, 8, 32) x M processes mixing fib, exception loops, cycle-making closures, `set-attr` on own dicts, `state-set`/`get`/`delete` under the group's prefix, in-group `msg-send`/`msg-recv` with broadcast, fork, spawn (through `make_spawn_context`), and `await` on inline-executor kernels. Each group run alone on one thread with steps-only budgets, then all on the pool at threads 1, 2, 4, 8, 16 (the effective count printed) | every group's per-process results, step counts, exit statuses and registry's `tracked_objects()` after a final collection equal its solo run (time limits excluded). With workers, some pump ran on one (`stats().worker_pumps`; GR-2 and GR-3 too) |
| GR-2 | GR-1 with `CVC_EXEC_FORCE_MOVE` and seeded random pump budgets, the solo runs given the same budgets (`GR02_ForcedMovesAndRandomBudgets`) | the same |
| GR-3 | GR-1 with host-affinity groups mixed in, every third, each recording its native's thread (`GR03_HostAffinityGroupsStayOnTheCaller`) | the same; every pump of a host-affinity group, and every call of its native, ran on the caller's thread |
| GR-4 | a cycle made in group A, whose process then parks, while B on another thread collects (`GR04_ACycleBelongsToItsGroup`) | B's collections and registry never see it; A's registry tracks it and, once A's process ends, frees it |
| GR-5 | a group destroyed after its last process, with cycles through its global env (`GR05_ARegistryDiesWithItsGroup`); an `Exec` run from 100 short-lived Python threads, each making cycles (`test_gr05_exec_from_short_lived_threads`) | its registry and every cycle in it freed (weak observers); the `Exec`'s `tracked_objects()` stays bounded, and the ASan leg finds no leak |
| GR-6 | `math.random` with a fixed seed in a group pumped by a different worker every pump (`GR06_ARandomStreamFollowsItsGroup`) | the stream equals a one-thread run's |
| GR-7 | pids from 8 groups and the app scheduler on one pool, submitted in turn and spawned during the pumps (`GR07_PidsAreUniqueAcrossGroups`) | unique app-wide, increasing within each group (narrowed from "increasing app-wide": one source gives pids in creation order, and a group sees only its own) |
| GR-8 | `pump_all` of 16 groups each with 0.1 s of work, threads 4, frame deadline 0.1 s (`GR08_TheFrameDeadlineBoundsAFrame`) | the groups that did not start count in `stats().skipped` and go first next frame; the frame's time is printed, under a generous ceiling |
| GR-9 | last-pump durations injected (`set_last_pump_for_test`) under 100 us in total, then above (`GR09_SmallFramesWakeNoHelper`) | no helper woken (`stats().helper_wakes` 0); then helpers woken |
| GR-10 | a `host_only` and a `host_proxied` native installed by kind in a worker-pumped group (`GR10_HostOnlyAndHostProxiedNatives`) | NameError; the `host_proxied` native runs on the caller's thread, outside any pump, and the process resumes at the group's next pump; at threads 0 it runs inline |
| GR-11 | a `host_proxied` native called inside a generator body on a worker (`GR11_AProxiedCallInAGeneratorBlocksItsWorker`) | blocks that worker until the caller runs it during the join; the frame completes, under a generous ceiling; pumped by the caller, it runs inline |
| GR-12 | `pump_hook_for_test` throws a C++ exception from one group's pump, on a worker and on the caller (`GR12_AnExceptionEscapingAPumpFailsOnlyItsGroup`) | that group in `failed_groups()`, its processes killed with "internal error", `pump_all` does not throw; other groups equal their solo runs |
| RX-1 | a bare scheduler on a `boost::thread` leaves a cycle and the thread ends, with and without `collect_cycles_at_thread_exit()` (`RX01_AnOptedInThreadsCyclesAreCollectedWhenItEnds`; S1-Q3) | with it the cycle's weak observer has expired after the join, without it not; the ASan leg finds no leak |
| RX-2 | on a bare scheduler with `set_cycle_registry`, `kill(pid)` from the host thread between pumps (`RX02_AHostKillNotesTheSchedulersRegistry`) | that registry's exit is pending, not the thread's or a registry bound around the kill |
| RX-3 | one scheduler stepped through `cvc::pump_exec_frame`, `sync_run`, `sync_step` and `restore` (`RX03_EveryDriverTracksIntoTheSchedulersRegistry`) | every new cycle is in its registry; the thread's own gains nothing |
| PO-1 | `Exec.run` pumping on thread A; raw calls and `register_fn` from thread B (`test_po01_raw_call_while_another_thread_pumps`; S1-Q4) | `RuntimeError` with S1-Q4's text; `set_priority` across the quiesce works; after `run()` the raw call works; `stop()` and `is_running()` work throughout; `run` still releases the GIL |
| PO-2 | the pump owner (`PO02_TheOwnerRule`) | a scope nests on one thread; another thread's is refused while one is held; a sync scheduler pumped from successive threads works |
| FF-1 | a threaded future of scheduler A awaited from B, through `await`, `await-all` and `msg-recv` (`FF01_AnotherSchedulersFutureRaises`; S1-Q2) | `RuntimeError` with S1-Q2's text at once; A's call list untouched, and A's own await gets the value |
| FF-2 | an inline future awaited from another scheduler (`FF02_AnInlineFutureIsAValue`) | its value |
| FF-3 | `Exec` B awaiting `Exec` A's threaded future (`test_ff03_future_across_execs_raises`); Stage 1b's init case, a new one beside an unchanged HZ-13 (`AriadneHazards.FF03_InitLaneRunsProviderCallsOnItsOwnHost`): a provider registering on `*ictx.host`, from `init:`, awaits a future twice and calls the function once | raises at once instead of waiting to its idle limit; the init lane gives `(21 21 6)`, its providers saw a host that is neither null, nor the app's, nor one that parks, and `(budget-usage)` in init's process counts its 2 calls, a step each at least |
| GL-1 | a holder in group A and a waiter in group B on one key, at threads 0 (single manager) and 4 (shared) (`GL01_TwoGroupsExcludeOneKey`; S1-Q1) | B waits; once A releases, B is granted at its next pump |
| GL-2 | group B destroyed with a waiter while A holds; `teardown_hook_for_test` releases A between B's port close and its kills (`GL02_TeardownClosesThePortBeforeItsKills`) | B's waiter's kill releases the grant, and group C's waiter is granted |
| KO-1 | the step rate's publication during `kill_owner` fires a tree slot that reaps one of the owner's processes and submits another (`AsyncScheduler.KO01_KillOwnerSurvivesSlotsThatReapAndSubmit`) | every process of the owner alive at the call is killed; the count is right |
| KO-2 | one kill throws (`AsyncScheduler.KO02_AThrowingKillDoesNotStopTheRest`) | the others are still killed; the first throw propagates |
| RB-1 | two groups alternate on one thread, then with `CVC_EXEC_FORCE_MOVE`, making closures from one lambda form with equal atom bodies (`RB01_ClosureBodiesAreNotSharedAcrossGroups`) | each group's body lists are its own |
| AS-1 | one `value_t` with a quoted literal executed into two groups (`AS01_ASharedAstIsCopied`) | one mutates its literal; the other, and the host's AST, see the original |
| BF-1 | an env-less top-level `set` of a builtin's name in group A (`BF01_TheFrozenBaseIsShared`, `BF01a_TheFrozenScopeRefusesASet`) | group B does not see it; the frozen scope keeps the builtin and refuses a `set` |
| OC-1 | `on_complete` in a worker-pumped and a caller-pumped group, one throwing (`OC01_OnCompleteRunsOnTheCallerAfterTheJoin`) | the worker-pumped group's run on the caller after the join, in completion order, the throw rethrown after the others ran; the caller-pumped group's run inline |
| PA-1 | two threads call `pump_all` on one pool with disjoint groups, one making a `host_proxied` call (`PA01_TwoCallersShareThePool`; S1-Q11) | both finish; each caller services only its own queue; a group given to both asserts in debug builds |

The pool's implementer added `exec_pool_test.cpp` (`CpuPlacement`, `ExecPool`, `ExecGroup`,
`AppExecPool`: clamps, order, the helper wake, deadlines, failures, `on_complete`, install by kind,
the AST copy, the teardown hook, the app's pool and its lock mode, `exec_pump_frame`, the shared
pid source), and with the R1-mid fixes: single-manager groups kept on the caller, a busy group 0
not starving frame groups, `wants_pump` counting a frame-waiter, threaded callbacks after the join
with their registry, the default owner, deterministic group 0, failed groups per caller, and
`ExecGroupDeathTest.DestroyedFromInsideItsOwnPumpAborts`. `state_exec_async_calls_test.cpp` has
`AsyncCalls.AwaitAllRefusesAForeignFutureBeforeItWaits` and
`AsyncCalls.AFutureOfAGoneSchedulerRaisesAtItsAddress` (the serial check). pycvc's
`test_pycvc_exec.py` covers `ExecPool`'s order, async functions, first failure, idle limit
(frame-yielding programs kept alive), default size and `close()`, and `Exec.collect_cycles()`;
`test_pycvc_state_exec_locks.py` covers the pump owner on the raw pump calls and `run()` holding
it until it returns.

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
| IS-1 | an Ariadne action `spawn`s a child that `msg-send`s on a document channel and `state-set`s (Stage 1); Stage 1b's IS-1b, the same in an isolated document's group under `CVC_EXEC_FORCE_MOVE`, so workers pump it (`SpawnContext.IS01b_InAnIsolatedDocument`) | the message reaches a resident; the write lands under the document's prefix; `kill_owner` reaps the child. In IS-1b also: a worker pumped the group, the child is in the group and not in group 0, nothing is written outside the prefix, and the group goes with the document |
| IS-2 | a pycvc program `spawn`s a child that calls a registered Python function (Stage 1; Stage 1a re-runs it in a pooled `Exec`, `test_is02_in_a_pooled_exec`) | the child calls it; the parent's top-level definitions persist across `run()`, and in a pooled `Exec` across its programs |
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
| AR-1 | 1b | the Ariadne host and runtime suites with `CVC_EXEC_THREADS=auto` and no document opted in (CTest `ariadne_runtime_test_threads_auto`, `ariadne_host_test_threads_auto`, `AppLockManager.*` filtered out); and three documents in group 0 at threads `auto` running an `init:` script, tick residents, actions, a spawned child and an await through `drain_begin` / `exec_pump_frame` / `drain_end` (`AriadneFrames.AR01_NoDocumentOptedInStaysOnTheFrameThread`) | both suites pass; an `any` probe native in every lane ran on the frame thread only, and the app's pool counted no worker pump (`pool_stats::worker_pumps`) |
| AR-2 | 1b | 3 documents in group 0 via `drain_begin` / `exec_pump_frame` / `drain_end`; an action `(await (slow-op-async))` whose call ends between two Runtimes' `drain_end`s (`AriadneFrames.AR02_ThreeDocumentsInGroupZeroPumpOnceAFrame`) | resumes exactly one frame later, at the next frame's pump, not in a later `drain_end` of its frame; another component's `(await #f)` loop advances once a frame |
| AR-3 | 1b | 4 eligible isolated documents, each with a busy resident, threads 4 (`AriadneFrames.AR03_IsolatedDocumentsRunOnWorkers`) | residents step on at least two threads and workers pump; no frame begins inside a `pump_all` (the test renders between pumps itself, so that check cannot fail: the real latch is render's debug assertion, compiled out of Release builds); a quick action completes in its drain, or in the next frame when the frame's deadline skipped its group (4.10: a Debug build on a 3-core runner, 2 workers and the caller for 4 busy groups); one document's `ps` lists none of the others' processes, and its `inspect` and `kill` of another's pid are refused; pids are unique |
| AR-3b | 1b | an isolated `any` document at threads 4 under `CVC_EXEC_FORCE_MOVE`: an AppRuntime async verb with a done channel, a stream opened, posted to from a producer thread and closed, and a request to the suite's local HTTP server, each call held at a gate (`AriadneFrames.AR03b_ProvidersPostToTheirGroup`; required in both TSan legs) | each call is tracked by the document's group, never group 0; the done message, the stream's `live` and `closed` and the producer's seq reach a resident in the group; group 0's ingress count does not move across the action (`ingress_posts_for_test`), and it holds no message; a worker pumped the group, and an `any` probe in the action ran off the frame thread |
| AR-4 | 1a | `pycvc.ExecPool(app, threads=8).run_all` of 64 fib programs; the same with a Python native; `fresh_env=True` (`test_ar04_run_all`) | correct results; with the native, at most one Python call at a time (a counter in the native); a fresh env per program. Scaling is recorded by the bench (`mt.groups`), not asserted |
| AR-5 | 3 | a document with `parallel_actions: 4`, 8 actions in one frame | actions ran on several threads; each completed in its drain; `doc_scope_` verbs ran on the host thread |
| AR-6 | 3 | a pycvc `Exec` program spawning 8 isolated children that report by message | correct; children on several threads; `Exec.run` wakes on their messages |
| AR-7 | 2 | `exec_coordinator` with a pool attached: 16 local jobs | each in a job group; results correct; control from a bus thread is TSan-clean |

As built in Stage 1b: the Ariadne cases are in `ariadne_runtime_test.cpp` (the section's own at its
end, under `// --- Stage 1b Ariadne (td 8.2) ---`; HZ-8's rework beside HZ-8), and PIN-1 is in
`state_lifetime_stress_test.cpp`. All of them and HZ-8 are required lines in every sanitizer leg
(`SAN_REQUIRED_S1B`), and so are the implementers' cases below in `ariadne_runtime_test` and
`state_exec_async_calls_test` (`ExecPool.AForceMovedCallEndsAtItsDeadline` is in Stage 1a's list).
Beside the rows above:

| ID | Situation | Result |
|---|---|---|
| SP-1 | a resident's chroot is unlinked (`state::removeChild`), then it spawns a child that writes (`SpawnContext.SP01_AChildOfADeletedChrootWritesTheDetachedNode`; S1.10, 4.15) | the write lands in the detached node the resident's context holds; no node is made at the old path |
| EL-1 | with `isolate_documents` on, documents breaking each rule of S1-Q6 (no `channels:` block, `lint: channels: warn`, a `global: true` channel, a writable hole leaving the prefix, a static `/` reference in a button's `on_click`, the empty prefix, oidc), and eligible ones (plain, a writable hole inside, a read-only hole leaving, a thread-affine prefix declared under and above, a `host_only` native in the env) (`AriadneFrames.EL01_IneligibleDocumentsStayInGroupZero`) | each ineligible one stays in group 0 with a reason, logged once over its frames (read through `system.log_output`; not checked in a log4cplus build); the `/` reason names the channel; each eligible one gets a group, `host` for the declarations and the native, else `any` |
| PIN-1 | modelled on Stage 0's `ContextRootPinsReleaseTheChrootFirst` (`StateLifetime.UriStateChainsReleaseTheLeafFirst`, `StateLifetime.StreamDescriptorPinsReleaseTheLeafFirst`) | a `destroyed()` slot on the leaf sees its ancestors alive |

The implementers' own cases, beside the code's existing tests:
- `ariadne_runtime_test.cpp`: `AriadneScheduler.DrainEndDoesNotPumpAgainAfterAFramePump`;
  `AriadneIsolatedDocs.ProvidersAreInstalledByTheirKind` (an `any` native on a worker, a
  `host_proxied` one on the frame thread and its constant bound, and a `host_only` one,
  registered while the document's tick resident runs, a `NameError` that `has?` does not see),
  `IdleAndWaitIdleCountTheDocumentsGroup`,
  `AnInitThatUsesOidcKeepsTheDocumentInGroupZero`, `TeardownReapsItsProcessesAndDestroysItsGroup`,
  `ALaterDocumentThatIsNotEligibleMovesToGroupZero`,
  `AThreadAffinePrefixAboveTheDocumentMakesItHost`, `AProgramSetAfterThePlacementIsScanned`,
  `AMovedDocumentsStreamPostsWhereItNowRuns`, `AnEligibilityThatFlipsKeepsOneGroupAtMost`,
  `IdleAndWaitIdleDecideNothing`; with the skeptic round's fixes,
  `IdleAndWaitIdleCountARetiredGroupsCalls` (after a move, `idle()` is false and `wait_idle()`
  waits while the retired group's kernel runs),
  `AMoveLeavesANestedDocumentsStreamAlone` (a group-0 document nested under a moving one keeps its
  stream on group 0, and after the mover's teardown the stream still names group 0),
  `AStreamTheDocumentOwnsFollowsItsMoveWhateverItsScope` (a proxy the document owns, with its
  publisher's scope, follows the move), `AMoveDoesNotWaitForTheOldGroupsCalls` (the move returns
  while the old group's 1.5 s kernel runs, the action is told it moved, and the retired group goes
  at a drain once the kernel has ended), `AHostVerbRegisteredWhileItRunsBindsAtTheNextLoad` (a verb
  registered while a resident runs does not move the document: a parked action finishes, the verb
  is a `NameError` in the `any` group, and the next load binds it, the group then `host`) and
  `AHostVerbRegisteredBeforeTheFirstFrameIsBound` (renamed from
  `AHostVerbRegisteredAfterLoadIsBound`: before the first frame the registration still moves the
  document to a `host` group and binds the verb); and `AriadneIsolatedDocsDeathTest`'s
  `ATeardownInsideAPumpIsRefused` (the victim an isolated document whose group the same
  `exec_pump_frame` holds; the death prints "refused"),
  `ATeardownFromAnotherThreadWhileAPumpHoldsItsGroupIsRefused` and
  `TearingDownAnUnpumpedDocumentInsideAPumpIsAllowed` (a host group's native tearing down a
  never-run group-0 document inside the pump exits cleanly);
  `AriadneInit.ItsCyclesAreItsOwnAndFreedWhenItEnds`;
  `AriadneNetIntrinsics.RequestsGoThroughTheContextsHost`;
  `AriadneNetL4.AStartedRequestKeepsTheHostItStartedOn`;
  `AriadneStreamIntrinsics.AStreamPostsToItsOpenersGroup` and
  `AStreamOfAnInlineHostPostsToGroupZero`; `AriadneHazards.AResidentAReloadStopsIsReaped`;
  `AriadneNetHost.ACancelledRequestStopsItsTransfer` (an `http-get` to a server that stalls 20 s:
  its host's shutdown, and a kill, end the kernel in about a second; the ceilings are 8 s, for a
  loaded machine).
- `ariadne_host_test.cpp`: `AriadneHostVerbs.AnInitLanesAsyncVerbPostsItsDoneChannelToGroupZero`
  and `ANameTwoTablesBindResolvesInTheirOrder`.
- `stream_test.cpp`: `Stream.EventsGoToTheSchedulerItWasOpenedWith` (from another thread too).
- `state_exec_async_calls_test.cpp`: `AsyncCalls.AStartedCallsFinisherPostsOnlyWhileItsHostIsUp`
  (the guard checked directly; the race itself is not deterministic) and
  `AsyncCalls.AShutdownsCancelHookMayWaitForAThreadInTheFinisher` (a hook that waits up to 5 s for
  a thread finishing the call through the finisher ends in time, and nothing is posted).
- `net_http_client_test.cpp`: `NetCurlBackend.ACancelledRequestStopsItsTransfer` (against a server
  that never answers, a request whose predicate turns true ends about a second later with
  "cvc::net: cancelled", not at its 30 s timeout; one cancelled
  before it goes is never made).
- `net_ws_client_test.cpp`: `NetWsClient.TwoThreadsSendingOnOneConnectionSendWholeMessages`.
- `exec_pool_test.cpp`: `ExecPool.AForceMovedCallEndsAtItsDeadline`, which times the call in its
  own caller's thread: it must end within half the busy worker's nap, at its deadline, not when
  the worker comes. It counts a try only when the worker napped on for over half its nap after the
  call began (a caller thread started late on a loaded machine is tried again, five times at most).
- cvcGL's `cvcgl_unattached_node_thread`:
  `test_another_threads_matrix_during_the_nodes_own_pose_write_is_not_its_echo` (4.8).

### 8.7 Sanitizers (TS)

| ID | Situation | Result |
|---|---|---|
| TS-1 | Linux clang `-fsanitize=thread` (Stage 1a: the sanitizer job's `clang-tsan` leg, S1-Q9): state_exec tests, GR (`state_exec_groups_test`), PL, MS, IS and ST at threads 4, the Ariadne runtime tests at threads 4, the TSan pycvc set (`test_pycvc_exec_pool.py` among it), `mt.groups` | clean, every required line found |
| TS-2 | ASan and UBSan jobs on the same set (and gcc's TSan leg); MSVC `/fsanitize=address` on GR and IS, run locally and recorded in the PR | clean |
| TS-3 | Windows debug build with `CVC_EXEC_VERIFY` on GR, MS and IS (MSVC has no TSan), run locally and recorded in the PR | no ownership assertion (in Stage 1, the pump owner's check) |

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
| DT-1 | deterministic mode, the same program set twice, threads 4 (`Groups.DT01_DeterministicRunsRepeat`) | identical per-group step counts at every pump (narrowed from step traces: no trace hook exists), results, random streams, and order of host posts and callbacks; cross-group message order is Stage 2's |
| DT-2 | `threads = 0` against today's scheduler (`Groups.DT02_ThreadsZeroEqualsBareSchedulers`: GR-1's programs through the pool against bare `async_scheduler`s pumped with the same budgets; the whole suite at threads 0 is today's default and already runs) | identical results, step counts and statuses; the bench's pinned `steps_total` covers the scheduler families |
| DT-3 | deterministic mode against threaded mode on MS-12 | the same outcomes |

### 8.10 Slices (SL, Stage S)

SL-1 to SL-9 are as `threaded-impl/S2.md` section 4 restates them, and SL-10 covers the settings'
precedence. The gtest cases are `Slices.<ID>_<Name>` in `state_exec_async_test.cpp`, and their
oracles use steps-only budgets (but SL-9, since `slice_seconds` applies only under a time budget)
and `limits.async_steps_per_cpu_ms` 0. SL-5 is restated: on one thread, only three things can act
during a process's run (its own step, a tree slot inside it, or a posted control), so it covers a
self or slot kill and a posted kill. SL-7 to SL-9 are new.

| ID | Situation | Result |
|---|---|---|
| SL-1 | the async suite at `slice_steps` 1; a three-process program | passes; per-process results, step counts and completion order equal master's (narrowed from a per-step trace: no trace hook exists, and none is added to the default path) |
| SL-2 | `max_steps` 1000 on an endless loop, `slice_steps` 256 | killed at exactly 1000 steps |
| SL-3 | `max_time` 0.05 s on an endless loop | killed, at most 16 steps over (step counts) |
| SL-4 | a resident with `max_step_time` 0.05 s whose native loops | killed by its per-step deadline |
| SL-5 | a self kill and a tree-slot kill during a run; a kill posted from another thread | the first two end the run at once; the posted kill lands within one step |
| SL-6 | a watched node changes during a run | the handler fires at the watcher's next run |
| SL-7 | under the priority policy, a process readied at a better level during a run; a better-level sleeper that falls due | the first runs within one step; the sleeper within 16 steps |
| SL-8 | a script-engine activation whose slices charge several steps each | its charged steps end a run at `slice_steps` |
| SL-9 | data steps in a run, under a run loop with a time budget | the run ends at `slice_seconds` (step counts at each run's end) |
| SL-10 | `set_slice`, either half alone (`set_slice_steps`, `set_slice_seconds`), or `exec_group::options` given one or both; the tree's `state_exec.slice_steps` and `slice_seconds`; and `CVC_EXEC_SLICE_STEPS`, set in combination | `set_slice` wins over the tree, the tree over `CVC_EXEC_SLICE_STEPS`, and that over 1; a half set alone, or a group given one option, leaves the tree's other in force; a tree write applies when the next run loop starts; a tree value that is not a whole number above 0 (steps), or not a number read strictly (seconds: `"2ms"`), leaves the one in force; tree seconds of 0 or less mean 1 ms; a tree key deleted or emptied reverts to the default (`CVC_EXEC_SLICE_STEPS`, else 1; 1 ms) |

### 8.11 Locks and priority across groups (LG, Stages 1-3)

The locks track's L4 runs these with Stages 1-3. Stage 1a took S1-Q1's part instead: GL-1 and
GL-2 (8.2), the lock attach and the teardown's order; the LG cases stay L4's.

| ID | Situation | Result |
|---|---|---|
| LG-1 | the GR-1 oracle with `with-lock` sections inside each group | every group equal to its solo run |
| LG-2 | a cross-group grant racing a kill and a timeout, fork-join and continuous (`CVC_EXEC_FUZZ`); a lock post racing the target group's destruction, 10^4 times (ASan, TSan); a grant to a waiter of a group in teardown, between its port's close and its kills | one outcome each (released by the kill, or adopted by the timeout); nothing lost or doubled; no use after free, no race: each post's `wake(g)` completes before the close or the post is dropped; that waiter's kill releases the grant, and the next waiter is granted |
| LG-3 | `CVC_EXEC_FORCE_MOVE` on lock holders; a move of a lock waiter | results unchanged; the waiter's move refused |
| LG-4 | `set-priority` on a pid in another group, same owner and another owner | posted and applied at the next step boundary; `#f` |
| LG-5 | group weights under a priority policy: 2 busy groups at best priorities 0 and 10 at threads 1; 4 at 0, 0, 10, 10 on 2 workers | `extra.share_ratio` within 20% of the CFS ratio for each configuration (about 9.3 between a 0 and a 10 group) |
| LG-6 | deterministic mode, threads 4, LG-1's programs twice | identical grant order |
| LG-7 | a lock cycle across two groups, in deterministic mode | the same `DeadlockError` victim every run |

### 8.12 Moving programs between hosts (MG, Stage M)

| ID | Situation | Result |
|---|---|---|
| MG-1 | a running program moved between two hosts on loopback | the same result as an unmoved run; inbox, watches, signal handlers and pending signals kept |
| MG-2 | locks LM-1 (refusal, `release_locks`, the source resumed after a failure) through a real move | as LM-1; the moved process raises `LockError` ("lost") |
| MG-3 | with lock leases (locks L7): rebind acknowledged, refused, timed out | resumes; `LockError` ("lost"); the same |
| MG-4 | a lock waiter or a process with a call in flight | refused |
| MG-5 | locks PR-11 (priority and floor carried, the target's floor applied) through a real move; a boosted holder | as PR-11; the boost not carried |
| MG-6 | the target fails before acknowledging; the source fails after; the first with `release_locks` | one copy left, the source's resumed; the target's copy runs; reported either way; the resumed source raises `LockError` ("lost") before its next step |

## 9. Risks and decisions

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
10. **Locks across groups.** Every lock operation takes the app's one lock-manager mutex, which can
    contend at high thread counts with short sections; `lock.cross_thread` and `mt.groups.locks`
    measure it, and sharding the manager by subtree is the fallback. A hot key also pays a post
    and a wake per handoff; placement keeps hot keys in one group, as for chatty channels.
11. **Migration.** Pids and leases across hosts, and a host failing mid-move, are Stage M's hard
    parts. A process must never run on two hosts at once.

### 9.2 Decisions

The user adopted this design on 2026-10-26, with the proposed answer to each question below. Four
questions offered no proposal of their own (Q7, Q10, Q13, and Q11's width). Their answers are the
plan the rest of this document already follows, marked "default".

1. **Q1. The model: groups first (E).** Option A is kept as Stage 4, built only on evidence
   (section 7). Parallelism inside one program comes from Stage 3's `spawn-isolated` and parallel
   actions.
2. **Q2. The built-in exception classes are made read-only** in Stage 0, not copied per group. This
   is a behaviour change (section 5, item 1): `set-attr` and `del-attr` on one raise `TypeError`.
   Per-group copies stay the fallback if `mt.groups` raise/catch shows refcount contention (B25).
3. **Q3. Callables are refused at any depth** in a value crossing groups: closures, natives,
   generators, environments and futures (D's structured clone). Closures are not copied with their
   environments (4.3).
4. **Q4. Spawn.**
   - A spawned child inherits the parent's intrinsics, chroot, owner and channel policy, through a
     host callback (4.15).
   - A pycvc child also inherits the `Exec`'s Python natives.
   - Only `spawn-isolated` places a child in another group. Plain `spawn` stays in the caller's
     group; D's `auto_spawn` is not taken.
5. **Q5. Cross-group control.** `kill`, `pause`, `ps` and `inspect` from a program reach other
   groups only when they have the caller's owner (4.5). Documents stop seeing and killing each
   other's processes.
6. **Q6. The router.** App-global channels are homed at the pool's group 0. A receive on a channel
   homed in another group is allowed, through the subscription protocol (4.6).
7. **Q7. Ariadne (default).**
   - Stage 1 builds its Ariadne half as planned.
   - Stage 1's audit answers the questions for the owners: whether any shipped host (tvision
     desktop, conference room, holon participants) runs several Runtimes in one process, and which
     providers and subscribers must stay on the host thread. A doubtful one is `host_only` or not
     thread-safe.
   - The deferred-notification sink stays rejected (3.4). It is reconsidered only if the audit
     leaves most documents on the host thread.
8. **Q8. Defaults.** `state_exec.threads` defaults to 0, so nothing moves to workers until a host
   or the app tree opts in. `state_exec.cpu_set` defaults to `performance`: workers on P-cores
   only.
9. **Q9. The kernel pools are split** in Stage 2, into `background_cpu` and `background_io` (4.9).
   This reverses settled question 4 of
   [`docs/STATE_EXEC_EXCEPTIONS.md`](STATE_EXEC_EXCEPTIONS.md) from Stage 2 on, and that document
   says so.
10. **Q10. Slices (default).** `slice_steps` stays 1 by default after Stage S, so the round robin
    stays exact. Hosts opt in to longer runs.
11. **Q11. Pids.** One pid source per pool (4.5). Pids stay `int` (default): widening them to
    64 bits is a separate change, outside this design, and the wrap after 2^31 processes stays a
    known limit.
12. **Q12. `ExecPool` reuses its pooled `Exec`s**, so top-level definitions persist per pooled
    `Exec`. `fresh_env=True` gives each program a fresh env (4.12).
13. **Q13. `exec_coordinator` (default).** Its local jobs move to job groups through `attach_pool`
    in Stage 2. The sync-scheduler path stays for the CLI (4.12).
14. **Q14. The names** are as proposed:
    - `state_exec.threads`, `state_exec.cpu_set`, `state_exec.deterministic`,
      `state_exec.groups.ingress_cap`;
    - `ariadne.exec.isolate_documents`, `ariadne.exec.parallel_actions`;
    - `spawn-isolated`, `pycvc.ExecPool`.

The locks and priority design ([`docs/STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md)) and Stage M were
added on 2026-10-27 at the user's request. Their open questions are recorded in that document
(section 14.2), and three of them touch decisions here:
- **Bringing B27 and B28 forward, for approval.** The locks track lands B28 whole in its L1
  (before Stage 0) and B27's single-scheduler part in its L1b (before Stage 1), each with section
  5's compatibility scan, so forked and spawned programs can lock on one machine before any
  group exists. Sections 4.15, 4.18, 5 and 7 record it.
- **A reading of Q5, for approval.** Q5 stops documents seeing and controlling each other's
  processes. The locks design lets lock inheritance cross owners (a waiter may raise another
  owner's holder, only while the holder blocks it and only to a priority the waiter already has),
  lets `lock-query` count other owners' holders without their pids, and shows other owners' pids
  in a `DeadlockError` cycle as `nil`. None gives control of another owner's process or names its
  pid; without inheritance a document could be starved through a shared `/` key by another
  document's low-priority holder.
- **New names, for approval as Q14's were:** the forms `with-lock`, `locks-held`, `lock-query`,
  `set-priority`, `get-priority`, `state-incr`, `state-cas`, `state-swap`; the classes
  `LockError`, `DeadlockError`, `PermissionError`; the settings `state_exec.policy`,
  `state_exec.locks.threading`, `state_exec.groups.priority_weights`, `state_exec.process_class`,
  `state_exec.worker_class` and `ariadne.exec.lock_timeout`; and the pycvc and C++ host API:
  `Exec(policy=, idle_timeout=)`, `Exec.set_priority`, `Exec.get_priority`, `Exec.lock_query`,
  `pycvc.set_thread_class`, `execute_options.priority_floor`, `resource_policy::priority_floor`,
  `pool_options::worker_class`, `exec_group::options::thread_class`,
  `lock_manager::options::inheritance` (test-only) and `intrinsics_context::lock_wait_allowed`.

Locks are not channels, so Q6 does not home them at group 0: all lock state is in the app's lock
manager, under one mutex (4.19). Stage M depends on Stage 0 and the locks track's L1-L3 only, and
is scheduled last.

Stage 0's plan added six decisions, which the user approved on 2026-11-01:
- **S0-Q1.** Stage 0's TSan runs extend C0's gcc 13 `sanitizers` job; the Linux clang job stays
  Stage 1's TS-1 (7).
- **S0-Q2.** `exec_coordinator::migrate` is asynchronous: one posted entry pauses, serializes,
  sends and kills, and `migrate()` returns "requested" (4.12).
- **S0-Q3.** The coordinator's other bus-thread paths (`handle_submit`'s `execute`, the migrate
  ingest, `admin_pause/resume/kill`) are posted too (4.12).
- **S0-Q4.** `state-data-set` refuses user-class instances and the internal objects (`with-lock`'s
  form, a lock wait, an async call) at any depth; built-in exception instances are stored with
  their class dict shared (4.3, 5 item 2). Rebuilding an instance by class name is a roadmap item.
- **S0-Q5.** pycvc's raw scheduler mutators are the pump thread's; only `stop()` is made safe
  from any thread (4.12).
- **S0-Q6.** No foreign-future `RuntimeError` in Stage 0; it comes with Stage 1's per-group host
  (4.9).

Stage 1's plan added twelve decisions, which the user approved as recommended on 2026-11-07
(`threaded-impl/S1.md` section 1 keeps the alternatives weighed):
- **S1-Q1. Locks in groups before L4.** Every group's scheduler attaches to `app::lock_manager()`
  in the manager's own mode, as the app scheduler does, so `with-lock` in a group excludes the same
  keys app-wide. Workers pump only when the app's manager is shared; with a single one the pool
  pumps every group on the caller and logs why. `app::exec_pool(opts)` asking for threads above 0
  before the manager exists makes it shared, for that app only. A group's teardown closes its lock
  port first. GL-1 and GL-2 prove it; L4 keeps the LG cases and the rest (4.13, 4.19).
- **S1-Q2. The init lane's host, its charging, and the inline exemption.** A threaded future whose
  owner is not the awaiting scheduler raises `RuntimeError` "await: the future belongs to another
  scheduler"; an inline future is a value and exempt. The init lane gets an inline host of its own,
  which charges its async verbs to init's process, and its done channels still post to group 0
  (built in Stage 1b). The check closes the pycvc cross-`Exec` race (4.9).
- **S1-Q3. The registry a thread leaves when it ends.** Every group owns its registry and collects
  and frees it after its scheduler and envs. A thread's own registry is collected at its exit only
  if the thread opted in (`collect_cycles_at_thread_exit()`): the pool's workers and test threads
  do, Python threads and the main thread never (4.7).
- **S1-Q4. Enforcing pycvc's pump-thread rule (S0-Q5).** Each scheduler gets a pump owner, held by
  every driver while it drives: `Exec::run` until its quiesce (as built, until it returns, R1-mid),
  a group's pump, the Python pump wrappers per call. The rule is "no owner, or me": a raw call, and
  `Exec`'s pump-thread calls, raise `RuntimeError` "scheduler: another thread is pumping it" only
  while another thread holds the owner. Debug builds and `CVC_EXEC_VERIFY=1` assert it in the
  schedulers' public mutators (4.1, 4.12).
- **S1-Q5. `ExecPool`'s API and the app's pool.** `pycvc.ExecPool(app, threads=None, size=None,
  prefix="exec_pool", fresh_env=False, idle_timeout=60.0)` on the app's one pool, sized by whoever
  builds it first; pooled `Exec` i chrooted at `<prefix>.<i>`; `run_all`, `register_fn`,
  `register_async_fn`, `threads` and `close()` the whole API (4.12).
- **S1-Q6. Which documents may leave the host thread** (built in Stage 1b, 4.12): with
  `ariadne.exec.isolate_documents` on, only a document with a prefix, a strict `channels:` block
  with no global channel or static `/` reference, no oidc and no writable `link:` hole leaving its
  prefix; `affinity::host` when its env holds a `host_only` native or its host declared a
  thread-affine subscriber under its prefix. As built, a declared prefix at or above the
  document's counts too, since a write emits `childChanged` on every ancestor (R1b-mid S-M2), and
  the placement is decided again after each change it reads.
- **S1-Q7. The kind of each native and subscriber** (built in Stage 1b, 4.8, 4.9). Kinds are
  registered per provider, so AppRuntime's provider splits in four and net's in two; a doubtful one
  is `host_only` or not thread-safe; S1.md's table records each. As built, AppRuntime's four kinds
  are seven providers, which keep the one provider's binding order.
- **S1-Q8. Ariadne's teardown reaps the action processes** (built in Stage 1b, 4.17), its own
  included (S0.11's question; HZ-8), and a resident a reload stops is reaped then.
- **S1-Q9. The clang TSan job (TS-1)** is a third leg of the sanitizer job, `clang-tsan`, with the
  runner's clang; TS-2's MSVC ASan and TS-3's `CVC_EXEC_VERIFY` runs are local and recorded in the
  PR (8.7).
- **S1-Q10. Stage 0's pre-existing bugs Stage 1 does not need** (a restored process's unshared
  scopes, `election_start_time_`, the coordinator ignoring the addressed node): three small PRs
  from master, after R1-mid, each with a failing-first test.
- **S1-Q11. Two callers of the one pool at once.** Each `pump_all` call has its own work list,
  host-call queue and accounting; concurrent calls share the workers; a group belongs to one call
  at a time (asserted); each caller services only its own queue (4.4, 4.9).
- **S1-Q12. Two PRs, in order:** Stage 1a (groups, the pool, pycvc), then Stage 1b (Ariadne's
  half), each with its own reviews and A/B (7).

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
