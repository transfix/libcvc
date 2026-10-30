# state_exec: locks on the state tree, and process priority changes

Status: design, 2026-10-27. L0 (the atomic natives, section 4.4) and L1 (`with-lock` on one
scheduler) are built; L1b, L2 and L3 follow, and L4 onward are design only.
Tracked in the `state-exec-perf` workstream,
beside the threaded design ([`docs/STATE_EXEC_THREADED.md`](STATE_EXEC_THREADED.md)), which this
document extends. The exact lock protocol is
[`docs/STATE_EXEC_LOCKS_PROTOCOL.md`](STATE_EXEC_LOCKS_PROTOCOL.md).

The user asked for mutexes in the DSL, "maybe to lock sections of the state tree that are being
worked on". The design must work on one thread and on several, later survive a program moving to
another host (a server or a client), and let programs change process priority for the schedulers.
Four proposals were written and two judges scored them; the named, scoped locks proposal won.
Two adversarial review rounds then showed its protocol was not converging, and its core was
simplified (3.3). This document is that design.

**Sources.**

- `sched:` marks a reference to `perf/sched` at `c9b55253`. It rewrites `async_scheduler.cpp`,
  `scheduler.cpp`, `process.*` and adds `process_table.h`. Its sync `run()` is `run_steps()`, and
  its `idle_wait(max_steps, steps, budget)` charges an idle wait to both of `run()`'s budgets
  (`sched:scheduler.cpp:482-561`).
- The implementation builds on the three single-thread optimization branches together (section
  12): `perf/sched`; `perf/eval` (special forms by a switch in `special_forms.h`, a call's function
  looked up once at init, recycled call scopes and frame vectors, `step_init`'s arguments a span
  into the live form); and `perf/async` (persistent background-pool workers).
- Every other file:line reference is to `a7b966f0` (master plus the benchmark suite). On
  `perf/sched` most of the files they name are unchanged; in `pycvc_exec.cpp`, `pycvc.i` and
  `process.h` the cited lines move down by up to four, and `Exec::run` (`pycvc_exec.cpp:579-648`)
  waits for work instead of sleeping. On `perf/eval`, `stackless_evaluator.cpp` has moved; each
  reference to it also names the function, and the function is what counts.
- `td` is [`docs/STATE_EXEC_THREADED.md`](STATE_EXEC_THREADED.md); `Bn` are its blockers (td 2.3)
  and `Qn` its decisions (td 9.2). `protocol` is
  [`docs/STATE_EXEC_LOCKS_PROTOCOL.md`](STATE_EXEC_LOCKS_PROTOCOL.md). Measured costs are those of
  `docs/STATE_EXEC_BENCH.md` and td 2.5.

## 1. Goal and non-goals

### 1.1 Goal

1. **Critical sections over the tree.** A program can hold a section of the state tree, a path and
   everything below it, across several steps, a park, an async call or a frame, so that no other
   cooperating program sees it half written or changes it meanwhile.
2. **Shared and exclusive.** Readers share a section; a writer excludes everyone.
3. **Nothing leaks.** A lock is released on every way a process ends, and composes with
   `try`/`finally`, exceptions, `return`, `break`, pause, signals, watches, deterministic mode and
   slices.
4. **Cheap when uncontended.** On one thread an uncontended section costs no extra step and a small
   fraction of one scheduled step (8.8).
5. **One design from one thread to many**: the sync scheduler and `async_scheduler`, programs a
   host submits and their forked and spawned children, several pycvc `Exec`s on several threads,
   and execution groups (td Stages 1-3, Stage 4 in reserve), without redesign.
6. **Priority changes from the DSL**, with Unix rules, effective under a priority policy, which
   hosts opt into (`state_exec.policy`, 7.4), and priority inheritance through locks.
7. **OS thread and process priority** for the hosts' worker threads, as a host-only knob.
8. **Migration later.** The single-machine phases leave what a cross-host stage needs (grant
   generations, base and floor priorities). The cross-host stage comes last.
9. **Atomic single-node updates** (`state-incr`, `state-cas`, `state-swap`), so the most common
   read-modify-write needs no lock.

### 1.2 Non-goals

- **Mandatory locking.** Locks are advisory. `state-get`, `state-set` and the other tree intrinsics
  never consult the locks, so existing programs keep their behaviour, step counts and cost. Hosts,
  Python and replication writers are never checked. Opt-in enforcement is a later phase (L5).
- **Lock objects as values.** A lock is named by a path string. There is no mutex `data_object`,
  so nothing has to be shared across groups or snapshotted (td 4.3, invariant G).
- **An explicit `lock`/`unlock` pair.** Only the scoped `with-lock` exists, so a lock cannot
  outlive its frame.
- **Transactions and STM**: they change the tree's hottest write path and the unwinders for gains
  the atomics and locks cover. **Condition variables**: a lock guards the invariant and a channel
  does the waking (4.3). **A lock-server process**: a message round trip costs 23-95 us (td 2.5),
  about 1000 times an uncontended acquire.
- **Exclusion beyond one manager before L3, or beyond one host before L7.** A scheduler given no
  manager makes a private one. Before L3 each pycvc `Exec`, and Ariadne's init lane, is such a
  scheduler (8.4).

## 2. Today

1. **No DSL primitive locks.** The intrinsics (`intrinsics.cpp:1062-1125`) and special forms
   (`stackless_evaluator.cpp:36-41`) have no mutex, compare-and-set or transaction. The tree has
   one `boost::mutex` per node (`state.h:855`). `state::value(v)` compares and assigns under two
   separate holds of it (`state.cpp:564-616`), so even a single-node read-modify-write from two
   threads can lose an update.
2. **A step is atomic inside one scheduler.** One process steps at a time and a step is never
   preempted (`sched:async_scheduler.cpp:226-228`). A lock matters only across steps, and across
   schedulers and threads.
3. **The only resumable park is the `await_call` frame.** A native applied through `apply_native`
   (`stackless_evaluator.cpp:1250-1272`) may call `request_await`, which needs `can_await()`
   (451-455), true only inside `apply_native`: a special form cannot park by itself.
   `step_await_call` (1543-1590) completes with the awaitable's `outcome()` (1583).
4. **Kill never unwinds.** `kill`, SIGKILL, `kill_owner` and every budget end the process without
   running `finally` ([`docs/STATE_EXEC_EXCEPTIONS.md`](STATE_EXEC_EXCEPTIONS.md) 2.5). A lock must
   be released by the end functions `terminate_process`, `kill_process`, `fail_process`
   (`sched:async_scheduler.cpp:1059, 1069, 1077`; `sched:scheduler.cpp:798, 808, 815`), each of
   which starts with `procs_.set_status(...)` and ends with `release_if_idle`.
5. **Channel delivery cannot carry a grant.** `deliver_to_receivers` wakes every waiter of a
   channel (`sched:async_scheduler.cpp:576-603`), and the receiver buckets are not FIFO
   (`sched:process.cpp:424-459`). A `#` channel is reachable from the DSL: `resolve_channel_key`
   returns any key holding `#` verbatim (`intrinsics.cpp:1139-1140`) and `msg-send` delivers to
   it (1011). A lock waiter parked on a `#lock...` channel could be woken by a forged message.
6. **Priority is a field, not a mechanism, in the shipped hosts.** The policy (`round_robin`,
   `priority`, `priority_rr`, `scheduler_base.h:41-45`) is fixed at construction, and Ariadne and
   pycvc build `round_robin` (`app.cpp:1027`, `pycvc_exec.cpp:377`). `set_priority` exists on both
   schedulers but not on `scheduler_base`, so no intrinsic reaches it; priorities are documented
   as -20..19 (`process.h:57`) and never checked.
7. **The run queue keys on `priority`** under a priority policy (`sched:process.cpp:202`), re-levelled
   by `procs_.update` (`sched:async_scheduler.cpp:1215-1222`).
8. **Who is related to whom.** `fork` sets `parent_pid` and copies `uid` but not `owner`, and forks
   `ctx->pid`, not the running process (td B28). `spawn` copies only `uid` (`intrinsics.cpp:420-431`)
   and records no parent. Spawned children get the bare base env until td's spawn context (B27),
   whose single-scheduler part this design brings forward as phase L1b.
9. **Migration.** `exec_coordinator::migrate` pauses, ships metadata, and kills the original
   (`exec_coordinator.cpp:712-753`); the target starts a `nil` process. `encode_process` keeps
   priority (`state_value_codec.cpp:1114`) but not the inbox, watches or pending signals.

## 3. The options and the decision

### 3.1 The four proposals

Named scoped: `with-lock` on tree paths with an app-wide manager and its own wait kind. Lease
manager: leases homed like channels, with fencing tokens. Tree hierarchical: path locks with
anchors, explicit and scoped forms, condition waits. Optimistic: per-node versions,
`state-transact`, atomics and a minimal mutex.

| Proposal | Judge 1 | Judge 2 | Main weakness found |
|---|---|---|---|
| **Named scoped** | **67** | **54** | a lock-free entry word; domains registered on first use; parking from a special form |
| Lease manager | 61 | 53 | a `#lock` wait channel reachable from `msg-send`; bare-pid owners; every cross-group acquire a round trip |
| Tree hierarchical | 58 | 48.5 | a forgeable `#lock` channel; `with-lock` in `all_special_forms()`; the largest scope |
| Optimistic | 51 | 43.5 | bare-pid owners; an ABI change in `state.h`; a new jump kind; no subtree lock |

### 3.2 Scores

Two judges scored each proposal from 1 to 10 on six criteria (correctness, one-thread performance,
groups fit, migration, simplicity, usability). Judge 1 weighted correctness x3 and one-thread
performance x2 (max 90); judge 2 weighted groups fit, migration and simplicity x1.5 (max 75). The
totals are in the table above.

### 3.3 The decision

**Named scoped locks, with grafts.** Both judges chose it. It is the only proposal whose lock wait
a DSL `msg-send` cannot wake, and its app-wide manager gives a spawned DSL thread in any group an
uncontended acquire with no round trip, the user's stated goal. Lease-manager's homed tables
would make each such acquire a post round trip from td Stage 2.

Grafts from the others: a threading mode fixed at construction, `single` or `shared` (lease, tree;
8.3); parking through a native applied by `apply_native`, and tickets that own the outcome (lease;
8.5); `migrate(release_locks)` and late leases with fencing (lease; 10); the atomics and later
semaphores (optimistic; 4.4, L6); the handler rule, broadening, `state_exec.policy`,
`get-priority` as a dict and `check()` invariants (tree; 5, 7, 13); and cross-thread
`set_priority` through `post_control` (all; 7.4).

Not taken: tree's process-carried lock binding for spawned children (L1b's spawn context makes it
unnecessary), tree's runtime `set_policy` (no caller), lease's homed tables (a round trip per
acquire), tree's condition waits and watch deferral, and optimistic's transactions (1.2).

**Then a simplified core (2026-10-27).** The chosen proposal's protocol went through two
adversarial review rounds against the code. Round 1 confirmed 55 findings (21 major) and round 2
confirmed 45 (21 major). Most were holes in the protocol's own machinery: intention marks kept by
waiters, multi-key requests taken key by key with a partial chain, continuations carrying a
grantee's chain from a release to a lower level and across homes, pass-on of grants orphaned in
flight, 64 shard mutexes with a lock order and lock-then-validate, document domains whose cutting
raced with outer locks, and a lazy cross-home deadlock check that could not see intention marks.
Each fix opened another. The lead engineer decided to simplify the core until it can be shown
correct:

1. **All or nothing.** A request (one key, or a canonical list, in one mode) is granted every key
   at once, or waits holding nothing new. Multi-key requests cannot deadlock each other.
2. **One manager, one mutex.** One manager per app (per scheduler before L3), all its state under
   one mutex in shared mode and none in single mode. No shards, no lock order, no domains.
3. **Holds on a path trie.** Each grant is a hold of `(home, pid)`. Two requests conflict when a key
   of one equals, or is an ancestor or descendant of, a key of the other and the modes are not
   both shared; subtree counts find a conflict in O(depth) plus the overlapping holds.
4. **One FIFO.** A waiter is granted when no other process holds a conflicting hold and no earlier
   waiter of another process conflicts with it, less the conversion rule (5.4). Non-conflicting
   requests pass freely.
5. **Tickets hold the outcome.** `queued`, `granted`, `cancelled` or `failed`, changed only by the
   manager; a wake is a hint. A process's end releases everything it holds, a grant not yet
   resumed included, so nothing is ever passed on.
6. **Exact deadlock detection at block time**, across every home and thread, since all lock state
   is under one mutex. No lazy check, no timer.
7. **Inheritance over the same wait-for graph**, recomputed under the mutex whenever an edge or a
   base changes.

[`docs/STATE_EXEC_LOCKS_PROTOCOL.md`](STATE_EXEC_LOCKS_PROTOCOL.md) specifies it as pseudo-code,
with its invariants and their proofs, for a model checker to implement literally (LT-5).

**What it costs.**
- **One mutex for the app in shared mode.** An uncontended acquire and release still take one
  mutex each, so the costs of 8.8 stand. But threads locking disjoint keys now share that mutex
  and its cache lines. An operation holds it for about 30-150 ns, so sections of a few
  microseconds see no difference, while sections of a few hundred nanoseconds on 8 threads can
  serialize. `lock.cross_thread` measures it (11.2; gate: disjoint at t = 4 at least 0.9x the
  no-lock run). If it fails, the documented fallback is **sharding by subtree**: a manager per
  top-level subtree or document prefix, requests spanning subtrees taking their mutexes in a fixed
  order, behind the same API, after its own review and model check (14.2, question 9).
- **No document domains.** An unchrooted X on `docs` now excludes every document's locks below it,
  as the tree means, instead of being refused. Documents share only their common ancestors'
  counters.
- **A multi-key request waits for all its keys at once**, so under churn on one key it cannot
  progress piecemeal; it holds nothing others wait for meanwhile.
- **The conversion rule is pairwise.** A chain of three can raise `DeadlockError` where a rule over
  whole paths would grant (LD-10): still an exact cycle, and a reachability rule would make grants
  depend on distant parts of the graph.

### 3.4 Corrections to the proposals

Each was found by a judge and checked against the code.

1. **Forgeable lock waits** (tree, optimistic, lease): a `#lock...` channel is reachable by
   `msg-send` (2, item 5). Here a lock wait has no channel, and the outcome is read from the
   manager's ticket, never from a delivered value (8.5).
2. **Bare-pid owners** (optimistic, lease): each pycvc `Exec`'s pids start at 1
   (`pycvc_exec.cpp:373-378`). Here a hold is owned by `(lock_home, pid)`, and reentrancy is
   decided from the process's own mirror.
3. **Children and owners**: `spawn` records no parent and `fork` does not copy `owner`; the field
   is `parent_pid`, set only by fork. Here 7.2 also accepts descendants by `parent_pid`, L2 makes
   `spawn` record it, L1 makes fork copy `owner` (td B28), and L1b gives spawned children their
   parent's owner.
4. **The special-form gate** (tree): a name in `all_special_forms()` is refused by a restricted
   evaluator (`stackless_evaluator.cpp:917-918`). Here `with-lock` is not in that set (4.2).
5. **Parking from a special form** (named scoped, tree): `can_await()` needs `apply_native`'s
   await slot (451-455, 1253). Here the contended path applies a lock-wait native.
6. **`priority` as the effective value** (lease): `ps`, `inspect`, the codec and migration read
   `process::priority`, so it stays the base and the boost is a separate field. **The end hook**
   (lease) goes right after `set_status`. **Domains on first use** (named scoped): none exist now.
   **`Exec.set_priority`** is new (7.4); `pycvc.i:3712` wraps only the scheduler classes'.

## 4. The DSL API

### 4.1 Forms and intrinsics

| Form | Returns | Raises |
|---|---|---|
| `(with-lock KEY option... body...)` | the body's value; with `:if-available` and the lock not taken, the `:else` value (default `nil`) | `ValueError` (bad key or option), `PermissionError` (`/` key not allowed), `TimeoutError`, `DeadlockError`, `LockError` |
| `(locks-held)` | a list of dicts `{key mode count generation}`, the caller's holds, outermost first | - |
| `(lock-query [prefix])` | a dict `{held: [{key mode pids count generation abandoned}], waiting: [{key mode pid}]}` for keys under the caller's scope; pids of another owner (Q5) or another scheduler (whose pids repeat the caller's) are left out, counts are not | `ValueError` |
| `(set-priority p)`, `(set-priority pid p)` | `#t` when applied, or posted to another group; `#f` for a pid the caller may not see (7.2): unknown, finished, another owner's, or neither of the caller's `uid` nor a descendant | `ValueError` (not an int in -20..19), `PermissionError` |
| `(get-priority [pid])` | a dict `{base effective floor policy}`; `nil` for a pid the caller may not see, as for `set-priority` | - |
| `(state-incr path [delta])` | the new number | `ValueError` (not a number), the write's errors |
| `(state-cas path expected new)` | `#t` if the value equalled `expected` and is now `new`, else `#f` | `TypeError` (a compound `expected` or `new`), the write's errors |
| `(state-swap path new)` | the old value; `nil` when the node is absent or its value is `""` | `TypeError` (a compound `new`), the write's errors |

**`with-lock` options** follow `KEY`, as keyword symbols read unevaluated (`:` is a symbol
character, `parser.cpp:402`). The body starts at the first form that is not an option.

| Option | Meaning |
|---|---|
| `:exclusive` / `:shared` | the mode; exclusive by default |
| `:timeout EXPR` | seconds, evaluated once; one deadline for the request, set when it is first tried (5.5). `0` tries once and raises `TimeoutError` if the lock is not grantable now |
| `:if-available` | never wait: if the lock is not grantable now, skip the body and return `:else` |
| `:else EXPR` | with `:if-available` only; evaluated only when the lock was not taken |
| `:as SYM` | bind `SYM` in the body's scope to the grant `{key mode generation abandoned}`, `key` being the request's keys shown as `locks-held` shows a hold's (below: canonical order, a string for one key and a list for several). A covered (reentrant) request binds `{key: its keys, mode: its mode, generation: the covering hold's, abandoned: #f}`: an abandoned report belongs to the section whose grant took it. With `:if-available`, when the lock was not taken, `SYM` is bound to `nil` in `:else` |

An unknown option, an option given twice, or `:else` without `:if-available` raises `ValueError`.

**`KEY`** is evaluated: a string, or a list of strings for a multi-key request.
- **Segments.** A key is split on `.`, segments trimmed and empty ones dropped, as the tree does
  (`state.cpp:1058-1100`). `""`, `"."` and any key containing `#` raise `ValueError`.
- **Scope.** A key is relative to the caller's chroot (`root_path + "." + key`), with no `channels`
  segment (unlike `resolve_channel_key`, `intrinsics.cpp:1138-1146`): two documents' `"counter"`
  are different locks.
- **App-global.** A leading `/` means relative to the app root (`"/jobs"` is the absolute key
  `jobs`). It is allowed exactly where a `/` channel key is (`enforce_channel_policy`,
  `intrinsics.cpp:934-958`), and otherwise raises `PermissionError`.
- **Links are not followed.** A key names a path, not a node.
- **Multi-key.** A list is canonicalized: keys sorted segment by segment, and a key equal to or
  under another in the list dropped. The request is granted all at once, every key in the
  request's mode, or waits holding nothing new (5.2). Every key is checked (upgrade, the handler
  rule) before the manager sees any.
- **No node needed.** A key need not exist in the tree, so `(with-lock "/locks.printer" ...)` is a
  plain named mutex.
- **Generation.** Every X grant takes the next value of one manager-wide counter, so a grant's
  generation is greater than every earlier X grant's, on any key: a fencing token for each key it
  covers. An S grant reports the counter's value at its grant.
- **Shown keys.** Every key a program is shown (`:as`, `locks-held`, `lock-query`, the `key` of
  `LockError` and `TimeoutError`, `DeadlockError`'s `cycle`, `inspect`) is written as the program
  would write it: relative to its chroot when it lies strictly under it, else with a leading `/`.
  A request's or hold's keys are shown in canonical order, as a string for one key and a list for
  several: `(list "accounts.bob" "accounts.alice")` shows as `("accounts.alice" "accounts.bob")`.

**New built-in classes**, appended to the hierarchy at `exceptions.cpp:58-64`:
- `LockError(RuntimeError)`, with fields `key`, `mode` and `reason`. The reasons are `"upgrade"`
  (5.2), `"cannot wait here"`, `"generator"`, `"no process"` (5.5), `"fork"` (5.8), `"handler"`
  (5.7), `"woken without a grant"` (8.5) and, from L7, `"lost"` (10);
- `DeadlockError(LockError)`, with `cycle`, a list of `{pid key}`;
- `PermissionError(Exception)`.

That makes 16 built-in class dicts; td Stage 0 makes them all read-only (Q2). `inspect` gains
`effective_priority`, `priority_floor`, `locks_held` (a count) and `waiting_on` (`"lock <key>"`,
`"msg <channel>"`, `"sleep"`, `"frame"`, or `nil`). `ps` is unchanged: its `priority` stays the
base.

### 4.2 Why `with-lock` is a "soft" special form

A body must not run before its lock is held, so `with-lock` cannot be a function. It is also not
listed in `all_special_forms()`:

- **Recognition.** `step_init` looks the head up before evaluating a call's arguments
  (`stackless_evaluator.cpp:1196`; on `perf/eval`, the call path when `special_form_of` returns
  none). When it is the `lock_ops` object `register_intrinsics` binds to `with-lock`, the list is
  the form. A user macro, `defun` or `set` named `with-lock` wins. The cost is one variant-index
  test, only for a `data_object` head.
- **No intrinsics, no locks.** Where `register_intrinsics` did not run (bare evaluators; spawned
  children before L1b), `with-lock` is a `NameError`. Restricted evaluators (Ariadne's reactive
  lane) do not bind it, and the gate at `stackless_evaluator.cpp:917-918` and Ariadne's lints
  (`ariadne.cpp:782`) are unaffected.
- **The recursive evaluators** (`evaluator.cpp`, `async_evaluator.cpp`, `pycvc.evaluator`) have no
  scheduler and evaluate every argument before testing the head; L1 makes the `lock_ops` head raise
  `TypeError("with-lock needs a scheduler")` first, so the body never runs (LW-11).

### 4.3 Examples

**A critical section over a subtree:**

```lisp
(defun move-stock (from to n)
  (with-lock "inventory"                       ; X on <chroot>.inventory and everything below
    (let ((a (int (state-get (str "inventory." from))))
          (b (int (state-get (str "inventory." to)))))
      (if (< a n) (raise (ValueError "not enough stock")))
      (state-set (str "inventory." from) (- a n))
      (await (audit-async from to n))          ; parks: the lock stays held
      (state-set (str "inventory." to) (+ b n)))))
```

The `raise` unwinds out of the body and releases the lock. A kill mid-way releases it through the
end hook, and the next exclusive grant on `inventory` says `abandoned: #t` (5.6).

**Readers, a writer, and a finer lock below:**

```lisp
(with-lock "inventory" :shared (report (state-json "inventory")))   ; many readers at once
(with-lock "inventory.apples" (state-set "inventory.apples" 7))     ; waits only for X/S on inventory
```

**Producer and consumer.** The producer appends under `(with-lock "q" ...)` and then `msg-send`s
`"q.ready"`; the consumer takes an item under the same lock, and `msg-recv`s `"q.ready"` outside it
when the queue is empty. The lock guards the multi-node invariant, the channel does the waking,
and no process waits for a message while it holds the lock.

**Try-lock, and a timed multi-key section:**

```lisp
(with-lock "cache" :if-available :else (state-get "cache.value")   ; someone else refreshes
  (state-set "cache.value" (expensive-fetch))
  (state-get "cache.value"))

(try
  (with-lock (list "accounts.bob" "accounts.alice") :timeout 0.5 :as g   ; both at once
    (if (get-attr g "abandoned") (repair-accounts))
    (transfer "alice" "bob" 10))
  (catch (TimeoutError e) (msg-send "retry" "transfer"))
  (catch (DeadlockError e) (log-cycle (get-attr e "cycle"))))
```

**No lock needed:**

```lisp
(state-incr "stats.hits")                          ; one step, atomic against every writer
(if (state-cas "jobs.j7.owner" nil (self-pid))     ; claim a job once
    (begin (run-job "j7")
           (state-cas "jobs.j7.owner" (self-pid) nil)))   ; release: claimable again
```

**Which tool.** One node: `state-incr`, `state-cas`, `state-swap`. Several nodes that must change
together, or a section that spans a park: `with-lock`. A queue between programs: channels.

### 4.4 The atomic natives

- **Resolution.** Each resolves its path relative to the chroot and through a transparent link
  exactly as `state-get` reads it (`read_through_link`, `intrinsics.cpp:120-127`). A transparent
  link that is not writable raises the read-only error (`state::value` writes through only a
  writable link, `state.cpp:568-588`). An absent node is created when the native writes.
- **Atomicity.** The compare and the assignment happen under one hold of the node's mutex, in new
  non-virtual `cvc::state` members (`compareAndSetValue`, `exchangeValue`, `updateValue` taking a
  function), so `state.h` stays compatible. Each takes `value()`'s shape (`state.cpp:564-616`):
  resolve a writable link, check `_readOnly`, compare, assign, stamp and notify under one hold,
  then fire `valueChanged` and `emitChildChanged` with no lock held.
- **Values.** `nil` as `expected` matches a node that is absent or whose value is `""`; `nil` as
  `new` stores `""`, so a claim released with `(state-cas p me nil)` can be claimed again.
  `_initialized` is not consulted (creating a child sets it on the parent,
  `state.cpp:1083-1088`). Other values are compared after `coerce_state_string`
  (`intrinsics.cpp:140-159`); a compound `expected` or `new` raises `TypeError`.
- **`state-incr`.** An absent or empty node counts as 0. An int plus an int delta (default 1) stays
  an int; otherwise the result is a float in the shortest round-trip form (`std::to_chars`), the
  form `state-set` stores and `to_string` prints, so 1000000 plus 0.5 stores `1000000.5`. A non-number raises `ValueError` and leaves the node unchanged.
- **Signals.** A write that changes the value fires watches and `valueChanged` once. A
  compare-and-set or swap whose new value equals the old one succeeds without writing or firing.
- **Cost.** One native call each, like `state-set`, with one mutex hold instead of two.

## 5. Semantics

This section states the rules for a program's author; the protocol document states them exactly.

### 5.1 Ownership

- **The owner is the process**: `ctx->sched->current_pid()`, never `ctx->pid`, which hosts leave at
  a placeholder (`intrinsics.cpp:511-521`). Not the thread, the group or a value.
- **Holds live in the manager**, each a granted request owned by `(lock_home, pid)` with an id
  unique in the manager. A `lock_home` is one per scheduler; pids repeat across schedulers (each
  pycvc `Exec` starts at 1), the pair does not.
- **The home keeps a mirror** of each process's holds in `process::locks`, in acquisition order,
  with reentrant counts. Reentrancy is decided from it, never by comparing pids.

### 5.2 Modes, coverage, reentrancy

- **Conflict.** `:shared` (S) and `:exclusive` (X). Two requests, or a request and a hold, conflict
  when a key of one equals, or is an ancestor or descendant of, a key of the other (on segment
  boundaries) and the modes are not both S. X on `a.b` blocks S or X on `a` and anything on
  `a.b.c`, and leaves `a.c` and `a.bc` free. There are no intention modes: the manager's trie
  keeps subtree counts (8.2).
- **All or nothing.** A list of keys is one request, granted every key at once or waiting with
  nothing new held, so two multi-key requests cannot deadlock each other.
- **Coverage.** A held X covers S and X requests on its keys and below; a held S covers S requests.
  A request all of whose keys are covered adds 1 to the covering hold's count and touches no
  manager state: this is reentrancy.
- **Upgrade.** An X request on a key covered only by a held S raises `LockError` (`"upgrade"`) at
  once: two upgraders would deadlock. It is the only refusal among a process's own requests.
- **Broadening.** X on `a` while holding `a.b`, or S on `a` while holding X on `a.b`, is allowed: a
  process's own holds never conflict with its requests. Two processes broadening against each
  other form a cycle, and the one that closes it gets `DeadlockError` (LD-3).

### 5.3 Overlapping keys, and no domains

Any key may be locked by anyone who can name it (4.1). An unchrooted X on `docs` waits for, then
excludes, every document's locks below it, which is what the tree means (LK-34). Hosts register
nothing. The round-2 design cut documents into domains and refused outer locks over them, to keep
their shard mutexes apart; with one manager mutex nothing is saved that way, and domains produced
holes in both review rounds (3.3). A lock on `docs.A.x` updates the counts of `docs` and `docs.A`.

### 5.4 Fairness

- **One FIFO per manager**, in arrival order. A request is granted when no other process holds a
  conflicting hold and no earlier waiter of another process conflicts with it: strict FIFO among
  conflicting requests, so lockers of `a.b` cannot starve a queued writer of `a`, while a request
  that conflicts with nothing passes freely (LK-32).
- **The conversion rule.** An earlier waiter that conflicts with a hold the requester already has
  does not block it. That waiter cannot be granted before the hold is released, and sections are
  scoped, so the new section ends first: granting it delays the waiter not at all, while queueing
  behind it would deadlock. So a nested section on a sibling key, or broadening, goes ahead of a
  writer of the parent (LK-20, LK-27, LK-28, LD-7). The rule is pairwise: a waiter blocked only
  through a third process still blocks, and if that closes a cycle the requester gets
  `DeadlockError` (LD-10).
- **Grant all, in order.** After every release, timeout and end the manager re-examines the
  waiters overlapping what was freed, in arrival order, and grants all it can: behind a released
  X, `[S, S, X, S]` grants the two first S together (LK-21). The grant is made on the waiter's
  behalf, so the lock is never free in between and a releaser cannot barge back in (LK-5).
- **`:if-available` and `:timeout 0`** take the lock exactly when the fast acquire grants it now,
  every waiter counted as earlier, and never queue or barge (LK-6, LK-20).
- **Priority never reorders the queue**; inheritance (6.2) handles the other direction.

### 5.5 Waiting and timeouts

- **A contended request parks its process** (8.5) through `park_lock` (8.4): `waiting`, with the
  lock fields set and no `recv_path`, so nothing a program sends reaches it.
- **`:timeout S`** is one deadline, computed when the request is first tried; a request waits once,
  so it is the wait's `lock_deadline`, in the timer heap (`sched:process.cpp:153-161, 499-527`). On
  expiry of a queued request the manager cancels it and re-examines the waiters overlapping it,
  so a writer timing out lets in the readers behind it (LK-22, LK-25); then `TimeoutError` is
  raised. A request granted on another thread before its deadline fired adopts the grant (LX-5).
- **No implicit timeout**, except a host's `lock_default_timeout`. A parked process is not charged
  time (`process.h:100`), so Ariadne sets 1.0 s for actions (`ariadne.exec.lock_timeout`). Spawned
  children inherit the default and `lock_wait_allowed` (LK-31, LW-9).
- **Contexts that must never wait.** Ariadne's init scripts run synchronously on the host thread,
  which also steps every resident (`run_scoped_program`, `ariadne.cpp:377-420`), so a holder could
  never release while an init waited. `intrinsics_context` gains `bool lock_wait_allowed = true`,
  and `run_scoped_program` sets it to false: every busy request there behaves as `:timeout 0`,
  whatever its `:timeout` (`TimeoutError` at once, or `:else` with `:if-available`), and the host
  thread never waits for a lock (LW-9). Its default timeout is 0 as well. A child the init script
  spawns is stepped by the same `sched.run` on the host thread, and the spawn context copies both
  values to it (L1b), so it never waits either.
- **Where a process cannot park** (`can_await()` false: a nested evaluator, under
  `park_forbidden_scope`, a native called inline where `can_await()` is false), an uncontended
  `with-lock` works, a contended one raises `LockError` (`"cannot wait here"`) without blocking
  the thread (B21), and `:if-available` takes `:else`. A wait inside a DSL function that a native
  called (`collections.map`, `sort`, `apply`, a constructor) runs as frames of the calling process
  at the outermost step depth, so it parks as an async call does there
  ([`docs/STATE_EXEC_EXCEPTIONS.md`](STATE_EXEC_EXCEPTIONS.md) 3.3) and the native's job completes
  after the grant (LW-1b). In Ariadne's reactive lane `with-lock` is not bound (4.2): `NameError`.
- **Generators.** `with-lock` in a closure generator's body raises `LockError` (`"generator"`),
  even uncontended: a generator runs its own `evaluator_state` inside a native
  (`generator.h:26-75`), can `yield` with its frames alive and be dropped without unwinding, so a
  lock there could outlive every frame. A `with-lock` around `(next g)` is unaffected.
- **With no stepping process** (`current_pid() < 0`) it raises `LockError` (`"no process"`).

### 5.6 Release on every exit and every unwind

| Way out | What releases | Abandoned |
|---|---|---|
| The body's value, `return`, `break` | the `lock_body` frame, as it completes or as `unwind_jump` passes it (`stackless_evaluator.cpp:332-364`) | no |
| An exception, caught outside or uncaught | `unwind_exception` passing the frame (282-325), before any outer `finally` | no |
| An exception from the lock frame's own step | `fail_step` (757-768) calls `release_lock_frame` before popping it (8.5) | no |
| A handler's exception raised into the program, also between a grant and its body | the same unwind; a `lock_enter` frame releases its granted, unstarted hold | no |
| `kill`, SIGKILL, `kill_owner`, every budget, a step deadline, an internal step error | `process_ended(proc, true)` in `kill_process` (`sched:async_scheduler.cpp:1069`, `sched:scheduler.cpp:808`) right after `set_status`: one manager call that releases every hold and cancels the ticket | X holds whose body started |
| `fail_process` (1077, 815) | the same call | the same |
| `terminate_process` with holds left (1059, 798) | the same call with `false`, before `on_complete`; counted in `locks.leaked_at_exit` | no |
| The scheduler destroyed | `~lock_home`: closes its port (under the port's mutex), then one call that ends every process of the home and drops its cache references | the same |

- **Abandoned** means an X holder ended without unwinding after its body started, so its section
  may be half written. Each of its keys is flagged, and the next X grant on exactly that key
  reports it once (`:as` shows `abandoned: #t`); a lock on an ancestor or descendant does not, so a
  program that repairs must lock at the granularity it writes. It is counted (`locks.abandoned`)
  and logged once; nothing is sticky.
- **A report is never lost.** A grant that takes a flag and is released before its body started
  puts it back (LX-3).
- **A grant before its body** already holds the lock in the manager, so the process's end releases
  it, wherever its wake is: nothing is passed on. A wake that arrives after the end, after a
  timeout adopted the grant, or after a newer request is ignored (LX-10, LX-11). `reap` and
  `on_complete` are never involved.

### 5.7 Pause, signals and watches, budgets, determinism, slices

- **Pause.** A paused holder keeps its locks. A waiter cannot be paused (`pause` needs ready or
  running, `sched:async_scheduler.cpp:1093-1106`).
- **The handler rule.** A waiter's handlers run after it is woken. While `in_signal_handler` or
  `in_watch_handler` is set (`sched:process.h:147-148`), a request covered by, or conflicting
  with, a hold of the interrupted program (a granted hold not yet entered included) raises
  `LockError` (`"handler"`): a handler cannot read a half-written section by reentrancy, nor
  deadlock on its own program. A handler's own holds are reentrant inside it and released when its
  frames unwind.
- **Budgets.** An uncontended `with-lock` adds no step (8.8); a wait is not charged; a budget kill
  releases, abandoned.
- **Deterministic mode.** Grants follow arrival order, which is step order inside a group; across
  groups, wakes are posts applied in post order (td 4.4). The deadlock victim is the requester
  that closes the cycle.
- **Slices (td Stage S).** A contended acquire parks, which ends the run; an uncontended one does
  not. With the default `slice_steps = 1` nothing changes.

### 5.8 Fork, spawn, snapshots

- **Fork while holding a lock** is refused: the intrinsic forks `current_pid()` (td B28), the
  scheduler's `fork` returns -1 when it holds locks (`sched:async_scheduler.cpp:1156`,
  `sched:scheduler.cpp:1000`), and the intrinsic raises `LockError` (`"fork"`). The child would
  otherwise copy `lock_body` frames it does not hold. A child keeps the parent's `owner` (L1, td
  B28's other half), so `kill_owner` reaps it and Q5's owner tests treat it as its parent's (LW-6).
- **Spawn.** A child holds nothing. From L1b the spawn context gives it the lock intrinsics against
  the parent's chroot; from L2 it records `parent_pid`, and its floor follows 7.2.
- **Snapshots** (`snapshot`, `encode_process`) of a holder are refused, naming the keys. A waiter
  holding nothing may be snapshotted: its wait encodes as `nil`, and the wait child is a plain call
  `(%lock-wait KEYS MODE TIMEOUT)` whose `results` hold the arguments. `step_await_call`
  (`stackless_evaluator.cpp:1555-1577`; on `perf/eval`, 1778-1820) turns the restored await frame (a
  nil handle) back to its apply phase and, in the same step, re-applies `%lock-wait`, which it looks
  up by name (8.5), so the copy queues afresh at the tail. `TIMEOUT` is the request's resolved
  timeout in seconds (its `:timeout`, else the context's `lock_default_timeout`, else `nil`), and
  the codec rewrites it to the time left (`p.lock_deadline - now`, at least 0) when it encodes a
  waiter, so a restored waiter waits only for what remained (LW-7), even in an `Exec` whose context
  has another default. Nothing of the wait lives outside that plain call between steps, so a
  restored waiter snapshotted again before its next step queues afresh the same way, with the
  `TIMEOUT` it was restored with, since it has no deadline yet (LW-7).
- **The `lock_ops` object is treated like a native**: the codec encodes it as a payload-free marker
  decoding as `nil`, `rebind_natives` rebinds `with-lock` and `%lock-wait` from the restoring
  `natives` (a restored wait resolves `%lock-wait` by name, so it must name the restoring
  scheduler's lock natives), and a restored lock frame re-resolves its `lock_ops` slot from the
  restoring environment.
- **A restored `lock_body` frame that recorded a hold** cannot come from any snapshot before L7, so
  restore refuses such a stack (LW-12). L7, where `migrate(release_locks)` makes one possible,
  replaces the refusal with `locks_lost`: `LockError` (`"lost"`) before the first step (10.1).

### 5.9 Advisory, and opt-in enforcement later

Locks are advisory, as in Web Locks and Chubby: writers that do not ask (another program, Python,
the host, replication) are not stopped. Opt-in guarded subtrees (L5) can later make the DSL write
intrinsics require a covering X, at one relaxed load per write when none is declared.

## 6. Deadlocks and priority inversion

### 6.1 Deadlock

1. **Prevention where it is free.** Requests are all or nothing (5.2), so multi-key requests never
   deadlock each other, and the conversion rule (5.4) removes the cycles strict FIFO alone would
   build through a requester's own holds.
2. **Exact detection at block time.**
   - **Edges.** A waiter waits for every other process holding a conflicting hold, and for the
     process of every earlier waiter whose request conflicts with it, less those the conversion
     rule exempts. These are exactly the grant rule's blockers, so a waiter with no edge is
     grantable, and the manager never leaves one queued (protocol I6).
   - **Check.** Before queueing a request the manager follows the edges from its would-be blockers.
     If they lead back to the requester, it gets `DeadlockError` and never queues. The check runs
     under the manager mutex over every home's holds and waiters, so it is exact across `Exec`s,
     groups and threads. No hop cap, no lazy check, no timer.
   - **Once is enough.** Edges appear only when a request queues: a grant adds none (protocol
     Lemma A), and a waiter's earlier waiters and own holds do not change while it waits. So the
     graph stays acyclic, and no later release, timeout or grant can close a cycle.
   - **Result.** `DeadlockError`, catchable, with `cycle`, a list of `{pid key}` from the
     requester round the cycle; the pid of another owner's process (Q5), or of a process on
     another scheduler, whose pids repeat the requester's (3.4 item 2), shows as `nil` (LT-11;
     from L4 a pid in the same pool shows, since pids are pool-wide). The victim is the
     requester that closes the cycle: deterministic for a deterministic arrival order.
   - **Cost.** A depth-first search over the queued requests, on the slow path only
     (`lock.deadlock`, 11.2).
3. **Timeouts** remain the backstop for waits that are not lock deadlocks: a holder parked for ever
   on a message.

On `perf/sched` a sync `run()` left with only lock waiters behaves as with only message waiters
(`sched:scheduler.cpp:515-561`): a run bounded by `max_steps` returns at once when no timer is set
or its remaining steps would run out first (536-540), and otherwise it waits for a timer or the
end of its time budget. From L3 a lock post also ends that wait (8.7). So a bounded `run(N)`
whose waiter waits for a live process on another home, with no timer set, returns at once, and
the next `run()` applies the grant (LT-2).

### 6.2 Priority inversion: inheritance

- **Why.** Under `priority` and `priority_rr` the pick is strict, so a low-priority holder behind a
  medium-priority loop starves a high-priority waiter for ever (the Pathfinder case). Ceilings need
  static knowledge of which programs use which keys.
- **Rule.** Under the priority policies only, a process inherits the best base priority among the
  processes it blocks, directly or through other waiters, up to 8 edges of the wait-for graph,
  and runs at `min(base, inherited)`. A holder is boosted by a waiter queued behind another
  waiter it blocks (PI-9); a granted process not yet resumed is boosted like any holder. The value
  is taken over paths of up to 8 edges, not as the best effective priority of the direct waiters:
  a round-robin waiter lends nothing yet passes on what it inherits, and the cap bounds paths, so
  neither the manager nor `check()` may compute it by recursion or fixpoint over direct waiters
  (protocol 3.3).
- **Fields.** `process::priority` stays the base; `inherited_priority` is new, and the run queue
  keys on `effective_priority()` from L2, re-levelled by `procs_.update`. One helper,
  `process_table::level_key(p)` (`by_priority_ ? p.effective_priority() : 0`), defines the key
  for `update` (`sched:process.cpp:202`) and `check` (552) alike, so the invariant check agrees
  with a boosted level (8.7).
- **Exact at every step.** The manager recomputes inherited priorities under its mutex whenever an
  edge or a base changes: a request queues, is granted or cancelled, a hold is released, a process
  ends, or `set-priority` runs. Every value always equals the rule over the current graph
  (protocol I8), so the order of events never decides a boost, and a boost whose source leaves
  is withdrawn at once, down the whole chain (PI-3, PI-8, PI-11).
- **Applying it.** A change for a process of the calling home is applied at once; for another
  home it is a `relevel` post that carries no value, and the home reads the current value when it
  applies it, so a late or duplicated post changes nothing (PI-7).
- **`set-priority` on a boosted holder** changes its base; it runs at the boost while it blocks a
  better waiter.
- **Under `round_robin`** a process lends nothing and its scheduler applies nothing. The manager
  does no inheritance work while no attached home has a priority policy (when the last such home
  detaches it resets every inherited value to none, posting nothing; when the first attaches it
  recomputes them all), and sends no `relevel` to a round-robin home or to a detached one, so a
  round-robin process's unchecked spawn priority (7.2) cannot boost a holder elsewhere (PI-10). A
  test-only switch, `lock_manager::options::inheritance`, turns inheritance off for the
  `lock.inherit` bench (11.2).
- **Phases.** In L1 every home attaches with no priority policy (protocol 4.4), whatever its
  scheduler's, so no inherited value is ever set; L2 attaches each home with its scheduler's
  policy and keys the run queue on `effective_priority()` (12).
- **Inheritance is not user control,** so this design reads Q5 as not gating it (14.2): it raises a
  holder only to a priority a waiter already has, only while it blocks that waiter. It shows in
  `get-priority` (`effective`) and `inspect`, never in `ps`, snapshots or migration.

## 7. Process priority changes

### 7.1 The DSL

- `(set-priority p)` changes the caller; `(set-priority pid p)` another process.
  `(get-priority [pid])` returns `{base effective floor policy}`.
- **Range.** -20..19, lower is better. Anything else raises `ValueError`; nothing is clamped. This
  also bounds the levels a DSL change can create to 40.
- **Under `round_robin`** a change is stored and reported, returns `#t`, and does not affect the
  pick, as today. `policy` in `get-priority` makes that visible.

### 7.2 Permissions (Unix nice, plus Q5)

- **The floor.** Every process has `priority_floor`, the best priority it may ask for, with
  `floor <= priority` at submit. It is the submit priority unless the host sets
  `execute_options::priority_floor` or `resource_policy::priority_floor`. Fork copies the parent's
  floor; a spawned child's is the parent's floor, or its granted priority when better (possible
  only where `spawn` is unchecked, below).
- **Self.** Lowering (a higher number) is always allowed; raising, down to the floor. A process
  that demoted itself can come back, never above what its host gave it.
- **Another process.** It must be live, have the caller's `owner` (unowned hosts count `""` as one
  owner), and either have the caller's `uid` or descend from the caller by `parent_pid` (at most
  64 hops). Otherwise the call returns `#f`, as for an unknown pid (Q5's "looks unknown"), and
  `get-priority` returns `nil`. Forked and spawned children keep their parent's `owner` (L1,
  L1b), so a program reaches its own children and never another document's. Lowering it is always
  allowed; raising it, down to `max(target floor, caller floor)`.
- **A refusal raises `PermissionError`**, never a silent no-op. **Host calls are unchecked**.
- **`spawn`'s priority argument.** Under `priority` and `priority_rr`, a value better than the
  caller's floor is clamped to it, logged once and counted (`priority_clamped`). Under
  `round_robin` it is stored unchecked, as today; it affects no pick, its own or, through
  inheritance, another home's (6.2), and the policy cannot change later (7.3). An explicit host
  floor clamps under every policy.

### 7.3 How a change takes effect

- **One scheduler.** A change writes `priority` and calls `procs_.update(p)`, which re-levels on
  the Fenwick levels (`sched:process.cpp:192-265, 358-373`), about 100-300 ns. A running process
  re-levelling itself is safe; the new level applies from the next pick. If the process holds or
  waits for a lock, the change also updates its base in the manager, which recomputes inheritance
  (6.2).
- **Levels.** Nothing is reserved. A new level allocates on first use, at most 40 for DSL values;
  `regrow` drops empty levels. The pick's `first_nonempty` (`sched:process.cpp:282-287`) scans
  the levels in use since the last regrow; `prio.set`'s `levels=40` point watches that cost.
- **Sync parity.** Both schedulers implement the same overrides with one shared checker
  (`detail::check_priority_change`), and both run on `process_table`.
- **The policy is fixed when the scheduler is built**, from `state_exec.policy`, `Exec(policy=...)`
  or the constructor argument. No runtime `set_policy`: no shipped host has a caller.

### 7.4 Host APIs

- **C++.** `scheduler_base.h` gains, before the class, a struct and a forward declaration, and the
  class gains non-pure virtuals, appended. The two schedulers derive from it, and from td Stage 3
  so does the forwarder `group_scheduler_ref` (td 4.15), which overrides every one of them by
  forwarding to `exec_group::current()->sched()` (9.3), so a DSL call never reaches a default:

  ```cpp
  struct priority_info { int base, effective, floor; scheduling_policy policy; };
  namespace detail { class lock_home; }

  enum class priority_change { applied, unknown, refused };   // unknown: a pid the caller may
                                                              // not see (7.2)

  // in class scheduler_base:
  virtual bool set_priority(int pid, int priority) { return false; }    // host: unchecked
  virtual std::optional<priority_info> get_priority(int pid) const { return std::nullopt; }
  virtual priority_change set_priority_checked(int caller_pid, int pid, int priority) {
    return priority_change::unknown;
  }                                                                      // the DSL's (7.2)
  virtual std::optional<priority_info> get_priority_as(int caller_pid, int pid) const {
    return std::nullopt;
  }                                                                      // nullopt: may not see
  virtual bool set_priority_floor(int pid, int floor) { return false; }
  virtual scheduling_policy policy() const { return scheduling_policy::round_robin; }
  virtual detail::lock_home *locks() { return nullptr; }                 // null: no locks
  ```

  The existing `set_priority` and `policy()` of both schedulers become overrides with the same
  signatures; `set_priority` writes the base. The intrinsics hold only a `scheduler_base *`, and
  `process_info` has no `owner`, so the DSL's checks live behind the two `_as`/`_checked`
  virtuals: both schedulers implement them with `detail::check_priority_change`, which reads the
  caller's and the target's `owner`, `uid`, `parent_pid` and floor from the process objects.
  `set-priority` maps `unknown` to `#f` and `refused` to `PermissionError`; `get-priority` maps
  nullopt to `nil`. A caller's own pid is always visible to it.
- **Settings.** `state_exec.policy` (`round_robin` default, `priority`, `priority_rr`), read by
  `app::exec_scheduler()` (`app.cpp:1024-1038`) and by pycvc `Exec` when the scheduler is built
  with `policy=None`.
- **pycvc.** `Exec(app, policy=None, idle_timeout=60.0)`, `Exec.set_priority(pid, p)`,
  `Exec.get_priority(pid)`, `Exec.lock_query(prefix="")`, `execute_options.priority_floor`.
  - **Arguments.** pycvc's SWIG module enables no keyword arguments today, so `pycvc.i` adds
    `%feature("kwargs")` for the new `Exec` constructor and for `Exec::lock_query` (8.7). SWIG
    cannot use keyword arguments on an overloaded function, so the constructor is one
    declaration that replaces today's (`pycvc_exec.h:32`):

    ```cpp
    explicit Exec(const std::shared_ptr<cvc::app> &app, PyObject *policy = nullptr,
                  double idle_timeout = 60.0);
    ```

    `policy` is `None` (the `state_exec.policy` setting), a `scheduling_policy` IntEnum member,
    or its name as a string (`"round_robin"`, `"priority"`, `"priority_rr"`), parsed in
    `pycvc_exec.cpp`; anything else raises `ValueError` (PR-5). A `scheduling_policy` parameter
    would not do: pycvc's enum typemap accepts only the IntEnum (`pycvc.i:2437-2440`) and would
    reject a name with `TypeError`. With `kwargs`, SWIG wraps the defaults inside the one
    wrapper, as `%feature("compactdefaultargs")` does (`pycvc.i:4133-4134`), not as an overload
    per default.
  - **Threads.** Another Python thread may call these while `run()` pumps, so they never call into a
    pumped scheduler (the B17 race). `run()` records its thread id in `ExecImpl`, and a thread-local
    pointer to its `ExecImpl`, when it starts. A `run()` is *active* while a new flag,
    `ExecImpl::accepting_queries`, is set: `run()` sets it under the control mutex before its first
    step and clears it at the end (below). It is not the re-entrancy guard `ExecImpl::running`
    (`sched:pycvc_exec.cpp:391`), which keeps its own job. Under the `Exec`'s control mutex: with no
    `run()` active they act directly; called from the thread that is running this `Exec`'s `run()`
    (a registered Python function, coroutine or callback, which runs between steps on the scheduler
    thread) they also act directly; from any other thread they post a control or query entry
    (`post_control`), answered between steps, and the caller waits on a condition variable with the
    GIL released. From a thread that is running another `Exec`'s `run()`, a call that would wait
    raises `RuntimeError("Exec.run is active on another thread")` instead, so two `Exec`s whose
    registered functions call each other cannot wait for each other inside a step.
  - **The control mutex stands in for the pump token.** The `Exec`'s home is touched only by the
    thread of its active `run()` or, with none active, by the holder of its control mutex (9.1).
    Acting directly, that holder is the home's thread: it calls `set_base`, delivers the
    notifications and applies this home's relevels at once (protocol 4.14), as `run()` would between
    steps. Debug owner checks (8.3, 9.1) accept it.
  - **The GIL comes before the control mutex.** A thread may take the control mutex while it holds
    the GIL (`run()` at entry, for its clear and its `reap`, `RunGuard`, a direct action), but never
    takes the GIL while it holds the control mutex. A caller waiting for its entry's answer waits on
    the condition variable with the GIL released; once woken it copies the answer out and releases
    the control mutex, and only then re-takes the GIL to build the Python result. The scheduler
    thread answers entries between steps with C++ values and never calls into Python under the
    mutex. So a woken caller never holds the mutex while it waits for a GIL that `run()` holds while
    it waits for the mutex (PR-8).
  - **`lock_query` never waits.** It reads only the app's lock manager, which every `Exec` uses
    in shared mode (8.3), so it calls `query` directly from any thread, under the manager mutex,
    viewing as this `Exec`'s own home with owner `""`: only this `Exec`'s pids show (4.1).
  - **No query is left unanswered.** On every exit path (the program's result, a kill, an
    exception, the catch included), after its last step and any kill and before `wait_idle(host)`
    (`pycvc_exec.cpp`, both paths), `run()` takes the control mutex, clears `accepting_queries`,
    and in the same critical section drains the queue, answering every pending entry or failing
    it with `RuntimeError("Exec.run ended")`. A kernel or thread that calls during the quiesce
    that follows sees no active `run()` and acts directly, under the control mutex, which `run()`
    also takes for its `reap`, so no caller waits for `run()` while `run()` waits for the kernels
    (PR-8).
  - **A second `run()` still raises.** `running` stays set through the quiesce: `run()` tests and
    sets it under the control mutex when it starts (`sched:pycvc_exec.cpp:536-539`), and
    `RunGuard` clears it, under the same mutex, last (544-558), after `wait_idle`, the reap and
    the coroutine cleanup. So a `run()` of the same `Exec` from any thread during the quiesce, a
    kernel's included, raises today's `RuntimeError` at once: it never steps the scheduler under
    the first `run()`, nor waits in its own `wait_idle(host)` for itself (PR-8).
- **`exec_coordinator`.** `handle_control` (`exec_coordinator.cpp:1045-1071`) gains `set_priority`,
  applied through `post_control`. Remote submit and migration metadata already carry priority
  (556, 818) and gain `priority_floor` (PR-11).
- **`post_control`.** If td Stage 0 has not landed when L2 does, L2 adds the `post_control` entry
  kind to both schedulers for `set_priority` (and pycvc's queries) only, with Stage 0's
  signature; Stage 0 extends it to kill, pause, resume and signal.

### 7.5 Groups

- **Another group's process** (td Stage 2). `set-priority` on a pid in another group applies 7.2
  to the target's published info, which carries its `owner`, `uid`, `parent_pid` and floor (td
  4.5): the same owner, then the caller's `uid` or descent, walked over the pool's published
  `parent_pid`s (pids are pool-wide, at most 64 hops), then the floor. A pid that fails the first
  two returns `#f` (Q5), a refused raise raises `PermissionError`, and otherwise the call posts
  `control{set_priority, p, caller}` through `pool.post_control`, `caller` carrying the caller's
  pid, `owner`, `uid` and floor, and returns `#t`. The target re-applies the whole rule with
  `detail::check_priority_change`, against its authoritative process and the carried caller, the
  descent walked as before; a refusal there is counted and logged.
- **A group's place in the pool follows its priorities.** At the end of each pump a group publishes
  `best_runnable`, the best effective priority among its runnable processes, boosts included.
  Under a priority policy, Stage 2's ready queue advances the group's virtual time by
  `pump_time / weight(best_runnable)`, with CFS weights (1024 at 0, about 1.25x per step): a group
  holding a -10 process gets about 9x the pump share of a nice-0 group, and none starves. Weights
  matter only while runnable groups outnumber workers. `state_exec.groups.priority_weights`
  (default on) turns it off; it ships on only after `prio.groups_weighted` passes (11.2).
- **Fork-join (Stage 1)** keeps its LPT order, with ties broken by `best_runnable`.

### 7.6 OS thread and process priority (host-only)

- **API.** `inc/cvc/core/thread_class.h`: `enum class thread_class { background, utility, normal }`,
  `bool set_current_thread_class(thread_class)` and `bool set_process_class(thread_class)`. Never
  above normal, never `REALTIME`. No DSL intrinsic reaches them. Every result is logged.

  | Platform | `background` | `utility` | `normal` |
  |---|---|---|---|
  | Windows | `THREAD_MODE_BACKGROUND_BEGIN` | `THREAD_PRIORITY_BELOW_NORMAL` | `THREAD_MODE_BACKGROUND_END`, `THREAD_PRIORITY_NORMAL` |
  | Linux | `setpriority(PRIO_PROCESS, gettid(), 10)` | 5 | 0; raising back needs `CAP_SYS_NICE` or an `RLIMIT_NICE` that allows 0, else `EACCES` and false |
  | macOS | `QOS_CLASS_BACKGROUND` | `QOS_CLASS_UTILITY` | `QOS_CLASS_DEFAULT` |
  | Emscripten | false | false | false |

- **`set_process_class`.** Windows: `SetPriorityClass` or `PROCESS_MODE_BACKGROUND_BEGIN`. Linux:
  per thread, applied to the main thread at app start and to each worker as it is created. macOS:
  `background` is `PRIO_DARWIN_BG` (which also throttles I/O), `utility` returns false. On macOS
  `utility` and `background` steer threads to E-cores, against Q8; the docs say so.
- **Hosts.** `state_exec.process_class`, `pycvc.set_thread_class`; from td Stage 1
  `pool_options::worker_class` (`state_exec.worker_class`) and `exec_group::options::thread_class`,
  applied at pump start when it differs.
- **Linux workers are demoted one way**, since an unprivileged thread cannot raise its nice value
  back: a shared worker is demoted for a group only if raising back is known to work; otherwise
  per-group classes apply only to dedicated workers.

## 8. The C++ architecture

### 8.1 Files

| File | Content | Phase |
|---|---|---|
| `inc/cvc/state/state_exec/lock_manager.h`, `src/cvc/state/state_exec/lock_manager.cpp` | `lock_mode`, `lock_threading`, `lock_manager` (the trie, holds, tickets, the queue, per-process records, the grant rule, the deadlock check, inheritance, `query`, `stats`, `check()`), `lock_info`, `lock_stats` | L1 (shared mode L3) |
| `inc/cvc/state/state_exec/detail/lock_home.h`, `src/cvc/state/state_exec/lock_home.cpp` | `detail::lock_home`, one per scheduler: key cache, mirrors, the wait, delivery, the end hook, `lock_sched_ops`, the port, `last_progress`, `check_quiescent()` (test builds, 13) | L1 (port L3) |
| `src/cvc/state/state_exec/lock_intrinsics.cpp` | the `lock_ops` object behind `with-lock`, the hidden `%lock-wait`, `locks-held`, `lock-query`, `set-priority`, `get-priority` | L1, L2 |
| `inc/cvc/state/state_exec/detail/priority.h`, `.../priority.cpp`; `inc/cvc/core/thread_class.h`, `.../thread_class.cpp` | `check_priority_change`, the descendant walk, CFS weights; 7.6 | L2 |
| `src/cvc/tests/state_exec_locks_test.cpp`, `state_exec_locks_model_test.cpp`, `state_exec_priority_test.cpp`, `bindings/pycvc/test_pycvc_state_exec_locks.py` | section 13; the model check (LT-5) | L0-L3 |
| `tools/locks_model/` | the design-time model of the protocol, in Python (protocol 7): the manager, the homes and their actions, bounded exhaustive and scripted exploration, random runs, the mutation scenarios and the long-run script; it lands with the docs, before the code | L1 |
| `docs/STATE_EXEC_LOCKS.md`, [`docs/STATE_EXEC_LOCKS_PROTOCOL.md`](STATE_EXEC_LOCKS_PROTOCOL.md) | this document and the protocol; a chapter in [`STATE_EXEC_DEVELOPER_GUIDE.md`](STATE_EXEC_DEVELOPER_GUIDE.md); the three classes in [`STATE_EXEC_EXCEPTIONS.md`](STATE_EXEC_EXCEPTIONS.md) 2.3 | L1 |

### 8.2 Data structures

```cpp
namespace cvc::state_exec {
enum class lock_mode : std::uint8_t { shared, exclusive };
enum class lock_threading : std::uint8_t { single, shared };   // fixed at construction

class lock_manager {                        // app::lock_manager(), or a scheduler's private one
 public:
  struct options {
    lock_threading threading = lock_threading::single;   // shared exists from L3 (8.3)
    bool inheritance = true;                // test-only: lock.inherit's off points (6.2)
  };
  explicit lock_manager(options);
  std::vector<lock_info> query(std::string_view abs_prefix,          // protocol 4.13: pids shown
                               const detail::lock_home &viewer_home, // only of the viewer's home
                               std::string_view viewer_owner) const; // and owner
  lock_stats stats() const;                 // holds, waits, grants, deadlocks, abandoned, ...
  lock_threading threading() const;
  void check() const;                       // test builds: the manager-side invariants (protocol 6)
};
}  // namespace cvc::state_exec
```

- **Inside the manager** (protocol 2): a path trie whose nodes count, per mode, the holds on exactly
  that key and in its subtree, and list the holds and queued tickets on that key; the holds (owner,
  keys, mode, id, generation, body started, abandoned flags taken); the tickets (owner, keys,
  mode, sequence number, state, hold or error), queued in sequence order; and a record per process
  with a hold or a ticket (owner, base priority, holds, ticket, inherited priority). Ids, sequence
  numbers and generations are counters advanced under the mutex the operation already holds.
- **On the home's side**, `detail::process_locks`: the mirror (hold id, keys as trie nodes (8.4),
  mode, reentrant count, generation, abandoned, taken in a handler) and the ticket the process is
  parked on.

**Added to existing structs**, at their ends (source-compatible):
- `process` (`sched:process.h`): `std::shared_ptr<detail::process_locks> locks` (null until used),
  `bool lock_waiting`, `std::optional<clock::time_point> lock_deadline`,
  `int inherited_priority = INT_MAX`, `std::optional<int> priority_floor` (unset reads as
  `priority`), `bool lock_expired_pending` (8.7), `int effective_priority() const`; L7 adds
  `bool locks_lost`.
- `execute_options` (`scheduler_base.h:48-67`): `priority_floor`, `int parent_pid = -1`.
  `process_info` (71-114): `effective_priority`, `priority_floor`, `locks_held`, `waiting_on`.
  `resource_policy`: `priority_floor`. `intrinsics_context` (`intrinsics.h:139-159`):
  `std::optional<double> lock_default_timeout` and `bool lock_wait_allowed = true` (5.5), plain
  members so SWIG wraps them; from L1b, `spawn_context`.
- `evaluator_state::in_generator`; `eval_phase` (`stackless_evaluator.h:36-63`): `lock_enter`,
  `lock_body`, appended. The `lock_ops` payload holds its intrinsics context and a
  `std::shared_ptr<void> keep`, which pycvc fills so the context outlives Python's reference.

### 8.3 Single and shared mode

- **`single`.** No mutex, no atomics. Debug builds take an owner-thread token on every call and
  assert it is free, so a pycvc `Exec` may `run()` from different Python threads in turn. From L3,
  several homes may attach when one thread runs them all (Ariadne's init lane, groups pumped
  inline at threads 0, Emscripten); they still exchange wakes by post (9.5).
- **`shared`** (from L3). One `std::mutex` (an SRWLOCK on MSVC, a futex on glibc) guards the whole
  manager: about 20 ns for an uncontended lock and unlock.
- **Which mode.** A scheduler's private manager is `single`. The app manager's mode is chosen at the
  first `app::lock_manager()` call: `shared` if threads are compiled in and either
  `state_exec.threads > 0` or pycvc is loaded, otherwise `single`; `state_exec.locks.threading`
  (`auto`, `single`, `shared`) overrides it. Attaching a home that may be pumped on another thread
  to a single-mode manager is refused with an error naming the setting, in every build.
- **Lock order.** The manager mutex is innermost apart from the tree's node mutexes, and nothing
  else is locked under it. It is never held while calling into a scheduler, a port, a process or
  a tree node: an operation collects its wakes and relevels and the caller delivers them after the
  unlock (8.6). DSL locks are outermost, tree locks innermost.
- **Later, only on data:** sharding by subtree (3.3), with the same API and semantics.

### 8.4 The per-scheduler `lock_home`

- **Key cache.** Open addressing, 1024 entries, CLOCK eviction. An entry maps one key, keyed by
  `(scope id, key as written)`, to that key's trie node, and holds one reference on the node
  (`M.cache_refs`, protocol 4.15); the scope id is the chroot string interned once per intrinsics
  context, so a hit hashes the relative key only. A multi-key request probes and references each
  key that its canonical form keeps separately, one entry per key. A miss or an eviction passes
  its key with the request, and the request's own `try_acquire` or `block` takes or drops the
  reference in the same critical section, so no extra mutex round trip is paid; `detach_home`
  drops every reference of the home.
- **No allocation on a hit.** A trie node stands for its canonical absolute key: its path from
  the root is the key, and its parent, segment and depth never change while it lives. So a
  request made of cache hits is its nodes, in an inline array of up to 4 (a longer list goes to a
  per-home scratch vector, reused), canonicalized by comparing node paths, and `try_acquire` takes
  the nodes as they are. The mirror keeps the same nodes: `process_locks` holds its `MHold`s in a
  vector that is cleared, never shrunk, and each `MHold` keeps up to 4 keys inline. A hold keeps
  its nodes alive in the manager for as long as the mirror names them (protocol 2.4), and the
  fields read are immutable and were published under the manager mutex, so the mirror checks
  (coverage, upgrade, the handler rule) walk parent pointers with no lock and no string work.
  Keys become strings only when shown (`locks-held`, `:as`, errors).
- **A request's entries are pinned.** A probe pins every entry it hits or makes for the request it
  builds until the manager call that carries its references (`try_acquire` or `block`) returns, and
  the CLOCK sweep passes over a pinned entry. So a miss never evicts another key of the same
  request: `refs.drop` never names a key of the request, and `apply_refs`, which runs before the
  grant test (protocol 4.5, 4.15), never frees a node the request is made of. A key that finds every
  entry pinned (a request with more keys than the cache has entries) is passed uncached, as a key,
  and its node is found or made under the manager mutex. After the call the request's nodes live on
  through its hold or ticket, or through the cache's references, and no other probe runs on the home
  before `%lock-wait`'s own probe in the same step (protocol 5.1).
- **Mirrors and wait fields** belong to the home's thread; the manager never reads them. The home
  passes the owner and base priority with each request.
- **Port.** `std::shared_ptr<lock_port>`, the endpoint for `wake{pid, seq}` and `relevel{pid}` from
  other homes. Both are hints: a wake readies a process only if it is still parked on that ticket,
  and the outcome is collected from the manager; a relevel re-reads the manager's value. The port
  owns its synchronization (protocol 2.3): its own mutex over the queue and `closed`, a `pending`
  flag, and a `shared_ptr` to the scheduler's wakeup, which the scheduler creates at
  construction. A post locks the port's mutex, tests `closed`, appends, sets `pending` and
  notifies the wakeup, all inside that mutex, and touches no scheduler member, so a poster on
  another thread is safe while the scheduler is destroyed. `~lock_home` closes the port under
  the same mutex before it detaches and before any scheduler member goes; a closed port refuses
  posts, which are dropped (LT-3b).
- **Progress.** `std::shared_ptr<std::atomic<clock::rep>> last_progress`, the steady time the
  home last made or awaited progress. The home makes it and passes it to `attach_home`, so the
  manager's record of the home shares the cell (protocol 2.1, 4.4). Every step ends with a clock
  reading that both schedulers already take, for the process's running time and as the next
  step's start (`sched:scheduler.cpp:397`, `sched:async_scheduler.cpp:269`), and the step stores
  that reading in the cell, a relaxed store. So the cell costs one store a step and no clock read,
  and it is current to the home's last step however the home is pumped: by `run_loop` or
  `run_steps`, between the timer waits inside the sync scheduler's `idle_wait`, or one `step()` at
  a time (cvc-cli's serve loop steps a sync scheduler every 200 ms, `cvc-cli/main.cpp:1422`). A
  store every N steps would not do: a holder that sleeps between steps steps once per timer wake,
  so its home's value would age by N wakes, and the idle rule (9.2) would kill a waiter behind a
  holder that keeps stepping (LT-7). pycvc's `run()` also stores it whenever its own idle test
  finds work pending without the lock rule (an async call in flight, a pending coroutine, a
  runnable process). Only `blocking_sinks` reads it
  for another thread: it returns each sink with its home's value, loaded under the manager mutex
  (protocol 4.13), so pycvc's idle rule (9.2) never reaches another home after the mutex is
  released, and an `Exec` destroyed meanwhile on another thread is never touched. A lock wait
  counted as pending by that rule never writes it, so two `Exec`s whose waiters wait on each
  other's parked holders cannot keep each other alive.
- **The scheduler side**, given at construction over the private `procs_`:

  ```cpp
  struct lock_sched_ops {
    virtual bool park_lock(process &, std::optional<clock::time_point> deadline) = 0;
    virtual void ready_lock_waiter(process &) = 0;
    virtual void relevel(process &) = 0;       // after inherited_priority changes
  };
  ```

  `park_lock` closes the running slice as `receive_message` does, then sets the wait fields and the
  status `waiting`. The schedulers call `process_ended(proc, abandoned)`, `wait_expired(procs)`,
  `apply_post(post)`, `holds_locks(pid)` and `on_priority_changed(proc)`.
- **The manager.** `set_lock_manager(std::shared_ptr<lock_manager>)` on both schedulers makes the
  scheduler's `lock_home` and attaches it at once. `app::exec_scheduler()`, Ariadne's init lane
  (from L3), every pycvc `Exec` (from L3) and any host scheduler over an app's tree pass
  `app::lock_manager()`. A scheduler with none makes a private `single` manager, and its
  `lock_home`, lazily: the first time the lock intrinsics ask for its home (`locks()`, 7.4), at
  its first `with-lock` that reaches the manager or its first `lock-query`, logging once when an
  app manager exists. Until then it has no port to drain and no progress cell, so a program
  that never locks runs exactly as before L1 (11.2's `locks=none`). Before L3 a manager accepts
  one home.

### 8.5 The evaluator

- **Entry** (`step_init` at `stackless_evaluator.cpp:1196`; on `perf/eval`, the call path after
  `special_form_of` returns none), when the head is the `lock_ops` object. In order: the generator
  refusal; the options (a literal or symbol `KEY` and `:timeout` evaluated inline, anything else in
  a `lock_enter` sub-step); the mirror checks (handler rule, upgrade, coverage; protocol 5.1), a
  covered request becoming `lock_body` at once; the `:as` scope, made before the acquire so nothing
  after it can throw; then `try_acquire`. A grant makes the frame `lock_body`. Busy, `:if-available`
  makes a `lock_body` with no hold whose body is `:else`, a timeout of 0 (or a context with
  `lock_wait_allowed` false, 5.5) raises `TimeoutError`, and otherwise the frame becomes
  `lock_enter` with a wait child `(%lock-wait KEYS MODE TIMEOUT)`, its arguments in `results`
  (kept by `begin_await`, 419-430, so a restore re-applies them), applied through `apply_native`.
  `TIMEOUT` is the request's resolved timeout in seconds (its `:timeout`, else the context's
  `lock_default_timeout`), or `nil`; `%lock-wait` computes the deadline, so the fast path reads no
  clock (protocol 5.1, 5.2).
- **The lock-enter token.** A thread-local token names the `lock_enter` frame and process, armed
  around the `apply_native` call that applies `%lock-wait`. From `step_init` that is the call above.
  For a restored wait, `step_await_call`'s `again` path (a nil handle; `perf/eval` 1778-1820) today
  only turns the frame back to its apply phase and returns, and the next step's `step_apply`
  (`perf/eval` 1460-1481) applies the native. A snapshot may be taken between those two steps and
  would lose anything the first left for the second, so for a lock wait the `again` path applies the
  native itself, in the same step: when the frame directly below is a `lock_enter` frame of
  `current_pid()` and the head resolves, by name through `callee_of` (the callee record is not
  serialized), to the `%lock-wait` native, it turns the frame back to apply as today, then calls
  `apply_native` on it with the token armed. No mark outlives the step, so the codec has nothing to
  encode, and `step_apply` never arms the token. A user's call to `%lock-wait` never reaches that
  path: it raises `LockError` at its first application (LW-2), so it never parks, even as the `KEY`
  or `:timeout` of a `with-lock`, which a `lock_enter` sub-step evaluates directly above the
  `lock_enter` frame. `rebind_natives` keeps `%lock-wait` bound to the restoring scheduler's
  `lock_ops` (5.8).
- **`%lock-wait`** raises `LockError("cannot wait here")` when `can_await()` is false (LW-1), and
  `LockError` without the token (a user's direct call). Otherwise `block` grants, fails with
  `DeadlockError`, or queues. A grant or failure is collected at once and the native returns `nil`;
  a queued request parks (`park_lock`) and calls `request_await` with the wait's handle and no
  channel, so no `msg-send` reaches it.
- **Resume.** The home wakes a parked waiter only by a wake for its ticket, or when its deadline
  fires and the manager has cancelled or granted it. It first *collects* the outcome from the
  ticket into the `lock_enter` frame (a grant joins the mirror there, before any handler runs),
  then calls `deliver_await(state, nil)` and `ready_lock_waiter`; the handle is `finished()` once
  collected. The `lock_enter` step then marks a grant's body started and becomes `lock_body`, or
  raises `TimeoutError("with-lock: not granted within N s")` (origin `"dsl"`, fields `key`, `mode`)
  or the `DeadlockError`. A wake with nothing collected takes `step_await_call`'s `RuntimeError`
  path (1551-1556), and a `lock_enter` step with no outcome raises
  `LockError("woken without a grant")` and withdraws the ticket. No `awaitable::expired()` hook.
- **Completion and unwinding.** `step_lock_body` runs the body as `begin_next` does, then releases.
  `unwind_exception` (282-325), `unwind_jump` (332-364) and `fail_step` (757-768, which today pops
  the raising frame with a bare `pop_back`) call `release_lock_frame(top)` (noexcept) before
  popping a lock frame and before the `try` checks, so the order with `finally` is LIFO. A release
  looks its hold id up in the current process's mirror, so a restored copy's frame finds nothing.
- **Frame layout.** `extra_vals = {lock_ops, keys, :else, wait handle or outcome, mode, :as name,
  hold id or nil, body forms...}`, the body from slot 7, with `extra_lists` and `extra_strs`
  empty: with `perf/eval`'s recycled vectors a single-key, one-form section allocates nothing.

### 8.6 The protocol

[`docs/STATE_EXEC_LOCKS_PROTOCOL.md`](STATE_EXEC_LOCKS_PROTOCOL.md) specifies it as pseudo-code over
the manager's state, with its invariants and their proofs; the model checker (LT-5) implements it
literally, and `lock_manager::check()` asserts its manager-side invariants after every operation in
test builds (the list is protocol 6; the home-side ones are asserted by the tests at quiescence,
13).

Its operations are `try_acquire` (the fast path), `block` (grant, deadlock check, or queue),
`release`, `cancel` (a timeout), `resume` and `start_body` (collecting a wake, entering the body),
`end` and `detach_home` (a process or a scheduler ends), `set_base`, the key-cache references
(`ref_nodes`, `unref_nodes`, and the same carried by `try_acquire` and `block`) and the reads.
Moves between homes (Stage 3's `adopt`) are not in it yet: L4 adds them, model-checked, before it
merges (9.3, protocol 9). Every release, cancel and end re-examines the waiters that overlap what
was freed, in arrival order, and every change to the wait-for graph or a base recomputes
inheritance.

An operation collects its wakes and relevels under the mutex, and the caller delivers them after
the unlock: at once to its own processes, by post to another home. **No lost wakeup:** a grant and
a cancel each happen under the mutex, so one wins and the outcome is in the ticket; after every
operation no queued ticket is grantable (protocol I6), since a ticket queues only when not
grantable and its grantability grows only when an overlapping hold or waiter leaves, which
re-examines it; and a wake goes to the waiter's port, under the port's own mutex, and is applied
between steps, after the park. A port is closed only as its home is destroyed, under that mutex,
with no pump running and before the home's processes are ended: by `~lock_home`, before the
detach that ends them, or, for a group, by its teardown as soon as it holds the group's token,
before it kills them (9.2).

### 8.7 Hooks in existing files

| File:line | Change | Phase |
|---|---|---|
| `stackless_evaluator.h:36-63`; `generator.h:26-75` | `lock_enter`, `lock_body`; `in_generator` set on a closure generator's state | L1 |
| `stackless_evaluator.cpp`: `step_init` (1196), dispatch (694-760), `unwind_exception` (282-325), `unwind_jump` (332-364), `fail_step` (757-768), `step_await_call`'s `again` path (`perf/eval` 1778-1820) | the soft form and its mirror checks; `step_lock_enter`, `step_lock_body`; `release_lock_frame` before popping a lock frame; the lock-enter token around `apply_native` of `%lock-wait`, in `step_init` and, for a restored wait, in `step_await_call`'s `again` path, which applies `%lock-wait` itself, in the same step, when the frame below is a `lock_enter` frame of `current_pid()` and the head resolves to that native (8.5); `step_apply` is unchanged | L1 |
| `evaluator.cpp:295-314`, `async_evaluator.cpp:229-244` | the `lock_ops` head raises `TypeError("with-lock needs a scheduler")` before the arguments | L1 |
| `intrinsics.cpp:1062-1125, 83-91, 120-127, 631-700`; `exceptions.cpp:58-64`; `state.h`, `state.cpp` | the new intrinsics and `%lock-wait`; the atomics through `read_through_link`, `std::to_chars`; `inspect` fields; three classes; the three `cvc::state` members | L0-L2 |
| `intrinsics.h:139-159`, `intrinsics.cpp:420-440` (`spawn`, `fork`) | `lock_default_timeout` and `lock_wait_allowed` (L1); fork of `current_pid()` and `LockError("fork")` (L1, td B28); `spawn_context` copying the parent's `sched`, `host`, pinned `root`, `root_path`, `uid`, `channels`, `lock_default_timeout`, `lock_wait_allowed` and the running process's owner (L1b); `parent_pid`, floor, clamp (L2) | L1, L1b, L2 |
| `scheduler_base.h:48-67, 71-114, 159-210`; `sched:process.h:180-193` | the fields and virtuals of 7.4 and 8.2: `set_priority` (host, unchecked), `get_priority`, the DSL's `set_priority_checked(caller_pid, pid, p)` and `get_priority_as(caller_pid, pid)` (owner, uid, descent and floor checked by `detail::check_priority_change`; `process_info` gains no `owner`), `set_priority_floor`, `policy`, `locks` | L1, L2 |
| `sched:process.cpp`: `deadline_of` (153-161), `update` (192-265), `wake_due` (499-527), `check` (531-605); `sched:process_table.h:113` | a `lock_deadline` branch. A `level_key(const process &)` helper (`by_priority_ ? p.effective_priority() : 0`) used by `update` (202) and `check` (552), and by every other reader of the level key (L2; until then `by_priority_ ? p.priority : 0`, today's key, so the round-robin table keeps its one level). `std::size_t wake_due(clock::time_point now, std::vector<process *> *lock_expired = nullptr)`: a due `lock_deadline` is always cleared and never re-pushed (else `update` re-pushes it for ever); with a vector the process is appended, with `nullptr` it gets `lock_expired_pending = true` (a `process` field, 8.2) and the table counts it. Both schedulers' call sites (`drain`, `sched:async_scheduler.cpp:345`; `step_once`, `sched:scheduler.cpp:450`) pass a vector and call the home's `wait_expired` on it right after, in the same pass, before any step or wait, so a scheduler never sleeps past an expired waiter, which has no timer left. `nullptr` serves only the table's own tests (below); a process it flags is collected by the next `wait_expired`, which also takes every flagged process while the count is non-zero. Lock expiries count in the return value, so the existing callers' counts (both schedulers, `sched:state_exec_scheduler_test.cpp:1386-1431`) are unchanged when no lock waiter is due. Lock-waiter checks and `lock_manager::check()` | L1, L2 |
| both schedulers: `lock_sched_ops` (`sched:async_scheduler.cpp:504-574`), end functions (1059, 1069, 1077; 798, 808, 815), `fork` (1156; 1000), `restore`, `has_runnable` (565-569) | `park_lock`, `ready_lock_waiter`, `relevel`; `process_ended(proc, abandoned)` after `set_status` when `proc.locks`; -1 for a holder, the child copies `owner` (and the floor, L2); reset lock fields and refuse a stack with a held `lock_body`; lock waiters count | L1, L2 |
| both schedulers: `set_priority` (1215-1222; 1062-1069), the constructors (`sched:async_scheduler.cpp:17-20`), `wait_for_work` (459-497), `run_loop` (384), `drain` (317-348), `step_once` (445), `run_steps`, `idle_wait` (515-561), `execute_process_step`, the destructors | the override, also setting the manager's base (L2). The lock port (protocol 2.3): the scheduler's wakeup becomes a `shared_ptr<detail::wakeup>` made at construction and shared with the port (the sync scheduler's `idle_`, `sched:scheduler.cpp:542-543`, is no longer made lazily in `idle_wait`); `drain` and `step_once` apply the port's posts when its `pending` is set, and pass a vector to `wake_due` and call `wait_expired` with it in the same pass (the `sched:process.cpp` row); `wait_for_work` tests `pending` before it blocks, as it tests its ingress, and returns when a post notifies. The sync scheduler has no ingress, and `idle_wait`'s wait loop (`sched:scheduler.cpp:548-551`) today tests only `stop_requested_`, ignores `wait_until`'s result and sleeps in 10 ms slices up to `until`: it also tests the port's `pending` before each slice and leaves the loop when it is set, and `idle_wait` then returns true, so `run_steps` steps again and `step_once` drains the port. `idle_wait`'s early return (`sched:scheduler.cpp:536-540`) is unchanged: a run bounded by `max_steps` returns at once when no timer is set or its remaining steps would run out first, lock waiters or not, as with message waiters (6.1). `~lock_home` closes the port first in the destructors. `last_progress`: `execute_process_step` stores the clock reading it ends each step with (`sched:scheduler.cpp:397`, `sched:async_scheduler.cpp:269`), a relaxed store, and pycvc's `run()` stores it too (8.4); none of it without a lock home; the `locks_lost` raise (L7) | L2, L3, L7 |
| `snapshot` (1142; 986), `state_value_codec.cpp:286-288, 638-647, 683-688, 1087-1105, 1114` | refuse holders; a wait handle encodes as `nil`, a `lock_ops` object as a marker decoding as `nil`; a waiter's wait child `(%lock-wait KEYS MODE TIMEOUT)` encoded with `TIMEOUT` rewritten to the time left (`p.lock_deadline - now`, at least 0; `nil` stays `nil`) when the process is parked on it, and kept as decoded for a restored copy not yet stepped, 5.8; `rebind_natives` replaces a decoded or stale `lock_ops` and rebinds `%lock-wait`; the new phases; `priority_floor` | L1, L2 |
| `exec_coordinator.cpp:712-753, 1045-1071` | `set_priority` control (L2); `migrate`'s refusal, `release_locks` and `locks_lost` inside the paused control entry (L7) | L2, L7 |
| `app.h`, `app.cpp:1024-1038`; `ariadne.cpp:377-420, 1636-1686` | `app::lock_manager()`, `state_exec.policy`, `state_exec.locks.threading`; actions' default timeout; `run_scoped_program` sets `lock_wait_allowed = false` (and a default of 0) on the init lane's context (5.5); the init lane on the app manager (L3) | L1-L3 |
| `pycvc_exec.cpp:366-424, 579-648`, `pycvc_exec.h:32`; `pycvc.i` (3437, 3502, 3959-3969, 4152-4180, 4192-4205) | `Exec(app, policy=None, idle_timeout=60.0)` as one constructor declaration replacing `pycvc_exec.h:32` (no overload), `policy` a `PyObject *` (`None`, a `scheduling_policy` member, or its name) parsed in `pycvc_exec.cpp` (`ValueError` otherwise); `%feature("kwargs") pycvc::Exec::Exec;` and `%feature("kwargs") pycvc::Exec::lock_query;` (the class is `pycvc::Exec`, `pycvc_exec.h:27`, and SWIG silently ignores a feature whose name matches nothing), placed beside `%pythonappend pycvc::Exec::scheduler` (4192), before `%include "pycvc_exec.h"` (4205), since the module enables no keyword arguments; the priority API under the control mutex, `run()`'s thread id and thread-local `ExecImpl` pointer, `RuntimeError` for a waiting call from another `Exec`'s `run()` thread, a new `ExecImpl::accepting_queries` flag (not the re-entrancy guard `running`) set before the first step and cleared, with the queue answered or failed, under the control mutex before `wait_idle(host)` on both exit paths, `reap` under that mutex, and `running` tested and set at entry and cleared by `RunGuard` last, under the same mutex, so a second `run()` during the quiesce raises as today (7.4); the control mutex as the home's pump token when no `run()` is active (7.4, 9.1); `lock_query` straight to the manager's `query` with this `Exec`'s home and owner `""`; the app manager; `idle_timeout` and the lock-wait idle rule with `blocking_sinks`' progress values (9.2); the new struct fields as `%extend` properties; the `lock_ops` `keep` slot | L1-L3 |

### 8.8 The uncontended fast path on one thread

Against `(begin body...)`, a literal key, single mode: **0 extra steps** (the entry and completion
steps replace `begin`'s), and about 1 ns to recognize the form, 0-10 ns for options, 1-3 ns for
the mirror scan, 10-15 ns for the cache probe, 5-10 ns for the trie walk and counts at depth 1-2,
3-5 ns for the hold record, and 5-10 ns for the release (re-examination skipped when no waiter
overlaps), with no clock read (the deadline is computed only on the busy path, protocol 5.2) or
allocation (records and holds come from free lists, and the key's node stays alive through the
cache's reference, protocol 2.4; the home builds the request and the mirror's keys from the cached
nodes in reused storage, 8.4) after a process's first lock and a key's first use:
**~25-50 ns** in single mode, and **~65-90 ns** in shared mode with one uncontended mutex lock and
unlock on each side. Against a scheduled step of 249-263 ns (sync) and 456-470 ns (async) before
`perf/sched` (td 2.5), that is 0.1-0.2 and 0.25-0.35 of a sync step; a lock-server round trip
costs 23-95 us. Programs that never lock pay one variant-index test per `data_object` head and
one predicted branch per end function; step counts are identical. On a scheduler that has a lock
home, every step also tests the port's `pending` flag and stores the clock reading it already
ends with in `last_progress` (8.4), a relaxed store; a scheduler given no manager has no home
before its first lock request, and pays neither until then.

## 9. Across execution groups

### 9.1 Who touches what

| State | Writers | Protection |
|---|---|---|
| The trie, holds, tickets, the queue, per-process records, inherited values | any home's thread, inside a manager operation | the manager mutex (shared mode) |
| Mirrors, lock fields of processes, `inherited_priority`, key caches, lock frames | the home's pumping thread; for a pycvc `Exec` with no `run()` active, the holder of its control mutex (7.4) | the pump token (td 4.1); for a pycvc `Exec`, being its active `run()`'s thread, or else holding its control mutex, which debug owner checks accept |
| Posts to a home (`wake`, `relevel`) | any thread | the port's own mutex, its `pending` flag and the wakeup it shares with the scheduler (protocol 2.3); for a group (Stage 2) that wakeup is `wake(g)`, called inside the port's mutex while the port is open, and a group's teardown closes its port as soon as it holds the group's token, before it kills the group's processes (9.2); never a scheduler member |

Only a home's own thread readies, parks, re-levels or ends its processes. Another home only posts
hints to it, and every outcome is read from the manager.

### 9.2 Stages 0-2

- **Several `Exec`s or schedulers on several threads** (L3, before any td stage) share one
  `app::lock_manager()` in shared mode. Wakes and relevels to another home are posts to its lock
  port (8.4); an `Exec` in `wait_for_work` wakes on them (`sched:async_scheduler.cpp:459-497`),
  since the port shares its wakeup. The sync scheduler gets the same port, its wakeup made at
  construction, and its `idle_wait` returns when the port's `pending` is set; a `run(N)` frame
  pump still returns on budget, and at once when no timer is set, as with message waiters (6.1,
  8.7). A deadlock between `Exec`s is found when the closing request blocks (LD-5).
- **pycvc's idle limit.** `Exec.run()` kills a program parked for its idle limit (`idle_timeout`,
  60 s by default) with nothing it counts as pending (`pycvc_exec.cpp:579-648`). From L3 a lock
  wait counts as pending only on observable progress elsewhere: some sink that
  `blocking_sinks(waiter)` returns (protocol 4.13) is on another home, and the progress time
  returned with it, that home's `last_progress` (8.4) loaded under the manager mutex, lies within
  the idle window. So a wait behind another `Exec`'s long section that keeps stepping or awaits an
  async call is not killed (LT-7), while a wait behind a holder whose `Exec` nobody pumps, or that
  has made no progress within the limit, is killed at the limit, naming the key (LT-8). The rule
  is per home, not per holder: a holder parked for ever on another `Exec` that keeps stepping
  other processes keeps the waiter alive, as a busy sibling keeps a parked program alive today. A
  sink on the waiter's own home counts only through `run()`'s existing test (`in_flight`,
  `pending_coros`, `has_runnable`, `had_runnable`), never as pumped merely because this `run()`
  pumps it: a same-`Exec` holder parked on `msg-recv`, with nothing else pending there, is killed
  at the limit, as at L1-L2 (LT-8b).
- **Stage 1 (groups, fork-join).** Each group's scheduler is a home with its own cache and mirrors.
  Uncontended locks across groups are immediate. A grant to a waiter in another group lands at
  that group's next pump: the next frame in fork-join mode. The spawn context gives children the
  lock intrinsics against the parent's chroot, in any group.
- **Stage 2 (continuous pool).** A lock post wakes its group: the wakeup a group's scheduler shares
  with its lock port notifies by `wake(g)` (td 4.4), so a grant's latency is the post plus a wake
  when the group is parked (5-20 us, td 6.1; td gates the wake at 50 us p50 on Windows). Lock posts
  go to the group's lock port, outside the ingress cap, and are dropped only once the port is
  closed. A post calls `wake(g)` inside the port's mutex, after it finds the port open (protocol
  2.3). A group's teardown (td 4.2) closes its lock port right after it takes the group's token,
  waiting for a pump in progress, and before it kills the group's processes, so while the group and
  its state word are whole; `~lock_home`'s own close then finds the port closed. So every `wake(g)`
  from a lock post completes before the close, on a whole group (td's teardown already meets such a
  `wake(g)`: a kernel's completion during its step 3 makes one), or never happens, and none reaches
  a dying group (LG-2). Taking the token first means no pump runs while the port is closed: a grant
  to one of the group's waiters after the close has its wake dropped, and the waiter's kill in the
  next teardown step releases it (protocol 5.7). The port's mutex comes before the pool's
  ready-queue mutex, which is never held while a port is posted to or drained. `pool.kill_owner`'s
  acknowledgement covers locks, since a kill releases when it is applied. Cross-group `set-priority`
  uses `post_control`, and groups publish `best_runnable` (7.5).
- **No homes for locks, and no router.** Locks are not channels, so Q6 does not apply. The manager
  is synchronized internally, so an uncontended acquire costs the same from any group, and only
  wakes and relevels cross groups.

### 9.3 Stage 3: `spawn-isolated` and moves

- **Holders may move.** Holds are per pid and pids are kept on a move (Q11). The mirror travels with
  the process. `adopt` runs on the target's thread (td 4.15, step 4); there it renames the
  process's manager record from `(old home, pid)` to `(new home, pid)` under the mutex. A boost is
  never carried: `adopt` resets every arriving process's `inherited_priority` to none, and the new
  home re-reads it from the manager when it applies priorities. So a holder moved to a
  round-robin home shows no stale boost in `effective_priority()`, which does not know the policy,
  or in `inspect`, and a relevel lost on the old home leaves nothing stale. The protocol does not
  cover this yet: every hold and record is keyed by `(home, pid)`, and `applies` reads the home.
  L4 adds the operation sketched in protocol 9, `move(P, H2)`, which the new home calls inside
  `adopt` (rekey the record and its holds, recompute inheritance, re-read the moved process's
  inherited value at once when the new home applies priorities), with moves as model-checker
  actions, before it merges.
- **Waiters cannot move**: they are parked, which td 4.15's eligibility already excludes.
- **Posts in flight** to the old home for a moved pid are hints that find no process there; a moved
  process has no ticket, and its inherited priority is reset and re-read on arrival.
- **Movable processes** reach their home through `group_scheduler_ref` (td 4.15), which forwards
  `locks()` and every priority virtual of 7.4 (`set_priority`, `get_priority`,
  `set_priority_checked`, `get_priority_as`, `set_priority_floor`, `policy`) to
  `exec_group::current()->sched()`. Without those overrides a movable process would reach the
  defaults: its `set-priority` would return `#f` and its `get-priority` `nil`.

### 9.4 Stage 4 (in reserve)

The manager is already safe for threads. The key cache and mirrors become per domain, and a wake
targets the waiter's domain inbox (td 4.16). If the manager mutex or handoff convoys show in the
data, sharding by subtree (3.3), or Go's 1 ms starvation rule inside the grant rule, can be added
without changing the DSL.

### 9.5 Emscripten and one thread

The same code in `single` mode. With several homes on one thread (groups pumped inline at threads
0, td 4.14), wakes and relevels to another home still go through its lock port (8.4), whose own
mutex is then uncontended. Deadlock detection is the same exact check.

## 10. Migration between hosts (the late stage, L7)

This comes last, after the single-machine phases, as the user asked. It depends on L1-L3 and td
Stage 0's `post_control`, not on groups.

### 10.1 Holders, waiters and priority

- **Holders, until td Stage M.** `migrate` ships metadata and kills the source
  (`exec_coordinator.cpp:712-753`). The kill releases the source's locks through the end hook,
  abandoned, like any kill (5.6), so nothing moves unlocked and L1-L6 change nothing here (LW-8).
- **Holders, from L7** (with td Stage M, when `migrate` ships `encode_process` and waits for an
  acknowledgement). `migrate` refuses a process that holds locks, naming the keys. The refusal is
  checked inside the paused control entry, on the pumping thread that owns the mirror, never from
  the coordinator's thread. With `release_locks = true` the same entry releases the holds through
  the end hook's path, marked abandoned (the section was cut short exactly as by a kill), and marks
  the source `locks_lost`. If the source is ever resumed (the target nacks, fails or never
  acknowledges), `LockError` (`"lost"`) is raised into it before its next step, and the moved
  process raises the same at its first step (5.8), so neither continues its section unlocked
  beside a new holder (LM-1). L7 adds `locks_lost` and its raise; nothing before needs them.
- **Waiters** cannot migrate: `pause` fails on a waiting process (`exec_coordinator.cpp:719-722`).
- **Priority.** Base and floor travel in the metadata; boosts never do. The target applies its own
  `resource_policy::priority_floor`, clamping and logging a better carried base (PR-11).
- **Host failure.** A host's locks guard its own tree and vanish with it.

### 10.2 L7: cross-host locks

- **Where a lock lives.** A key under a prefix whose authority is another cluster
  (`state_authority_map`, `state_delegation_manager::route`) is held at that cluster's host. A
  local acquire of such a key goes over the coordinator bus and parks as a cross-thread wait does.
  An `expired` route raises `LockError` (`"lost"`).
- **Leases.** A remote grant is a lease: owner `{host incarnation, pid}`, a TTL (default 10 s),
  renewed in batches on the coordinator heartbeat (`exec_coordinator.h:129`, 2000 ms).
- **The authority host** is the host whose lock manager holds the key's leases: the manager of the
  authority cluster's app (one per app, 8.4). It is not a `lock_home`, which is one per scheduler
  and has no lease table.
- **Tokens.** `(authority manager incarnation << 40) | generation`, the generation being that
  manager's one counter (4.1), so a restarted authority's tokens dominate every earlier one. `:as`
  exposes the generation from L1 and the token from L7.
- **Fencing.** With `:fence`, the holder's writes under the key carry the token in the
  `state_mutation` (`state_change_journal.h`), and the authority's shard rejects a token lower than
  the current grant's (`state_cluster_shard.cpp:263-297`).
- **Rebind on migrate.** The codec carries `held_leases {authority, key, mode, token, count}`. The
  target restores the process paused, sends `lease-rebind{key, token, new host, new pid}` to each
  authority host, and resumes after every ack. A nack, or no ack within the TTL, raises
  `LockError("lost")` at the first step.
- **Failures.** The holder's host fails: renewals stop, the authority host expires the lease at
  TTL, marks it abandoned, bumps the token and grants the next waiter; the old holder's fenced
  writes are rejected. The authority host fails: grants become invalid, fenced writes fail, remote
  waiters get `LockError("lost")`, and authority failover follows the delegation manager.

## 11. Performance and the bench scenarios

### 11.1 What to expect

- **One thread, uncontended:** 25-50 ns single, 65-90 ns shared, no step (8.8).
- **One thread, contended:** the waiter's park and resume (about 3 steps: the wait native, the await
  frame, the body start) and 0.5-1 us of bookkeeping, a deadlock search over the queued requests
  included. About 1/50 of a message round trip.
- **Many parked waiters** cost nothing per step on `perf/sched`: a waiter is not in the run queue
  and its timer is in the heap. A release with no overlapping waiter skips re-examination.
- **Many threads:** every lock operation takes the one manager mutex. Disjoint keys share it and the
  counters of common ancestors; a contended key also pays a post and a wake per handoff. The docs
  advise keeping hot keys within one group, as for chatty channels (td 4.6). `lock.cross_thread`
  decides whether the manager needs sharding (3.3).

### 11.2 New `state_exec_bench` scenarios

The lock metrics are measured as in `native.*`: the loop with and without `with-lock` (or the
atomic) alternated within each rep, the fastest of each kept, and 5 or more reps before a change is
read. They are reported under `extra.*`.

| Name | Executors | Workload | Sweep | Metrics and gate |
|---|---|---|---|---|
| `native.state_atomic` | sync, async | the `native.*` method, one scenario whose `op` param picks the native | depth 1, 4; `op` set, incr, cas | `extra.ns_per_call`; gate: `incr` and `cas` at most 1.1x `set` (`compare RUN RUN --pair op=set:incr`, and `op=set:cas`, `--metric extra.ns_per_call --threshold 10`: no `WORSE`) |
| `lock.uncontended` | sync, async | `(with-lock K (set s (+ s 1)))` against the same loop with `begin`, and nested covered sections | depth 1, 2, 4; mode x, s; chroot 0, 1; threading single, shared (L3) | `extra.ns_per_lock`, `extra.steps_per_lock`, `extra.allocs_per_lock`; gate: extra over `begin` at most 60 ns single (L1) and 120 ns shared (L3) (sync, flat key, Release), 0 steps, 0 allocations for a single-form body |
| `lock.handoff` | sync, async | p `forever` loops on one key, a `(sleep 0)` in the body, 200 x p handoffs after warm-up; host-submitted or spawned (L1b) | p 2, 8, 64; workers host, spawn | `handoffs_per_s`, `wait_ns` p50/p99, `extra.fairness_spread` (most grants minus fewest); gate: at most 1 |
| `lock.rw` | sync, async | r readers, w writers | 8/1, 64/1, 8/8 | writer `wait_ns` p99; gate: at most 2x the longest reader section at 64/1 |
| `lock.many_keys` | sync, async | random keys out of k, single and two-key requests | 10, 1000, 100000 | `extra.ns_per_lock`, cache hit rate, live trie nodes |
| `lock.bystanders` | sync, async | `sched.single.arith_loop` beside b parked lock waiters | 0, 100, 1000 | `ns_per_step`; gate: at b = 1000 within 3% (and the noise) of b = 0 (`compare RUN RUN --pair b=0:1000 --metric ns_per_step`: no `WORSE`) |
| `lock.timeout` | sync, async | p waiters with a 1 ms timeout | 10, 1000 | expiry lateness p99 |
| `lock.deadlock` | sync, async | cycles formed repeatedly, beside q unrelated queued waiters | n 2, 8; q 0, 1000 | `ns_per_detection` |
| `lock.inherit` | async, `priority_rr` | low holder, high waiter, m medium spinners; `inherit=off` reps bounded by a 50 ms spinner window | m 0, 1, 8; `inherit` on, off | the high waiter's `wait_ns`, `extra.starved`; gate (on): p99 at most 2x the holder's section |
| `lock.cross_thread` | t async schedulers on t threads, one shared manager | disjoint keys (sections of 0.3, 3, 30 us), then one hot key; the disjoint workload without `with-lock` in the same rep | t 1, 2, 4, 8 | scaling, mutex wait time, cross-thread `ns_per_handoff`; gate: disjoint 3 us sections at t=4 at least 0.9x the no-lock run, handoff p50 under 50 us (Windows) and 20 us (Linux); decides sharding (3.3) |
| `prio.set` | sync, async | `set-priority` flipping one process with n queued | n 1, 1000; policy rr, priority; levels 1, 40 | `extra.ns_per_change`, `extra.ns_per_pick`; gate: at most 500 ns per change at n=1000 under `priority` |
| `mt.groups.locks` | pool (td Stage 2) | G groups locking keys under their own prefix, then one shared `/` key; the same without `with-lock` in the same rep | threads 1-8 | throughput, cross-group handoff latency; gate: disjoint keys within 5% of the no-lock run |
| `mt.groups.locks.isolated` | pool (td Stage 3) | `spawn-isolated` children locking their parent document's keys; disjoint, then one hot key | threads 1-8 | throughput, handoff latency |
| `prio.groups_weighted` | pool (td Stage 2) | 2 busy groups at best priorities 0 and 10 at threads 1; 4 at 0, 0, 10, 10 on 2 workers | - | `extra.share_ratio` against the CFS ratio (about 9.3 between a 0 and a 10 group); gate: within 20% |
| `mt.migrate` | coordinator harness (td Stage M) | a program moved between two hosts on loopback, with and without held leases (L7) | - | move latency, rebind round trips |

**The policy and locks sweeps, and paired points.** `compare BASE CAND` matches records on
scenario, executor and params (`docs/STATE_EXEC_BENCH.md`), so a gate between two points of one
scenario, or on a param value the base binary lacks ("only in cand", no verdict), needs a way to
pair points. A bench-only commit on the base branches, before L0 and L1 merge, adds:
- `compare --pair KEY=A:B`, which matches each base record whose `KEY` is `A` with the candidate
  record whose `KEY` is `B` and whose scenario, executor and other params are the same, with the
  usual medians, noise and threshold. `compare RUN RUN --pair ...` then compares two points of one
  run;
- two sweep keys, read where `world` (`state_exec_bench.cpp:2669-2700`) builds its scheduler and
  swept by `sched.single.*`, `flood.small` and `msg.*`: `policy` (`rr`, `priority`,
  `priority_rr`; default `rr`), and `locks`, whose only value there is `none` (no lock manager,
  today's behaviour).

L1 adds the `locks` value `private` (a single-mode manager attached, as a scheduler's private one
is), and L3 adds `shared` (a shared-mode manager attached); neither can exist before the lock
manager does. In an L1 or later binary, `locks=none` still builds today's scheduler: `world`
passes no manager, and a scheduler given none makes its private manager and lock home only at
its first lock request (8.4), so the swept workloads, which never lock, run exactly as before.
`private` and `shared` pass their manager through `set_lock_manager`, which attaches the home at
once, so those points carry an attached home's per-step costs (8.8). Base and candidate binaries
share every `locks=none` point, so the regression gates run `compare BASE CAND` there.
A `locks` value the base lacks is gated only within the candidate's own run: L3's exit is
`compare CAND CAND --pair locks=none:shared --fail-on-worse` on `sched.single.*`, `flood.small`
and `msg.*`, within 3% (and the noise).

**Regression gates** for every phase, at `locks=none`: `sched.single.*` (from L2 also under
`priority_rr`), `flood.small`, `native.state_set`, `msg.*`, `eval.try_noraise`, `eval.exceptions`
and `eval.raise_depth` (d 16, 64; stackless) within noise (base and candidate interleaved twice
with 5 reps, `docs/STATE_EXEC_BENCH.md`).

## 12. Phases

Every phase builds on `perf/sched`, `perf/eval` and `perf/async` together: the lock wait is a
process-table wait kind and cross-thread wakes ride the lock port's shared `detail::wakeup` and
`perf/sched`'s `wait_for_work`; the soft form hooks `perf/eval`'s call path, and the 0-allocation
gate relies on its recycled frame vectors. Each phase ships on its own, on Windows, Linux and
macOS, with Emscripten building.

| Phase | Content | Built now | Exit |
|---|---|---|---|
| **L0: atomic natives** (~1 week) | `state-incr`, `state-cas`, `state-swap`; the three `cvc::state` members | yes | AT; `native.state_atomic`'s gate (`incr` and `cas` at most 1.1x `set`) |
| **L1: locks on one scheduler** (2-3 weeks) | the manager in single mode with one home; `lock_home`; soft `with-lock` with every option; `locks-held`, `lock-query`; the end hooks and `fail_step`; abandoned; the handler rule; the generator, fork, snapshot and held-restore refusals; td B28 whole; the codec's `lock_ops` marker; the three classes; both schedulers; the recursive evaluators' refusal; Ariadne's timeouts and `lock_wait_allowed`; the model check on one home; the bench's `locks=private` (11.2); docs. Every home attaches with no priority policy (protocol 4.4), so inheritance is computed for no one and the run queue keeps today's key (`by_priority_ ? p.priority : 0`) | yes | LK (L1 cases), LX (L1 cases), LW, LD-1..LD-4, LD-7, LD-9, LD-10 on both schedulers; LT-5 on one home, `round_robin`, no `set-priority`; `lock.uncontended` single, `lock.handoff` fairness and `lock.bystanders` gates; every existing suite identical, step counts included, after td 5's fork scan |
| **L1b: spawn context** (~1 week) | td 4.15's single-scheduler part of B27: `spawn_context`, `make_spawn_context`, Ariadne's and pycvc's callbacks; `with-lock` in children against the parent's chroot | yes, after L1 | LK-24, LK-31; td IS-1 and IS-2 (on one scheduler, no groups); `lock.handoff` `workers=spawn`; td 5's spawn scan |
| **L2: priority** (1-1.5 weeks) | `set-priority`, `get-priority`, floors and the spawn clamp, `parent_pid` on spawn, the `scheduler_base` virtuals, `state_exec.policy`, inheritance switched on (homes attach with their scheduler's policy; the run queue keys on `effective_priority()` through `level_key`), `inspect` fields, `thread_class`, pycvc's policy and priority API, coordinator control (with `post_control`). It uses the bench's `policy` sweep (11.2) | yes | PR, PI-1..PI-5, PI-8, PI-9, PI-11; LT-5 with priorities; `prio.set` and `lock.inherit` gates; `sched.single.*` under `priority_rr` within noise |
| **L3: one app, several threads** (1-1.5 weeks) | shared mode (the manager mutex), `app::lock_manager()` for every scheduler, several homes, the port with `wake` and `relevel` posts on both schedulers, the sync `idle_wait` contract, pycvc `Exec`s on the app manager with `idle_timeout`, `Exec.lock_query`, Ariadne's init lane on the app manager, the bench's `locks=shared` (11.2), TSan | yes, after L1-L2 are hardened | LT; LD-5, LD-6; PI-7, PI-10; LX-5, LX-7, LX-10, LX-11 posted; LT-5 on two homes; TSan clean; `lock.uncontended` shared and `lock.cross_thread` gates; `sched.single.*`, `flood.small` and `msg.*` at `locks=shared` within 3% (and the noise) of `locks=none` in the same run (`compare --pair`, 11.2) |
| **L4: groups** (with td Stages 1-3, ~1 week each) | a home per group, `wake(g)` on posts, the spawn context in groups, cross-group `set-priority`, `best_runnable` and pool weights, `worker_class`, `adopt`'s record rename (the protocol's `move`, protocol 9, specified, proved and model-checked first), `group_scheduler_ref`'s `locks()` and priority overrides (9.3) | no: waits for td Stages 1-3 | LT-5 with moves; LG; `mt.groups.locks`, `mt.groups.locks.isolated`, `prio.groups_weighted` |
| **L5: guarded subtrees** (optional) | host-declared prefixes where DSL writes need a covering X | no: on request | `native.state_set` within 2% with none declared |
| **L6: semaphores** (optional) | `:permits N` on a key, owned permits released on end | no: on request | semaphore tests; the model check extended |
| **L7: migration between hosts** (late) | `migrate`'s refusal of holders, `release_locks`, `locks_lost` and its raise (10.1); leases, tokens, fencing, rebind, failures (10.2), with td's Stage M | no: last; needs L1-L3 and td Stage 0 | LM; `mt.migrate` recorded |

L0 and L1 are independent, and both need the bench-only commit of 11.2 (`compare --pair`, the
`policy` and `locks` keys); L1b needs L1 (and L0 for LK-24); L2 needs L1; L3 needs L1 and L2. L1
brings td B28 forward from Stage 0, L1b brings B27's single-scheduler part forward from Stage 1,
and L2 adds `post_control` for `set_priority` if Stage 0 has not landed (7.4). L1-L3 with L1b are
the "hardened on one machine, single or multiple threads" milestone: host-submitted, forked and
spawned programs all lock there. L4 follows the threaded stages, and L7 comes last. Nothing before
Stage M needs `locks_lost`: before L7 no held section can be snapshotted, so restore refuses one.

## 13. The test list

Unless a case says otherwise, cases run on both schedulers, on Windows (MSVC) and Linux (gcc).
`process_table::check()` and `lock_manager::check()` run after every step in the test builds.
`lock_manager::check()` asserts the manager-side clauses (protocol 6): I1 (no conflicting holds
of different processes), the manager's clauses of I2, I3 and I4 (the ticket and record tables,
the trie's counts equal to their definitions), I5 (FIFO), I6's first half (no queued ticket is
grantable), I7 (an acyclic wait-for graph), I8's manager clause (inherited priorities equal to
the rule), I9's key-cache clause (a reference entry exactly for each attached home), I11 and I15.
The home-side clauses read a home's private state, and some need every home drained, so the
tests assert them at quiescent points, where every home has delivered its notifications and
drained its port, with `lock_home::check_quiescent()` (test builds): the home-side clauses of I2
and I3 (tickets and mirrors against the manager), I4's (each key cache against its references),
I6's second half (no process `lock_waiting` on a non-queued ticket), and I8's base clause
(`M.recs[P].base == p.priority`) and quiescent clause (every applying process's
`inherited_priority` equal to the manager's). LK-18 asserts them after every step (one home
delivers every notification at once and receives no post), LT-4 and LT-10 at quiescent points
during the run and at its end, where the manager must also be empty (I9).
Tests of machinery the simplified core removed are retired and their IDs not reused: LK-12,
LK-29, LX-9, LD-8, PI-6, LT-6 and LT-9.

### 13.1 Atomics (AT, L0)

| ID | Situation | Result |
|---|---|---|
| AT-1 | `(state-incr "n")` on an absent node, then `(state-incr "n" 5)`, then `(state-incr "n" 0.5)` | `1`, `6`, `6.5` |
| AT-2 | `state-incr` on `"abc"` | `ValueError`; the node unchanged |
| AT-3 | `(state-cas "k" nil "a")` twice | `#t`, then `#f` |
| AT-4 | `(state-swap "k" "b")` | `"a"`; the node is `"b"` |
| AT-5 | all three through a writable transparent link and on a read-only node | write through; the read-only error |
| AT-6 | two threads x 10^6 `updateValue` increments on one node (TSan) | exact; clean |
| AT-7 | `state-cas` and `state-incr` changing a watched node | the watch and `valueChanged` fire once each |
| AT-8 | a `state-cas` or `state-swap` whose new value equals the old one | succeeds; nothing fires |
| AT-9 | `state-swap` on a never-set node | `nil` |
| AT-10 | claim with `(state-cas p nil me)`, release with `(state-cas p me nil)`, claim again with `nil` | `#t`, `#t`, `#t` |
| AT-11 | `(state-cas "a" nil "x")` where `a.b` exists and `a` was never set | `#t` |
| AT-12 | all three through a transparent link that is not writable | the read-only error; neither node changed |
| AT-13 | `(state-incr "n" 0.5)` on `1000000` | `1000000.5`, stored as `"1000000.5"` |

### 13.2 Locks (LK, L1 unless marked)

| ID | Situation | Result |
|---|---|---|
| LK-1 | `with-lock` returns the body's value; `locks-held` inside and outside | the hold shown inside; empty outside |
| LK-2 | X excludes X and S; S shares | as stated |
| LK-3 | 5 writers queue in order | granted in arrival order; deterministic traces identical across runs |
| LK-4 | readers hold; a writer queues; more readers arrive | the later readers wait behind the writer |
| LK-5 | a releaser re-acquires at once with a waiter queued | the waiter goes first |
| LK-6 | `:if-available` and `:timeout 0` with a waiter queued | neither barges; `:else` evaluated once; `TimeoutError` |
| LK-7 | X `a` then S `a.b`; S `a` then X `a.b`; X `a.b` then X `a` | counted, no manager call; `LockError("upgrade")`; granted when no other process holds under `a` |
| LK-8 | X `a.b` against S `a`, X `a`, S `a.b.c`, X `a.c`, X `a.bc`; a queued X on `a` against a later X `a.b` | the first three wait, the last two granted; the later X waits |
| LK-9 | `(list "b" "a" "a.x")` | one grant of `a` and `b`; `a.x` dropped |
| LK-10 | `" a..b "`; two chroots with the same relative key; a chrooted key against the same absolute key unchrooted | the key `a.b`; two locks; one lock (no `channels` segment) |
| LK-11 | a `/` key under a channel policy forbidding `/` | `PermissionError` |
| LK-13 | a user `defun with-lock`, a user `defmacro with-lock` | the user's runs; step counts equal today's |
| LK-14 | `:as g` for X grants on two keys in turn, and an S grant; a covered section nested in an X grant that reported `abandoned: #t`; `:if-available` taking `:else`; a chrooted `(list "b" "a")` grant | `{key mode generation abandoned}`; each X generation above every earlier one; the nested `g` has the covering hold's generation and `abandoned: #f`; `g` is `nil` in `:else`; `key` is `("a" "b")`, as `locks-held` shows it |
| LK-15 | `lock-query` from a chrooted program with another owner's holder on a `/` key | the holder counted, its pid left out |
| LK-16 | a `msg-send` to `"#lock"` and to every `#` key while a process waits on a lock | the waiter stays parked |
| LK-17 | the whole state_exec, Ariadne and pycvc suites | identical results and step counts |
| LK-18 | random S/X single- and multi-key requests on paths of depth 1-8 in a 3-ary tree, 10^5 operations with timeouts and kills | `check()` and `check_quiescent()` pass every step (no process `lock_waiting` on a non-queued ticket, mirrors, bases and the key cache equal to the manager's); the manager empty at the end (I9) |
| LK-19 | Q holds `a.y`; P requests `(list "a.x" "a.y")`; R then requests X `a`; Q releases | while P waits `a.x` is not held; P gets both keys in one grant, no `DeadlockError`; R after P |
| LK-20 | P inside `(with-lock "a.x" ...)` nests `(with-lock "a.y" ...)` with an X on `a` queued; the same with `:if-available`, and with `:timeout 0` | granted at once each time: no `DeadlockError`, no `:else`, no `TimeoutError` |
| LK-21 | an X on `a` releases with `[S a, S a, X a, S a]` queued | the two first S granted together; the X after both; the last S after the X |
| LK-22 | H holds S `a.b`; W waits for X `a.b`; R waits for S `a` behind it; W times out | R granted at once; W raises `TimeoutError` |
| LK-23 | pycvc: `register_intrinsics(env, ctx)`, drop `ctx`, then `(with-lock "k" 1)` | `1`; no use after free (ASan) |
| LK-24 | (L1b) a parent spawns k children, each running 1000 sections of `(state-incr "q.n")` and a read-modify-write of `q.m` under `(with-lock "q" ...)` | both exactly k x 1000; the keys under the parent's chroot |
| LK-25 | H holds S `k`; W queues X `k`; R queues S `k`; W times out | R granted at once |
| LK-26 | P holds X `a.b`, R holds S `a.c`; P queues for X `a`; R releases | P granted X `a` |
| LK-27 | P holds S `a.x`; Q's X on `a` queues; P nests X `a.y` | granted at once (the conversion rule); Q after P releases both |
| LK-28 | P holds X `a.b`; Q's X on `a` queues; P broadens to X `a` | P granted at once; Q after P |
| LK-30 | `(with-lock (list "a" "b") :timeout 0.5 ...)`; `a` held until 0.45 s, `b` throughout | `TimeoutError` at about 0.5 s; nothing ever held |
| LK-31 | (L1b) an Ariadne action's spawned child waits on a key a resident holds | `TimeoutError` after the action's 1 s default |
| LK-32 | H holds X `a`; W queues S `a.b`; R requests X `b`, then X `ab` | both granted at once; W still queued |
| LK-33 | P holds X `a` and nests `(with-lock (list "a.b" "a.c") :shared ...)` | covered: `locks-held` shows one hold, count 2; no manager operation |
| LK-34 | an Ariadne document holds X `x` (absolute `docs.A.x`); an unchrooted program requests X `docs` | it waits for the section, then excludes the document's locks until it releases |

### 13.3 Every way out (LX, L1 unless marked)

Each case runs with the process queued, and granted but not yet resumed.

| ID | Situation | Result |
|---|---|---|
| LX-1 | the body ends by value, `return`, `break`, a caught raise, an uncaught raise | released each time, not abandoned; outer `finally` blocks run after the release |
| LX-2 | every kill and budget of 5.6, of an X holder inside its body and of an S holder | released; the next X grant on the key sees `abandoned: #t` once, an X on its parent does not; the S release clean; the manager empty |
| LX-3 | grant, then kill the grantee before it steps; the same after an abandonment of the key | released clean, the next waiter granted; the next X to enter its body sees `abandoned: #t` |
| LX-4 | grant, then a pending signal handler that raises before the `lock_enter` step; one that requests the granted key | the unwind releases the granted hold; `LockError("handler")` |
| LX-5 | (L3) a timeout and a kill, each racing a grant made on another home, before and after the wake is applied | one outcome: the timeout adopts the grant; the kill releases it; the late wake ignored |
| LX-5a | a timeout or a kill racing a same-home grant before the waiter steps | one outcome; `check()` passes |
| LX-6 | `on_complete` that reaps; `terminate_process` with a hold left by a handler | locks free before `on_complete`; `locks.leaked_at_exit` counted |
| LX-7 | the scheduler destroyed with holders and waiters; (L3) with a wake to one of its waiters already in its port | every hold released and ticket cancelled in one call; other homes' waiters granted |
| LX-8 | a multi-key request whose second key would raise (upgrade, the handler rule); an error injected into `step_lock_enter` and into `step_init` after the grant (test builds) | raised before the manager sees a key; `fail_step` releases the frame; nothing held |
| LX-10 | (L3) a grant to a waiter on another home, then the waiter killed and reaped before the wake is applied | the end releases the granted hold; the next waiter granted; the wake ignored |
| LX-11 | a wake for a ticket the process no longer waits on: adopted by its timeout, then a new request queued (same home; (L3) posted) | the stale wake ignored; the new request stays queued until its own grant |

### 13.4 Where waits are allowed, handlers, fork, snapshots (LW, L1)

| ID | Situation | Result |
|---|---|---|
| LW-1 | a contended `with-lock` in a nested evaluator, under `park_forbidden_scope`; `with-lock` in Ariadne's reactive lane | `LockError("cannot wait here")`; uncontended works; `:if-available` takes `:else`; `NameError` in the reactive lane |
| LW-1b | `(collections.map (lambda (x) (with-lock "k" x)) (list 1 2))` while another process holds `"k"`, which then releases | the process parks inside the map, is granted on release, and the map returns `(1 2)` |
| LW-2 | `with-lock` with no stepping process; a direct call `(%lock-wait KEYS MODE nil)` (the wait child's form, 8.5) on `"k"`, held by another process, at top level, as the `KEY` of a `with-lock`, and as its `:timeout` expression | `LockError("no process")`; `LockError` each time, nothing queued, held or mirrored |
| LW-3 | a signal handler requesting a key its program holds; a conflicting key; an unrelated key | `LockError("handler")`; the same; granted |
| LW-4 | a watch handler, the same three | the same |
| LW-5 | a paused holder; pausing a waiter | keeps its lock; `pause` fails |
| LW-6 | fork inside `with-lock`; fork outside, from pycvc (`ctx->pid` 0) and an Ariadne action (`ctx->pid` -1); a forked child at the document's teardown | `LockError("fork")`; a fork of the running process, the child with the parent's `owner`; reaped by `kill_owner` |
| LW-7 | a snapshot of a holder; of a waiter, restored in place, through a serialized round trip, and into another `Exec`, then a new `with-lock`; a restored waiter snapshotted again before its next step, and that snapshot restored; a waiter with `:timeout 10` snapshotted after 9 s and restored | refused, naming the keys; the waiter queues afresh at the tail (its re-applied `%lock-wait` runs with the token, in the step that re-applies it); the new `with-lock` uses the restoring scheduler; the twice-restored waiter queues afresh the same way, no `LockError`; the restored waiter times out about 1 s after the restore |
| LW-8 | `migrate` of an X holder (today's metadata-only move) | the source's kill releases it; the next X grant sees `abandoned: #t` |
| LW-9 | Ariadne: an action waits for a resident's lock; the action's default; an init script locking a key a resident holds (from L3), with no `:timeout`, with an explicit `:timeout 0.8`, and with `:if-available`; (from L3) a child the init script spawns, requesting the same key with `:timeout 0.8` | granted on release; `TimeoutError` after 1 s; the init raises `TimeoutError` at once both times (`lock_wait_allowed` false), no host-thread wait; `:else`; the child raises `TimeoutError` at once (it inherits `lock_wait_allowed` false), no host-thread wait |
| LW-10 | `with-lock` in a closure generator's body: uncontended, around a dropped `yield`, shared across fork; around `(next g)` | `LockError("generator")`, nothing held; the last works |
| LW-11 | pycvc: `pycvc.evaluator` with intrinsics evaluates `(with-lock "k" (state-set "x" 1))` | `TypeError("with-lock needs a scheduler")`; `x` never written |
| LW-12 | a restore of a stack holding a `lock_body` frame with a hold id (built by hand); one with an `:else` `lock_body` | the first refused; the second runs normally |

### 13.5 Deadlock (LD)

| ID | Situation | Result |
|---|---|---|
| LD-1 | two processes, opposite order | one `DeadlockError`, in the closer, catchable, its `finally` run; the other completes |
| LD-2 | a cycle of 8; a cycle through an S lock with several holders | detected; the cycle field lists it |
| LD-3 | two processes broadening against each other | one `DeadlockError`; the survivor granted when the victim unwinds |
| LD-4 | a long chain without a cycle | no false positive |
| LD-5 | (L3) a cycle between two `Exec`s on two threads | `DeadlockError` when the closing request blocks, no delay |
| LD-6 | (L3) two schedulers on one shared manager, stepped alternately by one thread in a fixed order; a cross-home cycle | the same victim every run |
| LD-7 | Q holds S `a.b.c`; P waits for X `a.b.c`; Q requests S `a` | granted at once (the conversion rule); P after Q releases both |
| LD-9 | H holds X `a.b`; V queues S `a`; W holds X `z` and queues X `a.c`; H requests X `z` | `DeadlockError` in H (H, W, V, through W's waiter edge to V) |
| LD-10 | P holds S `a.x`; R queues X `a`; Q queues S `a.q`; P requests X `a.q.w` | `DeadlockError` in P (P, Q, R): R is exempt for P, Q is not; the conversion rule's documented limit |

### 13.6 Priority (PR, PI; L2)

| ID | Situation | Result |
|---|---|---|
| PR-1 | self lower, back to the floor, better than the floor, out of range | `#t`, `#t`, `PermissionError`, `ValueError` |
| PR-2 | another: a spawned child, a forked child, the same owner and uid, another owner, an unknown pid, the same owner with another uid and not a descendant; `get-priority` on each | `#t`, `#t`, `#t`, `#f`, `#f`, `#f`; a dict for the first three, `nil` for the rest |
| PR-3 | raise a child above the caller's floor | `PermissionError` |
| PR-4 | `priority_rr`, a change | the next pick follows the new level; `check()` holds |
| PR-5 | `state_exec.policy: priority_rr`, then `app::exec_scheduler()` and `Exec(app)`; with the setting unset, `Exec(app, policy="priority_rr")` by keyword and `Exec(app, policy=pycvc.scheduling_policy.priority_rr)`; an unknown value | built with `priority_rr` each time; the unknown logged and `round_robin` used for the setting, `ValueError` for `Exec` |
| PR-6 | `round_robin` | `#t`; `get-priority` reports the policy; no inheritance work |
| PR-7 | `spawn` at priority -5 from a caller at floor 0: `round_robin`; `priority_rr`; an explicit host floor 0 | -5, floor -5; 0, logged and counted; the same |
| PR-8 | pycvc `Exec.set_priority`, `get_priority`, (L3) `lock_query` from another thread while `run()` pumps (TSan), and on an idle `Exec`; the coordinator's control; a registered Python function calling `get_priority` and `lock_query` from inside the program; a monitor thread polling `get_priority` and `lock_query` in a loop while the program terminates, is killed, and raises, 10^4 runs each; an async function's kernel calling `get_priority` on its `Exec` while the program is killed, and while it terminates; two `Exec`s on two threads whose registered functions call each other's `get_priority`; a kernel, and another thread, calling the same `Exec`'s `run()` during the quiesce | applied between steps, clean; at once when idle; applied; answered at once on the `run()` thread, no hang; every query answered or failed with `RuntimeError("Exec.run ended")`, none left waiting, `lock_query` never waiting, no hang (the GIL order, 7.4); the kernel answered by `run()` or acting directly during the quiesce, no hang; `RuntimeError("Exec.run is active on another thread")` in a caller whose target runs, no hang; today's re-entrancy `RuntimeError` in both, no hang, the first `run()` returns its result |
| PR-9 | the thread and process classes on each platform; Linux unprivileged, demoted then raised; Emscripten | reported, no exception; Linux raise false; Emscripten false |
| PR-10 | PR-1..PR-6 on both schedulers | identical |
| PR-11 | priority and floor carried in coordinator metadata and the codec; a target with floor 3 receives base 5 floor 0, and base 1 | floor 3; base 5 kept, base 1 clamped to 3 and logged |
| PI-1 | a low holder, a high waiter, a medium loop (`priority_rr`) | the holder boosted; the waiter granted within the holder's section; unboosted after |
| PI-2 | a chain of 3 | the boost propagates |
| PI-3 | the waiter times out; is killed | the boost withdrawn at once |
| PI-4 | `set-priority` on a boosted holder | the base changes; the effective priority stays at the boost |
| PI-5 | a snapshot and `inspect` of a boosted holder | the base, never the boost |
| PI-7 | (L3) a relevel post for a holder on another home arrives after it released | re-reads the manager; the holder at its base |
| PI-8 | W (-10) waits for H1, which waits for H2; W times out | H1 and H2 both back to their base |
| PI-9 | `priority_rr`: H (10) holds X `a.b`; V (10) queues X `a`; W (-10) queues S `a.c`; medium spinners | H and V boosted to -10 through W's waiter edge; V granted within H's section, W within V's |
| PI-10 | (L3) a round-robin home's child spawned at -20 waits behind a `priority_rr` holder at 10 | not boosted; no relevel posted |
| PI-11 | P (5) holds X `a.b`, R (5) holds S `a.c`; P queues for X `a`; W (-10) queues X `a`, then times out | P and R boosted to -10, then both back to 5 |

### 13.7 Threads (LT, L3)

| ID | Situation | Result |
|---|---|---|
| LT-1 | 2 `Exec`s on 2 Python threads, one hot key, 10^5 sections each (TSan) | counts exact; clean |
| LT-2 | a cross-thread grant to an `Exec` in `wait_for_work`; to a sync scheduler in `idle_wait` beside a 5 s sleeper; to a sync `run(N)` too short for the wait, then another `run(N)`; a sync `run(10^7)` left with only the waiter and no timer | woken when the wake is posted; the first `run(N)` returns on budget, the second applies the wake; `run(10^7)` returns at once, as with only message waiters (6.1), and the next `run()` applies the wake |
| LT-3 | an `Exec`'s holder killed while another `Exec` waits; an `Exec` destroyed with a wake in its port | the next waiter granted; nothing held by the destroyed `Exec`'s processes |
| LT-3b | a post in flight racing `~Exec` on another thread: `Exec` A's holder releases a key `Exec` B waits for while the last reference to B is dropped on a third thread, 10^4 times, under ASan and TSan, on `async_scheduler` and the sync scheduler | no use after free, no race; each post lands before the close or is dropped |
| LT-4 | fuzzed yields at the protocol points (`CVC_EXEC_FUZZ`), 1000 seeds; every 100 operations the drivers stop until every home has delivered and drained its port | no lost wakeup, no double grant, FIFO kept; `check()` after every operation; `check_quiescent()` at each stop and at the end of each run (no process `lock_waiting` on a non-queued ticket, mirrors, bases, key caches and inherited priorities equal to the manager's); the manager empty at the end (I9) |
| LT-5 | the model check: the protocol implemented literally, with keys `a`, `a.b`, `a.c`, `b` (protocol 7), and its mutations: on one home (L1: `round_robin`, no `set-priority`; L2: `priority_rr` with `set-priority`), and from L3 on two homes of 2-3 processes. The design-time model (`tools/locks_model/`, protocol 7) runs bounded, smaller versions of these configurations, and L4's group teardown, before the code exists | every invariant in every state; liveness under fairness; each mutation caught |
| LT-7 | `Exec(app, idle_timeout=0.5)`: one `Exec` holds a key for 1.5 s across an `await`, pumping; another waits with no timeout. The same waiter behind a sync scheduler on another thread, on the app's manager, whose holder steps once every 0.1 s for 2 s (`(sleep 0.1)` in a loop) inside one `run()`; and behind the same scheduler pumped only by a `step()` call every 0.1 s | granted at about 1.5 s; granted when the holder releases in the other two; never killed |
| LT-8 | `Exec` A spawns a holder and its `run()` returns; `Exec` B (`idle_timeout=0.5`) waits; the same with A pumped; A pumped with its holder parked on `msg-recv` beside a sibling that keeps stepping for 2 s | B killed at the limit, naming the key; granted when A's holder releases; B not killed while A steps (the rule is per home, 9.2), then killed at the limit once A's sibling ends |
| LT-8b | one `Exec` (`idle_timeout=0.5`): `(spawn "(with-lock \"k\" (msg-recv \"never\"))")`, `(sleep 0.1)`, then `(with-lock "k" 1)` with no timeout | the waiter killed at the limit, naming the key; `run()` returns |
| LT-10 | 8 `Exec`s on 8 threads running random single- and multi-key requests over 16 keys with timeouts, kills and `set-priority`, 10^5 operations (TSan); every 1000 operations the threads stop at a barrier until every `Exec` has drained its port | `check()` passes after every operation; clean; `check_quiescent()` at each barrier and at the end (no process `lock_waiting` on a non-queued ticket, every applying process's `inherited_priority` equal to the manager's, mirrors, bases and key caches equal to the manager's); the manager empty at the end (I9) |
| LT-11 | two `Exec`s, same owner: A's pid 2 holds X `"/k"`; B's pid 3 runs `(lock-query "/k")`; then a cycle across the two `Exec`s closed by B | the hold counted with its pid left out; B's `DeadlockError` `cycle` shows A's pids as `nil` and B's own |

### 13.8 Groups (LG, L4)

| ID | Situation | Result |
|---|---|---|
| LG-1 | the td GR-1 oracle with locks inside each group | each group equal to its solo run |
| LG-2 | a cross-group grant under kill and timeout races, fork-join and continuous; a lock post racing the target group's destruction, 10^4 times (ASan, TSan); a grant to a waiter of a group in teardown, between its port's close and its kills | one outcome each; nothing lost; no use after free, no race: each post's `wake(g)` completes before the close or the post is dropped; that waiter's kill releases the grant, and the next waiter is granted |
| LG-3 | `CVC_EXEC_FORCE_MOVE` on holders; a waiter's move | results unchanged; the move refused |
| LG-4 | a cross-group `set-priority`, same owner and other owner | applied at the next step boundary; `#f` |
| LG-5 | group weights: 2 busy groups at 0 and 10 at threads 1; 4 at 0, 0, 10, 10 on 2 workers | `extra.share_ratio` within 20% of the CFS ratio |
| LG-6 | deterministic mode, threads 4 | identical grant order across runs |
| LG-7 | LD-6's cycle across two groups, deterministic mode | the same victim every run |

### 13.9 Migration (LM, L7)

| ID | Situation | Result |
|---|---|---|
| LM-1 | `migrate` of a holder; with `release_locks`; with it and the target failing before it acknowledges | refused on the pumping thread; released by the paused control entry, abandoned; the resumed source raises `LockError("lost")` before its next step |
| LM-2 | see PR-11 (L2) | - |
| LM-3 | lease rebind: ack, nack, TTL expiry | resumed; `LockError("lost")`; the same |
| LM-4 | the holder's host fails | a new token, abandoned set, the next waiter granted |
| LM-5 | the authority host fails | holders' grants invalid; remote waiters get `LockError("lost")` |
| LM-6 | a fenced write with a stale token | rejected by the shard |

## 14. Risks and open questions

### 14.1 Risks

1. **Evaluator surgery** in code the exceptions work just hardened. *Mitigation:* the frames follow
   `try`'s shape; LX and the exceptions suite on both schedulers; parking reuses `apply_native`.
2. **Concurrency.** *Mitigation:* one mutex over all lock state, posts that carry no outcome, the
   protocol model-checked literally (LT-5) and its invariants checked after every operation
   (LT-4, LT-10, TSan).
3. **Contention on the manager mutex** with many threads and short sections. *Mitigation:*
   `lock.cross_thread` decides on data; sharding by subtree is the fallback (3.3).
4. **Advisory surprises**: a user may expect `with-lock` to stop a Python writer. *Mitigation:* the
   docs, `lock-query`, and L5 if asked.
5. **Strict-priority starvation outside locks** remains; the group weights are the one new tool.
6. **Conservative corners**: a multi-key request waits for all its keys at once, and the pairwise
   conversion rule raises `DeadlockError` in a three-process chain (LD-10). Documented; neither
   loses safety or hangs.

### 14.2 Open questions (each with the proposed answer)

1. **Ariadne's default lock timeout**: 1.0 s for actions and their children
   (`ariadne.exec.lock_timeout`), 0 for init scripts, none for residents.
2. **Pool weighting by priority** (td Stage 2): on under a priority policy, once
   `prio.groups_weighted` passes.
3. **L5 and L6**: built only on a concrete request.
4. **The app manager's mode**: `shared` when threads are compiled in and `state_exec.threads > 0` or
   pycvc is loaded, else `single`, chosen at the first `app::lock_manager()` call. The alternative
   is `shared` whenever the build has threads (65-90 ns instead of 25-50 ns per section).
5. **Default policy for Ariadne, pycvc and the coordinator**: keep `round_robin`, since
   `priority_rr` changes step interleaving and step-count tests; hosts opt in.
6. **Inheritance, `lock-query` and `DeadlockError` across owners** (a reading of Q5): inheritance may
   raise another owner's holder, only while it blocks the waiter and only to the waiter's
   priority; `lock-query` counts other owners' holders without pids; a cycle shows their pids as
   `nil`. None gives control of another owner's process. Pids of another scheduler are left out
   and shown as `nil` the same way, whatever their owner, since they repeat the viewer's own
   (LT-11).
7. **The new names**, for approval as Q14's were: `with-lock`, `locks-held`, `lock-query`,
   `set-priority`, `get-priority`, `state-incr`, `state-cas`, `state-swap`; `LockError`,
   `DeadlockError`, `PermissionError`; `state_exec.policy`, `state_exec.locks.threading`,
   `state_exec.groups.priority_weights`, `state_exec.process_class`, `state_exec.worker_class`,
   `ariadne.exec.lock_timeout`; and the pycvc and C++ host API: `Exec(policy=, idle_timeout=)`,
   `Exec.set_priority`, `Exec.get_priority`, `Exec.lock_query`, `pycvc.set_thread_class`;
   `execute_options.priority_floor`, `resource_policy::priority_floor`,
   `pool_options::worker_class`, `exec_group::options::thread_class`,
   `lock_manager::options::inheritance` (test-only), `intrinsics_context::lock_wait_allowed`.
8. **Bringing td work forward**: B28 whole in L1 and B27's single-scheduler part in L1b, each with td
   section 5's compatibility scan; `post_control` for `set_priority` in L2 if Stage 0 has not
   landed. The threaded design records it.
9. **When to shard the manager**: only if `lock.cross_thread` misses its disjoint gate at t = 4 on
   Windows or Linux, or `mt.groups.locks` its 5% gate; then by subtree (3.3), with its own review
   and model check.
