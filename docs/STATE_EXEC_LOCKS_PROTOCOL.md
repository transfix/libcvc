# state_exec locks: the protocol

Status: specification, 2026-10-27. Nothing here is built yet. It is the exact protocol behind
[`docs/STATE_EXEC_LOCKS.md`](STATE_EXEC_LOCKS.md) (the design), section 8.6. `td` is
[`docs/STATE_EXEC_THREADED.md`](STATE_EXEC_THREADED.md); `Qn` are its decisions (td 9.2). A model
checker implements it literally (test LT-5), and `lock_manager::check()` asserts the invariants
section 6 lists for it after every operation in test builds.

The protocol is the simplified core the lead engineer chose after two review rounds of the
intention-mark design (100 confirmed findings, 42 major, most of them holes in kept marks, chains,
continuations, pass-on, shard lock order and the lazy check). Its whole state is one table under
one mutex, every request is granted all at once or not at all, and a waiter holds nothing new.

## 0. Conventions

- **Actors.** One *manager* per app (or a scheduler's private one) and one *home* per scheduler
  attached to it. A home runs on the thread that pumps its scheduler. A pycvc `Exec` with no
  `run()` active has no pumping thread: its home's thread is then whichever thread holds the
  `Exec`'s control mutex, which stands in for the pump token (design 7.4, 9.1).
- **Manager operations** (section 4) are atomic. In shared mode each runs under the manager's one
  mutex `M.mu`; in single mode there is no mutex and only one thread calls the manager. Each is
  called by a home on that home's thread; the operation calls that home `C`, the caller. Each
  returns a result and a list `N` of notifications. The caller delivers `N` after the operation
  returns, with `M.mu` released (4.14). No manager operation calls into a scheduler, a port, a
  process or the tree.
- **Home actions** (section 5) run on the home's thread, between or inside its scheduler's steps.
  They read and write the home's private state (its processes, their mirrors, frames and wait
  fields) without a lock. Only the home's thread touches its private state.
- **Posts** are the only way one home's action reaches another home: `wake` and `relevel`,
  appended to the target home's port and applied by that home between steps (5.9). Both are
  hints: neither carries an outcome or a value.
- Sets and maps are mathematical. The implementation's structures (2.4) must be observationally
  equal to the definitions here, which `check()` verifies.
- `INF` is `INT_MAX`. A priority is better when it is lower.

## 1. Keys, requests and conflict

```
Key            = non-empty tuple of segments (non-empty strings with no '.' and no '#'); absolute
prefix(a, b)   = len(a) <= len(b) and b[0:len(a)] == a      # a is b, or an ancestor of b
overlap(a, b)  = prefix(a, b) or prefix(b, a)
Mode           = S | X
compat(m, n)   = (m == S and n == S)
Request        = { keys: set of Key, mode: Mode }           # canonical (below)
conflicts(A, B) = not compat(A.mode, B.mode)
                  and exists a in A.keys, b in B.keys: overlap(a, b)
    # A and B are anything with keys and a mode: a request, a hold, a ticket
under(h, k)    = exists hk in h.keys: prefix(hk, k)          # hold h's keys include k or an ancestor
```

**Canonical requests.** The home resolves each key against the caller's chroot (design 4.1),
splits it into segments, sorts the keys by their segment tuples, and drops every key that has an
earlier key as a prefix (an equal key or an ancestor). The result is non-empty, and no key in it is
a prefix of another. The mode applies to every key.

**Conflict** is decided on segment boundaries: `a.b` overlaps `a` and `a.b.c`, not `a.bc`. Two
shared requests never conflict. Ownership never enters `conflicts`; the rules below exclude a
requester's own holds where they apply.

## 2. State

### 2.1 The manager

```
M.threading             single | shared            # fixed at construction
M.mu                    a mutex (shared mode only)
M.options.inheritance   bool, default true         # test-only switch (design 6.2); fixed at
                                                   # construction

M.homes   : HomeId -> Home
Home      = { prio: bool,                 # the home applies priorities (from L2: its scheduler
                                          # uses priority or priority_rr; in L1 false, 4.4)
              port: shared_ptr<Port>,     # 2.3
              progress: shared_ptr<atomic time> }  # the home's last_progress (design 8.4),
                                          # shared with it; read only by blocking_sinks (4.13)

Pid       = (HomeId, int)                 # a process; pids repeat across homes, pairs do not
M.recs    : Pid -> Rec                    # one per process with a hold or a ticket
Rec       = { owner: string,              # the process's owner (Q5), for reads only
              base: int,                  # mirror of process::priority (5.8)
              holds: set of HoldId,
              ticket: TicketId | none,
              inh: int = INF }            # the inherited priority (3.3)

M.holds   : HoldId -> Hold
Hold      = { pid: Pid, keys, mode,
              gen: int,                   # the grant's generation (4.2)
              body_started: bool,         # the holder's body has begun (5.3)
              taken: set of Key }         # abandoned flags this grant consumed (4.2)

M.tickets : TicketId -> Ticket
Ticket    = { pid: Pid, keys, mode,
              seq: int,                   # = its TicketId; the FIFO order
              state: queued | granted | cancelled | failed,
              hold: HoldId | none,        # set when granted
              error: value | none }       # set when failed

M.queue     : the set of TicketId whose state is queued, ordered by seq
M.abandoned : set of Key
M.next_seq, M.next_hold, M.next_gen : int, all starting at 0
M.cache_refs : HomeId -> multiset of Key  # the key-cache references of each attached home, one
                                          # entry per attached home (4.4, 4.11, 4.15)
```

Derived:

```
applies(P) = P.home in M.homes and M.homes[P.home].prio         # P's home applies priorities
lends(P)   = M.options.inheritance and applies(P)               # P's priority can boost others
lending    = exists H in M.homes: M.options.inheritance and M.homes[H].prio
```

`applies` and `lends` are total: for a process whose home has been detached (`detach_home`
deletes the home before `finish`, 4.11) both are false, short-circuit, and no deleted home is
read.

### 2.2 A home's private state

For each process `p` of home `H`, with `P = (H, p.pid)`:

```
p.status               the scheduler's (ready, running, waiting, paused, finished, ...)
p.priority             the base priority; written only by set-priority (5.8) and by hosts
p.inherited_priority   int = INF; written only by applying a relevel (5.9)
p.locks                list of MHold in acquisition order: the mirror of P's completed holds
MHold                  = { id: HoldId, keys, mode, count: int >= 1, gen, abandoned: bool,
                           in_handler: bool }
p.lock_ticket          TicketId | none: the ticket P is parked on; set only while it is queued,
                       or granted or cancelled with the wake not yet applied
p.lock_waiting         bool: parked on p.lock_ticket
p.lock_deadline        time | none: in the process table's timer heap while lock_waiting
p.in_handler           bool: in_signal_handler or in_watch_handler
```

Frames (design 8.5): `lock_enter { outcome: none | granted(HoldId) | error }` (a request being
waited for; the outcome is set when the home collects it with `resume`) and
`lock_body { hold: HoldId | none }` (a section; `none` for an `:if-available` request's `:else`).

### 2.3 Ports and notifications

```
Port         = { mu: mutex,                    # the port's own, never a scheduler's
                 queue: FIFO of Post,          # guarded by mu
                 closed: bool,                 # guarded by mu; written only by close()
                 pending: atomic bool,         # set by post, cleared by drain
                 wake: shared_ptr<wakeup> }    # the scheduler's wakeup, shared with it
Post         = wake { pid: int, tid: TicketId } | relevel { pid: int }
Notification = { home: HomeId, port: shared_ptr<Port>, post: Post }  # port captured under M.mu
port.post(x) = lock mu
               if closed: unlock; return false
               append x; pending = true; wake->notify()
               unlock; return true
port.close() = lock mu; closed = true; unlock          # only as its home is destroyed (5.7)
port.drain() = lock mu; take the queue; pending = false; unlock; apply the posts taken (5.9)
```

- **Lifetimes.** The port is a `shared_ptr<lock_port>` held by its home and by every
  notification captured for it, so a poster keeps it alive. The scheduler creates `wake` eagerly,
  at construction, and shares it with the port; it is the wakeup the scheduler blocks on
  (`async_scheduler::wait_for_work`, and the sync scheduler's `idle_wait`, which no longer creates
  its wakeup lazily). `wait_for_work` tests `pending` before it blocks, as it tests its ingress.
  The sync scheduler has no ingress, and its `idle_wait` sleeps in slices of at most 10 ms: it
  tests `pending` before each slice and leaves its wait when `pending` is set (design 8.7). A
  post touches only the port and the wakeup it shares: never a scheduler member (not its ingress
  mutex or flag, nor `notify_work`), so it is safe while the scheduler is being destroyed. From
  L4 a group's scheduler shares a wakeup whose `notify()` is the pool's `wake(g)` (design 9.2).
  A post calls it only inside `mu` and with the port open, and a group's teardown closes the port
  as soon as it holds the group's token, before it kills the group's processes (5.7, td 4.2), so
  a post never reaches a dying group either.
- **Post and close exclude each other.** The closed test, the append and the signal are one
  critical section under `mu`, and `closed` is written only by `close()`, under the same `mu`.
  `~lock_home` closes the port first, before `detach_home` and before any scheduler member is
  destroyed (5.7). A post racing it either completes before the close (its entry is discarded
  with the queue) or sees `closed` and is dropped; none lands after.
- A home applies its port's posts in FIFO order, between steps (5.9). A post is dropped only to a
  closed port, which happens only while the home is destroyed: by `~lock_home`, or from L4 first by
  its group's teardown, once it holds the group's token (5.7). In both cases no pump runs from the
  close until the home's processes are ended, by `detach_home` or, for a group, by its teardown's
  kills, so a dropped post concerns only a process that never steps again.

### 2.4 The implementation's structures

The definitions in sections 1 and 3 are the reference. The implementation keeps:

- **A path trie.** One node per key that has a hold, a queued ticket, an abandoned flag, or a
  key-cache reference in `M.cache_refs` (4.15, design 8.4), and one per proper prefix of such a
  key (the path from the root, through which the node is reached). Per node `n` for key `k`:
  - `here[m]`: the number of holds of mode `m` with `k` in their keys, and the list of them;
  - `sub[m]`: the sum of `here[m]` over `k` and every descendant;
  - `wq`: the queued tickets with `k` in their keys; `subw`: the sum of `|wq|` over `k` and every
    descendant;
  - `abandoned`: `k in M.abandoned`;
  - `refs`: the number of times `k` is in `M.cache_refs[H]`, over every home `H`.
- **Key caches.** Each home's cache maps a key as written to its trie node, which the entry's
  reference keeps alive, so a hit needs no trie walk from the root. The cache itself is the
  home's; only the references are manager state, changed under `M.mu` by the operation that
  makes the miss or the eviction (4.15).
- **Free lists.** Recs, holds, tickets, their key storage and the nodes of the tables that index
  them come from per-manager free lists, so an uncontended section allocates nothing after a
  process's first lock (design 8.8). A deleted rec goes back to its free list at once: none is
  kept for a process with no hold and no ticket, so a process that ends holding nothing leaves
  nothing in the manager.
- **Conflict enumeration** for a request `R`: for each key `k` of `R`, walk the root to `k`, reading
  each node's `here` lists (ancestors and `k` itself); then descend from `k` only into children
  whose `sub` counts a mode that conflicts with `R.mode` (only `X` for an S request; both for an X
  request). Waiters are found the same way through `wq` and `subw`. The cost is the depth plus
  the holds and tickets that actually overlap. An empty `sub` and `subw` on the path, the common
  uncontended case, costs only the walk.
- **The queue** as an intrusive list in `seq` order, and each ticket linked into the `wq` of each
  of its keys.
- **Nodes** are freed when all their counts are zero, their lists empty, their flag clear, their
  `refs` zero and they have no child node. Neither the abandoned flag nor `refs` is counted in a
  subtree count, so an ancestor of a flagged or cached node can have every count zero; it is kept
  as long as it has a child. Freeing a node may leave its parent childless, so the check repeats
  up the path.

`check()` (test builds) recomputes every count from `M.holds`, `M.queue`, `M.abandoned` and
`M.cache_refs` and compares (I4).

## 3. Derived relations

All are pure functions of the manager's state at the moment they are evaluated.

### 3.1 Blocking and the grant rule

```
blk_hold(P, R)      = { h in M.holds : h.pid != P and conflicts(R, h) }

exempt(P, U)        = exists h in M.holds: h.pid == P and conflicts(U, h)
    # U is a queued ticket that conflicts with a hold P already has, so U cannot be granted
    # before P releases that hold

blk_wait(P, R, s)   = { U in M.queue : U.seq < s and U.pid != P and conflicts(R, U)
                                       and not exempt(P, U) }

grantable(P, R, s)  = blk_hold(P, R) == {} and blk_wait(P, R, s) == {}

grantable(T)        = grantable(T.pid, T, T.seq)          # a queued ticket
grantable_new(P, R) = grantable(P, R, INF)                # a request not yet queued: every
                                                           # queued ticket is earlier
```

In words: a request is granted when no other process holds anything that conflicts with it, and
no earlier still-queued request of another process conflicts with it, except an earlier request
that itself conflicts with a hold the requester already has.

**Why the exemption (the conversion rule).** Let `P` hold `h`, let an earlier queued `U` (another
process's) conflict with `h`, and let `P`, inside `h`'s section, ask for `R` that conflicts with
`U`. Strict FIFO would make `P` wait for `U` while `U` waits for `h`: a deadlock built by the queue
rule alone. Every `with-lock` is scoped, and `R` is asked for inside `h`'s body, so `R`'s hold is
released before `h` is. `U` cannot be granted before `h` is released, so granting `R` first delays
`U` not at all, and refusing would only raise `DeadlockError`. This covers a nested section on a
sibling key (an S `a.x` held, an X `a` queued by another, a nested `a.y`) and broadening (X on `a`
asked for while holding X `a.b`), tests LK-20, LK-27, LK-28 and LD-7. The exemption is decided per
pair from `P`'s holds, which do not change while `P` waits, so it is stable (Lemma C). It does not
extend to chains: an earlier `Q` that is blocked by `U` but does not itself conflict with any hold
of `P` still blocks `P`, and if that closes a cycle `P` gets `DeadlockError` (LD-10). Extending it
to reachability would make grantability depend on distant parts of the graph and lose Lemma B.

### 3.2 The wait-for graph

```
succ(P) = if P in M.recs and M.recs[P].ticket = t and M.tickets[t].state == queued:
              { h.pid : h in blk_hold(P, T) } union { U.pid : U in blk_wait(P, T, T.seq) }
              where T = M.tickets[t]
          else {}
```

The graph has an edge `P -> Q` for every `Q` in `succ(P)`. Only a process with a queued ticket has
edges out. The edges are exactly the blockers of the grant rule, so `grantable(T)` holds exactly
when `T.pid` has no edges out.

```
reach(S)  = every process reachable from a process of S by one or more edges, plus S
anc8(X)   = { Y != X : some path Y -> ... -> X has 1 to 8 edges }
```

### 3.3 Inherited and effective priority

```
inh_formula(X) = min({ M.recs[Y].base : Y in anc8(X), lends(Y) } union { INF })
                 if lending, else INF
eff(p)         = min(p.priority, p.inherited_priority)  if p's scheduler uses a priority policy
                 p.priority                              otherwise
```

A process inherits the best base priority among the processes it blocks, directly or through up
to 8 edges. The graph is acyclic (I7), so `X` is never in `anc8(X)`. A process on a round-robin
home lends nothing (`lends` false), but a priority it inherits still passes through it: `anc8` is
over every path. `inh_formula` is the definition. It is not min over direct predecessors `Y` of
`eff(Y)`: a round-robin `Y` lends nothing although a priority it inherits passes through it, and
the 8-edge cap bounds the paths, so neither a direct-set recursion nor a fixpoint over effective
priorities may stand in for it in the implementation or in `check()`.

## 4. Manager operations

`with M:` means: in shared mode lock `M.mu` for the block; in single mode nothing. Every operation
below is one such block. `N` starts empty in each.

### 4.1 Helpers (inside an operation)

```
ensure_rec(P, info):                        # info = { owner, base } from the home (5.1)
    if P not in M.recs: M.recs[P] = Rec{ owner: info.owner, base: info.base,
                                         holds: {}, ticket: none, inh: INF }

new_hold(P, R, started) -> HoldId:
    M.next_hold += 1; id = M.next_hold
    if R.mode == X:
        M.next_gen += 1; gen = M.next_gen
        taken = R.keys intersect M.abandoned; M.abandoned -= taken
    else:
        gen = M.next_gen; taken = {}
    M.holds[id] = Hold{ pid: P, keys: R.keys, mode: R.mode, gen, body_started: started, taken }
    M.recs[P].holds += { id }
    return id

drop_hold(id, abandon) -> set of Key:       # returns the hold's keys, the region to re-examine
    h = M.holds[id]
    if not h.body_started:
        M.abandoned += h.taken              # a report its holder never saw goes back
    else if abandon and h.mode == X:
        M.abandoned += h.keys               # the section may be half written
    M.recs[h.pid].holds -= { id }
    delete M.holds[id]
    return h.keys

enqueue(T):  M.queue += { T.seq }           # and link T into the wq of each of its keys
dequeue(T):  M.queue -= { T.seq }           # and unlink it

maybe_drop_rec(P):
    if P in M.recs and M.recs[P].holds == {} and M.recs[P].ticket == none:
        delete M.recs[P]
```

### 4.2 `re_examine(region)`: grant every waiter that became grantable

```
re_examine(region) -> set of Pid:
    G = {}
    for t in M.queue in ascending seq order, taken as a snapshot of the queue on entry:
        T = M.tickets[t]
        if T.state != queued: continue
        if not exists k in T.keys, r in region: overlap(k, r): continue
        if grantable(T):
            dequeue(T)
            T.hold = new_hold(T.pid, T, started = false)
            T.state = granted
            N += wake(T.pid, T.seq) to T.pid.home     # a Notification (2.3)
            G += { T.pid }
    return G
```

- Only tickets that overlap the region can have become grantable (Lemma D), so the rest are
  skipped.
- Each test sees the grants made earlier in the same pass. Granting in `seq` order makes the
  notifications deterministic. The set granted does not depend on the order (Lemma D).
- A grant only adds a hold and removes a queued ticket. By Lemma A it adds no edge to the graph,
  and the ticket it removes now conflicts as a hold with every waiter it conflicted with as a
  ticket, so a grant never makes another waiter grantable. One pass is complete.

### 4.3 `finish(region, seeds, skip)`: the tail of every mutating operation

```
finish(region, seeds, skip):
    G = re_examine(region)
    for P in seeds union G: maybe_drop_rec(P)
    relevel_all(skip)
```

`relevel_all` is the inheritance recompute:

```
relevel_all(skip):
    for X in (the recs at the start of this operation) union M.recs:
        old = X's inh at the start of the operation (INF if it had no rec)
        new = (X in M.recs) ? inh_formula(X) : INF
        if X in M.recs: M.recs[X].inh = new
        if new != old and X not in skip and applies(X):
            N += relevel(X.pid) to X.home
```

The test is evaluated left to right and stops at the first false term, so `skip` (which holds every
process the operation ended, among them all those of a home `detach_home` just deleted) is tested
before `applies`; `applies` is total in any case (2.1).

**Implementation note (equivalent, cheaper).** An inherited priority changes only for a process
within 8 edges downstream of an edge that appeared or disappeared, or of a base that changed. The
implementation computes `A`, the processes reachable within 8 edges from the operation's seeds
in the graph before the change, plus the same in the graph after it, plus the seeds themselves,
and recomputes only `A`, each by a reverse breadth-first search of depth 8. The seeds are the
process whose request, holds or base changed, and the processes granted; for `end` and
`detach_home` they are the ended processes (plus the processes granted). These seeds are not
`finish`'s `seeds` argument (which is `{}` for `end` and `detach_home`); that argument is used only
for `maybe_drop_rec`. A process of `A` with no rec any more has no `inh` to set and is otherwise
treated as in the reference loop (`new = INF`, same `skip` and `applies` test).
`lending` before and after the operation decides the rest:

- false before and after: it does nothing (every `inh` is already `INF`, I8);
- true before, false after (only `detach_home` of the last attached home with a priority policy,
  since `M.options` is fixed at construction): it sets every rec's `inh` to `INF` and posts
  nothing, since no attached home then applies priorities;
- false before, true after (only `attach_home` of the first home with a priority policy): it runs
  the reference loop over every rec;
- true before and after: the set `A` above.

The reference is the loop above; `check()` compares every `inh` with `inh_formula` (I8).

**Coalescing.** An implementation may keep at most one `relevel` per process in a port (a flag in
the rec, cleared by the apply under `M.mu`), since applying one reads the current value (5.9).

### 4.4 `attach_home(H, prio, port, progress)` and the manager's construction

```
attach_home(H, prio, port, progress):
    with M:
        require H not in M.homes
        M.homes[H] = Home{ prio, port, progress }
        M.cache_refs[H] = {}                # empty; detach_home deletes it (4.11)
        relevel_all({})                     # lending may have become true
        return ok, N
```

Before phase L3 a manager accepts one home; attaching a second is refused (design 8.4). From L3,
attaching a home that may be pumped on another thread to a single-mode manager is refused
(design 8.3). In phase L1 every home attaches with `prio = false`, whatever its scheduler's
policy, so `lending` is false and every `inh` stays `INF` (I8 holds trivially); from L2 a home
attaches with `prio` true exactly when its scheduler uses `priority` or `priority_rr` (design 12).

### 4.5 `try_acquire(P, info, R, refs)`: the fast path

```
try_acquire(P, info, R, refs) -> granted(id, gen, abandoned) | busy:
    with M:
        require P not in M.recs or M.recs[P].ticket == none
        apply_refs(P.home, refs)                # the home's cache misses and evictions (4.15)
        if not grantable_new(P, R): return busy, []
        ensure_rec(P, info)
        id = new_hold(P, R, started = true)
        h = M.holds[id]
        return granted(id, h.gen, h.taken != {}), []
```

- **No notifications.** By Lemma A the new hold adds no edge, so no inherited priority changes and
  nothing becomes grantable.
- `started = true`: the home enters the body in the same step (5.1), so there is no window in
  which the process holds the lock without its body having begun.
- `busy` changes no lock state: only `refs` (4.15), which no grant, edge or count of 2.4 other
  than `refs` depends on. A `busy` request that must not wait (`:if-available`, `:timeout 0`, a
  zero default timeout, a context that forbids lock waits) therefore never barges and never
  queues.

### 4.6 `block(P, info, R, refs)`: queue, check for deadlock, inherit

```
block(P, info, R, refs) -> (tid, state):
    with M:
        require P not in M.recs or M.recs[P].ticket == none
        apply_refs(P.home, refs)                      # usually none: try_acquire just cached R
        ensure_rec(P, info)
        M.next_seq += 1; s = M.next_seq
        if grantable_new(P, R):                       # the state changed since try_acquire
            id = new_hold(P, R, started = false)
            M.tickets[s] = Ticket{ P, R.keys, R.mode, seq: s, state: granted, hold: id }
            M.recs[P].ticket = s
            return (s, granted), []
        targets = { h.pid : h in blk_hold(P, R) } union { U.pid : U in blk_wait(P, R, INF) }
        if P in reach(targets):
            cycle = deadlock_cycle(P, R, targets)     # below
            M.tickets[s] = Ticket{ P, R.keys, R.mode, seq: s, state: failed,
                                   error: DeadlockError(cycle) }
            M.recs[P].ticket = s
            return (s, failed), []
        T = Ticket{ P, R.keys, R.mode, seq: s, state: queued }
        M.tickets[s] = T; M.recs[P].ticket = s; enqueue(T)
        relevel_all({})                               # P has edges out now
        return (s, queued), N
```

- **Exact deadlock detection.** The test is made under the one mutex over every home's holds and
  tickets, so it sees the whole graph, across homes and threads. Edges are added only here
  (Lemma B), so a block that does not close a cycle leaves the graph acyclic, and one that would
  close a cycle never queues. There is no hop cap on the search, no lazy check and no timer for
  it. Its cost is bounded by the number of queued tickets.
- **`deadlock_cycle`.** A depth-first search from `targets`, each successor list taken in
  ascending `(home, pid)` order, stopping at the first path back to `P`. The cycle is the list of
  `{pid key}` for each edge `A -> B` on it, starting with `P`: `pid` is `A`'s pid when `A` shares
  `P`'s pid space and owner, and `nil` otherwise. A process shares `P`'s pid space when it is on
  `P`'s home; from L4, on any home of `P`'s pool, whose pids are pool-wide (td B32). Pids repeat
  across schedulers (each pycvc `Exec` numbers its own), so another scheduler's pid would name a
  process of `P`'s own. The owner test is Q5's. `key` is the first key, in canonical order, of
  `A`'s request that overlaps the hold or ticket of `B` that makes the edge.
- **The victim is the requester that closes the cycle**: deterministic for a deterministic
  arrival order.
- A granted or failed ticket is returned without parking (5.2); the home collects it at once with
  `resume` (4.7), and the `lock_enter` step acts on it, as for a ticket granted later.

### 4.7 `resume(P, tid)`: collect the outcome

```
resume(P, tid) -> outcome:
    with M:
        T = M.tickets[tid]; require T.pid == P and M.recs[P].ticket == tid
        if T.state == queued: return queued, []            # unreachable (I6); nothing changes
        if T.state == granted:
            h = M.holds[T.hold]
            out = granted(T.hold, h.gen, h.taken != {})
        else if T.state == cancelled:
            out = TimeoutError
        else:                                               # failed
            out = T.error
        delete M.tickets[tid]; M.recs[P].ticket = none
        maybe_drop_rec(P)
        return out, []
```

- The outcome is read from the ticket, never from a delivered value: there is no channel and no
  `#` key, and a post carries no outcome. The home calls `resume` when it wakes the process (5.9)
  or right after a `block` that did not queue (5.2), and keeps the outcome in the `lock_enter`
  frame until that frame's step (5.3).
- A granted hold joins the process's mirror here, before any signal or watch handler can run in
  the process, so the handler rule sees it as the program's hold (5.1), and a handler's own
  requests never meet a ticket still pending (I14).
- A non-queued ticket has no edges, so `resume` changes no inherited priority (and a dropped rec
  had no holds, so no edges in either).

### 4.8 `start_body(P, id)`: the body begins

```
start_body(P, id):
    with M:
        h = M.holds.get(id); require h is not none and h.pid == P
        h.body_started = true
        return []
```

Called by the `lock_enter` step that turns the frame into `lock_body` (5.3). The fast path sets
the flag in `try_acquire` instead. Between the grant and this call the process may be killed, and
`end` then releases the hold clean, returning any abandoned flag it took (I11).

### 4.9 `cancel(P, tid)`: a timeout

```
cancel(P, tid) -> state | none:
    with M:
        T = M.tickets.get(tid)
        if T is none or T.pid != P: return none, []
        if T.state != queued: return T.state, []       # granted: the waiter adopts the grant
        dequeue(T); T.state = cancelled
        finish(region = T.keys, seeds = { P }, skip = {})
        return cancelled, N
```

The ticket stays, `cancelled`, until `resume` collects it. Re-examining `T.keys` lets in the
waiters that waited only behind `T`, by FIFO (LK-22, LK-25). The same call withdraws a queued
ticket in the one defensive path where a `lock_enter` frame unwinds while its ticket is still
queued (5.4).

### 4.10 `release(P, id)`: a section ends

```
release(P, id):
    with M:
        h = M.holds.get(id)
        if h is none or h.pid != P: return []                   # the owner check
        region = drop_hold(id, abandon = false)
        finish(region, seeds = { P }, skip = {})
        return N
```

Called by the body's completion and by every unwinder (5.4). A release is never abandoned:
unwinding means the program chose to leave the section.

### 4.11 `end(P, abandon)` and `detach_home(H)`: a process or a scheduler ends

```
end_locked(P, abandon) -> set of Key:       # inside an operation
    rec = M.recs.get(P); if rec is none: return {}
    region = {}
    if rec.ticket != none:
        T = M.tickets[rec.ticket]
        if T.state == queued: dequeue(T); region += T.keys
        # granted: its hold is in rec.holds and is dropped below, unstarted, so clean
        delete M.tickets[rec.ticket]; rec.ticket = none
    for id in rec.holds: region += drop_hold(id, abandon)
    delete M.recs[P]
    return region

end(P, abandon):
    with M:
        region = end_locked(P, abandon)
        finish(region, seeds = {}, skip = { P })
        return N

detach_home(H):                              # precondition: the caller, ~lock_home, closed
                                             # H's port first (5.7); nothing here reads it
    with M:
        require H in M.homes
        for k in a copy of M.cache_refs[H]: unref(H, k)    # every cache reference of H (4.15)
        delete M.cache_refs[H]
        region = {}; ended = {}
        for P in a copy of the keys of M.recs, with P.home == H:     # end_locked deletes recs
            region += end_locked(P, abandon = true); ended += { P }
        delete M.homes[H]
        finish(region, seeds = {}, skip = ended)       # lending may have become false
        return N
```

- **One call per end.** Every way a process ends (design 5.6) is this one call: it releases every
  hold of the process, a granted but unresumed one included, cancels its queued ticket, and
  re-examines. Nothing is passed on, because nothing outlives the process's record: a grant
  already made is a hold of `P`, and a `wake` post still in flight finds no waiting process
  (5.9).
- `abandon` is true for kills, budgets and `fail_process`, false for `terminate_process`.
- **The port is closed before the detach, under its own mutex.** `~lock_home` calls
  `port.close()` first (5.7), on the same thread, before it calls `detach_home`; that order is
  the operation's precondition, which a debug build may assert in `~lock_home`, outside `M.mu`.
  `detach_home` reads no port state: `closed` is never read or written under `M.mu`, so nothing
  is locked under `M.mu` (section 0, design 8.3). A notification for `H` captured under `M.mu` is
  posted later, without `M.mu` (4.14), and its post is one critical section under `port.mu`
  (2.3): it completes before the close, and its entry is discarded with the queue, or it sees
  `closed` and is dropped. Either way it is a hint for a process the detach ends. No
  notification captured after `detach_home` targets `H`, since `M.homes` no longer has it. A
  grant made between the close and the detach, for a waiter of `H`, has its wake dropped; the
  detach then ends that waiter and releases the grant.
- **Cache references go with the home.** `detach_home` drops every reference in
  `M.cache_refs[H]`, so a detached home keeps no node alive (I9).

### 4.12 `set_base(P, base)`: a priority change

```
set_base(P, base):
    with M:
        if P not in M.recs: return []
        M.recs[P].base = base
        relevel_all({})
        return N
```

### 4.13 Reads

```
read_inh(P) -> int:
    with M: return (P in M.recs) ? M.recs[P].inh : INF

query(prefix, viewer) -> { held, waiting }:    # lock-query, design 4.1; viewer = { home, owner }
    with M:
        keys under prefix: every k with prefix(prefix, k), from holds and tickets
        held: for each such k with a hold: { key: k,
              mode: X if an X hold has k else S,
              pids: the pids of the holds with k that are visible to the viewer (below),
              count: the number of holds with k,
              generation: the largest gen among the X holds with k (the latest X grant
                          on k: one process may hold two, since its own holds never
                          block it, 3.1), or nil,
              abandoned: k in M.abandoned }
        waiting: for each queued T with a key under prefix, in seq order:
              { key: T.keys (a string for one key), mode,
                pid: T.pid's pid when visible to the viewer, else nil }
    # visible(Q) = Q shares the viewer's pid space (Q.home == viewer's home; from L4, a home of
    #              the viewer's pool, td B32) and M.recs[Q].owner == viewer.owner

blocking_sinks(P) -> set of (Pid, time):     # pycvc's idle test (design 9.2), L3
    with M: return { (Q, M.homes[Q.home].progress.load()) :
                     Q in reach({P}), Q != P, succ(Q) == {} }
```

`query(prefix, viewer)` takes the viewer's home with its owner, copies under `M.mu` and formats
after. Counts include every hold; only the pids are filtered, since pids repeat across schedulers
and another scheduler's pid would name one of the viewer's own processes (4.6). `blocking_sinks`
returns the processes the waiter ultimately waits for, as `(home, pid)` pairs: they run, or are
ready, paused or parked elsewhere. Each comes with its home's `progress` value, loaded under
`M.mu`: every process with a rec is on an attached home (`detach_home` ends a home's recs before
it deletes the home), and the cell is shared, so the load needs no other lock and reaches no
home object. pycvc counts only a sink on another home whose value lies within its idle window,
and a sink on the waiter's own home is judged by `run()`'s own test (design 9.2). The caller
gets values only: it never reaches another home after `M.mu` is released, so a home destroyed
meanwhile on another thread is never touched.

### 4.14 Delivery

```
deliver(N):               # by the calling home C, after the operation, without M.mu
    for n in N in order:
        if n.home == C: apply(n.post)       # 5.9, at once, on this thread
        else: n.port.post(n.post)           # 2.3, under n.port.mu; false when closed: dropped
```

### 4.15 Key-cache references

```
apply_refs(H, refs):                       # inside try_acquire and block, under M.mu
    for k in refs.add: ref(H, k)            # cache misses: each new entry's reference
    for k in refs.drop: unref(H, k)         # evictions

ref(H, k):   require H in M.homes; M.cache_refs[H] += { k }; the node of k (made if absent)
             gets refs += 1
unref(H, k): require k in M.cache_refs[H]; M.cache_refs[H] -= { k } (one occurrence; the
             entry stays, maybe empty, until detach_home); the node of k gets refs -= 1, and is
             freed if it is then empty (2.4)

ref_nodes(H, keys):   with M: for k in keys: ref(H, k); return []
unref_nodes(H, keys): with M: for k in keys: unref(H, k); return []
```

- The home probes its cache before calling the manager. A miss makes an entry and passes its
  key in `refs.add`; an entry the cache evicts to make room passes its key in `refs.drop`. Both
  ride the request's own `try_acquire` (or `block`), in the same critical section, so a hit's node
  is alive whenever the home reads it, and no extra mutex round trip is paid. `ref_nodes` and
  `unref_nodes` serve the rest: an eviction outside a request, or a cache cleared by its host.
- References change no hold, ticket, edge, count of 2.4 other than `refs`, or abandoned flag, so
  they affect no grant, inheritance or invariant except I4 and I9. `detach_home` drops every
  reference of the home (4.11).
- **A miss may evict a key of the same request here, not in the implementation.** In this
  specification a request is keys, so `refs.drop` naming a key of `R` is harmless: `unref` may free
  its node, and `new_hold` or `block` makes it again. The implementation's request is its trie nodes
  (design 8.4), so its probe pins the entries of the request it builds, and `refs.drop` never names
  a key of `R`. The implementation's behaviours are therefore a subset of this specification's,
  which the model explores in full (section 7).

## 5. Home actions

### 5.1 `with-lock` entry

In `step_init` when the head is the `lock_ops` object (design 4.2, 8.5). `P = (H, p.pid)`.

```
enter(p, form):
    if the evaluator state is a generator's: raise LockError("generator")
    if no process is stepping (current_pid() < 0): raise LockError("no process")
    parse the options; evaluate KEY and :timeout (design 8.5); R = canonical request;
        timeout = :timeout, else the context's lock_default_timeout, else none   # no clock read
    view = [ h in p.locks : not p.in_handler or h.in_handler ]   # holds that count as own

    # 1. the handler rule (design 5.7)
    if p.in_handler:
        for k in R.keys, h in p.locks with not h.in_handler:
            if (under(h, k) and (h.mode == X or R.mode == S))      # covered by it
               or (exists hk in h.keys: overlap(hk, k) and not compat(h.mode, R.mode)):
                raise LockError("handler")

    # 2. upgrade
    if R.mode == X:
        for k in R.keys:
            if (exists h in view: h.mode == S and under(h, k))
               and not (exists h in view: h.mode == X and under(h, k)):
                raise LockError("upgrade")

    # 3. covered: reentrant, the manager is not called
    if forall k in R.keys: exists h in view: under(h, k) and (h.mode == X or R.mode == S):
        c = the first h in view, in acquisition order, with under(h, first key of R) and
            (h.mode == X or R.mode == S)
        c.count += 1
        :as binds { key: shown(R.keys), mode: R.mode, generation: c.gen, abandoned: #f }
        frame -> lock_body { hold: c.id }; run the body; return

    # 4. the fast path
    refs = the home's key-cache probe of R (4.15)
    r = M.try_acquire(P, { owner: p.owner, base: p.priority }, R, refs)
    if r is granted(id, gen, ab):
        p.locks.append(MHold{ id, R.keys, R.mode, count: 1, gen, ab, p.in_handler })
        :as binds { key: shown(R.keys), mode: R.mode, generation: gen, abandoned: ab }
        frame -> lock_body { hold: id }; run the body; return

    # 5. busy
    if :if-available:
        :as binds nil
        frame -> lock_body { hold: none }; evaluate :else as the body; return
    if timeout == 0 or not the context's lock_wait_allowed: raise TimeoutError
    frame -> lock_enter; push the wait child; apply_native(%lock-wait, R, timeout)     # 5.2
```

- Steps 1-3 read only the mirror and change nothing in the manager, so a refusal leaves no state
  (I13). Every key is checked before the manager sees any.
- A covered request touches no manager state and cannot conflict with another process: a key
  under an own X hold is excluded from every other holder by I1, a key under an own S hold
  likewise for X holders, and a queued ticket that conflicts with the request conflicts with the
  covering hold, so it is exempt (3.1). The manager would have granted it.
- Within a handler, `view` holds only the handler's own holds; the program's holds that overlap a
  handler request were refused in step 1, unless both are S and neither covers the other.
- Nothing after a successful `try_acquire` in `step_init` can throw (design 8.5).
- **`:as`.** A covered request reports the covering hold's generation and `abandoned: #f`: an
  abandoned flag is consumed by the grant that took it and reported once, by the section that
  made that grant (I11), never again by a section nested in it. With `:if-available`, `:else`
  sees the symbol bound to `nil`. A granted request reports its own grant, on the fast path here
  and after a wait at 5.3. `key` is the request's canonical keys as `locks-held` shows a hold's
  (`shown`, 5.10): a string for one key and a list for several, each written as the program
  would write it.
- **No clock read on the fast path.** The deadline is computed by `%lock-wait` from `timeout` on
  the busy path only (5.2); it is still one deadline, set when the request is first tried, since
  the wait follows the failed try in the same step.
- **A context that forbids lock waits** (`lock_wait_allowed` false: Ariadne's init lane, design
  5.5) treats every busy request as `:timeout 0`, whatever its `:timeout`: `TimeoutError`, or
  `:else` with `:if-available`. Its thread never waits for a lock.

### 5.2 The wait native

`%lock-wait`, applied by `apply_native` from `step_init`, or, for a restored wait, from
`step_await_call`, which turns the restored await frame back to its apply phase and applies the
native in the same step (design 5.8). The lock-enter token is armed around that `apply_native`
call in both places (design 8.5), so it never has to outlive a step.

```
lock_wait(p, R, timeout):
    if not can_await(): raise LockError("cannot wait here")
    if there is no lock-enter token (design 8.5): raise LockError
    deadline = (timeout is none) ? none : now + timeout          # the busy path's one clock read
    refs = the home's key-cache probe of R (4.15)                 # usually all hits
    ((tid, state), N) = M.block(P, { owner: p.owner, base: p.priority }, R, refs)
    deliver(N)                                    # relevels only
    if state == queued:
        p.lock_ticket = tid; p.lock_waiting = true; p.lock_deadline = deadline
        park_lock(p, deadline)                    # status waiting; the timer heap
        request_await(handle = the ticket's awaitable, no channel)
        return
    collect(p, frame, tid)                        # granted or failed: not parked (5.9)
    return nil
```

- Between `block` and `park_lock` this home runs no other action, and posts are applied only
  between steps, so a `wake` for `tid` posted by another thread meanwhile finds the process
  parked when it is applied (no lost wakeup, I6).
- The ticket's awaitable reports `finished()` once the home has collected the outcome, and its
  `outcome()` is `nil`: the await frame completes with `nil`, and the `lock_enter` step acts on
  the outcome kept in the frame (5.3).

### 5.3 The `lock_enter` step

```
step_lock_enter(p, frame):           # after the wait child popped with nil
    if frame.outcome is none:
        raise LockError("woken without a grant")      # unreachable (I2); the unwind (5.4)
                                                      # withdraws the still-queued ticket
    if frame.outcome is granted(id):
        M.start_body(P, id)
        h = the MHold in p.locks with id == id                # joined by collect (5.9)
        :as binds { key: shown(h.keys), mode: h.mode, generation: h.gen,
                    abandoned: h.abandoned }
        frame -> lock_body { hold: id }; run the body
    else:
        raise frame.outcome          # TimeoutError("with-lock: not granted within N s", key,
                                     # mode; origin "dsl"), or the DeadlockError
```

`start_body` marks the hold in the same step that turns the frame into `lock_body`: a kill before
this step finds `body_started` false and releases clean (I11).

### 5.4 The body's end and every unwind

```
release_lock_frame(p, frame):        # noexcept; the body's last form completing, and
                                     # unwind_exception, unwind_jump and fail_step passing frame
    id = frame.hold if frame is lock_body
         else id if frame is lock_enter with outcome granted(id)
         else none
    if id != none:
        h = the MHold in p.locks with id == id
        if h is none: return                       # another process's frame (a restored copy)
        h.count -= 1
        if h.count == 0:
            remove h from p.locks
            N = M.release(P, h.id); deliver(N)
    if frame is lock_enter and p.lock_ticket != none:  # defensive: woken without a grant
        tid = p.lock_ticket
        (_, N) = M.cancel(P, tid); deliver(N); M.resume(P, tid)
        p.lock_ticket = none; p.lock_waiting = false; p.lock_deadline = none
```

- Sections are scoped, so frames release in LIFO order: a covered request's `count` decrement
  comes before the decrement of the frame that made the hold, and only that frame's reaches 0.
- A `lock_enter` frame unwound with a granted outcome is the release of a grant whose process
  was woken but left before its body began: a signal handler that raised, or an error injected
  into the `lock_enter` step (LX-4, LX-8). Its hold was never started, so the release is clean
  and returns any abandoned flag the grant took.

### 5.5 A lock deadline

```
on_lock_deadline(p):                 # wake_due found p.lock_waiting and now >= p.lock_deadline
    tid = p.lock_ticket
    (st, N) = M.cancel(P, tid); deliver(N)
    apply_wake(p.pid, tid)           # st is cancelled (TimeoutError at 5.3) or granted (adopted)
```

A grant made by another thread before the cancel, with its `wake` still in flight, is adopted:
`cancel` returns `granted`, the process resumes into the body, and the later `wake` is stale.

### 5.6 A process ends

```
on_end(p, abandon):                  # in kill_process (true), fail_process (true) and
                                     # terminate_process (false), right after set_status
    if p.locks == [] and p.lock_ticket == none: return
    if not abandon and p.locks != []: count locks.leaked_at_exit
    N = M.end(P, abandon); deliver(N)
    p.locks = []; p.lock_ticket = none; p.lock_waiting = false; p.lock_deadline = none
    p.inherited_priority = INF
```

`kill`, SIGKILL, `kill_owner`, every budget, a step deadline and an internal step error reach
`kill_process`. The test `p.locks == [] and p.lock_ticket == none` is exact: `M.recs` has `P`
exactly when one of them is set (I3).

### 5.7 A scheduler is destroyed

```
~lock_home(H):                       # on the thread that owns H, with no pump running
    H.port.close()                                   # under port.mu (2.3), first
    debug: assert H.port is closed                   # detach_home's precondition, outside M.mu
    N = M.detach_home(H); deliver(N)                 # no notification targets H
    discard H's port's posts and drop H's reference to the port
    for p in H's processes: clear p's lock fields as in on_end
```

It runs before any scheduler member is destroyed: before the processes are freed, and before the
scheduler's wakeup, ingress or queues go. After the close no post appends to the port or
notifies its wakeup, and a post in flight holds the port and the shared wakeup alive through its
`shared_ptr`s (2.3), so a poster on another thread never touches the dying scheduler (test
LT-3b). Between the close and the detach another home may grant a waiter of `H`; its wake is
dropped, and the detach ends the waiter and releases the grant (4.11).

From L4 a group's teardown (td 4.2) closes the port earlier: as soon as it holds the group's token,
so with no pump running, and before it kills the group's processes, each by `end` and its delivery
(5.6), as separate steps between which other homes act and post. A grant made after the close for a
waiter of the group has its wake dropped, and the waiter's kill releases it, as the detach would.
`~lock_home`'s close is then a no-op, and its `detach_home` finds no rec of the home left (the model
asserts it). The model checks this order (configuration `gdestroy`, section 7).

### 5.8 `set-priority`

```
set_priority(p, base):               # after the permission checks (design 7.2), on p's home
    p.priority = base; sched.update(p)
    if p.locks != [] or p.lock_ticket != none:
        N = M.set_base(P, base); deliver(N)
```

So `M.recs[P].base == p.priority` whenever the rec exists: the rec is made from `p.priority` on
this thread (5.1, 5.2) and every later change goes through here (I8). A host's unchecked
`set_priority` and a posted `control{set_priority}` use the same path.

### 5.9 Applying posts

```
apply(wake{pid, tid}):  apply_wake(pid, tid)
apply(relevel{pid}):
    p = the process pid of this home; if p is none: return
    v = M.read_inh((H, pid))
    if p.inherited_priority != v: p.inherited_priority = v; sched.relevel(p)

apply_wake(pid, tid):
    p = the process pid of this home
    if p is none or p.lock_ticket != tid or not p.lock_waiting: return     # stale
    if not collect(p, the lock_enter frame below p's await frame, tid): return
    p.lock_ticket = none; p.lock_waiting = false; p.lock_deadline = none  # leaves the heap
    deliver_await(p's evaluator state, nil)
    sched.ready_lock_waiter(p)

collect(p, frame, tid) -> bool:
    (out, _) = M.resume(P, tid)
    if out is queued: return false                  # unreachable: wakes follow grants and
                                                    # cancels only; the process stays parked
    if out is granted(id, gen, ab):
        p.locks.append(MHold{ id, frame's keys, frame's mode, count: 1, gen, ab, p.in_handler })
        frame.outcome = granted(id)
    else:
        frame.outcome = out                          # TimeoutError or DeadlockError
    return true
```

- A `wake` is stale when its ticket was adopted by a timeout, the process ended, or the process
  has since made another request (`tid` differs). It never wakes a newer wait (LX-11).
- `p.in_handler` when the hold joins the mirror is the value it had when the request was made: a
  parked process runs nothing in between.
- A `relevel` reads the current value, so a late or duplicated one is harmless (PI-7).
- A scheduler drains its port (`port.drain()`, 2.3) between steps whenever `pending` is set: the
  sync scheduler in `step_once`, `async_scheduler` in `drain`. `wait_for_work` tests `pending`
  before it blocks, `idle_wait` before each slice of its wait, and both return from the wait when
  a post notifies the shared wakeup (2.3, design 8.7).

### 5.10 Reads

- **Keys shown to a program.** `shown(k)` writes the absolute key `k` as the program would:
  relative to its chroot when `k` lies strictly under it, else with a leading `/`. `shown(keys)`
  of a request or hold takes its canonical keys in order: a string for one key, a list for
  several. `:as` (5.1, 5.3), `locks-held`, `lock-query`, the `key` fields of `LockError` and
  `TimeoutError`, a `DeadlockError`'s `cycle` and `inspect` show keys this way.
- `(locks-held)`: from `p.locks`: `{key mode count generation}` per hold, outermost first; `key` is
  `shown(h.keys)`. No manager call.
- `(lock-query [prefix])`: `M.query(abs prefix, { H, p.owner })` (4.13), its keys shown with
  `shown`.
- `(get-priority [pid])`: `{base: p.priority, effective: eff(p), floor, policy}`.
- `inspect`: `locks_held = len(p.locks)`, `waiting_on = "lock <k>"` while `p.lock_waiting`, `k`
  the first of the request's keys, shown.

### 5.11 Fork, snapshots and restores

- `fork` of a process with `p.locks != []` is refused (`LockError("fork")`). The child starts with
  empty lock fields.
- A snapshot of a process with `p.locks != []` is refused. A waiter (`p.locks == []`) may be
  snapshotted: its ticket is not encoded, and a restored copy is a new process whose re-applied
  `%lock-wait` calls `block` afresh, at the tail.
- Before phase L7, a restore of a stack holding a `lock_body` frame with a hold id is refused (no
  such snapshot can be made); L7 adds `locks_lost` (design 5.8, 10.1).

## 6. Invariants

Each holds after every manager operation and every home action completes. The model checker
asserts all of them. In test builds `lock_manager::check()` asserts, after every operation, the
manager's clauses, which read only manager state: I1, the manager's clauses of I2, I3 and I4, I5,
I6's first half (no queued ticket is grantable), I7, I8's manager clause (`inh == inh_formula`),
I9's key-cache clause (an entry exactly for each attached home), I11 and I15. The home-side
clauses read a home's private state: those of I2, I3 and I4, I6's second half, and I8's base and
quiescent clauses, of which I6's second half and I8's quiescent clause also need every home
drained. The tests assert them at quiescent points, where every home has delivered its `N` and
drained its port, with `lock_home::check_quiescent()` (test builds, design 13): LK-18 after every
step (one home delivers every notification at once and receives no post), and LT-4 and LT-10
whenever their drivers stop with every port drained during the run, and at its end, where the
manager must also be empty (I9). I10 and I12-I14 constrain transitions rather than states; the
model checker checks them, and the tests that name them exercise them.

**I1. Safety.** No two holds of different processes conflict:
`forall h1, h2 in M.holds: h1.pid != h2.pid => not conflicts(h1, h2)`.
*Why:* a hold is created only by `new_hold`, called only when `blk_hold` is empty under the same
mutex (4.5, 4.6, 4.2); covered requests make no hold (5.1).

**I2. Tickets.**
- `M.queue == { t : M.tickets[t].state == queued }`;
- a ticket is in `M.tickets` exactly when it is some rec's `ticket`, and each rec has at most one;
- `T.state == granted => T.hold in M.holds and M.holds[T.hold].pid == T.pid and
  not M.holds[T.hold].body_started`;
- states only move `queued -> granted`, `queued -> cancelled`, or are created `granted` or
  `failed`; a ticket is deleted only by `resume` or `end`, once.
- Home side, when the home is between actions: a ticket `t` of `P` exists exactly when
  `p.lock_ticket == t` and `p.lock_waiting`; its state is `queued`, or `granted` with a `wake` for
  it in a port or in an undelivered `N`. A ticket returned `granted` or `failed` by `block` is
  collected in the same action (5.2). A failed ticket is never queued: `failed` is set only by
  `block` (I7).
- A `lock_enter` frame whose outcome is `granted(id)` has `id` in `p.locks` with `count == 1`
  and `M.holds[id].body_started` false.

**I3. Records.** `P in M.recs <=> (M.recs[P].holds != {} or M.recs[P].ticket != none)`;
`M.recs[P].holds == { id : M.holds[id].pid == P }`. Home side:
`P in M.recs <=> (p.locks != [] or p.lock_ticket != none)`, and
`{ h.id : h in p.locks } == M.recs[P].holds - { M.tickets[p.lock_ticket].hold if granted }`.

**I4. Counts.** Every trie count, list, flag and `refs` equals its definition in 2.4 over
`M.holds`, `M.queue`, `M.abandoned` and `M.cache_refs`; no node is kept with all of them empty or
zero and no child. Home side, when the home is between actions: its cache holds exactly the keys
of its `M.cache_refs` entry, one reference per entry.

**I5. FIFO among conflicting requests.** When a hold is created for `P` with request `R` (a ticket
`T` with `seq s`, or a new request with `s = INF`), every queued `U` with `U.seq < s`,
`U.pid != P` and `conflicts(R, U)` is `exempt(P, U)`. So a request is never granted ahead of an
earlier conflicting one, except by the conversion rule; non-conflicting requests pass freely; and
priority never reorders. *Why:* `grantable` requires it, evaluated under the mutex at the grant.

**I6. No lost wakeups at quiescence.** After every manager operation, no queued ticket is
grantable: `forall T in M.queue: not grantable(T)`. And when every home has delivered its `N` and
drained its port, no process is `lock_waiting` on a non-queued ticket.
*Why:* grantability of a queued ticket `T` can only grow when a hold is dropped or a queued ticket
leaves (its blockers only shrink then); every such operation re-examines the tickets overlapping the
dropped keys, and every blocker of `T` overlaps `T` (Lemma D). `block` queues only a ticket that is
not grantable. A grant's `wake` is applied directly on the same home or posted to the waiter's port,
which is open unless that home is being destroyed, and then the waiter is ended, by the detach or,
for a group, by its teardown's kills (5.7); the timeout path wakes directly (5.5). Between `block`
and `park_lock` no post is applied (5.2).

**I7. No deadlock; exact detection.** The wait-for graph (3.2) is acyclic. `block` returns
`failed` with `DeadlockError` exactly when queueing the request would close a cycle.
*Why:* Lemma B (edges appear only in `block`) and the reach test in `block`.

**I8. Inheritance consistency.** Manager clause, after every manager operation:
`forall P in M.recs: M.recs[P].inh == inh_formula(P)`. Base clause, when `P`'s home is between
actions: `M.recs[P].base == p.priority`. Quiescent clause, when every home has delivered and
drained: every process `p` on a home with `applies` has `p.inherited_priority == read_inh(P)`,
and so `eff(p)` is exact.
*Why:* `relevel_all` after every operation that changes an edge or a base; every change of `inh` for
an `applies` process emits a `relevel`, delivered directly or posted (a port is closed only while
its home is destroyed, with no pump running, and its processes are then ended, by the detach or by a
group's teardown, 5.7), and the apply reads the current value. `try_acquire` and `resume` change no
edge (Lemma A, 4.7).

**I9. Empty when every process has ended.** If `end` has been called for every process that had
a rec, or every home has been detached: `M.recs`, `M.holds`, `M.tickets` and `M.queue` are empty,
every trie count is zero, and the only nodes left are those with an abandoned flag or a key-cache
reference, and their ancestors. At all times `M.cache_refs` has an entry exactly for each attached
home: `attach_home` makes it empty, and `detach_home` drops its references and deletes it (4.4,
4.11).

**I10. A grant is made once and released once.** A ticket goes to `granted` at most once (it
leaves `M.queue` as it does); a hold is dropped exactly once, by `release` or `end`,
whichever comes first, and the others find it gone (`M.holds.get` is none, or the ticket was
deleted). No hold outlives its process's rec, so nothing needs passing on.

**I11. Abandoned reports are conserved.** `M.abandoned` and the `taken` sets of the holds are
pairwise disjoint. A key's flag is set only by `end` (or `detach_home`) of an X hold whose body
started, and is consumed only by an X grant on exactly that key, which reports it through `:as`
when its body starts; if the body never starts the flag goes back (`drop_hold`). So the next X
holder of that key that enters its body sees `abandoned: #t` exactly once (LX-2, LX-3).

**I12. Owner checks.** `release`, `start_body`, `resume` and `cancel` act only on a hold or ticket of
the calling process; a frame of a restored copy in another process finds nothing (5.4).

**I13. Refusals change nothing.** `LockError` for `generator`, `no process`, `handler`, `upgrade`
and `cannot wait here`, `TimeoutError` for `:timeout 0` or a context that forbids lock waits, and
`:if-available`'s `:else` leave the manager's lock state exactly as before: everything but
`M.cache_refs` and the `refs` counts, which a busy `try_acquire` may change (4.15).

**I14. One request per process.** A process has at most one ticket, and has one only while it is
parked on it (I2): it has none whenever it runs, so a request made by its program, its body or a
signal or watch handler never meets a pending ticket of its own. `try_acquire` and `block`
require it.

**I15. Generations increase.** Every X grant's `gen` is greater than every earlier grant's, on
any key; an S grant's `gen` is the last X generation at its grant.

### Lemmas

- **Lemma A (a grant adds no edge).** Let a hold be created for `P` from request `R` that was
  `grantable(P, R, s)`. Any queued `U` of another process that conflicts with the new hold
  already had an edge to `P`: if `U.seq > s` (`U` later than a ticket `R`), `U` had `R` in
  `blk_wait(U)` unless `exempt(U.pid, R)`, which would put `R` in conflict with a hold of
  `U.pid` and make `R` not grantable; if `U.seq < s`, grantability needs `exempt(P, U)`, so `U`
  conflicts with a hold of `P` and has the hold edge. And `P`, granted, has no edges out.
- **Lemma B (edges appear only in `block`).** `succ(P)` for a queued `P` changes only through
  `blk_hold(P, T)` and `blk_wait(P, T, T.seq)`. New holds add no process-level edge (Lemma A).
  `blk_wait` only loses members (no ticket with a smaller `seq` is ever queued later). `exempt(P,
  U)` depends on `P`'s holds, which do not change while `P` waits (Lemma C). So an edge appears
  only when `block` queues a ticket.
- **Lemma C (the exemption is stable).** While `P` has a queued ticket it does not step, so it
  neither acquires nor releases; its holds change only through `end`, which also removes the
  ticket.
- **Lemma D (re-examination is complete).** A queued `T` that was not grantable becomes grantable
  only when one of its blockers goes: a hold that conflicts with `T` (so overlaps it) is dropped,
  or a queued ticket that conflicts with `T` (so overlaps it) leaves. Both overlap `T`'s keys, and
  the operation that removes them re-examines that region. A grant in the pass turns a queued
  blocker of `T` into a hold that still conflicts with `T`, so it unblocks nothing, and the set
  granted in one pass does not depend on the order.

## 7. Model checking (LT-5)

- **Configuration.** Two homes (threads), each with two or three processes; keys from
  `{a, a.b, a.c, b}`; requests of one or two keys, S or X; nesting depth up to 2; one home under
  `priority_rr` and one under `round_robin`, and both under `priority_rr`; key caches of 0 to 3
  entries per home (FIFO eviction), so `M.cache_refs` changes with requests and detaches. These
  are L3's. The phases before it, whose manager accepts one home (4.4), run the same keys and
  requests on one home of three processes: L1 with `prio` false and no `set-priority` action, L2
  under `priority_rr` with `set-priority` (design 12).
- **Atomic actions.** Each manager operation is one atomic action, together with the calling
  home's private updates around it up to its `deliver`. Each `post` is one action, so another
  home may act between an operation and its delivery; in the `splitpost` configurations the
  post's critical section under `port.mu` (2.3) is a further separate action of the posting
  thread. Applying one post is one action of the target home, enabled between its steps. Per
  process, enabled at its step boundaries: start a request (5.1, then 5.2 when busy); the
  `lock_enter` step (5.3) once readied; finish a body (5.4); raise inside a body or a handler at
  `lock_enter` (5.4); a deadline firing on a waiter (5.5); a kill (5.6) at any point;
  `set-priority` with any value in `-2..2` (5.8). Per home: destruction (5.7), as two actions,
  `port.close()` and then `detach_home`, between which other homes act and post.
- **Properties.** I1-I15 after every action; at every state where every `N` is delivered and
  every port drained, I6's second half and I8's quiescent clause. Liveness under weak fairness of
  every enabled action, with every holder eventually finishing its body: every queued ticket is
  eventually granted or cancelled, and every granted ticket's process resumes or ends. No post
  appends to a closed port or reaches a destroyed scheduler (`port-race`).
- **Expected passes.** The `splitpost` configurations (2.3 and 5.7 as written: a post in flight
  across the close and the detach) report no violation.
- **Expected counterexamples** when the spec is mutated, as a check on the checker: removing the
  exemption makes LK-27's nested request raise `DeadlockError`; re-examining only the dropped
  holds' keys, not a cancelled ticket's, leaves LK-25's reader queued while grantable (I6);
  skipping `relevel_all` in `cancel` leaves PI-3's boost (I8); letting `wake` carry an outcome
  lets a stale wake resume a newer request (LX-11); testing `closed` outside `port.mu` (a reading
  2.3 rules out) lets a post land after the close (`port-race`); and keeping a detached home's
  cache references leaves its nodes (I9).
- **The design-time model.** Until the C++ test exists, a Python model checks this specification:
  [`tools/locks_model/`](../tools/locks_model/). It implements section 4's operations literally, as
  methods over the state of section 2 whose `check()` asserts section 6 after every operation, and
  section 5's home actions with these abstractions:
  - shared-mode semantics only: there is no `M.mu` and no thread, every operation is one atomic
    action, and 4.4's L3 refusal of a home pumped on another thread is absent (the one-home rule
    before L3 is modelled);
  - a port has no `pending` flag or wakeup, since homes never block: a queued post is always
    enabled. Posts are applied one per action, so a home's processes may step between the posts of
    one `drain` (an over-approximation of 5.9). A notification names its target home, whose port is
    looked up at delivery (equivalent, since a home's port is never replaced), and `shared_ptr`
    lifetimes are not modelled: a post that reaches a destroyed scheduler is the `port-race`
    violation;
  - time is not modelled: a progress cell is a logical clock that its home's steps advance, a
    deadline is a flag, and its firing an action;
  - keys are canonical, with no chroots, shown keys or `:as` values; a cache maps canonical keys
    with FIFO eviction, and a miss may evict another key of the same request, which the
    implementation rules out (4.15), so the model has more interleavings, not fewer;
  - not modelled: 5.1's generator, no-process and cannot-wait-here refusals and the lock-enter token
    (`lock_wait_allowed` false shares a request kind with `:timeout 0`); 5.2's restored waits;
    5.10's reads other than `query` and `blocking_sinks`; 5.11 (fork, snapshots, restores); 2.4's
    free lists; and section 9.

  It also models L4's group teardown (5.7: the close, then the kills, then the detach). It explores
  bounded, smaller versions of this section's configuration: every configuration exhaustively up to
  4 external events (5 for seven of them; home steps and posts cost nothing), every interleaving of
  three scripted processes (one or two nested requests each) with one or two faults, and random runs
  of 3-14 processes over 1-3 homes with caches of 0-3 entries. Its one-home configurations, `l1` and
  `l2`, have three processes, the requests `X a`, `S a.b`, `X a.c` and `X {a.b, b}`, nesting depth 2
  and caches of 0-3 entries. Its two-home configurations have three processes, two on one home (the
  `splitpost` pair has one on each), with nesting depth 2 in most and one fixed cache size each (0,
  1, 2 or 3); `two2` has two processes on each home but one request each, so no nesting, and `three`
  has three homes of one process. `set-priority` ranges over `-2..2` in `priority` and `l2`, and
  over `(-2, 0, 2)` in `coalesce`, `noinh` and `grantseeds`. Liveness is checked by a fair drain at
  the bound (every body finishes, every post is applied, and no process may stay parked), not as
  weak fairness over cycles of the state graph.

  Revision 5b is the one that follows this text. Revision 5 added 2.1's `progress` cell; 4.4's
  `attach_home` with its fourth argument and its empty `cache_refs` entry; 4.11's `detach_home`,
  which reads no port (the closed assertion sits in the destroy action, outside the operation);
  4.13's `(Pid, time)` sinks; 4.15's `unref`, which keeps the home's entry; I9's entry exactly for
  each attached home; and the one-home configurations. Revision 5b adds 4.13's mode and generation
  check and the group teardown (`gdestroy`, half of the random worlds with two or three homes, and
  half of the scripted destroy pairs). Earlier revisions' runs do not cover those clauses.

## 8. What the simplification removed

From the round-2 design: intention modes (IS, IX) and their compatibility table; per-entry
queues; kept marks, `applied` chains and `own_is`/`own_ix` on waiters; partial multi-key chains
and their continuations; `lock_requeued`, `lock_fail`, `lock_reprio` and `lock_grant` posts;
pass-on of orphaned grants; 64 shards, their lock order and lock-then-validate; cut and uncut
domains, `add_domain`/`remove_domain`, `LockError("covers domain")`; the per-home holder list;
the local deadlock walk, the lazy cross-home check, `graph_mutex_`, `lock_check_at` and
`state_exec.locks.deadlock_check_seconds`; per-entry generations; and `awaitable::expired()`.
What replaces them is sections 1-6: one table, one mutex, all-or-nothing grants, two kinds of
hint post, and a ticket that holds the outcome.

## 9. Outside this specification: moves (L4)

Stage 3's `adopt` (design 9.3, td 4.15) moves a holder to another home. Every `Hold.pid` and
`M.recs` key is a `(HomeId, int)`, and `applies` and `lends` read the home, so a move changes
state this specification covers. It is not specified or model-checked here, and phase L4 does not
merge until it is. The operation L4 is to add, model-check (with moves as actions in section 7)
and prove against I1-I15 is `move`, with `adopt_locks`, the new home's part of `adopt` that calls
it:

```
move(P, H2) -> P2:                          # the new home H2 calls it, on its own thread,
                                            # inside adopt (td 4.15, step 4): C is H2
    with M:
        require P in M.recs and M.recs[P].ticket == none
        require H2 in M.homes and H2 != P.home
        P2 = (H2, P.pid)                    # pids are pool-wide from Stage 1 (td B32)
        rekey M.recs[P] to P2, and every hold h with h.pid == P to h.pid = P2
        relevel_all({ P, P2 })              # P has no rec now; P2 is re-read below
        if applies(P2): N += relevel(P2.pid) to H2      # to C: applied at once (4.14)
        return P2, N

adopt_locks(p, H1):                         # H2's part of adopt, for every arriving process p
                                            # from home H1, as it inserts p into its table
    p.inherited_priority = INF              # a boost is never carried
    if p.locks != []:                       # a holder; a waiter is never moved
        (P2, N) = M.move((H1, p.pid), H2); deliver(N)   # re-reads it when applies(P2)
```

`relevel_all` reads each process's values by its key (4.3): `P2`, which had no rec at the start,
has an old value of `INF`, and `P`, which has none at the end, a new value of `INF`. Both are
skipped: no `relevel` goes to the old home, and the moved process is re-read once, by the one
`relevel` to `H2`, whatever its value. That re-read is needed even when the manager's value did
not change, since `adopt_locks` has just reset the process's copy. Other processes are
re-levelled as usual: `lends` of the moved process may change with its home's policy.

The reset keeps every process's copy exact on arrival. On a home without `applies`, where no
`relevel` is ever sent, a moved holder keeps `INF`, so `effective_priority()`, which does not
know the policy, and `inspect` never show a boost its old home applied. A process with no rec
arrives with `INF`, which is `read_inh` for it, so a `relevel` lost on the old home leaves
nothing stale. When L4 model-checks moves, I8's quiescent clause gains a second half: every
process on a home without `applies` has `p.inherited_priority == INF`.

A waiter is never moved (it is parked), so no ticket is rekeyed. Posts already sent to the old
home for `P` are hints that find no process there (5.9). L4 must also settle a move in flight
when its source home is destroyed: between the source's removal of the process and `adopt`,
`detach_home` of the source would end `P`'s rec (it ends every rec of the home), so either a
group is not destroyed with a move in flight, or `move` handles a rec already ended.
