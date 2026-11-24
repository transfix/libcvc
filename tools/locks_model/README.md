# locks_model: the design-time model of the state_exec lock protocol

This is the Python model checker of
[`docs/STATE_EXEC_LOCKS_PROTOCOL.md`](../../docs/STATE_EXEC_LOCKS_PROTOCOL.md), described in its
section 7 ("The design-time model"). It checks the protocol before any C++ exists; the C++ model
test of [`docs/STATE_EXEC_LOCKS.md`](../../docs/STATE_EXEC_LOCKS.md) (LT-5) replaces it once the
code lands.

`lockmodel.py` implements the lock manager of protocol section 4 literally, one method per
operation over the state of section 2, and its `check()` asserts the manager-side invariants of
section 6 after every operation. `world.py` adds the homes, processes, ports and posts of section
5 as a step relation, and checks the home-side invariants after every action. The other scripts
drive that world. This is revision 5b of the model, the one the design's review ran, with one
later fix (see "Revisions" below).

What the model abstracts (shared mode only, no time, canonical keys, the parts of section 5 that
are not modelled, and so on) is listed in protocol section 7 and is not repeated here.

Python 3, standard library only (run with 3.11). Run every script with `-I`: each one puts its
own directory on `sys.path`, so they run from any directory. The results the long run writes go
to `results/`, which is not committed.

## Files

| File | What it is |
|---|---|
| `lockmodel.py` | the manager (protocol 2-4, 6), the `Violation` the checks raise, and `STATS`, the coverage counters |
| `world.py` | homes, processes, ports and posts (protocol 5), `Config`, and the world's invariants |
| `scenarios.py` | directed scenarios from the design's test list, and protocol 7's mutations |
| `explore.py` | bounded exhaustive exploration of one configuration |
| `explore_scripts.py` | every interleaving of scripted processes, with a fault budget |
| `random_runs.py` | long random runs over larger worlds |
| `run_long.sh` | the long run: all of the above at their full sizes |
| `export_traces.py` | the manager operations of one-home scenarios and random runs (phases L1 and L2), as traces the C++ model check replays |
| `state_word.py` | the group state word of the continuous pool (S2a, PL-1): a separate model, below |
| `test_state_word.py` | its tests: the quick suite, and checks of its explorer |
| `router.py` | the router protocol across exec groups (S2b, MS): a separate model, below |
| `test_router.py` | its tests: the quick suite, and checks of its explorer |

## Running it

**Quick check (under a second).** Directed scenarios, then each mutation of protocol 7 with the
violation it must produce:

    python -I scenarios.py

Every scenario prints `ok` or its expected outcome (a `DeadlockError` cycle, a refusal), and each
mutation line says `caught as expected (NAME)`. The last line is
`scenarios.py: 26 scenarios, 13 mutations, 0 unexpected`; the exit status is 1 if anything is
unexpected.

**Bounded exploration.** Every reachable state of one configuration (below) within a bound on
external events (requests, finishes, raises, signals, kills, deadlines, priority changes,
destroys); home steps and post deliveries cost nothing. States at the bound also run a fair drain,
which checks liveness:

    python -I explore.py CONFIG BOUND [MAX_STATES]

It prints JSON on stdout: `states`, `transitions`, `seconds`, `complete` (false if `MAX_STATES`,
default 10^8, stopped it), and `violations` and `soft`, each with the shortest trace found. Bound 3
takes seconds (`core` at 3: 15,746 states in 12 s). Each step of the bound multiplies the states;
bound 5 for `cache` and `l2` is the longest part of the long run.

**Scripted exploration.** Each process runs a fixed script of nested requests; everything else (the
order of the homes, the delivery of posts, and the faults) is explored exhaustively for each
combination, capped at 400,000 states per combination:

    python -I explore_scripts.py MODE [N] [SEED] [WORKERS]

`N` defaults to 1000, `SEED` to 1 and `WORKERS` to 20 (a `multiprocessing` pool; use 1 for a
single process). The modes:

- `single`: every combination of one request per process from 13 request shapes, on the two
  two-home layouts (4,394 combinations; `N` is ignored), with one fault.
- `nested`: `N` random combinations over six layouts (a priority and a round-robin home, two
  priority homes, three homes, a priority home attached late, and the one-home `l1` and `l2`), one
  or two nested requests per process, a signal handler that locks in 70% of them, and one fault.
  About 30% of the combinations may destroy a home (`destroy_with_lock_state` in the coverage
  counts the destroys that find lock state).
- `pairs`: `N` combinations with two faults from one of 12 pairs of fault kinds, among them
  destroy and kill, and destroy and deadline; every other round of the destroy pairs destroys a
  home as a group's teardown (not on the one-home layouts).

It prints JSON: `combos`, `states`, `transitions`, `incomplete` (combinations stopped at the cap),
`violations`, `soft` and `coverage`, the summed `STATS` counters. Seven `nested` combinations take
about 100 s on one worker.

**Random runs.** Worlds of one to three homes, 3-14 processes, the keys `a`, `a.b`, `a.c`,
`a.b.d`, `b`, `b.e` and `c` in requests of one to three keys, every fault, priorities -2..2 under
`priority_rr`, key caches of 0-3 entries, some managers without inheritance, a home attached late,
and a group's teardown in half of the worlds with two or three homes. Every step is checked, each
episode ends with a fair drain, and a violating episode is shrunk:

    python -I random_runs.py TOTAL_STEPS [WORKERS] [SEED0]

`WORKERS` defaults to 20 and `SEED0` to 7. 20,000 steps take about 10 s on one worker. It prints
JSON: `steps`, `episodes`, `violations`, `soft` and `coverage`.

**The long run.** Everything above at the sizes the design's review used: `scenarios.py`;
`explore.py` at bound 4 for every configuration and at bound 5 for `core`, `destroy`, `gdestroy`,
`handlers`, `attach`, `cache` and `l2`; `explore_scripts.py` `single`, `nested 1500` and
`pairs 1200`; and three `random_runs.py` runs of 4-6 million steps.

    sh run_long.sh                    # or PY=/path/to/python sh run_long.sh

The script checks first that `PY` (by default `python3`, else `python`, on `PATH`) runs Python 3.8
or later, and stops if it does not. On Windows the `python3` on `PATH` is often the Microsoft
Store's stub, so set `PY` to a real interpreter there.

It takes about 2 hours on 22 cores and about 15 GB of memory at the peak (the bound-5 runs keep
every state they visit). `B4_JOBS`, `W` and `OVERLAP` trade time for cores and memory; the
script's header describes them. It writes one file per run into `results/` (or `$R`; a relative
`R` is taken from this directory, where the script runs), and `SUMMARY.txt` there checks them
all: its last line is `0 unexpected` when every scenario is ok, every mutation is caught, every
exploration is complete and no result has a violation except `splitpost_mut`'s expected
`port-race`. The script exits 0 only then.

## The configurations of explore.py

Every configuration uses the keys `a`, `a.b`, `a.c` and `b`, three processes over two homes, and
deadlines on waits, unless it says otherwise.

| Configuration | What it covers |
|---|---|
| `core` | multi-key requests and nesting, a `priority_rr` and a `round_robin` home |
| `wide` | all 13 request shapes, one request per process, no nesting, no kills |
| `handlers` | signals and handlers that lock, timeouts, `:if-available`, terminate |
| `priority` | priorities and inheritance on two `priority_rr` homes, `set-priority` over -2..2 |
| `destroy` | a scheduler destroyed with holders and waiters (5.7), a key cache of 1 |
| `gdestroy` | `destroy` as L4's group teardown: the port closed, a kill per process, then the detach |
| `coalesce` | relevel coalescing (4.3), with priority changes and destroys |
| `attach` | a priority home attached after a round-robin home's processes lock, then destroyed |
| `noinh` | `M.options.inheritance` false, with priority changes |
| `foreign` | release and cancel naming another process's hold or ticket (I12) |
| `splitpost` | two processes, one `X a` request each, no deadlines: a post as a separate action after its operation, under the port's mutex (2.3, 4.14), with destroys |
| `splitpost_mut` | `splitpost` with the closed flag tested outside the mutex: must report `port-race` |
| `splitpost2` | the split post with more requests, kills, a timeout and a key cache |
| `cache` | key-cache references (2.4, 4.15) under every request shape, kills and destruction |
| `l1` | phase L1: one `round_robin` home of three processes, no `set-priority`, caches of 0-3, a host's cache clear |
| `l2` | phase L2: one `priority_rr` home of three processes, `set-priority` over -2..2, caches of 0-3 |
| `two2` | two homes of two processes each, one request each, a key cache of 3 |
| `grantseeds` | 4.3's seeds without the granted processes, an equivalent mutation: must pass |
| `three` | three homes of one process each |

## Coverage counters

`lockmodel.STATS` counts events as the model runs; they are not part of the state.
`explore_scripts.py` and `random_runs.py` print them as `coverage`. Among them:
`destroy_with_lock_state` and `detach_with_lock_state` (a home destroyed, or detached, while the
manager holds records of its processes), `group_teardown`, `post_dropped_closed_port`,
`posted_wake`, `posted_relevel`, `stale_wake_ignored`, `boosted` and `deadlock_len_N`.

## The C++ model check (LT-5)

`src/cvc/tests/state_exec_locks_model_test.cpp` (suites `LocksModel` and `LocksModelStress`, in
the `state_exec_locks_model_test` executable) ports this model to C++ and runs it against the real
lock table, `detail::lock_table` (locks plan L1.3). It was written from the protocol and from these
files, not from the table's code. Phases L1 and L2 run it on one home (L2.3 added priorities,
inheritance and their mutations); L3.4 runs it on two and three homes.

- **The spec oracle** (`spec_manager`) is `lockmodel.py`'s `Manager`: protocol 2-4 over sets and
  maps, without the trie. It takes `lock_table::mutation`'s bits, its `check()` asserts section
  6's manager clauses as `Manager.check()` does, and its `canonical()` writes the state in the
  format of `lock_table::canonical()` (`detail/lock_table.h`).
- **The lockstep** (`lockstep`) runs every manager operation on the oracle and on the real table,
  the way a lock home calls the table: a key the home's cache holds passes its trie node, a miss
  passes its key with a reference (4.15's `refs.add`), an eviction passes the evicted node. The
  results, the notification lists and the two `canonical()` dumps must be equal, then both
  `check()`s run, and the table's `stats()` counts of holds, queued tickets, grants, cache
  references, relevels and coalesced relevels must match the oracle's. The table runs with
  `self_check` on, so its grant-time assertions of I5 and I15 and its own check of protocol 4.3's
  implementation note (`impl-note`) run too. When the oracle refuses an operation (a `require` of
  the protocol, the traces' `refused`), the same call goes to the table, which must refuse it too
  (its precondition's `lock_violation`, or 4.4's `std::logic_error`), and the two states must
  still be equal. Each home attaches with a progress cell, which the oracle's `M.homes` entry and
  the table's share, and on the table with a `lock_port` of its own: every notification the table
  returns must carry its home's port (2.3's `Notification`, captured under `M.mu`, 4.14).
  `blocking_sinks` (4.13) is the oracle's, compared with the table's. A finding is a
  `lock_violation` named as here (`I6`, `4.13`, `lockstep`, ...).
- **Inheritance, three ways.** The table recomputes inherited priorities by 4.3's implementation
  note (the set A, a reverse breadth-first search of depth 8 per process over predecessor edges
  found through its trie); its `check()` compares every record with its own layered search over
  edges recomputed from the definitions of 3.1; the oracle computes `inh_formula` with
  `lockmodel.py`'s layered `anc8` and its `check()` compares it with an enumeration of every path
  of 1 to 8 edges. The oracle also checks the implementation note against its reference loop, as
  `Manager._impl_note_check` does.
- **The world** (`world`) is `world.py`: homes, processes, frames, key caches, ports and posts,
  the per-action micro-operations, and the world's invariants, plus three checks of the table
  alone: a mirror hold's keys against `hold_keys()`; I4's home clause, one reference of the home
  on each cached key's node and none other (`home_cache_refs()`); and at I9's points `empty()`
  and a trie of only the flagged and cached keys and their ancestors (`stats().trie_nodes`). Its key cache
  pins a request's entries, as the implementation does (protocol 4.15, design 8.4): the probe
  looks up every key of the request first, pinning each hit, and only then makes the misses, each
  taking a free entry or evicting the oldest unpinned one, so a miss never evicts a key of the
  request it builds, and a key that finds every entry pinned passes uncached. `world.py`'s plain
  FIFO, which may evict a key of the same request, is an option (`pin`), used for the state counts
  below. On several homes (L3.4) it has `world.py`'s ports: a post to another home is an action of
  its own (in the `splitpost` configurations the post's critical section under the port's mutex,
  `dlv_append`, is a further one), a home is destroyed in two actions (the port's close, then
  `detach_home`, other homes acting in between), a late home is attached by an action, and each
  step of a process advances its home's progress cell, a logical clock. At every quiescent state
  it checks `blocking_sinks` for every waiter: each sink a process that can run, with its home's
  cell.
- **The explorer** (`explore()`) is `explore.py`'s 0-1 BFS (external events cost 1, home steps
  and posts 0) with a fair drain at the bound. States are deduplicated on a 128-bit fingerprint
  of a dump of the world (the oracle's `canonical()` and every field `World.key()` has), and kept
  as (parent, action): a state over the real table is rebuilt by replaying its path into a fresh
  world, since the table cannot be copied; on the oracle alone it is copied. The shortest trace
  of each violation is printed.

### Its configurations and tests

`l1` is `explore.py`'s `l1`. `l1x` adds what locks plan L1 section 3 (LT-5 item 4) lists and
`l1` lacks: one signal per process, whose handler may lock and raise, and foreign release and
cancel calls (I12). `LocksModel.L1Bounded` explores `l1x`; `LocksModel.RandomL1` runs
`random_runs.py`'s episodes restricted to one home of prio false (phase L1).

`l2` (locks plan L2.3) is `explore.py`'s `l2`, one `priority_rr` home of three processes with
`set-priority` over -2..2 and caches of 0-3 entries, with relevel coalescing on, the production
setting. On one home every relevel is applied right after its operation (4.14), so coalescing
changes no notification list, and the states are those of `explore.py`'s `l2` with coalescing on
or off. `noinh1` is `l2` with `lock_manager::options::inheritance` off (design 6.2's test switch):
nothing lends, every inherited priority stays INF, while bases change. `LocksModel.L2Bounded` and
`LocksModel.NoInheritance` explore them; `LocksModel.RandomL2` runs the random episodes on one home
of prio true, with `set-priority` over -2..2.

The scenarios of `scenarios.py` that L1 can run are `TEST(LocksModel, ...)` cases, each on one
home, except LX-10 and LX-11, which need a post between two homes: they run with `one_home` off,
which the L1 table honours (were a table to refuse a second home whatever the option, they would
run on the oracle alone and say so). The L1 mutations each report exactly the violation `scenarios.py` names: the manager's
(`no_exemption`, `cancel_reexamines_holds_only`, `keep_detached_refs`, `unref_pops_entry`,
`cancel_marks_failed`, `query_first_gen`) on the oracle and the table together, the world's
(`wake_carries_outcome`, `cache_miss_unreferenced`) in the world. Two scenarios differ from
`scenarios.py` because of the single home: `cycle_other_home_pid` becomes
`CycleOtherOwnerPid` (a pid is `nil` for another owner), and the cache scenarios check one home's
references.

Phase L2 adds `scenarios.py`'s inheritance scenarios with their homes merged into one
`priority_rr` home: `L2OneHome`, `PI09BoostThroughWaiterEdge`, `PI11MultiHolderBoostAndWithdraw`,
`PIChainCappedAt8` (`chain_cap8`), `PICoalesce`, `PIImplNote` and `SetBaseL2`, and their
mutations, each reporting exactly the violation `scenarios.py` names: the manager's
`cancel_skips_relevel` (`no-relevel-cancel`, PI-11's boost kept: `I8`), `coalesce_no_clear`
(`coalesce-no-clear`, the withdrawal of a boost coalesced away for ever: `I8`'s quiescent clause)
and `impl_note_after_only` (`impl-note-after-only`, the set A without the graph before: `impl-note`)
on the oracle and the table together (`Mutation_CancelSkipsRelevel`, `Mutation_CoalesceNoClear`,
`Mutation_ImplNoteAfterOnly`), and the world's `no_set_base` (`no-set-base`, `I8`'s base clause,
`Mutation_NoSetBase`). Only `post-test-outside-mu`, L3.4's, is left of `scenarios.py`'s 13. The
`LockTable` suite has the same three manager mutations as directed cases on the table alone, where
the table's own `impl-note` check, not the oracle's, reports the third.

Phase L3 (locks plan L3.4) explores `explore.py`'s configurations of two and three homes over the
table, each exactly as `explore.py` has it: `core`, `wide`, `handlers`, `priority`, `destroy`,
`coalesce`, `attach`, `noinh`, `foreign`, `splitpost`, `splitpost2`, `cache`, `two2`,
`grantseeds` and `three` (`LocksModel.Homes<Name>` at the regular bound of its lane, below, and
`LocksModelStress.Homes<Name>` at 3). Coalescing is on in `coalesce` alone, on the table and the
oracle together, as `explore.py` sets it (G-39); `CVC_LOCKS_MODEL_COALESCE=1` turns it on in every
`LocksModelStress.Homes*` case (the `+c` variants of `locks-deep.yml`). `grantseeds` checks 4.3's
implementation note with seeds that leave out the processes granted ('no-granted-seeds', the
oracle's check alone). `LocksModel.RandomHomes` runs `random_runs.py`'s episodes over one to three
homes (a home attached late in 30% of the worlds of several, coalescing per seed; L4's group
teardown, and with it `gdestroy`, is left out).

`scenarios.py`'s scenarios also run as written, on two `priority_rr` homes (`TwoHomes*`): the L1
and L2 ones above, with posts between the homes, and `CacheDetach`, `CacheClear`,
`CycleOtherHomePid`, `QueryVisibility`, `SplitPost` and `SinksProgress`, whose assertions name
the homes. All 13 of its mutations run there, each reporting exactly the violation it names:
L3.4's `closed_outside_mu` (`post-test-outside-mu`: a post tests the port's `closed` outside the
port's mutex and lands after the close, `port-race`), set on the oracle and the table together and
acted on by the world's ports, and the L1 and L2 ones again (`TwoHomesMutation*`). Two explorations
check the same: `HomesSplitpostMut` (`splitpost_mut`: `port-race`) and `HomesCoalesceNoClear`
(`coalesce` with `coalesce_no_clear`: `I8`).

`LocksModel.PortMatchesPythonStateCounts` ties the port to this model: on the oracle alone and
with `world.py`'s FIFO cache, `l1`, `l1x` (as `explore.py`'s `l1` with `signals=1` and
`foreign=True`), `explore.py`'s two-home `core`, `l2` and `noinh1` (as `explore.py`'s `l2` with
`inheritance=False`) must have exactly the distinct states and the transitions `explore.py`
counts, both copied and replayed:

| Configuration | Bound 1 | Bound 2 | Bound 3 (`LocksModelStress`) |
|---|---|---|---|
| `l1` | 73 states, 132 transitions | 1,633, 2,412 | 26,834, 43,572 |
| `l1x` | 85, 168 | 1,921, 3,552 | 34,266, 68,514 |
| `core` | 25, 24 | 776, 871 | 15,746, 23,201 |
| `l2` | 121, 120 | 2,587, 3,624 | 42,075, 63,798 |
| `noinh1` | 121, 120 | 2,323, 3,360 | 33,297, 55,020 |

`LocksModel.PortMatchesPythonStateCountsHomes` does the same for L3.4's other configurations
(replayed too only in an optimized build; bound 3 in `LocksModelStress`):

| Configuration | Bound 1 | Bound 2 | Bound 3 (`LocksModelStress`) |
|---|---|---|---|
| `wide` | 40, 39 | 2,612, 2,683 | 97,523, 145,729 |
| `handlers` | 19, 33 | 449, 754 | 9,857, 18,248 |
| `priority` | 31, 30 | 813, 1,121 | 20,610, 34,143 |
| `destroy` | 20, 19 | 344, 419 | 4,855, 7,132 |
| `coalesce` | 29, 28 | 764, 989 | 17,414, 28,209 |
| `attach` | 16, 15 | 227, 265 | 2,210, 3,067 |
| `noinh` | 29, 28 | 624, 849 | 10,438, 17,485 |
| `foreign` | 19, 24 | 425, 619 | 6,751, 12,801 |
| `splitpost` | 7, 6 | 31, 40 | 159, 252 |
| `splitpost2` | 17, 16 | 219, 279 | 2,419, 3,655 |
| `cache` | 26, 43 | 802, 1,207 | 19,932, 34,312 |
| `two2` | 29, 28 | 747, 888 | 13,959, 20,516 |
| `grantseeds` | 25, 24 | 663, 833 | 15,920, 25,129 |
| `three` | 28, 27 | 784, 945 | 18,382, 29,205 |

**`explore.py` used to under-count `l2`.** It deduplicated states on `hash(World.key())`, and in
CPython `hash(-1) == hash(-2)`, so two states that differed only in a priority of -1 against -2
collided, and the one found second was never explored: 113, 2,383 and 37,647 states at bounds
1-3 for `l2`, and 18,929 instead of 20,610 for `priority` at 3. Both explorers now deduplicate on
`explore.state_id`, a 128-bit digest of the key's `repr` (the keys themselves would multiply the
bound-5 runs' memory), and give the counts above, which match the port exactly; `priority` at 3
finds no violation in the states it used to skip. The other counts above are the same either way.

At bound 4, `l1` with `world.py`'s cache has 324,347 states and 586,802 transitions in both. With
the implementation's pinned cache the counts differ (`l1` at bound 3: 26,744 states and `l1x`
34,176; at bound 4, 322,916 and 459,161): pinning removes the states in which a request evicted
one of its own keys. Before L2.3 the world pinned a request's keys one at a time, so a miss could
still evict a later key of the same request, which the home never does; bound 3 had the same
counts, bound 4 had 321,206 and 457,451.

`LocksModel.LockstepCatchesPlantedDivergence` checks the check: with `plant_divergence` on the
table only, exploring `l1` must report one violation, a `lockstep` mismatch of the
`counters grants=` line at a `release`.

### The traces of the Python model

`export_traces.py` runs `scenarios.py`'s scenarios with their homes merged into one home of prio
false (the ones that need several homes or L2's inheritance are left out, and listed), and
`random_runs.py`'s episodes of the first seeds whose world is one home of prio false, four with
coalescing off and four with it on, 200 steps each and the fair drain. For phase L2 it does the
same on one home of prio true, files `l2_*.txt`: the inheritance scenarios (`L2_SCENARIOS`: those
of `chain_cap8`, `coalesce`, `impl_note`, `l2_one_home`, `pi11`, `pi9` and `set_base`) and eight
episodes of the first seeds whose world is one home of prio true. For phase L3, files `l3_*.txt`:
the scenarios of two or more homes as `scenarios.py` writes them (L4's group teardown left out)
and eight episodes of the first seeds whose world has two or three homes and no group teardown;
these also record `blocking_sinks` with the progress cells it loaded, which the replay stores
into its own cells first. It records every manager operation the world makes, with its result and
notifications, one per line; its docstring specifies the format. `LocksModel.PythonTraces` replays
every file into the oracle and into the table and requires the recorded results and notifications.
Regenerate the committed files after a change to the model (the output does not depend on the hash
seed):

    python -I export_traces.py ../../src/cvc/tests/data/locks_model_traces

A request in these traces may evict one of its own keys (the model's FIFO cache); the lockstep
passes such an eviction to the table as an `unref_nodes` right after the request, since the
table's `cache_delta` never names a key of the request.

### Sizes

Each test reads its size from the environment, with defaults that depend on the build (locks plan
L1 section 3, "Sizes, by lane"):

| Variable | Read by | Default: Release / Debug / `CVC_LOCKS_SANITIZER=1` | `LocksModelStress` |
|---|---|---|---|
| `CVC_LOCKS_MODEL_BOUND` | `L1Bounded`, `L2Bounded`, `NoInheritance`; `L1Bound4`, `L2Bound4` | 3 / 2 / 2 | 4 (5 in `locks-deep.yml`) |
| `CVC_LOCKS_MODEL_BOUND` | `Homes*` (L3.4) | 3, but 2 for `wide` / 3 for `attach`, `splitpost` and `splitpost2`, else 2 / 2 | 3 (4, and 5 for some, in `locks-deep.yml`) |
| `CVC_LOCKS_MODEL_STEPS` | `RandomL1`, `RandomL2`, `RandomHomes` | 2*10^4 / 5*10^3 / 2*10^3 | 2*10^5 |
| `CVC_LOCKS_MODEL_COALESCE` | `LocksModelStress.Homes*` | - | 1: coalescing on in every configuration (the `+c` variants) |

"Debug" is a build without `NDEBUG`. Two more variables serve development:
`CVC_LOCKS_MODEL_REAL=0` runs every test on the oracle alone (the tests that need the table skip
while it is the interface commit's stub, which throws "not built yet"), and
`CVC_LOCKS_MODEL_TRACES` names another trace directory. `LocksModelStress.DISABLED_Measure`
measures the explorer: run it with `--gtest_also_run_disabled_tests` and
`CVC_LOCKS_MODEL_MEASURE=CONFIG:BOUND[:replay]` (comma-separated; one per process gives each its
own peak memory).

### Measurements

Taken on 2026-10-28 on the Windows builder (MSVC 19.44, 22 cores), not in a quiet window: other
builds ran beside them, so the seconds are indicative. Peak memory is the process's peak working
set. "Copied" is the explorer on the oracle alone, each state copied; "replayed" rebuilds each
state by replaying its path, as it must over the real table; "table" is the explorer over the
real table (replayed, both managers, both dumps and both `check()`s after every operation).

Release (`/O2`), the explorer:

| Configuration, bound | States | Transitions | Copied | Replayed | Table | Peak |
|---|---|---|---|---|---|---|
| `l1`, 3 | 26,744 | 43,572 | 1.2 s | 1.3 s | | 18 MB |
| `l1`, 4 | 322,916 | 584,048 | 32 s | | | 52 MB |
| `l1`, 5 | 2,846,064 | 5,626,181 | 383 s | | | 303 MB |
| `l1x`, 3 | 34,176 | 68,514 | 2.0 s | 3.8 s | 8.0 s | 19 MB |
| `l1x`, 4 | 459,161 | 1,032,517 | 50 s | | 179 s | 60 MB |
| `l1x`, 5 | 4,696,383 | 11,749,705 | 763 s | | | 511 MB |
| `l2`, 3 | 42,057 | 63,798 | 4.2 s | 3.9 s | 13 s | 21 MB |
| `l2`, 4 | 485,490 | 765,892 | 65 s | 73 s | 216 s | 66 MB |
| `l2`, 5 | 4,072,404 | 6,666,736 | 624 s | | 2,865 s | 410 MB |
| `noinh1`, 3 | 33,279 | 55,020 | 1.8 s | | 6.1 s | 19 MB |
| `noinh1`, 4 | 352,782 | 633,184 | 38 s | | 130 s | 54 MB |
| `noinh1`, 5 | 2,832,711 | 5,427,043 | 320 s | | | 308 MB |

The `l1` and `l1x` rows at bounds 4 and 5, the `l1x` row over the table at bound 3, and every `l2`
and `noinh1` row were measured for L2.3, after its fix of the world's pinning (the counts of `l1`
and `l1x` grew by 0.4-1.5%), with several other agents' builds on the machine: `l1x` at bound 4 over
the table took 179 s, against L1.3's 99 s for nearly the same states, and at bound 5 L1.3 measured
1,823 s. The other times are L1.3's. `explore.py` takes 22 s for `l1` at bound 3 and 262 s at bound
4, and 25 s for `l2` at bound 3 deduplicating on `World.key()`.

Over the table `l2` costs 3.3 times its copied run at bound 4 and 4.6 times at bound 5 (`l1x`:
3.6 at bound 4 here, 4.2 at bound 5 in L1.3). `l2` at bound 5 over the table, 48 minutes, is the
longest of these runs, for `locks-deep.yml`'s daily job.

Without optimization (`/Od`, asserts on; the test file alone, linked against the Release library,
so the table's own share runs optimized): `l1x` at bound 2 takes 0.45 s on the oracle alone and
0.71 s over the table; at bound 3, 12 s on the oracle alone. For L2.3: `l2` at bound 2 takes
0.59 s on the oracle alone and 1.1 s over the table, `noinh1` 0.42 s and 0.92 s, and `l2` at
bound 3 16 s and 32 s. The whole regular suite (`LocksModel.*`, 43 cases since L2.3) takes 26 s
in Release over the table (9.4 s before L2.3), and 11.5 s unoptimized at its Debug sizes. So the
defaults above, the locks plan's, stand for L2 as for L1: they leave room under the 60 s budget for
a Debug build's unoptimized table and coverage, which bound 3 in a Debug lane would not (`l2`
alone would take 32 s, before coverage). In a build without `NDEBUG`,
`LockstepCatchesPlantedDivergence` stops at its first violation instead of exploring all of `l1`
at bound 3 (7 s unoptimized).

**L3.4, two and three homes.** Taken on 2026-11-01 on the same machine, again not in a quiet
window: the bound-3 column was measured twice, once with several builds beside it and once in a
quieter spell (`wide` took 129 s and 48 s), and both are shown. The explorer over the real table,
Release, with the implementation's pinned cache (`attach`, `cache` and `three` have a few states
fewer than `explore.py`'s FIFO counts above):

| Configuration | Bound 2 | Bound 3 | Bound 4 | Bound 5 |
|---|---|---|---|---|
| `core` | 776 states, 0.18 s | 15,746, 8.9 s / 4.6 s | 203,381, 72 s | 1,776,323, 2,158 s, 189 MB |
| `wide` | 2,612, 0.85 s | 97,523, 129 s / 48 s | 622,225, 2,478 s, 189 MB | |
| `handlers` | 449, 0.09 s | 9,857, 4.8 s / 2.6 s | 167,194, 53 s | |
| `priority` | 813, 0.16 s | 20,610, 11 s / 5.9 s | 415,593, 155 s, 54 MB | |
| `destroy` | 344, 0.08 s | 4,855, 2.1 s / 1.3 s | 52,365, 15 s | 430,966, 130 s, 55 MB |
| `coalesce` | 764, 0.21 s | 17,414, 9.0 s / 5.1 s | 289,468, 98 s | |
| `attach` | 229, 0.05 s | 2,229, 0.66 s / 0.36 s | 16,894, 3.5 s | 122,116, 29 s, 27 MB |
| `noinh` | 624, 0.13 s | 10,438, 3.7 s / 2.1 s | 130,087, 31 s | |
| `foreign` | 425, 0.12 s | 6,751, 3.9 s / 2.3 s | 74,106, 29 s | |
| `splitpost` | 31, 0.004 s | 159, 0.04 s / 0.02 s | 356, 0.05 s | |
| `splitpost2` | 219, 0.05 s | 2,419, 0.87 s / 0.49 s | 21,711, 5.4 s | 153,711, 45 s, 55 MB |
| `cache` | 802, 0.23 s | 19,910, 10.5 s / 6.1 s | 358,673, 147 s | |
| `two2` | 747, 0.25 s | 13,959, 7.8 s / 4.6 s | 180,214, 120 s | |
| `grantseeds` | 663, 0.20 s | 15,920, 9.5 s / 4.8 s | 282,685, 134 s | |
| `three` | 784, 0.23 s | 18,279, 13 s / 6.1 s | 314,250, 121 s, 54 MB | |

Peak memory is under 35 MB where none is shown. Every run reported no violation.

Without optimization (`/Od`, asserts on, the test file alone against the Release library, as
above), bound 3 took 1.4 to 2.3 times the first Release column (`core` 12.7 s, `cache` 24 s, `three`
20 s, `attach` 1.4 s, `splitpost2` 1.7 s), and the whole regular suite (`LocksModel.*`, 98 cases
with L3.4) took 22.8 s at its Debug sizes. A real Debug lane is slower than that: on CI's Linux
Debug lane with coverage, `L1Bounded` (`l1x` at bound 2) took 7.8 s and `RandomL1` (5,000 steps)
10.9 s, about 30 times this machine's Release (0.23 s and 0.40 s). At 30 times the quieter Release
column only `attach` (11 s), `splitpost` and `splitpost2` (15 s) finish within 60 s at bound 3
(`destroy` would take about 40 s, too close), so the Debug lane runs those three at bound 3 and the
rest at 2 (at most `wide`'s 26 s); Release runs every one at 3 but `wide` (48-129 s), which
`LocksModelStress` covers. The sanitizer lanes run bound 2. `PortMatchesPythonStateCountsHomes`
replays its configurations only in an optimized build: copied and replayed, it would take about a
minute in that Debug lane.

## The state-word model (S2a, PL-1)

`state_word.py` is a second model, separate from the lock model: the group state word of
threaded execution's continuous pool (S2a;
[`docs/STATE_EXEC_THREADED.md`](../../docs/STATE_EXEC_THREADED.md) 4.4 and 8.3's PL-1; the plan's
S2-Q2, S2-Q4, S2-Q5 and S2-Q15), as the S2a interface commit froze it in its comment on
`detail::group_pool_link` in
[`exec_group.h`](../../inc/cvc/state/state_exec/exec_group.h): the bits SCHEDULED, NOTIFIED,
PUMPING and DETACHING, and the pool's `queued`, `host_held` and `deferred` beside them under the
ready-queue mutex Q. It shares only `Violation` with the lock model, and like `gdestroy` it has no
C++ port. `test_state_word.py` tests it.

**A step** is one atomic operation: a fetch_or, fetch_and, CAS, load or store of the word; a take
or release of Q, of the notifier lock NL (`detail::wakeup`'s, held while the notifier runs), of an
ingress mutex or of the lock port's mutex; a take or release of the token. What only Q's holder
touches (the ready queue, the pool's record of the group, the timer heap) is folded into the step
that holds Q. A state is a flat list of small ints, kept as bytes, explored breadth first, so each
trace is a shortest one. The workers are interchangeable, so a state is kept with its workers
sorted; and with one group pop is under Q and the group is queued once, so at most one worker is
past its pop at a time: a third worker adds no state (`deep-three-workers` has the default's
count).

**The actors** (S2-Q15):
- posters, alternately an ingress post (the ingress mutex, the entry, then `wakeup::notify`
  under NL calling wake(g)) and a lock post (the same under the port's mutex, dropped once the
  port is closed);
- workers: pop and PUMPING in one step under Q, clear NOTIFIED, take the token, drain and run,
  release the token, the release under Q. A run's outcome is a choice: runnable work left or
  not, a sleeper's deadline (none, future, or already due at the release), a failure
  (`failures`), a waiting host body (`waiting`, run by the attaching thread under the blocked
  worker's token);
- the timer: a filed deadline firing under Q as wake(g), at any time, so stale fires are
  explored;
- hosts: with_group on other threads. Its fn does nothing, posts, or submits directly under the
  token, as `exec_group::execute` does (a ready process, no post). A host may race a detach at
  any point, but not a teardown: a host program does not use a group it is destroying, so only a
  with_group that set host_held before the teardown began is the teardown's to wait for. With the
  hold handoff, or in the liveness check, a host waits as the pool waits: a look under Q, then a
  wait linked on the group until a release or a hold's end notifies it, then a look again;
- the attaching thread: attach's wake, its own with_group calls (`bodies`), its host entries
  (`entries`: bodies or callbacks a pump queued, each run under the token as the pool's
  `service_once` runs one), the servicing of waiting bodies, take_failed, and last the teardown
  or the detach, S2-Q4's five steps.

**The properties** (PL-1), each a violation name. In every state: `double-queue`,
`queued-while-pumped`, `queued-unscheduled`, `two-pumpers`, `pump-host-held`, `token-contended`
(a take finds another thread's token) and `body-without-token`. After the detach:
`wake-after-detach`, `read-after-detach` (a worker or the timer), `record-after-detach` (anyone,
once the attachment is dropped) and `use-after-free`. When every actor is idle (between posts, at
the top of its loop, not started or done, between operations), for an attached group not
detaching: `lost-wake` (a post not drained, or runnable work left, and the group not queued),
`lost-timer` and `stuck-scheduled` (SCHEDULED set, the group in no queue, so every later wake is a
no-op). No step enabled while an actor is not idle is a `deadlock`. With the hold handoff:
`stranded-hold` (at an idle point, a hold handed to a host that will not take it up),
`frozen-while-host-away` (a hold handed to the attaching thread while it is busy outside the
pool: every pop defers until it services again), `handoff-over-hold`, `handed-without-holder`
and `waiting-body-dropped` (step 4 drops a waiting body its worker blocks on). The liveness check
(`--opts live=1`) adds `host-starved` and `entry-starved`, below.

**Running it.**

    python -I state_word.py --actors posters=2,workers=2,hosts=1,bodies=1,teardown=1 --amend all
    python -I state_word.py --suite quick     # the CI checks, about 30 s
    python -I state_word.py --suite deep      # quick, then the larger bounds (locks-deep.yml)
    python -I test_state_word.py              # unittest: the quick suite, and the explorer's checks

The first prints JSON like `explore.py`'s: `states`, `transitions`, `idle_states`, `seconds`,
`complete` and `violations`, each with its detail, its length in steps and its trace. `--actors`
counts posters, workers, hosts and bodies and ends the attaching thread with `teardown=1` (the
default) or `detach=1`; `--opts` sets `posts` (per poster), `timer`, `sleeps` (sleepers the runs
may arm), `waiting`, `failures`, `pre` (posts before attach), `nested` (a body that detaches
inside its fn), `entries` (host entries the pumps may queue), `repop` (a worker whose release
queues the group again pops it in the same critical section, as the pool's next slot does) and
`live` (the liveness check); `--amend` takes `all` (the default: the transitions as amended in
`exec_group.h`, with the hold handoff, `handoff-idle`, `handoff-waiting` and `handoff-giveback`),
`none` (as S2a.0 first froze them) or names; `--mutation` a name; `away=1` lets the attaching
thread leave the pool's calls for good (busy elsewhere). `--suite` exits 0 only when every check
holds: the amended transitions find nothing, each finding is found without its amendment, and
each mutation is caught.

### What it finds in the frozen transitions

Posters, workers, the timer and the teardown alone satisfy every property at every bound tried.
The host side does not. Each finding has an amendment (`--amend`); with all four nothing is found.
The `finding-*` checks of the quick suite run every amendment but one and must find its violation.

1. **`host-wake`, a lost wake.** with_group's release re-queues the group only "if deferred". A
   with_group or host body that submits under the token (with_group's purpose: "submit, read,
   kill, reap"), or a deferred `on_complete` that submits a follow-up, makes an idle group
   runnable and posts nothing, so nothing ever pumps it. The amendment: when no pop was deferred,
   the release does wake's transition under Q (fetch_or(S|N), and queue unless SCHEDULED or
   DETACHING was set).
2. **`host-excl`.** Hosts wait only for PUMPING, and host_held is one flag: a with_group on
   another thread and the attaching thread's body take the token side by side
   (`token-contended`), and in a teardown one's release clears the other's flag, step 2 passes,
   and the group is destroyed under a with_group (`use-after-free`, 33 steps). The amendment:
   host_held names its thread, and a host also waits while another thread holds it.
3. **`host-detaching`.** A with_group racing a detach sets host_held without looking at
   DETACHING, after step 2 looked: the token is contended at step 3, and the host's release
   writes the pool's record after step 4 dropped it (`record-after-detach`). The amendment: a
   host that finds DETACHING set, or the group no longer attached, under Q sets nothing and takes
   the token as for an unattached group once the detach is over.
4. **`nested-release`** (`--opts nested=1`). Step 2 waits for "a host_held of another thread",
   which admits a detach inside the same thread's with_group; with_group's release then writes
   the dropped record (25 steps). The amendment: the release reads the attachment first (or a
   detach inside with_group is refused).

With the four amendments, now in `exec_group.h`, the host transition reads as follows (with the
fourth finding's other choice its last sentence would be "Never a detach or a teardown of g
inside fn (asserted)", and step 2 would wait for any host_held); step 4 says too that the word is
stored 0 only after the notifier is cleared (the mutation `zero-before-clear` below). The hold
handoff changes this row again (next section):

    - with_group, or a non-waiting host body, on a host thread: under Q (each time it looks):
      if detaching is set or g is no longer attached, set nothing and take the token as for an
      unattached group once the detach is over (at once on the attaching thread); else wait
      while pumping is set or another thread holds host_held (servicing that thread's host
      queue meanwhile), then set host_held to this thread; take the token, run, release it;
      under Q clear host_held, and if deferred, clear it and queue g again, else as wake(g) (fn
      may have submitted, killed or resumed under the token, which posts nothing). If fn
      detached g, the release leaves the pool's record alone.

The shortest traces of the first three (`hh` host_held, `tok` the token, `fl` posts not drained,
`ready` runnable work left; `python -I state_word.py` with the options shown prints them):

`finding-host-wake`, `lost-wake` (23 steps):

    python -I state_word.py --actors posters=0,workers=1,bodies=0,teardown=0 --opts timer=0 \
        --amend host-excl,host-detaching,nested-release

    initial: word 0, nothing queued or held
    H1: with_group: the group is attached                          0
    A: attach: installs the notifier; wakes g: fetch_or(S|N) finds 0 S|N
    A: takes Q                                                     S|N
    A: queues g                                                    S|N queued
    A: releases Q                                                  S|N queued
    W1: takes Q, pops g, fetch_or(P)                               S|N|P
    W1: releases Q                                                 S|N|P
    W1: fetch_and(~N)                                              S|P
    W1: takes the token                                            S|P tok=W1
    W1: run: ends with runnable=False, next_deadline=none          S|P tok=W1
    W1: releases the token                                         S|P
    W1: takes Q                                                    S|P
    W1: release: fetch_and(~P)                                     S
    W1: release: CAS(S -> 0) succeeds                              0
    W1: releases Q                                                 0
    H1: takes Q, looks: free: sets host_held                       0 hh=H1
    H1: releases Q                                                 0 hh=H1
    H1: takes the token                                            0 hh=H1 tok=H1
    H1: runs fn: submits a process directly (exec_group::execute: no post) 0 hh=H1 tok=H1 ready
    H1: releases the token                                         0 hh=H1 ready
    H1: takes Q                                                    0 hh=H1 ready
    H1: clears host_held (not deferred)                            0 ready
    H1: releases Q                                                 0 ready
    every actor is idle or done -> VIOLATION lost-wake: runnable work left and the group is not queued (word 0)

`finding-host-excl`, `token-contended` (12 steps):

    python -I state_word.py --actors posters=0,workers=1,teardown=0 --opts timer=0 --amend \
        host-wake,host-detaching,nested-release

    initial: word 0, nothing queued or held
    H1: with_group: the group is attached                          0
    H1: takes Q (PUMPING clear), sets host_held                    0 hh=H1
    H1: releases Q                                                 0 hh=H1
    H1: takes the token                                            0 hh=H1 tok=H1
    A: attach: installs the notifier; wakes g: fetch_or(S|N) finds 0 S|N hh=H1 tok=H1
    A: takes Q                                                     S|N hh=H1 tok=H1
    A: queues g                                                    S|N queued hh=H1 tok=H1
    A: releases Q                                                  S|N queued hh=H1 tok=H1
    A: starts a host body                                          S|N queued hh=H1 tok=H1
    A: takes Q (PUMPING clear), sets host_held                     S|N queued hh=A tok=H1
    A: releases Q                                                  S|N queued hh=A tok=H1
    A: takes the token  -> VIOLATION token-contended: A takes the token while H1 holds it

`finding-host-detaching`, `token-contended` (15 steps):

    python -I state_word.py --actors posters=0,workers=1,bodies=0,teardown=0,detach=1 --opts \
        timer=0 --amend host-wake,host-excl,nested-release

    initial: word 0, nothing queued or held
    H1: with_group: the group is attached                          0
    A: attach: installs the notifier; wakes g: fetch_or(S|N) finds 0 S|N
    A: takes Q                                                     S|N
    A: queues g                                                    S|N queued
    A: releases Q                                                  S|N queued
    A: starts the detach                                           S|N queued
    A: detach step 1: takes Q                                      S|N queued
    A: fetch_or(D)                                                 S|N|D queued
    A: releases Q                                                  S|N|D queued
    A: step 2: takes Q: PUMPING clear, no host_held of another thread S|N|D queued
    A: releases Q                                                  S|N|D queued
    H1: takes Q, looks: free: sets host_held                       S|N|D queued hh=H1
    H1: releases Q                                                 S|N|D queued hh=H1
    A: step 3: takes the token                                     S|N|D queued hh=H1 tok=A
    H1: takes the token  -> VIOLATION token-contended: H1 takes the token while A holds it

### The hold handoff and dropped host entries (S2a.4)

The pool (S2a.4) added two things to the amended transitions; the model checks both.

**The handoff** (`hold-handoff`). A release that queues the group again (runnable, a deadline
due, or the CAS failed on NOTIFIED) first sets host_held for a host waiting to hold it, on its
behalf: a with_group linked as waiting, else the attaching thread when it has host entries for
the group. The pop that follows finds host_held set and leaves the group notified and deferred;
the host takes the hold up at its next look (or, the attaching thread, when it runs the entry),
runs, and releases as the host row says (deferred: queue it again; else wake's transition). A
detach's step 1 takes back a hold handed over and not yet taken up. Without it, a worker that
queues its group and pops it again in one critical section (`repop=1`, as the pool's next slot
does) keeps every host off a group that stays runnable: the pool saw with_group and host bodies
starve (CT-3 took 359 s), and the model finds it (`finding-no-handoff`: `host-starved`,
`entry-starved`).

**It is safe**: with the handoff, every property holds at every bound below, with host entries,
the same-section re-pop, failures, waiting bodies, detach and teardown; no hold is stranded,
handed over another or left without a holder, and no token is contended. Its progress has two
holes, each with a fix the model checks: a group made idle and woken again at once, and an
attaching thread busy outside the pool.

**But it does not stop starvation** (`finding-handoff-idle`). A release that makes the group
idle (its CAS succeeds) hands nothing over. When the group is woken again at once (a short timer,
such as the 1 ms frame-waiter beat, or a steady stream of posts), a worker pops it before the
waiting host, notified by the release, takes Q and looks; the host finds it pumping, links itself
again, and the next release again makes the group idle. The trace below repeats for ever.
**`handoff-idle`**, the same handoff in a release whose CAS makes the group idle, cures it: the
host then holds an idle group, and its release's wake queues the group again (one spurious pump
at most). With both, the liveness check finds no starvation at any bound below.

**To whom the attaching thread's hold goes.** As built, the release hands the hold to the
attaching thread whenever it has a host entry for the group, whatever that thread is doing. A
thread busy outside the pool (not in pump_host, wait_for_work, with_group or a detach) then has
the group frozen: every pop defers, and nothing pumps the group's posted or runnable work until
the thread services again (R2b's probe: one pump in 10 s). The model lets the attaching thread
leave the pool's calls for good (`--opts away=1`) and checks, when every actor is idle, that no
hold is handed to it then (`frozen-while-host-away`; `finding-handoff-as-built`, 18 steps). The
fix handing the hold only to a host waiting now (`handoff-waiting`: a with_group linked as
waiting, or the attaching thread inside pump_host or wait_for_work with entries for the group) is
not enough alone: the thread can be handed the hold while it services and then return from
pump_host or wait_for_work before it runs the entry (`finding-handoff-giveback`, 18 steps).
`handoff-giveback` closes it: a thread leaving those calls gives back a hold handed to it and not
taken up (host_held cleared, its waiters notified, and the group queued again if a pop deferred
it). With both, nothing is found, and an attaching thread that services is never starved (the
liveness check, `live-amended-away`).

**The liveness check** (`--opts live=1`, without the symmetry reduction) looks, for each host
waiting to hold the group and for the group's host entries, for a reachable cycle that keeps it
waiting and is fair: each actor enabled in every state of the cycle steps in it (weak fairness),
and so does each actor that waits only for a free mutex in some state of it (the mutexes are taken
as starvation-free; a host notified by a release waits only for Q). Each strongly connected
component of the waiting states is tested; one with such a cycle is reported with the path to it
and the cycle.

**Revoking a handed hold** (step 1) is safe, and not needed for safety as the pool builds the
wait: a host linked as waiting always takes a handed hold up at its next look, before it looks at
DETACHING, and step 2 then waits for it. The revocation is what keeps a hold from stranding should
a host ever stop waiting: the mutation `no-revoke`, where a with_group stops waiting when a
teardown begins, leaves step 2 waiting for ever (`deadlock`). With the revocation, a with_group
that was waiting when ~exec_group began is turned away to the unattached path and then uses a
destroyed group; that is the host program's race, as before the handoff (it held nothing of its
own when the teardown began).

**Host entries dropped by `detach_end`** (the teardown's step 4; `detach` runs them in step 5).
No waiting body can be among them: step 2 waited for PUMPING, and a waiting body's worker pumps
until the body ran (the check `waiting-body-dropped` holds). After step 4 nothing runs one, and
no worker or host waits on one; the processes parked on them are killed in step 5.

In the words of `exec_group.h`'s comment, the rows that change (the model runs these):

    - release, under Q: clear pumping. detaching: done (the teardown waits for this). A failed
      pump: see failure. Else, when a host waits to hold g now (a with_group linked on g as
      waiting, else its attaching thread inside pump_host or wait_for_work with host entries for
      g), set host_held to that host on its behalf (the hold is handed over: the pop that follows
      leaves g notified and deferred); an attaching thread that leaves pump_host or
      wait_for_work with a hold handed to it and not taken up gives it back under Q (host_held
      cleared, its waiters notified, and g queued again if deferred).
      Then pump_result::runnable, or its next_deadline due: queue g again. Else CAS the word
      from exactly `scheduled` to 0, and only if that succeeds file next_deadline (if any) in
      the pool's timer heap; if it fails (a post set notified), queue g again.
    - with_group, or a non-waiting host body, on a host thread: under Q (each time it looks): a
      hold a release handed this thread is taken up; else if detaching is set or g is no longer
      attached, [as now]; else wait, linked on g as waiting to hold it, while pumping is set or
      another thread holds host_held (servicing this thread's host queue meanwhile), then set
      host_held to this thread; [the rest as now].
    - detach and teardown: (1) under Q set detaching, and take back a hold handed over and not
      yet taken up (host_held and deferred cleared, its host notified); [(2), (3) as now]; (4)
      under Q leave the ready queue, the next slots, the ready list, the failed list and the
      timer heap, and take the group's host-queue entries off its attaching thread's queue (no
      waiting body is among them: step 2 waited for pumping); [the rest of (4) as now]; (5) the
      teardown drops those entries unrun and kills (which settles their calls), shuts the host
      down, purges and reaps; detach runs them instead and releases the token.

The cycle (`hh` host_held):

`finding-handoff-idle`, `host-starved` (27 steps):

    python -I state_word.py --actors posters=0,workers=1,bodies=0,teardown=0 --opts \
        pre=1,repop=1,live=1,entries=1 --amend \
        host-wake,host-excl,host-detaching,nested-release,hold-handoff,handoff-waiting,handoff-giveback

    initial: word 0, nothing queued or held
    H1: with_group: the group is attached                          0 fl=1
    A: attach: installs the notifier; wakes g: fetch_or(S|N) finds 0 S|N fl=1
    A: takes Q                                                     S|N fl=1
    A: queues g                                                    S|N queued fl=1
    A: releases Q                                                  S|N queued fl=1
    W1: takes Q, pops g, fetch_or(P)                               S|N|P fl=1
    W1: releases Q                                                 S|N|P fl=1
    W1: fetch_and(~N)                                              S|P fl=1
    W1: takes the token                                            S|P tok=W1 fl=1
    W1: drains 1 post(s); run: ends with runnable=False, next_deadline=future (armed a sleeper) S|P tok=W1
    H1: takes Q, looks: pumping: links itself as waiting to hold g S|P tok=W1
    H1: releases Q, waits                                          S|P tok=W1
    W1: releases the token                                         S|P
    W1: takes Q                                                    S|P
    W1: release: fetch_and(~P)                                     S
    W1: release: CAS(S -> 0) succeeds; files next_deadline in the heap 0
    W1: releases Q                                                 0
    T: takes Q, the deadline falls due                             0
    T: as wake: fetch_or(S|N) finds 0: queues g                    S|N queued
    T: releases Q                                                  S|N queued
    W1: takes Q, pops g, fetch_or(P)                               S|N|P
    W1: releases Q                                                 S|N|P
    W1: fetch_and(~N)                                              S|P
    W1: takes the token                                            S|P tok=W1
    W1: run: ends with runnable=False, next_deadline=future        S|P tok=W1
    H1: wakes, takes Q, unlinks itself, looks again: pumping: links itself as waiting to hold g S|P tok=W1
    H1: releases Q, waits                                          S|P tok=W1
    ... and the cycle from step 13 repeats for ever: H1, waiting to hold g in with_group, waits for ever: a fair cycle of 24 states keeps it pending (each actor enabled throughout, or waiting for a free mutex, steps in it)

### The mutations

Each breaks one transition of the amended protocol and must be caught (`mutation-*`):
- `release-store`: the release clears SCHEDULED by a load and a plain store
  instead of the CAS. A post lands between them: its wake finds SCHEDULED and leaves NOTIFIED,
  which the store wipes, so its entry is never drained (`lost-wake`, below).
- `pump-host-held`: pop pumps a host-held group; the worker holds the token
  with host_held set (`pump-host-held`, below), or finds the host's token (`token-contended`).
- `zero-before-clear`: the detach stores 0 before clearing the notifier. A wake in progress finds
  DETACHING clear under Q and queues the detached group (`queued-unscheduled`, then a worker's
  `read-after-detach` and `use-after-free`): the word goes to 0 only after the notifier is
  cleared.
- `wake-no-recheck`: wake queues without looking at DETACHING again under Q; a wake whose fetch_or
  preceded step 1 queues the group after step 4 left the queue (`read-after-detach`).
- `pop-no-detaching`: pop pumps a detaching group after step 2 passed (`read-after-detach`,
  `token-contended`).

`mutation-release-store`, `lost-wake` (19 steps):

    python -I state_word.py --actors posters=1,workers=1,hosts=0,bodies=0,teardown=0 --amend \
        all --mutation release-store

    initial: word 0, nothing queued or held
    A: attach: installs the notifier; wakes g: fetch_or(S|N) finds 0 S|N
    A: takes Q                                                     S|N
    A: queues g                                                    S|N queued
    A: releases Q                                                  S|N queued
    W1: takes Q, pops g, fetch_or(P)                               S|N|P
    W1: releases Q                                                 S|N|P
    W1: fetch_and(~N)                                              S|P
    W1: takes the token                                            S|P tok=W1
    W1: run: ends with runnable=False, next_deadline=none          S|P tok=W1
    P1: takes IM, pushes an ingress entry                          S|P tok=W1 fl=1
    P1: takes NL: calls the notifier, wake(g)                      S|P tok=W1 fl=1
    W1: releases the token                                         S|P fl=1
    W1: takes Q                                                    S|P fl=1
    W1: release: fetch_and(~P)                                     S fl=1
    W1: release (mutated): loads S                                 S fl=1
    P1: wake: fetch_or(S|N) finds S: done                          S|N fl=1
    P1: releases NL, IM                                            S|N fl=1
    W1: release (mutated): stores 0                                0 fl=1
    W1: releases Q                                                 0 fl=1
    every actor is idle or done -> VIOLATION lost-wake: a post not drained and the group is not queued (word 0)

`mutation-pump-host-held`, `pump-host-held` (11 steps):

    python -I state_word.py --actors posters=1,workers=1,bodies=0,teardown=0 --amend all \
        --mutation pump-host-held

    initial: word 0, nothing queued or held
    H1: with_group: the group is attached                          0
    H1: takes Q, looks: free: sets host_held                       0 hh=H1
    H1: releases Q                                                 0 hh=H1
    A: attach: installs the notifier; wakes g: fetch_or(S|N) finds 0 S|N hh=H1
    A: takes Q                                                     S|N hh=H1
    A: queues g                                                    S|N queued hh=H1
    A: releases Q                                                  S|N queued hh=H1
    W1: takes Q, pops g, fetch_or(P)                               S|N|P hh=H1
    W1: releases Q                                                 S|N|P hh=H1
    W1: fetch_and(~N)                                              S|P hh=H1
    W1: takes the token  -> VIOLATION pump-host-held: W1 holds the token while H1 holds host_held

### State counts and times

Measured on 2026-11-20 on the Windows builder (Python 3.11, one process, other builds beside it):
the states (one per orbit of the workers), the transitions and the seconds of each check.

| Check | Configuration (beside the defaults) | States | Transitions | Seconds | Found |
|---|---|---|---|---|---|
| `amended` | amend all | 364,236 | 1,144,396 | 6.6 | - |
| `amended-detach` | posters=1, hosts=2, bodies=0, detach, failures=1, amend all | 195,532 | 548,287 | 3.5 | - |
| `amended-waiting` | posters=1, hosts=0, waiting=1, failures=1, amend all | 14,060 | 30,548 | 0.2 | - |
| `amended-nested` | posters=1, workers=1, hosts=0, detach, timer=0, nested=1, amend all | 5,892 | 11,471 | 0.0 | - |
| `finding-host-wake` | posters=0, workers=1, bodies=0, no teardown, timer=0, amend all but host-wake | 423 | 666 | 0.0 | lost-wake |
| `finding-host-excl` | posters=0, workers=1, no teardown, timer=0, amend all but host-excl, hold-handoff, handoff-idle, handoff-waiting, handoff-giveback | 5,500 | 11,044 | 0.0 | token-contended |
| `finding-host-excl-teardown` | posters=0, workers=1, timer=0, amend all but host-excl, hold-handoff, handoff-idle, handoff-waiting, handoff-giveback | 13,992 | 30,397 | 0.1 | record-after-detach, token-contended, use-after-free |
| `finding-host-detaching` | posters=0, workers=1, bodies=0, detach, timer=0, amend all but host-detaching | 1,976 | 3,491 | 0.0 | record-after-detach, token-contended |
| `finding-nested` | posters=1, workers=1, hosts=0, detach, timer=0, nested=1, amend all but nested-release | 5,832 | 11,413 | 0.1 | record-after-detach |
| `mutation-release-store` | posters=1, workers=1, hosts=0, bodies=0, no teardown, amend all, mutation release-store | 395 | 652 | 0.0 | lost-wake |
| `mutation-pump-host-held` | posters=1, workers=1, bodies=0, no teardown, amend all, mutation pump-host-held | 6,970 | 16,974 | 0.1 | pump-host-held, token-contended |
| `mutation-zero-before-clear` | posters=1, workers=1, hosts=0, bodies=0, amend all, mutation zero-before-clear | 1,604 | 3,077 | 0.0 | queued-unscheduled, read-after-detach, use-after-free |
| `mutation-wake-no-recheck` | posters=1, workers=1, hosts=0, bodies=0, amend all, mutation wake-no-recheck | 1,632 | 3,112 | 0.0 | read-after-detach, use-after-free |
| `mutation-pop-no-detaching` | posters=1, workers=1, hosts=0, bodies=0, amend all, mutation pop-no-detaching | 2,350 | 4,592 | 0.0 | read-after-detach, token-contended, use-after-free |
| `amended-entries` | posters=1, entries=1, amend all | 239,232 | 657,070 | 4.6 | - |
| `amended-entries-repop` | posters=1, entries=1, repop=1, amend all | 216,575 | 586,388 | 4.4 | - |
| `live-amended` | posters=1, workers=1, no teardown, pre=1, repop=1, live=1, entries=1, amend all | 229,885 | 614,407 | 4.9 | - |
| `live-amended-detach` | posters=0, workers=1, detach, pre=1, repop=1, live=1, entries=1, failures=1, amend all | 76,059 | 175,725 | 1.1 | - |
| `finding-no-handoff` | posters=0, workers=1, no teardown, pre=1, repop=1, live=1, entries=1, amend all but hold-handoff, handoff-idle, handoff-waiting, handoff-giveback | 35,576 | 79,834 | 0.5 | entry-starved, host-starved |
| `finding-handoff-idle` | posters=0, workers=1, bodies=0, no teardown, pre=1, repop=1, live=1, entries=1, amend all but handoff-idle | 5,742 | 11,003 | 0.1 | entry-starved, host-starved |
| `mutation-no-revoke` | posters=0, workers=1, bodies=0, timer=0, pre=1, entries=1, amend all, mutation no-revoke | 20,359 | 40,836 | 0.2 | deadlock |
| `finding-handoff-as-built` | posters=1, workers=1, hosts=0, bodies=0, no teardown, entries=1, away=1, timer=0, amend all but handoff-waiting, handoff-giveback | 1,627 | 2,686 | 0.0 | frozen-while-host-away |
| `finding-handoff-giveback` | posters=1, workers=1, hosts=0, bodies=0, no teardown, entries=1, away=1, timer=0, amend all but handoff-giveback | 1,780 | 2,982 | 0.0 | frozen-while-host-away |
| `amended-away` | posters=1, workers=1, bodies=0, no teardown, entries=1, away=1, amend all | 29,426 | 68,476 | 0.3 | - |
| `live-amended-away` | posters=1, workers=1, bodies=0, no teardown, entries=1, away=1, pre=1, repop=1, live=1, amend all | 61,999 | 158,532 | 0.8 | - |
| `deep-frozen` | amend none | 548,193 | 1,813,537 | 10.0 | lost-wake, record-after-detach, token-contended, use-after-free |
| `deep-finding-host-wake` | amend all but host-wake | 361,885 | 1,137,139 | 6.2 | lost-wake |
| `deep-finding-host-excl` | amend all but host-excl, hold-handoff, handoff-idle, handoff-waiting, handoff-giveback | 577,188 | 1,906,407 | 10.5 | record-after-detach, token-contended, use-after-free |
| `deep-finding-host-detaching` | detach, amend all but host-detaching | 382,114 | 1,187,265 | 6.3 | record-after-detach, token-contended |
| `deep-finding-nested` | detach, nested=1, amend all but nested-release | 400,652 | 1,220,874 | 6.7 | record-after-detach |
| `deep-nested` | detach, nested=1, amend all | 404,180 | 1,229,336 | 6.9 | - |
| `deep-failures-waiting` | failures=1, waiting=1, amend all | 796,689 | 2,485,259 | 13.9 | - |
| `deep-three-workers` | workers=3, amend all | 364,236 | 1,179,637 | 7.2 | - |
| `deep-two-posts` | posts=2, pre=1, amend all | 2,796,143 | 9,936,948 | 58.1 | - |
| `deep-detach` | hosts=2, detach, failures=1, amend all | 3,180,277 | 10,934,786 | 64.7 | - |
| `deep-two-hosts` | posters=1, hosts=2, failures=1, waiting=1, amend all | 1,122,487 | 3,428,831 | 22.4 | - |
| `deep-entries` | posters=1, hosts=2, entries=2, failures=1, amend all | 4,814,071 | 15,293,942 | 181.7 | - |
| `deep-away` | entries=1, away=1, amend all | 1,622,436 | 5,223,772 | 57.6 | - |
| `deep-entries-detach` | posters=1, hosts=2, bodies=0, detach, entries=2, failures=1, waiting=1, amend all | 2,632,485 | 7,678,855 | 83.4 | - |
| `deep-live-two-hosts` | posters=1, workers=1, hosts=2, no teardown, pre=1, repop=1, live=1, entries=1, amend all | 1,676,924 | 5,057,074 | 53.4 | - |
| `deep-live-detach` | posters=1, workers=1, hosts=2, detach, pre=1, repop=1, live=1, entries=1, failures=1, amend all | 3,134,547 | 9,706,436 | 106.6 | - |

`--suite deep` runs the quick checks, then the deep ones: about 12 minutes in all here, the quick
ones 28 s; its largest check peaks at a working set of
2.5 GB (deep-entries, 4.8 million states). The defaults
(`posters=2,workers=2,hosts=1,bodies=1,teardown=1`, the timer, one sleeper) are the `amended`
check; posters, posts and hosts multiply the states, workers do not. Larger runs outside the
suites found nothing either. Before the handoff: `--actors posters=3 --opts posts=2` (8,554,393
states, 389 s) and `--actors hosts=2,bodies=2 --opts posts=2` (9,135,178 states, 390 s). With
the handoff and `handoff-idle`, the liveness check (no symmetry reduction): two workers and
`--actors posters=1,hosts=2,teardown=0,detach=1 --opts pre=1,repop=1,live=1,entries=1,failures=1`
(4,330,051 states, 136 s); two workers and
`--actors posters=1 --opts pre=1,repop=1,live=1,entries=2,waiting=1` (2,470,913 states, 66 s);
and `--actors posters=1,workers=1,hosts=2,teardown=0 --opts pre=1,repop=1,live=1,entries=1`
(1,666,942 states, 53 s).

## The router model (S2b, MS)

`router.py` is a third model, separate from the other two: the router protocol across exec groups
of threaded execution's Stage 2 (S2b;
[`docs/STATE_EXEC_THREADED.md`](../../docs/STATE_EXEC_THREADED.md) 4.6, 4.11 and 8.4's MS-1..MS-12;
the plan's S2-Q10 and S2-Q15), as the S2b interface commit froze it in the header comment of
[`exec_router.h`](../../inc/cvc/state/state_exec/exec_router.h): the homes, islands, when the
router is on, the ordered ingress's throw rule, the protocol's entries (send, subscribe,
unsubscribe, deliver, requeue, taken, adopt), broadcast tracking, the cap, and teardown and
rehoming. `--amend none` is S2b.0's text; the default, `--amend all`, is the text as S2b.0a
amended it from this model's findings. It shares only `Violation` with the lock model and has no
C++ port. `test_router.py` tests it.

**The world.** One resolved channel key, homed at a group H that registered its prefix, and at
group 0 (G0) once H's teardown unregisters it; receiving groups R1 and R2, a sender group S (any
group may hold senders, H and G0 included), and X, host threads posting with
exec_pool::post_message. Processes run scripts: `r` a msg-recv, `t` one with a deadline, `s` a
msg-send of a new message, `p` (in X) a host's post of one. A process whose group is the home
receives locally (the FIFO's front, else it parks as a local waiter); elsewhere it parks with a
new recv_seq and subscribes at the home. A send applied at the home goes to every current waiter
(a local one at once, a remote one by a deliver), else joins the pending FIFO. R1, R2 and S
register prefixes of their own elsewhere, so the router is on throughout, unless
`--opts islands=1`: they register none, and once H is gone the key is each island's own (S2b.0a's
island rule; in S2b.0's text, its own while the router is off, which it is once H is gone and no
record is left).

**A step** is one group's thread doing one thing atomically: applying the head of its ordered
ingress, stepping a process, firing a timeout, applying a kill, or one step of its teardown; or
a timed wait's deadline passing (after which the home skips its record and its group may fire the
timeout). A post is an atomic append; a closed handle refuses it, and a home-bound poster
re-resolves the home (with S2b.0a's order, waiting while the home is still that closed group).
Every interleaving is explored breadth first, so each trace is a shortest one. A state is kept as
its pickle with no memo (`encode`; equal states give equal bytes), about 300 bytes with its
index.

**The properties**, each a violation name. At a receipt: `duplicate` (a message reaches a second
wait: one it went pending for, or one outside the waiters its send found once one of those took
it) and `misaddressed`. At a tracker's creation or adoption, and at each answer: `id-collision`
(a home tracks two broadcasts under one id, or an answer lands on another message's tracker).
When a message is first handed to a wait (a receipt or a deliver posted): `fifo` (an earlier
message of the same sender group is not handed out yet: td 4.11's FIFO per (sender group,
channel), as the home's order). When every ingress is empty, no teardown is under way and no
process is runnable: `lost` (neither received nor pending; a fresh send a full FIFO dropped is
today's behaviour, counted, and allowed), `duplicate` (pending twice, or pending and received),
`stranded` (a message pending where a wait not past its deadline is served, or an island's own
message, sent once H was gone, pending elsewhere while that island waits locally: liveness, as
the state graph has no cycle), `orphan-wait` (such a wait with no local waiter or record where
it is served), `stale-record` and `tracker-leak`. No step enabled while not idle is a
`deadlock`. With `--opts choice=1` the requeue's "next waiter" is chosen every way the text
allows, and a step whose choices give different states is `nondeterministic`. The documented gaps
are reported under their own names and do not stop the exploration: `gap-requeue-order` and
`gap-island-rehomed` (below).

**Running it.**

    python -I router.py --procs S:ss,R1:t,R2:t --opts kills=1      # S2b.0a's text
    python -I router.py --procs S:ss,R1:t --but rehome-order        # every amendment but one
    python -I router.py --suite quick     # the CI checks, about 25 s
    python -I router.py --suite deep      # quick, then the larger bounds (locks-deep.yml)
    python -I test_router.py              # unittest: the quick suite, and the explorer's checks

The first prints JSON like `state_word.py`'s, with `peak_mb`. `--opts` sets `kills` (a budget of
kills, any process but X's, any time), `teardown` (H's, default 1), `rdie` (R1's teardown),
`throws` (a deliver that throws before it patches its frame), `icap` (the ingress cap), `fcap`
and `fcap0` (H's and G0's max_pending_messages), `busy0` (a capped send on another key in G0's
ingress at the start), `choice` and `islands`. `--amend` takes `all` (the default), `none`
(S2b.0's text) or names; `--but` every amendment but the ones named; `--mutation` a name.
`--suite` exits 0 only when every check holds.

### What it found in S2b.0's text, and S2b.0a

S2b.0's text had twelve findings. S2b.0a took ten amendments as proposed, answered the eleventh
with the island rule, and closed the broadcast gap with `adopt`; with every amendment nothing is
found at any bound tried but the two documented gaps. Each `finding-*` check runs every amendment
but one and must find its violation.

1. **`rehome-order`: FIFO and stale records across a teardown** (`fifo`, 6 steps;
   `stale-record`, 13). The teardown unregistered, then closed, then forwarded: in between, a
   sender whose cache saw the new epoch posted to G0 directly, ahead of its earlier message still
   queued at H, and a receiver's unsubscribe reached G0 before the subscribe H forwarded, which
   left a record for a wait that ended. S2b.0a: close the handle first, forward to homes resolved
   without closed groups, unregister last; a poster the closed handle refuses waits while the
   home is still that group (the forward waits on nothing, so the wait is bounded). `adopt`
   relies on it (`finding-rehome-order-adopt`: without the wait a late answer reaches G0 before
   the adopt, is applied untracked, and the tracker waits for ever, 14 steps).
2. **`forward-pending-first`** (`fifo`, 9 steps; `duplicate`, 13). The text listed the queued
   entries, then the records, then the pending messages; forwarded in that order, entries H took
   after it applied its pending messages reached G0 first: a queued send overtook its sender's
   pending ones, and two queued subscribes made records at G0 before a pending message arrived
   there as a send, which G0 then broadcast to both waiters (it went pending for one taker).
   S2b.0a: pending messages, then records, then trackers, then the queued entries.
3. **`forward-uncapped`** (`lost`, 9 steps). A queued send was forwarded capped: a new home at
   its ingress cap (other traffic) refused it, and its sender had been told it was sent.
4. **`uncapped-past-cap`** (`lost`, 14 steps, `--opts fcap0=1`). A rehomed pending message was
   applied at the new home as a send, so a FIFO there at its max_pending_messages (a
   per-scheduler field) dropped a message H had accepted.
5. **`stale-rule`** (`duplicate`, 10 steps with `rdie=1`, 10 with `throws=1`). "A deliver as a
   requeue" (a receiving group's teardown) and the throw rule's requeue ignored
   requeue_if_stale: a copy of a message a local waiter at the home took came back and was handed
   to another waiter.
6. **`throw-resubscribe`** (`orphan-wait`, `stranded`, 6 steps). After a deliver threw before it
   patched the frame, the process still waited with that seq, but the home had used its record.
7. **`refused-stale`** (`lost`, 7 steps, `rdie=1`). Nothing said what a home does when a
   receiving group's closed handle refuses a deliver; dropped, the message was lost.
8. **`queue-as-requeue`** (`stranded`, 17 steps). "If every copy comes back stale H queues it
   once": into the FIFO, past a waiter that parked meanwhile.
9. **`unique-ids`** (`id-collision`; its effect, `lost`, 30 steps in
   `deep-finding-unique-ids-lost`). With per-home ids and `adopt`, G0 tracks H's adopted b1 and
   then its own b1: G0's own copy's taken ends H's tracker, and H's stale copy is absorbed by
   G0's, so H's message is lost. (Without `adopt` an orphan answer for H's b1 ended G0's b1 early,
   and a message was delivered twice, 26 steps.)
10. **`next-waiter-order`** (`nondeterministic`, 12 steps, `choice=1`). "The next waiter" was open
    between a local waiter and a record, and among records.
11. **`island-rule`** (`stranded`, 10 steps, `islands=1`). `active()` counts prefixes, so groups
    without one switched between routed and local messaging as the router turned on and off:
    once H was gone, an island's send went to G0 while a record left there kept the router on,
    and its own receiver, parked locally while it was off, never saw it. S2b.0a's island rule (not
    this model's first proposal, the router on while any group exists, which would have routed
    every plain group's own messaging through group 0): a group with no prefix keeps every key no
    other group's prefix covers in its own scheduler, router on or off; its teardown forwards its
    own pending messages as a home's.
12. **`adopt`** (`duplicate`, 15 steps; S2b.0 documented it as a gap). A tracked broadcast whose
    home was torn down with copies outstanding lost its tracking, each stale copy was applied
    untracked at the new home, and the message reached a second wait. S2b.0a: the teardown
    forwards each tracker as an `adopt` (value, id, outstanding, taken) after the records and
    before the queued entries; the new home takes it over.
13. **`forward-host-posts`** (`lost`, 7 steps). A host's post_message queued at a dying home is
    not a router entry; read as "its queued router entries", it died there although the host was
    told it was taken. S2b.0a: forwarded as an uncapped send, in queue order.

Three of the shortest traces, condensed (each command prints its trace in full, with the state
after every step, as the `finding-*` checks' configurations print the others):

`finding-rehome-order`, `fifo` (6 steps):

    python -I router.py --procs S:ss,R1:t --but rehome-order

    p1@S step: posts send(m1) to H                       [H<-[send(m1)]]
    H teardown step: unregisters its prefix               [home G0; H<-[send(m1)]]
    p1@S step: posts send(m2) to G0                       [G0<-[send(m2)]; H<-[send(m1)]]
    G0 drains: applies send(m2): no waiter: joins G0's FIFO
    p2@R1 step: parks (#1); posts subscribe(p2@R1#1) to G0
    G0 drains: applies subscribe(p2@R1#1)  -> VIOLATION fifo: m2 is handed out before m1, sent
        earlier by S

`finding-pending-first`, `duplicate` (13 steps):

    python -I router.py --procs S:ss,R1:r,R2:r --but forward-pending-first

    p1@S step: posts send(m1) to H; H drains: m1: no waiter: joins H's FIFO
    p2@R1 step: parks (#1); posts subscribe(p2@R1#1) to H
    p3@R2 step: parks (#1); posts subscribe(p3@R2#1) to H
    H teardown step: closes its handle; to forward: subscribe(p2@R1#1), subscribe(p3@R2#1),
        send(m1, uncapped)
    H forwards both subscribes; G0 records p2@R1#1 and p3@R2#1
    H forwards send(m1, uncapped); G0 drains: posts deliver(m1) to R1 and to R2 (tracked)
    R1 drains: p2@R1 takes m1
    R2 drains: p3@R2 takes m1  -> VIOLATION duplicate: m1 reaches a second wait

`finding-island-rule`, `stranded` (10 steps):

    python -I router.py --procs R1:r,R1:s,R2:t --opts islands=1 --but island-rule

    p3@R2 step: parks (#1); posts subscribe(p3@R2#1, timed) to H; its deadline passes
    H teardown step: closes its handle; forwards subscribe(p3@R2#1) to G0; unregisters
                                                          [home G0; router off]
    p1@R1 step: parks locally (#1)                        [R1{waiters p1@R1}]
    G0 drains: records p3@R2#1 (past its deadline)        [router on]
    p2@R1 step: posts send(m1) to G0; G0 drains: no live waiter: joins G0's FIFO
    idle -> VIOLATION stranded: m1 pending at G0 while p1@R1 waits (served at R1)

### The documented gaps

The amended text keeps two gaps, and the model reports what they allow (`gap-*` checks: each must
be found):
- `gap-requeue-order`: a requeued message is handed out after messages its sender sent later (15
  steps: m1's deliver goes stale while m2 is handed to the same process's next wait; m1 returns
  to the front of the FIFO and goes to the next receiver). Nothing is lost or duplicated.
- `gap-island-rehomed` (`islands=1`): once H is gone the key is each island's own, so islands no
  longer exchange messages on it: a message pending in one place while a wait is served in
  another, never handed over (5 steps: S's send stays in S's FIFO while R1 parks in its own
  scheduler; or a wait parked at H, rehomed to G0, beside its island's later messages). Nothing is
  lost or duplicated, and an island's own messaging works (an island's own message, sent once H
  was gone, never strands beside its own local wait: that is `stranded`, found only without the
  island rule).

### The mutations

Each breaks one rule of the amended protocol and must be caught (`mutation-*`): `no-requeue` (a
stale deliver dropped: `lost`), `requeue-always` (a stale copy requeued although a local waiter
took the message: `duplicate`), `untracked` (k >= 2 copies untracked: `duplicate`),
`no-unsubscribe` (`stale-record`), `subscribe-no-fifo` (a subscribe recorded beside pending
messages: `stranded`), `deliver-no-seq` (a deliver patches a later wait of its process:
`misaddressed`).

### Its state counts and times

Measured on 2026-11-23 on the Windows builder (Python 3.11, one process, other work beside it),
on S2b.0a's text: the states, the transitions and the seconds of each check, in `--suite
deep`'s run (the quick checks keep their states as tuples, the deep ones encoded).

| Check | Configuration | States | Transitions | Seconds | Found |
|---|---|---|---|---|---|
| `amended` | `S:ss,R1:t,R2:t`, amend all | 65,441 | 158,369 | 2.9 | - |
| `amended-kills` | `S:s,G0:t,R1:t`, kills=1, amend all | 32,694 | 73,123 | 1.4 | - |
| `amended-receiver-teardown` | `S:s,H:r,R1:t,R2:r`, choice=1, rdie=1, throws=1, amend all | 44,167 | 108,123 | 2.1 | - |
| `amended-caps` | `S:ss,R1:t,R2:r`, busy0=1, fcap=1, fcap0=1, icap=1, amend all | 20,156 | 47,819 | 0.7 | - |
| `amended-islands` | `S:ss,R1:t,R2:r`, islands=1, kills=1, amend all | 62,219 | 144,763 | 3.0 | gap-island-rehomed |
| `amended-island-teardown` | `R1:r,R1:s,R2:t`, islands=1, rdie=1, amend all | 10,191 | 24,832 | 0.4 | gap-island-rehomed |
| `amended-host-posts` | `X:p,S:s,R1:t,R2:r`, icap=1, amend all | 25,716 | 58,442 | 0.9 | - |
| `gap-island-rehomed` | `S:s,R1:r`, islands=1, amend all | 91 | 143 | 0.0 | gap-island-rehomed |
| `gap-requeue-order` | `S:ss,R1:tr,R2:r`, teardown=0, amend all | 2,806 | 5,269 | 0.1 | gap-requeue-order |
| `finding-rehome-order` | `S:ss,R1:t`, all but rehome-order | 2,881 | 6,100 | 0.1 | fifo, stale-record |
| `finding-rehome-order-adopt` | `H:s,R1:t,R2:t`, all but rehome-order | 12,542 | 28,738 | 0.5 | duplicate, stale-record, tracker-leak |
| `finding-pending-first` | `S:ss,R1:r,R2:r`, all but forward-pending-first | 3,111 | 6,600 | 0.1 | duplicate, fifo |
| `finding-forward-uncapped` | `S:ss,R1:r`, busy0=1, icap=1, all but forward-uncapped | 630 | 1,235 | 0.0 | fifo, lost |
| `finding-uncapped-past-cap` | `S:ss,R1:r`, fcap0=1, all but uncapped-past-cap | 370 | 680 | 0.0 | lost |
| `finding-stale-rule-teardown` | `S:s,H:r,R1:r`, rdie=1, teardown=0, all but stale-rule | 130 | 224 | 0.0 | duplicate |
| `finding-stale-rule-throw` | `S:s,H:r,R1:r,R2:r`, teardown=0, throws=1, all but stale-rule | 378 | 661 | 0.0 | duplicate |
| `finding-throw-resubscribe` | `S:s,R1:r`, teardown=0, throws=1, all but throw-resubscribe | 17 | 18 | 0.0 | orphan-wait, stranded |
| `finding-refused-stale` | `S:s,R1:r`, rdie=1, teardown=0, all but refused-stale | 58 | 77 | 0.0 | lost |
| `finding-queue-as-requeue` | `S:s,H:r,R1:t,R2:t`, teardown=0, all but queue-as-requeue | 2,292 | 5,496 | 0.1 | stranded |
| `finding-unique-ids` | `S:ss,R1:rr,R2:r,R2:t`, all but unique-ids | 308,949 | 721,143 | 14.0 | gap-requeue-order, id-collision |
| `finding-next-waiter-order` | `S:s,H:r,R1:t,R2:r`, choice=1, teardown=0, all but next-waiter-order | 605 | 1,268 | 0.0 | nondeterministic |
| `finding-island-rule` | `R1:r,R1:s,R2:t`, islands=1, all but island-rule | 2,484 | 5,281 | 0.1 | gap-island-rehomed, stranded |
| `finding-adopt` | `H:s,R1:t,R2:t`, all but adopt | 4,979 | 11,394 | 0.2 | duplicate |
| `finding-forward-host-posts` | `X:pp,R1:r`, all but forward-host-posts | 285 | 514 | 0.0 | fifo, lost |
| `mutation-no-requeue` | `S:s,R1:t`, teardown=0, amend all, mutation no-requeue | 44 | 61 | 0.0 | lost |
| `mutation-requeue-always` | `S:s,H:r,R1:t`, teardown=0, amend all, mutation requeue-always | 110 | 191 | 0.0 | duplicate |
| `mutation-untracked` | `S:s,R1:t,R2:r`, teardown=0, amend all, mutation untracked | 233 | 404 | 0.0 | duplicate |
| `mutation-no-unsubscribe` | `R1:t`, teardown=0, amend all, mutation no-unsubscribe | 7 | 8 | 0.0 | stale-record |
| `mutation-subscribe-no-fifo` | `S:s,R1:r`, teardown=0, amend all, mutation subscribe-no-fifo | 12 | 13 | 0.0 | stranded |
| `mutation-deliver-no-seq` | `S:s,R1:tr`, teardown=0, amend all, mutation deliver-no-seq | 91 | 133 | 0.0 | misaddressed |
| `deep-frozen` | `S:ss,R1:t,R2:r,H:r`, busy0=1, fcap0=1, icap=1, rdie=1, throws=1, amend none | 759,298 | 2,001,279 | 49.9 | duplicate, fifo, gap-requeue-order, lost, orphan-wait, stale-record, stranded, tracker-leak |
| `deep-frozen-choice-islands` | `S:s,H:r,R1:t,R2:r`, choice=1, islands=1, kills=1, amend none | 57,999 | 130,307 | 3.2 | duplicate, gap-island-rehomed, nondeterministic, stale-record, stranded |
| `deep-amended` | `S:ss,R1:t,R2:t,H:t`, kills=1, amend all | 1,365,660 | 3,328,385 | 76.2 | gap-requeue-order |
| `deep-amended-g0` | `S:ss,R1:t,R2:t,G0:t`, amend all | 3,990,975 | 9,748,755 | 229.4 | gap-requeue-order |
| `deep-amended-receiver-teardown` | `S:ss,R1:t,R2:t`, rdie=1, throws=1, amend all | 817,849 | 2,195,387 | 56.3 | gap-requeue-order |
| `deep-amended-choice` | `H:t,S:ss,R1:t,R2:t`, choice=1, amend all | 313,849 | 770,590 | 19.8 | gap-requeue-order |
| `deep-amended-caps` | `S:ss,R1:tr,R2:t`, busy0=1, fcap=1, fcap0=1, icap=1, amend all | 322,622 | 785,129 | 19.0 | gap-requeue-order |
| `deep-amended-two-senders` | `S:ss,R1:st,R2:t`, amend all | 247,353 | 624,086 | 14.9 | - |
| `deep-amended-ids` | `S:ss,R1:rr,R2:r,R2:t`, kills=1, amend all | 3,021,495 | 7,096,132 | 188.8 | gap-requeue-order |
| `deep-amended-islands` | `S:ss,R1:t,R2:t`, islands=1, kills=1, rdie=1, amend all | 691,889 | 1,884,503 | 44.1 | gap-island-rehomed |
| `deep-amended-host-posts` | `X:pp,R1:t,R2:t`, icap=1, kills=1, amend all | 237,595 | 558,445 | 11.9 | - |
| `deep-finding-unique-ids-lost` | `H:s,S:s,R1:tr,R2:tr`, all but unique-ids | 800,596 | 1,797,954 | 33.5 | id-collision, lost |

`--suite deep` runs the quick checks, then the deep ones: 42 checks in about 13 minutes here,
the quick ones about 27 s; the process peaks at a working set of 1.5 GB (`deep-amended-g0`, 4.0
million states, about 300 bytes a state). Processes and messages multiply the states; kills,
deadlines (each adds a clock step) and a second message per sender most of all. Outside the
suites, on S2b.0's text, `--procs S:ss,R1:st,R2:tt --opts kills=1` and the frozen text with
every option at once passed 4 million states and were stopped there, finding nothing new.

## Revisions

`lockmodel.py`'s docstring lists every revision of the model. The design's review ran revision
5b. Revision 5 added protocol 2.1's `progress` cell; 4.4's `attach_home` with its fourth argument
and its empty `cache_refs` entry; 4.11's `detach_home`, which reads no port (the closed assertion
sits in the destroy action, outside the operation); 4.13's `(Pid, time)` sinks; 4.15's `unref`,
which keeps the home's entry; I9's entry exactly for each attached home; and the one-home
configurations. Revision 5b added 4.13's mode and generation check and the group teardown
(`gdestroy`, half of the random worlds with two or three homes, and half of the scripted destroy
pairs). Runs of earlier revisions do not cover those clauses.

### Changes since the revision 5b runs

`explore_scripts.py`'s `nested` mode drew a destroy flag for about 30% of its combinations but
used the default fault kinds, which have no `destroy`, so none of its runs destroyed a home. Its
destroy combinations now add `destroy` to their fault kinds; every combination is otherwise
unchanged. The long run's `nested` phase takes longer than it did.
