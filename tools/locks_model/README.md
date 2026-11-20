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
  with_group that set host_held before the teardown began is the teardown's to wait for;
- the attaching thread: attach's wake, its host bodies (`bodies`, as a host), the servicing of
  waiting bodies, take_failed, and last the teardown or the detach, S2-Q4's five steps.

**The properties** (PL-1), each a violation name. In every state: `double-queue`,
`queued-while-pumped`, `queued-unscheduled`, `two-pumpers`, `pump-host-held`, `token-contended`
(a take finds another thread's token) and `body-without-token`. After the detach:
`wake-after-detach`, `read-after-detach` (a worker or the timer), `record-after-detach` (anyone,
once the attachment is dropped) and `use-after-free`. When every actor is idle (between posts, at
the top of its loop, not started or done, between operations), for an attached group not
detaching: `lost-wake` (a post not drained, or runnable work left, and the group not queued),
`lost-timer` and `stuck-scheduled` (SCHEDULED set, the group in no queue, so every later wake is a
no-op). No step enabled while an actor is not idle is a `deadlock`.

**Running it.**

    python -I state_word.py --actors posters=2,workers=2,hosts=1,bodies=1,teardown=1 --amend all
    python -I state_word.py --suite quick     # the CI checks, about 12 s
    python -I state_word.py --suite deep      # quick, then the larger bounds (locks-deep.yml)
    python -I test_state_word.py              # unittest: the quick suite, and the explorer's checks

The first prints JSON like `explore.py`'s: `states`, `transitions`, `idle_states`, `seconds`,
`complete` and `violations`, each with its detail, its length in steps and its trace. `--actors`
counts posters, workers, hosts and bodies and ends the attaching thread with `teardown=1` (the
default) or `detach=1`; `--opts` sets `posts` (per poster), `timer`, `sleeps` (sleepers the runs
may arm), `waiting`, `failures`, `pre` (posts before attach) and `nested` (a body that detaches
inside its fn); `--amend` takes `all` (the default: the transitions as amended in `exec_group.h`), `none` (as S2a.0
first froze them) or names;
`--mutation` a name. `--suite` exits 0 only when every check holds: the amended transitions find
nothing, each finding is found without its amendment, and each mutation is caught.

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

With every amendment, the host transition reads as follows in the words of `exec_group.h` (the
model runs this; with the fourth finding's other choice its last sentence becomes "Never a
detach or a teardown of g inside fn (asserted)", and step 2 then waits for any host_held). Step 4
should also say that the word is stored 0 only after the notifier is cleared (the mutation
`zero-before-clear` below):

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
    H1: takes Q (PUMPING clear), sets host_held                    0 hh=H1
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
    H1: takes Q (PUMPING clear), sets host_held                    S|N|D queued hh=H1
    H1: releases Q                                                 S|N|D queued hh=H1
    A: step 3: takes the token                                     S|N|D queued hh=H1 tok=A
    H1: takes the token  -> VIOLATION token-contended: H1 takes the token while A holds it

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
    H1: takes Q (PUMPING clear), sets host_held                    0 hh=H1
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
| `amended` | amend all | 233,380 | 734,152 | 7.6 | - |
| `amended-detach` | posters=1, hosts=2, bodies=0, detach, failures=1, amend all | 134,428 | 377,312 | 4.0 | - |
| `amended-waiting` | posters=1, hosts=0, waiting=1, failures=1, amend all | 13,286 | 28,927 | 0.3 | - |
| `amended-nested` | posters=1, workers=1, hosts=0, detach, timer=0, nested=1, amend all | 5,563 | 10,801 | 0.1 | - |
| `finding-host-wake` | posters=0, workers=1, bodies=0, no teardown, timer=0, amend all but host-wake | 409 | 643 | 0.0 | lost-wake |
| `finding-host-excl` | posters=0, workers=1, no teardown, timer=0, amend all but host-excl | 5,500 | 11,044 | 0.1 | token-contended |
| `finding-host-excl-teardown` | posters=0, workers=1, timer=0, amend all but host-excl | 13,992 | 30,397 | 0.2 | record-after-detach, token-contended, use-after-free |
| `finding-host-detaching` | posters=0, workers=1, bodies=0, detach, timer=0, amend all but host-detaching | 1,904 | 3,351 | 0.0 | record-after-detach, token-contended |
| `finding-nested` | posters=1, workers=1, hosts=0, detach, timer=0, nested=1, amend all but nested-release | 5,503 | 10,743 | 0.1 | record-after-detach |
| `mutation-release-store` | posters=1, workers=1, hosts=0, bodies=0, no teardown, amend all, mutation release-store | 395 | 652 | 0.0 | lost-wake |
| `mutation-pump-host-held` | posters=1, workers=1, bodies=0, no teardown, amend all, mutation pump-host-held | 6,398 | 15,668 | 0.1 | pump-host-held, token-contended |
| `mutation-zero-before-clear` | posters=1, workers=1, hosts=0, bodies=0, amend all, mutation zero-before-clear | 1,604 | 3,077 | 0.0 | queued-unscheduled, read-after-detach, use-after-free |
| `mutation-wake-no-recheck` | posters=1, workers=1, hosts=0, bodies=0, amend all, mutation wake-no-recheck | 1,632 | 3,112 | 0.0 | read-after-detach, use-after-free |
| `mutation-pop-no-detaching` | posters=1, workers=1, hosts=0, bodies=0, amend all, mutation pop-no-detaching | 2,350 | 4,592 | 0.0 | read-after-detach, token-contended, use-after-free |
| `deep-frozen` | amend none | 548,193 | 1,813,537 | 20.3 | lost-wake, record-after-detach, token-contended, use-after-free |
| `deep-finding-host-wake` | amend all but host-wake | 230,813 | 726,219 | 8.1 | lost-wake |
| `deep-finding-host-excl` | amend all but host-excl | 577,188 | 1,906,407 | 20.4 | record-after-detach, token-contended, use-after-free |
| `deep-finding-host-detaching` | detach, amend all but host-detaching | 272,282 | 849,696 | 8.3 | record-after-detach, token-contended |
| `deep-finding-nested` | detach, nested=1, amend all but nested-release | 282,471 | 864,970 | 8.7 | record-after-detach |
| `deep-nested` | detach, nested=1, amend all | 285,621 | 872,592 | 8.0 | - |
| `deep-failures-waiting` | failures=1, waiting=1, amend all | 514,469 | 1,595,091 | 15.7 | - |
| `deep-three-workers` | workers=3, amend all | 233,380 | 759,134 | 8.8 | - |
| `deep-two-posts` | posts=2, pre=1, amend all | 1,615,947 | 5,727,147 | 59.0 | - |
| `deep-detach` | hosts=2, detach, failures=1, amend all | 1,574,918 | 5,420,419 | 53.8 | - |
| `deep-two-hosts` | hosts=2, failures=1, waiting=1, amend all | 2,566,739 | 8,949,393 | 94.0 | - |
| `deep-three-posters` | posters=3, hosts=2, failures=1, amend all | 5,304,545 | 19,838,565 | 177.3 | - |

`--suite deep` runs the quick checks, then the deep ones: about 8 minutes in all here, the quick
ones 12 s; its largest check peaks at a working set of 1.3 GB (5.3 million states). The defaults
(`posters=2,workers=2,hosts=1,bodies=1,teardown=1`, the timer, one sleeper) are the `amended`
check; posters, posts and hosts multiply the states, workers do not. Two larger amended runs,
outside the suites, found nothing either: `--actors posters=3 --opts posts=2` (8,554,393 states,
389 s) and `--actors hosts=2,bodies=2 --opts posts=2` (9,135,178 states, 390 s).

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
