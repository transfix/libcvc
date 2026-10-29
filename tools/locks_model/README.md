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
inheritance and their mutations); L3.4 extends it to several.

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
  still be equal. A finding is a `lock_violation` named as here (`I6`, `4.13`, `lockstep`, ...).
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
  below.
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
episodes of the first seeds whose world is one home of prio true. It records every manager
operation the world makes, with its result and notifications, one per line; its docstring
specifies the format. `LocksModel.PythonTraces` replays every file into the oracle and into the
table and requires the recorded results and notifications. Regenerate the committed files after
a change to the model (the output does not depend on the hash seed):

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
| `CVC_LOCKS_MODEL_STEPS` | `RandomL1`, `RandomL2` | 2*10^4 / 5*10^3 / 2*10^3 | 2*10^5 |

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
