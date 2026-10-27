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
