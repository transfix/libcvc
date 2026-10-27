# state_exec benchmark suite

`state_exec_bench` measures what the state_exec DSL runtime costs: time and allocations per step,
per program and per call, latency under load, memory per process, and the threads and handles it
leaves behind. It covers every executor:

| executor    | what runs                                                                                          |
|-------------|----------------------------------------------------------------------------------------------------|
| `recursive` | `cvc::state_exec::evaluator`, the tree walker. Standalone only: no scheduler uses it.              |
| `stackless` | `stackless_evaluator`, stepped by hand (`create_state` + `step` until done).                       |
| `sync`      | `scheduler`, driven in `run(steps)` slices with no time budget (the CLI's `run()` has none).       |
| `async`     | `async_scheduler`, set up the way `app::exec_scheduler()` and pycvc's `Exec` set theirs up (`host_parser_defaults`, `host_data_defaults`, the app tree as watch root) and pumped the way Ariadne and pycvc pump it. |

The numbers are the baseline for single-thread optimization of the runtime and, after that, for a
threaded-execution design. So they must be stable and comparable between runs, and they must show
how costs scale; most scenarios sweep a size for that reason.

The pieces:

- `src/cvc/tests/state_exec_bench.cpp`: the executable (plain `main()`, no gtest), built as
  `state_exec_bench` with the tests (`CVC_BUILD_TESTS=ON`), on every native platform (not
  Emscripten). CTest runs it as `state_exec_bench_smoke` (`--smoke`).
- `tools/bench/state_exec_bench.py`: runs the executable once per scenario, collects JSON lines,
  prints a table, and compares two runs. Standard library only.
- `src/cvc/tests/state_exec_bench_test.cpp`: the older gtest micro-benchmarks (gated on
  `CVC_STATE_EXEC_BENCH=1`). They stay as they are.

## Running it

The executable lands in the build tree's `bin` directory: `build/bin` with the CMake presets
(`build-win/bin` in a Windows tree configured by hand). Examples use `build/bin`.

```sh
# What is there, with default and smoke sizes:
build/bin/state_exec_bench --list

# Every scenario at tiny sizes, about 8 s; fails if any program returns the wrong thing:
build/bin/state_exec_bench --smoke

# A real run: a fresh process per scenario, 5 reps after 1 warmup, the driver pinned to a P-core:
python3 tools/bench/state_exec_bench.py run --pin auto --reps 5 --out base.jsonl

# Some scenarios, a size sweep of your own, one executor:
python3 tools/bench/state_exec_bench.py run --scenario 'flood.*' --size n=100,1000,3000 \
    --executor async --out floods.jsonl

# Summarize, then compare:
python3 tools/bench/state_exec_bench.py table base.jsonl
python3 tools/bench/state_exec_bench.py compare base.jsonl cand.jsonl
```

On Windows use `python` and the executable needs the cvcpkg DLLs on `PATH`. Run it from an
environment that has them, or pass `--dll-dir` to the runner (repeatable). The runner puts the
executable's own directory on `PATH` for you. A full default run (`--reps 1`) takes about
15 minutes on the 22-thread machine this suite was written on; `--reps 5` about
five times that.

### Executable options

| option | meaning |
|--------|---------|
| `--list`, `--list-json` | the scenarios, their executors, default and smoke sizes (`--list-json` is what the runner reads) |
| `--scenario NAME\|GLOB` | run these (repeatable; default all), e.g. `'eval.*'` |
| `--executor E` | only `recursive`, `stackless`, `sync` or `async` (repeatable; anything else is a usage error) |
| `--reps N` | measured reps per sweep point (default 5; 1 with `--smoke`) |
| `--warmup N` | unrecorded reps before them (default 1; 0 with `--smoke`) |
| `--json FILE` | append one JSON record per rep |
| `--smoke` | every scenario at its smoke sizes |
| `--size KEY=V[,V...]` | override a sweep key for every chosen scenario that has it (repeatable); integers are written in canonical form |
| `--pin auto\|none\|CPU` | pin the driver thread (below) |
| `--priority high` | raise the process priority (`HIGH_PRIORITY_CLASS`; `nice -10` on POSIX, which needs privileges) |
| `--timeout S` | give up on a rep after S seconds of driving (default 120); the record carries `ok:false`. A rep whose driver is blocked inside the runtime can't notice, so a watchdog ends the process 60 s past the timeout, after writing an `ok:false` "hung" record for the rep; a sweep point whose setup or teardown blocks (`~app` joins every thread it started) is ended the same way once it outlives all its reps' budgets. The exit code is then 3 |
| `--bg-oversubscribe` | run background-pool points that make more calls at once than the app's pool has workers (skipped otherwise; they stall and can hang: "Known pathologies") |
| `--sample PREFIX` | sample the driver's stack during the first measured rep of each point and, at the end of the run, write `PREFIX.<scenario>.<executor>.<point>.folded` and `.top.txt` (Windows x64; "Profiling") |
| `--sample-us N` | the sampling interval in microseconds (default 1000, at least 100) |
| `--sample-depth N` | frames kept per sample (default 64, at most 1024) |
| `--sample-all-reps` | sample every measured rep, not just the first (files get `.repN`) |

The header gives the compiler, whether `NDEBUG` is set, `sizeof` of `value_t`, `process`,
`eval_frame`, `evaluator_state`, `environment`, `closure` and `native_fn`, the logical CPU count,
the pinned CPU and its SMT siblings, how long `sleep_for(1 ms)` really takes on this machine, the
app's background pool size, what the allocation counter counts, and whether peak memory resets per
sweep point.

The exit code is 1 if any check failed, 2 for a usage error (including a selection that runs no
sweep point), 3 when the watchdog ended a hung rep or point.

### Runner options

`run` takes `--scenario`, `--executor`, `--size`, `--reps`, `--warmup`, `--pin` (default `auto`),
`--priority`, `--timeout`, `--smoke` and `--bg-oversubscribe`, and passes them on. `--split` runs one
process per executor and sweep point instead of one per scenario, so even the points of one sweep
don't share a peak working set or a thread count. `--out` names the JSONL file; it is appended to,
so several runs can go into one file (`compare` and `table` take the median over every rep they
find). `--process-timeout` (default 3600 s) kills a process that outlives it, as a backstop to the
executable's own watchdog. When a process is killed, or exits nonzero without having written an
`ok:false` record (a crash), the runner appends a synthesized `ok:false` record for it.

`compare BASE CAND` matches records on scenario, executor and params. For each metric (default
`wall_ns` and `ns_per_step`; `--metric` for any other field, dotted for nested ones such as
`latency_ns.p99`, `allocs_per_step` or `extra.ns_per_call`) it prints both medians, the change in
percent of the base's magnitude, and the noise: per side, the larger of half the min-max range and 3
times the scaled median absolute deviation, relative to the median; the larger side's counts. A
change is flagged `better` or `WORSE` only when it is larger than both the noise and `--threshold`
(default 3 %). A base median of 0 has no relative change and prints `n/a`. With one rep per side
there is no noise estimate, so only the threshold applies: use 5 reps or more for decisions, and
for derived differences such as `extra.ns_per_call` (a difference of two timed loops) all the more.
A point whose checks failed on either side is reported as `CHECK FAILED` rather than compared, and
a point only the base has is reported as `only in base`; `--fail-on-worse` exits 1 on a regression,
a failed candidate point or a missing one.

## Getting stable numbers

- **Pin the driver to a performance core.** `--pin auto` (the runner's default) picks, on
  Windows, a CPU set with the highest `EfficiencyClass` from `GetSystemCpuSetInformation`, off the
  core that logical CPU 0 is on (most interrupts land there), and pins the driver thread to it
  with `SetThreadAffinityMask` and `SetThreadSelectedCpuSets`. On Linux it picks among the CPUs
  with the highest `/sys/devices/system/cpu/cpu*/cpu_capacity` (hybrid and big.LITTLE parts), or,
  where the kernel reports no capacities, the P-cores listed in `/sys/devices/cpu_core/cpus` (Intel
  hybrid), skipping CPU 0 and its SMT siblings; otherwise it pins nothing. `--pin 2` pins to CPU 2
  anywhere but macOS, which has no thread affinity. On a hybrid Intel part an unpinned driver that
  the OS moves to an E-core runs about half as fast, which is noise far larger than most changes
  worth measuring.
- **Only the driver is pinned.** The app's background threads, lanes, compute pool and the bench's
  own threads float, as they do in a real host. Windows gives a new thread the process's affinity,
  so this holds by itself. Linux gives a new thread its creator's mask, so there the bench builds
  apps, lanes and its threads with the driver's mask widened, and widens every other thread again
  when a measured region begins. Threads the driver starts inside a region (a background kernel's
  thread) would still inherit the pin, so on Linux the scenarios that start threads
  (`async.*`, `msg.ingress`, `mt.*`; `spawns_threads` in `--list-json`) run with the driver
  unpinned, and their records say `pin_cpu: -1`.
- **Keep the pinned core's SMT sibling idle.** The header and the records (`pin_sibling`) name the
  other logical CPUs on the driver's core. A busy sibling slows the driver by tens of percent and
  is a large part of the rep-to-rep spread. On Windows, keep other work off it (for example
  restrict other processes with `SetProcessDefaultCpuSets` or Task Manager affinity); on Linux,
  isolate it (`isolcpus`, `taskset` for other work) or turn SMT off.
- **Power plan.** On Windows use the High performance or Ultimate Performance plan, on AC power.
  On Linux set the `performance` governor (`cpupower frequency-set -g performance`). Turbo still
  varies with temperature; long runs drift.
- **An idle machine.** Close browsers, IDE indexers, builds and sync clients. Antivirus scanning
  of the build directory can stall the first run of a fresh binary; the warmup rep absorbs some
  of it.
- **Reps.** 5 reps and 1 warmup per point is the default. For a decision, run base and candidate
  twice each, interleaved (base, cand, base, cand) into two files, so drift over the session lands
  on both sides. Allocation counts (`allocs_per_step`) are deterministic and show an
  optimization's effect well below the wall-time noise; compare them first.
- **Same build, same sizes.** Compare Release builds (`NDEBUG 1` in the header) from the same
  compiler. A record carries its params, `pin_cpu`, `logical_cpus` and `sleep_1ms_ns`; `compare`
  matches on params, so a size override on one side only shows as "only in base/cand".
- **`--priority high`** keeps other user processes off the driver's core. It doesn't help with
  interrupts and can starve the app's own threads if overused; leave it off for the async
  scenarios unless you are chasing noise.

## What a record holds

One JSON object per measured rep (`rep` 0..N-1; a failed warmup, a hung point and a synthesized
failure are recorded as rep -1). Fields that don't apply are left out.

| field | meaning |
|-------|---------|
| `scenario`, `executor`, `params` | what ran; `params` is the sweep point (keys an executor doesn't vary, such as `driver` and `reap` for `sync`, are left out) |
| `ok`, `error` | whether every program returned what it should (and finished) |
| `wall_ns` | the measured region: first submit (or first step) through the last program's end; setup, parsing for pre-parsed scenarios and teardown are outside it |
| `driver_cpu_ns` | CPU time of the driver thread in the region (`thread_cpu_seconds`: the cycle counter on Windows, `CLOCK_THREAD_CPUTIME_ID` elsewhere) |
| `process_cpu_ns` | CPU time of all threads in the region, threads that exited included (`QueryProcessCycleTime` over the TSC rate on Windows, falling back to the 15.6 ms-tick `GetProcessTimes`; `CLOCK_PROCESS_CPUTIME_ID` on POSIX) |
| `steps_total`, `ns_per_step` | evaluator steps in the region: the scheduler's `total_steps`, or the evaluator's own count; `wall_ns / steps_total` |
| `steps_by_kind` | `eval`, `read` (script read a slice per step), `data` (deep-data slices), `async` (CPU time of async calls charged as steps), summed from `process_info` / `evaluation_stats` |
| `iterations`, `ns_per_iter` | the scenario's unit of work (a loop iteration, a call, a round trip, a parse) and `wall_ns` per unit |
| `allocs_driver`, `allocs_per_step`, `allocs_per_iter` | allocations the driver thread made in the region. POSIX: `operator new` calls (this executable replaces it, and every shared library's resolves to it). Windows: `malloc`/`calloc`/`realloc` calls from every module outside the Windows directory (`cvc.dll` included), counted through their import tables, since each DLL links its own `operator new`. The two counts differ in kind, so compare them within a platform |
| `programs`, `programs_per_s` | programs submitted, and throughput |
| `latency_ns` | `p50`, `p90`, `p99`, `max`, `n` (nearest rank) of submit-to-finish per program, from `execute_options::on_complete` |
| `async_calls`, `async_latency_ns` | async calls made; from each kernel's end to its caller's end (the resume path) |
| `peak_working_set_bytes`, `peak_private_bytes`, `peak_inherited` | peaks (`PeakWorkingSetSize`, `PeakPagefileUsage`; `VmHWM` on Linux, reset per sweep point through `/proc/self/clear_refs`; `resident_size_max` on macOS). Where the peak can't be reset (Windows, macOS) it is the process's, and `peak_inherited: true` marks a record whose process ran an earlier point: its peak may be that point's. `--split` gives every point its own process |
| `working_set_bytes`, `working_set_delta_bytes`, `private_bytes`, `private_delta_bytes` | at the region's end, and the change across it (private: commit on Windows, `RssAnon` on Linux) |
| `heap_delta_bytes` | bytes the C heap handed out across the region (`HeapSummary` on the process heap, `mallinfo2`, `malloc_zone_statistics`), which a working-set delta can't see once a warmup freed the same amount; recorded for `eval.*`, `sched.single.*`, `data.*`, footprint and retention scenarios |
| `thread_count_before`, `thread_count_after`, `thread_count_teardown` | OS threads of the process before and after the region, and after the sweep point's app and schedulers were destroyed |
| `threads_created` | threads the process started during the region, counted as they attach (Windows only: a TLS callback), so a thread that started and exited inside the region still counts |
| `handle_count_before`, `handle_count_after`, `handle_count_teardown` | Windows handles; open file descriptors on POSIX |
| `app_threads_before`, `app_threads_after`, `app_pool_active_after`, `app_pool_pending_after` | the app's thread map (`app::threads()`: entries it has not erased) and its background pool's active and pending counts, for the scenarios that use the background pool |
| `extra` | scenario-specific figures (below) |
| `sampled`, `sample_file`, `sample_top_file`, `samples`, `samples_waiting`, `samples_syscall`, `sample_interval_us`, `sample_timer`, `sample_depth`, `sample_held_us_mean`, `sample_held_us_max`, `samples_truncated`, `samples_unwind_stopped`, `samples_failed`, `samples_overwritten`, `sample_warning` | with `--sample` only: whether this rep was sampled, and for the sampled one its files, the stacks kept, those caught blocked in a wait and those in another system call, the mean achieved interval and the timer behind it, how long a sample held the driver suspended, and the stacks cut short ("Profiling") |
| `pin_cpu`, `pin_sibling`, `logical_cpus`, `sleep_1ms_ns`, `smoke` | the run's conditions (`pin_cpu` -1: this point ran unpinned) |

Thread counts include the OS's own worker threads (on Windows the loader's thread-pool workers come
and go on their own), so read a change of a few threads as noise and a change that grows with `n`
as the runtime's.

Every scenario checks its programs' results: the value each returned (compared as printed), that
every one finished, and where it applies that every process parked, that endless loops are still
running, or that a step count is exact. A failed check makes the record `ok:false` with the reason,
the line on stdout says `FAIL`, and the exit code is 1. A benchmark that measures failing programs
measures nothing.

### How the schedulers are driven

- **sync**: `scheduler::run(200000 steps)` slices, with no time budget, until every program has
  ended. The CLI calls `run()` with neither; the slices only let the bench check its timeout and
  sweep. The sync scheduler has no reap.
- **async**: per pump iteration, `drain_ingress()`, `wake_awaiting()`, then, if `has_runnable()`,
  `sync_run(200000, 0.1)` (Ariadne's pump budget); then the finished programs are reaped
  (`get_process_info`, then `reap`). The async scheduler has no wakeup signal, so when nothing is
  runnable the driver either yields (`driver=spin`, for throughput) or sleeps 1 ms
  (`driver=poll1ms`, what pycvc does when its program is parked). The record's `driver` param says
  which.
- **host**: by default (`host=bare`) the sweep looks only at the programs that just finished and no
  limits are armed: a lower bound. Ariadne arms `max_steps` 5000, `max_time` 0.1 s and `max_memory`
  4 MiB on every action and sweeps every live action with `get_process_info` on every pump
  (`ariadne.cpp:1675-1730`); `flood.small_host` runs both (`host=ariadne`).
- Each scheduler gets the app's state tree as its watch root and the intrinsics (`msg-*`,
  `state-*`, ...) registered with that tree as their root, like a host. No `state_cluster_shard` is
  installed, so `msg-send`'s write into the tree (`state::sendMessage`) returns `no_shard` after
  its registry lookup (`state.cpp:1613-1617`); Ariadne and arisrv hosts install one and pay the
  shard's send path, which these numbers leave out.
- A sweep point gets one `cvc::app`, created before its reps and destroyed after them, and each rep
  a fresh scheduler, host and environment (outside the measured region). The scenarios on the
  app's background pool (`async.*`) build a fresh app per rep instead, outside the region, because
  the pool accumulates state (see "Known pathologies"); a shared app would make the reps a trend,
  not samples. `async.pool_aging` and `async.pool_batches` measure that accumulation on purpose.
- Processes run in their own scope over one shared environment (`environment::extend`): the
  builtins, intrinsics and natives are shared, while each process's top-level `set`s stay its own,
  so concurrent programs don't clobber each other's globals. That is one scope level more than
  pycvc, which runs its one program in its shared environment directly (`pycvc_exec.cpp:565`), and
  than Ariadne, which gives each action a whole default environment (`ariadne.cpp:1657`): every
  builtin lookup walks one more scope. `env=own` (`flood.small`, `flood.small_host`) gives each
  process a whole default environment instead.

## Profiling

A record says how fast a point ran; `--sample` says where the time went. It is a sampling profiler
inside the bench, because the system profilers need rights a developer box often lacks: WPR/xperf
and the Visual Studio collector (`VSDiagnostics`) need elevation (its `stop` fails with
`E_ACCESSDENIED` without it). An in-process sampler needs none, and it sees exactly the measured
region and nothing else.

```sh
# The first measured rep of each point, sampled every 1 ms; files land next to the prefix when
# the run ends:
build/bin/state_exec_bench --scenario sched.single.arith_loop --executor sync \
    --size n=200000 --reps 1 --sample prof/arith
# -> prof/arith.sched.single.arith_loop.sync.n=200000.folded
#    prof/arith.sched.single.arith_loop.sync.n=200000.top.txt
```

Use a build with symbols: Release flags plus PDBs (`/Zi` for the compiler, `/DEBUG` for the
linker, so `/O2` code is what gets sampled). Without PDBs the stacks still come out, named by
export or by `module+offset`.

How it works. When the sampled rep's measured region begins (the meter's `begin()`, where its
clocks start), a sampler thread started at launch wakes every `--sample-us`: it calls
`SuspendThread` on the driver and `GetThreadContext` (control and integer registers only), unwinds
the driver's stack in place with `RtlLookupFunctionEntry` (with a per-sample history table) and
`RtlVirtualUnwind`, writes up to `--sample-depth` addresses into a ring allocated and faulted in at
startup, and calls `ResumeThread`. Only the top frame may lack unwind info (a leaf, whose return
address is at `[RSP]`); a deeper frame without it stops the walk, and the stack is marked
`[unwind stopped]` rather than followed through a guessed address. The walk's own reads stay
between the suspended `RSP` and the stack's base. While the driver is suspended the sampler
allocates nothing, takes no lock and prints nothing, since the driver may hold the heap lock. A
second thread watches each sample from the moment the driver is suspended until the sampler goes
idle after the region: if one keeps the driver suspended over 50 ms (the unwinder waiting on a
lock the driver holds, say), it resumes the driver, the unwind stops, the sample is dropped and
sampling is off for the rest of the region; the record's `sample_warning` says so. A sample that
would land after the region has ended (in the meter's closing probes) is dropped too. The handle
on the driver is opened at launch, so the sampled rep's `handle_count` matches the others'.

The interval comes from a high-resolution waitable timer (Windows 10 1803+), else
`timeBeginPeriod(1)` and `Sleep`, else spinning on `QueryPerformanceCounter`, whichever first keeps
the mean interval within 25 % at startup. With `timeBeginPeriod` the raised timer rate is on only
while a sampled region runs (it makes every 1 ms sleep in the process last 1 ms instead of
`sleep_1ms_ns`, so a `poll1ms` driver's sampled rep runs differently from its others). Ticks are
paced against a schedule: the high-resolution timer fires on a ~0.5 ms grid, so a single 1 ms wait
takes 1.5 ms, but a late tick shortens the next wait and the mean stays at 1 ms (the jitter also
keeps the samples from locking onto a periodic loop). The header line names the timer and any it
passed over; the records carry the achieved mean (`sample_interval_us`). Both threads start
floating before the driver is pinned and are then moved off the driver's CPU and its SMT sibling,
so a spinning sampler never competes with the driver for its core (the header says so).

A sample whose RIP sits at the return of a system-call stub in `ntdll.dll` or `win32u.dll` (the
bytes before it are `syscall`, the bytes at it the `ret; int 2Eh; ret` every x64 stub ends with,
so a stray `0F 05` inside other code does not count) caught the driver in the kernel. The stub's
export name says which kind: a wait (`NtWait*`, `NtSignalAndWait*`, `NtDelayExecution`,
`NtRemoveIoCompletion*`, `NtReplyWait*`, `NtUserMsgWait*`, `NtUserWaitMessage`) counts in
`samples_waiting`, its folded stack ends in `[waiting]`, its self time goes to `[waiting] <stub>`,
and the top list breaks the waits down by caller (the first frame outside ntdll, KERNELBASE,
KERNEL32 and win32u); any other system call (`NtAllocateVirtualMemory`, `NtYieldExecution`, ...)
counts in `samples_syscall` and ends in `[syscall]`.

When the run ends, with every point measured, the bench loads DbgHelp (at run time, so a run
without `--sample` never loads `dbghelp.dll`), calls `SymInitialize` on the process, searching the
exe's directory and the directory of the `cvc.dll` that was loaded, with `SYMOPT_UNDNAME |
SYMOPT_DEFERRED_LOADS | SYMOPT_LOAD_LINES`, names each address (`SymFromAddr`, cached per address;
a return address is looked up one byte back, at its call) and its module (`SymGetModuleBase64`),
and writes for each sampled rep:

- `<stem>.folded`: collapsed stacks, `outer;...;inner count`, one line per distinct stack. Names
  lose `cvc::state_exec::`, template argument lists over 24 characters become `<...>`, and frames
  outside `cvc.dll` and the bench carry their module (`ntdll!RtlAllocateHeap`). Functions are told
  apart by module and full name: two that shorten to the same label get ` #2`, ` #3`, ... (in the
  order the run met them), so overloads and instantiations stay separate. A stack cut at the depth
  starts with `[truncated]`, one stopped at a frame without unwind info with `[unwind stopped]`.
- `<stem>.top.txt`: the sample count and achieved interval, how long samples held the driver,
  running versus waiting versus other system calls, the waits by caller, then the top 40 functions
  by self samples, the top 40 by inclusive samples (a function counted once per stack it is on),
  and the top 20 source lines by self samples (`SymGetLineFromAddr64`, per line and function), as
  percent of the samples. A failed `SymInitialize` is a `WARNING` line here and on stderr.

To view a folded file, drop it on [speedscope.app](https://www.speedscope.app) (it opens collapsed
stacks as they are; "Left Heavy" and "Sandwich" are the useful views), or make an SVG with
`flamegraph.pl stem.folded > stem.svg` from Brendan Gregg's FlameGraph scripts.

Limits:

- Windows x64 only (MSVC or clang-cl, which have `__try`). Elsewhere `--sample` says so on stderr
  and the run goes on unsampled. On Linux use perf on the whole process, e.g.
  `perf record -F 999 -g --call-graph dwarf -- build/bin/state_exec_bench --scenario ... --reps 1`,
  then `perf script | stackcollapse-perf.pl` for the same folded format; it sees setup and
  teardown too, so keep the sizes large enough that the region dominates.
- Only the driver thread is sampled. Scenarios whose work runs on the app's pools or lanes
  (`async.*`) show the driver waiting; their worker threads are not followed.
- The sampled rep is slower (every sample stops the driver for a few to tens of microseconds) and
  its `process_cpu_ns` includes the sampler's thread. Only the first measured rep is sampled unless
  `--sample-all-reps` is given, so the other reps' timings stay clean; compare timings from
  unsampled runs only.
- `--sample` costs memory and handles for the whole run, sampled reps or not: the ring (33 MB at
  the default depth), the two threads and their handles, so its records show two more threads,
  about six more handles and about 35 MB more private memory than an unsampled run's. Each sampled
  rep's stacks are copied out of the ring until the run ends (a few hundred KB per sampled
  second), which later records' memory includes. DbgHelp and its PDBs (over 100 MB with `cvc.pdb`)
  load only after the last point, so no record's memory, handles or peaks include them; the price
  is that the files appear only at the end, and a run the hang watchdog ends (exit code 3) writes
  none.
- Without `--sample` nothing runs: no thread, no ring, no `dbghelp.dll` (it is loaded at run time,
  not imported), one false flag tested per region.
- A sample sees where the driver is, not whether it had a CPU: on a busy machine a driver preempted
  in user code is sampled as running there. `[waiting]` only marks a driver blocked in a waiting
  system call.
- The ring holds about 65 s of 1 ms samples at depth 64 (4 Mi frame slots, never more than 1 Mi
  samples); a longer region keeps the last ones (`samples_overwritten`). Stacks deeper than
  `--sample-depth` (deep recursion in the recursive evaluator) lose their outer frames, so their
  inclusive counts are low; raise the depth.
- The stack is unwound in place while the driver is suspended, as asked of the design, rather than
  copied and unwound after resuming it (as Chromium's sampler does). For code in loaded images
  `RtlLookupFunctionEntry` reads the loader's tables without blocking, but code registered at run
  time (dynamic function tables, JIT code) is looked up under a lock the driver may hold; the
  watchdog is the backstop for that.
- Inlined functions are attributed to the function they were inlined into (DbgHelp names the
  physical function); source lines still point into the inlined code.
- Without Windows' own PDBs, frames in `ntdll.dll` and the other system DLLs are named after the
  nearest export. Where the image's `.pdata` puts the start of the function holding the address
  past that export, the frame is named by that start instead, so a heap internal shows as, say,
  `ntdll!RtlAllocateHeap+0x1200` (the function starting 0x1200 bytes past `RtlAllocateHeap`), and
  distinct internals stay apart. Read them as "inside ntdll". `_NT_SYMBOL_PATH` (and
  `_NT_ALT_SYMBOL_PATH`) are added to the search path, so a local symbol cache or a symbol server
  there names them properly (a server also needs `symsrv.dll` beside the `dbghelp.dll` that gets
  loaded).
- The sampled rep's numbers carry the sampling: each sample holds the driver suspended for about
  30 us (`sample_held_us_mean`; on the machine this was written on, about 3 % at 1 ms), and caches
  disturbed by the sampler cost a little more.

## Scenarios

`--list` prints the sizes. Each scenario runs every executor it lists at every sweep point.

### eval.* (recursive, stackless)

The bare evaluators, run to completion on a pre-parsed AST: `evaluator::evaluate` for `recursive`,
`create_state` and `step` until done for `stackless`. The same program for both:

| program | work per iteration |
|---------|--------------------|
| `arith_loop` | `(set s (+ s i))` in a `while` loop |
| `fib` | a call of a recursive `defun` (an iteration is a call: `2 fib(n+1) - 1` of them) |
| `closure_calls` | a call of a stored `lambda` |
| `closure_map` | `collections.map` with a `lambda` over a list (stackless only: the recursive evaluator's natives can't call DSL functions) |
| `list_build_walk` | `append` to a list, then a `for` over it |
| `dict_ops` | `get-attr` + `set-attr` on a dict of `k` keys (dicts are linear vectors: `k` 4, 64, 512) |
| `string_ops` | `str`, `str-concat`, `string.substring`, `string.length`; `lit=long` uses a 24-character literal, past the 15-character small-string buffer |
| `exceptions` | `try` / `raise ValueError` / `catch` |
| `generators` | `next` on `(range n)` until it returns nil |
| `for_range` | `for` over `(range n)` (stackless only: the recursive `for` takes lists only) |
| `native_calls` | a registered trivial native, `(bench-inc s)` |
| `closure_body` | a call of a `defun` of `p` parameters (0, 1, 4) whose body is 1 or 3 forms (`forms`): a binding per parameter, and a body of more than one form evaluated as a `(begin ...)` list (rebuilt on every call when this suite was written; see "Known pathologies") |
| `lookup_depth` | the arith loop over globals run `depth` scopes down (0, 4, 16 nested closures), so each reference walks `depth + 1` scopes; `names=long` uses 24-character names |
| `try_noraise` | a `try` whose body doesn't raise: the form's setup alone |
| `raise_depth` | a `raise` `d` calls deep (0, 16, 64), caught at the top: traceback capture and unwinding |
| `yield_gen` | a value from a DSL `generator` that `yield`s (stackless only) |
| `tail_rec` | one level of a tail-position self call `n` deep (1e3, 1e4, 1e5; no TCO, so the stack grows: `heap_delta_bytes`); stackless and schedulers only, the recursive evaluator would overflow the C++ stack |
| `range_create` | `(next (range 3))`: a generator built and stepped once |
| `method_send` | `(send o "m" s)` with `m` defined `depth` classes up the `__super__` chain (1, 4; stackless only) |

Read `ns_per_iter` across executors: that is the same work. `ns_per_step` is not comparable
between them: a recursive "step" is an `eval_form` (one per AST node), a stackless step is one
phase of a frame (about twice as many).

### sched.single.* (sync, async)

The `eval.*` programs from `arith_loop` to `native_calls`, and `tail_rec`, as one process on each
scheduler, submitted as a pre-parsed AST. The difference from `eval.*.stackless` in `ns_per_iter`
(both run the stackless evaluator) is the scheduler's per-step overhead: selecting the process, the
per-step guards and clock reads, and on the async scheduler the ingress drain and the coroutine
frames.

### flood.small (sync, async)

`n` tiny programs, half `(+ 1 2)` and half a 10-iteration loop, submitted at once as source text
(so parsing, or on the async scheduler the in-process read, is part of each), then run to the end.
It reports per-program latency, throughput, peak memory, the processes left behind
(`extra.processes_left`) and the submit phase's share (`extra.submit_ns_total`,
`extra.ns_per_submit`). On the async scheduler `reap=1` reaps each program as it ends and `reap=0`
doesn't. `env` is `extend` or `own` (above). `n` is 100, 300 and 1000 by default: every step scans
every process the scheduler holds, so a flood submitted at once costs O(n^2) on both schedulers
(n=1000 takes 2-3 s a rep; n=10000 several minutes). Sweep further with
`--size n=3000,10000 --timeout 900`. Since everything is submitted at once, most processes are
live for most of the run whether or not they are reaped; `flood.stream` separates the cost of
finished processes kept.

### flood.small_host (async)

`flood.small` at `n=300` with `host=bare` against `host=ariadne` (Ariadne's limits armed, every
live process swept each pump), `watch=1` against `watch=0` (no watch root: no settings reads per
submit and per charge from the tree), and `env=extend` against `env=own`.

### flood.large (sync, async)

`p` loops at once, `work / p` iterations each, so the total work is the same at every `p`
(1, 10, 100, 1000). At large `p` each loop is short (at `p=1000`, 10 iterations, the same program
as `flood.small`'s loop), so parse, submit and exit weigh in; `sched.scaling` measures the pure
per-step cost against the process count. `extra.finish_spread` is (last finish - first finish) /
last finish: near 0 means round robin kept them level.

### flood.mixed, flood.mixed_stream (sync, async)

`flood.mixed`: `large` long loops submitted first, then `small` tiny programs, all at once.
`latency_ns` is the small programs' submit-to-finish latency, `extra.latency_large_ns` the long
ones'. On the sync scheduler the finished small programs stay in every scan the long loops' steps
make, which is most of the sync-async gap here.

`flood.mixed_stream`: `live` endless loops (10, 100) start first; then one `(+ 1 2)` is submitted
per pump of `slice` steps, as Ariadne submits an action per frame, until `small` of them have
finished. `latency_ns` is each small program's submit-to-finish latency at a fixed live set. The
loops are killed afterwards (and checked to be still running).

### flood.stream (sync, async)

`r` batches (20) of `b` (500) `(+ 1 2)` programs into ONE scheduler, each batch run to the end
before the next is submitted. `extra.batch_ns` (every batch), `batch_first_ns`, `batch_last_ns` and
`batch_last_over_first`: batch time rising with the batch index is what keeping finished processes
costs (the sync scheduler never erases one; async `reap=0`).

### sched.scaling, sched.bystanders, sched.watch_poll (sync, async)

Steady-state cost per step, with nothing submitted or finishing in the region:

- `sched.scaling`: `p` endless loops (1 to 2000), warmed up, then exactly `steps` (50000) steps
  timed: `ns_per_step` against the number of runnable processes. `extra.fairness` is the fewest
  steps a process got over the most. The step count is checked exactly, and every loop must still
  be running.
- `sched.bystanders`: one endless loop beside `b` processes (0 to 3000) parked on a channel nobody
  sends to: the loop's `ns_per_step` against `b`, since the scans walk parked processes too.
  `extra.loop_share_of_steps` is the loop's share of the steps (below 1 means parked processes were
  stepped).
- `sched.watch_poll`: the same with `w` parked processes (0 to 1000) that each registered a
  `state-watch` (`watch=1`) or not (`watch=0`, the control): every step polls every watch.

### sched.sleepers (sync, async)

`n` processes (1, 100, 300) each `(sleep 0.001)` 20 times (n=1000 took 13-21 s a rep). `extra.driver_cpu_per_wall` near 1.0
means the driver spun while everything slept; `extra.lateness_ns` is each program's latency minus
the 20 ms it asked for; `extra.steps_per_wall_s`.

### sched.gc_spike (sync)

One process holding a list of `n` items (0, 1e4, 1e5) while it makes 2000 closures in fresh
scopes, stepped one `step()` at a time with each step timed. `extra.step_ns` (p50/p99/max): an
unsliced collection or release shows as the max.

### async.flood.noop, async.flood.sleep1ms, async.flood.cpu100us (async, sync)

`n` programs (10, 100, 1000), each calling one async function once, registered with
`register_async_fn` on an `async_host`. The kernel returns its argument at once (`noop`: the call's
pure overhead), after blocking for 1 ms (`sleep1ms`: I/O-like; on Windows a high-resolution
waitable timer, since `sleep_for(1 ms)` there lasts a ~15.6 ms timer tick), or after 100 us of busy
CPU (`cpu100us`). `where` picks the executor:

- `background`: the app's background task pool (`async_host(async_scheduler, app)`, the same as
  `app::exec_async_host()`); the process parks until the kernel's result comes back through the
  ingress.
- `lane`: a `cvc::async_lane` added with `add_lane`: one persistent thread, FIFO.
- `inline`: the sync scheduler's host; the kernel runs inside the step, which blocks.

Both drivers run on the async scheduler. `async_latency_ns` is the time from a kernel's end to its
caller's end (the completion path: ingress, drain, wake, resume). `extra.kernel_threads` is the
number of distinct OS threads the kernels ran on (one per call on the background pool, one for a
lane); `threads_created` and `app_threads_after` show the same from the process and the app.

With `where=background`, a point that makes more calls at once than the app's background pool has
workers (`app::getThreadPoolSize()`, the hardware concurrency) is skipped unless
`--bg-oversubscribe` is given (`n` here, `k` for `async.futures`, `b` for `async.pool_batches`, `c`
for `async.inflight_tax`, `m` for `async.compute_fanout*`): past the pool size the pool stalls 5 s
per key collision and can hang (see "Known pathologies"). The skip is printed. With `--smoke` the
count is clamped to the pool size instead, so hosted CI runners with 2-4 workers still run the path.
To measure it: `--scenario async.flood.noop --size n=100 --executor async --bg-oversubscribe
--timeout 60`; the watchdog ends the process if it hangs.

### async.futures (async, sync)

One program launches `k` futures with `<fn>-async` and waits with `await-all`; `kernel` is `noop`
or `sleep1ms`, `where` as above. `extra.await_all_tail_ns` is the time from the last kernel's end
to the program's end.

### async.pool_aging, async.pool_batches, async.pool_retention (async)

What the background executor accumulates in one app, against a lane:

- `pool_aging`: ONE program making `k` sequential calls (100, 1000, 3000) with at most one in
  flight, so the pool never oversubscribes. `extra.first_ns_per_call` and `last_ns_per_call` are
  the mean time per call over the first and the last 10 % (from the kernels' end times), and
  `last_over_first` their ratio; `app_threads_after` is the app's thread-map entries.
- `pool_batches`: `r` batches (20) of `b` (16) concurrent calls into one app: `extra.batch_ns`
  against the batch index.
- `pool_retention`: `k` (100) sequential calls whose kernel returns a 1 MiB string that the program
  drops: `extra.private_bytes_per_call` and `heap_bytes_per_call` near 1 MiB would mean every
  call's result is kept until the app goes.

### async.inflight_tax (async)

`c` callers (0, 1, 4, 8; `--size c=20` for more) blocked in async calls that stay in flight (on the background pool, or on
`c` lanes, one per caller), then one `arith_loop` process (`n` 10000) timed: its `ns_per_step`
against `c`. `rate=tree` leaves the callers' step rate to the tree, read at every charge;
`rate=opt` sets `execute_options::async_steps_per_cpu_ms` on them. `watch=0` (sweepable) drops the
watch root. The latch is released after the region and every caller must return its argument.

### async.compute_fanout, async.compute_fanout_narrow, async.compute_fanout_herd (async)

Kernels that run `parallel_for` over `app.computePool()`:

- `compute_fanout`: `m` processes (1, 4, 16), each making `r` calls of 64 chunks of 50 us, as wide
  as the pool. Each fan-out saturates the pool alone, so serializing them costs no throughput.
  `extra.pool_efficiency` is the chunk work over (wall time x pool concurrency).
- `compute_fanout_narrow`: 4 chunks of 1 ms at most 4 wide, `m` 1 and 4, `r` 5: fan-outs that
  could run side by side. `extra.fanout_stretch` is the median fan-out's time over the time it
  takes alone (about 1 if concurrent fan-outs overlap, about `m` if they serialize);
  `extra.fanout_wait_ns` is the time from calling `parallel_for` to its first chunk starting.
- `compute_fanout_herd`: `r` (200) fan-outs of 22 chunks of 1 us at most 2 wide:
  `extra.fanout_ns` is mostly waking the pool's workers.

### msg.* (sync, async; msg.ingress async)

- `pingpong`: two processes, `r` (2000) round trips of `msg-send` / `msg-recv`, beside `bystanders` (0,
  100, 1000) processes parked on another channel; `extra.round_trip_ns`.
- `producer_consumer`: one producer `msg-send`s `m` messages, one consumer receives them
  (`max_pending_messages = 0`, unbounded, so nothing is dropped); `extra.messages_per_s`.
- `broadcast`: `k` receivers parked on one channel, one sender; per round the sender polls a
  native (`bench-parked`, which reads the `k` receivers' status) until all `k` are parked, sends
  once, and collects `k` acks. `extra.ns_per_delivery` is per receiver per round, the polls
  included; `extra.parked_polls` and `parked_poll_ns` are the polls' count and time, and
  `ns_per_delivery_without_polls` takes their time out (the polling loop's own DSL steps stay in).
- `fanout`: one sender cycling `m` rounds over `k` receivers (10, 100, 300; k=1000 took ~24 s a rep), each on its own
  channel, no gate (a send to a receiver that isn't parked queues): `extra.ns_per_delivery`.
- `ingress`: `t` native threads (1, 4) calling `async_scheduler::post_message`, the thread-safe
  entry, into `r` parked receivers: `extra.messages_per_s` and `extra.post_to_receive_ns`.

### native.* (sync, async)

The per-call cost of one call of a kind: a loop of `n` iterations with the call against the same
loop without it, `rounds` (3) of each alternated within every rep, the fastest of each kept. The
record is the fastest run with the call; `extra.baseline_wall_ns` and `baseline_steps` are the
loop's without it; `extra.ns_per_call` is the difference over `n`; `extra.steps_per_call` the
evaluator steps the call form adds; `extra.native_ns_per_call` the difference less those steps at
the baseline's ns/step, the call itself. `plain` is `(bench-inc i)`; `apply` is
`(apply bench-inc (list i))`, the list included; `state_get` and `state_set` read and write a node
`depth` levels down (1, 4, 8); `state_delete` is a `state-set` and a `state-delete` with `nodes`
(100, 10000) other nodes under the root, which `state-delete`'s expiry sweep walks; `msg_send`
sends to a channel nobody receives on, so messages queue up to `max_pending_messages` (1024) and
are then dropped. These are differences of two timed loops: use 5 reps or more before reading a
change in them.

### footprint.idle_process (sync, async)

`n` processes (1000, 3000) parked on `msg-recv`: `extra.heap_bytes_per_process`,
`working_set_bytes_per_process` and `private_bytes_per_process` are the deltas across submitting
and parking them, divided by `n`. The heap figure is the reliable one. The region ends when every
process has parked, so it holds no time with everything idle; `sched.sleepers` and
`sched.bystanders` measure what idle processes cost.

### footprint.closure, footprint.closure_forms, footprint.environment (stackless)

`n` closures kept in a list: sharing one environment (`closure`, a one-form body; `closure_forms`,
a body of two forms, which a call evaluates as a `(begin ...)`), or each over an environment of its
own made by a call (`environment`). `extra.heap_bytes_per_item`; the difference between `closure`
and `environment` is what an environment costs.

### limits.* (sync, async)

The `arith_loop` process with one resource limit armed, set beyond what it uses, so only the
guard's cost is measured: `max_time` 1e6 s, `max_memory` 1 GiB, `max_steps` 2^62, and
`max_step_time` 1e6 s (async only: the sync scheduler ignores `max_step_time`). Compare
`ns_per_step` with `limits.none`.

- `max_time_ariadne`: `max_time` 0.1 s, Ariadne's action default, on a loop sized (`n` 8000) to run
  ~70 ms, so its second half runs past the budget's midpoint, where every step also reads the
  thread's CPU time. It must still finish. Resize `n` on a much faster or slower machine.
- `max_memory_bigheap`: the loop holding a list of `b` (100000) items, `mem=none`, `mem=1g`, or
  `mem=near` (`max_memory` = what the process measures to unlimited, x 2; a tighter budget
  makes the async scheduler's sliced `collect` hit the allocation limit). `extra.heap_measurements` counts the measurements.

### data.copy_equal, data.print_floats, data.release (stackless, sync, async)

Deep data, `n` 1e3 and 1e5 (1e3 and 5e3 for `print_floats`: `str` caps its result at 64 KiB): a `slice` copy of a list and `=` of the two (`copy_equal`), `str` of a
list of floats (`print_floats`: one formatted double each; the stackless evaluator's answer is the
reference the schedulers must match), and building then dropping a nested list (`release`). On the
schedulers `deadline=1` arms `max_time`, which adds a clock read per node in the walks.
`extra.ns_per_node`; `steps_by_kind.data` shows the slicing on the async scheduler.

### mt.independent (stackless)

`t` threads (1, 2, 4, 8), each with its own stackless evaluator and default environment, running
`fib 18` or 2000 caught raises `runs` (5) times; `ast=private` parses per thread, `ast=shared` runs
one parsed AST on every thread. `ns_per_iter` is aggregate; `extra.speedup` is t x the single-thread
time (measured in the same rep) over the wall time; `extra.thread_ns_per_iter` is per thread;
`extra.allocs_per_iter_all_threads`; `extra.threads_on_lower_efficiency_class` (Windows) counts the
threads that ended on a slower core class. These are the "before" numbers for threaded execution:
refcounts on a shared AST, the global exception class tables and the allocator under contention.

### parse.small, parse.large

`parse()` of a small program (a `fib` defun) `r` times, and of a generated ~`kb` KiB program
(defuns with strings, floats, nesting and dict calls) `r` times. `extra.ns_per_byte` and
`extra.mb_per_s`. No executor.

## Known pathologies the suite exposes

Each names the scenario that shows it; the file:line references are to the runtime at the time this
suite was written.

- **No run queue.** Both schedulers keep processes in an `unordered_map` and, on every step,
  rebuild a vector of the runnable ones by scanning all processes, sort it by pid and index it
  (`scheduler.cpp:234-281`, `async_scheduler.cpp:102-146`), on top of wake scans
  (`scheduler.cpp:547-563`; twice per step on the async scheduler, `async_scheduler.cpp:314, 542`)
  and a `has_runnable()` scan per `run()` iteration (`scheduler.cpp:462, 477-489`). A step costs
  O(P log P) in the processes it holds: `sched.scaling` `ns_per_step` against `p`, and
  `sched.bystanders` against parked processes.
- **The sync scheduler never forgets a process.** It has no reap, so finished processes stay in
  every scan: `flood.stream` batch time against the batch index on `sync` (and `async reap=0`),
  `flood.small` `extra.processes_left`. First run: the 20th batch of 500 took 16 times the first
  on `sync`, 14 times on `async reap=0`, and the same as the first with `reap=1`.
- **The sync scheduler busy-spins while everything waits.** Its `has_runnable()` counts processes
  waiting on a message or a sleep as runnable (`scheduler.cpp:477-489`), so `run()` keeps calling
  `step()`, which finds nothing to run, at 100 % CPU: `sched.sleepers` on `sync`
  (`extra.driver_cpu_per_wall` about 1.0), and the cost of parked processes in `msg.pingpong
  bystanders=...` on `sync`. First run: 1000 processes sleeping 1 ms 20 times took 13 s on
  `sync` (21 s on `async` with `driver=spin`, whose wake scans walk every process too).
- **Async scheduler step overhead.** Every step drains the ingress under a mutex even when it is
  empty (`async_scheduler.cpp:313, 523-543`) and runs through two heap-allocated coroutine frames
  consumed at once by `sync_wait` (`async_scheduler.cpp:229, 310`;
  `async_stackless_evaluator.cpp:40-44`): `sched.single.*` async against sync, and their
  `allocs_per_step`.
- **In-flight async calls tax every step.** Each step copies the list of in-flight calls and, for
  each started one, charges it, reading `limits.async_steps_per_cpu_ms` from the state tree unless
  the process set it (`async_scheduler.cpp:590-618, 638-650`): `async.inflight_tax` `rate=tree`
  against `rate=opt`. First run: with 8 calls in flight an unrelated step cost ~25 us with the
  tree read and ~1 us without it.
- **The background async executor starts an OS thread per call and never forgets it.**
  `app::startThreadPooled` creates a `boost::thread` per task (`app.cpp:1075-1095`), keeps the entry
  in `_threads` until `~app` (`app.cpp:741-757`), finds a unique key by probing
  `state_exec.async.<fn>0`, `1`, ... under a mutex (`app.cpp:780-788`) and rebuilds the key map on
  every start (`app.cpp:594-635, 915-948`); `async_host` submits with `wait=false`
  (`async_host.cpp:530`). `extra.kernel_threads`, `threads_created` and `app_threads_after` in
  `async.flood.*` `where=background` against `lane`; the cost of a call against the calls the app
  has made in `async.pool_aging` (`last_over_first`) and `async.pool_batches`. First run: the last
  10 % of 3000 sequential calls took 17 times as long each as the first 10 % (a lane: flat), and
  `async.pool_retention` kept ~1 MiB per call, every call's result, until the app went.
- **The background pool stalls, then hangs, past its size.** `uniqueThreadKey` only sees threads
  that have started (`app.cpp:780-788`), so calls queued while the pool is full can get the same
  key. A worker that finishes starts the next queued task from its own thread
  (`threadPoolWorker` calling `tryStartWorker`, `app.cpp:1075-1120`), and `app::threads(key, ...)`
  interrupts and waits up to 5 s for the thread already holding that key (`app.cpp:594-620`);
  two workers doing that to each other each wait the full 5 s ("did not stop in 5s" in the log).
  Found by this suite: on a 22-thread machine, `async.flood.noop --size n=100` on the background
  pool logged eight such stalls and then hung for good, the driver thread idle (no CPU time)
  inside the runtime. A likely cause, not yet confirmed in a debugger: the
  `interruption_point()` at the top of `app::threads` throws in an interrupted worker after
  `tryStartWorker` has released `_threadPoolMutex` (`app.cpp:1092`), so the caller's lock guard
  unlocks it a second time and later lockers block forever. Default sizes stay under the pool
  size (`--bg-oversubscribe` runs past it); the watchdog turns a hang into a failed record.
- **Concurrent compute-pool fan-outs serialize.** `thread_pool::parallel_for` holds one job slot
  (`thread_pool.cpp:170`) and wakes every worker on each post (`thread_pool.cpp:192-195`):
  `async.compute_fanout_narrow` `extra.fanout_stretch` against `m`, and
  `async.compute_fanout_herd` `extra.fanout_ns`.
- **Per-call overhead of an async function.** About 30 allocations and 10 lock acquisitions before
  a kernel runs, even a no-op one (`async_host.cpp:386-430, 518-539`), and two state-tree reads of
  the step rate on the inline path (`async_host.cpp:434, 465`): `async.flood.noop`.
- **Polling hosts sleep a timer tick.** The async scheduler has no wakeup signal, so a host that
  sleeps when idle (pycvc: `sleep_for(1 ms)`) waits for the OS timer; on Windows that is ~15.6 ms
  unless something raised the timer resolution (the header's `sleep_for(1ms) takes` line).
  `driver=poll1ms` against `driver=spin` in `async.flood.*`, `async.futures` and
  `sched.sleepers` (`extra.lateness_ns`).
- **Per-form argument copies and string dispatch.** Each call form copies its argument ASTs into a
  fresh vector (`stackless_evaluator.cpp:912`, `evaluator.cpp:248`) and finds special forms by up
  to 19-21 sequential string compares (`stackless_evaluator.cpp:921-1172`,
  `evaluator.cpp:251-284`); environment lookups hash the name again at every scope
  (`types.cpp:413-420`): `eval.lookup_depth`; every step does an atomic increment of the step count
  (`evaluator.h:82-87`). This is the floor under every `eval.*` `ns_per_step` and
  `allocs_per_step`. Since fixed: special forms are found by a switch on the name
  (`special_forms.h`). The stackless evaluator reads a call's arguments in place and takes the
  vectors a frame needs from ones kept per thread; the recursive one copies them into a vector it
  keeps; neither allocates for them. A lookup hashes the name once and skips empty scopes, call
  scopes are recycled per thread, and a call looks its function up once, at its init. The
  evaluators count their own eval steps with a plain load and store (`count_eval_step()`);
  `increment_step()`, for any other thread, is still a locked add, of another counter.
- **Function bodies and try forms rebuilt per use.** A closure body of more than one form is
  rebuilt as a `(begin ...)` list on every call (`stackless_evaluator.cpp:1679-1691`):
  `eval.closure_body` `forms=3` against `forms=1`. A `try` is re-parsed on entry
  (`stackless_evaluator.cpp:1148-1157`): `eval.try_noraise`. Since fixed for bodies: a closure
  keeps its body's `(begin ...)`, made once and shared by the closures one lambda form makes
  (`closure_body.h`); a generator gets one of its own. A `try` is still parsed on entry, but its
  clauses are moved into its frame, not copied.
- **Exceptions in the recursive evaluator.** `return` and `raise` are C++ throws, and the first
  unwinding trail guard copies up to 64 traceback values (`evaluator.cpp:162-172, 426-433`):
  `eval.exceptions` and `eval.raise_depth` recursive against stackless.
- **Linear dicts.** A dict is a vector of pairs searched by string compare
  (`builtins.cpp:575-599`): `eval.dict_ops` `k` 4, 64, 512.
- **Guards cost clock reads.** With `max_time` armed a step reads the clock 6-8 times instead of 3
  (`scheduler.cpp:335, 346, 355, 399, 420`), and past half of `max_time` adds two thread-CPU reads
  per step (`process.cpp:46-53`): `limits.max_time` and `limits.max_time_ariadne`. With
  `max_memory` armed, an allocation guard per step and a heap measurement every
  `heap_check_steps`, one O(reachable) step re-run near the budget (`process.cpp:76-84`,
  `heap.cpp:265-281, 516-554`): `limits.max_memory_bigheap`. A deadline also adds a clock read per
  node in the deep-data walks (`data_job.cpp:272, 373, 572, 593`): `data.*` `deadline=1`.
- **An environment per process.** A default environment is 124 bindings and about 150 allocations
  to build (`builtins.cpp:851-938`); Ariadne builds one per action (`ariadne.cpp:1657-1658`):
  `flood.small` `env=own` against `env=extend`.
- **`range` builds a whole evaluator.** Each `(range ...)` makes a generator holding a
  `stackless_evaluator`, state and environment (`builtins.cpp:772-811`): `eval.range_create`.
- **No tail calls.** A self call in tail position still pushes a frame: `eval.tail_rec` and
  `sched.single.tail_rec` `heap_delta_bytes` against `n`.

## Adding a scenario

Add it to `make_scenarios()` in `state_exec_bench.cpp` (an `eval.*` program goes in
`eval_programs()`, which also makes its `sched.single.*` twin when `sched` is set): a name
(`group.what`), the executors, the default and smoke sizes (each key a list: a sweep),
`async_only` or `varies_for` for keys only some executors vary, `bg_calls_key` for a count of
concurrent background-pool calls, `spawns_threads` when the driver starts threads inside the
region, and a `run` that builds what the point needs, then calls `pc.reps` with a function that
does one rep: set up, `m.begin()`, the work, `m.end()`, then check the results with `o.fail(...)`
and fill `rep_out`. `run_flood`, `run_single`, `run_batch` and `park_bystanders` submit, drive,
reap and check programs for the scheduler scenarios. Keep a default-size rep under ~10 s and the
whole `--smoke` run under ~10 s; `--smoke` runs in CTest.
