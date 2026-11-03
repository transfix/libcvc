#!/usr/bin/env python3
# Copyright 2026 The University of Texas at Austin
#
# This file is part of libcvc.
#
# libcvc is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License version 2.1 as published by the Free Software Foundation.
"""mt.pycvc_threads: T Python threads, each running programs through a pycvc.Exec of its own, all
over one app (docs/STATE_EXEC_BENCH.md; docs/STATE_EXEC_THREADED.md 6.3, Stage 0).

  mt_pycvc_threads.py [--t 1,2,4,8] [--prog fib,exceptions] [--runs 5] [--reps 5] [--warmup 1]
                      [--smoke] [--pycvc-dir DIR] [--dll-dir DIR]... [--json FILE]

Each thread makes its Exec, waits at a barrier, then calls Exec.run on its program `runs` times
(fib 18 or 2000 caught raises, mt.independent's programs; --smoke: fib 10, 50 raises). Exec.run
releases the GIL around its slices, so the threads' programs can overlap. A rep times one thread
first, then t threads: extra.speedup is t x the one thread's time over the t threads' wall time,
from the barrier to the last join. Records are state_exec_bench's JSON lines (scenario
mt.pycvc_threads, executor pycvc), appended to --json, so state_exec_bench.py's table and compare
read them. The scenario is recorded, not gated.

pycvc comes from --pycvc-dir (the build's bindings/pycvc), else from sys.path; on Windows each
--dll-dir, and each directory in CVC_DLL_DIRS (os.pathsep-separated, as pycvc's tests read it), is
added with os.add_dll_directory. Standard library only, besides pycvc.
"""

import argparse
import json
import os
import statistics
import sys
import threading
import time

SCENARIO = "mt.pycvc_threads"


def fib(n):
    a, b = 0, 1
    for _ in range(n):
        a, b = b, a + b
    return a


def program(prog, smoke):
    """(source, expected result, iterations per run) of a program, as mt.independent runs it."""
    if prog == "fib":
        n = 10 if smoke else 18
        src = ("(begin (defun fib (n) (if (< n 2) n (+ (fib (- n 1)) (fib (- n 2))))) "
               f"(fib {n}))")
        return src, str(fib(n)), 2 * fib(n + 1) - 1
    if prog == "exceptions":
        n = 50 if smoke else 2000
        body = '(set c (+ c (try (raise (ValueError "boom")) (catch (ValueError e) 1))))'
        src = f"(begin (set c 0) (set i 0) (while (< i {n}) (begin {body} (set i (+ i 1)))) c)"
        return src, str(n), n
    raise ValueError(f"prog={prog}: not fib or exceptions")


def run_threads(pycvc, app, count, src, expect, runs):
    """`count` threads, each with an Exec of its own on `app`, released together; each runs `src`
    `runs` times. Returns (wall ns from the barrier to the last join, per-thread ns, errors)."""
    barrier = threading.Barrier(count + 1)
    spans = [0] * count
    errors = []
    lock = threading.Lock()

    def body(j):
        try:
            ex = pycvc.Exec(app)
            barrier.wait()
            t0 = time.perf_counter_ns()
            for _ in range(runs):
                got = ex.run(src)
                if got != expect:
                    raise RuntimeError(f"returned {got}, expected {expect}")
            spans[j] = time.perf_counter_ns() - t0
        except BaseException as e:  # noqa: BLE001 - reported in the record
            with lock:
                errors.append(f"thread {j}: {type(e).__name__}: {e}")
            if not barrier.broken:
                barrier.abort()

    threads = [threading.Thread(target=body, args=(j,), name=f"mt-pycvc-{j}")
               for j in range(count)]
    for t in threads:
        t.start()
    try:
        barrier.wait()
    except threading.BrokenBarrierError:
        pass
    t0 = time.perf_counter_ns()
    for t in threads:
        t.join()
    return time.perf_counter_ns() - t0, spans, errors


def record(t, prog, runs, rep, iters, wall, one, spans, errors, smoke):
    """A JSON record of one rep, in state_exec_bench's form."""
    r = {"scenario": SCENARIO, "executor": "pycvc",
         "params": {"t": t, "prog": prog, "runs": runs}, "rep": rep, "ok": not errors}
    if errors:
        r["error"] = "; ".join(errors)
    total = t * runs * iters
    r["wall_ns"] = wall
    r["iterations"] = total
    if wall > 0:
        r["ns_per_iter"] = wall / total
        r["programs_per_s"] = t * runs * 1e9 / wall
    r["programs"] = t * runs
    extra = {"single_thread_ns_per_iter": one / (runs * iters)}
    if wall > 0:
        extra["speedup"] = t * one / wall
    done = [s for s in spans if s > 0]
    if done:
        extra["thread_ns_per_iter"] = statistics.median(done) / (runs * iters)
    r["extra"] = extra
    r.update({"pin_cpu": -1, "logical_cpus": os.cpu_count() or 0, "smoke": smoke,
              "python": sys.version.split()[0]})
    return r


def ints(text):
    return [int(x) for x in text.split(",") if x]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--t", default=None, help="thread counts (default 1,2,4,8; smoke 1,2)")
    ap.add_argument("--prog", default="fib,exceptions", help="fib, exceptions or both")
    ap.add_argument("--runs", type=int, default=None, help="runs a thread (default 5; smoke 1)")
    ap.add_argument("--reps", type=int, default=None, help="measured reps (default 5; smoke 1)")
    ap.add_argument("--warmup", type=int, default=None, help="unrecorded reps (default 1; smoke 0)")
    ap.add_argument("--smoke", action="store_true", help="tiny sizes, one rep")
    ap.add_argument("--pycvc-dir", help="the directory pycvc imports from")
    ap.add_argument("--dll-dir", action="append", default=[],
                    help="a directory of DLLs pycvc needs (Windows); repeatable")
    ap.add_argument("--json", help="append one JSON record per rep")
    args = ap.parse_args(argv)
    smoke = args.smoke
    counts = ints(args.t) if args.t else ([1, 2] if smoke else [1, 2, 4, 8])
    progs = [p for p in args.prog.split(",") if p]
    runs = args.runs if args.runs is not None else (1 if smoke else 5)
    reps = args.reps if args.reps is not None else (1 if smoke else 5)
    warmup = args.warmup if args.warmup is not None else (0 if smoke else 1)
    for p in progs:
        program(p, smoke)  # a usage error before anything runs

    if args.pycvc_dir:
        sys.path.insert(0, os.path.abspath(args.pycvc_dir))
    dll_dirs = args.dll_dir + os.environ.get("CVC_DLL_DIRS", "").split(os.pathsep)
    for d in dll_dirs:
        if d and os.path.isdir(d) and hasattr(os, "add_dll_directory"):
            os.add_dll_directory(os.path.abspath(d))
    import pycvc  # noqa: E402 - located by the options above

    print(f"[{SCENARIO}] python {sys.version.split()[0]} | logical CPUs {os.cpu_count()} | "
          f"reps {reps}, warmup {warmup}{' | smoke' if smoke else ''}", flush=True)
    out = open(args.json, "a", encoding="utf-8") if args.json else None
    failed = False
    try:
        for prog in progs:
            src, expect, iters = program(prog, smoke)
            for t in counts:
                app = pycvc.make_app()
                for n in range(warmup + reps):
                    one, _, one_err = run_threads(pycvc, app, 1, src, expect, runs)
                    wall, spans, errors = run_threads(pycvc, app, t, src, expect, runs)
                    errors = [f"one thread: {e}" for e in one_err] + errors
                    if n < warmup and not errors:
                        continue
                    rep = n - warmup if n >= warmup else -1
                    r = record(t, prog, runs, rep, iters, wall, one, spans, errors, smoke)
                    failed = failed or bool(errors)
                    speedup = r["extra"].get("speedup", 0.0)
                    print(f"  {SCENARIO:<28} {'pycvc':<9} "
                          f"{f't={t} prog={prog} runs={runs}':<34} rep {rep}  "
                          f"{wall / 1e6:10.3f} ms  speedup {speedup:5.2f}  "
                          f"{'ok' if not errors else 'FAIL: ' + r['error']}", flush=True)
                    if out:
                        out.write(json.dumps(r) + "\n")
                        out.flush()
                    if errors and n < warmup:
                        break
                del app
    finally:
        if out:
            out.close()
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
