#!/usr/bin/env python3
# Copyright 2026 The University of Texas at Austin
#
# This file is part of libcvc.
#
# libcvc is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License version 2.1 as published by the Free Software Foundation.
"""Run the state_exec benchmark suite and compare runs (docs/STATE_EXEC_BENCH.md).

  state_exec_bench.py run [--exe PATH] [--scenario GLOB]... [--reps 5] [--pin auto] --out base.jsonl
  state_exec_bench.py table base.jsonl
  state_exec_bench.py compare base.jsonl cand.jsonl [--metric wall_ns] [--threshold 3]

`run` starts the executable once per scenario (with --split, once per scenario, executor and sweep
point), so the thread count and the handle count a record carries belong to that scenario alone and
a leak in one can't bleed into the next. Peak memory is per process where the OS can't reset it
(Windows, macOS): a record whose peak may belong to an earlier point of its process carries
peak_inherited, and the table marks it with '*'; use --split for exact peaks. Records are appended
to --out as JSON lines, one per rep, then summarized as a table. A process that crashed, hung past
--process-timeout or failed without writing a failure record gets a synthesized ok:false record.

`compare` matches records on (scenario, executor, params), takes the median of a metric on each
side and the noise of each side (half the min-max range, and 3 x the scaled MAD, relative to the
median), and flags a change whose size exceeds both the noise and --threshold percent. A point
whose checks failed on either side, or that only the base has, is reported as such and counts as a
regression for --fail-on-worse.

Standard library only.
"""

import argparse
import fnmatch
import itertools
import json
import os
import shutil
import statistics
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))

# Metrics where a larger value is better; every other metric is a cost.
HIGHER_IS_BETTER = {"programs_per_s", "extra.messages_per_s", "extra.mb_per_s",
                    "extra.pool_efficiency", "extra.speedup", "extra.fairness",
                    "extra.steps_per_wall_s"}


def find_exe(explicit):
    if explicit:
        return explicit
    names = ["state_exec_bench.exe", "state_exec_bench"]
    for build in ["build-win", "build", "build-release", "out/build"]:
        for sub in ["bin", "src/cvc/tests", "bin/Release", "src/cvc/tests/Release"]:
            for n in names:
                p = os.path.join(REPO, build, sub, n)
                if os.path.isfile(p):
                    return p
    found = shutil.which("state_exec_bench")
    if found:
        return found
    sys.exit("state_exec_bench.py: can't find the state_exec_bench executable; pass --exe")


def child_env(dll_dirs, exe):
    env = dict(os.environ)
    dirs = list(dll_dirs or [])
    dirs.insert(0, os.path.dirname(os.path.abspath(exe)))
    env["PATH"] = os.pathsep.join(dirs + [env.get("PATH", "")])
    return env


def list_scenarios(exe, env):
    out = subprocess.run([exe, "--list-json"], capture_output=True, text=True, env=env, check=True)
    return json.loads(out.stdout)


def load(path):
    recs = []
    with open(path, encoding="utf-8") as f:
        for n, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                recs.append(json.loads(line))
            except json.JSONDecodeError as e:
                print(f"{path}:{n}: skipped a bad line ({e})", file=sys.stderr)
    return recs


def key_of(r):
    params = r.get("params", {})
    return (r.get("scenario", "?"), r.get("executor", ""),
            " ".join(f"{k}={params[k]}" for k in sorted(params)))


def metric(r, name):
    cur = r
    for part in name.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return None
        cur = cur[part]
    return cur if isinstance(cur, (int, float)) and not isinstance(cur, bool) else None


def group(recs):
    groups = {}
    for r in recs:
        if r.get("rep", 0) < 0 and r.get("ok", True):
            continue
        groups.setdefault(key_of(r), []).append(r)
    return groups


def noise_of(values):
    """Relative noise of a sample: the larger of half its range and 3 x its scaled MAD."""
    if len(values) < 2:
        return 0.0
    med = statistics.median(values)
    if med == 0:
        return 0.0
    half_range = (max(values) - min(values)) / 2.0
    mad = statistics.median(abs(v - med) for v in values) * 1.4826
    return max(half_range, 3.0 * mad) / abs(med)


def fmt_ms(ns):
    return "-" if ns is None else f"{ns / 1e6:.3f}"


def fmt(v, digits=1):
    return "-" if v is None else f"{v:.{digits}f}"


def med(rs, name):
    vals = [v for v in (metric(r, name) for r in rs) if v is not None]
    return statistics.median(vals) if vals else None


def table(recs, out=sys.stdout):
    groups = group(recs)
    head = (f"{'scenario':<28} {'exec':<9} {'params':<40} {'n':>2} {'wall ms':>10} {'+-%':>5} "
            f"{'ns/step':>8} {'ns/iter':>9} {'prog/s':>10} {'p99 ms':>9} {'peakWS MB':>9} "
            f"{'threads':>11} {'ok':>3}")
    print(head, file=out)
    print("-" * len(head), file=out)
    for key in sorted(groups):
        rs = groups[key]
        walls = [r["wall_ns"] for r in rs if "wall_ns" in r]
        p99 = [r["latency_ns"]["p99"] for r in rs if "latency_ns" in r and "p99" in r["latency_ns"]]
        peak = max((r.get("peak_working_set_bytes", 0) for r in rs), default=0)
        inherited = any(r.get("peak_inherited") for r in rs)
        tb = min((r.get("thread_count_before", -1) for r in rs), default=-1)
        ta = max((r.get("thread_count_after", -1) for r in rs), default=-1)
        tt = rs[-1].get("thread_count_teardown")
        threads = f"{tb}>{ta}" + (f">{tt}" if tt is not None else "")
        ok = all(r.get("ok", False) for r in rs)
        print(f"{key[0]:<28} {key[1]:<9} {key[2][:40]:<40} {len(rs):>2} "
              f"{fmt_ms(statistics.median(walls) if walls else None):>10} "
              f"{fmt(100 * noise_of(walls)):>5} {fmt(med(rs, 'ns_per_step')):>8} "
              f"{fmt(med(rs, 'ns_per_iter')):>9} {fmt(med(rs, 'programs_per_s'), 0):>10} "
              f"{fmt_ms(statistics.median(p99) if p99 else None):>9} "
              f"{peak / 2**20:>8.1f}{'*' if inherited else ' '} "
              f"{threads:>11} {'ok' if ok else 'NO':>3}", file=out)
        if not ok:
            errs = {r.get("error", "?") for r in rs if not r.get("ok", False)}
            for e in sorted(errs):
                print(f"    error: {e}", file=out)


def compare(base, cand, metrics, threshold):
    gb, gc = group(base), group(cand)
    flagged = worse = 0
    failed_keys = set()
    for name in metrics:
        better_up = name in HIGHER_IS_BETTER
        print(f"\nmetric {name} (median; {'higher' if better_up else 'lower'} is better; "
              f"flag when |delta| > max(noise, {threshold}%))")
        head = (f"{'scenario':<28} {'exec':<9} {'params':<36} {'base':>12} {'cand':>12} "
                f"{'delta%':>8} {'noise%':>7}  verdict")
        print(head)
        print("-" * len(head))
        for key in sorted(set(gb) | set(gc)):
            if key not in gb or key not in gc:
                side = "base" if key in gb else "cand"
                print(f"{key[0]:<28} {key[1]:<9} {key[2][:36]:<36} only in {side}")
                if side == "base":
                    failed_keys.add(key)  # the candidate lost a point: crashed, hung or skipped
                continue
            okb = all(r.get("ok", False) for r in gb[key])
            okc = all(r.get("ok", False) for r in gc[key])
            vb = [v for v in (metric(r, name) for r in gb[key]) if v is not None]
            vc = [v for v in (metric(r, name) for r in gc[key]) if v is not None]
            if not (okb and okc):
                # Checked before the values: a hung or crashed rep's record has no metrics.
                where = "base" if not okb else "cand"
                if not okc:
                    failed_keys.add(key)
                print(f"{key[0]:<28} {key[1]:<9} {key[2][:36]:<36} {'':>12} {'':>12} "
                      f"{'':>8} {'':>7}  CHECK FAILED in {where}")
                continue
            if not vb or not vc:
                continue
            mb, mc = statistics.median(vb), statistics.median(vc)
            noise = 100.0 * max(noise_of(vb), noise_of(vc))
            if mb == 0:
                # No relative change to speak of.
                print(f"{key[0]:<28} {key[1]:<9} {key[2][:36]:<36} {mb:>12.6g} {mc:>12.6g} "
                      f"{'n/a':>8} {noise:>7.1f}  n/a (base 0)")
                continue
            # Relative to |base|, so a negative base (a derived difference such as
            # extra.ns_per_call) doesn't flip the sign of the change.
            delta = 100.0 * (mc - mb) / abs(mb)
            verdict = "~"
            if abs(delta) > max(noise, threshold):
                improved = (delta > 0) == better_up
                verdict = "better" if improved else "WORSE"
                flagged += 1
                worse += 0 if improved else 1
            print(f"{key[0]:<28} {key[1]:<9} {key[2][:36]:<36} {mb:>12.6g} {mc:>12.6g} "
                  f"{delta:>+8.1f} {noise:>7.1f}  {verdict}")
    print(f"\n{flagged} change(s) beyond the noise, {worse} of them worse; "
          f"{len(failed_keys)} point(s) failed or missing in the candidate")
    return worse + len(failed_keys)


def expand_points(sc, sizes_over, executors, smoke):
    """The (executor, sizes) points --split runs one process each for."""
    execs = sc["executors"] or [""]
    for ex in execs:
        if executors and ex and ex not in executors:
            continue
        sizes = dict(sc["smoke" if smoke else "sizes"])
        for k, v in sizes_over.items():
            if k in sizes:
                sizes[k] = v
        # Keys this executor doesn't vary (the async scheduler's driving, say): the executable
        # runs them at their first value and leaves them out of the params.
        fixed = {k for k in sc.get("async_only", []) if ex != "async"}
        fixed |= {k for k, execs in sc.get("varies_for", {}).items() if ex not in execs}
        for k in fixed:
            sizes.pop(k, None)
        keys = list(sizes)
        for combo in itertools.product(*(sizes[k] for k in keys)):
            yield ex, dict(zip(keys, (str(c) for c in combo)))


def count_lines(path):
    if not os.path.exists(path):
        return 0
    with open(path, encoding="utf-8") as f:
        return sum(1 for _ in f)


def synthesize_failure(out, before, ident, smoke, why):
    """Append an ok:false record for a process that ended without writing one."""
    new = []
    if os.path.exists(out):
        with open(out, encoding="utf-8") as f:
            for n, line in enumerate(f):
                if n >= before and line.strip():
                    try:
                        new.append(json.loads(line))
                    except json.JSONDecodeError:
                        pass
    if any(not r.get("ok", True) for r in new):
        return  # the executable recorded the failure itself (a check, or its watchdog)
    rec = dict(ident)
    rec["params"] = {k: (int(v) if isinstance(v, str) and v.lstrip("-").isdigit() else v)
                     for k, v in ident.get("params", {}).items()}
    rec.update({"rep": -1, "ok": False, "error": why, "smoke": smoke, "synthesized": True})
    with open(out, "a", encoding="utf-8") as f:
        f.write(json.dumps(rec) + "\n")


def cmd_run(args):
    exe = find_exe(args.exe)
    env = child_env(args.dll_dir, exe)
    scenarios = list_scenarios(exe, env)
    patterns = args.scenario or ["*"]
    chosen = [s for s in scenarios if any(fnmatch.fnmatchcase(s["name"], p) for p in patterns)]
    if not chosen:
        sys.exit("state_exec_bench.py: no scenario matches")
    out = args.out or time.strftime("state_exec_bench-%Y%m%d-%H%M%S.jsonl")
    sizes_over = {}
    for kv in args.size or []:
        k, _, v = kv.partition("=")
        sizes_over[k] = [x for x in v.split(",") if x]
    common = ["--json", out, "--pin", args.pin, "--timeout", str(args.timeout)]
    if args.reps is not None:
        common += ["--reps", str(args.reps)]
    if args.warmup is not None:
        common += ["--warmup", str(args.warmup)]
    if args.priority:
        common += ["--priority", args.priority]
    if args.smoke:
        common += ["--smoke"]
    if args.bg_oversubscribe:
        common += ["--bg-oversubscribe"]
    jobs = []
    for sc in chosen:
        if args.split:
            for ex, pt in expand_points(sc, sizes_over, args.executor, args.smoke):
                argv = ["--scenario", sc["name"]] + (["--executor", ex] if ex else [])
                for k, v in pt.items():
                    argv += ["--size", f"{k}={v}"]
                ident = {"scenario": sc["name"], "params": pt}
                if ex:
                    ident["executor"] = ex
                jobs.append((f"{sc['name']} {ex} " + " ".join(f"{k}={v}" for k, v in pt.items()),
                             argv, ident))
        else:
            argv = ["--scenario", sc["name"]]
            for ex in args.executor or []:
                argv += ["--executor", ex]
            for k, v in sizes_over.items():
                argv += ["--size", f"{k}={','.join(v)}"]
            jobs.append((sc["name"], argv, {"scenario": sc["name"], "params": {}}))
    failures = []
    t0 = time.time()
    for label, argv, ident in jobs:
        start = time.time()
        before = count_lines(out)
        try:
            proc = subprocess.run([exe] + argv + common, env=env, capture_output=not args.verbose,
                                  text=True, timeout=args.process_timeout)
        except subprocess.TimeoutExpired:
            # The executable's own watchdog ends a hung rep; this is the backstop for the rest.
            print(f"[run] {label:<60} killed after {args.process_timeout:.0f} s", flush=True)
            failures.append(label)
            synthesize_failure(out, before, ident, args.smoke,
                               f"killed by the runner after {args.process_timeout:.0f} s")
            continue
        took = time.time() - start
        status = "ok" if proc.returncode == 0 else f"FAILED (exit {proc.returncode})"
        print(f"[run] {label:<60} {took:7.1f} s  {status}", flush=True)
        if proc.returncode != 0:
            failures.append(label)
            synthesize_failure(out, before, ident, args.smoke,
                               f"the process exited with {proc.returncode} without a failure "
                               f"record (crashed?)")
            if not args.verbose and proc.stdout:
                bad = [ln for ln in proc.stdout.splitlines() if "FAIL" in ln]
                for ln in (bad or proc.stdout.splitlines()[-5:]):
                    print("      " + ln.strip())
            if not args.verbose and proc.stderr:
                print("      " + proc.stderr.strip()[-500:])
    print(f"[run] {len(jobs)} process(es) in {time.time() - t0:.1f} s, records in {out}\n")
    if os.path.exists(out):
        table(load(out))
    if failures:
        print(f"\n{len(failures)} failed: " + ", ".join(failures))
        return 1
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="run scenarios, a fresh process each")
    r.add_argument("--exe", help="the state_exec_bench executable (default: found in build dirs)")
    r.add_argument("--dll-dir", action="append",
                   help="a directory to put on PATH for the run (Windows DLLs); repeatable")
    r.add_argument("--scenario", action="append", help="name or glob; repeatable (default: all)")
    r.add_argument("--executor", action="append", help="recursive|stackless|sync|async; repeatable")
    r.add_argument("--size", action="append", help="KEY=V[,V...] size override; repeatable")
    r.add_argument("--reps", type=int, help="measured reps per point (executable default 5)")
    r.add_argument("--warmup", type=int, help="warmup reps per point (executable default 1)")
    r.add_argument("--pin", default="auto", help="auto|none|CPU (default auto)")
    r.add_argument("--priority", choices=["high", "normal"])
    r.add_argument("--timeout", type=float, default=120.0, help="seconds per rep before failing")
    r.add_argument("--process-timeout", type=float, default=3600.0,
                   help="seconds before a whole process is killed (default 3600)")
    r.add_argument("--smoke", action="store_true", help="tiny sizes, 1 rep")
    r.add_argument("--split", action="store_true",
                   help="one process per executor and sweep point, not per scenario")
    r.add_argument("--bg-oversubscribe", action="store_true",
                   help="run background-pool points with more calls than the pool has workers "
                        "(they stall and can hang: docs/STATE_EXEC_BENCH.md)")
    r.add_argument("--out", help="JSONL file to append to (default: a timestamped name)")
    r.add_argument("--verbose", action="store_true", help="show the executable's own output")
    t = sub.add_parser("table", help="summarize a JSONL file")
    t.add_argument("file")
    c = sub.add_parser("compare", help="compare two JSONL files")
    c.add_argument("base")
    c.add_argument("cand")
    c.add_argument("--metric", action="append",
                   help="record field to compare, dotted for nested (default: wall_ns and "
                        "ns_per_step); repeatable")
    c.add_argument("--threshold", type=float, default=3.0,
                   help="minimum |delta| percent to flag, whatever the noise (default 3)")
    c.add_argument("--fail-on-worse", action="store_true",
                   help="exit 1 when any change beyond the noise is a regression, or a point "
                        "failed its checks or is missing in the candidate")
    args = ap.parse_args()
    if args.cmd == "run":
        return cmd_run(args)
    if args.cmd == "table":
        table(load(args.file))
        return 0
    metrics = args.metric or ["wall_ns", "ns_per_step"]
    worse = compare(load(args.base), load(args.cand), metrics, args.threshold)
    return 1 if (args.fail_on_worse and worse) else 0


if __name__ == "__main__":
    sys.exit(main())
