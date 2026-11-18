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
                      [--pair KEY=A:B] [--max-noise P] [--fail-on-worse | --fail-on-change]
                      [--same-pin] [--geomean-better PCT]
  state_exec_bench.py gate run.jsonl --scenario GLOB --metric M (--max X | --min X) [--stat median]

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
regression for --fail-on-worse. A param at its implicit default (IMPLICIT_DEFAULTS) is left out of
the match, so records from binaries that predate the param still match. --pair KEY=A:B compares
the points whose KEY is A in the base with those whose KEY is B in the candidate (one run against
itself, say); a scenario that has records on one side only fails it. With --max-noise P, a point
noisier than P percent is INCONCLUSIVE rather than judged. A matched point whose two sides ran their
driver on different CPUs (pin_cpu) is warned about, and fails with --same-pin. --geomean-better PCT
takes, per metric, the geometric mean over the matched points (with --pair, the matched pairs) of
the candidate's median over the base's (the base's over the candidate's for a metric where higher
is better), and asks it to be at most 1 - PCT/100: a whole family faster by PCT percent, as one
number. Exit status: 1 when a --pair leaves a scenario unpaired, a point fails under
--fail-on-worse or --fail-on-change, the CPUs differ under --same-pin, or --geomean-better misses
(or has a point that failed on either side or is missing in the candidate, or no pair at all);
else 2 when a point is INCONCLUSIVE (repeat the run on a quieter machine; never a pass); else 0.

`gate` checks one statistic of a metric against an absolute bound on every matching point of one
run (a gate the design states in nanoseconds, steps or allocations rather than as a change), with
the same exit statuses. A scenario in GATE_GUARDS has its gate judge only points that could fail
it: mt.ariadne_docs' points with documents outside `any` groups of their own fail as vacuous,
points that can wake no helper are n/a (a gate with nothing but n/a fails), and the run's heavy
points must show a worker pumping (the control).

Standard library only.
"""

import argparse
import fnmatch
import itertools
import json
import math
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
                    "extra.steps_per_wall_s", "extra.of_ceiling"}


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


# Sweep keys some scenarios gained after records existed, at the value every point ran at before
# them. A param at its implicit default is left out of a record's match key, so a record from a
# binary without the key and one from a binary with it at its default are the same point. Stage 0
# of threaded execution added chroot (native.state_get, native.state_set, sched.watch_poll; it was
# already lock.uncontended's) and writes (sched.watch_poll); Stage 1 added base (sched.single.*'s
# async points: driver=group's base env, own by default). sched.single.*'s driver key was already
# there (spin, the bare scheduler's loop), so its new value, group, needs no default. Stage S added
# slice_steps (sched.single.*'s and limits.*'s async points): 1 is the loop every point ran before.
IMPLICIT_DEFAULTS = {"policy": "rr", "locks": "none", "chroot": "0", "writes": "none",
                     "base": "own", "slice_steps": "1"}


def param(r, name):
    """A record's value of param `name` as text: its implicit default when the record has none."""
    v = r.get("params", {}).get(name)
    return str(v) if v is not None else IMPLICIT_DEFAULTS.get(name)


def key_of(r, drop=()):
    params = {k: str(v) for k, v in r.get("params", {}).items()
              if k not in drop and IMPLICIT_DEFAULTS.get(k) != str(v)}
    return (r.get("scenario", "?"), r.get("executor", ""),
            " ".join(f"{k}={params[k]}" for k in sorted(params)))


def metric(r, name):
    cur = r
    for part in name.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return None
        cur = cur[part]
    return cur if isinstance(cur, (int, float)) and not isinstance(cur, bool) else None


def group(recs, drop=()):
    groups = {}
    for r in recs:
        if r.get("rep", 0) < 0 and r.get("ok", True):
            continue
        groups.setdefault(key_of(r, drop), []).append(r)
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


def pins(recs):
    """The driver CPUs (pin_cpu) a point's records ran on; empty when they don't say."""
    return {r["pin_cpu"] for r in recs if "pin_cpu" in r}


def compare(base, cand, metrics, threshold, pair=None, max_noise=None, geomean_better=None):
    """Print the comparison; return the counts of worse, changed (any verdict but ~), inconclusive
    and failed (failed its checks or missing in the candidate) points; with `pair`, the scenarios
    only one side has records of (unpaired); and the matched points whose sides ran their driver on
    different CPUs (pins). With `geomean_better` (a percent), also "geomean": per metric the
    geometric mean of the candidate's median over the base's across the matched points, their
    count and how many were INCONCLUSIVE, and the verdict ("pass", "fail" or "inconclusive")."""
    drop = ()
    unpaired = 0
    if pair:
        # The base side keeps the points at A, the candidate side those at B, matched without the
        # key; a point at neither value is left out. A scenario with records on one side only is a
        # pair that matched nothing there (a value misspelled, or not run): it fails, rather than
        # passing with every point "only in" one side.
        key, a, b = pair
        base = [r for r in base if param(r, key) == a]
        cand = [r for r in cand if param(r, key) == b]
        drop = (key,)
        sb = {r.get("scenario", "?") for r in base}
        sc = {r.get("scenario", "?") for r in cand}
        for s in sorted(sb ^ sc):
            side, value = ("base", a) if s not in sb else ("candidate", b)
            print(f"pair {key}={a}:{b}: {s} has no {side} record at {key}={value}")
        unpaired = len(sb ^ sc)
        if not sb and not sc:
            print(f"pair {key}={a}:{b}: no record has {key}={a} or {key}={b}")
            unpaired = 1
    gb, gc = group(base, drop), group(cand, drop)
    # The driver's CPU is part of what a timing measures: an A/B whose two binaries pinned it to
    # different CPUs (another core, another scheduling class) compares the cores as much as the
    # code. Warned about here; --same-pin fails it.
    pin_keys = [k for k in sorted(set(gb) & set(gc))
                if pins(gb[k]) and pins(gc[k]) and pins(gb[k]) != pins(gc[k])]
    for k in pin_keys:
        print(f"warning: {k[0]} {k[1]} {k[2]}: the driver ran on CPU "
              f"{','.join(map(str, sorted(pins(gb[k]))))} in base and "
              f"{','.join(map(str, sorted(pins(gc[k]))))} in cand (pin_cpu)")
    flagged = worse = inconclusive = 0
    failed_keys = set()
    base_failed = set()  # failed its checks in the base: no ratio to take (--geomean-better)
    geo = {}  # metric -> the matched points' ratios, and how many were noisy or had none
    for name in metrics:
        better_up = name in HIGHER_IS_BETTER
        g = geo.setdefault(name, {"ratios": [], "inconclusive": 0, "no_ratio": 0})
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
                if not okb:
                    base_failed.add(key)
                print(f"{key[0]:<28} {key[1]:<9} {key[2][:36]:<36} {'':>12} {'':>12} "
                      f"{'':>8} {'':>7}  CHECK FAILED in {where}")
                continue
            if not vb or not vc:
                continue
            mb, mc = statistics.median(vb), statistics.median(vc)
            noise = 100.0 * max(noise_of(vb), noise_of(vc))
            if mb > 0 and mc > 0:
                # The point's ratio, as a cost: lower is better either way.
                g["ratios"].append(mb / mc if better_up else mc / mb)
                if max_noise is not None and noise > max_noise:
                    g["inconclusive"] += 1
            else:
                g["no_ratio"] += 1  # a median at or below 0 has no ratio
            if mb == 0:
                # No relative change to speak of; a value that left zero still changed.
                verdict = "n/a (base 0)"
                if mc != 0:
                    verdict = "CHANGED (base 0)"
                    flagged += 1
                print(f"{key[0]:<28} {key[1]:<9} {key[2][:36]:<36} {mb:>12.6g} {mc:>12.6g} "
                      f"{'n/a':>8} {noise:>7.1f}  {verdict}")
                continue
            # Relative to |base|, so a negative base (a derived difference such as
            # extra.ns_per_call) doesn't flip the sign of the change.
            delta = 100.0 * (mc - mb) / abs(mb)
            verdict = "~"
            if max_noise is not None and noise > max_noise:
                # Too noisy to rule a change in or out at the size asked for.
                verdict = "INCONCLUSIVE"
                inconclusive += 1
            elif abs(delta) > max(noise, threshold):
                improved = (delta > 0) == better_up
                verdict = "better" if improved else "WORSE"
                flagged += 1
                worse += 0 if improved else 1
            print(f"{key[0]:<28} {key[1]:<9} {key[2][:36]:<36} {mb:>12.6g} {mc:>12.6g} "
                  f"{delta:>+8.1f} {noise:>7.1f}  {verdict}")
        if geomean_better is not None:
            print_geomean(name, g, geomean_better, pair, max_noise)
    print(f"\n{flagged} change(s) beyond the noise, {worse} of them worse; "
          f"{len(failed_keys)} point(s) failed or missing in the candidate"
          + (f"; {inconclusive} inconclusive (noise above {max_noise:g}%)"
             if max_noise is not None else "")
          + (f"; {unpaired} scenario(s) unpaired" if unpaired else "")
          + (f"; {len(pin_keys)} point(s) with the driver on another CPU" if pin_keys else ""))
    res = {"worse": worse, "changed": flagged, "inconclusive": inconclusive,
           "failed": len(failed_keys), "unpaired": unpaired, "pins": len(pin_keys)}
    if geomean_better is not None:
        res["geomean"] = geomean_verdict(geo, geomean_better, res, len(base_failed))
    return res


def geomean(ratios):
    return math.exp(sum(math.log(r) for r in ratios) / len(ratios)) if ratios else None


def print_geomean(name, g, pct, pair, max_noise):
    """--geomean-better's line for one metric: the geomean, over how many pairs, and the target."""
    target = 1.0 - pct / 100.0
    if pair:
        key, a, b = pair
        what = f"{key}={b} over {key}={a}"
    else:
        what = "cand over base"
    if name in HIGHER_IS_BETTER:
        what = f"the inverse of {what} (higher is better)"
    gm = geomean(g["ratios"])
    if gm is None:
        print(f"\ngeomean of {name}, {what}: no pair to take it over; --geomean-better {pct:g} "
              f"wants at most {target:.3f}")
        return
    print(f"\ngeomean of {name}, {what}, over {len(g['ratios'])} pair(s): {gm:.3f} "
          f"({100.0 * (gm - 1.0):+.1f}%); --geomean-better {pct:g} wants at most {target:.3f}"
          + (f"; {g['inconclusive']} pair(s) noisier than {max_noise:g}%"
             if g["inconclusive"] else "")
          + (f"; {g['no_ratio']} point(s) with a median at or below 0 (no ratio)"
             if g["no_ratio"] else ""))


def geomean_verdict(geo, pct, res, base_failed):
    """--geomean-better's verdict over every metric compared: "fail" when a point failed its checks
    on either side or is missing in the candidate, a --pair left a scenario unpaired, or a metric
    has no pair or a point with no ratio; otherwise "inconclusive" when a pair is noisier than
    --max-noise (a geomean over a pair too noisy to judge is not judged either way, as compare
    judges no noisy point); otherwise "pass" when every metric's geomean is at most 1 - pct/100,
    and "fail" when one is not."""
    target = 1.0 - pct / 100.0
    metrics = {name: {"geomean": geomean(g["ratios"]), "pairs": len(g["ratios"]),
                      "inconclusive": g["inconclusive"]} for name, g in geo.items()}
    broken = bool(res["failed"] or base_failed or res["unpaired"]
                  or any(not g["ratios"] or g["no_ratio"] for g in geo.values()))
    if broken:
        verdict = "fail"
    elif any(g["inconclusive"] for g in geo.values()):
        verdict = "inconclusive"
    elif all(m["geomean"] <= target for m in metrics.values()):
        verdict = "pass"
    else:
        verdict = "fail"
    print(f"--geomean-better {pct:g}: {verdict.upper()}"
          + (" (a point failed its checks or is missing in the candidate, a scenario is "
             "unpaired, or a metric has no usable pair)" if broken else ""))
    return {"verdict": verdict, "metrics": metrics}


def parse_pair(text):
    """KEY=A:B -> (KEY, A, B)."""
    key, sep, vals = text.partition("=")
    a, sep2, b = vals.partition(":")
    if not (key and sep and sep2 and a and b) or a == b:
        raise ValueError(f"--pair wants KEY=A:B with A != B, not {text!r}")
    return key, a, b


# Scenarios whose gates judge something only at points that could fail them, and the metrics that
# holds for. mt.ariadne_docs' small-frames gate (S1.md section 6: small frames wake no helper):
# - vacuous: each row's metric must be at least the row's value of the param, else the point
#   fails. Every document must be in a group of its own (extra.isolated) and an `any` one
#   (extra.any_groups): the caller pumps group 0 and host-affinity groups, and they wake nobody;
# - not_applicable: a point whose rows all have this metric at 0 can wake no helper (t=0, no
#   worker; docs=1, the caller takes the lone group itself), so it prints n/a, neither PASS nor
#   FAIL. A gate where every point is n/a fails: it judged nothing;
# - control: the same run's points matching `where` that can wake a helper must have the metric's
#   median at least `min` (heavy frames pumped on a worker), else the gate fails: a pool whose
#   adaptive wake never fires passes small frames too.
GATE_GUARDS = {
    "mt.ariadne_docs": {
        "metrics": ("extra.helper_wakes", "extra.wakes_per_frame", "extra.worker_pumps"),
        "vacuous": (("extra.isolated", "docs"), ("extra.any_groups", "docs")),
        "not_applicable": "extra.wakeable",
        "control": {"where": {"frame": "heavy"}, "metric": "extra.worker_pumps", "min": 1},
    },
}


def guard_of(scenario, metric_name):
    """The GATE_GUARDS entry a gate of `metric_name` on `scenario` takes, or None."""
    g = GATE_GUARDS.get(scenario)
    return g if g and metric_name in g["metrics"] else None


def unmet_guard(rs, guard):
    """Why the rows of one point leave the gate vacuous (the guard's `vacuous`), or ""."""
    for metric_name, name in guard["vacuous"]:
        for r in rs:
            have, need = metric(r, metric_name), param(r, name)
            if have is None:
                return f"vacuous: no {metric_name}"
            try:
                below = need is not None and have < float(need)
            except ValueError:
                below = False
            if below:
                return f"vacuous: {metric_name} {have:g} below {name} {need}"
    return ""


def not_applicable(rs, guard):
    """Whether every row of one point has the guard's not_applicable metric at 0."""
    name = guard["not_applicable"]
    return all(metric(r, name) == 0 for r in rs)


def control_failures(recs, scenario, guard, executor, out):
    """Check the guard's control on `recs`' points of `scenario`; return the failures (1 when no
    point could be checked)."""
    ctl, na = guard["control"], guard["not_applicable"]
    picked = [r for r in recs
              if r.get("scenario", "") == scenario
              and (executor is None or r.get("executor", "") == executor)
              and all(param(r, k) == v for k, v in ctl["where"].items())]
    where = " ".join(f"{k}={v}" for k, v in sorted(ctl["where"].items()))
    print(f"control: median({ctl['metric']}) >= {ctl['min']:g} at {scenario}'s {where} points "
          f"that can wake a helper ({na} above 0)", file=out)
    checked = failed = 0
    groups = group(picked)
    for key in sorted(groups):
        rs = groups[key]
        label = f"  {key[0]:<28} {key[1]:<9} {key[2][:40]:<40}"
        if not all((metric(r, na) or 0) > 0 for r in rs):
            print(f"{label} {'':>12} {'':>7}  n/a ({na} 0)", file=out)
            continue
        checked += 1
        vals = [v for v in (metric(r, ctl["metric"]) for r in rs) if v is not None]
        if not all(r.get("ok", False) for r in rs) or not vals:
            print(f"{label} {'':>12} {'':>7}  FAIL (a rep failed, or no {ctl['metric']})",
                  file=out)
            failed += 1
            continue
        v = statistics.median(vals)
        verdict = "PASS" if v >= ctl["min"] else "FAIL"
        failed += verdict == "FAIL"
        print(f"{label} {v:>12.6g} {'':>7}  {verdict}", file=out)
    if not checked:
        print(f"  no {where} point that can wake a helper in the run: the gate has no control",
              file=out)
        return 1
    return failed


def gate(recs, scenario, metric_name, lo=None, hi=None, stat="median", executor=None,
         where=None, max_noise=None, out=sys.stdout):
    """Check `stat` of `metric_name` over each matching point's measured reps against [lo, hi].
    Return 1 when a point fails, a matching point has a failed rep or leaves the gate vacuous,
    nothing matches, every point is n/a or a control fails (GATE_GUARDS); else 2 when a point is
    inconclusive (noisier than max_noise percent); else 0."""
    where = where or {}
    picked = [r for r in recs
              if fnmatch.fnmatchcase(r.get("scenario", ""), scenario)
              and (executor is None or r.get("executor", "") == executor)
              and all(param(r, k) == v for k, v in where.items())]
    groups = group(picked)
    bound = " and ".join(([f">= {lo:g}"] if lo is not None else []) +
                         ([f"<= {hi:g}"] if hi is not None else []))
    print(f"gate {stat}({metric_name}) {bound}"
          + (f", noise at most {max_noise:g}%" if max_noise is not None else ""), file=out)
    if not groups:
        print("  no point matches", file=out)
        return 1
    stat_fn = {"median": statistics.median, "min": min, "max": max}[stat]
    failed = inconclusive = judged = na = 0
    guarded = {}
    for key in sorted(groups):
        rs = groups[key]
        label = f"  {key[0]:<28} {key[1]:<9} {key[2][:40]:<40}"
        guard = guard_of(key[0], metric_name)
        if guard:
            guarded[key[0]] = guard
        if not all(r.get("ok", False) for r in rs):
            print(f"{label} {'':>12} {'':>7}  FAIL (a rep failed its checks)", file=out)
            failed += 1
            continue
        unmet = unmet_guard(rs, guard) if guard else ""
        if unmet:
            print(f"{label} {'':>12} {'':>7}  FAIL ({unmet})", file=out)
            failed += 1
            continue
        if guard and not_applicable(rs, guard):
            print(f"{label} {'':>12} {'':>7}  n/a ({guard['not_applicable']} 0: no helper to "
                  "wake)", file=out)
            na += 1
            continue
        judged += 1
        vals = [v for v in (metric(r, metric_name) for r in rs) if v is not None]
        if not vals:
            print(f"{label} {'':>12} {'':>7}  FAIL (no {metric_name})", file=out)
            failed += 1
            continue
        v = stat_fn(vals)
        noise = 100.0 * noise_of(vals)
        if max_noise is not None and noise > max_noise:
            verdict = "INCONCLUSIVE"
            inconclusive += 1
        elif (lo is None or v >= lo) and (hi is None or v <= hi):
            verdict = "PASS"
        else:
            verdict = "FAIL"
            failed += 1
        print(f"{label} {v:>12.6g} {noise:>7.1f}  {verdict}", file=out)
    if na and not judged and not failed:
        print("  every point is n/a: the gate judged nothing", file=out)
        failed += 1
    for name in sorted(guarded):
        failed += control_failures(recs, name, guarded[name], executor, out)
    return 1 if failed else (2 if inconclusive else 0)


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
    c.add_argument("--fail-on-change", action="store_true",
                   help="exit 1 when any point changed beyond the noise in either direction, or "
                        "failed its checks or is missing in the candidate (for metrics such as "
                        "steps_total, where a drop is as wrong as a rise)")
    c.add_argument("--pair", metavar="KEY=A:B",
                   help="compare the base's points at KEY=A with the candidate's at KEY=B, "
                        "matched without KEY (both files may be the same run)")
    c.add_argument("--max-noise", type=float, metavar="P",
                   help="a point noisier than P percent is INCONCLUSIVE, not judged (exit 2)")
    c.add_argument("--same-pin", action="store_true",
                   help="exit 1 when a matched point's base and candidate ran their driver on "
                        "different CPUs (pin_cpu); without it they are only warned about")
    c.add_argument("--geomean-better", type=float, metavar="PCT",
                   help="per metric, the geomean over the matched points (with --pair, the "
                        "pairs) of the candidate's median over the base's must be at most "
                        "1 - PCT/100: exit 0 when it is, 1 when not (or a point failed or is "
                        "missing), 2 when --max-noise finds a pair too noisy to judge")
    g = sub.add_parser("gate", help="check a metric of one run against an absolute bound")
    g.add_argument("run")
    g.add_argument("--scenario", required=True, help="name or glob")
    g.add_argument("--executor", help="only this executor's points")
    g.add_argument("--where", action="append", metavar="K=V",
                   help="only points whose param K is V (implicit defaults apply); repeatable")
    g.add_argument("--metric", required=True, help="record field, dotted for nested")
    g.add_argument("--max", type=float, help="the statistic must be at most this")
    g.add_argument("--min", type=float, help="the statistic must be at least this")
    g.add_argument("--stat", choices=["median", "min", "max"], default="median",
                   help="over the measured reps (default median)")
    g.add_argument("--max-noise", type=float, metavar="P",
                   help="a point noisier than P percent is INCONCLUSIVE, not judged (exit 2)")
    args = ap.parse_args()
    if args.cmd == "run":
        return cmd_run(args)
    if args.cmd == "table":
        table(load(args.file))
        return 0
    if args.cmd == "gate":
        if args.max is None and args.min is None:
            ap.error("gate: give --max, --min or both")
        where = {}
        for kv in args.where or []:
            k, sep, v = kv.partition("=")
            if not (k and sep):
                ap.error(f"gate: --where wants K=V, not {kv!r}")
            where[k] = v
        return gate(load(args.run), args.scenario, args.metric, lo=args.min, hi=args.max,
                    stat=args.stat, executor=args.executor, where=where,
                    max_noise=args.max_noise)
    pair = None
    if args.pair:
        try:
            pair = parse_pair(args.pair)
        except ValueError as e:
            ap.error(str(e))
    if args.geomean_better is not None and not 0.0 <= args.geomean_better < 100.0:
        ap.error(f"compare: --geomean-better wants a percent from 0 up to (not including) 100, "
                 f"not {args.geomean_better:g}")
    metrics = args.metric or ["wall_ns", "ns_per_step"]
    res = compare(load(args.base), load(args.cand), metrics, args.threshold, pair=pair,
                  max_noise=args.max_noise, geomean_better=args.geomean_better)
    if res["unpaired"] or (args.same_pin and res["pins"]):
        return 1
    if args.fail_on_change and (res["changed"] or res["failed"]):
        return 1
    if args.fail_on_worse and (res["worse"] or res["failed"]):
        return 1
    if args.geomean_better is not None:
        verdict = res["geomean"]["verdict"]
        return 1 if verdict == "fail" else (2 if verdict == "inconclusive" else 0)
    return 2 if res["inconclusive"] else 0


if __name__ == "__main__":
    sys.exit(main())
