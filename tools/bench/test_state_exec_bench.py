#!/usr/bin/env python3
# Copyright 2026 The University of Texas at Austin
#
# This file is part of libcvc.
#
# libcvc is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License version 2.1 as published by the Free Software Foundation.
"""Self-test of state_exec_bench.py's compare and gate rules, on synthetic records.

The lock phases' and threaded execution's gates (docs/STATE_EXEC_BENCH.md) are decided by these
rules: the implicit defaults that keep old records matching, --pair, the noise rule (INCONCLUSIVE,
exit 2), the change rule (a steps_total drop fails), gate's absolute bounds and its guard against
a vacuous pass (GATE_GUARDS). The records of
mt_pycvc_threads.py are checked here too, without pycvc. Standard library only; CTest runs it as
state_exec_bench_tool_test.
"""

import io
import json
import os
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stdout

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import mt_pycvc_threads as mpt  # noqa: E402
import state_exec_bench as seb  # noqa: E402


def rec(scenario, params, rep=0, ok=True, executor="async", **metrics):
    r = {"scenario": scenario, "executor": executor, "params": params, "rep": rep, "ok": ok}
    r.update(metrics)
    return r


def reps(scenario, params, values, metric="wall_ns", **kw):
    return [rec(scenario, params, rep=i, **{metric: v}, **kw) for i, v in enumerate(values)]


def quiet(fn, *args, **kwargs):
    with redirect_stdout(io.StringIO()):
        return fn(*args, **kwargs)


class Runner:
    """Runs the script as CTest users do, on records written to a temporary directory."""

    def __init__(self, tmp):
        self.tmp = tmp
        self.n = 0

    def write(self, recs):
        self.n += 1
        path = os.path.join(self.tmp, f"run{self.n}.jsonl")
        with open(path, "w", encoding="utf-8") as f:
            for r in recs:
                f.write(json.dumps(r) + "\n")
        return path

    def __call__(self, *argv):
        script = os.path.join(HERE, "state_exec_bench.py")
        p = subprocess.run([sys.executable, "-I", script] + list(argv), capture_output=True,
                           text=True)
        return p.returncode, p.stdout + p.stderr


class KeyOf(unittest.TestCase):
    def test_implicit_defaults_match_records_without_the_keys(self):
        old = rec("sched.single.fib", {"n": 20, "driver": "spin"})
        new = rec("sched.single.fib", {"n": 20, "driver": "spin", "policy": "rr",
                                       "locks": "none"})
        self.assertEqual(seb.key_of(old), seb.key_of(new))

    def test_a_value_off_its_default_is_its_own_point(self):
        a = rec("flood.small", {"n": 100, "policy": "rr"})
        b = rec("flood.small", {"n": 100, "policy": "priority_rr"})
        self.assertNotEqual(seb.key_of(a), seb.key_of(b))
        self.assertIn("policy=priority_rr", seb.key_of(b)[2])

    def test_param_falls_back_to_the_implicit_default(self):
        r = rec("msg.pingpong", {"r": 2000})
        self.assertEqual(seb.param(r, "locks"), "none")
        self.assertEqual(seb.param(r, "r"), "2000")
        self.assertIsNone(seb.param(r, "missing"))

    def test_stage0_keys_match_records_without_them(self):
        # Threaded Stage 0 gave native.state_get/state_set and sched.watch_poll a chroot key, and
        # sched.watch_poll a writes key: a base binary's points match the candidate's at chroot=0
        # and writes=none, so compare doesn't count them "only in base".
        for scenario, old in [("native.state_get", {"n": 20000, "depth": 4, "rounds": 3}),
                              ("native.state_set", {"n": 20000, "depth": 1, "rounds": 3}),
                              ("sched.watch_poll", {"w": 100, "watch": 1, "steps": 50000})]:
            new = dict(old, chroot=0)
            if scenario == "sched.watch_poll":
                new["writes"] = "none"
            self.assertEqual(seb.key_of(rec(scenario, old)), seb.key_of(rec(scenario, new)))
        # lock.uncontended always had chroot: both sides drop chroot=0 and still match.
        a = rec("lock.uncontended", {"n": 10000, "chroot": 0})
        b = rec("lock.uncontended", {"n": 10000, "chroot": 0})
        self.assertEqual(seb.key_of(a), seb.key_of(b))

    def test_stage0_points_off_the_defaults_are_their_own(self):
        base = rec("sched.watch_poll", {"w": 100, "watch": 1, "steps": 50000})
        for extra in [{"chroot": 1}, {"writes": "elsewhere"}, {"chroot": 1, "writes": "elsewhere"}]:
            other = rec("sched.watch_poll", dict(base["params"], **extra))
            self.assertNotEqual(seb.key_of(base), seb.key_of(other), extra)
        self.assertIn("chroot=1", seb.key_of(rec("native.state_get", {"n": 1, "chroot": 1}))[2])
        self.assertEqual(seb.param(base, "writes"), "none")

    def test_stage1_base_key_matches_records_without_it(self):
        # Threaded Stage 1 gave sched.single.*'s async points a base key (own by default) beside
        # their driver key: a base binary's point (no base) matches the candidate's at base=own.
        old = rec("sched.single.fib", {"n": 20, "driver": "spin", "policy": "rr",
                                       "locks": "none"})
        new = rec("sched.single.fib", dict(old["params"], base="own"))
        self.assertEqual(seb.key_of(old), seb.key_of(new))
        self.assertEqual(seb.param(old, "base"), "own")
        frozen = rec("sched.single.fib", dict(old["params"], driver="group", base="frozen"))
        self.assertIn("base=frozen", seb.key_of(frozen)[2])
        self.assertIn("driver=group", seb.key_of(frozen)[2])


class ParsePair(unittest.TestCase):
    def test_parses(self):
        self.assertEqual(seb.parse_pair("op=set:incr"), ("op", "set", "incr"))

    def test_rejects(self):
        for bad in ["op", "op=set", "op=set:", "=a:b", "op=set:set"]:
            with self.assertRaises(ValueError, msg=bad):
                seb.parse_pair(bad)


class Compare(unittest.TestCase):
    def test_pair_compares_two_points_of_one_run(self):
        run = (reps("native.state_atomic", {"op": "set", "n": 100}, [100, 101, 99]) +
               reps("native.state_atomic", {"op": "incr", "n": 100}, [150, 151, 149]) +
               reps("native.state_atomic", {"n": 100}, [1, 1, 1]))  # neither value: left out
        res = quiet(seb.compare, run, run, ["wall_ns"], 10.0, pair=("op", "set", "incr"))
        self.assertEqual(res, {"worse": 1, "changed": 1, "inconclusive": 0, "failed": 0, "unpaired": 0,
                               "pins": 0})
        res = quiet(seb.compare, run, run, ["wall_ns"], 60.0, pair=("op", "set", "incr"))
        self.assertEqual(res["changed"], 0)

    def test_pair_on_an_implicit_default(self):
        run = (reps("flood.small", {"n": 10}, [100, 100]) +  # policy absent: rr
               reps("flood.small", {"n": 10, "policy": "priority_rr"}, [100, 100]))
        res = quiet(seb.compare, run, run, ["wall_ns"], 3.0, pair=("policy", "rr", "priority_rr"))
        self.assertEqual(res, {"worse": 0, "changed": 0, "inconclusive": 0, "failed": 0, "unpaired": 0,
                               "pins": 0})

    def test_one_disturbed_rep_is_inconclusive(self):
        base = reps("sched.single.fib", {"n": 20}, [100, 100, 100, 100, 100])
        cand = reps("sched.single.fib", {"n": 20}, [100, 101, 100, 160, 100])
        res = quiet(seb.compare, base, cand, ["wall_ns"], 3.0, max_noise=3.0)
        self.assertEqual(res["inconclusive"], 1)
        self.assertEqual(res["worse"], 0)

    def test_a_missing_candidate_point_fails(self):
        base = reps("msg.fanout", {"k": 10}, [1, 1])
        res = quiet(seb.compare, base, [], ["wall_ns"], 3.0)
        self.assertEqual(res["failed"], 1)

    def test_a_value_leaving_zero_is_a_change(self):
        base = reps("eval.fib", {"n": 1}, [0, 0], metric="steps_total")
        cand = reps("eval.fib", {"n": 1}, [5, 5], metric="steps_total")
        res = quiet(seb.compare, base, cand, ["steps_total"], 0.0)
        self.assertEqual(res["changed"], 1)

    def test_pair_chroot_in_one_run(self):
        # Stage 0's chroot=1 points are paired with chroot=0 in the candidate's own run.
        run = (reps("native.state_get", {"n": 100, "depth": 4}, [100, 101, 99]) +  # chroot 0
               reps("native.state_get", {"n": 100, "depth": 4, "chroot": 1}, [130, 131, 129]))
        res = quiet(seb.compare, run, run, ["wall_ns"], 3.0, pair=("chroot", "0", "1"))
        self.assertEqual(res, {"worse": 1, "changed": 1, "inconclusive": 0, "failed": 0, "unpaired": 0,
                               "pins": 0})

    def test_pair_driver_and_base_in_one_run(self):
        # Stage 1's paired points: driver=group against today's driver=spin at base=own, and the
        # frozen base against its own on driver=group. A group point with no spin twin
        # (base=frozen) is only in the candidate, which fails nothing.
        p = {"n": 100, "policy": "rr", "locks": "none"}
        run = (reps("sched.single.fib", dict(p, driver="spin", base="own"), [100, 101, 99]) +
               reps("sched.single.fib", dict(p, driver="group", base="own"), [101, 102, 100]) +
               reps("sched.single.fib", dict(p, driver="group", base="frozen"), [110, 111, 109]))
        res = quiet(seb.compare, run, run, ["wall_ns"], 3.0, pair=("driver", "spin", "group"))
        self.assertEqual(res, {"worse": 0, "changed": 0, "inconclusive": 0, "failed": 0, "unpaired": 0,
                               "pins": 0})
        groups = [r for r in run if r["params"]["driver"] == "group"]
        res = quiet(seb.compare, groups, groups, ["wall_ns"], 3.0, pair=("base", "own", "frozen"))
        self.assertEqual(res, {"worse": 1, "changed": 1, "inconclusive": 0, "failed": 0, "unpaired": 0,
                               "pins": 0})
        # A base binary's spin points carry no base, and still pair with the group's base=own.
        old = [dict(r, params={k: v for k, v in r["params"].items() if k != "base"})
               for r in run if r["params"]["driver"] == "spin"]
        res = quiet(seb.compare, old + groups, old + groups, ["wall_ns"], 3.0,
                    pair=("driver", "spin", "group"))
        self.assertEqual(res["failed"], 0)
        self.assertEqual(res["changed"], 0)

    def test_a_pair_matching_nothing_on_one_side_fails(self):
        # Stage 1's review (R1, M4): a --pair value that no record has (driver=bare, where the
        # bench says spin) left the base side empty, every point "only in cand", and passed.
        p = {"n": 100, "policy": "rr", "locks": "none"}
        run = (reps("sched.single.fib", dict(p, driver="spin"), [100, 101, 99]) +
               reps("sched.single.fib", dict(p, driver="group"), [101, 102, 100]))
        res = quiet(seb.compare, run, run, ["wall_ns"], 3.0, pair=("driver", "bare", "group"))
        self.assertEqual(res["unpaired"], 1)
        res = quiet(seb.compare, run, run, ["wall_ns"], 3.0, pair=("base", "own", "frozn"))
        self.assertEqual(res["unpaired"], 1)
        res = quiet(seb.compare, run, run, ["wall_ns"], 3.0, pair=("driver", "bare", "grup"))
        self.assertEqual(res["unpaired"], 1)  # neither value anywhere
        res = quiet(seb.compare, run, run, ["wall_ns"], 3.0, pair=("driver", "spin", "group"))
        self.assertEqual(res["unpaired"], 0)

    def test_differing_driver_cpus_are_counted(self):
        # R1, M2: an A/B whose binaries pinned the driver to different CPUs.
        base = reps("sched.single.fib", {"n": 20}, [100, 100], pin_cpu=10)
        cand = reps("sched.single.fib", {"n": 20}, [100, 100], pin_cpu=18)
        self.assertEqual(quiet(seb.compare, base, cand, ["wall_ns"], 3.0)["pins"], 1)
        same = reps("sched.single.fib", {"n": 20}, [100, 100], pin_cpu=10)
        self.assertEqual(quiet(seb.compare, base, same, ["wall_ns"], 3.0)["pins"], 0)
        unknown = reps("sched.single.fib", {"n": 20}, [100, 100])  # no pin_cpu: not compared
        self.assertEqual(quiet(seb.compare, base, unknown, ["wall_ns"], 3.0)["pins"], 0)

    def test_of_ceiling_is_better_higher(self):
        def point(value):
            recs = reps("mt.multi_exec", {"t": 8, "prog": "fib", "runs": 5}, [1, 1, 1])
            for r in recs:
                r["extra"] = {"of_ceiling": value}
            return recs
        res = quiet(seb.compare, point(0.95), point(0.80), ["extra.of_ceiling"], 3.0)
        self.assertEqual(res["worse"], 1)
        res = quiet(seb.compare, point(0.80), point(0.95), ["extra.of_ceiling"], 3.0)
        self.assertEqual((res["worse"], res["changed"]), (0, 1))


class CompareExit(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.TemporaryDirectory()
        self.run = Runner(self.tmpdir.name)

    def tearDown(self):
        self.tmpdir.cleanup()

    def test_steps_drop_fails_on_change_but_not_on_worse(self):
        base = self.run.write(reps("eval.fib", {"n": 20}, [1000, 1000], metric="steps_total"))
        cand = self.run.write(reps("eval.fib", {"n": 20}, [990, 990], metric="steps_total"))
        args = ["compare", base, cand, "--metric", "steps_total", "--threshold", "0"]
        self.assertEqual(self.run(*args, "--fail-on-change")[0], 1)
        self.assertEqual(self.run(*args, "--fail-on-worse")[0], 0)
        same = self.run.write(reps("eval.fib", {"n": 20}, [1000, 1000], metric="steps_total"))
        self.assertEqual(self.run("compare", base, same, "--metric", "steps_total",
                                  "--threshold", "0", "--fail-on-change")[0], 0)

    def test_inconclusive_exits_2_and_failure_wins(self):
        base = self.run.write(reps("msg.pingpong", {"r": 10}, [100, 100, 100, 100, 100]))
        noisy = self.run.write(reps("msg.pingpong", {"r": 10}, [100, 100, 170, 100, 100]))
        code, out = self.run("compare", base, noisy, "--metric", "wall_ns", "--max-noise", "3",
                             "--fail-on-worse")
        self.assertEqual(code, 2, out)
        self.assertIn("INCONCLUSIVE", out)
        # A failed point decides the run even when another is inconclusive.
        failed = self.run.write(
            reps("msg.pingpong", {"r": 10}, [100, 100, 170, 100, 100]) +
            [rec("msg.fanout", {"k": 10}, ok=False, wall_ns=1)])
        both = self.run.write(reps("msg.pingpong", {"r": 10}, [100] * 5) +
                              reps("msg.fanout", {"k": 10}, [1, 1]))
        self.assertEqual(self.run("compare", both, failed, "--metric", "wall_ns",
                                  "--max-noise", "3", "--fail-on-worse")[0], 1)

    def test_an_unpaired_pair_exits_1(self):
        run = self.run.write(reps("sched.single.fib", {"n": 1, "driver": "spin"}, [1, 1]) +
                             reps("sched.single.fib", {"n": 1, "driver": "group"}, [1, 1]))
        code, out = self.run("compare", run, run, "--pair", "driver=bare:group")
        self.assertEqual(code, 1, out)
        self.assertIn("has no base record at driver=bare", out)
        self.assertEqual(self.run("compare", run, run, "--pair", "driver=spin:group")[0], 0)

    def test_same_pin_fails_differing_cpus(self):
        base = self.run.write(reps("eval.fib", {"n": 20}, [100, 100], pin_cpu=10))
        cand = self.run.write(reps("eval.fib", {"n": 20}, [100, 100], pin_cpu=18))
        code, out = self.run("compare", base, cand)
        self.assertEqual(code, 0, out)  # warned only
        self.assertIn("the driver ran on CPU 10 in base and 18 in cand", out)
        self.assertEqual(self.run("compare", base, cand, "--same-pin")[0], 1)
        self.assertEqual(self.run("compare", base, base, "--same-pin")[0], 0)

    def test_bad_pair_is_a_usage_error(self):
        f = self.run.write(reps("eval.fib", {"n": 1}, [1]))
        self.assertEqual(self.run("compare", f, f, "--pair", "op=set:set")[0], 2)


class Gate(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.TemporaryDirectory()
        self.run = Runner(self.tmpdir.name)

    def tearDown(self):
        self.tmpdir.cleanup()

    def gate(self, recs, *extra):
        return self.run("gate", self.run.write(recs), *extra)

    def test_max_and_min(self):
        recs = reps("lock.uncontended", {"owner": "short"}, [40, 50, 55], metric="extra.ns")
        recs = [dict(r, extra={"ns": r.pop("extra.ns")}) for r in recs]
        base = ["--scenario", "lock.*", "--metric", "extra.ns"]
        self.assertEqual(self.gate(recs, *base, "--max", "60")[0], 0)
        self.assertEqual(self.gate(recs, *base, "--max", "45")[0], 1)
        self.assertEqual(self.gate(recs, *base, "--max", "60", "--stat", "max")[0], 0)
        self.assertEqual(self.gate(recs, *base, "--max", "54", "--stat", "max")[0], 1)
        # --min and --max together pin a value.
        steps = reps("lock.uncontended", {"owner": "short"}, [3, 3, 3], metric="steps")
        pin = ["--scenario", "lock.uncontended", "--metric", "steps", "--min", "3", "--max", "3"]
        self.assertEqual(self.gate(steps, *pin)[0], 0)
        steps4 = reps("lock.uncontended", {"owner": "short"}, [4, 4, 4], metric="steps")
        self.assertEqual(self.gate(steps4, *pin)[0], 1)

    def test_where_and_executor_select_points(self):
        recs = (reps("lock.rw", {"readers": 64}, [10, 10], metric="p") +
                reps("lock.rw", {"readers": 1}, [99, 99], metric="p") +
                reps("lock.rw", {"readers": 64}, [99, 99], metric="p", executor="sync"))
        code, out = self.gate(recs, "--scenario", "lock.rw", "--metric", "p", "--max", "20",
                              "--where", "readers=64", "--executor", "async")
        self.assertEqual(code, 0, out)
        # --where takes implicit defaults: these records have no locks param at all.
        self.assertEqual(self.gate(recs, "--scenario", "lock.rw", "--metric", "p", "--max", "20",
                                   "--where", "readers=64", "--where", "locks=none",
                                   "--executor", "async")[0], 0)

    def test_nothing_matching_fails(self):
        recs = reps("eval.fib", {"n": 1}, [1, 1])
        self.assertEqual(self.gate(recs, "--scenario", "lock.*", "--metric", "wall_ns",
                                   "--max", "1")[0], 1)

    def test_a_failed_rep_fails(self):
        recs = reps("lock.x", {"n": 1}, [1, 1]) + [rec("lock.x", {"n": 1}, rep=2, ok=False)]
        self.assertEqual(self.gate(recs, "--scenario", "lock.x", "--metric", "wall_ns",
                                   "--max", "5")[0], 1)

    def test_noise_rule(self):
        recs = reps("lock.x", {"n": 1}, [100, 100, 100, 100, 180])
        args = ["--scenario", "lock.x", "--metric", "wall_ns", "--max", "500"]
        self.assertEqual(self.gate(recs, *args)[0], 0)
        self.assertEqual(self.gate(recs, *args, "--max-noise", "10")[0], 2)

    def test_needs_a_bound(self):
        recs = reps("lock.x", {"n": 1}, [1])
        self.assertEqual(self.gate(recs, "--scenario", "lock.x", "--metric", "wall_ns")[0], 2)

    def test_multi_exec_gate_names_its_prog(self):
        # Stage 0's gate: mt.multi_exec at t=8 at least 0.9 of mt.independent's speedup, per prog.
        # prog=tree has no ceiling, so a gate that didn't name prog would fail on it.
        def point(prog, t, values):
            out = []
            for i, v in enumerate(values):
                r = rec("mt.multi_exec", {"t": t, "prog": prog, "runs": 5}, rep=i, wall_ns=1)
                r["extra"] = {"speedup": 7.0} if v is None else {"speedup": 7.0, "of_ceiling": v}
                out.append(r)
            return out
        recs = (point("fib", 8, [0.95, 0.96, 0.94]) + point("exceptions", 8, [0.85, 0.86, 0.84]) +
                point("tree", 8, [None, None, None]) + point("fib", 4, [0.5, 0.5, 0.5]))
        gate = ["--scenario", "mt.multi_exec", "--where", "t=8", "--metric", "extra.of_ceiling",
                "--min", "0.9", "--max-noise", "10"]
        code, out = self.gate(recs, *gate, "--where", "prog=fib")
        self.assertEqual(code, 0, out)
        self.assertEqual(self.gate(recs, *gate, "--where", "prog=exceptions")[0], 1)
        code, out = self.gate(recs, *gate)
        self.assertEqual(code, 1, out)
        self.assertIn("no extra.of_ceiling", out)


    def test_groups_gate_takes_every_groups_size(self):
        # Stage 1's gate: mt.groups fib at t=8 at least 0.9 of mt.independent's speedup, which
        # holds for every groups size the run has (8 and 16).
        def point(groups, prog, values):
            out = []
            for i, v in enumerate(values):
                r = rec("mt.groups", {"t": 8, "groups": groups, "prog": prog, "runs": 5}, rep=i,
                        wall_ns=1)
                r["extra"] = {"of_ceiling": v, "helper_wakes": 7, "effective_threads": 7}
                out.append(r)
            return out
        gate = ["--scenario", "mt.groups", "--where", "t=8", "--where", "prog=fib", "--metric",
                "extra.of_ceiling", "--min", "0.9", "--max-noise", "10"]
        eight = point(8, "fib", [0.95, 0.94, 0.96])
        code, out = self.gate(eight + point(16, "fib", [0.93, 0.92, 0.93]) +
                              point(8, "cycle", [0.5, 0.5, 0.5]), *gate)
        self.assertEqual(code, 0, out)
        self.assertEqual(self.gate(eight + point(16, "fib", [0.85, 0.85, 0.85]), *gate)[0], 1)

    def test_ariadne_docs_gate_needs_every_document_isolated(self):
        # Stage 1b's gate: mt.ariadne_docs' small frames wake no helper. It holds only where every
        # document is in a group of its own (extra.isolated == docs): in group 0 no helper is ever
        # woken, so the gate fails as vacuous rather than passing.
        def point(t, docs, frame, wakes, isolated):
            out = []
            for i, w in enumerate(wakes):
                r = rec("mt.ariadne_docs", {"t": t, "docs": docs, "frame": frame, "frames": 200},
                        rep=i, wall_ns=1)
                r["extra"] = {"helper_wakes": w, "isolated": isolated, "worker_pumps": 0}
                out.append(r)
            return out
        gate = ["--scenario", "mt.ariadne_docs", "--where", "frame=small", "--metric",
                "extra.helper_wakes", "--max", "0"]
        isolated = (point(0, 1, "small", [0, 0, 0], 1) + point(4, 8, "small", [0, 0, 0], 8) +
                    point(4, 8, "heavy", [40, 41, 39], 8))
        code, out = self.gate(isolated, *gate)
        self.assertEqual(code, 0, out)
        # Small frames that woke a helper fail: in the median rep, or in any with --stat max.
        self.assertEqual(self.gate(isolated + point(2, 2, "small", [0, 1, 1], 2), *gate)[0], 1)
        once = isolated + point(2, 2, "small", [0, 1, 0], 2)
        self.assertEqual(self.gate(once, *gate)[0], 0)
        self.assertEqual(self.gate(once, *gate, "--stat", "max")[0], 1)
        # Documents left in group 0 (S1b.0's stubs: extra.isolated 0) make it vacuous: it fails.
        code, out = self.gate(point(2, 2, "small", [0, 0, 0], 0), *gate)
        self.assertEqual(code, 1, out)
        self.assertIn("vacuous: extra.isolated 0 below docs 2", out)
        # One rep with a document short is enough.
        partly = point(4, 4, "small", [0, 0], 4) + point(4, 4, "small", [0], 3)[0:1]
        partly[-1]["rep"] = 2
        self.assertEqual(self.gate(partly, *gate)[0], 1)
        # A record without extra.isolated (a binary before the scenario had it) cannot pass.
        bare = point(2, 2, "small", [0, 0], 2)
        for r in bare:
            del r["extra"]["isolated"]
        code, out = self.gate(bare, *gate)
        self.assertEqual(code, 1, out)
        self.assertIn("vacuous: no extra.isolated", out)
        # Other scenarios' gates take no guard: a record without extra.isolated passes there.
        other = reps("mt.groups", {"t": 8, "groups": 8, "prog": "fib", "runs": 5}, [0, 0],
                     metric="extra.helper_wakes")
        other = [dict(r, extra={"helper_wakes": r.pop("extra.helper_wakes")}) for r in other]
        self.assertEqual(self.gate(other, "--scenario", "mt.groups", "--metric",
                                   "extra.helper_wakes", "--max", "0")[0], 0)


class MtPycvcThreads(unittest.TestCase):
    """mt_pycvc_threads.py's programs and records, without pycvc."""

    def test_programs_are_mt_independents(self):
        src, expect, iters = mpt.program("fib", smoke=False)
        self.assertIn("(fib 18)", src)
        self.assertEqual((expect, iters), ("2584", 2 * 4181 - 1))
        src, expect, iters = mpt.program("exceptions", smoke=False)
        self.assertIn("(while (< i 2000)", src)
        self.assertEqual((expect, iters), ("2000", 2000))
        self.assertEqual(mpt.program("fib", smoke=True)[1], "55")
        with self.assertRaises(ValueError):
            mpt.program("tree", smoke=False)

    def test_an_unknown_prog_fails_before_pycvc_loads(self):
        with self.assertRaises(ValueError):
            mpt.main(["--prog", "fib,tree"])
        self.assertNotIn("pycvc", sys.modules)

    def test_records_are_the_runners(self):
        ok = mpt.record(4, "fib", 5, 0, 100, wall=1000, one=2000, spans=[900, 950, 1000, 990],
                        errors=[], smoke=True)
        self.assertEqual(seb.key_of(ok), ("mt.pycvc_threads", "pycvc", "prog=fib runs=5 t=4"))
        self.assertTrue(ok["ok"])
        self.assertEqual(seb.metric(ok, "extra.speedup"), 4 * 2000 / 1000)
        self.assertEqual(seb.metric(ok, "ns_per_iter"), 1000 / (4 * 5 * 100))
        self.assertEqual(seb.metric(ok, "extra.thread_ns_per_iter"), 970 / (5 * 100))
        bad = mpt.record(4, "fib", 5, 1, 100, wall=1000, one=2000, spans=[900, 0, 0, 0],
                         errors=["thread 1: RuntimeError: returned 1, expected 2584"], smoke=True)
        self.assertFalse(bad["ok"])
        self.assertIn("thread 1", bad["error"])
        buf = io.StringIO()
        seb.table([ok, bad], out=buf)
        self.assertIn("mt.pycvc_threads", buf.getvalue())
        self.assertIn("error: thread 1", buf.getvalue())
        self.assertEqual(json.loads(json.dumps(ok)), ok)  # a JSON line


if __name__ == "__main__":
    unittest.main()
