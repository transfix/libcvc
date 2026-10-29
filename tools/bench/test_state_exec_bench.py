#!/usr/bin/env python3
# Copyright 2026 The University of Texas at Austin
#
# This file is part of libcvc.
#
# libcvc is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License version 2.1 as published by the Free Software Foundation.
"""Self-test of state_exec_bench.py's compare and gate rules, on synthetic records.

The lock phases' gates (docs/STATE_EXEC_BENCH.md) are decided by these rules: the implicit
defaults that keep old records matching, --pair, the noise rule (INCONCLUSIVE, exit 2), the change
rule (a steps_total drop fails) and gate's absolute bounds. Standard library only; CTest runs it as
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
        self.assertEqual(res, {"worse": 1, "changed": 1, "inconclusive": 0, "failed": 0})
        res = quiet(seb.compare, run, run, ["wall_ns"], 60.0, pair=("op", "set", "incr"))
        self.assertEqual(res["changed"], 0)

    def test_pair_on_an_implicit_default(self):
        run = (reps("flood.small", {"n": 10}, [100, 100]) +  # policy absent: rr
               reps("flood.small", {"n": 10, "policy": "priority_rr"}, [100, 100]))
        res = quiet(seb.compare, run, run, ["wall_ns"], 3.0, pair=("policy", "rr", "priority_rr"))
        self.assertEqual(res, {"worse": 0, "changed": 0, "inconclusive": 0, "failed": 0})

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


if __name__ == "__main__":
    unittest.main()
