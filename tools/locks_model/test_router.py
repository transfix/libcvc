#!/usr/bin/env python3
"""Tests of router.py, the model of the router protocol across exec groups (S2b; README.md "The
router model").

    python -I tools/locks_model/test_router.py

Runs router.py's quick suite (the CI lane's checks: the amended protocol finds nothing but the
documented gaps, each finding of the frozen text is found without its amendment, each mutation is
caught), and checks the explorer itself: the compact storage finds what the plain one finds, the
state codec is canonical, a trace replays to its violation, the internal choices are enumerated,
the state graph has no cycle (so the idle checks are the liveness check), and a poster waits on a
closed home (rehome-order). Standard library only; about 25 s.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import router as rt  # noqa: E402


class QuickSuite(unittest.TestCase):
    def test_each_check(self):
        for check in rt.QUICK:
            with self.subTest(check[0]):
                lines = []
                ok = rt.run_check(check, lines.append, compact=False)
                self.assertTrue(ok, '\n'.join(lines))


class Explorer(unittest.TestCase):
    def test_compact_storage_finds_what_the_plain_one_finds(self):
        for procs, opts, amend in (('S:ss,R1:t', {}, ()), ('H:s,R1:t,R2:t', {}, rt.ALL),
                                   ('S:s,H:r,R1:t,R2:r', dict(teardown=0, choice=1), ())):
            cfg = rt.make_config(procs, opts, amend)
            a = rt.explore(cfg, compact=False)
            b = rt.explore(cfg, compact=True)
            self.assertEqual((a['states'], a['transitions'], a['idle_states']),
                             (b['states'], b['transitions'], b['idle_states']), procs)
            self.assertEqual(sorted(a['violations']), sorted(b['violations']), procs)
            for n in a['violations']:
                self.assertEqual(a['violations'][n]['trace'], b['violations'][n]['trace'], n)

    def test_the_codec_is_canonical(self):
        x = (1, 2)
        self.assertEqual(rt.encode(((1, 2), (1, 2))), rt.encode((x, x)))
        m = rt.Model(rt.make_config('S:ss,R1:t,R2:t', dict(kills=1)))
        s = m.initial()
        self.assertEqual(rt.decode(rt.encode(s)), s)

    def test_a_trace_replays_to_its_violation(self):
        # the frozen forward order: the queued subscribes reach G0 before the pending message,
        # which G0 then broadcasts to both
        r = rt.explore(rt.make_config('S:s,R1:r,R2:r', {}, rt.BUT('forward-pending-first')))
        v = r['violations']['duplicate']
        self.assertIn('VIOLATION duplicate', v['trace'][-1])
        self.assertTrue(any('to forward: subscribe' in line for line in v['trace']))
        self.assertEqual(v['steps'], len(v['trace']) - 1)

    def test_the_frozen_teardown_unregisters_before_it_closes(self):
        r = rt.explore(rt.make_config('S:ss,R1:t', {}, ()))
        trace = r['violations']['fifo']['trace']
        unreg = [i for i, line in enumerate(trace) if 'unregisters' in line]
        self.assertTrue(unreg)
        self.assertIn('posts send(m2) to G0', ' '.join(trace[unreg[0]:]))

    def test_the_next_waiter_is_chosen_every_way(self):
        m = rt.Model(rt.make_config('S:s,H:r,R1:r', dict(teardown=0, choice=1)))
        # a requeue at H with a local waiter and a record: two outcomes
        s = list(m.initial())
        loc, recs, pend, trs, nid = s[rt.HOMES][rt.H]
        homes = list(s[rt.HOMES])
        homes[rt.H] = ((1,), ((rt.R1, 2, 1, False),), pend, trs, nid)
        s[rt.HOMES] = tuple(homes)
        s[rt.PROCS] = ((1, rt.DONE, 0, 0), (0, rt.WL, 1, 0), (0, rt.WR, 1, 0))
        s[rt.MSGS] = ((rt.M_SENT, (), 0, 0, rt.F_ASSIGNED),)
        sent = [()] * rt.NG
        sent[rt.S] = (0,)
        s[rt.SENT] = tuple(sent)
        ing = list(s[rt.ING])
        ing[rt.H] = (('Q', 0, 0),)
        s[rt.ING] = tuple(ing)
        outs = m.successors(tuple(s), ('drain', rt.H))
        self.assertEqual(sorted(o[0] for o in outs), [(0,), (1,)])
        self.assertEqual(len({o[1] for o in outs}), 2)
        m2 = rt.Model(rt.make_config('S:s,H:r,R1:r', dict(teardown=0, choice=1),
                                     ('next-waiter-order',)))
        self.assertEqual(len(m2.successors(tuple(s), ('drain', rt.H))), 1)

    def test_the_state_graph_has_no_cycle(self):
        # so every fair schedule reaches an idle state, and the idle checks are the liveness check
        for procs, opts, amend in (('H:s,R1:t,R2:t', dict(kills=1), rt.ALL),
                                   ('S:s,H:r,R1:t', dict(rdie=1, throws=1, choice=1), ()),
                                   ('S:s,R1:t', dict(islands=1), rt.BUT('island-rule')),
                                   ('X:p,R1:t,R2:t', dict(islands=1, rdie=1), rt.ALL)):
            m = rt.Model(rt.make_config(procs, opts, amend))
            succ = {}
            todo = [m.initial()]
            while todo:
                s = todo.pop()
                if s in succ:
                    continue
                out = []
                for label in m.options(s):
                    try:
                        out += [o[1] for o in m.successors(s, label)]
                    except (rt.Blocked, rt.Violation):
                        pass
                succ[s] = out
                todo += out
            colour = dict.fromkeys(succ, 0)
            for root in succ:
                if colour[root]:
                    continue
                colour[root] = 1
                stack = [(root, iter(succ[root]))]
                while stack:
                    v, it = stack[-1]
                    for w in it:
                        self.assertNotEqual(colour[w], 1, 'a cycle: ' + procs)
                        if colour[w] == 0:
                            colour[w] = 1
                            stack.append((w, iter(succ[w])))
                            break
                    else:
                        colour[v] = 2
                        stack.pop()

    def test_a_poster_waits_on_a_closed_home(self):
        # rehome-order: H closed and still registered; R1's subscribe waits for the unregister
        m = rt.Model(rt.make_config('R1:r', {}, rt.ALL))
        s = m.successors(m.initial(), ('td', rt.H))[0][1]
        self.assertTrue(s[rt.CLOSED][rt.H] and s[rt.REG])
        with self.assertRaises(rt.Blocked):
            m.successors(s, ('step', 0))
        s = m.successors(s, ('td', rt.H))[0][1]   # nothing to forward: unregisters
        self.assertFalse(s[rt.REG])
        s = m.successors(s, ('step', 0))[0][1]
        self.assertEqual(s[rt.ING][rt.G0][0][0], 'U')


if __name__ == '__main__':
    unittest.main()
