#!/usr/bin/env python3
"""Tests of state_word.py, the group state-word model (S2a, PL-1; README.md "The state-word model").

    python -I tools/locks_model/test_state_word.py

Runs state_word.py's quick suite (the CI lane's checks: the amended protocol finds nothing, each
finding of the frozen protocol is found without its amendment, each mutation is caught), and
checks the explorer itself: the workers' symmetry reduction finds what the full exploration
finds, a trace replays to its violation, and the liveness check's components and cycles.
Standard library only; about 30 s.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import state_word as sw  # noqa: E402


class QuickSuite(unittest.TestCase):
    def test_each_check(self):
        for check in sw.QUICK:
            with self.subTest(check[0]):
                lines = []
                ok = sw.run_check(check, lines.append)
                self.assertTrue(ok, '\n'.join(lines))


class Explorer(unittest.TestCase):
    def test_symmetry_finds_what_the_full_exploration_finds(self):
        # two workers, every amendment but host-wake, and one mutation: the same violations
        # either way, in fewer states with the reduction
        actors = dict(posters=1, workers=2, hosts=1, bodies=0, teardown=1)
        for amend, mutation in (((), ()), ('all', ('release-store',)),
                                ('all', ('pop-no-detaching',))):
            cfg = sw.make_config(actors, dict(timer=0), amend, mutation)
            full = sw.explore(cfg, symmetry=False)
            reduced = sw.explore(cfg)
            self.assertEqual(set(full['violations']), set(reduced['violations']), (amend, mutation))
            self.assertLess(reduced['states'], full['states'])
            for n in full['violations']:
                self.assertEqual(full['violations'][n]['steps'], reduced['violations'][n]['steps'], n)

    def test_a_trace_replays_to_its_violation(self):
        cfg = sw.make_config(dict(posters=1, workers=2, hosts=0, bodies=0, teardown=0), {},
                             'all', ('release-store',))
        r = sw.explore(cfg)
        v = r['violations']['lost-wake']
        self.assertIn('VIOLATION lost-wake', v['trace'][-1])
        self.assertTrue(any('stores 0' in line for line in v['trace']))

    def test_sccs(self):
        graph = {1: [2], 2: [3], 3: [1, 4], 4: [5], 5: [4], 6: [6], 7: []}
        comps = sorted(sorted(c) for c in sw.sccs(list(graph), graph.__getitem__))
        self.assertEqual(comps, [[1, 2, 3], [4, 5], [6], [7]])

    def test_a_starved_host_is_shown_by_a_cycle(self):
        # P's hold handoff alone: a group made idle and woken at once by its timer keeps a
        # with_group waiting (handoff-idle cures it)
        actors = dict(posters=0, workers=1, bodies=0, teardown=0)
        opts = dict(pre=1, repop=1, live=1)
        r = sw.explore(sw.make_config(actors, opts, sw.BUT('handoff-idle')))
        v = r['violations']['host-starved']
        self.assertIn('repeats for ever', v['trace'][-1])
        self.assertTrue(any('the deadline falls due' in line for line in v['trace']))
        self.assertEqual(sw.explore(sw.make_config(actors, opts, 'all'))['violations'], {})

    def test_the_frozen_host_release_loses_a_submit(self):
        # finding 1 (README.md, host-wake): a with_group that submits to an idle group is never
        # pumped, with no other actor
        r = sw.explore(sw.make_config(dict(posters=0, workers=1, hosts=1, bodies=0, teardown=0),
                                      dict(timer=0)))
        v = r['violations']['lost-wake']
        self.assertTrue(any('submits a process directly' in line for line in v['trace']))
        self.assertIn('runnable work left', v['detail'])


if __name__ == '__main__':
    unittest.main()
