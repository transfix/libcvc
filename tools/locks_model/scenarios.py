"""Directed scenarios from the design's test list, as a sanity check of the model, and the
section-7 mutations (each must produce its expected counterexample)."""

import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))

import sys
from lockmodel import K, S, X, INF, Violation, canonical, STATS
from world import Config, World

REQS = {}


def mk(homes, procs, reqs, kinds=('wait', 'wait_dl', 'ifavail', 't0'), mutation=None, **kw):
    cfg = Config(homes, procs, reqs, kinds=kinds, max_depth=3, max_hdepth=2, signals=2,
                 kills=(True, False), destroy=True, prio_values=(-10, -2, 0, 2, 5, 10),
                 mutation=mutation, prio_changes=3, req_per_proc=5, **kw)
    return World(cfg)


def settle(w):
    """Run every pending home step and post to quiescence, checking after each."""
    while True:
        did = False
        for hid in sorted(w.homes):
            h = w.homes[hid]
            if not h.destroyed and h.pending:
                w.do(('hstep', hid)); w.check(); did = True
                break
        if did:
            continue
        for hid in sorted(w.homes):
            h = w.homes[hid]
            if not h.destroyed and h.port.queue:
                w.do(('post', hid)); w.check(); did = True
                break
        if not did:
            return


def run(w, script):
    for a in script:
        w.do(a)
        w.check()
        settle(w)


def outcome_of(w, pid):
    p = w.procs[pid]
    return p.status, [(f.kind, f.hold, f.outcome) for f in p.frames], [(h.id, h.count) for h in p.locks]


def req_index(w, keys, mode):
    reqs = w.cfg.requests
    t = (canonical(keys), mode)
    for i, r in enumerate(reqs):
        if (canonical(r[0]), r[1]) == t:
            return i
    reqs.append(t)
    return len(reqs) - 1


def R(w, pid, keys, mode, kind='wait'):
    return ('req', pid, req_index(w, [K(k) for k in keys], mode), kind)


def scenario_lk27(mutation=None):
    # P holds S a.x; Q's X on a queues; P nests X a.y: granted at once
    w = mk([(1, True), (2, True)], [(1, 1, 'o', 0), (2, 2, 'o', 0)], [], mutation=mutation)
    run(w, [R(w, 1, ['a.x'], S), R(w, 2, ['a'], X), R(w, 1, ['a.y'], X)])
    p1 = w.procs[1]
    assert p1.status == 'run' and len(p1.locks) == 2, outcome_of(w, 1)
    assert p1.frames[-1].kind == 'body', outcome_of(w, 1)
    run(w, [('finish', 1), ('finish', 1)])
    assert w.procs[2].status == 'run'
    run(w, [('enter_step', 2)])
    assert w.procs[2].frames[-1].kind == 'body'
    w.drain()
    return 'ok'


def scenario_lk27_deadlock_when_no_exempt():
    w = mk([(1, True), (2, True)], [(1, 1, 'o', 0), (2, 2, 'o', 0)], [], mutation={'no-exempt'})
    run(w, [R(w, 1, ['a.x'], S), R(w, 2, ['a'], X), R(w, 1, ['a.y'], X)])
    p1 = w.procs[1]
    f = p1.frames[-1]
    assert f.kind == 'enter' and f.outcome and f.outcome[0] == 'DeadlockError', outcome_of(w, 1)
    return 'DeadlockError as expected: %r' % (f.outcome,)


def scenario_lk25(mutation=None):
    # H holds S k; W queues X k; R queues S k; W times out: R granted at once
    w = mk([(1, True), (2, True)], [(1, 1, 'o', 0), (2, 2, 'o', 0), (1, 3, 'o', 0)], [], mutation=mutation)
    run(w, [R(w, 1, ['k'], S), R(w, 2, ['k'], X, 'wait_dl'), R(w, 3, ['k'], S)])
    assert w.procs[3].status == 'parked'
    run(w, [('deadline', 2)])
    assert w.procs[3].status == 'run', outcome_of(w, 3)
    assert w.procs[2].frames[-1].outcome == ('TimeoutError',)
    w.drain()
    return 'ok'


def scenario_lk21():
    w = mk([(1, True), (2, True)], [(1, 1, 'o', 0), (2, 2, 'o', 0), (1, 3, 'o', 0), (2, 4, 'o', 0), (1, 5, 'o', 0)], [])
    run(w, [R(w, 1, ['a'], X), R(w, 2, ['a'], S), R(w, 3, ['a'], S), R(w, 4, ['a'], X), R(w, 5, ['a'], S)])
    run(w, [('finish', 1)])
    st = [w.procs[i].status for i in (2, 3, 4, 5)]
    assert st == ['run', 'run', 'parked', 'parked'], st
    w.drain()
    return 'ok'


def scenario_ld10():
    # P holds S a.x; R queues X a; Q queues S a.q; P requests X a.q.w: DeadlockError (P, Q, R)
    w = mk([(1, True), (2, True)], [(1, 1, 'o', 0), (2, 2, 'o', 0), (1, 3, 'o', 0)], [])
    run(w, [R(w, 1, ['a.x'], S), R(w, 2, ['a'], X), R(w, 3, ['a.q'], S), R(w, 1, ['a.q.w'], X)])
    f = w.procs[1].frames[-1]
    assert f.outcome[0] == 'DeadlockError', outcome_of(w, 1)
    w.drain()
    return 'DeadlockError %r' % (f.outcome[1],)


def scenario_ld9():
    # H holds X a.b; V queues S a; W holds X z and queues X a.c; H requests X z: DeadlockError (H, W, V)
    w = mk([(1, True), (2, True)], [(1, 1, 'o', 0), (2, 2, 'o', 0), (1, 3, 'o', 0)], [])
    run(w, [R(w, 1, ['a.b'], X), R(w, 2, ['a'], S), R(w, 3, ['z'], X), R(w, 3, ['a.c'], X), R(w, 1, ['z'], X)])
    f = w.procs[1].frames[-1]
    assert f.outcome[0] == 'DeadlockError', outcome_of(w, 1)
    w.drain()
    return 'DeadlockError %r' % (f.outcome[1],)


def scenario_pi9():
    # H (10) holds X a.b; V (10) queues X a; W (-10) queues S a.c: H and V boosted to -10
    w = mk([(1, True), (2, True)], [(1, 1, 'o', 10), (2, 2, 'o', 10), (1, 3, 'o', -10)], [])
    run(w, [R(w, 1, ['a.b'], X), R(w, 2, ['a'], X), R(w, 3, ['a.c'], S)])
    assert w.procs[1].inherited == -10 and w.procs[2].inherited == -10, (w.procs[1].inherited, w.procs[2].inherited)
    run(w, [('finish', 1)])
    assert w.procs[2].status == 'run' and w.procs[3].status == 'parked'
    w.drain()
    return 'ok'


def scenario_pi11(mutation=None):
    # P (5) holds X a.b, R (5) holds S a.c; P queues X a; W (-10) queues X a then times out
    w = mk([(1, True), (2, True)], [(1, 1, 'o', 5), (2, 2, 'o', 5), (1, 3, 'o', -10)], [], mutation=mutation)
    run(w, [R(w, 1, ['a.b'], X), R(w, 2, ['a.c'], S), R(w, 1, ['a'], X), R(w, 3, ['a'], X, 'wait_dl')])
    assert w.procs[1].inherited == -10 and w.procs[2].inherited == -10
    run(w, [('deadline', 3)])
    eff = [min(w.procs[i].priority, w.procs[i].inherited) for i in (1, 2)]
    assert eff == [5, 5], (w.procs[1].inherited, w.procs[2].inherited)
    w.drain()
    return 'ok'


def scenario_lx11(mutation=None):
    # a granted wake posted to another home; the waiter's timeout adopts it; it then queues anew;
    # the stale wake must not resume the new request
    w = mk([(1, True), (2, True)], [(1, 1, 'o', 0), (2, 2, 'o', 0), (1, 3, 'o', 0)], [], mutation=mutation)
    run(w, [R(w, 1, ['k'], X), R(w, 2, ['k'], X, 'wait_dl')])
    # p1 releases on home 1: the wake to p2 is posted to home 2's port
    w.do(('finish', 1)); w.check()
    while w.homes[1].pending:
        w.do(('hstep', 1)); w.check()
    assert w.homes[2].port.queue, 'wake should be in flight'
    w.do(('deadline', 2)); w.check()
    while w.homes[2].pending:
        w.do(('hstep', 2)); w.check()
    assert w.procs[2].frames[-1].outcome[0] == 'granted'
    w.do(('enter_step', 2)); w.check()
    w.do(R(w, 3, ['m'], X)); w.check()
    w.do(R(w, 2, ['m'], X)); w.check()       # nested: p2 waits for m behind p3
    while w.homes[2].pending:
        w.do(('hstep', 2)); w.check()
    assert w.procs[2].status == 'parked'
    w.do(('post', 2)); w.check()              # the stale wake
    settle(w)
    assert w.procs[2].status == 'parked', 'stale wake resumed a newer request'
    w.drain()
    return 'ok'


def scenario_lx10():
    # grant to a waiter on another home, then the waiter killed before the wake is applied
    w = mk([(1, True), (2, True)], [(1, 1, 'o', 0), (2, 2, 'o', 0), (1, 3, 'o', 0)], [])
    run(w, [R(w, 1, ['k'], X), R(w, 2, ['k'], X), R(w, 3, ['k'], X)])
    w.do(('finish', 1)); w.check()
    while w.homes[1].pending:
        w.do(('hstep', 1)); w.check()
    w.do(('kill', 2, True)); w.check()
    settle(w)
    assert w.procs[3].status == 'run'
    w.drain()
    return 'ok'


def scenario_lx4():
    # grant, then a pending signal handler that raises before the lock_enter step
    w = mk([(1, True), (2, True)], [(1, 1, 'o', 0), (2, 2, 'o', 0), (1, 3, 'o', 0)], [])
    run(w, [R(w, 1, ['k'], X), R(w, 2, ['k'], X)])
    run(w, [('finish', 1)])
    run(w, [('signal', 2), R(w, 2, ['k'], S)])   # handler: refused ("handler")
    assert len(w.procs[2].frames) == 1
    run(w, [('hraise', 2)])
    assert not w.procs[2].frames and not w.M.holds
    w.drain()
    return 'ok'


def scenario_chain_cap8():
    # p_i holds X k_i and nests X k_(i+1); p10 holds k10. p1 (base -5) reaches p2..p9 (1..8 edges),
    # not p10 (9 edges); both homes priority_rr, processes alternate homes
    n = 10
    procs = [((i % 2) + 1, i, 'o', 3) for i in range(1, n + 1)]
    procs[0] = (2, 1, 'o', -5)
    w = mk([(1, True), (2, True)], procs, [])
    for i in range(n, 0, -1):
        run(w, [R(w, i, ['k%d' % i], X)])
    for i in range(n - 1, 0, -1):
        run(w, [R(w, i, ['k%d' % (i + 1)], X)])
    inh = [w.procs[i].inherited for i in range(1, n + 1)]
    assert inh[1:9] == [-5] * 8 and inh[9] == 3, inh
    run(w, [('kill', 1, True)])
    inh = [w.procs[i].inherited for i in range(1, n + 1)]
    assert inh[1] == INF and inh[2:] == [3] * 8, inh
    w.drain()
    return 'ok %r' % inh


def scenario_coalesce(mutation=None):
    # P1 (5) holds X a on H1; P2 (-1) on H2 queues X a: P1 boosted by a posted relevel; P2 times
    # out: P1's boost must go, through a second relevel that coalescing must not swallow
    w = mk([(1, True), (2, True)], [(1, 1, 'o', 5), (2, 2, 'o', -1)], [], mutation=mutation, coalesce=True)
    run(w, [R(w, 1, ['a'], X), R(w, 2, ['a'], X, 'wait_dl')])
    assert w.procs[1].inherited == -1, w.procs[1].inherited
    run(w, [('deadline', 2)])
    assert w.procs[1].inherited == INF, w.procs[1].inherited
    w.drain()
    return 'ok'


def scenario_impl_note(mutation=None):
    # Q holds X a; P (10) queues X a; W (-5) queues X a then times out: P's inh changes, and P is
    # reachable from the seed W only in the graph before the cancel
    w = mk([(1, True), (2, True)], [(1, 1, 'o', 0), (2, 2, 'o', 10), (1, 3, 'o', -5)], [], mutation=mutation)
    run(w, [R(w, 1, ['a'], X), R(w, 2, ['a'], X), R(w, 3, ['a'], X, 'wait_dl')])
    assert w.procs[2].inherited == -5
    run(w, [('deadline', 3)])
    assert w.procs[2].inherited == INF
    w.drain()
    return 'ok'


def scenario_splitpost(mutation=None):
    # rev 4 (2.3, 5.7): H2 captures a relevel for H1 under M.mu, H1 is destroyed (port closed under
    # port.mu, then detach_home), then H2's post runs: it sees closed and drops (no port-race)
    w = mk([(1, True), (2, True)], [(1, 1, 'o', 0), (2, 2, 'o', -1)], [], mutation=mutation, split_post=True)
    run(w, [R(w, 1, ['a'], X)])
    w.do(R(w, 2, ['a'], X)); w.check()
    w.do(('hstep', 2)); w.check()             # block: N = [relevel p1 -> H1]
    assert w.homes[2].pending and w.homes[2].pending[0][0] == 'dlv', w.homes[2].pending
    w.do(('hstep', 2)); w.check()             # dlv: the post is now in flight (dlv_append)
    assert w.homes[2].pending[0][0] == 'dlv_append', w.homes[2].pending
    w.do(('destroy', 1)); w.check()           # ~lock_home: close the port under port.mu
    while w.homes[2].pending:
        w.do(('hstep', 2)); w.check()         # the in-flight post sees closed: dropped
    settle(w)
    assert w.procs[2].status == 'run', outcome_of(w, 2)   # granted when H1's holder ended
    w.drain()
    return 'ok'


def scenario_cache_detach(mutation=None):
    # rev 4 (2.4, 4.11): a home's key-cache references keep its nodes while it is attached, and
    # detach_home drops them, so the trie is empty afterwards (I9)
    w = mk([(1, True), (2, True)], [(1, 1, 'o', 0), (2, 2, 'o', 0)], [], mutation=mutation, cache=2)
    run(w, [R(w, 1, ['a.b'], X), ('finish', 1), R(w, 2, ['b'], S), ('finish', 2)])
    assert ('a', 'b') in w.M.trie and w.M.cache_refs.get(1) == {('a', 'b'): 1}, w.M.cache_refs
    run(w, [('destroy', 1)])
    assert 1 not in w.M.cache_refs and ('a', 'b') not in w.M.trie and ('a',) not in w.M.trie,         (w.M.cache_refs, sorted(w.M.trie))
    assert ('b',) in w.M.trie                   # H2 still caches b
    w.drain()
    return 'ok'


def scenario_cycle_other_home_pid():
    # rev 4 (4.6): a cycle across two homes, one owner: the other home's pid shows as nil
    w = mk([(1, True), (2, True)], [(1, 1, 'o', 0), (2, 2, 'o', 0), (1, 3, 'o', 0)], [])
    run(w, [R(w, 1, ['a'], X), R(w, 3, ['c'], X), R(w, 2, ['b'], X), R(w, 1, ['b'], X), R(w, 3, ['a'], X),
            R(w, 2, ['c'], X)])
    f = w.procs[2].frames[-1]
    assert f.outcome[0] == 'DeadlockError', outcome_of(w, 2)
    pids = [c[0] for c in f.outcome[1]]
    assert pids[0] == 2 and None in pids, pids  # H1's p1 and p3 are nil for H2's p2
    w.drain()
    return 'DeadlockError pids %r' % (pids,)


def scenario_cache_clear(mutation=None):
    # rev 5 (4.4, 4.15, I9): attach makes an empty entry; a host's clear (unref_nodes) empties it
    # and frees the nodes, and the entry stays until detach_home deletes it
    w = mk([(1, True), (2, True)], [(1, 1, 'o', 0), (2, 2, 'o', 0)], [], mutation=mutation, cache=2,
           cache_clear=True)
    assert w.M.cache_refs == {1: {}, 2: {}}, w.M.cache_refs
    run(w, [R(w, 1, ['a.b'], X), ('finish', 1), R(w, 1, ['b'], S), ('finish', 1)])
    assert w.M.cache_refs[1] == {('a', 'b'): 1, ('b',): 1}, w.M.cache_refs
    run(w, [('cclear', 1)])
    assert w.M.cache_refs == {1: {}, 2: {}} and sorted(w.M.trie) == [()], (w.M.cache_refs, sorted(w.M.trie))
    run(w, [R(w, 1, ['a'], X), ('finish', 1)])         # the entry takes references again
    assert w.M.cache_refs[1] == {('a',): 1}, w.M.cache_refs
    run(w, [('destroy', 1)])
    assert w.M.cache_refs == {2: {}}, w.M.cache_refs
    w.drain()
    return 'ok'


def scenario_ref_nodes():
    # rev 5 (4.15): ref_nodes requires an attached home; unref_nodes requires a reference
    w = mk([(1, True)], [(1, 1, 'o', 0)], [], cache=2)
    w.M.ref_nodes(1, [K('a.b')]); w.M.check()
    assert w.M.cache_refs[1] == {('a', 'b'): 1} and w.M.trie[('a', 'b')].refs == 1
    w.M.unref_nodes(1, [K('a.b')]); w.M.check()
    assert w.M.cache_refs[1] == {} and sorted(w.M.trie) == [()]
    for fn, args in ((w.M.ref_nodes, (2, [K('a')])), (w.M.unref_nodes, (1, [K('a')]))):
        try:
            fn(*args)
        except Violation as v:
            assert v.name == 'require', v
        else:
            raise AssertionError('%s%r accepted' % (fn.__name__, args))
    return 'ok'


def scenario_sinks_progress():
    # rev 5 (4.13): blocking_sinks returns (Pid, time) pairs, the time loaded from the sink's
    # home's progress cell, which the home shares with the manager
    w = mk([(1, True), (2, True)], [(1, 1, 'o', 0), (2, 2, 'o', 0), (1, 3, 'o', 0)], [])
    run(w, [R(w, 1, ['a'], X), R(w, 3, ['b'], X), ('finish', 3), R(w, 2, ['a'], X)])
    s = w.M.blocking_sinks((2, 2))
    assert s == {((1, 1), w.homes[1].progress.load())} and w.homes[1].progress.load() == 3, s
    run(w, [R(w, 3, ['c'], S)])               # H1 makes progress: the cell, not a copy
    assert w.M.blocking_sinks((2, 2)) == {((1, 1), 4)}, w.M.blocking_sinks((2, 2))
    w.drain()
    return 'ok %r' % (sorted(s),)


def scenario_query_visibility(mutation=None):
    # rev 5 (4.13): lock-query counts every hold, shows pids only for the viewer's home and owner;
    # one process may hold two X holds with the same key (X a.b, then a nested X {a.b, b}); rev 5b:
    # its generation is the latest X grant's (the world checks it at quiescence)
    w = mk([(1, True), (2, True)], [(1, 1, 'o', 0), (2, 2, 'o', 0), (1, 3, 'p', 0)], [], mutation=mutation)
    run(w, [R(w, 1, ['a.b'], X), R(w, 1, ['a.b', 'b'], X), R(w, 2, ['a'], X), R(w, 3, ['b'], S)])
    q1 = w.M.query((), (1, 'o'))
    q2 = w.M.query((), (2, 'o'))
    q3 = w.M.query((), (1, 'p'))
    ab = [e for e in q1['held'] if e['key'] == ('a', 'b')][0]
    assert ab['count'] == 2 and ab['pids'] == [1, 1] and ab['generation'] == 2, ab
    assert [e['pids'] for e in q2['held']] == [[], []] and q2['waiting'][0]['pid'] == 2, q2
    assert q3['waiting'][1]['pid'] == 3 and q3['waiting'][0]['pid'] is None, q3
    w.drain()
    return 'ok: two X holds of p1 with a.b (generations 1 and 2)'


def scenario_l1_one_home():
    # rev 5 (protocol 7, L1): one home, prio false: no inheritance, every inh INF, no posts
    w = mk([(1, False)], [(1, 1, 'o', 5), (1, 2, 'o', -5), (1, 3, 'o', 0)], [], cache=3)
    run(w, [R(w, 1, ['a'], X), R(w, 2, ['a'], X), R(w, 3, ['a.b'], S)])
    assert all(r.inh == 2 ** 31 - 1 for r in w.M.recs.values()) and w.procs[1].inherited == 2 ** 31 - 1
    run(w, [('finish', 1)])
    assert w.procs[2].status == 'run' and w.procs[3].status == 'parked'
    w.drain()
    return 'ok'


def scenario_l2_one_home():
    # rev 5 (protocol 7, L2): one home under priority_rr: a waiter boosts the holder, applied at
    # once on the same home (4.14), and a set-priority re-levels it
    w = mk([(1, True)], [(1, 1, 'o', 5), (1, 2, 'o', -5)], [], cache=1)
    run(w, [R(w, 1, ['a'], X), R(w, 2, ['a'], X)])
    assert w.procs[1].inherited == -5 and not w.homes[1].port.queue, w.procs[1].inherited
    run(w, [('prio', 2, 2)])
    assert w.procs[1].inherited == 2, w.procs[1].inherited
    w.drain()
    return 'ok'


def scenario_one_home_refusal():
    # rev 5 (4.4): before L3 a manager accepts one home; attaching a second is refused
    w = mk([(1, False)], [(1, 1, 'o', 0), (2, 2, 'o', 0)], [], late_homes=[(2, False)], one_home=True)
    run(w, [R(w, 1, ['a'], X)])
    try:
        w.do(('attach', 2))
    except Violation as v:
        assert v.name == 'require' and 'one home' in v.detail, v
        return 'refused: %s' % v.detail
    raise AssertionError('a second home was attached to a one-home manager')


def scenario_set_base(mutation=None):
    # I8's base clause: a holder's set-priority reaches its rec (5.8)
    w = mk([(1, True), (2, True)], [(1, 1, 'o', 5), (2, 2, 'o', 0)], [], mutation=mutation)
    run(w, [R(w, 1, ['a'], X), R(w, 2, ['a'], X), ('prio', 1, -2)])
    assert w.M.recs[(1, 1)].base == -2
    w.drain()
    return 'ok'


def scenario_group_teardown():
    # rev 5b (5.7, L4; design 9.2, td 4.2): a group's teardown closes its port first, with no pump
    # running, then kills its processes. A grant made between the close and the kills has its wake
    # dropped; the waiter's kill releases it, and the next waiter, on another home, is granted.
    w = mk([(1, True), (2, True)], [(1, 1, 'o', 0), (2, 2, 'o', 0), (2, 3, 'o', 0), (1, 4, 'o', 0)], [],
           group_teardown=True)
    run(w, [R(w, 1, ['k'], X), R(w, 2, ['k'], X), R(w, 3, ['k'], S), R(w, 4, ['k'], S)])
    assert [w.procs[i].status for i in (2, 3, 4)] == ['parked'] * 3, [outcome_of(w, i) for i in (2, 3, 4)]
    w.do(('destroy', 2)); w.check()                 # H2's port closed; its kills and detach pending
    assert w.homes[2].port.closed and w.homes[2].pending[0] == ('gkill', 2), w.homes[2].pending
    dropped = STATS['post_dropped_closed_port']
    w.do(('finish', 1)); w.check()                  # H1 releases k: p2 (H2) granted
    while w.homes[1].pending:                       # H1 delivers p2's wake: dropped at the closed port
        w.do(('hstep', 1)); w.check()
    t2 = w.M.tickets[w.procs[2].lock_ticket]
    assert t2.state == 'granted' and w.procs[2].status == 'parked', (t2.state, outcome_of(w, 2))
    assert STATS['post_dropped_closed_port'] == dropped + 1
    while w.homes[2].pending:                       # the kills, then the detach
        w.do(('hstep', 2)); w.check()
    assert w.homes[2].destroyed and 2 not in w.M.homes and not any(P[0] == 2 for P in w.M.recs)
    settle(w)                                       # H1 applies p4's wake
    p4 = w.procs[4]
    assert p4.status == 'run' and len(p4.locks) == 1 and p4.locks[0].mode == S, outcome_of(w, 4)
    w.drain()
    return 'ok'


def main():
    res = {}
    bad = 0
    for name, fn in sorted(globals().items()):
        if name.startswith('scenario_'):
            try:
                res[name] = fn()
            except Violation as v:
                res[name] = 'VIOLATION %s' % v
            except AssertionError as e:
                res[name] = 'ASSERT %s' % e
            except Exception as e:          # rev 5b: a crash is reported, not fatal to the run
                res[name] = 'CRASH %s: %s' % (type(e).__name__, e)
            if str(res[name]).startswith(('VIOLATION', 'ASSERT', 'CRASH')):
                bad += 1
    # mutations of section 7, each with the violation that must catch it (rev 5b): any other
    # outcome, an assertion or a different violation included, is unexpected
    muts = [
        ('no-exempt (LK-27 nested request raises DeadlockError)', scenario_lk27_deadlock_when_no_exempt, None,
         'DeadlockError'),
        ('region-holds-only (LK-25 reader left queued, I6)', scenario_lk25, {'region-holds-only'}, 'I6'),
        ('no-relevel-cancel (PI-11 boost kept, I8)', scenario_pi11, {'no-relevel-cancel'}, 'I8'),
        ('wake-no-tid-check (LX-11 stale wake resumes newer request)', scenario_lx11, {'wake-no-tid-check'}, 'I2'),
        ('coalesce-no-clear (relevel flag never cleared: boost kept, I8)', scenario_coalesce, {'coalesce-no-clear'},
         'I8'),
        ('impl-note-after-only (A without the graph before: impl-note)', scenario_impl_note, {'impl-note-after-only'},
         'impl-note'),
        ('post-test-outside-mu (2.3: closed tested outside the port mutex: port-race)', scenario_splitpost,
         {'post-test-outside-mu'}, 'port-race'),
        ('no-unref-on-detach (4.11: cache references of a detached home kept: I9)', scenario_cache_detach,
         {'no-unref-on-detach'}, 'I9'),
        ('unref-pops-entry (4.15: unref deletes an emptied entry, the rev-4 reading: I9)', scenario_cache_clear,
         {'unref-pops-entry'}, 'I9'),
        ('no-set-base (5.8: set-priority skips set_base: I8 base clause)', scenario_set_base, {'no-set-base'}, 'I8'),
        ('cache-miss-unreferenced (4.15: a miss cached without its reference: I4 home side)',
         scenario_cache_detach, {'cache-miss-unreferenced'}, 'I4'),
        ('cancel-marks-failed (4.9: a timed-out ticket marked failed, same outcome: I2 transitions)',
         scenario_lk25, {'cancel-marks-failed'}, 'I2'),
        ('query-first-gen (4.13: the first X grant\'s generation shown, not the latest)',
         scenario_query_visibility, {'query-first-gen'}, '4.13'),
    ]
    for label, fn, m, want in muts:
        try:
            r = fn(m) if m is not None else fn()
            got = 'result %s' % (r,)
            ok = want == 'DeadlockError' and str(r).startswith('DeadlockError as expected')
        except Violation as v:
            got = 'violation %s' % v
            ok = v.name == want
        except AssertionError as e:
            got = 'assert %s' % e
            ok = False
        except Exception as e:
            got = 'crash %s: %s' % (type(e).__name__, e)
            ok = False
        res['mutation ' + label] = ('caught as expected (%s): %s' % (want, got) if ok
                                    else 'NOT AS EXPECTED (wanted %s): %s' % (want, got))
        bad += not ok
    for k in sorted(res):
        print('%-70s %s' % (k, res[k]))
    print('scenarios.py: %d scenarios, %d mutations, %d unexpected'
          % (sum(1 for k in res if k.startswith('scenario_')), len(muts), bad))
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
