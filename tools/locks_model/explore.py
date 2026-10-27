"""Bounded exhaustive exploration (0-1 BFS: external events cost 1, home steps and posts cost 0).

Rev 5b: 'gdestroy', the 'destroy' configuration with a group's teardown (5.7, L4): the close,
then a kill per process, then the detach, other homes acting and posting in between.

Rev 5: a configuration whose cache is a tuple starts with a zero-cost action choosing the key-cache
capacity, so one run covers each capacity (the one-home 'l1' and 'l2' configurations of
protocol 7, caches of 0-3 entries).

Every reachable state within the event bound is visited once (deduplicated on its full state)
and checked; states at the bound also run a fair drain (liveness, I9). The shortest trace (in
events) found for every violation name is kept.

Rev 5 (memory): a state at the bound has only zero-cost steps left, so it is expanded depth-first
as soon as it is found instead of being queued; the queue then holds only the levels below the
bound (about a tenth of the states). A state found at the bound and later at a lower cost is
expanded again at the lower cost, so the set of states visited is unchanged.

usage: python -I explore.py CONFIG BOUND [max_states]
"""

import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))

import collections
import json
import sys
import time

from lockmodel import K, S, X, Violation
from world import Config, World

A, AB, AC, B = K('a'), K('a.b'), K('a.c'), K('b')

ALL_REQ = [((A,), S), ((A,), X), ((AB,), S), ((AB,), X), ((AC,), S), ((AC,), X), ((B,), S), ((B,), X),
           ((AB, AC), S), ((AB, AC), X), ((AB, B), X), ((AC, B), S), ((A, B), X)]

CONFIGS = {
    # multi-key and nesting, one priority and one round-robin home
    'core': dict(homes=[(1, True), (2, False)], procs=[(1, 1, 'o1', 0), (1, 2, 'o1', 1), (2, 3, 'o2', -1)],
                 requests=[((A,), X), ((AB,), S), ((AB,), X), ((AC,), X), ((B,), X), ((AB, B), X), ((A,), S)],
                 kinds=('wait_dl',), kills=(True,), req_per_proc=2),
    # the full request set, fewer requests per process
    'wide': dict(homes=[(1, True), (2, True)], procs=[(1, 1, 'o1', 0), (2, 2, 'o1', 1), (2, 3, 'o2', -1)],
                 requests=ALL_REQ, kinds=('wait_dl',), kills=(), req_per_proc=1, max_depth=1),
    # signals and handlers, timeouts, :if-available, terminate
    'handlers': dict(homes=[(1, True), (2, True)], procs=[(1, 1, 'o1', 0), (2, 2, 'o1', 1), (1, 3, 'o2', -1)],
                     requests=[((A,), X), ((AB,), S), ((AC,), X), ((B,), X)],
                     kinds=('wait_dl', 'ifavail'), kills=(True, False), signals=1, req_per_proc=2,
                     max_hdepth=1),
    # priorities and inheritance under priority_rr on both homes
    'priority': dict(homes=[(1, True), (2, True)], procs=[(1, 1, 'o1', 0), (2, 2, 'o1', 2), (1, 3, 'o2', -2)],
                     requests=[((A,), X), ((AB,), X), ((AC,), S), ((B,), X), ((AB, B), X)],
                     kinds=('wait_dl',), kills=(True,), prio_values=(-2, -1, 0, 1, 2), prio_changes=1,
                     req_per_proc=2),
    # a scheduler destroyed with holders and waiters; a priority and a round-robin home
    'destroy': dict(homes=[(1, True), (2, False)], procs=[(1, 1, 'o1', 0), (2, 2, 'o1', 1), (1, 3, 'o2', -2)],
                    requests=[((A,), X), ((AB,), S), ((AC,), X), ((B,), X)],
                    kinds=('wait_dl',), kills=(True,), destroy=True, req_per_proc=2, cache=1),
    # rev 5b (5.7, L4): 'destroy' with a group's teardown: the port closed, then a kill per
    # process, then the detach, other homes acting and posting in between
    'gdestroy': dict(homes=[(1, True), (2, False)], procs=[(1, 1, 'o1', 0), (2, 2, 'o1', 1), (1, 3, 'o2', -2)],
                     requests=[((A,), X), ((AB,), S), ((AC,), X), ((B,), X)],
                     kinds=('wait_dl',), kills=(True,), destroy=True, req_per_proc=2, cache=1,
                     group_teardown=True),
    # priorities with relevel coalescing (4.3)
    'coalesce': dict(homes=[(1, True), (2, True)], procs=[(1, 1, 'o1', 0), (2, 2, 'o1', 2), (1, 3, 'o2', -2)],
                     requests=[((A,), X), ((AB,), X), ((AC,), S), ((B,), X), ((AB, B), X)],
                     kinds=('wait_dl',), kills=(True,), prio_values=(-2, 0, 2), prio_changes=1, req_per_proc=2,
                     coalesce=True, destroy=True),
    # rev 3: a priority home attached after a round-robin home's processes lock (lending false ->
    # true), then destroyed (true -> false)
    'attach': dict(homes=[(1, False)], late_homes=[(2, True)],
                   procs=[(1, 1, 'o1', 0), (2, 2, 'o1', -2), (1, 3, 'o2', 1)],
                   requests=[((A,), X), ((AB,), S), ((AC,), X), ((B,), X), ((AB, B), X)],
                   kinds=('wait_dl',), kills=(True,), destroy=True, req_per_proc=2, cache=1),
    # rev 3: M.options.inheritance false, priorities changing
    'noinh': dict(homes=[(1, True), (2, True)], procs=[(1, 1, 'o1', 0), (2, 2, 'o1', 2), (1, 3, 'o2', -2)],
                  requests=[((A,), X), ((AB,), X), ((AC,), S), ((B,), X), ((AB, B), X)],
                  kinds=('wait_dl',), kills=(True,), prio_values=(-2, 0, 2), prio_changes=1, req_per_proc=2,
                  inheritance=False, destroy=True),
    # rev 3: owner checks (I12): release/cancel naming another process's hold or ticket
    'foreign': dict(homes=[(1, True), (2, False)], procs=[(1, 1, 'o1', 0), (1, 2, 'o1', 1), (2, 3, 'o2', -1)],
                    requests=[((A,), X), ((AB,), S), ((AC,), X), ((B,), X), ((AB, B), X)],
                    kinds=('wait_dl',), kills=(True,), req_per_proc=2, foreign=True),
    # rev 4: the post as a separate action of the posting thread after the operation (4.14), one
    # critical section under the port's mutex (2.3); ~lock_home closes the port before
    # detach_home (5.7). Expected to pass; 'splitpost_mut' tests the closed outside the mutex
    # (the rev-3 reading) and must report port-race
    'splitpost': dict(homes=[(1, True), (2, True)], procs=[(1, 1, 'o1', 0), (2, 2, 'o1', 1)],
                      requests=[((A,), X)], kinds=('wait',), kills=(), destroy=True, req_per_proc=1,
                      max_depth=1, split_post=True),
    'splitpost_mut': dict(homes=[(1, True), (2, True)], procs=[(1, 1, 'o1', 0), (2, 2, 'o1', 1)],
                          requests=[((A,), X)], kinds=('wait',), kills=(), destroy=True, req_per_proc=1,
                          max_depth=1, split_post=True, mutation={'post-test-outside-mu'}),
    # rev 4: the split post with more requests, kills, a timeout and a key cache
    'splitpost2': dict(homes=[(1, True), (2, False)], procs=[(1, 1, 'o1', 0), (2, 2, 'o1', 1), (2, 3, 'o2', -1)],
                       requests=[((A,), X), ((AB,), S), ((B,), X)], kinds=('wait_dl',), kills=(True,),
                       destroy=True, req_per_proc=2, split_post=True, cache=1),
    # rev 4: key-cache references (2.4, 4.15) under every request shape, kills and destruction
    'cache': dict(homes=[(1, True), (2, False)], procs=[(1, 1, 'o1', 0), (1, 2, 'o1', 1), (2, 3, 'o2', -1)],
                  requests=[((A,), X), ((AB,), S), ((AC,), X), ((B,), X), ((AB, B), X), ((AB, AC), S)],
                  kinds=('wait_dl', 'ifavail'), kills=(True,), destroy=True, req_per_proc=2, cache=2),
    # rev 5 (protocol 7): the phases before L3, one home of three processes, keys a, a.b, a.c, b
    # in one- and two-key requests (the same in both), nesting depth 2, key caches of 0-3 entries
    # (a zero-cost first action picks one). L1: prio false, no set-priority, :if-available,
    # terminate, a host's cache clear (unref_nodes); L2: priority_rr, set-priority over -2..2
    'l1': dict(homes=[(1, False)], procs=[(1, 1, 'o1', 0), (1, 2, 'o1', 1), (1, 3, 'o2', -1)],
               requests=[((A,), X), ((AB,), S), ((AC,), X), ((AB, B), X)],
               kinds=('wait_dl', 'ifavail'), kills=(True, False), destroy=True, req_per_proc=2,
               cache=(0, 1, 2, 3), cache_clear=True, one_home=True),
    'l2': dict(homes=[(1, True)], procs=[(1, 1, 'o1', 0), (1, 2, 'o1', 2), (1, 3, 'o2', -2)],
               requests=[((A,), X), ((AB,), S), ((AC,), X), ((AB, B), X)],
               kinds=('wait_dl',), kills=(True,), prio_values=(-2, -1, 0, 1, 2), prio_changes=1,
               destroy=True, req_per_proc=2, cache=(0, 1, 2, 3), one_home=True),
    # rev 5 (protocol 7): two homes of two processes each, a cache of 3 entries
    'two2': dict(homes=[(1, True), (2, False)],
                 procs=[(1, 1, 'o1', 0), (1, 2, 'o2', 1), (2, 3, 'o1', -1), (2, 4, 'o2', 2)],
                 requests=[((A,), X), ((AB,), S), ((AC,), X), ((B,), X), ((AB, B), X)],
                 kinds=('wait_dl',), kills=(True,), destroy=True, req_per_proc=1, cache=3),
    # rev 5: 4.3's seeds without the granted processes, an equivalent mutation (a granted process's
    # edges out went only to the operation's other seeds, so it is already in A): expected to pass
    'grantseeds': dict(homes=[(1, True), (2, True)], procs=[(1, 1, 'o1', 0), (2, 2, 'o1', 2), (1, 3, 'o2', -2)],
                       requests=[((A,), X), ((AB,), X), ((AC,), S), ((B,), X), ((AB, B), X)],
                       kinds=('wait_dl',), kills=(True,), prio_values=(-2, 0, 2), prio_changes=1,
                       req_per_proc=2, mutation={'no-granted-seeds'}),
    # three homes, one process each
    'three': dict(homes=[(1, True), (2, True), (3, False)], procs=[(1, 1, 'o1', 0), (2, 2, 'o1', 1), (3, 3, 'o2', -2)],
                  requests=[((A,), X), ((AB,), S), ((AB,), X), ((AC,), X), ((B,), X), ((AB, B), X)],
                  kinds=('wait_dl',), kills=(True,), destroy=True, req_per_proc=2, cache=1),
}


def make(name):
    c = dict(CONFIGS[name])
    return Config(c.pop('homes'), c.pop('procs'), c.pop('requests'), **c)


def replay(cfg, actions):
    w = World(cfg)
    lines = []
    for a in actions:
        lines.append(w.describe(a))
        w.do(a)
    return lines


def explore(name, bound, max_states=10 ** 8, drain_at_bound=True, log=None):
    cfg = make(name)
    w0 = World(cfg)
    w0.check()
    k0 = hash(w0.key())
    parent = {k0: (None, None, 0)}
    dq = collections.deque([(w0, 0)])
    viol = {}
    soft = {}
    states = 0
    transitions = 0
    t0 = time.time()
    deep = []           # rev 5: states at the bound, expanded depth-first at once

    def note(table, name, detail, trace, cost):
        old = table.get(name)
        if old is None or cost < old[2]:
            table[name] = (detail, trace, cost)

    def trace_of(h):
        acts = []
        while True:
            ph, a, _ = parent[h]
            if ph is None:
                break
            acts.append(a)
            h = ph
        acts.reverse()
        return acts

    while dq or deep:
        w, cost = deep.pop() if deep else dq.popleft()
        states += 1
        if states > max_states:
            break
        hk = hash(w.key())
        if parent[hk][2] < cost:
            continue
        acts = w.enabled()
        if drain_at_bound and cost == bound:
            d = w.clone()
            try:
                d.drain()
            except Violation as v:
                note(viol, v.name + '(drain)', str(v), trace_of(hk) + [('<drain>',)], cost)
        for a in acts:
            ext = a[0] not in ('hstep', 'post', 'cachecap')
            nc = cost + (1 if ext else 0)
            if nc > bound:
                continue
            w2 = w.clone()
            transitions += 1
            try:
                w2.do(a)
                w2.check()
            except Violation as v:
                note(viol, v.name, str(v), trace_of(hk) + [a], nc)
                continue
            except Exception as e:  # a crash in the model is a finding too
                note(viol, 'crash:' + type(e).__name__, repr(e), trace_of(hk) + [a], nc)
                continue
            for sn, sd in w2.soft:
                note(soft, sn, sd, trace_of(hk) + [a], nc)
            h2 = hash(w2.key())
            prev = parent.get(h2)
            if prev is not None and prev[2] <= nc:
                continue
            parent[h2] = (hk, a, nc)
            if nc == bound:
                deep.append((w2, nc))
            elif ext:
                dq.append((w2, nc))
            else:
                dq.appendleft((w2, nc))
        if log and states % 50000 == 0:
            log('%s: %d states, %d transitions, %.0fs' % (name, states, transitions, time.time() - t0))
    res = {'config': name, 'bound': bound, 'states': len(parent), 'expanded': states,
           'transitions': transitions, 'seconds': round(time.time() - t0, 1),
           'complete': not dq and not deep, 'violations': {}, 'soft': {}}
    for n, (d, tr, _) in viol.items():
        res['violations'][n] = {'detail': d, 'trace': render(cfg, tr)}
    for n, (d, tr, _) in soft.items():
        res['soft'][n] = {'detail': d, 'trace': render(cfg, tr)}
    return res


def render(cfg, tr):
    w = World(cfg)
    out = []
    for a in tr:
        if a[0] == '<drain>':
            out.append('<fair drain: finish every body, apply every post>')
            break
        out.append(w.describe(a))
        try:
            w.do(a)
        except Exception as e:
            out.append('  -> %s' % e)
            break
    return out


if __name__ == '__main__':
    name = sys.argv[1]
    bound = int(sys.argv[2])
    ms = int(sys.argv[3]) if len(sys.argv) > 3 else 10 ** 8
    r = explore(name, bound, ms, log=lambda s: print(s, file=sys.stderr, flush=True))
    print(json.dumps(r, indent=1))
