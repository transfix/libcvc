"""Exhaustive exploration of every interleaving of scripted processes, with a fault budget.

Each process runs a fixed script of nested requests; nondeterminism is the interleaving of the
homes (threads), the delivery of posts, and up to `faults` faults anywhere: kill (abandon),
terminate, an injected raise, a signal whose handler runs handler_script, a deadline firing, a
priority change, a home destroyed. The state space of one combination is explored completely.

usage: python -I explore_scripts.py MODE [N] [SEED] [WORKERS]
"""

import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))

import collections
import itertools
import json
import multiprocessing
import random
import sys
import time
import traceback

from lockmodel import K, S, X, Violation, req_str, STATS
from world import Config, World

A, AB, AC, B = K('a'), K('a.b'), K('a.c'), K('b')
ALPHA = [((A,), S), ((A,), X), ((AB,), S), ((AB,), X), ((AC,), S), ((AC,), X), ((B,), S), ((B,), X),
         ((AB, AC), S), ((AB, AC), X), ((AB, B), X), ((AC, B), S), ((A, B), X)]

LAYOUTS = {
    # one priority_rr home with two processes, one round_robin home
    'pr+rr': ([(1, True), (2, False)], [(1, 1, 'o1', 0), (1, 2, 'o1', 1), (2, 3, 'o2', -1)]),
    # two priority_rr homes
    'pr+pr': ([(1, True), (2, True)], [(1, 1, 'o1', 1), (2, 2, 'o1', 0), (2, 3, 'o2', -1)]),
    # three homes
    'three': ([(1, True), (2, True), (3, False)], [(1, 1, 'o1', 1), (2, 2, 'o1', 0), (3, 3, 'o2', -1)]),
    # rev 3: a round-robin home, and a priority_rr home attached while the first's processes lock
    'rr+late': ([(1, False)], [(1, 1, 'o1', 1), (2, 2, 'o1', -1), (1, 3, 'o2', 0)], [(2, True)]),
    # rev 5 (protocol 7): one home of three processes; L1 has no set-priority
    'l1': ([(1, False)], [(1, 1, 'o1', 1), (1, 2, 'o1', 0), (1, 3, 'o2', -1)]),
    'l2': ([(1, True)], [(1, 1, 'o1', 1), (1, 2, 'o1', 0), (1, 3, 'o2', -1)]),
}

# world.Config's default fault kinds, plus a home destroyed: the 'nested' mode's destroy
# combinations (a scripted run destroys a home only when 'destroy' is among its fault kinds)
NESTED_FAULTS = ('kill', 'raise', 'signal', 'deadline', 'prio', 'terminate', 'destroy')


def explore_cfg(cfg, max_states=400000, drain=True):
    w0 = World(cfg)
    w0.check()
    k0 = hash(w0.key())
    parent = {k0: (None, None)}
    dq = collections.deque([(w0, k0)])
    viol = {}
    soft = {}
    trans = 0

    def trace_of(h):
        acts = []
        while True:
            ph, a = parent[h]
            if ph is None:
                break
            acts.append(a)
            h = ph
        acts.reverse()
        return acts

    while dq:
        w, hk = dq.popleft()
        if len(parent) > max_states:
            return {'states': len(parent), 'trans': trans, 'viol': viol, 'soft': soft, 'complete': False}
        acts = w.enabled()
        # where the scripts can make no further progress on their own, drain fairly (liveness, I9)
        if drain and not any(a[0] in ('hstep', 'post', 'req', 'enter_step') for a in acts):
            d = w.clone()
            try:
                d.drain()
            except Violation as v:
                n = v.name + '(drain)'
                if n not in viol:
                    viol[n] = (str(v), trace_of(hk) + [('<drain>',)])
        for a in acts:
            w2 = w.clone()
            trans += 1
            try:
                w2.do(a)
                w2.check()
            except Violation as v:
                if v.name not in viol:
                    viol[v.name] = (str(v), trace_of(hk) + [a])
                continue
            except Exception as e:
                n = 'crash:' + type(e).__name__
                if n not in viol:
                    viol[n] = (repr(e) + ' ' + traceback.format_exc()[-800:], trace_of(hk) + [a])
                continue
            for sn, sd in w2.soft:
                if sn not in soft:
                    soft[sn] = (sd, trace_of(hk) + [a])
            h2 = hash(w2.key())
            if h2 in parent:
                continue
            parent[h2] = (hk, a)
            dq.append((w2, h2))
    return {'states': len(parent), 'trans': trans, 'viol': viol, 'soft': soft, 'complete': True}


def render(cfg, tr):
    w = World(cfg)
    out = []
    for a in tr:
        if a[0] == '<drain>':
            out.append('<fair drain: no new requests; finish every body; apply every post>')
            break
        out.append(w.describe(a))
        try:
            w.do(a)
        except Exception as e:
            out.append('  -> %s' % e)
            break
    return out


def make_cfg(layout, scripts, hscript, faults, destroy, fault_kinds=None, coalesce=False, cache=0,
             group_teardown=False):
    homes, procs = LAYOUTS[layout][:2]
    kw = {}
    if group_teardown:
        kw['group_teardown'] = True       # rev 5b: destroy is a group's teardown (5.7, L4)
    if len(LAYOUTS[layout]) > 2:
        kw['late_homes'] = LAYOUTS[layout][2]
    if fault_kinds:
        kw['fault_kinds'] = fault_kinds
    return Config(homes, procs, list(ALPHA), max_depth=2, max_hdepth=1,
                  prio_values=() if layout == 'l1' else (-2, 2), one_home=layout in ('l1', 'l2'),
                  signals=1, destroy=destroy, scripts=scripts, handler_script=hscript, faults=faults,
                  coalesce=coalesce, cache=cache, **kw)


def job(spec):
    layout, scripts, hscript, faults, destroy = spec[:5]
    fk = spec[5] if len(spec) > 5 else None
    co = spec[6] if len(spec) > 6 else False
    ca = spec[7] if len(spec) > 7 else 0
    gt = spec[8] if len(spec) > 8 else False
    cfg = make_cfg(layout, scripts, hscript, faults, destroy, fk, co, ca, gt)
    STATS.clear()
    t0 = time.time()
    r = explore_cfg(cfg)
    out = {'spec': spec, 'states': r['states'], 'trans': r['trans'], 'complete': r['complete'],
           'secs': round(time.time() - t0, 2), 'viol': {}, 'soft': {}, 'stats': dict(STATS)}
    for n, (d, tr) in r['viol'].items():
        out['viol'][n] = {'detail': d, 'trace': render(cfg, tr), 'len': len(tr)}
    for n, (d, tr) in r['soft'].items():
        out['soft'][n] = {'detail': d, 'trace': render(cfg, tr), 'len': len(tr)}
    return out


def gen_specs(mode, n, seed):
    rnd = random.Random(seed)
    specs = []
    if mode == 'single':
        # every combination of one request per process, both two-home layouts
        for layout in ('pr+rr', 'pr+pr'):
            for combo in itertools.product(range(len(ALPHA)), repeat=3):
                scripts = dict((i + 1, ((combo[i], 'wait_dl'),)) for i in range(3))
                specs.append((layout, scripts, ((rnd.randrange(len(ALPHA)), 'wait_dl'),), 1, False, None,
                              rnd.random() < 0.5, rnd.choice([0, 1])))
    elif mode == 'nested':
        for _ in range(n):
            layout = rnd.choice(['pr+rr', 'pr+pr', 'three', 'rr+late', 'l1', 'l2'])
            scripts = {}
            for pid in (1, 2, 3):
                ln = rnd.choice([1, 2, 2])
                scripts[pid] = tuple((rnd.randrange(len(ALPHA)),
                                      rnd.choice(['wait_dl', 'wait_dl', 'wait_dl', 'ifavail', 't0', 'wait']))
                                     for _ in range(ln))
            hs = ((rnd.randrange(len(ALPHA)), rnd.choice(['wait_dl', 'ifavail'])),) if rnd.random() < 0.7 else ()
            # a destroy combination takes NESTED_FAULTS: Config's default fault kinds have no
            # 'destroy', so with them no nested run ever destroyed a home. The generator is drawn
            # in the same order as before, so every combination is otherwise unchanged
            destroy = rnd.random() < 0.3
            specs.append((layout, scripts, hs, 1, destroy, NESTED_FAULTS if destroy else None,
                          rnd.random() < 0.5, rnd.choice([0, 1, 2])))
    elif mode == 'pairs':
        # two faults, from a pair of fault kinds, on random nested scripts: the cross-thread races
        pairs = [('deadline', 'kill'), ('deadline', 'signal'), ('signal', 'raise'), ('kill', 'prio'),
                 ('deadline', 'prio'), ('destroy', 'kill'), ('destroy', 'deadline'), ('terminate', 'raise'),
                 ('signal', 'deadline'), ('raise', 'deadline'), ('prio', 'prio'), ('kill', 'kill')]
        for i in range(n):
            layout = rnd.choice(['pr+rr', 'pr+pr', 'three', 'rr+late', 'l1', 'l2'])
            scripts = {}
            for pid in (1, 2, 3):
                ln = rnd.choice([1, 1, 2])
                scripts[pid] = tuple((rnd.randrange(len(ALPHA)), 'wait_dl') for _ in range(ln))
            hs = ((rnd.randrange(len(ALPHA)), 'wait_dl'),)
            fk = pairs[i % len(pairs)]
            specs.append((layout, scripts, hs, 2, 'destroy' in fk, fk, rnd.random() < 0.5,
                          rnd.choice([0, 1, 2])))
            # rev 5b: every other round of the pairs, a destroy is a group's teardown (5.7, L4);
            # chosen without the generator, so every other combination is as in rev 5
            if 'destroy' in fk and layout not in ('l1', 'l2') and (i // len(pairs)) % 2 == 1:
                specs[-1] = specs[-1] + (True,)
    return specs


def main():
    mode = sys.argv[1]
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 1000
    seed = int(sys.argv[3]) if len(sys.argv) > 3 else 1
    workers = int(sys.argv[4]) if len(sys.argv) > 4 else 20
    specs = gen_specs(mode, n, seed)
    t0 = time.time()
    tot_states = tot_trans = incomplete = 0
    viol = {}
    soft = {}
    done = 0
    stats = {}
    with multiprocessing.Pool(workers) as pool:
        for r in pool.imap_unordered(job, specs, chunksize=4):
            done += 1
            tot_states += r['states']
            tot_trans += r['trans']
            if not r['complete']:
                incomplete += 1
            for k, v in r['stats'].items():
                stats[k] = stats.get(k, 0) + v
            for nm, v in r['viol'].items():
                if nm not in viol or v['len'] < viol[nm]['len']:
                    viol[nm] = dict(v, spec=repr(r['spec']))
            for nm, v in r['soft'].items():
                if nm not in soft or v['len'] < soft[nm]['len']:
                    soft[nm] = dict(v, spec=repr(r['spec']))
            if done % 200 == 0:
                print('%d/%d combos, %d states, %d transitions, %.0fs, viol=%s soft=%s'
                      % (done, len(specs), tot_states, tot_trans, time.time() - t0, sorted(viol), sorted(soft)),
                      file=sys.stderr, flush=True)
    print(json.dumps({'mode': mode, 'combos': len(specs), 'states': tot_states, 'transitions': tot_trans,
                      'incomplete': incomplete, 'seconds': round(time.time() - t0),
                      'violations': viol, 'soft': soft, 'coverage': dict(sorted(stats.items()))}, indent=1))


if __name__ == '__main__':
    main()
