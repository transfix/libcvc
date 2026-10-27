"""Long random runs: more processes, homes and keys, priorities under priority_rr, every fault.

Each worker runs episodes from fixed seeds: random enabled actions (weighted), the invariants
checked after every step, then a fair drain (liveness) and the end of every process (I9). A
violating episode is shrunk by greedy deletion of actions.

usage: python -I random_runs.py TOTAL_STEPS [WORKERS] [SEED0]
"""

import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))

import itertools
import json
import multiprocessing
import random
import sys
import time
import traceback

from lockmodel import K, S, X, Violation, canonical, STATS
from world import Config, World

KEYS = [K(k) for k in ('a', 'a.b', 'a.c', 'a.b.d', 'b', 'b.e', 'c')]


def all_requests():
    reqs = set()
    for n in (1, 2, 3):
        for ks in itertools.combinations(KEYS, n):
            c = canonical(ks)
            if len(c) != n:
                continue
            for m in (S, X):
                reqs.add((c, m))
    return sorted(reqs)


REQS = all_requests()

WEIGHTS = {'hstep': 6, 'post': 6, 'req': 4, 'enter_step': 6, 'finish': 3, 'raise': 0.4, 'hraise': 0.4,
           'signal': 0.25, 'hend': 2, 'prio': 0.3, 'kill': 0.06, 'deadline': 0.6, 'destroy': 0.01,
           'attach': 0.05, 'foreign': 0.15, 'cclear': 0.05}


def make_world(rnd, variant, coalesce=False):
    nh = rnd.choice([1, 2, 3, 3])          # rev 5: one-home worlds (protocol 7's L1 and L2)
    if variant != 1:
        prios = [True] * nh                       # every home priority_rr
    else:
        prios = [nh > 1] + [rnd.random() < 0.5 for _ in range(nh - 1)]
    homes = [(h + 1, prios[h]) for h in range(nh)]
    np_ = rnd.randint(3, 8) if variant < 2 else rnd.randint(10, 14)
    procs = []
    for i in range(np_):
        hid = rnd.randint(1, nh)
        procs.append((hid, i + 1, rnd.choice(['o1', 'o2']), rnd.randint(-2, 2)))
    # rev 3: some runs without inheritance (M.options.inheritance false); some attach a home late
    inheritance = rnd.random() >= 0.15
    late = []
    if rnd.random() < 0.3 and nh > 1:
        late = [homes.pop()]
    l1 = nh == 1 and not prios[0]           # L1: no set-priority
    cache = rnd.choice([0, 1, 2, 3])          # rev 4: key-cache capacity per home
    gt = nh > 1 and rnd.random() < 0.5      # rev 5b: a group's teardown (5.7, L4) in half the others
    cfg = Config(homes, procs, REQS, kinds=('wait', 'wait_dl', 'wait_dl', 'ifavail', 't0'), max_depth=3,
                 max_hdepth=2, prio_values=() if l1 else (-2, -1, 0, 1, 2), signals=10 ** 9,
                 kills=(True, False), destroy=True, prio_changes=10 ** 9, req_per_proc=10 ** 9,
                 coalesce=coalesce, inheritance=inheritance, late_homes=late, foreign=True, cache=cache,
                 cache_clear=True, one_home=nh == 1, group_teardown=gt)
    return World(cfg)


def coalesce_of(seed):
    return (seed // 3) % 2 == 1


def pick(rnd, acts):
    ws = []
    for a in acts:
        w = WEIGHTS[a[0]]
        if a[0] == 'req':
            w = w / (len(REQS) * 5.0)
        elif a[0] == 'prio':
            w = w / 4.0
        elif a[0] == 'kill':
            w = w / 2.0
        elif a[0] == 'foreign':
            w = w / 6.0
        ws.append(w)
    return rnd.choices(acts, ws)[0]


def run_episode(seed, steps, record=True):
    rnd = random.Random(seed)
    w = make_world(rnd, seed % 3, coalesce_of(seed))
    trace = []
    n = 0
    w.check()
    while n < steps:
        acts = w.enabled()
        if not acts:
            break
        a = pick(rnd, acts)
        trace.append(a)
        w.do(a)
        w.check()
        n += 1
    w.drain()
    return n, w


def replay(seed, actions, drain=True):
    """Replay `actions` on the episode's initial world, skipping actions not enabled."""
    rnd = random.Random(seed)
    w = make_world(rnd, seed % 3, coalesce_of(seed))
    w.check()
    used = []
    for a in actions:
        if a not in w.enabled():
            continue
        used.append(a)
        w.do(a)
        w.check()
    if drain:
        w.drain()
    return used, w


def episode_with_trace(seed, steps):
    rnd = random.Random(seed)
    w = make_world(rnd, seed % 3, coalesce_of(seed))
    trace = []
    try:
        w.check()
        n = 0
        while n < steps:
            acts = w.enabled()
            if not acts:
                break
            a = pick(rnd, acts)
            trace.append(a)
            w.do(a)
            w.check()
            n += 1
        w.drain()
        return n, None, trace, w.soft
    except Violation as v:
        return len(trace), v, trace, w.soft
    except Exception as e:
        return len(trace), Violation('crash:' + type(e).__name__, repr(e) + ' ' + traceback.format_exc()[-800:]), trace, w.soft


def violation_of(seed, actions):
    try:
        replay(seed, actions)
    except Violation as v:
        return v.name
    except Exception as e:
        return 'crash:' + type(e).__name__
    return None


def shrink(seed, trace, name, budget=4000):
    cur = list(trace)
    tries = 0
    changed = True
    while changed and tries < budget:
        changed = False
        chunk = max(1, len(cur) // 8)
        while chunk >= 1 and tries < budget:
            i = 0
            while i < len(cur) and tries < budget:
                cand = cur[:i] + cur[i + chunk:]
                tries += 1
                if violation_of(seed, cand) == name:
                    cur = cand
                    changed = True
                else:
                    i += chunk
            chunk //= 2
    used, _ = None, None
    try:
        used, _ = replay(seed, cur, drain=False)
    except Exception:
        pass
    return cur


def render(seed, actions):
    rnd = random.Random(seed)
    w = make_world(rnd, seed % 3, coalesce_of(seed))
    out = ['homes %r; late %r; procs %r; coalesce %r; inheritance %r; cache %r; group teardown %r'
           % (w.cfg.homes, w.cfg.late_homes, w.cfg.procs, w.cfg.coalesce, w.cfg.inheritance,
              w.cfg.cache, w.cfg.group_teardown)]
    for a in actions:
        if a not in w.enabled():
            continue
        out.append(w.describe(a))
        try:
            w.do(a)
            w.check()
        except Exception as e:
            out.append('  -> %s' % e)
            return out
    try:
        w.drain()
    except Exception as e:
        out.append('<fair drain> -> %s' % e)
    return out


def worker(args):
    seed0, total, ep_len = args
    steps = 0
    eps = 0
    viol = {}
    soft = {}
    seed = seed0
    while steps < total:
        n, v, trace, sf = episode_with_trace(seed, ep_len)
        steps += n
        eps += 1
        for sn, sd in sf:
            if sn not in soft:
                soft[sn] = {'detail': sd, 'seed': seed}
        if v is not None and v.name not in viol:
            small = shrink(seed, trace, v.name)
            viol[v.name] = {'detail': str(v), 'seed': seed, 'trace': render(seed, small)}
        seed += 1000003
    return {'steps': steps, 'episodes': eps, 'viol': viol, 'soft': soft, 'stats': dict(STATS)}


def main():
    total = int(sys.argv[1])
    workers = int(sys.argv[2]) if len(sys.argv) > 2 else 20
    seed0 = int(sys.argv[3]) if len(sys.argv) > 3 else 7
    per = total // workers
    t0 = time.time()
    jobs = [(seed0 + i, per, 400) for i in range(workers)]
    allv = {}
    alls = {}
    steps = eps = 0
    stats = {}
    with multiprocessing.Pool(workers) as pool:
        for r in pool.imap_unordered(worker, jobs):
            steps += r['steps']
            eps += r['episodes']
            for k, v in r['viol'].items():
                if k not in allv or len(v['trace']) < len(allv[k]['trace']):
                    allv[k] = v
            for k, v in r['soft'].items():
                alls.setdefault(k, v)
            for k, v in r['stats'].items():
                stats[k] = stats.get(k, 0) + v
    print(json.dumps({'steps': steps, 'episodes': eps, 'seconds': round(time.time() - t0),
                      'violations': allv, 'soft': alls, 'coverage': dict(sorted(stats.items()))}, indent=1))


if __name__ == '__main__':
    main()
