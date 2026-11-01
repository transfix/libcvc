"""Export the Python model's manager operations as traces for the C++ model check (LT-5).

Runs scenarios.py's scenarios on one home and random_runs.py's one-home episodes on fixed seeds
(coalescing off and on) through lockmodel.Manager, and writes every manager operation the world
makes, with its result and its notification list, one operation per line, to OUTDIR/<name>.txt.
state_exec_locks_model_test (LocksModel.PythonTraces) replays every file into its spec oracle and
into the real detail::lock_table, operation by operation, and requires each result and each
notification list to equal the one recorded here.

Phase L1 (docs/STATE_EXEC_LOCKS_PROTOCOL.md 4.4): every home attaches with prio false, and a
manager accepts one home. So every scenario runs with its homes merged into one home of prio
false (the multi-home scenarios whose assertions do not hold on one home, or that need L2's
inheritance, are left out and listed on stderr), and the random episodes are the seeds whose
world is one home of prio false. Phase L2 adds the same on one home of prio true (a scheduler
under priority_rr), files named l2_*: the inheritance scenarios (L2_SCENARIOS) with their homes
merged into that home, and the random episodes of the seeds whose world is one home of prio true.

usage: python -I export_traces.py OUTDIR

The format, one line per item; tokens are separated by single spaces:

    # text                        a comment
    config inheritance=I coalesce=C one_home=O mutations=NAMES
                                  the Manager's options (0 or 1) and its mutation set (lockmodel's
                                  names, comma-separated, '-' for none); the first non-comment line
    OP ARG... -> RESULT... | NOTE...
                                  one operation, its result and its notification list (possibly
                                  empty: the line then ends with '|')

Homes are numbered by the manager, in attach order from 0 (the C++ table hands out home ids that
way); pids are the world's. KEYS is a comma-separated list of canonical absolute keys (segments
joined by '.'), '-' when empty. An owner has no spaces. The operations:

    attach_home PRIO                         -> HOME | refused
    try_acquire HOME PID OWNER BASE MODE KEYS ADD DROP -> granted HOLD GEN AB | busy
    block HOME PID OWNER BASE MODE KEYS ADD DROP       -> TID STATE
    resume HOME PID TID      -> queued | granted HOLD GEN AB | timeout | deadlock CYCLE
    start_body HOME PID HOLD -> ok
    cancel HOME PID TID      -> none | queued | granted | cancelled | failed
    release HOME PID HOLD    -> ok
    end HOME PID ABANDON     -> ok
    detach_home HOME         -> ok
    set_base HOME PID BASE   -> ok
    read_inh HOME PID        -> INH (an integer, or INF)
    query PREFIX VHOME VOWNER -> held ENTRY... waiting ENTRY...
    ref_nodes HOME KEYS      -> ok | refused
    unref_nodes HOME KEYS    -> ok | refused

MODE is S or X; AB, PRIO and ABANDON are 0 or 1; STATE is queued, granted, cancelled or failed.
ADD and DROP are 4.15's refs.add and refs.drop (KEYS lists). CYCLE is one token,
PID:KEY,PID:KEY,... with PID 'nil' for an edge whose source is not shown (4.6). PREFIX is '-' for
the root. A held ENTRY is KEY/MODE/PIDS/COUNT/GEN/AB, PIDS a comma list ('-' when empty) and GEN
'-' for nil; a waiting ENTRY is KEYS/MODE/PID, PID '-' when not visible. 'refused' is a
violation of a 'require' of the protocol (the Manager raised Violation('require')).

A NOTE is wake(HOME,PID,TID) or relevel(HOME,PID).

query is recorded at most once per viewer between two operations that change the lock state
(everything but the key-cache references; the world's check runs it after every action), so the
files stay small; the queries kept are those that see a changed state.

stdlib only.
"""

import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))

import random
import sys

import lockmodel
import random_runs
import scenarios
import world
from lockmodel import INF, Violation, kstr

MUTATING = ('attach_home', 'try_acquire', 'block', 'resume', 'start_body', 'cancel', 'release',
            'end', 'detach_home', 'set_base', 'ref_nodes', 'unref_nodes')


def keys_tok(keys):
    return ','.join(kstr(k) for k in keys) if keys else '-'


def notes_tok(hmap, N):
    out = []
    for h, post in N:
        if post[0] == 'wake':
            out.append('wake(%d,%d,%d)' % (hmap[h], post[1], post[2]))
        else:
            out.append('relevel(%d,%d)' % (hmap[h], post[1]))
    return out


def cycle_tok(cycle):
    return ','.join('%s:%s' % ('nil' if pid is None else pid, kstr(k)) for pid, k, _ in cycle)


class LoggingManager(lockmodel.Manager):
    """lockmodel.Manager, recording every public operation the world calls."""

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.lines = ['config inheritance=%d coalesce=%d one_home=%d mutations=%s'
                      % (self.inheritance, self.coalesce, self.one_home,
                         ','.join(sorted(self.mutation)) or '-')]
        self.hmap = {}
        self.version = 0
        self.queried = {}

    def emit(self, op, args, result, notes=()):
        self.lines.append(' '.join([op] + [str(a) for a in args] + ['->'] + list(result) + ['|']
                                   + list(notes)))

    def P(self, P):
        return [self.hmap[P[0]], P[1]]

    def _call(self, op, fn, args, fmt_args):
        """Run one operation; a 'require' violation is recorded as refused and re-raised."""
        before = self.lock_key() if op in MUTATING else None
        try:
            return fn(*args)
        except Violation as v:
            if v.name == 'require':
                self.emit(op, fmt_args, ['refused'])
            raise
        finally:
            if before is not None and self.lock_key() != before:
                self.version += 1

    def attach_home(self, H, prio, port, progress):
        r = self._call('attach_home', super().attach_home, (H, prio, port, progress), [int(prio)])
        self.hmap[H] = len(self.hmap)
        self.emit('attach_home', [int(prio)], [str(self.hmap[H])], notes_tok(self.hmap, r[1]))
        return r

    def _req_args(self, P, info, keys, mode, refs):
        add, drop = refs
        return self.P(P) + [info[0], info[1], mode, keys_tok(keys), keys_tok(add), keys_tok(drop)]

    def try_acquire(self, P, info, keys, mode, refs=((), ())):
        a = self._req_args(P, info, keys, mode, refs)
        r, N = self._call('try_acquire', super().try_acquire, (P, info, keys, mode, refs), a)
        res = ['busy'] if r[0] == 'busy' else ['granted', str(r[1]), str(r[2]), str(int(r[3]))]
        self.emit('try_acquire', a, res, notes_tok(self.hmap, N))
        return r, N

    def block(self, P, info, keys, mode, refs=((), ())):
        a = self._req_args(P, info, keys, mode, refs)
        (s, st), N = self._call('block', super().block, (P, info, keys, mode, refs), a)
        self.emit('block', a, [str(s), st], notes_tok(self.hmap, N))
        return (s, st), N

    def resume(self, P, tid):
        a = self.P(P) + [tid]
        out, N = self._call('resume', super().resume, (P, tid), a)
        if out[0] == 'queued':
            res = ['queued']
        elif out[0] == 'granted':
            res = ['granted', str(out[1]), str(out[2]), str(int(out[3]))]
        elif out[0] == 'TimeoutError':
            res = ['timeout']
        else:
            res = ['deadlock', cycle_tok(out[1])]
        self.emit('resume', a, res, notes_tok(self.hmap, N))
        return out, N

    def start_body(self, P, hid):
        a = self.P(P) + [hid]
        N = self._call('start_body', super().start_body, (P, hid), a)
        self.emit('start_body', a, ['ok'], notes_tok(self.hmap, N))
        return N

    def cancel(self, P, tid):
        a = self.P(P) + [tid]
        st, N = self._call('cancel', super().cancel, (P, tid), a)
        self.emit('cancel', a, ['none' if st is None else st], notes_tok(self.hmap, N))
        return st, N

    def release(self, P, hid):
        a = self.P(P) + [hid]
        N = self._call('release', super().release, (P, hid), a)
        self.emit('release', a, ['ok'], notes_tok(self.hmap, N))
        return N

    def end(self, P, abandon):
        a = self.P(P) + [int(abandon)]
        N = self._call('end', super().end, (P, abandon), a)
        self.emit('end', a, ['ok'], notes_tok(self.hmap, N))
        return N

    def detach_home(self, H):
        a = [self.hmap[H]]
        N = self._call('detach_home', super().detach_home, (H,), a)
        self.emit('detach_home', a, ['ok'], notes_tok(self.hmap, N))
        return N

    def set_base(self, P, base):
        a = self.P(P) + [base]
        N = self._call('set_base', super().set_base, (P, base), a)
        self.emit('set_base', a, ['ok'], notes_tok(self.hmap, N))
        return N

    def read_inh(self, P):
        a = self.P(P)
        v = self._call('read_inh', super().read_inh, (P,), a)
        self.emit('read_inh', a, ['INF' if v == INF else str(v)])
        return v

    def query(self, pre, viewer):
        r = super().query(pre, viewer)
        if self.queried.get(viewer) == self.version:
            return r
        self.queried[viewer] = self.version
        res = ['held']
        for e in r['held']:
            res.append('%s/%s/%s/%d/%s/%d' % (
                kstr(e['key']), e['mode'], ','.join(str(p) for p in e['pids']) or '-', e['count'],
                '-' if e['generation'] is None else e['generation'], int(e['abandoned'])))
        res.append('waiting')
        for e in r['waiting']:
            res.append('%s/%s/%s' % (keys_tok(e['key']), e['mode'], '-' if e['pid'] is None else e['pid']))
        self.emit('query', [kstr(pre) or '-', self.hmap[viewer[0]], viewer[1]], res)
        return r

    def ref_nodes(self, H, keys):
        a = [self.hmap.get(H, H), keys_tok(keys)]
        N = self._call('ref_nodes', super().ref_nodes, (H, keys), a)
        self.emit('ref_nodes', a, ['ok'], notes_tok(self.hmap, N))
        return N

    def unref_nodes(self, H, keys):
        a = [self.hmap.get(H, H), keys_tok(keys)]
        N = self._call('unref_nodes', super().unref_nodes, (H, keys), a)
        self.emit('unref_nodes', a, ['ok'], notes_tok(self.hmap, N))
        return N


# -- scenarios on one home ------------------------------------------------------------------------

def one_home_mk(created, prio=False):
    """scenarios.mk with every home merged into home 1, of prio false (protocol 4.4 in L1) or, for
    phase L2, of prio `prio`, on a manager that accepts one home; a late home stays a late home
    (prio false)."""
    def mk(homes, procs, reqs, kinds=('wait', 'wait_dl', 'ifavail', 't0'), mutation=None, **kw):
        kw = dict(kw)
        late = [(h, False) for h, _ in kw.pop('late_homes', ())]
        lids = set(h for h, _ in late)
        procs = [(hid if hid in lids else 1, pid, owner, base) for hid, pid, owner, base in procs]
        kw.pop('one_home', None)
        cfg = world.Config([(1, prio)], procs, reqs, kinds=kinds, max_depth=3, max_hdepth=2, signals=2,
                           kills=(True, False), destroy=True, prio_values=(-10, -2, 0, 2, 5, 10),
                           mutation=mutation, prio_changes=3, req_per_proc=5, late_homes=late,
                           one_home=True, **kw)
        w = world.World(cfg)
        created.append(w)
        return w
    return mk


# the inheritance scenarios phase L2 runs on one home of prio true
L2_SCENARIOS = ('scenario_chain_cap8', 'scenario_coalesce', 'scenario_impl_note', 'scenario_l2_one_home',
                'scenario_pi11', 'scenario_pi9', 'scenario_set_base')


def run_scenarios(out, prio=False):
    skipped = []
    names = sorted(n for n in dir(scenarios) if n.startswith('scenario_'))
    if prio:
        names = [n for n in names if n in L2_SCENARIOS]
    for name in names:
        fn = getattr(scenarios, name)
        created = []
        scenarios.mk = one_home_mk(created, prio)
        try:
            fn()
        except Violation as v:
            if not (name == 'scenario_one_home_refusal' and v.name == 'require'):
                skipped.append('%s (%s)' % (name, v))
                continue
        except Exception as e:      # an assertion of the scenario that needs several homes or L2
            skipped.append('%s (%s: %s)' % (name, type(e).__name__, str(e)[:60]))
            continue
        finally:
            scenarios.mk = REAL_MK
        if len(created) != 1:
            skipped.append('%s (%d worlds)' % (name, len(created)))
            continue
        # scenario_one_home_refusal returns from its except clause: the refusal is its last line
        out[('l2_' if prio else '') + 'scenario_' + name[len('scenario_'):]] = (
            ['# scenarios.py %s, its homes merged into one home of prio %s'
             % (name, 'true' if prio else 'false')]
            + created[0].M.lines)
    return skipped


# -- random runs on one home ------------------------------------------------------------------------

def one_home_seeds(coalesce, count, start=1, prio=False):
    """The first `count` seeds from `start` whose random_runs world is one home of prio `prio`
    (false: phase L1, true: phase L2) with the given coalescing."""
    seeds = []
    seed = start
    while len(seeds) < count:
        if random_runs.coalesce_of(seed) == coalesce:
            rnd = random.Random(seed)
            w = random_runs.make_world(rnd, seed % 3, coalesce)
            if len(w.cfg.homes) == 1 and not w.cfg.late_homes and w.cfg.homes[0][1] == prio:
                seeds.append(seed)
        seed += 1
    return seeds


def run_random(out, steps, prio=False):
    for coalesce in (False, True):
        for seed in one_home_seeds(coalesce, 4, prio=prio):
            n, w = random_runs.run_episode(seed, steps)
            out[('l2_' if prio else '') + 'random_seed%d' % seed] = (
                ['# random_runs.py seed %d (coalescing %s): %d steps then the fair drain; procs %r, '
                 'cache %r, inheritance %r' % (seed, 'on' if coalesce else 'off', n, w.cfg.procs,
                                               w.cfg.cache, w.cfg.inheritance)]
                + w.M.lines)


REAL_MK = scenarios.mk


def main():
    if len(sys.argv) != 2:
        print(__doc__.split('\n\n')[3], file=sys.stderr)
        sys.exit(2)
    outdir = sys.argv[1]
    world.Manager = LoggingManager          # World() makes its manager through this name
    out = {}
    skipped = run_scenarios(out)
    run_random(out, 200)
    skipped += ['%s (prio true)' % s for s in run_scenarios(out, prio=True)]
    run_random(out, 200, prio=True)
    _os.makedirs(outdir, exist_ok=True)
    total = 0
    for name in sorted(out):
        text = '\n'.join(out[name]) + '\n'
        total += len(text)
        with open(_os.path.join(outdir, name + '.txt'), 'w', newline='\n') as f:
            f.write(text)
        print('%-32s %6d lines' % (name, len(out[name])))
    print('%d files, %d bytes' % (len(out), total))
    for s in skipped:
        print('not on one home: %s' % s, file=sys.stderr)


if __name__ == '__main__':
    main()
