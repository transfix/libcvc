"""Homes, processes and the step relation (docs/STATE_EXEC_LOCKS_PROTOCOL.md sections 5 and 7)."""

import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))

from lockmodel import (INF, S, X, Violation, Manager, Port, Cell, conflicts, overlap, under, compat,
                       canonical, kstr, req_str, STATS)


def pid_of(p):
    return p.pid


class MHold(object):
    __slots__ = ('id', 'keys', 'mode', 'count', 'gen', 'abandoned', 'in_handler')

    def __init__(self, id, keys, mode, count, gen, abandoned, in_handler):
        self.id = id
        self.keys = keys
        self.mode = mode
        self.count = count
        self.gen = gen
        self.abandoned = abandoned
        self.in_handler = in_handler

    def clone(self):
        return MHold(self.id, self.keys, self.mode, self.count, self.gen, self.abandoned, self.in_handler)

    def key(self):
        return (self.id, self.keys, self.mode, self.count, self.gen, self.abandoned, self.in_handler)


class Frame(object):
    """lock_body {hold} or lock_enter {outcome}; `keys`, `mode` the request, `ih` in a handler."""
    __slots__ = ('kind', 'hold', 'keys', 'mode', 'outcome', 'deadline', 'ih', 'tid')

    def __init__(self, kind, hold, keys, mode, outcome, deadline, ih, tid=None):
        self.kind = kind
        self.hold = hold
        self.keys = keys
        self.mode = mode
        self.outcome = outcome
        self.deadline = deadline
        self.ih = ih
        self.tid = tid

    def clone(self):
        return Frame(self.kind, self.hold, self.keys, self.mode, self.outcome, self.deadline, self.ih, self.tid)

    def key(self):
        return (self.kind, self.hold, self.keys, self.mode, self.outcome, self.deadline, self.ih, self.tid)


class Proc(object):
    __slots__ = ('hid', 'pid', 'owner', 'status', 'priority', 'inherited', 'locks', 'lock_ticket',
                 'lock_waiting', 'lock_deadline', 'in_handler', 'hbase', 'frames',
                 'req_left', 'sig_left', 'prio_left', 'pc', 'hpc')

    def __init__(self, hid, pid, owner, priority, req_left, sig_left, prio_left):
        self.hid = hid
        self.pid = pid
        self.owner = owner
        self.status = 'run'
        self.priority = priority
        self.inherited = INF
        self.locks = []
        self.lock_ticket = None
        self.lock_waiting = False
        self.lock_deadline = False
        self.in_handler = False
        self.hbase = None
        self.frames = []
        self.req_left = req_left
        self.sig_left = sig_left
        self.prio_left = prio_left
        self.pc = 0
        self.hpc = 0

    def clone(self):
        p = Proc.__new__(Proc)
        p.hid = self.hid
        p.pid = self.pid
        p.owner = self.owner
        p.status = self.status
        p.priority = self.priority
        p.inherited = self.inherited
        p.locks = [h.clone() for h in self.locks]
        p.lock_ticket = self.lock_ticket
        p.lock_waiting = self.lock_waiting
        p.lock_deadline = self.lock_deadline
        p.in_handler = self.in_handler
        p.hbase = self.hbase
        p.frames = [f.clone() for f in self.frames]
        p.req_left = self.req_left
        p.sig_left = self.sig_left
        p.prio_left = self.prio_left
        p.pc = self.pc
        p.hpc = self.hpc
        return p

    def key(self):
        return (self.pid, self.status, self.priority, self.inherited, tuple(h.key() for h in self.locks),
                self.lock_ticket, self.lock_waiting, self.lock_deadline, self.in_handler, self.hbase,
                tuple(f.key() for f in self.frames), self.req_left, self.sig_left, self.prio_left,
                self.pc, self.hpc)

    @property
    def P(self):
        return (self.hid, self.pid)


class Home(object):
    __slots__ = ('hid', 'prio', 'port', 'pending', 'destroyed', 'attached', 'cache', 'progress')

    def __init__(self, hid, prio, attached=True):
        self.hid = hid
        self.prio = prio
        self.port = Port()
        self.pending = []
        self.destroyed = False
        self.attached = attached
        self.cache = ()         # rev 4: the key cache, FIFO, at most World.cache_cap keys
        self.progress = Cell()  # rev 5: last_progress (2.1), shared with M.homes; not in key()

    def clone(self):
        h = Home.__new__(Home)
        h.hid = self.hid
        h.prio = self.prio
        h.port = self.port.clone()
        h.pending = list(self.pending)
        h.destroyed = self.destroyed
        h.attached = self.attached
        h.cache = self.cache
        h.progress = Cell(self.progress.value)
        return h

    def key(self):
        return (self.hid, self.prio, tuple(self.port.queue), self.port.closed, tuple(self.pending), self.destroyed,
                self.attached, self.cache)


class Config(object):
    def __init__(self, homes, procs, requests, kinds=('wait',), max_depth=2, max_hdepth=1,
                 prio_values=(), signals=0, kills=(True,), destroy=False, inheritance=True,
                 mutation=None, prio_changes=0, req_per_proc=2, scripts=None, handler_script=(),
                 faults=0, fault_kinds=('kill', 'raise', 'signal', 'deadline', 'prio', 'terminate'),
                 coalesce=False, late_homes=(), split_post=False, foreign=False, cache=0,
                 cache_clear=False, one_home=False, group_teardown=False):
        self.homes = homes            # list of (hid, prio)
        self.procs = procs            # list of (hid, pid, owner, base)
        self.requests = requests      # list of (keys, mode)
        self.kinds = kinds            # wait, wait_dl, ifavail, t0
        self.max_depth = max_depth
        self.max_hdepth = max_hdepth
        self.prio_values = prio_values
        self.signals = signals
        self.kills = kills
        self.destroy = destroy
        self.inheritance = inheritance
        self.mutation = mutation or set()
        self.prio_changes = prio_changes
        self.req_per_proc = req_per_proc
        # scripted mode: pid -> tuple of (request index, kind), issued nested; a handler runs
        # handler_script the same way; faults bounds kills, injected raises, signals, deadline
        # firings and priority changes over the whole run
        self.scripts = scripts
        self.handler_script = handler_script
        self.faults = faults
        self.fault_kinds = fault_kinds
        self.coalesce = coalesce          # 4.3: at most one relevel per process in a port
        self.late_homes = late_homes      # (hid, prio) attached by an action, after others lock
        self.split_post = split_post      # 2.3 read literally: closed test, then append + signal
        self.foreign = foreign            # I12: release/cancel calls naming another's hold/ticket
        self.cache = cache                # rev 4: key-cache capacity per home (0: no cache); rev 5:
                                          # a tuple is a choice, made by the first action
        self.cache_clear = cache_clear    # rev 5: a host may clear its cache (unref_nodes, 4.15)
        self.one_home = one_home          # rev 5: a manager before L3, which accepts one home (4.4)
        self.group_teardown = group_teardown  # rev 5b: destroy is a group's teardown (5.7, L4)


EXTERNAL = ('req', 'enter_step', 'finish', 'raise', 'hraise', 'signal', 'hend', 'prio', 'kill',
            'deadline', 'destroy', 'attach', 'foreign', 'cclear')

# rev 5: actions that run a process's step, which advance its home's progress cell (a logical clock)
STEPS = ('req', 'enter_step', 'finish', 'raise', 'hraise', 'signal', 'hend')


class World(object):
    def __init__(self, cfg):
        self.cfg = cfg
        self.M = Manager(inheritance=cfg.inheritance, mutation=cfg.mutation, coalesce=cfg.coalesce,
                         one_home=cfg.one_home)
        self.homes = {}
        for hid, prio in cfg.homes:
            self.homes[hid] = Home(hid, prio)
            self.M.attach_home(hid, prio, self.homes[hid].port, self.homes[hid].progress)
        for hid, prio in cfg.late_homes:
            self.homes[hid] = Home(hid, prio, attached=False)
        self.procs = {}
        for hid, pid, owner, base in cfg.procs:
            self.procs[pid] = Proc(hid, pid, owner, base, cfg.req_per_proc, cfg.signals, cfg.prio_changes)
        self.soft = []
        self.events = 0
        self.faults_left = cfg.faults
        self._need_mcheck = True
        # rev 5: the key-cache capacity; None until the first action picks one of a tuple
        self.cache_cap = None if isinstance(cfg.cache, tuple) else cfg.cache

    def clone(self):
        w = World.__new__(World)
        w.cfg = self.cfg
        w.homes = dict((h, v.clone()) for h, v in self.homes.items())
        ports = dict((h, v.port) for h, v in w.homes.items())
        cells = dict((h, v.progress) for h, v in w.homes.items())
        w.M = self.M.clone(ports, cells)
        w.procs = dict((p, v.clone()) for p, v in self.procs.items())
        w.soft = []
        w.events = self.events
        w.faults_left = self.faults_left
        w._need_mcheck = False
        w.cache_cap = self.cache_cap
        return w

    def key(self):
        return (self.M.key(), tuple(self.homes[h].key() for h in sorted(self.homes)),
                tuple(self.procs[p].key() for p in sorted(self.procs)), self.faults_left, self.cache_cap)

    # ------------------------------------------------------------------------------------------
    def fail(self, name, detail):
        raise Violation(name, detail)

    def info(self, p):
        return (p.owner, p.priority)

    def mcall(self, fn, *args):
        """A manager operation: atomic; collect its soft findings; then the manager's invariants
        (section 6: after every operation)."""
        M = self.M
        tb = dict((t, T.state) for t, T in M.tickets.items())
        hb = set(M.holds)
        r = fn(*args)
        M.check()
        self.check_transitions(fn.__name__, tb, hb)
        if self.M.soft:
            self.soft.extend(self.M.soft)
            self.M.soft = []
        return r

    MOVES = (('queued', 'granted'), ('queued', 'cancelled'))

    def check_transitions(self, op, tb, hb):
        """I2: ticket states only move queued -> granted or queued -> cancelled, are created queued
        (by block), granted or failed (by block), and a ticket is deleted only by resume or end
        (detach_home ends recs too); I10: a hold is dropped only by release or end, once (hold ids
        are never reused)."""
        M = self.M
        for t, st in tb.items():
            T = M.tickets.get(t)
            if T is None:
                if op not in ('resume', 'end', 'detach_home'):
                    self.fail('I2', 'ticket %d deleted by %s' % (t, op))
            elif T.state != st and (st, T.state) not in self.MOVES:
                self.fail('I2', 'ticket %d moved %s -> %s in %s' % (t, st, T.state, op))
        for t, T in M.tickets.items():
            if t not in tb and (op != 'block' or T.state not in ('queued', 'granted', 'failed')):
                self.fail('I2', 'ticket %d created %s by %s' % (t, T.state, op))
        for i in hb:
            if i not in M.holds and op not in ('release', 'end', 'detach_home'):
                self.fail('I10', 'hold %d dropped by %s' % (i, op))

    def proc_of_home(self, hid, pid):
        p = self.procs.get(pid)
        if p is None or p.hid != hid or p.status == 'ended':
            return None
        return p

    def idle(self, hid):
        h = self.homes[hid]
        return not h.destroyed and not h.pending and h.attached

    # ------------------------------------------------------------------------------------------
    # enabled actions
    def enabled(self):
        if self.cache_cap is None:
            return [('cachecap', c) for c in self.cfg.cache]
        if self.cfg.scripts is not None:
            return self.enabled_scripted()
        acts = []
        cfg = self.cfg
        for hid in sorted(self.homes):
            h = self.homes[hid]
            if h.destroyed:
                continue
            if not h.attached:
                acts.append(('attach', hid))
                continue
            if h.pending:
                acts.append(('hstep', hid))
                continue
            if h.port.queue:
                acts.append(('post', hid))
            if cfg.destroy:
                acts.append(('destroy', hid))
            if cfg.cache_clear and h.cache:
                acts.append(('cclear', hid))
        for pid in sorted(self.procs):
            p = self.procs[pid]
            if p.status == 'ended' or not self.idle(p.hid):
                continue
            if cfg.foreign:
                acts.extend(self.foreign_acts(p))
            if p.prio_left > 0:
                for v in cfg.prio_values:
                    if v != p.priority:
                        acts.append(('prio', pid, v))
            for ab in cfg.kills:
                acts.append(('kill', pid, ab))
            if p.status == 'parked':
                if p.lock_deadline:
                    acts.append(('deadline', pid))
                continue
            base = p.hbase if p.in_handler else 0
            top = p.frames[-1] if len(p.frames) > base else None
            if top is not None and top.kind == 'enter':
                acts.append(('enter_step', pid))
                acts.append(('raise', pid))
            else:
                depth = len(p.frames) - base
                lim = cfg.max_hdepth if p.in_handler else cfg.max_depth
                if p.req_left > 0 and depth < lim:
                    for ri in range(len(cfg.requests)):
                        for kind in cfg.kinds:
                            acts.append(('req', pid, ri, kind))
                if top is not None:
                    acts.append(('finish', pid))
                    acts.append(('raise', pid))
            if not p.in_handler and p.sig_left > 0:
                acts.append(('signal', pid))
            if p.in_handler:
                if len(p.frames) == p.hbase:
                    acts.append(('hend', pid))
                acts.append(('hraise', pid))
        return acts

    def enabled_scripted(self):
        acts = []
        cfg = self.cfg
        fk = cfg.fault_kinds if self.faults_left > 0 else ()
        for hid in sorted(self.homes):
            h = self.homes[hid]
            if h.destroyed:
                continue
            if not h.attached:
                acts.append(('attach', hid))
                continue
            if h.pending:
                acts.append(('hstep', hid))
                continue
            if h.port.queue:
                acts.append(('post', hid))
            if cfg.destroy and 'destroy' in fk:
                acts.append(('destroy', hid))
        for pid in sorted(self.procs):
            p = self.procs[pid]
            if p.status == 'ended' or not self.idle(p.hid):
                continue
            if 'kill' in fk:
                acts.append(('kill', pid, True))
            if 'terminate' in fk:
                acts.append(('kill', pid, False))
            if 'prio' in fk:
                for v in cfg.prio_values:
                    if v != p.priority:
                        acts.append(('prio', pid, v))
            if p.status == 'parked':
                if p.lock_deadline and 'deadline' in fk:
                    acts.append(('deadline', pid))
                continue
            base = p.hbase if p.in_handler else 0
            top = p.frames[-1] if len(p.frames) > base else None
            script = cfg.handler_script if p.in_handler else cfg.scripts.get(pid, ())
            pc = p.hpc if p.in_handler else p.pc
            if top is not None and top.kind == 'enter':
                acts.append(('enter_step', pid))
                if 'raise' in fk:
                    acts.append(('raise', pid))
            elif pc < len(script):
                acts.append(('req', pid, script[pc][0], script[pc][1]))
                if top is not None and 'raise' in fk:
                    acts.append(('raise', pid))
            elif top is not None:
                acts.append(('finish', pid))
                if 'raise' in fk:
                    acts.append(('raise', pid))
            elif p.in_handler:
                acts.append(('hend', pid))
            if p.in_handler and 'raise' in fk:
                acts.append(('hraise', pid))
            if not p.in_handler and 'signal' in fk and p.sig_left > 0 and cfg.handler_script:
                acts.append(('signal', pid))
        return acts

    def foreign_acts(self, p):
        out = []
        for i, h in sorted(self.M.holds.items()):
            if h.pid != p.P:
                out.append(('foreign', p.pid, 'release', i))
        for t, T in sorted(self.M.tickets.items()):
            if T.pid != p.P:
                out.append(('foreign', p.pid, 'cancel', t))
        out.append(('foreign', p.pid, 'release', self.M.next_hold + 1))
        out.append(('foreign', p.pid, 'cancel', self.M.next_seq + 1))
        return out

    def describe(self, a):
        if a[0] == 'req':
            keys, mode = self.cfg.requests[a[2]]
            return 'req p%d %s %s' % (a[1], req_str(keys, mode), a[3])
        if a[0] == 'cachecap':
            return 'key caches of %d entries' % a[1]
        if a[0] == 'cclear':
            return 'H%s clears its key cache %s' % (a[1], [kstr(k) for k in self.homes[a[1]].cache])
        if a[0] in ('hstep', 'post', 'destroy', 'attach'):
            h = self.homes[a[1]]
            if a[0] == 'hstep':
                return 'H%s: %r' % (a[1], h.pending[0])
            if a[0] == 'post':
                return 'H%s applies post %r' % (a[1], h.port.queue[0])
            if a[0] == 'attach':
                return 'attach H%s (prio %s)' % (a[1], h.prio)
            return 'destroy H%s' % a[1]
        return ' '.join(str(x) if not isinstance(x, int) or i != 1 else 'p%d' % x for i, x in enumerate(a))

    # ------------------------------------------------------------------------------------------
    def do(self, a):
        k = a[0]
        if k in EXTERNAL:
            self.events += 1
        if k in STEPS:
            self.homes[self.procs[a[1]].hid].progress.value += 1
        if self.cfg.scripts is not None and k in ('kill', 'raise', 'hraise', 'signal', 'deadline', 'prio', 'destroy'):
            self.faults_left -= 1
        getattr(self, 'a_' + k)(*a[1:])

    def push_front(self, hid, ops):
        h = self.homes[hid]
        h.pending[0:0] = ops

    def dlv_ops(self, N):
        return [('dlv', tgt, post) for (tgt, post) in N]

    # -- home steps ---------------------------------------------------------------------------
    LOCAL_OPS = ('after_block', 'end_clear', 'exit_handler', 'destroy_clear')

    def cache_plan(self, hid, keys):
        """4.15: the home probes its key cache; a miss adds an entry (and its reference), and
        an entry over capacity is evicted (dropping its reference), in the manager operation's
        own critical section. Returns the new cache and (add, drop)."""
        cap = self.cache_cap
        if not cap:
            return (), ((), ())
        cache = list(self.homes[hid].cache)
        add, drop = [], []
        for k in keys:
            if k in cache:
                continue
            cache.append(k)
            add.append(k)
            while len(cache) > cap:
                drop.append(cache.pop(0))
        if 'cache-miss-unreferenced' in self.cfg.mutation:
            add = []            # the home caches a miss without its reference: I4 (home side)
            drop = [k for k in drop if k in self.M.cache_refs.get(hid, {})]
        return tuple(cache), (tuple(add), tuple(drop))

    def a_cachecap(self, cap):
        self.cache_cap = cap

    def a_hstep(self, hid):
        h = self.homes[hid]
        op = h.pending.pop(0)
        getattr(self, 'op_' + op[0])(hid, *op[1:])
        # private micro-operations commute with every other home's action: run them at once
        while not h.destroyed and h.pending and h.pending[0][0] in self.LOCAL_OPS:
            op = h.pending.pop(0)
            getattr(self, 'op_' + op[0])(hid, *op[1:])

    def op_dlv(self, hid, tgt, post):
        # 4.14: deliver one notification: to the caller's own home, applied at once; else
        # n.port.post(n.post)
        if tgt == hid:
            self.apply(hid, post)
            return
        if self.cfg.split_post:
            # rev 4: N was captured under M.mu; the post's critical section under the port's
            # mutex (2.3) is a later action of the posting thread (section 7)
            if 'post-test-outside-mu' in self.cfg.mutation and self.homes[tgt].port.closed:
                STATS['post_dropped_closed_port'] += 1      # the rev-3 reading: tested outside mu
                return
            self.push_front(hid, [('dlv_append', tgt, post)])
            return
        self.port_post(hid, tgt, post)

    def op_dlv_append(self, hid, tgt, post):
        h = self.homes[tgt]
        if 'post-test-outside-mu' in self.cfg.mutation:
            # the rev-3 reading: the closed test was made in op_dlv, outside any lock
            if h.port.closed:
                self.fail('port-race', 'H%s appends %r to H%s\'s port and signals its scheduler after '
                          'it was closed%s' % (hid, post, tgt,
                                               ' and the scheduler was destroyed' if h.destroyed else ''))
            STATS['posted_' + post[0]] += 1
            h.port.queue.append(post)
            return
        self.port_post(hid, tgt, post)

    def port_post(self, hid, tgt, post):
        # 2.3 port.post(x): lock mu; if closed: unlock, return false; append x; pending = true;
        # wake->notify(); unlock; return true. One action: one critical section.
        h = self.homes[tgt]
        if h.port.closed:
            STATS['post_dropped_closed_port'] += 1
            return False
        if h.destroyed:
            self.fail('port-race', 'H%s posts to H%s after its scheduler was destroyed with the port open'
                      % (hid, tgt))
        STATS['posted_' + post[0]] += 1
        h.port.queue.append(post)
        return True

    def op_block(self, hid, pid):
        p = self.procs[pid]
        f = p.frames[-1]
        if f.kind != 'enter' or f.outcome is not None:
            self.fail('model', 'block without a pending lock_enter frame')
        cache, refs = self.cache_plan(hid, f.keys)
        (tid, st), N = self.mcall(self.M.block, p.P, self.info(p), f.keys, f.mode, refs)
        self.homes[hid].cache = cache
        f.tid = tid
        STATS['block_' + st] += 1
        self.push_front(hid, self.dlv_ops(N) + [('after_block', pid, tid, st)])

    def op_after_block(self, hid, pid, tid, st):
        p = self.procs[pid]
        f = p.frames[-1]
        if st == 'queued':
            p.lock_ticket = tid
            p.lock_waiting = True
            p.lock_deadline = f.deadline
            p.status = 'parked'
        else:
            self.collect(p, f, tid)

    def op_wake_self(self, hid, pid, tid, st):
        p = self.procs[pid]
        self.apply_wake(hid, pid, tid)
        f = p.frames[-1]
        # a timeout that finds a grant adopts it
        if st == 'granted' and (f.outcome is None or f.outcome[0] != 'granted'):
            self.fail('adopt', 'cancel returned granted but the waiter did not adopt the grant')
        if st == 'cancelled' and (f.outcome is None or f.outcome[0] != 'TimeoutError'):
            self.fail('adopt', 'cancel cancelled but the outcome is %r' % (f.outcome,))
        if st is None:
            self.fail('adopt', 'cancel found no ticket for a waiting process')
        if p.lock_waiting:
            self.fail('adopt', 'process still parked after its deadline fired')

    def op_unwind(self, hid, pid):
        p = self.procs[pid]
        f = p.frames.pop()
        self.release_lock_frame(hid, p, f)

    def op_exit_handler(self, hid, pid):
        p = self.procs[pid]
        if len(p.frames) != p.hbase:
            self.fail('model', 'exit handler with handler frames')
        p.in_handler = False
        p.hbase = None

    def op_end_clear(self, hid, pid):
        p = self.procs[pid]
        p.locks = []
        p.lock_ticket = None
        p.lock_waiting = False
        p.lock_deadline = False
        p.inherited = INF
        p.frames = []
        p.in_handler = False
        p.hbase = None

    def op_destroy_clear(self, hid):
        h = self.homes[hid]
        h.port.queue = []
        h.cache = ()
        for p in self.procs.values():
            if p.hid == hid:
                p.status = 'ended'
                self.op_end_clear(hid, p.pid)
        h.destroyed = True

    # -- 5.9 posts ------------------------------------------------------------------------------
    def a_post(self, hid):
        post = self.homes[hid].port.queue.pop(0)
        self.apply(hid, post)

    def apply(self, hid, post):
        if post[0] == 'wake':
            self.apply_wake(hid, post[1], post[2])
        else:
            p = self.proc_of_home(hid, post[1])
            if p is None:
                return
            v = self.mcall(self.M.read_inh, p.P)
            if p.inherited != v:
                STATS['relevel_changed_priority'] += 1
                if v != INF:
                    STATS['boosted'] += 1
                p.inherited = v

    def apply_wake(self, hid, pid, tid):
        p = self.proc_of_home(hid, pid)
        if p is None:
            return
        if 'wake-no-tid-check' in self.cfg.mutation:
            if not p.lock_waiting:
                return
            if p.lock_ticket != tid:
                # the wake carries an outcome: it resumes whatever the process waits on now
                f = p.frames[-1]
                p.lock_ticket = None
                p.lock_waiting = False
                p.lock_deadline = False
                p.status = 'run'
                return
        if p.lock_ticket != tid or not p.lock_waiting:
            STATS['stale_wake_ignored'] += 1
            return
        f = p.frames[-1]
        if f.kind != 'enter' or f.tid != tid:
            self.fail('model', 'parked process without its lock_enter frame on top')
        if not self.collect(p, f, tid):
            return
        p.lock_ticket = None
        p.lock_waiting = False
        p.lock_deadline = False
        p.status = 'run'

    def collect(self, p, f, tid):
        out, _ = self.mcall(self.M.resume, p.P, tid)
        if out[0] == 'queued':
            return False
        if out[0] == 'granted':
            _, hid_, gen, ab = out
            h = self.M.holds[hid_]
            if h.keys != f.keys or h.mode != f.mode:
                self.fail('collect', 'granted hold %r %s does not match the frame\'s request %s'
                          % (hid_, req_str(h.keys, h.mode), req_str(f.keys, f.mode)))
            p.locks.append(MHold(hid_, f.keys, f.mode, 1, gen, ab, p.in_handler))
            f.outcome = ('granted', hid_)
        else:
            f.outcome = out
        return True

    # -- 5.4 release_lock_frame --------------------------------------------------------------------
    def release_lock_frame(self, hid, p, f):
        rid = None
        if f.kind == 'body':
            rid = f.hold
        elif f.outcome is not None and f.outcome[0] == 'granted':
            rid = f.outcome[1]
        ops = []
        if rid is not None and f.kind == 'enter':
            STATS['unwound_granted_unstarted'] += 1
        if rid is not None:
            h = None
            for m in p.locks:
                if m.id == rid:
                    h = m
                    break
            if h is None:
                return
            h.count -= 1
            if h.count == 0:
                p.locks.remove(h)
                N = self.mcall(self.M.release, p.P, h.id)
                ops += self.dlv_ops(N)
        if f.kind == 'enter' and p.lock_ticket is not None:
            self.soft.append(('defensive-path', 'release_lock_frame withdrew a ticket still pending'))
            tid = p.lock_ticket
            _, N2 = self.mcall(self.M.cancel, p.P, tid)
            ops += self.dlv_ops(N2) + [('defensive_resume', pid_of(p), tid)]
        self.push_front(hid, ops)

    def op_defensive_resume(self, hid, pid, tid):
        p = self.procs[pid]
        self.mcall(self.M.resume, p.P, tid)
        p.lock_ticket = None
        p.lock_waiting = False
        p.lock_deadline = False

    # -- process actions ------------------------------------------------------------------------------
    def a_req(self, pid, ri, kind):
        p = self.procs[pid]
        p.req_left -= 1
        if p.in_handler:
            p.hpc += 1
        else:
            p.pc += 1
        keys, mode = self.cfg.requests[ri]
        keys = canonical(keys)
        hid = p.hid
        in_h = p.in_handler
        view = [h for h in p.locks if (not in_h) or h.in_handler]
        # 1. the handler rule
        if in_h:
            for k in keys:
                for h in p.locks:
                    if h.in_handler:
                        continue
                    if ((under(h.keys, k) and (h.mode == X or mode == S))
                            or (any(overlap(hk, k) for hk in h.keys) and not compat(h.mode, mode))):
                        return  # LockError("handler"): nothing changes
        # 2. upgrade
        if mode == X:
            for k in keys:
                if (any(h.mode == S and under(h.keys, k) for h in view)
                        and not any(h.mode == X and under(h.keys, k) for h in view)):
                    return  # LockError("upgrade")
        # 3. covered
        if all(any(under(h.keys, k) and (h.mode == X or mode == S) for h in view) for k in keys):
            c = None
            for h in view:
                if under(h.keys, keys[0]) and (h.mode == X or mode == S):
                    c = h
                    break
            self._g_covered(p, keys, mode)
            STATS['covered'] += 1
            c.count += 1
            p.frames.append(Frame('body', c.id, keys, mode, None, False, in_h))
            return
        # 4. fast path
        k_before = self.M.lock_key()
        cache, refs = self.cache_plan(hid, keys)
        r = self.mcall(self.M.try_acquire, p.P, self.info(p), keys, mode, refs)
        self.homes[hid].cache = cache
        if r[0][0] == 'busy' and self.M.lock_key() != k_before:
            self.fail('I13', 'a busy try_acquire changed the manager')
        if r[0][0] == 'granted':
            _, hid_, gen, ab = r[0]
            p.locks.append(MHold(hid_, keys, mode, 1, gen, ab, in_h))
            p.frames.append(Frame('body', hid_, keys, mode, None, False, in_h))
            return
        # 5. busy
        if kind == 'ifavail':
            p.frames.append(Frame('body', None, keys, mode, None, False, in_h))
            return
        if kind == 't0':
            return  # TimeoutError
        p.frames.append(Frame('enter', None, keys, mode, None, kind == 'wait_dl', in_h))
        self.homes[hid].pending.append(('block', pid))

    def _g_covered(self, p, keys, mode):
        M = self.M
        if M.blk_hold(p.P, keys, mode):
            self.fail('covered', 'covered request %s of %r conflicts with another process\'s hold'
                      % (req_str(keys, mode), p.P))
        for t in sorted(M.queue):
            U = M.tickets[t]
            if U.pid != p.P and conflicts(keys, mode, U.keys, U.mode) and not M.exempt(p.P, U):
                self.fail('covered', 'covered request %s of %r passes earlier non-exempt waiter %r'
                          % (req_str(keys, mode), p.P, U.pid))

    def a_enter_step(self, pid):
        p = self.procs[pid]
        f = p.frames[-1]
        if f.outcome is None:
            self.fail('I2', 'lock_enter step with no outcome (woken without a grant)')
        if f.outcome[0] == 'granted':
            self.mcall(self.M.start_body, p.P, f.outcome[1])
            p.frames[-1] = Frame('body', f.outcome[1], f.keys, f.mode, None, False, f.ih)
        else:
            p.frames.pop()
            self.release_lock_frame(p.hid, p, f)

    def a_finish(self, pid):
        p = self.procs[pid]
        self.homes[p.hid].pending.append(('unwind', pid))
        self.a_hstep(p.hid)

    a_raise = a_finish

    def a_hraise(self, pid):
        p = self.procs[pid]
        ops = [('unwind', pid)] * (len(p.frames) - p.hbase) + [('exit_handler', pid)]
        if p.hbase > 0:
            ops.append(('unwind', pid))
        self.homes[p.hid].pending.extend(ops)
        self.a_hstep(p.hid)

    def a_signal(self, pid):
        p = self.procs[pid]
        p.sig_left -= 1
        if p.frames and p.frames[-1].kind == 'enter':
            STATS['handler_between_grant_and_body'] += 1
        p.in_handler = True
        p.hbase = len(p.frames)
        p.hpc = 0

    def a_hend(self, pid):
        p = self.procs[pid]
        p.in_handler = False
        p.hbase = None

    def a_prio(self, pid, v):
        p = self.procs[pid]
        p.prio_left -= 1
        p.priority = v
        if (p.locks or p.lock_ticket is not None) and 'no-set-base' not in self.cfg.mutation:
            N = self.mcall(self.M.set_base, p.P, v)
            self.push_front(p.hid, self.dlv_ops(N))

    def a_kill(self, pid, abandon):
        p = self.procs[pid]
        if p.lock_ticket is not None:
            T = self.M.tickets.get(p.lock_ticket)
            STATS['end_while_ticket_' + (T.state if T else 'none')] += 1
        p.status = 'ended'
        if not p.locks and p.lock_ticket is None:
            self.op_end_clear(p.hid, pid)
            return
        N = self.mcall(self.M.end, p.P, abandon)
        self.push_front(p.hid, self.dlv_ops(N) + [('end_clear', pid)])

    def a_deadline(self, pid):
        p = self.procs[pid]
        tid = p.lock_ticket
        st, N = self.mcall(self.M.cancel, p.P, tid)
        STATS['deadline_cancel_' + str(st)] += 1
        self.push_front(p.hid, self.dlv_ops(N) + [('wake_self', pid, tid, st)])

    def a_attach(self, hid):
        h = self.homes[hid]
        before = self.M.lending()
        _, N = self.mcall(self.M.attach_home, hid, h.prio, h.port, h.progress)
        h.attached = True
        STATS['attach_late'] += 1
        if self.M.recs:
            STATS['attach_with_lock_state'] += 1
        if not before and self.M.lending():
            STATS['attach_lending_false_to_true'] += 1
            if any(r.inh != INF for r in self.M.recs.values()):
                STATS['attach_changed_inh'] += 1
        self.push_front(hid, self.dlv_ops(N))

    def a_cclear(self, hid):
        # rev 5 (4.15): the host clears the home's key cache: unref_nodes over every entry, on the
        # home's thread; the home's entry in M.cache_refs stays, empty (I9)
        h = self.homes[hid]
        keys = h.cache
        h.cache = ()
        self.mcall(self.M.unref_nodes, hid, keys)
        STATS['cache_cleared'] += 1

    def a_foreign(self, pid, kind, ident):
        # I12: a frame of a restored copy (or any stale id) names another process's hold or
        # ticket; the owner check must leave the manager exactly as it was
        p = self.procs[pid]
        before = self.M.lock_key()
        if kind == 'release':
            N = self.mcall(self.M.release, p.P, ident)
            r = None
        else:
            r, N = self.mcall(self.M.cancel, p.P, ident)
        STATS['foreign_' + kind] += 1
        if N or r is not None or self.M.lock_key() != before:
            self.fail('I12', '%s by %r of %r (not its own) changed the manager: result %r, N %r'
                      % (kind, p.P, ident, r, N))

    def a_destroy(self, hid):
        # 5.7 (rev 4): ~lock_home first closes the port under port.mu; detach_home is a later
        # action of the same thread, so other homes may act (and post) in between
        if any(P[0] == hid for P in self.M.recs):
            STATS['destroy_with_lock_state'] += 1
        self.homes[hid].port.closed = True
        ops = [('detach',)]
        if self.cfg.group_teardown:
            # rev 5b (5.7, L4; design 9.2, td 4.2): a group's teardown closes the port once it
            # holds the group's token, so no pump runs from here on (the home's processes take no
            # action), then kills each process (end(P, abandon) and its delivery: home steps
            # between which other homes act and post), and then ~lock_home finds the port closed
            # and detaches a home with no rec left
            live = [pid for pid in sorted(self.procs)
                    if self.procs[pid].hid == hid and self.procs[pid].status != 'ended']
            ops = [('gkill', pid) for pid in live] + ops
            STATS['group_teardown'] += 1
        self.push_front(hid, ops)

    def op_gkill(self, hid, pid):
        # rev 5b: one kill of a group's teardown, on the group's thread (5.6, as a_kill)
        p = self.procs[pid]
        if p.status == 'ended':
            return
        T = self.M.tickets.get(p.lock_ticket) if p.lock_ticket is not None else None
        if T is not None:
            STATS['group_teardown_kills_waiter_' + T.state] += 1
        self.a_kill(pid, True)

    def op_detach(self, hid):
        # 5.7 (rev 5): `debug: assert H.port is closed`, detach_home's precondition, asserted
        # outside M.mu, after port.close() and before the call; detach_home reads no port
        if not self.homes[hid].port.closed:
            self.fail('5.7', 'detach_home of H%s called with its port open' % hid)
        if any(P[0] == hid for P in self.M.recs):
            STATS['detach_with_lock_state'] += 1
            if self.cfg.group_teardown:
                self.fail('5.7', 'group teardown: detach_home of H%s finds recs its kills left' % hid)
        N = self.mcall(self.M.detach_home, hid)
        self.push_front(hid, self.dlv_ops(N) + [('destroy_clear',)])

    # ------------------------------------------------------------------------------------------
    # invariants
    def wake_in_flight(self, hid, pid, tid):
        post = ('wake', pid, tid)
        if post in self.homes[hid].port.queue:
            return True
        for h in self.homes.values():
            if ('dlv', hid, post) in h.pending or ('dlv_append', hid, post) in h.pending:
                return True
        return False

    def quiescent(self):
        for h in self.homes.values():
            if h.destroyed:
                continue
            if h.pending or h.port.queue:
                return False
        return True

    def check(self):
        M = self.M
        if self._need_mcheck:
            # every manager state change is an mcall, which checks the manager after it
            M.check()
            self._need_mcheck = False
        if M.soft:
            self.soft.extend(M.soft)
            M.soft = []
        # no hold outlives its process
        for i, h in M.holds.items():
            p = self.procs.get(h.pid[1])
            if p is None or p.status == 'ended':
                self.fail('I10', 'hold %d of ended process %r' % (i, h.pid))
        for t, T in M.tickets.items():
            p = self.procs.get(T.pid[1])
            if p.status == 'ended':
                self.fail('I10', 'ticket %d of ended process %r' % (t, T.pid))
        for hid, home in self.homes.items():
            if hid not in M.homes:
                # not yet attached, destroyed, or detached by ~lock_home and about to be cleared
                if hid in M.cache_refs:
                    self.fail('I9', 'detached home H%s still has key-cache references' % hid)
            else:
                # I4 home side: the cache holds exactly the keys of its entry (which exists, I9),
                # one reference per entry; the cache changes in its manager operation's action,
                # so this holds at every action boundary
                d = M.cache_refs[hid]
                if sorted(home.cache) != sorted(d) or any(c != 1 for c in d.values()):
                    self.fail('I4', 'H%s key cache %r != its references %r' % (hid, home.cache, d))
            if home.destroyed or home.pending:
                continue
            for p in self.procs.values():
                if p.hid != hid:
                    continue
                self.check_proc(p)
        # I8 base clause (rev 5: at every action boundary, every home): rec.base == p.priority
        for P, r in M.recs.items():
            p = self.procs.get(P[1])
            if p is not None and p.hid == P[0] and r.base != p.priority:
                self.fail('I8', 'rec %r base %s != p.priority %s' % (P, r.base, p.priority))
        # I9 (rev 5: after every action): once end has been called for every process, or every
        # home is detached, the manager is empty
        if not M.homes or all(p.status == 'ended' for p in self.procs.values()):
            self.check_empty()
        if self.quiescent():
            self.check_quiescent()

    def check_proc(self, p):
        M = self.M
        P = p.P
        rec = M.recs.get(P)
        if p.status == 'ended':
            if rec is not None:
                self.fail('I3', 'ended process %r has a rec' % (P,))
            return
        has = bool(p.locks) or p.lock_ticket is not None
        if has != (rec is not None):
            self.fail('I3', 'process %r: mirror/ticket %s but rec %s' % (P, has, rec is not None))
        if rec is None:
            if p.frames and any(f.kind == 'body' and f.hold is not None for f in p.frames):
                self.fail('mirror', '%r has a lock_body frame with a hold but no rec' % (P,))
        else:
            exp = set(rec.holds)
            if rec.ticket is not None:
                T = M.tickets[rec.ticket]
                if T.state == 'granted':
                    exp.discard(T.hold)
            if set(h.id for h in p.locks) != exp:
                self.fail('I3', 'process %r mirror %r != rec holds less granted %r'
                          % (P, sorted(h.id for h in p.locks), sorted(exp)))
            if rec.ticket is not None:
                if p.lock_ticket != rec.ticket or not p.lock_waiting:
                    self.fail('I2', 'ticket %d of %r exists but the process is not parked on it' % (rec.ticket, P))
                T = M.tickets[rec.ticket]
                if T.state == 'granted' and not self.wake_in_flight(p.hid, p.pid, T.seq):
                    self.fail('I2', 'granted ticket %d of %r with no wake in flight (lost wakeup)' % (T.seq, P))
                if T.state not in ('queued', 'granted'):
                    self.fail('I2', 'ticket %d of %r in state %s between actions' % (T.seq, P, T.state))
        if p.lock_ticket is not None and (rec is None or rec.ticket != p.lock_ticket):
            self.fail('I2', '%r parked on ticket %r the manager does not have' % (P, p.lock_ticket))
        if (p.status == 'parked') != p.lock_waiting:
            self.fail('model', 'status/lock_waiting mismatch for %r' % (P,))
        # mirror counts against frames
        for h in p.locks:
            n = 0
            for f in p.frames:
                if f.kind == 'body' and f.hold == h.id:
                    n += 1
                if f.kind == 'enter' and f.outcome is not None and f.outcome[0] == 'granted' and f.outcome[1] == h.id:
                    n += 1
                    if h.count != 1:
                        self.fail('I2', 'lock_enter frame granted %d with count %d' % (h.id, h.count))
                    mh = M.holds.get(h.id)
                    if mh is None or mh.body_started:
                        self.fail('I2', 'lock_enter frame granted %d whose hold started or is gone' % h.id)
            if n != h.count:
                self.fail('mirror', '%r hold %d count %d but %d frames refer to it (leak)' % (P, h.id, h.count, n))
            mh = M.holds.get(h.id)
            if mh is None or mh.keys != h.keys or mh.mode != h.mode or mh.pid != P:
                self.fail('mirror', '%r mirror hold %d does not match the manager' % (P, h.id))
        for f in p.frames:
            if f.kind == 'body' and f.hold is not None and not any(h.id == f.hold for h in p.locks):
                self.fail('mirror', '%r frame refers to hold %d not in its mirror' % (P, f.hold))
        if p.lock_waiting:
            f = p.frames[-1] if p.frames else None
            if f is None or f.kind != 'enter' or f.outcome is not None:
                self.fail('model', 'parked %r without a pending lock_enter frame' % (P,))

    def check_quiescent(self):
        M = self.M
        for p in self.procs.values():
            if p.status == 'ended':
                continue
            P = p.P
            if p.lock_waiting:
                T = M.tickets.get(p.lock_ticket)
                if T is None or T.state != 'queued':
                    self.fail('I6', 'quiescent, %r parked on a non-queued ticket %r (lost wakeup)' % (P, p.lock_ticket))
            # I8 quiescent clause: a process on a home with applies has read_inh's value (a pure
            # read here: the checker must not clear the coalescing flag as an apply does)
            if M.applies(P):
                v = M.peek_inh(P)
                if p.inherited != v:
                    self.fail('I8', 'quiescent, %r inherited_priority %s != read_inh %s' % (P, p.inherited, v))
            elif p.inherited != INF:
                # protocol 9's L4 addition to the quiescent clause, which holds before moves too
                self.fail('I8', 'quiescent, %r on a home without applies has inherited_priority %s'
                          % (P, p.inherited))
        # 4.13 blocking_sinks: every process a waiter ultimately waits for can run, and comes with
        # its home's progress cell, loaded under M.mu (rev 5: (Pid, time) pairs)
        for P in sorted(M.graph()):
            for Q, t in sorted(M.blocking_sinks(P)):
                q = self.procs[Q[1]]
                hq = self.homes[Q[0]]
                if q.status != 'run' or hq.destroyed or not hq.attached or Q[0] not in M.homes:
                    self.fail('stuck', 'quiescent: blocking_sinks(%r) has %r which is %s' % (P, Q, q.status))
                if t != hq.progress.load():
                    self.fail('4.13', 'blocking_sinks(%r) gives %r time %r, not its home\'s cell %r'
                              % (P, Q, t, hq.progress.load()))
        # stuck: every sink a waiter leads to must be able to run
        g = M.graph()
        for P in g:
            for Q in M.reach_in(g, {P}):
                if g.get(Q):
                    continue
                if Q == P:
                    continue
                q = self.procs[Q[1]]
                if q.status != 'run':
                    self.fail('stuck', 'quiescent: waiter %r leads to sink %r which is %s' % (P, Q, q.status))
        self.check_query()

    def check_empty(self):
        # I9: M.recs, M.holds, M.tickets and M.queue empty, every trie count zero, and the only
        # nodes left those with an abandoned flag or a key-cache reference, and their ancestors
        M = self.M
        if M.recs or M.holds or M.tickets or M.queue:
            self.fail('I9', 'every process ended (or every home detached) but the manager is not empty')
        for k, n in M.trie.items():
            if n.here[S] or n.here[X] or n.wq or n.sub[S] or n.sub[X] or n.subw:
                self.fail('I9', 'trie count left at %s' % (kstr(k) or '<root>'))
            if k == ():
                continue
            cached = any(c[:len(k)] == k for H in M.cache_refs for c in M.cache_refs[H])
            if not any(a[:len(k)] == k for a in M.abandoned) and not cached:
                self.fail('I9', 'trie node %s left that is neither flagged nor cached by an attached home, '
                          'nor an ancestor of one' % kstr(k))

    def check_query(self):
        # 4.13 query (rev 5): counts every hold; shows a pid only for the viewer's home and owner
        M = self.M
        viewers = set((p.hid, p.owner) for p in self.procs.values() if p.hid in M.homes)
        for H, owner in sorted(viewers):
            r = M.query((), (H, owner))
            for e in r['held']:
                hs = [h for h in M.holds.values() if e['key'] in h.keys]
                if not hs or e['count'] != len(hs):
                    self.fail('4.13', 'lock-query count %r for %s, %d holds' % (e['count'], kstr(e['key']), len(hs)))
                # rev 5b: mode X when an X hold has the key; generation the largest gen among the
                # X holds with it (the latest X grant on it, I15), else nil
                xs = [h.gen for h in hs if h.mode == X]
                if e['mode'] != (X if xs else S) or e['generation'] != (max(xs) if xs else None):
                    self.fail('4.13', 'lock-query of %s shows mode %s generation %r; its X holds have '
                              'generations %r' % (kstr(e['key']), e['mode'], e['generation'], sorted(xs)))
                for pid in e['pids']:
                    Q = (H, pid)
                    if not any(h.pid == Q for h in hs) or M.recs[Q].owner != owner:
                        self.fail('4.13', 'lock-query by H%s/%s shows pid %d for %s, not a holder of its '
                                  'home and owner' % (H, owner, pid, kstr(e['key'])))
            for e in r['waiting']:
                if e['pid'] is not None:
                    Q = (H, e['pid'])
                    r_ = M.recs.get(Q)
                    T = M.tickets.get(r_.ticket) if r_ is not None and r_.ticket is not None else None
                    if T is None or T.state != 'queued' or T.keys != e['key'] or r_.owner != owner:
                        self.fail('4.13', 'lock-query by H%s/%s shows waiting pid %d that is not its own'
                                  % (H, owner, e['pid']))

    # ------------------------------------------------------------------------------------------
    def drain(self, limit=10000):
        """A fair completion with no new requests: every body finishes, every post is applied.
        Afterwards no process may be parked (liveness), then every process terminates (I9)."""
        steps = 0
        while steps < limit:
            steps += 1
            act = None
            for hid in sorted(self.homes):
                h = self.homes[hid]
                if not h.destroyed and h.pending:
                    act = ('hstep', hid)
                    break
            if act is None:
                for hid in sorted(self.homes):
                    h = self.homes[hid]
                    if not h.destroyed and h.port.queue:
                        act = ('post', hid)
                        break
            if act is None:
                for pid in sorted(self.procs):
                    p = self.procs[pid]
                    if p.status != 'run':
                        continue
                    base = p.hbase if p.in_handler else 0
                    if len(p.frames) > base:
                        top = p.frames[-1]
                        act = ('enter_step', pid) if top.kind == 'enter' else ('finish', pid)
                        break
                    if p.in_handler:
                        act = ('hend', pid)
                        break
            if act is None:
                break
            self.do(act)
            self.check()
        for p in self.procs.values():
            if p.status == 'parked':
                self.fail('liveness', 'after every body finished, %r is still parked on ticket %r'
                          % (p.P, p.lock_ticket))
        for pid in sorted(self.procs):
            p = self.procs[pid]
            if p.status != 'ended':
                self.do(('kill', pid, False))
                self.check()
                while self.homes[p.hid].pending:
                    self.do(('hstep', p.hid))
                    self.check()
        while not self.quiescent():
            for hid in sorted(self.homes):
                h = self.homes[hid]
                if not h.destroyed and (h.pending or h.port.queue):
                    self.do(('hstep', hid) if h.pending else ('post', hid))
                    self.check()
        self.check()
        M = self.M
        if M.recs or M.holds or M.tickets or M.queue:
            self.fail('I9', 'after every process ended the manager is not empty')
